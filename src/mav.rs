//! One receive loop, a 1 Hz GCS heartbeat, identity-aware discovery and requests.
//!
//! Commands are serialized per connection: MAVLink COMMAND_ACK has no transaction
//! ID. ACKs must come from the selected component and, when present, their MAVLink
//! 2 target extensions must address this client. Zero extensions are accepted for
//! older flight stacks. Use a different client identity for every application.
pub mod mav_data;
pub mod task;

use std::{collections::BTreeMap, fmt, future::Future, sync::Arc, time::Duration};

use anyhow::{Context, Result, anyhow, bail, ensure};
use dialect::{
    AUTOPILOT_VERSION_DATA, COMMAND_ACK_DATA, COMMAND_LONG_DATA, GLOBAL_POSITION_INT_DATA,
    HEARTBEAT_DATA, MavAutopilot, MavCmd, MavMessage, MavModeFlag, MavResult, MavState, MavType,
};
pub use mavlink::dialects::ardupilotmega as dialect;
use mavlink::{
    AsyncMavConnection, MavHeader, MavlinkVersion, MessageData, error::MessageReadError,
};
use tokio::{
    sync::{Mutex, RwLock, broadcast, watch},
    task::JoinHandle,
    time::{Instant, timeout},
};

#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub struct VehicleId {
    pub system_id: u8,
    pub component_id: u8,
}

impl fmt::Display for VehicleId {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}:{}", self.system_id, self.component_id)
    }
}

#[derive(Clone, Debug)]
pub struct ConnectionConfig {
    pub endpoint: String,
    /// QGroundControl normally uses 255:190. The reference client uses 250:190.
    pub client_id: VehicleId,
    pub connect_timeout: Duration,
    /// Observe the full window so a second autopilot cannot be silently ignored.
    pub discovery_timeout: Duration,
    pub request_timeout: Duration,
    /// Includes UDP silence and half-open TCP connections.
    pub idle_timeout: Duration,
}

impl Default for ConnectionConfig {
    fn default() -> Self {
        Self {
            endpoint: "tcpout:127.0.0.1:5760".into(),
            client_id: VehicleId {
                system_id: 250,
                component_id: 190,
            },
            connect_timeout: Duration::from_secs(5),
            discovery_timeout: Duration::from_secs(5),
            request_timeout: Duration::from_secs(5),
            idle_timeout: Duration::from_secs(10),
        }
    }
}

#[derive(Clone, Debug)]
pub struct Vehicle {
    pub id: VehicleId,
    pub autopilot: MavAutopilot,
    pub vehicle_type: MavType,
    pub last_heartbeat: Instant,
}

/// A decoded telemetry event. Its source always includes BOTH MAVLink IDs.
#[derive(Clone, Debug)]
pub struct TelemetryEvent {
    pub source: VehicleId,
    pub message: MavMessage,
}

#[derive(Clone, Copy, Debug)]
pub struct Position {
    pub latitude_deg: f64,
    pub longitude_deg: f64,
    pub altitude_amsl_m: f64,
    pub altitude_relative_m: f64,
    pub heading_deg: Option<f64>,
}

impl From<&GLOBAL_POSITION_INT_DATA> for Position {
    fn from(data: &GLOBAL_POSITION_INT_DATA) -> Self {
        Self {
            latitude_deg: f64::from(data.lat) / 1e7,
            longitude_deg: f64::from(data.lon) / 1e7,
            altitude_amsl_m: f64::from(data.alt) / 1000.,
            altitude_relative_m: f64::from(data.relative_alt) / 1000.,
            heading_deg: (data.hdg != u16::MAX).then(|| f64::from(data.hdg) / 100.),
        }
    }
}

impl Position {
    /// Reject the uninitialized (0,0) fix and impossible wire values.
    pub fn is_valid(&self) -> bool {
        (-90. ..=90.).contains(&self.latitude_deg)
            && (-180. ..=180.).contains(&self.longitude_deg)
            && (self.latitude_deg != 0. || self.longitude_deg != 0.)
            && (-1000. ..=100_000.).contains(&self.altitude_amsl_m)
            && (-100_000. ..=100_000.).contains(&self.altitude_relative_m)
            && self
                .heading_deg
                .is_none_or(|value| (0. ..360.).contains(&value))
    }
}

/// Each subscriber gets its own stream; it never reads from the socket itself.
pub struct Telemetry {
    events: broadcast::Receiver<TelemetryEvent>,
    status: watch::Receiver<Option<String>>,
}

impl Telemetry {
    pub async fn recv(&mut self) -> Result<TelemetryEvent> {
        loop {
            if let Some(reason) = self.status.borrow().as_ref() {
                bail!("MAVLink disconnected: {reason}");
            }
            tokio::select! {
                result = self.events.recv() => return result.context("MAVLink event stream overflowed or closed"),
                result = self.status.changed() => { result.context("MAVLink connection closed")?; }
            }
        }
    }
}

type Transport = Arc<dyn AsyncMavConnection<MavMessage> + Send + Sync>;
type Vehicles = Arc<RwLock<BTreeMap<VehicleId, Vehicle>>>;

pub struct Connection {
    config: ConnectionConfig,
    transport: Transport,
    vehicles: Vehicles,
    events: broadcast::Sender<TelemetryEvent>,
    status: watch::Sender<Option<String>>,
    requests: Mutex<()>,
    receiver: JoinHandle<()>,
    heartbeat: JoinHandle<()>,
}

fn disconnect(status: &watch::Sender<Option<String>>, reason: String) {
    status.send_if_modified(|current| {
        if current.is_some() {
            return false;
        }
        *current = Some(reason);
        true
    });
}

struct PendingRequest<'a> {
    status: &'a watch::Sender<Option<String>>,
    finished: bool,
}

impl Drop for PendingRequest<'_> {
    fn drop(&mut self) {
        if !self.finished {
            disconnect(
                self.status,
                "request cancelled or timed out; reconnect to avoid accepting a late ACK".into(),
            );
        }
    }
}

impl Connection {
    pub async fn connect(config: ConnectionConfig) -> Result<Self> {
        ensure!(
            config.client_id.system_id != 0 && config.client_id.component_id != 0,
            "Client IDs must be nonzero"
        );
        ensure!(
            !config.connect_timeout.is_zero()
                && !config.discovery_timeout.is_zero()
                && !config.request_timeout.is_zero()
                && !config.idle_timeout.is_zero(),
            "Connection timeouts must be positive"
        );
        let mut transport = timeout(config.connect_timeout, async {
            let (kind, address) = config.endpoint.split_once(':').context(
                "Endpoint must be tcpout:host:port, udpout:host:port or udpin:host:port",
            )?;
            ensure!(
                matches!(kind, "tcpout" | "udpout" | "udpin"),
                "Unsupported endpoint type {kind}"
            );
            // Resolve with Tokio BEFORE calling the crate, which otherwise does blocking DNS.
            let address = tokio::net::lookup_host(address)
                .await?
                .next()
                .context("Endpoint hostname has no addresses")?;
            Ok::<_, anyhow::Error>(
                mavlink::connect_async::<MavMessage>(&format!("{kind}:{address}")).await?,
            )
        })
        .await
        .context("MAVLink connect timed out")?
        .with_context(|| {
            format!(
                "Connect to {} (start scripts/sitl up <autopilot>)",
                config.endpoint
            )
        })?;
        transport.set_protocol_version(MavlinkVersion::V2);
        transport.set_allow_recv_any_version(true);
        let transport: Transport = Arc::from(transport);
        let vehicles: Vehicles = Arc::default();
        let (events, _) = broadcast::channel(2048);
        let (status, _) = watch::channel(None);

        let receiver = {
            let transport = transport.clone();
            let vehicles = vehicles.clone();
            let events = events.clone();
            let status = status.clone();
            let mut stopped = status.subscribe();
            let idle_timeout = config.idle_timeout;
            tokio::spawn(async move {
                loop {
                    if stopped.borrow().is_some() {
                        break;
                    }
                    // Only cancellation/shutdown interrupts recv, never a heartbeat tick.
                    let result = tokio::select! {
                        result = timeout(idle_timeout, transport.recv()) => result,
                        _ = stopped.changed() => break,
                    };
                    let (header, message) = match result {
                        Ok(Ok(frame)) => frame,
                        Ok(Err(MessageReadError::Parse(_))) => continue,
                        Ok(Err(error)) => {
                            disconnect(&status, error.to_string());
                            break;
                        }
                        Err(_) => {
                            disconnect(&status, format!("no MAVLink traffic for {idle_timeout:?}"));
                            break;
                        }
                    };
                    let source = VehicleId {
                        system_id: header.system_id,
                        component_id: header.component_id,
                    };
                    if let MavMessage::HEARTBEAT(data) = &message
                        && data.autopilot != MavAutopilot::MAV_AUTOPILOT_INVALID
                        && data.mavtype != MavType::MAV_TYPE_GCS
                        && source.system_id != 0
                        && source.component_id != 0
                    {
                        vehicles.write().await.insert(
                            source,
                            Vehicle {
                                id: source,
                                autopilot: data.autopilot,
                                vehicle_type: data.mavtype,
                                last_heartbeat: Instant::now(),
                            },
                        );
                    }
                    let _ = events.send(TelemetryEvent { source, message });
                }
            })
        };

        let heartbeat = {
            let transport = transport.clone();
            let status = status.clone();
            let mut stopped = status.subscribe();
            let client = config.client_id;
            let send_timeout = config.request_timeout;
            tokio::spawn(async move {
                let mut interval = tokio::time::interval(Duration::from_secs(1));
                interval.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
                let message = MavMessage::HEARTBEAT(HEARTBEAT_DATA {
                    mavtype: MavType::MAV_TYPE_GCS,
                    autopilot: MavAutopilot::MAV_AUTOPILOT_INVALID,
                    base_mode: MavModeFlag::empty(),
                    custom_mode: 0,
                    system_status: MavState::MAV_STATE_ACTIVE,
                    mavlink_version: 3,
                });
                let header = MavHeader {
                    system_id: client.system_id,
                    component_id: client.component_id,
                    sequence: 0,
                };
                loop {
                    if stopped.borrow().is_some() {
                        break;
                    }
                    tokio::select! {
                        _ = stopped.changed() => break,
                        _ = interval.tick() => {
                            match timeout(send_timeout, transport.send(&header, &message)).await {
                                Ok(Ok(_)) => {},
                                Ok(Err(error)) => { disconnect(&status, error.to_string()); break; },
                                Err(_) => { disconnect(&status, "heartbeat send timed out".into()); break; }
                            }
                        }
                    }
                }
            })
        };

        Ok(Self {
            config,
            transport,
            vehicles,
            events,
            status,
            requests: Mutex::new(()),
            receiver,
            heartbeat,
        })
    }

    pub fn subscribe(&self) -> Telemetry {
        Telemetry {
            events: self.events.subscribe(),
            status: self.status.subscribe(),
        }
    }

    pub async fn discover(&self) -> Result<Vec<Vehicle>> {
        let mut status = self.status.subscribe();
        if let Some(reason) = status.borrow().as_ref() {
            bail!("MAVLink disconnected: {reason}");
        }
        tokio::select! {
            _ = tokio::time::sleep(self.config.discovery_timeout) => {},
            _ = status.changed() => bail!("MAVLink disconnected during discovery: {:?}", *status.borrow()),
        }
        let vehicles = self
            .vehicles
            .read()
            .await
            .values()
            .filter(|vehicle| vehicle.last_heartbeat.elapsed() <= self.config.idle_timeout)
            .cloned()
            .collect::<Vec<_>>();
        ensure!(
            !vehicles.is_empty(),
            "No autopilot heartbeat within {:?} on {}",
            self.config.discovery_timeout,
            self.config.endpoint
        );
        ensure!(
            vehicles
                .iter()
                .all(|vehicle| vehicle.id.system_id != self.config.client_id.system_id),
            "Client system ID {} collides with an autopilot; choose another client identity",
            self.config.client_id.system_id
        );
        Ok(vehicles)
    }

    async fn send_command(
        &self,
        target: VehicleId,
        command: MavCmd,
        params: [f32; 7],
    ) -> Result<()> {
        ensure!(
            target.system_id != 0 && target.component_id != 0,
            "Requests need an explicit, nonzero vehicle/component target"
        );
        ensure!(
            target.system_id != self.config.client_id.system_id,
            "Client and vehicle system IDs must differ"
        );
        if let Some(reason) = self.status.borrow().as_ref() {
            bail!("MAVLink disconnected: {reason}");
        }
        let message = MavMessage::COMMAND_LONG(COMMAND_LONG_DATA {
            target_system: target.system_id,
            target_component: target.component_id,
            command,
            confirmation: 0,
            param1: params[0],
            param2: params[1],
            param3: params[2],
            param4: params[3],
            param5: params[4],
            param6: params[5],
            param7: params[6],
        });
        let client = self.config.client_id;
        self.transport
            .send(
                &MavHeader {
                    system_id: client.system_id,
                    component_id: client.component_id,
                    sequence: 0,
                },
                &message,
            )
            .await?;
        Ok(())
    }

    fn matching_ack(
        &self,
        event: &TelemetryEvent,
        target: VehicleId,
        command: MavCmd,
    ) -> Result<Option<COMMAND_ACK_DATA>> {
        let MavMessage::COMMAND_ACK(ack) = &event.message else {
            return Ok(None);
        };
        let client = self.config.client_id;
        if event.source != target
            || ack.command != command
            || (ack.target_system != 0 && ack.target_system != client.system_id)
            || (ack.target_component != 0 && ack.target_component != client.component_id)
        {
            return Ok(None);
        }
        match ack.result {
            MavResult::MAV_RESULT_ACCEPTED => Ok(Some(ack.clone())),
            MavResult::MAV_RESULT_IN_PROGRESS => Ok(None),
            result => bail!(
                "{target} rejected {command:?}: {result:?} (detail {})",
                ack.result_param2
            ),
        }
    }

    async fn exchange<T>(
        &self,
        context: String,
        request: impl Future<Output = Result<T>>,
    ) -> Result<T> {
        timeout(self.config.request_timeout, async {
            let _lock = self.requests.lock().await;
            let mut pending = PendingRequest {
                status: &self.status,
                finished: false,
            };
            let result = request.await;
            pending.finished = true;
            result
        })
        .await
        .context(context)?
    }

    /// Send a targeted command and await a final ACK, with one bounded deadline.
    /// Cancelling/timing out an in-flight exchange closes the client session:
    /// COMMAND_ACK has no transaction ID to distinguish a late reply on retry.
    pub async fn command(
        &self,
        target: VehicleId,
        command: MavCmd,
        params: [f32; 7],
    ) -> Result<COMMAND_ACK_DATA> {
        self.exchange(
            format!("Timed out waiting for {command:?} ACK from {target}"),
            async {
                let mut events = self.subscribe();
                self.send_command(target, command, params).await?;
                loop {
                    if let Some(ack) = self.matching_ack(&events.recv().await?, target, command)? {
                        return Ok(ack);
                    }
                }
            },
        )
        .await
    }

    /// Validate both the REQUEST_MESSAGE ACK and a fresh AUTOPILOT_VERSION reply.
    /// Replies may precede the ACK; other vehicles and QGC's ACKs are ignored.
    pub async fn request_autopilot_version(
        &self,
        target: VehicleId,
    ) -> Result<AUTOPILOT_VERSION_DATA> {
        let command = MavCmd::MAV_CMD_REQUEST_MESSAGE;
        self.exchange(
            format!("Timed out waiting for AUTOPILOT_VERSION and ACK from {target}"),
            async {
                let mut events = self.subscribe();
                let message_id = AUTOPILOT_VERSION_DATA::ID;
                self.send_command(target, command, [message_id as f32, 0., 0., 0., 0., 0., 1.])
                    .await?;
                let mut acknowledged = false;
                let mut version = None;
                loop {
                    let event = events.recv().await?;
                    acknowledged |= self.matching_ack(&event, target, command)?.is_some();
                    if event.source == target
                        && let MavMessage::AUTOPILOT_VERSION(data) = event.message
                    {
                        ensure!(
                            data.flight_sw_version != 0,
                            "{target} returned an empty firmware version"
                        );
                        version = Some(data);
                    }
                    if acknowledged && let Some(version) = version.take() {
                        return Ok(version);
                    }
                }
            },
        )
        .await
    }

    /// Enable GLOBAL_POSITION_INT and validate its ACK and a fresh position fix.
    pub async fn request_position(&self, target: VehicleId, rate_hz: f32) -> Result<Position> {
        ensure!(
            rate_hz.is_finite() && rate_hz > 0. && rate_hz <= 100.,
            "Position rate must be in (0,100] Hz"
        );
        let command = MavCmd::MAV_CMD_SET_MESSAGE_INTERVAL;
        self.exchange(
            format!("Timed out waiting for valid position and ACK from {target}"),
            async {
                let mut events = self.subscribe();
                self.send_command(
                    target,
                    command,
                    [
                        GLOBAL_POSITION_INT_DATA::ID as f32,
                        1_000_000. / rate_hz,
                        0.,
                        0.,
                        0.,
                        0.,
                        // PX4 supports only flight-stack default response targeting.
                        0.,
                    ],
                )
                .await?;
                let mut acknowledged = false;
                let mut position = None;
                loop {
                    let event = events.recv().await?;
                    acknowledged |= self.matching_ack(&event, target, command)?.is_some();
                    if event.source == target
                        && let MavMessage::GLOBAL_POSITION_INT(data) = event.message
                    {
                        let candidate = Position::from(&data);
                        if candidate.is_valid() {
                            position = Some(candidate);
                        }
                    }
                    if acknowledged && let Some(position) = position.take() {
                        return Ok(position);
                    }
                }
            },
        )
        .await
    }

    /// Abort and join background tasks; dropping also aborts them.
    pub async fn shutdown(mut self) {
        disconnect(&self.status, "client shut down".into());
        self.receiver.abort();
        self.heartbeat.abort();
        let _ = (&mut self.receiver).await;
        let _ = (&mut self.heartbeat).await;
    }
}

impl Drop for Connection {
    fn drop(&mut self) {
        disconnect(&self.status, "client dropped".into());
        self.receiver.abort();
        self.heartbeat.abort();
    }
}

/// A system filter is sufficient for normal autopilots; a component filter can
/// disambiguate multiple autopilot components on the same system.
pub fn select_vehicle(
    vehicles: &[Vehicle],
    system: Option<u8>,
    component: Option<u8>,
) -> Result<Vehicle> {
    let matches = vehicles
        .iter()
        .filter(|vehicle| {
            system.is_none_or(|id| vehicle.id.system_id == id)
                && component.is_none_or(|id| vehicle.id.component_id == id)
        })
        .collect::<Vec<_>>();
    let available = vehicles
        .iter()
        .map(|vehicle| format!("{} ({:?})", vehicle.id, vehicle.autopilot))
        .collect::<Vec<_>>()
        .join(", ");
    match matches.as_slice() {
        [vehicle] => Ok((*vehicle).clone()),
        [] => Err(anyhow!(
            "Requested autopilot not found; discovered: {available}"
        )),
        _ => Err(anyhow!(
            "Multiple autopilots discovered: {available}; select --target-system (and --target-component if needed)"
        )),
    }
}
