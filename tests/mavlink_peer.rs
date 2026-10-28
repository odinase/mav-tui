//! Exercise real MAVLink 2 framing over localhost, without Docker or a flight stack.
use std::time::Duration;

use mav_tui::mav::{Connection, ConnectionConfig, VehicleId, dialect::*, select_vehicle};
use mavlink::{AsyncMavlinkReader, MavHeader, MavlinkVersion};
use tokio::{
    io::AsyncWriteExt,
    net::{
        TcpListener,
        tcp::{OwnedReadHalf, OwnedWriteHalf},
    },
    time::{Instant, timeout},
};

const AP: VehicleId = VehicleId {
    system_id: 1,
    component_id: 1,
};
const PX: VehicleId = VehicleId {
    system_id: 2,
    component_id: 1,
};

struct Peer {
    reader: AsyncMavlinkReader<OwnedReadHalf>,
    writer: OwnedWriteHalf,
    sequence: u8,
}

async fn pair(request_timeout: Duration) -> (Connection, Peer) {
    let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
    let config = ConnectionConfig {
        endpoint: format!("tcpout:{}", listener.local_addr().unwrap()),
        discovery_timeout: Duration::from_millis(100),
        request_timeout,
        ..ConnectionConfig::default()
    };
    let connection = Connection::connect(config).await.unwrap();
    let (stream, _) = listener.accept().await.unwrap();
    let (reader, writer) = stream.into_split();
    (
        connection,
        Peer {
            reader: AsyncMavlinkReader::new(reader),
            writer,
            sequence: 0,
        },
    )
}

impl Peer {
    async fn send(&mut self, source: VehicleId, message: MavMessage) {
        let mut bytes = Vec::new();
        mavlink::write_v2_msg(
            &mut bytes,
            MavHeader {
                system_id: source.system_id,
                component_id: source.component_id,
                sequence: self.sequence,
            },
            &message,
        )
        .unwrap();
        self.sequence = self.sequence.wrapping_add(1);
        self.writer.write_all(&bytes).await.unwrap();
    }

    async fn recv(&mut self) -> (MavHeader, MavMessage) {
        timeout(
            Duration::from_secs(2),
            self.reader.read_message::<MavMessage>(MavlinkVersion::V2),
        )
        .await
        .unwrap()
        .unwrap()
    }

    async fn command(&mut self) -> (MavHeader, COMMAND_LONG_DATA) {
        loop {
            let (header, message) = self.recv().await;
            if let MavMessage::COMMAND_LONG(command) = message {
                return (header, command);
            }
        }
    }

    async fn heartbeat(&mut self, source: VehicleId, autopilot: MavAutopilot) {
        self.send(
            source,
            MavMessage::HEARTBEAT(HEARTBEAT_DATA {
                autopilot,
                mavtype: MavType::MAV_TYPE_QUADROTOR,
                mavlink_version: 3,
                ..Default::default()
            }),
        )
        .await;
    }

    async fn ack(
        &mut self,
        source: VehicleId,
        command: MavCmd,
        result: MavResult,
        client: VehicleId,
    ) {
        self.send(
            source,
            MavMessage::COMMAND_ACK(COMMAND_ACK_DATA {
                command,
                result,
                target_system: client.system_id,
                target_component: client.component_id,
                ..Default::default()
            }),
        )
        .await;
    }
}

fn version(value: u32) -> MavMessage {
    MavMessage::AUTOPILOT_VERSION(AUTOPILOT_VERSION_DATA {
        flight_sw_version: value,
        ..Default::default()
    })
}

fn position(lat: i32) -> MavMessage {
    MavMessage::GLOBAL_POSITION_INT(GLOBAL_POSITION_INT_DATA {
        lat,
        lon: if lat == 0 { 0 } else { 85_455_940 },
        alt: 488_000,
        relative_alt: 10_000,
        ..Default::default()
    })
}

#[tokio::test]
async fn mixed_stream_requires_target_and_matches_source_command_and_recipient() {
    let (connection, mut peer) = pair(Duration::from_secs(2)).await;
    peer.heartbeat(AP, MavAutopilot::MAV_AUTOPILOT_ARDUPILOTMEGA)
        .await;
    peer.heartbeat(PX, MavAutopilot::MAV_AUTOPILOT_PX4).await;
    peer.heartbeat(
        VehicleId {
            system_id: 255,
            component_id: 190,
        },
        MavAutopilot::MAV_AUTOPILOT_INVALID,
    )
    .await;
    peer.send(AP, MavMessage::AHRS(AHRS_DATA::default())).await;
    let vehicles = connection.discover().await.unwrap();
    assert_eq!(vehicles.len(), 2);
    assert!(
        select_vehicle(&vehicles, None, None)
            .unwrap_err()
            .to_string()
            .contains("Multiple autopilots")
    );
    assert!(select_vehicle(&vehicles, Some(3), None).is_err());
    assert_eq!(
        select_vehicle(&vehicles, Some(2), None).unwrap().autopilot,
        MavAutopilot::MAV_AUTOPILOT_PX4
    );

    let mut request = Box::pin(connection.request_autopilot_version(PX));
    let (client_header, command) = tokio::select! {
        value = peer.command() => value,
        result = &mut request => panic!("request returned early: {result:?}"),
    };
    assert_eq!((command.target_system, command.target_component), (2, 1));
    assert_eq!(command.param1, 148.);
    let client = VehicleId {
        system_id: client_header.system_id,
        component_id: client_header.component_id,
    };
    assert_eq!(
        client,
        VehicleId {
            system_id: 250,
            component_id: 190
        }
    );
    peer.send(AP, version(0x040702ff)).await;
    peer.ack(AP, command.command, MavResult::MAV_RESULT_ACCEPTED, client)
        .await;
    peer.ack(
        VehicleId {
            system_id: 2,
            component_id: 42,
        },
        command.command,
        MavResult::MAV_RESULT_ACCEPTED,
        client,
    )
    .await;
    peer.ack(
        PX,
        MavCmd::MAV_CMD_SET_MESSAGE_INTERVAL,
        MavResult::MAV_RESULT_ACCEPTED,
        client,
    )
    .await;
    peer.ack(
        PX,
        command.command,
        MavResult::MAV_RESULT_DENIED,
        VehicleId {
            system_id: 255,
            component_id: 190,
        },
    )
    .await;
    peer.ack(
        PX,
        command.command,
        MavResult::MAV_RESULT_DENIED,
        VehicleId {
            system_id: 250,
            component_id: 191,
        },
    )
    .await;
    // The firmware reply can arrive BEFORE the final ACK.
    peer.send(PX, version(0x011100ff)).await;
    peer.ack(
        PX,
        command.command,
        MavResult::MAV_RESULT_IN_PROGRESS,
        client,
    )
    .await;
    assert!(
        timeout(Duration::from_millis(40), &mut request)
            .await
            .is_err()
    );
    peer.ack(PX, command.command, MavResult::MAV_RESULT_ACCEPTED, client)
        .await;
    assert_eq!(request.await.unwrap().flight_sw_version, 0x011100ff);

    let mut request = Box::pin(connection.request_position(PX, 5.));
    let (_, command) = tokio::select! {
        value = peer.command() => value,
        result = &mut request => panic!("request returned early: {result:?}"),
    };
    assert_eq!(command.param1, 33.);
    assert_eq!(command.param2, 200_000.);
    assert_eq!(command.param7, 0.); // PX4 rejects a non-default response target.
    // Also support legacy zeroed ACK extension fields.
    peer.ack(
        PX,
        command.command,
        MavResult::MAV_RESULT_ACCEPTED,
        VehicleId {
            system_id: 0,
            component_id: 0,
        },
    )
    .await;
    peer.send(AP, position(473_977_420)).await;
    peer.send(PX, position(0)).await;
    assert!(
        timeout(Duration::from_millis(40), &mut request)
            .await
            .is_err()
    );
    peer.send(PX, position(473_977_420)).await;
    let position = request.await.unwrap();
    assert!(position.is_valid());
    assert_eq!(position.altitude_relative_m, 10.);
    connection.shutdown().await;
}

#[tokio::test]
async fn response_without_ack_times_out() {
    let (connection, mut peer) = pair(Duration::from_millis(150)).await;
    let mut request = Box::pin(connection.request_autopilot_version(AP));
    tokio::select! {
        _ = peer.command() => {},
        result = &mut request => panic!("request returned early: {result:?}"),
    }
    peer.send(AP, version(0x040702ff)).await;
    let error = request.await.unwrap_err();
    assert!(
        error
            .to_string()
            .contains("Timed out waiting for AUTOPILOT_VERSION"),
        "{error:#}"
    );
    assert!(
        connection
            .request_autopilot_version(AP)
            .await
            .unwrap_err()
            .to_string()
            .contains("disconnected")
    );
    connection.shutdown().await;
}

#[tokio::test]
async fn cancelling_an_inflight_request_prevents_a_late_ack_from_matching_a_retry() {
    let (connection, mut peer) = pair(Duration::from_secs(2)).await;
    let mut request = Box::pin(connection.request_autopilot_version(AP));
    tokio::select! {
        _ = peer.command() => {},
        result = &mut request => panic!("request returned early: {result:?}"),
    }
    drop(request);
    let error = connection.request_autopilot_version(AP).await.unwrap_err();
    assert!(error.to_string().contains("disconnected"), "{error:#}");
    connection.shutdown().await;
}

#[tokio::test]
async fn rejected_command_returns_immediately() {
    let (connection, mut peer) = pair(Duration::from_secs(2)).await;
    let mut request = Box::pin(connection.command(
        AP,
        MavCmd::MAV_CMD_REQUEST_MESSAGE,
        [148., 0., 0., 0., 0., 0., 1.],
    ));
    let (header, command) = tokio::select! {
        value = peer.command() => value,
        result = &mut request => panic!("request returned early: {result:?}"),
    };
    peer.ack(
        AP,
        command.command,
        MavResult::MAV_RESULT_UNSUPPORTED,
        VehicleId {
            system_id: header.system_id,
            component_id: header.component_id,
        },
    )
    .await;
    assert!(
        request
            .await
            .unwrap_err()
            .to_string()
            .contains("MAV_RESULT_UNSUPPORTED")
    );
    connection.shutdown().await;
}

#[tokio::test]
async fn disconnect_interrupts_request_and_telemetry() {
    let (connection, mut peer) = pair(Duration::from_secs(2)).await;
    let mut telemetry = connection.subscribe();
    let mut request = Box::pin(connection.request_autopilot_version(AP));
    tokio::select! {
        _ = peer.command() => {},
        result = &mut request => panic!("request returned early: {result:?}"),
    }
    drop(peer);
    let error = timeout(Duration::from_millis(500), request)
        .await
        .unwrap()
        .unwrap_err();
    assert!(error.to_string().contains("disconnected"), "{error:#}");
    assert!(telemetry.recv().await.is_err());
    connection.shutdown().await;
}

#[tokio::test]
async fn no_heartbeat_discovery_is_bounded() {
    let (connection, _peer) = pair(Duration::from_secs(1)).await;
    assert!(
        connection
            .discover()
            .await
            .unwrap_err()
            .to_string()
            .contains("No autopilot heartbeat")
    );
    connection.shutdown().await;
}

#[tokio::test]
async fn gcs_heartbeat_uses_configured_identity_at_one_hz_and_stops_on_shutdown() {
    let (connection, mut peer) = pair(Duration::from_secs(2)).await;
    let (header, message) = peer.recv().await;
    assert_eq!((header.system_id, header.component_id), (250, 190));
    let MavMessage::HEARTBEAT(data) = message else {
        panic!("expected heartbeat")
    };
    assert_eq!(data.mavtype, MavType::MAV_TYPE_GCS);
    let start = Instant::now();
    assert!(matches!(peer.recv().await.1, MavMessage::HEARTBEAT(_)));
    assert!(start.elapsed() >= Duration::from_millis(800));
    connection.shutdown().await;
    assert!(
        timeout(
            Duration::from_millis(500),
            peer.reader.read_any_message::<MavMessage>()
        )
        .await
        .unwrap()
        .is_err()
    );
}

#[tokio::test]
async fn silent_tcp_connection_and_udp_connection_fail_with_idle_timeout() {
    for kind in ["tcpout", "udpin"] {
        let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
        let endpoint = if kind == "tcpout" {
            format!("tcpout:{}", listener.local_addr().unwrap())
        } else {
            "udpin:127.0.0.1:0".into()
        };
        let connection = Connection::connect(ConnectionConfig {
            endpoint,
            idle_timeout: Duration::from_millis(100),
            ..ConnectionConfig::default()
        })
        .await
        .unwrap();
        let _peer = if kind == "tcpout" {
            Some(listener.accept().await.unwrap().0)
        } else {
            None
        };
        let mut telemetry = connection.subscribe();
        let error = timeout(Duration::from_millis(500), telemetry.recv())
            .await
            .unwrap()
            .unwrap_err();
        assert!(
            error.to_string().contains("no MAVLink traffic"),
            "{error:#}"
        );
        connection.shutdown().await;
    }
}
