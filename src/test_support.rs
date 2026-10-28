use std::time::Duration;

use crate::mav::{ConnectionConfig, TelemetryEvent, VehicleId, dialect::*};
use mavlink::{AsyncMavlinkReader, MavHeader, MavlinkVersion};
use ratatui::{Terminal, backend::TestBackend};
use tokio::{
    io::{AsyncReadExt, AsyncWriteExt},
    net::{TcpListener, TcpStream},
    time::timeout,
};

pub(crate) const DEADLINE: Duration = Duration::from_secs(2);

pub(crate) fn terminal() -> Terminal<TestBackend> {
    Terminal::new(TestBackend::new(100, 24)).unwrap()
}

pub(crate) fn screen(terminal: &Terminal<TestBackend>) -> String {
    terminal
        .backend()
        .buffer()
        .content
        .iter()
        .map(|cell| cell.symbol())
        .collect()
}

pub(crate) async fn listener() -> (TcpListener, ConnectionConfig) {
    let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
    let config = ConnectionConfig {
        endpoint: format!("tcpout:{}", listener.local_addr().unwrap()),
        discovery_timeout: Duration::from_millis(100),
        idle_timeout: Duration::from_secs(1),
        ..ConnectionConfig::default()
    };
    (listener, config)
}

pub(crate) async fn heartbeat(peer: &mut TcpStream, system_id: u8, autopilot: MavAutopilot) {
    send(
        peer,
        VehicleId {
            system_id,
            component_id: 1,
        },
        MavMessage::HEARTBEAT(HEARTBEAT_DATA {
            autopilot,
            mavtype: MavType::MAV_TYPE_QUADROTOR,
            mavlink_version: 3,
            ..Default::default()
        }),
    )
    .await;
}

pub(crate) async fn send(peer: &mut TcpStream, source: VehicleId, message: MavMessage) {
    let mut bytes = Vec::new();
    mavlink::write_v2_msg(
        &mut bytes,
        MavHeader {
            system_id: source.system_id,
            component_id: source.component_id,
            sequence: 0,
        },
        &message,
    )
    .unwrap();
    peer.write_all(&bytes).await.unwrap();
}

pub(crate) async fn local_stream_request(peer: &mut TcpStream) -> MavHeader {
    let mut reader = AsyncMavlinkReader::new(peer);
    let (header, command) = timeout(DEADLINE, async {
        loop {
            let (header, message) = reader
                .read_message::<MavMessage>(MavlinkVersion::V2)
                .await
                .unwrap();
            if let MavMessage::COMMAND_LONG(command) = message {
                break (header, command);
            }
        }
    })
    .await
    .expect("client did not request local-position telemetry");
    assert_eq!((command.target_system, command.target_component), (1, 1));
    assert_eq!(command.command, MavCmd::MAV_CMD_SET_MESSAGE_INTERVAL);
    assert_eq!(command.param1, 32.0);
    assert_eq!(command.param2, 200_000.0);
    assert_eq!(command.param7, 0.0);
    header
}

pub(crate) async fn acknowledge_local_stream(peer: &mut TcpStream, result: MavResult) {
    let client = local_stream_request(peer).await;
    send(
        peer,
        autopilot_id(),
        MavMessage::COMMAND_ACK(COMMAND_ACK_DATA {
            command: MavCmd::MAV_CMD_SET_MESSAGE_INTERVAL,
            result,
            target_system: client.system_id,
            target_component: client.component_id,
            ..Default::default()
        }),
    )
    .await;
}

pub(crate) fn autopilot_id() -> VehicleId {
    VehicleId {
        system_id: 1,
        component_id: 1,
    }
}

pub(crate) fn local_position(source: VehicleId, time_boot_ms: u32, z: f32) -> Box<TelemetryEvent> {
    Box::new(TelemetryEvent {
        source,
        message: MavMessage::LOCAL_POSITION_NED(LOCAL_POSITION_NED_DATA {
            time_boot_ms,
            x: 2.0,
            y: 3.0,
            z,
            ..Default::default()
        }),
    })
}

pub(crate) async fn assert_peer_closed(mut peer: TcpStream) {
    let mut bytes = Vec::new();
    timeout(DEADLINE, peer.read_to_end(&mut bytes))
        .await
        .expect("client kept its connection open after shutdown")
        .unwrap();
}
