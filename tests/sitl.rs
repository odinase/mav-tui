use std::time::Duration;

use mav_tui::mav::{Connection, ConnectionConfig, select_vehicle};

/// Start either `scripts/sitl up ardupilot` or `scripts/sitl up px4` first.
/// Run once with QGC closed and again with QGC's TCP link connected.
#[tokio::test]
#[ignore = "requires a running Docker SITL and router on localhost:5760"]
async fn discover_firmware_ack_and_position_through_router() -> anyhow::Result<()> {
    let connection = Connection::connect(ConnectionConfig {
        endpoint: std::env::var("MAVLINK_ENDPOINT")
            .unwrap_or_else(|_| "tcpout:127.0.0.1:5760".into()),
        discovery_timeout: Duration::from_secs(15),
        request_timeout: Duration::from_secs(15),
        idle_timeout: Duration::from_secs(30),
        ..ConnectionConfig::default()
    })
    .await?;
    let result = async {
        let vehicles = connection.discover().await?;
        let system = std::env::var("MAVLINK_TARGET_SYSTEM")
            .ok()
            .map(|value| value.parse())
            .transpose()?;
        let vehicle = select_vehicle(&vehicles, system, None)?;
        println!("Discovered {} {:?}", vehicle.id, vehicle.autopilot);
        let version = connection.request_autopilot_version(vehicle.id).await?;
        println!(
            "Firmware response and REQUEST_MESSAGE ACK verified: {:#x}",
            version.flight_sw_version
        );
        let position = connection.request_position(vehicle.id, 5.).await?;
        println!("Position and SET_MESSAGE_INTERVAL ACK verified: {position:?}");
        anyhow::ensure!(position.is_valid(), "Invalid global position");
        anyhow::ensure!(
            (position.latitude_deg - 47.397742).abs() < 0.001,
            "Unexpected simulation latitude"
        );
        anyhow::ensure!(
            (position.longitude_deg - 8.545594).abs() < 0.001,
            "Unexpected simulation longitude"
        );
        anyhow::ensure!(
            (position.altitude_amsl_m - 488.).abs() < 30.,
            "Unexpected simulation altitude"
        );
        Ok(())
    }
    .await;
    connection.shutdown().await;
    result
}
