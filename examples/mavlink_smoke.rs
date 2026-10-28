use std::time::Duration;

use anyhow::{Context, Result, bail};
use mav_tui::mav::{
    Connection, ConnectionConfig, Position,
    dialect::{
        EXTENDED_SYS_STATE_DATA, MISSION_CURRENT_DATA, MavAutopilot, MavCmd, MavMessage,
        MavModeFlag,
    },
    select_vehicle,
};
use mavlink::MessageData;

#[derive(Default)]
struct Options {
    config: ConnectionConfig,
    watch: bool,
    system: Option<u8>,
    component: Option<u8>,
}

fn options() -> Result<Option<Options>> {
    let mut options = Options::default();
    let mut args = std::env::args().skip(1);
    while let Some(arg) = args.next() {
        if arg == "--watch" {
            options.watch = true;
            continue;
        }
        if arg == "--help" || arg == "-h" {
            println!(
                "mavlink_smoke [--watch] [--endpoint tcpout:127.0.0.1:5760]\n  [--target-system ID] [--target-component ID]\n  [--client-system ID] [--client-component ID] [--discovery-seconds N]"
            );
            return Ok(None);
        }
        let value = args
            .next()
            .with_context(|| format!("Missing value for {arg}"))?;
        match arg.as_str() {
            "--endpoint" => options.config.endpoint = value,
            "--target-system" => {
                options.system = Some(value.parse().context("Invalid target system ID")?)
            }
            "--target-component" => {
                options.component = Some(value.parse().context("Invalid target component ID")?)
            }
            "--client-system" => options.config.client_id.system_id = value.parse()?,
            "--client-component" => options.config.client_id.component_id = value.parse()?,
            "--discovery-seconds" => {
                options.config.discovery_timeout = Duration::from_secs(value.parse()?)
            }
            _ => bail!("Unknown option {arg}; see --help"),
        }
    }
    Ok(Some(options))
}

async fn run(connection: &Connection, options: &Options) -> Result<()> {
    let vehicles = connection.discover().await?;
    for vehicle in &vehicles {
        println!(
            "Discovered {} {:?} {:?}",
            vehicle.id, vehicle.autopilot, vehicle.vehicle_type
        );
    }
    let vehicle = select_vehicle(&vehicles, options.system, options.component)?;
    let version = connection.request_autopilot_version(vehicle.id).await?;
    let firmware = version.flight_sw_version;
    println!(
        "{} firmware {}.{}.{} (type {})",
        vehicle.id,
        firmware >> 24,
        (firmware >> 16) & 255,
        (firmware >> 8) & 255,
        firmware & 255
    );
    let position = connection.request_position(vehicle.id, 5.).await?;
    println!("Position verified: {position:?}");
    println!("Smoke checks passed for {}", vehicle.id);
    if !options.watch {
        return Ok(());
    }
    let mut streams = vec![(EXTENDED_SYS_STATE_DATA::ID, 500_000.)];
    if vehicle.autopilot == MavAutopilot::MAV_AUTOPILOT_ARDUPILOTMEGA {
        streams.push((MISSION_CURRENT_DATA::ID, 1_000_000.));
    }
    // PX4's mission manager emits MISSION_CURRENT automatically, outside its
    // configurable named streams. ArduPilot requires an interval request.
    for (message_id, interval_us) in streams {
        connection
            .command(
                vehicle.id,
                MavCmd::MAV_CMD_SET_MESSAGE_INTERVAL,
                [message_id as f32, interval_us, 0., 0., 0., 0., 0.],
            )
            .await
            .with_context(|| format!("Enable watch stream {message_id} for {}", vehicle.id))?;
    }
    println!(
        "Watching {} — operate the mission in QGC; Ctrl-C exits",
        vehicle.id
    );
    let mut telemetry = connection.subscribe();
    loop {
        let event = telemetry.recv().await?;
        if event.source != vehicle.id {
            continue;
        }
        match event.message {
            MavMessage::GLOBAL_POSITION_INT(data) => {
                let p = Position::from(&data);
                println!(
                    "{} position {:.7},{:.7} altitude {:.1} m above home / {:.1} m AMSL heading {:?}",
                    vehicle.id,
                    p.latitude_deg,
                    p.longitude_deg,
                    p.altitude_relative_m,
                    p.altitude_amsl_m,
                    p.heading_deg
                );
            }
            MavMessage::HEARTBEAT(data) => println!(
                "{} armed={} state={:?} custom_mode={}",
                vehicle.id,
                data.base_mode
                    .contains(MavModeFlag::MAV_MODE_FLAG_SAFETY_ARMED),
                data.system_status,
                data.custom_mode
            ),
            MavMessage::MISSION_CURRENT(data) => println!(
                "{} mission item={} total={} state={:?}",
                vehicle.id, data.seq, data.total, data.mission_state
            ),
            MavMessage::MISSION_ITEM_REACHED(data) => {
                println!("{} reached mission item {}", vehicle.id, data.seq)
            }
            MavMessage::EXTENDED_SYS_STATE(data) => {
                println!("{} landed={:?}", vehicle.id, data.landed_state)
            }
            MavMessage::STATUSTEXT(data) => println!(
                "{} {:?}: {}",
                vehicle.id,
                data.severity,
                String::from_utf8_lossy(data.text.as_ref()).trim_end_matches('\0')
            ),
            _ => {}
        }
    }
}

#[tokio::main]
async fn main() -> Result<()> {
    let Some(options) = options()? else {
        return Ok(());
    };
    println!(
        "Connecting to {} as {}",
        options.config.endpoint, options.config.client_id
    );
    let connection = Connection::connect(options.config.clone()).await?;
    let result = tokio::select! {
        result = run(&connection, &options) => result,
        signal = tokio::signal::ctrl_c() => signal.context("Ctrl-C handler"),
    };
    connection.shutdown().await;
    result
}
