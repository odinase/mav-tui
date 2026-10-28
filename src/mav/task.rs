use super::mav_data::MavData;
use crate::mav::{
    Connection,
    dialect::{LOCAL_POSITION_NED_DATA, MavCmd, MavMessage},
    select_vehicle,
};
use anyhow::{Context, Result};
use mavlink::MessageData;
use tokio::{select, sync::mpsc};
use tokio_util::sync::CancellationToken;

const LOCAL_POSITION_RATE_HZ: f32 = 5.0;

pub struct MavTask {
    mavcon: Connection,
    tx: mpsc::Sender<MavData>,
    cancel_token: CancellationToken,
}

impl MavTask {
    pub fn new(
        mavcon: Connection,
        tx: mpsc::Sender<MavData>,
        cancel_token: Option<CancellationToken>,
    ) -> Self {
        Self {
            mavcon,
            tx,
            cancel_token: cancel_token.unwrap_or_default(),
        }
    }

    pub async fn start(self) -> Result<()> {
        // Cancellation covers discovery, configuration, streaming and sends to
        // a full display channel. All exits join the connection's background tasks.
        let result = select! {
            biased;
            _ = self.cancel_token.cancelled() => Ok(()),
            result = self.run_session() => result,
        };
        self.mavcon.shutdown().await;
        result
    }

    async fn run_session(&self) -> Result<()> {
        self.tx
            .send(MavData::Discovering)
            .await
            .context("Vehicle display closed")?;
        let vehicles = self
            .mavcon
            .discover()
            .await
            .context("Discover autopilots")?;
        let target = select_vehicle(&vehicles, None, None)
            .context("Altitude plotting requires one autopilot; run one simulator at a time")?
            .id;
        // Subscribe before requesting telemetry: samples may precede its ACK.
        let mut telemetry = self.mavcon.subscribe();
        self.tx
            .send(MavData::Connected(vehicles))
            .await
            .context("Vehicle display closed")?;

        self.mavcon
            .command(
                target,
                MavCmd::MAV_CMD_SET_MESSAGE_INTERVAL,
                [
                    LOCAL_POSITION_NED_DATA::ID as f32,
                    1_000_000.0 / LOCAL_POSITION_RATE_HZ,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                ],
            )
            .await
            .context("Request local position telemetry at 5 Hz")?;

        loop {
            let telemetry_event = telemetry
                .recv()
                .await
                .context("Monitor MAVLink connection")?;
            if telemetry_event.source == target
                && matches!(
                    telemetry_event.message,
                    MavMessage::GLOBAL_POSITION_INT(_) | MavMessage::LOCAL_POSITION_NED(_)
                )
            {
                self.tx
                    .send(MavData::Telemetry(Box::new(telemetry_event)))
                    .await
                    .context("Vehicle display closed")?;
            }
        }
    }
}

#[cfg(test)]
#[path = "task_tests.rs"]
mod tests;
