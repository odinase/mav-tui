use std::{
    collections::{HashMap, VecDeque},
    time::Duration,
};

use anyhow::{Result, bail};
use mavlink::Message;
use ratatui::{
    Frame, Terminal,
    backend::Backend,
    layout::{Constraint, Layout},
    prelude::{Color, Line, Span},
    style::Stylize,
    symbols::Marker,
    widgets::{Axis, Bar, BarChart, Block, Chart, Dataset, GraphType, Paragraph, Wrap},
};
use tokio::{
    select,
    sync::mpsc,
    time::{MissedTickBehavior, interval},
};
use tokio_util::sync::CancellationToken;

use tokio::time::Instant;

use crate::{
    event::event_data::EventData,
    mav::{
        Position, Vehicle, VehicleId,
        dialect::{LOCAL_POSITION_NED_DATA, MavMessage},
        mav_data::MavData,
    },
};

const REDRAW_INTERVAL: Duration = Duration::from_millis(100);
const HISTORY_WINDOW_MS: u32 = 30_000;
const MAX_LOCAL_SAMPLES: usize = 3_000;

pub struct TuiTask {
    mav_rx: mpsc::Receiver<MavData>,
    event_rx: mpsc::Receiver<EventData>,
    endpoint: String,
    cancel_token: CancellationToken,
    app: TuiApp,
}

impl TuiTask {
    pub fn new(
        mav_rx: mpsc::Receiver<MavData>,
        event_rx: mpsc::Receiver<EventData>,
        endpoint: impl Into<String>,
        cancel_token: CancellationToken,
    ) -> Self {
        Self {
            mav_rx,
            event_rx,
            endpoint: endpoint.into(),
            cancel_token,
            app: TuiApp::default(),
        }
    }

    pub async fn start<B>(mut self, terminal: &mut Terminal<B>) -> Result<()>
    where
        B: Backend,
        B::Error: Send + Sync + 'static,
    {
        terminal.draw(|frame| self.app.render(frame, &self.endpoint))?;
        let mut redraw = interval(REDRAW_INTERVAL);
        redraw.set_missed_tick_behavior(MissedTickBehavior::Skip);

        loop {
            select! {
                biased;
                _ = self.cancel_token.cancelled() => return Ok(()),
                event = self.event_rx.recv() => match event {
                    Some(EventData::Abort) => {
                        self.cancel_token.cancel();
                        return Ok(());
                    },
                    Some(EventData::MenuToRight) => {
                        self.app.mode = match self.app.mode {
                            UiMode::Plotting => UiMode::MessageRate,
                            UiMode::MessageRate => UiMode::Plotting,
                        };
                    },
                    Some(EventData::MenuToLeft) => {
                        self.app.mode = match self.app.mode {
                            UiMode::Plotting => UiMode::MessageRate,
                            UiMode::MessageRate => UiMode::Plotting,
                        };
                    },
                    None => bail!("Terminal event channel closed unexpectedly"),
                },
                _ = redraw.tick() => {
                    terminal.draw(|frame| self.app.render(frame, &self.endpoint))?;
                },
                message = self.mav_rx.recv() => match message {
                    Some(message) => self.app.update(message),
                    None => bail!("Vehicle update channel closed unexpectedly"),
                },
            }
        }
    }
}

#[derive(Default)]
enum Status {
    #[default]
    Connecting,
    Discovering,
    Connected,
}

#[derive(Debug, Default, Clone, Copy)]
struct LocalPosition {
    time_boot_ms: u32,
    x: f64,
    y: f64,
    z: f64,
}

impl From<&LOCAL_POSITION_NED_DATA> for LocalPosition {
    fn from(position: &LOCAL_POSITION_NED_DATA) -> Self {
        Self {
            time_boot_ms: position.time_boot_ms,
            x: f64::from(position.x),
            y: f64::from(position.y),
            z: f64::from(position.z),
        }
    }
}

#[derive(Default)]
struct LocalPositionHistory {
    samples: VecDeque<LocalPosition>,
    start_boot_ms: Option<u32>,
}

impl LocalPositionHistory {
    fn push(&mut self, position: LocalPosition) {
        if ![position.x, position.y, position.z]
            .into_iter()
            .all(f64::is_finite)
        {
            return;
        }
        if let Some(last) = self.samples.back() {
            if position.time_boot_ms == last.time_boot_ms {
                return;
            }
            // A reboot or boot-clock wrap starts a new plot timeline.
            if position.time_boot_ms < last.time_boot_ms {
                self.samples.clear();
                self.start_boot_ms = None;
            }
        }
        self.start_boot_ms.get_or_insert(position.time_boot_ms);
        self.samples.push_back(position);
        let cutoff = position.time_boot_ms.saturating_sub(HISTORY_WINDOW_MS);
        while self.samples.len() > MAX_LOCAL_SAMPLES
            || self
                .samples
                .front()
                .is_some_and(|sample| sample.time_boot_ms < cutoff)
        {
            self.samples.pop_front();
        }
    }

    fn elapsed_seconds(&self, time_boot_ms: u32) -> f64 {
        self.start_boot_ms.map_or(0.0, |start| {
            f64::from(time_boot_ms.saturating_sub(start)) / 1_000.0
        })
    }

    fn altitude_data(&self) -> Vec<(f64, f64)> {
        self.samples
            .iter()
            .map(|position| (self.elapsed_seconds(position.time_boot_ms), -position.z))
            .collect()
    }

    fn time_bounds(&self) -> [f64; 2] {
        let latest = self
            .samples
            .back()
            .map_or(0.0, |position| self.elapsed_seconds(position.time_boot_ms));
        [
            (latest - f64::from(HISTORY_WINDOW_MS) / 1_000.0).max(0.0),
            latest.max(1.0),
        ]
    }

    fn altitude_bounds(&self) -> [f64; 2] {
        let (low, high) = self
            .samples
            .iter()
            .fold((0.0_f64, 0.0_f64), |(low, high), p| {
                (low.min(-p.z), high.max(-p.z))
            });
        let padding = ((high - low) * 0.1).max(1.0);
        [low - padding, high + padding]
    }
}

#[derive(Debug, Clone, Copy, Default)]
enum UiMode {
    #[default]
    Plotting,
    #[expect(dead_code, reason = "The message-rate view is not implemented yet")]
    MessageRate,
}

#[derive(Default)]
struct TuiApp {
    status: Status,
    vehicles: Vec<Vehicle>,
    target: Option<VehicleId>,
    curr_global_pos: Option<Position>,
    local_positions: LocalPositionHistory,
    #[expect(dead_code, reason = "Mode switching is not implemented yet")]
    mode: UiMode,
    message_counts: HashMap<String, u64>,
    start_time: Option<Instant>,
}

impl TuiApp {
    fn update(&mut self, message: MavData) {
        match message {
            MavData::Discovering => self.status = Status::Discovering,
            MavData::Connected(vehicles) => {
                self.status = Status::Connected;
                self.target = vehicles.first().map(|vehicle| vehicle.id);
                self.vehicles = vehicles;
                self.curr_global_pos = None;
                self.local_positions = LocalPositionHistory::default();
                self.start_time = Some(Instant::now());
            }
            MavData::Telemetry(telemetry_event) => {
                if Some(telemetry_event.source) != self.target {
                    return;
                }

                *self
                    .message_counts
                    .entry(telemetry_event.message.message_name().into())
                    .or_insert(0) += 1;

                match telemetry_event.message {
                    MavMessage::GLOBAL_POSITION_INT(global_pos) => {
                        self.curr_global_pos = Some((&global_pos).into());
                    }
                    MavMessage::LOCAL_POSITION_NED(local_pos) => {
                        self.local_positions.push((&local_pos).into());
                    }
                    _ => {}
                }
            }
        }
    }

    fn render_plotting(&self, frame: &mut Frame, endpoint: &str) {
        let status = match self.status {
            Status::Connecting => "Connecting to autopilot...",
            Status::Discovering => "Discovering autopilots...",
            Status::Connected => "Connected",
        };
        let mut lines = vec![
            Line::from(status.bold()),
            Line::from(format!("Endpoint: {endpoint}")),
            Line::from("Esc / q: quit"),
        ];
        for vehicle in &self.vehicles {
            lines.extend([
                Line::default(),
                Line::from("VEHICLE".bold()),
                Line::from_iter(["Vehicle id: ".bold(), Span::raw(vehicle.id.to_string())]),
                Line::from_iter([
                    "Autopilot: ".bold(),
                    Span::raw(format!("{:?}", vehicle.autopilot)),
                ]),
                Line::from_iter([
                    "Vehicle type: ".bold(),
                    Span::raw(format!("{:?}", vehicle.vehicle_type)),
                ]),
            ]);
        }
        let layout = Layout::vertical([
            Constraint::Length(8),
            Constraint::Length(3),
            Constraint::Min(0),
        ]);
        let [status_area, global_pos_area, local_pos_area] = frame.area().layout(&layout);

        frame.render_widget(Paragraph::new(lines).style(Color::White), status_area);
        if let Some(pos) = &self.curr_global_pos {
            frame.render_widget(
                Paragraph::new(format!("Global position: {pos:?}")).wrap(Wrap { trim: true }),
                global_pos_area,
            );
        }

        let Some(latest) = self.local_positions.samples.back() else {
            frame.render_widget("Waiting for local position...", local_pos_area);
            return;
        };
        let pos_plots_area =
            Layout::horizontal([Constraint::Percentage(50), Constraint::Percentage(50)]);
        let [north_east_area, altitude_area] = local_pos_area.layout(&pos_plots_area);

        let north_east_data: Vec<_> = self
            .local_positions
            .samples
            .iter()
            .map(|p| (p.y, p.x))
            .collect();

        let local_pos_dataset = Dataset::default()
            .name("Local position")
            .marker(Marker::Braille)
            .style(Color::Red)
            .data(&north_east_data);

        let x_axis = Axis::default()
            .title("East")
            .bounds([-10.0, 50.0])
            .labels(["-10.0", "50.0"]);

        let y_axis = Axis::default()
            .title("North")
            .bounds([-10.0, 50.0])
            .labels(["-10.0", "50.0"]);

        let local_pos_chart = Chart::new(vec![local_pos_dataset])
            .x_axis(x_axis)
            .y_axis(y_axis);

        frame.render_widget(local_pos_chart, north_east_area);

        let altitude_data = self.local_positions.altitude_data();
        let altitude_dataset = Dataset::default()
            .name("Altitude")
            .marker(Marker::Dot)
            .graph_type(GraphType::Line)
            .style(Color::Blue)
            .data(&altitude_data);

        let time_bounds = self.local_positions.time_bounds();
        let altitude_bounds = self.local_positions.altitude_bounds();
        let x_axis = Axis::default()
            .title("Time [s]")
            .bounds(time_bounds)
            .labels(time_bounds.map(|value| format!("{value:.1}")));

        let y_axis = Axis::default()
            .title("Local altitude [m]")
            .bounds(altitude_bounds)
            .labels(altitude_bounds.map(|value| format!("{value:.1}")));

        let altitude_chart = Chart::new(vec![altitude_dataset])
            .block(Block::bordered().title(format!("Local altitude: {:.1} m", -latest.z)))
            .x_axis(x_axis)
            .y_axis(y_axis);

        frame.render_widget(altitude_chart, altitude_area);
    }

    fn render_message_rate(&self, frame: &mut Frame) {
        let Some(start_time) = self.start_time else {
            return;
        };
        let time_since_start = Instant::now() - start_time;
        let message_bars: Vec<_> = self
            .message_counts
            .iter()
            .map(|(msg_name, count)| {
                Bar::with_label(
                    msg_name.as_str(),
                    count.div_euclid(time_since_start.as_secs()),
                )
            })
            .collect();
        let msg_bars = BarChart::horizontal(message_bars).max(250);

        let layout = Layout::vertical([Constraint::Length(10), Constraint::Fill(1)]);
        let [title_area, bars_area] = frame.area().layout(&layout);
        frame.render_widget("Message rates".bold(), title_area);
        frame.render_widget(msg_bars, bars_area);
    }

    fn render(&self, frame: &mut Frame, endpoint: &str) {
        match self.mode {
            UiMode::Plotting => self.render_plotting(frame, endpoint),
            UiMode::MessageRate => self.render_message_rate(frame),
        }
    }
}

#[cfg(test)]
#[path = "task_tests.rs"]
mod tests;
