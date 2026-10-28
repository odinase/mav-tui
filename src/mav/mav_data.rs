use crate::mav::{TelemetryEvent, Vehicle};

#[derive(Debug)]
pub enum MavData {
    Discovering,
    Connected(Vec<Vehicle>),
    Telemetry(Box<TelemetryEvent>),
}
