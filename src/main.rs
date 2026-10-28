use anyhow::Result;
use mav_tui::{app, mav::ConnectionConfig};

#[tokio::main]
async fn main() -> Result<()> {
    app::run(ConnectionConfig::default()).await
}
