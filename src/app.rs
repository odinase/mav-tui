use std::future::Future;

use anyhow::{Context, Result};
use ratatui::{Terminal, backend::Backend};
use tokio::{select, sync::mpsc, task::JoinSet};
use tokio_util::sync::CancellationToken;

use crate::{
    event::task::EventTask,
    mav::{Connection, ConnectionConfig, mav_data::MavData, task::MavTask},
    tui::TuiTask,
};

struct TerminalGuard;

impl Drop for TerminalGuard {
    fn drop(&mut self) {
        ratatui::restore();
    }
}

pub async fn run(config: ConnectionConfig) -> Result<()> {
    // Keep restoration outside all workers, including when initialization fails.
    let _terminal_guard = TerminalGuard;
    let terminal = ratatui::try_init().context("Initialize terminal")?;
    run_session(terminal, config, EventTask::start).await
}

async fn run_session<B, F>(
    mut terminal: Terminal<B>,
    config: ConnectionConfig,
    start_events: impl FnOnce(EventTask) -> F,
) -> Result<()>
where
    B: Backend + Send + 'static,
    B::Error: Send + Sync + 'static,
    F: Future<Output = Result<()>> + Send + 'static,
{
    let (mav_tx, mav_rx) = mpsc::channel(10);
    let (event_tx, event_rx) = mpsc::channel(10);
    let cancel_token = CancellationToken::new();
    let tui = TuiTask::new(
        mav_rx,
        event_rx,
        config.endpoint.clone(),
        cancel_token.clone(),
    );
    let mut tasks = JoinSet::new();

    tasks.spawn(start_events(EventTask::new(
        event_tx.clone(),
        Some(cancel_token.clone()),
    )));
    tasks.spawn(run_mav_task(
        Connection::connect(config),
        mav_tx.clone(),
        cancel_token.clone(),
    ));
    tasks.spawn(async move {
        tui.start(&mut terminal)
            .await
            .context("Vehicle display failed")
    });

    // Keep both channels open until worker results are observed, so channel
    // closure cannot race with and hide the original worker failure.
    let result = supervise(tasks, cancel_token).await;
    drop((mav_tx, event_tx));
    result
}

async fn run_mav_task(
    connection: impl Future<Output = Result<Connection>>,
    tx: mpsc::Sender<MavData>,
    cancel_token: CancellationToken,
) -> Result<()> {
    let connection = select! {
        biased;
        _ = cancel_token.cancelled() => return Ok(()),
        result = connection => result?,
    };
    MavTask::new(connection, tx, Some(cancel_token))
        .start()
        .await
}

async fn supervise(mut tasks: JoinSet<Result<()>>, cancel_token: CancellationToken) -> Result<()> {
    let mut first_error = None;
    while let Some(result) = tasks.join_next().await {
        let result = result
            .context("Application worker failed")
            .and_then(|result| result);
        if let Err(error) = result
            && first_error.is_none()
        {
            first_error = Some(error);
        }
        // Every worker must stop when a sibling exits, including the keyboard.
        cancel_token.cancel();
    }
    match first_error {
        Some(error) => Err(error),
        None => Ok(()),
    }
}

#[cfg(test)]
#[path = "app_tests.rs"]
mod tests;
