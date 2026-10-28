use std::io;

use super::event_data::EventData;
use anyhow::{Context, Result, bail};
use crossterm::event::{Event, EventStream, KeyCode, KeyEvent, KeyEventKind};
use futures::{Stream, StreamExt};
use tokio::{select, sync::mpsc};
use tokio_util::sync::CancellationToken;

pub struct EventTask {
    event_tx: mpsc::Sender<EventData>,
    cancel_token: CancellationToken,
}

pub enum HandleEventResult {
    CloseApp,
    Continue,
    MenuToRight,
    MenuToLeft,
}

impl EventTask {
    pub fn new(event_tx: mpsc::Sender<EventData>, cancel_token: Option<CancellationToken>) -> Self {
        Self {
            event_tx,
            cancel_token: cancel_token.unwrap_or_default(),
        }
    }

    pub async fn start(self) -> Result<()> {
        if self.cancel_token.is_cancelled() {
            return Ok(());
        }
        self.run_with_reader(EventStream::new()).await
    }

    pub(crate) async fn run_with_reader<S>(self, mut reader: S) -> Result<()>
    where
        S: Stream<Item = io::Result<Event>> + Unpin,
    {
        loop {
            let maybe_event = select! {
                biased;
                _ = self.cancel_token.cancelled() => return Ok(()),
                event = reader.next() => event,
            };

            match maybe_event {
                Some(Ok(event)) => match self.handle_event(&event) {
                    HandleEventResult::CloseApp => {
                        self.cancel_token.cancel();
                        // Cancellation remains authoritative if this channel is full or closed.
                        let _ = self.event_tx.try_send(EventData::Abort);
                        return Ok(());
                    }
                    HandleEventResult::MenuToLeft => {
                        let _ = self.event_tx.try_send(EventData::MenuToLeft);
                    },
                    HandleEventResult::MenuToRight => {
                        let _ = self.event_tx.try_send(EventData::MenuToRight);
                    },
                    HandleEventResult::Continue => {}
                },
                Some(Err(error)) => return Err(error).context("Read terminal input"),
                None => bail!("Terminal input stream closed"),
            }
        }
    }

    fn is_close_app_event(&self, event: &Event) -> bool {
        matches!(
            event,
            Event::Key(KeyEvent {
                code: KeyCode::Esc | KeyCode::Char('q' | 'Q'),
                kind: KeyEventKind::Press,
                ..
            })
        )
    }

    /// Decide whether the input loop should stop for this event.
    fn handle_event(&self, event: &Event) -> HandleEventResult {
        if self.is_close_app_event(event) {
            return HandleEventResult::CloseApp;
        }
        match event {
            Event::Key(KeyEvent { code, kind: KeyEventKind::Press, ..}) => match code {
                KeyCode::Right => HandleEventResult::MenuToRight,
                KeyCode::Left => HandleEventResult::MenuToLeft,
                _ => HandleEventResult::Continue
            }
            _ => HandleEventResult::Continue,
        }
    }
}

#[cfg(test)]
#[path = "task_tests.rs"]
mod tests;
