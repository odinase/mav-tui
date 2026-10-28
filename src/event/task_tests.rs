use super::*;

use std::{task::Poll, time::Duration};

use crossterm::event::KeyModifiers;
use futures::stream;
use tokio::{sync::oneshot, time::timeout};

const DEADLINE: Duration = Duration::from_secs(1);

fn key(code: KeyCode, kind: KeyEventKind) -> Event {
    Event::Key(KeyEvent::new_with_kind(code, KeyModifiers::NONE, kind))
}

#[tokio::test]
async fn quit_keys_cancel_notify_once_and_stop_before_reading_another_event() {
    for code in [KeyCode::Esc, KeyCode::Char('q'), KeyCode::Char('Q')] {
        let (tx, mut rx) = mpsc::channel(2);
        let cancel = CancellationToken::new();
        let task = EventTask::new(tx, Some(cancel.clone()));
        let reader = stream::iter([
            Ok(key(code, KeyEventKind::Press)),
            Err(io::Error::other("must not read after quitting")),
        ]);

        timeout(DEADLINE, task.run_with_reader(reader))
            .await
            .unwrap()
            .unwrap();

        assert!(cancel.is_cancelled());
        assert!(matches!(rx.recv().await, Some(EventData::Abort)));
        assert!(rx.recv().await.is_none());
    }
}

#[tokio::test]
async fn ordinary_events_and_quit_key_releases_and_repeats_are_ignored() {
    let (tx, mut rx) = mpsc::channel(1);
    let cancel = CancellationToken::new();
    let task = EventTask::new(tx, Some(cancel.clone()));
    let reader = stream::iter([
        Ok(key(KeyCode::Char('x'), KeyEventKind::Press)),
        Ok(key(KeyCode::Esc, KeyEventKind::Release)),
        Ok(key(KeyCode::Char('q'), KeyEventKind::Repeat)),
        Ok(key(KeyCode::Char('Q'), KeyEventKind::Release)),
        Ok(Event::Resize(80, 24)),
        Ok(Event::FocusGained),
    ]);

    let error = timeout(DEADLINE, task.run_with_reader(reader))
        .await
        .unwrap()
        .unwrap_err();

    assert_eq!(error.to_string(), "Terminal input stream closed");
    assert!(!cancel.is_cancelled());
    assert!(rx.recv().await.is_none());
}

#[tokio::test]
async fn cancellation_stops_an_input_stream_that_is_waiting_for_events() {
    let (tx, mut rx) = mpsc::channel(1);
    let cancel = CancellationToken::new();
    let task = EventTask::new(tx, Some(cancel.clone()));
    let (ready, started) = oneshot::channel();
    let mut ready = Some(ready);
    let reader = stream::poll_fn(move |_| {
        if let Some(ready) = ready.take() {
            let _ = ready.send(());
        }
        Poll::Pending
    });
    let worker = tokio::spawn(task.run_with_reader(reader));
    timeout(DEADLINE, started).await.unwrap().unwrap();

    cancel.cancel();
    timeout(DEADLINE, worker).await.unwrap().unwrap().unwrap();

    assert!(rx.recv().await.is_none());
}

#[tokio::test]
async fn start_with_an_already_cancelled_token_does_not_open_terminal_input() {
    let (tx, mut rx) = mpsc::channel(1);
    let cancel = CancellationToken::new();
    cancel.cancel();

    timeout(DEADLINE, EventTask::new(tx, Some(cancel)).start())
        .await
        .unwrap()
        .unwrap();

    assert!(rx.recv().await.is_none());
}

#[tokio::test]
async fn input_errors_keep_their_context_and_unexpected_eof_is_an_error() {
    for (events, expected) in [
        (vec![], "Terminal input stream closed"),
        (
            vec![Err(io::Error::other("synthetic input failure"))],
            "Read terminal input",
        ),
    ] {
        let (tx, mut rx) = mpsc::channel(1);
        let cancel = CancellationToken::new();
        let task = EventTask::new(tx, Some(cancel.clone()));
        let is_io_error = !events.is_empty();
        let error = timeout(DEADLINE, task.run_with_reader(stream::iter(events)))
            .await
            .unwrap()
            .unwrap_err();

        assert_eq!(error.to_string(), expected);
        if is_io_error {
            assert_eq!(error.root_cause().to_string(), "synthetic input failure");
        }
        assert!(!cancel.is_cancelled());
        assert!(rx.recv().await.is_none());
    }
}

#[tokio::test]
async fn full_or_closed_notification_channels_do_not_block_quitting() {
    for full in [true, false] {
        let (tx, rx) = mpsc::channel(1);
        let mut rx = Some(rx);
        if full {
            tx.try_send(EventData::Abort).unwrap();
        } else {
            drop(rx.take());
        }
        let cancel = CancellationToken::new();
        let task = EventTask::new(tx, Some(cancel.clone()));
        let reader = stream::iter([Ok(key(KeyCode::Char('q'), KeyEventKind::Press))]);

        timeout(DEADLINE, task.run_with_reader(reader))
            .await
            .unwrap()
            .unwrap();

        assert!(cancel.is_cancelled());
        if let Some(mut rx) = rx {
            assert!(matches!(rx.recv().await, Some(EventData::Abort)));
            assert!(rx.recv().await.is_none());
        }
    }
}

#[tokio::test]
async fn optional_token_still_allows_a_quit_notification() {
    let (tx, mut rx) = mpsc::channel(1);
    let task = EventTask::new(tx, None);
    let reader = stream::iter([Ok(key(KeyCode::Esc, KeyEventKind::Press))]);

    timeout(DEADLINE, task.run_with_reader(reader))
        .await
        .unwrap()
        .unwrap();

    assert!(matches!(rx.recv().await, Some(EventData::Abort)));
    assert!(rx.recv().await.is_none());
}
