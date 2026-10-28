use super::*;

use std::{io, time::Duration};

use anyhow::bail;
use crossterm::event::{Event, KeyCode, KeyEvent, KeyModifiers};
use futures::{Stream, channel::mpsc::unbounded, stream};
use tokio::{
    sync::oneshot,
    time::{sleep, timeout},
};

use crate::{mav::dialect::*, test_support::*};

async fn run_app<S>(
    terminal: Terminal<ratatui::backend::TestBackend>,
    reader: S,
    config: ConnectionConfig,
) -> Result<()>
where
    S: Stream<Item = io::Result<Event>> + Unpin + Send + 'static,
{
    run_session(terminal, config, |task| task.run_with_reader(reader)).await
}

#[tokio::test]
async fn multiple_autopilots_are_rejected_instead_of_mixing_their_plots() {
    let (listener, config) = listener().await;
    let app = tokio::spawn(run_app(terminal(), stream::pending(), config));
    let (mut peer, _) = timeout(DEADLINE, listener.accept()).await.unwrap().unwrap();
    heartbeat(&mut peer, 1, MavAutopilot::MAV_AUTOPILOT_ARDUPILOTMEGA).await;
    heartbeat(&mut peer, 2, MavAutopilot::MAV_AUTOPILOT_PX4).await;
    let error = timeout(DEADLINE, app).await.unwrap().unwrap().unwrap_err();
    let message = format!("{error:#}");
    assert!(message.contains("Multiple autopilots"), "{message}");
    assert!(
        message.contains("1:1") && message.contains("2:1"),
        "{message}"
    );
    assert_peer_closed(peer).await;
}

#[tokio::test]
async fn refused_connection_exits_without_keyboard_input_and_preserves_the_error() {
    let (listener, config) = listener().await;
    let endpoint = config.endpoint.clone();
    drop(listener);
    let error = timeout(DEADLINE, run_app(terminal(), stream::pending(), config))
        .await
        .expect("application waited for keyboard input after a connection error")
        .unwrap_err();
    let message = format!("{error:#}");
    assert!(message.contains(&endpoint), "{message}");
    assert!(message.contains("Connect to"), "{message}");
    assert!(!message.contains("channel closed"), "{message}");
}

#[tokio::test]
async fn discovery_timeout_stops_the_ui_keyboard_and_connection() {
    let (listener, config) = listener().await;
    let app = tokio::spawn(run_app(terminal(), stream::pending(), config));
    let (peer, _) = timeout(DEADLINE, listener.accept()).await.unwrap().unwrap();
    let error = timeout(DEADLINE, app)
        .await
        .expect("application hung after discovery timed out")
        .unwrap()
        .unwrap_err();
    assert!(format!("{error:#}").contains("No autopilot heartbeat"));
    assert_peer_closed(peer).await;
}

#[tokio::test]
async fn disconnect_exits_during_discovery_and_after_connection() {
    for connected in [false, true] {
        let (listener, config) = listener().await;
        let app = tokio::spawn(run_app(terminal(), stream::pending(), config));
        let (mut peer, _) = timeout(DEADLINE, listener.accept()).await.unwrap().unwrap();
        if connected {
            heartbeat(&mut peer, 1, MavAutopilot::MAV_AUTOPILOT_ARDUPILOTMEGA).await;
            acknowledge_local_stream(&mut peer, MavResult::MAV_RESULT_ACCEPTED).await;
            sleep(Duration::from_millis(250)).await;
            assert!(!app.is_finished(), "application exited after discovery");
        }
        drop(peer);
        let error = timeout(DEADLINE, app)
            .await
            .expect("application hung after peer disconnect")
            .unwrap()
            .unwrap_err();
        let message = format!("{error:#}");
        assert!(message.contains("disconnected"), "{message}");
        if connected {
            assert!(message.contains("Monitor MAVLink connection"), "{message}");
        }
    }
}

#[tokio::test]
async fn silent_live_connection_exits_with_the_idle_timeout() {
    let (listener, mut config) = listener().await;
    config.idle_timeout = Duration::from_millis(350);
    let app = tokio::spawn(run_app(terminal(), stream::pending(), config));
    let (mut peer, _) = timeout(DEADLINE, listener.accept()).await.unwrap().unwrap();
    heartbeat(&mut peer, 1, MavAutopilot::MAV_AUTOPILOT_ARDUPILOTMEGA).await;
    acknowledge_local_stream(&mut peer, MavResult::MAV_RESULT_ACCEPTED).await;
    let error = timeout(DEADLINE, app).await.unwrap().unwrap().unwrap_err();
    let message = format!("{error:#}");
    assert!(message.contains("no MAVLink traffic"), "{message}");
    assert!(message.contains("Monitor MAVLink connection"), "{message}");
    assert_peer_closed(peer).await;
}

#[tokio::test]
async fn quit_during_discovery_or_connected_stops_every_worker_and_closes_the_peer() {
    for connected in [false, true] {
        let (listener, config) = listener().await;
        let (keys, reader) = unbounded();
        let app = tokio::spawn(run_app(terminal(), reader, config));
        let (mut peer, _) = timeout(DEADLINE, listener.accept()).await.unwrap().unwrap();
        if connected {
            heartbeat(&mut peer, 1, MavAutopilot::MAV_AUTOPILOT_ARDUPILOTMEGA).await;
            acknowledge_local_stream(&mut peer, MavResult::MAV_RESULT_ACCEPTED).await;
            sleep(Duration::from_millis(250)).await;
            assert!(!app.is_finished());
        }
        keys.unbounded_send(Ok(Event::Key(KeyEvent::new(
            KeyCode::Char('q'),
            crossterm::event::KeyModifiers::NONE,
        ))))
        .unwrap();
        timeout(DEADLINE, app).await.unwrap().unwrap().unwrap();
        assert_peer_closed(peer).await;
    }
}

#[tokio::test]
async fn cancellation_before_connect_skips_connection_setup() {
    let cancel = CancellationToken::new();
    cancel.cancel();
    let (tx, mut rx) = mpsc::channel(1);
    let config = ConnectionConfig {
        endpoint: "invalid endpoint".into(),
        ..ConnectionConfig::default()
    };
    timeout(
        DEADLINE,
        run_mav_task(Connection::connect(config), tx, cancel),
    )
    .await
    .unwrap()
    .unwrap();
    assert!(rx.recv().await.is_none());
}

#[tokio::test]
async fn supervisor_preserves_the_first_error_and_awaits_sibling_cleanup() {
    let cancel = CancellationToken::new();
    let sibling_cancel = cancel.clone();
    let (cleaned, cleanup) = tokio::sync::oneshot::channel();
    let mut tasks = JoinSet::new();
    tasks.spawn(async { bail!("original connection failure") });
    tasks.spawn(async move {
        sibling_cancel.cancelled().await;
        cleaned.send(()).unwrap();
        bail!("secondary shutdown failure")
    });
    let error = timeout(DEADLINE, supervise(tasks, cancel))
        .await
        .unwrap()
        .unwrap_err();
    assert_eq!(error.to_string(), "original connection failure");
    cleanup.await.unwrap();
}

#[tokio::test]
async fn supervisor_reports_panics_and_cancels_siblings() {
    let cancel = CancellationToken::new();
    let mut tasks = JoinSet::new();
    tasks.spawn(async { panic!("simulated worker panic") });
    let (tx, _rx) = mpsc::channel(1);
    tasks.spawn(EventTask::new(tx, Some(cancel.clone())).run_with_reader(stream::pending()));
    let error = timeout(DEADLINE, supervise(tasks, cancel))
        .await
        .unwrap()
        .unwrap_err();
    assert!(format!("{error:#}").contains("simulated worker panic"));
}

#[tokio::test]
async fn telemetry_request_rejection_and_timeout_stop_the_application() {
    for result in [Some(MavResult::MAV_RESULT_DENIED), None] {
        let (listener, mut config) = listener().await;
        config.request_timeout = Duration::from_millis(150);
        let app = tokio::spawn(run_app(terminal(), stream::pending(), config));
        let (mut peer, _) = timeout(DEADLINE, listener.accept()).await.unwrap().unwrap();
        heartbeat(&mut peer, 1, MavAutopilot::MAV_AUTOPILOT_ARDUPILOTMEGA).await;
        if let Some(result) = result {
            acknowledge_local_stream(&mut peer, result).await;
        } else {
            local_stream_request(&mut peer).await;
        }
        let error = timeout(DEADLINE, app).await.unwrap().unwrap().unwrap_err();
        let message = format!("{error:#}");
        assert!(
            message.contains("Request local position telemetry at 5 Hz"),
            "{message}"
        );
        assert!(
            message.contains(if result.is_some() {
                "MAV_RESULT_DENIED"
            } else {
                "Timed out"
            }),
            "{message}"
        );
        assert_peer_closed(peer).await;
    }
}

#[tokio::test]
async fn quitting_while_waiting_for_the_stream_ack_closes_the_connection() {
    let (listener, config) = listener().await;
    let (keys, reader) = unbounded();
    let app = tokio::spawn(run_app(terminal(), reader, config));
    let (mut peer, _) = timeout(DEADLINE, listener.accept()).await.unwrap().unwrap();
    heartbeat(&mut peer, 1, MavAutopilot::MAV_AUTOPILOT_ARDUPILOTMEGA).await;
    local_stream_request(&mut peer).await;
    keys.unbounded_send(Ok(Event::Key(KeyEvent::new(
        KeyCode::Char('q'),
        crossterm::event::KeyModifiers::NONE,
    ))))
    .unwrap();
    timeout(DEADLINE, app).await.unwrap().unwrap().unwrap();
    assert_peer_closed(peer).await;
}

#[tokio::test]
async fn input_failure_preserves_its_cause_while_the_other_workers_stop() {
    let (_listener, config) = listener().await;
    for reader in [
        stream::iter(vec![Err(io::Error::other("synthetic input failure"))]),
        stream::iter(vec![]),
    ] {
        let error = timeout(DEADLINE, run_app(terminal(), reader, config.clone()))
            .await
            .expect("input failure did not stop the application")
            .unwrap_err();
        let message = format!("{error:#}");
        assert!(
            message.contains("synthetic input failure")
                || message.contains("Terminal input stream closed"),
            "{message}"
        );
        assert!(
            !message.contains("channel closed unexpectedly"),
            "{message}"
        );
    }
}

#[tokio::test]
async fn cancellation_or_quit_while_connecting_drops_setup_and_stops_workers() {
    for quit in [false, true] {
        let cancel = CancellationToken::new();
        let (mav_tx, mav_rx) = mpsc::channel(1);
        let (event_tx, event_rx) = mpsc::channel(1);
        let (keys, reader) = unbounded();
        let (ready, started) = oneshot::channel();
        let (setup_guard, setup_dropped) = oneshot::channel::<()>();
        let connection = async move {
            let _setup_guard = setup_guard;
            ready.send(()).unwrap();
            std::future::pending::<Result<Connection>>().await
        };
        let mut tasks = JoinSet::new();
        tasks.spawn(run_mav_task(connection, mav_tx.clone(), cancel.clone()));
        tasks.spawn(EventTask::new(event_tx.clone(), Some(cancel.clone())).run_with_reader(reader));
        let tui = TuiTask::new(mav_rx, event_rx, "test endpoint", cancel.clone());
        tasks.spawn(async move { tui.start(&mut terminal()).await });
        timeout(DEADLINE, started).await.unwrap().unwrap();
        if quit {
            keys.unbounded_send(Ok(Event::Key(KeyEvent::new(
                KeyCode::Char('q'),
                KeyModifiers::NONE,
            ))))
            .unwrap();
        } else {
            cancel.cancel();
        }

        timeout(DEADLINE, supervise(tasks, cancel.clone()))
            .await
            .unwrap()
            .unwrap();
        assert!(cancel.is_cancelled());
        assert!(
            setup_dropped.await.is_err(),
            "connection setup was retained"
        );
        drop((mav_tx, event_tx));
    }
}
