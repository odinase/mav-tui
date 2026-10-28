use super::*;

use crate::{
    mav::{Connection, TelemetryEvent, dialect::*, task::MavTask},
    test_support::*,
};
use ratatui::backend::TestBackend;
use tokio::time::timeout;

fn connected_state() -> TuiApp {
    let mut state = TuiApp::default();
    state.update(MavData::Connected(vec![Vehicle {
        id: autopilot_id(),
        autopilot: MavAutopilot::MAV_AUTOPILOT_ARDUPILOTMEGA,
        vehicle_type: MavType::MAV_TYPE_QUADROTOR,
        last_heartbeat: tokio::time::Instant::now(),
    }]));
    state
}

#[tokio::test]
async fn discovery_requests_local_telemetry_and_keeps_the_plot_alive_until_quit() {
    let (listener, config) = listener().await;
    let (tx, mut rx) = mpsc::channel(10);
    let cancel = CancellationToken::new();
    let worker_cancel = cancel.clone();
    let mut worker = tokio::spawn(async move {
        let connection = Connection::connect(config).await?;
        MavTask::new(connection, tx, Some(worker_cancel))
            .start()
            .await
    });
    let (mut peer, _) = timeout(DEADLINE, listener.accept()).await.unwrap().unwrap();
    assert!(matches!(rx.recv().await, Some(MavData::Discovering)));
    heartbeat(&mut peer, 1, MavAutopilot::MAV_AUTOPILOT_ARDUPILOTMEGA).await;

    let Some(MavData::Connected(vehicles)) = timeout(DEADLINE, rx.recv()).await.unwrap() else {
        panic!("expected discovered vehicles")
    };
    assert_eq!(vehicles.len(), 1);
    assert_eq!(vehicles[0].id.system_id, 1);
    let mut state = TuiApp::default();
    state.update(MavData::Connected(vehicles));
    let client = local_stream_request(&mut peer).await;
    let early_sample = local_position(autopilot_id(), 500, -7.0);
    send(&mut peer, early_sample.source, early_sample.message).await;
    assert!(timeout(Duration::from_millis(30), rx.recv()).await.is_err());
    send(
        &mut peer,
        autopilot_id(),
        MavMessage::COMMAND_ACK(COMMAND_ACK_DATA {
            command: MavCmd::MAV_CMD_SET_MESSAGE_INTERVAL,
            result: MavResult::MAV_RESULT_ACCEPTED,
            target_system: client.system_id,
            target_component: client.component_id,
            ..Default::default()
        }),
    )
    .await;
    state.update(timeout(DEADLINE, rx.recv()).await.unwrap().unwrap());

    let unrelated = local_position(
        VehicleId {
            system_id: 2,
            component_id: 1,
        },
        1_000,
        -90.0,
    );
    send(&mut peer, unrelated.source, unrelated.message).await;
    let unrelated = local_position(
        VehicleId {
            system_id: 1,
            component_id: 42,
        },
        1_000,
        -80.0,
    );
    send(&mut peer, unrelated.source, unrelated.message).await;
    for (time_boot_ms, z) in [(1_000, 0.0), (2_000, -10.0), (3_000, -3.0)] {
        let event = local_position(autopilot_id(), time_boot_ms, z);
        send(&mut peer, event.source, event.message).await;
        let message = timeout(DEADLINE, rx.recv()).await.unwrap().unwrap();
        state.update(message);
    }
    assert_eq!(
        state.local_positions.altitude_data(),
        vec![(0.0, 7.0), (0.5, 0.0), (1.5, 10.0), (2.5, 3.0)]
    );
    let mut terminal = terminal();
    terminal
        .draw(|frame| state.render(frame, "test endpoint"))
        .unwrap();
    assert!(screen(&terminal).contains("Local altitude: 3.0 m"));
    assert!(timeout(Duration::from_millis(30), rx.recv()).await.is_err());
    assert!(
        timeout(Duration::from_millis(200), &mut worker)
            .await
            .is_err()
    );

    cancel.cancel();
    timeout(DEADLINE, worker).await.unwrap().unwrap().unwrap();
    assert!(rx.recv().await.is_none());
    assert_peer_closed(peer).await;
}

fn sample(time_boot_ms: u32, z: f64) -> LocalPosition {
    LocalPosition {
        time_boot_ms,
        x: 2.0,
        y: 3.0,
        z,
    }
}

#[test]
fn altitude_uses_seconds_and_upward_metres_without_zeroing_the_first_height() {
    let mut history = LocalPositionHistory::default();
    history.push(sample(900_000, -7.0));
    history.push(sample(901_500, -12.0));
    assert_eq!(history.altitude_data(), vec![(0.0, 7.0), (1.5, 12.0)]);
    assert_eq!(history.time_bounds(), [0.0, 1.5]);
    let [low, high] = history.altitude_bounds();
    assert!(low < 0.0 && high > 12.0);
}

#[test]
fn altitude_axes_handle_empty_flat_single_point_and_negative_heights() {
    let mut history = LocalPositionHistory::default();
    assert_eq!(history.time_bounds(), [0.0, 1.0]);
    assert_eq!(history.altitude_bounds(), [-1.0, 1.0]);
    history.push(sample(1_000, 0.0));
    assert_eq!(history.time_bounds(), [0.0, 1.0]);
    assert_eq!(history.altitude_bounds(), [-1.0, 1.0]);
    history.push(sample(2_000, 0.0));
    assert_eq!(history.altitude_bounds(), [-1.0, 1.0]);
    history.push(sample(3_000, 5.0));
    let [low, high] = history.altitude_bounds();
    assert!(low < -5.0 && high > 0.0 && high - low >= 2.0);
    assert_eq!(history.altitude_data().last().unwrap().1, -5.0);
}

#[test]
fn history_scrolls_thirty_seconds_and_enforces_the_sample_cap() {
    let mut history = LocalPositionHistory::default();
    for index in 0..=400 {
        history.push(sample(index * 100, -10.0));
    }
    assert_eq!(history.samples.len(), 301);
    assert_eq!(history.samples.front().unwrap().time_boot_ms, 10_000);
    assert_eq!(history.time_bounds(), [10.0, 40.0]);
    assert_eq!(history.altitude_data()[0], (10.0, 10.0));

    let mut history = LocalPositionHistory::default();
    for index in 0..=MAX_LOCAL_SAMPLES as u32 {
        history.push(sample(index, -10.0));
    }
    assert_eq!(history.samples.len(), MAX_LOCAL_SAMPLES);
    assert_eq!(history.samples.front().unwrap().time_boot_ms, 1);
    assert_eq!(history.altitude_data()[0].0, 0.001);
}

#[test]
fn history_ignores_duplicates_and_nonfinite_values_and_resets_after_a_reboot_or_wrap() {
    let mut history = LocalPositionHistory::default();
    history.push(sample(10_000, -10.0));
    history.push(sample(10_000, -99.0));
    for invalid in [
        LocalPosition {
            x: f64::NAN,
            ..sample(11_000, -10.0)
        },
        LocalPosition {
            y: f64::INFINITY,
            ..sample(11_000, -10.0)
        },
        sample(11_000, f64::NEG_INFINITY),
    ] {
        history.push(invalid);
    }
    assert_eq!(history.altitude_data(), vec![(0.0, 10.0)]);

    history.push(sample(1_000, -2.0));
    assert_eq!(history.altitude_data(), vec![(0.0, 2.0)]);
    history.push(sample(u32::MAX - 10, -3.0));
    history.push(sample(5, -4.0));
    assert_eq!(history.altitude_data(), vec![(0.0, 4.0)]);
    assert_eq!(history.time_bounds(), [0.0, 1.0]);
}

#[test]
fn ui_filters_both_vehicle_ids_and_preserves_global_position() {
    let mut state = connected_state();
    for source in [
        VehicleId {
            system_id: 2,
            component_id: 1,
        },
        VehicleId {
            system_id: 1,
            component_id: 42,
        },
    ] {
        state.update(MavData::Telemetry(local_position(source, 1_000, -99.0)));
        state.update(MavData::Telemetry(Box::new(TelemetryEvent {
            source,
            message: MavMessage::GLOBAL_POSITION_INT(GLOBAL_POSITION_INT_DATA::default()),
        })));
    }
    assert!(state.local_positions.samples.is_empty());
    assert!(state.curr_global_pos.is_none());
    state.update(MavData::Telemetry(local_position(
        autopilot_id(),
        1_000,
        -10.0,
    )));
    state.update(MavData::Telemetry(Box::new(TelemetryEvent {
        source: autopilot_id(),
        message: MavMessage::GLOBAL_POSITION_INT(GLOBAL_POSITION_INT_DATA {
            lat: 473_977_420,
            lon: 85_455_940,
            alt: 488_000,
            relative_alt: 10_000,
            ..Default::default()
        }),
    })));
    assert_eq!(state.local_positions.altitude_data(), vec![(0.0, 10.0)]);
    assert_eq!(state.curr_global_pos.unwrap().altitude_amsl_m, 488.0);
}

#[test]
fn rendering_waits_for_samples_draws_a_single_point_and_does_not_add_history() {
    let mut state = connected_state();
    let mut terminal = terminal();
    terminal.draw(|frame| state.render(frame, "test")).unwrap();
    assert!(screen(&terminal).contains("Waiting for local position..."));

    state.update(MavData::Telemetry(local_position(
        autopilot_id(),
        42_000,
        -10.0,
    )));
    for _ in 0..10 {
        terminal.draw(|frame| state.render(frame, "test")).unwrap();
    }
    assert_eq!(state.local_positions.samples.len(), 1);
    let content = screen(&terminal);
    assert!(content.contains("Local altitude: 10.0 m"));
    assert!(content.contains("Local altitude [m]"));
    assert!(content.contains("Time [s]"));
    assert!(content.contains("East") && content.contains("North"));
    assert!(
        terminal
            .backend()
            .buffer()
            .content
            .iter()
            .any(|cell| { cell.fg == Color::Blue && cell.symbol() != " " }),
        "single-point altitude plot has no visible blue marker"
    );

    for (width, height) in [(0, 0), (1, 1), (40, 10), (120, 30)] {
        let mut resized = Terminal::new(TestBackend::new(width, height)).unwrap();
        resized.draw(|frame| state.render(frame, "test")).unwrap();
    }
    assert_eq!(state.local_positions.samples.len(), 1);
}

#[tokio::test]
async fn ui_renders_immediately_and_keeps_updates_without_new_messages() {
    for connected in [false, true] {
        let mut terminal = terminal();
        let (mav_tx, mav_rx) = mpsc::channel(10);
        let (_event_tx, event_rx) = mpsc::channel(1);
        let cancel = CancellationToken::new();
        if connected {
            mav_tx
                .send(MavData::Connected(connected_state().vehicles))
                .await
                .unwrap();
            for (time, z) in [(1_000, -7.0), (2_000, -12.0)] {
                mav_tx
                    .send(MavData::Telemetry(local_position(autopilot_id(), time, z)))
                    .await
                    .unwrap();
            }
        }
        let task = TuiTask::new(mav_rx, event_rx, "test endpoint", cancel.clone());
        {
            let mut ui = Box::pin(task.start(&mut terminal));
            assert!(timeout(REDRAW_INTERVAL * 3, &mut ui).await.is_err());
            cancel.cancel();
            timeout(DEADLINE, ui).await.unwrap().unwrap();
        }
        let content = screen(&terminal);
        assert!(content.contains("Endpoint: test endpoint"));
        if connected {
            assert!(content.contains("Connected"));
            assert!(content.contains("Vehicle id: 1:1"));
            assert!(content.contains("MAV_AUTOPILOT_ARDUPILOTMEGA"));
            assert!(content.contains("Local altitude: 12.0 m"));
        } else {
            assert!(content.contains("Connecting to autopilot..."));
        }
    }
}

#[tokio::test]
async fn abort_notification_cancels_the_ui_and_its_siblings() {
    let (_mav_tx, mav_rx) = mpsc::channel(1);
    let (event_tx, event_rx) = mpsc::channel(1);
    let cancel = CancellationToken::new();
    event_tx.send(EventData::Abort).await.unwrap();

    timeout(
        DEADLINE,
        TuiTask::new(mav_rx, event_rx, "test", cancel.clone()).start(&mut terminal()),
    )
    .await
    .unwrap()
    .unwrap();
    assert!(cancel.is_cancelled());
}

#[tokio::test]
async fn unexpected_channel_closure_is_an_error_unless_already_cancelled() {
    for close_events in [false, true] {
        for cancelled in [false, true] {
            let (mav_tx, mav_rx) = mpsc::channel(1);
            let (event_tx, event_rx) = mpsc::channel(1);
            let cancel = CancellationToken::new();
            let _sender_guard = if close_events {
                drop(event_tx);
                (Some(mav_tx), None)
            } else {
                drop(mav_tx);
                (None, Some(event_tx))
            };
            if cancelled {
                cancel.cancel();
            }
            let result = timeout(
                DEADLINE,
                TuiTask::new(mav_rx, event_rx, "test", cancel).start(&mut terminal()),
            )
            .await
            .unwrap();
            if cancelled {
                result.unwrap();
            } else {
                let error = result.unwrap_err().to_string();
                assert!(error.contains(if close_events {
                    "Terminal event"
                } else {
                    "Vehicle update"
                }));
                assert!(error.contains("channel closed unexpectedly"));
            }
        }
    }
}

#[tokio::test]
async fn display_task_bounds_incoming_history_and_redraws_after_backend_resize() {
    let mut terminal = terminal();
    let (mav_tx, mav_rx) = mpsc::channel(MAX_LOCAL_SAMPLES + 10);
    let (_event_tx, event_rx) = mpsc::channel(1);
    let cancel = CancellationToken::new();
    mav_tx
        .send(MavData::Connected(connected_state().vehicles))
        .await
        .unwrap();
    for index in 0..=MAX_LOCAL_SAMPLES as u32 {
        mav_tx
            .send(MavData::Telemetry(local_position(
                autopilot_id(),
                index,
                if index == 0 { -99.0 } else { -10.0 },
            )))
            .await
            .unwrap();
    }
    let task = TuiTask::new(mav_rx, event_rx, "test", cancel.clone());
    terminal.backend_mut().resize(120, 30);
    {
        let mut ui = Box::pin(task.start(&mut terminal));
        assert!(timeout(REDRAW_INTERVAL * 2, &mut ui).await.is_err());
        cancel.cancel();
        timeout(DEADLINE, ui).await.unwrap().unwrap();
    }
    assert_eq!(terminal.backend().buffer().area.width, 120);
    let content = screen(&terminal);
    assert!(content.contains("Local altitude: 10.0 m"));
    // The first height must have left the bounded history and its axis range.
    assert!(content.contains("11.0"), "{content}");
    assert!(!content.contains("108.9"), "{content}");
}
