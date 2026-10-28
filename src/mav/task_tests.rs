use super::*;

use crate::{mav::dialect::MavAutopilot, test_support::*};
use tokio::time::{Duration, timeout};

#[tokio::test]
async fn cancelled_or_backpressured_startup_joins_the_connection_tasks() {
    for already_cancelled in [false, true] {
        let (listener, config) = listener().await;
        let connection = Connection::connect(config).await.unwrap();
        let mut telemetry = connection.subscribe();
        let (peer, _) = listener.accept().await.unwrap();
        let (tx, mut rx) = mpsc::channel(1);
        let cancel = CancellationToken::new();
        if already_cancelled {
            cancel.cancel();
        } else {
            tx.try_send(MavData::Discovering).unwrap();
        }
        let mut worker = tokio::spawn(MavTask::new(connection, tx, Some(cancel.clone())).start());
        if !already_cancelled {
            assert!(
                timeout(Duration::from_millis(30), &mut worker)
                    .await
                    .is_err()
            );
            cancel.cancel();
        }
        timeout(DEADLINE, worker).await.unwrap().unwrap().unwrap();
        if !already_cancelled {
            assert!(matches!(rx.recv().await, Some(MavData::Discovering)));
        }
        assert!(rx.recv().await.is_none());
        let error = telemetry.recv().await.unwrap_err().to_string();
        assert!(error.contains("client shut down"), "{error}");
        assert_peer_closed(peer).await;
    }
}

#[tokio::test]
async fn cancellation_interrupts_discovery_and_streaming_backpressure() {
    for streaming in [false, true] {
        let (listener, mut config) = listener().await;
        if !streaming {
            config.discovery_timeout = Duration::from_secs(60);
        }
        let connection = Connection::connect(config).await.unwrap();
        let (mut peer, _) = listener.accept().await.unwrap();
        let (tx, mut rx) = mpsc::channel(1);
        let cancel = CancellationToken::new();
        let worker = tokio::spawn(MavTask::new(connection, tx, Some(cancel.clone())).start());
        assert!(matches!(rx.recv().await, Some(MavData::Discovering)));
        if streaming {
            heartbeat(&mut peer, 1, MavAutopilot::MAV_AUTOPILOT_ARDUPILOTMEGA).await;
            assert!(matches!(
                timeout(DEADLINE, rx.recv()).await.unwrap(),
                Some(MavData::Connected(_))
            ));
            acknowledge_local_stream(
                &mut peer,
                crate::mav::dialect::MavResult::MAV_RESULT_ACCEPTED,
            )
            .await;
            for time in [1_000, 2_000] {
                let event = local_position(autopilot_id(), time, -10.0);
                send(&mut peer, event.source, event.message).await;
            }
            timeout(DEADLINE, async {
                while rx.is_empty() {
                    tokio::task::yield_now().await;
                }
            })
            .await
            .unwrap();
            // Give the worker a chance to reach the second, blocked send.
            tokio::time::sleep(Duration::from_millis(30)).await;
        }
        cancel.cancel();
        timeout(DEADLINE, worker).await.unwrap().unwrap().unwrap();
        assert_peer_closed(peer).await;
    }
}

#[tokio::test]
async fn display_channel_error_explicitly_shuts_down_the_connection() {
    let (listener, config) = listener().await;
    let connection = Connection::connect(config).await.unwrap();
    let mut telemetry = connection.subscribe();
    let (peer, _) = listener.accept().await.unwrap();
    let (tx, rx) = mpsc::channel(1);
    drop(rx);

    let error = timeout(DEADLINE, MavTask::new(connection, tx, None).start())
        .await
        .unwrap()
        .unwrap_err();
    assert_eq!(error.to_string(), "Vehicle display closed");
    let error = telemetry.recv().await.unwrap_err().to_string();
    assert!(error.contains("client shut down"), "{error}");
    assert_peer_closed(peer).await;
}
