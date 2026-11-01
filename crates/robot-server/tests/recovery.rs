// FEATURE: operator bumper recovery and reset without the acquire/reset cycle.
// SPEC: a live grant can reset an unowned latch without claiming control;
// a pressed bumper permits only explicit bounded reverse pulses, never an
// ordinary drive. Cliff/drop, stale/missing sensors, foreign ownership and
// exhausted budgets refuse recovery. Stop/release/new hazards cancel it.
// IMPLEMENTATION: shared recovery executor, per-latch Safety budget and
// sensor freshness; reset checks the operator grant without claiming control.
// RESULT: IMPLEMENTED — offline Player/supervisor and WS scenarios pass.
// STATUS: DONE
mod harness {
    include!("mcp.rs");
}
use futures_util::{SinkExt, StreamExt};
use harness::{rpc, rpc_json, Server};
use serde_json::{json, Value};
use std::{
    sync::{Arc, Mutex},
    time::Duration,
};
use tokio::{
    io::{AsyncBufReadExt, AsyncWriteExt, BufReader},
    net::TcpListener,
};
use tokio_tungstenite::{connect_async, tungstenite::Message};

type Log = Arc<Mutex<Vec<String>>>;
async fn supervisor() -> (String, Log) {
    supervisor_delayed(Duration::ZERO).await
}
async fn supervisor_delayed(ack_delay: Duration) -> (String, Log) {
    let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
    let address = listener.local_addr().unwrap().to_string();
    let log = Arc::new(Mutex::new(Vec::new()));
    let logs = log.clone();
    tokio::spawn(async move {
        loop {
            let (s, _) = listener.accept().await.unwrap();
            let log = logs.clone();
            tokio::spawn(async move {
                let (rx, mut tx) = s.into_split();
                let mut lines = BufReader::new(rx).lines();
                while let Ok(Some(line)) = lines.next_line().await {
                    log.lock().unwrap().push(line.clone());
                    let reply = if line.starts_with("AUTH ") {
                        "AUTHENTICATED\n"
                    } else if line.starts_with("DRIVE ") {
                        "DRIVING\n"
                    } else if line == "STOP" {
                        "STOPPED\n"
                    } else {
                        "REFUSED\n"
                    };
                    if line.starts_with("DRIVE ") {
                        tokio::time::sleep(ack_delay).await;
                    }
                    if tx.write_all(reply.as_bytes()).await.is_err() {
                        break;
                    }
                }
            });
        }
    });
    (address, log)
}
async fn call(s: &Server, tool: &str, args: Value) -> Value {
    rpc_json(
        s.1,
        json!({"jsonrpc":"2.0","id":1,"method":"tools/call",
        "params":{"name":tool,"arguments":args}}),
        None,
    )
    .await
}
async fn rig(grant: &str) -> (Server, Log) {
    let (address, log) = supervisor().await;
    let s = harness::start_env_supervisor(None, grant, (&address, "")).await;
    tokio::time::timeout(Duration::from_secs(10), async {
        loop {
            if rpc(
                s.1,
                json!({"jsonrpc":"2.0","id":0,"method":"tools/list"}),
                None,
            )
            .await
            .is_some()
            {
                break;
            }
            tokio::time::sleep(Duration::from_millis(20)).await;
        }
    })
    .await
    .unwrap();
    (s, log)
}
fn bumper(pressed: bool) -> Vec<u8> {
    harness::packet(
        player_bridge::BUMPER_CODE,
        0,
        1,
        10.,
        &[0, 0, 0, 3, 0, 0, 0, 3, pressed as u8, 0, 0, 0],
    )
}
fn ir(index: u16, active: bool) -> Vec<u8> {
    let mut payload = Vec::new();
    for values in [vec![0.0_f32], vec![if active { 1.0_f32 } else { 0.0 }]] {
        payload.extend(1_u32.to_be_bytes());
        payload.extend(1_u32.to_be_bytes());
        payload.extend(values[0].to_be_bytes());
    }
    harness::packet(player_bridge::IR_CODE, index, 1, 10., &payload)
}
async fn sensors(s: &Server, pressed: bool, cliff: bool, drop: bool) {
    s.2.send(bumper(pressed)).await.unwrap();
    s.2.send(ir(1, cliff)).await.unwrap();
    s.2.send(ir(2, drop)).await.unwrap();
    // Input and HTTP are independent transports: synchronize on telemetry.
    tokio::time::timeout(Duration::from_secs(3), async {
        loop {
            let o = call(s, "robot_observe", json!({})).await;
            let d = &o["result"]["structuredContent"]["sensors"];
            if d["bumper"] == if pressed { "PRESSED" } else { "clear" }
                && d["cliff_ranges"][0] == if cliff { 1.0 } else { 0.0 }
                && d["drop_ranges"][0] == if drop { 1.0 } else { 0.0 }
            {
                break;
            }
            tokio::time::sleep(Duration::from_millis(10)).await;
        }
    })
    .await
    .expect("hazard telemetry");
}
fn ok(r: &Value) {
    assert_eq!(r["result"]["isError"], false, "{r}");
}
fn error(r: &Value, needle: &str) {
    assert_eq!(r["result"]["isError"], true, "{r}");
    assert!(
        r["result"]["content"][0]["text"]
            .as_str()
            .unwrap()
            .to_lowercase()
            .contains(needle),
        "{r}"
    );
}
async fn stopped(log: &Log) {
    tokio::time::timeout(Duration::from_secs(2), async {
        loop {
            if log.lock().unwrap().iter().any(|l| l == "STOP") {
                break;
            }
            tokio::time::sleep(Duration::from_millis(10)).await;
        }
    })
    .await
    .expect("recovery must send supervisor STOP");
}
const GRANT: &str = "mcp,0.2,1.0,2000,10000,600";

#[tokio::test]
async fn recovery_reset_unowned_latch_then_acquire() {
    let (s, _) = rig(GRANT).await;
    sensors(&s, true, false, false).await;
    error(
        &call(&s, "robot_acquire_control", json!({})).await,
        "blocked",
    );
    error(&call(&s, "robot_reset_hazards", json!({})).await, "bumper");
    sensors(&s, false, false, false).await;
    ok(&call(&s, "robot_reset_hazards", json!({})).await);
    let acquired = call(&s, "robot_acquire_control", json!({})).await;
    ok(&acquired);
    assert_eq!(
        acquired["result"]["structuredContent"]["state"], "acquired",
        "reset must not acquire: {acquired}"
    );
}

#[tokio::test]
async fn recovery_reset_requires_grant_and_strict_arguments() {
    let (s, _) = rig("other,0.2,1.0,2000,10000,600").await;
    error(&call(&s, "robot_reset_hazards", json!({})).await, "grant");
    let (s, _) = rig(GRANT).await;
    error(
        &call(&s, "robot_reset_hazards", json!({"force":true})).await,
        "unknown",
    );
}

#[tokio::test]
async fn recovery_pressed_bumper_reverse_is_bounded_idempotent_and_latched() {
    let (s, log) = rig(GRANT).await;
    sensors(&s, true, false, false).await;
    let request = json!({"command_id":"recover-1"});
    let sent = call(&s, "robot_recover_bumper", request.clone()).await;
    ok(&sent);
    assert_eq!(
        sent["result"]["structuredContent"]["requested"],
        json!({"linear_mps":-0.1,"angular_radps":0.0,"duration_ms":300})
    );
    ok(&call(&s, "robot_recover_bumper", request.clone()).await);
    error(
        &call(
            &s,
            "robot_recover_bumper",
            json!({"command_id":"recover-2"}),
        )
        .await,
        "busy",
    );
    error(&call(&s, "robot_reset_hazards", json!({})).await, "busy");
    error(
        &call(
            &s,
            "robot_drive_for",
            json!({"command_id":"normal","linear_mps":-0.1,"angular_radps":0.0,"duration_ms":200}),
        )
        .await,
        "blocked",
    );
    tokio::time::sleep(Duration::from_millis(600)).await;
    let outcome = call(&s, "robot_get_action", json!({"command_id":"recover-1"})).await;
    assert_eq!(
        outcome["result"]["structuredContent"]["outcome"]["state"], "completed",
        "{outcome}"
    );
    ok(&call(&s, "robot_recover_bumper", request).await);
    assert_eq!(
        log.lock()
            .unwrap()
            .iter()
            .filter(|l| l.starts_with("DRIVE "))
            .count(),
        1
    );
    assert_eq!(
        call(&s, "robot_observe", json!({})).await["result"]["structuredContent"]["blocked"],
        true
    );
}

#[tokio::test]
async fn recovery_validates_shape_duration_and_operator_budget() {
    let (s, log) = rig(GRANT).await;
    sensors(&s, true, false, false).await;
    for args in [
        json!({}),
        json!({"command_id":""}),
        json!({"command_id":"x","duration_ms":0}),
        json!({"command_id":"x","duration_ms":501}),
        json!({"command_id":"x","linear_mps":0.1}),
        json!({"command_id":"x","angular_radps":1}),
        json!(null),
    ] {
        let r = call(&s, "robot_recover_bumper", args).await;
        assert_eq!(r["result"]["isError"], true, "{r}");
    }
    assert!(!log.lock().unwrap().iter().any(|l| l.starts_with("DRIVE ")));
    let (s, log) = rig("mcp,0.05,1.0,2000,10000,600").await;
    sensors(&s, true, false, false).await;
    error(
        &call(&s, "robot_recover_bumper", json!({"command_id":"cap"})).await,
        "ceiling",
    );
    assert!(!log.lock().unwrap().iter().any(|l| l.starts_with("DRIVE ")));
    let (s, log) = rig("mcp,0.2,1.0,2000,200,600").await;
    sensors(&s, true, false, false).await;
    error(
        &call(&s, "robot_recover_bumper", json!({"command_id":"budget"})).await,
        "budget",
    );
    assert!(!log.lock().unwrap().iter().any(|l| l.starts_with("DRIVE ")));
}

#[tokio::test]
async fn recovery_rejects_missing_stale_cliff_drop_and_prior_floor_hazard() {
    let (s, log) = rig(GRANT).await;
    s.2.send(bumper(true)).await.unwrap();
    tokio::time::sleep(Duration::from_millis(100)).await;
    error(
        &call(&s, "robot_recover_bumper", json!({"command_id":"missing"})).await,
        "sensor",
    );
    sensors(&s, true, false, false).await;
    tokio::time::sleep(Duration::from_millis(1100)).await;
    error(
        &call(&s, "robot_recover_bumper", json!({"command_id":"stale"})).await,
        "stale",
    );
    sensors(&s, true, true, false).await;
    error(
        &call(&s, "robot_recover_bumper", json!({"command_id":"cliff"})).await,
        "cliff",
    );
    sensors(&s, true, false, true).await;
    error(
        &call(&s, "robot_recover_bumper", json!({"command_id":"drop"})).await,
        "drop",
    );
    // A later bumper edge cannot hide a cliff/drop latch, even if levels clear.
    sensors(&s, false, false, false).await;
    sensors(&s, true, false, false).await;
    error(
        &call(&s, "robot_recover_bumper", json!({"command_id":"masked"})).await,
        "hazard",
    );
    assert!(!log.lock().unwrap().iter().any(|l| l.starts_with("DRIVE ")));
}

#[tokio::test]
async fn recovery_new_floor_hazard_or_release_cancels_supervisor() {
    for trigger in ["cliff", "drop", "release", "stop"] {
        let (s, log) = rig(GRANT).await;
        sensors(&s, true, false, false).await;
        ok(&call(
            &s,
            "robot_recover_bumper",
            json!({"command_id":"cancel","duration_ms":500}),
        )
        .await);
        match trigger {
            "cliff" => sensors(&s, true, true, false).await,
            "drop" => sensors(&s, true, false, true).await,
            "release" => ok(&call(&s, "robot_release_control", json!({})).await),
            "stop" => ok(&call(&s, "robot_stop", json!({})).await),
            _ => unreachable!(),
        }
        stopped(&log).await;
        let action = call(&s, "robot_get_action", json!({"command_id":"cancel"})).await;
        assert_eq!(
            action["result"]["structuredContent"]["outcome"]["state"], "stopped",
            "{trigger}: {action}"
        );
    }
}

#[tokio::test]
async fn recovery_cumulative_latch_budget_survives_release() {
    let (s, _) = rig(GRANT).await;
    sensors(&s, true, false, false).await;
    for n in 0..2 {
        sensors(&s, true, false, false).await;
        ok(&call(
            &s,
            "robot_recover_bumper",
            json!({"command_id":format!("pulse-{n}"),"duration_ms":500}),
        )
        .await);
        tokio::time::sleep(Duration::from_millis(600)).await;
        ok(&call(&s, "robot_release_control", json!({})).await);
    }
    sensors(&s, true, false, false).await;
    error(
        &call(&s, "robot_recover_bumper", json!({"command_id":"too-far"})).await,
        "budget",
    );
    error(
        &call(&s, "robot_recover_bumper", json!({"command_id":"too-far"})).await,
        "budget",
    );
}

#[tokio::test]
async fn recovery_reset_and_reverse_never_steal_panel_owner() {
    let (s, log) = rig(GRANT).await;
    let (mut ws, _) = connect_async(format!("ws://127.0.0.1:{}/ws", s.1))
        .await
        .unwrap();
    ws.send(Message::Text(json!({"v":0.0,"w":0.0}).to_string().into()))
        .await
        .unwrap();
    tokio::time::sleep(Duration::from_millis(50)).await;
    sensors(&s, true, false, false).await;
    error(&call(&s, "robot_reset_hazards", json!({})).await, "busy");
    error(
        &call(&s, "robot_recover_bumper", json!({"command_id":"steal"})).await,
        "busy",
    );
    assert!(!log.lock().unwrap().iter().any(|l| l.starts_with("DRIVE ")));
}

#[tokio::test]
async fn recovery_invalid_floor_reading_is_not_clear_floor() {
    for invalid in [Vec::new(), vec![f32::NAN], vec![-1.0]] {
        let (s, log) = rig(GRANT).await;
        sensors(&s, true, false, false).await;
        let mut payload = Vec::new();
        // IR voltages precede ranges in the vendor wire format.
        payload.extend([0_u8; 8]);
        for _ in 0..2 {
            payload.extend((invalid.len() as u32).to_be_bytes());
        }
        for v in &invalid {
            payload.extend(v.to_be_bytes());
        }
        s.2.send(harness::packet(player_bridge::IR_CODE, 1, 1, 10., &payload))
            .await
            .unwrap();
        tokio::time::sleep(Duration::from_millis(100)).await;
        error(
            &call(
                &s,
                "robot_recover_bumper",
                json!({"command_id":"invalid-floor"}),
            )
            .await,
            "sensor",
        );
        assert!(!log.lock().unwrap().iter().any(|l| l.starts_with("DRIVE ")));
    }
}

#[tokio::test]
async fn recovery_stop_is_not_delayed_by_pending_drive_ack() {
    let (address, log) = supervisor_delayed(Duration::from_millis(700)).await;
    let s = harness::start_env_supervisor(None, GRANT, (&address, "")).await;
    tokio::time::sleep(Duration::from_millis(800)).await;
    sensors(&s, true, false, false).await;
    let port = s.1;
    let pending = tokio::spawn(async move {
        rpc_json(port, json!({"jsonrpc":"2.0","id":1,"method":"tools/call","params":{
            "name":"robot_recover_bumper","arguments":{"command_id":"late-ack","duration_ms":500}}}), None).await
    });
    tokio::time::timeout(Duration::from_secs(3), async {
        while !log.lock().unwrap().iter().any(|l| l.starts_with("DRIVE ")) {
            tokio::time::sleep(Duration::from_millis(5)).await;
        }
    })
    .await
    .unwrap();
    sensors(&s, true, true, false).await;
    tokio::time::timeout(Duration::from_millis(250), stopped(&log))
        .await
        .expect("STOP must not wait for the delayed DRIVING reply");
    assert_eq!(pending.await.unwrap()["result"]["isError"], true);
}

#[tokio::test]
async fn recovery_reset_cannot_clear_known_hazard_from_malformed_frame() {
    for hazard in ["bumper", "cliff", "drop"] {
        let (s, _) = rig(GRANT).await;
        sensors(&s, hazard == "bumper", hazard == "cliff", hazard == "drop").await;
        let invalid = if hazard == "bumper" {
            harness::packet(player_bridge::BUMPER_CODE, 0, 1, 10., &[0; 8])
        } else {
            harness::packet(
                player_bridge::IR_CODE,
                if hazard == "cliff" { 1 } else { 2 },
                1,
                10.,
                &[0; 16],
            )
        };
        s.2.send(invalid).await.unwrap();
        tokio::time::sleep(Duration::from_millis(100)).await;
        error(&call(&s, "robot_reset_hazards", json!({})).await, hazard);
    }
}

#[tokio::test]
async fn recovery_invalid_bumper_frame_is_not_fresh_contact_data() {
    let (s, log) = rig(GRANT).await;
    sensors(&s, true, false, false).await;
    s.2.send(harness::packet(
        player_bridge::BUMPER_CODE,
        0,
        1,
        10.,
        &[0; 8],
    ))
    .await
    .unwrap();
    tokio::time::sleep(Duration::from_millis(100)).await;
    error(
        &call(
            &s,
            "robot_recover_bumper",
            json!({"command_id":"missing-bumper"}),
        )
        .await,
        "sensor",
    );
    assert!(!log.lock().unwrap().iter().any(|l| l.starts_with("DRIVE ")));
}

#[tokio::test]
async fn recovery_sensor_expiry_during_pulse_sends_stop() {
    let (s, log) = rig(GRANT).await;
    sensors(&s, true, false, false).await;
    tokio::time::sleep(Duration::from_millis(800)).await;
    ok(&call(
        &s,
        "robot_recover_bumper",
        json!({"command_id":"aging","duration_ms":500}),
    )
    .await);
    stopped(&log).await;
    assert_eq!(
        call(&s, "robot_get_action", json!({"command_id":"aging"})).await["result"]
            ["structuredContent"]["outcome"]["state"],
        "stopped"
    );
}

#[tokio::test]
async fn recovery_reset_expired_grant_and_no_supervisor_refused() {
    let (s, log) = rig("mcp,0.2,1.0,2000,10000,1").await;
    tokio::time::sleep(Duration::from_millis(1100)).await;
    error(&call(&s, "robot_reset_hazards", json!({})).await, "grant");
    sensors(&s, true, false, false).await;
    error(
        &call(&s, "robot_recover_bumper", json!({"command_id":"expired"})).await,
        "grant",
    );
    assert!(!log.lock().unwrap().iter().any(|l| l.starts_with("DRIVE ")));
    let mut s = harness::start_env(None, GRANT).await;
    // The existing harness provides an endpoint readiness loop.
    tokio::time::timeout(Duration::from_secs(5), async {
        loop {
            if rpc(
                s.1,
                json!({"jsonrpc":"2.0","id":0,"method":"tools/list"}),
                None,
            )
            .await
            .is_some()
            {
                break;
            }
            tokio::time::sleep(Duration::from_millis(20)).await;
        }
    })
    .await
    .unwrap();
    sensors(&s, true, false, false).await;
    error(
        &call(
            &s,
            "robot_recover_bumper",
            json!({"command_id":"no-local-deadline"}),
        )
        .await,
        "supervisor",
    );
    let _ = &mut s;
}

#[tokio::test]
async fn recovery_panel_pulse_uses_same_supervisor_gate_and_disconnect_stops() {
    let (s, log) = rig(GRANT).await;
    sensors(&s, true, false, false).await;
    let (mut ws, _) = connect_async(format!("ws://127.0.0.1:{}/ws", s.1))
        .await
        .unwrap();
    ws.send(Message::Text(
        json!({"type":"recover_bumper","command_id":"panel-pulse","duration_ms":500})
            .to_string()
            .into(),
    ))
    .await
    .unwrap();
    let reply = tokio::time::timeout(Duration::from_secs(2), async {
        loop {
            let msg = ws.next().await.unwrap().unwrap();
            if let Message::Text(text) = msg {
                let v: Value = serde_json::from_str(&text).unwrap();
                if v["type"] == "recovery" {
                    break v;
                }
            }
        }
    })
    .await
    .expect("panel recovery reply");
    assert_eq!(reply["data"]["isError"], false, "{reply}");
    assert!(log.lock().unwrap().iter().any(|l| l == "DRIVE -0.1 0 500"));
    ws.close(None).await.unwrap();
    stopped(&log).await;
}
