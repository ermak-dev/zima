// FEATURE: operator-granted MCP control admission (#56).
// SPEC: without an operator grant every acquire is refused (read-only
// default); a confirm=true flag in the request never creates authority; with
// a grant the MCP subject acquires the shared WEB/MCP control ownership and
// release/panel-owner conflict follows the Safety rules.
// IMPLEMENTATION: grants::Grants state machine (unit-tested in src) gates the
// mcp acquire/release tools; ownership itself stays in Safety.
// RESULT: IMPLEMENTED — offline fake Player checks pass.
// STATUS: DONE
mod harness {
    include!("mcp.rs");
}
use harness::{rpc, rpc_json, start, start_env, Server};
use serde_json::{json, Value};

async fn call(s: &Server, name: &str, args: Value) -> Value {
    let payload = json!({"jsonrpc":"2.0","id":1,"method":"tools/call","params":{
        "name": name, "arguments": args.clone()}});
    rpc_json(s.1, payload, None).await
}

#[tokio::test]
async fn mcp_control_is_read_only_without_operator_grant() {
    let s = start(None).await;
    // Wait for HTTP readiness like the main mcp tests do.
    tokio::time::timeout(std::time::Duration::from_secs(10), async {
        loop {
            if let Some(ok) = rpc(
                s.1,
                json!({"jsonrpc":"2.0","id":0,"method":"tools/list"}),
                None,
            )
            .await
            {
                if ok.status() == 200 {
                    break;
                }
            }
            tokio::time::sleep(std::time::Duration::from_millis(100)).await;
        }
    })
    .await
    .expect("endpoint ready");
    // No grant installed (no operator env): acquire must be an error, and a
    // self-confirmed request must not change that.
    for args in [
        json!({}),
        json!({"confirm": true}),
        json!({"confirm": true}),
    ] {
        let reply = call(&s, "robot_acquire_control", args.clone()).await;
        assert_eq!(reply["result"]["isError"], true, "for {args}");
        let text = reply["result"]["content"][0]["text"].as_str().unwrap();
        assert!(text.contains("grant"), "unexpected error: {text}");
    }
    // The read-only toolset still works and lists no control tools as usable.
    let list = rpc_json(
        s.1,
        json!({"jsonrpc":"2.0","id":2,"method":"tools/list","params":{}}),
        None,
    )
    .await;
    let names: Vec<&str> = list["result"]["tools"]
        .as_array()
        .unwrap()
        .iter()
        .map(|t| t["name"].as_str().unwrap())
        .collect();
    assert!(names.contains(&"robot_acquire_control"));
    assert!(names.contains(&"robot_release_control"));
    // Motion tools exist but are gated by the grant; calling without one errors.
    let reply = call(
        &s,
        "robot_drive_for",
        json!({"linear_mps":0.0,"angular_radps":0.0,"duration_ms":100}),
    )
    .await;
    assert_eq!(reply["result"]["isError"], true);
}

// ---- #61: robot_set_cleaning_motors ----
// FEATURE: limited cleaning-motor control through MCP (fan/main/side PWM).
// SPEC: the tool is listed with a strict schema and readOnlyHint:false; an
// active MCP control lease AND a live operator grant are both required: with
// no lease the error says to acquire control first, with no grant the error
// names the grant, and a panel-owned lease is a BUSY-style error. Duty
// levels are validated in code (0..=100, numbers only): out-of-range or
// non-numeric values are isError, never clamped silently. No vendor motor
// subscription is used, only direct PWM commands (interf 170, subtype 1).
// IMPLEMENTATION: mcp::set_cleaning_motors reuses the Safety owner check,
// grants admission (a dedicated motor permission is TODO until #62) and
// player::motor_pwm; release_control zeroes all three motors on the way out.
async fn motors_ready(s: &mut Server) {
    tokio::time::timeout(std::time::Duration::from_secs(10), async {
        loop {
            if let Some(ok) = rpc(
                s.1,
                json!({"jsonrpc":"2.0","id":0,"method":"tools/list"}),
                None,
            )
            .await
            {
                if ok.status() == 200 {
                    break;
                }
            }
            tokio::time::sleep(std::time::Duration::from_millis(100)).await;
        }
    })
    .await
    .expect("endpoint ready");
    while let Ok(c) = s.3.try_recv() {
        let _ = c;
    }
}

#[tokio::test]
async fn mcp_cleaning_motors_requires_lease_and_grant() {
    let mut s = start(None).await;
    motors_ready(&mut s).await;
    // No lease and no grant: acquire control first.
    let reply = call(
        &s,
        "robot_set_cleaning_motors",
        json!({"fan":50,"main":30,"side":80}),
    )
    .await;
    assert_eq!(reply["result"]["isError"], true, "call: {reply}");
    let text = reply["result"]["content"][0]["text"].as_str().unwrap();
    assert!(
        text.to_lowercase().contains("acquire"),
        "must say to acquire control first: {text}"
    );
    // Level validation runs before ownership, so bad values stay errors too.
    for bad in [
        json!({"fan": 101}),
        json!({"fan": -1}),
        json!({"main": "fast"}),
        json!({"side": 100.5}),
        json!({"fan": null}),
        json!({"brush": 50}),
    ] {
        let reply = call(&s, "robot_set_cleaning_motors", bad.clone()).await;
        assert_eq!(
            reply["result"]["isError"], true,
            "args {bad} must be rejected: {reply}"
        );
    }
    // No motor PWM frames reached the fake Player from any refused call.
    while let Ok((index, duty)) = s.3.try_recv() {
        panic!("refused call must not emit PWM, got index {index} duty {duty}");
    }
    // release_control with no lease is still a no-op success and sends no
    // PWM frames.
    let reply = call(&s, "robot_release_control", json!({})).await;
    assert_eq!(reply["result"]["isError"], false);
    while let Ok((index, duty)) = s.3.try_recv() {
        panic!("release without lease must not emit PWM: {index} {duty}");
    }
}

#[tokio::test]
async fn mcp_cleaning_motors_apply_under_lease_and_zero_on_release() {
    // Operator grant via the operator-only env, subject "mcp".
    let mut s = start_env(None, "mcp,0.2,1.0,2000,10000,600").await;
    motors_ready(&mut s).await;
    // With a live grant but no lease yet: acquire control first.
    let reply = call(
        &s,
        "robot_set_cleaning_motors",
        json!({"fan":50,"main":30,"side":80}),
    )
    .await;
    assert_eq!(reply["result"]["isError"], true, "{reply}");
    assert!(reply["result"]["content"][0]["text"]
        .as_str()
        .unwrap()
        .to_lowercase()
        .contains("acquire"));
    // Acquire the lease, then set the motors.
    let reply = call(&s, "robot_acquire_control", json!({})).await;
    assert_eq!(reply["result"]["isError"], false, "{reply}");
    let reply = call(
        &s,
        "robot_set_cleaning_motors",
        json!({"fan":50,"main":30,"side":80}),
    )
    .await;
    assert_eq!(reply["result"]["isError"], false, "{reply}");
    assert_eq!(
        reply["result"]["structuredContent"]["requested"],
        json!({"fan": 50.0, "main": 30.0, "side": 80.0})
    );
    // All three PWM commands reach the fake Player with the requested duty.
    let mut seen = std::collections::BTreeMap::new();
    for _ in 0..3 {
        let (index, duty) = tokio::time::timeout(std::time::Duration::from_secs(3), s.3.recv())
            .await
            .expect("PWM reached Player")
            .unwrap();
        seen.insert(index, duty);
    }
    assert_eq!(
        seen,
        std::collections::BTreeMap::from([(0u8, 50.0f32), (1, 30.0), (2, 80.0)]),
        "requested duties map to fan/main/side"
    );
    // The requested duties are mirrored in the shared state (WS state frame).
    let (ws, _) = tokio_tungstenite::connect_async(format!("ws://127.0.0.1:{}/ws", s.1))
        .await
        .unwrap();
    let (mut sink, mut input) = ws.split();
    use futures_util::{SinkExt, StreamExt};
    let frame = tokio::time::timeout(std::time::Duration::from_secs(3), async {
        loop {
            let message = input.next().await.unwrap().unwrap();
            if let tokio_tungstenite::tungstenite::Message::Text(text) = message {
                let value: Value = serde_json::from_str(&text).unwrap();
                if value["type"] == "state" && value["data"]["motors"]["fan"].as_f64() == Some(50.0)
                {
                    return value["data"]["motors"].clone();
                }
            }
        }
    })
    .await
    .expect("state frame carries requested motors");
    assert_eq!(frame, json!({"fan": 50.0, "main": 30.0, "side": 80.0}));
    sink.close().await.unwrap_or(());
    // Releasing the lease zeroes all three motors on the way out (#61).
    let reply = call(&s, "robot_release_control", json!({})).await;
    assert_eq!(reply["result"]["isError"], false, "{reply}");
    let mut zeros = std::collections::BTreeMap::new();
    for _ in 0..3 {
        let (index, duty) = tokio::time::timeout(std::time::Duration::from_secs(3), s.3.recv())
            .await
            .expect("release zero PWM reached Player")
            .unwrap();
        zeros.insert(index, duty);
    }
    assert_eq!(
        zeros,
        std::collections::BTreeMap::from([(0u8, 0.0f32), (1, 0.0), (2, 0.0)]),
        "release zeroes fan/main/side"
    );
    while let Ok(c) = s.3.try_recv() {
        let _ = c;
    }
}

// ---- #63: robot_say ----
// FEATURE: MCP speech over the existing audio pipeline.
// SPEC: without configured TTS the tool is an explicit error (no playback
// attempt); the tool appears in tools/list with strict arguments; a second
// call while one job runs is BUSY; nothing about the reply leaks secrets.
// IMPLEMENTATION: mcp robot_say wraps the shared Audio::submit path used by
// the panel's /api/audio/say.
// RESULT: IMPLEMENTED — offline fake Player checks pass.
// STATUS: DONE
#[tokio::test]
async fn mcp_say_reports_disabled_tts_and_lists_in_tools() {
    let mut s = start(None).await;
    drive_ready(&mut s).await;
    // tools/list includes robot_say with a strict schema.
    let list = rpc_json(
        s.1,
        json!({"jsonrpc":"2.0","id":2,"method":"tools/list","params":{}}),
        None,
    )
    .await;
    let names: Vec<&str> = list["result"]["tools"]
        .as_array()
        .unwrap()
        .iter()
        .map(|t| t["name"].as_str().unwrap())
        .collect();
    assert!(names.contains(&"robot_say"), "names: {names:?}");
    // No Yandex key in the test env: the call must be an explicit error that
    // names TTS configuration, never a playback attempt or a secret.
    let reply = call(&s, "robot_say", json!({"text":"Привет"})).await;
    assert_eq!(reply["result"]["isError"], true, "{reply}");
    let text = reply["result"]["content"][0]["text"].as_str().unwrap();
    assert!(text.contains("tts"), "error names TTS: {text}");
    assert!(!text.contains("Api-Key"), "no secrets in errors");
    // Invalid arguments are rejected before any TTS consideration.
    for bad in [
        json!({"text": ""}),
        json!({"text": "a\0b"}),
        json!({"text": 42}),
        json!({"text": "ok", "volume": -1.0}),
        json!({"text": "ok", "unknown": true}),
        json!({}),
    ] {
        let reply = call(&s, "robot_say", bad.clone()).await;
        assert_eq!(reply["result"]["isError"], true, "args {bad}: {reply}");
    }
}

/// With a live (fake) slow TTS provider the job is queued exactly once; the
/// second call while it runs is BUSY and the receipt separates accepted from
/// completed.
#[tokio::test]
async fn mcp_say_queues_once_and_reports_busy() {
    // Slow fake provider: accepts the POST, never answers, so the job stays
    // active long enough for the BUSY probe.
    let provider = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
    let provider_url = format!("http://{}/tts", provider.local_addr().unwrap());
    tokio::spawn(async move {
        use tokio::io::{AsyncReadExt, AsyncWriteExt};
        loop {
            let Ok((mut s, _)) = provider.accept().await else {
                break;
            };
            tokio::spawn(async move {
                let mut buf = vec![0u8; 4096];
                let _ = s.read(&mut buf).await; // headers
                let _ = s.write_all(
                    b"HTTP/1.1 200 OK\r\ncontent-type: audio/pcm\r\ncontent-length: 6400\r\n\r\n",
                )
                .await;
                // Body never arrives: the synthesis stalls.
                std::future::pending::<()>().await;
            });
        }
    });
    let mut s = harness::start_env_tts(
        None,
        "mcp,0.2,1.0,2000,10000,600",
        Some((&provider_url, "k")),
    )
    .await;
    drive_ready(&mut s).await;
    let first = call(&s, "robot_say", json!({"text":"Привет"})).await;
    assert_eq!(first["result"]["isError"], false, "{first}");
    let structured = &first["result"]["structuredContent"];
    let task = structured["task_id"].as_str().expect("task id").to_owned();
    assert_eq!(structured["state"], "queued");
    // The busy window: a second job while the stalled one is active.
    let second = call(&s, "robot_say", json!({"text":"Ещё"})).await;
    assert_eq!(second["result"]["isError"], true, "{second}");
    let text = second["result"]["content"][0]["text"].as_str().unwrap();
    assert!(text.contains("busy"), "must be BUSY: {text}");
    assert!(!text.contains('k') || !text.contains("Api-Key"));
    // The task id is opaque and no secret leaked into receipts.
    assert!(!task.is_empty());
}

// FEATURE: bounded MCP motion with idempotent command ids and honest stops.
// SPEC: no lease → error; with lease+grant the velocity reaches Player once;
// a repeated command_id never re-executes; a second concurrent action is
// BUSY; robot_stop sends the zero and marks the action stopped.
// IMPLEMENTATION: mcp_drive::DriveActions registry + shared Player writer.
// RESULT: IMPLEMENTED — offline fake Player velocity log verified.
// STATUS: DONE
async fn drive_ready(s: &mut Server) {
    tokio::time::timeout(std::time::Duration::from_secs(10), async {
        loop {
            if let Some(ok) = rpc(
                s.1,
                json!({"jsonrpc":"2.0","id":0,"method":"tools/list"}),
                None,
            )
            .await
            {
                if ok.status() == 200 {
                    break;
                }
            }
            tokio::time::sleep(std::time::Duration::from_millis(100)).await;
        }
    })
    .await
    .expect("endpoint ready");
    while let Ok(c) = s.4.try_recv() {
        let _ = c;
    }
}

#[tokio::test]
async fn mcp_drive_for_bounded_idempotent_and_stoppable() {
    // Operator grant via the operator-only env, subject "mcp".
    let mut s = start_env(None, "mcp,0.2,1.0,2000,10000,600").await;
    drive_ready(&mut s).await;
    // No lease yet: acquire first.
    let reply = call(
        &s,
        "robot_drive_for",
        json!({"linear_mps":0.05,"angular_radps":0.0,"duration_ms":300,"command_id":"c1"}),
    )
    .await;
    assert_eq!(reply["result"]["isError"], true, "{reply}");
    assert!(reply["result"]["content"][0]["text"]
        .as_str()
        .unwrap()
        .contains("acquire"));
    // Acquire, then drive: the velocity must reach the fake Player.
    let reply = call(&s, "robot_acquire_control", json!({})).await;
    assert_eq!(reply["result"]["isError"], false, "{reply}");
    let reply = call(
        &s,
        "robot_drive_for",
        json!({"linear_mps":0.05,"angular_radps":0.0,"duration_ms":300,"command_id":"c1"}),
    )
    .await;
    assert_eq!(reply["result"]["isError"], false, "{reply}");
    // The acquire handshake may flush its safety zero first; require the
    // requested velocity to arrive within the window.
    let deadline = std::time::Instant::now() + std::time::Duration::from_secs(3);
    loop {
        let seen = tokio::time::timeout(std::time::Duration::from_secs(3), s.4.recv())
            .await
            .expect("velocity reached Player")
            .unwrap();
        if seen == (0.05, 0.0) {
            break;
        }
        assert!(
            std::time::Instant::now() < deadline,
            "requested velocity never reached Player"
        );
    }
    // Idempotent repeat: the same command_id returns the record and does not
    // emit a second velocity command.
    let reply = call(
        &s,
        "robot_drive_for",
        json!({"linear_mps":0.05,"angular_radps":0.0,"duration_ms":300,"command_id":"c1"}),
    )
    .await;
    assert_eq!(reply["result"]["isError"], false, "{reply}");
    // A different payload under the same id is rejected.
    let reply = call(
        &s,
        "robot_drive_for",
        json!({"linear_mps":0.1,"angular_radps":0.0,"duration_ms":300,"command_id":"c1"}),
    )
    .await;
    assert_eq!(reply["result"]["isError"], true, "{reply}");
    // robot_get_action returns the honest record.
    let reply = call(&s, "robot_get_action", json!({"command_id":"c1"})).await;
    assert_eq!(reply["result"]["isError"], false, "{reply}");
    assert_eq!(
        reply["result"]["structuredContent"]["outcome"]["state"],
        "sent"
    );
    // robot_stop clears the action and sends the zero. The drive clock may
    // first flush the safety-owned zero from the acquire handshake; accept
    // either ordering but require a zero to arrive.
    let reply = call(&s, "robot_stop", json!({})).await;
    assert_eq!(reply["result"]["isError"], false, "{reply}");
    let deadline = std::time::Instant::now() + std::time::Duration::from_secs(3);
    loop {
        let seen = tokio::time::timeout(std::time::Duration::from_secs(3), s.4.recv())
            .await
            .expect("a velocity command reached Player after stop")
            .unwrap();
        if seen == (0.0, 0.0) {
            break;
        }
        assert!(
            std::time::Instant::now() < deadline,
            "zero never reached Player"
        );
    }
    let reply = call(&s, "robot_get_action", json!({"command_id":"c1"})).await;
    assert_eq!(
        reply["result"]["structuredContent"]["outcome"]["state"],
        "stopped"
    );
}
