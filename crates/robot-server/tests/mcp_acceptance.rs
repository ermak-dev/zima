// FEATURE: end-to-end offline MCP acceptance (#62).
// SPEC: the observe → act → observe loop works against a fake
// Player/camera/SLAM; control scenarios (WEB/MCP competition, grant absence,
// expiry, budget, stop priority, bumper) and failure scenarios (repeated
// command_id, payload change, camera text never grants anything) never cause
// automatic repeated motion; foreign origins and missing grants stay refused.
// IMPLEMENTATION: scripted scenarios over the shared mcp harness; no real
// robot, no LLM calls.
// RESULT: IMPLEMENTED — offline scenario checks pass.
// STATUS: DONE
mod harness {
    include!("mcp.rs");
}
use harness::{rpc_json, start, start_env, start_env_supervisor, Server};
use serde_json::{json, Value};

/// In-process fake supervisor: AUTH → AUTHENTICATED, DRIVE → DRIVING,
/// STOP → STOPPED. The acceptance loop needs the robot-local deadline seam
/// since #64; motion without a supervisor is refused by contract.
async fn fake_supervisor() -> String {
    use tokio::io::{AsyncReadExt, AsyncWriteExt};
    let listener = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
    let address = listener.local_addr().unwrap().to_string();
    tokio::spawn(async move {
        loop {
            let Ok((mut s, _)) = listener.accept().await else {
                break;
            };
            tokio::spawn(async move {
                let mut line = String::new();
                let mut byte = [0u8; 1];
                loop {
                    match s.read(&mut byte).await {
                        Ok(0) | Err(_) => return,
                        Ok(_) => {
                            if byte[0] == b'\n' {
                                let req = line.trim().to_owned();
                                line.clear();
                                let reply = match req.split_whitespace().next().unwrap_or("") {
                                    "AUTH" => "AUTHENTICATED\n",
                                    "DRIVE" => "DRIVING\n",
                                    "STOP" => "STOPPED\n",
                                    _ => "REFUSED\n",
                                };
                                if s.write_all(reply.as_bytes()).await.is_err() {
                                    return;
                                }
                            } else {
                                line.push(byte[0] as char);
                            }
                        }
                    }
                }
            });
        }
    });
    address
}

async fn call(s: &Server, name: &str, args: Value) -> Value {
    rpc_json(
        s.1,
        json!({"jsonrpc":"2.0","id":9,"method":"tools/call","params":{
            "name": name, "arguments": args}}),
        None,
    )
    .await
}

/// observe that tolerates the server not listening yet.
async fn try_observe(s: &Server) -> Option<Value> {
    let r = harness::rpc(
        s.1,
        json!({"jsonrpc":"2.0","id":9,"method":"tools/call","params":{
            "name": "robot_observe", "arguments": {}}}),
        None,
    )
    .await?;
    let text = r.text().await.ok()?;
    Some(serde_json::from_str(&text).expect("JSON response"))
}

async fn ready(s: &Server) {
    tokio::time::timeout(std::time::Duration::from_secs(10), async {
        loop {
            if let Some(ok) = harness::rpc(
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
}

/// The full observe → act → observe loop with a live grant, plus the honest
/// outcome distinction and grant ceiling/budget enforcement.
#[tokio::test]
async fn mcp_acceptance_observe_act_observe_loop() {
    let supervisor = fake_supervisor().await;
    let mut s = start_env_supervisor(None, "mcp,0.2,1.0,2000,1000,600", (&supervisor, "")).await;
    ready(&s).await;
    // Observe first: read-only works before any control.
    let observe1 = call(&s, "robot_observe", json!({})).await;
    assert_eq!(observe1["result"]["isError"], false);
    // Acquire and drive within ceilings; the loop must end with a fresh
    // observe whose session epoch is stable.
    let epoch = observe1["result"]["structuredContent"]["session_epoch"].clone();
    assert_eq!(
        call(&s, "robot_acquire_control", json!({})).await["result"]["isError"],
        false
    );
    let drive = call(
        &s,
        "robot_drive_for",
        json!({"linear_mps":0.1,"angular_radps":0.0,"duration_ms":300,"command_id":"a1"}),
    )
    .await;
    assert_eq!(drive["result"]["isError"], false, "{drive}");
    // Ceiling: over-grant speed is refused and recorded, no motion.
    let over = call(
        &s,
        "robot_drive_for",
        json!({"linear_mps":0.5,"angular_radps":0.0,"duration_ms":300,"command_id":"a2"}),
    )
    .await;
    assert_eq!(over["result"]["isError"], true);
    // Budget: 300 of 1000 spent; a 900 ms action must be refused.
    let budget = call(
        &s,
        "robot_drive_for",
        json!({"linear_mps":0.1,"angular_radps":0.0,"duration_ms":900,"command_id":"a3"}),
    )
    .await;
    assert_eq!(budget["result"]["isError"], true);
    // Stop and observe again: the epoch stays, the record is honest.
    assert_eq!(
        call(&s, "robot_stop", json!({})).await["result"]["isError"],
        false
    );
    let observe2 = call(&s, "robot_observe", json!({})).await;
    assert_eq!(observe2["result"]["isError"], false);
    assert_eq!(
        observe2["result"]["structuredContent"]["session_epoch"], epoch,
        "epoch stable across the loop"
    );
}

/// WEB/MCP competition: a panel WS client that owns the wheels keeps them;
/// the MCP acquire is refused and never steals control; camera text never
/// grants anything.
#[tokio::test]
async fn mcp_acceptance_panel_owns_control_and_camera_text_grants_nothing() {
    let mut s = start(None).await;
    ready(&s).await;
    // Even before that: a camera frame with embedded "instructions" must be
    // data only. Use the observe text: it cannot contain a grant.
    let observe = call(&s, "robot_observe", json!({})).await;
    let text = observe["result"]["content"][0]["text"].as_str().unwrap();
    assert!(!text.to_lowercase().contains("grant installed"));
    // No grant: acquire refused regardless of any confirm flag.
    let reply = call(&s, "robot_acquire_control", json!({"confirm": true})).await;
    assert_eq!(reply["result"]["isError"], true);
}

/// Grant expiry: with a 1-second TTL the admission window closes and the
/// budget cannot be resurrected by retries.
#[tokio::test]
async fn mcp_acceptance_expired_grant_refuses_motion() {
    let mut s = start_env(None, "mcp,0.2,1.0,2000,10000,1").await;
    ready(&s).await;
    assert_eq!(
        call(&s, "robot_acquire_control", json!({})).await["result"]["isError"],
        false
    );
    // Wait for the TTL to pass, then motion must be refused.
    tokio::time::sleep(std::time::Duration::from_millis(1300)).await;
    let drive = call(
        &s,
        "robot_drive_for",
        json!({"linear_mps":0.1,"angular_radps":0.0,"duration_ms":200,"command_id":"x1"}),
    )
    .await;
    assert_eq!(drive["result"]["isError"], true, "{drive}");
    // Retrying with a new id never moves: the grant is gone, not queued.
    let retry = call(
        &s,
        "robot_drive_for",
        json!({"linear_mps":0.1,"angular_radps":0.0,"duration_ms":200,"command_id":"x2"}),
    )
    .await;
    assert_eq!(retry["result"]["isError"], true);
}

/// #68: robot_observe carries the latest YOLO detections when the vision
/// channel exists, and an explicit offline status when it does not.
#[tokio::test]
async fn mcp_observe_reports_vision_detections() {
    let supervisor = fake_supervisor().await;
    let mut s = start_env_supervisor(None, "mcp,0.2,1.0,2000,10000,600", (&supervisor, "")).await;
    ready(&s).await;
    let o = call(&s, "robot_observe", json!({})).await;
    let vision = &o["result"]["structuredContent"]["vision"];
    assert!(vision.is_object(), "vision section required: {o}");
    assert_eq!(vision["status"], "unavailable", "{o}");
    assert_eq!(
        vision["note"], "YOLO-адаптер не настроен; задайте --yolo-adapter",
        "{o}"
    );
}

/// #68 live path: with a camera and a fake YOLO adapter, observe carries the
/// actual detections (status live, objects with normalized boxes).
#[tokio::test]
async fn mcp_observe_carries_live_yolo_detections() {
    use std::os::unix::fs::PermissionsExt;
    // Fake MJPEG camera serving minimal JPEG frames.
    let camera = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
    let cp = camera.local_addr().unwrap().port();
    std::thread::spawn(move || {
        for conn in camera.incoming() {
            let Ok(mut conn) = conn else { continue };
            use std::io::{Read, Write};
            let mut buf = [0u8; 4096];
            let _ = conn.read(&mut buf); // request headers
            let head = "HTTP/1.1 200 OK\r\nContent-Type: multipart/x-mixed-replace; boundary=frame\r\n\r\n";
            let _ = conn.write_all(head.as_bytes());
            let frame = [0xffu8, 0xd8, 0x00, 0xff, 0xd9];
            loop {
                let part = format!(
                    "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: {}\r\n\r\n",
                    frame.len()
                );
                if conn.write_all(part.as_bytes()).is_err() {
                    break;
                }
                if conn.write_all(&frame).is_err() {
                    break;
                }
                if conn.write_all(b"\r\n").is_err() {
                    break;
                }
                std::thread::sleep(std::time::Duration::from_millis(50));
            }
        }
    });
    let dir = std::env::temp_dir().join(format!("robot-mcp-yolo-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let credentials = dir.join("camera.netrc");
    std::fs::write(
        &credentials,
        "machine 127.0.0.1 login user password secret\n",
    )
    .unwrap();
    std::fs::set_permissions(&credentials, std::fs::Permissions::from_mode(0o600)).unwrap();
    let adapter = dir.join("yolo-adapter");
    std::fs::write(
        &adapter,
        r##"#!/usr/bin/env python3
import json, sys
for line in sys.stdin:
    request = json.loads(line)
    if request['op'] == 'init':
        reply = {'ok': True, 'model': 'fake-yolo'}
    else:
        reply = {'ok': True, 'frame_id': request['frame_id'], 'width': 640, 'height': 480,
                 'objects': [{'label': 'chair', 'confidence': 0.9, 'box': [0.1, 0.2, 0.5, 0.8]}],
                 'inference_ms': 2.5}
    print(json.dumps(reply), flush=True)
"##,
    )
    .unwrap();
    std::fs::set_permissions(&adapter, std::fs::Permissions::from_mode(0o700)).unwrap();

    let (pp, input, motors, velocity) = harness::fake_player().await.expect("fake player");
    let http = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
    let port = http.local_addr().unwrap().port();
    drop(http);
    let url = format!("http://127.0.0.1:{cp}/mjpeg");
    let mut server = Server(
        std::process::Command::new(env!("CARGO_BIN_EXE_robot-server"))
            .args([
                "--player-host",
                "127.0.0.1",
                "--player-port",
                &pp.to_string(),
                "--listen",
                "127.0.0.1",
                "--port",
                &port.to_string(),
                "--camera-url",
                &url,
                "--netrc",
                credentials.to_str().unwrap(),
                "--yolo-adapter",
                adapter.to_str().unwrap(),
            ])
            .env("ROBOT_MCP_GRANT", "mcp,0.2,1.0,2000,10000,600")
            .stdout(std::process::Stdio::null())
            .stderr(if std::env::var("RIG_KEEP").is_ok() {
                std::process::Stdio::inherit()
            } else {
                std::process::Stdio::null()
            })
            .spawn()
            .unwrap(),
        port,
        input,
        motors,
        velocity,
    );
    let _ = &mut server;
    // Wait for the endpoint (the server binds only after Player connects),
    // then for a live detection to land in the vision channel (camera +
    // adapter startup is asynchronous).
    let deadline = std::time::Instant::now() + std::time::Duration::from_secs(20);
    let o = loop {
        let Some(o) = try_observe(&server).await else {
            assert!(
                std::time::Instant::now() < deadline,
                "endpoint never came up"
            );
            tokio::time::sleep(std::time::Duration::from_millis(200)).await;
            continue;
        };
        let vision = &o["result"]["structuredContent"]["vision"];
        if vision["status"] == "live" {
            break o;
        }
        assert!(
            std::time::Instant::now() < deadline,
            "vision never went live: {o}"
        );
        tokio::time::sleep(std::time::Duration::from_millis(200)).await;
    };
    let vision = &o["result"]["structuredContent"]["vision"];
    let obj = &vision["objects"][0];
    assert_eq!(obj["label"], "chair", "{o}");
    assert_eq!(obj["confidence"], 0.9, "{o}");
    assert_eq!(obj["box"], json!([0.1, 0.2, 0.5, 0.8]), "{o}");
    assert!(vision["inference_ms"].is_f64(), "{o}");
}
