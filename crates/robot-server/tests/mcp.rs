// FEATURE: read-only MCP endpoint beside the panel on the same server.
// SPEC: Streamable HTTP JSON-RPC at /mcp: initialize negotiates the protocol
// version, tools/list exposes schema'd read-only tools, tools/call returns
// structured results; a foreign Origin is rejected before any RPC runs; the
// endpoint never touches the mask, controller, or drive.
// IMPLEMENTATION: minimal JSON-RPC dispatcher on the existing axum router;
// state comes from the same watch channel as the panel; no LLM calls.
// RESULT: IMPLEMENTED — offline fake Player HTTP checks pass.
// STATUS: DONE
use player_bridge::*;
use serde_json::{json, Value};
use std::{
    path::PathBuf,
    process::{Child, Command, Stdio},
    sync::{Arc, Mutex},
    time::Duration,
};
use tokio::{
    io::{AsyncReadExt, AsyncWriteExt},
    net::{TcpListener, TcpSocket},
    sync::mpsc,
    time::timeout,
};

type Failure = Box<dyn std::error::Error + Send + Sync>;

/// A minimal JPEG with a real SOF0 header (640x480) for the fake camera:
/// SOI, APP0/JFIF, SOF0 (height, width), entropy bytes, EOI.
fn jpeg_640x480() -> Vec<u8> {
    let mut frame = vec![
        0xff, 0xd8, 0xff, 0xe0, 0x00, 0x10, b'J', b'F', b'I', b'F', 0x00, 0x01, 0x02, 0x00, 0x00,
        0x01, 0x00, 0x01, 0x00, 0x00, 0xff, 0xc0, 0x00, 0x11, 0x08, 0x01, 0xe0, 0x02, 0x80, 0x03,
        0x01, 0x22, 0x00, 0x02, 0x11, 0x01, 0x03, 0x11, 0x01,
    ];
    frame.extend(std::iter::repeat(0xab).take(512));
    frame.extend_from_slice(&[0xff, 0xd9]);
    frame
}

/// Minimal MJPEG upstream: one 200 multipart response with a single JPEG
/// frame per connection, then the connection is held open until the client
/// drops it.
async fn fake_camera() -> String {
    let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
    let port = listener.local_addr().unwrap().port();
    tokio::spawn(async move {
        while let Ok((mut socket, _)) = listener.accept().await {
            let frame = jpeg_640x480();
            let mut body = format!(
                "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: {}\r\n\r\n",
                frame.len()
            )
            .into_bytes();
            body.extend_from_slice(&frame);
            body.extend_from_slice(b"\r\n--frame\r\n");
            let mut req = [0u8; 2048];
            let _ = socket.read(&mut req).await;
            let _ = socket.write_all(
                b"HTTP/1.1 200 OK\r\nContent-Type: multipart/x-mixed-replace; boundary=frame\r\n\r\n",
            )
            .await;
            let _ = socket.write_all(&body).await;
            // Hold the connection open until the client drops it.
            let mut sink = [0u8; 64];
            let _ = socket.read(&mut sink).await;
            let _ = socket.shutdown().await;
        }
    });
    format!("http://127.0.0.1:{port}/mjpeg")
}

/// Minimal fake Player: banner, then ACK for every subscription request; the
/// injected `inbound` packets (SLAM scans for #58) are forwarded to the server
/// concurrently with the endless subscription drain, like the fuller fake in
/// tests/server.rs.
async fn fake_player() -> Result<
    (
        u16,
        mpsc::Sender<Vec<u8>>,
        mpsc::Receiver<(u8, f32)>,
        mpsc::Receiver<(f64, f64)>,
    ),
    Failure,
> {
    let socket = TcpSocket::new_v4()?;
    socket.bind("127.0.0.1:0".parse()?)?;
    let listener = socket.listen(1)?;
    let port = listener.local_addr()?.port();
    let (inbound_tx, mut inbound_rx) = mpsc::channel::<Vec<u8>>(32);
    // Server-to-Player motor PWM commands (interf 170, CMD subtype 1), logged
    // like the fuller fake in tests/server.rs (#61). Velocity commands
    // (position2d CMD) are logged too for the motion tools (#60).
    let (motor_tx, motor_rx) = mpsc::channel::<(u8, f32)>(64);
    let (vel_tx, vel_rx) = mpsc::channel::<(f64, f64)>(64);
    tokio::spawn(async move {
        let Ok((mut s, _)) = listener.accept().await else {
            return;
        };
        let mut banner = [0u8; 32];
        banner[..9].copy_from_slice(b"Player v.");
        if s.write_all(&banner).await.is_err() {
            return;
        }
        let (mut rd, wr) = s.into_split();
        let wr = Arc::new(tokio::sync::Mutex::new(wr));
        let ack_wr = wr.clone();
        tokio::spawn(async move {
            loop {
                let mut header = [0u8; 40];
                if rd.read_exact(&mut header).await.is_err() {
                    break;
                }
                let Some(h) = decode_header(&header) else {
                    break;
                };
                let mut payload = vec![0u8; h.size as usize];
                if rd.read_exact(&mut payload).await.is_err() {
                    break;
                }
                // Motor PWM commands carry one XDR float and get no ACK.
                if h.addr.interf == MOTOR_CODE {
                    if (h.msg_type, h.subtype, h.size) == (2, MOTOR_CMD_PWM, 4) {
                        let duty = f32::from_be_bytes(payload[0..4].try_into().unwrap());
                        let _ = motor_tx.send((h.addr.index as u8, duty)).await;
                    }
                    continue;
                }
                // Velocity commands (position2d CMD, 28 bytes) are logged, not
                // ACKed, exactly like the real Player.
                if h.addr.interf == POSITION2D_CODE && h.msg_type == 2 {
                    let vx = f64::from_be_bytes(payload[0..8].try_into().unwrap());
                    let vyaw = f64::from_be_bytes(payload[16..24].try_into().unwrap());
                    let _ = vel_tx.send((vx, vyaw)).await;
                    continue;
                }
                let reply = MsgHeader {
                    msg_type: MSGTYPE_RESP_ACK,
                    subtype: h.subtype,
                    size: 28,
                    ..h
                };
                let mut ack = vec![0u8; 28];
                ack[19] = payload.get(19).copied().unwrap_or(0);
                let mut w = ack_wr.lock().await;
                let ok = w.write_all(&encode_header(&reply)).await.is_ok()
                    && w.write_all(&ack).await.is_ok();
                if !ok {
                    break;
                }
            }
        });
        while let Some(p) = inbound_rx.recv().await {
            let mut w = wr.lock().await;
            if w.write_all(&p).await.is_err() {
                break;
            }
        }
    });
    Ok((port, inbound_tx, motor_rx, vel_rx))
}

pub struct Server(
    pub Child,
    pub u16,
    pub mpsc::Sender<Vec<u8>>,
    /// Server-to-Player motor PWM commands (index, duty), like `cmds` in the
    /// server harness (#61).
    pub mpsc::Receiver<(u8, f32)>,
    /// Server-to-Player velocity commands (vx, vyaw), like `cmds` (#60).
    pub mpsc::Receiver<(f64, f64)>,
);
impl Drop for Server {
    fn drop(&mut self) {
        let _ = self.0.kill();
        let _ = self.0.wait();
    }
}
pub async fn start(camera_url: Option<&str>) -> Server {
    start_with(camera_url, None).await
}
/// Full variant for #58: an optional SLAM adapter executable and the fake
/// Player inbound channel for injecting scan/odom packets.
/// Variant with an operator MCP grant spec (#56/#60): ROBOT_MCP_GRANT is set
/// for the child only, exactly as an operator would.
pub async fn start_env(camera_url: Option<&str>, grant: &str) -> Server {
    start_env_tts(camera_url, grant, None).await
}

/// Variant with a fake slow TTS provider and supervisor (#63): when `tts`
/// is Some(url) the child sees enabled cloud TTS against that URL; the
/// provider deliberately stalls so the job stays active for BUSY checks.
pub async fn start_env_tts(
    camera_url: Option<&str>,
    grant: &str,
    tts: Option<(&str, &str)>,
) -> Server {
    let (pp, input, motors, velocity) = fake_player().await.expect("fake player");
    let http = TcpListener::bind("127.0.0.1:0").await.unwrap();
    let port = http.local_addr().unwrap().port();
    drop(http);
    let mut command = Command::new(env!("CARGO_BIN_EXE_robot-server"));
    command.args([
        "--player-host",
        "127.0.0.1",
        "--player-port",
        &pp.to_string(),
        "--listen",
        "127.0.0.1",
        "--port",
        &port.to_string(),
    ]);
    if let Some(url) = camera_url {
        command.arg("--camera-url").arg(url);
    }
    let child = command
        .env_remove("ROBOT_SERVER_HEARTBEAT")
        .env_remove("ROBOT_SUPERVISOR_ADDR")
        .env_remove("ROBOT_SUPERVISOR_TOKEN")
        .env("ROBOT_MCP_GRANT", grant)
        .stdout(Stdio::null())
        .stderr(Stdio::piped());
    if let Some((url, token)) = tts {
        child
            .env("ROBOT_YANDEX_ENABLED", "true")
            .env("ROBOT_YANDEX_API_KEY", token)
            .env("ROBOT_YANDEX_URL", url);
    }
    let child = child.spawn().unwrap();
    let mut server = Server(child, port, input, motors, velocity);
    for _ in 0..100 {
        match server.0.try_wait().unwrap() {
            Some(status) => {
                use std::io::Read;
                let mut log = String::new();
                let _ = server.0.stderr.as_mut().unwrap().read_to_string(&mut log);
                panic!("robot-server exited {status}:\n{log}");
            }
            None => break,
        }
    }
    server
}

/// Variant with a reachable supervisor for the local drive lease (#64).
pub async fn start_env_supervisor(
    camera_url: Option<&str>,
    grant: &str,
    supervisor: (&str, &str),
) -> Server {
    let (pp, input, motors, velocity) = fake_player().await.expect("fake player");
    let http = TcpListener::bind("127.0.0.1:0").await.unwrap();
    let port = http.local_addr().unwrap().port();
    drop(http);
    let child = Command::new(env!("CARGO_BIN_EXE_robot-server"))
        .args([
            "--player-host",
            "127.0.0.1",
            "--player-port",
            &pp.to_string(),
            "--listen",
            "127.0.0.1",
            "--port",
            &port.to_string(),
        ])
        .env_remove("ROBOT_SERVER_HEARTBEAT")
        .env("ROBOT_SUPERVISOR_ADDR", supervisor.0)
        .env("ROBOT_SUPERVISOR_TOKEN", supervisor.1)
        .env("ROBOT_MCP_GRANT", grant)
        .stdout(Stdio::null())
        .stderr(Stdio::piped())
        .spawn()
        .unwrap();
    let mut server = Server(child, port, input, motors, velocity);
    for _ in 0..100 {
        match server.0.try_wait().unwrap() {
            Some(status) => {
                use std::io::Read;
                let mut log = String::new();
                let _ = server.0.stderr.as_mut().unwrap().read_to_string(&mut log);
                panic!("robot-server exited {status}:\n{log}");
            }
            None => break,
        }
    }
    server
}

pub async fn start_with(
    camera_url: Option<&str>,
    slam_adapter: Option<&std::path::Path>,
) -> Server {
    let (pp, input, motors, velocity) = fake_player().await.expect("fake player");
    let http = TcpListener::bind("127.0.0.1:0").await.unwrap();
    let port = http.local_addr().unwrap().port();
    drop(http);
    let mut command = Command::new(env!("CARGO_BIN_EXE_robot-server"));
    command.args([
        "--player-host",
        "127.0.0.1",
        "--player-port",
        &pp.to_string(),
        "--listen",
        "127.0.0.1",
        "--port",
        &port.to_string(),
    ]);
    if let Some(url) = camera_url {
        command.arg("--camera-url").arg(url);
    }
    if let Some(exe) = slam_adapter {
        command.arg("--slam-adapter").arg(exe);
    }
    let child = command
        .env_remove("ROBOT_SERVER_HEARTBEAT")
        .env_remove("ROBOT_SUPERVISOR_ADDR")
        .env_remove("ROBOT_SUPERVISOR_TOKEN")
        .stdout(Stdio::null())
        .stderr(Stdio::piped())
        .spawn()
        .unwrap();
    let mut server = Server(child, port, input, motors, velocity);
    // Fail fast with the child's own diagnostics if startup dies.
    for _ in 0..100 {
        match server.0.try_wait().unwrap() {
            Some(status) => {
                use std::io::Read;
                let mut log = String::new();
                let _ = server.0.stderr.as_mut().unwrap().read_to_string(&mut log);
                panic!("robot-server exited {status}:\n{log}");
            }
            None => break,
        }
    }
    server
}

pub(crate) async fn rpc(port: u16, body: Value, origin: Option<&str>) -> Option<reqwest::Response> {
    let mut request = reqwest::Client::builder()
        .no_proxy()
        .build()
        .unwrap()
        .post(format!("http://127.0.0.1:{port}/mcp"));
    if let Some(origin) = origin {
        request = request.header("origin", origin);
    }
    request
        .header("accept", "application/json, text/event-stream")
        .header("content-type", "application/json")
        .body(body.to_string())
        .send()
        .await
        .ok()
}
pub(crate) async fn rpc_json(port: u16, body: Value, origin: Option<&str>) -> Value {
    let response = rpc(port, body, origin).await.expect("connection refused");
    assert_eq!(response.status(), 200, "RPC must succeed");
    let text = response.text().await.unwrap();
    serde_json::from_str(&text).expect("JSON response")
}

#[tokio::test]
async fn mcp_endpoint_initializes_lists_and_calls_read_only_tools() {
    let mut s = start(None).await;
    // Wait for HTTP readiness.
    let init_body = json!({"jsonrpc":"2.0","id":1,"method":"initialize","params":{
        "protocolVersion":"2025-11-25","capabilities":{},"clientInfo":{"name":"t","version":"0"}}});
    let init = timeout(Duration::from_secs(10), async {
        loop {
            if let Some(response) = rpc(s.1, init_body.clone(), None).await {
                if response.status() != 404 && response.status() != 502 {
                    break response;
                }
            }
            tokio::time::sleep(Duration::from_millis(100)).await;
        }
    })
    .await
    .expect("MCP endpoint never became ready");
    assert_eq!(init.status(), 200);
    let init: Value = serde_json::from_str(&init.text().await.unwrap()).unwrap();
    assert_eq!(init["jsonrpc"], "2.0");
    assert_eq!(init["result"]["protocolVersion"], "2025-11-25");
    assert!(init["result"]["capabilities"]["tools"].is_object());
    // tools/list: schema'd read-only tools with safe defaults.
    let list = rpc_json(
        s.1,
        json!({"jsonrpc":"2.0","id":2,"method":"tools/list","params":{}}),
        None,
    )
    .await;
    let tools = list["result"]["tools"].as_array().expect("tools listed");
    let names: Vec<&str> = tools.iter().map(|t| t["name"].as_str().unwrap()).collect();
    assert!(names.contains(&"robot_observe"), "names: {names:?}");
    assert!(
        names.contains(&"robot_get_camera_frame"),
        "names: {names:?}"
    );
    for tool in tools {
        assert!(tool["description"].is_string());
        assert!(tool["inputSchema"].is_object());
        // Control tools (#56/#60/#61) are deliberately not read-only;
        // everything else on this endpoint is.
        let control = [
            "robot_acquire_control",
            "robot_release_control",
            "robot_drive_for",
            "robot_stop",
            "robot_set_cleaning_motors",
            "robot_say",
        ];
        if !control.contains(&tool["name"].as_str().unwrap_or("")) {
            assert_eq!(tool["annotations"]["readOnlyHint"], true);
        }
    }
    // tools/call: an observe snapshot comes from the live shared state.
    let call = rpc_json(
        s.1,
        json!({"jsonrpc":"2.0","id":3,"method":"tools/call","params":{
            "name":"robot_observe","arguments":{}}}),
        None,
    )
    .await;
    assert_eq!(call["result"]["isError"], false);
    let text = call["result"]["content"][0]["text"].as_str().unwrap();
    let snapshot: Value = serde_json::from_str(text).expect("text mirrors structured");
    assert_eq!(snapshot["schema_version"], 1);
    assert!(snapshot["sensors"].is_object());
    // Unknown tool is a tool execution error, not a protocol error.
    let bad = rpc_json(
        s.1,
        json!({"jsonrpc":"2.0","id":4,"method":"tools/call","params":{
            "name":"robot_fire_missiles","arguments":{}}}),
        None,
    )
    .await;
    assert_eq!(bad["result"]["isError"], true);
    // Unsupported method is a protocol-level error.
    let bad = rpc_json(
        s.1,
        json!({"jsonrpc":"2.0","id":5,"method":"tools/nope"}),
        None,
    )
    .await;
    assert!(bad["error"]["code"].is_i64());
    assert!(s.0.try_wait().unwrap().is_none(), "server stays alive");
}

#[tokio::test]
async fn mcp_endpoint_rejects_foreign_origin_and_bad_json() {
    let mut s = start(None).await;
    let _ = timeout(Duration::from_secs(10), async {
        loop {
            let down = match rpc(
                s.1,
                json!({"jsonrpc":"2.0","id":1,"method":"tools/list"}),
                None,
            )
            .await
            {
                Some(ok) if ok.status() == 200 => false,
                Some(_) => false,
                None => true,
            };
            if !down {
                break;
            }
            if let Some(status) = s.0.try_wait().unwrap() {
                use std::io::Read;
                let mut log = String::new();
                let _ = s.0.stderr.as_mut().unwrap().read_to_string(&mut log);
                panic!("server exited {status}: {log}");
            }
            tokio::time::sleep(Duration::from_millis(100)).await;
        }
    })
    .await
    .expect("endpoint ready");
    let evil = rpc(
        s.1,
        json!({"jsonrpc":"2.0","id":1,"method":"tools/list","params":{}}),
        Some("https://evil.example"),
    )
    .await
    .expect("evil request connects");
    assert_eq!(evil.status(), 403);
    // Invalid JSON body is a client error, not a crash.
    let bad = reqwest::Client::builder()
        .no_proxy()
        .build()
        .unwrap()
        .post(format!("http://127.0.0.1:{}/mcp", s.1))
        .header("accept", "application/json, text/event-stream")
        .body("{not json")
        .send()
        .await
        .unwrap();
    assert_eq!(bad.status(), 400);
    assert!(s.0.try_wait().unwrap().is_none(), "server stays alive");
}

#[tokio::test]
async fn mcp_camera_frame_reports_unavailable_camera_without_dying() {
    // Port 1 refuses connections immediately: an honest camera-down answer.
    let mut s = start(Some("http://127.0.0.1:1/mjpeg")).await;
    let _ = timeout(Duration::from_secs(10), async {
        loop {
            if let Some(ok) = rpc(
                s.1,
                json!({"jsonrpc":"2.0","id":1,"method":"tools/list"}),
                None,
            )
            .await
            {
                if ok.status() == 200 {
                    break;
                }
            }
            tokio::time::sleep(Duration::from_millis(100)).await;
        }
    })
    .await
    .expect("endpoint ready");
    let call = rpc_json(
        s.1,
        json!({"jsonrpc":"2.0","id":2,"method":"tools/call","params":{
            "name":"robot_get_camera_frame","arguments":{}}}),
        None,
    )
    .await;
    assert_eq!(call["result"]["isError"], true, "call: {call}");
    let text = call["result"]["content"][0]["text"].as_str().unwrap();
    assert!(
        text.contains("camera"),
        "error names the camera as the reason: {text}"
    );
    assert!(
        !text.contains('@') && !text.contains("127.0.0.1:1"),
        "no URL/credentials in the error: {text}"
    );
    assert!(
        call["result"]["structuredContent"].is_null(),
        "no frame metadata on error"
    );
    assert!(s.0.try_wait().unwrap().is_none(), "server stays alive");
    // The MCP surface still serves other tools afterwards.
    let observe = rpc_json(
        s.1,
        json!({"jsonrpc":"2.0","id":3,"method":"tools/call","params":{
            "name":"robot_observe","arguments":{}}}),
        None,
    )
    .await;
    assert_eq!(observe["result"]["isError"], false);
}

#[tokio::test]
async fn mcp_camera_frame_returns_image_and_metadata() {
    let url = fake_camera().await;
    let mut s = start(Some(&url)).await;
    let _ = timeout(Duration::from_secs(10), async {
        loop {
            if let Some(ok) = rpc(
                s.1,
                json!({"jsonrpc":"2.0","id":1,"method":"tools/list"}),
                None,
            )
            .await
            {
                if ok.status() == 200 {
                    break;
                }
            }
            tokio::time::sleep(Duration::from_millis(100)).await;
        }
    })
    .await
    .expect("endpoint ready");
    let call = rpc_json(
        s.1,
        json!({"jsonrpc":"2.0","id":2,"method":"tools/call","params":{
            "name":"robot_get_camera_frame","arguments":{}}}),
        None,
    )
    .await;
    assert_eq!(call["result"]["isError"], false, "call: {call}");
    // ImageContent block: base64 JPEG, never a base64 text blob.
    let image = &call["result"]["content"][0];
    assert_eq!(image["type"], "image");
    assert_eq!(image["mimeType"], "image/jpeg");
    let data = image["data"].as_str().unwrap();
    let decoded_len = data.len() * 3 / 4 - data.chars().rev().take_while(|c| *c == '=').count();
    assert_eq!(decoded_len, jpeg_640x480().len(), "frame round-trips");
    let text = call["result"]["content"][1]["text"].as_str().unwrap_or("");
    assert!(!data.contains(&text) || text.is_empty());
    assert!(
        call["result"]["content"]
            .as_array()
            .unwrap()
            .iter()
            .all(|b| b["type"] != "text" || !b["text"].as_str().unwrap_or("").contains(data)),
        "text mirror must not repeat the base64 image"
    );
    let meta = &call["result"]["structuredContent"];
    assert_eq!(meta["bytes"], jpeg_640x480().len() as u64);
    assert_eq!(meta["width"], 640, "width from the SOF marker: {meta}");
    assert_eq!(meta["height"], 480, "height from the SOF marker: {meta}");
    assert_eq!(meta["timing_basis"], "received");
    assert!(meta["frame_id"].as_u64().unwrap() >= 1);
    assert!(meta["age_hint_ms"].as_u64().is_some());
    // A second call advances the frame counter and reuses the source.
    let second = rpc_json(
        s.1,
        json!({"jsonrpc":"2.0","id":3,"method":"tools/call","params":{
            "name":"robot_get_camera_frame","arguments":{"max_age_ms":3000}}}),
        None,
    )
    .await;
    assert_eq!(second["result"]["isError"], false);
    assert_eq!(
        second["result"]["structuredContent"]["frame_id"]
            .as_u64()
            .unwrap(),
        meta["frame_id"].as_u64().unwrap() + 1,
        "frame_id is monotonic"
    );
    // Invalid arguments are a tool execution error, not a protocol error.
    let bad = rpc_json(
        s.1,
        json!({"jsonrpc":"2.0","id":4,"method":"tools/call","params":{
            "name":"robot_get_camera_frame","arguments":{"max_age_ms":99}}}),
        None,
    )
    .await;
    assert_eq!(bad["result"]["isError"], true);
    let bad = rpc_json(
        s.1,
        json!({"jsonrpc":"2.0","id":5,"method":"tools/call","params":{
            "name":"robot_get_camera_frame","arguments":{"max_bytes":1023}}}),
        None,
    )
    .await;
    assert_eq!(bad["result"]["isError"], true);
    let bad = rpc_json(
        s.1,
        json!({"jsonrpc":"2.0","id":6,"method":"tools/call","params":{
            "name":"robot_get_camera_frame","arguments":{"nope":1}}}),
        None,
    )
    .await;
    assert_eq!(bad["result"]["isError"], true);
    assert!(s.0.try_wait().unwrap().is_none(), "server stays alive");
}

// ---- #58: robot_get_map ----
// FEATURE: bounded SLAM map access over MCP (summary / grid / image).
// SPEC: the tool reads the same watch channel the panel "map" frames come
// from; a client-held known_version short-circuits to not_modified; the grid
// and image are cropped around the consistent SLAM pose (map center without
// one) with an explicit bounded rectangle; without a SLAM adapter the call is
// an honest isError, never a fabricated map. Read-only, no invented rooms or
// semantics.
/// Minimal shell SLAM adapter copied from tests/server.rs: the first `update`
/// publishes a fixed 2x1 grid, pose stays null (never fabricated).
fn slam_fake(dir: &std::path::Path) -> PathBuf {
    use std::os::unix::fs::PermissionsExt;
    let path = dir.join("slam-adapter-mcp");
    std::fs::write(
        &path,
        "#!/bin/sh\nn=0\nwhile read line; do\n  case \"$line\" in \
         *start_mapping*) echo '{\"ok\":true,\"mode\":\"mapping\"}' ;; \
         *update*) n=$((n+1)); echo '{\"ok\":true,\"mode\":\"mapping\",\"pose\":null,\"quality\":0.5}' ;; \
         *map*) echo '{\"ok\":true,\"grid\":{\"x_min\":0.0,\"y_min\":0.0,\"res\":0.05,\"w\":2,\"h\":1,\"cells\":\"50,-1\"}}' ;; \
         *) echo '{\"ok\":true}' ;; \
         esac\ndone\n"
            .replace("\\\"", "\""),
    )
    .unwrap();
    std::fs::set_permissions(&path, std::fs::Permissions::from_mode(0o700)).unwrap();
    path
}

fn scan_packet(stamp: f64) -> Vec<u8> {
    let mut p = 10f32.to_be_bytes().to_vec();
    p.extend(arr(&[1., 2., 0., 0.1, f32::NAN, 11.]));
    p.extend(arr(&[0., 0.7, 1., 2., 3., 4.]));
    p.extend(arr(&[]));
    p.extend(1u32.to_be_bytes());
    packet(6, 0, 3, stamp, &p)
}
/// Data packet builder copied from tests/server.rs (local there, too).
pub fn packet(interf: u16, index: u16, subtype: u8, stamp: f64, payload: &[u8]) -> Vec<u8> {
    let mut p = encode_header(&MsgHeader {
        addr: DevAddr {
            host: 0,
            robot: 0,
            interf,
            index,
        },
        msg_type: 1,
        subtype,
        timestamp: stamp,
        seq: 0,
        size: payload.len() as u32,
    })
    .to_vec();
    p.extend(payload);
    p
}
fn arr(v: &[f32]) -> Vec<u8> {
    let mut b = Vec::new();
    for _ in 0..2 {
        b.extend((v.len() as u32).to_be_bytes());
    }
    for f in v {
        b.extend(f.to_be_bytes());
    }
    b
}

/// Wait until a robot_get_map call returns the first non-null version. The
/// transient "not available yet" answer before the adapter's first export is
/// tolerated and retried until the deadline.
async fn wait_for_map(server: &Server) -> Value {
    timeout(Duration::from_secs(10), async {
        loop {
            let call = rpc_json(
                server.1,
                json!({"jsonrpc":"2.0","id":1,"method":"tools/call","params":{
                    "name":"robot_get_map","arguments":{}}}),
                None,
            )
            .await;
            let v = call["result"]["structuredContent"]["version"].clone();
            if !v.is_null() {
                assert_eq!(call["result"]["isError"], false, "call: {call}");
                return call;
            }
            tokio::time::sleep(Duration::from_millis(100)).await;
        }
    })
    .await
    .expect("map never appeared in the MCP tool")
}

async fn endpoint_ready(server: &Server) {
    timeout(Duration::from_secs(10), async {
        loop {
            if let Some(ok) = rpc(
                server.1,
                json!({"jsonrpc":"2.0","id":1,"method":"tools/list"}),
                None,
            )
            .await
            {
                if ok.status() == 200 {
                    break;
                }
            }
            tokio::time::sleep(Duration::from_millis(100)).await;
        }
    })
    .await
    .expect("endpoint ready");
}

#[tokio::test]
async fn mcp_observe_reports_freshness_lidar_sectors_and_monotonic_ids() {
    // No slam adapter here: observe must still answer with honest freshness
    // bases, an unknown-marked lidar summary and monotonic observation ids.
    let mut s = start(None).await;
    endpoint_ready(&s).await;
    let mut call = rpc_json(
        s.1,
        json!({"jsonrpc":"2.0","id":1,"method":"tools/call","params":{
            "name":"robot_observe","arguments":{}}}),
        None,
    )
    .await;
    let first = call["result"]["structuredContent"].clone();
    assert_eq!(first["schema_version"], 1);
    let id1 = first["observation_id"].as_u64().expect("observation id");
    assert!(first["session_epoch"].is_u64(), "session epoch present");
    let freshness = &first["freshness"];
    assert!(
        freshness["player_age_ms"].is_u64(),
        "player age in ms: {freshness}"
    );
    assert_eq!(
        freshness["basis"],
        "server receive time, not sensor exposure time"
    );
    // Without any scan the lidar summary must be explicitly unavailable,
    // never "free space".
    assert_eq!(first["lidar"]["available"], false);
    // Feed one scan through the fake Player and observe the sector summary.
    s.2.send(scan_packet(1.1)).await.unwrap();
    let deadline = std::time::Instant::now() + Duration::from_secs(5);
    loop {
        call = rpc_json(
            s.1,
            json!({"jsonrpc":"2.0","id":2,"method":"tools/call","params":{
                "name":"robot_observe","arguments":{}}}),
            None,
        )
        .await;
        let snapshot = &call["result"]["structuredContent"];
        let id2 = snapshot["observation_id"].as_u64().unwrap();
        assert!(id2 > id1, "observation ids are monotonic");
        if snapshot["lidar"]["available"] == true {
            let sectors = snapshot["lidar"]["sectors"].as_array().unwrap();
            assert_eq!(sectors.len(), 8, "eight angular sectors");
            // The fake scan has three beams in one direction: sectors with no
            // valid beams must be marked unknown, not zero-distance.
            assert!(
                sectors.iter().any(|sec| sec["unknown"] == true),
                "unknown sectors marked: {sectors:?}"
            );
            assert!(
                snapshot["freshness"]["scan_sequence"].is_u64(),
                "scan sequence for pairing: {snapshot}"
            );
            break;
        }
        assert!(
            std::time::Instant::now() < deadline,
            "scan never reached observe"
        );
        tokio::time::sleep(Duration::from_millis(50)).await;
    }
}

#[tokio::test]
async fn mcp_get_map_reports_missing_slam_honestly() {
    // No --slam-adapter: an explicit error, never a fabricated map.
    let mut s = start(None).await;
    endpoint_ready(&s).await;
    let call = rpc_json(
        s.1,
        json!({"jsonrpc":"2.0","id":2,"method":"tools/call","params":{
            "name":"robot_get_map","arguments":{}}}),
        None,
    )
    .await;
    assert_eq!(call["result"]["isError"], true, "call: {call}");
    let text = call["result"]["content"][0]["text"].as_str().unwrap();
    assert!(text.contains("SLAM"), "error names SLAM: {text}");
    assert!(
        call["result"]["structuredContent"].is_null(),
        "no map payload on error"
    );
    assert!(s.0.try_wait().unwrap().is_none(), "server stays alive");
}

/// Odom + scan pair that wakes the SLAM adapter (fields as in tests/server.rs).
async fn feed_slam(server: &Server) {
    let mut odom = vec![0u8; 49];
    for (i, value) in [0f64, 0., 0., 0.05, 0., 0.].iter().enumerate() {
        odom[i * 8..i * 8 + 8].copy_from_slice(&value.to_be_bytes());
    }
    server.2.send(packet(4, 0, 1, 1., &odom)).await.unwrap();
    server.2.send(scan_packet(1.1)).await.unwrap();
}

#[tokio::test]
async fn mcp_get_map_summary_not_modified_and_argument_validation() {
    let dir = std::env::temp_dir().join(format!("robot-mcp-map-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let fake = slam_fake(&dir);
    let mut s = start_with(None, Some(&fake)).await;
    endpoint_ready(&s).await;
    feed_slam(&s).await;
    let first = wait_for_map(&s).await;
    let meta = &first["result"]["structuredContent"];
    let version = meta["version"].as_u64().expect("version");
    assert!(version >= 1);
    assert_eq!(meta["representation"], "summary");
    assert_eq!(meta["metadata"]["resolution"], 0.05);
    assert_eq!(meta["metadata"]["size"], json!({"w": 2, "h": 1}));
    assert_eq!(meta["metadata"]["frame"], "map");
    assert_eq!(
        meta["pose_source"], "map_center",
        "a null adapter pose is not fabricated into a crop center"
    );
    // Same version: short not_modified, no grid/image payload.
    let same = rpc_json(
        s.1,
        json!({"jsonrpc":"2.0","id":2,"method":"tools/call","params":{
            "name":"robot_get_map","arguments":{"known_version": version}}}),
        None,
    )
    .await;
    assert_eq!(same["result"]["isError"], false);
    assert_eq!(
        same["result"]["structuredContent"],
        json!({"version": version, "not_modified": true})
    );
    // Stale known_version falls through to the full payload.
    let stale = rpc_json(
        s.1,
        json!({"jsonrpc":"2.0","id":3,"method":"tools/call","params":{
            "name":"robot_get_map","arguments":{"known_version": 9999}}}),
        None,
    )
    .await;
    assert_eq!(stale["result"]["isError"], false);
    assert!(stale["result"]["structuredContent"]["not_modified"].is_null());
    // Argument validation is enforced in code, not trusted from the schema.
    for bad_args in [
        json!({"representation": "hologram"}),
        json!({"radius_m": 0.05}),
        json!({"radius_m": 21.0}),
        json!({"downsample": 0}),
        json!({"downsample": 17}),
        json!({"known_version": "seven"}),
        json!({"nope": 1}),
    ] {
        let bad = rpc_json(
            s.1,
            json!({"jsonrpc":"2.0","id":4,"method":"tools/call","params":{
                "name":"robot_get_map","arguments": bad_args}}),
            None,
        )
        .await;
        assert_eq!(
            bad["result"]["isError"], true,
            "args {bad_args} must be rejected: {bad}"
        );
    }
    assert!(s.0.try_wait().unwrap().is_none(), "server stays alive");
    std::fs::remove_file(&fake).ok();
    std::fs::remove_dir(&dir).ok();
}

#[tokio::test]
async fn mcp_get_map_grid_and_image_return_bounded_payloads() {
    let dir = std::env::temp_dir().join(format!("robot-mcp-map-img-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let fake = slam_fake(&dir);
    let mut s = start_with(None, Some(&fake)).await;
    endpoint_ready(&s).await;
    feed_slam(&s).await;
    let first = wait_for_map(&s).await;
    let version = first["result"]["structuredContent"]["version"]
        .as_u64()
        .unwrap();
    // Grid: explicit bounded rectangle around the map center (no pose).
    let grid = rpc_json(
        s.1,
        json!({"jsonrpc":"2.0","id":2,"method":"tools/call","params":{
            "name":"robot_get_map","arguments":{
                "representation":"grid","radius_m":0.2,"downsample":1}}}),
        None,
    )
    .await;
    assert_eq!(grid["result"]["isError"], false, "grid: {grid}");
    let g = &grid["result"]["structuredContent"];
    assert_eq!(g["version"], version);
    assert_eq!(g["representation"], "grid");
    // The crop is explicit, never a truncation of the full map: a radius of
    // 0.2 m around the map center (0.05, 0.025) snaps to a 9x9 rectangle whose
    // cells outside the known 2x1 area are unknown, and the one known cell
    // ("50") stays in place at the crop origin row.
    assert_eq!(g["grid"]["w"], 9);
    assert_eq!(g["grid"]["h"], 9);
    assert_eq!(g["grid"]["res"], 0.05);
    assert_eq!(g["grid"]["x_min"], -0.2);
    assert_eq!(g["grid"]["y_min"], -0.2);
    let cells: Vec<&str> = g["grid"]["cells"].as_str().unwrap().split(',').collect();
    assert_eq!(cells.len(), 81, "explicit bounded rectangle");
    assert!(cells.contains(&"50"), "the known cell survives the crop");
    assert!(
        cells.iter().filter(|c| **c == "-1").count() >= 79,
        "unknown space outside the map stays unknown"
    );
    assert!(g["metadata"]["bounds"]["min"].is_array());
    assert_eq!(g["pose_source"], "map_center");
    // Image: an MCP ImageContent PNG block plus metadata-only text mirror.
    let image = rpc_json(
        s.1,
        json!({"jsonrpc":"2.0","id":3,"method":"tools/call","params":{
            "name":"robot_get_map","arguments":{
                "representation":"image","radius_m":0.2}}}),
        None,
    )
    .await;
    assert_eq!(image["result"]["isError"], false, "image: {image}");
    let block = &image["result"]["content"][0];
    assert_eq!(block["type"], "image");
    assert_eq!(block["mimeType"], "image/png");
    let data = block["data"].as_str().unwrap();
    assert!(data.len() > 8, "base64 png present");
    use base64::Engine;
    let bytes = base64::engine::general_purpose::STANDARD
        .decode(data)
        .expect("valid base64");
    assert_eq!(
        &bytes[..8],
        &[0x89, b'P', b'N', b'G', b'\r', b'\n', 0x1a, b'\n'],
        "PNG signature"
    );
    // The text mirror carries metadata only, never the base64 payload.
    assert!(
        image["result"]["content"]
            .as_array()
            .unwrap()
            .iter()
            .all(|b| b["type"] != "text" || !b["text"].as_str().unwrap_or("").contains(data)),
        "text mirror must not repeat the base64 image"
    );
    let im = &image["result"]["structuredContent"];
    assert_eq!(im["version"], version);
    assert_eq!(im["representation"], "image");
    assert!(im["metadata"]["png"]["width"].as_u64().is_some());
    assert!(im.get("grid").is_none(), "image carries no raw grid");
    assert!(s.0.try_wait().unwrap().is_none(), "server stays alive");
    std::fs::remove_file(&fake).ok();
    std::fs::remove_dir(&dir).ok();
}
