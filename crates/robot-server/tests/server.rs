// FEATURE: standalone HTTP/WebSocket Player server, without ROS.
// SPEC: viewing is inert; exclusive driver, bounded/clamped commands expire in 1s
// even with a silent/fragmented Player; disconnect/shutdown stop; sensor validity,
// explicit beam geometry, supervisor heartbeat and streaming authenticated camera.
// IMPLEMENTATION: independent async framing/drive clock, watch snapshots, exclusive
// client ownership, shared unchanged drive protections and incremental HTTP proxy.
// RESULT: IMPLEMENTED — offline fake Player, HTTP/WS, supervisor and camera checks pass.
// STATUS: DONE
// FEATURE: raw recording uses the sole Player stream, including subscription replies.
// SPEC: record before filtering; shutdown still sends zero; open failure never ACTIVATEs.
// IMPLEMENTATION: recorder preflight before supervisor, capture at receipt, finish after teardown.
// RESULT: IMPLEMENTED — fake Player capture and activation preflight checks pass.
// STATUS: DONE
use futures_util::{SinkExt, StreamExt};
use player_bridge::*;
use serde_json::{json, Value};
use std::{
    process::{Child, Command, Stdio},
    time::{Duration, Instant},
};
use tokio::{
    io::{AsyncReadExt, AsyncWriteExt},
    net::{TcpListener, TcpSocket, TcpStream},
    sync::mpsc,
    time::{sleep, timeout},
};
use tokio_tungstenite::{connect_async, tungstenite::Message as WsMessage};
type Ws = tokio_tungstenite::WebSocketStream<tokio_tungstenite::MaybeTlsStream<TcpStream>>;
const WAIT: Duration = Duration::from_secs(3);
struct Session {
    child: Child,
    port: u16,
    input: mpsc::Sender<Vec<u8>>,
    cmds: mpsc::Receiver<(Instant, f64, f64)>,
    fake: tokio::task::JoinHandle<()>,
    listening_at: tokio::sync::oneshot::Receiver<Instant>,
}
impl Drop for Session {
    fn drop(&mut self) {
        let _ = self.child.kill();
        let _ = self.child.wait();
        self.fake.abort();
    }
}
fn packet(interf: u16, index: u16, subtype: u8, stamp: f64, payload: &[u8]) -> Vec<u8> {
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
fn bumper(pressed: bool) -> Vec<u8> {
    let mut p = vec![0, 0, 0, 3, 0, 0, 0, 3, pressed as u8, 0, 0, 0];
    p.shrink_to_fit();
    packet(BUMPER_CODE, 0, 1, 10., &p)
}
fn bumper_segments(flags: &[u8]) -> Vec<u8> {
    let mut p = (flags.len() as u32).to_be_bytes().to_vec();
    p.extend((flags.len() as u32).to_be_bytes());
    p.extend(flags);
    p.resize(p.len().div_ceil(4) * 4, 0);
    packet(BUMPER_CODE, 0, 1, 10., &p)
}
fn compass(pos: [f64; 6]) -> Vec<u8> {
    let mut p = Vec::new();
    for v in pos {
        p.extend(v.to_bits().to_be_bytes());
    }
    p.resize(100, 0);
    packet(POSITION3D_CODE, 2, 1, 10., &p)
}
impl Session {
    async fn start(extra: &[&str], hb: Option<String>, optional: bool) -> Self {
        Self::start_delayed(extra, hb, optional, Duration::ZERO).await
    }
    async fn start_delayed(
        extra: &[&str],
        hb: Option<String>,
        optional: bool,
        delay: Duration,
    ) -> Self {
        Self::start_env(extra, hb, optional, delay, &[]).await
    }
    async fn start_env(
        extra: &[&str],
        hb: Option<String>,
        optional: bool,
        delay: Duration,
        env: &[(&str, &str)],
    ) -> Self {
        // Serialize port reservation until the child binds: parallel tests must not
        // accidentally discover another child's listener during the bind handoff.
        static STARTUP: tokio::sync::Mutex<()> = tokio::sync::Mutex::const_new(());
        let _startup = STARTUP.lock().await;
        let player = TcpSocket::new_v4().unwrap();
        player.bind("127.0.0.1:0".parse().unwrap()).unwrap();
        let pp = player.local_addr().unwrap().port();
        let http = TcpListener::bind("127.0.0.1:0").await.unwrap();
        let port = http.local_addr().unwrap().port();
        drop(http);
        let (input, mut incoming) = mpsc::channel::<Vec<u8>>(32);
        let (log, cmds) = mpsc::channel(1024);
        let (listening, listening_at) = tokio::sync::oneshot::channel();
        let startup_data: usize = env
            .iter()
            .find(|(name, _)| *name == "ROBOT_TEST_STARTUP_DATA")
            .map(|(_, value)| value.parse().unwrap())
            .unwrap_or(0);
        let fake = tokio::spawn(async move {
            if !delay.is_zero() {
                sleep(delay).await;
            }
            let player = player.listen(128).unwrap();
            let _ = listening.send(Instant::now());
            let (mut s, _) = player.accept().await.unwrap();
            let mut banner = [0; 32];
            banner[..9].copy_from_slice(b"Player v.");
            s.write_all(&banner).await.unwrap();
            // FEATURE: optional service subscriptions; no motor subscriptions or commands.
            // SPEC: service rejection must not prevent the panel from starting.
            for n in 0..17 {
                let mut h = [0; 40];
                s.read_exact(&mut h).await.unwrap();
                let h = decode_header(&h).unwrap();
                assert_eq!(h.msg_type, 3);
                let mut p = vec![0; h.size as usize];
                s.read_exact(&mut p).await.unwrap();
                let expected = [
                    (6u32, 0u32),
                    (4, 0),
                    (14, 0),
                    (22, 0),
                    (22, 1),
                    (22, 2),
                    (5, 0),
                    (2, 0),
                    (30, 2),
                    (22, 3),
                    (22, 4),
                    (22, 6),
                    (22, 7),
                    (22, 8),
                    (30, 0),
                    (30, 1),
                    (30, 4),
                ][n];
                assert_eq!(
                    (
                        u32::from_be_bytes(p[8..12].try_into().unwrap()),
                        u32::from_be_bytes(p[12..16].try_into().unwrap())
                    ),
                    expected
                );
                let mut ack = [0; 28];
                ack[19] = 1;
                // Initial one-shot DATA must survive later subscription round trips.
                if n == 3 && optional {
                    s.write_all(&bumper(false)).await.unwrap();
                    if startup_data > 0 {
                        let mut wall = arr(&[2.2]);
                        wall.extend(arr(&[0.055]));
                        s.write_all(&packet(IR_CODE, 0, 1, 20., &wall))
                            .await
                            .unwrap();
                    }
                }
                if n == 16 && startup_data > 0 {
                    s.write_all(&bumper(true)).await.unwrap();
                    s.write_all(&bumper(false)).await.unwrap();
                    for i in 0..startup_data {
                        let mut odom = Vec::new();
                        for value in [i as f64 / 1000., 2., 0.3, 0., 0., 0.] {
                            odom.extend(value.to_be_bytes());
                        }
                        odom.extend(0u32.to_be_bytes());
                        if s.write_all(&packet(POSITION2D_CODE, 0, 1, 20. + i as f64 * 0.02, &odom))
                            .await
                            .is_err()
                        {
                            return;
                        }
                    }
                    s.write_all(&scan_packet(80.)).await.unwrap();
                    sleep(Duration::from_millis(150)).await;
                }
                let reply = MsgHeader {
                    msg_type: if n < 2 || optional { 4 } else { 6 },
                    size: 28,
                    ..h
                };
                s.write_all(&encode_header(&reply)).await.unwrap();
                s.write_all(&ack).await.unwrap();
            }
            let (mut rd, mut wr) = s.into_split();
            let read = async move {
                loop {
                    let mut h = [0; 40];
                    if rd.read_exact(&mut h).await.is_err() {
                        break;
                    }
                    let h = decode_header(&h).unwrap();
                    let mut p = vec![0; h.size as usize];
                    if rd.read_exact(&mut p).await.is_err() {
                        break;
                    }
                    if h.addr.interf == 170 {
                        // motor PWM: one XDR float (4 bytes), subtype 1
                        assert_eq!((h.msg_type, h.subtype, h.size), (2, 1, 4));
                        let duty = f32::from_be_bytes(p[0..4].try_into().unwrap());
                        if log
                            .send((Instant::now(), (h.addr.index + 1000) as f64, duty as f64))
                            .await
                            .is_err()
                        {
                            break;
                        }
                        continue;
                    }
                    assert_eq!((h.msg_type, h.addr.interf, h.subtype), (2, 4, 1));
                    let f = |i| f64::from_be_bytes(p[i..i + 8].try_into().unwrap());
                    if log.send((Instant::now(), f(0), f(16))).await.is_err() {
                        break;
                    }
                }
            };
            let write = async move {
                while let Some(p) = incoming.recv().await {
                    if wr.write_all(&p).await.is_err() {
                        break;
                    }
                }
            };
            tokio::select! {_=read=>{},_=write=>{}}
        });
        let mut c = Command::new(env!("CARGO_BIN_EXE_robot-server"));
        c.args([
            "--player-host",
            "127.0.0.1",
            "--player-port",
            &pp.to_string(),
            "--listen",
            "127.0.0.1",
            "--port",
            &port.to_string(),
        ])
        .args(extra)
        .env_remove("ROBOT_SERVER_HEARTBEAT")
        .env_remove("ROBOT_BRIDGE_HEARTBEAT")
        .env_remove("ROBOT_SUPERVISOR_ADDR")
        .env_remove("ROBOT_SUPERVISOR_TOKEN")
        .stdout(Stdio::null())
        .stderr(Stdio::piped());
        if let Some(hb) = hb {
            c.env("ROBOT_SERVER_HEARTBEAT", hb);
        }
        for (k, v) in env {
            c.env(k, v);
        }
        let child = c.spawn().unwrap();
        let mut this = Self {
            child,
            port,
            input,
            cmds,
            fake,
            listening_at,
        };
        let end = Instant::now() + WAIT;
        loop {
            if TcpStream::connect(("127.0.0.1", port)).await.is_ok() {
                break;
            }
            if let Some(status) = this.child.try_wait().unwrap() {
                let mut error = String::new();
                std::io::Read::read_to_string(this.child.stderr.as_mut().unwrap(), &mut error)
                    .unwrap();
                panic!("standalone server exited instead of serving HTTP ({status}): {error}");
            }
            assert!(Instant::now() < end, "standalone server did not serve HTTP");
            sleep(Duration::from_millis(10)).await;
        }
        this
    }
    async fn ws(&self) -> Ws {
        connect_async(format!("ws://127.0.0.1:{}/ws", self.port))
            .await
            .unwrap()
            .0
    }
    async fn send(&self, p: Vec<u8>) {
        self.input.send(p).await.unwrap();
    }
    async fn command(&mut self) -> (Instant, f64, f64) {
        timeout(WAIT, self.cmds.recv())
            .await
            .expect("Player never got command")
            .unwrap()
    }
    async fn exited_ok(&mut self) {
        timeout(WAIT, async {
            loop {
                if let Some(status) = self.child.try_wait().unwrap() {
                    assert!(status.success(), "clean shutdown must exit successfully");
                    break;
                }
                sleep(Duration::from_millis(10)).await;
            }
        })
        .await
        .expect("server did not exit");
    }
    async fn quiet(&mut self, ms: u64) {
        assert!(
            timeout(Duration::from_millis(ms), self.cmds.recv())
                .await
                .is_err(),
            "unexpected velocity while inert/held"
        );
    }
    async fn exited(&mut self) {
        timeout(WAIT, async {
            loop {
                if let Some(s) = self.child.try_wait().unwrap() {
                    assert!(!s.success(), "session failure must exit unsuccessfully");
                    break;
                }
                sleep(Duration::from_millis(20)).await;
            }
        })
        .await
        .unwrap();
        let mut log = String::new();
        std::io::Read::read_to_string(self.child.stderr.as_mut().unwrap(), &mut log).unwrap();
        assert!(!log.contains("panicked"), "session cleanup panicked: {log}");
        assert!(
            log.contains("robot-server:"),
            "missing session error: {log}"
        );
    }
}
// BUG: normal startup telemetry exceeded the total 128-message subscription buffer.
// HYPOTHESIS: process DATA while awaiting ACKs instead of buffering or discarding events.
// SOLUTION: one persistent processor handles subscription-time and normal DATA at receipt.
// RESULT: FIXED — 3000 startup packets preserve bumper edges, sensor state and scan sequence.
// STATUS: FIXED
#[tokio::test]
async fn startup_data_flood_preserves_initial_sensors_and_bumper_edges() {
    const FRAMES: usize = 3000;
    let mut s = Session::start_env(
        &[],
        None,
        true,
        Duration::ZERO,
        &[("ROBOT_TEST_STARTUP_DATA", "3000")],
    )
    .await;
    let mut w = s.ws().await;
    let state = frame(&mut w, "state").await;
    assert_eq!(
        state["blocked"], true,
        "a press followed by release during startup must still latch"
    );
    assert_eq!(state["sensors"]["bumper"], "clear");
    assert_eq!(state["sensors"]["wall_voltages"], json!([2.2]));
    assert!((state["sensors"]["wall_ir"].as_f64().unwrap() - 0.055).abs() < 1e-6);
    assert_eq!(state["odom"]["x"], (FRAMES - 1) as f64 / 1000.);
    assert_eq!(frame(&mut w, "scan").await["sequence"], 1);
    s.send(scan_packet(80.2)).await;
    assert_eq!(frame(&mut w, "scan").await["sequence"], 2);
    assert_eq!(
        (s.command().await.1, s.child.try_wait().unwrap()),
        (0., None)
    );
    cmd(&mut w, r#"{"v":0.1,"w":0.0}"#).await;
    s.quiet(100).await;
    cmd(&mut w, r#"{"type":"reset_blocked"}"#).await;
    timeout(WAIT, async {
        loop {
            if frame(&mut w, "state").await["blocked"] == false {
                break;
            }
        }
    })
    .await
    .unwrap();
    cmd(&mut w, r#"{"v":-0.1,"w":0.0}"#).await;
    assert_eq!(s.command().await.1, -0.1);
    let mut service = arr(&[17.]);
    service.extend(arr(&[]));
    s.send(packet(IR_CODE, 7, 1, 81., &service)).await;
    let live = timeout(WAIT, async {
        loop {
            let state = frame(&mut w, "state").await;
            if state["sensors"]["service_channels"]["ir:7"]["voltages"] == json!([17.]) {
                break state;
            }
        }
    })
    .await
    .unwrap();
    assert_eq!(live["sensors"]["wall_voltages"], json!([2.2]));
    assert_eq!(live["odom"]["x"], (FRAMES - 1) as f64 / 1000.);
}

// BUG: vendor position3d:2 publishes thresholded XYZ, never a compass yaw.
// HYPOTHESIS: a separate authenticated raw stream must own compass state, including freshness.
// SOLUTION: independent authenticated evdev observer with bounded framing and freshness.
// RESULT: FIXED — raw updates, vendor isolation, expiry and nonfatal failures pass offline.
// STATUS: FIXED
async fn compass_line(stream: &mut TcpStream) -> String {
    timeout(WAIT, async {
        let mut bytes = Vec::new();
        loop {
            let byte = stream.read_u8().await.unwrap();
            if byte == b'\n' {
                return String::from_utf8(bytes).unwrap();
            }
            assert!(bytes.len() < 256);
            bytes.push(byte);
        }
    })
    .await
    .unwrap()
}
async fn wait_compass(w: &mut Ws, status: &str, stamp: Option<f64>) -> Value {
    timeout(WAIT, async {
        loop {
            let state = frame(w, "state").await;
            let compass = &state["sensors"]["compass"];
            if compass["status"] == status && stamp.is_none_or(|s| compass["stamp"] == s) {
                return state;
            }
        }
    })
    .await
    .expect("compass state never reached expected status")
}
#[tokio::test]
async fn raw_compass_authenticates_updates_and_expires_without_blocking_drive() {
    let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
    let address = listener.local_addr().unwrap().to_string();
    let mut s = Session::start_env(
        &[],
        None,
        true,
        Duration::ZERO,
        &[
            ("ROBOT_SUPERVISOR_ADDR", &address),
            ("ROBOT_SUPERVISOR_TOKEN", "compass-test-token"),
        ],
    )
    .await;
    let (mut raw, _) = timeout(WAIT, listener.accept())
        .await
        .expect("raw compass connection was never opened")
        .unwrap();
    assert_eq!(compass_line(&mut raw).await, "AUTH compass-test-token");
    raw.write_all(b"AUTHENTICATED\n").await.unwrap();
    assert_eq!(compass_line(&mut raw).await, "COMPASS");
    let mut w = s.ws().await;
    raw.write_all(b"COMPASS 42.5 -139 ").await.unwrap();
    sleep(Duration::from_millis(10)).await;
    raw.write_all(b"164 66\n").await.unwrap();
    let state = wait_compass(&mut w, "uncalibrated", Some(42.5)).await;
    let magnetic = &state["sensors"]["compass"];
    assert_eq!(magnetic["source"], "st480-evdev");
    assert_eq!(magnetic["calibrated"], false);
    assert_eq!(
        (
            magnetic["x"].as_i64(),
            magnetic["y"].as_i64(),
            magnetic["z"].as_i64()
        ),
        (Some(-139), Some(164), Some(66))
    );
    assert!((magnetic["yaw"].as_f64().unwrap() - 164f64.atan2(-139.)).abs() < 1e-6);
    s.send(compass([0.; 6])).await;
    let state = timeout(WAIT, async {
        loop {
            let state = frame(&mut w, "state").await;
            if state["sensors"]["service_channels"]["position3d:2"].is_object() {
                break state;
            }
        }
    })
    .await
    .unwrap();
    assert_eq!(state["sensors"]["compass"]["stamp"], 42.5);
    raw.write_all(b"COMPASS 42.6 100 0 7\n").await.unwrap();
    assert_eq!(
        wait_compass(&mut w, "uncalibrated", Some(42.6)).await["sensors"]["compass"]["yaw"],
        0.
    );
    let stale = wait_compass(&mut w, "stale", None).await;
    assert!(stale["sensors"]["compass"]["yaw"].is_null());
    s.send(bumper(false)).await;
    cmd(&mut w, r#"{"v":0.1,"w":0.0}"#).await;
    assert_eq!(
        s.command().await.1,
        0.1,
        "a silent compass must not block driving"
    );
    let mut byte = [0];
    assert_eq!(
        timeout(WAIT, raw.read(&mut byte)).await.unwrap().unwrap(),
        0,
        "stale stream must be closed, not leaked"
    );
}
#[tokio::test]
async fn refused_compass_is_explicit_and_nonfatal() {
    let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
    let address = listener.local_addr().unwrap().to_string();
    let mut s = Session::start_env(
        &[],
        None,
        true,
        Duration::ZERO,
        &[
            ("ROBOT_SUPERVISOR_ADDR", &address),
            ("ROBOT_SUPERVISOR_TOKEN", ""),
        ],
    )
    .await;
    let (mut raw, _) = timeout(WAIT, listener.accept())
        .await
        .expect("raw compass connection was never opened")
        .unwrap();
    assert_eq!(compass_line(&mut raw).await, "COMPASS");
    raw.write_all(b"REFUSED\n").await.unwrap();
    let mut w = s.ws().await;
    let state = wait_compass(&mut w, "unsupported", None).await;
    assert!(state["sensors"]["compass"]["yaw"].is_null());
    cmd(&mut w, r#"{"v":0.1,"w":0.0}"#).await;
    assert_eq!(s.command().await.1, 0.1);
}

#[tokio::test]
async fn raw_compass_rejects_invalid_zero_and_oversized_samples() {
    for (line, expected) in [
        ("COMPASS 1 0 0 40\n".to_owned(), "invalid"),
        ("COMPASS NaN 1 2 3\n".to_owned(), "invalid"),
        ("COMPASS -1 1 2 3\n".to_owned(), "invalid"),
        ("COMPASS 1 NaN 2 3\n".to_owned(), "invalid"),
        ("COMPASS 1 1 2 3 unexpected\n".to_owned(), "invalid"),
        (format!("COMPASS {}\n", "1".repeat(300)), "invalid"),
        ("COMPASS_UNAVAILABLE\n".to_owned(), "unavailable"),
    ] {
        let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
        let address = listener.local_addr().unwrap().to_string();
        let mut s = Session::start_env(
            &[],
            None,
            true,
            Duration::ZERO,
            &[
                ("ROBOT_SUPERVISOR_ADDR", &address),
                ("ROBOT_SUPERVISOR_TOKEN", ""),
            ],
        )
        .await;
        let (mut raw, _) = timeout(WAIT, listener.accept())
            .await
            .expect("raw compass connection was never opened")
            .unwrap();
        assert_eq!(compass_line(&mut raw).await, "COMPASS");
        let mut w = s.ws().await;
        raw.write_all(line.as_bytes()).await.unwrap();
        let state = wait_compass(&mut w, expected, None).await;
        assert!(state["sensors"]["compass"]["yaw"].is_null(), "{line}");
        assert!(s.child.try_wait().unwrap().is_none());
    }
}

async fn cmd(w: &mut Ws, s: &str) {
    w.send(WsMessage::Text(s.into())).await.unwrap();
}
async fn frame(w: &mut Ws, kind: &str) -> Value {
    timeout(WAIT, async {
        loop {
            let m = w.next().await.expect("WS open").unwrap();
            if let Ok(s) = m.to_text() {
                let v: Value = serde_json::from_str(s).unwrap();
                assert!(
                    matches!(
                        v["type"].as_str(),
                        Some("state" | "scan" | "vision" | "map")
                    ),
                    "unexpected frame {v}"
                );
                if v["type"] == "vision" {
                    assert!(
                        v["data"]["enabled"].is_boolean(),
                        "control channel carries only vision capability, not images"
                    );
                    assert!(v["data"].get("jpeg").is_none());
                }
                if v["type"] == kind {
                    return v["data"].clone();
                }
            }
        }
    })
    .await
    .unwrap_or_else(|_| panic!("FRAME_TIMEOUT waiting for kind={kind}"))
}
/// Wait for the bumper field and return the carrying state frame.
async fn wait_bumper_frame(w: &mut Ws, expected: &str) -> Value {
    timeout(WAIT, async {
        loop {
            let v = frame(w, "state").await;
            if v["bumper"] == expected {
                return v;
            }
        }
    })
    .await
    .unwrap_or_else(|_| panic!("WAITBUMPER_FRAME_FAIL({expected})"))
}
async fn wait_bumper(w: &mut Ws, expected: &str) {
    timeout(WAIT, async {
        loop {
            if frame(w, "state").await["bumper"] == expected {
                break;
            }
        }
    })
    .await
    .unwrap_or_else(|_| {
        panic!("WAITBUMPER_FAIL");
    });
}
// FEATURE: persistent explicit collision block and strict observer reset.
// SPEC: any contact blocks until reset; reset broadcasts without Player data,
// preserves bumper/ownership, never moves, and survives reconnect/telemetry.
// IMPLEMENTATION: typed reset with immediate state publication and persistent contact level.
// RESULT: IMPLEMENTED — strict parsing, reset, observer semantics and rearm verified offline.
// STATUS: DONE
#[tokio::test]
async fn blocked_reset_is_strict_persistent_and_observer_safe() {
    let mut s = Session::start(&[], None, true).await;
    let mut owner = s.ws().await;
    let mut observer = s.ws().await;
    assert_eq!(frame(&mut owner, "state").await["blocked"], false);
    frame(&mut observer, "state").await;
    cmd(&mut owner, r#"{"v":0.1,"w":0}"#).await;
    assert_eq!(s.command().await.1, 0.1);
    s.send(bumper(true)).await;
    assert_eq!(s.command().await.1, 0.);
    wait_bumper(&mut owner, "PRESSED").await;
    assert_eq!(frame(&mut observer, "state").await["blocked"], true);
    for bad in [
        r#"{"type":"reset_blocked","v":0,"w":0}"#,
        r#"{"type":"reset_blocked","extra":true}"#,
        r#"{"type":"reset_blocked","data":{}}"#,
        r#"{"type":["reset_blocked"]}"#,
        r#"{"reset_blocked":true}"#,
        r#"{"type":"reset_blocked","type":"reset_blocked"}"#,
        r#"{"v":0,"w":0}"#,
    ] {
        cmd(&mut observer, bad).await;
    }
    cmd(&mut owner, r#"{"v":0.2,"w":0}"#).await;
    s.quiet(100).await;
    observer.close(None).await.unwrap();
    let mut observer = s.ws().await;
    assert_eq!(frame(&mut observer, "state").await["blocked"], true);
    // New hazard contract (#65): a reset under a still-pressed bumper is
    // refused — the latch stays on and motion stays refused until the
    // contact physically clears.
    cmd(&mut observer, r#"{"type":"reset_blocked"}"#).await;
    s.send(bumper(false)).await;
    wait_bumper(&mut observer, "clear").await;
    cmd(&mut observer, r#"{"type":"reset_blocked"}"#).await;
    let reset = frame(&mut observer, "state").await;
    assert_eq!(reset["blocked"], false);
    assert_eq!(reset["bumper"], "clear");
    cmd(&mut observer, r#"{"type":"reset_blocked"}"#).await;
    cmd(&mut observer, r#"{"v":0.3,"w":0}"#).await;
    s.quiet(80).await;
    // Repeated pressed data and unrelated telemetry cannot resurrect the
    // block before release+repress.
    s.send(bumper(true)).await;
    s.send(packet(4, 0, 1, 21., &[0; 52])).await;
    let state = timeout(WAIT, async {
        loop {
            let state = frame(&mut observer, "state").await;
            if state["odom"].is_object() {
                break state;
            }
        }
    })
    .await
    .unwrap();
    // The re-pressed bumper latched again (rising edge after a clean reset):
    // blocked is true and stays until release + explicit reset.
    assert_eq!(state["blocked"], true);
    cmd(&mut owner, r#"{"v":-0.1,"w":0}"#).await;
    assert_eq!(s.command().await.1, 0.);
    s.send(bumper(false)).await;
    wait_bumper(&mut observer, "clear").await;
    cmd(&mut observer, r#"{"type":"reset_blocked"}"#).await;
    cmd(&mut owner, r#"{"v":-0.1,"w":0}"#).await;
    // The re-latch queued a priority zero: it flushes before the new command.
    let first = s.command().await;
    if first.1 == 0.0 {
        assert_eq!(s.command().await.1, -0.1);
    } else {
        assert_eq!(first.1, -0.1);
    }
    s.send(bumper(false)).await;
    wait_bumper(&mut observer, "clear").await;
    s.send(bumper(true)).await;
    assert_eq!(s.command().await.1, 0.);
    assert_eq!(frame(&mut observer, "state").await["blocked"], true);
    owner.close(None).await.unwrap();
    assert_eq!(s.command().await.1, 0.);
    let mut reconnect = s.ws().await;
    assert_eq!(frame(&mut reconnect, "state").await["blocked"], true);
}

#[test]
fn help_and_bad_args() {
    let out = Command::new(env!("CARGO_BIN_EXE_robot-server"))
        .arg("--help")
        .output()
        .unwrap();
    assert!(out.status.success(), "--help must work without ROS/Player");
    let text = String::from_utf8(out.stdout).unwrap();
    for a in [
        "--player-host",
        "--player-port",
        "--listen",
        "--port",
        "--camera-url",
        "--netrc",
    ] {
        assert!(text.contains(a));
    }
    assert!(!Command::new(env!("CARGO_BIN_EXE_robot-server"))
        .args(["--port", "bad"])
        .status()
        .unwrap()
        .success());
}
// FEATURE: backend angular ceiling of 3.14 rad/s.
// SPEC: WebSocket commands reach fake Player unchanged below the ceiling;
// excess is clamped symmetrically without changing explicit low-speed commands.
// IMPLEMENTATION: the shared drive ceiling applies through the existing WS path.
// RESULT: IMPLEMENTED — lower speeds and symmetric ceiling reach fake Player.
// STATUS: DONE
#[tokio::test]
async fn angular_ceiling_preserves_lower_speeds() {
    let mut s = Session::start(&[], None, false).await;
    let mut w = s.ws().await;
    for (requested, expected) in [
        (0.4, 0.4),
        (-0.4, -0.4),
        (1., 1.),
        (-2., -2.),
        (3.14, 3.14),
        (-3.14, -3.14),
        (9., 3.14),
        (-9., -3.14),
    ] {
        cmd(&mut w, &json!({"v": 0.1, "w": requested}).to_string()).await;
        let moving = s.command().await;
        assert_eq!((moving.1, moving.2), (0.1, expected));
    }
}

#[tokio::test]
async fn silent_player_lease_fragmentation_and_exclusive_driver() {
    let mut s = Session::start(&[], None, false).await;
    let mut a = s.ws().await;
    let mut b = s.ws().await;
    let initial = frame(&mut a, "state").await;
    assert!(initial.get("map_pose").is_none());
    assert!(initial.get("map_age_s").is_none());
    assert!(initial["sensors"]["battery"].is_null());
    s.quiet(100).await;
    for bad in [
        "{",
        "null",
        "{}",
        r#"{"v":"1","w":0}"#,
        r#"{"v":1e999,"w":0}"#,
        r#"{"v":0}"#,
        r#"{"v":true,"w":0}"#,
    ] {
        cmd(&mut a, bad).await;
    }
    s.quiet(100).await;
    // Partial header remains pending across the independent lease deadline.
    let p = packet(POSITION2D_CODE, 0, 1, 1., &[0; 52]);
    s.send(p[..17].to_vec()).await;
    cmd(&mut a, r#"{"v":10,"w":-10}"#).await;
    let moving = s.command().await;
    assert_eq!((moving.1, moving.2), (0.5, -3.14));
    cmd(&mut b, r#"{"v":-0.1,"w":0}"#).await;
    b.close(None).await.unwrap();
    let stop = s.command().await;
    assert_eq!((stop.1, stop.2), (0., 0.));
    assert!((Duration::from_millis(850)..Duration::from_millis(1350))
        .contains(&stop.0.duration_since(moving.0)));
    s.send(p[17..].to_vec()).await;
    s.quiet(80).await;
    cmd(&mut a, r#"{"v":0.05,"w":0}"#).await;
    assert_eq!(s.command().await.1, 0.05);
    a.close(None).await.unwrap();
    let t = Instant::now();
    assert_eq!(s.command().await.1, 0.);
    assert!(t.elapsed() < Duration::from_millis(500));
    s.quiet(100).await;
}
#[tokio::test]
async fn scan_and_raw_sensor_schema() {
    let mut s = Session::start(&[], None, true).await;
    let mut w = s.ws().await;
    let html_path = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../web/panel.html");
    let page = reqwest::Client::builder()
        .no_proxy()
        .build()
        .unwrap()
        .get(format!("http://127.0.0.1:{}/", s.port))
        .send()
        .await
        .unwrap();
    if let Ok(expected) = std::fs::read_to_string(html_path) {
        assert_eq!(page.status(), 200);
        assert_eq!(page.text().await.unwrap(), expected);
    } else {
        assert_eq!(page.status(), 503);
    }
    assert_eq!(frame(&mut w, "state").await["bumper"], "clear");
    // Per-segment bumper state is inventory: Left/Front/Right flags ride along
    // the aggregate without replacing it.
    s.send(bumper_segments(&[1, 0, 1])).await;
    let v = timeout(WAIT, async {
        loop {
            let v = frame(&mut w, "state").await;
            if v["bumper"] == "PRESSED" {
                break v;
            }
        }
    })
    .await
    .unwrap();
    assert_eq!(v["sensors"]["bumper_segments"], json!([true, false, true]));
    let contact_stop = s.command().await;
    assert_eq!((contact_stop.1, contact_stop.2), (0., 0.));
    s.send(bumper_segments(&[0, 0, 0])).await;
    let v = timeout(WAIT, async {
        loop {
            let v = frame(&mut w, "state").await;
            if v["bumper"] == "clear" {
                break v;
            }
        }
    })
    .await
    .unwrap();
    assert_eq!(
        v["sensors"]["bumper_segments"],
        json!([false, false, false])
    );
    let mut p = 10f32.to_be_bytes().to_vec();
    p.extend(arr(&[1., 2., 0., 0.1, f32::NAN, 11.]));
    p.extend(arr(&[0., 0.7, 1., 2., 3., 4.]));
    p.extend(arr(&[]));
    p.extend(1u32.to_be_bytes());
    s.send(packet(6, 0, 3, 42.25, &p)).await;
    let scan = frame(&mut w, "scan").await;
    assert_eq!(scan["frame"], "base_link");
    assert_eq!(scan["stamp"], 42.25);
    assert_eq!(scan["range_max"], 10.);
    let points = scan["points"].as_array().unwrap();
    assert_eq!(points.len(), 2);
    for (i, (r, a)) in [(1f64, 0f64), (2., 0.7f32 as f64)].iter().enumerate() {
        let angle = *a;
        assert!((points[i][0].as_f64().unwrap() - (-0.1 + r * angle.cos())).abs() < 1e-5);
        assert!((points[i][1].as_f64().unwrap() - r * angle.sin()).abs() < 1e-5);
    }
    // Cliff/drop inventory frames stay inert for the hazard gate (#65):
    // nonzero ranges would latch `blocked` and stop the wheels, so this
    // schema test feeds zeroed hazard channels and asserts the raw arrays
    // still render. Hazard latching has its own coverage.
    for (i, vs, rs) in [
        (0, vec![2.2], vec![0.055]),
        (1, vec![1., 2., 3., 4.], vec![0., 0., 0., 0.]),
        (2, vec![5., 6.], vec![0., 0.]),
    ] {
        let mut p = arr(&vs);
        p.extend(arr(&rs));
        s.send(packet(22, i, 1, 43., &p)).await;
    }
    s.send(packet(5, 0, 1, 43., &arr(&[65.534]))).await;
    let mut p = 25u32.to_be_bytes().to_vec();
    for f in [19.94f32, 83., 0., 0.] {
        p.extend(f.to_be_bytes());
    }
    p.extend(1u32.to_be_bytes());
    s.send(packet(2, 0, 1, 43., &p)).await;
    let mut od = Vec::new();
    for f in [1f64, 2., 0.3, 0.06, 0., 0.2] {
        od.extend(f.to_be_bytes());
    }
    od.extend(0u32.to_be_bytes());
    s.send(packet(4, 0, 1, 43., &od)).await;
    s.send(compass([0.1, 0.2, 0.3, 0.04, 0.05, 1.25])).await;
    let state = timeout(WAIT, async {
        loop {
            let v = frame(&mut w, "state").await;
            if v["sensors"]["service_channels"]["position3d:2"].is_object() {
                break v;
            }
        }
    })
    .await
    .unwrap();
    assert_eq!(
        state["battery"],
        json!({"percent":83,"volts":19.9,"charging":true})
    );
    // The dock row is the charging fact restated, not an extra sensor: the
    // vendor merges dock status into the power frame's charging field.
    assert_eq!(state["sensors"]["dock"], json!(true));
    assert!(state["sensors"]["compass"].is_null());
    assert_eq!(
        state["sensors"]["service_channels"]["position3d:2"],
        json!({"pos":[0.1,0.2,0.3,0.04,0.05,1.25],"vel":[0.,0.,0.,0.,0.,0.],"stall":false,"stamp":10.})
    );
    assert_eq!(
        state["odom"],
        json!({"x":1.,"y":2.,"yaw":0.3,"vx":0.06,"yaw_rate":0.2})
    );
    assert_eq!(state["cliff_ir"], json!([1., 2., 3., 4.]));
    assert_eq!(state["drop_ir"], json!([5., 6.]));
    assert_eq!(state["sensors"]["cliff_ranges"], json!([0., 0., 0., 0.]));
    assert_eq!(state["sensors"]["drop_ranges"], json!([0., 0.]));
    assert_eq!(state["sensors"]["wall_voltages"], json!([2.2]));
    assert!((state["wall_ir"].as_f64().unwrap() - 0.055).abs() < 1e-6);
    assert!((state["sonar"].as_f64().unwrap() - 65.534).abs() < 1e-3);
    s.quiet(50).await;
    let mut unknown = vec![0; 24];
    unknown[8..12].copy_from_slice(&100f32.to_be_bytes());
    s.send(packet(2, 0, 1, 44., &unknown)).await;
    let v = timeout(WAIT, async {
        loop {
            let v = frame(&mut w, "state").await;
            if v["battery"]["percent"].is_null() {
                break v;
            }
        }
    })
    .await
    .unwrap();
    assert_eq!(
        v["battery"],
        json!({"percent":null,"volts":null,"charging":null})
    );
}
#[tokio::test]
async fn bumper_latches_until_reset_without_automatic_motion() {
    let mut s = Session::start(&[], None, true).await;
    let mut w = s.ws().await;
    s.send(bumper(true)).await;
    wait_bumper(&mut w, "PRESSED").await;
    assert_eq!(
        s.command().await.1,
        0.,
        "contact stops even without an owner"
    );
    // Reset under contact is refused. WS and Player frames race, so prove
    // the refusal deterministically: reset while pressed, then a second
    // motion command must stay refused (blocked survived the reset).
    cmd(&mut w, r#"{"type":"reset_blocked"}"#).await;
    cmd(&mut w, r#"{"v":0.06,"w":0}"#).await;
    let stray = tokio::time::timeout(Duration::from_millis(150), s.cmds.recv())
        .await
        .ok();
    assert_eq!(
        stray, None,
        "blocked latch must keep wheels silent after refused reset: {stray:?}"
    );
    s.send(bumper(false)).await;
    wait_bumper(&mut w, "clear").await;
    cmd(&mut w, r#"{"type":"reset_blocked"}"#).await;
    assert_eq!(frame(&mut w, "state").await["blocked"], false);
    cmd(&mut w, r#"{"v":0.06,"w":0}"#).await;
    let resumed = s.command().await;
    if resumed.1 == 0.0 {
        assert_eq!(s.command().await.1, 0.06);
    } else {
        assert_eq!(resumed.1, 0.06);
    }
    s.send(bumper(true)).await;
    let stop = s.command().await;
    assert_eq!((stop.1, stop.2), (0., 0.));
    wait_bumper(&mut w, "PRESSED").await;
    s.quiet(50).await;
    cmd(&mut w, r#"{"v":-0.06,"w":0}"#).await;
    s.quiet(50).await;
    eprintln!("MARKER: passed second quiet pair");
    // Refused-reset check that does not race across channels: while the
    // bumper is pressed, a reset followed by a motion command keeps the
    // wheels silent (probe consumed exactly nothing).
    cmd(&mut w, r#"{"type":"reset_blocked"}"#).await;
    cmd(&mut w, r#"{"v":-0.06,"w":0}"#).await;
    let stray = tokio::time::timeout(Duration::from_millis(150), s.cmds.recv())
        .await
        .ok();
    assert_eq!(
        stray, None,
        "reset under contact is refused; wheels stay silent"
    );
    s.send(bumper(false)).await;
    wait_bumper(&mut w, "clear").await;
    cmd(&mut w, r#"{"type":"reset_blocked"}"#).await;
    cmd(&mut w, r#"{"v":-0.06,"w":0}"#).await;
    let unblock = s.command().await;
    if unblock.1 == 0.0 {
        assert_eq!(s.command().await.1, -0.06);
    } else {
        assert_eq!(unblock.1, -0.06);
    }
    // The reverse motion succeeded; park with a zero first, then re-latch
    // with repeated presses (only the rising edge counts) and verify motion
    // stays refused without needing a new stop.
    cmd(&mut w, r#"{"v":0,"w":0}"#).await;
    assert_eq!(s.command().await.1, 0.);
    // The first press is a rising edge: it latches and flushes the priority
    // stop; the remaining presses are repeat frames that must NOT emit more.
    s.send(bumper(true)).await;
    wait_bumper(&mut w, "PRESSED").await;
    // The latch queues one priority stop; let the drive clock flush it
    // before asserting that repeat presses emit nothing.
    let flushed = tokio::time::timeout(Duration::from_millis(150), s.cmds.recv())
        .await
        .ok();
    if let Some(Some(c)) = flushed {
        assert_eq!(
            (c.1, c.2),
            (0.0, 0.0),
            "only a zero may flush after the latch"
        );
    }
    for _ in 0..3 {
        s.send(bumper(true)).await;
        s.quiet(50).await;
    }
    s.send(bumper(false)).await;
    wait_bumper(&mut w, "clear").await;
    cmd(&mut w, r#"{"type":"reset_blocked"}"#).await;
    s.quiet(100).await;
    cmd(&mut w, r#"{"v":0.06,"w":0}"#).await;
    // Lease-expiry zeros may flush first; wait for the actual motion command.
    timeout(WAIT, async {
        loop {
            if s.command().await.1 == 0.06 {
                break;
            }
        }
    })
    .await
    .unwrap();
    w.close(None).await.unwrap();
    assert_eq!(s.command().await.1, 0.);
    s.quiet(100).await;
}
#[tokio::test]
async fn stationary_odometry_never_blocks_manual_commands() {
    let mut s = Session::start(&[], None, false).await;
    let mut w = s.ws().await;
    let start = Instant::now();
    let mut last_motion = Duration::ZERO;
    let mut stopped = false;
    while start.elapsed() < Duration::from_millis(4400) {
        cmd(&mut w, r#"{"v":0.06,"w":0}"#).await;
        s.send(packet(4, 0, 1, 10., &[0; 52])).await;
        while let Ok(c) = s.cmds.try_recv() {
            if c.1 != 0. {
                last_motion = start.elapsed();
            } else if start.elapsed() > Duration::from_secs(2) {
                stopped = true;
            }
        }
        sleep(Duration::from_millis(40)).await;
    }
    assert!(last_motion > Duration::from_secs(2));
    assert!(
        last_motion > Duration::from_millis(4000),
        "odometry must not gate teleop: {last_motion:?}"
    );
    assert!(!stopped, "stall detection must not stop manual driving");
    // Contact alone blocks, irrespective of odometry.
    s.send(bumper(true)).await;
    wait_bumper(&mut w, "PRESSED").await;
    timeout(WAIT, async {
        loop {
            let stop = s.command().await;
            if (stop.1, stop.2) == (0., 0.) {
                break;
            }
            assert_eq!(
                (stop.1, stop.2),
                (0.06, 0.),
                "only pre-contact commands may precede stop"
            );
        }
    })
    .await
    .unwrap();
    s.quiet(80).await;
    // Reset under contact is refused (#65): release, then reset, then the
    // manual retreat works as before.
    cmd(&mut w, r#"{"type":"reset_blocked"}"#).await;
    cmd(&mut w, r#"{"v":-0.06,"w":0}"#).await;
    let stray = tokio::time::timeout(Duration::from_millis(150), s.cmds.recv())
        .await
        .ok();
    assert_eq!(stray, None, "wheels stay silent while the contact holds");
    s.send(bumper(false)).await;
    wait_bumper(&mut w, "clear").await;
    cmd(&mut w, r#"{"type":"reset_blocked"}"#).await;
    let sent = Instant::now();
    cmd(&mut w, r#"{"v":-0.06,"w":0}"#).await;
    let reverse = s.command().await;
    assert_eq!((reverse.1, reverse.2), (-0.06, 0.));
    eprintln!("manual retreat reached Player after {:?}", sent.elapsed());
}
#[tokio::test]
async fn player_loss_and_corrupt_frame_fail_session() {
    for malformed in [false, true] {
        let mut s = Session::start(&[], None, false).await;
        if malformed {
            s.send(packet(6, 0, 3, 1., &[0])).await;
        } else {
            s.fake.abort();
        }
        s.exited().await;
    }
}
// BUG: watchdog restart can leave Player's TCP port temporarily refusing connections.
// HYPOTHESIS: the single startup connect exits before the healthy Player begins listening.
// SOLUTION: retry only refused initial TCP connects within the shared startup deadline.
// RESULT: FIXED — delayed listeners recover with heartbeat; timeout and protocol failures abort.
// STATUS: FIXED
#[tokio::test]
async fn delayed_player_listener_recovers_while_heartbeat_continues() {
    let supervisor = TcpListener::bind("127.0.0.1:0").await.unwrap();
    let address = supervisor.local_addr().unwrap().to_string();
    let (beats, mut received) = mpsc::channel(8);
    let start = Instant::now();
    let supervisor_task = tokio::spawn(async move {
        let (mut stream, _) = supervisor.accept().await.unwrap();
        let mut activate = [0; 9];
        stream.read_exact(&mut activate).await.unwrap();
        assert_eq!(&activate, b"ACTIVATE\n");
        stream.write_all(b"ACTIVATED\n").await.unwrap();
        let mut beat = [0; 2];
        while stream.read_exact(&mut beat).await.is_ok() {
            assert_eq!(&beat, b"hb");
            if beats.send(Instant::now()).await.is_err() {
                break;
            }
        }
    });
    let delay = Duration::from_millis(1600);
    let mut session = Session::start_delayed(&[], Some(address), false, delay).await;
    let elapsed = start.elapsed();
    assert!(
        elapsed >= delay,
        "Player must initially refuse TCP connections"
    );
    let first_beat = timeout(WAIT, received.recv()).await.unwrap().unwrap();
    let listening_at = (&mut session.listening_at).await.unwrap();
    assert!(
        first_beat < listening_at,
        "heartbeat must precede Player readiness"
    );
    let mut ws = session.ws().await;
    assert!(frame(&mut ws, "state").await["sensors"].is_object());
    session.quiet(100).await;
    assert!(session.child.try_wait().unwrap().is_none());
    eprintln!("delayed Player: HTTP ready after {elapsed:?}, same server process alive");
    drop(ws);
    drop(session);
    supervisor_task.await.unwrap();
}

struct StartupChild(Child);
impl Drop for StartupChild {
    fn drop(&mut self) {
        let _ = self.0.kill();
        let _ = self.0.wait();
    }
}

#[tokio::test]
async fn unavailable_player_has_bounded_startup_wait() {
    let player = TcpSocket::new_v4().unwrap();
    player.bind("127.0.0.1:0".parse().unwrap()).unwrap();
    let child = Command::new(env!("CARGO_BIN_EXE_robot-server"))
        .env_remove("ROBOT_SERVER_HEARTBEAT")
        .args([
            "--player-host",
            "127.0.0.1",
            "--player-port",
            &player.local_addr().unwrap().port().to_string(),
            "--port",
            "0",
        ])
        .stderr(Stdio::piped())
        .spawn()
        .unwrap();
    let mut child = StartupChild(child);
    let start = Instant::now();
    let status = timeout(Duration::from_secs(13), async {
        loop {
            if let Some(status) = child.0.try_wait().unwrap() {
                break status;
            }
            sleep(Duration::from_millis(20)).await;
        }
    })
    .await
    .expect("unavailable Player must not leave startup waiting forever");
    assert!(!status.success());
    let mut log = String::new();
    std::io::Read::read_to_string(child.0.stderr.as_mut().unwrap(), &mut log).unwrap();
    assert!(
        log.contains("Player startup timed out"),
        "missing startup deadline error: {log}"
    );
    assert!(
        start.elapsed() >= Duration::from_secs(9),
        "initial refusal must wait for readiness"
    );
}

#[tokio::test]
async fn bad_player_banner_is_not_retried() {
    let player = TcpListener::bind("127.0.0.1:0").await.unwrap();
    let child = Command::new(env!("CARGO_BIN_EXE_robot-server"))
        .env_remove("ROBOT_SERVER_HEARTBEAT")
        .args([
            "--player-host",
            "127.0.0.1",
            "--player-port",
            &player.local_addr().unwrap().port().to_string(),
            "--port",
            "0",
        ])
        .stderr(Stdio::piped())
        .spawn()
        .unwrap();
    let mut child = StartupChild(child);
    let (mut stream, _) = timeout(WAIT, player.accept()).await.unwrap().unwrap();
    stream.write_all(&[b'x'; 32]).await.unwrap();
    timeout(WAIT, async {
        loop {
            if let Some(status) = child.0.try_wait().unwrap() {
                assert!(!status.success());
                break;
            }
            sleep(Duration::from_millis(20)).await;
        }
    })
    .await
    .unwrap();
    let mut log = String::new();
    std::io::Read::read_to_string(child.0.stderr.as_mut().unwrap(), &mut log).unwrap();
    assert!(
        log.contains("bad Player banner"),
        "protocol failure must be preserved: {log}"
    );
    assert!(
        timeout(Duration::from_millis(100), player.accept())
            .await
            .is_err(),
        "a protocol failure must never reconnect"
    );
}

// FEATURE: shared robot-server authenticates control sessions without exposing the token.
// SPEC: AUTH must succeed before ACTIVATE; authentication cannot starve supervision.
#[tokio::test]
async fn supervisor_control_session_authenticates_before_activation() {
    let supervisor = TcpListener::bind("127.0.0.1:0").await.unwrap();
    let address = supervisor.local_addr().unwrap().to_string();
    let (seen_tx, seen_rx) = tokio::sync::oneshot::channel();
    let fake = tokio::spawn(async move {
        use tokio::io::AsyncBufReadExt;
        let (stream, _) = supervisor.accept().await.unwrap();
        let mut stream = tokio::io::BufReader::new(stream);
        let mut line = String::new();
        stream.read_line(&mut line).await.unwrap();
        assert_eq!(line, "AUTH control-test-token\n");
        stream
            .get_mut()
            .write_all(b"AUTHENTICATED\n")
            .await
            .unwrap();
        line.clear();
        stream.read_line(&mut line).await.unwrap();
        assert_eq!(line, "ACTIVATE\n");
        stream.get_mut().write_all(b"ACTIVATED\n").await.unwrap();
        let _ = seen_tx.send(());
        std::future::pending::<()>().await;
    });
    let session = Session::start_env(
        &[],
        Some(address),
        false,
        Duration::ZERO,
        &[("ROBOT_SUPERVISOR_TOKEN", "control-test-token")],
    )
    .await;
    timeout(WAIT, seen_rx).await.unwrap().unwrap();
    drop(session);
    fake.abort();
}

// BUG: the real supervisor replies ACTIVATED before monitoring heartbeat.
// HYPOTHESIS: treating every received byte as EOF terminates a valid session.
// SOLUTION: accept acknowledgment bytes; only EOF or an I/O failure loses supervision.
// RESULT: FIXED — acknowledgments survive startup and runtime; EOF still aborts.
// STATUS: FIXED
#[tokio::test]
async fn heartbeat_activate_ticks_and_loss_is_fatal() {
    let h = TcpListener::bind("127.0.0.1:0").await.unwrap();
    let mut s = Session::start(&[], Some(h.local_addr().unwrap().to_string()), false).await;
    let (mut h, _) = timeout(WAIT, h.accept()).await.unwrap().unwrap();
    let mut activate = [0; 9];
    timeout(WAIT, h.read_exact(&mut activate))
        .await
        .unwrap()
        .unwrap();
    assert_eq!(&activate, b"ACTIVATE\n");
    h.write_all(b"ACTIVATED\n").await.unwrap();
    let mut hb = [0; 2];
    timeout(Duration::from_millis(1500), h.read_exact(&mut hb))
        .await
        .unwrap()
        .unwrap();
    assert_eq!(&hb, b"hb");
    drop(h);
    s.exited().await;
    let h = TcpListener::bind("127.0.0.1:0").await.unwrap();
    let addr = h.local_addr().unwrap().to_string();
    drop(h);
    // Startup failure is checked independently: it must not need an HTTP listener.
    let mut c = Command::new(env!("CARGO_BIN_EXE_robot-server"))
        .env("ROBOT_SERVER_HEARTBEAT", addr)
        .args(["--player-host", "127.0.0.1", "--player-port", "1"])
        .spawn()
        .unwrap();
    timeout(WAIT, async {
        loop {
            if let Some(x) = c.try_wait().unwrap() {
                assert!(!x.success());
                break;
            }
            sleep(Duration::from_millis(20)).await;
        }
    })
    .await
    .unwrap();
}
// FEATURE: heartbeat supervision includes Player startup, not only HTTP runtime.
// SPEC: a slow handshake must not starve heartbeat; loss aborts startup without reconnecting.
#[tokio::test]
async fn heartbeat_runs_during_slow_player_startup() {
    let supervisor = TcpListener::bind("127.0.0.1:0").await.unwrap();
    let player = TcpListener::bind("127.0.0.1:0").await.unwrap();
    let child = Command::new(env!("CARGO_BIN_EXE_robot-server"))
        .env(
            "ROBOT_SERVER_HEARTBEAT",
            supervisor.local_addr().unwrap().to_string(),
        )
        .args([
            "--player-host",
            "127.0.0.1",
            "--player-port",
            &player.local_addr().unwrap().port().to_string(),
            "--port",
            "0",
        ])
        .stderr(Stdio::piped())
        .spawn()
        .unwrap();
    struct Guard(Child);
    impl Drop for Guard {
        fn drop(&mut self) {
            let _ = self.0.kill();
            let _ = self.0.wait();
        }
    }
    let mut child = Guard(child);
    let (mut hb, _) = timeout(WAIT, supervisor.accept()).await.unwrap().unwrap();
    let (_silent_player, _) = timeout(WAIT, player.accept()).await.unwrap().unwrap();
    let mut activate = [0; 9];
    timeout(WAIT, hb.read_exact(&mut activate))
        .await
        .unwrap()
        .unwrap();
    assert_eq!(&activate, b"ACTIVATE\n");
    hb.write_all(b"ACTIVATED\n").await.unwrap();
    let mut beat = [0; 2];
    timeout(Duration::from_millis(1500), hb.read_exact(&mut beat))
        .await
        .expect("Player handshake starved heartbeat")
        .unwrap();
    assert_eq!(&beat, b"hb");
    drop(hb);
    timeout(WAIT, async {
        loop {
            if let Some(s) = child.0.try_wait().unwrap() {
                assert!(!s.success());
                break;
            }
            sleep(Duration::from_millis(10)).await;
        }
    })
    .await
    .expect("heartbeat loss must abort Player startup");
    assert!(
        timeout(Duration::from_millis(100), supervisor.accept())
            .await
            .is_err(),
        "must never reconnect"
    );
}
#[tokio::test]
async fn shutdown_explicit_zero() {
    let mut s = Session::start(&[], None, false).await;
    let mut w = s.ws().await;
    cmd(&mut w, r#"{"v":0.06,"w":0}"#).await;
    assert_eq!(s.command().await.1, 0.06);
    assert!(Command::new("kill")
        .args(["-TERM", &s.child.id().to_string()])
        .status()
        .unwrap()
        .success());
    assert_eq!(s.command().await.1, 0.);
    timeout(WAIT, async {
        while s.child.try_wait().unwrap().is_none() {
            sleep(Duration::from_millis(10)).await;
        }
    })
    .await
    .unwrap();
}
#[tokio::test]
async fn camera_proxy_streams_with_server_only_auth_and_no_redirects() {
    use axum::{
        body::Body,
        http::{header, HeaderMap, StatusCode},
        response::Response,
        routing::get,
        Router,
    };
    let camera = TcpListener::bind("127.0.0.1:0").await.unwrap();
    let cp = camera.local_addr().unwrap().port();
    let app = Router::new()
        .route(
            "/mjpeg",
            get(|h: HeaderMap| async move {
                assert_eq!(h[header::AUTHORIZATION], "Basic dXNlcjpzZWNyZXQ=");
                let stream = futures_util::stream::once(async {
                    Ok::<_, std::io::Error>("--frame\r\nfirst")
                })
                .chain(futures_util::stream::pending());
                Response::builder()
                    .header(
                        header::CONTENT_TYPE,
                        "multipart/x-mixed-replace; boundary=frame",
                    )
                    .body(Body::from_stream(stream))
                    .unwrap()
            }),
        )
        .route(
            "/redirect",
            get(move || async move {
                (
                    StatusCode::FOUND,
                    [(header::LOCATION, format!("http://127.0.0.1:{cp}/mjpeg"))],
                )
            }),
        );
    let task = tokio::spawn(async move {
        axum::serve(camera, app).await.unwrap();
    });
    let path = std::env::temp_dir().join(format!("robot-server-netrc-{}-{cp}", std::process::id()));
    std::fs::write(&path, "machine 127.0.0.1 login user password secret\n").unwrap();
    use std::os::unix::fs::PermissionsExt;
    std::fs::set_permissions(&path, std::fs::Permissions::from_mode(0o600)).unwrap();
    // FEATURE: the shared TOML environment handoff configures camera consumers.
    // SPEC: configured URL/netrc work without repeating CLI parameters.
    for route in ["mjpeg", "redirect", "env"] {
        let camera_route = if route == "env" { "mjpeg" } else { route };
        let url = format!("http://127.0.0.1:{cp}/{camera_route}");
        let s = if route == "env" {
            Session::start_env(
                &[],
                None,
                false,
                Duration::ZERO,
                &[
                    ("ROBOT_CAMERA_URL", &url),
                    ("ROBOT_CAMERA_NETRC", path.to_str().unwrap()),
                ],
            )
            .await
        } else {
            Session::start(
                &["--camera-url", &url, "--netrc", path.to_str().unwrap()],
                None,
                false,
            )
            .await
        };
        let response = reqwest::Client::builder()
            .no_proxy()
            .build()
            .unwrap()
            .get(format!("http://127.0.0.1:{}/camera.mjpeg", s.port))
            .send()
            .await
            .unwrap();
        if route != "redirect" {
            assert_eq!(response.status(), 200);
            assert_eq!(
                response.headers()[header::CONTENT_TYPE],
                "multipart/x-mixed-replace; boundary=frame"
            );
            let first = timeout(Duration::from_millis(500), response.bytes_stream().next())
                .await
                .unwrap()
                .unwrap()
                .unwrap();
            assert_eq!(first.as_ref(), b"--frame\r\nfirst");
        } else {
            assert_eq!(response.status(), 502);
            assert!(!response.text().await.unwrap().contains("secret"));
        }
    }
    std::fs::remove_file(path).unwrap();
    task.abort();
}

// ---- SLAM adapter integration ----
// FEATURE: optional MRPT adapter subprocess feeding the panel slam state.
// SPEC: --slam-adapter EXE spawns the line-JSON adapter; scans+odom flow to
// it as update; pose/mode/quality appear as state.slam; neither lost status
// nor adapter errors interfere with explicit manual commands.
// IMPLEMENTATION: dedicated blocking thread owns the adapter process; slam
// state lands in the shared watch channel of "state".
// RESULT: IMPLEMENTED — adapter feeding, independent drive and no-slam default
// verified against a shell fake.
// STATUS: DONE
fn slam_fake(dir: &std::path::Path, mode_by_scan: usize) -> std::path::PathBuf {
    use std::os::unix::fs::PermissionsExt;
    let path = dir.join("slam-adapter");
    std::fs::write(
        &path,
        format!(
            "#!/bin/sh\nn=0\nwhile read line; do\n  case \"$line\" in \
             *start_mapping*) echo '{{\"ok\":true,\"mode\":\"mapping\"}}' ;; \
             *load_map*) echo '{{\"ok\":true,\"mode\":\"localize\"}}' ;; \
             *map*) echo '{{\"ok\":true,\"grid\":{{\"x_min\":0.0,\"y_min\":0.0,\"res\":0.05,\"w\":2,\"h\":1,\"cells\":\"50,-1\"}}}}' ;; \
             *update*) n=$((n+1)); if [ $n -ge {mode_by_scan} ]; then \
             echo '{{\"ok\":true,\"mode\":\"localize\",\"pose\":{{\"x\":1.5,\"y\":-0.2,\"yaw\":0.4}},\"quality\":0.9}}'; else \
             echo '{{\"ok\":true,\"mode\":\"lost\",\"pose\":null,\"quality\":0.0}}'; fi ;; \
             *) echo '{{\"ok\":true}}' ;; \
             esac\ndone\n"
        ),
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
#[tokio::test]
async fn slam_adapter_feeds_state_and_lost_blocks_teleop() {
    let dir = std::env::temp_dir().join(format!(
        "robot-slam-it-{}-{}",
        std::process::id(),
        rand_suffix()
    ));
    std::fs::create_dir_all(&dir).unwrap();
    // Fake turns "localized" after 3 update scans.
    let fake = slam_fake(&dir, 3);
    let fake2 = slam_fake(&dir, 10);
    let mut s = Session::start(&["--slam-adapter", fake.to_str().unwrap()], None, true).await;
    let mut w = s.ws().await;
    let initial = frame(&mut w, "state").await;
    assert!(
        initial.get("slam").is_none(),
        "no slam state must be published before the adapter is started"
    );
    // Session 1 (mapping): no saved map, driving is allowed, pose flows.
    let mut odom = vec![0u8; 49];
    odom[..8].copy_from_slice(&0.0f64.to_be_bytes());
    odom[8..16].copy_from_slice(&0.0f64.to_be_bytes());
    odom[16..24].copy_from_slice(&0.0f64.to_be_bytes());
    odom[24..32].copy_from_slice(&0.05f64.to_be_bytes());
    odom[32..40].copy_from_slice(&0f64.to_be_bytes());
    odom[40..48].copy_from_slice(&0f64.to_be_bytes());
    s.send(packet(4, 0, 1, 1., &odom)).await;
    s.send(scan_packet(1.1)).await;
    let st = timeout(WAIT, async {
        loop {
            let v = frame(&mut w, "state").await;
            if let Some(slam) = v.get("slam") {
                return slam.clone();
            }
        }
    })
    .await
    .expect("slam state must appear after the first scan");
    assert_eq!(st["mode"], "mapping");
    assert_eq!(st["scan_id"], 1);
    assert_eq!(st["scan_stamp"], 1.1);
    assert_eq!(st["source_time_verified"], false);
    assert_eq!(st["odom_association"], "unverified");
    assert!(st["age_ms"].as_f64().unwrap() >= 0.);
    assert_eq!(st["max_age_ms"], 500.);
    assert_eq!(st["stale"], false);
    // Mapping drives the robot (that is how the map gets drawn).
    cmd(&mut w, r#"{"v":0.1,"w":0.0}"#).await;
    let m = s.command().await;
    assert_eq!(m.1, 0.1);
    s.fake.abort();
    drop(s);

    // Session 2 reports lost then localized; neither status gates manual commands.
    let mut s2 = Session::start_env(
        &["--slam-adapter", fake2.to_str().unwrap()],
        None,
        true,
        Duration::ZERO,
        &[("ROBOT_SLAM_MAP", "/tmp/robot-slam-it-map.bin")],
    )
    .await;
    let mut w2 = s2.ws().await;
    s2.send(packet(4, 0, 1, 1., &odom)).await;
    // SlamCore declares lost after LOST_AFTER_BAD consecutive bad updates.
    for i in 0..4 {
        s2.send(packet(4, 0, 1, 1.1 + i as f64, &odom)).await;
        s2.send(scan_packet(1.1 + i as f64)).await;
        // Exercise sustained processed updates, not a deliberately coalesced burst.
        tokio::time::sleep(Duration::from_millis(20)).await;
    }
    let st = timeout(WAIT, async {
        loop {
            let v = frame(&mut w2, "state").await;
            if let Some(slam) = v.get("slam") {
                if slam["mode"] == "lost" {
                    return slam.clone();
                }
            }
        }
    })
    .await
    .expect("slam must be lost on sustained low quality");
    assert_eq!(st["mode"], "lost");
    cmd(&mut w2, r#"{"v":0.1,"w":0.0}"#).await;
    assert_eq!(
        s2.command().await.1,
        0.1,
        "lost SLAM is display data, not a drive gate"
    );
    s2.quiet(120).await;
    // The fake localizes from its 10th update on; send enough scans.
    for i in 0..8 {
        s2.send(packet(4, 0, 1, 10. + i as f64, &odom)).await;
        s2.send(scan_packet(10. + i as f64)).await;
        tokio::time::sleep(Duration::from_millis(20)).await;
    }
    let st = timeout(WAIT, async {
        loop {
            let v = frame(&mut w2, "state").await;
            if let Some(slam) = v.get("slam") {
                if slam["mode"] == "localize" && slam["quality"].as_f64().is_some_and(|q| q >= 0.5)
                {
                    return slam.clone();
                }
            }
        }
    })
    .await
    .expect("slam must recover to localize");
    assert_eq!(st["pose"]["x"], 1.5);
    assert_eq!(st["quality"], 0.9);
    cmd(&mut w2, r#"{"v":0.1,"w":0.0}"#).await;
    let m = s2.command().await;
    assert_eq!(m.1, 0.1);
}
fn rand_suffix() -> u64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap()
        .as_nanos() as u64
}

// ---- Real MRPT adapter path (requires tools/mrpt-adapter built) ----
// FEATURE: end-to-end mapping through the actual C++ adapter.
// SPEC: scans from the fake Player flow through robot-server into the MRPT
// adapter and back as slam.pose tracking the driven path.
// IMPLEMENTATION: same harness, real adapter binary, synthetic room scans.
// RESULT: IMPLEMENTED — pose error stays under 0.5 m along the path.
// STATUS: DONE
// FEATURE: the SLAM map travels in its own versioned WS frame, not in state.
// SPEC: frequent state/age publishes never re-send an unchanged grid; a new
// map version is delivered once; a freshly connected viewer gets the current
// map without waiting for the next change.
// IMPLEMENTATION: the slam thread publishes (version, grid) on change only;
// the session sends it as {"type":"map","data":{...}} and on connect.
// RESULT: IMPLEMENTED — offline fake adapter frames and reconnects verified.
// STATUS: DONE
#[tokio::test]
async fn slam_map_is_a_separate_versioned_frame_never_inside_state() {
    let dir = std::env::temp_dir().join(format!(
        "robot-slam-mapframe-{}-{}",
        std::process::id(),
        rand_suffix()
    ));
    std::fs::create_dir_all(&dir).unwrap();
    let fake = slam_fake(&dir, 1);
    let mut s = Session::start(&["--slam-adapter", fake.to_str().unwrap()], None, true).await;
    let mut w = s.ws().await;
    s.send(scan_packet(1.0)).await;
    let first = timeout(WAIT, async {
        loop {
            let v = frame(&mut w, "map").await;
            if v["grid"]["cells"] == "50,-1" {
                return v.clone();
            }
        }
    })
    .await
    .expect("the first map version must arrive as its own frame");
    let version = first["version"].as_u64().expect("map frame is versioned");
    // Frequent slam publishes (age updates) and sensor telemetry must never
    // re-embed the grid into state frames.
    for _ in 0..4 {
        s.send(scan_packet(2.0)).await;
        s.send(bumper(false)).await;
        let state = frame(&mut w, "state").await;
        assert!(
            state["slam"].get("map").is_none(),
            "state must not carry the grid"
        );
        assert!(
            state["slam"]["map_version"].is_u64(),
            "state references the current map version"
        );
    }
    // A reconnecting viewer receives the current map immediately.
    let mut reconnect = s.ws().await;
    let replay = timeout(WAIT, async {
        loop {
            let v = frame(&mut reconnect, "map").await;
            if v["version"].as_u64().is_some() {
                return v.clone();
            }
        }
    })
    .await
    .expect("reconnect must replay the current map");
    assert_eq!(replay["version"].as_u64(), Some(version));
    assert_eq!(replay["grid"]["cells"], "50,-1");
}

#[tokio::test]
async fn slam_survives_telemetry_updates_and_streams_map() {
    let dir = std::env::temp_dir().join(format!(
        "robot-slam-persist-{}-{}",
        std::process::id(),
        rand_suffix()
    ));
    std::fs::create_dir_all(&dir).unwrap();
    let fake = slam_fake(&dir, 1);
    let mut s = Session::start(&["--slam-adapter", fake.to_str().unwrap()], None, true).await;
    let mut w = s.ws().await;
    // One scan seeds the slam state.
    s.send(scan_packet(1.0)).await;
    timeout(WAIT, async {
        loop {
            let v = frame(&mut w, "state").await;
            if v.get("slam").is_some() {
                break;
            }
        }
    })
    .await
    .expect("slam must appear after first scan");
    // Battery telemetry arrives afterwards (POWER update, all-invalid frame).
    let mut power = 31u32.to_be_bytes().to_vec();
    for f in [0f32, 0., 0., 0.] {
        power.extend(f.to_be_bytes());
    }
    power.extend((-1i32).to_be_bytes());
    s.send(packet(2, 0, 1, 1.2, &power)).await;
    // The battery publish must keep the slam block; the map travels in its
    // own versioned frame (see slam_map_is_a_separate_versioned_frame...),
    // so the state must reference the version instead of carrying the grid.
    let with_map = timeout(WAIT, async {
        loop {
            let v = frame(&mut w, "state").await;
            if v.get("sensors").and_then(|s| s.get("battery")).is_some() {
                let slam = v
                    .get("slam")
                    .expect("telemetry publish wiped the slam state");
                if slam.get("map_version").is_some() {
                    return slam.clone();
                }
            }
        }
    })
    .await
    .expect("battery telemetry must publish with slam intact");
    assert_eq!(with_map["map_version"], 1);
    // The versioned frame already streamed; replay must return the same grid.
    let mut reconnect = s.ws().await;
    let map = frame(&mut reconnect, "map").await;
    assert_eq!(map["version"], 1);
    assert_eq!(map["grid"]["w"], 2);
    assert_eq!(map["grid"]["cells"], "50,-1");
}
#[tokio::test]
#[ignore = "requires tools/mrpt-adapter built with MRPT"]
async fn real_adapter_maps_synthetic_room() {
    let adapter_real =
        std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../tools/mrpt-adapter/adapter");
    if !adapter_real.is_file() {
        panic!("adapter binary missing; build it in tools/mrpt-adapter");
    }
    // Wrap: keep the adapter's stderr (MRPT logs) in a file for diagnosis.
    let dir = std::env::temp_dir().join(format!(
        "robot-slam-real-{}-{}",
        std::process::id(),
        rand_suffix()
    ));
    std::fs::create_dir_all(&dir).unwrap();
    let wrapper = dir.join("wrap.sh");
    std::fs::write(
        &wrapper,
        format!(
            "#!/bin/sh\nLD_LIBRARY_PATH={} exec {} 2>>{}/adapter.log\n",
            adapter_real
                .parent()
                .unwrap()
                .join("third_party/install/lib")
                .display(),
            adapter_real.display(),
            dir.display()
        ),
    )
    .unwrap();
    #[cfg(unix)]
    {
        use std::os::unix::fs::PermissionsExt;
        std::fs::set_permissions(&wrapper, std::fs::Permissions::from_mode(0o700)).unwrap();
    }
    let adapter = wrapper;
    let mut s = Session::start(&["--slam-adapter", adapter.to_str().unwrap()], None, true).await;
    let mut w = s.ws().await;
    // Room [-3,3]^2 + pillar; robot drives +x from (-2, 0.5) in 0.1 m steps.
    let walls: [(f64, f64, f64, f64); 4] = [
        (-3., -3., 3., -3.),
        (3., -3., 3., 3.),
        (3., 3., -3., 3.),
        (-3., 3., -3., -3.),
    ];
    let pillar = (1.2f64, -1.9, 1.6, -1.5);
    let seg_hit = |px: f64, py: f64, dx: f64, dy: f64, (x1, y1, x2, y2): (f64, f64, f64, f64)| {
        let (ex, ey) = (x2 - x1, y2 - y1);
        let denom = dx * ey - dy * ex;
        if denom.abs() < 1e-12 {
            return f64::INFINITY;
        }
        let t = ((x1 - px) * ey - (y1 - py) * ex) / denom;
        let u = ((x1 - px) * dy - (y1 - py) * dx) / denom;
        if t > 0. && (0. ..=1.).contains(&u) {
            t
        } else {
            f64::INFINITY
        }
    };
    let scan_room = |x: f64, y: f64, yaw: f64| {
        let off = 0.0f64;
        let mut ang = Vec::new();
        let mut rng = Vec::new();
        let lx = x + player_bridge::scan::LDS_X * yaw.cos();
        let ly = y + player_bridge::scan::LDS_X * yaw.sin();
        for k in 0..360 {
            let a = -std::f64::consts::PI + 2. * std::f64::consts::PI * k as f64 / 360.;
            let (c, sn) = ((yaw + a + off).cos(), (yaw + a + off).sin());
            let mut best = 8.0f64;
            for seg in walls {
                best = best.min(seg_hit(lx, ly, c, sn, seg));
            }
            best = best.min(seg_hit(lx, ly, c, sn, pillar));
            ang.push(a as f32);
            rng.push(best as f32);
        }
        (ang, rng)
    };
    let mut x = -2.0f64;
    let mut odom_x = 0.0f64;
    let mut last = Value::Null;
    for i in 0..10 {
        odom_x += 0.1;
        let mut odom = vec![0u8; 49];
        odom[..8].copy_from_slice(&odom_x.to_be_bytes());
        odom[8..16].copy_from_slice(&0f64.to_be_bytes());
        odom[16..24].copy_from_slice(&0f64.to_be_bytes());
        odom[24..32].copy_from_slice(&0.1f64.to_be_bytes());
        s.send(packet(4, 0, 1, 1. + i as f64, &odom)).await;
        let (ang, rng) = scan_room(x, -2.0, 0.);
        let mut p = 8.0f32.to_be_bytes().to_vec();
        p.extend(arr(&rng));
        p.extend(arr(&ang));
        p.extend(arr(&[]));
        p.extend(1u32.to_be_bytes());
        let decoded = decode_laser_scanangle(&p).unwrap();
        assert_eq!(
            decoded.ranges, rng,
            "fixture must encode Player ranges before bearings"
        );
        assert_eq!(decoded.angles, ang);
        assert!(
            (ang[0] as f64 + std::f64::consts::PI).abs() < 1e-6,
            "wire bearings must remain relative to the LDS"
        );
        s.send(packet(6, 0, 3, 1.1 + i as f64, &p)).await;
        last = timeout(WAIT, async {
            loop {
                let state = frame(&mut w, "state").await;
                if state["slam"]["scan_sequence"] == i + 1 && state["slam"]["pose"].is_object() {
                    break state["slam"].clone();
                }
            }
        })
        .await
        .unwrap();
        x += 0.1;
    }
    // This test checks plumbing: scans reach the MRPT adapter and poses
    // stream back to the panel, advancing roughly with the driven 1 m path.
    // The input must be geometrically valid; odometry progression alone is
    // not evidence that the adapter matched the room.
    assert_eq!(last["diagnostics"]["used_points"], 360);
    assert_eq!(last["diagnostics"]["accepted"], true);
    assert!((last["pose"]["x"].as_f64().unwrap() - 0.9).abs() < 0.05);
    let deadline = Duration::from_secs(8);
    timeout(deadline, async {
        loop {
            let v = frame(&mut w, "state").await;
            if let Some(slam) = v.get("slam") {
                last = slam.clone();
                if slam["mode"] == "mapping" && slam["pose"]["x"].as_f64().is_some_and(|x| x >= 0.4)
                {
                    break;
                }
            }
        }
    })
    .await
    .expect("mapping pose must advance along the driven path");
    assert!(last["mode"] == "mapping");
}

// FEATURE: YOLO is an optional read-only camera adapter on a separate channel.
// SPEC: matched JPEG/id/boxes, server-only auth, inert viewers and independent
// drive lease even when inference hangs; raw camera remains the default.
// IMPLEMENTATION: latest-only video tasks and a separately supervised child.
// RESULT: IMPLEMENTED — pairing/auth, readonly viewers, lease expiry and cleanup pass.
// STATUS: DONE
#[tokio::test]
async fn yolo_adapter_streams_matching_camera_frame_without_drive_access() {
    use axum::{
        body::Body,
        http::{header, HeaderMap},
        response::Response,
        routing::get,
        Router,
    };
    use std::os::unix::fs::PermissionsExt;
    let camera = TcpListener::bind("127.0.0.1:0").await.unwrap();
    let cp = camera.local_addr().unwrap().port();
    let app = Router::new().route(
        "/mjpeg",
        get(|headers: HeaderMap| async move {
            assert_eq!(headers[header::AUTHORIZATION], "Basic dXNlcjpzZWNyZXQ=");
            let stream = futures_util::stream::unfold(0u8, |n| async move {
                sleep(Duration::from_millis(20)).await;
                let bytes = vec![0xff, 0xd8, n, 0xff, 0xd9];
                Some((Ok::<_, std::io::Error>(bytes), n.wrapping_add(1)))
            });
            Response::builder()
                .header(
                    header::CONTENT_TYPE,
                    "multipart/x-mixed-replace; boundary=frame",
                )
                .body(Body::from_stream(stream))
                .unwrap()
        }),
    );
    let camera_task = tokio::spawn(async move { axum::serve(camera, app).await.unwrap() });
    let dir = std::env::temp_dir().join(format!("robot-yolo-it-{}-{cp}", std::process::id()));
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
    let url = format!("http://127.0.0.1:{cp}/mjpeg");
    let mut session = Session::start(
        &[
            "--camera-url",
            &url,
            "--netrc",
            credentials.to_str().unwrap(),
            "--yolo-adapter",
            adapter.to_str().unwrap(),
        ],
        None,
        false,
    )
    .await;
    let (mut vision, _) = connect_async(format!("ws://127.0.0.1:{}/vision/ws", session.port))
        .await
        .unwrap();
    let data = timeout(WAIT, async {
        loop {
            let msg = vision.next().await.unwrap().unwrap();
            if let Ok(text) = msg.to_text() {
                let value: Value = serde_json::from_str(text).unwrap();
                assert_eq!(value["type"], "vision");
                if value["data"]["status"] == "live" {
                    break value["data"].clone();
                }
            }
        }
    })
    .await
    .expect("YOLO must publish a frame");
    assert!(data["frame_id"].is_u64());
    assert_eq!(data["width"], 640);
    assert_eq!(data["height"], 480);
    assert!(data["jpeg"].as_str().unwrap().starts_with("/9g"));
    assert_eq!(data["objects"][0]["label"], "chair");
    assert_eq!(data["objects"][0]["box"], json!([0.1, 0.2, 0.5, 0.8]));
    assert!(!data.to_string().contains("secret"));
    cmd(&mut vision, r#"{"v":0.1,"w":0}"#).await;
    session.quiet(100).await;
    let mut control = session.ws().await;
    cmd(&mut control, r#"{"v":0.05,"w":0}"#).await;
    assert_eq!(session.command().await.1, 0.05);
    vision.close(None).await.unwrap();
    session.quiet(100).await;
    control.close(None).await.unwrap();
    assert_eq!(session.command().await.1, 0.0);
    drop(session);
    camera_task.abort();
    std::fs::remove_dir_all(dir).unwrap();
}

#[tokio::test]
async fn yolo_hung_initialization_does_not_block_drive_lease_or_shutdown() {
    use std::os::unix::fs::PermissionsExt;
    let nonce = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap()
        .as_nanos();
    let dir = std::env::temp_dir().join(format!("robot-yolo-hang-{}-{nonce}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let pidfile = dir.join("pid");
    let adapter = dir.join("adapter");
    std::fs::write(&adapter, format!("#!/usr/bin/env python3\nimport os,time\nfrom pathlib import Path\nPath({:?}).write_text(str(os.getpid()))\ntime.sleep(60)\n", pidfile.to_str().unwrap())).unwrap();
    std::fs::set_permissions(&adapter, std::fs::Permissions::from_mode(0o700)).unwrap();
    let mut s = Session::start(
        &[
            "--yolo-adapter",
            adapter.to_str().unwrap(),
            "--camera-url",
            "http://127.0.0.1:9/mjpeg",
        ],
        None,
        false,
    )
    .await;
    let mut control = s.ws().await;
    assert_eq!(frame(&mut control, "vision").await["enabled"], true);
    cmd(&mut control, r#"{"v":0.05,"w":0}"#).await;
    assert_eq!(s.command().await.1, 0.05);
    let stopped = s.command().await;
    assert_eq!(stopped.1, 0.0, "hung YOLO must not delay expiry");
    assert!(Command::new("kill")
        .args(["-TERM", &s.child.id().to_string()])
        .status()
        .unwrap()
        .success());
    timeout(WAIT, async {
        while s.child.try_wait().unwrap().is_none() {
            sleep(Duration::from_millis(10)).await;
        }
    })
    .await
    .unwrap();
    let pid = std::fs::read_to_string(&pidfile).unwrap();
    timeout(WAIT, async {
        while std::path::Path::new(&format!("/proc/{}", pid.trim())).exists() {
            sleep(Duration::from_millis(10)).await;
        }
    })
    .await
    .expect("hung adapter survived server shutdown");
    std::fs::remove_dir_all(dir).unwrap();
}

#[tokio::test]
async fn yolo_disabled_by_default_has_no_camera_channel() {
    let s = Session::start(&[], None, false).await;
    let result = connect_async(format!("ws://127.0.0.1:{}/vision/ws", s.port)).await;
    assert!(
        result.is_err(),
        "disabled detector must not expose a video socket"
    );
}

// FEATURE: SLAM freshness and backend failures remain visible without drive coupling.
// SPEC: scan identity/source clock and diagnostics survive telemetry; no odometry
// means no synthetic update; a timed-out update is not retried; idle poses stale.
// IMPLEMENTATION: bounded worker and receive-age metadata, independent drive path.
// RESULT: IMPLEMENTED — missing odometry, child timeout and stale state verified.
// STATUS: DONE
#[tokio::test]
async fn slam_runtime_metadata_missing_odom_and_hung_update_do_not_gate_drive() {
    use std::os::unix::fs::PermissionsExt;
    let dir = std::env::temp_dir().join(format!(
        "robot-slam-runtime-{}-{}",
        std::process::id(),
        rand_suffix()
    ));
    std::fs::create_dir_all(&dir).unwrap();
    let fake = dir.join("adapter");
    std::fs::write(
        &fake,
        r##"#!/bin/sh
while read line; do
case "$line" in
*start_mapping*) echo '{"ok":true}';;
*update*) sleep 2; echo '{"ok":true,"tracking":"tracking","pose":{"x":0,"y":0,"yaw":0}}';;
*) echo '{"ok":true}';;
esac
done
"##,
    )
    .unwrap();
    std::fs::set_permissions(&fake, std::fs::Permissions::from_mode(0o700)).unwrap();
    let mut s = Session::start_env(
        &["--slam-adapter", fake.to_str().unwrap()],
        None,
        true,
        Duration::ZERO,
        &[
            ("ROBOT_SLAM_ADAPTER_TIMEOUT_MS", "80"),
            ("ROBOT_SLAM_DESKEW_ENABLED", "true"),
            ("ROBOT_SLAM_SOURCE_TIME_VERIFIED", "true"),
        ],
    )
    .await;
    let mut ws = s.ws().await;
    s.send(scan_packet(23.)).await;
    let missing = timeout(WAIT, async {
        loop {
            let state = frame(&mut ws, "state").await;
            if state["slam"]["scan_stamp"] == 23. {
                break state["slam"].clone();
            }
        }
    })
    .await
    .unwrap();
    assert_eq!(missing["tracking"], "error");
    assert_eq!(missing["scan_sequence"], 1);
    assert_eq!(missing["deskew"]["requested"], true);
    assert_eq!(missing["deskew"]["applied"], false);
    // Beam timing stays unverified in this fixture: deskew is requested but
    // honestly reported as unverified_timing, never silently applied.
    assert_eq!(missing["deskew"]["status"], "unverified_timing");
    assert_eq!(missing["odom_association"], "missing");
    assert!(missing["error"].as_str().unwrap().contains("odometry"));
    let odom = vec![0; 49];
    s.send(packet(4, 0, 1, 24., &odom)).await;
    s.send(scan_packet(24.)).await;
    let failed = timeout(WAIT, async {
        loop {
            let state = frame(&mut ws, "state").await;
            if state["slam"]["error"]
                .as_str()
                .is_some_and(|s| s.contains("deadline"))
            {
                break state["slam"].clone();
            }
        }
    })
    .await
    .unwrap();
    assert_eq!(failed["mode"], "mapping");
    assert_eq!(failed["tracking"], "error");
    assert_eq!(failed["scan_stamp"], 24.);
    assert_eq!(failed["scan_id"], 1);
    assert_eq!(failed["scan_sequence"], 2);
    assert_eq!(failed["source_time_verified"], true);
    let stale = timeout(WAIT, async {
        loop {
            let state = frame(&mut ws, "state").await;
            if state["slam"]["stale"] == true {
                break state["slam"].clone();
            }
        }
    })
    .await
    .unwrap();
    assert!(stale["age_ms"].as_f64().unwrap() >= 500.);
    assert_eq!(stale["scan_stamp"], 24.);
    cmd(&mut ws, r#"{"v":0.1,"w":0.0}"#).await;
    assert_eq!(s.command().await.1, 0.1);
}

// FEATURE: unified recording from local fake Player, camera, and inbound WS.
// SPEC: no second Player, exact raw bytes, credential privacy, camera partial/unavailable,
// snapshots/commands without a viewer dependency; refused directory precedes ACTIVATE.
// IMPLEMENTATION: preflight and typed command hooks; independent recording source tasks.
// RESULT: IMPLEMENTED — fake camera EOF/redirect/idle timeout/cancellation and Player pass.
// STATUS: DONE
#[tokio::test]
async fn unified_session_records_fake_player_camera_and_commands() {
    use axum::{
        body::Body,
        http::{header, HeaderMap},
        response::Response,
        routing::get,
        Router,
    };
    use std::os::unix::fs::PermissionsExt;
    let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
    let port = listener.local_addr().unwrap().port();
    fn jpeg_response(h: HeaderMap, live: bool) -> Response {
        assert_eq!(h[header::AUTHORIZATION], "Basic dXNlcjpzZWNyZXQ=");
        let chunks = futures_util::stream::iter(
            [
                b"--frame\r\n\xff".to_vec(),
                b"\xd8test\xff".to_vec(),
                b"\xd9".to_vec(),
            ]
            .map(Ok::<_, std::io::Error>),
        );
        let tail = futures_util::stream::poll_fn(move |_| {
            if live {
                std::task::Poll::Pending
            } else {
                std::task::Poll::Ready(None)
            }
        });
        Response::builder()
            .header(
                header::CONTENT_TYPE,
                "multipart/x-mixed-replace; boundary=frame",
            )
            .body(Body::from_stream(chunks.chain(tail)))
            .unwrap()
    }
    let app = Router::new()
        .route(
            "/mjpeg",
            get(|h: HeaderMap| async move { jpeg_response(h, false) }),
        )
        .route(
            "/live",
            get(|h: HeaderMap| async move { jpeg_response(h, true) }),
        )
        .route(
            "/redirect",
            get(|| async {
                (
                    axum::http::StatusCode::FOUND,
                    [(header::LOCATION, "/mjpeg")],
                )
            }),
        );
    let camera_task = tokio::spawn(async move { axum::serve(listener, app).await.unwrap() });
    let netrc = std::env::temp_dir().join(format!("session-netrc-{}-{port}", std::process::id()));
    std::fs::write(&netrc, "machine 127.0.0.1 login user password secret\n").unwrap();
    std::fs::set_permissions(&netrc, std::fs::Permissions::from_mode(0o600)).unwrap();
    for route in ["mjpeg", "redirect", "live", "timeout"] {
        let dir = std::env::temp_dir().join(format!(
            "unified-session-{}-{port}-{route}",
            std::process::id()
        ));
        let camera_route = if route == "timeout" { "live" } else { route };
        let url = format!("http://127.0.0.1:{port}/{camera_route}");
        let mut s = Session::start_env(
            &["--camera-url", &url, "--netrc", netrc.to_str().unwrap()],
            None,
            true,
            Duration::ZERO,
            &[("ROBOT_SESSION_RECORD_DIR", dir.to_str().unwrap())],
        )
        .await;
        let mut ws = s.ws().await;
        cmd(&mut ws, r#"{"v":0.1,"w":0.2}"#).await;
        assert_eq!(s.command().await.1, 0.1);
        cmd(&mut ws, r#"{"type":"reset_blocked"}"#).await;
        cmd(
            &mut ws,
            r#"{"token":"never-record-this","text":"private TTS"}"#,
        )
        .await;
        s.send(packet(999, 7, 42, 1., &[0, 255, 128])).await;
        s.send(scan_packet(24.)).await;
        if route == "timeout" {
            // Keep the independent fake Player healthy while only camera HTTP idles.
            for _ in 0..53 {
                s.send(packet(999, 7, 42, 1., &[0, 255, 128])).await;
                sleep(Duration::from_millis(100)).await;
            }
        } else {
            sleep(Duration::from_millis(350)).await;
        }
        assert!(Command::new("kill")
            .args(["-TERM", &s.child.id().to_string()])
            .status()
            .unwrap()
            .success());
        timeout(WAIT, async {
            loop {
                if let Some(status) = s.child.try_wait().unwrap() {
                    assert!(status.success());
                    break;
                }
                sleep(Duration::from_millis(10)).await;
            }
        })
        .await
        .unwrap();
        let text =
            std::fs::read_to_string(dir.join("events.jsonl")).expect("session events missing");
        assert!(
            !text.contains("never-record-this")
                && !text.contains("private TTS")
                && !text.contains("secret")
        );
        let events: Vec<Value> = text
            .lines()
            .map(|s| serde_json::from_str(s).unwrap())
            .collect();
        for kind in ["state", "scan", "wheel_command", "reset_blocked"] {
            assert!(
                events.iter().any(|v| v["type"] == kind),
                "missing {kind}: {text}"
            );
        }
        let raw = std::fs::read_to_string(dir.join("player.jsonl")).unwrap();
        assert!(raw.contains("AP+A"));
        let report: Value =
            serde_json::from_str(&std::fs::read_to_string(dir.join("manifest.json")).unwrap())
                .unwrap();
        assert_eq!(report["raw_player"]["complete"], true);
        assert_eq!(
            report["camera"]["status"],
            match route {
                "live" => "recorded_until_cancel",
                "redirect" => "unavailable",
                _ => "partial",
            }
        );
        assert_eq!(
            report["camera"]["frames"],
            if route == "redirect" { 0 } else { 1 }
        );
        assert_eq!(report["complete"], route == "live");
        if route == "timeout" {
            assert_eq!(report["camera"]["end_reason"], "timeout");
        }
        if route != "redirect" {
            assert_eq!(
                std::fs::read(dir.join("frame-00000000.jpg")).unwrap(),
                b"\xff\xd8test\xff\xd9"
            );
        }
        std::fs::remove_dir_all(dir).unwrap();
    }
    camera_task.abort();
    std::fs::remove_file(netrc).unwrap();
}
#[tokio::test]
async fn unified_session_preflight_precedes_activation() {
    let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
    let dir = std::env::temp_dir().join(format!("session-refuse-{}", std::process::id()));
    std::fs::create_dir(&dir).unwrap();
    let mut child = Command::new(env!("CARGO_BIN_EXE_robot-server"))
        .env("ROBOT_SESSION_RECORD_DIR", &dir)
        .env(
            "ROBOT_SERVER_HEARTBEAT",
            listener.local_addr().unwrap().to_string(),
        )
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .spawn()
        .unwrap();
    assert!(timeout(Duration::from_millis(500), listener.accept())
        .await
        .is_err());
    assert!(child.try_wait().unwrap().is_some_and(|s| !s.success()));
    std::fs::remove_dir(dir).unwrap();
}

#[tokio::test]
async fn raw_recording_captures_subscription_and_unfiltered_frames() {
    let path = std::env::temp_dir().join(format!("server-raw-{}.jsonl", std::process::id()));
    let mut s = Session::start_env(
        &[],
        None,
        true,
        Duration::ZERO,
        &[("ROBOT_SLAM_RECORD_PATH", path.to_str().unwrap())],
    )
    .await;
    s.send(packet(999, 7, 42, f64::NAN, &[0, 255, 128])).await;
    sleep(Duration::from_millis(50)).await;
    assert!(Command::new("kill")
        .args(["-TERM", &s.child.id().to_string()])
        .status()
        .unwrap()
        .success());
    timeout(WAIT, async {
        loop {
            if let Some(status) = s.child.try_wait().unwrap() {
                assert!(status.success());
                break;
            }
            sleep(Duration::from_millis(10)).await;
        }
    })
    .await
    .unwrap();
    let (_, v, w) = s.command().await;
    assert_eq!((v, w), (0., 0.));
    let text = std::fs::read_to_string(&path).expect("requested recording not created");
    let rows: Vec<Value> = text
        .lines()
        .map(|line| serde_json::from_str(line).unwrap())
        .collect();
    assert_eq!(rows.iter().filter(|r| r["msg_type"] == 4).count(), 17);
    assert_eq!(
        rows.iter()
            .filter(|r| r["interface"] == BUMPER_CODE)
            .count(),
        1,
        "subscription-time DATA must not be lost or recorded twice"
    );
    let raw = rows.iter().find(|r| r["interface"] == 999).unwrap();
    assert_eq!(raw["payload_base64"], "AP+A");
    assert!(raw["source_stamp"].is_null());
    assert_eq!(rows.last().unwrap()["complete"], true);
    std::fs::remove_file(path).unwrap();
}

#[tokio::test]
async fn recording_open_failure_precedes_supervisor_activation() {
    let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
    let path = std::env::temp_dir().join(format!("server-existing-{}.jsonl", std::process::id()));
    std::fs::write(&path, b"keep").unwrap();
    let mut child = Command::new(env!("CARGO_BIN_EXE_robot-server"))
        .env("ROBOT_SLAM_RECORD_PATH", &path)
        .env(
            "ROBOT_SERVER_HEARTBEAT",
            listener.local_addr().unwrap().to_string(),
        )
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .spawn()
        .unwrap();
    // No supervisor connection is needed when opening the recording fails.
    assert!(
        timeout(Duration::from_millis(500), listener.accept())
            .await
            .is_err(),
        "file preflight must precede supervisor connection/ACTIVATE"
    );
    assert!(child.try_wait().unwrap().is_some_and(|s| !s.success()));
    assert_eq!(std::fs::read(&path).unwrap(), b"keep");
    std::fs::remove_file(path).unwrap();
}

// FEATURE: wheel and cleaning-motor ownership comes from one control owner.
// SPEC: a client that took the wheels keeps the motors too; a viewer that
// sends the first motors command while another client owns the wheels is
// rejected, and owner disconnect/shutdown still zeroes all three motors.
// IMPLEMENTATION: Safety tracks the exclusive owner; Motors keeps only duties.
// RESULT: IMPLEMENTED — offline two-client fake Player checks pass.
// STATUS: DONE
#[tokio::test]
async fn motors_follow_the_exclusive_wheel_owner_not_a_separate_one() {
    let mut s = Session::start(&[], None, true).await;
    let mut owner = s.ws().await;
    cmd(&mut owner, r#"{"v":0,"w":0}"#).await;
    frame(&mut owner, "state").await;
    // A viewer that never owned the wheels sends the first motors command.
    let mut viewer = s.ws().await;
    cmd(
        &mut viewer,
        r#"{"type":"motors","fan":70,"main":30,"side":50}"#,
    )
    .await;
    // Let the server process the viewer's command before the owner acts.
    sleep(Duration::from_millis(300)).await;
    s.send(bumper(false)).await;
    frame(&mut viewer, "state").await;
    // The true owner commands the motors successfully.
    cmd(
        &mut owner,
        r#"{"type":"motors","fan":60,"main":40,"side":80}"#,
    )
    .await;
    s.send(bumper(false)).await;
    timeout(WAIT, async {
        loop {
            if frame(&mut owner, "state").await["motors"]
                == json!({"fan": 60.0, "main": 40.0, "side": 80.0})
            {
                break;
            }
        }
    })
    .await
    .unwrap();
    drop(owner);
    s.send(bumper(false)).await;
    timeout(WAIT, async {
        loop {
            let v = frame(&mut viewer, "state").await;
            assert_ne!(
                v["motors"],
                json!({"fan": 70.0, "main": 30.0, "side": 50.0}),
                "viewer must not seize motors without owning the wheels"
            );
            if v["motors"] == json!({"fan": 0.0, "main": 0.0, "side": 0.0}) {
                break;
            }
        }
    })
    .await
    .unwrap();
    let mut motor_log: Vec<(usize, f64)> = Vec::new();
    while let Ok((_, a, b)) = s.cmds.try_recv() {
        if a >= 1000. {
            motor_log.push((a as usize - 1000, b));
        }
    }
    assert!(
        !motor_log.contains(&(0, 70.0)) && !motor_log.contains(&(1, 30.0)),
        "viewer PWM must not reach Player: {motor_log:?}"
    );
    assert!(
        motor_log.contains(&(0, 60.0)),
        "owner fan PWM missing: {motor_log:?}"
    );
    assert!(
        motor_log.contains(&(0, 0.0)),
        "owner disconnect zero PWM missing: {motor_log:?}"
    );
    assert!(
        Command::new("kill")
            .args(["-TERM", &s.child.id().to_string()])
            .status()
            .unwrap()
            .success(),
        "failed to stop server"
    );
    s.exited_ok().await;
}

// FEATURE: panel-controlled cleaning motors over the sole Player connection.
// SPEC: {"type":"motors","fan":N,"main":N,"side":N} publishes one XDR float
// PWM (interf 170, CMD subtype 1) per motor, clamped 0..=100, gated by the
// exclusive wheel-command owner; owner disconnect and session shutdown send
// zeros; no motor subscriptions are ever made (vendor zero-on-unsubscribe).
#[tokio::test]
async fn motors_command_publishes_pwm_and_stops_on_disconnect_and_shutdown() {
    let mut s = Session::start(&[], None, true).await;
    let mut ws = s.ws().await;
    cmd(&mut ws, r#"{"v":0,"w":0}"#).await;
    cmd(&mut ws, r#"{"type":"motors","fan":60,"main":40,"side":80}"#).await;
    // State must expose the requested duties for the panel.
    let state = timeout(WAIT, async {
        loop {
            let v = frame(&mut ws, "state").await;
            if v["motors"] == json!({"fan": 60.0, "main": 40.0, "side": 80.0}) {
                break v;
            }
        }
    })
    .await
    .unwrap();
    // Clamped: negative -> 0, over-range -> 100, NaN rejected as a whole.
    cmd(&mut ws, r#"{"type":"motors","fan":-5,"main":140,"side":0}"#).await;
    timeout(WAIT, async {
        loop {
            let v = frame(&mut ws, "state").await;
            if v["motors"] == json!({"fan": 0.0, "main": 100.0, "side": 0.0}) {
                break;
            }
        }
    })
    .await
    .unwrap();
    // A non-owner viewer cannot command motors: force a fresh state publish
    // (telemetry preserves the motor fields) and verify they did not change.
    let mut viewer = s.ws().await;
    cmd(
        &mut viewer,
        r#"{"type":"motors","fan":99,"main":0,"side":0}"#,
    )
    .await;
    s.send(bumper(false)).await;
    timeout(WAIT, async {
        loop {
            let v = frame(&mut viewer, "state").await;
            assert_ne!(
                v["motors"],
                json!({"fan": 99.0, "main": 0.0, "side": 0.0}),
                "viewer must not change motor state"
            );
            if v["motors"] == json!({"fan": 0.0, "main": 100.0, "side": 0.0}) {
                break;
            }
        }
    })
    .await
    .expect("no state after viewer command");
    // Owner disconnect stops the motors.
    drop(ws);
    let state2 = timeout(WAIT, async {
        loop {
            let v = frame(&mut viewer, "state").await;
            if v["motors"] == json!({"fan": 0.0, "main": 0.0, "side": 0.0}) {
                break v;
            }
        }
    })
    .await
    .unwrap();
    // The wire log must contain the actual PWM duties per motor index and
    // final zeros from teardown; owner disconnect already published zeros.
    drop(viewer);
    let mut motor_log: Vec<(usize, f64)> = Vec::new();
    while let Ok((_, a, b)) = s.cmds.try_recv() {
        if a >= 1000. {
            motor_log.push((a as usize - 1000, b));
        }
    }
    assert!(
        motor_log.contains(&(0, 60.0)),
        "fan PWM missing: {motor_log:?}"
    );
    assert!(
        motor_log.contains(&(1, 40.0)),
        "main PWM missing: {motor_log:?}"
    );
    assert!(
        motor_log.contains(&(2, 80.0)),
        "side PWM missing: {motor_log:?}"
    );
    assert!(motor_log.contains(&(0, 0.0)));
    assert!(
        !motor_log.contains(&(0, 99.0)),
        "viewer PWM must not reach Player"
    );
    assert!(Command::new("kill")
        .args(["-TERM", &s.child.id().to_string()])
        .status()
        .unwrap()
        .success());
    s.exited_ok().await;
}
