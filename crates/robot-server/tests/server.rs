// FEATURE: standalone HTTP/WebSocket Player server, without ROS.
// SPEC: viewing is inert; exclusive driver, bounded/clamped commands expire in 1s
// even with a silent/fragmented Player; disconnect/shutdown stop; sensor validity,
// explicit beam geometry, supervisor heartbeat and streaming authenticated camera.
// IMPLEMENTATION: independent async framing/drive clock, watch snapshots, exclusive
// client ownership, shared unchanged drive protections and incremental HTTP proxy.
// RESULT: IMPLEMENTED — offline fake Player, HTTP/WS, supervisor and camera checks pass.
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
            for n in 0..8 {
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
        .stdout(Stdio::null())
        .stderr(Stdio::piped());
        if let Some(hb) = hb {
            c.env("ROBOT_SERVER_HEARTBEAT", hb);
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
            assert!(
                this.child.try_wait().unwrap().is_none(),
                "standalone server exited instead of serving HTTP"
            );
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
                    matches!(v["type"].as_str(), Some("state" | "scan")),
                    "unexpected frame {v}"
                );
                if v["type"] == kind {
                    return v["data"].clone();
                }
            }
        }
    })
    .await
    .unwrap()
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
    .unwrap();
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
    assert_eq!((moving.1, moving.2), (0.1, -0.4));
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
        let angle = a + 97.6f64.to_radians();
        assert!((points[i][0].as_f64().unwrap() - (-0.1 + r * angle.cos())).abs() < 1e-5);
        assert!((points[i][1].as_f64().unwrap() - r * angle.sin()).abs() < 1e-5);
    }
    for (i, vs, rs) in [
        (0, vec![2.2], vec![0.055]),
        (1, vec![1., 2., 3., 4.], vec![0., 0.1, 0.2, 0.3]),
        (2, vec![5., 6.], vec![0., 0.4]),
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
    let state = timeout(WAIT, async {
        loop {
            let v = frame(&mut w, "state").await;
            if v.get("odom").is_some() {
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
    assert_eq!(
        state["odom"],
        json!({"x":1.,"y":2.,"yaw":0.3,"vx":0.06,"yaw_rate":0.2})
    );
    assert_eq!(state["cliff_ir"], json!([1., 2., 3., 4.]));
    assert_eq!(state["drop_ir"], json!([5., 6.]));
    assert_eq!(state["sensors"]["cliff_ranges"], json!([0., 0.1, 0.2, 0.3]));
    assert_eq!(state["sensors"]["drop_ranges"], json!([0., 0.4]));
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
async fn bumper_is_bounded_edge_triggered_and_disconnect_cancels_reverse() {
    let mut s = Session::start(&[], None, true).await;
    let mut w = s.ws().await;
    // A contact during read-only viewing updates telemetry but may not initiate motion.
    s.send(bumper(true)).await;
    wait_bumper(&mut w, "PRESSED").await;
    s.quiet(80).await;
    s.send(bumper(false)).await;
    wait_bumper(&mut w, "clear").await;
    cmd(&mut w, r#"{"v":0.06,"w":0}"#).await;
    assert_eq!(s.command().await.1, 0.06);
    s.send(bumper(true)).await;
    let start = s.command().await;
    assert_eq!(start.1, -0.1);
    let end = Instant::now() + Duration::from_millis(1550);
    let mut stop = false;
    while Instant::now() < end {
        s.send(bumper(true)).await;
        cmd(&mut w, r#"{"v":0.06,"w":0}"#).await;
        while let Ok(c) = s.cmds.try_recv() {
            if c.1 == 0. {
                stop = true;
            } else {
                assert_eq!(c.1, -0.1, "forward must not leak during/after held press");
            }
        }
        sleep(Duration::from_millis(40)).await;
    }
    assert!(stop, "reverse must finish with a stop");
    wait_bumper(&mut w, "PRESSED").await;
    s.send(bumper(false)).await;
    wait_bumper(&mut w, "clear").await;
    cmd(&mut w, r#"{"v":0.06,"w":0}"#).await;
    let _ = s.command().await;
    s.send(bumper(true)).await;
    assert_eq!(s.command().await.1, -0.1);
    w.close(None).await.unwrap();
    assert_eq!(s.command().await.1, 0.);
    s.quiet(100).await;
}
#[tokio::test]
async fn stalled_drive_stops_and_holds_commands() {
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
        last_motion < Duration::from_millis(3600),
        "commands continued after stall: {last_motion:?}"
    );
    assert!(stopped);
    s.quiet(80).await;
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
    for route in ["mjpeg", "redirect"] {
        let url = format!("http://127.0.0.1:{cp}/{route}");
        let s = Session::start(
            &["--camera-url", &url, "--netrc", path.to_str().unwrap()],
            None,
            false,
        )
        .await;
        let response = reqwest::Client::builder()
            .no_proxy()
            .build()
            .unwrap()
            .get(format!("http://127.0.0.1:{}/camera.mjpeg", s.port))
            .send()
            .await
            .unwrap();
        if route == "mjpeg" {
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
