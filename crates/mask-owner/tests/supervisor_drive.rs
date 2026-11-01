//! Bounded local drive deadline (#55): the supervisor accepts DRIVE commands
//! with an explicit duration and enforces the zero-velocity deadline itself,
//! independent of the host connection. The fake Player records velocity
//! commands; the rig matches supervisor_e2e.rs.
use std::{
    io::{Read, Write},
    net::{TcpListener, TcpStream},
    path::PathBuf,
    process::{Child, Command, Stdio},
    sync::{Arc, Mutex},
    time::{Duration, Instant},
};

const BIN: &str = env!("CARGO_BIN_EXE_robot-supervisor");

/// Records one velocity command: (vx, vyaw, state).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Velocity {
    pub vx: f64,
    pub vyaw: f64,
    pub state: u8,
}

/// Records one connection lifecycle event: "open" / "closed".
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum ConnEvent {
    Open,
    Closed,
}

/// Minimal Player TCP fake: banner, then accept CMD frames and log velocity.
/// Mirrors the live server: CMD_VEL on position2d is honoured only after the
/// connection subscribed to that device, and a disconnect of a subscribed
/// connection makes the server enqueue a zero (vendor unsubscribe behaviour,
/// see player.rs) — recorded as an implicit shadow zero.
fn spawn_fake_player(log: Arc<Mutex<Vec<Velocity>>>, events: Arc<Mutex<Vec<ConnEvent>>>) -> u16 {
    let listener = TcpListener::bind("127.0.0.1:0").unwrap();
    let port = listener.local_addr().unwrap().port();
    std::thread::spawn(move || {
        for conn in listener.incoming() {
            let Ok(mut conn) = conn else { continue };
            let log = log.clone();
            let events = events.clone();
            std::thread::spawn(move || {
                let mut banner = [0u8; 32];
                banner[..9].copy_from_slice(b"Player v.");
                if conn.write_all(&banner).is_err() {
                    return;
                }
                events.lock().unwrap().push(ConnEvent::Open);
                let mut subscribed: Vec<(u16, u16)> = Vec::new();
                let mut header = [0u8; 40];
                while conn.read_exact(&mut header).is_ok() {
                    let Some(h) = player_bridge::decode_header(&header) else {
                        return;
                    };
                    let mut payload = vec![0u8; h.size as usize];
                    if conn.read_exact(&mut payload).is_err() {
                        return;
                    }
                    // REQ_DEV on the `player` device: grant and ACK the
                    // subscription so later CMD_VEL frames are honoured.
                    if h.msg_type == player_bridge::MSGTYPE_REQ
                        && h.subtype == player_bridge::REQ_DEV
                        && h.addr.interf == player_bridge::PLAYER_CODE
                    {
                        subscribed.push((payload_interf(&payload), payload_index(&payload)));
                        let mut ack = [0u8; 32];
                        ack[19] = 1; // granted access: open
                        let ack_header = player_bridge::MsgHeader {
                            addr: h.addr,
                            msg_type: player_bridge::MSGTYPE_RESP_ACK,
                            subtype: player_bridge::REQ_DEV,
                            timestamp: 0.0,
                            seq: 0,
                            size: ack.len() as u32,
                        };
                        let _ = conn.write_all(&player_bridge::encode_header(&ack_header));
                        let _ = conn.write_all(&ack);
                    }
                    // PLAYER_MSGTYPE_CMD(2) on position2d(4): velocity command.
                    if h.msg_type == 2 && h.addr.interf == 4 && payload.len() == 28 {
                        if subscribed.contains(&(4, 0)) {
                            let vx = f64::from_be_bytes(payload[0..8].try_into().unwrap());
                            let vyaw = f64::from_be_bytes(payload[16..24].try_into().unwrap());
                            let state = payload[24];
                            log.lock().unwrap().push(Velocity { vx, vyaw, state });
                        }
                    }
                }
                // Connection end: a subscribed client's disconnect enqueues the
                // vendor zero — the live robot stops when the writer hangs up.
                if subscribed.contains(&(4, 0)) {
                    log.lock().unwrap().push(Velocity {
                        vx: 0.0,
                        vyaw: 0.0,
                        state: 1,
                    });
                }
                events.lock().unwrap().push(ConnEvent::Closed);
            });
        }
    });
    port
}

fn payload_interf(p: &[u8]) -> u16 {
    u32::from_be_bytes(p[8..12].try_into().unwrap()) as u16
}

fn payload_index(p: &[u8]) -> u16 {
    u32::from_be_bytes(p[12..16].try_into().unwrap()) as u16
}

struct Rig {
    dir: PathBuf,
    port: u16,
    velocities: Arc<Mutex<Vec<Velocity>>>,
    events: Arc<Mutex<Vec<ConnEvent>>>,
    child: Option<Child>,
}

impl Drop for Rig {
    fn drop(&mut self) {
        if let Some(mut child) = self.child.take() {
            let _ = child.kill();
            let _ = child.wait();
        }
        if std::env::var("RIG_KEEP").is_ok() {
            eprintln!("rig kept: {}", self.dir.display());
            return;
        }
        let _ = std::fs::remove_dir_all(&self.dir);
    }
}

fn spawn(tag: &str) -> Rig {
    let dir = std::env::temp_dir().join(format!("super-drive-{tag}-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).unwrap();
    let velocities = Arc::new(Mutex::new(Vec::new()));
    let events = Arc::new(Mutex::new(Vec::new()));
    let player_port = spawn_fake_player(velocities.clone(), events.clone());
    let script = dir.join("restore.sh");
    std::fs::write(&script, "#!/bin/sh\nexit 0\n").unwrap();
    use std::os::unix::fs::PermissionsExt;
    std::fs::set_permissions(&script, std::fs::Permissions::from_mode(0o755)).unwrap();
    let mut rig = Rig {
        dir: dir.clone(),
        port: 0,
        velocities,
        events,
        child: None,
    };
    // FIFOs are untouched by DRIVE; the paths only satisfy the config.
    let send = rig.dir.join("RcMc");
    let recv = rig.dir.join("McRc");
    let _ = std::process::Command::new("mkfifo")
        .args([&send, &recv])
        .status();
    let listener = TcpListener::bind("127.0.0.1:0").unwrap();
    let port = listener.local_addr().unwrap().port();
    drop(listener);
    let dir_err = dir.join("supervisor.err");
    let child = Command::new(BIN)
        .arg("listen")
        .args(["--listen", "127.0.0.1"])
        .arg("--heartbeat-port")
        .arg(port.to_string())
        .arg("--timeout-ms")
        .arg("5000")
        .env("ROBOT_SUPERVISOR_TOKEN", "t0")
        .env("MASK_OWNER_SEND", &send)
        .env("MASK_OWNER_RECV", &recv)
        .env(
            "ROBOT_SUPERVISOR_PLAYER",
            format!("127.0.0.1:{player_port}"),
        )
        .env("ROBOT_SUPERVISOR_RESTORE", script.to_str().unwrap())
        .stderr(std::fs::File::create(&dir_err).unwrap())
        .spawn()
        .unwrap();
    rig.child = Some(child);
    rig.port = port;
    rig
}

impl Rig {
    fn connect(&self) -> TcpStream {
        let deadline = Instant::now() + Duration::from_secs(10);
        loop {
            if let Ok(conn) = TcpStream::connect(("127.0.0.1", self.port)) {
                return conn;
            }
            assert!(Instant::now() < deadline, "supervisor never listened");
            std::thread::sleep(Duration::from_millis(50));
        }
    }
}

fn expect(velocities: &Arc<Mutex<Vec<Velocity>>>, want: Velocity) {
    let deadline = Instant::now() + Duration::from_secs(5);
    loop {
        let all = velocities.lock().unwrap().clone();
        if all.contains(&want) {
            return;
        }
        assert!(
            Instant::now() < deadline,
            "velocity {want:?} never arrived; got {all:?}"
        );
        std::thread::sleep(Duration::from_millis(20));
    }
}

fn read_line(conn: &mut TcpStream) -> String {
    let _ = conn.set_read_timeout(Some(Duration::from_secs(10)));
    let mut bytes = Vec::new();
    let mut byte = [0u8; 1];
    while conn.read(&mut byte).is_ok() && byte[0] != b'\n' {
        bytes.push(byte[0]);
    }
    String::from_utf8_lossy(&bytes).into_owned()
}

/// The velocity writer must keep its Player connection open for the whole
/// drive: the live server enqueues a zero when a subscribed client hangs up
/// (vendor unsubscribe behaviour), so an immediate close after the CMD would
/// kill the motion. Found in the 2026-11-01 live run (#65).
#[test]
fn drive_velocity_connection_stays_open_until_the_zero() {
    let rig = spawn("connhold");
    let mut conn = rig.connect();
    conn.write_all(b"AUTH t0\n").unwrap();
    assert_eq!(read_line(&mut conn), "AUTHENTICATED");
    conn.write_all(b"DRIVE 0.12 0 1200\n").unwrap();
    let reply = read_line(&mut conn);
    assert!(reply.starts_with("DRIVING"), "unexpected reply: {reply}");
    // Halfway through the drive the velocity connection must still be open:
    // no Closed event and no shadow zero after the velocity command.
    std::thread::sleep(Duration::from_millis(400));
    let events = rig.events.lock().unwrap().clone();
    assert!(
        !events.contains(&ConnEvent::Closed),
        "velocity connection closed mid-drive: {events:?}"
    );
    let vels = rig.velocities.lock().unwrap().clone();
    let last = vels.last().copied();
    assert_ne!(
        last,
        Some(Velocity {
            vx: 0.0,
            vyaw: 0.0,
            state: 1
        })
    );
    // Wait out the deadline: the explicit zero then arrives on the SAME
    // still-open connection, and only after it may the connection close.
    let deadline = Instant::now() + Duration::from_secs(5);
    loop {
        let vels = rig.velocities.lock().unwrap().clone();
        let idx = vels
            .iter()
            .rposition(|v| v.vx == 0.12 && v.vyaw == 0.0 && v.state == 1);
        if let Some(i) = idx {
            let after = &vels[i + 1..];
            if after.iter().any(|v| v.vx == 0.0 && v.state == 1) {
                break;
            }
            assert!(
                Instant::now() < deadline,
                "no explicit zero after the drive; got {vels:?}"
            );
        } else {
            assert!(Instant::now() < deadline, "0.12 never arrived: {vels:?}");
        }
        std::thread::sleep(Duration::from_millis(20));
    }
}

/// DRIVE is bounded: the supervisor itself sends the zero at the deadline,
/// even while the client connection stays silent.
#[test]
fn drive_deadline_stops_wheels_without_any_followup() {
    let rig = spawn("deadline");
    let mut conn = rig.connect();
    conn.write_all(b"AUTH t0\n").unwrap();
    assert_eq!(read_line(&mut conn), "AUTHENTICATED");
    conn.write_all(b"DRIVE 0.05 0 400\n").unwrap();
    let reply = read_line(&mut conn);
    assert!(reply.starts_with("DRIVING"), "unexpected reply: {reply}");
    expect(
        &rig.velocities,
        Velocity {
            vx: 0.05,
            vyaw: 0.0,
            state: 1,
        },
    );
    // Say nothing more: the local deadline must fire within a bound.
    let deadline = Instant::now() + Duration::from_secs(5);
    loop {
        let all = rig.velocities.lock().unwrap().clone();
        if all.contains(&Velocity {
            vx: 0.0,
            vyaw: 0.0,
            state: 1,
        }) {
            break;
        }
        assert!(
            Instant::now() < deadline,
            "local deadline did not stop the wheels; got {all:?}"
        );
        std::thread::sleep(Duration::from_millis(20));
    }
}

/// STOP cancels an active bounded drive immediately, and DRIVE arguments are
/// validated server-side.
#[test]
fn drive_stop_and_validation() {
    let rig = spawn("stop");
    let mut conn = rig.connect();
    conn.write_all(b"AUTH t0\n").unwrap();
    assert_eq!(read_line(&mut conn), "AUTHENTICATED");
    // Non-numeric, out-of-range duration, missing fields.
    for bad in [
        "DRIVE x 0 400\n",
        "DRIVE 0.1 0 0\n",
        "DRIVE 0.1 0 61000\n",
        "DRIVE 0.1\n",
    ] {
        conn.write_all(bad.as_bytes()).unwrap();
        let reply = read_line(&mut conn);
        assert_eq!(reply, "INVALID_DRIVE", "for {bad}: {reply}");
    }
    // Over-ceiling speed is clamped (matching the panel contract), not rejected.
    conn.write_all(b"DRIVE 9 0 400\n").unwrap();
    assert!(read_line(&mut conn).starts_with("DRIVING"));
    expect(
        &rig.velocities,
        Velocity {
            vx: 0.5,
            vyaw: 0.0,
            state: 1,
        },
    );
    conn.write_all(b"STOP\n").unwrap();
    assert_eq!(read_line(&mut conn), "STOPPED");
    conn.write_all(b"DRIVE 0.1 0.2 2000\n").unwrap();
    let reply = read_line(&mut conn);
    assert!(reply.starts_with("DRIVING"), "unexpected reply: {reply}");
    // A second concurrent DRIVE is refused while one is active.
    conn.write_all(b"DRIVE 0.1 0 500\n").unwrap();
    assert_eq!(read_line(&mut conn), "DRIVE_BUSY");
    conn.write_all(b"STOP\n").unwrap();
    assert_eq!(read_line(&mut conn), "STOPPED");
    expect(
        &rig.velocities,
        Velocity {
            vx: 0.0,
            vyaw: 0.0,
            state: 1,
        },
    );
    // After STOP a new DRIVE is accepted again.
    conn.write_all(b"DRIVE 0 0 100\n").unwrap();
    let reply = read_line(&mut conn);
    assert!(reply.starts_with("DRIVING"), "unexpected reply: {reply}");
}
