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

/// Minimal Player TCP fake: banner, then accept CMD frames and log velocity.
fn spawn_fake_player(log: Arc<Mutex<Vec<Velocity>>>) -> u16 {
    let listener = TcpListener::bind("127.0.0.1:0").unwrap();
    let port = listener.local_addr().unwrap().port();
    std::thread::spawn(move || {
        for conn in listener.incoming() {
            let Ok(mut conn) = conn else { continue };
            let log = log.clone();
            std::thread::spawn(move || {
                let mut banner = [0u8; 32];
                banner[..9].copy_from_slice(b"Player v.");
                if conn.write_all(&banner).is_err() {
                    return;
                }
                let mut header = [0u8; 40];
                while conn.read_exact(&mut header).is_ok() {
                    let Some(h) = player_bridge::decode_header(&header) else {
                        return;
                    };
                    let mut payload = vec![0u8; h.size as usize];
                    if conn.read_exact(&mut payload).is_err() {
                        return;
                    }
                    // PLAYER_MSGTYPE_CMD(2) on position2d(4): velocity command.
                    if h.msg_type == 2 && h.addr.interf == 4 && payload.len() == 28 {
                        let vx = f64::from_be_bytes(payload[0..8].try_into().unwrap());
                        let vyaw = f64::from_be_bytes(payload[16..24].try_into().unwrap());
                        let state = payload[24];
                        log.lock().unwrap().push(Velocity { vx, vyaw, state });
                    }
                }
            });
        }
    });
    port
}

struct Rig {
    dir: PathBuf,
    port: u16,
    velocities: Arc<Mutex<Vec<Velocity>>>,
    child: Option<Child>,
}

impl Drop for Rig {
    fn drop(&mut self) {
        if let Some(mut child) = self.child.take() {
            let _ = child.kill();
            let _ = child.wait();
        }
        let _ = std::fs::remove_dir_all(&self.dir);
    }
}

fn spawn(tag: &str) -> Rig {
    let dir = std::env::temp_dir().join(format!("super-drive-{tag}-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).unwrap();
    let velocities = Arc::new(Mutex::new(Vec::new()));
    let player_port = spawn_fake_player(velocities.clone());
    let script = dir.join("restore.sh");
    std::fs::write(&script, "#!/bin/sh\nexit 0\n").unwrap();
    use std::os::unix::fs::PermissionsExt;
    std::fs::set_permissions(&script, std::fs::Permissions::from_mode(0o755)).unwrap();
    let mut rig = Rig {
        dir,
        port: 0,
        velocities,
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
        .stdout(Stdio::null())
        .stderr(Stdio::null())
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

fn expect(velocities: &[Velocity], want: Velocity) {
    let deadline = Instant::now() + Duration::from_secs(5);
    loop {
        let all = velocities.to_vec();
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
        &rig.velocities.lock().unwrap().clone(),
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
        &rig.velocities.lock().unwrap().clone(),
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
        &rig.velocities.lock().unwrap().clone(),
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
