// FEATURE: authenticated, idle-only supervisor shutdown for binary replacement.
// SPEC: exact LF command after AUTH; STOPPING and successful exit only when alone;
// BUSY preserves service and ownership; idle shutdown never touches control I/O.
// IMPLEMENTATION: SeqCst stopping gate, lifetime client count and nonblocking owner lock.
// RESULT: IMPLEMENTED — shutdown contracts pass using only local fake control paths.
// STATUS: DONE
use std::io::{Read, Write};
use std::net::{TcpListener, TcpStream};
use std::path::PathBuf;
use std::process::{Child, Command, Stdio};
use std::time::{Duration, Instant};

struct Service {
    child: Child,
    port: u16,
    dir: PathBuf,
}
impl Service {
    fn new(tag: &str, authenticated: bool) -> Self {
        let dir = std::env::temp_dir().join(format!("super-shutdown-{tag}-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        for name in ["send", "recv"] {
            std::fs::write(dir.join(name), "untouched").unwrap();
        }
        let port = TcpListener::bind("127.0.0.1:0")
            .unwrap()
            .local_addr()
            .unwrap()
            .port();
        let mut command = Command::new(env!("CARGO_BIN_EXE_robot-supervisor"));
        command.env_remove("ROBOT_SUPERVISOR_TOKEN");
        if authenticated {
            command.env("ROBOT_SUPERVISOR_TOKEN", "shutdown-secret");
        }
        let child = command
            .args([
                "listen",
                "--listen",
                "127.0.0.1",
                "--heartbeat-port",
                &port.to_string(),
                "--activate-timeout-ms",
                "5000",
            ])
            .env("MASK_OWNER_SEND", dir.join("send"))
            .env("MASK_OWNER_RECV", dir.join("recv"))
            .env("ROBOT_SUPERVISOR_PLAYER", "127.0.0.1:1")
            .env(
                "ROBOT_SUPERVISOR_RESTORE",
                format!("touch {}", dir.join("restored").display()),
            )
            .stdout(Stdio::null())
            .stderr(Stdio::null())
            .spawn()
            .unwrap();
        Self { child, port, dir }
    }
    fn connect(&self) -> TcpStream {
        let deadline = Instant::now() + Duration::from_secs(3);
        loop {
            if let Ok(s) = TcpStream::connect(("127.0.0.1", self.port)) {
                s.set_read_timeout(Some(Duration::from_secs(2))).unwrap();
                s.set_write_timeout(Some(Duration::from_secs(2))).unwrap();
                return s;
            }
            assert!(Instant::now() < deadline, "supervisor never listened");
            std::thread::sleep(Duration::from_millis(10));
        }
    }
    fn auth(&self) -> TcpStream {
        let mut s = self.connect();
        s.write_all(b"AUTH shutdown-secret\n").unwrap();
        assert_eq!(response(&mut s), "AUTHENTICATED\n");
        s
    }
    fn untouched(&self) {
        for name in ["send", "recv"] {
            assert_eq!(
                std::fs::read_to_string(self.dir.join(name)).unwrap(),
                "untouched"
            );
        }
        assert!(!self.dir.join("restored").exists());
    }
    fn wait_success(&mut self) {
        let deadline = Instant::now() + Duration::from_secs(3);
        loop {
            if let Some(status) = self.child.try_wait().unwrap() {
                assert!(status.success(), "shutdown exited {status}");
                return;
            }
            assert!(Instant::now() < deadline, "shutdown did not exit");
            std::thread::sleep(Duration::from_millis(10));
        }
    }
}
impl Drop for Service {
    fn drop(&mut self) {
        let _ = self.child.kill();
        let _ = self.child.wait();
        let _ = std::fs::remove_dir_all(&self.dir);
    }
}
fn response(s: &mut TcpStream) -> String {
    let mut out = Vec::new();
    loop {
        let mut byte = [0];
        s.read_exact(&mut byte).expect("expected protocol response");
        out.push(byte[0]);
        assert!(out.len() <= 256);
        if byte[0] == b'\n' {
            return String::from_utf8(out).unwrap();
        }
    }
}

#[test]
fn shutdown_requires_auth_even_when_legacy_commands_allow_no_token() {
    for configured in [true, false] {
        let mut service = Service::new(if configured { "auth" } else { "no-token" }, configured);
        let mut s = service.connect();
        s.write_all(b"SHUTDOWN\n").unwrap();
        assert_eq!(response(&mut s), "UNAUTHORIZED\n");
        assert!(service.child.try_wait().unwrap().is_none());
        service.untouched();
    }
}

#[test]
fn shutdown_idle_exact_fragmented_line_exits_success_without_control_activity() {
    let mut service = Service::new("idle", true);
    let mut s = service.connect();
    s.write_all(b"AUTH shutdown-secret\nSHUT").unwrap();
    assert_eq!(response(&mut s), "AUTHENTICATED\n");
    s.set_read_timeout(Some(Duration::from_millis(100)))
        .unwrap();
    let error = s.read(&mut [0]).unwrap_err();
    assert!(matches!(
        error.kind(),
        std::io::ErrorKind::WouldBlock | std::io::ErrorKind::TimedOut
    ));
    assert!(service.child.try_wait().unwrap().is_none());
    s.set_read_timeout(Some(Duration::from_secs(2))).unwrap();
    s.write_all(b"DOWN\nPING\n").unwrap();
    assert_eq!(response(&mut s), "STOPPING\n");
    service.wait_success();
    service.untouched();
}

#[test]
fn shutdown_rejects_nonexact_lines() {
    let mut service = Service::new("framing", true);
    for command in [
        "SHUTDOWN extra",
        "SHUTDOWN\r",
        "SHUTDOWNevil",
        " SHUTDOWN",
        "SHUTDOWN ",
    ] {
        let mut s = service.auth();
        writeln!(s, "{command}").unwrap();
        assert_eq!(response(&mut s), "REFUSED\n");
        assert_eq!(s.read(&mut [0]).unwrap(), 0);
    }
    assert!(service.child.try_wait().unwrap().is_none());
    service.untouched();
}

#[test]
fn shutdown_busy_with_another_idle_client_resets_gate_and_preserves_ping() {
    let mut service = Service::new("busy", true);
    let mut idle = service.auth(); // AUTH reply proves accept counted this client.
    let mut shutdown = service.auth();
    shutdown.write_all(b"SHUTDOWN\n").unwrap();
    assert_eq!(response(&mut shutdown), "BUSY\n");
    assert_eq!(shutdown.read(&mut [0]).unwrap(), 0);
    assert!(service.child.try_wait().unwrap().is_none());
    idle.write_all(b"PING\n").unwrap();
    assert_eq!(response(&mut idle), "PONG IDLE\n");
    assert_eq!(idle.read(&mut [0]).unwrap(), 0);
    service.untouched();
    // Retry only BUSY while the completed handler's lifetime guard is retiring.
    let deadline = Instant::now() + Duration::from_secs(2);
    loop {
        let mut s = service.auth();
        s.write_all(b"SHUTDOWN\n").unwrap();
        match response(&mut s).as_str() {
            "STOPPING\n" => break,
            "BUSY\n" => assert!(Instant::now() < deadline, "idle client slot leaked"),
            other => panic!("unexpected shutdown response {other:?}"),
        }
        assert_eq!(s.read(&mut [0]).unwrap(), 0);
        std::thread::sleep(Duration::from_millis(10));
    }
    service.wait_success();
    service.untouched();
}
