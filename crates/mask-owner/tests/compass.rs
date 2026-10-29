// FEATURE: passive authenticated ST480 observer in the existing supervisor.
// SPEC: exact COMPASS command; unavailable closes only the observer; no control I/O.
// IMPLEMENTATION: read-only evdev source, cached SYN decoder and bounded observer poll loop.
// RESULT: IMPLEMENTED — focused contracts and the affected supervisor suite pass.
// STATUS: DONE
// Import the source seam without exporting it from the mask-owner control library.
#[allow(dead_code)]
#[path = "../src/compass.rs"]
mod compass;
use std::io::{Read, Write};
use std::net::{TcpListener, TcpStream};
use std::process::{Child, Command, Stdio};
use std::time::{Duration, Instant};

struct Supervisor(Child);
impl Drop for Supervisor {
    fn drop(&mut self) {
        let _ = self.0.kill();
        let _ = self.0.wait();
    }
}
fn response(s: &mut TcpStream) -> String {
    let mut out = Vec::new();
    loop {
        let mut b = [0];
        s.read_exact(&mut b).unwrap();
        out.push(b[0]);
        assert!(out.len() < 256);
        if b[0] == b'\n' {
            return String::from_utf8(out).unwrap();
        }
    }
}
#[test]
fn authenticated_unavailable_observer_is_isolated_and_never_opens_control() {
    let port = TcpListener::bind("127.0.0.1:0")
        .unwrap()
        .local_addr()
        .unwrap()
        .port();
    // Regular files record any accidental mask writes; restore command records calls.
    let dir = std::env::temp_dir().join(format!("compass-test-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let send = dir.join("send");
    let recv = dir.join("recv");
    std::fs::write(&send, "untouched").unwrap();
    std::fs::write(&recv, "untouched").unwrap();
    let mut sup = Supervisor(
        Command::new(env!("CARGO_BIN_EXE_robot-supervisor"))
            .args([
                "listen",
                "--listen",
                "127.0.0.1",
                "--heartbeat-port",
                &port.to_string(),
            ])
            .env("ROBOT_SUPERVISOR_TOKEN", "test-secret")
            .env("ROBOT_COMPASS_DEVICE", dir.join("missing"))
            .env("MASK_OWNER_SEND", &send)
            .env("MASK_OWNER_RECV", &recv)
            .env(
                "ROBOT_SUPERVISOR_RESTORE",
                format!("touch {}", dir.join("restored").display()),
            )
            .stdout(Stdio::null())
            .stderr(Stdio::null())
            .spawn()
            .unwrap(),
    );
    let connect = || {
        let deadline = Instant::now() + Duration::from_secs(3);
        loop {
            if let Ok(s) = TcpStream::connect(("127.0.0.1", port)) {
                s.set_read_timeout(Some(Duration::from_secs(2))).unwrap();
                return s;
            }
            assert!(Instant::now() < deadline);
            std::thread::sleep(Duration::from_millis(10));
        }
    };
    let mut bad = connect();
    bad.write_all(b"COMPASS\n").unwrap();
    assert_eq!(response(&mut bad), "UNAUTHORIZED\n");
    let mut s = connect();
    s.write_all(b"AUTH test-secret\nCOMPASS\n").unwrap();
    assert_eq!(response(&mut s), "AUTHENTICATED\n");
    assert_eq!(response(&mut s), "COMPASS_UNAVAILABLE\n");
    assert_eq!(s.read(&mut [0]).unwrap(), 0);
    for command in ["COMPASS extra", "COMPASS\r", "COMPASSevil"] {
        let mut s = connect();
        write!(s, "AUTH test-secret\n{command}\n").unwrap();
        assert_eq!(response(&mut s), "AUTHENTICATED\n");
        assert_eq!(response(&mut s), "REFUSED\n");
    }
    let mut ping = connect();
    ping.write_all(b"AUTH test-secret\nPING\n").unwrap();
    assert_eq!(response(&mut ping), "AUTHENTICATED\n");
    assert_eq!(response(&mut ping), "PONG IDLE\n");
    assert_eq!(std::fs::read_to_string(send).unwrap(), "untouched");
    assert_eq!(std::fs::read_to_string(recv).unwrap(), "untouched");
    assert!(!dir.join("restored").exists());
    assert!(sup.0.try_wait().unwrap().is_none());
    drop(sup);
    std::fs::remove_dir_all(dir).unwrap();
}

// SPEC: native input_event ABI, cached partial updates, drop resync and stop/error.
fn event(kind: u16, code: u16, value: i32) -> compass::Event {
    compass::Event {
        sec: 123,
        usec: 456789,
        kind,
        code,
        value,
    }
}
#[test]
fn native_samples_preserve_unchanged_axes_and_resync_after_drops() {
    use compass::{Decoder, Event};
    assert_eq!(
        compass::NATIVE_EVENT_SIZE,
        if cfg!(target_pointer_width = "64") {
            24
        } else {
            16
        }
    );
    // Serialize actual native C long fields, never a fixed host64 wire structure.
    let mut bytes = Vec::new();
    bytes.extend_from_slice(&(123 as std::os::raw::c_long).to_ne_bytes());
    bytes.extend_from_slice(&(456789 as std::os::raw::c_long).to_ne_bytes());
    bytes.extend_from_slice(&3u16.to_ne_bytes());
    bytes.extend_from_slice(&0x10u16.to_ne_bytes());
    bytes.extend_from_slice(&(-12345i32).to_ne_bytes());
    assert_eq!(Event::decode(&bytes).unwrap(), event(3, 0x10, -12345));
    assert!(Event::decode(&bytes[..bytes.len() - 1]).is_err());
    let mut d = Decoder::new([10, 20, 30]);
    let mut queries = 0;
    let mut query = || {
        queries += 1;
        Ok([40, 50, 60])
    };
    assert!(d
        .push(event(3, 0x10, -12345), &mut query)
        .unwrap()
        .is_none());
    let frame = d.push(event(0, 0, 0), &mut query).unwrap().unwrap();
    assert_eq!(frame.line(), "COMPASS 123.456789 -12345 20 30\n");
    d.push(event(3, 0x11, -2), &mut query).unwrap();
    d.push(event(3, 0x0a, i32::MIN), &mut query).unwrap();
    assert_eq!(
        d.push(event(0, 0, 0), &mut query).unwrap().unwrap().xyz,
        [-12345, -2, i32::MIN]
    );
    d.push(event(0, 3, 0), &mut query).unwrap();
    d.push(event(3, 0x10, 999), &mut query).unwrap();
    assert!(d.push(event(0, 0, 0), &mut query).unwrap().is_none());
    assert_eq!(
        d.push(event(0, 0, 0), &mut query).unwrap().unwrap().xyz,
        [40, 50, 60]
    );
    assert_eq!(queries, 1);
    for status in [1, 2] {
        assert!(d.push(event(3, 0x18, status), &mut || Ok([0; 3])).is_err());
    }
    let mut d = Decoder::new([0; 3]);
    d.push(event(0, 3, 0), &mut || Ok([0; 3])).unwrap();
    assert!(d
        .push(
            event(0, 0, 0),
            &mut || Err(std::io::ErrorKind::Other.into())
        )
        .is_err());
}

use std::os::fd::{AsRawFd, RawFd};
use std::os::unix::net::UnixStream;
struct Fake {
    signal: UnixStream,
    events: std::collections::VecDeque<compass::Event>,
    fail_init: bool,
}
impl compass::Source for Fake {
    fn fd(&self) -> RawFd {
        self.signal.as_raw_fd()
    }
    fn axes(&mut self) -> std::io::Result<[i32; 3]> {
        if self.fail_init {
            Err(std::io::ErrorKind::Other.into())
        } else {
            Ok([1, 2, 3])
        }
    }
    fn next_event(&mut self) -> std::io::Result<Option<compass::Event>> {
        let mut b = [0];
        match self.signal.read(&mut b) {
            Ok(0) => Err(std::io::ErrorKind::UnexpectedEof.into()),
            Ok(_) => Ok(self.events.pop_front()),
            Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => Ok(None),
            Err(e) => Err(e),
        }
    }
}
fn streams() -> (TcpStream, TcpStream) {
    let l = TcpListener::bind("127.0.0.1:0").unwrap();
    let client = TcpStream::connect(l.local_addr().unwrap()).unwrap();
    client
        .set_read_timeout(Some(Duration::from_secs(2)))
        .unwrap();
    (client, l.accept().unwrap().0)
}
fn fake(events: Vec<compass::Event>) -> (Fake, UnixStream) {
    let (s, writer) = UnixStream::pair().unwrap();
    s.set_nonblocking(true).unwrap();
    (
        Fake {
            signal: s,
            events: events.into(),
            fail_init: false,
        },
        writer,
    )
}
#[test]
fn silent_sensor_disconnect_reclaims_reader_and_fd_within_poll_bound() {
    for _ in 0..40 {
        let (client, mut server) = streams();
        let (mut source, _writer) = fake(vec![]);
        let (done, wait) = std::sync::mpsc::channel();
        let worker = std::thread::spawn(move || {
            compass::stream(&mut server, &mut source);
            done.send(()).unwrap();
        });
        drop(client);
        wait.recv_timeout(Duration::from_millis(600))
            .expect("silent sensor leaked observer");
        worker.join().unwrap();
    }
}
#[test]
fn stream_initializes_before_publishing_limits_rate_and_reports_read_errors() {
    let (mut client, mut server) = streams();
    let (mut source, mut writer) = fake(vec![
        event(3, 0x10, -7),
        event(0, 0, 0),
        event(0, 0, 0),
        event(0, 0, 0),
    ]);
    let worker = std::thread::spawn(move || compass::stream(&mut server, &mut source));
    writer.write_all(&[1; 4]).unwrap();
    assert_eq!(response(&mut client), "COMPASS 123.456789 -7 2 3\n");
    // A burst must not produce a second line; EOF still terminates with unavailable.
    std::thread::sleep(Duration::from_millis(120));
    drop(writer);
    assert_eq!(response(&mut client), "COMPASS_UNAVAILABLE\n");
    assert_eq!(client.read(&mut [0]).unwrap(), 0);
    worker.join().unwrap();
    let (mut client, mut server) = streams();
    let (mut source, _writer) = fake(vec![]);
    source.fail_init = true;
    let worker = std::thread::spawn(move || compass::stream(&mut server, &mut source));
    assert_eq!(response(&mut client), "COMPASS_UNAVAILABLE\n");
    worker.join().unwrap();
}
#[test]
fn stop_or_error_pressure_terminates_observer() {
    for status in [1, 2] {
        let (mut client, mut server) = streams();
        let (mut source, mut writer) = fake(vec![event(3, 0x18, status)]);
        let worker = std::thread::spawn(move || compass::stream(&mut server, &mut source));
        writer.write_all(&[1]).unwrap();
        assert_eq!(response(&mut client), "COMPASS_UNAVAILABLE\n");
        worker.join().unwrap();
    }
}
