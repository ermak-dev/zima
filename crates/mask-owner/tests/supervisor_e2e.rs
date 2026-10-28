//! End-to-end tests for robot-supervisor: the on-robot process that
//! holds the session mask while the workstation server stays alive.
//!
//! Lifecycle under test:
//!   supervisor listen --heartbeat-port P --timeout T
//!     - waits for ACTIVATE on the heartbeat connection
//!     - applies the session mask, then rolls back when the connection
//!       dies (EOF) or goes silent past the timeout
//!   normal disconnect/timeout path: standby mask, restore command
//!
//! The FIFO/Player fakes are the same shape as rollback.rs; the restore
//! command is a recording script.

use std::io::{Read, Write};
use std::net::TcpStream;
use std::os::unix::fs::{OpenOptionsExt, PermissionsExt};
use std::path::{Path, PathBuf};
use std::process::{Child, Command, Stdio};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

const BIN: &str = env!("CARGO_BIN_EXE_robot-supervisor");

struct Rig {
    dir: PathBuf,
    applied: Arc<Mutex<Vec<u32>>>,
    stop: Arc<AtomicBool>,
    peer: Option<std::thread::JoinHandle<()>>,
}

fn mkfifo(path: &Path) {
    extern "C" {
        fn mkfifo(path: *const std::os::raw::c_char, mode: u32) -> i32;
    }
    let c = std::ffi::CString::new(path.to_str().unwrap()).unwrap();
    assert_eq!(unsafe { mkfifo(c.as_ptr(), 0o666) }, 0);
}

fn open_rw_nb(path: &Path) -> std::fs::File {
    std::fs::OpenOptions::new()
        .read(true)
        .write(true)
        .custom_flags(0o4000)
        .open(path)
        .unwrap()
}

fn spawn_rig(tag: &str) -> Rig {
    use mask_owner::{parse_header, EVENT_ACK, EVENT_CHANGE, MODULE_MC, MODULE_RC, PKT_LEN};
    let dir = std::env::temp_dir().join(format!("super-e2e-{tag}-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).unwrap();
    let send = dir.join("RcMc");
    let recv = dir.join("McRc");
    mkfifo(&send);
    mkfifo(&recv);

    let applied: Arc<Mutex<Vec<u32>>> = Arc::new(Mutex::new(Vec::new()));
    let stop = Arc::new(AtomicBool::new(false));
    let (applied_t, stop_t, send_t, recv_t) = (applied.clone(), stop.clone(), send, recv);
    let peer = std::thread::spawn(move || {
        let mut from_rc = open_rw_nb(&send_t);
        let mut to_rc = open_rw_nb(&recv_t);
        let mut buf = [0u8; PKT_LEN];
        loop {
            if stop_t.load(Ordering::SeqCst) {
                return;
            }
            match from_rc.read(&mut buf) {
                Ok(PKT_LEN) => {
                    let h = parse_header(&buf).unwrap();
                    if h.event != EVENT_CHANGE {
                        continue;
                    }
                    let bitmap = u32::from_le_bytes(buf[12..16].try_into().unwrap());
                    applied_t.lock().unwrap().push(bitmap);
                    let mut ack = [0u8; PKT_LEN];
                    ack[0..2].copy_from_slice(&EVENT_ACK.to_le_bytes());
                    ack[2] = MODULE_MC;
                    ack[3] = MODULE_RC;
                    to_rc.write_all(&ack).unwrap();
                }
                Ok(_) => {}
                Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => {
                    std::thread::sleep(Duration::from_millis(5));
                }
                Err(_) => return,
            }
        }
    });

    Rig {
        dir,
        applied,
        stop,
        peer: Some(peer),
    }
}

impl Rig {
    fn heartbeat_port(&self) -> u16 {
        // Bind, read the port, then release it so the supervisor can
        // take it; a short race window is fine because the supervisor
        // retries nothing — the e2e connect below uses retry.
        let l = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
        let p = l.local_addr().unwrap().port();
        drop(l);
        p
    }

    fn restore_script(&self) -> (PathBuf, PathBuf) {
        let script = self.dir.join("restore.sh");
        std::fs::write(&script, "#!/bin/sh\necho restore >> \"$1\"\n").unwrap();
        std::fs::set_permissions(&script, std::fs::Permissions::from_mode(0o755)).unwrap();
        let calls = self.dir.join("restore.calls");
        std::fs::write(&calls, "").unwrap();
        (script, calls)
    }

    fn spawn_supervisor(&self, heartbeat_port: u16, timeout_ms: u64) -> Child {
        self.spawn_supervisor_with_auth(heartbeat_port, timeout_ms, None)
    }

    fn spawn_supervisor_with_auth(
        &self,
        heartbeat_port: u16,
        timeout_ms: u64,
        token: Option<&str>,
    ) -> Child {
        let (script, calls) = self.restore_script();
        let mut command = Command::new(BIN);
        command.env_remove("ROBOT_SUPERVISOR_TOKEN");
        if let Some(token) = token {
            command.env("ROBOT_SUPERVISOR_TOKEN", token);
        }
        command
            .arg("listen")
            .args(["--listen", "127.0.0.1"])
            .arg("--heartbeat-port")
            .arg(heartbeat_port.to_string())
            .arg("--timeout-ms")
            .arg(timeout_ms.to_string())
            .arg("--activate-timeout-ms")
            .arg("1000")
            .env("MASK_OWNER_SEND", self.dir.join("RcMc"))
            .env("MASK_OWNER_RECV", self.dir.join("McRc"))
            .env("ROBOT_SUPERVISOR_PLAYER", "127.0.0.1:1") // unreachable is fine: zero-vel logs an error but rollback continues
            .env(
                "ROBOT_SUPERVISOR_RESTORE",
                format!("{} {}", script.display(), calls.display()),
            )
            .stdout(Stdio::null())
            .stderr(Stdio::null())
            .spawn()
            .unwrap()
    }

    fn wait_mask(&self, mask: u32) {
        let deadline = Instant::now() + Duration::from_secs(5);
        while *self.applied.lock().unwrap() != vec![mask] {
            assert!(Instant::now() < deadline, "mask 0x{mask:x} never applied");
            std::thread::sleep(Duration::from_millis(20));
        }
    }

    fn finish(mut self) {
        self.stop.store(true, Ordering::SeqCst);
        if let Some(p) = self.peer.take() {
            p.join().unwrap();
        }
        std::fs::remove_dir_all(&self.dir).ok();
    }
}

fn connect_with_retry(port: u16) -> TcpStream {
    let deadline = Instant::now() + Duration::from_secs(5);
    loop {
        if let Ok(s) = TcpStream::connect(("127.0.0.1", port)) {
            return s;
        }
        assert!(Instant::now() < deadline, "supervisor never listened");
        std::thread::sleep(Duration::from_millis(20));
    }
}

#[test]
fn supervisor_applies_mask_on_activate_and_rolls_back_on_eof() {
    let rig = spawn_rig("eof");
    let port = rig.heartbeat_port();
    let mut sup = rig.spawn_supervisor(port, 3000);

    let mut hb = connect_with_retry(port);
    hb.write_all(b"ACTIVATE\n").unwrap();
    rig.wait_mask(mask_owner::MASK_NO_BRUSHES);

    // Bridge dies without any goodbye: EOF must trigger the rollback.
    drop(hb);

    let deadline = Instant::now() + Duration::from_secs(10);
    loop {
        let applied = rig.applied.lock().unwrap().clone();
        if applied == vec![mask_owner::MASK_NO_BRUSHES, mask_owner::MASK_STANDBY] {
            break;
        }
        assert!(
            Instant::now() < deadline,
            "no rollback after EOF; applied {applied:?}"
        );
        std::thread::sleep(Duration::from_millis(50));
    }
    wait_restore(&rig);
    assert!(
        sup.try_wait().unwrap().is_none(),
        "persistent listener exited"
    );
    sup.kill().unwrap();
    sup.wait().unwrap();
    rig.finish();
}

#[test]
fn supervisor_rolls_back_on_silent_heartbeat() {
    let rig = spawn_rig("silent");
    let port = rig.heartbeat_port();
    let mut sup = rig.spawn_supervisor(port, 1500);

    let mut hb = connect_with_retry(port);
    hb.write_all(b"ACTIVATE\n").unwrap();
    rig.wait_mask(mask_owner::MASK_NO_BRUSHES);

    // Stay connected but say nothing: the timeout must fire.
    let deadline = Instant::now() + Duration::from_secs(10);
    loop {
        let applied = rig.applied.lock().unwrap().clone();
        if applied.len() == 2 {
            assert_eq!(applied[1], mask_owner::MASK_STANDBY, "wrong rollback mask");
            break;
        }
        assert!(
            Instant::now() < deadline,
            "no rollback after silence; applied {applied:?}"
        );
        std::thread::sleep(Duration::from_millis(50));
    }
    drop(hb);
    wait_restore(&rig);
    assert!(
        sup.try_wait().unwrap().is_none(),
        "persistent listener exited"
    );
    sup.kill().unwrap();
    sup.wait().unwrap();
    rig.finish();
}

#[test]
fn supervisor_disconnects_idle_client_without_touching_control() {
    let rig = spawn_rig("quiet");
    let port = rig.heartbeat_port();
    let mut sup = rig.spawn_supervisor(port, 1500);

    // Only this idle connection expires; the listener remains available.
    let mut hb = connect_with_retry(port);
    hb.set_read_timeout(Some(Duration::from_secs(3))).unwrap();
    assert_eq!(hb.read(&mut [0]).unwrap(), 0);
    assert!(sup.try_wait().unwrap().is_none());
    assert!(
        rig.applied.lock().unwrap().is_empty(),
        "supervisor touched masks without ACTIVATE"
    );
    assert_eq!(
        std::fs::read_to_string(rig.dir.join("restore.calls")).unwrap(),
        ""
    );
    sup.kill().unwrap();
    sup.wait().unwrap();
    rig.finish();
}

// FEATURE: persistent multi-client authenticated supervisor.
// SPEC: exclusive ACTIVATE ownership; non-owners never restore the controller.
// IMPLEMENTATION: bounded LF framing, isolated client handlers and audio guards.
// RESULT: IMPLEMENTED — sessions restore independently of listener lifetime.
// STATUS: DONE
// Every child and FIFO peer is reaped even when a contract assertion fails.
struct ServiceRig {
    rig: Option<Rig>,
    child: Child,
    port: u16,
}

impl ServiceRig {
    fn new(tag: &str, token: Option<&str>) -> Self {
        let rig = spawn_rig(tag);
        let port = rig.heartbeat_port();
        let child = rig.spawn_supervisor_with_auth(port, 3000, token);
        Self {
            rig: Some(rig),
            child,
            port,
        }
    }

    fn rig(&self) -> &Rig {
        self.rig.as_ref().unwrap()
    }

    fn connect(&self) -> TcpStream {
        let conn = connect_with_retry(self.port);
        conn.set_read_timeout(Some(Duration::from_secs(2))).unwrap();
        conn.set_write_timeout(Some(Duration::from_secs(2)))
            .unwrap();
        conn
    }

    fn wait_restored(&self, sessions: usize) {
        let deadline = Instant::now() + Duration::from_secs(6);
        let expected: Vec<_> = (0..sessions)
            .flat_map(|_| [mask_owner::MASK_NO_BRUSHES, mask_owner::MASK_STANDBY])
            .collect();
        loop {
            let actual = self.rig().applied.lock().unwrap().clone();
            let restores = std::fs::read_to_string(self.rig().dir.join("restore.calls"))
                .unwrap()
                .lines()
                .count();
            if actual == expected && restores == sessions {
                return;
            }
            assert!(
                Instant::now() < deadline,
                "expected {sessions} restored sessions, masks {actual:?}, restores {restores}"
            );
            std::thread::sleep(Duration::from_millis(10));
        }
    }

    fn assert_untouched(&self) {
        assert!(
            self.rig().applied.lock().unwrap().is_empty(),
            "non-owner touched masks"
        );
        assert_eq!(
            std::fs::read_to_string(self.rig().dir.join("restore.calls")).unwrap(),
            "",
            "non-owner restored somebody else's controller"
        );
    }
}

impl Drop for ServiceRig {
    fn drop(&mut self) {
        let _ = self.child.kill();
        let _ = self.child.wait();
        if let Some(rig) = self.rig.take() {
            rig.finish();
        }
    }
}

fn response(conn: &mut TcpStream) -> String {
    let mut line = Vec::new();
    loop {
        let mut byte = [0];
        conn.read_exact(&mut byte)
            .expect("expected bounded protocol response, not EOF/timeout");
        line.push(byte[0]);
        assert!(line.len() <= 256, "unbounded protocol response");
        if byte[0] == b'\n' {
            return String::from_utf8(line).unwrap();
        }
    }
}

// FEATURE: authenticated non-mutating readiness checks for persistent daemon reuse.
// SPEC: PING distinguishes an idle daemon from an owned session without acquiring masks.
#[test]
fn persistent_ping_reports_ownership_without_touching_hardware() {
    let service = ServiceRig::new("ping-reuse", Some("ping-secret"));
    let ping = |expected: &str| {
        let mut conn = service.connect();
        conn.write_all(b"AUTH ping-secret\nPING\n").unwrap();
        assert_eq!(response(&mut conn), "AUTHENTICATED\n");
        assert_eq!(response(&mut conn), expected);
    };
    ping("PONG IDLE\n");
    service.assert_untouched();
    let mut owner = service.connect();
    owner.write_all(b"AUTH ping-secret\nACTIVATE\n").unwrap();
    assert_eq!(response(&mut owner), "AUTHENTICATED\n");
    assert_eq!(response(&mut owner), "ACTIVATED\n");
    ping("PONG ACTIVE\n");
    owner.write_all(b"GOODBYE\n").unwrap();
    service.wait_restored(1);
    ping("PONG IDLE\n");
}

#[test]
fn persistent_contract_restores_then_accepts_a_new_owner() {
    let mut service = ServiceRig::new("persistent-reuse", None);
    let mut first = service.connect();
    first.write_all(b"ACTIVATE\n").unwrap();
    assert_eq!(response(&mut first), "ACTIVATED\n");
    service.rig().wait_mask(mask_owner::MASK_NO_BRUSHES);
    first.write_all(b"GOODBYE\n").unwrap();
    service.wait_restored(1);
    assert!(
        service.child.try_wait().unwrap().is_none(),
        "service exited after owner restore"
    );

    let mut second = service.connect();
    second.write_all(b"ACTIVATE\n").unwrap();
    assert_eq!(response(&mut second), "ACTIVATED\n");
    drop(second);
    service.wait_restored(2);
}

#[test]
fn persistent_contract_idle_client_does_not_block_an_owner() {
    let service = ServiceRig::new("parallel-accept", None);
    let _idle = service.connect();
    let mut owner = service.connect();
    owner.write_all(b"ACTIVATE\n").unwrap();
    assert_eq!(response(&mut owner), "ACTIVATED\n");
    service.rig().wait_mask(mask_owner::MASK_NO_BRUSHES);
    drop(owner);
    service.wait_restored(1);
}

#[test]
fn persistent_contract_second_activate_is_busy_without_restoring_owner() {
    let service = ServiceRig::new("exclusive-owner", None);
    let mut owner = service.connect();
    owner.write_all(b"ACTIVATE\n").unwrap();
    assert_eq!(response(&mut owner), "ACTIVATED\n");
    service.rig().wait_mask(mask_owner::MASK_NO_BRUSHES);
    let mut rival = service.connect();
    rival.write_all(b"ACTIVATE\n").unwrap();
    assert_eq!(response(&mut rival), "BUSY\n");
    drop(rival);
    assert_eq!(
        *service.rig().applied.lock().unwrap(),
        vec![mask_owner::MASK_NO_BRUSHES]
    );
    assert_eq!(
        std::fs::read_to_string(service.rig().dir.join("restore.calls")).unwrap(),
        ""
    );
    owner.write_all(b"GOODBYE\n").unwrap();
    service.wait_restored(1);
}

#[test]
fn persistent_contract_auth_precedes_every_command_and_bad_auth_is_isolated() {
    let service = ServiceRig::new("auth-isolated", Some("offline-test-secret"));
    for request in [
        "ACTIVATE\n",
        "AUTH wrong\n",
        "CANCEL aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa\n",
    ] {
        let mut conn = service.connect();
        conn.write_all(request.as_bytes()).unwrap();
        assert_eq!(response(&mut conn), "UNAUTHORIZED\n");
        service.assert_untouched();
    }
    let mut owner = service.connect();
    // Deliberate fragmentation and coalescing: parsing is by LF, not read().
    owner.write_all(b"AU").unwrap();
    owner
        .write_all(b"TH offline-test-secret\nACTIVATE\n")
        .unwrap();
    assert_eq!(response(&mut owner), "AUTHENTICATED\n");
    assert_eq!(response(&mut owner), "ACTIVATED\n");
    service.rig().wait_mask(mask_owner::MASK_NO_BRUSHES);
    drop(owner);
    service.wait_restored(1);
}

#[test]
fn persistent_contract_activation_requires_a_complete_exact_line() {
    let service = ServiceRig::new("exact-command", None);
    let mut bad = service.connect();
    bad.write_all(b"ACTIVATEevil\n").unwrap();
    assert_eq!(response(&mut bad), "REFUSED\n");
    service.assert_untouched();
}

#[test]
fn persistent_contract_audio_rejects_invalid_and_oversized_wav_without_mask() {
    let service = ServiceRig::new("audio-validation", None);
    for request in [
        "AUDIO 0.35 aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa 3\nbad",
        "AUDIO 0.35 aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa 1000001\n",
        "AUDIO nan aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa 44\n",
        "AUDIO 0.35 invalid-token 44\n",
    ] {
        let mut conn = service.connect();
        conn.write_all(request.as_bytes()).unwrap();
        assert_eq!(response(&mut conn), "INVALID_AUDIO\n");
        service.assert_untouched();
    }
}

#[test]
fn persistent_contract_same_elf_audio_worker_validates_before_loading_alsa() {
    let mut worker = Command::new(BIN)
        .args(["--audio-worker", "0.35", "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"])
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .unwrap();
    worker.stdin.take().unwrap().write_all(b"bad").unwrap();
    let output = worker.wait_with_output().unwrap();
    assert!(!output.status.success());
    assert_eq!(
        output.stdout, b"INVALID_AUDIO\n",
        "same executable must dispatch the isolated C guard, not the listener CLI"
    );
}

fn wait_restore(rig: &Rig) {
    let deadline = Instant::now() + Duration::from_secs(6);
    while std::fs::read_to_string(rig.dir.join("restore.calls")).unwrap() != "restore\n" {
        assert!(Instant::now() < deadline, "controller was not restored");
        std::thread::sleep(Duration::from_millis(10));
    }
}

// FEATURE: same-ELF playback guard with persistent, independent audio clients.
// SPEC: fake ALSA success, cancellation acknowledgement after mixer restoration,
// oversized rejection, concurrent control ownership; audio never changes masks.
#[cfg(feature = "audio-testing")]
mod audio_contract {
    use super::*;
    const TOKEN: &str = "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa";

    fn audio_service(tag: &str) -> ServiceRig {
        let rig = spawn_rig(tag);
        let source = rig.dir.join("fake.c");
        let library = rig.dir.join("fake-alsa.so");
        std::fs::write(&source, include_str!("fake-alsa.c")).unwrap();
        assert!(Command::new("cc")
            .args(["-shared", "-fPIC"])
            .arg(&source)
            .arg("-o")
            .arg(&library)
            .status()
            .unwrap()
            .success());
        let port = rig.heartbeat_port();
        let (script, calls) = rig.restore_script();
        let child = Command::new(BIN)
            .args([
                "listen",
                "--heartbeat-port",
                &port.to_string(),
                "--timeout-ms",
                "3000",
            ])
            .env_remove("ROBOT_SUPERVISOR_TOKEN")
            .env("MASK_OWNER_SEND", rig.dir.join("RcMc"))
            .env("MASK_OWNER_RECV", rig.dir.join("McRc"))
            .env("ROBOT_SUPERVISOR_PLAYER", "127.0.0.1:1")
            .env(
                "ROBOT_SUPERVISOR_RESTORE",
                format!("{} {}", script.display(), calls.display()),
            )
            .env("ROBOT_AUDIO_ALSA_LIBRARY", library)
            .env("ROBOT_AUDIO_LOCK", rig.dir.join("audio.lock"))
            .env("ROBOT_AUDIO_TEST_LOG", rig.dir.join("mixer.log"))
            .stdout(Stdio::null())
            .stderr(Stdio::null())
            .spawn()
            .unwrap();
        ServiceRig {
            rig: Some(rig),
            child,
            port,
        }
    }

    fn audio(service: &ServiceRig, frames: usize) -> TcpStream {
        let bytes = (frames * 2) as u32;
        let mut wav = Vec::new();
        wav.extend_from_slice(b"RIFF");
        wav.extend_from_slice(&(36 + bytes).to_le_bytes());
        wav.extend_from_slice(b"WAVEfmt ");
        wav.extend_from_slice(&16u32.to_le_bytes());
        wav.extend_from_slice(&[1, 0, 1, 0]);
        wav.extend_from_slice(&16000u32.to_le_bytes());
        wav.extend_from_slice(&32000u32.to_le_bytes());
        wav.extend_from_slice(&[2, 0, 16, 0]);
        wav.extend_from_slice(b"data");
        wav.extend_from_slice(&bytes.to_le_bytes());
        wav.resize(44 + bytes as usize, 0);
        let mut conn = service.connect();
        write!(conn, "AUDIO 0.600000 {TOKEN} {}\n", wav.len()).unwrap();
        // Header and body can be coalesced or fragmented independently.
        conn.write_all(&wav).unwrap();
        conn
    }

    fn until(conn: &mut TcpStream, terminal: &str) {
        for _ in 0..100 {
            let line = response(conn);
            if line == format!("{terminal}\n") {
                return;
            }
            assert!(
                line.starts_with("WORKER ") || line == "ALIVE\n" || line == "STARTED\n",
                "unexpected {line:?}"
            );
        }
        panic!("missing terminal {terminal}");
    }

    fn restored(service: &ServiceRig) {
        let log = std::fs::read_to_string(service.rig().dir.join("mixer.log")).unwrap();
        for (name, value) in [
            ("speaker volume control", 0),
            ("Audio speaker out", 0),
            ("MIC1 boost AMP gain control", 4),
        ] {
            let prefix = format!("SET {name} ");
            assert_eq!(
                log.lines()
                    .filter(|l| l.starts_with(&prefix))
                    .last()
                    .unwrap(),
                format!("{prefix}{value}")
            );
        }
    }

    #[test]
    fn audio_success_and_oversized_do_not_touch_control() {
        let service = audio_service("audio-success");
        let mut bad = service.connect();
        bad.write_all(format!("AUDIO 0.600000 {TOKEN} 1000001\n").as_bytes())
            .unwrap();
        assert_eq!(response(&mut bad), "INVALID_AUDIO\n");
        assert!(!service.rig().dir.join("mixer.log").exists());
        let mut conn = audio(&service, 1600);
        until(&mut conn, "STARTED");
        until(&mut conn, "COMPLETED");
        restored(&service);
        service.assert_untouched();
    }

    #[test]
    fn audio_cancel_and_busy_preserve_concurrent_control_owner() {
        let service = audio_service("audio-cancel-control");
        let mut owner = service.connect();
        owner.write_all(b"ACTIVATE\n").unwrap();
        assert_eq!(response(&mut owner), "ACTIVATED\n");
        service.rig().wait_mask(mask_owner::MASK_NO_BRUSHES);
        let mut conn = audio(&service, 16000 * 10);
        until(&mut conn, "STARTED");
        let mut busy = audio(&service, 1600);
        assert_eq!(response(&mut busy), "BUSY\n");
        let mut wrong = service.connect();
        wrong
            .write_all(b"CANCEL bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb\n")
            .unwrap();
        assert_ne!(response(&mut wrong), "CANCELLED\n");
        let mut cancel = service.connect();
        cancel
            .write_all(format!("CANCEL {TOKEN}\n").as_bytes())
            .unwrap();
        assert_eq!(response(&mut cancel), "CANCELLED\n");
        restored(&service);
        until(&mut conn, "CANCELLED");
        assert_eq!(
            *service.rig().applied.lock().unwrap(),
            vec![mask_owner::MASK_NO_BRUSHES]
        );
        assert_eq!(
            std::fs::read_to_string(service.rig().dir.join("restore.calls")).unwrap(),
            ""
        );
        owner.write_all(b"GOODBYE\n").unwrap();
        service.wait_restored(1);
    }
}

// FEATURE: uncertain handoff failure must fail closed, even in persistent mode.
// SPEC: attempt local restoration, then terminate nonzero instead of reusing
// a service whose control state could not be established.
#[test]
fn persistent_contract_fifo_open_failure_restores_and_fails_closed() {
    let mut service = ServiceRig::new("failed-handoff", None);
    let mut conn = service.connect();
    std::fs::remove_file(service.rig().dir.join("RcMc")).unwrap();
    conn.write_all(b"ACTIVATE\n").unwrap();
    wait_restore(service.rig());
    let deadline = Instant::now() + Duration::from_secs(2);
    loop {
        if let Some(status) = service.child.try_wait().unwrap() {
            assert!(!status.success());
            break;
        }
        assert!(
            Instant::now() < deadline,
            "uncertain handoff left listener accepting new owners"
        );
        std::thread::sleep(Duration::from_millis(10));
    }
}
