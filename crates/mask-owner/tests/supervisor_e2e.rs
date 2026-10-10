//! End-to-end tests for robot-supervisor: the on-robot process that
//! holds the session mask while the workstation bridge stays alive.
//!
//! Lifecycle under test:
//!   supervisor listen --heartbeat-port P --timeout T
//!     - waits for ACTIVATE on the heartbeat connection
//!     - applies the session mask, then rolls back when the connection
//!       dies (EOF) or goes silent past the timeout
//!   rollback order: zero velocity, standby mask, restore command
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
        let (script, calls) = self.restore_script();
        Command::new(BIN)
            .arg("listen")
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
    let _ = sup.wait();
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
    let _ = sup.wait();
    rig.finish();
}

#[test]
fn supervisor_exits_quietly_without_activate() {
    let rig = spawn_rig("quiet");
    let port = rig.heartbeat_port();
    let mut sup = rig.spawn_supervisor(port, 1500);

    // Connect and say nothing until the activation window expires: the
    // supervisor must exit WITHOUT touching the FIFOs.
    let _hb = connect_with_retry(port);
    let start = Instant::now();
    let status = sup.wait().unwrap();
    assert!(start.elapsed() < Duration::from_secs(10));
    assert!(status.success(), "expected clean exit, got {status}");
    assert!(
        rig.applied.lock().unwrap().is_empty(),
        "supervisor touched masks without ACTIVATE"
    );
    rig.finish();
}
