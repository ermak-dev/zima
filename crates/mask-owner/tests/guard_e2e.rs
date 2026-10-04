//! End-to-end guard test with the real binary: the owner `hold`s the
//! mask, is killed with SIGKILL (no cleanup code can run), and the guard
//! child must restore standby on the channel. This is the safety property
//! measured live on 2026-10-03, now kept as a regression test.

use std::io::{Read, Write};
use std::os::unix::fs::OpenOptionsExt;
use std::os::fd::{AsRawFd, FromRawFd};
use std::os::unix::process::CommandExt;
use std::path::{Path, PathBuf};
use std::process::{Child, Command, Stdio};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

const BIN: &str = env!("CARGO_BIN_EXE_mask-owner");

fn mkfifo(path: &Path) {
    extern "C" {
        fn mkfifo(path: *const std::os::raw::c_char, mode: u32) -> i32;
    }
    let c = std::ffi::CString::new(path.to_str().unwrap()).unwrap();
    assert_eq!(unsafe { mkfifo(c.as_ptr(), 0o666) }, 0, "mkfifo {path:?}");
}

fn open_rw_nb(path: &Path) -> std::fs::File {
    std::fs::OpenOptions::new()
        .read(true)
        .write(true)
        .custom_flags(0o4000)
        .open(path)
        .unwrap()
}

struct FakeMc {
    dir: PathBuf,
    applied: Arc<Mutex<Vec<u32>>>,
    stop: Arc<AtomicBool>,
    peer: Option<std::thread::JoinHandle<()>>,
}

fn spawn_fake_mc(tag: &str) -> FakeMc {
    use mask_owner::{parse_header, EVENT_ACK, EVENT_CHANGE, MODULE_MC, MODULE_RC, PKT_LEN};
    let dir = std::env::temp_dir().join(format!("mask-owner-e2e-{tag}"));
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).unwrap();
    let send = dir.join("RcMc"); // owner writes here
    let recv = dir.join("McRc"); // we answer here
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
            if stop_t.load(Ordering::Relaxed) {
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
    FakeMc { dir, applied, stop, peer: Some(peer) }
}

impl Drop for FakeMc {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(p) = self.peer.take() {
            p.join().ok();
        }
        std::fs::remove_dir_all(&self.dir).ok();
    }
}

#[test]
fn guard_restores_standby_after_sigkill() {
    let mc = spawn_fake_mc("kill");
    let send = mc.dir.join("RcMc");
    let recv = mc.dir.join("McRc");

    let mut owner: Child = Command::new(BIN)
        .args(["hold", "--mask", "10f8ff", "--timeout", "3"])
        .env("MASK_OWNER_SEND", &send)
        .env("MASK_OWNER_RECV", &recv)
        .stderr(Stdio::null())
        .spawn()
        .unwrap();

    // Wait until the mask is applied.
    let deadline = Instant::now() + Duration::from_secs(5);
    while *mc.applied.lock().unwrap() != vec![mask_owner::MASK_NO_BRUSHES] {
        assert!(Instant::now() < deadline, "owner never applied the mask");
        std::thread::sleep(Duration::from_millis(50));
    }

    // Kill hard: no cleanup code can run; only the guard can restore.
    let mut owner = owner;
    owner.kill().unwrap();
    let _ = owner.wait();

    // The guard polls liveness once a second and then restores standby.
    let deadline = Instant::now() + Duration::from_secs(10);
    loop {
        let applied = mc.applied.lock().unwrap().clone();
        if applied == vec![mask_owner::MASK_NO_BRUSHES, mask_owner::MASK_STANDBY] {
            break;
        }
        assert!(
            Instant::now() < deadline,
            "guard did not restore standby; applied so far: {applied:?}"
        );
        std::thread::sleep(Duration::from_millis(100));
    }
}

#[test]
fn hold_restores_standby_on_sigterm() {
    let mc = spawn_fake_mc("term");
    let send = mc.dir.join("RcMc");
    let recv = mc.dir.join("McRc");

    let mut owner = Command::new(BIN)
        .args(["hold", "--mask", "10f8ff", "--timeout", "3"])
        .env("MASK_OWNER_SEND", &send)
        .env("MASK_OWNER_RECV", &recv)
        .stderr(Stdio::null())
        .spawn()
        .unwrap();

    let deadline = Instant::now() + Duration::from_secs(5);
    while *mc.applied.lock().unwrap() != vec![mask_owner::MASK_NO_BRUSHES] {
        assert!(Instant::now() < deadline, "owner never applied the mask");
        std::thread::sleep(Duration::from_millis(50));
    }

    // Graceful stop: the owner restores standby itself and stands the
    // guard down via the marker file.
    nix_kill(owner.id() as i32, 15);
    let status = owner.wait().unwrap();
    assert!(status.success(), "owner exited with {status}");

    let applied = mc.applied.lock().unwrap().clone();
    assert_eq!(applied, vec![mask_owner::MASK_NO_BRUSHES, mask_owner::MASK_STANDBY]);
}

/// Live regression (2026-10-03, second): the guard inherited the owner's
/// stderr pipe; when the owner (running over ssh) died, the pipe closed
/// and the guard's first log write SIGPIPEd it before it could restore
/// the mask. The guard must survive writing to a closed pipe.
#[test]
fn guard_survives_closed_stderr_pipe() {
    let mc = spawn_fake_mc("pipe");
    let send = mc.dir.join("RcMc");
    let recv = mc.dir.join("McRc");

    // A pipe whose read end we drop right before killing the owner: the
    // guard's stderr is this broken pipe, like a dead ssh channel.
    let (read_end, write_end) = std::os::unix::net::UnixStream::pair().unwrap();
    drop(read_end);
    let write_fd = write_end.as_raw_fd();
    let stderr_fd = unsafe { libc_dup(write_fd) };

    let owner = Command::new(BIN)
        .args(["hold", "--mask", "10f8ff", "--timeout", "3"])
        .env("MASK_OWNER_SEND", &send)
        .env("MASK_OWNER_RECV", &recv)
        .stdout(unsafe { Stdio::from_raw_fd(write_fd) })
        .stderr(unsafe { Stdio::from_raw_fd(stderr_fd) })
        .process_group(0)
        .spawn()
        .unwrap();
    // write_end's fd was consumed by Stdio; forget the socket wrapper so
    // its Drop does not close the child's fd.
    std::mem::forget(write_end);

    let deadline = Instant::now() + Duration::from_secs(5);
    while *mc.applied.lock().unwrap() != vec![mask_owner::MASK_NO_BRUSHES] {
        assert!(Instant::now() < deadline, "owner never applied the mask");
        std::thread::sleep(Duration::from_millis(50));
    }

    let mut owner = owner;
    owner.kill().unwrap();
    let _ = owner.wait();

    let deadline = Instant::now() + Duration::from_secs(10);
    loop {
        let applied = mc.applied.lock().unwrap().clone();
        if applied == vec![mask_owner::MASK_NO_BRUSHES, mask_owner::MASK_STANDBY] {
            break;
        }
        assert!(
            Instant::now() < deadline,
            "guard died on the closed pipe; applied so far: {applied:?}"
        );
        std::thread::sleep(Duration::from_millis(100));
    }
}

extern "C" {
    #[link_name = "dup"]
    fn libc_dup(fd: i32) -> i32;
}

extern "C" {
    #[link_name = "kill"]
    fn libc_kill(pid: i32, sig: i32) -> i32;
}

fn nix_kill(pid: i32, sig: i32) {
    let rc = unsafe { libc_kill(pid, sig) };
    assert_eq!(rc, 0, "kill({pid}, {sig}) failed");
}

/// Live regression (2026-10-03): the owner ran over ssh, was SIGKILLed,
/// and sshd's session teardown SIGHUPed the guard before it could restore
/// the mask — LDS kept spinning. The guard must survive a HUP to the
/// owner's process group.
#[test]
fn guard_survives_session_teardown_sighup() {
    let mc = spawn_fake_mc("hup");
    let send = mc.dir.join("RcMc");
    let recv = mc.dir.join("McRc");

    let mut owner = Command::new(BIN)
        .args(["hold", "--mask", "10f8ff", "--timeout", "3"])
        .env("MASK_OWNER_SEND", &send)
        .env("MASK_OWNER_RECV", &recv)
        .stderr(Stdio::null())
        .process_group(0) // own session-like group, as sshd gives remote processes
        .spawn()
        .unwrap();

    let deadline = Instant::now() + Duration::from_secs(5);
    while *mc.applied.lock().unwrap() != vec![mask_owner::MASK_NO_BRUSHES] {
        assert!(Instant::now() < deadline, "owner never applied the mask");
        std::thread::sleep(Duration::from_millis(50));
    }

    // Session teardown as sshd does it: HUP the process group, then the
    // owner dies hard. Negative pid addresses the whole group.
    nix_kill(-(owner.id() as i32), 1); // SIGHUP to the group
    owner.kill().unwrap();
    let _ = owner.wait();

    let deadline = Instant::now() + Duration::from_secs(10);
    loop {
        let applied = mc.applied.lock().unwrap().clone();
        if applied == vec![mask_owner::MASK_NO_BRUSHES, mask_owner::MASK_STANDBY] {
            break;
        }
        assert!(
            Instant::now() < deadline,
            "guard did not survive session teardown; applied so far: {applied:?}"
        );
        std::thread::sleep(Duration::from_millis(100));
    }
}
