//! Tests for the full rollback path: zero velocity first, then the
//! standby mask, then the controller-restore command. The order matters:
//! zero velocity reaches the MCU over TCP immediately, the mask then
//! disables the wheel permission (a second, independent stop path), and
//! only then is the stock controller brought back.

use std::io::{Read, Write};
use std::net::TcpListener;
use std::os::unix::fs::OpenOptionsExt;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use mask_owner::ipc::{full_rollback, Channel};
use mask_owner::*;

struct Rig {
    dir: PathBuf,
    applied: Arc<Mutex<Vec<u32>>>,
    player: Option<std::net::TcpStream>,
    stop: Arc<AtomicBool>,
    peer: Option<std::thread::JoinHandle<()>>,
    restore_calls: Arc<Mutex<Vec<String>>>,
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

/// Fake Mc peer on the FIFOs plus a fake Player TCP server that records
/// velocity commands.
fn spawn_rig(tag: &str) -> Rig {
    use player_bridge::{Client, POSITION2D_CODE};
    let _ = POSITION2D_CODE;
    let dir = std::env::temp_dir().join(format!("supervisor-{tag}-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).unwrap();
    let send = dir.join("RcMc");
    let recv = dir.join("McRc");
    mkfifo(&send);
    mkfifo(&recv);

    let applied: Arc<Mutex<Vec<u32>>> = Arc::new(Mutex::new(Vec::new()));
    let stop = Arc::new(AtomicBool::new(false));
    let restore_calls: Arc<Mutex<Vec<String>>> = Arc::new(Mutex::new(Vec::new()));

    // Fake Player TCP server: accept one connection, record velocity
    // commands (subtype CMD_VEL with zero payload values).
    let listener = TcpListener::bind("127.0.0.1:0").unwrap();
    let player_port = listener.local_addr().unwrap().port();
    std::fs::write(dir.join("player.port"), player_port.to_string()).unwrap();
    let vel_commands: Arc<Mutex<Vec<(f64, f64)>>> = Arc::new(Mutex::new(Vec::new()));
    let vel_t = vel_commands.clone();
    let stop_t = stop.clone();
    let tcp_peer = std::thread::spawn(move || {
        listener.set_nonblocking(true).ok();
        let mut conn = loop {
            if stop_t.load(Ordering::SeqCst) {
                return;
            }
            if let Ok(c) = listener.accept() {
                break c.0;
            }
            std::thread::sleep(Duration::from_millis(5));
        };
        conn.set_nonblocking(true).ok();
        conn.write_all(b"Player v.3.1.1-test\0\0\0\0\0\0\0\0\0\0\0\0\0").ok();
        let mut buf = [0u8; 4096];
        loop {
            if stop_t.load(Ordering::SeqCst) {
                return;
            }
            match conn.read(&mut buf) {
                Ok(0) => return,
                Ok(n) => {
                    // XDR messages: 40-byte header + payload; CMD_VEL is
                    // msg_type=2, subtype=1, interf=4 (position2d).
                    let mut off = 0;
                    while off + 40 <= n {
                        let interf = u32::from_be_bytes(buf[off + 8..off + 12].try_into().unwrap());
                        let mtype = u32::from_be_bytes(buf[off + 16..off + 20].try_into().unwrap());
                        let sub = u32::from_be_bytes(buf[off + 20..off + 24].try_into().unwrap());
                        let size = u32::from_be_bytes(buf[off + 36..off + 40].try_into().unwrap()) as usize;
                        if mtype == 2 && sub == 1 && interf == 4 && off + 40 + 28 <= n {
                            let vx = f64::from_be_bytes(buf[off + 40..off + 48].try_into().unwrap());
                            let vyaw = f64::from_be_bytes(buf[off + 56..off + 64].try_into().unwrap());
                            vel_t.lock().unwrap().push((vx, vyaw));
                        }
                        off += 40 + size;
                    }
                }
                Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => {
                    std::thread::sleep(Duration::from_millis(5));
                }
                Err(_) => return,
            }
        }
    });

    // Fake Mc peer on the FIFOs (reuses the shape of fifo.rs peer).
    let (applied_t, stop_r, send_t, recv_t) = (applied.clone(), stop.clone(), send.clone(), recv.clone());
    let peer = std::thread::spawn(move || {
        let mut from_rc = open_rw_nb(&send_t);
        let mut to_rc = open_rw_nb(&recv_t);
        let mut buf = [0u8; PKT_LEN];
        loop {
            if stop_r.load(Ordering::SeqCst) {
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

    Rig { dir, applied, player: None, stop, peer: Some(peer), restore_calls }
}

impl Rig {
    fn player_port(&self) -> u16 {
        std::fs::read_to_string(self.dir.join("player.port"))
            .unwrap()
            .trim()
            .parse()
            .unwrap()
    }

    fn finish(mut self) {
        self.stop.store(true, Ordering::SeqCst);
        if let Some(p) = self.peer.take() {
            p.join().unwrap();
        }
        std::fs::remove_dir_all(&self.dir).ok();
    }
}

#[test]
fn full_rollback_sends_zero_velocity_then_standby_then_restore() {
    let rig = spawn_rig("order");

    // Fake restore command that records its invocation.
    let restore_sh = rig.dir.join("restore.sh");
    std::fs::write(&restore_sh, "#!/bin/sh\necho called >> \"$1\"\n").unwrap();
    use std::os::unix::fs::PermissionsExt;
    std::fs::set_permissions(&restore_sh, std::fs::Permissions::from_mode(0o755)).unwrap();
    let calls = rig.dir.join("restore.calls");
    std::fs::write(&calls, "").unwrap();

    let rollback = mask_owner::ipc::Rollback {
        player_host: "127.0.0.1".into(),
        player_port: rig.player_port(),
        send_fifo: rig.dir.join("RcMc").to_str().unwrap().into(),
        recv_fifo: rig.dir.join("McRc").to_str().unwrap().into(),
        restore_command: format!("{} {}", restore_sh.display(), calls.display()),
    };

    // Apply a non-standby mask first so the rollback has something to undo.
    {
        let mut ch = Channel::open_with(
            rig.dir.join("RcMc").to_str().unwrap(),
            rig.dir.join("McRc").to_str().unwrap(),
        )
        .unwrap();
        ch.drain().ok();
        ch.exchange(1, MASK_NO_BRUSHES, Duration::from_secs(2)).unwrap();
    }
    assert_eq!(*rig.applied.lock().unwrap(), vec![MASK_NO_BRUSHES]);

    full_rollback(&rollback, Duration::from_secs(3)).unwrap();

    // Standby mask reached the FIFO peer.
    assert_eq!(*rig.applied.lock().unwrap(), vec![MASK_NO_BRUSHES, MASK_STANDBY]);
    // Restore command ran exactly once.
    let called = std::fs::read_to_string(&calls).unwrap();
    assert!(called.trim().contains("called"), "restore not invoked: {called:?}");

    rig.finish();
}
