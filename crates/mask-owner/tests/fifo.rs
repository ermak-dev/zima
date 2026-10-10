//! Integration tests: the owner session against a fake Mc peer over real
//! FIFOs. Tests run sequentially (cargo runs files one at a time; inside
//! a file the harness may parallel threads, so a global test mutex
//! serializes path setup).
//!
//! The end-to-end guard behavior (owner killed hard -> guard restores
//! standby) lives in tests/guard_e2e.rs against the real binary.

use std::fs::{File, OpenOptions};
use std::io::{Read, Write};
use std::os::unix::fs::OpenOptionsExt;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicU32, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use mask_owner::ipc::{Channel, Owner};
use mask_owner::*;

static TEST_MUTEX: Mutex<()> = Mutex::new(());

struct Rig {
    dir: PathBuf,
    applied: Arc<Mutex<Vec<u32>>>,
    stop: Arc<AtomicU32>,
    peer: Option<std::thread::JoinHandle<()>>,
}

fn open_rw_nb(path: &Path) -> File {
    OpenOptions::new()
        .read(true)
        .write(true)
        .custom_flags(0o4000)
        .open(path)
        .unwrap()
}

fn mkfifo(path: &Path) {
    extern "C" {
        fn mkfifo(path: *const std::os::raw::c_char, mode: u32) -> i32;
    }
    let c = std::ffi::CString::new(path.to_str().unwrap()).unwrap();
    let rc = unsafe { mkfifo(c.as_ptr(), 0o666) };
    assert_eq!(rc, 0, "mkfifo failed for {path:?}");
}

/// Fake Mc peer: answers each 0x45 with `foreign_burst` telemetry packets
/// and then the 0xa9 ack; records applied bitmaps.
fn spawn_rig(tag: &str, foreign_burst: usize) -> Rig {
    let dir = std::env::temp_dir().join(format!("mask-owner-{tag}-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).unwrap();
    let send = dir.join("RcMc");
    let recv = dir.join("McRc");
    mkfifo(&send);
    mkfifo(&recv);

    let applied: Arc<Mutex<Vec<u32>>> = Arc::new(Mutex::new(Vec::new()));
    let stop = Arc::new(AtomicU32::new(0));
    let applied_t = applied.clone();
    let stop_t = stop.clone();

    let peer = std::thread::spawn(move || {
        let mut from_rc = open_rw_nb(&send);
        let mut to_rc = open_rw_nb(&recv);
        let mut buf = [0u8; PKT_LEN];
        loop {
            if stop_t.load(Ordering::SeqCst) != 0 {
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
                    for _ in 0..foreign_burst {
                        let mut foreign = [0u8; PKT_LEN];
                        foreign[0..2].copy_from_slice(&0x01f9u16.to_le_bytes());
                        foreign[2] = MODULE_MC;
                        foreign[3] = MODULE_RC;
                        to_rc.write_all(&foreign).unwrap();
                    }
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
    fn finish(mut self) {
        self.stop.store(1, Ordering::SeqCst);
        if let Some(peer) = self.peer.take() {
            peer.join().unwrap();
        }
        std::fs::remove_dir_all(&self.dir).ok();
    }
}

#[test]
fn owner_applies_mask_and_restores_standby() {
    let _guard = TEST_MUTEX.lock().unwrap();
    let rig = spawn_rig("basic", 0);
    let mut owner = Owner::start_with(
        MASK_NO_BRUSHES,
        Duration::from_secs(2),
        rig.dir.join("RcMc").to_str().unwrap(),
        rig.dir.join("McRc").to_str().unwrap(),
        false,
    )
    .unwrap();
    assert_eq!(*rig.applied.lock().unwrap(), vec![MASK_NO_BRUSHES]);
    owner.restore(Duration::from_secs(2)).unwrap();
    assert_eq!(
        *rig.applied.lock().unwrap(),
        vec![MASK_NO_BRUSHES, MASK_STANDBY]
    );
    drop(owner);
    rig.finish();
}

#[test]
fn foreign_packets_before_ack_are_skipped() {
    let _guard = TEST_MUTEX.lock().unwrap();
    let rig = spawn_rig("foreign", 2);
    let mut owner = Owner::start_with(
        MASK_NO_BRUSHES,
        Duration::from_secs(2),
        rig.dir.join("RcMc").to_str().unwrap(),
        rig.dir.join("McRc").to_str().unwrap(),
        false,
    )
    .unwrap();
    owner.restore(Duration::from_secs(2)).unwrap();
    assert_eq!(
        *rig.applied.lock().unwrap(),
        vec![MASK_NO_BRUSHES, MASK_STANDBY]
    );
    drop(owner);
    rig.finish();
}

#[test]
fn timeout_when_peer_silent() {
    let _guard = TEST_MUTEX.lock().unwrap();
    let dir = std::env::temp_dir().join(format!("mask-owner-silent-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).unwrap();
    let send = dir.join("RcMc");
    let recv = dir.join("McRc");
    mkfifo(&send);
    mkfifo(&recv);
    let _reader = open_rw_nb(&send); // absorb writes so no EPIPE
    let err = Owner::start_with(
        MASK_NO_BRUSHES,
        Duration::from_millis(300),
        send.to_str().unwrap(),
        recv.to_str().unwrap(),
        false,
    )
    .err();
    assert_eq!(err, Some(OwnerError::Timeout));
    std::fs::remove_dir_all(&dir).ok();
}

#[test]
fn channel_drain_drops_stale_packets() {
    let _guard = TEST_MUTEX.lock().unwrap();
    let dir = std::env::temp_dir().join(format!("mask-owner-drain-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).unwrap();
    let send = dir.join("RcMc");
    let recv = dir.join("McRc");
    mkfifo(&send);
    mkfifo(&recv);

    // A writer on McRc queues two stale telemetry packets; it must stay
    // open: the kernel discards buffered FIFO data once the last writer
    // fd closes.
    let mut mc_writer = open_rw_nb(&recv);
    for _ in 0..2 {
        let mut stale = [0u8; PKT_LEN];
        stale[0..2].copy_from_slice(&0x01f9u16.to_le_bytes());
        stale[2] = MODULE_MC;
        stale[3] = MODULE_RC;
        mc_writer.write_all(&stale).unwrap();
    }

    let mut channel = Channel::open_with(send.to_str().unwrap(), recv.to_str().unwrap()).unwrap();
    assert_eq!(channel.drain().unwrap(), 2);
    assert_eq!(channel.drain().unwrap(), 0);
    drop(mc_writer);
    drop(channel);
    std::fs::remove_dir_all(&dir).ok();
}
