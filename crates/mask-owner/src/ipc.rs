//! FIFO transport and the owner/guard lifecycle.
//!
//! Everything the 2026-10-03 live session taught, as code:
//! - both FIFOs open `O_RDWR|O_NONBLOCK`: after a watchdog restart Player
//!   has no writer for McRc, and a blocking `O_RDONLY` open hangs forever;
//! - the guard child is forked BEFORE the mask is applied, because
//!   `kill -9` skips any cleanup code the owner itself might run, and
//!   Player does not roll the device mask back on its own (measured:
//!   LDS still spinning 12+ s after the owner died);
//! - liveness is checked through /proc with a zombie check, since
//!   `kill(0)` stays true for unreaped children;
//! - the foreign-packet skip is bounded, then the deadline wins.
//!
//! Paths are explicit throughout; production defaults come from the
//! `MASK_OWNER_SEND`/`MASK_OWNER_RECV` environment variables so the guard
//! child (which inherits them) opens the same channel as its parent.

use std::fs::{File, OpenOptions};
use std::io::{Read, Write};
use std::os::unix::fs::OpenOptionsExt;
use std::process;
use std::time::{Duration, Instant};

use crate::*;

/// Bound on foreign packets skipped while waiting for one ack.
pub const MAX_FOREIGN: usize = 4;

#[cfg(target_os = "linux")]
const O_NONBLOCK: i32 = 0o4000; // libc::O_NONBLOCK without the libc crate

pub fn default_send_fifo() -> String {
    std::env::var("MASK_OWNER_SEND").unwrap_or_else(|_| "/tmp/RcMc".to_string())
}

pub fn default_recv_fifo() -> String {
    std::env::var("MASK_OWNER_RECV").unwrap_or_else(|_| "/tmp/McRc".to_string())
}

/// Non-blocking channel to the McuProxy side of the IPC.
pub struct Channel {
    send: File,
    recv: File,
}

impl Channel {
    /// Open both FIFOs `O_RDWR|O_NONBLOCK`. Never blocks; keeps a reader
    /// on the receive FIFO from the first moment so Player can open its
    /// write end whenever it needs to answer.
    pub fn open_with(send_path: &str, recv_path: &str) -> std::io::Result<Self> {
        let mut opts = OpenOptions::new();
        opts.read(true).write(true).custom_flags(O_NONBLOCK);
        let recv = opts.open(recv_path)?;
        let send = opts.open(send_path)?;
        Ok(Channel { send, recv })
    }

    /// Open the production channel (env-configured paths).
    pub fn open() -> std::io::Result<Self> {
        Self::open_with(&default_send_fifo(), &default_recv_fifo())
    }

    /// Drop stale packets buffered in the receive FIFO before an exchange,
    /// so a queued telemetry burst cannot eat our ack window.
    pub fn drain(&mut self) -> std::io::Result<usize> {
        let mut dropped = 0usize;
        let mut buf = [0u8; PKT_LEN];
        loop {
            match self.recv.read(&mut buf) {
                Ok(PKT_LEN) => dropped += 1,
                Ok(0) => return Ok(dropped),
                Ok(_) => return Err(std::io::Error::other("short packet in queue")),
                Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => return Ok(dropped),
                Err(e) => return Err(e),
            }
        }
    }

    /// Send one request and wait for its ack until the deadline.
    pub fn exchange(&mut self, seq: u32, bitmap: u32, timeout: Duration) -> Result<(), OwnerError> {
        let pkt = build_request(seq, bitmap)?;
        self.send
            .write_all(&pkt)
            .map_err(|_| OwnerError::WriteFailed)?;
        self.send.flush().map_err(|_| OwnerError::WriteFailed)?;

        let deadline = Instant::now() + timeout;
        let mut foreign = 0usize;
        let mut buf = [0u8; PKT_LEN];
        loop {
            let left = deadline.saturating_duration_since(Instant::now());
            if left.is_zero() {
                return Err(OwnerError::Timeout);
            }
            match self.recv.read(&mut buf) {
                Ok(0) => return Err(OwnerError::Eof),
                Ok(PKT_LEN) => match is_ack(&buf) {
                    AckShape::Ack => return Ok(()),
                    AckShape::AckWrongDirection => return Err(OwnerError::CorruptChannel),
                    AckShape::NotAnAck => {
                        foreign += 1;
                        if foreign > MAX_FOREIGN {
                            return Err(OwnerError::Timeout);
                        }
                    }
                    AckShape::TooShort => return Err(OwnerError::ShortRead),
                },
                Ok(_) => return Err(OwnerError::ShortRead),
                Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => {
                    std::thread::sleep(left.min(Duration::from_millis(20)));
                }
                Err(_) => return Err(OwnerError::ReadFailed),
            }
        }
    }
}

/// A process is alive only if it exists and is not a zombie.
fn process_alive(pid: u32) -> bool {
    let stat = match std::fs::read_to_string(format!("/proc/{pid}/stat")) {
        Ok(s) => s,
        Err(_) => return false,
    };
    // field 3 is the state after "pid (comm)"; comm may contain spaces, so
    // split after the last ')'.
    let after_paren = match stat.rsplit_once(')') {
        Some((_, rest)) => rest,
        None => return false,
    };
    let state = after_paren.split_whitespace().next().unwrap_or("Z");
    state != "Z" && state != "X"
}

/// Guard child: watch the owner; if it dies without standing down (marker
/// file absent), take over the channel and restore standby with retries.
/// FIFO paths come from the inherited environment.
pub fn guard_main(owner_pid: u32, standdown_marker: &str) -> ! {
    eprintln!("[guard] watching owner pid={owner_pid}");
    while process_alive(owner_pid) {
        std::thread::sleep(Duration::from_secs(1));
    }
    eprintln!("[guard] owner gone");
    if std::path::Path::new(standdown_marker).exists() {
        eprintln!("[guard] owner stood down itself; exiting");
        process::exit(0);
    }
    eprintln!("[guard] owner died hard; restoring standby");
    for attempt in 1..=3u32 {
        let seq = ((process::id() + attempt) & 0xffff) as u32;
        let result = Channel::open().and_then(|mut c| {
            c.drain().ok();
            c.exchange(seq, MASK_STANDBY, Duration::from_secs(5))
                .map_err(|e| std::io::Error::other(e.to_string()))
        });
        match result {
            Ok(()) => {
                eprintln!("[guard] standby restore ACKed (attempt {attempt})");
                process::exit(0);
            }
            Err(e) => eprintln!("[guard] restore attempt {attempt} failed: {e}"),
        }
        std::thread::sleep(Duration::from_secs(1));
    }
    eprintln!("[guard] FAILED to restore standby");
    process::exit(6);
}

/// Owner session: fork the guard, apply the mask, hold until stop, restore.
pub struct Owner {
    marker: std::path::PathBuf,
    channel: Channel,
    restored: bool,
}

impl Owner {
    /// Fork the guard FIRST (unless disabled in tests), then open the
    /// channel and apply the mask.
    pub fn start_with(
        mask: u32,
        timeout: Duration,
        send_path: &str,
        recv_path: &str,
        spawn_guard: bool,
    ) -> Result<Self, OwnerError> {
        let marker = std::env::temp_dir().join(format!("mask-owner-standdown-{}", process::id()));
        let _ = std::fs::remove_file(&marker);

        if spawn_guard {
            process::Command::new(std::env::current_exe().unwrap_or_default())
                .arg("--guard")
                .arg(process::id().to_string())
                .arg(marker.to_str().unwrap())
                .env("MASK_OWNER_SEND", send_path)
                .env("MASK_OWNER_RECV", recv_path)
                .spawn()
                .map_err(|_| OwnerError::WriteFailed)?;
        }

        let mut channel =
            Channel::open_with(send_path, recv_path).map_err(|_| OwnerError::WriteFailed)?;
        channel.drain().ok();
        channel.exchange(process::id() & 0xffff, mask, timeout)?;
        eprintln!("mask 0x{mask:08x} applied (ack)");
        Ok(Owner {
            marker,
            channel,
            restored: false,
        })
    }

    /// Production start: env-configured paths, guard enabled.
    pub fn start(mask: u32, timeout: Duration) -> Result<Self, OwnerError> {
        Self::start_with(
            mask,
            timeout,
            &default_send_fifo(),
            &default_recv_fifo(),
            true,
        )
    }

    /// Restore standby and stand the guard down. Idempotent: a second
    /// call is a no-op, so the explicit restore in main and the Drop
    /// cleanup never double-restore.
    pub fn restore(&mut self, timeout: Duration) -> Result<(), OwnerError> {
        if self.restored {
            return Ok(());
        }
        let r = self
            .channel
            .exchange((process::id() + 1) & 0xffff, MASK_STANDBY, timeout);
        if r.is_ok() {
            self.restored = true;
            std::fs::write(&self.marker, b"ok").ok();
        }
        r
    }
}

impl Drop for Owner {
    fn drop(&mut self) {
        // Best effort: the guard is the real safety net.
        let _ = self.restore(Duration::from_secs(5));
    }
}

/// Full rollback recipe: everything needed to stop the robot and hand
/// control back to the stock controller, executable entirely on the
/// robot with no network access to the workstation.
#[derive(Debug, Clone)]
pub struct Rollback {
    /// Player TCP host for the zero-velocity command (loopback on the robot).
    pub player_host: String,
    pub player_port: u16,
    pub send_fifo: String,
    pub recv_fifo: String,
    /// Shell command (script path + args) restoring the stock controller.
    pub restore_command: String,
}

/// Execute the rollback in the safety-relevant order:
/// 1. zero velocity over Player TCP (reaches the MCU immediately),
/// 2. standby mask over the FIFO (disables the wheel permission — an
///    independent second stop path that also zeroes the saved command),
/// 3. controller-restore command.
/// Steps are best-effort in sequence: a failure in an earlier step is
/// logged and the later steps still run, but the function reports the
/// first error so callers can retry.
pub fn full_rollback(r: &Rollback, timeout: Duration) -> Result<(), OwnerError> {
    let mut first_err = None;

    // 1. Zero velocity.
    match player_bridge::Client::connect(&r.player_host, r.player_port) {
        Ok(mut client) => {
            // Velocity commands (PLAYER_MSGTYPE_CMD) need no subscription:
            // send the stop directly, before any data could confuse us.
            if let Err(e) = client.send_velocity(0, 0.0, 0.0, 1) {
                eprintln!("[rollback] zero velocity failed: {e}");
                first_err.get_or_insert(OwnerError::WriteFailed);
            } else {
                eprintln!("[rollback] zero velocity sent");
            }
        }
        Err(e) => {
            eprintln!("[rollback] cannot reach Player for zero velocity: {e}");
            first_err.get_or_insert(OwnerError::WriteFailed);
        }
    }

    // 2. Standby mask (with a bounded number of retries).
    let mut mask_ok = false;
    for attempt in 1..=3u32 {
        let seq = ((std::process::id() + attempt) & 0xffff) as u32;
        let result = Channel::open_with(&r.send_fifo, &r.recv_fifo).and_then(|mut c| {
            c.drain().ok();
            c.exchange(seq, MASK_STANDBY, timeout)
                .map_err(|e| std::io::Error::other(e.to_string()))
        });
        match result {
            Ok(()) => {
                eprintln!("[rollback] standby mask applied (attempt {attempt})");
                mask_ok = true;
                break;
            }
            Err(e) => eprintln!("[rollback] standby attempt {attempt} failed: {e}"),
        }
        std::thread::sleep(Duration::from_secs(1));
    }
    if !mask_ok {
        first_err.get_or_insert(OwnerError::Timeout);
    }

    // 3. Restore the stock controller. Split the command string the
    // simple way (script + args, no quoting needs so far).
    let mut parts = r.restore_command.split_whitespace();
    if let Some(script) = parts.next() {
        let args: Vec<&str> = parts.collect();
        match std::process::Command::new(script).args(&args).status() {
            Ok(status) if status.success() => {
                eprintln!("[rollback] controller restore command succeeded");
            }
            Ok(status) => {
                eprintln!("[rollback] controller restore exited {status}");
                first_err.get_or_insert(OwnerError::WriteFailed);
            }
            Err(e) => {
                eprintln!("[rollback] cannot run controller restore: {e}");
                first_err.get_or_insert(OwnerError::WriteFailed);
            }
        }
    }

    match first_err {
        Some(e) => Err(e),
        None => Ok(()),
    }
}
