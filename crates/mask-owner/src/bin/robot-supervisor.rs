//! robot-supervisor: the on-robot half of a bridge session.
//!
//! The workstation bridge starts `robot disable`, then connects here and
//! sends ACTIVATE. From that moment the supervisor:
//!   - applies the session device mask (LDS + wheels, brushes off) and
//!     holds it;
//!   - watches the heartbeat connection: bytes from the bridge mean
//!     "still alive";
//!   - when the connection dies (EOF) or goes silent past the timeout,
//!     performs the full rollback entirely on the robot, with no network
//!     access to the workstation: zero velocity over loopback Player TCP,
//!     the standby mask over the FIFO, then the controller-restore script.
//!
//! If ACTIVATE never arrives, the supervisor exits without touching
//! anything: a failed `robot disable` leaves a harmless listener behind.
//!
//! Environment:
//!   ROBOT_SUPERVISOR_PLAYER   host:port of Player for zero velocity
//!                             (default 127.0.0.1:6665)
//!   ROBOT_SUPERVISOR_RESTORE  controller-restore command line
//!   MASK_OWNER_SEND/RECV      FIFO paths (shared with mask-owner)

use std::io::{Read, Write};
use std::net::{TcpListener, TcpStream};
use std::time::{Duration, Instant};

use mask_owner::ipc::{full_rollback, Rollback};
use mask_owner::{MASK_NO_BRUSHES, MASK_STANDBY};

fn env_or(key: &str, default: &str) -> String {
    std::env::var(key).unwrap_or_else(|_| default.to_string())
}

/// Split "host:port" with a default port.
fn parse_player_addr(s: &str) -> (String, u16) {
    match s.rsplit_once(':') {
        Some((h, p)) => (h.to_string(), p.parse().unwrap_or(6665)),
        None => (s.to_string(), 6665),
    }
}

fn main() {
    let mut args: Vec<String> = std::env::args().skip(1).collect();
    if args.first().map(String::as_str) != Some("listen") {
        eprintln!("usage: robot-supervisor listen --heartbeat-port P --timeout-ms T [--activate-timeout-ms A] [--mask HEX]");
        std::process::exit(2);
    }
    args.remove(0);

    let mut heartbeat_port: u16 = 7665;
    let mut timeout_ms: u64 = 5000;
    let mut activate_timeout_ms: u64 = 10_000;
    let mut mask: u32 = MASK_NO_BRUSHES;
    let mut it = args.iter();
    while let Some(arg) = it.next() {
        match arg.as_str() {
            "--heartbeat-port" => {
                heartbeat_port = it.next().and_then(|v| v.parse().ok()).unwrap_or(heartbeat_port);
            }
            "--timeout-ms" => {
                timeout_ms = it.next().and_then(|v| v.parse().ok()).unwrap_or(timeout_ms);
            }
            "--activate-timeout-ms" => {
                activate_timeout_ms = it.next().and_then(|v| v.parse().ok()).unwrap_or(activate_timeout_ms);
            }
            "--mask" => {
                mask = it
                    .next()
                    .and_then(|v| u32::from_str_radix(v.trim_start_matches("0x"), 16).ok())
                    .unwrap_or(mask);
            }
            other => {
                eprintln!("robot-supervisor: unknown argument {other}");
                std::process::exit(2);
            }
        }
    }

    let listener = match TcpListener::bind(("0.0.0.0", heartbeat_port)) {
        Ok(l) => l,
        Err(e) => {
            eprintln!("robot-supervisor: cannot listen on {heartbeat_port}: {e}");
            std::process::exit(3);
        }
    };
    eprintln!("[supervisor] listening on {heartbeat_port}, timeout {timeout_ms} ms");

    // Wait for the bridge. The activation window is generous: the bridge
    // connects only after `robot disable` has fully succeeded.
    let (mut conn, _) = match listener.accept() {
        Ok(c) => c,
        Err(e) => {
            eprintln!("[supervisor] accept failed: {e}");
            std::process::exit(3);
        }
    };
    conn.set_read_timeout(Some(Duration::from_millis(activate_timeout_ms))).ok();
    let mut buf = [0u8; 64];
    let activated = match conn.read(&mut buf) {
        Ok(n) if n >= 8 && buf.starts_with(b"ACTIVATE") => true,
        Ok(_) => {
            eprintln!("[supervisor] no ACTIVATE; exiting without touching the robot");
            let _ = conn.write_all(b"REFUSED\n");
            std::process::exit(0);
        }
        Err(e) => {
            eprintln!("[supervisor] activation read failed: {e}; exiting");
            std::process::exit(0);
        }
    };
    if !activated {
        unreachable!();
    }
    let _ = conn.write_all(b"ACTIVATED\n");
    eprintln!("[supervisor] activated");

    let (player_host, player_port) = parse_player_addr(&env_or("ROBOT_SUPERVISOR_PLAYER", "127.0.0.1:6665"));
    let rollback = Rollback {
        player_host,
        player_port,
        send_fifo: env_or("MASK_OWNER_SEND", "/tmp/RcMc"),
        recv_fifo: env_or("MASK_OWNER_RECV", "/tmp/McRc"),
        restore_command: env_or("ROBOT_SUPERVISOR_RESTORE", "/tmp/mask-owner/controller-restore.sh"),
    };

    // Apply the session mask. A failure here means the robot state is
    // unknown: roll back immediately and exit nonzero.
    let timeout = Duration::from_millis(timeout_ms.max(1000));
    {
        let mut channel = match mask_owner::ipc::Channel::open_with(&rollback.send_fifo, &rollback.recv_fifo) {
            Ok(c) => c,
            Err(e) => {
                eprintln!("[supervisor] cannot open FIFOs: {e}");
                std::process::exit(4);
            }
        };
        channel.drain().ok();
        match channel.exchange(std::process::id() & 0xffff, mask, Duration::from_secs(5)) {
            Ok(()) => eprintln!("[supervisor] session mask 0x{mask:08x} applied"),
            Err(e) => {
                eprintln!("[supervisor] mask apply failed ({e}); rolling back");
                let _ = full_rollback(&rollback, timeout);
                std::process::exit(4);
            }
        }
        // Hold the FIFOs open for the whole session (Mc must always have
        // a reader), and keep the channel for the graceful restore.
        hold_until_dead(&mut conn, timeout_ms);

        // Graceful path: the bridge said GOODBYE (or died — EOF is
        // handled the same way). Restore standby ourselves.
        let _ = channel.drain();
        match channel.exchange((std::process::id() + 1) & 0xffff, MASK_STANDBY, Duration::from_secs(5)) {
            Ok(()) => eprintln!("[supervisor] graceful standby restore done"),
            Err(e) => eprintln!("[supervisor] graceful restore failed ({e}); full rollback follows"),
        }
    }

    // Controller restore always runs (it is idempotent).
    if let Err(e) = run_restore(&rollback) {
        eprintln!("[supervisor] controller restore failed: {e}");
        std::process::exit(5);
    }
    eprintln!("[supervisor] done");
}

/// Read the heartbeat until EOF, a GOODBYE line, or `timeout_ms` of
/// silence. Any of the three returns: the caller then restores.
fn hold_until_dead(conn: &mut TcpStream, timeout_ms: u64) {
    let timeout = Duration::from_millis(timeout_ms.max(100));
    conn.set_read_timeout(Some(timeout)).ok();
    let mut buf = [0u8; 64];
    let mut last_beat = Instant::now();
    loop {
        match conn.read(&mut buf) {
            Ok(0) => {
                eprintln!("[supervisor] bridge disconnected");
                return;
            }
            Ok(n) => {
                if buf[..n].starts_with(b"GOODBYE") {
                    eprintln!("[supervisor] bridge said goodbye");
                    return;
                }
                last_beat = Instant::now();
            }
            Err(e)
                if e.kind() == std::io::ErrorKind::WouldBlock
                    || e.kind() == std::io::ErrorKind::TimedOut =>
            {
                if last_beat.elapsed() >= Duration::from_millis(timeout_ms.max(100)) {
                    eprintln!("[supervisor] heartbeat timed out");
                    return;
                }
            }
            Err(e) => {
                eprintln!("[supervisor] heartbeat error: {e}");
                return;
            }
        }
    }
}

fn run_restore(r: &Rollback) -> Result<(), String> {
    let mut parts = r.restore_command.split_whitespace();
    if let Some(script) = parts.next() {
        let args: Vec<&str> = parts.collect();
        match std::process::Command::new(script).args(&args).status() {
            Ok(s) if s.success() => return Ok(()),
            Ok(s) => return Err(format!("restore exited {s}")),
            Err(e) => return Err(format!("cannot run restore: {e}")),
        }
    }
    Ok(())
}
