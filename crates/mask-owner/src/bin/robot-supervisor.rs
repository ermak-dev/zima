//! Persistent on-robot supervisor. Only the exclusive ACTIVATE owner may
//! touch FIFOs or restore the controller. Audio clients run isolated guards
//! from this same ELF and never enter the control ownership path.
use std::io::{Read, Write};
use std::net::{TcpListener, TcpStream};
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use mask_owner::ipc::{full_rollback, Rollback};
use mask_owner::{MASK_NO_BRUSHES, MASK_STANDBY};
#[path = "../supervisor_audio.rs"]
mod audio;
#[path = "../compass.rs"]
mod compass;

fn env_or(key: &str, default: &str) -> String {
    std::env::var(key).unwrap_or_else(|_| default.to_string())
}

fn parse_player_addr(s: &str) -> (String, u16) {
    match s.rsplit_once(':') {
        Some((h, p)) => (h.to_string(), p.parse().unwrap_or(6665)),
        None => (s.to_string(), 6665),
    }
}

struct Config {
    timeout_ms: u64,
    activate_timeout_ms: u64,
    mask: u32,
    token: Option<String>,
    rollback: Rollback,
    // Held through the entire rollback, not merely until owner disconnect.
    owner: Mutex<()>,
    clients: AtomicUsize,
    stopping: AtomicBool,
    // Bounded local drive lease (#55): one active deadline at a time; the
    // zero is sent by the supervisor thread itself, never by the host.
    drive: Arc<Mutex<Option<std::sync::mpsc::Sender<()>>>>,
}

impl Config {
    fn drive_lease(&self) -> Arc<Mutex<Option<std::sync::mpsc::Sender<()>>>> {
        self.drive.clone()
    }
}

/// Send a velocity command to the local Player. Best-effort: failures are
/// logged by the caller; the deadline/stop logic never depends on the
/// network beyond the robot itself.
fn send_velocity(r: &Rollback, vx: f64, vyaw: f64) -> Result<(), String> {
    let mut client = player_bridge::Client::connect(&r.player_host, r.player_port)
        .map_err(|e| format!("cannot reach Player: {e}"))?;
    client
        .send_velocity(0, vx, vyaw, 1)
        .map_err(|e| format!("velocity write failed: {e}"))
}

/// Validate a bounded DRIVE request. Returns None for a malformed line.
fn parse_drive(request: &str) -> Option<(f64, f64, u64)> {
    let fields: Vec<&str> = request.split_whitespace().collect();
    if fields.len() != 4 || fields[0] != "DRIVE" {
        return None;
    }
    let vx: f64 = fields[1].parse().ok()?;
    let vyaw: f64 = fields[2].parse().ok()?;
    let ms: u64 = fields[3].parse().ok()?;
    if !vx.is_finite() || !vyaw.is_finite() || ms == 0 || ms > 60_000 {
        return None;
    }
    // Reuse the shared chassis ceilings; they are clamped, not rejected, so
    // the panel's slider ranges keep working.
    let (vx, vyaw) = player_bridge::drive::clamp_velocity(vx, vyaw);
    Some((vx, vyaw, ms))
}

/// Execute a bounded drive: send the velocity, acknowledge the caller, then
/// hold the lease until STOP or the local deadline, then send the zero. The
/// zero is always attempted by this local thread, never by the host.
fn run_bounded_drive(
    rollback: &Rollback,
    drive: &Arc<Mutex<Option<std::sync::mpsc::Sender<()>>>>,
    vx: f64,
    vyaw: f64,
    ms: u64,
    ack: std::sync::mpsc::Sender<bool>,
) -> bool {
    if send_velocity(rollback, vx, vyaw).is_err() {
        let _ = ack.send(false);
        return false;
    }
    let (tx, rx) = std::sync::mpsc::channel::<()>();
    *drive.lock().unwrap() = Some(tx);
    let _ = ack.send(true);
    // Hold the lease on this thread: STOP wakes the channel, the deadline
    // bounds the wait. Either way the local zero follows.
    let _cancelled = rx.recv_timeout(Duration::from_millis(ms)).is_ok();
    drive.lock().unwrap().take();
    let _ = send_velocity(rollback, 0.0, 0.0);
    true
}

fn main() {
    audio::dispatch();
    let mut args: Vec<String> = std::env::args().skip(1).collect();
    if args.first().map(String::as_str) != Some("listen") {
        eprintln!("usage: robot-supervisor listen --heartbeat-port P --timeout-ms T [--activate-timeout-ms A] [--mask HEX]");
        std::process::exit(2);
    }
    args.remove(0);
    let mut listen = "0.0.0.0".to_owned();
    let mut heartbeat_port: u16 = 7665;
    let mut timeout_ms: u64 = 5000;
    let mut activate_timeout_ms: u64 = 10_000;
    let mut mask: u32 = MASK_NO_BRUSHES;
    let mut it = args.iter();
    while let Some(arg) = it.next() {
        match arg.as_str() {
            "--listen" => {
                listen = it.next().cloned().unwrap_or_else(|| {
                    eprintln!("robot-supervisor: --listen requires an address");
                    std::process::exit(2);
                });
            }
            "--heartbeat-port" => {
                heartbeat_port = it
                    .next()
                    .and_then(|v| v.parse().ok())
                    .unwrap_or(heartbeat_port)
            }
            "--timeout-ms" => {
                timeout_ms = it.next().and_then(|v| v.parse().ok()).unwrap_or(timeout_ms)
            }
            "--activate-timeout-ms" => {
                activate_timeout_ms = it
                    .next()
                    .and_then(|v| v.parse().ok())
                    .unwrap_or(activate_timeout_ms)
            }
            "--mask" => {
                mask = it
                    .next()
                    .and_then(|v| u32::from_str_radix(v.trim_start_matches("0x"), 16).ok())
                    .unwrap_or(mask)
            }
            other => {
                eprintln!("robot-supervisor: unknown argument {other}");
                std::process::exit(2);
            }
        }
    }
    // Validate before listening, rather than discovering an invalid mask after
    // the host has disabled its stock controller.
    if mask_owner::build_request(0, mask).is_err() {
        eprintln!("robot-supervisor: invalid mask");
        std::process::exit(2);
    }
    let (player_host, player_port) =
        parse_player_addr(&env_or("ROBOT_SUPERVISOR_PLAYER", "127.0.0.1:6665"));
    let config = Arc::new(Config {
        timeout_ms,
        activate_timeout_ms,
        mask,
        token: std::env::var("ROBOT_SUPERVISOR_TOKEN").ok(),
        rollback: Rollback {
            player_host,
            player_port,
            send_fifo: env_or("MASK_OWNER_SEND", "/tmp/RcMc"),
            recv_fifo: env_or("MASK_OWNER_RECV", "/tmp/McRc"),
            restore_command: env_or(
                "ROBOT_SUPERVISOR_RESTORE",
                "/tmp/mask-owner/controller-restore.sh",
            ),
        },
        owner: Mutex::new(()),
        clients: AtomicUsize::new(0),
        stopping: AtomicBool::new(false),
        drive: Arc::new(Mutex::new(None)),
    });
    let listener = TcpListener::bind((listen.as_str(), heartbeat_port)).unwrap_or_else(|e| {
        eprintln!("robot-supervisor: cannot listen on {heartbeat_port}: {e}");
        std::process::exit(3);
    });
    eprintln!("[supervisor] listening on {heartbeat_port}, timeout {timeout_ms} ms");
    for incoming in listener.incoming() {
        let Ok(mut conn) = incoming else { continue };
        // Bounded clients and per-request deadlines prevent idle connections
        // from consuming unbounded threads/memory. Never block accept on I/O.
        if config
            .clients
            .fetch_update(Ordering::SeqCst, Ordering::SeqCst, |n| {
                (n < 32).then_some(n + 1)
            })
            .is_err()
        {
            continue;
        }
        let c = config.clone();
        let result = std::thread::Builder::new().spawn(move || {
            struct ClientSlot(Arc<Config>);
            impl Drop for ClientSlot {
                fn drop(&mut self) {
                    self.0.clients.fetch_sub(1, Ordering::SeqCst);
                }
            }
            let _slot = ClientSlot(c.clone());
            let _ = conn.set_write_timeout(Some(Duration::from_secs(2)));
            serve(&mut conn, &c);
        });
        if result.is_err() {
            config.clients.fetch_sub(1, Ordering::SeqCst);
        }
    }
}

/// Bytewise bounded LF framing leaves a coalesced WAV body or heartbeat unread.
fn line(conn: &mut TcpStream, deadline: Instant) -> std::io::Result<String> {
    let mut bytes = Vec::new();
    loop {
        let remaining = deadline.saturating_duration_since(Instant::now());
        if remaining.is_zero() {
            return Err(std::io::ErrorKind::TimedOut.into());
        }
        conn.set_read_timeout(Some(remaining))?;
        let mut byte = [0];
        conn.read_exact(&mut byte)?;
        if byte[0] == b'\n' {
            return String::from_utf8(bytes).map_err(|_| std::io::ErrorKind::InvalidData.into());
        }
        bytes.push(byte[0]);
        if bytes.len() > 256 {
            return Err(std::io::ErrorKind::InvalidData.into());
        }
    }
}

fn serve(conn: &mut TcpStream, c: &Config) {
    // One command per connection, except DRIVE/STOP clients which stay to
    // issue a bounded sequence of drive commands and finally STOP; each
    // follow-up line gets a fresh bounded deadline.
    let deadline = Instant::now() + Duration::from_millis(c.activate_timeout_ms.max(1));
    let Ok(mut request) = line(conn, deadline) else {
        return;
    };
    let mut drive_session = false;
    if let Some(token) = &c.token {
        if request.strip_prefix("AUTH ") != Some(token.as_str()) || token.is_empty() {
            let _ = conn.write_all(b"UNAUTHORIZED\n");
            return;
        }
        if conn.write_all(b"AUTHENTICATED\n").is_err() {
            return;
        }
        let follow = Instant::now() + Duration::from_millis(c.activate_timeout_ms.max(1));
        let Ok(next) = line(conn, follow) else {
            return;
        };
        request = next;
    }
    loop {
        // Accept counts each handler before it can observe this gate. A
        // handler that saw false is therefore included in SHUTDOWN's final
        // client count; handlers accepted after that count see true and
        // cannot start any work.
        if c.stopping.load(Ordering::SeqCst) {
            let _ = conn.write_all(b"BUSY\n");
            return;
        }
        if request == "SHUTDOWN" {
            // Unlike legacy commands, shutdown is never available without AUTH.
            if c.token.is_none() {
                let _ = conn.write_all(b"UNAUTHORIZED\n");
                return;
            }
            // Only the thread that closes the gate may reopen it on refusal.
            if c.stopping.swap(true, Ordering::SeqCst) {
                let _ = conn.write_all(b"BUSY\n");
                return;
            }
            if c.clients.load(Ordering::SeqCst) == 1 {
                if let Ok(_owner) = c.owner.try_lock() {
                    // Keep the gate and owner lock through process exit. No control
                    // restoration is needed (or permitted) for an idle shutdown.
                    let _ = conn.write_all(b"STOPPING\n");
                    std::process::exit(0);
                }
            }
            c.stopping.store(false, Ordering::SeqCst);
            let _ = conn.write_all(b"BUSY\n");
        } else if request == "PING" {
            let reply: &[u8] = if c.owner.try_lock().is_ok() {
                b"PONG IDLE\n"
            } else {
                b"PONG ACTIVE\n"
            };
            let _ = conn.write_all(reply);
        } else if request == "COMPASS" {
            compass::observe(conn);
        } else if request == "ACTIVATE" {
            let Ok(_owner) = c.owner.try_lock() else {
                let _ = conn.write_all(b"BUSY\n");
                return;
            };
            control(conn, c);
        } else if request.starts_with("AUDIO ") {
            audio::play(conn, &request.split_whitespace().collect::<Vec<_>>());
        } else if let Some(token) = request.strip_prefix("CANCEL ") {
            audio::cancel(conn, token);
        } else if request.starts_with("DRIVE") {
            drive_session = true;
            match parse_drive(&request) {
                Some((vx, vyaw, ms)) => {
                    // One bounded drive at a time: a second lease is refused,
                    // not queued; the session stays open for STOP.
                    if c.drive.lock().unwrap().is_some() {
                        let _ = conn.write_all(b"DRIVE_BUSY\n");
                    } else {
                        // The velocity write is confirmed before DRIVING; the deadline
                        // then runs on its own thread so STOP keeps this connection.
                        let (ack_tx, ack_rx) = std::sync::mpsc::channel::<bool>();
                        let rollback = c.rollback.clone();
                        let drive = c.drive_lease();
                        let _holder = std::thread::spawn(move || {
                            run_bounded_drive(&rollback, &drive, vx, vyaw, ms, ack_tx);
                        });
                        match ack_rx.recv_timeout(Duration::from_secs(5)) {
                            Ok(true) => {
                                let _ = conn.write_all(b"DRIVING\n");
                            }
                            _ => {
                                let _ = conn.write_all(b"DRIVE_FAILED\n");
                            }
                        }
                    }
                }
                None => {
                    let _ = conn.write_all(b"INVALID_DRIVE\n");
                }
            }
        } else if request == "STOP" {
            drive_session = true;
            if let Some(tx) = c.drive.lock().unwrap().take() {
                // Waking the holder is enough: it owns the local zero send.
                let _ = tx.send(());
            }
            let _ = send_velocity(&c.rollback, 0.0, 0.0);
            let _ = conn.write_all(b"STOPPED\n");
        } else if request == "QUIT" {
            return;
        } else {
            let _ = conn.write_all(b"REFUSED\n");
        }
        // Only drive sessions keep the command loop; one-shot clients
        // (ACTIVATE/AUDIO/COMPASS/SHUTDOWN) retain their original semantics.
        if !drive_session {
            return;
        }
        let follow = Instant::now() + Duration::from_millis(c.activate_timeout_ms.max(1));
        match line(conn, follow) {
            Ok(next) => request = next,
            Err(_) => return,
        }
    }
}

fn control(conn: &mut TcpStream, c: &Config) {
    let r = &c.rollback;
    let timeout = Duration::from_millis(c.timeout_ms.max(1000));
    let mut channel = match mask_owner::ipc::Channel::open_with(&r.send_fifo, &r.recv_fifo) {
        Ok(channel) => channel,
        Err(e) => {
            eprintln!("[supervisor] cannot open FIFOs: {e}");
            failed_handoff(conn, r, timeout);
        }
    };
    let _ = channel.drain();
    if let Err(e) = channel.exchange(std::process::id() & 0xffff, c.mask, Duration::from_secs(5)) {
        eprintln!("[supervisor] mask apply failed ({e}); rolling back");
        drop(channel);
        failed_handoff(conn, r, timeout);
    }
    // Report ACTIVATED only after the FIFO ACK, and roll back even if this
    // acknowledgement cannot be delivered to the host.
    if conn.write_all(b"ACTIVATED\n").is_ok() {
        hold_until_dead(conn, c.timeout_ms);
    }
    let _ = channel.drain();
    if let Err(e) = channel.exchange(
        (std::process::id() + 1) & 0xffff,
        MASK_STANDBY,
        Duration::from_secs(5),
    ) {
        eprintln!("[supervisor] standby failed ({e}); full rollback follows");
        drop(channel);
        failed_handoff(conn, r, timeout);
    }
    drop(channel);
    if let Err(e) = run_restore(r) {
        // Do not hand control to a new client after a failed restoration.
        eprintln!("[supervisor] controller restore failed: {e}");
        std::process::exit(5);
    }
}

/// Legacy `hb` bytes remain accepted. Keep a bounded suffix so a fragmented
/// GOODBYE is recognized as well; EOF and silence have the same rollback.
fn hold_until_dead(conn: &mut TcpStream, timeout_ms: u64) {
    let _ = conn.set_read_timeout(Some(Duration::from_millis(timeout_ms.max(100))));
    let mut buf = [0; 64];
    let mut suffix = Vec::new();
    loop {
        match conn.read(&mut buf) {
            Ok(0) | Err(_) => return,
            Ok(n) => {
                suffix.extend_from_slice(&buf[..n]);
                if suffix.windows(8).any(|w| w == b"GOODBYE\n") {
                    return;
                }
                if suffix.len() > 7 {
                    suffix.drain(..suffix.len() - 7);
                }
            }
        }
    }
}

fn failed_handoff(conn: &mut TcpStream, r: &Rollback, timeout: Duration) -> ! {
    let _ = full_rollback(r, timeout);
    let _ = conn.write_all(b"ERROR\n");
    // Preserve the old fail-closed behavior: uncertain control state must not
    // become a reusable persistent listener, regardless of rollback's result.
    std::process::exit(4);
}

fn run_restore(r: &Rollback) -> Result<(), String> {
    let mut parts = r.restore_command.split_whitespace();
    if let Some(script) = parts.next() {
        match std::process::Command::new(script).args(parts).status() {
            Ok(s) if s.success() => return Ok(()),
            Ok(s) => return Err(format!("restore exited {s}")),
            Err(e) => return Err(format!("cannot run restore: {e}")),
        }
    }
    Ok(())
}
