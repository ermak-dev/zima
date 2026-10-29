//! Opt-in local raw Player capture. The reader only clones a bounded payload and
//! try-sends; serialization and disk I/O belong exclusively to a detached thread.
use base64::{engine::general_purpose::STANDARD, Engine};
use player_bridge::Message;
use serde::Serialize;
use std::{
    fs::OpenOptions,
    io::{self, Write},
    os::unix::fs::{OpenOptionsExt, PermissionsExt},
    path::Path,
    sync::{
        atomic::{AtomicBool, AtomicU64, Ordering},
        mpsc::{self, Receiver, SyncSender, TrySendError},
        Arc, Mutex,
    },
    time::{Duration, Instant},
};

const MAX_PAYLOAD: usize = 64 * 1024;
const MAX_FILE: u64 = 64 * 1024 * 1024;
const FOOTER_RESERVE: u64 = 1024;
const QUEUE: usize = 128;

#[derive(Debug, Default, Clone, Serialize)]
pub struct Stats {
    pub complete: bool,
    pub writer_finished: bool,
    pub frames_written: u64,
    pub queue_dropped: u64,
    pub oversized: u64,
    pub size_limited: u64,
    pub error: Option<String>,
}
#[derive(Default)]
struct Shared {
    closing: AtomicBool,
    frames_written: AtomicU64,
    queue_dropped: AtomicU64,
    oversized: AtomicU64,
    size_limited: AtomicU64,
}
impl Shared {
    fn stats(&self) -> Stats {
        Stats {
            frames_written: self.frames_written.load(Ordering::Relaxed),
            queue_dropped: self.queue_dropped.load(Ordering::Relaxed),
            oversized: self.oversized.load(Ordering::Relaxed),
            size_limited: self.size_limited.load(Ordering::Relaxed),
            ..Stats::default()
        }
    }
}
struct Frame {
    message: Message,
    receive_offset_ns: u64,
}
pub struct Recorder {
    start: Instant,
    sender: SyncSender<Frame>,
    shared: Arc<Shared>,
    completion: Mutex<Receiver<Stats>>,
}
impl Recorder {
    /// Atomically creates a new owner-private file, refusing existing files and
    /// symlinks (including dangling ones). No thread or hardware is activated on failure.
    pub fn open(path: impl AsRef<Path>) -> io::Result<Self> {
        let start = Instant::now();
        let mut file = OpenOptions::new()
            .write(true)
            .create_new(true)
            .mode(0o600)
            .open(path)?;
        // mode() is filtered by umask; set the exact contract on the new inode,
        // never by reopening a pathname that another process could replace.
        file.set_permissions(std::fs::Permissions::from_mode(0o600))?;
        let header = serde_json::json!({
            "type": "header", "format": "robot-player-raw-v1",
            "receive_clock": "monotonic offset from recorder open in nanoseconds",
            "source_clock_verified": false,
            "max_file_bytes": MAX_FILE, "max_payload_bytes": MAX_PAYLOAD,
            "queue_capacity": QUEUE
        })
        .to_string()
            + "\n";
        file.write_all(header.as_bytes())?;
        let (sender, receiver) = mpsc::sync_channel(QUEUE);
        let (done, completion) = mpsc::channel();
        let shared = Arc::new(Shared::default());
        let state = shared.clone();
        std::thread::Builder::new()
            .name("player-recording".into())
            .spawn(move || {
                let result = write_frames(&mut file, header.len() as u64, receiver, &state);
                let mut stats = state.stats();
                stats.writer_finished = true;
                stats.complete = result.is_ok()
                    && stats.queue_dropped == 0
                    && stats.oversized == 0
                    && stats.size_limited == 0;
                stats.error = result.err().map(|e| e.to_string());
                if stats.error.is_none() {
                    let footer =
                        serde_json::json!({"type": "completion", "complete": stats.complete,
                    "frames_written": stats.frames_written, "queue_dropped": stats.queue_dropped,
                    "oversized": stats.oversized, "size_limited": stats.size_limited})
                        .to_string()
                            + "\n";
                    if let Err(e) = file.write_all(footer.as_bytes()).and_then(|_| file.flush()) {
                        stats.complete = false;
                        stats.error = Some(e.to_string());
                    }
                }
                let _ = done.send(stats);
            })?;
        Ok(Self {
            start,
            sender,
            shared,
            completion: Mutex::new(completion),
        })
    }

    pub fn from_env() -> io::Result<Option<Arc<Self>>> {
        match std::env::var_os("ROBOT_SLAM_RECORD_PATH") {
            Some(path) if !path.is_empty() => Self::open(path).map(|r| Some(Arc::new(r))),
            _ => Ok(None),
        }
    }

    /// Never waits for queue space or disk; losses are explicit in finish/footer.
    /// Call exactly once at receipt, before interpreting or filtering the message.
    pub fn record(&self, message: &Message, received: Instant) {
        if self.shared.closing.load(Ordering::Acquire) {
            return;
        }
        if message.payload.len() > MAX_PAYLOAD {
            self.shared.oversized.fetch_add(1, Ordering::Relaxed);
            return;
        }
        let frame = Frame {
            message: message.clone(),
            receive_offset_ns: received
                .saturating_duration_since(self.start)
                .as_nanos()
                .min(u64::MAX as u128) as u64,
        };
        match self.sender.try_send(frame) {
            Ok(()) => {}
            Err(TrySendError::Full(_)) | Err(TrySendError::Disconnected(_)) => {
                self.shared.queue_dropped.fetch_add(1, Ordering::Relaxed);
            }
        }
    }

    /// Call only after stopping producers, attempting final zero, and dropping
    /// heartbeat. Never joins the disk thread; even a stuck filesystem cannot
    /// delay safety teardown. A missing completion/footer is an incomplete capture.
    pub fn finish(&self, timeout: Duration) -> Stats {
        self.shared.closing.store(true, Ordering::Release);
        match self
            .completion
            .lock()
            .unwrap()
            .recv_timeout(timeout.min(Duration::from_secs(2)))
        {
            Ok(stats) => stats,
            Err(_) => Stats {
                error: Some(
                    "recording incomplete: writer completion timed out or disconnected".into(),
                ),
                ..self.shared.stats()
            },
        }
    }
}
impl Drop for Recorder {
    fn drop(&mut self) {
        self.shared.closing.store(true, Ordering::Release);
    }
}
fn write_frames(
    file: &mut impl Write,
    mut bytes: u64,
    receiver: Receiver<Frame>,
    shared: &Shared,
) -> io::Result<()> {
    loop {
        // Once producers have stopped, drain the bounded queue before completing.
        let frame = if shared.closing.load(Ordering::Acquire) {
            match receiver.try_recv() {
                Ok(f) => f,
                Err(_) => break,
            }
        } else {
            match receiver.recv_timeout(Duration::from_millis(10)) {
                Ok(f) => f,
                Err(mpsc::RecvTimeoutError::Timeout) => continue,
                Err(mpsc::RecvTimeoutError::Disconnected) => break,
            }
        };
        let m = frame.message;
        let h = m.header;
        let line = serde_json::json!({
            "type": "frame", "interface": h.addr.interf, "index": h.addr.index,
            "msg_type": h.msg_type, "subtype": h.subtype,
            "source_stamp": h.timestamp.is_finite().then_some(h.timestamp),
            "source_seq": h.seq, "receive_offset_ns": frame.receive_offset_ns,
            "payload_base64": STANDARD.encode(&m.payload)
        })
        .to_string()
            + "\n";
        if bytes + line.len() as u64 + FOOTER_RESERVE > MAX_FILE {
            shared.size_limited.fetch_add(1, Ordering::Relaxed);
            continue;
        }
        file.write_all(line.as_bytes())?;
        bytes += line.len() as u64;
        shared.frames_written.fetch_add(1, Ordering::Relaxed);
    }
    Ok(())
}
