//! Passive session bundles. Only preflight touches disk on the caller thread.
use crate::player_recording::Recorder;
use serde_json::{json, Value};
use std::{
    fs::{self, DirBuilder, File, OpenOptions},
    io::{self, Seek, SeekFrom, Write},
    os::{
        fd::AsRawFd,
        unix::fs::{DirBuilderExt, OpenOptionsExt, PermissionsExt},
    },
    path::{Component, Path, PathBuf},
    sync::{
        atomic::{AtomicBool, AtomicU64, Ordering},
        mpsc::{self, Receiver, SyncSender},
        Arc, Mutex,
    },
    time::{Duration, Instant},
};
const QUEUE: usize = 32;
const JPEG: usize = 2 * 1024 * 1024;
const MAX_EVENT: usize = 256 * 1024;
const MAX_BYTES: u64 = 64 * 1024 * 1024;
const MAX_DURATION: Duration = Duration::from_secs(3600);
struct Item {
    row: Value,
    jpeg: Option<Vec<u8>>,
}
#[derive(Default)]
struct Shared {
    closing: AtomicBool,
    dropped: AtomicU64,
    camera_dropped: AtomicU64,
    sensor_dropped: AtomicU64,
    camera: Mutex<String>,
    final_data: Mutex<Option<Value>>,
}
impl Shared {
    fn loss(&self, kind: &str) {
        self.dropped.fetch_add(1, Ordering::Relaxed);
        match kind {
            "camera_frame" => {
                self.camera_dropped.fetch_add(1, Ordering::Relaxed);
            }
            "state" | "scan" => {
                self.sensor_dropped.fetch_add(1, Ordering::Relaxed);
            }
            _ => {}
        }
    }
}
#[derive(Default)]
struct Written {
    events: u64,
    frames: u64,
    states: u64,
    scans: u64,
}
pub struct Session {
    start: Instant,
    pub raw: Arc<Recorder>,
    tx: SyncSender<Item>,
    shared: Arc<Shared>,
    done: Mutex<Receiver<Value>>,
}
fn private(path: impl AsRef<Path>) -> io::Result<File> {
    let f = OpenOptions::new()
        .write(true)
        .create_new(true)
        .mode(0o600)
        .open(path)?;
    f.set_permissions(fs::Permissions::from_mode(0o600))?;
    Ok(f)
}
// Linux server: walk parents through pinned descriptors with O_NOFOLLOW, rather
// than checking a pathname and reopening it. Keep the new directory pinned for
// later JPEG writes even if someone renames/replaces its original pathname.
fn directory(path: &Path) -> io::Result<(File, PathBuf)> {
    const DIRECTORY_NOFOLLOW: i32 = 0o200000 | 0o400000;
    let absolute = std::path::absolute(path)?;
    let name = absolute
        .file_name()
        .ok_or_else(|| io::Error::other("session directory requires a new name"))?;
    let mut parent = File::open("/")?;
    for part in absolute.parent().unwrap().components() {
        if let Component::Normal(name) = part {
            parent = OpenOptions::new()
                .read(true)
                .custom_flags(DIRECTORY_NOFOLLOW)
                .open(PathBuf::from(format!("/proc/self/fd/{}", parent.as_raw_fd())).join(name))?;
        } else if part == Component::ParentDir {
            return Err(io::Error::other("session path must not contain '..'"));
        }
    }
    let child = PathBuf::from(format!("/proc/self/fd/{}", parent.as_raw_fd())).join(name);
    DirBuilder::new().mode(0o700).create(&child)?;
    let dir = OpenOptions::new()
        .read(true)
        .custom_flags(DIRECTORY_NOFOLLOW)
        .open(child)?;
    dir.set_permissions(fs::Permissions::from_mode(0o700))?;
    let pinned = PathBuf::from(format!("/proc/self/fd/{}", dir.as_raw_fd()));
    Ok((dir, pinned))
}
impl Session {
    pub fn open(path: impl AsRef<Path>) -> io::Result<Self> {
        let (directory, path) = directory(path.as_ref())?;
        let start = Instant::now();
        let raw = Arc::new(Recorder::open_at(path.join("player.jsonl"), start)?);
        let mut manifest = private(path.join("manifest.json"))?;
        let header = json!({
            "format": "robot-session-v1",
            "complete": false,
            "status": "recording_or_interrupted",
            "clock": "monotonic nanoseconds since session start; receipt, not exposure/measurement time",
            "source_clock_verified": false,
            "map": "omitted",
            "queue_capacity": QUEUE,
            "max_event_bytes": MAX_EVENT,
            "max_jpeg_bytes": JPEG,
            "max_event_and_jpeg_bytes": MAX_BYTES,
            "max_duration_seconds": MAX_DURATION.as_secs(),
            "raw_player_max_bytes": 64 * 1024 * 1024,
            "camera_source": "dedicated HTTP MJPEG stream via configured Camera; credentials never stored"
        });
        manifest.write_all(header.to_string().as_bytes())?;
        manifest.flush()?;
        let events = private(path.join("events.jsonl"))?;
        let (tx, rx) = mpsc::sync_channel(QUEUE);
        let (done, completion) = mpsc::channel();
        let shared = Arc::new(Shared::default());
        let s = shared.clone();
        std::thread::Builder::new()
            .name("session-recording".into())
            .spawn(move || {
                let _directory = directory;
                let mut counts = Written::default();
                let result = write_items(events, &rx, &s, &path, start, &mut counts);
                if result.is_err() {
                    // A local writer failure must not invent the still-running
                    // raw recorder's result. Discard with accounting until the
                    // caller publishes final raw stats and closes this worker.
                    while !s.closing.load(Ordering::Acquire) {
                        match rx.recv_timeout(Duration::from_millis(10)) {
                            Ok(item) => s.loss(item.row["type"].as_str().unwrap_or("unknown")),
                            Err(mpsc::RecvTimeoutError::Timeout) => {},
                            Err(_) => break,
                        }
                    }
                    for item in rx.try_iter().take(QUEUE) {
                        s.loss(item.row["type"].as_str().unwrap_or("unknown"));
                    }
                }
                let base = s.final_data.lock().unwrap().clone().unwrap_or_else(||
                    json!({"raw_player":{"complete":false},"session_ok":false}));
                let dropped = s.dropped.load(Ordering::Relaxed);
                let Written {events:written, frames, states, scans} = counts;
                let events_ok = result.is_ok() && dropped == 0;
                let camera_dropped = s.camera_dropped.load(Ordering::Relaxed);
                let sensor_dropped = s.sensor_dropped.load(Ordering::Relaxed);
                let camera_ok = result.is_ok() && camera_dropped == 0;
                let sensors_ok = result.is_ok() && sensor_dropped == 0;
                let camera_reason = s.camera.lock().unwrap().clone();
                let camera_status = if frames == 0 { "unavailable" }
                    else if camera_reason == "cancelled" && camera_ok { "recorded_until_cancel" }
                    else { "partial" };
                let mut report = header;
                report["status"] = json!("finalized");
                report["events"] = json!({"complete":events_ok,"written":written,"dropped":dropped,
                    "error":result.err().map(|_|"writer I/O failure")});
                report["sensors"] = json!({"status":if states==0 && scans==0 {"unavailable"} else if sensors_ok {"sampled"} else {"partial"},
                    "dropped":sensor_dropped,"state_snapshots":states,"scan_snapshots":scans,"state_interval_ms":100,
                    "timestamp_semantics":"snapshot sampling time; scan may be decimated by latest-only watch channel"});
                report["camera"] = json!({"status":camera_status,"frames":frames,"dropped":camera_dropped,"end_reason":camera_reason});
                report["raw_player"] = base["raw_player"].clone();
                report["raw_player"]["status"] = json!(if base["raw_player"]["frames_written"]==0 {"unavailable"}
                    else if base["raw_player"]["complete"]==true {"recorded"} else {"partial"});
                report["session_ok"] = base["session_ok"].clone();
                report["complete"] = json!(events_ok && base["raw_player"]["complete"]==true
                    && base["session_ok"]==true && camera_status=="recorded_until_cancel" && states>0
                    && report["raw_player"]["status"]=="recorded");
                let saved = (|| -> io::Result<()> {
                    manifest.seek(SeekFrom::Start(0))?;
                    manifest.set_len(0)?;
                    manifest.write_all(report.to_string().as_bytes())?;
                    manifest.flush()
                })();
                if saved.is_err() {
                    report["complete"] = json!(false);
                    report["manifest_error"] = json!(true);
                }
                let _ = done.send(report);
            })?;
        Ok(Self {
            start,
            raw,
            tx,
            shared,
            done: Mutex::new(completion),
        })
    }
    pub fn from_env() -> io::Result<Option<Arc<Self>>> {
        match std::env::var_os("ROBOT_SESSION_RECORD_DIR") {
            Some(p) if !p.is_empty() => {
                if std::env::var_os("ROBOT_SLAM_RECORD_PATH").is_some_and(|p| !p.is_empty()) {
                    return Err(io::Error::other(
                        "session recording conflicts with ROBOT_SLAM_RECORD_PATH",
                    ));
                }
                Self::open(p).map(|s| Some(Arc::new(s)))
            }
            _ => Ok(None),
        }
    }
    pub fn start(&self) -> Instant {
        self.start
    }
    fn enqueue(&self, kind: &str, data: Value, jpeg: Option<Vec<u8>>, received: Instant) {
        if self.shared.closing.load(Ordering::Acquire) {
            return;
        }
        let row = json!({"type":kind,"receive_offset_ns":received.saturating_duration_since(self.start).as_nanos().min(u64::MAX as u128) as u64,"data":data});
        if received.saturating_duration_since(self.start) > MAX_DURATION
            || jpeg.as_ref().is_some_and(|v| v.len() > JPEG)
            || self.tx.try_send(Item { row, jpeg }).is_err()
        {
            self.shared.loss(kind);
        }
    }
    pub fn loss(&self) {
        self.shared.loss("camera_frame");
    }
    pub fn snapshot(&self, kind: &str, value: &Value, received: Instant) {
        if !matches!(kind, "state" | "scan") {
            return;
        }
        // A conservative serialized-size budget bounds cloning and queued memory;
        // repeated maps and secret-bearing fields are skipped before traversing them.
        let mut budget = MAX_EVENT - 1024;
        match bounded_snapshot(value, &mut budget, 0) {
            Some(data) => self.enqueue(kind, data, None, received),
            None => self.shared.loss(kind),
        }
    }
    pub fn command(&self, client: u64, wheels: Option<(f64, f64)>, received: Instant) {
        match wheels {
            Some((v, w)) => self.enqueue(
                "wheel_command",
                json!({"client":client, "v":v, "w":w, "meaning":"inbound request, not applied velocity"}),
                None,
                received,
            ),
            None => self.enqueue("reset_blocked", json!({"client":client}), None, received),
        }
    }
    pub fn jpeg(&self, jpeg: Vec<u8>, received: Instant) {
        self.enqueue("camera_frame", Value::Null, Some(jpeg), received)
    }
    pub fn camera_end(&self, reason: &str) {
        let mut end = self.shared.camera.lock().unwrap();
        if end.is_empty() {
            *end = match reason {
                "cancelled" | "timeout" | "eof" | "size_limit" | "unavailable" => reason,
                _ => "unavailable",
            }
            .into();
        }
    }
    pub fn finish(&self, timeout: Duration, session_ok: bool) -> Value {
        let budget = timeout.min(Duration::from_secs(2));
        let began = Instant::now();
        let raw = self.raw.finish(budget);
        *self.shared.final_data.lock().unwrap() =
            Some(json!({"raw_player":raw,"session_ok":session_ok}));
        self.shared.closing.store(true, Ordering::Release);
        self.done
            .lock()
            .unwrap()
            .recv_timeout(budget.saturating_sub(began.elapsed()))
            .unwrap_or_else(|_| json!({"complete":false,"status":"writer_timeout_or_disconnected"}))
    }
}
impl Drop for Session {
    fn drop(&mut self) {
        self.shared.closing.store(true, Ordering::Release);
    }
}
fn write_items(
    mut events: File,
    rx: &Receiver<Item>,
    s: &Shared,
    path: &Path,
    start: Instant,
    counts: &mut Written,
) -> io::Result<()> {
    let mut bytes = 0u64;
    loop {
        let item = if s.closing.load(Ordering::Acquire) {
            match rx.try_recv() {
                Ok(i) => i,
                Err(_) => break,
            }
        } else {
            match rx.recv_timeout(Duration::from_millis(10)) {
                Ok(i) => i,
                Err(mpsc::RecvTimeoutError::Timeout) => continue,
                Err(_) => break,
            }
        };
        let mut row = item.row;
        let name = format!("frame-{:08}.jpg", counts.frames);
        if item.jpeg.is_some() {
            row["data"] = json!({"file":name});
        }
        let line = row.to_string() + "\n";
        let size = line.len() as u64 + item.jpeg.as_ref().map_or(0, |v| v.len()) as u64;
        if line.len() > MAX_EVENT || bytes + size > MAX_BYTES || start.elapsed() > MAX_DURATION {
            s.loss(row["type"].as_str().unwrap_or("unknown"));
            continue;
        }
        if let Some(jpeg) = item.jpeg {
            let mut f = private(path.join(name))?;
            f.write_all(&jpeg)?;
            counts.frames += 1;
        }
        events.write_all(line.as_bytes())?;
        if row["type"] == "state" {
            counts.states += 1;
        }
        if row["type"] == "scan" {
            counts.scans += 1;
        }
        bytes += size;
        counts.events += 1;
    }
    events.flush()?;
    Ok(())
}
// Cancellation never joins HTTP or disk work and cannot delay physical teardown.
pub struct Sources {
    tasks: Vec<tokio::task::JoinHandle<()>>,
    recording: Arc<Session>,
}
impl Drop for Sources {
    fn drop(&mut self) {
        for task in &self.tasks {
            task.abort();
        }
        self.recording.camera_end("cancelled");
    }
}
/// The caller supplies a factory for the existing authenticated/no-redirect
/// Camera::stream. Failures while the camera boots (shared LDS power) are
/// retried on a bounded overall budget; no credentials, camera URL, Player
/// client, or drive handle enter this module.
pub fn start_sources<F, Fut, E>(
    recording: Arc<Session>,
    camera: F,
    mut state: tokio::sync::watch::Receiver<Value>,
    mut scan: tokio::sync::watch::Receiver<Option<Value>>,
) -> Sources
where
    F: Fn() -> Fut + Send + 'static,
    Fut: std::future::Future<Output = Result<reqwest::Response, E>> + Send + 'static,
    E: Send + 'static,
{
    use futures_util::StreamExt;
    const OPEN_ATTEMPT: Duration = Duration::from_secs(5);
    // The OpenIPC camera shares LDS power and needs about a minute to boot.
    const BOOT_BUDGET: Duration = Duration::from_secs(90);
    let r = recording.clone();
    let snapshots = tokio::spawn(async move {
        let mut timer = tokio::time::interval(Duration::from_millis(100));
        timer.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
        loop {
            tokio::select! {
                _=timer.tick()=>{ r.snapshot("state", &state.borrow_and_update(), Instant::now()); },
                changed=scan.changed()=>{if changed.is_err(){break;} if let Some(v)=scan.borrow_and_update().as_ref(){r.snapshot("scan",v,Instant::now());}}
            }
        }
    });
    let r = recording.clone();
    let capture = tokio::spawn(async move {
        let boot_deadline = tokio::time::Instant::now() + BOOT_BUDGET;
        let response = loop {
            let attempt = tokio::time::timeout(OPEN_ATTEMPT, camera());
            match attempt.await {
                Ok(Ok(response)) => break response,
                Ok(Err(_)) | Err(_)
                    if tokio::time::Instant::now() + OPEN_ATTEMPT < boot_deadline =>
                {
                    tokio::time::sleep(Duration::from_secs(1)).await
                }
                Ok(Err(_)) => {
                    r.camera_end("unavailable");
                    return;
                }
                Err(_) => {
                    r.camera_end("timeout");
                    return;
                }
            }
        };
        let mut bytes = response.bytes_stream();
        let mut parser = Jpegs::default();
        let mut total = 0usize;
        let deadline = tokio::time::Instant::now() + MAX_DURATION;
        loop {
            let next = tokio::time::timeout_at(
                deadline.min(tokio::time::Instant::now() + Duration::from_secs(5)),
                bytes.next(),
            )
            .await;
            match next {
                Ok(Some(Ok(chunk))) => {
                    total = total.saturating_add(chunk.len());
                    if total > MAX_BYTES as usize {
                        r.camera_end("size_limit");
                        r.loss();
                        return;
                    }
                    let received = Instant::now();
                    let before = parser.dropped;
                    // Bound the output list too: a single HTTP chunk may contain
                    // many tiny JPEGs. Never accumulate a whole video chunk's frames.
                    for piece in chunk.chunks(65536) {
                        for jpeg in parser.feed(piece) {
                            r.jpeg(jpeg, received);
                        }
                    }
                    for _ in before..parser.dropped {
                        r.loss();
                    }
                }
                Ok(None) => {
                    r.camera_end("eof");
                    return;
                }
                Ok(Some(Err(_))) => {
                    r.camera_end("unavailable");
                    return;
                }
                Err(_) => {
                    r.camera_end("timeout");
                    return;
                }
            }
        }
    });
    Sources {
        tasks: vec![snapshots, capture],
        recording,
    }
}
fn bounded_snapshot(value: &Value, budget: &mut usize, depth: usize) -> Option<Value> {
    if depth > 32 {
        return None;
    }
    *budget = budget.checked_sub(32)?;
    Some(match value {
        Value::Object(fields) => {
            let mut out = serde_json::Map::new();
            for (key, value) in fields {
                if matches!(
                    key.as_str(),
                    "map" | "token" | "password" | "authorization" | "api_key" | "tts" | "text"
                ) {
                    continue;
                }
                *budget = budget.checked_sub(key.len().checked_mul(6)?.checked_add(4)?)?;
                out.insert(key.clone(), bounded_snapshot(value, budget, depth + 1)?);
            }
            Value::Object(out)
        }
        Value::Array(values) => Value::Array(
            values
                .iter()
                .map(|v| bounded_snapshot(v, budget, depth + 1))
                .collect::<Option<_>>()?,
        ),
        Value::String(text) => {
            *budget = budget.checked_sub(text.len().checked_mul(6)?)?;
            Value::String(text.clone())
        }
        _ => value.clone(),
    })
}
/// Marker-based extraction, bounded even with fragmented hostile MJPEG bodies.
#[derive(Default)]
pub struct Jpegs {
    buffer: Vec<u8>,
    pub dropped: u64,
}
impl Jpegs {
    pub fn feed(&mut self, bytes: &[u8]) -> Vec<Vec<u8>> {
        let mut frames = Vec::new();
        for &b in bytes {
            self.buffer.push(b);
            if self.buffer.len() == 2 && self.buffer != [0xff, 0xd8] {
                self.buffer.remove(0);
            }
            if self.buffer.len() > JPEG {
                self.buffer.clear();
                self.dropped += 1;
            } else if self.buffer.len() >= 4 && self.buffer.ends_with(&[0xff, 0xd9]) {
                frames.push(std::mem::take(&mut self.buffer));
            }
        }
        frames
    }
}
