//! Line-oriented JSON protocol with the MRPT adapter child process. The
//! adapter is a separate C++ executable so MRPT's C++ dependencies can never
//! enter the Rust build or the session's failure domain.
use serde_json::{json, Value};
use std::{
    collections::VecDeque,
    io::{Read, Write},
    path::{Path, PathBuf},
    process::{Child, Command, Stdio},
    sync::{Arc, Condvar, Mutex},
    time::{Duration, Instant},
};

/// One request line. Scan bearings are radians in the base_link frame, ranges
/// meters; odometry is the incremental pose delta since the previous update.
pub fn request(op: &str, extra: Value) -> String {
    let mut v = json!({"op": op});
    if let (Some(obj), Some(add)) = (v.as_object_mut(), extra.as_object()) {
        for (k, val) in add {
            obj.insert(k.clone(), val.clone());
        }
    }
    v.to_string()
}

#[derive(Debug, PartialEq, Clone)]
pub struct Pose {
    pub x: f64,
    pub y: f64,
    pub yaw: f64,
}

/// The localized/mapped verdict of one update plus the filter's confidence.
#[derive(Debug, PartialEq)]
pub struct SlamStatus {
    pub ok: bool,
    pub error: Option<String>,
    pub mode: Option<String>,
    pub pose: Option<Pose>,
    pub quality: Option<f64>,
    pub localization_quality: Option<f64>,
    pub tracking: Option<String>,
    pub diagnostics: Value,
}

pub fn parse_status(line: &str) -> Result<SlamStatus, String> {
    let v: Value =
        serde_json::from_str(line).map_err(|e| format!("adapter reply is not JSON: {e}"))?;
    let obj = v
        .as_object()
        .ok_or_else(|| "adapter reply is not an object".to_string())?;
    if !obj.get("ok").and_then(Value::as_bool).unwrap_or(false) {
        return Ok(SlamStatus {
            ok: false,
            error: Some(
                obj.get("error")
                    .and_then(Value::as_str)
                    .unwrap_or("unknown adapter error")
                    .to_string(),
            ),
            mode: None,
            pose: None,
            quality: None,
            localization_quality: None,
            tracking: Some("error".into()),
            diagnostics: obj.get("diagnostics").cloned().unwrap_or(Value::Null),
        });
    }
    let mode = obj.get("mode").and_then(Value::as_str).map(str::to_string);
    let quality = obj.get("quality").and_then(Value::as_f64);
    let pose = obj.get("pose").and_then(|p| {
        Some(Pose {
            x: p.get("x")?.as_f64()?,
            y: p.get("y")?.as_f64()?,
            yaw: p.get("yaw")?.as_f64()?,
        })
    });
    Ok(SlamStatus {
        ok: true,
        error: None,
        mode,
        pose,
        quality,
        localization_quality: obj.get("localization_quality").and_then(Value::as_f64),
        tracking: obj
            .get("tracking")
            .and_then(Value::as_str)
            .map(str::to_string),
        diagnostics: obj.get("diagnostics").cloned().unwrap_or(Value::Null),
    })
}

/// Bounded nonblocking pipe transport. A transport error poisons this process:
/// an update may have been applied, so it must never be retried implicitly.
pub struct Adapter {
    child: Child,
    reader: std::process::ChildStdout,
    timeout: Duration,
    limit: usize,
    poisoned: bool,
}
fn env_number(name: &str, default: f64, min: f64, max: f64) -> f64 {
    std::env::var(name)
        .ok()
        .and_then(|s| s.parse::<f64>().ok())
        .filter(|v| v.is_finite() && *v >= min && *v <= max)
        .unwrap_or(default)
}
pub fn source_time_verified() -> bool {
    std::env::var("ROBOT_SLAM_SOURCE_TIME_VERIFIED").is_ok_and(|v| v == "true")
}
fn max_age_ms() -> f64 {
    env_number("ROBOT_SLAM_MAX_POSE_AGE_MS", 500., 1., 60000.)
}
pub fn odom_max_skew_ms() -> f64 {
    env_number("ROBOT_SLAM_ODOM_MAX_SKEW_MS", 100., 1., 10000.)
}
fn sanitize(s: &str) -> String {
    s.chars().filter(|c| !c.is_control()).take(240).collect()
}
#[cfg(unix)]
fn nonblocking(fd: std::os::fd::RawFd) -> Result<(), String> {
    unsafe extern "C" {
        fn fcntl(fd: i32, cmd: i32, ...) -> i32;
    }
    // Linux F_GETFL/F_SETFL/O_NONBLOCK; server is Linux-only.
    let flags = unsafe { fcntl(fd, 3) };
    if flags < 0 || unsafe { fcntl(fd, 4, flags | 0o4000) } < 0 {
        return Err("cannot configure adapter pipe".into());
    }
    Ok(())
}
impl Adapter {
    pub fn spawn(exe: &Path) -> Result<Self, String> {
        Self::spawn_limits(
            exe,
            Duration::from_millis(
                env_number("ROBOT_SLAM_ADAPTER_TIMEOUT_MS", 1500., 1., 60000.) as u64,
            ),
            4 * 1024 * 1024,
        )
    }
    pub fn spawn_limits(exe: &Path, timeout: Duration, limit: usize) -> Result<Self, String> {
        use std::os::fd::AsRawFd;
        let mut child = Command::new(exe)
            .stdin(Stdio::piped())
            .stdout(Stdio::piped())
            .stderr(Stdio::null())
            .spawn()
            .map_err(|e| format!("cannot spawn SLAM adapter: {e}"))?;
        let reader = child.stdout.take().expect("adapter stdout");
        let mut adapter = Self {
            child,
            reader,
            timeout,
            limit,
            poisoned: false,
        };
        nonblocking(adapter.reader.as_raw_fd())?;
        nonblocking(adapter.child.stdin.as_mut().unwrap().as_raw_fd())?;
        Ok(adapter)
    }
    pub fn call(&mut self, req: &str) -> Result<SlamStatus, String> {
        let line = self.call_raw(req)?;
        match parse_status(&line) {
            Ok(status) => Ok(status),
            Err(e) => {
                self.poisoned = true;
                Err(e)
            }
        }
    }
    pub fn call_raw(&mut self, req: &str) -> Result<String, String> {
        if self.poisoned {
            return Err("adapter transport failed; explicit restart required".into());
        }
        let result = self.exchange(req);
        if result.is_err() {
            self.poisoned = true;
            let _ = self.child.kill();
        }
        result
    }
    fn exchange(&mut self, req: &str) -> Result<String, String> {
        let deadline = Instant::now() + self.timeout;
        let mut bytes = req.as_bytes().to_vec();
        bytes.push(b'\n');
        let mut offset = 0;
        while offset < bytes.len() {
            if Instant::now() >= deadline {
                return Err("adapter write deadline exceeded".into());
            }
            match self.child.stdin.as_mut().unwrap().write(&bytes[offset..]) {
                Ok(0) => return Err("adapter stdin closed".into()),
                Ok(n) => offset += n,
                Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => {
                    std::thread::sleep(Duration::from_millis(2))
                }
                Err(e) if e.kind() == std::io::ErrorKind::Interrupted => {}
                Err(_) => return Err("adapter write failed".into()),
            }
        }
        let mut reply = Vec::new();
        let mut buf = [0; 4096];
        loop {
            if Instant::now() >= deadline {
                return Err("adapter read deadline exceeded".into());
            }
            match self.reader.read(&mut buf) {
                Ok(0) => return Err("adapter closed before complete reply".into()),
                Ok(n) => {
                    if reply.len() + n > self.limit {
                        return Err("adapter reply exceeds size limit".into());
                    }
                    reply.extend_from_slice(&buf[..n]);
                    if let Some(end) = reply.iter().position(|b| *b == b'\n') {
                        if end + 1 != reply.len() {
                            return Err("adapter unsolicited reply".into());
                        }
                        return String::from_utf8(reply)
                            .map_err(|_| "adapter reply is not UTF-8".into());
                    }
                }
                Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => {
                    std::thread::sleep(Duration::from_millis(2))
                }
                Err(e) if e.kind() == std::io::ErrorKind::Interrupted => {}
                Err(_) => return Err("adapter read failed".into()),
            }
        }
    }
}

impl Drop for Adapter {
    fn drop(&mut self) {
        let _ = self.child.kill();
        let _ = self.child.wait();
    }
}

/// One observation ready for the adapter: scan in base_link (LDS mount
/// offset applied here, as the panel renderer does) plus absolute odom pose.
#[derive(Debug, Clone)]
pub struct Observation {
    pub angles: Vec<f32>,
    pub ranges: Vec<f32>,
    pub max_range: f32,
    pub odom: Option<(f64, f64, f64)>,
    pub scan_id: u32,
    pub sequence: u64,
    pub scan_stamp: Option<f64>,
    pub received: Instant,
    pub association: &'static str,
    pub odom_skew_ms: Option<f64>,
    pub source_time_verified: bool,
    /// Slot-model compensated scan (PLAYER_TIMING.md verified timing). The
    /// adapter request then honestly declares motion_compensated.
    pub deskewed: Option<(Vec<f32>, Vec<f32>, f64)>,
}

impl Observation {
    /// Preserve vendor body-aligned bearings and normalize their angular range.
    /// Ranges still originate at the LDS, not the center of base_link.
    pub fn from_scan(s: &player_bridge::LaserScanAngle, odom: Option<(f64, f64, f64)>) -> Self {
        let offset = player_bridge::scan::LDS_YAW_DEG.to_radians() as f32;
        // Normalize to [-pi, pi): the adapter bins beams into fixed angular
        // slots starting at -pi.
        let norm = |a: f32| {
            // Remainder is bounded even for very large finite angles; never loop.
            ((a as f64 + offset as f64 + std::f64::consts::PI).rem_euclid(std::f64::consts::TAU)
                - std::f64::consts::PI) as f32
        };
        Self {
            angles: s.angles.iter().map(|a| norm(*a)).collect(),
            ranges: s.ranges.clone(),
            max_range: s.max_range,
            odom,
            scan_id: s.id,
            sequence: 0,
            scan_stamp: None,
            received: Instant::now(),
            association: if odom.is_some() {
                "unverified"
            } else {
                "missing"
            },
            odom_skew_ms: None,
            source_time_verified: false,
            deskewed: None,
        }
    }
    /// Request line for the adapter: valid beams only, each with its own
    /// base_link bearing; ranges are measured from the LDS at `sensor_x`.
    /// `delta` is the odom pose change since the previous update, in the
    /// previous robot frame.
    pub fn to_update_request(&self, delta: (f64, f64, f64)) -> String {
        let (source_angles, source_ranges, compensated) = if let Some((a, r, _)) = &self.deskewed {
            (a, r, true)
        } else {
            (&self.angles, &self.ranges, false)
        };
        let mut angles = Vec::with_capacity(source_angles.len());
        let mut ranges = Vec::with_capacity(source_ranges.len());
        for (a, r) in source_angles.iter().zip(source_ranges) {
            if a.is_finite()
                && r.is_finite()
                && *r >= player_bridge::scan::LDS_RANGE_MIN
                && *r <= self.max_range
            {
                angles.push(round(*a as f64));
                ranges.push(round(*r as f64));
            }
        }
        let mut fields = json!({
            "scan_id": self.scan_id,
            "scan": {"angles": angles, "ranges": ranges},
            "range_max": round(self.max_range as f64),
            "sensor_x": player_bridge::scan::LDS_X,
            "odom": {"dx": round(delta.0), "dy": round(delta.1), "dyaw": round(delta.2)}
        });
        if compensated {
            // Honest declaration consumed by the adapter's admission gate.
            fields["motion_compensated"] = json!(true);
        }
        if self.source_time_verified {
            if let Some(stamp) = self.scan_stamp.filter(|s| s.is_finite()) {
                fields["stamp"] = json!(stamp);
            }
        }
        request("update", fields)
    }
}

fn round(v: f64) -> f64 {
    (v * 10000.).round() / 10000.
}

fn wrap(yaw: f64) -> f64 {
    (yaw + std::f64::consts::PI).rem_euclid(std::f64::consts::TAU) - std::f64::consts::PI
}
fn valid_pose(p: (f64, f64, f64)) -> bool {
    p.0.is_finite() && p.1.is_finite() && p.2.is_finite()
}
struct OdomSample {
    source: f64,
    received: Instant,
    pose: (f64, f64, f64),
}
pub struct Association {
    pub pose: Option<(f64, f64, f64)>,
    pub kind: &'static str,
    pub skew: Option<f64>,
}
pub struct OdomHistory {
    samples: VecDeque<OdomSample>,
}
impl OdomHistory {
    pub fn new() -> Self {
        Self {
            samples: VecDeque::new(),
        }
    }
    pub fn push(&mut self, source: f64, received: Instant, pose: (f64, f64, f64)) -> bool {
        if !valid_pose(pose) {
            return false;
        }
        if let Some(last) = self.samples.back() {
            if received < last.received
                || (source.is_finite() && source > 0. && last.source.is_finite() && last.source > 0. && source <= last.source)
                // Conservative per-sample discontinuities, not accumulated
                // displacement since the last backend acknowledgement.
                || (pose.0 - last.pose.0).hypot(pose.1 - last.pose.1) > 2.
                || wrap(wrap(pose.2) - wrap(last.pose.2)).abs() > 2.
            {
                return false;
            }
        }
        self.samples.push_back(OdomSample {
            source,
            received,
            pose,
        });
        while self.samples.len() > 256
            || self.samples.front().is_some_and(|s| {
                received.saturating_duration_since(s.received) > Duration::from_secs(5)
            })
        {
            self.samples.pop_front();
        }
        true
    }
    pub fn associate(
        &self,
        source: Option<f64>,
        received: Instant,
        verified: bool,
        max_skew: f64,
    ) -> Association {
        let missing = || Association {
            pose: None,
            kind: "missing",
            skew: None,
        };
        if !verified {
            let Some(last) = self.samples.iter().rev().find(|s| s.received <= received) else {
                return missing();
            };
            let skew = received
                .saturating_duration_since(last.received)
                .as_secs_f64()
                * 1000.;
            return if skew <= max_skew {
                Association {
                    pose: Some(last.pose),
                    kind: "unverified",
                    skew: Some(skew),
                }
            } else {
                missing()
            };
        }
        let Some(t) = source.filter(|s| s.is_finite() && *s > 0.) else {
            return missing();
        };
        let samples: Vec<_> = self
            .samples
            .iter()
            .filter(|s| {
                s.source.is_finite()
                    && s.source > 0.
                    && received.saturating_duration_since(s.received).as_secs_f64() * 1000.
                        <= max_age_ms()
            })
            .collect();
        for pair in samples.windows(2) {
            let (a, b) = (pair[0], pair[1]);
            if a.source <= t
                && t <= b.source
                && (t - a.source) * 1000. <= max_skew
                && (b.source - t) * 1000. <= max_skew
            {
                use robot_server::motion_compensation::{interpolate_pose, Pose2, TimedPose};
                let samples = [
                    TimedPose {
                        time: a.source,
                        pose: Pose2 {
                            x: a.pose.0,
                            y: a.pose.1,
                            yaw: a.pose.2,
                        },
                    },
                    TimedPose {
                        time: b.source,
                        pose: Pose2 {
                            x: b.pose.0,
                            y: b.pose.1,
                            yaw: b.pose.2,
                        },
                    },
                ];
                let Ok(pose) = interpolate_pose(&samples, t) else {
                    return missing();
                };
                return Association {
                    pose: Some((pose.x, pose.y, pose.yaw)),
                    kind: "interpolated",
                    skew: Some(((t - a.source).max(b.source - t)) * 1000.),
                };
            }
        }
        let Some(s) = samples
            .into_iter()
            .min_by(|a, b| (a.source - t).abs().total_cmp(&(b.source - t).abs()))
        else {
            return missing();
        };
        let skew = (s.source - t).abs() * 1000.;
        if skew <= max_skew {
            Association {
                pose: Some(s.pose),
                kind: "nearest",
                skew: Some(skew),
            }
        } else {
            missing()
        }
    }
}

/// Single pending latest observation, plus one in flight. Dropped observations
/// retain motion through their absolute odometry; there is no FIFO backlog.
struct Slot {
    pending: Option<Observation>,
    dropped: u64,
}
#[derive(Clone)]
pub struct ObservationSender {
    shared: Arc<(Mutex<Slot>, Condvar)>,
}
impl ObservationSender {
    pub fn send(&self, obs: Observation) -> Result<(), ()> {
        let (lock, wake) = &*self.shared;
        let mut slot = lock.lock().unwrap();
        if slot.pending.replace(obs).is_some() {
            slot.dropped += 1;
        }
        wake.notify_one();
        Ok(())
    }
}
// Sender clones are not used by production; Arc count determines closure.
impl Drop for ObservationSender {
    fn drop(&mut self) {
        self.shared.1.notify_one();
    }
}
pub struct SlamThread {
    pub observe: ObservationSender,
    pub maps: tokio::sync::watch::Receiver<Value>,
}
impl SlamThread {
    pub fn start(
        exe: PathBuf,
        mapping: bool,
        map_path: Option<String>,
        on_state: Box<dyn Fn(Value) + Send + 'static>,
    ) -> Result<Self, String> {
        let mut adapter = Adapter::spawn(&exe)?;
        let init = if mapping {
            adapter.call(&request("start_mapping", json!({})))?
        } else if let Some(path) = map_path {
            adapter.call(&request("load_map", json!({"path":path})))?
        } else {
            return Err("localize mode needs --slam-map".into());
        };
        if !init.ok {
            return Err("adapter init failed".into());
        }
        let shared = Arc::new((
            Mutex::new(Slot {
                pending: None,
                dropped: 0,
            }),
            Condvar::new(),
        ));
        let observe = ObservationSender {
            shared: shared.clone(),
        };
        // The grid leaves the frequent state channel: only changed versions
        // are published here, the state carries just the version counter.
        let (maps, maps_rx) = tokio::sync::watch::channel(Value::Null);
        let maps_thread = maps.clone();
        std::thread::spawn(move || {
            let mut core = SlamCore::new(adapter, mapping);
            let mut last: Option<(Observation, Value)> = None;
            let mut last_map: Option<Value> = None;
            let mut map_version: u64 = 0;
            let mut count = 0;
            loop {
                let (lock, wake) = &*shared;
                let mut slot = lock.lock().unwrap();
                if slot.pending.is_none() {
                    slot = wake
                        .wait_timeout(slot, Duration::from_millis(50))
                        .unwrap()
                        .0;
                }
                let obs = slot.pending.take();
                let dropped = slot.dropped;
                drop(slot);
                if let Some(obs) = obs {
                    let mut state = if obs.received.elapsed().as_secs_f64() * 1000. > max_age_ms() {
                        core.error = Some("observation expired before processing".into());
                        core.state_json(None)
                    } else if obs.odom.is_none() {
                        core.error = Some("missing or stale odometry".into());
                        core.state_json(None)
                    } else {
                        core.process_observation(&obs)
                    };
                    count += 1;
                    if count % 10 == 1 && !core.failed {
                        if let Some(map) = core.fetch_map() {
                            if last_map.as_ref() != Some(&map) {
                                map_version += 1;
                                let _ = maps_thread
                                    .send_replace(json!({"version": map_version, "grid": map}));
                            }
                            last_map = Some(map);
                        }
                        if core.failed {
                            state = core.state_json(None);
                        }
                    }
                    state["map_version"] = json!(map_version);
                    last = Some((obs, state));
                }
                if let Some((obs, state)) = &mut last {
                    let age = obs.received.elapsed().as_secs_f64() * 1000.;
                    state["scan_id"] = json!(obs.scan_id);
                    state["scan_sequence"] = json!(obs.sequence);
                    state["scan_stamp"] = json!(obs.scan_stamp);
                    state["age_ms"] = json!(age);
                    state["max_age_ms"] = json!(max_age_ms());
                    state["stale"] = json!(age > max_age_ms());
                    state["dropped_scans"] = json!(dropped);
                    state["odom_association"] = json!(obs.association);
                    state["odom_skew_ms"] = json!(obs.odom_skew_ms);
                    state["source_time_verified"] = json!(obs.source_time_verified);
                    let requested = deskew_enabled();
                    let applied = obs.deskewed.is_some();
                    state["deskew"] = json!({
                        "requested": requested, "applied": applied,
                        "status": if applied { "applied" }
                            else if requested && beam_time_verified() { "degraded" }
                            else if requested { "unverified_timing" }
                            else { "disabled" },
                        "reason": if requested && !applied && !beam_time_verified() {
                            Some("beam timing not verified; enable ROBOT_SLAM_BEAM_TIME_VERIFIED after the PLAYER_TIMING evidence")
                        } else if requested && !applied {
                            Some("missing stamp, unstable revolution period or uncovered trajectory")
                        } else { None },
                        "period_ms": obs.deskewed.as_ref().map(|d| d.2),
                    });
                    on_state(state.clone());
                }
                if Arc::strong_count(&shared) == 1 {
                    break;
                }
            }
        });
        Ok(Self {
            observe,
            maps: maps_rx,
        })
    }
}

impl SlamCore<Adapter> {
    /// Ask the adapter for its current occupancy grid. Export failures remain
    /// visible, and transport failure stops further requests to this child.
    pub fn fetch_map(&mut self) -> Option<Value> {
        let result = self
            .backend
            .call_raw(&request("map", json!({})))
            .and_then(|line| {
                serde_json::from_str::<Value>(&line)
                    .map_err(|_| "invalid adapter map reply".to_string())
            });
        match result {
            Ok(v) if v["ok"].as_bool() == Some(true) => v.get("grid").cloned(),
            Ok(_) => {
                self.error = Some("adapter map request rejected".into());
                None
            }
            Err(e) => {
                self.error = Some(sanitize(&e));
                self.failed = true;
                None
            }
        }
    }
}

/// What the adapter process must implement. Production uses `Adapter`;
/// tests script replies to drive the state machine.
pub trait Backend {
    fn call(&mut self, req: &str) -> Result<SlamStatus, String>;
}
impl Backend for Adapter {
    fn call(&mut self, req: &str) -> Result<SlamStatus, String> {
        Adapter::call(self, req)
    }
}

/// Odometry delta between absolute poses, expressed in the previous frame.
pub fn odom_delta(
    (px, py, pyaw): (f64, f64, f64),
    (cx, cy, cyaw): (f64, f64, f64),
) -> (f64, f64, f64) {
    let (dx, dy) = (cx - px, cy - py);
    let (s, c) = (pyaw.sin(), pyaw.cos());
    (
        c * dx + s * dy,
        -s * dx + c * dy,
        wrap(wrap(cyaw) - wrap(pyaw)),
    )
}

/// Pure SLAM orchestration: update calls, quality tracking, lost detection
/// and reset escalation. No I/O of its own; everything goes through Backend.
pub struct SlamCore<T: Backend> {
    backend: T,
    mapping: bool,
    pose: Option<Pose>,
    odom: Option<(f64, f64, f64)>,
    lost: bool,
    bad: u32,
    resets: u32,
    log: Vec<String>,
    tracking: Option<String>,
    localization_quality: Option<f64>,
    diagnostics: Value,
    error: Option<String>,
    failed: bool,
    acknowledged: bool,
}

pub const LOST_AFTER_BAD: u32 = 4;
pub const LOCAL_RESET_AT: u32 = 2;
pub const GLOBAL_RESET_AT: u32 = 6;
pub const QUALITY_BAD: f64 = 0.2;

impl<T: Backend> SlamCore<T> {
    pub fn new(backend: T, mapping: bool) -> Self {
        Self {
            backend,
            mapping,
            pose: None,
            odom: None,
            lost: false,
            bad: 0,
            resets: 0,
            log: Vec::new(),
            tracking: None,
            localization_quality: None,
            diagnostics: Value::Null,
            error: None,
            failed: false,
            acknowledged: false,
        }
    }
    /// Feed one scan (and the absolute odom pose) through the adapter.
    /// Returns the panel-facing slam state.
    /// Pure-path API exercised by unit tests; production uses update_request.
    #[allow(dead_code)]
    pub fn update(&mut self, scan: &Value, odom: Option<(f64, f64, f64)>) -> Value {
        let delta = match (self.odom, odom) {
            (Some(p), Some(c)) => odom_delta(p, c),
            _ => (0., 0., 0.),
        };
        let req = request(
            "update",
            json!({"scan":scan,"odom":{"dx":delta.0,"dy":delta.1,"dyaw":delta.2}}),
        );
        let state = self.update_request(&req);
        if self.acknowledged {
            self.odom = odom;
        }
        state
    }
    fn process_observation(&mut self, obs: &Observation) -> Value {
        if self.failed {
            return self.state_json(None);
        }
        if !obs.max_range.is_finite()
            || obs.max_range < player_bridge::scan::LDS_RANGE_MIN
            || obs.angles.len() != obs.ranges.len()
            || !obs.angles.iter().zip(&obs.ranges).any(|(a, r)| {
                a.is_finite()
                    && r.is_finite()
                    && *r >= player_bridge::scan::LDS_RANGE_MIN
                    && *r <= obs.max_range
            })
        {
            self.error = Some("invalid or empty laser scan".into());
            return self.state_json(None);
        }
        let (delta, usable) = self.feed_odom(obs.odom);
        if !usable {
            self.error = Some("invalid odometry or odometry jump".into());
            return self.state_json(None);
        }
        let state = self.update_request(&obs.to_update_request(delta));
        if self.acknowledged {
            self.odom = obs.odom;
        }
        state
    }
    pub fn state_json(&self, quality: Option<f64>) -> Value {
        json!({
            "mode": if self.mapping { "mapping" } else if self.lost { "lost" } else { "localize" },
            "pose": self.pose.as_ref().map(|p| json!({"x": p.x, "y": p.y, "yaw": p.yaw})),
            "quality": quality,
            "localization_quality": self.localization_quality,
            "lost": self.lost,
            "tracking": if self.error.is_some() { Some("error") } else { self.tracking.as_deref() },
            "diagnostics": self.diagnostics,
            "error": self.error.as_deref().map(sanitize),
        })
    }
    #[allow(dead_code)]
    pub fn save_map(&mut self, path: &str) -> Result<(), String> {
        let s = self
            .backend
            .call(&request("save_map", json!({"path": path})))?;
        if s.ok {
            Ok(())
        } else {
            Err(s.error.unwrap_or_else(|| "save_map failed".into()))
        }
    }
    /// Track absolute odom poses; emit the delta for this observation and
    /// whether it is usable at all (first pose yields a zero delta).
    pub fn feed_odom(&mut self, odom: Option<(f64, f64, f64)>) -> ((f64, f64, f64), bool) {
        let Some(c) = odom.filter(|p| valid_pose(*p)) else {
            return ((0., 0., 0.), false);
        };
        match self.odom {
            Some(p) => (odom_delta(p, c), true),
            None => ((0., 0., 0.), true),
        }
    }
    /// One adapter update request -> panel state (keeps SlamCore::update for
    /// the pure unit-tested path).
    pub fn update_request(&mut self, req: &str) -> Value {
        self.acknowledged = false;
        if self.failed {
            return self.state_json(None);
        }
        match self.backend.call(req) {
            Ok(status) => {
                if !status.ok {
                    self.error = Some(status.error.unwrap_or_else(|| "adapter error".into()));
                    return self.state_json(None);
                }
                self.acknowledged = true;
                self.error = None;
                self.tracking = status.tracking;
                self.localization_quality = status.localization_quality;
                self.diagnostics = status.diagnostics;
                if let Some(p) = status.pose.clone() {
                    self.pose = Some(p);
                }
                if !self.mapping {
                    let bad = status
                        .localization_quality
                        .or(status.quality)
                        .is_none_or(|q| q < QUALITY_BAD);
                    if bad {
                        self.bad += 1;
                    } else {
                        self.bad = 0;
                        self.resets = 0;
                        self.lost = false;
                    }
                    if self.bad == LOCAL_RESET_AT || self.bad == GLOBAL_RESET_AT {
                        let op = if self.bad == LOCAL_RESET_AT {
                            "local_reset"
                        } else {
                            "global_reset"
                        };
                        self.log
                            .push(format!("{op} issued after {} bad updates", self.bad));
                        match self.backend.call(&request(op, json!({}))) {
                            Err(e) => {
                                self.error = Some(sanitize(&e));
                                self.failed = true;
                            }
                            Ok(s) if !s.ok => {
                                self.error = Some(
                                    s.error.unwrap_or_else(|| "adapter reset rejected".into()),
                                );
                            }
                            _ => {}
                        }
                        self.resets += 1;
                    }
                    if self.bad >= LOST_AFTER_BAD {
                        self.lost = true;
                    }
                }
                self.state_json(status.quality)
            }
            Err(e) => {
                self.error = Some(sanitize(&e));
                self.failed = true;
                self.log.push(format!("adapter failure: {}", sanitize(&e)));
                self.state_json(None)
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    // FEATURE: update request carries exact LDS geometry to the adapter.
    // SPEC: vendor bearings are already body-aligned and wrapped to [-pi,pi);
    // beams inside the body (< LDS_RANGE_MIN) or invalid are dropped; the
    // LDS lever arm is sent so the adapter can place the ray origin.
    // IMPLEMENTATION: Observation::from_scan + to_update_request.
    // RESULT: IMPLEMENTED — body hits dropped, offset and bearings verified.
    // STATUS: DONE
    #[test]
    fn update_request_drops_body_hits_and_sends_lidar_offset() {
        let scan = player_bridge::LaserScanAngle {
            max_range: 6.0,
            ranges: vec![0.1, 1.0, f32::NAN, 7.0, 2.0],
            angles: vec![0.0, 0.0, 0.5, 1.0, 7.0],
            intensities: vec![],
            id: 0,
        };
        let req = Observation::from_scan(&scan, None).to_update_request((0., 0., 0.));
        let v: Value = serde_json::from_str(&req).unwrap();
        let ranges = v["scan"]["ranges"].as_array().unwrap();
        assert_eq!(ranges.len(), 2, "0.1 m body hit, NaN and >max must go");
        assert_eq!(v["scan"]["ranges"][0], 1.0);
        assert_eq!(v["sensor_x"], -0.1);
        let a0 = v["scan"]["angles"][0].as_f64().unwrap();
        assert!(a0.abs() < 1e-3, "vendor yaw must not be applied twice");
        let a1 = v["scan"]["angles"][1].as_f64().unwrap();
        let expected = 7.0 - 2. * std::f64::consts::PI;
        assert!(
            (a1 - expected).abs() < 1e-3,
            "bearing must wrap into [-pi,pi)"
        );
    }
    // FEATURE: line JSON protocol with the MRPT adapter child process.
    // SPEC: requests carry an op plus merged fields; replies parse into a
    // typed status; adapter failures are data, not transport crashes.
    // IMPLEMENTATION: serde_json object merge and tolerant field extraction.
    // RESULT: IMPLEMENTED — request shape and reply parsing verified.
    // STATUS: DONE
    #[test]
    fn request_merges_op_and_fields() {
        let line = request(
            "update",
            json!({"scan": {"angles": [0.0], "ranges": [1.5]}, "odom": {"dx": 0.1}}),
        );
        let v: Value = serde_json::from_str(&line).unwrap();
        assert_eq!(v["op"], "update");
        assert_eq!(v["scan"]["ranges"], json!([1.5]));
    }
    #[test]
    fn status_parses_pose_quality_and_mode() {
        let s = parse_status(
            r#"{"ok":true,"mode":"mapping","quality":0.83,"pose":{"x":1.5,"y":-0.2,"yaw":0.4}}"#,
        )
        .unwrap();
        assert!(s.ok);
        assert_eq!(s.mode.as_deref(), Some("mapping"));
        assert_eq!(s.quality, Some(0.83));
        assert_eq!(
            s.pose,
            Some(Pose {
                x: 1.5,
                y: -0.2,
                yaw: 0.4
            })
        );
    }
    #[test]
    fn adapter_error_is_reported_not_invented() {
        let s = parse_status(r#"{"ok":false,"error":"map not loaded"}"#).unwrap();
        assert!(!s.ok);
        assert_eq!(s.error.as_deref(), Some("map not loaded"));
        assert!(parse_status("not json").is_err());
        let missing = parse_status(r#"{"ok":false}"#).unwrap();
        assert_eq!(missing.error.as_deref(), Some("unknown adapter error"));
    }
    #[test]
    fn incomplete_success_keeps_optional_fields_none() {
        let s = parse_status(r#"{"ok":true}"#).unwrap();
        assert!(s.ok);
        assert_eq!(s.mode, None);
        assert_eq!(s.pose, None);
        assert_eq!(s.quality, None);
    }
    // FEATURE: adapter child process wrapper.
    // SPEC: one request line yields one parsed reply; adapter error objects
    // surface as ok=false; a dead adapter is an Err, never a hang or panic.
    // IMPLEMENTATION: blocking std pipes on a dedicated thread in production;
    // tests talk to a shell fake directly.
    // RESULT: IMPLEMENTED — happy path, adapter error and crash verified.
    // STATUS: DONE
    fn shell_fake(script: &str) -> PathBuf {
        let dir =
            std::env::temp_dir().join(format!("slam-fake-{}-{}", std::process::id(), script.len()));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("adapter");
        std::fs::write(&path, format!("#!/bin/sh\n{script}\n")).unwrap();
        #[cfg(unix)]
        {
            use std::os::unix::fs::PermissionsExt;
            std::fs::set_permissions(&path, std::fs::Permissions::from_mode(0o700)).unwrap();
        }
        path
    }
    #[test]
    fn adapter_roundtrip_error_and_crash() {
        let fake = shell_fake(
            "while read line; do case \"$line\" in *crash*) exit 3;; *load_map*) echo '{\"ok\":false,\"error\":\"map not loaded\"}';; *) echo '{\"ok\":true,\"mode\":\"mapping\",\"pose\":{\"x\":0,\"y\":0,\"yaw\":0}}';; esac; done",
        );
        let mut a = Adapter::spawn(&fake).unwrap();
        let ok = a.call(&request("update", json!({}))).unwrap();
        assert!(ok.ok);
        assert_eq!(ok.mode.as_deref(), Some("mapping"));
        let bad = a
            .call(&request("load_map", json!({"path": "/none"})))
            .unwrap();
        assert!(!bad.ok);
        assert_eq!(bad.error.as_deref(), Some("map not loaded"));
        assert!(a.call(&request("crash", json!({}))).is_err());
        // A wedged adapter must not hang forever: the dead pipe errors fast.
        assert!(a.call(&request("update", json!({}))).is_err());
        assert!(Adapter::spawn(Path::new("/nonexistent/adapter")).is_err());
    }
    // FEATURE: SLAM orchestration state machine over the adapter backend.
    // SPEC: good updates keep localize mode; sustained bad quality escalates
    // local then global reset, then lost; recovery clears it; mapping never
    // goes lost; adapter failures degrade without panic.
    // IMPLEMENTATION: SlamCore with scripted quality over a Vec of replies.
    // RESULT: IMPLEMENTED — escalation, recovery, mapping and failure paths
    // verified.
    // STATUS: DONE
    struct Scripted(std::vec::IntoIter<Result<SlamStatus, String>>);
    impl Backend for Scripted {
        fn call(&mut self, _req: &str) -> Result<SlamStatus, String> {
            self.0.next().unwrap_or_else(|| {
                Ok(SlamStatus {
                    ok: true,
                    error: None,
                    mode: None,
                    pose: Some(Pose {
                        x: 0.,
                        y: 0.,
                        yaw: 0.,
                    }),
                    quality: Some(0.9),
                    localization_quality: None,
                    tracking: None,
                    diagnostics: Value::Null,
                })
            })
        }
    }
    fn scripted(qualities: &[f64]) -> Scripted {
        Scripted(
            qualities
                .iter()
                .map(|&q| {
                    Ok(SlamStatus {
                        ok: true,
                        error: None,
                        mode: None,
                        pose: Some(Pose {
                            x: 0.,
                            y: 0.,
                            yaw: 0.,
                        }),
                        quality: Some(q),
                        localization_quality: None,
                        tracking: None,
                        diagnostics: Value::Null,
                    })
                })
                .collect::<Vec<_>>()
                .into_iter(),
        )
    }
    fn scan_value() -> Value {
        json!({"points": [], "range_max": 8.0, "frame": "base_link"})
    }
    #[test]
    fn good_updates_stay_localize_and_track_pose() {
        let mut core = SlamCore::new(scripted(&[0.9, 0.8]), false);
        let v = core.update(&scan_value(), Some((0., 0., 0.)));
        assert_eq!(v["mode"], "localize");
        assert_eq!(v["quality"], 0.9);
        let v = core.update(&scan_value(), Some((1., 0., 0.)));
        assert_eq!(v["pose"]["x"], 0.);
    }
    #[test]
    fn bad_quality_escalates_resets_then_lost_then_recovers() {
        let mut core = SlamCore::new(scripted(&[0.1; 12]), false);
        let mut modes = Vec::new();
        for i in 0..12 {
            let v = core.update(&scan_value(), None);
            modes.push(v["mode"].clone());
            let _ = i;
        }
        assert!(core.log.iter().any(|l| l.contains("local_reset")));
        assert!(core.log.iter().any(|l| l.contains("global_reset")));
        assert_eq!(modes[3], "lost");
        // recovery: adapter back to good quality
        let mut core2 = SlamCore::new(scripted(&[0.1, 0.1, 0.1, 0.1, 0.9]), false);
        core2.update(&scan_value(), None);
        core2.update(&scan_value(), None);
        core2.update(&scan_value(), None);
        core2.update(&scan_value(), None);
        let v = core2.update(&scan_value(), None);
        assert_eq!(v["mode"], "localize");
    }
    #[test]
    fn mcl_confidence_is_separate_from_icp_goodness_and_controls_localization_health() {
        struct Localized;
        impl Backend for Localized {
            fn call(&mut self, _: &str) -> Result<SlamStatus, String> {
                parse_status(
                    r#"{"ok":true,"pose":{"x":0,"y":0,"yaw":0},"quality":null,"localization_quality":0.95,"tracking":"tracking","diagnostics":{"accepted":false,"reason":"mcl_update","goodness":null}}"#,
                )
            }
        }
        let mut core = SlamCore::new(Localized, false);
        for _ in 0..8 {
            let state = core.update(&scan_value(), Some((0., 0., 0.)));
            assert_eq!(state["localization_quality"], 0.95);
            assert!(state["quality"].is_null());
            assert_eq!(state["mode"], "localize");
        }
        assert!(
            core.log.is_empty(),
            "healthy MCL must not trigger particle resets"
        );
    }

    #[test]
    fn mapping_mode_never_goes_lost() {
        let mut core = SlamCore::new(scripted(&[0.0; 10]), true);
        for _ in 0..10 {
            let v = core.update(&scan_value(), None);
            assert_eq!(v["mode"], "mapping");
        }
    }
    #[test]
    fn adapter_failure_degrades_to_lost_pose_but_no_panic() {
        struct Dead;
        impl Backend for Dead {
            fn call(&mut self, _req: &str) -> Result<SlamStatus, String> {
                Err("child crashed".into())
            }
        }
        let mut core = SlamCore::new(Dead, false);
        let v = core.update(&scan_value(), None);
        assert!(v["pose"].is_null());
        assert!(!core.log.is_empty());
    }
}

/// Apply the verified slot-model deskew to one observation. Requires a
/// finite publication stamp, a stable measured revolution period from the
/// last stamps, and an odometry trajectory covering the whole revolution on
/// the verified source clock. Errors keep the caller on the raw scan path;
/// compensation is never silently approximated.
pub fn apply_deskew(
    obs: &Observation,
    history: &OdomHistory,
    recent_stamps: &[f64],
) -> Result<DeskewedScan, &'static str> {
    let stamp = obs
        .scan_stamp
        .filter(|s| s.is_finite() && *s > 0.)
        .ok_or("missing_scan_stamp")?;
    let period_ms = revolution_period_ms(recent_stamps).ok_or("unstable_revolution_period")?;
    let period = period_ms / 1000.;
    let from = stamp - period;
    let trajectory = history
        .trajectory_between(from, stamp)
        .ok_or("trajectory_does_not_cover_revolution")?;
    // Slot timing needs the ORIGINAL revolution length: invalid beams keep
    // their slot position, so valid beams keep exact acquisition times. A
    // beam measured before the first odometry sample (at most one frame step
    // into the revolution) is skipped rather than extrapolated.
    let n = obs.angles.len();
    let first_time = trajectory.first().map(|t| t.time).unwrap_or(f64::INFINITY);
    let mut angles = Vec::new();
    let mut ranges = Vec::new();
    for (slot, (a, r)) in obs.angles.iter().zip(&obs.ranges).enumerate() {
        if !(a.is_finite()
            && r.is_finite()
            && *r >= player_bridge::scan::LDS_RANGE_MIN
            && *r <= obs.max_range)
        {
            continue;
        }
        let time = stamp - (n - 1 - slot) as f64 * period / n as f64;
        if time < first_time {
            continue;
        }
        match robot_server::motion_compensation::deskew(
            &[robot_server::motion_compensation::TimedRay {
                time,
                angle: *a as f64,
                range: *r as f64,
            }],
            &trajectory,
            stamp,
            player_bridge::scan::LDS_X,
        ) {
            Ok(corrected) => {
                angles.push(corrected[0].angle as f32);
                ranges.push(corrected[0].range as f32);
            }
            Err(_) => return Err("deskew_kernel_rejected"),
        }
    }
    if angles.is_empty() {
        return Err("no_valid_beams");
    }
    Ok(DeskewedScan {
        angles,
        ranges,
        period_ms,
        stamp,
    })
}

pub struct DeskewedScan {
    pub angles: Vec<f32>,
    pub ranges: Vec<f32>,
    pub period_ms: f64,
    pub stamp: f64,
}

#[cfg(test)]
mod runtime_tests {
    use super::*;
    // FEATURE: bounded, honest SLAM timing and transport.
    // SPEC: preserve scan identity, interpolate verified clocks only, wrap yaw,
    // retain rejected motion and fail bounded child I/O without retries.
    // IMPLEMENTATION: bounded history/latest slot, nonblocking deadline pipes,
    // acknowledgement-based absolute odometry and preserved adapter metadata.
    // RESULT: IMPLEMENTED — local fakes cover timing, rejection and transport.
    // STATUS: DONE
    #[test]
    fn runtime_tracking_and_wrapped_delta() {
        let status = parse_status(
            r#"{"ok":true,"tracking":"degraded","diagnostics":{"accepted":false,"reason":"weak"}}"#,
        )
        .unwrap();
        assert_eq!(status.tracking.as_deref(), Some("degraded"));
        assert_eq!(status.diagnostics["accepted"], false);
        assert!((odom_delta((0., 0., 3.13), (0., 0., -3.13)).2 - 0.023185307).abs() < 1e-6);
    }
    #[test]
    fn runtime_history_verified_interpolation_and_unverified_default() {
        let now = std::time::Instant::now();
        let mut h = OdomHistory::new();
        h.push(1., now, (0., 0., 3.13));
        h.push(2., now, (0.2, 0., -3.13));
        let a = h.associate(Some(1.5), now, true, 600.);
        assert_eq!(a.kind, "interpolated");
        assert!((a.pose.unwrap().0 - 0.1).abs() < 1e-9);
        assert_eq!(h.associate(Some(1.5), now, false, 600.).kind, "unverified");
        assert!(h.associate(Some(9.), now, true, 100.).pose.is_none());
        assert!(!h.push(1.9, now, (0., 0., 0.)));
        assert!(!h.push(3., now, (f64::NAN, 0., 0.)));
        assert!(OdomHistory::new()
            .associate(None, now, false, 100.)
            .pose
            .is_none());
    }
    #[test]
    fn repeated_absent_source_times_do_not_discard_fresh_receive_order_odometry() {
        let now = Instant::now();
        let mut history = OdomHistory::new();
        assert!(history.push(0., now, (0., 0., 0.)));
        assert!(history.push(0., now + Duration::from_millis(20), (0.01, 0., 0.)));
        let sample = history.associate(None, now + Duration::from_millis(25), false, 100.);
        assert_eq!(sample.pose, Some((0.01, 0., 0.)));
        assert_eq!(sample.kind, "unverified");
        assert!(history
            .associate(Some(0.), now + Duration::from_millis(25), true, 100.)
            .pose
            .is_none());
    }

    #[test]
    fn verified_history_interpolates_constant_twist_on_an_arc() {
        let now = Instant::now();
        let mut history = OdomHistory::new();
        assert!(history.push(1., now, (0., 0., 0.)));
        assert!(history.push(2., now, (1., 1., std::f64::consts::FRAC_PI_2)));
        let association = history.associate(Some(1.5), now, true, 600.);
        assert_eq!(association.kind, "interpolated");
        let pose = association.pose.unwrap();
        assert!((pose.0 - std::f64::consts::FRAC_1_SQRT_2).abs() < 1e-8);
        assert!((pose.1 - (1. - std::f64::consts::FRAC_1_SQRT_2)).abs() < 1e-8);
    }

    // FEATURE: verified slot-model deskew applied to Player observations.
    // SPEC: beam times come from the measured revolution period and slot
    // indices; requires verified source clocks and an odometry trajectory
    // covering the whole scan; success is reported honestly, and any gap
    // keeps the raw scan without claiming compensation.
    #[test]
    fn revolution_period_is_measured_and_gated() {
        assert!((revolution_period_ms(&[100., 100.2, 100.4, 100.6]).unwrap() - 200.).abs() < 1e-6);
        assert!(revolution_period_ms(&[100., 100.5, 100.4]).is_none()); // non-monotonic
        assert!(revolution_period_ms(&[100., 100.01, 100.02]).is_none()); // too fast
        assert!(revolution_period_ms(&[100., 100., 100.]).is_none()); // zero
        assert!(revolution_period_ms(&[100., 100.2, 100.9]).is_none()); // jitter
        assert!(revolution_period_ms(&[100., 100.2]).is_none()); // too few
    }

    #[test]
    fn trajectory_between_covers_only_verified_source_bounds() {
        let now = Instant::now();
        let mut history = OdomHistory::new();
        for i in 0..=10 {
            assert!(history.push(
                100. + i as f64 * 0.04,
                now + Duration::from_millis(40 * i),
                (0., 0., 0.1 * i as f64)
            ));
        }
        let trajectory = history.trajectory_between(100.0, 100.4).unwrap();
        assert_eq!(trajectory.len(), 11);
        assert!((trajectory[0].time - 100.).abs() < 1e-9);
        assert!((trajectory[10].pose.yaw - 1.0).abs() < 1e-9);
        assert!(history.trajectory_between(99.9, 100.4).is_none());
        assert!(history.trajectory_between(100.0, 100.5).is_none());
        let empty = OdomHistory::new();
        assert!(empty.trajectory_between(100., 100.1).is_none());
    }

    #[test]
    fn deskew_apply_uses_slot_model_with_measured_period() {
        let period = 0.2;
        let n = 360;
        let w = 1.0f64;
        let stamp = 100.4;
        let t0 = stamp - 2. * period;
        let (wx, wy) = (2.0f64, 0.5);
        let rays: Vec<robot_server::motion_compensation::TimedRay> = (0..n)
            .map(|slot| {
                let time = stamp - (n - 1 - slot) as f64 * period / n as f64;
                let yaw = w * (time - t0);
                let bx = yaw.cos() * wx + yaw.sin() * wy;
                let by = -yaw.sin() * wx + yaw.cos() * wy;
                robot_server::motion_compensation::TimedRay {
                    time,
                    angle: by.atan2(bx + 0.1),
                    range: (bx + 0.1).hypot(by),
                }
            })
            .collect();
        let mut angles = vec![0f32; n];
        let mut ranges = vec![0f32; n];
        for (slot, ray) in rays.iter().enumerate() {
            angles[slot] = ray.angle as f32;
            ranges[slot] = ray.range as f32;
        }
        let mut observation = Observation::from_scan(
            &player_bridge::LaserScanAngle {
                max_range: 6.,
                ranges,
                angles,
                intensities: vec![],
                id: 1,
            },
            None,
        );
        observation.scan_stamp = Some(stamp);
        let mut history = OdomHistory::new();
        let now = Instant::now();
        for i in 0..=30 {
            assert!(history.push(
                t0 + i as f64 * 0.02,
                now + Duration::from_millis(20 * i),
                (0., 0., w * 0.02 * i as f64)
            ));
        }
        let applied = apply_deskew(&observation, &history, &stamp_positions(stamp, 3));
        let result = applied.expect("deskew must apply with full coverage");
        assert_eq!(result.angles.len(), n);
        // The compensated scan must reproduce the fixed world point from the reference pose.
        let ref_yaw = w * (stamp - t0);
        for (a, r) in result.angles.iter().zip(&result.ranges) {
            let x = player_bridge::scan::LDS_X + *r as f64 * (*a as f64).cos();
            let y = *r as f64 * (*a as f64).sin();
            let mx = ref_yaw.cos() * x - ref_yaw.sin() * y;
            let my = ref_yaw.sin() * x + ref_yaw.cos() * y;
            assert!((mx - wx).abs() < 1e-3, "x {mx} vs {wx}");
            assert!((my - wy).abs() < 1e-3, "y {my} vs {wy}");
        }
    }

    #[test]
    fn deskew_declines_without_coverage_or_stamps() {
        let observation = Observation::from_scan(
            &player_bridge::LaserScanAngle {
                max_range: 6.,
                ranges: vec![1.; 4],
                angles: vec![0.1, 0.2, 0.3, 0.4],
                intensities: vec![],
                id: 1,
            },
            None,
        );
        let empty = OdomHistory::new();
        assert!(apply_deskew(&observation, &empty, &[]).is_err());
        let mut history = OdomHistory::new();
        let now = Instant::now();
        for i in 0..=10 {
            assert!(history.push(100. + i as f64 * 0.05, now, (0., 0., 0.)));
        }
        let no_stamp = Observation {
            scan_stamp: None,
            ..observation.clone()
        };
        assert!(apply_deskew(&no_stamp, &history, &[100., 100.2, 100.4]).is_err());
    }

    fn stamp_positions(latest: f64, count: usize) -> Vec<f64> {
        let mut stamps: Vec<f64> = (0..count).map(|i| latest - i as f64 * 0.2).collect();
        stamps.reverse();
        stamps
    }

    #[test]
    fn runtime_latest_queue_motion_rejections_and_ambiguous_failure() {
        struct Recording {
            requests: Arc<Mutex<Vec<Value>>>,
            n: usize,
        }
        impl Backend for Recording {
            fn call(&mut self, req: &str) -> Result<SlamStatus, String> {
                self.requests
                    .lock()
                    .unwrap()
                    .push(serde_json::from_str(req).unwrap());
                self.n += 1;
                parse_status(match self.n {
                    2 => r#"{"ok":false,"error":"rejected\ninput"}"#,
                    4 => return Err("ambiguous transport failure".into()),
                    _ => {
                        r#"{"ok":true,"tracking":"degraded","diagnostics":{"accepted":false},"pose":{"x":0,"y":0,"yaw":0}}"#
                    }
                })
            }
        }
        let requests = Arc::new(Mutex::new(Vec::new()));
        let mut core = SlamCore::new(
            Recording {
                requests: requests.clone(),
                n: 0,
            },
            true,
        );
        let scan = player_bridge::LaserScanAngle {
            angles: vec![0.],
            ranges: vec![1.],
            max_range: 6.,
            id: 17,
            intensities: vec![],
        };
        let mut obs = Observation::from_scan(&scan, Some((0., 0., 0.)));
        obs.scan_stamp = Some(42.);
        let first = core.process_observation(&obs);
        assert_eq!(first["diagnostics"]["accepted"], false);
        obs.odom = Some((0.1, 0., 0.));
        let rejected = core.process_observation(&obs);
        assert_eq!(rejected["tracking"], "error");
        assert!(!rejected["error"].as_str().unwrap().contains('\n'));
        let shared = Arc::new((
            Mutex::new(Slot {
                pending: None,
                dropped: 0,
            }),
            Condvar::new(),
        ));
        let tx = ObservationSender {
            shared: shared.clone(),
        };
        tx.send(obs.clone()).unwrap();
        obs.odom = Some((0.3, 0., 0.));
        tx.send(obs.clone()).unwrap();
        let latest = shared.0.lock().unwrap().pending.take().unwrap();
        assert_eq!(shared.0.lock().unwrap().dropped, 1);
        core.process_observation(&latest);
        assert_eq!(requests.lock().unwrap()[2]["odom"]["dx"], 0.3);
        assert!(requests.lock().unwrap()[2].get("stamp").is_none());
        obs.odom = Some((0.4, 0., 0.));
        obs.source_time_verified = true;
        core.process_observation(&obs);
        assert_eq!(requests.lock().unwrap()[3]["stamp"], 42.);
        core.process_observation(&obs);
        assert_eq!(
            requests.lock().unwrap().len(),
            4,
            "ambiguous updates must not be retried"
        );
        let mut h = OdomHistory::new();
        let now = Instant::now();
        h.push(1., now, (0., 0., 0.));
        assert!(h
            .associate(Some(1.), now + Duration::from_secs(1), false, 100.)
            .pose
            .is_none());
        assert!(!h.push(2., now, (10., 0., 0.)));
        let huge = player_bridge::LaserScanAngle {
            angles: vec![f32::MAX, f32::INFINITY, f32::NAN],
            ranges: vec![1.; 3],
            max_range: 6.,
            id: 9,
            intensities: vec![],
        };
        let v: Value = serde_json::from_str(
            &Observation::from_scan(&huge, None).to_update_request((0., 0., 0.)),
        )
        .unwrap();
        assert_eq!(v["scan"]["angles"].as_array().unwrap().len(), 1);
        assert_eq!(v["scan_id"], 9);
    }
    #[test]
    fn runtime_invalid_geometry_and_reset_transport_failure_are_explicit() {
        struct Never;
        impl Backend for Never {
            fn call(&mut self, _: &str) -> Result<SlamStatus, String> {
                panic!("invalid observations cannot reach backend")
            }
        }
        let mut core = SlamCore::new(Never, true);
        let scan = player_bridge::LaserScanAngle {
            angles: vec![0.],
            ranges: vec![1.],
            max_range: f32::NAN,
            id: 9,
            intensities: vec![],
        };
        let obs = Observation::from_scan(&scan, Some((0., 0., 0.)));
        assert_eq!(core.process_observation(&obs)["tracking"], "error");
        struct ResetFailure;
        impl Backend for ResetFailure {
            fn call(&mut self, req: &str) -> Result<SlamStatus, String> {
                if req.contains("reset") {
                    Err("reset transport failure".into())
                } else {
                    parse_status(r#"{"ok":true,"quality":0.0}"#)
                }
            }
        }
        let mut core = SlamCore::new(ResetFailure, false);
        core.update_request("{}");
        assert_eq!(core.update_request("{}")["tracking"], "error");
        assert!(core.failed);
    }
    #[test]
    fn runtime_accumulated_motion_is_not_a_sample_jump() {
        struct NoCall;
        impl Backend for NoCall {
            fn call(&mut self, _: &str) -> Result<SlamStatus, String> {
                unreachable!()
            }
        }
        let mut core = SlamCore::new(NoCall, true);
        core.odom = Some((0., 0., 0.));
        assert_eq!(
            core.feed_odom(Some((3., 0., 0.))),
            ((3., 0., 0.), true),
            "coalescing may accumulate many individually valid odometry samples"
        );
        let mut h = OdomHistory::new();
        let now = Instant::now();
        h.push(1., now, (0., 0., 0.));
        assert!(
            !h.push(2., now, (0., 0., 3.)),
            "single-sample yaw discontinuity must be rejected"
        );
    }
    #[test]
    fn runtime_acknowledgement_survives_rejected_auxiliary_reset() {
        struct ResetReject {
            updates: Arc<Mutex<Vec<Value>>>,
        }
        impl Backend for ResetReject {
            fn call(&mut self, req: &str) -> Result<SlamStatus, String> {
                if req.contains("reset") {
                    return parse_status(r#"{"ok":false,"error":"reset unsupported"}"#);
                }
                self.updates
                    .lock()
                    .unwrap()
                    .push(serde_json::from_str(req).unwrap());
                parse_status(r#"{"ok":true,"quality":0.0}"#)
            }
        }
        let updates = Arc::new(Mutex::new(Vec::new()));
        let mut core = SlamCore::new(
            ResetReject {
                updates: updates.clone(),
            },
            false,
        );
        let scan = player_bridge::LaserScanAngle {
            angles: vec![0.],
            ranges: vec![1.],
            max_range: 6.,
            id: 1,
            intensities: vec![],
        };
        for x in [0., 0.1, 0.2] {
            core.process_observation(&Observation::from_scan(&scan, Some((x, 0., 0.))));
        }
        assert_eq!(
            updates.lock().unwrap()[2]["odom"]["dx"],
            0.1,
            "successful update was acknowledged even though following reset failed"
        );
        assert!(odom_delta((0., 0., f64::MAX), (0., 0., -f64::MAX))
            .2
            .is_finite());
    }
    #[test]
    fn runtime_transport_hung_oversize_eof_and_slow() {
        use std::os::unix::fs::PermissionsExt;
        for (i, script) in [
            "read line; sleep 2",
            "read line; yes x | tr -d '\\n'",
            "exit 0",
            "read line; sleep 0.02; printf '{\"ok\":true}\\n'",
        ]
        .iter()
        .enumerate()
        {
            let p = std::env::temp_dir().join(format!("slam-deadline-{}-{i}", std::process::id()));
            std::fs::write(&p, format!("#!/bin/sh\n{script}\n")).unwrap();
            std::fs::set_permissions(&p, std::fs::Permissions::from_mode(0o700)).unwrap();
            let mut a =
                Adapter::spawn_limits(&p, std::time::Duration::from_millis(100), 1024).unwrap();
            let t = std::time::Instant::now();
            let r = a.call("{}");
            assert_eq!(r.is_ok(), i == 3, "{i}: {r:?}");
            assert!(t.elapsed() < std::time::Duration::from_secs(1));
            if i != 3 {
                assert!(a.call("{}").is_err());
            }
            std::fs::remove_file(p).unwrap();
        }
        let p = std::env::temp_dir().join(format!("slam-write-deadline-{}", std::process::id()));
        std::fs::write(&p, "#!/bin/sh\nsleep 2\n").unwrap();
        std::fs::set_permissions(&p, std::fs::Permissions::from_mode(0o700)).unwrap();
        let mut adapter = Adapter::spawn_limits(&p, Duration::from_millis(80), 1024).unwrap();
        let started = Instant::now();
        let error = adapter.call_raw(&"x".repeat(1024 * 1024)).unwrap_err();
        assert!(error.contains("write deadline"));
        assert!(started.elapsed() < Duration::from_secs(1));
        std::fs::remove_file(p).unwrap();
    }
}

// Verified physical beam timing (PLAYER_TIMING.md, 2026-10-29): slots are
// acquired in order, the revolution spans the measured publication period
// and the scan is published right after the last beam. The revolution
// period is measured live from consecutive scan stamps, never assumed.
pub fn beam_time_verified() -> bool {
    std::env::var("ROBOT_SLAM_BEAM_TIME_VERIFIED").is_ok_and(|v| v == "true")
}
pub fn deskew_enabled() -> bool {
    std::env::var("ROBOT_SLAM_DESKEW_ENABLED").is_ok_and(|v| v == "true")
}
pub fn revolution_period_ms(stamps: &[f64]) -> Option<f64> {
    if stamps.len() < 3 {
        return None;
    }
    let deltas: Vec<f64> = stamps.windows(2).map(|w| (w[1] - w[0]) * 1000.).collect();
    let n = deltas.len() as f64;
    let mean = deltas.iter().sum::<f64>() / n;
    let variance = deltas.iter().map(|d| (d - mean).powi(2)).sum::<f64>() / n;
    // Reject unstable periods: jitter beyond 15 ms means the revolution
    // timing is not observable from these stamps.
    if !mean.is_finite() || !(150. ..=400.).contains(&mean) || variance.sqrt() > 15. {
        return None;
    }
    Some(mean)
}

impl OdomHistory {
    /// Trajectory samples covering [from, to] on the verified source clock.
    /// Both bounds must lie inside the sampled odometry interval; `to` may
    /// extend one sample step past the last sample because the scan is
    /// published right after the final beam, before the next odometry frame.
    pub fn trajectory_between(
        &self,
        from: f64,
        to: f64,
    ) -> Option<Vec<robot_server::motion_compensation::TimedPose>> {
        if !(from.is_finite() && to.is_finite()) || to < from {
            return None;
        }
        let samples: Vec<_> = self
            .samples
            .iter()
            .filter(|s| s.source.is_finite() && s.source > 0.)
            .collect();
        if samples.len() < 2 {
            return None;
        }
        let first = samples.first()?.source;
        let last = samples.last()?.source;
        let step = samples
            .windows(2)
            .map(|w| w[1].source - w[0].source)
            .fold(f64::MIN, f64::max);
        if from < first - 2. * step.max(0.) || to > last + 2. * step.max(0.) {
            return None;
        }
        let mut trajectory: Vec<robot_server::motion_compensation::TimedPose> = samples
            .iter()
            .filter(|s| s.source >= from && s.source <= to)
            .map(|s| robot_server::motion_compensation::TimedPose {
                time: s.source,
                pose: robot_server::motion_compensation::Pose2 {
                    x: s.pose.0,
                    y: s.pose.1,
                    yaw: s.pose.2,
                },
            })
            .collect();
        if to > last && trajectory.len() >= 2 {
            // The scan is published between odometry frames: continue the
            // constant twist of the last two samples by that sub-step so the
            // kernel has an exact sample at the reference instant. Bounded by
            // the caller's coverage check above (at most one frame step).
            let a = trajectory[trajectory.len() - 2];
            let b = trajectory[trajectory.len() - 1];
            let span = b.time - a.time;
            if span > 0. {
                let f = (to - b.time) / span;
                let (ca, sa) = a.pose.yaw.sin_cos();
                let dx = ca * (b.pose.x - a.pose.x) + sa * (b.pose.y - a.pose.y);
                let dy = -sa * (b.pose.x - a.pose.x) + ca * (b.pose.y - a.pose.y);
                let yaw = wrap(b.pose.yaw - a.pose.yaw);
                let fr = f * yaw;
                let (uf, vf) = if fr.abs() < 1e-7 {
                    (1. - fr * fr / 6., fr / 2. - fr.powi(3) / 24.)
                } else {
                    (fr.sin() / fr, (1. - fr.cos()) / fr)
                };
                let (cb, sb) = b.pose.yaw.sin_cos();
                let ex = f * (uf * dx - vf * dy);
                let ey = f * (vf * dx + uf * dy);
                trajectory.push(robot_server::motion_compensation::TimedPose {
                    time: to,
                    pose: robot_server::motion_compensation::Pose2 {
                        x: b.pose.x + cb * ex - sb * ey,
                        y: b.pose.y + sb * ex + cb * ey,
                        yaw: wrap(b.pose.yaw + fr),
                    },
                });
            }
        }
        Some(trajectory)
    }
}
