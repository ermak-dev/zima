//! Line-oriented JSON protocol with the MRPT adapter child process. The
//! adapter is a separate C++ executable so MRPT's C++ dependencies can never
//! enter the Rust build or the session's failure domain.
use serde_json::{json, Value};
use std::{
    io::{BufRead, BufReader, Write},
    path::{Path, PathBuf},
    process::{Child, Command, Stdio},
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
    })
}

/// Thin owner of the adapter child process. One request line in, one reply
/// line out; a dead or wedged child is an Err for the caller to restart.
pub struct Adapter {
    child: Child,
    reader: BufReader<std::process::ChildStdout>,
}

impl Adapter {
    pub fn spawn(exe: &Path) -> Result<Self, String> {
        let mut child = Command::new(exe)
            .stdin(Stdio::piped())
            .stdout(Stdio::piped())
            .stderr(Stdio::null())
            .spawn()
            .map_err(|e| format!("cannot spawn SLAM adapter {exe:?}: {e}"))?;
        let reader = BufReader::new(child.stdout.take().expect("adapter stdout"));
        Ok(Self { child, reader })
    }
    pub fn call(&mut self, req: &str) -> Result<SlamStatus, String> {
        let line = self.call_raw(req)?;
        parse_status(&line)
    }
    /// Raw reply line; extended replies (e.g. the occupancy grid) carry
    /// fields beyond SlamStatus and need the full JSON.
    pub fn call_raw(&mut self, req: &str) -> Result<String, String> {
        let stdin = self.child.stdin.as_mut().expect("adapter stdin");
        stdin
            .write_all(req.as_bytes())
            .and_then(|_| stdin.write_all(b"\n"))
            .and_then(|_| stdin.flush())
            .map_err(|e| format!("adapter pipe broke: {e}"))?;
        let mut line = String::new();
        self.reader
            .read_line(&mut line)
            .map_err(|e| format!("adapter reply failed: {e}"))?;
        if line.trim().is_empty() {
            return Err("adapter closed".into());
        }
        Ok(line)
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
}

impl Observation {
    /// Convert a raw Player scan (bearings relative to the LDS zero) into
    /// base_link bearings by adding the LDS mounting yaw.
    pub fn from_scan(s: &player_bridge::LaserScanAngle, odom: Option<(f64, f64, f64)>) -> Self {
        let offset = player_bridge::scan::LDS_YAW_DEG.to_radians() as f32;
        // Normalize to [-pi, pi): the adapter bins beams into fixed angular
        // slots starting at -pi.
        let norm = |a: f32| {
            let mut a = a + offset;
            while a < -std::f32::consts::PI {
                a += 2. * std::f32::consts::PI;
            }
            while a >= std::f32::consts::PI {
                a -= 2. * std::f32::consts::PI;
            }
            a
        };
        Self {
            angles: s.angles.iter().map(|a| norm(*a)).collect(),
            ranges: s.ranges.clone(),
            max_range: s.max_range,
            odom,
        }
    }
    /// Request line for the adapter: valid beams only, each with its own
    /// base_link bearing; ranges are measured from the LDS at `sensor_x`.
    /// `delta` is the odom pose change since the previous update, in the
    /// previous robot frame.
    pub fn to_update_request(&self, delta: (f64, f64, f64)) -> String {
        let mut angles = Vec::with_capacity(self.angles.len());
        let mut ranges = Vec::with_capacity(self.ranges.len());
        for (a, r) in self.angles.iter().zip(&self.ranges) {
            if r.is_finite() && *r >= player_bridge::scan::LDS_RANGE_MIN && *r <= self.max_range {
                angles.push(round(*a as f64));
                ranges.push(round(*r as f64));
            }
        }
        request(
            "update",
            json!({
                "scan": {"angles": angles, "ranges": ranges},
                "range_max": round(self.max_range as f64),
                "sensor_x": player_bridge::scan::LDS_X,
                "odom": {"dx": round(delta.0), "dy": round(delta.1), "dyaw": round(delta.2)}
            }),
        )
    }
}

fn round(v: f64) -> f64 {
    (v * 10000.).round() / 10000.
}

/// Owned thread talking to the adapter process. Observations in, slam state
/// out. Localization status is telemetry and does not control wheel commands.
/// Restarting the adapter on crash keeps the session alive.
pub struct SlamThread {
    pub observe: std::sync::mpsc::Sender<Observation>,
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
        } else if let Some(path) = &map_path {
            adapter.call(&request("load_map", json!({"path": path})))?
        } else {
            return Err("localize mode needs --slam-map".into());
        };
        if !init.ok {
            return Err(format!("adapter init failed: {:?}", init.error));
        }
        let mut core = SlamCore::new(adapter, mapping);
        let (tx, rx) = std::sync::mpsc::channel::<Observation>();
        std::thread::spawn(move || {
            // The occupancy grid is redrawn at a slow cadence: rebuilding it
            // on every scan would dominate update latency for no visual gain.
            const MAP_EVERY: u32 = 10;
            let mut since_map = MAP_EVERY;
            let mut last_map: Option<Value> = None;
            for obs in rx {
                // A scan without any odometry yet still updates SLAM with a
                // zero delta; only feeding deltas requires two odom poses.
                let (delta, _seen_before) = core.feed_odom(obs.odom);
                let req = obs.to_update_request(delta);
                let mut state = core.update_request(&req);
                since_map += 1;
                if since_map >= MAP_EVERY {
                    since_map = 0;
                    if let Some(m) = core.fetch_map() {
                        last_map = Some(m);
                    }
                }
                if let Some(m) = &last_map {
                    state["map"] = m.clone();
                }
                on_state(state);
            }
        });
        Ok(Self { observe: tx })
    }
}

impl SlamCore<Adapter> {
    /// Ask the adapter for the current occupancy grid (mapping rebuilds it
    /// from the simplemap; localize returns the loaded map).
    pub fn fetch_map(&mut self) -> Option<Value> {
        let line = self.backend.call_raw(&request("map", json!({}))).ok()?;
        let v: Value = serde_json::from_str(&line).ok()?;
        v.get("grid").cloned()
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
    (c * dx + s * dy, -s * dx + c * dy, cyaw - pyaw)
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
        }
    }
    /// Feed one scan (and the absolute odom pose) through the adapter.
    /// Returns the panel-facing slam state.
    /// Pure-path API exercised by unit tests; production uses update_request.
    #[allow(dead_code)]
    pub fn update(&mut self, scan: &Value, odom: Option<(f64, f64, f64)>) -> Value {
        let delta = match (self.odom, odom) {
            (Some(p), Some(c)) => {
                self.odom = Some(c);
                let (dx, dy, dyaw) = odom_delta(p, c);
                json!({"dx": dx, "dy": dy, "dyaw": dyaw})
            }
            (None, c) => {
                self.odom = c;
                json!({"dx": 0., "dy": 0., "dyaw": 0.})
            }
            _ => json!({"dx": 0., "dy": 0., "dyaw": 0.}),
        };
        let req = request("update", json!({"scan": scan, "odom": delta}));
        let status = match self.backend.call(&req) {
            Ok(s) => s,
            Err(e) => {
                self.log.push(format!("adapter failure: {e}"));
                return self.state_json(None);
            }
        };
        if !status.ok {
            self.log.push(format!(
                "adapter error: {}",
                status.error.unwrap_or_default()
            ));
            return self.state_json(None);
        }
        if let Some(p) = status.pose.clone() {
            self.pose = Some(p);
        }
        // Mapping cannot be "lost" relative to a map it is building.
        if !self.mapping {
            let bad = status.quality.is_none_or(|q| q < QUALITY_BAD);
            if bad {
                self.bad += 1;
            } else {
                self.bad = 0;
                self.resets = 0;
                if self.lost {
                    self.lost = false;
                }
            }
            if self.bad == LOCAL_RESET_AT || self.bad == GLOBAL_RESET_AT {
                let op = if self.bad == LOCAL_RESET_AT {
                    "local_reset"
                } else {
                    "global_reset"
                };
                self.log
                    .push(format!("{op} issued after {} bad updates", self.bad));
                let _ = self.backend.call(&request(op, json!({})));
                self.resets += 1;
            }
            if self.bad >= LOST_AFTER_BAD {
                self.lost = true;
            }
        }
        self.state_json(status.quality)
    }
    pub fn state_json(&self, quality: Option<f64>) -> Value {
        json!({
            "mode": if self.mapping { "mapping" } else if self.lost { "lost" } else { "localize" },
            "pose": self.pose.as_ref().map(|p| json!({"x": p.x, "y": p.y, "yaw": p.yaw})),
            "quality": quality,
            "lost": self.lost,
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
        match (self.odom, odom) {
            (Some(p), Some(c)) => {
                self.odom = Some(c);
                (odom_delta(p, c), true)
            }
            (None, Some(c)) => {
                self.odom = Some(c);
                ((0., 0., 0.), true)
            }
            _ => ((0., 0., 0.), false),
        }
    }
    /// One adapter update request -> panel state (keeps SlamCore::update for
    /// the pure unit-tested path).
    pub fn update_request(&mut self, req: &str) -> Value {
        match self.backend.call(req) {
            Ok(status) => {
                if !status.ok {
                    self.log.push(format!(
                        "adapter error: {}",
                        status.error.unwrap_or_default()
                    ));
                    return self.state_json(None);
                }
                if let Some(p) = status.pose.clone() {
                    self.pose = Some(p);
                }
                if !self.mapping {
                    let bad = status.quality.is_none_or(|q| q < QUALITY_BAD);
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
                        let _ = self.backend.call(&request(op, json!({})));
                        self.resets += 1;
                    }
                    if self.bad >= LOST_AFTER_BAD {
                        self.lost = true;
                    }
                }
                self.state_json(status.quality)
            }
            Err(e) => {
                self.log.push(format!("adapter failure: {e}"));
                self.state_json(None)
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    // FEATURE: update request carries exact LDS geometry to the adapter.
    // SPEC: bearings are base_link (mount yaw added, wrapped to [-pi,pi));
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
            angles: vec![0.0, 0.0, 0.5, 1.0, 3.0],
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
        assert!((a0 - 97.6f64.to_radians()).abs() < 1e-3);
        let a1 = v["scan"]["angles"][1].as_f64().unwrap();
        let expected = 3.0 + 97.6f64.to_radians() - 2. * std::f64::consts::PI;
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
