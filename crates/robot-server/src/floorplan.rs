//! Polygonal floorplan layer (#83): runs the offline floorplan tool
//! (`tools/floorplan/floorplan.py --json`) over the current SLAM grid and
//! publishes the versioned result to every panel through a watch channel.
//!
//! Honesty rules: no grid is an explicit error; a failing, missing or timed
//! out tool keeps the last successful layer; rebuilds are rate-limited. The
//! tool path comes from `ROBOT_FLOORPLAN_TOOL` (tests) or the repository
//! default; the process runs with `-I` and a 10 s deadline.
use crate::Failure;
use serde_json::{json, Value};
use std::sync::Mutex;
use std::time::{Duration, Instant};
use tokio::io::AsyncWriteExt;
use tokio::process::Command;
use tokio::sync::watch;

const REBUILD_INTERVAL: Duration = Duration::from_secs(5);
const TOOL_TIMEOUT: Duration = Duration::from_secs(10);

/// Shared rebuild state: last success version, its grid version and the
/// last rebuild attempt time (for the rate limit).
#[derive(Default)]
pub struct Floorplans {
    inner: Mutex<State>,
    pub frames: watch::Sender<Value>,
}

#[derive(Default)]
struct State {
    version: u64,
    grid_version: Option<u64>,
    last_attempt: Option<Instant>,
}

impl Floorplans {
    pub fn new() -> (Self, watch::Receiver<Value>) {
        let (frames, rx) = watch::channel(Value::Null);
        (
            Self {
                inner: Mutex::new(State::default()),
                frames,
            },
            rx,
        )
    }

    fn default_tool() -> Option<std::path::PathBuf> {
        if let Ok(path) = std::env::var("ROBOT_FLOORPLAN_TOOL") {
            return Some(path.into());
        }
        // The binary lives in crates/target; the repository root is three
        // levels up from the manifest directory (crates/robot-server).
        let manifest = std::path::Path::new(env!("CARGO_MANIFEST_DIR"));
        let root = manifest.ancestors().nth(2)?;
        let tool = root.join("tools/floorplan/floorplan.py");
        tool.is_file().then_some(tool)
    }

    /// Run the tool over `grid`. Returns the HTTP status payload on success.
    pub async fn rebuild(&self, grid: Value, grid_version: u64) -> Result<Value, (u16, String)> {
        {
            let mut state = self.inner.lock().unwrap();
            if let Some(last) = state.last_attempt {
                if last.elapsed() < REBUILD_INTERVAL {
                    return Err((429, "rebuild rate limit: not more than once per 5 s".into()));
                }
            }
            state.last_attempt = Some(Instant::now());
        }
        let tool =
            Self::default_tool().ok_or_else(|| (500, "floorplan tool not found".to_string()))?;
        let stdin = serde_json::to_vec(&grid).map_err(|e| (500, format!("grid encode: {e}")))?;
        let mut child = Command::new("python3")
            .arg("-I")
            .arg(&tool)
            .arg("--json")
            .stdin(std::process::Stdio::piped())
            .stdout(std::process::Stdio::piped())
            .stderr(std::process::Stdio::null())
            .kill_on_drop(true)
            .spawn()
            .map_err(|e| (500, format!("spawn floorplan tool: {e}")))?;
        if let Some(mut pipe) = child.stdin.take() {
            pipe.write_all(&stdin)
                .await
                .map_err(|e| (500, format!("grid write: {e}")))?;
        }
        let output = tokio::time::timeout(TOOL_TIMEOUT, child.wait_with_output())
            .await
            .map_err(|_| (500, "floorplan tool timed out".to_string()))?
            .map_err(|e| (500, format!("floorplan tool: {e}")))?;
        if !output.status.success() {
            return Err((500, "floorplan tool failed".into()));
        }
        let parsed: Value = serde_json::from_slice(&output.stdout)
            .map_err(|_| (500, "floorplan tool produced invalid JSON".into()))?;
        let (version, summary) = {
            let mut state = self.inner.lock().unwrap();
            state.version += 1;
            state.grid_version = Some(grid_version);
            (
                state.version,
                json!({
                    "walls": parsed.get("walls").and_then(Value::as_array).map(Vec::len),
                    "rooms": parsed.get("rooms").and_then(Value::as_array).map(Vec::len),
                    "doors": parsed.get("doors").and_then(Value::as_array).map(Vec::len),
                    "obstacles": parsed.get("obstacles").and_then(Value::as_array).map(Vec::len),
                }),
            )
        };
        let frame = json!({
            "version": version,
            "grid_version": grid_version,
            "walls": parsed["walls"],
            "rooms": parsed["rooms"],
            "doors": parsed["doors"],
            "obstacles": parsed["obstacles"],
            "report": parsed["report"],
        });
        self.frames.send_replace(frame.clone());
        Ok(json!({"version": version, "summary": summary}))
    }
}

/// `POST /api/floorplan/rebuild`: no request body expected. The grid comes
/// from the live SLAM map channel; without a map this is an explicit 409.
pub async fn rebuild(app: &crate::App) -> (u16, Value) {
    let Some(maps) = app.maps.clone() else {
        return (
            409,
            json!({"error": "no SLAM map: floorplan rebuild needs a grid"}),
        );
    };
    let mut maps = maps;
    let current = maps.borrow_and_update().clone();
    let grid_version = current
        .get("version")
        .and_then(Value::as_u64)
        .or_else(|| current.get("grid_version").and_then(Value::as_u64));
    let (grid_version, grid) = match grid_version.zip(current.get("grid").cloned()) {
        Some((v, g)) => (v, g),
        None => {
            return (
                409,
                json!({"error": "no SLAM map yet: floorplan rebuild needs a grid"}),
            )
        }
    };
    match app.floorplans.rebuild(grid, grid_version).await {
        Ok(v) => (200, v),
        Err((status, message)) => (status, json!({"error": message})),
    }
}

// Silence the unused-import lint for the Failure alias kept for symmetry.
#[allow(dead_code)]
fn _unused(_: Failure) {}
