// FEATURE: passive bounded raw Player recording on the existing connection.
// SPEC: private create-new JSONL, exact payload/header and original monotonic receive
// clock, nonfinite source stamps null, bounded payload/queue/file and bounded finish.
// IMPLEMENTATION: bounded try-send to detached writer; explicit loss and completion stats.
// RESULT: IMPLEMENTED — private raw bytes, timing, bounds and loss reporting pass.
// STATUS: DONE
use base64::{engine::general_purpose::STANDARD, Engine};
use player_bridge::{DevAddr, Message, MsgHeader};
use robot_server::player_recording::Recorder;
use serde_json::Value;
use std::{
    fs,
    os::unix::fs::{symlink, PermissionsExt},
    path::PathBuf,
    sync::atomic::{AtomicU64, Ordering},
    time::{Duration, Instant},
};

struct Temp(PathBuf);
impl Temp {
    fn new() -> Self {
        static ID: AtomicU64 = AtomicU64::new(0);
        let p = std::env::temp_dir().join(format!(
            "player-recording-{}-{}",
            std::process::id(),
            ID.fetch_add(1, Ordering::Relaxed)
        ));
        fs::create_dir(&p).unwrap();
        Self(p)
    }
    fn path(&self) -> PathBuf {
        self.0.join("raw.jsonl")
    }
}
impl Drop for Temp {
    fn drop(&mut self) {
        let _ = fs::remove_dir_all(&self.0);
    }
}
fn message(stamp: f64, payload: Vec<u8>) -> Message {
    Message {
        header: MsgHeader {
            addr: DevAddr {
                host: 0,
                robot: 0,
                interf: 6,
                index: 2,
            },
            msg_type: 6,
            subtype: 3,
            timestamp: stamp,
            seq: 4294967295,
            size: payload.len() as u32,
        },
        payload,
    }
}
fn rows(path: &PathBuf) -> Vec<Value> {
    fs::read_to_string(path)
        .unwrap()
        .lines()
        .map(|s| serde_json::from_str(s).unwrap())
        .collect()
}
#[test]
fn private_exact_raw_frames_and_original_receive_clock() {
    let t = Temp::new();
    let r = Recorder::open(t.path()).unwrap();
    let received = Instant::now();
    std::thread::sleep(Duration::from_millis(40));
    let payload = vec![0, 255, 1, 128, 0, 42];
    r.record(&message(-2.5, payload.clone()), received);
    r.record(
        &message(f64::NAN, vec![]),
        received + Duration::from_millis(7),
    );
    let stats = r.finish(Duration::from_secs(2));
    assert!(stats.complete, "{stats:?}");
    assert_eq!(stats.frames_written, 2);
    assert_eq!(
        fs::metadata(t.path()).unwrap().permissions().mode() & 0o777,
        0o600
    );
    let v = rows(&t.path());
    assert_eq!(v[0]["type"], "header");
    assert_eq!(v[0]["format"], "robot-player-raw-v1");
    assert_eq!(v.len(), 4);
    let f = &v[1];
    assert_eq!(f["type"], "frame");
    assert_eq!(f["interface"], 6);
    assert_eq!(f["index"], 2);
    assert_eq!(f["msg_type"], 6);
    assert_eq!(f["subtype"], 3);
    assert_eq!(f["source_stamp"], -2.5);
    assert_eq!(f["source_seq"], 4294967295u64);
    assert_eq!(
        STANDARD
            .decode(f["payload_base64"].as_str().unwrap())
            .unwrap(),
        payload
    );
    assert!(
        f["receive_offset_ns"].as_u64().unwrap() < 30_000_000,
        "must not use enqueue time"
    );
    assert_eq!(
        v[2]["receive_offset_ns"].as_u64().unwrap() - f["receive_offset_ns"].as_u64().unwrap(),
        7_000_000
    );
    assert!(v[2]["source_stamp"].is_null());
    assert_eq!(v[3]["type"], "completion");
    assert_eq!(v[3]["complete"], true);
}
#[test]
fn private_mode_is_exact_even_with_restrictive_umask() {
    if let Some(path) = std::env::var_os("PLAYER_RECORDING_UMASK_TEST_PATH") {
        let r = Recorder::open(&path).unwrap();
        assert_eq!(
            fs::metadata(path).unwrap().permissions().mode() & 0o777,
            0o600
        );
        assert!(r.finish(Duration::from_secs(2)).complete);
        return;
    }
    // Isolate process-global umask from parallel tests; create the directory first.
    let t = Temp::new();
    let status = std::process::Command::new("bash")
        .args(["-c", "umask 0200; exec \"$@\"", "umask-test"])
        .arg(std::env::current_exe().unwrap())
        .args([
            "--exact",
            "private_mode_is_exact_even_with_restrictive_umask",
        ])
        .env("PLAYER_RECORDING_UMASK_TEST_PATH", t.path())
        .status()
        .unwrap();
    assert!(status.success());
}
#[test]
fn never_overwrites_existing_files_or_symlinks() {
    let t = Temp::new();
    fs::write(t.path(), b"keep").unwrap();
    assert!(Recorder::open(t.path()).is_err());
    let link = t.0.join("link");
    symlink(t.path(), &link).unwrap();
    assert!(Recorder::open(&link).is_err());
    let dangling = t.0.join("dangling");
    symlink(t.0.join("missing"), &dangling).unwrap();
    assert!(Recorder::open(&dangling).is_err());
    assert_eq!(fs::read(t.path()).unwrap(), b"keep");
    assert!(!t.0.join("missing").exists());
}
#[test]
fn oversize_is_explicitly_incomplete_and_finish_is_bounded() {
    let t = Temp::new();
    let r = Recorder::open(t.path()).unwrap();
    r.record(&message(f64::INFINITY, vec![1; 65537]), Instant::now());
    let s = r.finish(Duration::from_secs(20));
    assert!(!s.complete);
    assert!(s.writer_finished);
    assert_eq!(s.oversized, 1);
    let v = rows(&t.path());
    assert_eq!(v.len(), 2);
    assert_eq!(v[1]["complete"], false);
    assert_eq!(v[1]["oversized"], 1);
    let t = Temp::new();
    let r = Recorder::open(t.path()).unwrap();
    let start = Instant::now();
    let _ = r.finish(Duration::ZERO);
    assert!(start.elapsed() < Duration::from_millis(100));
}
#[test]
fn bounded_file_retains_footer_and_reports_truncation() {
    let t = Temp::new();
    let r = Recorder::open(t.path()).unwrap();
    let m = message(1., vec![123; 65536]);
    for _ in 0..850 {
        r.record(&m, Instant::now());
        std::thread::sleep(Duration::from_millis(3));
    }
    let s = r.finish(Duration::from_secs(2));
    assert!(s.writer_finished);
    assert!(!s.complete);
    assert!(s.size_limited > 0, "{s:?}");
    assert!(fs::metadata(t.path()).unwrap().len() <= 64 * 1024 * 1024);
    let v = rows(&t.path());
    assert_eq!(v.last().unwrap()["type"], "completion");
    assert_eq!(v.last().unwrap()["size_limited"], s.size_limited);
}
#[test]
fn enqueue_never_waits_for_writer_capacity() {
    let t = Temp::new();
    let r = Recorder::open(t.path()).unwrap();
    let m = message(1., vec![0; 65536]);
    let start = Instant::now();
    for _ in 0..10000 {
        r.record(&m, Instant::now());
    }
    assert!(start.elapsed() < Duration::from_secs(1));
    let s = r.finish(Duration::from_secs(2));
    assert!(s.queue_dropped > 0, "bounded queue must report loss: {s:?}");
    assert!(!s.complete);
}
