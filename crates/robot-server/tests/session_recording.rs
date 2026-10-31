// FEATURE: opt-in unified private session bundles without control or disk blocking.
// SPEC: shared receipt clock, bounded queues/files, no map repeats or secrets,
// independent camera/raw status, refuse overwrite/symlinks, bounded completion.
// IMPLEMENTATION: descriptor-pinned private bundle, shared raw clock, bounded worker
// and independent latest-only snapshots/authenticated camera capture.
// RESULT: IMPLEMENTED — focused tests cover privacy, loss, clocks, limits and cleanup.
// STATUS: DONE
use robot_server::session_recording::{JpegStream, Session};
use serde_json::{json, Value};
use std::{
    fs,
    os::unix::fs::{symlink, PermissionsExt},
    path::PathBuf,
    sync::atomic::{AtomicU64, Ordering},
    time::{Duration, Instant},
};
#[test]
fn event_writer_failure_does_not_fabricate_raw_player_failure() {
    let t = Temp::new();
    let r = Session::open(&t.0).unwrap();
    fs::write(t.0.join("frame-00000000.jpg"), b"keep").unwrap();
    r.jpeg(b"\xff\xd8test\xff\xd9".to_vec(), Instant::now());
    std::thread::sleep(Duration::from_millis(100));
    r.raw.record(
        &player_bridge::Message {
            header: player_bridge::MsgHeader {
                addr: player_bridge::DevAddr {
                    host: 0,
                    robot: 0,
                    interf: 6,
                    index: 0,
                },
                msg_type: 1,
                subtype: 3,
                timestamp: 123.,
                seq: 1,
                size: 3,
            },
            payload: vec![0, 255, 128],
        },
        Instant::now(),
    );
    let report = r.finish(Duration::from_secs(2), true);
    assert_eq!(report["raw_player"]["complete"], true);
    assert_eq!(report["raw_player"]["frames_written"], 1);
    assert_eq!(report["events"]["error"], "writer I/O failure");
    assert_eq!(report["complete"], false);
    assert_eq!(fs::read(t.0.join("frame-00000000.jpg")).unwrap(), b"keep");
}
#[test]
fn empty_raw_stream_cannot_make_an_overall_complete_bundle() {
    let t = Temp::new();
    let r = Session::open(&t.0).unwrap();
    r.snapshot("state", &json!({"sensors":{}}), Instant::now());
    r.jpeg(b"\xff\xd8test\xff\xd9".to_vec(), Instant::now());
    r.camera_end("cancelled");
    let report = r.finish(Duration::from_secs(2), true);
    assert_eq!(report["raw_player"]["status"], "unavailable");
    assert_eq!(report["complete"], false);
}
// FEATURE: camera capture tolerates the shared LDS/camera power boot delay.
// SPEC: a failing stream open is retried on a bounded budget; persistent
// failure stays honest `unavailable`/`timeout`, never claimed success.
#[tokio::test]
async fn camera_boot_delay_is_retried_within_a_bounded_budget() {
    let t = Temp::new();
    let r = std::sync::Arc::new(Session::open(&t.0).unwrap());
    let (_state, state) = tokio::sync::watch::channel(json!({"sensors":{}}));
    let (_scan, scan) = tokio::sync::watch::channel(None);
    let attempts = std::sync::Arc::new(AtomicU64::new(0));
    let seen = attempts.clone();
    let sources = robot_server::session_recording::start_sources(
        r.clone(),
        move || {
            let count = seen.fetch_add(1, Ordering::Relaxed);
            async move {
                if count < 2 {
                    Err(std::io::Error::other("camera booting"))
                } else {
                    // A body error after opening: capture ends but boot retries stop.
                    Err(std::io::Error::other("stream ended"))
                }
            }
        },
        state,
        scan,
    );
    tokio::time::sleep(Duration::from_millis(2600)).await;
    drop(sources);
    let report = r.finish(Duration::from_secs(2), true);
    assert!(
        attempts.load(Ordering::Relaxed) >= 3,
        "must retry boot failures, saw {}",
        attempts.load(Ordering::Relaxed)
    );
    assert_eq!(report["camera"]["frames"], 0);
    assert_eq!(report["camera"]["status"], "unavailable");
}

// A pending future proves session teardown never waits for the boot budget:
// dropping sources cancels capture immediately with no success claim.
#[tokio::test]
async fn camera_open_future_has_an_outer_deadline() {
    let t = Temp::new();
    let r = std::sync::Arc::new(Session::open(&t.0).unwrap());
    let (_state, state) = tokio::sync::watch::channel(json!({"sensors":{}}));
    let (_scan, scan) = tokio::sync::watch::channel(None);
    let sources = robot_server::session_recording::start_sources(
        r.clone(),
        || async { std::future::pending::<Result<reqwest::Response, std::io::Error>>().await },
        state,
        scan,
    );
    tokio::time::sleep(Duration::from_millis(300)).await;
    let began = Instant::now();
    drop(sources);
    let report = r.finish(Duration::from_secs(2), true);
    assert!(
        began.elapsed() < Duration::from_secs(1),
        "teardown must not wait for the boot budget"
    );
    assert_eq!(report["camera"]["status"], "unavailable");
}

struct Temp(PathBuf);
impl Temp {
    fn new() -> Self {
        static ID: AtomicU64 = AtomicU64::new(0);
        Self(std::env::temp_dir().join(format!(
            "session-test-{}-{}",
            std::process::id(),
            ID.fetch_add(1, Ordering::Relaxed)
        )))
    }
}
impl Drop for Temp {
    fn drop(&mut self) {
        let _ = fs::remove_dir_all(&self.0);
    }
}
fn rows(p: &PathBuf) -> Vec<Value> {
    fs::read_to_string(p)
        .unwrap()
        .lines()
        .map(|s| serde_json::from_str(s).unwrap())
        .collect()
}
#[test]
fn private_bundle_shared_clock_snapshots_and_commands() {
    let t = Temp::new();
    let r = Session::open(&t.0).unwrap();
    let received = r.start() + Duration::from_millis(7);
    r.raw.record(
        &player_bridge::Message {
            header: player_bridge::MsgHeader {
                addr: player_bridge::DevAddr {
                    host: 0,
                    robot: 0,
                    interf: 6,
                    index: 0,
                },
                msg_type: 1,
                subtype: 3,
                timestamp: 123.,
                seq: 1,
                size: 3,
            },
            payload: vec![0, 255, 128],
        },
        received,
    );
    r.snapshot("state", &json!({"odom":{"x":1},"slam":{"map":"x".repeat(2_000_000),"tracking":"ok"},"blocked":false}),received);
    r.snapshot("scan", &json!({"points":[[1,2]]}), received);
    r.command(9, Some((0.1, 0.2)), received);
    r.command(9, None, received);
    r.jpeg(b"\xff\xd8test\xff\xd9".to_vec(), received);
    r.camera_end("eof");
    let report = r.finish(Duration::from_secs(2), true);
    assert_eq!(report["raw_player"]["complete"], true);
    assert_eq!(report["camera"]["status"], "partial");
    assert_eq!(report["events"]["complete"], true);
    assert_eq!(
        fs::metadata(&t.0).unwrap().permissions().mode() & 0o777,
        0o700
    );
    for e in fs::read_dir(&t.0).unwrap() {
        assert_eq!(
            e.unwrap().metadata().unwrap().permissions().mode() & 0o777,
            0o600
        );
    }
    let v = rows(&t.0.join("events.jsonl"));
    assert_eq!(v[0]["receive_offset_ns"], 7_000_000);
    assert!(v[0]["data"]["slam"].get("map").is_none());
    assert_eq!(v[2]["type"], "wheel_command");
    assert_eq!(v[3]["type"], "reset_blocked");
    assert_eq!(v[4]["receive_offset_ns"], 7_000_000);
    assert_eq!(
        fs::read(t.0.join("frame-00000000.jpg")).unwrap(),
        b"\xff\xd8test\xff\xd9"
    );
    assert!(fs::metadata(t.0.join("events.jsonl")).unwrap().len() < 4096);
    let raw = rows(&t.0.join("player.jsonl"));
    assert_eq!(raw[1]["receive_offset_ns"], 7_000_000);
    assert_eq!(raw[1]["payload_base64"], "AP+A");
    assert_eq!(
        rows(&t.0.join("player.jsonl"))[0]["receive_clock"],
        "monotonic offset from session start in nanoseconds"
    );
}
#[test]
fn logical_bundle_size_and_duration_limits_are_explicit() {
    let t = Temp::new();
    let r = Session::open(&t.0).unwrap();
    for _ in 0..150 {
        r.jpeg(vec![0; 512 * 1024], Instant::now());
        std::thread::sleep(Duration::from_millis(5));
    }
    r.command(1, None, r.start() + Duration::from_secs(3601));
    r.camera_end("cancelled");
    let report = r.finish(Duration::from_secs(2), true);
    assert!(report["camera"]["frames"].as_u64().unwrap() >= 100);
    assert!(report["camera"]["dropped"].as_u64().unwrap() > 0);
    assert_eq!(report["complete"], false);
    let size: u64 = fs::read_dir(&t.0)
        .unwrap()
        .map(|e| {
            let e = e.unwrap();
            if e.file_name() == "events.jsonl"
                || e.file_name().to_string_lossy().starts_with("frame-")
            {
                e.metadata().unwrap().len()
            } else {
                0
            }
        })
        .sum();
    assert!(size <= 64 * 1024 * 1024);
    assert!(!rows(&t.0.join("events.jsonl"))
        .iter()
        .any(|v| v["type"] == "reset_blocked"));
}
#[test]
fn restrictive_umask_still_creates_exact_private_modes() {
    if let Some(path) = std::env::var_os("SESSION_RECORDING_UMASK_TEST_PATH") {
        let r = Session::open(&path).unwrap();
        r.finish(Duration::from_secs(2), true);
        assert_eq!(
            fs::metadata(&path).unwrap().permissions().mode() & 0o777,
            0o700
        );
        for e in fs::read_dir(path).unwrap() {
            assert_eq!(
                e.unwrap().metadata().unwrap().permissions().mode() & 0o777,
                0o600
            );
        }
        return;
    }
    let t = Temp::new();
    let status = std::process::Command::new("bash")
        .args(["-c", "umask 0200; exec \"$@\"", "umask-test"])
        .arg(std::env::current_exe().unwrap())
        .args([
            "--exact",
            "restrictive_umask_still_creates_exact_private_modes",
        ])
        .env("SESSION_RECORDING_UMASK_TEST_PATH", &t.0)
        .status()
        .unwrap();
    assert!(status.success());
}
#[test]
fn camera_loss_does_not_claim_sensor_loss() {
    let t = Temp::new();
    let r = Session::open(&t.0).unwrap();
    r.snapshot("state", &json!({"odom":{"x":1}}), Instant::now());
    r.jpeg(vec![0; 2 * 1024 * 1024 + 1], Instant::now());
    r.camera_end("cancelled");
    let report = r.finish(Duration::from_secs(2), true);
    assert_eq!(report["sensors"]["status"], "sampled");
    assert_eq!(report["sensors"]["dropped"], 0);
    assert_eq!(report["camera"]["dropped"], 1);
}
#[test]
fn snapshots_reject_oversize_before_queue_and_exclude_private_fields() {
    let t = Temp::new();
    let r = Session::open(&t.0).unwrap();
    r.snapshot("state",&json!({"odom":{"x":1},"token":"secret-token","tts":{"text":"private speech"},"slam":{"tracking":"ok","authorization":"private-auth"}}),Instant::now());
    r.snapshot(
        "scan",
        &json!({"points":vec![json!([1,2]);100_000]}),
        Instant::now(),
    );
    let report = r.finish(Duration::from_secs(2), true);
    let text = fs::read_to_string(t.0.join("events.jsonl")).unwrap();
    assert!(
        !text.contains("secret-token")
            && !text.contains("private speech")
            && !text.contains("private-auth")
    );
    assert_eq!(report["events"]["dropped"], 1);
    assert_eq!(rows(&t.0.join("events.jsonl")).len(), 1);
}
#[test]
fn refuses_existing_and_symlink_components() {
    let t = Temp::new();
    fs::create_dir(&t.0).unwrap();
    fs::write(t.0.join("keep"), "safe").unwrap();
    assert!(Session::open(&t.0).is_err());
    let link = t.0.with_extension("link");
    symlink(&t.0, &link).unwrap();
    assert!(Session::open(link.join("new")).is_err());
    assert!(!t.0.join("new").exists());
    fs::remove_file(link).unwrap();
    assert_eq!(fs::read_to_string(t.0.join("keep")).unwrap(), "safe");
}
#[test]
fn backpressure_oversize_and_finish_are_bounded_and_honest() {
    let t = Temp::new();
    let r = Session::open(&t.0).unwrap();
    let began = Instant::now();
    for _ in 0..10000 {
        r.jpeg(vec![1; 65536], Instant::now());
    }
    assert!(began.elapsed() < Duration::from_secs(2));
    let report = r.finish(Duration::from_secs(2), false);
    assert_eq!(report["complete"], false);
    assert!(report["events"]["dropped"].as_u64().unwrap() > 0);
    let t = Temp::new();
    let r = Session::open(&t.0).unwrap();
    r.jpeg(vec![0; 2 * 1024 * 1024 + 1], Instant::now());
    let began = Instant::now();
    let report = r.finish(Duration::ZERO, false);
    assert!(began.elapsed() < Duration::from_millis(100));
    assert_eq!(report["complete"], false);
}
#[test]
fn directory_replacement_cannot_redirect_frame_writes() {
    let t = Temp::new();
    let outside = Temp::new();
    fs::create_dir(&outside.0).unwrap();
    let r = Session::open(&t.0).unwrap();
    let moved = t.0.with_extension("moved");
    fs::rename(&t.0, &moved).unwrap();
    symlink(&outside.0, &t.0).unwrap();
    r.jpeg(b"\xff\xd8test\xff\xd9".to_vec(), Instant::now());
    r.camera_end("cancelled");
    let report = r.finish(Duration::from_secs(2), true);
    fs::remove_file(&t.0).unwrap();
    fs::rename(&moved, &t.0).unwrap();
    assert_eq!(report["camera"]["frames"], 1);
    assert!(
        !outside.0.join("frame-00000000.jpg").exists(),
        "frame escaped through replaced directory"
    );
    assert!(t.0.join("frame-00000000.jpg").exists());
}
#[test]
fn independent_empty_sensor_and_camera_status_is_not_success() {
    let t = Temp::new();
    let r = Session::open(&t.0).unwrap();
    r.camera_end("unavailable");
    let report = r.finish(Duration::from_secs(2), true);
    assert_eq!(report["sensors"]["status"], "unavailable");
    assert_eq!(report["camera"]["status"], "unavailable");
    assert_eq!(report["complete"], false);
    assert_eq!(report["raw_player"]["status"], "unavailable");
}
// DEDUP (issue #51): one bounded MJPEG extractor serves both session
// recording (with an honest oversize drop counter) and vision capture.
// SPEC: fragmented markers resynchronize, several frames per feed are
// returned in order, garbage without a start marker never accumulates,
// oversize frames drop with a count, and the parser recovers afterwards.
#[test]
fn fragmented_mjpeg_is_bounded_and_reports_oversize() {
    let mut p = JpegStream::default();
    assert!(p.feed(b"--frame\r\n\xff").is_empty());
    assert!(p.feed(b"\xd8one\xff").is_empty());
    assert_eq!(
        p.feed(b"\xd9\xff\xd8two\xff\xd9"),
        vec![
            b"\xff\xd8one\xff\xd9".to_vec(),
            b"\xff\xd8two\xff\xd9".to_vec()
        ]
    );
    let mut big = vec![0xff, 0xd8];
    big.resize(2 * 1024 * 1024 + 10, 1);
    assert!(p.feed(&big).is_empty());
    assert_eq!(p.dropped, 1);
    assert!(p.buffered() <= 2 * 1024 * 1024);
    // Recovery: a well-formed frame still decodes after the oversize drop.
    assert_eq!(
        p.feed(b"\xff\xd8next\xff\xd9"),
        vec![b"\xff\xd8next\xff\xd9".to_vec()]
    );
    assert_eq!(p.dropped, 1);
}

// SPEC: the memory bound holds for garbage without any start marker too —
// the residue is trimmed continuously, not only when a frame completes.
#[test]
fn markerless_garbage_stays_bounded_and_drops_once() {
    let mut p = JpegStream::default();
    let garbage = vec![7u8; 2 * 1024 * 1024 + 100];
    assert!(p.feed(&garbage).is_empty());
    assert!(p.buffered() <= 2 * 1024 * 1024);
    assert!(p.dropped <= 1);
}

// SPEC: a huge feed splitting one frame is chunk-processed internally and
// never accumulates more than the limit, whatever chunk size the caller has.
#[test]
fn oversized_single_frame_split_across_chunks_drops_and_stays_bounded() {
    let mut p = JpegStream::default();
    let mut big = vec![0xff, 0xd8];
    big.resize(2 * 1024 * 1024 + 10, 1);
    for piece in big.chunks(65536) {
        assert!(p.feed(piece).is_empty());
        assert!(p.buffered() <= 2 * 1024 * 1024);
    }
    assert_eq!(p.dropped, 1);
    assert_eq!(
        p.feed(b"\xff\xd8ok\xff\xd9"),
        vec![b"\xff\xd8ok\xff\xd9".to_vec()]
    );
}
