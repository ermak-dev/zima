# Unified session recording

Opt-in host-side recording for `robot-server`. It is passive: the recorder has
no Player client, drive handle, supervisor authentication, or movement API.
A stationary physical session on 2026-10-29 verified the CLI/HTTP/WS path,
raw capture, commands, shared clock, private files, and honest unavailable-camera
status. Actual camera JPEG capture was verified with local HTTP fixtures only;
the physical camera remained unreachable.

## Launch and configuration

```sh
bin/zima run --record-dir /absolute/path/to/new-session
```

This is a normal live robot session, not a passive attach command. Usual hardware
authorization and controller restoration rules apply. The CLI refuses an existing
recording destination before SSH/handoff; the server performs authoritative
no-follow preflight before `ACTIVATE`.

`[run].record_dir` in the shared TOML is the same opt-in directory setting;
TOML paths are relative to the configuration file. CLI overrides environment,
which overrides TOML. Keep it empty normally and choose a fresh target for each
recording; there is deliberately no automatic overwrite/reuse.

## Environment

`ROBOT_SESSION_RECORD_DIR=/absolute/path/to/new-session` enables a bundle.
Unset or empty disables it. All parent directories must already exist. The last
directory **must not exist**; existing directories/files and symlink components
are refused. Paths containing `..` are refused. Linux `/proc/self/fd` is required:
parent traversal uses no-follow directory descriptors, and the bundle directory
stays pinned so a later rename/symlink replacement cannot redirect JPEG writes.
The new directory is `0700`; every file is create-new `0600`, including under a
restrictive umask. A failed preflight may leave a partial private directory;
never reuse it as the next destination.

Server preflight happens before connecting to the supervisor or sending
`ACTIVATE`. An existing destination is rejected by CLI before handoff; other
server preflight errors still rely on the CLI's normal controller rollback. Audio-only mode
ignores recording, just as it ignores the existing raw Player recorder.

`ROBOT_SLAM_RECORD_PATH` and `ROBOT_SESSION_RECORD_DIR` are mutually exclusive
when nonempty. A session opens the **existing** raw recorder at
`player.jsonl`, and passes it to the sole existing Player connection. No second
Player subscription/client is created. The only change to the raw recorder is
`Recorder::open_at(path, Instant)`, allowing a shared clock origin; its queue,
payload limits, format, footer, and standalone environment option are retained.

There are no additional recording environment knobs or HTTP endpoints. Camera
URL/netrc still come from the existing server configuration. No auth headers,
netrc contents, supervisor tokens, URLs, TTS text, or arbitrary inbound WS text
are sent to the recorder. Only successfully parsed finite wheel requests and
strict reset requests are logged. Snapshots also exclude map and secret-bearing
fields before copying. JPEG images and telemetry themselves can be sensitive;
private permissions are not encryption.

## Bundle and clocks

- `manifest.json`: initially `complete:false`, `status:recording_or_interrupted`;
  replaced in place by the disk worker with final counts/statuses. Missing,
  invalid, initial, timed-out, or interrupted manifests mean **incomplete**.
  Rewriting is not crash-atomic or fsync-based durability.
- `player.jsonl`: original Player headers/payloads, including startup and
  uninterpreted frames, using the existing `robot-player-raw-v1` format/footer.
- `events.jsonl`: `type`, `receive_offset_ns`, `data`. Types are `state`, `scan`,
  `wheel_command`, `reset_blocked`, and `camera_frame`.
- `frame-00000000.jpg`, etc.: extracted JPEGs. Their corresponding event contains
  `data.file`; no video container, audio, or YOLO annotations are recorded.

All offsets use **one `std::time::Instant` session origin**, in nanoseconds.
Player offsets are receipt times from the existing reader. Camera timestamps
are when the HTTP chunk completing the JPEG was received, not exposure time.
State is sampled at 10 Hz; scan snapshots use the existing latest-only watch
channel and can skip intervening publications. Their offsets are **snapshot
sampling times**, not original sensor receipt times. Original scan/Player source
stamps remain separate, unverified metadata, not UTC or verified measurement
clocks. Events from independent producers need not be ordered by timestamp.
Wheel entries contain a local client ID, v/w, and an explicit inbound-request
label: they do **not** prove safety acceptance, applied velocity, or motion.
Reset logging likewise does not prove a state transition.

`slam.map` is omitted **before cloning**, including repeated maps. Other SLAM
state/diagnostics and telemetry remain available within snapshot bounds. This
bundle is for synchronized inspection, not occupancy-map persistence or a new
SLAM replay format.

## Camera and completion

Capture uses a dedicated HTTP MJPEG stream via the existing `Camera::stream()`
implementation, even when vision/proxy consumers are also active. This avoids
coupling capture to a browser or a stalled detector, at the cost of an additional
camera HTTP connection/bandwidth. It is **not** another Player connection. The
configured Basic credentials stay server-side; redirects are never followed.
There is no automatic camera reconnect: an early stream ending is not silently
represented as full-session video. JPEG marker extraction tolerates fragmented
HTTP/multipart chunks; it is not a JPEG decoder or image-integrity validator.

Manifest sections are independent:

- `raw_player`: the raw recorder's actual completion/loss/error stats plus
  `recorded`, `partial`, or `unavailable` (zero raw frames).
- `sensors`: `sampled`, `partial`, or `unavailable`, snapshot counts and sensor
  drops. `sampled` does not imply every physical sensor supplied fresh data.
- `camera`: frame count, camera drops, and `unavailable`, `partial`, or
  `recorded_until_cancel`. End reasons include `eof`, `timeout`, `size_limit`,
  `unavailable`, and normal session `cancelled`. Zero frames are never success.
- `events`: combined event counts, explicit drop count, writer I/O failure.
- `session_ok`: whether server lifecycle returned successfully. Overall
  `complete` additionally requires successful raw/event capture, state snapshots,
  and camera recording through normal cancellation. An unavailable camera never
  makes the sensor section unavailable, nor does it stop robot control.

Limits: 32 queued events; conservative 256 KiB per snapshot (including copy
budget); 2 MiB per JPEG; 64 MiB total event/JPEG content; 64 MiB raw Player file
(separate existing 1024-frame queue and 64 KiB payload cap); one-hour session-event
window; 64 MiB incoming camera-stream budget; five-second camera idle timeout;
a five-second outer camera-open deadline (including credential loading),
and existing bounded camera connect/request timeouts. Filesystem inode/block
allocation overhead is not included in byte budgets. Queue overflow, oversize,
and size/time limits are reported as drops/incomplete; no old commands or frames
are replayed to compensate. Snapshots intentionally sample/skip updates.

Serialization and disk writes run on a detached bounded-queue worker. Reader
and control paths never wait for disk/queue space. On cleanup, local capture
cancellation never joins HTTP/disk work. After command producers stop, the
existing bounded final-zero attempt and heartbeat teardown occur **before** a
shared finalization budget of at most two seconds. Missing completion remains
incomplete. SIGKILL, filesystem failure, or process exit can leave only the
initial manifest or a partial final rewrite.

## Offline inspection and API

Without launching a server or contacting any device:

```sh
jq . /path/to/session/manifest.json
jq -c 'select(.type == "wheel_command" or .type == "reset_blocked")' \
  /path/to/session/events.jsonl
jq -c 'select(.type == "camera_frame") | {receive_offset_ns, file:.data.file}' \
  /path/to/session/events.jsonl
# Verify the raw footer independently of the camera section:
tail -n 1 /path/to/session/player.jsonl | jq .
```

Rust API lives in `robot_server::session_recording`: `Session::open` /
`from_env`, `raw: Arc<player_recording::Recorder>`, `start()`, bounded
`snapshot`, typed `command`, `jpeg`, `camera_end`, and
`finish(Duration, session_ok) -> serde_json::Value`. `start_sources` accepts
existing state/scan watch receivers and the existing camera-stream future;
its RAII `Sources` guard cancels only local tasks on drop. Stop producers before
calling `finish`, once. `Jpegs` is the bounded marker parser used by capture.

Offline tests:

```sh
cargo test --manifest-path crates/Cargo.toml -p robot-server --test session_recording
cargo test --manifest-path crates/Cargo.toml -p robot-server --test server unified_session
cargo test --manifest-path crates/Cargo.toml --workspace
```
