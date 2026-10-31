//! Independent, latest-only camera inference; no drive or safety access.
use crate::{allowed_origin, camera::Camera, App, Failure};
use axum::{
    extract::{
        ws::{Message, WebSocket},
        State, WebSocketUpgrade,
    },
    http::{HeaderMap, StatusCode},
    response::{IntoResponse, Response},
};
use base64::{engine::general_purpose::STANDARD, Engine};
use futures_util::{SinkExt, StreamExt};
use robot_server::session_recording::JpegStream;
use serde_json::{json, Value};
use std::{
    path::PathBuf,
    process::Stdio,
    sync::Arc,
    time::{Duration, Instant},
};
use tokio::{
    io::{AsyncBufReadExt, AsyncReadExt, AsyncWriteExt, BufReader},
    process::{Child, ChildStdin, ChildStdout, Command},
    sync::watch,
    task::JoinHandle,
};

const REPLY_LIMIT: usize = 65536;
const MAX_AGE: Duration = Duration::from_secs(3);

fn validate_reply(frame_id: u64, value: Value) -> Result<Value, Failure> {
    if value["ok"] != true || value["frame_id"].as_u64() != Some(frame_id) {
        return Err("YOLO adapter failed or returned a different frame".into());
    }
    let width = value["width"]
        .as_u64()
        .filter(|n| (1..=8192).contains(n))
        .ok_or("invalid detection width")?;
    let height = value["height"]
        .as_u64()
        .filter(|n| (1..=8192).contains(n))
        .ok_or("invalid detection height")?;
    if width * height > 16_777_216 {
        return Err("detection image too large".into());
    }
    let objects = value["objects"]
        .as_array()
        .filter(|v| v.len() <= 100)
        .ok_or("invalid detections list")?;
    let latency = value["inference_ms"]
        .as_f64()
        .filter(|v| v.is_finite() && *v >= 0.)
        .ok_or("invalid inference time")?;
    let mut checked = Vec::new();
    for object in objects {
        let label = object["label"]
            .as_str()
            .filter(|s| !s.is_empty() && s.len() <= 256)
            .ok_or("invalid object label")?;
        let confidence = object["confidence"]
            .as_f64()
            .filter(|v| (0.0..=1.0).contains(v))
            .ok_or("invalid object confidence")?;
        let bbox = object["box"]
            .as_array()
            .filter(|v| v.len() == 4)
            .ok_or("invalid object box")?;
        let coords: Vec<f64> = bbox
            .iter()
            .map(|v| {
                v.as_f64()
                    .filter(|n| (0.0..=1.0).contains(n))
                    .ok_or("invalid box coordinate")
            })
            .collect::<Result<_, _>>()?;
        if coords[0] >= coords[2] || coords[1] >= coords[3] {
            return Err("empty object box".into());
        }
        checked.push(json!({"label":label,"confidence":confidence,"box":coords}));
    }
    Ok(
        json!({"status":"live","frame_id":frame_id,"width":width,"height":height,
        "objects":checked,"inference_ms":latency}),
    )
}

struct Adapter {
    child: Child,
    input: ChildStdin,
    output: BufReader<ChildStdout>,
}
impl Adapter {
    fn spawn(exe: &PathBuf) -> Result<Self, Failure> {
        let mut child = Command::new(exe)
            .stdin(Stdio::piped())
            .stdout(Stdio::piped())
            .stderr(Stdio::inherit())
            .kill_on_drop(true)
            .spawn()?;
        let input = child.stdin.take().ok_or("missing detector stdin")?;
        let output = BufReader::new(child.stdout.take().ok_or("missing detector stdout")?);
        Ok(Self {
            child,
            input,
            output,
        })
    }
    async fn call(&mut self, request: Value, deadline: Duration) -> Result<Value, Failure> {
        tokio::time::timeout(deadline, async {
            let mut line = serde_json::to_vec(&request)?;
            line.push(b'\n');
            self.input.write_all(&line).await?;
            self.input.flush().await?;
            let mut reply = Vec::new();
            (&mut self.output)
                .take((REPLY_LIMIT + 1) as u64)
                .read_until(b'\n', &mut reply)
                .await?;
            if reply.len() > REPLY_LIMIT || reply.last() != Some(&b'\n') {
                return Err::<Value, Failure>("YOLO adapter closed or reply exceeds limit".into());
            }
            Ok(serde_json::from_slice(&reply)?)
        })
        .await
        .map_err(|_| "YOLO adapter timed out")?
    }
}
impl Drop for Adapter {
    fn drop(&mut self) {
        let _ = self.child.start_kill();
    }
}

struct Frame {
    id: u64,
    jpeg: Vec<u8>,
    received: Instant,
}
fn error(tx: &watch::Sender<Value>, message: &str) {
    tx.send_replace(json!({"status":"error","error":message}));
}

async fn capture(
    camera: Camera,
    frames: watch::Sender<Option<Arc<Frame>>>,
    updates: watch::Sender<Value>,
) {
    let mut id = 0;
    loop {
        let stream = camera.stream().await;
        if let Ok(response) = stream {
            let mut bytes = response.bytes_stream();
            let mut parser = JpegStream::default();
            while let Ok(Some(Ok(chunk))) =
                tokio::time::timeout(Duration::from_secs(5), bytes.next()).await
            {
                for jpeg in parser.feed(&chunk) {
                    id += 1;
                    frames.send_replace(Some(Arc::new(Frame {
                        id,
                        jpeg,
                        received: Instant::now(),
                    })));
                }
            }
        }
        frames.send_replace(None);
        error(&updates, "Камера недоступна для распознавания");
        tokio::time::sleep(Duration::from_secs(2)).await;
    }
}

async fn detect(
    exe: PathBuf,
    mut frames: watch::Receiver<Option<Arc<Frame>>>,
    updates: watch::Sender<Value>,
) {
    loop {
        let result = async {
            let mut adapter = Adapter::spawn(&exe)?;
            let ready = adapter
                .call(json!({"op":"init"}), Duration::from_secs(180))
                .await?;
            if ready["ok"] != true {
                return Err::<(), Failure>("YOLO adapter initialization failed".into());
            }
            loop {
                frames.changed().await?;
                let frame = frames.borrow_and_update().clone();
                let Some(frame) = frame else { continue };
                if frame.received.elapsed() > MAX_AGE {
                    continue;
                }
                let jpeg = STANDARD.encode(&frame.jpeg);
                let reply = adapter
                    .call(
                        json!({"op":"detect","frame_id":frame.id,"jpeg":jpeg}),
                        Duration::from_secs(30),
                    )
                    .await?;
                let mut data = validate_reply(frame.id, reply)?;
                {
                    // Serialize publication against capture invalidating its source.
                    let current = frames.borrow();
                    if current.is_none() {
                        continue;
                    }
                    if frame.received.elapsed() > MAX_AGE {
                        error(&updates, "Распознавание отстаёт от камеры");
                        continue;
                    }
                    data["jpeg"] = json!(jpeg);
                    updates.send_replace(data);
                }
                tokio::time::sleep(Duration::from_millis(50)).await;
            }
        }
        .await;
        if result.is_err() {
            error(&updates, "YOLO-адаптер недоступен или вернул ошибку");
        }
        tokio::time::sleep(Duration::from_secs(3)).await;
    }
}

async fn expire(updates: watch::Sender<Value>, age: Duration) {
    let mut view = updates.subscribe();
    loop {
        let current = view.borrow_and_update().clone();
        if current["status"] == "live" {
            tokio::select! {
                _ = tokio::time::sleep(age) => {
                    updates.send_if_modified(|value| {
                        if value["status"] == "live" && value["frame_id"] == current["frame_id"] {
                            *value = json!({"status":"error","error":"Нет свежих кадров распознавания"});
                            true
                        } else { false }
                    });
                }
                result = view.changed() => { if result.is_err() { return; } }
            }
        } else if view.changed().await.is_err() {
            return;
        }
    }
}

pub struct Vision {
    tasks: Vec<JoinHandle<()>>,
}
impl Vision {
    pub fn start(camera: Camera, exe: PathBuf) -> (Self, watch::Receiver<Value>) {
        let (updates, rx) = watch::channel(json!({"status":"loading"}));
        let (frames, reader) = watch::channel(None);
        let tasks = vec![
            tokio::spawn(capture(camera, frames, updates.clone())),
            tokio::spawn(expire(updates.clone(), MAX_AGE)),
            tokio::spawn(detect(exe, reader, updates)),
        ];
        (Self { tasks }, rx)
    }
}
impl Drop for Vision {
    fn drop(&mut self) {
        for task in &self.tasks {
            task.abort();
        }
    }
}

pub async fn upgrade(State(app): State<App>, headers: HeaderMap, ws: WebSocketUpgrade) -> Response {
    if !allowed_origin(&headers) {
        return StatusCode::FORBIDDEN.into_response();
    }
    let Some(updates) = app.vision else {
        return StatusCode::NOT_FOUND.into_response();
    };
    ws.max_message_size(4096)
        .max_frame_size(4096)
        .on_upgrade(move |socket| view(socket, updates))
}
async fn view(socket: WebSocket, mut updates: watch::Receiver<Value>) {
    let (mut sink, mut input) = socket.split();
    let send = async {
        loop {
            let value = updates.borrow_and_update().clone();
            tokio::time::timeout(
                Duration::from_secs(2),
                sink.send(Message::Text(
                    json!({"type":"vision","data":value}).to_string().into(),
                )),
            )
            .await??;
            updates.changed().await?;
        }
        #[allow(unreachable_code)]
        Ok::<(), Failure>(())
    };
    let receive = async {
        while let Some(Ok(message)) = input.next().await {
            if matches!(message, Message::Close(_)) {
                break;
            }
        }
    };
    tokio::select! { _=send=>{}, _=receive=>{} }
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    // Same bound the shared parser enforces (session_recording::JPEG).
    const JPEG_LIMIT: usize = 2 * 1024 * 1024;

    // FEATURE: bounded subprocess lifecycle for optional inference.
    // SPEC: hung/oversized replies fail promptly and do not leave a child.
    #[tokio::test]
    async fn adapter_deadline_reply_limit_and_drop_are_bounded() {
        use std::os::unix::fs::PermissionsExt;
        for body in [
            "import time; time.sleep(60)",
            "print('x' * 200000, flush=True)",
        ] {
            let nonce = std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_nanos();
            let path =
                std::env::temp_dir().join(format!("robot-yolo-rpc-{}-{nonce}", std::process::id()));
            std::fs::write(&path, format!("#!/usr/bin/env python3\n{body}\n")).unwrap();
            std::fs::set_permissions(&path, std::fs::Permissions::from_mode(0o700)).unwrap();
            let mut adapter = Adapter::spawn(&path).unwrap();
            let pid = adapter.child.id().unwrap();
            let began = Instant::now();
            assert!(adapter
                .call(json!({"op":"init"}), Duration::from_millis(100))
                .await
                .is_err());
            assert!(began.elapsed() < Duration::from_secs(1));
            drop(adapter);
            tokio::time::timeout(Duration::from_secs(1), async {
                while std::path::Path::new(&format!("/proc/{pid}")).exists() {
                    tokio::time::sleep(Duration::from_millis(10)).await;
                }
            })
            .await
            .expect("adapter child survived drop");
            std::fs::remove_file(path).unwrap();
        }
    }

    // BUG: a late result must not revive a frame after the camera disconnected.
    #[tokio::test]
    async fn late_result_cannot_override_camera_disconnect() {
        use std::os::unix::fs::PermissionsExt;
        let nonce = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos();
        let dir = std::env::temp_dir().join(format!(
            "robot-yolo-disconnect-{}-{nonce}",
            std::process::id()
        ));
        std::fs::create_dir_all(&dir).unwrap();
        let signal = dir.join("detecting");
        let exe = dir.join("adapter");
        let script = format!(
            r#"#!/usr/bin/env python3
import json, sys, time
from pathlib import Path
for line in sys.stdin:
    r=json.loads(line)
    if r['op']=='init': out={{'ok':True}}
    else:
        Path({:?}).write_text('detecting')
        time.sleep(0.5)
        out={{'ok':True,'frame_id':r['frame_id'],'width':640,'height':480,'objects':[],'inference_ms':500}}
    print(json.dumps(out),flush=True)
"#,
            signal.to_str().unwrap()
        );
        std::fs::write(&exe, script).unwrap();
        std::fs::set_permissions(&exe, std::fs::Permissions::from_mode(0o700)).unwrap();
        let (frames, receiver) = watch::channel(Some(Arc::new(Frame {
            id: 12,
            jpeg: vec![1, 2, 3],
            received: Instant::now(),
        })));
        let (updates, mut view) = watch::channel(json!({"status":"loading"}));
        let task = tokio::spawn(detect(exe, receiver, updates.clone()));
        // Mark the initial frame unseen so the worker handles it after init.
        frames.send_modify(|_| {});
        tokio::time::timeout(Duration::from_secs(2), async {
            while !signal.exists() {
                tokio::time::sleep(Duration::from_millis(10)).await;
            }
        })
        .await
        .unwrap();
        frames.send_replace(None);
        error(&updates, "camera disconnected");
        view.borrow_and_update();
        let result = tokio::time::timeout(Duration::from_millis(750), view.changed()).await;
        task.abort();
        let _ = task.await;
        std::fs::remove_dir_all(dir).unwrap();
        assert!(
            result.is_err(),
            "late inference revived the disconnected image: {:?}",
            view.borrow()
        );
    }

    #[tokio::test]
    async fn hung_inference_expires_last_frame_for_new_viewers() {
        let (updates, mut view) =
            watch::channel(json!({"status":"live","frame_id":3,"jpeg":"old"}));
        let task = tokio::spawn(expire(updates, Duration::from_millis(20)));
        tokio::time::timeout(Duration::from_secs(1), view.changed())
            .await
            .unwrap()
            .unwrap();
        assert_eq!(view.borrow()["status"], "error");
        assert!(view.borrow().get("jpeg").is_none());
        task.abort();
    }

    // FEATURE: bounded camera framing and strict adapter replies.
    // SPEC: fragmented JPEGs round-trip; invalid coordinates, IDs and sizes fail.
    #[test]
    fn fragmented_jpegs_ignore_multipart_and_resynchronize() {
        let mut parser = JpegStream::default();
        assert!(parser.feed(b"--frame\r\nheaders\r\n\xff").is_empty());
        assert!(parser.feed(b"\xd8one\xff").is_empty());
        assert_eq!(
            parser.feed(b"\xd9\r\n\xff\xd8two\xff\xd9"),
            vec![
                b"\xff\xd8one\xff\xd9".to_vec(),
                b"\xff\xd8two\xff\xd9".to_vec()
            ]
        );
        let garbage = vec![7; JPEG_LIMIT + 100];
        assert!(parser.feed(&garbage).is_empty());
        assert!(parser.buffered() <= JPEG_LIMIT);
    }

    fn reply() -> Value {
        json!({"ok":true,"frame_id":12,"width":640,"height":480,"inference_ms":3.2,
            "objects":[{"label":"chair","confidence":0.9,"box":[0.1,0.2,0.8,0.9]}]})
    }

    #[test]
    fn reply_validates_frame_identity_and_original_dimensions() {
        let value = validate_reply(12, reply()).unwrap();
        assert_eq!(value["objects"][0]["label"], "chair");
        assert!(validate_reply(13, reply()).is_err());
        let mut bad = reply();
        bad["width"] = json!(0);
        assert!(validate_reply(12, bad).is_err());
    }

    #[test]
    fn reply_rejects_invalid_boxes_confidence_and_overlarge_lists() {
        for bbox in [
            json!([-0.1, 0.2, 0.8, 0.9]),
            json!([0.8, 0.2, 0.1, 0.9]),
            json!([0, 0, 1, 2]),
            json!([0, 0, 1]),
        ] {
            let mut bad = reply();
            bad["objects"][0]["box"] = bbox;
            assert!(validate_reply(12, bad).is_err());
        }
        let mut bad = reply();
        bad["objects"][0]["confidence"] = json!(1.1);
        assert!(validate_reply(12, bad).is_err());
        let mut bad = reply();
        bad["objects"] = json!(vec![reply()["objects"][0].clone(); 101]);
        assert!(validate_reply(12, bad).is_err());
        assert!(validate_reply(12, json!({"ok":false,"error":"failed"})).is_err());
    }
}
