// FEATURE: one TTS HTTP API for the panel and CLI, independent of chassis activation.
// SPEC: bounded validated requests, same-origin writes, server-side credentials,
// asynchronous completion receipts, exclusive playback and no retries after uncertainty.
// IMPLEMENTATION: host synthesis and authenticated streaming to the persistent robot server.
// RESULT: IMPLEMENTED — synthesis, cancellation, limits, settings and uncertain outcomes verified.
// STATUS: DONE
use axum::{body::Bytes, http::HeaderMap, routing::post, Router};
use serde_json::{json, Value};
use std::process::{Child, Command, Stdio};
use std::sync::{Arc, Mutex};
use std::time::Duration;
use tokio::io::{AsyncBufReadExt, AsyncReadExt, AsyncWriteExt, BufReader};
use tokio::net::{TcpListener, TcpStream};
use tokio::time::{sleep, timeout};

trait JsonResponse {
    async fn json(self) -> Result<Value, Box<dyn std::error::Error>>;
}
impl JsonResponse for reqwest::Response {
    async fn json(self) -> Result<Value, Box<dyn std::error::Error>> {
        Ok(serde_json::from_str(&self.text().await?)?)
    }
}

struct Server {
    child: Child,
    url: String,
    tasks: Vec<tokio::task::JoinHandle<()>>,
    speech: Arc<Mutex<Vec<(String, String)>>>,
    audio: Arc<Mutex<Vec<Vec<u8>>>>,
}
impl Drop for Server {
    fn drop(&mut self) {
        let _ = self.child.kill();
        let _ = self.child.wait();
        for task in &self.tasks {
            task.abort();
        }
    }
}
impl Server {
    async fn start(outcome: &'static str, slow_tts: bool) -> Self {
        Self::start_enabled(outcome, slow_tts, true).await
    }
    async fn start_enabled(outcome: &'static str, slow_tts: bool, enabled: bool) -> Self {
        let speech = Arc::new(Mutex::new(Vec::new()));
        let seen = speech.clone();
        let provider = TcpListener::bind("127.0.0.1:0").await.unwrap();
        let provider_url = format!("http://{}/tts", provider.local_addr().unwrap());
        let route = Router::new().route(
            "/tts",
            post(move |headers: HeaderMap, body: Bytes| {
                let seen = seen.clone();
                async move {
                    seen.lock().unwrap().push((
                        headers["authorization"].to_str().unwrap().into(),
                        String::from_utf8(body.to_vec()).unwrap(),
                    ));
                    if slow_tts {
                        sleep(Duration::from_secs(10)).await;
                    }
                    vec![20u8; 3200]
                }
            }),
        );
        let provider_task = tokio::spawn(async move {
            axum::serve(provider, route).await.unwrap();
        });
        let supervisor = TcpListener::bind("127.0.0.1:0").await.unwrap();
        let address = supervisor.local_addr().unwrap().to_string();
        let audio = Arc::new(Mutex::new(Vec::new()));
        let recorded = audio.clone();
        let cancelled = Arc::new(tokio::sync::Notify::new());
        let supervisor_task = tokio::spawn(async move {
            loop {
                let (stream, _) = supervisor.accept().await.unwrap();
                let recorded = recorded.clone();
                let cancelled = cancelled.clone();
                tokio::spawn(async move {
                    let mut stream = BufReader::new(stream);
                    let mut line = String::new();
                    stream.read_line(&mut line).await.unwrap();
                    assert_eq!(line, "AUTH test-supervisor-key\n");
                    stream
                        .get_mut()
                        .write_all(b"AUTHENTICATED\n")
                        .await
                        .unwrap();
                    line.clear();
                    stream.read_line(&mut line).await.unwrap();
                    let words: Vec<_> = line.split_whitespace().collect();
                    if words[0] == "CANCEL" {
                        assert_eq!(words[1].len(), 32);
                        cancelled.notify_one();
                        stream.get_mut().write_all(b"CANCELLED\n").await.unwrap();
                        return;
                    }
                    assert_eq!(words[0], "AUDIO");
                    assert_eq!(words[1], "0.400000");
                    assert_eq!(words[2].len(), 32);
                    let length: usize = words[3].parse().unwrap();
                    let mut wav = vec![0; length];
                    stream.read_exact(&mut wav).await.unwrap();
                    recorded.lock().unwrap().push(wav);
                    stream.get_mut().write_all(b"STARTED\n").await.unwrap();
                    if outcome == "WAIT_CANCEL" {
                        cancelled.notified().await;
                        stream.get_mut().write_all(b"CANCELLED\n").await.unwrap();
                    } else if outcome != "disconnect" {
                        stream
                            .get_mut()
                            .write_all(format!("{outcome}\n").as_bytes())
                            .await
                            .unwrap();
                    }
                });
            }
        });
        let http = TcpListener::bind("127.0.0.1:0").await.unwrap();
        let port = http.local_addr().unwrap().port();
        drop(http);
        let child = Command::new(env!("CARGO_BIN_EXE_robot-server"))
            .args([
                "--audio-only",
                "--listen",
                "127.0.0.1",
                "--port",
                &port.to_string(),
            ])
            .env_remove("ROBOT_SERVER_HEARTBEAT")
            .env("ROBOT_SUPERVISOR_ADDR", address)
            .env("ROBOT_SUPERVISOR_TOKEN", "test-supervisor-key")
            .env(
                "ROBOT_YANDEX_ENABLED",
                if enabled { "true" } else { "false" },
            )
            .env("ROBOT_YANDEX_API_KEY", "test-yandex-key")
            .env("ROBOT_YANDEX_URL", provider_url)
            .env("ROBOT_AUDIO_MAX_VOLUME", "0.9")
            .stdout(Stdio::null())
            .stderr(Stdio::null())
            .spawn()
            .unwrap();
        let mut server = Self {
            child,
            url: format!("http://127.0.0.1:{port}"),
            tasks: vec![provider_task, supervisor_task],
            speech,
            audio,
        };
        timeout(Duration::from_secs(5), async {
            loop {
                assert!(
                    server.child.try_wait().unwrap().is_none(),
                    "audio-only server must start without Player or controller activation"
                );
                if TcpStream::connect(("127.0.0.1", port)).await.is_ok() {
                    break;
                }
                sleep(Duration::from_millis(20)).await;
            }
        })
        .await
        .expect("audio API should become ready");
        server
    }
    async fn say(&self, body: Value) -> reqwest::Response {
        reqwest::Client::new()
            .post(format!("{}/api/audio/say", self.url))
            .header("content-type", "application/json")
            .body(body.to_string())
            .send()
            .await
            .unwrap()
    }
    async fn task(&self, id: &str) -> Value {
        reqwest::get(format!("{}/api/audio/tasks/{id}", self.url))
            .await
            .unwrap()
            .json()
            .await
            .unwrap()
    }
    async fn terminal(&self, id: &str) -> Value {
        timeout(Duration::from_secs(5), async {
            loop {
                let value = self.task(id).await;
                if matches!(
                    value["state"].as_str(),
                    Some("completed" | "failed" | "unknown" | "cancelled")
                ) {
                    return value;
                }
                sleep(Duration::from_millis(20)).await;
            }
        })
        .await
        .unwrap()
    }
}

#[tokio::test]
async fn audio_settings_expose_limits_but_never_credentials() {
    let server = Server::start("COMPLETED", false).await;
    let response = reqwest::get(format!("{}/api/audio/config", server.url))
        .await
        .unwrap();
    assert_eq!(response.status(), 200);
    let value = response.text().await.unwrap();
    assert!(!value.contains("test-yandex-key"));
    assert!(!value.contains("test-supervisor-key"));
    let value: Value = serde_json::from_str(&value).unwrap();
    assert_eq!(
        value,
        json!({"enabled":true,"volume":0.35,"max_volume":0.9})
    );
}

#[tokio::test]
async fn configured_key_does_not_silently_enable_cloud_synthesis() {
    let server = Server::start_enabled("COMPLETED", false, false).await;
    assert_eq!(server.say(json!({"text":"hi"})).await.status(), 503);
    assert!(server.speech.lock().unwrap().is_empty());
}

#[tokio::test]
async fn tts_api_synthesizes_on_host_and_waits_for_robot_completion() {
    let server = Server::start("COMPLETED", false).await;
    let response = server
        .say(json!({"text":"Привет, робот", "volume":0.4}))
        .await;
    assert_eq!(response.status(), 202);
    let value: Value = response.json().await.unwrap();
    let id = value["id"].as_str().unwrap();
    assert_eq!(server.terminal(id).await["state"], "completed");
    let speech = server.speech.lock().unwrap();
    assert_eq!(speech.len(), 1);
    assert_eq!(speech[0].0, "Api-Key test-yandex-key");
    assert!(speech[0].1.contains("voice=filipp"));
    assert!(!speech[0].1.contains("folder_id"));
    let audio = server.audio.lock().unwrap();
    assert_eq!(audio.len(), 1);
    assert_eq!(&audio[0][..4], b"RIFF");
}

#[tokio::test]
async fn tts_api_rejects_invalid_requests_and_foreign_origins_without_synthesis() {
    let server = Server::start("COMPLETED", false).await;
    for body in [
        json!({"text":" "}),
        json!({"text":"x".repeat(2001)}),
        json!({"text":"\u{0}"}),
        json!({"text":"hi","volume":1.1}),
        json!({"text":"hi","volume":0.95}),
        json!({"text":"hi","api_key":"must-not-be-accepted"}),
    ] {
        assert!(server.say(body).await.status().is_client_error());
    }
    let response = reqwest::Client::new()
        .post(format!("{}/api/audio/say", server.url))
        .header("origin", "https://foreign.example")
        .header("content-type", "application/json")
        .body(json!({"text":"hi"}).to_string())
        .send()
        .await
        .unwrap();
    assert_eq!(response.status(), 403);
    assert!(server.speech.lock().unwrap().is_empty());
}

#[tokio::test]
async fn tts_api_reports_uncertain_playback_without_retry() {
    let server = Server::start("disconnect", false).await;
    let value: Value = server
        .say(json!({"text":"hi","volume":0.4}))
        .await
        .json()
        .await
        .unwrap();
    assert_eq!(
        server.terminal(value["id"].as_str().unwrap()).await["state"],
        "unknown"
    );
    assert_eq!(server.audio.lock().unwrap().len(), 1);
    assert_eq!(server.speech.lock().unwrap().len(), 1);
}

#[tokio::test]
async fn tts_api_cancels_running_robot_audio_with_confirmed_receipt() {
    let server = Server::start("WAIT_CANCEL", false).await;
    let value: Value = server
        .say(json!({"text":"hi","volume":0.4}))
        .await
        .json()
        .await
        .unwrap();
    let id = value["id"].as_str().unwrap();
    timeout(Duration::from_secs(3), async {
        while server.task(id).await["state"] != "playing" {
            sleep(Duration::from_millis(10)).await;
        }
    })
    .await
    .unwrap();
    let response = reqwest::Client::new()
        .post(format!("{}/api/audio/tasks/{id}/cancel", server.url))
        .send()
        .await
        .unwrap();
    assert_eq!(response.status(), 202);
    assert_eq!(server.terminal(id).await["state"], "cancelled");
    assert_eq!(server.audio.lock().unwrap().len(), 1);
}

#[tokio::test]
async fn tts_api_cancels_synthesis_and_rejects_overlapping_jobs() {
    let server = Server::start("COMPLETED", true).await;
    let value: Value = server
        .say(json!({"text":"hi","volume":0.4}))
        .await
        .json()
        .await
        .unwrap();
    let id = value["id"].as_str().unwrap();
    assert_eq!(server.say(json!({"text":"second"})).await.status(), 409);
    let response = reqwest::Client::new()
        .post(format!("{}/api/audio/tasks/{id}/cancel", server.url))
        .send()
        .await
        .unwrap();
    assert!(response.status().is_success());
    assert_eq!(server.terminal(id).await["state"], "cancelled");
    assert!(server.audio.lock().unwrap().is_empty());
    assert_eq!(
        reqwest::get(format!("{}/api/audio/tasks/absent", server.url))
            .await
            .unwrap()
            .status(),
        404
    );
}
