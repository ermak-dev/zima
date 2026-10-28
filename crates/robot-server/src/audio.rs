use axum::{
    extract::{DefaultBodyLimit, Path, State},
    http::{HeaderMap, StatusCode},
    response::{IntoResponse, Response},
    routing::{get, post},
    Json, Router,
};
use serde::Deserialize;
use serde_json::json;
use std::{
    collections::{HashMap, VecDeque},
    sync::{
        atomic::{AtomicU64, Ordering},
        Arc, Mutex,
    },
    time::{Duration, SystemTime, UNIX_EPOCH},
};
use tokio::{
    io::{AsyncBufReadExt, AsyncWriteExt, BufReader},
    net::TcpStream,
    sync::watch,
    time::timeout,
};

#[derive(Clone)]
pub struct Audio(Arc<Settings>, Arc<Mutex<Jobs>>);
struct Settings {
    enabled: bool,
    key: String,
    url: String,
    voice: String,
    address: String,
    token: String,
    volume: f64,
    max_volume: f64,
    client: reqwest::Client,
}
#[derive(Default)]
struct Jobs {
    entries: HashMap<String, Job>,
    order: VecDeque<String>,
    active: Option<String>,
}
struct Job {
    state: &'static str,
    error: Option<&'static str>,
    cancel: watch::Sender<bool>,
}
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Say {
    text: String,
    volume: Option<f64>,
}
fn env(name: &str, fallback: &str) -> String {
    std::env::var(name).unwrap_or_else(|_| fallback.into())
}
impl Audio {
    pub fn new() -> Result<Self, crate::Failure> {
        let volume: f64 = env("ROBOT_AUDIO_VOLUME", "0.35")
            .parse()
            .map_err(|_| "invalid audio volume")?;
        let max_volume: f64 = env("ROBOT_AUDIO_MAX_VOLUME", "0.9")
            .parse()
            .map_err(|_| "invalid audio maximum volume")?;
        if !volume.is_finite()
            || !max_volume.is_finite()
            || !(0.0..=1.0).contains(&max_volume)
            || !(0.0..=max_volume).contains(&volume)
        {
            return Err("invalid audio volume limits".into());
        }
        Ok(Self(
            Arc::new(Settings {
                enabled: env("ROBOT_YANDEX_ENABLED", "false") == "true",
                key: env("ROBOT_YANDEX_API_KEY", ""),
                url: env(
                    "ROBOT_YANDEX_URL",
                    "https://tts.api.cloud.yandex.net/speech/v1/tts:synthesize",
                ),
                voice: env("ROBOT_YANDEX_VOICE", "filipp"),
                address: env("ROBOT_SUPERVISOR_ADDR", "robot:7665"),
                token: env("ROBOT_SUPERVISOR_TOKEN", ""),
                volume,
                max_volume,
                client: reqwest::Client::builder()
                    .no_proxy()
                    .timeout(Duration::from_secs(20))
                    .build()?,
            }),
            Arc::new(Mutex::new(Jobs::default())),
        ))
    }
    fn state(&self, id: &str, state: &'static str, error: Option<&'static str>) {
        if let Some(job) = self.1.lock().unwrap().entries.get_mut(id) {
            job.state = state;
            job.error = error;
        }
    }
    async fn execute(
        &self,
        id: &str,
        text: String,
        volume: f64,
        mut cancel: watch::Receiver<bool>,
    ) {
        self.state(id, "synthesizing", None);
        let pcm = tokio::select! {
            _ = cancel.changed() => { self.finish(id, "cancelled", None); return; },
            result = self.synthesize(&text) => match result {
                Ok(pcm) => pcm,
                Err(error) => { self.finish(id, "failed", Some(error)); return; }
            }
        };
        for segment in pcm.chunks(960_000) {
            if *cancel.borrow() {
                self.finish(id, "cancelled", None);
                return;
            }
            let token = next_id();
            let result = self.play(id, &token, segment, volume, &mut cancel).await;
            match result {
                Ok(()) => {}
                Err((state, error)) => {
                    self.finish(id, state, Some(error));
                    return;
                }
            }
        }
        self.finish(id, "completed", None);
    }
    fn finish(&self, id: &str, state: &'static str, error: Option<&'static str>) {
        let mut jobs = self.1.lock().unwrap();
        if let Some(job) = jobs.entries.get_mut(id) {
            job.state = state;
            job.error = error;
        }
        if jobs.active.as_deref() == Some(id) {
            jobs.active = None;
        }
    }
    async fn synthesize(&self, text: &str) -> Result<Vec<u8>, &'static str> {
        let mut url = reqwest::Url::parse(&self.0.url).map_err(|_| "tts_configuration_invalid")?;
        // Credentials must never travel to an arbitrary plaintext endpoint.
        if url.scheme() != "https"
            && !(url.scheme() == "http"
                && matches!(url.host_str(), Some("127.0.0.1" | "localhost" | "[::1]")))
        {
            return Err("tts_configuration_invalid");
        }
        url.set_fragment(None);
        let mut body = reqwest::Url::parse("http://localhost/").unwrap();
        body.query_pairs_mut()
            .append_pair("text", text)
            .append_pair("voice", &self.0.voice)
            .append_pair("format", "lpcm")
            .append_pair("sampleRateHertz", "16000");
        let mut response = self
            .0
            .client
            .post(url)
            .header("authorization", format!("Api-Key {}", self.0.key))
            .header("content-type", "application/x-www-form-urlencoded")
            .body(body.query().unwrap().to_owned())
            .send()
            .await
            .map_err(|_| "tts_unreachable")?;
        if !response.status().is_success() {
            return Err("tts_provider_error");
        }
        let mut pcm = Vec::new();
        while let Some(chunk) = response
            .chunk()
            .await
            .map_err(|_| "tts_response_incomplete")?
        {
            if pcm.len() + chunk.len() > 4_000_000 {
                return Err("tts_audio_too_large");
            }
            pcm.extend_from_slice(&chunk);
        }
        if pcm.is_empty() || pcm.len() % 2 != 0 {
            return Err("tts_audio_invalid");
        }
        Ok(pcm)
    }
    async fn connect(&self) -> Result<BufReader<TcpStream>, &'static str> {
        let stream = timeout(Duration::from_secs(5), TcpStream::connect(&self.0.address))
            .await
            .map_err(|_| "robot_unreachable")?
            .map_err(|_| "robot_unreachable")?;
        let mut stream = stream;
        authenticate(&mut stream, &self.0.token).await?;
        Ok(BufReader::new(stream))
    }
    async fn cancel_playback(&self, token: &str) -> bool {
        let operation = async {
            let mut stream = self.connect().await.ok()?;
            stream
                .get_mut()
                .write_all(format!("CANCEL {token}\n").as_bytes())
                .await
                .ok()?;
            Some(read_line(&mut stream).await.ok()? == "CANCELLED")
        };
        timeout(Duration::from_secs(5), operation)
            .await
            .ok()
            .flatten()
            .unwrap_or(false)
    }
    async fn play(
        &self,
        id: &str,
        token: &str,
        pcm: &[u8],
        volume: f64,
        cancel: &mut watch::Receiver<bool>,
    ) -> Result<(), (&'static str, &'static str)> {
        let mut stream = self.connect().await.map_err(|e| ("failed", e))?;
        if *cancel.borrow() {
            return Err(("cancelled", "cancelled"));
        }
        let wav = wav(pcm);
        let header = format!("AUDIO {volume:.6} {token} {}\n", wav.len());
        let send = async {
            stream.get_mut().write_all(header.as_bytes()).await?;
            stream.get_mut().write_all(&wav).await
        };
        timeout(Duration::from_secs(10), send)
            .await
            .map_err(|_| ("unknown", "robot_send_timeout"))?
            .map_err(|_| ("unknown", "robot_send_failed"))?;
        let deadline = tokio::time::Instant::now() + Duration::from_secs(40);
        let mut started = false;
        let mut cancellation_sent = false;
        loop {
            let line = tokio::select! {
                _ = cancel.changed(), if !cancellation_sent => {
                    cancellation_sent = true;
                    if !self.cancel_playback(token).await { return Err(("unknown", "cancellation_unconfirmed")); }
                    continue;
                },
                line = tokio::time::timeout_at(deadline, read_line(&mut stream)) => line.map_err(|_| ("unknown", "playback_timeout"))?
                    .map_err(|_| ("unknown", "playback_unconfirmed"))?,
            };
            match line.as_str() {
                "STARTED" => {
                    started = true;
                    self.state(id, "playing", None);
                }
                "COMPLETED" if started => return Ok(()),
                "CANCELLED" => return Err(("cancelled", "cancelled")),
                "BUSY" => return Err(("failed", "robot_audio_busy")),
                "ERROR" | "UNAVAILABLE" | "INVALID_AUDIO" => {
                    return Err(("failed", "robot_audio_failed"))
                }
                "RESTORE_FAILED" => return Err(("unknown", "mixer_restore_failed")),
                "ALIVE" => {}
                line if line.starts_with("WORKER ") => {}
                _ => return Err(("unknown", "invalid_robot_response")),
            }
        }
    }
}
pub async fn authenticate(stream: &mut TcpStream, token: &str) -> Result<(), &'static str> {
    if token.is_empty() {
        return Ok(());
    }
    if token.len() > 128 || token.chars().any(char::is_whitespace) {
        return Err("robot_auth_configuration_invalid");
    }
    let operation = async {
        stream
            .write_all(format!("AUTH {token}\n").as_bytes())
            .await
            .map_err(|_| "robot_auth_failed")?;
        let mut reader = BufReader::new(stream);
        if read_line(&mut reader).await? != "AUTHENTICATED" {
            return Err("robot_auth_failed");
        }
        Ok(())
    };
    timeout(Duration::from_secs(5), operation)
        .await
        .map_err(|_| "robot_auth_timeout")?
}
async fn read_line<T: tokio::io::AsyncRead + Unpin>(
    stream: &mut BufReader<T>,
) -> Result<String, &'static str> {
    let mut line = Vec::new();
    loop {
        let data = stream.fill_buf().await.map_err(|_| "read_failed")?;
        if data.is_empty() {
            return Err("connection_closed");
        }
        let n = data
            .iter()
            .position(|b| *b == b'\n')
            .map(|n| n + 1)
            .unwrap_or(data.len());
        if line.len() + n > 256 {
            return Err("response_too_large");
        }
        line.extend_from_slice(&data[..n]);
        stream.consume(n);
        if line.last() == Some(&b'\n') {
            line.pop();
            return String::from_utf8(line).map_err(|_| "invalid_response");
        }
    }
}
fn wav(pcm: &[u8]) -> Vec<u8> {
    let mut wav = Vec::with_capacity(44 + pcm.len());
    wav.extend_from_slice(b"RIFF");
    wav.extend_from_slice(&(36 + pcm.len() as u32).to_le_bytes());
    wav.extend_from_slice(b"WAVEfmt \x10\0\0\0\x01\0\x01\0");
    wav.extend_from_slice(&16000u32.to_le_bytes());
    wav.extend_from_slice(&32000u32.to_le_bytes());
    wav.extend_from_slice(b"\x02\0\x10\0data");
    wav.extend_from_slice(&(pcm.len() as u32).to_le_bytes());
    wav.extend_from_slice(pcm);
    wav
}
fn next_id() -> String {
    static IDS: AtomicU64 = AtomicU64::new(1);
    let now = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_nanos() as u64;
    format!("{now:016x}{:016x}", IDS.fetch_add(1, Ordering::Relaxed))
}
fn error(code: StatusCode, message: &str) -> Response {
    (code, Json(json!({"error":message}))).into_response()
}
async fn say(State(audio): State<Audio>, headers: HeaderMap, Json(request): Json<Say>) -> Response {
    if !crate::allowed_origin(&headers) {
        return error(StatusCode::FORBIDDEN, "foreign_origin");
    }
    let volume = request.volume.unwrap_or(audio.0.volume);
    if request.text.trim().is_empty()
        || request.text.chars().count() > 2000
        || request.text.contains('\0')
        || !volume.is_finite()
        || !(0.0..=audio.0.max_volume).contains(&volume)
    {
        return error(StatusCode::BAD_REQUEST, "invalid_text_or_volume");
    }
    if !audio.0.enabled {
        return error(StatusCode::SERVICE_UNAVAILABLE, "cloud_tts_disabled");
    }
    if audio.0.key.is_empty() {
        return error(
            StatusCode::SERVICE_UNAVAILABLE,
            "yandex_api_key_not_configured",
        );
    }
    let id = next_id();
    let (cancel, receiver) = watch::channel(false);
    {
        let mut jobs = audio.1.lock().unwrap();
        if jobs.active.is_some() {
            return error(StatusCode::CONFLICT, "audio_busy");
        }
        while jobs.order.len() >= 64 {
            if let Some(old) = jobs.order.pop_front() {
                jobs.entries.remove(&old);
            }
        }
        jobs.entries.insert(
            id.clone(),
            Job {
                state: "queued",
                error: None,
                cancel,
            },
        );
        jobs.order.push_back(id.clone());
        jobs.active = Some(id.clone());
    }
    let task_id = id.clone();
    tokio::spawn(async move {
        audio
            .execute(&task_id, request.text, volume, receiver)
            .await;
    });
    (
        StatusCode::ACCEPTED,
        Json(json!({"id":id,"state":"queued"})),
    )
        .into_response()
}
async fn task(State(audio): State<Audio>, Path(id): Path<String>) -> Response {
    let jobs = audio.1.lock().unwrap();
    match jobs.entries.get(&id) {
        Some(job) => Json(json!({"id":id,"state":job.state,"error":job.error})).into_response(),
        None => error(StatusCode::NOT_FOUND, "task_not_found"),
    }
}
async fn cancel(
    State(audio): State<Audio>,
    Path(id): Path<String>,
    headers: HeaderMap,
) -> Response {
    if !crate::allowed_origin(&headers) {
        return error(StatusCode::FORBIDDEN, "foreign_origin");
    }
    let jobs = audio.1.lock().unwrap();
    match jobs.entries.get(&id) {
        Some(job) => {
            if jobs.active.as_deref() == Some(&id) {
                let _ = job.cancel.send(true);
            }
            (
                StatusCode::ACCEPTED,
                Json(json!({"id":id,"state":job.state})),
            )
                .into_response()
        }
        None => error(StatusCode::NOT_FOUND, "task_not_found"),
    }
}
async fn settings(State(audio): State<Audio>) -> Json<serde_json::Value> {
    Json(
        json!({"enabled":audio.0.enabled && !audio.0.key.is_empty(),"volume":audio.0.volume,"max_volume":audio.0.max_volume}),
    )
}
pub fn router(audio: Audio) -> Router {
    Router::new()
        .route("/api/audio/config", get(settings))
        .route("/api/audio/say", post(say))
        .route("/api/audio/tasks/{id}", get(task))
        .route("/api/audio/tasks/{id}/cancel", post(cancel))
        .layer(DefaultBodyLimit::max(16384))
        .with_state(audio)
}
