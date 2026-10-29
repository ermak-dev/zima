mod audio;
mod camera;
mod compass;
mod player;
mod safety;
mod slam;
mod telemetry;
mod vision;

use axum::{
    extract::{
        ws::{Message, WebSocket},
        State, WebSocketUpgrade,
    },
    http::{HeaderMap, StatusCode},
    response::{Html, IntoResponse, Response},
    routing::get,
    Router,
};
use futures_util::{SinkExt, StreamExt};
use robot_server::{
    player_recording::Recorder,
    session_recording::{start_sources, Session as SessionRecording},
};
use safety::Safety;
use serde::Deserialize;
use serde_json::{json, Value};
use std::{
    path::PathBuf,
    sync::{
        atomic::{AtomicU64, Ordering},
        Arc, Mutex,
    },
    time::Duration,
};
use tokio::{
    io::AsyncWriteExt,
    net::{TcpListener, TcpStream},
    sync::watch,
};

type Failure = Box<dyn std::error::Error + Send + Sync>;
#[derive(Clone)]
struct Config {
    player_host: String,
    player_port: u16,
    listen: String,
    port: u16,
    camera_url: String,
    netrc: PathBuf,
    slam_adapter: Option<PathBuf>,
    yolo_adapter: Option<PathBuf>,
    audio_only: bool,
}
impl Config {
    fn parse() -> Result<Option<Self>, Failure> {
        let mut c = Self {
            player_host: "robot".into(),
            player_port: 6665,
            listen: "127.0.0.1".into(),
            port: 8085,
            camera_url: std::env::var("ROBOT_CAMERA_URL")
                .unwrap_or_else(|_| "http://cam/mjpeg".into()),
            netrc: std::env::var_os("ROBOT_CAMERA_NETRC")
                .map(PathBuf::from)
                .unwrap_or_else(|| {
                    PathBuf::from(std::env::var_os("HOME").unwrap_or_default())
                        .join(".config/robot/camera.netrc")
                }),
            slam_adapter: None,
            yolo_adapter: None,
            audio_only: false,
        };
        let mut args = std::env::args().skip(1);
        while let Some(a) = args.next() {
            if a == "--audio-only" {
                c.audio_only = true;
                continue;
            }
            if a == "--help" {
                println!("robot-server\n\n--player-host HOST  Player host (robot)\n--player-port PORT  Player port (6665)\n--listen HOST       HTTP bind host (127.0.0.1)\n--port PORT         HTTP port (8085)\n--camera-url URL    MJPEG URL (http://cam/mjpeg)\n--netrc PATH        Camera credentials (~/.config/robot/camera.netrc)\n--slam-adapter EXE  Optional MRPT SLAM adapter (line JSON over stdio)\n--yolo-adapter EXE  Optional camera object detector (line JSON over stdio)\n--audio-only       Serve HTTP/TTS without Player or controller handoff\n--help             Show help");
                return Ok(None);
            }
            let v = args.next().ok_or("option requires a value")?;
            match a.as_str() {
                "--player-host" => c.player_host = v,
                "--player-port" => c.player_port = v.parse()?,
                "--listen" => c.listen = v,
                "--port" => c.port = v.parse()?,
                "--camera-url" => c.camera_url = v,
                "--netrc" => c.netrc = v.into(),
                "--slam-adapter" => {
                    let p = PathBuf::from(&v);
                    if !p.is_file() {
                        return Err(format!("--slam-adapter: {v} is not a file").into());
                    }
                    c.slam_adapter = Some(p);
                }
                "--yolo-adapter" => {
                    let p = PathBuf::from(&v);
                    if !p.is_file() {
                        return Err(format!("--yolo-adapter: {v} is not a file").into());
                    }
                    #[cfg(unix)]
                    {
                        use std::os::unix::fs::PermissionsExt;
                        if p.metadata()?.permissions().mode() & 0o111 == 0 {
                            return Err("YOLO adapter must be executable".into());
                        }
                    }
                    c.yolo_adapter = Some(p);
                }
                _ => return Err("unknown option (see --help)".into()),
            }
        }
        Ok(Some(c))
    }
}
#[derive(Clone)]
struct App {
    safety: Arc<Mutex<Safety>>,
    state: watch::Receiver<Value>,
    states: watch::Sender<Value>,
    scan: watch::Receiver<Option<Value>>,
    camera: camera::Camera,
    vision: Option<watch::Receiver<Value>>,
    ids: Arc<AtomicU64>,
    recording: Option<Arc<SessionRecording>>,
}
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Command {
    v: f64,
    w: f64,
}
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct ResetCommand {
    #[serde(rename = "type")]
    _kind: ResetKind,
}
#[derive(Deserialize)]
enum ResetKind {
    #[serde(rename = "reset_blocked")]
    ResetBlocked,
}
async fn index() -> Response {
    let path = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../web/panel.html");
    match tokio::fs::read_to_string(path).await {
        Ok(s) => Html(s).into_response(),
        Err(_) => (StatusCode::SERVICE_UNAVAILABLE, "panel bundle unavailable").into_response(),
    }
}
fn allowed_origin(headers: &HeaderMap) -> bool {
    let Some(origin) = headers.get("origin") else {
        return true;
    };
    origin
        .to_str()
        .ok()
        .and_then(|s| reqwest::Url::parse(s).ok())
        .is_some_and(|u| {
            matches!(u.scheme(), "http" | "https")
                && headers
                    .get("host")
                    .and_then(|h| h.to_str().ok())
                    .is_some_and(|h| {
                        reqwest::Url::parse(&format!("{}://{h}", u.scheme()))
                            .is_ok_and(|host| host.origin() == u.origin())
                    })
        })
}
async fn upgrade(State(app): State<App>, headers: HeaderMap, ws: WebSocketUpgrade) -> Response {
    // Do not let a foreign web page drive a localhost session using browser WS.
    if !allowed_origin(&headers) {
        return StatusCode::FORBIDDEN.into_response();
    }
    ws.max_message_size(4096)
        .max_frame_size(4096)
        .on_upgrade(move |socket| session(socket, app))
}
async fn send_frame(
    sink: &mut futures_util::stream::SplitSink<WebSocket, Message>,
    kind: &str,
    data: Value,
) -> Result<(), Failure> {
    tokio::time::timeout(
        Duration::from_secs(2),
        sink.send(Message::Text(
            json!({"type":kind,"data":data}).to_string().into(),
        )),
    )
    .await??;
    Ok(())
}
struct DriverGuard {
    id: u64,
    safety: Arc<Mutex<Safety>>,
}
impl Drop for DriverGuard {
    fn drop(&mut self) {
        self.safety.lock().unwrap().disconnect(self.id);
    }
}
async fn session(socket: WebSocket, app: App) {
    let id = app.ids.fetch_add(1, Ordering::Relaxed);
    let _guard = DriverGuard {
        id,
        safety: app.safety.clone(),
    };
    let (mut output, mut input) = socket.split();
    let mut state = app.state;
    let mut scan = app.scan;
    // Independent send/receive futures: a slow viewer never delays a disconnect
    // or command lease. watch channels retain only the latest state and scan.
    let receive = async {
        while let Some(Ok(m)) = input.next().await {
            match m {
                Message::Text(text) => {
                    if let Ok(c) = serde_json::from_str::<Command>(&text) {
                        if c.v.is_finite() && c.w.is_finite() {
                            if let Some(r) = &app.recording {
                                r.command(id, Some((c.v, c.w)), std::time::Instant::now());
                            }
                            app.safety.lock().unwrap().command(
                                id,
                                c.v,
                                c.w,
                                std::time::Instant::now(),
                            );
                        }
                    } else if serde_json::from_str::<ResetCommand>(&text).is_ok() {
                        if let Some(r) = &app.recording {
                            r.command(id, None, std::time::Instant::now());
                        }
                        let mut safety = app.safety.lock().unwrap();
                        if safety.reset_blocked() {
                            app.states
                                .send_modify(|state| state["blocked"] = json!(false));
                        }
                    }
                }
                Message::Close(_) => break,
                _ => {}
            }
        }
    };
    let send = async {
        let initial = state.borrow_and_update().clone();
        send_frame(&mut output, "state", initial).await?;
        send_frame(
            &mut output,
            "vision",
            json!({"enabled": app.vision.is_some()}),
        )
        .await?;
        let initial = scan.borrow_and_update().clone();
        if let Some(v) = initial {
            send_frame(&mut output, "scan", v).await?;
        }
        loop {
            tokio::select! {
             r=state.changed()=>{r?;let v=state.borrow_and_update().clone();send_frame(&mut output,"state",v).await?;},
             r=scan.changed()=>{r?;let v=scan.borrow_and_update().clone();if let Some(v)=v{send_frame(&mut output,"scan",v).await?;}}
            }
        }
        #[allow(unreachable_code)]
        Ok::<(), Failure>(())
    };
    tokio::select! {_=receive=>{},_=send=>{}}
}
async fn shutdown_signal() {
    #[cfg(unix)]
    {
        let mut terminate =
            tokio::signal::unix::signal(tokio::signal::unix::SignalKind::terminate())
                .expect("signal handler");
        tokio::select! {_=tokio::signal::ctrl_c()=>{},_=terminate.recv()=>{}}
    }
    #[cfg(not(unix))]
    {
        let _ = tokio::signal::ctrl_c().await;
    }
}
async fn heartbeat(mut stream: TcpStream) -> Result<(), Failure> {
    // Read EOF as well as write failures: a closed supervisor must terminate
    // the session even before TCP's next write notices it. Never reconnect.
    let (mut rd, mut wr) = stream.split();
    let mut byte = [0; 1];
    loop {
        tokio::select! {
         result=tokio::io::AsyncReadExt::read(&mut rd,&mut byte)=>{if result? == 0 { return Err("supervisor heartbeat connection lost".into()); }},
         _=tokio::time::sleep(Duration::from_secs(1))=>{tokio::time::timeout(Duration::from_secs(1),wr.write_all(b"hb")).await??;}
        }
    }
}
async fn run(c: Config) -> Result<(), Failure> {
    // File preflight must precede ACTIVATE (the CLI's earlier handoff is separate).
    let recording = if c.audio_only {
        None
    } else {
        SessionRecording::from_env()?
    };
    let recorder = if c.audio_only {
        None
    } else if let Some(r) = &recording {
        Some(r.raw.clone())
    } else {
        Recorder::from_env()?
    };
    let result = run_session(c, recorder.clone(), recording.clone()).await;
    if let Some(recording) = recording {
        let report = recording.finish(Duration::from_secs(2), result.is_ok());
        eprintln!("robot-server: session recording: {report}");
        return result;
    }
    // run_session has stopped command producers, attempted zero where connected,
    // and dropped heartbeat before any bounded wait on the detached disk writer.
    if let Some(recorder) = recorder {
        let stats = recorder.finish(Duration::from_secs(2));
        if stats.complete {
            eprintln!("robot-server: Player recording complete: {stats:?}");
        } else {
            eprintln!("robot-server: Player recording INCOMPLETE: {stats:?}");
        }
    }
    result
}
async fn run_session(
    c: Config,
    recorder: Option<Arc<Recorder>>,
    recording: Option<Arc<SessionRecording>>,
) -> Result<(), Failure> {
    let audio = audio::Audio::new()?;
    if c.audio_only {
        let listener = TcpListener::bind((c.listen.as_str(), c.port)).await?;
        let router = Router::new()
            .route("/", get(index))
            .merge(audio::router(audio));
        axum::serve(listener, router)
            .with_graceful_shutdown(shutdown_signal())
            .await?;
        return Ok(());
    }
    let camera = camera::Camera::new(&c)?;
    let mut supervisor = if let Ok(addr) = std::env::var("ROBOT_SERVER_HEARTBEAT") {
        let mut stream = tokio::time::timeout(Duration::from_secs(5), TcpStream::connect(addr))
            .await
            .map_err(|_| "supervisor connection timed out")?
            .map_err(|_| "supervisor connection failed")?;
        audio::authenticate(
            &mut stream,
            &std::env::var("ROBOT_SUPERVISOR_TOKEN").unwrap_or_default(),
        )
        .await?;
        tokio::time::timeout(Duration::from_secs(1), stream.write_all(b"ACTIVATE\n")).await??;
        Some(stream)
    } else {
        None
    };
    let safety = Arc::new(Mutex::new(Safety::new()));
    let mut initial = telemetry::Telemetry::new().json();
    initial["blocked"] = json!(false);
    let (states, state) = watch::channel(initial);
    let (scans, scan) = watch::channel(None);
    let _recording_tasks = recording.as_ref().map(|r| {
        let source = camera.clone();
        start_sources(
            r.clone(),
            // The camera shares LDS power and needs about a minute to boot;
            // retry the stream within a bounded budget instead of marking it
            // unavailable from the first failed attempt.
            move || {
                let source = source.clone();
                async move { source.stream().await }
            },
            state.clone(),
            scan.clone(),
        )
    });
    // SLAM observations come from Player; results are display data, not drive gates.
    let slam_tx = if let Some(exe) = c.slam_adapter.clone() {
        // Without a saved map the only meaningful mode is mapping; with one,
        // localize unless mapping is explicitly requested.
        let map_path = std::env::var("ROBOT_SLAM_MAP").ok();
        let mapping = map_path.is_none() || std::env::var_os("ROBOT_SLAM_MAPPING").is_some();
        let states_tx = states.clone();
        match slam::SlamThread::start(
            exe,
            mapping,
            map_path,
            Box::new(move |slam_state| {
                states_tx.send_if_modified(|old| {
                    let mut next = old.clone();
                    next["slam"] = slam_state.clone();
                    if *old == next {
                        false
                    } else {
                        *old = next;
                        true
                    }
                });
            }),
        ) {
            Ok(t) => Some(t.observe),
            Err(e) => {
                return Err(format!("SLAM adapter failed to start: {e}").into());
            }
        }
    } else {
        None
    };
    let mut hb = tokio::spawn(async move {
        if let Some(s) = supervisor.take() {
            heartbeat(s).await
        } else {
            std::future::pending().await
        }
    });
    let processor = player::DataProcessor::new(states.clone(), scans, safety.clone(), slam_tx);
    let connected = tokio::select! {
     r=player::connect(&c, recorder, processor)=>r,
     r=&mut hb=>return r.map_err(|e|->Failure{e.into()}).and_then(|r|r),
    };
    let (reader, mut writer) = match connected {
        Ok(c) => c,
        Err(e) => {
            hb.abort();
            let _ = hb.await;
            return Err(e);
        }
    };
    let mut read_task = tokio::spawn(player::read(reader));
    let mut drive_task = tokio::spawn(player::drive(writer.clone(), safety.clone()));
    let _compass_task = compass::start(states.clone());
    let (_vision_tasks, vision) = if let Some(exe) = c.yolo_adapter {
        let (tasks, updates) = vision::Vision::start(camera.clone(), exe);
        (Some(tasks), Some(updates))
    } else {
        (None, None)
    };
    let app = App {
        safety,
        state,
        states,
        scan,
        camera,
        vision,
        ids: Arc::new(AtomicU64::new(1)),
        recording,
    };
    let listener = match TcpListener::bind((c.listen.as_str(), c.port)).await {
        Ok(l) => l,
        Err(e) => {
            read_task.abort();
            drive_task.abort();
            let _ = read_task.await;
            let _ = drive_task.await;
            let _ = tokio::time::timeout(
                Duration::from_millis(300),
                player::velocity(&mut writer, 0., 0.),
            )
            .await;
            hb.abort();
            let _ = hb.await;
            return Err(e.into());
        }
    };
    let router = Router::new()
        .route("/", get(index))
        .route("/ws", get(upgrade))
        .route("/camera.mjpeg", get(camera::proxy))
        .route("/vision/ws", get(vision::upgrade))
        .with_state(app)
        .merge(audio::router(audio));
    let mut http = tokio::spawn(async move { axum::serve(listener, router).await });
    let (result, completed) = tokio::select! {
     _=shutdown_signal()=>(Ok(()),0),
     r=&mut read_task=>(r.map_err(|e|->Failure{e.into()}).and_then(|r|r),1),
     r=&mut drive_task=>(r.map_err(|e|->Failure{e.into()}).and_then(|r|r),2),
     r=&mut http=>{let _=r;(Err("HTTP server stopped".into()),3)},
     r=&mut hb=>(r.map_err(|e|->Failure{e.into()}).and_then(|r|r),4),
    };
    // Cancel all command producers first; the final zero is the only permitted
    // unconditional velocity. A deadline prevents cleanup hanging on a dead peer.
    http.abort();
    read_task.abort();
    drive_task.abort();
    hb.abort();
    if completed != 3 {
        let _ = http.await;
    }
    if completed != 1 {
        let _ = read_task.await;
    }
    if completed != 2 {
        let _ = drive_task.await;
    }
    if completed != 4 {
        let _ = hb.await;
    }
    let _ = tokio::time::timeout(
        Duration::from_millis(300),
        player::velocity(&mut writer, 0., 0.),
    )
    .await;
    result
}
#[tokio::main]
async fn main() {
    let result = match Config::parse() {
        Ok(Some(c)) => run(c).await,
        Ok(None) => Ok(()),
        Err(e) => Err(e),
    };
    if let Err(e) = result {
        eprintln!("robot-server: {e}");
        std::process::exit(1);
    }
}
