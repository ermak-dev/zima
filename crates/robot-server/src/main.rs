mod audio;
mod camera;
mod compass;
mod grants;
mod mcp;
mod mcp_drive;
mod player;
mod recovery;
mod safety;
mod slam;
mod state;
mod supervisor_drive;
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
    /// The panel/MCP view of the typed state (JSON is built at the boundary).
    state: watch::Receiver<state::State>,
    states: watch::Sender<state::State>,
    scan: watch::Receiver<Option<Value>>,
    camera: camera::Camera,
    vision: Option<watch::Receiver<Value>>,
    maps: Option<watch::Receiver<Value>>,
    ids: Arc<AtomicU64>,
    recording: Option<Arc<SessionRecording>>,
    motors: Arc<std::sync::Mutex<Motors>>,
    player: player::Writer,
    /// Operator-granted MCP admission (#56); installed/revoked only from
    /// operator-side paths, never from an LLM request.
    grants: Arc<std::sync::Mutex<grants::Grants>>,
    /// Idempotent registry of bounded MCP drive actions (#60).
    drive_actions: Arc<std::sync::Mutex<mcp_drive::DriveActions>>,
    /// Shared TTS pipeline (panel HTTP and the MCP robot_say tool, #63).
    audio: audio::Audio,
    /// Supervisor address/token for the robot-local drive lease (#64);
    /// None when the session runs without a supervisor (audio-only, tests).
    supervisor: Option<(String, String)>,
}
/// Whether this session ever commanded the cleaning motors; duties live in
/// the typed state. The write half sends PWM commands on the sole Player
/// connection. Ownership lives in Safety: wheels and cleaning motors share
/// one exclusive control owner. `active` records whether this session ever
/// commanded the motors, so an owner who never touched them does not emit
/// teardown PWM frames.
struct Motors {
    active: bool,
}
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Command {
    v: f64,
    w: f64,
}
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct MotorsCommand {
    #[serde(rename = "type")]
    kind: MotorsKind,
    fan: Option<f64>,
    main: Option<f64>,
    side: Option<f64>,
}
#[derive(Deserialize, PartialEq)]
#[serde(rename_all = "lowercase")]
enum MotorsKind {
    Motors,
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
// Dropping the control owner publishes zeros on the shared Player writer; the
// vendor queue is not a reliable cancel path.
struct MotorsGuard {
    id: u64,
    motors: Arc<Mutex<Motors>>,
    safety: Arc<Mutex<Safety>>,
    writer: player::Writer,
    states: watch::Sender<state::State>,
}
impl Drop for MotorsGuard {
    fn drop(&mut self) {
        let stop = {
            if self.safety.lock().unwrap().owner() != Some(self.id) {
                false
            } else {
                let mut motors = self.motors.lock().unwrap();
                std::mem::take(&mut motors.active)
            }
        };
        if stop {
            self.states.send_modify(|state| state.stop_motors());
            let writer = self.writer.clone();
            tokio::spawn(async move {
                for index in 0..3u8 {
                    let _ = player::motor_pwm(&writer, index, 0.).await;
                }
            });
        }
    }
}
async fn session(socket: WebSocket, app: App) {
    let id = app.ids.fetch_add(1, Ordering::Relaxed);
    let _guard = DriverGuard {
        id,
        safety: app.safety.clone(),
    };
    let _motors_guard = MotorsGuard {
        id,
        motors: app.motors.clone(),
        safety: app.safety.clone(),
        writer: app.player.clone(),
        states: app.states.clone(),
    };
    let (mut output, mut input) = socket.split();
    let mut state = app.state.clone();
    let mut scan = app.scan.clone();
    let mut maps = app.maps.clone();
    let (recovery_tx, mut recovery_rx) = tokio::sync::mpsc::channel(4);
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
                    } else if let Ok(m) = serde_json::from_str::<MotorsCommand>(&text) {
                        let duties = (m.fan, m.main, m.side);
                        if duties.0.map(|v| v.is_finite()).unwrap_or(true)
                            && duties.1.map(|v| v.is_finite()).unwrap_or(true)
                            && duties.2.map(|v| v.is_finite()).unwrap_or(true)
                        {
                            let writer = app.player.clone();
                            let allowed = {
                                // Same lock order as reset: motors never
                                // command while blocked or under another owner.
                                let safety = app.safety.lock().unwrap();
                                if safety.is_blocked() || safety.owner() != Some(id) {
                                    false
                                } else {
                                    app.motors.lock().unwrap().active = true;
                                    true
                                }
                            };
                            if allowed {
                                app.states.send_modify(|state| {
                                    state.set_motors(
                                        player_bridge::motor_pwm_clamp(duties.0.unwrap_or(0.)),
                                        player_bridge::motor_pwm_clamp(duties.1.unwrap_or(0.)),
                                        player_bridge::motor_pwm_clamp(duties.2.unwrap_or(0.)),
                                    );
                                });
                                for (index, duty) in
                                    [duties.0, duties.1, duties.2].into_iter().enumerate()
                                {
                                    let index = u8::try_from(index).expect("three motors");
                                    if let Err(e) = player::motor_pwm(
                                        &writer,
                                        index,
                                        player_bridge::motor_pwm_clamp(duty.unwrap_or(0.)),
                                    )
                                    .await
                                    {
                                        eprintln!("robot-server: motor command failed: {e}");
                                    }
                                }
                            }
                        }
                    } else if let Ok(mut request) =
                        serde_json::from_str::<Value>(&text).and_then(|v| {
                            if v.get("type").and_then(Value::as_str) == Some("recover_bumper") {
                                Ok(v)
                            } else {
                                Err(serde::de::Error::custom("not a recovery request"))
                            }
                        })
                    {
                        request.as_object_mut().unwrap().remove("type");
                        let reply =
                            recovery::start(&app, recovery::Actor::Panel(id), request).await;
                        let _ = recovery_tx.try_send(reply);
                    } else if serde_json::from_str::<ResetCommand>(&text).is_ok() {
                        if let Some(r) = &app.recording {
                            r.command(id, None, std::time::Instant::now());
                        }
                        let mut safety = app.safety.lock().unwrap();
                        if safety.is_recovering() {
                            let _ = recovery_tx
                                .try_send(recovery::failure("active recovery; reset BUSY"));
                            continue;
                        }
                        // A reset under a still-present hazard is refused with
                        // the reason; the latch stays on (#65).
                        let outcome = safety.reset_blocked();
                        if outcome.is_ok() {
                            app.states.send_modify(|state| state.clear_blocked());
                        } else {
                            let reason = format!("{:?}", outcome.err().unwrap()).to_lowercase();
                            eprintln!("robot-server: reset refused: {reason} still present");
                        }
                    }
                }
                Message::Close(_) => break,
                _ => {}
            }
        }
    };
    let send = async {
        let initial = state.borrow_and_update().to_json();
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
        // The current map replays to a fresh viewer without waiting for the
        // next grid change; only new versions are streamed afterwards.
        if let Some(maps) = maps.as_mut() {
            let current = maps.borrow_and_update().clone();
            if current != Value::Null {
                send_frame(&mut output, "map", current).await?;
            }
        }
        loop {
            let map_change = async {
                match maps.as_mut() {
                    Some(maps) => maps.changed().await,
                    None => std::future::pending().await,
                }
            };
            tokio::select! {
             Some(v)=recovery_rx.recv()=>{send_frame(&mut output,"recovery",v).await?;},
             r=state.changed()=>{r?;let v=state.borrow_and_update().to_json();send_frame(&mut output,"state",v).await?;},
             r=scan.changed()=>{r?;let v=scan.borrow_and_update().clone();if let Some(v)=v{send_frame(&mut output,"scan",v).await?;}}
             _=map_change=>{if let Some(maps)=maps.as_mut(){let v=maps.borrow_and_update().clone();send_frame(&mut output,"map",v).await?;}}
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
/// Operator-installed MCP admission from the environment (#56): only the
/// operator controls this variable, never an LLM request. Format:
/// subject,max_linear_mps,max_angular_radps,max_action_ms,budget_ms,ttl_s.
fn mcp_grant_from_env() -> grants::Grants {
    let mut grants = grants::Grants::default();
    let Ok(spec) = std::env::var("ROBOT_MCP_GRANT") else {
        return grants;
    };
    let fields: Vec<&str> = spec.split(',').collect();
    if fields.len() != 6 {
        eprintln!("robot-server: ROBOT_MCP_GRANT must have 6 comma-separated fields");
        return grants;
    }
    let parsed = (
        fields[1].parse::<f64>(),
        fields[2].parse::<f64>(),
        fields[3].parse::<u64>(),
        fields[4].parse::<u64>(),
        fields[5].parse::<u64>(),
    );
    if let (Ok(lin), Ok(ang), Ok(action), Ok(budget), Ok(ttl)) = parsed {
        if lin.is_finite()
            && ang.is_finite()
            && lin >= 0.
            && ang >= 0.
            && action > 0
            && budget > 0
            && ttl > 0
        {
            grants.install(grants::ControlGrant {
                subject: fields[0].to_owned(),
                max_linear_mps: lin,
                max_angular_radps: ang,
                max_action_ms: action,
                total_budget_ms: budget,
                expires_at: std::time::Instant::now() + std::time::Duration::from_secs(ttl),
                spent_ms: 0,
            });
            eprintln!(
                "robot-server: MCP control grant installed for subject '{}' (operator env)",
                fields[0]
            );
            return grants;
        }
    }
    eprintln!("robot-server: invalid ROBOT_MCP_GRANT; no grant installed");
    grants
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
    let (states, state) = watch::channel(state::State::initial());
    let (scans, scan) = watch::channel(None);
    // Session recording keeps its JSON snapshot view of the typed state; the
    // boundary serialization stays out of the frequent producers.
    let (mirror_tx, mirror_rx) = watch::channel(Value::Null);
    let _mirror_task = tokio::spawn({
        let mut state = state.clone();
        async move {
            loop {
                let changed = state.changed().await;
                if changed.is_err() {
                    break;
                }
                mirror_tx.send_replace(state.borrow_and_update().to_json());
            }
        }
    });
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
            mirror_rx.clone(),
            scan.clone(),
        )
    });
    // SLAM observations come from Player; results are display data, not drive gates.
    let mut maps_rx = None;
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
                states_tx.send_if_modified(|old| old.publish_slam(slam_state));
            }),
        ) {
            Ok(t) => {
                maps_rx = Some(t.maps);
                Some(t.observe)
            }
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
    let motors = Arc::new(Mutex::new(Motors { active: false }));
    let app = App {
        safety,
        state,
        states,
        scan,
        camera,
        vision,
        maps: maps_rx,
        ids: Arc::new(AtomicU64::new(1)),
        recording,
        motors,
        player: writer.clone(),
        grants: Arc::new(std::sync::Mutex::new(mcp_grant_from_env())),
        drive_actions: Arc::new(std::sync::Mutex::new(mcp_drive::DriveActions::default())),
        audio: audio.clone(),
        supervisor: std::env::var("ROBOT_SUPERVISOR_ADDR").ok().map(|addr| {
            (
                addr,
                std::env::var("ROBOT_SUPERVISOR_TOKEN").unwrap_or_default(),
            )
        }),
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
        .route("/mcp", axum::routing::post(mcp::endpoint))
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
    for index in 0..3u8 {
        let _ = tokio::time::timeout(
            Duration::from_millis(300),
            player::motor_pwm(&writer, index, 0.),
        )
        .await;
    }
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
