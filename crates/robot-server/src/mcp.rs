//! Read-only MCP surface beside the panel: Streamable HTTP JSON-RPC at /mcp.
//! The dispatcher reuses the shared state watch channel; it never touches the
//! mask, controller, drive writer or camera credentials. No LLM is called.
use crate::{mcp_drive, App, Failure};
use axum::{
    extract::State,
    http::{HeaderMap, StatusCode},
    response::{IntoResponse, Response},
};
use base64::{engine::general_purpose::STANDARD, Engine};
use serde_json::{json, Value};
use std::{
    sync::atomic::{AtomicU64, Ordering},
    time::Duration,
};

const PROTOCOL_VERSION: &str = "2025-11-25";
const BODY_LIMIT: usize = 64 * 1024;
/// Wall clock bound for one camera tool call; `snapshot` is bounded itself,
/// but the HTTP request must not hang longer than this either (#59).
const FRAME_CALL_LIMIT: Duration = Duration::from_secs(30);
/// Monotonic frame counter shared by all calls in this process.
static FRAME_ID: AtomicU64 = AtomicU64::new(0);
/// Monotonic observation counter for `robot_observe` (#57).
static OBSERVE_ID: AtomicU64 = AtomicU64::new(1);
/// Process-lifetime session epoch: bumps when the operator grant state
/// changes (#56); stale epochs are rejected by control tools.
pub static EPOCH: AtomicU64 = AtomicU64::new(1);
/// Dedicated id range for MCP control leases, above any panel client id.
static MCP_OWNER_ID: AtomicU64 = AtomicU64::new(0);

fn error(id: Value, code: i64, message: &str) -> Response {
    json_response(
        json!({"jsonrpc":"2.0","id":id,"error":{"code":code,"message":message}}),
        StatusCode::OK,
    )
}
fn json_response(value: Value, status: StatusCode) -> Response {
    (
        status,
        [(axum::http::header::CONTENT_TYPE, "application/json")],
        value.to_string(),
    )
        .into_response()
}

/// The only parameterless observation tool of the first MCP milestone. Later
/// milestones add map/camera/motion tools; all of them stay read-only until
/// the operator-granted control lease exists (#56).
fn tools() -> Value {
    json!([
        {
            "name": "robot_observe",
        "description": "Компактный слепок состояния робота: датчики, одометрия, SLAM-статус, владелец управления и возраст источников. Только чтение; не двигает робота.",
        "inputSchema": {"type":"object","additionalProperties":false},
        "outputSchema": {"type":"object"},
        "annotations": {"readOnlyHint": true, "destructiveHint": false,
                        "idempotentHint": true, "openWorldHint": false}
        },
        camera_frame_tool(),
        map_tool(),
        acquire_control_tool(),
        release_control_tool(),
        drive_for_tool(),
        mcp_stop_tool(),
        get_action_tool(),
        cleaning_motors_tool(),
        say_tool(),
    ])
}

/// The admission tools exist unconditionally so clients can discover them,
/// but without an operator-installed grant every call is an error (#56):
/// `confirm` in the request never creates authority.
fn acquire_control_tool() -> Value {
    json!({
        "name": "robot_acquire_control",
        "description": "Запросить управление в рамках операторского допуска. Требуется заранее установленный оператором допуск с лимитами; параметр confirm не создаёт права. Без допуска — ошибка.",
        "inputSchema": {"type":"object","additionalProperties":false,"properties":{
            "confirm": {"type":"boolean","description":"игнорируется как источник права"}
        }},
        "outputSchema": {"type":"object"},
        "annotations": {"readOnlyHint": false, "destructiveHint": false,
                        "idempotentHint": false, "openWorldHint": false}
    })
}

/// Bounded motion tool (#60). readOnlyHint is false by design.
fn drive_for_tool() -> Value {
    json!({
        "name": "robot_drive_for",
        "description": "Ограниченное движение корпуса: скорость на duration_ms. Требует активного MCP-управления и операторского допуска; command_id идемпотентен (тот же id+параметры — тот же результат без повторного исполнения).",
        "inputSchema": {"type":"object","additionalProperties":false,"required":["linear_mps","angular_radps","duration_ms","command_id"],"properties":{
            "linear_mps": {"type":"number"},
            "angular_radps": {"type":"number"},
            "duration_ms": {"type":"integer","minimum":1,"maximum":60000},
            "command_id": {"type":"string"},
            "based_on_observation": {"type":"integer","description":"observation_id из robot_observe"}
        }},
        "outputSchema": {"type":"object"},
        "annotations": {"readOnlyHint": false, "destructiveHint": false,
                        "idempotentHint": true, "openWorldHint": false}
    })
}

fn mcp_stop_tool() -> Value {
    json!({
        "name": "robot_stop",
        "description": "Приоритетная остановка активного действия. Отправленный ноль не доказывает физическую остановку; локальный дедлайн — супервизор.",
        "inputSchema": {"type":"object","additionalProperties":false},
        "outputSchema": {"type":"object"},
        "annotations": {"readOnlyHint": false, "destructiveHint": false,
                        "idempotentHint": true, "openWorldHint": false}
    })
}

fn get_action_tool() -> Value {
    json!({
        "name": "robot_get_action",
        "description": "Узнать честный исход ранее отправленной команды по command_id.",
        "inputSchema": {"type":"object","additionalProperties":false,"required":["command_id"],"properties":{
            "command_id": {"type":"string"}
        }},
        "outputSchema": {"type":"object"},
        "annotations": {"readOnlyHint": true, "destructiveHint": false,
                        "idempotentHint": true, "openWorldHint": false}
    })
}

fn release_control_tool() -> Value {
    json!({
        "name": "robot_release_control",
        "description": "Освободить управление, если им владеет этот MCP-субъект; остановка активного действия выполняется приоритетно.",
        "inputSchema": {"type":"object","additionalProperties":false},
        "outputSchema": {"type":"object"},
        "annotations": {"readOnlyHint": false, "destructiveHint": false,
                        "idempotentHint": true, "openWorldHint": false}
    })
}

/// Cleaning-motor tool (#61). readOnlyHint is false by design.
/// Speech tool (#63): the shared Audio pipeline behind the panel's
/// /api/audio/say. Acceptance only: the final outcome is polled via the
/// returned task id or a later observe; no secrets in errors.
fn say_tool() -> Value {
    json!({
        "name": "robot_say",
        "description": "Озвучить текст через динамик робота. Возвращает принятое задание (task_id, state) — принято ≠ завершено; итог узнавайте по task_id. Требует настроенного облачного TTS на сервере.",
        "inputSchema": {"type":"object","additionalProperties":false,"required":["text"],"properties":{
            "text": {"type":"string","minLength":1,"maxLength":2000},
            "volume": {"type":"number","minimum":0,"maximum":1}
        }},
        "outputSchema": {"type":"object"},
        "annotations": {"readOnlyHint": false, "destructiveHint": false,
                        "idempotentHint": false, "openWorldHint": false}
    })
}

fn cleaning_motors_tool() -> Value {
    json!({
        "name": "robot_set_cleaning_motors",
        "description": "Ограниченное управление моторами уборки: вентилятор, основная и боковая щётка (PWM 0..100). Требует активного MCP-управления (robot_acquire_control) и живого операторского допуска. Значения вне 0..100 отвергаются, а не приводятся к границе. Это запрошенный PWM, не измеренные обороты; release_control обнуляет все три мотора.",
        "inputSchema": {"type":"object","additionalProperties":false,"properties":{
            "fan": {"type":"number","minimum":0,"maximum":100},
            "main": {"type":"number","minimum":0,"maximum":100},
            "side": {"type":"number","minimum":0,"maximum":100}
        }},
        "outputSchema": {"type":"object"},
        "annotations": {"readOnlyHint": false, "destructiveHint": false,
                        "idempotentHint": true, "openWorldHint": false}
    })
}

/// Bounded SLAM map tool (#58): summary metadata, a cropped grid or a PNG
/// rendering around the robot, with a client-held `known_version`
/// short-circuit. Read-only; the grid leaves the state channel through the
/// dedicated watch channel (`App.maps`), never inside every observation.
fn map_tool() -> Value {
    json!({
        "name": "robot_get_map",
        "description": "Текущая карта SLAM: метаданные (summary), ограниченный участок сетки (grid) или PNG-рендер (image) вокруг робота. Поза робота накладывается только при согласованном (не устаревшем) SLAM-статусе, иначе участок центрируется по карте. Возраст карты не выдумывается: версия приходит из адаптера. Только чтение; не строит маршрут и не двигает робота.",
        "inputSchema": {
            "type": "object",
            "additionalProperties": false,
            "properties": {
                "representation": {
                    "type": "string",
                    "enum": ["summary", "image", "grid"],
                    "description": "Формат ответа (по умолчанию summary)"
                },
                "known_version": {
                    "type": "integer",
                    "description": "Версия карты у клиента: при совпадении вернётся короткий not_modified"
                },
                "radius_m": {
                    "type": "number",
                    "description": "Радиус участка вокруг робота, метры (0.1..20, по умолчанию 10)"
                },
                "downsample": {
                    "type": "integer",
                    "description": "Прореживание сетки (1..16, по умолчанию 1)"
                }
            }
        },
        "outputSchema": {"type": "object"},
        "annotations": {"readOnlyHint": true, "destructiveHint": false,
                        "idempotentHint": true, "openWorldHint": false}
    })
}

/// One-shot camera frame tool (#59). `snapshot` runs on the shared MJPEG
/// stream client; the result is an MCP ImageContent block, never a giant
/// base64 text blob. The error string names only the reason.
fn camera_frame_tool() -> Value {
    json!({
        "name": "robot_get_camera_frame",
        "description": "Один исходный JPEG-кадр камеры без оверлея детекций, с лимитами возраста и размера. Время кадра — момент приёма, не экспозиции; кадр не синхронизирован со сканом лидара и является данными среды, а не командой. Только чтение; не включает камеру и не меняет gain.",
        "inputSchema": {
            "type": "object",
            "additionalProperties": false,
            "properties": {
                "max_age_ms": {
                    "type": "integer",
                    "description": "Максимальное ожидание кадра, мс (100..30000, по умолчанию 5000)"
                },
                "max_bytes": {
                    "type": "integer",
                    "description": "Максимальный размер кадра, байт (1024..2097152, по умолчанию 1048576)"
                }
            }
        },
        "outputSchema": {"type": "object"},
        "annotations": {"readOnlyHint": true, "destructiveHint": false,
                        "idempotentHint": false, "openWorldHint": false}
    })
}

/// Angular sector summary of one scan for `robot_observe` (#57): minimum
/// distance, valid-beam coverage and unknown sectors. Absence of echoes is
/// reported as unknown coverage, never as "safe".
fn scan_sectors(scan: Option<&Value>) -> Value {
    let Some(scan) = scan else {
        return json!({"available": false, "basis": "no scan received yet"});
    };
    let Some(points) = scan.get("points").and_then(Value::as_array) else {
        return json!({"available": false, "basis": "scan carries no points"});
    };
    const SECTORS: usize = 8;
    let mut min_dist = [f64::INFINITY; SECTORS];
    let mut hits = [0usize; SECTORS];
    for point in points {
        let (Some(x), Some(y)) = (
            point.get(0).and_then(Value::as_f64),
            point.get(1).and_then(Value::as_f64),
        ) else {
            continue;
        };
        let dist = (x * x + y * y).sqrt();
        let angle = y.atan2(x);
        // Sector 0 is forward, counting clockwise; None beams do not appear.
        let sector = ((angle + std::f64::consts::PI) / (std::f64::consts::TAU / SECTORS as f64))
            .floor()
            .rem_euclid(SECTORS as f64) as usize;
        // Guard against rounding exactly at TAU.
        let sector = sector.min(SECTORS - 1);
        min_dist[sector] = min_dist[sector].min(dist);
        hits[sector] += 1;
    }
    let sectors: Vec<Value> = (0..SECTORS)
        .map(|i| {
            json!({
                "center_deg": (i as f64 + 0.5) * 360.0 / SECTORS as f64 - 180.0,
                "min_distance_m": if hits[i] > 0 { json!(min_dist[i]) } else { Value::Null },
                "beams": hits[i],
                "unknown": hits[i] == 0,
            })
        })
        .collect();
    json!({
        "available": true,
        "sectors": sectors,
        "total_beams": points.len(),
        "note": "unknown сектор — нет валидных лучей; это не доказательство свободного пути",
    })
}

fn observe(state: &crate::state::State, scan: Option<&Value>) -> Value {
    // Age of the latest data is derived server-side at call time from the
    // monotonic receive instants; the snapshot is not a simultaneous physical
    // measurement of all sensors (#57).
    let now = std::time::Instant::now();
    let frame = state.to_json();
    let player_age = now.duration_since(state.last_player).as_millis() as u64;
    let slam_age = state
        .last_slam
        .map(|t| now.duration_since(t).as_millis() as u64);
    json!({
        "schema_version": 1,
        "observation_id": OBSERVE_ID.fetch_add(1, Ordering::Relaxed),
        "session_epoch": EPOCH.fetch_add(0, Ordering::Relaxed),
        "freshness": {
            "basis": "server receive time, not sensor exposure time",
            "player_age_ms": player_age,
            "slam_age_ms": slam_age,
            "scan_sequence": scan.and_then(|s| s.get("sequence")).cloned().unwrap_or(Value::Null),
            "scan_stamp": scan.and_then(|s| s.get("stamp")).cloned().unwrap_or(Value::Null),
        },
        "blocked": frame.get("blocked").cloned().unwrap_or(Value::Null),
        "motors": frame.get("motors").cloned().unwrap_or(Value::Null),
        "odom": frame.get("odom").cloned().unwrap_or(Value::Null),
        "sensors": frame.get("sensors").cloned().unwrap_or(Value::Null),
        "lidar": scan_sectors(scan),
        "slam": {
            "mode": frame["slam"].get("mode").cloned().unwrap_or(Value::Null),
            "pose": frame["slam"].get("pose").cloned().unwrap_or(Value::Null),
            "stale": frame["slam"].get("stale").cloned().unwrap_or(Value::Null),
            "map_version": frame["slam"].get("map_version").cloned().unwrap_or(Value::Null),
            "scan_sequence": frame["slam"].get("scan_sequence").cloned().unwrap_or(Value::Null),
        },
    })
}

fn dispatch(app: &App, request: Value) -> Response {
    let id = request.get("id").cloned().unwrap_or(Value::Null);
    let Some(method) = request.get("method").and_then(Value::as_str) else {
        return error(id, -32600, "invalid request");
    };
    match method {
        "initialize" => json_response(
            json!({"jsonrpc":"2.0","id":id,"result":{
                "protocolVersion": PROTOCOL_VERSION,
                "capabilities": {"tools": {"listChanged": false}},
                "serverInfo": {"name": "robot-server", "version": env!("CARGO_PKG_VERSION")}
            }}),
            StatusCode::OK,
        ),
        "notifications/initialized" => (StatusCode::ACCEPTED, "").into_response(),
        "ping" => json_response(json!({"jsonrpc":"2.0","id":id,"result":{}}), StatusCode::OK),
        "tools/list" => json_response(
            json!({"jsonrpc":"2.0","id":id,"result":{"tools": tools()}}),
            StatusCode::OK,
        ),
        "tools/call" => {
            let params = request.get("params").cloned().unwrap_or(Value::Null);
            let name = params.get("name").and_then(Value::as_str).unwrap_or("");
            if name == "robot_get_map" {
                let args = params.get("arguments").cloned().unwrap_or(Value::Null);
                return get_map(id, app, args);
            }
            if name == "robot_acquire_control" {
                let args = params.get("arguments").cloned().unwrap_or(Value::Null);
                return acquire_control(id, app, args);
            }
            if name == "robot_release_control" {
                return release_control(id, app);
            }
            if name == "robot_say" {
                let args = params.get("arguments").cloned().unwrap_or(Value::Null);
                return say(id, app, args);
            }
            if name == "robot_set_cleaning_motors" {
                let args = params.get("arguments").cloned().unwrap_or(Value::Null);
                return set_cleaning_motors(id, app, args);
            }
            if name == "robot_get_action" {
                let args = params.get("arguments").cloned().unwrap_or(Value::Null);
                return get_action(id, app, args);
            }
            // `robot_get_camera_frame` never reaches here: `endpoint` runs it
            // with a real async timeout before the sync dispatcher.
            if name != "robot_observe" {
                // Tool execution error: actionable for the model, not protocol-level.
                return json_response(
                    json!({"jsonrpc":"2.0","id":id,"result":{
                        "content": [{"type":"text","text": format!("unknown tool: {name}")}],
                        "isError": true}}),
                    StatusCode::OK,
                );
            }
            let mut state = app.state.clone();
            let mut scan = app.scan.clone();
            let snapshot = observe(
                &state.borrow_and_update(),
                scan.borrow_and_update().as_ref(),
            );
            json_response(
                json!({"jsonrpc":"2.0","id":id,"result":{
                    "content": [{"type":"text","text": snapshot.to_string()}],
                    "structuredContent": snapshot,
                    "isError": false}}),
                StatusCode::OK,
            )
        }
        _ => error(id, -32601, "method not found"),
    }
}

fn tool_error(id: Value, message: String) -> Response {
    json_response(
        json!({"jsonrpc":"2.0","id":id,"result":{
            "content": [{"type":"text","text": message}],
            "isError": true}}),
        StatusCode::OK,
    )
}

/// Acquire the shared WEB/MCP control ownership for the MCP subject (#56).
/// Admission comes only from an operator-installed grant; `confirm` in the
/// request is explicitly not a source of authority. Without the panel being
/// free, acquisition is BUSY and never steals ownership.
fn acquire_control(id: Value, app: &App, args: Value) -> Response {
    if !args.is_object() && !args.is_null() {
        return tool_error(id, "arguments must be an object".into());
    }
    if let Some(extra) = args.as_object() {
        const ALLOWED: [&str; 1] = ["confirm"];
        if extra.keys().any(|k| !ALLOWED.contains(&k.as_str())) {
            return tool_error(id, "unknown arguments".into());
        }
    }
    // The MCP subject identity for grants. Until a per-client authenticated
    // identity exists, this endpoint binds the grant to the single local
    // subject "mcp" — the grant still has to come from the operator.
    let subject = "mcp";
    let mut grants = app.grants.lock().unwrap();
    let now = std::time::Instant::now();
    if let crate::grants::Admission::Denied(reason) = grants.admit(subject, 0.0, 0.0, 0, now) {
        return tool_error(id, format!("no operator grant: {reason}"));
    }
    let epoch = grants.epoch();
    drop(grants);
    // Take the shared ownership through the same Safety gate as the panel:
    // the MCP subject uses a dedicated high id range above panel clients.
    let id_value = MCP_OWNER_ID.fetch_add(1, Ordering::Relaxed) + 1_000_000;
    let mut safety = app.safety.lock().unwrap();
    if let Some(owner) = safety.owner() {
        if owner >= 1_000_000 {
            // Another MCP lease: refresh is idempotent for the same subject.
            return json_response(
                json!({"jsonrpc":"2.0","id":id,"result":{
                    "content": [{"type":"text","text": format!("control already held by MCP lease {owner}")}],
                    "structuredContent": {"epoch": epoch, "owner": owner, "state": "already_held"},
                    "isError": false}}),
                StatusCode::OK,
            );
        }
        return tool_error(id, format!("panel client {owner} owns control; BUSY"));
    }
    // Claim with a zero command (same entry path as a panel command): the
    // lease semantics stay in Safety; MCP only chooses to enter or leave.
    safety.command(id_value, 0., 0., now);
    json_response(
        json!({"jsonrpc":"2.0","id":id,"result":{
            "content": [{"type":"text","text": format!("control acquired for subject {subject}, epoch {epoch}")}],
            "structuredContent": {"epoch": epoch, "owner": id_value, "state": "acquired",
                "note": "limиты допуска применяются к инструментам движения; release обязателен по завершении"},
            "isError": false}}),
        StatusCode::OK,
    )
}

/// Release the MCP control lease, if this endpoint holds it. The panel is
/// never affected; a release with no lease is a no-op success (#56).
fn release_control(id: Value, app: &App) -> Response {
    let mut safety = app.safety.lock().unwrap();
    let owner = safety.owner();
    let held = owner.is_some_and(|owner| owner >= 1_000_000);
    if held {
        // disconnect() drops the lease and schedules the priority stop.
        safety.disconnect(owner.unwrap());
    }
    drop(safety);
    // Lease teardown zeroes the cleaning motors (#61): the MotorsGuard only
    // covers a panel WS session, so the MCP release path must request zeros
    // itself for all three motors. Teardown never relies on the vendor
    // zero-on-unsubscribe queue.
    if held {
        app.states.send_modify(|state| state.stop_motors());
        let mut writer = app.player.clone();
        for index in 0..3u8 {
            let _ = futures_util::FutureExt::now_or_never(async {
                crate::player::motor_pwm(&writer, index, 0.).await
            });
        }
    }
    let epoch = app.grants.lock().unwrap().epoch();
    json_response(
        json!({"jsonrpc":"2.0","id":id,"result":{
            "content": [{"type":"text","text": if held { "control released; stop pending" } else { "no MCP control lease held" }}],
            "structuredContent": {"epoch": epoch, "released": held},
            "isError": false}}),
        StatusCode::OK,
    )
}

// ---- #61: limited cleaning-motor control ----

/// Validate the cleaning-motor arguments. Levels are enforced in code, not
/// trusted from the schema: only finite numbers in 0..=100 are accepted —
/// out-of-range values are rejected outright, never silently clamped (the
/// panel path clamps via motor_pwm_clamp, but an LLM request gets an honest
/// error instead of a different level than requested).
fn parse_motors_args(args: &Value) -> Result<(f64, f64, f64), String> {
    const KNOWN: [&str; 3] = ["fan", "main", "side"];
    let Some(object) = args.as_object() else {
        return Err("arguments must be an object".into());
    };
    for key in object.keys() {
        if !KNOWN.contains(&key.as_str()) {
            return Err(format!("unknown argument: {key}"));
        }
    }
    let mut level = |key: &str| -> Result<f64, String> {
        let Some(value) = object.get(key) else {
            return Ok(0.0);
        };
        let number = value
            .as_f64()
            .ok_or(format!("{key} must be a number in 0..100"))?;
        if !number.is_finite() || !(0.0..=100.0).contains(&number) {
            return Err(format!("{key} must be a number in 0..100"));
        }
        Ok(number)
    };
    Ok((level("fan")?, level("main")?, level("side")?))
}

/// `robot_set_cleaning_motors` (#61): bounded PWM levels for the cleaning
/// motors under the shared Safety ownership. Requires an active MCP control
/// lease (BUSY-style error when the panel owns control, "acquire control
/// first" when nobody does) plus a live operator grant; until a dedicated
/// motor permission exists, the grant admission is checked with a 1 ms
/// zero-motion action — TODO(#62): separate operator permission for the
/// cleaning motors and a local activity deadline on the robot (the vendor
/// zero-on-unsubscribe semantics make a host-side timeout an unsafe
/// substitute, so the hardware acceptance of PWM termination is #62).
fn set_cleaning_motors(id: Value, app: &App, args: Value) -> Response {
    let (fan, main, side) = match parse_motors_args(&args) {
        Ok(v) => v,
        Err(message) => return tool_error(id, message),
    };
    // MCP control lease must be held (the shared Safety owner from #56).
    let mut safety = app.safety.lock().unwrap();
    match safety.owner() {
        Some(owner) if owner >= 1_000_000 => {}
        Some(owner) => {
            return tool_error(id, format!("panel client {owner} owns control; BUSY"));
        }
        None => {
            return tool_error(
                id,
                "control not acquired; call robot_acquire_control first".into(),
            );
        }
    }
    // Operator admission: an installed, live grant is required. A 1 ms
    // zero-motion check verifies the grant without consuming real budget.
    let now = std::time::Instant::now();
    if let crate::grants::Admission::Denied(reason) =
        app.grants.lock().unwrap().admit("mcp", 0.0, 0.0, 1, now)
    {
        return tool_error(id, format!("no operator grant: {reason}"));
    }
    // Same lock order as the panel motors handler: blocked or ownerless
    // never commands motors; mark the session as having touched them so the
    // MotorsGuard teardown emits PWM zeros.
    if safety.is_blocked() {
        return tool_error(id, "control is blocked (bumper); motors refused".into());
    }
    app.motors.lock().unwrap().active = true;
    drop(safety);
    // Requested duties are mirrored into the shared typed state (requested
    // PWM, not measured speed/current) and sent synchronously on the sole
    // Player writer, like the panel motors handler.
    app.states.send_modify(|state| {
        state.set_motors(fan as f32, main as f32, side as f32);
    });
    let writer = app.player.clone();
    let mut sent = true;
    for (index, duty) in [fan, main, side].into_iter().enumerate() {
        let index = u8::try_from(index).expect("three motors");
        let writer = writer.clone();
        let ok = futures_util::FutureExt::now_or_never(async move {
            crate::player::motor_pwm(&writer, index, duty as f32).await
        })
        .unwrap_or_else(|| Ok(()))
        .is_ok();
        sent = sent && ok;
    }
    json_response(
        json!({"jsonrpc":"2.0","id":id,"result":{
            "content": [{"type":"text","text":
                format!("requested PWM fan={fan}, main={main}, side={side}")}],
            "structuredContent": {
                "requested": {"fan": fan, "main": main, "side": side},
                "sent": sent,
                "note": "запрошенный PWM, не измеренные обороты; release_control обнуляет моторы"
            },
            "isError": false}}),
        StatusCode::OK,
    )
}

/// Validate the camera tool arguments; ranges are enforced in code, not
/// trusted from the schema (issue #59: missing frames must not be dressed
/// up as fresh ones, limits must hold).
fn parse_frame_args(args: &Value) -> Result<(u64, usize), String> {
    const MAX_AGE_DEFAULT_MS: u64 = 5_000;
    const MAX_AGE_MIN_MS: u64 = 100;
    const MAX_AGE_MAX_MS: u64 = 30_000;
    const MAX_BYTES_DEFAULT: usize = 1024 * 1024;
    const MAX_BYTES_MIN: usize = 1024;
    const MAX_BYTES_MAX: usize = 2 * 1024 * 1024;
    const KNOWN: [&str; 2] = ["max_age_ms", "max_bytes"];
    let Some(object) = args.as_object() else {
        return Err("arguments must be an object".into());
    };
    for key in object.keys() {
        if !KNOWN.contains(&key.as_str()) {
            return Err(format!("unknown argument: {key}"));
        }
    }
    let max_age_ms = match args.get("max_age_ms") {
        None => MAX_AGE_DEFAULT_MS,
        Some(v) => v
            .as_u64()
            .filter(|v| (MAX_AGE_MIN_MS..=MAX_AGE_MAX_MS).contains(v))
            .ok_or("max_age_ms must be an integer in 100..30000")?,
    };
    let max_bytes = match args.get("max_bytes") {
        None => MAX_BYTES_DEFAULT,
        Some(v) => v
            .as_u64()
            .filter(|v| (MAX_BYTES_MIN as u64..=MAX_BYTES_MAX as u64).contains(v))
            .ok_or("max_bytes must be an integer in 1024..2097152")? as usize,
    };
    Ok((max_age_ms, max_bytes))
}

/// Frame dimensions from the first SOF marker (baseline/progressive/lossless
/// variants all carry 16-bit height/width after the marker header).
fn jpeg_dimensions(frame: &[u8]) -> Option<(u32, u32)> {
    let mut i = 2usize;
    while i + 9 < frame.len() {
        if frame[i] != 0xff {
            return None;
        }
        let marker = frame[i + 1];
        if (0xd0..=0xd9).contains(&marker) || marker == 0x01 {
            // Standalone marker without a length field.
            i += 2;
            continue;
        }
        let length = u16::from_be_bytes([frame[i + 2], frame[i + 3]]) as usize;
        let sof = matches!(marker,
            0xc0..=0xc3 | 0xc5..=0xc7 | 0xc9..=0xcb | 0xcd..=0xcf);
        if sof {
            if length < 7 || i + 9 > frame.len() {
                return None;
            }
            let height = u16::from_be_bytes([frame[i + 5], frame[i + 6]]);
            let width = u16::from_be_bytes([frame[i + 7], frame[i + 8]]);
            if height > 0 && width > 0 {
                return Some((width as u32, height as u32));
            }
            return None;
        }
        // Skip over the segment payload (including the 2 length bytes).
        i = i.checked_add(2 + length.max(2))?;
        if marker == 0xda {
            // Start of scan: dimensions must appear before entropy data.
            return None;
        }
    }
    None
}

/// Validate the map tool arguments; ranges are enforced in code, not trusted
/// from the schema (#58: bounded region, bounded downsample).
fn parse_map_args(args: &Value) -> Result<(&'static str, Option<u64>, f64, u32), String> {
    const RADIUS_DEFAULT_M: f64 = 10.0;
    const RADIUS_MIN_M: f64 = 0.1;
    const RADIUS_MAX_M: f64 = 20.0;
    const DOWNSAMPLE_MAX: u64 = 16;
    let known: [&str; 4] = ["representation", "known_version", "radius_m", "downsample"];
    let Some(object) = args.as_object() else {
        return Err("arguments must be an object".into());
    };
    for key in object.keys() {
        if !known.contains(&key.as_str()) {
            return Err(format!("unknown argument: {key}"));
        }
    }
    let representation: &'static str = match args.get("representation") {
        None => "summary",
        Some(v) => match v.as_str() {
            Some("summary") => "summary",
            Some("image") => "image",
            Some("grid") => "grid",
            _ => return Err("representation must be one of summary, image, grid".into()),
        },
    };
    let known_version = match args.get("known_version") {
        None => None,
        Some(v) => Some(
            v.as_u64()
                .ok_or("known_version must be a non-negative integer")?,
        ),
    };
    let radius_m = match args.get("radius_m") {
        None => RADIUS_DEFAULT_M,
        Some(v) => v
            .as_f64()
            .filter(|r| *r >= RADIUS_MIN_M && *r <= RADIUS_MAX_M)
            .ok_or("radius_m must be a number in 0.1..20")?,
    };
    let downsample = match args.get("downsample") {
        None => 1,
        Some(v) => v
            .as_u64()
            .filter(|d| (1..=DOWNSAMPLE_MAX).contains(d))
            .ok_or("downsample must be an integer in 1..16")? as u32,
    };
    Ok((representation, known_version, radius_m, downsample))
}

/// The map tool (#58): serves the current SLAM map from the dedicated watch
/// channel (`App.maps`, the same source as the panel "map" frames). The crop
/// is centered on the consistent SLAM pose when there is one (not stale, pose
/// present) and on the map center otherwise; the pose itself is drawn onto
/// the PNG only when it comes from that consistent status — neither the pose
/// nor the map age is ever invented. All grid work is bounded by map_view's
/// cell caps, so the call cannot stall the panel.
fn get_map(id: Value, app: &App, args: Value) -> Response {
    let (representation, known_version, radius_m, downsample) = match parse_map_args(&args) {
        Ok(v) => v,
        Err(message) => return tool_error(id, message),
    };
    // No adapter (or no first export yet): an honest error, never a fake map.
    let Some(mut maps) = app.maps.clone() else {
        return tool_error(
            id,
            "SLAM adapter is not running: start robot-server with --slam-adapter \
             to build a map"
                .into(),
        );
    };
    let current = maps.borrow_and_update().clone();
    if current.is_null() {
        return tool_error(
            id,
            "SLAM map is not available yet: no grid has been exported by the adapter".into(),
        );
    }
    let grid = current.get("grid").cloned().unwrap_or(Value::Null);
    let version = current.get("version").and_then(Value::as_u64);
    let (Some(version), true) = (version, !grid.is_null()) else {
        return tool_error(id, "SLAM map watch value is malformed".into());
    };
    if known_version.is_some_and(|known| robot_server::map_view::not_modified(known, version)) {
        return json_response(
            json!({"jsonrpc":"2.0","id":id,"result":{
                "content": [{"type":"text","text":
                    format!("{{\"version\":{version},\"not_modified\":true}}")}],
                "structuredContent": {"version": version, "not_modified": true},
                "isError": false}}),
            StatusCode::OK,
        );
    }
    // Crop center: the consistent SLAM pose, else the map center. A null or
    // stale pose is never fabricated into a center.
    let mut state = app.state.clone();
    let state = state.borrow_and_update().to_json();
    let slam = state.get("slam").cloned().unwrap_or(Value::Null);
    let consistent_pose = slam
        .get("pose")
        .cloned()
        .filter(|_| slam.get("stale").and_then(Value::as_bool) != Some(true))
        .and_then(|p| {
            let (x, y, yaw) = (
                p.get("x").and_then(Value::as_f64)?,
                p.get("y").and_then(Value::as_f64)?,
                p.get("yaw").and_then(Value::as_f64)?,
            );
            if x.is_finite() && y.is_finite() && yaw.is_finite() {
                Some((x, y, yaw))
            } else {
                None
            }
        });
    let map_center = || -> Option<(f64, f64)> {
        Some((
            grid.get("x_min").and_then(Value::as_f64)?
                + grid.get("w").and_then(Value::as_f64)?
                    * grid.get("res").and_then(Value::as_f64)?
                    / 2.0,
            grid.get("y_min").and_then(Value::as_f64)?
                + grid.get("h").and_then(Value::as_f64)?
                    * grid.get("res").and_then(Value::as_f64)?
                    / 2.0,
        ))
    };
    let (center, pose_source, png_pose) = match consistent_pose {
        Some((x, y, yaw)) => ((x, y), "slam_pose", Some((x, y, yaw))),
        None => match map_center() {
            Some(c) => (c, "map_center", None),
            None => return tool_error(id, "SLAM map grid is malformed".into()),
        },
    };
    let build = || -> Result<(Value, Option<Vec<u8>>, Option<Value>), String> {
        // summary needs no crop; grid/image crop around the center first so
        // the rendered area is bounded even for a huge map.
        let cropped = match representation {
            "summary" => grid.clone(),
            _ => robot_server::map_view::crop_map(&grid, (center.0, center.1, radius_m))?,
        };
        let (served, pose) = match representation {
            "summary" => (cropped.clone(), None),
            _ => (
                robot_server::map_view::downsample(&cropped, downsample)?,
                png_pose,
            ),
        };
        let metadata = robot_server::map_view::metadata(&served)?;
        let png = match representation {
            "image" => Some(robot_server::map_view::render_png(&served, pose)?),
            _ => None,
        };
        let out_grid = (representation == "grid").then_some(served);
        Ok((metadata, png, out_grid))
    };
    let (metadata, png, out_grid) = match build() {
        Ok(v) => v,
        Err(reason) => return tool_error(id, format!("map processing failed: {reason}")),
    };
    let mut structured = json!({
        "version": version,
        "representation": representation,
        "pose_source": pose_source,
        "radius_m": if representation == "summary" { Value::Null } else { json!(radius_m) },
        "metadata": metadata,
    });
    if let Some(out_grid) = out_grid {
        structured["grid"] = out_grid;
    }
    let content = match &png {
        Some(bytes) => vec![
            json!({"type":"image","data": STANDARD.encode(bytes), "mimeType": "image/png"}),
            json!({"type":"text","text": structured.to_string()}),
        ],
        None => vec![json!({"type":"text","text": structured.to_string()})],
    };
    json_response(
        json!({"jsonrpc":"2.0","id":id,"result":{
            "content": content,
            "structuredContent": structured,
            "isError": false}}),
        StatusCode::OK,
    )
}

/// The camera tool: one frame on the shared safe MJPEG client, bounded in
/// wall clock so the HTTP request cannot outlive the caller's patience.
/// The base64 image rides in an MCP ImageContent block only; the text mirror
/// carries the metadata alone.
async fn camera_frame(id: Value, app: &App, args: Value) -> Response {
    let (max_age_ms, max_bytes) = match parse_frame_args(&args) {
        Ok(v) => v,
        Err(message) => return tool_error(id, message),
    };
    let camera = app.camera.clone();
    let (frame, received) = match tokio::time::timeout(
        FRAME_CALL_LIMIT,
        camera.snapshot(max_bytes, Duration::from_millis(max_age_ms)),
    )
    .await
    {
        Ok(Ok(frame)) => frame,
        Ok(Err(reason)) => return tool_error(id, format!("camera frame unavailable: {reason}")),
        Err(_) => return tool_error(id, "camera frame call timed out".into()),
    };
    let frame_id = FRAME_ID.fetch_add(1, std::sync::atomic::Ordering::Relaxed) + 1;
    let age_hint_ms = received.elapsed().as_millis() as u64;
    let mut structured = json!({
        "frame_id": frame_id,
        "bytes": frame.len(),
        "age_hint_ms": age_hint_ms,
        "timing_basis": "received",
        "transform": "none",
        "overlay": "none",
    });
    if let Some((width, height)) = jpeg_dimensions(&frame) {
        structured["width"] = json!(width);
        structured["height"] = json!(height);
    }
    json_response(
        json!({"jsonrpc":"2.0","id":id,"result":{
            "content": [
                {"type":"image","data": STANDARD.encode(&frame), "mimeType": "image/jpeg"},
                {"type":"text","text": structured.to_string()}
            ],
            "structuredContent": structured,
            "isError": false}}),
        StatusCode::OK,
    )
}

pub async fn endpoint(State(app): State<App>, headers: HeaderMap, body: String) -> Response {
    // The MCP endpoint drives a robot: never let a foreign page reach it.
    if !crate::allowed_origin(&headers) {
        return StatusCode::FORBIDDEN.into_response();
    }
    if body.len() > BODY_LIMIT {
        return StatusCode::PAYLOAD_TOO_LARGE.into_response();
    }
    let request: Value = match serde_json::from_str(&body) {
        Ok(v) => v,
        Err(_) => return StatusCode::BAD_REQUEST.into_response(),
    };
    // Tools with bounded network I/O run before the sync dispatcher so their
    // wall clock bounds are real timeouts, not guesses: the camera snapshot
    // and the supervisor drive lease round trip.
    if request.get("method").and_then(Value::as_str) == Some("tools/call") {
        let params = request.get("params").cloned().unwrap_or(Value::Null);
        match params.get("name").and_then(Value::as_str) {
            Some("robot_get_camera_frame") => {
                let id = request.get("id").cloned().unwrap_or(Value::Null);
                let args = params.get("arguments").cloned().unwrap_or(Value::Null);
                return camera_frame(id, &app, args).await;
            }
            Some("robot_drive_for") => {
                let id = request.get("id").cloned().unwrap_or(Value::Null);
                let args = params.get("arguments").cloned().unwrap_or(Value::Null);
                return drive_for_async(id, &app, args).await;
            }
            Some("robot_stop") => {
                let id = request.get("id").cloned().unwrap_or(Value::Null);
                return mcp_stop_async(id, &app).await;
            }
            _ => {}
        }
    }
    // Bound dispatch work per call so a heavy caller cannot stall the panel.
    let app = app.clone();
    match tokio::time::timeout(Duration::from_secs(2), std::future::ready(())).await {
        Ok(()) => dispatch(&app, request),
        Err(_) => error(Value::Null, -32603, "dispatch timeout"),
    }
}

#[allow(dead_code)]
fn _unused(_: Failure) {}

// ---- #60: bounded motion tools ----

/// `robot_drive_for`: a bounded body-velocity action validated against the
/// operator grant, recorded idempotently and sent to the sole Player writer.
/// The physical deadline is enforced by the supervisor's DRIVE lease (#55);
/// this host-side path only starts bounded motions and tracks outcomes.
/// Async entry (#64): the supervisor round trip needs a real timeout
/// context; validation and grant checks run inline first.
async fn drive_for_async(id: Value, app: &App, args: Value) -> Response {
    match prepare_drive(id, app, args) {
        Ok((id, record, address, token)) => {
            // A replayed record must never repeat the physical side effect:
            // skip the network round trip entirely.
            if !record.fresh {
                let mut payload = mcp_drive::record_json(&record);
                payload["replay"] = json!(true);
                return json_response(
                    json!({"jsonrpc":"2.0","id":id,"result":{
                        "content": [{"type":"text","text": payload.to_string()}],
                        "structuredContent": payload,
                        "isError": false}}),
                    StatusCode::OK,
                );
            }
            let (vx, vyaw, ms) = record.requested;
            let outcome = tokio::time::timeout(
                Duration::from_secs(12),
                crate::supervisor_drive::drive(&address, &token, vx, vyaw, ms),
            )
            .await;
            finish_drive(
                id,
                app,
                record,
                outcome.map_err(|_| "timeout".into()).and_then(|r| r),
            )
        }
        Err(response) => response,
    }
}
async fn mcp_stop_async(id: Value, app: &App) -> Response {
    let supervisor_stop = match app.supervisor.clone() {
        Some((address, token)) => tokio::time::timeout(
            Duration::from_secs(10),
            crate::supervisor_drive::stop(&address, &token),
        )
        .await
        .ok()
        .and_then(|r| r.ok())
        .map(|reply| reply == "STOPPED")
        .unwrap_or(false),
        None => false,
    };
    stop_common(id, app, supervisor_stop)
}

fn prepare_drive(
    id: Value,
    app: &App,
    args: Value,
) -> Result<(Value, mcp_drive::ActionRecord, String, String), Response> {
    let Some(obj) = args.as_object() else {
        return Err(tool_error(id, "arguments must be an object".into()));
    };
    let get_num = |key: &str| obj.get(key).and_then(Value::as_f64);
    let get_str = |key: &str| obj.get(key).and_then(Value::as_str).map(str::to_owned);
    let (Some(linear), Some(angular), Some(duration), Some(command_id)) = (
        get_num("linear_mps"),
        get_num("angular_radps"),
        obj.get("duration_ms").and_then(Value::as_u64),
        get_str("command_id"),
    ) else {
        return Err(tool_error(
            id,
            "linear_mps, angular_radps, duration_ms and command_id are required".into(),
        ));
    };
    if !linear.is_finite() || !angular.is_finite() {
        return Err(tool_error(id, "speeds must be finite".into()));
    }
    if duration == 0 || duration > 60_000 {
        return Err(tool_error(id, "duration_ms must be 1..60000".into()));
    }
    // MCP control lease must be held (the shared Safety owner from #56).
    let owner = app.safety.lock().unwrap().owner();
    if !owner.is_some_and(|o| o >= 1_000_000) {
        return Err(tool_error(
            id,
            "control not acquired; call robot_acquire_control first".into(),
        ));
    }
    // The bounded motion runs under the robot-local deadline (#64): no
    // supervisor means an explicit error, never a host-lease fallback.
    let Some((address, token)) = app.supervisor.clone() else {
        return Err(tool_error(
            id,
            "no supervisor: robot-local drive deadline unavailable; motion refused".into(),
        ));
    };
    // Grant check + idempotent record + conservative budget spend.
    let epoch = app.grants.lock().unwrap().epoch();
    let mut grants = app.grants.lock().unwrap();
    let mut actions = app.drive_actions.lock().unwrap();
    match actions.submit(
        &mut grants,
        "mcp",
        epoch,
        &command_id,
        linear,
        angular,
        duration,
        std::time::Instant::now(),
    ) {
        Ok(record) => {
            drop(grants);
            Ok((id, record, address, token))
        }
        Err(reason) => Err(tool_error(id, reason)),
    }
}

/// Turn the supervisor round-trip result into the tool response. A replayed
/// non-Sent record short-circuits before any network I/O.
fn finish_drive(
    id: Value,
    app: &App,
    record: mcp_drive::ActionRecord,
    outcome: Result<String, crate::Failure>,
) -> Response {
    if !record.fresh {
        let mut payload = mcp_drive::record_json(&record);
        payload["replay"] = json!(true);
        return json_response(
            json!({"jsonrpc":"2.0","id":id,"result":{
                "content": [{"type":"text","text": payload.to_string()}],
                "structuredContent": payload,
                "isError": false}}),
            StatusCode::OK,
        );
    }
    if !matches!(record.outcome, mcp_drive::Outcome::Sent) {
        return json_response(
            json!({"jsonrpc":"2.0","id":id,"result":{
                "content": [{"type":"text","text": mcp_drive::record_json(&record).to_string()}],
                "structuredContent": mcp_drive::record_json(&record),
                "isError": false}}),
            StatusCode::OK,
        );
    }
    match outcome {
        Ok(reply) if reply == "DRIVING" => json_response(
            json!({"jsonrpc":"2.0","id":id,"result":{
                "content": [{"type":"text","text": mcp_drive::record_json(&record).to_string()}],
                "structuredContent": mcp_drive::record_json(&record),
                "isError": false}}),
            StatusCode::OK,
        ),
        Ok(reply) => {
            let reason = format!("supervisor refused drive: {reply}");
            app.drive_actions
                .lock()
                .unwrap()
                .stop_active(mcp_drive::Outcome::Refused(reason.clone()));
            tool_error(id, reason)
        }
        Err(e) => {
            let reason = format!("supervisor drive failed: {e}");
            app.drive_actions
                .lock()
                .unwrap()
                .stop_active(mcp_drive::Outcome::Refused(reason.clone()));
            tool_error(id, reason)
        }
    }
}

/// Common stop tail (#64): the supervisor owns the local deadline; the host
/// Safety zero stays as a belt-and-braces flush of the shared lease.
#[allow(clippy::too_many_lines)]
fn stop_common(id: Value, app: &App, supervisor_stop: bool) -> Response {
    let stopped = app
        .drive_actions
        .lock()
        .unwrap()
        .stop_active(mcp_drive::Outcome::Stopped);
    let owner = app.safety.lock().unwrap().owner();
    if let Some(owner) = owner.filter(|o| *o >= 1_000_000) {
        app.safety
            .lock()
            .unwrap()
            .command(owner, 0., 0., std::time::Instant::now());
    }
    // When the supervisor confirmed the stop, the motion lease lives there:
    // no host Player write is needed (and none may be emitted, #64).
    let zero_sent = if supervisor_stop {
        true
    } else {
        let mut writer = app.player.clone();
        futures_util::FutureExt::now_or_never(async {
            crate::player::velocity(&mut writer, 0., 0.).await
        })
        .unwrap_or_else(|| Ok(()))
        .is_ok()
    };
    json_response(
        json!({"jsonrpc":"2.0","id":id,"result":{
            "content": [{"type":"text","text": if zero_sent { "stop requested" } else { "stop record cleared; zero send unconfirmed" }}],
            "structuredContent": {
                "supervisor_stopped": supervisor_stop,
                "zero_sent": zero_sent,
                "had_active": stopped.is_some(),
                "note": "отправленный ноль не доказывает физическую остановку; локальный дедлайн — супервизор"
            },
            "isError": false}}),
        StatusCode::OK,
    )
}

/// `robot_get_action`: return the honest recorded outcome of a command id.
fn get_action(id: Value, app: &App, args: Value) -> Response {
    let Some(command_id) = args.get("command_id").and_then(Value::as_str) else {
        return tool_error(id, "command_id is required".into());
    };
    match app.drive_actions.lock().unwrap().get(command_id) {
        Some(record) => json_response(
            json!({"jsonrpc":"2.0","id":id,"result":{
                "content": [{"type":"text","text": mcp_drive::record_json(record).to_string()}],
                "structuredContent": mcp_drive::record_json(record),
                "isError": false}}),
            StatusCode::OK,
        ),
        None => tool_error(id, format!("unknown command_id {command_id}")),
    }
}

/// `robot_say` (#63): shared acceptance path with the panel. Returns the
/// queued task id; polling the outcome happens through job_state by id in a
/// later call (the observe-style honest separation accepted/sent/completed).
fn say(id: Value, app: &App, args: Value) -> Response {
    let Some(obj) = args.as_object() else {
        return tool_error(id, "arguments must be an object".into());
    };
    let Some(text) = obj.get("text").and_then(Value::as_str) else {
        return tool_error(id, "text is required".into());
    };
    let volume = obj.get("volume").and_then(Value::as_f64);
    if let Some(extra) = obj
        .keys()
        .find(|k| k.as_str() != "text" && k.as_str() != "volume")
    {
        return tool_error(id, format!("unknown argument {extra}"));
    }
    match app.audio.submit(text, volume) {
        crate::audio::Submit::Queued(task) => json_response(
            json!({"jsonrpc":"2.0","id":id,"result":{
                    "content": [{"type":"text","text": format!("speech job {task} queued")}],
                    "structuredContent": {
                        "task_id": task,
                        "state": "queued",
                        "volume": volume.unwrap_or_else(|| app.audio.default_volume()),
                        "note": "принято ≠ завершено; итог неизвестен до подтверждения состояния"
                    },
                    "isError": false}}),
            StatusCode::OK,
        ),
        crate::audio::Submit::Invalid(what) => tool_error(id, what.into()),
        crate::audio::Submit::Disabled(what) => tool_error(id, format!("tts unavailable: {what}")),
        crate::audio::Submit::Busy => tool_error(id, "audio_busy: one speech job at a time".into()),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn frame_args_validate_ranges_and_defaults() {
        let (age, bytes) = parse_frame_args(&json!({})).unwrap();
        assert_eq!((age, bytes), (5_000, 1024 * 1024));
        assert!(parse_frame_args(&json!({"max_age_ms": 99})).is_err());
        assert!(parse_frame_args(&json!({"max_age_ms": 30_001})).is_err());
        assert!(parse_frame_args(&json!({"max_age_ms": "soon"})).is_err());
        assert!(parse_frame_args(&json!({"max_bytes": 1023})).is_err());
        assert!(parse_frame_args(&json!({"max_bytes": 2 * 1024 * 1024 + 1})).is_err());
        assert!(parse_frame_args(&json!({"nope": 1})).is_err());
        assert!(parse_frame_args(&json!({"max_age_ms": 2.5})).is_err());
        assert_eq!(
            parse_frame_args(&json!({"max_age_ms": 30_000, "max_bytes": 1024})).unwrap(),
            (30_000, 1024)
        );
    }

    #[test]
    fn jpeg_dimensions_read_the_first_sof_marker() {
        // SOI, APP0 (declared length 16 = 14 payload bytes), SOF0 480x640.
        let head: Vec<u8> = [0xff, 0xd8, 0xff, 0xe0, 0x00, 0x10]
            .into_iter()
            .chain(std::iter::repeat(0).take(14))
            .collect();
        let sof0: [u8; 10] = [0xff, 0xc0, 0x00, 0x11, 0x08, 0x01, 0xe0, 0x02, 0x80, 0x03];
        let frame = [&head[..], &sof0].concat();
        assert_eq!(jpeg_dimensions(&frame), Some((640, 480)));
        // Progressive JPEG (SOF2) parses the same way.
        let sof2: [u8; 10] = [0xff, 0xc2, 0x00, 0x11, 0x08, 0x00, 0x64, 0x00, 0xc8, 0x01];
        let progressive = [&head[..], &sof2].concat();
        assert_eq!(jpeg_dimensions(&progressive), Some((200, 100)));
        assert_eq!(jpeg_dimensions(&[0xff, 0xd8, 0xff, 0xd9]), None);
        assert_eq!(jpeg_dimensions(b"not a jpeg"), None);
    }

    #[test]
    fn map_args_validate_ranges_and_defaults() {
        let (repr, known, radius, down) = parse_map_args(&json!({})).unwrap();
        assert_eq!((repr, known, radius, down), ("summary", None, 10.0, 1));
        assert!(parse_map_args(&json!({"representation": "hologram"})).is_err());
        assert!(parse_map_args(&json!({"representation": 1})).is_err());
        assert!(parse_map_args(&json!({"known_version": "seven"})).is_err());
        assert!(parse_map_args(&json!({"known_version": -1.5})).is_err());
        assert!(parse_map_args(&json!({"radius_m": 0.05})).is_err());
        assert!(parse_map_args(&json!({"radius_m": 21.0})).is_err());
        assert!(parse_map_args(&json!({"downsample": 0})).is_err());
        assert!(parse_map_args(&json!({"downsample": 17})).is_err());
        assert!(parse_map_args(&json!({"downsample": 2.5})).is_err());
        assert!(parse_map_args(&json!({"nope": 1})).is_err());
        assert!(parse_map_args(&json!("not an object")).is_err());
        assert_eq!(
            parse_map_args(&json!({
                "representation": "grid", "known_version": 3,
                "radius_m": 0.5, "downsample": 8
            }))
            .unwrap(),
            ("grid", Some(3), 0.5, 8)
        );
    }
}
