//! Explicit, bounded reverse pulses under a bumper latch. Normal driving
//! stays blocked; neither reset nor recovery changes the hardware mask.
use crate::{
    grants::{Admission, ControlGrant, Grants},
    mcp_drive::{self, ActionKind, ActionRecord, Outcome},
    safety::{RECOVERY_MAX_MS, RECOVERY_SPEED},
    App,
};
use serde_json::{json, Value};
use std::time::{Duration, Instant};

#[derive(Clone, Copy)]
pub enum Actor {
    Mcp,
    Panel(u64),
}

pub fn failure(message: impl Into<String>) -> Value {
    let message = message.into();
    json!({"isError":true,"content":[{"type":"text","text":message}],"error":message})
}
fn record_reply(record: &ActionRecord, replay: bool) -> Value {
    if let Outcome::Refused(reason) = &record.outcome {
        return failure(reason);
    }
    let mut value = mcp_drive::record_json(record);
    if replay {
        value["replay"] = json!(true);
    }
    json!({"isError":false,"content":[{"type":"text","text":value.to_string()}],"structuredContent":value})
}
fn parse(args: &Value) -> Result<(&str, u64), &'static str> {
    let obj = args.as_object().ok_or("arguments must be an object")?;
    if obj.keys().any(|k| k != "command_id" && k != "duration_ms") {
        return Err("unknown arguments: recovery fixes speed and direction");
    }
    let id = obj
        .get("command_id")
        .and_then(Value::as_str)
        .filter(|id| !id.is_empty() && id.len() <= 64)
        .ok_or("command_id must be 1..64 bytes")?;
    let ms = match obj.get("duration_ms") {
        None => 300,
        Some(v) => v
            .as_u64()
            .ok_or("duration_ms must be an integer in 1..500")?,
    };
    if ms == 0 || ms > RECOVERY_MAX_MS {
        return Err("duration_ms must be 1..500");
    }
    Ok((id, ms))
}

pub async fn start(app: &App, actor: Actor, args: Value) -> Value {
    let (id, ms) = match parse(&args) {
        Ok(v) => v,
        Err(e) => return failure(e),
    };
    let Some((address, token)) = app.supervisor.clone() else {
        return failure("no supervisor: robot-local recovery deadline unavailable");
    };
    let command_id = match actor {
        Actor::Mcp => id.to_owned(),
        Actor::Panel(owner) => format!("panel-{owner}-{id}"),
    };
    let record = {
        // Match the motor path: Safety precedes grant and action locks.
        let mut safety = app.safety.lock().unwrap();
        let owner = match actor {
            Actor::Mcp => match safety.owner() {
                Some(id) if id < 1_000_000 => return failure("panel client owns control; BUSY"),
                Some(id) => id,
                None => 1_000_000,
            },
            Actor::Panel(id) => {
                if safety.owner().is_some_and(|owner| owner != id) {
                    return failure("another client owns control; BUSY");
                }
                id
            }
        };
        let now = Instant::now();
        // Panel control has the same network-local authority as ordinary WS
        // teleop, but still shares the hard per-latch recovery budget.
        let mut panel_grants = Grants::default();
        panel_grants.install(ControlGrant {
            subject: "panel".into(),
            max_linear_mps: RECOVERY_SPEED.abs(),
            max_angular_radps: 0.0,
            max_action_ms: RECOVERY_MAX_MS,
            total_budget_ms: RECOVERY_MAX_MS,
            spent_ms: 0,
            expires_at: now + Duration::from_secs(1),
        });
        let mut mcp_grants = app.grants.lock().unwrap();
        let (grants, subject, epoch) = match actor {
            Actor::Mcp => {
                let epoch = mcp_grants.epoch();
                (&mut *mcp_grants, "mcp", epoch)
            }
            Actor::Panel(_) => (&mut panel_grants, "panel", 0),
        };
        if let Admission::Denied(reason) = grants.admit(subject, 0.0, 0.0, 0, now) {
            return failure(format!("no operator grant: {reason}"));
        }
        let mut actions = app.drive_actions.lock().unwrap();
        // Retries never start a new pulse, spend another budget or require
        // sensors that may have expired since the original pulse completed.
        if actions.get(&command_id).is_some() {
            return match actions.submit_kind(
                grants,
                subject,
                epoch,
                &command_id,
                RECOVERY_SPEED,
                0.0,
                ms,
                now,
                ActionKind::BumperRecovery,
            ) {
                Ok(record) => record_reply(&record, true),
                Err(reason) => failure(reason),
            };
        }
        if safety.is_recovering() {
            return failure("recovery active; BUSY");
        }
        if let Err(reason) = safety.recovery_ready(now) {
            return failure(reason);
        }
        if let Admission::Denied(reason) = grants.admit(subject, RECOVERY_SPEED, 0.0, ms, now) {
            return failure(reason);
        }
        let record = match actions.submit_kind(
            grants,
            subject,
            epoch,
            &command_id,
            RECOVERY_SPEED,
            0.0,
            ms,
            now,
            ActionKind::BumperRecovery,
        ) {
            Ok(record) => record,
            Err(reason) => return failure(reason),
        };
        if let Err(reason) = safety.begin_recovery(owner, &command_id, ms, now) {
            actions.stop_active(Outcome::Refused(reason.into()));
            return failure(reason);
        }
        record
    };
    let (tx, rx) = tokio::sync::oneshot::channel();
    // Once accepted, the local cleanup task survives a cancelled HTTP call.
    let app = app.clone();
    tokio::spawn(async move {
        run(app, actor, record, address, token, tx).await;
    });
    rx.await
        .unwrap_or_else(|_| failure("recovery executor unavailable; outcome unknown"))
}

fn valid(app: &App, actor: Actor, record: &ActionRecord) -> bool {
    let now = Instant::now();
    if !app
        .safety
        .lock()
        .unwrap()
        .recovery_valid(&record.command_id, now)
    {
        return false;
    }
    if let Actor::Mcp = actor {
        let grants = app.grants.lock().unwrap();
        if grants.epoch() != record.epoch
            || matches!(grants.admit("mcp", 0.0, 0.0, 0, now), Admission::Denied(_))
        {
            return false;
        }
    }
    let actions = app.drive_actions.lock().unwrap();
    actions.active_id().as_deref() == Some(record.command_id.as_str())
        && actions
            .get(&record.command_id)
            .is_some_and(|r| r.outcome == Outcome::Sent)
}

async fn stop(address: &str, token: &str) {
    let _ = tokio::time::timeout(
        Duration::from_secs(2),
        crate::supervisor_drive::stop(address, token),
    )
    .await;
}
fn finish(app: &App, record: &ActionRecord, outcome: Outcome) {
    // Keep ownership until an explicit release, including after recovery.
    let mut safety = app.safety.lock().unwrap();
    let mut actions = app.drive_actions.lock().unwrap();
    if actions.active_id().as_deref() == Some(record.command_id.as_str()) {
        actions.stop_active(outcome);
    }
    safety.end_recovery(&record.command_id);
}

async fn run(
    app: App,
    actor: Actor,
    record: ActionRecord,
    address: String,
    token: String,
    reply: tokio::sync::oneshot::Sender<Value>,
) {
    let mut writer = app.player.clone();
    let zero = tokio::time::timeout(
        Duration::from_millis(250),
        crate::player::velocity(&mut writer, 0.0, 0.0),
    )
    .await;
    if !matches!(zero, Ok(Ok(()))) || !valid(&app, actor, &record) {
        finish(&app, &record, Outcome::Stopped);
        let _ = reply.send(failure(
            "recovery cancelled before DRIVE; zero or hazard admission failed",
        ));
        return;
    }
    let ms = record.requested.2;
    let drive = tokio::time::timeout(
        Duration::from_secs(12),
        crate::supervisor_drive::drive(&address, &token, RECOVERY_SPEED, 0.0, ms),
    );
    tokio::pin!(drive);
    let mut cancelled = false;
    let sent = loop {
        tokio::select! {
            sent = &mut drive => break sent,
            _ = tokio::time::sleep(Duration::from_millis(20)), if !cancelled => {
                if !valid(&app, actor, &record) {
                    cancelled = true;
                    stop(&address, &token).await;
                }
            }
        }
    };
    if !matches!(sent, Ok(Ok(ref line)) if line == "DRIVING") {
        stop(&address, &token).await;
        finish(&app, &record, Outcome::Stopped);
        let _ = reply.send(failure(
            "supervisor recovery unconfirmed; stop requested, outcome unknown",
        ));
        return;
    }
    if cancelled || !valid(&app, actor, &record) {
        stop(&address, &token).await;
        finish(&app, &record, Outcome::Stopped);
        let _ = reply.send(failure(
            "recovery cancelled by hazard, owner or grant change",
        ));
        return;
    }
    let _ = reply.send(record_reply(&record, false));
    let deadline = tokio::time::Instant::now() + Duration::from_millis(ms);
    loop {
        tokio::select! {
            _ = tokio::time::sleep_until(deadline) => {
                finish(&app, &record, Outcome::Completed { zero_observed: false });
                return;
            }
            _ = tokio::time::sleep(Duration::from_millis(20)) => {
                if !valid(&app, actor, &record) {
                    stop(&address, &token).await;
                    finish(&app, &record, Outcome::Stopped);
                    return;
                }
            }
        }
    }
}
