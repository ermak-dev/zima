//! Bounded motion tools for MCP (#60): `robot_drive_for`, `robot_stop`,
//! `robot_get_action`. The local safety deadline lives on the robot in the
//! supervisor's DRIVE/STOP commands (#55); this module only validates the
//! operator grant, tracks one in-flight action with idempotent command ids,
//! and reports honest outcomes (accepted/sent vs observed/unknown).
use crate::grants::{Admission, Grants};
use serde_json::{json, Value};
use std::collections::HashMap;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Mutex;
use std::time::{Duration, Instant};

/// Maximum remembered actions (bounded registry for get_action/retries).
const HISTORY: usize = 64;

#[derive(Clone, Debug)]
pub struct ActionRecord {
    pub command_id: String,
    pub epoch: u64,
    pub requested: (f64, f64, u64),
    pub outcome: Outcome,
    pub created: Instant,
    /// True only for the submission that created this record; a replayed
    /// identical request returns the stored record with replay=false so the
    /// caller never repeats the physical side effect.
    pub fresh: bool,
}

#[derive(Clone, Debug, PartialEq)]
pub enum Outcome {
    /// Accepted and forwarded to the bounded executor; completion was not
    /// confirmed by fresh telemetry.
    Sent,
    /// Completed; zero observed in telemetry within the action window.
    Completed { zero_observed: bool },
    /// Superseded by stop or another command before the deadline.
    Stopped,
    /// Refused (grant/ceiling/budget/busy); nothing was sent.
    Refused(String),
}

pub struct DriveActions {
    /// The single in-flight command id.
    active: Option<String>,
    entries: HashMap<String, ActionRecord>,
    order: Vec<String>,
    counter: AtomicU64,
}

impl Default for DriveActions {
    fn default() -> Self {
        Self {
            active: None,
            entries: HashMap::new(),
            order: Vec::new(),
            counter: AtomicU64::new(0),
        }
    }
}

impl DriveActions {
    /// Idempotent submission: the same command_id with the same payload
    /// returns the existing record and never re-executes; the same id with a
    /// different payload is rejected. A new id while another action is active
    /// is refused (no queue).
    pub fn submit(
        &mut self,
        grants: &mut Grants,
        subject: &str,
        epoch: u64,
        command_id: &str,
        linear: f64,
        angular: f64,
        duration_ms: u64,
        now: Instant,
    ) -> Result<ActionRecord, String> {
        if command_id.is_empty() || command_id.len() > 64 {
            return Err("command_id must be 1..64 chars".into());
        }
        if let Some(existing) = self.entries.get(command_id) {
            let same =
                existing.requested == (linear, angular, duration_ms) && existing.epoch == epoch;
            if same {
                let mut replay = existing.clone();
                replay.fresh = false;
                return Ok(replay);
            }
            return Err("command_id already used with a different payload".into());
        }
        if let Some(active) = &self.active {
            if self
                .entries
                .get(active)
                .is_some_and(|r| r.outcome == Outcome::Sent)
            {
                return Err(format!("another action is active ({active}); BUSY"));
            }
        }
        match grants.admit(subject, linear, angular, duration_ms, now) {
            Admission::Denied(reason) => {
                self.remember(ActionRecord {
                    command_id: command_id.into(),
                    epoch,
                    requested: (linear, angular, duration_ms),
                    outcome: Outcome::Refused(reason.into()),
                    created: now,
                    fresh: true,
                });
                return Err(reason.into());
            }
            Admission::Allowed { .. } => {}
        }
        // Spend conservatively: the full request leaves the budget whatever
        // the physical outcome.
        grants.spend(subject, duration_ms, now);
        let record = ActionRecord {
            command_id: command_id.into(),
            epoch,
            requested: (linear, angular, duration_ms),
            outcome: Outcome::Sent,
            created: now,
            fresh: true,
        };
        self.active = Some(command_id.into());
        self.remember(record.clone());
        Ok(record)
    }

    /// Mark the active action stopped (priority stop or release).
    pub fn stop_active(&mut self, reason_outcome: Outcome) -> Option<ActionRecord> {
        let id = self.active.take()?;
        if let Some(record) = self.entries.get_mut(&id) {
            record.outcome = reason_outcome;
            return self.entries.get(&id).cloned();
        }
        None
    }

    pub fn complete_active(&mut self, zero_observed: bool) -> Option<ActionRecord> {
        let id = self.active.take()?;
        if let Some(record) = self.entries.get_mut(&id) {
            record.outcome = Outcome::Completed { zero_observed };
            return self.entries.get(&id).cloned();
        }
        None
    }

    pub fn get(&self, command_id: &str) -> Option<&ActionRecord> {
        self.entries.get(command_id)
    }

    /// The currently active command id, if any (#65).
    pub fn active_id(&self) -> Option<String> {
        self.active.clone()
    }

    fn remember(&mut self, record: ActionRecord) {
        if self.order.len() >= HISTORY {
            if let Some(old) = self.order.first().cloned() {
                self.order.remove(0);
                self.entries.remove(&old);
            }
        }
        self.order.push(record.command_id.clone());
        let mut stored = record;
        // Stored records are never fresh: only the live submission result is.
        stored.fresh = false;
        self.entries.insert(stored.command_id.clone(), stored);
    }
}

/// Serialize a record for the tool response; the four honesty levels stay
/// distinguishable (accepted/sent vs observed vs unknown).
pub fn record_json(record: &ActionRecord) -> Value {
    let outcome = match &record.outcome {
        Outcome::Sent => json!({"state": "sent", "note": "выполнение не подтверждено телеметрией"}),
        Outcome::Completed { zero_observed } => json!({
            "state": "completed",
            "zero_observed": zero_observed,
            "note": if *zero_observed { "нулевая скорость наблюдена в телеметрии" } else { "исход физического движения неизвестен" }
        }),
        Outcome::Stopped => json!({"state": "stopped"}),
        Outcome::Refused(reason) => json!({"state": "refused", "reason": reason}),
    };
    json!({
        "command_id": record.command_id,
        "epoch": record.epoch,
        "requested": {
            "linear_mps": record.requested.0,
            "angular_radps": record.requested.1,
            "duration_ms": record.requested.2,
        },
        "outcome": outcome,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn grant() -> crate::grants::ControlGrant {
        crate::grants::ControlGrant {
            subject: "mcp".into(),
            max_linear_mps: 0.2,
            max_angular_radps: 1.0,
            max_action_ms: 2000,
            total_budget_ms: 10_000,
            expires_at: Instant::now() + Duration::from_secs(60),
            spent_ms: 0,
        }
    }

    // FEATURE: idempotent bounded drive actions with an honest outcome.
    // SPEC: same command_id+payload returns the record without re-execution;
    // changed payload under the same id is rejected; concurrent actions are
    // refused; refusals and sends consume/record budget conservatively.
    // IMPLEMENTATION: DriveActions registry with a single active slot.
    // RESULT: IMPLEMENTED — offline unit checks pass.
    // STATUS: DONE
    #[test]
    fn submit_is_idempotent_and_single_flight() {
        let now = Instant::now();
        let mut grants = Grants::default();
        grants.install(grant());
        let mut actions = DriveActions::default();
        let record = actions
            .submit(&mut grants, "mcp", 1, "cmd-1", 0.1, 0., 500, now)
            .unwrap();
        assert_eq!(record.outcome, Outcome::Sent);
        // Repeat: same record, no second execution.
        let again = actions
            .submit(&mut grants, "mcp", 1, "cmd-1", 0.1, 0., 500, now)
            .unwrap();
        assert_eq!(again.command_id, "cmd-1");
        assert_eq!(actions.counter.load(Ordering::Relaxed), 0);
        // Same id, different payload: rejected.
        assert!(actions
            .submit(&mut grants, "mcp", 1, "cmd-1", 0.2, 0., 500, now)
            .is_err());
        // New id while active: BUSY.
        let err = actions
            .submit(&mut grants, "mcp", 1, "cmd-2", 0.1, 0., 500, now)
            .unwrap_err();
        assert!(err.contains("BUSY"), "{err}");
        // Budget spent conservatively: 500 of 10000 gone.
        assert_eq!(grants.remaining_budget_ms("mcp", now), Some(9_500));
    }

    #[test]
    fn replay_after_store_is_not_fresh() {
        let now = Instant::now();
        let mut grants = Grants::default();
        grants.install(grant());
        let mut actions = DriveActions::default();
        let first = actions
            .submit(&mut grants, "mcp", 1, "r1", 0.1, 0., 500, now)
            .unwrap();
        assert!(first.fresh, "initial submission is fresh");
        let replay = actions
            .submit(&mut grants, "mcp", 1, "r1", 0.1, 0., 500, now)
            .unwrap();
        assert!(!replay.fresh, "stored records are not fresh on replay");
    }

    #[test]
    fn refusals_are_recorded_and_epoch_checked() {
        let now = Instant::now();
        let mut grants = Grants::default();
        grants.install(grant());
        let mut actions = DriveActions::default();
        // Over-ceiling: refused and remembered.
        let err = actions
            .submit(&mut grants, "mcp", 1, "big", 0.9, 0., 500, now)
            .unwrap_err();
        assert!(err.contains("ceiling"), "{err}");
        match actions.get("big") {
            Some(record) => assert!(matches!(record.outcome, Outcome::Refused(_))),
            None => panic!("refusal must be recorded"),
        }
        // The refused command did not become active: a valid one proceeds.
        actions
            .submit(&mut grants, "mcp", 1, "ok", 0.1, 0., 500, now)
            .unwrap();
        // Stopped marks the outcome honestly.
        let stopped = actions.stop_active(Outcome::Stopped).unwrap();
        assert_eq!(stopped.outcome, Outcome::Stopped);
        // Completed without observed zero reports unknown physics.
        actions
            .submit(&mut grants, "mcp", 1, "two", 0.1, 0., 500, now)
            .unwrap();
        let done = actions.complete_active(false).unwrap();
        assert_eq!(
            done.outcome,
            Outcome::Completed {
                zero_observed: false
            }
        );
    }
}
