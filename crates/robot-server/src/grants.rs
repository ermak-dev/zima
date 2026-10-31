//! Operator-granted control admission (#56): a bounded session permit that
//! gates MCP control commands. The permit is created only outside the LLM's
//! authority (env/config from the operator), carries hard ceilings and a
//! movement budget, and expires. Wheel/motor ownership itself stays in
//! `Safety`; this module only decides whether an MCP subject may enter it.
//! No I/O here — pure state machine, unit-tested.
use std::time::Instant;

/// Hard ceilings for an MCP control session. Units are SI; the budget is
/// consumed conservatively: an action with an unknown outcome still spends
/// its full requested duration.
#[derive(Clone, Debug)]
pub struct ControlGrant {
    pub subject: String,
    pub max_linear_mps: f64,
    pub max_angular_radps: f64,
    pub max_action_ms: u64,
    pub total_budget_ms: u64,
    pub expires_at: Instant,
    pub(crate) spent_ms: u64,
}

#[derive(Debug, PartialEq)]
pub enum Admission {
    /// The action fits the ceilings and the remaining budget.
    Allowed { remaining_budget_ms: u64 },
    /// The grant exists but this action exceeds a ceiling or the budget.
    Denied(&'static str),
}

/// Why a subject has no usable grant.
#[derive(Debug, PartialEq)]
pub enum GrantError {
    NoGrant,
    Expired,
    SubjectMismatch,
}

#[derive(Default)]
pub struct Grants {
    current: Option<ControlGrant>,
    epoch: u64,
}

impl Grants {
    /// The operator installs a grant; this never happens from an LLM request.
    /// Installing replaces any previous grant and bumps the epoch.
    pub fn install(&mut self, grant: ControlGrant) -> u64 {
        self.current = Some(grant);
        self.epoch += 1;
        self.epoch
    }

    /// Revoke immediately; in-flight actions see the next admission check fail.
    pub fn revoke(&mut self) -> bool {
        self.epoch += 1;
        self.current.take().is_some()
    }

    pub fn epoch(&self) -> u64 {
        self.epoch
    }

    fn live_grant(&self, subject: &str, now: Instant) -> Result<&ControlGrant, GrantError> {
        let grant = self.current.as_ref().ok_or(GrantError::NoGrant)?;
        if grant.subject != subject {
            return Err(GrantError::SubjectMismatch);
        }
        if now >= grant.expires_at {
            return Err(GrantError::Expired);
        }
        Ok(grant)
    }

    /// Check one proposed action against the ceilings and remaining budget.
    /// Does not spend anything; `spend` commits after the action is accepted.
    pub fn admit(
        &self,
        subject: &str,
        linear: f64,
        angular: f64,
        action_ms: u64,
        now: Instant,
    ) -> Admission {
        let Ok(grant) = self.live_grant(subject, now) else {
            return Admission::Denied("no active operator grant for this subject");
        };
        if linear.abs() > grant.max_linear_mps || angular.abs() > grant.max_angular_radps {
            return Admission::Denied("action exceeds the granted speed ceilings");
        }
        if action_ms > grant.max_action_ms {
            return Admission::Denied("action exceeds the granted duration ceiling");
        }
        let remaining = grant.total_budget_ms.saturating_sub(grant.spent_ms);
        if action_ms > remaining {
            return Admission::Denied("action exceeds the remaining movement budget");
        }
        Admission::Allowed {
            remaining_budget_ms: remaining - action_ms,
        }
    }

    /// Conservatively spend the full requested duration (unknown physical
    /// outcome must still consume the budget). No-op when the grant is gone.
    pub fn spend(&mut self, subject: &str, action_ms: u64, now: Instant) -> bool {
        if self.live_grant(subject, now).is_err() {
            return false;
        }
        if let Some(grant) = self.current.as_mut() {
            grant.spent_ms = grant.spent_ms.saturating_add(action_ms);
            true
        } else {
            false
        }
    }

    pub fn remaining_budget_ms(&self, subject: &str, now: Instant) -> Option<u64> {
        self.live_grant(subject, now)
            .ok()
            .map(|g| g.total_budget_ms.saturating_sub(g.spent_ms))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::Duration;

    fn grant(subject: &str) -> ControlGrant {
        ControlGrant {
            subject: subject.into(),
            max_linear_mps: 0.2,
            max_angular_radps: 1.0,
            max_action_ms: 2000,
            total_budget_ms: 10_000,
            expires_at: Instant::now() + Duration::from_secs(60),
            spent_ms: 0,
        }
    }

    // FEATURE: operator-granted admission with ceilings, budget and expiry.
    // SPEC: no grant denies; ceilings/budget bound each action; spending is
    // conservative (full request), expiry and subject binding are enforced,
    // revoke bumps the epoch and kills the grant.
    // IMPLEMENTATION: pure Grants state machine; install/revoke happen only
    // from operator-side paths, never from an LLM request.
    // RESULT: IMPLEMENTED — offline unit checks pass.
    // STATUS: DONE
    #[test]
    fn admission_requires_ceiling_and_budget_checks() {
        let now = Instant::now();
        let mut g = Grants::default();
        assert_eq!(
            g.admit("llm", 0.1, 0.0, 500, now),
            Admission::Denied("no active operator grant for this subject")
        );
        g.install(grant("llm"));
        assert_eq!(
            g.admit("llm", 0.1, 0.0, 500, now),
            Admission::Allowed {
                remaining_budget_ms: 9_500
            }
        );
        assert_eq!(
            g.admit("llm", 0.3, 0.0, 500, now),
            Admission::Denied("action exceeds the granted speed ceilings")
        );
        assert_eq!(
            g.admit("llm", 0.1, 2.0, 500, now),
            Admission::Denied("action exceeds the granted speed ceilings")
        );
        assert_eq!(
            g.admit("llm", 0.1, 0.0, 3000, now),
            Admission::Denied("action exceeds the granted duration ceiling")
        );
        // 11_000 ms exceeds both the action ceiling and the budget; the
        // duration ceiling is checked first. Budget exhaustion is covered
        // by the spending test below.
        assert_eq!(
            g.admit("llm", 0.1, 0.0, 11_000, now),
            Admission::Denied("action exceeds the granted duration ceiling")
        );
        // A different subject cannot use the grant.
        assert_eq!(
            g.admit("other", 0.1, 0.0, 500, now),
            Admission::Denied("no active operator grant for this subject")
        );
    }

    #[test]
    fn spending_is_conservative_and_expiry_revokes() {
        let now = Instant::now();
        let mut g = Grants::default();
        let mut short = grant("llm");
        short.expires_at = now + Duration::from_millis(100);
        g.install(short);
        assert!(g.spend("llm", 6_000, now));
        assert_eq!(g.remaining_budget_ms("llm", now), Some(4_000));
        // Spending the rest exhausts the budget.
        assert!(g.spend("llm", 4_000, now));
        assert_eq!(
            g.admit("llm", 0.1, 0.0, 1, now),
            Admission::Denied("action exceeds the remaining movement budget")
        );
        // After expiry nothing is admitted and spending fails.
        let later = now + Duration::from_millis(200);
        assert_eq!(
            g.admit("llm", 0.1, 0.0, 1, later),
            Admission::Denied("no active operator grant for this subject")
        );
        assert!(!g.spend("llm", 1, later));
    }

    #[test]
    fn revoke_and_epoch_bump_invalidate_old_grants() {
        let now = Instant::now();
        let mut g = Grants::default();
        let epoch = g.install(grant("llm"));
        assert_eq!(epoch, 1);
        assert!(g.revoke());
        assert_eq!(g.epoch(), 2);
        assert_eq!(
            g.admit("llm", 0.1, 0.0, 500, now),
            Admission::Denied("no active operator grant for this subject")
        );
        // A fresh install gets a new epoch; the old one is not resurrected.
        let epoch2 = g.install(grant("llm"));
        assert_eq!(epoch2, 3);
        assert!(matches!(
            g.admit("llm", 0.1, 0.0, 500, now),
            Admission::Allowed { .. }
        ));
    }
}
