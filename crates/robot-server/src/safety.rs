//! Safety sensor gate (#65): bumper, cliff and drop edges participate in one
//! latching block state machine. Pure state, no I/O.
//!
//! Contract (the "adequate sensor algorithm" from the live run):
//! * Bumper press (rising edge) latches `blocked` with reason "bumper".
//! * Cliff (any cliff channel active) and drop (any drop channel active)
//!   latch `blocked` immediately — these are level-sensitive: while the
//!   condition is present every frame re-asserts it.
//! * A manual reset clears the latch but is REFUSED while any latching
//!   condition is still physically present (bumper pressed, cliff or drop
//!   active): resetting under contact would allow driving further into the
//!   hazard. The panel gets an explicit "still_pressed" answer instead.
//! * After a successful reset, a present-but-not-rising bumper does not
//!   re-latch (edge semantics preserved), but cliff/drop levels re-latch
//!   every frame while present (level semantics).
use player_bridge::drive::DriveCommand;
use std::time::{Duration, Instant};

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum BlockReason {
    Bumper,
    Cliff,
    Drop,
}

pub struct Safety {
    owner: Option<u64>,
    drive: DriveCommand,
    pressed: bool,
    blocked: bool,
    block_reason: Option<BlockReason>,
    stop_pending: bool,
    /// Level-sensitive hazards from the latest sensor frame.
    cliff_active: bool,
    drop_active: bool,
}

impl Safety {
    pub fn new() -> Self {
        Self {
            owner: None,
            drive: DriveCommand::new(Duration::from_secs(1)),
            pressed: false,
            blocked: false,
            block_reason: None,
            stop_pending: false,
            cliff_active: false,
            drop_active: false,
        }
    }

    pub fn is_blocked(&self) -> bool {
        self.blocked
    }

    /// The exclusive control owner shared by wheel and cleaning-motor commands.
    pub fn owner(&self) -> Option<u64> {
        self.owner
    }

    pub fn block_reason(&self) -> Option<BlockReason> {
        self.block_reason
    }

    /// Manual reset: refused while a latching condition is still present.
    /// Returns Ok(()) when the latch cleared, Err(reason) when it cannot.
    pub fn reset_blocked(&mut self) -> Result<(), BlockReason> {
        if !self.blocked {
            return Ok(());
        }
        if let Some(reason) = self.hazard_present() {
            // Keep the latch; the operator sees why the reset is refused.
            return Err(reason);
        }
        self.blocked = false;
        self.block_reason = None;
        self.drive = DriveCommand::new(Duration::from_secs(1));
        Ok(())
    }

    /// The still-present physical condition that keeps the latch on.
    fn hazard_present(&self) -> Option<BlockReason> {
        if self.cliff_active {
            return Some(BlockReason::Cliff);
        }
        if self.drop_active {
            return Some(BlockReason::Drop);
        }
        if self.pressed {
            return Some(BlockReason::Bumper);
        }
        None
    }

    pub fn command(&mut self, id: u64, v: f64, w: f64, now: Instant) {
        if self.blocked || self.owner.is_some_and(|owner| owner != id) {
            return;
        }
        self.owner = Some(id);
        self.drive.set(v, w, now);
    }

    pub fn disconnect(&mut self, id: u64) {
        if self.owner != Some(id) {
            return;
        }
        self.owner = None;
        self.drive = DriveCommand::new(Duration::from_secs(1));
        self.stop_pending = true;
    }

    /// Bumper frame: rising edge latches with the bumper reason.
    pub fn bumper(&mut self, pressed: bool, _now: Instant) {
        if pressed && !self.pressed {
            self.blocked = true;
            self.block_reason = Some(BlockReason::Bumper);
            self.drive = DriveCommand::new(Duration::from_secs(1));
            self.stop_pending = true;
        }
        self.pressed = pressed;
    }

    /// Cliff frame (level-sensitive): any active channel latches every frame
    /// while present; it also heals only via a manual reset when clear.
    pub fn cliff(&mut self, active: bool, now: Instant) {
        self.cliff_active = active;
        if active {
            self.blocked = true;
            self.block_reason = Some(BlockReason::Cliff);
            self.drive = DriveCommand::new(Duration::from_secs(1));
            self.stop_pending = true;
            let _ = now;
        }
    }

    /// Drop frame (level-sensitive), same contract as cliff.
    pub fn drop_sensor(&mut self, active: bool, now: Instant) {
        self.drop_active = active;
        if active {
            self.blocked = true;
            self.block_reason = Some(BlockReason::Drop);
            self.drive = DriveCommand::new(Duration::from_secs(1));
            self.stop_pending = true;
            let _ = now;
        }
    }

    pub fn next(&mut self, now: Instant) -> Option<(f64, f64)> {
        if self.stop_pending {
            self.stop_pending = false;
            return Some((0., 0.));
        }
        if self.blocked {
            return None;
        }
        self.drive.next(now)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // FEATURE: hazard state machine with bumper edge and cliff/drop levels.
    // SPEC: bumper rising edge latches; cliff/drop latch every frame while
    // present; reset is refused while the hazard is still physically present;
    // after a clean reset a still-pressed bumper does not re-latch until it
    // is released and pressed again.
    // IMPLEMENTATION: BlockReason + level flags in Safety; reset_blocked
    // returns Result.
    // RESULT: IMPLEMENTED — offline unit checks pass.
    // STATUS: DONE

    #[test]
    fn bumper_edge_latches_and_reset_requires_release() {
        let now = Instant::now();
        let mut s = Safety::new();
        s.bumper(true, now);
        assert_eq!(s.block_reason(), Some(BlockReason::Bumper));
        assert!(s.is_blocked());
        // Still pressed: reset refused.
        assert_eq!(s.reset_blocked(), Err(BlockReason::Bumper));
        assert!(s.is_blocked(), "latch stays on while pressed");
        // Release, then reset clears.
        s.bumper(false, now);
        assert_eq!(s.reset_blocked(), Ok(()));
        assert!(!s.is_blocked());
        // A repeated clear frame does not re-latch (edge semantics).
        s.bumper(false, now);
        assert!(!s.is_blocked());
    }

    #[test]
    fn cliff_and_drop_are_level_sensitive() {
        let now = Instant::now();
        let mut s = Safety::new();
        s.cliff(true, now);
        assert_eq!(s.block_reason(), Some(BlockReason::Cliff));
        // Cliff still present: reset refused even though bumper is clear.
        assert_eq!(s.reset_blocked(), Err(BlockReason::Cliff));
        // Clear the level, reset succeeds, then a fresh cliff re-latches.
        s.cliff(false, now);
        assert_eq!(s.reset_blocked(), Ok(()));
        s.cliff(true, now);
        assert!(s.is_blocked());
        // Drop behaves the same and overrides the reason while active.
        s.drop_sensor(true, now);
        assert_eq!(s.block_reason(), Some(BlockReason::Drop));
        s.drop_sensor(false, now);
        s.cliff(false, now);
        assert_eq!(s.reset_blocked(), Ok(()));
    }

    #[test]
    fn after_clean_reset_pressed_bumper_does_not_re_latch_until_repress() {
        // The live-run trap: latch, release, latch again, reset — the bumper
        // is still physically pressed from the second press.
        let now = Instant::now();
        let mut s = Safety::new();
        s.bumper(true, now);
        s.bumper(false, now);
        s.bumper(true, now);
        assert!(s.is_blocked());
        // Cannot reset while pressed; robot must back off first.
        assert_eq!(s.reset_blocked(), Err(BlockReason::Bumper));
        // Back off (bumper clears), reset, motion resumes. The collision
        // stop is still queued: the first next() flushes the priority zero
        // before the new command runs.
        s.bumper(false, now);
        assert_eq!(s.reset_blocked(), Ok(()));
        s.command(1, -0.1, 0., now);
        assert_eq!(s.next(now), Some((0.0, 0.0)), "priority stop first");
        assert_eq!(s.next(now), Some((-0.1, 0.)));
    }

    #[test]
    fn priority_stop_and_lease_semantics_preserved() {
        let now = Instant::now();
        let mut s = Safety::new();
        s.command(1, 0.1, 0., now);
        assert_eq!(s.next(now), Some((0.1, 0.)));
        s.bumper(true, now);
        assert_eq!(s.next(now), Some((0., 0.)));
        s.command(1, -0.1, 0., now); // refused: still blocked
        s.bumper(false, now);
        assert!(s.is_blocked());
        assert_eq!(s.reset_blocked(), Ok(()));
        s.command(1, -0.1, 0., now);
        // The single queued collision stop was already flushed above, so the
        // new command runs immediately.
        assert_eq!(s.next(now), Some((-0.1, 0.)));
        // Ownership transitions unchanged: client 2 cannot command while 1
        // owns; owner 1's command keeps the lease until it expires.
        s.command(2, 0.1, 0., now);
        assert_eq!(s.next(now), None, "owner 1 lease still fresh");
        s.disconnect(1);
        assert_eq!(s.next(Instant::now()), Some((0., 0.)));
        s.command(2, 0.1, 0., Instant::now());
        assert_eq!(s.next(Instant::now()), Some((0.1, 0.)));
    }
}
