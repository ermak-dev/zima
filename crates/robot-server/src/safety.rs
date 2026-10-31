//! Exclusive wheel-command ownership, lease and explicit bumper blocking. No I/O here.
use player_bridge::drive::DriveCommand;
use std::time::{Duration, Instant};

pub struct Safety {
    owner: Option<u64>,
    drive: DriveCommand,
    pressed: bool,
    blocked: bool,
    stop_pending: bool,
}

impl Safety {
    pub fn new() -> Self {
        Self {
            owner: None,
            drive: DriveCommand::new(Duration::from_secs(1)),
            pressed: false,
            blocked: false,
            stop_pending: false,
        }
    }

    /// The exclusive control owner shared by wheel and cleaning-motor commands.
    pub fn owner(&self) -> Option<u64> {
        self.owner
    }

    pub fn is_blocked(&self) -> bool {
        self.blocked
    }

    pub fn reset_blocked(&mut self) -> bool {
        if !self.blocked {
            return false;
        }
        self.blocked = false;
        self.drive = DriveCommand::new(Duration::from_secs(1));
        true
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

    pub fn bumper(&mut self, pressed: bool, _now: Instant) {
        if pressed && !self.pressed {
            self.blocked = true;
            self.drive = DriveCommand::new(Duration::from_secs(1));
            self.stop_pending = true;
        }
        self.pressed = pressed;
    }

    pub fn next(&mut self, now: Instant) -> Option<(f64, f64)> {
        // A reset/new command must not overwrite an unsent collision stop.
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
    // FEATURE: explicit bumper blocker with a priority stop and manual reset.
    // SPEC: reset preserves held contact; old commands never resume and stop cannot be overwritten.
    // IMPLEMENTATION: latched block and separate stop slot; no odometry or SLAM gate.
    // RESULT: IMPLEMENTED — reset, rearm, priority, ownership and lease verified offline.
    // STATUS: DONE
    #[test]
    fn collision_stop_survives_reset_and_new_command_before_drive_tick() {
        let now = Instant::now();
        let mut s = Safety::new();
        s.command(1, 0.1, 0., now);
        s.bumper(true, now);
        assert!(s.is_blocked());
        s.command(1, -0.1, 0., now);
        s.reset_blocked();
        assert!(!s.is_blocked());
        s.command(1, -0.06, 0., now);
        assert_eq!(s.next(now), Some((0., 0.)));
        assert_eq!(s.next(now), Some((-0.06, 0.)));
        s.bumper(true, now);
        assert!(!s.is_blocked());
        s.bumper(false, now);
        s.bumper(true, now);
        assert!(s.is_blocked());
    }

    #[test]
    fn contact_blocks_without_owner_until_explicit_reset() {
        let now = Instant::now();
        let mut s = Safety::new();
        assert!(!s.is_blocked());
        assert_eq!(s.next(now), None);
        s.bumper(true, now);
        assert!(s.is_blocked());
        assert_eq!(s.next(now), Some((0., 0.)));
        s.command(1, 0.1, 0., now);
        s.bumper(false, now);
        assert!(s.is_blocked());
        assert_eq!(s.next(now + Duration::from_secs(30)), None);
        assert!(s.reset_blocked());
        assert_eq!(s.next(now), None);
        s.command(2, -0.06, 0., now);
        assert_eq!(s.next(now), Some((-0.06, 0.)));
    }

    #[test]
    fn reset_preserves_contact_level_and_never_resumes_old_commands() {
        let now = Instant::now();
        let mut s = Safety::new();
        s.command(1, 0.1, 0., now);
        s.next(now);
        s.bumper(true, now);
        assert_eq!(s.next(now), Some((0., 0.)));
        assert!(s.reset_blocked());
        assert!(!s.reset_blocked());
        s.bumper(true, now);
        assert!(!s.is_blocked());
        assert_eq!(s.next(now), None);
        s.command(1, -0.06, 0., now);
        assert_eq!(s.next(now), Some((-0.06, 0.)));
        s.bumper(false, now);
        assert_eq!(s.next(now), None);
        s.bumper(true, now);
        assert!(s.is_blocked());
        assert_eq!(s.next(now), Some((0., 0.)));
    }

    #[test]
    fn disconnect_does_not_clear_block_and_idle_reset_does_not_stop() {
        let now = Instant::now();
        let mut s = Safety::new();
        s.command(1, 0.1, 0., now);
        assert!(!s.reset_blocked());
        assert_eq!(s.next(now), Some((0.1, 0.)));
        s.bumper(true, now);
        s.next(now);
        s.disconnect(2);
        assert!(s.is_blocked());
        assert_eq!(s.next(now), None);
        s.disconnect(1);
        assert!(s.is_blocked());
        assert_eq!(s.next(now), Some((0., 0.)));
        s.reset_blocked();
        assert_eq!(s.next(now), None);
        s.command(2, -0.1, 0., now);
        assert_eq!(s.next(now), Some((-0.1, 0.)));
    }

    #[test]
    fn angular_ceiling_preserves_lower_speeds() {
        let now = Instant::now();
        let mut s = Safety::new();
        for (requested, expected) in [
            (0.4, 0.4),
            (-0.4, -0.4),
            (1., 1.),
            (-2., -2.),
            (3.14, 3.14),
            (-3.14, -3.14),
            (9., 3.14),
            (-9., -3.14),
        ] {
            s.command(1, 0.1, requested, now);
            assert_eq!(s.next(now), Some((0.1, expected)));
        }
    }

    #[test]
    fn latest_only_lease_and_owner_transitions() {
        let now = Instant::now();
        let mut s = Safety::new();
        assert_eq!(s.next(now), None);
        s.disconnect(5);
        assert_eq!(s.next(now), None);
        for i in 0..10000 {
            s.command(1, i as f64 / 10000., -9., now);
        }
        s.command(2, -0.1, 0., now);
        assert_eq!(s.next(now), Some((0.5, -3.14)));
        assert_eq!(s.next(now), None);
        s.disconnect(2);
        assert_eq!(s.next(now), None);
        assert_eq!(s.next(now + Duration::from_secs(1)), Some((0., 0.)));
        assert_eq!(s.next(now + Duration::from_secs(2)), None);
        s.command(1, 0.08, 0., now + Duration::from_secs(2));
        assert_eq!(s.next(now + Duration::from_secs(4)), None);
        s.disconnect(1);
        assert_eq!(s.next(Instant::now()), Some((0., 0.)));
        s.command(2, -1., 9., now + Duration::from_secs(5));
        assert_eq!(s.next(now + Duration::from_secs(5)), Some((-0.5, 3.14)));
    }
}
