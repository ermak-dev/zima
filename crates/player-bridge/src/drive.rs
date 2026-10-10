//! Drive safety for velocity commands headed to Player.
//!
//! Player keeps executing the last velocity it was given until another
//! one replaces it; nothing on the robot expires it. So the bridge, not
//! its callers, enforces two rules before a command reaches the wheels:
//!
//! - limits: the project ceiling (0.1 m/s, 0.4 rad/s) whatever the
//!   source (Nav2, teleop, a stray `ros2 topic pub`); non-finite values
//!   become zero;
//! - lease: a velocity holds only while commands keep arriving. When the
//!   sender goes quiet for longer than the lease (Nav2 crashed, teleop
//!   key released, publisher killed), the wheels get one zero.
//!
//! The ROS side records commands with [`DriveCommand::set`]; the Player
//! side asks [`DriveCommand::next`] what to send between reads. Only the
//! latest command matters, so there is no queue to fall behind.

use std::time::{Duration, Instant};

/// Linear speed ceiling, m/s (project limit, same as `robot move`).
pub const MAX_LINEAR: f64 = 0.1;
/// Angular speed ceiling, rad/s.
pub const MAX_ANGULAR: f64 = 0.4;

/// Clamp a command to the project limits; non-finite values become zero.
pub fn clamp_velocity(vx: f64, vyaw: f64) -> (f64, f64) {
    fn limit(v: f64, max: f64) -> f64 {
        if v.is_finite() { v.clamp(-max, max) } else { 0.0 }
    }
    (limit(vx, MAX_LINEAR), limit(vyaw, MAX_ANGULAR))
}

/// The latest velocity command and its lease.
pub struct DriveCommand {
    timeout: Duration,
    /// Latest command not yet sent to Player.
    pending: Option<(f64, f64)>,
    /// When the latest command arrived (the lease start).
    last: Option<Instant>,
    /// The last command sent to Player was nonzero.
    moving: bool,
}

impl DriveCommand {
    pub fn new(timeout: Duration) -> Self {
        DriveCommand { timeout, pending: None, last: None, moving: false }
    }

    /// Record a command from ROS (clamped); the latest one wins.
    pub fn set(&mut self, vx: f64, vyaw: f64, now: Instant) {
        self.pending = Some(clamp_velocity(vx, vyaw));
        self.last = Some(now);
    }

    /// What to send to Player now: the pending command while its lease
    /// is fresh, a single zero once the lease lapsed while moving, or
    /// nothing.
    pub fn next(&mut self, now: Instant) -> Option<(f64, f64)> {
        let fresh = self
            .last
            .is_some_and(|t| now.saturating_duration_since(t) < self.timeout);
        // A pending command that outlived its lease is dropped: it must
        // not start the wheels late.
        if let Some(cmd) = self.pending.take() {
            if fresh {
                self.moving = cmd != (0.0, 0.0);
                return Some(cmd);
            }
        }
        if self.moving && !fresh {
            self.moving = false;
            return Some((0.0, 0.0));
        }
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const LEASE: Duration = Duration::from_millis(1000);

    fn ms(t0: Instant, ms: u64) -> Instant {
        t0 + Duration::from_millis(ms)
    }

    #[test]
    fn clamps_to_project_limits_both_ways() {
        assert_eq!(clamp_velocity(0.5, 1.0), (MAX_LINEAR, MAX_ANGULAR));
        assert_eq!(clamp_velocity(-0.5, -1.0), (-MAX_LINEAR, -MAX_ANGULAR));
        assert_eq!(clamp_velocity(0.06, -0.3), (0.06, -0.3));
    }

    #[test]
    fn non_finite_command_becomes_zero() {
        assert_eq!(clamp_velocity(f64::NAN, f64::INFINITY), (0.0, 0.0));
        assert_eq!(clamp_velocity(f64::NEG_INFINITY, 0.2), (0.0, 0.2));
    }

    #[test]
    fn sends_the_latest_command_once_clamped() {
        let t0 = Instant::now();
        let mut d = DriveCommand::new(LEASE);
        d.set(0.05, 0.0, t0);
        d.set(0.5, -2.0, ms(t0, 10)); // replaces the first: latest wins
        assert_eq!(d.next(ms(t0, 20)), Some((MAX_LINEAR, -MAX_ANGULAR)));
        assert_eq!(d.next(ms(t0, 30)), None, "a command is sent once");
    }

    #[test]
    fn lapsed_lease_stops_the_wheels_once() {
        let t0 = Instant::now();
        let mut d = DriveCommand::new(LEASE);
        d.set(0.06, 0.1, t0);
        assert_eq!(d.next(ms(t0, 5)), Some((0.06, 0.1)));
        assert_eq!(d.next(ms(t0, 999)), None, "still within the lease");
        assert_eq!(d.next(ms(t0, 1000)), Some((0.0, 0.0)), "lease lapsed: stop");
        assert_eq!(d.next(ms(t0, 1500)), None, "zero is sent once");
        assert_eq!(d.next(ms(t0, 9000)), None);
    }

    #[test]
    fn fresh_commands_keep_the_lease() {
        let t0 = Instant::now();
        let mut d = DriveCommand::new(LEASE);
        for i in 0..30 {
            // 20 Hz, like the Nav2 controller
            let t = ms(t0, i * 50);
            d.set(0.06, 0.0, t);
            assert_eq!(d.next(t), Some((0.06, 0.0)));
        }
        assert_eq!(d.next(ms(t0, 29 * 50 + 999)), None);
        assert_eq!(d.next(ms(t0, 29 * 50 + 1000)), Some((0.0, 0.0)));
    }

    #[test]
    fn a_stop_needs_no_second_stop() {
        let t0 = Instant::now();
        let mut d = DriveCommand::new(LEASE);
        d.set(0.0, 0.0, t0);
        assert_eq!(d.next(t0), Some((0.0, 0.0)));
        assert_eq!(d.next(ms(t0, 5000)), None);
    }

    #[test]
    fn stale_pending_command_is_dropped() {
        // No Player traffic for longer than the lease: the command that
        // waited all that time must not start the wheels late.
        let t0 = Instant::now();
        let mut d = DriveCommand::new(LEASE);
        d.set(0.06, 0.0, t0);
        assert_eq!(d.next(ms(t0, 1500)), None);

        // While moving, a stale replacement turns into the stop.
        d.set(0.06, 0.0, ms(t0, 2000));
        assert_eq!(d.next(ms(t0, 2010)), Some((0.06, 0.0)));
        d.set(0.08, 0.0, ms(t0, 2500));
        assert_eq!(d.next(ms(t0, 3600)), Some((0.0, 0.0)));
        assert_eq!(d.next(ms(t0, 3700)), None);
    }

    #[test]
    fn nothing_to_send_before_any_command() {
        let mut d = DriveCommand::new(LEASE);
        assert_eq!(d.next(Instant::now()), None);
    }
}
