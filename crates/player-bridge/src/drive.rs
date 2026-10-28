//! Bounded latest-only velocity commands headed to Player.
//!
//! Player retains its last velocity. Callers must keep polling `next` so a
//! silent sender is stopped after the command lease expires.
use std::time::{Duration, Instant};

/// Linear speed ceiling, m/s (matching the configured chassis driver limit).
pub const MAX_LINEAR: f64 = 0.5;
/// Angular speed ceiling, rad/s.
pub const MAX_ANGULAR: f64 = 3.14;

pub fn clamp_velocity(vx: f64, vyaw: f64) -> (f64, f64) {
    fn limit(v: f64, max: f64) -> f64 {
        if v.is_finite() {
            v.clamp(-max, max)
        } else {
            0.0
        }
    }
    (limit(vx, MAX_LINEAR), limit(vyaw, MAX_ANGULAR))
}

pub struct DriveCommand {
    timeout: Duration,
    pending: Option<(f64, f64)>,
    last: Option<Instant>,
    moving: bool,
}

impl DriveCommand {
    pub fn new(timeout: Duration) -> Self {
        DriveCommand {
            timeout,
            pending: None,
            last: None,
            moving: false,
        }
    }

    pub fn set(&mut self, vx: f64, vyaw: f64, now: Instant) {
        self.pending = Some(clamp_velocity(vx, vyaw));
        self.last = Some(now);
    }

    pub fn next(&mut self, now: Instant) -> Option<(f64, f64)> {
        let fresh = self
            .last
            .is_some_and(|t| now.saturating_duration_since(t) < self.timeout);
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
    fn angular_ceiling_preserves_lower_speeds_and_lease() {
        assert_eq!(MAX_ANGULAR, 3.14);
        for w in [0., 0.4, -0.4, 1., -2., 3.14, -3.14] {
            assert_eq!(clamp_velocity(0.1, w), (0.1, w));
        }
        assert_eq!(clamp_velocity(9., 9.), (0.5, 3.14));
        assert_eq!(clamp_velocity(-9., -9.), (-0.5, -3.14));
        let t0 = Instant::now();
        let mut d = DriveCommand::new(LEASE);
        d.set(0., 9., t0);
        assert_eq!(d.next(t0), Some((0., 3.14)));
        assert_eq!(d.next(ms(t0, 999)), None);
        assert_eq!(d.next(ms(t0, 1000)), Some((0., 0.)));
        assert_eq!(d.next(ms(t0, 1001)), None);
    }

    #[test]
    fn accepts_driver_linear_ceiling_and_clamps_excess() {
        assert_eq!(MAX_LINEAR, 0.5);
        for v in [0.1, 0.3, 0.5, -0.3, -0.5] {
            assert_eq!(clamp_velocity(v, 0.), (v, 0.));
        }
        assert_eq!(clamp_velocity(9., 9.), (0.5, 3.14));
        assert_eq!(clamp_velocity(-9., -9.), (-0.5, -3.14));
        let t0 = Instant::now();
        let mut d = DriveCommand::new(LEASE);
        d.set(0.5, 0., t0);
        assert_eq!(d.next(t0), Some((0.5, 0.)));
        assert_eq!(d.next(ms(t0, 1000)), Some((0., 0.)));
    }

    #[test]
    fn clamps_to_project_limits_both_ways() {
        assert_eq!(clamp_velocity(0.5, 9.0), (MAX_LINEAR, MAX_ANGULAR));
        assert_eq!(clamp_velocity(-0.5, -9.0), (-MAX_LINEAR, -MAX_ANGULAR));
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
        d.set(0.5, -2.0, ms(t0, 10));
        assert_eq!(d.next(ms(t0, 20)), Some((MAX_LINEAR, -2.0)));
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
        let t0 = Instant::now();
        let mut d = DriveCommand::new(LEASE);
        d.set(0.06, 0.0, t0);
        assert_eq!(d.next(ms(t0, 1500)), None);
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
