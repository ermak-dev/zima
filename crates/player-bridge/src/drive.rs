//! Drive safety for velocity commands headed to Player.
//!
//! Player keeps executing the last velocity it was given until another
//! one replaces it; nothing on the robot expires it. So the bridge, not
//! its callers, enforces two rules before a command reaches the wheels:
//!
//! - limits: the project ceiling (0.5 m/s, 0.4 rad/s) whatever the
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

/// Linear speed ceiling, m/s (matching the configured chassis driver limit).
pub const MAX_LINEAR: f64 = 0.5;
/// Angular speed ceiling, rad/s.
pub const MAX_ANGULAR: f64 = 0.4;

/// Clamp a command to the project limits; non-finite values become zero.
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

/// Watch for a blocked drive: velocity commands flow, the wheels do not.
///
/// After the first live SLAM drive the vacuum sat pressed against a wall
/// with Nav2 commanding 0.06 m/s for a minute: Player accepts the command,
/// odometry reports zero, and nothing signals the deadlock (the MCU had
/// latched an error that only a mask cycle clears). The detector compares
/// commanded speed with the odometry twist the bridge itself relays:
/// sustained nonzero commands with a stalled twist mean the drive is
/// blocked — publishing more commands just pushes the wall.
pub struct StallDetector {
    /// Command magnitude that counts as "trying to move".
    cmd_threshold: f64,
    /// Twist magnitude below which the robot counts as stationary.
    odom_threshold: f64,
    /// How long the mismatch must persist.
    window: Duration,
    commanded_since: Option<Instant>,
    latched: bool,
}

impl StallDetector {
    /// Defaults tuned for this project: commands at 0.03 m/s or 0.15 rad/s
    /// and up, twist under a quarter of that counts as stationary, and the
    /// mismatch must hold for 3 s (wheel spin-up takes under a second).
    pub fn new() -> Self {
        StallDetector {
            cmd_threshold: 0.03,
            odom_threshold: 0.01,
            window: Duration::from_secs(3),
            commanded_since: None,
            latched: false,
        }
    }

    /// Feed the current commanded velocity (pre-clamp is fine) and the
    /// latest odometry twist; returns true once per stall episode.
    pub fn update(&mut self, cmd: (f64, f64), odom: (f64, f64), now: Instant) -> bool {
        if self.latched {
            // Reset when commands stop (the caller reacts by zeroing).
            if cmd.0.abs() < self.cmd_threshold && cmd.1.abs() < self.cmd_threshold {
                self.latched = false;
                self.commanded_since = None;
            }
            return false;
        }
        let commanding = cmd.0.abs() >= self.cmd_threshold || cmd.1.abs() >= self.cmd_threshold;
        let stationary = odom.0.abs() < self.odom_threshold && odom.1.abs() < self.odom_threshold;
        if commanding && stationary {
            if let Some(t0) = self.commanded_since {
                if now.saturating_duration_since(t0) >= self.window {
                    self.latched = true;
                    return true;
                }
            } else {
                self.commanded_since = Some(now);
            }
        } else {
            self.commanded_since = None;
        }
        false
    }
}

impl Default for StallDetector {
    fn default() -> Self {
        Self::new()
    }
}

/// The bumper-triggered back-off manoeuvre.
///
/// Mirrors the stock `StateHandleBumperError -> MoveBack` path: on bumper
/// contact, drive straight back for a short distance **immediately** —
/// before the MCU latches a wheel error (measured live: once latched, only
/// a mask cycle recovers the drive). The bridge owns this manoeuvre; Nav2
/// commands are ignored while it runs.
pub struct BackOff {
    /// Back-off speed, m/s (negative = reverse).
    speed: f64,
    /// Manoeuvre duration.
    duration: Duration,
    until: Option<Instant>,
    /// Bumper state as of the last frame (edge detection).
    was_pressed: bool,
    /// The reverse command was sent and the manoeuvre has ended: Player
    /// keeps executing the last velocity until replaced, so exactly one
    /// stop must follow (observed live: the robot kept reversing after
    /// the manoeuvre because nothing replaced the -0.1 m/s).
    stop_pending: bool,
}

impl BackOff {
    /// Stock-tuned: ~0.1 m back at a gentle reverse speed.
    pub fn new() -> Self {
        BackOff {
            speed: -0.1,
            duration: Duration::from_millis(1200),
            until: None,
            was_pressed: false,
            stop_pending: false,
        }
    }

    /// Feed the bumper state. A back-off starts on the released ->
    /// pressed transition ONLY: the vendor driver streams the pressed
    /// frame continuously while contact lasts, and re-triggering on every
    /// frame made the manoeuvre endless (observed live: the robot kept
    /// reversing until the session was killed by hand).
    pub fn observe(&mut self, pressed: bool, now: Instant) {
        if pressed && !self.was_pressed {
            self.until = Some(now + self.duration);
            self.stop_pending = false;
        }
        self.was_pressed = pressed;
    }

    /// Velocity to enforce right now: the reverse while the manoeuvre
    /// runs, then exactly one stop, then None.
    pub fn active(&mut self, now: Instant) -> Option<(f64, f64)> {
        if let Some(u) = self.until {
            if now < u {
                return Some((self.speed, 0.0));
            }
            // Manoeuvre just ended: replace the reverse with a stop once.
            if !self.stop_pending {
                self.stop_pending = true;
                return Some((0.0, 0.0));
            }
        }
        None
    }

    pub fn is_running(&self, now: Instant) -> bool {
        self.until.is_some_and(|u| now < u)
    }
}

impl Default for BackOff {
    fn default() -> Self {
        Self::new()
    }
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
        DriveCommand {
            timeout,
            pending: None,
            last: None,
            moving: false,
        }
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

    // FEATURE: driver-ceiling linear commands without raising angular or recovery speeds.
    // SPEC: accept up to 0.5 m/s both ways; clamp excess and retain the one-second lease.
    // IMPLEMENTATION: raise MAX_LINEAR only and keep the unused BackOff helper at 0.1 m/s.
    // RESULT: IMPLEMENTED — ceiling, angular bound, lease and recovery speed verified.
    // STATUS: DONE
    #[test]
    fn accepts_driver_linear_ceiling_and_clamps_excess() {
        assert_eq!(MAX_LINEAR, 0.5);
        for v in [0.1, 0.3, 0.5, -0.3, -0.5] {
            assert_eq!(clamp_velocity(v, 0.), (v, 0.));
        }
        assert_eq!(clamp_velocity(9., 9.), (0.5, 0.4));
        assert_eq!(clamp_velocity(-9., -9.), (-0.5, -0.4));
        let t0 = Instant::now();
        let mut d = DriveCommand::new(LEASE);
        d.set(0.5, 0., t0);
        assert_eq!(d.next(t0), Some((0.5, 0.)));
        assert_eq!(d.next(ms(t0, 1000)), Some((0., 0.)));
        let mut b = BackOff::new();
        b.observe(true, t0);
        assert_eq!(b.active(t0), Some((-0.1, 0.)));
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
    fn stall_fires_after_sustained_push_and_resets() {
        let t0 = Instant::now();
        let mut d = StallDetector::new();
        // Commands flowing, robot stationary: silence until the window
        // elapses, then exactly one alarm.
        for at in [0, 500, 1000, 2000, 2900] {
            assert!(
                !d.update((0.06, 0.0), (0.0, 0.0), ms(t0, at)),
                "fired at {at} ms"
            );
        }
        assert!(d.update((0.06, 0.0), (0.0, 0.0), ms(t0, 3000)));
        assert!(
            !d.update((0.06, 0.0), (0.0, 0.0), ms(t0, 4000)),
            "alarm repeats"
        );
        // Commands stop: the detector rearms (commands flow at 20 Hz again
        // and the fresh window must elapse before the next alarm).
        assert!(!d.update((0.0, 0.0), (0.0, 0.0), ms(t0, 5000)));
        for at in [5000, 6000, 7000, 7900] {
            assert!(
                !d.update((0.05, 0.0), (0.0, 0.0), ms(t0, at)),
                "fired at {at}"
            );
        }
        assert!(d.update((0.05, 0.0), (0.0, 0.0), ms(t0, 8000)), "rearmed");
    }

    #[test]
    fn moving_robot_or_idle_commands_do_not_stall() {
        let t0 = Instant::now();
        let mut d = StallDetector::new();
        for at in (0..6000).step_by(500) {
            // Robot actually moves.
            assert!(!d.update((0.06, 0.0), (0.05, 0.0), ms(t0, at)));
            // No commands.
            assert!(!d.update((0.0, 0.0), (0.0, 0.0), ms(t0, at)));
        }
        // Rotation commanded and happening.
        assert!(!d.update((0.0, 0.3), (0.0, 0.28), ms(t0, 7000)));
        // Brief mismatch (a start transient) does not fire.
        let mut d2 = StallDetector::new();
        assert!(!d2.update((0.06, 0.0), (0.0, 0.0), ms(t0, 0)));
        assert!(
            !d2.update((0.06, 0.0), (0.06, 0.0), ms(t0, 1500)),
            "wheels spun up"
        );
        assert!(
            !d2.update((0.06, 0.0), (0.0, 0.0), ms(t0, 2500)),
            "window restarted"
        );
    }

    #[test]
    fn backoff_runs_once_per_press_regardless_of_frame_stream() {
        let t0 = Instant::now();
        let mut b = BackOff::new();
        assert!(!b.is_running(t0));
        // The vendor driver streams the pressed frame continuously (5+ Hz).
        // First frame starts the manoeuvre...
        b.observe(true, t0);
        assert_eq!(b.active(t0), Some((-0.1, 0.0)));
        // ...the stream must NOT extend it (observed live: endless reverse).
        for at in (50..1200).step_by(50) {
            b.observe(true, ms(t0, at));
        }
        assert_eq!(b.active(ms(t0, 1199)), Some((-0.1, 0.0)));
        assert_eq!(
            b.active(ms(t0, 1200)),
            Some((0.0, 0.0)),
            "one stop after the manoeuvre"
        );
        assert_eq!(b.active(ms(t0, 1300)), None, "then quiet");
        assert!(
            !b.is_running(ms(t0, 1300)),
            "still pressed, still no reverse"
        );
        // Release, press again: a fresh manoeuvre.
        b.observe(false, ms(t0, 1500));
        b.observe(true, ms(t0, 1600));
        assert!(b.is_running(ms(t0, 1700)));
        assert!(!b.is_running(ms(t0, 2801)));
        // A hold-press across the manoeuvre end stays stopped.
        b.observe(true, ms(t0, 3000));
        assert!(!b.is_running(ms(t0, 3100)), "no second window while held");
    }

    #[test]
    fn backoff_sends_exactly_one_stop_after_the_reverse() {
        let t0 = Instant::now();
        let mut b = BackOff::new();
        b.observe(true, t0);
        assert_eq!(b.active(t0), Some((-0.1, 0.0)));
        assert_eq!(b.active(ms(t0, 1199)), Some((-0.1, 0.0)));
        // The manoeuvre ends: one stop replaces the reverse (Player keeps
        // the last velocity until replaced — the live bug).
        assert_eq!(
            b.active(ms(t0, 1200)),
            Some((0.0, 0.0)),
            "stop after reverse"
        );
        assert_eq!(b.active(ms(t0, 1250)), None, "stop is sent once");
        assert_eq!(b.active(ms(t0, 5000)), None);
        // is_running is false already at the stop stage.
        assert!(!b.is_running(ms(t0, 1200)));
    }

    #[test]
    fn released_bumper_never_triggers() {
        let t0 = Instant::now();
        let mut b = BackOff::new();
        for at in (0..3000).step_by(100) {
            b.observe(false, ms(t0, at));
        }
        assert!(!b.is_running(ms(t0, 2900)));
    }

    #[test]
    fn nothing_to_send_before_any_command() {
        let mut d = DriveCommand::new(LEASE);
        assert_eq!(d.next(Instant::now()), None);
    }
}
