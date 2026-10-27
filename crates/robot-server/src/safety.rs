//! One lock protects ownership and the bounded latest-command slot. No I/O here.
use player_bridge::drive::{BackOff, DriveCommand, StallDetector};
use std::time::{Duration, Instant};
pub struct Safety {
    owner: Option<u64>,
    drive: DriveCommand,
    backoff: BackOff,
    stall: StallDetector,
    hold: Option<Instant>,
    pressed: bool,
    current: (f64, f64),
    slam_ok: bool,
}
impl Safety {
    pub fn new() -> Self {
        Self {
            owner: None,
            drive: DriveCommand::new(Duration::from_secs(1)),
            backoff: BackOff::new(),
            stall: StallDetector::new(),
            hold: None,
            pressed: false,
            current: (0., 0.),
            slam_ok: true,
        }
    }
    /// SLAM verdict for the drive gate. Without a SLAM subsystem the verdict
    /// stays true (open gate); a lost localization stops and holds driving.
    pub fn slam(&mut self, ok: bool, now: Instant) {
        if self.slam_ok == ok {
            return;
        }
        self.slam_ok = ok;
        if !ok {
            self.drive = DriveCommand::new(Duration::from_secs(1));
            self.drive.set(0., 0., now);
        }
    }
    pub fn command(&mut self, id: u64, v: f64, w: f64, now: Instant) {
        if self.owner.is_some_and(|owner| owner != id) {
            return;
        }
        self.owner = Some(id);
        if self.pressed
            || !self.slam_ok
            || self.backoff.is_running(now)
            || self.hold.is_some_and(|until| now < until)
        {
            return;
        }
        self.drive.set(v, w, now);
    }
    pub fn disconnect(&mut self, id: u64) {
        if self.owner != Some(id) {
            return;
        }
        self.owner = None;
        self.backoff = BackOff::new();
        self.drive = DriveCommand::new(Duration::from_secs(1));
        self.drive.set(0., 0., Instant::now());
        self.current = (0., 0.);
        self.stall = StallDetector::new();
    }
    pub fn bumper(&mut self, pressed: bool, now: Instant) {
        // Safety recovery is enabled only during intentional driving, not viewing.
        if !pressed || (self.owner.is_some() && self.current != (0., 0.)) {
            self.backoff.observe(pressed, now);
        }
        if pressed && !self.pressed && self.backoff.is_running(now) {
            self.drive = DriveCommand::new(Duration::from_secs(1));
        }
        self.pressed = pressed;
    }
    pub fn odom(&mut self, v: f64, w: f64, now: Instant) {
        if self.stall.update(self.current, (v, w), now) {
            eprintln!("robot-server: DRIVE STALLED; holding commands for 10 s");
            self.hold = Some(now + Duration::from_secs(10));
            self.backoff = BackOff::new();
            self.drive = DriveCommand::new(Duration::from_secs(1));
            self.drive.set(0., 0., now);
        }
    }
    pub fn next(&mut self, now: Instant) -> Option<(f64, f64)> {
        let next = self.backoff.active(now).or_else(|| self.drive.next(now));
        if let Some(v) = next {
            self.current = v;
        }
        next
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    // FEATURE: bounded latest-only commands and exclusive ownership.
    // SPEC: no startup/viewer zero; overwrite rather than queue; stale pending
    // commands never start late; only the controlling socket's disconnect stops.
    // IMPLEMENTATION: shared DriveCommand handles lease/clamp; Safety gates ownership.
    // RESULT: IMPLEMENTED — latest command, expiry and ownership transitions verified.
    // STATUS: DONE
    #[test]
    fn latest_only_lease_and_owner_transitions() {
        let now = Instant::now();
        let mut s = Safety::new();
        assert_eq!(s.next(now), None);
        s.disconnect(5);
        assert_eq!(s.next(now), None);
        for i in 0..10000 {
            s.command(1, i as f64 / 10000., -1., now);
        }
        s.command(2, -0.1, 0., now);
        assert_eq!(s.next(now), Some((0.1, -0.4)));
        assert_eq!(s.next(now), None);
        s.disconnect(2);
        assert_eq!(s.next(now), None);
        assert_eq!(s.next(now + Duration::from_secs(1)), Some((0., 0.)));
        assert_eq!(s.next(now + Duration::from_secs(2)), None);
        s.command(1, 0.08, 0., now + Duration::from_secs(2));
        assert_eq!(
            s.next(now + Duration::from_secs(4)),
            None,
            "stale pending may not start"
        );
        s.disconnect(1);
        assert_eq!(s.next(Instant::now()), Some((0., 0.)));
        s.command(2, -1., 1., now + Duration::from_secs(5));
        assert_eq!(s.next(now + Duration::from_secs(5)), Some((-0.1, 0.4)));
    }
    // FEATURE: SLAM gate on manual driving.
    // SPEC: when localization is not confirmed, commands are ignored and any
    // active drive is stopped; re-confirmation re-enables; without SLAM the
    // gate is open and behavior is unchanged.
    // IMPLEMENTATION: a slam_ok flag in Safety, cleared drive on losing it.
    // RESULT: IMPLEMENTED — block, stop, resume and default-open verified.
    // STATUS: DONE
    #[test]
    fn slam_loss_blocks_and_stops_manual_commands() {
        let now = Instant::now();
        let mut s = Safety::new();
        s.command(1, 0.1, 0., now);
        assert_eq!(s.next(now), Some((0.1, 0.0)));
        s.slam(false, now);
        assert_eq!(
            s.next(now + Duration::from_millis(10)),
            Some((0., 0.)),
            "losing localization must stop the active drive"
        );
        s.command(1, 0.1, 0., now + Duration::from_millis(20));
        assert_eq!(
            s.next(now + Duration::from_millis(20)),
            None,
            "commands must be ignored while localization is lost"
        );
        s.slam(true, now + Duration::from_millis(30));
        s.command(1, 0.1, 0., now + Duration::from_millis(40));
        assert_eq!(s.next(now + Duration::from_millis(40)), Some((0.1, 0.0)));
    }
}
