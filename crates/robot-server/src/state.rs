//! Typed server state (issue #50): every producer owns one disjoint section
//! and cannot overwrite another producer's data. The JSON object is a
//! transport representation built at the boundary (`State::to_json`) for the
//! panel WS frames, MCP observe and session recording; it is never edited
//! in place.
use crate::telemetry::Telemetry;
use serde_json::{json, Value};

/// Published cleaning-motor duties: requested values (clamped), not measured
/// rotation. Wheels stay in `safety::Safety`.
pub struct Motors {
    pub fan: f32,
    pub main: f32,
    pub side: f32,
}

pub struct State {
    /// Player sensor inventory; `Telemetry` stays its only producer.
    sensors: Telemetry,
    /// Display data from the SLAM thread; absent until the first verdict.
    slam: Option<Value>,
    /// Raw ST480 magnetometer stream. Single canonical copy: `sensors.compass`.
    compass: Value,
    motors: Motors,
    blocked: bool,
    /// Monotonic receive time of the last accepted Player section (#57):
    /// the honest freshness basis for observation snapshots.
    pub last_player: std::time::Instant,
    /// Receive time of the last SLAM verdict (None until the first one).
    pub last_slam: Option<std::time::Instant>,
}

impl State {
    pub fn initial() -> Self {
        Self {
            sensors: Telemetry::new(),
            slam: None,
            compass: Value::Null,
            motors: Motors {
                fan: 0.,
                main: 0.,
                side: 0.,
            },
            blocked: false,
            last_player: std::time::Instant::now(),
            last_slam: None,
        }
    }

    /// Publish the Player telemetry section and refresh the latched block
    /// verdict (safety stays authoritative). Returns false when the serialized
    /// section is unchanged, so idle frames do not wake watch listeners.
    pub fn publish_player(&mut self, sensors: Telemetry, blocked: bool) -> bool {
        let changed = sensors.json() != self.sensors.json() || blocked != self.blocked;
        if changed {
            self.last_player = std::time::Instant::now();
        }
        self.sensors = sensors;
        self.blocked = blocked;
        changed
    }

    /// Publish the SLAM section; returns false on an identical verdict.
    pub fn publish_slam(&mut self, slam: Value) -> bool {
        let changed = self.slam.as_ref() != Some(&slam);
        if changed {
            self.last_slam = Some(std::time::Instant::now());
        }
        self.slam = Some(slam);
        changed
    }

    /// Publish the compass sample unconditionally: status transitions must
    /// reach viewers even when the payload repeats.
    pub fn set_compass(&mut self, sample: Value) {
        self.compass = sample;
    }

    /// Publish requested (already clamped) cleaning-motor duties.
    pub fn set_motors(&mut self, fan: f32, main: f32, side: f32) {
        self.motors = Motors { fan, main, side };
    }

    /// Owner teardown or session end zeroes the published duties.
    pub fn stop_motors(&mut self) {
        self.motors = Motors {
            fan: 0.,
            main: 0.,
            side: 0.,
        };
    }

    /// Manual reset of the latched collision block (`reset_blocked`).
    pub fn clear_blocked(&mut self) {
        self.blocked = false;
    }

    /// Serialize the whole state into the panel wire frame. This is the only
    /// place the transport shape exists; missing data stays absent or null.
    pub fn to_json(&self) -> Value {
        let mut v = self.sensors.json();
        // The compass reading is embedded into the sensor inventory instead
        // of a second top-level copy (issue #50: one canonical location).
        v["sensors"]["compass"] = self.compass.clone();
        v["motors"] = json!({
            "fan": self.motors.fan,
            "main": self.motors.main,
            "side": self.motors.side,
        });
        v["blocked"] = json!(self.blocked);
        if let Some(slam) = &self.slam {
            v["slam"] = slam.clone();
        }
        v
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use player_bridge::*;

    fn fed_sensors() -> Telemetry {
        let mut t = Telemetry::new();
        t.update(&PlayerData::Sonar(SonarState { ranges: vec![1.5] }), 42.)
            .unwrap();
        t
    }

    fn populated() -> State {
        let mut s = State::initial();
        s.publish_slam(json!({"mode": "mapping", "map_version": 3}));
        s.set_compass(json!({"source": "st480-evdev", "status": "uncalibrated", "stamp": 42.5}));
        s.set_motors(60., 40., 80.);
        s
    }

    // FEATURE (#50): typed sections; a producer updates only its own section.
    // SPEC: a Player telemetry publish must not erase the SLAM verdict, the
    // compass sample or the motor duties written by their own producers.
    #[test]
    fn player_publish_keeps_slam_compass_and_motors_sections() {
        let mut s = populated();
        assert!(s.publish_player(fed_sensors(), true));
        let v = s.to_json();
        assert_eq!(v["slam"]["map_version"], 3);
        assert_eq!(v["sensors"]["compass"]["stamp"], 42.5);
        assert_eq!(
            v["motors"],
            json!({"fan": 60.0, "main": 40.0, "side": 80.0})
        );
        assert_eq!(v["sonar"], 1.5);
        assert_eq!(v["blocked"], true);
    }

    // SPEC: SLAM and compass updates must not erase Player sensors, motors
    // or the block verdict.
    #[test]
    fn slam_and_compass_updates_keep_player_and_motor_sections() {
        let mut s = State::initial();
        assert!(s.publish_player(fed_sensors(), true));
        assert!(s.publish_slam(json!({"mode": "lost"})));
        s.set_compass(json!({"status": "stale", "yaw": null}));
        s.set_motors(0., 100., 0.);
        let v = s.to_json();
        assert_eq!(v["sonar"], 1.5);
        assert_eq!(v["blocked"], true);
        assert_eq!(v["motors"], json!({"fan": 0.0, "main": 100.0, "side": 0.0}));
        assert_eq!(v["slam"]["mode"], "lost");
        // stop_motors zeroes duties but not other sections.
        s.stop_motors();
        let v = s.to_json();
        assert_eq!(v["motors"], json!({"fan": 0.0, "main": 0.0, "side": 0.0}));
        assert_eq!(v["sonar"], 1.5);
        assert_eq!(v["sensors"]["compass"]["status"], "stale");
    }

    // SPEC: reset_blocked clears only the verdict; missing data stays null.
    #[test]
    fn blocked_reset_touches_only_the_verdict() {
        let mut s = populated();
        s.publish_player(fed_sensors(), true);
        s.clear_blocked();
        let v = s.to_json();
        assert_eq!(v["blocked"], false);
        assert_eq!(v["slam"]["map_version"], 3);
        assert_eq!(v["sensors"]["compass"]["stamp"], 42.5);
        assert_eq!(v["motors"]["fan"], 60.0);
    }

    // FEATURE (#50): the compass reading has exactly one independently
    // changeable copy, under sensors.compass.
    #[test]
    fn compass_has_a_single_copy_at_sensors_compass() {
        let mut s = State::initial();
        let mut v = s.to_json();
        assert!(v["sensors"]["compass"].is_null());
        assert!(v.get("compass").is_none());
        s.set_compass(json!({"status": "uncalibrated"}));
        v = s.to_json();
        assert_eq!(v["sensors"]["compass"]["status"], "uncalibrated");
        assert!(v.get("compass").is_none());
    }

    // SPEC: unchanged player frames and identical SLAM verdicts must not
    // notify watch listeners (the pre-#50 merge contract).
    #[test]
    fn unchanged_sections_do_not_notify_watchers() {
        let mut s = State::initial();
        assert!(s.publish_player(fed_sensors(), false));
        assert!(!s.publish_player(fed_sensors(), false), "identical frame");
        assert!(!s.publish_player(fed_sensors().clone(), false));
        assert!(s.publish_slam(json!({"mode": "mapping"})));
        assert!(
            !s.publish_slam(json!({"mode": "mapping"})),
            "identical verdict"
        );
    }

    // FEATURE (#50): the initial state serializes into the exact panel frame.
    // SPEC: blocked=false, zero motors, a sensors inventory of nulls, no slam
    // section and none of the removed legacy keys.
    #[test]
    fn initial_state_serializes_the_panel_contract() {
        let v = State::initial().to_json();
        assert_eq!(v["blocked"], false);
        assert_eq!(v["motors"], json!({"fan": 0.0, "main": 0.0, "side": 0.0}));
        assert!(v["sensors"].is_object());
        assert!(v["sensors"]["battery"].is_null());
        assert!(v["sensors"]["compass"].is_null());
        assert!(v.get("slam").is_none());
        for legacy in ["map_pose", "map_age_s", "mcp_owner"] {
            assert!(v.get(legacy).is_none(), "{legacy} must not exist");
        }
    }
}
