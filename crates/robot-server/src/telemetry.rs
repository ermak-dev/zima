use crate::Failure;
use player_bridge::*;
use serde_json::{json, Value};
use std::collections::BTreeMap;

pub struct Telemetry {
    fields: BTreeMap<String, Value>,
}
fn rounded(v: f64, places: i32) -> Value {
    let scale = 10f64.powi(places);
    if v.is_finite() {
        json!((v * scale).round() / scale)
    } else {
        Value::Null
    }
}
fn values(v: &[f32]) -> Value {
    Value::Array(v.iter().map(|v| rounded(*v as f64, 6)).collect())
}
impl Telemetry {
    pub fn new() -> Self {
        Self {
            fields: BTreeMap::new(),
        }
    }
    pub fn json(&self) -> Value {
        let mut out = json!(self.fields);
        let mut sensors = json!({});
        for key in [
            "battery",
            "bumper",
            "bumper_segments",
            "wall_ir",
            "cliff_ir",
            "drop_ir",
            "sonar",
            "wall_voltages",
            "cliff_ranges",
            "drop_ranges",
        ] {
            sensors[key] = self.fields.get(key).cloned().unwrap_or(Value::Null);
        }
        for raw in ["wall_voltages", "cliff_ranges", "drop_ranges"] {
            out.as_object_mut().unwrap().remove(raw);
        }
        out["sensors"] = sensors;
        out
    }
    pub fn update(&mut self, msg: &Message) -> Result<bool, Failure> {
        let p = &msg.payload;
        let a = msg.header.addr;
        if msg.header.msg_type != MSGTYPE_DATA || msg.header.subtype != 1 {
            return Ok(false);
        }
        match (a.interf, a.index) {
            (POSITION2D_CODE, 0) => {
                let s = decode_position2d(p)?;
                self.fields.insert("odom".into(),json!({"x":rounded(s.x,3),"y":rounded(s.y,3),"yaw":rounded(s.yaw,3),"vx":rounded(s.vx,2),"yaw_rate":rounded(s.vyaw,2)}));
            }
            (BUMPER_CODE, 0) => {
                let s = decode_bumper(p)?;
                // Aggregate keeps the safety/panel contract; the per-segment
                // flags (vendor: Left/Front/Right) ride along as inventory.
                self.fields.insert(
                    "bumper".into(),
                    json!(if s.pressed.iter().any(|v| *v) {
                        "PRESSED"
                    } else {
                        "clear"
                    }),
                );
                self.fields.insert(
                    "bumper_segments".into(),
                    if s.pressed.is_empty() {
                        Value::Null
                    } else {
                        Value::Array(s.pressed.iter().map(|v| json!(v)).collect())
                    },
                );
            }
            (IR_CODE, 0..=2) => {
                let s = decode_ir(p)?;
                if a.index == 0 {
                    self.fields.insert(
                        "wall_ir".into(),
                        s.ranges
                            .first()
                            .map(|v| rounded(*v as f64, 6))
                            .unwrap_or(Value::Null),
                    );
                    self.fields
                        .insert("wall_voltages".into(), values(&s.voltages));
                } else {
                    let name = if a.index == 1 { "cliff" } else { "drop" };
                    self.fields
                        .insert(format!("{name}_ir"), values(&s.voltages));
                    self.fields
                        .insert(format!("{name}_ranges"), values(&s.ranges));
                }
            }
            (SONAR_CODE, 0) => {
                let s = decode_sonar(p)?;
                self.fields.insert(
                    "sonar".into(),
                    s.ranges
                        .first()
                        .map(|v| rounded(*v as f64, 6))
                        .unwrap_or(Value::Null),
                );
            }
            (POWER_CODE, 0) => {
                let s = decode_power(p)?;
                // Valid bits are authoritative for each frame: never retain an earlier
                // reading where the new frame says unknown. No current is synthesized.
                self.fields.insert("battery".into(),json!({"percent":if s.valid&8!=0 && s.percent.is_finite() && (0.0..=100.0).contains(&s.percent){json!((s.percent as f64).round() as i64)}else{Value::Null},"volts":if s.valid&1!=0 && s.volts>0.0{rounded(s.volts as f64,1)}else{Value::Null},"charging":if s.valid&16!=0 && matches!(s.charging,0|1){json!(s.charging==1)}else{Value::Null}}));
            }
            _ => return Ok(false),
        }
        Ok(true)
    }
}
pub fn scan(msg: &Message) -> Result<Option<Value>, Failure> {
    let s = decode_laser_scanangle(&msg.payload)?;
    if s.ranges.len() != s.angles.len() {
        return Err("scan bearing/range count mismatch".into());
    }
    if !s.max_range.is_finite() || s.max_range < player_bridge::scan::LDS_RANGE_MIN {
        return Ok(None);
    }
    let points: Vec<[f64; 2]> = s
        .ranges
        .iter()
        .zip(&s.angles)
        .filter_map(|(&r, &a)| {
            if !r.is_finite()
                || !a.is_finite()
                || r < player_bridge::scan::LDS_RANGE_MIN
                || r > s.max_range
            {
                return None;
            }
            let angle = a as f64 + player_bridge::scan::LDS_YAW_DEG.to_radians();
            Some([
                player_bridge::scan::LDS_X + r as f64 * angle.cos(),
                r as f64 * angle.sin(),
            ])
        })
        .collect();
    let mut out = json!({"points":points,"range_max":s.max_range,"frame":"base_link"});
    if msg.header.timestamp.is_finite() && msg.header.timestamp > 0. {
        out["stamp"] = json!(msg.header.timestamp);
    }
    Ok(Some(out))
}
#[cfg(test)]
mod tests {
    use super::*;
    // FEATURE: preserve unknown battery fields and the panel's exact schema.
    // SPEC: invalid numbers/charging enums remain null; raw arrays are inventory-only.
    // IMPLEMENTATION: validate each power field independently, serialize raw arrays only in sensors.
    // RESULT: IMPLEMENTED — invalid battery values stay unknown and exact inventory schema passes.
    // STATUS: DONE
    fn data(interf: u16, payload: Vec<u8>) -> Message {
        Message {
            header: MsgHeader {
                addr: DevAddr {
                    host: 0,
                    robot: 0,
                    interf,
                    index: 0,
                },
                msg_type: 1,
                subtype: 1,
                timestamp: 0.,
                seq: 0,
                size: payload.len() as u32,
            },
            payload,
        }
    }
    #[test]
    fn invalid_power_values_are_unknown() {
        for (volts, percent, charging) in [
            (0f32, 101f32, -1i32),
            (-1., -1., 2),
            (f32::NAN, f32::INFINITY, -99),
        ] {
            let mut p = 31u32.to_be_bytes().to_vec();
            for f in [volts, percent, 0., 0.] {
                p.extend(f.to_be_bytes());
            }
            p.extend(charging.to_be_bytes());
            let mut t = Telemetry::new();
            t.update(&data(POWER_CODE, p)).unwrap();
            assert_eq!(
                t.json()["battery"],
                json!({"percent":null,"volts":null,"charging":null})
            );
        }
    }
    #[test]
    fn raw_arrays_only_appear_in_sensor_inventory() {
        let mut t = Telemetry::new();
        let mut p = Vec::new();
        for v in [2f32, 0.05] {
            p.extend(1u32.to_be_bytes());
            p.extend(1u32.to_be_bytes());
            p.extend(v.to_be_bytes());
        }
        t.update(&data(IR_CODE, p)).unwrap();
        let v = t.json();
        assert!(
            v.get("wall_voltages").is_none(),
            "schema must match state_to_json"
        );
        assert_eq!(v["sensors"]["wall_voltages"], json!([2.]));
    }
    #[test]
    fn empty_scan_and_invalid_source_time_do_not_invent_beams_or_stamp() {
        for stamp in [0., -1., f64::NAN, f64::INFINITY] {
            let mut m = data(LASER_CODE, vec![]);
            m.header.subtype = 3;
            m.header.timestamp = stamp;
            m.payload.extend(6f32.to_be_bytes());
            m.payload.extend([0; 28]);
            let v = scan(&m).unwrap().unwrap();
            assert_eq!(v["points"], json!([]));
            assert!(v.get("stamp").is_none());
        }
    }
}
