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
            "dock",
            "bumper",
            "bumper_segments",
            "compass",
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
        let mut service = json!({});
        for key in [
            "ir:3",
            "ir:4",
            "ir:6",
            "ir:7",
            "ir:8",
            "position3d:0",
            "position3d:1",
            "position3d:2",
            "position3d:4",
        ] {
            service[key] = self.fields.get(key).cloned().unwrap_or(Value::Null);
            out.as_object_mut().unwrap().remove(key);
        }
        sensors["service_channels"] = service;
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
            (IR_CODE, 3 | 4 | 6 | 7 | 8) => {
                let s = decode_ir(p)?;
                self.fields.insert(format!("ir:{}", a.index), json!({
                    "voltages": values(&s.voltages),
                    "ranges": values(&s.ranges),
                    "stamp": if msg.header.timestamp > 0. { rounded(msg.header.timestamp, 6) } else { Value::Null },
                }));
            }
            (POSITION3D_CODE, 0 | 1 | 2 | 4) => {
                let s = decode_position3d(p)?;
                self.fields.insert(format!("position3d:{}", a.index), json!({
                    "pos": s.pos.iter().map(|v| rounded(*v, 6)).collect::<Vec<_>>(),
                    "vel": s.vel.iter().map(|v| rounded(*v, 6)).collect::<Vec<_>>(),
                    "stall": s.stall,
                    "stamp": if msg.header.timestamp > 0. { rounded(msg.header.timestamp, 6) } else { Value::Null },
                }));
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
                // The vendor merges dock presence into the power frame's charging
                // field; the dock row restates that fact instead of inventing a
                // separate sensor that does not exist on the wire.
                self.fields.insert(
                    "dock".into(),
                    if s.valid & 16 != 0 && matches!(s.charging, 0 | 1) {
                        json!(s.charging == 1)
                    } else {
                        Value::Null
                    },
                );
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
    let mut out = json!({"id":s.id,"points":points,"range_max":s.max_range,"frame":"base_link"});
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
    // BUG: vendor SCANANGLE bearings already include the configured mounting yaw.
    // SPEC: panel and SLAM preserve body-aligned bearings and apply only LDS translation.
    // RESULT: FIXED — native bearings reach both consumers without duplicate yaw.
    #[test]
    fn native_scan_bearings_are_not_rotated_a_second_time() {
        let mut payload = 6f32.to_be_bytes().to_vec();
        for array in [
            vec![1f32, 2.],
            vec![0., std::f32::consts::FRAC_PI_2],
            vec![],
        ] {
            payload.extend((array.len() as u32).to_be_bytes());
            payload.extend((array.len() as u32).to_be_bytes());
            for value in array {
                payload.extend(value.to_be_bytes());
            }
        }
        payload.extend(41u32.to_be_bytes());
        let mut message = data(LASER_CODE, payload);
        message.header.subtype = LASER_DATA_SCANANGLE;
        let rendered = scan(&message).unwrap().unwrap();
        let points = rendered["points"].as_array().unwrap();
        assert!((points[0][0].as_f64().unwrap() - 0.9).abs() < 1e-6);
        assert!(points[0][1].as_f64().unwrap().abs() < 1e-6);
        assert!((points[1][0].as_f64().unwrap() + 0.1).abs() < 1e-6);
        assert!((points[1][1].as_f64().unwrap() - 2.).abs() < 1e-6);
        let decoded = decode_laser_scanangle(&message.payload).unwrap();
        let observation = crate::slam::Observation::from_scan(&decoded, None);
        assert!(observation.angles[0].abs() < 1e-6);
        assert!((observation.angles[1] - std::f32::consts::FRAC_PI_2).abs() < 1e-6);
    }

    // FEATURE: service-channel inventory without inferred units or motor commands.
    // SPEC: preserve raw arrays, null nonfinite values, and expose missing readings.
    // IMPLEMENTATION: optional IR/position3d channels live in sensors.service_channels.
    // RESULT: IMPLEMENTED — arrays, empty frames and unknown values verified offline.
    // STATUS: DONE
    #[test]
    fn service_ir_channels_preserve_raw_arrays_and_clear_old_values() {
        let mut t = Telemetry::new();
        for index in [3, 4, 6, 7, 8] {
            let key = format!("ir:{index}");
            assert_eq!(t.json()["sensors"]["service_channels"][&key], Value::Null);
            let mut p = Vec::new();
            for array in [vec![0f32, 2., f32::NAN], vec![17f32]] {
                p.extend((array.len() as u32).to_be_bytes());
                p.extend((array.len() as u32).to_be_bytes());
                for v in array {
                    p.extend(v.to_be_bytes());
                }
            }
            let mut m = data(IR_CODE, p);
            m.header.addr.index = index;
            m.header.timestamp = 42.;
            assert!(t.update(&m).unwrap());
            assert_eq!(
                t.json()["sensors"]["service_channels"][&key],
                json!({"voltages":[0.,2.,null],"ranges":[17.],"stamp":42.})
            );
            m.payload = vec![0; 16];
            t.update(&m).unwrap();
            assert_eq!(
                t.json()["sensors"]["service_channels"][&key]["ranges"],
                json!([])
            );
        }
        let mut m = data(IR_CODE, vec![0; 16]);
        m.header.addr.index = 5;
        assert!(!t.update(&m).unwrap());
    }

    #[test]
    fn service_position3d_channels_preserve_all_axes() {
        let mut t = Telemetry::new();
        for index in [0, 1, 4] {
            let mut p = Vec::new();
            for v in [1f64, 2., 3., 0.1, 0.2, 0.3, 4., 5., 6., 0.4, 0.5, f64::NAN] {
                p.extend(v.to_be_bytes());
            }
            p.extend(1u32.to_be_bytes());
            let mut m = data(POSITION3D_CODE, p);
            m.header.addr.index = index;
            assert!(t.update(&m).unwrap());
            assert_eq!(
                t.json()["sensors"]["service_channels"][format!("position3d:{index}")],
                json!({"pos":[1.,2.,3.,0.1,0.2,0.3],"vel":[4.,5.,6.,0.4,0.5,null],"stall":true,"stamp":null})
            );
        }
    }

    // BUG: vendor position3d:2 never computes yaw, even when XYZ is nonzero.
    // HYPOTHESIS: retain the entire vendor packet only as service inventory, not a heading.
    // SOLUTION: position3d:2 is raw inventory; only ST480 evdev supplies compass samples.
    // RESULT: FIXED — zero, nonzero and invalid vendor yaw never become a heading.
    // STATUS: FIXED
    #[test]
    fn vendor_compass_never_supplies_a_heading() {
        let mut t = Telemetry::new();
        let mut m = data(POSITION3D_CODE, vec![0; 100]);
        m.header.addr.index = 2;
        for (x, yaw) in [(0., 0.), (600., 0.), (600., 1.25), (600., f64::NAN)] {
            m.payload[..8].copy_from_slice(&f64::to_be_bytes(x));
            m.payload[40..48].copy_from_slice(&yaw.to_be_bytes());
            m.header.timestamp = 50.;
            t.update(&m).unwrap();
            let state = t.json();
            assert!(state["sensors"]["compass"].is_null());
            assert!(state.get("compass").is_none());
            let raw = &state["sensors"]["service_channels"]["position3d:2"];
            assert_eq!(raw["pos"][0], x);
            assert_eq!(
                raw["pos"][5],
                if yaw.is_finite() {
                    json!(yaw)
                } else {
                    Value::Null
                }
            );
            assert_eq!(raw["stamp"], 50.);
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
            assert_eq!(v["id"], 0, "decoded scan identity must be preserved");
            assert_eq!(v["points"], json!([]));
            assert!(v.get("stamp").is_none());
        }
    }
}
