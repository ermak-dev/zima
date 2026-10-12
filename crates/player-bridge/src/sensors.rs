//! The auxiliary sensor interfaces of the vacuum's Player server.
//!
//! Wire forms (XDR, big-endian, 4-byte aligned):
//! - `ir` (code 22, subtype RANGES=1): each array is `u32 count, u32
//!   count again (the vendor driver duplicates the xdr length word,
//!   same as the bumper interface), then count floats` — first the
//!   voltages array, then the ranges array. On the V1: ir:0 is the
//!   wall sensor, ir:1 the cliff sensors (four pairs), ir:2 the
//!   wheel-drop sensors (two pairs). Measured live 2026-10-11: the
//!   duplicated word is present (a decoder that skips it reads the
//!   duplicate as a float `1.4e-45` — bit pattern of `1`).
//! - `sonar` (code 5, subtype RANGES=1): u32 ranges_count, then floats.
//!   The V1 ultrasonic sensor streams a single range.
//! - `power` (code 2, subtype STATE=1): u32 valid mask, four floats
//!   (volts, percent, joules, watts), i32 charging.

use crate::Error;

pub const POWER_CODE: u16 = 2;
pub const SONAR_CODE: u16 = 5;
pub const IR_CODE: u16 = 22;

pub const POWER_DATA_STATE: u8 = 1;
pub const SONAR_DATA_RANGES: u8 = 1;
pub const IR_DATA_RANGES: u8 = 1;

/// A decoded ir data frame: raw detector voltages [V] and ranges [m].
#[derive(Debug, Clone, Default, PartialEq)]
pub struct IrState {
    pub voltages: Vec<f32>,
    pub ranges: Vec<f32>,
}

/// A decoded sonar data frame: ranges [m].
#[derive(Debug, Clone, Default, PartialEq)]
pub struct SonarState {
    pub ranges: Vec<f32>,
}

/// A decoded power data frame.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct PowerState {
    pub valid: u32,
    pub volts: f32,
    pub percent: f32,
    pub joules: f32,
    pub watts: f32,
    pub charging: i32,
}

fn rd_u32(payload: &[u8], off: usize) -> Result<u32, Error> {
    payload
        .get(off..off + 4)
        .map(|b| u32::from_be_bytes(b.try_into().unwrap()))
        .ok_or(Error::ShortPayload {
            need: off + 4,
            got: payload.len(),
        })
}

fn rd_f32(payload: &[u8], off: usize) -> Result<f32, Error> {
    payload
        .get(off..off + 4)
        .map(|b| f32::from_be_bytes(b.try_into().unwrap()))
        .ok_or(Error::ShortPayload {
            need: off + 4,
            got: payload.len(),
        })
}

fn rd_i32(payload: &[u8], off: usize) -> Result<i32, Error> {
    payload
        .get(off..off + 4)
        .map(|b| i32::from_be_bytes(b.try_into().unwrap()))
        .ok_or(Error::ShortPayload {
            need: off + 4,
            got: payload.len(),
        })
}

/// Decode a player_ir_data_t payload (vendor form: each float array is
/// preceded by its length twice — xdr_bytes style, see the bumper).
pub fn decode_ir(payload: &[u8]) -> Result<IrState, Error> {
    let mut off = 0usize;
    let rd_count = |payload: &[u8], off: &mut usize| -> Result<usize, Error> {
        let count = rd_u32(payload, *off)? as usize;
        *off += 4;
        // The vendor driver duplicates the array length word.
        let dup = rd_u32(payload, *off)? as usize;
        *off += 4;
        if dup != count {
            return Err(Error::CorruptChannel);
        }
        Ok(count)
    };
    let vcount = rd_count(payload, &mut off)?;
    let mut voltages = Vec::with_capacity(vcount);
    for _ in 0..vcount {
        voltages.push(rd_f32(payload, off)?);
        off += 4;
    }
    let rcount = rd_count(payload, &mut off)?;
    let mut ranges = Vec::with_capacity(rcount);
    for _ in 0..rcount {
        ranges.push(rd_f32(payload, off)?);
        off += 4;
    }
    if payload.len() < off {
        return Err(Error::ShortPayload {
            need: off,
            got: payload.len(),
        });
    }
    Ok(IrState { voltages, ranges })
}

/// Decode a player_sonar_data_t payload (vendor form: the float array
/// carries its length word twice, xdr_bytes style like the ir arrays —
/// measured live 2026-10-11).
pub fn decode_sonar(payload: &[u8]) -> Result<SonarState, Error> {
    let count = rd_u32(payload, 0)? as usize;
    let dup = rd_u32(payload, 4)? as usize;
    if dup != count {
        return Err(Error::CorruptChannel);
    }
    let mut off = 8;
    let mut ranges = Vec::with_capacity(count);
    for _ in 0..count {
        ranges.push(rd_f32(payload, off)?);
        off += 4;
    }
    if payload.len() < off {
        return Err(Error::ShortPayload {
            need: off,
            got: payload.len(),
        });
    }
    Ok(SonarState { ranges })
}

/// Decode a player_power_data_t payload (fixed 24 bytes).
pub fn decode_power(payload: &[u8]) -> Result<PowerState, Error> {
    if payload.len() < 24 {
        return Err(Error::ShortPayload {
            need: 24,
            got: payload.len(),
        });
    }
    Ok(PowerState {
        valid: rd_u32(payload, 0)?,
        volts: rd_f32(payload, 4)?,
        percent: rd_f32(payload, 8)?,
        joules: rd_f32(payload, 12)?,
        watts: rd_f32(payload, 16)?,
        charging: rd_i32(payload, 20)?,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn f(v: f32) -> Vec<u8> {
        v.to_bits().to_be_bytes().to_vec()
    }

    fn u(v: u32) -> Vec<u8> {
        v.to_be_bytes().to_vec()
    }

    /// Vendor form of one float array: length word twice, then floats.
    fn arr(vals: &[f32]) -> Vec<u8> {
        let mut p = u(vals.len() as u32);
        p.extend(u(vals.len() as u32));
        for v in vals {
            p.extend(f(*v));
        }
        p
    }

    #[test]
    fn decodes_live_wall_ir_frame() {
        // Survey (2026-10-02, controller off): one range 0.055 m,
        // one voltage 0.000 V, streamed at ~22 Hz. Live 2026-10-11:
        // the arrays carry the duplicated length word (bumper style).
        let mut p = arr(&[0.0]);
        p.extend(arr(&[0.055]));
        assert_eq!(
            decode_ir(&p).unwrap(),
            IrState {
                voltages: vec![0.0],
                ranges: vec![0.055],
            }
        );
    }

    #[test]
    fn decodes_cliff_ir_frame() {
        // Survey: ir:1 carries four range/voltage pairs, all zeros.
        let mut p = arr(&[0.0; 4]);
        p.extend(arr(&[0.0; 4]));
        let st = decode_ir(&p).unwrap();
        assert_eq!(st.voltages.len(), 4);
        assert_eq!(st.ranges.len(), 4);
    }

    #[test]
    fn ir_rejects_mismatched_duplicate_count() {
        // First array's duplicate word disagrees: broken framing.
        let mut p = u(1);
        p.extend(u(2));
        p.extend(f(0.0));
        assert!(matches!(decode_ir(&p), Err(Error::CorruptChannel)));
    }

    #[test]
    fn ir_rejects_truncated_ranges() {
        // Two ranges claimed, only one present.
        let mut p = arr(&[]);
        p.extend(u(2));
        p.extend(u(2));
        p.extend(f(1.0));
        assert!(matches!(decode_ir(&p), Err(Error::ShortPayload { .. })));
    }

    #[test]
    fn decodes_live_sonar_frame() {
        // Survey: one range 65.534 m (sentinel; still a well-formed
        // frame). Live 2026-10-11: duplicated length word present.
        let mut p = u(1);
        p.extend(u(1));
        p.extend(f(65.534));
        assert_eq!(
            decode_sonar(&p).unwrap(),
            SonarState {
                ranges: vec![65.534]
            }
        );
    }

    #[test]
    fn sonar_rejects_mismatched_duplicate() {
        let mut p = u(1);
        p.extend(u(2));
        p.extend(f(65.534));
        assert!(matches!(decode_sonar(&p), Err(Error::CorruptChannel)));
    }

    #[test]
    fn sonar_rejects_truncated_range() {
        let p = u(2);
        assert!(matches!(decode_sonar(&p), Err(Error::ShortPayload { .. })));
    }

    #[test]
    fn decodes_live_power_frame() {
        // Survey: valid_mask=31, 19.939 V, 100%, charging=1.
        let mut p = u(31);
        p.extend(f(19.939));
        p.extend(f(100.0));
        p.extend(f(0.0));
        p.extend(f(0.0));
        p.extend(vec![0, 0, 0, 1]);
        let st = decode_power(&p).unwrap();
        assert_eq!(st.valid, 31);
        assert!((st.volts - 19.939).abs() < 1e-4);
        assert_eq!(st.percent, 100.0);
        assert_eq!(st.charging, 1);
    }

    #[test]
    fn power_rejects_short_payload() {
        let p = vec![0u8; 20];
        assert!(matches!(decode_power(&p), Err(Error::ShortPayload { .. })));
    }
}
