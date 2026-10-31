//! The bumper interface (`bumper:0`): per-bumper pressed flags.
//!
//! Wire form (player_bumper_data_t, XDR): u32 count, then the count
//! again (xdr_bytes duplicate word), then the flags as consecutive bytes
//! padded to a 4-byte boundary. Measured live: three flags (the vendor
//! driver's "Left, Front, Right" segments), `1` while pressed.

use crate::Error;

/// Interface code (`bumper`).
pub const BUMPER_CODE: u16 = 14;
/// Data subtype: per-bumper state flags.
pub const BUMPER_DATA_STATE: u8 = 1;

/// A decoded bumper state frame.
#[derive(Debug, Clone, PartialEq)]
pub struct BumperState {
    pub pressed: Vec<bool>,
}

/// Decode a player_bumper_data_t payload (XDR).
pub fn decode_bumper(payload: &[u8]) -> Result<BumperState, Error> {
    let rd_u32 = |off: usize| -> Result<u32, Error> {
        payload
            .get(off..off + 4)
            .map(|b| u32::from_be_bytes(b.try_into().unwrap()))
            .ok_or(Error::ShortPayload {
                need: off + 4,
                got: payload.len(),
            })
    };
    let count = rd_u32(0)? as usize;
    // xdr_bytes duplicate count word
    let dup = rd_u32(4)? as usize;
    if dup != count {
        return Err(Error::CorruptChannel);
    }
    // Guard the allocation before the byte reserve below: a hostile count
    // must fail here, not inside the slice/pad arithmetic.
    if count > payload.len().saturating_sub(8) {
        return Err(Error::ShortPayload {
            need: 8 + count,
            got: payload.len(),
        });
    }
    // Flags are packed bytes (xdr_bytes), not one-per-slot: the live
    // released frame is 12 bytes for three flags, not 20.
    let padded = count.div_ceil(4) * 4;
    let bytes = payload.get(8..8 + padded).ok_or(Error::ShortPayload {
        need: 8 + padded,
        got: payload.len(),
    })?;
    let pressed = bytes[..count].iter().map(|&b| b != 0).collect();
    Ok(BumperState { pressed })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn frame(flags: &[u8]) -> Vec<u8> {
        // count, duplicate count, packed flags, zero padding to 4 bytes
        let mut p = vec![0u8; 8];
        p[0..4].copy_from_slice(&(flags.len() as u32).to_be_bytes());
        p[4..8].copy_from_slice(&(flags.len() as u32).to_be_bytes());
        p.extend_from_slice(flags);
        p.resize(p.len().div_ceil(4) * 4, 0);
        p
    }

    #[test]
    fn decodes_live_pressed_frame() {
        // Live 2026-10-10, bumper against the wall: three flags, first set.
        let p: Vec<u8> = [0, 0, 0, 3, 0, 0, 0, 3, 1, 0, 0, 0].into();
        assert_eq!(
            decode_bumper(&p).unwrap(),
            BumperState {
                pressed: vec![true, false, false]
            }
        );
    }

    #[test]
    fn decodes_live_released_frame() {
        // Same robot moments later, released (standby, still publishing).
        let p: Vec<u8> = [0, 0, 0, 3, 0, 0, 0, 3, 0, 0, 0, 0].into();
        assert_eq!(
            decode_bumper(&p).unwrap(),
            BumperState {
                pressed: vec![false, false, false]
            }
        );
    }

    #[test]
    fn rejects_mismatched_duplicate_count() {
        let p: Vec<u8> = [0, 0, 0, 2, 0, 0, 0, 3].into();
        assert!(matches!(decode_bumper(&p), Err(Error::CorruptChannel)));
    }

    #[test]
    fn rejects_truncated_flags() {
        let p: Vec<u8> = [0, 0, 0, 3, 0, 0, 0, 3, 0, 0].into();
        assert!(matches!(decode_bumper(&p), Err(Error::ShortPayload { .. })));
    }

    #[test]
    fn empty_frame_is_valid() {
        let p: Vec<u8> = [0, 0, 0, 0, 0, 0, 0, 0].into();
        assert_eq!(decode_bumper(&p).unwrap(), BumperState { pressed: vec![] });
    }

    #[test]
    fn padding_bytes_are_ignored() {
        // 5 flags need 8 bytes of padding; the last 3 bytes are padding.
        let p = frame(&[1, 0, 1, 0, 1]);
        assert_eq!(
            decode_bumper(&p).unwrap().pressed,
            vec![true, false, true, false, true]
        );
    }

    #[test]
    fn builds_flags_helper() {
        let p = frame(&[1, 0, 1]);
        assert_eq!(decode_bumper(&p).unwrap().pressed, vec![true, false, true]);
    }
}
