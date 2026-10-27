//! Library crate: packet codec and the owner/guard lifecycle.
//!
//! Wire format (see HARDWARE.md): 76-byte packets,
//! `+0 u16 event` (0x45 request, 0xa9 ack), `+2 src`, `+3 dst`,
//! `+4 u32 sender sequence`, `+8 u32 payload length`, `+12 u32 bitmap`.

/// Total IPC packet size in bytes.
pub const PKT_LEN: usize = 76;

/// Change-device-state request event id.
pub const EVENT_CHANGE: u16 = 0x0045;
/// Acknowledgement event id.
pub const EVENT_ACK: u16 = 0x00a9;
/// RoboController module id.
pub const MODULE_RC: u8 = 1;
/// McuProxy (Player) module id.
pub const MODULE_MC: u8 = 4;

/// Stock standby bitmap (measured on the wire after a cleaning stop).
pub const MASK_STANDBY: u32 = 0x0000_0040;
/// Stock cleaning bitmap (both brushes + LDS + fan).
pub const MASK_STOCK_CLEANING: u32 = 0x0010_feff;
/// Candidate proven live on 2026-10-03: cleaning minus both brushes,
/// LDS spinning and scans flowing, brushes observed motionless.
pub const MASK_NO_BRUSHES: u32 = 0x0010_f8ff;

/// Routing header of one 76-byte packet.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Header {
    pub event: u16,
    pub src: u8,
    pub dst: u8,
    pub seq: u32,
}

/// Errors of the owner session.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OwnerError {
    /// The bitmap is empty, uses unknown bits, or sets both LDS bits.
    BadBitmap,
    /// Writing the request failed (e.g. EPIPE).
    WriteFailed,
    /// Fewer than 76 bytes written.
    ShortWrite,
    /// Reading the reply failed for another reason.
    ReadFailed,
    /// No acknowledgement within the timeout (foreign packets skipped
    /// beyond the bound also land here as Timeout, mirroring the finite
    /// deadline the stock RoboController lacks).
    Timeout,
    /// The peer closed the channel.
    Eof,
    /// Fewer than 76 bytes read: the stream is no longer frame-aligned.
    ShortRead,
    /// An ack-shaped packet arrived from the wrong direction: the channel
    /// is corrupt, retrying or skipping cannot help.
    CorruptChannel,
}

impl std::fmt::Display for OwnerError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let s = match self {
            OwnerError::BadBitmap => "bad bitmap",
            OwnerError::WriteFailed => "write failed",
            OwnerError::ShortWrite => "short write",
            OwnerError::ReadFailed => "read failed",
            OwnerError::Timeout => "timeout",
            OwnerError::Eof => "channel closed",
            OwnerError::ShortRead => "short read",
            OwnerError::CorruptChannel => "corrupt channel",
        };
        f.write_str(s)
    }
}

impl std::error::Error for OwnerError {}

/// Decode the routing header of a packet.
pub fn parse_header(pkt: &[u8]) -> Option<Header> {
    if pkt.len() < 8 {
        return None;
    }
    Some(Header {
        event: u16::from_le_bytes([pkt[0], pkt[1]]),
        src: pkt[2],
        dst: pkt[3],
        seq: u32::from_le_bytes(pkt[4..8].try_into().unwrap()),
    })
}

/// Build the 76-byte Rc->Mc change-device-state request.
///
/// Rejects bitmaps that are empty, use bits outside the known device set,
/// or set both LDS bits at once (they are two mutually exclusive modes).
pub fn build_request(seq: u32, bitmap: u32) -> Result<[u8; PKT_LEN], OwnerError> {
    const KNOWN_BITS: u32 = 0x0017_ffff;
    const LDS: u32 = 1 << 7;
    const LDS_F: u32 = 1 << 8;
    if bitmap == 0 || bitmap & !KNOWN_BITS != 0 || (bitmap & LDS != 0 && bitmap & LDS_F != 0) {
        return Err(OwnerError::BadBitmap);
    }
    let mut pkt = [0u8; PKT_LEN];
    pkt[0..2].copy_from_slice(&EVENT_CHANGE.to_le_bytes());
    pkt[2] = MODULE_RC;
    pkt[3] = MODULE_MC;
    pkt[4..8].copy_from_slice(&seq.to_le_bytes());
    pkt[8..12].copy_from_slice(&4u32.to_le_bytes());
    pkt[12..16].copy_from_slice(&bitmap.to_le_bytes());
    Ok(pkt)
}

/// Whether a packet is the acknowledgement of our request, matched by
/// event id and direction: the responder's sequence counter is its own
/// and is deliberately not checked.
pub fn is_ack(pkt: &[u8]) -> AckShape {
    if pkt.len() != PKT_LEN {
        return AckShape::TooShort;
    }
    let Some(h) = parse_header(pkt) else {
        return AckShape::TooShort;
    };
    if h.event == EVENT_ACK {
        if h.src == MODULE_MC && h.dst == MODULE_RC {
            AckShape::Ack
        } else {
            AckShape::AckWrongDirection
        }
    } else {
        AckShape::NotAnAck
    }
}

/// Shape of an incoming packet relative to the awaited acknowledgement.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AckShape {
    Ack,
    AckWrongDirection,
    NotAnAck,
    TooShort,
}

pub mod ipc;

#[cfg(test)]
mod tests {
    use super::*;

    // Real wire vectors captured in the saved trace (2026-10-03):
    // request 45 00 01 04 | 36 00 00 00 | 04 00 00 00 | ff fe 10 00
    // ack     a9 00 04 01 | 50 00 00 00 | ...
    #[test]
    fn request_matches_traced_packet() {
        let pkt = build_request(0x36, MASK_STOCK_CLEANING).unwrap();
        assert_eq!(
            &pkt[..16],
            &[
                0x45, 0x00, 0x01, 0x04, 0x36, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0xff, 0xfe,
                0x10, 0x00
            ]
        );
        assert_eq!(pkt.len(), 76);
    }

    #[test]
    fn header_roundtrip() {
        let pkt = build_request(0x42, MASK_NO_BRUSHES).unwrap();
        let h = parse_header(&pkt).unwrap();
        assert_eq!(h.event, EVENT_CHANGE);
        assert_eq!(h.src, MODULE_RC);
        assert_eq!(h.dst, MODULE_MC);
        assert_eq!(h.seq, 0x42);
        assert_eq!(
            u32::from_le_bytes(pkt[12..16].try_into().unwrap()),
            MASK_NO_BRUSHES
        );
    }

    #[test]
    fn rejects_bad_bitmaps() {
        assert_eq!(build_request(1, 0), Err(OwnerError::BadBitmap));
        assert_eq!(build_request(1, 1 << 19), Err(OwnerError::BadBitmap));
        assert_eq!(build_request(1, 1 << 24), Err(OwnerError::BadBitmap));
        assert_eq!(
            build_request(1, (1 << 7) | (1 << 8)),
            Err(OwnerError::BadBitmap)
        );
    }

    #[test]
    fn ack_detection() {
        let mut ack = [0u8; PKT_LEN];
        ack[0..2].copy_from_slice(&EVENT_ACK.to_le_bytes());
        ack[2] = MODULE_MC;
        ack[3] = MODULE_RC;
        ack[4..8].copy_from_slice(&0x50u32.to_le_bytes());
        assert_eq!(is_ack(&ack), AckShape::Ack);

        let mut wrong = ack;
        wrong[2] = MODULE_RC;
        wrong[3] = MODULE_MC;
        assert_eq!(is_ack(&wrong), AckShape::AckWrongDirection);

        let mut foreign = [0u8; PKT_LEN];
        foreign[0..2].copy_from_slice(&0x01f9u16.to_le_bytes());
        foreign[2] = MODULE_MC;
        foreign[3] = MODULE_RC;
        assert_eq!(is_ack(&foreign), AckShape::NotAnAck);

        assert_eq!(is_ack(&ack[..10]), AckShape::TooShort);
    }
}
