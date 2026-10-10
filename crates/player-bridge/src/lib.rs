//! Player wire-protocol core (no ROS here — see main.rs for the node).
//!
//! What this implements, and why it is enough for the bridge:
//! - TCP connect: the server sends a 32-byte banner "Player v..." first;
//! - messages: a 40-byte XDR header (PLAYERXDR_MSGHDR_SIZE) followed by
//!   `size` payload bytes. XDR encodes every field 4-byte aligned,
//!   big-endian; the header is two u32, two u16 (each padded), two u8
//!   (padded), one f64, two u32;
//! - device subscribe: PLAYER_PLAYER_REQ_DEV (subtype 3) on the `player`
//!   device with the target address and PLAYER_OPEN_MODE; the reply
//!   echoes the granted access;
//! - push mode (default): the server streams DATA messages on its own.
//!
//! Ground truth for these constants: libplayerinterface headers and
//! client.c of the locally built Player 3.1 tree; the integration test
//! runs against that real server (see tests/against_server.rs).

pub mod drive;

use std::io::{Read, Write};
use std::net::TcpStream;
use std::time::Duration;

pub const MSGTYPE_DATA: u8 = 1;
pub const MSGTYPE_REQ: u8 = 3;
pub const MSGTYPE_RESP_ACK: u8 = 4;
pub const MSGTYPE_RESP_NACK: u8 = 6;

pub const PLAYER_CODE: u16 = 1; // "player" meta-device
pub const POSITION2D_CODE: u16 = 4;
pub const LASER_CODE: u16 = 6;

pub const REQ_DEV: u8 = 3; // PLAYER_PLAYER_REQ_DEV
pub const OPEN_MODE: u8 = 1;

pub const LASER_DATA_SCAN: u8 = 1; // PLAYER_LASER_DATA_SCAN
pub const LASER_DATA_SCANANGLE: u8 = 3; // PLAYER_LASER_DATA_SCANANGLE (vendor form)
pub const POSITION2D_DATA_STATE: u8 = 1;

pub const MSGHDR_SIZE: usize = 40;

/// Routing address of one device (XDR: u32, u32, u16, u16).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DevAddr {
    pub host: u32,
    pub robot: u32,
    pub interf: u16,
    pub index: u16,
}

/// Decoded 40-byte message header.
#[derive(Debug, Clone, Copy)]
pub struct MsgHeader {
    pub addr: DevAddr,
    pub msg_type: u8,
    pub subtype: u8,
    pub timestamp: f64,
    pub seq: u32,
    pub size: u32,
}

/// A full message: header plus payload bytes.
#[derive(Debug, Clone)]
pub struct Message {
    pub header: MsgHeader,
    pub payload: Vec<u8>,
}

/// A decoded laser scan (player_laser_data_t).
#[derive(Debug, Clone, PartialEq)]
pub struct LaserScan {
    pub min_angle: f32,
    pub max_angle: f32,
    pub resolution: f32,
    pub max_range: f32,
    pub ranges: Vec<f32>,
    pub intensities: Vec<u8>,
    pub id: u32,
}

/// A decoded position2d state (player_position2d_data_t).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Position2dState {
    pub x: f64,
    pub y: f64,
    pub yaw: f64,
    pub vx: f64,
    pub vy: f64,
    pub vyaw: f64,
    pub stalled: bool,
}

#[derive(Debug)]
pub enum Error {
    Io(std::io::Error),
    BadBanner,
    Nak,
    UnexpectedReply { msg_type: u8, subtype: u8 },
    ShortPayload { need: usize, got: usize },
    /// A payload's internal length fields disagree: XDR framing is broken.
    CorruptChannel,
}

impl std::fmt::Display for Error {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Error::Io(e) => write!(f, "io: {e}"),
            Error::BadBanner => write!(f, "bad banner"),
            Error::Nak => write!(f, "request nacked"),
            Error::UnexpectedReply { msg_type, subtype } => {
                write!(f, "unexpected reply type={msg_type} subtype={subtype}")
            }
            Error::ShortPayload { need, got } => {
                write!(f, "short payload: need {need}, got {got}")
            }
            Error::CorruptChannel => write!(f, "corrupt channel framing"),
        }
    }
}

impl std::error::Error for Error {}

impl From<std::io::Error> for Error {
    fn from(e: std::io::Error) -> Self {
        Error::Io(e)
    }
}

fn be32(buf: &[u8], off: usize) -> u32 {
    u32::from_be_bytes(buf[off..off + 4].try_into().unwrap())
}

fn be16(buf: &[u8], off: usize) -> u16 {
    u16::from_be_bytes(buf[off..off + 2].try_into().unwrap())
}

fn bef64(buf: &[u8], off: usize) -> f64 {
    f64::from_be_bytes(buf[off..off + 8].try_into().unwrap())
}

fn bef32(buf: &[u8], off: usize) -> f32 {
    f32::from_be_bytes(buf[off..off + 4].try_into().unwrap())
}

fn put32(buf: &mut [u8], off: usize, v: u32) {
    buf[off..off + 4].copy_from_slice(&v.to_be_bytes());
}

fn put16(buf: &mut [u8], off: usize, v: u16) {
    buf[off..off + 2].copy_from_slice(&v.to_be_bytes());
}

fn putf64(buf: &mut [u8], off: usize, v: f64) {
    buf[off..off + 8].copy_from_slice(&v.to_be_bytes());
}

/// Decode a 40-byte XDR message header.
///
/// XDR layout (every scalar occupies 4 bytes, big-endian; the double is
/// 8 bytes at offset 24, which is already 8-aligned):
/// host u32 @0, robot u32 @4, interf u32 @8, index u32 @12, type u8 @16,
/// subtype u8 @20, timestamp f64 @24, seq u32 @32, size u32 @36.
pub fn decode_header(buf: &[u8]) -> Option<MsgHeader> {
    if buf.len() < MSGHDR_SIZE {
        return None;
    }
    let addr = DevAddr {
        host: be32(buf, 0),
        robot: be32(buf, 4),
        interf: be32(buf, 8) as u16,
        index: be32(buf, 12) as u16,
    };
    let msg_type = be32(buf, 16) as u8;
    let subtype = be32(buf, 20) as u8;
    let timestamp = bef64(buf, 24);
    let seq = be32(buf, 32);
    let size = be32(buf, 36);
    Some(MsgHeader { addr, msg_type, subtype, timestamp, seq, size })
}

/// Encode a 40-byte XDR message header.
///
/// u8 fields (type, subtype) live in 4-byte XDR slots with the value in
/// the LOW byte — verified against a tcpdump of a real libplayerc client:
/// writing the value into the high byte makes the server decode type=0
/// and drop the message as "unsupported type unknown:0".
pub fn encode_header(hdr: &MsgHeader) -> [u8; MSGHDR_SIZE] {
    let mut buf = [0u8; MSGHDR_SIZE];
    put32(&mut buf, 0, hdr.addr.host);
    put32(&mut buf, 4, hdr.addr.robot);
    put32(&mut buf, 8, hdr.addr.interf as u32);
    put32(&mut buf, 12, hdr.addr.index as u32);
    put32(&mut buf, 16, hdr.msg_type as u32);
    put32(&mut buf, 20, hdr.subtype as u32);
    putf64(&mut buf, 24, hdr.timestamp);
    put32(&mut buf, 32, hdr.seq);
    put32(&mut buf, 36, hdr.size);
    buf
}

/// Decode a player_laser_data_t payload (XDR).
///
/// Wire layout (verified against the local Player server dump): 4 f32
/// (min, max, res, max_range); u32 ranges_count; xdr_array writes ANOTHER
/// u32 count before the f32 data; then u32 intensity_count; xdr_bytes
/// writes another u32 count before the packed bytes (padded to a word);
/// then u32 id. The doubled counts are an XDR artifact: both the struct
/// field and the array encoder serialize their own length.
pub fn decode_laser_scan(payload: &[u8]) -> Result<LaserScan, Error> {
    if payload.len() < 24 {
        return Err(Error::ShortPayload { need: 24, got: payload.len() });
    }
    let min_angle = bef32(payload, 0);
    let max_angle = bef32(payload, 4);
    let resolution = bef32(payload, 8);
    let max_range = bef32(payload, 12);
    let ranges_count = be32(payload, 16) as usize;
    let mut off = 20;
    if off + 4 > payload.len() {
        return Err(Error::ShortPayload { need: off + 4, got: payload.len() });
    }
    let array_count = be32(payload, off) as usize;
    if array_count != ranges_count {
        return Err(Error::CorruptChannel);
    }
    off += 4;
    let mut ranges = Vec::with_capacity(ranges_count);
    for _ in 0..ranges_count {
        if off + 4 > payload.len() {
            return Err(Error::ShortPayload { need: off + 4, got: payload.len() });
        }
        ranges.push(bef32(payload, off));
        off += 4;
    }
    if off + 4 > payload.len() {
        return Err(Error::ShortPayload { need: off + 4, got: payload.len() });
    }
    let intensity_count = be32(payload, off) as usize;
    off += 4;
    if off + 4 > payload.len() {
        return Err(Error::ShortPayload { need: off + 4, got: payload.len() });
    }
    let bytes_count = be32(payload, off) as usize;
    if bytes_count != intensity_count {
        return Err(Error::CorruptChannel);
    }
    off += 4;
    let mut intensities = Vec::with_capacity(intensity_count);
    for i in 0..intensity_count {
        if off + i >= payload.len() {
            return Err(Error::ShortPayload { need: off + i + 1, got: payload.len() });
        }
        intensities.push(payload[off + i]);
    }
    off += intensity_count + ((4 - intensity_count % 4) % 4);
    if off + 4 > payload.len() {
        return Err(Error::ShortPayload { need: off + 4, got: payload.len() });
    }
    let id = be32(payload, off);
    Ok(LaserScan { min_angle, max_angle, resolution, max_range, ranges, intensities, id })
}

/// Decode a player_position2d_data_t payload (XDR).
pub fn decode_position2d(payload: &[u8]) -> Result<Position2dState, Error> {
    // pose: 3 f64; vel: 3 f64; stall u8 (padded)
    if payload.len() < 49 {
        return Err(Error::ShortPayload { need: 49, got: payload.len() });
    }
    Ok(Position2dState {
        x: bef64(payload, 0),
        y: bef64(payload, 8),
        yaw: bef64(payload, 16),
        vx: bef64(payload, 24),
        vy: bef64(payload, 32),
        vyaw: bef64(payload, 40),
        stalled: payload[48] != 0,
    })
}

/// Connected client speaking the subset the bridge needs.
pub struct Client {
    stream: TcpStream,
}

impl Client {
    pub fn connect(host: &str, port: u16) -> Result<Self, Error> {
        let stream = TcpStream::connect((host, port))?;
        stream.set_read_timeout(Some(Duration::from_secs(5)))?;
        let mut client = Client { stream };
        let mut banner = [0u8; 32];
        client.stream.read_exact(&mut banner)?;
        if &banner[..9] != b"Player v." {
            return Err(Error::BadBanner);
        }
        Ok(client)
    }

    fn send(&mut self, header: &MsgHeader, payload: &[u8]) -> Result<(), Error> {
        self.stream.write_all(&encode_header(header))?;
        self.stream.write_all(payload)?;
        Ok(())
    }

    /// Read one full message (header + payload).
    pub fn read_message(&mut self) -> Result<Message, Error> {
        let mut hdr_buf = [0u8; MSGHDR_SIZE];
        self.stream.read_exact(&mut hdr_buf)?;
        let header = decode_header(&hdr_buf).ok_or(Error::BadBanner)?;
        let mut payload = vec![0u8; header.size as usize];
        if header.size > 0 {
            self.stream.read_exact(&mut payload)?;
        }
        Ok(Message { header, payload })
    }

    /// Subscribe to a device; returns the granted access code.
    ///
    /// player_device_req_t XDR encoding: devaddr (16 bytes), u8 access in
    /// a 4-byte slot @16, u32 driver_name_count @20, then xdr_bytes for
    /// the name writes ANOTHER u32 count @24 followed by the data — with
    /// a zero-length name the total is 28 bytes. (Sending the padded
    /// 88-byte C struct makes the live server log "unknown access mode 0"
    /// and silently drop the request.)
    pub fn subscribe(&mut self, interf: u16, index: u16) -> Result<u8, Error> {
        let mut req = [0u8; 28];
        put32(&mut req, 8, interf as u32);
        put32(&mut req, 12, index as u32);
        // XDR u_char occupies a full 4-byte big-endian slot: the value must
        // land in the LOW byte. Writing it to the high byte makes the server
        // decode access=0 ("unknown access mode 0 requested") and reply
        // with a NACK whose size coincides with an ACK.
        put32(&mut req, 16, OPEN_MODE as u32);
        put32(&mut req, 20, 0); // driver_name_count
        put32(&mut req, 24, 0); // xdr_bytes inline count

        let header = MsgHeader {
            addr: DevAddr { host: 0, robot: 0, interf: PLAYER_CODE, index: 0 },
            msg_type: MSGTYPE_REQ,
            subtype: REQ_DEV,
            timestamp: 0.0,
            seq: 0,
            size: req.len() as u32,
        };
        self.send(&header, &req)?;
        // In push mode the server may start streaming device data
        // immediately after granting access; skip DATA messages until
        // the acknowledgement arrives.
        let reply = loop {
            let msg = self.read_message()?;
            if msg.header.msg_type == MSGTYPE_DATA {
                continue;
            }
            break msg;
        };
        match reply.header.msg_type {
            MSGTYPE_RESP_ACK => Ok(reply.payload.get(19).copied().unwrap_or(0)),
            MSGTYPE_RESP_NACK => Err(Error::Nak),
            t => Err(Error::UnexpectedReply { msg_type: t, subtype: reply.header.subtype }),
        }
    }

    /// Send a position2d velocity command (player_position2d_cmd_vel_t).
    pub fn send_velocity(&mut self, index: u16, vx: f64, vyaw: f64, state: u8) -> Result<(), Error> {
        // pose: 3 f64; state u8 padded to 4
        let mut cmd = [0u8; 28];
        putf64(&mut cmd, 0, vx);
        putf64(&mut cmd, 8, 0.0);
        putf64(&mut cmd, 16, vyaw);
        cmd[24] = state;
        let header = MsgHeader {
            addr: DevAddr { host: 0, robot: 0, interf: POSITION2D_CODE, index },
            msg_type: 2, // PLAYER_MSGTYPE_CMD
            subtype: 1,  // PLAYER_POSITION2D_CMD_VEL
            timestamp: 0.0,
            seq: 0,
            size: cmd.len() as u32,
        };
        self.send(&header, &cmd)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn subscribe_payload_places_access_at_16() {
        // Mirrors what Client::subscribe builds; asserted byte-exactly to
        // catch XDR offset regressions (a live server logs "unknown access
        // mode 0" when this breaks).
        let mut req = [0u8; 28];
        put32(&mut req, 8, LASER_CODE as u32);
        put32(&mut req, 12, 0);
        put32(&mut req, 16, OPEN_MODE as u32);
        put32(&mut req, 20, 0);
        assert_eq!(&req[8..12], &[0, 0, 0, 6]);
        assert_eq!(&req[16..20], &[0, 0, 0, 1]);
    }
}

/// A decoded laser scan in per-beam angle form (player_laser_data_scanangle_t,
/// subtype 3): the vendor ruby_laser driver publishes this on the real robot.
#[derive(Debug, Clone, PartialEq)]
pub struct LaserScanAngle {
    pub max_range: f32,
    pub ranges: Vec<f32>,
    /// Per-beam bearing [rad] — the authoritative geometry, since the
    /// driver does not publish a single start/resolution pair.
    pub angles: Vec<f32>,
    pub intensities: Vec<u8>,
    pub id: u32,
}

/// Decode a player_laser_data_scanangle_t payload (XDR).
pub fn decode_laser_scanangle(payload: &[u8]) -> Result<LaserScanAngle, Error> {
    let rd_u32 = |off: usize| -> Result<u32, Error> {
        payload
            .get(off..off + 4)
            .map(|b| u32::from_be_bytes(b.try_into().unwrap()))
            .ok_or(Error::ShortPayload { need: off + 4, got: payload.len() })
    };
    let rd_f32 = |off: usize| -> Result<f32, Error> {
        payload
            .get(off..off + 4)
            .map(|b| f32::from_be_bytes(b.try_into().unwrap()))
            .ok_or(Error::ShortPayload { need: off + 4, got: payload.len() })
    };
    let mut off = 0;
    let max_range = rd_f32(off)?;
    off += 4;

    let read_array_f32 = |off: &mut usize| -> Result<Vec<f32>, Error> {
        let count = rd_u32(*off)? as usize;
        *off += 4;
        // xdr_array duplicate count word
        let dup = rd_u32(*off)? as usize;
        if dup != count {
            return Err(Error::CorruptChannel);
        }
        *off += 4;
        let mut v = Vec::with_capacity(count);
        for _ in 0..count {
            v.push(rd_f32(*off)?);
            *off += 4;
        }
        Ok(v)
    };

    let ranges = read_array_f32(&mut off)?;
    let angles = read_array_f32(&mut off)?;
    let intensity_count = rd_u32(off)? as usize;
    off += 4;
    let dup = rd_u32(off)? as usize;
    if dup != intensity_count {
        return Err(Error::CorruptChannel);
    }
    off += 4;
    let mut intensities = Vec::with_capacity(intensity_count);
    for i in 0..intensity_count {
        match payload.get(off + i) {
            Some(&b) => intensities.push(b),
            None => return Err(Error::ShortPayload { need: off + i + 1, got: payload.len() }),
        }
    }
    off += intensity_count + ((4 - intensity_count % 4) % 4);
    let id = rd_u32(off)?;
    Ok(LaserScanAngle { max_range, ranges, angles, intensities, id })
}
