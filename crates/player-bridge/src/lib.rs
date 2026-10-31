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

pub mod bumper;
pub mod drive;
pub mod scan;
pub mod sensors;

pub use bumper::{decode_bumper, BumperState, BUMPER_CODE, BUMPER_DATA_STATE};
pub use sensors::{
    decode_ir, decode_position3d, decode_power, decode_sonar, IrState, Position3dState, PowerState,
    SonarState, IR_CODE, IR_DATA_RANGES, POSITION3D_CODE, POSITION3D_DATA_STATE, POWER_CODE,
    POWER_DATA_STATE, SONAR_CODE, SONAR_DATA_RANGES,
};

use std::collections::VecDeque;
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

pub const MSGTYPE_CMD: u8 = 2; // PLAYER_MSGTYPE_CMD
/// PLAYER_POSITION2D_CMD_VEL (`player_position2d_cmd_vel_t`).
pub const POSITION2D_CMD_VEL: u8 = 1;

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
    UnexpectedReply {
        msg_type: u8,
        subtype: u8,
    },
    ShortPayload {
        need: usize,
        got: usize,
    },
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

fn bef64(buf: &[u8], off: usize) -> f64 {
    f64::from_be_bytes(buf[off..off + 8].try_into().unwrap())
}

fn bef32(buf: &[u8], off: usize) -> f32 {
    f32::from_be_bytes(buf[off..off + 4].try_into().unwrap())
}

fn put32(buf: &mut [u8], off: usize, v: u32) {
    buf[off..off + 4].copy_from_slice(&v.to_be_bytes());
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
    Some(MsgHeader {
        addr,
        msg_type,
        subtype,
        timestamp,
        seq,
        size,
    })
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
        return Err(Error::ShortPayload {
            need: 24,
            got: payload.len(),
        });
    }
    let min_angle = bef32(payload, 0);
    let max_angle = bef32(payload, 4);
    let resolution = bef32(payload, 8);
    let max_range = bef32(payload, 12);
    let ranges_count = be32(payload, 16) as usize;
    let mut off = 20;
    if off + 4 > payload.len() {
        return Err(Error::ShortPayload {
            need: off + 4,
            got: payload.len(),
        });
    }
    let array_count = be32(payload, off) as usize;
    if array_count != ranges_count {
        return Err(Error::CorruptChannel);
    }
    off += 4;
    let mut ranges = Vec::with_capacity(ranges_count);
    for _ in 0..ranges_count {
        if off + 4 > payload.len() {
            return Err(Error::ShortPayload {
                need: off + 4,
                got: payload.len(),
            });
        }
        ranges.push(bef32(payload, off));
        off += 4;
    }
    if off + 4 > payload.len() {
        return Err(Error::ShortPayload {
            need: off + 4,
            got: payload.len(),
        });
    }
    let intensity_count = be32(payload, off) as usize;
    off += 4;
    if off + 4 > payload.len() {
        return Err(Error::ShortPayload {
            need: off + 4,
            got: payload.len(),
        });
    }
    let bytes_count = be32(payload, off) as usize;
    if bytes_count != intensity_count {
        return Err(Error::CorruptChannel);
    }
    off += 4;
    let mut intensities = Vec::with_capacity(intensity_count);
    for i in 0..intensity_count {
        if off + i >= payload.len() {
            return Err(Error::ShortPayload {
                need: off + i + 1,
                got: payload.len(),
            });
        }
        intensities.push(payload[off + i]);
    }
    off += intensity_count + ((4 - intensity_count % 4) % 4);
    if off + 4 > payload.len() {
        return Err(Error::ShortPayload {
            need: off + 4,
            got: payload.len(),
        });
    }
    let id = be32(payload, off);
    Ok(LaserScan {
        min_angle,
        max_angle,
        resolution,
        max_range,
        ranges,
        intensities,
        id,
    })
}

/// Decode a player_position2d_data_t payload (XDR).
pub fn decode_position2d(payload: &[u8]) -> Result<Position2dState, Error> {
    // pose: 3 f64; vel: 3 f64; stall u8 (padded)
    if payload.len() < 49 {
        return Err(Error::ShortPayload {
            need: 49,
            got: payload.len(),
        });
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
    pending_data: VecDeque<Message>,
}

impl Client {
    pub fn connect(host: &str, port: u16) -> Result<Self, Error> {
        let stream = TcpStream::connect((host, port))?;
        stream.set_read_timeout(Some(Duration::from_secs(5)))?;
        let mut client = Client {
            stream,
            pending_data: VecDeque::new(),
        };
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

    /// Read one full message, including DATA buffered during subscriptions.
    pub fn read_message(&mut self) -> Result<Message, Error> {
        if let Some(msg) = self.pending_data.pop_front() {
            return Ok(msg);
        }
        self.read_wire_message()
    }

    fn read_wire_message(&mut self) -> Result<Message, Error> {
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
            addr: DevAddr {
                host: 0,
                robot: 0,
                interf: PLAYER_CODE,
                index: 0,
            },
            msg_type: MSGTYPE_REQ,
            subtype: REQ_DEV,
            timestamp: 0.0,
            seq: 0,
            size: req.len() as u32,
        };
        self.send(&header, &req)?;
        // DATA can precede this reply, including one-shot initial states
        // from earlier subscriptions. Preserve it for the caller; read only
        // the wire here so later subscriptions do not reconsume the queue.
        let reply = loop {
            let msg = self.read_wire_message()?;
            if msg.header.msg_type == MSGTYPE_DATA {
                self.pending_data.push_back(msg);
                continue;
            }
            break msg;
        };
        match reply.header.msg_type {
            MSGTYPE_RESP_ACK => Ok(reply.payload.get(19).copied().unwrap_or(0)),
            MSGTYPE_RESP_NACK => Err(Error::Nak),
            t => Err(Error::UnexpectedReply {
                msg_type: t,
                subtype: reply.header.subtype,
            }),
        }
    }

    /// Send a position2d velocity command (player_position2d_cmd_vel_t).
    pub fn send_velocity(
        &mut self,
        index: u16,
        vx: f64,
        vyaw: f64,
        state: u8,
    ) -> Result<(), Error> {
        let (header, cmd) = velocity_packet(index, vx, vyaw, state);
        self.send(&header, &cmd)
    }
}

/// Encode a player_position2d_cmd_vel_t message: 40-byte XDR header plus a
/// 28-byte payload (pose: 3 f64 with vy=0; state u8 in a padded slot).
/// Byte-exact home of the velocity wire form; the sync `Client` and every
/// async writer must share it so both transports stay identical.
pub fn velocity_packet(index: u16, vx: f64, vyaw: f64, state: u8) -> (MsgHeader, [u8; 28]) {
    // pose: 3 f64; state u8 padded to 4
    let mut cmd = [0u8; 28];
    putf64(&mut cmd, 0, vx);
    putf64(&mut cmd, 8, 0.0);
    putf64(&mut cmd, 16, vyaw);
    cmd[24] = state;
    let header = MsgHeader {
        addr: DevAddr {
            host: 0,
            robot: 0,
            interf: POSITION2D_CODE,
            index,
        },
        msg_type: MSGTYPE_CMD,
        subtype: POSITION2D_CMD_VEL,
        timestamp: 0.0,
        seq: 0,
        size: cmd.len() as u32,
    };
    (header, cmd)
}

#[cfg(test)]
mod tests {
    use super::*;

    // BUG: subscription round trips discard initial, possibly one-shot sensor DATA.
    // HYPOTHESIS: DATA before ACK/NACK must be queued, not skipped.
    // SOLUTION: queue DATA while reading replies directly from the wire.
    // RESULT: FIXED — initial states are delivered once, in wire order, after ACK/NACK.
    #[test]
    fn subscription_round_trips_preserve_data_in_wire_order() {
        use std::net::TcpListener;

        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let port = listener.local_addr().unwrap().port();
        let devices = [
            (BUMPER_CODE, 0),
            (IR_CODE, 1),
            (IR_CODE, 2),
            (POWER_CODE, 0),
        ];
        let server = std::thread::spawn(move || {
            let (mut stream, _) = listener.accept().unwrap();
            stream
                .set_read_timeout(Some(Duration::from_secs(2)))
                .unwrap();
            let mut banner = [0; 32];
            banner[..9].copy_from_slice(b"Player v.");
            stream.write_all(&banner).unwrap();
            let send =
                |stream: &mut TcpStream, interf, index, msg_type, subtype, payload: &[u8]| {
                    let header = MsgHeader {
                        addr: DevAddr {
                            host: 0,
                            robot: 0,
                            interf,
                            index,
                        },
                        msg_type,
                        subtype,
                        timestamp: 123.0,
                        seq: 0,
                        size: payload.len() as u32,
                    };
                    stream.write_all(&encode_header(&header)).unwrap();
                    stream.write_all(payload).unwrap();
                };
            for round in 0..4 {
                let mut header = [0; MSGHDR_SIZE];
                stream.read_exact(&mut header).unwrap();
                let header = decode_header(&header).unwrap();
                assert_eq!((header.msg_type, header.subtype), (MSGTYPE_REQ, REQ_DEV));
                let mut request = vec![0; header.size as usize];
                stream.read_exact(&mut request).unwrap();
                // Initial bumper/cliff/drop states arrive after an earlier ACK,
                // during the next subscribe; power arrives before its own ACK.
                let events: &[usize] = match round {
                    0 => &[],
                    1 => &[0, 1],
                    2 => &[2],
                    _ => &[3],
                };
                for &event in events {
                    let (interf, index) = devices[event];
                    send(&mut stream, interf, index, MSGTYPE_DATA, 1, &[event as u8]);
                }
                let mut reply = [0; 28];
                reply[19] = OPEN_MODE;
                send(
                    &mut stream,
                    PLAYER_CODE,
                    0,
                    if round == 2 {
                        MSGTYPE_RESP_NACK
                    } else {
                        MSGTYPE_RESP_ACK
                    },
                    REQ_DEV,
                    &reply,
                );
            }
            // A fresh wire message must follow buffered DATA without duplication.
            send(&mut stream, SONAR_CODE, 0, MSGTYPE_DATA, 1, &[4]);
        });

        let mut client = Client::connect("127.0.0.1", port).unwrap();
        assert_eq!(client.subscribe(BUMPER_CODE, 0).unwrap(), OPEN_MODE);
        assert_eq!(client.subscribe(IR_CODE, 1).unwrap(), OPEN_MODE);
        assert!(matches!(client.subscribe(SONAR_CODE, 1), Err(Error::Nak)));
        assert_eq!(client.subscribe(POWER_CODE, 0).unwrap(), OPEN_MODE);
        for (event, (interf, index)) in devices.into_iter().chain([(SONAR_CODE, 0)]).enumerate() {
            let msg = client
                .read_message()
                .expect("initial sensor DATA must survive subscription");
            assert_eq!(msg.header.msg_type, MSGTYPE_DATA);
            assert_eq!(
                (msg.header.addr.interf, msg.header.addr.index),
                (interf, index),
                "initial sensor DATA must survive subscription in wire order"
            );
            assert_eq!(msg.header.timestamp, 123.0);
            assert_eq!(msg.payload, vec![event as u8]);
        }
        assert!(
            client.read_message().is_err(),
            "no DATA may be delivered twice"
        );
        server.join().unwrap();
    }

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

    // DEDUP (issue #51): the velocity wire form had two encoders (sync
    // Client::send_velocity and robot-server's async writer). Byte-exact
    // acceptance test for the single shared encoder both transports use.
    #[test]
    fn velocity_packet_is_byte_exact_xdr_with_padded_state_and_index() {
        let (header, cmd) = velocity_packet(7, -0.06, 2.5, 1);
        let wire = [encode_header(&header).as_slice(), &cmd].concat();
        // Header: position2d (4) @8, index (7) @12, CMD (2) @16, VEL (1) @20,
        // zero timestamp/seq, size 28.
        assert_eq!(&wire[8..12], &4u32.to_be_bytes());
        assert_eq!(&wire[12..16], &7u32.to_be_bytes());
        assert_eq!(&wire[16..20], &2u32.to_be_bytes());
        assert_eq!(&wire[20..24], &1u32.to_be_bytes());
        assert_eq!(&wire[24..32], &0f64.to_be_bytes());
        assert_eq!(&wire[32..36], &0u32.to_be_bytes());
        assert_eq!(&wire[36..40], &28u32.to_be_bytes());
        // Payload: vx @0, vy=0 @8, vyaw @16, state in the LOW byte @24 with
        // XDR zero padding after it.
        assert_eq!(&wire[40..48], &(-0.06f64).to_be_bytes());
        assert_eq!(&wire[48..56], &0f64.to_be_bytes());
        assert_eq!(&wire[56..64], &2.5f64.to_be_bytes());
        assert_eq!(&wire[64..68], &[1, 0, 0, 0]);
        assert_eq!(wire.len(), 68);
        // An old live session (start-stop against the real Player server)
        // produced exactly this zero-command frame for index 0.
        let (_, stop) = velocity_packet(0, 0.0, 0.0, 1);
        assert_eq!(stop, {
            let mut zeroed = [0u8; 28];
            zeroed[24] = 1;
            zeroed
        });
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
    pub intensities: Vec<u16>,
    pub id: u32,
}

/// Decode a player_laser_data_scanangle_t payload (XDR).
pub fn decode_laser_scanangle(payload: &[u8]) -> Result<LaserScanAngle, Error> {
    let rd_u32 = |off: usize| -> Result<u32, Error> {
        payload
            .get(off..off + 4)
            .map(|b| u32::from_be_bytes(b.try_into().unwrap()))
            .ok_or(Error::ShortPayload {
                need: off + 4,
                got: payload.len(),
            })
    };
    let rd_f32 = |off: usize| -> Result<f32, Error> {
        payload
            .get(off..off + 4)
            .map(|b| f32::from_be_bytes(b.try_into().unwrap()))
            .ok_or(Error::ShortPayload {
                need: off + 4,
                got: payload.len(),
            })
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
        // Guard the allocation: a hostile count must fail before reserving.
        if count > payload.len().saturating_sub(*off) / 4 {
            return Err(Error::ShortPayload {
                need: *off + count * 4,
                got: payload.len(),
            });
        }
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
    let mut intensities = Vec::new();
    for _ in 0..intensity_count {
        let value = u16::try_from(rd_u32(off)?).map_err(|_| Error::CorruptChannel)?;
        intensities.push(value);
        off += 4;
    }
    let id = rd_u32(off)?;
    Ok(LaserScanAngle {
        max_range,
        ranges,
        angles,
        intensities,
        id,
    })
}

/// One decoded Player DATA message: each subscribed sensor frame is decoded
/// exactly once into its typed state (issue #51 dedup) so telemetry, odometry
/// history, safety and SLAM all consume the same decode.
#[derive(Debug, Clone, PartialEq)]
pub enum PlayerData {
    /// `laser:0` SCANANGLE (vendor per-beam bearings).
    ScanAngle(LaserScanAngle),
    /// `position2d:0` STATE.
    Position2d(Position2dState),
    /// `bumper:0` STATE.
    Bumper(BumperState),
    /// `ir:N` RANGES for index N.
    Ir(u16, IrState),
    /// `sonar:0` RANGES.
    Sonar(SonarState),
    /// `power:0` STATE.
    Power(PowerState),
    /// `position3d:N` STATE for index N.
    Position3d(u16, Position3dState),
}

/// Decode one DATA message into its typed payload state. Non-DATA messages
/// and unconsumed addresses return `None`; malformed frames return an error
/// after the codecs' size checks have rejected any hostile count words.
pub fn decode_data(msg: &Message) -> Result<Option<PlayerData>, Error> {
    if msg.header.msg_type != MSGTYPE_DATA {
        return Ok(None);
    }
    let a = msg.header.addr;
    let p = &msg.payload;
    Ok(Some(match (a.interf, a.index, msg.header.subtype) {
        (LASER_CODE, 0, LASER_DATA_SCANANGLE) => PlayerData::ScanAngle(decode_laser_scanangle(p)?),
        (POSITION2D_CODE, 0, POSITION2D_DATA_STATE) => {
            PlayerData::Position2d(decode_position2d(p)?)
        }
        (BUMPER_CODE, 0, BUMPER_DATA_STATE) => PlayerData::Bumper(decode_bumper(p)?),
        (IR_CODE, index, IR_DATA_RANGES) => PlayerData::Ir(index, decode_ir(p)?),
        (SONAR_CODE, _, SONAR_DATA_RANGES) => PlayerData::Sonar(decode_sonar(p)?),
        (POWER_CODE, _, POWER_DATA_STATE) => PlayerData::Power(decode_power(p)?),
        (POSITION3D_CODE, index, POSITION3D_DATA_STATE) => {
            PlayerData::Position3d(index, decode_position3d(p)?)
        }
        _ => return Ok(None),
    }))
}

// --- vendor motor command (interface 170) ---
// Ground truth: installed-firmware XDR table + RubyChassisDriver::ProcessMessage
// (docs/player-platform-survey.md, 2026-10-03): motor:0 fan, motor:1 main brush,
// motor:2 side brush; CMD subtype 1 = player_motor_cmd_pwm_pack, one XDR float.
// The driver scales by 100 internally; clients publish the raw duty value.
// Subscriptions are never made: the vendor enqueues a zero on the last
// unsubscribe and the queue is not reliably cancellable.
pub const MOTOR_CODE: u16 = 170;
pub const MOTOR_CMD_PWM: u8 = 1;
pub const MOTOR_MAX_PWM: f32 = 100.0;

/// Clamp a requested duty to the safe [0, 100] range; negative or non-finite
/// requests become zero, mirroring the drive path's non-finite handling.
pub fn motor_pwm_clamp(value: f64) -> f32 {
    if value.is_finite() && value > 0.0 {
        value.min(MOTOR_MAX_PWM as f64) as f32
    } else {
        0.0
    }
}

#[cfg(test)]
mod motor_tests {
    use super::*;

    // BUG/FEATURE: motor PWM command wire format for interface 170.
    // SPEC: one XDR float, MSGTYPE_CMD subtype 1; duty clamped to 0..=100,
    // negative/NaN -> 0; never a subscribe (vendor zero-on-unsubscribe side effect).
    #[test]
    fn motor_pwm_command_encodes_one_clamped_xdr_float() {
        assert_eq!(motor_pwm_clamp(50.0), 50.0);
        assert_eq!(motor_pwm_clamp(120.0), 100.0);
        assert_eq!(motor_pwm_clamp(-5.0), 0.0);
        assert_eq!(motor_pwm_clamp(f64::NAN), 0.0);
        assert_eq!(motor_pwm_clamp(f64::INFINITY), 0.0);
    }
}

/// One vendor motor PWM command on the existing connection (no subscribe).
/// `index`: 0 fan, 1 main brush, 2 side brush; duty already clamped.
pub fn motor_pwm_packet(index: u8, duty: f32) -> Vec<u8> {
    let h = MsgHeader {
        addr: DevAddr {
            host: 0,
            robot: 0,
            interf: MOTOR_CODE,
            index: index as u16,
        },
        msg_type: 2, // PLAYER_MSGTYPE_CMD
        subtype: MOTOR_CMD_PWM,
        timestamp: 0.,
        seq: 0,
        size: 4,
    };
    let mut out = encode_header(&h).to_vec();
    out.extend_from_slice(&duty.to_be_bytes());
    out
}
