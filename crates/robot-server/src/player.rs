//! Async framing lives independently of the drive clock. read_exact is never
//! cancelled by a timer tick (which would lose partial headers/payloads).
use crate::{safety::Safety, telemetry, Config, Failure};
use player_bridge::*;
use serde_json::Value;
use std::{
    sync::{mpsc, Arc, Mutex},
    time::{Duration, Instant},
};
use tokio::{
    io::{AsyncRead, AsyncReadExt, AsyncWriteExt},
    net::{
        tcp::{OwnedReadHalf, OwnedWriteHalf},
        TcpStream,
    },
    sync::{watch, Mutex as AsyncMutex},
};
type Writer = Arc<AsyncMutex<OwnedWriteHalf>>;
pub struct Reader {
    stream: OwnedReadHalf,
    pending: Vec<Message>,
}
const LIMIT: usize = 1024 * 1024;
async fn message<R: AsyncRead + Unpin>(rd: &mut R) -> Result<Message, Failure> {
    let mut h = [0; MSGHDR_SIZE];
    rd.read_exact(&mut h).await?;
    let header = decode_header(&h).ok_or("invalid Player header")?;
    if header.size as usize > LIMIT {
        return Err("Player frame exceeds size limit".into());
    }
    let mut payload = vec![0; header.size as usize];
    rd.read_exact(&mut payload).await?;
    Ok(Message { header, payload })
}
async fn write(stream: &mut TcpStream, h: MsgHeader, p: &[u8]) -> Result<(), Failure> {
    stream.write_all(&encode_header(&h)).await?;
    stream.write_all(p).await?;
    Ok(())
}
pub async fn connect(c: &Config) -> Result<(Reader, Writer), Failure> {
    tokio::time::timeout(Duration::from_secs(10), async {
        let mut waiting = false;
        let mut stream = loop {
            match TcpStream::connect((c.player_host.as_str(), c.player_port)).await {
                Ok(stream) => break stream,
                Err(e) if e.kind() == std::io::ErrorKind::ConnectionRefused => {
                    if !waiting {
                        eprintln!(
                            "robot-server: waiting for Player at {}:{} (startup timeout 10 s)",
                            c.player_host, c.player_port
                        );
                        waiting = true;
                    }
                    tokio::time::sleep(Duration::from_millis(200)).await;
                }
                Err(e) => return Err(e.into()),
            }
        };
        stream.set_nodelay(true)?;
        let mut banner = [0; 32];
        stream.read_exact(&mut banner).await?;
        if &banner[..9] != b"Player v." {
            return Err("bad Player banner".into());
        }
        let mut pending = Vec::new();
        for (interf, index) in [
            (LASER_CODE, 0),
            (POSITION2D_CODE, 0),
            (BUMPER_CODE, 0),
            (IR_CODE, 0),
            (IR_CODE, 1),
            (IR_CODE, 2),
            (SONAR_CODE, 0),
            (POWER_CODE, 0),
        ] {
            let mut p = [0; 28];
            p[8..12].copy_from_slice(&(interf as u32).to_be_bytes());
            p[12..16].copy_from_slice(&(index as u32).to_be_bytes());
            p[19] = OPEN_MODE;
            let h = MsgHeader {
                addr: DevAddr {
                    host: 0,
                    robot: 0,
                    interf: PLAYER_CODE,
                    index: 0,
                },
                msg_type: MSGTYPE_REQ,
                subtype: REQ_DEV,
                timestamp: 0.,
                seq: 0,
                size: 28,
            };
            write(&mut stream, h, &p).await?;
            loop {
                let m = message(&mut stream).await?;
                if m.header.msg_type == MSGTYPE_DATA {
                    if pending.len() >= 128 {
                        return Err("too much DATA during Player subscriptions".into());
                    }
                    pending.push(m);
                    continue;
                }
                if m.header.msg_type == MSGTYPE_RESP_NACK
                    && interf != LASER_CODE
                    && interf != POSITION2D_CODE
                {
                    break;
                }
                if m.header.msg_type != MSGTYPE_RESP_ACK
                    || m.header.subtype != REQ_DEV
                    || m.payload.get(19) != Some(&OPEN_MODE)
                {
                    return Err("Player subscription rejected".into());
                }
                break;
            }
        }
        let (stream, wr) = stream.into_split();
        Ok((Reader { stream, pending }, Arc::new(AsyncMutex::new(wr))))
    })
    .await
    .map_err(|_| -> Failure { "Player startup timed out after 10 s".into() })?
}
pub async fn velocity(writer: &mut Writer, v: f64, w: f64) -> Result<(), Failure> {
    let mut p = [0; 28];
    p[..8].copy_from_slice(&v.to_be_bytes());
    p[16..24].copy_from_slice(&w.to_be_bytes());
    p[24] = 1;
    let h = MsgHeader {
        addr: DevAddr {
            host: 0,
            robot: 0,
            interf: POSITION2D_CODE,
            index: 0,
        },
        msg_type: 2,
        subtype: 1,
        timestamp: 0.,
        seq: 0,
        size: 28,
    };
    let mut wr = writer.lock().await;
    wr.write_all(&encode_header(&h)).await?;
    wr.write_all(&p).await?;
    Ok(())
}
pub async fn drive(mut writer: Writer, safety: Arc<Mutex<Safety>>) -> Result<(), Failure> {
    let mut ticks = tokio::time::interval(Duration::from_millis(20));
    ticks.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
    loop {
        ticks.tick().await;
        let next = safety.lock().unwrap().next(Instant::now());
        if let Some((v, w)) = next {
            tokio::time::timeout(Duration::from_millis(250), velocity(&mut writer, v, w)).await??;
        }
    }
}
// Guard decoder allocations before calling the shared wire codecs. Their count
// words are untrusted; a tiny malformed payload must not reserve gigabytes.
fn array(p: &[u8], offset: &mut usize, width: usize) -> Result<(), Failure> {
    let word = |o| {
        p.get(o..o + 4)
            .map(|b| u32::from_be_bytes(b.try_into().unwrap()) as usize)
            .ok_or("truncated Player array")
    };
    let n = word(*offset)?;
    if word(*offset + 4)? != n {
        return Err("mismatched Player array counts".into());
    }
    let bytes = n.checked_mul(width).ok_or("Player array overflow")?;
    let end = offset
        .checked_add(8)
        .and_then(|o| o.checked_add(bytes))
        .ok_or("Player array overflow")?;
    if end > p.len() {
        return Err("truncated Player array".into());
    }
    *offset = end.div_ceil(4) * 4;
    Ok(())
}
fn validate(m: &Message) -> Result<(), Failure> {
    if m.header.msg_type != MSGTYPE_DATA {
        return Ok(());
    }
    let mut off = 0;
    match (m.header.addr.interf, m.header.subtype) {
        (LASER_CODE, LASER_DATA_SCANANGLE) => {
            off = 4;
            array(&m.payload, &mut off, 4)?;
            array(&m.payload, &mut off, 4)?;
            array(&m.payload, &mut off, 1)?;
        }
        (IR_CODE, IR_DATA_RANGES) => {
            array(&m.payload, &mut off, 4)?;
            array(&m.payload, &mut off, 4)?;
        }
        (SONAR_CODE, SONAR_DATA_RANGES) => array(&m.payload, &mut off, 4)?,
        (BUMPER_CODE, BUMPER_DATA_STATE) => array(&m.payload, &mut off, 1)?,
        _ => {}
    }
    Ok(())
}
pub async fn read(
    mut reader: Reader,
    states: watch::Sender<Value>,
    scans: watch::Sender<Option<Value>>,
    safety: Arc<Mutex<Safety>>,
    slam: Option<mpsc::Sender<crate::slam::Observation>>,
) -> Result<(), Failure> {
    let mut telemetry = telemetry::Telemetry::new();
    let mut odom_pose: Option<(f64, f64, f64)> = None;
    let mut pending = std::mem::take(&mut reader.pending).into_iter();
    loop {
        let m = if let Some(m) = pending.next() {
            m
        } else {
            tokio::time::timeout(Duration::from_secs(5), message(&mut reader.stream))
                .await
                .map_err(|_| "Player data timed out")??
        };
        validate(&m)?;
        if m.header.msg_type != MSGTYPE_DATA {
            continue;
        }
        match (m.header.addr.interf, m.header.addr.index, m.header.subtype) {
            (LASER_CODE, 0, LASER_DATA_SCANANGLE) => {
                if let Some(v) = telemetry::scan(&m)? {
                    scans.send_replace(Some(v));
                }
                if let Some(tx) = &slam {
                    let scan = decode_laser_scanangle(&m.payload)?;
                    let _ = tx.send(crate::slam::Observation::from_scan(&scan, odom_pose));
                }
            }
            (BUMPER_CODE, 0, BUMPER_DATA_STATE) => {
                let v = decode_bumper(&m.payload)?;
                safety
                    .lock()
                    .unwrap()
                    .bumper(v.pressed.iter().any(|v| *v), Instant::now());
            }
            (POSITION2D_CODE, 0, POSITION2D_DATA_STATE) => {
                let v = decode_position2d(&m.payload)?;
                safety.lock().unwrap().odom(v.vx, v.vyaw, Instant::now());
                odom_pose = Some((v.x as f64, v.y as f64, v.yaw as f64));
            }
            _ => {}
        }
        if telemetry.update(&m)? {
            let v = telemetry.json();
            states.send_if_modified(|old| {
                // Merge, not replace: the slam thread writes its own fields
                // into this shared state and must survive telemetry publishes.
                let mut next = v.clone();
                if let (Some(dst), Some(src)) = (next.as_object_mut(), old.as_object()) {
                    for (k, val) in src {
                        if k == "slam" {
                            dst.insert(k.clone(), val.clone());
                        }
                    }
                }
                if *old == next {
                    false
                } else {
                    *old = next;
                    true
                }
            });
        }
    }
}
