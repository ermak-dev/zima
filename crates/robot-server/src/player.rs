//! Async framing lives independently of the drive clock. read_exact is never
//! cancelled by a timer tick (which would lose partial headers/payloads).
use crate::{safety::Safety, telemetry, Config, Failure};
use player_bridge::*;
use robot_server::player_recording::Recorder;
use serde_json::Value;
use std::{
    sync::{Arc, Mutex},
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
    pending: Vec<(Message, Instant)>,
    recorder: Option<Arc<Recorder>>,
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
pub async fn connect(
    c: &Config,
    recorder: Option<Arc<Recorder>>,
) -> Result<(Reader, Writer), Failure> {
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
            (POSITION3D_CODE, 2),
            (IR_CODE, 3),
            (IR_CODE, 4),
            (IR_CODE, 6),
            (IR_CODE, 7),
            (IR_CODE, 8),
            (POSITION3D_CODE, 0),
            (POSITION3D_CODE, 1),
            (POSITION3D_CODE, 4),
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
                let received = Instant::now();
                if let Some(recorder) = &recorder {
                    recorder.record(&m, received);
                }
                if m.header.msg_type == MSGTYPE_DATA {
                    if pending.len() >= 128 {
                        return Err("too much DATA during Player subscriptions".into());
                    }
                    pending.push((m, received));
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
        Ok((
            Reader {
                stream,
                pending,
                recorder,
            },
            Arc::new(AsyncMutex::new(wr)),
        ))
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
    slam: Option<crate::slam::ObservationSender>,
) -> Result<(), Failure> {
    let mut telemetry = telemetry::Telemetry::new();
    let mut odom_history = crate::slam::OdomHistory::new();
    let mut scan_sequence = 0u64;
    let mut pending = std::mem::take(&mut reader.pending).into_iter();
    loop {
        let (m, received) = if let Some(m) = pending.next() {
            m
        } else {
            let m = tokio::time::timeout(Duration::from_secs(5), message(&mut reader.stream))
                .await
                .map_err(|_| "Player data timed out")??;
            let received = Instant::now();
            if let Some(recorder) = &reader.recorder {
                recorder.record(&m, received);
            }
            (m, received)
        };
        validate(&m)?;
        if m.header.msg_type != MSGTYPE_DATA {
            continue;
        }
        match (m.header.addr.interf, m.header.addr.index, m.header.subtype) {
            (LASER_CODE, 0, LASER_DATA_SCANANGLE) => {
                scan_sequence += 1;
                if let Some(mut v) = telemetry::scan(&m)? {
                    v["sequence"] = serde_json::json!(scan_sequence);
                    scans.send_replace(Some(v));
                }
                if let Some(tx) = &slam {
                    let scan = decode_laser_scanangle(&m.payload)?;
                    let verified = crate::slam::source_time_verified();
                    let stamp = (m.header.timestamp.is_finite() && m.header.timestamp > 0.)
                        .then_some(m.header.timestamp);
                    let association = odom_history.associate(
                        stamp,
                        received,
                        verified,
                        crate::slam::odom_max_skew_ms(),
                    );
                    let mut obs = crate::slam::Observation::from_scan(&scan, association.pose);
                    obs.scan_stamp = stamp;
                    obs.sequence = scan_sequence;
                    obs.received = received;
                    obs.association = association.kind;
                    obs.odom_skew_ms = association.skew;
                    obs.source_time_verified = verified;
                    let _ = tx.send(obs);
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
                odom_history.push(
                    m.header.timestamp,
                    received,
                    (v.x as f64, v.y as f64, v.yaw as f64),
                );
            }
            _ => {}
        }
        if telemetry.update(&m)? {
            let mut v = telemetry.json();
            // Match reset's lock order so telemetry cannot publish an old block verdict.
            let safety = safety.lock().unwrap();
            v["blocked"] = serde_json::json!(safety.is_blocked());
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
                if let Some(compass) = old.get("compass") {
                    next["compass"] = compass.clone();
                    next["sensors"]["compass"] = compass.clone();
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

#[cfg(test)]
mod tests {
    use super::*;
    // FEATURE: retain monotonic receive clocks for subscription-time DATA.
    // SPEC: buffering a DATA frame while subscribing must not refresh its age.
    // IMPLEMENTATION: retain Instant alongside each pending decoded frame.
    // RESULT: IMPLEMENTED — delayed subscription fake retains original age.
    // STATUS: DONE
    #[tokio::test]
    async fn pending_data_retains_actual_receive_instant() {
        let listener = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
        let port = listener.local_addr().unwrap().port();
        let fake = tokio::spawn(async move {
            let (mut stream, _) = listener.accept().await.unwrap();
            let mut banner = [0; 32];
            banner[..9].copy_from_slice(b"Player v.");
            stream.write_all(&banner).await.unwrap();
            for n in 0..17 {
                let m = message(&mut stream).await.unwrap();
                if n == 0 {
                    let data = MsgHeader {
                        msg_type: MSGTYPE_DATA,
                        size: 0,
                        ..m.header
                    };
                    stream.write_all(&encode_header(&data)).await.unwrap();
                    tokio::time::sleep(Duration::from_millis(120)).await;
                }
                let ack = MsgHeader {
                    msg_type: MSGTYPE_RESP_ACK,
                    ..m.header
                };
                stream.write_all(&encode_header(&ack)).await.unwrap();
                stream.write_all(&m.payload).await.unwrap();
            }
            stream
        });
        let config = Config {
            player_host: "127.0.0.1".into(),
            player_port: port,
            listen: "127.0.0.1".into(),
            port: 0,
            camera_url: String::new(),
            netrc: std::path::PathBuf::new(),
            slam_adapter: None,
            yolo_adapter: None,
            audio_only: false,
        };
        let (reader, _) = connect(&config, None).await.unwrap();
        assert!(reader.pending[0].1.elapsed() >= Duration::from_millis(100));
        drop(fake.await.unwrap());
    }
}
