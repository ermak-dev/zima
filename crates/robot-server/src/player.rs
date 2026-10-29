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
    processor: DataProcessor,
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
    mut processor: DataProcessor,
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
                    processor.process(&m, received)?;
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
                processor,
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
pub struct DataProcessor {
    telemetry: telemetry::Telemetry,
    odom_history: crate::slam::OdomHistory,
    scan_sequence: u64,
    states: watch::Sender<Value>,
    scans: watch::Sender<Option<Value>>,
    safety: Arc<Mutex<Safety>>,
    slam: Option<crate::slam::ObservationSender>,
}
impl DataProcessor {
    pub fn new(
        states: watch::Sender<Value>,
        scans: watch::Sender<Option<Value>>,
        safety: Arc<Mutex<Safety>>,
        slam: Option<crate::slam::ObservationSender>,
    ) -> Self {
        Self {
            telemetry: telemetry::Telemetry::new(),
            odom_history: crate::slam::OdomHistory::new(),
            scan_sequence: 0,
            states,
            scans,
            safety,
            slam,
        }
    }
    fn process(&mut self, m: &Message, received: Instant) -> Result<(), Failure> {
        validate(m)?;
        if m.header.msg_type != MSGTYPE_DATA {
            return Ok(());
        }
        match (m.header.addr.interf, m.header.addr.index, m.header.subtype) {
            (LASER_CODE, 0, LASER_DATA_SCANANGLE) => {
                self.scan_sequence += 1;
                if let Some(mut v) = telemetry::scan(m)? {
                    v["sequence"] = serde_json::json!(self.scan_sequence);
                    self.scans.send_replace(Some(v));
                }
                if let Some(tx) = &self.slam {
                    let scan = decode_laser_scanangle(&m.payload)?;
                    let verified = crate::slam::source_time_verified();
                    let stamp = (m.header.timestamp.is_finite() && m.header.timestamp > 0.)
                        .then_some(m.header.timestamp);
                    let association = self.odom_history.associate(
                        stamp,
                        received,
                        verified,
                        crate::slam::odom_max_skew_ms(),
                    );
                    let mut obs = crate::slam::Observation::from_scan(&scan, association.pose);
                    obs.scan_stamp = stamp;
                    obs.sequence = self.scan_sequence;
                    obs.received = received;
                    obs.association = association.kind;
                    obs.odom_skew_ms = association.skew;
                    obs.source_time_verified = verified;
                    let _ = tx.send(obs);
                }
            }
            (BUMPER_CODE, 0, BUMPER_DATA_STATE) => {
                let v = decode_bumper(&m.payload)?;
                self.safety
                    .lock()
                    .unwrap()
                    .bumper(v.pressed.iter().any(|v| *v), received);
            }
            (POSITION2D_CODE, 0, POSITION2D_DATA_STATE) => {
                let v = decode_position2d(&m.payload)?;
                self.odom_history.push(
                    m.header.timestamp,
                    received,
                    (v.x as f64, v.y as f64, v.yaw as f64),
                );
            }
            _ => {}
        }
        if self.telemetry.update(m)? {
            let mut v = self.telemetry.json();
            // Match reset's lock order so telemetry cannot publish an old block verdict.
            let safety = self.safety.lock().unwrap();
            v["blocked"] = serde_json::json!(safety.is_blocked());
            self.states.send_if_modified(|old| {
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
        Ok(())
    }
}
pub async fn read(mut reader: Reader) -> Result<(), Failure> {
    loop {
        let m = tokio::time::timeout(Duration::from_secs(5), message(&mut reader.stream))
            .await
            .map_err(|_| "Player data timed out")??;
        let received = Instant::now();
        if let Some(recorder) = &reader.recorder {
            recorder.record(&m, received);
        }
        reader.processor.process(&m, received)?;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn config(port: u16) -> Config {
        Config {
            player_host: "127.0.0.1".into(),
            player_port: port,
            listen: "127.0.0.1".into(),
            port: 0,
            camera_url: String::new(),
            netrc: std::path::PathBuf::new(),
            slam_adapter: None,
            yolo_adapter: None,
            audio_only: false,
        }
    }
    fn processor() -> DataProcessor {
        DataProcessor::new(
            watch::channel(serde_json::json!({})).0,
            watch::channel(None).0,
            Arc::new(Mutex::new(Safety::new())),
            None,
        )
    }
    async fn accept_subscription(listener: tokio::net::TcpListener) -> TcpStream {
        let (mut stream, _) = listener.accept().await.unwrap();
        let mut banner = [0; 32];
        banner[..9].copy_from_slice(b"Player v.");
        stream.write_all(&banner).await.unwrap();
        message(&mut stream).await.unwrap();
        stream
    }

    #[tokio::test]
    async fn startup_stream_without_ack_keeps_the_total_deadline() {
        let listener = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
        let config = config(listener.local_addr().unwrap().port());
        let fake = tokio::spawn(async move {
            let mut stream = accept_subscription(listener).await;
            let payload = [0, 0, 0, 3, 0, 0, 0, 3, 0, 0, 0, 0];
            let header = MsgHeader {
                addr: DevAddr {
                    host: 0,
                    robot: 0,
                    interf: BUMPER_CODE,
                    index: 0,
                },
                msg_type: MSGTYPE_DATA,
                subtype: BUMPER_DATA_STATE,
                timestamp: 42.,
                seq: 0,
                size: payload.len() as u32,
            };
            loop {
                if stream.write_all(&encode_header(&header)).await.is_err()
                    || stream.write_all(&payload).await.is_err()
                {
                    break;
                }
                tokio::time::sleep(Duration::from_millis(5)).await;
            }
        });
        let started = Instant::now();
        let error = connect(&config, None, processor())
            .await
            .err()
            .expect("missing ACK must fail startup");
        assert_eq!(error.to_string(), "Player startup timed out after 10 s");
        assert!(started.elapsed() < Duration::from_secs(13));
        tokio::time::timeout(Duration::from_secs(1), fake)
            .await
            .unwrap()
            .unwrap();
    }

    #[tokio::test]
    async fn oversized_data_during_subscription_is_rejected_before_payload_allocation() {
        let listener = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
        let config = config(listener.local_addr().unwrap().port());
        let fake = tokio::spawn(async move {
            let mut stream = accept_subscription(listener).await;
            let header = MsgHeader {
                addr: DevAddr {
                    host: 0,
                    robot: 0,
                    interf: LASER_CODE,
                    index: 0,
                },
                msg_type: MSGTYPE_DATA,
                subtype: LASER_DATA_SCANANGLE,
                timestamp: 42.,
                seq: 0,
                size: LIMIT as u32 + 1,
            };
            stream.write_all(&encode_header(&header)).await.unwrap();
            stream
        });
        let error = connect(&config, None, processor())
            .await
            .err()
            .expect("oversize header must be rejected");
        assert_eq!(error.to_string(), "Player frame exceeds size limit");
        drop(fake.await.unwrap());
    }
    // FEATURE: retain monotonic receive clocks for subscription-time DATA.
    // SPEC: waiting for ACK must not refresh the age of an already received measurement.
    // IMPLEMENTATION: process startup odometry at receipt with the normal bounded history.
    // RESULT: IMPLEMENTED — delayed subscription fake leaves old odometry stale.
    // STATUS: DONE
    #[tokio::test]
    async fn subscription_odometry_retains_actual_receive_age() {
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
                    let mut payload = Vec::new();
                    for value in [1f64, 2., 0.3, 0., 0., 0.] {
                        payload.extend(value.to_be_bytes());
                    }
                    payload.extend(0u32.to_be_bytes());
                    let data = MsgHeader {
                        addr: DevAddr {
                            interf: POSITION2D_CODE,
                            index: 0,
                            ..m.header.addr
                        },
                        msg_type: MSGTYPE_DATA,
                        subtype: POSITION2D_DATA_STATE,
                        timestamp: 42.,
                        size: payload.len() as u32,
                        ..m.header
                    };
                    stream.write_all(&encode_header(&data)).await.unwrap();
                    stream.write_all(&payload).await.unwrap();
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
        let config = config(port);
        let (states, mut state) = watch::channel(serde_json::json!({}));
        let (scans, _scan) = watch::channel(None);
        let processor =
            DataProcessor::new(states, scans, Arc::new(Mutex::new(Safety::new())), None);
        let connecting = connect(&config, None, processor);
        tokio::pin!(connecting);
        tokio::select! {
            result = &mut connecting => panic!("startup finished before publishing pre-ACK odometry: {}", result.is_ok()),
            update = state.wait_for(|value| value["odom"]["x"] == 1.) => { drop(update.unwrap()); }
        }
        let (reader, _) = connecting.await.unwrap();
        assert_eq!(state.borrow()["odom"]["x"], 1.);
        assert!(
            reader
                .processor
                .odom_history
                .associate(None, Instant::now(), false, 100.)
                .pose
                .is_none(),
            "an old subscription-time sample must not become fresh when the final ACK arrives"
        );
        drop(fake.await.unwrap());
    }
}
