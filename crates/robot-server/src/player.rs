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
pub type Writer = Arc<AsyncMutex<OwnedWriteHalf>>;
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
    let (header, payload) = velocity_packet(0, v, w, 1);
    let mut wr = writer.lock().await;
    wr.write_all(&encode_header(&header)).await?;
    wr.write_all(&payload).await?;
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
pub struct DataProcessor {
    telemetry: telemetry::Telemetry,
    odom_history: crate::slam::OdomHistory,
    recent_stamps: Vec<f64>,
    scan_sequence: u64,
    states: watch::Sender<crate::state::State>,
    scans: watch::Sender<Option<Value>>,
    safety: Arc<Mutex<Safety>>,
    slam: Option<crate::slam::ObservationSender>,
}
impl DataProcessor {
    pub fn new(
        states: watch::Sender<crate::state::State>,
        scans: watch::Sender<Option<Value>>,
        safety: Arc<Mutex<Safety>>,
        slam: Option<crate::slam::ObservationSender>,
    ) -> Self {
        Self {
            telemetry: telemetry::Telemetry::new(),
            odom_history: crate::slam::OdomHistory::new(),
            recent_stamps: Vec::new(),
            scan_sequence: 0,
            states,
            scans,
            safety,
            slam,
        }
    }
    /// Decode each DATA message exactly once (issue #51) and hand the typed
    /// result to telemetry, odometry history, safety and SLAM.
    fn process(&mut self, m: &Message, received: Instant) -> Result<(), Failure> {
        let decoded = decode_data(m)?;
        let stamp = m.header.timestamp;
        if let Some(data) = &decoded {
            match data {
                PlayerData::ScanAngle(scan) => {
                    self.scan_sequence += 1;
                    if let Some(mut v) = telemetry::scan_value(scan, stamp)? {
                        v["sequence"] = serde_json::json!(self.scan_sequence);
                        self.scans.send_replace(Some(v));
                    }
                    if let Some(tx) = &self.slam {
                        let verified = crate::slam::source_time_verified();
                        let stamp = (stamp.is_finite() && stamp > 0.).then_some(stamp);
                        let association = self.odom_history.associate(
                            stamp,
                            received,
                            verified,
                            crate::slam::odom_max_skew_ms(),
                        );
                        let mut obs = crate::slam::Observation::from_scan(scan, association.pose);
                        obs.scan_stamp = stamp;
                        obs.sequence = self.scan_sequence;
                        obs.received = received;
                        obs.association = association.kind;
                        obs.odom_skew_ms = association.skew;
                        obs.source_time_verified = verified;
                        if verified
                            && stamp.is_some()
                            && crate::slam::beam_time_verified()
                            && crate::slam::deskew_enabled()
                        {
                            // Verified slot timing (PLAYER_TIMING.md): apply the
                            // measured-period deskew; any gap keeps the raw scan.
                            if let Ok(result) = crate::slam::apply_deskew(
                                &obs,
                                &self.odom_history,
                                &self.recent_stamps,
                            ) {
                                obs.deskewed =
                                    Some((result.angles, result.ranges, result.period_ms));
                            }
                        }
                        if let Some(stamp) = stamp {
                            self.recent_stamps.push(stamp);
                            if self.recent_stamps.len() > 8 {
                                self.recent_stamps.remove(0);
                            }
                        }
                        let _ = tx.send(obs);
                    }
                }
                PlayerData::Bumper(v) => {
                    self.safety
                        .lock()
                        .unwrap()
                        .bumper(v.pressed.iter().any(|v| *v), received);
                }
                PlayerData::Position2d(v) => {
                    self.odom_history
                        .push(stamp, received, (v.x as f64, v.y as f64, v.yaw as f64));
                }
                _ => {}
            }
        }
        if let Some(data) = &decoded {
            if self.telemetry.update(data, stamp)? {
                // Match reset's lock order so telemetry cannot publish an old
                // block verdict. The typed state replaces only the telemetry
                // section (issue #50); slam/compass/motors keep their producers.
                let blocked = self.safety.lock().unwrap().is_blocked();
                self.states.send_if_modified(|state| {
                    state.publish_player(self.telemetry.clone(), blocked)
                });
            }
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
            watch::channel(crate::state::State::initial()).0,
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

    // DEDUP (issue #51): the async drive path must emit byte-exactly the
    // shared player-bridge velocity encoder; a local re-encode would drift
    // back into two divergent wire forms.
    #[tokio::test]
    async fn velocity_wires_the_shared_encoder_bytes_exactly() {
        let listener = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
        let port = listener.local_addr().unwrap().port();
        let fake = tokio::spawn(async move {
            let (mut stream, _) = listener.accept().await.unwrap();
            let mut wire = vec![0; 68];
            stream.read_exact(&mut wire).await.unwrap();
            wire
        });
        let socket = TcpStream::connect(("127.0.0.1", port)).await.unwrap();
        let (_, wr) = socket.into_split();
        let mut writer: Writer = Arc::new(AsyncMutex::new(wr));
        velocity(&mut writer, -0.06, 2.5).await.unwrap();
        let wire = tokio::time::timeout(Duration::from_secs(2), fake)
            .await
            .unwrap()
            .unwrap();
        let (header, payload) = velocity_packet(0, -0.06, 2.5, 1);
        let expected = [encode_header(&header).as_slice(), &payload].concat();
        assert_eq!(wire, expected);
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
        let (states, mut state) = watch::channel(crate::state::State::initial());
        let (scans, _scan) = watch::channel(None);
        let processor =
            DataProcessor::new(states, scans, Arc::new(Mutex::new(Safety::new())), None);
        let connecting = connect(&config, None, processor);
        tokio::pin!(connecting);
        tokio::select! {
            result = &mut connecting => panic!("startup finished before publishing pre-ACK odometry: {}", result.is_ok()),
            update = state.wait_for(|value| value.to_json()["odom"]["x"] == 1.) => { drop(update.unwrap()); }
        }
        let (reader, _) = connecting.await.unwrap();
        assert_eq!(state.borrow().to_json()["odom"]["x"], 1.);
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

// Cleaning motors publish PWM commands on the same connection without any
// subscription (vendor enqueues a zero on last unsubscribe). Zero duties are
// still sent explicitly: teardown must not rely on the vendor queue.
pub async fn motor_pwm(writer: &Writer, index: u8, duty: f32) -> Result<(), Failure> {
    let mut wr = writer.lock().await;
    wr.write_all(&motor_pwm_packet(index, duty)).await?;
    Ok(())
}
