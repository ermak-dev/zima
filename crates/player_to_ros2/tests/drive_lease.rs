//! Integration test: the bridge node's drive safety, end to end, against
//! a fake Player server that records every velocity command it receives.
//! One over-limit /cmd_vel must reach Player clamped to the project
//! limits, and once the command lease lapses with no new /cmd_vel the
//! wheels must get a zero (Player itself never expires a velocity).
//!
//! Needs a sourced ROS 2 environment (run with the ros-ws overlay); it is
//! skipped otherwise. The publisher runs in a private ROS domain limited
//! to localhost, so the test /cmd_vel can never reach a live session.

use std::io::{Read, Write};
use std::net::{TcpListener, TcpStream};
use std::process::Command;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use player_bridge::{
    decode_header, encode_header, DevAddr, MsgHeader, MSGHDR_SIZE, MSGTYPE_DATA, MSGTYPE_REQ,
    MSGTYPE_RESP_ACK, POSITION2D_CODE, POSITION2D_DATA_STATE,
};

const MSGTYPE_CMD: u8 = 2;
const POSITION2D_CMD_VEL: u8 = 1;

type Log = Arc<Mutex<Vec<(Instant, f64, f64)>>>;

fn send(out: &Mutex<TcpStream>, header: MsgHeader, payload: &[u8]) -> std::io::Result<()> {
    let mut s = out.lock().unwrap();
    s.write_all(&encode_header(&header))?;
    s.write_all(payload)
}

/// Serve one bridge connection: banner, ACK every subscription, stream
/// position2d data at 20 Hz, and record position2d velocity commands.
fn fake_player(listener: TcpListener, log: Log, stop: Arc<AtomicBool>) {
    let (stream, _) = listener.accept().expect("bridge connects");
    let mut banner = [0u8; 32];
    banner[..14].copy_from_slice(b"Player v.3.1.0");
    let out = Arc::new(Mutex::new(stream.try_clone().unwrap()));
    out.lock().unwrap().write_all(&banner).unwrap();

    let data_out = out.clone();
    let data_stop = stop.clone();
    std::thread::spawn(move || {
        while !data_stop.load(Ordering::Relaxed) {
            let header = MsgHeader {
                addr: DevAddr { host: 0, robot: 0, interf: POSITION2D_CODE, index: 0 },
                msg_type: MSGTYPE_DATA,
                subtype: POSITION2D_DATA_STATE,
                timestamp: 0.0,
                seq: 0,
                size: 52,
            };
            if send(&data_out, header, &[0u8; 52]).is_err() {
                return;
            }
            std::thread::sleep(Duration::from_millis(50));
        }
    });

    let mut input = stream;
    input.set_read_timeout(Some(Duration::from_millis(200))).unwrap();
    while !stop.load(Ordering::Relaxed) {
        let mut hdr = [0u8; MSGHDR_SIZE];
        match input.read_exact(&mut hdr) {
            Ok(()) => {}
            Err(e) if e.kind() == std::io::ErrorKind::WouldBlock
                || e.kind() == std::io::ErrorKind::TimedOut => continue,
            Err(_) => return,
        }
        let header = decode_header(&hdr).expect("valid header");
        let mut payload = vec![0u8; header.size as usize];
        input.set_read_timeout(None).unwrap();
        input.read_exact(&mut payload).unwrap();
        input.set_read_timeout(Some(Duration::from_millis(200))).unwrap();
        match (header.msg_type, header.addr.interf, header.subtype) {
            (MSGTYPE_REQ, _, _) => {
                let mut ack = [0u8; 28];
                ack[19] = 1; // granted access
                let reply = MsgHeader { msg_type: MSGTYPE_RESP_ACK, size: 28, ..header };
                send(&out, reply, &ack).unwrap();
            }
            (MSGTYPE_CMD, POSITION2D_CODE, POSITION2D_CMD_VEL) => {
                let f = |o: usize| f64::from_be_bytes(payload[o..o + 8].try_into().unwrap());
                log.lock().unwrap().push((Instant::now(), f(0), f(16)));
            }
            _ => {}
        }
    }
}

#[test]
fn over_limit_command_is_clamped_and_lapses_to_zero() {
    if std::env::var("ROS_DISTRO").is_err() {
        eprintln!("skipping: no sourced ROS 2 environment");
        return;
    }
    let listener = TcpListener::bind("127.0.0.1:0").unwrap();
    let port = listener.local_addr().unwrap().port();
    let log: Log = Arc::new(Mutex::new(Vec::new()));
    let stop = Arc::new(AtomicBool::new(false));
    let server = {
        let (log, stop) = (log.clone(), stop.clone());
        std::thread::spawn(move || fake_player(listener, log, stop))
    };

    // A private domain on localhost: this test's /cmd_vel stays here.
    let domain = (100 + std::process::id() % 100).to_string();
    let mut bridge = Command::new(env!("CARGO_BIN_EXE_player_to_ros2"))
        .args(["--ros-args", "-p", "player_host:=127.0.0.1", "-p"])
        .arg(format!("player_port:={port}"))
        .args(["-p", "cmd_vel_timeout_s:=0.5"])
        .env("ROS_DOMAIN_ID", &domain)
        .env("ROS_AUTOMATIC_DISCOVERY_RANGE", "LOCALHOST")
        .stdout(std::process::Stdio::null())
        .stderr(std::process::Stdio::null())
        .spawn()
        .expect("spawn bridge");

    // One command, far over the limits, then silence.
    let published = Command::new("ros2")
        .args(["topic", "pub", "--once", "-w", "1", "/cmd_vel", "geometry_msgs/msg/Twist"])
        .arg("{linear: {x: 0.5}, angular: {z: -1.0}}")
        .env("ROS_DOMAIN_ID", &domain)
        .env("ROS_AUTOMATIC_DISCOVERY_RANGE", "LOCALHOST")
        .stdout(std::process::Stdio::null())
        .status()
        .expect("run ros2 topic pub");
    std::thread::sleep(Duration::from_millis(2000));

    stop.store(true, Ordering::Relaxed);
    let _ = bridge.kill();
    let _ = bridge.wait();
    let _ = server.join();
    assert!(published.success(), "ros2 topic pub failed");

    let log = log.lock().unwrap().clone();
    let first = log.iter().position(|&(_, vx, _)| vx != 0.0).expect("a motion command reached Player");
    let (t_move, vx, vyaw) = log[first];
    assert_eq!((vx, vyaw), (0.1, -0.4), "command not clamped to the project limits: {log:?}");
    let stop_cmd = log[first + 1..]
        .iter()
        .find(|&&(_, vx, vyaw)| vx == 0.0 && vyaw == 0.0)
        .expect("lease lapsed but the wheels never got a zero");
    let after = stop_cmd.0.duration_since(t_move);
    assert!(
        after >= Duration::from_millis(400) && after <= Duration::from_millis(900),
        "zero came {after:?} after the command (lease 0.5 s)"
    );
    assert!(
        log[first + 1..].iter().all(|&(_, vx, vyaw)| vx == 0.0 && vyaw == 0.0),
        "motion resumed without a new command: {log:?}"
    );
}
