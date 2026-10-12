//! Integration test: the bridge detects a blocked drive — /cmd_vel flows,
//! the wheels do not turn (the fake Player's odometry twist stays zero) —
//! and stops forwarding commands for a hold period instead of pushing the
//! wall. Skipped without a sourced ROS 2 environment.

use std::io::{Read, Write};
use std::net::{TcpListener, TcpStream};
use std::process::Command;
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use player_bridge::{
    decode_header, encode_header, DevAddr, MsgHeader, MSGHDR_SIZE, MSGTYPE_DATA, MSGTYPE_REQ,
    MSGTYPE_RESP_ACK, POSITION2D_CODE, POSITION2D_DATA_STATE,
};

const MSGTYPE_CMD: u8 = 2;
const POSITION2D_CMD_VEL: u8 = 1;

fn send(out: &Mutex<TcpStream>, header: MsgHeader, payload: &[u8]) -> std::io::Result<()> {
    let mut s = out.lock().unwrap();
    s.write_all(&encode_header(&header))?;
    s.write_all(payload)
}

/// Fake Player whose wheels are blocked: odometry twist stays zero no
/// matter what velocity arrives (as measured live against the wall).
fn fake_player(listener: TcpListener, cmds: Arc<AtomicUsize>, stop: Arc<AtomicBool>) {
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
                addr: DevAddr {
                    host: 0,
                    robot: 0,
                    interf: POSITION2D_CODE,
                    index: 0,
                },
                msg_type: MSGTYPE_DATA,
                subtype: POSITION2D_DATA_STATE,
                timestamp: 0.0,
                seq: 0,
                size: 52,
            };
            // Twist stays zero: vx/vyaw at offsets 24/40.
            if send(&data_out, header, &[0u8; 52]).is_err() {
                return;
            }
            std::thread::sleep(Duration::from_millis(50));
        }
    });

    let mut input = stream;
    input
        .set_read_timeout(Some(Duration::from_millis(200)))
        .unwrap();
    while !stop.load(Ordering::Relaxed) {
        let mut hdr = [0u8; MSGHDR_SIZE];
        match input.read_exact(&mut hdr) {
            Ok(()) => {}
            Err(e)
                if e.kind() == std::io::ErrorKind::WouldBlock
                    || e.kind() == std::io::ErrorKind::TimedOut =>
            {
                continue
            }
            Err(_) => return,
        }
        let header = decode_header(&hdr).expect("valid header");
        let mut payload = vec![0u8; header.size as usize];
        input.set_read_timeout(None).unwrap();
        input.read_exact(&mut payload).unwrap();
        input
            .set_read_timeout(Some(Duration::from_millis(200)))
            .unwrap();
        match (header.msg_type, header.addr.interf, header.subtype) {
            (MSGTYPE_REQ, _, _) => {
                let mut ack = [0u8; 28];
                ack[19] = 1;
                let reply = MsgHeader {
                    msg_type: MSGTYPE_RESP_ACK,
                    size: 28,
                    ..header
                };
                send(&out, reply, &ack).unwrap();
            }
            (MSGTYPE_CMD, POSITION2D_CODE, POSITION2D_CMD_VEL) => {
                let f = |o: usize| f64::from_be_bytes(payload[o..o + 8].try_into().unwrap());
                if f(0) != 0.0 || f(16) != 0.0 {
                    cmds.fetch_add(1, Ordering::Relaxed);
                }
            }
            _ => {}
        }
    }
}

const PUBLISHER: &str = r#"
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

rclpy.init()
n = Node('cmd_pub')
pub = n.create_publisher(Twist, '/cmd_vel', 10)
end = time.time() + 9
while time.time() < end:
    t = Twist(); t.linear.x = 0.06
    pub.publish(t)
    rclpy.spin_once(n, timeout_sec=0.05)
print('PUB DONE')
"#;

#[test]
fn stalled_drive_stops_forwarding_commands() {
    if std::env::var("ROS_DISTRO").is_err() {
        eprintln!("skipping: no sourced ROS 2 environment");
        return;
    }
    if Command::new("ros2")
        .arg("--help")
        .stdout(std::process::Stdio::null())
        .status()
        .is_err()
    {
        eprintln!("skipping: the ros2 CLI is not on PATH");
        return;
    }
    let listener = TcpListener::bind("127.0.0.1:0").unwrap();
    let port = listener.local_addr().unwrap().port();
    let stop = Arc::new(AtomicBool::new(false));
    let cmds = Arc::new(AtomicUsize::new(0));
    let server = {
        let (stop, cmds) = (stop.clone(), cmds.clone());
        std::thread::spawn(move || fake_player(listener, cmds, stop))
    };

    let domain = (100 + std::process::id() % 100).to_string();
    let log = std::env::temp_dir().join(format!("stall-{}.log", std::process::id()));
    let mut bridge = Command::new(env!("CARGO_BIN_EXE_player_to_ros2"))
        .args(["--ros-args", "-p", "player_host:=127.0.0.1", "-p"])
        .arg(format!("player_port:={port}"))
        .env("ROS_DOMAIN_ID", &domain)
        .env("ROS_AUTOMATIC_DISCOVERY_RANGE", "LOCALHOST")
        .stdout(std::fs::File::create(&log).unwrap())
        .stderr(std::fs::File::create(&log).unwrap())
        .spawn()
        .expect("spawn bridge");

    std::thread::sleep(Duration::from_millis(1500));
    let mut publisher = Command::new("python3")
        .arg("-c")
        .arg(PUBLISHER)
        .env("ROS_DOMAIN_ID", &domain)
        .env("ROS_AUTOMATIC_DISCOVERY_RANGE", "LOCALHOST")
        .stdout(std::process::Stdio::null())
        .spawn()
        .expect("spawn publisher");
    let _ = publisher.wait();

    // Let the 3 s window elapse and the hold engage; count commands.
    std::thread::sleep(Duration::from_millis(6000));
    let during = cmds.load(Ordering::Relaxed);
    let t0 = std::time::Instant::now();
    while t0.elapsed() < Duration::from_secs(4) {
        std::thread::sleep(Duration::from_millis(500));
    }
    let after = cmds.load(Ordering::Relaxed);

    stop.store(true, Ordering::Relaxed);
    let _ = bridge.kill();
    let _ = bridge.wait();
    let _ = server.join();
    let bridge_log = std::fs::read_to_string(&log).unwrap_or_default();
    eprintln!(
        "cmds during push: {during}, after hold: {after}\nlog tail: {}",
        bridge_log
            .lines()
            .rev()
            .take(3)
            .collect::<Vec<_>>()
            .join("\n")
    );
    assert!(
        bridge_log.contains("DRIVE STALLED"),
        "the stall was not detected; log: {bridge_log}"
    );
    // The wall is not being pushed: no new motion commands after the hold
    // engaged (the publisher is done by then).
    assert_eq!(after, during, "commands kept flowing after the stall latch");
}
