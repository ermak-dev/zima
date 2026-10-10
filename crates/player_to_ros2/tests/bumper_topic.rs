//! Integration test: the bridge subscribes to bumper:0 and publishes the
//! bumper state on /bumper (std_msgs/Bool, true while any segment is
//! pressed). The fake Player streams bumper frames like the vendor driver:
//! one released frame at start, then one pressed frame when the test
//! raises the flag. Skipped without a sourced ROS 2 environment (like the
//! other e2e tests of this repo).

use std::io::{Read, Write};
use std::net::{TcpListener, TcpStream};
use std::process::Command;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use player_bridge::{
    decode_header, encode_header, DevAddr, MsgHeader, BUMPER_CODE, BUMPER_DATA_STATE, MSGHDR_SIZE,
    MSGTYPE_DATA, MSGTYPE_REQ, MSGTYPE_RESP_ACK, POSITION2D_CODE, POSITION2D_DATA_STATE,
};

fn bumper_frame(flags: &[u8]) -> Vec<u8> {
    let mut p = vec![0u8; 8];
    p[0..4].copy_from_slice(&(flags.len() as u32).to_be_bytes());
    p[4..8].copy_from_slice(&(flags.len() as u32).to_be_bytes());
    p.extend_from_slice(flags);
    p.resize(p.len().div_ceil(4) * 4, 0);
    p
}

fn send(out: &Mutex<TcpStream>, header: MsgHeader, payload: &[u8]) -> std::io::Result<()> {
    let mut s = out.lock().unwrap();
    s.write_all(&encode_header(&header))?;
    s.write_all(payload)
}

fn data_header(interf: u16, subtype: u8, size: u32) -> MsgHeader {
    MsgHeader {
        addr: DevAddr {
            host: 0,
            robot: 0,
            interf,
            index: 0,
        },
        msg_type: MSGTYPE_DATA,
        subtype,
        timestamp: 0.0,
        seq: 0,
        size,
    }
}

/// Serve the bridge: banner, ACK every request, odometry at 20 Hz, and
/// bumper frames — released once at start, pressed once when the test
/// raises `pressed` (the vendor driver publishes on change).
fn fake_player(listener: TcpListener, pressed: Arc<AtomicBool>, stop: Arc<AtomicBool>) {
    let (stream, _) = listener.accept().expect("bridge connects");
    let mut banner = [0u8; 32];
    banner[..14].copy_from_slice(b"Player v.3.1.0");
    let out = Arc::new(Mutex::new(stream.try_clone().unwrap()));
    out.lock().unwrap().write_all(&banner).unwrap();

    let data_out = out.clone();
    let data_stop = stop.clone();
    let press_flag = pressed.clone();
    std::thread::spawn(move || {
        let mut sent_pressed = false;
        while !data_stop.load(Ordering::Relaxed) {
            // Initial released frame once the bridge has had a moment to
            // subscribe (the vendor driver sends one frame at subscribe).
            // Released frames keep flowing until the press: the vendor
            // driver republishes the same state; the bridge must see the
            // initial released state whenever it subscribes and must
            // publish it exactly once (dedup).
            if !press_flag.load(Ordering::Relaxed) {
                let initial = bumper_frame(&[0, 0, 0]);
                let _ = send(
                    &data_out,
                    data_header(BUMPER_CODE, BUMPER_DATA_STATE, initial.len() as u32),
                    &initial,
                );
            }
            if send(
                &data_out,
                data_header(POSITION2D_CODE, POSITION2D_DATA_STATE, 52),
                &[0u8; 52],
            )
            .is_err()
            {
                return;
            }
            if press_flag.load(Ordering::Relaxed) && !sent_pressed {
                sent_pressed = true;
                let payload = bumper_frame(&[1, 0, 0]);
                let _ = send(
                    &data_out,
                    data_header(BUMPER_CODE, BUMPER_DATA_STATE, payload.len() as u32),
                    &payload,
                );
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
        if header.msg_type == MSGTYPE_REQ {
            let mut ack = [0u8; 28];
            ack[19] = 1; // granted access
            let reply = MsgHeader {
                msg_type: MSGTYPE_RESP_ACK,
                size: 28,
                ..header
            };
            send(&out, reply, &ack).unwrap();
        }
    }
}

/// Probe: subscribe, wait for the released state, signal the parent (via
/// the marker file) to press, then keep collecting.
const PROBE: &str = r#"
import os, sys, time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

marker = sys.argv[1]
rclpy.init()
n = Node('bumper_probe')
vals = []
def cb(m):
    vals.append(m.data)
    if not vals[-1] and False not in vals[:-1] and len(vals) >= 1:
        open(marker, 'w').write('seen-released')
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                 reliability=QoSReliabilityPolicy.RELIABLE)
n.create_subscription(Bool, '/bumper', cb, qos)
end = time.time() + 8
while time.time() < end and True not in vals:
    rclpy.spin_once(n, timeout_sec=0.1)
time.sleep(0.5)
print('VALUES', vals)
"#;

#[test]
fn bumper_state_reaches_the_bumper_topic() {
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
    let pressed = Arc::new(AtomicBool::new(false));
    let server = {
        let (stop, pressed) = (stop.clone(), pressed.clone());
        std::thread::spawn(move || fake_player(listener, pressed, stop))
    };

    // A private domain on localhost: this test's topics stay here.
    let domain = (100 + std::process::id() % 100).to_string();
    let mut bridge = Command::new(env!("CARGO_BIN_EXE_player_to_ros2"))
        .args(["--ros-args", "-p", "player_host:=127.0.0.1", "-p"])
        .arg(format!("player_port:={port}"))
        .env("ROS_DOMAIN_ID", &domain)
        .env("ROS_AUTOMATIC_DISCOVERY_RANGE", "LOCALHOST")
        .stdout(std::process::Stdio::null())
        .stderr(std::process::Stdio::null())
        .spawn()
        .expect("spawn bridge");

    // The fake holds its bumper frames until the probe is up: no state is
    // published before the subscriber exists.
    let marker = std::env::temp_dir().join(format!("bumper-probe-{}.mk", std::process::id()));
    let _ = std::fs::remove_file(&marker);
    let mut probe = Command::new("python3")
        .arg("-c")
        .arg(PROBE)
        .arg(&marker)
        .env("ROS_DOMAIN_ID", &domain)
        .env("ROS_AUTOMATIC_DISCOVERY_RANGE", "LOCALHOST")
        .stdout(std::process::Stdio::piped())
        .spawn()
        .expect("spawn probe");
    // Wait until the probe has actually seen the released state, then press.
    let marker = std::env::temp_dir().join(format!("bumper-probe-{}.mk", std::process::id()));
    let _ = std::fs::remove_file(&marker);
    for _ in 0..100 {
        if marker.exists() {
            break;
        }
        std::thread::sleep(Duration::from_millis(100));
    }
    assert!(marker.exists(), "probe never saw the released bumper");
    pressed.store(true, Ordering::Relaxed);

    let out = probe.wait_with_output().expect("probe finishes");
    stop.store(true, Ordering::Relaxed);
    let _ = bridge.kill();
    let _ = bridge.wait();
    let _ = server.join();

    let text = String::from_utf8_lossy(&out.stdout);
    eprintln!("probe: {text}");
    let line = text
        .lines()
        .find(|l| l.starts_with("VALUES "))
        .expect("probe printed VALUES");
    let vals: Vec<bool> = line["VALUES ".len()..]
        .trim_matches(|c| c == '[' || c == ']')
        .split(',')
        .map(|v| v.trim() == "True")
        .collect();
    assert!(!vals.is_empty(), "no /bumper messages received");
    assert!(vals[0] == false, "initial state must be released: {vals:?}");
    assert!(
        vals.contains(&true),
        "pressed state never reached the topic: {vals:?}"
    );
    let first_true = vals.iter().position(|&v| v).unwrap();
    assert!(
        vals[..first_true].iter().all(|&v| !v),
        "released first, then pressed: {vals:?}"
    );
}
