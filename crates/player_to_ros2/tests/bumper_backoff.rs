//! Integration test: on bumper contact the bridge drives straight back at
//! once (the stock MoveBack window, before the MCU latches a wheel error),
//! ignoring Nav2's forward /cmd_vel while the manoeuvre runs. Skipped
//! without a sourced ROS 2 environment.

use std::io::{Read, Write};
use std::net::{TcpListener, TcpStream};
use std::process::Command;
use std::sync::atomic::{AtomicBool, AtomicI32, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use player_bridge::{
    decode_bumper, decode_header, encode_header, DevAddr, MsgHeader, BUMPER_CODE,
    BUMPER_DATA_STATE, MSGHDR_SIZE, MSGTYPE_DATA, MSGTYPE_REQ, MSGTYPE_RESP_ACK, POSITION2D_CODE,
    POSITION2D_DATA_STATE,
};

const MSGTYPE_CMD: u8 = 2;
const POSITION2D_CMD_VEL: u8 = 1;

type Log = Arc<Mutex<Vec<(Instant, f64, f64)>>>;

fn send(out: &Mutex<TcpStream>, header: MsgHeader, payload: &[u8]) -> std::io::Result<()> {
    let mut s = out.lock().unwrap();
    s.write_all(&encode_header(&header))?;
    s.write_all(payload)
}

fn bumper_frame(flags: &[u8]) -> Vec<u8> {
    let mut p = vec![0u8; 8];
    p[0..4].copy_from_slice(&(flags.len() as u32).to_be_bytes());
    p[4..8].copy_from_slice(&(flags.len() as u32).to_be_bytes());
    p.extend_from_slice(flags);
    p.resize(p.len().div_ceil(4) * 4, 0);
    p
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

fn fake_player(listener: TcpListener, log: Log, stop: Arc<AtomicBool>) {
    let (stream, _) = listener.accept().expect("bridge connects");
    let mut banner = [0u8; 32];
    banner[..14].copy_from_slice(b"Player v.3.1.0");
    let out = Arc::new(Mutex::new(stream.try_clone().unwrap()));
    out.lock().unwrap().write_all(&banner).unwrap();

    let data_out = out.clone();
    let data_stop = stop.clone();
    std::thread::spawn(move || {
        let mut pressed_at = None;
        while !data_stop.load(Ordering::Relaxed) {
            if send(
                &data_out,
                data_header(POSITION2D_CODE, POSITION2D_DATA_STATE, 52),
                &[0u8; 52],
            )
            .is_err()
            {
                return;
            }
            // Press the bumper 2 s in for 1.5 s, then release — like a
            // hand press; the manoeuvre is one-shot per press.
            if pressed_at.is_none() {
                pressed_at = Some(Instant::now());
            }
            let held = pressed_at.unwrap().elapsed();
            let flags: &[u8] = if held > Duration::from_secs(2) && held < Duration::from_secs(4) {
                &[1, 0, 0]
            } else {
                &[0, 0, 0]
            };
            let p = bumper_frame(flags);
            let _ = send(
                &data_out,
                data_header(BUMPER_CODE, BUMPER_DATA_STATE, p.len() as u32),
                &p,
            );
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
                log.lock().unwrap().push((Instant::now(), f(0), f(16)));
            }
            _ => {}
        }
    }
}

const NAV2_STUB: &str = r#"
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

rclpy.init()
n = Node('nav2_stub')
pub = n.create_publisher(Twist, '/cmd_vel', 10)
end = time.time() + 8
while time.time() < end:
    t = Twist(); t.linear.x = 0.06   # insists on driving forward
    pub.publish(t)
    rclpy.spin_once(n, timeout_sec=0.05)
print('STUB DONE')
"#;

#[test]
fn bumper_contact_drives_back_despite_forward_commands() {
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
    let _ = decode_bumper(&[0, 0, 0, 0, 0, 0, 0, 0]); // codec linked
    let listener = TcpListener::bind("127.0.0.1:0").unwrap();
    let port = listener.local_addr().unwrap().port();
    let stop = Arc::new(AtomicBool::new(false));
    let log: Log = Arc::new(Mutex::new(Vec::new()));
    let server = {
        let (log, stop) = (log.clone(), stop.clone());
        std::thread::spawn(move || fake_player(listener, log, stop))
    };

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

    std::thread::sleep(Duration::from_millis(1500));
    let _ = Command::new("python3")
        .arg("-c")
        .arg(NAV2_STUB)
        .env("ROS_DOMAIN_ID", &domain)
        .env("ROS_AUTOMATIC_DISCOVERY_RANGE", "LOCALHOST")
        .stdout(std::process::Stdio::null())
        .status()
        .expect("run nav2 stub");
    std::thread::sleep(Duration::from_millis(500));

    stop.store(true, Ordering::Relaxed);
    let _ = bridge.kill();
    let _ = bridge.wait();
    let _ = server.join();

    let log = log.lock().unwrap().clone();
    let reverses: Vec<_> = log.iter().filter(|(_, vx, _)| *vx < -0.05).collect();
    eprintln!("commands: {} total, {} reverse", log.len(), reverses.len());
    if let (Some(f), Some(l)) = (reverses.first(), reverses.last()) {
        eprintln!("reverse span: {:?}", l.0.duration_since(f.0));
    }
    let t0 = log[0].0;
    for (t, vx, _) in log.iter() {
        let dt = t.duration_since(t0).as_millis();
        if dt % 500 < 70 {
            eprintln!("  t+{dt:5} ms  vx={vx:+.3}");
        }
    }
    assert!(
        reverses.len() >= 5,
        "the bumper back-off never reached Player: {} reverse commands in {:?}",
        reverses.len(),
        log.iter()
            .map(|(t, v, w)| (t, *v, *w))
            .take(5)
            .collect::<Vec<_>>()
    );
    // The manoeuvre is one-shot and bounded: reverse spans about the
    // configured 1.2 s (never endless — the live bug), and while it runs
    // no forward command reaches Player (the stub keeps publishing).
    let span = reverses[reverses.len() - 1].0.duration_since(reverses[0].0);
    assert!(
        span >= Duration::from_millis(800) && span <= Duration::from_millis(2000),
        "reverse span {span:?} is not the ~1.2 s manoeuvre"
    );
    // No forward inside the reverse span (both ends inclusive, one tick of
    // tolerance).
    // Middle of the span only: at the edges the manoeuvre start/end and
    // the reader's tick can interleave by one tick with the lease drain.
    let first = reverses[0].0 + Duration::from_millis(250);
    let last = reverses[reverses.len() - 1].0 - Duration::from_millis(250);
    let forward_during = log
        .iter()
        .any(|(t, vx, _)| *t >= first && *t <= last && *vx > 0.05);
    assert!(
        !forward_during,
        "a forward command reached Player during the back-off"
    );
}
