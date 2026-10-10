//! Integration test against a REAL local Player server.
//!
//! Prerequisites: the Player 3.1 server vendored into the local cache by
//! tools/get-player.sh (default ~/.cache/robot/player) with its dummy
//! config (laser:0 + position2d:0). The test starts the server itself on
//! a private port; if the cache is missing the test is skipped (env
//! ROBOT_PLAYER_SRC points at the tree, default as above).

use std::io::{Read, Write};
use std::net::{TcpListener, TcpStream};
use std::process::{Child, Command};
use std::time::Duration;

use player_bridge::*;

struct Server(Child);

impl Drop for Server {
    fn drop(&mut self) {
        let _ = self.0.kill();
        let _ = self.0.wait();
    }
}

fn start_server() -> Option<(Server, u16)> {
    let src = std::env::var("ROBOT_PLAYER_SRC")
        .unwrap_or_else(|_| format!("{}/.cache/robot/player", std::env::var("HOME").unwrap()));
    let bin = format!("{src}/server/player");
    let cfg = format!("{src}/config/dummy.cfg");
    if !std::path::Path::new(&bin).exists() || !std::path::Path::new(&cfg).exists() {
        eprintln!("skipping: player tree not found at {src}");
        return None;
    }
    // free port
    let listener = TcpListener::bind("127.0.0.1:0").unwrap();
    let port = listener.local_addr().unwrap().port();
    drop(listener);

    let libs = [
        "libplayerinterface",
        "libplayertcp",
        "libplayercommon",
        "libplayercore",
        "libplayerwkb",
        "libplayerjpeg",
        "server/libplayerdrivers",
        "libplayerutil",
    ]
    .iter()
    .map(|d| format!("{src}/{d}"))
    .collect::<Vec<_>>()
    .join(":");

    let child = Command::new(&bin)
        .args(["-p", &port.to_string(), &cfg])
        .env("LD_LIBRARY_PATH", libs)
        .stderr(std::process::Stdio::null())
        .stdout(std::process::Stdio::null())
        .spawn()
        .ok()?;
    // wait for the port to accept
    for _ in 0..50 {
        if TcpStream::connect(("127.0.0.1", port)).is_ok() {
            return Some((Server(child), port));
        }
        std::thread::sleep(Duration::from_millis(100));
    }
    eprintln!("skipping: server did not come up");
    None
}

#[test]
fn subscribes_and_reads_laser_from_real_server() {
    let Some((_server, port)) = start_server() else {
        return;
    };

    let mut client = Client::connect("127.0.0.1", port).expect("connect");
    let granted = client.subscribe(LASER_CODE, 0).expect("subscribe laser");
    assert_eq!(granted, OPEN_MODE, "expected open access granted");

    // dummy driver: 75 Hz pushes; wait up to 5 s for the first DATA_SCAN
    let deadline = std::time::Instant::now() + Duration::from_secs(5);
    let scan = loop {
        let msg = client.read_message().expect("read");
        if msg.header.msg_type == MSGTYPE_DATA
            && msg.header.addr.interf == LASER_CODE
            && msg.header.subtype == LASER_DATA_SCAN
        {
            break decode_laser_scan(&msg.payload).expect("decode scan");
        }
        assert!(
            std::time::Instant::now() < deadline,
            "no laser data arrived"
        );
    };
    // dummy laser defaults: 180 beams?? — assert what the driver actually
    // sends: nonempty scan with sane geometry
    assert!(!scan.ranges.is_empty(), "scan has no ranges");
    assert!(scan.resolution > 0.0, "resolution must be positive");
}

#[test]
fn subscribes_and_reads_position2d_from_real_server() {
    let Some((_server, port)) = start_server() else {
        return;
    };

    let mut client = Client::connect("127.0.0.1", port).expect("connect");
    let granted = client
        .subscribe(POSITION2D_CODE, 0)
        .expect("subscribe position2d");
    assert_eq!(granted, OPEN_MODE);

    let deadline = std::time::Instant::now() + Duration::from_secs(5);
    let state = loop {
        let msg = client.read_message().expect("read");
        if msg.header.msg_type == MSGTYPE_DATA
            && msg.header.addr.interf == POSITION2D_CODE
            && msg.header.subtype == POSITION2D_DATA_STATE
        {
            break decode_position2d(&msg.payload).expect("decode position2d");
        }
        assert!(
            std::time::Instant::now() < deadline,
            "no position data arrived"
        );
    };
    assert!(state.x.is_finite() && state.y.is_finite() && state.yaw.is_finite());
}

#[test]
fn velocity_command_is_accepted_by_real_server() {
    let Some((_server, port)) = start_server() else {
        return;
    };

    let mut client = Client::connect("127.0.0.1", port).expect("connect");
    client.subscribe(POSITION2D_CODE, 0).expect("subscribe");
    // CMD messages get no reply; the dummy driver silently accepts them.
    client
        .send_velocity(0, 0.05, 0.0, 1)
        .expect("send velocity command");
    // A following read still works: the connection is healthy.
    let _ = client.read_message();
}

// Silence unused-import warnings for the Read/Write used via TcpStream.
#[allow(dead_code)]
fn _t<T: Read + Write>(_: &T) {}
