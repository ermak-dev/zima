//! Integration test: the bridge node against a REAL local Player server
//! (dummy laser + position2d). Checks the fix for RViz dropping scans:
//! the static base_link->laser transform must be latched on /tf_static
//! with transient_local QoS, so tf2 can resolve odom->laser at scan time.
//!
//! Prerequisites: the vendored Player tree (tools/get-player.sh, default
//! ~/.cache/robot/player). Skipped when absent. Needs a sourced ROS 2
//! environment, so it runs via colcon; when run bare (cargo test) it is
//! also skipped unless ROS_DISTRO is set.

use std::net::{TcpListener, TcpStream};
use std::process::{Child, Command};
use std::time::Duration;

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
fn static_laser_tf_is_latched_and_resolves_at_scan_time() {
    let Some((_server, port)) = start_server() else {
        return;
    };
    eprintln!("player server on port {port}");

    // Run the bridge node against the local server and capture its log.
    let bridge = env!("CARGO_BIN_EXE_player_to_ros2");
    let log = std::env::temp_dir().join("player_to_ros2_it.log");
    let mut child = Command::new(&bridge)
        .args([
            "--ros-args",
            "-p",
            "player_host:=127.0.0.1",
            "-p",
            &format!("player_port:={port}"),
        ])
        .stderr(std::fs::File::create(&log).unwrap())
        .stdout(std::fs::File::create(&log).unwrap())
        .spawn()
        .expect("spawn bridge");

    // Give the bridge a moment, then assert from a ROS test node.
    std::thread::sleep(Duration::from_secs(2));

    let mut assertion = Command::new("python3")
        .arg("-")
        .stdin(std::process::Stdio::piped())
        .stdout(std::process::Stdio::piped())
        .env("PYTHONUNBUFFERED", "1")
        .spawn()
        .expect("spawn python probe");
    let script = r#"
import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener

rclpy.init()
node = Node('tf_static_probe')

qos = QoSProfile(
    depth=1,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST,
    reliability=QoSReliabilityPolicy.RELIABLE,
)

got_static = {}
def on_tf_static(m):
    got_static['msg'] = m
node.create_subscription(TFMessage, '/tf_static', on_tf_static, qos)

got_tf = {}
def on_tf(m):
    got_tf['stamp'] = m.transforms[0].header.stamp if m.transforms else None
node.create_subscription(TFMessage, '/tf', on_tf, 10)

buf = Buffer()
TransformListener(buf, node)

deadline = node.get_clock().now() + rclpy.duration.Duration(seconds=8)
while node.get_clock().now() < deadline and ('msg' not in got_static or got_tf.get('stamp') is None):
    rclpy.spin_once(node, timeout_sec=0.2)

# spin_once runs a single callback per call: keep spinning so the
# TransformListener's own /tf subscription lands in the buffer too.
extra = node.get_clock().now() + rclpy.duration.Duration(seconds=1)
while node.get_clock().now() < extra:
    rclpy.spin_once(node, timeout_sec=0.1)

if 'msg' not in got_static:
    print('FAIL: no /tf_static message received'); sys.exit(1)

t = got_static['msg'].transforms[0]
if t.header.frame_id != 'base_link' or t.child_frame_id != 'laser':
    print(f'FAIL: wrong frames {t.header.frame_id}->{t.child_frame_id}'); sys.exit(1)
if t.header.stamp.sec == 0 and t.header.stamp.nanosec == 0:
    print('FAIL: /tf_static transform has zero stamp'); sys.exit(1)

stamp = got_tf.get('stamp')
if stamp is None:
    print('FAIL: no /tf odom messages received'); sys.exit(1)

# The odom->base_link TF carries the latest odometry time; resolving
# odom->laser at that stamp requires the latched static transform to be
# in the buffer — exactly what RViz's message filter needs at scan time.
# The probe's own /tf subscription can see that message before the
# TransformListener stores it ("extrapolation into the future"), so keep
# spinning until the buffer catches up with the very same stamp.
error = None
retry_until = node.get_clock().now() + rclpy.duration.Duration(seconds=2)
while True:
    try:
        buf.lookup_transform('odom', 'laser', rclpy.time.Time.from_msg(stamp))
        print('LOOKUP OK')
        break
    except Exception as e:
        error = e
    if node.get_clock().now() > retry_until:
        print(f'FAIL: odom->laser lookup failed: {error}'); sys.exit(1)
    rclpy.spin_once(node, timeout_sec=0.05)
"#;
    {
        use std::io::Write;
        let mut stdin = assertion.stdin.as_ref().unwrap();
        stdin.write_all(script.as_bytes()).unwrap();
    }
    let out = assertion.wait_with_output().unwrap();
    let _ = child.kill();
    let _ = child.wait();
    print!("{}", String::from_utf8_lossy(&out.stdout));
    if !out.status.success() {
        panic!("tf_static assertion failed");
    }
}
