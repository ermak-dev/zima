//! player_to_ros2: bridge the vacuum's Player server into ROS 2.
//!
//!   laser:0       -> /scan   (sensor_msgs/LaserScan)
//!   position2d:0  -> /odom   (nav_msgs/Odometry)
//!   /cmd_vel      -> velocity commands to position2d:0
//!
//! Safety notes carried over from the platform survey:
//! - The Player odometry frame is unknown until verified live; frame_ids
//!   default to "odom"/"laser" and are ROS parameters.
//! - The bridge never enables the device mask itself: run the mask owner
//!   (bin/robot mask hold) separately, or the laser will stay silent.

use std::io::Write;
use std::sync::mpsc::{Receiver, TryRecvError};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use rclrs::*;
use ros_env::sensor_msgs::msg::LaserScan;
use ros_env::nav_msgs::msg::Odometry;

use player_bridge::{
    decode_position2d, decode_laser_scanangle, Client, Error as PlayerError, LaserScanAngle,
    Position2dState, LASER_CODE, LASER_DATA_SCANANGLE, MSGTYPE_DATA, POSITION2D_CODE,
    POSITION2D_DATA_STATE,
};

/// A decoded Player message handed from the reader thread to the ROS loop.
enum BridgeEvent {
    Scan(LaserScanAngle),
    Odom(Position2dState),
    ProtocolError(String),
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();
    let node = executor.commands().create_node("player_to_ros2")?;

    let host: rclrs::MandatoryParameter<Arc<str>> = node
        .declare_parameter("player_host")
        .default("robot".into())
        .mandatory()?;
    let port: rclrs::MandatoryParameter<i64> = node
        .declare_parameter("player_port")
        .default(6665i64)
        .mandatory()?;
    let laser_frame: rclrs::MandatoryParameter<Arc<str>> = node
        .declare_parameter("laser_frame")
        .default("laser".into())
        .mandatory()?;
    let odom_frame: rclrs::MandatoryParameter<Arc<str>> = node
        .declare_parameter("odom_frame")
        .default("odom".into())
        .mandatory()?;

    // Player reader thread: decodes wire messages, forwards them, and
    // flushes outgoing velocity commands between reads.
    let (tx, rx): (_, Receiver<BridgeEvent>) = std::sync::mpsc::channel();
    let cmd_queue: Arc<Mutex<Vec<(f64, f64)>>> = Arc::new(Mutex::new(Vec::new()));
    let cmd_queue_reader = cmd_queue.clone();

    let host = host.get().to_string();
    let port = u16::try_from(port.get()).unwrap_or(6665);
    let laser_frame: String = laser_frame.get().to_string();
    let odom_frame: String = odom_frame.get().to_string();

    let host_for_log = host.clone();

    // Heartbeat to the on-robot supervisor (if configured): one byte per
    // second proves this process is alive. The supervisor stops the robot
    // and restores the stock controller when the bytes stop.
    let heartbeat = std::env::var("ROBOT_BRIDGE_HEARTBEAT").ok().map(|addr| {
        std::thread::spawn(move || {
            let mut stream = std::net::TcpStream::connect(&addr).ok();
            if let Some(s) = stream.as_mut() {
                let _ = s.write_all(b"ACTIVATE\n");
                let _ = s.flush();
            }
            loop {
                std::thread::sleep(std::time::Duration::from_secs(1));
                let beat_ok = match stream.as_mut() {
                    Some(s) => s.write_all(b"hb").is_ok(),
                    None => false,
                };
                if !beat_ok {
                    eprintln!("player_to_ros2: heartbeat connection lost");
                    // Losing the supervisor means the robot is being
                    // rolled back: exit so no velocity commands keep
                    // flowing from a dead session.
                    std::process::exit(0);
                }
            }
        })
    });

    let reader = std::thread::spawn(move || {
        let run = || -> Result<(), PlayerError> {
            let mut client = Client::connect(&host, port)?;
            client.subscribe(LASER_CODE, 0)?;
            client.subscribe(POSITION2D_CODE, 0)?;
            loop {
                loop {
                    let next = {
                        let mut q = cmd_queue_reader.lock().unwrap();
                        if q.is_empty() { None } else { Some(q.remove(0)) }
                    };
                    match next {
                        Some((vx, vyaw)) => client.send_velocity(0, vx, vyaw, 1)?,
                        None => break,
                    }
                }
                let msg = client.read_message()?;
                match (msg.header.msg_type, msg.header.addr.interf, msg.header.subtype) {
                    (MSGTYPE_DATA, LASER_CODE, LASER_DATA_SCANANGLE) => {
                        // The vendor driver publishes per-beam angles
                        // (subtype 3) in a wrapped, descending order; the
                        // conversion to a ROS LaserScan happens on the
                        // ROS side (see the Scan arm below).
                        let scan = decode_laser_scanangle(&msg.payload)?;
                        if tx.send(BridgeEvent::Scan(scan)).is_err() {
                            return Ok(()); // ROS side went away
                        }
                    }
                    (MSGTYPE_DATA, POSITION2D_CODE, POSITION2D_DATA_STATE) => {
                        let state = decode_position2d(&msg.payload)?;
                        if tx.send(BridgeEvent::Odom(state)).is_err() {
                            return Ok(());
                        }
                    }
                    _ => {} // other interfaces: ignore
                }
            }
        };
        if let Err(e) = run() {
            let _ = tx.send(BridgeEvent::ProtocolError(e.to_string()));
        }
    });

    let scan_pub = node.create_publisher::<LaserScan>("/scan")?;
    let odom_pub = node.create_publisher::<Odometry>("/odom")?;
    let clock = Clock::system();

    let subscription = node.create_subscription(
        "/cmd_vel",
        move |msg: ros_env::geometry_msgs::msg::Twist| {
            cmd_queue.lock().unwrap().push((msg.linear.x, msg.angular.z));
        },
    )?;

    let mut last_report = std::time::Instant::now();
    loop {
        loop {
            match rx.try_recv() {
                Ok(BridgeEvent::Scan(scan)) => {
                    // Normalize the vendor scan: unwrap the circular
                    // order, sort ascending, and derive a uniform grid
                    // (start, step, count) for sensor_msgs/LaserScan.
                    let mut beams: Vec<(f32, f32)> =
                        scan.angles.iter().copied().zip(scan.ranges.iter().copied()).collect();
                    // unwrap: rotate so the array starts at the smallest
                    // angle after adding 2*pi to wrapped negatives
                    let min_a = beams.iter().map(|b| b.0).fold(f32::INFINITY, f32::min);
                    for b in beams.iter_mut() {
                        while b.0 < min_a {
                            b.0 += std::f32::consts::TAU;
                        }
                    }
                    beams.sort_by(|x, y| x.0.partial_cmp(&y.0).unwrap());
                    let n = beams.len();
                    let mut ros_scan = LaserScan::default();
                    ros_scan.header.frame_id = laser_frame.clone();
                    if n >= 2 {
                        ros_scan.angle_min = beams[0].0;
                        ros_scan.angle_max = beams[n - 1].0;
                        ros_scan.angle_increment = (beams[n - 1].0 - beams[0].0) / (n - 1) as f32;
                    }
                    ros_scan.range_max = scan.max_range;
                    ros_scan.ranges = beams.iter().map(|b| b.1).collect();
                    ros_scan.intensities = scan.intensities.iter().map(|&i| i as f32).collect();
                    let _ = scan_pub.publish(&ros_scan);
                }
                Ok(BridgeEvent::Odom(state)) => {
                    let mut odom = Odometry::default();
                    odom.header.frame_id = odom_frame.clone();
                    odom.child_frame_id = "base_link".into();
                    if let Ok(stamp) = clock.now().to_ros_msg() {
                        odom.header.stamp = stamp;
                    }
                    odom.pose.pose.position.x = state.x;
                    odom.pose.pose.position.y = state.y;
                    odom.pose.pose.orientation = yaw_to_quat(state.yaw);
                    odom.twist.twist.linear.x = state.vx;
                    odom.twist.twist.angular.z = state.vyaw;
                    let _ = odom_pub.publish(&odom);
                }
                Ok(BridgeEvent::ProtocolError(e)) => return Err(e.into()),
                Err(TryRecvError::Empty) => break,
                Err(TryRecvError::Disconnected) => {
                    std::thread::sleep(Duration::from_millis(50));
                }
            }
        }
        executor.spin(SpinOptions::spin_once().timeout(Duration::from_millis(10)));
        if last_report.elapsed() >= Duration::from_secs(30) {
            log_info!(node.logger(), "bridge alive (player {}:{})", host_for_log, port);
            last_report = std::time::Instant::now();
        }
    }

    let _ = reader.join();
    let _ = subscription;
    Ok(())
}

fn yaw_to_quat(yaw: f64) -> ros_env::geometry_msgs::msg::Quaternion {
    ros_env::geometry_msgs::msg::Quaternion {
        x: 0.0,
        y: 0.0,
        z: (yaw / 2.0).sin(),
        w: (yaw / 2.0).cos(),
    }
}
