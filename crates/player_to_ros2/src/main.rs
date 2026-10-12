//! player_to_ros2: bridge the vacuum's Player server into ROS 2.
//!
//!   laser:0       -> /scan   (sensor_msgs/LaserScan)
//!   bumper:0      -> /bumper (std_msgs/Bool, any segment pressed)
//!   position2d:0  -> /odom   (nav_msgs/Odometry)
//!   ir:0 (wall)   -> /wall_ir + /voltages/ir0  (Range / Float32MultiArray)
//!   ir:1 (cliff)  -> /cliff_ir_N + /voltages/ir1
//!   ir:2 (drop)   -> /drop_ir_N + /voltages/ir2
//!   sonar:0       -> /sonar   (sensor_msgs/Range, ULTRASOUND)
//!   power:0       -> /battery_state (sensor_msgs/BatteryState)
//!   /cmd_vel      -> velocity commands to position2d:0
//!
//! Drive safety (player_bridge::drive): every /cmd_vel is clamped to the
//! project limits (0.1 m/s, 0.4 rad/s), only the latest command is sent,
//! and a command holds for `cmd_vel_timeout_s` (default 1 s): when the
//! sender goes quiet, the wheels get one zero. Player alone would keep
//! the last velocity forever.
//!
//! Safety notes carried over from the platform survey:
//! - The Player odometry frame is unknown until verified live; frame_ids
//!   default to "odom"/"laser" and are ROS parameters.
//! - The bridge never enables the device mask itself: run the mask owner
//!   (bin/robot mask hold) separately, or the laser will stay silent.

use std::io::Write;
use std::sync::mpsc::{Receiver, TryRecvError};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use rclrs::*;
use ros_env::geometry_msgs::msg::{Transform, TransformStamped, Vector3};
use ros_env::nav_msgs::msg::Odometry;
use ros_env::sensor_msgs::msg::LaserScan;
use ros_env::tf2_msgs::msg::TFMessage;

use player_bridge::drive::{BackOff, DriveCommand, StallDetector, MAX_ANGULAR, MAX_LINEAR};
use player_bridge::scan::uniform_scan;
use player_bridge::{
    decode_bumper, decode_ir, decode_laser_scanangle, decode_position2d, decode_power,
    decode_sonar, Client, Error as PlayerError, IrState, LaserScanAngle, Position2dState,
    PowerState, SonarState, BUMPER_CODE, BUMPER_DATA_STATE, IR_CODE, IR_DATA_RANGES, LASER_CODE,
    LASER_DATA_SCANANGLE, MSGTYPE_DATA, POSITION2D_CODE, POSITION2D_DATA_STATE, POWER_CODE,
    POWER_DATA_STATE, SONAR_CODE, SONAR_DATA_RANGES,
};

/// A decoded Player message handed from the reader thread to the ROS loop.
enum BridgeEvent {
    Scan(LaserScanAngle),
    Odom(Position2dState),
    /// Bumper segments pressed, on change only.
    Bumper(Vec<bool>),
    /// An ir array frame tagged with its Player index.
    Ir(u16, IrState),
    Sonar(SonarState),
    Power(PowerState),
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
    let base_frame: rclrs::MandatoryParameter<Arc<str>> = node
        .declare_parameter("base_frame")
        .default("base_link".into())
        .mandatory()?;
    // Laser mounting on the vacuum: from the Player config of the vendor
    // driver (position [-0.100, 0, 0] m, yaw 97.6 deg). Override until a
    // live calibration says otherwise.
    let laser_x: rclrs::MandatoryParameter<f64> = node
        .declare_parameter("laser_x")
        .default(-0.100)
        .mandatory()?;
    let laser_y: rclrs::MandatoryParameter<f64> =
        node.declare_parameter("laser_y").default(0.0).mandatory()?;
    // Command lease: how long one /cmd_vel keeps the wheels turning. Nav2
    // publishes at 20 Hz; teleop key autorepeat starts after ~0.5 s.
    let cmd_vel_timeout_s: rclrs::MandatoryParameter<f64> = node
        .declare_parameter("cmd_vel_timeout_s")
        .default(1.0)
        .mandatory()?;
    let laser_yaw_deg: rclrs::MandatoryParameter<f64> = node
        .declare_parameter("laser_yaw_deg")
        .default(97.6)
        .mandatory()?;

    // Player reader thread: decodes wire messages, forwards them, and
    // flushes outgoing velocity commands between reads.
    let (tx, rx): (_, Receiver<BridgeEvent>) = std::sync::mpsc::channel();
    let lease = Duration::from_secs_f64(cmd_vel_timeout_s.get().clamp(0.05, 5.0));
    let drive = Arc::new(Mutex::new(DriveCommand::new(lease)));
    let drive_reader = drive.clone();
    // For the ROS loop: the reader thread owns drive_reader.
    let drive_ros = drive.clone();
    // Blocked-drive detection: commands flow but the wheels do not turn
    // (first live drive: pressed against a wall for a minute). Once
    // latched, /cmd_vel is swallowed until it goes quiet — pushing the
    // wall harder only deepens the MCU error latch.
    let stall = Arc::new(Mutex::new(StallDetector::new()));
    let stall_reader = stall.clone();
    let stall_hold = Arc::new(Mutex::new(None::<Instant>));
    let stall_hold_reader = stall_hold.clone();
    // Bumper back-off: on contact the bridge drives straight back at once
    // (stock MoveBack), before the MCU latches a wheel error.
    let backoff = Arc::new(Mutex::new(BackOff::new()));
    let backoff_ros = backoff.clone();
    let backoff_reader = backoff.clone();
    // Latest commanded velocity (pre-clamp) for the detector.
    let latest_cmd = Arc::new(Mutex::new((0.0, 0.0)));
    let latest_cmd_sub = latest_cmd.clone();
    let latest_cmd_reader = latest_cmd.clone();
    let host = host.get().to_string();
    let port = u16::try_from(port.get()).unwrap_or(6665);
    let laser_frame: String = laser_frame.get().to_string();
    let odom_frame: String = odom_frame.get().to_string();
    let base_frame: String = base_frame.get().to_string();
    let (laser_x, laser_y, laser_yaw) = (
        laser_x.get(),
        laser_y.get(),
        laser_yaw_deg.get().to_radians(),
    );

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

    // Latest bumper state, shared with the reader thread for change detection.
    let bumper_state = Arc::new(Mutex::new(None));
    let bumper_state_reader = bumper_state.clone();

    let reader = std::thread::spawn(move || {
        let run = || -> Result<(), PlayerError> {
            let mut client = Client::connect(&host, port)?;
            client.subscribe(LASER_CODE, 0)?;
            client.subscribe(POSITION2D_CODE, 0)?;
            // The bumper is optional: test Player servers (dummy.cfg) have
            // no bumper:0 and NACK the subscription; the robot always has
            // it. Without it the bridge runs without /bumper and without
            // the back-off rather than not at all.
            if client.subscribe(BUMPER_CODE, 0).is_err() {
                eprintln!(
                    "player_to_ros2: no bumper:0 on this Player; /bumper and back-off disabled"
                );
            }
            // Optional sensor devices: the robot always has them, the
            // test dummy.cfg does not. Each failure only drops its own
            // topics. Player indexes: ir:0 wall, ir:1 cliff, ir:2 drop,
            // sonar:0 ultrasonic, power:0 battery.
            for (interf, index) in [
                (IR_CODE, 0u16),
                (IR_CODE, 1),
                (IR_CODE, 2),
                (SONAR_CODE, 0),
                (POWER_CODE, 0),
            ] {
                if client.subscribe(interf, index).is_err() {
                    eprintln!(
                        "player_to_ros2: no {}:{} on this Player; its topics are disabled",
                        interf, index
                    );
                }
            }
            loop {
                // Between reads (Player streams odometry at 20+ Hz). The
                // bumper back-off owns the drive while it runs; otherwise
                // the latest command, or the stop once its lease lapsed.
                let next = backoff_reader
                    .lock()
                    .unwrap()
                    .active(Instant::now())
                    .or_else(|| drive_reader.lock().unwrap().next(Instant::now()));
                if let Some((vx, vyaw)) = next {
                    client.send_velocity(0, vx, vyaw, 1)?;
                }
                let msg = client.read_message()?;
                match (
                    msg.header.msg_type,
                    msg.header.addr.interf,
                    msg.header.subtype,
                ) {
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
                    (MSGTYPE_DATA, BUMPER_CODE, BUMPER_DATA_STATE) => {
                        // Publish on change only: the vendor driver streams
                        // the same frame; consumers care about transitions.
                        let pressed = decode_bumper(&msg.payload)?.pressed;
                        let any = pressed.iter().any(|&p| p);
                        // Edge-triggered (observe): the vendor driver
                        // streams the pressed frame; re-triggering on every
                        // frame made the reverse endless (seen live).
                        backoff_reader.lock().unwrap().observe(any, Instant::now());
                        let mut last = bumper_state_reader.lock().unwrap();
                        if last.as_ref() != Some(&pressed) {
                            *last = Some(pressed.clone());
                            drop(last);
                            if tx.send(BridgeEvent::Bumper(pressed)).is_err() {
                                return Ok(());
                            }
                        }
                    }
                    (MSGTYPE_DATA, IR_CODE, IR_DATA_RANGES) => {
                        let state = decode_ir(&msg.payload)?;
                        let index = msg.header.addr.index;
                        if tx.send(BridgeEvent::Ir(index, state)).is_err() {
                            return Ok(());
                        }
                    }
                    (MSGTYPE_DATA, SONAR_CODE, SONAR_DATA_RANGES) => {
                        let state = decode_sonar(&msg.payload)?;
                        if tx.send(BridgeEvent::Sonar(state)).is_err() {
                            return Ok(());
                        }
                    }
                    (MSGTYPE_DATA, POWER_CODE, POWER_DATA_STATE) => {
                        let state = decode_power(&msg.payload)?;
                        if tx.send(BridgeEvent::Power(state)).is_err() {
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

    // Latched: a late subscriber (RViz opened later, a recovery node
    // starting up) immediately learns the current bumper state.
    let bumper_pub = node.create_publisher("/bumper".transient_local())?;
    let scan_pub = node.create_publisher::<LaserScan>("/scan")?;
    let odom_pub = node.create_publisher::<Odometry>("/odom")?;
    let tf_pub = node.create_publisher::<TFMessage>("/tf")?;
    // Static transforms belong on /tf_static with transient_local QoS:
    // latched for late subscribers and valid at any query time. Publishing
    // an unstamped static transform into /tf (as before) made tf2 unable
    // to resolve odom->laser at scan time, so RViz dropped every scan.
    let tf_static_pub = node.create_publisher("/tf_static".transient_local())?;

    // Sensor publishers: one Range per array element lets RViz render
    // each cliff/wall/sonar beam; the raw voltages and battery go to
    // their own topics. Publishers are created once, keyed by topic
    // name, so repeated frames do not churn.
    use ros_env::sensor_msgs::msg::{BatteryState, Range};
    use std::collections::HashMap;
    let range_pubs: Arc<Mutex<HashMap<String, Publisher<Range>>>> =
        Arc::new(Mutex::new(HashMap::new()));
    // Event-driven sensors (cliff/drop/power) send one frame per state
    // change: latch their publishers so a late subscriber still learns
    // the current state, like the bumper topic does.
    let range_pub_new = |topic: &str| -> Result<Publisher<Range>, rclrs::RclrsError> {
        node.create_publisher(topic.transient_local())
    };
    let volt_pub_wall = node.create_publisher("/wall_ir/voltages".transient_local())?;
    let volt_pub_cliff = node.create_publisher("/cliff_ir/voltages".transient_local())?;
    let volt_pub_drop = node.create_publisher("/drop_ir/voltages".transient_local())?;
    let sonar_pub = node.create_publisher("/sonar".transient_local())?;
    let battery_pub = node.create_publisher::<BatteryState>("/battery_state".transient_local())?;

    // Static base_link -> laser transform (mounting from the vendor config).
    let clock = Clock::system();
    let mut static_tf = {
        let mut m = TFMessage::default();
        let mut t = TransformStamped::default();
        t.header.frame_id = base_frame.clone();
        t.child_frame_id = laser_frame.clone();
        t.transform = Transform {
            translation: Vector3 {
                x: laser_x,
                y: laser_y,
                z: 0.0,
            },
            rotation: yaw_to_quat(laser_yaw),
        };
        m.transforms.push(t);
        m
    };
    // A nonzero stamp keeps the message publishable on /tf_static without
    // breaking tf2's static cache (which keys on the frame pair, not time).
    if let Some(t) = static_tf.transforms.first_mut() {
        if let Ok(stamp) = clock.now().to_ros_msg() {
            t.header.stamp = stamp;
        }
    }

    let subscription = node.create_subscription(
        "/cmd_vel",
        move |msg: ros_env::geometry_msgs::msg::Twist| {
            *latest_cmd_sub.lock().unwrap() = (msg.linear.x, msg.angular.z);
            // While a bumper back-off runs, Nav2 commands do not reach the
            // wheels — the manoeuvre owns the drive.
            if backoff_ros.lock().unwrap().is_running(Instant::now()) {
                return;
            }
            let mut hold = stall_hold.lock().unwrap();
            if hold.is_some_and(|until| Instant::now() < until) {
                return; // stalled: swallow commands until the hold expires
            }
            *hold = None;
            drop(hold);
            drive
                .lock()
                .unwrap()
                .set(msg.linear.x, msg.angular.z, Instant::now());
        },
    )?;
    log_info!(
        node.logger(),
        "cmd_vel clamped to {} m/s, {} rad/s; lease {:?}",
        MAX_LINEAR,
        MAX_ANGULAR,
        lease
    );

    let _ = tf_static_pub.publish(&static_tf);
    let mut last_report = std::time::Instant::now();
    loop {
        loop {
            match rx.try_recv() {
                Ok(BridgeEvent::Scan(scan)) => {
                    // Vendor per-beam bearings -> uniform ascending grid,
                    // with range_min that marks empty (0.0) and turret
                    // beams invalid (see player_bridge::scan).
                    let grid = uniform_scan(&scan);
                    let mut ros_scan = LaserScan::default();
                    // Stamp at local receive time: the Player scan carries
                    // no portable timestamp, and an unstamped message is
                    // unusable for TF-synced consumers (SLAM drops it).
                    // The stamp then honestly reflects the data's age.
                    if let Ok(stamp) = clock.now().to_ros_msg() {
                        ros_scan.header.stamp = stamp;
                    }
                    ros_scan.header.frame_id = laser_frame.clone();
                    ros_scan.angle_min = grid.angle_min;
                    ros_scan.angle_max = grid.angle_max;
                    ros_scan.angle_increment = grid.angle_increment;
                    ros_scan.range_min = grid.range_min;
                    ros_scan.range_max = grid.range_max;
                    ros_scan.ranges = grid.ranges;
                    ros_scan.intensities = grid.intensities;
                    let _ = scan_pub.publish(&ros_scan);
                }
                Ok(BridgeEvent::Bumper(pressed)) => {
                    // The reader thread already observed the frame (edge-
                    // triggered back-off); here only the topic follows.
                    let mut msg = ros_env::std_msgs::msg::Bool::default();
                    msg.data = pressed.iter().any(|&p| p);
                    let _ = bumper_pub.publish(&msg);
                }
                Ok(BridgeEvent::Odom(state)) => {
                    let stalled = {
                        let mut sd = stall_reader.lock().unwrap();
                        sd.update(
                            *latest_cmd_reader.lock().unwrap(),
                            (state.vx, state.vyaw),
                            Instant::now(),
                        )
                    };
                    if stalled {
                        eprintln!(
                            "player_to_ros2: DRIVE STALLED — commands flow but the wheels \
                             do not turn (pushed against an obstacle?); holding /cmd_vel 10 s"
                        );
                        *stall_hold_reader.lock().unwrap() =
                            Some(Instant::now() + Duration::from_secs(10));
                        drive_ros.lock().unwrap().set(0.0, 0.0, Instant::now());
                    }
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

                    // odom -> base_link from the same odometry pose.
                    let mut tf = TFMessage::default();
                    let mut t = TransformStamped::default();
                    t.header.frame_id = odom_frame.clone();
                    t.child_frame_id = base_frame.clone();
                    t.header.stamp = odom.header.stamp;
                    t.transform = Transform {
                        translation: Vector3 {
                            x: state.x,
                            y: state.y,
                            z: 0.0,
                        },
                        rotation: yaw_to_quat(state.yaw),
                    };
                    tf.transforms.push(t);
                    let _ = tf_pub.publish(&tf);
                }
                Ok(BridgeEvent::Ir(index, state)) => {
                    // ir:0 wall, ir:1 cliff, ir:2 drop. Each element of
                    // the array becomes its own Range topic; voltages
                    // (the analog readings) ride a single array topic.
                    let (prefix, volt_pub): (
                        &str,
                        &Publisher<ros_env::std_msgs::msg::Float32MultiArray>,
                    ) = match index {
                        0 => ("/wall_ir", &volt_pub_wall),
                        1 => ("/cliff_ir", &volt_pub_cliff),
                        2 => ("/drop_ir", &volt_pub_drop),
                        _ => continue,
                    };
                    for (i, &r) in state.ranges.iter().enumerate() {
                        let topic = format!("{prefix}_{i}");
                        let pub_once = {
                            let mut map = range_pubs.lock().unwrap();
                            map.entry(topic.clone())
                                .or_insert_with(|| {
                                    range_pub_new(&topic)
                                        .expect("publisher creation cannot fail here")
                                })
                                .clone()
                        };
                        let mut header = ros_env::std_msgs::msg::Header::default();
                        header.frame_id = base_frame.clone();
                        if let Ok(stamp) = clock.now().to_ros_msg() {
                            header.stamp = stamp;
                        }
                        // Player claims metres; the vendor firmware's
                        // semantics are unproven (survey: cliff zeros may
                        // be states), so the range is passed through
                        // as-is rather than interpreted.
                        let msg = Range {
                            header,
                            radiation_type: Range::INFRARED,
                            field_of_view: 0.0,
                            min_range: 0.0,
                            max_range: 1.0,
                            range: r,
                            variance: 0.0,
                        };
                        let _ = pub_once.publish(&msg);
                    }
                    let volts = ros_env::std_msgs::msg::Float32MultiArray {
                        data: state.voltages.clone(),
                        ..Default::default()
                    };
                    let _ = volt_pub.publish(&volts);
                }
                Ok(BridgeEvent::Sonar(state)) => {
                    let mut header = ros_env::std_msgs::msg::Header::default();
                    header.frame_id = base_frame.clone();
                    if let Ok(stamp) = clock.now().to_ros_msg() {
                        header.stamp = stamp;
                    }
                    // The live frame carries one range of 65.534 m — a
                    // suspected sentinel/error, not a distance; publish
                    // it as-is and let consumers decide.
                    let msg = Range {
                        header,
                        radiation_type: Range::ULTRASOUND,
                        field_of_view: 0.0,
                        min_range: 0.0,
                        max_range: 6.0,
                        range: state.ranges.first().copied().unwrap_or(0.0),
                        variance: 0.0,
                    };
                    let _ = sonar_pub.publish(&msg);
                }
                Ok(BridgeEvent::Power(state)) => {
                    let mut header = ros_env::std_msgs::msg::Header::default();
                    header.frame_id = base_frame.clone();
                    if let Ok(stamp) = clock.now().to_ros_msg() {
                        header.stamp = stamp;
                    }
                    // PLAYER_POWER_MASK_VOLTS | WATTS | PERCENT | CHARGING
                    let msg = BatteryState {
                        header,
                        voltage: if state.valid & 1 != 0 {
                            state.volts
                        } else {
                            0.0
                        },
                        percentage: if state.valid & 8 != 0 {
                            state.percent / 100.0
                        } else {
                            0.0
                        },
                        power_supply_status: if state.valid & 16 != 0 && state.charging > 0 {
                            BatteryState::POWER_SUPPLY_STATUS_CHARGING
                        } else {
                            BatteryState::POWER_SUPPLY_STATUS_DISCHARGING
                        },
                        ..Default::default()
                    };
                    let _ = battery_pub.publish(&msg);
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
            log_info!(
                node.logger(),
                "bridge alive (player {}:{})",
                host_for_log,
                port
            );
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
