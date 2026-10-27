//! BUG: transport delay and queue batching replace Player measurement intervals.
//! HYPOTHESIS: the reader discards timestamps and the ROS loop stamps on dequeue.
//! SOLUTION: pass reader-mapped measurement times through scan, odom and TF.
//! RESULT: FIXED — timing and stationary-wall checks pass against a local Player fake.
//! STATUS: FIXED

#[test]
fn player_measurement_times_survive_transport_and_batching() {
    let status = std::process::Command::new("python3")
        .arg(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/tests/support/timing_probe.py"
        ))
        .arg(env!("CARGO_BIN_EXE_player_to_ros2"))
        .env("ROS_DOMAIN_ID", (200 + std::process::id() % 30).to_string())
        .env("ROS_AUTOMATIC_DISCOVERY_RANGE", "LOCALHOST")
        .env_remove("ROS_LOCALHOST_ONLY")
        .env_remove("ROS_STATIC_PEERS")
        .env_remove("ROBOT_BRIDGE_HEARTBEAT")
        .status()
        .expect("run local Player/ROS timing probe (requires sourced ROS environment)");
    assert!(
        status.success(),
        "Player/ROS timestamp pipeline assertions failed"
    );
}
