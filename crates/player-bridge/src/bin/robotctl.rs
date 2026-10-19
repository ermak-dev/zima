//! robotctl: direct Player CLI used by bin/robot (position/fan/move/stop).
//!
//! A Rust replacement for the former bin/controller/player.c, built on the
//! same player-bridge protocol core as the ROS node — one implementation
//! of the Player wire protocol, no C client library needed.
//!
//! Movement limits mirror the old C client: 0.1 m/s, 0.4 rad/s, at most
//! 2 seconds per call, zero velocity on exit and on SIGINT/SIGTERM.

use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

use player_bridge::{Client, Error, MSGTYPE_DATA, POSITION2D_CODE, POSITION2D_DATA_STATE};

static INTERRUPTED: AtomicBool = AtomicBool::new(false);

extern "C" fn on_signal(_: i32) {
    INTERRUPTED.store(true, Ordering::Relaxed);
}

fn install_signal_handlers() {
    extern "C" {
        fn signal(signum: i32, handler: extern "C" fn(i32)) -> usize;
    }
    unsafe {
        signal(2, on_signal); // SIGINT
        signal(15, on_signal); // SIGTERM
    }
}

fn usage() -> ! {
    eprintln!(
        "usage: robotctl <host> position|fan|battery|stop|move <forward|backward|left|right> <seconds (0, 2]>"
    );
    std::process::exit(2);
}

fn parse_duration(s: &str) -> f64 {
    let d: f64 = s.parse().unwrap_or_else(|_| usage());
    if !d.is_finite() || d <= 0.0 || d > 2.0 {
        usage();
    }
    d
}

/// Wait up to `timeout` for one fresh position2d DATA message.
fn read_position(
    client: &mut Client,
    timeout: Duration,
) -> Result<player_bridge::Position2dState, Error> {
    let deadline = Instant::now() + timeout;
    loop {
        if deadline.saturating_duration_since(Instant::now()).is_zero() {
            return Err(Error::Io(std::io::Error::new(
                std::io::ErrorKind::TimedOut,
                "no fresh data in time",
            )));
        }
        let msg = client.read_message()?;
        if msg.header.msg_type == MSGTYPE_DATA
            && msg.header.addr.interf == POSITION2D_CODE
            && msg.header.subtype == POSITION2D_DATA_STATE
        {
            return player_bridge::decode_position2d(&msg.payload);
        }
    }
}

// A separate path is essential: ordinary robotctl commands subscribe position2d
// and send a stop on exit. Battery must never do either, even on errors.
fn battery(host: String, port: u16) -> i32 {
    let (tx, rx) = std::sync::mpsc::sync_channel(1);
    std::thread::spawn(move || {
        let result = (|| {
            let mut client = Client::connect(&host, port)?;
            if client.subscribe(player_bridge::POWER_CODE, 0)? != player_bridge::OPEN_MODE {
                return Err(Error::Nak);
            }
            loop {
                let msg = client.read_message()?;
                if msg.header.msg_type == MSGTYPE_DATA
                    && msg.header.addr.interf == player_bridge::POWER_CODE
                    && msg.header.addr.index == 0
                    && msg.header.subtype == player_bridge::POWER_DATA_STATE
                {
                    return player_bridge::decode_power(&msg.payload);
                }
            }
        })();
        let _ = tx.send(result);
    });
    // Bounds the whole operation, including DNS, banner and ACK. main exits the
    // process after this returns, so a stalled worker cannot keep the CLI alive.
    let state = match rx.recv_timeout(Duration::from_secs(3)) {
        Ok(Ok(state)) => Some(state),
        error => {
            eprintln!("robotctl: battery unavailable within 3 seconds: {error:?}");
            None
        }
    };
    let number = |value: Option<f32>| {
        value
            .map(|v| v.to_string())
            .unwrap_or_else(|| "null".into())
    };
    let valid = state.as_ref().map(|s| s.valid).unwrap_or(0);
    let percent = state
        .as_ref()
        .filter(|s| valid & 8 != 0 && s.percent.is_finite() && (0.0..=100.0).contains(&s.percent))
        .map(|s| s.percent);
    let voltage = state
        .as_ref()
        .filter(|s| valid & 1 != 0 && s.volts.is_finite() && s.volts > 0.0)
        .map(|s| s.volts);
    let charging = match state
        .as_ref()
        .filter(|_| valid & 16 != 0)
        .map(|s| s.charging)
    {
        Some(0) => "false",
        Some(1) => "true",
        _ => "null",
    };
    // Player's header timestamp is not evidence of sensor measurement time.
    println!("{{\"valid\":{valid},\"percent\":{},\"voltage\":{},\"charging\":{charging},\"freshness\":\"unknown\",\"measurement_time\":null}}", number(percent), number(voltage));
    if state.is_some() {
        0
    } else {
        1
    }
}

fn main() {
    let mut args: Vec<String> = std::env::args().skip(1).collect();
    if args.len() < 2 || args[0].is_empty() {
        usage();
    }
    let host = args.remove(0);
    let cmd = args.remove(0);

    let (linear, angular, duration) = match cmd.as_str() {
        "move" => {
            if args.len() != 2 || args[1].is_empty() {
                usage();
            }
            let duration = parse_duration(&args[1]);
            let (linear, angular) = match args[0].as_str() {
                "forward" => (0.1f64, 0.0f64),
                "backward" => (-0.1, 0.0),
                "left" => (0.0, 0.4),
                "right" => (0.0, -0.4),
                _ => usage(),
            };
            (linear, angular, duration)
        }
        "position" | "fan" | "battery" | "stop" => {
            if !args.is_empty() {
                usage();
            }
            (0.0, 0.0, 0.0)
        }
        _ => usage(),
    };

    install_signal_handlers();

    // Port override for local testing against a non-default Player server.
    let port: u16 = std::env::var("ROBOTCTL_PORT")
        .ok()
        .and_then(|p| p.parse().ok())
        .unwrap_or(6665);

    if cmd == "battery" {
        std::process::exit(battery(host, port));
    }

    let mut client = match Client::connect(&host, port) {
        Ok(c) => c,
        Err(e) => {
            eprintln!("robotctl: connect: {e}");
            std::process::exit(1);
        }
    };
    if let Err(e) = client.subscribe(POSITION2D_CODE, 0) {
        eprintln!("robotctl: subscribe: {e}");
        std::process::exit(1);
    }

    let exit_code = match cmd.as_str() {
        "position" => match read_position(&mut client, Duration::from_secs(3)) {
            Ok(s) => {
                println!(
                    "x={:.3} y={:.3} heading={:.3} vx={:.3} omega={:.3}",
                    s.x, s.y, s.yaw, s.vx, s.vyaw
                );
                0
            }
            Err(e) => {
                eprintln!("robotctl: no fresh odometry within 3 seconds: {e}");
                1
            }
        },
        "fan" => {
            // The IR interface (vendor fan_motor_status, index 6) is not
            // part of the player-bridge core yet. Report unavailability
            // instead of guessing: the value is a raw, uncalibrated
            // sensor anyway (see docs/player-platform-survey.md).
            eprintln!("robotctl: fan telemetry is not implemented in the Rust client yet");
            1
        }
        "stop" => 0,
        _ => {
            let deadline = Instant::now() + Duration::from_secs_f64(duration);
            while !INTERRUPTED.load(Ordering::Relaxed) && Instant::now() < deadline {
                if let Err(e) = client.send_velocity(0, linear, angular, 1) {
                    eprintln!("robotctl: move: {e}");
                    break;
                }
                std::thread::sleep(Duration::from_millis(100));
            }
            if INTERRUPTED.load(Ordering::Relaxed) {
                1
            } else {
                0
            }
        }
    };

    // Contract from the old C client: always leave the robot stopped.
    if let Err(e) = client.send_velocity(0, 0.0, 0.0, 1) {
        eprintln!("robotctl: WARNING: failed to send zero velocity; stop robot locally: {e}");
        std::process::exit(1);
    }
    std::process::exit(exit_code);
}
