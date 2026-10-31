use crate::{audio, Failure};
use serde_json::{json, Value};
use std::time::Duration;
use tokio::{
    io::{AsyncReadExt, AsyncWriteExt, BufReader},
    net::TcpStream,
    sync::watch,
    task::JoinHandle,
    time::timeout,
};

const FRESHNESS: Duration = Duration::from_secs(1);

pub struct CompassTask(JoinHandle<()>);
impl Drop for CompassTask {
    fn drop(&mut self) {
        self.0.abort();
    }
}

fn unknown(status: &str) -> Value {
    json!({"source":"st480-evdev", "calibrated":false, "frame":"sensor",
        "x":null, "y":null, "z":null, "yaw":null, "stamp":null, "status":status})
}
fn publish(states: &watch::Sender<crate::state::State>, sample: Value) {
    states.send_modify(|state| state.set_compass(sample));
}
fn sample(line: &str) -> Option<Value> {
    let mut fields = line.split_whitespace();
    if fields.next()? != "COMPASS" {
        return None;
    }
    let stamp: f64 = fields.next()?.parse().ok()?;
    let x: i32 = fields.next()?.parse().ok()?;
    let y: i32 = fields.next()?.parse().ok()?;
    let z: i32 = fields.next()?.parse().ok()?;
    if fields.next().is_some() || !stamp.is_finite() || stamp <= 0. {
        return None;
    }
    let valid = x != 0 || y != 0;
    // This is the horizontal field angle in sensor axes, not calibrated robot yaw or north.
    Some(
        json!({"source":"st480-evdev", "calibrated":false, "frame":"sensor",
        "x":x, "y":y, "z":z, "stamp":stamp,
        "yaw": if valid { Some((y as f64).atan2(x as f64)) } else { None },
        "status": if valid { "uncalibrated" } else { "invalid" }}),
    )
}
async fn line(reader: &mut BufReader<TcpStream>) -> Result<String, Failure> {
    let mut bytes = Vec::new();
    loop {
        let byte = reader.read_u8().await?;
        if byte == b'\n' {
            return Ok(String::from_utf8(bytes)?);
        }
        if bytes.len() >= 256 {
            return Err("compass response too long".into());
        }
        bytes.push(byte);
    }
}
async fn receive(
    address: &str,
    token: &str,
    states: &watch::Sender<crate::state::State>,
) -> Result<&'static str, Failure> {
    let mut stream = timeout(Duration::from_secs(5), TcpStream::connect(address)).await??;
    audio::authenticate(&mut stream, token).await?;
    timeout(FRESHNESS, stream.write_all(b"COMPASS\n")).await??;
    let mut reader = BufReader::new(stream);
    let mut last_stamp = 0.;
    let mut expires = tokio::time::Instant::now() + FRESHNESS;
    loop {
        let line = match tokio::time::timeout_at(expires, line(&mut reader)).await {
            Err(_) => return Ok("stale"),
            Ok(Err(_)) => return Ok("invalid"),
            Ok(Ok(line)) => line,
        };
        match line.as_str() {
            "REFUSED" => return Ok("unsupported"),
            "COMPASS_UNAVAILABLE" => return Ok("unavailable"),
            _ => {}
        }
        let Some(sample) = sample(&line) else {
            return Ok("invalid");
        };
        let stamp = sample["stamp"].as_f64().unwrap();
        if stamp <= last_stamp {
            continue;
        }
        last_stamp = stamp;
        expires = tokio::time::Instant::now() + FRESHNESS;
        publish(states, sample);
    }
}
pub fn start(states: watch::Sender<crate::state::State>) -> Option<CompassTask> {
    let address = std::env::var("ROBOT_SUPERVISOR_ADDR").ok()?;
    let token = std::env::var("ROBOT_SUPERVISOR_TOKEN").unwrap_or_default();
    Some(CompassTask(tokio::spawn(async move {
        loop {
            publish(&states, unknown("connecting"));
            let status = receive(&address, &token, &states)
                .await
                .unwrap_or("unavailable");
            publish(&states, unknown(status));
            if status == "unsupported" {
                return;
            }
            tokio::time::sleep(Duration::from_secs(2)).await;
        }
    })))
}
