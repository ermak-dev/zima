//! FEATURE: bounded read-only power:0 JSON; missing validity never means zero.
//! SPEC: Player masks VOLTS=1, PERCENT=8, CHARGING=16; no sensor age evidence.
use player_bridge::*;
use std::io::{Read, Write};
use std::net::TcpListener;
use std::process::Command;
use std::time::{Duration, Instant};

fn run_power(valid: u32, volts: f32, percent: f32, charging: i32) -> String {
    let listener = TcpListener::bind("127.0.0.1:0").unwrap();
    let port = listener.local_addr().unwrap().port();
    let server = std::thread::spawn(move || {
        let (mut stream, _) = listener.accept().unwrap();
        stream
            .set_read_timeout(Some(Duration::from_secs(5)))
            .unwrap();
        let mut banner = [0; 32];
        banner[..9].copy_from_slice(b"Player v.");
        stream.write_all(&banner).unwrap();
        let mut hdr = [0; MSGHDR_SIZE];
        stream.read_exact(&mut hdr).unwrap();
        let header = decode_header(&hdr).unwrap();
        assert_eq!(header.msg_type, MSGTYPE_REQ);
        let mut request = vec![0; header.size as usize];
        stream.read_exact(&mut request).unwrap();
        assert_eq!(
            u32::from_be_bytes(request[8..12].try_into().unwrap()),
            POWER_CODE as u32
        );
        assert_eq!(u32::from_be_bytes(request[12..16].try_into().unwrap()), 0);
        let send = |stream: &mut std::net::TcpStream, interf, index, kind, payload: &[u8]| {
            let hdr = MsgHeader {
                addr: DevAddr {
                    host: 0,
                    robot: 0,
                    interf,
                    index,
                },
                msg_type: kind,
                subtype: 1,
                timestamp: 123456789.0,
                seq: 0,
                size: payload.len() as u32,
            };
            stream.write_all(&encode_header(&hdr)).unwrap();
            stream.write_all(payload).unwrap();
        };
        let mut payload = valid.to_be_bytes().to_vec();
        for f in [volts, percent, 0.0, 0.0] {
            payload.extend(f.to_be_bytes());
        }
        payload.extend(charging.to_be_bytes());
        // Wrong device index must not be accepted. Initial state before ACK is preserved.
        send(&mut stream, POWER_CODE, 1, MSGTYPE_DATA, &[]);
        send(&mut stream, POWER_CODE, 0, MSGTYPE_DATA, &payload);
        let mut ack = [0; 28];
        ack[19] = OPEN_MODE;
        send(&mut stream, PLAYER_CODE, 0, MSGTYPE_RESP_ACK, &ack);
        let mut rest = Vec::new();
        stream.read_to_end(&mut rest).unwrap();
        assert!(
            rest.is_empty(),
            "read-only battery emitted extra wire commands: {rest:?}"
        );
    });
    let out = Command::new(env!("CARGO_BIN_EXE_robotctl"))
        .args(["127.0.0.1", "battery"])
        .env("ROBOTCTL_PORT", port.to_string())
        .output()
        .unwrap();
    assert!(
        out.status.success(),
        "{}",
        String::from_utf8_lossy(&out.stderr)
    );
    server.join().unwrap();
    let text = String::from_utf8(out.stdout).unwrap();
    assert!(text.len() < 512);
    text
}

#[test]
fn battery_validity_and_ranges() {
    let text = run_power(25, 19.5, 42.0, 1);
    assert!(text.contains("\"percent\":42"), "{text}");
    assert!(text.contains("\"voltage\":19.5"));
    assert!(text.contains("\"charging\":true"));
    assert!(text.contains("\"valid\":25"));
    assert!(text.contains("\"freshness\":\"unknown\""));
    assert!(text.contains("\"measurement_time\":null"));
    for (valid, volts, percent, charging) in [
        (0, 19.5, 42.0, 0),
        (25, f32::NAN, f32::INFINITY, -1),
        (25, -1.0, 101.0, 2),
    ] {
        let text = run_power(valid, volts, percent, charging);
        for key in ["percent", "voltage", "charging"] {
            assert!(text.contains(&format!("\"{key}\":null")), "{text}");
        }
    }
    for percent in [0.0, 100.0] {
        let text = run_power(24, 0.0, percent, 0);
        assert!(text.contains(&format!("\"percent\":{percent}")));
        assert!(text.contains("\"charging\":false"));
    }
}

#[test]
fn battery_has_total_deadline_even_before_banner() {
    let listener = TcpListener::bind("127.0.0.1:0").unwrap();
    let start = Instant::now();
    let out = Command::new(env!("CARGO_BIN_EXE_robotctl"))
        .args(["127.0.0.1", "battery"])
        .env(
            "ROBOTCTL_PORT",
            listener.local_addr().unwrap().port().to_string(),
        )
        .output()
        .unwrap();
    assert_eq!(out.status.code(), Some(1));
    assert!(start.elapsed() < Duration::from_secs(4));
    let text = String::from_utf8(out.stdout).unwrap();
    assert!(text.contains("\"percent\":null"), "{text}");
}
