//! Process isolation is required: the C guard forks and owns signal handlers.
//! Never call it in the multithreaded listener itself.
use std::io::{BufRead, BufReader, Read, Write};
use std::net::TcpStream;
use std::process::{Command, Stdio};
use std::time::{Duration, Instant};

pub fn dispatch() {
    let args: Vec<String> = std::env::args().collect();
    let operation = match args.get(1).map(String::as_str) {
        Some("--audio-worker" | "play") => "play",
        Some("--audio-cancel") => "cancel",
        _ => return,
    };
    #[cfg(supervisor_audio)]
    {
        use std::ffi::CString;
        extern "C" {
            fn robot_audio_main(argc: i32, argv: *mut *mut std::os::raw::c_char) -> i32;
        }
        let strings: Vec<_> = std::iter::once(args[0].as_str())
            .chain(std::iter::once(operation))
            .chain(args.iter().skip(2).map(String::as_str))
            .map(|s| CString::new(s).unwrap())
            .collect();
        let mut argv: Vec<_> = strings.iter().map(|s| s.as_ptr() as *mut _).collect();
        argv.push(std::ptr::null_mut());
        std::process::exit(unsafe { robot_audio_main(strings.len() as i32, argv.as_mut_ptr()) });
    }
    #[cfg(not(supervisor_audio))]
    {
        let _ = (args, operation);
        println!("UNAVAILABLE");
        std::process::exit(2);
    }
}

fn token_valid(token: &str) -> bool {
    token.len() == 32
        && token
            .bytes()
            .all(|b| b.is_ascii_digit() || (b'a'..=b'f').contains(&b))
}

pub fn cancel(conn: &mut TcpStream, token: &str) {
    let result = if token_valid(token) {
        Command::new(std::env::current_exe().unwrap())
            .args(["--audio-cancel", token])
            .stdin(Stdio::null())
            .stderr(Stdio::null())
            .output()
            .ok()
            .is_some_and(|o| o.status.success() && o.stdout == b"CANCELLED\n")
    } else {
        false
    };
    let _ = conn.write_all(if result { b"CANCELLED\n" } else { b"ERROR\n" });
}

pub fn play(conn: &mut TcpStream, fields: &[&str]) {
    if !valid_header(fields) {
        let _ = conn.write_all(b"INVALID_AUDIO\n");
        return;
    }
    let len: usize = fields[3].parse().unwrap();
    let mut wav = vec![0; len];
    let deadline = Instant::now() + Duration::from_secs(5);
    let mut filled = 0;
    while filled < len {
        let remaining = deadline.saturating_duration_since(Instant::now());
        if remaining.is_zero() {
            break;
        }
        let _ = conn.set_read_timeout(Some(remaining));
        match conn.read(&mut wav[filled..]) {
            Ok(0) | Err(_) => break,
            Ok(n) => filled += n,
        }
    }
    if filled != len {
        let _ = conn.write_all(b"INVALID_AUDIO\n");
        return;
    }
    // `play` is a private alias for --audio-worker. The unchanged C cancel
    // verifier requires both `play` and the token in /proc/<guard>/cmdline.
    let mut child = match Command::new(std::env::current_exe().unwrap())
        .args(["play", fields[1], fields[2]])
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .stderr(Stdio::null())
        .spawn()
    {
        Ok(child) => child,
        Err(_) => {
            let _ = conn.write_all(b"UNAVAILABLE\n");
            return;
        }
    };
    let mut input = child.stdin.take().unwrap();
    // Feed concurrently: the C guard can reject a header without draining its
    // input; its stdout must always be drained, including after client EOF.
    let feeder = std::thread::spawn(move || {
        let _ = input.write_all(&wav);
    });
    let mut output = BufReader::new(child.stdout.take().unwrap());
    let mut connected = true;
    let mut line = String::new();
    while output.read_line(&mut line).is_ok_and(|n| n > 0) {
        if connected && conn.write_all(line.as_bytes()).is_err() {
            connected = false;
            // Ask the guard to stop and restore; never SIGKILL the guard.
            let _ = Command::new(std::env::current_exe().unwrap())
                .args(["--audio-cancel", fields[2]])
                .stdin(Stdio::null())
                .stdout(Stdio::null())
                .stderr(Stdio::null())
                .status();
        }
        line.clear();
    }
    let _ = child.wait();
    let _ = feeder.join();
}

fn valid_header(fields: &[&str]) -> bool {
    fields.len() == 4
        && fields[1]
            .parse::<f64>()
            .is_ok_and(|v| v.is_finite() && (0.0..=1.0).contains(&v))
        && token_valid(fields[2])
        && fields[3]
            .parse::<usize>()
            .is_ok_and(|n| (1..=1_000_000).contains(&n))
}
