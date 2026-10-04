//! CLI tests for robotctl: argument validation, limits, and the zero
//! velocity on exit contract. Network interaction is not tested here —
//! against_server.rs covers the protocol; these tests exit before
//! connecting by feeding invalid arguments.

use std::process::Command;

const BIN: &str = env!("CARGO_BIN_EXE_robotctl");

fn run(args: &[&str]) -> (i32, String) {
    let out = Command::new(BIN).args(args).output().expect("spawn robotctl");
    (out.status.code().unwrap_or(-1), String::from_utf8_lossy(&out.stderr).into_owned())
}

#[test]
fn rejects_invalid_usage_before_connecting() {
    for args in [
        vec!["host"],
        vec!["host", "sideways", "1"],
        vec!["host", "move", "sideways", "1"],
        vec!["host", "move", "forward", "0"],
        vec!["host", "move", "forward", "4"],
        vec!["host", "move", "forward", "nan"],
        vec!["host", "move", "forward", "-1"],
        vec!["host", "move", "forward"],
        vec!["host", "move", "forward", "1", "extra"],
        vec!["host", "position", "extra"],
        vec!["host", "unknown"],
        vec![],
    ] {
        let (code, stderr) = run(&args);
        assert_ne!(code, 0, "expected failure for {args:?}");
        assert!(!stderr.is_empty(), "expected a usage message for {args:?}");
    }
}

#[test]
fn move_directions_are_within_limits() {
    // Valid directions must parse past validation and die in the connect
    // stage against a port nothing listens on (immediate refusal, unlike
    // an unroutable TEST-NET address that waits for a timeout).
    for dir in ["forward", "backward", "left", "right"] {
        let (code, stderr) = run(&["127.0.0.1", "move", dir, "1"]);
        assert!(stderr.contains("connect") || stderr.contains("io"), "dir {dir}: {stderr}");
        assert_eq!(code, 1, "dir {dir}");
    }
}
