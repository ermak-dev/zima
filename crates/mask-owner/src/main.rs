//! mask-owner: hold the change-device-state mask as the sole IPC owner.
//!
//! Usage:
//!   mask-owner hold [--mask HEX] [--timeout SECS]   hold until SIGTERM/SIGINT
//!   mask-owner restore                              one-shot standby restore
//!
//! `hold` refuses to run unless it is the sole interested party: run it only
//! with the stock RoboController stopped. The guard child is forked before
//! the mask is applied and restores standby if this process dies hard.

use std::time::Duration;

use mask_owner::ipc::{guard_main, Channel, Owner};
use mask_owner::{MASK_NO_BRUSHES, MASK_STANDBY};

fn main() {
    let args: Vec<String> = std::env::args().collect();
    match args.get(1).map(String::as_str) {
        Some("--guard") => {
            let owner_pid: u32 = args
                .get(2)
                .and_then(|s| s.parse().ok())
                .expect("guard: owner pid");
            let marker = args.get(3).expect("guard: marker path");
            signal_handler::install_guard();
            guard_main(owner_pid, marker);
        }
        Some("hold") => {
            let mut mask = MASK_NO_BRUSHES;
            let mut timeout = Duration::from_secs(5);
            let mut it = args[2..].iter();
            while let Some(arg) = it.next() {
                match arg.as_str() {
                    "--mask" => {
                        mask = u32::from_str_radix(
                            it.next().expect("--mask HEX").trim_start_matches("0x"),
                            16,
                        )
                        .expect("mask must be hex");
                    }
                    "--timeout" => {
                        timeout = Duration::from_secs(
                            it.next()
                                .expect("--timeout SECS")
                                .parse()
                                .expect("timeout must be seconds"),
                        );
                    }
                    other => {
                        eprintln!("mask-owner: unknown argument {other}");
                        std::process::exit(2);
                    }
                }
            }
            let mut owner = match Owner::start(mask, timeout) {
                Ok(o) => o,
                Err(e) => {
                    eprintln!("mask-owner: hold failed: {e}");
                    std::process::exit(3);
                }
            };
            eprintln!("mask-owner: holding; send SIGTERM to stop");
            signal_handler::install();
            while !signal_handler::is_stopped() {
                std::thread::sleep(Duration::from_millis(200));
            }
            eprintln!("mask-owner: stop requested");
            match owner.restore(timeout) {
                Ok(()) => eprintln!("mask-owner: standby restored"),
                Err(e) => {
                    eprintln!("mask-owner: restore failed: {e} (guard will retry)");
                    std::process::exit(4);
                }
            }
        }
        Some("restore") => {
            let mut channel = match Channel::open() {
                Ok(c) => c,
                Err(e) => {
                    eprintln!("mask-owner: open failed: {e}");
                    std::process::exit(3);
                }
            };
            channel.drain().ok();
            match channel.exchange(
                std::process::id() & 0xffff,
                MASK_STANDBY,
                Duration::from_secs(5),
            ) {
                Ok(()) => println!("mask-owner: standby restored"),
                Err(e) => {
                    eprintln!("mask-owner: restore failed: {e}");
                    std::process::exit(4);
                }
            }
        }
        _ => {
            eprintln!("usage: mask-owner hold [--mask HEX] [--timeout SECS] | restore | --guard PID MARKER (internal)");
            std::process::exit(2);
        }
    }
}

/// Signal handling without external crates: a handler may only touch
/// atomics, so set one flag and let the main loop do the work. The guard
/// additionally detaches from the session and ignores HUP/TERM/INT: when
/// the owner runs over ssh and the channel dies (or the owner is
/// SIGKILLed), sshd sends SIGHUP to the whole session and the guard must
/// survive it to do its restore — measured live on 2026-10-03.
mod signal_handler {
    use std::sync::atomic::{AtomicBool, Ordering};

    static STOP: AtomicBool = AtomicBool::new(false);

    extern "C" fn on_stop_signal(_: i32) {
        STOP.store(true, Ordering::Relaxed);
    }

    extern "C" fn on_ignore_signal(_: i32) {}

    pub fn install() {
        unsafe {
            libc_signal(2, on_stop_signal); // SIGINT
            libc_signal(15, on_stop_signal); // SIGTERM
        }
    }

    /// Guard setup: detach from the controlling session, ignore the
    /// signals that arrive with the dead owner's session teardown, and
    /// detach stdio from the owner's inherited ssh pipe: when the owner
    /// dies the pipe closes, and the guard's first stderr write would
    /// kill it with SIGPIPE before it can restore the mask (measured
    /// live on 2026-10-03). Output goes to /tmp/mask-owner-guard.log.
    pub fn install_guard() {
        unsafe {
            libc_setsid();
            libc_signal(1, on_ignore_signal); // SIGHUP
            libc_signal(2, on_ignore_signal); // SIGINT
            libc_signal(15, on_ignore_signal); // SIGTERM
            libc_signal(13, on_ignore_signal); // SIGPIPE
            libc_detach_stdio();
        }
    }

    pub fn is_stopped() -> bool {
        STOP.load(Ordering::Relaxed)
    }

    unsafe fn libc_signal(signum: i32, handler: extern "C" fn(i32)) {
        extern "C" {
            fn signal(signum: i32, handler: extern "C" fn(i32)) -> usize;
        }
        signal(signum, handler);
    }

    unsafe fn libc_setsid() {
        extern "C" {
            fn setsid() -> i32;
        }
        setsid();
    }

    /// Redirect stdin from /dev/null and stdout/stderr to a log file:
    /// the inherited ssh pipe closes when the owner dies. If the log
    /// cannot be opened (permissions, read-only fs), fall back to
    /// /dev/null — the guard must never die over logging.
    unsafe fn libc_detach_stdio() {
        extern "C" {
            fn open(path: *const std::os::raw::c_char, flags: i32) -> i32;
            fn dup2(fd: i32, fd2: i32) -> i32;
            fn close(fd: i32) -> i32;
        }
        let devnull = std::ffi::CString::new("/dev/null").unwrap();
        let logpath = std::ffi::CString::new("/tmp/mask-owner-guard.log").unwrap();
        // O_RDONLY=0, O_WRONLY|O_CREAT|O_APPEND = 1|0o100|0o2000
        let rd = open(devnull.as_ptr(), 0);
        if rd >= 0 {
            dup2(rd, 0);
            if rd != 0 {
                close(rd);
            }
        }
        let mut wr = open(logpath.as_ptr(), 1 | 0o100 | 0o2000);
        if wr < 0 {
            wr = open(devnull.as_ptr(), 1);
        }
        if wr >= 0 {
            dup2(wr, 1);
            dup2(wr, 2);
            if wr > 2 {
                close(wr);
            }
        }
    }
}
