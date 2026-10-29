//! Read-only ST480 evdev observer. No mask ownership, device writes or grab.
//! Linux's native input_event uses two native C longs (timeval on ARM32
//! and host64), followed by u16 type/code and i32 value. No clock is changed:
//! timestamps are the SYN_REPORT's kernel event time, not asserted to be UTC.
use std::fs::{File, OpenOptions};
use std::io::{self, Read, Write};
use std::net::TcpStream;
use std::os::fd::{AsRawFd, RawFd};
use std::os::raw::{c_int, c_long, c_short, c_ulong};
use std::os::unix::fs::OpenOptionsExt;
use std::path::{Path, PathBuf};
use std::time::{Duration, Instant};

const LONG: usize = std::mem::size_of::<c_long>();
pub const NATIVE_EVENT_SIZE: usize = 2 * LONG + 8;
const AXES: [u16; 3] = [0x10, 0x11, 0x0a];
const PERIOD: Duration = Duration::from_millis(100);
const O_NONBLOCK: c_int = 0o4000;
const POLLIN: c_short = 1;
#[repr(C)]
struct PollFd {
    fd: c_int,
    events: c_short,
    revents: c_short,
}
unsafe extern "C" {
    fn poll(fds: *mut PollFd, nfds: c_ulong, timeout: c_int) -> c_int;
    fn ioctl(fd: c_int, request: c_ulong, ...) -> c_int;
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Event {
    pub sec: c_long,
    pub usec: c_long,
    pub kind: u16,
    pub code: u16,
    pub value: i32,
}
impl Event {
    pub fn decode(bytes: &[u8]) -> io::Result<Self> {
        if bytes.len() != NATIVE_EVENT_SIZE {
            return Err(io::ErrorKind::InvalidData.into());
        }
        Ok(Self {
            sec: c_long::from_ne_bytes(bytes[..LONG].try_into().unwrap()),
            usec: c_long::from_ne_bytes(bytes[LONG..2 * LONG].try_into().unwrap()),
            kind: u16::from_ne_bytes(bytes[2 * LONG..2 * LONG + 2].try_into().unwrap()),
            code: u16::from_ne_bytes(bytes[2 * LONG + 2..2 * LONG + 4].try_into().unwrap()),
            value: i32::from_ne_bytes(bytes[2 * LONG + 4..].try_into().unwrap()),
        })
    }
}
pub struct Frame {
    sec: c_long,
    usec: c_long,
    pub xyz: [i32; 3],
}
impl Frame {
    pub fn line(&self) -> String {
        format!(
            "COMPASS {}.{:06} {} {} {}\n",
            self.sec, self.usec, self.xyz[0], self.xyz[1], self.xyz[2]
        )
    }
}
pub struct Decoder {
    xyz: Option<[i32; 3]>,
}
impl Decoder {
    pub fn new(xyz: [i32; 3]) -> Self {
        Self { xyz: Some(xyz) }
    }
    pub fn push(
        &mut self,
        e: Event,
        axes: &mut impl FnMut() -> io::Result<[i32; 3]>,
    ) -> io::Result<Option<Frame>> {
        if e.kind == 3 && e.code == 0x18 && matches!(e.value, 1 | 2) {
            self.xyz = None;
            return Err(io::ErrorKind::InvalidData.into());
        }
        if e.kind == 0 && e.code == 3 {
            self.xyz = None;
            return Ok(None);
        }
        if e.kind == 0 && e.code == 0 {
            let Some(xyz) = self.xyz else {
                // Ignore everything through the report following SYN_DROPPED,
                // then rebuild all axes, including values suppressed by evdev.
                self.xyz = Some(axes()?);
                return Ok(None);
            };
            if !(0..1_000_000).contains(&e.usec) {
                return Err(io::ErrorKind::InvalidData.into());
            }
            return Ok(Some(Frame {
                sec: e.sec,
                usec: e.usec,
                xyz,
            }));
        }
        if e.kind == 3 {
            if let (Some(xyz), Some(i)) = (&mut self.xyz, AXES.iter().position(|&c| c == e.code)) {
                xyz[i] = e.value;
            }
        }
        Ok(None)
    }
}

/// Minimal evdev seam for offline fakes; axes must be queried before any frame.
pub trait Source {
    fn fd(&self) -> RawFd;
    fn axes(&mut self) -> io::Result<[i32; 3]>;
    fn next_event(&mut self) -> io::Result<Option<Event>>;
}
struct Device {
    file: File,
    bytes: [u8; NATIVE_EVENT_SIZE],
    used: usize,
}
// Linux _IOR on the supported ARMv7 and x86_64 ABIs. Only read-only NAME/ABS.
fn ior(number: u16, size: usize) -> c_ulong {
    (0x8000_0000u32 | ((size as u32) << 16) | ((b'E' as u32) << 8) | number as u32) as c_ulong
}
impl Device {
    fn open(path: &Path) -> io::Result<Self> {
        let file = OpenOptions::new()
            .read(true)
            .custom_flags(O_NONBLOCK)
            .open(path)?;
        let mut name = [0u8; 256];
        let n = unsafe { ioctl(file.as_raw_fd(), ior(0x06, name.len()), name.as_mut_ptr()) };
        if n < 0 {
            return Err(io::Error::last_os_error());
        }
        let end = name.iter().position(|&b| b == 0).unwrap_or(name.len());
        if &name[..end] != b"compass" {
            return Err(io::ErrorKind::InvalidData.into());
        }
        Ok(Self {
            file,
            bytes: [0; NATIVE_EVENT_SIZE],
            used: 0,
        })
    }
    fn discover() -> io::Result<Self> {
        // An explicit fixture path still undergoes NAME verification and ABS
        // initialization: a regular file/FIFO is never accepted as evdev.
        if let Some(path) = std::env::var_os("ROBOT_COMPASS_DEVICE") {
            return Self::open(Path::new(&path));
        }
        if let Ok(device) = Self::open(Path::new("/dev/input/event3")) {
            return Ok(device);
        }
        for entry in std::fs::read_dir("/sys/class/input")? {
            let entry = entry?;
            let name = entry.file_name();
            if !name.to_string_lossy().starts_with("event") {
                continue;
            }
            if std::fs::read_to_string(entry.path().join("device/name"))
                .is_ok_and(|s| s.trim_end() == "compass")
            {
                if let Ok(device) = Self::open(&PathBuf::from("/dev/input").join(name)) {
                    return Ok(device);
                }
            }
        }
        Err(io::ErrorKind::NotFound.into())
    }
}
impl Source for Device {
    fn fd(&self) -> RawFd {
        self.file.as_raw_fd()
    }
    fn axes(&mut self) -> io::Result<[i32; 3]> {
        let mut xyz = [0; 3];
        for (i, code) in AXES.iter().enumerate() {
            // struct input_absinfo consists of six i32 fields; only value is
            // used. EVIOCGABS does not modify the sensor or enable polling.
            let mut info = [0i32; 6];
            if unsafe {
                ioctl(
                    self.fd(),
                    ior(0x40 + code, std::mem::size_of_val(&info)),
                    info.as_mut_ptr(),
                )
            } < 0
            {
                return Err(io::Error::last_os_error());
            }
            xyz[i] = info[0];
        }
        Ok(xyz)
    }
    fn next_event(&mut self) -> io::Result<Option<Event>> {
        loop {
            match self.file.read(&mut self.bytes[self.used..]) {
                Ok(0) => return Err(io::ErrorKind::UnexpectedEof.into()),
                Ok(n) => {
                    self.used += n;
                    if self.used == NATIVE_EVENT_SIZE {
                        self.used = 0;
                        return Event::decode(&self.bytes).map(Some);
                    }
                }
                Err(e) if e.kind() == io::ErrorKind::WouldBlock => return Ok(None),
                Err(e) if e.kind() == io::ErrorKind::Interrupted => continue,
                Err(e) => return Err(e),
            }
        }
    }
}

pub fn observe(conn: &mut TcpStream) {
    match Device::discover() {
        Ok(mut device) => stream(conn, &mut device),
        Err(_) => {
            let _ = conn.write_all(b"COMPASS_UNAVAILABLE\n");
        }
    }
}
pub fn stream(conn: &mut TcpStream, source: &mut impl Source) {
    // Keep writes bounded even when this seam is used outside the supervisor.
    if conn
        .set_write_timeout(Some(Duration::from_secs(2)))
        .is_err()
    {
        return;
    }
    if run(conn, source).is_err() {
        let _ = conn.write_all(b"COMPASS_UNAVAILABLE\n");
    }
}
fn run(conn: &mut TcpStream, source: &mut impl Source) -> io::Result<()> {
    let mut decoder = Decoder::new(source.axes()?);
    let mut last: Option<Instant> = None;
    loop {
        let mut fds = [
            PollFd {
                fd: conn.as_raw_fd(),
                events: POLLIN,
                revents: 0,
            },
            PollFd {
                fd: source.fd(),
                events: POLLIN,
                revents: 0,
            },
        ];
        let rc = unsafe { poll(fds.as_mut_ptr(), fds.len() as c_ulong, 200) };
        if rc < 0 {
            let e = io::Error::last_os_error();
            if e.kind() == io::ErrorKind::Interrupted {
                continue;
            }
            return Err(e);
        }
        // No further inbound messages belong to an observer. POLLIN includes
        // EOF, so even a silent sensor releases its handler on disconnect.
        if fds[0].revents != 0 {
            return Ok(());
        }
        if fds[1].revents & !POLLIN != 0 {
            return Err(io::ErrorKind::UnexpectedEof.into());
        }
        if fds[1].revents & POLLIN == 0 {
            continue;
        }
        // Bound draining too, so a flooded device cannot starve disconnect.
        for _ in 0..64 {
            let Some(e) = source.next_event()? else {
                break;
            };
            if let Some(frame) = decoder.push(e, &mut || source.axes())? {
                if last.is_none_or(|t| t.elapsed() >= PERIOD) {
                    conn.write_all(frame.line().as_bytes())?;
                    last = Some(Instant::now());
                }
            }
        }
    }
}
