//! The two ways a fix arrives: a serial port, or gpsd on TCP.
//!
//! Both end at the same parser. gpsd frames NMEA in JSON and answers a
//! `?WATCH` with a stream of `TPV` objects, which carry the same numbers with
//! names instead of positions in a comma list, so the JSON path is read
//! directly rather than asked for raw NMEA: `?WATCH={"nmea":true}` exists, but
//! a daemon that has already parsed the sentences and merged the
//! constellations is doing the work better than this crate would.
//!
//! A source is a thread and a channel. It reconnects on its own, because a
//! GPS on USB disappears when the cable moves and a survey should survive
//! that: what the consumer sees is fixes stopping and starting, never an
//! error it has to handle.

use crate::nmea::{Assembler, Fix};
use std::io::{BufRead, BufReader, Read, Write};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Duration;

/// Where the fixes come from.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Transport {
    /// gpsd, as `host:port`. The daemon's own default is 2947.
    Gpsd(String),
    /// A serial port carrying NMEA, and the rate it runs at.
    Serial { path: String, baud: u32 },
}

impl Transport {
    /// Read a transport as an operator writes one: `gpsd:host:port`,
    /// `/dev/ttyACM0`, or `/dev/ttyUSB0@4800`.
    ///
    /// A bare path is a serial port and anything else is gpsd, because a path
    /// is the thing that cannot be mistaken for something else. NMEA's
    /// original rate is 4800 and every USB module since has shipped at 9600,
    /// so that is the default and the `@` is for the ones that did not.
    pub fn parse(s: &str) -> Option<Self> {
        let s = s.trim();
        if s.is_empty() {
            return None;
        }
        if let Some(rest) = s.strip_prefix("gpsd:") {
            let host = if rest.contains(':') { rest.to_string() } else { format!("{rest}:2947") };
            return Some(Self::Gpsd(host));
        }
        if s.starts_with('/') || s.starts_with("COM") {
            let (path, baud) = match s.split_once('@') {
                Some((p, b)) => (p.to_string(), b.parse().ok()?),
                None => (s.to_string(), 9_600),
            };
            return Some(Self::Serial { path, baud });
        }
        Some(Self::Gpsd(if s.contains(':') { s.to_string() } else { format!("{s}:2947") }))
    }
}

impl std::fmt::Display for Transport {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Gpsd(h) => write!(f, "gpsd:{h}"),
            Self::Serial { path, baud } => write!(f, "{path}@{baud}"),
        }
    }
}

#[derive(Clone, Debug)]
pub struct Config {
    pub transport: Transport,
    /// How long to wait before trying again after the link drops.
    pub retry: Duration,
    /// A fix older than this is stale: the GPS is talking but has lost the
    /// sky, and a survey should stop attributing sightings to the last place
    /// it saw rather than carry a position under a bridge for ten minutes.
    pub max_age: Duration,
}

impl Config {
    pub fn new(transport: Transport) -> Self {
        Self { transport, retry: Duration::from_secs(5), max_age: Duration::from_secs(10) }
    }
}

/// A running GPS reader.
///
/// Holds the most recent fix rather than a queue of them. A consumer asks
/// where the receiver is when it needs to know, and a fix nobody read before
/// the next one arrived is not a loss: it is the same place.
pub struct Source {
    cfg: Config,
    state: Arc<Mutex<Option<(Fix, std::time::Instant)>>>,
    connected: Arc<AtomicBool>,
    stop: Arc<AtomicBool>,
    fixes: Arc<std::sync::atomic::AtomicU64>,
}

impl Source {
    /// Start reading. The thread runs until the source is dropped.
    pub fn start(cfg: Config) -> Self {
        let state = Arc::new(Mutex::new(None));
        let connected = Arc::new(AtomicBool::new(false));
        let stop = Arc::new(AtomicBool::new(false));
        let fixes = Arc::new(std::sync::atomic::AtomicU64::new(0));
        let s = Self {
            cfg: cfg.clone(),
            state: state.clone(),
            connected: connected.clone(),
            stop: stop.clone(),
            fixes: fixes.clone(),
        };
        std::thread::Builder::new()
            .name("gps".into())
            .spawn(move || run(cfg, state, connected, stop, fixes))
            .ok();
        s
    }

    pub fn transport(&self) -> &Transport {
        &self.cfg.transport
    }

    /// Whether the link is up, which is not the same as having a fix: a GPS
    /// indoors is connected and lost.
    pub fn connected(&self) -> bool {
        self.connected.load(Ordering::Relaxed)
    }

    pub fn fixes(&self) -> u64 {
        self.fixes.load(Ordering::Relaxed)
    }

    /// The current position, or `None` when there has never been one or the
    /// last one has gone stale.
    pub fn fix(&self) -> Option<Fix> {
        let held = *self.state.lock().ok()?;
        let (fix, at) = held?;
        (at.elapsed() < self.cfg.max_age).then_some(fix)
    }
}

impl Drop for Source {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
    }
}

fn run(
    cfg: Config,
    state: Arc<Mutex<Option<(Fix, std::time::Instant)>>>,
    connected: Arc<AtomicBool>,
    stop: Arc<AtomicBool>,
    fixes: Arc<std::sync::atomic::AtomicU64>,
) {
    while !stop.load(Ordering::Relaxed) {
        let opened: std::io::Result<Box<dyn Read + Send>> = match &cfg.transport {
            Transport::Gpsd(addr) => open_gpsd(addr),
            Transport::Serial { path, baud } => open_serial(path, *baud),
        };
        match opened {
            Ok(stream) => {
                connected.store(true, Ordering::Relaxed);
                read_stream(stream, &state, &stop, &fixes);
                connected.store(false, Ordering::Relaxed);
            }
            Err(e) => {
                tracing::debug!("gps {}: {e}", cfg.transport);
            }
        }
        // A disconnect and a failed connect wait the same: the cable is out,
        // or the daemon is not up yet, and neither is worth a busy loop.
        let until = std::time::Instant::now() + cfg.retry;
        while !stop.load(Ordering::Relaxed) && std::time::Instant::now() < until {
            std::thread::sleep(Duration::from_millis(100));
        }
    }
}

fn read_stream(
    stream: Box<dyn Read + Send>,
    state: &Arc<Mutex<Option<(Fix, std::time::Instant)>>>,
    stop: &Arc<AtomicBool>,
    fixes: &Arc<std::sync::atomic::AtomicU64>,
) {
    let mut asm = Assembler::new();
    let mut lines = BufReader::new(stream).lines();
    while let Some(Ok(line)) = lines.next() {
        if stop.load(Ordering::Relaxed) {
            return;
        }
        let fix = if line.starts_with('{') { parse_tpv(&line) } else { asm.push(&line) };
        if let Some(f) = fix {
            fixes.fetch_add(1, Ordering::Relaxed);
            if let Ok(mut g) = state.lock() {
                *g = Some((f, std::time::Instant::now()));
            }
        }
    }
}

/// gpsd's `TPV` object, which is one fix with names on the fields.
///
/// `mode` is 0 for no fix, 1 for no position, 2 for a two-dimensional fix and
/// 3 for one with altitude. Anything under 2 has no position in it, and gpsd
/// sends those continuously while the receiver is searching.
fn parse_tpv(line: &str) -> Option<Fix> {
    let v: serde_json::Value = serde_json::from_str(line).ok()?;
    if v.get("class")?.as_str()? != "TPV" {
        return None;
    }
    if v.get("mode").and_then(|m| m.as_u64()).unwrap_or(0) < 2 {
        return None;
    }
    let num = |k: &str| v.get(k).and_then(|x| x.as_f64());
    Some(Fix {
        lat: num("lat")?,
        lon: num("lon")?,
        alt_m: num("altMSL").or_else(|| num("alt")),
        speed_ms: num("speed"),
        track_deg: num("track"),
        // gpsd reports the count in a separate SKY object and the error
        // estimate here in metres, so neither maps onto HDOP without
        // inventing a conversion. What is missing stays missing.
        sats: None,
        hdop: None,
        utc: v.get("time").and_then(|t| t.as_str()).and_then(iso8601),
    })
}

/// gpsd timestamps are ISO 8601 in UTC, `2026-09-07T09:42:50.000Z`.
fn iso8601(s: &str) -> Option<u64> {
    let (date, rest) = s.split_once('T')?;
    let time = rest.split(['.', 'Z']).next()?;
    let mut d = date.split('-');
    let (y, m, day) = (d.next()?, d.next()?, d.next()?);
    let mut t = time.split(':');
    let (h, min, sec) = (t.next()?, t.next()?, t.next()?);
    crate::nmea::utc_from_parts(
        y.parse().ok()?,
        m.parse().ok()?,
        day.parse().ok()?,
        h.parse().ok()?,
        min.parse().ok()?,
        sec.parse().ok()?,
    )
}

fn open_gpsd(addr: &str) -> std::io::Result<Box<dyn Read + Send>> {
    let mut sock = std::net::TcpStream::connect(addr)?;
    sock.set_read_timeout(Some(Duration::from_secs(30)))?;
    // Without a watch gpsd says hello and then nothing at all.
    sock.write_all(b"?WATCH={\"enable\":true,\"json\":true};\n")?;
    Ok(Box::new(sock))
}

/// Open a serial port and put it in the shape NMEA arrives in: eight bits, no
/// parity, one stop bit, no flow control, and raw so that a line is a line.
///
/// The termios call is what makes this a serial port rather than a file. A
/// USB CDC device ignores the baud rate and works either way, which is why
/// leaving it out appears to work; a real UART on a header does not, and
/// comes back as line noise that fails every checksum.
fn open_serial(path: &str, baud: u32) -> std::io::Result<Box<dyn Read + Send>> {
    use std::os::unix::io::AsRawFd;
    let file = std::fs::OpenOptions::new().read(true).write(true).open(path)?;
    let fd = file.as_raw_fd();
    let speed = match baud {
        4_800 => libc::B4800,
        9_600 => libc::B9600,
        19_200 => libc::B19200,
        38_400 => libc::B38400,
        57_600 => libc::B57600,
        115_200 => libc::B115200,
        other => {
            return Err(std::io::Error::new(
                std::io::ErrorKind::InvalidInput,
                format!("unsupported baud rate {other}"),
            ))
        }
    };
    // SAFETY: `fd` is open for the lifetime of `file`, and `tty` is a valid
    // termios the calls only ever fill in or read.
    unsafe {
        let mut tty: libc::termios = std::mem::zeroed();
        if libc::tcgetattr(fd, &mut tty) != 0 {
            return Err(std::io::Error::last_os_error());
        }
        libc::cfmakeraw(&mut tty);
        libc::cfsetispeed(&mut tty, speed);
        libc::cfsetospeed(&mut tty, speed);
        tty.c_cflag |= libc::CLOCAL | libc::CREAD;
        tty.c_cflag &= !libc::CRTSCTS;
        // Block until at least one byte, with a ten second idle timeout, so a
        // silent port ends the read rather than hanging the thread forever.
        tty.c_cc[libc::VMIN] = 0;
        tty.c_cc[libc::VTIME] = 100;
        if libc::tcsetattr(fd, libc::TCSANOW, &tty) != 0 {
            return Err(std::io::Error::last_os_error());
        }
    }
    Ok(Box::new(file))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_transport_is_read_the_way_an_operator_writes_one() {
        assert_eq!(
            Transport::parse("/dev/ttyACM0"),
            Some(Transport::Serial { path: "/dev/ttyACM0".into(), baud: 9_600 })
        );
        assert_eq!(
            Transport::parse("/dev/ttyUSB0@4800"),
            Some(Transport::Serial { path: "/dev/ttyUSB0".into(), baud: 4_800 })
        );
        assert_eq!(Transport::parse("gpsd:localhost"), Some(Transport::Gpsd("localhost:2947".into())));
        assert_eq!(Transport::parse("127.0.0.1:2947"), Some(Transport::Gpsd("127.0.0.1:2947".into())));
        assert_eq!(Transport::parse(""), None);
    }

    /// gpsd's own wire format, from a daemon watching a u-blox.
    #[test]
    fn a_tpv_object_is_a_fix() {
        let line = r#"{"class":"TPV","device":"/dev/ttyACM0","mode":3,"time":"2026-09-07T09:42:50.000Z","lat":53.636900,"lon":-6.652800,"altMSL":42.5,"speed":13.2,"track":271.4}"#;
        let f = parse_tpv(line).expect("a fix");
        assert!((f.lat - 53.6369).abs() < 1e-6);
        assert!((f.lon + 6.6528).abs() < 1e-6);
        assert_eq!(f.alt_m, Some(42.5));
        assert_eq!(f.utc, Some(1_788_774_170));
    }

    /// A receiver still searching reports mode 1 with no position, once a
    /// second, for as long as it takes.
    #[test]
    fn a_tpv_without_a_fix_is_not_one() {
        let line = r#"{"class":"TPV","device":"/dev/ttyACM0","mode":1,"time":"2026-09-07T09:42:50.000Z"}"#;
        assert!(parse_tpv(line).is_none());
        // And neither is anything else gpsd sends on the same socket.
        assert!(parse_tpv(r#"{"class":"SKY","device":"/dev/ttyACM0","satellites":[]}"#).is_none());
    }

    /// A survey under a bridge should stop recording positions, not keep
    /// attributing sightings to the last place the sky was visible.
    #[test]
    fn a_fix_goes_stale() {
        let s = Source {
            cfg: Config { max_age: Duration::from_millis(30), ..Config::new(Transport::Gpsd("x:1".into())) },
            state: Arc::new(Mutex::new(Some((Fix { lat: 1.0, lon: 2.0, ..Default::default() }, std::time::Instant::now())))),
            connected: Arc::new(AtomicBool::new(true)),
            stop: Arc::new(AtomicBool::new(true)),
            fixes: Arc::new(std::sync::atomic::AtomicU64::new(1)),
        };
        assert!(s.fix().is_some());
        std::thread::sleep(Duration::from_millis(50));
        assert!(s.fix().is_none(), "a fix older than max_age is not a position");
    }
}
