//! A sub-ghz-modem as a GPS receiver.
//!
//! The modem is a LoRa/FSK board driven over a framed binary TLV, and the
//! boards that carry a GNSS receiver (LilyGO T-Beam, T-Echo) will stream every
//! NMEA sentence to the host in its own frame once asked. That makes it a
//! third transport for this crate: the frames come off the same serial port
//! the other one reads, and what is inside them is the same NMEA the parser
//! already handles, so the de-framer is a `Read` that turns frames back into
//! lines and nothing above it changes.
//!
//! The wire format is `src/proto.h` in the sub-ghz-modem tree:
//!
//! ```text
//! A5 5A | TYPE u8 | LEN u16 | VALUE[LEN] | CRC16 u16
//! ```
//!
//! CRC16-CCITT over type, length and value, little endian throughout. Config
//! and info values are nested `ID u8 | LEN u8 | VALUE` TLVs.
//!
//! Only the three messages a GPS needs are here: `GET_INFO` to find out
//! whether a port has a modem on it and whether that modem has a receiver,
//! `GPS` to start the feed, and `NMEA` to read it. Transmitting is the
//! modem's own business.

use crate::source::Transport;
use std::io::{Read, Write};
use std::time::{Duration, Instant};

const SOF: [u8; 2] = [0xA5, 0x5A];
/// `PROTO_MAX_VALUE`. A frame claiming more than this is a resync artefact,
/// not a frame, and is dropped without waiting for the bytes.
const MAX_VALUE: usize = 512;
/// Two magic bytes, a type, a length and a CRC.
const OVERHEAD: usize = 7;

const MSG_GET_INFO: u8 = 0x02;
const MSG_GPS: u8 = 0x11;
const MSG_INFO: u8 = 0x83;
const MSG_NMEA: u8 = 0x8B;

const I_FW: u8 = 0x01;
const I_BOARD: u8 = 0x02;
const I_RADIO: u8 = 0x03;
const I_BATT_MV: u8 = 0x05;
const I_GPS: u8 = 0x08;
const I_BLE_NAME: u8 = 0x0A;

/// The rate every board's host link runs at (`monitor_speed` in
/// `platformio.ini`). The USB CDC boards ignore it; the T-Beam's second UART
/// variant does not.
pub const BAUD: u32 = 115_200;

/// What the modem says about its receiver, in `INFO`'s `gps` field.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum GpsState {
    /// No receiver on this board, as on the Nucleo.
    #[default]
    None,
    /// A receiver with no fix and no feed running.
    Off,
    /// Talking, searching.
    NoFix,
    /// A fix no older than the firmware's thirty seconds.
    Fix,
}

impl GpsState {
    fn parse(s: &str) -> Self {
        match s {
            "off" => Self::Off,
            "no fix" => Self::NoFix,
            "fix" => Self::Fix,
            _ => Self::None,
        }
    }

    /// Whether asking this modem for a feed could ever produce a position.
    pub fn has_receiver(self) -> bool {
        self != Self::None
    }
}

impl std::fmt::Display for GpsState {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(match self {
            Self::None => "no receiver",
            Self::Off => "receiver idle",
            Self::NoFix => "searching",
            Self::Fix => "fix",
        })
    }
}

/// What a modem answered `GET_INFO` with.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct Info {
    pub fw: String,
    pub board: String,
    pub radio: String,
    pub gps: GpsState,
    pub batt_mv: Option<u16>,
    /// The name it advertises over Bluetooth, which is the only name a modem
    /// has and is what an operator sees printed on nothing at all.
    pub ble_name: Option<String>,
}

impl Info {
    fn read(value: &[u8]) -> Self {
        let mut info = Self::default();
        for (id, v) in tlvs(value) {
            let text = || String::from_utf8_lossy(v).trim().to_string();
            match id {
                I_FW => info.fw = text(),
                I_BOARD => info.board = text(),
                I_RADIO => info.radio = text(),
                I_GPS => info.gps = GpsState::parse(&text()),
                I_BLE_NAME => info.ble_name = Some(text()),
                I_BATT_MV if v.len() == 2 => {
                    info.batt_mv = Some(u16::from_le_bytes([v[0], v[1]]));
                }
                _ => {}
            }
        }
        info
    }

    /// One line for a list: the board, the radio it drives and its receiver.
    pub fn summary(&self) -> String {
        let board = if self.board.is_empty() { "modem" } else { &self.board };
        let radio = if self.radio.is_empty() { String::new() } else { format!(", {}", self.radio) };
        format!("{board}{radio}, {}", self.gps)
    }
}

/// A modem found on a port.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Found {
    pub transport: Transport,
    pub info: Info,
}

pub fn crc16(data: &[u8]) -> u16 {
    let mut crc: u16 = 0xFFFF;
    for b in data {
        crc ^= (*b as u16) << 8;
        for _ in 0..8 {
            crc = if crc & 0x8000 != 0 { (crc << 1) ^ 0x1021 } else { crc << 1 };
        }
    }
    crc
}

pub fn frame(msg: u8, value: &[u8]) -> Vec<u8> {
    let mut body = Vec::with_capacity(value.len() + OVERHEAD);
    body.push(msg);
    body.extend_from_slice(&(value.len() as u16).to_le_bytes());
    body.extend_from_slice(value);
    let crc = crc16(&body);
    let mut out = Vec::with_capacity(body.len() + 4);
    out.extend_from_slice(&SOF);
    out.extend_from_slice(&body);
    out.extend_from_slice(&crc.to_le_bytes());
    out
}

/// Walks the nested `ID u8 | LEN u8 | VALUE` TLVs of a value field.
fn tlvs(value: &[u8]) -> impl Iterator<Item = (u8, &[u8])> {
    let mut pos = 0usize;
    std::iter::from_fn(move || {
        let (id, len) = (*value.get(pos)?, *value.get(pos + 1)? as usize);
        let end = pos + 2 + len;
        let v = value.get(pos + 2..end)?;
        pos = end;
        Some((id, v))
    })
}

/// Bytes in, frames out.
///
/// The magic is there to resync after a truncated write, so a bad CRC drops a
/// single byte and looks for the next magic rather than the whole frame: the
/// two bytes of magic can appear inside a payload, and a false start that ate
/// its length in real bytes would take the frame after it as well.
#[derive(Default)]
pub struct Frames {
    buf: Vec<u8>,
}

impl Frames {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn push(&mut self, bytes: &[u8]) {
        self.buf.extend_from_slice(bytes);
    }

    pub fn next(&mut self) -> Option<(u8, Vec<u8>)> {
        loop {
            let start = self.buf.windows(2).position(|w| w == SOF)?;
            if start > 0 {
                self.buf.drain(..start);
            }
            if self.buf.len() < OVERHEAD {
                return None;
            }
            let len = u16::from_le_bytes([self.buf[3], self.buf[4]]) as usize;
            if len > MAX_VALUE {
                self.buf.drain(..1);
                continue;
            }
            let total = OVERHEAD + len;
            if self.buf.len() < total {
                return None;
            }
            let body = &self.buf[2..5 + len];
            let want = u16::from_le_bytes([self.buf[5 + len], self.buf[6 + len]]);
            if crc16(body) != want {
                self.buf.drain(..1);
                continue;
            }
            let msg = self.buf[2];
            let value = self.buf[5..5 + len].to_vec();
            self.buf.drain(..total);
            return Some((msg, value));
        }
    }
}

/// A modem's NMEA feed, read as lines.
///
/// Wraps the port so everything above it reads sentences: the feed is turned
/// on once, and each `NMEA` frame becomes the sentence it holds plus the
/// newline the firmware strips. Every other frame is dropped, including the
/// received packets a modem left in receive mode will keep sending, because
/// this is a GPS and the packets belong to whatever is driving the radio.
pub struct Feed<T> {
    port: T,
    frames: Frames,
    lines: std::collections::VecDeque<u8>,
    raw: [u8; 512],
}

impl<T: Read + Write> Feed<T> {
    /// Turn the feed on and start reading it.
    pub fn start(mut port: T) -> std::io::Result<Self> {
        port.write_all(&frame(MSG_GPS, &[1]))?;
        port.flush()?;
        Ok(Self { port, frames: Frames::new(), lines: Default::default(), raw: [0; 512] })
    }
}

impl<T: Read> Read for Feed<T> {
    fn read(&mut self, out: &mut [u8]) -> std::io::Result<usize> {
        while self.lines.is_empty() {
            let n = self.port.read(&mut self.raw)?;
            if n == 0 {
                return Ok(0);
            }
            self.frames.push(&self.raw[..n]);
            while let Some((msg, value)) = self.frames.next() {
                if msg == MSG_NMEA {
                    self.lines.extend(value);
                    self.lines.push_back(b'\n');
                }
            }
        }
        let n = out.len().min(self.lines.len());
        for slot in out.iter_mut().take(n) {
            *slot = self.lines.pop_front().unwrap_or(0);
        }
        Ok(n)
    }
}

/// Ask a port what is on it.
///
/// An ESP32 behind a CP210x or a CH340 resets when the port is opened, so the
/// question is asked repeatedly until the deadline rather than once: the first
/// `GET_INFO` goes into a bootloader and the reply arrives a second later.
/// `READY` and any other frame the modem volunteers are ignored; only `INFO`
/// answers this.
pub fn probe(path: &str, baud: u32, timeout: Duration) -> std::io::Result<Info> {
    let mut port = crate::source::open_port(path, baud, Duration::from_millis(200))?;
    let mut frames = Frames::new();
    let mut buf = [0u8; 512];
    let deadline = Instant::now() + timeout;
    let mut asked = Instant::now() - Duration::from_secs(1);
    while Instant::now() < deadline {
        if asked.elapsed() >= Duration::from_millis(400) {
            port.write_all(&frame(MSG_GET_INFO, &[]))?;
            port.flush()?;
            asked = Instant::now();
        }
        let n = port.read(&mut buf)?;
        frames.push(&buf[..n]);
        while let Some((msg, value)) = frames.next() {
            if msg == MSG_INFO {
                return Ok(Info::read(&value));
            }
        }
    }
    Err(std::io::Error::new(std::io::ErrorKind::TimedOut, "no modem answered"))
}

/// How long a probe waits for one port. Two seconds covers an ESP32 that the
/// open reset: the bootloader delay and the radio's own init come before the
/// first frame.
pub const PROBE: Duration = Duration::from_secs(2);

/// The serial ports worth asking, which is the USB ones.
///
/// A modem is always behind a USB bridge or a USB CDC device, never on a
/// header, so the on-board UARTs are left alone: opening `/dev/ttyS0` on a
/// machine with a serial console attached is not a free question.
pub fn candidates() -> Vec<String> {
    #[cfg(unix)]
    {
        let keep = ["ttyACM", "ttyUSB", "cu.usbmodem", "cu.usbserial", "cu.wchusbserial"];
        let mut found: Vec<String> = std::fs::read_dir("/dev")
            .into_iter()
            .flatten()
            .flatten()
            .filter_map(|e| {
                let name = e.file_name().to_string_lossy().to_string();
                keep.iter().any(|p| name.starts_with(p)).then(|| format!("/dev/{name}"))
            })
            .collect();
        found.sort();
        found
    }
    #[cfg(windows)]
    {
        // Windows has no directory of ports, and an absent COM port fails to
        // open immediately, so the first thirty-two are simply tried.
        (1..=32).map(|n| format!("COM{n}")).collect()
    }
}

/// Look for modems on every USB serial port, in parallel.
///
/// This is on demand rather than continuous because opening a serial port is
/// exclusive: a probe takes the port from whatever else was reading it for as
/// long as it runs, which is not something to do to an operator's own GPS
/// behind their back.
pub fn discover() -> Vec<Found> {
    let workers: Vec<_> = candidates()
        .into_iter()
        .map(|path| {
            std::thread::spawn(move || {
                let info = probe(&path, BAUD, PROBE);
                match info {
                    Ok(info) => {
                        tracing::info!("modem on {path}: {}", info.summary());
                        Some(Found { transport: Transport::Modem { path, baud: BAUD }, info })
                    }
                    Err(e) => {
                        tracing::debug!("no modem on {path}: {e}");
                        None
                    }
                }
            })
        })
        .collect();
    workers.into_iter().filter_map(|w| w.join().ok().flatten()).collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The frame `tools/modem.py` writes for `info`, byte for byte.
    #[test]
    fn a_get_info_is_the_bytes_the_python_writes() {
        assert_eq!(frame(MSG_GET_INFO, &[]), vec![0xA5, 0x5A, 0x02, 0x00, 0x00, 0xFC, 0xA2]);
        assert_eq!(frame(MSG_GPS, &[1]), vec![0xA5, 0x5A, 0x11, 0x01, 0x00, 0x01, 0xC2, 0xCE]);
        // And the CRC is CCITT with an all-ones seed, over type, length and
        // value but not the magic.
        assert_eq!(crc16(&[0x02, 0x00, 0x00]), 0xA2FC);
    }

    #[test]
    fn a_frame_round_trips() {
        let mut f = Frames::new();
        f.push(&frame(MSG_NMEA, b"$GPGGA,,,*00"));
        assert_eq!(f.next(), Some((MSG_NMEA, b"$GPGGA,,,*00".to_vec())));
        assert_eq!(f.next(), None);
    }

    /// A host attaching mid-sentence sees the tail of a frame it never saw
    /// the start of, and the magic is how it finds its feet again.
    #[test]
    fn a_truncated_write_resyncs() {
        let good = frame(MSG_NMEA, b"$GPRMC,1");
        let mut f = Frames::new();
        f.push(&good[4..]); // half a frame, no magic
        f.push(&good);
        assert_eq!(f.next(), Some((MSG_NMEA, b"$GPRMC,1".to_vec())));
        assert_eq!(f.next(), None);

        // A frame whose CRC is wrong is dropped and does not eat the next one.
        let mut bad = frame(MSG_NMEA, b"xy");
        *bad.last_mut().expect("a crc") ^= 0xFF;
        let mut f = Frames::new();
        f.push(&bad);
        f.push(&frame(MSG_NMEA, b"ok"));
        assert_eq!(f.next(), Some((MSG_NMEA, b"ok".to_vec())));
    }

    /// A length longer than the protocol allows is a false start, and waiting
    /// for its bytes would stall the parser until that many arrived.
    #[test]
    fn an_impossible_length_is_not_waited_for() {
        let mut f = Frames::new();
        f.push(&[0xA5, 0x5A, 0x8B, 0xFF, 0xFF, 0x00, 0x00]);
        f.push(&frame(MSG_NMEA, b"z"));
        assert_eq!(f.next(), Some((MSG_NMEA, b"z".to_vec())));
    }

    /// An `INFO` from a T-Beam v1.1, as `tools/modem.py info` prints it.
    #[test]
    fn an_info_says_which_board_and_whether_it_has_a_receiver() {
        let mut v = Vec::new();
        for (id, s) in
            [(I_FW, "0.9.0"), (I_BOARD, "T-Beam v1.1"), (I_RADIO, "SX1276"), (I_GPS, "fix")]
        {
            v.push(id);
            v.push(s.len() as u8);
            v.extend_from_slice(s.as_bytes());
        }
        v.extend_from_slice(&[I_BATT_MV, 2, 0x10, 0x10]);
        v.extend_from_slice(&[I_BLE_NAME, 10]);
        v.extend_from_slice(b"modem-FAC9");
        let info = Info::read(&v);
        assert_eq!(info.board, "T-Beam v1.1");
        assert_eq!(info.radio, "SX1276");
        assert_eq!(info.fw, "0.9.0");
        assert_eq!(info.gps, GpsState::Fix);
        assert_eq!(info.batt_mv, Some(4_112));
        assert_eq!(info.ble_name.as_deref(), Some("modem-FAC9"));
        assert!(info.gps.has_receiver());
        assert_eq!(info.summary(), "T-Beam v1.1, SX1276, fix");

        // The Nucleo has no receiver and says so, which is the one answer
        // that must not end up in the picker.
        let none = Info::read(&[I_GPS, 4, b'n', b'o', b'n', b'e']);
        assert_eq!(none.gps, GpsState::None);
        assert!(!none.gps.has_receiver());
    }

    /// The feed exists to make a modem look like a serial GPS, so what comes
    /// out of it is sentences with newlines and nothing else: a modem in
    /// receive mode interleaves `RX` frames into the same stream.
    #[test]
    fn a_feed_turns_frames_back_into_lines() {
        use std::io::Cursor;
        struct Port(Cursor<Vec<u8>>, Vec<u8>);
        impl Read for Port {
            fn read(&mut self, b: &mut [u8]) -> std::io::Result<usize> {
                self.0.read(b)
            }
        }
        impl Write for Port {
            fn write(&mut self, b: &[u8]) -> std::io::Result<usize> {
                self.1.extend_from_slice(b);
                Ok(b.len())
            }
            fn flush(&mut self) -> std::io::Result<()> {
                Ok(())
            }
        }

        let mut wire = Vec::new();
        wire.extend_from_slice(&frame(MSG_NMEA, b"$GPGGA,123519,4807.038,N"));
        wire.extend_from_slice(&frame(0x86, b"\x00\x00\x00\x00\x00packet"));
        wire.extend_from_slice(&frame(MSG_NMEA, b"$GPRMC,123519,A"));
        let mut feed = Feed::start(Port(Cursor::new(wire), Vec::new())).expect("a feed");

        let mut out = String::new();
        feed.read_to_string(&mut out).expect("lines");
        assert_eq!(out, "$GPGGA,123519,4807.038,N\n$GPRMC,123519,A\n");
        // And starting it asked for the feed, which is off until somebody does.
        assert_eq!(feed.port.1, frame(MSG_GPS, &[1]));
    }
}
