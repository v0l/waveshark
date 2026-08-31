//! Packets from another receiver, over TCP.
//!
//! A receiver on a rooftop hears things a receiver on a desk does not, and
//! dump1090 and its relatives already publish what they demodulate. Attaching
//! one to the packet bus makes it another front end: its frames appear in the
//! packet list, go into the log, and reach the flight list, with no view
//! needing to know where they came from.
//!
//! Two wire formats, both of which carry frames rather than conclusions:
//! Beast binary (usually port 30005) and AVR hex (usually 30002). The
//! BaseStation CSV feed on port 30003 is deliberately not supported: it
//! carries fields somebody else already decoded, and a log of conclusions
//! cannot be re-decoded, corrected, or shown to be wrong.

use common::{Error, Hz, Packet, PacketBody, Result};
use pipeline::node::{Node, NodeCtx, PortSpec};
use pipeline::port::{Payload, PortKind};
use pipeline::StreamSpec;
use std::io::Read;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::mpsc::{Receiver, Sender, TryRecvError};
use std::sync::Arc;

/// Mode S occupies the band rather than a channel in it.
const MODES_BAND_HZ: u32 = 2_000_000;

#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
pub enum FeedFormat {
    /// `0x1a`-framed binary with a timestamp and a signal level, the format
    /// dump1090 serves on port 30005.
    #[default]
    Beast,
    /// `*8d4840d6...;` lines, one frame each, on port 30002.
    Avr,
}

impl FeedFormat {
    pub fn label(&self) -> &'static str {
        match self {
            FeedFormat::Beast => "beast",
            FeedFormat::Avr => "avr",
        }
    }

    pub fn default_port(&self) -> u16 {
        match self {
            FeedFormat::Beast => 30005,
            FeedFormat::Avr => 30002,
        }
    }

    pub fn parse(&self, s: &str) -> Option<Self> {
        match s {
            "beast" => Some(FeedFormat::Beast),
            "avr" => Some(FeedFormat::Avr),
            _ => None,
        }
    }
}

#[derive(Clone, PartialEq, Eq, Debug)]
pub struct FeedSpec {
    pub host: String,
    pub port: u16,
    pub format: FeedFormat,
    /// What the far end is tuned to. Every format here carries Mode S, but
    /// the packet log records a frequency for everything on it and inventing
    /// one at the far end is better than leaving it zero.
    pub center_hz: u64,
}

impl FeedSpec {
    pub fn new(host: impl Into<String>, port: u16, format: FeedFormat) -> Self {
        Self { host: host.into(), port, format, center_hz: 1_090_000_000 }
    }

    pub fn address(&self) -> String {
        format!("{}:{}", self.host, self.port)
    }
}

/// A frame off the wire, before it is a packet.
#[derive(Clone, PartialEq, Debug)]
pub struct WireFrame {
    pub bytes: Vec<u8>,
    /// Level as the far end reported it, or `NaN` for a format that does not
    /// carry one.
    pub rssi_dbfs: f32,
}

#[derive(Default)]
struct FeedState {
    connected: AtomicBool,
    frames: AtomicU64,
    error: std::sync::Mutex<Option<String>>,
}

/// A feed, as a source node with no input of its own.
pub struct FeedNode {
    spec: FeedSpec,
    rx: Receiver<Packet>,
    state: Arc<FeedState>,
    stop: Arc<AtomicBool>,
    thread: Option<std::thread::JoinHandle<()>>,
}

impl FeedNode {
    pub fn new(spec: FeedSpec) -> Self {
        let (tx, rx) = std::sync::mpsc::channel();
        let state = Arc::new(FeedState::default());
        let stop = Arc::new(AtomicBool::new(false));
        let thread = {
            let (spec, state, stop) = (spec.clone(), state.clone(), stop.clone());
            std::thread::Builder::new()
                .name(format!("feed-{}", spec.address()))
                .spawn(move || run(spec, tx, state, stop))
                .ok()
        };
        Self { spec, rx, state, stop, thread }
    }

    pub fn spec(&self) -> &FeedSpec {
        &self.spec
    }

    pub fn connected(&self) -> bool {
        self.state.connected.load(Ordering::Relaxed)
    }

    pub fn frames(&self) -> u64 {
        self.state.frames.load(Ordering::Relaxed)
    }

    /// Why it is not connected, when it is not.
    pub fn error(&self) -> Option<String> {
        self.state.error.lock().ok().and_then(|e| e.clone())
    }
}

impl Drop for FeedNode {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        // The reader has a one second timeout, so it notices without being
        // interrupted; joining keeps the socket from outliving the node.
        if let Some(t) = self.thread.take() {
            let _ = t.join();
        }
    }
}

impl Node for FeedNode {
    fn name(&self) -> &str {
        "feed"
    }

    fn as_any(&self) -> Option<&dyn std::any::Any> {
        Some(self)
    }

    fn as_any_mut(&mut self) -> Option<&mut dyn std::any::Any> {
        Some(self)
    }

    /// So a retune carries the open socket into the new graph instead of
    /// dropping the connection and starting again.
    fn into_any(self: Box<Self>) -> Option<Box<dyn std::any::Any>> {
        Some(self)
    }

    fn num_inputs(&self) -> usize {
        0
    }

    fn negotiate(&mut self, _inputs: &[PortSpec]) -> Result<Vec<StreamSpec>> {
        let mut out = StreamSpec::iq(0.0, Hz(self.spec.center_hz)).with_kind(PortKind::Packets);
        out.rate = 0.0;
        out.bandwidth = f64::from(MODES_BAND_HZ);
        Ok(vec![out])
    }

    fn process(
        &mut self,
        _inputs: &[&Payload],
        outputs: &mut [Payload],
        _ctx: &mut NodeCtx<'_>,
    ) -> Result<()> {
        let out = outputs[0].packets_mut();
        loop {
            match self.rx.try_recv() {
                Ok(p) => out.push(p),
                Err(TryRecvError::Empty) => break,
                // The reader died. Nothing more will arrive, and saying so
                // every block would drown the log.
                Err(TryRecvError::Disconnected) => break,
            }
        }
        Ok(())
    }
}

fn run(spec: FeedSpec, tx: Sender<Packet>, state: Arc<FeedState>, stop: Arc<AtomicBool>) {
    let mut backoff = std::time::Duration::from_millis(500);
    while !stop.load(Ordering::Relaxed) {
        match connect(&spec) {
            Ok(sock) => {
                set_error(&state, None);
                state.connected.store(true, Ordering::Relaxed);
                backoff = std::time::Duration::from_millis(500);
                read_loop(&spec, sock, &tx, &state, &stop);
                state.connected.store(false, Ordering::Relaxed);
            }
            Err(e) => {
                set_error(&state, Some(e.to_string()));
                state.connected.store(false, Ordering::Relaxed);
            }
        }
        // Backing off rather than hammering: a feed that is down is usually
        // down for as long as it takes someone to notice.
        let deadline = std::time::Instant::now() + backoff;
        while std::time::Instant::now() < deadline {
            if stop.load(Ordering::Relaxed) {
                return;
            }
            std::thread::sleep(std::time::Duration::from_millis(50));
        }
        backoff = (backoff * 2).min(std::time::Duration::from_secs(15));
    }
}

fn connect(spec: &FeedSpec) -> std::io::Result<std::net::TcpStream> {
    use std::net::ToSocketAddrs;
    let addr = spec
        .address()
        .to_socket_addrs()?
        .next()
        .ok_or_else(|| std::io::Error::other("no address"))?;
    let sock = std::net::TcpStream::connect_timeout(&addr, std::time::Duration::from_secs(5))?;
    // A feed goes quiet at night; a read timeout is how the thread notices it
    // has been asked to stop rather than blocking until a frame arrives.
    sock.set_read_timeout(Some(std::time::Duration::from_secs(1)))?;
    Ok(sock)
}

fn read_loop(
    spec: &FeedSpec,
    mut sock: std::net::TcpStream,
    tx: &Sender<Packet>,
    state: &FeedState,
    stop: &AtomicBool,
) {
    let mut buf = Vec::with_capacity(8192);
    let mut chunk = [0u8; 4096];
    while !stop.load(Ordering::Relaxed) {
        let n = match sock.read(&mut chunk) {
            Ok(0) => {
                set_error(state, Some("the far end closed the connection".into()));
                return;
            }
            Ok(n) => n,
            Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => continue,
            Err(e) if e.kind() == std::io::ErrorKind::TimedOut => continue,
            Err(e) => {
                set_error(state, Some(e.to_string()));
                return;
            }
        };
        buf.extend_from_slice(&chunk[..n]);
        let frames = match spec.format {
            FeedFormat::Beast => parse_beast(&mut buf),
            FeedFormat::Avr => parse_avr(&mut buf),
        };
        // A stream of junk on the right port would otherwise grow the buffer
        // forever looking for a frame that never starts.
        if buf.len() > 1 << 20 {
            buf.clear();
        }
        let at_us = now_us();
        for f in frames {
            state.frames.fetch_add(1, Ordering::Relaxed);
            let packet = Packet {
                at_us,
                center_hz: spec.center_hz,
                bandwidth_hz: MODES_BAND_HZ,
                rssi_dbfs: f.rssi_dbfs,
                snr_db: f32::NAN,
                body: PacketBody::Frame(f.bytes),
            };
            if tx.send(packet).is_err() {
                return;
            }
        }
    }
}

/// Recording why a feed is not working, for the settings modal to show.
fn set_error(state: &FeedState, msg: Option<String>) {
    if let Ok(mut e) = state.error.lock() {
        *e = msg;
    }
}

fn now_us() -> u64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_micros() as u64)
        .unwrap_or(0)
}

/// Beast binary: `0x1a`, a type byte, six bytes of timestamp, one of signal
/// level, then the frame. Any `0x1a` in the body is doubled, which is the
/// only reason this needs a parser rather than a slice.
///
/// Complete messages are taken and the rest is left in `buf` for the next
/// read, since a TCP read has nothing to do with a message boundary.
pub fn parse_beast(buf: &mut Vec<u8>) -> Vec<WireFrame> {
    let mut out = Vec::new();
    let mut i = 0usize;
    let mut consumed = 0usize;
    while i < buf.len() {
        if buf[i] != 0x1a {
            i += 1;
            continue;
        }
        let Some(&kind) = buf.get(i + 1) else { break };
        let data_len = match kind {
            b'1' => 2,  // Mode A/C, not a Mode S frame
            b'2' => 7,  // short reply
            b'3' => 14, // extended squitter
            _ => {
                // Not a message we know. Skip the marker and resynchronise on
                // the next one rather than guessing a length.
                i += 1;
                continue;
            }
        };
        let want = 6 + 1 + data_len;
        let mut body = Vec::with_capacity(want);
        let mut j = i + 2;
        let mut truncated = false;
        while body.len() < want {
            let Some(&b) = buf.get(j) else {
                truncated = true;
                break;
            };
            if b == 0x1a {
                match buf.get(j + 1) {
                    Some(0x1a) => {
                        body.push(0x1a);
                        j += 2;
                    }
                    // A lone marker is the start of the next message, so this
                    // one was cut short by the sender.
                    Some(_) => break,
                    None => {
                        truncated = true;
                        break;
                    }
                }
            } else {
                body.push(b);
                j += 1;
            }
        }
        if truncated {
            break;
        }
        if body.len() == want {
            if kind != b'1' {
                out.push(WireFrame {
                    bytes: body[7..].to_vec(),
                    rssi_dbfs: level_to_dbfs(body[6]),
                });
            }
            i = j;
            consumed = j;
        } else {
            // Short message: drop it and pick up at whatever follows.
            i += 1;
        }
    }
    if consumed > 0 {
        buf.drain(..consumed);
    } else if buf.len() > 4096 {
        // Nothing parsed and the buffer is growing: keep only the tail, which
        // is the only part a message could still start in.
        let keep = buf.len() - 64;
        buf.drain(..keep);
    }
    out
}

/// dump1090 sends the square root of the mean power, scaled to a byte. Squaring
/// undoes that, and zero is reported as silence rather than as minus infinity.
fn level_to_dbfs(level: u8) -> f32 {
    if level == 0 {
        return f32::NAN;
    }
    let v = f32::from(level) / 255.0;
    10.0 * (v * v).log10()
}

/// AVR: one frame per line, `*hex;`, or `@<12 hex digits of timestamp>hex;`.
/// No signal level, which is why Beast is the better feed where both are
/// offered.
pub fn parse_avr(buf: &mut Vec<u8>) -> Vec<WireFrame> {
    let mut out = Vec::new();
    let mut consumed = 0usize;
    while let Some(end) = buf[consumed..].iter().position(|b| *b == b';') {
        let line = &buf[consumed..consumed + end];
        consumed += end + 1;
        let text = String::from_utf8_lossy(line);
        let text = text.trim();
        let hex = match text.as_bytes().first() {
            Some(b'*') => &text[1..],
            // The timestamp is the far end's clock, not ours, and the log
            // stamps arrival time anyway.
            Some(b'@') if text.len() > 13 => &text[13..],
            _ => continue,
        };
        let Some(bytes) = from_hex(hex) else { continue };
        // Short reply, extended squitter, or nothing this cares about.
        if bytes.len() == 7 || bytes.len() == 14 {
            out.push(WireFrame { bytes, rssi_dbfs: f32::NAN });
        }
    }
    if consumed > 0 {
        buf.drain(..consumed);
    } else if buf.len() > 4096 {
        buf.clear();
    }
    out
}

fn from_hex(s: &str) -> Option<Vec<u8>> {
    let s = s.trim();
    if !s.len().is_multiple_of(2) || s.is_empty() {
        return None;
    }
    (0..s.len() / 2)
        .map(|i| u8::from_str_radix(s.get(i * 2..i * 2 + 2)?, 16).ok())
        .collect()
}

/// Build a Beast message, for tests and for anything that wants to speak the
/// format rather than only read it.
pub fn beast_message(bytes: &[u8], level: u8, timestamp: u64) -> Result<Vec<u8>> {
    let kind = match bytes.len() {
        7 => b'2',
        14 => b'3',
        n => return Err(Error::other(format!("{n} is not a Mode S frame length"))),
    };
    let mut out = vec![0x1a, kind];
    let mut push = |b: u8| {
        out.push(b);
        if b == 0x1a {
            out.push(b);
        }
    };
    for shift in (0..6).rev() {
        push((timestamp >> (shift * 8)) as u8);
    }
    push(level);
    for b in bytes {
        push(*b);
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    const LONG: [u8; 14] = [
        0x8d, 0x48, 0x40, 0xd6, 0x20, 0x2c, 0xc3, 0x71, 0xc3, 0x2c, 0xe0, 0x57, 0x60, 0x98,
    ];

    #[test]
    fn a_beast_message_round_trips() {
        let mut buf = beast_message(&LONG, 200, 0x0102_0304_0506).unwrap();
        let out = parse_beast(&mut buf);
        assert_eq!(out.len(), 1);
        assert_eq!(out[0].bytes, LONG);
        assert!(out[0].rssi_dbfs < 0.0 && out[0].rssi_dbfs > -3.0, "{}", out[0].rssi_dbfs);
        assert!(buf.is_empty(), "the whole message should have been consumed");
    }

    /// The escape is the only hard part of the format: a `0x1a` in the frame
    /// is sent twice, and a parser that misses that loses the rest of the
    /// stream, not just one frame.
    #[test]
    fn an_escaped_marker_in_the_payload_is_not_a_new_message() {
        let mut frame = LONG;
        frame[3] = 0x1a;
        frame[9] = 0x1a;
        let mut buf = beast_message(&frame, 128, 0x1a1a_1a1a_1a1a).unwrap();
        let out = parse_beast(&mut buf);
        assert_eq!(out.len(), 1);
        assert_eq!(out[0].bytes, frame);
    }

    #[test]
    fn a_message_split_across_two_reads_survives() {
        let msg = beast_message(&LONG, 100, 7).unwrap();
        let mut buf = msg[..8].to_vec();
        assert!(parse_beast(&mut buf).is_empty(), "half a message is not a frame");
        buf.extend_from_slice(&msg[8..]);
        let out = parse_beast(&mut buf);
        assert_eq!(out.len(), 1);
        assert_eq!(out[0].bytes, LONG);
    }

    #[test]
    fn several_messages_and_junk_between_them() {
        let mut buf = vec![0x00, 0xff, 0x42];
        buf.extend(beast_message(&LONG, 90, 1).unwrap());
        buf.extend([0x1a, b'1', 0, 0, 0, 0, 0, 0, 0, 1, 2]);
        buf.extend(beast_message(&LONG[..7], 90, 2).unwrap());
        let out = parse_beast(&mut buf);
        // Mode A/C carries no Mode S frame, so it is counted out rather than
        // handed on as one.
        assert_eq!(out.len(), 2);
        assert_eq!(out[0].bytes.len(), 14);
        assert_eq!(out[1].bytes.len(), 7);
    }

    #[test]
    fn avr_lines_with_and_without_timestamps() {
        let mut buf = b"*8d4840d6202cc371c32ce0576098;\n@000000000001\
8d4840d6202cc371c32ce0576098;\n*5d4007fb3e0376;\n"
            .to_vec();
        let out = parse_avr(&mut buf);
        assert_eq!(out.len(), 3);
        assert_eq!(out[0].bytes, LONG);
        assert_eq!(out[1].bytes, LONG);
        assert_eq!(out[2].bytes.len(), 7);
        assert!(out[0].rssi_dbfs.is_nan(), "AVR carries no level to report");
    }

    #[test]
    fn a_partial_avr_line_waits_for_the_rest() {
        let mut buf = b"*8d4840d6202cc3".to_vec();
        assert!(parse_avr(&mut buf).is_empty());
        buf.extend_from_slice(b"71c32ce0576098;");
        let out = parse_avr(&mut buf);
        assert_eq!(out.len(), 1);
        assert_eq!(out[0].bytes, LONG);
        assert!(buf.is_empty());
    }

    #[test]
    fn rubbish_is_dropped_rather_than_decoded() {
        let mut buf = b"*not hex at all;*8d4840;\n".to_vec();
        // Wrong length and not hex: neither is a frame.
        assert!(parse_avr(&mut buf).is_empty());
    }

    /// The node is a source, so it has to keep working with nothing on the
    /// far end: an unreachable feed is a status line, not a failed build.
    #[test]
    fn a_feed_that_cannot_connect_still_builds_and_runs() {
        // Port 1 on localhost, which nothing listens on.
        let mut node = FeedNode::new(FeedSpec::new("127.0.0.1", 1, FeedFormat::Beast));
        let spec = node.negotiate(&[]).unwrap();
        assert_eq!(spec[0].kind, PortKind::Packets);
        let mut out = vec![Payload::Packets(Vec::new())];
        let mut events = Vec::new();
        let tags = Vec::new();
        let mut new_tags = Vec::new();
        let mut ctx = NodeCtx::new(0, &[], &tags, &mut events, &mut new_tags);
        node.process(&[], &mut out, &mut ctx).unwrap();
        assert!(out[0].as_packets().unwrap().is_empty());
        assert!(!node.connected());
    }

    #[test]
    fn a_feed_delivers_what_a_socket_sends_it() {
        let listener = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
        let port = listener.local_addr().unwrap().port();
        std::thread::spawn(move || {
            use std::io::Write;
            let (mut s, _) = listener.accept().unwrap();
            for _ in 0..3 {
                let _ = s.write_all(&beast_message(&LONG, 200, 0).unwrap());
            }
            let _ = s.flush();
            std::thread::sleep(std::time::Duration::from_millis(300));
        });

        let mut node = FeedNode::new(FeedSpec::new("127.0.0.1", port, FeedFormat::Beast));
        node.negotiate(&[]).unwrap();
        let mut got = Vec::new();
        let deadline = std::time::Instant::now() + std::time::Duration::from_secs(5);
        while got.len() < 3 && std::time::Instant::now() < deadline {
            let mut out = vec![Payload::Packets(Vec::new())];
            let mut events = Vec::new();
            let tags = Vec::new();
            let mut new_tags = Vec::new();
            let mut ctx = NodeCtx::new(0, &[], &tags, &mut events, &mut new_tags);
            node.process(&[], &mut out, &mut ctx).unwrap();
            got.extend(out[0].as_packets().unwrap().iter().cloned());
            std::thread::sleep(std::time::Duration::from_millis(20));
        }
        assert_eq!(got.len(), 3, "three frames sent, {} arrived", got.len());
        assert_eq!(got[0].frame().unwrap(), &LONG);
        assert_eq!(got[0].center_hz, 1_090_000_000);
        assert_eq!(node.frames(), 3);
    }
}
