//! The ports dump1090 serves, and the fan-out behind them.
//!
//! One listener a port, one thread each, and a shared list of connected
//! sockets. A client that stops reading fills its socket buffer and is dropped
//! rather than stalling the demodulator, which is the only behaviour a
//! receiver can have: the samples keep arriving whatever the network does.

use std::io::Write;
use std::net::{TcpListener, TcpStream};
use std::sync::mpsc::{Receiver, Sender};
use std::sync::{Arc, Mutex};

/// A port and everybody listening on it
#[derive(Clone)]
pub struct Fanout {
    /// What the port carries, for the log: AVR, SBS, Beast.
    name: &'static str,
    clients: Arc<Mutex<Vec<TcpStream>>>,
    addr: std::net::SocketAddr,
}

impl Fanout {
    /// Start listening, or return the error a caller should print and exit on.
    pub fn serve(name: &'static str, addr: &str, port: u16) -> std::io::Result<Self> {
        let listener = TcpListener::bind((addr, port))?;
        let out = Self { name, clients: Arc::default(), addr: listener.local_addr()? };
        let clients = out.clients.clone();
        std::thread::spawn(move || {
            for sock in listener.incoming().flatten() {
                let _ = sock.set_nodelay(true);
                // Nothing is ever read from a client, and a write that would
                // block is a client to drop rather than wait for.
                let _ = sock.set_write_timeout(Some(std::time::Duration::from_millis(50)));
                match sock.peer_addr() {
                    Ok(peer) => tracing::info!("{name}: {peer} connected"),
                    Err(_) => tracing::info!("{name}: a client connected and left"),
                }
                clients.lock().unwrap().push(sock);
            }
        });
        Ok(out)
    }

    /// Where it is listening, which a port of zero only settles here.
    pub fn addr(&self) -> std::net::SocketAddr {
        self.addr
    }

    /// How many are reading it now.
    pub fn clients(&self) -> usize {
        self.clients.lock().unwrap().len()
    }

    /// Write to every client, dropping the ones that fail.
    pub fn send(&self, bytes: &[u8]) {
        let mut clients = self.clients.lock().unwrap();
        clients.retain_mut(|c| match c.write_all(bytes) {
            Ok(()) => true,
            Err(e) => {
                match c.peer_addr() {
                    Ok(peer) => tracing::info!("{}: {peer} dropped: {e}", self.name),
                    Err(_) => tracing::info!("{}: a client dropped: {e}", self.name),
                }
                false
            }
        });
    }
}

/// Frames arriving from somebody else, on a port dump1090 calls `net-bi`.
///
/// This is how an mlat client hands its results back: it computes a position
/// from several receivers and returns synthetic frames, which the receiver
/// republishes so everything downstream sees the aircraft. Beast and AVR are
/// both accepted, since a client picks either.
pub fn accept_frames(addr: &str, port: u16) -> std::io::Result<Receiver<Vec<u8>>> {
    let listener = TcpListener::bind((addr, port))?;
    let (tx, rx) = std::sync::mpsc::channel();
    std::thread::spawn(move || {
        for sock in listener.incoming().flatten() {
            let tx = tx.clone();
            std::thread::spawn(move || read_frames(sock, port, tx));
        }
    });
    Ok(rx)
}

fn read_frames(sock: TcpStream, port: u16, tx: Sender<Vec<u8>>) {
    use std::io::Read;
    let mut sock = sock;
    let peer = sock.peer_addr().map(|p| p.to_string()).unwrap_or_else(|_| "?".into());
    tracing::info!("input {port}: {peer} connected");
    let (mut buf, mut chunk) = (Vec::new(), [0u8; 4096]);
    let mut frames = 0u64;
    loop {
        let n = match sock.read(&mut chunk) {
            Ok(0) | Err(_) => {
                tracing::info!("input {port}: {peer} left after {frames} frames");
                return;
            }
            Ok(n) => n,
        };
        buf.extend_from_slice(&chunk[..n]);
        while let Some((frame, used)) = next_frame(&buf) {
            buf.drain(..used);
            if let Some(f) = frame {
                frames += 1;
                if tx.send(f).is_err() {
                    return;
                }
            }
        }
        // A client sending nothing a frame recognises is a client to stop
        // buffering for.
        if buf.len() > 64 * 1024 {
            buf.clear();
        }
    }
}

/// The next frame in `buf`, and how many bytes it consumed.
///
/// `None` for the whole answer means there is not enough yet; a `Some` with a
/// `None` frame is a byte skipped as noise between frames.
pub fn next_frame(buf: &[u8]) -> Option<(Option<Vec<u8>>, usize)> {
    match buf.first()? {
        0x1a => {
            let kind = *buf.get(1)?;
            let len = match kind {
                b'1' => 2,
                b'2' => 7,
                b'3' => 14,
                // Not a frame type, so the marker was data: step over it.
                _ => return Some((None, 1)),
            };
            // Six of timestamp and one of level ahead of the frame, with
            // every 0x1a in any of it doubled.
            let mut out = Vec::with_capacity(len + 7);
            let mut i = 2;
            while out.len() < len + 7 {
                let b = *buf.get(i)?;
                if b == 0x1a {
                    if *buf.get(i + 1)? != 0x1a {
                        return Some((None, 1));
                    }
                    i += 1;
                }
                out.push(b);
                i += 1;
            }
            Some((Some(out.split_off(7)), i))
        }
        b'*' | b'@' => {
            let end = buf.iter().position(|b| *b == b';')?;
            // A `@` line carries a timestamp ahead of the frame, in hex; the
            // frame is the tail, and its length says where it starts.
            let body = &buf[1..end];
            let bytes = unhex(body).filter(|b| matches!(b.len(), 7 | 14)).or_else(|| {
                unhex(body).and_then(|b| (b.len() > 14).then(|| b[b.len() - 14..].to_vec()))
            });
            Some((bytes, end + 1))
        }
        _ => Some((None, 1)),
    }
}

fn unhex(s: &[u8]) -> Option<Vec<u8>> {
    let s: Vec<u8> = s.iter().copied().filter(|c| !c.is_ascii_whitespace()).collect();
    (s.len() % 2 == 0 && s.iter().all(|c| c.is_ascii_hexdigit()))
        .then(|| {
            s.chunks(2)
                .map(|p| u8::from_str_radix(std::str::from_utf8(p).ok()?, 16).ok())
                .collect::<Option<Vec<u8>>>()
        })
        .flatten()
}

/// A frame as AVR hex, the format dump1090 serves on 30002: `*8d4840d6...;`
pub fn avr(bytes: &[u8]) -> String {
    let mut s = String::with_capacity(bytes.len() * 2 + 3);
    s.push('*');
    for b in bytes {
        s.push_str(&format!("{b:02x}"));
    }
    s.push_str(";\r\n");
    s
}

/// A frame in Beast binary, the format dump1090 serves on 30005.
///
/// `0x1a`, a type byte, a six byte 12 MHz timestamp, a signal level, then the
/// frame. Every `0x1a` in any of that is doubled, since it is also the start
/// marker.
pub fn beast(bytes: &[u8], clock_12mhz: u64, rssi_dbfs: f32) -> Vec<u8> {
    let kind = match bytes.len() {
        7 => b'2',
        _ => b'3',
    };
    // dump1090 carries the level as 255 times the square root of the power
    // referred to full scale, which is the amplitude.
    let level = (255.0 * 10f32.powf(rssi_dbfs / 20.0)).clamp(0.0, 255.0) as u8;
    let mut body = Vec::with_capacity(bytes.len() + 8);
    body.extend_from_slice(&clock_12mhz.to_be_bytes()[2..]);
    body.push(level);
    body.extend_from_slice(bytes);
    let mut out = Vec::with_capacity(body.len() * 2 + 2);
    out.push(0x1a);
    out.push(kind);
    for b in body {
        out.push(b);
        if b == 0x1a {
            out.push(0x1a);
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn an_avr_line_is_the_frame_in_lower_case_hex() {
        assert_eq!(avr(&[0x8d, 0x48, 0x40, 0xd6]), "*8d4840d6;\r\n");
    }

    /// The escape is the whole difficulty of the format: a client resyncs on
    /// `0x1a`, so a byte of data equal to it has to arrive doubled or the
    /// frame after it is read at the wrong offset.
    #[test]
    fn a_beast_frame_doubles_every_escape_byte_in_its_body() {
        let out = beast(&[0x1a, 0x00, 0x1a, 0, 0, 0, 0], 0x0000_0000_0001, -6.0);
        assert_eq!(out[0], 0x1a, "the start marker");
        assert_eq!(out[1], b'2', "a short frame");
        // Six of timestamp, one of level, then the frame with both 0x1a
        // bytes doubled.
        assert_eq!(&out[2..8], &[0, 0, 0, 0, 0, 1]);
        assert_eq!(out[8], 127, "half amplitude is half of full scale");
        assert_eq!(&out[9..], &[0x1a, 0x1a, 0x00, 0x1a, 0x1a, 0, 0, 0, 0]);
    }

    /// What goes out has to come back in: an mlat client returns Beast, and a
    /// frame that does not survive the round trip is one nothing downstream
    /// ever sees.
    #[test]
    fn a_beast_frame_reads_back_as_the_frame_that_was_written() {
        let frame = [0x8d, 0x1a, 0x40, 0xd6, 0x1a, 0x1a, 0, 0, 0, 0, 0, 0, 0, 9];
        let wire = beast(&frame, 0x1234_5678, -12.0);
        let (got, used) = next_frame(&wire).expect("a whole frame");
        assert_eq!(got.as_deref(), Some(&frame[..]));
        assert_eq!(used, wire.len(), "and nothing left over");
        // Half a frame is not yet an answer, rather than a wrong one.
        assert!(next_frame(&wire[..8]).is_none());
    }

    #[test]
    fn an_avr_line_reads_back_as_its_frame() {
        let line = b"*8d4840d6202cc371c32ce0576098;\r\n";
        let (got, used) = next_frame(line).expect("a whole line");
        assert_eq!(got.expect("a frame").len(), 14);
        assert_eq!(used, 30);
        // With a timestamp ahead of it, which is what `@` means.
        let stamped = b"@0000000000008d4840d6202cc371c32ce0576098;";
        let (got, _) = next_frame(stamped).expect("a whole line");
        assert_eq!(got.expect("a frame").len(), 14);
    }

    #[test]
    fn a_long_frame_is_typed_as_one() {
        assert_eq!(beast(&[0u8; 14], 0, -100.0)[1], b'3');
    }
}
