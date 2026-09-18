//! The ports dump1090 serves, and the fan-out behind them.
//!
//! One listener a port, one thread each, and a shared list of connected
//! sockets. A client that stops reading fills its socket buffer and is dropped
//! rather than stalling the demodulator, which is the only behaviour a
//! receiver can have: the samples keep arriving whatever the network does.

use std::io::Write;
use std::net::{TcpListener, TcpStream};
use std::sync::{Arc, Mutex};

/// A port and everybody listening on it
#[derive(Clone, Default)]
pub struct Fanout {
    clients: Arc<Mutex<Vec<TcpStream>>>,
}

impl Fanout {
    /// Start listening, or return the error a caller should print and exit on.
    pub fn serve(addr: &str, port: u16) -> std::io::Result<Self> {
        let listener = TcpListener::bind((addr, port))?;
        let out = Self::default();
        let clients = out.clients.clone();
        std::thread::spawn(move || {
            for sock in listener.incoming().flatten() {
                let _ = sock.set_nodelay(true);
                // Nothing is ever read from a client, and a write that would
                // block is a client to drop rather than wait for.
                let _ = sock.set_write_timeout(Some(std::time::Duration::from_millis(50)));
                clients.lock().unwrap().push(sock);
            }
        });
        Ok(out)
    }

    /// Write to every client, dropping the ones that fail.
    pub fn send(&self, bytes: &[u8]) {
        let mut clients = self.clients.lock().unwrap();
        clients.retain_mut(|c| c.write_all(bytes).is_ok());
    }
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

    #[test]
    fn a_long_frame_is_typed_as_one() {
        assert_eq!(beast(&[0u8; 14], 0, -100.0)[1], b'3');
    }
}
