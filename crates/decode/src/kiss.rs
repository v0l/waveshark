//! KISS, the framing a host uses to talk to a TNC.
//!
//! Bytes in both directions and nothing else: a frame is delimited by `FEND`,
//! its first byte says which port and which command, and the two bytes that
//! would otherwise be a delimiter are escaped. What rides inside a data frame
//! is an AX.25 frame without its flags, its stuffing or its check sequence,
//! all of which belong to `dsp::hdlc` and are added by whatever puts it on
//! the air.
//!
//! Sources: the KISS specification (Chepponis and Karn, 1987) for the frame
//! and the commands, and the AX.25 2.2 standard for what the payload is.

/// Frame delimiter, and the escape that lets one appear inside a frame.
pub const FEND: u8 = 0xC0;
pub const FESC: u8 = 0xDB;
pub const TFEND: u8 = 0xDC;
pub const TFESC: u8 = 0xDD;

/// What a client is asking the TNC to do.
///
/// `Return` is the whole byte `0xFF` rather than a nibble, which is why the
/// port is not part of it: it asks the TNC to leave KISS mode altogether.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Command {
    /// The payload is an AX.25 frame to transmit.
    Data,
    /// Flags sent in front of a frame, in units of 10 ms.
    TxDelay,
    /// The p of p-persistent CSMA, as (p * 256) - 1.
    Persistence,
    /// Slot interval, in units of 10 ms.
    SlotTime,
    /// Carrier held after a frame, in units of 10 ms. Deprecated by the
    /// specification and still sent by clients.
    TxTail,
    /// Nonzero for full duplex, zero for the CSMA the default is.
    FullDuplex,
    /// Whatever the hardware defines. Nothing here does.
    SetHardware,
    /// Leave KISS mode.
    Return,
    /// A command this TNC does not implement, kept so it can be counted
    /// rather than read as data.
    Unknown(u8),
}

impl Command {
    pub fn from_code(code: u8) -> Self {
        match code {
            0x00 => Self::Data,
            0x01 => Self::TxDelay,
            0x02 => Self::Persistence,
            0x03 => Self::SlotTime,
            0x04 => Self::TxTail,
            0x05 => Self::FullDuplex,
            0x06 => Self::SetHardware,
            0x0F => Self::Return,
            other => Self::Unknown(other),
        }
    }

    pub fn code(self) -> u8 {
        match self {
            Self::Data => 0x00,
            Self::TxDelay => 0x01,
            Self::Persistence => 0x02,
            Self::SlotTime => 0x03,
            Self::TxTail => 0x04,
            Self::FullDuplex => 0x05,
            Self::SetHardware => 0x06,
            Self::Return => 0x0F,
            Self::Unknown(c) => c,
        }
    }
}

/// One KISS frame: which of the TNC's ports, what it asks for, and its bytes.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Frame {
    pub port: u8,
    pub command: Command,
    pub payload: Vec<u8>,
}

impl Frame {
    /// The AX.25 frame this carries, if it carries one.
    pub fn data(&self) -> Option<&[u8]> {
        (self.command == Command::Data).then_some(&self.payload[..])
    }
}

/// Wrap a payload as a KISS frame, escaping the two bytes that cannot appear
/// raw inside one.
pub fn encode(port: u8, command: Command, payload: &[u8]) -> Vec<u8> {
    let mut out = Vec::with_capacity(payload.len() + 4);
    out.push(FEND);
    out.push((port & 0x0F) << 4 | (command.code() & 0x0F));
    for &b in payload {
        match b {
            FEND => out.extend_from_slice(&[FESC, TFEND]),
            FESC => out.extend_from_slice(&[FESC, TFESC]),
            other => out.push(other),
        }
    }
    out.push(FEND);
    out
}

/// An AX.25 frame on its way to a client.
pub fn data(port: u8, frame: &[u8]) -> Vec<u8> {
    encode(port, Command::Data, frame)
}

/// The longest frame assembled before the stream is treated as junk.
///
/// An AX.25 frame is at most 256 bytes of information over a path of eight
/// addresses; a kilobyte is well clear of that and stops a client sending
/// something other than KISS from growing the buffer without limit.
const MAX_FRAME: usize = 1024;

/// Bytes off a socket to whole frames.
///
/// Fed whatever arrived, which has nothing to do with where a frame starts:
/// a partial frame stays here until the rest of it comes.
#[derive(Default)]
pub struct Decoder {
    /// The frame being assembled, unescaped, with the type byte at the front.
    buf: Vec<u8>,
    /// Whether a delimiter has been seen, so bytes before the first one are
    /// dropped rather than read as the front of a frame.
    started: bool,
    /// Whether the last byte was the escape.
    escaped: bool,
    /// Frames thrown away for running past [`MAX_FRAME`].
    overlong: u64,
}

impl Decoder {
    pub fn new() -> Self {
        Self::default()
    }

    /// Frames dropped for being longer than any AX.25 frame can be, which is
    /// how a client sending something that is not KISS shows up.
    pub fn overlong(&self) -> u64 {
        self.overlong
    }

    /// Feed bytes, appending every frame that closed inside them.
    pub fn push(&mut self, bytes: &[u8], out: &mut Vec<Frame>) {
        for &b in bytes {
            match b {
                FEND => {
                    if let Some(f) = self.take() {
                        out.push(f);
                    }
                    self.started = true;
                    self.escaped = false;
                    self.buf.clear();
                }
                _ if !self.started => {}
                FESC => self.escaped = true,
                _ => {
                    // An escape followed by anything else is undefined. The
                    // byte is kept as it arrived rather than dropped, so a
                    // client with a broken escaper loses its check sequence
                    // and the frame, not the frame after it as well.
                    let b = match (self.escaped, b) {
                        (true, TFEND) => FEND,
                        (true, TFESC) => FESC,
                        (_, other) => other,
                    };
                    self.escaped = false;
                    if self.buf.len() >= MAX_FRAME {
                        self.overlong += 1;
                        self.started = false;
                        self.buf.clear();
                        continue;
                    }
                    self.buf.push(b);
                }
            }
        }
    }

    /// The assembled frame, if there is one. Two delimiters in a row are an
    /// empty frame and mean nothing, which is what a client sends to shake
    /// a half-written frame out of the TNC.
    fn take(&mut self) -> Option<Frame> {
        let bytes = std::mem::take(&mut self.buf);
        let (&kind, payload) = bytes.split_first()?;
        Some(Frame {
            port: kind >> 4,
            command: match kind {
                0xFF => Command::Return,
                _ => Command::from_code(kind & 0x0F),
            },
            payload: payload.to_vec(),
        })
    }
}

/// What a client has told the TNC about keying up.
///
/// Held in the units the operator and the modulator think in rather than the
/// bytes the wire carries, because every one of them is a count of 10 ms
/// slots that nothing outside this file should have to know about.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Params {
    /// Flags in front of a frame, for the far end's clock to settle on.
    pub txdelay_ms: u32,
    /// Carrier held after a frame.
    pub txtail_ms: u32,
    /// The p of p-persistent CSMA.
    pub persistence: f32,
    pub slot_ms: u32,
    pub full_duplex: bool,
}

impl Default for Params {
    fn default() -> Self {
        // The defaults every TNC ships with: 500 ms of flags, p = 0.25 and a
        // 100 ms slot.
        Self { txdelay_ms: 500, txtail_ms: 30, persistence: 0.25, slot_ms: 100, full_duplex: false }
    }
}

impl Params {
    /// Apply a command frame, answering whether it was one of the settings.
    pub fn apply(&mut self, f: &Frame) -> bool {
        let Some(&v) = f.payload.first() else { return false };
        match f.command {
            Command::TxDelay => self.txdelay_ms = u32::from(v) * 10,
            Command::TxTail => self.txtail_ms = u32::from(v) * 10,
            Command::Persistence => self.persistence = (f32::from(v) + 1.0) / 256.0,
            Command::SlotTime => self.slot_ms = u32::from(v) * 10,
            Command::FullDuplex => self.full_duplex = v != 0,
            Command::Data | Command::SetHardware | Command::Return | Command::Unknown(_) => {
                return false;
            }
        }
        true
    }

    /// How many HDLC flags the delay asks for at this baud rate. A flag is
    /// eight bits, so 500 ms at 1200 baud is 75 of them.
    pub fn lead_flags(&self, baud: f64) -> usize {
        ((f64::from(self.txdelay_ms) / 1000.0 * baud) / 8.0).round() as usize
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A frame whose bytes include both of the ones KISS cannot send raw.
    fn awkward() -> Vec<u8> {
        vec![0x01, FEND, 0x02, FESC, FEND, FESC, 0x03]
    }

    #[test]
    fn a_frame_survives_the_escaping() {
        let wire = data(0, &awkward());
        // Seven bytes, three of which are escaped into two, plus the two
        // delimiters and the type byte.
        assert_eq!(wire.len(), 7 + 4 + 3);
        assert_eq!(wire.iter().filter(|&&b| b == FEND).count(), 2, "a delimiter leaked");
        let mut d = Decoder::new();
        let mut out = Vec::new();
        d.push(&wire, &mut out);
        assert_eq!(out.len(), 1);
        assert_eq!(out[0], Frame { port: 0, command: Command::Data, payload: awkward() });
    }

    /// A socket read has nothing to do with a frame boundary.
    #[test]
    fn frames_are_assembled_across_reads() {
        let mut wire = Vec::new();
        for n in 0..3u8 {
            wire.extend(data(n, &[n, FESC, n]));
        }
        // Without its closing delimiter, which is the state a socket is in
        // between a client's write and the next one.
        let end = wire.pop().expect("the wire ends in a delimiter");
        let mut d = Decoder::new();
        let mut out = Vec::new();
        for chunk in wire.chunks(3) {
            d.push(chunk, &mut out);
        }
        assert_eq!(out.len(), 2, "the last frame has not closed yet");
        d.push(&[end], &mut out);
        assert_eq!(out.len(), 3);
        let ports: Vec<u8> = out.iter().map(|f| f.port).collect();
        assert_eq!(ports, [0, 1, 2]);
        assert_eq!(out[2].payload, vec![2, FESC, 2]);
    }

    /// Junk in front of the first delimiter is not the front of a frame, and
    /// empty frames are how a client clears the line.
    #[test]
    fn leading_junk_and_empty_frames_produce_nothing() {
        let mut d = Decoder::new();
        let mut out = Vec::new();
        d.push(b"not kiss at all", &mut out);
        d.push(&[FEND, FEND, FEND], &mut out);
        assert_eq!(out.len(), 0);
        d.push(&data(0, b"hello"), &mut out);
        assert_eq!(out.len(), 1);
        assert_eq!(out[0].data(), Some(&b"hello"[..]));
    }

    /// A stream that is not KISS at all must not grow the buffer forever.
    #[test]
    fn an_overlong_frame_is_dropped() {
        let mut d = Decoder::new();
        let mut out = Vec::new();
        d.push(&[FEND], &mut out);
        d.push(&vec![0x41; MAX_FRAME * 2], &mut out);
        d.push(&[FEND], &mut out);
        assert_eq!(out.len(), 0, "an overlong frame was accepted");
        assert_eq!(d.overlong(), 1);
        // And the decoder still reads the next frame.
        d.push(&data(0, b"after"), &mut out);
        assert_eq!(out.len(), 1);
        assert_eq!(out[0].data(), Some(&b"after"[..]));
    }

    /// The four settings a client sends before it transmits, in the units
    /// the specification gives them in.
    #[test]
    fn the_commands_a_client_sends_are_read() {
        let mut wire = Vec::new();
        for (c, v) in [
            (Command::TxDelay, 30u8),
            (Command::Persistence, 63),
            (Command::SlotTime, 10),
            (Command::FullDuplex, 1),
        ] {
            wire.extend(encode(0, c, &[v]));
        }
        let mut d = Decoder::new();
        let mut out = Vec::new();
        d.push(&wire, &mut out);
        assert_eq!(out.len(), 4);
        let mut p = Params::default();
        assert_eq!(out.iter().filter(|f| p.apply(f)).count(), 4);
        assert_eq!(p.txdelay_ms, 300);
        assert_eq!(p.slot_ms, 100);
        assert!((p.persistence - 0.25).abs() < 1e-6);
        assert!(p.full_duplex);
        // 300 ms of flags at 1200 baud is 45 of them.
        assert_eq!(p.lead_flags(1200.0), 45);
        assert_eq!(Params::default().lead_flags(1200.0), 75);
    }

    /// `0xFF` is a whole byte, not a port and a command.
    #[test]
    fn leaving_kiss_mode_is_not_a_port_fifteen_data_frame() {
        let mut d = Decoder::new();
        let mut out = Vec::new();
        d.push(&[FEND, 0xFF, FEND], &mut out);
        assert_eq!(out.len(), 1);
        assert_eq!(out[0].command, Command::Return);
        assert_eq!(out[0].data(), None);
        assert!(!Params::default().apply(&out[0]));
    }
}
