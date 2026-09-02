//! What an M17 frame says: callsigns, stream type, position, packet text.
//!
//! The link layer is `dsp::m17`, and nothing that decides whether a frame
//! happened at all is repeated here. What arrives has already passed its
//! error correction, so this file is tables and reassembly: a base-40
//! alphabet, a two byte type field, a metadata field whose meaning depends on
//! two bits of that type, and the counters that put a packet's 25 byte
//! fragments back in order.
//!
//! # Late listening
//!
//! A transmission opens with one link setup frame and then never repeats it,
//! so a receiver that tuned in a second late would otherwise never learn who
//! is talking. M17 solves that by sending a sixth of the link setup frame in
//! every stream frame, in the link information channel, under a Golay code of
//! its own. Six frames, 240 ms, and a late listener has rebuilt the whole
//! thing, CRC included. [`Assembler`] does that rebuilding, and treats a
//! rebuilt link setup exactly like a received one: the CRC is what makes them
//! equivalent.

use common::Value;
use dsp::m17::{Body, Frame};

/// The M17 alphabet, ordered by value. A space is zero, so trailing spaces
/// cost nothing and a callsign is encoded shortest first.
const ALPHABET: &[u8; 40] = b" ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789-/.";

/// The first address that no sequence of nine alphabet characters reaches,
/// which is 40^9. Everything from here up is for applications to define.
const EXTENDED: u64 = 40u64.pow(9);

/// The whole 48 bit space set, which means the frame is for anyone.
const BROADCAST: u64 = 0xFFFF_FFFF_FFFF;

/// A 48 bit M17 address.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Address {
    /// Reserved, and transmitted by nothing.
    Empty,
    /// Any receiver.
    Broadcast,
    /// Up to nine characters of the M17 alphabet: usually a callsign, often a
    /// reflector name or a command like `ECHO` in a destination.
    Text(String),
    /// An address outside the encodable range. Kept as it arrived, because
    /// applications are free to define these and a receiver has no business
    /// inventing a name for one.
    Raw(u64),
}

impl Address {
    pub fn from_bytes(b: &[u8]) -> Self {
        let mut v = 0u64;
        for &x in b.iter().take(6) {
            v = v << 8 | u64::from(x);
        }
        Self::from_value(v)
    }

    pub fn from_value(mut v: u64) -> Self {
        match v {
            0 => Address::Empty,
            BROADCAST => Address::Broadcast,
            _ if v >= EXTENDED => Address::Raw(v),
            _ => {
                // Encoded last character first, so decoding walks back out in
                // the order the characters were written.
                let mut s = String::new();
                while v > 0 {
                    s.push(ALPHABET[(v % 40) as usize] as char);
                    v /= 40;
                }
                Address::Text(s)
            }
        }
    }

    /// Encode text back to an address. The transmit half, and what the tests
    /// build frames with.
    pub fn encode(text: &str) -> u64 {
        let mut v = 0u64;
        for c in text.bytes().take(9).rev() {
            let c = c.to_ascii_uppercase();
            let n = ALPHABET.iter().position(|&a| a == c).unwrap_or(0) as u64;
            v = v * 40 + n;
        }
        v
    }
}

impl std::fmt::Display for Address {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Address::Empty => write!(f, ""),
            Address::Broadcast => write!(f, "BROADCAST"),
            Address::Text(s) => write!(f, "{s}"),
            Address::Raw(v) => write!(f, "#{v:012X}"),
        }
    }
}

/// What the payload of a stream is.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DataType {
    Reserved,
    Data,
    /// Codec 2 at 3200 bits per second, filling all 128 payload bits.
    Voice,
    /// Codec 2 at 1600, leaving 64 bits a frame for anything else.
    VoiceData,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Encryption {
    None,
    Scrambler,
    Aes,
    Other,
}

/// The link setup frame: who is calling whom, how, and with what attached.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Lsf {
    pub bytes: [u8; 30],
}

impl Lsf {
    /// Read a link setup frame, refusing one whose CRC does not check.
    ///
    /// There is no other constructor on purpose. A callsign is the whole
    /// point of this frame, and a callsign taken from an unchecked frame is a
    /// station that was never on the air.
    pub fn new(bytes: [u8; 30]) -> Option<Self> {
        (dsp::m17::fec::crc16(&bytes) == 0).then_some(Self { bytes })
    }

    pub fn destination(&self) -> Address {
        Address::from_bytes(&self.bytes[0..6])
    }

    pub fn source(&self) -> Address {
        Address::from_bytes(&self.bytes[6..12])
    }

    pub fn type_field(&self) -> u16 {
        u16::from_be_bytes([self.bytes[12], self.bytes[13]])
    }

    pub fn meta(&self) -> &[u8] {
        &self.bytes[14..28]
    }

    /// True for a stream, false for a packet.
    pub fn is_stream(&self) -> bool {
        self.type_field() & 1 == 1
    }

    pub fn data_type(&self) -> DataType {
        match self.type_field() >> 1 & 3 {
            1 => DataType::Data,
            2 => DataType::Voice,
            3 => DataType::VoiceData,
            _ => DataType::Reserved,
        }
    }

    pub fn encryption(&self) -> Encryption {
        match self.type_field() >> 3 & 3 {
            0 => Encryption::None,
            1 => Encryption::Scrambler,
            2 => Encryption::Aes,
            _ => Encryption::Other,
        }
    }

    /// The two bits that say what the metadata field holds, which only mean
    /// anything when nothing is encrypted: with a cipher in use the same bits
    /// give the key length instead.
    pub fn meta_kind(&self) -> u16 {
        self.type_field() >> 5 & 3
    }

    /// Channel access number: a squelch code, effectively, for sharing a
    /// frequency between groups.
    pub fn can(&self) -> u8 {
        (self.type_field() >> 7 & 0xF) as u8
    }

    /// Whether the stream carries an ECDSA signature over its frames.
    pub fn signed(&self) -> bool {
        self.type_field() >> 11 & 1 == 1
    }

    /// What the metadata field carries, when it carries anything readable.
    pub fn metadata(&self) -> Option<Meta> {
        if self.encryption() != Encryption::None {
            return None;
        }
        match self.meta_kind() {
            0 => Meta::text(self.meta()),
            1 => Meta::position(self.meta()),
            2 => Meta::callsigns(self.meta()),
            _ => None,
        }
    }
}

/// A position, as a stream's metadata reports one.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Position {
    pub lat: f64,
    pub lon: f64,
    /// Metres, when the transmitter said the altitude was valid.
    pub altitude_m: Option<f64>,
    pub speed_kmh: Option<f64>,
    pub bearing_deg: Option<f64>,
    /// What sort of station this is: fixed, mobile or handheld.
    pub station: &'static str,
}

/// The readable interpretations of the 14 byte metadata field.
#[derive(Clone, Debug, PartialEq)]
pub enum Meta {
    /// One block of up to 13 bytes of a message that may run to four blocks.
    /// The control byte says which block this is and how many there are.
    Text { control: u8, text: String },
    Position(Position),
    /// A repeater or gateway naming the station it is relaying, and the
    /// reflector it came through.
    Callsigns { origin: Address, via: Option<Address> },
}

impl Meta {
    fn text(meta: &[u8]) -> Option<Self> {
        let control = meta[0];
        if control == 0 {
            return None;
        }
        let text: String = String::from_utf8_lossy(&meta[1..14]).trim_end().to_string();
        Some(Meta::Text { control, text })
    }

    fn callsigns(meta: &[u8]) -> Option<Self> {
        let origin = Address::from_bytes(&meta[0..6]);
        if origin == Address::Empty {
            return None;
        }
        let via = match Address::from_bytes(&meta[6..12]) {
            Address::Empty => None,
            a => Some(a),
        };
        Some(Meta::Callsigns { origin, via })
    }

    fn position(meta: &[u8]) -> Option<Self> {
        let validity = meta[1] >> 4;
        if validity & 0b1000 == 0 {
            // No fix. The rest of the field is zeroed by the transmitter, and
            // reporting the equator to nobody helps nobody.
            return None;
        }
        let signed24 = |b: &[u8]| -> f64 {
            let v = i32::from(b[0]) << 16 | i32::from(b[1]) << 8 | i32::from(b[2]);
            f64::from(if v & 0x80_0000 != 0 { v - 0x100_0000 } else { v })
        };
        // Both are binary fractions of a quarter and a half turn.
        let lat = signed24(&meta[3..6]) * 90.0 / 8_388_607.0;
        let lon = signed24(&meta[6..9]) * 180.0 / 8_388_607.0;
        let altitude_m = (validity & 0b0100 != 0)
            .then(|| f64::from(u16::from_be_bytes([meta[9], meta[10]])) * 0.5 - 500.0);
        let velocity = validity & 0b0010 != 0;
        let bearing_deg = velocity.then(|| {
            f64::from(u16::from(meta[1] & 1) << 8 | u16::from(meta[2]))
        });
        let speed_kmh = velocity
            .then(|| f64::from(u16::from(meta[11]) << 4 | u16::from(meta[12] >> 4)) * 0.5);
        let station = match meta[0] & 0xF {
            0 => "fixed",
            1 => "mobile",
            2 => "handheld",
            _ => "other",
        };
        Some(Meta::Position(Position { lat, lon, altitude_m, speed_kmh, bearing_deg, station }))
    }
}

/// The reserved packet protocol identifiers. Anything else is an application
/// defined type and is reported as its number.
pub fn packet_protocol(id: u8) -> Option<&'static str> {
    Some(match id {
        0x00 => "RAW",
        0x01 => "AX.25",
        0x02 => "APRS",
        0x03 => "6LoWPAN",
        0x04 => "IPv4",
        0x05 => "SMS",
        0x06 => "Winlink",
        _ => return None,
    })
}

/// What a transmission amounts to, once its frames have been put together.
#[derive(Clone, Debug, PartialEq)]
pub enum Event {
    /// A link setup was heard, or rebuilt from six stream frames. Reported as
    /// soon as it is known rather than at the end of the transmission, since
    /// this is the row an operator wants while somebody is still talking.
    LinkSetup { lsf: Lsf, late: bool },
    /// A complete packet whose CRC checked.
    Packet { lsf: Option<Lsf>, data: Vec<u8> },
    /// A stream ended, either because the transmitter said so or because it
    /// stopped being heard.
    Stream { lsf: Option<Lsf>, frames: u32, complete: bool },
}

impl Event {
    /// The bytes this travels on the packet bus as: a tag, the link setup
    /// frame when there is one, then whatever the variant carries.
    pub fn to_bytes(&self) -> Vec<u8> {
        let (tag, lsf, rest): (u8, Option<&Lsf>, Vec<u8>) = match self {
            Event::LinkSetup { lsf, late } => (1, Some(lsf), vec![u8::from(*late)]),
            Event::Packet { lsf, data } => (2, lsf.as_ref(), data.clone()),
            Event::Stream { lsf, frames, complete } => {
                let mut v = frames.to_be_bytes().to_vec();
                v.push(u8::from(*complete));
                (3, lsf.as_ref(), v)
            }
        };
        let mut out = vec![tag, u8::from(lsf.is_some())];
        if let Some(l) = lsf {
            out.extend_from_slice(&l.bytes);
        }
        out.extend_from_slice(&rest);
        out
    }

    /// Read back what [`Event::to_bytes`] wrote.
    ///
    /// Strict about lengths on purpose. M17 lives in the amateur bands, which
    /// already carry APRS and, a little further up, paging, so where a frame
    /// was received does not identify it the way it does for AIS or Mode S.
    /// What identifies an M17 event is its shape: a known tag, an exact
    /// length for its variant, and a link setup frame whose CRC checks.
    pub fn parse(bytes: &[u8]) -> Option<Self> {
        let (&tag, rest) = bytes.split_first()?;
        let (&has_lsf, rest) = rest.split_first()?;
        let (lsf, rest) = if has_lsf == 1 {
            let (head, rest) = rest.split_at_checked(30)?;
            let mut b = [0u8; 30];
            b.copy_from_slice(head);
            (Lsf::new(b), rest)
        } else {
            (None, rest)
        };
        if has_lsf > 1 {
            return None;
        }
        Some(match (tag, rest.len()) {
            (1, 1) => Event::LinkSetup { lsf: lsf?, late: rest[0] == 1 },
            (2, 3..) => Event::Packet { lsf, data: rest.to_vec() },
            (3, 5) => Event::Stream {
                lsf,
                frames: u32::from_be_bytes(rest[..4].try_into().ok()?),
                complete: rest[4] == 1,
            },
            _ => return None,
        })
    }
}

/// Frames in, transmissions out.
///
/// One transmission at a time, which is what a channel carries: M17 is a
/// half duplex mode and two stations on one frequency talking over each other
/// is a collision, not something to be demultiplexed.
pub struct Assembler {
    /// Samples of quiet after which whatever is open is closed. A frame is
    /// 40 ms, so this is a few frames' worth.
    gap: u64,
    last_sample: u64,
    open: bool,
    lsf: Option<Lsf>,
    /// Reported so the same link setup is not announced twice.
    announced: bool,
    /// Link information chunks collected from stream frames, by counter.
    chunks: [Option<[u8; 5]>; 6],
    frames: u32,
    ended: bool,
    /// Packet fragments by frame counter, and the last frame's byte count.
    parts: Vec<Option<[u8; 25]>>,
    tail: Option<(u8, u8)>,
}

impl Assembler {
    pub fn new(rate: f64) -> Self {
        Self {
            gap: (rate * 0.2) as u64,
            last_sample: 0,
            open: false,
            lsf: None,
            announced: false,
            chunks: Default::default(),
            frames: 0,
            ended: false,
            parts: vec![None; 32],
            tail: None,
        }
    }

    /// Feed one frame, returning anything it completed.
    pub fn push(&mut self, frame: &Frame) -> Vec<Event> {
        let mut out = Vec::new();
        if self.open && frame.start_sample.saturating_sub(self.last_sample) > self.gap {
            out.extend(self.close());
        }
        self.last_sample = frame.start_sample;
        self.open = true;

        match &frame.body {
            Body::Lsf(bytes) => {
                // A link setup opens a transmission, so anything still open
                // belongs to the one before it.
                if self.frames > 0 || self.lsf.is_some() {
                    out.extend(self.close());
                    self.open = true;
                    self.last_sample = frame.start_sample;
                }
                if let Some(lsf) = Lsf::new(*bytes) {
                    out.push(Event::LinkSetup { lsf: lsf.clone(), late: false });
                    self.lsf = Some(lsf);
                    self.announced = true;
                }
            }
            Body::Stream { lich, number, last, .. } => {
                let cnt = (lich[5] >> 5) as usize;
                if cnt < 6 {
                    let mut chunk = [0u8; 5];
                    chunk.copy_from_slice(&lich[..5]);
                    self.chunks[cnt] = Some(chunk);
                }
                self.frames += 1;
                if !self.announced {
                    if let Some(lsf) = self.rebuild() {
                        out.push(Event::LinkSetup { lsf: lsf.clone(), late: true });
                        self.lsf = Some(lsf);
                        self.announced = true;
                    }
                }
                let _ = number;
                if *last {
                    self.ended = true;
                    out.extend(self.close());
                }
            }
            Body::Packet { data, eof, counter } => {
                if *eof {
                    // On the last frame the counter is a byte count rather
                    // than a position, so the frame's own place is wherever
                    // the ones before it left off.
                    let at = self.parts.iter().take_while(|p| p.is_some()).count();
                    if at < self.parts.len() {
                        self.parts[at] = Some(*data);
                        self.tail = Some((at as u8, (*counter).clamp(1, 25)));
                    }
                    out.extend(self.close());
                } else if (*counter as usize) < self.parts.len() {
                    self.parts[*counter as usize] = Some(*data);
                    self.frames += 1;
                }
            }
            Body::Bert => {}
        }
        out
    }

    /// Close anything open if nothing has been heard for a while.
    ///
    /// Driven by the sample counter rather than a clock, so a recording
    /// replayed faster than real time behaves the same as a live receiver.
    pub fn poll(&mut self, now_sample: u64) -> Vec<Event> {
        if self.open && now_sample.saturating_sub(self.last_sample) > self.gap {
            return self.close();
        }
        Vec::new()
    }

    /// Rebuild the link setup frame from the chunks the stream carried.
    /// Returns nothing until all six are in and the CRC over them checks.
    fn rebuild(&self) -> Option<Lsf> {
        let mut bytes = [0u8; 30];
        for (i, chunk) in self.chunks.iter().enumerate() {
            bytes[i * 5..i * 5 + 5].copy_from_slice(chunk.as_ref()?);
        }
        Lsf::new(bytes)
    }

    fn close(&mut self) -> Vec<Event> {
        let mut out = Vec::new();
        let lsf = self.lsf.clone();
        if self.parts.iter().any(|p| p.is_some()) {
            if let Some(data) = self.packet() {
                out.push(Event::Packet { lsf: lsf.clone(), data });
            }
        } else if self.frames > 0 {
            out.push(Event::Stream { lsf, frames: self.frames, complete: self.ended });
        }
        self.open = false;
        self.lsf = None;
        self.announced = false;
        self.chunks = Default::default();
        self.frames = 0;
        self.ended = false;
        self.parts.iter_mut().for_each(|p| *p = None);
        self.tail = None;
        out
    }

    /// Join the packet's fragments and check the CRC that covers all of them.
    ///
    /// A packet with a frame missing is dropped rather than reported with a
    /// hole: the CRC is over the whole thing, so what would be reported is
    /// unverifiable, and a truncated message reads as a complete one.
    fn packet(&self) -> Option<Vec<u8>> {
        let (last, valid) = self.tail?;
        let mut data = Vec::new();
        for i in 0..=last as usize {
            let part = self.parts[i].as_ref()?;
            let take = if i == last as usize { valid as usize } else { 25 };
            data.extend_from_slice(&part[..take]);
        }
        if data.len() < 3 || dsp::m17::fec::crc16(&data) != 0 {
            return None;
        }
        data.truncate(data.len() - 2);
        Some(data)
    }
}

/// The fields a link setup frame contributes to a packet log row.
pub fn fields(lsf: &Lsf) -> Vec<(String, Value)> {
    let mut f: Vec<(String, Value)> = vec![
        ("from".into(), Value::Text(lsf.source().to_string())),
        ("to".into(), Value::Text(lsf.destination().to_string())),
        ("can".into(), Value::Int(i64::from(lsf.can()))),
    ];
    let mode = match (lsf.is_stream(), lsf.data_type()) {
        (false, _) => "packet",
        (true, DataType::Voice) => "voice",
        (true, DataType::VoiceData) => "voice+data",
        (true, DataType::Data) => "data",
        (true, DataType::Reserved) => "stream",
    };
    f.push(("mode".into(), Value::Text(mode.into())));
    if lsf.encryption() != Encryption::None {
        let e = match lsf.encryption() {
            Encryption::Scrambler => "scrambler",
            Encryption::Aes => "aes",
            _ => "other",
        };
        f.push(("encryption".into(), Value::Text(e.into())));
    }
    if lsf.signed() {
        f.push(("signed".into(), Value::Bool(true)));
    }
    match lsf.metadata() {
        Some(Meta::Text { text, .. }) if !text.is_empty() => {
            f.push(("message".into(), Value::Text(text)));
        }
        Some(Meta::Position(p)) => {
            f.push(("lat".into(), Value::Float((p.lat * 1e5).round() / 1e5)));
            f.push(("lon".into(), Value::Float((p.lon * 1e5).round() / 1e5)));
            f.push(("station".into(), Value::Text(p.station.into())));
            if let Some(a) = p.altitude_m {
                f.push(("altitude_m".into(), Value::Float(a)));
            }
            if let Some(s) = p.speed_kmh {
                f.push(("speed_kmh".into(), Value::Float(s)));
            }
            if let Some(b) = p.bearing_deg {
                f.push(("track_deg".into(), Value::Float(b)));
            }
        }
        Some(Meta::Callsigns { origin, via }) => {
            f.push(("originator".into(), Value::Text(origin.to_string())));
            if let Some(v) = via {
                f.push(("via".into(), Value::Text(v.to_string())));
            }
        }
        _ => {}
    }
    f
}

#[cfg(test)]
mod tests {
    use super::*;
    use dsp::m17::fec;

    fn lsf_of(dst: &str, src: &str, type_field: u16, meta: &[u8]) -> Lsf {
        let mut b = [0u8; 30];
        b[..6].copy_from_slice(&Address::encode(dst).to_be_bytes()[2..]);
        b[6..12].copy_from_slice(&Address::encode(src).to_be_bytes()[2..]);
        b[12..14].copy_from_slice(&type_field.to_be_bytes());
        b[14..14 + meta.len()].copy_from_slice(meta);
        let crc = fec::crc16(&b[..28]);
        b[28..].copy_from_slice(&crc.to_be_bytes());
        Lsf::new(b).expect("built frame does not check")
    }

    /// The worked example from the specification's address encoding appendix.
    #[test]
    fn the_callsign_encoding_matches_the_published_example() {
        assert_eq!(Address::encode("AB1CD"), 0x9F_DD51);
        assert_eq!(Address::from_value(0x9F_DD51), Address::Text("AB1CD".into()));
        assert_eq!(Address::from_value(BROADCAST), Address::Broadcast);
        assert_eq!(Address::from_value(0), Address::Empty);
        // Trailing spaces are zeros in the least significant digits, so they
        // encode to nothing at all.
        assert_eq!(Address::encode("ABC   "), Address::encode("ABC"));
        // A reflector module, which is what a destination usually is.
        let m17 = Address::encode("M17-M17 C");
        assert_eq!(Address::from_value(m17), Address::Text("M17-M17 C".into()));
        // Above 40^9 nothing decodes, and inventing a name would be worse
        // than saying so.
        assert!(matches!(Address::from_value(EXTENDED + 1), Address::Raw(_)));
    }

    #[test]
    fn a_frame_that_fails_its_crc_yields_no_callsign() {
        let mut lsf = lsf_of("BROADCAST", "M0ABC", 0x0005, &[]);
        lsf.bytes[7] ^= 0x20;
        assert_eq!(Lsf::new(lsf.bytes), None);
    }

    #[test]
    fn the_type_field_says_how_to_read_the_stream() {
        // Stream, voice, no encryption, CAN 7.
        let lsf = lsf_of("M17-M17 C", "M0ABC", 1 | 2 << 1 | 7 << 7, &[]);
        assert!(lsf.is_stream());
        assert_eq!(lsf.data_type(), DataType::Voice);
        assert_eq!(lsf.encryption(), Encryption::None);
        assert_eq!(lsf.can(), 7);
        assert!(!lsf.signed());

        // AES, which is where the same two bits mean a key length instead of
        // a metadata format, so no metadata is claimed.
        let enc = lsf_of("ALL", "M0ABC", 1 | 2 << 1 | 2 << 3 | 1 << 5, &[9; 14]);
        assert_eq!(enc.encryption(), Encryption::Aes);
        assert_eq!(enc.metadata(), None);
    }

    #[test]
    fn a_position_comes_out_of_the_metadata() {
        // Handheld, OpenRTX, everything valid, near 51.5 N 0.13 W.
        let (lat, lon) = (51.5074_f64, -0.1278_f64);
        let enc = |v: f64, full: f64| -> [u8; 3] {
            let n = (v / full * 8_388_607.0).round() as i32;
            let u = n as u32 & 0xFF_FFFF;
            [(u >> 16) as u8, (u >> 8) as u8, u as u8]
        };
        let mut meta = [0u8; 14];
        meta[0] = 1 << 4 | 2;
        meta[1] = 0b1110 << 4 | 0b010 << 1 | 0; // valid position, altitude, velocity
        meta[2] = 90; // bearing, due east
        meta[3..6].copy_from_slice(&enc(lat, 90.0));
        meta[6..9].copy_from_slice(&enc(lon, 180.0));
        meta[9..11].copy_from_slice(&1_100u16.to_be_bytes()); // 50 m
        meta[11] = 0x03;
        meta[12] = 0x20; // 50.0 km/h

        let lsf = lsf_of("ALL", "M0ABC", 1 | 2 << 1 | 1 << 5, &meta);
        let Some(Meta::Position(p)) = lsf.metadata() else { panic!("no position") };
        assert!((p.lat - lat).abs() < 1e-4, "latitude came out at {}", p.lat);
        assert!((p.lon - lon).abs() < 1e-4, "longitude came out at {}", p.lon);
        assert_eq!(p.altitude_m, Some(50.0));
        assert_eq!(p.speed_kmh, Some(25.0));
        assert_eq!(p.bearing_deg, Some(90.0));
        assert_eq!(p.station, "handheld");
    }

    #[test]
    fn an_invalid_fix_is_not_reported_as_the_equator() {
        let mut meta = [0u8; 14];
        meta[1] = 0b0100 << 4;
        let lsf = lsf_of("ALL", "M0ABC", 1 | 2 << 1 | 1 << 5, &meta);
        assert_eq!(lsf.metadata(), None);
    }

    fn frame(body: Body, at: u64) -> Frame {
        Frame { body, ber: 0.0, correlation: 1.0, evm: 0.0, start_sample: at }
    }

    fn stream_frame(lsf: &Lsf, cnt: u8, number: u16, last: bool, at: u64) -> Frame {
        let mut lich = [0u8; 6];
        lich[..5].copy_from_slice(&lsf.bytes[cnt as usize * 5..cnt as usize * 5 + 5]);
        lich[5] = cnt << 5;
        frame(
            Body::Stream { lich, lich_errors: 0, number, last, payload: [0; 16] },
            at,
        )
    }

    /// The point of the link information channel: a receiver that missed the
    /// link setup frame still learns who is talking, from six stream frames
    /// and the same CRC.
    #[test]
    fn a_late_listener_rebuilds_the_link_setup() {
        let lsf = lsf_of("M17-M17 C", "M0ABC", 1 | 2 << 1, &[]);
        let mut a = Assembler::new(48_000.0);
        let mut events = Vec::new();
        for n in 0..12u16 {
            let f = stream_frame(&lsf, (n % 6) as u8, n, n == 11, 1920 * u64::from(n));
            events.extend(a.push(&f));
        }
        let setup = events.iter().find_map(|e| match e {
            Event::LinkSetup { lsf, late } => Some((lsf.clone(), *late)),
            _ => None,
        });
        let (got, late) = setup.expect("the link setup was never rebuilt");
        assert_eq!(got, lsf);
        assert!(late, "a rebuilt link setup should say it was rebuilt");
        // Rebuilt at the sixth frame, not the twelfth.
        assert_eq!(events.len(), 2, "{events:?}");
        assert!(matches!(events[1], Event::Stream { frames: 12, complete: true, .. }));
    }

    #[test]
    fn a_stream_that_stops_being_heard_is_closed_anyway() {
        let lsf = lsf_of("ALL", "M0ABC", 1 | 2 << 1, &[]);
        let mut a = Assembler::new(48_000.0);
        a.push(&frame(Body::Lsf(lsf.bytes), 0));
        for n in 0..4u16 {
            a.push(&stream_frame(&lsf, (n % 6) as u8, n, false, 1920 * u64::from(n + 1)));
        }
        assert!(a.poll(1920 * 6).is_empty(), "closed while the stream was still running");
        let events = a.poll(48_000 * 3);
        assert!(matches!(events[..], [Event::Stream { frames: 4, complete: false, .. }]));
    }

    #[test]
    fn packet_fragments_are_joined_and_checked() {
        let mut data = vec![0x05u8]; // SMS
        data.extend_from_slice("A message long enough to need two frames".as_bytes());
        data.push(0);
        let crc = fec::crc16(&data);
        data.extend_from_slice(&crc.to_be_bytes());

        let lsf = lsf_of("ALL", "M0ABC", 0, &[]);
        let mut a = Assembler::new(48_000.0);
        a.push(&frame(Body::Lsf(lsf.bytes), 0));
        let mut events = Vec::new();
        for (i, chunk) in data.chunks(25).enumerate() {
            let mut part = [0u8; 25];
            part[..chunk.len()].copy_from_slice(chunk);
            let eof = (i + 1) * 25 >= data.len();
            let counter = if eof { chunk.len() as u8 } else { i as u8 };
            events.extend(a.push(&frame(
                Body::Packet { data: part, eof, counter },
                1920 * (i as u64 + 1),
            )));
        }
        let got = events.iter().find_map(|e| match e {
            Event::Packet { data, .. } => Some(data.clone()),
            _ => None,
        });
        let got = got.expect("no packet came out");
        assert_eq!(got[0], 0x05);
        assert_eq!(packet_protocol(got[0]), Some("SMS"));
        assert_eq!(&got[1..got.len() - 1], "A message long enough to need two frames".as_bytes());
    }

    /// A packet missing a fragment cannot be checked, and a message with a
    /// hole in it reads exactly like a message without one.
    #[test]
    fn a_packet_with_a_frame_missing_is_dropped() {
        let mut data = vec![0x05u8];
        data.extend_from_slice(&[b'x'; 60]);
        let crc = fec::crc16(&data);
        data.extend_from_slice(&crc.to_be_bytes());

        let mut a = Assembler::new(48_000.0);
        let mut events = Vec::new();
        for (i, chunk) in data.chunks(25).enumerate() {
            if i == 1 {
                continue;
            }
            let mut part = [0u8; 25];
            part[..chunk.len()].copy_from_slice(chunk);
            let eof = (i + 1) * 25 >= data.len();
            let counter = if eof { chunk.len() as u8 } else { i as u8 };
            events.extend(a.push(&frame(
                Body::Packet { data: part, eof, counter },
                1920 * (i as u64 + 1),
            )));
        }
        assert!(events.is_empty(), "an incomplete packet was reported: {events:?}");
    }

    #[test]
    fn an_event_survives_the_trip_over_the_bus() {
        let lsf = lsf_of("M17-M17 C", "M0ABC", 1 | 2 << 1 | 3 << 7, &[]);
        for e in [
            Event::LinkSetup { lsf: lsf.clone(), late: true },
            Event::Packet { lsf: Some(lsf.clone()), data: vec![5, b'h', b'i', 0] },
            Event::Stream { lsf: Some(lsf.clone()), frames: 250, complete: true },
            Event::Stream { lsf: None, frames: 3, complete: false },
        ] {
            assert_eq!(Event::parse(&e.to_bytes()), Some(e));
        }
    }
}
