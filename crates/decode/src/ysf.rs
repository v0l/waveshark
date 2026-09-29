//! Yaesu System Fusion: C4FM frames of 100 ms, a frame information channel
//! behind Golay and a convolutional code, and callsigns in the data channel.

use common::packet::{Entity, Link, Party, Proto};
use dsp::conv::{self, Ends, Viterbi};
use dsp::m17::fec::{golay_decode, golay_encode};

pub const SYNC: u64 = 0xD4_71C9_634D;

pub const SYNC_BITS: usize = 40;

pub const FICH_BITS: usize = 200;

pub const FRAME_BITS: usize = 960;

pub const FRAME_DIBITS: usize = FRAME_BITS / 2;

pub const CALLSIGN: usize = 10;

const CODE: conv::Code = conv::M17;

const WHITENING: [u8; 20] = [
    0x93, 0xd7, 0x51, 0x21, 0x9c, 0x2f, 0x6c, 0xd0, 0xef, 0x0f, 0xf8, 0x3d, 0xf1, 0x73, 0x20, 0x94,
    0xed, 0x1e, 0x7c, 0xd8,
];

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Kind {
    Header,
    Communications,
    Terminator,
    Test,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Mode {
    VoiceData1,
    Data,
    VoiceData2,
    Voice,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Fich {
    pub kind: Kind,
    pub frame: u8,
    pub frames: u8,
    pub mode: Mode,
    pub group: u8,
    pub raw: [u8; 4],
}

impl Fich {
    fn of(raw: [u8; 4]) -> Self {
        Self {
            kind: match raw[0] >> 6 {
                0 => Kind::Header,
                1 => Kind::Communications,
                2 => Kind::Terminator,
                _ => Kind::Test,
            },
            frame: raw[1] >> 3 & 7,
            frames: raw[1] & 7,
            mode: match raw[2] & 3 {
                0 => Mode::VoiceData1,
                1 => Mode::Data,
                2 => Mode::VoiceData2,
                _ => Mode::Voice,
            },
            group: raw[3] & 0x7f,
            raw,
        }
    }
}

fn crc_ok(bytes: &[u8]) -> bool {
    let (data, crc) = bytes.split_at(bytes.len() - 2);
    crate::bits::crc16(data, 0x1021, 0) ^ 0xffff == u16::from_be_bytes([crc[0], crc[1]])
}

fn with_crc(data: &[u8]) -> Vec<u8> {
    let mut out = data.to_vec();
    out.extend((crate::bits::crc16(data, 0x1021, 0) ^ 0xffff).to_be_bytes());
    out
}

fn interleaved(rows: usize, k: usize) -> usize {
    let (i, j) = (k / rows, k % rows);
    40 * j + 2 * i
}

fn uncode(air: &[bool], rows: usize, bits: usize) -> Vec<bool> {
    let mut soft = Vec::with_capacity(2 * rows * 20);
    for k in 0..rows * 20 {
        let n = interleaved(rows, k);
        for b in [air[n], air[n + 1]] {
            soft.push(if b { -1.0 } else { 1.0 });
        }
    }
    Viterbi::decode_block(CODE, &soft, conv::P_1_2, bits + 4, Ends::Zero)
        .into_iter()
        .take(bits)
        .map(|b| b != 0)
        .collect()
}

fn code(data: &[bool], rows: usize) -> Vec<bool> {
    let mut with_tail: Vec<u8> = data.iter().map(|b| u8::from(*b)).collect();
    with_tail.extend([0u8; 4]);
    let coded = conv::Encoder::new(CODE).punctured(&with_tail, conv::P_1_2);
    let mut air = vec![false; rows * 40];
    for k in 0..rows * 20 {
        let n = interleaved(rows, k);
        air[n] = coded[2 * k] != 0;
        air[n + 1] = coded[2 * k + 1] != 0;
    }
    air
}

fn bytes_of(bits: &[bool]) -> Vec<u8> {
    bits.chunks(8).map(|c| c.iter().fold(0u8, |a, b| a << 1 | u8::from(*b))).collect()
}

fn bits_of(bytes: &[u8]) -> Vec<bool> {
    bytes.iter().flat_map(|b| (0..8).rev().map(move |k| b >> k & 1 == 1)).collect()
}

pub fn fich(air: &[bool]) -> Option<Fich> {
    let bits = uncode(air.get(..FICH_BITS)?, 5, 96);
    let mut data = 0u64;
    for word in bits.chunks(24) {
        let w = word.iter().fold(0u32, |a, b| a << 1 | u32::from(*b));
        let (d, _) = golay_decode(w)?;
        data = data << 12 | u64::from(d);
    }
    let bytes = data.to_be_bytes();
    let six = &bytes[2..8];
    crc_ok(six).then(|| Fich::of([six[0], six[1], six[2], six[3]]))
}

pub fn fich_air(raw: [u8; 4]) -> Vec<bool> {
    let six = with_crc(&raw);
    let data = six.iter().fold(0u64, |a, b| a << 8 | u64::from(*b));
    let mut bits = Vec::with_capacity(96);
    for q in 0..4 {
        let w = golay_encode((data >> (36 - 12 * q) & 0xfff) as u16);
        bits.extend((0..24).rev().map(|k| w >> k & 1 == 1));
    }
    code(&bits, 5)
}

fn gather(payload: &[bool], per_block: usize) -> Vec<bool> {
    (0..5).flat_map(|b| payload[144 * b..144 * b + per_block].iter().copied()).collect()
}

fn scatter(payload: &mut [bool], dch: &[bool], per_block: usize) {
    for b in 0..5 {
        payload[144 * b..144 * b + per_block]
            .copy_from_slice(&dch[per_block * b..per_block * (b + 1)]);
    }
}

fn whiten(bytes: &mut [u8]) {
    for (b, w) in bytes.iter_mut().zip(WHITENING) {
        *b ^= w;
    }
}

pub fn header_callsigns(payload: &[bool]) -> Option<([u8; CALLSIGN], [u8; CALLSIGN])> {
    let bits = uncode(&gather(payload.get(..720)?, 72), 9, 176);
    let mut bytes = bytes_of(&bits);
    if !crc_ok(&bytes[..22]) {
        return None;
    }
    whiten(&mut bytes[..20]);
    Some((bytes[..10].try_into().ok()?, bytes[10..20].try_into().ok()?))
}

pub fn vd2_data(payload: &[bool]) -> Option<[u8; CALLSIGN]> {
    let bits = uncode(&gather(payload.get(..720)?, 40), 5, 96);
    let mut bytes = bytes_of(&bits);
    if !crc_ok(&bytes[..12]) {
        return None;
    }
    whiten(&mut bytes[..10]);
    bytes[..10].try_into().ok()
}

pub fn header_payload(dest: &str, source: &str) -> Vec<bool> {
    let mut data = [b' '; 20];
    for (d, s) in data[..10].iter_mut().zip(dest.bytes()) {
        *d = s;
    }
    for (d, s) in data[10..].iter_mut().zip(source.bytes()) {
        *d = s;
    }
    whiten(&mut data);
    let dch = code(&bits_of(&with_crc(&data)), 9);
    let mut payload = vec![false; 720];
    scatter(&mut payload, &dch, 72);
    payload
}

pub fn vd2_payload(field: &str) -> Vec<bool> {
    let mut data = [b' '; 10];
    for (d, s) in data.iter_mut().zip(field.bytes()) {
        *d = s;
    }
    whiten(&mut data);
    let dch = code(&bits_of(&with_crc(&data)), 5);
    let mut payload = vec![false; 720];
    scatter(&mut payload, &dch, 40);
    payload
}

pub fn frame_air(fich_raw: [u8; 4], payload: &[bool]) -> Vec<bool> {
    let mut out: Vec<bool> = (0..SYNC_BITS).rev().map(|k| SYNC >> k & 1 == 1).collect();
    out.extend(fich_air(fich_raw));
    out.extend_from_slice(payload);
    out
}

#[derive(Clone, Debug, PartialEq)]
pub struct Frame {
    pub at: usize,
    pub fich: Fich,
    pub dest: Option<String>,
    pub source: Option<String>,
}

fn field(bytes: &[u8]) -> Option<String> {
    let s: String = bytes.iter().map(|b| *b as char).collect();
    let s = s.trim();
    (!s.is_empty() && s.chars().all(|c| c.is_ascii_graphic())).then(|| s.to_string())
}

pub fn read_frame(air: &[bool], at: usize) -> Option<Frame> {
    let fich = fich(air.get(SYNC_BITS..SYNC_BITS + FICH_BITS)?)?;
    let payload = air.get(SYNC_BITS + FICH_BITS..FRAME_BITS)?;
    let (mut dest, mut source) = (None, None);
    match (fich.kind, fich.mode) {
        (Kind::Header | Kind::Terminator, _) => {
            if let Some((d, s)) = header_callsigns(payload) {
                dest = field(&d);
                source = field(&s);
            }
        }
        (Kind::Communications, Mode::VoiceData2) => match fich.frame {
            0 => dest = vd2_data(payload).and_then(|d| field(&d)),
            1 => source = vd2_data(payload).and_then(|d| field(&d)),
            _ => {}
        },
        _ => {}
    }
    Some(Frame { at, fich, dest, source })
}

pub const TAG: [u8; 2] = *b"YF";

pub fn encode_frame(f: &Frame) -> Vec<u8> {
    let mut v = TAG.to_vec();
    v.extend(f.fich.raw);
    for part in [&f.dest, &f.source] {
        let mut b = [0u8; CALLSIGN];
        if let Some(s) = part {
            for (d, c) in b.iter_mut().zip(s.bytes()) {
                *d = c;
            }
        }
        v.extend(b);
    }
    v
}

pub fn read(bytes: &[u8]) -> Option<Proto> {
    if bytes.len() != 2 + 4 + 2 * CALLSIGN || bytes[..2] != TAG {
        return None;
    }
    let fich = Fich::of(bytes[2..6].try_into().ok()?);
    let text = |b: &[u8]| {
        let s: String = b.iter().take_while(|c| **c != 0).map(|c| *c as char).collect();
        (!s.is_empty()).then_some(s)
    };
    let (dest, source) = (text(&bytes[6..16]), text(&bytes[16..26]));
    let kind = match fich.kind {
        Kind::Header => "header",
        Kind::Communications => "voice",
        Kind::Terminator => "terminator",
        Kind::Test => "test",
    };
    let mut p = Proto::new("ysf", kind);
    if let Some(s) = &source {
        p = p.by(Entity::call("ysf", s.clone()));
    }
    if source.is_some() || dest.is_some() {
        let to = dest.map_or_else(Party::broadcast, Party::unit);
        let from = source.map(Party::unit);
        p = p.between(Link { from, to: Some(to) });
    }
    Some(p)
}

pub struct Framer {
    marks: Vec<f32>,
    base: usize,
    scan: usize,
    polarity: Option<bool>,
}

impl Default for Framer {
    fn default() -> Self {
        Self::new()
    }
}

pub const SYNC_TOLERANCE: usize = 2;

const WINDOW: usize = 2 * FRAME_DIBITS;

fn sync_dibits() -> [u8; SYNC_BITS / 2] {
    std::array::from_fn(|k| (SYNC >> (SYNC_BITS - 2 - 2 * k) & 3) as u8)
}

impl Framer {
    pub fn new() -> Self {
        Self { marks: Vec::new(), base: 0, scan: 0, polarity: None }
    }

    pub fn reset(&mut self) {
        *self = Self::new();
    }

    fn dibit(level: u8, flip: bool) -> u8 {
        match if flip { 3 - level } else { level } {
            3 => 1,
            2 => 0,
            1 => 2,
            _ => 3,
        }
    }

    pub fn push(&mut self, syms: &[f32], out: &mut Vec<Frame>) {
        self.marks.extend_from_slice(syms);
        if self.marks.len() < WINDOW {
            return;
        }
        let Some(levels) = dsp::c4fm::slice(&self.marks) else {
            return;
        };
        let sync = sync_dibits();
        let mut i = self.scan.saturating_sub(self.base);
        while i + FRAME_DIBITS <= levels.len() {
            let polarities: [bool; 2] = match self.polarity {
                Some(p) => [p, p],
                None => [false, true],
            };
            let mut read = None;
            for flip in polarities {
                let wrong = sync
                    .iter()
                    .enumerate()
                    .filter(|(k, d)| Self::dibit(levels[i + k], flip) != **d)
                    .count();
                if wrong > SYNC_TOLERANCE {
                    continue;
                }
                let air: Vec<bool> = levels[i..i + FRAME_DIBITS]
                    .iter()
                    .flat_map(|l| {
                        let d = Self::dibit(*l, flip);
                        [d & 2 != 0, d & 1 != 0]
                    })
                    .collect();
                if let Some(f) = read_frame(&air, self.base + i) {
                    read = Some((flip, f));
                    break;
                }
            }
            match read {
                Some((flip, frame)) => {
                    self.polarity = Some(flip);
                    out.push(frame);
                    i += FRAME_DIBITS;
                }
                None => i += 1,
            }
        }
        self.scan = self.base + i;
        let keep = WINDOW.min(self.marks.len());
        let drop = (self.marks.len() - keep).min(i);
        self.marks.drain(..drop);
        self.base += drop;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_frame_information_channel_survives_the_round_trip() {
        let raw = [0x24, 0x06, 0x52, 0x00];
        let got = fich(&fich_air(raw)).expect("a FICH");
        assert_eq!(got.raw, raw);
        assert_eq!((got.kind, got.frames, got.mode), (Kind::Header, 6, Mode::VoiceData2));
        let mut air = fich_air(raw);
        air[7] = !air[7];
        air[100] = !air[100];
        assert_eq!(fich(&air).map(|f| f.raw), Some(raw));
    }

    #[test]
    fn callsigns_come_back_out_of_a_header_and_the_data_channel() {
        let head = frame_air([0x24, 0x06, 0x52, 0x00], &header_payload("ALL", "G1RCE"));
        let f = read_frame(&head, 0).expect("a header");
        assert_eq!((f.dest.as_deref(), f.source.as_deref()), (Some("ALL"), Some("G1RCE")));
        let comms = frame_air([0x64, 0x0e, 0x52, 0x00], &vd2_payload("G1RCE"));
        let f = read_frame(&comms, 0).expect("a frame");
        assert_eq!((f.fich.kind, f.fich.frame), (Kind::Communications, 1));
        assert_eq!(f.source.as_deref(), Some("G1RCE"));
        let row = read(&encode_frame(&f)).expect("a row");
        assert_eq!((row.id, row.kind), ("ysf", "voice"));
        assert_eq!(row.subject.map(|e| e.id.to_string()).as_deref(), Some("G1RCE"));
    }
}
