//! DRM's two signalling channels, ETSI ES 201 980: the fast access channel
//! that says how the multiplex is put together, and the service description
//! channel that names the services in it.
//!
//! Cells in, fields out. The waveform, the pilots and where on the grid each
//! channel sits are `dsp::drm`; what happens between a cell and a field is
//! the same for both channels and is here: a 4-QAM cell is two soft bits,
//! the bits are put back in order by the standard's multiplicative
//! interleaver, the rate 1/6 mother code is depunctured and decoded, a
//! nine bit sequence is taken back off the result, and what is left is
//! checked and read.
//!
//! The multiplex itself is not decoded. Its audio is xHE-AAC, which nothing
//! here can link, so what this reads is what a receiver needs to list the
//! services: the mode, the occupancy, the service identifiers and languages,
//! and the labels the transmitter gives them.

use crate::bits::{crc8, crc16};
use crate::dab::ProgrammeType;
use common::C32;
use dsp::conv;
use dsp::drm::{Mode, Occupancy};

/// Bits of information the fast access channel carries in a frame.
pub const FAC_BITS: usize = 64;
/// The check that follows them.
pub const FAC_CRC: usize = 8;

/// The fast access channel's puncturing, clause 7.2.1.1: five of every
/// eighteen mother bits are sent, which is rate 3/5 over three input bits.
const FAC_PUNCTURE: [u8; 18] = [1, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0];

/// The description channel's puncturing at its only rate, 1/2.
const SDC_PUNCTURE: [u8; 6] = [1, 1, 0, 0, 0, 0];

/// The interleaver's multiplier for the fast access channel and for the
/// description channel in 4-QAM, clause 7.3.3.
const T0: usize = 21;

/// The generator of the bit permutation both channels use, clause 7.3.3:
/// each place is the one before it times t0 plus q, modulo the next power of
/// two up from the block, and a place outside the block is taken again until
/// it lands inside.
fn interleave(size: usize, t0: usize) -> Vec<usize> {
    let mut s = 1usize;
    while s < size {
        s <<= 1;
    }
    let q = s / 4 - 1;
    let mut map = Vec::with_capacity(size);
    map.push(0usize);
    for i in 1..size {
        let mut next = (t0 * map[i - 1] + q) % s;
        while next >= size {
            next = (t0 * next + q) % s;
        }
        map.push(next);
    }
    map
}

/// The energy dispersal both channels use, clause 7.2.2: a nine stage
/// register of ones, tapped at nine and five.
fn dispersal(len: usize) -> Vec<u8> {
    let mut reg = [1u8; 9];
    let mut out = Vec::with_capacity(len);
    for _ in 0..len {
        let t = reg[8] ^ reg[4];
        out.push(t);
        reg.copy_within(0..8, 1);
        reg[0] = t;
    }
    out
}

/// A 4-QAM cell as two soft bits, positive for a zero, which is the sign
/// convention the Viterbi decoder takes.
fn soft(cells: &[C32]) -> Vec<f32> {
    let mut out = Vec::with_capacity(2 * cells.len());
    for c in cells {
        out.push(c.re);
        out.push(c.im);
    }
    out
}

/// The 4-QAM point a pair of bits is sent as, clause 8.6.1.
fn point(b0: u8, b1: u8) -> C32 {
    let half = 0.5f32.sqrt();
    C32::new(if b0 == 0 { half } else { -half }, if b1 == 0 { half } else { -half })
}

fn bits_to_bytes(bits: &[u8]) -> Vec<u8> {
    bits.chunks(8).map(|c| c.iter().fold(0u8, |acc, &b| (acc << 1) | (b & 1))).collect()
}

fn field(bits: &[u8], at: usize, n: usize) -> u32 {
    bits[at..at + n].iter().fold(0u32, |acc, &b| (acc << 1) | (b & 1) as u32)
}

/// How the multiplex is carried, clause 6.3.3: which constellation and
/// whether the two halves of it are protected differently.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MscMode {
    /// Standard mapping, 64-QAM.
    Sm64,
    /// Standard mapping, 16-QAM.
    Sm16,
    /// Hierarchical, on the in-phase axis.
    Hmi64,
    /// Hierarchical, on both.
    Hmix64,
}

impl MscMode {
    fn from_bits(rm: u8, v: u32) -> MscMode {
        match (rm, v) {
            (0, 0) => MscMode::Sm64,
            (0, 1) => MscMode::Hmi64,
            (0, 2) => MscMode::Hmix64,
            _ => MscMode::Sm16,
        }
    }

    pub const fn label(self) -> &'static str {
        match self {
            MscMode::Sm64 => "64-QAM",
            MscMode::Sm16 => "16-QAM",
            MscMode::Hmi64 => "64-QAM hierarchical",
            MscMode::Hmix64 => "64-QAM hierarchical I/Q",
        }
    }
}

/// Which constellation the description channel is sent in, clause 6.3.4.
/// Only the 4-QAM form is read here; the 16-QAM one needs the multilevel
/// decoder the multiplex needs.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SdcMode {
    Qam16,
    Qam4,
}

/// The language of a service, clause 6.3.6 table 71.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Language {
    Unspecified,
    Arabic,
    Bengali,
    Chinese,
    Dutch,
    English,
    French,
    German,
    Hindi,
    Japanese,
    Javanese,
    Korean,
    Portuguese,
    Russian,
    Spanish,
    Other,
}

impl Language {
    const ALL: [Language; 16] = [
        Language::Unspecified,
        Language::Arabic,
        Language::Bengali,
        Language::Chinese,
        Language::Dutch,
        Language::English,
        Language::French,
        Language::German,
        Language::Hindi,
        Language::Japanese,
        Language::Javanese,
        Language::Korean,
        Language::Portuguese,
        Language::Russian,
        Language::Spanish,
        Language::Other,
    ];

    pub fn from_code(code: u8) -> Language {
        Language::ALL[(code & 15) as usize]
    }

    pub fn code(self) -> u8 {
        Language::ALL.iter().position(|l| *l == self).unwrap_or(0) as u8
    }

    pub const fn label(self) -> &'static str {
        match self {
            Language::Unspecified => "unspecified",
            Language::Arabic => "Arabic",
            Language::Bengali => "Bengali",
            Language::Chinese => "Chinese",
            Language::Dutch => "Dutch",
            Language::English => "English",
            Language::French => "French",
            Language::German => "German",
            Language::Hindi => "Hindi",
            Language::Japanese => "Japanese",
            Language::Javanese => "Javanese",
            Language::Korean => "Korean",
            Language::Portuguese => "Portuguese",
            Language::Russian => "Russian",
            Language::Spanish => "Spanish",
            Language::Other => "other",
        }
    }
}

/// The one service a frame's fast access channel describes. A multiplex
/// carries up to four, one frame each in turn.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Service {
    /// Which of the multiplex's services this is, 0 to 3.
    pub short_id: u8,
    /// Its 24 bit identifier, which is what it keeps across frequencies.
    pub id: u32,
    pub language: Language,
    /// Whether it carries audio rather than data.
    pub audio: bool,
    pub programme: ProgrammeType,
}

/// A frame's fast access channel, read.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Fac {
    /// Which frame of the super frame this is; the description channel is in
    /// frame zero.
    pub frame_id: u8,
    /// What the transmission occupies, or `None` for the two simulcast
    /// occupancies this does not read.
    pub occupancy: Option<Occupancy>,
    /// Whether the interleaver is the long one, two seconds deep.
    pub long_interleave: bool,
    pub msc: MscMode,
    pub sdc: SdcMode,
    pub audio_services: u8,
    pub data_services: u8,
    pub service: Service,
}

impl Fac {
    /// The 64 information bits, as the standard lays them out.
    fn from_bits(v: &[u8]) -> Fac {
        let rm = v[3];
        let service = Service {
            short_id: field(v, 44, 2) as u8,
            id: field(v, 20, 24),
            language: Language::from_code(field(v, 47, 4) as u8),
            audio: v[51] == 0,
            programme: ProgrammeType::from_code(field(v, 52, 5) as u8),
        };
        // Table 61: the pair of counts is signalled as one four bit number.
        let (audio, data) = match field(v, 11, 4) {
            0 => (4, 0),
            1 => (0, 1),
            2 => (0, 2),
            3 => (0, 3),
            4 => (1, 0),
            5 => (1, 1),
            6 => (1, 2),
            7 => (1, 3),
            8 => (2, 0),
            9 => (2, 1),
            10 => (2, 2),
            12 => (3, 0),
            13 => (3, 1),
            15 => (0, 4),
            _ => (0, 0),
        };
        Fac {
            frame_id: field(v, 1, 2) as u8,
            occupancy: Occupancy::from_bits(field(v, 4, 3) as u8),
            long_interleave: v[7] == 0,
            msc: MscMode::from_bits(rm, field(v, 8, 2)),
            sdc: if v[10] == 0 { SdcMode::Qam16 } else { SdcMode::Qam4 },
            audio_services: audio,
            data_services: data,
            service,
        }
    }

    /// The same bits again, for a transmitter.
    fn to_bits(self) -> Vec<u8> {
        let mut v = vec![0u8; FAC_BITS];
        let put = |v: &mut Vec<u8>, at: usize, n: usize, value: u32| {
            for i in 0..n {
                v[at + i] = ((value >> (n - 1 - i)) & 1) as u8;
            }
        };
        put(&mut v, 1, 2, self.frame_id as u32);
        put(&mut v, 4, 3, self.occupancy.map(|o| o.bits()).unwrap_or(3) as u32);
        v[7] = u8::from(!self.long_interleave);
        put(&mut v, 8, 2, if self.msc == MscMode::Sm16 { 3 } else { 0 });
        v[10] = u8::from(self.sdc == SdcMode::Qam4);
        let count = match (self.audio_services, self.data_services) {
            (4, 0) => 0,
            (0, 1) => 1,
            (0, 2) => 2,
            (0, 3) => 3,
            (1, 0) => 4,
            (1, 1) => 5,
            (1, 2) => 6,
            (1, 3) => 7,
            (2, 0) => 8,
            (2, 1) => 9,
            (2, 2) => 10,
            (3, 0) => 12,
            (3, 1) => 13,
            (0, 4) => 15,
            _ => 4,
        };
        put(&mut v, 11, 4, count);
        put(&mut v, 20, 24, self.service.id);
        put(&mut v, 44, 2, self.service.short_id as u32);
        put(&mut v, 47, 4, self.service.language.code() as u32);
        v[51] = u8::from(!self.service.audio);
        put(&mut v, 52, 5, self.service.programme.code() as u32);
        v
    }
}

/// Read a frame's fast access channel off its 65 cells. `None` where the
/// check fails, which is what a frame read off noise does.
pub fn fac(cells: &[C32]) -> Option<Fac> {
    let want = dsp::drm::fac_cells(Mode::B).len();
    if cells.len() != want {
        return None;
    }
    let bits = channel(&soft(cells), T0, &FAC_PUNCTURE, FAC_BITS + FAC_CRC)?;
    let bytes = bits_to_bytes(&bits);
    // The check bits go out inverted, clause 7.4.1, so a receiver compares
    // against the complement rather than looking for a zero remainder.
    if crc8(&bytes[..FAC_BITS / 8], 0x1D, 0xFF) != !bytes[FAC_BITS / 8] {
        return None;
    }
    Some(Fac::from_bits(&bits))
}

/// The cells a transmitter would send for one fast access channel.
pub fn encode_fac(f: Fac) -> Vec<C32> {
    let mut bits = f.to_bits();
    let crc = !crc8(&bits_to_bytes(&bits), 0x1D, 0xFF);
    for i in 0..8 {
        bits.push((crc >> (7 - i)) & 1);
    }
    encode(&bits, T0, &FAC_PUNCTURE)
}

/// One channel's bits, from its soft cells: put back in order, depunctured,
/// decoded and undispersed.
fn channel(soft: &[f32], t0: usize, mask: &[u8], out_bits: usize) -> Option<Vec<u8>> {
    let map = interleave(soft.len(), t0);
    let mut ordered = vec![0.0f32; soft.len()];
    for (i, &v) in soft.iter().enumerate() {
        ordered[map[i]] = v;
    }
    // Six bits of tail flush the register, and are thrown away with it.
    let mut bits =
        conv::Viterbi::decode_block(conv::DRM_1_6, &ordered, mask, out_bits + 6, conv::Ends::Zero);
    bits.truncate(out_bits);
    if bits.len() != out_bits {
        return None;
    }
    for (b, d) in bits.iter_mut().zip(dispersal(out_bits)) {
        *b ^= d;
    }
    Some(bits)
}

/// The same the other way: bits to cells.
fn encode(bits: &[u8], t0: usize, mask: &[u8]) -> Vec<C32> {
    let dispersed: Vec<u8> = bits.iter().zip(dispersal(bits.len())).map(|(b, d)| b ^ d).collect();
    let mut with_tail = dispersed;
    with_tail.extend(std::iter::repeat_n(0u8, 6));
    let coded = conv::Encoder::new(conv::DRM_1_6).punctured(&with_tail, mask);
    let map = interleave(coded.len(), t0);
    let ordered: Vec<u8> = map.iter().map(|&i| coded[i]).collect();
    ordered.chunks(2).map(|p| point(p[0], p[1])).collect()
}

/// How many bytes of description channel a mode and occupancy carry in its
/// 4-QAM form, table 56. The 16-QAM form is not read.
fn sdc_bytes(mode: Mode, occ: Occupancy) -> Option<usize> {
    let i = occ.bits() as usize;
    Some(match mode {
        Mode::A => [17, 20, 41, 47][i],
        Mode::B => [13, 15, 32, 37][i],
        Mode::C if occ == Occupancy::Full10 => 32,
        Mode::D if occ == Occupancy::Full10 => 15,
        _ => return None,
    })
}

/// One entity of the description channel, clause 6.4.3.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Entity {
    pub kind: u8,
    /// Whether it describes the configuration that is on the air now, or the
    /// one it is about to change to.
    pub next: bool,
    pub body: Vec<u8>,
}

/// A super frame's description channel, read.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct Sdc {
    /// The labels the transmitter gives its services, by short identifier.
    pub labels: Vec<(u8, String)>,
    pub entities: Vec<Entity>,
}

impl Sdc {
    /// The label of one service, where the transmitter has sent one.
    pub fn label(&self, short_id: u8) -> Option<&str> {
        self.labels.iter().find(|(id, _)| *id == short_id).map(|(_, s)| s.as_str())
    }
}

/// Read a super frame's description channel off its cells, in the 4-QAM form
/// signalled by the fast access channel. `None` where the check fails or the
/// transmission is one this does not read.
pub fn sdc(cells: &[C32], mode: Mode, occ: Occupancy) -> Option<Sdc> {
    let bytes = sdc_bytes(mode, occ)?;
    let want = dsp::drm::sdc_cells(mode, occ).len();
    if cells.len() != want {
        return None;
    }
    // What the code puts out, and what of it the standard says is the
    // channel: the rest is padding.
    let coded = cells.len() - 6;
    let length = 4 + 8 * bytes + 16;
    if length > coded {
        return None;
    }
    let bits = channel(&soft(cells), T0, &SDC_PUNCTURE, coded)?;
    // The check covers four zero bits the transmitter never sends, clause
    // 6.4.1, and the check bits themselves go out inverted.
    let mut framed = vec![0u8; 4];
    framed.extend_from_slice(&bits[..length]);
    let bytes_in = bits_to_bytes(&framed);
    let split = bytes_in.len() - 2;
    let sent = u16::from_be_bytes([bytes_in[split], bytes_in[split + 1]]);
    if crc16(&bytes_in[..split], 0x1021, 0xFFFF) != !sent {
        return None;
    }
    Some(entities(&framed, length))
}

/// The cells a transmitter would send for one description channel.
pub fn encode_sdc(entities: &[Entity], mode: Mode, occ: Occupancy) -> Option<Vec<C32>> {
    let bytes = sdc_bytes(mode, occ)?;
    let cells = dsp::drm::sdc_cells(mode, occ).len();
    let coded = cells - 6;
    let covered = 8 + 8 * bytes;
    if covered + 16 > coded + 4 {
        return None;
    }
    // Four zero bits for the check, then the AFS index, then the entities.
    let mut framed = vec![0u8; 8];
    for e in entities {
        let body = e.body.len();
        if body < 1 || body % 8 != 4 {
            return None;
        }
        let bytes = (body - 4) / 8;
        for i in 0..7 {
            framed.push(((bytes >> (6 - i)) & 1) as u8);
        }
        framed.push(u8::from(e.next));
        for i in 0..4 {
            framed.push((e.kind >> (3 - i)) & 1);
        }
        framed.extend_from_slice(&e.body);
    }
    if framed.len() > covered {
        return None;
    }
    framed.resize(covered, 0);
    let crc = !crc16(&bits_to_bytes(&framed), 0x1021, 0xFFFF);
    for i in 0..16 {
        framed.push(((crc >> (15 - i)) & 1) as u8);
    }
    // The four bits the check covers are not transmitted.
    let mut out = framed.split_off(4);
    out.resize(coded, 0);
    Some(encode(&out, T0, &SDC_PUNCTURE))
}

/// Walk the entities of a description channel.
fn entities(framed: &[u8], length: usize) -> Sdc {
    let mut out = Sdc::default();
    let mut at = 8usize;
    let end = 4 + length - 16;
    while at + 12 <= end {
        let body = field(framed, at, 7) as usize;
        let next = framed[at + 7] == 1;
        let kind = field(framed, at + 8, 4) as u8;
        at += 12;
        if kind == 0 && body == 0 {
            break;
        }
        let bits = body * 8 + 4;
        if at + bits > end {
            break;
        }
        let entity = Entity { kind, next, body: framed[at..at + bits].to_vec() };
        if kind == 1 && !next && body >= 1 {
            let short_id = field(&entity.body, 0, 2) as u8;
            let text: Vec<u8> =
                (0..body).map(|i| field(&entity.body, 4 + 8 * i, 8) as u8).collect();
            if let Ok(s) = String::from_utf8(text) {
                let s = s.trim_end_matches('\0').to_string();
                if !s.is_empty() && !out.labels.iter().any(|(id, _)| *id == short_id) {
                    out.labels.push((short_id, s));
                }
            }
        }
        out.entities.push(entity);
        at += bits;
    }
    out
}

/// A label entity, for a transmitter: the service it names and the text.
pub fn label_entity(short_id: u8, text: &str) -> Entity {
    let mut body = Vec::new();
    for i in 0..2 {
        body.push((short_id >> (1 - i)) & 1);
    }
    body.extend_from_slice(&[0, 0]);
    for byte in text.as_bytes() {
        for i in 0..8 {
            body.push((byte >> (7 - i)) & 1);
        }
    }
    Entity { kind: 1, next: false, body }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn a_service() -> Fac {
        Fac {
            frame_id: 0,
            occupancy: Some(Occupancy::Full10),
            long_interleave: true,
            msc: MscMode::Sm16,
            sdc: SdcMode::Qam4,
            audio_services: 1,
            data_services: 0,
            service: Service {
                short_id: 0,
                id: 0xA1_B2C3,
                language: Language::English,
                audio: true,
                programme: ProgrammeType::News,
            },
        }
    }

    /// The interleaver is a permutation: every place is used once.
    #[test]
    fn the_interleaver_moves_every_bit_somewhere_different() {
        for size in [130usize, 260, 644, 810] {
            let map = interleave(size, T0);
            let mut seen = vec![false; size];
            for &m in &map {
                assert!(m < size && !seen[m], "size {size} maps twice onto {m}");
                seen[m] = true;
            }
            assert_eq!(map.len(), size);
            assert_eq!(map[0], 0);
        }
    }

    /// The dispersal repeats after 511 bits, which is what a nine stage
    /// register does, and starts with the sequence the standard gives.
    #[test]
    fn the_dispersal_is_a_nine_stage_register() {
        let d = dispersal(600);
        assert_eq!(&d[..12], &[0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 1, 1]);
        assert_eq!(d[..89], d[511..600]);
    }

    /// A fast access channel through the whole chain and back: 65 cells out,
    /// every field as it went in.
    #[test]
    fn a_fast_access_channel_reads_back_what_was_sent() {
        let sent = a_service();
        let cells = encode_fac(sent);
        assert_eq!(cells.len(), 65);
        let read = fac(&cells).expect("the channel checks out");
        assert_eq!(read, sent);
        assert_eq!(read.service.id, 0xA1_B2C3);
        assert_eq!(read.service.language, Language::English);
        assert_eq!(read.service.programme, ProgrammeType::News);
        assert_eq!(read.occupancy, Some(Occupancy::Full10));
        assert_eq!(read.audio_services, 1);
    }

    /// Cells of noise: the code decodes something, as it always does, and
    /// the check throws it away.
    #[test]
    fn noise_is_not_a_fast_access_channel() {
        let mut state = 0x1234_5678u32;
        let mut read = 0;
        for _ in 0..2000 {
            let cells: Vec<C32> = (0..65)
                .map(|_| {
                    let mut next = || {
                        state = state.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
                        (state >> 16) as i16 as f32 / 32768.0
                    };
                    C32::new(next(), next())
                })
                .collect();
            read += usize::from(fac(&cells).is_some());
        }
        // An eight bit check lets one block in 256 through, so two thousand
        // blocks of noise get a handful past it and none of them is a frame.
        assert!(read <= 20, "{read} of 2000 noise blocks passed an eight bit check");
    }

    /// A description channel through the chain: the labels come back.
    #[test]
    fn a_description_channel_reads_back_its_labels() {
        for (mode, occ) in [
            (Mode::A, Occupancy::Full10),
            (Mode::B, Occupancy::Full10),
            (Mode::B, Occupancy::Full9),
            (Mode::C, Occupancy::Full10),
            (Mode::D, Occupancy::Full10),
        ] {
            // Two labels and their headers are 112 bits, which is what the
            // narrowest of these channels holds: mode D carries 15 bytes.
            let entities = vec![label_entity(0, "Shark"), label_entity(1, "Reef")];
            let cells = encode_sdc(&entities, mode, occ).expect("a channel this wide");
            assert_eq!(cells.len(), dsp::drm::sdc_cells(mode, occ).len());
            let read = sdc(&cells, mode, occ).expect("the channel checks out");
            assert_eq!(read.label(0), Some("Shark"), "mode {}", mode.label());
            assert_eq!(read.label(1), Some("Reef"));
            assert_eq!(read.entities.len(), 2);
        }
    }

    /// A label longer than the channel holds is refused rather than sent
    /// truncated.
    #[test]
    fn a_label_too_long_for_the_channel_is_refused() {
        // The narrowest mode B channel holds 13 bytes, of which a label
        // entity spends two on its header.
        let long = "x".repeat(12);
        assert!(encode_sdc(&[label_entity(0, &long)], Mode::B, Occupancy::Half45).is_none());
        assert!(encode_sdc(&[label_entity(0, "Shark SW")], Mode::B, Occupancy::Half45).is_some());
        assert!(encode_sdc(&[label_entity(0, &long)], Mode::B, Occupancy::Full10).is_some());
    }

    /// One bit of the description channel flipped: the check catches it.
    #[test]
    fn a_broken_description_channel_is_thrown_away() {
        let entities = vec![label_entity(0, "Shark SW")];
        let mut cells =
            encode_sdc(&entities, Mode::B, Occupancy::Full10).expect("a channel this wide");
        assert!(sdc(&cells, Mode::B, Occupancy::Full10).is_some());
        for c in cells.iter_mut().take(40) {
            *c = -*c;
        }
        assert_eq!(sdc(&cells, Mode::B, Occupancy::Full10), None);
    }
}
