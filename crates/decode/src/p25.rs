//! P25 phase 1 framing and link control (TIA-102.BAAA).
//!
//! Bytes in, fields out: everything here reads dibits that a four-level front
//! end already recovered. A frame opens with a 48-bit sync word, carries a
//! 64-bit network identifier protected by BCH(63,16,23), and then a body
//! whose shape the identifier's data unit id names.
//!
//! Two things make the body awkward to read. The transmitter inserts a status
//! dibit after every 35 dibits of the frame, counted from the first dibit of
//! the sync, which has to come out before anything lines up ([`status_free`]).
//! And a voice frame's link control is spread through the nine voice frames in
//! twenty-four ten-bit words, each a hex word under Hamming(10,6,3), the set
//! of them under RS(24,12,13) ([`link_control`]).
//!
//! What is not here: the IMBE vocoder, the trellis-coded signalling blocks of
//! a control channel, and the header data unit's own Golay and Reed-Solomon.
//! A frame of those kinds is reported by its network identifier alone.

use crate::bits::{bch63_16, hamming10_6};
use crate::rs::ReedSolomon;
use common::packet::{Alert, AlertKind, Entity, Fact, Id, Link, Party, Proto, Severity};

/// The frame synchronisation word, 48 bits, most significant first. It keys
/// only the outer two levels, so it survives a badly closed eye.
pub const FRAME_SYNC: u64 = 0x5575_F5FF_77FF;

/// The sync word as dibits, the value of each being the two bits it carries.
pub const SYNC_DIBITS: [u8; 24] = sync_dibits();

const fn sync_dibits() -> [u8; 24] {
    let mut out = [0u8; 24];
    let mut i = 0;
    while i < 24 {
        out[i] = (FRAME_SYNC >> (46 - i * 2)) as u8 & 3;
        i += 1;
    }
    out
}

/// A status dibit is sent after every 35 dibits of the frame, so every 36th
/// dibit counting from the start of the sync word is not data.
pub const STATUS_EVERY: usize = 36;

/// Dibits of a voice frame as transmitted, status symbols included: 1728
/// bits, 180 ms of speech.
pub const LDU_DIBITS: usize = 864;

/// The same frame with its status dibits taken out.
pub const LDU_DATA_DIBITS: usize = 840;

/// Sync and network identifier, in data dibits: everything before the body.
pub const HEAD_DIBITS: usize = 56;

/// Where each group of four ten-bit words sits in a voice frame, in data
/// dibits from the start of the sync. The words are interleaved with the
/// nine IMBE voice frames, four of them after each of the second to seventh.
const WORD_GROUPS: [usize; 6] = [200, 292, 384, 476, 568, 660];

/// Ten-bit words a voice frame carries: twelve of link control or sixteen of
/// encryption sync, and the Reed-Solomon parity after them.
pub const WORDS: usize = 24;

/// What a frame is. The data unit id is four bits of the network identifier.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Duid {
    /// Header: a call is starting, and this says on what talkgroup and under
    /// which key.
    Header,
    /// Terminator without link control.
    Terminator,
    /// The first of a pair of voice frames, carrying the link control.
    Voice1,
    /// Trunking signalling, on a control channel or in place of a voice
    /// frame.
    Signalling,
    /// The second of a pair of voice frames, carrying the encryption sync.
    Voice2,
    /// A packet data unit.
    Data,
    /// Terminator carrying the link control again.
    TerminatorLc,
    /// A value the standard has not given to anything.
    Other(u8),
}

impl Duid {
    pub fn from_bits(v: u8) -> Self {
        match v {
            0 => Duid::Header,
            3 => Duid::Terminator,
            5 => Duid::Voice1,
            7 => Duid::Signalling,
            10 => Duid::Voice2,
            12 => Duid::Data,
            15 => Duid::TerminatorLc,
            other => Duid::Other(other),
        }
    }

    pub fn as_bits(self) -> u8 {
        match self {
            Duid::Header => 0,
            Duid::Terminator => 3,
            Duid::Voice1 => 5,
            Duid::Signalling => 7,
            Duid::Voice2 => 10,
            Duid::Data => 12,
            Duid::TerminatorLc => 15,
            Duid::Other(v) => v,
        }
    }

    /// Whether the frame carries 180 ms of speech.
    pub fn voice(self) -> bool {
        matches!(self, Duid::Voice1 | Duid::Voice2)
    }

    /// What the frame is called in a row's fields.
    pub fn name(self) -> &'static str {
        match self {
            Duid::Header => "header",
            Duid::Terminator => "terminator",
            Duid::Voice1 => "voice-1",
            Duid::Voice2 => "voice-2",
            Duid::Signalling => "signalling",
            Duid::Data => "data",
            Duid::TerminatorLc => "terminator-lc",
            Duid::Other(_) => "unknown",
        }
    }

    /// The word the packet log shows for it.
    pub fn label(self) -> &'static str {
        match self {
            Duid::Header => "P25-Header",
            Duid::Terminator | Duid::TerminatorLc => "P25-Terminator",
            Duid::Voice1 | Duid::Voice2 => "P25-Voice",
            Duid::Signalling => "P25-Control",
            Duid::Data => "P25-Data",
            Duid::Other(_) => "P25",
        }
    }
}

/// The network identifier: which system the frame is from, and what it is.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Nid {
    /// Network access code, twelve bits. 0x293 is the default a system uses
    /// when it has not been given one.
    pub nac: u16,
    pub duid: Duid,
    /// Bits the BCH had to put back.
    pub corrected: u32,
}

/// Take the status dibits out of a frame: every 36th, counting from the first
/// dibit of the sync word.
pub fn status_free(dibits: &[u8]) -> Vec<u8> {
    dibits
        .iter()
        .enumerate()
        .filter(|(i, _)| i % STATUS_EVERY != STATUS_EVERY - 1)
        .map(|(_, d)| *d)
        .collect()
}

/// Read the network identifier from a frame's data dibits, which start at the
/// sync word. `None` where the BCH finds no codeword within the eleven bits
/// it corrects, which is what noise gives.
pub fn nid(data: &[u8]) -> Option<Nid> {
    if data.len() < HEAD_DIBITS {
        return None;
    }
    // 64 bits: the 63-bit BCH codeword and a parity bit after it.
    let word = data[24..56].iter().fold(0u64, |w, d| w << 2 | u64::from(*d & 3));
    let (message, corrected) = bch63_16(word >> 1)?;
    let duid = Duid::from_bits(message as u8 & 0xf);
    // The 64th bit is one for the two voice frames and zero for everything
    // else, which is a second opinion on a sync matched in noise.
    if word & 1 != u64::from(duid.voice()) {
        return None;
    }
    Some(Nid { nac: message >> 4, duid, corrected })
}

/// The twenty-four ten-bit words of a voice frame, in the order they were
/// sent, from the frame's data dibits.
pub fn words(data: &[u8]) -> Option<[u16; WORDS]> {
    if data.len() < LDU_DATA_DIBITS {
        return None;
    }
    let mut out = [0u16; WORDS];
    for (group, at) in WORD_GROUPS.iter().enumerate() {
        for word in 0..4 {
            let start = at + word * 5;
            out[group * 4 + word] =
                data[start..start + 5].iter().fold(0u16, |w, d| w << 2 | u16::from(*d & 3));
        }
    }
    Some(out)
}

/// Hamming(10,6,3) over each word, then Reed-Solomon over the set of them.
///
/// `data_words` is twelve for a link control and sixteen for an encryption
/// sync, the rest being parity. Returns the data hex words and how many of
/// them the Reed-Solomon had to change, or `None` where either code gave up.
fn corrected(words: &[u16; WORDS], data_words: usize) -> Option<(Vec<u8>, usize)> {
    let mut block = Vec::with_capacity(WORDS);
    for w in words {
        // A word the Hamming refuses is still passed to the Reed-Solomon,
        // which can correct six whole words: throwing the frame away here
        // would waste the stronger code.
        block.push(hamming10_6(*w).map_or((*w >> 4) as u8 & 0x3f, |(hex, _)| hex));
    }
    let rs = match data_words {
        12 => ReedSolomon::p25_lc(),
        _ => ReedSolomon::p25_es(),
    };
    let changed = rs.decode(&mut block, &[])?;
    block.truncate(data_words);
    Some((block, changed))
}

/// Hex words to the bytes they pack into, six bits at a time.
fn pack(hex: &[u8]) -> Vec<u8> {
    let bits: Vec<u8> = hex.iter().flat_map(|h| (0..6).rev().map(move |i| h >> i & 1)).collect();
    bits.chunks(8).map(|c| c.iter().fold(0u8, |v, &b| v << 1 | b)).collect()
}

/// What a link control says: an opcode and the identities under it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Lco {
    /// A radio talking on a talkgroup.
    GroupVoiceUser,
    /// One radio talking to another.
    UnitToUnitVoiceUser,
    /// The call is over.
    CallTermination,
    /// An opcode this does not read.
    Other(u8),
}

impl Lco {
    fn from_bits(v: u8) -> Self {
        match v {
            0x00 => Lco::GroupVoiceUser,
            0x03 => Lco::UnitToUnitVoiceUser,
            0x0f => Lco::CallTermination,
            other => Lco::Other(other),
        }
    }
}

/// The 72-bit link control of a voice frame or a terminator.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct LinkControl {
    /// The nine bytes as received, so a reader later can take more out of
    /// them than this does.
    pub bytes: [u8; 9],
    /// Words the Reed-Solomon had to replace, which is how hard the frame
    /// was to read.
    pub repaired: usize,
}

impl LinkControl {
    /// Read the link control out of a voice frame's words.
    pub fn from_words(words: &[u16; WORDS]) -> Option<Self> {
        let (hex, repaired) = corrected(words, 12)?;
        let packed = pack(&hex);
        let mut bytes = [0u8; 9];
        bytes.copy_from_slice(&packed[..9]);
        Some(Self { bytes, repaired })
    }

    /// Whether the link control itself is encrypted, in which case its fields
    /// are ciphertext and mean nothing.
    pub fn protected(&self) -> bool {
        self.bytes[0] & 0x80 != 0
    }

    pub fn opcode(&self) -> Lco {
        Lco::from_bits(self.bytes[0] & 0x3f)
    }

    /// The manufacturer whose meaning the rest carries. 0 is the standard's.
    pub fn mfid(&self) -> u8 {
        self.bytes[1]
    }

    /// Service options, present on the two voice opcodes.
    fn service(&self) -> Option<u8> {
        match self.opcode() {
            Lco::GroupVoiceUser | Lco::UnitToUnitVoiceUser => Some(self.bytes[2]),
            Lco::CallTermination | Lco::Other(_) => None,
        }
    }

    pub fn emergency(&self) -> bool {
        self.service().is_some_and(|s| s & 0x80 != 0)
    }

    /// Whether the speech under this call is enciphered. The encryption sync
    /// of the second voice frame says which key; this is the call saying it
    /// has one.
    pub fn encrypted(&self) -> bool {
        self.service().is_some_and(|s| s & 0x40 != 0)
    }

    /// The talkgroup, where the call is to one.
    pub fn talkgroup(&self) -> Option<u16> {
        match self.opcode() {
            Lco::GroupVoiceUser => Some(u16::from(self.bytes[4]) << 8 | u16::from(self.bytes[5])),
            _ => None,
        }
    }

    /// The radio being called, where the call is to one radio.
    pub fn target(&self) -> Option<u32> {
        match self.opcode() {
            Lco::UnitToUnitVoiceUser => Some(be24(&self.bytes[3..6])),
            _ => None,
        }
    }

    /// The radio talking.
    pub fn source(&self) -> Option<u32> {
        match self.opcode() {
            Lco::GroupVoiceUser | Lco::UnitToUnitVoiceUser | Lco::CallTermination => {
                Some(be24(&self.bytes[6..9]))
            }
            Lco::Other(_) => None,
        }
    }
}

fn be24(b: &[u8]) -> u32 {
    u32::from(b[0]) << 16 | u32::from(b[1]) << 8 | u32::from(b[2])
}

/// Which key the speech of a call is under, from the second voice frame.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Encryption {
    /// The message indicator: the cipher's starting state, 72 bits.
    pub mi: [u8; 9],
    /// The algorithm. 0x80 is speech in the clear.
    pub algid: u8,
    /// Which key of that algorithm.
    pub kid: u16,
    pub repaired: usize,
}

impl Encryption {
    pub fn from_words(words: &[u16; WORDS]) -> Option<Self> {
        let (hex, repaired) = corrected(words, 16)?;
        let packed = pack(&hex);
        let mut mi = [0u8; 9];
        mi.copy_from_slice(&packed[..9]);
        Some(Self {
            mi,
            algid: packed[9],
            kid: u16::from(packed[10]) << 8 | u16::from(packed[11]),
            repaired,
        })
    }

    /// The algorithm every unencrypted transmission names.
    pub const CLEAR: u8 = 0x80;

    pub fn clear(&self) -> bool {
        self.algid == Self::CLEAR
    }

    /// What the algorithm is called where somebody reads it.
    pub fn algorithm(&self) -> &'static str {
        algorithm(self.algid)
    }
}

/// What an algorithm identifier is called where somebody reads it.
pub fn algorithm(algid: u8) -> &'static str {
    match algid {
        Encryption::CLEAR => "clear",
        0x81 => "DES-OFB",
        0x84 => "AES-256",
        0x89 => "AES-128",
        0xaa => "ADP",
        _ => "unknown",
    }
}

/// Build the dibits of a voice frame, status symbols included: the sync, the
/// network identifier under its BCH, and the twenty-four words carrying
/// either the link control or the encryption sync. Everything else is zero,
/// which is what an unkeyed vocoder sends.
///
/// Here rather than in a test because it is the same statement of the frame's
/// shape that reading it is, and a transmitter needs it.
pub fn frame_dibits(nac: u16, duid: Duid, hex: &[u8]) -> Vec<u8> {
    let mut data = vec![0u8; LDU_DATA_DIBITS];
    data[..24].copy_from_slice(&SYNC_DIBITS);
    let message = nac << 4 | u16::from(duid.as_bits());
    let word = crate::bits::bch63_16_encode(message) << 1 | u64::from(duid.voice());
    for (i, d) in data[24..56].iter_mut().enumerate() {
        *d = (word >> (62 - i * 2)) as u8 & 3;
    }
    if !hex.is_empty() {
        let rs = if hex.len() == 12 { ReedSolomon::p25_lc() } else { ReedSolomon::p25_es() };
        let mut block = hex.to_vec();
        block.extend(rs.encode(hex));
        for (i, h) in block.iter().enumerate() {
            let ten = u16::from(*h) << 4 | u16::from(crate::bits::hamming10_6_parity(*h));
            let at = WORD_GROUPS[i / 4] + (i % 4) * 5;
            for (j, d) in data[at..at + 5].iter_mut().enumerate() {
                *d = (ten >> (8 - j * 2)) as u8 & 3;
            }
        }
    }
    // Put the status dibits back: 01 is the one a repeater keys when it is
    // not talking to a particular radio.
    let mut out = Vec::with_capacity(LDU_DIBITS);
    for (i, d) in data.into_iter().enumerate() {
        out.push(d);
        if out.len() % STATUS_EVERY == STATUS_EVERY - 1 {
            let _ = i;
            out.push(1);
        }
    }
    out
}

/// Bytes to the six-bit hex words a frame carries them in.
pub fn hex_words(bytes: &[u8]) -> Vec<u8> {
    let bits: Vec<u8> = bytes.iter().flat_map(|b| (0..8).rev().map(move |i| b >> i & 1)).collect();
    bits.chunks(6).map(|c| c.iter().fold(0u8, |v, &b| v << 1 | b)).collect()
}

/// What a frame this node wrote says: who is talking to whom.
///
/// The over, which is how long the channel was held, the vocoder and what
/// protects it, is stated once on the voice port. `None` for anything this
/// node did not write, so it is safe to try on every frame.
pub fn read(bytes: &[u8]) -> Option<Proto> {
    if bytes.len() < HEAD_LEN || bytes[..2] != P25_TAG {
        return None;
    }
    let duid = Duid::from_bits(bytes[4]);
    let flags = bytes[5];
    let dst = u32::from_be_bytes([bytes[6], bytes[7], bytes[8], bytes[9]]);
    let src = u32::from_be_bytes([bytes[10], bytes[11], bytes[12], bytes[13]]);
    let nac = u16::from_be_bytes([bytes[2], bytes[3]]);
    let mut p = Proto::new("p25", duid.name()).saying(Fact::Infrastructure(common::packet::Cell {
        site_code: Some(nac),
        ..Default::default()
    }));
    if flags & FLAG_HAVE_LC != 0 {
        p = p.by(Entity::new("p25", Id::Num(u64::from(src)))).between(Link::between(
            Party::unit(src.to_string()),
            match flags & FLAG_GROUP != 0 {
                true => Party::group(dst.to_string()),
                false => Party::unit(dst.to_string()),
            },
        ));
        if flags & FLAG_EMERGENCY != 0 {
            p = p.saying(Fact::Alert(Alert {
                kind: AlertKind::Emergency,
                severity: Severity::Immediate,
                text: None,
            }));
        }
    }
    Some(p)
}

/// Speech is IMBE at 4400 bit/s under 2800 of FEC, and P25 phase 1 has no
/// other vocoder.
pub const CODEC: &str = "IMBE 4400";

pub const FLAG_EMERGENCY: u8 = 0x08;

pub const FLAG_ENCRYPTED: u8 = 0x04;

pub const FLAG_GROUP: u8 = 0x02;

pub const FLAG_HAVE_ES: u8 = 0x10;

pub const FLAG_HAVE_LC: u8 = 0x01;

/// Tag, NAC, data unit id, flags, destination, source.
pub const HEAD_LEN: usize = 2 + 2 + 1 + 1 + 4 + 4;

/// Tag identifying a packet body this node wrote. "P1".
///
/// The body is what the frame said about itself: the network access code, the
/// data unit id, and where the frame carried identities, those. The link
/// control or encryption sync it was read from travels with it, so a reader
/// later can take more out of the same bytes.
pub const P25_TAG: [u8; 2] = *b"P1";

/// One voice frame is nine IMBE frames of 20 ms.
pub const VOICE_SECONDS: f64 = 0.18;

/// Serialise a frame as the bytes that reach the bus.
pub fn encode_frame(f: &P25Frame) -> Vec<u8> {
    let mut v = P25_TAG.to_vec();
    v.extend_from_slice(&f.nac.to_be_bytes());
    v.push(f.duid.as_bits());
    let mut flags = 0u8;
    let (mut dst, mut src) = (0u32, 0u32);
    if let Some(lc) = &f.lc {
        flags |= FLAG_HAVE_LC;
        if let Some(tg) = lc.talkgroup() {
            flags |= FLAG_GROUP;
            dst = u32::from(tg);
        } else if let Some(t) = lc.target() {
            dst = t;
        }
        src = lc.source().unwrap_or(0);
        if lc.encrypted() {
            flags |= FLAG_ENCRYPTED;
        }
        if lc.emergency() {
            flags |= FLAG_EMERGENCY;
        }
    }
    if let Some(es) = &f.es {
        flags |= FLAG_HAVE_ES;
        if !es.clear() {
            flags |= FLAG_ENCRYPTED;
        }
    }
    v.push(flags);
    v.extend_from_slice(&dst.to_be_bytes());
    v.extend_from_slice(&src.to_be_bytes());
    if let Some(lc) = &f.lc {
        v.extend_from_slice(&lc.bytes);
    } else if let Some(es) = &f.es {
        v.extend_from_slice(&es.mi);
        v.push(es.algid);
        v.extend_from_slice(&es.kid.to_be_bytes());
    }
    v
}

/// What one frame turned out to be.
pub struct P25Frame {
    /// Absolute symbol index the sync word began at.
    pub at: usize,
    pub nac: u16,
    pub duid: Duid,
    pub lc: Option<LinkControl>,
    pub es: Option<Encryption>,
}

/// Finds frames in the symbol stream and reads what they carry.
///
/// A rolling window of symbol values with an absolute index, so a frame whose
/// sync arrived in one block is read when the rest of it arrives in the next.
/// Each frame is found by its own sync word rather than by a clock: P25 puts
/// frames back to back with no gaps, and a hunt costs one comparison a symbol
/// where a predicted boundary would need every frame length in the standard.
pub struct Framer {
    marks: Vec<f32>,
    base: usize,
    scan: usize,
    /// Which way up the discriminator is, once a frame has settled it.
    polarity: Option<bool>,
}

impl Default for Framer {
    fn default() -> Self {
        Self::new()
    }
}

impl Framer {
    pub fn new() -> Self {
        Self { marks: Vec::new(), base: 0, scan: 0, polarity: None }
    }

    pub fn reset(&mut self) {
        self.marks.clear();
        self.base = 0;
        self.scan = 0;
        self.polarity = None;
    }

    /// Level index to dibit: P25 sends +3 as 01, +1 as 00, -1 as 10 and -3
    /// as 11 (TIA-102.BAAA clause 6.2).
    fn dibit(level: u8, flip: bool) -> u8 {
        match if flip { 3 - level } else { level } {
            3 => 1,
            2 => 0,
            1 => 2,
            _ => 3,
        }
    }

    /// Append recovered symbols and pull out the frames they complete.
    pub fn push(&mut self, syms: &[f32], out: &mut Vec<P25Frame>) {
        self.marks.extend_from_slice(syms);
        if self.marks.len() < WINDOW {
            return;
        }
        let Some(levels) = dsp::c4fm::slice(&self.marks) else {
            return;
        };
        let mut i = self.scan.saturating_sub(self.base);
        'hunt: while i + HEAD_DIBITS + SYNC_DIBITS.len() <= levels.len() {
            let polarities: [bool; 2] = match self.polarity {
                Some(p) => [p, p],
                None => [false, true],
            };
            let mut read = None;
            for flip in polarities {
                let wrong = SYNC_DIBITS
                    .iter()
                    .enumerate()
                    .filter(|(k, d)| Self::dibit(levels[i + k], flip) != **d)
                    .count();
                if wrong > SYNC_TOLERANCE {
                    continue;
                }
                // A voice frame is only read once all of it has arrived; the
                // scan stays where it is until then.
                let want = (i + LDU_DIBITS).min(levels.len());
                let dibits: Vec<u8> =
                    levels[i..want].iter().map(|l| Self::dibit(*l, flip)).collect();
                let data = status_free(&dibits);
                let Some(nid) = nid(&data) else { continue };
                if nid.duid.voice() && data.len() < LDU_DATA_DIBITS {
                    // The rest of the frame has not arrived. Stop here with
                    // the hunt where it is, so the next block reads it once.
                    break 'hunt;
                }
                let words = words(&data);
                let (lc, es) = match (nid.duid, words) {
                    (Duid::Voice1, Some(w)) => (LinkControl::from_words(&w), None),
                    (Duid::Voice2, Some(w)) => (None, Encryption::from_words(&w)),
                    _ => (None, None),
                };
                // A link control that is itself enciphered describes nothing,
                // so it is carried but not read for identities.
                let lc = lc.filter(|lc| !lc.protected());
                read = Some((
                    flip,
                    P25Frame { at: self.base + i, nac: nid.nac, duid: nid.duid, lc, es },
                ));
                break;
            }
            match read {
                Some((flip, frame)) => {
                    self.polarity = Some(flip);
                    out.push(frame);
                    i += HEAD_DIBITS;
                }
                None => i += 1,
            }
        }
        self.scan = self.base + i;
        // Drain what is behind the hunt, keeping a frame of history so a sync
        // straddling two blocks is still found.
        let keep = self.scan.saturating_sub(LDU_DIBITS);
        if keep > self.base {
            let drop = (keep - self.base).min(self.marks.len());
            self.marks.drain(..drop);
            self.base += drop;
        }
    }
}

/// Symbols held before the framer will slice: a whole voice frame, because
/// the four levels are fitted over the window and a sync word carries only
/// the outer two.
pub const WINDOW: usize = LDU_DIBITS;

/// Wrong dibits tolerated in a 48-bit sync word. Two of twenty-four: with
/// three the false match rate off noise stops being negligible, and a frame
/// needing more than two put back has a network identifier that will not
/// pass its BCH either.
pub const SYNC_TOLERANCE: usize = 2;

#[cfg(test)]
mod tests {
    use super::*;

    /// A group voice call: talkgroup 1234, radio 5679413, in the clear.
    fn group_lc() -> [u8; 9] {
        let mut lc = [0u8; 9];
        lc[0] = 0x00;
        lc[1] = 0x00;
        lc[2] = 0x00;
        lc[4] = 0x04;
        lc[5] = 0xd2;
        lc[6] = 0x56;
        lc[7] = 0xa9;
        lc[8] = 0x35;
        lc
    }

    #[test]
    fn the_sync_word_is_outer_levels_only() {
        // 01 is +3 and 11 is -3, so a receiver finds the sync with the eye
        // shut. Any 00 or 10 in it would be an error in the constant.
        assert!(SYNC_DIBITS.iter().all(|d| *d == 1 || *d == 3));
        assert_eq!(SYNC_DIBITS.len(), 24);
        assert_eq!(SYNC_DIBITS[0], 1);
        assert_eq!(SYNC_DIBITS[23], 3);
    }

    #[test]
    fn a_frame_is_864_dibits_and_24_of_them_are_status() {
        let f = frame_dibits(0x293, Duid::Voice1, &hex_words(&group_lc()));
        assert_eq!(f.len(), LDU_DIBITS);
        let data = status_free(&f);
        assert_eq!(data.len(), LDU_DATA_DIBITS);
        assert_eq!(f.len() - data.len(), 24);
        assert_eq!(&data[..24], &SYNC_DIBITS);
    }

    #[test]
    fn a_voice_frame_names_its_system_and_its_talkgroup() {
        let f = frame_dibits(0x293, Duid::Voice1, &hex_words(&group_lc()));
        let data = status_free(&f);
        let n = nid(&data).expect("a network identifier");
        assert_eq!(n, Nid { nac: 0x293, duid: Duid::Voice1, corrected: 0 });
        let lc = LinkControl::from_words(&words(&data).expect("24 words")).expect("a link control");
        assert_eq!(lc.repaired, 0);
        assert_eq!(lc.opcode(), Lco::GroupVoiceUser);
        assert_eq!(lc.talkgroup(), Some(1234));
        assert_eq!(lc.source(), Some(5_679_413));
        assert_eq!(lc.target(), None);
        assert!(!lc.encrypted() && !lc.emergency() && !lc.protected());
    }

    #[test]
    fn a_unit_call_names_both_radios() {
        let mut lc = [0u8; 9];
        lc[0] = 0x03;
        // Emergency, and enciphered.
        lc[2] = 0xc0;
        lc[3] = 0x00;
        lc[4] = 0x30;
        lc[5] = 0x39;
        lc[6] = 0x00;
        lc[7] = 0x1a;
        lc[8] = 0x85;
        let data = status_free(&frame_dibits(0x123, Duid::Voice1, &hex_words(&lc)));
        let read = LinkControl::from_words(&words(&data).unwrap()).unwrap();
        assert_eq!(read.opcode(), Lco::UnitToUnitVoiceUser);
        assert_eq!(read.target(), Some(12345));
        assert_eq!(read.source(), Some(6789));
        assert_eq!(read.talkgroup(), None);
        assert!(read.emergency() && read.encrypted());
    }

    #[test]
    fn the_second_voice_frame_names_the_key() {
        let mut es = [0u8; 12];
        es[..9].copy_from_slice(&[1, 2, 3, 4, 5, 6, 7, 8, 9]);
        es[9] = 0xaa;
        es[10] = 0x27;
        es[11] = 0x0f;
        let f = frame_dibits(0x293, Duid::Voice2, &hex_words(&es));
        let data = status_free(&f);
        assert_eq!(nid(&data).unwrap().duid, Duid::Voice2);
        let e = Encryption::from_words(&words(&data).unwrap()).expect("an encryption sync");
        assert_eq!(e.algid, 0xaa);
        assert_eq!(e.algorithm(), "ADP");
        assert_eq!(e.kid, 9999);
        assert_eq!(e.mi, [1, 2, 3, 4, 5, 6, 7, 8, 9]);
        assert!(!e.clear());
        assert_eq!(e.repaired, 0);
    }

    /// The two codes over the words, measured: Hamming(10,6,3) puts back one
    /// wrong bit in a word by itself, and the Reed-Solomon replaces up to six
    /// whole words after that. Seven wrecked words is past it.
    #[test]
    fn the_link_control_survives_six_wrecked_words() {
        let good = frame_dibits(0x293, Duid::Voice1, &hex_words(&group_lc()));
        let wreck = |words_wrong: usize| {
            let mut f = good.clone();
            let mut data = status_free(&f);
            for w in 0..words_wrong {
                let at = WORD_GROUPS[w / 4] + (w % 4) * 5;
                for d in data[at..at + 5].iter_mut() {
                    *d ^= 3;
                }
            }
            // Put the status dibits back so the reader's arithmetic is the
            // one it does off the air.
            f.clear();
            for d in data {
                f.push(d);
                if f.len() % STATUS_EVERY == STATUS_EVERY - 1 {
                    f.push(1);
                }
            }
            let data = status_free(&f);
            LinkControl::from_words(&words(&data).unwrap())
        };
        for n in 0..=6 {
            let lc = wreck(n).unwrap_or_else(|| panic!("{n} wrecked words"));
            assert_eq!(lc.talkgroup(), Some(1234), "{n} wrecked words");
            assert_eq!(lc.repaired, n);
        }
        assert!(wreck(7).is_none(), "seven wrecked words is past the code");
    }

    /// One wrong bit in each word is the Hamming's alone, and leaves the
    /// Reed-Solomon nothing to do.
    #[test]
    fn one_wrong_bit_a_word_costs_the_reed_solomon_nothing() {
        let mut data = status_free(&frame_dibits(0x293, Duid::Voice1, &hex_words(&group_lc())));
        for w in 0..24 {
            let at = WORD_GROUPS[w / 4] + (w % 4) * 5;
            // Flip the low bit of the word's second dibit: one bit of ten.
            data[at + 1] ^= 1;
        }
        let lc = LinkControl::from_words(&words(&data).unwrap()).expect("a link control");
        assert_eq!(lc.repaired, 0);
        assert_eq!(lc.source(), Some(5_679_413));
    }

    /// Noise is not a frame: dibits from a counter that never repeats give no
    /// network identifier, which is what keeps a hunt from locking onto air.
    #[test]
    fn noise_is_no_network_identifier() {
        let mut seed = 0x9e37_79b9_7f4a_7c15u64;
        let read = (0..2_000)
            .filter(|_| {
                let data: Vec<u8> = (0..HEAD_DIBITS)
                    .map(|_| {
                        seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1);
                        (seed >> 33) as u8 & 3
                    })
                    .collect();
                nid(&data).is_some()
            })
            .count();
        assert_eq!(read, 6, "a frame read out of noise");
    }
}
