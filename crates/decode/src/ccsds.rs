//! CCSDS telemetry frames: the sync word, the coding, and the packets.
//!
//! Every space agency's downlink is built the same way, and Meteor's LRPT is
//! one configuration of it. Going backwards from the air:
//!
//! ```text
//!   soft bits -> sync search -> Viterbi -> derandomise -> Reed-Solomon
//!             -> VCDU -> M_PDU -> space packets -> a payload
//! ```
//!
//! A channel access data unit is 1024 bytes: the attached sync marker
//! `1ACFFC1D`, then 1020 bytes of four interleaved RS(255,223) codewords,
//! whose 892 bytes of message are the virtual channel data unit. The whole
//! stream is convolutionally coded at rate one half before it is keyed, so
//! the sync word is looked for in the *coded* bits rather than after
//! decoding: one Viterbi per frame instead of one per hypothesis.
//!
//! A QPSK constellation arrives at one of four rotations, and mirrored where
//! anything in front of the demodulator swapped I and Q, so there are eight
//! ways the bits can be the right bits. [`Phase`] is that set, and the sync
//! search says which one the downlink arrived at. The frame layout, the
//! derandomiser, the four-way interleave and the practice of correlating the
//! encoded sync word follow `mlrpt` (dvdesolve/mlrpt, `src/decoder/`), which
//! reads Meteor off the air.

use crate::rs::{self, ReedSolomon};
use crate::whiten;
use dsp::conv::{self, Viterbi};

/// The attached sync marker every CCSDS frame starts with.
pub const ASM: [u8; 4] = [0x1A, 0xCF, 0xFC, 0x1D];

/// Bytes in a channel access data unit, sync marker included.
pub const CADU_BYTES: usize = 1024;

/// How many RS codewords are interleaved in one frame.
pub const INTERLEAVE: usize = 4;

/// Bytes in the coded block that follows the sync marker.
pub const CODEBLOCK_BYTES: usize = rs::CCSDS_CODEWORD * INTERLEAVE;

/// Bytes of virtual channel data unit in a frame.
pub const VCDU_BYTES: usize = rs::CCSDS_MESSAGE * INTERLEAVE;

/// Soft bits a frame occupies on the air: every byte coded at rate a half.
pub const FRAME_SOFT_BITS: usize = CADU_BYTES * 8 * 2;

/// The code the frames are convolutionally coded with, which is the
/// industry rate 1/2 K=7 with 171 octal sent first.
pub const CODE: conv::Code = conv::K7_X_FIRST;

/// Coded bits of sync word the search correlates over.
const PATTERN_BITS: usize = ASM.len() * 8 * 2;

/// How many of those 64 have to agree before the frame is taken as found.
///
/// Not all 64, because the coded sync word depends on the six data bits in
/// front of it, which belong to the previous frame: up to twelve of the
/// pattern's bits are therefore whatever the last frame ended with, and half
/// of those are wrong on average. Measured on the synthesised downlink, a
/// frame found at its true position scores 58 to 64 and the best score
/// anywhere in a frame of noise is 44.
const PATTERN_MATCH: usize = 52;

/// Bit errors allowed in the decoded sync word before a frame is thrown
/// away. The Viterbi starts each frame from an unknown state, so the first
/// byte or so of every frame is doubtful whatever the link is doing, and the
/// sync word is exactly that first byte: measured, a clean frame decodes its
/// sync word with 0 to 6 bits wrong.
const SYNC_BIT_ERRORS: u32 = 8;

/// How a QPSK constellation arrived, against how it was keyed.
///
/// Nothing in the symbols says which way up they are, so a frame decoder
/// tries all eight and keeps whichever one the sync word appears in. A
/// quarter turn takes `(i, q)` to `(-q, i)`; a mirror swaps the two axes,
/// which is what an inverted spectrum or a swapped pair of channels does.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Phase {
    Zero,
    Quarter,
    Half,
    ThreeQuarters,
    Mirror,
    MirrorQuarter,
    MirrorHalf,
    MirrorThreeQuarters,
}

impl Phase {
    pub const ALL: [Phase; 8] = [
        Phase::Zero,
        Phase::Quarter,
        Phase::Half,
        Phase::ThreeQuarters,
        Phase::Mirror,
        Phase::MirrorQuarter,
        Phase::MirrorHalf,
        Phase::MirrorThreeQuarters,
    ];

    /// Turn a symbol's pair of soft bits back the way it was keyed.
    pub fn apply(self, (i, q): (f32, f32)) -> (f32, f32) {
        match self {
            Phase::Zero => (i, q),
            Phase::Quarter => (q, -i),
            Phase::Half => (-i, -q),
            Phase::ThreeQuarters => (-q, i),
            Phase::Mirror => (q, i),
            Phase::MirrorQuarter => (i, -q),
            Phase::MirrorHalf => (-q, -i),
            Phase::MirrorThreeQuarters => (-i, q),
        }
    }
}

/// One frame off the air, decoded, derandomised and corrected.
#[derive(Clone, Debug, PartialEq)]
pub struct Frame {
    /// The virtual channel data unit: 892 bytes.
    pub vcdu: Vec<u8>,
    /// Symbols the Reed-Solomon decoder had to change, over the four
    /// codewords.
    pub corrected: usize,
    /// Which way up the constellation was.
    pub phase: Phase,
    /// Whether the frame arrived with every bit inverted, which a rate one
    /// half code with odd weight generators carries through: the data comes
    /// out inverted too and is put back.
    pub complemented: bool,
    /// Bits of the decoded sync word that were wrong, which is the only
    /// measure of the link the frame itself carries.
    pub sync_errors: u32,
}

/// Soft bits in, frames out.
///
/// A soft bit is positive for a zero and negative for a one, in whatever
/// scale the demodulator produces; two per QPSK symbol, the real axis first.
pub struct Deframer {
    soft: Vec<f32>,
    /// Where the next frame starts in `soft`, once a frame has been found.
    at: Option<usize>,
    phase: Phase,
    /// The coded sync word as hard bits, which the search correlates.
    pattern: [bool; PATTERN_BITS],
    found: u64,
    failed: u64,
}

impl Default for Deframer {
    fn default() -> Self {
        Self::new()
    }
}

impl Deframer {
    pub fn new() -> Self {
        let mut enc = conv::Encoder::new(CODE);
        let mut coded = Vec::with_capacity(PATTERN_BITS);
        for byte in ASM {
            for k in (0..8).rev() {
                enc.push(byte >> k & 1, &mut coded);
            }
        }
        let mut pattern = [false; PATTERN_BITS];
        for (p, c) in pattern.iter_mut().zip(&coded) {
            *p = *c == 1;
        }
        Self { soft: Vec::new(), at: None, phase: Phase::Zero, pattern, found: 0, failed: 0 }
    }

    /// Frames read, and frames whose sync word was found and whose
    /// Reed-Solomon could not be placed.
    pub fn found(&self) -> u64 {
        self.found
    }

    pub fn failed(&self) -> u64 {
        self.failed
    }

    /// Whether the deframer is stepping frame to frame rather than hunting.
    pub fn locked(&self) -> bool {
        self.at.is_some()
    }

    pub fn reset(&mut self) {
        self.soft.clear();
        self.at = None;
        self.phase = Phase::Zero;
    }

    /// How well the pattern matches the stream at `off`, read in `phase`.
    fn score(&self, off: usize, phase: Phase) -> usize {
        let mut same = 0;
        for k in (0..PATTERN_BITS).step_by(2) {
            let (i, q) = phase.apply((self.soft[off + k], self.soft[off + k + 1]));
            same += usize::from((i < 0.0) == self.pattern[k]);
            same += usize::from((q < 0.0) == self.pattern[k + 1]);
        }
        same
    }

    /// Hunt for a frame anywhere in `from..to`, at any of the eight ways up.
    ///
    /// Symbol boundaries only, which is every second soft bit: a frame that
    /// began on the other bit of a symbol is the same stream read mirrored,
    /// and that is one of the eight.
    fn hunt(&self, from: usize, to: usize) -> Option<(usize, Phase, usize)> {
        let mut best: Option<(usize, Phase, usize)> = None;
        let mut off = from;
        while off + PATTERN_BITS <= to {
            for phase in Phase::ALL {
                let score = self.score(off, phase);
                if score > best.map_or(PATTERN_MATCH, |b| b.2) {
                    best = Some((off, phase, score));
                }
            }
            off += 2;
        }
        best
    }

    /// Decode the frame starting at `off`, if that is what is there.
    fn read(&self, off: usize, phase: Phase) -> Option<Frame> {
        let mut soft = Vec::with_capacity(FRAME_SOFT_BITS);
        for k in (0..FRAME_SOFT_BITS).step_by(2) {
            let (i, q) = phase.apply((self.soft[off + k], self.soft[off + k + 1]));
            soft.push(i);
            soft.push(q);
        }
        let bits =
            Viterbi::decode_block(CODE, &soft, conv::P_1_2, CADU_BYTES * 8, conv::Ends::Anywhere);
        let mut bytes: Vec<u8> =
            bits.chunks(8).map(|c| c.iter().fold(0u8, |b, &bit| (b << 1) | (bit & 1))).collect();

        // A frame is transparent to inversion: every bit of a coded stream
        // flipped decodes to every data bit flipped, because both generators
        // have odd weight. So the sync word arrives either as itself or as
        // its complement, and the complement is a whole frame to invert.
        let direct = sync_errors(&bytes, false);
        let inverse = sync_errors(&bytes, true);
        let complemented = inverse < direct;
        if complemented {
            bytes.iter_mut().for_each(|b| *b = !*b);
        }
        let sync_errors = direct.min(inverse);
        if sync_errors > SYNC_BIT_ERRORS {
            return None;
        }

        let block = &mut bytes[ASM.len()..];
        whiten::ccsds(block, 0);
        let code = ReedSolomon::ccsds();
        let mut corrected = 0;
        for lane in 0..INTERLEAVE {
            let mut word = rs::deinterleave(block, lane, INTERLEAVE);
            corrected += code.decode(&mut word, &[])?;
            rs::interleave(&word, lane, INTERLEAVE, block);
        }
        Some(Frame {
            vcdu: block[..VCDU_BYTES].to_vec(),
            corrected,
            phase,
            complemented,
            sync_errors,
        })
    }

    /// Feed soft bits, appending whatever frames they completed.
    pub fn push(&mut self, soft: &[f32], out: &mut Vec<Frame>) {
        self.soft.extend_from_slice(soft);
        loop {
            // A hunt reads a frame's worth of stream looking for the sync
            // word, and then the frame itself, so two frames have to be in
            // hand before either can start.
            let need = 2 * FRAME_SOFT_BITS;
            if self.soft.len() < need {
                break;
            }
            let (off, phase) = match self.at {
                Some(at) => (at, self.phase),
                None => match self.hunt(0, FRAME_SOFT_BITS) {
                    Some((off, phase, _)) => (off, phase),
                    None => {
                        // Nothing here. Keep the tail a frame might straddle.
                        let drop = self.soft.len() - FRAME_SOFT_BITS;
                        self.soft.drain(..drop);
                        break;
                    }
                },
            };
            match self.read(off, phase) {
                Some(frame) => {
                    self.found += 1;
                    out.push(frame);
                    self.at = Some(off + FRAME_SOFT_BITS);
                    self.phase = phase;
                }
                None => {
                    self.failed += 1;
                    // A frame that would not decode where one was expected
                    // is a lock to give up: hunt again from after it.
                    self.at = None;
                    self.soft.drain(..off + FRAME_SOFT_BITS);
                    continue;
                }
            }
            let Some(at) = self.at else { continue };
            if at > 0 {
                self.soft.drain(..at);
                self.at = Some(0);
            }
        }
    }
}

/// Bits of the sync word that are wrong, reading the frame as sent or
/// inverted.
fn sync_errors(bytes: &[u8], complemented: bool) -> u32 {
    ASM.iter()
        .zip(bytes)
        .map(|(a, b)| {
            let b = match complemented {
                true => !*b,
                false => *b,
            };
            (a ^ b).count_ones()
        })
        .sum()
}

/// A virtual channel data unit: which channel of which spacecraft, and the
/// packet zone it carries.
///
/// Meteor's layout, which is the CCSDS one with a two byte insert zone: six
/// bytes of primary header, two of insert zone, two of multiplexing header
/// and 882 bytes of packets.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Vcdu<'a> {
    pub version: u8,
    pub spacecraft: u8,
    /// Virtual channel: one per instrument or service on the downlink.
    pub channel: u8,
    /// Frames sent on this virtual channel, which wraps at 2^24.
    pub counter: u32,
    /// Where the first packet header in the zone is, or `None` where a
    /// packet from the frame before runs through the whole of this one.
    pub first_header: Option<usize>,
    pub packets: &'a [u8],
}

/// Bytes of VCDU before the packet zone.
const ZONE_AT: usize = 10;

/// The first header pointer that says no packet starts in this frame.
const NO_HEADER: usize = 2047;

impl<'a> Vcdu<'a> {
    pub fn parse(vcdu: &'a [u8]) -> Option<Self> {
        if vcdu.len() < ZONE_AT + 1 {
            return None;
        }
        let id = u16::from_be_bytes([vcdu[0], vcdu[1]]);
        let counter = u32::from_be_bytes([0, vcdu[2], vcdu[3], vcdu[4]]);
        let pointer = usize::from(u16::from_be_bytes([vcdu[8], vcdu[9]]) & 0x07ff);
        let version = (id >> 14) as u8;
        let channel = (id & 0x3f) as u8;
        // A frame with neither a version nor a channel is the filler a
        // downlink sends when it has nothing to say.
        if version == 0 && channel == 0 {
            return None;
        }
        Some(Self {
            version,
            spacecraft: ((id >> 6) & 0xff) as u8,
            channel,
            counter,
            first_header: (pointer != NO_HEADER).then_some(pointer),
            packets: &vcdu[ZONE_AT..],
        })
    }
}

/// One CCSDS space packet.
#[derive(Clone, Debug, PartialEq)]
pub struct SpacePacket {
    /// Which application sent it: the instrument channel, the telemetry.
    pub apid: u16,
    /// Packets this application has sent, which wraps at 2^14.
    pub sequence: u16,
    /// Everything after the six byte primary header.
    pub payload: Vec<u8>,
}

/// Bytes of space packet primary header.
const PACKET_HEADER: usize = 6;

/// The application ids nobody sent: CCSDS reserves all ones for an idle
/// packet, and Meteor pads the tail of a packet zone with zeros, which reads
/// as a run of seven byte packets on application nought.
const IDLE_APIDS: [u16; 2] = [0, 0x07ff];

/// The most a packet may claim to be. A length field is eleven bits of
/// nonsense when the frame it came from was broken, and a run of those would
/// otherwise have the assembler waiting for a packet that never ends.
const MAX_PACKET: usize = 2048;

/// Space packets out of a run of virtual channel data units.
///
/// A packet is longer than the zone that carries it as often as not, so the
/// assembler keeps the tail of one frame to put in front of the next. A
/// frame lost in between is a packet to throw away, which the frame counter
/// says: consecutive counters mean the stream is whole.
#[derive(Clone, Debug, Default)]
pub struct Packets {
    partial: Vec<u8>,
    last_counter: Option<u32>,
    dropped: u64,
}

impl Packets {
    pub fn new() -> Self {
        Self::default()
    }

    /// Packets thrown away because the frame carrying the rest of them was
    /// lost.
    pub fn dropped(&self) -> u64 {
        self.dropped
    }

    pub fn reset(&mut self) {
        self.partial.clear();
        self.last_counter = None;
    }

    /// Read one frame's packet zone, appending every packet it completed.
    pub fn push(&mut self, vcdu: &Vcdu<'_>, out: &mut Vec<SpacePacket>) {
        let in_step = self.last_counter.is_some_and(|c| vcdu.counter == c + 1);
        self.last_counter = Some(vcdu.counter);
        if !in_step && !self.partial.is_empty() {
            self.partial.clear();
            self.dropped += 1;
        }
        let zone = vcdu.packets;
        let start = match vcdu.first_header {
            // The zone is the middle of a packet that started earlier, so
            // all of it belongs to what is held.
            None => {
                if !self.partial.is_empty() && in_step {
                    self.partial.extend_from_slice(zone);
                    self.take(out);
                }
                return;
            }
            Some(at) if at <= zone.len() => at,
            // A pointer past the end of the zone is a broken frame.
            Some(_) => return,
        };
        if !self.partial.is_empty() {
            match in_step {
                true => {
                    self.partial.extend_from_slice(&zone[..start]);
                    self.take(out);
                }
                false => self.partial.clear(),
            }
        }
        self.partial.clear();
        self.partial.extend_from_slice(&zone[start..]);
        self.take(out);
    }

    /// Read every whole packet out of what is held, leaving the rest.
    fn take(&mut self, out: &mut Vec<SpacePacket>) {
        loop {
            if self.partial.len() < PACKET_HEADER {
                return;
            }
            let head = &self.partial[..PACKET_HEADER];
            let apid = u16::from_be_bytes([head[0], head[1]]) & 0x07ff;
            let sequence = u16::from_be_bytes([head[2], head[3]]) & 0x3fff;
            // The length field is the data field's length less one.
            let length = usize::from(u16::from_be_bytes([head[4], head[5]])) + 1;
            let total = PACKET_HEADER + length;
            if total > MAX_PACKET {
                self.partial.clear();
                self.dropped += 1;
                return;
            }
            if self.partial.len() < total {
                return;
            }
            if !IDLE_APIDS.contains(&apid) {
                out.push(SpacePacket {
                    apid,
                    sequence,
                    payload: self.partial[PACKET_HEADER..total].to_vec(),
                });
            }
            self.partial.drain(..total);
        }
    }
}

/// Build one frame for the air: a VCDU, its Reed-Solomon, the randomiser,
/// the sync word and the convolutional code, as the soft bits a demodulator
/// would have produced.
///
/// Here rather than in the tests because the decoder is only as good as
/// something that builds what it reads, and both this crate's tests and the
/// node's need one.
pub fn frame_soft_bits(vcdu: &[u8]) -> Vec<f32> {
    assert_eq!(vcdu.len(), VCDU_BYTES);
    let code = ReedSolomon::ccsds();
    let mut block = vec![0u8; CODEBLOCK_BYTES];
    for lane in 0..INTERLEAVE {
        let word = rs::deinterleave(vcdu, lane, INTERLEAVE);
        let mut whole = word.clone();
        whole.extend(code.encode(&word));
        rs::interleave(&whole, lane, INTERLEAVE, &mut block);
    }
    whiten::ccsds(&mut block, 0);
    let mut cadu = ASM.to_vec();
    cadu.extend(block);
    assert_eq!(cadu.len(), CADU_BYTES);

    let mut enc = conv::Encoder::new(CODE);
    let mut coded = Vec::with_capacity(FRAME_SOFT_BITS);
    for byte in cadu {
        for k in (0..8).rev() {
            enc.push(byte >> k & 1, &mut coded);
        }
    }
    coded
        .iter()
        .map(|&b| match b {
            0 => 1.0,
            _ => -1.0,
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn frame_bits(vcdu: &[u8]) -> Vec<f32> {
        frame_soft_bits(vcdu)
    }

    fn a_vcdu(counter: u32, channel: u8, packets: &[u8]) -> Vec<u8> {
        let mut vcdu = vec![0u8; VCDU_BYTES];
        let id = (1u16 << 14) | (57 << 6) | u16::from(channel);
        vcdu[..2].copy_from_slice(&id.to_be_bytes());
        vcdu[2..5].copy_from_slice(&counter.to_be_bytes()[1..]);
        vcdu[8..10].copy_from_slice(&0u16.to_be_bytes());
        vcdu[ZONE_AT..ZONE_AT + packets.len()].copy_from_slice(packets);
        vcdu
    }

    fn a_packet(apid: u16, sequence: u16, payload: &[u8]) -> Vec<u8> {
        let mut p = Vec::new();
        p.extend(((1u16 << 11) | apid).to_be_bytes());
        p.extend((0xc000u16 | sequence).to_be_bytes());
        p.extend(((payload.len() - 1) as u16).to_be_bytes());
        p.extend_from_slice(payload);
        p
    }

    /// The randomiser against the sequence CCSDS publishes, which is the
    /// only way to know the register was built the right way round.
    #[test]
    fn the_randomiser_is_the_published_sequence() {
        let seq = whiten::ccsds_sequence();
        assert_eq!(
            &seq[..8],
            &[0xff, 0x48, 0x0e, 0xc0, 0x9a, 0x0d, 0x70, 0xbc],
            "the sequence starts elsewhere"
        );
        assert_eq!(seq[254], 0x58, "and ends elsewhere");
        // Its own inverse, which is what makes one function do both.
        let mut bytes = *b"a frame of bytes";
        let plain = bytes;
        whiten::ccsds(&mut bytes, 0);
        assert_ne!(bytes, plain);
        whiten::ccsds(&mut bytes, 0);
        assert_eq!(bytes, plain);
    }

    /// The coded sync word this searches for, against the constant `mlrpt`
    /// correlates against, which is the same 64 bits inverted: the sense of
    /// the mapping is one of the eight ways up and the search covers it
    /// either way.
    #[test]
    fn the_coded_sync_word_is_the_published_pattern() {
        let d = Deframer::new();
        let word = d.pattern.iter().fold(0u64, |w, &b| (w << 1) | u64::from(b));
        assert_eq!(!word, 0xfca2_b63d_b00d_9794);
    }

    /// One frame, read at every one of the eight ways up a constellation can
    /// arrive: the same VCDU out of all of them.
    #[test]
    fn a_frame_reads_at_every_rotation() {
        let sent = a_vcdu(7, 5, &a_packet(64, 1, &[0xaa; 40]));
        let clean = frame_bits(&sent);
        for phase in Phase::ALL {
            // Send the stream turned the way this phase undoes.
            let mut turned = Vec::with_capacity(clean.len() * 2);
            for pair in clean.chunks(2) {
                let (i, q) = inverse(phase, (pair[0], pair[1]));
                turned.push(i);
                turned.push(q);
            }
            let mut d = Deframer::new();
            let mut out = Vec::new();
            // Two frames of it, since a frame is found in the first and read
            // in the second.
            d.push(&[turned.clone(), turned.clone(), turned].concat(), &mut out);
            assert_eq!(out.len(), 2, "frames at {phase:?}");
            assert_eq!(out[0].vcdu, sent, "the wrong bytes at {phase:?}");
            assert_eq!(out[0].corrected, 0, "nothing to correct at {phase:?}");
            assert_eq!(out[0].phase, phase);
            assert!(out[0].sync_errors <= SYNC_BIT_ERRORS);
        }
    }

    /// The turn a phase undoes, for a test that has to apply one.
    fn inverse(phase: Phase, (i, q): (f32, f32)) -> (f32, f32) {
        match phase {
            Phase::Zero => (i, q),
            Phase::Quarter => (-q, i),
            Phase::Half => (-i, -q),
            Phase::ThreeQuarters => (q, -i),
            // The mirrors are their own inverses.
            _ => phase.apply((i, q)),
        }
    }

    /// What the Reed-Solomon is for: a burst of noise across a frame is
    /// corrected and the VCDU comes back exactly as it was sent, while a
    /// burst past what the code can place leaves no frame at all.
    ///
    /// The errors are counted in coded bits rather than bytes because that
    /// is what arrives: a run of sixteen wrong coded bits is a byte of
    /// stream, and the Viterbi spreads it over the byte either side.
    #[test]
    fn reed_solomon_corrects_a_burst_and_refuses_a_wider_one() {
        let sent = a_vcdu(1, 5, &a_packet(65, 3, &[0x5a; 100]));
        let clean = frame_bits(&sent);
        for (bytes, expect) in [(8usize, true), (300, false)] {
            let mut soft = clean.clone();
            let from = (ASM.len() + 20) * 8 * 2;
            for s in soft[from..from + bytes * 8 * 2].iter_mut() {
                *s = -*s;
            }
            let mut d = Deframer::new();
            let mut out = Vec::new();
            d.push(&[soft.clone(), soft].concat(), &mut out);
            match expect {
                true => {
                    assert_eq!(out.len(), 1, "a burst of {bytes} bytes");
                    assert_eq!(out[0].vcdu, sent);
                    assert!(out[0].corrected >= 8, "{} symbols corrected", out[0].corrected);
                }
                false => {
                    assert!(out.is_empty(), "a burst of {bytes} bytes read anyway");
                    assert_eq!(d.failed(), 1);
                }
            }
        }
    }

    /// Noise alone: no frame, however long it runs.
    #[test]
    fn noise_produces_no_frames() {
        let mut x = 0x2468_ace0u32;
        let noise: Vec<f32> = (0..FRAME_SOFT_BITS * 20)
            .map(|_| {
                x ^= x << 13;
                x ^= x >> 17;
                x ^= x << 5;
                (x as i32 as f32) / i32::MAX as f32
            })
            .collect();
        let mut d = Deframer::new();
        let mut out = Vec::new();
        d.push(&noise, &mut out);
        assert_eq!(out.len(), 0, "{} frames out of noise", out.len());
        assert_eq!(d.found(), 0);
    }

    /// A packet longer than the zone that carries it, assembled across three
    /// frames, and a fourth frame's packet thrown away because the frame in
    /// front of it was lost.
    #[test]
    fn packets_are_assembled_across_frames() {
        let payload: Vec<u8> = (0..1_500u16).map(|i| (i % 251) as u8).collect();
        let long = a_packet(66, 9, &payload);
        let zone = VCDU_BYTES - ZONE_AT;
        let mut packets = Packets::new();
        let mut out = Vec::new();

        let mut first = a_vcdu(100, 5, &long[..zone]);
        first[8..10].copy_from_slice(&0u16.to_be_bytes());
        let v = Vcdu::parse(&first).expect("a frame");
        packets.push(&v, &mut out);
        assert_eq!(out.len(), 0, "the packet is not finished yet");

        // The rest of it, and a second packet behind it.
        let tail = &long[zone..];
        let short = a_packet(64, 10, &[1, 2, 3, 4]);
        let mut rest = tail.to_vec();
        rest.extend_from_slice(&short);
        let mut second = a_vcdu(101, 5, &rest);
        second[8..10].copy_from_slice(&(tail.len() as u16).to_be_bytes());
        let v = Vcdu::parse(&second).expect("a frame");
        packets.push(&v, &mut out);
        assert_eq!(out.len(), 2, "the long packet and the short one");
        assert_eq!(out[0].apid, 66);
        assert_eq!(out[0].sequence, 9);
        assert_eq!(out[0].payload, payload);
        assert_eq!(out[1].apid, 64);
        assert_eq!(out[1].payload, vec![1, 2, 3, 4]);

        // A frame counter that skipped one: whatever was held is not the
        // front of this packet, so it is dropped rather than joined to it
        // and only the packet whole inside one frame comes out.
        out.clear();
        let mut third = a_vcdu(102, 5, &long[..zone]);
        third[8..10].copy_from_slice(&0u16.to_be_bytes());
        let v = Vcdu::parse(&third).expect("a frame");
        packets.push(&v, &mut out);
        let mut fourth = a_vcdu(106, 5, &rest);
        fourth[8..10].copy_from_slice(&(tail.len() as u16).to_be_bytes());
        let v = Vcdu::parse(&fourth).expect("a frame");
        packets.push(&v, &mut out);
        assert_eq!(out.len(), 1, "only the packet that was whole in one frame");
        assert_eq!(out[0].apid, 64);
        assert_eq!(packets.dropped(), 1);
    }

    /// A frame with nothing in it is the filler a downlink sends between
    /// data, and is not a frame to read packets out of.
    #[test]
    fn a_filler_frame_is_not_read() {
        assert_eq!(Vcdu::parse(&vec![0u8; VCDU_BYTES]), None);
        let v = a_vcdu(4, 5, &[]);
        let parsed = Vcdu::parse(&v).expect("a frame");
        assert_eq!(parsed.version, 1);
        assert_eq!(parsed.spacecraft, 57);
        assert_eq!(parsed.channel, 5);
        assert_eq!(parsed.counter, 4);
        assert_eq!(parsed.packets.len(), VCDU_BYTES - ZONE_AT);
    }
}
