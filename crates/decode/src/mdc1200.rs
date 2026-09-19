//! MDC-1200, the data burst a Motorola radio sends when the key goes down.
//!
//! Bits in, a unit id and an operation out. The waveform above it is fast
//! FSK at 1200 baud ([`dsp::afsk::FFSK1200`]), and what arrives here is the
//! tone decisions: true for the 1800 Hz tone, false for the 1200 Hz one.
//!
//! # The line code
//!
//! The transmitter sends 1800 Hz when the data bit changes and 1200 Hz when
//! it does not, so the data is recovered by differencing: `d[i] = d[i-1] ^
//! t[i]`. That leaves the polarity of the whole stream undecided, which is
//! why the sync hunt looks for the sync word and for its complement and
//! flips everything when it is the complement that matched. The 0x00 or 0x55
//! leader comes out as a steady tone either way, which is what the receiver
//! locks its clock to.
//!
//! # The block
//!
//! After the 40-bit sync word `07 09 2A 44 6F`, 112 bits are interleaved
//! across a 16 by 7 grid: on-air bit `j*16 + i` is information bit `i*7 + j`.
//! De-interleaved and packed least significant bit first, the first seven
//! bytes are the operation, its argument, the unit id, the CRC of those four
//! and a status byte; the last seven are the parity of a rate 1/2 code with
//! taps {0, 2, 5, 6}, which this reads as a second check rather than using
//! to correct.
//!
//! Checked against Matthew Kaufman's `mdc-encode-decode`, whose encoder
//! produced the on-air blocks the tests carry.

use crate::bits;
use common::Decoded;

/// The sync word every MDC decoder looks for, most significant bit first.
pub const SYNC: u64 = 0x07_09_2A_44_6F;
const SYNC_BITS: u32 = 40;

/// How many of the 40 sync bits may be wrong and the burst still be taken as
/// MDC. The reference decoder allows five, and so does this.
const SYNC_SLACK: u32 = 5;

/// Information bits in a block, and the bytes they make.
const BLOCK_BITS: usize = 112;
const BLOCK_BYTES: usize = BLOCK_BITS / 8;

/// What the radio is saying, from the operation and argument pair.
///
/// The published pairs only: an unrecognised pair is [`Operation::Other`]
/// and keeps its bytes, because every fleet has vendor codes and a burst
/// whose CRC passed is still a radio identifying itself.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Operation {
    /// The unit id at the start of a transmission, which is most of the
    /// traffic on any channel that has this switched on.
    PttIdPre,
    /// The same at the end of it, so a receiving radio can close the call.
    PttIdPost,
    Emergency,
    RequestToTalk,
    RemoteMonitor,
    /// Page a particular radio. The id is the target's, not the sender's.
    CallAlert,
    /// Unmute a particular radio for the voice that follows. Also a target.
    SelectiveCall,
    Other {
        op: u8,
        arg: u8,
    },
}

impl Operation {
    pub fn of(op: u8, arg: u8) -> Self {
        match (op, arg) {
            (0x01, 0x80) => Operation::PttIdPre,
            (0x00, 0x80) => Operation::PttIdPost,
            (0x40, 0x80) => Operation::Emergency,
            (0x35, 0x89) => Operation::RequestToTalk,
            (0x11, 0x80) => Operation::RemoteMonitor,
            (0x63, 0x85) => Operation::CallAlert,
            (0x35, 0x80) => Operation::SelectiveCall,
            (op, arg) => Operation::Other { op, arg },
        }
    }

    /// What a person reads.
    pub fn label(&self) -> String {
        match self {
            Operation::PttIdPre => "PTT-ID".into(),
            Operation::PttIdPost => "PTT-ID end".into(),
            Operation::Emergency => "emergency".into(),
            Operation::RequestToTalk => "request to talk".into(),
            Operation::RemoteMonitor => "remote monitor".into(),
            Operation::CallAlert => "call alert".into(),
            Operation::SelectiveCall => "selective call".into(),
            Operation::Other { op, arg } => format!("op {op:02x} arg {arg:02x}"),
        }
    }

    /// Whether the id in the burst is the radio being called rather than the
    /// one transmitting. A call alert names whom it is paging.
    pub fn addresses_target(&self) -> bool {
        matches!(self, Operation::CallAlert | Operation::SelectiveCall)
    }
}

/// One burst: who, and what they are saying.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Message {
    pub op: u8,
    pub arg: u8,
    pub unit: u16,
    pub status: u8,
    pub operation: Operation,
}

impl Message {
    /// The unit id as a radio's display and a programming sheet write it.
    pub fn unit_hex(&self) -> String {
        format!("{:04X}", self.unit)
    }
}

/// The seven information bytes as they go on the bus: operation, argument,
/// unit id, CRC and status. Self-checking, so anything reading them back can
/// say whether the bytes are MDC without being told.
pub const INFO_BYTES: usize = 7;

/// The CRC MDC puts over the first four information bytes.
///
/// CRC-CCITT with the reflected polynomial, a zero start and a final
/// inversion. Vector: `01 80 12 34` gives 0x3E2E, which is on the wire low
/// byte first.
pub fn crc(info: &[u8]) -> u16 {
    bits::crc16le(info, 0x8408, 0x0000) ^ 0xFFFF
}

/// Read the seven information bytes, where the CRC says they are a burst.
pub fn parse(info: &[u8]) -> Option<Message> {
    if info.len() < INFO_BYTES {
        return None;
    }
    let want = u16::from(info[4]) | u16::from(info[5]) << 8;
    if crc(&info[..4]) != want {
        return None;
    }
    let (op, arg) = (info[0], info[1]);
    Some(Message {
        op,
        arg,
        unit: u16::from(info[2]) << 8 | u16::from(info[3]),
        status: info[6],
        operation: Operation::of(op, arg),
    })
}

/// De-interleave one 112-bit block, most significant bit first on the air,
/// into its fourteen information bytes.
pub fn deinterleave(air: &[bool]) -> Option<[u8; BLOCK_BYTES]> {
    if air.len() < BLOCK_BITS {
        return None;
    }
    let mut out = [0u8; BLOCK_BYTES];
    for i in 0..16 {
        for j in 0..7 {
            if air[j * 16 + i] {
                let k = i * 7 + j;
                out[k / 8] |= 1 << (k % 8);
            }
        }
    }
    Some(out)
}

/// The parity of the rate 1/2 code over the seven information bytes: taps
/// {0, 2, 5, 6} of an eight-bit register that runs across the whole block
/// rather than restarting per byte.
///
/// Used as a check and not as a correction: a burst whose CRC failed is
/// thrown away here, where the reference decoder can repair three or four
/// bits first. That is the obvious next thing to add, and it wants a
/// recording to be worth measuring.
pub fn parity(info: &[u8; INFO_BYTES]) -> [u8; INFO_BYTES] {
    let mut sr = 0u8;
    let mut out = [0u8; INFO_BYTES];
    for (i, &byte) in info.iter().enumerate() {
        for bit in 0..8 {
            sr = (sr << 1) | (byte >> bit) & 1;
            let p = (sr >> 6) ^ (sr >> 5) ^ (sr >> 2) ^ sr;
            out[i] |= (p & 1) << bit;
        }
    }
    out
}

/// Tone decisions in, bursts out.
///
/// True is the 1800 Hz tone. The framer differences the stream, hunts the
/// sync word in either polarity and collects the block behind it.
pub struct Framer {
    /// The previous data bit, which is what the differencing is against.
    prev: bool,
    /// Whether the polarity was found flipped, so every bit is complemented.
    inverted: bool,
    window: u64,
    filled: u32,
    block: Vec<bool>,
    hunting: bool,
    /// Bursts whose block was collected and whose CRC then failed.
    refused: u64,
}

impl Default for Framer {
    fn default() -> Self {
        Self::new()
    }
}

impl Framer {
    pub fn new() -> Self {
        Self {
            prev: false,
            inverted: false,
            window: 0,
            filled: 0,
            block: Vec::with_capacity(BLOCK_BITS),
            hunting: true,
            refused: 0,
        }
    }

    pub fn reset(&mut self) {
        let refused = self.refused;
        *self = Self::new();
        self.refused = refused;
    }

    /// Blocks that reached the CRC and failed it, since the framer was
    /// built: a channel with a burst on it that never reads is a different
    /// fault from a channel with nothing on it.
    pub fn refused(&self) -> u64 {
        self.refused
    }

    /// One tone decision. `Some` on the bit that completes a block whose
    /// CRC passed, carrying its seven information bytes.
    pub fn push(&mut self, space: bool) -> Option<[u8; INFO_BYTES]> {
        // The tone says whether the data bit changed, so the data is the
        // running difference of the tones.
        let data = self.prev ^ space;
        self.prev = data;
        let bit = data ^ self.inverted;

        if self.hunting {
            self.window = (self.window << 1 | u64::from(bit)) & ((1 << SYNC_BITS) - 1);
            self.filled = (self.filled + 1).min(SYNC_BITS);
            if self.filled < SYNC_BITS {
                return None;
            }
            let wrong = (self.window ^ SYNC).count_ones();
            if wrong <= SYNC_SLACK {
                self.hunting = false;
                self.block.clear();
            } else if wrong >= SYNC_BITS - SYNC_SLACK {
                // The sync word arrived complemented, so the differencing
                // started on the wrong phase: flip from here on.
                self.inverted = !self.inverted;
                self.hunting = false;
                self.block.clear();
            }
            return None;
        }

        self.block.push(bit);
        if self.block.len() < BLOCK_BITS {
            return None;
        }
        self.hunting = true;
        self.filled = 0;
        self.window = 0;
        let block = deinterleave(&self.block)?;
        let info: [u8; INFO_BYTES] = block[..INFO_BYTES].try_into().ok()?;
        match parse(&info) {
            Some(_) => Some(info),
            None => {
                self.refused += 1;
                None
            }
        }
    }
}

/// The on-air block a message becomes: the fourteen interleaved bytes, most
/// significant bit first. For a transmitter, and for the tests.
pub fn encode_block(op: u8, arg: u8, unit: u16, status: u8) -> [u8; BLOCK_BYTES] {
    let mut info = [0u8; INFO_BYTES];
    info[0] = op;
    info[1] = arg;
    info[2] = (unit >> 8) as u8;
    info[3] = unit as u8;
    let c = crc(&info[..4]);
    info[4] = c as u8;
    info[5] = (c >> 8) as u8;
    info[6] = status;

    let mut full = [0u8; BLOCK_BYTES];
    full[..INFO_BYTES].copy_from_slice(&info);
    full[INFO_BYTES..].copy_from_slice(&parity(&info));

    let mut air = [0u8; BLOCK_BYTES];
    for i in 0..16 {
        for j in 0..7 {
            let k = i * 7 + j;
            if full[k / 8] >> (k % 8) & 1 == 1 {
                let at = j * 16 + i;
                air[at / 8] |= 1 << (7 - at % 8);
            }
        }
    }
    air
}

/// The tones a burst is sent as: leader, sync, block, differenced. True is
/// the mark tone, which is what [`dsp::afsk::modulate`] takes.
pub fn encode_tones(op: u8, arg: u8, unit: u16, status: u8, leader_bytes: usize) -> Vec<bool> {
    let mut data: Vec<bool> = vec![false; leader_bytes * 8];
    for i in 0..SYNC_BITS {
        data.push(SYNC >> (SYNC_BITS - 1 - i) & 1 == 1);
    }
    let air = encode_block(op, arg, unit, status);
    for i in 0..BLOCK_BITS {
        data.push(air[i / 8] >> (7 - i % 8) & 1 == 1);
    }
    // The post-preamble a transmitter sends after the block: four bytes of
    // zeros, which is a steady tone. Without it the last information bit is
    // the last sample and a receiver's bit clock never reaches it.
    data.resize(data.len() + 4 * 8, false);
    let mut prev = false;
    data.iter()
        .map(|&d| {
            let changed = d != prev;
            prev = d;
            // The tone is the mark where nothing changed, which is why a
            // leader of zeros is a steady tone.
            !changed
        })
        .collect()
}

/// One row: which radio, and what it was saying.
pub fn decoded(bytes: &[u8], center: common::Hz) -> Option<Decoded> {
    let m = parse(bytes)?;
    let unit = m.unit_hex();
    let role = match m.operation.addresses_target() {
        true => "target",
        false => "unit",
    };
    let fields = vec![
        (role.to_string(), common::Value::Text(unit.clone())),
        ("operation".into(), common::Value::Text(m.operation.label())),
        ("op".into(), common::Value::Int(i64::from(m.op))),
        ("arg".into(), common::Value::Int(i64::from(m.arg))),
        ("status".into(), common::Value::Int(i64::from(m.status))),
    ];
    Some(
        Decoded::bytes("MDC-1200", center, 0.0, bytes.to_vec())
            // The same namespace `ident` publishes a DTMF PTT-ID under: one
            // fleet's unit numbers, sent two ways.
            .by(common::Identity::new("radio-unit", unit.clone()))
            .with_detail(format!("{unit} {}", m.operation.label()))
            .with_fields(fields)
            .with_modulation(common::Modulation::Msk)
            // The burst carried a CRC over its four information bytes and
            // this row exists because it passed.
            .with_crc(Some(true)),
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Matthew Kaufman's `mdc-encode-decode`, asked for op 0x01, arg 0x80,
    /// unit 0x1234, puts these fourteen bytes on the air after the sync
    /// word. Compiled and run to get them, so this is a second
    /// implementation's answer and not this one's.
    const REFERENCE: [u8; 14] =
        [0x8e, 0x94, 0x2a, 0x2e, 0x06, 0x99, 0x22, 0x04, 0x00, 0x1c, 0x18, 0xa2, 0x2c, 0x88];

    /// The same library asked for op 0x63, arg 0x00, unit 0xABCD.
    const REFERENCE_2: [u8; 14] =
        [0x96, 0x93, 0x80, 0xc9, 0x3e, 0xb6, 0x3c, 0xe6, 0x00, 0x2c, 0xb4, 0x56, 0x90, 0x86];

    fn air_bits(block: &[u8; 14]) -> Vec<bool> {
        (0..BLOCK_BITS).map(|i| block[i / 8] >> (7 - i % 8) & 1 == 1).collect()
    }

    /// The published CRC vector, which every implementation of this agrees
    /// on: `01 80 12 34` is 0x3E2E, and it goes on the wire low byte first.
    #[test]
    fn the_crc_matches_the_published_vector() {
        assert_eq!(crc(&[0x01, 0x80, 0x12, 0x34]), 0x3E2E);
        assert_eq!(crc(&[0xAB, 0xCD, 0x00, 0x00]), crc(&[0xAB, 0xCD, 0x00, 0x00]));
    }

    /// The reference encoder's block, de-interleaved, is the radio that sent
    /// it: op, argument, unit id, CRC and the parity bytes.
    #[test]
    fn the_reference_block_reads_as_the_unit_that_sent_it() {
        let full = deinterleave(&air_bits(&REFERENCE)).expect("112 bits");
        assert_eq!(&full[..7], &[0x01, 0x80, 0x12, 0x34, 0x2E, 0x3E, 0x00]);
        // The published FEC vector for those seven bytes.
        assert_eq!(&full[7..], &[0x65, 0x80, 0xA8, 0x62, 0xDD, 0x88, 0x08]);
        let info: [u8; 7] = full[..7].try_into().unwrap();
        assert_eq!(parity(&info), [0x65, 0x80, 0xA8, 0x62, 0xDD, 0x88, 0x08]);
        let m = parse(&full[..7]).expect("a burst");
        assert_eq!(m.unit, 0x1234);
        assert_eq!(m.unit_hex(), "1234");
        assert_eq!(m.operation, Operation::PttIdPre);
        assert_eq!(m.status, 0x00);

        let second = deinterleave(&air_bits(&REFERENCE_2)).expect("112 bits");
        let m = parse(&second[..7]).expect("a burst");
        assert_eq!(m.unit, 0xABCD);
        assert_eq!(m.op, 0x63);
        assert_eq!(m.arg, 0x00);
        assert_eq!(m.operation, Operation::Other { op: 0x63, arg: 0x00 });
    }

    /// And this side builds the same block the reference did, bit for bit,
    /// which is the interleave, the CRC and the parity all agreeing with a
    /// second implementation.
    #[test]
    fn the_block_this_builds_is_the_block_the_reference_built() {
        assert_eq!(encode_block(0x01, 0x80, 0x1234, 0x00), REFERENCE);
        assert_eq!(encode_block(0x63, 0x00, 0xABCD, 0x00), REFERENCE_2);
    }

    /// Tones in, the burst out, whichever phase the differencing started on.
    ///
    /// The receiver has no idea what the data bit before the first tone it
    /// heard was, and an odd number of transitions in the noise ahead of the
    /// burst complements everything that follows. One click before the
    /// leader is that case, and the sync hunt has to see through it.
    #[test]
    fn a_burst_is_framed_on_either_phase() {
        for click in [false, true] {
            let mut f = Framer::new();
            let mut read = Vec::new();
            if click {
                read.extend(f.push(true));
            }
            for _ in 0..40 {
                read.extend(f.push(false));
            }
            for t in encode_tones(0x40, 0x80, 0x0042, 0x00, 3) {
                read.extend(f.push(!t));
            }
            assert_eq!(read.len(), 1, "a click before it: {click}");
            let m = parse(&read[0]).expect("a burst");
            assert_eq!(m.unit, 0x0042);
            assert_eq!(m.operation, Operation::Emergency);
            assert_eq!(f.refused(), 0);
        }
    }

    /// Two bursts in a row, which is what a radio sends when the key goes
    /// down and again when it is released.
    #[test]
    fn a_pre_and_a_post_burst_are_two_messages() {
        let mut f = Framer::new();
        let mut read = Vec::new();
        for (op, arg) in [(0x01u8, 0x80u8), (0x00, 0x80)] {
            for t in encode_tones(op, arg, 0x1234, 0x00, 3) {
                read.extend(f.push(!t));
            }
        }
        assert_eq!(read.len(), 2);
        assert_eq!(parse(&read[0]).unwrap().operation, Operation::PttIdPre);
        assert_eq!(parse(&read[1]).unwrap().operation, Operation::PttIdPost);
        assert_eq!(parse(&read[1]).unwrap().unit, 0x1234);
    }

    /// A block with bits knocked out of it is thrown away rather than
    /// reported with a wrong unit id, and it is counted.
    #[test]
    fn a_corrupted_block_is_refused() {
        let mut tones = encode_tones(0x01, 0x80, 0x1234, 0x00, 3);
        let at = tones.len() - 60;
        for t in &mut tones[at..at + 12] {
            *t = !*t;
        }
        let mut f = Framer::new();
        let mut read = Vec::new();
        for t in tones {
            read.extend(f.push(!t));
        }
        assert_eq!(read.len(), 0);
        assert_eq!(f.refused(), 1);
    }

    /// Random tones are not bursts. The sync word allows five wrong bits, so
    /// this is the test that says five is not so slack that noise frames.
    #[test]
    fn noise_frames_nothing() {
        let mut seed = 0x243f_6a88_85a3_08d3u64;
        let mut rng = || {
            seed ^= seed << 13;
            seed ^= seed >> 7;
            seed ^= seed << 17;
            seed & 1 == 1
        };
        let mut f = Framer::new();
        let mut read = Vec::new();
        // Ten minutes of tone decisions at 1200 baud.
        for _ in 0..1200 * 600 {
            read.extend(f.push(rng()));
        }
        assert_eq!(read.len(), 0, "noise made {} bursts", read.len());
    }
}
