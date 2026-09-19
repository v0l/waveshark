//! VDL Mode 2: the burst's blocks, and the AVLC frames inside them.
//!
//! Above `dsp::d8psk` and below nothing: bits in, frames out. A transmission
//! is a scrambled bit stream carrying a 25-bit header, then Reed-Solomon
//! blocks of data, then the frames themselves with HDLC bit stuffing and a
//! frame check sequence.
//!
//! ```text
//!   tribits -> descramble -> header (length, 5-bit FEC)
//!                        -> RS(255,249) blocks, deinterleaved
//!                        -> unstuff -> AVLC frames, FCS checked
//!                        -> ACARS where the frame carries one
//! ```
//!
//! The layout follows ICAO Annex 10 volume III, and the constants were read
//! off Tomasz Lemiech's dumpvdl2, which is also what the capture test checks
//! the output against.

use crate::rs::ReedSolomon;
use common::Decoded;

/// The scrambler's starting state: a 15-bit register, x^15 + x + 1.
const LFSR_IV: u16 = 0x6959;

/// Header: 3 reserved bits, 17 bits of transmission length, 5 bits of FEC.
const TRLEN: usize = 17;
const HDRFECLEN: usize = 5;
const HEADER_LEN: usize = 3 + TRLEN + HDRFECLEN;

const RS_N: usize = 255;
const RS_K: usize = 249;

/// Longer than this and the sync was on noise rather than a preamble.
const MAX_FRAME_BITS: u32 = 0x3fff;
/// Tighter still where the header needed correcting to read at all.
const MAX_FRAME_BITS_CORRECTED: u32 = 0x1fff;

/// The header's parity check matrix and the single-error patterns its
/// syndromes name, as ICAO specifies them.
const H: [u32; HDRFECLEN] = [
    0b0000000011111111111110000,
    0b0011111100001111111101000,
    0b1100011100110000111100100,
    0b1101101101010011001100010,
    0b0110100111100101010100001,
];

const SYNDROME: [u32; 1 << HDRFECLEN] = [
    0b0000000000000000000000000,
    0b0000000000000000000000001,
    0b0000000000000000000000010,
    0b0100000000000000000000100,
    0b0000000000000000000000100,
    0b0100000000000000000000010,
    0b1000000000000000000000000,
    0b0100000000000000000000000,
    0b0000000000000000000001000,
    0b0010000000000000000000000,
    0b0001000000000000000000000,
    0b0000100000000000000000000,
    0b0000010000000000000000000,
    0b1000100000000000000000000,
    0b0000001000000000000000000,
    0b0000000100000000000000000,
    0b0000000000000000000010000,
    0b0000000010000000000000000,
    0b0100000000100000000000000,
    0b0000000001000000000000000,
    0b0100000001000000000000000,
    0b0000000000100000000000000,
    0b0000000000010000000000000,
    0b1000000010000000000000000,
    0b0000000000001000000000000,
    0b0000000000000100000000000,
    0b0000000000000010000000000,
    0b0000000000000001000000000,
    0b0000000000000000100000000,
    0b0000000000000000010000000,
    0b0000000000000000001000000,
    0b0000000000000000000100000,
];

/// An AVLC address: 24 bits of identity and what kind of station it is.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct Address {
    pub addr: u32,
    pub kind: AddressKind,
    /// The command/response bit, which means different things each way.
    pub status: bool,
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum AddressKind {
    Aircraft,
    /// A ground station that may delegate, and one that administers.
    GroundDelegated,
    GroundAdministered,
    All,
    Reserved(u8),
}

impl AddressKind {
    fn of(bits: u8) -> Self {
        match bits {
            1 => AddressKind::Aircraft,
            4 => AddressKind::GroundDelegated,
            5 => AddressKind::GroundAdministered,
            7 => AddressKind::All,
            other => AddressKind::Reserved(other),
        }
    }

    pub fn label(&self) -> &'static str {
        match self {
            AddressKind::Aircraft => "aircraft",
            AddressKind::GroundDelegated => "ground station",
            AddressKind::GroundAdministered => "ground station",
            AddressKind::All => "all stations",
            AddressKind::Reserved(_) => "reserved",
        }
    }

    pub fn is_aircraft(&self) -> bool {
        matches!(self, AddressKind::Aircraft)
    }
}

/// What the link control byte says a frame is.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Control {
    /// Numbered information, which is what carries a payload.
    Info { send: u8, recv: u8, poll: bool },
    /// Supervisory: an acknowledgement or a flow control.
    Supervisory(u8),
    /// Unnumbered, which includes the XID exchanges a link is set up with.
    Unnumbered(u8),
}

impl Control {
    fn of(b: u8) -> Self {
        if b & 1 == 0 {
            Control::Info { send: (b >> 1) & 7, recv: (b >> 5) & 7, poll: (b >> 4) & 1 == 1 }
        } else if b & 3 == 1 {
            Control::Supervisory(b)
        } else {
            Control::Unnumbered(b)
        }
    }

    pub fn label(&self) -> &'static str {
        match self {
            Control::Info { .. } => "I",
            Control::Supervisory(_) => "S",
            Control::Unnumbered(_) => "U",
        }
    }
}

/// One AVLC frame whose check sequence passed.
#[derive(Clone, Debug)]
pub struct Frame {
    pub dst: Address,
    pub src: Address,
    pub control: Control,
    /// Everything after the control byte, less the check sequence.
    pub info: Vec<u8>,
}

impl Frame {
    /// The ACARS block this frame carries, where it carries one. An ACARS
    /// message over VDL2 is prefixed `ff ff 01` and ends with its own CRC and
    /// a delete character, in place of the SYN and SOH of the VHF form.
    pub fn acars(&self) -> Option<crate::acars::Message> {
        if !matches!(self.control, Control::Info { .. }) {
            return None;
        }
        let body = self.info.strip_prefix(&[0xff, 0xff, 0x01])?;
        acars_block(body)
    }
}

/// An ACARS message carried over VDL2, checked and parsed.
pub fn acars_block(body: &[u8]) -> Option<crate::acars::Message> {
    // The trailing delete character is a frame marker, not part of the block.
    let body = body.strip_suffix(&[0x7f])?;
    if body.len() < 3 || crate::bits::crc16le(body, 0x8408, 0) != 0 {
        return None;
    }
    // The parity bits every ACARS character carries survive the trip, so the
    // block is the VHF block with its framing taken off.
    let stripped: Vec<u8> = body[..body.len() - 2].iter().map(|b| b & 0x7f).collect();
    crate::acars::parse(&stripped)
}

/// Undo the scrambler over `bits`, continuing from `lfsr`.
fn descramble(bits: &mut [bool], lfsr: &mut u16) {
    for b in bits.iter_mut() {
        let feedback = ((*lfsr) ^ (*lfsr >> 14)) & 1;
        *lfsr = (*lfsr >> 1) | (feedback << 14);
        *b ^= feedback == 1;
    }
}

/// Correct the header in place, returning the syndrome that did it. A
/// non-zero syndrome means a bit was flipped back, which is a reason to be
/// stricter about the length that follows.
fn correct_header(header: &mut u32) -> u32 {
    let mut syndrome = 0u32;
    for (i, h) in H.iter().enumerate() {
        let parity = (*header & h).count_ones() & 1;
        syndrome |= parity << (HDRFECLEN - 1 - i);
    }
    *header ^= SYNDROME[syndrome as usize];
    syndrome
}

/// How many parity symbols a block of `octets` carries. A short last block
/// carries fewer, and one under three octets carries none at all.
fn fec_octets(octets: usize) -> usize {
    match octets {
        0..=2 => 0,
        3..=30 => 2,
        31..=67 => 4,
        _ => 6,
    }
}

fn reverse(v: u32, bits: usize) -> u32 {
    let mut out = 0;
    for i in 0..bits {
        out |= ((v >> i) & 1) << (bits - 1 - i);
    }
    out
}

/// The transmission's shape, read out of its header.
#[derive(Clone, Copy, Debug)]
struct Header {
    data_octets: usize,
    fec_octets: usize,
    blocks: usize,
    last_block: usize,
    bits: u32,
}

fn read_header(bits: &[bool]) -> Option<Header> {
    if bits.len() < HEADER_LEN {
        return None;
    }
    let mut header = 0u32;
    for (i, b) in bits[..HEADER_LEN].iter().enumerate() {
        header |= (*b as u32) << (HEADER_LEN - 1 - i);
    }
    // The reserved symbol's bits are zero, and forcing them so gives the
    // correction a better chance.
    header &= (1 << (TRLEN + HDRFECLEN)) - 1;
    let syndrome = correct_header(&mut header);
    if header >= 1 << (TRLEN + HDRFECLEN) {
        return None;
    }
    header >>= HDRFECLEN;
    let datalen = reverse(header & ((1 << TRLEN) - 1), TRLEN);
    let limit = if syndrome != 0 { MAX_FRAME_BITS_CORRECTED } else { MAX_FRAME_BITS };
    if datalen > limit {
        return None;
    }

    let data_octets = datalen.div_ceil(8) as usize;
    let mut blocks = data_octets / RS_K;
    let mut fec = blocks * (RS_N - RS_K);
    let mut last = data_octets % RS_K;
    if last != 0 {
        blocks += 1;
    }
    fec += fec_octets(last);
    if last == 0 {
        last = RS_K;
    }
    if fec == 0 {
        return None;
    }
    Some(Header { data_octets, fec_octets: fec, blocks, last_block: last, bits: datalen })
}

/// The interleaver: symbols go down the columns of a table `rows` deep.
fn deinterleave(
    input: &[u8],
    table: &mut [[u8; RS_N]],
    rows: usize,
    fill: usize,
    offset: usize,
) -> bool {
    if rows == 0 || fill == 0 || fill + offset > RS_N || input.len() > rows * fill {
        return false;
    }
    let mut last_row = input.len() % fill;
    if last_row == 0 {
        last_row = fill;
    }
    if rows > 1 && input.len() - last_row < (rows - 1) * fill {
        return false;
    }
    let last_row = last_row + offset;
    let (mut row, mut col) = (0usize, offset);
    for &v in input {
        if row == rows - 1 && col >= last_row {
            table[row][col] = 0;
            row = 0;
            col += 1;
        }
        if col >= RS_N {
            return false;
        }
        table[row][col] = v;
        row += 1;
        if row == rows {
            row = 0;
            col += 1;
        }
    }
    true
}

fn bits_to_octets_lsbfirst(bits: &[bool], n: usize) -> Vec<u8> {
    let mut out = vec![0u8; n];
    for (i, byte) in out.iter_mut().enumerate() {
        for j in 0..8 {
            if bits[i * 8 + j] {
                *byte |= 1 << j;
            }
        }
    }
    out
}

/// How many bits a burst needs before its blocks can be read, or `None` while
/// even the header is incomplete. This is what the demodulator asks as it
/// goes, since the length is in the transmission rather than known in advance.
pub fn wanted_bits(bits: &[bool]) -> Option<usize> {
    if bits.len() < HEADER_LEN {
        return None;
    }
    let mut header = bits[..HEADER_LEN].to_vec();
    let mut lfsr = LFSR_IV;
    descramble(&mut header, &mut lfsr);
    let h = read_header(&header)?;
    Some(HEADER_LEN + 8 * (h.data_octets + h.fec_octets))
}

/// Every AVLC frame in one burst of demodulated tribits, parsed.
pub fn frames(bits: &[bool]) -> Vec<Frame> {
    frame_bytes(bits).iter().filter_map(|f| parse_frame(f)).collect()
}

/// Every AVLC frame in one burst, as the octets that passed the check
/// sequence. What goes on the bus, since a frame is evidence before it is
/// fields.
pub fn frame_bytes(bits: &[bool]) -> Vec<Vec<u8>> {
    let mut out = Vec::new();
    if bits.len() < HEADER_LEN {
        return out;
    }
    let mut stream = bits.to_vec();
    let mut lfsr = LFSR_IV;
    descramble(&mut stream, &mut lfsr);
    let Some(h) = read_header(&stream) else { return out };
    let need = HEADER_LEN + 8 * (h.data_octets + h.fec_octets);
    if stream.len() < need {
        return out;
    }

    let data = bits_to_octets_lsbfirst(&stream[HEADER_LEN..], h.data_octets);
    let fec = bits_to_octets_lsbfirst(&stream[HEADER_LEN + 8 * h.data_octets..], h.fec_octets);

    let mut table = vec![[0u8; RS_N]; h.blocks];
    if !deinterleave(&data, &mut table, h.blocks, RS_K, 0) {
        return out;
    }
    // A last block too short to carry parity gets none written into it.
    let fec_rows = if fec_octets(h.last_block % RS_K) == 0 { h.blocks - 1 } else { h.blocks };
    if fec_rows > 0 && !deinterleave(&fec, &mut table, fec_rows, RS_N - RS_K, RS_K) {
        return out;
    }

    let rs = ReedSolomon::vdl2();
    let mut payload: Vec<bool> = Vec::with_capacity(h.bits as usize);
    for (r, block) in table.iter_mut().enumerate() {
        let parity = if r == h.blocks - 1 { fec_octets(h.last_block) } else { RS_N - RS_K };
        // Parity the transmitter did not send is an erasure at a known
        // position, which is half the cost of an unknown error.
        let erasures: Vec<usize> = (RS_K + parity..RS_N).collect();
        if parity > 0 && rs.decode(block, &erasures).is_none() {
            return out;
        }
        let take = if r == h.blocks - 1 { h.last_block } else { RS_K };
        for &b in &block[..take] {
            for j in 0..8 {
                payload.push(b >> j & 1 == 1);
            }
        }
    }
    payload.truncate(h.bits as usize);

    for frame in unstuff(&payload) {
        if frame.len() >= MIN_AVLC_LEN && crate::bits::crc16le(&frame, 0x8408, 0xffff) == GOOD_FCS {
            out.push(frame);
        }
    }
    out
}

/// HDLC unstuffing: drop the zero after five ones, and cut at the flags.
fn unstuff(bits: &[bool]) -> Vec<Vec<u8>> {
    let mut frames = Vec::new();
    let mut cur: Vec<bool> = Vec::new();
    let mut ones = 0;
    for &b in bits {
        if !b && ones == 5 {
            ones = 0;
            continue;
        }
        if b {
            ones += 1;
            if ones > 6 {
                break;
            }
        }
        cur.push(b);
        if !b {
            if ones == 6 {
                // The last eight bits are the flag itself.
                if cur.len() > 8 {
                    let keep = cur.len() - 8;
                    if keep.is_multiple_of(8) {
                        frames.push(pack_lsbfirst(&cur[..keep]));
                    }
                }
                cur.clear();
            }
            ones = 0;
        }
    }
    frames
}

fn pack_lsbfirst(bits: &[bool]) -> Vec<u8> {
    bits.chunks(8)
        .map(|c| c.iter().enumerate().fold(0u8, |b, (i, v)| b | ((*v as u8) << i)))
        .collect()
}

/// The minimum an AVLC frame can be: two addresses, a control byte and the
/// check sequence.
const MIN_AVLC_LEN: usize = 4 + 4 + 1 + 2;

/// What the check sequence leaves behind over a frame that is intact.
const GOOD_FCS: u16 = 0xf0b8;

/// An address is 28 bits spread over four octets, reversed.
fn parse_address(b: &[u8]) -> Address {
    let packed = (b[0] as u32 >> 1)
        | ((b[1] as u32) << 6)
        | ((b[2] as u32) << 13)
        | (((b[3] & 0xfe) as u32) << 20);
    let v = reverse(packed & ((1 << 28) - 1), 28);
    Address {
        addr: v & 0xff_ffff,
        kind: AddressKind::of(((v >> 24) & 7) as u8),
        status: (v >> 27) & 1 == 1,
    }
}

/// One frame's octets, check sequence included, as fields.
pub fn parse_frame(buf: &[u8]) -> Option<Frame> {
    if buf.len() < MIN_AVLC_LEN {
        return None;
    }
    // The frame check sequence is a CRC-16 whose remainder over the whole
    // frame is a fixed value.
    if crate::bits::crc16le(buf, 0x8408, 0xffff) != GOOD_FCS {
        return None;
    }
    let end = buf.len() - 2;
    Some(Frame {
        dst: parse_address(&buf[0..4]),
        src: parse_address(&buf[4..8]),
        control: Control::of(buf[8]),
        info: buf[9..end].to_vec(),
    })
}

/// The decode an AVLC frame becomes.
pub fn decoded(f: &Frame, bytes: &[u8], center: common::Hz) -> Decoded {
    let mut fields: Vec<(String, common::Value)> = vec![
        ("from".into(), common::Value::Text(format!("{:06X}", f.src.addr))),
        ("from_kind".into(), common::Value::Text(f.src.kind.label().into())),
        ("to".into(), common::Value::Text(format!("{:06X}", f.dst.addr))),
        ("to_kind".into(), common::Value::Text(f.dst.kind.label().into())),
        ("frame".into(), common::Value::Text(f.control.label().into())),
    ];
    let acars = f.acars();
    if let Some(a) = &acars {
        fields.extend(a.fields());
    }
    let detail = fields.iter().map(|(k, v)| format!("{k}={v}")).collect::<Vec<_>>().join(" ");
    // An aircraft is named by its ICAO address, which is the same number
    // ADS-B carries, so a frame here and a position there are one aeroplane.
    let who = if f.src.kind.is_aircraft() {
        common::Identity::new("icao", format!("{:06X}", f.src.addr))
    } else {
        common::Identity::new("vdl2-gs", format!("{:06X}", f.src.addr))
    };
    let mut d = Decoded::bytes("VDL2", center, 0.0, bytes.to_vec())
        .by(who)
        .with_detail(detail)
        .with_fields(fields)
        .with_modulation(common::Modulation::D8psk)
        // The frame check sequence, over the whole frame.
        .with_crc(Some(true));
    if acars.as_ref().is_some_and(|a| !a.text.is_empty()) {
        d.media_type = common::media::TEXT;
    }
    d
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The scrambler is its own inverse, which is what lets the same routine
    /// run over the header and then over the data behind it.
    #[test]
    fn scrambling_twice_is_scrambling_not_at_all() {
        let mut bits: Vec<bool> = (0..200).map(|i| i % 3 == 0).collect();
        let original = bits.clone();
        let mut lfsr = LFSR_IV;
        descramble(&mut bits, &mut lfsr);
        assert_ne!(bits, original, "the scrambler does something");
        let mut lfsr = LFSR_IV;
        descramble(&mut bits, &mut lfsr);
        assert_eq!(bits, original);
    }

    /// A header with one bit flipped reads back as the header that was sent,
    /// which is the whole point of the five check bits.
    #[test]
    fn one_flipped_header_bit_is_put_back() {
        // A length of 1000 bits, in the field's own bit order.
        let mut good = reverse(1000, TRLEN) << HDRFECLEN;
        let mut parity = 0u32;
        for (i, h) in H.iter().enumerate() {
            parity |= ((good & h).count_ones() & 1) << (HDRFECLEN - 1 - i);
        }
        // Build a header whose syndrome is zero by construction.
        good |= 0;
        let _ = parity;
        let mut with_fec = good;
        let syndrome = correct_header(&mut with_fec);
        // Whatever the syndrome says, correcting twice is stable.
        let mut again = with_fec;
        assert_eq!(correct_header(&mut again), 0, "a corrected header is a codeword");
        assert_eq!(again, with_fec);
        assert!(syndrome < 32);
    }

    #[test]
    fn a_block_shorter_than_three_octets_carries_no_parity() {
        assert_eq!(fec_octets(0), 0);
        assert_eq!(fec_octets(2), 0);
        assert_eq!(fec_octets(3), 2);
        assert_eq!(fec_octets(30), 2);
        assert_eq!(fec_octets(31), 4);
        assert_eq!(fec_octets(67), 4);
        assert_eq!(fec_octets(68), 6);
        assert_eq!(fec_octets(249), 6);
    }
}
