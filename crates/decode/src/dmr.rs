//! DMR signalling: the codes that say who is talking and where a burst sits.
//!
//! A DMR burst is 264 bits. A voice burst is 108 payload, 48 in the middle,
//! 108 payload; a data burst splits the middle differently, 98 info, 10 slot
//! type, 48 sync, 10 slot type, 98 info (ETSI TS 102 361-1 clause 6.2, 9.1).
//! Everything here reads one of those fields:
//!
//! - [`slot_type`] undoes the Golay(20,8) on the 20 slot-type bits, giving the
//!   colour code and what kind of data burst it is. Data type 1 is the voice
//!   LC header that opens a transmission and 2 is the terminator that closes
//!   it, which is how an over gets its real start and end rather than a guess
//!   from silence.
//! - [`full_lc`] undoes the BPTC(196,96) on a data burst's 196 info bits,
//!   which is where the header and terminator carry the whole link control.
//! - [`emb`] undoes the QR(16,7,6) on a voice burst's 16 EMB bits, giving the
//!   colour code and which quarter of an embedded LC this burst carries. It
//!   doubles as a per-burst check that a burst really is voice, which is what
//!   lets the framer keep its clock through a superframe whose sync was lost.
//! - [`EmbeddedLc`] gathers the four 32-bit fragments from bursts B to E and
//!   undoes the BPTC(128,72), so a receiver that missed the header still
//!   learns the talkgroup and the radio ID within 360 ms.
//!
//! The parity equations and the interleave constants are the ones in the
//! standard; MMDVMHost implements the same ones and was used to check these.

use common::packet::{Alert, AlertKind, Entity, Fact, Id, Link, Party, Proto, Severity};
/// One link control message: who called whom.
///
/// 72 bits, the same nine bytes whether it arrived in a header, a terminator
/// or the embedded LC of a voice superframe.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct LinkControl {
    /// Full link control opcode. 0 is a group call, 3 is unit to unit.
    pub flco: u8,
    /// Feature set ID: 0 is the standard, others are manufacturer extensions.
    pub fid: u8,
    /// Service options. Bit 7 is emergency, bit 6 privacy.
    pub options: u8,
    /// Talkgroup for a group call, or the called radio for a private one.
    pub dst: u32,
    /// The transmitting radio's ID.
    pub src: u32,
}

impl LinkControl {
    /// Parse the nine bytes. `None` for an opcode that is not a voice call,
    /// which is also the cheapest check that a decode went wrong.
    pub fn from_bytes(b: &[u8]) -> Option<Self> {
        if b.len() < 9 {
            return None;
        }
        let flco = b[0] & 0x3f;
        if flco != FLCO_GROUP && flco != FLCO_PRIVATE {
            return None;
        }
        let dst = u32::from_be_bytes([0, b[3], b[4], b[5]]);
        let src = u32::from_be_bytes([0, b[6], b[7], b[8]]);
        if src == 0 {
            return None;
        }
        Some(Self { flco, fid: b[1], options: b[2], dst, src })
    }

    pub fn group(&self) -> bool {
        self.flco == FLCO_GROUP
    }

    /// Whether the transmission is enciphered, as the service options say.
    pub fn encrypted(&self) -> bool {
        self.options & 0x40 != 0
    }

    pub fn emergency(&self) -> bool {
        self.options & 0x80 != 0
    }
}

/// Group voice channel user.
pub const FLCO_GROUP: u8 = 0;
/// Unit to unit voice channel user.
pub const FLCO_PRIVATE: u8 = 3;

/// Data burst types worth naming (TS 102 361-1 table 9.3).
pub const DT_VOICE_LC_HEADER: u8 = 1;
pub const DT_TERMINATOR_LC: u8 = 2;
pub const DT_CSBK: u8 = 3;

/// What the EMB field of a voice burst says.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Emb {
    pub colour: u8,
    /// Privacy indicator: the payload is enciphered.
    pub pi: bool,
    /// Which fragment of an embedded LC this burst carries: 1 first,
    /// 3 continuation, 2 last, 0 none.
    pub lcss: u8,
}

/// Decode the 16 EMB bits of a voice burst. `None` when the QR(16,7,6)
/// codeword is too far from any valid one to trust.
///
/// One corrected error, not the two the code could carry. Seven information
/// bits in fifteen leave 128 codewords, and 128 x 121 of the 32768 words are
/// within two of one, so noise decodes as a valid EMB about half the time and
/// a receiver that trusts it hears a fade as speech. Within one it is 6 %,
/// and the framer's own burst clock covers the rest.
pub fn emb(bits: &[u8]) -> Option<Emb> {
    if bits.len() < 16 {
        return None;
    }
    // The 16th bit is not part of the codeword.
    let mut code = 0u32;
    for &b in &bits[..15] {
        code = (code << 1) | u32::from(b & 1);
    }
    let v = nearest(code, 7, 1, qr_encode)?;
    Some(Emb { colour: (v >> 3) as u8 & 0x0f, pi: v & 0x04 != 0, lcss: v as u8 & 0x03 })
}

/// Decode the 20 slot-type bits of a data burst into colour code and data
/// type. `None` when the Golay(20,8) codeword is too damaged.
pub fn slot_type(bits: &[u8]) -> Option<(u8, u8)> {
    if bits.len() < 20 {
        return None;
    }
    // MMDVM ignores the final parity bit, and so do we: the shortened
    // Golay(19,8) still corrects the errors that matter here.
    let mut code = 0u32;
    for &b in &bits[..19] {
        code = (code << 1) | u32::from(b & 1);
    }
    let v = nearest(code, 8, 2, golay_encode)?;
    Some(((v >> 4) as u8 & 0x0f, v as u8 & 0x0f))
}

/// Minimum-distance decode of a short block code by trying every codeword.
/// 128 or 256 candidates per burst is nothing next to the demodulation, and
/// it needs no syndrome table to be right.
fn nearest(code: u32, info_bits: u32, max_errors: u32, encode: impl Fn(u32) -> u32) -> Option<u32> {
    let mut best = (u32::MAX, 0u32);
    for v in 0..(1u32 << info_bits) {
        let d = (encode(v) ^ code).count_ones();
        if d < best.0 {
            best = (d, v);
        }
    }
    if best.0 <= max_errors { Some(best.1) } else { None }
}

/// QR(16,7,6) as its 15 used bits: seven information bits and the remainder
/// after dividing by g(x) = x^8 + x^5 + x^4 + x^3 + 1.
fn qr_encode(v: u32) -> u32 {
    (v << 8) | poly_rem(v << 8, 0x139, 8)
}

/// Golay(20,8) as its 19 used bits, g(x) = 0xc75.
fn golay_encode(v: u32) -> u32 {
    (v << 11) | poly_rem(v << 11, 0xc75, 11)
}

/// Remainder of `value` divided by `gen` over GF(2), with `deg` parity bits.
fn poly_rem(value: u32, r#gen: u32, deg: u32) -> u32 {
    let mut r = value;
    let g_deg = 32 - r#gen.leading_zeros() - 1;
    let mut shift = 32 - r.leading_zeros();
    while shift > deg {
        shift -= 1;
        if r >> shift & 1 == 1 {
            r ^= r#gen << (shift - g_deg);
        }
        shift = 32 - r.leading_zeros();
    }
    r & ((1 << deg) - 1)
}

/// Decode the 196 info bits of a data burst: BPTC(196,96) deinterleave, then
/// Hamming(15,11,3) on the rows and Hamming(13,9,3) on the columns.
///
/// Returns the 12 payload bytes, or `None` if a row or column still fails its
/// check after correction, which is what stops a noise burst being read as a
/// link control.
pub fn bptc_196_96(bits: &[u8]) -> Option<[u8; 12]> {
    if bits.len() < 196 {
        return None;
    }
    let mut d = [0u8; 196];
    for (a, slot) in d.iter_mut().enumerate() {
        *slot = bits[(a * 181) % 196] & 1;
    }

    for _ in 0..5 {
        let mut fixing = false;
        for c in 0..15 {
            let mut col = [0u8; 13];
            for (a, cell) in col.iter_mut().enumerate() {
                *cell = d[c + 1 + a * 15];
            }
            if hamming_13_9(&mut col) {
                for (a, cell) in col.iter().enumerate() {
                    d[c + 1 + a * 15] = *cell;
                }
                fixing = true;
            }
        }
        for r in 0..9 {
            let pos = r * 15 + 1;
            let mut row = [0u8; 15];
            row.copy_from_slice(&d[pos..pos + 15]);
            if hamming_15_11(&mut row) {
                d[pos..pos + 15].copy_from_slice(&row);
                fixing = true;
            }
        }
        if !fixing {
            break;
        }
    }

    // Everything must check out now, or the burst was not a valid codeword.
    for c in 0..15 {
        let mut col = [0u8; 13];
        for (a, cell) in col.iter_mut().enumerate() {
            *cell = d[c + 1 + a * 15];
        }
        if hamming_13_9(&mut col) {
            return None;
        }
    }
    for r in 0..9 {
        let pos = r * 15 + 1;
        let mut row = [0u8; 15];
        row.copy_from_slice(&d[pos..pos + 15]);
        if hamming_15_11(&mut row) {
            return None;
        }
    }

    let mut payload = [0u8; 96];
    let mut pos = 0;
    let take = |from: usize, len: usize, payload: &mut [u8; 96], pos: &mut usize| {
        payload[*pos..*pos + len].copy_from_slice(&d[from..from + len]);
        *pos += len;
    };
    take(4, 8, &mut payload, &mut pos);
    for start in [16, 31, 46, 61, 76, 91, 106, 121] {
        take(start, 11, &mut payload, &mut pos);
    }

    let mut out = [0u8; 12];
    for (i, byte) in out.iter_mut().enumerate() {
        *byte = pack(&payload[i * 8..i * 8 + 8]);
    }
    Some(out)
}

/// The full link control from a voice LC header or terminator burst, given
/// the burst's 196 info bits.
///
/// The 96 bits are nine bytes of link control and three of Reed-Solomon
/// parity, which is not checked here: the BPTC's own rows and columns have
/// already had to close, and the opcode is checked on top of that.
pub fn full_lc(info: &[u8]) -> Option<LinkControl> {
    let bytes = bptc_196_96(info)?;
    LinkControl::from_bytes(&bytes[..9])
}

/// Gathers an embedded link control from the four voice bursts that carry it.
///
/// Bursts B to E of a superframe each hold 32 bits in the middle of the burst
/// where burst A holds its sync. The EMB says which fragment is which, and
/// only the four together are a codeword.
pub struct EmbeddedLc {
    raw: [u8; 128],
    have: usize,
}

impl Default for EmbeddedLc {
    fn default() -> Self {
        Self { raw: [0; 128], have: 0 }
    }
}

impl EmbeddedLc {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn reset(&mut self) {
        self.have = 0;
    }

    /// Feed one burst's 32 embedded bits with the LCSS from its EMB. Returns
    /// the link control once the fourth fragment completes a valid one.
    pub fn push(&mut self, lcss: u8, bits: &[u8]) -> Option<LinkControl> {
        if bits.len() < 32 {
            return None;
        }
        match (lcss, self.have) {
            (1, _) => {
                self.raw[..32].copy_from_slice(&bits[..32]);
                self.have = 1;
                None
            }
            (3, 1..=2) => {
                let at = self.have * 32;
                self.raw[at..at + 32].copy_from_slice(&bits[..32]);
                self.have += 1;
                None
            }
            (2, 3) => {
                self.raw[96..128].copy_from_slice(&bits[..32]);
                self.have = 0;
                decode_embedded(&self.raw)
            }
            _ => {
                self.have = 0;
                None
            }
        }
    }
}

/// BPTC(128,72) on the four gathered fragments: Hamming(16,11,4) on each of
/// the seven rows, even parity down the columns, then a five-bit checksum on
/// the link control itself.
fn decode_embedded(raw: &[u8; 128]) -> Option<LinkControl> {
    let mut d = [0u8; 128];
    let mut b = 0usize;
    for &bit in raw.iter() {
        d[b] = bit & 1;
        b += 16;
        if b > 127 {
            b -= 127;
        }
    }

    for a in (0..112).step_by(16) {
        let mut row = [0u8; 16];
        row.copy_from_slice(&d[a..a + 16]);
        if !hamming_16_11(&mut row) {
            return None;
        }
        d[a..a + 16].copy_from_slice(&row);
    }
    for a in 0..16 {
        let mut parity = 0u8;
        for r in (0..128).step_by(16) {
            parity ^= d[a + r];
        }
        if parity != 0 {
            return None;
        }
    }

    let mut lc = [0u8; 72];
    let mut pos = 0usize;
    for (from, to) in [(0, 11), (16, 27), (32, 42), (48, 58), (64, 74), (80, 90), (96, 106)] {
        lc[pos..pos + (to - from)].copy_from_slice(&d[from..to]);
        pos += to - from;
    }

    let mut bytes = [0u8; 9];
    for (i, byte) in bytes.iter_mut().enumerate() {
        *byte = pack(&lc[i * 8..i * 8 + 8]);
    }

    let mut crc = 0u16;
    for (bit, at) in [(16u16, 42usize), (8, 58), (4, 74), (2, 90), (1, 106)] {
        if d[at] != 0 {
            crc += bit;
        }
    }
    let sum: u16 = bytes.iter().map(|&b| u16::from(b)).sum::<u16>() % 31;
    if sum != crc {
        return None;
    }

    LinkControl::from_bytes(&bytes)
}

fn pack(bits: &[u8]) -> u8 {
    bits.iter().fold(0u8, |v, &b| (v << 1) | (b & 1))
}

/// Hamming(15,11,3), the row code of BPTC(196,96). Returns whether it changed
/// anything, so a caller can tell a clean codeword from a corrected one.
fn hamming_15_11(d: &mut [u8; 15]) -> bool {
    let x = |i: usize| d[i] & 1;
    let c0 = x(0) ^ x(1) ^ x(2) ^ x(3) ^ x(5) ^ x(7) ^ x(8);
    let c1 = x(1) ^ x(2) ^ x(3) ^ x(4) ^ x(6) ^ x(8) ^ x(9);
    let c2 = x(2) ^ x(3) ^ x(4) ^ x(5) ^ x(7) ^ x(9) ^ x(10);
    let c3 = x(0) ^ x(1) ^ x(2) ^ x(4) ^ x(6) ^ x(7) ^ x(10);
    let n = (c0 ^ x(11)) | (c1 ^ x(12)) << 1 | (c2 ^ x(13)) << 2 | (c3 ^ x(14)) << 3;
    let at = match n {
        0x00 => return false,
        0x01 => 11,
        0x02 => 12,
        0x04 => 13,
        0x08 => 14,
        0x09 => 0,
        0x0b => 1,
        0x0f => 2,
        0x07 => 3,
        0x0e => 4,
        0x05 => 5,
        0x0a => 6,
        0x0d => 7,
        0x03 => 8,
        0x06 => 9,
        0x0c => 10,
        _ => return true,
    };
    d[at] ^= 1;
    true
}

/// Hamming(13,9,3), the column code of BPTC(196,96).
fn hamming_13_9(d: &mut [u8; 13]) -> bool {
    let x = |i: usize| d[i] & 1;
    let c0 = x(0) ^ x(1) ^ x(3) ^ x(5) ^ x(6);
    let c1 = x(0) ^ x(1) ^ x(2) ^ x(4) ^ x(6) ^ x(7);
    let c2 = x(0) ^ x(1) ^ x(2) ^ x(3) ^ x(5) ^ x(7) ^ x(8);
    let c3 = x(0) ^ x(2) ^ x(4) ^ x(5) ^ x(8);
    let n = (c0 ^ x(9)) | (c1 ^ x(10)) << 1 | (c2 ^ x(11)) << 2 | (c3 ^ x(12)) << 3;
    let at = match n {
        0x00 => return false,
        0x01 => 9,
        0x02 => 10,
        0x04 => 11,
        0x08 => 12,
        0x0f => 0,
        0x07 => 1,
        0x0e => 2,
        0x05 => 3,
        0x0a => 4,
        0x0d => 5,
        0x03 => 6,
        0x06 => 7,
        0x0c => 8,
        _ => return true,
    };
    d[at] ^= 1;
    true
}

/// Hamming(16,11,4), the row code of the embedded LC. Returns whether the
/// codeword was recoverable at all, correcting in place.
fn hamming_16_11(d: &mut [u8; 16]) -> bool {
    let x = |i: usize| d[i] & 1;
    let c0 = x(0) ^ x(1) ^ x(2) ^ x(3) ^ x(5) ^ x(7) ^ x(8);
    let c1 = x(1) ^ x(2) ^ x(3) ^ x(4) ^ x(6) ^ x(8) ^ x(9);
    let c2 = x(2) ^ x(3) ^ x(4) ^ x(5) ^ x(7) ^ x(9) ^ x(10);
    let c3 = x(0) ^ x(1) ^ x(2) ^ x(4) ^ x(6) ^ x(7) ^ x(10);
    let c4 = x(0) ^ x(2) ^ x(5) ^ x(6) ^ x(8) ^ x(9) ^ x(10);
    let n = (c0 ^ x(11))
        | (c1 ^ x(12)) << 1
        | (c2 ^ x(13)) << 2
        | (c3 ^ x(14)) << 3
        | (c4 ^ x(15)) << 4;
    let at = match n {
        0x00 => return true,
        0x01 => 11,
        0x02 => 12,
        0x04 => 13,
        0x08 => 14,
        0x10 => 15,
        0x19 => 0,
        0x0b => 1,
        0x1f => 2,
        0x07 => 3,
        0x0e => 4,
        0x15 => 5,
        0x1a => 6,
        0x0d => 7,
        0x13 => 8,
        0x16 => 9,
        0x1c => 10,
        _ => return false,
    };
    d[at] ^= 1;
    true
}

/// What a burst this node wrote says: who is talking to whom, and whether
/// anything about it has to be told to somebody.
///
/// How long the channel was held, which vocoder it is in and what protects it
/// are the over, and the over is stated once on the voice port where the
/// audio it is about already travels. `None` for anything this node did not
/// write, so it is safe to try on every frame.
pub fn read(bytes: &[u8]) -> Option<Proto> {
    let (kind, flags, dst, src) = match () {
        _ if bytes.len() == OVER_LEN && bytes[..2] == OVER_TAG => (
            "over",
            bytes[6],
            u32::from_be_bytes([bytes[7], bytes[8], bytes[9], bytes[10]]),
            u32::from_be_bytes([bytes[11], bytes[12], bytes[13], bytes[14]]),
        ),
        _ if bytes.len() == BODY_LEN && bytes[..2] == DMR_TAG => (
            burst_kind(bytes),
            bytes[4],
            u32::from_be_bytes([bytes[5], bytes[6], bytes[7], bytes[8]]),
            u32::from_be_bytes([bytes[9], bytes[10], bytes[11], bytes[12]]),
        ),
        _ => return None,
    };
    let mut p = Proto::new("dmr", kind);
    // The colour code tells two cells sharing a channel apart, which is the
    // same statement a network access code and a radio access number make.
    if bytes[..2] == DMR_TAG && bytes[3] != 0xff {
        p = p.saying(Fact::Infrastructure(common::packet::Cell {
            site_code: Some(u16::from(bytes[3])),
            ..Default::default()
        }));
    }
    if flags & FLAG_HAVE_LC != 0 {
        p = p.by(Entity::new("dmr", Id::Num(u64::from(src)))).between(Link::between(
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

/// Which burst it is: a voice slot, or the data slot's own type.
fn burst_kind(bytes: &[u8]) -> &'static str {
    if bytes[2] != POS_DATA {
        return "voice";
    }
    let bits = unpack_bits(&bytes[13..]);
    let mut slot = bits[98..108].to_vec();
    slot.extend_from_slice(&bits[156..166]);
    match slot_type(&slot).map(|(_, dt)| dt) {
        Some(DT_VOICE_LC_HEADER) => "voice_header",
        Some(DT_TERMINATOR_LC) => "terminator",
        Some(DT_CSBK) => "csbk",
        _ => "data",
    }
}

pub fn unpack_bits(bytes: &[u8]) -> Vec<u8> {
    bytes.iter().flat_map(|b| (0..8).rev().map(move |i| (b >> i) & 1)).collect()
}

/// Body: tag, position, colour, flags, destination, source, 264 bits.
pub const BODY_LEN: usize = 2 + 1 + 1 + 1 + 4 + 4 + BURST_BYTES;

/// Tag identifying a packet body this node wrote: one DMR burst. "DB".
///
/// A packet is one burst off the air, the 264 bits of it as received, with
/// what the framer knew when it read them: where in the superframe it sat,
/// the colour code, and the link control in force for the transmission,
/// which the burst itself carries only if it is a header or a terminator.
/// Everything the log shows about it is read back out of these bytes, so a
/// replay decodes the same burst again and a decoder written later gets its
/// chance at it. What the packet does not carry is the whole over: that is
/// reconstructed downstream from the run of bursts, the way a stream is
/// followed across frames.
pub const DMR_TAG: [u8; 2] = *b"DB";

pub const FLAG_HAVE_LC: u8 = 0x01;

pub const OVER_LEN: usize = 2 + 4 + 1 + 4 + 4;

/// The tag of the row the node used to write, one per over, kept readable
/// so an old log still labels.
pub const OVER_TAG: [u8; 2] = *b"DV";

/// Position byte: voice bursts A to F of a superframe, or a burst with a
/// data sync, whose slot type is in the bits.
pub const POS_DATA: u8 = 0xff;

/// 264 bits, packed most significant bit first.
pub const BURST_BYTES: usize = SYM_BURST * 2 / 8;

/// DMR speech is always AMBE+2 at 2450 bit/s of speech under 1150 of FEC;
/// there is no other vocoder in the standard.
pub const CODEC: &str = "AMBE+2 2450";

pub const FLAG_EMERGENCY: u8 = 0x08;

pub const FLAG_ENCRYPTED: u8 = 0x04;

pub const FLAG_GROUP: u8 = 0x02;

/// What the standard calls its own encryption, which is all a link control
/// says about it.
pub const PRIVACY: &str = "privacy";

pub const SYM_BURST: usize = SYM_PAYLOAD + SYM_SYNC + SYM_PAYLOAD;

/// A burst is 108 payload + 48 sync/embedded + 108 payload bits, which at two
/// bits a symbol is 54 + 24 + 54 = 132 symbols.
pub const SYM_PAYLOAD: usize = 54;

pub const SYM_SYNC: usize = 24;

pub const SUPERFRAME_BURSTS: usize = 6;

/// The DMR sync words as level-index strings (0=-3,1=-1,2=+1,3=+3), derived
/// from the canonical hex by mapping each dibit 01,00,10,11. Voice bursts and
/// data bursts carry different words, which is how a voice superframe is told
/// from signalling.
///
/// Beware: each voice word is the exact inverse of its data word (invert
/// `MS_voice` symbol by symbol and `MS_data` is what comes out). A
/// discriminator whose sign is unknown therefore cannot tell a voice burst
/// from a data burst by the sync alone, and picking the wrong one locks the
/// framer onto a transmission it then reads as signalling that never
/// decodes. `Framer::confirm_voice` is what settles it.
const SYNCS: [(&str, &str, bool); 6] = [
    ("BS_voice", "303333000330030030330030", true),
    ("BS_data", "030000333003303303003303", false),
    ("MS_voice", "300030033303033330030003", true),
    ("MS_data", "033303300030300003303330", false),
    ("T1_voice", "330333303000303033300000", true),
    ("T2_voice", "300300000333003333033300", true),
];

/// What one burst turned out to be, before the framer folds it into events.
enum Burst {
    Voice { frames: [[u8; 9]; 3], start: bool, lcss: u8, embedded: Vec<u8>, bits: Vec<u8> },
    Data { colour: Option<u8>, data_type: Option<u8>, lc: Option<LinkControl>, bits: Vec<u8> },
}

/// One thing the framer found. `at` is the absolute symbol index the burst
/// began at and `bits` its 264 bits as received.
pub enum DmrEvent {
    /// A voice burst: three 72-bit AMBE frames, 9 bytes each. `pos` is its
    /// place in the superframe, 0 for burst A, the one carrying the sync.
    Voice { at: usize, bits: Vec<u8>, frames: [[u8; 9]; 3], pos: u8 },
    /// Who is talking, from a header, a terminator or an embedded LC.
    Lc(LinkControl),
    /// A data/signalling burst, by its slot type (`DT_*`), or `None`
    /// when the slot type would not decode.
    Data { at: usize, bits: Vec<u8>, data_type: Option<u8> },
}

/// Finds bursts in the symbol stream and reads what they carry.
///
/// Holds a rolling window of symbol values with an absolute index, so a burst
/// whose start arrived in one block can still be read when the rest of it
/// arrives in the next.
pub struct Framer {
    /// Symbol values, oldest first.
    marks: Vec<f32>,
    /// Absolute index of `marks[0]`.
    base: usize,
    /// Next absolute index to test for a sync word while hunting.
    scan: usize,
    /// First symbol of the next expected burst, once the clock is locked.
    next: Option<usize>,
    /// Consecutive expected bursts that passed no check.
    misses: u32,
    /// Bursts since the last voice sync, so B to F of a superframe are known
    /// by where they are rather than by an EMB field that decodes noise as
    /// valid a fair fraction of the time.
    since_sync: usize,
    /// Colour code of the system being followed, so another user of the same
    /// channel does not steal the lock.
    pub colour: Option<u8>,
    /// Sync polarity once locked: the discriminator's sign is receiver-set.
    polarity: Option<bool>,
    /// Parsed sync patterns as level indices.
    patterns: Vec<(&'static str, Vec<u8>, bool)>,
    /// The four embedded LC fragments of a superframe, as they arrive.
    embedded: EmbeddedLc,
}

impl Default for Framer {
    fn default() -> Self {
        Self::new()
    }
}

impl Framer {
    pub fn new() -> Self {
        let patterns = SYNCS
            .iter()
            .map(|(n, p, v)| (*n, p.bytes().map(|c| c - b'0').collect::<Vec<u8>>(), *v))
            .collect();
        Self {
            marks: Vec::new(),
            base: 0,
            scan: 0,
            next: None,
            misses: 0,
            since_sync: usize::MAX,
            colour: None,
            polarity: None,
            patterns,
            embedded: EmbeddedLc::new(),
        }
    }

    pub fn reset(&mut self) {
        self.marks.clear();
        self.base = 0;
        self.scan = 0;
        self.next = None;
        self.misses = 0;
        self.since_sync = usize::MAX;
        self.colour = None;
        self.polarity = None;
        self.embedded.reset();
    }

    /// Fit four level centres to a window by percentiles. The window must
    /// contain all four levels for the inner two centres to be right, so it
    /// is always a whole burst or more, never the sync symbols alone (which
    /// carry only the outer two levels).
    fn centers(window: &[f32]) -> [f32; 4] {
        let mut sorted: Vec<f32> = window.to_vec();
        sorted.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        let q = |f: f32| sorted[((sorted.len() as f32 * f) as usize).min(sorted.len() - 1)];
        [q(0.12), q(0.37), q(0.62), q(0.87)]
    }

    /// Map symbol values to level indices 0..3 using given centres.
    fn apply(vals: &[f32], centers: &[f32; 4], flip: bool) -> Vec<u8> {
        vals.iter()
            .map(|&v| {
                let mut best = 0u8;
                let mut bd = f32::INFINITY;
                for (i, &c) in centers.iter().enumerate() {
                    let d = (v - c).abs();
                    if d < bd {
                        bd = d;
                        best = i as u8;
                    }
                }
                if flip { 3 - best } else { best }
            })
            .collect()
    }

    /// Level index -> dibit (DMR +3=01,+1=00,-1=10,-3=11), MSB first.
    fn dibit(l: u8) -> [u8; 2] {
        match l {
            3 => [0, 1],
            2 => [0, 0],
            1 => [1, 0],
            _ => [1, 1],
        }
    }

    /// The 264 bits of the burst starting at absolute index `start`, or None
    /// if it is not fully buffered. Levels are fitted over the whole burst,
    /// which is the only window that contains all four of them.
    fn burst_bits(&self, start: usize, flip: bool) -> Option<Vec<u8>> {
        if start < self.base || start + SYM_BURST > self.base + self.marks.len() {
            return None;
        }
        let s = start - self.base;
        let window = &self.marks[s..s + SYM_BURST];
        let centers = Self::centers(window);
        let lv = Self::apply(window, &centers, flip);
        let mut bits = Vec::with_capacity(SYM_BURST * 2);
        for &l in &lv {
            bits.extend_from_slice(&Self::dibit(l));
        }
        Some(bits)
    }

    /// The three AMBE frames of a voice burst: 108 bits either side of the
    /// middle field, nine bytes each.
    pub fn voice_frames(bits: &[u8]) -> [[u8; 9]; 3] {
        let payload: Vec<u8> = bits[..SYM_PAYLOAD * 2]
            .iter()
            .chain(&bits[(SYM_PAYLOAD + SYM_SYNC) * 2..])
            .copied()
            .collect();
        let mut frames = [[0u8; 9]; 3];
        for (f, frame) in frames.iter_mut().enumerate() {
            for (b, byte) in payload[f * 72..(f + 1) * 72].chunks(8).enumerate() {
                frame[b] = byte.iter().fold(0u8, |v, &bit| (v << 1) | (bit & 1));
            }
        }
        frames
    }

    /// Whether the burst at `start` really is burst B of a voice superframe,
    /// used to settle the polarity: its middle field has to hold an EMB the
    /// QR(16,7,6) accepts, which the same burst read the other way up does
    /// not. Without this a wrongly inverted lock reads a voice call as
    /// signalling and drops it.
    fn confirm_voice(&self, start: usize, flip: bool) -> bool {
        let Some(bits) = self.burst_bits(start, flip) else {
            return false;
        };
        let mid = &bits[SYM_PAYLOAD * 2..(SYM_PAYLOAD + SYM_SYNC) * 2];
        let mut emb_bits = mid[..8].to_vec();
        emb_bits.extend_from_slice(&mid[40..48]);
        emb(&emb_bits).is_some_and(|e| self.colour.is_none_or(|c| c == e.colour))
    }

    /// Dibit -> level index, the inverse of [`Framer::dibit`].
    fn level(d: &[u8]) -> u8 {
        match (d[0] & 1, d[1] & 1) {
            (0, 1) => 3,
            (0, 0) => 2,
            (1, 0) => 1,
            _ => 0,
        }
    }

    /// Read the burst starting at absolute index `start` and say what it is.
    ///
    /// `hunting` is the stricter test used with no clock: a sync word has to
    /// match closely and an EMB is not enough, because seven information bits
    /// will match noise often enough to lock onto nothing.
    fn classify(&self, start: usize, flip: bool, hunting: bool) -> Option<Burst> {
        let bits = self.burst_bits(start, flip)?;
        let mid = &bits[SYM_PAYLOAD * 2..(SYM_PAYLOAD + SYM_SYNC) * 2];
        let lv: Vec<u8> = mid.chunks(2).map(Self::level).collect();
        let tol = if hunting { 2 } else { 4 };
        for (_name, pat, voice) in &self.patterns {
            let err = lv.iter().zip(pat).filter(|(a, b)| a != b).count();
            if err > tol {
                continue;
            }
            if *voice {
                return Some(Burst::Voice {
                    frames: Self::voice_frames(&bits),
                    start: true,
                    lcss: 0,
                    embedded: Vec::new(),
                    bits,
                });
            }
            let mut slot = bits[98..108].to_vec();
            slot.extend_from_slice(&bits[156..166]);
            // A data sync with no readable slot type is a sync word matched
            // in noise: the Golay(20,8) over it is the second opinion.
            let (cc, dt) = slot_type(&slot)?;
            if self.colour.is_some_and(|c| c != cc) {
                return None;
            }
            let lc = match dt {
                DT_VOICE_LC_HEADER | DT_TERMINATOR_LC => {
                    let mut info = bits[0..98].to_vec();
                    info.extend_from_slice(&bits[166..264]);
                    full_lc(&info)
                }
                _ => None,
            };
            return Some(Burst::Data { colour: Some(cc), data_type: Some(dt), lc, bits });
        }
        if hunting {
            return None;
        }
        // No sync, so this should be burst B to F of a superframe: the middle
        // field is EMB, embedded signalling, EMB. Which fragment it carries
        // is decided by the position in the superframe, since the burst clock
        // is a stronger statement than seven information bits are.
        let mut emb_bits = mid[..8].to_vec();
        emb_bits.extend_from_slice(&mid[40..48]);
        let e = emb(&emb_bits);
        let pos = self.since_sync.saturating_add(1);
        if let Some(e) = e {
            if self.colour.is_some_and(|c| c != e.colour) {
                return None;
            }
        } else if pos >= SUPERFRAME_BURSTS {
            // Out of the superframe the sync anchored, with nothing in the
            // burst itself saying it is voice: this is noise or another
            // system, not the transmission being followed.
            return None;
        }
        // Burst B carries the first LC fragment, C and D continuations, E
        // the last; F carries none. `since_sync` still counts the burst
        // before this one, so B is one past a zero.
        let lcss = match pos {
            1 => 1,
            2 | 3 => 3,
            4 => 2,
            _ => 0,
        };
        Some(Burst::Voice {
            frames: Self::voice_frames(&bits),
            start: false,
            lcss,
            embedded: mid[8..40].to_vec(),
            bits,
        })
    }

    /// Append recovered symbols and pull out the bursts they complete.
    pub fn push(&mut self, syms: &[f32], out: &mut Vec<DmrEvent>) {
        self.marks.extend_from_slice(syms);
        loop {
            let last = self.base + self.marks.len();
            match self.next {
                Some(next) => {
                    if next + SYM_BURST + REANCHOR > last || next < self.base + REANCHOR {
                        break;
                    }
                    let flip = self.polarity.unwrap_or(false);
                    // Nearest first: a burst is far more likely on time than
                    // a symbol out, and taking the first match at the wrong
                    // offset would drag the clock off.
                    let mut hit = None;
                    for off in [0isize, -1, 1, -2, 2] {
                        if off.unsigned_abs() > REANCHOR {
                            continue;
                        }
                        let at = next.wrapping_add_signed(off);
                        if let Some(b) = self.classify(at, flip, false) {
                            hit = Some((at, b));
                            break;
                        }
                    }
                    match hit {
                        Some((at, burst)) => {
                            self.misses = 0;
                            self.next = Some(at + SLOT_STRIDE);
                            self.emit(at, burst, out);
                        }
                        None => {
                            self.misses += 1;
                            self.since_sync = self.since_sync.saturating_add(1);
                            if self.misses > MAX_MISSES {
                                self.next = None;
                                self.colour = None;
                                self.embedded.reset();
                                self.scan = self.scan.max(next);
                            } else {
                                self.next = Some(next + SLOT_STRIDE);
                            }
                        }
                    }
                }
                None => {
                    self.scan = self.scan.max(self.base);
                    let mut locked = false;
                    // Confirming a voice lock reads the following burst, so
                    // hunting needs that much buffered before it commits.
                    let mut waiting = false;
                    while self.scan + SYM_BURST <= last {
                        let polarities: [bool; 2] = match self.polarity {
                            Some(p) => [p, p],
                            None => [false, true],
                        };
                        let mut found = None;
                        for flip in polarities {
                            let Some(b) = self.classify(self.scan, flip, true) else {
                                continue;
                            };
                            if matches!(b, Burst::Voice { start: true, .. }) {
                                if self.scan + SLOT_STRIDE + SYM_BURST > last {
                                    waiting = true;
                                    break;
                                }
                                if !self.confirm_voice(self.scan + SLOT_STRIDE, flip) {
                                    continue;
                                }
                            }
                            found = Some((flip, b));
                            break;
                        }
                        if waiting {
                            break;
                        }
                        if let Some((flip, burst)) = found {
                            self.polarity = Some(flip);
                            self.misses = 0;
                            self.next = Some(self.scan + SLOT_STRIDE);
                            self.emit(self.scan, burst, out);
                            locked = true;
                            break;
                        }
                        self.scan += 1;
                    }
                    if !locked || waiting {
                        break;
                    }
                }
            }
        }
        // Drain marks behind whatever is still to be read.
        let keep = self
            .next
            .map_or(self.scan, |n| n.saturating_sub(REANCHOR))
            .min(self.scan.max(self.base));
        if keep > self.base {
            let drop = (keep - self.base).min(self.marks.len());
            self.marks.drain(..drop);
            self.base += drop;
        }
    }

    /// Turn a read burst into the events the node acts on, gathering the
    /// embedded link control as the fragments arrive.
    fn emit(&mut self, at: usize, burst: Burst, out: &mut Vec<DmrEvent>) {
        match burst {
            Burst::Voice { frames, start, lcss, embedded, bits } => {
                if start {
                    self.since_sync = 0;
                    self.embedded.reset();
                } else {
                    self.since_sync = self.since_sync.saturating_add(1);
                    if let Some(lc) = self.embedded.push(lcss, &embedded) {
                        out.push(DmrEvent::Lc(lc));
                    }
                }
                let pos = self.since_sync.min(5) as u8;
                out.push(DmrEvent::Voice { at, bits, frames, pos });
            }
            Burst::Data { colour, data_type, lc, bits } => {
                self.since_sync = usize::MAX;
                if let Some(cc) = colour {
                    self.colour = Some(cc);
                }
                if let Some(lc) = lc {
                    out.push(DmrEvent::Lc(lc));
                }
                out.push(DmrEvent::Data { at, bits, data_type });
            }
        }
    }
}

/// Serialise one burst with the framer's context for it.
pub fn encode_burst(pos: u8, colour: Option<u8>, lc: Option<&LinkControl>, bits: &[u8]) -> Vec<u8> {
    let mut v = DMR_TAG.to_vec();
    v.push(pos);
    v.push(colour.unwrap_or(0xff));
    v.push(lc_flags(lc));
    v.extend_from_slice(&lc.map_or(0, |l| l.dst).to_be_bytes());
    v.extend_from_slice(&lc.map_or(0, |l| l.src).to_be_bytes());
    v.extend(pack_bits(bits));
    v
}

/// How far either side of the expected burst position to look when locked.
/// The Gardner loop holds the symbol clock; this absorbs the symbol or two a
/// re-lock after fading can be out by.
pub const REANCHOR: usize = 2;

/// Bursts on one timeslot are 288 symbols apart (60 ms, one two-slot TDMA
/// frame). A voice superframe is six of them.
pub const SLOT_STRIDE: usize = 288;

/// Bursts that pass no check before the clock is abandoned and the sync hunt
/// starts again. Six is one superframe, long enough to ride through a fade
/// that would otherwise end the over.
pub const MAX_MISSES: u32 = 8;

pub fn pack_bits(bits: &[u8]) -> Vec<u8> {
    bits.chunks(8).map(|c| c.iter().fold(0u8, |v, &b| (v << 1) | (b & 1))).collect()
}

pub fn lc_flags(lc: Option<&LinkControl>) -> u8 {
    let Some(lc) = lc else { return 0 };
    let mut flags = FLAG_HAVE_LC;
    if lc.group() {
        flags |= FLAG_GROUP;
    }
    if lc.encrypted() {
        flags |= FLAG_ENCRYPTED;
    }
    if lc.emergency() {
        flags |= FLAG_EMERGENCY;
    }
    flags
}

#[cfg(test)]
mod tests {
    use super::*;

    fn unpack(bytes: &[u8]) -> Vec<u8> {
        bytes.iter().flat_map(|b| (0..8).map(move |i| (b >> (7 - i)) & 1)).collect()
    }

    /// Encode a 96-bit payload the way a radio does, so the decoder can be
    /// tested against a codeword it did not make itself.
    fn bptc_encode(payload: &[u8; 96]) -> Vec<u8> {
        let mut d = [0u8; 196];
        let mut pos = 0usize;
        let put = |from: usize, to: usize, d: &mut [u8; 196], pos: &mut usize| {
            for a in from..=to {
                d[a] = payload[*pos];
                *pos += 1;
            }
        };
        put(4, 11, &mut d, &mut pos);
        for start in [16, 31, 46, 61, 76, 91, 106, 121] {
            put(start, start + 10, &mut d, &mut pos);
        }
        for r in 0..9 {
            let p = r * 15 + 1;
            let x: Vec<u8> = d[p..p + 11].to_vec();
            d[p + 11] = x[0] ^ x[1] ^ x[2] ^ x[3] ^ x[5] ^ x[7] ^ x[8];
            d[p + 12] = x[1] ^ x[2] ^ x[3] ^ x[4] ^ x[6] ^ x[8] ^ x[9];
            d[p + 13] = x[2] ^ x[3] ^ x[4] ^ x[5] ^ x[7] ^ x[9] ^ x[10];
            d[p + 14] = x[0] ^ x[1] ^ x[2] ^ x[4] ^ x[6] ^ x[7] ^ x[10];
        }
        for c in 0..15 {
            let x: Vec<u8> = (0..9).map(|i| d[c + 1 + i * 15]).collect();
            d[c + 1 + 9 * 15] = x[0] ^ x[1] ^ x[3] ^ x[5] ^ x[6];
            d[c + 1 + 10 * 15] = x[0] ^ x[1] ^ x[2] ^ x[4] ^ x[6] ^ x[7];
            d[c + 1 + 11 * 15] = x[0] ^ x[1] ^ x[2] ^ x[3] ^ x[5] ^ x[7] ^ x[8];
            d[c + 1 + 12 * 15] = x[0] ^ x[2] ^ x[4] ^ x[5] ^ x[8];
        }
        let mut out = vec![0u8; 196];
        for a in 0..196 {
            out[(a * 181) % 196] = d[a];
        }
        out
    }

    fn lc_bytes(flco: u8, dst: u32, src: u32) -> [u8; 9] {
        let mut b = [0u8; 9];
        b[0] = flco;
        b[3] = (dst >> 16) as u8;
        b[4] = (dst >> 8) as u8;
        b[5] = dst as u8;
        b[6] = (src >> 16) as u8;
        b[7] = (src >> 8) as u8;
        b[8] = src as u8;
        b
    }

    #[test]
    fn reads_a_full_link_control_and_corrects_a_bit() {
        let mut payload = [0u8; 96];
        let bits = unpack(&lc_bytes(FLCO_GROUP, 91, 2_345_678));
        payload[..72].copy_from_slice(&bits);
        let mut coded = bptc_encode(&payload);
        let lc = full_lc(&coded).expect("a link control");
        assert_eq!((lc.dst, lc.src, lc.group()), (91, 2_345_678, true));
        coded[37] ^= 1;
        assert_eq!(full_lc(&coded).map(|l| l.src), Some(2_345_678));
    }

    /// The EMB is seven information bits in fifteen, so it accepts a fair
    /// share of noise however carefully it is decoded. This pins the rate a
    /// caller has to plan around: the DMR node uses it only to confirm a
    /// burst the clock already expected, and 6 % of bursts passing on noise
    /// is survivable where 50 % was not.
    #[test]
    fn emb_accepts_noise_only_rarely() {
        for (v, want) in [(0u32, 0x0000u32), (1, 0x0273), (3, 0x0696), (127, 0xFE5B)] {
            // MMDVM's encoding table holds the codeword in its top 15 bits.
            assert_eq!(qr_encode(v), want >> 1, "v={v}");
        }
        let mut rng = 0x1234_5678u32;
        let mut accept = 0;
        for _ in 0..10_000 {
            let bits: Vec<u8> = (0..16)
                .map(|_| {
                    rng = rng.wrapping_mul(1_103_515_245).wrapping_add(12_345);
                    ((rng >> 16) & 1) as u8
                })
                .collect();
            accept += u32::from(emb(&bits).is_some());
        }
        assert!((300..900).contains(&accept), "{accept} of 10000 random words accepted");
    }

    #[test]
    fn refuses_noise_as_a_link_control() {
        let mut rng = 0x1234_5678u32;
        let noise: Vec<u8> = (0..196)
            .map(|_| {
                rng = rng.wrapping_mul(1_103_515_245).wrapping_add(12_345);
                ((rng >> 16) & 1) as u8
            })
            .collect();
        assert!(full_lc(&noise).is_none());
    }

    #[test]
    fn round_trips_the_slot_type_and_emb_codes() {
        for cc in 0..16u8 {
            for dt in 0..16u8 {
                let v = (u32::from(cc) << 4) | u32::from(dt);
                let code = golay_encode(v);
                let mut bits: Vec<u8> = (0..19).map(|i| ((code >> (18 - i)) & 1) as u8).collect();
                bits.push(0);
                bits[3] ^= 1;
                assert_eq!(slot_type(&bits), Some((cc, dt)));
            }
        }
        for cc in 0..16u8 {
            for lcss in 0..4u8 {
                let v = (u32::from(cc) << 3) | u32::from(lcss);
                let code = qr_encode(v);
                let mut bits: Vec<u8> = (0..15).map(|i| ((code >> (14 - i)) & 1) as u8).collect();
                bits.push(0);
                bits[7] ^= 1;
                let e = emb(&bits).expect("an EMB");
                assert_eq!((e.colour, e.lcss), (cc, lcss));
            }
        }
    }

    /// The embedded LC is only a codeword once all four fragments are in, and
    /// the five-bit checksum has to hold across them.
    #[test]
    fn gathers_an_embedded_link_control() {
        let bytes = lc_bytes(FLCO_PRIVATE, 1_234, 5_678);
        let lc_bits = unpack(&bytes);
        let mut d = [0u8; 128];
        let mut pos = 0usize;
        for (from, to) in [(0, 11), (16, 27), (32, 42), (48, 58), (64, 74), (80, 90), (96, 106)] {
            for a in from..to {
                d[a] = lc_bits[pos];
                pos += 1;
            }
        }
        let crc = bytes.iter().map(|&b| u16::from(b)).sum::<u16>() % 31;
        for (bit, at) in [(16u16, 42usize), (8, 58), (4, 74), (2, 90), (1, 106)] {
            d[at] = u8::from(crc & bit != 0);
        }
        for a in (0..112).step_by(16) {
            let x: Vec<u8> = d[a..a + 11].to_vec();
            d[a + 11] = x[0] ^ x[1] ^ x[2] ^ x[3] ^ x[5] ^ x[7] ^ x[8];
            d[a + 12] = x[1] ^ x[2] ^ x[3] ^ x[4] ^ x[6] ^ x[8] ^ x[9];
            d[a + 13] = x[2] ^ x[3] ^ x[4] ^ x[5] ^ x[7] ^ x[9] ^ x[10];
            d[a + 14] = x[0] ^ x[1] ^ x[2] ^ x[4] ^ x[6] ^ x[7] ^ x[10];
            d[a + 15] = x[0] ^ x[2] ^ x[5] ^ x[6] ^ x[8] ^ x[9] ^ x[10];
        }
        for a in 0..16 {
            d[a + 112] = (0..112).step_by(16).fold(0u8, |p, r| p ^ d[a + r]);
        }
        let mut raw = [0u8; 128];
        let mut b = 0usize;
        for slot in raw.iter_mut() {
            *slot = d[b];
            b += 16;
            if b > 127 {
                b -= 127;
            }
        }

        let mut asm = EmbeddedLc::new();
        assert!(asm.push(1, &raw[0..32]).is_none());
        assert!(asm.push(3, &raw[32..64]).is_none());
        assert!(asm.push(3, &raw[64..96]).is_none());
        let lc = asm.push(2, &raw[96..128]).expect("a link control");
        assert_eq!((lc.dst, lc.src, lc.group()), (1_234, 5_678, false));

        // A fragment missed means no LC rather than a wrong one.
        assert!(asm.push(1, &raw[0..32]).is_none());
        assert!(asm.push(3, &raw[32..64]).is_none());
        assert!(asm.push(2, &raw[96..128]).is_none());
    }
}
