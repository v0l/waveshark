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
        Some(Self {
            flco,
            fid: b[1],
            options: b[2],
            dst,
            src,
        })
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
    Some(Emb {
        colour: (v >> 3) as u8 & 0x0f,
        pi: v & 0x04 != 0,
        lcss: v as u8 & 0x03,
    })
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
    if best.0 <= max_errors {
        Some(best.1)
    } else {
        None
    }
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
fn poly_rem(value: u32, gen: u32, deg: u32) -> u32 {
    let mut r = value;
    let g_deg = 32 - gen.leading_zeros() - 1;
    let mut shift = 32 - r.leading_zeros();
    while shift > deg {
        shift -= 1;
        if r >> shift & 1 == 1 {
            r ^= gen << (shift - g_deg);
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
        Self {
            raw: [0; 128],
            have: 0,
        }
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
    for (from, to) in [
        (0, 11),
        (16, 27),
        (32, 42),
        (48, 58),
        (64, 74),
        (80, 90),
        (96, 106),
    ] {
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

#[cfg(test)]
mod tests {
    use super::*;

    fn unpack(bytes: &[u8]) -> Vec<u8> {
        bytes
            .iter()
            .flat_map(|b| (0..8).map(move |i| (b >> (7 - i)) & 1))
            .collect()
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
        assert!(
            (300..900).contains(&accept),
            "{accept} of 10000 random words accepted"
        );
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
        for (from, to) in [
            (0, 11),
            (16, 27),
            (32, 42),
            (48, 58),
            (64, 74),
            (80, 90),
            (96, 106),
        ] {
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
