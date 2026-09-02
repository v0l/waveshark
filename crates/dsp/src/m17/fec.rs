//! The error correction M17 wraps every frame in.
//!
//! Four codes, each doing a different job, and all of them defined in the
//! M17 specification's appendices:
//!
//! - a rate 1/2, K=5 convolutional code over the frame contents, punctured to
//!   fit whatever room the frame type leaves,
//! - Golay(24,12) over the link information channel, which has to be readable
//!   on its own so a receiver joining a transmission late can learn who is
//!   talking without the link setup frame,
//! - a quadratic permutation polynomial interleaver and a fixed 46 byte
//!   randomiser, which are not error correction but decide which bit lands
//!   where and therefore have to be undone in the right order,
//! - a CRC-16 with an unusual polynomial, which is the only thing in a
//!   transmission that says a decode is right rather than merely plausible.
//!
//! Everything here works on soft bits: an `f32` per bit, positive for a one,
//! and larger in magnitude the more the demodulator believes it. A punctured
//! bit is filled with exactly 0.0, which says "no evidence either way", and
//! that is the whole trick that lets one Viterbi decoder serve three
//! puncturing schemes.

/// The M17 CRC polynomial, x^16 + x^14 + x^12 + x^11 + x^8 + x^5 + x^4 + x^2 + 1.
///
/// Not a standard CRC-16: neither input nor output is reflected, because
/// M17's native bit order is most significant first throughout.
const CRC_POLY: u16 = 0x5935;

/// CRC-16 over a byte string, initialised to all ones.
///
/// Run over a whole link setup frame, including its own CRC field, this
/// returns zero when the frame is intact.
pub fn crc16(data: &[u8]) -> u16 {
    let mut crc = 0xFFFFu16;
    for &b in data {
        crc ^= u16::from(b) << 8;
        for _ in 0..8 {
            crc = if crc & 0x8000 != 0 { (crc << 1) ^ CRC_POLY } else { crc << 1 };
        }
    }
    crc
}

/// Puncturing for the link setup frame: 368 bits kept from 488.
pub const P1: [u8; 61] = [
    1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0,
    1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1,
];

/// Puncturing for stream frame contents: every twelfth bit dropped.
pub const P2: [u8; 12] = [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0];

/// Puncturing for packet frame contents: every eighth bit dropped.
pub const P3: [u8; 8] = [1, 1, 1, 1, 1, 1, 1, 0];

/// The randomiser sequence: 46 bytes, XORed over the 368 payload bits most
/// significant bit first, and repeated for every frame.
pub const RANDOMIZER: [u8; 46] = [
    0xD6, 0xB5, 0xE2, 0x30, 0x82, 0xFF, 0x84, 0x62, 0xBA, 0x4E, 0x96, 0x90, 0xD8, 0x98, 0xDD,
    0x5D, 0x0C, 0xC8, 0x52, 0x43, 0x91, 0x1D, 0xF8, 0x6E, 0x68, 0x2F, 0x35, 0xDA, 0x14, 0xEA,
    0xCD, 0x76, 0x19, 0x8D, 0xD5, 0x80, 0xD1, 0x33, 0x87, 0x13, 0x57, 0x18, 0x2D, 0x29, 0x78,
    0xC3,
];

/// Payload bits in a frame, which is also the interleaver's period.
pub const PAYLOAD_BITS: usize = 368;

/// The interleaver, as the quadratic permutation polynomial it is defined by
/// rather than as the table the specification prints from it.
///
/// pi(x) = (45x + 92x^2) mod 368 happens to be its own inverse, so the same
/// pass interleaves and deinterleaves. That is worth stating rather than
/// discovering: a receiver that assumed otherwise and inverted the
/// permutation would work anyway, which is exactly the kind of accident that
/// hides a misunderstanding.
const fn qpp() -> [u16; PAYLOAD_BITS] {
    let mut t = [0u16; PAYLOAD_BITS];
    let mut i = 0;
    while i < PAYLOAD_BITS {
        t[i] = ((45 * i + 92 * i * i) % PAYLOAD_BITS) as u16;
        i += 1;
    }
    t
}

pub const INTERLEAVE: [u16; PAYLOAD_BITS] = qpp();

/// Undo the randomiser on soft bits: a one in the sequence flips the bit,
/// which for a soft value means changing its sign and keeping its confidence.
pub fn derandomize(soft: &mut [f32]) {
    for (i, v) in soft.iter_mut().enumerate() {
        if RANDOMIZER[(i / 8) % RANDOMIZER.len()] >> (7 - i % 8) & 1 == 1 {
            *v = -*v;
        }
    }
}

/// The interleaver, applied to soft bits. Self-inverse; see [`INTERLEAVE`].
pub fn deinterleave(soft: &[f32], out: &mut [f32]) {
    for (i, &j) in INTERLEAVE.iter().enumerate() {
        out[i] = soft[j as usize];
    }
}

/// The two generator polynomials, G1 = 1 + D^3 + D^4 and G2 = 1 + D + D^2 +
/// D^4, evaluated for an input bit and the four before it.
///
/// `sr` holds those four, most recent in bit 0.
fn outputs(u: u8, sr: u8) -> (u8, u8) {
    let g1 = u ^ (sr >> 2 & 1) ^ (sr >> 3 & 1);
    let g2 = u ^ (sr & 1) ^ (sr >> 1 & 1) ^ (sr >> 3 & 1);
    (g1, g2)
}

/// Convolutionally encode `bits`, flush the register with four zeros, and
/// puncture the result with `pattern`.
///
/// The puncturing index advances for every encoder output rather than every
/// input bit, which is what makes P1 drop G1 on one pass and G2 on the next.
pub fn conv_encode(bits: &[u8], pattern: &[u8], out: &mut Vec<u8>) {
    out.clear();
    let mut sr = 0u8;
    let mut p = 0usize;
    for &u in bits.iter().chain([0, 0, 0, 0].iter()) {
        let (g1, g2) = outputs(u, sr);
        for g in [g1, g2] {
            if pattern[p] == 1 {
                out.push(g);
            }
            p = (p + 1) % pattern.len();
        }
        sr = (sr << 1 | u) & 0xF;
    }
}

/// Decode `soft` back to `count` content bits with a soft-decision Viterbi
/// decoder, returning the bits and the fraction of received bits the decoded
/// sequence disagrees with.
///
/// That fraction is the confidence measure the rest of the receiver runs on.
/// M17 puts no check sequence on a stream frame, so the only evidence that a
/// frame was read rather than invented is that the surviving path through the
/// trellis explains what arrived. A clean frame re-encodes to the received
/// bits exactly; noise, a wrong polarity or a false sync all show up here as
/// a disagreement rate near a half.
///
/// The transmitter flushes the register with four zeros, so the path is known
/// to end in state zero and the traceback starts there rather than at the
/// best final state.
pub fn viterbi(soft: &[f32], pattern: &[u8], count: usize) -> (Vec<u8>, f32) {
    const STATES: usize = 16;
    let steps = count + 4;
    let mut metric = [f32::NEG_INFINITY; STATES];
    metric[0] = 0.0;
    let mut next = [f32::NEG_INFINITY; STATES];
    let mut decisions = vec![0u16; steps];

    // Depuncture as we go: a bit the transmitter dropped arrives as 0.0,
    // which contributes nothing to either branch.
    let mut p = 0usize;
    let mut read = 0usize;
    let take = |p: &mut usize, read: &mut usize| -> f32 {
        let v = if pattern[*p] == 1 {
            let v = soft.get(*read).copied().unwrap_or(0.0);
            *read += 1;
            v
        } else {
            0.0
        };
        *p = (*p + 1) % pattern.len();
        v
    };

    for step in decisions.iter_mut() {
        let (s1, s2) = (take(&mut p, &mut read), take(&mut p, &mut read));
        next.fill(f32::NEG_INFINITY);
        let mut choice = 0u16;
        for t in 0..STATES {
            let u = (t & 1) as u8;
            for from in [t >> 1, (t >> 1) | 8] {
                if metric[from] == f32::NEG_INFINITY {
                    continue;
                }
                let (g1, g2) = outputs(u, from as u8);
                let m = metric[from]
                    + if g1 == 1 { s1 } else { -s1 }
                    + if g2 == 1 { s2 } else { -s2 };
                if m > next[t] {
                    next[t] = m;
                    choice = choice & !(1 << t) | u16::from(from >= 8) << t;
                }
            }
        }
        *step = choice;
        metric.copy_from_slice(&next);
    }

    let mut bits = vec![0u8; steps];
    let mut state = 0usize;
    for k in (0..steps).rev() {
        bits[k] = (state & 1) as u8;
        state = state >> 1 | usize::from(decisions[k] >> state & 1) << 3;
    }
    bits.truncate(count);

    // Re-encode and compare against what arrived, counting only the bits that
    // were actually transmitted.
    let mut check = Vec::new();
    conv_encode(&bits, pattern, &mut check);
    let mut wrong = 0usize;
    let mut total = 0usize;
    for (i, &b) in check.iter().enumerate() {
        let Some(&v) = soft.get(i) else { break };
        if v == 0.0 {
            continue;
        }
        total += 1;
        wrong += usize::from((v > 0.0) != (b == 1));
    }
    let ber = if total == 0 { 1.0 } else { wrong as f32 / total as f32 };
    (bits, ber)
}

/// The Golay(24,12) generator's parity half, one row per data bit.
///
/// Read from the specification's G = [I | P] matrix, least significant data
/// bit first, with each row holding its eleven check bits and the overall
/// parity bit in bit 0. Encoding is then a sum of the rows the data selects,
/// which is all a linear code ever is.
const GOLAY_P: [u16; 12] = [
    0x8EB, 0x93E, 0xA97, 0xDC6, 0x367, 0x6CD, 0xD99, 0x3DA, 0x7B4, 0xF68, 0x63B, 0xC75,
];

/// Golay(24,12): twelve data bits into a 24 bit codeword, data in the top
/// half.
pub fn golay_encode(data: u16) -> u32 {
    let mut check = 0u16;
    for (i, &row) in GOLAY_P.iter().enumerate() {
        if data >> i & 1 == 1 {
            check ^= row;
        }
    }
    u32::from(data & 0xFFF) << 12 | u32::from(check)
}

/// Decode a Golay codeword, correcting up to three bit errors.
///
/// Returns the data and how many bits had to be changed, or `None` when the
/// word is more than three bits from every codeword. Refusing rather than
/// guessing matters here: the link information channel is what a late
/// listener learns a callsign from, and a mis-corrected chunk is a plausible
/// looking callsign that was never transmitted.
///
/// The search is the textbook one for this code. The syndrome alone is the
/// error pattern when the errors are all in the parity half; otherwise one or
/// two data bits are wrong, and adding their generator rows to the syndrome
/// leaves a pattern light enough to recognise.
pub fn golay_decode(word: u32) -> Option<(u16, u32)> {
    let data = (word >> 12) as u16 & 0xFFF;
    let parity = (word & 0xFFF) as u16;
    let syndrome = parity ^ (golay_encode(data) & 0xFFF) as u16;

    if syndrome.count_ones() <= 3 {
        return Some((data, syndrome.count_ones()));
    }
    for i in 0..12 {
        let s = syndrome ^ GOLAY_P[i];
        if s.count_ones() <= 2 {
            return Some((data ^ (1 << i), s.count_ones() + 1));
        }
    }
    for i in 0..12 {
        for j in i + 1..12 {
            let s = syndrome ^ GOLAY_P[i] ^ GOLAY_P[j];
            if s.count_ones() <= 1 {
                return Some((data ^ (1 << i) ^ (1 << j), s.count_ones() + 2));
            }
        }
    }
    None
}

/// Unpack bytes to one bit per entry, most significant bit first.
pub fn unpack(bytes: &[u8], out: &mut Vec<u8>) {
    out.clear();
    for &b in bytes {
        for i in (0..8).rev() {
            out.push(b >> i & 1);
        }
    }
}

/// Pack bits back into bytes, most significant bit first. Trailing bits that
/// do not fill a byte are dropped.
pub fn pack(bits: &[u8]) -> Vec<u8> {
    bits.chunks_exact(8).map(|c| c.iter().fold(0u8, |acc, &b| acc << 1 | b)).collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The specification's CRC test vectors, which exist precisely because
    /// this is not a CRC any library computes by default.
    #[test]
    fn the_crc_matches_the_published_vectors() {
        assert_eq!(crc16(b""), 0xFFFF);
        assert_eq!(crc16(b"A"), 0x206E);
        assert_eq!(crc16(b"123456789"), 0x772B);
        let all: Vec<u8> = (0..=255).collect();
        assert_eq!(crc16(&all), 0x1C31);
    }

    #[test]
    fn a_frame_with_its_crc_checks_to_zero() {
        let mut lsf = vec![0x11u8; 28];
        lsf[3] = 0x7F;
        let crc = crc16(&lsf);
        lsf.extend_from_slice(&crc.to_be_bytes());
        assert_eq!(crc16(&lsf), 0);
    }

    /// Four entries from the specification's printed interleaving table, and
    /// the property that makes the table redundant.
    #[test]
    fn the_interleaver_matches_the_published_table() {
        assert_eq!(INTERLEAVE[1], 137);
        assert_eq!(INTERLEAVE[2], 90);
        assert_eq!(INTERLEAVE[47], 367);
        assert_eq!(INTERLEAVE[367], 47);
        for (i, &j) in INTERLEAVE.iter().enumerate() {
            assert_eq!(INTERLEAVE[j as usize] as usize, i, "pi is not its own inverse at {i}");
        }
    }

    #[test]
    fn the_randomizer_and_interleaver_undo_themselves() {
        let mut soft: Vec<f32> = (0..PAYLOAD_BITS).map(|i| if i % 3 == 0 { 1.0 } else { -0.5 }).collect();
        let want = soft.clone();
        let mut mid = vec![0.0f32; PAYLOAD_BITS];
        derandomize(&mut soft);
        deinterleave(&soft, &mut mid);
        let mut back = vec![0.0f32; PAYLOAD_BITS];
        deinterleave(&mid, &mut back);
        derandomize(&mut back);
        assert_eq!(back, want);
    }

    /// The three puncturing schemes have to produce exactly the bit counts
    /// the frame layouts leave room for, and nothing else in the receiver
    /// notices when they do not.
    #[test]
    fn puncturing_leaves_the_lengths_the_frames_have_room_for() {
        let mut out = Vec::new();
        conv_encode(&vec![0u8; 240], &P1, &mut out);
        assert_eq!(out.len(), 368, "link setup");
        conv_encode(&vec![0u8; 144], &P2, &mut out);
        assert_eq!(out.len(), 272, "stream contents");
        conv_encode(&vec![0u8; 206], &P3, &mut out);
        assert_eq!(out.len(), 368, "packet contents");
    }

    fn soften(bits: &[u8]) -> Vec<f32> {
        bits.iter().map(|&b| if b == 1 { 1.0 } else { -1.0 }).collect()
    }

    fn pattern(n: usize) -> Vec<u8> {
        let mut seed = 12345u64;
        (0..n)
            .map(|_| {
                seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1);
                (seed >> 60 & 1) as u8
            })
            .collect()
    }

    #[test]
    fn the_decoder_recovers_what_the_encoder_sent() {
        for (count, p) in [(240usize, &P1[..]), (144, &P2[..]), (206, &P3[..])] {
            let bits = pattern(count);
            let mut coded = Vec::new();
            conv_encode(&bits, p, &mut coded);
            let (got, ber) = viterbi(&soften(&coded), p, count);
            assert_eq!(got, bits, "clean decode failed for {count} bits");
            assert_eq!(ber, 0.0);
        }
    }

    #[test]
    fn the_decoder_carries_a_frame_through_bit_errors() {
        let bits = pattern(240);
        let mut coded = Vec::new();
        conv_encode(&bits, &P1, &mut coded);
        let mut soft = soften(&coded);
        // A dozen errors spread over the frame, which is well past what a
        // hard-decision reader could survive.
        for i in (7..soft.len()).step_by(31) {
            soft[i] = -soft[i];
        }
        let (got, ber) = viterbi(&soft, &P1, 240);
        assert_eq!(got, bits, "the code did not carry twelve errors");
        assert!(ber > 0.0 && ber < 0.1, "ber came out at {ber}");
    }

    /// What a decoder does with noise, which is not what it looks like it
    /// should do. A rate 1/2 K=5 code can explain about seven eighths of a
    /// random bit stream, so the disagreement rate on noise sits near an
    /// eighth rather than near a half. Everything upstream that uses this
    /// number as evidence has to be scaled to that gap.
    #[test]
    fn noise_decodes_to_something_that_does_not_explain_itself() {
        let mut seed = 99u64;
        let soft: Vec<f32> = (0..368)
            .map(|_| {
                seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1);
                if seed >> 60 & 1 == 1 { 1.0 } else { -1.0 }
            })
            .collect();
        let (_, ber) = viterbi(&soft, &P1, 240);
        assert!(ber > 0.07, "random bits re-encoded to themselves at {ber}");
    }

    /// The minimum weight of a linear code identifies it. Every non-zero
    /// codeword of the extended Golay code has at least eight bits set, which
    /// is what lets the decoder correct three errors and refuse four.
    #[test]
    fn the_golay_code_has_the_right_minimum_weight() {
        let min = (1u16..4096).map(|d| golay_encode(d).count_ones()).min().unwrap();
        assert_eq!(min, 8);
    }

    #[test]
    fn golay_corrects_three_errors_and_refuses_four() {
        for data in [0u16, 1, 0x555, 0xABC, 0xFFF] {
            let word = golay_encode(data);
            assert_eq!(golay_decode(word), Some((data, 0)));
            for bits in [
                [0usize, 1, 2].as_slice(),
                &[5, 13, 23],
                &[0, 12, 22],
                &[9, 10, 11],
            ] {
                let damaged = bits.iter().fold(word, |w, &b| w ^ 1 << b);
                assert_eq!(golay_decode(damaged), Some((data, 3)), "three errors at {bits:?}");
            }
            // Four errors leave the word four away from the codeword sent
            // and at least four from every other, so the decoder cannot
            // reach any of them and must say so.
            assert_eq!(golay_decode(word ^ 0b1111), None, "four errors were corrected anyway");
        }
    }
}
