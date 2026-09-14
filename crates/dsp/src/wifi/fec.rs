//! The bit-level coding an 802.11a/g frame is wrapped in: the scrambler, the
//! rate 1/2 K=7 convolutional code with its puncturing, and the per-symbol
//! interleaver.
//!
//! All of it is IEEE 802.11-2020 clause 17. Nothing here knows about samples
//! or subcarriers; `ofdm` maps soft bits to constellation points and `mod`
//! runs the receiver.

/// The data scrambler, `x^7 + x^4 + 1`, 127 bits long.
///
/// The transmitter seeds it with anything non-zero and never says what, so
/// the receiver recovers the seed from the SERVICE field's first seven bits
/// being defined as zero. `descramble` does that by trying all of them,
/// which costs 127 sixteen-bit checks and cannot be subtly wrong the way an
/// algebraic inversion can.
#[derive(Clone, Copy)]
pub struct Scrambler {
    state: u8,
}

impl Scrambler {
    /// `state` is seven bits, and must not be zero: the all-zero register is
    /// a fixed point that outputs zeros for ever.
    pub fn new(state: u8) -> Self {
        Self { state: state & 0x7f }
    }

    pub fn next_bit(&mut self) -> u8 {
        let out = (self.state >> 6 & 1) ^ (self.state >> 3 & 1);
        self.state = (self.state << 1 | out) & 0x7f;
        out
    }

    /// XOR the sequence over `bits` in place. Its own inverse.
    pub fn apply(&mut self, bits: &mut [u8]) {
        for b in bits.iter_mut() {
            *b ^= self.next_bit();
        }
    }
}

/// Undo the scrambling, given that the first sixteen bits are the SERVICE
/// field and are all zero. `None` when no seed makes them so, which means
/// the frame is not a frame.
pub fn descramble(bits: &mut [u8]) -> Option<u8> {
    if bits.len() < 16 {
        return None;
    }
    for seed in 1..128u8 {
        let mut s = Scrambler::new(seed);
        if (0..16).all(|i| bits[i] ^ s.next_bit() == 0) {
            let mut s = Scrambler::new(seed);
            s.apply(bits);
            return Some(seed);
        }
    }
    None
}

/// The pilot polarity sequence, the same scrambler run from an all-ones seed
/// with no input. One value per OFDM symbol, SIGNAL first, repeating after
/// 127.
pub fn pilot_polarity(symbol: usize) -> f32 {
    static SEQ: std::sync::OnceLock<[f32; 127]> = std::sync::OnceLock::new();
    let seq = SEQ.get_or_init(|| {
        let mut s = Scrambler::new(0x7f);
        let mut v = [0.0f32; 127];
        for x in v.iter_mut() {
            *x = if s.next_bit() == 0 { 1.0 } else { -1.0 };
        }
        v
    });
    seq[symbol % 127]
}

/// Puncturing patterns, one entry per coded bit, in transmission order.
///
/// The code itself is [`crate::conv`]: 802.11 and DVB-T use the same rate
/// 1/2, constraint length 7 code, and the only differences are which output
/// goes first and how the two standards puncture it. 802.11 sends the 133
/// octal output first and calls it A.
pub const P_1_2: &[u8] = &[1, 1];
pub const P_2_3: &[u8] = &[1, 1, 1, 0];
pub const P_3_4: &[u8] = &[1, 1, 1, 0, 0, 1];
/// The rate an HT frame adds at MCS 7.
pub const P_5_6: &[u8] = &[1, 1, 1, 0, 0, 1, 1, 0, 0, 1];

/// The code, as 802.11 names its outputs: 133 octal is A and goes first.
const CODE: crate::conv::Code = crate::conv::K7_A_FIRST;

/// Encode and puncture. `bits` must already carry its six zero tail bits.
pub fn encode(bits: &[u8], pattern: &[u8]) -> Vec<u8> {
    crate::conv::Encoder::new(CODE).punctured(bits, pattern)
}

/// Soft-decision Viterbi over the terminated code, depuncturing as it goes.
///
/// `soft` is positive for a one, which is this module's convention and the
/// opposite of [`crate::conv`]'s, so the signs are turned over on the way in.
/// A punctured bit is fed in as exactly 0.0. Returns the decoded bits
/// including the tail, and the fraction of received bits that disagree with
/// re-encoding the survivor, which is the only measure of confidence
/// available once the trellis has spoken.
pub fn viterbi(soft: &[f32], pattern: &[u8], count: usize) -> (Vec<u8>, f32) {
    let flipped: Vec<f32> = soft.iter().map(|v| -v).collect();
    let bits = crate::conv::Viterbi::decode_block(
        CODE,
        &flipped,
        pattern,
        count,
        crate::conv::Ends::Anywhere,
    );

    let check = encode(&bits, pattern);
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
    (bits, wrong as f32 / total.max(1) as f32)
}

/// The interleaver's permutation for one symbol: `map[k]` is where coded bit
/// `k` is transmitted. Clause 17.3.5.7's two permutations, in that order.
///
/// `columns` is the only thing an HT symbol changes: the legacy interleaver
/// writes 16 columns and the 20 MHz HT one writes 13, which with 52 rather
/// than 48 subcarriers keeps the rows a whole number of bits per
/// constellation point.
pub fn interleave_map(n_cbps: usize, n_bpsc: usize, columns: usize) -> Vec<usize> {
    let s = (n_bpsc / 2).max(1);
    let rows = n_cbps / columns;
    (0..n_cbps)
        .map(|k| {
            let i = rows * (k % columns) + k / columns;
            s * (i / s) + (i + n_cbps - (columns * i) / n_cbps) % s
        })
        .collect()
}

/// The check an HT SIGNAL field carries over its own first 34 bits.
///
/// Eight bits, MSB of the register first and inverted, which is the shift
/// register drawn in the standard rather than any named CRC-8. It is the
/// whole reason an HT header can be trusted: unlike the legacy SIGNAL field,
/// which has one parity bit, this refuses a wrong MCS outright.
pub fn ht_sig_crc(bits: &[u8]) -> [u8; 8] {
    let mut c = [1u8; 8];
    for &m in bits {
        let f = m ^ c[7];
        c = [f, f ^ c[0], f ^ c[1], c[2], c[3], c[4], c[5], c[6]];
    }
    let mut out = [0u8; 8];
    for (i, o) in out.iter_mut().enumerate() {
        *o = 1 - c[7 - i];
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The first values of the polarity sequence are printed in the
    /// standard, and getting the register's direction wrong reproduces the
    /// first four and then diverges.
    #[test]
    fn the_pilot_polarity_matches_the_published_sequence() {
        let want = [
            1.0, 1.0, 1.0, 1.0, -1.0, -1.0, -1.0, 1.0, -1.0, -1.0, -1.0, -1.0, 1.0, 1.0, -1.0, 1.0,
            -1.0, -1.0, 1.0, 1.0, -1.0, 1.0, 1.0, -1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, -1.0, 1.0,
        ];
        for (n, w) in want.iter().enumerate() {
            assert_eq!(pilot_polarity(n), *w, "p{n}");
        }
    }

    #[test]
    fn the_scrambler_is_its_own_inverse_and_is_recovered_from_service() {
        let mut bits = vec![0u8; 16];
        bits.extend((0..64).map(|i| (i * 7 % 3 == 0) as u8));
        let orig = bits.clone();
        Scrambler::new(0x5d).apply(&mut bits);
        assert_ne!(bits, orig);
        assert_eq!(descramble(&mut bits), Some(0x5d));
        assert_eq!(bits, orig);
    }

    #[test]
    fn the_interleaver_permutation_is_a_bijection() {
        for (n_cbps, n_bpsc) in [(48, 1), (96, 2), (192, 4), (288, 6)] {
            let mut m = interleave_map(n_cbps, n_bpsc, 16);
            m.sort_unstable();
            assert!(m.iter().copied().eq(0..n_cbps), "{n_cbps}");
        }
        for (n_cbps, n_bpsc) in [(52, 1), (104, 2), (208, 4), (312, 6)] {
            let mut m = interleave_map(n_cbps, n_bpsc, 13);
            m.sort_unstable();
            assert!(m.iter().copied().eq(0..n_cbps), "HT {n_cbps}");
        }
    }

    /// The standard prints one worked example beside the shift register, and
    /// it is the only check on this that is not circular.
    #[test]
    fn the_ht_signal_check_matches_the_published_example() {
        let bits: Vec<u8> =
            "1111000100100110000000001110000000".bytes().map(|b| b - b'0').collect();
        assert_eq!(bits.len(), 34);
        assert_eq!(ht_sig_crc(&bits), [1, 0, 1, 0, 1, 0, 0, 0]);
    }

    #[test]
    fn the_trellis_decodes_what_it_encoded_at_every_rate() {
        for pattern in [P_1_2, P_2_3, P_3_4, P_5_6] {
            let mut bits: Vec<u8> = (0..200).map(|i| (i * 5 % 7 < 3) as u8).collect();
            bits.extend([0; 6]);
            let coded = encode(&bits, pattern);
            let soft: Vec<f32> = coded.iter().map(|&b| if b == 1 { 1.0 } else { -1.0 }).collect();
            let (got, err) = viterbi(&soft, pattern, bits.len());
            assert_eq!(got, bits);
            assert_eq!(err, 0.0);
        }
    }

    /// Two flipped bits inside a rate 1/2 block are what the code is for.
    #[test]
    fn the_trellis_corrects_a_few_wrong_bits() {
        let mut bits: Vec<u8> = (0..120).map(|i| (i % 3 == 0) as u8).collect();
        bits.extend([0; 6]);
        let coded = encode(&bits, P_1_2);
        let mut soft: Vec<f32> = coded.iter().map(|&b| if b == 1 { 1.0 } else { -1.0 }).collect();
        soft[17] = -soft[17];
        soft[60] = -soft[60];
        let (got, err) = viterbi(&soft, P_1_2, bits.len());
        assert_eq!(got, bits);
        assert!(err > 0.0);
    }
}
