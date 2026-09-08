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
        Self {
            state: state & 0x7f,
        }
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

/// The generator polynomials, as masks over the shift register below.
///
/// The standard's are 133 and 171 octal, written with the tap on the newest
/// input bit at the top. The register here holds the newest bit at the
/// bottom, so the masks are those two reversed: 155 and 117. That is not a
/// cosmetic difference. Neither polynomial is a palindrome, so an encoder
/// with the standard's numbers written straight into this register is a
/// different code that decodes its own output perfectly and reads nothing
/// off the air, which cost an afternoon and is why the loopback test is not
/// on its own enough.
const G: [u8; 2] = [0o155, 0o117];

const fn parity(v: u8) -> u8 {
    (v.count_ones() & 1) as u8
}

/// The two output bits for input `u` from state `s`, where the state is the
/// six previous input bits with the oldest at the top.
fn outputs(u: u8, s: u8) -> (u8, u8) {
    let sr = (s << 1 | u) & 0x7f;
    (parity(sr & G[0]), parity(sr & G[1]))
}

/// Puncturing patterns, one entry per coded bit, in transmission order.
pub const P_1_2: &[u8] = &[1, 1];
pub const P_2_3: &[u8] = &[1, 1, 1, 0];
pub const P_3_4: &[u8] = &[1, 1, 1, 0, 0, 1];
/// The rate an HT frame adds at MCS 7.
pub const P_5_6: &[u8] = &[1, 1, 1, 0, 0, 1, 1, 0, 0, 1];

/// Encode and puncture. `bits` must already carry its six zero tail bits.
pub fn encode(bits: &[u8], pattern: &[u8]) -> Vec<u8> {
    let mut out = Vec::with_capacity(bits.len() * 2);
    let mut state = 0u8;
    let mut p = 0usize;
    for &u in bits {
        let (a, b) = outputs(u, state);
        for g in [a, b] {
            if pattern[p] == 1 {
                out.push(g);
            }
            p = (p + 1) % pattern.len();
        }
        state = (state << 1 | u) & 0x3f;
    }
    out
}

/// Soft-decision Viterbi over the terminated code, depuncturing as it goes.
///
/// `soft` is positive for a one. A punctured bit is fed in as exactly 0.0,
/// which is the whole reason one decoder serves all three rates. Returns the
/// decoded bits including the tail, and the fraction of received bits that
/// disagree with re-encoding the survivor, which is the only measure of
/// confidence available once the trellis has spoken.
pub fn viterbi(soft: &[f32], pattern: &[u8], count: usize) -> (Vec<u8>, f32) {
    const STATES: usize = 64;
    let mut metric = [f32::NEG_INFINITY; STATES];
    metric[0] = 0.0;
    let mut next = [f32::NEG_INFINITY; STATES];
    let mut decisions = vec![0u64; count];

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
        let mut choice = 0u64;
        for t in 0..STATES {
            let u = (t & 1) as u8;
            for from in [t >> 1, (t >> 1) | 32] {
                if metric[from] == f32::NEG_INFINITY {
                    continue;
                }
                let (g1, g2) = outputs(u, from as u8);
                let m =
                    metric[from] + if g1 == 1 { s1 } else { -s1 } + if g2 == 1 { s2 } else { -s2 };
                if m > next[t] {
                    next[t] = m;
                    choice = choice & !(1 << t) | u64::from(from >= 32) << t;
                }
            }
        }
        *step = choice;
        metric.copy_from_slice(&next);
    }

    // Traceback from whichever state ends best rather than from zero. The
    // tail terminates the code at the end of the PSDU, but the pad bits that
    // follow it to fill the last symbol are encoded too, so the block does
    // not end in state zero and a receiver that assumes it does loses the
    // last handful of bits of every frame that needed padding.
    let mut bits = vec![0u8; count];
    let mut state = metric
        .iter()
        .enumerate()
        .max_by(|a, b| a.1.partial_cmp(b.1).unwrap_or(std::cmp::Ordering::Equal))
        .map(|(i, _)| i)
        .unwrap_or(0);
    for k in (0..count).rev() {
        bits[k] = (state & 1) as u8;
        state = state >> 1 | ((decisions[k] >> state & 1) as usize) << 5;
    }

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
        let bits: Vec<u8> = "1111000100100110000000001110000000"
            .bytes()
            .map(|b| b - b'0')
            .collect();
        assert_eq!(bits.len(), 34);
        assert_eq!(ht_sig_crc(&bits), [1, 0, 1, 0, 1, 0, 0, 0]);
    }

    #[test]
    fn the_trellis_decodes_what_it_encoded_at_every_rate() {
        for pattern in [P_1_2, P_2_3, P_3_4, P_5_6] {
            let mut bits: Vec<u8> = (0..200).map(|i| (i * 5 % 7 < 3) as u8).collect();
            bits.extend([0; 6]);
            let coded = encode(&bits, pattern);
            let soft: Vec<f32> = coded
                .iter()
                .map(|&b| if b == 1 { 1.0 } else { -1.0 })
                .collect();
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
        let mut soft: Vec<f32> = coded
            .iter()
            .map(|&b| if b == 1 { 1.0 } else { -1.0 })
            .collect();
        soft[17] = -soft[17];
        soft[60] = -soft[60];
        let (got, err) = viterbi(&soft, P_1_2, bits.len());
        assert_eq!(got, bits);
        assert!(err > 0.0);
    }
}
