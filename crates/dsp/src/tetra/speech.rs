//! TETRA speech traffic channel (TCH/S) coding: EN 300 395-2 clause 5.
//!
//! One transmission time slot carries two 30 ms speech frames, A then B, each
//! 137 bits (STEC). The two are encoded together into a 432-bit block that
//! rides the same continuous downlink burst the SCH/F uses, so the physical
//! layer up to `TetraRx` is shared: scrambling (392-2 8.2.5) and block
//! interleaving with depth 103 (8.2.4). What is specific to speech, and lives
//! here, is the reordering and error control of clause 5:
//!
//!   type-1  two 137-bit STEC frames                 (274 speech bits)
//!   type-2  286 bits: 102 class 0, 112 class 1,      (table 5 reorders these
//!           60 class 2 + 8 CRC + 4 tail               from the speech frames)
//!   type-3  432 bits: class 0 verbatim, class 1 by a  16-state RCPC of rate
//!           2/3, class 2 by rate 8/18, the mother code continuous across the
//!           class 1/class 2 boundary
//!   type-4  432 bits, the (24,18) matrix transposed
//!
//! The convolutional code is the rate 1/3 mother of clause 5.4.3.1, a
//! different code from the rate 1/4 the control channels use, so it carries
//! its own generators and Viterbi here.
//!
//! Frame stealing (clause 5.6), where the first half slot is signalling and
//! only frame B is speech, is not decoded yet; its reorder table is present.

use super::coding;

include!("stec_tables.rs");

/// Bits carried by one speech block on the channel, both halves of the burst.
pub const CHAN_BITS: usize = 432;
/// One STEC speech frame.
pub const FRAME_BITS: usize = 137;

// Class sizes in the type-2 block (5.5.1, 5.5.2).
const CLASS0: usize = 102; // 2 x 51, unprotected
const CLASS1: usize = 112; // 2 x 56, RCPC 2/3
const CLASS2: usize = 72; //  2 x 30 + 8 CRC + 4 tail, RCPC 8/18
const TYPE2: usize = CLASS0 + CLASS1 + CLASS2; // 286
const CONV_IN: usize = CLASS1 + CLASS2; // 184, the continuous encoder input

// The rate 1/3 mother code, K = 5 (5.4.3.1): G1 = 1+D+D^2+D^3+D^4,
// G2 = 1+D+D^3+D^4, G3 = 1+D^2+D^4, in the window layout coding.rs uses (input
// at bit 4, the four delays in bits 0..3, most recent in bit 0). Verified
// branch by branch against osmo-tetra's conv_tch tables.
const GEN_TCH: [u8; 3] = [0b11111, 0b11101, 0b11010];

// Puncturing as the set of mother positions kept in each period (5.5.2.1-2).
// Class 1 rate 8/12 = 2/3: keep {1,2,4} of every 6. Class 2 rate 8/18: keep
// {1,2,3,4,5,7,8,10,11} of every 12. One-based in the spec, zero-based here.
const PUNCT1_PERIOD: usize = 6;
const PUNCT1_KEEP: [usize; 3] = [0, 1, 3];
const PUNCT2_PERIOD: usize = 12;
const PUNCT2_KEEP: [usize; 9] = [0, 1, 2, 3, 4, 6, 7, 9, 10];

fn branch(window: u8) -> [u8; 3] {
    let mut o = [0u8; 3];
    for (g, out) in GEN_TCH.iter().zip(o.iter_mut()) {
        *out = ((window & g).count_ones() & 1) as u8;
    }
    o
}

/// Encode `CONV_IN` bits with the rate 1/3 mother from the zero state.
fn conv_encode(bits: &[u8]) -> Vec<u8> {
    let mut state = 0u8;
    let mut out = Vec::with_capacity(bits.len() * 3);
    for &b in bits {
        out.extend_from_slice(&branch((b << 4) | state));
        state = ((state << 1) | b) & 0xf;
    }
    out
}

/// Viterbi over the rate 1/3 mother: `soft` is three values per step (+1 for a
/// received 0, -1 for a 1, 0 an erasure), `n` decoded bits out. The encoder
/// ends in the zero state because the type-2 block's four tail bits are zero.
fn viterbi(soft: &[i32], n: usize) -> Vec<u8> {
    const STATES: usize = 16;
    const NEG: i32 = i32::MIN / 4;
    let mut metric = [NEG; STATES];
    metric[0] = 0;
    let mut back = vec![0u8; n * STATES];
    for step in 0..n {
        let s = &soft[step * 3..step * 3 + 3];
        let mut next = [NEG; STATES];
        for state in 0..STATES as u8 {
            if metric[state as usize] <= NEG {
                continue;
            }
            for b in 0u8..2 {
                let o = branch((b << 4) | state);
                let m: i32 = (0..3)
                    .map(|k| if o[k] == 0 { s[k] } else { -s[k] })
                    .sum::<i32>()
                    + metric[state as usize];
                let ns = (((state << 1) | b) & 0xf) as usize;
                if m > next[ns] {
                    next[ns] = m;
                    back[step * STATES + ns] = state;
                }
            }
        }
        metric = next;
    }
    // Survivor from the zero state back to the start.
    let mut out = vec![0u8; n];
    let mut state = 0u8;
    for step in (0..n).rev() {
        let prev = back[step * STATES + state as usize];
        out[step] = state & 1; // the input bit is the lsb of the state it made
        state = prev;
    }
    out
}

fn depuncture(type3: &[u8], mother_len: usize, period: usize, keep: &[usize]) -> Vec<i32> {
    let mut mother = vec![0i32; mother_len];
    let mut j = 0;
    for p in 0..mother_len / period {
        for &k in keep {
            mother[p * period + k] = if type3[j] != 0 { -1 } else { 1 };
            j += 1;
        }
    }
    debug_assert_eq!(j, type3.len());
    mother
}

fn puncture(mother: &[u8], period: usize, keep: &[usize]) -> Vec<u8> {
    let mut out = Vec::new();
    for p in 0..mother.len() / period {
        for &k in keep {
            out.push(mother[p * period + k]);
        }
    }
    out
}

/// The (24,18) matrix interleave of 5.5.3: type-3 read as 24 lines of 18 lands
/// transposed as 18 lines of 24. Its own inverse when applied with the
/// dimensions swapped, which `matrix_deinterleave` does.
fn matrix_interleave(t3: &[u8]) -> Vec<u8> {
    let mut t4 = vec![0u8; CHAN_BITS];
    for l in 0..24 {
        for c in 0..18 {
            t4[c * 24 + l] = t3[l * 18 + c];
        }
    }
    t4
}

fn matrix_deinterleave(t4: &[u8]) -> Vec<u8> {
    let mut t3 = vec![0u8; CHAN_BITS];
    for l in 0..24 {
        for c in 0..18 {
            t3[l * 18 + c] = t4[c * 24 + l];
        }
    }
    t3
}

// CRC over the 60 class-2 speech bits (5.5.1): G(X) = X^7 + X^3 + 1, seven
// parity bits, plus b8 the overall parity. Computed over the bits in the order
// they sit in the type-2 block. The bit order the standard feeds the CRC is
// worth re-checking against a real transmission before trusting the check to
// reject; for the round trip here it is self-consistent.
fn crc7(bits: &[u8]) -> [u8; 8] {
    let mut reg = [0u8; 7];
    for &b in bits {
        let fb = b ^ reg[6];
        // shift toward higher index; taps at X^7 (out) and X^3.
        let mut nr = [0u8; 7];
        nr[0] = fb;
        nr[1] = reg[0];
        nr[2] = reg[1];
        nr[3] = reg[2] ^ fb;
        nr[4] = reg[3];
        nr[5] = reg[4];
        nr[6] = reg[5];
        reg = nr;
    }
    let mut out = [0u8; 8];
    out[..7].copy_from_slice(&reg);
    let mut overall = 0u8;
    for &b in bits {
        overall ^= b;
    }
    for &p in &reg {
        overall ^= p;
    }
    out[7] = overall;
    out
}

/// Build the 432 on-channel bits (type-5, scrambled) from two STEC frames.
/// The counterpart of `decode`, for tests and the synthetic corpus.
pub fn encode(scramb: u32, frame_a: &[u8; FRAME_BITS], frame_b: &[u8; FRAME_BITS]) -> [u8; CHAN_BITS] {
    let mut type2 = vec![0u8; TYPE2];
    for n in 0..FRAME_BITS {
        type2[TYPE2_A[n] as usize] = frame_a[n];
        type2[TYPE2_B[n] as usize] = frame_b[n];
    }
    // The 60 class-2 speech bits carry the CRC in bits 274..282.
    let class2_speech: Vec<u8> = (214..274).map(|i| type2[i]).collect();
    let crc = crc7(&class2_speech);
    for (i, &p) in crc.iter().enumerate() {
        type2[TYPE2_PARITY[i] as usize] = p;
    }

    let mut type3 = vec![0u8; CHAN_BITS];
    type3[..CLASS0].copy_from_slice(&type2[..CLASS0]);
    let mother = conv_encode(&type2[CLASS0..]);
    let (m1, m2) = mother.split_at(CLASS1 * 3);
    let p1 = puncture(m1, PUNCT1_PERIOD, &PUNCT1_KEEP);
    let p2 = puncture(m2, PUNCT2_PERIOD, &PUNCT2_KEEP);
    type3[CLASS0..CLASS0 + p1.len()].copy_from_slice(&p1);
    type3[CLASS0 + p1.len()..].copy_from_slice(&p2);

    let type4 = matrix_interleave(&type3);
    let mut type5 = vec![0u8; CHAN_BITS];
    coding::interleave(103, &type4, &mut type5);
    coding::scramble(scramb, &mut type5);
    let mut out = [0u8; CHAN_BITS];
    out.copy_from_slice(&type5);
    out
}

/// Decode the 432 on-channel bits of a traffic slot into its two STEC frames,
/// with a flag for whether the class-2 CRC checked. `chan` is the burst's two
/// 216-bit blocks concatenated, exactly what `TetraRx` hands the SCH/F.
pub fn decode(scramb: u32, chan: &[u8; CHAN_BITS]) -> ([[u8; FRAME_BITS]; 2], bool) {
    let mut type5 = chan.to_vec();
    coding::scramble(scramb, &mut type5);
    let mut type4 = vec![0u8; CHAN_BITS];
    coding::deinterleave(103, &type5, &mut type4);
    let type3 = matrix_deinterleave(&type4);

    let class1 = &type3[CLASS0..CLASS0 + 168];
    let class2 = &type3[CLASS0 + 168..];
    let mut mother = depuncture(class1, CLASS1 * 3, PUNCT1_PERIOD, &PUNCT1_KEEP);
    mother.extend(depuncture(class2, CLASS2 * 3, PUNCT2_PERIOD, &PUNCT2_KEEP));
    let decoded = viterbi(&mother, CONV_IN);

    let mut type2 = vec![0u8; TYPE2];
    type2[..CLASS0].copy_from_slice(&type3[..CLASS0]);
    type2[CLASS0..].copy_from_slice(&decoded);

    let class2_speech: Vec<u8> = (214..274).map(|i| type2[i]).collect();
    let crc = crc7(&class2_speech);
    let crc_ok = TYPE2_PARITY
        .iter()
        .zip(crc.iter())
        .all(|(&i, &p)| type2[i as usize] == p);

    let mut frames = [[0u8; FRAME_BITS]; 2];
    for n in 0..FRAME_BITS {
        frames[0][n] = type2[TYPE2_A[n] as usize];
        frames[1][n] = type2[TYPE2_B[n] as usize];
    }
    (frames, crc_ok)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn frame(seed: u64) -> [u8; FRAME_BITS] {
        // A cheap deterministic bit pattern; the codec never looks at values.
        let mut x = seed | 1;
        let mut f = [0u8; FRAME_BITS];
        for b in f.iter_mut() {
            x = x.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
            *b = (x >> 63) as u8;
        }
        f
    }

    #[test]
    fn a_speech_block_round_trips_clean() {
        let scramb = coding::scramb_init(901, 1, 5);
        let a = frame(1);
        let b = frame(2);
        let chan = encode(scramb, &a, &b);
        let (frames, crc_ok) = decode(scramb, &chan);
        assert!(crc_ok, "class-2 CRC should check on a clean block");
        assert_eq!(frames[0], a, "frame A recovered");
        assert_eq!(frames[1], b, "frame B recovered");
    }

    #[test]
    fn the_viterbi_corrects_a_few_channel_errors() {
        let scramb = coding::scramb_init(206, 2, 9);
        let a = frame(3);
        let b = frame(4);
        let mut chan = encode(scramb, &a, &b);
        // Flip a few on-channel bits; the matrix interleave spreads them, and
        // the RCPC-protected classes should still decode. Class 1 (rate 2/3)
        // is the weakest, so this stays within what one block can absorb.
        for i in [44usize, 200, 360] {
            chan[i] ^= 1;
        }
        let (frames, crc_ok) = decode(scramb, &chan);
        assert!(crc_ok, "CRC holds through a few errors");
        assert_eq!(frames[0], a);
        assert_eq!(frames[1], b);
    }

    #[test]
    fn the_matrix_interleave_is_a_transpose_inverse() {
        let mut v = vec![0u8; CHAN_BITS];
        for (i, b) in v.iter_mut().enumerate() {
            *b = (i % 2) as u8;
        }
        assert_eq!(matrix_deinterleave(&matrix_interleave(&v)), v);
    }
}
