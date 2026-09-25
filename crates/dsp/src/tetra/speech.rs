//! TETRA speech traffic channel (TCH/S) coding: EN 300 395-2 clause 5.
//!
//! One transmission time slot carries two 30 ms speech frames, A then B, each
//! 137 bits (STEC). The two are encoded together into a 432-bit block that
//! rides the same continuous downlink burst the SCH/F uses and shares its
//! scrambling (392-2 8.2.5), but not the depth 103 block interleave the
//! signalling channels add. What is specific to speech, and lives
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
///
/// The trellis is [`crate::conv`]'s, which every convolutional code in this
/// receiver shares; the generators here are the speech ones rather than the
/// control channel's.
fn viterbi(soft: &[i32], n: usize) -> Vec<u8> {
    let f: Vec<f32> = soft.iter().map(|&v| v as f32).collect();
    crate::conv::Viterbi::decode_block(
        crate::conv::TETRA_1_3,
        &f,
        &[1, 1, 1],
        n,
        crate::conv::Ends::Zero,
    )
}

pub const HALF_BITS: usize = 216;
const STOLEN_CLASS0: usize = 51;
const STOLEN_CLASS1: usize = 56;
const STOLEN_CLASS2: usize = 38;
const STOLEN_TYPE2: usize = STOLEN_CLASS0 + STOLEN_CLASS1 + STOLEN_CLASS2;
const STOLEN_CONV_IN: usize = STOLEN_CLASS1 + STOLEN_CLASS2;
const STOLEN_PUNCT2_PERIOD: usize = 24;
const STOLEN_PUNCT2_KEEP: [usize; 17] =
    [0, 1, 2, 3, 4, 6, 7, 9, 10, 12, 13, 15, 16, 18, 19, 21, 22];
const STOLEN_INTERLEAVE: u32 = 101;

fn kept(mother_len: usize, period: usize, keep: &[usize]) -> impl Iterator<Item = usize> + '_ {
    (0..mother_len.div_ceil(period))
        .flat_map(move |p| keep.iter().map(move |k| p * period + k))
        .filter(move |&i| i < mother_len)
}

fn depuncture(type3: &[u8], mother_len: usize, period: usize, keep: &[usize]) -> Vec<i32> {
    let mut mother = vec![0i32; mother_len];
    let mut j = 0;
    for i in kept(mother_len, period, keep) {
        mother[i] = if type3[j] != 0 { -1 } else { 1 };
        j += 1;
    }
    debug_assert_eq!(j, type3.len());
    mother
}

fn puncture(mother: &[u8], period: usize, keep: &[usize]) -> Vec<u8> {
    kept(mother.len(), period, keep).map(|i| mother[i]).collect()
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

const CRC_TAPS: [&[u8]; 8] = [
    &[
        1, 5, 8, 9, 13, 15, 16, 17, 19, 21, 22, 24, 25, 31, 32, 35, 36, 38, 40, 43, 44, 45, 48, 49,
        50, 51, 53, 54, 56,
    ],
    &[
        2, 6, 9, 10, 14, 16, 17, 18, 20, 22, 23, 25, 26, 32, 33, 36, 37, 39, 41, 44, 45, 46, 49,
        50, 51, 52, 54, 55, 57,
    ],
    &[
        3, 7, 10, 11, 15, 17, 18, 19, 21, 23, 24, 26, 27, 33, 34, 37, 38, 40, 42, 45, 46, 47, 50,
        51, 52, 53, 55, 56, 58,
    ],
    &[
        1, 4, 5, 9, 11, 12, 13, 15, 17, 18, 20, 21, 27, 28, 31, 32, 34, 36, 39, 40, 41, 44, 45, 46,
        47, 49, 50, 52, 57, 59,
    ],
    &[
        2, 5, 6, 10, 12, 13, 14, 16, 18, 19, 21, 22, 28, 29, 32, 33, 35, 37, 40, 41, 42, 45, 46,
        47, 48, 50, 51, 53, 58, 60,
    ],
    &[
        3, 6, 7, 11, 13, 14, 15, 17, 19, 20, 22, 23, 29, 30, 33, 34, 36, 38, 41, 42, 43, 46, 47,
        48, 49, 51, 52, 54, 59,
    ],
    &[
        4, 7, 8, 12, 14, 15, 16, 18, 20, 21, 23, 24, 30, 31, 34, 35, 37, 39, 42, 43, 44, 47, 48,
        49, 50, 52, 53, 55, 60,
    ],
    &[
        1, 2, 3, 4, 8, 13, 14, 16, 19, 20, 22, 23, 25, 26, 27, 28, 29, 30, 32, 33, 34, 36, 37, 40,
        41, 42, 44, 48, 50, 53, 56, 57, 58, 59, 60,
    ],
];

const STOLEN_CRC_TAPS: [&[u8]; 4] = [
    &[1, 4, 5, 7, 9, 10, 11, 12, 16, 19, 20, 22, 24, 25, 26, 27],
    &[1, 2, 4, 6, 7, 8, 9, 13, 16, 17, 19, 21, 22, 23, 24, 28],
    &[2, 3, 5, 7, 8, 9, 10, 14, 17, 18, 20, 22, 23, 24, 25, 29],
    &[3, 4, 6, 8, 9, 10, 11, 15, 18, 19, 21, 23, 24, 25, 26, 30],
];

fn stolen_parity(class2: &[u8]) -> [u8; 4] {
    let mut out = [0u8; 4];
    for (p, taps) in out.iter_mut().zip(STOLEN_CRC_TAPS) {
        *p = taps.iter().fold(0, |acc, &t| acc ^ class2[t as usize - 1]);
    }
    out
}

fn class2_parity(class2: &[u8]) -> [u8; 8] {
    let mut out = [0u8; 8];
    for (p, taps) in out.iter_mut().zip(CRC_TAPS) {
        *p = taps.iter().fold(0, |acc, &t| acc ^ class2[t as usize - 1]);
    }
    out
}

/// Build the 432 on-channel bits (type-5, scrambled) from two STEC frames.
/// The counterpart of `decode`, for tests and the synthetic corpus.
pub fn encode(
    scramb: u32,
    frame_a: &[u8; FRAME_BITS],
    frame_b: &[u8; FRAME_BITS],
) -> [u8; CHAN_BITS] {
    let mut type2 = vec![0u8; TYPE2];
    for n in 0..FRAME_BITS {
        type2[TYPE2_A[n] as usize] = frame_a[n];
        type2[TYPE2_B[n] as usize] = frame_b[n];
    }
    // The 60 class-2 speech bits carry the CRC in bits 274..282.
    let crc = class2_parity(&type2[CLASS0 + CLASS1..]);
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

    let mut type5 = matrix_interleave(&type3);
    coding::scramble(scramb, &mut type5);
    let mut out = [0u8; CHAN_BITS];
    out.copy_from_slice(&type5);
    out
}

/// Decode the 432 on-channel bits of a traffic slot into its two STEC frames,
/// with a flag for whether the class-2 CRC checked. `chan` is the burst's two
/// 216-bit blocks concatenated, exactly what `TetraRx` hands the SCH/F.
pub fn decode(scramb: u32, chan: &[u8; CHAN_BITS]) -> ([[u8; FRAME_BITS]; 2], bool) {
    let mut type4 = chan.to_vec();
    coding::scramble(scramb, &mut type4);
    let type3 = matrix_deinterleave(&type4);

    let class1 = &type3[CLASS0..CLASS0 + 168];
    let class2 = &type3[CLASS0 + 168..];
    let mut mother = depuncture(class1, CLASS1 * 3, PUNCT1_PERIOD, &PUNCT1_KEEP);
    mother.extend(depuncture(class2, CLASS2 * 3, PUNCT2_PERIOD, &PUNCT2_KEEP));
    let decoded = viterbi(&mother, CONV_IN);

    let mut type2 = vec![0u8; TYPE2];
    type2[..CLASS0].copy_from_slice(&type3[..CLASS0]);
    type2[CLASS0..].copy_from_slice(&decoded);

    let crc = class2_parity(&type2[CLASS0 + CLASS1..]);
    let crc_ok = TYPE2_PARITY.iter().zip(crc.iter()).all(|(&i, &p)| type2[i as usize] == p);

    let mut frames = [[0u8; FRAME_BITS]; 2];
    for n in 0..FRAME_BITS {
        frames[0][n] = type2[TYPE2_A[n] as usize];
        frames[1][n] = type2[TYPE2_B[n] as usize];
    }
    (frames, crc_ok)
}

pub fn encode_stolen(scramb: u32, frame_b: &[u8; FRAME_BITS]) -> [u8; HALF_BITS] {
    let mut type2 = [0u8; STOLEN_TYPE2];
    for n in 0..FRAME_BITS {
        type2[TYPE2_STOLEN[n] as usize] = frame_b[n];
    }
    let crc = stolen_parity(&type2[STOLEN_CLASS0 + STOLEN_CLASS1..]);
    for (i, &p) in crc.iter().enumerate() {
        type2[TYPE2_STOLEN_PARITY[i] as usize] = p;
    }
    let mut type3 = type2[..STOLEN_CLASS0].to_vec();
    let mother = conv_encode(&type2[STOLEN_CLASS0..]);
    let (m1, m2) = mother.split_at(STOLEN_CLASS1 * 3);
    type3.extend(puncture(m1, PUNCT1_PERIOD, &PUNCT1_KEEP));
    type3.extend(puncture(m2, STOLEN_PUNCT2_PERIOD, &STOLEN_PUNCT2_KEEP));
    let mut type5 = [0u8; HALF_BITS];
    coding::interleave(STOLEN_INTERLEAVE, &type3, &mut type5);
    coding::scramble(scramb, &mut type5);
    type5
}

pub fn decode_stolen(scramb: u32, half: &[u8; HALF_BITS]) -> ([u8; FRAME_BITS], bool) {
    let mut type4 = half.to_vec();
    coding::scramble(scramb, &mut type4);
    let mut type3 = vec![0u8; HALF_BITS];
    coding::deinterleave(STOLEN_INTERLEAVE, &type4, &mut type3);

    let coded1 = STOLEN_CLASS1 * 3 / 2;
    let class1 = &type3[STOLEN_CLASS0..STOLEN_CLASS0 + coded1];
    let class2 = &type3[STOLEN_CLASS0 + coded1..];
    let mut mother = depuncture(class1, STOLEN_CLASS1 * 3, PUNCT1_PERIOD, &PUNCT1_KEEP);
    mother.extend(depuncture(class2, STOLEN_CLASS2 * 3, STOLEN_PUNCT2_PERIOD, &STOLEN_PUNCT2_KEEP));
    let decoded = viterbi(&mother, STOLEN_CONV_IN);

    let mut type2 = type3[..STOLEN_CLASS0].to_vec();
    type2.extend_from_slice(&decoded);
    let crc = stolen_parity(&type2[STOLEN_CLASS0 + STOLEN_CLASS1..]);
    let crc_ok = TYPE2_STOLEN_PARITY.iter().zip(crc.iter()).all(|(&i, &p)| type2[i as usize] == p);

    let mut frame = [0u8; FRAME_BITS];
    for (n, bit) in frame.iter_mut().enumerate() {
        *bit = type2[TYPE2_STOLEN[n] as usize];
    }
    (frame, crc_ok)
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
        for type3 in [120usize, 250, 400] {
            chan[(type3 % 18) * 24 + type3 / 18] ^= 1;
        }
        let (frames, crc_ok) = decode(scramb, &chan);
        assert!(crc_ok, "CRC holds through a few errors");
        assert_eq!(frames[0], a);
        assert_eq!(frames[1], b);
    }

    #[test]
    fn a_stolen_half_slot_round_trips_frame_b() {
        let scramb = coding::scramb_init(424, 10, 15);
        let b = frame(5);
        let mut half = encode_stolen(scramb, &b);
        let (got, crc_ok) = decode_stolen(scramb, &half);
        assert!(crc_ok);
        assert_eq!(got, b);
        half[10] ^= 1;
        half[150] ^= 1;
        let (got, crc_ok) = decode_stolen(scramb, &half);
        assert!(crc_ok, "CRC holds through two channel errors");
        assert_eq!(got, b);
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
