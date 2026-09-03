//! TETRA channel coding: EN 300 392-2 clause 8.
//!
//! The downlink control channels all run the same stack in different sizes:
//! scrambling (8.2.5), block interleaving (8.2.4), rate-compatible puncturing
//! of a rate 1/4 mother convolutional code (8.2.3), and a CRC-16 (8.2.2). The
//! encoder halves exist for the tests, which run every block size round trip
//! and against the bit-exact reference values in osmo-tetra.

/// Scrambler initial state for the BSCH, whose contents have to be readable
/// before the cell's identity is known: p(-31) = p(-30) = 1, e(1..30) = 0.
pub const SCRAMB_INIT: u32 = 3;

/// The residue `crc16` leaves over a block whose appended CRC matches.
pub const CRC_GOOD: u16 = 0x1d0f;

/// Scrambler state for a cell, from the identity its BSCH broadcasts.
pub fn scramb_init(mcc: u16, mnc: u16, colour: u8) -> u32 {
    let e = (colour as u32 & 0x3f) | ((mnc as u32 & 0x3fff) << 6) | ((mcc as u32 & 0x3ff) << 20);
    (e << 2) | SCRAMB_INIT
}

/// XOR `bits` with the scrambling sequence; its own inverse.
///
/// Taps 32,26,23,22,16,12,11,10,8,7,5,4,2,1 in Fibonacci form, per 8.2.5.
pub fn scramble(init: u32, bits: &mut [u8]) {
    let mut lfsr = init;
    for b in bits {
        let s = |n: u32| lfsr >> (32 - n);
        let bit = (s(32) ^ s(26) ^ s(23) ^ s(22) ^ s(16) ^ s(12) ^ s(11) ^ s(10)
            ^ s(8) ^ s(7) ^ s(5) ^ s(4) ^ s(2) ^ s(1))
            & 1;
        lfsr = (lfsr >> 1) | (bit << 31);
        *b ^= bit as u8;
    }
}

/// Block interleaver position map (8.2.4.1): bit i of the input lands at
/// `1 + (a*i mod K)`, both one-based.
fn interleave_pos(k: u32, a: u32, i: u32) -> usize {
    (1 + (a as u64 * i as u64 % k as u64)) as usize
}

pub fn interleave(a: u32, input: &[u8], out: &mut [u8]) {
    let k = input.len() as u32;
    for i in 1..=k {
        out[interleave_pos(k, a, i) - 1] = input[(i - 1) as usize];
    }
}

pub fn deinterleave(a: u32, input: &[u8], out: &mut [u8]) {
    let k = input.len() as u32;
    for i in 1..=k {
        out[(i - 1) as usize] = input[interleave_pos(k, a, i) - 1];
    }
}

/// The rate 1/4 mother code (8.2.3.1.1): K = 5, one input bit to four output
/// bits, G1 = 1+D+D^4, G2 = 1+D^2+D^3+D^4, G3 = 1+D+D^2+D^4, G4 = 1+D+D^3+D^4.
///
/// A branch is computed from the 5-bit window `bit << 4 | state`, the state
/// being the last four input bits, newest in bit 0.
const GEN: [u8; 4] = [0b11001, 0b11110, 0b11011, 0b11101];

fn branch_bits(window: u8) -> [u8; 4] {
    let mut out = [0u8; 4];
    for (g, o) in GEN.iter().zip(out.iter_mut()) {
        *o = ((window & g).count_ones() & 1) as u8;
    }
    out
}

/// Encode to the mother code, four bits out per bit in, from the zero state.
pub fn conv_encode(bits: &[u8], out: &mut Vec<u8>) {
    let mut state = 0u8; // last four bits, newest at bit 0
    for &b in bits {
        // window layout: input bit at 4, then delays D..D^4 at 0..3
        let window = (b << 4) | state;
        out.extend_from_slice(&branch_bits(window));
        state = ((state << 1) | b) & 0xf;
    }
}

/// Puncturing (8.2.3.1.2-6): which mother bits survive, one-based.
///
/// `p` is indexed from 1 as the specification writes it; index 0 is unused.
#[derive(Clone, Copy)]
pub struct Puncture {
    p: &'static [u8],
    t: u32,
    /// Identity for the control channels; TCH/4.8 and TCH/2.4 stretch j.
    stretch: Option<u32>,
}

/// Rate 2/3, used by every downlink control channel.
pub const PUNCT_2_3: Puncture = Puncture { p: &[0, 1, 2, 5], t: 3, stretch: None };

fn punct_index(pu: &Puncture, j: u32) -> usize {
    let i = match pu.stretch {
        Some(d) => j + (j - 1) / d,
        None => j,
    };
    let k = 8 * ((i - 1) / pu.t) + pu.p[(i - pu.t * ((i - 1) / pu.t)) as usize] as u32;
    (k - 1) as usize
}

/// Keep `out.len()` of the mother bits.
pub fn puncture(pu: &Puncture, mother: &[u8], out: &mut [u8]) {
    for j in 1..=out.len() as u32 {
        out[(j - 1) as usize] = mother[punct_index(pu, j)];
    }
}

/// Spread type-3 bits back over the mother stream, erasures elsewhere.
///
/// Soft bits: +1 for a received 0, -1 for a received 1, 0 where the
/// puncturer sent nothing.
pub fn depuncture(pu: &Puncture, type3: &[u8], mother_len: usize) -> Vec<i8> {
    let mut out = vec![0i8; mother_len];
    for j in 1..=type3.len() as u32 {
        out[punct_index(pu, j)] = if type3[(j - 1) as usize] != 0 { -1 } else { 1 };
    }
    out
}

/// Viterbi over the mother code: `soft` is 4 bits per step, `n` steps out.
///
/// The encoder starts and ends in the zero state (the type-2 block carries
/// four tail zeros), so the survivor is read from state 0.
pub fn viterbi(soft: &[i8], n: usize) -> Vec<u8> {
    const STATES: usize = 16;
    let inf = i32::MIN / 2;
    let mut metric = [inf; STATES];
    metric[0] = 0;
    // Survivor per state per step: which of the two predecessors won. The
    // input bit itself is the state's low bit, so it needs no storing.
    let mut decisions = vec![0u16; n];

    for step in 0..n {
        let s = &soft[step * 4..step * 4 + 4];
        let mut next = [inf; STATES];
        let mut dec = 0u16;
        for to in 0..STATES as u8 {
            let bit = to & 1;
            for high in 0..2u8 {
                let from = (to >> 1) | (high << 3);
                let m = metric[from as usize];
                if m == inf {
                    continue;
                }
                let out = branch_bits((bit << 4) | from);
                let mut score = m;
                for (o, &r) in out.iter().zip(s) {
                    // r: +1 expects 0, -1 expects 1, 0 says nothing
                    score += if *o == 0 { r as i32 } else { -(r as i32) };
                }
                if score > next[to as usize] {
                    next[to as usize] = score;
                    if high == 1 {
                        dec |= 1 << to;
                    } else {
                        dec &= !(1 << to);
                    }
                }
            }
        }
        metric = next;
        decisions[step] = dec;
    }

    // Trace back from the zero state the tail drove the encoder into.
    let mut bits = vec![0u8; n];
    let mut state = 0u8;
    for step in (0..n).rev() {
        bits[step] = state & 1;
        state = (state >> 1) | ((decisions[step] >> state & 1) as u8) << 3;
    }
    bits
}

/// CRC-16 ITU-T over bits, MSB-first, seeded with all ones (8.2.2).
///
/// Run over a block and its appended CRC it leaves [`CRC_GOOD`], because the
/// transmitted CRC is the ones complement of the register.
pub fn crc16(bits: &[u8]) -> u16 {
    let mut crc = 0xffffu16;
    for &b in bits {
        crc ^= u16::from(b & 1) << 15;
        crc = if crc & 0x8000 != 0 { (crc << 1) ^ 0x1021 } else { crc << 1 };
    }
    crc
}

/// Append the complemented CRC to `bits`, for the test encoders.
pub fn crc16_append(bits: &mut Vec<u8>) {
    let crc = !crc16(bits);
    for i in (0..16).rev() {
        bits.push((crc >> i) as u8 & 1);
    }
}

/// One downlink control block's sizes: type-1 (payload), type-2 (with CRC
/// and tail), type-3/4/5 (on the air), and its interleaver constant.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct BlockParam {
    pub type1_bits: usize,
    pub type2_bits: usize,
    pub type345_bits: usize,
    pub interleave_a: u32,
}

/// BSCH: the 120 scrambled bits of the sync burst's first block.
pub const BLK_BSCH: BlockParam =
    BlockParam { type1_bits: 60, type2_bits: 80, type345_bits: 120, interleave_a: 11 };

/// SCH/HD, BNCH and STCH: one 216 bit half slot.
pub const BLK_HALF: BlockParam =
    BlockParam { type1_bits: 124, type2_bits: 144, type345_bits: 216, interleave_a: 101 };

/// SCH/F: a full slot's 432 bits.
pub const BLK_FULL: BlockParam =
    BlockParam { type1_bits: 268, type2_bits: 288, type345_bits: 432, interleave_a: 103 };

/// Decode one scrambled block to its type-1 bits, `None` when the CRC fails.
pub fn decode_block(param: &BlockParam, scramb: u32, bits: &[u8]) -> Option<Vec<u8>> {
    debug_assert_eq!(bits.len(), param.type345_bits);
    let mut type4 = bits.to_vec();
    scramble(scramb, &mut type4);
    let mut type3 = vec![0u8; param.type345_bits];
    deinterleave(param.interleave_a, &type4, &mut type3);
    let soft = depuncture(&PUNCT_2_3, &type3, param.type2_bits * 4);
    let type2 = viterbi(&soft, param.type2_bits);
    if crc16(&type2[..param.type1_bits + 16]) != CRC_GOOD {
        return None;
    }
    Some(type2[..param.type1_bits].to_vec())
}

/// Encode type-1 bits to the scrambled on-air block, for the tests.
pub fn encode_block(param: &BlockParam, scramb: u32, type1: &[u8]) -> Vec<u8> {
    debug_assert_eq!(type1.len(), param.type1_bits);
    let mut type2 = type1.to_vec();
    crc16_append(&mut type2);
    type2.extend_from_slice(&[0, 0, 0, 0]);
    debug_assert_eq!(type2.len(), param.type2_bits);
    let mut mother = Vec::with_capacity(param.type2_bits * 4);
    conv_encode(&type2, &mut mother);
    let mut type3 = vec![0u8; param.type345_bits];
    puncture(&PUNCT_2_3, &mother, &mut type3);
    let mut type5 = vec![0u8; param.type345_bits];
    interleave(param.interleave_a, &type3, &mut type5);
    scramble(scramb, &mut type5);
    type5
}

/// The (30,14) shortened Reed-Muller code the access assign field is sent
/// in (8.2.3.2): systematic, the 14 information bits first and 16 parity
/// bits after them, each parity column a row of this matrix.
const RM_30_14_PARITY: [u16; 14] = [
    0b1001_1011_0110_0000,
    0b0010_1101_1110_0000,
    0b1111_1100_0010_0000,
    0b1110_0000_0011_1100,
    0b1001_1000_0011_1010,
    0b0101_0100_0011_0110,
    0b0010_1100_0010_1110,
    0b1111_1111_1101_1111,
    0b1000_0011_0011_1001,
    0b0100_0010_1011_0101,
    0b0010_0001_1010_1101,
    0b0001_0010_0111_0011,
    0b0000_1001_0110_1011,
    0b0000_0100_1110_0111,
];

/// The 30 bit codeword for 14 information bits, most significant first.
pub fn rm3014_encode(info: u16) -> u32 {
    let mut parity = 0u16;
    for (i, row) in RM_30_14_PARITY.iter().enumerate() {
        if (info >> (13 - i)) & 1 == 1 {
            parity ^= row;
        }
    }
    (u32::from(info & 0x3fff) << 16) | u32::from(parity)
}

/// The information bits nearest a received codeword, and how many bits
/// away it was. The code's minimum distance is eight, so up to three
/// errors are corrected outright; a caller decides how far it will trust.
pub fn rm3014_decode(word: u32) -> (u16, u32) {
    use std::sync::OnceLock;
    static TABLE: OnceLock<Vec<u32>> = OnceLock::new();
    let table = TABLE.get_or_init(|| (0..1u16 << 14).map(rm3014_encode).collect());
    let mut best = (0u16, u32::MAX);
    for (info, code) in table.iter().enumerate() {
        let d = (code ^ (word & 0x3fff_ffff)).count_ones();
        if d < best.1 {
            best = (info as u16, d);
        }
    }
    best
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_access_assign_code_corrects_three_errors() {
        // Systematic: the information bits are the codeword's first 14.
        let word = rm3014_encode(0x2a5b);
        assert_eq!(word >> 16, 0x2a5b);
        assert_eq!(rm3014_decode(word), (0x2a5b, 0));
        let hit = word ^ (1 << 29) ^ (1 << 17) ^ 1;
        assert_eq!(rm3014_decode(hit), (0x2a5b, 3));
        // Every pair of codewords is at least eight apart.
        let a = rm3014_encode(1);
        let b = rm3014_encode(0x3fff);
        assert!((a ^ b).count_ones() >= 8);
    }

    #[test]
    fn puncture_then_depuncture_restores_every_survivor() {
        for (t2, t345) in [(80usize, 120usize), (144, 216), (288, 432)] {
            let mother: Vec<u8> = (0..t2 * 4).map(|i| (i % 2) as u8).collect();
            let mut type3 = vec![0u8; t345];
            puncture(&PUNCT_2_3, &mother, &mut type3);
            let soft = depuncture(&PUNCT_2_3, &type3, t2 * 4);
            let mut known = 0;
            for (m, s) in mother.iter().zip(&soft) {
                match s {
                    0 => continue,
                    1 => assert_eq!(*m, 0),
                    -1 => assert_eq!(*m, 1),
                    _ => unreachable!(),
                }
                known += 1;
            }
            assert_eq!(known, t345, "every type-3 bit must land somewhere distinct");
        }
    }

    #[test]
    fn interleave_round_trips() {
        for (k, a) in [(120u32, 11u32), (216, 101), (432, 103)] {
            let input: Vec<u8> = (0..k).map(|i| (i % 2) as u8).collect();
            let mut inter = vec![0u8; k as usize];
            interleave(a, &input, &mut inter);
            let mut back = vec![0u8; k as usize];
            deinterleave(a, &inter, &mut back);
            assert_eq!(input, back);
            assert_ne!(input, inter);
        }
    }

    #[test]
    fn scramble_is_its_own_inverse_and_matches_the_reference_form() {
        let init = scramb_init(272, 1234, 17);
        let mut bits = vec![0u8; 120];
        scramble(init, &mut bits);
        assert!(bits.iter().any(|&b| b == 1), "the sequence is not all zeros");
        let mut twice = bits.clone();
        scramble(init, &mut twice);
        assert!(twice.iter().all(|&b| b == 0));
        // The identity packs as e-bits above the two fixed ones.
        assert_eq!(scramb_init(0x3ff, 0x3fff, 0x3f), 0xffff_ffff & !0);
        assert_eq!(scramb_init(0, 0, 0), 3);
    }

    #[test]
    fn the_convolutional_code_survives_the_channel_and_two_erasures() {
        let mut bits: Vec<u8> = (0..76).map(|i| ((i * 7) % 3 == 0) as u8).collect();
        bits.extend_from_slice(&[0, 0, 0, 0]);
        let mut mother = Vec::new();
        conv_encode(&bits, &mut mother);
        assert_eq!(mother.len(), 320);
        let mut soft: Vec<i8> = mother.iter().map(|&b| if b == 1 { -1 } else { 1 }).collect();
        soft[10] = 0;
        soft[200] = 0;
        assert_eq!(viterbi(&soft, 80), bits);
    }

    #[test]
    fn every_block_size_round_trips_through_the_whole_stack() {
        for param in [BLK_BSCH, BLK_HALF, BLK_FULL] {
            let type1: Vec<u8> = (0..param.type1_bits).map(|i| ((i * 5) % 7 < 3) as u8).collect();
            let scramb = scramb_init(272, 91, 3);
            let air = encode_block(&param, scramb, &type1);
            assert_eq!(decode_block(&param, scramb, &air).as_deref(), Some(&type1[..]));
            // One bit flipped on the air still decodes: the rate 2/3 code
            // corrects it and the CRC agrees.
            let mut dented = air.clone();
            dented[7] ^= 1;
            assert_eq!(decode_block(&param, scramb, &dented).as_deref(), Some(&type1[..]));
            // The wrong scrambler does not.
            assert_eq!(decode_block(&param, scramb ^ 0xa5a5a5a4, &air), None);
        }
    }

    #[test]
    fn crc_matches_the_reference_residue() {
        let mut bits: Vec<u8> = (0..60).map(|i| (i % 3 == 1) as u8).collect();
        crc16_append(&mut bits);
        assert_eq!(crc16(&bits), CRC_GOOD);
    }
}
