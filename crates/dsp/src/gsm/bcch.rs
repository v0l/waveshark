//! The block a control channel carries, spread over four normal bursts.
//!
//! Everything a GSM base station says on its broadcast and common control
//! channels travels this way: 23 bytes, a 40 bit Fire code over them, the
//! same rate 1/2 convolutional code the synchronisation channel uses, and
//! then 456 coded bits dealt out over four consecutive bursts so that losing
//! one burst to a fade costs a quarter of every codeword rather than all of
//! one. GSM 05.03 calls it the xCCH coding and uses it for BCCH, the paging
//! and access grant channels, and the slow associated channel alike.
//!
//! Nothing here is ciphered. A5 applies to the traffic and to the dedicated
//! channels; what a cell broadcasts about itself has to be readable by a
//! phone that has not registered yet, so it is readable by anything.
//!
//! # The Fire code is what makes a block a block
//!
//! Forty bits of check over 184, and a real one: it is the only thing
//! separating a system information message from what a Viterbi decoder makes
//! of four bursts of noise. It can correct a burst of errors as well as
//! detect one, which nothing here does; used only as a check it still leaves
//! about one in a million-million wrong blocks accepted.

use super::coding::{self, crc, viterbi};

/// Bytes in a block, which is what the layer above reads.
pub const BLOCK_BYTES: usize = 23;

/// Those bytes as bits.
pub const DATA_BITS: usize = BLOCK_BYTES * 8;

/// The Fire code's width.
pub const PARITY_BITS: usize = 40;

/// Coded bits leaving the convolutional encoder.
pub const CODED_BITS: usize = 456;

/// Data bits one normal burst carries, either side of its training sequence
/// and not counting the two stealing flags.
pub const BURST_BITS: usize = 114;

/// Bursts a block is spread over.
pub const BURSTS: usize = 4;

/// The Fire code generator, (D^23 + 1)(D^17 + D^3 + 1), without its leading
/// term: D^26 + D^23 + D^17 + D^3 + 1.
const FIRE_POLY: u64 = 0x0000_0482_0009;

/// The check is inverted before transmission, so a run of zeros does not
/// carry a valid one.
const FIRE_INVERT: u64 = 0xFF_FFFF_FFFF;

/// Where a coded bit lands: which of the four bursts, and where in it.
///
/// GSM 05.03 section 4.1.4. The multiply by 49 modulo 57 is what spreads
/// neighbouring coded bits apart inside a burst, and the `k mod 4` is what
/// spreads them across bursts; together they mean no two bits the
/// convolutional code depends on most arrive in the same fade.
fn place(k: usize) -> (usize, usize) {
    (k % 4, 2 * ((49 * k) % 57) + (k % 8) / 4)
}

/// Encode a block into the data bits of four bursts.
///
/// The inverse of [`decode`], and here for the same reason the other
/// encoders in this tree are: a decoder with no recording to answer to can
/// only be tested against something known that was transmitted.
pub fn encode(bytes: &[u8]) -> Option<[[u8; BURST_BITS]; BURSTS]> {
    if bytes.len() != BLOCK_BYTES {
        return None;
    }
    let mut d = [0u8; DATA_BITS + PARITY_BITS + 4];
    for i in 0..DATA_BITS {
        d[i] = bytes[i / 8] >> (7 - i % 8) & 1;
    }
    let p = crc(&d[..DATA_BITS], FIRE_POLY, PARITY_BITS as u32) ^ FIRE_INVERT;
    for i in 0..PARITY_BITS {
        d[DATA_BITS + i] = (p >> (PARITY_BITS - 1 - i) & 1) as u8;
    }

    let mut coded = [0u8; CODED_BITS];
    coding::conv_encode(&d, &mut coded);
    let mut out = [[0u8; BURST_BITS]; BURSTS];
    for (k, &c) in coded.iter().enumerate() {
        let (b, j) = place(k);
        out[b][j] = c;
    }
    Some(out)
}

/// Recover a block from the soft bits of four consecutive bursts.
///
/// `None` means the Fire code did not hold, which is the only evidence that
/// a block was received rather than invented.
pub fn decode(bursts: &[[f32; BURST_BITS]; BURSTS]) -> Option<[u8; BLOCK_BYTES]> {
    let mut soft = [0.0f32; CODED_BITS];
    for (k, s) in soft.iter_mut().enumerate() {
        let (b, j) = place(k);
        *s = bursts[b][j];
    }
    let bits = viterbi(&soft, DATA_BITS + PARITY_BITS + 4);

    let want = crc(&bits[..DATA_BITS], FIRE_POLY, PARITY_BITS as u32) ^ FIRE_INVERT;
    if coding::bits_to_u64(&bits[DATA_BITS..DATA_BITS + PARITY_BITS]) != want {
        return None;
    }

    let mut out = [0u8; BLOCK_BYTES];
    for i in 0..DATA_BITS {
        out[i / 8] |= bits[i] << (7 - i % 8);
    }
    Some(out)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::gsm::coding::soften;

    /// A system information type 3, as a cell broadcasts it: the pseudo
    /// length, the radio resource protocol discriminator, the message type,
    /// then the cell identity and the location area it belongs to.
    fn si3() -> [u8; BLOCK_BYTES] {
        let mut b = [0x2Bu8; BLOCK_BYTES];
        b[..12].copy_from_slice(&[
            0x49, 0x06, 0x1B, 0x12, 0x34, 0x27, 0xF4, 0x50, 0x11, 0x22, 0x33, 0x44,
        ]);
        b
    }

    fn softened(bursts: &[[u8; BURST_BITS]; BURSTS]) -> [[f32; BURST_BITS]; BURSTS] {
        let mut out = [[0.0f32; BURST_BITS]; BURSTS];
        for (o, b) in out.iter_mut().zip(bursts) {
            o.copy_from_slice(&soften(b));
        }
        out
    }

    /// Every coded bit lands somewhere, and no two land in the same place.
    ///
    /// The interleaver is a formula with two modulos in it and no check of
    /// its own: get it wrong and the decoder still produces 184 bits, which
    /// the Fire code then rejects, and the fault looks like a demodulator
    /// that cannot hear anything.
    #[test]
    fn the_interleaver_fills_every_position_exactly_once() {
        let mut seen = [[false; BURST_BITS]; BURSTS];
        for k in 0..CODED_BITS {
            let (b, j) = place(k);
            assert!(j < BURST_BITS, "bit {k} lands at {j}");
            assert!(!seen[b][j], "bit {k} lands on burst {b} position {j} twice");
            seen[b][j] = true;
        }
        assert!(seen.iter().flatten().all(|&s| s), "a position nothing lands on");
    }

    #[test]
    fn a_block_survives_the_round_trip() {
        let want = si3();
        let bursts = encode(&want).expect("23 bytes");
        assert_eq!(decode(&softened(&bursts)), Some(want));
    }

    /// A whole burst lost, which is what a fade in one of the four frames
    /// looks like: the interleaving is there so this still decodes.
    #[test]
    fn one_burst_erased_is_still_a_block() {
        let want = si3();
        let bursts = encode(&want).unwrap();
        let mut soft = softened(&bursts);
        soft[2] = [0.0; BURST_BITS];
        assert_eq!(decode(&soft), Some(want), "a quarter of the block is erasures");
    }

    /// Two bursts gone is past what a rate 1/2 code carries, and the Fire
    /// code refuses rather than reporting a cell that was never on the air.
    #[test]
    fn half_the_block_lost_is_refused() {
        let want = si3();
        let bursts = encode(&want).unwrap();
        let mut soft = softened(&bursts);
        soft[0] = [0.0; BURST_BITS];
        soft[1] = [0.0; BURST_BITS];
        soft[3] = [0.0; BURST_BITS];
        assert_eq!(decode(&soft), None);
    }

    #[test]
    fn noise_does_not_become_a_block() {
        let mut state = 0x7F4A_7C15u32;
        let mut rand = move || {
            state ^= state << 13;
            state ^= state >> 17;
            state ^= state << 5;
            state
        };
        for _ in 0..200 {
            let mut soft = [[0.0f32; BURST_BITS]; BURSTS];
            for b in soft.iter_mut() {
                for s in b.iter_mut() {
                    *s = if rand() & 1 == 1 { 1.0 } else { -1.0 };
                }
            }
            assert_eq!(decode(&soft), None, "noise decoded as a block");
        }
    }

    #[test]
    fn a_short_block_is_not_encodable() {
        assert!(encode(&[0u8; 22]).is_none());
    }
}
