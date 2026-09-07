//! GSM's synchronisation channel: the one thing a base station says in the
//! clear that identifies it and says what time it is.
//!
//! A GSM cell repeats two bursts on its beacon carrier that carry no user
//! data and are never ciphered. The frequency correction burst is 148 zero
//! bits, which GMSK turns into an unmodulated tone a quarter of the symbol
//! rate above the carrier, and its only content is that it exists: it hands a
//! receiver the frequency error and the frame boundary. One TDMA frame later
//! the synchronisation burst carries 25 bits saying which cell this is
//! (the base station identity code) and which frame this is, and those 25
//! bits are what this module recovers.
//!
//! Everything above that is ciphered or is a stack, so this is the natural
//! stopping point: after the SCH a receiver knows the cell, the colour code
//! its training sequences use, and the frame number, which is exactly what is
//! needed to say "this carrier is a live cell, here is its identity" without
//! attacking anything.
//!
//! # The coding, from GSM 05.03 section 4.7
//!
//! 25 information bits, 10 parity bits, 4 tail bits, then a rate 1/2
//! constraint length 5 convolutional code: 39 in, 78 out. Those 78 are split
//! down the middle and sit either side of the burst's 64 bit training
//! sequence, which is why the demodulator hands this module two halves.
//!
//! The parity is a real check and not a plausibility argument, which matters
//! more here than usual. An SCH burst is 78 coded bits with no repetition and
//! no outer code; without the parity holding there is nothing separating a
//! cell identity from a Viterbi decoder's best guess at noise.

use super::coding::{self, crc, viterbi};

/// Information bits in an SCH burst: BSIC and the reduced frame number.
pub const INFO_BITS: usize = 25;

/// Parity bits over those 25.
pub const PARITY_BITS: usize = 10;

/// Coded bits leaving the convolutional encoder, and what a burst carries.
pub const CODED_BITS: usize = 78;

/// The parity polynomial, D^10 + D^8 + D^6 + D^5 + D^4 + D^2 + 1, without its
/// leading term.
const PARITY_POLY: u16 = 0x175;

/// The parity is inverted before transmission, so a run of zeros does not
/// carry a valid check: GSM 05.03 asks for a remainder of every bit set
/// rather than of zero.
const PARITY_INVERT: u16 = 0x3FF;

/// The parity over the information bits, before the inversion.
fn parity(bits: &[u8]) -> u16 {
    crc(bits, u64::from(PARITY_POLY), PARITY_BITS as u32) as u16
}

/// What a decoded synchronisation burst says.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Sch {
    /// Network colour code, 3 bits: which operator, where two overlap.
    pub ncc: u8,
    /// Base station colour code, 3 bits: which of the neighbours this is, and
    /// the training sequence its other bursts use.
    pub bcc: u8,
    /// The full TDMA frame number, reassembled from the three fields the
    /// burst carries.
    pub frame_number: u32,
}

impl Sch {
    /// The base station identity code as it is written and configured: the
    /// network colour code in the high three bits.
    pub fn bsic(&self) -> u8 {
        self.ncc << 3 | self.bcc
    }

    /// Position in the 51 frame control multiframe, which is what says
    /// whether a given burst is FCCH, SCH, BCCH or a paging block.
    pub fn t3(&self) -> u32 {
        self.frame_number % 51
    }

    /// Position in the 26 frame traffic multiframe.
    pub fn t2(&self) -> u32 {
        self.frame_number % 26
    }
}

/// Build the 78 coded bits a burst carries for a given cell and frame number.
///
/// The inverse of [`decode`], and here for the reason every decoder in this
/// tree has an encoder: without a recording, the only honest test of a
/// decoder is to transmit something known and see whether it comes back.
pub fn encode(sch: &Sch) -> Option<[u8; CODED_BITS]> {
    let mut d = [0u8; INFO_BITS + PARITY_BITS + 4];
    d[..INFO_BITS].copy_from_slice(&info_bits(sch)?);

    let p = parity(&d[..INFO_BITS]) ^ PARITY_INVERT;
    for i in 0..PARITY_BITS {
        d[INFO_BITS + i] = (p >> (PARITY_BITS - 1 - i) & 1) as u8;
    }

    let mut out = [0u8; CODED_BITS];
    coding::conv_encode(&d, &mut out);
    Some(out)
}

/// Recover a cell identity and frame number from 78 soft bits.
///
/// Soft bits are positive for a one and larger in magnitude the more the
/// demodulator believes it, the same convention the rest of this tree uses.
/// `None` means the parity did not hold, which is the only evidence there is
/// that a burst was read rather than invented.
pub fn decode(soft: &[f32]) -> Option<Sch> {
    if soft.len() < CODED_BITS {
        return None;
    }
    let bits = viterbi(&soft[..CODED_BITS], INFO_BITS + PARITY_BITS + 4);

    // The four tail bits flushed the register, so anything after the parity
    // is the encoder emptying itself and carries nothing.
    let want = parity(&bits[..INFO_BITS]) ^ PARITY_INVERT;
    let got = coding::bits_to_u64(&bits[INFO_BITS..INFO_BITS + PARITY_BITS]) as u16;
    if got != want {
        return None;
    }

    from_info(&bits[..INFO_BITS])
}

/// Where each field's bits sit in the 25, most significant first.
///
/// Not in order, and this is the thing about the synchronisation channel that
/// nothing local can catch. GSM writes layer 3 fields into octets least
/// significant bit first, so a field spanning an octet boundary comes out of
/// the bit stream in pieces and in the other order: the network colour code
/// is the eighth, seventh and sixth bits transmitted, and the frame number's
/// most significant part is split across three places.
///
/// Read as a plain sequence of fields, which is what this file did at first,
/// every burst decodes, the parity holds, the colour codes come out stable
/// because the bits behind them are stable, and the frame number is nonsense.
/// It took a recording of a live cell to see it: the frame numbers of
/// consecutive bursts disagreed with the time between them, and with this
/// order they agree exactly, on all 337 bursts in the capture.
/// The layout is 3GPP TS 44.018 section 9.1.30, and matches `gr-gsm`'s
/// `decode_sch`.
const NCC_BITS: [usize; 3] = [7, 6, 5];
const BCC_BITS: [usize; 3] = [4, 3, 2];
const T1_BITS: [usize; 11] = [1, 0, 15, 14, 13, 12, 11, 10, 9, 8, 23];
const T2_BITS: [usize; 5] = [22, 21, 20, 19, 18];
const T3P_BITS: [usize; 3] = [17, 16, 24];

/// The 25 information bits, in the order they go on the air.
fn info_bits(sch: &Sch) -> Option<[u8; INFO_BITS]> {
    let t1 = sch.frame_number / 1326;
    let t2 = sch.frame_number % 26;
    let t3 = sch.frame_number % 51;
    if t3 % 10 != 1 || t1 > 0x7FF || sch.ncc > 7 || sch.bcc > 7 {
        return None;
    }
    let mut d = [0u8; INFO_BITS];
    let mut put = |where_: &[usize], value: u32| {
        let width = where_.len();
        for (i, &at) in where_.iter().enumerate() {
            d[at] = (value >> (width - 1 - i) & 1) as u8;
        }
    };
    put(&NCC_BITS, u32::from(sch.ncc));
    put(&BCC_BITS, u32::from(sch.bcc));
    put(&T1_BITS, t1);
    put(&T2_BITS, t2);
    put(&T3P_BITS, t3 / 10);
    Some(d)
}

/// The reverse: the fields, and the frame number they add up to.
fn from_info(bits: &[u8]) -> Option<Sch> {
    let take = |where_: &[usize]| -> u32 {
        where_.iter().fold(0u32, |v, &at| v << 1 | u32::from(bits[at]))
    };
    let (ncc, bcc) = (take(&NCC_BITS) as u8, take(&BCC_BITS) as u8);
    let (t1, t2, t3p) = (take(&T1_BITS), take(&T2_BITS), take(&T3P_BITS));
    // T2 counts a 26 frame multiframe in five bits, and T3' counts the five
    // control multiframe positions an SCH can occupy in three, so both fields
    // can hold values no transmitter sends. The parity has already held here,
    // which makes these a guard against a carrier that is not GSM rather than
    // against noise.
    if t3p > 4 || t2 > 25 {
        return None;
    }
    let t3 = t3p * 10 + 1;
    let frame_number = 51 * ((t3 + 26 - t2) % 26) + t3 + 51 * 26 * t1;
    Some(Sch { ncc, bcc, frame_number })
}

/// The information field as bytes, which is what travels on the packet bus.
///
/// Four bytes holding the 25 bits the burst carried, left aligned and in the
/// order they were transmitted. A row in the packet list is evidence of what
/// arrived, so what it carries is the field rather than this crate's reading
/// of it, and [`unpack`] is how a consumer gets that reading back.
pub fn pack(sch: &Sch) -> Option<[u8; 4]> {
    let bits = info_bits(sch)?;
    let mut out = [0u8; 4];
    for (i, &b) in bits.iter().enumerate() {
        out[i / 8] |= b << (7 - i % 8);
    }
    Some(out)
}

/// Read back what [`pack`] wrote.
pub fn unpack(bytes: &[u8]) -> Option<Sch> {
    if bytes.len() < 4 {
        return None;
    }
    let bits: Vec<u8> = (0..INFO_BITS).map(|i| bytes[i / 8] >> (7 - i % 8) & 1).collect();
    from_info(&bits)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::gsm::coding::soften;

    fn sch(ncc: u8, bcc: u8, frame_number: u32) -> Sch {
        Sch { ncc, bcc, frame_number }
    }

    /// Frame numbers an SCH can be sent at: T3 = 1, 11, 21, 31 or 41.
    fn sch_frames() -> impl Iterator<Item = u32> {
        (0u32..2_715_648).filter(|fnum| fnum % 51 % 10 == 1 && fnum % 51 < 42)
    }

    #[test]
    fn a_burst_survives_the_round_trip() {
        let want = sch(3, 5, 51 * 26 * 7 + 11);
        let coded = encode(&want).expect("a legal frame number");
        assert_eq!(decode(&soften(&coded)), Some(want));
    }

    /// Every frame number a transmitter can put in an SCH, over a full
    /// hyperframe, comes back as itself. The reduced frame number is three
    /// fields modulo three different periods, and an off-by-one in the
    /// reassembly shows up only at particular values.
    #[test]
    fn every_legal_frame_number_reassembles() {
        for fnum in sch_frames().step_by(37) {
            let want = sch(1, 2, fnum);
            let coded = encode(&want).expect("legal");
            assert_eq!(decode(&soften(&coded)), Some(want), "frame {fnum}");
        }
    }

    #[test]
    fn the_identity_code_reads_as_it_is_configured() {
        let s = sch(6, 2, 1);
        assert_eq!(s.bsic(), 0x32);
        let coded = encode(&s).unwrap();
        let got = decode(&soften(&coded)).unwrap();
        assert_eq!((got.ncc, got.bcc, got.bsic()), (6, 2, 0x32));
    }

    /// Noise is refused rather than turned into a cell.
    ///
    /// The Viterbi decoder always produces 39 bits, so without the parity
    /// every stretch of noise is a base station somewhere. Ten bits of parity
    /// let through one in a thousand, and this checks the order of magnitude
    /// rather than the exact count.
    #[test]
    fn noise_does_not_become_a_cell() {
        let mut state = 0x1234_5678u32;
        let mut rand = move || {
            state ^= state << 13;
            state ^= state >> 17;
            state ^= state << 5;
            state
        };
        let mut accepted = 0;
        for _ in 0..2000 {
            let soft: Vec<f32> =
                (0..CODED_BITS).map(|_| if rand() & 1 == 1 { 1.0 } else { -1.0 }).collect();
            accepted += usize::from(decode(&soft).is_some());
        }
        assert!(accepted < 20, "{accepted} of 2000 noise bursts accepted");
    }

    /// Bit errors up to what the code can carry are corrected, and past that
    /// the parity refuses rather than lying.
    #[test]
    fn errors_are_corrected_and_then_refused() {
        let want = sch(4, 1, 51 * 26 * 100 + 21);
        let coded = encode(&want).unwrap();
        for flips in 0..=3 {
            let mut soft = soften(&coded);
            for i in 0..flips {
                soft[i * 7] = -soft[i * 7];
            }
            assert_eq!(decode(&soft), Some(want), "{flips} flipped bits");
        }
        // Half the burst destroyed: whatever comes out, it must not claim to
        // be a cell.
        let mut soft = soften(&coded);
        for (i, v) in soft.iter_mut().enumerate() {
            if i % 2 == 0 {
                *v = -*v;
            }
        }
        assert_eq!(decode(&soft), None, "a burst read backwards is not a cell");
    }

    /// A soft bit of zero says the demodulator had no opinion, which is what
    /// a punctured or erased bit looks like. Erasures cost less than errors:
    /// the same count of them is still decodable where flips are not.
    #[test]
    fn erasures_are_cheaper_than_errors() {
        let want = sch(2, 7, 41);
        let coded = encode(&want).unwrap();
        let mut soft = soften(&coded);
        for i in 0..8 {
            soft[i * 9] = 0.0;
        }
        assert_eq!(decode(&soft), Some(want));
    }

    /// The 25 bits as they go on the air, for a cell and a frame number, so
    /// that the field layout cannot be tidied into order by somebody who
    /// reasonably assumes it is in order.
    ///
    /// Not from this file's own encoder, which is the point: a round trip
    /// through a wrong layout passes, and did, for as long as nothing but
    /// this crate had an opinion. These positions are `gr-gsm`'s
    /// `decode_sch`, and the recording that settled it had 337 bursts whose
    /// frame numbers agreed with the time between them only this way round.
    #[test]
    fn the_fields_sit_where_gsm_puts_them() {
        let sch = sch(2, 7, 51 * 26 * 1180 + 21);
        let bits = info_bits(&sch).expect("a legal burst");
        let got: String = bits.iter().map(|b| char::from(b'0' + b)).collect();
        // Worked out by hand from those positions: colour code 2 as 010 in
        // bits 7, 6 and 5; colour code 7 as 111 in bits 4, 3 and 2; T1 =
        // 1180 as 10010011100 across bits 1, 0, 15 down to 8, and 23; T2 =
        // 21 as 10101 in bits 22 down to 18; T3' = 2 as 010 in bits 17, 16
        // and 24.
        assert_eq!(got, "0111101001110010101010100");
    }

    /// What the bus carries is the field, and reading it back gives the same
    /// cell: the packing is a container and not a second decoder.
    #[test]
    fn the_packed_field_reads_back() {
        for fnum in sch_frames().step_by(1009) {
            let want = sch(7, 0, fnum);
            assert_eq!(unpack(&pack(&want).unwrap()), Some(want), "frame {fnum}");
        }
        assert_eq!(unpack(&[0u8; 3]), None, "four bytes are needed");
    }

    #[test]
    fn an_illegal_frame_number_is_not_encodable() {
        // T3 = 2 is not a synchronisation burst position.
        assert!(encode(&sch(0, 0, 2)).is_none());
        assert!(encode(&sch(8, 0, 1)).is_none(), "the colour code is three bits");
    }
}
