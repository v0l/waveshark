//! Bluetooth LE Coded PHY, the long range one Bluetooth 5 added.
//!
//! The modulation is unchanged: one megasymbol a second of GFSK, the same
//! deviation, the same channels, so the front end in [`crate::ble`] reaches
//! the symbols and everything here works on them. What changed is what the
//! symbols mean. A rate 1/2 convolutional code with constraint length 4 sits
//! under the packet, and at S=8 a pattern mapper repeats each coded bit four
//! times with its inverse, so eight symbols carry one bit of PDU and the data
//! rate falls to 125 kbit/s. That is where the range comes from: about 6 dB
//! of coding gain, which is a doubling of distance.
//!
//! Layout from the Core specification, Vol 6 Part B, sections 2.2 and 3.3:
//!
//! ```text
//!   preamble    80 symbols, uncoded, ten repeats of 00111100
//!   FEC block 1 access address (32), coding indicator (2), TERM1 (3), always S=8
//!   FEC block 2 PDU, CRC-24, TERM2 (3), at S=8 or S=2 as the indicator says
//! ```
//!
//! Nothing about a packet says how long it is until the header inside it is
//! decoded, and the header is inside the coded block, so the decoder reads
//! the whole buffer it is given and lets the CRC decide where the packet
//! ended.
//!
//! # Why this is here rather than in the uncoded reader
//!
//! Open Drone ID. A drone broadcasting under ASTM F3411 must transmit on
//! Bluetooth 5 Long Range as well as Bluetooth 4, and the long range copy
//! carries a message pack: every message type in one packet instead of one
//! message per advertisement, which is a whole aircraft in a single reception
//! rather than five. The regulators require both, so a receiver that reads
//! only the legacy advertisement is not missing information today; it is
//! missing the transport that will still be there if the legacy one is ever
//! dropped, and it is missing every packet whose legacy copy was lost to a
//! collision.

use crate::ble::{crc24, Whitening, ADV_ACCESS_ADDRESS};

/// Ten repeats of the pattern, uncoded, at one symbol a microsecond.
pub const PREAMBLE: [bool; 8] = [false, false, true, true, true, true, false, false];

/// Coding schemes the indicator selects, as symbols per coded bit.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Coding {
    /// 125 kbit/s: the convolutional code and the pattern mapper.
    S8,
    /// 500 kbit/s: the convolutional code alone.
    S2,
}

impl Coding {
    fn symbols_per_coded_bit(self) -> usize {
        match self {
            Self::S8 => 4,
            Self::S2 => 1,
        }
    }

    pub fn bitrate(self) -> u32 {
        match self {
            Self::S8 => 125_000,
            Self::S2 => 500_000,
        }
    }
}

/// A decoded packet.
#[derive(Clone, Debug, PartialEq)]
pub struct CodedFrame {
    pub access_address: u32,
    pub coding: Coding,
    /// Header and payload, dewhitened, without the CRC.
    pub pdu: Vec<u8>,
    /// Where the preamble started in the symbols it was read from.
    pub start: usize,
}

/// The convolutional code: G0 = 1 + x + x^2 + x^3 and G1 = 1 + x^2 + x^3,
/// with G0's output transmitted first.
const G0: u8 = 0b1111;
const G1: u8 = 0b1101;

fn parity(v: u8) -> bool {
    v.count_ones() % 2 == 1
}

/// Encode bits the way a transmitter does. Used by the tests, and it is what
/// a transmitter here would call.
pub fn encode_block(bits: &[bool], coding: Coding) -> Vec<bool> {
    let mut state = 0u8;
    let mut out = Vec::with_capacity(bits.len() * 2 * coding.symbols_per_coded_bit());
    for &b in bits {
        // The register holds the current bit and the three before it.
        state = ((state << 1) | u8::from(b)) & 0x0f;
        for c in [parity(state & G0), parity(state & G1)] {
            match coding {
                // 0 becomes 0011 and 1 becomes 1100.
                Coding::S8 => out.extend_from_slice(&[c, c, !c, !c]),
                Coding::S2 => out.push(c),
            }
        }
    }
    out
}

/// Soft values for the coded bits carried by these symbols: positive for a
/// one. At S=8 the four symbols of a pattern are combined, which is where the
/// coding gain of the mapper comes from.
fn despread(symbols: &[f32], coding: Coding) -> Vec<f32> {
    match coding {
        Coding::S2 => symbols.to_vec(),
        Coding::S8 => symbols
            .chunks_exact(4)
            .map(|c| (c[0] + c[1] - c[2] - c[3]) / 4.0)
            .collect(),
    }
}

/// Viterbi over the eight states of a constraint length 4 code, with soft
/// inputs. The encoder starts and ends in the all-zero state, which is what
/// the three zero bits of a terminator are for, so the survivor is taken from
/// state zero when the block was terminated.
fn viterbi(soft: &[f32], want_bits: usize) -> Vec<bool> {
    const STATES: usize = 8;
    let steps = (soft.len() / 2).min(want_bits);
    let mut metric = [f32::NEG_INFINITY; STATES];
    metric[0] = 0.0;
    let mut back = vec![[0u8; STATES]; steps];

    for (t, chunk) in soft.chunks_exact(2).take(steps).enumerate() {
        let (r0, r1) = (chunk[0], chunk[1]);
        let mut next = [f32::NEG_INFINITY; STATES];
        for (state, &m) in metric.iter().enumerate() {
            if m == f32::NEG_INFINITY {
                continue;
            }
            for b in [false, true] {
                // The same register the encoder keeps: three bits of history
                // plus the bit going in.
                let full = ((state as u8) << 1 | u8::from(b)) & 0x0f;
                let c0 = parity(full & G0);
                let c1 = parity(full & G1);
                let sign = |c: bool| if c { 1.0 } else { -1.0 };
                let cand = m + r0 * sign(c0) + r1 * sign(c1);
                let to = usize::from(full & 0x07);
                if cand > next[to] {
                    next[to] = cand;
                    back[t][to] = (state as u8) << 1 | u8::from(b);
                }
            }
        }
        metric = next;
    }

    // The end state is unknown mid-packet, so the best survivor is taken and
    // traced back. A terminated block ends in state zero and that is where
    // the best metric lands anyway.
    let mut state = metric
        .iter()
        .enumerate()
        .max_by(|a, b| a.1.partial_cmp(b.1).unwrap_or(std::cmp::Ordering::Equal))
        .map(|(i, _)| i)
        .unwrap_or(0);
    let mut bits = vec![false; steps];
    for t in (0..steps).rev() {
        let entry = back[t][state];
        bits[t] = entry & 1 != 0;
        state = usize::from(entry >> 1) & 0x07;
    }
    bits
}

fn bits_to_bytes_lsb_first(bits: &[bool]) -> Vec<u8> {
    bits.chunks(8)
        .filter(|c| c.len() == 8)
        .map(|c| c.iter().enumerate().fold(0u8, |a, (i, &b)| a | (u8::from(b) << i)))
        .collect()
}

/// Where a coded preamble starts in these symbols, searched from `from`.
///
/// The preamble is not coded and not whitened, so it is the one part of the
/// packet that can be correlated for directly.
pub fn find_preamble(symbols: &[f32], from: usize) -> Option<usize> {
    let want: Vec<f32> = (0..80)
        .map(|i| if PREAMBLE[i % 8] { 1.0 } else { -1.0 })
        .collect();
    let mut best = (0usize, 0.0f32);
    for at in from..symbols.len().saturating_sub(want.len()) {
        let score: f32 = want
            .iter()
            .zip(&symbols[at..])
            .map(|(w, s)| w * s.signum())
            .sum();
        if score > best.1 {
            best = (at, score);
        }
    }
    // Sixty of the eighty symbols agreeing is a preamble; the pattern is
    // periodic, so a run of alternating noise scores about zero and a real
    // one that started inside the burst still scores most of the length.
    (best.1 >= 60.0).then_some(best.0)
}

/// Read one packet out of soft symbols, positive for a one, at one symbol a
/// microsecond.
///
/// `channel` is the advertising channel index, which seeds the whitening.
pub fn decode(symbols: &[f32], channel: u8) -> Option<CodedFrame> {
    let start = find_preamble(symbols, 0)?;
    let after = start + 80;

    // Block 1 is always S=8: 37 bits of access address, indicator and
    // terminator, so 37 * 2 * 4 symbols.
    let block1_bits = 32 + 2 + 3;
    let need = block1_bits * 2 * 4;
    if after + need > symbols.len() {
        return None;
    }
    let soft1 = despread(&symbols[after..after + need], Coding::S8);
    let bits1 = viterbi(&soft1, block1_bits);
    let aa_bytes = bits_to_bytes_lsb_first(&bits1[..32]);
    let access_address = u32::from_le_bytes([aa_bytes[0], aa_bytes[1], aa_bytes[2], aa_bytes[3]]);
    if access_address != ADV_ACCESS_ADDRESS {
        return None;
    }
    let coding = match (bits1[32], bits1[33]) {
        (false, false) => Coding::S8,
        (true, false) => Coding::S2,
        // The indicator is sent least significant bit first and the other two
        // values are reserved.
        _ => return None,
    };

    // Block 2 has no length until its own header is decoded, so the rest of
    // the buffer is decoded and the CRC decides where the packet ended.
    let rest = &symbols[after + need..];
    let soft2 = despread(rest, coding);
    let max_bits = (2 + 255 + 3 + 3) * 8;
    let bits2 = viterbi(&soft2, max_bits.min(soft2.len() / 2));
    let bytes = bits_to_bytes_lsb_first(&bits2);
    if bytes.len() < 5 {
        return None;
    }
    let mut whitened = bytes.clone();
    Whitening::new(channel).apply(&mut whitened);
    let len = usize::from(whitened[1]) + 2;
    if whitened.len() < len + 3 {
        return None;
    }
    let sent = u32::from(whitened[len])
        | u32::from(whitened[len + 1]) << 8
        | u32::from(whitened[len + 2]) << 16;
    if crc24(&whitened[..len]) != sent {
        return None;
    }
    Some(CodedFrame {
        access_address,
        coding,
        pdu: whitened[..len].to_vec(),
        start,
    })
}

/// Build a packet the way a transmitter does: preamble, block 1 at S=8, then
/// the whitened PDU and CRC at the coding the indicator names.
pub fn encode_packet(channel: u8, pdu: &[u8], coding: Coding) -> Vec<bool> {
    let mut out: Vec<bool> = (0..80).map(|i| PREAMBLE[i % 8]).collect();

    let mut block1: Vec<bool> = Vec::new();
    for i in 0..32 {
        block1.push(ADV_ACCESS_ADDRESS >> i & 1 != 0);
    }
    block1.extend_from_slice(&match coding {
        Coding::S8 => [false, false],
        Coding::S2 => [true, false],
    });
    block1.extend_from_slice(&[false; 3]);
    out.extend(encode_block(&block1, Coding::S8));

    let mut body = pdu.to_vec();
    let crc = crc24(&body);
    body.extend_from_slice(&[crc as u8, (crc >> 8) as u8, (crc >> 16) as u8]);
    Whitening::new(channel).apply(&mut body);
    let mut block2: Vec<bool> = Vec::new();
    for b in &body {
        for i in 0..8 {
            block2.push(b >> i & 1 != 0);
        }
    }
    block2.extend_from_slice(&[false; 3]);
    out.extend(encode_block(&block2, coding));
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    fn soft(bits: &[bool]) -> Vec<f32> {
        bits.iter().map(|&b| if b { 1.0 } else { -1.0 }).collect()
    }

    /// An ADV_EXT_IND carrying an auxiliary pointer, which is what a device
    /// advertising on the coded PHY sends on a primary channel.
    fn adv_ext_ind() -> Vec<u8> {
        let mut pdu = vec![0x07, 0x00];
        pdu.extend_from_slice(&[0x0a, 0x01, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66]);
        pdu[1] = (pdu.len() - 2) as u8;
        pdu
    }

    #[test]
    fn a_packet_survives_the_round_trip_at_both_codings() {
        for coding in [Coding::S8, Coding::S2] {
            let pdu = adv_ext_ind();
            let sym = soft(&encode_packet(37, &pdu, coding));
            let got = decode(&sym, 37).unwrap_or_else(|| panic!("{coding:?}: no packet"));
            assert_eq!(got.coding, coding);
            assert_eq!(got.pdu, pdu, "{coding:?}: the PDU came back changed");
            assert_eq!(got.access_address, ADV_ACCESS_ADDRESS);
        }
    }

    /// The coding gain is the point of the PHY, so it has to survive symbols
    /// that are wrong. At S=8 eight symbols carry one bit, and the code
    /// corrects on top of that.
    #[test]
    fn symbol_errors_are_corrected_at_s8() {
        let pdu = adv_ext_ind();
        let mut sym = soft(&encode_packet(37, &pdu, Coding::S8));
        // Every thirteenth symbol after the preamble inverted, which is 7.7%
        // of them and far past what the uncoded PHY tolerates.
        for i in (80..sym.len()).step_by(13) {
            sym[i] = -sym[i];
        }
        let got = decode(&sym, 37).expect("a packet through the errors");
        assert_eq!(got.pdu, pdu);
    }

    /// The whitening carries the channel, so a packet read as another channel
    /// fails its CRC rather than decoding to nonsense.
    #[test]
    fn a_packet_read_on_the_wrong_channel_is_refused() {
        let sym = soft(&encode_packet(37, &adv_ext_ind(), Coding::S8));
        assert!(decode(&sym, 38).is_none());
    }

    #[test]
    fn noise_holds_no_packet() {
        let mut seed = 0x9e37_79b9_7f4a_7c15u64;
        let sym: Vec<f32> = (0..40_000)
            .map(|_| {
                seed ^= seed << 13;
                seed ^= seed >> 7;
                seed ^= seed << 17;
                if seed & 1 == 0 {
                    1.0
                } else {
                    -1.0
                }
            })
            .collect();
        assert!(decode(&sym, 37).is_none());
    }

    /// The terminator returns the encoder to the state it started in, which
    /// is what lets a block be decoded on its own.
    #[test]
    fn three_zeros_return_the_encoder_to_its_start() {
        let a = encode_block(&[true, false, true, false, false, false], Coding::S2);
        let b = encode_block(&[true, false, true, false, false, false, true], Coding::S2);
        // The seventh bit's output is the same as a first bit's would be.
        assert_eq!(&b[..a.len()], &a[..]);
        assert_eq!(&b[a.len()..], &encode_block(&[true], Coding::S2)[..]);
    }
}
