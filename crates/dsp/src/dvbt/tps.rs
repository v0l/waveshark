//! The Transmission Parameter Signalling carriers, EN 300 744 clause 4.6.
//!
//! Sixty-eight symbols make a frame, and each symbol's TPS carriers hold one
//! bit of a sixty-eight bit word that says what the rest of the multiplex is:
//! constellation, code rates, guard interval, transmission mode, which of the
//! four frames this is, and half of a cell identifier. The bit is carried
//! differentially, all seventeen or sixty-eight carriers saying the same
//! thing, so a receiver that has an equaliser and a previous symbol can read
//! it without knowing anything else.
//!
//! A word is protected by a shortened BCH(67,53), which is BCH(127,113) with
//! sixty zeros in front of it. The parity is checked and not corrected: two
//! errors are correctable in principle, but a word that fails here is
//! repeated whole four symbols later, and taking it again costs less than
//! being wrong about the multiplex.

use super::{CodeRate, Constellation, Guard, Hierarchy, Mode, Params, SYMBOLS_PER_FRAME};

/// The synchronisation word of an even frame. An odd frame carries its
/// complement, 0xCA11.
const SYNC_EVEN: u16 = 0x35EE;

/// Bits 17 to 22, the length indicator: how many bits after the sync word are
/// in use. 23 without a cell identifier, 31 with one.
const LENGTH_NO_CELL_ID: u8 = 0x17;
const LENGTH_CELL_ID: u8 = 0x1F;

/// One decoded TPS word.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Tps {
    /// Which of the four frames of a super frame this is.
    pub frame: u8,
    pub mode: Mode,
    pub guard: Guard,
    pub constellation: Constellation,
    pub hierarchy: Hierarchy,
    pub code_rate_hp: CodeRate,
    pub code_rate_lp: CodeRate,
    /// The half of the cell identifier this frame carries: the high byte in
    /// an even frame, the low byte in an odd one. `None` where the length
    /// indicator says the transmitter sends no cell identifier.
    pub cell_id_byte: Option<u8>,
}

impl Tps {
    /// The multiplex's parameters, with a cell identifier where both halves
    /// have been seen.
    pub fn params(&self, cell_id: Option<u16>) -> Params {
        Params {
            mode: self.mode,
            guard: self.guard,
            constellation: self.constellation,
            hierarchy: self.hierarchy,
            code_rate_hp: self.code_rate_hp,
            code_rate_lp: self.code_rate_lp,
            cell_id,
        }
    }
}

/// Write `value` into `bits[stop..=start]`, most significant bit at `stop`,
/// which is how the standard numbers a TPS field.
fn put(bits: &mut [u8; SYMBOLS_PER_FRAME], start: usize, stop: usize, value: u32) {
    let mut value = value;
    for i in (stop..=start).rev() {
        bits[i] = (value & 1) as u8;
        value >>= 1;
    }
}

/// Read `bits[stop..=start]` as an unsigned field.
fn get(bits: &[u8], start: usize, stop: usize) -> u32 {
    let mut out = 0u32;
    for bit in &bits[stop..=start] {
        out = (out << 1) | *bit as u32;
    }
    out
}

/// The BCH parity of a word, over X^14+X^9+X^8+X^6+X^5+X^4+X^2+X+1 with the
/// fifty-three information bits preceded by the sixty zeros that shorten
/// BCH(127,113) to BCH(67,53). Returns the fourteen parity bits in the order
/// they are transmitted.
fn bch(info: &[u8]) -> [u8; 14] {
    debug_assert_eq!(info.len(), 53);
    let mut reg: u32 = 0;
    for i in 0..113 {
        let data = if i < 60 { 0 } else { info[i - 60] };
        let feedback = (data ^ reg as u8) & 1;
        reg >>= 1;
        reg |= (feedback as u32) << 13;
        reg ^= (feedback as u32)
            * ((1 << 12) | (1 << 11) | (1 << 9) | (1 << 8) | (1 << 7) | (1 << 5) | (1 << 4));
    }
    std::array::from_fn(|i| ((reg >> i) & 1) as u8)
}

/// The sixty-eight bits a transmitter puts on the TPS carriers of one frame.
/// Bit zero is the initialisation bit, which carries nothing and is left at
/// zero here; the frame's own reference value is what the receiver starts
/// from.
pub fn encode(params: &Params, frame: u8) -> [u8; SYMBOLS_PER_FRAME] {
    let mut bits = [0u8; SYMBOLS_PER_FRAME];
    let odd = frame % 2 == 1;
    let sync = if odd { !SYNC_EVEN } else { SYNC_EVEN };
    put(&mut bits, 16, 1, sync as u32);
    let length = if params.cell_id.is_some() { LENGTH_CELL_ID } else { LENGTH_NO_CELL_ID };
    put(&mut bits, 22, 17, length as u32);
    put(&mut bits, 24, 23, frame as u32 & 3);
    put(&mut bits, 26, 25, params.constellation.tps_bits() as u32);
    put(&mut bits, 29, 27, params.hierarchy.tps_bits() as u32);
    put(&mut bits, 32, 30, params.code_rate_hp.tps_bits() as u32);
    put(&mut bits, 35, 33, params.code_rate_lp.tps_bits() as u32);
    put(&mut bits, 37, 36, params.guard.tps_bits() as u32);
    put(&mut bits, 39, 38, params.mode.tps_bits() as u32);
    let cell = params.cell_id.unwrap_or(0);
    let byte = if odd { cell & 0xff } else { cell >> 8 };
    put(&mut bits, 47, 40, byte as u32);
    let parity = bch(&bits[1..54]);
    bits[54..68].copy_from_slice(&parity);
    bits
}

/// Read a sixty-eight bit window as a TPS word. `None` where the sync word is
/// not at the front, the parity fails, or a field names a combination the
/// standard does not define.
pub fn decode(bits: &[u8]) -> Option<Tps> {
    if bits.len() != SYMBOLS_PER_FRAME {
        return None;
    }
    let sync = get(bits, 16, 1) as u16;
    let odd = match sync {
        SYNC_EVEN => false,
        s if s == !SYNC_EVEN => true,
        _ => return None,
    };
    if bch(&bits[1..54])[..] != bits[54..68] {
        return None;
    }
    let length = get(bits, 22, 17) as u8;
    let frame = get(bits, 24, 23) as u8;
    if (frame % 2 == 1) != odd {
        return None;
    }
    let cell_id_byte = match length {
        LENGTH_CELL_ID => Some(get(bits, 47, 40) as u8),
        LENGTH_NO_CELL_ID => None,
        _ => return None,
    };
    Some(Tps {
        frame,
        mode: Mode::from_tps(get(bits, 39, 38) as u8)?,
        guard: Guard::from_tps(get(bits, 37, 36) as u8)?,
        constellation: Constellation::from_tps(get(bits, 26, 25) as u8)?,
        hierarchy: Hierarchy::from_tps(get(bits, 29, 27) as u8)?,
        code_rate_hp: CodeRate::from_tps(get(bits, 32, 30) as u8)?,
        code_rate_lp: CodeRate::from_tps(get(bits, 35, 33) as u8)?,
        cell_id_byte,
    })
}

/// Sixty-eight bits of history, read as a TPS word wherever one starts.
///
/// The receiver does not know where a frame begins until a sync word turns
/// up, so every bit is pushed in and every window of sixty-eight is a
/// candidate. Once a word decodes, the symbol it ended on is symbol 67 of its
/// frame and the frame boundary is known.
#[derive(Clone, Debug, Default)]
pub struct TpsDecoder {
    bits: Vec<u8>,
    /// The two halves of the cell identifier, kept across frames because one
    /// frame carries one of them.
    cell_hi: Option<u8>,
    cell_lo: Option<u8>,
}

impl TpsDecoder {
    pub fn new() -> Self {
        Self::default()
    }

    /// Push one symbol's bit. Returns a word when the last sixty-eight bits
    /// are one, which means the symbol just pushed was symbol 67.
    pub fn push(&mut self, bit: u8) -> Option<Tps> {
        self.bits.push(bit);
        if self.bits.len() > SYMBOLS_PER_FRAME {
            self.bits.remove(0);
        }
        if self.bits.len() < SYMBOLS_PER_FRAME {
            return None;
        }
        let tps = decode(&self.bits)?;
        match (tps.cell_id_byte, tps.frame % 2) {
            (Some(b), 0) => self.cell_hi = Some(b),
            (Some(b), _) => self.cell_lo = Some(b),
            (None, _) => {}
        }
        Some(tps)
    }

    /// The cell identifier, once both halves have been read.
    pub fn cell_id(&self) -> Option<u16> {
        match (self.cell_hi, self.cell_lo) {
            (Some(hi), Some(lo)) => Some(((hi as u16) << 8) | lo as u16),
            _ => None,
        }
    }

    pub fn reset(&mut self) {
        self.bits.clear();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Every combination the standard defines survives the word it is written
    /// into, all four frames of the super frame.
    #[test]
    fn every_parameter_set_survives_the_tps_word() {
        let mut checked = 0;
        for mode in Mode::ALL {
            for guard in Guard::ALL {
                for constellation in Constellation::ALL {
                    for rate in CodeRate::ALL {
                        let params = Params {
                            mode,
                            guard,
                            constellation,
                            hierarchy: Hierarchy::None,
                            code_rate_hp: rate,
                            code_rate_lp: rate,
                            cell_id: Some(0x1A2B),
                        };
                        for frame in 0..4u8 {
                            let bits = encode(&params, frame);
                            let tps = decode(&bits).expect("the word decodes");
                            assert_eq!(tps.frame, frame);
                            assert_eq!(tps.params(Some(0x1A2B)), params);
                            let expect = if frame % 2 == 0 { 0x1A } else { 0x2B };
                            assert_eq!(tps.cell_id_byte, Some(expect));
                            checked += 1;
                        }
                    }
                }
            }
        }
        assert_eq!(checked, 2 * 4 * 3 * 5 * 4);
    }

    /// The cell identifier arrives a byte at a time and is only whole once
    /// both an even and an odd frame have been read.
    #[test]
    fn the_cell_identifier_is_assembled_from_two_frames() {
        let params = Params { cell_id: Some(0xBEEF), ..Params::typical() };
        let mut rx = TpsDecoder::new();
        for bit in encode(&params, 0) {
            rx.push(bit);
        }
        assert_eq!(rx.cell_id(), None, "one frame is half an identifier");
        for bit in encode(&params, 1) {
            rx.push(bit);
        }
        assert_eq!(rx.cell_id(), Some(0xBEEF));
    }

    /// The decoder finds the frame boundary with no idea where it started:
    /// forty bits of nonsense, then a word, and the word is read.
    #[test]
    fn a_word_is_found_in_a_stream_that_started_anywhere() {
        let params = Params::typical();
        let mut rx = TpsDecoder::new();
        let mut found = Vec::new();
        for i in 0..40 {
            if let Some(t) = rx.push((i % 3 == 0) as u8) {
                found.push(t);
            }
        }
        for frame in 0..4u8 {
            for bit in encode(&params, frame) {
                if let Some(t) = rx.push(bit) {
                    found.push(t);
                }
            }
        }
        assert_eq!(found.len(), 4, "one word per frame and no false ones");
        assert_eq!(found.iter().map(|t| t.frame).collect::<Vec<_>>(), vec![0, 1, 2, 3]);
        assert_eq!(found[0].params(None), Params { cell_id: None, ..params });
    }

    /// A single bit error anywhere in the protected part is caught by the
    /// parity rather than read as a different multiplex.
    #[test]
    fn one_flipped_bit_is_refused() {
        let params = Params::typical();
        let good = encode(&params, 0);
        let mut caught = 0;
        for i in 1..SYMBOLS_PER_FRAME {
            let mut bad = good;
            bad[i] ^= 1;
            if decode(&bad).is_none() {
                caught += 1;
            }
        }
        assert_eq!(caught, SYMBOLS_PER_FRAME - 1, "every single error is refused");
    }
}
