//! DVB-T, the terrestrial television multiplex, as ETSI EN 300 744 describes
//! it: an 8 MHz channel of OFDM carrying an MPEG transport stream.
//!
//! Everything here is at the standard's own elementary period. An 8 MHz
//! channel has T = 7/64 us, so the modulation lives at 64/7 MS/s and the
//! receiver resamples to that before it gets here. The useful part of a
//! symbol is the FFT size: 2048 carriers in 2K mode, 8192 in 8K, of which
//! 1705 and 6817 are transmitted.
//!
//! A carrier is one of four things, and which one depends on the symbol's
//! place in the frame: a data cell, a scattered pilot that moves every
//! symbol, a continual pilot that does not, or one of the TPS carriers that
//! spell out the multiplex's own parameters over 68 symbols.
//!
//! The layer above this is `decode::dvbt`, which takes the soft bits a symbol
//! carries through the deinterleavers, the Viterbi and Reed-Solomon to
//! transport packets. Nothing here knows what a transport packet is.

pub mod inner;
pub mod rx;
pub mod tps;
pub mod tx;

use common::C32;

pub use inner::Inner;
pub use rx::{Dvbt, Symbol};
pub use tps::{Tps, TpsDecoder};

/// Sample rate of an 8 MHz DVB-T channel: the elementary period is 7/64 us.
pub const RATE_HZ: f64 = 64_000_000.0 / 7.0;
/// The allocation a multiplex owns. The modulation itself reaches 7.61 MHz
/// in 8K mode and 7.61 MHz in 2K; the channel raster is 8 MHz.
pub const CHANNEL_WIDTH_HZ: f64 = 8_000_000.0;
/// Symbols in one TPS frame, and so the period of the frame counter.
pub const SYMBOLS_PER_FRAME: usize = 68;
/// Frames in a super frame, which is what makes a whole number of transport
/// packets come out whatever the constellation and code rate.
pub const FRAMES_PER_SUPERFRAME: usize = 4;

/// The FFT size, and with it the number of transmitted carriers.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Mode {
    /// 2048 carrier FFT, 1705 transmitted.
    M2k,
    /// 8192 carrier FFT, 6817 transmitted.
    M8k,
}

impl Mode {
    pub const ALL: [Mode; 2] = [Mode::M2k, Mode::M8k];

    pub const fn fft(self) -> usize {
        match self {
            Mode::M2k => 2048,
            Mode::M8k => 8192,
        }
    }

    /// The highest transmitted carrier index. The lowest is always zero.
    pub const fn k_max(self) -> usize {
        match self {
            Mode::M2k => 1704,
            Mode::M8k => 6816,
        }
    }

    /// Transmitted carriers, pilots included.
    pub const fn carriers(self) -> usize {
        self.k_max() + 1
    }

    /// Data cells in one symbol, which is what the symbol interleaver
    /// permutes.
    pub const fn cells(self) -> usize {
        match self {
            Mode::M2k => 1512,
            Mode::M8k => 6048,
        }
    }

    pub const fn label(self) -> &'static str {
        match self {
            Mode::M2k => "2k",
            Mode::M8k => "8k",
        }
    }

    /// TPS bits 0 and 1 of field 4.6.2.8.
    pub const fn tps_bits(self) -> u8 {
        match self {
            Mode::M2k => 0b00,
            Mode::M8k => 0b01,
        }
    }

    pub const fn from_tps(bits: u8) -> Option<Mode> {
        match bits {
            0b00 => Some(Mode::M2k),
            0b01 => Some(Mode::M8k),
            _ => None,
        }
    }
}

/// The cyclic prefix, as a fraction of the useful part.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Guard {
    G1_32,
    G1_16,
    G1_8,
    G1_4,
}

impl Guard {
    pub const ALL: [Guard; 4] = [Guard::G1_32, Guard::G1_16, Guard::G1_8, Guard::G1_4];

    /// The denominator: a guard of 1/`divisor` of the useful part.
    pub const fn divisor(self) -> usize {
        match self {
            Guard::G1_32 => 32,
            Guard::G1_16 => 16,
            Guard::G1_8 => 8,
            Guard::G1_4 => 4,
        }
    }

    /// Prefix length in samples for `mode`.
    pub const fn samples(self, mode: Mode) -> usize {
        mode.fft() / self.divisor()
    }

    pub const fn label(self) -> &'static str {
        match self {
            Guard::G1_32 => "1/32",
            Guard::G1_16 => "1/16",
            Guard::G1_8 => "1/8",
            Guard::G1_4 => "1/4",
        }
    }

    pub const fn tps_bits(self) -> u8 {
        match self {
            Guard::G1_32 => 0b00,
            Guard::G1_16 => 0b01,
            Guard::G1_8 => 0b10,
            Guard::G1_4 => 0b11,
        }
    }

    pub const fn from_tps(bits: u8) -> Option<Guard> {
        match bits {
            0b00 => Some(Guard::G1_32),
            0b01 => Some(Guard::G1_16),
            0b10 => Some(Guard::G1_8),
            0b11 => Some(Guard::G1_4),
            _ => None,
        }
    }
}

/// What a data cell carries.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Constellation {
    Qpsk,
    Qam16,
    Qam64,
}

impl Constellation {
    pub const ALL: [Constellation; 3] =
        [Constellation::Qpsk, Constellation::Qam16, Constellation::Qam64];

    /// Bits a cell carries, the standard's v.
    pub const fn bits(self) -> usize {
        match self {
            Constellation::Qpsk => 2,
            Constellation::Qam16 => 4,
            Constellation::Qam64 => 6,
        }
    }

    /// The factor that makes the constellation unit power, non-hierarchical.
    pub fn norm(self) -> f32 {
        match self {
            Constellation::Qpsk => 1.0 / 2f32.sqrt(),
            Constellation::Qam16 => 1.0 / 10f32.sqrt(),
            Constellation::Qam64 => 1.0 / 42f32.sqrt(),
        }
    }

    pub const fn label(self) -> &'static str {
        match self {
            Constellation::Qpsk => "QPSK",
            Constellation::Qam16 => "16-QAM",
            Constellation::Qam64 => "64-QAM",
        }
    }

    pub const fn tps_bits(self) -> u8 {
        match self {
            Constellation::Qpsk => 0b00,
            Constellation::Qam16 => 0b01,
            Constellation::Qam64 => 0b10,
        }
    }

    pub const fn from_tps(bits: u8) -> Option<Constellation> {
        match bits {
            0b00 => Some(Constellation::Qpsk),
            0b01 => Some(Constellation::Qam16),
            0b10 => Some(Constellation::Qam64),
            _ => None,
        }
    }
}

/// The inner convolutional code's rate, after puncturing.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum CodeRate {
    R1_2,
    R2_3,
    R3_4,
    R5_6,
    R7_8,
}

impl CodeRate {
    pub const ALL: [CodeRate; 5] =
        [CodeRate::R1_2, CodeRate::R2_3, CodeRate::R3_4, CodeRate::R5_6, CodeRate::R7_8];

    /// Information bits in one puncturing period.
    pub const fn k(self) -> usize {
        match self {
            CodeRate::R1_2 => 1,
            CodeRate::R2_3 => 2,
            CodeRate::R3_4 => 3,
            CodeRate::R5_6 => 5,
            CodeRate::R7_8 => 7,
        }
    }

    /// Coded bits transmitted in one puncturing period.
    pub const fn n(self) -> usize {
        match self {
            CodeRate::R1_2 => 2,
            CodeRate::R2_3 => 3,
            CodeRate::R3_4 => 4,
            CodeRate::R5_6 => 6,
            CodeRate::R7_8 => 8,
        }
    }

    /// Which mother bits the transmitter sends, one entry a bit in the order
    /// X1 Y1 X2 Y2 and so on. EN 300 744 table 3, which writes the kept bits
    /// out as X1Y1, X1Y1Y2, X1Y1Y2X3, X1Y1Y2X3Y4X5 and X1Y1Y2Y3Y4X5Y6X7.
    pub const fn mask(self) -> &'static [u8] {
        match self {
            CodeRate::R1_2 => &[1, 1],
            CodeRate::R2_3 => &[1, 1, 0, 1],
            CodeRate::R3_4 => &[1, 1, 0, 1, 1, 0],
            CodeRate::R5_6 => &[1, 1, 0, 1, 1, 0, 0, 1, 1, 0],
            CodeRate::R7_8 => &[1, 1, 0, 1, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0],
        }
    }

    pub const fn label(self) -> &'static str {
        match self {
            CodeRate::R1_2 => "1/2",
            CodeRate::R2_3 => "2/3",
            CodeRate::R3_4 => "3/4",
            CodeRate::R5_6 => "5/6",
            CodeRate::R7_8 => "7/8",
        }
    }

    pub const fn tps_bits(self) -> u8 {
        match self {
            CodeRate::R1_2 => 0b000,
            CodeRate::R2_3 => 0b001,
            CodeRate::R3_4 => 0b010,
            CodeRate::R5_6 => 0b011,
            CodeRate::R7_8 => 0b100,
        }
    }

    pub const fn from_tps(bits: u8) -> Option<CodeRate> {
        match bits {
            0b000 => Some(CodeRate::R1_2),
            0b001 => Some(CodeRate::R2_3),
            0b010 => Some(CodeRate::R3_4),
            0b011 => Some(CodeRate::R5_6),
            0b100 => Some(CodeRate::R7_8),
            _ => None,
        }
    }
}

/// Hierarchical modulation, which splits the constellation into a rugged
/// stream and a fragile one. Nothing in Europe transmits it, but the TPS
/// says which it is and the demapper needs to know.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Hierarchy {
    None,
    Alpha1,
    Alpha2,
    Alpha4,
}

impl Hierarchy {
    pub const fn label(self) -> &'static str {
        match self {
            Hierarchy::None => "non-hierarchical",
            Hierarchy::Alpha1 => "alpha 1",
            Hierarchy::Alpha2 => "alpha 2",
            Hierarchy::Alpha4 => "alpha 4",
        }
    }

    pub const fn tps_bits(self) -> u8 {
        match self {
            Hierarchy::None => 0b000,
            Hierarchy::Alpha1 => 0b001,
            Hierarchy::Alpha2 => 0b010,
            Hierarchy::Alpha4 => 0b100,
        }
    }

    pub const fn from_tps(bits: u8) -> Option<Hierarchy> {
        match bits {
            0b000 => Some(Hierarchy::None),
            0b001 => Some(Hierarchy::Alpha1),
            0b010 => Some(Hierarchy::Alpha2),
            0b100 => Some(Hierarchy::Alpha4),
            _ => None,
        }
    }
}

/// Everything about a multiplex that the TPS carriers say, and that the
/// layers above need in order to read it.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct Params {
    pub mode: Mode,
    pub guard: Guard,
    pub constellation: Constellation,
    pub hierarchy: Hierarchy,
    pub code_rate_hp: CodeRate,
    pub code_rate_lp: CodeRate,
    pub cell_id: Option<u16>,
}

impl Params {
    /// The common British transmission: 8K, 1/32 guard, 64-QAM, 2/3.
    pub fn typical() -> Self {
        Self {
            mode: Mode::M8k,
            guard: Guard::G1_32,
            constellation: Constellation::Qam64,
            hierarchy: Hierarchy::None,
            code_rate_hp: CodeRate::R2_3,
            code_rate_lp: CodeRate::R2_3,
            cell_id: None,
        }
    }

    /// Samples in one symbol, prefix included.
    pub fn symbol_samples(&self) -> usize {
        self.mode.fft() + self.guard.samples(self.mode)
    }

    /// Useful bit rate, before the transport stream's own overheads: cells a
    /// second, times bits a cell, times both code rates, times 188/204.
    pub fn bitrate(&self) -> f64 {
        let symbol_s = self.symbol_samples() as f64 / RATE_HZ;
        let cells_s = self.mode.cells() as f64 / symbol_s;
        let rate = self.code_rate_hp;
        cells_s * self.constellation.bits() as f64 * rate.k() as f64 / rate.n() as f64
            * (188.0 / 204.0)
    }

    /// What an operator reads off the pane: "8k 1/32 64-QAM 2/3".
    pub fn label(&self) -> String {
        format!(
            "{} {} {} {}",
            self.mode.label(),
            self.guard.label(),
            self.constellation.label(),
            self.code_rate_hp.label()
        )
    }
}

/// The FFT bin carrier `k` lives in. The transmitted carriers are centred on
/// the channel, so carrier `k_max / 2` is DC and the rest run either side.
pub const fn bin(mode: Mode, k: usize) -> usize {
    (k + mode.fft() - mode.k_max() / 2) % mode.fft()
}

/// What a carrier carries in a given symbol.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Carrier {
    Data,
    /// A pilot, scattered or continual: both carry the same boosted value.
    Pilot,
    Tps,
}

/// The 2K continual pilot carriers, EN 300 744 table 6. The 8K list is this
/// one repeated every 1704 carriers, which `continual_pilots` builds.
const CONTINUAL_2K: [u16; 45] = [
    0, 48, 54, 87, 141, 156, 192, 201, 255, 279, 282, 333, 432, 450, 483, 525, 531, 618, 636, 714,
    759, 765, 780, 804, 873, 888, 918, 939, 942, 969, 984, 1050, 1101, 1107, 1110, 1137, 1140,
    1146, 1206, 1269, 1323, 1377, 1491, 1683, 1704,
];

/// The 2K TPS carriers, EN 300 744 table 8, with the 8K list built the same
/// way as the continual pilots.
const TPS_2K: [u16; 17] =
    [34, 50, 209, 346, 413, 569, 595, 688, 790, 901, 1073, 1219, 1262, 1286, 1469, 1594, 1687];

/// The continual pilot carriers of `mode`, ascending.
pub fn continual_pilots(mode: Mode) -> Vec<usize> {
    repeat_table(&CONTINUAL_2K, mode)
}

/// The TPS carriers of `mode`, ascending.
pub fn tps_carriers(mode: Mode) -> Vec<usize> {
    repeat_table(&TPS_2K, mode)
}

/// The 8K tables are the 2K tables translated by 1704 three times over, which
/// is how EN 300 744 lists them: 6816 = 4 x 1704, and a carrier landing on a
/// join is only listed once.
fn repeat_table(base: &[u16], mode: Mode) -> Vec<usize> {
    let mut out: Vec<usize> = Vec::new();
    let blocks = match mode {
        Mode::M2k => 1,
        Mode::M8k => 4,
    };
    for b in 0..blocks {
        for &k in base {
            let k = k as usize + b * 1704;
            if k <= mode.k_max() && out.last() != Some(&k) {
                out.push(k);
            }
        }
    }
    out
}

/// The pilot reference sequence w_k, EN 300 744 clause 4.5.2: an eleven stage
/// shift register over X^11 + X^2 + 1 started at all ones, one bit per
/// transmitted carrier.
pub fn prbs(mode: Mode) -> Vec<u8> {
    let mut reg: u16 = (1 << 11) - 1;
    let mut out = Vec::with_capacity(mode.carriers());
    for _ in 0..mode.carriers() {
        out.push((reg & 1) as u8);
        let bit = ((reg >> 2) ^ reg) & 1;
        reg = (reg >> 1) | (bit << 10);
    }
    out
}

/// The value a pilot on carrier `k` carries: +-4/3, boosted so that a channel
/// estimate taken off it is 2.5 dB better than the data around it.
pub fn pilot_value(w: &[u8], k: usize) -> f32 {
    4.0 / 3.0 * 2.0 * (0.5 - w[k] as f32)
}

/// The value a TPS carrier holds in the first symbol of a frame, before the
/// differential modulation that carries the bits: +-1, unboosted.
pub fn tps_reference(w: &[u8], k: usize) -> f32 {
    2.0 * (0.5 - w[k] as f32)
}

/// The carrier layout of one mode: what each carrier is, in each of the four
/// scattered pilot phases, worked out once and then read.
#[derive(Clone, Debug)]
pub struct Layout {
    pub mode: Mode,
    /// `kind[phase][k]`, with phase the symbol index modulo four.
    kind: [Vec<Carrier>; 4],
    /// The data carrier indices of each phase, in ascending order, which is
    /// the order cells leave the symbol in.
    data: [Vec<u16>; 4],
    w: Vec<u8>,
}

impl Layout {
    pub fn new(mode: Mode) -> Self {
        let w = prbs(mode);
        let continual = continual_pilots(mode);
        let tps = tps_carriers(mode);
        let mut kind: [Vec<Carrier>; 4] =
            std::array::from_fn(|_| vec![Carrier::Data; mode.carriers()]);
        let mut data: [Vec<u16>; 4] = std::array::from_fn(|_| Vec::with_capacity(mode.cells()));
        for phase in 0..4 {
            let k = &mut kind[phase];
            let mut c = 0;
            while c <= mode.k_max() {
                if c % 12 == 3 * phase {
                    k[c] = Carrier::Pilot;
                }
                c += 1;
            }
            for &c in &continual {
                k[c] = Carrier::Pilot;
            }
            for &c in &tps {
                k[c] = Carrier::Tps;
            }
            for (c, kind) in k.iter().enumerate() {
                if *kind == Carrier::Data {
                    data[phase].push(c as u16);
                }
            }
            assert_eq!(
                data[phase].len(),
                mode.cells(),
                "a {} symbol carries {} data cells",
                mode.label(),
                mode.cells()
            );
        }
        Self { mode, kind, data, w }
    }

    /// What carrier `k` is in a symbol whose index modulo four is `phase`.
    pub fn carrier(&self, phase: usize, k: usize) -> Carrier {
        self.kind[phase & 3][k]
    }

    /// The data carriers of a symbol, ascending.
    pub fn data(&self, phase: usize) -> &[u16] {
        &self.data[phase & 3]
    }

    /// The pilot reference sequence.
    pub fn w(&self) -> &[u8] {
        &self.w
    }

    /// The value the pilot on carrier `k` carries.
    pub fn pilot(&self, k: usize) -> f32 {
        pilot_value(&self.w, k)
    }

    /// The value a TPS carrier starts a frame at.
    pub fn tps_reference(&self, k: usize) -> f32 {
        tps_reference(&self.w, k)
    }
}

/// One data cell mapped to its point, EN 300 744 clause 4.3.5. `bits` is the
/// cell's v bits, most significant first: the even-numbered ones carry the
/// in-phase axis and the odd-numbered ones the quadrature, each Gray coded
/// with its first bit the sign.
pub fn map(bits: &[u8], constellation: Constellation) -> C32 {
    let v = constellation.bits();
    debug_assert_eq!(bits.len(), v);
    let m = v / 2;
    let word = |start: usize| -> usize {
        (0..m).fold(0, |acc, j| (acc << 1) | bits[start + 2 * j] as usize)
    };
    C32::new(axis(word(0), m), axis(word(1), m)) * constellation.norm()
}

/// One axis of a constellation point, before normalisation, from the `m` bits
/// that axis carries: the first is the sign and the rest are a Gray coded
/// magnitude, so a level differs from its neighbour in one bit and a slip
/// costs one bit rather than two.
pub fn axis(word: usize, m: usize) -> f32 {
    let sign = if word >> (m - 1) == 0 { 1.0 } else { -1.0 };
    let gray = word & ((1 << (m - 1)) - 1);
    let mut natural = 0usize;
    let mut prev = 0usize;
    for j in (0..m.saturating_sub(1)).rev() {
        prev ^= (gray >> j) & 1;
        natural |= prev << j;
    }
    let levels = 1usize << (m - 1);
    sign * (2 * (levels - 1 - natural) + 1) as f32
}

#[cfg(test)]
mod tests {
    use super::*;

    /// EN 300 744 tables 6 and 8 give the 8K lists in full. Their shape is
    /// the 2K list translated by 1704, which is what `repeat_table` builds,
    /// so what is checked here is the count and the ends.
    #[test]
    fn the_pilot_and_tps_tables_are_the_sizes_the_standard_gives() {
        assert_eq!(continual_pilots(Mode::M2k).len(), 45);
        assert_eq!(continual_pilots(Mode::M8k).len(), 177);
        assert_eq!(tps_carriers(Mode::M2k).len(), 17);
        assert_eq!(tps_carriers(Mode::M8k).len(), 68);
        assert_eq!(*continual_pilots(Mode::M8k).last().unwrap(), 6816);
        assert_eq!(*tps_carriers(Mode::M8k).last().unwrap(), 6799);
        for mode in Mode::ALL {
            let c = continual_pilots(mode);
            assert!(c.windows(2).all(|w| w[0] < w[1]), "the table is ascending and has no repeats");
        }
    }

    /// A TPS carrier never lands on a scattered pilot position, which is why
    /// the layout can classify without an order of precedence between them.
    #[test]
    fn no_tps_carrier_sits_where_a_scattered_pilot_goes() {
        for mode in Mode::ALL {
            for k in tps_carriers(mode) {
                assert!(!matches!(k % 12, 0 | 3 | 6 | 9), "carrier {k} collides");
            }
        }
    }

    /// Every phase of every mode leaves exactly the cell count the symbol
    /// interleaver is defined over.
    #[test]
    fn every_symbol_carries_the_same_number_of_data_cells() {
        for mode in Mode::ALL {
            let layout = Layout::new(mode);
            for phase in 0..4 {
                assert_eq!(layout.data(phase).len(), mode.cells());
            }
        }
    }

    /// The reference sequence starts 1,1,1,1,1,1,1,1,1,1,1,0 and so the first
    /// pilot is negative: w_0 = 1 gives -4/3 on carrier zero.
    #[test]
    fn the_reference_sequence_starts_all_ones() {
        let w = prbs(Mode::M2k);
        assert_eq!(&w[..12], &[1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0]);
        assert!((pilot_value(&w, 0) + 4.0 / 3.0).abs() < 1e-6);
    }

    /// Every point is unit power on average and every point is distinct.
    #[test]
    fn the_constellations_are_unit_power_and_one_to_one() {
        for c in Constellation::ALL {
            let v = c.bits();
            let mut seen: Vec<C32> = Vec::new();
            let mut power = 0.0f32;
            for word in 0..(1usize << v) {
                let bits: Vec<u8> = (0..v).map(|i| ((word >> (v - 1 - i)) & 1) as u8).collect();
                let p = map(&bits, c);
                power += p.norm_sqr();
                assert!(
                    seen.iter().all(|q| (q - p).norm() > 1e-6),
                    "{} maps two words to one point",
                    c.label()
                );
                seen.push(p);
            }
            let mean = power / (1 << v) as f32;
            assert!((mean - 1.0).abs() < 1e-5, "{} is {mean} power", c.label());
        }
    }

    /// 64-QAM 2/3 at a 1/32 guard is the 24.1 Mbit/s a British multiplex
    /// carries, which is the number Ofcom's own multiplex documents give.
    #[test]
    fn the_typical_multiplex_carries_twenty_four_megabits() {
        let bitrate = Params::typical().bitrate();
        assert!(
            (bitrate - 24_128_000.0).abs() < 2_000.0,
            "{bitrate} bit/s, expected 24.128 Mbit/s"
        );
    }
}
