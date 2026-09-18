//! DRM, the shortwave and medium wave broadcast standard ETSI ES 201 980
//! describes: OFDM in a 10 kHz channel carrying one to four services.
//!
//! Everything here is at 12 kS/s, which is the rate the frame lengths come
//! out whole at: a frame is 400 ms, and 4 800 samples whatever the
//! robustness mode. A mode is a set of symbol, guard and carrier lengths
//! chosen for how hard the path is, A being a ground wave and D a badly
//! scattered sky wave.
//!
//! The carriers are not differential, so a receiver has to estimate the
//! channel. It is given three kinds of pilot to do it with: gain references
//! scattered over the grid on a pattern this module computes, three
//! frequency references at 750, 2 250 and 3 000 Hz that are in every symbol,
//! and a row of time references in the first symbol of a frame that say
//! where a frame begins.
//!
//! What a cell carries depends on where it is: the fast access channel is a
//! fixed table of positions, the service description channel is whatever is
//! left of the first symbols, and the rest is the multiplex. The layer above
//! is `decode::drm`, which takes the cells through their codes to the
//! channel and service parameters. Nothing here knows what a service is.

pub mod rx;
pub mod tx;

use common::C32;
use std::f32::consts::TAU;

pub use rx::{Drm, Frame};

/// The rate the modes are defined at here: 400 ms of frame is 4 800 samples.
pub const RATE_HZ: f64 = 12_000.0;
/// What a channel occupies at the widest occupancy this reads.
pub const CHANNEL_WIDTH_HZ: f64 = 10_000.0;
/// A transmission frame, as the standard fixes it.
pub const FRAME_SECONDS: f64 = 0.4;
/// Frames in a transmission super frame. The service description channel is
/// in the first of them.
pub const FRAMES_PER_SUPER: usize = 3;

/// The robustness mode: one set of symbol, guard and carrier lengths.
///
/// A is for a ground wave with almost no echo, B for a sky wave, and C and D
/// for a path so scattered that the guard has to be a third of the symbol.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Mode {
    A,
    B,
    C,
    D,
}

impl Mode {
    pub const ALL: [Mode; 4] = [Mode::A, Mode::B, Mode::C, Mode::D];

    /// The useful part of a symbol, T_u, which is the transform size.
    pub const fn fft(self) -> usize {
        match self {
            Mode::A => 288,
            Mode::B => 256,
            Mode::C => 176,
            Mode::D => 112,
        }
    }

    /// The cyclic prefix, T_g.
    pub const fn guard(self) -> usize {
        match self {
            Mode::A => 32,
            Mode::B => 64,
            Mode::C => 64,
            Mode::D => 88,
        }
    }

    /// A whole symbol, prefix included.
    pub const fn symbol(self) -> usize {
        self.fft() + self.guard()
    }

    /// Symbols in a transmission frame.
    pub const fn symbols(self) -> usize {
        match self {
            Mode::A | Mode::B => 15,
            Mode::C => 20,
            Mode::D => 24,
        }
    }

    /// Samples in a transmission frame, which is 400 ms in every mode.
    pub const fn frame(self) -> usize {
        self.symbol() * self.symbols()
    }

    /// Symbols of a frame the service description channel is spread over.
    pub const fn sdc_symbols(self) -> usize {
        match self {
            Mode::A | Mode::B => 2,
            Mode::C | Mode::D => 3,
        }
    }

    pub const fn label(self) -> &'static str {
        match self {
            Mode::A => "A",
            Mode::B => "B",
            Mode::C => "C",
            Mode::D => "D",
        }
    }

    /// Carrier spacing.
    pub fn spacing_hz(self) -> f64 {
        RATE_HZ / self.fft() as f64
    }

    /// The gain reference pattern: how far apart the pilots are in carriers
    /// (x), how many symbols the pattern takes to repeat (y), and the
    /// carrier the first of them sits on (k0).
    pub const fn pattern(self) -> (i32, i32, i32) {
        match self {
            Mode::A => (4, 5, 2),
            Mode::B => (2, 3, 1),
            Mode::C => (2, 2, 1),
            Mode::D => (1, 3, 1),
        }
    }

    /// The Q of the pilot phase, clause 8.4.4.3.
    const fn q1024(self) -> i32 {
        match self {
            Mode::A => 36,
            Mode::B => 12,
            Mode::C => 12,
            Mode::D => 14,
        }
    }

    /// Carriers either side of the centre that carry anything, for the
    /// occupancy given. Modes C and D are only defined for the two widest
    /// occupancies and are read at 10 kHz.
    pub fn carriers(self, occ: Occupancy) -> (i32, i32) {
        match self {
            Mode::A => match occ {
                Occupancy::Half45 | Occupancy::Half5 => (2, if occ.narrow() { 102 } else { 114 }),
                Occupancy::Full9 => (-102, 102),
                Occupancy::Full10 => (-114, 114),
            },
            Mode::B => match occ {
                Occupancy::Half45 | Occupancy::Half5 => (1, if occ.narrow() { 91 } else { 103 }),
                Occupancy::Full9 => (-91, 91),
                Occupancy::Full10 => (-103, 103),
            },
            Mode::C => (-69, 69),
            Mode::D => (-44, 44),
        }
    }

    /// The widest carrier a mode ever uses, which is how wide a grid has to
    /// be to hold a symbol before the occupancy is known.
    pub fn span(self) -> i32 {
        self.carriers(Occupancy::Full10).1
    }

    /// Where the multiplex is, for a caller that wants to know how much of
    /// the channel is data rather than pilot.
    pub fn frame_seconds(self) -> f64 {
        self.frame() as f64 / RATE_HZ
    }
}

/// Spectrum occupancy: how wide the transmission is, signalled in the fast
/// access channel.
///
/// The standard has two more, 18 and 20 kHz, for a simulcast that nobody
/// transmits; a receiver told one of those reads nothing rather than
/// guessing, which is what [`Occupancy::from_bits`] returning `None` says.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Occupancy {
    /// 4.5 kHz, the carriers all on one side of the centre.
    Half45,
    /// 5 kHz, likewise.
    Half5,
    /// 9 kHz, a medium wave channel.
    Full9,
    /// 10 kHz, which is what a shortwave broadcast uses.
    Full10,
}

impl Occupancy {
    pub const ALL: [Occupancy; 4] =
        [Occupancy::Half45, Occupancy::Half5, Occupancy::Full9, Occupancy::Full10];

    pub fn from_bits(v: u8) -> Option<Self> {
        match v {
            0 => Some(Occupancy::Half45),
            1 => Some(Occupancy::Half5),
            2 => Some(Occupancy::Full9),
            3 => Some(Occupancy::Full10),
            _ => None,
        }
    }

    pub const fn bits(self) -> u8 {
        match self {
            Occupancy::Half45 => 0,
            Occupancy::Half5 => 1,
            Occupancy::Full9 => 2,
            Occupancy::Full10 => 3,
        }
    }

    /// Whether the narrower of a pair, which the carrier tables need.
    const fn narrow(self) -> bool {
        matches!(self, Occupancy::Half45 | Occupancy::Full9)
    }

    pub const fn width_hz(self) -> f64 {
        match self {
            Occupancy::Half45 => 4_500.0,
            Occupancy::Half5 => 5_000.0,
            Occupancy::Full9 => 9_000.0,
            Occupancy::Full10 => 10_000.0,
        }
    }

    pub const fn label(self) -> &'static str {
        match self {
            Occupancy::Half45 => "4.5 kHz",
            Occupancy::Half5 => "5 kHz",
            Occupancy::Full9 => "9 kHz",
            Occupancy::Full10 => "10 kHz",
        }
    }
}

/// What one cell of the grid carries.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Cell {
    /// Nothing: the centre carrier, and its neighbours in mode A.
    Unused,
    /// A gain reference, which is what the channel is estimated from.
    Gain,
    /// One of the three frequency references, in every symbol.
    Frequency,
    /// A time reference, in the first symbol of a frame only.
    Time,
    /// The fast access channel.
    Fac,
    /// The service description channel.
    Sdc,
    /// The multiplex.
    Msc,
}

/// A phase in 1024ths of a turn, at the amplitude a pilot is boosted to.
fn polar(amplitude: f32, phase1024: i32) -> C32 {
    let theta = TAU * (phase1024.rem_euclid(1024) as f32) / 1024.0;
    C32::new(amplitude * theta.cos(), amplitude * theta.sin())
}

/// The time reference cells of a mode: carrier and phase, clause 8.4.3.1.
fn time_table(mode: Mode) -> &'static [(i32, i32)] {
    match mode {
        Mode::A => &[
            (17, 973),
            (18, 205),
            (19, 717),
            (21, 264),
            (28, 357),
            (29, 357),
            (32, 952),
            (33, 440),
            (39, 856),
            (40, 88),
            (41, 88),
            (53, 68),
            (54, 836),
            (55, 836),
            (56, 836),
            (60, 1008),
            (61, 1008),
            (63, 752),
            (71, 215),
            (72, 215),
            (73, 727),
        ],
        Mode::B => &[
            (14, 304),
            (16, 331),
            (18, 108),
            (20, 620),
            (24, 192),
            (26, 704),
            (32, 44),
            (36, 432),
            (42, 588),
            (44, 844),
            (48, 651),
            (49, 651),
            (50, 651),
            (54, 460),
            (56, 460),
            (62, 944),
            (64, 555),
            (66, 940),
            (68, 428),
        ],
        Mode::C => &[
            (8, 722),
            (10, 466),
            (11, 214),
            (12, 214),
            (14, 479),
            (16, 516),
            (18, 260),
            (22, 577),
            (24, 662),
            (28, 3),
            (30, 771),
            (32, 392),
            (33, 392),
            (36, 37),
            (38, 37),
            (42, 474),
            (44, 242),
            (45, 242),
            (46, 754),
        ],
        Mode::D => &[
            (5, 636),
            (6, 124),
            (7, 788),
            (8, 788),
            (9, 200),
            (11, 688),
            (12, 152),
            (14, 920),
            (15, 920),
            (17, 644),
            (18, 388),
            (20, 652),
            (21, 1014),
            (23, 176),
            (24, 176),
            (26, 752),
            (27, 496),
            (28, 332),
            (29, 432),
            (30, 964),
            (32, 452),
        ],
    }
}

/// The three frequency references, clause 8.4.2: 750, 2 250 and 3 000 Hz,
/// which land on different carriers in each mode.
fn frequency_table(mode: Mode) -> &'static [(i32, i32)] {
    match mode {
        Mode::A => &[(18, 205), (54, 836), (72, 215)],
        Mode::B => &[(16, 331), (48, 651), (64, 555)],
        Mode::C => &[(11, 214), (33, 392), (44, 242)],
        Mode::D => &[(7, 788), (21, 1014), (28, 332)],
    }
}

/// The fast access channel's cells, clause 7.3.2: the symbol and carrier of
/// each of them, in the order the channel's bits are mapped onto them.
pub fn fac_cells(mode: Mode) -> &'static [(usize, i32)] {
    match mode {
        Mode::A => &[
            (2, 26),
            (2, 46),
            (2, 66),
            (2, 86),
            (3, 10),
            (3, 20),
            (3, 50),
            (3, 70),
            (3, 90),
            (4, 14),
            (4, 22),
            (4, 24),
            (4, 62),
            (4, 74),
            (4, 94),
            (5, 26),
            (5, 38),
            (5, 58),
            (5, 66),
            (5, 78),
            (6, 22),
            (6, 30),
            (6, 42),
            (6, 62),
            (6, 70),
            (6, 82),
            (7, 26),
            (7, 34),
            (7, 46),
            (7, 66),
            (7, 74),
            (7, 86),
            (8, 10),
            (8, 30),
            (8, 38),
            (8, 50),
            (8, 58),
            (8, 70),
            (8, 78),
            (8, 90),
            (9, 14),
            (9, 22),
            (9, 34),
            (9, 42),
            (9, 62),
            (9, 74),
            (9, 82),
            (9, 94),
            (10, 26),
            (10, 38),
            (10, 46),
            (10, 66),
            (10, 86),
            (11, 10),
            (11, 30),
            (11, 50),
            (11, 70),
            (11, 90),
            (12, 14),
            (12, 34),
            (12, 74),
            (12, 94),
            (13, 38),
            (13, 58),
            (13, 78),
        ],
        Mode::B => &[
            (2, 13),
            (2, 25),
            (2, 43),
            (2, 55),
            (2, 67),
            (3, 15),
            (3, 27),
            (3, 45),
            (3, 57),
            (3, 69),
            (4, 17),
            (4, 29),
            (4, 47),
            (4, 59),
            (4, 71),
            (5, 19),
            (5, 31),
            (5, 49),
            (5, 61),
            (5, 73),
            (6, 9),
            (6, 21),
            (6, 33),
            (6, 51),
            (6, 63),
            (6, 75),
            (7, 11),
            (7, 23),
            (7, 35),
            (7, 53),
            (7, 65),
            (7, 77),
            (8, 13),
            (8, 25),
            (8, 37),
            (8, 55),
            (8, 67),
            (8, 79),
            (9, 15),
            (9, 27),
            (9, 39),
            (9, 57),
            (9, 69),
            (9, 81),
            (10, 17),
            (10, 29),
            (10, 41),
            (10, 59),
            (10, 71),
            (10, 83),
            (11, 19),
            (11, 31),
            (11, 43),
            (11, 61),
            (11, 73),
            (12, 21),
            (12, 33),
            (12, 45),
            (12, 63),
            (12, 75),
            (13, 23),
            (13, 35),
            (13, 47),
            (13, 65),
            (13, 77),
        ],
        Mode::C => &[
            (3, 9),
            (3, 21),
            (3, 45),
            (3, 57),
            (4, 23),
            (4, 35),
            (4, 47),
            (5, 13),
            (5, 25),
            (5, 37),
            (5, 49),
            (6, 15),
            (6, 27),
            (6, 39),
            (6, 51),
            (7, 5),
            (7, 17),
            (7, 29),
            (7, 41),
            (7, 53),
            (8, 7),
            (8, 19),
            (8, 31),
            (8, 43),
            (8, 55),
            (9, 9),
            (9, 21),
            (9, 45),
            (9, 57),
            (10, 23),
            (10, 35),
            (10, 47),
            (11, 13),
            (11, 25),
            (11, 37),
            (11, 49),
            (12, 15),
            (12, 27),
            (12, 39),
            (12, 51),
            (13, 5),
            (13, 17),
            (13, 29),
            (13, 41),
            (13, 53),
            (14, 7),
            (14, 19),
            (14, 31),
            (14, 43),
            (14, 55),
            (15, 9),
            (15, 21),
            (15, 45),
            (15, 57),
            (16, 23),
            (16, 35),
            (16, 47),
            (17, 13),
            (17, 25),
            (17, 37),
            (17, 49),
            (18, 15),
            (18, 27),
            (18, 39),
            (18, 51),
        ],
        Mode::D => &[
            (3, 9),
            (3, 18),
            (3, 27),
            (4, 10),
            (4, 19),
            (5, 11),
            (5, 20),
            (5, 29),
            (6, 12),
            (6, 30),
            (7, 13),
            (7, 22),
            (7, 31),
            (8, 5),
            (8, 14),
            (8, 23),
            (8, 32),
            (9, 6),
            (9, 15),
            (9, 24),
            (9, 33),
            (10, 16),
            (10, 25),
            (10, 34),
            (11, 8),
            (11, 17),
            (11, 26),
            (11, 35),
            (12, 9),
            (12, 18),
            (12, 27),
            (12, 36),
            (13, 10),
            (13, 19),
            (13, 37),
            (14, 11),
            (14, 20),
            (14, 29),
            (15, 12),
            (15, 30),
            (16, 13),
            (16, 22),
            (16, 31),
            (17, 5),
            (17, 14),
            (17, 23),
            (17, 32),
            (18, 6),
            (18, 15),
            (18, 24),
            (18, 33),
            (19, 16),
            (19, 25),
            (19, 34),
            (20, 8),
            (20, 17),
            (20, 26),
            (20, 35),
            (21, 9),
            (21, 18),
            (21, 27),
            (21, 36),
            (22, 10),
            (22, 19),
            (22, 37),
        ],
    }
}

/// W and Z of the pilot phase, clause 8.4.4.3.1, indexed by the symbol's
/// place in the pattern (n) and which repeat of it (m).
fn w1024(mode: Mode, n: usize, m: usize) -> i32 {
    const A: [i32; 15] = [228, 341, 455, 455, 569, 683, 683, 796, 910, 910, 0, 114, 114, 228, 341];
    const B: [i32; 15] = [512, 0, 512, 0, 512, 0, 512, 0, 512, 0, 512, 0, 512, 0, 512];
    const C: [i32; 20] = [
        465, 372, 279, 186, 93, 0, 931, 838, 745, 652, 931, 838, 745, 652, 559, 465, 372, 279, 186,
        93,
    ];
    const D: [i32; 24] = [
        366, 439, 512, 585, 658, 731, 805, 676, 731, 805, 979, 951, 0, 73, 146, 219, 73, 146, 219,
        293, 366, 439, 512, 585,
    ];
    match mode {
        Mode::A => A[3 * n + m],
        Mode::B => B[5 * n + m],
        Mode::C => C[10 * n + m],
        Mode::D => D[8 * n + m],
    }
}

fn z256(mode: Mode, n: usize, m: usize) -> i32 {
    const A: [i32; 15] = [0, 81, 248, 18, 106, 106, 122, 116, 31, 129, 129, 39, 33, 32, 111];
    const B: [i32; 15] = [0, 57, 164, 64, 12, 168, 255, 161, 106, 118, 25, 232, 132, 233, 38];
    const C: [i32; 20] =
        [0, 76, 29, 76, 9, 190, 161, 248, 33, 108, 179, 178, 83, 253, 127, 105, 101, 198, 250, 145];
    const D: [i32; 24] = [
        0, 240, 17, 60, 220, 38, 151, 101, 110, 7, 78, 82, 175, 150, 106, 25, 165, 7, 252, 124,
        253, 177, 197, 142,
    ];
    match mode {
        Mode::A => A[3 * n + m],
        Mode::B => B[5 * n + m],
        Mode::C => C[10 * n + m],
        Mode::D => D[8 * n + m],
    }
}

/// Whether a cell is one of the four whose power is doubled at the edge of
/// the transmission, clause 8.4.4.2. Which they are depends on where the
/// edge is, so on the occupancy.
fn boosted(mode: Mode, occ: Occupancy, symbol: usize, k: i32) -> bool {
    match mode {
        Mode::A => match occ {
            Occupancy::Half45 => [2, 6, 98, 102].contains(&k),
            Occupancy::Half5 => [2, 6, 110, 114].contains(&k),
            Occupancy::Full9 => [-102, -98, 98, 102].contains(&k),
            Occupancy::Full10 => [-114, -110, 110, 114].contains(&k),
        },
        Mode::B => match occ {
            Occupancy::Half45 => [1, 3, 89, 91].contains(&k),
            Occupancy::Half5 => [1, 3, 101, 103].contains(&k),
            Occupancy::Full9 => [-91, -89, 89, 91].contains(&k),
            Occupancy::Full10 => [-103, -101, 101, 103].contains(&k),
        },
        // Mode C alternates which pair of the four is boosted, symbol by
        // symbol.
        Mode::C => {
            if symbol.is_multiple_of(2) {
                k == -69 || k == 67
            } else {
                k == -67 || k == 69
            }
        }
        Mode::D => [-44, -43, 43, 44].contains(&k),
    }
}

/// Whether a cell carries a gain reference, clause 8.4.4.1.
pub fn is_gain(mode: Mode, symbol: usize, k: i32) -> bool {
    if k == 0 {
        return false;
    }
    let (x, y, k0) = mode.pattern();
    let n = (symbol as i32) % y;
    (k - k0 - n * x).rem_euclid(x * y) == 0
}

/// What a pilot cell holds: the gain reference's phase, clause 8.4.4.3.
pub fn gain_value(mode: Mode, occ: Occupancy, symbol: usize, k: i32) -> C32 {
    let (x, y, k0) = mode.pattern();
    let n = (symbol as i32 % y) as usize;
    let m = symbol / y as usize;
    let p = (k - k0 - n as i32 * x) / (x * y);
    let phase =
        4 * z256(mode, n, m) + p * w1024(mode, n, m) + p * p * (1 + symbol as i32) * mode.q1024();
    // Two, or four at the four cells that hold the edge of the spectrum up.
    let amplitude = if boosted(mode, occ, symbol, k) { 4.0f32 } else { 2.0f32 };
    polar(amplitude.sqrt(), phase)
}

/// What is at one place on the grid.
pub fn cell(mode: Mode, occ: Occupancy, symbol: usize, k: i32) -> Cell {
    let (lo, hi) = mode.carriers(occ);
    if k < lo || k > hi || k == 0 || (mode == Mode::A && k.abs() == 1) {
        return Cell::Unused;
    }
    if frequency_table(mode).iter().any(|&(c, _)| c == k) {
        return Cell::Frequency;
    }
    if symbol == 0 && time_table(mode).iter().any(|&(c, _)| c == k) {
        return Cell::Time;
    }
    if is_gain(mode, symbol, k) {
        return Cell::Gain;
    }
    if fac_cells(mode).iter().any(|&(s, c)| s == symbol && c == k) {
        return Cell::Fac;
    }
    if symbol < mode.sdc_symbols() {
        return Cell::Sdc;
    }
    Cell::Msc
}

/// What a reference cell holds, or `None` where the cell carries data.
pub fn reference(mode: Mode, occ: Occupancy, symbol: usize, k: i32) -> Option<C32> {
    match cell(mode, occ, symbol, k) {
        Cell::Gain => Some(gain_value(mode, occ, symbol, k)),
        Cell::Frequency => frequency_table(mode)
            .iter()
            .find(|&&(c, _)| c == k)
            .map(|&(_, phase)| polar(2f32.sqrt(), phase)),
        Cell::Time => time_table(mode)
            .iter()
            .find(|&&(c, _)| c == k)
            .map(|&(_, phase)| polar(2f32.sqrt(), phase)),
        _ => None,
    }
}

/// Where the service description channel's cells are, in the order its bits
/// are mapped onto them: symbol by symbol, carrier by carrier.
pub fn sdc_cells(mode: Mode, occ: Occupancy) -> Vec<(usize, i32)> {
    let (lo, hi) = mode.carriers(occ);
    let mut out = Vec::new();
    for symbol in 0..mode.sdc_symbols() {
        for k in lo..=hi {
            if cell(mode, occ, symbol, k) == Cell::Sdc {
                out.push((symbol, k));
            }
        }
    }
    out
}

/// The time references of a frame's first symbol, for a receiver looking for
/// where a frame begins.
pub fn time_references(mode: Mode) -> Vec<(i32, C32)> {
    time_table(mode).iter().map(|&(k, phase)| (k, polar(2f32.sqrt(), phase))).collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Every mode's frame is 400 ms of 12 kS/s.
    #[test]
    fn a_frame_is_four_thousand_eight_hundred_samples() {
        for mode in Mode::ALL {
            assert_eq!(mode.frame(), 4_800, "mode {}", mode.label());
            assert!((mode.frame_seconds() - FRAME_SECONDS).abs() < 1e-9);
        }
        assert_eq!(Mode::B.spacing_hz(), 46.875);
    }

    /// The fast access channel is 65 cells in modes A and B and 64 in C and
    /// D, which is what its coding rate turns 72 bits into.
    #[test]
    fn the_fast_access_channel_has_its_own_cells() {
        for mode in Mode::ALL {
            assert_eq!(fac_cells(mode).len(), 65, "mode {}", mode.label());
        }
        for mode in Mode::ALL {
            for &(symbol, k) in fac_cells(mode) {
                assert_eq!(
                    cell(mode, Occupancy::Full10, symbol, k),
                    Cell::Fac,
                    "mode {} cell {symbol}/{k}",
                    mode.label()
                );
            }
        }
    }

    /// The counts the standard's table 55 gives for the service description
    /// channel, which is how many cells are left of the first symbols.
    #[test]
    fn the_description_channel_is_what_the_first_symbols_have_left() {
        assert_eq!(sdc_cells(Mode::A, Occupancy::Half45).len(), 167);
        assert_eq!(sdc_cells(Mode::A, Occupancy::Half5).len(), 190);
        assert_eq!(sdc_cells(Mode::A, Occupancy::Full9).len(), 359);
        assert_eq!(sdc_cells(Mode::A, Occupancy::Full10).len(), 405);
        assert_eq!(sdc_cells(Mode::B, Occupancy::Half45).len(), 130);
        assert_eq!(sdc_cells(Mode::B, Occupancy::Half5).len(), 150);
        assert_eq!(sdc_cells(Mode::B, Occupancy::Full9).len(), 282);
        assert_eq!(sdc_cells(Mode::B, Occupancy::Full10).len(), 322);
        assert_eq!(sdc_cells(Mode::C, Occupancy::Full10).len(), 288);
        assert_eq!(sdc_cells(Mode::D, Occupancy::Full10).len(), 152);
    }

    /// A pilot is a pilot in every mode on the pattern the standard gives,
    /// and no cell is two things at once.
    #[test]
    fn the_pilots_land_where_the_pattern_says() {
        for mode in Mode::ALL {
            let (x, y, k0) = mode.pattern();
            let mut pilots = 0;
            for symbol in 0..mode.symbols() {
                for k in mode.carriers(Occupancy::Full10).0..=mode.carriers(Occupancy::Full10).1 {
                    if cell(mode, Occupancy::Full10, symbol, k) == Cell::Gain {
                        pilots += 1;
                        assert_eq!((k - k0 - (symbol as i32 % y) * x).rem_euclid(x * y), 0);
                        assert!(reference(mode, Occupancy::Full10, symbol, k).is_some());
                    }
                }
            }
            // A frame's gain references, counted over the widest occupancy:
            // one carrier in twenty for mode A up to every carrier of every
            // third symbol for mode D.
            let want = match mode {
                Mode::A => 168,
                Mode::B => 519,
                Mode::C => 679,
                Mode::D => 680,
            };
            assert_eq!(pilots, want, "mode {}", mode.label());
        }
    }

    /// The boosted cells carry twice the power of an ordinary pilot, which
    /// is what holds the edge of the spectrum up.
    #[test]
    fn the_edge_pilots_are_boosted() {
        let plain = gain_value(Mode::B, Occupancy::Full10, 0, 7).norm();
        let edge = gain_value(Mode::B, Occupancy::Full10, 0, 103).norm();
        assert!((plain - 2f32.sqrt()).abs() < 1e-5, "{plain}");
        assert!((edge - 2.0).abs() < 1e-5, "{edge}");
    }
}
