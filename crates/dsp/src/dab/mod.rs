//! DAB, the broadcast radio multiplex ETSI EN 300 401 describes: an ensemble
//! of 1.536 MHz carrying a dozen stations, their names and their programme
//! types.
//!
//! Everything here is at the standard's own rate of 2.048 MS/s, whatever the
//! transmission mode. A frame opens with a null symbol, which is the
//! transmitter going quiet for about a symbol and a quarter and is what a
//! receiver finds a frame by; then a phase reference symbol, whose value the
//! standard writes out carrier by carrier; then the symbols carrying the fast
//! information channel and the main service channel.
//!
//! The modulation is differential QPSK, carrier by carrier against the same
//! carrier of the symbol before, so there are no pilots and no channel
//! estimate: a multipath echo or a timing offset shows up as a phase that
//! both symbols share and the difference cancels. That is the whole reason
//! this front end is shorter than the DVB-T one beside it.
//!
//! The layer above is `decode::dab`, which takes the soft bits of the fast
//! information channel through the convolutional code to the ensemble's own
//! tables. Nothing here knows what a service is.

pub mod rx;
pub mod tx;

use common::C32;
use std::f32::consts::PI;

pub use rx::{Dab, Symbol};

/// The sampling rate every transmission mode is defined at.
pub const RATE_HZ: f64 = 2_048_000.0;
/// What the carriers occupy: 1536 of them, 1 kHz apart, in Mode I.
pub const CHANNEL_WIDTH_HZ: f64 = 1_536_000.0;
/// The raster the band III blocks are on, 1.712 MHz apart, which is what an
/// ensemble is allocated even though it transmits 1.536 MHz of it.
pub const BLOCK_RASTER_HZ: f64 = 1_712_000.0;

/// The transmission mode, which is one set of frame, symbol and carrier
/// lengths. Europe transmits Mode I from high towers; the others are for
/// cable, satellite and gap fillers at higher frequencies.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Mode {
    I,
    II,
    III,
    IV,
}

impl Mode {
    pub const ALL: [Mode; 4] = [Mode::I, Mode::II, Mode::III, Mode::IV];

    /// The useful part of a symbol, T_u, which is the FFT size.
    pub const fn fft(self) -> usize {
        match self {
            Mode::I => 2048,
            Mode::II => 512,
            Mode::III => 256,
            Mode::IV => 1024,
        }
    }

    /// The cyclic prefix, T_g.
    pub const fn guard(self) -> usize {
        match self {
            Mode::I => 504,
            Mode::II => 126,
            Mode::III => 63,
            Mode::IV => 252,
        }
    }

    /// A whole symbol, prefix included.
    pub const fn symbol(self) -> usize {
        self.fft() + self.guard()
    }

    /// The null symbol, T_null, which is longer than a symbol so that it can
    /// be found without knowing where the symbols are.
    pub const fn null(self) -> usize {
        match self {
            Mode::I => 2656,
            Mode::II => 664,
            Mode::III => 345,
            Mode::IV => 1328,
        }
    }

    /// Transmitted carriers, K.
    pub const fn carriers(self) -> usize {
        match self {
            Mode::I => 1536,
            Mode::II => 384,
            Mode::III => 192,
            Mode::IV => 768,
        }
    }

    /// Symbols in a frame after the null, L. The first of them is the phase
    /// reference and carries no bits.
    pub const fn symbols(self) -> usize {
        match self {
            Mode::I => 76,
            Mode::II => 76,
            Mode::III => 153,
            Mode::IV => 76,
        }
    }

    /// Symbols the fast information channel occupies, which follow the phase
    /// reference. The main service channel has the rest.
    pub const fn fic_symbols(self) -> usize {
        match self {
            Mode::I => 3,
            Mode::II => 3,
            Mode::III => 8,
            Mode::IV => 3,
        }
    }

    /// Fast information blocks a frame carries.
    pub const fn fibs(self) -> usize {
        match self {
            Mode::I => 12,
            Mode::II => 3,
            Mode::III => 4,
            Mode::IV => 6,
        }
    }

    /// Samples in one transmission frame, T_F.
    pub const fn frame(self) -> usize {
        self.null() + self.symbols() * self.symbol()
    }

    /// Carrier spacing, which is the inverse of the useful part.
    pub const fn spacing_hz(self) -> f64 {
        RATE_HZ / self.fft() as f64
    }

    /// The frame repetition rate, in seconds.
    pub fn frame_seconds(self) -> f64 {
        self.frame() as f64 / RATE_HZ
    }

    pub const fn label(self) -> &'static str {
        match self {
            Mode::I => "I",
            Mode::II => "II",
            Mode::III => "III",
            Mode::IV => "IV",
        }
    }

    /// The seed of the frequency interleaver's generator, EN 300 401 clause
    /// 14.6.
    const fn interleave_seed(self) -> usize {
        match self {
            Mode::I => 511,
            Mode::II => 127,
            Mode::III => 63,
            Mode::IV => 255,
        }
    }
}

/// The FFT bin a carrier sits in. Carriers run from -K/2 to K/2 with the
/// centre one left out, because a transmitter's own carrier leak would land
/// on it.
pub const fn bin(mode: Mode, carrier: i32) -> usize {
    (carrier + mode.fft() as i32) as usize % mode.fft()
}

/// The frequency interleaving, EN 300 401 clause 14.6: bit position `n` of a
/// symbol to the carrier that bit is transmitted on.
///
/// The generator walks the whole FFT with `13 x + V1` and what it visits
/// inside the transmitted band, in that order, is the mapping. A code that
/// cannot correct a run of errors is why it is there: a notch takes out
/// neighbouring carriers, and neighbouring carriers are far apart in the
/// coded stream.
pub fn interleave(mode: Mode) -> Vec<i32> {
    let n = mode.fft();
    let lwb = n / 2 - mode.carriers() / 2;
    let upb = lwb + mode.carriers();
    let mut out = Vec::with_capacity(mode.carriers());
    let mut x = 0usize;
    for i in 0..n {
        if i > 0 {
            x = (13 * x + mode.interleave_seed()) % n;
        }
        if x == n / 2 || x < lwb || x > upb {
            continue;
        }
        out.push(x as i32 - n as i32 / 2);
    }
    debug_assert_eq!(out.len(), mode.carriers());
    out
}

/// One row of the phase reference table, EN 300 401 table 23: a run of
/// thirty-two carriers built from row `i` of the h table with `n` added.
struct Row {
    k_min: i32,
    i: usize,
    n: i32,
}

/// EN 300 401 table 22, the four h sequences of the phase reference symbol.
const H: [[i32; 32]; 4] = [
    [
        0, 2, 0, 0, 0, 0, 1, 1, 2, 0, 0, 0, 2, 2, 1, 1, 0, 2, 0, 0, 0, 0, 1, 1, 2, 0, 0, 0, 2, 2,
        1, 1,
    ],
    [
        0, 3, 2, 3, 0, 1, 3, 0, 2, 1, 2, 3, 2, 3, 3, 0, 0, 3, 2, 3, 0, 1, 3, 0, 2, 1, 2, 3, 2, 3,
        3, 0,
    ],
    [
        0, 0, 0, 2, 0, 2, 1, 3, 2, 2, 0, 2, 2, 0, 1, 3, 0, 0, 0, 2, 0, 2, 1, 3, 2, 2, 0, 2, 2, 0,
        1, 3,
    ],
    [
        0, 1, 2, 1, 0, 3, 3, 2, 2, 3, 2, 1, 2, 1, 3, 2, 0, 1, 2, 1, 0, 3, 3, 2, 2, 3, 2, 1, 2, 1,
        3, 2,
    ],
];

/// EN 300 401 table 23, the Mode I phase reference symbol: for each run of
/// thirty-two carriers, which h sequence it takes and what is added to it.
const PRS_I: [Row; 48] = [
    Row { k_min: -768, i: 0, n: 1 },
    Row { k_min: -736, i: 1, n: 2 },
    Row { k_min: -704, i: 2, n: 0 },
    Row { k_min: -672, i: 3, n: 1 },
    Row { k_min: -640, i: 0, n: 3 },
    Row { k_min: -608, i: 1, n: 2 },
    Row { k_min: -576, i: 2, n: 2 },
    Row { k_min: -544, i: 3, n: 3 },
    Row { k_min: -512, i: 0, n: 2 },
    Row { k_min: -480, i: 1, n: 1 },
    Row { k_min: -448, i: 2, n: 2 },
    Row { k_min: -416, i: 3, n: 3 },
    Row { k_min: -384, i: 0, n: 1 },
    Row { k_min: -352, i: 1, n: 2 },
    Row { k_min: -320, i: 2, n: 3 },
    Row { k_min: -288, i: 3, n: 3 },
    Row { k_min: -256, i: 0, n: 2 },
    Row { k_min: -224, i: 1, n: 2 },
    Row { k_min: -192, i: 2, n: 2 },
    Row { k_min: -160, i: 3, n: 1 },
    Row { k_min: -128, i: 0, n: 1 },
    Row { k_min: -96, i: 1, n: 3 },
    Row { k_min: -64, i: 2, n: 1 },
    Row { k_min: -32, i: 3, n: 2 },
    Row { k_min: 1, i: 0, n: 3 },
    Row { k_min: 33, i: 3, n: 1 },
    Row { k_min: 65, i: 2, n: 1 },
    Row { k_min: 97, i: 1, n: 1 },
    Row { k_min: 129, i: 0, n: 2 },
    Row { k_min: 161, i: 3, n: 2 },
    Row { k_min: 193, i: 2, n: 1 },
    Row { k_min: 225, i: 1, n: 0 },
    Row { k_min: 257, i: 0, n: 2 },
    Row { k_min: 289, i: 3, n: 2 },
    Row { k_min: 321, i: 2, n: 3 },
    Row { k_min: 353, i: 1, n: 3 },
    Row { k_min: 385, i: 0, n: 0 },
    Row { k_min: 417, i: 3, n: 2 },
    Row { k_min: 449, i: 2, n: 1 },
    Row { k_min: 481, i: 1, n: 3 },
    Row { k_min: 513, i: 0, n: 3 },
    Row { k_min: 545, i: 3, n: 3 },
    Row { k_min: 577, i: 2, n: 3 },
    Row { k_min: 609, i: 1, n: 0 },
    Row { k_min: 641, i: 0, n: 3 },
    Row { k_min: 673, i: 3, n: 0 },
    Row { k_min: 705, i: 2, n: 1 },
    Row { k_min: 737, i: 1, n: 1 },
];

/// The phase reference symbol of `mode`, by FFT bin, or `None` where the
/// standard's table for that mode is not here.
///
/// Only Mode I is written out. The receiver needs this to time a frame
/// exactly and the transmitter to send one, and neither can be asked to
/// guess a table: a mode without its table is a mode this cannot read.
pub fn phase_reference(mode: Mode) -> Option<Vec<C32>> {
    let rows: &[Row] = match mode {
        Mode::I => &PRS_I,
        Mode::II | Mode::III | Mode::IV => return None,
    };
    let mut out = vec![C32::default(); mode.fft()];
    let half = mode.carriers() as i32 / 2;
    for k in -half..=half {
        if k == 0 {
            continue;
        }
        let row = rows.iter().find(|r| k >= r.k_min && k < r.k_min + 32)?;
        let phi = PI / 2.0 * (H[row.i][(k - row.k_min) as usize] + row.n) as f32;
        out[bin(mode, k)] = C32::new(phi.cos(), phi.sin());
    }
    Some(out)
}

/// The QPSK point a pair of bits makes, EN 300 401 clause 14.5: a zero bit is
/// the positive side of its axis, the first bit of the pair carries the real
/// axis and the second the imaginary.
pub fn map(p: u8, q: u8) -> C32 {
    const A: f32 = std::f32::consts::FRAC_1_SQRT_2;
    C32::new(A * (1.0 - 2.0 * p as f32), A * (1.0 - 2.0 * q as f32))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// EN 300 401 table 38 gives the frame length of each mode directly, and
    /// it has to come out of the null and the symbols that follow it.
    #[test]
    fn a_frame_is_the_length_the_standard_gives() {
        assert_eq!(Mode::I.frame(), 196_608);
        assert_eq!(Mode::II.frame(), 49_152);
        assert_eq!(Mode::III.frame(), 49_152);
        assert_eq!(Mode::IV.frame(), 98_304);
        assert!((Mode::I.frame_seconds() - 0.096).abs() < 1e-9);
        assert!((Mode::I.spacing_hz() - 1000.0).abs() < 1e-9);
        assert!((Mode::II.spacing_hz() - 4000.0).abs() < 1e-9);
    }

    /// A symbol carries two bits per carrier, and the fast information
    /// channel's symbols have to hold exactly the frame's fast information
    /// blocks at the rate 1/3 the coding leaves.
    #[test]
    fn the_fic_symbols_hold_the_frames_fibs() {
        for mode in Mode::ALL {
            let bits = mode.fic_symbols() * 2 * mode.carriers();
            assert_eq!(
                bits,
                mode.fibs() * 256 * 3,
                "mode {} carries {} FIC bits for {} FIBs",
                mode.label(),
                bits,
                mode.fibs()
            );
        }
    }

    /// The interleaver is a permutation: every carrier once, none left out,
    /// and never the centre one.
    #[test]
    fn the_frequency_interleaver_is_a_permutation() {
        for mode in Mode::ALL {
            let map = interleave(mode);
            assert_eq!(map.len(), mode.carriers());
            let mut seen = map.clone();
            seen.sort_unstable();
            seen.dedup();
            assert_eq!(seen.len(), mode.carriers(), "mode {} repeats a carrier", mode.label());
            let half = mode.carriers() as i32 / 2;
            assert!(map.iter().all(|&k| k != 0 && k >= -half && k <= half));
            // The Mode I generator starts at zero, which is outside the
            // transmitted band and thrown away, so the first bit of a symbol
            // goes on carrier 511 - 1024.
            if mode == Mode::I {
                assert_eq!(map[0], -513);
                assert_eq!(map[1], -14);
            }
        }
    }

    /// Every carrier of the phase reference symbol is on the unit circle at
    /// one of the four quarter turns, and the centre carrier is empty.
    #[test]
    fn the_phase_reference_is_unit_qpsk() {
        let prs = phase_reference(Mode::I).expect("Mode I has its table");
        assert_eq!(prs.iter().filter(|c| c.norm() > 0.5).count(), Mode::I.carriers());
        assert_eq!(prs[0], C32::default());
        for c in prs.iter().filter(|c| c.norm() > 0.5) {
            assert!((c.norm() - 1.0).abs() < 1e-5);
            let turns = c.arg() / (PI / 2.0);
            assert!((turns - turns.round()).abs() < 1e-4, "{c} is not a quarter turn");
        }
        // Carrier -768 is the first entry of h0 with n = 1, so a quarter
        // turn: j.
        let first = prs[bin(Mode::I, -768)];
        assert!((first - C32::new(0.0, 1.0)).norm() < 1e-5, "{first}");
        assert!(phase_reference(Mode::II).is_none());
    }

    /// A zero bit is the positive side of its axis, and the four points are
    /// unit power.
    #[test]
    fn the_qpsk_mapping_puts_a_zero_bit_positive() {
        let a = std::f32::consts::FRAC_1_SQRT_2;
        assert!((map(0, 0) - C32::new(a, a)).norm() < 1e-6);
        assert!((map(1, 0) - C32::new(-a, a)).norm() < 1e-6);
        assert!((map(0, 1) - C32::new(a, -a)).norm() < 1e-6);
        for (p, q) in [(0, 0), (0, 1), (1, 0), (1, 1)] {
            assert!((map(p, q).norm() - 1.0).abs() < 1e-6);
        }
    }
}
