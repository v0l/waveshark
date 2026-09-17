//! What an RTL2832U's tuner is, and what it will do.
//!
//! Here rather than in `crates/rtlsdr` because the same tuner is reached two
//! ways: over USB through librtlsdr, and over TCP through `rtl_tcp`, which
//! reports the tuner as a number in its header and nothing else. Both need
//! the same table and neither owns it.

use crate::device::TunerRange;
use crate::units::{Hz, Sps};

/// The tuner chip in front of the RTL2832U, as librtlsdr numbers them.
///
/// The numbering is `enum rtlsdr_tuner` in librtlsdr and is also what
/// `rtl_tcp` puts in its greeting, so the codes are wire values.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Tuner {
    Unknown,
    E4000,
    Fc0012,
    Fc0013,
    Fc2580,
    R820t,
    R828d,
}

impl Tuner {
    pub fn from_code(code: u32) -> Self {
        match code {
            1 => Self::E4000,
            2 => Self::Fc0012,
            3 => Self::Fc0013,
            4 => Self::Fc2580,
            5 => Self::R820t,
            6 => Self::R828d,
            _ => Self::Unknown,
        }
    }

    pub fn name(self) -> &'static str {
        match self {
            Self::Unknown => "unknown",
            Self::E4000 => "E4000",
            Self::Fc0012 => "FC0012",
            Self::Fc0013 => "FC0013",
            Self::Fc2580 => "FC2580",
            Self::R820t => "R820T",
            Self::R828d => "R828D",
        }
    }

    /// Tunable spans, in hertz. The manufacturer figures librtlsdr will
    /// actually accept, not the optimistic datasheet ones.
    pub fn ranges(self) -> Vec<TunerRange> {
        match self {
            // E4000 has a genuine hole around the 1100-1250 MHz IF region.
            Self::E4000 => vec![
                TunerRange { range: Hz::mhz(52)..=Hz::mhz(1100), label: "low" },
                TunerRange { range: Hz::mhz(1250)..=Hz::mhz(2200), label: "high" },
            ],
            Self::R820t | Self::R828d => {
                vec![TunerRange { range: Hz::mhz(24)..=Hz::mhz(1766), label: "main" }]
            }
            Self::Unknown | Self::Fc0012 | Self::Fc0013 | Self::Fc2580 => {
                vec![TunerRange { range: Hz::mhz(22)..=Hz::mhz(1100), label: "main" }]
            }
        }
    }

    /// The widest gain the tuner offers, in dB. Used where the gain table
    /// itself cannot be read back, as over `rtl_tcp`, which sends a count of
    /// steps and not their values.
    pub fn max_gain_db(self) -> f32 {
        match self {
            Self::R820t | Self::R828d => 49.6,
            Self::E4000 => 42.0,
            Self::Fc0012 | Self::Fc0013 => 19.7,
            Self::Fc2580 => 0.0,
            Self::Unknown => 49.6,
        }
    }
}

/// The RTL2832U accepts 225001-300000 and 900001-3200000 S/s, but above
/// 2.4 MS/s most USB 2.0 host controllers cannot sustain the bulk rate and you
/// get silent sample loss. These are the rates worth offering.
pub const RATES: [Sps; 8] = [
    Sps(240_000),
    Sps(960_000),
    Sps(1_024_000),
    Sps(1_200_000),
    Sps(2_048_000),
    Sps(2_400_000),
    Sps(2_560_000),
    Sps(3_200_000),
];

pub const RATE_RANGE: std::ops::RangeInclusive<Sps> = Sps(225_001)..=Sps(3_200_000);

/// The RTL2832U has no analogue anti-alias filter worth the name; the outer
/// ~20% of the span is contaminated by the decimation filter's transition and
/// by the DC spur's skirt.
pub const USABLE_BANDWIDTH_RATIO: f32 = 0.80;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_tuner_codes_are_librtlsdrs() {
        assert_eq!(Tuner::from_code(5), Tuner::R820t);
        assert_eq!(Tuner::from_code(1), Tuner::E4000);
        assert_eq!(Tuner::from_code(0), Tuner::Unknown);
        assert_eq!(Tuner::from_code(99), Tuner::Unknown);
        assert_eq!(Tuner::R828d.name(), "R828D");
    }

    #[test]
    fn the_e4000_has_a_hole_and_the_r820t_does_not() {
        let e = Tuner::E4000.ranges();
        assert_eq!(e.len(), 2);
        assert!(!e.iter().any(|r| r.range.contains(&Hz::mhz(1200))));
        let r = Tuner::R820t.ranges();
        assert_eq!(r.len(), 1);
        assert!(r[0].range.contains(&Hz::mhz(1090)));
        assert!(!r[0].range.contains(&Hz::mhz(1800)));
    }
}
