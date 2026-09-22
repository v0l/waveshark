//! What an RTL2832U's tuner is, and what it will do.
//!
//! Here rather than in `crates/rtlsdr` because the same tuner is reached two
//! ways: over USB through librtlsdr, and over TCP through `rtl_tcp`, which
//! reports the tuner as a number in its header and nothing else. Both need
//! the same table and neither owns it.

use crate::device::{Toggle, TunerRange};
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

/// A switch an RTL2832U offers beyond gain and tuning.
///
/// The set is the same whichever way the dongle is reached, so the name an
/// operator's saved setting carries, the caption beside it and the warning
/// under it are written once here and read by the USB driver and by
/// `rtl_tcp`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Switch {
    RtlAgc,
    BiasTee,
    DirectSampling,
    OffsetTuning,
}

/// Every switch, in the order a panel draws them.
pub const SWITCHES: [Switch; 4] =
    [Switch::RtlAgc, Switch::BiasTee, Switch::DirectSampling, Switch::OffsetTuning];

impl Switch {
    /// The name a setting is saved and asked for under.
    pub fn name(self) -> &'static str {
        match self {
            Self::RtlAgc => "rtl_agc",
            Self::BiasTee => "bias_tee",
            Self::DirectSampling => "direct_sampling",
            Self::OffsetTuning => "offset_tuning",
        }
    }

    pub fn from_name(name: &str) -> Option<Self> {
        SWITCHES.into_iter().find(|s| s.name() == name)
    }

    pub fn label(self) -> &'static str {
        match self {
            Self::RtlAgc => "RTL2832U digital AGC",
            Self::BiasTee => "Bias tee",
            Self::DirectSampling => "Direct sampling (HF)",
            Self::OffsetTuning => "Offset tuning",
        }
    }

    pub fn help(self) -> &'static str {
        match self {
            Self::RtlAgc => {
                "Gain control in the demodulator chip, after the tuner. Recovers a weak signal on a quiet band, and ruins wideband work: the noise floor moves under you and every level measurement moves with it."
            }
            Self::BiasTee => {
                "Puts 4.5 V on the antenna socket to power a mast head amplifier. Leave it off unless you know what is on the other end of the cable, because a shorted or DC coupled antenna takes the current."
            }
            Self::DirectSampling => {
                "Bypasses the tuner and samples the Q branch directly, which reaches below the tuner's 24 MHz floor on a v3 dongle. Everything above about 14 MHz aliases, and the tuner gain does nothing while it is on."
            }
            Self::OffsetTuning => {
                "Moves the tuner's own oscillator off the centre so the DC spike falls outside the span. The E4000 is the only tuner that does it; a zero IF tuner ignores the request."
            }
        }
    }

    /// Whether this tuner does anything when the switch is thrown.
    ///
    /// Offset tuning is an E4000 register write and librtlsdr refuses it for
    /// every other tuner, so offering it on an R820T would be a switch that
    /// moves nothing.
    pub fn applies_to(self, tuner: Tuner) -> bool {
        match self {
            Self::OffsetTuning => tuner == Tuner::E4000,
            _ => true,
        }
    }
}

/// What an RTL2832U's switches are set to.
///
/// A dongle reads none of these back, over USB or over the network, so what
/// it is set to is what it was last told.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Switches {
    pub rtl_agc: bool,
    pub bias_tee: bool,
    pub direct_sampling: bool,
    pub offset_tuning: bool,
}

impl Switches {
    pub fn get(&self, which: Switch) -> bool {
        match which {
            Switch::RtlAgc => self.rtl_agc,
            Switch::BiasTee => self.bias_tee,
            Switch::DirectSampling => self.direct_sampling,
            Switch::OffsetTuning => self.offset_tuning,
        }
    }

    pub fn set(&mut self, which: Switch, on: bool) {
        match which {
            Switch::RtlAgc => self.rtl_agc = on,
            Switch::BiasTee => self.bias_tee = on,
            Switch::DirectSampling => self.direct_sampling = on,
            Switch::OffsetTuning => self.offset_tuning = on,
        }
    }

    /// The switches a driver says it can drive, in the order a panel draws
    /// them.
    pub fn toggles(&self, offered: &[Switch]) -> Vec<Toggle> {
        offered
            .iter()
            .copied()
            .map(|s| Toggle {
                name: s.name().to_string(),
                label: s.label().to_string(),
                help: s.help().to_string(),
                on: self.get(s),
            })
            .collect()
    }
}

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

    #[test]
    fn a_switch_is_named_the_same_whichever_way_the_dongle_is_reached() {
        assert_eq!(SWITCHES.len(), 4);
        for s in SWITCHES {
            assert_eq!(Switch::from_name(s.name()), Some(s));
            assert!(!s.label().is_empty());
            assert!(s.help().len() > 40, "{} says too little", s.name());
        }
        assert_eq!(Switch::from_name("antenna"), None);
    }

    #[test]
    fn only_the_e4000_offers_offset_tuning() {
        assert!(Switch::OffsetTuning.applies_to(Tuner::E4000));
        assert!(!Switch::OffsetTuning.applies_to(Tuner::R820t));
        assert!(!Switch::OffsetTuning.applies_to(Tuner::R828d));
        for s in [Switch::RtlAgc, Switch::BiasTee, Switch::DirectSampling] {
            assert!(s.applies_to(Tuner::R820t));
            assert!(s.applies_to(Tuner::E4000));
        }
    }

    #[test]
    fn switches_report_what_they_were_last_told() {
        let mut s = Switches::default();
        assert_eq!(s.toggles(&SWITCHES).len(), 4);
        assert!(s.toggles(&SWITCHES).iter().all(|t| !t.on));
        s.set(Switch::BiasTee, true);
        assert!(s.get(Switch::BiasTee));
        assert!(!s.get(Switch::RtlAgc));
        let offered = [Switch::RtlAgc, Switch::BiasTee, Switch::DirectSampling];
        let shown = s.toggles(&offered);
        assert_eq!(shown.len(), 3);
        assert_eq!(shown[1].name, "bias_tee");
        assert!(shown[1].on);
        assert!(!shown.iter().any(|t| t.name == "offset_tuning"));
    }
}
