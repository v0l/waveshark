//! How a channel is listened to.
//!
//! The audio mode an operator picks, and everything about a channel that
//! follows from it: how wide it is, what rate the demodulator runs at, where
//! the squelch sits. Here rather than in the receiver because the allocation
//! table in [`crate::bands`] names one per band, and a band plan is knowledge
//! about the world rather than about this program's audio path.

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Demod {
    Wfm,
    Nfm,
    Am,
    /// Upper sideband, the amateur convention above 10 MHz.
    Usb,
    /// Lower sideband, the convention on 160, 80 and 40 metres.
    Lsb,
    /// Morse, which is upper sideband through a narrow filter.
    Cw,
}

impl Demod {
    pub const ALL: [Demod; 6] =
        [Demod::Wfm, Demod::Nfm, Demod::Am, Demod::Usb, Demod::Lsb, Demod::Cw];

    /// The mode a band plan names, which is its label in lower case.
    pub fn from_id(s: &str) -> Option<Self> {
        Self::ALL.into_iter().find(|d| d.label().eq_ignore_ascii_case(s))
    }

    pub fn label(self) -> &'static str {
        match self {
            Demod::Wfm => "WFM",
            Demod::Nfm => "NFM",
            Demod::Am => "AM",
            Demod::Usb => "USB",
            Demod::Lsb => "LSB",
            Demod::Cw => "CW",
        }
    }

    /// Whether this mode listens to one sideband of the dial frequency.
    pub fn is_ssb(self) -> bool {
        matches!(self, Demod::Usb | Demod::Lsb | Demod::Cw)
    }

    pub fn has_auto_notch(self) -> bool {
        match self {
            Demod::Am | Demod::Usb | Demod::Lsb => true,
            Demod::Wfm | Demod::Nfm | Demod::Cw => false,
        }
    }

    /// Whether this mode listens below the dial frequency. The sideband type
    /// itself belongs to the demodulator, so the receiver turns this into one.
    pub fn is_lower(self) -> bool {
        matches!(self, Demod::Lsb)
    }

    /// Where the squelch opens by default, or None for a mode with none.
    ///
    /// Public because a control has to show the value in use before the
    /// operator has touched anything, and inventing a second copy of these
    /// numbers in the interface is how the two drift apart.
    pub fn default_squelch_db(self) -> Option<f32> {
        match self {
            // Measured, not guessed. Through this chain an empty channel
            // reads about 6.4 dB and an FM signal reads 24 dB and hardly
            // moves with signal strength, because FM captures. Sitting in the
            // middle of that gap keeps noise out with room for the reading to
            // wander, which live it does by a couple of dB.
            //
            // It was 9 dB, which is inside the noise's own variation: any
            // excursion opened the squelch, and the hysteresis then held it
            // open on noise indefinitely.
            Demod::Nfm => Some(14.0),
            // Off, at the bottom of the control's range.
            //
            // A level squelch has no fixed sensible setting: measured on an
            // empty 2 m channel the audio sits at -26 dBFS in AM, -36 in USB
            // and -59 in CW, and all three move with the RF gain. A number
            // picked here would be doing nothing on one mode and muting a
            // station on another, and SSB is normally listened to wide open
            // anyway. Drag it up against the meter to set one.
            Demod::Am | Demod::Usb | Demod::Lsb | Demod::Cw => Some(-90.0),
            Demod::Wfm => None,
        }
    }

    /// The range a squelch control should span for this mode, and whether the
    /// measurement is a noise ratio rather than a level.
    pub fn squelch_range(self) -> (f32, f32, bool) {
        match self {
            // How much of the signal is not noise: 0 dB is an empty channel
            // and 25 dB is full quieting.
            Demod::Nfm => (0.0, 25.0, true),
            _ => (-90.0, -10.0, false),
        }
    }

    /// The pitch a CW signal is heard at.
    ///
    /// The receiver is tuned this far below the carrier so that the dial
    /// reads the transmitted frequency rather than the note in the operator's
    /// ears, which is the convention every other radio follows and the one
    /// that makes two stations agree about where they are.
    pub fn cw_pitch(self) -> f64 {
        match self {
            Demod::Cw => 700.0,
            _ => 0.0,
        }
    }

    /// Occupied channel bandwidth, two-sided.
    pub fn bandwidth(self) -> f64 {
        match self {
            // Carson: 2 * (75 kHz deviation + 57 kHz highest modulating
            // frequency). The highest is RDS, not audio: taking 15 kHz gives
            // 180 kHz and cuts off precisely the sidebands that carry the
            // subcarrier, which decodes audio perfectly and RDS barely at all.
            Demod::Wfm => 264_000.0,
            Demod::Nfm => 12_500.0,
            Demod::Am => 10_000.0,
            // Twice the audio bandwidth, because only one sideband is there
            // but the IF filter around it is symmetric: half of this has to
            // reach the far edge of the sideband or the top of the voice is
            // filtered off before the demodulator sees it.
            Demod::Usb | Demod::Lsb => 6_000.0,
            Demod::Cw => 4_000.0,
        }
    }

    /// Sample rate to run the demodulator at.
    ///
    /// Comfortably above the channel bandwidth, never equal to it. Decimating
    /// until the output rate matches the bandwidth leaves no transition band,
    /// and the anti-alias filter then needs thousands of taps: 7947 for NFM
    /// against 281 here, with a history buffer too big for L2.
    pub fn if_rate(self) -> f64 {
        match self {
            // Must clear the 264 kHz occupied bandwidth with room for a
            // transition band.
            Demod::Wfm => 330_000.0,
            // The sideband filter runs here rather than after a further
            // decimation, because it is the thing that defines the channel
            // and 363 taps at this rate is a few percent of one core.
            Demod::Nfm | Demod::Am | Demod::Usb | Demod::Lsb | Demod::Cw => 48_000.0,
        }
    }

    /// Audio bandwidth after demodulation.
    pub fn audio_bw(self) -> f64 {
        match self {
            Demod::Wfm => 15_000.0,
            Demod::Nfm => 4_000.0,
            Demod::Am => 5_000.0,
            Demod::Usb | Demod::Lsb => 3_000.0,
            Demod::Cw => 1_200.0,
        }
    }

    pub fn deviation(self) -> f64 {
        match self {
            Demod::Wfm => 75_000.0,
            Demod::Nfm => 5_000.0,
            Demod::Am | Demod::Usb | Demod::Lsb | Demod::Cw => 0.0,
        }
    }
}

pub const SIDEBAND_AUDIO_HZ: (f64, f64) = (300.0, 2_700.0);
pub const CW_FILTER_HZ: f64 = 500.0;
pub const AUDIO_EDGE_RANGE_HZ: std::ops::RangeInclusive<f64> = 50.0..=6_000.0;
// A width below a hundred hertz is a mis-set control rather than a
// channel, and it would design a filter with thousands of taps.
pub const NARROWEST_HZ: f64 = 100.0;

pub fn clamp_audio(low_hz: f64, high_hz: f64) -> (f64, f64) {
    let (floor, ceiling) = (*AUDIO_EDGE_RANGE_HZ.start(), *AUDIO_EDGE_RANGE_HZ.end());
    let low = low_hz.clamp(floor, ceiling - NARROWEST_HZ);
    (low, high_hz.clamp(low + NARROWEST_HZ, ceiling))
}

#[derive(Clone, Copy, PartialEq, Debug)]
pub struct Passband {
    pub low_hz: f64,
    pub high_hz: f64,
}

impl Passband {
    pub fn around(width_hz: f64) -> Self {
        Self { low_hz: -width_hz / 2.0, high_hz: width_hz / 2.0 }
    }

    pub fn width(self) -> f64 {
        self.high_hz - self.low_hz
    }

    pub fn reach(self) -> f64 {
        self.low_hz.abs().max(self.high_hz.abs())
    }

    pub fn shifted(self, hz: f64) -> Self {
        Self { low_hz: self.low_hz + hz, high_hz: self.high_hz + hz }
    }
}

impl Demod {
    pub fn audio_edges(self, width_hz: Option<f64>, low_hz: Option<f64>) -> (f64, f64) {
        let width = width_hz.filter(|w| *w >= NARROWEST_HZ).unwrap_or(match self {
            Demod::Cw => CW_FILTER_HZ,
            _ => SIDEBAND_AUDIO_HZ.1 - SIDEBAND_AUDIO_HZ.0,
        });
        let low = low_hz.unwrap_or(match self {
            Demod::Cw => self.cw_pitch() - width / 2.0,
            _ => SIDEBAND_AUDIO_HZ.0,
        });
        clamp_audio(low, low + width)
    }

    pub fn passband(self, width_hz: Option<f64>, low_hz: Option<f64>) -> Passband {
        if !self.is_ssb() {
            return Passband::around(
                width_hz.filter(|w| *w >= NARROWEST_HZ).unwrap_or_else(|| self.bandwidth()),
            );
        }
        let (low, high) = self.audio_edges(width_hz, low_hz);
        match self {
            Demod::Lsb => Passband { low_hz: -high, high_hz: -low },
            _ => Passband { low_hz: low, high_hz: high }.shifted(-self.cw_pitch()),
        }
    }

    pub fn audio_of(self, p: Passband) -> (f64, f64) {
        match self {
            Demod::Lsb => clamp_audio(-p.high_hz, -p.low_hz),
            _ => {
                let a = p.shifted(self.cw_pitch());
                clamp_audio(a.low_hz, a.high_hz)
            }
        }
    }

    pub fn if_reach(self, p: Passband) -> f64 {
        match self.is_ssb() {
            true => p.shifted(self.cw_pitch()).reach().max(self.bandwidth() / 2.0),
            false => p.reach(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_sideband_passband_sits_on_its_side_of_the_dial() {
        assert_eq!(Demod::Usb.passband(None, None), Passband { low_hz: 300.0, high_hz: 2_700.0 });
        assert_eq!(Demod::Lsb.passband(None, None), Passband { low_hz: -2_700.0, high_hz: -300.0 });
        assert_eq!(Demod::Cw.passband(None, None), Passband { low_hz: -250.0, high_hz: 250.0 });
        assert_eq!(Demod::Nfm.passband(None, None), Passband::around(12_500.0));
    }

    #[test]
    fn a_width_and_a_low_edge_set_a_sideband_filter() {
        assert_eq!(Demod::Usb.audio_edges(Some(2_000.0), Some(500.0)), (500.0, 2_500.0));
        assert_eq!(
            Demod::Lsb.passband(Some(2_000.0), Some(500.0)),
            Passband { low_hz: -2_500.0, high_hz: -500.0 }
        );
        assert_eq!(Demod::Cw.audio_edges(Some(200.0), None), (600.0, 800.0));
        assert_eq!(Demod::Am.passband(Some(6_000.0), Some(500.0)), Passband::around(6_000.0));
    }

    #[test]
    fn audio_edges_stay_inside_what_the_filter_can_pass() {
        assert_eq!(Demod::Usb.audio_edges(Some(20_000.0), Some(-100.0)), (50.0, 6_000.0));
        assert_eq!(Demod::Usb.audio_edges(Some(40.0), Some(1_000.0)), (1_000.0, 3_400.0));
        assert_eq!(Demod::Usb.audio_edges(Some(500.0), Some(5_950.0)), (5_900.0, 6_000.0));
    }

    #[test]
    fn a_passband_reads_back_as_the_audio_it_came_from() {
        for d in [Demod::Usb, Demod::Lsb, Demod::Cw] {
            let p = d.passband(Some(1_800.0), Some(400.0));
            assert_eq!(d.audio_of(p), (400.0, 2_200.0), "{}", d.label());
        }
    }

    #[test]
    fn a_passband_dragged_through_the_dial_stops_at_the_filter_floor() {
        let through = Passband { low_hz: -400.0, high_hz: 2_700.0 };
        assert_eq!(Demod::Usb.audio_of(through), (50.0, 2_700.0));
        assert_eq!(
            Demod::Lsb.audio_of(Passband { low_hz: -2_700.0, high_hz: 400.0 }),
            (50.0, 2_700.0)
        );
        assert_eq!(Demod::Cw.audio_of(Passband { low_hz: -900.0, high_hz: 250.0 }), (50.0, 950.0));
    }

    #[test]
    fn the_if_covers_the_sideband_filter_and_never_narrows_below_the_mode() {
        assert_eq!(Demod::Usb.if_reach(Demod::Usb.passband(None, None)), 3_000.0);
        assert_eq!(Demod::Usb.if_reach(Demod::Usb.passband(Some(4_500.0), None)), 4_800.0);
        assert_eq!(Demod::Cw.if_reach(Demod::Cw.passband(None, None)), 2_000.0);
        assert_eq!(Demod::Cw.if_reach(Demod::Cw.passband(Some(3_000.0), Some(1_000.0))), 4_000.0);
        assert_eq!(Demod::Nfm.if_reach(Demod::Nfm.passband(Some(25_000.0), None)), 12_500.0);
    }
}
