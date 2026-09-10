//! Single sideband demodulation, and CW as a special case of it.
//!
//! SSB has no carrier to lock to and no envelope to follow: the receiver
//! simply mixes the signal down to audio and the operator adjusts the dial
//! until the voice sounds right. So the demodulator is a filter and a real
//! part, and all the work is in the filter, which has to keep one sideband
//! and reject the other rather than the symmetric passband every other mode
//! here uses.
//!
//! The filter is complex-tap, which is the direct way to get an asymmetric
//! response: take an ordinary lowpass of the right width and modulate its
//! taps up to the sideband's centre, and the response moves with it instead
//! of being mirrored around DC. The alternatives, a Hilbert transform pair or
//! Weaver's third method, need two matched paths and get their rejection from
//! how well those paths match; this gets it from the tap count, which is a
//! number you can choose.

use common::C32;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Sideband {
    /// Above the dial frequency, the amateur convention above 10 MHz.
    Upper,
    /// Below it, the convention on 160, 80 and 40 metres.
    Lower,
}

impl Sideband {
    /// What a setting or a menu calls it, and what [`FromStr`] reads back.
    pub fn label(self) -> &'static str {
        match self {
            Self::Upper => "usb",
            Self::Lower => "lsb",
        }
    }
}

impl std::str::FromStr for Sideband {
    type Err = common::Error;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        match s.trim().to_ascii_lowercase().as_str() {
            "usb" | "upper" => Ok(Self::Upper),
            "lsb" | "lower" => Ok(Self::Lower),
            other => Err(common::Error::other(format!(
                "no sideband called {other:?}"
            ))),
        }
    }
}

impl std::fmt::Display for Sideband {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(self.label())
    }
}

/// Stopband attenuation of the sideband filter.
///
/// Opposite-sideband rejection is what this number buys, and 60 dB is what a
/// decent commercial radio manages. Going further costs taps for something no
/// listener would hear over the band noise.
const ATTEN_DB: f64 = 60.0;

/// Width of the filter's skirt, as a fraction of the passband width.
///
/// The tap count is set by this: a skirt a fifth as wide as the passband puts
/// a 2.4 kHz voice filter at a few hundred taps, which is cheap at audio
/// rates, and keeps the adjacent signal 3 kHz away properly out.
const SKIRT: f64 = 0.2;

pub struct SsbDemod {
    taps: Vec<C32>,
    hist: Vec<C32>,
    pos: usize,
}

impl SsbDemod {
    /// A demodulator for `sideband`, passing audio between `low_hz` and
    /// `high_hz` away from the dial frequency.
    ///
    /// The low edge matters as much as the high one. Voice below about
    /// 200 Hz carries no intelligibility but plenty of mains hum and rumble,
    /// and on a crowded band it is where the neighbouring signal's carrier
    /// sits, so passing it costs nothing and buys interference.
    pub fn new(rate: f64, sideband: Sideband, low_hz: f64, high_hz: f64) -> Self {
        let (low, high) = (low_hz.min(high_hz), low_hz.max(high_hz));
        let width = (high - low).max(1.0);
        let mut centre = (low + high) / 2.0;
        if sideband == Sideband::Lower {
            centre = -centre;
        }

        let transition = (width * SKIRT / rate).max(1e-4);
        let n = crate::fir::estimate_taps(transition, ATTEN_DB);
        // Half the width, because the prototype is a lowpass that will be
        // shifted: its passband runs from minus cutoff to plus cutoff and
        // ends up spanning the full width once modulated.
        let proto = crate::fir::lowpass(n, width / 2.0 / rate, ATTEN_DB);

        let mid = (proto.len() - 1) as f64 / 2.0;
        let w = std::f64::consts::TAU * centre / rate;
        let taps: Vec<C32> = proto
            .iter()
            .enumerate()
            .map(|(k, &h)| {
                let p = w * (k as f64 - mid);
                C32::new(h * p.cos() as f32, h * p.sin() as f32)
            })
            .collect();

        let n = taps.len();
        Self { taps, hist: vec![C32::new(0.0, 0.0); n], pos: 0 }
    }

    /// The usual voice filter, 300 Hz to 2.7 kHz.
    pub fn voice(rate: f64, sideband: Sideband) -> Self {
        Self::new(rate, sideband, 300.0, 2_700.0)
    }

    /// A CW filter: a narrow window around the pitch the operator listens for.
    ///
    /// Morse is received by tuning the signal so it beats against the
    /// receiver's own reference at an audible pitch, which is why a CW
    /// receiver is an SSB receiver with a narrower filter. 500 Hz is the
    /// common contest setting: wide enough that being slightly off the dial
    /// still lets the tone through, narrow enough to silence the station a
    /// few hundred hertz away.
    pub fn cw(rate: f64, sideband: Sideband, pitch_hz: f64, width_hz: f64) -> Self {
        let half = width_hz.max(50.0) / 2.0;
        Self::new(rate, sideband, (pitch_hz - half).max(50.0), pitch_hz + half)
    }

    pub fn taps(&self) -> usize {
        self.taps.len()
    }

    pub fn reset(&mut self) {
        self.hist.fill(C32::new(0.0, 0.0));
        self.pos = 0;
    }

    pub fn process(&mut self, input: &[C32], out: &mut Vec<f32>) {
        out.reserve(input.len());
        let n = self.taps.len();
        for &x in input {
            self.hist.copy_within(0..n - 1, 1);
            self.hist[0] = x;
            let mut acc = 0.0f32;
            // Only the real part of the convolution is ever used, so only the
            // real part is computed: the imaginary half of the work would be
            // thrown away, and this runs on every audio sample.
            for (h, s) in self.taps.iter().zip(self.hist.iter()) {
                acc += h.re * s.re - h.im * s.im;
            }
            out.push(acc);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::TAU;

    /// A tone `offset` Hz from the dial frequency, at full scale.
    fn tone(rate: f64, offset: f64, n: usize) -> Vec<C32> {
        (0..n)
            .map(|i| {
                let p = TAU * offset * i as f64 / rate;
                C32::new(p.cos() as f32, p.sin() as f32)
            })
            .collect()
    }

    fn level(d: &mut SsbDemod, rate: f64, offset: f64) -> f32 {
        let mut out = Vec::new();
        d.process(&tone(rate, offset, 8192), &mut out);
        // Skip the filter's fill, then take the RMS of what settled.
        let tail = &out[out.len() / 2..];
        (tail.iter().map(|v| v * v).sum::<f32>() / tail.len() as f32).sqrt()
    }

    fn db(v: f32) -> f32 {
        20.0 * v.max(1e-12).log10()
    }

    #[test]
    fn upper_sideband_hears_above_the_dial_and_not_below() {
        let rate = 48_000.0;
        let mut d = SsbDemod::voice(rate, Sideband::Upper);
        let wanted = db(level(&mut d, rate, 1_000.0));
        d.reset();
        let image = db(level(&mut d, rate, -1_000.0));
        assert!(
            wanted - image > 50.0,
            "a signal on the wrong sideband was only {:.1} dB down",
            wanted - image
        );
    }

    #[test]
    fn lower_sideband_is_the_mirror_of_upper() {
        let rate = 48_000.0;
        let mut d = SsbDemod::voice(rate, Sideband::Lower);
        let wanted = db(level(&mut d, rate, -1_000.0));
        d.reset();
        let image = db(level(&mut d, rate, 1_000.0));
        assert!(wanted - image > 50.0, "only {:.1} dB of rejection", wanted - image);
    }

    #[test]
    fn a_tone_inside_the_passband_comes_out_at_the_level_it_went_in() {
        // Two demodulated signals of the same strength should sound the same
        // whichever mode they arrived in, so the filter's gain has to be one.
        let rate = 48_000.0;
        let mut d = SsbDemod::voice(rate, Sideband::Upper);
        let out = db(level(&mut d, rate, 1_500.0));
        // A full scale complex tone through a unity gain filter, then the
        // real part, is a full scale real sine: 0.707 RMS, or -3 dB. The
        // filter deliberately adds no make-up gain, because how loud a signal
        // should sound is the AGC's business and not the demodulator's.
        assert!((out + 3.0).abs() < 1.0, "a full scale tone came out at {out:.1} dBFS");
    }

    #[test]
    fn the_passband_edges_are_where_they_were_asked_for() {
        // Measured with a 2.4 kHz voice filter at 48 kHz, 363 taps: the
        // stated edges land on -6 dB, the response is inside 2 dB across the
        // passband, and 500 Hz outside it a signal is 60 dB down.
        let rate = 48_000.0;
        let mut d = SsbDemod::new(rate, Sideband::Upper, 300.0, 2_700.0);
        let mid = db(level(&mut d, rate, 1_500.0));
        for edge in [300.0, 2_700.0] {
            d.reset();
            let at = mid - db(level(&mut d, rate, edge));
            assert!((at - 6.0).abs() < 1.0, "the {edge} Hz edge sits at {at:.1} dB");
        }
        for inside in [400.0, 1_000.0, 2_600.0] {
            d.reset();
            let at = mid - db(level(&mut d, rate, inside));
            assert!(at < 2.0, "{inside} Hz is inside the filter but {at:.1} dB down");
        }
        for outside in [50.0, 3_200.0] {
            d.reset();
            let at = mid - db(level(&mut d, rate, outside));
            assert!(at > 60.0, "{outside} Hz is outside the filter but only {at:.1} dB down");
        }
    }

    #[test]
    fn the_cw_filter_is_narrow_around_the_pitch() {
        let rate = 48_000.0;
        let mut d = SsbDemod::cw(rate, Sideband::Upper, 700.0, 500.0);
        let on = db(level(&mut d, rate, 700.0));
        d.reset();
        // A station 500 Hz up the band is a different station, and the whole
        // point of a CW filter is that it is not audible.
        let off = db(level(&mut d, rate, 1_200.0));
        assert!(on - off > 45.0, "an adjacent signal was only {:.1} dB down", on - off);
    }
}
