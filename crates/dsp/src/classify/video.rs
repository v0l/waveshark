//! Analogue video on a frequency modulated carrier, which is what an FPV
//! transmitter sends.
//!
//! Nothing else here is a shape: the carrier is constant envelope, wide, on
//! all the time and carries no symbol clock, which is also a fair description
//! of a leaking oscillator seen through a wide filter. What settles it is the
//! only structure analogue video has, the line sync train, and
//! [`crate::video::examine_lines`] already looks for that in demodulated
//! baseband. The classifier's frequency track is demodulated baseband, so the
//! same test runs on it for the cost of one pass.
//!
//! Polarity is not known in advance. Broadcast television and most FPV
//! transmitters key sync to the lowest frequency, but the convention is a
//! designer's choice and an inverted link is a picture with the sync at the
//! top, so both signs are tried and the better one wins.

use super::hypothesis::{Evidence, Hypothesis, ramp};
use super::{Features, Modulation};

/// Below this the sync pulse is too few samples to find: 4.7 us is nine
/// samples at 2 MS/s, and the quarter-microsecond smoothing the test applies
/// is one tap. A video link is 6 MHz wide at the narrowest, so a receiver
/// that can hear one at all samples faster than this.
pub const MIN_RATE: f64 = 2e6;

/// Narrower than this is not a video carrier however the frequency track
/// behaves. The narrowest analogue link in use is about 6 MHz; 1 MHz is well
/// under that and only there to keep the test off narrowband channels, where
/// it costs a sort of the whole window and can find nothing.
pub const MIN_BANDWIDTH_HZ: f32 = 1e6;

/// How much of the window a line train has to explain before the gaps are a
/// line rate rather than a coincidence.
///
/// Measured on a PAL carrier at 20 MS/s with 4 MHz of deviation: sixteen
/// lines give 1.00 agreement and 0.94 coverage, and a window of noise gives
/// 0.00 because the median gap it finds names no standard at all. The gap is
/// wide enough that the threshold is not delicate.
const MIN_AGREEMENT: f32 = 0.5;

/// Fraction of a window a line sync train accounts for, or zero when the gaps
/// between candidate pulses do not agree on a period either standard names.
///
/// `freq` is the instantaneous frequency in hertz, which is what an FM
/// receiver would put out, and its scale does not matter: the test slices
/// between percentiles of the window it was given.
pub fn line_coverage(freq: &[f32], rate: f64) -> f32 {
    if rate < MIN_RATE || freq.is_empty() {
        return 0.0;
    }
    let normal = crate::video::examine_lines(freq, rate);
    let inverted: Vec<f32> = freq.iter().map(|v| -v).collect();
    let flipped = crate::video::examine_lines(&inverted, rate);
    [normal, flipped]
        .into_iter()
        .filter(|a| a.standard.is_some() && a.agreement >= MIN_AGREEMENT)
        .map(|a| a.coverage)
        .fold(0.0, f32::max)
}

/// A frequency modulated carrier with a line rate in it.
pub struct FmVideo;
impl Hypothesis for FmVideo {
    fn modulation(&self) -> Modulation {
        Modulation::Fm
    }
    fn score(&self, f: &Features, e: &Evidence) -> f32 {
        // No clock term and no bandwidth term: the line train is the evidence,
        // and it is not something any keyed modulation produces by accident.
        // What the envelope and the fill add is that the carrier was there for
        // the whole window, which rules out a window of noise whose percentiles
        // happened to slice into runs.
        e.constant_envelope * e.filled * ramp(f.video_lines, 0.3, 0.7)
    }
}

/// Composite video as a camera makes it, sync below blanking, in the units
/// the classifier's frequency track carries: hertz of deviation.
#[cfg(test)]
pub(crate) fn composite(
    standard: crate::video::Standard,
    rate: f64,
    deviation_hz: f32,
    lines: usize,
) -> Vec<f32> {
    let n = |s: f64| (s * rate).round() as usize;
    let (line, sync, back, active) = (
        n(standard.line_s()),
        n(standard.sync_s()),
        n(standard.back_porch_s()),
        n(standard.active_s()),
    );
    let mut out = Vec::new();
    for _ in 0..lines {
        out.extend(std::iter::repeat_n(-0.3 * deviation_hz, sync));
        out.extend(std::iter::repeat_n(0.0, back));
        for i in 0..active {
            out.push((i as f32 / active as f32) * 0.7 * deviation_hz);
        }
        out.extend(std::iter::repeat_n(0.0, line.saturating_sub(sync + back + active)));
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::video::Standard;

    #[test]
    fn a_line_train_is_found_at_either_polarity() {
        let rate = 20e6;
        let v = composite(Standard::Pal, rate, 4e6, 16);
        let normal = line_coverage(&v, rate);
        let inverted: Vec<f32> = v.iter().map(|s| -s).collect();
        let flipped = line_coverage(&inverted, rate);
        assert!(normal > 0.85, "PAL coverage was {normal}");
        assert!(flipped > 0.85, "inverted PAL coverage was {flipped}");
    }

    #[test]
    fn noise_has_no_line_rate() {
        let rate = 20e6;
        let mut seed = 0x1234_5678u32;
        let noise: Vec<f32> = (0..1 << 15)
            .map(|_| {
                seed = seed.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
                (seed >> 8) as f32 / 8.4e6 - 1.0
            })
            .collect();
        assert_eq!(line_coverage(&noise, rate), 0.0);
    }

    #[test]
    fn a_slow_receiver_does_not_guess() {
        let rate = 1e6;
        let v = composite(Standard::Pal, rate, 4e6, 16);
        assert_eq!(line_coverage(&v, rate), 0.0);
    }
}
