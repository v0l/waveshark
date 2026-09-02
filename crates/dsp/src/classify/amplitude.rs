//! Amplitude keying: on-off, and shallow.

use super::hypothesis::{band, ramp, Evidence, Hypothesis};
use super::{Features, Modulation};

/// Keyed all the way down to the noise. The plain envelope path reads these.
pub struct Ook;
impl Hypothesis for Ook {
    fn modulation(&self) -> Modulation {
        Modulation::Ook
    }
    fn score(&self, _f: &Features, e: &Evidence) -> f32 {
        e.two_levels * ramp(e.keyed_amplitude, 0.6, 0.85)
    }
}

/// Keyed, but the low level is still a signal. Needs the ASK front end rather
/// than a plain envelope threshold.
pub struct Ask;
impl Hypothesis for Ask {
    fn modulation(&self) -> Modulation {
        Modulation::Ask
    }
    fn score(&self, f: &Features, e: &Evidence) -> f32 {
        e.two_levels * band(f.envelope_ratio, 0.25, 0.45, 0.7, 0.8) * e.has_clock
    }
}
