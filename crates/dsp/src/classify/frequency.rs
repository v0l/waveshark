//! Frequency keying: two tones, four, and the minimum-shift case.

use super::hypothesis::{band, ramp, Evidence, Hypothesis};
use super::{Features, Modulation};

/// Two tones, far enough apart to threshold.
pub struct Fsk2;
impl Hypothesis for Fsk2 {
    fn modulation(&self) -> Modulation {
        Modulation::Fsk2
    }
    fn score(&self, f: &Features, e: &Evidence) -> f32 {
        e.constant_envelope
            * e.filled
            * f32::from(f.tones == 2)
            * ramp(f.mod_index, 0.8, 1.4)
            * (1.0 - e.sweeping)
    }
}

/// Two tones so close that the eye needs a matched receiver: modulation index
/// near 0.5, which is MSK and its filtered relative GMSK.
pub struct Msk;
impl Hypothesis for Msk {
    fn modulation(&self) -> Modulation {
        Modulation::Msk
    }
    fn score(&self, f: &Features, e: &Evidence) -> f32 {
        e.constant_envelope
            * e.filled
            * f32::from(f.tones == 2)
            * band(f.mod_index, 0.25, 0.4, 0.7, 0.95)
    }
}

/// Four levels.
pub struct Fsk4;
impl Hypothesis for Fsk4 {
    fn modulation(&self) -> Modulation {
        Modulation::Fsk4
    }
    fn score(&self, f: &Features, e: &Evidence) -> f32 {
        e.constant_envelope * e.filled * f32::from(f.tones == 4) * (1.0 - e.sweeping)
    }
}
