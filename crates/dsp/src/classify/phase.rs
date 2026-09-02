//! Phase keying.
//!
//! Both power laws collapse two phases onto one, so binary phase keying shows
//! a line in each. Four phases survive squaring, which is what separates the
//! two: a strong fourth-power line and no squared one.

use super::hypothesis::{ramp, Evidence, Hypothesis};
use super::{Features, Modulation};

pub struct Psk2;
impl Hypothesis for Psk2 {
    fn modulation(&self) -> Modulation {
        Modulation::Psk2
    }
    fn score(&self, f: &Features, e: &Evidence) -> f32 {
        e.constant_envelope * e.filled * e.unimodal * e.has_clock * ramp(f.square_line, 0.2, 0.5)
    }
}

pub struct Psk4;
impl Hypothesis for Psk4 {
    fn modulation(&self) -> Modulation {
        Modulation::Psk4
    }
    fn score(&self, f: &Features, e: &Evidence) -> f32 {
        e.constant_envelope
            * e.filled
            * e.unimodal
            * e.has_clock
            * ramp(f.quartic_line, 0.2, 0.5)
            * (1.0 - ramp(f.square_line, 0.2, 0.5))
    }
}
