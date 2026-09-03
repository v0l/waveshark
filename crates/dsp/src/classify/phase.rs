//! Phase keying.
//!
//! Each power law collapses the phases that divide it, so binary phase keying
//! shows a line in every one. Four phases survive squaring, which is what
//! separates the two: a strong fourth-power line and no squared one. Shift
//! the four by an eighth of a turn every symbol and the fourth power's line
//! splits into a pair a symbol rate apart, which is how a TETRA carrier is
//! told from plain QPSK.

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
            * (1.0 - ramp(f.quartic_pair, 0.15, 0.35))
    }
}

/// Four phases shifted by an eighth of a turn every symbol.
///
/// Told from QPSK by what the fourth power leaves: not one line but a pair
/// a symbol rate apart, since the shift flips the fourth power's sign every
/// symbol. TETRA keys this, and so do several trunked systems.
pub struct Dqpsk;
impl Hypothesis for Dqpsk {
    fn modulation(&self) -> Modulation {
        Modulation::Dqpsk
    }
    fn score(&self, f: &Features, e: &Evidence) -> f32 {
        // No clock term: the pair is the clock, found by a route that does
        // not need the samples a symbol the transition spectrum does.
        e.constant_envelope
            * e.filled
            * e.unimodal
            * ramp(f.quartic_pair, 0.15, 0.35)
            * (1.0 - ramp(f.square_line, 0.2, 0.5))
    }
}
