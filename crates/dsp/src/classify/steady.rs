//! Signals with no keying structure: a bare carrier, and the noise-like case.

use super::hypothesis::{ramp, Evidence, Hypothesis};
use super::{Features, Modulation};

/// Present, steady, and saying nothing.
///
/// A carrier has a squared line as strong as any phase-keyed signal, so what
/// identifies it is the absence of a symbol clock rather than the absence of a
/// line.
pub struct Carrier;
impl Hypothesis for Carrier {
    fn modulation(&self) -> Modulation {
        Modulation::Carrier
    }
    fn score(&self, f: &Features, e: &Evidence) -> f32 {
        e.constant_envelope
            * e.filled
            * e.unimodal
            * (1.0 - e.has_clock)
            * (1.0 - e.sweeping)
            * ramp(f.peakiness, 0.1, 0.3)
    }
}

/// Modulated, with no keying structure to find.
///
/// The fallback for multi-carrier and spread signals: [`super::ofdm`] and
/// [`super::dsss`] claim the two that can be told apart, and what neither
/// claims lands here with its features attached.
pub struct NoiseLike;
impl Hypothesis for NoiseLike {
    fn modulation(&self) -> Modulation {
        Modulation::NoiseLike
    }
    fn score(&self, _f: &Features, e: &Evidence) -> f32 {
        // What neither OFDM nor DSSS claimed. The two exclusions are what
        // make the three a partition rather than a hierarchy: without them
        // this always outscores its own refinements, since theirs are this
        // score multiplied by a number no greater than one.
        e.noise_like * (1.0 - e.has_clock) * (1.0 - e.prefix) * (1.0 - e.chips)
    }
}
