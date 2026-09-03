//! Orthogonal frequency division multiplexing.
//!
//! OFDM is a sum of many independent carriers, so by the central limit theorem
//! its samples are Gaussian and its spectrum is flat. That makes it
//! indistinguishable from noise, from interference, and from a spread single
//! carrier on every feature the rest of this module measures: all four land in
//! [`super::steady::NoiseLike`], which is why that class is documented as
//! covering them together.
//!
//! What separates it is the cyclic prefix. Every symbol's tail is copied to
//! its head, so the burst correlates with itself at exactly one lag, the
//! symbol period, and nowhere else. Noise has no such lag. That test was
//! measured against an LTE downlink capture and a recording of an empty band:
//! LTE's peak stands five to fifteen times the level noise reaches, and the
//! empty band's own spurs correlate across a plateau rather than at a point,
//! which the localization ratio rejects.

use super::hypothesis::{ramp, Evidence, Hypothesis};
use super::{Features, Modulation};

pub struct Ofdm;

impl Hypothesis for Ofdm {
    fn modulation(&self) -> Modulation {
        Modulation::Ofdm
    }

    fn score(&self, f: &Features, e: &Evidence) -> f32 {
        // Noise-like *and* prefixed. Requiring the noise-like core means a
        // burst that is not noise-like cannot become OFDM by finding a repeat
        // somewhere, because a keyed signal repeats at its symbol rate too.
        // Sub-Gaussian samples are a spread single carrier, not a sum of
        // subcarriers, which is what keeps this apart from DSSS.
        //
        // And no phase line at any power. A sum of subcarriers has none;
        // a phase-keyed carrier with a training sequence every slot has a
        // repeat at the slot period that reads as a prefix, and a TETRA
        // downlink was called OFDM on that alone.
        e.noise_like
            * e.prefix
            * ramp(f.kurtosis, 2.3, 2.7)
            * (1.0 - ramp(f.square_line.max(f.quartic_line), 0.2, 0.5))
            * (1.0 - ramp(f.quartic_pair, 0.15, 0.35))
    }
}
