//! Linear frequency sweeps: chirp spread spectrum and radar.

use super::hypothesis::{Evidence, Hypothesis};
use super::{Features, Modulation};

/// The one class whose evidence is the frequency track's slope rather than
/// its histogram.
pub struct Chirp;
impl Hypothesis for Chirp {
    fn modulation(&self) -> Modulation {
        Modulation::Chirp
    }
    fn score(&self, _f: &Features, e: &Evidence) -> f32 {
        e.constant_envelope * e.filled * e.sweeping
    }
}
