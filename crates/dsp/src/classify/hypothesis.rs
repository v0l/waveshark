//! The shape of a hypothesis, and the evidence they all share.
//!
//! One file per modulation, each scoring itself against the same measured
//! [`Features`]. A hypothesis is a self-contained answer to "how much does
//! this look like me?", and it cannot reach past the features to look at
//! another class's reasoning, which is what keeps a fix to one from moving
//! another.
//!
//! Scores are products of terms in 0 to 1, so a hypothesis is a conjunction:
//! every term is a condition it requires, and one of them being zero rules it
//! out. That is why they compose without ordering. A cascade of thresholds
//! would commit to the first plausible answer, and the earlier design of this
//! module says why that is wrong: the first mistake is unrecoverable because
//! nothing downstream reconsiders it.

use super::{Features, Modulation};

/// Terms derived from the features that more than one hypothesis needs.
///
/// Shared so that "constant envelope" means one thing across every class, and
/// changing what it means changes it everywhere at once rather than in the
/// four places that happened to spell it out.
pub struct Evidence {
    /// How far the envelope is keyed down. 1 is on-off, 0 is steady.
    pub keyed_amplitude: f32,
    /// Steady amplitude, either measured as one envelope level or as a high
    /// ratio between two.
    pub constant_envelope: f32,
    /// Two envelope levels, each held for about a symbol, and not on for
    /// nearly the whole burst.
    ///
    /// The last two conditions are what stop a frequency-keyed packet being
    /// read as amplitude keying: it is one long run of carrier, and the gaps
    /// in the window around it are between transmissions rather than inside
    /// one.
    pub two_levels: f32,
    /// The burst fills its window. An empty channel is flat and Gaussian too,
    /// and the difference is that noise is not on for the whole window at a
    /// level of its own.
    pub filled: f32,
    /// A symbol clock was found.
    ///
    /// The ramp is low because a real burst's line is weaker than a generated
    /// one's: on rtl_433's recordings the sensors come in between 3 and 4
    /// where a generated signal is above 4.5, and an on-off keyed burst with
    /// random data barely clears 1.6, because its transitions are impulses and
    /// an impulse train at random times is mostly white.
    pub has_clock: f32,
    /// One peak in the frequency histogram.
    pub unimodal: f32,
    /// The frequency track is a straight line.
    pub sweeping: f32,

    /// Everything the noise-like family has in common: filled, one envelope
    /// level, flat, Gaussian, unpeaked, not swept.
    ///
    /// Shared because three hypotheses need exactly it and then disagree only
    /// about what repeats. Written out in each of them instead, the OFDM case
    /// scored its own core times an extra term and so could never outscore
    /// the fallback it was meant to refine: a hypothesis built by multiplying
    /// another one's score can only ever lose to it.
    pub noise_like: f32,
    /// A cyclic prefix: one sharp repeat at a credible lag.
    pub prefix: f32,
    /// A spreading code: a strong repeat in envelope power, at many lags.
    pub chips: f32,
}

impl Evidence {
    pub fn from(f: &Features) -> Self {
        Self {
            keyed_amplitude: 1.0 - f.envelope_ratio,
            constant_envelope: ramp(f.envelope_ratio, 0.4, 0.75)
                .max(f32::from(f.envelope_modes == 1)),
            two_levels: f32::from(f.envelope_modes >= 2)
                * (1.0 - ramp(f.level_run_symbols, 4.0, 12.0))
                * (1.0 - ramp(f.duty, 0.8, 0.92)),
            filled: ramp(f.duty, 0.5, 0.8),
            has_clock: ramp(f.baud_line, 2.0, 4.0),
            unimodal: f32::from(f.tones == 1),
            sweeping: ramp(f.chirp_fit, 0.55, 0.85),
            noise_like: ramp(f.duty, 0.5, 0.8)
                * f32::from(f.envelope_modes == 1)
                * ramp(f.flatness, 0.3, 0.6)
                * ramp(f.kurtosis, 2.2, 2.6)
                * (1.0 - ramp(f.peakiness, 0.1, 0.3))
                * (1.0 - ramp(f.chirp_fit, 0.55, 0.85)),
            prefix: ramp(f.cyclic_ratio, 4.0, 8.0) * ramp(f.cyclic, 0.02, 0.05),
            chips: ramp(f.env_cyclic, 0.25, 0.5) * ramp(f.env_cyclic_ratio, 2.0, 3.0),
        }
    }
}

/// One modulation's case for itself.
pub trait Hypothesis: Sync {
    fn modulation(&self) -> Modulation;

    /// How well the burst fits, 0 to 1. Zero means ruled out.
    fn score(&self, f: &Features, e: &Evidence) -> f32;
}

/// 0 below `lo`, 1 above `hi`, straight line between.
pub fn ramp(v: f32, lo: f32, hi: f32) -> f32 {
    if hi <= lo {
        return f32::from(v >= hi);
    }
    ((v - lo) / (hi - lo)).clamp(0.0, 1.0)
}

/// A trapezoid: up between `lo0` and `lo1`, down between `hi0` and `hi1`.
pub fn band(v: f32, lo0: f32, lo1: f32, hi0: f32, hi1: f32) -> f32 {
    ramp(v, lo0, lo1).min(1.0 - ramp(v, hi0, hi1))
}
