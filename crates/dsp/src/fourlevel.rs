//! Shared machinery for fitting four evenly spaced levels to a burst.
//!
//! The two-level version of this problem, in [`crate::twolevel`], only has to
//! find where the boundary between two clusters lies. Four levels need more:
//! the outer pair carries three times the inner pair's offset, so a fit has to
//! recover a centre *and* a step, and both move with the tuning error and the
//! transmitter's deviation.
//!
//! Levels are numbered 0 to 3 in ascending order of the quantity measured,
//! which for a discriminator means ascending frequency. That is deliberately
//! not a dibit. Every four-level protocol maps the levels to bits its own way
//! (DMR sends +3 as `01`, FLEX counts the other direction), and a front end
//! that guesses one of them is wrong for the rest, so the mapping is the
//! protocol's business and this layer stays numeric.

/// A fitted four-level constellation: levels sit at `center` plus `step` times
/// -3, -1, +1 and +3.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct Levels {
    pub center: f32,
    pub step: f32,
}

/// The ideal offset of each level, in steps.
pub(crate) const IDEAL: [f32; 4] = [-3.0, -1.0, 1.0, 3.0];

impl Levels {
    /// Which of the four levels a sample is nearest, 0 (lowest) to 3.
    pub(crate) fn index(&self, v: f32) -> u8 {
        let n = (v - self.center) / self.step;
        if n < -2.0 {
            0
        } else if n < 0.0 {
            1
        } else if n < 2.0 {
            2
        } else {
            3
        }
    }

    /// How far a sample sits from the level it was assigned, in steps. One
    /// step is half the distance to the next decision boundary, so anything
    /// approaching 1.0 is a symbol that could as easily have been its
    /// neighbour.
    pub(crate) fn error(&self, v: f32) -> f32 {
        let n = (v - self.center) / self.step;
        n - IDEAL[self.index(v) as usize]
    }

    /// Root mean square of [`Self::error`] over a set of samples: the eye
    /// closure, in steps.
    pub(crate) fn evm(&self, samples: &[f32]) -> f32 {
        if samples.is_empty() {
            return f32::INFINITY;
        }
        let sum: f32 = samples.iter().map(|&v| self.error(v).powi(2)).sum();
        (sum / samples.len() as f32).sqrt()
    }

    /// Fraction of samples landing on each level. A signal that is really
    /// two-level fits this model happily with its two inner levels empty, so
    /// the occupancy is what tells the two apart.
    pub(crate) fn occupancy(&self, samples: &[f32]) -> [f32; 4] {
        let mut counts = [0u32; 4];
        for &v in samples {
            counts[self.index(v) as usize] += 1;
        }
        let n = samples.len().max(1) as f32;
        [
            counts[0] as f32 / n,
            counts[1] as f32 / n,
            counts[2] as f32 / n,
            counts[3] as f32 / n,
        ]
    }
}

/// Fit a four-level constellation to symbol samples.
///
/// Seeded from the 2nd and 98th percentiles, which are the outer levels with
/// the discriminator's spikes trimmed off, then refined by least squares
/// against the level each sample was assigned. Percentiles rather than a mean
/// and a spread because a burst is rarely level-balanced: four fifths of a
/// sync word can be outer levels, and any seed that assumes an even mix starts
/// with a step half again too large and refines into the wrong assignment.
///
/// Returns `None` when there is too little to fit or the samples are all one
/// value, which is a carrier rather than a signal.
pub(crate) fn levels(scratch: &mut Vec<f32>, samples: &[f32]) -> Option<Levels> {
    if samples.len() < 16 {
        return None;
    }
    scratch.clear();
    scratch.extend(samples.iter().copied().filter(|v| v.is_finite()));
    if scratch.len() < 16 {
        return None;
    }
    scratch.sort_unstable_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));

    let idx = |q: f32| ((scratch.len() - 1) as f32 * q).round() as usize;
    let (lo, hi) = (scratch[idx(0.02)], scratch[idx(0.98)]);
    let mut center = 0.5 * (lo + hi);
    // The outer levels are six steps apart.
    let mut step = (hi - lo) / 6.0;
    if step <= 0.0 || !step.is_finite() {
        return None;
    }

    for _ in 0..4 {
        let fit = Levels { center, step };
        let mut sum_a = 0.0f32;
        let mut sum_aa = 0.0f32;
        let mut sum_v = 0.0f32;
        let mut sum_va = 0.0f32;
        for &v in samples.iter().filter(|v| v.is_finite()) {
            let a = IDEAL[fit.index(v) as usize];
            sum_a += a;
            sum_aa += a * a;
            sum_v += v;
            sum_va += v * a;
        }
        let n = scratch.len() as f32;
        // Least squares for v = center + step * a over both unknowns.
        let det = n * sum_aa - sum_a * sum_a;
        if det.abs() < 1e-12 {
            break;
        }
        let new_center = (sum_v * sum_aa - sum_a * sum_va) / det;
        let new_step = (n * sum_va - sum_a * sum_v) / det;
        if new_step <= 0.0 || !new_step.is_finite() {
            break;
        }
        let settled = (new_center - center).abs() < 1e-6 * step && (new_step - step).abs() < 1e-6 * step;
        center = new_center;
        step = new_step;
        if settled {
            break;
        }
    }

    Some(Levels { center, step })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fit(samples: &[f32]) -> Levels {
        levels(&mut Vec::new(), samples).expect("no fit")
    }

    fn spread(center: f32, step: f32, pattern: &[usize], jitter: f32) -> Vec<f32> {
        let mut seed = 99u64;
        let mut rng = move || {
            seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
            ((seed >> 33) as f32 / (1u64 << 30) as f32 - 1.0) * jitter
        };
        pattern.iter().map(|&i| center + step * IDEAL[i] + rng()).collect()
    }

    #[test]
    fn recovers_centre_and_step_from_balanced_levels() {
        let pattern: Vec<usize> = (0..64).map(|i| i % 4).collect();
        let got = fit(&spread(1_000.0, 600.0, &pattern, 30.0));
        assert!((got.center - 1_000.0).abs() < 60.0, "centre came out at {}", got.center);
        assert!((got.step - 600.0).abs() < 60.0, "step came out at {}", got.step);
    }

    #[test]
    fn refinement_survives_a_lopsided_mix() {
        // Four fifths outer levels, as a sync word tends to be. The initial
        // mean absolute deviation is far too large here, and only the least
        // squares pass brings the step back.
        let pattern: Vec<usize> = (0..80)
            .map(|i| if i % 5 == 0 { 1 + (i / 5) % 2 } else { (i % 2) * 3 })
            .collect();
        let got = fit(&spread(-200.0, 500.0, &pattern, 20.0));
        assert!((got.center + 200.0).abs() < 70.0, "centre came out at {}", got.center);
        assert!((got.step - 500.0).abs() < 70.0, "step came out at {}", got.step);
    }

    #[test]
    fn assigns_levels_in_ascending_order() {
        let l = Levels { center: 0.0, step: 100.0 };
        assert_eq!(l.index(-320.0), 0);
        assert_eq!(l.index(-90.0), 1);
        assert_eq!(l.index(110.0), 2);
        assert_eq!(l.index(400.0), 3);
    }

    #[test]
    fn a_two_level_signal_leaves_the_inner_levels_empty() {
        // Only the outer pair is sent, which is what two-level FSK looks like
        // once a four-level fit has been forced onto it.
        let pattern: Vec<usize> = (0..64).map(|i| i % 2 * 3).collect();
        let samples = spread(0.0, 400.0, &pattern, 20.0);
        let got = fit(&samples);
        let occ = got.occupancy(&samples);
        // Which two levels the fit lands on is not determined: two clusters
        // can be described as the outer pair, or as one outer and one inner.
        // What matters downstream is that two of the four go unused, which no
        // real four-level frame does.
        assert_eq!(occ.iter().filter(|&&s| s < 0.01).count(), 2, "all four levels were used: {occ:?}");
    }

    #[test]
    fn evm_reports_the_eye_closing() {
        let pattern: Vec<usize> = (0..64).map(|i| i % 4).collect();
        let clean = spread(0.0, 500.0, &pattern, 5.0);
        let noisy = spread(0.0, 500.0, &pattern, 200.0);
        assert!(fit(&clean).evm(&clean) < 0.05);
        assert!(fit(&noisy).evm(&noisy) > 0.15);
    }

    #[test]
    fn a_flat_carrier_has_no_fit() {
        assert!(levels(&mut Vec::new(), &[1_000.0; 64]).is_none());
    }
}
