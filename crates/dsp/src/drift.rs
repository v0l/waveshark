//! How far apart two receivers hear the same band.
//!
//! Two tuners on separate crystals disagree about where a signal is, because
//! each one's local oscillator is off by its own few parts per million. Give
//! this the same piece of spectrum as each of them heard it, and it says how
//! far the second is above the first, in hertz. Nothing here knows why there
//! are two receivers: a stitched span uses it on its overlap, and so could a
//! pair of radios on one aerial.
//!
//! The estimate comes from a steady carrier rather than from correlating the
//! two streams. Uncoupled receivers slip against each other by whole blocks,
//! and a delay of a millisecond destroys the correlation of a 500 kHz band
//! outright, while a carrier sits at the same frequency whenever it is
//! looked at. So each side gets an averaged power spectrum, the strongest bin
//! is interpolated against its neighbours, and the difference is the answer.
//! The price is that the band has to hold something steady: with nothing but
//! noise in it there is no estimate, and a caller holds what it had.

use common::C32;
use rustfft::{Fft, FftPlanner};
use std::sync::Arc;

/// The offset between two views of one band.
pub struct Drift {
    rate: f64,
    bins: usize,
    fft: Arc<dyn Fft<f32>>,
    window: Vec<f32>,
    /// Averaged power per bin for each side, kept across blocks so a short
    /// block still contributes.
    power: [Vec<f32>; 2],
    /// Blocks folded into the averages so far.
    seen: usize,
    /// How far a bin has to stand over the median of the band before it is a
    /// carrier rather than the noise the band is made of. Thirteen dB: the
    /// loudest bin of 4096 bins of noise alone sits about 10.8 dB over the
    /// median, so a lower gate corrects the tuners against a noise peak.
    floor_db: f32,
    /// The running answer, smoothed. A crystal's error moves with its
    /// temperature over minutes, so nothing here has to be quick.
    estimate: Option<f64>,
    /// Weight of a new measurement in the running answer.
    alpha: f64,
}

/// Blocks averaged before an estimate is offered.
const AVERAGES: usize = 8;

impl Drift {
    /// An estimator for two streams at `rate`, comparing over `bins`.
    ///
    /// The bin width is the resolution before interpolation, so 4096 bins of
    /// a 600 kHz extract is 146 Hz, and a parabola through the peak gets that
    /// to tens of hertz, which is hundredths of a ppm at UHF.
    pub fn new(rate: f64, bins: usize) -> Self {
        let bins = bins.next_power_of_two();
        Self {
            rate,
            bins,
            fft: FftPlanner::new().plan_fft_forward(bins),
            window: crate::window::hann(bins),
            power: [vec![0.0; bins], vec![0.0; bins]],
            seen: 0,
            floor_db: 13.0,
            estimate: None,
            alpha: 0.25,
        }
    }

    /// Feed one block from each side and return the running estimate of how
    /// far `b` is above `a`, in hertz.
    ///
    /// Blocks need not be the same length or line up in time.
    pub fn feed(&mut self, a: &[C32], b: &[C32]) -> Option<f64> {
        if a.len() < self.bins || b.len() < self.bins {
            return self.estimate;
        }
        for (side, buf) in [a, b].into_iter().enumerate() {
            self.fold(side, buf);
        }
        self.seen += 1;
        if self.seen < AVERAGES {
            return self.estimate;
        }
        self.seen = 0;
        let (pa, pb) = (self.peak(0), self.peak(1));
        self.power.iter_mut().for_each(|p| p.iter_mut().for_each(|x| *x = 0.0));
        let (pa, pb) = (pa?, pb?);
        let now = pb - pa;
        // A step of more than a bin's worth of ppm is a different carrier on
        // each side, not a crystal that moved.
        self.estimate = Some(match self.estimate {
            Some(was) => was + (now - was) * self.alpha,
            None => now,
        });
        self.estimate
    }

    /// The running answer without feeding it anything.
    pub fn estimate(&self) -> Option<f64> {
        self.estimate
    }

    /// Forget both the averages and the answer.
    pub fn reset(&mut self) {
        self.power.iter_mut().for_each(|p| p.iter_mut().for_each(|x| *x = 0.0));
        self.seen = 0;
        self.estimate = None;
    }

    fn fold(&mut self, side: usize, buf: &[C32]) {
        let mut scratch: Vec<C32> =
            buf[..self.bins].iter().zip(&self.window).map(|(s, w)| *s * *w).collect();
        self.fft.process(&mut scratch);
        for (p, s) in self.power[side].iter_mut().zip(scratch.iter()) {
            *p += s.norm_sqr();
        }
    }

    /// The strongest carrier in one side's averaged spectrum, as an offset
    /// from the centre in hertz, or nothing when the band holds only noise.
    fn peak(&self, side: usize) -> Option<f64> {
        let p = &self.power[side];
        let (at, best) =
            p.iter().enumerate().fold((0usize, 0.0f32), |(i, v), (j, x)| match *x > v {
                true => (j, *x),
                false => (i, v),
            });
        if best <= 0.0 {
            return None;
        }
        let mut sorted: Vec<f32> = p.clone();
        sorted.sort_by(|x, y| x.partial_cmp(y).unwrap());
        let median = sorted[sorted.len() / 2].max(f32::MIN_POSITIVE);
        if 10.0 * (best / median).log10() < self.floor_db {
            return None;
        }
        // Three points through a parabola, in decibels, which is where a
        // windowed carrier's shape is nearest to one.
        let n = p.len();
        let db = |i: usize| 10.0 * p[i % n].max(f32::MIN_POSITIVE).log10();
        let (l, c, r) = (db((at + n - 1) % n), db(at), db((at + 1) % n));
        let denom = l - 2.0 * c + r;
        let delta = match denom.abs() > f32::EPSILON {
            true => (0.5 * (l - r) / denom).clamp(-0.5, 0.5),
            false => 0.0,
        };
        let bin = at as f64 + delta as f64;
        // Bins past the middle are negative frequencies.
        let bin = if bin > n as f64 / 2.0 { bin - n as f64 } else { bin };
        Some(bin * self.rate / n as f64)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn tone(rate: f64, hz: f64, n: usize, noise: f32) -> Vec<C32> {
        let mut seed = 0x1234_5678u32;
        let mut rnd = move || {
            seed = seed.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            (seed >> 8) as f32 / (1 << 23) as f32 - 1.0
        };
        (0..n)
            .map(|k| {
                let p = std::f64::consts::TAU * hz * k as f64 / rate;
                C32::new(p.cos() as f32, p.sin() as f32) + C32::new(rnd(), rnd()) * noise
            })
            .collect()
    }

    /// Two receivers hearing one carrier, one of them 8.66 kHz high, which is
    /// what 20 ppm at 433 MHz looks like.
    #[test]
    fn it_measures_how_far_the_second_receiver_is_out() {
        let rate = 500_000.0;
        let mut d = Drift::new(rate, 4096);
        let mut got = None;
        for _ in 0..AVERAGES {
            got = d.feed(&tone(rate, 40_000.0, 4096, 0.2), &tone(rate, 48_660.0, 4096, 0.2));
        }
        let hz = got.expect("an estimate");
        // Measured: 8659.9 Hz against 8660 asked for, so under a hertz on a
        // 122 Hz bin, which the parabola buys.
        assert!((hz - 8_660.0).abs() < 5.0, "said {hz:.1} Hz, wanted 8660");
    }

    /// A band with nothing steady in it is no estimate at all, rather than a
    /// correction taken off a noise peak.
    #[test]
    fn noise_alone_says_nothing() {
        let rate = 500_000.0;
        let mut d = Drift::new(rate, 4096);
        let noise = |seed: f64| {
            let mut s = seed as u32 | 1;
            (0..4096)
                .map(|_| {
                    s = s.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
                    let a = (s >> 8) as f32 / (1 << 23) as f32 - 1.0;
                    s = s.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
                    let b = (s >> 8) as f32 / (1 << 23) as f32 - 1.0;
                    C32::new(a, b)
                })
                .collect::<Vec<_>>()
        };
        for _ in 0..AVERAGES * 2 {
            d.feed(&noise(11.0), &noise(97.0));
        }
        assert_eq!(d.estimate(), None);
    }

    /// Two receivers in step is nothing to correct.
    #[test]
    fn receivers_in_step_read_zero() {
        let rate = 500_000.0;
        let mut d = Drift::new(rate, 4096);
        let mut got = None;
        for _ in 0..AVERAGES {
            got = d.feed(&tone(rate, -30_000.0, 4096, 0.1), &tone(rate, -30_000.0, 4096, 0.1));
        }
        assert!(got.unwrap().abs() < 1.0, "said {:.2} Hz", got.unwrap());
    }

    #[test]
    fn a_reset_forgets_the_answer() {
        let rate = 500_000.0;
        let mut d = Drift::new(rate, 1024);
        for _ in 0..AVERAGES {
            d.feed(&tone(rate, 10_000.0, 1024, 0.1), &tone(rate, 12_000.0, 1024, 0.1));
        }
        assert!(d.estimate().is_some());
        d.reset();
        assert_eq!(d.estimate(), None);
    }
}
