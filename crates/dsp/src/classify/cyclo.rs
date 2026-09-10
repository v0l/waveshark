//! Cyclostationarity: does anything in this burst repeat at a fixed lag?
//!
//! Two measurements, and the difference between them is the point.
//!
//! The complex autocorrelation finds a cyclic prefix, because OFDM copies the
//! tail of every symbol to its head and those copies are identical samples.
//! It cannot find a spreading code: direct sequence keys the *sign* of each
//! symbol with data, and over a burst those flips cancel the correlation to
//! nothing. Squaring the envelope throws the sign away, so the same test on
//! envelope power finds the chip sequence that the complex one is blind to.
//!
//! Both report a peak and how far it stands above the median across lags.
//! The ratio is what makes them mean anything: a narrowband signal correlates
//! with itself at every small lag simply because it is narrowband, and a DC
//! offset correlates at all of them. Peak alone called an empty band OFDM.

use common::C32;
use rustfft::{Fft, FftPlanner};
use std::cell::RefCell;
use std::sync::Arc;

/// Samples a correlation is taken over. Lags run to 8192 at most, so a
/// window a few times that measures the same thing as the whole burst; the
/// whole burst cost a transform of 131072 points, twice, on every burst
/// classified, which was half of what a classification cost.
const TAKE: usize = 1 << 15;

/// A forward and inverse plan of one size.
type Pair = (Arc<dyn Fft<f32>>, Arc<dyn Fft<f32>>);

/// Plans made so far, by size, and the buffers the transforms run in.
#[derive(Default)]
struct Plans {
    planner: Option<FftPlanner<f32>>,
    made: Vec<(usize, Pair)>,
    /// The correlation buffer and the transform's own scratch, kept between
    /// bursts. Both are a quarter of a megabyte at the sizes used here, and
    /// `Fft::process` allocates and zeroes the scratch on every call: four
    /// transforms a burst were a megabyte of allocation for arithmetic that
    /// reuses the same two buffers every time.
    buf: Vec<C32>,
    scratch: Vec<C32>,
}

thread_local! {
    /// Reused: planning a large transform computes its twiddles, and a
    /// planner made per call did that on every burst.
    static PLANS: RefCell<Plans> = RefCell::new(Plans::default());
}

fn plans(p: &mut Plans, n: usize) -> Pair {
    if let Some((_, pair)) = p.made.iter().find(|(k, _)| *k == n) {
        return pair.clone();
    }
    let planner = p.planner.get_or_insert_with(FftPlanner::new);
    let pair = (planner.plan_fft_forward(n), planner.plan_fft_inverse(n));
    p.made.push((n, pair.clone()));
    pair
}

/// Peak, its lag, and the peak over the median across lags.
pub struct Cyclic {
    pub peak: f32,
    pub lag: usize,
    pub ratio: f32,
}

/// Least lag at which a correlation can mean periodicity rather than the
/// signal's own bandwidth.
///
/// A signal occupying a fraction `occ` of the span has a correlation width of
/// roughly `1/occ` samples, so anything inside that is measuring bandwidth.
pub fn lag_floor(occupied_fraction: f32) -> usize {
    ((8.0 / occupied_fraction.max(0.01)) as usize).clamp(8, 2048)
}

/// Autocorrelation of the samples themselves, by FFT.
pub fn complex(z: &[C32], lag_min: usize) -> Cyclic {
    correlate(z.iter().copied(), z.len(), lag_min)
}

/// Autocorrelation of envelope power, with its mean removed.
pub fn envelope(z: &[C32], lag_min: usize) -> Cyclic {
    let take = z.len().min(TAKE);
    let mean = z[..take].iter().map(|s| s.norm_sqr()).sum::<f32>() / take.max(1) as f32;
    correlate(z.iter().map(|s| C32::new(s.norm_sqr() - mean, 0.0)), z.len(), lag_min)
}

fn correlate(src: impl Iterator<Item = C32>, len: usize, lag_min: usize) -> Cyclic {
    let take = len.min(TAKE);
    if take < 4 * lag_min.max(1) {
        return Cyclic { peak: 0.0, lag: 0, ratio: 1.0 };
    }
    let n = (2 * take).next_power_of_two();
    let hi = (take / 2).min(8192).max(lag_min + 1);
    let mut best = (0.0f32, 0usize);
    let mut vals: Vec<f32> = Vec::with_capacity(hi.saturating_sub(lag_min));
    PLANS.with(|p| {
        let p = &mut *p.borrow_mut();
        let (forward, inverse) = plans(p, n);
        let need = forward.get_inplace_scratch_len().max(inverse.get_inplace_scratch_len());
        if p.scratch.len() < need {
            p.scratch.resize(need, C32::new(0.0, 0.0));
        }
        p.buf.clear();
        p.buf.resize(n, C32::new(0.0, 0.0));
        for (b, s) in p.buf.iter_mut().zip(src.take(take)) {
            *b = s;
        }
        forward.process_with_scratch(&mut p.buf, &mut p.scratch);
        for b in p.buf.iter_mut() {
            *b = C32::new(b.norm_sqr(), 0.0);
        }
        inverse.process_with_scratch(&mut p.buf, &mut p.scratch);

        let r0 = p.buf[0].re.max(1e-20);
        for (k, b) in p.buf.iter().enumerate().take(hi).skip(lag_min) {
            let v = b.norm() / r0;
            vals.push(v);
            if v > best.0 {
                best = (v, k);
            }
        }
    });
    if vals.is_empty() {
        return Cyclic { peak: 0.0, lag: 0, ratio: 1.0 };
    }
    vals.sort_by(f32::total_cmp);
    let median = vals[vals.len() / 2].max(1e-9);
    Cyclic { peak: best.0, lag: best.1, ratio: best.0 / median }
}

/// What autocorrelation noise alone reaches: a few times one over root N.
pub fn noise_bound(samples: usize) -> f32 {
    5.0 / (samples.clamp(1, 1 << 17) as f32).sqrt()
}
