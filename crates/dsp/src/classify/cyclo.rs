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
use rustfft::FftPlanner;

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
    correlate(z.iter().map(|s| *s), z.len(), lag_min)
}

/// Autocorrelation of envelope power, with its mean removed.
pub fn envelope(z: &[C32], lag_min: usize) -> Cyclic {
    let take = z.len().min(1 << 17);
    let mean = z[..take].iter().map(|s| s.norm_sqr()).sum::<f32>() / take.max(1) as f32;
    correlate(z.iter().map(|s| C32::new(s.norm_sqr() - mean, 0.0)), z.len(), lag_min)
}

fn correlate(src: impl Iterator<Item = C32>, len: usize, lag_min: usize) -> Cyclic {
    let take = len.min(1 << 17);
    if take < 4 * lag_min.max(1) {
        return Cyclic { peak: 0.0, lag: 0, ratio: 1.0 };
    }
    let n = (2 * take).next_power_of_two();
    let mut planner = FftPlanner::<f32>::new();
    let mut buf = vec![C32::new(0.0, 0.0); n];
    for (b, s) in buf.iter_mut().zip(src.take(take)) {
        *b = s;
    }
    planner.plan_fft_forward(n).process(&mut buf);
    for b in buf.iter_mut() {
        *b = C32::new(b.norm_sqr(), 0.0);
    }
    planner.plan_fft_inverse(n).process(&mut buf);

    let r0 = buf[0].re.max(1e-20);
    let hi = (take / 2).min(8192).max(lag_min + 1);
    let mut best = (0.0f32, 0usize);
    let mut vals: Vec<f32> = Vec::with_capacity(hi.saturating_sub(lag_min));
    for (k, b) in buf.iter().enumerate().take(hi).skip(lag_min) {
        let v = b.norm() / r0;
        vals.push(v);
        if v > best.0 {
            best = (v, k);
        }
    }
    if vals.is_empty() {
        return Cyclic { peak: 0.0, lag: 0, ratio: 1.0 };
    }
    vals.sort_by(f32::total_cmp);
    let median = vals[vals.len() / 2].max(1e-9);
    Cyclic { peak: best.0, lag: best.1, ratio: best.0 / median }
}

/// What autocorrelation noise alone reaches: a few times one over root N.
pub fn noise_bound(samples: usize) -> f32 {
    5.0 / (samples.min(1 << 17).max(1) as f32).sqrt()
}
