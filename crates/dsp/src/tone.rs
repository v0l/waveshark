//! What frequency is this piece of audio?
//!
//! One question, asked of a short window of real samples: the strongest tone
//! in it, to better than a bin. Anything that carries its information in the
//! frequency of an audio tone wants this, which is SSTV today and weather fax
//! or a tone-keyed telemetry link tomorrow.
//!
//! A Hann window, a real FFT, the largest bin, and a barycentric
//! interpolation across its neighbours for the fraction of a bin. The
//! interpolation is what makes a 4 ms window good enough to tell 1500 Hz from
//! 1503 Hz, which is one shade of grey in an SSTV picture.

use rustfft::num_complex::Complex32;
use rustfft::{Fft, FftPlanner};
use std::collections::HashMap;
use std::sync::Arc;

pub struct ToneMeter {
    rate: f64,
    planner: FftPlanner<f32>,
    plans: HashMap<usize, Arc<dyn Fft<f32>>>,
    window: HashMap<usize, Vec<f32>>,
    buf: Vec<Complex32>,
}

impl ToneMeter {
    pub fn new(rate: f64) -> Self {
        Self {
            rate,
            planner: FftPlanner::new(),
            plans: HashMap::new(),
            window: HashMap::new(),
            buf: Vec::new(),
        }
    }

    pub fn rate(&self) -> f64 {
        self.rate
    }

    /// The strongest tone in `samples`, in hertz. Windows of any length are
    /// allowed and each length's plan and window are kept, since a decoder
    /// asks the same few lengths many thousands of times.
    pub fn peak_hz(&mut self, samples: &[f32]) -> f64 {
        let n = samples.len();
        let mut mags = Vec::new();
        self.magnitudes(samples, &mut mags);
        let peak = mags
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.partial_cmp(b.1).unwrap_or(std::cmp::Ordering::Equal))
            .map(|(i, _)| i)
            .unwrap_or(0);
        self.bin_hz(&mags, peak, n)
    }

    /// Every bin of the window's magnitude spectrum up to Nyquist, appended
    /// to `out`. For a caller choosing a tone by something other than which
    /// is loudest: a keyed tone is the one whose level moves, and only the
    /// whole spectrum says which that is.
    pub fn magnitudes(&mut self, samples: &[f32], out: &mut Vec<f32>) {
        let n = samples.len();
        if n < 4 {
            return;
        }
        let planner = &mut self.planner;
        let fft = self.plans.entry(n).or_insert_with(|| planner.plan_fft_forward(n)).clone();
        let window = self.window.entry(n).or_insert_with(|| hann(n));

        self.buf.clear();
        self.buf.extend(samples.iter().zip(window.iter()).map(|(s, w)| Complex32::new(s * w, 0.0)));
        fft.process(&mut self.buf);

        // Real input, so only the first half says anything.
        let half = n / 2 + 1;
        out.extend(self.buf[..half].iter().map(|c| c.norm()));
    }

    /// Where a peak at bin `at` of an `n` point window really sits, in
    /// hertz, with its neighbours saying where between the bins it is.
    pub fn bin_hz(&self, mags: &[f32], at: usize, n: usize) -> f64 {
        if mags.is_empty() || n == 0 {
            return 0.0;
        }
        (interpolate(mags, at.min(mags.len() - 1)) * self.rate) / n as f64
    }

    /// The bin a frequency falls in, for a window of `n` samples.
    pub fn bin_of(&self, hz: f64, n: usize) -> usize {
        ((hz * n as f64 / self.rate).round().max(0.0)) as usize
    }
}

/// The peak's position in bins, using the two neighbours to find where
/// between them it really is.
fn interpolate(mags: &[f32], at: usize) -> f64 {
    let left = if at == 0 { mags[at] } else { mags[at - 1] };
    let right = if at + 1 >= mags.len() { mags[at] } else { mags[at + 1] };
    let denom = left + mags[at] + right;
    if denom == 0.0 {
        return 0.0;
    }
    at as f64 + ((right - left) / denom) as f64
}

fn hann(n: usize) -> Vec<f32> {
    // The symmetric window, as every decoder this is checked against uses.
    (0..n)
        .map(|i| {
            let x = std::f64::consts::PI * i as f64 / (n - 1) as f64;
            (x.sin() * x.sin()) as f32
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn tone(hz: f64, rate: f64, n: usize) -> Vec<f32> {
        (0..n).map(|i| (std::f64::consts::TAU * hz * i as f64 / rate).sin() as f32).collect()
    }

    /// A tone between two bins is found between them, which is the whole
    /// reason for the interpolation: at these window lengths a bin is 50 Hz.
    /// The barycentric estimate is biased towards the bin centre, so a tone
    /// half way between two bins is the worst case and lands about 6 Hz low;
    /// that bias is what the reference decoders have too.
    #[test]
    fn a_tone_off_the_bin_grid_is_still_read_closely() {
        let rate = 44_100.0;
        let n = 882; // 20 ms, a 50 Hz bin
        let mut m = ToneMeter::new(rate);
        for hz in [1200.0, 1500.0, 1900.0, 2300.0, 1723.0] {
            let got = m.peak_hz(&tone(hz, rate, n));
            assert!((got - hz).abs() < 8.0, "{hz} Hz read as {got:.1}");
        }
    }

    /// The window a picture is sampled with is a few hundred samples, and
    /// that is where the estimate has to hold up.
    #[test]
    fn a_short_window_still_names_the_tone() {
        let rate = 44_100.0;
        let n = 215; // about 4.9 ms, which is one Martin 1 pixel window
        let mut m = ToneMeter::new(rate);
        for hz in [1500.0, 1800.0, 2300.0] {
            let got = m.peak_hz(&tone(hz, rate, n));
            assert!((got - hz).abs() < 25.0, "{hz} Hz read as {got:.1}");
        }
    }
}
