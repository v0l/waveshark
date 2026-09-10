//! Changing a stream's rate by a ratio no decimator can reach.
//!
//! Everything else in this crate decimates by a whole number, because that is
//! what a receiver usually wants: a channel is cut out of a span and the rate
//! it lands on is the span's over some integer. A decoder that insists on one
//! exact rate breaks that. An 802.11 symbol is 64 subcarriers 312.5 kHz apart,
//! so its receiver has to run at 20 MS/s and nothing else; from a LimeSDR's
//! 61.44 MS/s the reachable rates are 20.48, 15.36, 12.288, and none of them
//! is 20. Two and a half percent out is not a small error here: the transform
//! window slips a sample and a half per symbol and nothing decodes.
//!
//! So: interpolate by `l`, filter, keep one sample in `m`, all in one pass.
//! The filter is stored as `l` phases of the same prototype, and only the
//! phase an output needs is ever evaluated, so the cost is the taps per phase
//! per output sample rather than the whole filter at the interpolated rate.

use common::C32;

/// A polyphase resampler by a ratio of whole numbers.
pub struct Rational {
    l: usize,
    m: usize,
    /// Taps per phase, which is the cost of one output sample.
    per_phase: usize,
    /// `phases[p][k]` is tap `p + k * l` of the prototype.
    phases: Vec<Vec<f32>>,
    /// The newest samples, oldest first, `per_phase` of them.
    hist: Vec<C32>,
    acc: usize,
}

impl Rational {
    /// A resampler from `rate_in` to `rate_out`, or `None` when the ratio in
    /// lowest terms is too large to build a filter for.
    ///
    /// `max_denominator` bounds that: 61.44 to 20 MS/s is 125/384 and cheap,
    /// while two rates that share nothing would ask for a filter with a phase
    /// per output sample of a second.
    pub fn new(rate_in: f64, rate_out: f64, max_denominator: usize) -> Option<Self> {
        let (l, m) = ratio(rate_out, rate_in, max_denominator)?;
        Some(Self::with_ratio(l, m))
    }

    /// Interpolate by `l` and decimate by `m`, whatever those mean in rates.
    pub fn with_ratio(l: usize, m: usize) -> Self {
        assert!(l >= 1 && m >= 1);
        let per_phase = 24;
        // Designed at the interpolated rate, stopping below whichever Nyquist
        // is lower: the input's when interpolating, the output's when
        // decimating. A tenth of margin keeps the transition out of the band
        // a decoder cares about.
        let cutoff = 0.45 / l.max(m) as f64;
        let taps = crate::fir::lowpass((per_phase * l) | 1, cutoff, 60.0);
        let mut phases = vec![Vec::with_capacity(per_phase); l];
        for (p, phase) in phases.iter_mut().enumerate() {
            for k in 0..per_phase {
                // Gain `l`, because interpolation spreads one sample's energy
                // over `l` of them.
                phase.push(taps.get(p + k * l).copied().unwrap_or(0.0) * l as f32);
            }
        }
        Self { l, m, per_phase, phases, hist: vec![C32::default(); per_phase], acc: 0 }
    }

    /// Output samples per input sample, as the ratio it was built for.
    pub fn ratio(&self) -> f64 {
        self.l as f64 / self.m as f64
    }

    /// Whether this would pass its input through untouched.
    pub fn is_identity(&self) -> bool {
        self.l == 1 && self.m == 1
    }

    pub fn reset(&mut self) {
        self.hist.iter_mut().for_each(|x| *x = C32::default());
        self.acc = 0;
    }

    /// Resample `input`, appending to `out`. Continuous across calls.
    pub fn process(&mut self, input: &[C32], out: &mut Vec<C32>) {
        if self.is_identity() {
            out.extend_from_slice(input);
            return;
        }
        out.reserve(input.len() * self.l / self.m + 1);
        for &x in input {
            self.hist.rotate_left(1);
            self.hist[self.per_phase - 1] = x;
            while self.acc < self.l {
                let h = &self.phases[self.acc];
                let mut sum = C32::default();
                for (k, &t) in h.iter().enumerate() {
                    sum += self.hist[self.per_phase - 1 - k] * t;
                }
                out.push(sum);
                self.acc += self.m;
            }
            self.acc -= self.l;
        }
    }
}

/// `a / b` in lowest terms, or `None` when the denominator is over the bound.
fn ratio(a: f64, b: f64, max_denominator: usize) -> Option<(usize, usize)> {
    // Both rates are whole numbers of hertz in every case this receiver has,
    // so the ratio is exact rather than approximated.
    let (a, b) = (a.round() as u64, b.round() as u64);
    if a == 0 || b == 0 {
        return None;
    }
    let g = gcd(a, b);
    let (l, m) = ((a / g) as usize, (b / g) as usize);
    (l.max(m) <= max_denominator).then_some((l, m))
}

fn gcd(a: u64, b: u64) -> u64 {
    if b == 0 {
        a
    } else {
        gcd(b, a % b)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn tone(hz: f64, rate: f64, n: usize) -> Vec<C32> {
        (0..n)
            .map(|i| {
                let ph = std::f32::consts::TAU * (hz / rate) as f32 * i as f32;
                C32::new(ph.cos(), ph.sin())
            })
            .collect()
    }

    /// The frequency of the strongest bin, and its amplitude.
    fn peak(v: &[C32], rate: f64) -> (f64, f32) {
        use rustfft::FftPlanner;
        let n = 8192.min(v.len() / 2 * 2);
        let mut buf: Vec<C32> = v[v.len() - n..].to_vec();
        FftPlanner::new().plan_fft_forward(n).process(&mut buf);
        let (i, p) = buf
            .iter()
            .enumerate()
            .map(|(i, x)| (i, x.norm()))
            .max_by(|a, b| a.1.partial_cmp(&b.1).unwrap())
            .unwrap();
        let k = if i > n / 2 { i as f64 - n as f64 } else { i as f64 };
        (k * rate / n as f64, p / n as f32)
    }

    /// The one this exists for: a LimeSDR's 61.44 MS/s decimated by three
    /// lands on 20.48, and 802.11 needs 20.
    #[test]
    fn a_tone_keeps_its_frequency_and_amplitude_across_the_awkward_ratio() {
        let (rate_in, rate_out) = (20_480_000.0, 20_000_000.0);
        let mut r = Rational::new(rate_in, rate_out, 512).expect("a ratio");
        assert_eq!((r.l, r.m), (125, 128));

        let iq = tone(1_000_000.0, rate_in, 200_000);
        let mut out = Vec::new();
        // In uneven blocks, because a radio delivers what it delivers.
        for block in iq.chunks(4097) {
            r.process(block, &mut out);
        }
        let want = (iq.len() as f64 * r.ratio()) as usize;
        assert!(out.len().abs_diff(want) <= 2, "{} against {want}", out.len());
        let (hz, _) = peak(&out, rate_out);
        assert!((hz - 1_000_000.0).abs() < 5_000.0, "{hz} Hz");
        // The level of a tone is its own, not a bin's: 1 MHz does not land on
        // a bin of this transform and the scalloping alone is 2 dB.
        let settled = &out[out.len() / 2..];
        let rms = (settled.iter().map(|x| x.norm_sqr()).sum::<f32>() / settled.len() as f32).sqrt();
        assert!((rms - 1.0).abs() < 0.02, "level {rms}");
    }

    /// A tone above the output's Nyquist is stopped rather than folded back
    /// in, which is the whole job of the filter inside this.
    #[test]
    fn what_will_not_fit_in_the_output_is_filtered_out() {
        let (rate_in, rate_out) = (20_480_000.0, 5_120_000.0);
        let mut r = Rational::new(rate_in, rate_out, 512).expect("a ratio");
        assert_eq!((r.l, r.m), (1, 4));
        let mut out = Vec::new();
        r.process(&tone(4_000_000.0, rate_in, 100_000), &mut out);
        let settled = &out[out.len() / 2..];
        let rms = (settled.iter().map(|x| x.norm_sqr()).sum::<f32>() / settled.len() as f32).sqrt();
        assert!(rms < 0.01, "an alias came through at {rms}");
    }

    #[test]
    fn a_ratio_no_filter_could_hold_is_refused_and_one_to_one_is_free() {
        // Two rates with nothing in common.
        assert!(Rational::new(1_000_003.0, 48_000.0, 512).is_none());
        let mut r = Rational::new(20e6, 20e6, 512).expect("a ratio");
        assert!(r.is_identity());
        let iq = tone(1e6, 20e6, 1000);
        let mut out = Vec::new();
        r.process(&iq, &mut out);
        assert_eq!(out, iq);
    }
}
