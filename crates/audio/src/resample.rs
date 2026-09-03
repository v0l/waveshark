//! Arbitrary-ratio resampling.
//!
//! Prefer an exact integer chain where possible: 2.304 MS/s / 8 / 6 is exactly
//! 48 kHz. This is for devices that insist on 44.1 kHz.

/// Windowed-sinc resampler. Linear interpolation folds HF content back into
/// the audio band as hiss; a short sinc is transparent for a few more mults.
pub struct Resampler {
    ratio: f64,
    /// Fractional read position within `hist`.
    pos: f64,
    hist: Vec<f32>,
    /// Half-width of the interpolation kernel, in input samples.
    half: usize,
}

impl Resampler {
    /// `in_rate` to `out_rate`. `quality` is the kernel half-width; 8 is
    /// transparent for audio, 4 is cheaper and fine for voice.
    pub fn new(in_rate: f64, out_rate: f64, quality: usize) -> Self {
        let half = quality.max(2);
        Self {
            ratio: in_rate / out_rate,
            // `hist` holds 2*half zeros standing for input[-2half..0], so the
            // first real sample lands at index 2*half. Starting `pos` there
            // makes output[0] align with input[0].
            pos: (half * 2) as f64,
            hist: vec![0.0; half * 2],
            half,
        }
    }

    pub fn ratio(&self) -> f64 {
        self.ratio
    }

    /// Nudge the ratio while running, for drift tracking.
    pub fn set_ratio(&mut self, ratio: f64) {
        self.ratio = ratio.max(1e-6);
    }

    pub fn reset(&mut self) {
        self.hist.clear();
        self.hist.resize(self.half * 2, 0.0);
        self.pos = (self.half * 2) as f64;
    }

    /// Resample `input`, appending to `out`.
    pub fn process(&mut self, input: &[f32], out: &mut Vec<f32>) {
        self.hist.extend_from_slice(input);
        out.reserve((input.len() as f64 / self.ratio) as usize + 1);

        // Stop while a full kernel fits, so no output needs future samples.
        let limit = self.hist.len().saturating_sub(self.half) as f64;
        while self.pos < limit {
            let i = self.pos.floor() as usize;
            let frac = self.pos - i as f64;
            let mut acc = 0.0f32;
            for k in 0..self.half * 2 {
                let idx = i + k - self.half + 1;
                if idx >= self.hist.len() {
                    break;
                }
                let x = (k as f64 - self.half as f64 + 1.0) - frac;
                acc += self.hist[idx] * kernel(x, self.half as f64) as f32;
            }
            out.push(acc);
            self.pos += self.ratio;
        }

        // Discard consumed history, keeping enough for the next kernel. At a
        // large ratio the read position can have stepped past the end of
        // what is held, so the drain is capped at what there is and the
        // position keeps the remainder.
        let keep = self.half * 2;
        let consumed = (self.pos.floor() as usize).saturating_sub(keep).min(self.hist.len());
        if consumed > 0 {
            self.hist.drain(..consumed);
            self.pos -= consumed as f64;
        }
    }
}

/// Lanczos kernel: a sinc windowed by a wider sinc.
fn kernel(x: f64, a: f64) -> f64 {
    if x.abs() < 1e-9 {
        return 1.0;
    }
    if x.abs() >= a {
        return 0.0;
    }
    let px = std::f64::consts::PI * x;
    (px.sin() / px) * ((px / a).sin() / (px / a))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_large_ratio_does_not_overrun_the_history() {
        // 2.4 MS/s straight to 48 kHz is a ratio of fifty, and the read
        // position steps past the end of the block by up to that much.
        let mut r = Resampler::new(2_400_000.0, 48_000.0, 8);
        let mut out = Vec::new();
        for _ in 0..4 {
            r.process(&vec![0.5f32; 65_536], &mut out);
        }
        assert!(out.len() > 5_000 && out.len() < 5_600, "{} out", out.len());
    }

    fn tone(n: usize, hz: f64, rate: f64) -> Vec<f32> {
        (0..n)
            .map(|i| {
                let p = (hz * i as f64 / rate).rem_euclid(1.0) * std::f64::consts::TAU;
                p.sin() as f32
            })
            .collect()
    }

    fn rms(v: &[f32]) -> f32 {
        (v.iter().map(|x| x * x).sum::<f32>() / v.len() as f32).sqrt()
    }

    /// Frequency estimate by counting zero crossings; crude but independent of
    /// the resampler's own maths.
    fn freq_of(v: &[f32], rate: f64) -> f64 {
        let mut crossings = 0;
        for w in v.windows(2) {
            if w[0] <= 0.0 && w[1] > 0.0 {
                crossings += 1;
            }
        }
        crossings as f64 * rate / v.len() as f64
    }

    #[test]
    fn preserves_a_tone_across_a_rate_change() {
        // 50 kHz to 48 kHz, the awkward ratio this exists for.
        let mut r = Resampler::new(50_000.0, 48_000.0, 8);
        let input = tone(50_000, 1_000.0, 50_000.0);
        let mut out = Vec::new();
        r.process(&input, &mut out);

        assert!(
            (out.len() as f64 - 48_000.0).abs() < 100.0,
            "expected about 48000 samples, got {}",
            out.len()
        );
        let f = freq_of(&out[1000..], 48_000.0);
        assert!((f - 1_000.0).abs() < 5.0, "tone came out at {f:.1} Hz");
        assert!((rms(&out[1000..]) - rms(&input[1000..])).abs() < 0.02, "level changed");
    }

    #[test]
    fn upsampling_does_not_change_pitch() {
        let mut r = Resampler::new(8_000.0, 48_000.0, 8);
        let input = tone(8_000, 500.0, 8_000.0);
        let mut out = Vec::new();
        r.process(&input, &mut out);
        let f = freq_of(&out[500..], 48_000.0);
        assert!((f - 500.0).abs() < 5.0, "tone came out at {f:.1} Hz");
    }

    #[test]
    fn block_boundaries_are_seamless() {
        let input = tone(20_000, 1_000.0, 50_000.0);

        let mut a = Resampler::new(50_000.0, 48_000.0, 8);
        let mut whole = Vec::new();
        a.process(&input, &mut whole);

        let mut b = Resampler::new(50_000.0, 48_000.0, 8);
        let mut split = Vec::new();
        for c in input.chunks(377) {
            b.process(c, &mut split);
        }

        assert_eq!(whole.len(), split.len(), "sample count differs across block splits");
        for (i, (x, y)) in whole.iter().zip(&split).enumerate() {
            assert!((x - y).abs() < 1e-5, "sample {i} differs: {x} vs {y}");
        }
    }

    #[test]
    fn a_unity_ratio_is_close_to_transparent() {
        let mut r = Resampler::new(48_000.0, 48_000.0, 8);
        let input = tone(4_800, 1_000.0, 48_000.0);
        let mut out = Vec::new();
        r.process(&input, &mut out);

        let worst = (100..1_100)
            .map(|i| (out[i] - input[i]).abs())
            .fold(0.0f32, f32::max);
        assert!(worst < 0.02, "unity resample is not transparent, worst error {worst:.4}");
    }
}
