use common::C32;

pub const SETTLE_SECONDS: f64 = 0.25;

pub struct IqBalance {
    ii: f64,
    qq: f64,
    iq: f64,
    tau: f64,
    seen: f64,
}

impl IqBalance {
    pub fn new(rate: f64) -> Self {
        Self { ii: 0.0, qq: 0.0, iq: 0.0, tau: (rate * SETTLE_SECONDS).max(1.0), seen: 0.0 }
    }

    pub fn reset(&mut self) {
        self.seen = 0.0;
    }

    pub fn amplitude_db(&self) -> f64 {
        10.0 * (self.qq / self.ii.max(f64::MIN_POSITIVE)).log10()
    }

    pub fn phase_deg(&self) -> f64 {
        (self.iq / (self.ii * self.qq).sqrt().max(f64::MIN_POSITIVE)).asin().to_degrees()
    }

    pub fn process(&mut self, buf: &mut [C32]) {
        if buf.is_empty() {
            return;
        }
        let (mut ii, mut qq, mut iq) = (0.0f64, 0.0f64, 0.0f64);
        for s in buf.iter() {
            let (i, q) = (s.re as f64, s.im as f64);
            ii += i * i;
            qq += q * q;
            iq += i * q;
        }
        let n = buf.len() as f64;
        let (ii, qq, iq) = (ii / n, qq / n, iq / n);
        let w = (n / (self.seen + n)).max(1.0 - (-n / self.tau).exp());
        self.seen += n;
        self.ii += w * (ii - self.ii);
        self.qq += w * (qq - self.qq);
        self.iq += w * (iq - self.iq);
        let rest = self.qq - self.iq * self.iq / self.ii.max(f64::MIN_POSITIVE);
        if self.ii <= f64::MIN_POSITIVE || rest <= f64::MIN_POSITIVE {
            return;
        }
        let mu = (self.iq / self.ii) as f32;
        let g = (self.ii / rest).sqrt() as f32;
        for s in buf.iter_mut() {
            s.im = g * (s.im - mu * s.re);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const RATE: f64 = 768_000.0;

    fn skewed(n: usize, tones: &[(f64, f64)], gain: f64, phase_deg: f64, noise: f64) -> Vec<C32> {
        let (sp, cp) = phase_deg.to_radians().sin_cos();
        let mut seed = 0x2545_f491_4f6c_dd1du64;
        let mut uniform = move || {
            seed ^= seed << 13;
            seed ^= seed >> 7;
            seed ^= seed << 17;
            (seed >> 11) as f64 / (1u64 << 53) as f64 - 0.5
        };
        (0..n)
            .map(|k| {
                let (mut i, mut q) = (noise * uniform(), noise * uniform());
                for (hz, a) in tones {
                    let t = std::f64::consts::TAU * hz * k as f64 / RATE;
                    i += a * t.cos();
                    q += a * t.sin();
                }
                C32::new(i as f32, (gain * (q * cp - i * sp)) as f32)
            })
            .collect()
    }

    fn at_db(x: &[C32], hz: f64) -> f64 {
        let step = -std::f64::consts::TAU * hz / RATE;
        let (mut re, mut im) = (0.0f64, 0.0f64);
        for (k, s) in x.iter().enumerate() {
            let (sn, cs) = (step * k as f64).sin_cos();
            re += s.re as f64 * cs - s.im as f64 * sn;
            im += s.re as f64 * sn + s.im as f64 * cs;
        }
        20.0 * ((re * re + im * im).sqrt() / x.len() as f64).log10()
    }

    fn balance(mut x: Vec<C32>) -> (Vec<C32>, IqBalance) {
        let mut b = IqBalance::new(RATE);
        for block in x.chunks_mut(4096) {
            b.process(block);
        }
        (x, b)
    }

    fn image_db(x: &[C32], hz: f64) -> f64 {
        at_db(x, -hz) - at_db(x, hz)
    }

    #[test]
    fn a_five_percent_three_degree_skew_is_read_back_and_its_image_goes_from_29_to_below_80_db() {
        let x = skewed(RATE as usize, &[(50_000.0, 0.5)], 1.05, 3.0, 0.0);
        let before = image_db(&x[..32_768], 50_000.0);
        let (out, b) = balance(x);
        let after = image_db(&out[out.len() - 32_768..], 50_000.0);
        assert!((-29.5..-28.5).contains(&before), "the skew put the image at {before:.1} dB");
        assert!(after < -80.0, "the image is left at {after:.1} dB");
        assert!((b.amplitude_db() - 0.424).abs() < 0.01, "amplitude {:.3} dB", b.amplitude_db());
        assert!((b.phase_deg() + 3.0).abs() < 0.01, "phase {:.3} degrees", b.phase_deg());
    }

    #[test]
    fn four_stations_over_noise_each_keep_their_image_69_db_below_them() {
        let tones = [(-230_000.0, 0.3), (-41_000.0, 0.05), (12_500.0, 0.2), (187_000.0, 0.02)];
        let x = skewed(2 * RATE as usize, &tones, 0.97, -1.5, 0.01);
        let (out, _) = balance(x);
        let tail = &out[out.len() - 262_144..];
        for (hz, a) in tones {
            let image = at_db(tail, -hz) - 20.0 * a.log10();
            assert!(image < -69.0, "the image of {hz} Hz is at {image:.1} dB below it");
        }
    }

    #[test]
    fn a_balanced_stream_comes_through_untouched() {
        let x = skewed(200_000, &[(50_000.0, 0.5), (-120_000.0, 0.1)], 1.0, 0.0, 0.1);
        let (out, b) = balance(x.clone());
        let worst = x.iter().zip(&out).map(|(a, b)| (a - b).norm()).fold(0.0f32, f32::max);
        assert!(worst < 5e-3, "a balanced sample moved by {worst}");
        assert!(b.amplitude_db().abs() < 0.05 && b.phase_deg().abs() < 0.2);
    }

    #[test]
    fn silence_is_left_alone() {
        let (out, _) = balance(vec![C32::new(0.0, 0.0); 10_000]);
        assert!(out.iter().all(|s| s.norm() == 0.0));
    }
}
