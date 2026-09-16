//! The twenty point transform the vocoder ends in, and the one its excitation
//! goes through.
//!
//! Small enough that a direct transform is the whole cost: twenty points and
//! eleven bins, once per five samples. It is plain arithmetic on a slice
//! rather than a tensor graph, which on a card costs one copy each way and
//! saves a graph of complex operations candle would run a kernel per step of.
//!
//! Centred like PyTorch's: the signal is reflected by half a window at each
//! end so that frame `k` is centred on sample `k * hop`, and the inverse
//! divides out the overlapped window energy and trims that padding again.

pub struct Stft {
    n_fft: usize,
    hop: usize,
    window: Vec<f32>,
}

impl Stft {
    pub fn new(n_fft: usize, hop: usize) -> Self {
        // Periodic Hann, which is what torch.hann_window gives by default and
        // what makes the overlap add sum to a constant.
        let window = (0..n_fft)
            .map(|i| {
                let t = std::f64::consts::TAU * i as f64 / n_fft as f64;
                (0.5 - 0.5 * t.cos()) as f32
            })
            .collect();
        Self { n_fft, hop, window }
    }

    pub fn bins(&self) -> usize {
        self.n_fft / 2 + 1
    }

    /// Magnitude and phase, each `[bins, frames]` flattened by row.
    pub fn transform(&self, samples: &[f32]) -> (Vec<f32>, Vec<f32>) {
        let half = self.n_fft / 2;
        let padded = reflect_pad(samples, half);
        let frames = samples.len() / self.hop + 1;
        let bins = self.bins();
        let mut mag = vec![0f32; bins * frames];
        let mut phase = vec![0f32; bins * frames];
        for f in 0..frames {
            let start = f * self.hop;
            for k in 0..bins {
                let (mut re, mut im) = (0f64, 0f64);
                for n in 0..self.n_fft {
                    let v = (padded[start + n] * self.window[n]) as f64;
                    let a = -std::f64::consts::TAU * (k * n) as f64 / self.n_fft as f64;
                    re += v * a.cos();
                    im += v * a.sin();
                }
                mag[k * frames + f] = (re * re + im * im).sqrt() as f32;
                phase[k * frames + f] = im.atan2(re) as f32;
            }
        }
        (mag, phase)
    }

    /// Samples from a magnitude and a phase, each `[bins, frames]` by row.
    pub fn inverse(&self, mag: &[f32], phase: &[f32], frames: usize) -> Vec<f32> {
        let bins = self.bins();
        let half = self.n_fft / 2;
        let padded_len = (frames - 1) * self.hop + self.n_fft;
        let mut out = vec![0f64; padded_len];
        let mut weight = vec![0f64; padded_len];
        for f in 0..frames {
            // The real signal of a frame, from the half spectrum: every bin
            // but DC and Nyquist stands for a conjugate pair, hence the two.
            let mut frame = vec![0f64; self.n_fft];
            for (n, v) in frame.iter_mut().enumerate() {
                let mut acc = 0f64;
                for k in 0..bins {
                    let m = mag[k * frames + f] as f64;
                    let p = phase[k * frames + f] as f64;
                    let a = std::f64::consts::TAU * (k * n) as f64 / self.n_fft as f64 + p;
                    let scale = match k == 0 || (self.n_fft.is_multiple_of(2) && k == bins - 1) {
                        true => 1.0,
                        false => 2.0,
                    };
                    acc += scale * m * a.cos();
                }
                *v = acc / self.n_fft as f64;
            }
            let start = f * self.hop;
            for n in 0..self.n_fft {
                let w = self.window[n] as f64;
                out[start + n] += frame[n] * w;
                weight[start + n] += w * w;
            }
        }
        let len = (frames - 1) * self.hop;
        (0..len)
            .map(|i| {
                let at = i + half;
                match weight[at] > 1e-11 {
                    true => (out[at] / weight[at]) as f32,
                    false => 0.0,
                }
            })
            .collect()
    }
}

fn reflect_pad(x: &[f32], pad: usize) -> Vec<f32> {
    let mut out = Vec::with_capacity(x.len() + 2 * pad);
    out.extend((1..=pad).rev().map(|i| x[i]));
    out.extend_from_slice(x);
    out.extend((1..=pad).map(|i| x[x.len() - 1 - i]));
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A transform and its inverse give back what went in, away from the ends
    /// where the overlap is incomplete.
    #[test]
    fn the_transform_round_trips() {
        let stft = Stft::new(20, 5);
        let x: Vec<f32> = (0..2000)
            .map(|i| (i as f32 * 0.03).sin() * 0.5 + (i as f32 * 0.17).cos() * 0.2)
            .collect();
        let (mag, phase) = stft.transform(&x);
        let frames = mag.len() / stft.bins();
        let y = stft.inverse(&mag, &phase, frames);
        assert_eq!(y.len(), x.len());
        let worst = x
            .iter()
            .zip(&y)
            .skip(40)
            .take(x.len() - 80)
            .map(|(a, b)| (a - b).abs())
            .fold(0.0, f32::max);
        assert!(worst < 1e-3, "round trip off by {worst}");
    }
}
