use anyhow::Result;
use rustfft::{num_complex::Complex, FftPlanner};

/// Compute Hann window coefficients.
fn hann_window(n: usize) -> Vec<f32> {
    (0..n)
        .map(|i| {
            let x = 2.0 * std::f32::consts::PI * i as f32 / (n as f32 - 1.0);
            0.5 * (1.0 - x.cos())
        })
        .collect()
}

/// Reflection-pad a signal on both sides.
fn reflection_pad(signal: &[f32], pad: usize) -> Vec<f32> {
    let n = signal.len();
    let mut padded = Vec::with_capacity(n + 2 * pad);
    // Left padding: reflect from index 1..pad+1 in reverse
    for i in (1..=pad.min(n - 1)).rev() {
        padded.push(signal[i]);
    }
    padded.extend_from_slice(signal);
    // Right padding: reflect from n-pad-1..n-1 in reverse
    let right_start = if n > pad { n - pad - 1 } else { 0 };
    for i in (right_start..n - 1).rev() {
        padded.push(signal[i]);
    }
    padded
}

/// Compute STFT power spectrum: shape [n_freqs, num_frames].
fn compute_power_stft(
    signal: &[f32],
    n_fft: usize,
    hop_length: usize,
    window: &[f32],
) -> (Vec<f32>, usize, usize) {
    let n_freqs = n_fft / 2 + 1;
    let n_frames = if signal.len() >= n_fft { (signal.len() - n_fft) / hop_length + 1 } else { 0 };

    let mut planner = FftPlanner::<f32>::new();
    let fft = planner.plan_fft_forward(n_fft);

    // power stored as [n_freqs × n_frames] row-major
    let mut power = vec![0.0f32; n_freqs * n_frames];
    let mut frame_buf = vec![Complex::new(0.0f32, 0.0f32); n_fft];

    for i in 0..n_frames {
        let start = i * hop_length;
        for j in 0..n_fft {
            frame_buf[j] = Complex::new(signal[start + j] * window[j], 0.0);
        }
        fft.process(&mut frame_buf);
        for k in 0..n_freqs {
            let re = frame_buf[k].re;
            let im = frame_buf[k].im;
            power[k * n_frames + i] = re * re + im * im;
        }
    }

    (power, n_freqs, n_frames)
}

/// Create mel filterbank matrix (num_mel_bins × n_freqs).
fn create_mel_filterbank(
    num_mels: usize,
    n_fft: usize,
    sample_rate: u32,
    fmin: f64,
    fmax: f64,
) -> Vec<f32> {
    let n_freqs = n_fft / 2 + 1;
    let sr = sample_rate as f64;

    // Slaney mel scale
    let f_sp = 200.0 / 3.0;
    let min_log_hz = 1000.0;
    let min_log_mel = min_log_hz / f_sp;
    let logstep = (6.4f64).ln() / 27.0;

    let hz_to_mel = |f: f64| -> f64 {
        if f < min_log_hz {
            f / f_sp
        } else {
            min_log_mel + (f / min_log_hz).ln() / logstep
        }
    };

    let mel_to_hz = |m: f64| -> f64 {
        if m < min_log_mel {
            f_sp * m
        } else {
            min_log_hz * (logstep * (m - min_log_mel)).exp()
        }
    };

    let mel_min = hz_to_mel(fmin);
    let mel_max = hz_to_mel(fmax);

    let filter_freqs: Vec<f64> = (0..num_mels + 2)
        .map(|i| {
            let mel = mel_min + (mel_max - mel_min) * i as f64 / (num_mels + 1) as f64;
            mel_to_hz(mel)
        })
        .collect();

    let all_freqs: Vec<f64> = (0..n_freqs).map(|j| j as f64 * sr / n_fft as f64).collect();

    let f_diff: Vec<f64> = filter_freqs.windows(2).map(|w| w[1] - w[0]).collect();

    let mut filters = vec![0.0f32; num_mels * n_freqs];

    for j in 0..n_freqs {
        for i in 0..num_mels {
            let down = (all_freqs[j] - filter_freqs[i]) / f_diff[i];
            let up = (filter_freqs[i + 2] - all_freqs[j]) / f_diff[i + 1];
            let val = down.min(up).max(0.0);
            filters[i * n_freqs + j] = val as f32;
        }
    }

    // Slaney normalization
    for i in 0..num_mels {
        let enorm = 2.0 / (filter_freqs[i + 2] - filter_freqs[i]);
        for j in 0..n_freqs {
            filters[i * n_freqs + j] *= enorm as f32;
        }
    }

    filters
}

pub(crate) struct MelExtractor {
    n_fft: usize,
    hop_length: usize,
    num_mel_bins: usize,
    mel_filters: Vec<f32>, // [num_mel_bins × n_freqs]
    n_freqs: usize,
}

impl MelExtractor {
    pub(crate) fn new(
        n_fft: usize,
        hop_length: usize,
        num_mel_bins: usize,
        sample_rate: u32,
    ) -> Self {
        let n_freqs = n_fft / 2 + 1;
        let mel_filters =
            create_mel_filterbank(num_mel_bins, n_fft, sample_rate, 0.0, sample_rate as f64 / 2.0);
        Self { n_fft, hop_length, num_mel_bins, mel_filters, n_freqs }
    }

    /// Extract log-mel spectrogram.
    /// Returns flat vec [num_mel_bins × num_frames], and (num_mel_bins, num_frames).
    pub(crate) fn extract(&self, samples: &[f32]) -> Result<(Vec<f32>, usize, usize)> {
        // Pad to next multiple of hop_length
        let padded_len = samples.len().div_ceil(self.hop_length) * self.hop_length;
        let mut padded_samples = samples.to_vec();
        padded_samples.resize(padded_len, 0.0);

        // Reflection-pad center padding (n_fft/2 on each side)
        let pad = self.n_fft / 2;
        let padded_signal = reflection_pad(&padded_samples, pad);

        let window = hann_window(self.n_fft);

        // STFT power spectrum [n_freqs × n_frames]
        let (power, _n_freqs, n_frames_with_last) =
            compute_power_stft(&padded_signal, self.n_fft, self.hop_length, &window);

        // Remove last frame (match Python: magnitudes[..., :-1])
        let n_frames = if n_frames_with_last > 0 { n_frames_with_last - 1 } else { 0 };

        // Apply mel filterbank: [num_mel_bins × n_freqs] × [n_freqs × n_frames]
        //
        // Loop order m → f → t makes the innermost access `power_row[t]` contiguous
        // in memory (stride 1), enabling auto-vectorisation.  The original t → f order
        // had `power[f * n_frames_with_last + t]` with stride n_frames_with_last.
        // Accumulation order over f is identical in both forms → bit-identical output.
        let mut mel_spec = vec![0.0f32; self.num_mel_bins * n_frames];
        for m in 0..self.num_mel_bins {
            let filter_row = &self.mel_filters[m * self.n_freqs..(m + 1) * self.n_freqs];
            let out_row = &mut mel_spec[m * n_frames..(m + 1) * n_frames];
            for f in 0..self.n_freqs {
                let w = filter_row[f];
                if w == 0.0 {
                    continue;
                } // filterbank is sparse; skip zero weights
                let power_row = &power[f * n_frames_with_last..f * n_frames_with_last + n_frames];
                for (t, &p) in power_row.iter().enumerate() {
                    out_row[t] += w * p;
                }
            }
        }

        // Log normalization
        let log10_factor = 1.0 / 10.0f32.ln();
        let mut max_val = f32::NEG_INFINITY;
        for v in mel_spec.iter_mut() {
            *v = v.max(1e-10f32).ln() * log10_factor;
            if *v > max_val {
                max_val = *v;
            }
        }

        // Clamp and normalize: (max(log_mel, max_val-8) + 4) / 4
        let min_val = max_val - 8.0;
        for v in mel_spec.iter_mut() {
            *v = (v.max(min_val) + 4.0) / 4.0;
        }

        Ok((mel_spec, self.num_mel_bins, n_frames))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_hann_window_edges() {
        let w = hann_window(5);
        assert_eq!(w.len(), 5);
        assert!(w[0].abs() < 1e-6, "window[0] should be 0");
        assert!(w[4].abs() < 1e-6, "window[n-1] should be ~0");
        // Center: x = 2π*2/4 = π → cos(π) = -1 → 0.5*(1-(-1)) = 1.0
        assert!((w[2] - 1.0).abs() < 1e-6, "window[center] should be 1");
    }

    #[test]
    fn test_reflection_pad_basic() {
        let signal = vec![1.0f32, 2.0, 3.0];
        let padded = reflection_pad(&signal, 1);
        assert_eq!(padded, vec![2.0f32, 1.0, 2.0, 3.0, 2.0]);
    }

    #[test]
    fn test_mel_filterbank_shape_and_nonneg() {
        let num_mels = 128;
        let n_fft = 400;
        let sr = 16000u32;
        let n_freqs = n_fft / 2 + 1; // 201
        let filters = create_mel_filterbank(num_mels, n_fft, sr, 0.0, sr as f64 / 2.0);
        assert_eq!(filters.len(), num_mels * n_freqs);
        assert!(filters.iter().all(|&v| v >= 0.0), "all filterbank values must be >= 0");
    }

    #[test]
    fn test_mel_extractor_silent_signal() {
        // 1 second of silence at 16 kHz
        let samples = vec![0.0f32; 16000];
        let extractor = MelExtractor::new(400, 160, 128, 16000);
        let (mel, n_mels, n_frames) = extractor.extract(&samples).unwrap();
        assert_eq!(n_mels, 128);
        assert!(n_frames > 0, "n_frames should be positive");
        assert_eq!(mel.len(), n_mels * n_frames);
        assert!(mel.iter().all(|v| v.is_finite()), "all mel values must be finite");
    }
}
