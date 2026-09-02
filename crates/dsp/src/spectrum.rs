//! Welch-averaged power spectrum for display.

use crate::window;
use common::C32;
use rustfft::{Fft, FftPlanner};
use std::sync::Arc;

/// Windowed, overlapped, averaged FFT producing dBFS bins in display order
/// (negative frequencies first, DC in the middle).
pub struct Spectrum {
    fft: Arc<dyn Fft<f32>>,
    size: usize,
    win: Vec<f32>,
    /// Normalisation for window loss and FFT size, so a full-scale tone reads 0 dBFS.
    scale: f32,
    scratch: Vec<C32>,
    buf: Vec<C32>,
    /// Exponential average of linear power, not of dB. Averaging logarithms
    /// biases the result low, because occasional deep nulls dominate a mean
    /// taken in dB but are negligible in power.
    avg: Vec<f32>,
    out: Vec<f32>,
    primed: bool,
    /// Samples carried between calls, so a frame can span input blocks.
    pending: Vec<C32>,
    pub smoothing: f32,
}

impl Spectrum {
    pub fn new(size: usize) -> Self {
        assert!(size.is_power_of_two() && size >= 16, "fft size must be a power of two >= 16");
        let fft = FftPlanner::new().plan_fft_forward(size);
        let win = window::blackman_harris(size);
        let cg = window::coherent_gain(&win);
        Self {
            scratch: vec![C32::default(); fft.get_inplace_scratch_len()],
            fft,
            size,
            scale: 1.0 / (cg * size as f32),
            win,
            buf: vec![C32::default(); size],
            avg: vec![0.0; size],
            out: vec![0.0; size],
            primed: false,
            pending: Vec::new(),
            smoothing: 0.35,
        }
    }

    pub fn size(&self) -> usize {
        self.size
    }

    /// Consume samples, averaging every full frame with 50% overlap.
    ///
    /// Leftovers are carried to the next call, so the FFT may be larger than
    /// the blocks the radio delivers. Without that, asking for 16384 bins
    /// while the driver hands over 8192 samples produces no frames at all and
    /// the display simply stops.
    pub fn process(&mut self, iq: &[C32]) -> bool {
        let hop = self.size / 2;
        self.pending.extend_from_slice(iq);
        let mut any = false;
        let mut pos = 0;
        while pos + self.size <= self.pending.len() {
            self.frame_at(pos);
            pos += hop;
            any = true;
        }
        self.pending.drain(..pos);
        any
    }

    /// Windows `size` samples starting at `pos` in `pending`. Taking an index
    /// rather than a slice keeps the borrow checker happy without unsafe.
    fn frame_at(&mut self, pos: usize) {
        for i in 0..self.size {
            self.buf[i] = self.pending[pos + i] * self.win[i];
        }
        self.fft.process_with_scratch(&mut self.buf, &mut self.scratch);

        let a = if self.primed { self.smoothing } else { 1.0 };
        let half = self.size / 2;
        for i in 0..self.size {
            // Rotate so DC lands in the middle, matching how the span is drawn.
            let src_bin = (i + half) % self.size;
            let p = self.buf[src_bin].norm_sqr() * self.scale * self.scale;
            self.avg[i] += a * (p - self.avg[i]);
        }
        self.primed = true;
    }

    /// Averaged spectrum in dBFS, lowest frequency first.
    pub fn power_db(&mut self) -> &[f32] {
        for (o, &p) in self.out.iter_mut().zip(&self.avg) {
            *o = 10.0 * (p + 1e-20).log10();
        }
        &self.out
    }

    pub fn reset(&mut self) {
        self.avg.fill(0.0);
        self.primed = false;
        self.pending.clear();
    }
}

/// A spectrogram of one burst, for a view that shows what it is: `cols`
/// columns across its length and `rows` frequency bins from the lowest
/// frequency at index zero to the highest, each cell the power in decibels
/// below the burst's peak.
///
/// This is the view a burst is read from, the way inspectrum and Universal
/// Radio Hacker show one: a two-tone signal is two lines, a chirp a ramp,
/// on-off keying a broken bar, a multi-carrier signal a filled band. `rows`
/// is rounded up to a power of two for the transform.
pub fn spectrogram(samples: &[C32], cols: usize, rows: usize) -> Vec<f32> {
    let n = rows.max(16).next_power_of_two();
    let cols = cols.max(1);
    let mut out = vec![-120.0f32; cols * n];
    if samples.len() < 4 {
        return out;
    }
    let fft = FftPlanner::new().plan_fft_forward(n);
    let win = window::hann(n);
    let mut buf = vec![C32::default(); n];
    let mut scratch = vec![C32::default(); fft.get_inplace_scratch_len()];
    let half = n / 2;
    let mut peak = 1e-20f32;
    // One column per pixel, its window centred on the column's place in the
    // burst so the first and last columns are the burst's ends, not silence
    // beyond them.
    for c in 0..cols {
        let centre = (c as f64 + 0.5) / cols as f64 * samples.len() as f64;
        let start = (centre as isize - half as isize).max(0) as usize;
        for i in 0..n {
            let s = samples.get(start + i).copied().unwrap_or(C32::new(0.0, 0.0));
            buf[i] = s * win[i];
        }
        fft.process_with_scratch(&mut buf, &mut scratch);
        for r in 0..n {
            // Shift so the lowest frequency is at row zero.
            let bin = (r + half) % n;
            let power = buf[bin].norm_sqr();
            peak = peak.max(power);
            out[r * cols + c] = power;
        }
    }
    let scale = 1.0 / peak;
    for v in &mut out {
        *v = 10.0 * (*v * scale + 1e-12).log10();
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    fn tone(n: usize, cycles_per_frame: f64, size: usize, amp: f32) -> Vec<C32> {
        (0..n)
            .map(|i| {
                let ph = std::f64::consts::TAU * cycles_per_frame * i as f64 / size as f64;
                C32::new(amp * ph.cos() as f32, amp * ph.sin() as f32)
            })
            .collect()
    }

    #[test]
    fn a_full_scale_tone_reads_near_zero_dbfs() {
        let mut s = Spectrum::new(1024);
        s.smoothing = 1.0;
        assert!(s.process(&tone(8192, 100.0, 1024, 1.0)));
        let peak = s.power_db().iter().cloned().fold(f32::MIN, f32::max);
        assert!(peak.abs() < 0.5, "full scale tone read {peak:.2} dBFS");
    }

    #[test]
    fn a_frame_can_span_several_input_blocks() {
        // The driver's block size and the FFT size are unrelated, so a large
        // FFT must accumulate rather than silently produce nothing.
        let mut s = Spectrum::new(4096);
        s.smoothing = 1.0;
        let sig = tone(8192, 400.0, 4096, 1.0);
        let mut produced = false;
        for chunk in sig.chunks(512) {
            produced |= s.process(chunk);
        }
        assert!(produced, "no frame from blocks smaller than the FFT");
        let db = s.power_db();
        let idx = db.iter().enumerate().max_by(|a, b| a.1.partial_cmp(b.1).unwrap()).unwrap().0;
        assert_eq!(idx, 2048 + 400);
    }

    #[test]
    fn leftovers_do_not_accumulate_without_bound() {
        let mut s = Spectrum::new(1024);
        for _ in 0..500 {
            s.process(&tone(300, 10.0, 1024, 1.0));
        }
        assert!(s.pending.len() < 1024 + 300, "carried {} samples", s.pending.len());
    }

    #[test]
    fn dc_lands_in_the_middle() {
        let mut s = Spectrum::new(256);
        s.smoothing = 1.0;
        let dc = vec![C32::new(1.0, 0.0); 2048];
        s.process(&dc);
        let db = s.power_db();
        let idx = db.iter().enumerate().max_by(|a, b| a.1.partial_cmp(b.1).unwrap()).unwrap().0;
        assert_eq!(idx, 128, "DC ended up in bin {idx}, not the centre");
    }

    #[test]
    fn positive_frequencies_sit_above_the_centre() {
        let mut s = Spectrum::new(512);
        s.smoothing = 1.0;
        s.process(&tone(8192, 64.0, 512, 1.0));
        let db = s.power_db();
        let idx = db.iter().enumerate().max_by(|a, b| a.1.partial_cmp(b.1).unwrap()).unwrap().0;
        assert_eq!(idx, 256 + 64, "positive tone landed in bin {idx}");
    }

    #[test]
    fn averaging_in_power_does_not_bias_low() {
        // Half the time full scale, half silent, in runs long enough that few
        // frames straddle a transition. Averaging power gives about -3 dB;
        // averaging decibels would be dragged toward the silent frames.
        // The averaging window must be much longer than one loud/silent run,
        // or the reading is just wherever the oscillation happened to stop.
        let mut s = Spectrum::new(256);
        s.smoothing = 0.005;
        for i in 0..400 {
            let amp = if i % 2 == 0 { 1.0 } else { 0.0 };
            s.process(&tone(2048, 32.0, 256, amp));
        }
        let peak = s.power_db().iter().cloned().fold(f32::MIN, f32::max);
        assert!((peak + 3.0).abs() < 2.0, "expected about -3 dBFS, got {peak:.2}");
    }

    #[test]
    fn a_spectrogram_of_two_tones_lights_two_rows() {
        // A signal that spends its first half at one tone and its second at
        // another, as two-level keying does. The two show as two rows, well
        // apart, and each only where its half of the burst is.
        let rate = 1_000_000.0;
        let n = 20_000usize;
        let sig: Vec<C32> = (0..n)
            .map(|i| {
                let hz = if i < n / 2 { -100_000.0 } else { 100_000.0 };
                let ph = std::f64::consts::TAU * hz * i as f64 / rate;
                C32::new(ph.cos() as f32, ph.sin() as f32)
            })
            .collect();
        let (cols, rows) = (64usize, 128usize);
        let img = spectrogram(&sig, cols, rows);
        // Row of -100 kHz is below centre, +100 kHz above; find the loudest
        // row in the first and last columns.
        let loudest = |col: usize| {
            (0..rows).max_by(|&a, &b| img[a * cols + col].partial_cmp(&img[b * cols + col]).unwrap()).unwrap()
        };
        let lo = loudest(4);
        let hi = loudest(cols - 5);
        assert!(lo < rows / 2, "first tone should sit below centre, row {lo}");
        assert!(hi > rows / 2, "second tone should sit above centre, row {hi}");
        assert!(hi.abs_diff(lo) > rows / 8, "the tones should be well apart: {lo} vs {hi}");
    }

    #[test]
    fn the_noise_floor_is_far_below_a_tone() {
        let mut s = Spectrum::new(1024);
        s.smoothing = 1.0;
        s.process(&tone(16384, 200.0, 1024, 1.0));
        let db = s.power_db().to_vec();
        let peak_bin = 512 + 200;
        let floor: f32 = db
            .iter()
            .enumerate()
            .filter(|(i, _)| (*i as i32 - peak_bin as i32).abs() > 10)
            .map(|(_, v)| *v)
            .fold(f32::MIN, f32::max);
        // Blackman-Harris reaches roughly -92 dB sidelobes.
        assert!(floor < -80.0, "sidelobes only reached {floor:.1} dBFS");
    }
}
