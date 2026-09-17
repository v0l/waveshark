//! Welch-averaged power spectrum for display.

use crate::window;
use common::C32;
use rustfft::{Fft, FftPlanner};
use std::sync::Arc;

/// What a run of transforms came to, in dBFS, read three ways.
///
/// The same split a spectrum analyser makes, where a trace point covers a
/// bucket of samples and a detector says what the point shows: the loudest
/// finds a burst, the mean measures a floor, and the newest is the band as
/// it is at this instant, jitter and all.
#[derive(Clone, Debug, PartialEq)]
pub struct Frame {
    pub peak: Vec<f32>,
    pub mean: Vec<f32>,
    pub sample: Vec<f32>,
}

/// Which of a frame's readings a display takes.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Detector {
    /// The newest transform: what the band is doing now, and what this
    /// drew before there was a choice.
    Sample,
    /// Mean power over the frame. A steady floor, and a burst pulled down
    /// by however little of the frame it occupied.
    #[default]
    Average,
    /// The loudest each bin reached. Finds a transmission shorter than the
    /// frame, at the price of a floor that reads high.
    Peak,
}

impl Detector {
    pub const ALL: [Detector; 3] = [Detector::Sample, Detector::Average, Detector::Peak];

    pub fn label(self) -> &'static str {
        match self {
            Detector::Sample => "sample",
            Detector::Average => "average",
            Detector::Peak => "peak",
        }
    }

    pub fn parse(s: &str) -> Self {
        match s.trim().to_ascii_lowercase().as_str() {
            "sample" => Detector::Sample,
            "peak" => Detector::Peak,
            _ => Detector::Average,
        }
    }

    /// The reading this detector takes out of a frame.
    pub fn of(self, f: &Frame) -> &[f32] {
        match self {
            Detector::Sample => &f.sample,
            Detector::Average => &f.mean,
            Detector::Peak => &f.peak,
        }
    }
}

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
    /// Loudest and total linear power per bin since the last frame was
    /// taken, and how many transforms went into them.
    ///
    /// Every transform lands here, so nothing that happened between two
    /// published frames is lost: a heatmap is read to find transmissions,
    /// and a burst shorter than the gap between frames is exactly what it
    /// is being read for.
    peak: Vec<f32>,
    sum: Vec<f32>,
    /// The most recent transform on its own, which is what the display's
    /// average advances from.
    ///
    /// A display wants the band as it is now: averaging a frame's worth of
    /// transforms into it flattens exactly the short bursts an operator is
    /// watching for, which is the opposite of what a heatmap wants from the
    /// same seconds.
    last: Vec<f32>,
    taken: u32,
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
            peak: vec![0.0; size],
            sum: vec![0.0; size],
            last: vec![0.0; size],
            taken: 0,
            out: vec![0.0; size],
            primed: false,
            pending: Vec::new(),
            smoothing: 0.35,
        }
    }

    pub fn size(&self) -> usize {
        self.size
    }

    /// Consume samples, transforming every full frame with 50% overlap.
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

        let half = self.size / 2;
        for i in 0..self.size {
            // Rotate so DC lands in the middle, matching how the span is drawn.
            let src_bin = (i + half) % self.size;
            let p = self.buf[src_bin].norm_sqr() * self.scale * self.scale;
            if p > self.peak[i] {
                self.peak[i] = p;
            }
            self.sum[i] += p;
            self.last[i] = p;
        }
        self.taken += 1;
    }

    /// Averaged spectrum in dBFS, lowest frequency first.
    pub fn power_db(&mut self) -> &[f32] {
        for (o, &p) in self.out.iter_mut().zip(&self.avg) {
            *o = 10.0 * (p + 1e-20).log10();
        }
        &self.out
    }

    /// Whether anything has been transformed since the last [`Self::take`].
    pub fn pending_frames(&self) -> u32 {
        self.taken
    }

    /// Everything transformed since the last call, as decibels, and start
    /// again: the loudest each bin reached, and its mean power.
    ///
    /// The peak is what finds transmissions and the mean is what measures a
    /// floor, and the two answer different questions of the same seconds:
    /// a remote keyed for 20 ms inside a half-second row is 14 dB down in
    /// the mean and at its own level in the peak. Taking also advances the
    /// What is drawn from it is the caller's choice of detector, handed
    /// back through [`Self::fold`], which is what the `smoothing` control
    /// acts on.
    pub fn take(&mut self) -> Frame {
        let n = self.taken.max(1) as f32;
        let mut peak = vec![0.0f32; self.size];
        let mut mean = vec![0.0f32; self.size];
        let mut sample = vec![0.0f32; self.size];
        for i in 0..self.size {
            let m = self.sum[i] / n;
            peak[i] = 10.0 * (self.peak[i] + 1e-20).log10();
            mean[i] = 10.0 * (m + 1e-20).log10();
            sample[i] = 10.0 * (self.last[i] + 1e-20).log10();
        }
        self.peak.fill(0.0);
        self.sum.fill(0.0);
        self.taken = 0;
        Frame { peak, mean, sample }
    }

    /// Advance the smoothed trace by one frame of whatever was chosen.
    ///
    /// Per frame drawn rather than per transform: the transform rate is the
    /// sample rate divided by the FFT size, so smoothing applied there
    /// meant a control that acted differently on every radio and every
    /// span. A frame is what somebody looking at the screen counts in.
    pub fn fold(&mut self, db: &[f32]) {
        let a = if self.primed { self.smoothing } else { 1.0 };
        self.primed = true;
        for (o, &d) in self.avg.iter_mut().zip(db) {
            // Averaged in power, never in decibels: a mean of logarithms is
            // dragged down by deep nulls that carry almost no power.
            let p = 10f32.powf(d / 10.0);
            *o += a * (p - *o);
        }
    }

    pub fn reset(&mut self) {
        self.avg.fill(0.0);
        self.peak.fill(0.0);
        self.sum.fill(0.0);
        self.last.fill(0.0);
        self.taken = 0;
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
/// is rounded up to a power of two for the transform, and `win_len` is how
/// many samples one column spans, which sets the time resolution.
pub fn spectrogram(samples: &[C32], cols: usize, rows: usize, win_len: usize) -> Vec<f32> {
    let n = rows.max(16).next_power_of_two();
    let cols = cols.max(1);
    // The window is how much of the burst one column sees, so it sets the
    // time resolution; the transform size `n` sets the frequency detail.
    // Keeping the window a fixed fraction of the burst rather than a fixed
    // number of samples is what makes the same signal look the same however
    // fast it was sampled: a wide window averages the spectrum over many
    // symbols and smears keying into a solid band. The window is zero-padded
    // up to `n`, which interpolates the spectrum smoothly.
    let w = win_len.clamp(4, n);
    let mut out = vec![-120.0f32; cols * n];
    if samples.len() < 4 {
        return out;
    }
    let fft = FftPlanner::new().plan_fft_forward(n);
    let hann = window::hann(w);
    let mut buf = vec![C32::default(); n];
    let mut scratch = vec![C32::default(); fft.get_inplace_scratch_len()];
    let half = n / 2;
    let mut peak = 1e-20f32;
    // One column per pixel, its windows centred on the column's place in
    // the burst so the first and last columns are the burst's ends, not
    // silence beyond them. A column of a long burst spans more samples than
    // one window holds, so it averages the windows across its span rather
    // than reading one and skipping the rest: a chirp then draws as the
    // same soft continuous trace a short burst does, instead of a hairline
    // through one sample in ten.
    let stride = samples.len() as f64 / cols as f64;
    let hops = ((stride / (w as f64 / 2.0)).ceil() as usize).clamp(1, 16);
    for c in 0..cols {
        let centre = (c as f64 + 0.5) / cols as f64 * samples.len() as f64;
        let col = &mut out[c..];
        for r in 0..n {
            col[r * cols] = 0.0;
        }
        for h in 0..hops {
            let at = centre + ((h as f64 + 0.5) / hops as f64 - 0.5) * stride;
            let start = at as isize - (w / 2) as isize;
            for i in 0..n {
                buf[i] = if i < w {
                    let j = start + i as isize;
                    let s = if j >= 0 { samples.get(j as usize).copied() } else { None };
                    s.unwrap_or(C32::new(0.0, 0.0)) * hann[i]
                } else {
                    C32::new(0.0, 0.0)
                };
            }
            fft.process_with_scratch(&mut buf, &mut scratch);
            for r in 0..n {
                // Shift so the lowest frequency is at row zero.
                let bin = (r + half) % n;
                out[r * cols + c] += buf[bin].norm_sqr() / hops as f32;
            }
        }
        for r in 0..n {
            peak = peak.max(out[r * cols + c]);
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
        let frame = s.take();
        let peak = frame.peak.iter().cloned().fold(f32::MIN, f32::max);
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
        let db = s.take().peak;
        let idx = db.iter().enumerate().max_by(|a, b| a.1.partial_cmp(b.1).unwrap()).unwrap().0;
        assert_eq!(idx, 2048 + 400);
    }

    /// What transforming every sample costs, which is what buys a heatmap
    /// that sees a 20 ms burst. Measured here so the number in the comment
    /// beside the spectrum node is one somebody took.
    #[test]
    #[ignore = "a measurement, not a check"]
    fn what_full_coverage_costs() {
        for size in [1024usize, 4096, 16384] {
            let mut s = Spectrum::new(size);
            let sig = tone(1 << 20, 100.0, size, 1.0);
            let t = std::time::Instant::now();
            s.process(&sig);
            s.take();
            let secs = t.elapsed().as_secs_f64();
            let rate = 2_400_000.0;
            println!(
                "{size:>6} bins: {:.1} Ms/s, {:.1}% of a core at 2.4 MS/s",
                sig.len() as f64 / secs / 1e6,
                rate * secs / sig.len() as f64 * 100.0
            );
        }
    }

    /// What the display draws is the band as it is now, not the mean of the
    /// frame: the same transforms feed the heatmap's peak, and averaging
    /// them into the trace would flatten the bursts an operator watches for.
    #[test]
    fn the_display_average_follows_the_newest_transform() {
        let mut s = Spectrum::new(256);
        s.smoothing = 1.0;
        // A frame's worth of silence, then a full-scale tone at the end.
        s.process(&tone(2048, 32.0, 256, 0.0));
        s.process(&tone(1024, 32.0, 256, 1.0));
        let frame = s.take();
        s.fold(&frame.sample);
        let drawn = s.power_db().iter().cloned().fold(f32::MIN, f32::max);
        let peak = frame.peak.iter().cloned().fold(f32::MIN, f32::max);
        let mean = frame.mean.iter().cloned().fold(f32::MIN, f32::max);
        assert!(drawn.abs() < 0.5, "the trace shows the tone that is there now: {drawn:.1}");
        assert!(peak.abs() < 0.5, "and so does the peak: {peak:.1}");
        assert!(mean < drawn - 3.0, "while the mean is pulled down by the silence: {mean:.1}");
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
        let db = s.take().peak;
        let idx = db.iter().enumerate().max_by(|a, b| a.1.partial_cmp(b.1).unwrap()).unwrap().0;
        assert_eq!(idx, 128, "DC ended up in bin {idx}, not the centre");
    }

    #[test]
    fn positive_frequencies_sit_above_the_centre() {
        let mut s = Spectrum::new(512);
        s.smoothing = 1.0;
        s.process(&tone(8192, 64.0, 512, 1.0));
        let db = s.take().peak;
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
            // The display average advances a frame at a time, which is when
            // the readings are taken rather than when a transform runs.
            let f = s.take();
            s.fold(&f.sample);
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
        let img = spectrogram(&sig, cols, rows, rows);
        // Row of -100 kHz is below centre, +100 kHz above; find the loudest
        // row in the first and last columns.
        let loudest = |col: usize| {
            (0..rows)
                .max_by(|&a, &b| img[a * cols + col].partial_cmp(&img[b * cols + col]).unwrap())
                .unwrap()
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
            .filter(|(i, _)| (*i as i32 - peak_bin).abs() > 10)
            .map(|(_, v)| *v)
            .fold(f32::MIN, f32::max);
        // Blackman-Harris reaches roughly -92 dB sidelobes.
        assert!(floor < -80.0, "sidelobes only reached {floor:.1} dBFS");
    }
}
