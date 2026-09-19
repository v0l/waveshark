//! Welch-averaged power spectrum for display.

use crate::window;
use common::C32;
use rustfft::{Fft, FftPlanner};
use std::sync::Arc;

/// What a run of transforms came to, in dBFS, read several ways.
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
    /// A reading per percentile [`Spectrum::take`] was asked for.
    ///
    /// Only what was asked for, because taking one costs a selection over
    /// every bin and a frame nobody is drawing a percentile from should not
    /// pay for it.
    pub pct: Vec<(u8, Vec<f32>)>,
}

/// The percentile a detector takes when nothing else is chosen.
pub const DEFAULT_PERCENT: u8 = 80;

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
    /// The level the given percent of the frame's transforms fell below.
    ///
    /// Between the mean and the peak, and what either of them cannot do: a
    /// high percentile finds a signal present for part of the frame without
    /// the peak's habit of drawing the whole span up to one stray transform,
    /// and a low one reads a floor a nearby burst cannot lift.
    Percentile(u8),
    /// The loudest each bin reached. Finds a transmission shorter than the
    /// frame, at the price of a floor that reads high.
    Peak,
}

impl Detector {
    pub const ALL: [Detector; 4] = [
        Detector::Sample,
        Detector::Average,
        Detector::Percentile(DEFAULT_PERCENT),
        Detector::Peak,
    ];

    /// The menu entries, the percentile carrying `current`'s own number.
    ///
    /// A picker compares the value it holds against its options, so the
    /// percentile entry has to be the one that is set or nothing matches and
    /// the control shows the first option instead.
    pub fn options(current: Detector) -> [Detector; 4] {
        let mut all = Detector::ALL;
        all[2] = Detector::Percentile(current.percent().unwrap_or(DEFAULT_PERCENT));
        all
    }

    /// The percent this detector takes, where it takes one.
    pub fn percent(self) -> Option<u8> {
        match self {
            Detector::Percentile(p) => Some(p),
            _ => None,
        }
    }

    /// The same detector at another percent, which a plain reading ignores.
    pub fn at_percent(self, p: u8) -> Self {
        match self {
            Detector::Percentile(_) => Detector::Percentile(p.clamp(1, 99)),
            other => other,
        }
    }

    pub fn label(self) -> String {
        match self {
            Detector::Sample => "sample".into(),
            Detector::Average => "average".into(),
            Detector::Percentile(p) => format!("p{p}"),
            Detector::Peak => "peak".into(),
        }
    }

    pub fn parse(s: &str) -> Self {
        let s = s.trim().to_ascii_lowercase();
        // Parsed wide and clamped, so a number that is not a percent lands
        // at the nearest one it could have meant rather than silently
        // reading as the default detector.
        if let Some(n) = s.strip_prefix('p')
            && let Ok(p) = n.parse::<u32>()
        {
            return Detector::Percentile(p.clamp(1, 99) as u8);
        }
        match s.as_str() {
            "sample" => Detector::Sample,
            "peak" => Detector::Peak,
            "percentile" => Detector::Percentile(DEFAULT_PERCENT),
            _ => Detector::Average,
        }
    }

    /// The reading this detector takes out of a frame.
    ///
    /// A percentile the frame was not asked for falls back to the mean,
    /// which is what a frame taken by something that does not know about
    /// this detector holds.
    pub fn of(self, f: &Frame) -> &[f32] {
        match self {
            Detector::Sample => &f.sample,
            Detector::Average => &f.mean,
            Detector::Peak => &f.peak,
            Detector::Percentile(p) => {
                f.pct.iter().find(|(q, _)| *q == p).map_or(&f.mean[..], |(_, v)| &v[..])
            }
        }
    }
}

/// How many of a frame's transforms a percentile is selected over.
///
/// Every transform still reaches the peak and the mean; this is only how
/// finely the distribution behind them is kept. Sixty-four puts the answer
/// within a bin and a half of the asked-for rank and costs 4 MB at the
/// largest transform this offers.
const KEPT: usize = 64;

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
    /// A subsample of the frame's transforms, `KEPT` slots of `size` bins,
    /// which is what a percentile is selected over.
    keep: Vec<f32>,
    /// Slots filled, and how many transforms apart the ones kept are.
    slots: usize,
    step: u32,
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
            keep: vec![0.0; size * KEPT],
            slots: 0,
            step: 1,
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
        let slot = self.taken.is_multiple_of(self.step).then(|| self.slots * self.size);
        for i in 0..self.size {
            // Rotate so DC lands in the middle, matching how the span is drawn.
            let src_bin = (i + half) % self.size;
            let p = self.buf[src_bin].norm_sqr() * self.scale * self.scale;
            if p > self.peak[i] {
                self.peak[i] = p;
            }
            self.sum[i] += p;
            self.last[i] = p;
            if let Some(base) = slot {
                self.keep[base + i] = p;
            }
        }
        if slot.is_some() {
            self.slots += 1;
            if self.slots == KEPT {
                self.thin();
            }
        }
        self.taken += 1;
    }

    /// Throw away every other kept slot and take them half as often.
    ///
    /// A frame holds however many transforms the sample rate and the refresh
    /// rate leave it, which is hundreds at a wide span, and a percentile over
    /// the newest sixty-four of those would be a percentile of the last few
    /// milliseconds. Halving instead keeps the surviving slots spread over
    /// the whole frame, so what is selected is a fair sample of it.
    fn thin(&mut self) {
        for dst in 1..KEPT / 2 {
            let src = dst * 2;
            self.keep.copy_within(src * self.size..(src + 1) * self.size, dst * self.size);
        }
        self.slots = KEPT / 2;
        self.step *= 2;
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
    pub fn take(&mut self, want: &[Detector]) -> Frame {
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
        let mut pct: Vec<(u8, Vec<f32>)> = Vec::new();
        for p in want.iter().filter_map(|d| d.percent()) {
            if !pct.iter().any(|(q, _)| *q == p) {
                pct.push((p, self.percentile(p)));
            }
        }
        self.peak.fill(0.0);
        self.sum.fill(0.0);
        self.slots = 0;
        self.step = 1;
        self.taken = 0;
        Frame { peak, mean, sample, pct }
    }

    /// The level `p` percent of the kept transforms fell below, per bin, in
    /// dBFS.
    ///
    /// Nearest rank, selected in power and converted afterwards, for the
    /// reason the mean is: the ordering is the same either way but the value
    /// between two ranks is not.
    fn percentile(&mut self, p: u8) -> Vec<f32> {
        let n = self.slots;
        if n == 0 {
            return vec![-200.0; self.size];
        }
        let rank = ((f32::from(p.clamp(1, 99)) / 100.0 * n as f32).ceil() as usize).clamp(1, n) - 1;
        let mut col = vec![0.0f32; n];
        let mut out = vec![0.0f32; self.size];
        for i in 0..self.size {
            for (s, c) in col.iter_mut().enumerate() {
                *c = self.keep[s * self.size + i];
            }
            // Selection rather than a sort: the rank is all that is wanted
            // and this runs over every bin of every frame.
            let (_, at, _) = col.select_nth_unstable_by(rank, f32::total_cmp);
            out[i] = 10.0 * (*at + 1e-20).log10();
        }
        out
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
        self.slots = 0;
        self.step = 1;
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
        let frame = s.take(&[]);
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
        let db = s.take(&[]).peak;
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
            s.take(&[]);
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
        let frame = s.take(&[]);
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
        let db = s.take(&[]).peak;
        let idx = db.iter().enumerate().max_by(|a, b| a.1.partial_cmp(b.1).unwrap()).unwrap().0;
        assert_eq!(idx, 128, "DC ended up in bin {idx}, not the centre");
    }

    #[test]
    fn positive_frequencies_sit_above_the_centre() {
        let mut s = Spectrum::new(512);
        s.smoothing = 1.0;
        s.process(&tone(8192, 64.0, 512, 1.0));
        let db = s.take(&[]).peak;
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
            let f = s.take(&[]);
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

    /// A tone keyed for a fifth of the frame, which is exactly the case the
    /// mean and the peak each read wrong.
    ///
    /// 16384 samples at 1024 bins with 50% overlap is 31 transforms, of
    /// which the loud ones are the first 6. So p50 sits in the silence, p95
    /// is in the tone, and the peak reads the tone's own level.
    #[test]
    fn a_percentile_sits_between_the_floor_and_the_peak() {
        let mut s = Spectrum::new(1024);
        let mut sig = tone(3277, 200.0, 1024, 1.0);
        sig.extend(tone(13107, 200.0, 1024, 0.0));
        s.process(&sig);
        let f = s.take(&[Detector::Percentile(50), Detector::Percentile(95)]);
        let bin = 512 + 200;
        let p50 = Detector::Percentile(50).of(&f)[bin];
        let p95 = Detector::Percentile(95).of(&f)[bin];
        assert!((f.peak[bin] - 0.0).abs() < 1.0, "peak read {:.1} dBFS", f.peak[bin]);
        assert!(p95 > -3.0, "p95 read {p95:.1} dBFS, which is not the tone");
        assert!(p50 < -100.0, "p50 read {p50:.1} dBFS, which is not the silence");
        // The mean of a fifth at full scale is about -7 dB, which is neither.
        assert!((f.mean[bin] + 7.0).abs() < 2.0, "mean read {:.1} dBFS", f.mean[bin]);
    }

    /// A percentile is taken over a subsample when a frame holds more
    /// transforms than there are slots, and the subsample spans the frame
    /// rather than its tail.
    #[test]
    fn a_long_frame_is_subsampled_across_its_whole_length() {
        let mut s = Spectrum::new(256);
        // 512 transforms, four times what thinning keeps: loud for the first
        // half, silent for the second. A tail-biased keep would read silence
        // at every rank.
        s.process(&tone(65536, 32.0, 256, 1.0));
        s.process(&tone(65536, 32.0, 256, 0.0));
        let f = s.take(&[Detector::Percentile(25), Detector::Percentile(75)]);
        let bin = 128 + 32;
        let lo = Detector::Percentile(25).of(&f)[bin];
        let hi = Detector::Percentile(75).of(&f)[bin];
        assert!(lo < -100.0, "p25 read {lo:.1} dBFS, expected the silent half");
        assert!(hi > -3.0, "p75 read {hi:.1} dBFS, expected the loud half");
    }

    /// A percentile nobody asked for is not computed, and reads as the mean.
    #[test]
    fn an_unasked_percentile_falls_back_to_the_mean() {
        let mut s = Spectrum::new(256);
        s.process(&tone(4096, 32.0, 256, 1.0));
        let f = s.take(&[Detector::Percentile(80)]);
        assert_eq!(f.pct.len(), 1, "only the asked-for rank is computed");
        assert_eq!(Detector::Percentile(10).of(&f), &f.mean[..]);
        assert_ne!(Detector::Percentile(80).of(&f), &f.mean[..]);
    }

    #[test]
    fn a_detector_survives_being_written_out_and_read_back() {
        for d in [
            Detector::Sample,
            Detector::Average,
            Detector::Peak,
            Detector::Percentile(80),
            Detector::Percentile(1),
            Detector::Percentile(99),
        ] {
            assert_eq!(Detector::parse(&d.label()), d, "{}", d.label());
        }
        assert_eq!(Detector::parse("percentile"), Detector::Percentile(DEFAULT_PERCENT));
        assert_eq!(Detector::parse("p200"), Detector::Percentile(99), "clamped, not discarded");
        assert_eq!(Detector::parse("p0"), Detector::Percentile(1));
        assert_eq!(Detector::parse("px"), Detector::Average, "not a number at all");
        // The picker's options carry whatever is set, or nothing matches.
        assert_eq!(Detector::options(Detector::Percentile(35))[2], Detector::Percentile(35));
        assert_eq!(Detector::options(Detector::Peak)[2], Detector::Percentile(DEFAULT_PERCENT));
        assert_eq!(Detector::Peak.at_percent(35), Detector::Peak, "a peak takes no rank");
    }
}
