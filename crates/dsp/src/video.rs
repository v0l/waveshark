//! Analogue television baseband: sync separation and a picture out of it.
//!
//! Analogue video is not a protocol. A camera produces composite video, a
//! transmitter frequency modulates a carrier with it, and that is the whole
//! stack: no framing, no addressing, no integrity check anywhere. What
//! carries structure is the video itself, and it is the structure television
//! had in 1960: a horizontal sync pulse below black at the start of every
//! line, a pattern of broad pulses between fields, and two interlaced fields
//! to a frame. A model aircraft's video link, a security camera and a bench
//! pattern generator all produce the same thing; what differs is the band it
//! is sent on.
//!
//! So this file takes the output of an FM demodulator and finds those pulses.
//! Levels are relative, because the demodulator's output scale depends on the
//! deviation it was told and a transmitter's deviation is whatever its
//! designer chose: sync tip and blanking are measured from the signal itself
//! rather than assumed, the way `crate::ble`'s gate tracks its own floor.
//!
//! # Colour
//!
//! Optional, and off unless asked for. PAL puts chrominance on a 4.43 MHz
//! subcarrier in quadrature, U on one axis and V on the other, and flips the
//! sign of V every line: that is what the P and the A stand for, and it is
//! what makes a phase error tint alternate lines in opposite directions so
//! the eye averages it away. The receiver does the averaging properly, with
//! a delay line: the U and V of a line are meaned with the line above.
//!
//! The reference is the burst on the back porch, ten cycles at 135 or 225
//! degrees. One burst alone cannot say which, since a phase error and a line
//! flip look the same; the mean of two consecutive bursts is the -U axis,
//! which is why the phase is estimated over a pair.
//!
//! Not here: audio. Most transmitters put it on a 6.0 or 6.5 MHz subcarrier,
//! which is another demodulator on this same baseband. The line spectrum of
//! the capture in `testdata` has it plainly at 6.5 MHz.

/// Which set of timings the camera is using.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Standard {
    /// 625 lines, 25 frames, 15.625 kHz line rate. What almost every camera
    /// sold outside North America produces.
    Pal,
    /// 525 lines, 29.97 frames, 15.734 kHz line rate.
    Ntsc,
}

impl Standard {
    pub fn line_hz(self) -> f64 {
        match self {
            Self::Pal => 15_625.0,
            Self::Ntsc => 15_734.264,
        }
    }

    /// Lines in a whole frame, both fields, including the ones with no
    /// picture in them.
    pub fn lines(self) -> usize {
        match self {
            Self::Pal => 625,
            Self::Ntsc => 525,
        }
    }

    /// The shape of the picture, which is not the shape of the sample grid.
    ///
    /// Both standards are 4:3. How many samples a line is cut into is a
    /// property of the receiver's clock and says nothing about what the
    /// camera saw: at 20 MS/s a 640 by 288 field drawn from its own numbers
    /// comes out at 10:9, which is a picture squeezed in from the sides.
    pub fn aspect(self) -> f32 {
        4.0 / 3.0
    }

    /// Picture lines in one field.
    pub fn active_lines(self) -> usize {
        match self {
            Self::Pal => 288,
            Self::Ntsc => 240,
        }
    }

    /// Seconds of horizontal sync at the start of a line.
    pub fn sync_s(self) -> f64 {
        match self {
            Self::Pal => 4.7e-6,
            Self::Ntsc => 4.7e-6,
        }
    }

    /// Seconds from the sync edge to the start of the picture.
    pub fn back_porch_s(self) -> f64 {
        match self {
            Self::Pal => 5.7e-6,
            Self::Ntsc => 4.5e-6,
        }
    }

    /// Seconds of picture on a line.
    pub fn active_s(self) -> f64 {
        match self {
            Self::Pal => 52.0e-6,
            Self::Ntsc => 52.6e-6,
        }
    }

    pub fn line_s(self) -> f64 {
        1.0 / self.line_hz()
    }

    /// The colour subcarrier.
    pub fn subcarrier_hz(self) -> f64 {
        match self {
            Self::Pal => 4_433_618.75,
            Self::Ntsc => 3_579_545.45,
        }
    }

    /// The standard a measured line period names, or `None` when it is
    /// neither. The two are 0.7% apart, which is far wider than a
    /// transmitter's timebase error, so measuring settles it.
    pub fn from_line_period(period_s: f64) -> Option<Self> {
        [Self::Pal, Self::Ntsc]
            .into_iter()
            .find(|s| (period_s / s.line_s() - 1.0).abs() < 0.003)
    }
}

/// Whether a demodulated baseband is analogue video, and which standard.
///
/// The positive test, rather than "try to decode it and see". A camera keys a
/// sync pulse of about 4.7 us at the start of every line, so the gaps between
/// those pulses cluster hard at the line period: 64 us for PAL, 63.55 for
/// NTSC, the two 0.7% apart, which no transmitter's timebase error reaches.
/// Noise and every other modulation here give gaps scattered across the
/// range instead.
///
/// So the measurement is the median gap, and the evidence is how many of the
/// gaps agree with it. `agreement` is that fraction, and it separates the two
/// cases a median alone cannot: a real camera puts nearly every gap within a
/// percent of the median, while a burst that happens to have two pulses the
/// right distance apart puts almost none there.
///
/// The samples must be filtered first, exactly as [`SyncSeparator`] filters:
/// a discriminator on a 20 MHz span carries all of that bandwidth's noise and
/// a single sample above the threshold ends a run that has to last 4.7 us.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Lock {
    pub standard: Standard,
    /// Sync pulses found.
    pub pulses: usize,
    /// Fraction of the gaps between them within one percent of the median.
    pub agreement: f32,
}

/// Look for a line rate in demodulated baseband.
///
/// `None` when there is no pulse train at either standard's rate, which is
/// what everything that is not video looks like.
pub fn find_lines(baseband: &[f32], rate: f64) -> Option<Lock> {
    // Two fields is enough to see six hundred lines, and more is only more
    // cost on a source that is not video.
    let take = ((0.04 * rate) as usize).min(baseband.len());
    let base = &baseband[..take];
    let taps = ((0.25e-6 * rate) as usize).max(1);
    if base.len() < taps * 4 {
        return None;
    }

    let mut sorted: Vec<f32> = base.to_vec();
    sorted.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    let (tip, black) = (sorted[sorted.len() / 50], sorted[sorted.len() * 13 / 100]);
    // A signal with no sync in it has no gap between those percentiles worth
    // slicing between, and slicing anyway finds runs in the noise.
    if black <= tip {
        return None;
    }
    let thresh = (tip + black) / 2.0;

    let (mut edges, mut low) = (Vec::new(), 0usize);
    let (lo, hi) = ((2e-6 * rate) as usize, (8e-6 * rate) as usize);
    // A running mean over the same quarter microsecond the separator uses,
    // carried rather than resummed: this runs over every sample of a wide
    // source.
    let mut sum: f32 = base[..taps].iter().sum();
    for i in 0..base.len() - taps {
        let mean = sum / taps as f32;
        sum += base[i + taps] - base[i];
        if mean < thresh {
            low += 1;
        } else {
            if (lo..=hi).contains(&low) {
                edges.push(i);
            }
            low = 0;
        }
    }
    // Six hundred lines are on offer in two fields. A hundred is a signal
    // that has been keying steadily for six milliseconds and is still a long
    // way short of a camera.
    if edges.len() < 100 {
        return None;
    }

    let mut gaps: Vec<f64> = edges
        .windows(2)
        .map(|w| (w[1] - w[0]) as f64 / rate)
        .collect();
    gaps.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    let median = gaps[gaps.len() / 2];
    let standard = Standard::from_line_period(median)?;
    let agree = gaps
        .iter()
        .filter(|g| (*g / median - 1.0).abs() < 0.01)
        .count();
    let agreement = agree as f32 / gaps.len() as f32;
    // Measured, not guessed. The off-air capture in `testdata` scores 0.67,
    // a synthesised camera 0.99, and the three wide captures that are not
    // cameras (WiFi, BLE, impulsive noise) never get this far: they fail on
    // the pulse count or on the median landing at no line period at all. Half
    // is between the two with room for a worse signal than the one recorded.
    (agreement > 0.5).then_some(Lock {
        standard,
        pulses: edges.len(),
        agreement,
    })
}

/// One field, as luma samples.
#[derive(Clone, Debug)]
pub struct Field {
    pub width: usize,
    pub height: usize,
    /// How wide the picture is against its height when it is drawn, which
    /// the sample grid does not say.
    pub aspect: f32,
    /// Row major, 0 for sync-black and 255 for peak white.
    pub luma: Vec<u8>,
    /// Row major RGB triples, when colour was asked for and the burst was
    /// found. `None` means the field was read as luma only, which is what a
    /// monochrome camera and a lost burst both look like.
    pub rgb: Option<Vec<u8>>,
    /// Lines whose sync was found, out of `height`. A field assembled from
    /// half its lines is a picture of a fade, and a caller deciding whether
    /// to show it needs the number.
    pub lines_seen: usize,
}

/// Sync separator: level tracking, horizontal sync detection, and fields
/// assembled from the lines between vertical pulses.
pub struct SyncSeparator {
    rate: f64,
    standard: Standard,
    width: usize,
    /// Sync tip and blanking level, tracked from the signal.
    sync_level: f32,
    black_level: f32,
    /// Samples since the last horizontal sync edge.
    since_sync: usize,
    /// The current field's lines.
    lines: Vec<Vec<u8>>,
    lines_seen: usize,
    /// Whether the level tracker has seen enough to be trusted.
    primed: bool,
    hist: Vec<f32>,
    /// Samples of a run below the sync threshold, for telling a horizontal
    /// pulse from a vertical one.
    low_run: usize,
    /// Running mean over `smooth_len` samples, and the window behind it.
    ///
    /// A discriminator reading a 20 MHz span gives a video baseband with all
    /// of that bandwidth's noise on it, and the picture occupies 5 MHz of it.
    /// Slicing that unfiltered does not find a sync pulse at all: a single
    /// noisy sample above the threshold ends the run, and the run has to last
    /// 4.7 us. Averaging over a quarter of a microsecond keeps the pulse,
    /// which is nearly twenty times longer, and throws most of the noise
    /// away. Off air this was the difference between no fields and every
    /// field.
    /// Ring of the last `smooth_len` samples, and where the next goes. A
    /// plain array rather than a deque: this runs on every sample of a 20 MS/s
    /// span, and the deque's bookkeeping was most of what the separator cost.
    smooth: Vec<f32>,
    smooth_at: usize,
    smooth_filled: usize,
    smooth_sum: f32,
    smooth_len: usize,
    /// Where the last line started, so the picture can be cut out of it.
    line: Vec<f32>,
    /// The same line unfiltered. The running mean that makes sync findable
    /// has its cutoff below the colour subcarrier, so chroma has to be taken
    /// from the samples as they arrived.
    raw_line: Vec<f32>,
    /// Whether to demodulate chroma at all.
    colour: bool,
    /// Chroma rows of the field being assembled, as (U, V) per pixel.
    chroma: Vec<Vec<(f32, f32)>>,
    /// The reference axis the last line's burst gave, which the next line's
    /// is resolved against.
    ///
    /// A burst is 135 or 225 degrees and one on its own cannot say which, so
    /// the choice is made by continuity: the axis a receiver locks to does
    /// not move between lines, and the polarity that keeps it still is the
    /// right one. This is the delay line's other job.
    last_axis: Option<f32>,
    /// Samples since the separator was built.
    ///
    /// The subcarrier phase is counted from here rather than from the start
    /// of each line, because PAL's subcarrier is continuous across lines and
    /// sync edges land on whole samples: at 20 MS/s one sample of jitter is
    /// 80 degrees of subcarrier, so a per-line origin rotates the colour by
    /// most of a turn from line to line.
    sample: u64,
    /// Where in that count the line being collected started.
    line_start: u64,
    stats: Stats,
}

/// What the separator saw, for telling a signal it cannot lock to from one
/// that is not there.
#[derive(Clone, Copy, Debug, Default)]
pub struct Stats {
    pub line_syncs: u64,
    pub broad_pulses: u64,
    pub lines_kept: u64,
    pub fields: u64,
    /// Fields thrown away for holding too few lines.
    pub fields_short: u64,
    pub sync_level: f32,
    pub black_level: f32,
}

impl SyncSeparator {
    /// `width` is the picture width to resample each line to. 720 is what a
    /// PAL line holds at broadcast sampling; a small camera is usually 600 or
    /// 800, and the sampling here is of the demodulated baseband rather than
    /// of the camera's own pixels, so the number is a choice and not a
    /// measurement.
    pub fn new(rate: f64, standard: Standard, width: usize) -> Self {
        Self {
            rate,
            standard,
            width,
            sync_level: 0.0,
            black_level: 0.0,
            since_sync: 0,
            lines: Vec::new(),
            lines_seen: 0,
            primed: false,
            hist: Vec::new(),
            low_run: 0,
            smooth: Vec::new(),
            smooth_at: 0,
            smooth_filled: 0,
            smooth_sum: 0.0,
            smooth_len: ((0.25e-6 * rate).round() as usize).max(1),
            line: Vec::new(),
            raw_line: Vec::new(),
            colour: false,
            chroma: Vec::new(),
            last_axis: None,
            sample: 0,
            line_start: 0,
            stats: Stats::default(),
        }
    }

    pub fn stats(&self) -> Stats {
        Stats {
            sync_level: self.sync_level,
            black_level: self.black_level,
            ..self.stats
        }
    }

    /// Demodulate colour as well as luma. Costs a quadrature demodulation and
    /// two box filters per line.
    pub fn with_colour(mut self) -> Self {
        self.colour = true;
        self
    }

    pub fn standard(&self) -> Standard {
        self.standard
    }

    fn samples(&self, seconds: f64) -> usize {
        (seconds * self.rate).round() as usize
    }

    /// Sync tip and blanking, from the distribution of the signal itself.
    ///
    /// A line is 7.3% sync and another 11% porches, and nothing in a picture
    /// goes below blanking, so the sorted samples have the tip in the bottom
    /// twentieth and blanking around the eighth. Taking blanking any higher
    /// reads picture as black level, which stretches the white point and
    /// washes the picture out: at the 30th percentile a full-scale ramp came
    /// back peaking at 61%.
    fn prime(&mut self) {
        let mut v = self.hist.clone();
        v.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        if v.len() < 100 {
            return;
        }
        self.sync_level = v[v.len() / 50];
        self.black_level = v[v.len() * 13 / 100];
        self.primed = self.black_level > self.sync_level;
    }

    fn threshold(&self) -> f32 {
        // Halfway between the tip and blanking, which is where a sync
        // separator has always sliced.
        (self.sync_level + self.black_level) / 2.0
    }

    /// One sample of the running mean.
    fn smoothed(&mut self, x: f32) -> f32 {
        if self.smooth.len() < self.smooth_len {
            self.smooth = vec![0.0; self.smooth_len];
            self.smooth_at = 0;
            self.smooth_filled = 0;
            self.smooth_sum = 0.0;
        }
        self.smooth_sum += x - self.smooth[self.smooth_at];
        self.smooth[self.smooth_at] = x;
        self.smooth_at = (self.smooth_at + 1) % self.smooth_len;
        self.smooth_filled = (self.smooth_filled + 1).min(self.smooth_len);
        self.smooth_sum / self.smooth_filled as f32
    }

    /// Feed demodulated baseband. Whole fields come back as they complete.
    pub fn process(&mut self, baseband: &[f32], out: &mut Vec<Field>) {
        let mut filtered: Vec<f32> = Vec::with_capacity(baseband.len());
        for &x in baseband {
            let v = self.smoothed(x);
            filtered.push(v);
        }
        let raw = baseband;
        let baseband = &filtered[..];
        if !self.primed {
            self.hist.extend_from_slice(baseband);
            // A field and a half, so the sample includes sync, blanking and
            // picture whatever the phase.
            if self.hist.len() as f64 > 0.03 * self.rate {
                self.prime();
                self.hist.clear();
            }
            if !self.primed {
                return;
            }
        }

        let thresh = self.threshold();
        // A horizontal pulse is 4.7 us and a vertical broad pulse 27.3; the
        // line either side of that is where they are told apart.
        let h_min = self.samples(self.standard.sync_s() * 0.6);
        let broad = self.samples(self.standard.sync_s() * 3.0);
        for (i, &x) in baseband.iter().enumerate() {
            self.line.push(x);
            if self.colour {
                self.raw_line.push(raw[i]);
            }
            self.since_sync += 1;
            self.sample += 1;
            if x < thresh {
                self.low_run += 1;
                continue;
            }
            let run = std::mem::take(&mut self.low_run);
            if run < h_min {
                continue;
            }
            if run >= broad {
                self.stats.broad_pulses += 1;
                // A broad pulse: the field is over. What is in hand is the
                // field, whether or not every line arrived.
                self.finish_field(out);
                self.line_start = self.sample;
                self.since_sync = 0;
                self.line.clear();
                self.raw_line.clear();
                continue;
            }
            // An ordinary line sync. The line just ended is cut and kept.
            self.stats.line_syncs += 1;
            let start = self.line_start;
            self.line_start = self.sample;
            self.take_line(run, start);
            self.since_sync = 0;
        }
    }

    /// Cut the picture out of the line that just ended and store it.
    ///
    /// The buffer runs from the end of the previous line's sync pulse, so it
    /// holds the back porch, the picture, the front porch and the sync that
    /// ended it. `sync_run` is that trailing sync, which is where the
    /// picture's far end is measured back from.
    fn take_line(&mut self, sync_run: usize, line_start: u64) {
        let line = std::mem::take(&mut self.line);
        self.line = Vec::with_capacity(line.len());
        // Taken here rather than beside the chroma below, because every
        // return in between would otherwise leave it holding this line as
        // well as the next and put the subcarrier phase a line out.
        let raw = std::mem::take(&mut self.raw_line);
        self.raw_line = Vec::with_capacity(raw.len());
        if self.lines.len() >= self.standard.active_lines() {
            return;
        }
        let start = self.samples(self.standard.back_porch_s());
        let want = self.samples(self.standard.active_s());
        if line.len() < start + sync_run + want / 2 {
            return;
        }
        let span = (line.len() - start - sync_run).min(want);
        let scale = self.black_level - self.sync_level;
        // White is about 0.7 V above blanking where sync is 0.3 V below it,
        // so the picture range is a bit over twice the sync depth. Taking it
        // from the levels rather than from a constant keeps the picture right
        // when a transmitter's deviation differs.
        let white = self.black_level + scale * 7.0 / 3.0;
        let mut row = vec![0u8; self.width];
        for (i, p) in row.iter_mut().enumerate() {
            let at = start + i * span / self.width.max(1);
            let v = line.get(at).copied().unwrap_or(self.black_level);
            let norm = (v - self.black_level) / (white - self.black_level).max(1e-6);
            *p = (norm.clamp(0.0, 1.0) * 255.0) as u8;
        }
        self.lines.push(row);
        self.lines_seen += 1;
        self.stats.lines_kept += 1;

        if self.colour {
            let uv = self.chroma_of(&raw, line_start, start, span, scale);
            self.chroma.push(uv);
        }
    }

    /// Demodulate one line's chroma against its own burst.
    ///
    /// Returns (U, V) per output pixel, already through the delay line: the
    /// mean with the line above, which is what cancels a phase error in PAL
    /// rather than leaving it as alternating tint.
    fn chroma_of(
        &mut self,
        raw: &[f32],
        line_start: u64,
        start: usize,
        span: usize,
        scale: f32,
    ) -> Vec<(f32, f32)> {
        let w = std::f64::consts::TAU * self.standard.subcarrier_hz() / self.rate;
        // Phase of the free-running subcarrier at an offset into this line.
        // Straight into f64 rather than through a modulo: an f64 holds the
        // product exactly for hours of samples, and folding the index instead
        // would step the phase every time it wrapped, since the subcarrier is
        // not a whole number of samples.
        let phase_at = |k: usize| (w * (line_start + k as u64) as f64).rem_euclid(std::f64::consts::TAU) as f32;
        // The burst sits on the back porch, about 0.9 us after the sync ends
        // and ten cycles long.
        let b0 = self.samples(0.8e-6);
        let b1 = self.samples(3.2e-6).min(raw.len());
        if b1 <= b0 + 8 {
            return Vec::new();
        }
        let mean = raw[b0..b1].iter().sum::<f32>() / (b1 - b0) as f32;
        let (mut bi, mut bq) = (0.0f32, 0.0f32);
        for (n, &x) in raw[b0..b1].iter().enumerate() {
            let p = phase_at(b0 + n);
            bi += (x - mean) * p.cos();
            bq += (x - mean) * p.sin();
        }
        let n = (b1 - b0) as f32;
        let (bi, bq) = (2.0 * bi / n, 2.0 * bq / n);
        let amp = (bi * bi + bq * bq).sqrt();
        // A burst is half the sync depth. Much less than that is a burst that
        // was not there, and demodulating noise against it invents colour.
        if amp < 0.15 * scale {
            self.last_axis = None;
            return Vec::new();
        }
        let phase = bq.atan2(bi);

        // Both polarities of the line give a candidate axis, 90 degrees
        // apart. The one that keeps the axis where the last line put it is
        // the right one; with no previous line there is nothing to be
        // continuous with, so the line is left grey rather than guessed.
        // A burst is the vector -U plus or minus V, which is 135 degrees from
        // the U axis on one side or the other.
        let eighth = 0.75 * std::f32::consts::PI;
        let axis_plus = wrap(phase - eighth);
        let axis_minus = wrap(phase + eighth);
        let Some(prev_axis) = self.last_axis else {
            self.last_axis = Some(axis_plus);
            return Vec::new();
        };
        let (theta, swing) = if wrap(axis_plus - prev_axis).abs() < wrap(axis_minus - prev_axis).abs()
        {
            (axis_plus, 1.0f32)
        } else {
            (axis_minus, -1.0f32)
        };
        self.last_axis = Some(theta);

        // Quadrature demodulate against that reference and box filter to
        // about 1.3 MHz, which is the chroma bandwidth.
        let taps = ((self.rate / 2.6e6).round() as usize).max(1);
        // The burst is the reference for chroma the way the sync tip is for
        // luma: it is sent at 0.15 V where white is 0.7 V above blanking, so
        // dividing by the measured burst and multiplying by that ratio puts
        // U and V in the same units as the luma the picture is built from.
        // Leaving chroma in volts against a luma normalised to white halves
        // the saturation, which off air looks like a camera with the colour
        // turned down rather than like a bug.
        let gain = (0.15 / 0.7) / amp;
        // The reference, once per line, as a rotation rather than a sine per
        // sample. Every output pixel integrates eight samples against it, so
        // a 20 MS/s span was asking for seventy million sines a second and
        // the separator cost nearly twice real time on its own. Seeded from
        // the exact phase at the start of the line and advanced by one
        // sample's rotation, which over a line is far inside the precision
        // the burst itself is measured to.
        let need = (span + taps).min(raw.len().saturating_sub(start));
        let (dc, ds) = ((w as f32).cos(), (w as f32).sin());
        let mut osc: Vec<(f32, f32)> = Vec::with_capacity(need);
        let p0 = phase_at(start) + theta;
        let (mut c, mut sn) = (p0.cos(), p0.sin());
        for _ in 0..need {
            osc.push((c, sn));
            let (nc, ns) = (c * dc - sn * ds, sn * dc + c * ds);
            c = nc;
            sn = ns;
        }
        let mut out = Vec::with_capacity(self.width);
        for i in 0..self.width {
            let at = start + i * span / self.width.max(1);
            let (mut u, mut v) = (0.0f32, 0.0f32);
            let mut count = 0.0f32;
            for k in 0..taps {
                let Some(&x) = raw.get(at + k) else { break };
                let Some(&(pc, ps)) = osc.get(at + k - start) else { break };
                u += x * pc;
                v += x * ps;
                count += 1.0;
            }
            if count == 0.0 {
                out.push((0.0, 0.0));
                continue;
            }
            // Scaled by the burst, so the picture's colours are referred to
            // the amplitude the transmitter sent rather than to the
            // demodulator's own scale.
            let u = 2.0 * u / count * gain;
            let v = 2.0 * v / count * gain * swing;
            out.push((u, v));
        }
        // The delay line: mean with the line above.
        if let Some(prev_row) = self.chroma.last() {
            if prev_row.len() == out.len() {
                for (o, p) in out.iter_mut().zip(prev_row) {
                    o.0 = (o.0 + p.0) / 2.0;
                    o.1 = (o.1 + p.1) / 2.0;
                }
            }
        }
        out
    }

    fn finish_field(&mut self, out: &mut Vec<Field>) {
        let rgb = self.colour.then(|| self.to_rgb()).flatten();
        let height = self.standard.active_lines();
        let seen = std::mem::take(&mut self.lines_seen);
        if seen < height / 4 {
            // Fewer than a quarter of the lines is a fade or a false start,
            // not a picture.
            self.lines.clear();
            self.chroma.clear();
            self.last_axis = None;
            self.stats.fields_short += 1;
            return;
        }
        let mut luma = Vec::with_capacity(self.width * height);
        for r in 0..height {
            match self.lines.get(r) {
                Some(row) => luma.extend_from_slice(row),
                // A line that never arrived is black rather than a repeat of
                // its neighbour: a receiver should be able to see what it
                // missed.
                None => luma.extend(std::iter::repeat_n(0u8, self.width)),
            }
        }
        self.lines.clear();
        self.chroma.clear();
        self.last_axis = None;
        self.stats.fields += 1;
        out.push(Field {
            width: self.width,
            height,
            aspect: self.standard.aspect(),
            luma,
            rgb,
            lines_seen: seen,
        });
    }

    /// Combine the luma rows and the chroma rows into RGB.
    ///
    /// `None` when too few lines carried a burst to call it a colour picture,
    /// which is what a monochrome camera looks like and is not a failure.
    fn to_rgb(&self) -> Option<Vec<u8>> {
        let height = self.standard.active_lines();
        let with_burst = self.chroma.iter().filter(|r| !r.is_empty()).count();
        if with_burst * 2 < self.lines.len() {
            return None;
        }
        let mut out = Vec::with_capacity(self.width * height * 3);
        for r in 0..height {
            for c in 0..self.width {
                let y = self
                    .lines
                    .get(r)
                    .and_then(|row| row.get(c))
                    .map(|&v| f32::from(v) / 255.0)
                    .unwrap_or(0.0);
                let (u, v) = self
                    .chroma
                    .get(r)
                    .and_then(|row| row.get(c))
                    .copied()
                    .unwrap_or((0.0, 0.0));
                // U and V are the weighted colour differences, so undoing the
                // weights gives B-Y and R-Y, and green follows from the luma
                // equation.
                let bmy = u / 0.493;
                let rmy = v / 0.877;
                let g = y - 0.5094 * rmy - 0.1942 * bmy;
                for ch in [y + rmy, g, y + bmy] {
                    out.push((ch.clamp(0.0, 1.0) * 255.0) as u8);
                }
            }
        }
        Some(out)
    }

    pub fn reset(&mut self) {
        self.since_sync = 0;
        self.lines.clear();
        self.lines_seen = 0;
        self.low_run = 0;
        self.line.clear();
        self.hist.clear();
        self.primed = false;
        self.raw_line.clear();
        self.chroma.clear();
        self.last_axis = None;
    }
}

/// An angle folded into -pi to pi.
fn wrap(a: f32) -> f32 {
    let t = std::f32::consts::TAU;
    let mut a = a % t;
    if a > std::f32::consts::PI {
        a -= t;
    }
    if a < -std::f32::consts::PI {
        a += t;
    }
    a
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Build composite video the way a camera does: sync, back porch,
    /// picture, front porch, with a vertical pulse between fields. Levels are
    /// the 1 V standard, sync at -0.3 and white at +0.7 above blanking.
    fn synth(standard: Standard, rate: f64, fields: usize, ramp: bool) -> Vec<f32> {
        let n = |s: f64| (s * rate).round() as usize;
        let line = n(standard.line_s());
        let sync = n(standard.sync_s());
        let back = n(standard.back_porch_s());
        let active = n(standard.active_s());
        let mut out = Vec::new();
        for _ in 0..fields {
            for r in 0..standard.active_lines() {
                out.extend(std::iter::repeat_n(-0.3f32, sync));
                out.extend(std::iter::repeat_n(0.0f32, back));
                for i in 0..active {
                    // A horizontal ramp, so a picture read back at the wrong
                    // offset is visibly wrong rather than plausibly grey.
                    let v = if ramp {
                        i as f32 / active as f32
                    } else {
                        r as f32 / standard.active_lines() as f32
                    };
                    out.push(v * 0.7);
                }
                let used = sync + back + active;
                out.extend(std::iter::repeat_n(0.0f32, line.saturating_sub(used)));
            }
            // The vertical interval, one broad pulse long enough to be told
            // from a line sync.
            out.extend(std::iter::repeat_n(-0.3f32, n(27.3e-6)));
            out.extend(std::iter::repeat_n(0.0f32, line));
        }
        out
    }

    #[test]
    fn a_field_comes_back_with_its_lines() {
        let rate = 16e6;
        let v = synth(Standard::Pal, rate, 3, true);
        let mut sep = SyncSeparator::new(rate, Standard::Pal, 320);
        let mut out = Vec::new();
        sep.process(&v, &mut out);
        assert!(!out.is_empty(), "no field came out");
        let f = out.last().unwrap();
        assert_eq!(f.height, 288);
        assert_eq!(f.width, 320);
        assert!(
            f.lines_seen > 250,
            "only {} lines of 288 were found",
            f.lines_seen
        );
    }

    /// The picture has to come back the right way round and at the right
    /// level: a ramp that reads flat means the line was cut in the wrong
    /// place, and one that reads reversed means the sample order is wrong.
    #[test]
    fn the_picture_is_the_one_that_was_transmitted() {
        let rate = 16e6;
        let v = synth(Standard::Pal, rate, 3, true);
        let mut sep = SyncSeparator::new(rate, Standard::Pal, 320);
        let mut out = Vec::new();
        sep.process(&v, &mut out);
        let f = out.last().expect("a field");
        let row = &f.luma[f.width * 100..f.width * 101];
        assert!(row[10] < 40, "the left of a ramp should be dark: {}", row[10]);
        assert!(row[300] > 200, "the right should be white: {}", row[300]);
        for w in row.windows(2).step_by(16) {
            assert!(w[1] + 8 >= w[0], "the ramp is not monotonic");
        }
    }

    /// Two standards, told apart by the line period rather than by being
    /// configured. They are 0.7% apart, which no transmitter's timebase
    /// error reaches.
    #[test]
    fn the_line_period_names_the_standard() {
        assert_eq!(Standard::from_line_period(64e-6), Some(Standard::Pal));
        assert_eq!(Standard::from_line_period(63.55e-6), Some(Standard::Ntsc));
        assert_eq!(Standard::from_line_period(50e-6), None);
    }

    /// Build one line of PAL with a burst and a chroma vector on it. `uv` is
    /// the colour in the same weighted units the decoder reports, and `swing`
    /// is the line's V polarity.
    ///
    /// The subcarrier phase is counted from the start of the back porch,
    /// which is where the decoder's line buffer begins.
    fn colour_line(
        out: &mut Vec<f32>,
        standard: Standard,
        rate: f64,
        y: f32,
        uv: (f32, f32),
        swing: f32,
    ) {
        let n = |s: f64| (s * rate).round() as usize;
        let w = std::f64::consts::TAU * standard.subcarrier_hz() / rate;
        // The subcarrier runs continuously through the whole signal, sync
        // pulses included, as a transmitter's does.
        let base = out.len();
        let phase_at = |k: usize| (w * (base + k) as f64) as f32;
        out.extend(std::iter::repeat_n(-0.3f32, n(standard.sync_s())));

        let back = n(standard.back_porch_s());
        let burst_at = n(0.9e-6);
        let burst_len = n(2.25e-6);
        let sync_len = n(standard.sync_s());
        for k in 0..back {
            let p = phase_at(sync_len + k);
            // The burst is -U plus or minus V at 0.15, which is 135 or 225
            // degrees.
            let v = if (burst_at..burst_at + burst_len).contains(&k) {
                0.15 * (-p.cos() + swing * p.sin()) / std::f32::consts::SQRT_2
            } else {
                0.0
            };
            out.push(v);
        }
        let active = n(standard.active_s());
        for k in 0..active {
            let p = phase_at(sync_len + back + k);
            out.push(y * 0.7 + uv.0 * p.cos() + swing * uv.1 * p.sin());
        }
        let used = sync_len + back + active;
        out.extend(std::iter::repeat_n(
            0.0f32,
            n(standard.line_s()).saturating_sub(used),
        ));
    }

    /// Colour bars through the whole chain: a burst, a chroma vector, the
    /// line flip, and the delay line that makes the flip worth having.
    #[test]
    fn a_colour_field_comes_back_as_the_colours_that_were_sent() {
        let rate = 20e6;
        let standard = Standard::Pal;
        // A mid blue and a mid red, in the weighted units the decoder
        // reports: U is 0.493(B-Y) and V is 0.877(R-Y).
        let blue = (0.493 * 0.5, 0.0);
        let mut v = Vec::new();
        for _ in 0..3 {
            for r in 0..standard.active_lines() {
                let swing = if r % 2 == 0 { 1.0 } else { -1.0 };
                colour_line(&mut v, standard, rate, 0.5, blue, swing);
            }
            v.extend(std::iter::repeat_n(-0.3f32, (27.3e-6 * rate) as usize));
            v.extend(std::iter::repeat_n(0.0f32, (standard.line_s() * rate) as usize));
        }

        let mut sep = SyncSeparator::new(rate, standard, 320).with_colour();
        let mut out = Vec::new();
        sep.process(&v, &mut out);
        let f = out.last().expect("a field");
        let rgb = f.rgb.as_ref().expect("colour");
        // Sample the middle of the picture, away from the edges where the
        // box filter is still filling.
        let at = (f.width * 150 + 160) * 3;
        let (r, g, b) = (rgb[at], rgb[at + 1], rgb[at + 2]);
        assert!(
            b > r + 40 && b > g + 40,
            "a blue vector should read blue, got r{r} g{g} b{b}"
        );
    }

    /// The positive test: a camera's sync pulses agree with each other, and
    /// nothing else does. This is what keeps a video front end off the wide
    /// sources that are not cameras, rather than letting it demodulate every
    /// block of one forever.
    #[test]
    fn a_camera_is_told_from_everything_else_by_its_line_rate() {
        let rate = 16e6;
        let lock = find_lines(&synth(Standard::Pal, rate, 3, true), rate).expect("a camera");
        assert_eq!(lock.standard, Standard::Pal);
        assert!(lock.agreement > 0.9, "agreement {}", lock.agreement);
        assert!(lock.pulses > 500, "{} pulses", lock.pulses);

        let ntsc = find_lines(&synth(Standard::Ntsc, rate, 3, true), rate).expect("a camera");
        assert_eq!(ntsc.standard, Standard::Ntsc);
    }

    #[test]
    fn noise_is_not_a_camera() {
        let mut seed = 0x2545_f491_4f6c_dd1du64;
        let noise: Vec<f32> = (0..2_000_000)
            .map(|_| {
                seed ^= seed << 13;
                seed ^= seed >> 7;
                seed ^= seed << 17;
                (seed >> 40) as f32 / 8_388_608.0 - 1.0
            })
            .collect();
        assert_eq!(find_lines(&noise, 16e6), None);

        // Nor is a keyed carrier: a burst train at some other rate has gaps
        // that do not land on a line period, which is the case a median alone
        // would fall for.
        let rate = 16e6;
        let mut keyed = Vec::new();
        for i in 0..2000 {
            let on = ((i as f64 * 0.0007 * rate) as usize % 90) + 40;
            keyed.extend(std::iter::repeat_n(-0.3f32, on));
            keyed.extend(std::iter::repeat_n(0.2f32, 700));
        }
        assert_eq!(find_lines(&keyed, rate), None);
    }

    #[test]
    fn noise_produces_no_field() {
        let mut seed = 0x2545_f491_4f6c_dd1du64;
        let v: Vec<f32> = (0..2_000_000)
            .map(|_| {
                seed ^= seed << 13;
                seed ^= seed >> 7;
                seed ^= seed << 17;
                (seed >> 40) as f32 / 8_388_608.0 - 1.0
            })
            .collect();
        let mut sep = SyncSeparator::new(16e6, Standard::Pal, 320);
        let mut out = Vec::new();
        sep.process(&v, &mut out);
        assert!(out.is_empty(), "noise produced {} fields", out.len());
    }

    /// The picture's shape is the standard's, not the sample grid's. Drawn
    /// from its own numbers a field is 10:9 and looks like a 4:3 picture
    /// with the sides pushed in.
    #[test]
    fn a_field_carries_the_shape_of_the_picture_and_not_of_its_samples() {
        let rate = 20e6;
        let mut sep = SyncSeparator::new(rate, Standard::Pal, 640);
        let mut fields = Vec::new();
        sep.process(&synth(Standard::Pal, rate, 3, true), &mut fields);
        let f = fields.first().expect("a field");
        assert!((f.aspect - 4.0 / 3.0).abs() < 1e-6, "{}", f.aspect);
        assert_ne!(
            f.aspect,
            f.width as f32 / (f.height as f32 * 2.0),
            "the sample grid decided the shape"
        );
    }
}
