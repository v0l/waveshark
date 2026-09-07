//! Analogue television baseband: sync separation and a picture out of it.
//!
//! Analogue FPV is not a protocol. A camera produces composite video, the
//! transmitter frequency modulates a 5.8 GHz carrier with it, and that is the
//! whole stack: no framing, no addressing, no integrity check anywhere. What
//! carries structure is the video itself, and it is the structure television
//! had in 1960: a horizontal sync pulse below black at the start of every
//! line, a pattern of broad pulses between fields, and two interlaced fields
//! to a frame.
//!
//! So this file takes the output of an FM demodulator and finds those pulses.
//! Levels are relative, because the demodulator's output scale depends on the
//! deviation it was told and a transmitter's deviation is whatever its
//! designer chose: sync tip and blanking are measured from the signal itself
//! rather than assumed, the way `crate::ble`'s gate tracks its own floor.
//!
//! # What is not here
//!
//! Colour. PAL carries chrominance on a 4.43 MHz subcarrier whose phase
//! alternates line to line, and recovering it means a burst-locked oscillator
//! and a delay line. Luma alone is a grey picture, which is what an FPV feed
//! is mostly judged on and all that is needed to say what a camera is looking
//! at. The subcarrier is still in the samples for whoever writes it.
//!
//! Audio, for the same reason: most transmitters put it on a 6.0 or 6.5 MHz
//! subcarrier, which is another demodulator on this same baseband.

/// Which set of timings the camera is using.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Standard {
    /// 625 lines, 25 frames, 15.625 kHz line rate. What almost every FPV
    /// camera sold outside North America produces.
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

    /// The standard a measured line period names, or `None` when it is
    /// neither. The two are 0.7% apart, which is far wider than a
    /// transmitter's timebase error, so measuring settles it.
    pub fn from_line_period(period_s: f64) -> Option<Self> {
        for s in [Self::Pal, Self::Ntsc] {
            if (period_s / s.line_s() - 1.0).abs() < 0.003 {
                return Some(s);
            }
        }
        None
    }
}

/// One field, as luma samples.
#[derive(Clone, Debug)]
pub struct Field {
    pub width: usize,
    pub height: usize,
    /// Row major, 0 for sync-black and 255 for peak white.
    pub luma: Vec<u8>,
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
    /// Where the last line started, so the picture can be cut out of it.
    line: Vec<f32>,
}

impl SyncSeparator {
    /// `width` is the picture width to resample each line to. 720 is what a
    /// PAL line holds at broadcast sampling; an FPV camera is usually 600 or
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
            line: Vec::new(),
        }
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

    /// Feed demodulated baseband. Whole fields come back as they complete.
    pub fn process(&mut self, baseband: &[f32], out: &mut Vec<Field>) {
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
        for &x in baseband {
            self.line.push(x);
            self.since_sync += 1;
            if x < thresh {
                self.low_run += 1;
                continue;
            }
            let run = std::mem::take(&mut self.low_run);
            if run < h_min {
                continue;
            }
            if run >= broad {
                // A broad pulse: the field is over. What is in hand is the
                // field, whether or not every line arrived.
                self.finish_field(out);
                self.since_sync = 0;
                self.line.clear();
                continue;
            }
            // An ordinary line sync. The line just ended is cut and kept.
            self.take_line(run);
            self.since_sync = 0;
        }
    }

    /// Cut the picture out of the line that just ended and store it.
    ///
    /// The buffer runs from the end of the previous line's sync pulse, so it
    /// holds the back porch, the picture, the front porch and the sync that
    /// ended it. `sync_run` is that trailing sync, which is where the
    /// picture's far end is measured back from.
    fn take_line(&mut self, sync_run: usize) {
        let line = std::mem::take(&mut self.line);
        self.line = Vec::with_capacity(line.len());
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
    }

    fn finish_field(&mut self, out: &mut Vec<Field>) {
        let height = self.standard.active_lines();
        let seen = std::mem::take(&mut self.lines_seen);
        if seen < height / 4 {
            // Fewer than a quarter of the lines is a fade or a false start,
            // not a picture.
            self.lines.clear();
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
        out.push(Field {
            width: self.width,
            height,
            luma,
            lines_seen: seen,
        });
    }

    pub fn reset(&mut self) {
        self.since_sync = 0;
        self.lines.clear();
        self.lines_seen = 0;
        self.low_run = 0;
        self.line.clear();
        self.hist.clear();
        self.primed = false;
    }
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
}
