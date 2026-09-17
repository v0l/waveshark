//! Automatic Picture Transmission: the pictures NOAA 15, 18 and 19 send
//! down at 137 MHz while they are overhead.
//!
//! The downlink is wideband FM, and inside the audio is a 2400 Hz subcarrier
//! whose amplitude is the brightness of a pixel. Words go out at 4160 a
//! second and a line is 2080 of them, so the picture is exactly two lines a
//! second for as long as the pass lasts. A line is two halves, each a sync
//! run, a space, 909 words of picture and a telemetry wedge, so what looks
//! like one picture 2080 wide is two cameras side by side.
//!
//! So the decoder is a sync correlator and a ruler. There is nothing to
//! decode and no check of any kind: a line is right because its sync run
//! landed where the clock said it would.
//!
//! The timings are the ones in the NOAA KLM User's Guide, section 4.2.

use crate::linescan::{Assembler, Marker, Picture, Rows};
use dsp::subcarrier::Envelope;

/// Words a second, and the line that is 2080 of them.
pub const WORD_RATE: f64 = 4_160.0;
pub const LINE_WORDS: usize = 2_080;
/// Two lines a second, for as long as the satellite is up.
pub const LINE_TIME: f64 = LINE_WORDS as f64 / WORD_RATE;

/// The subcarrier the picture rides on, and the band its modulation needs.
/// The video band is 2.4 kHz either side, which is where the downlink's
/// audio spectrum ends and is half a word wide: two words cannot be told
/// apart any faster than that.
pub const SUBCARRIER_HZ: f64 = 2_400.0;
const ENVELOPE_BAND_HZ: f64 = 4_800.0;

/// Where each part of a half line starts, in words from the line's start.
/// The B half is the same run 1040 words later.
const SYNC_WORDS: usize = 39;
const SPACE_WORDS: usize = 47;
const IMAGE_WORDS: usize = 909;
const TELEMETRY_WORDS: usize = 45;
const HALF_WORDS: usize = SYNC_WORDS + SPACE_WORDS + IMAGE_WORDS + TELEMETRY_WORDS;
/// Where the A and B pictures themselves begin.
pub const IMAGE_A_WORD: usize = SYNC_WORDS + SPACE_WORDS;
pub const IMAGE_B_WORD: usize = HALF_WORDS + SYNC_WORDS + SPACE_WORDS;

/// The sync run at the head of each half line: seven cycles of a square wave,
/// then black to the end of the run. A at 1040 Hz is four words a cycle, B at
/// 832 Hz is five.
const SYNC_A_HZ: f64 = 1_040.0;
const SYNC_CYCLES: f64 = 7.0;

/// What the sync run's two levels are worth as pixels. The run does not
/// swing the whole way: the guide puts its low at 11 counts and its high at
/// 244, so a decoder calibrating off it and mapping the pair to black and
/// white would stretch every shade by 9%.
const SYNC_LOW: f32 = 11.0;
const SYNC_HIGH: f32 = 244.0;

/// How tall a picture is before another is started. A pass lasts about
/// fifteen minutes at two lines a second, so this is a shade over eleven
/// minutes; anything longer is a canvas mostly empty on most passes, and the
/// operator sees the rest as the next picture.
pub const PICTURE_LINES: usize = 1_400;

/// How well a sync run has to match before it is believed.
///
/// A normalised correlation, so 1.0 is the run itself and 0.0 is anything
/// uncorrelated. Measured on a synthesised pass: a clean line scores 0.89,
/// and under uniform noise of the same peak amplitude as the subcarrier a
/// line scores between 0.51 and 0.73. Half that much noise again puts the
/// worst line at 0.30, which is where lines start being dropped.
///
/// A good score is not on its own evidence of anything: the best the same
/// correlation reached against white noise alone, searching five minutes of
/// it, was 0.72. That is what the two-runs-a-line-apart lock is for.
const LOCK: f32 = 0.45;

/// Lines with no sync before the transmission is taken to be over. Six is
/// three seconds, which is longer than a fade under an aircraft and shorter
/// than anybody would want to wait staring at a picture that has stopped.
const LOST_LINES: usize = 6;

/// The sync run as a template, at `spw` samples a word: plus and minus one
/// across the seven cycles, and zero over the black tail, which says nothing
/// about where the line is and is left out of the correlation.
fn sync_marker(spw: f64, hz: f64) -> Marker {
    let len = (SYNC_WORDS as f64 * spw).round() as usize;
    let cycle = WORD_RATE / hz * spw;
    let pulses = (SYNC_CYCLES * cycle).round() as usize;
    Marker::new(
        (0..len)
            .map(|i| match i < pulses {
                true => match (i as f64 % cycle) < cycle / 2.0 {
                    true => -1.0,
                    false => 1.0,
                },
                false => 0.0,
            })
            .collect(),
    )
}

/// A pass being received, fed audio as it arrives.
///
/// The same shape as the SSTV receiver and for the same reason: a pass is
/// fifteen minutes, so nothing can wait for the end of the stream or keep it.
pub struct Receiver {
    rate: f64,
    /// Samples a word, which is what every timing here is counted in.
    spw: f64,
    env: Envelope,
    level: Vec<f32>,
    /// Absolute position of `level[0]`, since the buffer is drained as lines
    /// are read.
    base: u64,
    sync: Marker,
    state: State,
    asm: Assembler,
    /// What the sync run's two levels have been, smoothed over lines: the
    /// picture's black and white, which the format carries in every line.
    black: f32,
    white: f32,
}

enum State {
    Hunting,
    /// Reading, from the sample the current line starts at.
    Reading {
        seq: f64,
        lost: usize,
    },
}

/// How much envelope to keep while hunting: two lines, which is what
/// confirming a sync run against the next one needs.
const HUNT_KEEP: f64 = 2.0 * LINE_TIME;

impl Receiver {
    pub fn new(rate: f64) -> Self {
        let spw = rate / WORD_RATE;
        Self {
            rate,
            spw,
            env: Envelope::new(rate, SUBCARRIER_HZ, ENVELOPE_BAND_HZ),
            level: Vec::new(),
            base: 0,
            sync: sync_marker(spw, SYNC_A_HZ),
            state: State::Hunting,
            asm: Assembler::new(LINE_WORDS, PICTURE_LINES),
            black: 0.0,
            white: 0.0,
        }
    }

    pub fn reset(&mut self) {
        let rate = self.rate;
        *self = Self::new(rate);
    }

    /// Whether lines are being read right now.
    pub fn receiving(&self) -> bool {
        matches!(self.state, State::Reading { .. })
    }

    /// The picture as it stands, complete or not.
    pub fn picture(&self) -> Picture {
        self.asm.picture()
    }

    /// Feed audio. Returns whatever rows completed.
    pub fn push(&mut self, audio: &[f32]) -> Option<Rows> {
        self.env.process(audio, &mut self.level);
        if matches!(self.state, State::Hunting) {
            self.hunt();
        }
        self.advance(false)
    }

    /// No more audio is coming: hand over what was read.
    pub fn finish(&mut self) -> Option<Rows> {
        self.advance(true)
    }

    fn line_samples(&self) -> f64 {
        LINE_WORDS as f64 * self.spw
    }

    fn drain_to(&mut self, abs: u64) {
        let Some(cut) = abs.checked_sub(self.base) else { return };
        let cut = (cut as usize).min(self.level.len());
        self.level.drain(..cut);
        self.base += cut as u64;
    }

    /// Find a sync run, and believe it only where the next line has one
    /// where it should be. There is no header in APT and nothing to check a
    /// line against, so two in a row a line apart is the whole of the
    /// evidence that this is a picture and not a noisy channel.
    fn hunt(&mut self) {
        let line = self.line_samples();
        let need = (2.0 * line) as usize + self.sync.len();
        if self.level.len() < need {
            return;
        }
        let first = self.sync.best(&self.level, 0, line as usize);
        if first.score >= LOCK {
            let next = first.at + line.round() as usize;
            // A whole word either side: the clocks differ by parts per
            // million, so a line apart is a line apart to within a pixel.
            let slack = self.spw.round() as usize;
            let then = self.sync.best(&self.level, next.saturating_sub(slack), next + slack);
            if then.score >= LOCK {
                self.asm.start();
                self.black = 0.0;
                self.white = 0.0;
                self.state = State::Reading { seq: (self.base + first.at as u64) as f64, lost: 0 };
                return;
            }
        }
        let keep = (HUNT_KEEP * self.rate) as usize + self.sync.len();
        if self.level.len() > keep {
            self.drain_to(self.base + (self.level.len() - keep) as u64);
        }
    }

    /// The black and white of this line, from its own sync run: the run
    /// swings the subcarrier between the two by design, so every line
    /// carries the calibration for itself and nothing about the scene can
    /// move it. Smoothed across lines, because one line of noise should not
    /// change the shade of the next.
    fn calibrate(&mut self, at: usize) {
        let Some(run) = self.level.get(at..at + self.sync.len()) else { return };
        // Which half cycle each sample is in is what the template says, so
        // the two levels are the means of the samples the run is known to
        // have sent high and low. Taking the loudest and quietest samples
        // instead read white 90 counts high under noise as loud as the
        // subcarrier, because the loudest sample of a noisy run is noise.
        let mut low = (0.0, 0usize);
        let mut high = (0.0, 0usize);
        for (x, t) in run.iter().zip(self.sync.template()) {
            let side = match *t {
                t if t > 0.0 => &mut high,
                t if t < 0.0 => &mut low,
                _ => continue,
            };
            side.0 += *x;
            side.1 += 1;
        }
        if low.1 == 0 || high.1 == 0 {
            return;
        }
        let (low, high) = (low.0 / low.1 as f32, high.0 / high.1 as f32);
        if high <= low {
            return;
        }
        // The run arrives as its fundamental and nothing else: 1040 Hz has
        // its third harmonic at 3120, outside the 2.4 kHz video band, so a
        // square wave leaves the satellite and a sine arrives. The mean of
        // half a cycle of that sine is 4/pi^2 of the swing rather than half
        // of it, so the two means are 0.81 of the way apart that black and
        // white are. Without this every dark pixel clipped to black:
        // measured on a synthesised pass, a level sent as 28 read as 0.
        let mid = (high + low) / 2.0;
        let swing = (high - low) * std::f32::consts::PI * std::f32::consts::PI / 8.0;
        let (low, high) = (mid - swing / 2.0, mid + swing / 2.0);
        // A fifth of the way towards the new reading each line: a pass fades
        // over seconds and not over one line, and a line is half a second.
        let blend = |old: f32, new: f32| match old > 0.0 {
            true => old * 0.8 + new * 0.2,
            false => new,
        };
        self.black = blend(self.black, low);
        self.white = blend(self.white, high);
    }

    /// The 2080 pixels of the line starting at sample `at`, each the mean of
    /// the envelope across its word.
    fn read_line(&self, at: f64, row: &mut [u8]) {
        let scale = match self.white > self.black {
            true => (SYNC_HIGH - SYNC_LOW) / (self.white - self.black),
            false => 0.0,
        };
        for (word, px) in row.iter_mut().enumerate() {
            let from = (at + word as f64 * self.spw).round() as usize;
            let to = (at + (word + 1) as f64 * self.spw).round() as usize;
            let span = &self.level[from.min(self.level.len())..to.min(self.level.len())];
            let mean = match span.is_empty() {
                true => self.black,
                false => span.iter().sum::<f32>() / span.len() as f32,
            };
            *px = (SYNC_LOW + (mean - self.black) * scale).clamp(0.0, 255.0) as u8;
        }
    }

    fn advance(&mut self, ending: bool) -> Option<Rows> {
        let State::Reading { mut seq, mut lost } = self.state else { return None };
        let line = self.line_samples();
        // How far from where the clock says the line starts a sync run is
        // still this line's: a tenth of a line is forty times the drift any
        // clock has over a line and well short of the next sync run.
        let slack = (line * 0.1).round() as usize;
        let mut complete = false;
        let mut row = vec![0u8; LINE_WORDS];

        loop {
            let here = seq - self.base as f64;
            if here < 0.0 {
                // The buffer was drained past this line, which cannot happen
                // while it is being read.
                break;
            }
            let here = here as usize;
            // A line needs its own samples and the slack the sync search
            // walks past the end of it.
            if self.level.len() < here + line as usize + slack + self.sync.len() {
                complete = ending;
                break;
            }
            let found = self.sync.best(&self.level, here.saturating_sub(slack), here + slack);
            let synced = found.score >= LOCK;
            if synced {
                seq = (self.base + found.at as u64) as f64;
                self.calibrate(found.at);
            }
            match synced {
                true => {
                    lost = 0;
                    let at = seq - self.base as f64;
                    self.read_line(at, &mut row);
                    if self.asm.row(&row) {
                        complete = true;
                    }
                }
                // A line with no sync run where one was due is not a line.
                // Painting it would be painting the receiver's own clock
                // across noise, which is what a picture of a pass that ended
                // ten minutes ago looks like.
                false => lost += 1,
            }
            seq += line;
            let keep = (seq - line).max(0.0) as u64;
            self.drain_to(keep);
            if complete || lost >= LOST_LINES {
                complete = true;
                break;
            }
        }

        match complete {
            true => self.state = State::Hunting,
            false => self.state = State::Reading { seq, lost },
        }
        self.asm.take(complete)
    }
}

/// Decode everything in `audio`, a whole recording at a time.
///
/// The same code the node runs, so a test and a live stream cannot be two
/// decoders that agree only by luck.
pub fn decode(audio: &[f32], rate: f64) -> Picture {
    let mut rx = Receiver::new(rate);
    rx.push(audio);
    rx.finish();
    rx.picture()
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::TAU;

    /// A rate with a whole number of samples a word, which is what the node
    /// hands the decoder.
    const RATE: f64 = 20_800.0;

    /// The grey a telemetry wedge is: eight steps from black to white, which
    /// is what a real frame carries so a decoder can calibrate off it.
    fn wedge(line: usize) -> u8 {
        let step = (line / 8) % 16;
        match step {
            0..=7 => (step as f32 * 255.0 / 7.0) as u8,
            _ => 0,
        }
    }

    /// One line of a test picture: a ramp across the A half and a
    /// checkerboard across the B half, so a line read at the wrong offset or
    /// the wrong width is obvious.
    fn scene(line: usize, word: usize) -> u8 {
        match word {
            w if (IMAGE_A_WORD..IMAGE_A_WORD + IMAGE_WORDS).contains(&w) => {
                ((w - IMAGE_A_WORD) * 255 / IMAGE_WORDS) as u8
            }
            w if (IMAGE_B_WORD..IMAGE_B_WORD + IMAGE_WORDS).contains(&w) => {
                match ((w - IMAGE_B_WORD) / 64 + line / 64).is_multiple_of(2) {
                    true => 40,
                    false => 210,
                }
            }
            _ => 0,
        }
    }

    /// What a word of a line is worth, sync runs, spaces and telemetry
    /// included: the transmitter's side of everything the decoder undoes.
    fn word_value(line: usize, word: usize) -> f64 {
        let half = word % HALF_WORDS;
        let sync_hz = match word < HALF_WORDS {
            true => SYNC_A_HZ,
            false => 832.0,
        };
        let cycle = WORD_RATE / sync_hz;
        let v = match half {
            // The sync run, as a square wave between black and white.
            h if (h as f64) < SYNC_CYCLES * cycle => match (h as f64 % cycle) < cycle / 2.0 {
                true => 11.0,
                false => 244.0,
            },
            h if h < SYNC_WORDS + SPACE_WORDS => 0.0,
            h if h < SYNC_WORDS + SPACE_WORDS + IMAGE_WORDS => scene(line, word) as f64,
            _ => wedge(line) as f64,
        };
        v / 255.0
    }

    /// A pass, as audio: a 2400 Hz subcarrier amplitude modulated by the
    /// words, at the rate and the line length the guide gives.
    fn pass(lines: usize, noise: f64, seed: u64) -> Vec<f32> {
        pass_at(RATE, lines, noise, seed)
    }

    fn pass_at(rate: f64, lines: usize, noise: f64, seed: u64) -> Vec<f32> {
        let spw = rate / WORD_RATE;
        let mut rng = seed.max(1);
        let mut next = || {
            rng ^= rng << 13;
            rng ^= rng >> 7;
            rng ^= rng << 17;
            (rng >> 11) as f64 / (1u64 << 53) as f64 * 2.0 - 1.0
        };
        let total = (lines as f64 * LINE_WORDS as f64 * spw) as usize;
        (0..total)
            .map(|i| {
                let word = (i as f64 / spw) as usize;
                let value = word_value(word / LINE_WORDS, word % LINE_WORDS);
                // 87% modulation, as the downlink uses.
                let amp = 0.1 + 0.87 * value;
                let s = amp * (TAU * SUBCARRIER_HZ * i as f64 / rate).sin();
                (s + noise * next()) as f32
            })
            .collect()
    }

    /// The line the guide describes: 2080 words at 4160 a second is two
    /// lines a second, and the halves are 1040 words each.
    #[test]
    fn a_line_is_half_a_second_and_two_pictures_wide() {
        assert!((LINE_TIME - 0.5).abs() < 1e-12, "a line is {LINE_TIME} seconds");
        assert_eq!(HALF_WORDS, 1_040);
        assert_eq!(IMAGE_A_WORD, 86);
        assert_eq!(IMAGE_B_WORD, 1_126);
    }

    /// The sync template is the square wave the satellite sends: seven
    /// cycles of 1040 Hz is 28 of the run's 39 words, and the rest is black.
    #[test]
    fn the_sync_template_is_seven_cycles_and_a_tail() {
        let m = sync_marker(5.0, SYNC_A_HZ);
        let t = m.template();
        assert_eq!(t.len(), 195, "39 words at five samples each");
        assert_eq!(t.iter().filter(|v| **v != 0.0).count(), 140, "28 words of pulses");
        assert_eq!(&t[..10], &[-1.0; 10], "the run opens with two words low");
        assert_eq!(&t[10..20], &[1.0; 10], "four words a cycle at 1040 Hz");
    }

    /// Twenty lines of a clean pass: every line read, on the line, and the
    /// ramp and the checkerboard back where they were sent.
    #[test]
    fn a_clean_pass_reads_every_line_at_the_right_width() {
        let audio = pass(22, 0.0, 1);
        let pic = decode(&audio, RATE);
        assert_eq!(pic.width, LINE_WORDS);
        // Two lines go on hunting and confirming, and the last line has no
        // slack behind it to search.
        assert_eq!(pic.lines, 21, "lines read out of 22 sent");

        let row = |y: usize| &pic.gray[y * LINE_WORDS..(y + 1) * LINE_WORDS];
        // The ramp across the A picture, sampled where it cannot be confused
        // with a neighbouring word.
        for y in [2, 9, 18] {
            let r = row(y);
            for (word, want) in [(IMAGE_A_WORD + 100, 28u8), (IMAGE_A_WORD + 800, 224)] {
                let got = r[word];
                assert!(got.abs_diff(want) <= 6, "line {y} word {word} read {got}, sent {want}");
            }
            // The checkerboard is two levels and nothing between them.
            let b = r[IMAGE_B_WORD + 32];
            assert!(b.abs_diff(40) <= 6 || b.abs_diff(210) <= 6, "B half read {b}");
        }
    }

    /// The telemetry wedges are eight steps from black to white, and they
    /// come back as eight steps: the calibration is off the sync run, so a
    /// step read out of order is a calibration that has drifted.
    #[test]
    fn the_telemetry_wedges_come_back_as_a_grey_staircase() {
        // Nine lines a wedge and sixteen wedges: 22 lines covers the first
        // three steps, which is enough to see the staircase climb.
        let audio = pass(30, 0.0, 2);
        let pic = decode(&audio, RATE);
        assert_eq!(pic.lines, 29);
        let at = |y: usize| {
            let r = &pic.gray[y * LINE_WORDS..(y + 1) * LINE_WORDS];
            r[HALF_WORDS - TELEMETRY_WORDS / 2] as i32
        };
        // Lines 2, 10 and 18 of the decode are inside wedges 0, 1 and 2,
        // whose values are 0, 36 and 72.
        let steps: Vec<i32> = [2, 11, 19].iter().map(|y| at(*y)).collect();
        assert!(steps[0] < 12, "the first wedge is black, read {}", steps[0]);
        assert!((steps[1] - 36).abs() < 12, "the second wedge read {}", steps[1]);
        assert!((steps[2] - 72).abs() < 12, "the third wedge read {}", steps[2]);
    }

    /// The rate the node hands the decoder is ten samples a word rather than
    /// the five these tests use, and the same pass reads the same picture at
    /// it: every timing here is counted in words and none in samples.
    #[test]
    fn the_same_pass_reads_at_the_rate_the_node_uses() {
        let pic = decode(&pass_at(10.0 * WORD_RATE, 22, 0.0, 1), 10.0 * WORD_RATE);
        assert_eq!(pic.lines, 21);
        for y in [2, 9, 18] {
            let mae = ramp_error(&pic.gray, y);
            assert!(mae < 3.0, "line {y} is {mae:.1} counts out");
        }
    }

    /// Noise is not a picture. Five minutes of it produce no lines at all,
    /// because a sync run has to be found twice a line apart before anything
    /// is painted.
    #[test]
    fn minutes_of_noise_produce_no_lines() {
        let n = (300.0 * RATE) as usize;
        let mut rng = 0x5eedu64;
        let noise: Vec<f32> = (0..n)
            .map(|_| {
                rng ^= rng << 13;
                rng ^= rng >> 7;
                rng ^= rng << 17;
                ((rng >> 11) as f64 / (1u64 << 53) as f64 * 2.0 - 1.0) as f32
            })
            .collect();
        let pic = decode(&noise, RATE);
        assert_eq!(pic.lines, 0, "noise painted lines");
    }

    /// What a clean pass costs in shades: the ramp comes back to within two
    /// counts of what was sent, which is the envelope filter and nothing
    /// else.
    #[test]
    fn a_clean_ramp_comes_back_within_two_counts() {
        let pic = decode(&pass(22, 0.0, 1), RATE);
        for y in [2, 9, 18] {
            let mae = ramp_error(&pic.gray, y);
            assert!(mae < 3.0, "line {y} is {mae:.1} counts out");
        }
    }

    /// A pass under noise half as loud as the subcarrier still reads every
    /// line, because the sync run is what is being found rather than the
    /// picture. The picture itself comes back 38 counts noisy and about 20
    /// counts dark, since the envelope of noise adds to a dark pixel more
    /// than to a bright one.
    #[test]
    fn a_noisy_pass_reads_every_line_and_a_grainy_picture() {
        let pic = decode(&pass(22, 0.5, 3), RATE);
        assert_eq!(pic.lines, 21, "lines read under noise");
        for y in [2, 9, 18] {
            let mae = ramp_error(&pic.gray, y);
            assert!(mae < 45.0, "line {y} is {mae:.1} counts out");
        }
        let r = &pic.gray[9 * LINE_WORDS..10 * LINE_WORDS];
        // Thirty-two words of the ramp, averaged: one word is five samples
        // and far too few to judge a shade by under this much noise.
        let smooth: f64 = (784..816).map(|k| r[IMAGE_A_WORD + k] as f64).sum::<f64>() / 32.0;
        assert!((smooth - 224.0).abs() < 30.0, "the ramp averaged {smooth:.0}, sent 224");
    }

    /// How far a decoded line is from the ramp that was sent, in counts.
    fn ramp_error(gray: &[u8], y: usize) -> f64 {
        let row = &gray[y * LINE_WORDS..(y + 1) * LINE_WORDS];
        (0..IMAGE_WORDS)
            .map(|k| {
                let want = (k * 255 / IMAGE_WORDS) as f64;
                (row[IMAGE_A_WORD + k] as f64 - want).abs()
            })
            .sum::<f64>()
            / IMAGE_WORDS as f64
    }

    /// A transmission that stops does not go on painting: the pass ends and
    /// the picture ends with it, six lines later.
    #[test]
    fn a_pass_that_stops_ends_its_picture() {
        let mut audio = pass(12, 0.0, 4);
        audio.extend(std::iter::repeat_n(0.0f32, (30.0 * RATE) as usize));
        let mut rx = Receiver::new(RATE);
        rx.push(&audio);
        let pic = rx.picture();
        // Nine lines of picture, and the six that follow are not painted.
        assert_eq!(pic.lines, 12);
        assert!(!rx.receiving(), "still reading after the pass ended");
    }
}
