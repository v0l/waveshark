use crate::display::{self, Mode};
use common::C32;
use dsp::raster::{self, Limits, Periods, Raster};

pub const SEARCH_RATE_HZ: f64 = 2e6;

const WINDOW_FRAMES: f64 = 3.5;

const BACKOFF_S: f64 = 1.0;

const RECHECK_S: f64 = 4.0;

const PUBLISH_FRAMES: u64 = 12;

/// Frames of a 640x480 desktop correlate with the average at a median of
/// 0.998 and frames of noise alone at 0.229 at most, so a lock whose frames
/// mostly fail to correlate is not a screen.
const MIN_MATCHED: f64 = 0.5;

/// Line rates outside the table's own, with a fifth either side: 3840x2160
/// runs the fastest line in the table at 135 kHz and 640x480 the slowest at
/// 31.5 kHz. A 60 Hz frame of 2400 lines would be 144 kHz, which the refresh
/// and line ranges allow and no display does.
fn line_hz_limits() -> (f64, f64) {
    let (mut lo, mut hi) = (f64::MAX, f64::MIN);
    for m in display::modes() {
        lo = lo.min(m.line_hz());
        hi = hi.max(m.line_hz());
    }
    (lo * 0.8, hi * 1.2)
}

const MAX_WIDTH: usize = 2048;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Locked {
    pub periods: Periods,
    pub frame_hz: f64,
    pub line_hz: f64,
    pub mode: Option<&'static Mode>,
    /// Where the pixel clock harmonic sits against the dial, for a lock
    /// that found one. `None` is a lock read out of the envelope alone,
    /// which is averaged as magnitudes.
    pub carrier_hz: Option<f64>,
}

impl Locked {
    pub fn label(&self) -> String {
        match self.mode {
            Some(m) => m.label(),
            None => format!("{} lines {:.2} Hz", self.periods.lines, self.frame_hz),
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct Picture {
    pub width: usize,
    pub height: usize,
    pub gray: Vec<u8>,
    pub aspect: f32,
    pub sequence: u64,
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct Read {
    pub picture: Option<Picture>,
    pub locked: bool,
    pub released: bool,
}

/// Samples kept for the carrier search, and so how fine its bins are.
///
/// Half a million at 20 MS/s is 26 ms and a bin of 38 Hz, which a parabola
/// through the peak turns into single figures. That is what the phase loop
/// needs to start from: it corrects less than half a turn a frame, and a
/// frame at 60 Hz is 60 Hz of pull-in.
const CARRIER_SAMPLES: usize = 1 << 19;

/// How far from where the mode says its harmonic should be the carrier is
/// looked for, as a fraction of that frequency.
///
/// A graphics card's clock is out by tens of parts per million, so a
/// hundred is generous; what the window really guards against is the rest
/// of the band, which is full of teeth belonging to other things. The
/// window is around the harmonic and not around the dial, because an
/// operator tunes a round number: 595 MHz for a fourth harmonic that sits
/// at 594.0071.
const CARRIER_TOLERANCE: f64 = 1e-4;

/// The narrowest that window is allowed to get, so a low harmonic of a slow
/// clock still has somewhere to be found.
const CARRIER_FLOOR_HZ: f64 = 20e3;

/// How many teeth either side of the harmonic have to be there before it is
/// taken for a screen's clock.
const COMB_TEETH: usize = 3;

/// How far the weakest of those teeth has to stand over the median bin.
///
/// Measured at 595 MHz on the fourth harmonic of a 1920x1080 screen: 42 dB
/// with the screen on and -3 dB on the same band three minutes later with
/// its output switched off. The strongest line alone does not separate
/// those two at all, standing 49 dB up with the screen on and 40 dB with it
/// off.
const MIN_COMB_DB: f32 = 15.0;

/// How much of the turn measured between a frame and the average is taken
/// out of the mixing frequency.
const PHASE_GAIN: f64 = 0.5;

/// Samples between putting the mixing phasor back on the unit circle.
const RENORMALISE: usize = 4096;

/// What a lock found from the carrier alone reports as its score, which is
/// not the same measurement as a lock found in the envelope.
const LOCK_SCORE_FROM_CARRIER: f32 = f32::INFINITY;

pub struct Reader {
    rate_hz: f64,
    dial_hz: f64,
    decimation: usize,
    env: Vec<f32>,
    cells: Vec<C32>,
    iq: Vec<C32>,
    iq_at: usize,
    iq_full: bool,
    mix_hz: f64,
    mix_phase: f64,
    coherent: bool,
    search: Vec<f32>,
    partial: f32,
    partial_n: usize,
    raster: Option<Raster>,
    lock: Option<Locked>,
    forced: Option<&'static Mode>,
    depth: f64,
    nudge: (isize, isize),
    align: bool,
    backoff: usize,
    since_check: usize,
    published: u64,
    sequence: u64,
}

impl Reader {
    pub fn new(rate_hz: f64) -> Self {
        let decimation = (rate_hz / SEARCH_RATE_HZ).round().max(1.0) as usize;
        Self {
            rate_hz,
            dial_hz: 0.0,
            cells: Vec::new(),
            iq: Vec::new(),
            iq_at: 0,
            iq_full: false,
            mix_hz: 0.0,
            mix_phase: 0.0,
            coherent: false,
            decimation,
            env: Vec::new(),
            search: Vec::new(),
            partial: 0.0,
            partial_n: 0,
            raster: None,
            lock: None,
            forced: None,
            depth: 8.0,
            nudge: (0, 0),
            align: true,
            backoff: 0,
            since_check: 0,
            published: 0,
            sequence: 0,
        }
    }

    pub fn search_rate_hz(&self) -> f64 {
        self.rate_hz / self.decimation as f64
    }

    pub fn locked(&self) -> Option<&Locked> {
        self.lock.as_ref()
    }

    pub fn frames(&self) -> u64 {
        self.raster.as_ref().map_or(0, Raster::frames)
    }

    /// Whether the harmonic was found and frames are being averaged as
    /// complex numbers rather than as magnitudes.
    pub fn coherent(&self) -> bool {
        self.coherent
    }

    pub fn held(&self) -> (u64, u64) {
        self.raster.as_ref().map_or((0, 0), |r| (r.matched(), r.judged()))
    }

    pub fn drift_ppm(&self) -> f64 {
        self.raster.as_ref().map_or(0.0, |r| r.drift() / r.period() * 1e6)
    }

    pub fn force(&mut self, mode: Option<&'static Mode>) {
        if mode != self.forced {
            self.forced = mode;
            self.drop_lock();
        }
    }

    pub fn set_depth(&mut self, depth: f64) {
        self.depth = depth.max(1.0);
        if let Some(r) = self.raster.as_mut() {
            r.set_depth(self.depth);
        }
    }

    pub fn set_nudge(&mut self, x: isize, y: isize) {
        self.nudge = (x, y);
    }

    pub fn set_align(&mut self, align: bool) {
        self.align = align;
    }

    pub fn reset(&mut self) {
        self.drop_lock();
        self.search.clear();
        self.backoff = 0;
        self.partial = 0.0;
        self.partial_n = 0;
    }

    fn drop_lock(&mut self) {
        self.raster = None;
        self.lock = None;
        self.published = 0;
    }

    /// Where the receiver is tuned, which is what makes a carrier near the
    /// middle of the span a pixel clock harmonic rather than an offset.
    pub fn set_dial(&mut self, hz: f64) {
        self.dial_hz = hz;
    }

    pub fn push(&mut self, iq: &[C32]) -> Read {
        self.env.clear();
        raster::envelope(iq, &mut self.env);
        self.keep(iq);
        let env = std::mem::take(&mut self.env);
        let mut out = self.feed(&env);
        self.env = env;
        self.paint(iq);
        if let Some(r) = self.raster.as_ref()
            && r.frames() >= self.published + PUBLISH_FRAMES
        {
            self.published = r.frames();
            self.sequence += 1;
            out.picture = Some(self.picture());
        }
        out
    }

    /// Keep the last samples for the carrier search, written round a ring
    /// rather than shifted down one block at a time: the shift moved three
    /// megabytes per block for samples nothing reads until a lock is tried.
    fn keep(&mut self, iq: &[C32]) {
        if self.iq.len() < CARRIER_SAMPLES {
            self.iq.resize(CARRIER_SAMPLES, C32::new(0.0, 0.0));
            self.iq_at = 0;
            self.iq_full = false;
        }
        for s in iq.iter().rev().take(CARRIER_SAMPLES).rev() {
            self.iq[self.iq_at] = *s;
            self.iq_at += 1;
            if self.iq_at == CARRIER_SAMPLES {
                self.iq_at = 0;
                self.iq_full = true;
            }
        }
    }

    /// Those samples in the order they arrived, which is what a transform
    /// of them needs.
    fn in_order(&self) -> Vec<C32> {
        if !self.iq_full {
            return self.iq[..self.iq_at].to_vec();
        }
        let mut out = Vec::with_capacity(CARRIER_SAMPLES);
        out.extend_from_slice(&self.iq[self.iq_at..]);
        out.extend_from_slice(&self.iq[..self.iq_at]);
        out
    }

    /// Paint the block, with the pixel clock harmonic mixed to nothing where
    /// one was found, and as bare magnitudes where none was.
    fn paint(&mut self, iq: &[C32]) {
        let Some(r) = self.raster.as_mut() else { return };
        self.cells.clear();
        match self.coherent {
            true => {
                // One turn of the phasor per sample rather than a sine and
                // a cosine each: at 20 MS/s that was forty million
                // transcendental calls a second, and the whole of the
                // difference between reading a span twice over and reading
                // it three times over.
                let step = std::f64::consts::TAU * -self.mix_hz / self.rate_hz;
                let by = C32::new(step.cos() as f32, step.sin() as f32);
                let mut turn = C32::new(self.mix_phase.cos() as f32, self.mix_phase.sin() as f32);
                for (n, s) in iq.iter().enumerate() {
                    self.cells.push(*s * turn);
                    turn *= by;
                    // A phasor multiplied a million times over drifts off
                    // the unit circle, and the picture with it.
                    if n % RENORMALISE == 0 {
                        turn /= turn.norm().max(f32::MIN_POSITIVE);
                    }
                }
                self.mix_phase =
                    (self.mix_phase + step * iq.len() as f64).rem_euclid(std::f64::consts::TAU);
            }
            false => self.cells.extend(iq.iter().map(|s| C32::new(s.norm(), 0.0))),
        }
        let frames = r.push(&self.cells);
        if frames > 0 && self.coherent {
            // What is left of the harmonic after the mix, measured as the
            // angle a frame turned against the average, taken back out of
            // the mixing frequency.
            let seconds = r.period() / self.rate_hz;
            let turned = r.turned();
            self.mix_hz += PHASE_GAIN * turned / (std::f64::consts::TAU * seconds);
        }
    }

    fn feed(&mut self, env: &[f32]) -> Read {
        for v in env {
            self.partial += *v;
            self.partial_n += 1;
            if self.partial_n == self.decimation {
                self.search.push(self.partial / self.decimation as f32);
                self.partial = 0.0;
                self.partial_n = 0;
            }
        }
        let window = self.window();
        if self.search.len() > window {
            let over = self.search.len() - window;
            self.search.drain(..over);
        }
        self.since_check += env.len();
        if self.backoff > 0 {
            self.backoff = self.backoff.saturating_sub(env.len());
            return Read::default();
        }
        match self.lock.is_some() {
            true => self.recheck(),
            false => self.look(),
        }
    }

    fn window(&self) -> usize {
        (WINDOW_FRAMES * self.search_rate_hz() / self.limits().frame_hz.0) as usize
    }

    fn limits(&self) -> Limits {
        Limits { line_hz: line_hz_limits(), ..Limits::default() }
    }

    fn measure(&self) -> Option<Locked> {
        let rate = self.search_rate_hz();
        let found = self.forced.and_then(|m| self.carrier(m));
        let periods = match (self.forced, found) {
            // A named mode and a carrier are the whole measurement: the
            // clock is the dial plus the offset, the line count is the
            // mode's. Nothing has to repeat well enough to be found in the
            // envelope, which is what lets this lock on a screen the search
            // cannot see at all.
            (Some(m), Some((clock, _))) => Periods {
                frame_samples: rate * (m.total_width * m.total_height) as f64 / clock,
                lines: m.total_height,
                score: LOCK_SCORE_FROM_CARRIER,
            },
            (Some(m), None) => {
                let (line, score) = raster::find_line(&self.search, rate, self.limits())?;
                Periods {
                    frame_samples: line * m.total_height as f64,
                    lines: m.total_height,
                    score,
                }
            }
            (None, _) => raster::find_periods(&self.search, rate, self.limits())?,
        };
        let frame_hz = periods.frame_hz(rate);
        let mode = match self.forced {
            Some(m) => Some(m),
            None => display::match_mode(frame_hz, periods.lines),
        };
        Some(Locked {
            periods,
            frame_hz,
            line_hz: periods.line_hz(rate),
            mode,
            carrier_hz: found.map(|(_, offset)| offset),
        })
    }

    /// The pixel clock of a named mode and where its harmonic sits against
    /// the dial, from the comb the cable radiates.
    ///
    /// One transform, kept: the search is a quarter of a million points and
    /// a lock used to ask for it three times over, once to decide, once to
    /// measure and once to mix.
    fn carrier(&self, m: &Mode) -> Option<(f64, f64)> {
        if self.dial_hz <= 0.0 {
            return None;
        }
        let harmonic = (self.dial_hz / m.pixel_clock_hz as f64).round();
        if harmonic < 1.0 {
            return None;
        }
        let found = raster::find_comb(
            &self.in_order(),
            self.rate_hz,
            self.carrier_window(m, harmonic),
            m.line_hz(),
            COMB_TEETH,
        )?;
        (found.over_db >= MIN_COMB_DB)
            .then(|| ((self.dial_hz + found.offset_hz) / harmonic, found.offset_hz))
    }

    /// Where in the span this mode's harmonic should be, as an offset from
    /// the dial, with room either side for a clock that is not exactly what
    /// the standard says.
    fn carrier_window(&self, m: &Mode, harmonic: f64) -> (f64, f64) {
        let nominal = harmonic * m.pixel_clock_hz as f64;
        let reach = (nominal * CARRIER_TOLERANCE).max(CARRIER_FLOOR_HZ);
        let middle = nominal - self.dial_hz;
        (middle - reach, middle + reach)
    }

    fn look(&mut self) -> Read {
        if self.search.len() < self.window() {
            return Read::default();
        }
        self.since_check = 0;
        let Some(lock) = self.measure() else {
            self.backoff = (BACKOFF_S * self.rate_hz) as usize;
            return Read::default();
        };
        self.start(lock);
        Read { locked: true, ..Read::default() }
    }

    fn recheck(&mut self) -> Read {
        if (self.since_check as f64) < RECHECK_S * self.rate_hz {
            return Read::default();
        }
        self.since_check = 0;
        if !self.frames_hold_still() {
            self.drop_lock();
            self.backoff = (BACKOFF_S * self.rate_hz) as usize;
            return Read { released: true, ..Read::default() };
        }
        let was = self.lock;
        match self.measure() {
            None => {
                self.drop_lock();
                self.backoff = (BACKOFF_S * self.rate_hz) as usize;
                Read { released: true, ..Read::default() }
            }
            Some(now) => {
                if was.map(|l| l.periods.lines) != Some(now.periods.lines) {
                    self.start(now);
                }
                Read::default()
            }
        }
    }

    fn frames_hold_still(&self) -> bool {
        let Some(r) = self.raster.as_ref() else { return false };
        match r.judged() {
            0 => true,
            judged => r.matched() as f64 / judged as f64 >= MIN_MATCHED,
        }
    }

    fn start(&mut self, lock: Locked) {
        // Mix the harmonic to nothing so a frame lands at the same phase as
        // the one before it, which is what makes averaging them as complex
        // numbers mean anything.
        self.coherent = false;
        self.mix_phase = 0.0;
        if let Some(offset) = lock.carrier_hz {
            self.mix_hz = offset;
            self.coherent = true;
        }
        let period = lock.periods.frame_samples * self.decimation as f64;
        let line = period / lock.periods.lines as f64;
        let width = (line.round() as usize)
            .clamp(16, lock.mode.map_or(MAX_WIDTH, |m| m.total_width.min(MAX_WIDTH)));
        self.raster = Some(Raster::new(width, lock.periods.lines, period, self.depth));
        self.lock = Some(lock);
        self.published = 0;
    }

    fn picture(&self) -> Picture {
        let r = self.raster.as_ref().expect("a raster to draw from");
        let aspect = match self.lock.and_then(|l| l.mode) {
            Some(m) => m.total_width as f32 / m.total_height as f32,
            None => r.width() as f32 / r.height() as f32,
        };
        Picture {
            width: r.width(),
            height: r.height(),
            gray: r.picture(self.nudge, self.align),
            aspect,
            sequence: self.sequence,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn leak(mode: &Mode, rate: f64, seconds: f64, noise: f32) -> Vec<C32> {
        leak_flat(mode, rate, seconds, noise)
    }

    /// The same screen as a receiver really meets it: the picture riding a
    /// tooth of the cable's comb, `offset_hz` from the dial, with the noise
    /// added afterwards by the receiver rather than multiplied onto the
    /// carrier. Where the noise goes is the whole point of the comparison:
    /// noise that rides the carrier survives a magnitude detector as well as
    /// it survives a coherent one.
    fn carried(mode: &Mode, rate: f64, seconds: f64, noise: f32, offset_hz: f64) -> Vec<C32> {
        let mut state = 0x5DEECE66Du64;
        let mut rand = move || {
            state ^= state << 13;
            state ^= state >> 7;
            state ^= state << 17;
            (state >> 40) as f32 / 16_777_216.0 - 0.5
        };
        leak_flat(mode, rate, seconds, 0.0)
            .iter()
            .enumerate()
            .map(|(i, a)| {
                let phase = std::f64::consts::TAU * offset_hz * i as f64 / rate;
                let turn = C32::new(phase.cos() as f32, phase.sin() as f32);
                turn * a.re + C32::new(rand() * noise, rand() * noise)
            })
            .collect()
    }

    fn leak_flat(mode: &Mode, rate: f64, seconds: f64, noise: f32) -> Vec<C32> {
        let mut state = 0xDEADBEEF12345678u64;
        let mut rand = move || {
            state ^= state << 13;
            state ^= state >> 7;
            state ^= state << 17;
            (state >> 40) as f32 / 8_388_608.0 - 0.5
        };
        let n = (rate * seconds) as usize;
        (0..n)
            .map(|i| {
                let pixel = i as f64 * mode.pixel_clock_hz as f64 / rate;
                let x = pixel as usize % mode.total_width;
                let y = (pixel / mode.total_width as f64) as usize % mode.total_height;
                let lit = match (x < mode.width, y < mode.height) {
                    (true, true) => {
                        let (u, v) = (x as f64 / mode.width as f64, y as f64 / mode.height as f64);
                        0.35 + 0.4 * ((u > 0.05 && u < 0.45 && v > 0.1 && v < 0.6) as u8 as f32)
                            + 0.6 * ((u > 0.5 && u < 0.95 && v > 0.3 && v < 0.9) as u8 as f32)
                    }
                    _ => 0.0,
                };
                let a = lit + rand() * noise;
                C32::new(a, 0.0)
            })
            .collect()
    }

    fn run(reader: &mut Reader, iq: &[C32]) -> (Vec<Picture>, usize, usize) {
        let (mut pictures, mut locks, mut releases) = (Vec::new(), 0, 0);
        for block in iq.chunks(65_536) {
            let read = reader.push(block);
            locks += read.locked as usize;
            releases += read.released as usize;
            pictures.extend(read.picture);
        }
        (pictures, locks, releases)
    }

    #[test]
    fn a_screen_is_measured_named_and_drawn() {
        let mode = display::by_label("1024x768 60 Hz").expect("the mode");
        let rate = 12e6;
        let iq = leak(mode, rate, 0.55, 0.5);
        let mut reader = Reader::new(rate);
        let (pictures, locks, releases) = run(&mut reader, &iq);
        assert_eq!(locks, 1, "locks in 0.55 s");
        assert_eq!(releases, 0);
        let lock = reader.locked().expect("a lock");
        assert_eq!(lock.periods.lines, 806);
        assert_eq!(lock.mode.map(Mode::label).as_deref(), Some("1024x768 60 Hz"));
        assert!((lock.frame_hz - 60.0038).abs() < 0.05, "{} Hz", lock.frame_hz);
        assert!((lock.line_hz - 48_363.0).abs() < 40.0, "{} lines a second", lock.line_hz);
        assert_eq!(reader.frames(), 28, "frames painted in the 0.48 s after the lock");
        assert_eq!(pictures.len(), 2, "one picture every twelve frames");
        assert!(reader.drift_ppm().abs() < 1.0, "still drifting {} ppm", reader.drift_ppm());
        let (matched, judged) = reader.held();
        assert_eq!((matched, judged), (26, 26), "frames found where the period says they are");
        let p = pictures.last().expect("a picture");
        assert_eq!((p.width, p.height), (248, 806));
        assert_eq!(p.gray.len(), 248 * 806);
        assert!((p.aspect - 1344.0 / 806.0).abs() < 0.01, "drawn at {}", p.aspect);
    }

    /// A span-wide decoder is handed every sample for as long as the
    /// receiver runs, so it has to read faster than the air arrives, with or
    /// without a screen in the span. Measured on a 20 MS/s span: 5.4 times
    /// real time while locked to a 1280x1024 screen, 9.2 times on an empty
    /// band, where a failed look costs one correlation rather than two.
    #[test]
    fn a_span_is_read_faster_than_it_arrives() {
        if cfg!(debug_assertions) {
            return;
        }
        let rate = 20e6;
        let seconds = 2.0;
        let mode = display::by_label("1280x1024 60 Hz").expect("the mode");
        let screen = leak(mode, rate, seconds, 0.5);
        let mut reader = Reader::new(rate);
        let t = std::time::Instant::now();
        for b in screen.chunks(131_072) {
            reader.push(b);
        }
        let locked = seconds / t.elapsed().as_secs_f64();
        assert!(reader.locked().is_some(), "the benchmark never locked");
        assert!(locked > 2.0, "a locked screen reads at {locked:.2} times real time");

        let mut state = 0xABCD_1234_5678_9876u64;
        let mut rand = move || {
            state ^= state << 13;
            state ^= state >> 7;
            state ^= state << 17;
            (state >> 40) as f32 / 8_388_608.0 - 0.5
        };
        let mut block = vec![C32::new(0.0, 0.0); 131_072];
        let mut empty = Reader::new(rate);
        let t = std::time::Instant::now();
        for _ in 0..((seconds * rate / block.len() as f64) as usize) {
            block.iter_mut().for_each(|s| *s = C32::new(rand(), rand()));
            empty.push(&block);
        }
        let idle = seconds / t.elapsed().as_secs_f64();
        assert!(idle > 3.0, "an empty band reads at {idle:.2} times real time");
    }

    #[test]
    fn the_line_rates_allowed_are_the_ones_displays_run() {
        let (lo, hi) = line_hz_limits();
        assert!(
            (lo - 31_468.75 * 0.8).abs() < 100.0,
            "{lo} is not 640x480's line rate less a fifth"
        );
        assert!(
            (hi - 182_879.6 * 1.2).abs() < 200.0,
            "{hi} is not the 49 inch ultrawide's line rate and a fifth"
        );
        for m in display::modes() {
            assert!((lo..=hi).contains(&m.line_hz()), "{} is outside the range", m.label());
        }
        // 191.704 kHz was an off-air false lock, and it now sits inside the
        // range, because a 5120x1440 screen at 120 Hz runs 182.9 kHz a line
        // and the ceiling has to clear it. What refuses that lock is the
        // clarity of its frame, not its rate.
        assert!((lo..=hi).contains(&191_704.0));
        assert!(!(lo..=hi).contains(&300_000.0), "nothing runs 300 kHz a line");
    }

    /// A named mode and the carrier are enough to read a screen the
    /// envelope search cannot find at all.
    ///
    /// The comb's tooth is a line in the spectrum, so an FFT of half a
    /// million samples lifts it tens of dB over the floor, while the frame
    /// and line periods have to be found in the envelope, where the same
    /// signal is buried. Measured on a synthesised 1920x1080 screen: under
    /// noise six times the picture's own amplitude the envelope search
    /// still finds the raster, and at eight times it finds nothing at all
    /// while the carrier is still there to be read.
    #[test]
    fn the_carrier_reads_a_screen_the_search_cannot_find() {
        let mode = display::by_label("1920x1080 60 Hz").expect("the mode");
        let rate = 20e6;
        let dial = mode.pixel_clock_hz as f64 * 10.0;
        let iq = carried(mode, rate, 0.7, 8.0, 17_700.0);

        let mut lost = Reader::new(rate);
        lost.force(Some(mode));
        for b in iq.chunks(131_072) {
            lost.push(b);
        }
        assert!(lost.locked().is_none(), "the envelope search found something after all");

        let mut told = Reader::new(rate);
        told.set_dial(dial);
        told.force(Some(mode));
        for b in iq.chunks(131_072) {
            told.push(b);
        }
        let lock = told.locked().expect("the carrier was not enough");
        assert_eq!(lock.periods.lines, 1125);
        assert!((lock.frame_hz - 60.0).abs() < 0.01, "{} Hz", lock.frame_hz);
        assert!(told.coherent(), "found the carrier and did not use it");
    }

    /// The picture of a screen averaged as complex numbers against the same
    /// screen averaged as magnitudes.
    ///
    /// Mixing the comb's tooth to nothing makes every frame land at the
    /// same phase, so the frames add rather than their magnitudes. Off air
    /// on a 1920x1080 panel at 1485 MHz this is the difference between a
    /// dark rectangle with a bright border and a picture in which the four
    /// camera panels, the divider between them and the row of thumbnails
    /// can all be made out.
    #[test]
    fn a_screen_averaged_coherently_carries_more_of_its_picture() {
        let mode = display::by_label("1920x1080 60 Hz").expect("the mode");
        let rate = 20e6;
        let dial = mode.pixel_clock_hz as f64 * 10.0;
        let iq = carried(mode, rate, 0.9, 4.0, 17_700.0);

        let read = |dial_hz: f64| -> Option<Picture> {
            let mut r = Reader::new(rate);
            if dial_hz > 0.0 {
                r.set_dial(dial_hz);
            }
            r.force(Some(mode));
            let mut last = None;
            for b in iq.chunks(131_072) {
                if let Some(p) = r.push(b).picture {
                    last = Some(p);
                }
            }
            last
        };

        let coherent = read(dial).expect("a coherent picture");
        // The same samples read by a receiver that does not know where it is
        // tuned, which cannot name the harmonic and so averages magnitudes.
        let plain = read(0.0).expect("a magnitude picture");

        let score =
            |p: &Picture| -> f32 { matched(&p.gray, &source(mode, p.width, p.height), p.width) };
        let (with, without) = (score(&coherent), score(&plain));
        // Measured on this screen: 0.992 against 0.954 under noise twice
        // the picture's amplitude, 0.977 against 0.897 at four times.
        assert!(
            with > without + 0.05,
            "coherent correlates {with:.3} with the screen, magnitude {without:.3}"
        );
    }

    /// The screen the synthesiser drew, at the shape the raster paints.
    fn source(mode: &Mode, width: usize, height: usize) -> Vec<u8> {
        (0..height)
            .flat_map(|y| {
                (0..width).map(move |x| {
                    let px = x as f64 / width as f64 * mode.total_width as f64;
                    let py = y as f64 / height as f64 * mode.total_height as f64;
                    let lit = match (px < mode.width as f64, py < mode.height as f64) {
                        (true, true) => {
                            let (u, v) = (px / mode.width as f64, py / mode.height as f64);
                            0.35 + 0.4 * ((u > 0.05 && u < 0.45 && v > 0.1 && v < 0.6) as u8 as f32)
                                + 0.6 * ((u > 0.5 && u < 0.95 && v > 0.3 && v < 0.9) as u8 as f32)
                        }
                        _ => 0.0,
                    };
                    (lit * 255.0) as u8
                })
            })
            .collect()
    }

    /// How well a picture matches the screen, at the best alignment: what
    /// the raster paints is the whole frame, and where its first line falls
    /// is a property of when the capture started rather than of the screen.
    fn matched(got: &[u8], want: &[u8], width: usize) -> f32 {
        let height = got.len() / width;
        let (sx, sy) = (4usize, 8usize);
        let (w, h) = (width / sx, height / sy);
        let shrink = |v: &[u8]| -> Vec<f32> {
            (0..h)
                .flat_map(|y| {
                    (0..w).map(move |x| {
                        let mut sum = 0.0;
                        for dy in 0..sy {
                            for dx in 0..sx {
                                sum += v[(y * sy + dy) * width + x * sx + dx] as f32;
                            }
                        }
                        sum / (sx * sy) as f32
                    })
                })
                .collect()
        };
        let (g, t) = (shrink(got), shrink(want));
        let mean = |v: &[f32]| v.iter().sum::<f32>() / v.len() as f32;
        let (mg, mt) = (mean(&g), mean(&t));
        let mut best = -1.0f32;
        for dy in 0..h {
            for dx in 0..w {
                let (mut dot, mut pg, mut pt) = (0.0f32, 0.0f32, 0.0f32);
                for y in 0..h {
                    for x in 0..w {
                        let a = g[y * w + x] - mg;
                        let b = t[((y + dy) % h) * w + (x + dx) % w] - mt;
                        dot += a * b;
                        pg += a * a;
                        pt += b * b;
                    }
                }
                best = best.max(dot / (pg.sqrt() * pt.sqrt()).max(f32::MIN_POSITIVE));
            }
        }
        best
    }

    #[test]
    fn minutes_of_noise_are_never_a_screen() {
        let rate = 4e6;
        let mut state = 0x1234_5678_9ABC_DEF0u64;
        let mut rand = move || {
            state ^= state << 13;
            state ^= state >> 7;
            state ^= state << 17;
            (state >> 40) as f32 / 8_388_608.0 - 0.5
        };
        let mut reader = Reader::new(rate);
        let (mut pictures, mut locks) = (0, 0);
        let mut block = vec![C32::new(0.0, 0.0); 65_536];
        for _ in 0..(180.0 * rate / block.len() as f64) as usize {
            block.iter_mut().for_each(|s| *s = C32::new(rand(), rand()));
            let read = reader.push(&block);
            pictures += read.picture.is_some() as usize;
            locks += read.locked as usize;
        }
        assert_eq!((pictures, locks), (0, 0), "pictures and locks off three minutes of noise");
        assert_eq!(reader.frames(), 0);
    }

    #[test]
    fn a_named_mode_is_read_as_that_mode() {
        let mode = display::by_label("640x480 60 Hz").expect("the mode");
        let rate = 8e6;
        let iq = leak(mode, rate, 0.4, 0.4);
        let mut reader = Reader::new(rate);
        reader.force(Some(mode));
        let (pictures, locks, _) = run(&mut reader, &iq);
        assert_eq!(locks, 1);
        let lock = reader.locked().expect("a lock");
        assert_eq!(lock.periods.lines, 525);
        assert_eq!(lock.mode.map(Mode::label).as_deref(), Some("640x480 60 Hz"));
        assert!((lock.frame_hz - 59.94).abs() < 0.05, "{} Hz", lock.frame_hz);
        assert_eq!(reader.frames(), 20, "frames painted in the 0.33 s after the lock");
        assert_eq!(pictures.len(), 1, "one picture every twelve frames");
        assert_eq!((pictures[0].width, pictures[0].height), (254, 525));
    }

    #[test]
    fn a_screen_that_stops_is_given_up() {
        let mode = display::by_label("640x480 60 Hz").expect("the mode");
        let rate = 8e6;
        let mut reader = Reader::new(rate);
        let (_, locks, _) = run(&mut reader, &leak(mode, rate, 0.4, 0.4));
        assert_eq!(locks, 1);
        let mut state = 0xFEED_FACE_CAFE_BEEFu64;
        let quiet: Vec<C32> = (0..(rate * (RECHECK_S + 0.5)) as usize)
            .map(|_| {
                state ^= state << 13;
                state ^= state >> 7;
                state ^= state << 17;
                C32::new((state >> 40) as f32 / 8_388_608.0 - 0.5, 0.0)
            })
            .collect();
        let (_, _, releases) = run(&mut reader, &quiet);
        assert_eq!(releases, 1, "the span was never given back");
        assert!(reader.locked().is_none());
    }
}
