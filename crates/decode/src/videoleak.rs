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

pub struct Reader {
    rate_hz: f64,
    decimation: usize,
    env: Vec<f32>,
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

    pub fn push(&mut self, iq: &[C32]) -> Read {
        self.env.clear();
        raster::envelope(iq, &mut self.env);
        let env = std::mem::take(&mut self.env);
        let mut out = self.feed(&env);
        self.env = env;
        if let Some(r) = self.raster.as_mut() {
            r.push(&self.env);
            if r.frames() >= self.published + PUBLISH_FRAMES {
                self.published = r.frames();
                self.sequence += 1;
                out.picture = Some(self.picture());
            }
        }
        out
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
        let periods = match self.forced {
            // The operator named the raster, so the line period is the whole
            // measurement: a screen too weak to show its frame is exactly
            // the one somebody names the mode of.
            Some(m) => {
                let (line, score) = raster::find_line(&self.search, rate, self.limits())?;
                Periods {
                    frame_samples: line * m.total_height as f64,
                    lines: m.total_height,
                    score,
                }
            }
            None => raster::find_periods(&self.search, rate, self.limits())?,
        };
        let frame_hz = periods.frame_hz(rate);
        let mode = match self.forced {
            Some(m) => Some(m),
            None => display::match_mode(frame_hz, periods.lines),
        };
        Some(Locked { periods, frame_hz, line_hz: periods.line_hz(rate), mode })
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
