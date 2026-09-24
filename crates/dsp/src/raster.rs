use common::C32;
use rustfft::FftPlanner;

pub fn envelope(iq: &[C32], out: &mut Vec<f32>) {
    out.extend(iq.iter().map(|s| s.norm()));
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Periods {
    pub frame_samples: f64,
    pub lines: usize,
    pub score: f32,
}

impl Periods {
    pub fn frame_hz(&self, rate: f64) -> f64 {
        rate / self.frame_samples
    }

    pub fn line_samples(&self) -> f64 {
        self.frame_samples / self.lines as f64
    }

    pub fn line_hz(&self, rate: f64) -> f64 {
        rate / self.line_samples()
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Limits {
    pub frame_hz: (f64, f64),
    pub lines: (usize, usize),
    pub line_hz: (f64, f64),
}

impl Default for Limits {
    fn default() -> Self {
        Self { frame_hz: (49.0, 145.0), lines: (400, 2400), line_hz: (20e3, 250e3) }
    }
}

/// Noise alone peaks at 6.0 over the 49 to 87 Hz range (262144 samples);
/// the screens in the tests score 554 to 835.
pub const LOCK_SCORE: f32 = 10.0;

/// Measured over the whole field of line counts the refresh range allows,
/// about 600 of them: a synthesised 1920x1080 screen's own count stands
/// 18.1 sigma over that field, an off-air source at 595 MHz with a
/// 67.501 kHz line rate and no frame in it reached 2.9, and the largest of
/// 600 draws of noise is about 3.2 by itself.
pub const MIN_CLARITY: f32 = 8.0;

fn line_lags(r: &[f32], limits: Limits, rate: f64) -> (usize, usize) {
    let lo = (rate / limits.line_hz.1) as usize;
    let hi = ((rate / limits.line_hz.0) as usize).min(r.len().saturating_sub(2));
    (lo, hi)
}

/// Bins either side of nothing that a carrier is not looked for in.
const SPUR_BINS: usize = 3;

/// A line of the comb a display's cable radiates: where it sits against the
/// dial, and how far it stands over the rest of the band.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Carrier {
    pub offset_hz: f64,
    pub over_db: f32,
}

/// The pixel clock harmonic nearest the dial, found as a line in the
/// spectrum rather than as a period in the samples.
///
/// A cable clocked at a fixed rate radiates a comb, and the tooth the
/// receiver is parked on is a carrier: measured on a 1920x1080 screen at
/// 1485 MHz it stands 60 dB over the median bin, with its neighbours
/// 67.501 kHz either side, one line rate apart. Reading it as a frequency
/// rather than as a lag is what makes it precise: the dial is exact, so the
/// clock is the dial plus a small offset, and a bin of 9.5 Hz on the tenth
/// harmonic pins the pixel clock to a hertz.
/// The pixel clock harmonic, found as the tooth of a comb rather than as a
/// peak on its own.
///
/// A carrier by itself says nothing: measured on a band with the screen
/// switched off, the strongest line in the window a 1920x1080 mode puts its
/// fourth harmonic in still stood 40 dB over the median, against 49 dB with
/// the screen on. What a scanned raster has and a lone transmitter does not
/// is neighbours: teeth one line rate apart, either side, as far out as
/// anybody cares to look. This asks for `teeth` of them each way and
/// reports the weakest, so a tooth with nothing beside it scores nothing.
pub fn find_comb(
    iq: &[C32],
    rate: f64,
    window_hz: (f64, f64),
    spacing_hz: f64,
    teeth: usize,
) -> Option<Carrier> {
    let (power, n) = spectrum(iq)?;
    let bin_hz = rate / n as f64;
    let median = median_of(&power);
    let step = spacing_hz / bin_hz;
    let wrap = |b: f64| -> usize { (b.round() as i64).rem_euclid(n as i64) as usize };
    let weakest = |middle: i64| -> f32 {
        (0..=teeth as i64)
            .flat_map(|t| [middle as f64 + t as f64 * step, middle as f64 - t as f64 * step])
            .map(|b| {
                let at = wrap(b);
                power[at].max(power[wrap(b - 1.0)]).max(power[wrap(b + 1.0)])
            })
            .fold(f32::MAX, f32::min)
    };
    let bin_of = |hz: f64| -> i64 { (hz / bin_hz).round() as i64 };
    let (lo, hi) = (bin_of(window_hz.0), bin_of(window_hz.1));
    // The strongest line in the window, not the best comb: every tooth of a
    // comb scores alike, so choosing among them by their neighbours picks
    // whichever noise favoured and mixes the picture to the wrong place. The
    // pixel clock's own line stands over its sidebands, and the comb is what
    // says the line is a raster's rather than somebody's transmitter.
    let at = (lo..=hi)
        .filter(|b| b.unsigned_abs() as usize >= SPUR_BINS)
        .max_by(|a, b| power[wrap(*a as f64)].total_cmp(&power[wrap(*b as f64)]))?;
    let middle = wrap(at as f64);
    if middle == 0 || middle + 1 >= n {
        return None;
    }
    let refined = interpolate(&power, middle);
    let offset = match refined < n as f64 / 2.0 {
        true => refined,
        false => refined - n as f64,
    } * bin_hz;
    Some(Carrier { offset_hz: offset, over_db: 10.0 * (weakest(at) / median).log10() })
}

fn spectrum(iq: &[C32]) -> Option<(Vec<f32>, usize)> {
    let n = iq.len().next_power_of_two() / 2;
    if n < 4096 {
        return None;
    }
    let mut buf: Vec<C32> = iq[..n].to_vec();
    FftPlanner::new().plan_fft_forward(n).process(&mut buf);
    Some((buf.iter().map(|v| v.norm_sqr()).collect(), n))
}

fn median_of(power: &[f32]) -> f32 {
    let mut sorted = power.to_vec();
    sorted.sort_by(f32::total_cmp);
    sorted[sorted.len() / 2].max(f32::MIN_POSITIVE)
}

pub fn find_carrier(iq: &[C32], rate: f64, window_hz: (f64, f64)) -> Option<Carrier> {
    let n = iq.len().next_power_of_two() / 2;
    if n < 4096 {
        return None;
    }
    let mut buf: Vec<C32> = iq[..n].to_vec();
    FftPlanner::new().plan_fft_forward(n).process(&mut buf);
    let power: Vec<f32> = buf.iter().map(|v| v.norm_sqr()).collect();
    let bin_hz = rate / n as f64;
    let mut sorted = power.clone();
    sorted.sort_by(f32::total_cmp);
    let median = sorted[sorted.len() / 2].max(f32::MIN_POSITIVE);
    let bin_of = |hz: f64| -> i64 { (hz / bin_hz).round() as i64 };
    let (lo, hi) = (bin_of(window_hz.0), bin_of(window_hz.1));
    // Never the middle of the span: a direct conversion receiver puts its
    // own leakage there, so a peak at nothing says more about the radio
    // than about any screen.
    let bins = (lo..=hi)
        .filter(|b| b.unsigned_abs() as usize >= SPUR_BINS)
        .map(|b| b.rem_euclid(n as i64) as usize)
        .filter(|b| *b + 1 < n && *b > 0);
    let at = bins.max_by(|a, b| power[*a].total_cmp(&power[*b]))?;
    if at == 0 || at + 1 >= n {
        return None;
    }
    let refined = interpolate(&power, at);
    let offset = match refined < n as f64 / 2.0 {
        true => refined,
        false => refined - n as f64,
    } * bin_hz;
    Some(Carrier { offset_hz: offset, over_db: 10.0 * (power[at] / median).log10() })
}

pub fn find_line(env: &[f32], rate: f64, limits: Limits) -> Option<(f64, f32)> {
    let r = autocorrelation(env);
    let frame_lo = (rate / limits.frame_hz.1) as usize;
    let frame_hi = ((rate / limits.frame_hz.0) as usize).min(r.len().saturating_sub(2));
    if frame_lo + 2 >= frame_hi {
        return None;
    }
    let (line_lo, line_hi) = line_lags(&r, limits, rate);
    if line_lo + 2 >= line_hi {
        return None;
    }
    let (_, raw) = strongest(&r, frame_lo, frame_hi)?;
    let score = (raw - median(&r[frame_lo..frame_hi])) * (env.len() as f32).sqrt();
    if score < LOCK_SCORE {
        return None;
    }
    let line = refine_line(&r, fundamental(&r, line_lo, line_hi)?, frame_hi);
    let line_hz = rate / line;
    (line_hz >= limits.line_hz.0 && line_hz <= limits.line_hz.1).then_some((line, score))
}

pub fn find_periods(env: &[f32], rate: f64, limits: Limits) -> Option<Periods> {
    let r = autocorrelation(env);
    let frame_lo = (rate / limits.frame_hz.1) as usize;
    let frame_hi = ((rate / limits.frame_hz.0) as usize).min(r.len().saturating_sub(2));
    if frame_lo + 2 >= frame_hi {
        return None;
    }
    let (line_lo, line_hi) = line_lags(&r, limits, rate);
    if line_lo + 2 >= line_hi {
        return None;
    }
    let (_, raw) = strongest(&r, frame_lo, frame_hi)?;
    let score = (raw - median(&r[frame_lo..frame_hi])) * (env.len() as f32).sqrt();
    if score < LOCK_SCORE {
        return None;
    }
    let line = refine_line(&r, fundamental(&r, line_lo, line_hi)?, frame_hi);
    let edges = down_the_screen(env, line);
    let e = autocorrelation(&edges);
    let frame_hi = frame_hi.min(e.len().saturating_sub(2));
    if frame_lo + 2 >= frame_hi {
        return None;
    }
    let (lines, frame, clarity) = whole_frame(&e, line, frame_lo, frame_hi, limits.lines)?;
    let line_hz = rate * lines as f64 / frame;
    if line_hz < limits.line_hz.0 || line_hz > limits.line_hz.1 {
        return None;
    }
    if clarity < MIN_CLARITY {
        return None;
    }
    Some(Periods { frame_samples: frame, lines, score })
}

fn down_the_screen(env: &[f32], line: f64) -> Vec<f32> {
    let whole = line.floor() as usize;
    let frac = (line - whole as f64) as f32;
    if whole + 1 >= env.len() {
        return Vec::new();
    }
    let diff: Vec<f32> = (whole + 1..env.len())
        .map(|i| {
            let before = env[i - whole] * (1.0 - frac) + env[i - whole - 1] * frac;
            env[i] - before
        })
        .collect();
    along_the_line(&diff, (line / 8.0).round().max(1.0) as usize)
}

fn along_the_line(x: &[f32], width: usize) -> Vec<f32> {
    if width <= 1 || x.len() < width {
        return x.to_vec();
    }
    let mut out = Vec::with_capacity(x.len() - width + 1);
    let mut sum: f32 = x[..width].iter().sum();
    out.push(sum / width as f32);
    for i in width..x.len() {
        sum += x[i] - x[i - width];
        out.push(sum / width as f32);
    }
    out
}

fn refine_line(r: &[f32], coarse: f64, frame_hi: usize) -> f64 {
    let mut line = coarse;
    let mut multiple = 1usize;
    while (2 * multiple) as f64 * line < 0.6 * frame_hi as f64 {
        let target = (2 * multiple) as f64 * line;
        let reach = 0.5 * line;
        let lo = (target - reach).max(1.0) as usize;
        let hi = ((target + reach) as usize).min(r.len().saturating_sub(2));
        if lo + 2 >= hi {
            break;
        }
        let Some((at, _)) = strongest(r, lo, hi) else { break };
        multiple *= 2;
        line = at / multiple as f64;
    }
    line
}

fn whole_frame(
    r: &[f32],
    line: f64,
    lo: usize,
    hi: usize,
    lines: (usize, usize),
) -> Option<(usize, f64, f32)> {
    let first = ((lo as f64 / line).ceil() as usize).max(lines.0);
    let last = ((hi as f64 / line).floor() as usize).min(lines.1);
    let at = |count: usize| -> Option<(usize, f32)> {
        let near = (count as f64 * line).round() as usize;
        (near >= 1 && near + 1 < r.len()).then(|| (near, r[near - 1].max(r[near]).max(r[near + 1])))
    };
    let field: Vec<(usize, usize, f32)> =
        (first..=last).filter_map(|c| at(c).map(|(near, peak)| (c, near, peak))).collect();
    if field.len() < 8 {
        return None;
    }
    let n = field.len() as f32;
    let mean = field.iter().map(|f| f.2).sum::<f32>() / n;
    let sd = (field.iter().map(|f| (f.2 - mean) * (f.2 - mean)).sum::<f32>() / n).sqrt();
    let (count, near, peak) = *field.iter().max_by(|a, b| a.2.total_cmp(&b.2))?;
    let clarity = match sd > 0.0 {
        true => (peak - mean) / sd,
        false => 0.0,
    };
    Some((count, interpolate(r, near), clarity))
}

fn strongest(r: &[f32], lo: usize, hi: usize) -> Option<(f64, f32)> {
    let span = &r[lo..hi];
    let peak = span.iter().copied().fold(f32::MIN, f32::max);
    if !peak.is_finite() || peak <= 0.0 {
        return None;
    }
    let at = lo + span.iter().position(|v| *v == peak)?;
    Some((interpolate(r, at.clamp(lo.max(1), hi - 2)), peak))
}

fn fundamental(r: &[f32], lo: usize, hi: usize) -> Option<f64> {
    let span = &r[lo..hi];
    let peak = span.iter().copied().fold(f32::MIN, f32::max);
    if !peak.is_finite() || peak <= 0.0 {
        return None;
    }
    let at =
        (lo + 1..hi - 1).find(|&i| r[i] >= 0.9 * peak && r[i] >= r[i - 1] && r[i] >= r[i + 1])?;
    Some(interpolate(r, at))
}

fn median(span: &[f32]) -> f32 {
    let mut sorted = span.to_vec();
    sorted.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    sorted[sorted.len() / 2]
}

fn interpolate(r: &[f32], at: usize) -> f64 {
    let (a, b, c) = (r[at - 1] as f64, r[at] as f64, r[at + 1] as f64);
    let denom = a - 2.0 * b + c;
    let offset = match denom.abs() > f64::EPSILON {
        true => (0.5 * (a - c) / denom).clamp(-0.5, 0.5),
        false => 0.0,
    };
    at as f64 + offset
}

fn autocorrelation(x: &[f32]) -> Vec<f32> {
    let n = (2 * x.len()).next_power_of_two();
    let mean = x.iter().sum::<f32>() / x.len().max(1) as f32;
    let mut buf: Vec<C32> = Vec::with_capacity(n);
    buf.extend(x.iter().map(|v| C32::new(v - mean, 0.0)));
    buf.resize(n, C32::new(0.0, 0.0));
    let mut planner = FftPlanner::new();
    planner.plan_fft_forward(n).process(&mut buf);
    for v in buf.iter_mut() {
        *v = C32::new(v.norm_sqr(), 0.0);
    }
    planner.plan_fft_inverse(n).process(&mut buf);
    let zero = buf[0].re.max(f32::MIN_POSITIVE);
    let len = x.len();
    (0..len).map(|lag| buf[lag].re / zero * len as f32 / (len - lag).max(1) as f32).collect()
}

pub struct Raster {
    width: usize,
    height: usize,
    period: f64,
    phase: f64,
    nominal: f64,
    sum: Vec<C32>,
    count: Vec<u32>,
    average: Vec<C32>,
    /// The frame before this one, per cell, and which cells it filled.
    last: Vec<C32>,
    held: Vec<bool>,
    /// How far the phase has wandered since the first frame, which the
    /// blending takes back out so the average stays coherent while the
    /// caller's loop settles.
    roll: f64,
    seen: Vec<bool>,
    averaged: u64,
    frames: u64,
    depth: f64,
    drift: f64,
    turned: f64,
    judged: u64,
    matched: u64,
}

/// From a 30 ppm error on a 640x480 desktop: 0.25 settles under 1 ppm in 12
/// frames and jitters 0.06 ppm, 0.5 in 4 and 0.18, 1.0 in 2 and 0.37.
const DRIFT_GAIN: f64 = 0.5;

const SEARCH_LINE_FRACTION: f64 = 0.25;

impl Raster {
    pub fn new(width: usize, height: usize, period: f64, depth: f64) -> Self {
        Self {
            width,
            height,
            period,
            nominal: period,
            phase: 0.0,
            sum: vec![C32::new(0.0, 0.0); width * height],
            count: vec![0; width * height],
            average: vec![C32::new(0.0, 0.0); width * height],
            last: vec![C32::new(0.0, 0.0); width * height],
            held: vec![false; width * height],
            roll: 0.0,
            seen: vec![false; width * height],
            averaged: 0,
            frames: 0,
            depth: depth.max(1.0),
            drift: 0.0,
            turned: 0.0,
            judged: 0,
            matched: 0,
        }
    }

    pub fn width(&self) -> usize {
        self.width
    }

    pub fn height(&self) -> usize {
        self.height
    }

    pub fn period(&self) -> f64 {
        self.period
    }

    pub fn frames(&self) -> u64 {
        self.frames
    }

    pub fn drift(&self) -> f64 {
        self.drift
    }

    /// How far the last frame's phase had turned from the average's, in
    /// radians.
    ///
    /// The caller mixes the pixel clock harmonic to nothing, and this is
    /// what is left of that: a screen and a receiver keep their own
    /// crystals, and a part in a million of a 1485 MHz harmonic is 1485 Hz,
    /// which turns a frame's worth of phase twenty times over. Feeding this
    /// back into the mix is what lets frames be averaged as complex numbers
    /// rather than as magnitudes.
    pub fn turned(&self) -> f64 {
        self.turned
    }

    pub fn judged(&self) -> u64 {
        self.judged
    }

    pub fn matched(&self) -> u64 {
        self.matched
    }

    pub fn set_depth(&mut self, depth: f64) {
        self.depth = depth.max(1.0);
    }

    pub fn push(&mut self, env: &[C32]) -> usize {
        let (w, h) = (self.width as f64, self.height as f64);
        let mut per_sample = h / self.period;
        let mut down = self.phase * per_sample;
        let mut done = 0;
        for v in env {
            let y = (down as usize).min(self.height - 1);
            let x = (((down - y as f64) * w) as usize).min(self.width - 1);
            let at = y * self.width + x;
            self.sum[at] += *v;
            self.count[at] += 1;
            down += per_sample;
            if down >= h {
                self.phase = down / per_sample - self.period;
                self.finish();
                done += 1;
                per_sample = h / self.period;
                down = self.phase * per_sample;
            }
        }
        self.phase = down / per_sample;
        done
    }

    fn finish(&mut self) {
        self.frames += 1;
        if self.frames == 1 {
            self.sum.iter_mut().for_each(|s| *s = C32::new(0.0, 0.0));
            self.count.iter_mut().for_each(|c| *c = 0);
            return;
        }
        let shift = match self.averaged {
            0 => 0.0,
            _ => {
                self.judged += 1;
                self.turned = self.turn_from_last();
                self.roll += self.turned;
                match self.offset_from_average() {
                    Some(s) => {
                        self.matched += 1;
                        s
                    }
                    None => 0.0,
                }
            }
        };
        self.drift = shift;
        if self.averaged > 0 && shift != 0.0 {
            let pull = self.nominal * MAX_PULL;
            self.period =
                (self.period + DRIFT_GAIN * shift).clamp(self.nominal - pull, self.nominal + pull);
            self.phase -= shift;
            self.phase = self.phase.rem_euclid(self.period);
        }
        let (dy, dx) = steps(shift, self.width, self.line_samples());
        let a = 1.0 / self.depth as f32;
        let back = C32::new((-self.roll).cos() as f32, (-self.roll).sin() as f32);
        for y in 0..self.height {
            for x in 0..self.width {
                let from = y * self.width + x;
                if self.count[from] == 0 {
                    continue;
                }
                let value = self.sum[from] / self.count[from] as f32;
                self.last[from] = value;
                self.held[from] = true;
                let value = value * back;
                let ty = (y as isize - dy).rem_euclid(self.height as isize) as usize;
                let tx = (x as isize - dx).rem_euclid(self.width as isize) as usize;
                let to = ty * self.width + tx;
                self.average[to] = match self.seen[to] {
                    false => value,
                    true => self.average[to] * (1.0 - a) + value * a,
                };
                self.seen[to] = true;
            }
        }
        self.sum.iter_mut().for_each(|s| *s = C32::new(0.0, 0.0));
        self.count.iter_mut().for_each(|c| *c = 0);
        self.averaged += 1;
    }

    fn line_samples(&self) -> f64 {
        self.period / self.height as f64
    }

    fn profiles(&self) -> Vec<f32> {
        let mut cols = vec![0.0f32; self.width];
        let mut seen = vec![0u32; self.width];
        for y in 0..self.height {
            for x in 0..self.width {
                let at = y * self.width + x;
                let c = self.count[at];
                if c == 0 {
                    continue;
                }
                cols[x] += self.sum[at].norm();
                seen[x] += c;
            }
        }
        for (s, c) in cols.iter_mut().zip(&seen) {
            *s = match c {
                0 => 0.0,
                c => *s / *c as f32,
            };
        }
        cols
    }

    /// The angle between what was just painted and the frame before it.
    ///
    /// Against the frame before rather than against the average, because the
    /// average is a mixture of frames at whatever phases they arrived at and
    /// its own angle lags: measured that way the loop never settled, hunting
    /// between plus and minus two radians for as long as it ran.
    fn turn_from_last(&self) -> f64 {
        let mut dot = C32::new(0.0, 0.0);
        for (at, count) in self.count.iter().enumerate() {
            if *count == 0 || !self.held[at] {
                continue;
            }
            dot += (self.sum[at] / *count as f32) * self.last[at].conj();
        }
        match dot.norm() > 0.0 {
            true => f64::from(dot.arg()),
            false => 0.0,
        }
    }

    fn offset_from_average(&self) -> Option<f64> {
        let cols = self.profiles();
        let reach = (self.width as f64 * SEARCH_LINE_FRACTION) as isize;
        let seen: Vec<f32> = self.average.iter().map(|v| v.norm()).collect();
        let (dx, quality) = best_shift(&profile(&seen, self.width, Axis::Column), &cols, reach);
        if quality < MIN_CORRELATION {
            return None;
        }
        Some(dx * self.line_samples() / self.width as f64)
    }

    pub fn picture(&self, nudge: (isize, isize), align: bool) -> Vec<u8> {
        let flat = self.projected();
        let lit: Vec<f32> =
            flat.iter().zip(&self.seen).filter(|(_, s)| **s).map(|(v, _)| *v).collect();
        let fill = match lit.is_empty() {
            true => 0.0,
            false => lit.iter().sum::<f32>() / lit.len() as f32,
        };
        let mut out: Vec<f32> = flat
            .iter()
            .zip(&self.seen)
            .map(|(v, s)| match s {
                true => *v,
                false => fill,
            })
            .collect();
        let (mut dx, mut dy) = nudge;
        if align {
            let cols = profile(&out, self.width, Axis::Column);
            let rows = profile(&out, self.width, Axis::Row);
            dx += blank_end(&cols) as isize;
            dy += blank_end(&rows) as isize;
        }
        shift(&mut out, self.width, dx, dy);
        let mut sorted = out.clone();
        sorted.sort_by(f32::total_cmp);
        let cut = (sorted.len() as f64 * CONTRAST_TAIL) as usize;
        let (lo, hi) = (sorted[cut], sorted[sorted.len() - 1 - cut]);
        let range = (hi - lo).max(f32::MIN_POSITIVE);
        out.iter().map(|v| (((v - lo) / range) * 255.0).clamp(0.0, 255.0) as u8).collect()
    }

    /// The averaged cells on one axis of the complex plane: the one they
    /// vary most along.
    ///
    /// A cell is complex because the samples were painted coherently, and
    /// the picture is the part of that which changes from cell to cell. The
    /// axis is the principal one of the scatter, which for a cloud with no
    /// phase in it at all is the real axis and leaves the magnitudes as they
    /// were. Off air the phase is where a pair of colours with the same
    /// amplitude differ, and projecting rather than taking the magnitude is
    /// what keeps them apart.
    fn projected(&self) -> Vec<f32> {
        let lit = self.seen.iter().filter(|s| **s).count().max(1) as f32;
        let mean: C32 =
            self.average.iter().zip(&self.seen).filter(|(_, s)| **s).map(|(v, _)| *v).sum::<C32>()
                / lit;
        let spread: C32 = self
            .average
            .iter()
            .zip(&self.seen)
            .filter(|(_, s)| **s)
            .map(|(v, _)| {
                let d = *v - mean;
                d * d
            })
            .sum::<C32>();
        let angle = match spread.norm() > 0.0 {
            true => -0.5 * spread.arg(),
            false => 0.0,
        };
        let turn = C32::new(angle.cos(), angle.sin());
        self.average.iter().map(|v| ((*v - mean) * turn).re).collect()
    }
}

#[derive(Clone, Copy, PartialEq)]
enum Axis {
    Column,
    Row,
}

fn profile(canvas: &[f32], width: usize, axis: Axis) -> Vec<f32> {
    let height = canvas.len() / width.max(1);
    match axis {
        Axis::Column => (0..width)
            .map(|x| (0..height).map(|y| canvas[y * width + x]).sum::<f32>() / height as f32)
            .collect(),
        Axis::Row => {
            canvas.chunks(width).map(|row| row.iter().sum::<f32>() / width as f32).collect()
        }
    }
}

const MAX_PULL: f64 = 0.001;

/// What fraction of the canvas is allowed off each end of the grey scale.
const CONTRAST_TAIL: f64 = 0.02;

/// Frames of a 640x480 desktop correlate with the average at a median of
/// 0.998; frames of noise alone reach 0.229 at most.
const MIN_CORRELATION: f32 = 0.3;

fn best_shift(a: &[f32], b: &[f32], max: isize) -> (f64, f32) {
    let n = a.len();
    if n == 0 || b.len() != n {
        return (0.0, 0.0);
    }
    let mean = |v: &[f32]| v.iter().sum::<f32>() / n as f32;
    let (ma, mb) = (mean(a), mean(b));
    let at = |s: isize| -> f32 {
        (0..n)
            .map(|i| {
                let j = (i as isize - s).rem_euclid(n as isize) as usize;
                (b[i] - mb) * (a[j] - ma)
            })
            .sum()
    };
    let mut best = (0isize, f32::MIN);
    for s in -max..=max {
        let dot = at(s);
        if dot > best.1 {
            best = (s, dot);
        }
    }
    let (left, right) = (at(best.0 - 1) as f64, at(best.0 + 1) as f64);
    let denom = left - 2.0 * best.1 as f64 + right;
    let offset = match denom < 0.0 {
        true => (0.5 * (left - right) / denom).clamp(-0.5, 0.5),
        false => 0.0,
    };
    let power = |v: &[f32], m: f32| v.iter().map(|x| (x - m) * (x - m)).sum::<f32>();
    let scale = (power(a, ma) * power(b, mb)).sqrt();
    let quality = match scale > 0.0 {
        true => best.1 / scale,
        false => 0.0,
    };
    (best.0 as f64 + offset, quality)
}

fn blank_end(profile: &[f32]) -> usize {
    let n = profile.len();
    if n < 4 {
        return 0;
    }
    let smooth: Vec<f32> = (0..n)
        .map(|i| {
            let l = profile[(i + n - 1) % n];
            let r = profile[(i + 1) % n];
            (l + profile[i] + r) / 3.0
        })
        .collect();
    let (lo, hi) = smooth.iter().fold((f32::MAX, f32::MIN), |(lo, hi), v| (lo.min(*v), hi.max(*v)));
    let dark = lo + 0.25 * (hi - lo);
    let (mut best_end, mut best_len) = (0usize, 0usize);
    let (mut run, mut start) = (0usize, 0usize);
    for i in 0..2 * n {
        let at = i % n;
        if smooth[at] <= dark {
            if run == 0 {
                start = at;
            }
            run += 1;
            if run > best_len && run <= n {
                best_len = run;
                best_end = (start + run) % n;
            }
        } else {
            run = 0;
        }
    }
    best_end
}

fn shift(canvas: &mut [f32], width: usize, dx: isize, dy: isize) {
    let height = canvas.len() / width.max(1);
    if width == 0 || height == 0 || (dx == 0 && dy == 0) {
        return;
    }
    let src = canvas.to_vec();
    for y in 0..height {
        let sy = (y as isize + dy).rem_euclid(height as isize) as usize;
        for x in 0..width {
            let sx = (x as isize + dx).rem_euclid(width as isize) as usize;
            canvas[y * width + x] = src[sy * width + sx];
        }
    }
}

fn steps(samples: f64, width: usize, line_samples: f64) -> (isize, isize) {
    if line_samples <= 0.0 {
        return (0, 0);
    }
    let lines = samples / line_samples;
    let dy = lines.round();
    let dx = ((lines - dy) * width as f64).round();
    (dy as isize, dx as isize)
}

#[cfg(test)]
mod tests {
    use super::*;

    struct Screen {
        clock_hz: f64,
        htotal: usize,
        vtotal: usize,
        active: (usize, usize),
    }

    const VGA: Screen = Screen { clock_hz: 25.175e6, htotal: 800, vtotal: 525, active: (640, 480) };
    const XGA: Screen = Screen { clock_hz: 65.0e6, htotal: 1344, vtotal: 806, active: (1024, 768) };

    impl Screen {
        fn frame_hz(&self) -> f64 {
            self.clock_hz / (self.htotal * self.vtotal) as f64
        }

        fn emit(
            &self,
            rate: f64,
            seconds: f64,
            noise: f32,
            image: &dyn Fn(f64, f64) -> f32,
        ) -> Vec<f32> {
            let mut state = 0x2545F4914F6CDD1Du64;
            let mut rand = move || {
                state ^= state << 13;
                state ^= state >> 7;
                state ^= state << 17;
                (state >> 40) as f32 / 8_388_608.0 - 0.5
            };
            let n = (rate * seconds) as usize;
            (0..n)
                .map(|i| {
                    let pixel = i as f64 * self.clock_hz / rate;
                    let line = (pixel / self.htotal as f64) as usize;
                    let x = pixel as usize % self.htotal;
                    let y = line % self.vtotal;
                    let lit = match (x < self.active.0, y < self.active.1) {
                        (true, true) => {
                            image(x as f64 / self.active.0 as f64, y as f64 / self.active.1 as f64)
                        }
                        _ => 0.0,
                    };
                    lit + rand() * noise
                })
                .collect()
        }
    }

    fn desktop(u: f64, v: f64) -> f32 {
        let window = |x: f64, y: f64, w: f64, h: f64| {
            (u > x && u < x + w && v > y && v < y + h) as u8 as f32
        };
        0.35 + 0.4 * window(0.05, 0.1, 0.4, 0.5)
            + 0.6 * window(0.5, 0.3, 0.45, 0.6)
            + 0.25 * window(0.0, 0.94, 1.0, 0.06)
    }

    const MID_FRAME: usize = 61_237;

    /// An envelope as the raster takes it now, with no phase in it.
    fn real(env: &[f32]) -> Vec<C32> {
        env.iter().map(|v| C32::new(*v, 0.0)).collect()
    }

    fn bytes(v: &[u8]) -> Vec<f32> {
        v.iter().map(|b| *b as f32).collect()
    }

    fn correlation(a: &[u8], b: &[u8]) -> f32 {
        let mean = |v: &[u8]| v.iter().map(|x| *x as f32).sum::<f32>() / v.len() as f32;
        let (ma, mb) = (mean(a), mean(b));
        let (mut dot, mut pa, mut pb) = (0.0f32, 0.0f32, 0.0f32);
        for (x, y) in a.iter().zip(b) {
            let (u, v) = (*x as f32 - ma, *y as f32 - mb);
            dot += u * v;
            pa += u * u;
            pb += v * v;
        }
        dot / (pa.sqrt() * pb.sqrt()).max(f32::MIN_POSITIVE)
    }

    #[test]
    fn a_vga_screen_says_how_long_its_frame_and_its_line_are() {
        let rate = 8e6;
        let env = VGA.emit(rate, 0.06, 0.2, &desktop);
        let p = find_periods(&env, rate, Limits::default()).expect("a raster");
        assert_eq!(p.lines, 525, "whole lines in a frame");
        let hz = p.frame_hz(rate);
        assert!((hz - VGA.frame_hz()).abs() < 0.05, "{hz} Hz, sent {}", VGA.frame_hz());
        let line = p.line_hz(rate);
        let sent = VGA.clock_hz / VGA.htotal as f64;
        assert!((line - sent).abs() < 30.0, "{line} Hz a line, sent {sent}");
    }

    #[test]
    fn an_xga_screen_says_the_same_at_a_rate_that_cannot_hold_its_clock() {
        let rate = 12e6;
        let env = XGA.emit(rate, 0.06, 0.5, &desktop);
        let p = find_periods(&env, rate, Limits::default()).expect("a raster");
        assert_eq!(p.lines, 806);
        assert!((p.frame_hz(rate) - 60.0038).abs() < 0.05, "{} Hz", p.frame_hz(rate));
    }

    /// The comb's tooth is found as a frequency, and precisely: a line
    /// 17700 Hz off the dial under noise four times its own amplitude comes
    /// back to a fraction of a hertz, where a bin of this transform is 38.
    #[test]
    fn the_carrier_is_found_under_noise_four_times_its_size() {
        let rate = 20e6;
        let offset = 17_700.0;
        let mut state = 0x243F6A8885A308D3u64;
        let mut rand = move || {
            state ^= state << 13;
            state ^= state >> 7;
            state ^= state << 17;
            (state >> 40) as f32 / 16_777_216.0 - 0.5
        };
        let iq: Vec<C32> = (0..1 << 19)
            .map(|i| {
                let phase = std::f64::consts::TAU * offset * i as f64 / rate;
                C32::new(phase.cos() as f32, phase.sin() as f32)
                    + C32::new(rand() * 4.0, rand() * 4.0)
            })
            .collect();
        let found = find_carrier(&iq, rate, (-400e3, 400e3)).expect("a carrier");
        assert!(
            (found.offset_hz - offset).abs() < 1.0,
            "read {:.2} Hz for a carrier at {offset}",
            found.offset_hz
        );
        assert!(found.over_db > 30.0, "only {:.1} dB over the floor", found.over_db);

        // Noise alone has no line in it worth the name.
        let noise: Vec<C32> = (0..1 << 19).map(|_| C32::new(rand() * 4.0, rand() * 4.0)).collect();
        let none = find_carrier(&noise, rate, (-400e3, 400e3)).expect("a strongest bin");
        assert!(none.over_db < 20.0, "noise peaked {:.1} dB over its own median", none.over_db);
    }

    #[test]
    fn noise_alone_is_not_a_raster() {
        let mut state = 0x9E3779B97F4A7C15u64;
        let env: Vec<f32> = (0..262_144)
            .map(|_| {
                state ^= state << 13;
                state ^= state >> 7;
                state ^= state << 17;
                (state >> 40) as f32 / 8_388_608.0 - 0.5
            })
            .collect();
        assert_eq!(find_periods(&env, 8e6, Limits::default()), None);
    }

    #[test]
    fn a_screen_read_at_the_wrong_period_still_comes_out_still() {
        let rate = 8e6;
        let seconds = 0.5;
        let env = VGA.emit(rate, seconds, 0.6, &desktop);
        let truth = rate / VGA.frame_hz();
        let wrong = truth * (1.0 + 30e-6);
        let mut r = Raster::new(254, 525, wrong, 8.0);
        r.push(&real(&env));
        assert!(r.frames() >= 29, "{} frames of {seconds}s", r.frames());
        let ppm = (r.period() - truth) / truth * 1e6;
        assert!(ppm.abs() < 2.0, "{ppm} ppm out after {} frames", r.frames());
    }

    #[test]
    fn the_picture_that_comes_back_is_the_picture_that_was_sent() {
        let rate = 8e6;
        let env = VGA.emit(rate, 0.35, 0.4, &desktop);
        let width = 254;
        let mut r = Raster::new(width, 525, rate / VGA.frame_hz(), 8.0);
        r.push(&real(&env[MID_FRAME..]));
        let got = r.picture((0, 0), true);
        let want: Vec<u8> = (0..525)
            .flat_map(|y| {
                (0..width).map(move |x| {
                    let (u, v) = (x as f64 / width as f64, y as f64 / 525.0);
                    let lit = match (u < 640.0 / 800.0, v < 480.0 / 525.0) {
                        (true, true) => desktop(u * 800.0 / 640.0, v * 525.0 / 480.0),
                        _ => 0.0,
                    };
                    (lit * 255.0) as u8
                })
            })
            .collect();
        let (as_f32, want_f32) = (bytes(&got), bytes(&want));
        let dy = best_shift(
            &profile(&want_f32, width, Axis::Row),
            &profile(&as_f32, width, Axis::Row),
            40,
        )
        .0
        .round() as isize;
        let dx = best_shift(
            &profile(&want_f32, width, Axis::Column),
            &profile(&as_f32, width, Axis::Column),
            40,
        )
        .0
        .round() as isize;
        assert!(dy.abs() <= 12, "the blanking put the picture {dy} lines off");
        assert!(dx.abs() <= 8, "the blanking put the picture {dx} columns off");
        let mut aligned = want_f32;
        shift(&mut aligned, width, -dx, -dy);
        let want: Vec<u8> = aligned.iter().map(|v| *v as u8).collect();
        let c = correlation(&got, &want);
        assert!(c > 0.8, "the picture correlates {c} with what was on the screen");
    }
}
