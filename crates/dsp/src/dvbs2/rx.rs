use super::acquire;
use super::pl::{self, HEADER, Header, PILOT_BLOCK, PlFrame, SOF_LEN};
use crate::Mixer;
use common::C32;
use std::f64::consts::PI;

const PHASES: usize = 64;
const SPAN: f64 = 12.0;
const PAIRS: usize = 32;
const FOUND: f32 = 0.45;
const KEPT: f32 = 0.3;
const MISSES: u32 = 3;
const SEGMENT: usize = 15;
const ACQUIRE: usize = 1 << 16;
const TIMING_KP: f64 = 0.005;
const TIMING_KI: f64 = 2e-6;
const TIMING_RANGE: f64 = 1e-3;
const MER_FLOOR: f32 = 1.0;
const OMEGA_STEP: f64 = 2e-3;
const SETTLED: f64 = 2e-4;
const PILOTLESS_GAIN: f64 = 0.1;
const DD_PILOTS: f32 = 1e-2;
const DD_FIRST: f32 = 1e-2;
const DD_SECOND: f32 = 2e-5;

#[derive(Clone, Copy, Debug)]
pub struct Config {
    pub rate_hz: f64,
    pub symbol_rate: f64,
    pub rolloff: f64,
    pub gold: u32,
}

#[derive(Clone, Debug)]
pub struct Received {
    pub header: Header,
    pub llr: Vec<f32>,
    pub mer_db: f32,
    pub offset_hz: f64,
}

struct Timing {
    phases: Vec<f32>,
    taps: usize,
    width: usize,
    history: Vec<C32>,
    at: f64,
    period: f64,
    integral: f64,
    on_time: bool,
    last: C32,
    mid: C32,
    power: f32,
}

impl Timing {
    fn new(cfg: &Config) -> Timing {
        let sps = cfg.rate_hz / cfg.symbol_rate;
        let taps = ((SPAN * sps).ceil() as usize).max(4) & !1;
        let width = taps.div_ceil(4) * 4;
        let mut phases = vec![0f32; (PHASES + 1) * 2 * width];
        let mut energy = 0.0f32;
        for p in 0..=PHASES {
            let frac = p as f64 / PHASES as f64;
            for j in 0..taps {
                let d = frac + (taps / 2 - 1) as f64 - j as f64;
                let t = rrc(d / sps, cfg.rolloff) as f32;
                phases[(p * width + j) * 2] = t;
                phases[(p * width + j) * 2 + 1] = t;
                if p == 0 {
                    energy += t * t;
                }
            }
        }
        let gain = energy.sqrt().recip();
        phases.iter_mut().for_each(|t| *t *= gain);
        Timing {
            phases,
            taps,
            width,
            history: vec![C32::new(0.0, 0.0); width],
            at: width as f64,
            period: sps,
            integral: 0.0,
            on_time: true,
            last: C32::new(0.0, 0.0),
            mid: C32::new(0.0, 0.0),
            power: 1.0,
        }
    }

    fn push(&mut self, iq: &[C32], out: &mut Vec<C32>) {
        self.history.extend_from_slice(iq);
        let half = self.taps / 2;
        let flat = floats(&self.history);
        loop {
            let n = self.at as usize;
            let start = n + 1 - half;
            if start + self.width > self.history.len() {
                break;
            }
            let p = ((self.at - n as f64) * PHASES as f64 + 0.5) as usize;
            let taps = &self.phases[p * 2 * self.width..(p + 1) * 2 * self.width];
            let x = &flat[start * 2..(start + self.width) * 2];
            let mut acc = wide::f32x8::ZERO;
            for (xs, ts) in x.chunks_exact(8).zip(taps.chunks_exact(8)) {
                let xs = wide::f32x8::from(<[f32; 8]>::try_from(xs).unwrap_or_default());
                let ts = wide::f32x8::from(<[f32; 8]>::try_from(ts).unwrap_or_default());
                acc = xs.mul_add(ts, acc);
            }
            let acc = acc.to_array();
            let y = C32::new(acc[0] + acc[2] + acc[4] + acc[6], acc[1] + acc[3] + acc[5] + acc[7]);
            let step = self.period / 2.0 * (1.0 + self.integral);
            if self.on_time {
                let e = ((self.last - y) * self.mid.conj()).re / self.power.max(1e-12);
                let e = e.clamp(-1.0, 1.0) as f64;
                self.integral = (self.integral + TIMING_KI * e).clamp(-TIMING_RANGE, TIMING_RANGE);
                self.at += step + TIMING_KP * e * self.period / 2.0;
                self.power += 0.001 * (y.norm_sqr() - self.power);
                self.last = y;
                out.push(y);
            } else {
                self.at += step;
                self.mid = y;
            }
            self.on_time = !self.on_time;
        }
        let keep = (self.at as usize).saturating_sub(self.width);
        if keep > 0 {
            self.history.drain(..keep);
            self.at -= keep as f64;
        }
    }

    fn ratio(&self) -> f64 {
        1.0 + self.integral
    }
}

fn floats(x: &[C32]) -> &[f32] {
    unsafe { std::slice::from_raw_parts(x.as_ptr() as *const f32, x.len() * 2) }
}

fn rrc(t: f64, a: f64) -> f64 {
    if t.abs() < 1e-9 {
        return 1.0 - a + 4.0 * a / PI;
    }
    if (t.abs() - 1.0 / (4.0 * a)).abs() < 1e-9 {
        let q = PI / (4.0 * a);
        return a / 2f64.sqrt() * ((1.0 + 2.0 / PI) * q.sin() + (1.0 - 2.0 / PI) * q.cos());
    }
    let pt = PI * t;
    ((pt * (1.0 - a)).sin() + 4.0 * a * t * (pt * (1.0 + a)).cos())
        / (pt * (1.0 - (4.0 * a * t).powi(2)))
}

#[derive(Clone, Copy, Debug)]
enum State {
    Searching,
    Locked { at: usize, frame: PlFrame },
}

pub struct Dvbs2 {
    cfg: Config,
    mixer: Mixer,
    timing: Timing,
    shifted: Vec<C32>,
    symbols: Vec<C32>,
    state: State,
    misses: u32,
    omega: f64,
    settled: bool,
    offset_hz: f64,
    acquiring: Option<Vec<C32>>,
    mer_db: Option<f32>,
    heard: Option<Header>,
    scramble: std::sync::Arc<[u8]>,
    sof_diff: [C32; SOF_LEN - 1],
    pair_diff: [C32; PAIRS],
}

impl Dvbs2 {
    pub fn new(cfg: Config) -> Dvbs2 {
        let h = pl::header_symbols(0);
        let sof_diff = std::array::from_fn(|i| h[i].conj() * h[i + 1]);
        let pair_diff = std::array::from_fn(|i| h[SOF_LEN + 2 * i].conj() * h[SOF_LEN + 2 * i + 1]);
        Dvbs2 {
            cfg,
            mixer: Mixer::new(0.0, cfg.rate_hz),
            timing: Timing::new(&cfg),
            shifted: Vec::new(),
            symbols: Vec::new(),
            state: State::Searching,
            misses: 0,
            omega: 0.0,
            settled: false,
            offset_hz: 0.0,
            acquiring: Some(Vec::new()),
            mer_db: None,
            heard: None,
            scramble: pl::scrambling(cfg.gold).into_owned().into(),
            sof_diff,
            pair_diff,
        }
    }

    pub fn locked(&self) -> bool {
        matches!(self.state, State::Locked { .. })
    }

    pub fn heard(&self) -> Option<Header> {
        self.heard
    }

    pub fn mer_db(&self) -> Option<f32> {
        self.mer_db
    }

    pub fn offset_hz(&self) -> f64 {
        self.offset_hz + self.omega * self.cfg.symbol_rate / (2.0 * PI)
    }

    pub fn symbol_rate(&self) -> f64 {
        self.cfg.symbol_rate / self.timing.ratio()
    }

    pub fn push(&mut self, iq: &[C32], out: &mut Vec<Framed>) {
        if let Some(buf) = &mut self.acquiring {
            buf.extend_from_slice(iq);
            if buf.len() < ACQUIRE {
                return;
            }
            let buf = self.acquiring.take().unwrap_or_default();
            self.offset_hz = acquire::occupied(&buf, self.cfg.rate_hz)
                .filter(|b| b.width_hz < 1.6 * self.cfg.symbol_rate)
                .map_or(0.0, |b| b.centre_hz);
            self.mixer = Mixer::new(-self.offset_hz, self.cfg.rate_hz);
            self.push(&buf, out);
            return;
        }
        self.shifted.clear();
        self.mixer.process(iq, &mut self.shifted);
        let shifted = std::mem::take(&mut self.shifted);
        self.timing.push(&shifted, &mut self.symbols);
        self.shifted = shifted;
        self.run(out);
    }

    fn run(&mut self, out: &mut Vec<Framed>) {
        loop {
            match self.state {
                State::Searching => {
                    if !self.search() {
                        return;
                    }
                }
                State::Locked { at, frame } => {
                    let Some(len) = frame.symbols() else {
                        self.lose();
                        continue;
                    };
                    if self.symbols.len() < at + len + HEADER + 2 {
                        return;
                    }
                    let next = (at + len - 2..=at + len + 2)
                        .max_by(|&a, &b| self.metric(a).total_cmp(&self.metric(b)))
                        .unwrap_or(at + len);
                    let found = self.metric(next) >= KEPT;
                    let following = if found { Some(self.decode_pls(next)) } else { None };
                    if let PlFrame::Data(header) = frame
                        && let Some(r) = self.frame(at, header, found.then_some(next))
                    {
                        self.heard = Some(header);
                        out.push(r);
                    }
                    if found {
                        self.misses = 0;
                    } else {
                        self.misses += 1;
                        if self.misses >= MISSES {
                            self.lose();
                            continue;
                        }
                    }
                    let at = if found { next } else { at + len };
                    let frame = following.unwrap_or(frame);
                    self.symbols.drain(..at);
                    self.state = State::Locked { at: 0, frame };
                }
            }
        }
    }

    fn lose(&mut self) {
        self.state = State::Searching;
        self.misses = 0;
        self.mer_db = None;
        self.settled = false;
        self.omega = 0.0;
        self.acquiring = Some(Vec::new());
    }

    fn search(&mut self) -> bool {
        if self.symbols.len() < 2 * HEADER {
            return false;
        }
        let limit = self.symbols.len() - HEADER - 8;
        let Some(first) = (0..limit).find(|&p| self.metric(p) >= FOUND) else {
            self.symbols.drain(..limit);
            return false;
        };
        let best = (first..first + 8)
            .max_by(|&a, &b| self.metric(a).total_cmp(&self.metric(b)))
            .unwrap_or(first);
        let frame = self.decode_pls(best);
        if matches!(frame, PlFrame::Reserved(_)) {
            self.symbols.drain(..best + 1);
            return true;
        }
        self.symbols.drain(..best);
        self.state = State::Locked { at: 0, frame };
        true
    }

    fn metric(&self, p: usize) -> f32 {
        let s = &self.symbols[p..p + HEADER];
        let mut sof = C32::new(0.0, 0.0);
        let mut norm = 0.0f32;
        for i in 0..SOF_LEN - 1 {
            let d = s[i].conj() * s[i + 1];
            sof += d * self.sof_diff[i].conj();
            norm += d.norm();
        }
        let mut pairs = C32::new(0.0, 0.0);
        for i in 0..PAIRS {
            let d = s[SOF_LEN + 2 * i].conj() * s[SOF_LEN + 2 * i + 1];
            pairs += d * self.pair_diff[i].conj();
            norm += d.norm();
        }
        (sof + pairs).norm().max((sof - pairs).norm()) / norm.max(1e-20)
    }

    fn decode_pls(&self, p: usize) -> PlFrame {
        let s = &self.symbols[p..p + HEADER];
        let best = (0..128u8)
            .max_by(|&a, &b| {
                segmented(s, &pl::headers()[a as usize])
                    .total_cmp(&segmented(s, &pl::headers()[b as usize]))
            })
            .unwrap_or(0);
        PlFrame::from_code(best)
    }

    fn frame(&mut self, at: usize, header: Header, next: Option<usize>) -> Option<Framed> {
        let len = header.symbols();
        let mid = (HEADER / 2) as f64;
        let raw = &self.symbols[at..at + len];
        let omega = self.omega;
        let near = |k0: usize, k1: usize, source: &[C32]| -> Vec<C32> {
            let mut v = Vec::with_capacity(k1 - k0);
            rotate_run(&source[k0..k1], -omega * (k0 as f64 - mid), -omega, &mut v);
            v
        };
        let reference = &pl::headers()[header.code() as usize];
        let head = near(0, HEADER, raw);
        let slope = header_slope(&head, reference);
        let mut w = if self.settled && header.pilots { 0.0 } else { slope };
        let mut points: Vec<(f64, f64)> = vec![(mid, correlate(&head, reference).arg() as f64)];
        let runs = pl::runs(header);
        let pilot_runs: Vec<std::ops::Range<usize>> =
            runs.iter().filter(|r| r.pilot).map(|r| r.span.clone()).collect();
        let mut pilot_symbols = Vec::with_capacity(pilot_runs.len());
        for span in &pilot_runs {
            let v = near(HEADER + span.start, HEADER + span.end, raw);
            let acc: C32 = v
                .iter()
                .zip(span.clone())
                .map(|(s, i)| s * (pl::pilot() * pl::rotation(self.scramble[i])).conj())
                .sum();
            add_point(
                &mut points,
                &mut w,
                (HEADER + span.start + PILOT_BLOCK / 2) as f64,
                acc.arg() as f64,
            );
            pilot_symbols.push(v);
        }
        if let Some(n) = next.filter(|_| header.pilots) {
            let code = match self.decode_pls(n) {
                PlFrame::Data(h) => h.code(),
                PlFrame::Dummy { pilots } => pilots as u8,
                PlFrame::Reserved(c) => c,
            };
            let rel = n - at;
            let v = near(rel, rel + HEADER, &self.symbols[at..]);
            let acc = correlate(&v, &pl::headers()[code as usize]);
            add_point(&mut points, &mut w, rel as f64 + mid, acc.arg() as f64);
        }
        let known = head
            .iter()
            .zip(reference.iter())
            .enumerate()
            .map(|(k, (s, r))| (s * rot(-phase_at(&points, w, k as f64)), *r))
            .chain(pilot_runs.iter().zip(&pilot_symbols).flat_map(|(span, v)| {
                v.iter().zip(span.clone()).map(|(s, i)| {
                    let k = (HEADER + i) as f64;
                    (
                        s * rot(-phase_at(&points, w, k)) * pl::rotation(self.scramble[i]).conj(),
                        pl::pilot(),
                    )
                })
            }))
            .collect::<Vec<_>>();
        let a = (known.iter().map(|(s, r)| (s * r.conj()).re).sum::<f32>() / known.len() as f32)
            .max(1e-12);
        let noise: f32 = known.iter().map(|(s, r)| (s / a - r).norm_sqr()).sum();
        let sigma2 = (noise / known.len() as f32).max(1e-6);
        let mer = -10.0 * sigma2.log10();
        if !mer.is_finite() || mer < MER_FLOOR {
            self.settled = false;
            return None;
        }
        self.mer_db = Some(mer);
        let (residual, gain) = if header.pilots { (w, 0.5) } else { (slope, PILOTLESS_GAIN) };
        let most = if self.settled { OMEGA_STEP } else { 0.1 };
        self.omega += (gain * residual).clamp(-most, most);
        self.settled = residual.abs() < SETTLED;
        Some(Framed {
            header,
            raw: raw.to_vec(),
            omega,
            points,
            w,
            amplitude: a,
            sigma2,
            scramble: self.scramble.clone(),
            mer_db: mer,
            offset_hz: self.offset_hz(),
        })
    }
}

pub struct Framed {
    pub header: Header,
    raw: Vec<C32>,
    omega: f64,
    points: Vec<(f64, f64)>,
    w: f64,
    amplitude: f32,
    sigma2: f32,
    scramble: std::sync::Arc<[u8]>,
    pub mer_db: f32,
    pub offset_hz: f64,
}

impl Framed {
    pub fn demodulate(&self) -> Received {
        let header = self.header;
        let mid = (HEADER / 2) as f64;
        let mut v = Vec::with_capacity(self.raw.len());
        rotate_run(&self.raw, self.omega * mid, -self.omega, &mut v);
        let mut aligned = Vec::with_capacity(self.raw.len());
        track(&v, &self.points, self.w, &mut aligned);
        let constellation = header.modcod.points();
        let bps = header.modcod.constellation.bits();
        let mut llr = Vec::with_capacity(header.frame.bits());
        let mut dd = if header.pilots {
            Directed::new(DD_PILOTS, 0.0)
        } else {
            Directed::new(DD_FIRST, DD_SECOND)
        };
        let (scale, gain) = (1.0 / self.sigma2, 1.0 / self.amplitude);
        for run in pl::runs(header) {
            for i in run.span {
                let s = aligned[HEADER + i] * pl::rotation(self.scramble[i]).conj();
                let z = dd.correct(s * gain);
                let nearest = if run.pilot {
                    pl::pilot()
                } else {
                    demap(z, constellation, bps, scale, &mut llr)
                };
                dd.learn(z, nearest);
            }
        }
        Received { header, llr, mer_db: self.mer_db, offset_hz: self.offset_hz }
    }
}

fn phase_at(points: &[(f64, f64)], w: f64, k: f64) -> f64 {
    if points.len() == 1 {
        return points[0].1 + w * (k - points[0].0);
    }
    let i = points.iter().position(|p| p.0 > k).unwrap_or(points.len() - 1).max(1);
    let (a, b) = (points[i - 1], points[i]);
    a.1 + (b.1 - a.1) * (k - a.0) / (b.0 - a.0)
}

struct Directed {
    phasor: C32,
    rate: f32,
    first: f32,
    second: f32,
    count: u32,
}

impl Directed {
    fn new(first: f32, second: f32) -> Directed {
        Directed { phasor: C32::new(1.0, 0.0), rate: 0.0, first, second, count: 0 }
    }

    fn correct(&self, z: C32) -> C32 {
        z * self.phasor
    }

    fn learn(&mut self, z: C32, decided: C32) {
        let e = ((z * decided.conj()).im / decided.norm_sqr().max(1e-6)).clamp(-0.5, 0.5);
        self.rate += self.second * e;
        let step = -(self.first * e + self.rate);
        self.phasor *= C32::new(1.0 - step * step / 2.0, step);
        self.count += 1;
        if self.count.is_multiple_of(64) {
            self.phasor /= self.phasor.norm();
        }
    }
}

fn rotate_run(input: &[C32], start: f64, step: f64, out: &mut Vec<C32>) {
    let turn = rot(step);
    let mut p = rot(start);
    for (k, s) in input.iter().enumerate() {
        if k % 1024 == 0 {
            p = rot(start + step * k as f64);
        }
        out.push(s * p);
        p *= turn;
    }
}

fn track(v: &[C32], points: &[(f64, f64)], w: f64, out: &mut Vec<C32>) {
    let mut k = 0usize;
    for (i, &(ka, pa)) in points.iter().enumerate() {
        let (slope, until) = match points.get(i + 1) {
            Some(&(kb, pb)) => {
                ((pb - pa) / (kb - ka), if i + 2 < points.len() { kb as usize } else { v.len() })
            }
            None if points.len() == 1 => (w, v.len()),
            None => break,
        };
        let until = until.min(v.len());
        if k < until {
            rotate_run(&v[k..until], -(pa + slope * (k as f64 - ka)), -slope, out);
            k = until;
        }
    }
}

fn add_point(points: &mut Vec<(f64, f64)>, w: &mut f64, k: f64, measured: f64) {
    let (k0, p0) = *points.last().expect("a header first");
    let predicted = p0 + *w * (k - k0);
    let unwrapped = predicted + wrap(measured - predicted);
    points.push((k, unwrapped));
    let n = points.len() as f64;
    let mk = points.iter().map(|p| p.0).sum::<f64>() / n;
    let mp = points.iter().map(|p| p.1).sum::<f64>() / n;
    let num: f64 = points.iter().map(|p| (p.0 - mk) * (p.1 - mp)).sum();
    let den: f64 = points.iter().map(|p| (p.0 - mk).powi(2)).sum();
    if den > 0.0 {
        *w = num / den;
    }
}

fn wrap(x: f64) -> f64 {
    (x + PI).rem_euclid(2.0 * PI) - PI
}

fn rot(phase: f64) -> C32 {
    C32::new(phase.cos() as f32, phase.sin() as f32)
}

fn correlate(s: &[C32], reference: &[C32]) -> C32 {
    s.iter().zip(reference).map(|(a, b)| a * b.conj()).sum()
}

fn segmented(s: &[C32], reference: &[C32]) -> f32 {
    s.chunks(SEGMENT).zip(reference.chunks(SEGMENT)).map(|(a, b)| correlate(a, b).norm()).sum()
}

fn header_slope(s: &[C32], reference: &[C32]) -> f64 {
    let mut unwrapped = Vec::with_capacity(s.len());
    let mut last = 0.0f64;
    for (a, b) in s.iter().zip(reference) {
        let phase = (a * b.conj()).arg() as f64;
        let next = if unwrapped.is_empty() { phase } else { last + wrap(phase - last) };
        unwrapped.push(next);
        last = next;
    }
    let n = unwrapped.len() as f64;
    let mk = (n - 1.0) / 2.0;
    let mp = unwrapped.iter().sum::<f64>() / n;
    let num: f64 = unwrapped.iter().enumerate().map(|(k, p)| (k as f64 - mk) * (p - mp)).sum();
    let den: f64 = (0..unwrapped.len()).map(|k| (k as f64 - mk).powi(2)).sum();
    num / den
}

fn demap(y: C32, points: &[C32], bps: usize, scale: f32, out: &mut Vec<f32>) -> C32 {
    match bps {
        2 => demap_n::<4, 2>(y, points, scale, out),
        3 => demap_n::<8, 3>(y, points, scale, out),
        4 => demap_n::<16, 4>(y, points, scale, out),
        _ => demap_n::<32, 5>(y, points, scale, out),
    }
}

fn demap_n<const N: usize, const B: usize>(
    y: C32,
    points: &[C32],
    scale: f32,
    out: &mut Vec<f32>,
) -> C32 {
    let mut d = [0f32; N];
    let (mut nearest, mut best) = (0, f32::MAX);
    for (i, (d, p)) in d.iter_mut().zip(&points[..N]).enumerate() {
        *d = (y - p).norm_sqr();
        if *d < best {
            best = *d;
            nearest = i;
        }
    }
    for b in 0..B {
        let (mut zero, mut one) = (f32::MAX, f32::MAX);
        for (i, &v) in d.iter().enumerate() {
            if (i >> (B - 1 - b)) & 1 == 0 {
                zero = zero.min(v);
            } else {
                one = one.min(v);
            }
        }
        out.push((one - zero) * scale);
    }
    points[nearest]
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dvbs2::pl::{FecFrame, ModCod};
    use crate::dvbs2::tx;
    use crate::resample::Rational;

    struct Noise(u64);

    impl Noise {
        fn uniform(&mut self) -> f32 {
            self.0 = self.0.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
            ((self.0 >> 40) as f32 + 0.5) / (1u64 << 24) as f32
        }

        fn gauss(&mut self) -> C32 {
            let r = (-2.0 * self.uniform().ln()).sqrt();
            let t = 2.0 * std::f32::consts::PI * self.uniform();
            C32::new(r * t.cos(), r * t.sin()) * std::f32::consts::FRAC_1_SQRT_2
        }
    }

    fn air(
        header: Header,
        frames: usize,
        es_n0_db: f32,
        offset_hz: f64,
        clock_ppm: f64,
    ) -> (Vec<Vec<u8>>, Vec<C32>) {
        let mut noise = Noise(3);
        let mut sent = Vec::new();
        let mut symbols = Vec::new();
        for f in 0..frames {
            let bits: Vec<u8> =
                (0..header.frame.bits()).map(|_| (noise.uniform() < 0.5) as u8).collect();
            symbols.extend(tx::frame(header, &bits, 0));
            if f == 2 {
                symbols.extend(tx::dummy(0));
            }
            sent.push(bits);
        }
        let rs = 14.25e6;
        let shaped = tx::shape(&symbols, 4, 0.2);
        let mut resample = Rational::approx(4.0 * rs * (1.0 + clock_ppm * 1e-6), 20e6, 4096);
        let mut at_rate = Vec::new();
        resample.process(&shaped, &mut at_rate);
        let power = at_rate.iter().map(|x| x.norm_sqr()).sum::<f32>() / at_rate.len() as f32;
        let sps = 20e6 / rs;
        let sigma = (power * sps as f32 / 10f32.powf(es_n0_db / 10.0)).sqrt();
        let iq = at_rate
            .iter()
            .enumerate()
            .map(|(n, x)| x * rot(2.0 * PI * offset_hz * n as f64 / 20e6) + noise.gauss() * sigma)
            .collect();
        (sent, iq)
    }

    fn receive(iq: &[C32]) -> (Dvbs2, Vec<Received>) {
        let mut rx =
            Dvbs2::new(Config { rate_hz: 20e6, symbol_rate: 14.25e6, rolloff: 0.2, gold: 0 });
        let mut out = Vec::new();
        for block in iq.chunks(65_536) {
            rx.push(block, &mut out);
        }
        (rx, out.iter().map(Framed::demodulate).collect())
    }

    fn wrong(sent: &[u8], llr: &[f32]) -> usize {
        sent.iter().zip(llr).filter(|(b, l)| (**l < 0.0) != (**b == 1)).count()
    }

    #[test]
    fn eight_psk_three_quarters_at_14_25_msym_from_20_msps() {
        let header = Header {
            modcod: ModCod::from_index(14).unwrap(),
            frame: FecFrame::Normal,
            pilots: true,
        };
        let (sent, iq) = air(header, 8, 16.0, 385e3, 16.0);
        let (rx, got) = receive(&iq);
        let wrong: Vec<usize> =
            got.iter().map(|r| sent.iter().map(|s| wrong(s, &r.llr)).min().unwrap()).collect();
        assert_eq!(got.len(), 7, "every frame but the last, which has no header after it");
        assert!(got.iter().all(|r| r.header == header && r.llr.len() == 64_800));
        assert!(
            wrong.iter().all(|&w| w < 60),
            "raw bit errors {wrong:?}, about 20 a frame at 16 dB"
        );
        let mer = rx.mer_db().unwrap();
        assert!((15.0..16.5).contains(&mer), "MER {mer} for Es/N0 16 dB");
        assert!((rx.offset_hz() - 385e3).abs() < 2e3, "offset {}", rx.offset_hz());
        assert!(
            (rx.symbol_rate() / 14.25e6 - 1.0 - 16e-6).abs() < 3e-6,
            "symbol rate {}",
            rx.symbol_rate()
        );
    }

    fn header_is_apsk32(modcod: u8) -> bool {
        ModCod::from_index(modcod).unwrap().constellation == crate::dvbs2::Constellation::Apsk32
    }

    #[test]
    fn every_constellation_and_frame_size_comes_back() {
        for (modcod, frame, pilots) in [
            (4u8, FecFrame::Normal, true),
            (14, FecFrame::Normal, false),
            (14, FecFrame::Short, true),
            (20, FecFrame::Normal, true),
            (26, FecFrame::Short, true),
            (26, FecFrame::Normal, false),
        ] {
            let most = if header_is_apsk32(modcod) { 12 } else { 0 };
            let header = Header { modcod: ModCod::from_index(modcod).unwrap(), frame, pilots };
            let (sent, iq) = air(header, 16, 24.0, -120e3, -8.0);
            let (_, got) = receive(&iq);
            let wrong: Vec<usize> =
                got.iter().map(|r| sent.iter().map(|s| wrong(s, &r.llr)).min().unwrap()).collect();
            assert_eq!(got.len(), 15, "{}: every frame but the last", header.label());
            assert!(
                wrong.iter().all(|&w| w <= most),
                "{}: raw bit errors {wrong:?} at 24 dB",
                header.label()
            );
        }
    }

    #[test]
    fn the_carrier_is_found_from_the_samples() {
        let header = Header {
            modcod: ModCod::from_index(14).unwrap(),
            frame: FecFrame::Normal,
            pilots: true,
        };
        let (_, iq) = air(header, 4, 12.0, 385e3, 16.0);
        let c = crate::dvbs2::estimate(&iq, 20e6).expect("a carrier");
        assert!((c.symbol_rate / 14.25e6 - 1.0).abs() < 50e-6, "symbol rate {}", c.symbol_rate);
        assert!((c.offset_hz - 385e3).abs() < 150e3, "offset {}", c.offset_hz);

        let mut noise = Noise(5);
        let symbols: Vec<C32> = (0..3)
            .flat_map(|_| {
                tx::frame(
                    header,
                    &vec![0u8; 64_800]
                        .iter()
                        .map(|_| (noise.uniform() < 0.5) as u8)
                        .collect::<Vec<_>>(),
                    0,
                )
            })
            .collect();
        let narrow = tx::shape(&symbols, 4, 0.35);
        let with_noise: Vec<C32> = narrow
            .iter()
            .enumerate()
            .map(|(n, x)| x * rot(-2.0 * PI * 3e6 * n as f64 / 20e6) + noise.gauss() * 0.1)
            .collect();
        let c = crate::dvbs2::estimate(&with_noise, 20e6).expect("a narrow carrier");
        assert!((c.symbol_rate / 5e6 - 1.0).abs() < 50e-6, "symbol rate {}", c.symbol_rate);
        assert!((c.offset_hz + 3e6).abs() < 150e3, "offset {}", c.offset_hz);
    }

    #[test]
    fn a_centre_spur_is_found_to_within_a_hundred_hertz() {
        let header = Header {
            modcod: ModCod::from_index(14).unwrap(),
            frame: FecFrame::Normal,
            pilots: true,
        };
        let (_, mut iq) = air(header, 4, 12.0, 385e3, 0.0);
        let power = iq.iter().map(|x| x.norm_sqr()).sum::<f32>() / iq.len() as f32;
        for (n, x) in iq.iter_mut().enumerate() {
            *x += rot(2.0 * PI * -1_234_567.0 * n as f64 / 20e6) * (0.05 * power).sqrt();
        }
        let found = crate::dvbs2::acquire::spurs(&iq, 20e6);
        assert_eq!(found.len(), 1, "{found:?}");
        assert!((found[0] + 1_234_567.0).abs() < 100.0, "spur at {}", found[0]);
        let (_, clean) = air(header, 4, 12.0, 385e3, 0.0);
        assert_eq!(crate::dvbs2::acquire::spurs(&clean, 20e6), Vec::<f64>::new());
    }

    #[test]
    fn two_spurs_beating_inside_the_carrier_width_are_not_taken_for_its_symbol_rate() {
        let header = Header {
            modcod: ModCod::from_index(14).unwrap(),
            frame: FecFrame::Normal,
            pilots: true,
        };
        let mut noise = Noise(8);
        let bits: Vec<u8> = (0..64_800).map(|_| (noise.uniform() < 0.5) as u8).collect();
        let symbols: Vec<C32> = (0..2).flat_map(|_| tx::frame(header, &bits, 0)).collect();
        let shaped = tx::shape(&symbols, 4, 0.25);
        let power = shaped.iter().map(|x| x.norm_sqr()).sum::<f32>() / shaped.len() as f32;
        let a = (0.3 * power).sqrt();
        let iq: Vec<C32> = shaped
            .iter()
            .enumerate()
            .map(|(n, x)| {
                x + C32::new(a, 0.0)
                    + rot(2.0 * PI * 7.5e6 * n as f64 / 40e6) * a
                    + noise.gauss() * 0.05
            })
            .collect();
        let c = crate::dvbs2::estimate(&iq, 40e6).expect("a carrier");
        assert!(
            (c.symbol_rate / 10e6 - 1.0).abs() < 50e-6,
            "symbol rate {}, the spurs beat at 7.5 MHz",
            c.symbol_rate
        );
    }
}
