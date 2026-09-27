mod header_table;
mod uplink_table;

use common::C32;
use rayon::prelude::*;
use rustfft::{Fft, FftPlanner};
use std::sync::{Arc, OnceLock};

pub const RATE: f64 = 22_400_000.0;
pub const FFT: usize = 2048;
pub const CP: usize = 256;
pub const SYMBOL: usize = FFT + CP;
pub const SPACING_HZ: f64 = RATE / FFT as f64;
pub const CARRIERS: usize = 1729;
pub const DC: usize = CARRIERS / 2;
pub const SYMBOLS: usize = 50;
pub const HEADER_SYMBOLS: usize = 2;
pub const PREAMBLE: usize = 2 * FFT;
pub const FRAME: usize = PREAMBLE + SYMBOLS * SYMBOL;
pub const FRAME_PERIOD_S: f64 = 0.007163;
pub const DATA_CARRIERS: usize = 1536;
pub const WIDTH_HZ: f64 = 20_000_000.0;
pub const OCCUPIED_HALF_HZ: f64 = (DC as f64 + 0.5) * SPACING_HZ;
pub const REPORT_S: f64 = 1.0;

pub type HeaderBit = (u16, u8);
pub type SyncBit = (u16, u8, u8);
pub const UPLINK_SYMBOLS: usize = 6;
pub const UPLINK: usize = PREAMBLE + UPLINK_SYMBOLS * SYMBOL;
const UPLINK_SEARCH: i64 = 12;
const UPLINK_SYNC: f32 = 0.85;
const COUNTER_PERIOD: u8 = 64;
const COUNTER_MARGIN: f32 = 0.1;

const PILOT_SHIFT: [usize; 6] = [0, 3, 4, 1, 5, 2];
const PILOT_PERIOD: usize = 2047;
const PILOT_PHASE: usize = 874;
const PILOT_EPOCH: usize = 32;
const WINDOW_LEAD: usize = 64;
const TIMING_SEARCH: i64 = 48;
const OFFSET_SEARCH: i64 = 40;
const PREAMBLE_SCORE: f32 = 0.5;
const PILOT_COHERENCE: f32 = 0.5;
const DEMOD_EVERY: u32 = 4;
const PASSBAND_HZ: f64 = 10_800_000.0;
const MAX_DENOMINATOR: usize = 512;

fn pilot_sequence() -> &'static [f32; PILOT_PERIOD] {
    static SEQ: OnceLock<[f32; PILOT_PERIOD]> = OnceLock::new();
    SEQ.get_or_init(|| {
        let mut w = [1u8; PILOT_PERIOD];
        for i in 11..PILOT_PERIOD {
            w[i] = w[i - 2] ^ w[i - 11];
        }
        w.map(|b| 1.0 - 2.0 * f32::from(b))
    })
}

pub fn pilot_antenna(symbol: usize, n: usize) -> Option<usize> {
    if n >= CARRIERS || n % 3 != 2 {
        return None;
    }
    let phase = ((n - 2) / 3 + 4) % 6;
    let first = PILOT_SHIFT[symbol % 6];
    if phase == first {
        Some(0)
    } else if phase == (first + 3) % 6 {
        Some(1)
    } else {
        None
    }
}

pub fn pilot_value(symbol: usize, n: usize) -> f32 {
    pilot_sequence()[(n + PILOT_PHASE + PILOT_PERIOD - symbol % PILOT_EPOCH) % PILOT_PERIOD]
}

pub fn carries_data(symbol: usize, n: usize) -> bool {
    n < CARRIERS && n != DC && pilot_antenna(symbol, n).is_none()
}

struct Layout {
    pilots: [[Vec<usize>; 2]; 6],
    data: [Vec<usize>; 6],
}

fn layout() -> &'static Layout {
    static LAYOUT: OnceLock<Layout> = OnceLock::new();
    LAYOUT.get_or_init(|| Layout {
        pilots: std::array::from_fn(|s| {
            std::array::from_fn(|a| {
                (0..CARRIERS).filter(|&n| pilot_antenna(s, n) == Some(a)).collect()
            })
        }),
        data: std::array::from_fn(|s| (0..CARRIERS).filter(|&n| carries_data(s, n)).collect()),
    })
}

pub fn pilots(symbol: usize, antenna: usize) -> &'static [usize] {
    &layout().pilots[symbol % 6][antenna]
}

pub fn data_carriers(symbol: usize) -> &'static [usize] {
    &layout().data[symbol % 6]
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Constellation {
    Bpsk,
    Qpsk,
    Qam16,
    Qam64,
}

impl Constellation {
    pub const ALL: [Constellation; 4] =
        [Constellation::Bpsk, Constellation::Qpsk, Constellation::Qam16, Constellation::Qam64];

    fn levels(self) -> usize {
        match self {
            Constellation::Bpsk | Constellation::Qpsk => 2,
            Constellation::Qam16 => 4,
            Constellation::Qam64 => 8,
        }
    }

    pub fn nearest(self, z: C32) -> C32 {
        let l = self.levels();
        let scale = match self {
            Constellation::Bpsk => 1.0,
            _ => ((2 * (l * l - 1)) as f32 / 3.0).sqrt(),
        };
        let axis = |x: f32| {
            let i = ((x * scale + (l - 1) as f32) / 2.0).round().clamp(0.0, (l - 1) as f32);
            (2.0 * i - (l - 1) as f32) / scale
        };
        match self {
            Constellation::Bpsk => C32::new(axis(z.re), 0.0),
            _ => C32::new(axis(z.re), axis(z.im)),
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Measured {
    pub offset_hz: f64,
    pub snr_db: f32,
    pub coherence: f32,
    pub demodulated: Option<Cells>,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Cells {
    pub constellation: Constellation,
    pub mer_db: f32,
    pub balance_db: f32,
    pub counter: Option<u8>,
}

pub fn counter_template(bits: &[HeaderBit], counter: u8) -> impl Iterator<Item = (usize, u8)> + '_ {
    bits.iter().map(move |&(n, mask)| {
        let word = counter & (mask & 0x3f) | (mask & 0x40);
        (usize::from(n), (word.count_ones() & 1) as u8)
    })
}

fn counter_of(bits: &[(usize, [u8; 2])]) -> Option<u8> {
    let tables = [header_table::HEADER_ANTENNA_0, header_table::HEADER_ANTENNA_1];
    let mut best = [(usize::MAX, 0u8); 2];
    let mut second = [usize::MAX; 2];
    for (a, table) in tables.iter().enumerate() {
        for c in 0..COUNTER_PERIOD {
            let d = counter_template(table, c)
                .filter(|&(n, want)| {
                    bits.binary_search_by_key(&n, |b| b.0)
                        .map(|i| bits[i].1[a] != want)
                        .unwrap_or(false)
                })
                .count();
            if d < best[a].0 {
                second[a] = best[a].0;
                best[a] = (d, c);
            } else if d < second[a] {
                second[a] = d;
            }
        }
    }
    let margin = |a: usize| (second[a] - best[a].0) as f32 / tables[a].len() as f32;
    let sure: Vec<u8> =
        (0..2).filter(|&a| margin(a) >= COUNTER_MARGIN).map(|a| best[a].1).collect();
    match sure.as_slice() {
        [c] => Some(*c),
        [a, b] if a == b => Some(*a),
        _ => None,
    }
}

pub struct Demodulator {
    fft: Arc<dyn Fft<f32>>,
    scratch: Vec<rustfft::num_complex::Complex<f32>>,
    ramp: Vec<C32>,
}

impl Default for Demodulator {
    fn default() -> Self {
        Self::new()
    }
}

impl Demodulator {
    pub fn new() -> Self {
        let fft = FftPlanner::new().plan_fft_forward(FFT);
        let ramp = (0..FFT)
            .map(|i| {
                let k = if i < FFT / 2 { i as f64 } else { i as f64 - FFT as f64 };
                let ph = std::f64::consts::TAU * k * WINDOW_LEAD as f64 / FFT as f64;
                C32::new(ph.cos() as f32, ph.sin() as f32)
            })
            .collect();
        Self { fft, scratch: vec![Default::default(); FFT], ramp }
    }

    pub fn frame_len() -> usize {
        FRAME + 2 * TIMING_SEARCH as usize + WINDOW_LEAD
    }

    pub fn measure(&mut self, y: &[C32], preamble: usize, demodulate: bool) -> Option<Measured> {
        let lead = TIMING_SEARCH as usize;
        let from = preamble.checked_sub(lead)?;
        let to = from + Self::frame_len();
        if to > y.len() {
            return None;
        }
        let frac = fractional_offset(y, preamble);
        let mut z = y[from..to].to_vec();
        crate::Mixer::new(-frac, RATE).process_in_place(&mut z);
        let first = best_timing(&z, lead + PREAMBLE);
        let snr_db = cp_snr_db(&z, first);
        let spectra = |this: &mut Self, range: std::ops::Range<usize>| -> Vec<Vec<C32>> {
            range.map(|q| this.spectrum(&z[first + q * SYMBOL + CP - WINDOW_LEAD..])).collect()
        };
        let head = spectra(self, 0..6);
        let (dc, coherence) = carrier_offset(&head);
        let offset_hz = frac + dc as f64 * SPACING_HZ;
        if coherence < PILOT_COHERENCE {
            let (shift, sync) = uplink_sync(&head);
            if sync >= UPLINK_SYNC {
                return Some(Measured {
                    offset_hz: frac + shift as f64 * SPACING_HZ,
                    snr_db,
                    coherence: -sync,
                    demodulated: None,
                });
            }
        }
        if coherence < PILOT_COHERENCE || !demodulate {
            return Some(Measured { offset_hz, snr_db, coherence, demodulated: None });
        }
        let mut all = head;
        all.extend(spectra(self, 6..SYMBOLS));
        let demodulated = alamouti_cells(&all, dc);
        Some(Measured { offset_hz, snr_db, coherence, demodulated })
    }

    fn spectrum(&mut self, window: &[C32]) -> Vec<C32> {
        for (s, x) in self.scratch.iter_mut().zip(&window[..FFT]) {
            *s = rustfft::num_complex::Complex::new(x.re, x.im);
        }
        self.fft.process(&mut self.scratch);
        self.scratch.iter().zip(&self.ramp).map(|(s, r)| C32::new(s.re, s.im) * r).collect()
    }
}

fn uplink_sync(head: &[Vec<C32>]) -> (i64, f32) {
    let steps: Vec<Vec<C32>> = (3..5)
        .map(|q| head[q + 1].iter().zip(&head[q]).map(|(a, b)| a * b.conj()).collect())
        .collect();
    (-UPLINK_SEARCH..=UPLINK_SEARCH)
        .map(|shift| {
            let (mut agree, mut total) = (0usize, 0usize);
            for &(n, b3, b4) in uplink_table::UPLINK_SYNC {
                let k = bin(shift, usize::from(n));
                for (step, want) in [(0, b3), (1, b4)] {
                    let got = u8::from(steps[step][k].im < 0.0);
                    agree += usize::from(got == want);
                    total += 1;
                }
            }
            let frac = agree as f32 / total.max(1) as f32;
            (shift, frac.max(1.0 - frac))
        })
        .max_by(|a, b| a.1.total_cmp(&b.1))
        .unwrap_or((0, 0.0))
}

fn bin(dc: i64, n: usize) -> usize {
    (dc + n as i64 - DC as i64).rem_euclid(FFT as i64) as usize
}

fn fractional_offset(y: &[C32], preamble: usize) -> f64 {
    let s: C32 = (preamble..preamble + FFT).map(|i| y[i + FFT] * y[i].conj()).sum();
    f64::from(s.arg()) * RATE / (std::f64::consts::TAU * FFT as f64)
}

fn cp_correlation(z: &[C32], at: usize) -> (C32, f32, f32) {
    let (mut c, mut a, mut b) = (C32::default(), 0.0f32, 0.0f32);
    for i in at..at + CP {
        c += z[i + FFT] * z[i].conj();
        a += z[i].norm_sqr();
        b += z[i + FFT].norm_sqr();
    }
    (c, a, b)
}

fn best_timing(z: &[C32], nominal: usize) -> usize {
    (-TIMING_SEARCH..=TIMING_SEARCH)
        .map(|d| (nominal as i64 + d) as usize)
        .max_by(|&a, &b| {
            let score =
                |t: usize| (0..8).map(|q| cp_correlation(z, t + q * SYMBOL).0.norm()).sum::<f32>();
            score(a).total_cmp(&score(b))
        })
        .unwrap_or(nominal)
}

fn cp_snr_db(z: &[C32], first: usize) -> f32 {
    let lead = CP / 2;
    let (mut c, mut a, mut b) = (0.0f32, 0.0f32, 0.0f32);
    for q in 0..SYMBOLS {
        let at = first + q * SYMBOL + lead;
        let (mut cq, mut aq, mut bq) = (C32::default(), 0.0, 0.0);
        for i in at..at + CP - lead {
            cq += z[i + FFT] * z[i].conj();
            aq += z[i].norm_sqr();
            bq += z[i + FFT].norm_sqr();
        }
        c += cq.norm();
        a += aq;
        b += bq;
    }
    let rho = (c / (a * b).sqrt().max(1e-20)).clamp(0.0, 0.9999);
    10.0 * (rho / (1.0 - rho)).log10()
}

fn carrier_offset(head: &[Vec<C32>]) -> (i64, f32) {
    (-OFFSET_SEARCH..=OFFSET_SEARCH)
        .map(|dc| {
            let (mut num, mut den) = (0.0f32, 0.0f32);
            for (s, spec) in head.iter().enumerate() {
                for a in 0..2 {
                    let h: Vec<C32> = pilots(s, a)
                        .iter()
                        .map(|&n| spec[bin(dc, n)] * pilot_value(s, n))
                        .collect();
                    num += h.windows(2).map(|w| w[1] * w[0].conj()).sum::<C32>().norm();
                    den += h.iter().map(|x| x.norm_sqr()).sum::<f32>();
                }
            }
            (dc, num / den.max(1e-20))
        })
        .max_by(|a, b| a.1.total_cmp(&b.1))
        .unwrap_or((0, 0.0))
}

fn interpolate(points: &mut [(usize, C32)]) -> Vec<C32> {
    points.sort_by_key(|p| p.0);
    let mut out = vec![C32::default(); CARRIERS];
    let Some(&(first, v0)) = points.first() else { return out };
    let mut j = 0;
    for (n, o) in out.iter_mut().enumerate() {
        if n <= first {
            *o = v0;
            continue;
        }
        while j + 1 < points.len() && points[j + 1].0 < n {
            j += 1;
        }
        *o = match points.get(j + 1) {
            Some(&(n1, v1)) => {
                let (n0, v) = points[j];
                v + (v1 - v) * ((n - n0) as f32 / (n1 - n0) as f32)
            }
            None => points[j].1,
        };
    }
    out
}

fn pilot_estimates(spec: &[C32], symbol: usize, antenna: usize, dc: i64) -> Vec<(usize, C32)> {
    pilots(symbol, antenna)
        .iter()
        .map(|&n| (n, spec[bin(dc, n)] * pilot_value(symbol, n)))
        .collect()
}

fn pair_channels(spectra: &[Vec<C32>], s: usize, dc: i64) -> ([Vec<C32>; 2], C32) {
    let own: Vec<Vec<(usize, C32)>> =
        (0..2).map(|a| pilot_estimates(&spectra[s], s, a, dc)).collect();
    let next: Vec<Vec<(usize, C32)>> =
        (0..2).map(|a| pilot_estimates(&spectra[s + 1], s + 1, a, dc)).collect();
    let turn: C32 = (0..2)
        .map(|a| {
            let h0 = interpolate(&mut own[a].clone());
            let h1 = interpolate(&mut next[a].clone());
            h1.iter().zip(&h0).map(|(x, y)| x * y.conj()).sum::<C32>()
        })
        .sum();
    let back = C32::from_polar(1.0, -turn.arg());
    let h = std::array::from_fn(|a| {
        let mut pts = own[a].clone();
        pts.extend(next[a].iter().map(|&(n, v)| (n, v * back)));
        interpolate(&mut pts)
    });
    (h, back.conj())
}

fn header_bits(spectra: &[Vec<C32>], dc: i64) -> Vec<(usize, [u8; 2])> {
    let (h, _) = pair_channels(spectra, 0, dc);
    let y = &spectra[0];
    data_carriers(0)
        .iter()
        .map(|&n| {
            let (a, b) = (h[0][n], h[1][n]);
            let r = y[bin(dc, n)];
            let (mut best, mut bits) = (f32::INFINITY, [0u8; 2]);
            for (sa, sb) in [(1.0, 1.0), (1.0, -1.0), (-1.0, 1.0), (-1.0, -1.0)] {
                let e = (r - a * sa - b * sb).norm_sqr();
                if e < best {
                    best = e;
                    bits = [u8::from(sa < 0.0), u8::from(sb < 0.0)];
                }
            }
            (n, bits)
        })
        .collect()
}

fn alamouti_cells(spectra: &[Vec<C32>], dc: i64) -> Option<Cells> {
    let mut cells = Vec::with_capacity((SYMBOLS - HEADER_SYMBOLS) * DATA_CARRIERS);
    let (mut p0, mut p1) = (0.0f64, 0.0f64);
    let counter = counter_of(&header_bits(spectra, dc));
    for s in (HEADER_SYMBOLS..SYMBOLS).step_by(2) {
        let (h, fwd) = pair_channels(spectra, s, dc);
        for &n in data_carriers(s) {
            let (a, b) = (h[0][n], h[1][n]);
            let (c, d) = (a * fwd, b * fwd);
            let (y0, y1) = (spectra[s][bin(dc, n)], spectra[s + 1][bin(dc, n)]);
            let det = -a * c.conj() - b * d.conj();
            if det.norm_sqr() < 1e-30 {
                continue;
            }
            cells.push((-c.conj() * y0 - b * y1.conj()) / det);
            cells.push((-d.conj() * y0 + a * y1.conj()) / det);
            p0 += f64::from(a.norm_sqr());
            p1 += f64::from(b.norm_sqr());
        }
    }
    if cells.is_empty() {
        return None;
    }
    let rms = (cells.iter().map(|c| c.norm_sqr()).sum::<f32>() / cells.len() as f32).sqrt();
    cells.iter_mut().for_each(|c| *c /= rms.max(1e-20));
    let (constellation, err) = Constellation::ALL
        .iter()
        .map(|&k| {
            let e = cells.iter().map(|&c| (c - k.nearest(c)).norm_sqr()).sum::<f32>()
                / cells.len() as f32;
            (k, e)
        })
        .min_by(|a, b| a.1.total_cmp(&b.1))?;
    Some(Cells {
        constellation,
        mer_db: -10.0 * err.max(1e-12).log10(),
        balance_db: (10.0 * (p0 / p1.max(1e-30)).log10()) as f32,
        counter,
    })
}

pub fn preamble_scores(y: &[C32], from: usize, to: usize) -> Vec<f32> {
    if to <= from || to + 2 * FFT > y.len() {
        return Vec::new();
    }
    let len = to - from + FFT;
    let mut c = Vec::with_capacity(len + 1);
    let mut e1 = Vec::with_capacity(len + 1);
    let mut e2 = Vec::with_capacity(len + 1);
    let (mut sc, mut s1, mut s2) =
        (rustfft::num_complex::Complex::<f64>::default(), 0.0f64, 0.0f64);
    c.push(sc);
    e1.push(0.0);
    e2.push(0.0);
    for i in from..from + len {
        let (a, b) = (y[i], y[i + FFT]);
        let p = b * a.conj();
        sc += rustfft::num_complex::Complex::new(f64::from(p.re), f64::from(p.im));
        s1 += f64::from(a.norm_sqr());
        s2 += f64::from(b.norm_sqr());
        c.push(sc);
        e1.push(s1);
        e2.push(s2);
    }
    (0..to - from)
        .map(|p| {
            let s = c[p + FFT] - c[p];
            let d = (e1[p + FFT] - e1[p]) * (e2[p + FFT] - e2[p]);
            (s.norm_sqr() / d.max(1e-30)) as f32
        })
        .collect()
}

#[derive(Clone, Debug)]
pub struct Report {
    pub center_hz: f64,
    pub frames: u32,
    pub seconds: f64,
    pub snr_db: f32,
    pub rssi_dbfs: f32,
    pub offset_hz: f64,
    pub period_s: Option<f64>,
    pub constellation: Option<Constellation>,
    pub mer_db: Option<f32>,
    pub balance_db: Option<f32>,
    pub counter: Option<u8>,
    pub missed: u32,
    pub uplinks: u32,
    pub uplink_offset_hz: Option<f64>,
    pub samples: Vec<C32>,
    pub rate: f64,
}

#[derive(Default)]
struct Tally {
    frames: u32,
    first: Option<u64>,
    last: u64,
    snr: f64,
    rssi: f64,
    offset: f64,
    kinds: Vec<(Constellation, u32)>,
    mer: f64,
    balance: f64,
    demodulated: u32,
    counter: Option<u8>,
    missed: u32,
    uplinks: u32,
    uplink_offset: f64,
    samples: Vec<C32>,
}

impl Tally {
    fn add(&mut self, m: &Measured, at: u64, rssi: f32, samples: &[C32], stride: u32) {
        self.frames += 1;
        self.first.get_or_insert(at);
        self.last = at;
        self.snr += f64::from(m.snr_db);
        self.rssi += f64::from(rssi);
        self.offset += m.offset_hz;
        if let Some(c) = m.demodulated {
            self.demodulated += 1;
            self.mer += f64::from(c.mer_db);
            self.balance += f64::from(c.balance_db);
            if let (Some(now), Some(before)) = (c.counter, self.counter) {
                let step = u32::from(now.wrapping_sub(before) % COUNTER_PERIOD);
                self.missed += step.saturating_sub(stride);
            }
            self.counter = c.counter;
            match self.kinds.iter_mut().find(|k| k.0 == c.constellation) {
                Some(k) => k.1 += 1,
                None => self.kinds.push((c.constellation, 1)),
            }
            self.samples.clear();
            self.samples.extend_from_slice(samples);
        }
    }

    fn add_uplink(&mut self, m: &Measured) {
        self.uplinks += 1;
        self.uplink_offset += m.offset_hz;
    }

    fn report(&mut self, center_hz: f64, seconds: f64) -> Option<Report> {
        let t = std::mem::take(self);
        if t.frames == 0 && t.uplinks == 0 {
            return None;
        }
        let n = f64::from(t.frames.max(1));
        let d = f64::from(t.demodulated.max(1));
        let demodulated = t.demodulated > 0;
        Some(Report {
            center_hz,
            frames: t.frames,
            seconds,
            snr_db: (t.snr / n) as f32,
            rssi_dbfs: (t.rssi / n) as f32,
            offset_hz: t.offset / n,
            period_s: t
                .first
                .filter(|_| t.frames > 1)
                .map(|f| (t.last - f) as f64 / RATE / f64::from(t.frames - 1)),
            constellation: t.kinds.iter().max_by_key(|k| k.1).map(|k| k.0),
            mer_db: demodulated.then_some((t.mer / d) as f32),
            balance_db: demodulated.then_some((t.balance / d) as f32),
            counter: t.counter,
            missed: t.missed,
            uplinks: t.uplinks,
            uplink_offset_hz: (t.uplinks > 0).then(|| t.uplink_offset / f64::from(t.uplinks)),
            samples: t.samples,
            rate: RATE,
        })
    }
}

pub struct Channel {
    center_hz: f64,
    mixer: Option<crate::Mixer>,
    resampler: crate::resample::Rational,
    mixed: Vec<C32>,
    buf: Vec<C32>,
    scan: usize,
    consumed: u64,
    reported_at: u64,
    demod: Demodulator,
    tally: Tally,
    counted: u32,
    stride: u32,
    verified_at: Option<u64>,
    latest: Option<Report>,
}

impl Channel {
    pub fn new(span_rate: f64, span_hz: f64, center_hz: f64) -> Option<Self> {
        let shift = center_hz - span_hz;
        if shift.abs() + OCCUPIED_HALF_HZ > span_rate / 2.0 + 1.0 {
            return None;
        }
        Some(Self {
            center_hz,
            mixer: (shift.abs() > 0.5).then(|| crate::Mixer::new(-shift, span_rate)),
            resampler: crate::resample::Rational::approx_passband(
                span_rate,
                RATE,
                MAX_DENOMINATOR,
                PASSBAND_HZ,
            ),
            mixed: Vec::new(),
            buf: Vec::new(),
            scan: TIMING_SEARCH as usize,
            consumed: 0,
            reported_at: 0,
            demod: Demodulator::new(),
            tally: Tally::default(),
            counted: 0,
            stride: DEMOD_EVERY,
            verified_at: None,
            latest: None,
        })
    }

    pub fn center_hz(&self) -> f64 {
        self.center_hz
    }

    pub fn demodulate_every(&mut self, frames: u32) {
        self.stride = frames.max(1);
    }

    pub fn reset(&mut self) {
        self.buf.clear();
        self.scan = TIMING_SEARCH as usize;
        self.consumed = 0;
        self.reported_at = 0;
        self.tally = Tally::default();
        self.verified_at = None;
        self.latest = None;
        self.resampler.reset();
        if let Some(m) = self.mixer.as_mut() {
            m.reset();
        }
    }

    pub fn feed(&mut self, iq: &[C32], out: &mut Vec<Report>) {
        match self.mixer.as_mut() {
            Some(m) => {
                self.mixed.clear();
                m.process(iq, &mut self.mixed);
                self.resampler.process(&self.mixed, &mut self.buf);
            }
            None => self.resampler.process(iq, &mut self.buf),
        }
        self.read_frames();
        let drop = self.scan.saturating_sub(TIMING_SEARCH as usize);
        self.buf.drain(..drop);
        self.scan -= drop;
        self.consumed += drop as u64;
        if self.stream_len() - self.reported_at >= (REPORT_S * RATE) as u64 {
            self.flush(out);
        }
    }

    fn stream_len(&self) -> u64 {
        self.consumed + self.buf.len() as u64
    }

    pub fn flush(&mut self, out: &mut Vec<Report>) {
        let now = self.stream_len();
        let seconds = (now - self.reported_at) as f64 / RATE;
        self.reported_at = now;
        let Some(r) = self.tally.report(self.center_hz, seconds) else {
            self.latest = None;
            return;
        };
        self.latest = Some(Report { samples: Vec::new(), ..r.clone() });
        out.push(r);
    }

    pub fn locked(&self) -> bool {
        let window = (3.0 * FRAME_PERIOD_S * RATE) as u64;
        self.verified_at.is_some_and(|at| self.stream_len().saturating_sub(at) < window)
    }

    pub fn latest(&self) -> Option<&Report> {
        self.latest.as_ref()
    }

    fn read_frames(&mut self) {
        let end = self.buf.len().saturating_sub(2 * FFT);
        if end <= self.scan {
            return;
        }
        let base = self.scan;
        let scores = preamble_scores(&self.buf, base, end);
        let mut i = 0;
        while i < scores.len() {
            let Some(hit) = scores[i..].iter().position(|&s| s > PREAMBLE_SCORE).map(|h| h + i)
            else {
                break;
            };
            if hit + FFT > scores.len() {
                self.scan = base + hit;
                return;
            }
            let rise = scores[hit..hit + FFT].iter().enumerate().max_by(|a, b| a.1.total_cmp(b.1));
            let peak = base + hit + rise.map_or(0, |p| p.0);
            if peak + Demodulator::frame_len() > self.buf.len() {
                self.scan = base + hit;
                return;
            }
            let full = self.counted.is_multiple_of(self.stride);
            i = match self.demod.measure(&self.buf, peak, full) {
                Some(m) if m.coherence <= -UPLINK_SYNC => {
                    self.tally.add_uplink(&m);
                    peak - base + UPLINK
                }
                Some(m) if m.coherence >= PILOT_COHERENCE => {
                    let frame = &self.buf[peak..peak + FRAME];
                    let pow = frame.iter().map(|c| c.norm_sqr()).sum::<f32>() / FRAME as f32;
                    let at = self.consumed + peak as u64;
                    self.tally.add(&m, at, 10.0 * pow.max(1e-20).log10(), frame, self.stride);
                    self.counted = self.counted.wrapping_add(1);
                    self.verified_at = Some(self.consumed + (peak + FRAME) as u64);
                    peak - base + FRAME
                }
                _ => peak - base + PREAMBLE,
            };
        }
        self.scan = base + i.max(scores.len());
    }
}

pub struct Span {
    channels: Vec<Channel>,
}

impl Span {
    pub fn new(rate: f64, center_hz: f64, centers: &[f64]) -> Option<Self> {
        let channels: Vec<Channel> =
            centers.iter().filter_map(|&c| Channel::new(rate, center_hz, c)).collect();
        (!channels.is_empty()).then_some(Self { channels })
    }

    pub fn channels(&self) -> Vec<f64> {
        self.channels.iter().map(|c| c.center_hz).collect()
    }

    pub fn locked(&self) -> bool {
        self.channels.iter().any(Channel::locked)
    }

    pub fn demodulate_every(&mut self, frames: u32) {
        self.channels.iter_mut().for_each(|c| c.demodulate_every(frames));
    }

    pub fn latest(&self) -> impl Iterator<Item = &Report> {
        self.channels.iter().filter(|c| c.locked()).filter_map(Channel::latest)
    }

    pub fn reset(&mut self) {
        self.channels.iter_mut().for_each(Channel::reset);
    }

    pub fn flush(&mut self, out: &mut Vec<Report>) {
        self.channels.iter_mut().for_each(|c| c.flush(out));
    }

    pub fn process(&mut self, iq: &[C32], out: &mut Vec<Report>) {
        if iq.is_empty() {
            return;
        }
        let found: Vec<Vec<Report>> = self
            .channels
            .par_iter_mut()
            .map(|c| {
                let mut mine = Vec::new();
                c.feed(iq, &mut mine);
                mine
            })
            .collect();
        out.extend(found.into_iter().flatten());
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const MEASURED_SYMBOL_0_ANTENNA_0: &str = "+++++++++---++-+----++++--++-++-+-+-+-++--+-+++----+-++-++-+-++++-------++---++-+-+-+--+++--+-++";

    #[test]
    fn every_symbol_has_1536_data_carriers_and_96_pilots_an_antenna() {
        for s in 0..SYMBOLS {
            assert_eq!(data_carriers(s).len(), DATA_CARRIERS, "symbol {s}");
            assert_eq!(pilots(s, 0).len(), 96, "symbol {s} antenna 0");
            assert_eq!(pilots(s, 1).len(), 96, "symbol {s} antenna 1");
        }
    }

    #[test]
    fn the_two_symbols_of_a_pair_swap_antennas_on_one_set_of_carriers() {
        for s in (0..SYMBOLS).step_by(2) {
            assert_eq!(pilots(s, 0), pilots(s + 1, 1), "pair {s}");
            assert_eq!(pilots(s, 1), pilots(s + 1, 0), "pair {s}");
        }
    }

    #[test]
    fn the_pilot_formula_reproduces_the_signs_measured_off_a_walksnail_vtx() {
        let formula: String = pilots(0, 0)
            .iter()
            .filter(|&&n| (8..=1718).contains(&n))
            .map(|&n| if pilot_value(0, n) > 0.0 { '+' } else { '-' })
            .collect();
        assert_eq!(formula, MEASURED_SYMBOL_0_ANTENNA_0);
    }

    #[test]
    fn the_header_templates_decode_every_counter_through_five_percent_bit_errors() {
        let mut rng = Rng(5);
        for c in 0..COUNTER_PERIOD {
            let mut bits: Vec<(usize, [u8; 2])> = data_carriers(0)
                .iter()
                .map(|&n| (n, [rng.next() as u8 & 1, rng.next() as u8 & 1]))
                .collect();
            for (a, table) in
                [header_table::HEADER_ANTENNA_0, header_table::HEADER_ANTENNA_1].iter().enumerate()
            {
                for (n, want) in counter_template(table, c) {
                    let i = bits.binary_search_by_key(&n, |b| b.0).expect("a data carrier");
                    bits[i].1[a] = want ^ u8::from(rng.uniform() < 0.05);
                }
            }
            assert_eq!(counter_of(&bits), Some(c), "counter {c}");
        }
        let noise: Vec<(usize, [u8; 2])> = data_carriers(0)
            .iter()
            .map(|&n| (n, [rng.next() as u8 & 1, rng.next() as u8 & 1]))
            .collect();
        assert_eq!(counter_of(&noise), None, "random bits are not a counter");
    }

    #[test]
    fn a_nearest_point_is_on_the_grid_and_unit_power_on_average() {
        for k in Constellation::ALL {
            let mut pts = std::collections::HashSet::new();
            let mut power = 0.0f32;
            for i in -40..40 {
                for q in -40..40 {
                    let p = k.nearest(C32::new(i as f32 * 0.04, q as f32 * 0.04));
                    if pts.insert((p.re.to_bits(), p.im.to_bits())) {
                        power += p.norm_sqr();
                    }
                }
            }
            let expect = match k {
                Constellation::Bpsk => 2,
                Constellation::Qpsk => 4,
                Constellation::Qam16 => 16,
                Constellation::Qam64 => 64,
            };
            assert_eq!(pts.len(), expect, "{k:?}");
            assert!(
                (power / expect as f32 - 1.0).abs() < 1e-4,
                "{k:?} mean power {}",
                power / expect as f32
            );
        }
    }

    struct Rng(u64);
    impl Rng {
        fn next(&mut self) -> u64 {
            self.0 ^= self.0 << 13;
            self.0 ^= self.0 >> 7;
            self.0 ^= self.0 << 17;
            self.0
        }
        fn uniform(&mut self) -> f32 {
            (self.next() >> 40) as f32 / (1u64 << 24) as f32
        }
        fn gauss(&mut self) -> f32 {
            let (u, v) = (self.uniform().max(1e-9), self.uniform());
            (-2.0 * u.ln()).sqrt() * (std::f32::consts::TAU * v).cos()
        }
        fn qam64(&mut self) -> C32 {
            let l = |r: u64| ((r % 8) as f32 * 2.0 - 7.0) / 42f32.sqrt();
            C32::new(l(self.next()), l(self.next()))
        }
        fn sign(&mut self) -> f32 {
            if self.next() & 1 == 0 { 1.0 } else { -1.0 }
        }
    }

    fn ofdm(bins: &[C32]) -> Vec<C32> {
        let mut v: Vec<rustfft::num_complex::Complex<f32>> =
            bins.iter().map(|c| rustfft::num_complex::Complex::new(c.re, c.im)).collect();
        FftPlanner::new().plan_fft_inverse(FFT).process(&mut v);
        v.iter().map(|c| C32::new(c.re, c.im) / (FFT as f32).sqrt()).collect()
    }

    fn frame(rng: &mut Rng, dc: i64, gains: [C32; 2]) -> [Vec<C32>; 2] {
        let mut ant = [Vec::new(), Vec::new()];
        let mut pre = vec![C32::default(); FFT];
        for n in (1..CARRIERS).step_by(2) {
            pre[bin(dc, n)] = C32::new(rng.sign(), 0.0);
        }
        let pre = ofdm(&pre);
        for a in 0..2 {
            ant[a].extend(pre.iter().map(|x| x * gains[a]));
            ant[a].extend(pre.iter().map(|x| x * gains[a]));
        }
        let mut sym = [vec![C32::default(); FFT], vec![C32::default(); FFT]];
        for s in 0..SYMBOLS {
            if s % 2 == 0 {
                sym = [vec![C32::default(); FFT], vec![C32::default(); FFT]];
                let mut next = [vec![C32::default(); FFT], vec![C32::default(); FFT]];
                for n in 0..CARRIERS {
                    if !carries_data(s, n) {
                        continue;
                    }
                    let k = bin(dc, n);
                    if s < HEADER_SYMBOLS {
                        for a in 0..2 {
                            sym[a][k] = C32::new(rng.sign(), 0.0);
                            next[a][k] = C32::new(rng.sign(), 0.0);
                        }
                    } else {
                        let (x1, x2) = (rng.qam64(), rng.qam64());
                        sym[0][k] = x1;
                        sym[1][k] = x2;
                        next[0][k] = -x2.conj();
                        next[1][k] = x1.conj();
                    }
                }
                for t in [s, s + 1] {
                    for a in 0..2 {
                        for &n in pilots(t, a) {
                            let v = C32::new(pilot_value(t, n), 0.0);
                            if t == s {
                                sym[a][bin(dc, n)] = v;
                            } else {
                                next[a][bin(dc, n)] = v;
                            }
                        }
                    }
                }
                for a in 0..2 {
                    let body = ofdm(&sym[a]);
                    ant[a].extend(body[FFT - CP..].iter().chain(&body).map(|x| x * gains[a]));
                }
                sym = next;
            } else {
                for a in 0..2 {
                    let body = ofdm(&sym[a]);
                    ant[a].extend(body[FFT - CP..].iter().chain(&body).map(|x| x * gains[a]));
                }
            }
        }
        ant
    }

    fn air(
        frames: usize,
        dc: i64,
        gains: [C32; 2],
        cfo_hz: f64,
        snr_db: f32,
        seed: u64,
    ) -> Vec<C32> {
        let mut rng = Rng(seed);
        let period = (FRAME_PERIOD_S * RATE) as usize;
        let mut out = vec![C32::default(); period * frames + FFT];
        for f in 0..frames {
            let [a, b] = frame(&mut rng, dc, gains);
            let at = f * period + 3000;
            for (i, (x, y)) in a.iter().zip(&b).enumerate() {
                out[at + i] = x + y;
            }
        }
        let sigma = 10f32.powf(-snr_db / 20.0)
            * (gains[0].norm_sqr() + gains[1].norm_sqr()).sqrt()
            * (CARRIERS as f32 / FFT as f32).sqrt()
            / 2f32.sqrt();
        for (i, x) in out.iter_mut().enumerate() {
            let ph = std::f64::consts::TAU * cfo_hz * i as f64 / RATE;
            *x = *x * C32::from_polar(1.0, ph as f32) + C32::new(rng.gauss(), rng.gauss()) * sigma;
        }
        out
    }

    fn read(iq: &[C32], rate: f64) -> Vec<Report> {
        let mut ch = Channel::new(rate, 5_805e6, 5_805e6).expect("the channel fits");
        let mut out = Vec::new();
        for b in iq.chunks(100_000) {
            ch.feed(b, &mut out);
        }
        ch.flush(&mut out);
        out
    }

    #[test]
    fn a_two_antenna_frame_is_found_and_its_cells_are_64_qam() {
        let gains = [C32::new(0.8, 0.3), C32::from_polar(0.4, 2.0)];
        let iq = air(6, 3, gains, 1_300.0, 30.0, 7);
        let reports = read(&iq, RATE);
        let frames: u32 = reports.iter().map(|r| r.frames).sum();
        assert_eq!(frames, 6, "frames found of six sent");
        assert_eq!(
            reports.iter().map(|r| r.uplinks).sum::<u32>(),
            0,
            "a VTX frame is not a goggles burst"
        );
        let r = reports.iter().find(|r| r.mer_db.is_some()).expect("a demodulated frame");
        assert_eq!(r.constellation, Some(Constellation::Qam64));
        let mer = r.mer_db.unwrap();
        assert!(
            (24.0..32.0).contains(&mer),
            "MER {mer} dB against 30 dB of noise: floor 24, ceiling 32"
        );
        let want = 10.0 * (gains[0].norm_sqr() / gains[1].norm_sqr()).log10();
        let got = r.balance_db.unwrap();
        assert!((got - want).abs() < 0.5, "antenna balance {got} dB, sent {want} dB");
        let off = 3.0 * SPACING_HZ + 1_300.0;
        assert!((r.offset_hz - off).abs() < 100.0, "offset {} Hz, sent {off} Hz", r.offset_hz);
    }

    #[test]
    fn a_frame_resampled_from_20_ms_per_second_reads_the_same() {
        let gains = [C32::new(0.7, 0.0), C32::new(0.0, 0.5)];
        let iq = air(4, -2, gains, -2_000.0, 30.0, 11);
        let mut rs =
            crate::resample::Rational::approx_passband(RATE, 20e6, MAX_DENOMINATOR, PASSBAND_HZ);
        let mut twenty = Vec::new();
        rs.process(&iq, &mut twenty);
        let reports = read(&twenty, 20e6);
        let frames: u32 = reports.iter().map(|r| r.frames).sum();
        assert_eq!(frames, 4, "frames found of four sent");
        let r = reports.iter().find(|r| r.mer_db.is_some()).expect("a demodulated frame");
        assert_eq!(r.constellation, Some(Constellation::Qam64));
        let mer = r.mer_db.unwrap();
        assert!(mer > 20.0, "MER {mer} dB through two resamplers, floor 20");
    }

    #[test]
    fn noise_is_not_a_frame() {
        let mut rng = Rng(3);
        let iq: Vec<C32> =
            (0..(0.03 * RATE) as usize).map(|_| C32::new(rng.gauss(), rng.gauss())).collect();
        let reports = read(&iq, RATE);
        assert_eq!(reports.iter().map(|r| r.frames).sum::<u32>(), 0);
        assert_eq!(reports.iter().map(|r| r.uplinks).sum::<u32>(), 0);
    }
}
