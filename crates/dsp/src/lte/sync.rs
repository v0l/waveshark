use super::SUBCARRIER_HZ;
use super::grid::Numerology;
use common::C32;
use rustfft::{Fft, FftPlanner};
use std::f32::consts::PI;
use std::sync::Arc;

pub const RATE_HZ: f64 = 1.92e6;
pub const HALF_FRAME: usize = 9600;
const SEGMENT: usize = 10240;
pub const FRAME: usize = 2 * HALF_FRAME;
const FFT: usize = 128;
const ROOTS: [u32; 3] = [25, 29, 34];
pub const SHIFTS: i32 = 5;
const PER_ROOT: usize = 2;
pub const THRESHOLD: f32 = 10.0;

pub fn pss(n2: usize) -> [C32; 62] {
    let u = ROOTS[n2] as f32;
    std::array::from_fn(|n| {
        let n = n as f32;
        let phase = if n < 31.0 {
            -PI * u * n * (n + 1.0) / 63.0
        } else {
            -PI * u * (n + 1.0) * (n + 2.0) / 63.0
        };
        C32::from_polar(1.0, phase)
    })
}

fn m_sequence(taps: &[usize]) -> [f32; 31] {
    let mut x = [0u8; 31];
    x[4] = 1;
    for i in 0..26 {
        x[i + 5] = taps.iter().fold(0, |a, &t| a ^ x[i + t]);
    }
    x.map(|b| 1.0 - 2.0 * f32::from(b))
}

pub fn sss(n1: usize, n2: usize, second_half: bool) -> [f32; 62] {
    let (s, c, z) = (m_sequence(&[2, 0]), m_sequence(&[3, 0]), m_sequence(&[4, 2, 1, 0]));
    let q1 = n1 / 30;
    let q = (n1 + q1 * (q1 + 1) / 2) / 30;
    let m = n1 + q * (q + 1) / 2;
    let m0 = m % 31;
    let m1 = (m0 + m / 31 + 1) % 31;
    let mut d = [0f32; 62];
    for n in 0..31 {
        let s0 = s[(n + m0) % 31];
        let s1 = s[(n + m1) % 31];
        let c0 = c[(n + n2) % 31];
        let c1 = c[(n + n2 + 3) % 31];
        let z0 = z[(n + m0 % 8) % 31];
        let z1 = z[(n + m1 % 8) % 31];
        if second_half {
            d[2 * n] = s1 * c0;
            d[2 * n + 1] = s0 * c1 * z1;
        } else {
            d[2 * n] = s0 * c0;
            d[2 * n + 1] = s1 * c1 * z0;
        }
    }
    d
}

fn centre_bin(n: usize, shift: i32) -> usize {
    let k = if n < 31 { n as i32 - 31 } else { n as i32 - 30 };
    (k + shift).rem_euclid(FFT as i32) as usize
}

pub fn fractional_cfo_hz(y: &[C32]) -> f64 {
    let slot = Numerology::SYNC.slot();
    let cp = Numerology::SYNC.cp(1);
    if y.len() < FFT + slot {
        return 0.0;
    }
    let mut folded = vec![C32::default(); slot];
    for (n, (a, b)) in y.iter().zip(&y[FFT..]).enumerate() {
        folded[n % slot] += a * b.conj();
    }
    let window: Vec<C32> =
        (0..slot).map(|j| (0..cp).map(|i| folded[(j + i) % slot]).sum()).collect();
    let mean = window.iter().sum::<C32>() / slot as f32;
    let peak = window.iter().map(|w| w - mean).max_by(|a, b| a.norm_sqr().total_cmp(&b.norm_sqr()));
    peak.map_or(0.0, |p| -f64::from(p.arg()) * SUBCARRIER_HZ / (2.0 * std::f64::consts::PI))
}

pub fn occupied_centre_hz(iq: &[C32], rate_hz: f64) -> f64 {
    const N: usize = 1024;
    const SMOOTH: usize = 8;
    let fft = FftPlanner::<f32>::new().plan_fft_forward(N);
    let mut psd = vec![0f32; N];
    let mut buf = vec![C32::default(); N];
    for chunk in iq.chunks_exact(N).step_by(16).take(256) {
        buf.copy_from_slice(chunk);
        fft.process(&mut buf);
        for (p, v) in psd.iter_mut().zip(&buf) {
            *p += v.norm_sqr();
        }
    }
    psd.rotate_left(N / 2);
    let db: Vec<f32> = (0..N)
        .map(|i| {
            let (lo, hi) = (i.saturating_sub(SMOOTH), (i + SMOOTH + 1).min(N));
            10.0 * (psd[lo..hi].iter().sum::<f32>() / (hi - lo) as f32).max(1e-30).log10()
        })
        .collect();
    let mut sorted = db.clone();
    sorted.sort_by(f32::total_cmp);
    let threshold = (sorted[N / 10] + sorted[N - 1]) / 2.0;
    let (mut best, mut run) = ((0, 0), None::<usize>);
    let above = db.iter().map(|v| *v >= threshold).chain(std::iter::once(false));
    for (i, up) in above.enumerate() {
        match (up, run) {
            (true, None) => run = Some(i),
            (false, Some(start)) => {
                if i - start > best.1 - best.0 {
                    best = (start, i);
                }
                run = None;
            }
            _ => {}
        }
    }
    let mid = (best.0 + best.1) as f64 / 2.0;
    (mid - N as f64 / 2.0) * rate_hz / N as f64
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Found {
    pub pci: u16,
    pub offset_hz: f64,
    pub frame_start: usize,
    pub strength: f32,
}

pub struct Searcher {
    fwd: Arc<dyn Fft<f32>>,
    inv: Arc<dyn Fft<f32>>,
    small: Arc<dyn Fft<f32>>,
    templates: Vec<(usize, i32, Vec<C32>)>,
}

impl Default for Searcher {
    fn default() -> Self {
        Self::new()
    }
}

impl Searcher {
    pub fn new() -> Self {
        let mut planner = FftPlanner::<f32>::new();
        let fwd = planner.plan_fft_forward(SEGMENT);
        let inv = planner.plan_fft_inverse(SEGMENT);
        let small = planner.plan_fft_forward(FFT);
        let inv_small = planner.plan_fft_inverse(FFT);
        let mut templates = Vec::new();
        for n2 in 0..3 {
            for shift in -SHIFTS..=SHIFTS {
                let mut bins = vec![C32::default(); FFT];
                for (n, d) in pss(n2).iter().enumerate() {
                    bins[centre_bin(n, shift)] = *d;
                }
                inv_small.process(&mut bins);
                let mut padded = vec![C32::default(); SEGMENT];
                padded[..FFT].copy_from_slice(&bins);
                fwd.process(&mut padded);
                padded.iter_mut().for_each(|v| *v = v.conj());
                templates.push((n2, shift, padded));
            }
        }
        Self { fwd, inv, small, templates }
    }

    pub fn search(&self, y: &[C32]) -> Vec<Found> {
        let halves = y.len().saturating_sub(FFT) / HALF_FRAME;
        if halves < 2 {
            return Vec::new();
        }
        let spectra: Vec<Vec<C32>> = (0..halves)
            .map(|i| {
                let mut seg = vec![C32::default(); SEGMENT];
                let from = i * HALF_FRAME;
                let to = (from + HALF_FRAME + FFT).min(y.len());
                seg[..to - from].copy_from_slice(&y[from..to]);
                self.fwd.process(&mut seg);
                seg
            })
            .collect();
        let mut heard: Vec<(usize, f32, i32, usize)> = Vec::new();
        let mut corr = vec![C32::default(); SEGMENT];
        let mut power = vec![0f32; HALF_FRAME];
        for (n2, shift, t) in &self.templates {
            power.iter_mut().for_each(|p| *p = 0.0);
            for spectrum in &spectra {
                for ((c, f), t) in corr.iter_mut().zip(spectrum).zip(t) {
                    *c = f * t;
                }
                self.inv.process(&mut corr);
                for (p, c) in power.iter_mut().zip(&corr) {
                    *p += c.norm_sqr();
                }
            }
            let mean = power.iter().sum::<f32>() / HALF_FRAME as f32;
            let (at, peak) = power
                .iter()
                .enumerate()
                .max_by(|a, b| a.1.total_cmp(b.1))
                .map_or((0, 0.0), |(i, p)| (i, *p));
            let ratio = peak / mean.max(1e-30);
            if ratio >= THRESHOLD {
                heard.push((*n2, ratio, *shift, at));
            }
        }
        heard.sort_by(|a, b| b.1.total_cmp(&a.1));
        let mut out: Vec<Found> = (0..3)
            .flat_map(|n2| heard.iter().filter(move |h| h.0 == n2).take(PER_ROOT))
            .filter_map(|&(n2, ratio, shift, at)| self.identify(y, n2, shift, at, ratio))
            .collect();
        out.sort_by(|a, b| b.strength.total_cmp(&a.strength));
        out
    }

    fn identify(
        &self,
        y: &[C32],
        n2: usize,
        shift: i32,
        pss_at: usize,
        strength: f32,
    ) -> Option<Found> {
        let mut centred = Vec::with_capacity(y.len());
        crate::mixer::Mixer::new(-f64::from(shift) * SUBCARRIER_HZ, RATE_HZ)
            .process(y, &mut centred);
        let y = &centred;
        let back = FFT + Numerology::SYNC.cp(1);
        let d = pss(n2);
        let mut score = vec![[0f32; 2]; 168];
        let mut used = 0;
        let mut at = pss_at;
        let mut half = 0usize;
        while at + FFT <= y.len() {
            if at >= back {
                let p = self.bins(&y[at..at + FFT]);
                let s = self.bins(&y[at - back..at - back + FFT]);
                let e: Vec<C32> = (0..62).map(|n| s[n] * (p[n] * d[n].conj()).conj()).collect();
                for (n1, row) in score.iter_mut().enumerate() {
                    for (h, cell) in row.iter_mut().enumerate() {
                        let seq = sss(n1, n2, (half + h) % 2 == 1);
                        *cell += e.iter().zip(&seq).map(|(x, s)| x.re * s).sum::<f32>();
                    }
                }
                used += 1;
            }
            at += HALF_FRAME;
            half += 1;
        }
        if used == 0 {
            return None;
        }
        let (n1, h, _) = score
            .iter()
            .enumerate()
            .flat_map(|(n1, row)| row.iter().enumerate().map(move |(h, v)| (n1, h, *v)))
            .max_by(|a, b| a.2.total_cmp(&b.2))?;
        let pss0 = pss_at + h * HALF_FRAME;
        let frame_start = (pss0 + FRAME - Numerology::SYNC.symbol(6)) % FRAME;
        Some(Found {
            pci: (3 * n1 + n2) as u16,
            offset_hz: f64::from(shift) * SUBCARRIER_HZ,
            frame_start,
            strength,
        })
    }

    fn bins(&self, x: &[C32]) -> [C32; 62] {
        let mut buf = x.to_vec();
        self.small.process(&mut buf);
        std::array::from_fn(|n| buf[centre_bin(n, 0)])
    }
}
