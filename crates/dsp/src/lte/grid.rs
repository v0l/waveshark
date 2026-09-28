use super::{MAX_RB, SUBCARRIER_HZ, SUBCARRIERS_PER_RB, SYMBOLS_PER_SUBFRAME, gold};
use common::C32;
use rustfft::{Fft, FftPlanner};
use std::sync::Arc;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Numerology {
    pub fft: usize,
}

impl Numerology {
    pub const SYNC: Numerology = Numerology { fft: 128 };

    pub fn for_rate(rate: f64) -> Option<Self> {
        let fft = (rate / SUBCARRIER_HZ).round() as usize;
        let exact = (fft as f64 * SUBCARRIER_HZ - rate).abs() < 1.0;
        (exact && fft.is_power_of_two() && (128..=2048).contains(&fft)).then_some(Self { fft })
    }

    pub fn rate(self) -> f64 {
        self.fft as f64 * SUBCARRIER_HZ
    }

    pub fn subframe(self) -> usize {
        self.fft * 15
    }

    pub fn slot(self) -> usize {
        self.subframe() / 2
    }

    pub fn frame(self) -> usize {
        self.subframe() * 10
    }

    pub fn cp(self, l: usize) -> usize {
        let at_2048 = if l.is_multiple_of(7) { 160 } else { 144 };
        at_2048 * self.fft / 2048
    }

    pub fn symbol(self, l: usize) -> usize {
        (l / 7) * self.slot() + self.cp(0) + (l % 7) * (self.fft + self.cp(1))
    }
}

pub struct Grid {
    pub nrb: usize,
    fft: usize,
    re: Vec<C32>,
}

impl Grid {
    pub fn nsc(&self) -> usize {
        self.nrb * SUBCARRIERS_PER_RB
    }

    pub fn at(&self, l: usize, k: usize) -> C32 {
        self.re[l * self.nsc() + k]
    }

    pub fn fft(&self) -> usize {
        self.fft
    }
}

fn signed_bin(k: usize, nsc: usize) -> isize {
    let half = nsc / 2;
    if k < half { k as isize - half as isize } else { (k - half) as isize + 1 }
}

pub struct Demodulator {
    num: Numerology,
    fft: Arc<dyn Fft<f32>>,
    buf: Vec<C32>,
}

impl Demodulator {
    pub fn new(num: Numerology) -> Self {
        let fft = FftPlanner::<f32>::new().plan_fft_forward(num.fft);
        Self { num, fft, buf: vec![C32::default(); num.fft] }
    }

    pub fn numerology(&self) -> Numerology {
        self.num
    }

    pub fn subframe(&mut self, iq: &[C32], start: usize, nrb: usize) -> Option<Grid> {
        let (n, nsc) = (self.num.fft, nrb * SUBCARRIERS_PER_RB);
        if nsc >= n {
            return None;
        }
        let early = self.num.cp(1) / 4;
        let turn: Vec<C32> = (0..nsc)
            .map(|k| {
                let phase =
                    2.0 * std::f32::consts::PI * (signed_bin(k, nsc) * early as isize) as f32
                        / n as f32;
                C32::from_polar(1.0, phase)
            })
            .collect();
        let mut re = Vec::with_capacity(SYMBOLS_PER_SUBFRAME * nsc);
        for l in 0..SYMBOLS_PER_SUBFRAME {
            let at = start + self.num.symbol(l) - early;
            self.buf.copy_from_slice(iq.get(at..at + n)?);
            self.fft.process(&mut self.buf);
            re.extend(
                (0..nsc).map(|k| {
                    self.buf[signed_bin(k, nsc).rem_euclid(n as isize) as usize] * turn[k]
                }),
            );
        }
        Some(Grid { nrb, fft: n, re })
    }
}

pub fn crs(ns: usize, l: usize, pci: u16, nrb: usize) -> Vec<C32> {
    let pci = u32::from(pci);
    let c_init = (1 << 10) * (7 * (ns as u32 + 1) + l as u32 + 1) * (2 * pci + 1) + 2 * pci + 1;
    let c = gold::sequence(4 * MAX_RB, c_init);
    let s = std::f32::consts::FRAC_1_SQRT_2;
    let level = |b: u8| if b == 0 { s } else { -s };
    (MAX_RB - nrb..MAX_RB + nrb).map(|m| C32::new(level(c[2 * m]), level(c[2 * m + 1]))).collect()
}

pub fn crs_offset(port: usize, l: usize) -> Option<usize> {
    let odd_slot = l / 7;
    match (port, l % 7) {
        (0, 0) | (1, 4) => Some(0),
        (0, 4) | (1, 0) => Some(3),
        (2, 1) => Some(3 * odd_slot),
        (3, 1) => Some(3 + 3 * odd_slot),
        _ => None,
    }
}

pub fn is_crs(l: usize, k: usize, pci: u16, ports: usize) -> bool {
    let shift = usize::from(pci) % 6;
    (0..ports).any(|p| crs_offset(p, l).is_some_and(|v| k % 6 == (v + shift) % 6))
}

pub struct Channel {
    nsc: usize,
    fft: usize,
    h: Vec<C32>,
    pub snr_db: f32,
    pub reference_power: f32,
    pub symbol_power: f32,
}

impl Channel {
    pub fn rsrp_dbfs(&self) -> f32 {
        let scale = (self.fft * self.fft) as f32;
        (10.0 * (self.reference_power / scale).max(1e-30).log10()).max(-200.0)
    }

    pub fn cell_dbfs(&self, nrb: usize) -> f32 {
        self.rsrp_dbfs() + 10.0 * ((nrb * SUBCARRIERS_PER_RB) as f32).log10()
    }

    pub fn rsrq_db(&self) -> f32 {
        let per_rb = SUBCARRIERS_PER_RB as f32 * self.symbol_power;
        (10.0 * (self.reference_power / per_rb.max(1e-30)).max(1e-6).log10()).clamp(-40.0, 10.0)
    }

    pub fn at(&self, port: usize, l: usize, k: usize) -> C32 {
        self.h[(port * SYMBOLS_PER_SUBFRAME + l) * self.nsc + k]
    }
}

const SMOOTH: usize = 2;

pub fn estimate(grid: &Grid, subframe: usize, pci: u16, ports: usize) -> Channel {
    let (nrb, nsc) = (grid.nrb, grid.nsc());
    let shift = usize::from(pci) % 6;
    let mut h = vec![C32::default(); ports * SYMBOLS_PER_SUBFRAME * nsc];
    let (mut signal, mut noise) = (0f32, 0f32);
    let (mut reference, mut pilots, mut symbols, mut cells) = (0f32, 0usize, 0f32, 0usize);
    for port in 0..ports {
        let mut known: Vec<(usize, Vec<C32>)> = Vec::new();
        for l in 0..SYMBOLS_PER_SUBFRAME {
            let Some(v) = crs_offset(port, l) else { continue };
            let r = crs(2 * subframe + l / 7, l % 7, pci, nrb);
            let first = (v + shift) % 6;
            let raw: Vec<C32> =
                r.iter().enumerate().map(|(m, x)| grid.at(l, 6 * m + first) * x.conj()).collect();
            let smooth: Vec<C32> = (0..raw.len())
                .map(|m| {
                    let lo = m.saturating_sub(SMOOTH);
                    let hi = (m + SMOOTH + 1).min(raw.len());
                    raw[lo..hi].iter().sum::<C32>() / (hi - lo) as f32
                })
                .collect();
            if port == 0 {
                reference += raw.iter().map(|x| x.norm_sqr()).sum::<f32>();
                pilots += raw.len();
                symbols += (0..nsc).map(|k| grid.at(l, k).norm_sqr()).sum::<f32>();
                cells += nsc;
                signal += smooth.iter().map(|x| x.norm_sqr()).sum::<f32>();
                noise += raw.iter().zip(&smooth).map(|(a, b)| (a - b).norm_sqr()).sum::<f32>();
            }
            let row = (0..nsc)
                .map(|k| {
                    let pos = (k as f32 - first as f32) / 6.0;
                    let m = pos.floor().clamp(0.0, (smooth.len() - 1) as f32) as usize;
                    let n = (m + 1).min(smooth.len() - 1);
                    let t = (pos - m as f32).clamp(0.0, 1.0);
                    smooth[m] * (1.0 - t) + smooth[n] * t
                })
                .collect();
            known.push((l, row));
        }
        for l in 0..SYMBOLS_PER_SUBFRAME {
            let after = known.iter().position(|(at, _)| *at >= l).unwrap_or(known.len() - 1);
            let before = if known[after].0 > l { after.saturating_sub(1) } else { after };
            let (la, ra) = (&known[before].0, &known[before].1);
            let (lb, rb) = (&known[after].0, &known[after].1);
            let t = if lb == la {
                0.0
            } else {
                ((l as f32 - *la as f32) / (*lb - *la) as f32).clamp(0.0, 1.0)
            };
            let base = (port * SYMBOLS_PER_SUBFRAME + l) * nsc;
            for k in 0..nsc {
                h[base + k] = ra[k] * (1.0 - t) + rb[k] * t;
            }
        }
    }
    let snr_db = (10.0 * (signal / noise.max(1e-12)).max(1e-3).log10()).clamp(-30.0, 60.0);
    Channel {
        nsc,
        fft: grid.fft,
        h,
        snr_db,
        reference_power: reference / pilots.max(1) as f32,
        symbol_power: symbols / cells.max(1) as f32,
    }
}

pub fn combine(grid: &Grid, ch: &Channel, res: &[(usize, usize)], ports: usize) -> Vec<C32> {
    if ports == 1 {
        return res.iter().map(|&(l, k)| ch.at(0, l, k).conj() * grid.at(l, k)).collect();
    }
    let mut out = Vec::with_capacity(res.len());
    for (pair, two) in res.chunks(2).enumerate() {
        let &[(l0, k0), (l1, k1)] = two else {
            out.push(ch.at(0, two[0].0, two[0].1).conj() * grid.at(two[0].0, two[0].1));
            continue;
        };
        let (pa, pb) = if ports == 4 && pair % 2 == 1 {
            (1, 3)
        } else if ports == 4 {
            (0, 2)
        } else {
            (0, 1)
        };
        let a = (ch.at(pa, l0, k0) + ch.at(pa, l1, k1)) * 0.5;
        let b = (ch.at(pb, l0, k0) + ch.at(pb, l1, k1)) * 0.5;
        let (r0, r1) = (grid.at(l0, k0), grid.at(l1, k1));
        out.push(a.conj() * r0 + b * r1.conj());
        out.push(a.conj() * r1 - b * r0.conj());
    }
    out
}

pub fn soft_bits(symbols: &[C32]) -> Vec<f32> {
    symbols.iter().flat_map(|x| [x.re, x.im]).collect()
}

pub fn descramble(soft: &mut [f32], c_init: u32) {
    let c = gold::sequence(soft.len(), c_init);
    for (v, c) in soft.iter_mut().zip(c) {
        if c == 1 {
            *v = -*v;
        }
    }
}
