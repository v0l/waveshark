use super::receiver::{Heard, Receiver, SYNC_WIDTH_HZ};
use super::{Mib, bands};
use common::C32;
use rustfft::FftPlanner;
use std::collections::VecDeque;

const TRIAL_S: f64 = 0.040;

const TRIALS_AT_ONCE: usize = 4;

const SWEEP_S: f64 = 30.0;

const LOST_S: f64 = 60.0;

const PSD_SIZE: usize = 4096;

const PSD_FRAMES: usize = 64;

const SMOOTH_BINS: usize = 8;

const OVER_FLOOR_DB: f32 = 6.0;

const NARROWEST_BLOCK_HZ: f64 = 1.0e6;

const OCCUPIED_HZ: [f64; 6] = [9.0e6, 18.0e6, 13.5e6, 4.5e6, 2.7e6, 1.08e6];

struct Carrier {
    channel_hz: f64,
    rx: Receiver,
    occupied_hz: f64,
    last: u64,
}

struct Trial {
    channel_hz: f64,
    rx: Receiver,
    until: u64,
}

pub struct Scan {
    rate: f64,
    center_hz: f64,
    seed: Option<f64>,
    carriers: Vec<Carrier>,
    trials: Vec<Trial>,
    queue: VecDeque<f64>,
    psd: Psd,
    next_sweep: u64,
    at: u64,
    heard: Vec<Heard>,
}

impl Scan {
    pub fn new(rate: f64, center_hz: f64, seed: Option<f64>) -> Option<Self> {
        Receiver::new(rate, center_hz, center_hz)?;
        let seed = seed.map(bands::on_raster).filter(|&hz| usable(rate, center_hz, hz));
        Some(Self {
            rate,
            center_hz,
            seed,
            carriers: Vec::new(),
            trials: Vec::new(),
            queue: seed.into_iter().collect(),
            psd: Psd::new(),
            next_sweep: 0,
            at: 0,
            heard: Vec::new(),
        })
    }

    pub fn reset(&mut self) {
        self.carriers.clear();
        self.trials.clear();
        self.queue = self.seed.into_iter().collect();
        self.psd = Psd::new();
        self.next_sweep = 0;
        self.at = 0;
    }

    pub fn carriers(&self) -> impl Iterator<Item = f64> + '_ {
        self.carriers.iter().map(|c| c.channel_hz)
    }

    pub fn held(&self) -> Vec<(f64, f64)> {
        self.carriers
            .iter()
            .filter(|c| c.occupied_hz > 0.0)
            .map(|c| (c.channel_hz - c.occupied_hz / 2.0, c.channel_hz + c.occupied_hz / 2.0))
            .collect()
    }

    pub fn receiver_mut(&mut self, channel_hz: f64) -> Option<&mut Receiver> {
        self.carriers.iter_mut().find(|c| c.channel_hz == channel_hz).map(|c| &mut c.rx)
    }

    fn samples(&self, s: f64) -> u64 {
        (s * self.rate) as u64
    }

    pub fn push(&mut self, iq: &[C32], out: &mut Vec<(f64, Heard)>) {
        self.start_trials();
        self.at += iq.len() as u64;
        for c in &mut self.carriers {
            self.heard.clear();
            c.rx.push(iq, &mut self.heard);
            for h in self.heard.drain(..) {
                c.last = self.at;
                if let Some(mib) = mib_of(&h) {
                    c.occupied_hz = mib.bandwidth.occupied_hz();
                }
                out.push((c.channel_hz, h));
            }
        }
        let lost = self.samples(LOST_S);
        let at = self.at;
        self.carriers.retain(|c| at.saturating_sub(c.last) < lost);
        self.run_trials(iq, out);
        if self.trials.is_empty()
            && self.queue.is_empty()
            && self.at >= self.next_sweep
            && self.psd.add(iq)
        {
            self.sweep();
        }
    }

    fn run_trials(&mut self, iq: &[C32], out: &mut Vec<(f64, Heard)>) {
        let mut found = false;
        let mut k = 0;
        while k < self.trials.len() {
            self.heard.clear();
            self.trials[k].rx.push(iq, &mut self.heard);
            if self.heard.is_empty() {
                match self.at >= self.trials[k].until {
                    true => drop(self.trials.swap_remove(k)),
                    false => k += 1,
                }
                continue;
            }
            let t = self.trials.swap_remove(k);
            let occupied_hz =
                self.heard.iter().find_map(mib_of).map_or(0.0, |m| m.bandwidth.occupied_hz());
            for h in self.heard.drain(..) {
                out.push((t.channel_hz, h));
            }
            self.carriers.push(Carrier {
                channel_hz: t.channel_hz,
                rx: t.rx,
                occupied_hz,
                last: self.at,
            });
            found = true;
        }
        if !found {
            return;
        }
        let taken = self.taken();
        self.trials.retain(|t| !inside(&taken, t.channel_hz));
        self.queue.retain(|hz| !inside(&taken, *hz));
        if let Some(blocks) = self.psd.blocks.clone() {
            for hz in candidates(&blocks, &taken, self.rate, self.center_hz) {
                if !self.queue.contains(&hz) {
                    self.queue.push_back(hz);
                }
            }
        }
    }

    fn start_trials(&mut self) {
        let taken = self.taken();
        while self.trials.len() < TRIALS_AT_ONCE {
            let Some(hz) = self.queue.pop_front() else { return };
            if inside(&taken, hz) || self.trials.iter().any(|t| t.channel_hz == hz) {
                continue;
            }
            let Some(rx) = Receiver::new(self.rate, self.center_hz, hz) else { continue };
            let until = self.at + self.samples(TRIAL_S);
            self.trials.push(Trial { channel_hz: hz, rx, until });
        }
    }

    fn taken(&self) -> Vec<(f64, f64)> {
        self.carriers
            .iter()
            .map(|c| {
                let half = c.occupied_hz.max(SYNC_WIDTH_HZ) / 2.0;
                (c.channel_hz - half, c.channel_hz + half)
            })
            .collect()
    }

    fn sweep(&mut self) {
        let blocks = self.psd.finish(self.rate, self.center_hz);
        let taken = self.taken();
        self.queue.extend(self.seed.filter(|hz| !inside(&taken, *hz)));
        for hz in candidates(&blocks, &taken, self.rate, self.center_hz) {
            if !self.queue.contains(&hz) {
                self.queue.push_back(hz);
            }
        }
        self.next_sweep = self.at + self.samples(SWEEP_S);
    }
}

fn mib_of(h: &Heard) -> Option<Mib> {
    match h {
        Heard::Mib { mib, .. } => Some(*mib),
        Heard::SystemInformation { .. } => None,
    }
}

fn inside(taken: &[(f64, f64)], hz: f64) -> bool {
    taken.iter().any(|(lo, hi)| (*lo..=*hi).contains(&hz))
}

fn usable(rate: f64, center_hz: f64, hz: f64) -> bool {
    (hz - center_hz).abs() <= rate / 2.0 - SYNC_WIDTH_HZ / 2.0 && bands::containing(hz).is_some()
}

struct Psd {
    sum: Vec<f32>,
    frames: usize,
    held: Vec<C32>,
    blocks: Option<Vec<(f64, f64)>>,
}

impl Psd {
    fn new() -> Self {
        Self { sum: vec![0.0; PSD_SIZE], frames: 0, held: Vec::new(), blocks: None }
    }

    fn add(&mut self, iq: &[C32]) -> bool {
        self.held.extend_from_slice(iq);
        let fft = FftPlanner::<f32>::new().plan_fft_forward(PSD_SIZE);
        let mut buf = vec![C32::default(); PSD_SIZE];
        let mut used = 0;
        while self.frames < PSD_FRAMES && used + PSD_SIZE <= self.held.len() {
            buf.copy_from_slice(&self.held[used..used + PSD_SIZE]);
            fft.process(&mut buf);
            for (s, v) in self.sum.iter_mut().zip(&buf) {
                *s += v.norm_sqr();
            }
            self.frames += 1;
            used += PSD_SIZE;
        }
        self.held.drain(..used);
        self.frames >= PSD_FRAMES
    }

    fn finish(&mut self, rate: f64, center_hz: f64) -> Vec<(f64, f64)> {
        let mut psd = std::mem::replace(&mut self.sum, vec![0.0; PSD_SIZE]);
        self.frames = 0;
        self.held.clear();
        psd.rotate_left(PSD_SIZE / 2);
        let bin_hz = rate / PSD_SIZE as f64;
        let found = blocks(&psd, bin_hz, center_hz - rate / 2.0);
        self.blocks = Some(found.clone());
        found
    }
}

pub fn blocks(psd: &[f32], bin_hz: f64, lo_hz: f64) -> Vec<(f64, f64)> {
    let n = psd.len();
    let db: Vec<f32> = (0..n)
        .map(|i| {
            let (a, b) = (i.saturating_sub(SMOOTH_BINS), (i + SMOOTH_BINS + 1).min(n));
            10.0 * (psd[a..b].iter().sum::<f32>() / (b - a) as f32).max(1e-30).log10()
        })
        .collect();
    let mut sorted = db.clone();
    sorted.sort_by(f32::total_cmp);
    let threshold = sorted[n / 100] + OVER_FLOOR_DB;
    let mut out = Vec::new();
    let mut run: Option<usize> = None;
    for (i, up) in db.iter().map(|v| *v >= threshold).chain(std::iter::once(false)).enumerate() {
        match (up, run) {
            (true, None) => run = Some(i),
            (false, Some(start)) => {
                let (lo, hi) = (lo_hz + start as f64 * bin_hz, lo_hz + i as f64 * bin_hz);
                if hi - lo >= NARROWEST_BLOCK_HZ {
                    out.push((lo, hi));
                }
                run = None;
            }
            _ => {}
        }
    }
    out
}

pub fn candidates(
    blocks: &[(f64, f64)],
    taken: &[(f64, f64)],
    rate: f64,
    center_hz: f64,
) -> Vec<f64> {
    let mut out: Vec<f64> = Vec::new();
    for (lo, hi) in blocks.iter().flat_map(|b| free(*b, taken)) {
        let wide = hi - lo;
        let mut guesses = vec![(lo + hi) / 2.0];
        for w in OCCUPIED_HZ.iter().filter(|w| **w <= wide * 1.2 + 200e3) {
            guesses.push(lo + w / 2.0);
            guesses.push(hi - w / 2.0);
        }
        for g in guesses {
            let on = bands::on_raster(g);
            for hz in [on, on - bands::RASTER_HZ, on + bands::RASTER_HZ] {
                if usable(rate, center_hz, hz) && !inside(taken, hz) && !out.contains(&hz) {
                    out.push(hz);
                }
            }
        }
    }
    out
}

fn free((lo, hi): (f64, f64), taken: &[(f64, f64)]) -> Vec<(f64, f64)> {
    let mut pieces = vec![(lo, hi)];
    for (a, b) in taken {
        pieces = pieces
            .into_iter()
            .flat_map(|(lo, hi)| {
                if *b <= lo || *a >= hi {
                    return vec![(lo, hi)];
                }
                [(lo, *a), (*b, hi)]
                    .into_iter()
                    .filter(|(l, h)| h - l >= NARROWEST_BLOCK_HZ)
                    .collect()
            })
            .collect();
    }
    pieces
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn three_adjacent_ten_megahertz_carriers_are_guessed_from_the_edges_in() {
        let block = [(791.5e6, 820.5e6)];
        let first = candidates(&block, &[], 30.72e6, 806e6);
        assert!(first.contains(&796e6) && first.contains(&816e6), "{first:?}");
        let taken = [(791.5e6, 800.5e6), (811.5e6, 820.5e6)];
        let then = candidates(&block, &taken, 30.72e6, 806e6);
        assert_eq!(then.first(), Some(&806e6), "the middle once both edges are known: {then:?}");
    }

    #[test]
    fn a_guess_is_on_the_raster_inside_an_lte_band_and_the_span() {
        let got = candidates(&[(700e6, 710e6)], &[], 20e6, 705e6);
        assert_eq!(got, Vec::<f64>::new(), "700 to 710 MHz is in no LTE downlink");
        let got = candidates(&[(758.5e6, 767.5e6)], &[], 12e6, 762.95e6);
        assert_eq!(got.first(), Some(&763e6));
        assert!(got.iter().all(|hz| (hz / 100e3).fract() == 0.0));
        assert!(got.iter().all(|hz| (hz - 762.95e6).abs() <= 6e6 - SYNC_WIDTH_HZ / 2.0));
    }

    #[test]
    fn a_minute_of_noise_across_band_20_names_no_cell_and_tries_nothing() {
        let rate = 1.92e6;
        let mut scan = Scan::new(rate, 806e6, None).expect("fast enough");
        let mut seed = 0x2545_f491_4f6c_dd1du64;
        let mut r = move || {
            seed ^= seed << 13;
            seed ^= seed >> 7;
            seed ^= seed << 17;
            (seed >> 40) as f32 / 8_388_608.0 - 1.0
        };
        let (mut out, mut tried) = (Vec::new(), 0);
        for _ in 0..(60.0 * rate / 65_536.0) as usize {
            let block: Vec<C32> = (0..65_536).map(|_| C32::new(r(), r()) * 0.1).collect();
            scan.push(&block, &mut out);
            tried += scan.trials.len();
        }
        assert_eq!(out.len(), 0, "rows out of noise");
        assert_eq!(scan.carriers().count(), 0);
        assert_eq!(tried, 0, "flat noise is no block, so nothing is worth a trial");
    }

    #[test]
    fn a_block_is_a_run_a_megahertz_wide_above_the_floor() {
        let n = 1000;
        let psd: Vec<f32> =
            (0..n).map(|i| if (300..500).contains(&i) { 100.0 } else { 1.0 }).collect();
        assert_eq!(blocks(&psd, 10e3, 800e6), vec![(803e6 - 80e3, 805e6 + 80e3)]);
        let narrow: Vec<f32> =
            (0..n).map(|i| if (300..350).contains(&i) { 100.0 } else { 1.0 }).collect();
        assert_eq!(blocks(&narrow, 10e3, 800e6), Vec::<(f64, f64)>::new());
    }
}
