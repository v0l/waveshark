use super::grid::{Demodulator, Numerology, estimate};
use super::sync::{self, Searcher};
use super::{Mib, SUBCARRIER_HZ, control, pbch, pdsch};
use crate::fir::FirDecim;
use crate::mixer::Mixer;
use crate::resample::{self, Rational};
use common::C32;

const SEARCH_S: f64 = 0.030;
const IDLE_S: f64 = 1.0;
const REREAD_S: f64 = 10.0;
const SI_TRIES: usize = 8;
const SYNC_PASSBAND_HZ: f64 = 700e3;
const CHUNK_MARGIN_S: f64 = 250e-6;
pub const SYNC_WIDTH_HZ: f64 = 72.0 * SUBCARRIER_HZ;

#[derive(Clone, Debug)]
pub enum Heard {
    Mib {
        pci: u16,
        mib: Mib,
        offset_hz: f64,
        width_hz: f64,
        snr_db: f32,
        rssi_dbfs: f32,
        rsrp_dbfs: f32,
        rsrq_db: f32,
        samples: Vec<C32>,
        rate: f64,
    },
    SystemInformation {
        pci: u16,
        bytes: Vec<u8>,
        offset_hz: f64,
        width_hz: f64,
        snr_db: f32,
        rssi_dbfs: f32,
        rsrp_dbfs: f32,
        rsrq_db: f32,
        samples: Vec<C32>,
        rate: f64,
    },
}

#[derive(Clone, Copy, Debug)]
enum State {
    Search { from: u64 },
    Read,
    Idle { until: u64 },
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Wanted {
    Sib1,
    Window { id: usize, index: u8 },
}

#[derive(Clone, Copy, Debug)]
struct Target {
    frame: u64,
    subframe: u8,
    wanted: Wanted,
}

struct Cell {
    pci: u16,
    n2: usize,
    offset_hz: f64,
    mib: Mib,
    frame_raw: f64,
    work: Option<Numerology>,
    targets: std::collections::VecDeque<Target>,
}

impl Cell {
    fn done(&self) -> bool {
        self.targets.is_empty()
    }

    fn sfn(&self, frame: u64) -> u16 {
        ((u64::from(self.mib.sfn) + frame) % 1024) as u16
    }
}

pub struct Receiver {
    rate: f64,
    shift_hz: f64,
    raw: Vec<C32>,
    base: u64,
    state: State,
    searcher: Searcher,
    sync_demod: Demodulator,
    cells: Vec<Cell>,
}

impl Receiver {
    pub fn new(rate: f64, center_hz: f64, channel_hz: f64) -> Option<Self> {
        if rate + 1.0 < sync::RATE_HZ {
            return None;
        }
        Some(Self {
            rate,
            shift_hz: center_hz - channel_hz,
            raw: Vec::new(),
            base: 0,
            state: State::Search { from: 0 },
            searcher: Searcher::new(),
            sync_demod: Demodulator::new(Numerology::SYNC),
            cells: Vec::new(),
        })
    }

    pub fn reset(&mut self) {
        self.raw.clear();
        self.base = 0;
        self.state = State::Search { from: 0 };
        self.cells.clear();
    }

    fn end(&self) -> u64 {
        self.base + self.raw.len() as u64
    }

    fn samples(&self, s: f64) -> u64 {
        (s * self.rate).ceil() as u64
    }

    pub fn push(&mut self, iq: &[C32], out: &mut Vec<Heard>) {
        self.raw.extend_from_slice(iq);
        loop {
            match self.state {
                State::Search { from } => {
                    let to = from + self.samples(SEARCH_S);
                    if self.end() < to {
                        break;
                    }
                    let window = self.slice(from, to).to_vec();
                    self.search(&window, from, out);
                    self.state = if self.cells.iter().any(|c| !c.done()) {
                        State::Read
                    } else {
                        State::Idle {
                            until: to
                                + self.samples(if self.cells.is_empty() {
                                    IDLE_S
                                } else {
                                    REREAD_S
                                }),
                        }
                    };
                }
                State::Read => {
                    if !self.read_pending(out) {
                        break;
                    }
                    if self.cells.iter().all(Cell::done) {
                        self.state = State::Idle { until: self.end() + self.samples(REREAD_S) };
                    }
                }
                State::Idle { until } => {
                    if self.end() < until {
                        break;
                    }
                    self.cells.clear();
                    self.state = State::Search { from: until };
                }
            }
        }
        self.trim();
    }

    fn slice(&self, from: u64, to: u64) -> &[C32] {
        let a = (from.saturating_sub(self.base) as usize).min(self.raw.len());
        let b = (to.saturating_sub(self.base) as usize).min(self.raw.len());
        &self.raw[a..b]
    }

    fn trim(&mut self) {
        let keep_from = match self.state {
            State::Search { from } => from,
            State::Read => self
                .cells
                .iter()
                .filter_map(|c| c.targets.front().map(|t| self.chunk_start(c, t)))
                .min()
                .unwrap_or(self.end()),
            State::Idle { until } => until,
        };
        let drop = keep_from.saturating_sub(self.base).min(self.raw.len() as u64) as usize;
        if drop > 0 {
            self.raw.drain(..drop);
            self.base += drop as u64;
        }
    }

    fn to_sync(&self, window: &[C32], shift_hz: f64) -> Vec<C32> {
        let mut mixed = Vec::with_capacity(window.len());
        Mixer::new(shift_hz, self.rate).process(window, &mut mixed);
        let Some((factor, rational)) = resample::stage(self.rate, sync::RATE_HZ, 1024) else {
            return Vec::new();
        };
        let mut decim = FirDecim::design_hz(self.rate, factor, SYNC_PASSBAND_HZ, 60.0);
        let mut mid = Vec::new();
        decim.process(&mixed, &mut mid);
        match rational {
            Some(mut r) => {
                let mut y = Vec::new();
                r.process(&mid, &mut y);
                y
            }
            None => mid,
        }
    }

    fn search(&mut self, window: &[C32], from: u64, out: &mut Vec<Heard>) {
        self.cells.clear();
        let coarse = self.to_sync(window, self.shift_hz);
        let fraction = sync::fractional_cfo_hz(&coarse);
        let mut y = Vec::with_capacity(coarse.len());
        Mixer::new(-fraction, sync::RATE_HZ).process(&coarse, &mut y);
        let scale = self.rate / sync::RATE_HZ;
        for found in self.searcher.search(&y) {
            if self.cells.iter().any(|c| c.pci == found.pci) {
                continue;
            }
            let mut z = Vec::with_capacity(y.len());
            Mixer::new(-found.offset_hz, sync::RATE_HZ).process(&y, &mut z);
            let mut at = found.frame_start;
            while at + sync::FRAME / 10 <= z.len() {
                if let Some(read) = pbch::read(&mut self.sync_demod, &z, at, found.pci) {
                    let samples = z[at..at + sync::FRAME / 10].to_vec();
                    let offset_hz = found.offset_hz + fraction;
                    out.push(Heard::Mib {
                        pci: found.pci,
                        mib: read.mib,
                        offset_hz,
                        width_hz: SYNC_WIDTH_HZ,
                        snr_db: read.snr_db,
                        rssi_dbfs: read.rsrp_dbfs
                            + 10.0 * ((read.mib.bandwidth.prbs() * 12) as f32).log10(),
                        rsrp_dbfs: read.rsrp_dbfs,
                        rsrq_db: read.rsrq_db,
                        samples,
                        rate: sync::RATE_HZ,
                    });
                    let work = self.work_for(&read.mib);
                    let mut cell = Cell {
                        pci: found.pci,
                        n2: usize::from(found.pci) % 3,
                        offset_hz,
                        mib: read.mib,
                        frame_raw: from as f64 + at as f64 * scale,
                        work,
                        targets: Default::default(),
                    };
                    if work.is_some() {
                        let first = self.first_frame(&cell, from + window.len() as u64);
                        let first = first + u64::from(cell.sfn(first) % 2);
                        cell.targets = (0..SI_TRIES as u64)
                            .map(|i| Target {
                                frame: first + 2 * i,
                                subframe: 5,
                                wanted: Wanted::Sib1,
                            })
                            .collect();
                    }
                    self.cells.push(cell);
                    break;
                }
                at += sync::FRAME;
            }
        }
    }

    fn work_for(&self, mib: &Mib) -> Option<Numerology> {
        let occupied = mib.bandwidth.occupied_hz();
        if self.rate * 0.9 < occupied {
            return None;
        }
        let fft = (mib.bandwidth.prbs() * 12 + 1).next_power_of_two().max(128);
        Numerology::for_rate(fft as f64 * SUBCARRIER_HZ)
    }

    fn frame_len(&self) -> f64 {
        self.rate * 0.01
    }

    fn target_raw(&self, c: &Cell, t: &Target) -> f64 {
        c.frame_raw + (t.frame as f64 + f64::from(t.subframe) / 10.0) * self.frame_len()
    }

    fn first_frame(&self, c: &Cell, after: u64) -> u64 {
        let margin = self.rate * CHUNK_MARGIN_S;
        let behind = (after as f64 + margin - c.frame_raw) / self.frame_len();
        behind.max(0.0).ceil() as u64
    }

    fn chunk_start(&self, c: &Cell, t: &Target) -> u64 {
        (self.target_raw(c, t) - self.rate * CHUNK_MARGIN_S).max(0.0) as u64
    }

    fn chunk_end(&self, c: &Cell, t: &Target) -> u64 {
        (self.target_raw(c, t) + self.rate * (1e-3 + CHUNK_MARGIN_S)).ceil() as u64
    }

    pub fn schedule(&mut self, pci: u16, window_ms: u8, periods: &[u16]) {
        let end = self.end();
        let Some(i) = self.cells.iter().position(|c| c.pci == pci && c.work.is_some()) else {
            return;
        };
        let first = self.first_frame(&self.cells[i], end);
        let c = &mut self.cells[i];
        let w = u64::from(window_ms);
        for (id, &period) in periods.iter().enumerate() {
            let x = id as u64 * w;
            let (at_frame, at_subframe) = (x / 10, x % 10);
            let period = u64::from(period.max(1));
            let Some(start) = (first..first + period)
                .find(|&f| u64::from(c.sfn(f)) % period == at_frame % period)
            else {
                continue;
            };
            for index in 0..w {
                let s = at_subframe + index;
                let (frame, subframe) = (start + s / 10, (s % 10) as u8);
                if subframe == 5 && c.sfn(frame).is_multiple_of(2) {
                    continue;
                }
                c.targets.push_back(Target {
                    frame,
                    subframe,
                    wanted: Wanted::Window { id, index: index as u8 },
                });
            }
        }
        c.targets.make_contiguous().sort_by_key(|t| (t.frame, t.subframe));
        if !c.targets.is_empty() {
            self.state = State::Read;
        }
    }

    fn read_pending(&mut self, out: &mut Vec<Heard>) -> bool {
        let mut progressed = false;
        for i in 0..self.cells.len() {
            while let Some(t) = self.cells[i].targets.front().copied() {
                let c = &self.cells[i];
                let (from, to) = (self.chunk_start(c, &t), self.chunk_end(c, &t));
                if self.end() < to {
                    break;
                }
                progressed = true;
                let chunk = self.slice(from, to).to_vec();
                let expected = self.target_raw(c, &t) - from as f64;
                let (heard, moved) = self.read_subframe(c, &chunk, expected, &t);
                let c = &mut self.cells[i];
                c.frame_raw += moved;
                c.targets.pop_front();
                if let Some(h) = heard {
                    out.push(h);
                    c.targets.retain(|o| o.wanted.other_message(&t.wanted));
                }
            }
        }
        progressed
    }

    fn read_subframe(
        &self,
        c: &Cell,
        chunk: &[C32],
        expected_raw: f64,
        t: &Target,
    ) -> (Option<Heard>, f64) {
        let Some(num) = c.work else { return (None, 0.0) };
        let w = num.rate();
        let mut mixed = Vec::with_capacity(chunk.len());
        Mixer::new(self.shift_hz - c.offset_hz, self.rate).process(chunk, &mut mixed);
        let Some(y) = to_rate(&mixed, self.rate, w) else { return (None, 0.0) };
        let expected = (expected_raw * w / self.rate).round() as isize;
        let sf = usize::from(t.subframe);
        let nrb = c.mib.bandwidth.prbs();
        let mut demod = Demodulator::new(num);
        let start = match sf {
            0 | 5 => refine(&y, num, c.n2, expected),
            _ => {
                let Some(first) = usize::try_from(expected).ok() else { return (None, 0.0) };
                demod.subframe(&y, first, nrb).and_then(|g| {
                    let late = timing_error(&g, sf, c.pci);
                    usize::try_from(expected + late.round() as isize).ok()
                })
            }
        };
        let Some(start) = start else { return (None, 0.0) };
        let moved = (start as f64 - expected as f64) * self.rate / w;
        let sfn = c.sfn(t.frame);
        let rv_1c = match t.wanted {
            Wanted::Sib1 => control::si_rv(sfn),
            Wanted::Window { index, .. } => control::rv_of(index),
        };
        let heard = (|| {
            let grid = demod.subframe(&y, start, nrb)?;
            let ch = estimate(&grid, sf, c.pci, c.mib.ports);
            let cfi = control::cfi(&grid, &ch, sf, c.pci, c.mib.ports)?;
            let symbols = control::control_symbols(cfi, nrb);
            let soft = control::pdcch_soft(&grid, &ch, sf, c.pci, &c.mib, symbols)?;
            let grant = control::si_grant(&soft, nrb, rv_1c)?;
            let bytes = pdsch::read_si(&grid, &ch, &grant, sf, symbols, c.pci, c.mib.ports)?;
            let samples = y.get(start..start + num.subframe())?.to_vec();
            Some(Heard::SystemInformation {
                pci: c.pci,
                bytes,
                offset_hz: c.offset_hz,
                width_hz: c.mib.bandwidth.occupied_hz(),
                snr_db: ch.snr_db,
                rssi_dbfs: ch.cell_dbfs(nrb),
                rsrp_dbfs: ch.rsrp_dbfs(),
                rsrq_db: ch.rsrq_db(),
                samples,
                rate: w,
            })
        })();
        (heard, moved)
    }
}

impl Wanted {
    fn other_message(&self, got: &Wanted) -> bool {
        match (self, got) {
            (Wanted::Sib1, Wanted::Sib1) => false,
            (Wanted::Window { id: a, .. }, Wanted::Window { id: b, .. }) => a != b,
            _ => true,
        }
    }
}

fn timing_error(g: &super::grid::Grid, subframe: usize, pci: u16) -> f32 {
    let shift = usize::from(pci) % 6;
    let r = super::grid::crs(2 * subframe, 0, pci, g.nrb);
    let h: Vec<C32> =
        r.iter().enumerate().map(|(m, x)| g.at(0, 6 * m + shift) * x.conj()).collect();
    let turn: C32 = h.windows(2).map(|p| p[1] * p[0].conj()).sum();
    turn.arg() * g.fft() as f32 / (12.0 * std::f32::consts::PI)
}

fn to_rate(x: &[C32], rate: f64, want: f64) -> Option<Vec<C32>> {
    if (rate - want).abs() < 1.0 {
        return Some(x.to_vec());
    }
    let mut y = Vec::new();
    if rate < want {
        Rational::new(rate, want, 4096)
            .unwrap_or_else(|| Rational::approx(rate, want, 4096))
            .process(x, &mut y);
        return Some(y);
    }
    let (factor, rational) = resample::stage(rate, want, 4096)?;
    let mut mid = Vec::new();
    FirDecim::design_hz(rate, factor, want * 0.45, 60.0).process(x, &mut mid);
    match rational {
        Some(mut r) => r.process(&mid, &mut y),
        None => y = mid,
    }
    Some(y)
}

fn refine(y: &[C32], num: Numerology, n2: usize, expected: isize) -> Option<usize> {
    let n = num.fft;
    let mut bins = vec![C32::default(); n];
    for (i, d) in sync::pss(n2).iter().enumerate() {
        let k = if i < 31 { i as isize - 31 } else { i as isize - 30 };
        bins[k.rem_euclid(n as isize) as usize] = *d;
    }
    rustfft::FftPlanner::<f32>::new().plan_fft_inverse(n).process(&mut bins);
    let reach = 2 * num.cp(1) as isize;
    let centre = expected + num.symbol(6) as isize;
    let (mut best, mut at) = (0f32, None);
    for pos in centre - reach..=centre + reach {
        let Ok(p) = usize::try_from(pos) else { continue };
        let Some(win) = y.get(p..p + n) else { continue };
        let v: C32 = win.iter().zip(&bins).map(|(a, b)| a * b.conj()).sum();
        if v.norm_sqr() > best {
            (best, at) = (v.norm_sqr(), Some(p));
        }
    }
    at?.checked_sub(num.symbol(6))
}
