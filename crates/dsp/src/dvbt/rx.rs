//! The DVB-T receiver's front end: find the symbol, transform it, take the
//! channel off the pilots, and hand up equalised data cells.
//!
//! Nothing above this has to know where a symbol starts. The guard interval
//! is a copy of the tail of the useful part, so a delayed correlation over
//! every mode and guard the standard allows says which of the eight a
//! transmission is and where its symbols begin, and the phase of that same
//! correlation is the frequency error to within half a carrier. What is left
//! is an integer number of carriers, which the continual and scattered pilots
//! settle along with the symbol's place in the four symbol pilot cycle.
//!
//! The channel estimate is linear interpolation between the pilots of the
//! symbol being read, which are never more than twelve carriers apart. It
//! follows a common phase error for free, because every symbol is estimated
//! from its own pilots.

use super::{Carrier, Guard, Layout, Mode, Params, SYMBOLS_PER_FRAME, TpsDecoder, bin};
use common::C32;
use rustfft::{Fft, FftPlanner};
use std::collections::HashMap;
use std::f64::consts::TAU;
use std::sync::Arc;

/// One OFDM symbol, read.
#[derive(Clone, Debug)]
pub struct Symbol {
    /// The data cells, equalised, in ascending carrier order.
    pub cells: Vec<C32>,
    /// The channel power each cell was read through, which is what weights a
    /// soft decision: a cell in a null is worth less than one on a peak.
    pub csi: Vec<f32>,
    /// Where this symbol sits in the sixty-eight symbol frame, once a TPS
    /// word has said so.
    pub index: Option<usize>,
    /// Which of the four frames of the super frame it belongs to. The outer
    /// coding starts again on frame zero, so this is what says where the
    /// transport packets begin.
    pub frame: Option<u8>,
    /// The TPS bit this symbol carried.
    pub tps_bit: u8,
    /// Signal to noise on the pilots, in dB.
    pub snr_db: f32,
}

/// How far either side of the expected carrier the integer frequency search
/// looks. A tuner within a few kHz of the channel is well inside this.
const CARRIER_SEARCH: i32 = 6;

/// Symbols the acquisition folds the guard correlation over. Three is enough
/// to put the right hypothesis clear of the wrong ones and short enough that
/// a scanner does not sit on a channel.
const ACQUIRE_SYMBOLS: usize = 3;

/// Below this the folded correlation is noise rather than a multiplex.
const ACQUIRE_THRESHOLD: f32 = 0.12;

/// The DVB-T front end.
pub struct Dvbt {
    /// Fixed at construction, or found by searching every mode and guard.
    locked: Option<Lock>,
    want: Option<(Mode, Guard)>,
    buf: Vec<C32>,
    /// Phase of the fractional frequency correction at the front of `buf`.
    phase: f64,
    /// Radians a sample, the fractional frequency error.
    step: f64,
    tps: TpsDecoder,
    params: Option<Params>,
    layouts: HashMap<Mode, Layout>,
    plans: HashMap<Mode, Arc<dyn Fft<f32>>>,
    scratch: Vec<C32>,
    grid: Vec<C32>,
    /// Symbol index in the frame for the next symbol, once known.
    next_index: Option<usize>,
    /// Frame number for the next symbol, counted on from the last TPS word.
    next_frame: Option<u8>,
    h: Vec<C32>,
    /// The equalised value each TPS carrier held in the previous symbol.
    tps_prev: Vec<C32>,
}

struct Lock {
    mode: Mode,
    guard: Guard,
    /// Where the next symbol's guard starts in `buf`.
    pos: usize,
    /// Carriers the signal is offset by.
    shift: i32,
    /// Symbol index modulo four, which says where the scattered pilots are.
    phase: usize,
}

impl Default for Dvbt {
    fn default() -> Self {
        Self::new()
    }
}

impl Dvbt {
    /// A receiver that searches for whichever of the eight mode and guard
    /// combinations is on the air.
    pub fn new() -> Self {
        Self {
            locked: None,
            want: None,
            buf: Vec::new(),
            phase: 0.0,
            step: 0.0,
            tps: TpsDecoder::new(),
            params: None,
            layouts: HashMap::new(),
            plans: HashMap::new(),
            scratch: Vec::new(),
            grid: Vec::new(),
            next_index: None,
            next_frame: None,
            h: Vec::new(),
            tps_prev: Vec::new(),
        }
    }

    /// A receiver told what to expect, which skips the search.
    pub fn with_mode(mode: Mode, guard: Guard) -> Self {
        Self { want: Some((mode, guard)), ..Self::new() }
    }

    /// The multiplex's parameters, once the TPS has said what they are.
    pub fn params(&self) -> Option<Params> {
        self.params
    }

    /// The mode and guard in use, which are known one symbol after
    /// acquisition and before any TPS word has decoded.
    pub fn mode_guard(&self) -> Option<(Mode, Guard)> {
        self.locked.as_ref().map(|l| (l.mode, l.guard))
    }

    pub fn locked(&self) -> bool {
        self.locked.is_some()
    }

    /// Read what `x` holds, appending every symbol it completes to `out`.
    pub fn push(&mut self, x: &[C32], out: &mut Vec<Symbol>) {
        self.buf.extend_from_slice(x);
        loop {
            if self.locked.is_none() && !self.acquire() {
                break;
            }
            if !self.symbol(out) {
                break;
            }
        }
        self.trim();
    }

    /// Drop samples nothing will read again, keeping the buffer bounded.
    fn trim(&mut self) {
        let keep = match &self.locked {
            Some(l) => l.pos,
            // Unlocked, a window long enough for the widest hypothesis has to
            // stay whole, so only what is beyond one more of them goes.
            None => self.buf.len().saturating_sub(2 * Self::acquire_window()),
        };
        if keep == 0 {
            return;
        }
        self.buf.drain(..keep);
        self.phase = wrap(self.phase + keep as f64 * self.step);
        if let Some(l) = &mut self.locked {
            l.pos -= keep;
        }
    }

    /// Samples the search needs: three of the longest symbol and a useful
    /// part beyond them to correlate the last one against.
    fn acquire_window() -> usize {
        let ts = Mode::M8k.fft() + Guard::G1_4.samples(Mode::M8k);
        ACQUIRE_SYMBOLS * ts + Mode::M8k.fft()
    }

    /// Find the mode, the guard and where a symbol starts. True once locked.
    fn acquire(&mut self) -> bool {
        let window = Self::acquire_window();
        if self.buf.len() < window {
            return false;
        }
        let power: f32 =
            self.buf[..window].iter().map(|s| s.norm_sqr()).sum::<f32>() / window as f32;
        if power <= 0.0 {
            return false;
        }

        let candidates: Vec<(Mode, Guard)> = match self.want {
            Some(mg) => vec![mg],
            None => {
                Mode::ALL.iter().flat_map(|&m| Guard::ALL.iter().map(move |&g| (m, g))).collect()
            }
        };

        let mut best: Option<(Mode, Guard, usize, f32, f64)> = None;
        for (mode, guard) in candidates {
            let n = mode.fft();
            let g = guard.samples(mode);
            let ts = n + g;
            let last = window - n - g;
            // The delayed correlation, slid rather than recomputed: one
            // multiply in and one out for each step.
            let mut c = C32::default();
            for i in 0..g {
                c += self.buf[i] * self.buf[i + n].conj();
            }
            let mut sums = vec![C32::default(); ts];
            let mut counts = vec![0usize; ts];
            for start in 0..=last {
                sums[start % ts] += c;
                counts[start % ts] += 1;
                if start < last {
                    c += self.buf[start + g] * self.buf[start + g + n].conj();
                    c -= self.buf[start] * self.buf[start + n].conj();
                }
            }
            let (pos, peak) = sums
                .iter()
                .enumerate()
                .max_by(|a, b| a.1.norm().total_cmp(&b.1.norm()))
                .map(|(i, c)| (i, *c))
                .unwrap_or_default();
            let score = peak.norm() / (g as f32 * counts[pos].max(1) as f32 * power);
            // A slower rate of samples over the same air time correlates over
            // more of them, so the score is already normalised; what is not
            // is the frequency error, which is in carriers of this mode.
            let cfo = -(peak.im as f64).atan2(peak.re as f64) / TAU;
            if best.is_none_or(|b| score > b.3) {
                best = Some((mode, guard, pos, score, cfo));
            }
        }

        let Some((mode, guard, pos, score, cfo)) = best else { return false };
        if score < ACQUIRE_THRESHOLD {
            // Nothing here. Keep the tail in case a multiplex starts inside
            // the window that was just refused.
            let drop = self.buf.len().saturating_sub(Self::acquire_window());
            self.buf.drain(..drop);
            self.phase = wrap(self.phase + drop as f64 * self.step);
            return false;
        }
        self.step = -TAU * cfo / mode.fft() as f64;
        self.layouts.entry(mode).or_insert_with(|| Layout::new(mode));
        self.plans.entry(mode).or_insert_with(|| FftPlanner::new().plan_fft_forward(mode.fft()));
        self.locked = Some(Lock { mode, guard, pos, shift: 0, phase: usize::MAX });
        self.tps.reset();
        self.next_index = None;
        self.next_frame = None;
        true
    }

    /// Read one symbol if there is one. False when more samples are needed.
    fn symbol(&mut self, out: &mut Vec<Symbol>) -> bool {
        let Some(lock) = &self.locked else { return false };
        let (mode, guard, pos) = (lock.mode, lock.guard, lock.pos);
        let n = mode.fft();
        let g = guard.samples(mode);
        // The window sits a little inside the guard, so a timing estimate
        // that is slightly late still reads one symbol rather than two. What
        // it costs is a phase ramp across the carriers, and the ramp is a
        // fraction of a turn between neighbouring pilots at this backoff.
        let backoff = (g / 4).min(n / 256);
        let start = pos + g - backoff;
        if self.buf.len() < start + n + g {
            return false;
        }

        self.grid.resize(n, C32::default());
        let base = self.phase + start as f64 * self.step;
        for (i, out) in self.grid.iter_mut().enumerate() {
            let (s, c) = wrap(base + i as f64 * self.step).sin_cos();
            *out = self.buf[start + i] * C32::new(c as f32, s as f32);
        }
        let fft = self.plans[&mode].clone();
        self.scratch.resize(fft.get_inplace_scratch_len(), C32::default());
        fft.process_with_scratch(&mut self.grid, &mut self.scratch);

        // The backoff is a known delay, so take its phase ramp out before the
        // pilots are looked at rather than asking them to carry it.
        if backoff > 0 {
            for k in 0..n {
                let f = if k < n / 2 { k as f64 } else { k as f64 - n as f64 };
                let a = wrap(TAU * f * backoff as f64 / n as f64);
                let (s, c) = a.sin_cos();
                self.grid[k] *= C32::new(c as f32, s as f32);
            }
        }

        let (shift, phase) = match self.locked.as_ref().map(|l| (l.shift, l.phase)) {
            Some((shift, phase)) if phase != usize::MAX => (shift, (phase + 1) % 4),
            _ => match self.find_pilots(mode) {
                Some(found) => found,
                None => {
                    // Nothing pilot-shaped: this was not a multiplex.
                    self.abandon(pos + n + g);
                    return true;
                }
            },
        };

        let (snr_db, ok) = self.estimate_channel(mode, shift, phase);
        if !ok {
            self.abandon(pos + n + g);
            return true;
        }

        let mut cells = Vec::with_capacity(mode.cells());
        let mut csi = Vec::with_capacity(mode.cells());
        {
            let layout = &self.layouts[&mode];
            for &k in layout.data(phase) {
                let k = k as usize;
                let h = self.h[k];
                let y = self.grid[carrier_bin(mode, k, shift)];
                cells.push(y / h);
                csi.push(h.norm_sqr());
            }
        }

        let tps_bit = self.read_tps(mode, shift);
        // A word ends on symbol sixty-seven, so reading one both names this
        // symbol and puts the next at the start of a frame.
        let mut index = self.next_index;
        let mut frame = self.next_frame;
        match self.tps.push(tps_bit) {
            Some(tps) => {
                index = Some(SYMBOLS_PER_FRAME - 1);
                frame = Some(tps.frame);
                self.next_index = Some(0);
                self.next_frame = Some((tps.frame + 1) % 4);
                self.params = Some(tps.params(self.tps.cell_id()));
            }
            None => {
                self.next_index = index.map(|i| (i + 1) % SYMBOLS_PER_FRAME);
                if self.next_index == Some(0) {
                    self.next_frame = frame.map(|f| (f + 1) % 4);
                }
            }
        }

        out.push(Symbol { cells, csi, index, frame, tps_bit, snr_db });

        if let Some(l) = &mut self.locked {
            l.shift = shift;
            l.phase = phase;
            l.pos = pos + n + g;
        }
        self.track(mode, guard);
        true
    }

    /// Give up on a lock that read nothing, throwing away the samples it was
    /// taken on.
    ///
    /// The samples have to go. Acquisition looks at the buffer from the
    /// front, so a lock abandoned without consuming anything is a lock found
    /// again immediately, on the same samples, for as long as they are there.
    fn abandon(&mut self, past: usize) {
        self.locked = None;
        let drop = past.min(self.buf.len());
        self.buf.drain(..drop);
        self.phase = wrap(self.phase + drop as f64 * self.step);
    }

    /// Follow the symbol clock. A sample rate error walks the symbol start
    /// slowly, so the guard correlation is taken again either side of where
    /// the next symbol is expected and the position nudged by one sample.
    fn track(&mut self, mode: Mode, guard: Guard) {
        let Some(lock) = &self.locked else { return };
        let n = mode.fft();
        let g = guard.samples(mode);
        let pos = lock.pos;
        let reach = 2usize;
        if pos < reach || self.buf.len() < pos + reach + g + n {
            return;
        }
        let at = |p: usize| -> f32 {
            let mut c = C32::default();
            for i in 0..g {
                c += self.buf[p + i] * self.buf[p + i + n].conj();
            }
            c.norm()
        };
        let here = at(pos);
        let early = at(pos - 1);
        let late = at(pos + 1);
        if let Some(l) = &mut self.locked {
            if early > here && early > late {
                l.pos -= 1;
            } else if late > here && late > early {
                l.pos += 1;
            }
        }
    }

    /// Which carrier the signal sits on and which of the four pilot phases
    /// this symbol is, taken together: a wrong guess about either scatters
    /// the pilots over data cells and the metric collapses.
    ///
    /// The metric is differential along the pilot grid, so a channel with any
    /// phase of its own, and any slope, still adds up.
    fn find_pilots(&self, mode: Mode) -> Option<(i32, usize)> {
        let layout = &self.layouts[&mode];
        let w = layout.w();
        let mut best: Option<(i32, usize, f32)> = None;
        for shift in -CARRIER_SEARCH..=CARRIER_SEARCH {
            for phase in 0..4 {
                let mut acc = C32::default();
                let mut k = 3 * phase;
                while k + 12 <= mode.k_max() {
                    let sign = if w[k] == w[k + 12] { 1.0 } else { -1.0 };
                    let a = self.grid[carrier_bin(mode, k, shift)];
                    let b = self.grid[carrier_bin(mode, k + 12, shift)];
                    acc += a * b.conj() * sign;
                    k += 12;
                }
                let score = acc.norm();
                if best.is_none_or(|b| score > b.2) {
                    best = Some((shift, phase, score));
                }
            }
        }
        let (shift, phase, score) = best?;
        // Against the total power on the pilot grid, a real pilot pattern is
        // most of it and a wrong hypothesis is a random walk.
        let mut power = 0.0f32;
        let mut k = 3 * phase;
        while k <= mode.k_max() {
            power += self.grid[carrier_bin(mode, k, shift)].norm_sqr();
            k += 12;
        }
        (score > 0.2 * power).then_some((shift, phase))
    }

    /// The channel on every carrier, interpolated between this symbol's
    /// pilots, and the signal to noise the pilots imply.
    fn estimate_channel(&mut self, mode: Mode, shift: i32, phase: usize) -> (f32, bool) {
        let layout = &self.layouts[&mode];
        self.h.clear();
        self.h.resize(mode.carriers(), C32::default());
        let mut at: Vec<usize> = Vec::with_capacity(mode.carriers() / 6);
        for k in 0..mode.carriers() {
            if layout.carrier(phase, k) == Carrier::Pilot {
                let y = self.grid[carrier_bin(mode, k, shift)];
                self.h[k] = y / layout.pilot(k);
                at.push(k);
            }
        }
        if at.len() < 8 {
            return (0.0, false);
        }
        // Noise off the curvature of the estimate: on a channel that is
        // smooth over twelve carriers, what is left of the second difference
        // is noise, and the 3/2 undoes the averaging of three samples.
        let mut noise = 0.0f32;
        let mut signal = 0.0f32;
        for w in at.windows(3) {
            let mid = (self.h[w[0]] + self.h[w[2]]) * 0.5;
            noise += (self.h[w[1]] - mid).norm_sqr() * (2.0 / 3.0);
            signal += self.h[w[1]].norm_sqr();
        }
        let snr_db = if noise > 0.0 { 10.0 * (signal / noise).max(1e-9).log10() } else { 60.0 };

        for pair in at.windows(2) {
            let (a, b) = (pair[0], pair[1]);
            let span = (b - a) as f32;
            for k in a + 1..b {
                let t = (k - a) as f32 / span;
                self.h[k] = self.h[a] * (1.0 - t) + self.h[b] * t;
            }
        }
        let (first, last) = (at[0], at[at.len() - 1]);
        for k in 0..first {
            self.h[k] = self.h[first];
        }
        for k in last + 1..mode.carriers() {
            self.h[k] = self.h[last];
        }
        (snr_db, true)
    }

    /// The bit this symbol's TPS carriers hold, by majority over all of them.
    /// They are differential, so what is kept from symbol to symbol is the
    /// equalised value rather than the bit.
    fn read_tps(&mut self, mode: Mode, shift: i32) -> u8 {
        let carriers = super::tps_carriers(mode);
        let mut vote = 0i32;
        self.tps_prev.resize(carriers.len(), C32::default());
        for (i, &k) in carriers.iter().enumerate() {
            let y = self.grid[carrier_bin(mode, k, shift)] / self.h[k];
            let diff = y * self.tps_prev[i].conj();
            if self.tps_prev[i] != C32::default() {
                vote += if diff.re >= 0.0 { 1 } else { -1 };
            }
            self.tps_prev[i] = y;
        }
        u8::from(vote < 0)
    }
}

/// The FFT bin carrier `k` arrived in, given the signal sits `shift` carriers
/// off where the tuner put it.
fn carrier_bin(mode: Mode, k: usize, shift: i32) -> usize {
    let n = mode.fft() as i32;
    let b = bin(mode, k) as i32 + shift;
    (b.rem_euclid(n)) as usize
}

/// Keep a phase in the range an f64 sine is accurate over.
fn wrap(mut a: f64) -> f64 {
    while a > std::f64::consts::PI {
        a -= TAU;
    }
    while a < -std::f64::consts::PI {
        a += TAU;
    }
    a
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dvbt::{CodeRate, Constellation, Hierarchy, tx::Modulator};

    /// A deterministic cell sequence, so a test can say which cell it expected
    /// where. Not a transport stream: this is the front end, which cannot see
    /// one.
    struct Cells {
        state: u32,
        constellation: Constellation,
    }

    impl Cells {
        fn new(seed: u32, constellation: Constellation) -> Self {
            Self { state: seed | 1, constellation }
        }

        fn symbol(&mut self, n: usize) -> Vec<C32> {
            let v = self.constellation.bits();
            (0..n)
                .map(|_| {
                    self.state = self.state.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
                    let word = (self.state >> 9) as usize & ((1 << v) - 1);
                    let bits: Vec<u8> = (0..v).map(|i| ((word >> (v - 1 - i)) & 1) as u8).collect();
                    super::super::map(&bits, self.constellation)
                })
                .collect()
        }
    }

    /// Gaussian noise from a pair of uniforms, at a power relative to unity.
    struct Noise {
        state: u64,
    }

    impl Noise {
        fn new(seed: u64) -> Self {
            Self { state: seed | 1 }
        }

        fn uniform(&mut self) -> f32 {
            self.state = self.state.wrapping_mul(6_364_136_223_846_793_005).wrapping_add(1);
            ((self.state >> 33) as f32 / (1u32 << 31) as f32).clamp(1e-7, 1.0 - 1e-7)
        }

        fn sample(&mut self, sigma: f32) -> C32 {
            let (u, v) = (self.uniform(), self.uniform());
            let r = (-2.0 * u.ln()).sqrt() * sigma / 2f32.sqrt();
            C32::new(r * (TAU as f32 * v).cos(), r * (TAU as f32 * v).sin())
        }
    }

    fn transmit(params: Params, frames: usize) -> (Vec<Vec<C32>>, Vec<C32>) {
        let mut tx = Modulator::new(params);
        let mut cells = Cells::new(0x51D5, params.constellation);
        let mut sent = Vec::new();
        let mut air = Vec::new();
        for _ in 0..frames * SYMBOLS_PER_FRAME {
            let symbol = cells.symbol(params.mode.cells());
            tx.modulate(&symbol, &mut air);
            sent.push(symbol);
        }
        (sent, air)
    }

    /// Offset in carriers, noise at `snr_db`, and `lead` samples of nothing in
    /// front so the receiver has to find the symbol boundary itself.
    fn impair(air: &[C32], carriers: f64, snr_db: f32, mode: Mode, lead: usize) -> Vec<C32> {
        let mut noise = Noise::new(0xD7B);
        let sigma = 10f32.powf(-snr_db / 20.0);
        let mut out: Vec<C32> = (0..lead).map(|_| noise.sample(sigma)).collect();
        for (n, s) in air.iter().enumerate() {
            let a = TAU * carriers * n as f64 / mode.fft() as f64;
            let (sin, cos) = (a % TAU).sin_cos();
            out.push(s * C32::new(cos as f32, sin as f32) + noise.sample(sigma));
        }
        out
    }

    fn read(samples: &[C32]) -> (Dvbt, Vec<Symbol>) {
        let mut rx = Dvbt::new();
        let mut out = Vec::new();
        for block in samples.chunks(4096) {
            rx.push(block, &mut out);
        }
        (rx, out)
    }

    /// Three frames of 2K 64-QAM, offset by two and a third carriers, 35 dB
    /// down on noise and starting in the middle of nowhere. The receiver
    /// finds the mode and guard by itself, and every cell of every symbol it
    /// reads is the cell that was sent, within a quarter of the distance
    /// between constellation points.
    #[test]
    fn a_modulated_multiplex_is_read_back_cell_for_cell() {
        let params = Params {
            mode: Mode::M2k,
            guard: Guard::G1_32,
            constellation: Constellation::Qam64,
            hierarchy: Hierarchy::None,
            code_rate_hp: CodeRate::R2_3,
            code_rate_lp: CodeRate::R2_3,
            cell_id: Some(0x1A2B),
        };
        let (sent, air) = transmit(params, 3);
        let samples = impair(&air, 2.3333, 35.0, params.mode, 777);
        let (rx, got) = read(&samples);

        assert_eq!(rx.mode_guard(), Some((params.mode, params.guard)));
        assert_eq!(rx.params(), Some(params), "the TPS says what the multiplex is");
        // The search reads a window of the longest symbol the standard has
        // before it can refuse the other seven modes, but it reports the
        // first symbol start in that window, so only the last symbol, which
        // arrives short of its guard, is lost.
        assert_eq!(got.len(), 203, "symbols read out of 204 transmitted");

        // Half the distance between neighbouring points is the decision
        // boundary: a cell inside it demaps to the word that was sent.
        let limit = params.constellation.norm();
        let offset = (0..40)
            .min_by(|&a, &b| error(&sent[a], &got[0]).total_cmp(&error(&sent[b], &got[0])))
            .unwrap();
        assert_eq!(offset, 0, "the first symbol read is the first sent");

        let mut cells = 0usize;
        let mut worst = 0.0f32;
        let mut power = 0.0f64;
        for (i, symbol) in got.iter().enumerate() {
            let want = &sent[offset + i];
            for (a, b) in symbol.cells.iter().zip(want) {
                let e = (a - b).norm();
                worst = worst.max(e);
                power += (e * e) as f64;
                if e < limit {
                    cells += 1;
                }
            }
        }
        let total = got.len() * params.mode.cells();
        assert_eq!(cells, total, "every cell demaps to what was sent, worst error {worst}");
        // The modulation error is the noise it was given and no more: at
        // 35 dB the equaliser costs about a decibel, from the noise on the
        // pilots the channel estimate is interpolated between.
        let evm_db = 10.0 * (power / total as f64).log10();
        assert!((-35.0..-32.0).contains(&evm_db), "{evm_db} dB of error");
        assert!(got.iter().all(|s| s.snr_db > 25.0), "the pilots read the noise they were given");
    }

    /// The frame counter is found and then followed: once a TPS word has been
    /// read, every symbol after it is numbered, and the numbers run 0 to 67
    /// without a gap.
    #[test]
    fn the_frame_boundary_is_found_and_followed() {
        let params = Params { mode: Mode::M2k, ..Params::typical() };
        let (_, air) = transmit(params, 4);
        let samples = impair(&air, 0.0, 35.0, params.mode, 0);
        let (_, got) = read(&samples);
        let numbered: Vec<usize> = got.iter().filter_map(|s| s.index).collect();
        assert_eq!(got.len(), 271, "symbols read out of 272 transmitted");
        // A word is sixty-eight symbols long, so the first sixty-seven
        // symbols are read before anything can say where they sit.
        assert_eq!(numbered.len(), 204, "symbols numbered out of 271 read");
        assert!(
            numbered.windows(2).all(|w| w[1] == (w[0] + 1) % SYMBOLS_PER_FRAME),
            "the count runs without a gap"
        );
    }

    /// Every mode and guard the standard allows is found from the air alone.
    #[test]
    fn every_mode_and_guard_is_found() {
        for mode in Mode::ALL {
            for guard in Guard::ALL {
                let params = Params { mode, guard, ..Params::typical() };
                let frames = if mode == Mode::M2k { 2 } else { 1 };
                let (_, air) = transmit(params, frames);
                let samples = impair(&air, 1.0, 30.0, mode, 333);
                let (rx, got) = read(&samples);
                assert_eq!(
                    rx.mode_guard(),
                    Some((mode, guard)),
                    "{} {} was read as something else",
                    mode.label(),
                    guard.label()
                );
                assert!(!got.is_empty(), "{} {} read no symbols", mode.label(), guard.label());
            }
        }
    }

    fn error(want: &[C32], got: &Symbol) -> f32 {
        want.iter().zip(&got.cells).map(|(a, b)| (a - b).norm_sqr()).sum()
    }
}
