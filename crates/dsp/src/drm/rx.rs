//! The DRM front end: time the symbols off the guard, find the frame by its
//! time references, and hand up a frame of equalised cells.
//!
//! A frame is found in three steps. The cyclic prefix is a copy of the
//! symbol's tail, so correlating a symbol against itself one useful part
//! later peaks where a symbol begins, and the phase of that peak is the
//! fractional part of the frequency error. The first symbol of a frame
//! carries a row of cells whose phases the standard writes out, so
//! correlating a demodulated symbol against them says which of the fifteen
//! is the first and, searched a few carriers either side, what whole number
//! of carriers the tuner is off by.
//!
//! What is left is the channel, which a coherent system needs and DAB beside
//! it does not: the gain references scattered over the grid are divided out
//! of the cells they sit on and interpolated across the carriers between
//! them.

use super::{Cell, Mode, Occupancy, cell, gain_value};
use common::C32;
use rustfft::{Fft, FftPlanner};
use std::f32::consts::TAU;
use std::sync::Arc;

/// One transmission frame, read: every cell of the grid, equalised.
#[derive(Clone, Debug)]
pub struct Frame {
    pub mode: Mode,
    /// The cells, indexed by symbol and then by carrier plus [`Mode::span`],
    /// so carrier `-103` of mode B is index 0.
    grid: Vec<Vec<C32>>,
    span: i32,
    /// How far the fast access channel's cells landed from where a 4-QAM
    /// point should be, as a signal to noise ratio in dB.
    pub snr_db: f32,
    /// The frequency error the front end took out, in Hz.
    pub offset_hz: f64,
}

impl Frame {
    /// One cell of the grid, or zero where the carrier is outside the
    /// transmission.
    pub fn cell(&self, symbol: usize, k: i32) -> C32 {
        let i = k + self.span;
        match self.grid.get(symbol).and_then(|row| row.get(i.max(0) as usize)) {
            Some(&c) if i >= 0 => c,
            _ => C32::new(0.0, 0.0),
        }
    }

    /// The cells at a list of places, in the order given.
    pub fn cells(&self, places: &[(usize, i32)]) -> Vec<C32> {
        places.iter().map(|&(s, k)| self.cell(s, k)).collect()
    }
}

/// How far either side of the expected carrier the integer frequency search
/// looks. Mode B carriers are 46.875 Hz apart, so five of them is +-234 Hz,
/// which is more than a receiver tuned to a broadcast channel is ever out by
/// and still cheap: the search is nineteen cells of one symbol.
const CARRIER_SEARCH: i32 = 5;

/// How strongly the time references have to correlate, against the energy of
/// the symbol they were read from, before a frame is taken for the first of
/// one. Measured on synthesised frames the score is 1.0, and 0.92 for a mode
/// A frame timed a symbol out; the best noise scored over a three frame
/// window was 0.54, which is what the margin here is for.
const LOCK_THRESHOLD: f32 = 0.6;

/// How closely the frequency references have to land on the phases the
/// standard gives them, once the channel is divided out, before a frame is
/// handed up. Nineteen time reference cells searched over fifteen symbols
/// and eleven carrier offsets will find a correlation in noise now and
/// again; the three frequency references are in every symbol, so this reads
/// forty-five cells at one alignment and noise does not reach 0.5 of them.
const REFERENCE_THRESHOLD: f32 = 0.5;

/// The DRM front end.
pub struct Drm {
    mode: Mode,
    occupancy: Occupancy,
    buf: Vec<C32>,
    fft: Arc<dyn Fft<f32>>,
    scratch: Vec<C32>,
    /// Frames read since the last lock, which is what says a signal is there.
    frames: u64,
    locked: bool,
    offset_hz: f64,
}

impl Default for Drm {
    fn default() -> Self {
        Self::new(Mode::B)
    }
}

impl Drm {
    pub fn new(mode: Mode) -> Self {
        let mut planner = FftPlanner::new();
        Self {
            mode,
            occupancy: Occupancy::Full10,
            buf: Vec::new(),
            fft: planner.plan_fft_forward(mode.fft()),
            scratch: Vec::new(),
            frames: 0,
            locked: false,
            offset_hz: f64::NAN,
        }
    }

    pub fn mode(&self) -> Mode {
        self.mode
    }

    pub fn locked(&self) -> bool {
        self.locked
    }

    pub fn frames(&self) -> u64 {
        self.frames
    }

    pub fn offset_hz(&self) -> f64 {
        self.offset_hz
    }

    /// What the fast access channel said the transmission occupies, which
    /// the equaliser needs to know which pilots are boosted.
    pub fn set_occupancy(&mut self, occ: Occupancy) {
        self.occupancy = occ;
    }

    pub fn reset(&mut self) {
        self.buf.clear();
        self.frames = 0;
        self.locked = false;
        self.offset_hz = f64::NAN;
    }

    /// Read what `iq` holds, appending every frame found to `out`.
    pub fn push(&mut self, iq: &[C32], out: &mut Vec<Frame>) {
        self.buf.extend_from_slice(iq);
        // Two frames and a symbol: a frame can begin anywhere in the first of
        // them and still be whole inside what is held.
        let want = 2 * self.mode.frame() + self.mode.symbol();
        while self.buf.len() >= want {
            match self.frame() {
                Some((frame, used)) => {
                    self.frames += 1;
                    self.locked = true;
                    self.offset_hz = frame.offset_hz;
                    out.push(frame);
                    self.buf.drain(..used);
                }
                None => {
                    self.locked = false;
                    // Nothing was found where a frame had to be, so step on
                    // by one frame rather than one sample.
                    self.buf.drain(..self.mode.frame());
                }
            }
        }
    }

    /// One frame out of the head of the buffer, and how many samples it took.
    fn frame(&mut self) -> Option<(Frame, usize)> {
        let (tu, tg, ts) = (self.mode.fft(), self.mode.guard(), self.mode.symbol());
        let symbols = self.mode.symbols();
        let (at, step) = self.timing()?;

        // Every symbol of the search window, demodulated once: the frame can
        // start at any of them and the ones before it are thrown away.
        let mut grids = Vec::with_capacity(2 * symbols);
        for s in 0..2 * symbols {
            let start = at + s * ts + tg;
            if start + tu > self.buf.len() {
                break;
            }
            grids.push(self.transform(start, step));
        }
        if grids.len() < symbols + 1 {
            return None;
        }

        let (first, shift, score) = self.align(&grids)?;
        if score < LOCK_THRESHOLD {
            return None;
        }
        let span = self.mode.span();
        let width = (2 * span + 1) as usize;
        let mut grid = Vec::with_capacity(symbols);
        // The frame is read again with the window back inside the guard,
        // where a timing estimate a sample or two early cannot pull in the
        // symbol before it. What that costs is a phase ramp across the
        // carriers, which the equaliser takes out with the rest of the
        // channel but a correlation against the time references would not,
        // which is why the search above reads the boundary instead.
        // Measured on a synthesised mode A frame timed a sample early: 31 dB
        // on the boundary, over 100 a quarter of a guard back of it.
        let back = tg / 4;
        for s in 0..symbols {
            let start = at + (first + s) * ts + tg - back;
            if start + tu > self.buf.len() {
                return None;
            }
            let bins = self.transform(start, step);
            let mut row = vec![C32::new(0.0, 0.0); width];
            for (i, r) in row.iter_mut().enumerate() {
                let k = i as i32 - span + shift;
                // Reading early is a delay of the whole symbol, which is a
                // phase ramp across the carriers. It is taken out here
                // rather than left to the equaliser, because a channel
                // turning two radians between one pilot and the next cannot
                // be interpolated between them.
                let theta = TAU * (back as f32) * (k as f32) / tu as f32;
                *r = bin(&bins, k, tu) * C32::new(theta.cos(), theta.sin());
            }
            self.equalise(s, span, &mut row);
            grid.push(row);
        }
        let offset_hz =
            (step as f64) / TAU as f64 * super::RATE_HZ + shift as f64 * self.mode.spacing_hz();
        if self.coherence(&grid, span) < REFERENCE_THRESHOLD {
            return None;
        }
        let mut frame = Frame { mode: self.mode, grid, span, snr_db: f32::NAN, offset_hz };
        frame.snr_db = mer_db(&frame.cells(super::fac_cells(self.mode)));
        Some((frame, at + (first + symbols) * ts))
    }

    /// Where a symbol begins and how fast the phase is turning, off the
    /// cyclic prefix: a symbol's guard is a copy of its tail, so the
    /// correlation between the two peaks at the start of the guard and its
    /// angle is the frequency error over a useful part.
    fn timing(&self) -> Option<(usize, f32)> {
        let (tu, tg, ts) = (self.mode.fft(), self.mode.guard(), self.mode.symbol());
        let symbols = self.mode.symbols();
        let mut best = (0usize, 0.0f32, C32::new(0.0, 0.0));
        for n in 0..ts {
            let mut sum = C32::new(0.0, 0.0);
            // Every symbol of a frame votes, which is what lifts the peak
            // out of a noisy channel.
            for s in 0..symbols {
                let base = n + s * ts;
                if base + tg + tu > self.buf.len() {
                    break;
                }
                for j in 0..tg {
                    sum += self.buf[base + j] * self.buf[base + j + tu].conj();
                }
            }
            if sum.norm() > best.1 {
                best = (n, sum.norm(), sum);
            }
        }
        if best.1 <= 0.0 {
            return None;
        }

        // The angle is the error over a useful part, so radians a sample is
        // that over T_u, and the sign is the conjugate's.
        let step = -best.2.arg() / tu as f32;
        Some((best.0, step))
    }

    /// One symbol, de-rotated and transformed.
    fn transform(&mut self, start: usize, step: f32) -> Vec<C32> {
        let tu = self.mode.fft();
        let mut block: Vec<C32> = (0..tu)
            .map(|j| {
                let phase = -step * (start + j) as f32;
                self.buf[start + j] * C32::new(phase.cos(), phase.sin())
            })
            .collect();
        if self.scratch.len() < self.fft.get_inplace_scratch_len() {
            self.scratch.resize(self.fft.get_inplace_scratch_len(), C32::new(0.0, 0.0));
        }
        self.fft.process_with_scratch(&mut block, &mut self.scratch);
        let n = tu as f32;
        for b in block.iter_mut() {
            *b /= n;
        }
        block
    }

    /// Which of the demodulated symbols is the first of a frame, and how many
    /// carriers the tuner is out by, from the time references the standard
    /// puts in the first symbol.
    fn align(&self, grids: &[Vec<C32>]) -> Option<(usize, i32, f32)> {
        let refs = super::time_references(self.mode);
        let tu = self.mode.fft();
        let energy: f32 = refs.iter().map(|(_, v)| v.norm_sqr()).sum();
        let mut best: Option<(usize, i32, f32)> = None;
        for (first, bins) in grids.iter().enumerate().take(grids.len().saturating_sub(1)) {
            if first + self.mode.symbols() > grids.len() {
                break;
            }
            for shift in -CARRIER_SEARCH..=CARRIER_SEARCH {
                let mut sum = C32::new(0.0, 0.0);
                let mut power = 0.0f32;
                for &(k, value) in &refs {
                    let x = bin(bins, k + shift, tu);
                    sum += x * value.conj();
                    power += x.norm_sqr();
                }
                let score = if power > 0.0 { sum.norm() / (power * energy).sqrt() } else { 0.0 };
                // Two identical frames score the same to a part in ten
                // million, so a later one only displaces an earlier one when
                // it is really better.
                if best.is_none_or(|(_, _, b)| score > b + 1e-3) {
                    best = Some((first, shift, score));
                }
            }
        }
        best
    }

    /// How well the frequency references of a whole frame, which are in
    /// every symbol, match the phases the standard gives them once the
    /// channel has been divided out. A frame found in noise does not.
    fn coherence(&self, grid: &[Vec<C32>], span: i32) -> f32 {
        let mut sum = C32::new(0.0, 0.0);
        let (mut power, mut energy) = (0.0f32, 0.0f32);
        for (s, row) in grid.iter().enumerate() {
            for (i, &v) in row.iter().enumerate() {
                let k = i as i32 - span;
                if cell(self.mode, self.occupancy, s, k) != Cell::Frequency {
                    continue;
                }
                let Some(want) = super::reference(self.mode, self.occupancy, s, k) else {
                    continue;
                };
                sum += v * want.conj();
                power += v.norm_sqr();
                energy += want.norm_sqr();
            }
        }
        if power > 0.0 { sum.norm() / (power * energy).sqrt() } else { 0.0 }
    }

    /// Divide the channel out of one symbol: the gain references say what it
    /// did to the carriers they sit on, and everything between them is
    /// interpolated.
    fn equalise(&self, symbol: usize, span: i32, row: &mut [C32]) {
        let (lo, hi) = self.mode.carriers(self.occupancy);
        let mut pilots: Vec<(i32, C32)> = Vec::new();
        for k in lo..=hi {
            if cell(self.mode, self.occupancy, symbol, k) != Cell::Gain {
                continue;
            }
            let want = gain_value(self.mode, self.occupancy, symbol, k);
            let got = row[(k + span) as usize];
            pilots.push((k, got / want));
        }
        if pilots.len() < 2 {
            return;
        }
        // What is left of a timing error after the window was put back where
        // it belongs is a slow turn across the carriers, and a channel that
        // turns between one pilot and the next cannot be interpolated
        // between them. So the turn is measured off the pilots and taken out
        // of both sides of the division, where it cancels. Measured on a
        // mode A frame timed one sample early, whose pilots are twenty
        // carriers apart: 31 dB without this and over 100 with it.
        let turn: C32 = pilots.windows(2).map(|p| p[1].1 * p[0].1.conj()).sum();
        let gap = (pilots[1].0 - pilots[0].0) as f32;
        let slope = if turn.norm_sqr() > 0.0 { turn.arg() / gap } else { 0.0 };
        let flatten = |k: i32| {
            let theta = -slope * k as f32;
            C32::new(theta.cos(), theta.sin())
        };
        for p in pilots.iter_mut() {
            p.1 *= flatten(p.0);
        }
        for (i, r) in row.iter_mut().enumerate() {
            let k = i as i32 - span;
            let h = interpolate(&pilots, k);
            *r = if h.norm_sqr() > 0.0 { *r * flatten(k) / h } else { C32::new(0.0, 0.0) };
        }
    }
}

/// The channel at a carrier, from the pilots either side of it.
fn interpolate(pilots: &[(i32, C32)], k: i32) -> C32 {
    match pilots.binary_search_by_key(&k, |&(c, _)| c) {
        Ok(i) => pilots[i].1,
        Err(0) => pilots[0].1,
        Err(i) if i >= pilots.len() => pilots[pilots.len() - 1].1,
        Err(i) => {
            let (k0, h0) = pilots[i - 1];
            let (k1, h1) = pilots[i];
            let t = (k - k0) as f32 / (k1 - k0) as f32;
            h0 * (1.0 - t) + h1 * t
        }
    }
}

/// A carrier's bin, which is its index modulo the transform size.
fn bin(bins: &[C32], k: i32, tu: usize) -> C32 {
    bins[k.rem_euclid(tu as i32) as usize]
}

/// How far a set of cells landed from the 4-QAM points, in dB. The cells are
/// normalised first, so this says nothing about the level they arrived at.
fn mer_db(cells: &[C32]) -> f32 {
    if cells.is_empty() {
        return f32::NAN;
    }
    let power: f32 = cells.iter().map(|c| c.norm_sqr()).sum::<f32>() / cells.len() as f32;
    if power <= 0.0 {
        return f32::NAN;
    }
    let scale = (1.0 / power).sqrt();
    let mut error = 0.0f32;
    for c in cells {
        let c = *c * scale;
        let want = C32::new(0.5f32.sqrt().copysign(c.re), 0.5f32.sqrt().copysign(c.im));
        error += (c - want).norm_sqr();
    }
    let error = (error / cells.len() as f32).max(1e-12);
    10.0 * (1.0 / error).log10()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::drm::tx::Modulator;

    /// A run of 4-QAM cells, which is what the channels look like before
    /// anything decodes them.
    fn cells(n: usize, seed: u32) -> Vec<C32> {
        let mut state = seed;
        (0..n)
            .map(|_| {
                state = state.wrapping_mul(1_103_515_245).wrapping_add(12_345);
                let re = if state & 0x1_0000 == 0 { 1.0 } else { -1.0 };
                let im = if state & 0x2_0000 == 0 { 1.0 } else { -1.0 };
                C32::new(re, im) * 0.5f32.sqrt()
            })
            .collect()
    }

    fn air(mode: Mode, occ: Occupancy, frames: usize, fac: &[C32], sdc: &[C32]) -> Vec<C32> {
        let mut tx = Modulator::new(mode, occ);
        let mut out = Vec::new();
        for _ in 0..frames {
            tx.frame(fac, sdc, &mut out);
        }
        out
    }

    /// Every mode, modulated and read back: the frame is found, the cells
    /// come back where they were put, and a clean channel reads over 30 dB.
    #[test]
    fn a_frame_comes_back_cell_for_cell() {
        for mode in Mode::ALL {
            let occ = Occupancy::Full10;
            let fac = cells(super::super::fac_cells(mode).len(), 7);
            let sdc = cells(super::super::sdc_cells(mode, occ).len(), 11);
            let signal = air(mode, occ, 6, &fac, &sdc);
            let mut rx = Drm::new(mode);
            let mut frames = Vec::new();
            rx.push(&signal, &mut frames);
            // Six frames in, four back: a frame is only read once the one
            // after it is in hand, because the search for where a frame
            // begins looks over two frames, and the last of them is left.
            assert_eq!(frames.len(), 4, "mode {} read {} frames", mode.label(), frames.len());
            for frame in &frames {
                let read = frame.cells(super::super::fac_cells(mode));
                for (i, (got, want)) in read.iter().zip(fac.iter()).enumerate() {
                    assert!(
                        (got - want).norm() < 0.1,
                        "mode {} cell {i}: {got} for {want}",
                        mode.label()
                    );
                }
                assert!(frame.snr_db > 55.0, "mode {} reads {} dB", mode.label(), frame.snr_db);
            }
            assert!(rx.locked());
        }
    }

    /// Noise alone: nothing correlates with the time references, so no frame
    /// is handed up and the front end never says it is locked.
    #[test]
    fn noise_is_not_a_frame() {
        let mut state = 0xfeed_beefu32;
        let mut rx = Drm::new(Mode::B);
        let mut frames = Vec::new();
        for _ in 0..60 {
            let noise: Vec<C32> = (0..Mode::B.frame())
                .map(|_| {
                    let mut next = || {
                        state = state.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
                        (state >> 16) as i16 as f32 / 32768.0
                    };
                    C32::new(next(), next())
                })
                .collect();
            rx.push(&noise, &mut frames);
        }
        assert_eq!(frames.len(), 0);
        assert!(!rx.locked());
    }
}
