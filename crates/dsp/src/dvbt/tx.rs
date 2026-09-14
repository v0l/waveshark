//! A DVB-T modulator: data cells in, samples out.
//!
//! It exists because the receiver has to be tested against a signal whose
//! every parameter is known, and because the transmit chain draws the same
//! stages the receive chain does. It is the standard's own order: map the
//! cells onto the data carriers of this symbol, put the pilots and the TPS
//! bit on the rest, inverse transform, and copy the tail of the useful part
//! in front of it as the guard.

use super::tps;
use super::{Carrier, Layout, Params, SYMBOLS_PER_FRAME, bin};
use common::C32;
use rustfft::{Fft, FftPlanner};
use std::sync::Arc;

pub struct Modulator {
    params: Params,
    layout: Layout,
    fft: Arc<dyn Fft<f32>>,
    scratch: Vec<C32>,
    grid: Vec<C32>,
    /// The TPS word of the frame being transmitted, one bit per symbol.
    word: [u8; SYMBOLS_PER_FRAME],
    /// The value each TPS carrier held in the previous symbol, which is what
    /// the differential modulation is against.
    tps_val: Vec<f32>,
    tps_carriers: Vec<usize>,
    symbol: usize,
    frame: u8,
    /// Makes the time domain signal unit RMS whatever the FFT size.
    scale: f32,
}

impl Modulator {
    pub fn new(params: Params) -> Self {
        let mode = params.mode;
        let layout = Layout::new(mode);
        let fft = FftPlanner::new().plan_fft_inverse(mode.fft());
        let tps_carriers = super::tps_carriers(mode);
        Self {
            scratch: vec![C32::default(); fft.get_inplace_scratch_len()],
            fft,
            grid: vec![C32::default(); mode.fft()],
            word: tps::encode(&params, 0),
            tps_val: tps_carriers.iter().map(|&k| layout.tps_reference(k)).collect(),
            tps_carriers,
            symbol: 0,
            frame: 0,
            scale: 1.0 / (mode.carriers() as f32).sqrt(),
            params,
            layout,
        }
    }

    /// Data cells one symbol carries.
    pub fn cells(&self) -> usize {
        self.params.mode.cells()
    }

    /// Samples one symbol takes, guard included.
    pub fn symbol_samples(&self) -> usize {
        self.params.symbol_samples()
    }

    /// Where in the frame the next symbol will be.
    pub fn symbol_index(&self) -> usize {
        self.symbol
    }

    /// Modulate one symbol's worth of cells, appending its samples to `out`.
    pub fn modulate(&mut self, cells: &[C32], out: &mut Vec<C32>) {
        assert_eq!(cells.len(), self.cells(), "a symbol takes exactly its cells");
        let mode = self.params.mode;
        let phase = self.symbol % 4;
        self.grid.iter_mut().for_each(|c| *c = C32::default());

        let mut cell = 0;
        for k in 0..mode.carriers() {
            let value = match self.layout.carrier(phase, k) {
                Carrier::Data => {
                    cell += 1;
                    cells[cell - 1]
                }
                Carrier::Pilot => C32::new(self.layout.pilot(k), 0.0),
                Carrier::Tps => continue,
            };
            self.grid[bin(mode, k)] = value;
        }
        debug_assert_eq!(cell, cells.len());

        // The TPS carriers are differential: symbol zero is the reference the
        // frame starts from, and after that a one inverts what was there.
        for (i, &k) in self.tps_carriers.iter().enumerate() {
            if self.symbol == 0 {
                self.tps_val[i] = self.layout.tps_reference(k);
            } else if self.word[self.symbol] == 1 {
                self.tps_val[i] = -self.tps_val[i];
            }
            self.grid[bin(mode, k)] = C32::new(self.tps_val[i], 0.0);
        }

        self.fft.process_with_scratch(&mut self.grid, &mut self.scratch);
        let guard = self.params.guard.samples(mode);
        let start = out.len();
        out.extend_from_slice(&self.grid[mode.fft() - guard..]);
        out.extend_from_slice(&self.grid);
        for s in &mut out[start..] {
            *s *= self.scale;
        }

        self.symbol += 1;
        if self.symbol == SYMBOLS_PER_FRAME {
            self.symbol = 0;
            self.frame = (self.frame + 1) % 4;
            self.word = tps::encode(&self.params, self.frame);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dvbt::{Constellation, Guard, Mode};

    fn cells(n: usize, seed: u32, c: Constellation) -> Vec<C32> {
        let mut state = seed | 1;
        (0..n)
            .map(|_| {
                state = state.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
                let word = (state >> 8) as usize & ((1 << c.bits()) - 1);
                let bits: Vec<u8> =
                    (0..c.bits()).map(|i| ((word >> (c.bits() - 1 - i)) & 1) as u8).collect();
                super::super::map(&bits, c)
            })
            .collect()
    }

    /// The guard is a copy of the tail of the useful part, which is what a
    /// receiver correlates against to find a symbol at all.
    #[test]
    fn the_guard_repeats_the_end_of_the_symbol() {
        let params = Params { mode: Mode::M2k, guard: Guard::G1_4, ..Params::typical() };
        let mut tx = Modulator::new(params);
        let mut out = Vec::new();
        tx.modulate(&cells(params.mode.cells(), 7, params.constellation), &mut out);
        let g = params.guard.samples(params.mode);
        assert_eq!(out.len(), params.symbol_samples());
        for i in 0..g {
            let d = out[i] - out[i + params.mode.fft()];
            assert!(d.norm() < 1e-6, "sample {i} of the guard differs");
        }
    }

    /// A symbol is near enough unit power, which is what the levels the
    /// receiver reports are against.
    #[test]
    fn the_output_is_unit_power() {
        for mode in Mode::ALL {
            let params = Params { mode, ..Params::typical() };
            let mut tx = Modulator::new(params);
            let mut out = Vec::new();
            for _ in 0..4 {
                tx.modulate(&cells(mode.cells(), 3, params.constellation), &mut out);
            }
            let power: f32 = out.iter().map(|s| s.norm_sqr()).sum::<f32>() / out.len() as f32;
            assert!((power - 1.0).abs() < 0.15, "{} is {power} power", mode.label());
        }
    }
}
