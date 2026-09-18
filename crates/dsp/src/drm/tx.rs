//! A DRM transmitter, for tests: cells in, a frame of baseband out.
//!
//! It fills the grid the way the standard says to, with the pilots where
//! [`super::cell`] puts them, the fast access and description channels at the
//! places given, and a pseudo-random 4-QAM in the multiplex, which nothing
//! here decodes but the equaliser and the timing both see.

use super::{Cell, Mode, Occupancy, cell, reference};
use common::C32;
use rustfft::{Fft, FftPlanner};
use std::sync::Arc;

pub struct Modulator {
    mode: Mode,
    occupancy: Occupancy,
    ifft: Arc<dyn Fft<f32>>,
    scratch: Vec<C32>,
    state: u32,
}

impl Modulator {
    pub fn new(mode: Mode, occupancy: Occupancy) -> Self {
        let mut planner = FftPlanner::new();
        Self {
            mode,
            occupancy,
            ifft: planner.plan_fft_inverse(mode.fft()),
            scratch: Vec::new(),
            state: 0x1234_5678,
        }
    }

    /// Cells the fast access channel takes in a frame.
    pub fn fac_cells(&self) -> usize {
        super::fac_cells(self.mode).len()
    }

    /// Cells the service description channel takes in a frame.
    pub fn sdc_cells(&self) -> usize {
        super::sdc_cells(self.mode, self.occupancy).len()
    }

    fn noise_cell(&mut self) -> C32 {
        self.state = self.state.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
        let re = if self.state & 0x8000 == 0 { 1.0 } else { -1.0 };
        let im = if self.state & 0x4000 == 0 { 1.0 } else { -1.0 };
        C32::new(re, im) * 0.5f32.sqrt()
    }

    /// One transmission frame. `sdc` may be empty, for a frame of a super
    /// frame that carries no description channel.
    pub fn frame(&mut self, fac: &[C32], sdc: &[C32], out: &mut Vec<C32>) {
        let (tu, tg) = (self.mode.fft(), self.mode.guard());
        let (lo, hi) = self.mode.carriers(self.occupancy);
        let mut fac_at = 0usize;
        let mut sdc_at = 0usize;
        if self.scratch.len() < self.ifft.get_inplace_scratch_len() {
            self.scratch.resize(self.ifft.get_inplace_scratch_len(), C32::new(0.0, 0.0));
        }
        for symbol in 0..self.mode.symbols() {
            let mut bins = vec![C32::new(0.0, 0.0); tu];
            for k in lo..=hi {
                let value = match cell(self.mode, self.occupancy, symbol, k) {
                    Cell::Unused => continue,
                    Cell::Gain | Cell::Frequency | Cell::Time => {
                        reference(self.mode, self.occupancy, symbol, k).unwrap_or_default()
                    }
                    Cell::Fac => {
                        let v = fac.get(fac_at).copied().unwrap_or_else(|| self.noise_cell());
                        fac_at += 1;
                        v
                    }
                    Cell::Sdc => {
                        let v = sdc.get(sdc_at).copied().unwrap_or_else(|| self.noise_cell());
                        sdc_at += 1;
                        v
                    }
                    Cell::Msc => self.noise_cell(),
                };
                bins[k.rem_euclid(tu as i32) as usize] = value;
            }
            self.ifft.process_with_scratch(&mut bins, &mut self.scratch);
            let scale = 1.0 / (tu as f32).sqrt();
            for b in bins.iter_mut() {
                *b *= scale;
            }
            out.extend_from_slice(&bins[tu - tg..]);
            out.extend_from_slice(&bins);
        }
    }
}
