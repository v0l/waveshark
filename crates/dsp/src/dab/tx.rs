//! A DAB transmitter: the bits of a frame onto carriers, and the carriers
//! into samples.
//!
//! It is here so the receiver can be tested against an ensemble whose every
//! service is known, and it is the order of stages a transmit chain would
//! draw.

use super::{Mode, bin, interleave, map, phase_reference};
use common::C32;
use rustfft::{Fft, FftPlanner};
use std::sync::Arc;

/// One transmission frame at a time: a null, a phase reference and the
/// symbols the bits fill.
pub struct Modulator {
    mode: Mode,
    ifft: Arc<dyn Fft<f32>>,
    scratch: Vec<C32>,
    grid: Vec<C32>,
    prev: Vec<C32>,
    interleave: Vec<i32>,
}

impl Modulator {
    /// Panics for a mode whose phase reference table is not here, because
    /// there is nothing to start the differential modulation from.
    pub fn new(mode: Mode) -> Self {
        let prs = phase_reference(mode).expect("a transmission mode with its phase reference");
        Self {
            ifft: FftPlanner::new().plan_fft_inverse(mode.fft()),
            scratch: Vec::new(),
            grid: Vec::new(),
            prev: prs,
            interleave: interleave(mode),
            mode,
        }
    }

    /// Bits one frame carries: two per carrier for every symbol after the
    /// phase reference.
    pub fn bits_per_frame(&self) -> usize {
        (self.mode.symbols() - 1) * 2 * self.mode.carriers()
    }

    /// One whole frame onto the air.
    pub fn frame(&mut self, bits: &[u8], out: &mut Vec<C32>) {
        assert_eq!(bits.len(), self.bits_per_frame(), "a frame's worth of bits");
        let k = self.mode.carriers();
        out.resize(out.len() + self.mode.null(), C32::default());
        self.prev = phase_reference(self.mode).expect("a mode with its phase reference");
        let prs = self.prev.clone();
        self.symbol(&prs, out);
        for l in 0..self.mode.symbols() - 1 {
            let block = &bits[l * 2 * k..(l + 1) * 2 * k];
            let mut grid = vec![C32::default(); self.mode.fft()];
            for (n, &carrier) in self.interleave.iter().enumerate() {
                let b = bin(self.mode, carrier);
                grid[b] = self.prev[b] * map(block[n], block[n + k]);
            }
            self.prev = grid.clone();
            self.symbol(&grid, out);
        }
    }

    /// One symbol of carriers into samples, with its cyclic prefix.
    fn symbol(&mut self, carriers: &[C32], out: &mut Vec<C32>) {
        let n = self.mode.fft();
        self.grid.clear();
        self.grid.extend_from_slice(carriers);
        self.scratch.resize(self.ifft.get_inplace_scratch_len(), C32::default());
        self.ifft.process_with_scratch(&mut self.grid, &mut self.scratch);
        let scale = 1.0 / (self.mode.carriers() as f32).sqrt();
        out.extend(self.grid[n - self.mode.guard()..].iter().map(|s| *s * scale));
        out.extend(self.grid.iter().map(|s| *s * scale));
    }
}
