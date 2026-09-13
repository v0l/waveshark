//! The inner interleavers and the demapper, EN 300 744 clauses 4.3.4 and
//! 4.3.5: what turns a symbol's cells back into the coded bit stream the
//! Viterbi reads, and what turns that stream into cells at the transmitter.
//!
//! Two interleavers sit between the code and the carriers, and they are there
//! for the same reason: a notch in the channel takes out a run of carriers,
//! and a convolutional code cannot correct a run. The symbol interleaver
//! scatters the cells of a symbol across its carriers by a permutation built
//! from a shift register, and the bit interleaver, inside blocks of 126
//! cells, scatters the bits of a cell across the stream so that the ones that
//! share a constellation point are not neighbours in the code.

use super::{axis, Constellation, Mode};
use common::C32;

/// Cells in one bit interleaver block.
const BLOCK: usize = 126;

/// The rotations of the bit interleaver, EN 300 744 clause 4.3.4.
const ROTATE: [usize; 6] = [0, 63, 105, 42, 21, 84];

/// The bit permutation of the symbol interleaver's register.
const PERM_2K: [usize; 10] = [4, 3, 9, 6, 2, 8, 1, 5, 7, 0];
const PERM_8K: [usize; 12] = [7, 1, 4, 2, 9, 6, 8, 10, 0, 3, 11, 5];

/// The permutation H(q) the symbol interleaver applies, EN 300 744 clause
/// 4.3.4.1: a maximal length register whose bits are then shuffled, with the
/// values that land outside the used cells thrown away.
pub fn permutation(mode: Mode) -> Vec<u32> {
    let n = mode.fft();
    let nr = n.trailing_zeros() as usize;
    let perm: &[usize] = match mode {
        Mode::M2k => &PERM_2K,
        Mode::M8k => &PERM_8K,
    };
    let mut out = Vec::with_capacity(mode.cells());
    let mut reg = 0usize;
    for i in 0..n {
        match i {
            0 | 1 => reg = 0,
            2 => reg = 1,
            _ => {
                let bit = match mode {
                    Mode::M2k => (reg ^ (reg >> 3)) & 1,
                    Mode::M8k => (reg ^ (reg >> 1) ^ (reg >> 4) ^ (reg >> 6)) & 1,
                };
                reg = ((reg >> 1) | (bit << (nr - 2))) & ((1 << nr) - 1);
            }
        }
        let mut shuffled = 0usize;
        for (k, &to) in perm.iter().enumerate() {
            shuffled |= ((reg >> k) & 1) << to;
        }
        let h = ((i & 1) << (nr - 1)) | shuffled;
        if h < mode.cells() {
            out.push(h as u32);
        }
    }
    debug_assert_eq!(out.len(), mode.cells());
    out
}

/// Which of the v substreams the kth coded bit of a group goes to, EN 300 744
/// clause 4.3.4, non-hierarchical.
fn demux(k: usize, v: usize) -> usize {
    (k / (v / 2)) + 2 * (k % (v / 2))
}

/// The inner layers of one multiplex: cells in, coded soft bits out, and the
/// same thing backwards for a transmitter.
pub struct Inner {
    mode: Mode,
    constellation: Constellation,
    h: Vec<u32>,
    /// Every level one axis of the constellation can take, with the bits that
    /// put it there, which is what a soft decision is measured against.
    levels: Vec<f32>,
    cells: Vec<f32>,
    permuted: Vec<f32>,
}

impl Inner {
    pub fn new(mode: Mode, constellation: Constellation) -> Self {
        let m = constellation.bits() / 2;
        let levels = (0..1usize << m).map(|word| axis(word, m) * constellation.norm()).collect();
        Self {
            mode,
            constellation,
            h: permutation(mode),
            levels,
            cells: Vec::new(),
            permuted: Vec::new(),
        }
    }

    pub fn constellation(&self) -> Constellation {
        self.constellation
    }

    /// Coded bits one symbol carries.
    pub fn bits_per_symbol(&self) -> usize {
        self.mode.cells() * self.constellation.bits()
    }

    /// Read one symbol. `index` is its place in the frame, which says which
    /// way round the symbol interleaver ran, and `csi` is the channel power
    /// each cell was read through, which weights its bits: a cell in a notch
    /// says less than one on a peak, and the Viterbi should be told so.
    pub fn demodulate(&mut self, cells: &[C32], csi: &[f32], index: usize, out: &mut Vec<f32>) {
        let v = self.constellation.bits();
        let n = self.mode.cells();
        assert_eq!(cells.len(), n, "a symbol is its own cells");
        self.cells.clear();
        self.cells.resize(n * v, 0.0);
        for (w, (cell, weight)) in cells.iter().zip(csi).enumerate() {
            let m = v / 2;
            for e in 0..v {
                let value = if e % 2 == 0 { cell.re } else { cell.im };
                self.cells[w * v + e] = self.llr(value, e / 2, m) * weight;
            }
        }

        // Symbol deinterleaver: the transmitter ran the permutation one way
        // on even symbols and the other way on odd ones.
        self.permuted.clear();
        self.permuted.resize(n * v, 0.0);
        for q in 0..n {
            let (from, to) =
                if index % 2 == 1 { (q, self.h[q] as usize) } else { (self.h[q] as usize, q) };
            let src = &self.cells[from * v..from * v + v];
            self.permuted[to * v..to * v + v].copy_from_slice(src);
        }

        // Bit deinterleaver, within each block of 126 cells.
        for block in 0..n / BLOCK {
            let base = block * BLOCK;
            for i in 0..BLOCK {
                for k in 0..v {
                    let e = demux(k, v);
                    let w = (i + BLOCK - ROTATE[e] % BLOCK) % BLOCK;
                    out.push(self.permuted[(base + w) * v + e]);
                }
            }
        }
    }

    /// The soft value of bit `j` of one axis, positive for a zero bit: the
    /// distance to the nearest level that would carry a one, less the
    /// distance to the nearest that would carry a zero.
    fn llr(&self, value: f32, j: usize, m: usize) -> f32 {
        let mut near = [f32::INFINITY; 2];
        for (word, level) in self.levels.iter().enumerate() {
            let bit = (word >> (m - 1 - j)) & 1;
            let d = (value - level) * (value - level);
            near[bit] = near[bit].min(d);
        }
        near[1] - near[0]
    }

    /// The transmit side: coded bits to the cells of one symbol.
    pub fn modulate(&mut self, bits: &[u8], index: usize, out: &mut Vec<C32>) {
        let v = self.constellation.bits();
        let n = self.mode.cells();
        assert_eq!(bits.len(), n * v, "a symbol takes exactly its bits");
        let mut words = vec![0u8; n * v];
        for block in 0..n / BLOCK {
            let base = block * BLOCK;
            for i in 0..BLOCK {
                for k in 0..v {
                    let e = demux(k, v);
                    let w = (i + BLOCK - ROTATE[e] % BLOCK) % BLOCK;
                    words[(base + w) * v + e] = bits[(base + i) * v + k];
                }
            }
        }
        out.reserve(n);
        let start = out.len();
        out.resize(start + n, C32::default());
        for q in 0..n {
            let (from, to) =
                if index % 2 == 1 { (self.h[q] as usize, q) } else { (q, self.h[q] as usize) };
            out[start + to] = super::map(&words[from * v..from * v + v], self.constellation);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The permutation is a permutation: every cell position is used once.
    #[test]
    fn the_symbol_interleaver_permutes_and_drops_nothing() {
        for mode in Mode::ALL {
            let h = permutation(mode);
            assert_eq!(h.len(), mode.cells());
            let mut seen = vec![false; mode.cells()];
            for &q in &h {
                assert!(!seen[q as usize], "{} repeats {q}", mode.label());
                seen[q as usize] = true;
            }
        }
    }

    /// The first five cells of a 2K symbol go to 0, 1024, 16, 1025 and 128.
    /// The register that says so is EN 300 744 clause 4.3.4.1, and these are
    /// the values gnuradio's `dvbt_symbol_inner_interleaver` produces for the
    /// same mode, which is the implementation every other receiver is checked
    /// against.
    #[test]
    fn the_permutation_starts_where_the_register_puts_it() {
        assert_eq!(&permutation(Mode::M2k)[..5], &[0, 1024, 16, 1025, 128]);
        assert_eq!(&permutation(Mode::M8k)[..5], &[0, 4096, 128, 4128, 2048]);
    }

    /// Bits through both interleavers and the mapper, and back, are the bits
    /// that went in, at every constellation and both modes.
    #[test]
    fn the_inner_layers_round_trip() {
        for mode in Mode::ALL {
            for constellation in Constellation::ALL {
                let mut inner = Inner::new(mode, constellation);
                let n = inner.bits_per_symbol();
                let mut state = 99u32;
                let bits: Vec<u8> = (0..n)
                    .map(|_| {
                        state = state.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
                        ((state >> 19) & 1) as u8
                    })
                    .collect();
                for index in [0usize, 1] {
                    let mut cells = Vec::new();
                    inner.modulate(&bits, index, &mut cells);
                    assert_eq!(cells.len(), mode.cells());
                    let csi = vec![1.0f32; mode.cells()];
                    let mut soft = Vec::new();
                    inner.demodulate(&cells, &csi, index, &mut soft);
                    let got: Vec<u8> = soft.iter().map(|&s| u8::from(s < 0.0)).collect();
                    assert_eq!(
                        got,
                        bits,
                        "{} {} symbol {index} did not come back",
                        mode.label(),
                        constellation.label()
                    );
                }
            }
        }
    }
}
