//! The DAB front end: find the frame by its null, time it by its phase
//! reference, and hand up the soft bits each symbol carries.
//!
//! A frame is found twice over. The null symbol is the transmitter off the
//! air for 1.3 ms, so the quietest window of that length in a frame's worth
//! of samples says where a frame begins to within a symbol. The phase
//! reference symbol that follows it is a sequence the standard writes out,
//! so correlating against it says where the frame begins to within a sample
//! and, searched over a few carriers either side, what whole number of
//! carriers the tuner is off by.
//!
//! What is left is the fractional part of the frequency error, which the
//! cyclic prefix gives: a symbol's prefix is a copy of its tail, so the
//! phase between them is the error over a useful part.
//!
//! Nothing here estimates a channel, because differential QPSK does not need
//! one. A timing offset inside the guard is a phase ramp across the carriers
//! that two consecutive symbols share, and the difference between them
//! cancels it, which is why the read window sits back inside the guard and
//! nothing compensates for it afterwards.

use super::{Mode, bin, interleave, phase_reference};
use common::C32;
use rustfft::{Fft, FftPlanner};
use std::f64::consts::TAU;
use std::sync::Arc;

/// One symbol of a frame, read.
#[derive(Clone, Debug)]
pub struct Symbol {
    /// Where it sits in the frame: 1 is the first symbol after the phase
    /// reference, and the fast information channel is the first few.
    pub index: usize,
    /// The soft bits it carries, deinterleaved: 2K of them, positive for a
    /// zero bit. The first K are the real axis and the rest the imaginary,
    /// which is the order the standard's own bit numbering puts them in.
    pub soft: Vec<f32>,
    /// How far the differential points landed from where they should, as a
    /// signal to noise ratio in dB.
    pub snr_db: f32,
}

/// How far either side of the expected carrier the integer frequency search
/// looks. Mode I carriers are 1 kHz apart, so this is +-8 kHz, which is far
/// more than a tuner told a band III block is ever out by.
const CARRIER_SEARCH: i32 = 8;

/// How much quieter than the frame around it the null has to be before it is
/// taken for a null. Measured on a synthesised Mode I frame with no noise the
/// ratio is zero; with the transmitter's own shoulders and a receiver's floor
/// it sits around 0.1, and a frame with no null at all scores 1.
const NULL_RATIO: f32 = 0.5;

/// How strongly the phase reference has to correlate, against the energy of
/// the symbol it was read from, before a frame is believed.
const LOCK_THRESHOLD: f32 = 0.25;

/// The DAB front end.
pub struct Dab {
    mode: Mode,
    buf: Vec<C32>,
    /// Where the phase reference symbol of the next frame starts in `buf`,
    /// once a frame has been found.
    start: Option<usize>,
    /// Carriers the tuner is off by.
    shift: i32,
    /// Radians a sample, the fractional frequency error.
    step: f64,
    fft: Arc<dyn Fft<f32>>,
    ifft: Arc<dyn Fft<f32>>,
    scratch: Vec<C32>,
    grid: Vec<C32>,
    prev: Vec<C32>,
    reference: Vec<C32>,
    /// Bit position to carrier, from the frequency interleaver.
    map: Vec<i32>,
    /// Frames read since the last lock, which is what says a signal is there
    /// at all.
    frames: u64,
}

impl Default for Dab {
    fn default() -> Self {
        Self::new(Mode::I)
    }
}

impl Dab {
    /// A receiver for one transmission mode.
    ///
    /// The mode is not searched for. Mode I is what every European terrestrial
    /// ensemble transmits, and the others need their own phase reference
    /// tables before there is anything to correlate against.
    pub fn new(mode: Mode) -> Self {
        let mut planner = FftPlanner::new();
        Self {
            buf: Vec::new(),
            start: None,
            shift: 0,
            step: 0.0,
            fft: planner.plan_fft_forward(mode.fft()),
            ifft: planner.plan_fft_inverse(mode.fft()),
            scratch: Vec::new(),
            grid: Vec::new(),
            prev: Vec::new(),
            reference: phase_reference(mode).unwrap_or_default(),
            map: interleave(mode),
            frames: 0,
            mode,
        }
    }

    pub fn mode(&self) -> Mode {
        self.mode
    }

    /// Whether a frame has been found and is being followed.
    pub fn locked(&self) -> bool {
        self.start.is_some()
    }

    /// Frames read since the last time the receiver had to search again.
    pub fn frames(&self) -> u64 {
        self.frames
    }

    /// The frequency error the receiver is correcting, in hertz.
    pub fn offset_hz(&self) -> f64 {
        self.shift as f64 * self.mode.spacing_hz() - self.step / TAU * super::RATE_HZ
    }

    /// Read what `x` holds, appending every symbol of every whole frame in it.
    pub fn push(&mut self, x: &[C32], out: &mut Vec<Symbol>) {
        self.buf.extend_from_slice(x);
        loop {
            if self.start.is_none() && !self.find_frame() {
                break;
            }
            if !self.frame(out) {
                break;
            }
        }
        self.trim();
    }

    /// Drop what nothing will read again.
    fn trim(&mut self) {
        let keep = match self.start {
            Some(s) => s,
            // Unlocked, the search needs two frames whole, so only what is
            // beyond that goes.
            None => self.buf.len().saturating_sub(2 * self.mode.frame()),
        };
        if keep == 0 {
            return;
        }
        self.buf.drain(..keep);
        if let Some(s) = &mut self.start {
            *s -= keep;
        }
    }

    /// Find the null symbol, and with it where a frame starts. True once a
    /// frame has been found and timed.
    fn find_frame(&mut self) -> bool {
        let frame = self.mode.frame();
        let null = self.mode.null();
        // Two frames, so that one whole null and the frame it opens are
        // certainly inside the window whatever the stream started on.
        if self.buf.len() < 2 * frame {
            return false;
        }
        let window = frame + null;
        let mut power = Vec::with_capacity(window + 1);
        power.push(0.0f64);
        for s in &self.buf[..window] {
            let last = *power.last().unwrap_or(&0.0);
            power.push(last + s.norm_sqr() as f64);
        }
        let total = power[window];
        if total <= 0.0 {
            self.buf.drain(..frame);
            return false;
        }
        let mean = total / window as f64;
        let (at, quiet) = (0..=frame)
            .map(|s| (s, power[s + null] - power[s]))
            .min_by(|a, b| a.1.total_cmp(&b.1))
            .unwrap_or((0, 0.0));
        if quiet / (null as f64 * mean) > NULL_RATIO as f64 {
            // No null anywhere in a frame: whatever is here is not DAB.
            self.buf.drain(..frame);
            return false;
        }
        let start = at + null;
        if start + frame > self.buf.len() {
            return false;
        }
        match self.time_frame(start) {
            Some((start, shift)) => {
                self.start = Some(start);
                self.shift = shift;
                self.step = 0.0;
                self.frames = 0;
                true
            }
            None => {
                self.buf.drain(..(at + null).min(self.buf.len()));
                false
            }
        }
    }

    /// Time a frame exactly off its phase reference symbol, and say how many
    /// whole carriers the tuner is out by.
    ///
    /// The product of the received carriers with the reference is the channel
    /// the symbol came through, so its inverse transform is an impulse at the
    /// timing error. That the impulse is sharp at all is the evidence a frame
    /// is there: noise transforms to noise.
    fn time_frame(&mut self, start: usize) -> Option<(usize, i32)> {
        let n = self.mode.fft();
        let g = self.mode.guard();
        if self.reference.is_empty() || start + g + n > self.buf.len() {
            return None;
        }
        self.grid.clear();
        self.grid.extend_from_slice(&self.buf[start + g..start + g + n]);
        let energy: f32 = self.grid.iter().map(|s| s.norm_sqr()).sum::<f32>().max(1e-20);
        self.scratch.resize(self.fft.get_inplace_scratch_len(), C32::default());
        self.fft.process_with_scratch(&mut self.grid, &mut self.scratch);

        let half = self.mode.carriers() as i32 / 2;
        let mut best: Option<(f32, usize, i32)> = None;
        let mut work = vec![C32::default(); n];
        for shift in -CARRIER_SEARCH..=CARRIER_SEARCH {
            work.iter_mut().for_each(|c| *c = C32::default());
            for k in -half..=half {
                if k == 0 {
                    continue;
                }
                let b = bin(self.mode, k);
                work[b] = self.grid[bin(self.mode, k + shift)] * self.reference[b].conj();
            }
            self.ifft.process_with_scratch(&mut work, &mut self.scratch);
            let (at, peak) = work
                .iter()
                .enumerate()
                .map(|(i, c)| (i, c.norm_sqr()))
                .max_by(|a, b| a.1.total_cmp(&b.1))
                .unwrap_or((0, 0.0));
            // Against the most a correlation of this symbol with this many
            // carriers could reach, so a matched frame scores one and noise
            // scores about 1/K whatever the level.
            let score = peak / (n as f32 * energy * self.mode.carriers() as f32);
            if best.is_none_or(|b| score > b.0) {
                best = Some((score, at, shift));
            }
        }
        let (score, at, shift) = best?;
        if score < LOCK_THRESHOLD {
            return None;
        }
        // The impulse sits at the delay the window is late by, wrapped.
        let late = if at < n / 2 { at as isize } else { at as isize - n as isize };
        let timed = start as isize + late;
        if timed < 0 || timed as usize + self.mode.frame() > self.buf.len() {
            return None;
        }
        Some((timed as usize, shift))
    }

    /// Read one frame if there is one whole. False when more samples are
    /// needed.
    fn frame(&mut self, out: &mut Vec<Symbol>) -> bool {
        let Some(start) = self.start else { return false };
        let (n, g, ts) = (self.mode.fft(), self.mode.guard(), self.mode.symbol());
        let need = start + self.mode.symbols() * ts;
        if self.buf.len() < need {
            return false;
        }
        self.estimate_cfo(start);
        // Back inside the guard, so a timing estimate a sample late still
        // reads one symbol rather than two. Differential demodulation takes
        // the phase ramp this costs back out for nothing.
        let backoff = g / 4;
        self.prev.clear();
        for l in 0..self.mode.symbols() {
            let at = start + l * ts + g - backoff;
            self.transform(at);
            if l > 0 && !self.prev.is_empty() {
                let symbol = self.differential(l);
                out.push(symbol);
            }
            std::mem::swap(&mut self.prev, &mut self.grid);
        }
        self.frames += 1;

        // The next frame is one frame on, re-timed off its own phase
        // reference: a transmitter's clock and a receiver's differ by enough
        // to walk out of the guard over a few seconds.
        let next = start + self.mode.frame();
        self.start = match self.buf.len() >= next + g + n {
            true => match self.time_frame(next) {
                Some((timed, shift)) => {
                    self.shift = shift;
                    Some(timed)
                }
                // Nothing where the next frame should be. What was read
                // stands; the search starts again on what follows.
                None => None,
            },
            false => Some(next),
        };
        self.start.is_some()
    }

    /// The fractional frequency error, from the prefix of every symbol in the
    /// frame against its own tail.
    fn estimate_cfo(&mut self, start: usize) {
        let (n, g, ts) = (self.mode.fft(), self.mode.guard(), self.mode.symbol());
        let mut c = C32::default();
        for l in 0..self.mode.symbols() {
            let at = start + l * ts;
            if at + g + n > self.buf.len() {
                break;
            }
            for i in 0..g {
                c += self.buf[at + i] * self.buf[at + i + n].conj();
            }
        }
        if c.norm() > 0.0 {
            // The prefix leads its own tail by a useful part, so the phase
            // between them is the error over that span and the correction is
            // its opposite.
            self.step = (c.im as f64).atan2(c.re as f64) / n as f64;
        }
    }

    /// Radians a sample of correction: the fraction of a carrier the prefix
    /// measured and the whole carriers the phase reference counted.
    ///
    /// The whole carriers are taken out here rather than by reading the
    /// carriers a few bins along, because an offset of whole carriers is
    /// still a rotation of the constellation between one symbol and the next:
    /// three carriers of a Mode I signal turn the differential point by 265
    /// degrees a symbol, which reads half the bits the wrong way round.
    fn rotation(&self) -> f64 {
        self.step - TAU * self.shift as f64 / self.mode.fft() as f64
    }

    /// The useful part at `at`, frequency corrected and transformed.
    fn transform(&mut self, at: usize) {
        let n = self.mode.fft();
        let step = self.rotation();
        self.grid.resize(n, C32::default());
        for (i, out) in self.grid.iter_mut().enumerate() {
            let a = (at + i) as f64 * step;
            let (s, c) = a.sin_cos();
            *out = self.buf[at + i] * C32::new(c as f32, s as f32);
        }
        self.scratch.resize(self.fft.get_inplace_scratch_len(), C32::default());
        self.fft.process_with_scratch(&mut self.grid, &mut self.scratch);
    }

    /// The bits of the symbol in `grid` against the one in `prev`.
    fn differential(&self, index: usize) -> Symbol {
        let k = self.mode.carriers();
        let mut soft = vec![0.0f32; 2 * k];
        let mut error = 0.0f32;
        for (n, &carrier) in self.map.iter().enumerate() {
            let b = bin(self.mode, carrier);
            let y = self.grid[b] * self.prev[b].conj();
            let mag = y.norm();
            let (re, im) = match mag > 0.0 {
                true => (y.re / mag, y.im / mag),
                false => (0.0, 0.0),
            };
            soft[n] = re;
            soft[n + k] = im;
            // How far the point is from the corner it decided on, which for
            // a unit point is twice the noise on each axis.
            let a = std::f32::consts::FRAC_1_SQRT_2;
            error += (re.abs() - a).powi(2) + (im.abs() - a).powi(2);
        }
        let mean = error / k as f32;
        let snr_db = match mean > 0.0 {
            true => -10.0 * mean.log10(),
            false => 60.0,
        };
        Symbol { index, soft, snr_db: snr_db.clamp(-20.0, 60.0) }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dab::tx::Modulator;

    /// A cheap repeatable bit stream, so a test can say which bits it sent.
    fn stream(n: usize, seed: u32) -> Vec<u8> {
        let mut state = seed;
        (0..n)
            .map(|_| {
                state = state.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
                ((state >> 24) & 1) as u8
            })
            .collect()
    }

    fn noise(n: usize, seed: u32, level: f32) -> Vec<C32> {
        let mut state = seed;
        let mut next = || {
            state = state.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            (state >> 16) as i16 as f32 / 32768.0 * level
        };
        (0..n).map(|_| C32::new(next(), next())).collect()
    }

    /// Two frames on the air, read back: every symbol of both, in order, with
    /// every bit as it was sent.
    #[test]
    fn a_frame_reads_back_bit_for_bit() {
        let mode = Mode::I;
        let mut tx = Modulator::new(mode);
        let bits: Vec<u8> = stream(2 * tx.bits_per_frame(), 7);
        let mut air = noise(mode.frame() / 2, 3, 0.001);
        for f in 0..2 {
            tx.frame(&bits[f * tx.bits_per_frame()..(f + 1) * tx.bits_per_frame()], &mut air);
        }
        air.extend(noise(mode.frame(), 11, 0.001));

        let mut rx = Dab::new(mode);
        let mut out = Vec::new();
        rx.push(&air, &mut out);
        assert_eq!(out.len(), 2 * (mode.symbols() - 1), "150 symbols off two frames");
        assert_eq!(rx.frames(), 2);
        assert_eq!(out[0].index, 1);
        assert_eq!(out[74].index, 75);
        let read: Vec<u8> =
            out.iter().flat_map(|s| s.soft.iter().map(|&v| u8::from(v < 0.0))).collect();
        assert_eq!(read.len(), bits.len());
        assert_eq!(read, bits, "every bit of both frames");
        assert!(out.iter().all(|s| s.snr_db > 40.0), "a clean frame reads clean");
    }

    /// The same frame with the tuner three carriers off and a fifth of a
    /// carrier of drift on top, which is 3.2 kHz of a 1 kHz raster.
    #[test]
    fn a_mistuned_frame_still_reads() {
        let mode = Mode::I;
        let mut tx = Modulator::new(mode);
        let bits: Vec<u8> = stream(tx.bits_per_frame(), 21);
        let mut air = noise(mode.frame() / 3, 5, 0.001);
        tx.frame(&bits, &mut air);
        air.extend(noise(mode.frame(), 13, 0.001));
        let offset = 3.2 * mode.spacing_hz();
        for (i, s) in air.iter_mut().enumerate() {
            let a = TAU * offset * i as f64 / super::super::RATE_HZ;
            let (sin, cos) = a.sin_cos();
            *s *= C32::new(cos as f32, sin as f32);
        }

        let mut rx = Dab::new(mode);
        let mut out = Vec::new();
        rx.push(&air, &mut out);
        assert_eq!(out.len(), mode.symbols() - 1);
        let read: Vec<u8> =
            out.iter().flat_map(|s| s.soft.iter().map(|&v| u8::from(v < 0.0))).collect();
        let wrong = read.iter().zip(&bits).filter(|(a, b)| a != b).count();
        assert_eq!(wrong, 0, "{wrong} bits wrong at 3.2 kHz off");
        assert!(
            (rx.offset_hz() - offset).abs() < 60.0,
            "the receiver measured {} Hz of {offset}",
            rx.offset_hz()
        );
    }

    /// Minutes of noise: nothing locks, and no symbol comes out.
    #[test]
    fn noise_locks_on_nothing() {
        let mode = Mode::I;
        let mut rx = Dab::new(mode);
        let mut out = Vec::new();
        // Two minutes of air at the standard's rate, a frame at a time.
        for block in 0..1250 {
            let air = noise(mode.frame(), 100 + block, 0.05);
            rx.push(&air, &mut out);
        }
        assert_eq!(out.len(), 0);
        assert!(!rx.locked());
        assert_eq!(rx.frames(), 0);
    }
}
