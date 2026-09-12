//! Differentially encoded 8-PSK, the waveform VDL Mode 2 keys.
//!
//! Three bits a symbol, carried in the phase step between one symbol and the
//! next, so a receiver needs no absolute phase reference and an aircraft's
//! doppler costs a slope rather than a lock. The burst begins with a known
//! preamble of phase steps, and finding it is the whole of acquisition: the
//! error between the received steps and the expected ones is flat when the
//! alignment is right, sloped when there is a frequency offset, and noise
//! otherwise.
//!
//! Parameterised rather than written for VDL2, because the next 8-PSK link
//! wants the same code with a different preamble and rate. What is specific
//! to VDL2 is [`D8pskConfig::VDL2`] and the block layer in `decode::vdl2`.
//!
//! Ported from the demodulator in Tomasz Lemiech's dumpvdl2, whose sync
//! metric (the phase-error variance after a straight-line fit) and parabola
//! interpolation of the sync instant are what make it work at the levels an
//! aircraft arrives at.

use common::C32;
use std::f32::consts::PI;

/// The shape of one 8-PSK link.
#[derive(Clone, Copy, Debug)]
pub struct D8pskConfig {
    /// Symbols per second.
    pub baud: f64,
    /// Samples per symbol the demodulator runs at; the input is resampled to
    /// `baud * sps` before it gets here.
    pub sps: usize,
    /// The preamble's cumulative phase at each symbol, in radians.
    pub preamble: &'static [f32],
    /// How many symbols of gray-coded tribits to emit before giving up on a
    /// burst that never ends.
    pub max_symbols: usize,
}

/// The VDL Mode 2 preamble: 16 symbols of known phase at 10500 symbols a
/// second, from ICAO Annex 10 by way of dumpvdl2.
pub const VDL2_PREAMBLE: [f32; 16] = [
    0.0,
    3.0 * PI / 4.0,
    -3.0 * PI / 4.0,
    PI / 4.0,
    PI / 4.0,
    2.0 * PI / 4.0,
    0.0,
    PI,
    -3.0 * PI / 4.0,
    PI,
    -2.0 * PI / 4.0,
    3.0 * PI / 4.0,
    PI / 4.0,
    -2.0 * PI / 4.0,
    -3.0 * PI / 4.0,
    0.0,
];

impl D8pskConfig {
    pub const VDL2: D8pskConfig = D8pskConfig {
        baud: 10_500.0,
        sps: 10,
        preamble: &VDL2_PREAMBLE,
        // A transmission is at most 0x3fff bits, three bits a symbol.
        max_symbols: 0x3fff / 3 + 64,
    };

    /// The sample rate this demodulator must be fed at.
    pub fn rate(&self) -> f64 {
        self.baud * self.sps as f64
    }
}

/// What the demodulator hands back: a burst's gray-decoded tribits, most
/// significant bit first, with the level it was received at.
#[derive(Clone, Debug)]
pub struct Burst {
    /// Three bits per symbol, in transmission order.
    pub bits: Vec<bool>,
    /// Mean symbol power over the burst, linear.
    pub power: f32,
    /// The estimated carrier offset, as a fraction of the symbol rate.
    pub freq_error: f32,
    /// The sample the preamble was found at, counted from the first sample
    /// the demodulator ever saw.
    pub at_sample: u64,
}

/// Gray code: a symbol's three bits from its phase step.
const GRAY: [u8; 8] = [0, 1, 3, 2, 6, 7, 5, 4];

/// How often the sync metric is evaluated, in samples. Every third is what
/// dumpvdl2 uses and is enough at ten samples a symbol.
const SYNC_SKIP: usize = 3;

/// Below this the phase error over the preamble is a burst rather than noise.
/// It is a sum of sixteen squared radian errors, so about half a radian of
/// scatter per symbol.
const SYNC_THRESHOLD: f32 = 4.0;

const PHERR_MAX: f32 = 1000.0;

#[derive(Clone, Copy, PartialEq, Debug)]
enum State {
    Hunting,
    Reading,
}

pub struct D8pskDemod {
    cfg: D8pskConfig,
    /// A preamble's worth of phases, as a ring.
    sync: Vec<f32>,
    at: usize,
    /// The last three sync metrics, newest first: a burst is the moment the
    /// metric passes through its minimum.
    pherr: [f32; 3],
    /// Regression weights over the preamble, precomputed.
    lr_x: Vec<f32>,
    lr_denom: f32,
    state: State,
    clock: isize,
    skip: usize,
    prev_phi: f32,
    dphi: f32,
    prev_dphi: f32,
    bits: Vec<bool>,
    power: f32,
    power_n: u32,
    at_sample: u64,
    seen: u64,
}

impl D8pskDemod {
    pub fn new(cfg: D8pskConfig) -> Self {
        let n = cfg.preamble.len();
        let mean = (0..n).map(|i| i as f32).sum::<f32>() / n as f32;
        let lr_x: Vec<f32> = (0..n).map(|i| i as f32 - mean).collect();
        let lr_denom = lr_x.iter().map(|x| x * x).sum();
        Self {
            cfg,
            sync: vec![0.0; n * cfg.sps],
            at: 0,
            pherr: [PHERR_MAX; 3],
            lr_x,
            lr_denom,
            state: State::Hunting,
            clock: 0,
            skip: 0,
            prev_phi: 0.0,
            dphi: 0.0,
            prev_dphi: 0.0,
            bits: Vec::new(),
            power: 0.0,
            power_n: 0,
            at_sample: 0,
            seen: 0,
        }
    }

    pub fn reset(&mut self) {
        let cfg = self.cfg;
        let seen = self.seen;
        *self = Self::new(cfg);
        self.seen = seen;
    }

    /// Feed baseband at `cfg.rate()`. Bursts are handed to `out` as they end,
    /// which is when `done` says the block layer has what it asked for.
    ///
    /// `done` is given the bits so far and returns true when the burst is
    /// complete: the block layer knows the length only after it has read the
    /// header, so the demodulator cannot decide this for itself.
    pub fn process(
        &mut self,
        input: &[C32],
        done: &mut dyn FnMut(&[bool]) -> bool,
        out: &mut Vec<Burst>,
    ) {
        for &x in input {
            self.seen += 1;
            match self.state {
                State::Hunting => {
                    self.at = (self.at + 1) % self.sync.len();
                    self.sync[self.at] = x.im.atan2(x.re);
                    self.skip += 1;
                    if self.skip < SYNC_SKIP {
                        continue;
                    }
                    self.skip = 0;
                    if self.found() {
                        self.state = State::Reading;
                        self.at_sample = self.seen;
                        self.bits.clear();
                        self.power = 0.0;
                        self.power_n = 0;
                    }
                }
                State::Reading => {
                    self.clock += 1;
                    if (self.clock as usize) < self.cfg.sps {
                        continue;
                    }
                    self.clock = 0;
                    let phi = x.im.atan2(x.re);
                    let mut step = phi - self.prev_phi - self.dphi;
                    if step < 0.0 {
                        step += 2.0 * PI;
                    } else if step > 2.0 * PI {
                        step -= 2.0 * PI;
                    }
                    let idx = ((step / (PI / 4.0)).round() as i32).rem_euclid(8) as usize;
                    self.prev_phi = phi;
                    let p = x.re * x.re + x.im * x.im;
                    self.power = (self.power * self.power_n as f32 + p) / (self.power_n + 1) as f32;
                    self.power_n += 1;
                    let sym = GRAY[idx];
                    for b in (0..3).rev() {
                        self.bits.push(sym >> b & 1 == 1);
                    }
                    if done(&self.bits) || self.bits.len() >= self.cfg.max_symbols * 3 {
                        out.push(Burst {
                            bits: std::mem::take(&mut self.bits),
                            power: self.power,
                            freq_error: self.dphi / (2.0 * PI),
                            at_sample: self.at_sample,
                        });
                        self.state = State::Hunting;
                        self.pherr = [PHERR_MAX; 3];
                    }
                }
            }
        }
    }

    /// Whether the last preamble's worth of phases is the preamble, and if so
    /// where its symbol clock starts.
    fn found(&mut self) -> bool {
        let n = self.cfg.preamble.len();
        let sps = self.cfg.sps;
        let len = self.sync.len();
        let phase = |i: usize| self.sync[(self.at + (i + 1) * sps) % len];

        // The error between what arrived and the preamble, unwrapped: an
        // aligned preamble leaves a constant, whatever the starting phase.
        let mut err = vec![0.0f32; n];
        let mut prev = phase(0) - self.cfg.preamble[0];
        let mut unwrap = 0.0;
        err[0] = prev;
        let mut mean = prev;
        for i in 1..n {
            let cur = phase(i) - self.cfg.preamble[i];
            let d = cur - prev;
            prev = cur;
            if d > PI {
                unwrap -= 2.0 * PI;
            } else if d < -PI {
                unwrap += 2.0 * PI;
            }
            err[i] = cur + unwrap;
            mean += err[i];
        }
        mean /= n as f32;
        for e in err.iter_mut() {
            *e -= mean;
        }

        // A carrier offset tilts that constant into a ramp, so fit a line and
        // keep its slope: it is the frequency error, and it is what the
        // symbol decisions are corrected by afterwards.
        let slope: f32 =
            self.lr_x.iter().zip(&err).map(|(x, e)| x * e).sum::<f32>() / self.lr_denom;
        let metric: f32 = err.iter().zip(&self.lr_x).map(|(e, x)| (e - slope * x).powi(2)).sum();

        self.pherr[0] = metric;
        if self.pherr[1] < SYNC_THRESHOLD && self.pherr[0] > self.pherr[1] {
            // The metric has just turned upwards, so the minimum was one step
            // back. Three points fix a parabola, and its vertex is the sync
            // instant to better than a sample.
            let vertex = vertex(
                self.clock as f32,
                SYNC_SKIP as f32,
                self.pherr[2],
                self.pherr[1],
                self.pherr[0],
            );
            self.clock = -vertex.round() as isize;
            let sp = (self.at as isize - self.clock).rem_euclid(len as isize) as usize;
            self.prev_phi = self.sync[sp];
            self.dphi = self.prev_dphi;
            self.pherr[1] = PHERR_MAX;
            self.pherr[2] = PHERR_MAX;
            return true;
        }
        self.pherr[2] = self.pherr[1];
        self.pherr[1] = self.pherr[0];
        self.prev_dphi = slope;
        false
    }
}

/// The x of the vertex of the parabola through three points `d` apart, the
/// last of them at `x`.
fn vertex(x: f32, d: f32, y1: f32, y2: f32, y3: f32) -> f32 {
    let denom = d * 2.0 * d * -d;
    let a = (x * (y2 - y1) + (x - d) * (y1 - y3) + (x - 2.0 * d) * (y3 - y2)) / denom;
    let b = (x * x * (y1 - y2)
        + (x - d) * (x - d) * (y3 - y1)
        + (x - 2.0 * d) * (x - 2.0 * d) * (y2 - y3))
        / denom;
    -b / (2.0 * a)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_receiver_is_built_for_the_waveform_it_was_given() {
        let cfg = D8pskConfig::VDL2;
        assert_eq!(cfg.rate(), 105_000.0);
        assert_eq!(cfg.preamble.len(), 16);
    }

    /// Gray coding is what keeps a phase slipped by one step to one wrong
    /// bit, which is the reason the mapping is not the identity.
    #[test]
    fn neighbouring_symbols_differ_in_one_bit() {
        for i in 0..8 {
            let a = GRAY[i];
            let b = GRAY[(i + 1) % 8];
            assert_eq!((a ^ b).count_ones(), 1, "{a:03b} and {b:03b} are not neighbours");
        }
    }
}
