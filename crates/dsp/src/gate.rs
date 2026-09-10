//! Which channels of a span are worth reading this block.
//!
//! A front end that reads fixed channels off a span mixes and filters each of
//! them out of every block, whether or not anything is transmitting there.
//! That is the larger half of what a channel costs, and on a quiet band it
//! buys nothing: measured at 61.44 MS/s over a busy 2.4 GHz band, the two BLE
//! advertising channels the span holds cost 1.77 ms of a 2.13 ms block, most
//! of it a 449 tap channel filter run before a single bit is sliced.
//!
//! So: one coarse transform per block says how much power is in each
//! channel's bins, and a channel at its own floor is skipped. The transform
//! is shared by every channel of one front end, which is what makes it cheap
//! enough to be worth having.
//!
//! # The loudest window against the typical one
//!
//! A block is milliseconds and the transmissions this gates are shorter than
//! that: a BLE advertisement is 128 us and a DroneID burst 720. Averaged over
//! the block, a burst that fills a twentieth of it is 13 dB down and the
//! channel never wakes. So the block is sampled as windows and each channel
//! takes two numbers from them: the loudest window, which is the burst, and
//! the median window, which is the channel between bursts.
//!
//! Both come from the same block, so a channel that has something on it in
//! every block still has a floor of noise rather than a floor of signal,
//! which a floor remembered across blocks does not: on a capture read in
//! chunks of a megasample, every chunk holds a burst and a remembered floor
//! becomes the burst. The median is the right one of the two robust
//! statistics here, because the quietest of a few dozen noise windows is far
//! below their mean and would put the gate a factor of five out.
//!
//! The windows are spaced by the shortest burst the front end must not miss,
//! not by a count, since that is the property that decides whether the gate
//! is a saving or a way of losing packets.

use common::C32;
use rustfft::{Fft, FftPlanner};
use std::sync::Arc;

/// How fine the span is measured, in hertz a bin.
///
/// Not a bin count, because the two rates this runs at are three times apart
/// and what decides the gate is bins per channel rather than bins per span: a
/// band of a dozen bins is twenty-four degrees of freedom, which is where the
/// noise's own spread is small enough for a 6 dB test. Measured on the
/// 61.44 MS/s capture of a busy 2.4 GHz band, three bins across a BLE channel
/// left the gate open on noise and saved nothing at all, and a dozen took the
/// front end from 1.77 ms a block to 1.15.
const RESOLUTION_HZ: f64 = 120_000.0;

/// And the transform is bounded either way: below this the bins are wider
/// than a channel, and above it the transform costs more than the mixing it
/// decides about. Measured on the same capture, 1024 bins bought nothing 512
/// did not, and at 20 MS/s they cost twice as much for the same channels
/// awake.
const MIN_BINS: usize = 64;
const MAX_BINS: usize = 512;

/// How far over the channel's own median a window has to stand before the
/// channel is read, as a power ratio.
///
/// Six decibels, which is what the measurement's own spread costs: a band of
/// a dozen bins is twenty-four degrees of freedom, so the loudest of a few
/// dozen windows of pure noise stands a couple of decibels over their median,
/// and hundreds of them a little more. Anything transmitting is ten times its
/// channel's noise or the front end behind this could not read it anyway.
const WAKE_RATIO: f32 = 4.0;

/// Blocks a channel keeps being read after its level drops, so a burst that
/// straddles a block boundary is read whole and a reply that follows one is
/// not missed while the gate closes.
const HANGOVER: u8 = 8;

/// Fewest windows measured in a block, whatever the burst spacing says.
///
/// The median needs something to be a median of. A DroneID burst is 720 us
/// against a block of 2.1 ms, so spacing the windows by the burst alone gives
/// three of them, a burst lands in two, and the median is then the burst: the
/// gate would close on exactly the block it exists to open.
const MIN_WINDOWS: usize = 8;

/// The coarse spectrum of one block, shared by every channel gated on it.
pub struct SpanGate {
    bins: usize,
    fft: Arc<dyn Fft<f32>>,
    /// A window, because the leakage decides whether this gate is selective
    /// at all: unwindowed, a strong keyed transmitter anywhere in the span
    /// stands over the floor of every channel of it through the sidelobes
    /// alone, and every gate is open whatever is where.
    window: Vec<f32>,
    rate: f64,
    /// Samples between the windows measured, from the shortest burst that
    /// must not be missed.
    step: usize,
    /// Power per bin, window-major: `windows * BINS`.
    power: Vec<f32>,
    windows: usize,
    scratch: Vec<C32>,
}

impl SpanGate {
    /// A gate over a span at `rate`, sampling often enough that a burst of
    /// `shortest_s` lands inside at least one window.
    pub fn new(rate: f64, shortest_s: f64) -> Self {
        let bins = ((rate / RESOLUTION_HZ) as usize).next_power_of_two().clamp(MIN_BINS, MAX_BINS);
        Self {
            bins,
            fft: FftPlanner::new().plan_fft_forward(bins),
            window: crate::window::blackman_harris(bins),
            rate,
            // A window has to fall inside the burst whole, so the spacing is
            // the burst less the window: a step of exactly the burst length
            // lets one slip between two windows and be missed.
            step: ((shortest_s * rate) as usize).saturating_sub(bins).max(bins),
            power: Vec::new(),
            windows: 0,
            scratch: Vec::new(),
        }
    }

    /// The bin an offset from the span's centre lands in.
    fn bin(&self, offset_hz: f64) -> usize {
        let k = (offset_hz / self.rate * self.bins as f64).round() as i64;
        k.rem_euclid(self.bins as i64) as usize
    }

    /// The bins a channel `half_width_hz` either side of `offset_hz`
    /// occupies, low first, which may wrap.
    pub fn band(&self, offset_hz: f64, half_width_hz: f64) -> (usize, usize) {
        (self.bin(offset_hz - half_width_hz), self.bin(offset_hz + half_width_hz))
    }

    /// Measure a block. Every gate reading it sees this block until the next.
    pub fn measure(&mut self, iq: &[C32]) {
        self.windows = 0;
        self.power.clear();
        let step = self.step.min(iq.len() / MIN_WINDOWS).max(self.bins);
        let mut at = 0usize;
        while at + self.bins <= iq.len() {
            self.scratch.clear();
            self.scratch
                .extend(iq[at..at + self.bins].iter().zip(&self.window).map(|(x, w)| *x * *w));
            self.fft.process(&mut self.scratch);
            self.power.extend(self.scratch.iter().map(|x| x.norm_sqr() / self.bins as f32));
            self.windows += 1;
            at += step;
        }
    }

    /// Each window's mean power over a band of bins, into `out`.
    fn band_windows(&self, (lo, hi): (usize, usize), out: &mut Vec<f32>) {
        out.clear();
        for w in 0..self.windows {
            let bins = &self.power[w * self.bins..(w + 1) * self.bins];
            let (mut sum, mut n) = (0.0f32, 0usize);
            let mut k = lo;
            loop {
                sum += bins[k];
                n += 1;
                if k == hi {
                    break;
                }
                k = (k + 1) % self.bins;
            }
            out.push(sum / n.max(1) as f32);
        }
    }
}

/// One channel's share of a [`SpanGate`], and whether it is lit.
pub struct ChannelGate {
    band: (usize, usize),
    active: bool,
    hangover: u8,
    /// This block's window powers, sorted, so the median costs no allocation.
    windows: Vec<f32>,
}

impl ChannelGate {
    pub fn new(span: &SpanGate, offset_hz: f64, half_width_hz: f64) -> Self {
        Self {
            band: span.band(offset_hz, half_width_hz),
            active: true,
            hangover: HANGOVER,
            windows: Vec::new(),
        }
    }

    /// Whether this channel is worth reading the measured block, and account
    /// for it either way.
    pub fn awake(&mut self, span: &SpanGate) -> bool {
        if span.windows == 0 {
            return self.active;
        }
        span.band_windows(self.band, &mut self.windows);
        let loudest = self.windows.iter().copied().fold(0.0f32, f32::max);
        self.windows.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        let median = self.windows[self.windows.len() / 2].max(1e-20);
        if loudest > median * WAKE_RATIO {
            self.hangover = HANGOVER;
        } else {
            self.hangover = self.hangover.saturating_sub(1);
        }
        self.active = self.hangover > 0;
        self.active
    }

    /// Whether it was lit for the last block measured.
    pub fn lit(&self) -> bool {
        self.active
    }

    pub fn reset(&mut self) {
        self.active = true;
        self.hangover = HANGOVER;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A block of noise with a tone `hz` from the centre for `len` samples.
    fn block(rate: f64, n: usize, hz: f64, from: usize, len: usize, amp: f32) -> Vec<C32> {
        let mut seed = 0x1234_5678u32;
        let mut iq: Vec<C32> = (0..n)
            .map(|_| {
                seed = seed.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
                let a = (seed >> 16) as f32 / 65_536.0 - 0.5;
                seed = seed.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
                let b = (seed >> 16) as f32 / 65_536.0 - 0.5;
                C32::new(a, b) * 0.001
            })
            .collect();
        for i in 0..len {
            let ph = std::f64::consts::TAU * hz * (from + i) as f64 / rate;
            iq[from + i] += C32::new(ph.cos() as f32, ph.sin() as f32) * amp;
        }
        iq
    }

    /// A burst a fiftieth of a block long lights its channel and nothing
    /// else, which is the whole point: averaged over the block it would be
    /// 17 dB down and the channel would sleep through it.
    #[test]
    fn a_short_burst_lights_its_own_channel_and_no_other() {
        let rate = 61_440_000.0;
        let mut span = SpanGate::new(rate, 100e-6);
        let mut on = ChannelGate::new(&span, 5e6, 700e3);
        let mut off = ChannelGate::new(&span, -5e6, 700e3);
        // Long enough that the hangover from the first block has run out.
        for _ in 0..12 {
            span.measure(&block(rate, 131_072, 0.0, 0, 0, 0.0));
            on.awake(&span);
            off.awake(&span);
        }
        assert!(!on.awake(&span), "a quiet channel is not read");

        // A hundred microseconds, which is what the gate was built for: a
        // fiftieth of the block, and 17 dB down on it if it were averaged.
        span.measure(&block(rate, 131_072, 5e6, 40_000, 6_144, 0.2));
        assert!(on.awake(&span), "the channel the burst was on slept through it");
        assert!(!off.awake(&span), "a channel with nothing on it woke");
    }

    /// And a channel with something on it in every block still has a floor
    /// of noise: the quietest window of a block is the channel between
    /// bursts, and the loudest is the burst.
    #[test]
    fn a_channel_busy_in_every_block_keeps_a_floor_of_noise() {
        let rate = 20_000_000.0;
        let mut span = SpanGate::new(rate, 100e-6);
        let mut gate = ChannelGate::new(&span, 3e6, 700e3);
        for _ in 0..40 {
            span.measure(&block(rate, 131_072, 3e6, 10_000, 2_000, 0.2));
            assert!(gate.awake(&span), "the gate closed on a channel that is still busy");
        }
    }
}
