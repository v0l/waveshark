//! Minimum shift keying: audio in, bits out.
//!
//! MSK is continuous phase keying whose two tones sit exactly half the bit
//! rate apart, so each bit turns the phase a quarter turn one way or the
//! other and nothing jumps. That is what separates it from the tone pair
//! `afsk` reads: the phase runs on across a symbol, so a correlator per tone
//! throws away the very thing that carries the bit.
//!
//! What reads it instead is a phase-locked oscillator at the midpoint of the
//! two tones, a matched filter one bit long, and a decision taken on the real
//! axis and the imaginary axis in turn, because a quarter turn a bit lands the
//! point on alternate axes. The axis it did not land on is the phase error,
//! and feeding that back is also what resolves the ambiguity: the oscillator
//! is free to settle a quarter turn out, so the loop chooses the alignment
//! rather than the caller guessing it.
//!
//! What it cannot resolve is the polarity. Nothing in the waveform says which
//! tone is a one, so a stream can arrive inverted and whatever reads the bits
//! has to be able to see that, usually in a sync word read as its complement.
//!
//! The loop constants and the matched filter come from `msk.c` in Thierry
//! Leconte's `acarsdec`, generalised here from that decoder's fixed 12.5 kHz
//! and 2400 baud to whatever a caller has. What is checked against that
//! decoder is the whole receiver, on its own recording, in
//! `crates/decode/tests/acars_capture.rs`.
//!
//! There is no modulator here yet. A first attempt keyed the two tones with a
//! continuous phase and was read by nothing, which is a modulator that is
//! wrong rather than a transmitter, so it is left out until there is a
//! receiver to check it against.

use std::f64::consts::TAU;

/// A waveform: how fast, and where its two tones sit.
#[derive(Clone, Copy, Debug)]
pub struct MskConfig {
    pub baud: f64,
    /// Midpoint of the two tones, which are `baud / 4` either side of it.
    pub carrier_hz: f64,
}

impl MskConfig {
    /// ACARS: 2400 bits a second on 1200 and 2400 Hz tones, in the audio of an
    /// AM aircraft channel.
    pub const ACARS: Self = Self { baud: 2400.0, carrier_hz: 1800.0 };
}

/// Loop gain and pole of the oscillator's filter.
const PLL_GAIN: f64 = 38e-4;
const PLL_POLE: f64 = 0.52;
/// Steps the matched filter is interpolated to, so it can be evaluated at the
/// fractional sample the bit clock landed on.
const OVERSAMPLE: usize = 12;

pub struct MskDemod {
    rate: f64,
    cfg: MskConfig,
    /// A raised half cosine one bit long, at `OVERSAMPLE` times the sample
    /// rate so it can be read at a fractional offset.
    taps: Vec<f32>,
    /// Samples the filter spans, one bit's worth plus one.
    span: usize,
    /// Mixed input, as a ring of `span` samples.
    ring: Vec<(f32, f32)>,
    at: usize,
    phase: f64,
    offset: f64,
    /// The bit clock, which rides on the oscillator's own phase rather than
    /// counting samples: at the midpoint frequency a bit is a fixed fraction
    /// of a turn, and following the carrier follows the transmitter's clock
    /// with it.
    clock: f64,
    /// Which axis the next decision is taken on.
    step: u32,
}

impl MskDemod {
    pub fn new(rate: f64, cfg: MskConfig) -> Self {
        let span = (rate / (cfg.baud / 2.0)) as usize + 1;
        let n = span * OVERSAMPLE + 1;
        let cut = cfg.baud / 4.0;
        let taps = (0..n)
            .map(|i| {
                let x = TAU * cut / rate / OVERSAMPLE as f64 * (i as f64 - (n - 1) as f64 / 2.0);
                (x.cos() as f32).max(0.0)
            })
            .collect();
        Self {
            rate,
            cfg,
            taps,
            span,
            ring: vec![(0.0, 0.0); span],
            at: 0,
            phase: 0.0,
            offset: 0.0,
            clock: 0.0,
            step: 0,
        }
    }

    pub fn reset(&mut self) {
        self.ring.iter_mut().for_each(|s| *s = (0.0, 0.0));
        self.at = 0;
        self.phase = 0.0;
        self.offset = 0.0;
        self.clock = 0.0;
        self.step = 0;
    }

    /// Turns of the oscillator in one bit, which is what the clock counts off.
    fn turns_per_bit(&self) -> f64 {
        TAU * self.cfg.carrier_hz / self.cfg.baud
    }

    /// Demodulate audio, appending one bit per symbol.
    pub fn process(&mut self, audio: &[f32], bits: &mut Vec<bool>) {
        let per_bit = self.turns_per_bit();
        for &sample in audio {
            let advance = TAU * self.cfg.carrier_hz / self.rate + self.offset;
            self.phase += advance;
            if self.phase >= TAU {
                self.phase -= TAU;
            }
            let (sin, cos) = (-self.phase).sin_cos();
            self.ring[self.at] = (sample * cos as f32, sample * sin as f32);
            self.at = (self.at + 1) % self.span;

            self.clock += advance;
            if self.clock < per_bit - advance / 2.0 {
                continue;
            }
            self.clock -= per_bit;

            let o = ((OVERSAMPLE as f64 * (self.clock / advance + 0.5)) as usize).min(OVERSAMPLE);
            let (mut re, mut im) = (0.0f32, 0.0f32);
            for j in 0..self.span {
                let h = self.taps[o + j * OVERSAMPLE];
                let (i, q) = self.ring[(j + self.at) % self.span];
                re += h * i;
                im += h * q;
            }
            let level = (re * re + im * im).sqrt() + 1e-8;
            let (re, im) = (re / level, im / level);

            let (value, error) = if self.step & 1 != 0 {
                (im, if im >= 0.0 { -re } else { re })
            } else {
                (re, if re >= 0.0 { im } else { -im })
            };
            // The axis rotates a quarter turn a bit, so it points the other
            // way every second symbol and the decision's sign goes with it:
            // the pattern is plus, plus, minus, minus. Reading the axis and
            // not its direction gives a stream that is right a quarter of the
            // time, which looks like a demodulator that nearly works.
            let value = if self.step & 2 != 0 { -value } else { value };
            self.step += 1;
            self.offset = PLL_POLE * self.offset + (1.0 - PLL_POLE) * PLL_GAIN * error as f64;
            bits.push(value > 0.0);
        }
    }
}

/// Bits of a byte stream, least significant bit first, which is the order
/// every MSK protocol here sends them in.
pub fn bits_of(bytes: &[u8]) -> Vec<bool> {
    let mut bits = Vec::with_capacity(bytes.len() * 8);
    for b in bytes {
        for i in 0..8 {
            bits.push(b >> i & 1 != 0);
        }
    }
    bits
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    /// Bits come off the wire least significant first, which is the order
    /// every byte-oriented MSK protocol here sends them in.
    #[test]
    fn bytes_become_bits_least_significant_first() {
        assert_eq!(bits_of(&[0x16]), [false, true, true, false, true, false, false, false]);
    }

    /// The filter spans two bit periods, which is what an MSK symbol occupies
    /// on each of its two axes, and the clock counts three quarter turns of
    /// the midpoint carrier, which is one bit.
    #[test]
    fn the_receiver_is_built_for_the_waveform_it_was_given() {
        let d = MskDemod::new(12_500.0, MskConfig::ACARS);
        assert_eq!(d.span, 11);
        assert!((d.turns_per_bit() - 3.0 * PI / 2.0).abs() < 1e-9);
    }
}
