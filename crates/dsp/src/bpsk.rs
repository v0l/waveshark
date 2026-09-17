//! Binary phase shift keying: complex baseband in, soft symbols out.
//!
//! One bit a symbol, carried in a half turn of phase, so there is no
//! frequency to track the way `fsk` does and no tone pair the way `afsk`
//! has: what a receiver needs is a phase reference it builds itself and a
//! clock it recovers from the transitions. That is a Costas loop, which is
//! blind to the half turn the bit itself makes, and a Mueller and Müller
//! timing loop, which needs no sample between symbols.
//!
//! Parameterised by rate and baud, because the next BPSK link wants the same
//! code with a different number in it. Inmarsat's STD-C TDM is 1200 symbols
//! a second and is [`BpskConfig::INMARSAT_C`].
//!
//! The Costas loop cannot resolve the polarity, so a stream can arrive
//! inverted and whatever reads the symbols has to see that for itself, which
//! is what a unique word searched for both ways up does.
//!
//! A soft symbol is positive for a zero bit and negative for a one, which is
//! the convention [`crate::conv`] decodes in.

use common::C32;
use std::f32::consts::PI;

/// The shape of one BPSK link.
#[derive(Clone, Copy, Debug)]
pub struct BpskConfig {
    /// Symbols per second.
    pub baud: f64,
    /// How far the carrier recovery may pull, in hertz. A satellite's
    /// downlink is where the receiver put it, give or take the tuner.
    pub pull_hz: f64,
}

impl BpskConfig {
    /// Inmarsat-C: 1200 symbols a second on the L-band TDM, ±200 Hz of
    /// pull, which covers a consumer tuner's error at 1.5 GHz.
    pub const INMARSAT_C: Self = Self { baud: 1200.0, pull_hz: 1500.0 };
}

/// Loop gain of the carrier recovery, as a fraction of the residual phase
/// error taken out per symbol. The coarse estimate below leaves only a few
/// hertz, so this is a phase loop rather than an acquisition loop.
const CARRIER_GAIN: f32 = 0.1;
/// How much of the phase error goes to the frequency estimate rather than
/// the phase, per symbol.
const CARRIER_FREQ_GAIN: f32 = 0.002;
/// How fast the coarse estimate follows, as a fraction of its own value per
/// sample. A decision-directed loop alone pulls about 60 Hz at 1200 baud
/// and false-locks beyond that, which is nothing at L band: measured on
/// synthesised frames, the loop alone reads a 60 Hz offset and fails at 100,
/// 150, 200 and 300 Hz, where squaring first reads every one of them.
const COARSE_TRACK: f32 = 1e-5;
/// Loop gain of the timing recovery, in samples of clock phase per unit of
/// Gardner error.
///
/// Gardner rather than Mueller and Müller because the error has to survive
/// an unlocked carrier: taken on the complex sample the error does not know
/// what the phase is, where a decision-directed one does. Measured on
/// synthesised frames, a Mueller and Müller loop reads a 150 Hz offset and
/// throws away a fifth of the symbols at 300 Hz and beyond, because the
/// phase loop's own wobble arrives at it as timing error.
const TIMING_GAIN: f64 = 0.01;

pub struct BpskDemod {
    cfg: BpskConfig,
    /// Samples a symbol.
    sps: f64,
    /// Matched filter: a running sum one symbol long.
    window: Vec<C32>,
    at: usize,
    sum: C32,
    /// Carrier recovery: the coarse estimate from the squared signal, the
    /// loop's own correction on top of it, and the oscillator's phase.
    phase: f32,
    freq: f32,
    /// The squared sample before this one, and the smoothed step between
    /// them, whose angle is twice the carrier offset a sample.
    prev_sq: C32,
    coarse: C32,
    /// Timing recovery: samples until the next instant the clock wants,
    /// which alternates between the middle of a symbol and its end.
    countdown: f64,
    period: f64,
    last: C32,
    /// Whether the next instant is the middle of a symbol.
    midway: bool,
    mid: C32,
    prev_sym: C32,
}

impl BpskDemod {
    pub fn new(rate: f64, cfg: BpskConfig) -> Self {
        let sps = (rate / cfg.baud).max(2.0);
        let span = sps.round().max(1.0) as usize;
        Self {
            cfg,
            sps,
            window: vec![C32::new(0.0, 0.0); span],
            at: 0,
            sum: C32::new(0.0, 0.0),
            phase: 0.0,
            freq: 0.0,
            prev_sq: C32::new(0.0, 0.0),
            coarse: C32::new(0.0, 0.0),
            countdown: sps / 2.0,
            period: sps,
            last: C32::new(0.0, 0.0),
            midway: true,
            mid: C32::new(0.0, 0.0),
            prev_sym: C32::new(0.0, 0.0),
        }
    }

    pub fn reset(&mut self) {
        self.window.iter_mut().for_each(|s| *s = C32::new(0.0, 0.0));
        self.at = 0;
        self.sum = C32::new(0.0, 0.0);
        self.phase = 0.0;
        self.freq = 0.0;
        self.prev_sq = C32::new(0.0, 0.0);
        self.coarse = C32::new(0.0, 0.0);
        self.countdown = self.sps / 2.0;
        self.period = self.sps;
        self.last = C32::new(0.0, 0.0);
        self.midway = true;
        self.mid = C32::new(0.0, 0.0);
        self.prev_sym = C32::new(0.0, 0.0);
    }

    /// The carrier offset the loops have settled on, in hertz. What a front
    /// end reports as the transmitter's error against where it was tuned.
    pub fn offset_hz(&self, rate: f64) -> f64 {
        (self.coarse_freq() + self.freq) as f64 * rate / (2.0 * std::f64::consts::PI)
    }

    /// Radians a sample, from the angle the squared signal turns through:
    /// squaring takes the half turns of the modulation out, so what is left
    /// is twice the carrier offset and nothing else.
    fn coarse_freq(&self) -> f32 {
        if self.coarse.norm_sqr() <= 0.0 {
            return 0.0;
        }
        self.coarse.im.atan2(self.coarse.re) / 2.0
    }

    pub fn process(&mut self, iq: &[C32], out: &mut Vec<f32>) {
        // The oscillator runs per sample, so its pull is per sample too.
        let max_freq = 2.0 * PI * (self.cfg.pull_hz / (self.sps * self.cfg.baud)) as f32;
        for &x in iq {
            // Coarse: the angle between one squared sample and the last,
            // smoothed. Unit length, so a fade does not weigh more than a
            // strong sample.
            let sq = x * x;
            let n = sq.norm();
            if n > 0.0 {
                let unit = sq / n;
                let step = unit * self.prev_sq.conj();
                self.coarse += (step - self.coarse) * COARSE_TRACK;
                self.prev_sq = unit;
            }

            // Carrier: turn the sample back by the estimate, which advances
            // every sample and is corrected once a symbol.
            self.phase += self.coarse_freq().clamp(-max_freq, max_freq) + self.freq;
            if self.phase > PI {
                self.phase -= 2.0 * PI;
            } else if self.phase < -PI {
                self.phase += 2.0 * PI;
            }
            let (s, c) = self.phase.sin_cos();
            let y = C32::new(x.re * c + x.im * s, x.im * c - x.re * s);

            // Matched filter, a running sum one symbol long.
            self.sum += y - self.window[self.at];
            self.window[self.at] = y;
            self.at = (self.at + 1) % self.window.len();
            let z = self.sum / self.window.len() as f32;

            self.countdown -= 1.0;
            if self.countdown <= 0.0 {
                // Interpolate to the instant the clock asked for, which is
                // between this sample and the one before it.
                let frac = (-self.countdown) as f32;
                let sample = self.last * frac + z * (1.0 - frac);
                self.last = z;
                self.countdown += self.period / 2.0;

                if self.midway {
                    self.mid = sample;
                    self.midway = false;
                    continue;
                }
                self.midway = true;
                let sym = sample;

                // Gardner: the sample halfway between two symbols is zero
                // when the clock is right and leans towards whichever way
                // the symbol moved when it is not. On the complex sample,
                // so an unlocked carrier costs it nothing.
                let d = sym - self.prev_sym;
                let err = (d.re * self.mid.re + d.im * self.mid.im) as f64;
                let err = err.clamp(-1.0, 1.0);
                self.period = (self.period - TIMING_GAIN * err / 10.0)
                    .clamp(self.sps - self.sps / 50.0, self.sps + self.sps / 50.0);
                self.countdown -= TIMING_GAIN * err;
                self.prev_sym = sym;

                // Costas: with the decision removed, what is left on the
                // imaginary axis is the phase error.
                let value = sym.re;
                let dec = if value >= 0.0 { 1.0 } else { -1.0 };
                let err = (dec * sym.im / sym.re.abs().max(1e-6)).clamp(-1.0, 1.0);
                self.freq = (self.freq + CARRIER_FREQ_GAIN * err / self.sps as f32)
                    .clamp(-max_freq, max_freq);
                self.phase += CARRIER_GAIN * err;

                // A one is a half turn, so a positive sample is a zero,
                // which is the sign convention the Viterbi decoder reads.
                out.push(value);
            } else {
                self.last = z;
            }
        }
    }
}

/// Key `bits` as BPSK at `cfg.baud`, `offset_hz` off centre, one sample at a
/// time. The transmitter side of the loops above, which is what proves them:
/// nothing on the air is keyed from here.
pub fn modulate(
    bits: &[u8],
    rate: f64,
    cfg: BpskConfig,
    offset_hz: f64,
    amplitude: f32,
) -> Vec<C32> {
    let sps = rate / cfg.baud;
    let n = (bits.len() as f64 * sps) as usize;
    let mut out = Vec::with_capacity(n);
    for i in 0..n {
        let bit = bits[((i as f64 / sps) as usize).min(bits.len() - 1)];
        let sign = if bit == 0 { 1.0 } else { -1.0 };
        let phase = std::f64::consts::TAU * offset_hz * i as f64 / rate;
        out.push(C32::from_polar(amplitude * sign, phase as f32));
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    fn bits(n: usize, seed: u64) -> Vec<u8> {
        let mut s = seed;
        (0..n)
            .map(|_| {
                s = s.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
                (s >> 33 & 1) as u8
            })
            .collect()
    }

    /// Symbols read back against the bits that were keyed, at whichever lag
    /// fits: the matched filter and the clock cost a symbol or two at the
    /// start, and nothing here is being asked where a stream begins.
    fn wrong(sent: &[u8], soft: &[f32], tail: usize) -> usize {
        let read: Vec<u8> = soft.iter().map(|v| u8::from(*v < 0.0)).collect();
        let start = read.len() - tail;
        (-40i64..40)
            .map(|lag| {
                (0..tail)
                    .filter(|&i| {
                        let j = (start + i) as i64 + lag;
                        j < 0 || j as usize >= sent.len() || read[start + i] != sent[j as usize]
                    })
                    .count()
            })
            .min()
            .unwrap()
    }

    fn demodulate(iq: &[C32], rate: f64) -> Vec<f32> {
        let mut d = BpskDemod::new(rate, BpskConfig::INMARSAT_C);
        let mut soft = Vec::new();
        d.process(iq, &mut soft);
        soft
    }

    /// Every symbol of a keyed stream comes back, at the tuner error a
    /// consumer dongle has at 1.5 GHz. 12000 symbols in, the last 5000
    /// checked, and not one wrong at any offset inside the declared pull.
    #[test]
    fn a_keyed_stream_is_read_back_symbol_for_symbol() {
        let cfg = BpskConfig::INMARSAT_C;
        let rate = 9600.0;
        let sent = bits(12_000, 7);
        for offset in [0.0, 150.0, 600.0, 1500.0, -1000.0] {
            let iq = modulate(&sent, rate, cfg, offset, 1.0);
            let soft = demodulate(&iq, rate);
            assert!(soft.len() >= 11_990, "{offset} Hz: {} symbols of 12000", soft.len());
            assert_eq!(wrong(&sent, &soft, 5000), 0, "{offset} Hz");
        }
        // And past the pull it is nobody's stream: the loop has no business
        // claiming to read a carrier it was never pointed at.
        let iq = modulate(&sent, rate, cfg, 2400.0, 1.0);
        let soft = demodulate(&iq, rate);
        assert!(wrong(&sent, &soft, 5000) > 1000, "2400 Hz was read anyway");
    }

    /// A transmitter's clock is not the receiver's. 500 ppm out, which is
    /// ten times what a crystal drifts, and every symbol still lands.
    #[test]
    fn the_clock_is_followed_when_it_is_not_the_receivers() {
        let rate = 9600.0;
        let sent = bits(12_000, 11);
        let keyed = BpskConfig { baud: 1200.0 * 1.0005, pull_hz: 1500.0 };
        let iq = modulate(&sent, rate, keyed, 300.0, 1.0);
        let soft = demodulate(&iq, rate);
        assert_eq!(wrong(&sent, &soft, 5000), 0);
    }

    /// Noise keys nothing. A demodulator always produces symbols, so what is
    /// pinned here is that they stay finite and say nothing: the frame above
    /// is what refuses noise.
    #[test]
    fn noise_produces_finite_symbols() {
        let rate = 9600.0;
        let mut s = 99u64;
        let iq: Vec<C32> = (0..96_000)
            .map(|_| {
                let mut next = || {
                    s = s.wrapping_mul(6364136223846793005).wrapping_add(1);
                    (s >> 33) as f32 / (1u64 << 30) as f32 - 1.0
                };
                C32::new(next(), next())
            })
            .collect();
        let soft = demodulate(&iq, rate);
        assert!(soft.len() > 11_000, "{} symbols", soft.len());
        assert!(soft.iter().all(|v| v.is_finite()), "a symbol went to NaN");
    }
}
