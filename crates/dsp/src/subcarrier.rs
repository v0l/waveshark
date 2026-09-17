//! An audio subcarrier, read in amplitude or in frequency.
//!
//! Some transmissions put their information on a tone inside the audio rather
//! than on the radio carrier: a weather satellite sends its picture as the
//! amplitude of a 2400 Hz subcarrier inside an FM downlink, and a
//! shortwave fax sends it as the frequency of a tone around 1900 Hz inside a
//! sideband channel. Both are the same first two steps: mix the real audio
//! down by the subcarrier frequency into a complex baseband and filter away
//! the image at twice the carrier. What is left is the modulation, taken as
//! the magnitude by one and as the rate of change of phase by the other.
//!
//! Neither knows what it is carrying, so anything with an amplitude- or
//! frequency-modulated audio tone can read it: APT and weather fax today.

use crate::fir::{self, Fir};
use common::C32;
use std::f64::consts::TAU;

/// Mix a real signal down by `carrier_hz` and filter to `bandwidth_hz`.
///
/// Held apart from the two readers because both do it, and both are wrong if
/// they do it differently: the filter has to pass the modulation and stop the
/// image at twice the carrier, which for a 2400 Hz subcarrier is 4800 Hz away
/// and not far at all.
struct Baseband {
    rate: f64,
    carrier_hz: f64,
    phase: f64,
    fir: Fir,
    mixed: Vec<C32>,
}

impl Baseband {
    fn new(rate: f64, carrier_hz: f64, bandwidth_hz: f64) -> Self {
        // The image sits at twice the carrier, so the transition band has
        // that to fall in; 60 dB is what the rest of the receiver's filters
        // are designed to.
        let cutoff = (bandwidth_hz / 2.0 / rate).min(0.45);
        let taps = fir::estimate_taps((carrier_hz / rate).max(0.01), 60.0).max(31) | 1;
        Self {
            rate,
            carrier_hz,
            phase: 0.0,
            fir: Fir::new(fir::lowpass(taps, cutoff, 60.0)),
            mixed: Vec::new(),
        }
    }

    fn reset(&mut self) {
        self.phase = 0.0;
        self.fir.reset();
    }

    /// The complex baseband of `input`, appended to `out`.
    fn process(&mut self, input: &[f32], out: &mut Vec<C32>) {
        self.mixed.clear();
        self.mixed.reserve(input.len());
        let step = TAU * self.carrier_hz / self.rate;
        for x in input {
            let (s, c) = self.phase.sin_cos();
            self.mixed.push(C32::new(*x * c as f32, -*x * s as f32));
            self.phase += step;
            if self.phase > TAU {
                self.phase -= TAU;
            }
        }
        self.fir.process(&self.mixed, out);
    }
}

/// How loud the subcarrier is, sample by sample.
///
/// What an amplitude-modulated subcarrier carries: the envelope is the
/// brightness of an APT pixel. Scaled by two so that a subcarrier of unit
/// peak amplitude reads one, which is what a caller calibrating against a
/// known level expects.
pub struct Envelope {
    band: Baseband,
    buf: Vec<C32>,
}

impl Envelope {
    pub fn new(rate: f64, carrier_hz: f64, bandwidth_hz: f64) -> Self {
        Self { band: Baseband::new(rate, carrier_hz, bandwidth_hz), buf: Vec::new() }
    }

    pub fn reset(&mut self) {
        self.band.reset();
        self.buf.clear();
    }

    /// How many samples of delay the filter adds, so a caller counting
    /// samples knows where its signal went.
    pub fn latency(&self) -> usize {
        self.band.fir.len() / 2
    }

    pub fn process(&mut self, input: &[f32], out: &mut Vec<f32>) {
        self.buf.clear();
        self.band.process(input, &mut self.buf);
        out.extend(self.buf.iter().map(|z| 2.0 * z.norm()));
    }
}

/// What frequency the subcarrier is at, sample by sample, in hertz.
///
/// What a frequency-modulated subcarrier carries: the shade of a weather fax
/// pixel. The reading is absolute rather than a deviation, so a caller maps
/// hertz to its own scale and a mistuned signal shows up as an offset rather
/// than as a picture.
///
/// A discriminator over a phase difference, which needs one sample of
/// history: the first sample of the first block reads the carrier itself
/// rather than a wrong number.
pub struct Frequency {
    band: Baseband,
    carrier_hz: f64,
    rate: f64,
    last: Option<C32>,
    buf: Vec<C32>,
}

impl Frequency {
    pub fn new(rate: f64, carrier_hz: f64, bandwidth_hz: f64) -> Self {
        Self {
            band: Baseband::new(rate, carrier_hz, bandwidth_hz),
            carrier_hz,
            rate,
            last: None,
            buf: Vec::new(),
        }
    }

    pub fn reset(&mut self) {
        self.band.reset();
        self.last = None;
        self.buf.clear();
    }

    pub fn latency(&self) -> usize {
        self.band.fir.len() / 2
    }

    pub fn process(&mut self, input: &[f32], out: &mut Vec<f64>) {
        self.buf.clear();
        self.band.process(input, &mut self.buf);
        let scale = self.rate / TAU;
        for z in &self.buf {
            let prev = self.last.unwrap_or(*z);
            let d = z * prev.conj();
            // A sample with no amplitude has no phase either, and the
            // argument of a zero is zero, which would read as the carrier.
            // Saying so is the caller's job: it has the envelope too.
            let hz = self.carrier_hz + (d.im as f64).atan2(d.re as f64) * scale;
            out.push(hz);
            self.last = Some(*z);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn tone(rate: f64, hz: f64, amp: f64, n: usize) -> Vec<f32> {
        (0..n).map(|i| (amp * (TAU * hz * i as f64 / rate).sin()) as f32).collect()
    }

    /// A subcarrier at a steady amplitude reads that amplitude back, and the
    /// settled part of the reading is within 2% of it: the filter's ripple
    /// and nothing else.
    #[test]
    fn a_steady_subcarrier_reads_its_own_amplitude() {
        let rate = 20_800.0;
        let mut env = Envelope::new(rate, 2400.0, 4_160.0);
        let mut out = Vec::new();
        env.process(&tone(rate, 2400.0, 0.4, 8_000), &mut out);
        let settled = &out[env.latency() * 2..];
        let mean = settled.iter().sum::<f32>() / settled.len() as f32;
        assert!((mean - 0.4).abs() < 0.008, "read {mean} for an amplitude of 0.4");
    }

    /// Amplitude modulation at the line rate comes back as the modulation,
    /// not as the carrier: a square wave between two levels reads as two
    /// levels.
    #[test]
    fn a_keyed_subcarrier_reads_both_its_levels() {
        let rate = 20_800.0;
        let mut audio = Vec::new();
        for block in 0..8 {
            let amp = if block % 2 == 0 { 0.2 } else { 0.8 };
            audio.extend(tone(rate, 2400.0, amp, 2_000));
        }
        // Phase continuity across the joins is not what is being tested, and
        // each block starts at zero phase, which is where the tone is.
        let mut env = Envelope::new(rate, 2400.0, 4_160.0);
        let mut out = Vec::new();
        env.process(&audio, &mut out);
        // The middle of each block, well clear of the filter's edges.
        let at = |block: usize| {
            let mid = block * 2_000 + 1_000;
            out[mid - 100..mid + 100].iter().sum::<f32>() / 200.0
        };
        for block in 0..8 {
            let want = if block % 2 == 0 { 0.2 } else { 0.8 };
            assert!((at(block) - want).abs() < 0.01, "block {block} read {}", at(block));
        }
    }

    /// The two tones a weather fax sends read as 1500 and 2300 Hz to within
    /// a hertz, which is a shade of grey out of 255.
    #[test]
    fn a_shifted_tone_reads_the_frequency_it_was_sent_at() {
        let rate = 44_100.0;
        for hz in [1500.0, 1900.0, 2300.0] {
            let mut f = Frequency::new(rate, 1900.0, 1_600.0);
            let mut out = Vec::new();
            f.process(&tone(rate, hz, 0.5, 8_000), &mut out);
            let settled = &out[f.latency() * 2..];
            let mean = settled.iter().sum::<f64>() / settled.len() as f64;
            assert!((mean - hz).abs() < 1.0, "read {mean} Hz for a tone at {hz}");
        }
    }
}
