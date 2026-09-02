//! Complex frequency translation.

use common::C32;

/// Numerically controlled oscillator that shifts a signal in frequency.
///
/// Phase is accumulated in f64 and wrapped every sample. Accumulating in f32,
/// or letting the phase grow without wrapping, loses mantissa bits as the
/// count rises and injects phase noise: at 2.4 MS/s an f32 accumulator is
/// visibly degraded within a second. This is the same trap that made a
/// channelizer test appear to fail earlier.
///
/// The per-sample work is one complex multiply by a rotating phasor, not a
/// sine and cosine: at 4 MS/s a source's mixer was costing more than the
/// rest of its extraction together, and eight open sources brought the
/// receiver to real time. The phasor is stepped in single precision and
/// re-anchored from the double-precision phase every [`ANCHOR`] samples, so
/// the error it accumulates between anchors stays around a millionth and
/// never grows.
#[derive(Clone, Debug)]
pub struct Mixer {
    phase: f64,
    /// Radians per sample.
    step: f64,
    /// The phasor at `phase`, and one step's rotation.
    phasor: C32,
    rot: C32,
    /// Samples since the phasor was last set from `phase`.
    since: u32,
}

/// Samples between re-anchoring the phasor to the exact phase.
const ANCHOR: u32 = 1024;

impl Mixer {
    /// Shift by `shift_hz` at the given sample rate. A negative shift moves a
    /// signal at `+shift_hz` down to DC.
    pub fn new(shift_hz: f64, rate: f64) -> Self {
        let mut m = Self { phase: 0.0, step: 0.0, phasor: C32::new(1.0, 0.0), rot: C32::new(1.0, 0.0), since: 0 };
        m.set_shift(shift_hz, rate);
        m
    }

    pub fn set_shift(&mut self, shift_hz: f64, rate: f64) {
        self.step = std::f64::consts::TAU * shift_hz / rate;
        let (s, c) = self.step.sin_cos();
        self.rot = C32::new(c as f32, s as f32);
        self.anchor();
    }

    pub fn reset(&mut self) {
        self.phase = 0.0;
        self.anchor();
    }

    /// Set the phasor from the exact phase.
    fn anchor(&mut self) {
        let (s, c) = self.phase.sin_cos();
        self.phasor = C32::new(c as f32, s as f32);
        self.since = 0;
    }

    /// The phasor for the current sample, then advance.
    #[inline]
    fn next(&mut self) -> C32 {
        if self.since >= ANCHOR {
            self.anchor();
        }
        let p = self.phasor;
        self.phasor *= self.rot;
        self.since += 1;
        self.phase += self.step;
        // Wrap so the anchor's argument stays small and precision does not
        // decay over a long capture.
        if self.phase >= std::f64::consts::TAU {
            self.phase -= std::f64::consts::TAU;
        } else if self.phase < 0.0 {
            self.phase += std::f64::consts::TAU;
        }
        p
    }

    /// Shift `input` into `out`, appending.
    pub fn process(&mut self, input: &[C32], out: &mut Vec<C32>) {
        out.reserve(input.len());
        for &x in input {
            let p = self.next();
            out.push(x * p);
        }
    }

    /// In-place variant.
    pub fn process_in_place(&mut self, buf: &mut [C32]) {
        for x in buf.iter_mut() {
            let p = self.next();
            *x *= p;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn tone(n: usize, cps: f64) -> Vec<C32> {
        (0..n)
            .map(|i| {
                let p = ((cps * i as f64).rem_euclid(1.0) * std::f64::consts::TAU) as f32;
                C32::new(p.cos(), p.sin())
            })
            .collect()
    }

    /// Estimate frequency in cycles/sample from the mean phase advance.
    fn est_freq(v: &[C32]) -> f64 {
        let mut acc = C32::new(0.0, 0.0);
        for w in v.windows(2) {
            acc += w[1] * w[0].conj();
        }
        acc.arg() as f64 / std::f64::consts::TAU
    }

    #[test]
    fn shifts_a_tone_to_dc() {
        let rate = 2.4e6;
        let sig = tone(100_000, 0.1);
        let mut m = Mixer::new(-0.1 * rate, rate);
        let mut out = Vec::new();
        m.process(&sig, &mut out);
        assert!(est_freq(&out).abs() < 1e-9, "residual {}", est_freq(&out));
    }

    #[test]
    fn precision_holds_over_a_long_capture() {
        // Two million samples is under a second at 2.4 MS/s. A naive f32
        // phase accumulator has visibly degraded by this point.
        let rate = 2.4e6;
        let sig = tone(2_000_000, 0.25);
        let mut m = Mixer::new(-0.25 * rate, rate);
        let mut out = Vec::new();
        m.process(&sig, &mut out);

        let tail = &out[out.len() - 10_000..];
        let err = tail.iter().map(|c| (c.im).abs()).fold(0.0f32, f32::max);
        assert!(err < 1e-3, "phase drifted, worst imaginary part {err}");
    }

    #[test]
    fn shift_is_reversible() {
        let rate = 1e6;
        let sig = tone(10_000, 0.05);
        let mut up = Mixer::new(123_456.0, rate);
        let mut down = Mixer::new(-123_456.0, rate);
        let mut a = Vec::new();
        let mut b = Vec::new();
        up.process(&sig, &mut a);
        down.process(&a, &mut b);
        for (x, y) in sig.iter().zip(&b) {
            assert!((x - y).norm() < 1e-4, "{x} vs {y}");
        }
    }
}
