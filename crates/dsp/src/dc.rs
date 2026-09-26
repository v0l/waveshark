//! Removing the spur at the centre of a direct-conversion receiver.
//!
//! A zero-IF front end mixes the tuned frequency down to 0 Hz, so anything
//! that leaks from the local oscillator into the mixer input arrives at
//! exactly the same frequency as itself and lands at DC. Add the ADC's own
//! offset and the result is a permanent spike at the centre of the span that
//! moves with the tuning, because it *is* the tuning. It looks like a very
//! strong carrier and is not a signal at all.
//!
//! The HackRF is zero-IF, so it shows this plainly. The RTL2832U with an R820T
//! runs a low IF and shifts down digitally, which moves the spur but does not
//! remove it.
//!
//! The cure is a very narrow highpass at DC. Narrow matters: this notches out
//! real signals at the centre frequency too, so it must be far narrower than
//! anything being received. Tuning deliberately off-centre remains the better
//! answer where it is possible, and this is what makes the remainder tolerable.

use common::C32;

/// Default notch width. Wide enough to follow offset drift with temperature,
/// far narrower than the narrowest channel the app demodulates.
pub const DEFAULT_CUTOFF_HZ: f64 = 1_000.0;

/// Tracks the mean of a complex stream and subtracts it.
///
/// A first-order highpass rather than a subtracted block average: the offset
/// drifts, and a per-block mean would step at every block boundary and put a
/// click into the audio.
#[derive(Clone, Debug)]
pub struct DcBlock {
    mean: C32,
    alpha: f32,
}

impl DcBlock {
    pub fn new(rate: f64) -> Self {
        Self::with_cutoff(rate, DEFAULT_CUTOFF_HZ)
    }

    pub fn with_cutoff(rate: f64, cutoff_hz: f64) -> Self {
        // Single-pole coefficient for the requested corner. Clamped below so a
        // very high sample rate cannot make it denormal, and above so a silly
        // cutoff cannot turn this into a differentiator.
        let a = (std::f64::consts::TAU * cutoff_hz / rate.max(1.0)).clamp(1e-9, 0.5);
        Self { mean: C32::new(0.0, 0.0), alpha: a as f32 }
    }

    /// Current estimate of the offset, which is also a useful health readout:
    /// a large value means the front end is not well balanced.
    pub fn offset(&self) -> C32 {
        self.mean
    }

    pub fn reset(&mut self) {
        self.mean = C32::new(0.0, 0.0);
    }

    /// Remove the offset in place.
    pub fn process(&mut self, buf: &mut [C32]) {
        let a = self.alpha;
        let mut m = self.mean;
        for s in buf.iter_mut() {
            m += (*s - m) * a;
            *s -= m;
        }
        self.mean = m;
    }

    /// Prime the estimate from a block without altering it.
    ///
    /// Without this the first block is emitted with the full offset still in
    /// it, which the spectrum shows as a spike that fades over a second.
    pub fn prime(&mut self, buf: &[C32]) {
        if buf.is_empty() {
            return;
        }
        let mut sum = C32::new(0.0, 0.0);
        for s in buf {
            sum += *s;
        }
        self.mean = sum / buf.len() as f32;
    }
}

#[derive(Clone, Debug)]
pub struct SpurCancel {
    phase: f64,
    turn: f64,
    amplitude: C32,
    alpha: f32,
}

const SPUR_LANES: usize = 8;
const SPUR_ANCHOR: usize = 1024;

impl SpurCancel {
    pub fn new(offset_hz: f64, rate: f64, cutoff_hz: f64) -> Self {
        let turn = std::f64::consts::TAU * offset_hz / rate.max(1.0);
        let a = (std::f64::consts::TAU * cutoff_hz / rate.max(1.0)).clamp(1e-9, 0.5);
        Self { phase: 0.0, turn, amplitude: C32::new(0.0, 0.0), alpha: a as f32 }
    }

    pub fn amplitude(&self) -> C32 {
        self.amplitude
    }

    pub fn process(&mut self, buf: &mut [C32]) {
        use wide::f32x8;
        let alpha = self.alpha;
        let beta = 1.0 - alpha;
        let mut power = 1.0f32;
        let carry = f32x8::new(std::array::from_fn(|_| {
            power *= beta;
            power
        }));
        let (b1, b2, b4) =
            (f32x8::splat(beta), f32x8::splat(beta.powi(2)), f32x8::splat(beta.powi(4)));
        let (sin, cos) = (self.turn * SPUR_LANES as f64).sin_cos();
        let (step_re, step_im) = (f32x8::splat(cos as f32), f32x8::splat(sin as f32));
        let mut a = self.amplitude;
        for chunk in buf.chunks_mut(SPUR_ANCHOR) {
            let tones: [(f64, f64); SPUR_LANES] =
                std::array::from_fn(|k| (self.phase + self.turn * k as f64).sin_cos());
            let mut pr = f32x8::new(tones.map(|(_, cos)| cos as f32));
            let mut pi = f32x8::new(tones.map(|(sin, _)| sin as f32));
            let mut blocks = chunk.chunks_exact_mut(SPUR_LANES);
            for block in &mut blocks {
                let xr = f32x8::new(std::array::from_fn(|k| block[k].re));
                let xi = f32x8::new(std::array::from_fn(|k| block[k].im));
                let mut sr = (xr * pr + xi * pi) * alpha;
                let mut si = (xi * pr - xr * pi) * alpha;
                sr += shifted::<1>(sr) * b1;
                si += shifted::<1>(si) * b1;
                sr += shifted::<2>(sr) * b2;
                si += shifted::<2>(si) * b2;
                sr += shifted::<4>(sr) * b4;
                si += shifted::<4>(si) * b4;
                let ar = carry * a.re + sr;
                let ai = carry * a.im + si;
                let yr = (xr - (ar * pr - ai * pi)).to_array();
                let yi = (xi - (ar * pi + ai * pr)).to_array();
                for (k, s) in block.iter_mut().enumerate() {
                    *s = C32::new(yr[k], yi[k]);
                }
                a = C32::new(ar.to_array()[SPUR_LANES - 1], ai.to_array()[SPUR_LANES - 1]);
                (pr, pi) = (pr * step_re - pi * step_im, pr * step_im + pi * step_re);
            }
            let (pr, pi) = (pr.to_array(), pi.to_array());
            for (k, s) in blocks.into_remainder().iter_mut().enumerate() {
                let tone = C32::new(pr[k], pi[k]);
                a = a * beta + *s * tone.conj() * alpha;
                *s -= a * tone;
            }
            self.phase =
                (self.phase + self.turn * chunk.len() as f64).rem_euclid(std::f64::consts::TAU);
        }
        self.amplitude = a;
    }
}

fn shifted<const N: usize>(v: wide::f32x8) -> wide::f32x8 {
    let a = v.to_array();
    wide::f32x8::new(std::array::from_fn(|k| if k >= N { a[k - N] } else { 0.0 }))
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::TAU;

    #[test]
    fn a_spur_off_centre_is_cancelled_forty_db_and_a_tone_beside_it_kept() {
        let rate = 20e6;
        let spur_hz = -2_000_000.0;
        let keep_hz = -1_950_000.0;
        let tone = |hz: f64, n: usize, a: f32| {
            let p = TAU * hz * n as f64 / rate;
            C32::new(p.cos() as f32, p.sin() as f32) * a
        };
        let mut buf: Vec<C32> =
            (0..200_000).map(|n| tone(spur_hz, n, 1.0) + tone(keep_hz, n, 0.1)).collect();
        let mut cancel = SpurCancel::new(spur_hz, rate, 3_000.0);
        cancel.process(&mut buf);
        let tail = &buf[100_000..];
        let at = |hz: f64| {
            let s: C32 =
                tail.iter().enumerate().map(|(n, x)| x * tone(hz, n + 100_000, 1.0).conj()).sum();
            s.norm() / tail.len() as f32
        };
        assert!(at(spur_hz) < 0.01, "spur left at {}", at(spur_hz));
        assert!((at(keep_hz) - 0.1).abs() < 0.005, "tone beside it at {}", at(keep_hz));
    }

    const RATE: f64 = 2_400_000.0;

    fn rms(v: &[C32]) -> f64 {
        (v.iter().map(|c| c.norm_sqr() as f64).sum::<f64>() / v.len() as f64).sqrt()
    }

    #[test]
    fn a_constant_offset_is_removed() {
        let mut buf = vec![C32::new(0.3, -0.2); 200_000];
        let mut d = DcBlock::new(RATE);
        d.prime(&buf);
        d.process(&mut buf);
        assert!(rms(&buf[1000..]) < 1e-4, "offset survived: {}", rms(&buf[1000..]));
    }

    #[test]
    fn priming_removes_the_spur_from_the_very_first_block() {
        // Without priming the estimate starts at zero and the first samples
        // carry the whole offset, which shows as a spike that fades away.
        let make = || vec![C32::new(0.3, -0.2); 4096];
        let cold = {
            let mut b = make();
            DcBlock::new(RATE).process(&mut b);
            rms(&b)
        };
        let primed = {
            let mut b = make();
            let mut d = DcBlock::new(RATE);
            d.prime(&b);
            d.process(&mut b);
            rms(&b)
        };
        assert!(primed < cold / 100.0, "priming barely helped: {primed} vs {cold}");
    }

    #[test]
    fn a_signal_away_from_dc_is_left_alone() {
        // The notch must be narrow enough that a channel a few kHz off centre
        // passes untouched, or removing the spur costs more than it saves.
        let n = 200_000;
        let mut buf: Vec<C32> = (0..n)
            .map(|i| {
                let t = i as f64 / RATE;
                let p = TAU * 50_000.0 * t;
                C32::new(p.cos() as f32, p.sin() as f32) + C32::new(0.3, -0.2)
            })
            .collect();
        let mut d = DcBlock::new(RATE);
        d.prime(&buf);
        d.process(&mut buf);
        let level = rms(&buf[1000..]);
        assert!((level - 1.0).abs() < 0.01, "signal level changed to {level}");
    }

    #[test]
    fn the_notch_is_narrower_than_the_narrowest_channel() {
        // 1 kHz against 12.5 kHz narrowband voice, and against the 2.4 kHz
        // either side of the RDS subcarrier.
        assert!(DEFAULT_CUTOFF_HZ < 12_500.0 / 4.0);
    }

    #[test]
    fn the_offset_estimate_reports_what_was_removed() {
        let mut buf = vec![C32::new(0.25, 0.1); 100_000];
        let mut d = DcBlock::new(RATE);
        d.prime(&buf);
        d.process(&mut buf);
        let o = d.offset();
        assert!((o.re - 0.25).abs() < 1e-3 && (o.im - 0.1).abs() < 1e-3, "reported {o}");
    }

    #[test]
    fn block_boundaries_do_not_click() {
        // A per-block mean steps at every boundary; a tracking filter must not.
        let n = 60_000;
        let src: Vec<C32> = (0..n)
            .map(|i| {
                let t = i as f64 / RATE;
                let p = TAU * 30_000.0 * t;
                C32::new(0.5 * p.cos() as f32, 0.5 * p.sin() as f32) + C32::new(0.3, -0.2)
            })
            .collect();
        let whole = {
            let mut b = src.clone();
            let mut d = DcBlock::new(RATE);
            d.prime(&b);
            d.process(&mut b);
            b
        };
        let split = {
            let mut d = DcBlock::new(RATE);
            d.prime(&src[..4096]);
            let mut out = Vec::new();
            for c in src.chunks(4096) {
                let mut b = c.to_vec();
                d.process(&mut b);
                out.extend(b);
            }
            out
        };
        for (i, (a, b)) in whole.iter().zip(&split).enumerate().skip(8192) {
            assert!((a - b).norm() < 1e-5, "sample {i} differs across blocking");
        }
    }

    #[test]
    fn a_drifting_offset_is_followed() {
        // Thermal drift moves the offset, so a fixed correction measured once
        // at startup would come apart.
        let n = 400_000;
        let mut buf: Vec<C32> = (0..n)
            .map(|i| {
                let d = 0.3 + 0.2 * (i as f32 / n as f32);
                C32::new(d, -0.2)
            })
            .collect();
        let mut d = DcBlock::new(RATE);
        d.prime(&buf);
        d.process(&mut buf);
        assert!(rms(&buf[10_000..]) < 1e-3, "drift not tracked: {}", rms(&buf[10_000..]));
    }
}
