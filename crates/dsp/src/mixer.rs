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
/// receiver to real time. Four phasors a step apart are advanced together by
/// four steps' rotation, so a register full of samples is multiplied through
/// at once, and the set is re-anchored from the double-precision phase every
/// [`ANCHOR`] samples so the error it accumulates between anchors stays
/// around a millionth and never grows. Measured at 40 MS/s this is 0.2 ms per
/// 10 ms block against 1.1 ms one sample at a time.
#[derive(Clone, Debug)]
pub struct Mixer {
    phase: f64,
    /// Radians per sample.
    step: f64,
}

/// Samples between re-anchoring the phasors to the exact phase.
const ANCHOR: usize = 1024;

/// Complex samples one register holds.
const WIDTH: usize = 4;

impl Mixer {
    /// Shift by `shift_hz` at the given sample rate. A negative shift moves a
    /// signal at `+shift_hz` down to DC.
    pub fn new(shift_hz: f64, rate: f64) -> Self {
        let mut m = Self { phase: 0.0, step: 0.0 };
        m.set_shift(shift_hz, rate);
        m
    }

    pub fn set_shift(&mut self, shift_hz: f64, rate: f64) {
        self.step = std::f64::consts::TAU * shift_hz / rate;
    }

    pub fn reset(&mut self) {
        self.phase = 0.0;
    }

    /// The four phasors for the samples from the current phase, and the
    /// rotation that carries them four samples on.
    fn anchor(&self) -> ([f32; WIDTH * 2], [f32; 2]) {
        let mut p = [0.0f32; WIDTH * 2];
        for k in 0..WIDTH {
            let (s, c) = (self.phase + self.step * k as f64).sin_cos();
            p[2 * k] = c as f32;
            p[2 * k + 1] = s as f32;
        }
        let (s, c) = (self.step * WIDTH as f64).sin_cos();
        (p, [c as f32, s as f32])
    }

    /// Advance the exact phase by `n` samples, wrapped so the anchor's
    /// argument stays small and precision does not decay over a long capture.
    fn advance(&mut self, n: usize) {
        self.phase = (self.phase + self.step * n as f64).rem_euclid(std::f64::consts::TAU);
    }

    /// Shift `input` into `out`, appending.
    pub fn process(&mut self, input: &[C32], out: &mut Vec<C32>) {
        let at = out.len();
        out.extend_from_slice(input);
        self.process_in_place(&mut out[at..]);
    }

    /// In-place variant.
    pub fn process_in_place(&mut self, buf: &mut [C32]) {
        let rot = Rotate::pick();
        for chunk in buf.chunks_mut(ANCHOR) {
            let (phasors, step4) = self.anchor();
            let n = chunk.len();
            let whole = n / WIDTH * WIDTH;
            // SAFETY: Complex<f32> is repr(C) { re, im } with no padding, so
            // the chunk is exactly 2n contiguous f32.
            let flat =
                unsafe { std::slice::from_raw_parts_mut(chunk.as_mut_ptr() as *mut f32, n * 2) };
            rot.run(&mut flat[..whole * 2], phasors, step4);
            // The samples a register did not fill, from where the phasors
            // would have got to.
            if whole < n {
                let mut m = Mixer { phase: self.phase, step: self.step };
                m.advance(whole);
                let (p, _) = m.anchor();
                for (k, x) in chunk[whole..].iter_mut().enumerate() {
                    *x *= C32::new(p[2 * k], p[2 * k + 1]);
                }
            }
            self.advance(n);
        }
    }
}

/// Multiply a run of samples by a run of phasors that rotate as they go.
///
/// Both lanes of a complex product need the other component of the sample,
/// so each register of samples is multiplied twice: once by the phasors'
/// real parts spread over both lanes, once with its lanes swapped by the
/// imaginary parts, and the two are combined with a subtract on the even
/// lanes and an add on the odd. Hand-written for AVX2 for the reason the
/// decimator's kernel is: a vector crate fixes its instruction set at
/// compile time, and the shipped binary is built for the baseline.
#[derive(Clone, Copy)]
enum Rotate {
    #[cfg(target_arch = "x86_64")]
    Avx2Fma,
    Portable,
}

impl Rotate {
    #[inline]
    fn pick() -> Self {
        #[cfg(target_arch = "x86_64")]
        if is_x86_feature_detected!("avx2") && is_x86_feature_detected!("fma") {
            return Rotate::Avx2Fma;
        }
        Rotate::Portable
    }

    /// `flat` is whole registers of interleaved samples.
    #[inline]
    fn run(self, flat: &mut [f32], phasors: [f32; WIDTH * 2], step4: [f32; 2]) {
        debug_assert_eq!(flat.len() % (WIDTH * 2), 0);
        match self {
            #[cfg(target_arch = "x86_64")]
            // SAFETY: only constructed after the detection in `pick`.
            Rotate::Avx2Fma => unsafe { rotate_avx2(flat, phasors, step4) },
            Rotate::Portable => rotate_wide(flat, phasors, step4),
        }
    }
}

#[cfg(target_arch = "x86_64")]
#[target_feature(enable = "avx2,fma")]
unsafe fn rotate_avx2(flat: &mut [f32], phasors: [f32; WIDTH * 2], step4: [f32; 2]) {
    use std::arch::x86_64::*;
    let mut p = _mm256_loadu_ps(phasors.as_ptr());
    let r_re = _mm256_set1_ps(step4[0]);
    let r_im = _mm256_set1_ps(step4[1]);
    let mut i = 0;
    while i + 8 <= flat.len() {
        let x = _mm256_loadu_ps(flat.as_ptr().add(i));
        let p_re = _mm256_moveldup_ps(p);
        let p_im = _mm256_movehdup_ps(p);
        let x_sw = _mm256_permute_ps(x, 0b10_11_00_01);
        // even lanes: x.re*p.re - x.im*p.im; odd: x.im*p.re + x.re*p.im
        let y = _mm256_fmaddsub_ps(x, p_re, _mm256_mul_ps(x_sw, p_im));
        _mm256_storeu_ps(flat.as_mut_ptr().add(i), y);
        let p_sw = _mm256_permute_ps(p, 0b10_11_00_01);
        p = _mm256_fmaddsub_ps(p, r_re, _mm256_mul_ps(p_sw, r_im));
        i += 8;
    }
}

fn rotate_wide(flat: &mut [f32], phasors: [f32; WIDTH * 2], step4: [f32; 2]) {
    use wide::f32x8;
    let sign = f32x8::from([-1.0, 1.0, -1.0, 1.0, -1.0, 1.0, -1.0, 1.0]);
    let mut p = f32x8::from(phasors);
    let r_re = f32x8::splat(step4[0]);
    let r_im = f32x8::splat(step4[1]) * sign;
    let dup_re = |v: f32x8| {
        let a = v.to_array();
        f32x8::from([a[0], a[0], a[2], a[2], a[4], a[4], a[6], a[6]])
    };
    let dup_im = |v: f32x8| {
        let a = v.to_array();
        f32x8::from([a[1], a[1], a[3], a[3], a[5], a[5], a[7], a[7]])
    };
    let swap = |v: f32x8| {
        let a = v.to_array();
        f32x8::from([a[1], a[0], a[3], a[2], a[5], a[4], a[7], a[6]])
    };
    for reg in flat.chunks_exact_mut(WIDTH * 2) {
        let x = f32x8::from(<[f32; 8]>::try_from(&*reg).unwrap());
        let y = x.mul_add(dup_re(p), swap(x) * (dup_im(p) * sign));
        reg.copy_from_slice(&y.to_array());
        p = p.mul_add(r_re, swap(p) * r_im);
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

    /// One phasor per sample from the exact phase, which is what the block
    /// kernel has to agree with at every sample, not only every fourth.
    fn reference(shift_hz: f64, rate: f64, sig: &[C32]) -> Vec<C32> {
        let step = std::f64::consts::TAU * shift_hz / rate;
        sig.iter()
            .enumerate()
            .map(|(i, x)| {
                let (s, c) = (step * i as f64).sin_cos();
                x * C32::new(c as f32, s as f32)
            })
            .collect()
    }

    #[test]
    fn the_block_kernel_matches_a_phasor_per_sample_across_odd_blocks() {
        // Block sizes that are not whole registers, that straddle an anchor,
        // and that are shorter than one register, so the tail and the phase
        // carried between calls are all exercised.
        let rate = 2.4e6;
        let sig = tone(5_000, 0.013);
        let want = reference(-317_000.0, rate, &sig);
        for chunk in [1usize, 3, 4, 7, 1023, 1025, 5_000] {
            let mut m = Mixer::new(-317_000.0, rate);
            let mut got = Vec::new();
            for c in sig.chunks(chunk) {
                m.process(c, &mut got);
            }
            for (i, (a, b)) in got.iter().zip(&want).enumerate() {
                assert!((a - b).norm() < 2e-5, "chunk {chunk} sample {i}: {a} vs {b}");
            }
        }
    }

    #[test]
    fn both_kernels_agree() {
        let sig = tone(64, 0.021);
        let mut flat_a: Vec<f32> = sig.iter().flat_map(|c| [c.re, c.im]).collect();
        let mut flat_b = flat_a.clone();
        let m = Mixer::new(-100_000.0, 1e6);
        let (p, r) = m.anchor();
        Rotate::Portable.run(&mut flat_a, p, r);
        Rotate::pick().run(&mut flat_b, p, r);
        for (i, (a, b)) in flat_a.iter().zip(&flat_b).enumerate() {
            assert!((a - b).abs() < 1e-5, "lane {i}: {a} vs {b}");
        }
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
