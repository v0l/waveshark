//! 256-point real FFT with the packed frequency layout that jtransforms
//! (and therefore jmbe) uses, so the synthesiser algorithm ports literally.
//!
//! Packed layout: `a[0]` holds Re[0], `a[1]` holds Re[128], and for
//! `k` in 1..128 `a[2k]`/`a[2k+1]` hold Re[k]/Im[k].

use rustfft::{FftPlanner, Fft};
use std::sync::Arc;

pub const FFT_SIZE: usize = 256;

pub struct RealFft256 {
    fft: Arc<dyn Fft<f32>>,
    inverse: Arc<dyn Fft<f32>>,
}

impl Default for RealFft256 {
    fn default() -> Self {
        Self::new()
    }
}

impl RealFft256 {
    pub fn new() -> Self {
        let mut planner = FftPlanner::new();
        Self {
            fft: planner.plan_fft_forward(FFT_SIZE),
            inverse: planner.plan_fft_inverse(FFT_SIZE),
        }
    }

    /// Forward transform of 256 real samples, output in the packed layout
    /// described above. Matches jtransforms `realForward(float[])`.
    pub fn forward(&self, a: &mut [f32; FFT_SIZE]) {
        let mut buf: Vec<rustfft::num_complex::Complex<f32>> =
            a.iter().map(|s| rustfft::num_complex::Complex::new(*s, 0.0)).collect();
        self.fft.process(&mut buf);

        a[0] = buf[0].re;
        a[1] = buf[128].re;
        for k in 1..128 {
            a[2 * k] = buf[k].re;
            a[2 * k + 1] = buf[k].im;
        }
    }

    /// Inverse transform of a packed spectrum, scaling by 1/256, producing
    /// 256 real samples. Matches jtransforms `realInverse(float[], true)`.
    pub fn inverse(&self, a: &mut [f32; FFT_SIZE]) {
        let mut buf: Vec<rustfft::num_complex::Complex<f32>> = vec![
            rustfft::num_complex::Complex::ZERO;
            FFT_SIZE
        ];
        buf[0] = rustfft::num_complex::Complex::new(a[0], 0.0);
        buf[128] = rustfft::num_complex::Complex::new(a[1], 0.0);
        for k in 1..128 {
            let z = rustfft::num_complex::Complex::new(a[2 * k], a[2 * k + 1]);
            buf[k] = z;
            buf[FFT_SIZE - k] = z.conj();
        }

        self.inverse.process(&mut buf);

        for n in 0..FFT_SIZE {
            a[n] = buf[n].re / FFT_SIZE as f32;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn round_trip_through_packed_layout() {
        let fft = RealFft256::new();
        let mut a = [0.0f32; 256];
        // Real sine of 4 cycles, matching a Hermitian spectrum.
        for n in 0..256 {
            a[n] = ((2.0 * std::f32::consts::PI * 4.0 * n as f32) / 256.0).sin();
        }

        let original = a;
        fft.forward(&mut a);
        // Bin 4 should carry all the energy in Re/Im slots.
        assert!(a[2 * 4].abs() + a[2 * 4 + 1].abs() > 100.0);
        assert!(a[2 * 5].abs() + a[2 * 5 + 1].abs() < 1e-3);
        assert!(a[0].abs() < 1e-3);

        fft.inverse(&mut a);
        for n in 0..256 {
            assert!((a[n] - original[n]).abs() < 1e-3, "sample {n}");
        }
    }
}
