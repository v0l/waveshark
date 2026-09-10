//! Window functions.

/// Modified Bessel function of the first kind, order 0. Series converges fast
/// for the arguments a Kaiser window needs (beta up to ~20).
fn bessel_i0(x: f64) -> f64 {
    let mut sum = 1.0f64;
    let mut term = 1.0f64;
    let half = x / 2.0;
    for k in 1..=40 {
        term *= (half / k as f64) * (half / k as f64);
        sum += term;
        if term < sum * 1e-17 {
            break;
        }
    }
    sum
}

/// Kaiser window. `beta` trades main-lobe width against stopband attenuation.
pub fn kaiser(n: usize, beta: f64) -> Vec<f32> {
    if n == 0 {
        return Vec::new();
    }
    if n == 1 {
        return vec![1.0];
    }
    let denom = bessel_i0(beta);
    let m = (n - 1) as f64;
    (0..n)
        .map(|i| {
            let r = 2.0 * i as f64 / m - 1.0;
            (bessel_i0(beta * (1.0 - r * r).max(0.0).sqrt()) / denom) as f32
        })
        .collect()
}

/// Kaiser beta that achieves a given stopband attenuation, per Kaiser's rule.
pub fn kaiser_beta_for_atten(atten_db: f64) -> f64 {
    if atten_db > 50.0 {
        0.1102 * (atten_db - 8.7)
    } else if atten_db >= 21.0 {
        0.5842 * (atten_db - 21.0).powf(0.4) + 0.07886 * (atten_db - 21.0)
    } else {
        0.0
    }
}

pub fn hann(n: usize) -> Vec<f32> {
    if n <= 1 {
        return vec![1.0; n];
    }
    let m = (n - 1) as f64;
    (0..n).map(|i| (0.5 - 0.5 * (2.0 * std::f64::consts::PI * i as f64 / m).cos()) as f32).collect()
}

/// 4-term Blackman-Harris. -92 dB sidelobes, the right default for a waterfall
/// where a strong carrier must not smear across neighbouring bins.
pub fn blackman_harris(n: usize) -> Vec<f32> {
    if n <= 1 {
        return vec![1.0; n];
    }
    const A: [f64; 4] = [0.35875, 0.48829, 0.14128, 0.01168];
    let m = (n - 1) as f64;
    (0..n)
        .map(|i| {
            let t = 2.0 * std::f64::consts::PI * i as f64 / m;
            (A[0] - A[1] * t.cos() + A[2] * (2.0 * t).cos() - A[3] * (3.0 * t).cos()) as f32
        })
        .collect()
}

/// Coherent power gain of a window, used to un-bias spectrum magnitudes.
pub fn coherent_gain(w: &[f32]) -> f32 {
    w.iter().sum::<f32>() / w.len() as f32
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn kaiser_is_symmetric_and_peaks_at_one() {
        let w = kaiser(65, 8.6);
        assert!((w[32] - 1.0).abs() < 1e-6);
        for i in 0..32 {
            assert!((w[i] - w[64 - i]).abs() < 1e-6);
        }
    }

    #[test]
    fn bessel_i0_known_values() {
        assert!((bessel_i0(0.0) - 1.0).abs() < 1e-12);
        assert!((bessel_i0(1.0) - 1.2660658).abs() < 1e-6);
        assert!((bessel_i0(5.0) - 27.239872).abs() < 1e-4);
    }

    #[test]
    fn hann_endpoints_are_zero() {
        let w = hann(16);
        assert!(w[0].abs() < 1e-6);
        assert!(w[15].abs() < 1e-6);
    }
}
