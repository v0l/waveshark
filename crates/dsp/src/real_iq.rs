use crate::fir::{FirDecimReal, lowpass};
use common::C32;

pub const TAPS: usize = 47;
pub const ATTEN_DB: f64 = 80.0;
const DC_ALPHA: f32 = 1e-4;

pub struct RealToIq {
    in_phase: FirDecimReal,
    quadrature: Vec<f32>,
    delay: usize,
    at: usize,
    centre: f32,
    mean: f32,
    phase: usize,
    held: Option<f32>,
    even: Vec<f32>,
    odd: Vec<f32>,
    filtered: Vec<f32>,
}

impl Default for RealToIq {
    fn default() -> Self {
        Self::new()
    }
}

impl RealToIq {
    pub fn new() -> Self {
        let k = (TAPS - 3) / 4;
        let h: Vec<f32> = lowpass(TAPS, 0.25, ATTEN_DB).iter().map(|t| t * 2.0).collect();
        let even: Vec<f32> = h.iter().step_by(2).copied().collect();
        Self {
            in_phase: FirDecimReal::new(even, 1),
            quadrature: vec![0.0; k + 1],
            delay: k + 1,
            at: 0,
            centre: h[2 * k + 1],
            mean: 0.0,
            phase: 0,
            held: None,
            even: Vec::new(),
            odd: Vec::new(),
            filtered: Vec::new(),
        }
    }

    pub fn reset(&mut self) {
        self.in_phase.reset();
        self.quadrature.fill(0.0);
        self.at = 0;
        self.mean = 0.0;
        self.phase = 0;
        self.held = None;
    }

    pub fn latency(&self) -> usize {
        self.delay
    }

    pub fn process(&mut self, real: &[f32], out: &mut Vec<C32>) {
        self.even.clear();
        self.odd.clear();
        for &x in real {
            let x = x - self.mean;
            self.mean += DC_ALPHA * x;
            let v = if self.phase & 2 == 0 { x } else { -x };
            match self.held.take() {
                None => self.held = Some(v),
                Some(i) => {
                    self.even.push(i);
                    self.odd.push(v);
                }
            }
            self.phase = (self.phase + 1) & 3;
        }
        self.filtered.clear();
        self.in_phase.process(&self.even, &mut self.filtered);
        out.reserve(self.filtered.len());
        for (&i, &q) in self.filtered.iter().zip(&self.odd) {
            let late = std::mem::replace(&mut self.quadrature[self.at], q);
            self.at = (self.at + 1) % self.delay;
            out.push(C32::new(i, late * self.centre));
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const RATE: f64 = 20e6;

    fn tone(n: usize, hz: f64, amplitude: f64) -> Vec<f32> {
        (0..n)
            .map(|i| (amplitude * (std::f64::consts::TAU * hz * i as f64 / RATE).cos()) as f32)
            .collect()
    }

    fn convert(x: &[f32]) -> Vec<C32> {
        let mut c = RealToIq::new();
        let mut out = Vec::new();
        c.process(x, &mut out);
        out
    }

    fn read(out: &[C32]) -> (f64, f64) {
        let settled = &out[200..];
        let turn: f64 = settled.windows(2).map(|w| (w[1] * w[0].conj()).arg() as f64).sum::<f64>()
            / (settled.len() - 1) as f64;
        let level = settled.iter().map(|s| s.norm() as f64).sum::<f64>() / settled.len() as f64;
        (turn * RATE / 2.0 / std::f64::consts::TAU, level)
    }

    fn at_db(out: &[C32], hz: f64) -> f64 {
        let settled = &out[200..];
        let step = -std::f64::consts::TAU * hz / (RATE / 2.0);
        let sum = settled
            .iter()
            .enumerate()
            .map(|(n, s)| {
                let r = C32::new((step * n as f64).cos() as f32, (step * n as f64).sin() as f32);
                *s * r
            })
            .sum::<C32>();
        20.0 * (sum.norm() as f64 / settled.len() as f64).log10()
    }

    #[test]
    fn two_real_samples_make_one_complex_one_across_any_split() {
        let x = tone(10_001, 5.3e6, 0.5);
        let whole = convert(&x);
        assert_eq!(whole.len(), 5_000, "the odd last sample waits for its pair");
        let mut c = RealToIq::new();
        let mut parts = Vec::new();
        for chunk in x.chunks(777) {
            c.process(chunk, &mut parts);
        }
        assert_eq!(parts, whole);
    }

    #[test]
    fn a_tone_above_the_quarter_rate_reads_below_the_centre_as_libairspy_1_0_12_turns_it() {
        for offset in [-3.0e6, -200e3, 0.0, 200e3, 3.0e6] {
            let (hz, _) = read(&convert(&tone(40_000, RATE / 4.0 + offset, 0.5)));
            assert!((hz + offset).abs() < 1.0, "{offset} Hz from the quarter read at {hz:.1} Hz");
        }
    }

    #[test]
    fn a_full_scale_tone_comes_out_at_full_scale() {
        for offset in [0.0, 1e6, -2e6, 3e6] {
            let (_, level) = read(&convert(&tone(40_000, RATE / 4.0 + offset, 1.0)));
            assert!((level - 1.0).abs() < 0.001, "{offset} Hz came out at {level:.5}");
        }
    }

    #[test]
    fn a_ten_megahertz_span_is_flat_and_imageless_out_to_three_point_eight_either_side() {
        for offset in [1e6, 2e6, 3e6, 3.5e6, 3.8e6] {
            for side in [1.0, -1.0] {
                let out = convert(&tone(40_000, RATE / 4.0 + side * offset, 1.0));
                let wanted = at_db(&out, -side * offset);
                let image = at_db(&out, side * offset);
                assert!(wanted.abs() < 0.01, "{} Hz passes at {wanted:.3} dB", side * offset);
                assert!(image < -79.0, "the image of {} Hz is at {image:.1} dB", side * offset);
            }
        }
    }

    #[test]
    fn the_edge_of_a_ten_megahertz_span_is_where_the_image_comes_back() {
        let db = |offset: f64| {
            let out = convert(&tone(40_000, RATE / 4.0 + offset, 1.0));
            (at_db(&out, -offset), at_db(&out, offset))
        };
        let (wanted, image) = db(3.97e6);
        assert!(wanted.abs() < 0.01 && (-64.0..-61.0).contains(&image), "{wanted:.3} {image:.1}");
        let (wanted, image) = db(4.2e6);
        assert!(
            (-0.13..-0.11).contains(&wanted) && (-38.0..-36.5).contains(&image),
            "{wanted:.3} {image:.1}"
        );
        let (wanted, image) = db(4.5e6);
        assert!(
            (-0.9..-0.86).contains(&wanted) && (-21.0..-19.5).contains(&image),
            "{wanted:.3} {image:.1}"
        );
    }

    #[test]
    fn the_offset_of_the_converter_is_taken_off_the_edge_of_the_span() {
        let x: Vec<f32> = tone(400_000, RATE / 4.0 + 1e6, 0.1).iter().map(|v| v + 0.2).collect();
        let out = convert(&x);
        let tail = &out[out.len() - 20_000..];
        let edge = at_db(tail, RATE / 4.0);
        let (hz, level) = read(tail);
        assert!(edge < -90.0, "the offset is still at the edge at {edge:.1} dB");
        assert!((hz + 1e6).abs() < 5.0, "the tone moved to {hz:.1} Hz");
        assert!((level - 0.1).abs() < 0.001, "the tone came out at {level:.4}");
    }
}
