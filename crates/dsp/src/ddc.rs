use crate::fir::FirDecim;
use crate::mixer::Mixer;
use crate::resample::{self, Rational};
use common::C32;

const ATTEN_DB: f64 = 60.0;
const PASSBAND: f64 = 0.4;
const MAX_DENOMINATOR: usize = 4096;

pub struct Ddc {
    rate_in: f64,
    rate_out: f64,
    shift_hz: f64,
    mixer: Mixer,
    decim: FirDecim,
    resample: Option<Rational>,
    mixed: Vec<C32>,
    narrow: Vec<C32>,
}

impl Ddc {
    pub fn new(rate_in: f64, rate_out: f64, shift_hz: f64) -> Option<Self> {
        let (factor, resample) = resample::stage(rate_in, rate_out, MAX_DENOMINATOR)?;
        let decim = FirDecim::design_hz(rate_in, factor, rate_out * PASSBAND, ATTEN_DB);
        Some(Self {
            rate_in,
            rate_out,
            shift_hz,
            mixer: Mixer::new(-shift_hz, rate_in),
            decim,
            resample,
            mixed: Vec::new(),
            narrow: Vec::new(),
        })
    }

    pub fn rate_in(&self) -> f64 {
        self.rate_in
    }

    pub fn rate_out(&self) -> f64 {
        self.rate_out
    }

    pub fn shift_hz(&self) -> f64 {
        self.shift_hz
    }

    pub fn set_shift(&mut self, shift_hz: f64) {
        self.shift_hz = shift_hz;
        self.mixer.set_shift(-shift_hz, self.rate_in);
    }

    pub fn process(&mut self, input: &[C32], out: &mut Vec<C32>) {
        self.mixed.clear();
        self.mixer.process(input, &mut self.mixed);
        match &mut self.resample {
            None => self.decim.process(&self.mixed, out),
            Some(r) => {
                self.narrow.clear();
                self.decim.process(&self.mixed, &mut self.narrow);
                r.process(&self.narrow, out);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn tone(hz: f64, rate: f64, n: usize) -> Vec<C32> {
        (0..n)
            .map(|k| {
                let p = std::f64::consts::TAU * hz * k as f64 / rate;
                C32::new(p.cos() as f32, p.sin() as f32)
            })
            .collect()
    }

    fn peak_hz(x: &[C32], rate: f64) -> f64 {
        let n = x.len();
        let mut best = (0.0f64, 0.0f64);
        for bin in 0..n {
            let f = bin as f64 * rate / n as f64;
            let f = if f >= rate / 2.0 { f - rate } else { f };
            let mut acc = C32::new(0.0, 0.0);
            for (k, s) in x.iter().enumerate() {
                let p = -std::f64::consts::TAU * f * k as f64 / rate;
                acc += s * C32::new(p.cos() as f32, p.sin() as f32);
            }
            if acc.norm() as f64 > best.1 {
                best = (f, acc.norm() as f64);
            }
        }
        best.0
    }

    #[test]
    fn a_tone_300_khz_up_a_2_4_ms_span_lands_20_khz_up_a_240_khz_cut_at_280() {
        let rate = 2_400_000.0;
        let mut ddc = Ddc::new(rate, 240_000.0, 280_000.0).unwrap();
        let mut out = Vec::new();
        ddc.process(&tone(300_000.0, rate, 240_000), &mut out);
        assert_eq!(out.len(), 24_000);
        let tail = &out[out.len() - 480..];
        assert_eq!(peak_hz(tail, 240_000.0), 20_000.0);
    }

    #[test]
    fn a_rate_no_decimator_reaches_is_resampled_to() {
        let rate = 2_400_000.0;
        let mut ddc = Ddc::new(rate, 1_024_000.0, 0.0).unwrap();
        let mut out = Vec::new();
        ddc.process(&tone(100_000.0, rate, 240_000), &mut out);
        assert_eq!(out.len(), 102_400);
        let tail = &out[out.len() - 512..];
        assert_eq!(peak_hz(tail, 1_024_000.0), 100_000.0);
    }

    #[test]
    fn a_tone_outside_the_cut_is_60_db_down() {
        let rate = 2_400_000.0;
        let mut ddc = Ddc::new(rate, 150_000.0, 0.0).unwrap();
        let mut out = Vec::new();
        ddc.process(&tone(500_000.0, rate, 240_000), &mut out);
        let tail = &out[out.len() - 1000..];
        let power = tail.iter().map(|s| s.norm_sqr()).sum::<f32>() / tail.len() as f32;
        assert!(10.0 * power.log10() < -60.0, "{} dB", 10.0 * power.log10());
    }

    #[test]
    fn a_rate_above_the_span_is_refused() {
        assert!(Ddc::new(2_400_000.0, 3_200_000.0, 0.0).is_none());
    }
}
