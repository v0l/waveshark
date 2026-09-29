//! VHF omnidirectional range: a bearing in the phase between two 30 Hz tones.

use crate::{FirDecim, Mixer};
use common::C32;

pub const SUBCARRIER_HZ: f64 = 9_960.0;

pub const TONE_HZ: f64 = 30.0;

pub const IDENT_HZ: f64 = 1_020.0;

const SUBCARRIER_PASS_HZ: f64 = 800.0;

const SUBCARRIER_RATE_HZ: f64 = 4_000.0;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Bearing {
    pub radial_deg: f32,
    pub reference_hz: f32,
    pub variable_depth: f32,
}

impl Bearing {
    pub fn plausible(&self) -> bool {
        (250.0..=750.0).contains(&self.reference_hz) && self.variable_depth > 0.05
    }
}

pub struct VorDemod {
    rate: f64,
    window: u64,
    seen: u64,
    started: u64,
    mean: f32,
    var: C32,
    var_n: u64,
    sub: Mixer,
    narrow: FirDecim,
    delay: f64,
    factor: usize,
    out_seen: u64,
    prev: C32,
    reference: C32,
    reference_n: u64,
    ident: C32,
    ident_step: f64,
    ident_phase: f64,
    ident_len: usize,
    ident_n: usize,
    ident_levels: Vec<f32>,
    mixed: Vec<C32>,
    narrowed: Vec<C32>,
}

impl VorDemod {
    pub fn new(rate: f64, window_s: f64) -> Self {
        let factor = (rate / SUBCARRIER_RATE_HZ).floor().max(1.0) as usize;
        let narrow = FirDecim::design_hz(rate, factor, SUBCARRIER_PASS_HZ, 60.0);
        let delay = (narrow.taps() as f64 - 1.0) / 2.0;
        Self {
            rate,
            window: (window_s * rate) as u64,
            seen: 0,
            started: 0,
            mean: 0.0,
            var: C32::default(),
            var_n: 0,
            sub: Mixer::new(-SUBCARRIER_HZ, rate),
            narrow,
            delay,
            factor,
            out_seen: 0,
            prev: C32::new(1.0, 0.0),
            reference: C32::default(),
            reference_n: 0,
            ident: C32::default(),
            ident_step: -std::f64::consts::TAU * IDENT_HZ / rate,
            ident_phase: 0.0,
            ident_len: (rate * IDENT_BLOCK_S) as usize,
            ident_n: 0,
            ident_levels: Vec::new(),
            mixed: Vec::new(),
            narrowed: Vec::new(),
        }
    }

    fn tone(&self, at: f64) -> C32 {
        let ph = -std::f64::consts::TAU * TONE_HZ * at / self.rate;
        C32::new(ph.cos() as f32, ph.sin() as f32)
    }

    pub fn take_ident_levels(&mut self) -> Vec<f32> {
        std::mem::take(&mut self.ident_levels)
    }

    pub fn push(&mut self, iq: &[C32], out: &mut Vec<Bearing>) {
        let mut am = Vec::with_capacity(iq.len());
        let follow = (1.0 / (self.rate * LEVEL_S)) as f32;
        if self.mean <= 0.0 && !iq.is_empty() {
            self.mean = iq.iter().map(|x| x.norm()).sum::<f32>() / iq.len() as f32;
        }
        for x in iq {
            let e = x.norm();
            self.mean += (e - self.mean) * follow;
            am.push(e / self.mean.max(f32::MIN_POSITIVE) - 1.0);
        }
        for (k, &a) in am.iter().enumerate() {
            let at = (self.seen + k as u64) as f64;
            self.var += self.tone(at) * a;
            self.var_n += 1;
            let (s, c) = self.ident_phase.sin_cos();
            self.ident_phase =
                (self.ident_phase + self.ident_step).rem_euclid(std::f64::consts::TAU);
            self.ident += C32::new(c as f32, s as f32) * a;
            self.ident_n += 1;
            if self.ident_n == self.ident_len {
                self.ident_levels.push(self.ident.norm() / self.ident_n as f32);
                self.ident = C32::default();
                self.ident_n = 0;
            }
        }
        let real: Vec<C32> = am.iter().map(|a| C32::new(*a, 0.0)).collect();
        self.mixed.clear();
        self.sub.process(&real, &mut self.mixed);
        self.narrowed.clear();
        self.narrow.process(&self.mixed, &mut self.narrowed);
        let sub_rate = self.rate / self.factor as f64;
        for &z in &self.narrowed {
            let f = (z * self.prev.conj()).arg() as f64 * sub_rate / std::f64::consts::TAU;
            self.prev = z;
            let at = (self.out_seen as f64 + 0.5) * self.factor as f64 - self.delay;
            self.out_seen += 1;
            if at < self.started as f64 {
                continue;
            }
            self.reference += self.tone(at) * f as f32;
            self.reference_n += 1;
        }
        self.seen += iq.len() as u64;
        if self.seen - self.started >= self.window {
            if self.var_n > 0 && self.reference_n > 0 {
                let depth = 2.0 * self.var.norm() / self.var_n as f32;
                let dev = 2.0 * self.reference.norm() / self.reference_n as f32;
                let radial = (self.reference.arg() - self.var.arg()).to_degrees().rem_euclid(360.0);
                out.push(Bearing { radial_deg: radial, reference_hz: dev, variable_depth: depth });
            }
            self.started = self.seen;
            self.var = C32::default();
            self.var_n = 0;
            self.reference = C32::default();
            self.reference_n = 0;
        }
    }
}

pub const IDENT_BLOCK_S: f64 = 0.01;

const LEVEL_S: f64 = 2.0;

#[cfg(test)]
mod tests {
    use super::*;

    pub fn keyed(radial_deg: f64, rate: f64, seconds: f64) -> Vec<C32> {
        let n = (rate * seconds) as usize;
        let mut sub_phase = 0.0f64;
        (0..n)
            .map(|i| {
                let t = i as f64 / rate;
                let w = std::f64::consts::TAU * TONE_HZ * t;
                let reference = (w).cos();
                let variable = (w - radial_deg.to_radians()).cos();
                sub_phase += std::f64::consts::TAU * (SUBCARRIER_HZ + 480.0 * reference) / rate;
                let a = 1.0 + 0.3 * variable + 0.3 * sub_phase.cos();
                C32::new(a as f32, 0.0)
            })
            .collect()
    }

    #[test]
    fn a_keyed_radial_is_read_back() {
        let rate = 48_000.0;
        for want in [0.0, 45.0, 91.4, 180.0, 253.5, 359.0] {
            let iq = keyed(want, rate, 3.0);
            let mut d = VorDemod::new(rate, 1.0);
            let mut out = Vec::new();
            for b in iq.chunks(4096) {
                d.push(b, &mut out);
            }
            let last = out.last().expect("a bearing");
            let err = ((last.radial_deg as f64 - want + 540.0) % 360.0 - 180.0).abs();
            assert!(err < 1.0, "{want}: read {last:?}");
            assert!(last.plausible(), "{last:?}");
            assert!((last.reference_hz - 480.0).abs() < 30.0, "{last:?}");
        }
    }
}
