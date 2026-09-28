//! Noise reduction on an audio stream, by short-time spectral gain.

use rustfft::num_complex::Complex32;
use rustfft::{Fft, FftPlanner};
use std::collections::VecDeque;
use std::sync::Arc;

pub const DEFAULT_DEPTH_DB: f32 = 15.0;
pub const DEPTH_RANGE_DB: std::ops::RangeInclusive<f64> = 3.0..=30.0;

const FRAME_S: f64 = 0.02;
const POWER_SMOOTHING: f32 = 0.85;
const MINIMUM_WINDOW_S: f64 = 1.5;
const SUBWINDOWS: usize = 8;
const MINIMUM_BIAS: f32 = 2.0;
const DECISION_DIRECTED: f32 = 0.98;
const GATED_AMPLITUDE: f32 = 1e-6;

pub struct Denoiser {
    rate: f64,
    size: usize,
    hop: usize,
    window: Vec<f32>,
    forward: Arc<dyn Fft<f32>>,
    inverse: Arc<dyn Fft<f32>>,
    spectrum: Vec<Complex32>,
    scratch: Vec<Complex32>,
    history: Vec<f32>,
    fresh: usize,
    overlap: Vec<f32>,
    ready: VecDeque<f32>,
    smoothed: Vec<f32>,
    running_min: Vec<f32>,
    minima: VecDeque<Vec<f32>>,
    frames_in_subwindow: usize,
    subwindow_frames: usize,
    noise: Vec<f32>,
    clean: Vec<f32>,
    floor: f32,
    learned: bool,
}

impl Denoiser {
    pub fn new(rate: f64, depth_db: f32) -> Self {
        let size = ((rate * FRAME_S).round() as usize).max(16).next_power_of_two();
        let hop = size / 2;
        let bins = size / 2 + 1;
        let window = (0..size)
            .map(|i| (std::f64::consts::PI * i as f64 / size as f64).sin() as f32)
            .collect();
        let mut planner = FftPlanner::new();
        let forward = planner.plan_fft_forward(size);
        let inverse = planner.plan_fft_inverse(size);
        let scratch_len = forward.get_inplace_scratch_len().max(inverse.get_inplace_scratch_len());
        let frames_per_window = (MINIMUM_WINDOW_S * rate / hop as f64).round() as usize;
        let mut d = Self {
            rate,
            size,
            hop,
            window,
            forward,
            inverse,
            spectrum: vec![Complex32::default(); size],
            scratch: vec![Complex32::default(); scratch_len],
            history: vec![0.0; size],
            fresh: 0,
            overlap: vec![0.0; size],
            ready: VecDeque::with_capacity(2 * size),
            smoothed: vec![0.0; bins],
            running_min: vec![f32::INFINITY; bins],
            minima: VecDeque::with_capacity(SUBWINDOWS),
            frames_in_subwindow: 0,
            subwindow_frames: (frames_per_window / SUBWINDOWS).max(1),
            noise: vec![0.0; bins],
            clean: vec![0.0; bins],
            floor: 1.0,
            learned: false,
        };
        d.set_depth_db(depth_db);
        d
    }

    pub fn rate(&self) -> f64 {
        self.rate
    }

    pub fn latency(&self) -> usize {
        self.size - 1
    }

    pub fn set_depth_db(&mut self, depth_db: f32) {
        self.floor = 10f32.powf(-depth_db.max(0.0) / 20.0);
    }

    pub fn noise_power(&self) -> &[f32] {
        &self.noise
    }

    pub fn frame_len(&self) -> usize {
        self.size
    }

    pub fn reset(&mut self) {
        *self = Self::new(self.rate, -20.0 * self.floor.log10());
    }

    pub fn process(&mut self, audio: &mut [f32]) {
        for s in audio.iter_mut() {
            let at = self.size - self.hop + self.fresh;
            self.history[at] = *s;
            self.fresh += 1;
            if self.fresh == self.hop {
                self.frame();
                self.history.copy_within(self.hop.., 0);
                self.fresh = 0;
            }
            *s = self.ready.pop_front().unwrap_or(0.0);
        }
    }

    fn frame(&mut self) {
        for ((c, &x), &w) in self.spectrum.iter_mut().zip(&self.history).zip(&self.window) {
            *c = Complex32::new(x * w, 0.0);
        }
        self.forward.process_with_scratch(&mut self.spectrum, &mut self.scratch);
        let gated =
            self.history.iter().filter(|x| x.abs() < GATED_AMPLITUDE).count() > self.size / 16;
        if !gated {
            self.learn();
        }
        self.apply_gain();
        self.inverse.process_with_scratch(&mut self.spectrum, &mut self.scratch);
        let scale = 1.0 / self.size as f32;
        for ((o, c), &w) in self.overlap.iter_mut().zip(&self.spectrum).zip(&self.window) {
            *o += c.re * scale * w;
        }
        self.ready.extend(self.overlap.drain(..self.hop));
        self.overlap.resize(self.size, 0.0);
    }

    fn learn(&mut self) {
        let bins = self.smoothed.len();
        for k in 0..bins {
            let p = self.spectrum[k].norm_sqr();
            self.smoothed[k] = if self.learned {
                POWER_SMOOTHING * self.smoothed[k] + (1.0 - POWER_SMOOTHING) * p
            } else {
                p
            };
            self.running_min[k] = self.running_min[k].min(self.smoothed[k]);
        }
        self.frames_in_subwindow += 1;
        if self.frames_in_subwindow >= self.subwindow_frames {
            if self.minima.len() == SUBWINDOWS {
                self.minima.pop_front();
            }
            self.minima.push_back(self.running_min.clone());
            self.running_min.fill(f32::INFINITY);
            self.frames_in_subwindow = 0;
        }
        for k in 0..bins {
            let lowest = self.minima.iter().map(|m| m[k]).fold(self.running_min[k], f32::min);
            self.noise[k] = MINIMUM_BIAS * lowest;
        }
        self.learned = true;
    }

    fn apply_gain(&mut self) {
        let bins = self.smoothed.len();
        if !self.learned {
            return;
        }
        for k in 0..bins {
            let power = self.spectrum[k].norm_sqr();
            let noise = self.noise[k].max(1e-20);
            let posterior = power / noise;
            let prior = DECISION_DIRECTED * self.clean[k] / noise
                + (1.0 - DECISION_DIRECTED) * (posterior - 1.0).max(0.0);
            let gain = (prior / (1.0 + prior)).max(self.floor);
            self.clean[k] = gain * gain * power;
            self.spectrum[k] *= gain;
            if k > 0 && k < self.size - k {
                self.spectrum[self.size - k] *= gain;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::filter::{Biquad, Response};
    use std::f64::consts::TAU;

    const RATE: f64 = 48_000.0;

    struct Gauss(u64);

    impl Gauss {
        fn next(&mut self) -> f32 {
            let mut u = || {
                self.0 ^= self.0 << 13;
                self.0 ^= self.0 >> 7;
                self.0 ^= self.0 << 17;
                ((self.0 >> 11) as f64 + 0.5) / (1u64 << 53) as f64
            };
            let (a, b) = (u(), u());
            ((-2.0 * a.ln()).sqrt() * (TAU * b).cos()) as f32
        }
    }

    fn white(n: usize, rms: f32, seed: u64) -> Vec<f32> {
        let mut g = Gauss(seed | 1);
        (0..n).map(|_| rms * g.next()).collect()
    }

    fn voice_band(x: &mut [f32]) {
        let mut stages = [
            Biquad::design(Response::Highpass, RATE, 300.0, 0.707),
            Biquad::design(Response::Highpass, RATE, 300.0, 0.707),
            Biquad::design(Response::Lowpass, RATE, 3_000.0, 0.707),
            Biquad::design(Response::Lowpass, RATE, 3_000.0, 0.707),
            Biquad::design(Response::Lowpass, RATE, 3_000.0, 0.707),
        ];
        for s in x.iter_mut() {
            *s = stages.iter_mut().fold(*s, |v, b| b.process(v));
        }
    }

    fn hiss(n: usize, rms: f32, seed: u64) -> Vec<f32> {
        let mut x = white(n, 1.0, seed);
        voice_band(&mut x);
        let now = power(&x).sqrt();
        x.iter_mut().for_each(|s| *s *= rms / now);
        x
    }

    fn syllables(n: usize, amp: f32) -> Vec<f32> {
        let pitches = [520.0, 830.0, 1_210.0, 1_740.0, 2_330.0, 990.0, 660.0];
        let (on, off) = ((0.22 * RATE) as usize, (0.11 * RATE) as usize);
        (0..n)
            .map(|i| {
                let (syllable, at) = (i / (on + off), i % (on + off));
                if at >= on {
                    return 0.0;
                }
                let f = pitches[syllable % pitches.len()];
                let fade = (at.min(on - at) as f64 / (0.01 * RATE)).min(1.0);
                let t = i as f64 / RATE;
                let tone = (TAU * f * t).sin() + 0.5 * (TAU * 1.9 * f * t).sin();
                (amp as f64 * fade * tone / 1.118) as f32
            })
            .collect()
    }

    fn power(x: &[f32]) -> f32 {
        (x.iter().map(|v| (*v as f64).powi(2)).sum::<f64>() / x.len().max(1) as f64) as f32
    }

    fn db(x: f32) -> f32 {
        10.0 * x.log10()
    }

    fn denoise(input: &[f32], depth_db: f32) -> (Vec<f32>, usize) {
        let mut d = Denoiser::new(RATE, depth_db);
        let mut out = input.to_vec();
        for block in out.chunks_mut(1_000) {
            d.process(block);
        }
        (out, d.latency())
    }

    struct Reading {
        level_db: f32,
        snr_db: f32,
    }

    fn against(clean: &[f32], out: &[f32], latency: usize, skip: usize) -> Reading {
        let y = &out[skip + latency..];
        let s = &clean[skip..skip + y.len()];
        let cross: f64 = y.iter().zip(s).map(|(a, b)| *a as f64 * *b as f64).sum();
        let own: f64 = s.iter().map(|b| (*b as f64).powi(2)).sum();
        let a = (cross / own) as f32;
        let residual: Vec<f32> = y.iter().zip(s).map(|(y, s)| y - a * s).collect();
        Reading { level_db: 20.0 * a.log10(), snr_db: db(a * a * power(s) / power(&residual)) }
    }

    #[test]
    fn the_noise_floor_is_learned_within_a_decibel_of_the_hiss() {
        let rms = 0.05;
        let mut d = Denoiser::new(RATE, DEFAULT_DEPTH_DB);
        let mut x = white((6.0 * RATE) as usize, rms, 3);
        d.process(&mut x);
        let expected = rms * rms * d.frame_len() as f32 / 2.0;
        let bins = d.noise_power();
        let mean = bins[4..bins.len() - 4].iter().sum::<f32>() / (bins.len() - 8) as f32;
        let off = db(mean / expected);
        assert!(
            (-1.0..=1.0).contains(&off),
            "noise estimate {off:.2} dB from the hiss, -0.11 when measured; floor -1, ceiling 1"
        );
    }

    #[test]
    fn hiss_alone_comes_out_at_the_depth() {
        let x = hiss((8.0 * RATE) as usize, 0.05, 5);
        let (out, _) = denoise(&x, DEFAULT_DEPTH_DB);
        let skip = (3.0 * RATE) as usize;
        let drop = db(power(&out[skip..]) / power(&x[skip..]));
        assert!(
            (-15.5..=-14.0).contains(&drop),
            "hiss moved {drop:.2} dB at a 15 dB depth, -14.95 when measured; floor -15.5, ceiling -14"
        );
    }

    #[test]
    fn syllables_in_hiss_come_out_cleaner_at_the_same_level() {
        let n = (12.0 * RATE) as usize;
        let clean = syllables(n, 0.1);
        let noise = hiss(n, 0.1 * 10f32.powf(-5.0 / 20.0) * power(&clean).sqrt() / 0.1, 9);
        let noisy: Vec<f32> = clean.iter().zip(&noise).map(|(a, b)| a + b).collect();
        let skip = (3.0 * RATE) as usize;
        let before = against(&clean, &noisy, 0, skip);
        let (out, latency) = denoise(&noisy, DEFAULT_DEPTH_DB);
        let after = against(&clean, &out, latency, skip);
        assert!((4.5..=5.5).contains(&before.snr_db), "went in at {:.2} dB", before.snr_db);
        assert!(
            (13.0..=16.0).contains(&after.snr_db),
            "came out at {:.2} dB from 5, 14.27 when measured; floor 13, ceiling 16",
            after.snr_db
        );
        assert!(
            (-1.0..=0.0).contains(&after.level_db),
            "the syllables moved {:.2} dB, -0.47 when measured; floor -1, ceiling 0",
            after.level_db
        );
    }

    #[test]
    fn a_clean_signal_passes_unchanged() {
        let n = (8.0 * RATE) as usize;
        let clean = syllables(n, 0.3);
        let (out, latency) = denoise(&clean, DEFAULT_DEPTH_DB);
        let r = against(&clean, &out, latency, (1.0 * RATE) as usize);
        assert!(
            r.snr_db >= 40.0,
            "a clean signal came out at {:.2} dB, 43.32 when measured",
            r.snr_db
        );
        assert!(r.level_db.abs() <= 0.1, "a clean signal moved {:.2} dB", r.level_db);
    }

    #[test]
    fn a_squelched_gap_does_not_unlearn_the_hiss() {
        let second = RATE as usize;
        let mut x = hiss(8 * second, 0.05, 11);
        x[3 * second..5 * second].fill(0.0);
        let (out, latency) = denoise(&x, DEFAULT_DEPTH_DB);
        let reopened = 5 * second + latency..5 * second + latency + second / 2;
        let drop = db(power(&out[reopened]) / power(&x[5 * second..5 * second + second / 2]));
        assert!(
            (-15.5..=-14.0).contains(&drop),
            "hiss after the gap moved {drop:.2} dB, -14.96 when measured and 0.00 learning from \
             the zeros; floor -15.5, ceiling -14"
        );
    }

    #[test]
    fn a_band_ten_decibels_louder_is_learned_by_the_third_second() {
        let second = RATE as usize;
        let mut x = hiss(4 * second, 0.02, 13);
        x.extend(hiss(4 * second, 0.02 * 10f32.powf(10.0 / 20.0), 17));
        let (out, latency) = denoise(&x, DEFAULT_DEPTH_DB);
        let drops: Vec<f32> = (4..7)
            .map(|s| {
                let at = s * second;
                db(power(&out[at + latency..at + latency + second]) / power(&x[at..at + second]))
            })
            .collect();
        let want = [-2.77, -6.17, -14.98];
        for (d, w) in drops.iter().zip(want) {
            assert!(
                (d - w).abs() <= 1.0,
                "seconds after the rise moved {drops:.2?}, {want:?} when measured"
            );
        }
    }
}
