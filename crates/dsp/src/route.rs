//! One gate, one burst, one front end: classify first, then demodulate.
//!
//! The ISM channel graph used to run an envelope path and a discriminator path
//! over every channel unconditionally, because which one a device uses is not
//! knowable in advance and is not visible in a waterfall either. That is the
//! right answer when nothing measures the burst. Now something does, so the
//! order inverts: gate the burst once, measure it with [`crate::classify`],
//! and hand it to the one front end that can read it.
//!
//! # What this costs, and what it saves
//!
//! It saves a front end per channel. It costs a few FFTs per burst, and bursts
//! are rare: on a quiet band the whole path is the gate, and the gate is what
//! the old arrangement paid twice for anyway.
//!
//! # Refusing is not the same as failing
//!
//! A burst the classifier will not name is sent to *both* the on-off and the
//! two-level front ends, which is exactly what the old graph did with every
//! burst. So a refusal costs what the previous arrangement always cost, and
//! never loses a decode that used to work. That property is worth more than
//! the saving: a classifier that occasionally says nothing is a performance
//! question, and one that occasionally says the wrong thing would be a
//! correctness one.
//!
//! # Why the front ends still see silence
//!
//! Each of them gates the burst again, on its own terms, and a gate needs
//! noise to measure a signal against. So the burst handed over carries a
//! margin of the samples either side, which is also what the pulse layer needs
//! to place the gap that ends a package.

use crate::c4fm::{C4fmConfig, C4fmDetector, SymbolBurst};
use crate::classify::{BurstClass, ClassifyConfig, Classifier, Modulation};
use crate::pulse::{LevelGate, Package};
use crate::{AskConfig, AskDetector, FskConfig, FskDetector, OokDetector, PulseConfig};
use common::C32;
use std::collections::VecDeque;

#[derive(Clone, Copy, Debug)]
pub struct RouterConfig {
    /// Silence that ends a burst, in microseconds. Wider than any one
    /// protocol's inter-symbol gap, or a packet arrives in pieces.
    pub reset_us: u32,
    /// Samples kept either side of the burst, in microseconds, so the front
    /// ends have noise to measure against and a gap to end the package with.
    pub margin_us: u32,
    /// Longest burst held, in microseconds.
    pub max_burst_us: u32,
    /// Envelope estimator time constant, in microseconds.
    pub tau_us: f32,
    /// Minimum SNR before a burst is opened at all.
    pub min_snr_db: f32,
    pub noise_threshold_ratio: f32,
    pub classify: ClassifyConfig,
    pub ook: PulseConfig,
    pub ask: AskConfig,
    pub fsk: FskConfig,
    pub c4fm: C4fmConfig,
}

impl Default for RouterConfig {
    fn default() -> Self {
        Self {
            reset_us: 4_000,
            margin_us: 2_000,
            max_burst_us: 500_000,
            tau_us: 500.0,
            min_snr_db: 6.0,
            noise_threshold_ratio: 3.5,
            classify: ClassifyConfig::default(),
            ook: PulseConfig { min_pulses: 8, ..Default::default() },
            ask: AskConfig::default(),
            fsk: FskConfig { min_pulses: 8, ..Default::default() },
            c4fm: C4fmConfig::default(),
        }
    }
}

/// One burst, what it was measured to be, and whatever the front end it was
/// sent to made of it.
#[derive(Clone, Debug)]
pub struct RoutedBurst {
    pub class: BurstClass,
    /// Which front ends ran. Two of them means the classifier refused and the
    /// burst was tried both ways.
    pub routed_to: &'static str,
    /// Mark and gap timings, for the bursts that went to an amplitude or
    /// two-level front end.
    pub packages: Vec<Package>,
    /// Four-level symbols, for the bursts that went to [`C4fmDetector`].
    pub symbols: Vec<SymbolBurst>,
    pub start_sample: u64,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct RouterStats {
    pub bursts: u64,
    pub to_ook: u64,
    pub to_ask: u64,
    pub to_fsk: u64,
    pub to_c4fm: u64,
    /// Bursts the classifier would not name, sent to both pulse front ends.
    pub refused: u64,
    /// Bursts named as something no front end here reads: a carrier, a chirp,
    /// a phase-keyed signal, or noise.
    pub no_front_end: u64,
}

pub struct BurstRouter {
    cfg: RouterConfig,
    rate: f64,
    gate: LevelGate,
    classifier: Classifier,
    /// Samples before the burst opened, always kept.
    pre: VecDeque<C32>,
    margin: usize,
    burst: Vec<C32>,
    /// Samples of silence collected since the burst's last loud sample. They
    /// belong to the burst as its trailing margin, up to the margin length.
    tail: usize,
    in_burst: bool,
    low_run: u64,
    sample: u64,
    burst_start: u64,
    env: Vec<f32>,
    stats: RouterStats,
}

impl BurstRouter {
    pub fn new(rate: f64, cfg: RouterConfig) -> Self {
        let margin = ((cfg.margin_us as f64 * rate / 1e6) as usize).max(1);
        Self {
            cfg,
            rate,
            gate: LevelGate::new(rate, cfg.tau_us, 0.3, cfg.min_snr_db, cfg.noise_threshold_ratio),
            classifier: Classifier::new(rate, cfg.classify),
            pre: VecDeque::with_capacity(margin + 1),
            margin,
            burst: Vec::new(),
            tail: 0,
            in_burst: false,
            low_run: 0,
            sample: 0,
            burst_start: 0,
            env: Vec::new(),
            stats: RouterStats::default(),
        }
    }

    pub fn stats(&self) -> RouterStats {
        self.stats
    }

    pub fn take_stats(&mut self) -> RouterStats {
        std::mem::take(&mut self.stats)
    }

    pub fn snr_db(&self) -> f32 {
        self.gate.snr_db()
    }

    pub fn reset(&mut self) {
        self.gate.reset();
        self.pre.clear();
        self.burst.clear();
        self.in_burst = false;
        self.low_run = 0;
        self.tail = 0;
    }

    pub fn process(&mut self, input: &[C32], out: &mut Vec<RoutedBurst>) {
        let reset_samples = ((self.cfg.reset_us as f64 * self.rate / 1e6) as u64).max(1);
        let max_samples = ((self.cfg.max_burst_us as f64 * self.rate / 1e6) as usize).max(1);

        for &x in input {
            let high = self.gate.update(x.norm());
            self.sample += 1;

            if high {
                self.low_run = 0;
                if !self.in_burst {
                    self.in_burst = true;
                    self.burst.clear();
                    self.burst.extend(self.pre.iter().copied());
                    self.burst_start = self.sample.saturating_sub(self.pre.len() as u64 + 1);
                }
                self.tail = 0;
                self.burst.push(x);
            } else {
                self.low_run += 1;
                if self.in_burst {
                    self.burst.push(x);
                    self.tail += 1;
                    if self.low_run >= reset_samples {
                        self.finish(out);
                    }
                }
            }

            if self.pre.len() == self.margin {
                self.pre.pop_front();
            }
            self.pre.push_back(x);

            if self.in_burst && self.burst.len() >= max_samples {
                self.finish(out);
            }
        }
    }

    /// Force out any burst still being collected, for the end of a file.
    pub fn flush(&mut self, out: &mut Vec<RoutedBurst>) {
        if self.in_burst {
            self.finish(out);
        }
    }

    fn finish(&mut self, out: &mut Vec<RoutedBurst>) {
        self.in_burst = false;
        // Keep a margin of the trailing silence and drop the rest: the gap
        // that ends the last package comes from it, and the rest is the empty
        // channel, which drags every measurement of the burst toward noise.
        let keep = self.burst.len() - self.tail.saturating_sub(self.margin);
        self.burst.truncate(keep);
        self.low_run = 0;
        self.tail = 0;

        let burst = std::mem::take(&mut self.burst);
        let class = self.classifier.classify(&burst);
        self.stats.bursts += 1;

        let mut packages = Vec::new();
        let mut symbols = Vec::new();
        let routed_to = match class.modulation {
            Modulation::Ook => {
                self.stats.to_ook += 1;
                self.run_ook(&burst, &mut packages);
                "ook"
            }
            Modulation::Ask => {
                self.stats.to_ask += 1;
                self.run_ask(&burst, &mut packages);
                "ask"
            }
            Modulation::Fsk2 | Modulation::Msk => {
                self.stats.to_fsk += 1;
                self.run_fsk(&burst, &mut packages);
                "fsk"
            }
            Modulation::Fsk4 => {
                self.stats.to_c4fm += 1;
                // The classifier measured the symbol rate on the way to
                // deciding there were four levels, and the four-level front
                // end cannot open an eye without it.
                let mut cfg = self.cfg.c4fm;
                if class.features.baud > 0.0 {
                    cfg.baud = class.features.baud as f64;
                }
                let mut det = C4fmDetector::new(self.rate, cfg);
                det.process(&burst, &mut symbols);
                det.flush(&mut symbols);
                "c4fm"
            }
            Modulation::Unknown => {
                self.stats.refused += 1;
                self.run_ook(&burst, &mut packages);
                self.run_fsk(&burst, &mut packages);
                "ook+fsk"
            }
            _ => {
                self.stats.no_front_end += 1;
                "none"
            }
        };

        out.push(RoutedBurst {
            class,
            routed_to,
            packages,
            symbols,
            start_sample: self.burst_start,
        });
        self.burst = burst;
        self.burst.clear();
    }

    fn run_ook(&mut self, burst: &[C32], out: &mut Vec<Package>) {
        self.env.clear();
        self.env.extend(burst.iter().map(|c| c.norm()));
        let mut det = OokDetector::new(self.rate, self.cfg.ook);
        let from = out.len();
        det.process(&self.env, out);
        det.flush(out);
        Self::stamp(&mut out[from..], self.burst_start, "OOK");
    }

    fn run_ask(&mut self, burst: &[C32], out: &mut Vec<Package>) {
        self.env.clear();
        self.env.extend(burst.iter().map(|c| c.norm()));
        let mut det = AskDetector::new(self.rate, self.cfg.ask);
        let from = out.len();
        det.process(&self.env, out);
        det.flush(out);
        Self::stamp(&mut out[from..], self.burst_start, "ASK");
    }

    fn run_fsk(&mut self, burst: &[C32], out: &mut Vec<Package>) {
        let mut det = FskDetector::new(self.rate, self.cfg.fsk);
        let from = out.len();
        det.process(burst, out);
        det.flush(out);
        Self::stamp(&mut out[from..], self.burst_start, "FSK");
    }

    /// Put the package back on the stream's own timeline, and record which
    /// front end read it.
    ///
    /// Each front end sees one burst starting at sample zero, so its idea of
    /// when the burst happened is an offset into a buffer nothing downstream
    /// has ever seen. The modulation is stamped per front end rather than from
    /// the classification, so that a refused burst tried both ways still says
    /// which of the two produced the package.
    fn stamp(pkgs: &mut [Package], start: u64, modulation: &'static str) {
        for p in pkgs.iter_mut() {
            p.start_sample += start;
            p.modulation = Some(modulation);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const RATE: f64 = 250_000.0;

    /// An on-off keyed burst: `bits` at `sym_us` each, with silence either
    /// side.
    fn ook_burst(bits: &[u8], sym_us: u32) -> Vec<C32> {
        let mut v = silence(20_000);
        let mut phase = 0.0f64;
        for &b in bits {
            let amp = if b != 0 { 1.0 } else { 0.0 };
            for _ in 0..samples(sym_us) {
                phase += std::f64::consts::TAU * 3_000.0 / RATE;
                v.push(C32::new(amp * phase.cos() as f32, amp * phase.sin() as f32));
            }
        }
        v.extend(silence(20_000));
        v
    }

    /// A two-level FSK burst at 20 kHz separation.
    fn fsk_burst(bits: &[u8], sym_us: u32) -> Vec<C32> {
        let mut v = silence(20_000);
        let mut phase = 0.0f64;
        for &b in bits {
            let f = if b != 0 { 10_000.0 } else { -10_000.0 };
            for _ in 0..samples(sym_us) {
                phase += std::f64::consts::TAU * f / RATE;
                v.push(C32::new(phase.cos() as f32, phase.sin() as f32));
            }
        }
        v.extend(silence(20_000));
        v
    }

    fn samples(us: u32) -> usize {
        (us as f64 * RATE / 1e6) as usize
    }

    fn silence(us: u32) -> Vec<C32> {
        let mut seed = 7u64;
        (0..samples(us))
            .map(|_| {
                let mut r = || {
                    seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1);
                    ((seed >> 40) as f32 / (1u64 << 23) as f32 - 1.0) * 0.01
                };
                C32::new(r(), r())
            })
            .collect()
    }

    /// Pseudorandom bits. A periodic test pattern would put a line in the
    /// transition spectrum at the pattern's own rate, which is a signal no
    /// device sends and an estimator no receiver needs.
    fn pattern(n: usize) -> Vec<u8> {
        let mut seed = 99u64;
        (0..n)
            .map(|_| {
                seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1);
                ((seed >> 42) & 1) as u8
            })
            .collect()
    }

    fn route(iq: &[C32]) -> (Vec<RoutedBurst>, BurstRouter) {
        let cfg = RouterConfig {
            classify: ClassifyConfig { channel_hz: RATE as f32, ..Default::default() },
            ..Default::default()
        };
        let mut r = BurstRouter::new(RATE, cfg);
        let mut out = Vec::new();
        r.process(iq, &mut out);
        r.flush(&mut out);
        (out, r)
    }

    #[test]
    fn an_on_off_burst_goes_to_the_pulse_front_end_and_comes_back_with_timings() {
        let (bursts, mut r) = route(&ook_burst(&pattern(120), 500));
        assert_eq!(bursts.len(), 1, "expected one burst, got {}", bursts.len());
        assert_eq!(bursts[0].routed_to, "ook", "class was {:?}", bursts[0].class.modulation);
        assert!(!bursts[0].packages.is_empty(), "the front end produced no packages");
        assert_eq!(r.take_stats().to_ook, 1);
    }

    #[test]
    fn a_frequency_keyed_burst_goes_to_the_two_level_front_end() {
        let (bursts, mut r) = route(&fsk_burst(&pattern(120), 500));
        assert_eq!(bursts.len(), 1);
        assert_eq!(bursts[0].routed_to, "fsk", "class was {:?}", bursts[0].class.modulation);
        assert!(!bursts[0].packages.is_empty(), "the front end produced no packages");
        let s = r.take_stats();
        assert_eq!((s.to_fsk, s.to_ook), (1, 0));
    }

    #[test]
    fn a_burst_the_classifier_will_not_name_is_tried_both_ways() {
        // Under a millisecond of signal, which is fewer samples than any
        // spectrum can be measured from. The cheapest way to get a refusal.
        let (bursts, mut r) = route(&ook_burst(&[1, 0, 1, 0], 100));
        assert_eq!(bursts.len(), 1);
        assert_eq!(bursts[0].class.modulation, Modulation::Unknown);
        assert_eq!(bursts[0].routed_to, "ook+fsk", "a refusal must not lose the burst");
        assert_eq!(r.take_stats().refused, 1);
    }

    #[test]
    fn the_package_lands_where_the_burst_did() {
        let lead = samples(20_000) as u64;
        let (bursts, _) = route(&ook_burst(&pattern(120), 500));
        let p = &bursts[0].packages[0];
        // Within the margin of the leading silence, not at sample zero.
        let from_start = p.start_sample as i64 - lead as i64;
        assert!(
            from_start.abs() < samples(4_000) as i64,
            "package placed {from_start} samples from the burst"
        );
    }

    #[test]
    fn two_bursts_are_two_bursts() {
        let mut iq = ook_burst(&pattern(60), 500);
        iq.extend(fsk_burst(&pattern(60), 500));
        let (bursts, _) = route(&iq);
        assert_eq!(bursts.len(), 2, "got {} bursts", bursts.len());
        assert_eq!(bursts[0].routed_to, "ook");
        assert_eq!(bursts[1].routed_to, "fsk");
    }

    #[test]
    fn a_quiet_channel_produces_nothing() {
        let (bursts, _) = route(&silence(100_000));
        assert!(bursts.is_empty(), "silence produced {} bursts", bursts.len());
    }

    #[test]
    fn block_boundaries_do_not_change_the_result() {
        let iq = ook_burst(&pattern(120), 500);
        let (whole, _) = route(&iq);

        let cfg = RouterConfig {
            classify: ClassifyConfig { channel_hz: RATE as f32, ..Default::default() },
            ..Default::default()
        };
        let mut split = BurstRouter::new(RATE, cfg);
        let mut got = Vec::new();
        for c in iq.chunks(997) {
            split.process(c, &mut got);
        }
        split.flush(&mut got);
        assert_eq!(whole.len(), got.len());
        assert_eq!(whole[0].packages, got[0].packages, "block splitting changed the packages");
    }
}
