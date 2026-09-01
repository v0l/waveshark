//! Shallow ASK: amplitude keying where the low level is not silence.
//!
//! [`crate::pulse::OokDetector`] already handles amplitude keying, and for the
//! overwhelming majority of ISM devices it is the right tool: a cheap
//! transmitter keys its oscillator on and off, the low level is the noise
//! floor, and a threshold tracked between noise and signal finds the symbols
//! with no buffering and no latency.
//!
//! It stops working when the low level is a real signal rather than silence.
//! Measured on a synthetic sweep, the OOK detector recovers the pulse train
//! down to about 11 dB of modulation depth and below that returns one
//! continuous mark, because its noise estimate only updates while the input is
//! *below* threshold. A shallow low level never goes below, so the estimate
//! stays at the pre-burst noise floor, the threshold stays under the low
//! level, and the detector latches high for the whole packet. That asymmetry
//! is deliberate and load-bearing elsewhere: it is what stops a strong signal
//! dragging the noise estimate up after itself.
//!
//! So shallow ASK gets the same treatment as FSK: buffer the burst, measure
//! both levels from it, threshold between them. The cost is a burst of latency
//! and a burst of memory, which is why this is a separate detector rather than
//! a change to the OOK one.
//!
//! Sources of shallow depth are real: a transmitter with a poorly keyed PA, a
//! receiver whose AGC is compressing, and, most often, an adjacent channel
//! bleeding into a narrow channelizer bin and filling in the gaps.

use crate::pulse::{dbfs, LevelGate, Package, PulseStats};

#[derive(Clone, Copy, Debug)]
pub struct AskConfig {
    /// Level below the burst threshold for longer than this ends the burst.
    pub reset_us: u32,
    /// Ignore runs shorter than this, in either direction.
    pub min_run_us: u32,
    /// Discard bursts with fewer pulses than this.
    pub min_pulses: usize,
    /// Threshold hysteresis, as a fraction of half the level separation.
    pub hysteresis: f32,
    /// Envelope estimator time constant, in microseconds.
    pub tau_us: f32,
    /// Minimum envelope SNR before a burst is emitted.
    pub min_snr_db: f32,
    /// Hard floor on the carrier-detect threshold, as a multiple of the
    /// tracked noise mean. See [`crate::pulse::PulseConfig`].
    pub noise_threshold_ratio: f32,
    /// Minimum modulation depth, in dB, between the two levels.
    ///
    /// The equivalent of the FSK detector's tone separation check, and needed
    /// for the same reason: any burst at all has *some* amplitude spread, so
    /// without this a steady carrier gets thresholded down the middle and
    /// yields a pulse train made of nothing but its own ripple. Three dB is
    /// low enough to catch a badly compressed signal and far above the ripple
    /// of a clean one.
    pub min_depth_db: f32,
    /// Longest burst held before it is forced out, in microseconds.
    pub max_burst_us: u32,
}

impl Default for AskConfig {
    fn default() -> Self {
        Self {
            reset_us: 4_000,
            min_run_us: 100,
            min_pulses: 4,
            hysteresis: 0.15,
            tau_us: 500.0,
            min_snr_db: 6.0,
            noise_threshold_ratio: 3.5,
            min_depth_db: 3.0,
            max_burst_us: 500_000,
        }
    }
}

/// Amplitude-shift-keying detector for signals the OOK detector latches on.
///
/// Takes an envelope, exactly like [`crate::pulse::OokDetector`], so the two
/// are interchangeable in a chain.
pub struct AskDetector {
    cfg: AskConfig,
    rate: f64,
    us_per_sample: f64,
    gate: LevelGate,
    burst: Vec<f32>,
    burst_start: u64,
    in_burst: bool,
    low_run: u64,
    sample: u64,
    scratch: Vec<f32>,
    last_depth_db: f32,
    stats: PulseStats,
}

impl AskDetector {
    pub fn new(rate: f64, cfg: AskConfig) -> Self {
        Self {
            cfg,
            rate,
            us_per_sample: 1e6 / rate,
            gate: LevelGate::new(
                rate,
                cfg.tau_us,
                cfg.hysteresis,
                cfg.min_snr_db,
                cfg.noise_threshold_ratio,
            ),
            burst: Vec::new(),
            burst_start: 0,
            in_burst: false,
            low_run: 0,
            sample: 0,
            scratch: Vec::new(),
            last_depth_db: 0.0,
            stats: PulseStats::default(),
        }
    }

    pub fn rate(&self) -> f64 {
        self.rate
    }

    pub fn noise_level(&self) -> f32 {
        self.gate.noise_level()
    }

    pub fn signal_level(&self) -> f32 {
        self.gate.signal_level()
    }

    pub fn snr_db(&self) -> f32 {
        self.gate.snr_db()
    }

    /// Modulation depth of the most recent burst, in dB. A number near the
    /// OOK detector's limit of about 11 dB is the signal to look at this
    /// detector's output rather than that one's.
    pub fn depth_db(&self) -> f32 {
        self.last_depth_db
    }

    pub fn stats(&self) -> PulseStats {
        self.stats
    }

    pub fn take_stats(&mut self) -> PulseStats {
        std::mem::take(&mut self.stats)
    }

    pub fn reset(&mut self) {
        self.gate.reset();
        self.burst.clear();
        self.in_burst = false;
        self.low_run = 0;
    }

    /// Feed an envelope block, appending completed bursts to `out`.
    pub fn process(&mut self, env: &[f32], out: &mut Vec<Package>) {
        let reset_samples = (self.cfg.reset_us as f64 / self.us_per_sample) as usize;
        let max_samples = (self.cfg.max_burst_us as f64 / self.us_per_sample) as usize;

        for &v in env {
            // The noise estimate is held still for the duration of a burst:
            // in shallow ASK the low symbol sits below the gate threshold,
            // and a learning estimate would take it for the noise floor and
            // then declare the whole packet absent.
            let high = self.gate.update_learning(v, !self.in_burst);
            self.sample += 1;

            if high {
                self.low_run = 0;
                if !self.in_burst {
                    self.in_burst = true;
                    self.burst.clear();
                    self.burst_start = self.sample - 1;
                }
            } else {
                self.low_run += 1;
            }

            if self.in_burst {
                // Unlike the FSK detector, low samples are kept as values
                // rather than blanked. Here the low level *is* a symbol, and
                // discarding it would leave a burst with only one level in it.
                self.burst.push(v);
                let ended = !high && self.low_run as usize >= reset_samples;
                if ended || self.burst.len() >= max_samples {
                    self.finish(out);
                }
            }
        }
    }

    /// Force out a burst still being collected, for the end of a file where
    /// there is no trailing silence to close it.
    pub fn flush(&mut self, out: &mut Vec<Package>) {
        if self.in_burst {
            self.finish(out);
        }
    }

    fn finish(&mut self, out: &mut Vec<Package>) {
        self.in_burst = false;
        // The silence that ended the burst is not part of it, and leaving it
        // in would drag the low level down onto the noise floor and with it
        // the threshold.
        let tail = (self.low_run as usize).min(self.burst.len());
        self.burst.truncate(self.burst.len() - tail);

        let snr = self.snr_db();
        if snr < self.cfg.min_snr_db {
            self.stats.rejected_low_snr += 1;
            self.burst.clear();
            return;
        }

        let Some((lo, hi)) = crate::twolevel::levels(&mut self.scratch, &self.burst) else {
            self.burst.clear();
            return;
        };
        self.last_depth_db = 20.0 * (hi.max(1e-20) / lo.max(1e-20)).log10();
        if self.last_depth_db < self.cfg.min_depth_db {
            self.stats.rejected_no_separation += 1;
            self.burst.clear();
            return;
        }

        let min_run = ((self.cfg.min_run_us as f64 / self.us_per_sample) as usize).max(1);
        let runs = crate::twolevel::runs(
            &self.burst,
            0.5 * (lo + hi),
            self.cfg.hysteresis * 0.5 * (hi - lo),
            min_run,
            &mut self.stats.rejected_short_marks,
        );
        let pulses = crate::twolevel::pair_runs(&runs, self.us_per_sample, self.cfg.reset_us);

        if pulses.len() >= self.cfg.min_pulses {
            out.push(Package {
                pulses,
                snr_db: snr,
                rssi_dbfs: dbfs(self.gate.signal_level()),
                start_sample: self.burst_start,
                // Stamped by the node that owns this detector, which is where
                // the stream's centre frequency is known.
                center_hz: 0,
                modulation: Some("ASK"),
            });
            self.stats.accepted += 1;
        } else if !pulses.is_empty() {
            self.stats.rejected_too_few_pulses += 1;
        }
        self.burst.clear();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::pulse::{OokDetector, PulseConfig};

    const RATE: f64 = 250_000.0;

    /// An amplitude-keyed envelope with the given modulation depth: marks at
    /// 1.0, gaps at whatever `depth_db` below that comes to.
    fn ask(pulses: &[(u32, u32)], depth_db: f32, noise: f32) -> Vec<f32> {
        let sp = |us: u32| (us as f64 * RATE / 1e6).round() as usize;
        let low = 10f32.powf(-depth_db / 20.0);
        let mut seed = 3u64;
        let mut rng = move || {
            seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
            ((seed >> 33) as f32 / (1u64 << 31) as f32) * noise
        };
        let mut v: Vec<f32> = (0..sp(20_000)).map(|_| 0.01 + rng()).collect();
        for (m, g) in pulses {
            v.extend((0..sp(*m)).map(|_| 1.0 + rng()));
            v.extend((0..sp(*g)).map(|_| low + rng()));
        }
        v.extend((0..sp(20_000)).map(|_| 0.01 + rng()));
        v
    }

    fn train() -> Vec<(u32, u32)> {
        vec![
            (500, 500),
            (1000, 500),
            (500, 500),
            (1000, 500),
            (1000, 500),
            (500, 500),
            (500, 500),
            (1000, 500),
        ]
    }

    fn detect(env: &[f32]) -> Vec<Package> {
        let mut d = AskDetector::new(RATE, AskConfig::default());
        let mut out = Vec::new();
        d.process(env, &mut out);
        d.flush(&mut out);
        out
    }

    #[test]
    fn recovers_timings_the_ook_detector_latches_through() {
        // 6 dB depth: well inside where the OOK detector gives up.
        let env = ask(&train(), 6.0, 0.02);

        let mut ook = OokDetector::new(RATE, PulseConfig::default());
        let mut ook_out = Vec::new();
        ook.process(&env, &mut ook_out);
        let ook_pulses = ook_out.first().map(|p| p.pulses.len()).unwrap_or(0);
        assert!(ook_pulses <= 1, "the OOK detector was supposed to latch, got {ook_pulses}");

        let pkgs = detect(&env);
        assert_eq!(pkgs.len(), 1, "expected one burst, got {}", pkgs.len());
        let want = train();
        assert_eq!(pkgs[0].pulses.len(), want.len(), "{:?}", pkgs[0].pulses);
        for (got, exp) in pkgs[0].pulses.iter().zip(&want) {
            assert!(got.mark.abs_diff(exp.0) < 60, "mark {} vs {}", got.mark, exp.0);
        }
    }

    #[test]
    fn depth_does_not_change_the_pulse_train() {
        // The whole point: the same keying read the same way whether the gaps
        // are silence or merely quieter. 4 dB is a gap at 63% of the mark.
        let want = train();
        for depth_db in [40.0, 20.0, 10.0, 6.0, 4.0] {
            let pkgs = detect(&ask(&want, depth_db, 0.02));
            assert_eq!(pkgs.len(), 1, "{depth_db} dB depth produced {} bursts", pkgs.len());
            assert_eq!(
                pkgs[0].pulses.len(),
                want.len(),
                "{depth_db} dB depth: {:?}",
                pkgs[0].pulses
            );
            for (got, exp) in pkgs[0].pulses.iter().zip(&want) {
                assert!(
                    got.mark.abs_diff(exp.0) < 60,
                    "{depth_db} dB depth: mark {} vs {}",
                    got.mark,
                    exp.0
                );
            }
        }
    }

    #[test]
    fn reports_the_measured_depth() {
        let mut d = AskDetector::new(RATE, AskConfig::default());
        let mut out = Vec::new();
        d.process(&ask(&train(), 8.0, 0.01), &mut out);
        assert_eq!(out.len(), 1, "stats {:?}", d.stats());
        assert!((d.depth_db() - 8.0).abs() < 1.5, "depth read as {} dB", d.depth_db());
    }

    #[test]
    fn a_steady_carrier_is_not_keying() {
        let mut d = AskDetector::new(RATE, AskConfig::default());
        let mut out = Vec::new();
        let env: Vec<f32> = (0..250_000)
            .map(|i| if (20_000..80_000).contains(&i) { 1.0 } else { 0.01 })
            .collect();
        d.process(&env, &mut out);
        assert!(out.is_empty(), "a steady carrier produced {} packages", out.len());
        assert_eq!(d.take_stats().rejected_no_separation, 1, "rejection went unreported");
    }

    #[test]
    fn noise_alone_produces_nothing() {
        let env: Vec<f32> = {
            let mut seed = 11u64;
            (0..250_000)
                .map(|_| {
                    seed =
                        seed.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
                    (seed >> 33) as f32 / (1u64 << 31) as f32 * 0.05
                })
                .collect()
        };
        assert!(detect(&env).is_empty(), "noise produced packages");
    }

    #[test]
    fn block_boundaries_do_not_change_the_result() {
        let env = ask(&train(), 6.0, 0.02);
        let whole = detect(&env);

        let mut d = AskDetector::new(RATE, AskConfig::default());
        let mut got = Vec::new();
        for c in env.chunks(997) {
            d.process(c, &mut got);
        }
        d.flush(&mut got);
        assert_eq!(whole, got, "block splitting changed the pulse train");
    }
}
