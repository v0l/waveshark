//! FSK pulse extraction: a two-level frequency burst to mark/gap timings.
//!
//! The OOK detector in [`crate::pulse`] cannot see FSK at all. A two-level FSK
//! transmitter keys the *frequency* and leaves the amplitude alone, so its
//! envelope is a flat rectangle: one long mark, no timings, nothing to slice.
//! Roughly a third of the devices rtl_433 supports are FSK, and every one of
//! them looks like a single featureless blob to an envelope detector.
//!
//! The fix is to run a discriminator and threshold *that* instead, which puts
//! the output back in the same mark/gap vocabulary the slicers and protocols
//! already speak. By convention the higher of the two tones is the mark, which
//! is what rtl_433 assumes, so published protocol definitions transcribe
//! unchanged.
//!
//! # Why the burst is buffered
//!
//! The two tones sit at unknown frequencies. The tuner is off by its crystal
//! error, the transmitter is off by its own, and the pair drifts with
//! temperature, so nothing here can be a constant. What *is* stable is that
//! within one burst the two tones are separated by the protocol's deviation
//! and both are visible.
//!
//! So a burst is collected while the carrier is up and thresholded afterwards,
//! against levels measured from the burst itself. A streaming min/max tracker
//! avoids the buffer but has to be seeded from the first few symbols, and
//! seeding it on a preamble that is all one tone puts the threshold in the
//! wrong place for the entire packet. A burst is a few tens of milliseconds at
//! most, so the memory is nothing and the latency is invisible.
//!
//! The thresholding itself is in [`crate::twolevel`], shared with the ASK
//! detector, which has the same problem with amplitudes that this one has with
//! frequencies.

use crate::pulse::{dbfs, LevelGate, Package, PulseStats};
use common::C32;

#[derive(Clone, Copy, Debug)]
pub struct FskConfig {
    /// Carrier absent for longer than this ends the burst, in microseconds.
    pub reset_us: u32,
    /// Ignore tone runs shorter than this, in either direction.
    ///
    /// Lower than the OOK default because FSK devices are usually the faster
    /// ones: 20 to 40 kbit/s is common, which is 25 to 50 us per symbol.
    pub min_run_us: u32,
    /// Discard bursts with fewer pulses than this.
    pub min_pulses: usize,
    /// Threshold hysteresis, as a fraction of half the tone separation.
    pub hysteresis: f32,
    /// Envelope estimator time constant, in microseconds.
    pub tau_us: f32,
    /// Minimum envelope SNR before a burst is emitted.
    pub min_snr_db: f32,
    /// Hard floor on the carrier-detect threshold, as a multiple of the
    /// tracked noise mean. See [`crate::pulse::PulseConfig`].
    pub noise_threshold_ratio: f32,
    /// Minimum separation between the two tones, in hertz.
    ///
    /// This is the test that stops the detector inventing data. An unmodulated
    /// carrier, an OOK burst or a patch of noise all still produce *some*
    /// spread of instantaneous frequency, and thresholding at its midpoint
    /// yields a plausible-looking pulse train made of nothing. Requiring a
    /// real deviation is what separates FSK from everything else on the band.
    pub min_separation_hz: f32,
    /// Longest burst held before it is forced out, in microseconds. A stuck
    /// carrier must not grow the buffer without limit.
    pub max_burst_us: u32,
}

impl Default for FskConfig {
    fn default() -> Self {
        Self {
            reset_us: 1_000,
            min_run_us: 20,
            min_pulses: 8,
            hysteresis: 0.1,
            tau_us: 500.0,
            min_snr_db: 6.0,
            noise_threshold_ratio: 3.5,
            // Well under the ~30 kHz deviation typical of 868 and 915 MHz
            // telemetry, and far above the spread a carrier alone produces.
            min_separation_hz: 4_000.0,
            max_burst_us: 500_000,
        }
    }
}

/// Two-level FSK pulse detector.
///
/// Consumes complex baseband, unlike the OOK detector which takes an envelope,
/// because it needs both the amplitude (to know when a burst is happening) and
/// the phase (to know which tone is being sent).
pub struct FskDetector {
    cfg: FskConfig,
    rate: f64,
    us_per_sample: f64,
    gate: LevelGate,
    prev: C32,
    /// Instantaneous frequency in hertz for the burst being collected. NaN
    /// marks a sample the carrier dropped out for: it still occupies time, but
    /// its frequency is noise and must not reach the level estimate.
    burst: Vec<f32>,
    burst_start: u64,
    in_burst: bool,
    /// Samples since the carrier went away, whether or not a burst is open.
    low_run: u64,
    sample: u64,
    scratch: Vec<f32>,
    last_separation_hz: f32,
    stats: PulseStats,
}

impl FskDetector {
    pub fn new(rate: f64, cfg: FskConfig) -> Self {
        Self {
            cfg,
            rate,
            us_per_sample: 1e6 / rate,
            gate: LevelGate::new(
                rate,
                cfg.tau_us,
                // The envelope gate only decides where the burst starts and
                // stops, so it wants far more hysteresis than a detector
                // reading data off the same signal would.
                0.3,
                cfg.min_snr_db,
                cfg.noise_threshold_ratio,
            ),
            prev: C32::new(0.0, 0.0),
            burst: Vec::new(),
            burst_start: 0,
            in_burst: false,
            low_run: 0,
            sample: 0,
            scratch: Vec::new(),
            last_separation_hz: 0.0,
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

    /// Tone separation of the most recent burst, in hertz. Worth showing: it
    /// is the deviation the protocol tables quote, so it identifies a device
    /// family before anything has decoded.
    pub fn separation_hz(&self) -> f32 {
        self.last_separation_hz
    }

    pub fn stats(&self) -> PulseStats {
        self.stats
    }

    pub fn take_stats(&mut self) -> PulseStats {
        std::mem::take(&mut self.stats)
    }

    pub fn reset(&mut self) {
        self.gate.reset();
        self.prev = C32::new(0.0, 0.0);
        self.burst.clear();
        self.in_burst = false;
        self.low_run = 0;
    }

    /// Feed a block of complex baseband, appending completed bursts to `out`.
    pub fn process(&mut self, input: &[C32], out: &mut Vec<Package>) {
        let reset_samples = (self.cfg.reset_us as f64 / self.us_per_sample) as usize;
        let max_samples = (self.cfg.max_burst_us as f64 / self.us_per_sample) as usize;
        let hz_per_rad = (self.rate / std::f64::consts::TAU) as f32;

        for &x in input {
            let d = x * self.prev.conj();
            self.prev = x;
            let freq = if d.norm_sqr() > 0.0 { d.arg() * hz_per_rad } else { 0.0 };
            let high = self.gate.update(x.norm());
            self.sample += 1;

            if high {
                self.low_run = 0;
                if !self.in_burst {
                    self.in_burst = true;
                    self.burst.clear();
                    self.burst_start = self.sample - 1;
                }
                self.burst.push(freq);
            } else {
                self.low_run += 1;
                if self.in_burst {
                    // Hold the sample as a timing placeholder but not as
                    // evidence about either tone. A brief dropout inside a
                    // packet is a fade, not a symbol.
                    self.burst.push(f32::NAN);
                    if self.low_run as usize >= reset_samples {
                        self.finish(out);
                    }
                }
            }

            if self.in_burst && self.burst.len() >= max_samples {
                self.finish(out);
            }
        }
    }

    /// Force out any burst still being collected. Needed at the end of a file,
    /// where there is no trailing silence to close the last packet.
    pub fn flush(&mut self, out: &mut Vec<Package>) {
        if self.in_burst {
            self.finish(out);
        }
    }

    fn finish(&mut self, out: &mut Vec<Package>) {
        self.in_burst = false;
        // Trailing dropout samples belong to the silence that ended the burst,
        // not to the burst.
        while self.burst.last().is_some_and(|v| v.is_nan()) {
            self.burst.pop();
        }
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
        self.last_separation_hz = hi - lo;
        if self.last_separation_hz < self.cfg.min_separation_hz {
            self.stats.rejected_no_separation += 1;
            self.burst.clear();
            return;
        }

        let min_run = ((self.cfg.min_run_us as f64 / self.us_per_sample) as usize).max(1);
        let runs = crate::twolevel::runs(
            &self.burst,
            0.5 * (lo + hi),
            self.cfg.hysteresis * 0.5 * self.last_separation_hz,
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
                modulation: Some("FSK"),
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

    const RATE: f64 = 250_000.0;

    /// Build a two-level FSK burst from a list of (level, microseconds), with
    /// silence either side. `offset_hz` is the tuning error both tones ride on.
    fn burst(
        symbols: &[(bool, u32)],
        deviation_hz: f64,
        offset_hz: f64,
        amp: f32,
        noise: f32,
    ) -> Vec<C32> {
        let sp = |us: u32| (us as f64 * RATE / 1e6).round() as usize;
        let mut seed = 12345u64;
        let mut rng = move || {
            seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
            ((seed >> 33) as f32 / (1u64 << 30) as f32 - 1.0) * noise
        };
        let mut v = Vec::new();
        let mut phase = 0.0f64;
        for _ in 0..sp(5_000) {
            v.push(C32::new(rng(), rng()));
        }
        for (level, us) in symbols {
            let f = offset_hz + if *level { deviation_hz } else { -deviation_hz };
            for _ in 0..sp(*us) {
                phase = (phase + std::f64::consts::TAU * f / RATE).rem_euclid(std::f64::consts::TAU);
                v.push(C32::new(
                    amp * phase.cos() as f32 + rng(),
                    amp * phase.sin() as f32 + rng(),
                ));
            }
        }
        for _ in 0..sp(5_000) {
            v.push(C32::new(rng(), rng()));
        }
        v
    }

    /// Alternating symbols, one per bit, at `sym_us` each.
    fn nrz(bits: &[u8], sym_us: u32) -> Vec<(bool, u32)> {
        bits.iter().map(|b| (*b != 0, sym_us)).collect()
    }

    fn detect(iq: &[C32], cfg: FskConfig) -> (Vec<Package>, FskDetector) {
        let mut d = FskDetector::new(RATE, cfg);
        let mut out = Vec::new();
        d.process(iq, &mut out);
        d.flush(&mut out);
        (out, d)
    }

    #[test]
    fn recovers_symbol_timings_from_a_keyed_carrier() {
        let syms = nrz(&[1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 1, 0, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0], 100);
        let iq = burst(&syms, 25_000.0, 0.0, 1.0, 0.02);
        let (pkgs, _) = detect(&iq, FskConfig::default());

        assert_eq!(pkgs.len(), 1, "expected one burst, got {}", pkgs.len());
        let p = &pkgs[0];
        // Runs of like symbols merge, so count transitions rather than bits.
        for (i, pulse) in p.pulses.iter().enumerate() {
            assert!(pulse.mark % 100 < 25 || pulse.mark % 100 > 75, "pulse {i}: {pulse:?}");
        }
        let marks = p.mark_histogram(30);
        assert!(
            marks.iter().any(|(c, _)| c.abs_diff(100) < 25),
            "no cluster at one symbol: {marks:?}"
        );
    }

    #[test]
    fn a_tuning_offset_does_not_move_the_threshold() {
        // The tones sit 40 kHz off centre, far more than the deviation. A
        // fixed threshold at zero hertz would call every symbol a mark.
        let syms = nrz(&[1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 1, 0, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0], 100);
        let clean = burst(&syms, 25_000.0, 0.0, 1.0, 0.02);
        let offset = burst(&syms, 25_000.0, 40_000.0, 1.0, 0.02);
        let (a, _) = detect(&clean, FskConfig::default());
        let (b, _) = detect(&offset, FskConfig::default());
        assert_eq!(a.len(), 1);
        assert_eq!(a[0].pulses.len(), b[0].pulses.len(), "offset changed the pulse train");
    }

    #[test]
    fn measures_the_deviation() {
        let syms = nrz(&[1, 0, 1, 0, 1, 0, 1, 0, 1, 1, 0, 0, 1, 0, 1, 0], 100);
        let iq = burst(&syms, 20_000.0, 5_000.0, 1.0, 0.02);
        let (_, d) = detect(&iq, FskConfig::default());
        let sep = d.separation_hz();
        assert!((sep - 40_000.0).abs() < 4_000.0, "separation came out as {sep} Hz");
    }

    #[test]
    fn an_unmodulated_carrier_produces_nothing() {
        let syms: Vec<(bool, u32)> = vec![(true, 20_000)];
        let iq = burst(&syms, 25_000.0, 0.0, 1.0, 0.02);
        let (pkgs, mut d) = detect(&iq, FskConfig::default());
        assert!(pkgs.is_empty(), "a plain carrier produced {} packages", pkgs.len());
        assert_eq!(d.take_stats().rejected_no_separation, 1, "rejection went unreported");
    }

    #[test]
    fn noise_alone_produces_nothing() {
        let iq = burst(&[], 0.0, 0.0, 0.0, 0.05);
        let (pkgs, _) = detect(&iq, FskConfig::default());
        assert!(pkgs.is_empty(), "noise produced {} packages", pkgs.len());
    }

    #[test]
    fn block_boundaries_do_not_change_the_result() {
        let syms = nrz(&[1, 0, 0, 1, 1, 0, 1, 0, 1, 1, 1, 0, 0, 1, 0, 1], 120);
        let iq = burst(&syms, 25_000.0, 3_000.0, 1.0, 0.02);
        let (whole, _) = detect(&iq, FskConfig::default());

        let mut split = FskDetector::new(RATE, FskConfig::default());
        let mut got = Vec::new();
        for c in iq.chunks(997) {
            split.process(c, &mut got);
        }
        split.flush(&mut got);
        assert_eq!(whole, got, "block splitting changed the pulse train");
    }

    #[test]
    fn a_glitch_shorter_than_a_symbol_is_absorbed() {
        // One 8 us excursion in the middle of a long mark: far too short to be
        // a symbol at 100 us, and it must not split the run in three.
        let mut syms = nrz(&[1, 0, 1, 0, 1, 0, 1, 0], 100);
        syms.push((true, 300));
        syms.push((false, 8));
        syms.push((true, 300));
        syms.extend(nrz(&[0, 1, 0, 1, 0, 1, 0, 1], 100));
        let iq = burst(&syms, 25_000.0, 0.0, 1.0, 0.02);
        let (pkgs, _) = detect(&iq, FskConfig::default());
        let long = pkgs[0].pulses.iter().filter(|p| p.mark > 500).count();
        assert_eq!(long, 1, "the 600 us mark was split: {:?}", pkgs[0].pulses);
    }
}
