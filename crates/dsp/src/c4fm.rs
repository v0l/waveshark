//! Four-level FSK: a frequency burst to symbols, with timing recovery.
//!
//! The two-level detector in [`crate::fsk`] hands its output to the same
//! mark/gap vocabulary the OOK slicers speak, because with two levels a run
//! length *is* the data. Four levels do not survive that trip. Two consecutive
//! symbols at the same level are one run of double length, and with four
//! levels there is no longer any rule that says which pair of levels a run
//! belongs to. So this front end skips the pulse layer entirely and produces
//! numbered symbols on a clock it recovers itself.
//!
//! That clock is the real work. A run-length front end never needs to know the
//! symbol rate: it measures whatever the transmitter did. Here the rate is
//! known from the protocol and the *phase* is not, and a symbol sampled
//! halfway between two others reads as a level that was never sent. The chain
//! is therefore:
//!
//! 1. gate the burst on envelope, as the other detectors do, and buffer the
//!    discriminator output for its duration
//! 2. remove the centre, which is the tuning error plus the transmitter's own
//! 3. smooth over a fraction of a symbol, to stop discriminator noise
//!    deciding a level
//! 4. search one symbol period for the phase that opens the eye widest
//! 5. track from there with a Gardner loop, which follows the clock error
//!    between the two crystals over a long frame
//! 6. fit four levels to the recovered symbols and number them
//!
//! Steps 4 and 5 are both here on purpose. The search alone cannot follow a
//! clock that drifts across a frame, and the loop alone starts wherever the
//! burst happened to open, which on a preamble of alternating outer levels is
//! a stable but wrong lock half the time.
//!
//! # What this is for
//!
//! FLEX and ERMES at their higher rates, wireless M-Bus mode N, and the
//! four-level voice protocols (DMR, P25 phase 1, NXDN, M17) which need a
//! vocoder after this but reach their framing through exactly this layer.
//! Level numbering is ascending frequency; see [`crate::fourlevel`] for why
//! the dibit mapping is left to the protocol.

use crate::fourlevel;
use crate::pulse::{dbfs, LevelGate};
use common::C32;

#[derive(Clone, Copy, Debug)]
pub struct C4fmConfig {
    /// Symbol rate in baud. Unlike the OOK and two-level FSK front ends, this
    /// one has to be told: a four-level eye cannot be opened without knowing
    /// where the symbol boundaries are meant to be.
    pub baud: f64,
    /// Carrier absent for longer than this ends the burst, in microseconds.
    pub reset_us: u32,
    /// Discard bursts with fewer symbols than this.
    pub min_symbols: usize,
    /// Envelope estimator time constant, in microseconds.
    pub tau_us: f32,
    /// Minimum envelope SNR before a burst is emitted.
    pub min_snr_db: f32,
    /// Hard floor on the carrier-detect threshold, as a multiple of the
    /// tracked noise mean. See [`crate::pulse::PulseConfig`].
    pub noise_threshold_ratio: f32,
    /// Minimum peak deviation, in hertz: the offset of the outer levels from
    /// the centre. Below this the burst is a carrier, an OOK packet or noise,
    /// all of which a four-level fit will happily describe if allowed to.
    pub min_deviation_hz: f32,
    /// Smallest share of symbols any one level must carry.
    ///
    /// This is the test that separates four-level FSK from two-level. A
    /// two-level burst fits this model with its inner levels empty and its
    /// step at a third of the true separation, and every symbol then reads as
    /// a valid outer level. Real four-level traffic uses all four within a
    /// frame, sync words included.
    pub min_level_share: f32,
    /// Largest tolerable RMS distance from the fitted levels, in steps. One
    /// step is half the way to a decision boundary.
    pub max_evm: f32,
    /// Gardner loop gain, as a fraction of a symbol period per unit error.
    pub loop_gain: f32,
    /// Longest burst held before it is forced out, in microseconds.
    pub max_burst_us: u32,
}

impl Default for C4fmConfig {
    fn default() -> Self {
        Self {
            // DMR, P25 phase 1, NXDN wide and M17 all key at 4800 baud.
            baud: 4_800.0,
            reset_us: 2_000,
            min_symbols: 24,
            tau_us: 500.0,
            min_snr_db: 6.0,
            noise_threshold_ratio: 3.5,
            // DMR's outer levels sit at 1944 Hz, FLEX's at 4800. Well under
            // either, and far above the spread a carrier alone produces.
            min_deviation_hz: 600.0,
            min_level_share: 0.02,
            max_evm: 0.45,
            loop_gain: 0.02,
            max_burst_us: 2_000_000,
        }
    }
}

/// One burst of recovered symbols.
#[derive(Clone, Debug, PartialEq)]
pub struct SymbolBurst {
    /// Level indices, 0 (lowest frequency) to 3, one per symbol.
    pub symbols: Vec<u8>,
    /// Estimated SNR of the burst, in dB.
    pub snr_db: f32,
    /// Received level in dB relative to a full scale sample at the detector's
    /// input. Same reference as [`common::pulse::Package::rssi_dbfs`].
    pub rssi_dbfs: f32,
    /// Peak deviation measured from the burst, in hertz: three steps.
    pub deviation_hz: f32,
    /// RMS distance from the fitted levels, in steps.
    pub evm: f32,
    /// Sample index where the burst started, for correlating with a waterfall.
    pub start_sample: u64,
    /// Where the burst was received, in Hz. Stamped by the owning node, which
    /// is where the stream's centre frequency is known.
    pub center_hz: u32,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct SymbolStats {
    /// Bursts emitted.
    pub accepted: u64,
    /// Bursts where the signal never rose far enough above noise.
    pub rejected_low_snr: u64,
    /// Bursts that yielded fewer than `min_symbols` symbols.
    pub rejected_too_few_symbols: u64,
    /// Bursts whose deviation was too small to be a keyed signal.
    pub rejected_no_deviation: u64,
    /// Bursts that fit four levels but only used some of them, which is what
    /// two-level FSK and an unmodulated carrier both look like here.
    pub rejected_levels_unused: u64,
    /// Bursts whose symbols sat too far from the levels fitted to them.
    pub rejected_high_evm: u64,
}

impl SymbolStats {
    pub fn rejected_total(&self) -> u64 {
        self.rejected_low_snr
            + self.rejected_too_few_symbols
            + self.rejected_no_deviation
            + self.rejected_levels_unused
            + self.rejected_high_evm
    }

    pub fn is_empty(&self) -> bool {
        *self == Self::default()
    }
}

/// Four-level FSK symbol detector.
///
/// Consumes complex baseband, like [`crate::fsk::FskDetector`] and for the
/// same reason: it needs the amplitude to know when a burst is happening and
/// the phase to know which level is being sent.
pub struct C4fmDetector {
    cfg: C4fmConfig,
    rate: f64,
    us_per_sample: f64,
    sps: f64,
    gate: LevelGate,
    prev: C32,
    /// Instantaneous frequency in hertz for the burst being collected. NaN
    /// marks a sample the carrier dropped out for: it still occupies time, but
    /// its frequency is noise and must not reach the level fit.
    burst: Vec<f32>,
    burst_start: u64,
    in_burst: bool,
    low_run: u64,
    sample: u64,
    /// Smoothed, centre-removed copy of the burst.
    shaped: Vec<f32>,
    /// Symbol samples recovered from `shaped`, before slicing.
    marks: Vec<f32>,
    scratch: Vec<f32>,
    last_deviation_hz: f32,
    last_evm: f32,
    stats: SymbolStats,
}

impl C4fmDetector {
    pub fn new(rate: f64, cfg: C4fmConfig) -> Self {
        Self {
            cfg,
            rate,
            us_per_sample: 1e6 / rate,
            sps: rate / cfg.baud,
            gate: LevelGate::new(
                rate,
                cfg.tau_us,
                // The envelope gate only decides where the burst starts and
                // stops, so it wants far more hysteresis than the symbol
                // recovery reading data off the same signal.
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
            shaped: Vec::new(),
            marks: Vec::new(),
            scratch: Vec::new(),
            last_deviation_hz: 0.0,
            last_evm: 0.0,
            stats: SymbolStats::default(),
        }
    }

    pub fn rate(&self) -> f64 {
        self.rate
    }

    /// Samples per symbol, which need not be a whole number: the recovery
    /// interpolates.
    pub fn samples_per_symbol(&self) -> f64 {
        self.sps
    }

    pub fn snr_db(&self) -> f32 {
        self.gate.snr_db()
    }

    pub fn noise_level(&self) -> f32 {
        self.gate.noise_level()
    }

    pub fn signal_level(&self) -> f32 {
        self.gate.signal_level()
    }

    /// Peak deviation of the most recent burst, in hertz. Worth showing for
    /// the same reason the two-level detector shows its separation: it names
    /// the protocol family before anything has decoded.
    pub fn deviation_hz(&self) -> f32 {
        self.last_deviation_hz
    }

    /// Eye closure of the most recent burst, in steps.
    pub fn evm(&self) -> f32 {
        self.last_evm
    }

    pub fn stats(&self) -> SymbolStats {
        self.stats
    }

    pub fn take_stats(&mut self) -> SymbolStats {
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
    pub fn process(&mut self, input: &[C32], out: &mut Vec<SymbolBurst>) {
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
    pub fn flush(&mut self, out: &mut Vec<SymbolBurst>) {
        if self.in_burst {
            self.finish(out);
        }
    }

    fn finish(&mut self, out: &mut Vec<SymbolBurst>) {
        self.in_burst = false;
        while self.burst.last().is_some_and(|v| v.is_nan()) {
            self.burst.pop();
        }
        let snr = self.snr_db();
        if snr < self.cfg.min_snr_db {
            self.stats.rejected_low_snr += 1;
            self.burst.clear();
            return;
        }

        self.shape();
        recover(&self.shaped, self.sps, self.cfg.loop_gain, &mut self.marks);
        if self.marks.len() < self.cfg.min_symbols {
            if !self.marks.is_empty() {
                self.stats.rejected_too_few_symbols += 1;
            }
            self.burst.clear();
            return;
        }

        let Some(fit) = fourlevel::levels(&mut self.scratch, &self.marks) else {
            self.burst.clear();
            return;
        };
        self.last_deviation_hz = 3.0 * fit.step;
        self.last_evm = fit.evm(&self.marks);

        if self.last_deviation_hz < self.cfg.min_deviation_hz {
            self.stats.rejected_no_deviation += 1;
            self.burst.clear();
            return;
        }
        if fit.occupancy(&self.marks).iter().any(|&s| s < self.cfg.min_level_share) {
            self.stats.rejected_levels_unused += 1;
            self.burst.clear();
            return;
        }
        if self.last_evm > self.cfg.max_evm {
            self.stats.rejected_high_evm += 1;
            self.burst.clear();
            return;
        }

        out.push(SymbolBurst {
            symbols: self.marks.iter().map(|&v| fit.index(v)).collect(),
            snr_db: snr,
            rssi_dbfs: dbfs(self.gate.signal_level()),
            deviation_hz: self.last_deviation_hz,
            evm: self.last_evm,
            start_sample: self.burst_start,
            center_hz: 0,
        });
        self.stats.accepted += 1;
        self.burst.clear();
    }

    /// Remove the centre frequency and smooth over part of a symbol.
    ///
    /// The centre is the median rather than the mean because a dropout, a
    /// discriminator spike or a long run at one outer level all move a mean
    /// and none of them move a median much. Held dropout samples do not decide
    /// a level either way, so they take the value before them.
    fn shape(&mut self) {
        self.scratch.clear();
        self.scratch.extend(self.burst.iter().copied().filter(|v| v.is_finite()));
        let center = if self.scratch.is_empty() {
            0.0
        } else {
            self.scratch.sort_unstable_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
            self.scratch[self.scratch.len() / 2]
        };

        // Half a symbol: long enough to bury the noise, short enough that a
        // lone symbol between two of its opposite still reaches its level.
        let width = ((self.sps * 0.5).round() as usize).max(1);
        self.shaped.clear();
        self.shaped.reserve(self.burst.len());
        let mut acc = 0.0f32;
        let mut held = 0.0f32;
        for &v in self.burst.iter() {
            held = if v.is_finite() { v - center } else { held };
            acc += held;
            self.shaped.push(acc);
        }
        // Turn the running sum into a boxcar in place, back to front so each
        // entry still sees the prefix sums it needs.
        for i in (0..self.shaped.len()).rev() {
            let prefix = if i >= width { self.shaped[i - width] } else { 0.0 };
            let n = if i >= width { width } else { i + 1 };
            self.shaped[i] = (self.shaped[i] - prefix) / n as f32;
        }
    }
}

/// Sample `x` at a fractional position by linear interpolation.
fn interp(x: &[f32], pos: f64) -> f32 {
    if x.is_empty() {
        return 0.0;
    }
    if pos <= 0.0 {
        return x[0];
    }
    let i = pos.floor() as usize;
    if i + 1 >= x.len() {
        return x[x.len() - 1];
    }
    let f = (pos - i as f64) as f32;
    x[i] * (1.0 - f) + x[i + 1] * f
}

/// Recover symbol samples from a shaped burst.
///
/// A coarse search over one symbol period picks the starting phase, then a
/// Gardner loop tracks the difference between the two crystals across the
/// frame. The search is graded by the eye opening at the candidate phase,
/// measured as the mean absolute distance from the levels fitted at that
/// phase: sampling halfway between symbols smears the levels together and
/// scores badly however the fit is scaled.
fn recover(shaped: &[f32], sps: f64, loop_gain: f32, out: &mut Vec<f32>) {
    out.clear();
    if shaped.len() < (sps * 4.0) as usize || sps < 2.0 {
        return;
    }

    let mut scratch = Vec::new();
    let mut candidate = Vec::new();
    let mut best = (f32::INFINITY, 0.0f64);
    let steps = 16;
    for k in 0..steps {
        let phase = sps * k as f64 / steps as f64;
        candidate.clear();
        let mut pos = phase;
        while pos < shaped.len() as f64 - 1.0 {
            candidate.push(interp(shaped, pos));
            pos += sps;
        }
        let Some(fit) = fourlevel::levels(&mut scratch, &candidate) else {
            continue;
        };
        let score = fit.evm(&candidate);
        if score < best.0 {
            best = (score, phase);
        }
    }
    if !best.0.is_finite() {
        return;
    }

    // Second order loop: the proportional term pulls the phase back, the
    // integral term learns the clock error so the phase stops having to be
    // pulled. Gains are small because a burst is thousands of symbols and the
    // error estimate is one noisy symbol wide.
    let kp = loop_gain as f64;
    let ki = kp * kp * 0.25;
    let rms2 = {
        let sum: f32 = shaped.iter().map(|v| v * v).sum();
        (sum / shaped.len() as f32).max(1e-12)
    };

    let mut period = sps;
    let mut pos = best.1;
    let mut prev = interp(shaped, pos);
    out.push(prev);
    pos += period;
    while pos < shaped.len() as f64 - 1.0 {
        let cur = interp(shaped, pos);
        let mid = interp(shaped, pos - period * 0.5);
        out.push(cur);
        // Gardner: with the strobe late, the midpoint sits after the
        // transition and shares its sign with the step just taken.
        let e = (((cur - prev) * mid) / rms2).clamp(-1.0, 1.0) as f64;
        prev = cur;
        period -= ki * e * sps;
        period = period.clamp(sps * 0.97, sps * 1.03);
        pos += period - kp * e * sps;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const RATE: f64 = 50_000.0;
    const BAUD: f64 = 4_800.0;
    /// DMR: outer levels at 1944 Hz, inner at 648 Hz.
    const STEP_HZ: f64 = 648.0;

    /// Build a four-level FSK burst from level indices, with silence either
    /// side. `offset_hz` is the tuning error every level rides on, and
    /// `clock_ppm` the difference between the transmitter's symbol clock and
    /// the one the detector is configured for.
    fn burst(levels: &[u8], step_hz: f64, offset_hz: f64, clock_ppm: f64, amp: f32, noise: f32) -> Vec<C32> {
        let mut seed = 12345u64;
        let mut rng = move || {
            seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
            ((seed >> 33) as f32 / (1u64 << 30) as f32 - 1.0) * noise
        };
        let sps = RATE / (BAUD * (1.0 + clock_ppm * 1e-6));
        let mut v = Vec::new();
        for _ in 0..(RATE * 0.005) as usize {
            v.push(C32::new(rng(), rng()));
        }
        let mut phase = 0.0f64;
        let mut t = 0.0f64;
        let total = levels.len() as f64 * sps;
        while t < total {
            let f = offset_hz + step_hz * super::fourlevel::IDEAL[levels[(t / sps) as usize] as usize] as f64;
            phase = (phase + std::f64::consts::TAU * f / RATE).rem_euclid(std::f64::consts::TAU);
            v.push(C32::new(amp * phase.cos() as f32 + rng(), amp * phase.sin() as f32 + rng()));
            t += 1.0;
        }
        for _ in 0..(RATE * 0.005) as usize {
            v.push(C32::new(rng(), rng()));
        }
        v
    }

    /// A pseudorandom level sequence with all four levels present.
    fn pattern(n: usize) -> Vec<u8> {
        let mut seed = 7u64;
        (0..n)
            .map(|_| {
                seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1);
                ((seed >> 40) & 3) as u8
            })
            .collect()
    }

    fn detect(iq: &[C32], cfg: C4fmConfig) -> (Vec<SymbolBurst>, C4fmDetector) {
        let mut d = C4fmDetector::new(RATE, cfg);
        let mut out = Vec::new();
        d.process(iq, &mut out);
        d.flush(&mut out);
        (out, d)
    }

    /// Fraction of the transmitted symbols that come back, at the best
    /// alignment. The recovered burst starts a symbol or two either side of
    /// the transmitted one, because the envelope gate opens on a ramp, so an
    /// exact comparison is not available.
    fn agreement(sent: &[u8], got: &[u8]) -> f32 {
        let mut best = 0.0f32;
        for shift in -8i32..=8 {
            let mut matched = 0usize;
            let mut total = 0usize;
            for (i, &s) in sent.iter().enumerate() {
                let j = i as i32 + shift;
                if j < 0 || j as usize >= got.len() {
                    continue;
                }
                total += 1;
                matched += usize::from(got[j as usize] == s);
            }
            if total > sent.len() / 2 {
                best = best.max(matched as f32 / total as f32);
            }
        }
        best
    }

    fn cfg() -> C4fmConfig {
        C4fmConfig { baud: BAUD, ..Default::default() }
    }

    #[test]
    fn recovers_the_symbols_that_were_sent() {
        let sent = pattern(200);
        let iq = burst(&sent, STEP_HZ, 0.0, 0.0, 1.0, 0.02);
        let (bursts, _) = detect(&iq, cfg());
        assert_eq!(bursts.len(), 1, "expected one burst, got {}", bursts.len());
        let got = &bursts[0].symbols;
        assert!(
            agreement(&sent, got) > 0.95,
            "only {:.0}% of the sequence came back: {:?}",
            agreement(&sent, got) * 100.0,
            &got[..20.min(got.len())]
        );
    }

    #[test]
    fn a_tuning_offset_does_not_move_the_levels() {
        // Two kilohertz off centre, more than the peak deviation. A fit
        // anchored at zero hertz would read every symbol as the top level.
        let sent = pattern(200);
        let iq = burst(&sent, STEP_HZ, 2_000.0, 0.0, 1.0, 0.02);
        let (bursts, _) = detect(&iq, cfg());
        assert_eq!(bursts.len(), 1);
        assert!(agreement(&sent, &bursts[0].symbols) > 0.95, "the offset moved the fit");
    }

    #[test]
    fn tracks_a_symbol_clock_that_is_off() {
        // 2000 ppm between the two crystals, which is two and a half symbols
        // over the 1200 here. Held at a fixed phase the frame is lost halfway
        // through, so the second half of this test is what proves the loop is
        // doing anything at all.
        let sent = pattern(1_200);
        let iq = burst(&sent, STEP_HZ, 0.0, 2_000.0, 1.0, 0.02);
        let (tracked, _) = detect(&iq, cfg());
        assert_eq!(tracked.len(), 1);
        let got = agreement(&sent, &tracked[0].symbols);
        assert!(got > 0.99, "the clock error was not tracked: {:.0}%", got * 100.0);

        let (fixed, _) = detect(&iq, C4fmConfig { loop_gain: 0.0, ..cfg() });
        let without = fixed.first().map(|b| agreement(&sent, &b.symbols)).unwrap_or(0.0);
        assert!(without < 0.8, "a fixed phase read {without:.0}%, so the loop is untested here");
    }

    #[test]
    fn works_when_the_symbol_period_is_not_a_whole_number_of_samples() {
        let d = C4fmDetector::new(RATE, cfg());
        assert!(d.samples_per_symbol().fract() > 0.1, "the test rate is not awkward enough");
        let sent = pattern(200);
        let iq = burst(&sent, STEP_HZ, 0.0, 0.0, 1.0, 0.02);
        let (bursts, _) = detect(&iq, cfg());
        assert_eq!(bursts.len(), 1);
        assert!(agreement(&sent, &bursts[0].symbols) > 0.95);
    }

    #[test]
    fn measures_the_deviation() {
        let sent = pattern(200);
        let iq = burst(&sent, STEP_HZ, 500.0, 0.0, 1.0, 0.02);
        let (_, d) = detect(&iq, cfg());
        let dev = d.deviation_hz();
        assert!((dev - 1_944.0).abs() < 200.0, "peak deviation came out as {dev} Hz");
    }

    #[test]
    fn a_two_level_burst_is_refused() {
        // Only the outer levels, which is what the two-level detector in
        // crate::fsk exists to read. It fits this model perfectly with the
        // inner levels empty, and must not be claimed as four-level traffic.
        let sent: Vec<u8> = (0..200).map(|i| ((i * 7 + i / 3) % 2 * 3) as u8).collect();
        let iq = burst(&sent, STEP_HZ, 0.0, 0.0, 1.0, 0.02);
        let (bursts, mut d) = detect(&iq, cfg());
        assert!(bursts.is_empty(), "a two-level burst produced {} bursts", bursts.len());
        assert_eq!(d.take_stats().rejected_levels_unused, 1, "rejection went unreported");
    }

    #[test]
    fn an_unmodulated_carrier_produces_nothing() {
        let sent = vec![2u8; 400];
        let iq = burst(&sent, STEP_HZ, 0.0, 0.0, 1.0, 0.02);
        let (bursts, mut d) = detect(&iq, cfg());
        assert!(bursts.is_empty(), "a plain carrier produced {} bursts", bursts.len());
        let stats = d.take_stats();
        assert!(stats.rejected_total() > 0 && stats.accepted == 0, "{stats:?}");
    }

    #[test]
    fn noise_alone_produces_nothing() {
        let iq = burst(&[], STEP_HZ, 0.0, 0.0, 0.0, 0.05);
        let (bursts, _) = detect(&iq, cfg());
        assert!(bursts.is_empty(), "noise produced {} bursts", bursts.len());
    }

    #[test]
    fn block_boundaries_do_not_change_the_result() {
        let sent = pattern(300);
        let iq = burst(&sent, STEP_HZ, 300.0, 0.0, 1.0, 0.02);
        let (whole, _) = detect(&iq, cfg());

        let mut split = C4fmDetector::new(RATE, cfg());
        let mut got = Vec::new();
        for c in iq.chunks(997) {
            split.process(c, &mut got);
        }
        split.flush(&mut got);
        assert_eq!(whole, got, "block splitting changed the symbols");
    }

    #[test]
    fn a_noisy_burst_is_reported_with_its_eye_closure() {
        let sent = pattern(400);
        let clean = burst(&sent, STEP_HZ, 0.0, 0.0, 1.0, 0.01);
        let noisy = burst(&sent, STEP_HZ, 0.0, 0.0, 1.0, 0.25);
        let (a, _) = detect(&clean, cfg());
        let (b, _) = detect(&noisy, cfg());
        assert_eq!(a.len(), 1);
        if let Some(b) = b.first() {
            assert!(b.evm > a[0].evm, "the noisy burst read as clean: {} vs {}", b.evm, a[0].evm);
        }
    }
}
