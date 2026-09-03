//! Pulse extraction: turning an envelope into mark/gap timings.
//!
//! This is the shared front end that makes broad protocol support tractable.
//! rtl_433 supports hundreds of devices without hundreds of DSP chains,
//! because almost all of them are on-off keyed or two-level FSK, and both
//! reduce to the same thing: a list of how long the carrier was present and
//! how long it was absent. Everything protocol-specific happens afterwards, on
//! integers, at negligible cost.
//!
//! So the expensive work (magnitude, thresholding, timing) is done once per
//! channel, and adding the two hundredth protocol costs a table entry rather
//! than another filter chain.
//!
//! # Thresholding
//!
//! The threshold sits midway between a tracked noise estimate and a tracked
//! signal estimate, both updated only while the detector believes it is in the
//! corresponding state. A fixed threshold cannot work: ISM receivers see
//! signals ranging from a meter away to the edge of sensitivity, and the AGC
//! moves the floor underneath everything.

pub use common::pulse::{Package, Pulse};

/// Amplitude to dB relative to a full scale sample.
///
/// Amplitude, not power, so the reference is 1.0 rather than 0.5, and a signal
/// filling the ADC reads as 0 dBFS.
pub(crate) fn dbfs(amplitude: f32) -> f32 {
    20.0 * amplitude.max(1e-9).log10()
}

#[derive(Clone, Copy, Debug)]
pub struct PulseConfig {
    /// Gap longer than this ends the package, in microseconds.
    pub reset_us: u32,
    /// Ignore marks shorter than this: impulse noise, not signal.
    pub min_mark_us: u32,
    /// Discard packages with fewer pulses than this.
    pub min_pulses: usize,
    /// Hysteresis as a fraction of the gap between the noise and signal
    /// estimates. Without it a signal sitting near the threshold shreds into
    /// hundreds of spurious pulses.
    pub hysteresis: f32,
    /// Estimator time constant in microseconds.
    pub tau_us: f32,
    /// Minimum ratio between signal and noise estimates before pulses are
    /// emitted at all, guarding against a package made entirely of noise.
    pub min_snr_db: f32,
    /// Hard floor on the detection threshold, as a multiple of the tracked
    /// noise mean.
    ///
    /// Without this the threshold is simply the midpoint between the noise and
    /// signal estimates, which in silence collapses to about 1.5 times the
    /// noise mean. The envelope of bandlimited Gaussian noise is Rayleigh
    /// distributed and crosses 1.5 times its mean constantly, so the detector
    /// spends idle time manufacturing pulses out of noise.
    ///
    /// The effect is much worse in a narrow channel, which is what makes this
    /// a channelizer problem rather than a theoretical one: noise in a 31 kHz
    /// channel decorrelates eight times more slowly than in a 250 kHz span, so
    /// each excursion lasts eight times longer and comfortably survives
    /// `min_mark_us`. A wideband detector can get away with the sloppy
    /// threshold; a per-channel one cannot.
    ///
    /// A Rayleigh envelope exceeds 3.5 times its mean with probability about
    /// 2e-6, so that is the default.
    pub noise_threshold_ratio: f32,
    /// Take the floor under the threshold from the measured noise peak rather
    /// than from `noise_threshold_ratio` times the mean. See `QuietPeak`.
    ///
    /// Off by default, and the reason is measured rather than cautious: on
    /// rtl_433's corpus it moves the median noise floor from 6 dB to 3 dB and
    /// takes the captures that still decode at 0 dB from 8 to 25, and it costs
    /// the checksum-free protocols dearly, from 3 unverified claims across the
    /// corpus to 27, with one that passes a CRC. It is a sensitivity control,
    /// not an improvement, and which side of that trade is right depends on
    /// the band.
    pub measured_noise_floor: bool,
    /// How far above the measured noise peak the floor sits. Larger is more
    /// selective: 1.8 keeps most of the sensitivity at a third of the junk.
    pub noise_floor_margin: f32,
    /// Rejoin marks split by a dropout too short to be a symbol.
    ///
    /// See `Package::merge_dropouts`. Off gives the behaviour the corpus
    /// numbers in `corpus_metrics` were first measured with, so the two can be
    /// compared without rebuilding the harness.
    pub merge_dropouts: bool,
}

/// Tracks the noise and signal levels of an envelope and says, per sample,
/// whether a carrier is present.
///
/// Shared by the OOK detector, where the answer *is* the data, and the FSK
/// detector, where it only gates which samples are worth measuring the
/// frequency of. Both need the same adaptive threshold, and an ISM receiver
/// that gets it wrong either hears nothing or hears noise all day.
#[derive(Clone, Debug)]
pub struct LevelGate {
    alpha: f32,
    hysteresis: f32,
    min_ratio: f32,
    noise_threshold_ratio: f32,
    noise_floor_margin: f32,
    noise: f32,
    signal: f32,
    high: bool,
    seeded: bool,
    /// Samples summed towards the seed, and how many.
    warm_sum: f32,
    warm_n: u32,
    /// Measured peak of the quietest recent stretch, or `None` while the
    /// window is still filling. See [`QuietPeak`].
    quiet: Option<QuietPeak>,
    /// Whether the stream is being watched for having begun inside a
    /// transmission. See [`HotCheck`].
    hot: Option<HotCheck>,
    /// The stream began inside a transmission and the gate holds open.
    hot_stream: bool,
}

/// Watches a freshly seeded gate for a stream that began inside a
/// transmission.
///
/// The seed is the mean of the first stretch of the stream, taken to be
/// noise. A stream cut out around a source the detector found usually starts
/// with a lead-in of noise and that is right; one cut out around a carrier
/// that was already on when the receiver tuned starts with the carrier, the
/// seed is the signal, the threshold sits above it, and the gate never opens
/// on a signal thirty decibels clear of the floor. So for a stream the
/// detector says holds a strong source, the first fifth of a second is
/// watched: if it is steady, and nothing has crossed the threshold, the
/// stream is signal from its first sample.
///
/// From then on the gate holds open. Opening it and letting it follow the
/// envelope was tried and does not work for what such a stream carries: a
/// phase-keyed carrier's envelope dips at every symbol, the gate dropped on
/// a dip, learned the carrier as noise during it, and never opened again.
/// The detector that cut the stream out is the gate here, and it closes
/// the stream when the transmission stops.
#[derive(Clone, Debug)]
struct HotCheck {
    /// The source's level over the noise, as a ratio of amplitudes.
    ratio: f32,
    chunk: usize,
    sum: f64,
    filled: usize,
    means: Vec<f32>,
    want: usize,
}

/// The loudest sample in the quietest recent stretch of the envelope.
///
/// The floor under the detection threshold has to sit above what noise alone
/// does, and there are two ways to get there. Ours was a Rayleigh
/// calculation: the envelope of bandlimited Gaussian noise exceeds 3.5 times
/// its mean about twice in a million samples, so put the floor there. That is
/// right about Gaussian noise and says nothing about the noise a receiver
/// actually has, which is not Gaussian near a switching supply, a spurious
/// carrier or a neighbouring channel's skirt.
///
/// This measures it instead, the way Universal Radio Hacker does: cut the
/// stream into chunks, take the mean of each, and read the peak sample out of
/// the chunks whose means sit within a tenth of the quietest. Those chunks
/// are the ones with nothing in them, so their peak is what noise alone
/// reaches.
#[derive(Clone, Debug)]
pub struct QuietPeak {
    chunk: usize,
    /// Sum and peak of the chunk being accumulated.
    sum: f64,
    peak: f32,
    filled: usize,
    /// Mean and peak of each chunk in the window, oldest first.
    window: std::collections::VecDeque<(f32, f32)>,
    depth: usize,
    estimate: Option<f32>,
}

impl QuietPeak {
    /// `window_us` of history in chunks of `chunk_us`.
    fn new(rate: f64, chunk_us: f32, window_us: f32) -> Self {
        let chunk = ((chunk_us as f64 * rate / 1e6) as usize).max(16);
        let depth = ((window_us / chunk_us).round() as usize).max(4);
        Self {
            chunk,
            sum: 0.0,
            peak: 0.0,
            filled: 0,
            window: std::collections::VecDeque::with_capacity(depth),
            depth,
            estimate: None,
        }
    }

    fn reset(&mut self) {
        self.sum = 0.0;
        self.peak = 0.0;
        self.filled = 0;
        self.window.clear();
        self.estimate = None;
    }

    /// Feed one envelope sample, returning the current estimate if there is
    /// one yet.
    fn update(&mut self, v: f32) -> Option<f32> {
        self.sum += v as f64;
        self.peak = self.peak.max(v);
        self.filled += 1;
        if self.filled < self.chunk {
            return self.estimate;
        }
        let mean = (self.sum / self.filled as f64) as f32;
        if self.window.len() == self.depth {
            self.window.pop_front();
        }
        self.window.push_back((mean, self.peak));
        self.sum = 0.0;
        self.peak = 0.0;
        self.filled = 0;

        // Only once there is enough history for a quiet stretch to be in it.
        if self.window.len() >= self.depth / 2 {
            let quietest = self.window.iter().map(|(m, _)| *m).fold(f32::MAX, f32::min);
            let peak = self
                .window
                .iter()
                .filter(|(m, _)| *m <= quietest * 1.1)
                .map(|(_, p)| *p)
                .fold(0.0f32, f32::max);
            if peak > 0.0 {
                self.estimate = Some(peak);
            }
        }
        self.estimate
    }
}

impl LevelGate {
    pub fn new(rate: f64, tau_us: f32, hysteresis: f32, min_snr_db: f32, noise_ratio: f32) -> Self {
        let tau_samples = (tau_us * (rate as f32) / 1e6).max(1.0);
        Self {
            alpha: 1.0 / tau_samples,
            hysteresis,
            min_ratio: 10f32.powf(min_snr_db / 20.0),
            noise_threshold_ratio: noise_ratio,
            noise_floor_margin: 1.1,
            noise: 0.0,
            signal: 0.0,
            high: false,
            seeded: false,
            warm_sum: 0.0,
            warm_n: 0,
            quiet: None,
            hot: None,
            hot_stream: false,
        }
    }

    /// Whether the stream was found to have begun inside a transmission,
    /// and is being passed whole.
    pub fn is_hot_stream(&self) -> bool {
        self.hot_stream
    }

    /// Tell the gate the stream holds a source `snr_db` over the noise, so a
    /// stream that begins inside that source can be recognised as such
    /// rather than read as noise. See [`HotCheck`]. Ignored below 10 dB,
    /// where the seed and the signal cannot be told apart by steadiness.
    pub fn expect_signal(&mut self, rate: f64, snr_db: f32) {
        if snr_db < 10.0 {
            return;
        }
        self.hot = Some(HotCheck {
            ratio: 10f32.powf(snr_db / 20.0),
            chunk: ((rate / 1_000.0) as usize).max(16),
            sum: 0.0,
            filled: 0,
            means: Vec::new(),
            want: 200,
        });
    }

    /// Take the floor under the threshold from the measured noise peak rather
    /// than from the Rayleigh assumption. See [`QuietPeak`].
    pub fn with_measured_floor(mut self, rate: f64, margin: f32) -> Self {
        self.noise_floor_margin = margin.max(1.0);
        // A millisecond of envelope per chunk, a fifth of a second of window.
        // The window has to hold silence either side of a transmission, and
        // ISM bursts run to about a tenth of that.
        self.quiet = Some(QuietPeak::new(rate, 1_000.0, 200_000.0));
        self
    }

    pub fn noise_level(&self) -> f32 {
        self.noise
    }

    pub fn signal_level(&self) -> f32 {
        self.signal
    }

    pub fn snr_db(&self) -> f32 {
        20.0 * (self.signal.max(1e-20) / self.noise.max(1e-20)).log10()
    }

    pub fn is_high(&self) -> bool {
        self.high
    }

    pub fn reset(&mut self) {
        self.high = false;
        self.seeded = false;
        self.warm_sum = 0.0;
        self.warm_n = 0;
        if let Some(q) = self.quiet.as_mut() {
            q.reset();
        }
        if let Some(h) = self.hot.as_mut() {
            h.sum = 0.0;
            h.filled = 0;
            h.means.clear();
        }
        self.hot_stream = false;
    }

    /// Feed one envelope sample and return whether it is above threshold.
    pub fn update(&mut self, v: f32) -> bool {
        self.update_learning(v, true)
    }

    /// As [`Self::update`], but `learn_noise` can hold the noise estimate
    /// still.
    ///
    /// A detector that keys on amplitude must freeze it for the duration of a
    /// burst. The low level of a shallow ASK signal is below threshold, so an
    /// unfrozen estimate learns *it* as the noise floor, the floor under the
    /// threshold then rises above the high level, and the gate declares the
    /// whole packet absent.
    pub fn update_learning(&mut self, v: f32, learn_noise: bool) -> bool {
        // Seeded from the first time constant's worth of samples, not from
        // the first sample alone. One sample of noise is one Rayleigh draw,
        // and a tenth of those sit under a third of the mean: a threshold
        // built on one of them is cleared by half the noise that follows,
        // and the gate opens on nothing before the burst has arrived. That
        // was invisible while every stream ran from the radio's start, and
        // it was the first thing a stream cut out around a burst hit.
        if !self.seeded {
            let n = (1.0 / self.alpha).round().max(1.0) as u32;
            self.warm_sum += v;
            self.warm_n += 1;
            if self.warm_n < n {
                return false;
            }
            self.noise = self.warm_sum / self.warm_n as f32;
            self.signal = self.noise * 4.0;
            self.seeded = true;
        }
        if self.hot_stream {
            self.signal += self.alpha * (v - self.signal);
            self.high = true;
            return true;
        }

        // The floor under the threshold: measured if there is a measurement,
        // else the Rayleigh figure.
        let measured = self.quiet.as_mut().and_then(|q| q.update(v));
        let floor = match measured {
            // A little above the peak the noise reached, since a threshold
            // exactly at it is crossed by the next sample as loud as the last
            // one.
            Some(peak) => peak * self.noise_floor_margin,
            None => self.noise * self.noise_threshold_ratio,
        };
        // Midpoint between the two estimates, but never below the floor.
        let mid = (0.5 * (self.noise + self.signal)).max(floor);
        let hyst = self.hysteresis * (self.signal - mid).max(0.0);
        let thresh = if self.high { mid - hyst } else { mid + hyst };
        let now_high = v > thresh;

        // Track each level only while in that state, so a long transmission
        // cannot pull the noise estimate up after itself.
        if now_high {
            self.signal += self.alpha * (v - self.signal);
            // Without this clamp the detector destroys itself. A single noise
            // spike crosses the threshold, the signal estimate then adapts
            // *down* towards that noise, which lowers the threshold, which
            // admits more noise. The estimator converges on the noise floor
            // and emits hundreds of spurious micro-pulses. Holding the signal
            // estimate at least `min_snr_db` above the noise breaks the loop.
            let floor = self.noise * self.min_ratio;
            if self.signal < floor {
                self.signal = floor;
            }
        } else if learn_noise {
            self.noise += self.alpha * (v - self.noise);
            // The signal estimate only tracks while the gate is high, so
            // one that is too high can never be corrected by the signal
            // itself: the threshold sits above everything and nothing is
            // ever high again. A quarter second of full-scale constant at
            // the start of a capture, which is how a tuner settles in some
            // recordings, did exactly that. So it leaks, slowly, towards the
            // least level the gate would accept as signal: sixteen time
            // constants, long against any gap inside a packet and short
            // against a band gone quiet.
            let rest = self.noise * self.min_ratio;
            if self.signal > rest {
                self.signal -= self.alpha * 0.0625 * (self.signal - rest);
            }
        }

        // A stream that began inside its transmission: steady for the whole
        // watch and never once over the threshold. The seed was the signal.
        if let Some(h) = self.hot.as_mut() {
            if now_high {
                self.hot = None;
            } else {
                h.sum += v as f64;
                h.filled += 1;
                if h.filled >= h.chunk {
                    h.means.push((h.sum / h.filled as f64) as f32);
                    h.sum = 0.0;
                    h.filled = 0;
                    if h.means.len() >= h.want {
                        let lo = h.means.iter().copied().fold(f32::MAX, f32::min);
                        let hi = h.means.iter().copied().fold(0.0f32, f32::max);
                        let mean = h.means.iter().sum::<f32>() / h.means.len() as f32;
                        let ratio = h.ratio;
                        self.hot = None;
                        if hi < lo * 2.0 {
                            self.signal = mean;
                            self.noise = mean / ratio;
                            self.hot_stream = true;
                            self.high = true;
                            return true;
                        }
                    }
                }
            }
        }

        self.high = now_high;
        now_high
    }
}

impl Default for PulseConfig {
    fn default() -> Self {
        Self {
            // rtl_433's default reset is 100 klots at 250 kHz, about 400 us.
            // A little longer is safer: some protocols use long inter-symbol
            // gaps and would otherwise be split mid-packet.
            reset_us: 4_000,
            // Below about 100 us is noise for practically every ISM protocol:
            // the fastest common OOK symbol is around 100 us and anything
            // briefer is a threshold crossing, not a transmission.
            min_mark_us: 100,
            min_pulses: 4,
            hysteresis: 0.2,
            tau_us: 500.0,
            min_snr_db: 6.0,
            noise_threshold_ratio: 3.5,
            measured_noise_floor: false,
            noise_floor_margin: 1.8,
            merge_dropouts: true,
        }
    }
}

/// On-off-keying pulse detector.
///
/// Consumes a real envelope (magnitude, not magnitude squared) and produces
/// complete packages.
pub struct OokDetector {
    cfg: PulseConfig,
    rate: f64,
    /// Microseconds per sample.
    us_per_sample: f64,
    gate: LevelGate,
    /// Currently above threshold.
    high: bool,
    run: u64,
    current: Package,
    /// Mark awaiting its gap. Held back because a mark's gap is only known
    /// once the next valid mark begins.
    pending_mark: u32,
    /// Gap accumulated since `pending_mark`, including any sub-threshold
    /// crossings folded into it.
    gap_accum: u32,
    sample: u64,
    stats: PulseStats,
}

/// Why bursts were discarded.
///
/// A detector that silently drops everything is indistinguishable from a dead
/// antenna. These counters are what turn "nothing decoded" into "14 bursts
/// seen, all too short", which points straight at the parameter to change.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct PulseStats {
    /// Bursts that ended with fewer than `min_pulses` pulses.
    pub rejected_too_few_pulses: u64,
    /// Bursts rejected because the signal never rose far enough above noise.
    pub rejected_low_snr: u64,
    /// Marks discarded for being shorter than `min_mark_us`.
    pub rejected_short_marks: u64,
    /// Marks rejoined across a dropout too short to be a symbol.
    pub rejoined_marks: u64,
    /// FSK only: bursts whose two frequency levels were too close together to
    /// be a keyed signal, which is what a plain carrier or an OOK burst looks
    /// like to a discriminator.
    pub rejected_no_separation: u64,
    /// Bursts emitted.
    pub accepted: u64,
}

impl PulseStats {
    pub fn rejected_total(&self) -> u64 {
        self.rejected_too_few_pulses + self.rejected_low_snr + self.rejected_no_separation
    }

    pub fn is_empty(&self) -> bool {
        *self == Self::default()
    }
}

impl OokDetector {
    pub fn new(rate: f64, cfg: PulseConfig) -> Self {
        let us_per_sample = 1e6 / rate;
        Self {
            cfg,
            rate,
            us_per_sample,
            gate: {
                let g = LevelGate::new(
                    rate,
                    cfg.tau_us,
                    cfg.hysteresis,
                    cfg.min_snr_db,
                    cfg.noise_threshold_ratio,
                );
                if cfg.measured_noise_floor {
                    g.with_measured_floor(rate, cfg.noise_floor_margin)
                } else {
                    g
                }
            },
            high: false,
            run: 0,
            current: Package::default(),
            pending_mark: 0,
            gap_accum: 0,
            sample: 0,
            stats: PulseStats::default(),
        }
    }

    /// Rejection counters since the last call, then clear.
    pub fn take_stats(&mut self) -> PulseStats {
        std::mem::take(&mut self.stats)
    }

    pub fn stats(&self) -> PulseStats {
        self.stats
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

    pub fn reset(&mut self) {
        self.high = false;
        self.run = 0;
        self.current = Package::default();
        self.pending_mark = 0;
        self.gap_accum = 0;
        self.gate.reset();
    }

    fn us(&self, samples: u64) -> u32 {
        (samples as f64 * self.us_per_sample).round() as u32
    }

    /// Feed an envelope block, appending any completed packages to `out`.
    pub fn process(&mut self, env: &[f32], out: &mut Vec<Package>) {
        let reset_samples = (self.cfg.reset_us as f64 / self.us_per_sample) as u64;

        for &v in env {
            let now_high = self.gate.update(v);

            if now_high != self.high {
                let dur = self.us(self.run);
                if self.high {
                    // A high run ended: a candidate mark.
                    if dur >= self.cfg.min_mark_us {
                        // Emit the *previous* pulse, whose gap is now complete.
                        if self.pending_mark > 0 {
                            self.current.pulses.push(Pulse {
                                mark: self.pending_mark,
                                gap: self.gap_accum,
                            });
                        } else if self.current.pulses.is_empty() {
                            self.current.start_sample =
                                self.sample.saturating_sub(self.run);
                        }
                        self.pending_mark = dur;
                        self.gap_accum = 0;
                    } else {
                        // Too short to be a symbol. It is a threshold
                        // crossing inside a gap, so it must be folded *into*
                        // that gap. Simply discarding it would split one gap
                        // into two shorter ones and corrupt every timing that
                        // follows, which is far worse than the noise itself.
                        self.gap_accum += dur;
                        self.stats.rejected_short_marks += 1;
                    }
                } else {
                    self.gap_accum += dur;
                }
                self.high = now_high;
                self.run = 0;
            }

            self.run += 1;
            self.sample += 1;

            // A long silence terminates the package.
            if !self.high && self.run > reset_samples {
                self.close(out);
                self.run = 0;
            }
        }
    }

    /// Emit the package still being collected.
    ///
    /// Needed wherever the silence that would have ended it never arrives: the
    /// end of a file, and a burst handed over on its own by
    /// [`crate::route::BurstRouter`], which trims the silence off before
    /// passing it on precisely because that silence is what makes a
    /// measurement of the burst read as a measurement of an empty channel.
    pub fn flush(&mut self, out: &mut Vec<Package>) {
        self.close(out);
    }

    fn close(&mut self, out: &mut Vec<Package>) {
        if self.pending_mark >= self.cfg.min_mark_us {
            self.current
                .pulses
                .push(Pulse { mark: self.pending_mark, gap: self.cfg.reset_us });
        }
        self.pending_mark = 0;
        self.gap_accum = 0;
        if self.current.pulses.len() >= self.cfg.min_pulses {
            let snr = self.snr_db();
            if snr >= self.cfg.min_snr_db {
                let mut p = std::mem::take(&mut self.current);
                p.snr_db = snr;
                p.rssi_dbfs = dbfs(self.gate.signal_level());
                if self.cfg.merge_dropouts {
                    self.stats.rejoined_marks += p.merge_dropouts() as u64;
                }
                out.push(p);
                self.stats.accepted += 1;
            } else {
                self.stats.rejected_low_snr += 1;
            }
        } else if !self.current.pulses.is_empty() {
            self.stats.rejected_too_few_pulses += 1;
        }
        self.current.pulses.clear();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_stream_that_begins_inside_a_transmission_is_passed_whole() {
        // A carrier that was on when the receiver tuned starts the stream,
        // so the seed the gate takes for noise is the signal. Told how far
        // the detector found it over the floor, the gate notices the stream
        // is steady and never once over its threshold, and holds open.
        let rate = 120_000.0;
        let mut hot = LevelGate::new(rate, 500.0, 0.3, 6.0, 3.5);
        hot.expect_signal(rate, 30.0);
        let mut cold = LevelGate::new(rate, 500.0, 0.3, 6.0, 3.5);
        let carrier = |i: usize| 0.5 + 0.05 * ((i as f32) * 0.7).sin();
        let mut high_hot = 0;
        let mut high_cold = 0;
        for i in 0..(rate as usize / 2) {
            high_hot += usize::from(hot.update(carrier(i)));
            high_cold += usize::from(cold.update(carrier(i)));
        }
        assert!(hot.is_hot_stream(), "the steady stream was not recognised");
        assert!(high_hot > rate as usize / 4, "held open for {high_hot} samples of half a second");
        assert!(!cold.is_hot_stream());
        assert_eq!(high_cold, 0, "without the hint the gate has no reason to open");

        // A stream that starts in noise and then carries a burst is what
        // the seed is for, and the watch must not touch it.
        let mut gate = LevelGate::new(rate, 500.0, 0.3, 6.0, 3.5);
        gate.expect_signal(rate, 30.0);
        let mut seed = 12345u32;
        let mut noise = move || {
            seed = seed.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            0.01 + 0.005 * (seed >> 16) as f32 / 65_536.0
        };
        for _ in 0..(rate as usize / 10) {
            gate.update(noise());
        }
        let mut opened = 0;
        for _ in 0..(rate as usize / 100) {
            opened += usize::from(gate.update(0.5));
        }
        assert!(opened > 0, "the burst did not open the gate");
        assert!(!gate.is_hot_stream(), "a burst after a lead-in is not a hot stream");
    }

    const RATE: f64 = 250_000.0;

    /// Build an envelope from a mark/gap list in microseconds.
    fn envelope(pulses: &[(u32, u32)], amp: f32, noise_amp: f32) -> Vec<f32> {
        let sp = |us: u32| (us as f64 * RATE / 1e6).round() as usize;
        let mut seed = 42u64;
        let mut rng = move || {
            seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
            ((seed >> 33) as f32 / (1u64 << 31) as f32) * noise_amp
        };
        let mut v = Vec::new();
        v.extend((0..sp(20_000)).map(|_| rng()));
        for (m, g) in pulses {
            v.extend((0..sp(*m)).map(|_| amp + rng()));
            v.extend((0..sp(*g)).map(|_| rng()));
        }
        v.extend((0..sp(20_000)).map(|_| rng()));
        v
    }

    fn approx(a: u32, b: u32, tol: u32) -> bool {
        a.abs_diff(b) <= tol
    }

    #[test]
    fn recovers_pwm_timings() {
        // A typical PWM remote: 500 us short, 1000 us long, 500 us gaps.
        let want = [
            (500u32, 500u32),
            (1000, 500),
            (500, 500),
            (1000, 500),
            (1000, 500),
            (500, 500),
        ];
        let env = envelope(&want, 1.0, 0.02);
        let mut d = OokDetector::new(RATE, PulseConfig::default());
        let mut out = Vec::new();
        d.process(&env, &mut out);

        assert_eq!(out.len(), 1, "expected one package, got {}", out.len());
        let p = &out[0];
        assert_eq!(p.pulses.len(), want.len(), "pulses: {:?}", p.pulses);
        for (got, exp) in p.pulses.iter().zip(&want) {
            assert!(approx(got.mark, exp.0, 30), "mark {} vs {}", got.mark, exp.0);
        }
        // Every gap but the last, which is the terminating timeout.
        for (got, exp) in p.pulses[..want.len() - 1].iter().zip(&want) {
            assert!(approx(got.gap, exp.1, 30), "gap {} vs {}", got.gap, exp.1);
        }
    }

    #[test]
    fn histogram_separates_short_from_long() {
        let want = [(500u32, 500u32), (1000, 500), (500, 500), (1000, 500), (500, 500)];
        let env = envelope(&want, 1.0, 0.02);
        let mut d = OokDetector::new(RATE, PulseConfig::default());
        let mut out = Vec::new();
        d.process(&env, &mut out);

        let h = out[0].mark_histogram(100);
        assert_eq!(h.len(), 2, "expected two clusters, got {h:?}");
        assert!(approx(h[0].0, 500, 40) && h[0].1 == 3, "{h:?}");
        assert!(approx(h[1].0, 1000, 40) && h[1].1 == 2, "{h:?}");
    }

    #[test]
    fn separates_two_bursts_by_the_reset_gap() {
        let sp = |us: u32| (us as f64 * RATE / 1e6).round() as usize;
        let burst = [(500u32, 500u32), (1000, 500), (500, 500), (1000, 500)];
        let mut env = envelope(&burst, 1.0, 0.02);
        env.extend((0..sp(10_000)).map(|_| 0.01));
        env.extend(envelope(&burst, 1.0, 0.02));

        let mut d = OokDetector::new(RATE, PulseConfig::default());
        let mut out = Vec::new();
        d.process(&env, &mut out);
        assert_eq!(out.len(), 2, "expected two packages, got {}", out.len());
    }

    #[test]
    fn rejects_pure_noise() {
        let env: Vec<f32> = {
            let mut seed = 7u64;
            (0..250_000)
                .map(|_| {
                    seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
                    (seed >> 33) as f32 / (1u64 << 31) as f32 * 0.05
                })
                .collect()
        };
        let mut d = OokDetector::new(RATE, PulseConfig::default());
        let mut out = Vec::new();
        d.process(&env, &mut out);
        assert!(out.is_empty(), "noise produced {} packages: {:?}", out.len(), out);
    }

    #[test]
    fn hysteresis_prevents_shredding_a_weak_pulse() {
        // A long mark with amplitude wobbling around the threshold. Without
        // hysteresis this fragments into a great many pulses.
        let sp = |us: u32| (us as f64 * RATE / 1e6).round() as usize;
        let mut env: Vec<f32> = vec![0.01; sp(20_000)];
        for i in 0..sp(2_000) {
            env.push(if i % 3 == 0 { 0.45 } else { 0.55 });
        }
        env.extend(vec![0.01; sp(20_000)]);

        let mut d = OokDetector::new(RATE, PulseConfig::default());
        let mut out = Vec::new();
        d.process(&env, &mut out);
        let n = out.first().map(|p| p.pulses.len()).unwrap_or(0);
        assert!(n <= 2, "wobbling pulse shredded into {n} pulses");
    }

    #[test]
    fn block_boundaries_do_not_split_pulses() {
        let want = [(500u32, 500u32), (1000, 500), (500, 500), (1000, 500), (500, 500)];
        let env = envelope(&want, 1.0, 0.02);

        let mut whole = OokDetector::new(RATE, PulseConfig::default());
        let mut a = Vec::new();
        whole.process(&env, &mut a);

        let mut split = OokDetector::new(RATE, PulseConfig::default());
        let mut b = Vec::new();
        for c in env.chunks(997) {
            split.process(c, &mut b);
        }
        assert_eq!(a, b, "block splitting changed the pulse train");
    }

    #[test]
    fn duration_excludes_the_trailing_timeout() {
        let want = [(500u32, 500u32), (1000, 500), (500, 500), (1000, 500)];
        let env = envelope(&want, 1.0, 0.02);
        let mut d = OokDetector::new(RATE, PulseConfig::default());
        let mut out = Vec::new();
        d.process(&env, &mut out);
        let dur = out[0].duration_us();
        let expect: u64 = want.iter().map(|(m, g)| *m as u64 + *g as u64).sum::<u64>() - 500;
        assert!(dur.abs_diff(expect) < 200, "duration {dur} vs expected {expect}");
    }
}
