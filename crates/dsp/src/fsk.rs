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

use crate::pulse::{LevelGate, PulseStats, dbfs, detected};
use common::C32;
use common::packet::Detection;

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
    pub fn process(&mut self, input: &[C32], out: &mut Vec<Detection>) {
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
    pub fn flush(&mut self, out: &mut Vec<Detection>) {
        if self.in_burst {
            self.finish(out);
        }
    }

    fn finish(&mut self, out: &mut Vec<Detection>) {
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
            out.push(detected(
                pulses,
                common::Modulation::Fsk2,
                dbfs(self.gate.signal_level()),
                snr,
                self.burst_start,
            ));
            self.stats.accepted += 1;
        } else if !pulses.is_empty() {
            self.stats.rejected_too_few_pulses += 1;
        }
        self.burst.clear();
    }
}

/// Continuous two-level FSK at a known baud: samples in, a bit stream out.
///
/// The detector above cuts a burst into mark and gap timings, which is the
/// vocabulary a sensor packet is written in: a short burst, a few tens of
/// symbols, timings that a published protocol table transcribes directly.
/// That vocabulary does not fit a transmitter that keys thousands of symbols
/// at a fixed baud, where the clock has to be tracked rather than measured,
/// and where a run of twelve identical bits is ordinary rather than a sign
/// the burst has ended.
///
/// This is the other half: a discriminator, a matched filter one symbol
/// wide, and a Gardner timing loop reading at two samples a symbol. What
/// comes out is bits with no framing at all, because framing is the caller's
/// business ([`crate::hdlc`], a sync word, a header search).
///
/// Nothing here knows what it is reading. A radiosonde is 4800 baud and a
/// low-power telemetry link is 38400; both are this, parameterised.
pub struct BitSync {
    rate: f64,
    baud: f64,
    /// Input samples per symbol.
    sps: f64,
    /// Channel filter, or none where the stream is already no wider than
    /// the signal.
    lp: Option<crate::fir::Fir>,
    narrow: Vec<C32>,
    prev: C32,
    /// Slow mean of the discriminator, which is the tuning error between
    /// this receiver and the transmitter.
    dc: f32,
    dc_alpha: f32,
    /// Boxcar over one symbol: the matched filter for rectangular keying,
    /// and close enough for the Gaussian shaping a GFSK transmitter uses.
    box_ring: Vec<f32>,
    box_pos: usize,
    box_sum: f32,
    /// The last filtered sample, for interpolating between it and this one.
    last: f32,
    /// Fractional input-sample index of the next half-symbol strobe,
    /// relative to the newest sample.
    next: f64,
    /// Half-symbol strobes since the last symbol decision: the loop reads at
    /// two samples a symbol and decides on every second one.
    half_phase: bool,
    /// The reading halfway between the last two symbols, which is where a
    /// transition would be if the clock were early or late.
    mid: f32,
    /// The last symbol's reading.
    sym: f32,
    /// Running mean square of the filtered signal, which normalises the
    /// timing error so the loop gain does not depend on how loud it is.
    power: f32,
}

/// How hard the timing loop pulls, as a fraction of a symbol per unit of
/// normalised error. Low enough that noise does not walk the clock off a
/// long frame, high enough to pull in a few hundred ppm of crystal error
/// within a preamble.
const TIMING_GAIN: f64 = 0.02;

impl BitSync {
    /// A demodulator for `baud` symbols a second at modulation index one,
    /// which is where the deviation is half the baud and the signal occupies
    /// about `2 * baud` by Carson's rule. Use [`BitSync::with_bandwidth`]
    /// for a link keyed wider or narrower than that.
    pub fn new(rate: f64, baud: f64) -> Self {
        Self::with_bandwidth(rate, baud, 2.0 * baud)
    }

    /// The same, saying how much spectrum the signal occupies.
    ///
    /// The filter is why this matters rather than being a detail: handed a
    /// stream three times the signal's width it carries three times the
    /// noise into the discriminator, and a discriminator's output degrades
    /// sharply rather than gracefully once the noise reaches it. Measured on
    /// a 31.25 kS/s recording of a radiosonde, filtering the 9.6 kHz the
    /// sonde occupies out of it took the frames read from 8 of 28 to all 28.
    pub fn with_bandwidth(rate: f64, baud: f64, bandwidth_hz: f64) -> Self {
        let sps = rate / baud;
        let box_len = sps.round().max(1.0) as usize;
        let cutoff = bandwidth_hz / 2.0 / rate;
        // Nothing to do where the stream is already about as narrow as the
        // signal: a filter there is a pass over the samples for no gain.
        let lp = (cutoff < 0.4).then(|| {
            let taps = crate::fir::estimate_taps(cutoff / 2.0, 50.0).min(255);
            crate::fir::Fir::new(crate::fir::lowpass(taps, cutoff, 50.0))
        });
        Self {
            rate,
            baud,
            sps,
            lp,
            narrow: Vec::new(),
            prev: C32::new(1.0, 0.0),
            dc: 0.0,
            // Sixty-four symbols. Anything a transmitter keys this way is
            // balanced over that, whether by scrambling or by a preamble,
            // and it is short enough to follow a drifting tuner.
            dc_alpha: 1.0 / (64.0 * sps as f32),
            box_ring: vec![0.0; box_len],
            box_pos: 0,
            box_sum: 0.0,
            last: 0.0,
            next: 0.0,
            half_phase: false,
            mid: 0.0,
            sym: 0.0,
            power: 1e-6,
        }
    }

    /// Four samples a symbol is the floor: below it the half-symbol strobe
    /// has nothing to interpolate between.
    pub fn usable(&self) -> bool {
        self.sps >= 4.0
    }

    pub fn sps(&self) -> f64 {
        self.sps
    }

    pub fn baud(&self) -> f64 {
        self.baud
    }

    /// The tuning error the loop has settled on, in hertz. A sonde is
    /// specified to within a few kilohertz of its nominal channel and this
    /// is how far off it actually is.
    pub fn offset_hz(&self) -> f32 {
        self.dc * (self.rate / std::f64::consts::TAU) as f32
    }

    pub fn reset(&mut self) {
        if let Some(lp) = &mut self.lp {
            lp.reset();
        }
        self.prev = C32::new(1.0, 0.0);
        self.dc = 0.0;
        self.box_ring.fill(0.0);
        self.box_sum = 0.0;
        self.box_pos = 0;
        self.last = 0.0;
        self.next = 0.0;
        self.half_phase = false;
        self.mid = 0.0;
        self.sym = 0.0;
        self.power = 1e-6;
    }

    /// Feed samples, appending every bit the clock decided to `bits`.
    pub fn process(&mut self, input: &[C32], bits: &mut Vec<bool>) {
        if !self.usable() {
            return;
        }
        let half = self.sps / 2.0;
        let mut narrow = std::mem::take(&mut self.narrow);
        narrow.clear();
        if let Some(lp) = &mut self.lp {
            lp.process(input, &mut narrow);
        } else {
            narrow.extend_from_slice(input);
        }
        for &x in &narrow {
            let d = x * self.prev.conj();
            self.prev = x;
            let f = if d.norm_sqr() > 0.0 { d.arg() } else { 0.0 };
            self.dc += self.dc_alpha * (f - self.dc);
            let v = f - self.dc;
            self.box_sum += v - self.box_ring[self.box_pos];
            self.box_ring[self.box_pos] = v;
            self.box_pos = (self.box_pos + 1) % self.box_ring.len();
            let y = self.box_sum / self.box_ring.len() as f32;
            self.power += 0.001 * (y * y - self.power);

            // `next` counts down towards this sample as each one arrives,
            // so a strobe at 0 is the previous sample and at 1 is this one.
            self.next -= 1.0;
            while self.next <= 0.0 {
                let frac = (self.next + 1.0).clamp(0.0, 1.0) as f32;
                let s = self.last + (y - self.last) * frac;
                self.next += half;
                if self.half_phase {
                    // A symbol instant: decide, then ask the halfway
                    // reading whether the clock is early or late. Gardner's
                    // detector, which needs no decisions and so works
                    // before the loop has locked.
                    let e = ((s - self.sym) * self.mid / self.power.max(1e-9)) as f64;
                    self.sym = s;
                    bits.push(s > 0.0);
                    self.next -= (TIMING_GAIN * e).clamp(-0.4, 0.4) * half;
                } else {
                    self.mid = s;
                }
                self.half_phase = !self.half_phase;
            }
            self.last = y;
        }
        self.narrow = narrow;
    }
}

/// Narrow-shift two-level FSK on complex baseband: a correlator at each
/// tone, and a bit clock.
///
/// [`BitSync`] discriminates and slices, which wants a shift of about the
/// baud or more and a stream that is balanced between the two tones. RTTY is
/// neither: 170 Hz of shift at 45 baud is a modulation index near four, and
/// the line rests on the mark tone between overs, so the slow mean a
/// discriminator centres on walks onto the mark and the slicer then reads
/// noise as data.
///
/// A correlator pair has no such reference to lose. Each tone is integrated
/// over one symbol and the stronger wins, exactly as [`crate::afsk`] tells
/// 1200 Hz from 2200 Hz, except that the tones here are a shift either side
/// of the channel centre rather than audio frequencies. What comes out is
/// symbols; the start and stop framing above is the caller's.
///
/// The tones are where the operator tuned, and there is no search: measured
/// on a keyed 170 Hz shift at 45.45 baud, an over reads whole up to 25 Hz
/// off channel and loses characters beyond that.
pub struct TonePair {
    mark: ComplexTone,
    space: ComplexTone,
    sps: f32,
    /// How far the clock is taken to be through a symbol when a transition
    /// is seen, in samples.
    after_edge: f32,
    since: f32,
    last_sign: bool,
    min_level: f32,
    margin: f32,
    clock_gain: f32,
}

impl TonePair {
    /// Tones a `shift_hz` apart, straddling the middle of the stream, keyed
    /// at `baud`. The mark is the higher of the two, which is how every
    /// RTTY station keys and how a shift is quoted.
    pub fn new(rate: f64, baud: f64, shift_hz: f64) -> Self {
        let sps = rate / baud;
        // Two cycles of the shift, or a whole symbol where that is shorter.
        //
        // A boxcar correlator is blind at every multiple of the reciprocal
        // of its window, so a window of a whole symbol resolves a 45 baud
        // station to about 45 Hz and a station tuned 40 Hz off its channel
        // lands in the first null and reads as neither tone. Two cycles of
        // the shift puts the *other* tone in a null, where it belongs, and
        // widens the room a mistuned station has: measured on a keyed 170 Hz
        // shift at 45.45 baud, a whole symbol reads nothing 25 Hz off and
        // this reads the over whole.
        let window = (sps.round() as usize).min((2.0 * rate / shift_hz) as usize).max(2);
        Self {
            mark: ComplexTone::new(shift_hz / 2.0, rate, window),
            space: ComplexTone::new(-shift_hz / 2.0, rate, window),
            sps: sps as f32,
            // A correlator is a sliding window ending at the sample it
            // reports, so its verdict changes about half a window after the
            // tone did. Counting from there, the end of the symbol, where
            // the window covers that symbol and nothing else, is a symbol
            // less half a window away.
            after_edge: (window as f64 / 2.0).min(sps - 1.0).max(0.0) as f32,
            since: 0.0,
            last_sign: true,
            // Both correlators are normalised by the window, so this is a
            // level in the same units as the input samples rather than a
            // number that changes with the rate.
            min_level: 1e-5,
            margin: DEFAULT_MARGIN,
            clock_gain: 0.35,
        }
    }

    /// How far apart the two correlators must be to have decided anything.
    /// See [`DEFAULT_MARGIN`].
    pub fn with_margin(mut self, margin: f32) -> Self {
        self.margin = margin;
        self
    }

    /// Four samples a symbol, below which the clock has nothing to work
    /// with and the correlators no longer resolve the two tones.
    pub fn usable(&self) -> bool {
        self.sps >= 4.0
    }

    pub fn reset(&mut self) {
        self.mark.reset();
        self.space.reset();
        self.since = 0.0;
        self.last_sign = true;
    }

    /// Decide a symbol at every bit instant in this block of baseband.
    pub fn process(&mut self, iq: &[C32], out: &mut Vec<crate::afsk::Symbol>) {
        if !self.usable() {
            return;
        }
        for &x in iq {
            let m = self.mark.push(x);
            let s = self.space.push(x);
            let sign = m > s;
            if sign != self.last_sign {
                self.since += self.clock_gain * (self.after_edge - self.since);
                self.last_sign = sign;
            }
            self.since += 1.0;
            if self.since < self.sps {
                continue;
            }
            self.since -= self.sps;
            let sum = m + s;
            let quiet = sum < self.min_level || (m - s).abs() < self.margin * sum;
            out.push(crate::afsk::Symbol { mark: sign, quiet });
        }
    }
}

/// How much one correlator must beat the other by, as a fraction of the two
/// together, before the symbol has been decided.
///
/// This is what keeps an asynchronous framer above from reading noise. Over
/// noise the two correlators are independent and this ratio is spread evenly
/// across the whole range, so a threshold of a half calls about half of all
/// noise symbols undecided, and a character needs seven in a row. A keyed
/// tone beats the other correlator outright even when the station is
/// mistuned by a quarter of the shift.
pub const DEFAULT_MARGIN: f32 = 0.5;

/// One tone correlator over complex baseband: a sliding integration against
/// a reference at `freq`, whose magnitude says how much of that tone is
/// there. The complex twin of [`crate::afsk`]'s, and it needs no quadrature
/// pair because the input already has both.
struct ComplexTone {
    step: f64,
    phase: f64,
    hist: Vec<C32>,
    pos: usize,
    sum: C32,
    scale: f32,
}

impl ComplexTone {
    fn new(freq: f64, rate: f64, window: usize) -> Self {
        Self {
            step: -std::f64::consts::TAU * freq / rate,
            phase: 0.0,
            hist: vec![C32::new(0.0, 0.0); window],
            pos: 0,
            sum: C32::new(0.0, 0.0),
            scale: 1.0 / window as f32,
        }
    }

    fn push(&mut self, x: C32) -> f32 {
        let (s, c) = self.phase.sin_cos();
        self.phase = (self.phase + self.step).rem_euclid(std::f64::consts::TAU);
        let v = x * C32::new(c as f32, s as f32);
        self.sum += v - self.hist[self.pos];
        self.hist[self.pos] = v;
        self.pos = (self.pos + 1) % self.hist.len();
        self.sum.norm_sqr() * self.scale * self.scale
    }

    fn reset(&mut self) {
        self.hist.fill(C32::new(0.0, 0.0));
        self.sum = C32::new(0.0, 0.0);
        self.pos = 0;
        self.phase = 0.0;
    }
}

/// One framing a [`SyncDetector`] looks for: a sync word and the payload
/// that follows it.
#[derive(Clone, Copy, Debug)]
pub struct SyncPattern {
    /// The sync word, in the order it is keyed, in the low `bits` bits.
    pub word: u64,
    /// How long the sync word is, at most 64.
    pub bits: u32,
    /// Payload bits that follow it, fixed by the framing.
    pub payload_bits: usize,
    /// Sync bits allowed to be wrong and the word still be this one.
    pub max_errors: u32,
}

impl SyncPattern {
    fn mask(&self) -> u64 {
        if self.bits >= 64 { u64::MAX } else { (1u64 << self.bits) - 1 }
    }
}

/// A payload read off the air behind a sync word.
#[derive(Clone, Debug)]
pub struct SyncBurst {
    /// Which of the detector's patterns matched.
    pub pattern: usize,
    /// Sync bits that were wrong.
    pub errors: u32,
    pub bits: Vec<bool>,
    /// Input sample the sync word started at.
    pub at_sample: u64,
    /// Input samples the sync word and the payload together occupied.
    pub len_samples: usize,
}

/// A pattern matched, waiting for its payload to arrive.
#[derive(Clone, Copy)]
struct Pending {
    pattern: usize,
    errors: u32,
    /// Resampled index of the first payload bit's sample.
    first: u64,
    /// Resampled index of the last one.
    last: u64,
    /// The threshold this frame's bits are sliced against, measured off the
    /// sync word rather than off the stream: a payload that is mostly one
    /// tone drags a running mean onto itself, and the sync word is the one
    /// place in the frame where both levels are known.
    mid: f32,
    at_sample: u64,
}

/// Two-level FSK carrying fixed-length frames behind a sync word.
///
/// [`BitSync`] recovers a clock from the signal and hands back bits with no
/// framing; that wants four samples a symbol. A link keyed near a megabit
/// does not get four samples a symbol from a receiver running at a couple of
/// megasamples, so this takes the other approach: discriminate, resample to
/// exactly two samples a bit, and correlate a sync word against the hard
/// decisions at each of the two phases. Two samples a bit is the floor, and
/// what UAT at 1.041667 Mbit/s leaves at 2.4 MS/s.
///
/// Nothing here knows what it is reading. It is given sync words, payload
/// lengths and a baud; UAT's two framings are one configuration of it.
pub struct SyncDetector {
    rate: f64,
    baud: f64,
    patterns: Vec<SyncPattern>,
    /// Resampled samples per input sample.
    step: f64,
    /// Input samples per resampled sample, for reporting a frame's place in
    /// the stream it was read from.
    per_out: f64,
    acc: f64,
    prev: C32,
    last_f: f32,
    /// Slow mean of the discriminator, which is the tuning error between
    /// this receiver and the transmitter, and the threshold a sync word is
    /// hard-decided against.
    dc: f32,
    dc_alpha: f32,
    ring: Vec<f32>,
    mask: usize,
    /// Resampled samples written since the detector was built.
    idx: u64,
    /// Hard decisions, one register per phase of the two samples a bit.
    reg: [u64; 2],
    pending: Vec<Pending>,
    blank_until: u64,
    in_sample: u64,
}

/// How many matches may be waiting for their payloads at once.
///
/// A 36-bit sync word allowing four wrong bits matches noise about once in a
/// million samples, so a second match inside a 4.4 ms uplink frame is rare
/// and a third rarer still; the cap only stops a pathological signal growing
/// the list without limit.
const MAX_PENDING: usize = 4;

impl SyncDetector {
    pub fn new(rate: f64, baud: f64, patterns: Vec<SyncPattern>) -> Self {
        let longest =
            patterns.iter().map(|p| p.payload_bits + p.bits as usize).max().unwrap_or(1024);
        let cap = (2 * longest + 64).next_power_of_two();
        Self {
            rate,
            baud,
            patterns,
            step: 2.0 * baud / rate,
            per_out: rate / (2.0 * baud),
            acc: 0.0,
            prev: C32::new(1.0, 0.0),
            last_f: 0.0,
            dc: 0.0,
            // Sixty-four bits, as elsewhere in this module: long enough that
            // a sync word cannot pull it, short enough to follow a tuner.
            dc_alpha: 1.0 / 128.0,
            ring: vec![0.0; cap],
            mask: cap - 1,
            idx: 0,
            reg: [0; 2],
            pending: Vec::new(),
            blank_until: 0,
            in_sample: 0,
        }
    }

    /// Two input samples a bit is the floor: below it the two phases are the
    /// same phase and there is nothing to correlate.
    pub fn usable(&self) -> bool {
        self.rate / self.baud >= 2.0
    }

    pub fn reset(&mut self) {
        self.acc = 0.0;
        self.prev = C32::new(1.0, 0.0);
        self.last_f = 0.0;
        self.dc = 0.0;
        self.ring.fill(0.0);
        self.idx = 0;
        self.reg = [0; 2];
        self.pending.clear();
        self.blank_until = 0;
        self.in_sample = 0;
    }

    /// Read `input`, appending every frame whose sync word matched.
    pub fn process(&mut self, input: &[C32], out: &mut Vec<SyncBurst>) {
        self.process_valid(input, out, &|_| true);
    }

    /// The same, with `accept` deciding which frames are real.
    ///
    /// A frame `accept` believes blanks the air it occupied, so a sync word
    /// found inside a payload cannot cut a frame in half. A frame it rejects
    /// blanks nothing, because the evidence that a match was noise is the
    /// check `accept` runs and there is none here.
    pub fn process_valid(
        &mut self,
        input: &[C32],
        out: &mut Vec<SyncBurst>,
        accept: &dyn Fn(&SyncBurst) -> bool,
    ) {
        if !self.usable() {
            return;
        }
        for &x in input {
            let d = x * self.prev.conj();
            self.prev = x;
            let f = if d.norm_sqr() > 0.0 { d.arg() } else { 0.0 };
            let before = self.acc;
            self.acc += self.step;
            let mut k = 1.0;
            while self.acc >= 1.0 {
                // Where between the last input sample and this one the
                // output sample falls.
                let t = ((k - before) / self.step).clamp(0.0, 1.0) as f32;
                let v = self.last_f + (f - self.last_f) * t;
                self.push(v, out, accept);
                self.acc -= 1.0;
                k += 1.0;
            }
            self.last_f = f;
            self.in_sample += 1;
        }
    }

    fn push(&mut self, v: f32, out: &mut Vec<SyncBurst>, accept: &dyn Fn(&SyncBurst) -> bool) {
        let n = self.idx;
        self.ring[(n as usize) & self.mask] = v;
        self.idx = n + 1;
        self.dc += self.dc_alpha * (v - self.dc);
        let phase = (n & 1) as usize;
        self.reg[phase] = (self.reg[phase] << 1) | u64::from(v > self.dc);

        self.complete(n, out, accept);
        if n < self.blank_until || self.pending.len() >= MAX_PENDING {
            return;
        }
        for (i, p) in self.patterns.iter().enumerate() {
            if n + 1 < 2 * u64::from(p.bits) {
                continue;
            }
            let errors = ((self.reg[phase] ^ p.word) & p.mask()).count_ones();
            if errors > p.max_errors {
                continue;
            }
            let Some(mid) = self.sync_mid(n, p) else { continue };
            let first = n + 2;
            let pending = Pending {
                pattern: i,
                errors,
                first,
                last: n + 2 * p.payload_bits as u64,
                mid,
                // The sync word began two samples a bit ago, counted back
                // through the resampler into the stream that was handed in.
                at_sample: self
                    .in_sample
                    .saturating_sub((2.0 * f64::from(p.bits) * self.per_out) as u64),
            };
            self.pending.push(pending);
            break;
        }
    }

    /// The level halfway between the two tones, measured over the sync word
    /// whose bits are known.
    fn sync_mid(&self, n: u64, p: &SyncPattern) -> Option<f32> {
        let (mut mark, mut space) = (0.0f32, 0.0f32);
        let (mut marks, mut spaces) = (0u32, 0u32);
        for k in 0..u64::from(p.bits) {
            let v = self.ring[((n - 2 * k) as usize) & self.mask];
            if (p.word >> k) & 1 == 1 {
                mark += v;
                marks += 1;
            } else {
                space += v;
                spaces += 1;
            }
        }
        if marks == 0 || spaces == 0 {
            return None;
        }
        Some((mark / marks as f32 + space / spaces as f32) / 2.0)
    }

    fn complete(&mut self, n: u64, out: &mut Vec<SyncBurst>, accept: &dyn Fn(&SyncBurst) -> bool) {
        let Some(i) = self.pending.iter().position(|p| p.last == n) else { return };
        let p = self.pending.remove(i);
        let count = ((p.last - p.first) / 2 + 1) as usize;
        let bits = (0..count)
            .map(|k| self.ring[((p.first + 2 * k as u64) as usize) & self.mask] > p.mid)
            .collect();
        let pattern = self.patterns[p.pattern];
        let burst = SyncBurst {
            pattern: p.pattern,
            errors: p.errors,
            bits,
            at_sample: p.at_sample,
            len_samples: ((pattern.bits as usize + pattern.payload_bits) as f64 * self.rate
                / self.baud) as usize,
        };
        if accept(&burst) {
            self.blank_until = p.last + 1;
            self.pending.retain(|q| q.first > p.last);
            out.push(burst);
        }
    }
}

/// Key `bits` as two-level FSK at `baud`, for tests and for anything that
/// wants to make a signal.
pub fn modulate(bits: &[bool], rate: f64, baud: f64, deviation_hz: f64, amp: f32) -> Vec<C32> {
    let sps = rate / baud;
    let mut out = Vec::with_capacity((bits.len() as f64 * sps) as usize + 1);
    let mut ph = 0.0f64;
    for (i, &b) in bits.iter().enumerate() {
        let f = if b { deviation_hz } else { -deviation_hz };
        while (out.len() as f64) < (i + 1) as f64 * sps {
            ph += std::f64::consts::TAU * f / rate;
            out.push(C32::new(amp * ph.cos() as f32, amp * ph.sin() as f32));
        }
    }
    out
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
                phase =
                    (phase + std::f64::consts::TAU * f / RATE).rem_euclid(std::f64::consts::TAU);
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

    fn detect(iq: &[C32], cfg: FskConfig) -> (Vec<Detection>, FskDetector) {
        let mut d = FskDetector::new(RATE, cfg);
        let mut out = Vec::new();
        d.process(iq, &mut out);
        d.flush(&mut out);
        (out, d)
    }

    #[test]
    fn recovers_symbol_timings_from_a_keyed_carrier() {
        let syms =
            nrz(&[1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 1, 0, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0], 100);
        let iq = burst(&syms, 25_000.0, 0.0, 1.0, 0.02);
        let (pkgs, _) = detect(&iq, FskConfig::default());

        assert_eq!(pkgs.len(), 1, "expected one burst, got {}", pkgs.len());
        let p = &pkgs[0];
        // Runs of like symbols merge, so count transitions rather than bits.
        for (i, pulse) in p.pulses().iter().enumerate() {
            assert!(pulse.mark % 100 < 25 || pulse.mark % 100 > 75, "pulse {i}: {pulse:?}");
        }
        let marks = common::pulse::mark_histogram(p.pulses(), 30);
        assert!(
            marks.iter().any(|(c, _)| c.abs_diff(100) < 25),
            "no cluster at one symbol: {marks:?}"
        );
    }

    #[test]
    fn a_tuning_offset_does_not_move_the_threshold() {
        // The tones sit 40 kHz off centre, far more than the deviation. A
        // fixed threshold at zero hertz would call every symbol a mark.
        let syms =
            nrz(&[1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 1, 0, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0], 100);
        let clean = burst(&syms, 25_000.0, 0.0, 1.0, 0.02);
        let offset = burst(&syms, 25_000.0, 40_000.0, 1.0, 0.02);
        let (a, _) = detect(&clean, FskConfig::default());
        let (b, _) = detect(&offset, FskConfig::default());
        assert_eq!(a.len(), 1);
        assert_eq!(a[0].pulses().len(), b[0].pulses().len(), "offset changed the pulse train");
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

    /// A pseudo-random bit stream at 4800 baud, keyed and read back. The
    /// loop needs a lead-in to pull the clock in, so the test looks for its
    /// own sequence inside what came out rather than at the front of it.
    #[test]
    fn the_bit_clock_reads_a_stream_back() {
        let baud = 4800.0;
        let rate = 48_000.0;
        let mut seed = 0x243f_6a88_85a3_08d3u64;
        let bits: Vec<bool> = (0..4000)
            .map(|_| {
                seed ^= seed << 13;
                seed ^= seed >> 7;
                seed ^= seed << 17;
                seed & 1 != 0
            })
            .collect();
        // 2.4 kHz either way, which is what a radiosonde keys, riding on a
        // 900 Hz tuning error the loop has to take out by itself.
        let iq = modulate(&bits, rate, baud, 2_400.0, 0.5);
        let mut ph = 0.0f64;
        let iq: Vec<C32> = iq
            .iter()
            .map(|s| {
                ph += std::f64::consts::TAU * 900.0 / rate;
                s * C32::new(ph.cos() as f32, ph.sin() as f32)
            })
            .collect();
        let mut sync = BitSync::new(rate, baud);
        assert!(sync.usable());
        let mut got = Vec::new();
        for block in iq.chunks(1000) {
            sync.process(block, &mut got);
        }
        assert!(got.len() >= 3900, "{} bits out of 4000 symbols", got.len());
        // Find where the stream lines up, then require the rest exactly.
        let want = &bits[500..3500];
        let at = (0..got.len().saturating_sub(want.len()))
            .find(|&k| got[k..k + want.len()] == *want)
            .expect("the keyed stream is not in what came out");
        assert!(at < 600, "it took {at} bits to lock");
        // The offset is a one-pole mean over 64 symbols, so a random stream
        // leaves it a tenth of a symbol's swing out. That is a reading of
        // the tuning error, not a correction of it: the loop only needs the
        // mean to be close enough that the slicer is not biased.
        assert!((sync.offset_hz() - 900.0).abs() < 150.0, "{} Hz", sync.offset_hz());
    }

    /// A narrow shift at a low baud, which is RTTY: 170 Hz apart at 45.45
    /// baud is where a discriminator gives up and the correlator pair does
    /// not. The station is mistuned by 40 Hz, a quarter of the shift, which
    /// is about as far out as an operator leaves it.
    #[test]
    fn the_tone_pair_reads_a_narrow_shift() {
        let (rate, baud, shift) = (8_000.0, 45.45, 170.0);
        let mut seed = 0x1234_5678_9abc_def0u64;
        let bits: Vec<bool> = (0..200)
            .map(|_| {
                seed ^= seed << 13;
                seed ^= seed >> 7;
                seed ^= seed << 17;
                seed & 1 != 0
            })
            .collect();
        let iq = modulate(&bits, rate, baud, shift / 2.0, 0.5);
        let mut ph = 0.0f64;
        let iq: Vec<C32> = iq
            .iter()
            .map(|s| {
                ph += std::f64::consts::TAU * 40.0 / rate;
                s * C32::new(ph.cos() as f32, ph.sin() as f32)
            })
            .collect();

        let mut tp = TonePair::new(rate, baud, shift);
        assert!(tp.usable());
        let mut got = Vec::new();
        for block in iq.chunks(512) {
            tp.process(block, &mut got);
        }
        assert!(got.len() >= 198, "{} symbols out of 200", got.len());
        let levels: Vec<bool> = got.iter().map(|s| s.mark).collect();
        let want = &bits[4..190];
        let at = (0..levels.len().saturating_sub(want.len()))
            .find(|&k| levels[k..k + want.len()] == *want)
            .expect("the keyed symbols are not in what came out");
        assert!(at < 8, "it took {at} symbols to line up");
        assert!(got.iter().all(|s| !s.quiet), "a keyed carrier read as silence");
    }

    /// An empty channel is silence rather than a run of marks, which is what
    /// an asynchronous framer above needs to know the line is resting.
    #[test]
    fn the_tone_pair_calls_an_empty_channel_quiet() {
        let mut tp = TonePair::new(8_000.0, 45.45, 170.0);
        let mut got = Vec::new();
        tp.process(&vec![C32::new(0.0, 0.0); 8_000], &mut got);
        assert!(got.len() > 40, "{} symbols in a second", got.len());
        assert!(got.iter().all(|s| s.quiet), "silence read as signal");
    }

    /// Four samples a symbol is the floor, and below it the demodulator
    /// refuses rather than returning bits it cannot have read.
    #[test]
    fn the_bit_clock_refuses_a_stream_it_cannot_read() {
        assert!(!BitSync::new(14_400.0, 4800.0).usable());
        assert!(BitSync::new(19_200.0, 4800.0).usable());
        let mut slow = BitSync::new(14_400.0, 4800.0);
        let mut bits = Vec::new();
        slow.process(&vec![C32::new(0.5, 0.0); 1000], &mut bits);
        assert!(bits.is_empty());
    }

    /// A megabit link read at two samples a bit, which is what a 978 MHz
    /// UAT frame is at 2.4 MS/s.
    mod sync {
        use super::super::*;

        const RATE: f64 = 2_400_000.0;
        const BAUD: f64 = 1_041_667.0;
        const SYNC: u64 = 0xEAC_DDA_4E2;

        fn pattern(payload_bits: usize, max_errors: u32) -> SyncPattern {
            SyncPattern { word: SYNC, bits: 36, payload_bits, max_errors }
        }

        fn bits_of(word: u64, n: u32) -> Vec<bool> {
            (0..n).rev().map(|k| (word >> k) & 1 == 1).collect()
        }

        fn noise(n: usize, seed: u64, amp: f32) -> Vec<C32> {
            let mut s = seed;
            let mut rng = move || {
                s = s.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
                ((s >> 33) as f32 / (1u64 << 30) as f32 - 1.0) * amp
            };
            (0..n).map(|_| C32::new(rng(), rng())).collect()
        }

        /// A payload keyed behind its sync word comes back bit for bit.
        #[test]
        fn a_frame_behind_a_sync_word_reads_back_whole() {
            let payload: Vec<bool> = (0..240).map(|i| (i * 7 + i / 3) % 3 != 0).collect();
            let mut air = bits_of(SYNC, 36);
            air.extend_from_slice(&payload);
            let iq = modulate(&air, RATE, BAUD, 312_500.0, 1.0);

            let mut det = SyncDetector::new(RATE, BAUD, vec![pattern(240, 4)]);
            let mut out = Vec::new();
            det.process(&noise(4096, 1, 0.01), &mut out);
            det.process(&iq, &mut out);
            det.process(&noise(4096, 2, 0.01), &mut out);

            assert_eq!(out.len(), 1, "one frame off the air");
            assert_eq!(out[0].pattern, 0);
            assert_eq!(out[0].errors, 0);
            assert_eq!(out[0].bits, payload);
        }

        /// The two framings a UAT receiver looks for are told apart by their
        /// sync words, and each reads only its own length.
        #[test]
        fn two_patterns_are_told_apart_by_their_sync_words() {
            const UPLINK: u64 = 0x153_225_B1D;
            let payload: Vec<bool> = (0..384).map(|i| i % 5 < 2).collect();
            let mut air = bits_of(UPLINK, 36);
            air.extend_from_slice(&payload);
            let iq = modulate(&air, RATE, BAUD, 312_500.0, 1.0);

            let mut det = SyncDetector::new(
                RATE,
                BAUD,
                vec![
                    pattern(384, 4),
                    SyncPattern { word: UPLINK, bits: 36, payload_bits: 384, max_errors: 4 },
                ],
            );
            let mut out = Vec::new();
            det.process(&noise(4096, 3, 0.01), &mut out);
            det.process(&iq, &mut out);
            det.process(&noise(4096, 4, 0.01), &mut out);
            assert_eq!(out.len(), 1);
            assert_eq!(out[0].pattern, 1, "the ground station's word, not the aircraft's");
            assert_eq!(out[0].bits, payload);
        }

        /// A frame a mile off the tuned frequency still reads: the sync word
        /// is sliced against a running mean, so a tuning error moves both
        /// tones together and cancels.
        #[test]
        fn a_frame_off_frequency_still_reads() {
            let payload: Vec<bool> = (0..240).map(|i| i % 4 < 2).collect();
            let mut air = bits_of(SYNC, 36);
            air.extend_from_slice(&payload);
            // 40 kHz of tuning error, which is 40 ppm at 978 MHz.
            let iq: Vec<C32> = modulate(&air, RATE, BAUD, 312_500.0, 1.0)
                .iter()
                .enumerate()
                .map(|(i, x)| {
                    let ph = std::f64::consts::TAU * 40_000.0 * i as f64 / RATE;
                    x * C32::new(ph.cos() as f32, ph.sin() as f32)
                })
                .collect();
            let mut det = SyncDetector::new(RATE, BAUD, vec![pattern(240, 4)]);
            let mut out = Vec::new();
            det.process(&noise(4096, 5, 0.01), &mut out);
            det.process(&iq, &mut out);
            det.process(&noise(4096, 6, 0.01), &mut out);
            assert_eq!(out.len(), 1);
            assert_eq!(out[0].bits, payload);
        }

        /// Noise is not a frame. A 36-bit word allowing four wrong bits
        /// matches noise about once in a million samples, which over two
        /// seconds at 2.4 MS/s is a handful of candidates and no more; what
        /// throws them away is the check the caller runs, so this pins the
        /// number reaching that check rather than nought.
        #[test]
        fn noise_offers_only_a_handful_of_candidates() {
            let mut det = SyncDetector::new(RATE, BAUD, vec![pattern(240, 4)]);
            let mut out = Vec::new();
            for block in 0..48 {
                det.process(&noise(100_000, 100 + block, 0.2), &mut out);
            }
            // Measured on this stream: 4 candidates in 4.8 million samples.
            assert!(out.len() <= 12, "{} candidates off noise", out.len());
            // And none of them with a sync word that matched exactly.
            assert_eq!(out.iter().filter(|b| b.errors == 0).count(), 0);
        }

        /// Fewer than two samples a bit is not a stream this can read, and
        /// it says so rather than returning bits it did not see.
        #[test]
        fn it_refuses_a_stream_below_two_samples_a_bit() {
            let mut det = SyncDetector::new(1_500_000.0, BAUD, vec![pattern(240, 4)]);
            assert!(!det.usable());
            let mut out = Vec::new();
            det.process(&noise(100_000, 7, 0.5), &mut out);
            assert_eq!(out.len(), 0);
        }
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
        let long = pkgs[0].pulses().iter().filter(|p| p.mark > 500).count();
        assert_eq!(long, 1, "the 600 us mark was split: {:?}", pkgs[0].pulses());
    }
}
