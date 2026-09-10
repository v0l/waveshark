//! Blind signal detection: noise-floor tracking and burst detection.
//!
//! This is what turns a channelizer into a receiver that finds things on its
//! own. Two problems have to be solved together:
//!
//! 1. **Where is the noise floor?** It is not a constant. It varies per channel
//!    (filter roll-off, the DC spur, a nearby transmitter's skirts), and it
//!    drifts with gain, temperature and time of day. A fixed threshold is
//!    useless.
//! 2. **When did a burst start and stop?** Amateur, PMR and ISM traffic is
//!    bursty. Averaging over seconds hides it completely: a 500 ms
//!    transmission averaged over 2 s is attenuated by 6 dB, and between
//!    transmissions there is nothing at all. Detection must run on short
//!    frames with peak-hold, not on long means.
//!
//! The floor estimator uses minimum statistics: track the minimum power seen
//! over a sliding window, which is dominated by the gaps between bursts, then
//! correct for the downward bias that taking a minimum introduces. This is
//! robust to a channel being occupied most of the time, which an averaging
//! estimator is not, because a strong continuous carrier drags an average up
//! until it masks itself.

use rayon::prelude::*;
use std::collections::VecDeque;

/// Per-channel noise floor tracker using minimum statistics.
#[derive(Clone, Debug)]
pub struct NoiseFloor {
    /// Minimum power seen in each completed sub-window.
    mins: VecDeque<f32>,
    /// Running minimum of the sub-window currently being filled.
    current: f32,
    filled: usize,
    /// Frames per sub-window.
    sub_len: usize,
    /// Number of sub-windows retained.
    sub_count: usize,
    /// Bias correction: the minimum of N samples of an exponentially
    /// distributed power sits below the mean, so the raw minimum
    /// underestimates the floor. Scaling it back up avoids a threshold that
    /// sits permanently too low and produces constant false detections.
    bias: f32,
    floor: f32,
}

impl NoiseFloor {
    /// `sub_len` frames per sub-window, `sub_count` sub-windows retained. The
    /// product is the memory of the estimator: it must be longer than the
    /// longest expected transmission, or a long over will be learned as noise.
    pub fn new(sub_len: usize, sub_count: usize) -> Self {
        assert!(sub_len >= 1 && sub_count >= 1);
        Self {
            mins: VecDeque::with_capacity(sub_count),
            current: f32::INFINITY,
            filled: 0,
            sub_len,
            sub_count,
            bias: bias_correction(sub_len),
            floor: f32::NAN,
        }
    }

    /// Total frames of history this estimator spans.
    pub fn memory_frames(&self) -> usize {
        self.sub_len * self.sub_count
    }

    pub fn update(&mut self, power: f32) -> f32 {
        self.current = self.current.min(power);
        self.filled += 1;
        if self.filled >= self.sub_len {
            if self.mins.len() == self.sub_count {
                self.mins.pop_front();
            }
            self.mins.push_back(self.current);
            self.current = f32::INFINITY;
            self.filled = 0;

            let m = self.mins.iter().copied().fold(f32::INFINITY, f32::min);
            self.floor = m * self.bias;
        }
        // Before the first sub-window completes, fall back to the running
        // minimum so detection is merely insensitive rather than wrong.
        if self.floor.is_nan() {
            self.current.min(power) * self.bias
        } else {
            self.floor
        }
    }

    /// Current estimate, or NaN before the first sub-window completes.
    pub fn floor(&self) -> f32 {
        self.floor
    }

    /// Ready only once the *whole* window has been observed.
    ///
    /// Declaring readiness after a single sub-window looks harmless and is
    /// not: the estimate would then rest on a handful of samples, sit far too
    /// low, and produce a burst of false detections every time a stream
    /// starts.
    pub fn is_ready(&self) -> bool {
        !self.floor.is_nan() && self.mins.len() >= self.sub_count
    }

    pub fn reset(&mut self) {
        self.mins.clear();
        self.current = f32::INFINITY;
        self.filled = 0;
        self.floor = f32::NAN;
    }
}

/// Expected value of the minimum of `n` exponential samples is `mean / n`, so
/// the raw minimum must be scaled by roughly `n` to recover the mean. Real
/// power is correlated between frames, so the full factor over-corrects; the
/// square root is the standard practical compromise.
fn bias_correction(sub_len: usize) -> f32 {
    (sub_len as f32).sqrt().max(1.0)
}

/// One detected burst.
#[derive(Clone, Debug, PartialEq)]
pub struct Burst {
    pub channel: usize,
    /// Frame index where the burst opened.
    pub start_frame: u64,
    /// Frame index where it closed, or `None` while still open.
    pub end_frame: Option<u64>,
    /// Highest SNR seen during the burst, in dB.
    pub peak_snr_db: f32,
    /// Mean SNR over the burst, in dB.
    pub mean_snr_db: f32,
}

impl Burst {
    pub fn duration_frames(&self) -> Option<u64> {
        self.end_frame.map(|e| e - self.start_frame)
    }
}

#[derive(Clone, Copy, Debug)]
pub struct DetectorConfig {
    /// SNR in dB above the tracked floor required to open.
    pub open_db: f32,
    /// SNR in dB below which the burst closes. Must be under `open_db`;
    /// the gap is the hysteresis that stops a signal fading around the
    /// threshold from producing hundreds of spurious bursts.
    pub close_db: f32,
    /// Frames to keep a burst open after it drops below `close_db`. Bridges
    /// the gaps inside a modulated signal, and stops one transmission being
    /// reported as many.
    pub hang_frames: u64,
    /// Frames a burst must last to be reported at all, rejecting impulse noise
    /// and switching transients.
    pub min_frames: u64,
    /// Sub-window length and count for the floor tracker.
    pub floor_sub_len: usize,
    pub floor_sub_count: usize,
    /// Frames of power integration before thresholding.
    ///
    /// This is not optional smoothing, it is what makes detection work at all.
    /// The power of one frame of complex Gaussian noise is exponentially
    /// distributed, with a standard deviation of about 5.6 dB and a long tail;
    /// thresholding it directly gives false alarms at any usable threshold.
    /// Averaging `n` frames cuts the variance by `n`, so 16 frames brings the
    /// spread down to roughly 1.4 dB and a 10 dB threshold becomes meaningful.
    /// The cost is time resolution: detection cannot resolve bursts shorter
    /// than this.
    pub integrate_frames: usize,
    /// Impulse blanker strength, as a ratio above the current integrated
    /// power. `None` disables it.
    ///
    /// Integration defeats the obvious way of rejecting impulses. A spark, a
    /// switching transient or a USB glitch is one frame long, but after being
    /// smeared by the integrator it becomes a slowly decaying multi-frame
    /// event that a minimum-duration rule can no longer distinguish from a
    /// real short burst. So it has to be dealt with *before* integration:
    /// clamp each frame's power to a bounded multiple of the running average.
    /// A genuine signal ramps the average up over a few frames and passes; a
    /// single-frame outlier is capped and contributes almost nothing.
    pub blank_ratio: Option<f32>,
}

impl Default for DetectorConfig {
    fn default() -> Self {
        Self {
            open_db: 10.0,
            close_db: 6.0,
            hang_frames: 20,
            min_frames: 3,
            floor_sub_len: 64,
            floor_sub_count: 16,
            integrate_frames: 16,
            blank_ratio: Some(10.0),
        }
    }
}

#[derive(Clone, Debug)]
struct ChannelState {
    /// Exponentially integrated power, the quantity actually thresholded.
    smoothed: f32,
    /// Frames observed, used to skip the integrator's warm-up.
    seen: u64,
    floor: NoiseFloor,
    open: bool,
    start: u64,
    peak: f32,
    sum_snr: f64,
    n_snr: u64,
    /// Frames since the level last exceeded `close_db`.
    since_active: u64,
    /// Peak power held since the last read, for display.
    peak_hold: f32,
    /// Whether the burst was open at any point in the current block.
    seen_open: bool,
}

/// Watches every channel of a channelizer and reports bursts.
pub struct Detector {
    cfg: DetectorConfig,
    /// Integration coefficient derived from `cfg.integrate_frames`.
    alpha: f32,
    ch: Vec<ChannelState>,
    frame: u64,
    /// Bursts that closed since the last drain.
    finished: Vec<Burst>,
}

impl Detector {
    pub fn new(channels: usize, cfg: DetectorConfig) -> Self {
        assert!(cfg.close_db < cfg.open_db, "close_db must be below open_db for hysteresis");
        let alpha = 1.0 / cfg.integrate_frames.max(1) as f32;
        let mk = || ChannelState {
            smoothed: 0.0,
            seen: 0,
            floor: NoiseFloor::new(cfg.floor_sub_len, cfg.floor_sub_count),
            open: false,
            start: 0,
            peak: 0.0,
            sum_snr: 0.0,
            n_snr: 0,
            since_active: u64::MAX,
            peak_hold: 0.0,
            seen_open: false,
        };
        Self {
            cfg,
            alpha,
            ch: (0..channels).map(|_| mk()).collect(),
            frame: 0,
            finished: Vec::new(),
        }
    }

    pub fn channels(&self) -> usize {
        self.ch.len()
    }

    pub fn frame(&self) -> u64 {
        self.frame
    }

    pub fn config(&self) -> &DetectorConfig {
        &self.cfg
    }

    /// Feed one channelizer frame: one complex sample per channel.
    pub fn push_frame(&mut self, frame: &[common::C32]) {
        debug_assert_eq!(frame.len(), self.ch.len());
        for (i, s) in frame.iter().enumerate() {
            self.push_channel(i, s.norm_sqr());
        }
        self.frame += 1;
    }

    /// Feed one contiguous block per channel, in parallel.
    ///
    /// Frame-at-a-time updating is the natural way to write this and the wrong
    /// way to run it. At 50 MS/s with 512 channels it is 200 million channel
    /// updates per second of input, all on one thread, and it dominated
    /// everything else in the bank: with no decode chain configured at all the
    /// receiver still reached only 0.72x real time, and this was why.
    ///
    /// Channel states are entirely independent, so the work parallelises
    /// perfectly once samples are laid out channel-major, which the bank's
    /// transpose has already done.
    pub fn process_lanes(&mut self, lanes: &[Vec<common::C32>]) {
        debug_assert_eq!(lanes.len(), self.ch.len());
        let n = lanes.first().map(|l| l.len()).unwrap_or(0);
        let cfg = self.cfg;
        let alpha = self.alpha;
        let start = self.frame;
        for st in &mut self.ch {
            st.seen_open = false;
        }

        let finished: Vec<Vec<Burst>> = self
            .ch
            .par_iter_mut()
            .zip(lanes.par_iter())
            .enumerate()
            .map(|(i, (st, lane))| {
                let mut out = Vec::new();
                for (k, s) in lane.iter().enumerate() {
                    st.step(i, s.norm_sqr(), start + k as u64, &cfg, alpha, &mut out);
                }
                out
            })
            .collect();

        for mut b in finished {
            self.finished.append(&mut b);
        }
        // Keep bursts in channel then time order so output is deterministic
        // regardless of how rayon scheduled the channels.
        self.finished.sort_by_key(|b| (b.channel, b.start_frame));
        self.frame += n as u64;
    }

    /// Feed one channel's instantaneous power. Use when powers are computed
    /// elsewhere, for instance already accumulated over several frames.
    fn push_channel(&mut self, i: usize, power: f32) {
        let cfg = self.cfg;
        let alpha = self.alpha;
        let frame = self.frame;
        let st = &mut self.ch[i];
        st.step(i, power, frame, &cfg, alpha, &mut self.finished);
    }
}

impl ChannelState {
    /// One channel's update for one frame. Split out of `Detector` so it can
    /// run under a parallel iterator holding only this channel's state.
    fn step(
        &mut self,
        i: usize,
        power: f32,
        frame: u64,
        cfg: &DetectorConfig,
        alpha: f32,
        finished: &mut Vec<Burst>,
    ) {
        let st = self;
        // Peak hold stays on the instantaneous value: it exists for display,
        // where seeing the true peak of a short burst is the whole point.
        st.peak_hold = st.peak_hold.max(power);
        st.seen_open |= st.open;

        // Seed on the first frame rather than from zero. Starting at zero
        // makes the integrator ramp up from silence, and the floor tracker
        // faithfully learns that artificially low warm-up as the noise floor,
        // producing a guaranteed false detection the moment it settles.
        if st.seen == 0 {
            st.smoothed = power;
        }
        st.seen += 1;

        let limited = match cfg.blank_ratio {
            Some(r) => power.min(st.smoothed * r),
            None => power,
        };
        st.smoothed += alpha * (limited - st.smoothed);
        let p = st.smoothed;

        // Discard the integrator's settling time entirely, so it never reaches
        // the floor estimator.
        if st.seen <= cfg.integrate_frames as u64 * 4 {
            return;
        }

        let floor = st.floor.update(p);
        if !st.floor.is_ready() {
            return;
        }
        let snr_db = 10.0 * (p.max(1e-30) / floor.max(1e-30)).log10();

        let active = snr_db >= cfg.close_db;
        if active {
            st.since_active = 0;
        } else {
            st.since_active = st.since_active.saturating_add(1);
        }

        if !st.open {
            if snr_db >= cfg.open_db {
                st.open = true;
                st.start = frame;
                st.peak = snr_db;
                st.sum_snr = snr_db as f64;
                st.n_snr = 1;
            }
        } else {
            st.peak = st.peak.max(snr_db);
            st.sum_snr += snr_db as f64;
            st.n_snr += 1;
            if st.since_active > cfg.hang_frames {
                let end = frame - st.since_active.min(frame);
                let dur = end.saturating_sub(st.start);
                if dur >= cfg.min_frames {
                    finished.push(Burst {
                        channel: i,
                        start_frame: st.start,
                        end_frame: Some(end),
                        peak_snr_db: st.peak,
                        mean_snr_db: (st.sum_snr / st.n_snr.max(1) as f64) as f32,
                    });
                }
                st.open = false;
            }
        }
    }
}

impl Detector {
    /// Bursts that completed since the last call. Drains the queue.
    pub fn take_bursts(&mut self) -> Vec<Burst> {
        std::mem::take(&mut self.finished)
    }

    /// Channels with a burst currently in progress.
    pub fn active(&self) -> impl Iterator<Item = usize> + '_ {
        self.ch.iter().enumerate().filter(|(_, s)| s.open).map(|(i, _)| i)
    }

    pub fn is_open(&self, ch: usize) -> bool {
        self.ch.get(ch).map(|s| s.open).unwrap_or(false)
    }

    /// Whether a burst was open at any point during the last block, rather
    /// than at the instant the block ended.
    ///
    /// This is the question a caller gating work per block actually has. Most
    /// ISM transmissions are shorter than one block, so asking [`Self::is_open`]
    /// after the fact reports a channel as idle precisely because its burst
    /// began and ended inside the block that was being judged.
    pub fn was_open(&self, ch: usize) -> bool {
        self.ch.get(ch).map(|s| s.seen_open || s.open).unwrap_or(false)
    }

    pub fn floor_db(&self, ch: usize) -> f32 {
        self.ch.get(ch).map(|s| 10.0 * s.floor.floor().max(1e-30).log10()).unwrap_or(f32::NAN)
    }

    /// Peak power in dB seen on each channel since the last call, then clear.
    ///
    /// This is what a waterfall or a band scan should display. A mean would
    /// hide exactly the short bursts that matter.
    pub fn drain_peak_hold_db(&mut self, out: &mut Vec<f32>) {
        out.clear();
        out.extend(self.ch.iter_mut().map(|s| {
            let p = 10.0 * s.peak_hold.max(1e-30).log10();
            s.peak_hold = 0.0;
            p
        }));
    }

    /// Peak SNR in dB over the tracked floor since the last call, then clear.
    pub fn drain_peak_snr_db(&mut self, out: &mut Vec<f32>) {
        out.clear();
        out.extend(self.ch.iter_mut().map(|s| {
            let f = s.floor.floor();
            let v = if f.is_nan() {
                f32::NAN
            } else {
                10.0 * (s.peak_hold.max(1e-30) / f.max(1e-30)).log10()
            };
            s.peak_hold = 0.0;
            v
        }));
    }

    pub fn reset(&mut self) {
        for s in &mut self.ch {
            s.floor.reset();
            s.smoothed = 0.0;
            s.seen = 0;
            s.open = false;
            s.peak_hold = 0.0;
            s.seen_open = false;
            s.since_active = u64::MAX;
        }
        self.frame = 0;
        self.finished.clear();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use common::C32;

    fn cfg() -> DetectorConfig {
        DetectorConfig {
            open_db: 10.0,
            close_db: 6.0,
            hang_frames: 5,
            min_frames: 2,
            floor_sub_len: 8,
            floor_sub_count: 4,
            integrate_frames: 8,
            blank_ratio: Some(10.0),
        }
    }

    /// Deterministic pseudo-noise; a real RNG dependency is not worth it here.
    struct Lcg(u64);
    impl Lcg {
        fn next_f32(&mut self) -> f32 {
            self.0 = self.0.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
            ((self.0 >> 33) as f32 / (1u64 << 31) as f32) - 0.5
        }
        fn noise(&mut self, amp: f32) -> C32 {
            C32::new(self.next_f32() * amp, self.next_f32() * amp)
        }
    }

    #[test]
    fn floor_ignores_a_mostly_occupied_channel() {
        // A carrier present 80% of the time. An averaging estimator would
        // learn the carrier as the floor; minimum statistics must not.
        let mut nf = NoiseFloor::new(16, 8);
        let mut last = 0.0;
        for i in 0..2000 {
            let p = if i % 10 < 8 { 100.0 } else { 1.0 };
            last = nf.update(p);
        }
        assert!(last < 10.0, "floor was dragged up to {last}");
    }

    #[test]
    fn floor_tracks_a_change_in_gain() {
        let mut nf = NoiseFloor::new(8, 4);
        for _ in 0..500 {
            nf.update(1.0);
        }
        let before = nf.floor();
        for _ in 0..500 {
            nf.update(100.0);
        }
        let after = nf.floor();
        assert!(after > before * 50.0, "floor did not follow: {before} -> {after}");
    }

    #[test]
    fn detects_a_burst_and_reports_snr() {
        let mut d = Detector::new(1, cfg());
        let mut rng = Lcg(1);
        // Settle the floor on noise alone.
        for _ in 0..200 {
            d.push_frame(&[rng.noise(0.01)]);
        }
        // 50 frames of signal, roughly 30 dB up.
        for _ in 0..50 {
            d.push_frame(&[C32::new(0.3, 0.0) + rng.noise(0.01)]);
        }
        // Silence long enough to close, past the hang time.
        for _ in 0..50 {
            d.push_frame(&[rng.noise(0.01)]);
        }

        let bursts = d.take_bursts();
        assert_eq!(bursts.len(), 1, "expected exactly one burst, got {bursts:?}");
        let b = &bursts[0];
        assert_eq!(b.channel, 0);
        assert!(b.peak_snr_db > 20.0, "peak SNR only {}", b.peak_snr_db);
        // Measured duration is biased short, and unavoidably so: the
        // integrator has to ramp before the level crosses `open_db`, which
        // costs the leading edge several frames. Roughly `integrate_frames` of
        // a 50-frame burst is lost. Anything reading these durations as exact
        // on-air times will be wrong by about that much.
        let dur = b.duration_frames().unwrap();
        assert!((25..=70).contains(&dur), "duration {dur} frames, expected 25..=70");
    }

    #[test]
    fn hysteresis_prevents_chatter_at_the_threshold() {
        let mut d = Detector::new(1, cfg());
        let mut rng = Lcg(7);
        for _ in 0..200 {
            d.push_frame(&[rng.noise(0.01)]);
        }
        // A signal wobbling right around the open threshold. Without
        // hysteresis and hang time this fragments into many bursts.
        for i in 0..200 {
            let a = if i % 2 == 0 { 0.05 } else { 0.02 };
            d.push_frame(&[C32::new(a, 0.0) + rng.noise(0.01)]);
        }
        for _ in 0..50 {
            d.push_frame(&[rng.noise(0.01)]);
        }
        let bursts = d.take_bursts();
        assert!(bursts.len() <= 2, "chattered into {} bursts", bursts.len());
    }

    #[test]
    fn impulse_noise_is_rejected_by_min_frames() {
        let mut d = Detector::new(1, cfg());
        let mut rng = Lcg(3);
        for _ in 0..200 {
            d.push_frame(&[rng.noise(0.01)]);
        }
        // A single-frame spike.
        d.push_frame(&[C32::new(1.0, 0.0)]);
        for _ in 0..60 {
            d.push_frame(&[rng.noise(0.01)]);
        }
        assert!(d.take_bursts().is_empty(), "impulse was reported as a burst");
    }

    #[test]
    fn a_burst_inside_one_block_is_still_reported_as_open() {
        // The question a caller gating per block has to ask. Most ISM
        // transmissions are shorter than a block, so `is_open` after the fact
        // says idle and the work gets skipped precisely when it was needed.
        let mut d = Detector::new(1, DetectorConfig::default());
        let mut lane: Vec<common::C32> = vec![common::C32::new(0.01, 0.0); 4_000];
        for s in lane.iter_mut().skip(2_000).take(400) {
            *s = common::C32::new(1.0, 0.0);
        }
        d.process_lanes(&[lane]);

        assert!(!d.is_open(0), "the burst ended well before the block did");
        assert!(d.was_open(0), "a burst inside the block went unreported");

        // And the flag is per block, not sticky.
        d.process_lanes(&[vec![common::C32::new(0.01, 0.0); 4_000]]);
        assert!(!d.was_open(0), "the flag outlived the block it described");
    }

    #[test]
    fn peak_hold_survives_a_single_loud_frame() {
        let mut d = Detector::new(2, cfg());
        for _ in 0..99 {
            d.push_frame(&[C32::new(0.001, 0.0), C32::new(0.001, 0.0)]);
        }
        d.push_frame(&[C32::new(1.0, 0.0), C32::new(0.001, 0.0)]);

        let mut peaks = Vec::new();
        d.drain_peak_hold_db(&mut peaks);
        assert!(peaks[0] > -1.0, "peak hold lost the spike: {}", peaks[0]);
        assert!(peaks[1] < -50.0, "quiet channel reported {}", peaks[1]);

        // Draining resets, so the next window starts clean.
        d.push_frame(&[C32::new(0.001, 0.0), C32::new(0.001, 0.0)]);
        d.drain_peak_hold_db(&mut peaks);
        assert!(peaks[0] < -50.0, "peak hold did not reset: {}", peaks[0]);
    }

    #[test]
    fn channels_are_independent() {
        let mut d = Detector::new(4, cfg());
        let mut rng = Lcg(11);
        for _ in 0..200 {
            let f: Vec<C32> = (0..4).map(|_| rng.noise(0.01)).collect();
            d.push_frame(&f);
        }
        for _ in 0..30 {
            let mut f: Vec<C32> = (0..4).map(|_| rng.noise(0.01)).collect();
            f[2] = C32::new(0.3, 0.0);
            d.push_frame(&f);
        }
        assert_eq!(d.active().collect::<Vec<_>>(), vec![2]);
    }
}
