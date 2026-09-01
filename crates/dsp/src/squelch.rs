//! Muting a channel that has nothing on it.
//!
//! A receiver left open on a quiet frequency is a hiss generator, and with an
//! AGC in front of it, a loud one. Squelch is what makes it possible to leave
//! a radio on all day.
//!
//! Two measurements are useful, and they fail in different ways. Signal level
//! is obvious and works for any mode, but on a weak signal it either mutes the
//! station or passes the noise. For FM there is a better one: an FM
//! discriminator with no signal on it produces mostly high frequency noise,
//! and a signal, however weak, fills the audio band and pushes that noise
//! down, because an FM receiver captures. Measuring the energy above the
//! speech band against the energy inside it detects a station at the point
//! where it becomes intelligible rather than at some level chosen in advance,
//! which is why every FM radio for the last sixty years has done it this way.

/// The decision part: thresholds, hysteresis, hang, and a ramp.
///
/// Separated from the measurement so the same behaviour serves both, and so
/// the tests can drive it with numbers rather than with signals.
pub struct Squelch {
    open: bool,
    open_at: f32,
    close_at: f32,
    /// The measurement after smoothing, which is what the decision is made on.
    ///
    /// A block's measurement wanders several dB on a marginal signal, and
    /// comparing each one to the threshold is what makes a squelch set near
    /// the signal chatter: one loud block opens it, the hang holds it open for
    /// half a second, and the audio arrives in half-second slabs. Hysteresis
    /// does not help, because the wobble is wider than any sensible gap
    /// between the two thresholds.
    level: f32,
    primed: bool,
    /// Smoothing time constants, in samples. Opening is quick so the first
    /// syllable is not clipped; falling is slower so a dip inside a
    /// transmission does not start the hang counting down.
    rise_samples: f32,
    fall_samples: f32,
    /// Hang counted in samples rather than in calls.
    ///
    /// It used to be in blocks, on the assumption that a block was 1024
    /// samples. Nothing guarantees that: the audio chain hands this node
    /// whatever a radio read produced, five thousand samples at a time on a
    /// 2.3 MS/s stream, and half a second of hang became nearly three. A
    /// squelch that stays open for three seconds after a transmission ends is
    /// a squelch that does not work.
    hang_samples: u64,
    hang: u64,
    /// Where the mute ramp currently sits, 0 muted and 1 open.
    ramp: f32,
    step: f32,
}

impl Squelch {
    /// `open_at` and `close_at` are in dB on whatever the measurement is.
    ///
    /// They differ so that a signal sitting exactly on the threshold does not
    /// chatter the audio on and off, which is far more irritating than either
    /// state. `ramp_ms` is how long the mute takes to open or close: stepping
    /// straight from silence to audio is a click on every transmission.
    pub fn new(rate: f64, open_at: f32, close_at: f32, ramp_ms: f64) -> Self {
        Self {
            open: false,
            open_at,
            close_at: close_at.min(open_at),
            level: 0.0,
            primed: false,
            rise_samples: (rate * 0.010) as f32,
            fall_samples: (rate * 0.025) as f32,
            // Half a second of hang, so a pause for breath in the middle of a
            // transmission does not slam the squelch shut and clip the next
            // word.
            hang_samples: (rate * 0.5) as u64,
            hang: 0,
            ramp: 0.0,
            step: (1.0 / (rate * ramp_ms / 1000.0).max(1.0)) as f32,
        }
    }

    pub fn is_open(&self) -> bool {
        self.open
    }

    pub fn set_thresholds(&mut self, open_at: f32, close_at: f32) {
        self.open_at = open_at;
        self.close_at = close_at.min(open_at);
    }

    /// The smoothed measurement the decision is made on, in dB.
    ///
    /// Worth showing on a meter rather than the raw figure: a control set
    /// against a number that is not the one being compared is a control that
    /// appears to be lying whenever the two disagree.
    pub fn level_db(&self) -> f32 {
        self.level
    }

    /// Feed one block's measurement, and how many samples it covered.
    pub fn update(&mut self, measured_db: f32, samples: usize) -> bool {
        if !self.primed {
            self.level = measured_db;
            self.primed = true;
        } else {
            let tau = if measured_db > self.level { self.rise_samples } else { self.fall_samples };
            // Per block rather than per sample, so the time constant means the
            // same thing whatever size buffer the radio happens to deliver.
            let a = 1.0 - (-(samples as f32) / tau.max(1.0)).exp();
            self.level += (measured_db - self.level) * a;
        }

        if self.level >= self.open_at {
            self.open = true;
            self.hang = self.hang_samples;
        } else if self.level < self.close_at {
            self.hang = self.hang.saturating_sub(samples as u64);
            if self.hang == 0 {
                self.open = false;
            }
        }
        self.open
    }

    /// Apply the current decision to a block, ramping rather than switching.
    pub fn apply(&mut self, buf: &mut [f32]) {
        let want = if self.open { 1.0 } else { 0.0 };
        for s in buf.iter_mut() {
            if self.ramp < want {
                self.ramp = (self.ramp + self.step).min(want);
            } else if self.ramp > want {
                self.ramp = (self.ramp - self.step).max(want);
            }
            *s *= self.ramp;
        }
    }

    pub fn reset(&mut self) {
        self.open = false;
        self.hang = 0;
        self.ramp = 0.0;
        self.primed = false;
    }
}

/// How much of a demodulated FM block is noise above the speech band.
///
/// Returns a figure in dB where higher means more signal, so it feeds the
/// same [`Squelch`] as a level measurement does. It is the inverse of the
/// noise ratio: a quiet channel is all high frequency hiss and reads near
/// 0 dB, a fully quieting signal reads 20 dB or more.
pub struct NoiseMeter {
    /// Two cascaded single pole highpasses, as their lowpass complements.
    ///
    /// One pole is not enough: at 6 dB an octave, an 800 Hz tone is only
    /// 14 dB down at a 4 kHz corner, which measured as 11 dB of separation
    /// between a station and an empty channel. Two poles measure 25 dB, and
    /// the extra state is two floats.
    lp: [f32; 2],
    alpha: f32,
}

impl NoiseMeter {
    /// `rate` is the audio rate and `corner_hz` the frequency above which
    /// everything is assumed to be noise rather than speech. Around 4 kHz for
    /// narrowband FM, which passes 300 Hz to 3 kHz.
    pub fn new(rate: f64, corner_hz: f64) -> Self {
        let rc = 1.0 / (std::f64::consts::TAU * corner_hz);
        let dt = 1.0 / rate;
        Self { lp: [0.0; 2], alpha: (dt / (rc + dt)) as f32 }
    }

    pub fn reset(&mut self) {
        self.lp = [0.0; 2];
    }

    pub fn measure(&mut self, buf: &[f32]) -> f32 {
        if buf.is_empty() {
            return 0.0;
        }
        let mut noise = 0.0f64;
        let mut total = 0.0f64;
        for &s in buf {
            self.lp[0] += (s - self.lp[0]) * self.alpha;
            let first = s - self.lp[0];
            self.lp[1] += (first - self.lp[1]) * self.alpha;
            let hp = first - self.lp[1];
            noise += (hp * hp) as f64;
            total += (s * s) as f64;
        }
        // Guard against a block of exact silence, which is what a muted
        // upstream stage produces and which would otherwise read as a
        // perfect signal.
        if total < 1e-18 {
            return 0.0;
        }
        10.0 * (total / noise.max(1e-18)).log10() as f32
    }
}

/// Mean power of a block in dBFS, for the modes with no capture effect.
pub fn level_db(buf: &[f32]) -> f32 {
    if buf.is_empty() {
        return -120.0;
    }
    let p: f64 = buf.iter().map(|s| (s * s) as f64).sum::<f64>() / buf.len() as f64;
    10.0 * p.max(1e-12).log10() as f32
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::TAU;

    const RATE: f64 = 48_000.0;

    fn noise(n: usize, amp: f32, seed: u32) -> Vec<f32> {
        let mut x = seed | 1;
        (0..n)
            .map(|_| {
                x ^= x << 13;
                x ^= x >> 17;
                x ^= x << 5;
                amp * (x as f32 / u32::MAX as f32 - 0.5) * 2.0
            })
            .collect()
    }

    fn speech(n: usize, amp: f32) -> Vec<f32> {
        (0..n).map(|i| amp * (TAU * 800.0 * i as f64 / RATE).sin() as f32).collect()
    }

    #[test]
    fn an_open_channel_reads_as_noise_and_a_signal_does_not() {
        let mut m = NoiseMeter::new(RATE, 4_000.0);
        let hiss = m.measure(&noise(4096, 0.3, 7));
        m.reset();
        let signal = m.measure(&speech(4096, 0.3));
        assert!(
            signal - hiss > 20.0,
            "a station read {signal:.1} dB against {hiss:.1} dB for an empty channel"
        );
    }

    #[test]
    fn a_weak_signal_still_reads_as_a_signal() {
        // The point of measuring noise rather than level: a station 30 dB
        // quieter is still a station, and a level squelch set to pass the
        // loud one would mute this.
        let mut m = NoiseMeter::new(RATE, 4_000.0);
        let loud = m.measure(&speech(4096, 0.3));
        m.reset();
        let weak = m.measure(&speech(4096, 0.01));
        assert!((loud - weak).abs() < 3.0, "{loud:.1} dB against {weak:.1} dB");
    }

    #[test]
    fn the_squelch_does_not_chatter_on_a_signal_sitting_at_the_threshold() {
        let mut sq = Squelch::new(RATE, 9.0, 6.0, 5.0);
        assert!(sq.update(10.0, 1024), "a signal above the threshold should open it");
        // Wobbling either side of the opening threshold must not close it:
        // that is what the second threshold is for.
        for _ in 0..20 {
            assert!(sq.update(7.0, 1024), "the squelch closed inside the hysteresis");
            assert!(sq.update(9.5, 1024));
        }
    }

    #[test]
    fn the_squelch_hangs_through_a_pause_for_breath() {
        let mut sq = Squelch::new(RATE, 9.0, 6.0, 5.0);
        sq.update(12.0, 1024);
        for _ in 0..(RATE / 1024.0 * 0.4) as usize {
            assert!(sq.update(0.0, 1024), "closed during a short pause");
        }
        for _ in 0..(RATE / 1024.0 * 0.4) as usize {
            sq.update(0.0, 1024);
        }
        assert!(!sq.is_open(), "never closed at all");
    }

    #[test]
    fn the_hang_is_half_a_second_whatever_the_block_size() {
        // It was counted in calls, on the assumption that a call was 1024
        // samples. The audio chain hands this whatever a radio read produced,
        // and half a second of hang quietly became nearly three.
        //
        // The measurement is smoothed before it is compared, so a signal that
        // stops takes a few tens of milliseconds to read as stopped and the
        // total is a little over the hang. What must not vary is the block
        // size, which is what this is really testing.
        for block in [256usize, 1024, 5461, 16384] {
            let mut sq = Squelch::new(RATE, 9.0, 6.0, 5.0);
            sq.update(12.0, block);
            let mut silent = 0.0;
            while sq.is_open() && silent < 2.0 {
                sq.update(0.0, block);
                silent += block as f64 / RATE;
            }
            // The allowance is one block, which is the granularity of the
            // decision, plus the tens of milliseconds the detector takes to
            // read a stopped signal as stopped.
            assert!(
                (silent - 0.5).abs() < block as f64 / RATE + 0.06,
                "a {block} sample block held the squelch open for {silent:.2} s"
            );
        }
    }

    #[test]
    fn a_marginal_signal_does_not_gate_the_audio_in_slabs() {
        // The complaint this exists for: with the threshold set into the
        // signal's own level, one loud block opened the squelch, the hang held
        // it open for half a second, and the audio arrived in half second
        // slabs with hard edges. Smoothing the measurement is what stops a
        // single block deciding anything.
        let mut sq = Squelch::new(RATE, 9.0, 6.0, 5.0);
        let mut opens = 0;
        let mut was = false;
        // A channel wobbling either side of the threshold, as a real one does.
        for i in 0..600 {
            let m = if i % 2 == 0 { 11.0 } else { 4.0 };
            let now = sq.update(m, 1024);
            if now && !was {
                opens += 1;
            }
            was = now;
        }
        assert!(opens <= 1, "the gate opened {opens} times on one marginal signal");
    }

    #[test]
    fn the_level_shown_is_the_level_decided_on() {
        let mut sq = Squelch::new(RATE, 9.0, 6.0, 5.0);
        sq.update(20.0, 1024);
        assert_eq!(sq.level_db(), 20.0, "the first measurement is taken as it is");
        for _ in 0..200 {
            sq.update(0.0, 1024);
        }
        assert!(sq.level_db() < 1.0, "the meter kept reading a signal that had gone");
    }

    #[test]
    fn opening_and_closing_are_ramped_rather_than_switched() {
        // A hard cut is a click, and on a busy channel it is a click every
        // few seconds.
        let mut sq = Squelch::new(RATE, 9.0, 6.0, 5.0);
        sq.update(12.0, 1024);
        let mut buf = vec![1.0f32; 1024];
        sq.apply(&mut buf);
        assert!(buf[0] < 0.02, "the first sample jumped straight to {}", buf[0]);
        let biggest = buf.windows(2).map(|w| (w[1] - w[0]).abs()).fold(0.0f32, f32::max);
        assert!(biggest < 0.01, "a step of {biggest} is audible as a click");
    }
}
