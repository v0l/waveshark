//! Automatic gain control for audio.
//!
//! An SSB or CW signal arrives with no fixed level: the same station is 40 dB
//! louder when the band opens, and two stations calling in the same minute can
//! differ by as much again. Without gain control the operator rides the volume
//! knob continuously, which is exactly what the control was invented to stop.
//!
//! The shape is the one every communications receiver uses: attack fast enough
//! that a strong signal cannot blast, release slow enough that the gain does
//! not audibly climb during the gaps between words, and a hang time between
//! the two so that a pause in speech is not treated as a fade. The numbers
//! below are the conventional ones, and they are settable because CW and voice
//! genuinely want different ones.

/// Gain control acting on a real audio stream.
pub struct Agc {
    rate: f64,
    envelope: f32,
    gain: f32,
    target: f32,
    max_gain: f32,
    attack: f32,
    release: f32,
    hang_samples: u32,
    hang: u32,
}

impl Agc {
    /// `attack_ms` is how quickly the gain comes down on a loud signal,
    /// `release_ms` how quickly it comes back up, and `hang_ms` how long a
    /// gap is ignored before the release starts at all.
    pub fn new(rate: f64, attack_ms: f64, release_ms: f64, hang_ms: f64) -> Self {
        Self {
            rate,
            envelope: 0.0,
            gain: 1.0,
            // Well below full scale: this is the level speech peaks at, and
            // leaving headroom means an unusually loud syllable during the
            // attack time distorts nothing.
            target: 0.25,
            // 60 dB. Enough to lift a weak signal to a usable level, and low
            // enough that a silent channel does not turn into full volume
            // hiss, which is the failure everyone recognises.
            max_gain: 1_000.0,
            attack: coeff(rate, attack_ms),
            release: coeff(rate, release_ms),
            hang_samples: (rate * hang_ms / 1000.0) as u32,
            hang: 0,
        }
    }

    /// Voice settings: quick to catch a syllable, slow to recover.
    pub fn voice(rate: f64) -> Self {
        Self::new(rate, 5.0, 500.0, 300.0)
    }

    /// CW settings.
    ///
    /// A Morse element is a few tens of milliseconds and the gaps between
    /// them are shorter still, so a voice release would ride the gain up and
    /// down inside a single letter and turn the tone into something uneven.
    /// Hanging through a character and releasing slowly afterwards keeps the
    /// keying sounding like keying.
    pub fn cw(rate: f64) -> Self {
        Self::new(rate, 2.0, 1_000.0, 500.0)
    }

    pub fn set_target(&mut self, level: f32) {
        self.target = level.clamp(0.001, 1.0);
    }

    /// Maximum gain in dB. Also the noise level you accept on a dead channel.
    pub fn set_max_gain_db(&mut self, db: f32) {
        self.max_gain = 10f32.powf(db.clamp(0.0, 100.0) / 20.0);
    }

    pub fn gain_db(&self) -> f32 {
        20.0 * self.gain.max(1e-9).log10()
    }

    pub fn reset(&mut self) {
        self.envelope = 0.0;
        self.gain = 1.0;
        self.hang = 0;
    }

    pub fn rate(&self) -> f64 {
        self.rate
    }

    pub fn process(&mut self, buf: &mut [f32]) {
        // A muted block is left alone entirely, gain and envelope both.
        //
        // Digital silence only comes from a squelch upstream, never from a
        // radio: even a dead band has noise in the last bit. Releasing
        // through it would wind the gain to maximum while the channel is
        // quiet, and then the first syllable after the squelch opens arrives
        // 60 dB too loud for as long as the attack takes. Holding the gain
        // where the last real signal left it is both quieter and a better
        // guess at what the next one needs.
        if buf.iter().all(|s| s.abs() < 1e-6) {
            for s in buf.iter_mut() {
                *s *= self.gain;
            }
            return;
        }
        for s in buf.iter_mut() {
            let a = s.abs();
            if a > self.envelope {
                self.envelope += (a - self.envelope) * self.attack;
                self.hang = self.hang_samples;
            } else if self.hang > 0 {
                self.hang -= 1;
            } else {
                self.envelope += (a - self.envelope) * self.release;
            }
            // The floor is what stops a silent channel from being multiplied
            // by an arbitrarily large number; the clamp is what stops it from
            // being multiplied by a merely very large one.
            let want = self.target / self.envelope.max(1e-6);
            self.gain = want.min(self.max_gain);
            *s *= self.gain;
        }
    }
}

/// One-pole smoothing coefficient for a time constant in milliseconds.
fn coeff(rate: f64, ms: f64) -> f32 {
    if ms <= 0.0 {
        return 1.0;
    }
    (1.0 - (-1.0 / (rate * ms / 1000.0)).exp()) as f32
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::TAU;

    const RATE: f64 = 48_000.0;

    fn tone(amp: f32, secs: f64) -> Vec<f32> {
        let n = (RATE * secs) as usize;
        (0..n).map(|i| amp * (TAU * 700.0 * i as f64 / RATE).sin() as f32).collect()
    }

    fn peak(buf: &[f32]) -> f32 {
        buf.iter().fold(0.0f32, |m, v| m.max(v.abs()))
    }

    fn db(v: f32) -> f32 {
        20.0 * v.max(1e-12).log10()
    }

    #[test]
    fn a_quiet_signal_is_brought_up_to_the_target() {
        let mut agc = Agc::voice(RATE);
        let mut buf = tone(0.001, 2.0);
        agc.process(&mut buf);
        let settled = peak(&buf[buf.len() / 2..]);
        assert!(
            (db(settled) - db(0.25)).abs() < 1.5,
            "a signal 48 dB below the target settled at {:.1} dBFS",
            db(settled)
        );
    }

    #[test]
    fn a_loud_signal_is_brought_down_within_the_attack_time() {
        let mut agc = Agc::voice(RATE);
        let mut buf = tone(1.0, 0.1);
        agc.process(&mut buf);
        // Five milliseconds of attack, so by twenty the gain must be there.
        let after = &buf[(RATE * 0.02) as usize..];
        assert!(db(peak(after)) < db(0.25) + 2.0, "still at {:.1} dBFS 20 ms in", db(peak(after)));
    }

    #[test]
    fn the_gain_does_not_climb_through_a_gap_between_words() {
        // The audible failure this prevents: hiss swelling up in every pause,
        // then being slammed back down when the speaker starts again.
        let mut agc = Agc::voice(RATE);
        agc.process(&mut tone(0.05, 1.0));
        let speaking = agc.gain_db();

        let mut gap = vec![0.0f32; (RATE * 0.2) as usize];
        agc.process(&mut gap);
        let after = agc.gain_db();
        assert!(
            after - speaking < 6.0,
            "the gain rose {:.1} dB during a 200 ms pause",
            after - speaking
        );
    }

    #[test]
    fn a_dead_channel_does_not_become_full_volume_hiss() {
        let mut agc = Agc::voice(RATE);
        agc.set_max_gain_db(40.0);
        let mut buf: Vec<f32> = (0..RATE as usize)
            .map(|i| 0.0001 * ((i * 2654435761) as f32 / u32::MAX as f32 - 0.5))
            .collect();
        agc.process(&mut buf);
        assert!(agc.gain_db() <= 40.5, "gain reached {:.1} dB on noise alone", agc.gain_db());
    }

    #[test]
    fn a_muted_channel_does_not_wind_the_gain_up_to_maximum() {
        // What the listener would otherwise hear: the squelch opens on a
        // station and the first syllable arrives at full gain, blasting until
        // the attack catches up.
        let mut agc = Agc::voice(RATE);
        agc.process(&mut tone(0.2, 0.5));
        let on_signal = agc.gain_db();

        // Five seconds of the squelch holding the channel shut.
        agc.process(&mut vec![0.0f32; (RATE * 5.0) as usize]);
        assert!(
            (agc.gain_db() - on_signal).abs() < 0.1,
            "the gain drifted from {on_signal:.1} to {:.1} dB while muted",
            agc.gain_db()
        );
    }

    #[test]
    fn cw_holds_its_gain_through_a_character() {
        // Morse at 20 words a minute is 60 ms a dit, with 60 ms gaps. If the
        // gain moves inside that, the tone comes out lumpy.
        let mut agc = Agc::cw(RATE);
        agc.process(&mut tone(0.02, 0.5));
        let keyed = agc.gain_db();
        agc.process(&mut vec![0.0f32; (RATE * 0.06) as usize]);
        assert!(
            (agc.gain_db() - keyed).abs() < 1.0,
            "the gain moved {:.1} dB inside one character",
            agc.gain_db() - keyed
        );
    }
}
