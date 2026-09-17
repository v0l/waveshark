//! Keying a transmitter from the level of a voice.
//!
//! The same decision a squelch makes, pointed the other way: a level over a
//! threshold for as long as somebody is talking, held through the gaps
//! between words by a tail. So this is [`crate::squelch::Squelch`] with the
//! measurement taken off the audio about to be modulated, and one thing a
//! squelch has no need of.
//!
//! That thing is anti-trip. The speaker is a metre from the microphone, so
//! the station being listened to keys the transmitter and the transmitter
//! covers the station: a loop that ends when somebody pulls the power.
//!
//! A radio with one microphone and no idea what its speaker is doing has to
//! subtract a guessed replica of the received audio. This one knows exactly,
//! because the speaker is a stage in the same receiver, so it does the
//! certain thing instead and holds the key up for as long as the receiver is
//! playing anything. The two levels are not on one scale anyway: the
//! microphone's has the transmit gain and the limiter on it and the
//! speaker's has the master volume, so a margin between them would be a
//! guess dressed as a measurement. What it costs is that nobody can key over
//! the top of a station being listened to, which on a simplex channel is not
//! something to do.

use crate::squelch::Squelch;

/// How far under the opening threshold the level has to fall before the tail
/// starts running out, in dB.
///
/// Speech measured in blocks of twenty milliseconds swings several decibels
/// inside one word, so a single threshold chatters the key on syllable
/// boundaries: measured on a 20 ms block against recorded speech, 3 dB of
/// hysteresis left the key up for the whole of an utterance where 0 dB
/// dropped it between words.
const HYSTERESIS_DB: f32 = 3.0;

/// The level the speaker has to be over before it counts as playing rather
/// than as an idle output, in dBFS.
///
/// Sixty decibels under full scale is below anything an audio path delivers
/// while nothing is on it and well under the quietest station worth hearing,
/// so a receiver with the squelch closed does not hold the key up all day.
const PLAYING_DB: f32 = -60.0;

/// Level over a threshold for a hold time, with the speaker's own audio
/// taken out of the decision.
pub struct Vox {
    gate: Squelch,
    /// The threshold as the operator set it, in dB, before anti-trip raises
    /// it.
    threshold_db: f32,
    /// Whether the speaker enters the decision at all. Off for a headset,
    /// where nothing coming out of it can reach the microphone.
    anti_trip: bool,
    /// Whether the key is being held up because the receiver is playing.
    held: bool,
}

/// The level of a block, in dBFS, as RMS.
///
/// RMS rather than peak: a peak is one sample of a plosive and moves twenty
/// decibels between blocks, where the RMS of a spoken syllable is steady
/// enough to set a threshold against by eye.
/// Digital silence reads [`SILENCE_DB`] rather than minus infinity: the
/// smoothing this feeds is a difference between two levels, and infinity
/// minus infinity is a NaN that no comparison is true of, which is a key
/// that goes down and never comes up.
pub fn level_db(block: &[f32]) -> f32 {
    if block.is_empty() {
        return SILENCE_DB;
    }
    let sum: f64 = block.iter().map(|v| f64::from(*v) * f64::from(*v)).sum();
    let rms = (sum / block.len() as f64).sqrt();
    if rms <= 0.0 {
        return SILENCE_DB;
    }
    (20.0 * (rms as f32).log10()).max(SILENCE_DB)
}

/// What a block of nothing reads, in dBFS: below anything a converter can
/// deliver and finite.
pub const SILENCE_DB: f32 = -120.0;

impl Vox {
    /// `threshold` is an amplitude in 0..1, which is what the meter beside
    /// the control shows; `tail_ms` is how long the key stays down after the
    /// voice drops under it.
    pub fn new(rate: f64, threshold: f32, tail_ms: f64) -> Self {
        let threshold_db = amplitude_db(threshold);
        // No ramp: this decides whether to key, and the audio it is measuring
        // is passed through untouched.
        let mut gate = Squelch::new(rate, threshold_db, threshold_db - HYSTERESIS_DB, 0.0);
        gate.set_hang_ms(rate, tail_ms);
        // A room with nobody in it reads tens of decibels under any
        // threshold, and holding the key through exactly that is what the
        // tail is for: the squelch's shortcut for a carrier that has dropped
        // would let the key up on the first gap between words.
        gate.set_gone_below_db(f32::INFINITY);
        Self { gate, threshold_db, anti_trip: true, held: false }
    }

    pub fn set_threshold(&mut self, threshold: f32) {
        self.threshold_db = amplitude_db(threshold);
        self.gate.set_thresholds(self.threshold_db, self.threshold_db - HYSTERESIS_DB);
    }

    pub fn set_tail_ms(&mut self, rate: f64, tail_ms: f64) {
        self.gate.set_hang_ms(rate, tail_ms);
    }

    /// Whether what the speaker is playing raises the threshold at all.
    pub fn set_anti_trip(&mut self, on: bool) {
        self.anti_trip = on;
    }

    pub fn is_open(&self) -> bool {
        self.gate.is_open()
    }

    /// The smoothed level the decision is made on, in dB: the number a meter
    /// shows so the threshold can be set against something.
    pub fn level_db(&self) -> f32 {
        self.gate.level_db()
    }

    /// Whether the key is being held up because the receiver is playing,
    /// which is the one thing about anti-trip worth showing on a panel.
    pub fn is_held(&self) -> bool {
        self.held
    }

    /// One block of the audio that would be transmitted, and the level of
    /// what the speaker is playing at the same moment, in dBFS. Answers
    /// whether the key should be down.
    pub fn update(&mut self, block: &[f32], heard_db: f32, samples: usize) -> bool {
        self.held = self.anti_trip && heard_db > PLAYING_DB;
        self.gate.mute(self.held);
        self.gate.update(level_db(block), samples)
    }

    pub fn reset(&mut self) {
        self.gate.reset();
    }
}

/// An amplitude in 0..1 as dBFS, with zero meaning "open on anything".
fn amplitude_db(a: f32) -> f32 {
    match a > 0.0 {
        true => 20.0 * a.log10(),
        false => -120.0,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const RATE: f64 = 48_000.0;
    /// The transmit chain's own block, which is what this is fed.
    const BLOCK: usize = 960;

    fn speech(n: usize, amp: f32) -> Vec<f32> {
        (0..n)
            .map(|i| amp * (std::f32::consts::TAU * 400.0 * i as f32 / RATE as f32).sin())
            .collect()
    }

    /// Blocks of one level, then blocks of another, counting where the key
    /// went down and where it came up.
    fn run(vox: &mut Vox, amp: f32, heard_db: f32, blocks: usize) -> (usize, usize) {
        let block = speech(BLOCK, amp);
        let (mut open, mut first) = (0usize, usize::MAX);
        for i in 0..blocks {
            if vox.update(&block, heard_db, BLOCK) {
                open += 1;
                first = first.min(i);
            }
        }
        (first, open)
    }

    /// A voice over the threshold keys within a block or two, and silence
    /// under it does not key at all.
    #[test]
    fn a_voice_keys_and_a_quiet_room_does_not() {
        // 0.1 is -20 dBFS; the voice is -6 dBFS and the room -40.
        let mut vox = Vox::new(RATE, 0.1, 500.0);
        let (first, open) = run(&mut vox, 0.5, f32::NEG_INFINITY, 50);
        assert_eq!(first, 0, "the first block of speech did not key");
        assert_eq!(open, 50, "the key dropped during speech");

        let mut quiet = Vox::new(RATE, 0.1, 500.0);
        let (first, open) = run(&mut quiet, 0.01, f32::NEG_INFINITY, 50);
        assert_eq!(open, 0, "a quiet room keyed the transmitter");
        assert_eq!(first, usize::MAX);
    }

    /// The tail holds the key through a gap between words and lets it up
    /// after it: 500 ms at 20 ms a block is 25 blocks, measured at 24 because
    /// the block the voice stopped in is already under the threshold.
    #[test]
    fn the_tail_holds_the_key_for_as_long_as_it_says() {
        let mut vox = Vox::new(RATE, 0.1, 500.0);
        let (_, spoke) = run(&mut vox, 0.5, f32::NEG_INFINITY, 20);
        assert_eq!(spoke, 20);
        let (_, held) = run(&mut vox, 0.0, f32::NEG_INFINITY, 100);
        assert_eq!(held, 24, "500 ms of tail, in blocks of 20 ms");

        // A tenth of that tail lets go in a tenth of the time.
        let mut short = Vox::new(RATE, 0.1, 50.0);
        let (_, spoke) = run(&mut short, 0.5, f32::NEG_INFINITY, 20);
        assert_eq!(spoke, 20);
        let (_, held) = run(&mut short, 0.0, f32::NEG_INFINITY, 100);
        assert_eq!(held, 2, "50 ms of tail, in blocks of 20 ms");
    }

    /// The station coming out of the speaker, picked up by the microphone,
    /// does not key the transmitter; the same voice with the speaker quiet
    /// does.
    #[test]
    fn the_speaker_does_not_key_the_transmitter() {
        let mut vox = Vox::new(RATE, 0.02, 500.0);
        let heard = level_db(&speech(BLOCK, 0.5));
        let (_, open) = run(&mut vox, 0.16, heard, 50);
        assert_eq!(open, 0, "the receiver's own audio keyed the transmitter");
        assert!(vox.is_held(), "nothing said why the key was up");

        // The station stops and the same level at the microphone keys it.
        let (first, open) = run(&mut vox, 0.16, f32::NEG_INFINITY, 50);
        assert_eq!(first, 0, "a voice in a quiet room did not key");
        assert_eq!(open, 50);
        assert!(!vox.is_held());

        // And with anti-trip off the leak alone keys it: this is the loop
        // the hold exists to stop, not an accident of the levels.
        let mut bare = Vox::new(RATE, 0.02, 500.0);
        bare.set_anti_trip(false);
        let (first, open) = run(&mut bare, 0.16, heard, 50);
        assert_eq!(first, 0);
        assert_eq!(open, 50, "with anti-trip off the loop should close");
    }

    /// Minutes of a quiet room at the microphone key nothing at all.
    #[test]
    fn a_room_left_alone_never_keys() {
        let mut vox = Vox::new(RATE, 0.05, 500.0);
        let mut state = 12_345u32;
        let mut noise = || {
            state = state.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            (state >> 8) as f32 / 8_388_608.0 - 1.0
        };
        // Five minutes at 20 ms a block, the microphone at about -46 dBFS.
        let mut keyed = 0usize;
        for _ in 0..15_000 {
            let block: Vec<f32> = (0..BLOCK).map(|_| 0.005 * noise()).collect();
            if vox.update(&block, f32::NEG_INFINITY, BLOCK) {
                keyed += 1;
            }
        }
        assert_eq!(keyed, 0, "room noise keyed the transmitter {keyed} times in five minutes");
    }
}
