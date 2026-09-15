//! DTMF: two tones at once, read off audio as digits.
//!
//! The keypad is a grid. One tone from the low group says the row and one
//! from the high group says the column, so a digit is a pair and nothing
//! else in the audio band looks like one. Named for the modulation rather
//! than for what sends it: a radio's PTT-ID, a repeater control sequence and
//! a telephone dialling the same digits are one decoder.
//!
//! Eight Goertzel filters over a short frame, the strongest of each group,
//! and three tests before a frame counts as a digit: both tones well above
//! the frame's own energy floor, the two within a few dB of one another
//! (twist), and nothing else in the band strong enough to be a third tone.
//! Then a digit has to hold for [`MIN_DIGIT_MS`] before it is emitted and
//! release for [`MIN_GAP_MS`] before the same digit can be read again, which
//! is what tells one long 5 from two.

/// The row tones, in hertz, low group first.
pub const LOW: [f64; 4] = [697.0, 770.0, 852.0, 941.0];
/// The column tones. The fourth is the A to D column, which a radio uses and
/// a telephone does not have.
pub const HIGH: [f64; 4] = [1209.0, 1336.0, 1477.0, 1633.0];

/// The keypad, by row and column.
const KEYS: [[char; 4]; 4] =
    [['1', '2', '3', 'A'], ['4', '5', '6', 'B'], ['7', '8', '9', 'C'], ['*', '0', '#', 'D']];

/// The frame the tones are measured over.
///
/// Ten milliseconds is the shortest window that still separates 697 Hz from
/// 770 Hz. The window slides by half of it rather than a whole one, because a
/// 50 ms digit whose edges fall inside two windows gives only three clean
/// looks otherwise, and the first digit of an identity, which arrives as the
/// squelch is still opening, gave two and was thrown away.
pub const FRAME_MS: f64 = 10.0;

/// How far the window moves between looks.
pub const HOP_MS: f64 = FRAME_MS / 2.0;

/// How long a digit must hold before it is read, and how long the pair must
/// go before the same digit can be read again.
///
/// The gap matters as much as the hold: without it a 300 ms 5 is one digit,
/// and with too much of it two deliberate 5s become one.
pub const MIN_DIGIT_MS: f64 = 20.0;
pub const MIN_GAP_MS: f64 = 30.0;

/// How far above the frame's mean energy a tone has to be to count, in dB.
///
/// Measured against speech on a squelched NFM channel: a vowel puts several
/// dB into two of the eight bins at once, and 6 dB let a held "oh" read as a
/// digit. Ten is the least that kept speech out of a whole afternoon of a
/// repeater while still reading a handheld at the edge of the squelch.
const TONE_OVER_FLOOR_DB: f64 = 10.0;

/// The most the two tones of a pair may differ by, in dB.
///
/// A transmitter's pre-emphasis lifts the high tone and a receiver's
/// de-emphasis cuts it, so the pair arrives lopsided however it was sent.
/// The standard allows 8 dB of reverse twist; this is generous because the
/// audio has been through two filters by the time it is read here.
const MAX_TWIST_DB: f64 = 12.0;

/// One digit, and when it was read.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Digit {
    pub key: char,
    /// Seconds from the start of the stream this decoder has been fed.
    pub at_s: f64,
    /// How loud the pair was, as the mean of the two tones, in dBFS.
    pub level_db: f32,
}

/// Energy at one frequency over a frame, by Goertzel: one multiply and two
/// adds a sample, which is why this is not an FFT. Returned as power
/// normalised by the frame length, so frames of different lengths compare.
fn goertzel(samples: &[f32], rate: f64, hz: f64) -> f64 {
    if samples.is_empty() {
        return 0.0;
    }
    let w = std::f64::consts::TAU * hz / rate;
    let coeff = 2.0 * w.cos();
    let (mut s1, mut s2) = (0.0f64, 0.0f64);
    for x in samples {
        let s0 = f64::from(*x) + coeff * s1 - s2;
        s2 = s1;
        s1 = s0;
    }
    let power = s1 * s1 + s2 * s2 - coeff * s1 * s2;
    power.max(0.0) / (samples.len() as f64).powi(2)
}

fn db(power: f64) -> f64 {
    match power > 0.0 {
        true => 10.0 * power.log10(),
        false => -200.0,
    }
}

/// The strongest tone of a group, as (index, power).
fn strongest(powers: &[f64]) -> (usize, f64) {
    powers.iter().enumerate().fold((0, 0.0), |best, (i, p)| match *p > best.1 {
        true => (i, *p),
        false => best,
    })
}

/// Reads digits out of a stream of audio, a frame at a time.
///
/// Fed whatever blocks arrive; it keeps what did not fill a frame. The clock
/// is the samples it has been given, so a digit's time is where it was in the
/// audio and not when the thread got round to it.
/// Where the sub-audible traffic stops and the keypad starts, in hertz.
///
/// A CTCSS tone reaches 250 Hz and a DCS code is a 134.4 bps square wave, and
/// neither is removed anywhere in the audio path: measured on a PMR446 over,
/// the code was the loudest thing in the channel at 0.35 of full scale while
/// the voice band held almost nothing. Left in, it decides what the gain
/// control does and leaves the tones a fraction of the level they were sent
/// at, so it comes off before anything is measured.
const SUBAUDIBLE_HZ: f64 = 300.0;

pub struct Dtmf {
    rate: f64,
    frame: usize,
    hop: usize,
    /// The window being measured, copied out of `held` so the filters and
    /// the buffer are not borrowed at once.
    window: Vec<f32>,
    /// The sub-audible squelch code and whatever DC the discriminator has,
    /// taken off before the tones are looked for.
    highpass: crate::filter::Biquad,
    held: Vec<f32>,
    /// Samples consumed, which is the clock.
    fed: u64,
    /// The pair being held now, and for how many frames.
    holding: Option<(char, usize, f64)>,
    /// The last digit emitted and how many quiet frames since, so a long
    /// press is one digit and two presses are two.
    last: Option<(char, usize)>,
}

impl Dtmf {
    pub fn new(rate: f64) -> Self {
        let frame = ((rate * FRAME_MS / 1000.0) as usize).max(16);
        let hop = ((rate * HOP_MS / 1000.0) as usize).clamp(1, frame);
        let highpass = crate::filter::Biquad::design(
            crate::filter::Response::Highpass,
            rate,
            SUBAUDIBLE_HZ,
            0.707,
        );
        Self {
            rate,
            frame,
            hop,
            window: Vec::new(),
            highpass,
            held: Vec::new(),
            fed: 0,
            holding: None,
            last: None,
        }
    }

    pub fn rate(&self) -> f64 {
        self.rate
    }

    /// Looks a digit must hold, and quiet looks that end one. Counted in
    /// hops, since that is what a look costs.
    fn hold_frames(&self) -> usize {
        ((MIN_DIGIT_MS / HOP_MS).ceil() as usize).max(1)
    }

    fn gap_frames(&self) -> usize {
        ((MIN_GAP_MS / HOP_MS).ceil() as usize).max(1)
    }

    /// What one frame of audio is: a digit, or nothing.
    fn frame_key(&self, frame: &[f32]) -> Option<(char, f64)> {
        let lows: Vec<f64> = LOW.iter().map(|hz| goertzel(frame, self.rate, *hz)).collect();
        let highs: Vec<f64> = HIGH.iter().map(|hz| goertzel(frame, self.rate, *hz)).collect();
        let (row, low_p) = strongest(&lows);
        let (col, high_p) = strongest(&highs);
        if low_p <= 0.0 || high_p <= 0.0 {
            return None;
        }
        // The floor is the frame's own energy outside the pair: speech and
        // noise fill the band, a digit puts everything into two bins.
        let others: Vec<f64> = lows
            .iter()
            .enumerate()
            .filter(|(i, _)| *i != row)
            .chain(highs.iter().enumerate().filter(|(i, _)| *i != col))
            .map(|(_, p)| *p)
            .collect();
        let floor = others.iter().sum::<f64>() / others.len().max(1) as f64;
        let (low_db, high_db, floor_db) = (db(low_p), db(high_p), db(floor));
        if low_db - floor_db < TONE_OVER_FLOOR_DB || high_db - floor_db < TONE_OVER_FLOOR_DB {
            return None;
        }
        if (low_db - high_db).abs() > MAX_TWIST_DB {
            return None;
        }
        // A third tone as strong as the pair is two digits at once, a
        // harmonic, or speech that happened to peak in a bin: not a digit.
        let third = others.iter().copied().fold(0.0f64, f64::max);
        if db(third) > low_db.min(high_db) - TONE_OVER_FLOOR_DB / 2.0 {
            return None;
        }
        Some((KEYS[row][col], (low_p + high_p) / 2.0))
    }

    /// Feed audio, and take whatever digits it completed.
    pub fn push(&mut self, samples: &[f32]) -> Vec<Digit> {
        let mut out = Vec::new();
        for s in samples {
            let filtered = self.highpass.process(*s);
            self.held.push(filtered);
        }
        while self.held.len() >= self.frame {
            self.window.clear();
            self.window.extend_from_slice(&self.held[..self.frame]);
            self.held.drain(..self.hop);
            self.fed += self.hop as u64;
            let at_s = self.fed as f64 / self.rate;
            let window = std::mem::take(&mut self.window);
            let read = self.frame_key(&window);
            self.window = window;
            match read {
                Some((key, power)) => {
                    let (held_key, frames, peak) = self.holding.take().unwrap_or((key, 0, 0.0f64));
                    let (frames, peak) = match held_key == key {
                        true => (frames + 1, peak.max(power)),
                        false => (1, power),
                    };
                    self.holding = Some((key, frames, peak));
                    let repeat = self
                        .last
                        .is_some_and(|(last, quiet)| last == key && quiet < self.gap_frames());
                    if frames == self.hold_frames() && !repeat {
                        out.push(Digit {
                            key,
                            at_s,
                            level_db: (10.0 * peak.max(1e-20).log10()) as f32,
                        });
                        self.last = Some((key, 0));
                    }
                    // A digit still being held is not a gap, so the quiet
                    // count stays where it is.
                }
                None => {
                    self.holding = None;
                    if let Some((_, quiet)) = self.last.as_mut() {
                        *quiet += 1;
                    }
                }
            }
        }
        out
    }

    /// Forget what was being held: a squelch that shut, or a retune.
    pub fn reset(&mut self) {
        self.held.clear();
        self.holding = None;
        self.last = None;
        self.highpass.reset();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const RATE: f64 = 8_000.0;

    /// One digit's pair of tones, `ms` long, at `level` each.
    fn tones(key: char, ms: f64, level: f32) -> Vec<f32> {
        let (row, col) = KEYS
            .iter()
            .enumerate()
            .find_map(|(r, cols)| cols.iter().position(|k| *k == key).map(|c| (r, c)))
            .expect("a key on the pad");
        let n = (RATE * ms / 1000.0) as usize;
        (0..n)
            .map(|i| {
                let t = i as f64 / RATE;
                let a = (std::f64::consts::TAU * LOW[row] * t).sin();
                let b = (std::f64::consts::TAU * HIGH[col] * t).sin();
                ((a + b) as f32) * level / 2.0
            })
            .collect()
    }

    fn silence(ms: f64) -> Vec<f32> {
        vec![0.0; (RATE * ms / 1000.0) as usize]
    }

    /// A dialled sequence comes back as the digits that were sent, in order
    /// and once each.
    #[test]
    fn a_dialled_sequence_reads_back_as_itself() {
        let mut d = Dtmf::new(RATE);
        let mut got = String::new();
        for key in "0123456789*#ABCD".chars() {
            for digit in d.push(&tones(key, 60.0, 0.5)) {
                got.push(digit.key);
            }
            for digit in d.push(&silence(60.0)) {
                got.push(digit.key);
            }
        }
        assert_eq!(got, "0123456789*#ABCD");
    }

    /// One long press is one digit, and two presses are two: the gap decides,
    /// which is the whole of telling 55 from a held 5.
    #[test]
    fn a_held_digit_is_one_and_two_presses_are_two() {
        let mut d = Dtmf::new(RATE);
        let long = d.push(&tones('5', 400.0, 0.5));
        assert_eq!(long.len(), 1, "a held key is one digit: {long:?}");
        d.reset();
        let mut n = 0;
        n += d.push(&tones('5', 60.0, 0.5)).len();
        n += d.push(&silence(60.0)).len();
        n += d.push(&tones('5', 60.0, 0.5)).len();
        assert_eq!(n, 2, "two presses of the same key");
    }

    /// A digit shorter than the hold is noise, not a keypress: a click and a
    /// syllable both put energy in a bin for a moment.
    #[test]
    fn a_tone_too_short_to_be_a_digit_is_not_read() {
        let mut d = Dtmf::new(RATE);
        assert!(d.push(&tones('9', 15.0, 0.5)).is_empty());
        assert!(d.push(&silence(60.0)).is_empty());
    }

    /// Speech is not a digit. Read over a second of a vowel-like tone with
    /// harmonics, which is what used to come out of a repeater as 7s.
    #[test]
    fn speech_does_not_read_as_digits() {
        let mut d = Dtmf::new(RATE);
        let n = (RATE * 1.5) as usize;
        let voiced: Vec<f32> = (0..n)
            .map(|i| {
                let t = i as f64 / RATE;
                // A 120 Hz larynx with formants near the DTMF rows, which is
                // the worst case for a pair detector.
                let mut v = 0.0;
                for h in 1..=12 {
                    let f = 120.0 * h as f64;
                    v += (std::f64::consts::TAU * f * t).sin() / h as f64;
                }
                (v * 0.3) as f32
            })
            .collect();
        let read = d.push(&voiced);
        assert!(read.is_empty(), "speech read as {read:?}");
    }

    /// A digit is read through the squelch code it arrives on top of.
    ///
    /// Nothing in the audio path removes CTCSS or DCS, and on a real PMR446
    /// over the code was the loudest thing in the channel: a third of full
    /// scale of 134 Hz square wave under tones a tenth of that. Without the
    /// high pass the pair was a small part of the frame's energy, the gain
    /// control levelled against the code, and nothing read.
    #[test]
    fn a_digit_is_read_through_a_sub_audible_squelch_code() {
        let mut d = Dtmf::new(RATE);
        let pair = tones('4', 80.0, 0.06);
        let mixed: Vec<f32> = pair
            .iter()
            .enumerate()
            .map(|(i, s)| {
                // The code as it arrives, shaped by the transmitter's own
                // filters: 134 Hz at a third of full scale, which is what
                // was measured, and five times the tones on top of it.
                let t = i as f64 / RATE;
                s + ((std::f64::consts::TAU * 134.4 * t).sin() * 0.35) as f32
            })
            .collect();
        let read = d.push(&mixed);
        assert_eq!(read.len(), 1, "the code buried the digit: {read:?}");
        assert_eq!(read[0].key, '4');
    }

    /// The pair has to arrive together. One tone of it, however loud, is a
    /// whistle or a carrier, not a digit.
    #[test]
    fn a_single_tone_is_not_a_digit() {
        let mut d = Dtmf::new(RATE);
        let n = (RATE * 0.3) as usize;
        let one: Vec<f32> = (0..n)
            .map(|i| (std::f64::consts::TAU * 941.0 * i as f64 / RATE).sin() as f32 * 0.5)
            .collect();
        assert!(d.push(&one).is_empty());
    }

    /// Twist: a receiver's de-emphasis cuts the high tone by several dB, so
    /// a lopsided pair still has to read. Beyond the allowance it does not.
    #[test]
    fn a_lopsided_pair_is_read_until_the_twist_is_too_much() {
        let read = |twist_db: f64| {
            let mut d = Dtmf::new(RATE);
            let n = (RATE * 0.08) as usize;
            let gain = 10.0f64.powf(-twist_db / 20.0);
            let pcm: Vec<f32> = (0..n)
                .map(|i| {
                    let t = i as f64 / RATE;
                    let a = (std::f64::consts::TAU * LOW[3] * t).sin();
                    let b = (std::f64::consts::TAU * HIGH[1] * t).sin() * gain;
                    ((a + b) * 0.25) as f32
                })
                .collect();
            d.push(&pcm).len()
        };
        assert_eq!(read(6.0), 1, "six dB of twist is ordinary de-emphasis");
        assert_eq!(read(30.0), 0, "one tone thirty dB down is not a pair");
    }

    /// The digit's time is where it was in the audio, not when it was read:
    /// a decoder fed in one lump and one fed block by block agree.
    #[test]
    fn a_digit_is_timed_by_the_samples_it_arrived_in() {
        let mut whole = Dtmf::new(RATE);
        let mut pcm = silence(500.0);
        pcm.extend(tones('7', 80.0, 0.5));
        let one = whole.push(&pcm);
        assert_eq!(one.len(), 1);
        assert!(one[0].at_s > 0.5 && one[0].at_s < 0.62, "{} s", one[0].at_s);

        let mut blocks = Dtmf::new(RATE);
        let mut got = Vec::new();
        for chunk in pcm.chunks(37) {
            got.extend(blocks.push(chunk));
        }
        assert_eq!(got.len(), 1);
        assert!((got[0].at_s - one[0].at_s).abs() < 0.02, "{:?} {:?}", got[0], one[0]);
    }
}
