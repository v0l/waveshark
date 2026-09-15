//! CTCSS: the sub-audible tone a radio sends to say which group it is in.
//!
//! One tone from a closed set of 50, under the speech at 67 to 254 Hz and a
//! few hundred hertz of deviation. It is not an identity, it is a group: two
//! users of one frequency with different tones cannot hear each other, so the
//! tone is what tells their traffic apart on a channel that carries both.
//!
//! Measured the way the tones are spaced rather than the way audio usually
//! is. Neighbouring tones are 2.3 Hz apart at the bottom of the set, so the
//! window has to be half a second long to tell 67.0 from 69.3, and the audio
//! is decimated to a kilohertz first because nothing above 300 Hz matters
//! here and a Goertzel over 50 tones is 50 times the work at 48 kHz.
//!
//! DCS, the other coded squelch, is a 134.4 bps code rather than a tone and
//! is not read here.

/// The standard tones, in hertz, as every radio's menu lists them.
///
/// The EIA set plus the extras the cheap handhelds add. Closed, ordered, and
/// the index is what a radio's codeplug stores, so a reading is one of these
/// and not a frequency.
pub const TONES: [f32; 50] = [
    67.0, 69.3, 71.9, 74.4, 77.0, 79.7, 82.5, 85.4, 88.5, 91.5, 94.8, 97.4, 100.0, 103.5, 107.2,
    110.9, 114.8, 118.8, 123.0, 127.3, 131.8, 136.5, 141.3, 146.2, 151.4, 156.7, 159.8, 162.2,
    165.5, 167.9, 171.3, 173.8, 177.3, 179.9, 183.5, 186.2, 189.9, 192.8, 196.6, 199.5, 203.5,
    206.5, 210.7, 218.1, 225.7, 229.1, 233.6, 241.8, 250.3, 254.1,
];

/// The rate the tones are measured at. Nothing above 300 Hz matters, and a
/// kilohertz leaves room for the anti-aliasing a boxcar decimation gives.
const WORK_HZ: f64 = 1_000.0;

/// The window one measurement is taken over, in seconds.
///
/// Half a second is set by the closest pair in the set: 67.0 and 69.3 are
/// 2.3 Hz apart, and a shorter window cannot tell them apart however clean
/// the signal is.
pub const WINDOW_S: f64 = 0.5;

/// How far the window moves between measurements.
const HOP_S: f64 = 0.125;

/// How far the winning tone has to stand above the rest of the set, in dB.
///
/// Speech, a DCS code and the discriminator's own noise all put energy down
/// here, so a tone is a tone only when it is clearly the one thing present.
/// Measured against a PMR446 handheld sending 141.3 Hz: the tone stood 14 dB
/// over the mean of the other 49 with somebody talking over it.
const OVER_REST_DB: f64 = 8.0;

/// Windows in a row that must agree before a tone is reported, and windows
/// without it that drop it.
///
/// The tone is what tells a coded squelch from a voice, because a man's
/// pitch is 85 to 180 Hz and sits in the middle of the set: a vowel is a
/// strong line in this band for a moment, and only a line that does not move
/// for half a second is a squelch tone. The windows overlap by a hop, so
/// four of them is that half second.
const AGREE: u8 = 4;
const FORGET: u8 = 3;

/// A tone, as the set knows it.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Tone {
    /// Where it is in [`TONES`], which is what a codeplug stores.
    pub index: usize,
    pub hz: f32,
    /// How far it stood above the rest of the set, in dB.
    pub over_db: f32,
}

impl Tone {
    /// What a radio's menu calls it: one decimal place, always.
    pub fn label(self) -> String {
        format!("{:.1}", self.hz)
    }
}

/// Reads the coded squelch tone off channel audio.
pub struct Ctcss {
    /// Samples of input per working sample, and the accumulator for them.
    decim: usize,
    filled: usize,
    sum: f32,
    /// The decimated audio, one window of it at a time.
    work: Vec<f32>,
    window: usize,
    hop: usize,
    /// The tone reported now, and the agreement behind it.
    tone: Option<Tone>,
    candidate: Option<usize>,
    agreed: u8,
    missed: u8,
}

/// Energy at one frequency over a window, normalised by its length.
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
    (s1 * s1 + s2 * s2 - coeff * s1 * s2).max(0.0) / (samples.len() as f64).powi(2)
}

impl Ctcss {
    pub fn new(rate: f64) -> Self {
        let decim = ((rate / WORK_HZ).round() as usize).max(1);
        let work_rate = rate / decim as f64;
        Self {
            decim,
            filled: 0,
            sum: 0.0,
            work: Vec::new(),
            window: ((work_rate * WINDOW_S) as usize).max(64),
            hop: ((work_rate * HOP_S) as usize).max(1),
            tone: None,
            candidate: None,
            agreed: 0,
            missed: 0,
        }
    }

    /// The tone on the channel now, or `None` while nothing is being sent or
    /// nothing has been heard for long enough to say.
    pub fn tone(&self) -> Option<Tone> {
        self.tone
    }

    /// Feed audio. Returns the tone when it changes, which is what a caller
    /// wanting to say so once needs.
    pub fn push(&mut self, samples: &[f32]) -> Option<Tone> {
        let before = self.tone.map(|t| t.index);
        // Boxcar down to the working rate: the average over `decim` samples
        // is both the decimation and the anti-aliasing this needs, since
        // everything of interest is below 300 Hz.
        for s in samples {
            self.sum += *s;
            self.filled += 1;
            if self.filled == self.decim {
                self.work.push(self.sum / self.decim as f32);
                self.sum = 0.0;
                self.filled = 0;
            }
        }
        while self.work.len() >= self.window {
            let read = self.measure();
            self.work.drain(..self.hop);
            self.settle(read);
        }
        match self.tone.map(|t| t.index) != before {
            true => self.tone,
            false => None,
        }
    }

    /// The strongest tone of the set in the window held, if one stands out.
    fn measure(&self) -> Option<Tone> {
        let rate = WORK_HZ * 1.0;
        let window = &self.work[..self.window];
        let powers: Vec<f64> =
            TONES.iter().map(|hz| goertzel(window, rate, f64::from(*hz))).collect();
        let (best, power) = powers
            .iter()
            .enumerate()
            .fold((0usize, 0.0f64), |a, (i, p)| if *p > a.1 { (i, *p) } else { a });
        if power <= 0.0 {
            return None;
        }
        // Against the rest of the set rather than against the whole band: a
        // person talking fills the audio and would raise any absolute floor
        // with it, while a tone is one line among fifty.
        //
        // The neighbours either side are left out of that mean. A 141.3 Hz
        // tone leaks into 136.5 and 146.2 because the window is not long
        // enough to separate them, and counting the leak as background hid
        // the tone that caused it.
        let rest: Vec<f64> = powers
            .iter()
            .enumerate()
            .filter(|(i, _)| i.abs_diff(best) > 1)
            .map(|(_, p)| *p)
            .collect();
        let mean = rest.iter().sum::<f64>() / rest.len().max(1) as f64;
        let over = 10.0 * (power / mean.max(1e-30)).log10();
        (over >= OVER_REST_DB).then(|| Tone { index: best, hz: TONES[best], over_db: over as f32 })
    }

    /// One window's verdict, folded into what is being reported.
    fn settle(&mut self, read: Option<Tone>) {
        match read {
            Some(t) => {
                self.missed = 0;
                match self.candidate {
                    Some(i) if i == t.index => self.agreed = self.agreed.saturating_add(1),
                    _ => {
                        self.candidate = Some(t.index);
                        self.agreed = 1;
                    }
                }
                if self.agreed >= AGREE {
                    self.tone = Some(t);
                }
            }
            None => {
                self.missed = self.missed.saturating_add(1);
                self.candidate = None;
                self.agreed = 0;
                if self.missed >= FORGET {
                    self.tone = None;
                }
            }
        }
    }

    /// Forget everything: the squelch shut, or the channel was retuned.
    pub fn reset(&mut self) {
        self.work.clear();
        self.sum = 0.0;
        self.filled = 0;
        self.tone = None;
        self.candidate = None;
        self.agreed = 0;
        self.missed = 0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const RATE: f64 = 48_000.0;

    /// `seconds` of a tone at `hz`, at a deviation-like level, with optional
    /// speech over it.
    ///
    /// The speech glides in pitch, because that is the whole difficulty: a
    /// voice puts a strong line in the same band as the tone set and the
    /// only thing that tells them apart is that a squelch tone does not
    /// move.
    fn signal(hz: f64, level: f32, seconds: f64, speech: bool) -> Vec<f32> {
        let n = (RATE * seconds) as usize;
        let mut phase = vec![0.0f64; 9];
        (0..n)
            .map(|i| {
                let t = i as f64 / RATE;
                let mut v = (std::f64::consts::TAU * hz * t).sin() as f32 * level;
                if speech {
                    // 150 Hz up to 230 and back, twice a second, with the
                    // harmonics that go with it.
                    let pitch = 190.0 + 40.0 * (std::f64::consts::TAU * 2.0 * t).sin();
                    for h in 1..=8 {
                        let f = pitch * h as f64;
                        phase[h] += std::f64::consts::TAU * f / RATE;
                        v += phase[h].sin() as f32 * 0.25 / h as f32;
                    }
                }
                v
            })
            .collect()
    }

    /// The tone a handheld sends is read, named the way its menu names it.
    #[test]
    fn a_tone_is_read_and_named_as_a_radio_names_it() {
        let mut c = Ctcss::new(RATE);
        assert_eq!(c.push(&signal(141.3, 0.2, 0.3, false)), None, "half a second is the window");
        let heard = c.push(&signal(141.3, 0.2, 1.5, false)).expect("a tone");
        assert_eq!(heard.hz, 141.3);
        assert_eq!(heard.label(), "141.3");
        assert_eq!(heard.index, 22);
        assert_eq!(c.tone().map(|t| t.hz), Some(141.3));
        // Said once: a caller that wants to know it changed is not told
        // again every block.
        assert_eq!(c.push(&signal(141.3, 0.2, 0.5, false)), None);
    }

    /// Under speech, which is the only way it ever arrives.
    #[test]
    fn a_tone_is_read_under_somebody_talking() {
        let mut c = Ctcss::new(RATE);
        c.push(&signal(88.5, 0.15, 1.5, true));
        assert_eq!(c.tone().map(|t| t.hz), Some(88.5), "the tone was lost under the voice");
    }

    /// Two tones close together are told apart, which is what the long
    /// window is for: the bottom of the set is 2.3 Hz wide.
    #[test]
    fn neighbouring_tones_are_told_apart() {
        for hz in [67.0, 69.3, 71.9] {
            let mut c = Ctcss::new(RATE);
            c.push(&signal(f64::from(hz), 0.2, 1.5, false));
            assert_eq!(c.tone().map(|t| t.hz), Some(hz), "{hz} was read as something else");
        }
    }

    /// Speech alone is not a tone, however loud: an over with no coded
    /// squelch on it must report none rather than the nearest tone to a
    /// vowel.
    #[test]
    fn speech_alone_is_not_a_tone() {
        let mut c = Ctcss::new(RATE);
        c.push(&signal(0.0, 0.0, 2.0, true));
        assert_eq!(c.tone(), None);
    }

    /// The tone ends with the over, and a new one takes over from the last.
    #[test]
    fn a_tone_is_dropped_when_it_stops_and_replaced_when_it_changes() {
        let mut c = Ctcss::new(RATE);
        c.push(&signal(107.2, 0.2, 1.5, false));
        assert_eq!(c.tone().map(|t| t.hz), Some(107.2));
        // Silence, for longer than the windows it takes to forget.
        c.push(&vec![0.0f32; (RATE * 1.0) as usize]);
        assert_eq!(c.tone(), None);
        // Another station, with another tone.
        let changed = c.push(&signal(203.5, 0.2, 1.5, false)).expect("the new tone");
        assert_eq!(changed.hz, 203.5);
    }

    /// The set is the closed one every radio lists, in order.
    #[test]
    fn the_tone_set_is_the_standard_one() {
        assert_eq!(TONES.len(), 50);
        assert!(TONES.windows(2).all(|w| w[1] > w[0]), "the set must be in ascending order");
        assert_eq!(TONES[0], 67.0);
        assert_eq!(TONES[TONES.len() - 1], 254.1);
    }
}
