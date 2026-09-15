//! Digits off a channel, gathered into the sequences somebody dialled.
//!
//! The DSP layer says "a 7 arrived at 12.30 seconds". What that means is a
//! matter of the gaps: digits close together are one sequence, a pause ends
//! it. A radio's PTT-ID is such a sequence, sent at the start of an over, at
//! the end, or both.
//!
//! Nothing here knows about radios. A sequence is digits and a span, and
//! [`Sequence::id`] says whether the digits could be a unit identity: that
//! is the one judgement worth making here, because a repeater control code
//! and an identity are the same tones and only their shape tells them apart.

/// How long a pause ends a sequence, in seconds.
///
/// A radio sends its identity as one run at fifty to a hundred milliseconds a
/// digit; a person dialling a repeater control code is slower. Half a second
/// keeps a dialled pair together and still ends the identity before the
/// speech that follows it.
pub const GAP_S: f64 = 0.5;

/// The digits an identity may be. Three on a Baofeng, four on most
/// commercial radios, five on some fleets. Below three it is a control
/// digit; above eight it is a telephone number.
pub const ID_DIGITS: std::ops::RangeInclusive<usize> = 3..=8;

/// One run of digits, as dialled.
#[derive(Clone, Debug, PartialEq)]
pub struct Sequence {
    /// The keys, in order, as they were read.
    pub digits: String,
    /// Where the first and last of them were in the audio, in seconds.
    pub first_s: f64,
    pub last_s: f64,
    /// The loudest digit of the run, in dBFS.
    pub level_db: f32,
}

impl Sequence {
    /// The identity this is, or `None` if it is not shaped like one.
    ///
    /// Radios bracket the digits with the keys a telephone has not got: a
    /// Kenwood sends `A123D`, a Motorola `*1234#`, a Baofeng the bare digits.
    /// Those wrappers are punctuation, so they come off, and what is left has
    /// to be digits and only digits.
    pub fn id(&self) -> Option<&str> {
        let id = self.digits.trim_matches(|c: char| !c.is_ascii_digit());
        let plain = id.chars().all(|c| c.is_ascii_digit());
        (plain && ID_DIGITS.contains(&id.len())).then_some(id)
    }

    pub fn seconds(&self) -> f64 {
        (self.last_s - self.first_s).max(0.0)
    }
}

/// Gathers digits into sequences, closing one when the gap has run.
///
/// Fed the digits as they arrive and asked, each block, whether the pause has
/// ended a run: the clock is the audio's, so a receiver whose thread stalled
/// does not split a sequence in half.
#[derive(Default)]
pub struct Sequences {
    digits: String,
    first_s: f64,
    last_s: f64,
    level_db: f32,
}

impl Sequences {
    pub fn new() -> Self {
        Self::default()
    }

    /// Add one digit. Returns the sequence it ended, where the gap in front
    /// of it closed one.
    pub fn digit(&mut self, key: char, at_s: f64, level_db: f32) -> Option<Sequence> {
        let ended = (at_s - self.last_s > GAP_S).then(|| self.take()).flatten();
        if self.digits.is_empty() {
            self.first_s = at_s;
            self.level_db = level_db;
        }
        self.digits.push(key);
        self.last_s = at_s;
        self.level_db = self.level_db.max(level_db);
        ended
    }

    /// Whether the run has been quiet long enough to be finished, `now`
    /// being where the audio has reached.
    pub fn settled(&self, now_s: f64) -> Option<Sequence> {
        (!self.digits.is_empty() && now_s - self.last_s > GAP_S).then_some(Sequence {
            digits: self.digits.clone(),
            first_s: self.first_s,
            last_s: self.last_s,
            level_db: self.level_db,
        })
    }

    /// Close whatever is held, whether or not the gap has run: what a
    /// squelch shutting means.
    pub fn take(&mut self) -> Option<Sequence> {
        if self.digits.is_empty() {
            return None;
        }
        let out = Sequence {
            digits: std::mem::take(&mut self.digits),
            first_s: self.first_s,
            last_s: self.last_s,
            level_db: self.level_db,
        };
        self.level_db = 0.0;
        Some(out)
    }

    /// Digits held but not yet settled, for a readout.
    pub fn held(&self) -> &str {
        &self.digits
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn seq(digits: &str) -> Sequence {
        Sequence { digits: digits.into(), first_s: 0.0, last_s: 0.4, level_db: -20.0 }
    }

    /// What is an identity and what is somebody pressing keys.
    #[test]
    fn an_identity_is_digits_and_the_right_number_of_them() {
        assert_eq!(seq("123").id(), Some("123"), "three digits, a Baofeng");
        assert_eq!(seq("4321").id(), Some("4321"));
        // The keys a telephone has not got are the brackets radios send the
        // identity in, so they come off.
        assert_eq!(seq("A123D").id(), Some("123"), "a Kenwood");
        assert_eq!(seq("*1234#").id(), Some("1234"), "a Motorola");
        // And what is not an identity.
        assert_eq!(seq("7").id(), None, "one digit is a control key");
        assert_eq!(seq("12").id(), None);
        assert_eq!(seq("123456789").id(), None, "a telephone number");
        assert_eq!(seq("1A2").id(), None, "a letter in the middle is a code, not an identity");
        assert_eq!(seq("").id(), None);
    }

    /// The gaps decide where one sequence ends and the next begins, and the
    /// clock is the audio's.
    #[test]
    fn the_gap_closes_a_sequence() {
        let mut s = Sequences::new();
        assert_eq!(s.digit('1', 1.00, -20.0), None);
        assert_eq!(s.digit('2', 1.08, -22.0), None);
        assert_eq!(s.digit('3', 1.16, -19.0), None);
        assert_eq!(s.held(), "123");
        // Still inside the gap, so nothing is finished.
        assert_eq!(s.settled(1.5), None);
        let done = s.settled(2.0).expect("the run has ended");
        assert_eq!(done.digits, "123");
        assert_eq!(done.id(), Some("123"));
        assert_eq!(done.first_s, 1.00);
        assert_eq!(done.last_s, 1.16);
        assert!((done.level_db + 19.0).abs() < 1e-6, "the loudest digit: {}", done.level_db);

        // Taken, and the next digit starts a run of its own.
        assert_eq!(s.take().map(|x| x.digits), Some("123".into()));
        assert_eq!(s.take(), None);
        assert_eq!(s.digit('9', 9.0, -30.0), None);
        assert_eq!(s.held(), "9");
    }

    /// A digit arriving after the gap closes the run in front of it, so an
    /// identity at the end of one over and one at the start of the next are
    /// two sequences and not one six-digit nonsense.
    #[test]
    fn a_late_digit_closes_the_run_before_it() {
        let mut s = Sequences::new();
        s.digit('1', 1.0, -20.0);
        s.digit('2', 1.1, -20.0);
        s.digit('3', 1.2, -20.0);
        let ended = s.digit('4', 5.0, -20.0).expect("the gap closed the first run");
        assert_eq!(ended.digits, "123");
        s.digit('5', 5.1, -20.0);
        s.digit('6', 5.2, -20.0);
        assert_eq!(s.settled(6.0).map(|x| x.digits), Some("456".into()));
    }
}
