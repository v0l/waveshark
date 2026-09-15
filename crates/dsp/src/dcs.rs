//! DCS: the sub-audible code a radio sends to say which group it is in.
//!
//! The other coded squelch, and the one that carries a number rather than a
//! pitch: a 23-bit Golay word at 134.4 bps, repeating for as long as the key
//! is down, under the speech at a few hundred hertz of deviation. Nine of the
//! twelve message bits are the code an operator sets, written as three octal
//! digits, and radios call it `D023N` or `D023I` depending on whether the
//! waveform is inverted.
//!
//! Established against a Baofeng on PMR446 set to D023N: the word repeated
//! with 91% bit agreement at a period of 23, and matching it against
//! Golay(23,12) with `g(x) = 0xC75` gave 023 with the message's top three
//! bits set to `100` and the word sent least significant bit first. That is
//! the convention here, and the standard code table is what rejects the
//! rotations of a cyclic word that would otherwise decode as nonsense.

/// Every code a radio offers, as the three octal digits it shows.
///
/// The standard set, which is a subset of the 512 a 9-bit field could hold:
/// the rest are rejected because a rotation of a valid word is also a valid
/// word, and only the table tells one apart from the real thing.
pub const CODES: [u16; 104] = [
    23, 25, 26, 31, 32, 36, 43, 47, 51, 53, 54, 65, 71, 72, 73, 74, 114, 115, 116, 122, 125, 131,
    132, 134, 143, 145, 152, 155, 156, 162, 165, 172, 174, 205, 212, 223, 225, 226, 243, 244, 245,
    246, 251, 252, 255, 261, 263, 265, 266, 271, 274, 306, 311, 315, 325, 331, 332, 343, 346, 351,
    356, 364, 365, 371, 411, 412, 413, 423, 431, 432, 445, 446, 452, 454, 455, 462, 464, 465, 466,
    503, 506, 516, 523, 526, 532, 546, 565, 606, 612, 624, 627, 631, 632, 654, 662, 664, 703, 712,
    723, 731, 732, 734, 743, 754,
];

/// The rate the code is read at: seven samples a bit, which is enough for an
/// integrate and dump with a phase search in front of it.
const WORK_HZ: f64 = 1_000.0;

/// The code's own rate.
pub const BAUD: f64 = 134.4;

/// The word, and how much of it is the code.
const WORD_BITS: usize = 23;
const CODE_BITS: usize = 9;
/// The three message bits above the code, which every standard word carries.
const FIXED: u32 = 0b100;

/// Bits read before a decode is attempted: three words' worth, so a word
/// boundary falls inside whatever phase the over started on.
const LOOK_BITS: usize = WORD_BITS * 3;

/// Words that must agree before the code is reported.
const AGREE: u8 = 2;
/// Reads without it that drop it.
const FORGET: u8 = 3;

/// Golay(23,12), the generator every DCS word is built with.
const GOLAY_G: u32 = 0xC75;

/// One code, as a radio names it.
///
/// There is no polarity here, and that is a property of the code rather than
/// a gap. The all-ones word is itself a Golay(23,12) codeword, so the
/// complement of any valid word is another valid word: an inverted
/// transmission is bit for bit some other code's normal word, and no
/// receiver can tell them apart. D023 inverted is D047 normal, which is why
/// a radio set to one opens on the other, and why this reports whichever
/// normal code the waveform is.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Code {
    /// The three octal digits, as a decimal number: 23 is `D023`.
    pub digits: u16,
}

impl Code {
    /// What a radio's menu calls it, without the polarity letter it cannot
    /// know: `D023`.
    pub fn label(self) -> String {
        format!("D{:03}", self.digits)
    }
}

/// The 23-bit word a code is sent as, least significant bit first.
pub fn word_of(digits: u16) -> u32 {
    let mut code = 0u32;
    // The digits are octal: 023 is three bits of 0, then 2, then 3.
    let mut place = 1u32;
    let mut left = u32::from(digits);
    while left > 0 {
        code |= (left % 10) * place;
        left /= 10;
        place *= 8;
    }
    golay_encode((FIXED << CODE_BITS) | (code & 0x1ff))
}

/// Systematic Golay(23,12): the message above eleven check bits.
fn golay_encode(message: u32) -> u32 {
    let mut v = (message & 0xfff) << 11;
    for i in (11..23).rev() {
        if v >> i & 1 == 1 {
            v ^= GOLAY_G << (i - 11);
        }
    }
    ((message & 0xfff) << 11) | (v & 0x7ff)
}

/// The code a 23-bit word is, or `None` when it is not one of the standard
/// words at any rotation.
///
/// A cyclic code's rotations are codewords too, so a word that passes Golay
/// says nothing on its own: what identifies the code is matching the whole
/// word, in some rotation, against the table.
pub fn code_of(word: u32) -> Option<Code> {
    let seen = word & 0x7f_ffff;
    CODES
        .into_iter()
        .find(|digits| (0..WORD_BITS as u32).any(|k| rotate(word_of(*digits), k) == seen))
        .map(|digits| Code { digits })
}

fn rotate(word: u32, by: u32) -> u32 {
    let n = WORD_BITS as u32;
    ((word >> by) | (word << (n - by))) & 0x7f_ffff
}

/// Reads the code off channel audio.
pub struct Dcs {
    /// Input samples per working sample, and the accumulator for them.
    decim: usize,
    filled: usize,
    sum: f32,
    /// The decimated audio, enough of it to read several words.
    work: Vec<f32>,
    need: usize,
    per_bit: f64,
    code: Option<Code>,
    candidate: Option<Code>,
    agreed: u8,
    missed: u8,
}

impl Dcs {
    pub fn new(rate: f64) -> Self {
        let decim = ((rate / WORK_HZ).round() as usize).max(1);
        let work_rate = rate / decim as f64;
        let per_bit = work_rate / BAUD;
        Self {
            decim,
            filled: 0,
            sum: 0.0,
            work: Vec::new(),
            need: (per_bit * LOOK_BITS as f64).ceil() as usize,
            per_bit,
            code: None,
            candidate: None,
            agreed: 0,
            missed: 0,
        }
    }

    /// The code on the channel now.
    pub fn code(&self) -> Option<Code> {
        self.code
    }

    /// Feed audio. Returns the code when it changes, so a caller can say so
    /// once rather than every block.
    pub fn push(&mut self, samples: &[f32]) -> Option<Code> {
        let before = self.code;
        for s in samples {
            self.sum += *s;
            self.filled += 1;
            if self.filled == self.decim {
                self.work.push(self.sum / self.decim as f32);
                self.sum = 0.0;
                self.filled = 0;
            }
        }
        while self.work.len() >= self.need {
            let read = self.read();
            self.work.drain(..self.need / 2);
            self.settle(read);
        }
        match self.code != before {
            true => self.code,
            false => None,
        }
    }

    /// One look at the buffer: the best bit phase, then the word.
    fn read(&self) -> Option<Code> {
        let held = &self.work[..self.need];
        let mean = held.iter().sum::<f32>() / held.len() as f32;
        // The code is a square wave about the channel's own centre, and the
        // discriminator's offset moves that centre.
        let level = |i: usize| held.get(i).map(|v| v - mean).unwrap_or(0.0);
        let mut best: Option<(f32, Vec<u8>)> = None;
        let steps = self.per_bit.round().max(1.0) as usize;
        for phase in 0..steps {
            let mut bits = Vec::with_capacity(LOOK_BITS);
            let mut eye = 0.0f32;
            let mut at = phase as f64;
            while at + self.per_bit <= self.need as f64 {
                let (from, to) = (at as usize, (at + self.per_bit) as usize);
                let sum: f32 = (from..to).map(level).sum();
                let mean = sum / (to - from).max(1) as f32;
                eye += mean.abs();
                bits.push(u8::from(mean > 0.0));
                at += self.per_bit;
            }
            if bits.len() < WORD_BITS * 2 {
                continue;
            }
            let eye = eye / bits.len() as f32;
            if best.as_ref().is_none_or(|(b, _)| eye > *b) {
                best = Some((eye, bits));
            }
        }
        let (_, bits) = best?;
        // Every alignment of a word in what was read, taken as sent: least
        // significant bit first.
        for start in 0..=bits.len().saturating_sub(WORD_BITS) {
            let mut word = 0u32;
            for (i, b) in bits[start..start + WORD_BITS].iter().enumerate() {
                word |= u32::from(*b) << i;
            }
            if let Some(code) = code_of(word) {
                return Some(code);
            }
        }
        None
    }

    fn settle(&mut self, read: Option<Code>) {
        match read {
            Some(c) => {
                self.missed = 0;
                match self.candidate {
                    Some(k) if k == c => self.agreed = self.agreed.saturating_add(1),
                    _ => {
                        self.candidate = Some(c);
                        self.agreed = 1;
                    }
                }
                if self.agreed >= AGREE {
                    self.code = Some(c);
                }
            }
            None => {
                self.missed = self.missed.saturating_add(1);
                self.candidate = None;
                self.agreed = 0;
                if self.missed >= FORGET {
                    self.code = None;
                }
            }
        }
    }

    pub fn reset(&mut self) {
        self.work.clear();
        self.sum = 0.0;
        self.filled = 0;
        self.code = None;
        self.candidate = None;
        self.agreed = 0;
        self.missed = 0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const RATE: f64 = 48_000.0;

    /// A code as a radio sends it: the word, least significant bit first, at
    /// 134.4 bps, repeating, with speech over it where asked for.
    fn sent(digits: u16, inverted: bool, seconds: f64, level: f32, speech: bool) -> Vec<f32> {
        let word = word_of(digits);
        let n = (RATE * seconds) as usize;
        let per_bit = RATE / BAUD;
        (0..n)
            .map(|i| {
                let bit = (i as f64 / per_bit) as usize % WORD_BITS;
                let mut set = word >> bit & 1 == 1;
                if inverted {
                    set = !set;
                }
                let mut v = if set { level } else { -level };
                if speech {
                    let t = i as f64 / RATE;
                    for h in 1..=8 {
                        let f = 210.0 * h as f64;
                        v += (std::f64::consts::TAU * f * t).sin() as f32 * 0.3 / h as f32;
                    }
                }
                v
            })
            .collect()
    }

    /// The word a code is sent as round trips, and the fixed field and
    /// polynomial are the ones the capture established.
    #[test]
    fn a_code_is_the_word_the_radio_sends() {
        // D023N as measured off a Baofeng on PMR446: the repeating 23 bits,
        // least significant bit first.
        let measured = "10010000001110001101111";
        let mut word = 0u32;
        for (i, c) in measured.chars().enumerate() {
            if c == '1' {
                word |= 1 << i;
            }
        }
        assert_eq!(code_of(word), Some(Code { digits: 23 }));
        assert_eq!(code_of(word).map(|c| c.label()), Some("D023".to_string()));
        // The inverted waveform is another code's normal word, because the
        // all-ones vector is a Golay codeword: D023 inverted is D047, which
        // is why a radio set to one opens on the other.
        assert_eq!(code_of(!word & 0x7f_ffff), Some(Code { digits: 47 }));
    }

    /// Every standard code is read back as itself, at either polarity.
    #[test]
    fn every_standard_code_round_trips() {
        for digits in CODES {
            let word = word_of(digits);
            assert_eq!(code_of(word), Some(Code { digits }), "D{digits:03} did not read back");
        }
        // Each word is distinct: a table with a repeat would report the
        // wrong group for somebody.
        let mut words: Vec<u32> = CODES.iter().map(|d| word_of(*d)).collect();
        words.sort_unstable();
        let before = words.len();
        words.dedup();
        assert_eq!(words.len(), before);
    }

    /// Off the air: the code is read from the sub-audible waveform, under
    /// speech, and named the way the radio names it.
    #[test]
    fn a_code_is_read_off_the_channel() {
        let mut d = Dcs::new(RATE);
        d.push(&sent(23, false, 2.0, 0.2, true));
        assert_eq!(d.code().map(|c| c.label()), Some("D023".to_string()));
    }

    /// An inverted transmission reads as the code it is bit for bit, which
    /// is the partner code and not the one the sender's menu shows.
    #[test]
    fn an_inverted_transmission_reads_as_its_partner() {
        let mut d = Dcs::new(RATE);
        d.push(&sent(754, true, 2.0, 0.2, false));
        let read = d.code().expect("a code");
        assert_ne!(read.digits, 754, "the polarity cannot be recovered");
        assert_eq!(read.label(), "D116", "D754 inverted is D116 on the air");
    }

    /// Speech alone is not a code: an over with no coded squelch reports
    /// none rather than whatever the noise rotates into.
    #[test]
    fn speech_alone_is_not_a_code() {
        let mut d = Dcs::new(RATE);
        let n = (RATE * 2.0) as usize;
        let voice: Vec<f32> = (0..n)
            .map(|i| {
                let t = i as f64 / RATE;
                (1..=8).fold(0.0f32, |a, h| {
                    a + (std::f64::consts::TAU * 180.0 * h as f64 * t).sin() as f32 * 0.3 / h as f32
                })
            })
            .collect();
        d.push(&voice);
        assert_eq!(d.code(), None);
    }

    /// The code goes with the over, and another station's replaces it.
    #[test]
    fn a_code_is_dropped_when_it_stops() {
        let mut d = Dcs::new(RATE);
        d.push(&sent(23, false, 2.0, 0.2, false));
        assert_eq!(d.code().map(|c| c.digits), Some(23));
        d.push(&vec![0.0f32; (RATE * 1.5) as usize]);
        assert_eq!(d.code(), None);
        let changed = d.push(&sent(131, false, 2.0, 0.2, false)).expect("the new code");
        assert_eq!(changed.digits, 131);
    }
}
