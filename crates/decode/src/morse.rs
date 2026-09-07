//! Morse, both ways: text to mark/gap timings and timings back to text.
//!
//! The encoder is here rather than in a transmit crate because it is the
//! decoder's own table read backwards, and a table that exists twice drifts.
//! Timing is the standard PARIS convention: a dot is 1200/wpm milliseconds,
//! a dash is three dots, elements inside a character are separated by one
//! dot, characters by three, and words by seven.

use common::{Package, Pulse};

/// Dot length in microseconds at a given speed.
pub fn dot_us(wpm: f32) -> u32 {
    (1_200_000.0 / wpm.max(1.0)) as u32
}

/// Letters, digits and the punctuation a callsign or a distress call needs.
const TABLE: &[(char, &str)] = &[
    ('A', ".-"),
    ('B', "-..."),
    ('C', "-.-."),
    ('D', "-.."),
    ('E', "."),
    ('F', "..-."),
    ('G', "--."),
    ('H', "...."),
    ('I', ".."),
    ('J', ".---"),
    ('K', "-.-"),
    ('L', ".-.."),
    ('M', "--"),
    ('N', "-."),
    ('O', "---"),
    ('P', ".--."),
    ('Q', "--.-"),
    ('R', ".-."),
    ('S', "..."),
    ('T', "-"),
    ('U', "..-"),
    ('V', "...-"),
    ('W', ".--"),
    ('X', "-..-"),
    ('Y', "-.--"),
    ('Z', "--.."),
    ('0', "-----"),
    ('1', ".----"),
    ('2', "..---"),
    ('3', "...--"),
    ('4', "....-"),
    ('5', "....."),
    ('6', "-...."),
    ('7', "--..."),
    ('8', "---.."),
    ('9', "----."),
    ('.', ".-.-.-"),
    (',', "--..--"),
    ('?', "..--.."),
    ('/', "-..-."),
    ('=', "-...-"),
    ('+', ".-.-."),
    ('-', "-....-"),
];

fn pattern(c: char) -> Option<&'static str> {
    let c = c.to_ascii_uppercase();
    TABLE.iter().find(|(k, _)| *k == c).map(|(_, v)| *v)
}

fn letter(pat: &str) -> Option<char> {
    TABLE.iter().find(|(_, v)| *v == pat).map(|(k, _)| *k)
}

/// Turn text into the pulses that key it.
///
/// Characters with no Morse equivalent are skipped rather than substituted:
/// sending something other than what was asked for is worse than sending
/// less. Every mark carries the gap that follows it, so the last pulse's gap
/// is the word gap that ends the transmission.
pub fn encode(text: &str, wpm: f32) -> Package {
    let dot = dot_us(wpm);
    let mut pulses: Vec<Pulse> = Vec::new();
    let mut words = 0usize;

    for word in text.split_whitespace() {
        if words > 0 {
            if let Some(p) = pulses.last_mut() {
                p.gap = dot * 7;
            }
        }
        words += 1;
        let mut chars = 0usize;
        for ch in word.chars() {
            let Some(pat) = pattern(ch) else { continue };
            if chars > 0 {
                if let Some(p) = pulses.last_mut() {
                    p.gap = dot * 3;
                }
            }
            chars += 1;
            for el in pat.chars() {
                pulses.push(Pulse {
                    mark: if el == '-' { dot * 3 } else { dot },
                    gap: dot,
                });
            }
        }
    }
    if let Some(p) = pulses.last_mut() {
        p.gap = dot * 7;
    }
    Package {
        pulses,
        ..Default::default()
    }
}

/// Read timings back as text.
///
/// The dot length is measured from the burst rather than given, because a
/// received transmission is at whatever speed the operator was sending at,
/// and that is rarely the speed the receiver expected. The shortest mark is
/// the dot: dashes are three times longer, and a burst with no dots at all
/// (an unbroken run of dashes) is rare enough to be worth getting wrong.
pub fn decode(pkg: &Package) -> String {
    if pkg.pulses.is_empty() {
        return String::new();
    }
    let dot = pkg.pulses.iter().map(|p| p.mark).min().unwrap_or(1).max(1) as f32;
    let mut out = String::new();
    let mut pat = String::new();

    for (i, p) in pkg.pulses.iter().enumerate() {
        // Halfway between a dot and a dash separates them, which tolerates
        // the roughly 10% error a detector's threshold adds at each edge.
        pat.push(if p.mark as f32 / dot >= 2.0 { '-' } else { '.' });

        let last = i + 1 == pkg.pulses.len();
        let gaps = p.gap as f32 / dot;
        // Midway between one dot and three, and between three and seven.
        if last || gaps >= 2.0 {
            out.push(letter(&pat).unwrap_or('?'));
            pat.clear();
        }
        if !last && gaps >= 5.0 {
            out.push(' ');
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn text_survives_the_round_trip() {
        for text in ["PARIS", "CQ DE EI7XYZ", "SOS", "R 599 599"] {
            let pkg = encode(text, 20.0);
            assert_eq!(decode(&pkg), text, "round trip failed for {text:?}");
        }
    }

    #[test]
    fn the_word_paris_is_one_minute_at_one_word_per_minute() {
        // The definition of the speed unit: PARIS plus its word gap is 50
        // dots, so at 1 wpm it takes 60 seconds. A timing table that gets
        // this wrong is wrong at every speed.
        let pkg = encode("PARIS", 1.0);
        let total: u64 = pkg
            .pulses
            .iter()
            .map(|p| p.mark as u64 + p.gap as u64)
            .sum();
        assert_eq!(total, 60_000_000, "PARIS at 1 wpm is not a minute long");
    }

    #[test]
    fn speed_scales_the_timings_and_nothing_else() {
        let slow = encode("SOS", 10.0);
        let fast = encode("SOS", 20.0);
        assert_eq!(slow.pulses.len(), fast.pulses.len());
        for (a, b) in slow.pulses.iter().zip(&fast.pulses) {
            assert_eq!(a.mark, b.mark * 2);
            assert_eq!(a.gap, b.gap * 2);
        }
    }

    #[test]
    fn a_character_with_no_morse_equivalent_is_skipped_not_guessed() {
        assert_eq!(decode(&encode("A#B", 20.0)), "AB");
    }

    #[test]
    fn timings_a_detector_measured_still_decode() {
        // What comes back off the air is not exact: each edge moves by a few
        // percent as the envelope crosses the threshold.
        let mut pkg = encode("CQ", 20.0);
        for (i, p) in pkg.pulses.iter_mut().enumerate() {
            let skew = if i % 2 == 0 { 1.08 } else { 0.93 };
            p.mark = (p.mark as f32 * skew) as u32;
            p.gap = (p.gap as f32 * skew) as u32;
        }
        assert_eq!(decode(&pkg), "CQ");
    }
}
