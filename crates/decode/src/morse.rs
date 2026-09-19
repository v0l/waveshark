//! Morse, both ways: text to mark/gap timings and timings back to text.
//!
//! The encoder is here rather than in a transmit crate because it is the
//! decoder's own table read backwards, and a table that exists twice drifts.
//! Timing is the standard PARIS convention: a dot is 1200/wpm milliseconds,
//! a dash is three dots, elements inside a character are separated by one
//! dot, characters by three, and words by seven.

use common::Decoded;
use common::{Package, Pulse};
use dsp::cw::CwConfig;

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
        if words > 0
            && let Some(p) = pulses.last_mut()
        {
            p.gap = dot * 7;
        }
        words += 1;
        let mut chars = 0usize;
        for ch in word.chars() {
            let Some(pat) = pattern(ch) else { continue };
            if chars > 0
                && let Some(p) = pulses.last_mut()
            {
                p.gap = dot * 3;
            }
            chars += 1;
            for el in pat.chars() {
                pulses.push(Pulse { mark: if el == '-' { dot * 3 } else { dot }, gap: dot });
            }
        }
    }
    if let Some(p) = pulses.last_mut() {
        p.gap = dot * 7;
    }
    Package { pulses, ..Default::default() }
}

/// The dot length a burst was sent at, in microseconds.
///
/// Measured from the burst rather than given, because a received
/// transmission is at whatever speed the operator was sending at, and that
/// is rarely the speed the receiver expected. The shortest mark is the dot:
/// dashes are three times longer, and a burst with no dots at all (an
/// unbroken run of dashes) is rare enough to be worth getting wrong.
pub fn dot_of(pkg: &Package) -> u32 {
    pkg.pulses.iter().map(|p| p.mark).min().unwrap_or(0).max(1)
}

/// The speed a dot length is, in words a minute. The inverse of [`dot_us`].
pub fn wpm(dot_us: u32) -> f32 {
    1_200_000.0 / dot_us.max(1) as f32
}

/// How much of a burst sits on the 1:3:7 grid a keyed letter is made of.
///
/// Morse carries no check of any kind, so this is the only evidence there is
/// that a burst was sent by somebody rather than assembled out of noise and
/// filter skirts. A mark is a dot or a dash and a gap is one, three or seven
/// dots; a timing that is none of those, at the dot length the burst itself
/// measures, was not keyed by a person.
///
/// The last gap is the silence that ended the burst and says nothing.
pub fn fits(pkg: &Package) -> f32 {
    if pkg.pulses.len() < 2 {
        return 0.0;
    }
    let dot = dot_of(pkg) as f32;
    // Measured on a synthetic 18 wpm over: a station in the channel scores
    // 1.0 even with a fist skewed 15% each way, and a strong station a
    // kilohertz outside it, heard as blips through the filter skirt, scores
    // 0.52 to 0.72. A tighter tolerance than 40% starts costing the fist.
    let near = |v: f32, of: &[f32]| of.iter().any(|g| (v - g).abs() / g <= 0.4);
    let mut good = 0usize;
    let mut total = 0usize;
    for (i, p) in pkg.pulses.iter().enumerate() {
        total += 1;
        good += usize::from(near(p.mark as f32 / dot, &[1.0, 3.0]));
        if i + 1 < pkg.pulses.len() {
            total += 1;
            good += usize::from(near(p.gap as f32 / dot, &[1.0, 3.0, 7.0]));
        }
    }
    good as f32 / total as f32
}

/// Read timings back as text.
pub fn decode(pkg: &Package) -> String {
    if pkg.pulses.is_empty() {
        return String::new();
    }
    let dot = dot_of(pkg) as f32;
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

/// One row: what was sent, and how fast.
pub fn decoded(bytes: &[u8], center: common::Hz) -> Option<Decoded> {
    if bytes.len() <= ENVELOPE || bytes[..TAG.len()] != TAG {
        return None;
    }
    let dot_us = u32::from_le_bytes(bytes[TAG.len()..ENVELOPE].try_into().ok()?);
    let text = String::from_utf8_lossy(&bytes[ENVELOPE..]).to_string();
    let wpm = wpm(dot_us);
    let fields = vec![
        ("speed".into(), common::Value::Float(wpm as f64)),
        ("dot_ms".into(), common::Value::Float(dot_us as f64 / 1000.0)),
        ("message".into(), common::Value::Text(text.clone())),
    ];
    Some(
        Decoded::bytes("Morse", center, 0.0, bytes.to_vec())
            .with_modulation(common::Modulation::Ook)
            .with_detail(format!("{wpm:.0} wpm"))
            .with_fields(fields)
            // A person sent it to another person, so it belongs beside
            // anything else somebody wrote rather than in the packet list.
            .written()
            .with_text(text),
    )
}

pub const ENVELOPE: usize = TAG.len() + 4;

/// Bytes before the text on the bus: the tag the front end writes and the
/// dot length it measured, little endian.
pub const TAG: [u8; 4] = *b"MORS";

/// What a transmission puts on the bus: the tag, the dot length it was sent
/// at, and the text. `None` where too little of it was Morse at all.
///
/// The check is the timing and the text, because Morse has no other: the
/// elements either sit on the grid a hand produces or they do not, and a
/// pattern of them either is a letter or is not.
pub fn framed(pkg: &common::Package) -> Option<Vec<u8>> {
    if fits(pkg) < MIN_FIT {
        return None;
    }
    let text = decode(pkg);
    let letters = text.chars().filter(|c| !c.is_whitespace()).count();
    if letters < MIN_CHARS {
        return None;
    }
    let known = text.chars().filter(|c| !c.is_whitespace() && *c != '?').count();
    if (known as f32) / (letters as f32) < MIN_KNOWN {
        return None;
    }
    let mut out = Vec::with_capacity(ENVELOPE + text.len());
    out.extend_from_slice(&TAG);
    out.extend_from_slice(&dot_of(pkg).to_le_bytes());
    out.extend_from_slice(text.as_bytes());
    Some(out)
}

/// The pitch range the dial's reach becomes.
pub fn config(pitch_hz: f64, reach_hz: f64) -> CwConfig {
    CwConfig { pitch_hz: (pitch_hz - reach_hz, pitch_hz + reach_hz), ..CwConfig::default() }
}

/// How much of a transmission's timing has to sit on the 1:3:7 grid a hand
/// on a key produces. See [`fits`].
///
/// Measured on a synthetic 18 wpm over at 48 kS/s: a station in the channel
/// scores 1.0 at any speed and with a 15% fist, and a strong station a
/// kilohertz outside the channel, whose keying reaches the envelope through
/// the filter skirt as a string of blips, scores 0.52 at 900 Hz out and 0.72
/// at 1000. Nothing measured sits between 0.72 and 1.0.
pub const MIN_FIT: f32 = 0.8;

/// Characters a transmission must hold before it is published. Two letters
/// is the shortest thing worth showing somebody, and a pair of noise pulses
/// that got past the level test makes one.
pub const MIN_CHARS: usize = 3;

/// How much of a transmission has to be a pattern the table has. A burst
/// read at the wrong pitch or through a fade produces element counts no
/// letter uses, and those come back as `?`.
pub const MIN_KNOWN: f32 = 0.75;

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
        let total: u64 = pkg.pulses.iter().map(|p| p.mark as u64 + p.gap as u64).sum();
        assert_eq!(total, 60_000_000, "PARIS at 1 wpm is not a minute long");
        // And the speed is read back off the timings, which is how a
        // receiver reports what it heard.
        assert_eq!(dot_of(&pkg), 1_200_000);
        assert!((wpm(dot_of(&encode("PARIS", 18.0))) - 18.0).abs() < 0.01);
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

    /// What separates a keyed burst from a burst of noise, since nothing in
    /// Morse checks. The numbers are what the receiver's front end tests
    /// against.
    #[test]
    fn only_timings_on_the_grid_look_like_a_person_sending() {
        assert_eq!(fits(&encode("CQ DE MI0ABC", 18.0)), 1.0);
        let mut fist = encode("CQ DE MI0ABC", 18.0);
        for (i, p) in fist.pulses.iter_mut().enumerate() {
            let skew = if i % 2 == 0 { 1.15 } else { 0.85 };
            p.mark = (p.mark as f32 * skew) as u32;
            p.gap = (p.gap as f32 * skew) as u32;
        }
        assert_eq!(fits(&fist), 1.0, "a hand sending is still on the grid");

        // Blips of random length separated by silences, which is what a
        // strong station outside the channel looks like through the skirt.
        let junk = Package {
            pulses: (0..8)
                .map(|i| Pulse { mark: 12_000 + i * 9_000, gap: 200_000 + i * 40_000 })
                .collect(),
            ..Default::default()
        };
        assert!(fits(&junk) < 0.5, "noise scored {}", fits(&junk));
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
