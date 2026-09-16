//! English words into the phonemes Kokoro reads.
//!
//! The model takes phonemes, not letters, so something has to say how a word
//! is pronounced. That something is a dictionary: ninety thousand words with
//! the pronunciation Kokoro was trained against, published beside the model
//! and fetched with it. A word in the dictionary is exact; a word outside it
//! is a guess, and the guess is worth making rather than refusing, because
//! the words that fall through are call signs, place names and the
//! occasional protocol.
//!
//! What is deliberately not here is a part-of-speech tagger. The reference
//! front end runs one to choose between the readings of a homograph, so
//! "lead the way" and "a lead pipe" come out differently. Here the
//! dictionary's default reading is taken, which is wrong for a handful of
//! words a receiver rarely says and costs a parser nobody would otherwise
//! need. Words that were guessed at are reported, so a caller can see what
//! the sentence cost.

use common::{Error, Result};
use std::collections::HashMap;
use std::path::Path;

/// The pronunciation dictionary, and everything that turns a sentence into
/// something the model can read.
pub struct English {
    words: HashMap<String, String>,
}

/// A sentence as phonemes, with what had to be guessed at.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct Said {
    pub phonemes: String,
    /// Words that were not in the dictionary, in the order they appeared.
    pub guessed: Vec<String>,
}

impl English {
    /// Read a misaki lexicon: a JSON object of word to phonemes, where a word
    /// with more than one reading is an object keyed by part of speech.
    pub fn load(path: &Path) -> Result<Self> {
        let text = std::fs::read_to_string(path)?;
        let raw: HashMap<String, serde_json::Value> =
            serde_json::from_str(&text).map_err(|e| Error::other(format!("lexicon: {e}")))?;
        let mut words = HashMap::with_capacity(raw.len());
        for (word, v) in raw {
            let said = match &v {
                serde_json::Value::String(s) => Some(s.clone()),
                // A homograph: several readings by part of speech, of which
                // the default is the one taken here.
                serde_json::Value::Object(m) => {
                    m.get("DEFAULT").and_then(|d| d.as_str()).map(str::to_string)
                }
                _ => None,
            };
            if let Some(said) = said {
                words.insert(word, said);
            }
        }
        Ok(Self { words })
    }

    /// How many words it knows.
    pub fn len(&self) -> usize {
        self.words.len()
    }

    pub fn is_empty(&self) -> bool {
        self.words.is_empty()
    }

    /// A sentence as phonemes.
    ///
    /// Punctuation is kept where the model has a symbol for it, because that
    /// is how it knows to pause and where to put a question's rise.
    pub fn say(&self, text: &str) -> Said {
        let mut out = Said::default();
        for token in tokenise(text) {
            match token {
                Token::Word(w) => {
                    let (said, known) = self.word(&w);
                    if !known {
                        out.guessed.push(w);
                    }
                    push_word(&mut out.phonemes, &said);
                }
                Token::Number(n) => {
                    for w in spell_number(&n) {
                        let (said, _) = self.word(&w);
                        push_word(&mut out.phonemes, &said);
                    }
                }
                Token::Punctuation(c) => out.phonemes.push(c),
            }
        }
        out.phonemes = out.phonemes.trim().to_string();
        out
    }

    /// One word, and whether the dictionary knew it.
    fn word(&self, w: &str) -> (String, bool) {
        for form in [w.to_string(), w.to_lowercase(), title_case(w)] {
            if let Some(said) = self.words.get(&form) {
                return (said.clone(), true);
            }
        }
        // An inflection of a word it does know: the ending carries the
        // pronunciation it always carries, and the stem is looked up.
        if let Some(said) = self.inflected(&w.to_lowercase()) {
            return (said, true);
        }
        // An acronym a receiver is full of: two to four letters with no
        // vowel is said letter by letter, which is what an operator does.
        if is_initialism(w) {
            let mut said = String::new();
            let mut known = true;
            for c in w.chars() {
                match self.words.get(&c.to_uppercase().to_string()) {
                    Some(letter) => push_word(&mut said, letter),
                    None => known = false,
                }
            }
            if known {
                return (said, true);
            }
        }
        (guess(&w.to_lowercase()), false)
    }

    /// A plural, a past tense or a participle of a word in the dictionary.
    fn inflected(&self, w: &str) -> Option<String> {
        for (ending, stems, suffix) in INFLECTIONS {
            let Some(stem) = w.strip_suffix(ending) else { continue };
            for form in stems.iter() {
                let candidate = match *form {
                    "" => stem.to_string(),
                    "e" => format!("{stem}e"),
                    "y" => format!("{stem}y"),
                    // A doubled consonant before the ending: "stopped".
                    "-" => match stem.chars().last() == stem.chars().nth(stem.len() - 2) {
                        true => stem[..stem.len() - 1].to_string(),
                        false => continue,
                    },
                    other => format!("{stem}{other}"),
                };
                if let Some(said) = self.words.get(&candidate) {
                    return Some(format!("{said}{}", voiced_suffix(said, *suffix)));
                }
            }
        }
        None
    }
}

/// Endings that can be taken off a word to find it in the dictionary, the
/// stem forms to try, and what the ending is said as.
///
/// `-` means the stem doubled its last consonant before the ending.
const INFLECTIONS: &[(&str, &[&str], Suffix)] = &[
    ("'s", &["", "e"], Suffix::S),
    ("ies", &["y"], Suffix::S),
    ("es", &["", "e"], Suffix::S),
    ("s", &["", "e"], Suffix::S),
    ("ied", &["y"], Suffix::D),
    ("ed", &["", "e", "-"], Suffix::D),
    ("ing", &["", "e", "-"], Suffix::Ing),
];

#[derive(Clone, Copy, PartialEq, Eq)]
enum Suffix {
    S,
    D,
    Ing,
}

/// What an ending is said as depends on the sound before it: "cats" ends in
/// an s, "dogs" in a z, and "buses" in a syllable of its own. English spells
/// one ending; a speaker says three.
fn voiced_suffix(stem: &str, suffix: Suffix) -> &'static str {
    let last = stem.chars().last().unwrap_or(' ');
    match suffix {
        Suffix::S => match last {
            's' | 'z' | 'ʃ' | 'ʒ' | 'ʧ' | 'ʤ' => "ᵻz",
            'p' | 't' | 'k' | 'f' | 'θ' => "s",
            _ => "z",
        },
        Suffix::D => match last {
            't' | 'd' => "ᵻd",
            'p' | 'k' | 'f' | 's' | 'ʃ' | 'ʧ' | 'θ' => "t",
            _ => "d",
        },
        Suffix::Ing => "ɪŋ",
    }
}

fn title_case(w: &str) -> String {
    let mut c = w.chars();
    match c.next() {
        Some(first) => first.to_uppercase().collect::<String>() + &c.as_str().to_lowercase(),
        None => String::new(),
    }
}

/// Two to five letters, all capitals and no vowel worth the name: `RSSI`,
/// `DMR`, `FM`. Said letter by letter.
fn is_initialism(w: &str) -> bool {
    let letters = w.chars().count();
    (2..=5).contains(&letters)
        && w.chars().all(|c| c.is_ascii_uppercase())
        && !w.chars().any(|c| "AEIOU".contains(c) && letters > 3)
}

enum Token {
    Word(String),
    Number(String),
    Punctuation(char),
}

/// Split a sentence into words, numbers and the punctuation the model reads.
///
/// An apostrophe inside a word is part of it, because the dictionary holds
/// contractions; a hyphen is a word break, because the dictionary holds the
/// halves more often than the whole.
fn tokenise(text: &str) -> Vec<Token> {
    let mut out = Vec::new();
    let mut word = String::new();
    let mut number = String::new();
    let flush = |word: &mut String, number: &mut String, out: &mut Vec<Token>| {
        if !word.is_empty() {
            out.push(Token::Word(std::mem::take(word)));
        }
        if !number.is_empty() {
            out.push(Token::Number(std::mem::take(number)));
        }
    };
    for c in text.chars() {
        match c {
            _ if c.is_ascii_digit() => {
                if !word.is_empty() {
                    out.push(Token::Word(std::mem::take(&mut word)));
                }
                number.push(c);
            }
            '.' if !number.is_empty() => number.push(c),
            _ if c.is_alphabetic() || c == '\'' => {
                if !number.is_empty() {
                    out.push(Token::Number(std::mem::take(&mut number)));
                }
                word.push(c);
            }
            _ => {
                flush(&mut word, &mut number, &mut out);
                match c {
                    ',' | ';' | ':' | '.' | '!' | '?' => out.push(Token::Punctuation(c)),
                    _ => out.push(Token::Punctuation(' ')),
                }
            }
        }
    }
    flush(&mut word, &mut number, &mut out);
    out
}

fn push_word(out: &mut String, said: &str) {
    if !out.is_empty() && !out.ends_with(' ') {
        out.push(' ');
    }
    out.push_str(said);
}

/// A number as the words somebody would say.
///
/// A decimal point is said as "point" and then digit by digit, which is what
/// a frequency is: 433.92 is four three three point nine two, not four
/// hundred and thirty three point ninety two.
fn spell_number(n: &str) -> Vec<String> {
    let (whole, fraction) = match n.split_once('.') {
        Some((a, b)) => (a, Some(b)),
        None => (n, None),
    };
    // A long run of digits, or the whole part of a decimal, is read out
    // digit by digit: that is what a frequency, a call sign or a serial
    // number is, and they are most of the numbers a receiver says.
    let spelled_out = whole.len() > 4 || (fraction.is_some() && whole.len() > 2);
    let mut words = Vec::new();
    match whole.parse::<u64>() {
        Ok(v) if !spelled_out => words.extend(cardinal(v)),
        _ => words.extend(digits(whole)),
    }
    if let Some(f) = fraction {
        words.push("point".into());
        words.extend(digits(f));
    }
    words
}

fn digits(s: &str) -> Vec<String> {
    s.chars().filter_map(|c| c.to_digit(10)).map(|d| DIGITS[d as usize].to_string()).collect()
}

const DIGITS: [&str; 10] =
    ["zero", "one", "two", "three", "four", "five", "six", "seven", "eight", "nine"];
const TEENS: [&str; 10] = [
    "ten",
    "eleven",
    "twelve",
    "thirteen",
    "fourteen",
    "fifteen",
    "sixteen",
    "seventeen",
    "eighteen",
    "nineteen",
];
const TENS: [&str; 10] =
    ["", "", "twenty", "thirty", "forty", "fifty", "sixty", "seventy", "eighty", "ninety"];

fn cardinal(v: u64) -> Vec<String> {
    let mut out = Vec::new();
    match v {
        0 => out.push("zero".into()),
        1..=9 => out.push(DIGITS[v as usize].into()),
        10..=19 => out.push(TEENS[(v - 10) as usize].into()),
        20..=99 => {
            out.push(TENS[(v / 10) as usize].into());
            if !v.is_multiple_of(10) {
                out.push(DIGITS[(v % 10) as usize].into());
            }
        }
        100..=999 => {
            out.push(DIGITS[(v / 100) as usize].into());
            out.push("hundred".into());
            if !v.is_multiple_of(100) {
                out.push("and".into());
                out.extend(cardinal(v % 100));
            }
        }
        _ => {
            out.extend(cardinal(v / 1000));
            out.push("thousand".into());
            if !v.is_multiple_of(1000) {
                out.extend(cardinal(v % 1000));
            }
        }
    }
    out
}

/// A pronunciation for a word the dictionary does not have.
///
/// English spelling is not a code, so this is a guess and is reported as one.
/// The rules are the ones that hold often enough to be worth applying: a
/// final silent e lengthens the vowel before it, a vowel in a closed
/// syllable is short, and the digraphs are fixed. Stress goes on the first
/// syllable, which is where an English noun usually carries it.
fn guess(word: &str) -> String {
    let letters: Vec<char> = word.chars().collect();
    let silent_e = letters.len() > 2
        && letters[letters.len() - 1] == 'e'
        && !"aeiou".contains(letters[letters.len() - 2]);
    let mut out = String::new();
    let mut stressed = false;
    let mut i = 0;
    while i < letters.len() {
        if silent_e && i == letters.len() - 1 {
            break;
        }
        let rest: String = letters[i..].iter().collect();
        let (sound, used) = match DIGRAPHS.iter().find(|(s, _)| rest.starts_with(s)) {
            Some((s, sound)) => (*sound, s.chars().count()),
            None => (letter_sound(letters[i], silent_e && i + 2 == letters.len() - 1), 1),
        };
        if !sound.is_empty() {
            // The stress mark goes before the first vowel, which is where a
            // word with no better information available carries it.
            if !stressed && is_vowel_sound(sound) {
                out.push('ˈ');
                stressed = true;
            }
            out.push_str(sound);
        }
        i += used;
    }
    match out.is_empty() {
        true => "ˈʌ".to_string(),
        false => out,
    }
}

/// Letter pairs whose sound is not the sum of their letters.
const DIGRAPHS: &[(&str, &str)] = &[
    ("tch", "ʧ"),
    ("sch", "sk"),
    ("ch", "ʧ"),
    ("sh", "ʃ"),
    ("th", "θ"),
    ("ph", "f"),
    ("wh", "w"),
    ("ck", "k"),
    ("ng", "ŋ"),
    ("qu", "kw"),
    ("ee", "i"),
    ("ea", "i"),
    ("oo", "u"),
    ("ou", "W"),
    ("ow", "W"),
    ("ai", "A"),
    ("ay", "A"),
    ("oi", "Y"),
    ("oy", "Y"),
    ("au", "ɔ"),
    ("aw", "ɔ"),
    ("igh", "I"),
    ("ar", "ɑɹ"),
    ("er", "əɹ"),
    ("ir", "ɜɹ"),
    ("or", "ɔɹ"),
    ("ur", "ɜɹ"),
];

/// One letter's sound, long where a silent e follows the consonant after it.
fn letter_sound(c: char, long: bool) -> &'static str {
    match (c, long) {
        ('a', true) => "A",
        ('a', false) => "æ",
        ('e', true) => "i",
        ('e', false) => "ɛ",
        ('i', true) => "I",
        ('i', false) => "ɪ",
        ('o', true) => "O",
        ('o', false) => "ɑ",
        ('u', true) => "ju",
        ('u', false) => "ʌ",
        ('y', _) => "i",
        ('b', _) => "b",
        ('c', _) => "k",
        ('d', _) => "d",
        ('f', _) => "f",
        ('g', _) => "ɡ",
        ('h', _) => "h",
        ('j', _) => "ʤ",
        ('k', _) => "k",
        ('l', _) => "l",
        ('m', _) => "m",
        ('n', _) => "n",
        ('p', _) => "p",
        ('q', _) => "k",
        ('r', _) => "ɹ",
        ('s', _) => "s",
        ('t', _) => "t",
        ('v', _) => "v",
        ('w', _) => "w",
        ('x', _) => "ks",
        ('z', _) => "z",
        _ => "",
    }
}

fn is_vowel_sound(s: &str) -> bool {
    s.chars().next().is_some_and(|c| "AIOWYæɛɪɑʌiuəɜɔ".contains(c))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_number_is_said_as_words() {
        assert_eq!(spell_number("0"), ["zero"]);
        assert_eq!(spell_number("17"), ["seventeen"]);
        assert_eq!(spell_number("42"), ["forty", "two"]);
        assert_eq!(spell_number("1200"), ["one", "thousand", "two", "hundred"]);
        // A frequency is digits and a point, not four hundred and thirty
        // three point ninety two.
        assert_eq!(spell_number("433.92"), ["four", "three", "three", "point", "nine", "two"]);
    }

    #[test]
    fn an_ending_is_said_as_the_sound_before_it_requires() {
        assert_eq!(voiced_suffix("kˈæt", Suffix::S), "s");
        assert_eq!(voiced_suffix("dˈɑɡ", Suffix::S), "z");
        assert_eq!(voiced_suffix("bˈʌs", Suffix::S), "ᵻz");
        assert_eq!(voiced_suffix("wˈɔnt", Suffix::D), "ᵻd");
        assert_eq!(voiced_suffix("wˈɔk", Suffix::D), "t");
        assert_eq!(voiced_suffix("kˈɔl", Suffix::D), "d");
    }

    #[test]
    fn a_word_outside_the_dictionary_is_guessed_at() {
        // Stress on the first vowel, the digraph read as one sound, and the
        // silent e lengthening the vowel before it.
        assert_eq!(guess("shake"), "ʃˈAk");
        // A compound the rules cannot see the seam of: the silent e inside
        // it is sounded, which is what a guess costs.
        assert_eq!(guess("waveshark"), "wˈævɛʃɑɹk");
    }

    #[test]
    fn punctuation_the_model_reads_survives() {
        let lex = English { words: HashMap::new() };
        let said = lex.say("one, two?");
        assert!(said.phonemes.contains(','), "{}", said.phonemes);
        assert!(said.phonemes.ends_with('?'), "{}", said.phonemes);
    }
}
