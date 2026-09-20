//! Baudot off an asynchronous line: the five-bit code RTTY has carried since
//! teleprinters, and the small closed set of speeds and shifts it runs at.
//!
//! Nothing here is DSP and nothing here is framing. A caller hands over the
//! five-bit codes a start and stop framer assembled, in the order they were
//! received, and gets the text a teleprinter would have printed. The shift
//! state is the whole of the decoding: the same code is `A` after a letters
//! shift and `-` after a figures shift, and a shift character missed in the
//! noise turns the rest of the line into digits.
//!
//! The table is the variant amateur and utility stations use, where the
//! figures case carries `$ ! & # ' " ; =` in the positions ITA2 left to
//! national use. A station keying international ITA2 differs in those eight
//! positions alone.

use common::packet::{Fact, Proto};
use std::fmt;
use std::str::FromStr;

/// Shift to figures case.
pub const FIGS: u8 = 0x1b;
/// Shift back to letters case, and what an idle line rests on.
pub const LTRS: u8 = 0x1f;

/// A code that is a character in neither case, so it says nothing about
/// whether the framing is right.
const NONE: char = '\0';

/// Letters case, indexed by the code with the first bit on the air as the
/// least significant.
const LETTERS: [char; 32] = [
    NONE, 'E', '\n', 'A', ' ', 'S', 'I', 'U', '\r', 'D', 'R', 'J', 'N', 'F', 'C', 'K', 'T', 'Z',
    'L', 'W', 'H', 'Y', 'P', 'Q', 'O', 'B', 'G', NONE, 'M', 'X', 'V', NONE,
];

/// Figures case. `\x07` is the bell a teleprinter rang; it is a character
/// that was sent rather than noise, so it counts as read and is dropped
/// from the text.
const FIGURES: [char; 32] = [
    NONE, '3', '\n', '-', ' ', '\'', '8', '7', '\r', '$', '4', '\x07', ',', '!', ':', '(', '5',
    '"', ')', '2', '#', '6', '0', '1', '9', '?', '&', NONE, '.', '/', '=', NONE,
];

/// The speeds RTTY is keyed at, in the order a menu offers them.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum Speed {
    /// 45.45 baud, the amateur standard, which is 60 words a minute.
    #[default]
    Baud45,
    Baud50,
    Baud75,
    Baud100,
    Baud200,
}

impl Speed {
    pub const ALL: [Speed; 5] =
        [Speed::Baud45, Speed::Baud50, Speed::Baud75, Speed::Baud100, Speed::Baud200];

    pub fn baud(self) -> f64 {
        match self {
            Speed::Baud45 => 45.45,
            Speed::Baud50 => 50.0,
            Speed::Baud75 => 75.0,
            Speed::Baud100 => 100.0,
            Speed::Baud200 => 200.0,
        }
    }
}

impl fmt::Display for Speed {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.baud())
    }
}

impl FromStr for Speed {
    type Err = ();

    fn from_str(s: &str) -> Result<Self, ()> {
        let s = s.trim().to_ascii_lowercase();
        let want: f64 = s.trim_end_matches("baud").trim().parse().map_err(|_| ())?;
        Speed::ALL.into_iter().find(|s| (s.baud() - want).abs() < 0.5).ok_or(())
    }
}

/// The separations between the mark and the space tone that are in use.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum Shift {
    /// 170 Hz, what every amateur station keys.
    #[default]
    Narrow,
    /// 425 Hz, the European utility shift.
    Medium,
    /// 850 Hz, the old military and weather shift.
    Wide,
}

impl Shift {
    pub const ALL: [Shift; 3] = [Shift::Narrow, Shift::Medium, Shift::Wide];

    pub fn hz(self) -> f64 {
        match self {
            Shift::Narrow => 170.0,
            Shift::Medium => 425.0,
            Shift::Wide => 850.0,
        }
    }
}

impl fmt::Display for Shift {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.hz())
    }
}

impl FromStr for Shift {
    type Err = ();

    fn from_str(s: &str) -> Result<Self, ()> {
        let s = s.trim().to_ascii_lowercase();
        let want: f64 = s.trim_end_matches("hz").trim().parse().map_err(|_| ())?;
        Shift::ALL.into_iter().find(|s| (s.hz() - want).abs() < 1.0).ok_or(())
    }
}

/// How long the stop element lasts, in bit times. A mechanical teleprinter
/// needed one and a half; everything since sends one, and two is what a
/// station keys when the other end is a machine that needs the rest.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum Stop {
    One,
    #[default]
    OneAndHalf,
    Two,
}

impl Stop {
    pub fn bits(self) -> f64 {
        match self {
            Stop::One => 1.0,
            Stop::OneAndHalf => 1.5,
            Stop::Two => 2.0,
        }
    }
}

/// The text a teleprinter would have printed from these codes.
///
/// Both carriage return and line feed become a newline, and a run of them
/// becomes one, because a teleprinter is sent both to move the carriage and
/// a station sends several to space its overs out.
pub fn text(codes: &[u8]) -> String {
    let mut out = String::new();
    let mut figures = false;
    for &c in codes {
        match c & 0x1f {
            FIGS => figures = true,
            LTRS => figures = false,
            code => {
                let ch = case(figures)[code as usize];
                match ch {
                    NONE | '\x07' => {}
                    '\r' | '\n' => {
                        if !out.ends_with('\n') && !out.is_empty() {
                            out.push('\n');
                        }
                    }
                    ch => out.push(ch),
                }
            }
        }
    }
    out
}

fn case(figures: bool) -> &'static [char; 32] {
    match figures {
        true => &FIGURES,
        false => &LETTERS,
    }
}

/// How much of a run of codes is a character somebody sent, between zero and
/// one.
///
/// Weak evidence on its own, and worth knowing how weak. Letters case has a
/// character for every code but zero, so a run of arbitrary bits scores
/// around nine tenths. What it does catch is a line read upside down: mark
/// and space can be either way about, inverting them inverts every bit, and
/// the letters shift an idle line rests on inverts to the unassigned code
/// zero. The framing is the other half of that evidence, and a caller with
/// both should weigh both.
pub fn printable(codes: &[u8]) -> f32 {
    if codes.is_empty() {
        return 0.0;
    }
    let mut figures = false;
    let mut good = 0usize;
    for &c in codes {
        match c & 0x1f {
            FIGS => {
                figures = true;
                good += 1;
            }
            LTRS => {
                figures = false;
                good += 1;
            }
            code => good += usize::from(case(figures)[code as usize] != NONE),
        }
    }
    good as f32 / codes.len() as f32
}

/// The codes that print `text`, with the shift characters needed to reach
/// each case. The inverse of [`text`], for a transmitter and for a test.
pub fn encode(text: &str) -> Vec<u8> {
    let mut out = Vec::with_capacity(text.len() + 4);
    let mut figures: Option<bool> = None;
    for ch in text.chars() {
        let ch = ch.to_ascii_uppercase();
        let ch = match ch {
            '\n' => '\r',
            c => c,
        };
        let letter = LETTERS.iter().position(|&c| c == ch && c != NONE);
        let figure = FIGURES.iter().position(|&c| c == ch && c != NONE);
        // Space, carriage return and line feed are the same code in both
        // cases, so they never force a shift.
        let (want, code) = match (letter, figure) {
            (Some(l), Some(f)) if l == f => (figures, l),
            (Some(l), _) => (Some(false), l),
            (_, Some(f)) => (Some(true), f),
            (None, None) => continue,
        };
        if let Some(want) = want
            && figures != Some(want)
        {
            out.push(if want { FIGS } else { LTRS });
            figures = Some(want);
        }
        out.push(code as u8);
    }
    out
}

/// The keyed line that carries `codes`: true is mark, false is space.
///
/// The asynchronous discipline the teleprinter brought with it. Each
/// character is a space start element, five data elements least significant
/// first, and a mark stop element of [`Stop`] bit times; the line rests at
/// mark between characters, which is what the receiver's framer hunts the
/// next start edge against.
///
/// `per_bit` elements are produced per bit time, because a stop element is
/// one and a half bits and whole bits cannot say that: two is enough for
/// every [`Stop`] here, and the caller then keys at `per_bit * baud`.
pub fn line(codes: &[u8], stop: Stop, per_bit: usize) -> Vec<bool> {
    let per_bit = per_bit.max(1);
    let stop_elements = (stop.bits() * per_bit as f64).round() as usize;
    // Idle mark before the first start edge, so a receiver has a reference
    // for what mark is before it has to decide a data bit.
    let mut out = vec![true; 8 * per_bit];
    for &code in codes {
        out.extend(std::iter::repeat_n(false, per_bit));
        for b in 0..5 {
            out.extend(std::iter::repeat_n(code >> b & 1 == 1, per_bit));
        }
        out.extend(std::iter::repeat_n(true, stop_elements));
    }
    out.extend(std::iter::repeat_n(true, 8 * per_bit));
    out
}

/// What was typed.
///
/// An operator typed it and sent it to whoever was listening, so it belongs
/// beside anything else somebody wrote rather than with the machines.
pub fn read(bytes: &[u8]) -> Option<Proto> {
    let text = text(bytes);
    if text.trim().is_empty() {
        return None;
    }
    Some(Proto::new("rtty", "text").saying(Fact::message(text)))
}

/// Symbol times with no character framed either way up before a run is
/// closed and published. A stop element is at most two symbols and the next
/// character follows it, so this is the line resting rather than a gap
/// inside an over: about half a second at 45 baud.
pub const IDLE_SYMBOLS: usize = 24;

/// Undecided symbols in a row that close a run. One is a fade or a symbol
/// the correlators straddled; a pair is the station having stopped.
pub const QUIET_SYMBOLS: usize = 2;

/// Characters a run must hold before it is worth publishing. Below this a
/// run is as likely to be noise framed by luck as anything anybody sent.
pub const MIN_CHARS: usize = 6;

/// How much of a run has to be a character the tables have. Letters case
/// has one for every code but zero, so this is a low bar by itself and the
/// framing is what does the real refusing.
pub const MIN_PRINTABLE: f32 = 0.9;

/// The longest run held before it is forced out, in characters.
pub const MAX_CHARS: usize = 4_096;

/// The asynchronous line, read both ways up at once.
///
/// One line per polarity, and a run closes on the channel falling quiet or
/// on neither having read a character for a while, so the closing does not
/// depend on knowing which way up the station is. The line itself is
/// [`dsp::slice::Uart`] at five bits, which is Baudot.
pub struct Framer {
    upright: dsp::slice::Uart,
    inverted: dsp::slice::Uart,
    up: Vec<u8>,
    down: Vec<u8>,
    since_char: usize,
    undecided: usize,
}

impl Default for Framer {
    fn default() -> Self {
        Self::new()
    }
}

impl Framer {
    pub fn new() -> Self {
        Self {
            // Baudot has no idle count of its own: a run is closed by this
            // framer, on either polarity having framed nothing.
            upright: dsp::slice::Uart::new(5, usize::MAX),
            inverted: dsp::slice::Uart::new(5, usize::MAX),
            up: Vec::new(),
            down: Vec::new(),
            since_char: 0,
            undecided: 0,
        }
    }

    /// Feed one symbol, and hand back a run of codes where it ended one.
    pub fn push(&mut self, sym: dsp::afsk::Symbol) -> Option<Vec<u8>> {
        let flipped = dsp::afsk::Symbol { mark: !sym.mark, quiet: sym.quiet };
        let a = read_into(&mut self.upright, sym, &mut self.up);
        let b = read_into(&mut self.inverted, flipped, &mut self.down);
        self.since_char = match a || b {
            true => 0,
            false => self.since_char + 1,
        };
        self.undecided = match sym.quiet {
            true => self.undecided + 1,
            false => 0,
        };
        let resting = self.undecided >= QUIET_SYMBOLS || self.since_char >= IDLE_SYMBOLS;
        if !resting {
            return match self.up.len().max(self.down.len()) >= MAX_CHARS {
                true => self.take(),
                false => None,
            };
        }
        self.take()
    }

    /// Close whatever is open, and publish the better reading of it.
    pub fn take(&mut self) -> Option<Vec<u8>> {
        let up = std::mem::take(&mut self.up);
        let down = std::mem::take(&mut self.down);
        self.upright.reset();
        self.inverted.reset();
        self.undecided = 0;
        self.since_char = 0;
        // More characters framed is the first evidence, because a station
        // read upside down loses its stop bits and frames almost nothing.
        // Where both framed the same count, the tables decide.
        let best = match (up.len(), down.len()) {
            (a, b) if a > b => up,
            (a, b) if b > a => down,
            _ if printable(&down) > printable(&up) => down,
            _ => up,
        };
        if best.len() < MIN_CHARS || printable(&best) < MIN_PRINTABLE {
            return None;
        }
        Some(best)
    }

    pub fn reset(&mut self) {
        *self = Self::new();
    }
}

/// One symbol into one line, keeping the character where it framed.
fn read_into(line: &mut dsp::slice::Uart, sym: dsp::afsk::Symbol, codes: &mut Vec<u8>) -> bool {
    match line.push(sym) {
        dsp::slice::Read::Byte(code) => {
            if codes.len() < MAX_CHARS {
                codes.push(code);
            }
            true
        }
        _ => false,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Every character is a start element, five data elements and a stop of
    /// one and a half, which at half-bit resolution is 2 + 10 + 3.
    #[test]
    fn a_keyed_line_is_a_start_five_data_and_a_stop_per_character() {
        let idle = 16;
        let bits = line(&[0b10101], Stop::OneAndHalf, 2);
        assert_eq!(bits.len(), idle + 15 + idle);
        assert_eq!(&bits[idle..idle + 2], &[false, false], "the start element is a space");
        // Least significant data element first: 1, 0, 1, 0, 1.
        let data: Vec<bool> = bits[idle + 2..idle + 12].iter().step_by(2).copied().collect();
        assert_eq!(data, vec![true, false, true, false, true]);
        assert_eq!(&bits[idle + 12..idle + 15], &[true; 3], "a stop of one and a half bits");
        assert_eq!(line(&[0], Stop::Two, 2).len(), idle + 16 + idle);
    }

    #[test]
    fn a_shift_decides_which_character_a_code_is() {
        // Code 3 is `A` in letters and `-` in figures, and the shift that
        // came before it is the whole difference.
        assert_eq!(text(&[3]), "A");
        assert_eq!(text(&[FIGS, 3]), "-");
        assert_eq!(text(&[FIGS, 3, LTRS, 3]), "-A");
    }

    #[test]
    fn a_call_and_a_report_survive_the_round_trip() {
        let codes = encode("CQ DE MI0ABC 599");
        assert_eq!(text(&codes), "CQ DE MI0ABC 599");
        // A letters shift to open, one either side of the zero in the call,
        // and one for the report.
        assert_eq!(codes.iter().filter(|&&c| c == FIGS || c == LTRS).count(), 4);
    }

    #[test]
    fn a_run_of_returns_is_one_newline() {
        let codes = encode("RYRY\r\n\r\nTEST");
        assert_eq!(text(&codes), "RYRY\nTEST");
    }

    /// What the score is for: a line read upside down. The idle is where the
    /// evidence is, since letters case has a character for all but one code
    /// and inverted text alone still scores nine tenths.
    #[test]
    fn an_upside_down_line_scores_on_its_idle() {
        let mut codes = vec![LTRS; 16];
        codes.extend(encode("CQ CQ DE MI0ABC K"));
        let flipped: Vec<u8> = codes.iter().map(|c| !c & 0x1f).collect();
        assert_eq!(printable(&codes), 1.0);
        assert!(printable(&flipped) < 0.6, "{}", printable(&flipped));
        assert_eq!(printable(&[LTRS; 16]), 1.0);
        assert_eq!(printable(&[0; 16]), 0.0);
    }

    #[test]
    fn the_speeds_and_shifts_parse_once() {
        assert_eq!("45.45".parse(), Ok(Speed::Baud45));
        assert_eq!("45".parse(), Ok(Speed::Baud45));
        assert_eq!("75 baud".parse(), Ok(Speed::Baud75));
        assert_eq!("31".parse::<Speed>(), Err(()));
        assert_eq!("170".parse(), Ok(Shift::Narrow));
        assert_eq!("850 Hz".parse::<Shift>().map(|s| s.hz()), Ok(850.0));
        assert_eq!("200".parse::<Shift>(), Err(()));
        assert_eq!(Stop::OneAndHalf.bits(), 1.5);
    }
}
