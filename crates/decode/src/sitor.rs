//! SITOR-B: CCIR 476 characters sent twice for forward error correction.

pub const ALPHA: u8 = 0x0f;
pub const BETA: u8 = 0x33;
pub const REP: u8 = 0x66;
pub const LTRS: u8 = 0x5a;
pub const FIGS: u8 = 0x36;
pub const CHAR32: u8 = 0x6a;

pub const BITS: usize = 7;

pub const REPEAT_BITS: usize = 5 * BITS;

const TABLE: [(u8, char, char); 29] = [
    (0x17, 'J', '\''),
    (0x1b, 'F', '!'),
    (0x1d, 'C', ':'),
    (0x1e, 'K', '('),
    (0x27, 'W', '2'),
    (0x2b, 'Y', '6'),
    (0x2d, 'P', '0'),
    (0x2e, 'Q', '1'),
    (0x35, 'G', '&'),
    (0x39, 'M', '.'),
    (0x3a, 'X', '/'),
    (0x3c, 'V', ';'),
    (0x47, 'A', '-'),
    (0x4b, 'S', '\x07'),
    (0x4d, 'I', '8'),
    (0x4e, 'U', '7'),
    (0x53, 'D', '$'),
    (0x55, 'R', '4'),
    (0x56, 'E', '3'),
    (0x59, 'N', ','),
    (0x5c, ' ', ' '),
    (0x63, 'Z', '"'),
    (0x65, 'L', ')'),
    (0x69, 'H', '#'),
    (0x6c, '\n', '\n'),
    (0x71, 'O', '9'),
    (0x72, 'B', '?'),
    (0x74, 'T', '5'),
    (0x78, '\r', '\r'),
];

pub fn valid(code: u8) -> bool {
    code < 0x80 && code.count_ones() == 4
}

pub fn char_of(code: u8, figures: bool) -> Option<char> {
    TABLE.iter().find(|(c, _, _)| *c == code).map(|(_, l, f)| if figures { *f } else { *l })
}

pub fn code_of(ch: char) -> Option<(u8, bool)> {
    let ch = ch.to_ascii_uppercase();
    TABLE.iter().find_map(|(c, l, f)| match () {
        _ if *l == ch => Some((*c, false)),
        _ if *f == ch => Some((*c, true)),
        _ => None,
    })
}

pub fn text(codes: &[u8]) -> String {
    let mut out = String::new();
    let mut figures = false;
    for &c in codes {
        match c {
            LTRS => figures = false,
            FIGS => figures = true,
            c => match char_of(c, figures) {
                Some('\x07') | None => {}
                Some('\r') => {}
                Some(ch) => out.push(ch),
            },
        }
    }
    out
}

pub fn encode(text: &str) -> Vec<u8> {
    let mut out = vec![LTRS];
    let mut figures = false;
    for ch in text.chars() {
        let Some((code, figs)) = code_of(ch) else { continue };
        let both = char_of(code, false) == char_of(code, true);
        if !both && figs != figures {
            out.push(if figs { FIGS } else { LTRS });
            figures = figs;
        }
        out.push(code);
    }
    out
}

pub fn fec_bits(codes: &[u8], phasing: usize) -> Vec<bool> {
    let mut alphas: Vec<u8> = vec![ALPHA; phasing];
    alphas.extend_from_slice(codes);
    alphas.extend([ALPHA; 3]);
    let mut out = Vec::with_capacity(alphas.len() * 2 * BITS);
    for (k, &a) in alphas.iter().enumerate() {
        let rep = match k + 2 < phasing {
            true => REP,
            false => alphas.get(k + 2).copied().unwrap_or(ALPHA),
        };
        for code in [rep, a] {
            out.extend((0..BITS).map(|b| code >> b & 1 == 1));
        }
    }
    out
}

fn code_at(bits: &[bool], at: usize) -> u8 {
    (0..BITS).fold(0u8, |c, b| c | (u8::from(bits[at + b]) << b))
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Read {
    Code(u8),
    Lost,
}

pub struct Fec {
    bits: Vec<bool>,
    base: usize,
    cursor: Option<usize>,
    inverted: bool,
    misses: usize,
}

impl Default for Fec {
    fn default() -> Self {
        Self::new()
    }
}

pub const HUNT_BITS: usize = 40 * BITS;

pub const LOCK_SCORE: usize = 10;

pub const LOST_MISSES: usize = 8;

impl Fec {
    pub fn new() -> Self {
        Self { bits: Vec::new(), base: 0, cursor: None, inverted: false, misses: 0 }
    }

    pub fn reset(&mut self) {
        *self = Self::new();
    }

    pub fn locked(&self) -> bool {
        self.cursor.is_some()
    }

    pub fn push(&mut self, bit: bool, out: &mut Vec<Read>) {
        self.bits.push(bit);
        if self.cursor.is_none() {
            self.hunt();
        }
        while self.cursor.is_some_and(|at| at + BITS <= self.base + self.bits.len()) {
            self.read(out);
        }
    }

    fn at(&self, abs: usize) -> u8 {
        let c = code_at(&self.bits, abs - self.base);
        if self.inverted { !c & 0x7f } else { c }
    }

    fn hunt(&mut self) {
        if self.bits.len() < HUNT_BITS {
            return;
        }
        let mut best: Option<(usize, usize, bool)> = None;
        for inverted in [false, true] {
            self.inverted = inverted;
            for off in REPEAT_BITS..REPEAT_BITS + 2 * BITS {
                let score = self.score(off);
                if best.is_none_or(|(s, _, _)| score > s) {
                    best = Some((score, off, inverted));
                }
            }
        }
        match best {
            Some((score, off, inverted)) if score >= LOCK_SCORE => {
                self.inverted = inverted;
                self.cursor = Some(self.base + off);
                self.misses = 0;
            }
            _ => {
                let drop = self.bits.len() - HUNT_BITS + BITS;
                self.bits.drain(..drop);
                self.base += drop;
            }
        }
    }

    fn score(&self, off: usize) -> usize {
        let end = self.base + self.bits.len();
        let mut score = 0;
        let mut at = self.base + off;
        while at + BITS <= end {
            let a = self.at(at);
            if valid(a) {
                let r = self.at(at - REPEAT_BITS);
                let before = self.at(at - BITS);
                if a == r && a != ALPHA && a != REP {
                    score += 2;
                } else if a == ALPHA && before == REP {
                    score += 1;
                }
            }
            at += 2 * BITS;
        }
        score
    }

    fn read(&mut self, out: &mut Vec<Read>) {
        let Some(at) = self.cursor else { return };
        let (a, r) = (self.at(at), self.at(at - REPEAT_BITS));
        let code = match (valid(a), valid(r)) {
            (true, _) => Some(a),
            (false, true) => Some(r),
            (false, false) => None,
        };
        match code {
            Some(c) => {
                self.misses = 0;
                out.push(Read::Code(c));
            }
            None => self.misses += 1,
        }
        if self.misses >= LOST_MISSES {
            out.push(Read::Lost);
            self.cursor = None;
            let keep = HUNT_BITS.min(self.bits.len());
            let drop = self.bits.len() - keep;
            self.bits.drain(..drop);
            self.base += drop;
            return;
        }
        self.cursor = Some(at + 2 * BITS);
        let keep_from = (at + 2 * BITS).saturating_sub(REPEAT_BITS + BITS);
        if keep_from > self.base + HUNT_BITS {
            let drop = keep_from - self.base;
            self.bits.drain(..drop);
            self.base += drop;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_code_has_four_marks_and_reads_back() {
        for (c, l, f) in TABLE {
            assert!(valid(c), "{c:#x}");
            assert_eq!(char_of(c, false), Some(l));
            assert_eq!(char_of(c, true), Some(f));
        }
        for c in [ALPHA, BETA, REP, LTRS, FIGS, CHAR32] {
            assert!(valid(c), "{c:#x}");
            assert_eq!(char_of(c, false), None);
        }
        assert_eq!(text(&encode("ZCZC EA39\nWZ 144")), "ZCZC EA39\nWZ 144");
    }

    fn read_all(bits: &[bool]) -> (String, usize) {
        let mut fec = Fec::new();
        let mut out = Vec::new();
        for &b in bits {
            fec.push(b, &mut out);
        }
        let codes: Vec<u8> = out
            .iter()
            .filter_map(|r| match r {
                Read::Code(c) => Some(*c),
                Read::Lost => None,
            })
            .collect();
        let lost = out.iter().filter(|r| **r == Read::Lost).count();
        (text(&codes), lost)
    }

    #[test]
    fn a_message_comes_back_through_its_repeats_either_way_up() {
        let sent = "ZCZC EA39\nNASH POINT LIGHT, NORMAL CONDITIONS RESTORED.\nNNNN";
        let bits = fec_bits(&encode(sent), 3);
        for inverted in [false, true] {
            let air: Vec<bool> = bits.iter().map(|b| b ^ inverted).collect();
            let (got, _) = read_all(&air);
            assert_eq!(got.trim(), sent, "inverted={inverted}");
        }
    }

    #[test]
    fn a_character_lost_once_is_read_off_its_repeat() {
        let sent = "ZCZC EA39\nWZ 144 SELF CANCELLING\nNNNN";
        let mut bits = fec_bits(&encode(sent), 30);
        for k in (40 * 2 * BITS..bits.len() - 10 * BITS).step_by(6 * 2 * BITS) {
            bits[k] = !bits[k];
        }
        let (got, lost) = read_all(&bits);
        assert_eq!(got.trim(), sent);
        assert_eq!(lost, 0);
    }
}
