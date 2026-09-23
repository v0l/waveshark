//! FLEX paging: the words of a frame in, addressed pages out.
//!
//! The message layer only, as `pocsag` is. Everything that decides whether a
//! frame happened (the sync word, the speed it announces, the interleaving,
//! the bit clock) is `dsp::flex`. What arrives here is one phase of a frame:
//! eighty-eight raw 32-bit words, still to be corrected.
//!
//! # A phase
//!
//! A FLEX frame is 1.875 s and carries up to four interleaved phases. Each
//! phase is 88 words of 21 message bits, ten BCH(31,21) parity bits and an
//! overall parity bit. The first word is the block information word, which
//! says where the addresses start and where the vectors start; the addresses
//! run from there to the vector offset, and each address has a vector word at
//! the matching position after it. A vector says what kind of page it is and
//! which words hold the text.
//!
//! # Bit order, which is where this would go wrong
//!
//! FLEX transmits every word least significant bit first, the opposite of
//! POCSAG, so a word's bit 0 is the first bit on the air and the BCH code is
//! the same code read backwards. That is handled in [`repair`], and the
//! characters inside an alphanumeric page are then simply three seven-bit
//! characters per word from the bottom up.
//!
//! # Privacy
//!
//! The warning on `pocsag` applies here in full: FLEX carries hospital,
//! security and personal traffic in clear, and a channel left running writes
//! it to the packet log.

use crate::bits::{BCH_31_21_GEN, bch_parity, bch31_21};
use common::packet::{Fact, Link, Party, Proto};

/// Words in one phase of a frame.
pub const PHASE_WORDS: usize = 88;

/// The message bits of a word, once the parity has been stripped.
pub const MESSAGE_MASK: u32 = 0x001F_FFFF;

/// The numeric character set, indexed by the four bits as assembled: digits,
/// then a space, an urgency mark, a hyphen and the two brackets. Code 12 is
/// the fill that pads a message out to a word boundary and is dropped.
const NUMERIC: [char; 16] =
    ['0', '1', '2', '3', '4', '5', '6', '7', '8', '9', ' ', 'U', ' ', '-', ']', '['];

/// The offset every short address carries, so that capcode 1 is address
/// 0x8001 on the air.
const ADDRESS_BIAS: u32 = 0x8000;

/// Short addresses run from here to [`SHORT_ADDRESS_MAX`]; anything outside
/// is the two-word long form, which this does not read.
const SHORT_ADDRESS_MIN: u32 = 0x0_8001;
const SHORT_ADDRESS_MAX: u32 = 0x1E_0000;

const TEMPORARY_ADDRESSES: std::ops::RangeInclusive<u32> = 0x1F_7800..=0x1F_780F;

const INTERLEAVE_BLOCK_WORDS: usize = 8;

/// What a page carries. The three bits of the vector word's type field, as
/// the FLEX specification numbers them.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PageKind {
    Secure,
    ShortInstruction,
    Tone,
    StandardNumeric,
    SpecialNumeric,
    Alphanumeric,
    Binary,
    NumberedNumeric,
}

impl PageKind {
    fn from_bits(v: u32) -> Self {
        match v & 0x7 {
            0 => Self::Secure,
            1 => Self::ShortInstruction,
            2 => Self::Tone,
            3 => Self::StandardNumeric,
            4 => Self::SpecialNumeric,
            5 => Self::Alphanumeric,
            6 => Self::Binary,
            _ => Self::NumberedNumeric,
        }
    }

    fn bits(self) -> u32 {
        match self {
            Self::Secure => 0,
            Self::ShortInstruction => 1,
            Self::Tone => 2,
            Self::StandardNumeric => 3,
            Self::SpecialNumeric => 4,
            Self::Alphanumeric => 5,
            Self::Binary => 6,
            Self::NumberedNumeric => 7,
        }
    }

    /// How a row should be labelled, which is the page kind rather than the
    /// protocol: a tone page and an alphanumeric one are different things to
    /// whoever carries the pager.
    pub fn label(self) -> &'static str {
        match self {
            Self::Secure | Self::Alphanumeric => "FLEX-Alpha",
            Self::ShortInstruction => "FLEX-Instruction",
            Self::Tone => "FLEX-Tone",
            Self::StandardNumeric | Self::SpecialNumeric | Self::NumberedNumeric => "FLEX-Numeric",
            Self::Binary => "FLEX-Binary",
        }
    }
}

/// Whether an alphanumeric page stands on its own or is part of a message
/// split across frames.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Fragment {
    /// Complete: the whole message is in this page.
    Whole,
    /// The opening part, with more to follow.
    Opening,
    /// A continuation of a message that opened in an earlier frame.
    Continuation,
}

/// One page: who it was for and what it said.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Page {
    /// The pager's capcode, the address with its bias removed.
    pub capcode: u32,
    pub kind: PageKind,
    /// The text, for the kinds that carry any.
    pub text: Option<String>,
    /// Which of the four phases the page was read on, 'A' to 'D'.
    pub phase: char,
    pub fragment: Fragment,
}

/// Correct a 32-bit word and return its 21 message bits.
///
/// BCH(31,21) over bits 0 to 30, then the overall parity bit 31 as a check on
/// the correction: a word three or more bits wrong can still present a
/// syndrome that names a correctable pair, and applying that correction
/// yields a word that looks clean. The parity bit catches the odd cases, so
/// a correction that leaves the whole word odd is refused rather than kept.
///
/// Returns the message bits and how many bits were changed.
pub fn repair(word: u32) -> Option<(u32, u32)> {
    // code[i] is the coefficient of x^i, so the FLEX word is read from the
    // top down: bit 30 first.
    let mut code: Vec<bool> = (0..31).map(|i| word >> (30 - i) & 1 != 0).collect();
    let fixed_bits = bch31_21(&mut code)?;
    let fixed = (0..31).fold(0u32, |w, i| w | u32::from(code[30 - i]) << i);
    let parity_bad = (fixed | (word & 0x8000_0000)).count_ones() & 1 != 0;
    match (fixed_bits, parity_bad) {
        // Clean under BCH and odd overall: only the parity bit is wrong.
        (0, true) => Some((fixed & MESSAGE_MASK, 1)),
        (_, true) => None,
        (n, false) => Some((fixed & MESSAGE_MASK, n)),
    }
}

/// Build a word from 21 message bits: the BCH parity above them and the
/// overall even parity in the top bit.
pub fn encode_word(message: u32) -> u32 {
    let message = message & MESSAGE_MASK;
    // The message in transmission order, bit 0 first.
    let bits: Vec<bool> = (0..21).map(|i| message >> i & 1 != 0).collect();
    let parity = bch_parity(&bits, BCH_31_21_GEN, 10) as u32;
    // Parity coefficient i is the word's bit 30 - i.
    let word = (0..10).fold(message, |w, i| w | (parity >> i & 1) << (30 - i));
    word | (word.count_ones() & 1) << 31
}

/// The frame information word: which frame of which cycle this is.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Fiw {
    pub cycle: u8,
    pub frame: u8,
}

impl Fiw {
    /// Read a frame information word, or refuse it.
    ///
    /// The word carries its own check: the five nibbles and the twenty-first
    /// bit sum to 15 modulo 16. That is worth having on top of the BCH,
    /// because a frame number one out puts every page in the wrong frame and
    /// nothing else would notice.
    pub fn parse(word: u32) -> Option<Self> {
        let (fiw, _) = repair(word)?;
        let sum = (fiw & 0xF) + (fiw >> 4 & 0xF) + (fiw >> 8 & 0xF) + (fiw >> 12 & 0xF);
        let sum = sum + (fiw >> 16 & 0xF) + (fiw >> 20 & 0x1);
        if sum & 0xF != 0xF {
            return None;
        }
        Some(Self { cycle: (fiw >> 4 & 0xF) as u8, frame: (fiw >> 8 & 0x7F) as u8 })
    }

    /// The word a receiver would have heard for this frame, checksum
    /// included.
    pub fn encode(&self) -> u32 {
        let body = u32::from(self.cycle & 0xF) << 4 | u32::from(self.frame & 0x7F) << 8;
        let sum = (body >> 4 & 0xF) + (body >> 8 & 0xF) + (body >> 12 & 0xF) + (body >> 16 & 0xF);
        let checksum = (0xF - (sum & 0xF)) & 0xF;
        encode_word(body | checksum)
    }
}

/// Read one phase of a frame.
///
/// `words` is the phase as received, 88 raw 32-bit words. More words past
/// correction than one interleave block holds end the phase, since reading on
/// from there invents addresses; fewer lose only the pages that use them.
pub fn pages(words: &[u32], phase: char) -> Vec<Page> {
    if words.len() < PHASE_WORDS {
        return Vec::new();
    }
    let phase_words: Vec<Option<u32>> =
        words.iter().take(PHASE_WORDS).map(|&w| repair(w).map(|(message, _)| message)).collect();
    if phase_words.iter().filter(|w| w.is_none()).count() > INTERLEAVE_BLOCK_WORDS {
        return Vec::new();
    }
    let Some((address_at, vector_at)) = phase_words[0].and_then(layout) else {
        return Vec::new();
    };

    let mut out = Vec::new();
    for i in address_at..vector_at {
        let Some(address) = phase_words[i] else { continue };
        if address == 0 || address == MESSAGE_MASK {
            continue; // Idle codeword rather than an address.
        }
        if !(SHORT_ADDRESS_MIN..=SHORT_ADDRESS_MAX).contains(&address)
            && !TEMPORARY_ADDRESSES.contains(&address)
        {
            // The long form spans two words and carries its own bias; it is
            // rare enough on the air that reading it half way would be worse
            // than not reading it.
            continue;
        }
        let capcode = address - ADDRESS_BIAS;
        let vector_word = vector_at + i - address_at;
        if vector_word >= PHASE_WORDS {
            continue;
        }
        let Some(viw) = phase_words[vector_word] else { continue };
        let kind = PageKind::from_bits(viw >> 4);
        let first = (viw >> 7 & 0x7F) as usize;
        let len = (viw >> 14 & 0x7F) as usize;
        let mut page = Page { capcode, kind, text: None, phase, fragment: Fragment::Whole };
        match kind {
            PageKind::Alphanumeric | PageKind::Secure => {
                let Some(message) = message_words(&phase_words, first, len) else { continue };
                let (text, fragment) = alphanumeric(&message);
                page.fragment = fragment;
                page.text = Some(text);
            }
            PageKind::StandardNumeric | PageKind::SpecialNumeric | PageKind::NumberedNumeric => {
                let Some(message) = message_words(&phase_words, first, len) else { continue };
                page.text = Some(numeric(&message, kind));
            }
            PageKind::Tone => {}
            PageKind::ShortInstruction | PageKind::Binary => {}
        }
        out.push(page);
    }
    out
}

fn layout(biw: u32) -> Option<(usize, usize)> {
    if biw == 0 || biw == MESSAGE_MASK {
        return None;
    }
    let vector_at = (biw >> 10 & 0x3F) as usize;
    let address_at = (biw >> 8 & 0x03) as usize + 1;
    (vector_at > address_at && vector_at < PHASE_WORDS).then_some((address_at, vector_at))
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Assignment {
    pub capcode: u32,
    pub temporary: u8,
    pub frame: u8,
    pub words: [u32; 2],
}

impl Assignment {
    pub fn read(words: [u32; 2]) -> Option<Self> {
        let (address, _) = repair(words[0])?;
        let (viw, _) = repair(words[1])?;
        let activation =
            PageKind::from_bits(viw >> 4) == PageKind::ShortInstruction && viw >> 7 & 0x7 == 0;
        if !activation || !(SHORT_ADDRESS_MIN..=SHORT_ADDRESS_MAX).contains(&address) {
            return None;
        }
        Some(Self {
            capcode: address - ADDRESS_BIAS,
            temporary: (viw >> 17 & 0xF) as u8,
            frame: (viw >> 10 & 0x7F) as u8,
            words,
        })
    }

    pub fn instruction(capcode: u32, temporary: u8, frame: u8) -> [u32; 2] {
        let viw = PageKind::ShortInstruction.bits() << 4
            | u32::from(frame & 0x7F) << 10
            | u32::from(temporary & 0xF) << 17;
        [encode_word(capcode + ADDRESS_BIAS), encode_word(viw)]
    }
}

pub fn assignments(words: &[u32]) -> Vec<Assignment> {
    if words.len() < PHASE_WORDS {
        return Vec::new();
    }
    let Some((address_at, vector_at)) = repair(words[0]).and_then(|(biw, _)| layout(biw)) else {
        return Vec::new();
    };
    (address_at..vector_at)
        .filter_map(|i| words.get(vector_at + i - address_at).map(|v| [words[i], *v]))
        .filter_map(Assignment::read)
        .collect()
}

fn message_words(phase_words: &[Option<u32>], first: usize, len: usize) -> Option<Vec<u32>> {
    if first == 0 || len == 0 || first + len > PHASE_WORDS {
        return None;
    }
    phase_words[first..first + len].iter().copied().collect()
}

/// The text of an alphanumeric page.
///
/// The first word is a header: bits 11 and 12 say whether the message stands
/// alone, bit 10 whether more follows. The characters are three seven-bit
/// codes per word from the bottom up, and the first character of the first
/// word after the header is a check character rather than text when the
/// message is whole. ETX (0x03) is padding wherever it appears.
fn alphanumeric(words: &[u32]) -> (String, Fragment) {
    let header = words[0];
    let frag = header >> 11 & 0x03;
    let more = header >> 10 & 0x01 != 0;
    let fragment = match (more, frag == 3) {
        (true, _) => Fragment::Opening,
        (false, true) => Fragment::Whole,
        (false, false) => Fragment::Continuation,
    };
    let mut text = String::new();
    for (n, &word) in words.iter().skip(1).enumerate() {
        for shift in [0, 7, 14] {
            if n == 0 && shift == 0 && frag == 3 {
                continue;
            }
            let ch = (word >> shift & 0x7F) as u8;
            if ch != 0x03 {
                text.push(ch as char);
            }
        }
    }
    (text, fragment)
}

/// The digits of a numeric page: four bits each, least significant first,
/// running across the message words without regard to word boundaries.
///
/// The first bits are a header, two of them for a standard or special page
/// and ten for a numbered one, so the first digit starts part way into the
/// first word.
fn numeric(words: &[u32], kind: PageKind) -> String {
    let header_bits = match kind {
        PageKind::NumberedNumeric => 10,
        _ => 2,
    };
    let mut text = String::new();
    let (mut digit, mut count) = (0u8, 4 + header_bits);
    for &word in words {
        for k in 0..21 {
            digit = digit >> 1 & 0x0F;
            if word >> k & 1 != 0 {
                digit |= 0x08;
            }
            count -= 1;
            if count == 0 {
                // 12 is the fill that pads to a word boundary.
                if digit != 0x0C {
                    text.push(NUMERIC[digit as usize]);
                }
                count = 4;
            }
        }
    }
    text
}

/// What a page says, for building a phase to transmit or to test against.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Body {
    Tone,
    Numeric(String),
    Alpha(String),
}

/// Build one phase of a frame carrying the given pages.
///
/// The counterpart of [`pages`]: eighty-eight words, the block information
/// word first, then the addresses, the vectors and the message words, with
/// idle words filling the rest.
pub fn encode(pages: &[(u32, Body)]) -> Vec<u32> {
    let address_at = 1usize;
    let vector_at = address_at + pages.len();
    let mut words = vec![MESSAGE_MASK; PHASE_WORDS];
    words[0] = (vector_at as u32) << 10 | ((address_at - 1) as u32) << 8;
    let mut next = vector_at + pages.len();
    for (n, (capcode, body)) in pages.iter().enumerate() {
        words[address_at + n] = capcode + ADDRESS_BIAS;
        let (kind, message) = match body {
            Body::Tone => (PageKind::Tone, Vec::new()),
            Body::Alpha(s) => (PageKind::Alphanumeric, encode_alpha(s)),
            Body::Numeric(s) => (PageKind::StandardNumeric, encode_numeric(s)),
        };
        let (first, len) = match message.is_empty() {
            true => (0usize, 0usize),
            false => (next, message.len()),
        };
        words[vector_at + n] = kind.bits() << 4 | (first as u32) << 7 | (len as u32) << 14;
        for (k, w) in message.iter().enumerate() {
            words[next + k] = *w;
        }
        next += message.len();
    }
    words.into_iter().map(encode_word).collect()
}

/// The message words of an alphanumeric page: a header saying the message is
/// whole, a check character, then three characters per word.
fn encode_alpha(text: &str) -> Vec<u32> {
    // Fragment 3 and no continuation: a message complete in this frame.
    let mut words = vec![0x03 << 11];
    let mut chars: Vec<u8> = vec![0x03];
    chars.extend(text.bytes().map(|b| b & 0x7F));
    while !chars.len().is_multiple_of(3) {
        chars.push(0x03);
    }
    for triple in chars.chunks(3) {
        let mut word = 0u32;
        for (k, c) in triple.iter().enumerate() {
            word |= u32::from(*c) << (7 * k);
        }
        words.push(word);
    }
    words
}

/// The message words of a standard numeric page.
fn encode_numeric(text: &str) -> Vec<u32> {
    let mut bits: Vec<bool> = vec![false, false]; // The two header bits.
    for c in text.chars() {
        let code = NUMERIC.iter().position(|n| *n == c).unwrap_or(10) as u8;
        for k in 0..4 {
            bits.push(code >> k & 1 != 0);
        }
    }
    while !bits.len().is_multiple_of(21) {
        // Fill, code 12, in whatever space is left.
        for k in 0..4 {
            bits.push(0x0C >> k & 1 != 0);
            if bits.len().is_multiple_of(21) {
                break;
            }
        }
    }
    bits.chunks(21)
        .map(|c| c.iter().enumerate().fold(0u32, |w, (i, b)| w | u32::from(*b) << i))
        .collect()
}

/// The decodes one frame becomes: one per page, across every phase.
///
/// What a frame says: one layer per page.
///
/// A frame carries the whole transmitter's queue for its slot, so it is
/// several pages to several pagers and each is a statement of its own. The
/// capcode is the pager addressed, never the sender.
pub fn read(bytes: &[u8]) -> Vec<Proto> {
    let Some(frame) = dsp::flex::Frame::from_bytes(bytes) else { return Vec::new() };
    let names = frame.mode.phase_names();
    let this = Fiw::parse(frame.fiw).map(|f| f.frame);
    let mut groups: Vec<(u8, Vec<u32>)> = Vec::new();
    for a in frame.carried.iter().filter_map(|w| Assignment::read(*w)) {
        if Some(a.frame) != this {
            continue;
        }
        match groups.iter_mut().find(|(t, _)| *t == a.temporary) {
            Some((_, capcodes)) if capcodes.contains(&a.capcode) => {}
            Some((_, capcodes)) => capcodes.push(a.capcode),
            None => groups.push((a.temporary, vec![a.capcode])),
        }
    }
    let mut out = Vec::new();
    for (phase, name) in frame.phases.iter().zip(names) {
        for page in pages(phase, *name) {
            let to = to_party(page.capcode, &groups);
            out.push(
                Proto::new("flex", page.kind.label())
                    .between(Link { from: None, to: Some(to) })
                    .maybe(page.text.map(Fact::message)),
            );
        }
    }
    out
}

fn to_party(capcode: u32, groups: &[(u8, Vec<u32>)]) -> Party {
    let address = capcode + ADDRESS_BIAS;
    if !TEMPORARY_ADDRESSES.contains(&address) {
        return Party::unit(capcode.to_string());
    }
    let temporary = (address - TEMPORARY_ADDRESSES.start()) as u8;
    match groups.iter().find(|(t, _)| *t == temporary) {
        Some((_, capcodes)) => {
            Party::group(capcodes.iter().map(u32::to_string).collect::<Vec<_>>().join(", "))
        }
        None => Party::temporary(capcode.to_string()),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A word survives the round trip, one wrong bit is corrected, and three
    /// are refused rather than turned into a different word.
    #[test]
    fn a_word_carries_its_own_check() {
        let word = encode_word(0x0012_3456 & MESSAGE_MASK);
        assert_eq!(repair(word), Some((0x0012_3456 & MESSAGE_MASK, 0)));
        assert_eq!(repair(word ^ 1 << 5), Some((0x0012_3456 & MESSAGE_MASK, 1)));
        assert_eq!(repair(word ^ 1 << 31), Some((0x0012_3456 & MESSAGE_MASK, 1)));
        let two = word ^ 1 << 2 ^ 1 << 19;
        assert_eq!(repair(two), Some((0x0012_3456 & MESSAGE_MASK, 2)));
        // Three wrong bits: refused, or at worst not passed off as the word
        // that was sent.
        let mut refused = 0;
        for at in 0..29u32 {
            let bad = word ^ 1 << at ^ 1 << ((at + 7) % 31) ^ 1 << ((at + 17) % 31);
            let got = repair(bad);
            refused += u32::from(got.is_none() || got.unwrap().0 != 0x0012_3456 & MESSAGE_MASK);
        }
        assert_eq!(refused, 29, "a triple was read as the word that was sent");
    }

    /// The frame information word's checksum is what says the frame number is
    /// right, so a word one bit past what BCH can fix is refused.
    #[test]
    fn the_frame_word_checks_its_own_numbering() {
        let fiw = Fiw { cycle: 11, frame: 97 };
        assert_eq!(Fiw::parse(fiw.encode()), Some(fiw));
        assert_eq!(Fiw::parse(fiw.encode() ^ 1 << 9), Some(fiw));
        // Bits 8 to 14 are the frame number; corrupting four of them leaves
        // a word BCH cannot repair.
        assert_eq!(Fiw::parse(fiw.encode() ^ 0b111_1000_0000_0000), None);
    }

    /// Three pages in one phase, read back with their capcodes, their kinds
    /// and their text.
    #[test]
    fn a_phase_carries_a_page_for_each_address() {
        let phase = encode(&[
            (1_234_567, Body::Alpha("MOVE TO CHANNEL 2".into())),
            (98_765, Body::Numeric("0123456789".into())),
            (42, Body::Tone),
        ]);
        assert_eq!(phase.len(), PHASE_WORDS);
        let read = pages(&phase, 'A');
        assert_eq!(read.len(), 3);
        assert_eq!(read[0].capcode, 1_234_567);
        assert_eq!(read[0].kind, PageKind::Alphanumeric);
        assert_eq!(read[0].text.as_deref(), Some("MOVE TO CHANNEL 2"));
        assert_eq!(read[0].fragment, Fragment::Whole);
        assert_eq!(read[1].capcode, 98_765);
        assert_eq!(read[1].kind, PageKind::StandardNumeric);
        assert_eq!(read[1].text.as_deref(), Some("0123456789"));
        assert_eq!(read[2].capcode, 42);
        assert_eq!(read[2].kind, PageKind::Tone);
        assert_eq!(read[2].text, None);
    }

    /// One wrong bit in each of eight words is repaired and the pages still
    /// come out; a ninth word past repair ends the phase with nothing.
    #[test]
    fn a_damaged_phase_is_repaired_or_dropped() {
        let phase = encode(&[(1_234_567, Body::Alpha("CALL CONTROL".into()))]);
        let mut damaged = phase.clone();
        for w in damaged.iter_mut().take(8) {
            *w ^= 1 << 13;
        }
        let read = pages(&damaged, 'B');
        assert_eq!(read.len(), 1);
        assert_eq!(read[0].text.as_deref(), Some("CALL CONTROL"));
        assert_eq!(read[0].phase, 'B');

        let mut lost = phase;
        lost[3] ^= 0b1010_1010_1010;
        assert_eq!(pages(&lost, 'A').len(), 0, "an unrepairable word invented pages");
    }

    #[test]
    fn a_group_page_on_a_temporary_address_is_read() {
        let phase = encode(&[(0x1F_780E - ADDRESS_BIAS, Body::Alpha("P 1 BREDA".into()))]);
        let read = pages(&phase, 'A');
        assert_eq!(read.len(), 1);
        assert_eq!(read[0].capcode, 2_029_582);
        assert_eq!(read[0].text.as_deref(), Some("P 1 BREDA"));
        let past = encode(&[(0x1F_7810 - ADDRESS_BIAS, Body::Alpha("OPERATOR".into()))]);
        assert_eq!(pages(&past, 'A').len(), 0, "0x1F7810 is not a temporary address");
    }

    #[test]
    fn a_short_instruction_assigns_its_capcodes_to_a_temporary_address() {
        let mut phase = encode(&[(1_220_499, Body::Tone), (1_220_845, Body::Tone)]);
        for (at, capcode) in [(3, 1_220_499), (4, 1_220_845)] {
            phase[at] = Assignment::instruction(capcode, 14, 3)[1];
        }
        let heard: Vec<(u32, u8, u8)> =
            assignments(&phase).iter().map(|a| (a.capcode, a.temporary, a.frame)).collect();
        assert_eq!(heard, [(1_220_499, 14, 3), (1_220_845, 14, 3)]);
    }

    #[test]
    fn a_group_page_is_addressed_to_the_capcodes_assigned_to_it() {
        let group =
            encode(&[(0x1F_780E - ADDRESS_BIAS, Body::Alpha("A2 Breda rit: 169068".into()))]);
        let to = |carried: Vec<[u32; 2]>| -> Vec<Option<Party>> {
            let frame = dsp::flex::Frame {
                mode: dsp::flex::Mode { baud: 1600, levels: 2 },
                fiw: Fiw { cycle: 1, frame: 3 }.encode(),
                phases: vec![group.clone()],
                carried,
            };
            read(&frame.to_bytes()).into_iter().map(|r| r.link.to).collect()
        };
        let alone = vec![Some(Party::temporary("2029582"))];
        assert_eq!(
            to(vec![
                Assignment::instruction(1_220_499, 14, 3),
                Assignment::instruction(1_220_845, 14, 3),
                Assignment::instruction(1_220_499, 14, 3),
            ]),
            vec![Some(Party::group("1220499, 1220845"))]
        );
        assert_eq!(to(Vec::new()), alone);
        assert_eq!(to(vec![Assignment::instruction(1_220_499, 14, 4)]), alone, "meant for frame 4");
        assert_eq!(to(vec![Assignment::instruction(1_220_499, 13, 3)]), alone, "another group");
    }

    #[test]
    fn a_lost_interleave_block_costs_only_the_pages_that_use_it() {
        let phase = encode(&[
            (1_234_567, Body::Alpha("FIRST".into())),
            (98_765, Body::Alpha("SECOND".into())),
        ]);
        let smash = |at: std::ops::Range<usize>| {
            let mut p = phase.clone();
            for w in &mut p[at] {
                *w ^= 0b1010_1010_1010;
            }
            p
        };
        let texts = |p: &[u32]| -> Vec<String> {
            pages(p, 'A').into_iter().filter_map(|pg| pg.text).collect()
        };
        assert_eq!(texts(&smash(80..88)), ["FIRST", "SECOND"]);
        assert_eq!(texts(&smash(3..4)), ["SECOND"], "the first page's vector was lost");
        assert_eq!(texts(&smash(79..88)), Vec::<String>::new(), "nine lost words read as a phase");
        assert_eq!(texts(&smash(0..1)), Vec::<String>::new(), "a phase read without its BIW");
    }

    /// Noise is not a phase: random words present addresses and vectors that
    /// point anywhere, and none of it survives the word check.
    #[test]
    fn noise_is_not_a_phase() {
        let mut seed = 0x1234_5678u32;
        let mut next = move || {
            seed ^= seed << 13;
            seed ^= seed >> 17;
            seed ^= seed << 5;
            seed
        };
        let mut pages_read = 0;
        for _ in 0..2_000 {
            let phase: Vec<u32> = (0..PHASE_WORDS).map(|_| next()).collect();
            pages_read += pages(&phase, 'A').len();
        }
        assert_eq!(pages_read, 0, "noise was read as pages");
    }
}
