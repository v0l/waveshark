//! ACARS: the messages aircraft and ground stations send each other, ARINC
//! 618 over an AM voice channel.
//!
//! Bits in, blocks out, and then fields out of a block. The waveform below is
//! `dsp::msk`, which this knows nothing about beyond the bits it produces
//! least significant first.
//!
//! A transmission is a bit sync, two SYN characters, a start of header, the
//! block, and a CRC-16 that the block and the check bytes together divide to
//! zero. Every character is seven bits with odd parity in the top bit, which
//! is a second check on each one.
//!
//! ```text
//! [mode] [address x7] [ack] [label x2] [block id] [STX] [text] [ETX or ETB]
//! ```
//!
//! - `mode`     one character, which side sent it and on what category of link
//! - `address`  the aircraft's registration, dot padded on the left
//! - `ack`      the block being acknowledged, or NAK when there is none
//! - `label`    what the message is, two characters from a published list
//! - `block id` a sequence letter; a downlink is one of `0` to `9`
//! - a downlink then opens its text with a four character message number and
//!   six characters of flight number
//!
//! Nothing is authenticated and nothing is encrypted, so a message is what the
//! transmitter chose to say. An uplink's address is the aircraft it is for
//! rather than the ground station that sent it.

use crate::protocol::Value;
use common::packet::{Entity, Fact, Id, Proto, ThingKind};

const SYN: u8 = 0x16;
const SOH: u8 = 0x01;
const STX: u8 = 0x02;
const ETX: u8 = 0x83;
const ETB: u8 = 0x97;
const DLE: u8 = 0x7f;
const NAK: u8 = 0x15;

/// Longest block ARINC 618 allows, with room for the header.
const MAX_TEXT: usize = 240;
/// Mode, address, ack, label, block id and a start of text: the shortest
/// thing that can be a block.
const MIN_TEXT: usize = 13;
/// Characters whose parity may be wrong before a block is noise rather than a
/// fade. Nothing here repairs one, so this is a limit on how bad a reception
/// can be and still be worth checking.
const MAX_PARITY_ERRORS: usize = 2;

/// One decoded message.
#[derive(Clone, Debug, PartialEq)]
pub struct Message {
    /// Which link and which direction, as one character.
    pub mode: char,
    /// The aircraft's registration, without the padding dots.
    pub registration: String,
    /// The block this acknowledges, or `None` for a negative acknowledgement.
    pub ack: Option<char>,
    /// Two characters saying what the message is.
    pub label: String,
    pub block_id: char,
    /// Whether the aircraft sent it, which is what the block id says.
    pub downlink: bool,
    /// The message number a downlink opens its text with.
    pub number: Option<String>,
    /// The flight number, where the message carries one.
    pub flight: Option<String>,
    /// What the message says, once the header has been taken off it.
    pub text: String,
    /// True when the block ended in ETB, meaning more blocks follow.
    pub more: bool,
}

impl Message {
    /// The fields a row on the bus carries.
    pub fn fields(&self) -> Vec<(String, Value)> {
        let mut f: Vec<(String, Value)> = vec![
            ("mode".into(), Value::Text(self.mode.to_string())),
            ("registration".into(), Value::Text(self.registration.clone())),
            ("label".into(), Value::Text(self.label.clone())),
            ("block_id".into(), Value::Text(self.block_id.to_string())),
        ];
        if let Some(a) = self.ack {
            f.push(("ack".into(), Value::Text(a.to_string())));
        }
        if let Some(n) = &self.number {
            f.push(("message_number".into(), Value::Text(n.clone())));
        }
        if let Some(fl) = &self.flight {
            f.push(("flight".into(), Value::Text(fl.clone())));
        }
        if !self.text.is_empty() {
            f.push(("text".into(), Value::Text(self.text.clone())));
        }
        if self.more {
            f.push(("more".into(), Value::Bool(true)));
        }
        f
    }
}

/// Read the fields out of a block, which is what [`Framer`] hands over: the
/// characters from the mode to the end of text, parity already stripped.
pub fn parse(block: &[u8]) -> Option<Message> {
    if block.len() < MIN_TEXT {
        return None;
    }
    let mode = block[0] as char;
    let registration: String =
        block[1..8].iter().filter(|b| **b != b'.').map(|b| *b as char).collect();
    let ack = if block[8] == NAK { None } else { Some(block[8] as char) };
    // A squitter's label ends in a delete character, which every decoder and
    // every published list writes as `_d`.
    let label: String =
        block[9..11].iter().map(|b| if *b == DLE { 'd' } else { *b as char }).collect();
    let block_id = block[11] as char;
    // A downlink numbers its blocks with a digit; an uplink uses a letter.
    let downlink = block_id.is_ascii_digit();

    let mut at = 13;
    let end = block.len().saturating_sub(1);
    let more = block[end] == (ETB & 0x7f);
    let (mut number, mut flight) = (None, None);
    if downlink && block[12] == STX {
        if at + 4 <= end {
            number = Some(text_of(&block[at..at + 4]));
            at += 4;
        }
        if at + 6 <= end {
            flight = Some(text_of(&block[at..at + 6]));
            at += 6;
        }
    }
    let text = if at < end { text_of(&block[at..end]) } else { String::new() };
    Some(Message { mode, registration, ack, label, block_id, downlink, number, flight, text, more })
}

/// Characters as a person reads them: the seven bit set, with the control
/// codes a message uses as separators left in as newlines.
fn text_of(bytes: &[u8]) -> String {
    bytes
        .iter()
        .map(|b| match b & 0x7f {
            0x0d | 0x0a => '\n',
            c if (0x20..0x7f).contains(&c) => c as char,
            _ => '.',
        })
        .collect::<String>()
        .trim_end()
        .to_string()
}

/// ARINC 618 block assembly, fed one demodulated bit at a time.
///
/// Sync, header, text, and the two check bytes. Nothing here repairs a
/// character: `acarsdec` corrects single and double bit errors from the parity
/// positions and a syndrome table, and a block that would need that is dropped
/// instead, which costs the weakest receptions and no others.
#[derive(Default)]
pub struct Framer {
    /// The last bits, least significant first, as the wire sends them.
    byte: u8,
    have: usize,
    /// Bits to gather before looking again: one while hunting for sync, eight
    /// once aligned.
    want: usize,
    /// Whether the stream arrived inverted, which a sync character read as its
    /// complement is the evidence for. MSK carries no absolute polarity.
    flip: bool,
    state: State,
    text: Vec<u8>,
    crc: [u8; 2],
    errors: usize,
}

#[derive(Default, Clone, Copy, PartialEq)]
enum State {
    #[default]
    Sync1,
    Sync2,
    Soh,
    Text,
    Crc1,
    Crc2,
}

impl Framer {
    pub fn new() -> Self {
        Self { want: 1, ..Default::default() }
    }

    /// Feed a run of bits, appending every block that passed its parity and
    /// its CRC.
    pub fn process(&mut self, bits: &[bool], out: &mut Vec<Vec<u8>>) {
        for b in bits {
            if let Some(block) = self.bit(*b) {
                out.push(block);
            }
        }
    }

    pub fn reset(&mut self) {
        *self = Self::new();
    }

    fn bit(&mut self, bit: bool) -> Option<Vec<u8>> {
        self.byte >>= 1;
        if bit != self.flip {
            self.byte |= 0x80;
        }
        self.have += 1;
        if self.have < self.want.max(1) {
            return None;
        }
        self.have = 0;
        let r = self.byte;
        self.want = 8;
        match self.state {
            State::Sync1 => match r {
                SYN => self.state = State::Sync2,
                x if x == !SYN => {
                    self.flip = !self.flip;
                    self.state = State::Sync2;
                }
                _ => self.want = 1,
            },
            State::Sync2 => match r {
                SYN => self.state = State::Soh,
                x if x == !SYN => self.flip = !self.flip,
                _ => self.restart(),
            },
            State::Soh => {
                if r == SOH {
                    self.state = State::Text;
                    self.text.clear();
                    self.errors = 0;
                } else {
                    self.restart();
                }
            }
            State::Text => {
                self.text.push(r);
                if r.count_ones().is_multiple_of(2) {
                    self.errors += 1;
                    if self.errors > MAX_PARITY_ERRORS {
                        self.restart();
                        return None;
                    }
                }
                if r == ETX || r == ETB {
                    self.state = State::Crc1;
                } else if self.text.len() > MAX_TEXT {
                    self.restart();
                } else if self.text.len() > 20 && r == DLE {
                    // The end of text went missing and the delete character
                    // that closes a transmission arrived in its place, so the
                    // two characters before it were the check bytes.
                    let len = self.text.len() - 3;
                    self.crc = [self.text[len], self.text[len + 1]];
                    self.text.truncate(len);
                    return self.finish();
                }
            }
            State::Crc1 => {
                self.crc[0] = r;
                self.state = State::Crc2;
            }
            State::Crc2 => {
                self.crc[1] = r;
                return self.finish();
            }
        }
        None
    }

    /// Check a gathered block and hand it over with its parity bits stripped.
    fn finish(&mut self) -> Option<Vec<u8>> {
        let mut text = std::mem::take(&mut self.text);
        self.restart();
        if text.len() < MIN_TEXT {
            return None;
        }
        // The byte between the header and the text is a start of text or an
        // end of text and nothing else, which is worth forcing because the
        // block's shape hangs on it.
        text[12] = (text[12] & (ETX | STX)) | STX;
        if text.iter().any(|b| b.count_ones() % 2 == 0) {
            return None;
        }
        if crc(&text, &self.crc) != 0 {
            return None;
        }
        for b in text.iter_mut() {
            *b &= 0x7f;
        }
        Some(text)
    }

    fn restart(&mut self) {
        self.state = State::Sync1;
        self.want = 1;
        self.text.clear();
        self.errors = 0;
    }
}

/// CRC-16 CCITT, reflected, polynomial 0x8408, as ARINC 618 uses it: the check
/// bytes are part of the message and a whole block divides to zero.
pub fn crc(text: &[u8], check: &[u8; 2]) -> u16 {
    let whole: Vec<u8> = text.iter().chain(check.iter()).copied().collect();
    crate::bits::crc16le(&whole, 0x8408, 0)
}

/// What an ACARS block says.
///
/// The aircraft is the subject either way: an uplink is addressed to the
/// aeroplane, not sent by the ground station, whose name is nowhere in the
/// block. Most of these are a machine talking to its airline, so the text is
/// carried as a message only where a crew typed it.
pub fn read(m: &Message) -> Proto {
    let mut who = Entity::new("acars", Id::Text(m.registration.clone()));
    if let Some(f) = m.flight.as_ref().filter(|f| !f.trim().is_empty()) {
        who = who.named(f.clone());
    }
    let p = Proto::new("acars", if m.downlink { "downlink" } else { "uplink" })
        .by(who.clone())
        .saying(Fact::Named(common::packet::Named::new(who.to_string(), ThingKind::Aircraft)));
    match free_text(m) {
        true => p.saying(Fact::message(m.text.clone())),
        false => p,
    }
}

/// Whether a person composed the block rather than a system aboard.
///
/// The label says which: `80`, `10` and the free text labels are what a crew
/// types, and everything else is a position report, an OOOI time or a
/// weather request the box sent by itself. A message view that took every
/// readable block showed the flight management computer's chatter as though
/// somebody had written it.
fn free_text(m: &Message) -> bool {
    !m.text.trim().is_empty() && matches!(m.label.as_str(), "10" | "80" | "A6" | "AA" | "C1")
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Odd parity, which every character on the wire carries.
    fn parity(b: u8) -> u8 {
        if b.count_ones().is_multiple_of(2) { b | 0x80 } else { b }
    }

    /// A transmission as the bits arrive: sync, header, block, check bytes.
    fn wire(block: &[u8]) -> Vec<bool> {
        let body: Vec<u8> = block.iter().map(|b| parity(*b)).collect();
        let sum = crate::bits::crc16le(&body, 0x8408, 0);
        let mut bytes = vec![SYN, SYN, SOH];
        bytes.extend_from_slice(&body);
        bytes.push(sum as u8);
        bytes.push((sum >> 8) as u8);
        dsp::msk::bits_of(&bytes)
    }

    /// A downlink, as an aircraft sends: mode, registration, NAK, label,
    /// block id, then the number and flight the text opens with.
    fn downlink() -> Vec<u8> {
        let mut b = vec![b'2'];
        b.extend_from_slice(b".EI-DEO");
        b.push(NAK);
        b.extend_from_slice(b"Q0");
        b.push(b'1');
        b.push(STX);
        b.extend_from_slice(b"S01AEIN123");
        b.extend_from_slice(b"ENGINE OK");
        b.push(ETX & 0x7f);
        b
    }

    #[test]
    fn a_transmission_becomes_a_message() {
        let mut blocks = Vec::new();
        Framer::new().process(&wire(&downlink()), &mut blocks);
        assert_eq!(blocks.len(), 1, "one block, got {blocks:?}");
        let m = parse(&blocks[0]).expect("a message");
        assert_eq!(m.mode, '2');
        assert_eq!(m.registration, "EI-DEO");
        assert_eq!(m.label, "Q0");
        assert_eq!(m.block_id, '1');
        assert!(m.downlink);
        assert_eq!(m.ack, None);
        assert_eq!(m.number.as_deref(), Some("S01A"));
        assert_eq!(m.flight.as_deref(), Some("EIN123"));
        assert_eq!(m.text, "ENGINE OK");
        assert!(!m.more);
    }

    #[test]
    fn a_stream_that_arrived_inverted_reads_the_same() {
        // MSK has no absolute polarity, and a sync character read as its
        // complement is how the framer finds that out.
        let bits: Vec<bool> = wire(&downlink()).iter().map(|b| !b).collect();
        let mut blocks = Vec::new();
        Framer::new().process(&bits, &mut blocks);
        assert_eq!(blocks.len(), 1, "one block, got {blocks:?}");
        assert_eq!(parse(&blocks[0]).unwrap().registration, "EI-DEO");
    }

    #[test]
    fn a_block_with_a_bad_check_is_dropped() {
        let mut bits = wire(&downlink());
        // One bit of the text, which breaks that character's parity and the
        // block's CRC together.
        let at = 3 * 8 + 40;
        bits[at] = !bits[at];
        let mut blocks = Vec::new();
        Framer::new().process(&bits, &mut blocks);
        assert!(blocks.is_empty(), "a broken block was reported: {blocks:?}");
    }

    #[test]
    fn sync_is_found_wherever_it_starts() {
        // The demodulator runs from the moment the channel opens, so a block
        // is somewhere in a stream of noise rather than at bit zero.
        let mut bits: Vec<bool> = (0..1001).map(|i| i % 3 == 0).collect();
        bits.extend(wire(&downlink()));
        let mut blocks = Vec::new();
        Framer::new().process(&bits, &mut blocks);
        assert_eq!(blocks.len(), 1, "one block, got {blocks:?}");
    }

    #[test]
    fn an_uplink_is_not_read_as_a_downlink() {
        // An uplink numbers its blocks with a letter and carries no message
        // number or flight, so reading one as a downlink eats ten characters
        // of its text.
        let mut b = vec![b'2'];
        b.extend_from_slice(b".EI-DEO");
        b.push(b'A');
        b.extend_from_slice(b"H1");
        b.push(b'M');
        b.push(STX);
        b.extend_from_slice(b"CLIMB TO FL350");
        b.push(ETX & 0x7f);
        let mut blocks = Vec::new();
        Framer::new().process(&wire(&b), &mut blocks);
        let m = parse(&blocks[0]).expect("a message");
        assert!(!m.downlink);
        assert_eq!(m.ack, Some('A'));
        assert_eq!(m.number, None);
        assert_eq!(m.text, "CLIMB TO FL350");
    }
}
