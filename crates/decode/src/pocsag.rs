//! POCSAG pages: codewords in, addressed messages out.
//!
//! The message layer only. Everything that decides whether a codeword
//! happened at all (the sync word, the batching, the BCH correction) is
//! `dsp::pocsag`, for the same reason the AIS split is where it is: what
//! arrives here has already proved itself, and reading it is a table.
//!
//! # What a page is
//!
//! An address codeword names a pager and starts a message; the message
//! codewords that follow it carry the text, until the next address, an idle
//! codeword or the end of the transmission. The address is 21 bits: 18 in the
//! codeword and three from the frame it sat in, because a pager only listens
//! during its own frame and the position in the batch is therefore part of
//! the number. Getting that wrong gives an address a factor of eight out,
//! which looks entirely plausible and matches no pager.
//!
//! # Bit order, which is where this would go wrong
//!
//! Codewords go on the air most significant bit first, but the *characters*
//! inside a message do not: each character, four bits of a numeric page or
//! seven of an alphanumeric one, is transmitted least significant bit first.
//! So the bits come out of the codewords in order and every character is then
//! reversed. A decoder that skips the reversal still produces printable text,
//! which is what makes this worth a comment: '1' becomes '\u{8}', '7' becomes
//! 'p', and nothing announces the mistake.
//!
//! # Privacy
//!
//! Pager traffic is unencrypted and carries medical, security and personal
//! detail as a matter of routine, which is not true of anything else this
//! receiver decodes. The packet log stores what the demodulator produced, so
//! a pager channel left running overnight writes recoverable message text to
//! disk, and what is legal to receive or to keep depends on the country. That
//! is the operator's call, but it should be a call rather than a surprise.

use dsp::pocsag::{BATCH_WORDS, IDLE};

/// The numeric character set, indexed by the four bits as transmitted, which
/// is the character's code with its bits reversed.
///
/// Codes 0 to 9 are the digits; 10 is a spare, and 11 to 15 are the urgency
/// mark, a space, a hyphen and the two brackets, per ITU-R M.584-2.
const NUMERIC: [char; 16] =
    ['0', '1', '2', '3', '4', '5', '6', '7', '8', '9', '*', 'U', ' ', '-', ')', '('];

/// What a page carries.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Body {
    /// An address with no message codewords: the page is the beep, and which
    /// of the four function codes it used is the whole content.
    Tone,
    /// Four bits per character, digits and a handful of symbols.
    Numeric(String),
    /// Seven bit ASCII.
    Alpha(String),
}

/// One page: who it was for, and what it said.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Message {
    /// The 21-bit Receiver Identity Code, printed on the pager as a seven
    /// digit number.
    pub address: u32,
    /// Which of the transmitter's four address slots this used. Function 3 is
    /// alphanumeric by near-universal convention; the rest are numeric or
    /// tone-only.
    pub function: u8,
    pub body: Body,
}

/// Read every page in a transmission's codewords.
///
/// `codewords` is the whole transmission, sixteen per batch and in the order
/// they arrived, as `dsp::pocsag::Transmission` produces it. The batch
/// position matters, so codewords must not be filtered before they get here:
/// an idle removed is an address shifted into the wrong frame.
pub fn parse(codewords: &[u32]) -> Vec<Message> {
    let mut out = Vec::new();
    // The page currently being assembled: its address, function, and the
    // message bits collected so far.
    let mut open: Option<(u32, u8)> = None;
    let mut bits: Vec<bool> = Vec::new();

    for (i, &w) in codewords.iter().enumerate() {
        if w >> 31 == 1 {
            // A message codeword with no address before it belongs to a page
            // whose start was lost, and there is no way to say who it was
            // for. Kept out rather than reported to nobody.
            if open.is_some() {
                for b in (11..31).rev() {
                    bits.push(w >> b & 1 == 1);
                }
            }
            continue;
        }
        // Any address codeword, including an idle, ends the page before it.
        flush(&mut out, &mut open, &mut bits);
        if w == IDLE {
            continue;
        }
        // The frame within the batch carries the low three bits of the
        // address; two codewords to a frame.
        let frame = (i % BATCH_WORDS / 2) as u32;
        let address = ((w >> 13) & 0x3_FFFF) << 3 | frame;
        open = Some((address, (w >> 11 & 3) as u8));
    }
    flush(&mut out, &mut open, &mut bits);
    out
}

fn flush(out: &mut Vec<Message>, open: &mut Option<(u32, u8)>, bits: &mut Vec<bool>) {
    let Some((address, function)) = open.take() else {
        bits.clear();
        return;
    };
    let body = if bits.is_empty() {
        Body::Tone
    } else if function == 3 {
        Body::Alpha(alpha(bits))
    } else {
        Body::Numeric(numeric(bits))
    };
    bits.clear();
    out.push(Message { address, function, body });
}

/// Four bits per character, each reversed, five to a codeword.
fn numeric(bits: &[bool]) -> String {
    let mut s = String::with_capacity(bits.len() / 4);
    for c in bits.chunks_exact(4) {
        // Reversed: the character's least significant bit was sent first.
        let code = c.iter().enumerate().fold(0usize, |a, (i, &b)| a | usize::from(b) << i);
        s.push(NUMERIC[code]);
    }
    // A message that does not fill its last codeword is padded with spaces,
    // and those are padding rather than content.
    s.trim_end().to_string()
}

/// Seven bits per character, each reversed, running across codewords.
fn alpha(bits: &[bool]) -> String {
    let mut s = String::with_capacity(bits.len() / 7);
    for c in bits.chunks_exact(7) {
        let code = c.iter().enumerate().fold(0u8, |a, (i, &b)| a | u8::from(b) << i);
        // The tail of a message is padded with nulls and end-of-text marks,
        // and a pager display shows neither. Anything else unprintable is
        // kept as a replacement so that a corrupted message looks corrupted
        // rather than shorter than it was.
        match code {
            0x00 | 0x03 | 0x04 | 0x17 => break,
            0x20..=0x7E => s.push(code as char),
            b'\n' | b'\r' => s.push(' '),
            _ => s.push('\u{FFFD}'),
        }
    }
    s.trim_end().to_string()
}

/// Build the codeword contents for a page, ready for `dsp::pocsag::encode_bits`.
///
/// The 21-bit contents rather than finished codewords: the BCH parity belongs
/// with the code that checks it. Idle contents pad the frames before the
/// address, because the frame an address sits in is part of the address and a
/// page cannot simply be put at the front of a batch.
///
/// Public because a receiver with no recording to test against can only be
/// tested against a transmission somebody built deliberately, and because the
/// transmit path will need exactly this.
pub fn encode(address: u32, function: u8, body: &Body) -> Vec<u32> {
    let idle = IDLE >> 11 & 0x1F_FFFF;
    let frame = (address & 7) as usize;
    let mut out = vec![idle; frame * 2];
    out.push((address >> 3 & 0x3_FFFF) << 2 | u32::from(function & 3));

    let mut bits: Vec<bool> = match body {
        Body::Tone => Vec::new(),
        Body::Numeric(s) => s
            .chars()
            .flat_map(|c| {
                let code = NUMERIC.iter().position(|&n| n == c).unwrap_or(12);
                (0..4).map(move |i| code >> i & 1 == 1)
            })
            .collect(),
        Body::Alpha(s) => s
            .chars()
            .flat_map(|c| {
                let code = c as u32 & 0x7F;
                (0..7).map(move |i| code >> i & 1 == 1)
            })
            .collect(),
    };
    // A message that does not fill its last codeword is padded: with the
    // space character for a numeric page, and with nulls for an alphanumeric
    // one, which is what a pager display drops rather than shows.
    if let Body::Numeric(_) = body {
        while bits.len() % 20 != 0 {
            let space = NUMERIC.iter().position(|&c| c == ' ').unwrap_or(12);
            bits.extend((0..4).map(|i| space >> i & 1 == 1));
        }
    }
    for chunk in bits.chunks(20) {
        let mut w = 1u32 << 20;
        for (i, &b) in chunk.iter().enumerate() {
            if b {
                w |= 1 << (19 - i);
            }
        }
        out.push(w);
    }
    // Pad to whole batches, since that is what a transmission is made of.
    while out.len() % BATCH_WORDS != 0 {
        out.push(idle);
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The encoder produces codeword *contents*; the framer adds the error
    /// correction. A test that wants to read its own transmission back has to
    /// go through it, which is also a check that the two layers agree about
    /// where the twenty-one bits sit.
    fn words(contents: Vec<u32>) -> Vec<u32> {
        contents.into_iter().map(dsp::pocsag::encode_codeword).collect()
    }

    /// A real off-air page, captured and decoded by somebody else's program.
    ///
    /// The codewords and the expected result are from a published capture at
    /// rfcandy.biz, decoded there by POC32: address 1238681, function 0,
    /// numeric message "1724". Agreement with a separate implementation on
    /// real bits is the only evidence that separates a correct decoder from
    /// one that is merely self-consistent, and every trap in this file is in
    /// this one page: the frame number carrying the low bits of the address,
    /// and the reversed character bits.
    #[test]
    fn a_captured_page_decodes_as_the_reference_program_read_it() {
        let mut batch = vec![IDLE; BATCH_WORDS];
        // Frame 1, so codewords 2 and 3 of the batch.
        batch[2] = 0b0100_1011_1001_1010_0110_0110_0000_0011;
        batch[3] = 0b1100_0111_0010_0001_0001_1110_0000_0010;
        let msgs = parse(&batch);
        assert_eq!(msgs.len(), 1);
        assert_eq!(msgs[0].address, 1_238_681, "the frame carries the low three bits");
        assert_eq!(msgs[0].function, 0);
        assert_eq!(msgs[0].body, Body::Numeric("1724".into()));
    }

    /// Reading the same codewords without reversing each character is the
    /// mistake this file exists to avoid, and it does not look like a
    /// mistake: it produces printable characters and a plausible page.
    #[test]
    fn the_unreversed_reading_of_that_page_is_not_the_right_one() {
        let data = 0b1000_1110_0100_0010_0011u32;
        let straight: String = (0..5)
            .map(|i| NUMERIC[(data >> (16 - 4 * i) & 0xF) as usize])
            .collect();
        assert_eq!(straight, "8)423", "plausible, printable, and wrong");
    }

    /// An alphanumeric page, round tripped through the encoder.
    ///
    /// Weaker evidence than the captured page and said so: a shared
    /// convention between encoder and decoder would survive it. What it does
    /// catch is the packing across codeword boundaries, since seven does not
    /// divide twenty and every character after the second straddles one.
    #[test]
    fn an_alphanumeric_page_survives_the_codeword_boundaries() {
        let text = "CALL CONTROL 4412";
        let msgs = parse(&words(encode(1_234_568, 3, &Body::Alpha(text.into()))));
        assert_eq!(msgs.len(), 1);
        assert_eq!(msgs[0].address, 1_234_568);
        assert_eq!(msgs[0].function, 3);
        assert_eq!(msgs[0].body, Body::Alpha(text.into()));
    }

    #[test]
    fn a_numeric_page_round_trips_with_its_symbols() {
        let msgs = parse(&words(encode(999_999, 0, &Body::Numeric("01-234U(56)".into()))));
        assert_eq!(msgs[0].body, Body::Numeric("01-234U(56)".into()));
        assert_eq!(msgs[0].address, 999_999);
    }

    /// Every frame position, because the address is only right if the frame
    /// the codeword sits in is read as part of it.
    #[test]
    fn an_address_is_recovered_from_every_frame() {
        for frame in 0..8u32 {
            let address = 0x1_2345 << 3 | frame;
            let w = words(encode(address, 1, &Body::Numeric("42".into())));
            assert_eq!(parse(&w)[0].address, address, "frame {frame}");
        }
    }

    /// Several pages in one transmission, which is the normal case: a
    /// transmitter empties its queue in one go.
    #[test]
    fn a_transmission_carrying_several_pages_yields_all_of_them() {
        let mut contents = encode(1_000_001, 3, &Body::Alpha("FIRST".into()));
        contents.extend(encode(2_000_002, 0, &Body::Numeric("112".into())));
        contents.extend(encode(1_500_003, 2, &Body::Tone));
        let msgs = parse(&words(contents));
        assert_eq!(msgs.len(), 3);
        assert_eq!(msgs[0].body, Body::Alpha("FIRST".into()));
        assert_eq!(msgs[1].body, Body::Numeric("112".into()));
        assert_eq!(msgs[2].body, Body::Tone, "an address with nothing after it is a beep");
        assert_eq!(msgs[2].address, 1_500_003);
    }

    /// Message codewords whose address was lost are not reported. There is
    /// nobody to attribute them to, and a page addressed to nobody is worse
    /// than no page: it looks like traffic on a pager that does not exist.
    #[test]
    fn orphaned_message_codewords_are_dropped() {
        let words = vec![0x8000_0000u32; 4];
        assert!(parse(&words).is_empty());
    }

    /// A batch of nothing is a batch of nothing.
    #[test]
    fn idle_batches_produce_no_pages() {
        assert!(parse(&[IDLE; BATCH_WORDS * 2]).is_empty());
        assert!(parse(&[]).is_empty());
    }
}
