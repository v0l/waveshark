//! The LoRa data layer: symbols to a payload.
//!
//! What comes off the dechirp is a run of numbers between 0 and 2^SF. Between
//! them and the bytes sit four transforms, all of them undone here. The
//! symbol is Gray coded, so neighbouring values differ in one bit and a
//! dechirp that lands one bin out costs one bit rather than several. The bits
//! are then interleaved diagonally across a block, so a symbol lost outright
//! spreads one bit into each codeword instead of destroying one. Each
//! codeword is Hamming, four data bits with one to four parity bits
//! depending on the coding rate, which is where 4/5 to 4/8 comes from.
//! Finally the payload is whitened against a fixed sequence so a run of zero
//! bytes is not a run of identical symbols.
//!
//! The first eight symbols are a block of their own and are read differently
//! from the rest: always at the strongest coding rate, and always two bits
//! narrower, because the header has to be readable before anything in it is
//! known. It carries the payload length, the coding rate the rest uses, and
//! whether there is a CRC, under a five bit checksum of its own.
//!
//! The payload CRC is the awkward one. It is computed over all but the last
//! two payload bytes and then those two are XORed into the result, which is
//! not a mistake in any implementation: it is what the silicon does, and a
//! receiver that computes an honest CRC over the whole payload disagrees with
//! every LoRa transmitter in existence.
//!
//! Checked against two off-air Meshtastic transmissions at SF11: both give a
//! valid header checksum and a payload CRC that matches the transmitter's.

/// Whitening sequence: an eight bit LFSR over x^8 + x^6 + x^5 + x^4 + 1,
/// seeded all ones, taken a byte per step. Building it beats a 255 byte
/// literal because the polynomial is the thing worth writing down.
const WHITENING: [u8; 255] = {
    let mut seq = [0u8; 255];
    let mut s: u8 = 0xff;
    let mut i = 0;
    while i < 255 {
        seq[i] = s;
        let fb = ((s >> 7) ^ (s >> 5) ^ (s >> 4) ^ (s >> 3)) & 1;
        s = (s << 1) | fb;
        i += 1;
    }
    seq
};

/// The header's five checksum bits over its twelve data bits, as the rows of
/// a parity matrix.
const HEADER_CHECKSUM: [u16; 5] = [
    0b1111_0000_0000,
    0b1000_1110_0001,
    0b0100_1001_1010,
    0b0010_0101_0111,
    0b0001_0010_1111,
];

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Header {
    pub length: usize,
    /// 1 to 4, meaning 4/5 to 4/8.
    pub coding_rate: u8,
    pub has_crc: bool,
}

#[derive(Clone, Debug)]
pub struct Frame {
    pub header: Header,
    pub payload: Vec<u8>,
    /// `None` when the header said there is no CRC, or when the symbols ran
    /// out before both CRC bytes arrived.
    pub crc_ok: Option<bool>,
    /// Bins added to every symbol before decoding. See [`decode`]: anything
    /// other than zero means the demodulator's timing was a fraction of a
    /// chip out and the header said so.
    pub bin_offset: i16,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Error {
    /// Fewer than the eight symbols a header block needs.
    Short,
    /// The header's own checksum did not match, so nothing after it can be
    /// trusted to be a length or a coding rate.
    BadHeaderChecksum,
    /// A coding rate the standard does not define.
    BadCodingRate,
}

/// Decode one packet's data symbols.
///
/// `ldro` is low data rate optimisation: above a 16 ms symbol the last two
/// bits of every symbol are discarded by the transmitter as unreliable, so
/// the payload blocks become as narrow as the header's.
///
/// Every symbol is tried at one bin either side of where the demodulator put
/// it, because those three are not distinguishable upstream. A dechirped
/// symbol's bin moves with the timing as well as with the value, one bin per
/// chip, so half a chip of timing error left over from the preamble is half
/// a bin on every symbol in the packet and lands some of them on the wrong
/// side of a rounding. It is the same offset for all of them, and the header
/// checksum with the payload CRC behind it says which one it was. That is
/// cheaper and steadier than chasing the last fraction of a chip in the
/// timing, and the offset it settled on is reported rather than hidden.
pub fn decode(symbols: &[u16], sf: u8, ldro: bool) -> Result<Frame, Error> {
    let mut first = None;
    for offset in [0i16, -1, 1] {
        match decode_at(symbols, sf, ldro, offset) {
            Ok(frame) if frame.crc_ok != Some(false) => return Ok(frame),
            Ok(frame) => first.get_or_insert(Ok(frame)),
            Err(e) => first.get_or_insert(Err(e)),
        };
    }
    first.unwrap_or(Err(Error::Short))
}

fn decode_at(symbols: &[u16], sf: u8, ldro: bool, offset: i16) -> Result<Frame, Error> {
    if symbols.len() < 8 {
        return Err(Error::Short);
    }
    let n = 1u32 << sf;
    let symbols: Vec<u16> = symbols
        .iter()
        .map(|&v| (v as i32 + offset as i32).rem_euclid(n as i32) as u16)
        .collect();
    let gray = |v: u16, reduced: bool| -> u16 {
        let w = if reduced { v / 4 } else { v % n as u16 };
        w ^ (w >> 1)
    };

    let head: Vec<u16> = symbols[..8].iter().map(|&v| gray(v, true)).collect();
    let nibbles = hamming(&deinterleave(&head, sf as usize - 2), 8);
    let header = parse_header(&nibbles)?;
    let ppm = if ldro { sf as usize - 2 } else { sf as usize };
    let rdd = header.coding_rate as usize + 4;

    let mut out: Vec<u8> = nibbles[5..].to_vec();
    let mut i = 8;
    while i + rdd <= symbols.len() {
        let block: Vec<u16> = symbols[i..i + rdd].iter().map(|&v| gray(v, ldro)).collect();
        out.extend(hamming(&deinterleave(&block, ppm), rdd));
        i += rdd;
    }

    let bytes: Vec<u8> = out.chunks_exact(2).map(|c| c[0] | (c[1] << 4)).collect();
    let take = header.length.min(bytes.len());
    let payload: Vec<u8> = bytes[..take]
        .iter()
        .enumerate()
        .map(|(j, b)| b ^ WHITENING[j % WHITENING.len()])
        .collect();

    // The CRC bytes are not whitened and are not part of the length, so they
    // sit past the payload in the raw stream.
    let crc_ok = if !header.has_crc || bytes.len() < header.length + 2 {
        None
    } else {
        let want = [bytes[header.length], bytes[header.length + 1]];
        Some(checksum(&payload) == want)
    };

    Ok(Frame {
        header,
        payload,
        crc_ok,
        bin_offset: offset,
    })
}

fn parse_header(nibbles: &[u8]) -> Result<Header, Error> {
    if nibbles.len() < 5 {
        return Err(Error::Short);
    }
    let bits = (nibbles[0] as u16) << 8 | (nibbles[1] as u16) << 4 | nibbles[2] as u16;
    let want = ((nibbles[3] as u16 & 1) << 4) | nibbles[4] as u16;
    let got = HEADER_CHECKSUM
        .iter()
        .enumerate()
        .fold(0u16, |acc, (i, row)| {
            acc | (((row & bits).count_ones() as u16 & 1) << (4 - i))
        });
    if got != want {
        return Err(Error::BadHeaderChecksum);
    }
    let coding_rate = nibbles[2] >> 1;
    if !(1..=4).contains(&coding_rate) {
        return Err(Error::BadCodingRate);
    }
    Ok(Header {
        length: (nibbles[0] as usize) << 4 | nibbles[1] as usize,
        coding_rate,
        has_crc: nibbles[2] & 1 != 0,
    })
}

/// Undo the diagonal interleave: bit `k` of symbol `i` is bit `i` of codeword
/// `k + i`, wrapped, so a block of `rdd` symbols `ppm` bits wide becomes
/// `ppm` codewords `rdd` bits wide.
fn deinterleave(symbols: &[u16], ppm: usize) -> Vec<u8> {
    let mut cw = vec![0u8; ppm];
    for (i, &s) in symbols.iter().enumerate() {
        for (k, w) in cw.iter_mut().enumerate() {
            let b = (k + i) % ppm;
            let bit = (s >> (ppm - 1 - b)) & 1;
            *w |= (bit as u8) << i;
        }
    }
    cw.reverse();
    cw
}

/// Hamming decode, one nibble per codeword.
///
/// At 4/5 and 4/6 the parity only detects, so the nibble is taken as it
/// stands; at 4/7 and 4/8 three syndromes place a single bit error and it is
/// corrected. A codeword whose parity fails at the weaker rates is not
/// reported, because the CRC below is what decides whether the frame is real
/// and a per-codeword flag no caller acts on is noise.
fn hamming(codewords: &[u8], rdd: usize) -> Vec<u8> {
    codewords
        .iter()
        .map(|&c| {
            let mut c = c;
            if rdd >= 7 {
                let bit = |p: u32| (c >> (p - 1)) & 1;
                let p2 = bit(7) ^ bit(4) ^ bit(2) ^ bit(1);
                let p3 = bit(5) ^ bit(3) ^ bit(2) ^ bit(1);
                let p5 = bit(6) ^ bit(4) ^ bit(3) ^ bit(2);
                c ^= match p2 << 2 | p3 << 1 | p5 {
                    0b011 => 0b0100,
                    0b101 => 0b1000,
                    0b110 => 0b0001,
                    0b111 => 0b0010,
                    _ => 0,
                };
            }
            c & 0xf
        })
        .collect()
}

/// The payload CRC as a transmitter computes it: CRC-16/CCITT over all but
/// the last two bytes, with those two XORed into the result.
pub fn checksum(payload: &[u8]) -> [u8; 2] {
    match payload {
        [] => [0, 0],
        [a] => [*a, 0],
        [a, b] => [*b, *a],
        _ => {
            let mut crc: u16 = 0;
            for b in &payload[..payload.len() - 2] {
                crc ^= (*b as u16) << 8;
                for _ in 0..8 {
                    crc = if crc & 0x8000 != 0 {
                        (crc << 1) ^ 0x1021
                    } else {
                        crc << 1
                    };
                }
            }
            let n = payload.len();
            [
                (crc as u8) ^ payload[n - 1],
                ((crc >> 8) as u8) ^ payload[n - 2],
            ]
        }
    }
}

/// The sync word Meshtastic uses, which is the whole of what identifies it
/// on the air. LoRaWAN uses 0x34 and a private network 0x12.
pub const MESHTASTIC_SYNC: u8 = 0x2b;

/// What a decoded frame looks like on the packet bus.
///
/// A LoRa payload says nothing about itself: it is bytes, it can be anywhere
/// from 433 to 915 MHz, and the parameters that make it readable are not in
/// it. So the front end writes them in front, the way a pager transmission
/// carries its codewords and an M17 transmission its link setup, and
/// everything downstream reads one thing rather than being told out of band.
/// The tag is text because the first thing anyone does with an unfamiliar
/// frame is look at it in hex.
pub const TAG: [u8; 4] = *b"LoRa";

/// Bytes before the payload: the tag, the spreading factor, the bandwidth in
/// kilohertz, the coding rate, the sync word and the two check flags.
const ENVELOPE: usize = 4 + 1 + 2 + 1 + 1 + 1;

impl Frame {
    /// The frame as it travels: parameters, then the payload as sent.
    pub fn to_bytes(&self, sf: u8, bandwidth_hz: f64, sync_word: u8) -> Vec<u8> {
        let mut out = Vec::with_capacity(ENVELOPE + self.payload.len());
        out.extend_from_slice(&TAG);
        out.push(sf);
        out.extend_from_slice(&((bandwidth_hz / 1e3).round() as u16).to_le_bytes());
        out.push(self.header.coding_rate);
        out.push(sync_word);
        out.push(match self.crc_ok {
            Some(true) => 2,
            Some(false) => 1,
            None => 0,
        });
        out.extend_from_slice(&self.payload);
        out
    }
}

/// What a frame off the bus was: its parameters and its payload.
#[derive(Clone, Debug, PartialEq)]
pub struct Received {
    pub sf: u8,
    pub bandwidth_hz: f64,
    pub coding_rate: u8,
    pub sync_word: u8,
    pub crc_ok: Option<bool>,
    pub payload: Vec<u8>,
}

impl Received {
    /// Read one back, refusing anything that is not one.
    ///
    /// The tag alone is four bytes any burst could start with once in four
    /// billion, so the fields are checked as well: a spreading factor and a
    /// bandwidth LoRa does not have, or a coding rate outside 4/5 to 4/8, is
    /// not a LoRa frame however it starts.
    pub fn parse(bytes: &[u8]) -> Option<Self> {
        if bytes.len() < ENVELOPE || bytes[..4] != TAG {
            return None;
        }
        let sf = bytes[4];
        let khz = u16::from_le_bytes([bytes[5], bytes[6]]);
        let coding_rate = bytes[7];
        if !(7..=12).contains(&sf) || !(1..=4).contains(&coding_rate) || khz == 0 {
            return None;
        }
        Some(Self {
            sf,
            bandwidth_hz: f64::from(khz) * 1e3,
            coding_rate,
            sync_word: bytes[8],
            crc_ok: match bytes[9] {
                2 => Some(true),
                1 => Some(false),
                _ => None,
            },
            payload: bytes[ENVELOPE..].to_vec(),
        })
    }

    pub fn meshtastic(&self) -> Option<Meshtastic> {
        (self.sync_word == MESHTASTIC_SYNC).then(|| Meshtastic::parse(&self.payload)).flatten()
    }

    /// What the packet says, where the public default channel key opens it.
    ///
    /// The key is tried on every Meshtastic packet rather than only where the
    /// channel hash names a default preset: the hash is one byte and a
    /// private channel can collide with a preset, while a channel renamed but
    /// left on the default key does not match any preset and is still
    /// readable. What decides is whether the plaintext parses, which
    /// [`meshtastic::Decoded::of`] insists on.
    pub fn meshtastic_message(&self) -> Option<crate::meshtastic::Decoded> {
        let m = self.meshtastic()?;
        let ciphertext = self.payload.get(Meshtastic::HEADER..)?;
        crate::meshtastic::Decoded::of(
            ciphertext,
            m.source,
            m.packet_id,
            &crate::meshtastic::DEFAULT_KEY,
        )
    }
}

/// The sixteen byte header in front of every Meshtastic payload.
///
/// Everything past it is AES encrypted with the channel's key, so this is as
/// far as a receiver without that key reads. It is worth reading anyway: who
/// transmitted, who it was for, and how far it has already travelled is most
/// of what a mesh is interesting for.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Meshtastic {
    pub destination: u32,
    pub source: u32,
    pub packet_id: u32,
    /// Hops left before the packet is dropped.
    pub hop_limit: u8,
    /// Hops it was sent with, so limit against start is how far it came.
    pub hop_start: u8,
    pub want_ack: bool,
    pub via_mqtt: bool,
    /// One byte of the channel name's hash, not the channel itself.
    pub channel_hash: u8,
}

impl Meshtastic {
    /// Bytes of cleartext header in front of the encrypted payload.
    pub const HEADER: usize = 16;

    pub fn parse(payload: &[u8]) -> Option<Self> {
        if payload.len() < Self::HEADER {
            return None;
        }
        let word = |i: usize| {
            let mut b = [0u8; 4];
            b.copy_from_slice(&payload[i..i + 4]);
            u32::from_le_bytes(b)
        };
        let flags = payload[12];
        Some(Self {
            destination: word(0),
            source: word(4),
            packet_id: word(8),
            hop_limit: flags & 0x07,
            want_ack: flags & 0x08 != 0,
            via_mqtt: flags & 0x10 != 0,
            hop_start: (flags >> 5) & 0x07,
            channel_hash: payload[13],
        })
    }

    pub fn is_broadcast(&self) -> bool {
        self.destination == u32::MAX
    }

    /// The name of a well-known channel this hash could be, when it matches
    /// one of the modem presets carried with the default key.
    ///
    /// The hash is one byte, `xorHash(name) ^ xorHash(psk)`, so a match is a
    /// strong hint, not a certainty: a private channel whose name and key
    /// happen to xor to the same byte reads the same on air. What a match
    /// does mean is that the payload behind this header is very likely
    /// readable, because the default key is public. The default primary
    /// channel almost every node ships with is LongFast on that key, 0x08.
    pub fn well_known_channel(&self) -> Option<&'static str> {
        Some(match self.channel_hash {
            0x08 => "LongFast (default key)",
            0x0f => "LongSlow (default key)",
            0x37 => "VeryLongSlow (default key)",
            0x6e => "LongModerate (default key)",
            0x18 => "MediumSlow (default key)",
            0x1f => "MediumFast (default key)",
            0x77 => "ShortSlow (default key)",
            0x70 => "ShortFast (default key)",
            _ => return None,
        })
    }
}

/// How many data symbols a packet of this shape occupies, which is what says
/// whether a header's length agrees with the transmission it came from.
pub fn symbol_count(length: usize, sf: u8, coding_rate: u8, has_crc: bool, ldro: bool) -> usize {
    let bits = 8 * length as isize - 4 * sf as isize + 28 + if has_crc { 16 } else { 0 };
    let per = 4 * (sf as isize - if ldro { 2 } else { 0 });
    let blocks = if bits <= 0 { 0 } else { (bits + per - 1) / per };
    8 + blocks as usize * (coding_rate as usize + 4)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_whitening_sequence_starts_where_it_should() {
        assert_eq!(
            &WHITENING[..8],
            &[0xff, 0xfe, 0xfc, 0xf8, 0xf0, 0xe1, 0xc2, 0x85]
        );
        assert_eq!(WHITENING[254], 0x7f);
        // A maximal length LFSR visits every non-zero state exactly once.
        let mut seen = [false; 256];
        for b in WHITENING {
            assert!(!seen[b as usize], "0x{b:02x} twice");
            seen[b as usize] = true;
        }
    }

    #[test]
    fn a_symbol_count_matches_the_shape_it_describes() {
        // The two fixtures: 55 and 51 byte payloads at SF11 4/5 with a CRC,
        // both of which were 58 symbols on the air.
        assert_eq!(symbol_count(55, 11, 1, true, false), 58);
        assert_eq!(symbol_count(51, 11, 1, true, false), 58);
    }

    #[test]
    fn a_header_with_a_broken_checksum_is_refused() {
        let mut symbols = [0u16; 8];
        symbols[3] = 1;
        match decode(&symbols, 11, false) {
            Err(Error::BadHeaderChecksum) | Err(Error::BadCodingRate) => {}
            other => panic!("expected a refusal, got {other:?}"),
        }
    }

    #[test]
    fn too_few_symbols_for_a_header_is_short() {
        assert_eq!(decode(&[0; 7], 11, false).unwrap_err(), Error::Short);
    }

    #[test]
    fn the_default_channel_is_recognised_by_its_hash() {
        // xorHash("LongFast") ^ xorHash(default key) = 0x08, the hash almost
        // every node ships transmitting on. Build a minimal broadcast header
        // carrying it and check it is named.
        let mut payload = [0u8; 16];
        payload[13] = 0x08;
        let m = Meshtastic::parse(&payload).unwrap();
        assert_eq!(m.well_known_channel(), Some("LongFast (default key)"));
        payload[13] = 0x5b;
        assert_eq!(Meshtastic::parse(&payload).unwrap().well_known_channel(), None);
    }
}
