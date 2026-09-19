//! nRF24L01 ShockBurst and the XN297 clone, which is what a toy quadcopter's
//! remote transmits.
//!
//! Almost every cheap 2.4 GHz remote is one of two chips: a Nordic nRF24L01
//! or a Panchip XN297 that behaves like one. Both key GFSK at 250 kbit/s or
//! 1 Mbit/s, hop across the band, and send a frame of a preamble, an address,
//! a payload and a CRC. The protocols above them (Bayang, E010, Syma, Hubsan,
//! and the rest of the MultiProtocol list) differ only in what the payload
//! means.
//!
//! # Why the XN297 is worth reading and the plain nRF24 mostly is not
//!
//! A ShockBurst frame begins with a one byte preamble and then goes straight
//! into an address the receiver was told in advance. A listener that does not
//! know the address has one byte of known bits to lock onto, which is not
//! enough: the address is the sync word, and it is different for every toy.
//!
//! The XN297 fixes that for us. It sends a 28 bit preamble of its own,
//! `0xC710F55`, before the address, so a packet announces itself; the address
//! is then scrambled with a published table rather than kept secret, and the
//! payload bytes are bit reversed and scrambled with the same table. A CRC-16
//! covers the lot with a length dependent xorout. So a listener with no prior
//! knowledge can find the packet, recover the address, and check it, which is
//! exactly what the plain chip does not allow.
//!
//! Tables and layout from `pascallanger/DIY-Multiprotocol-TX-Module`
//! (`XN297_EMU.ino`), whose emulation real toys bind to.

use common::Decoded;
use common::Value;
use dsp::FirDecim;
use dsp::fsk::BitSync;

/// The XN297's fixed preamble, most significant bit first: 28 bits.
pub const PREAMBLE: u32 = 0x0c71_0f55;
pub const PREAMBLE_BITS: usize = 28;

/// Address, payload and CRC are all XORed with this, byte by byte.
const SCRAMBLE: [u8; 39] = [
    0xe3, 0xb1, 0x4b, 0xea, 0x85, 0xbc, 0xe5, 0x66, 0x0d, 0xae, 0x8c, 0x88, 0x12, 0x69, 0xee, 0x1f,
    0xc7, 0x62, 0x97, 0xd5, 0x0b, 0x79, 0xca, 0xcc, 0x1b, 0x5d, 0x19, 0x10, 0x24, 0xd3, 0xdc, 0x3f,
    0x8e, 0xc5, 0x2f, 0xaa, 0x16, 0xf3, 0x95,
];

/// What the CRC is XORed with at the end, indexed by address length plus
/// payload length minus three. Scrambled and unscrambled links use different
/// tables, which is the chip making the two incompatible on purpose.
const XOROUT_SCRAMBLED: [u16; 35] = [
    0x0000, 0x3448, 0x9ba7, 0x8bbb, 0x85e1, 0x3e8c, 0x451e, 0x18e6, 0x6b24, 0xe7ab, 0x3828, 0x814b,
    0xd461, 0xf494, 0x2503, 0x691d, 0xfe8b, 0x9ba7, 0x8b17, 0x2920, 0x8b5f, 0x61b1, 0xd391, 0x7401,
    0x2138, 0x129f, 0xb3a0, 0x2988, 0x23ca, 0xc0cb, 0x0c6c, 0xb329, 0xa0a1, 0x0a16, 0xa9d0,
];

const XOROUT_PLAIN: [u16; 35] = [
    0x0000, 0x3d5f, 0xa6f1, 0x3a23, 0xaa16, 0x1caf, 0x62b2, 0xe0eb, 0x0821, 0xbe07, 0x5f1a, 0xaf15,
    0x4f0a, 0xad24, 0x5e48, 0xed34, 0x068c, 0xf2c9, 0x1852, 0xdf36, 0x129d, 0xb17c, 0xd5f5, 0x70d7,
    0xb798, 0x5133, 0x67db, 0xd94e, 0x0a5b, 0xe445, 0xe6a5, 0x26e7, 0xbdab, 0xc379, 0x8e20,
];

fn bit_reverse(b: u8) -> u8 {
    b.reverse_bits()
}

/// CRC-16 CCITT with the chip's own starting value.
fn crc16(data: &[u8]) -> u16 {
    let mut crc = 0xb5d2u16;
    for &b in data {
        crc ^= u16::from(b) << 8;
        for _ in 0..8 {
            crc = if crc & 0x8000 != 0 { (crc << 1) ^ 0x1021 } else { crc << 1 };
        }
    }
    crc
}

/// One packet, as it was on the air.
#[derive(Clone, Debug, PartialEq)]
pub struct Packet {
    /// The receiver's address, descrambled: the identity of the toy, and what
    /// the transmitter is bound to.
    pub address: Vec<u8>,
    pub payload: Vec<u8>,
    /// Whether the link scrambles, which nearly all of them do.
    pub scrambled: bool,
    /// The bytes as they were on the air, address and payload together,
    /// still scrambled and not bit reversed.
    ///
    /// This is the part that is actually determined. Descrambling and bit
    /// reversal both depend on where the address is taken to end, so two
    /// readings of the same frame under different splits give different
    /// address and payload bytes but the same `raw`. Compare packets on this.
    pub raw: Vec<u8>,
    /// The check that decided all of it.
    pub crc: u16,
    /// Where in the bit stream the preamble began.
    pub start: usize,
}

impl Packet {
    /// Always true, and here to be read rather than to be checked: the CRC
    /// cannot separate the address from the payload, so the address length is
    /// the most likely one rather than a measurement. A caller comparing two
    /// packets should compare the address and payload together.
    pub fn split_is_a_guess(&self) -> bool {
        true
    }

    /// The frame as it went out, everything after the preamble: still
    /// scrambled, still in the order the chip sent it, check included.
    ///
    /// This is what a front end puts on the bus, because it is the part that
    /// is determined. Anything that reads it back gets the same CRC to check
    /// and the same split to guess at.
    pub fn on_air(&self) -> Vec<u8> {
        let mut out = self.raw.clone();
        out.extend(self.crc.to_be_bytes());
        out
    }

    /// How many bits the frame occupied, preamble and all.
    pub fn bits(&self) -> usize {
        PREAMBLE_BITS + (self.raw.len() + 2) * 8
    }
}

/// Read a packet out of the bytes a front end put on the bus, which are the
/// frame without its preamble. The CRC is checked again here: a reader that
/// took the front end's word for it would have no evidence of its own.
pub fn from_on_air(bytes: &[u8]) -> Option<Packet> {
    let mut bits: Vec<bool> =
        (0..PREAMBLE_BITS).map(|k| PREAMBLE >> (PREAMBLE_BITS - 1 - k) & 1 != 0).collect();
    for b in bytes {
        bits.extend((0..8).rev().map(|k| b >> k & 1 != 0));
    }
    decode(&bits, 0)
}

/// Read one XN297 packet from a bit stream, most significant bit first.
///
/// Neither the address length nor the payload length is transmitted, so both
/// are searched: three to five address bytes, one to thirty-two payload
/// bytes, both scrambled and not. The CRC-16 decides.
///
/// It cannot decide everything. The CRC covers the address and the payload
/// together and its xorout is indexed by the two added, so moving a byte from
/// one to the other leaves both unchanged: a five byte address with a fifteen
/// byte payload checks exactly as well as a three byte address with a
/// seventeen byte payload. The split is genuinely not in the signal. Five is
/// tried first because it is what nearly every toy uses, and
/// [`Packet::split_is_a_guess`] says so rather than letting a caller believe
/// the address is measured.
pub fn decode(bits: &[bool], from: usize) -> Option<Packet> {
    let start = find_preamble(bits, from)?;
    let after = start + PREAMBLE_BITS;
    let byte_at = |n: usize| -> Option<u8> {
        let at = after + n * 8;
        (at + 8 <= bits.len()).then(|| (0..8).fold(0u8, |a, k| (a << 1) | u8::from(bits[at + k])))
    };
    for addr_len in (3..=5usize).rev() {
        for payload_len in 1..=32usize {
            let total = addr_len + payload_len + 2;
            let raw: Option<Vec<u8>> = (0..total).map(byte_at).collect();
            let Some(raw) = raw else { continue };
            for scrambled in [true, false] {
                let table = if scrambled { &XOROUT_SCRAMBLED } else { &XOROUT_PLAIN };
                let Some(&xorout) = table.get(addr_len - 3 + payload_len) else {
                    continue;
                };
                let crc = crc16(&raw[..total - 2]) ^ xorout;
                let sent = (u16::from(raw[total - 2]) << 8) | u16::from(raw[total - 1]);
                if crc != sent {
                    continue;
                }
                // The address travels most significant byte first and
                // scrambled from the front of the table; the payload
                // continues through the same table, bit reversed.
                let mut address: Vec<u8> = (0..addr_len)
                    .map(|i| raw[i] ^ if scrambled { SCRAMBLE[i] } else { 0 })
                    .collect();
                address.reverse();
                let payload = (0..payload_len)
                    .map(|i| {
                        let b =
                            raw[addr_len + i] ^ if scrambled { SCRAMBLE[addr_len + i] } else { 0 };
                        bit_reverse(b)
                    })
                    .collect();
                return Some(Packet {
                    address,
                    payload,
                    scrambled,
                    raw: raw[..total - 2].to_vec(),
                    crc: sent,
                    start,
                });
            }
        }
    }
    None
}

/// Where an XN297 preamble starts, at or after `from`.
pub fn find_preamble(bits: &[bool], from: usize) -> Option<usize> {
    if bits.len() < PREAMBLE_BITS {
        return None;
    }
    (from..=bits.len() - PREAMBLE_BITS).find(|&i| {
        (0..PREAMBLE_BITS).all(|k| bits[i + k] == (PREAMBLE >> (PREAMBLE_BITS - 1 - k) & 1 != 0))
    })
}

/// Build a packet the way a transmitter does, for testing a demodulator
/// against something other than this file's own reader.
pub fn encode(address: &[u8], payload: &[u8], scrambled: bool) -> Vec<bool> {
    let mut raw: Vec<u8> = Vec::new();
    for (i, b) in address.iter().rev().enumerate() {
        raw.push(b ^ if scrambled { SCRAMBLE[i] } else { 0 });
    }
    for (i, b) in payload.iter().enumerate() {
        let s = if scrambled { SCRAMBLE[address.len() + i] } else { 0 };
        raw.push(bit_reverse(*b) ^ s);
    }
    let table = if scrambled { &XOROUT_SCRAMBLED } else { &XOROUT_PLAIN };
    let crc = crc16(&raw) ^ table[address.len() - 3 + payload.len()];
    raw.push((crc >> 8) as u8);
    raw.push(crc as u8);

    let mut bits: Vec<bool> =
        (0..PREAMBLE_BITS).map(|k| PREAMBLE >> (PREAMBLE_BITS - 1 - k) & 1 != 0).collect();
    for b in raw {
        for k in (0..8).rev() {
            bits.push(b >> k & 1 != 0);
        }
    }
    bits
}

/// The channel an nRF24 register value names. The chip tunes a megahertz a
/// step from 2400 MHz.
pub fn channel_hz(channel: u8) -> f64 {
    2_400e6 + f64::from(channel) * 1e6
}

/// The fields a log or a bus carries.
pub fn fields(p: &Packet) -> Vec<(String, Value)> {
    let hex = |b: &[u8]| b.iter().map(|x| format!("{x:02x}")).collect::<String>();
    vec![
        ("address".into(), Value::Text(hex(&p.address))),
        ("payload_len".into(), Value::Int(p.payload.len() as i64)),
        ("payload".into(), Value::Text(hex(&p.payload))),
        ("scrambled".into(), Value::Bool(p.scrambled)),
    ]
}

/// The row a frame off the bus becomes.
pub fn decoded(bytes: &[u8], center: common::Hz) -> Option<Decoded> {
    use common::Value;
    let p = from_on_air(bytes)?;
    let mut fields = fields(&p);
    if let Some(ch) = channel_of(center.as_f64()) {
        fields.insert(0, ("channel".into(), Value::Int(i64::from(ch))));
    }
    let address =
        fields.iter().find(|(k, _)| k == "address").map(|(_, v)| v.to_string()).unwrap_or_default();
    let detail = fields.iter().map(|(k, v)| format!("{k}={v}")).collect::<Vec<_>>().join(" ");
    Some(
        Decoded::bytes("XN297", center, 0.0, bytes.to_vec())
            .by(common::Identity::new("nrf24", address.clone()))
            .with_link(common::Link { from: Some(common::Party::unit(address)), to: None })
            .with_detail(detail)
            .with_fields(fields)
            .with_modulation(common::Modulation::Gfsk)
            // The CRC-16 was checked again here, on the bytes in the row,
            // rather than taken on trust from whatever put them on the bus.
            .with_crc(Some(true)),
    )
}

/// The channel index a centre names, as the chip's own register value.
pub fn channel_of(center_hz: f64) -> Option<u8> {
    let ch = ((center_hz - BAND.0) / 1e6).round();
    (0.0..=125.0).contains(&ch).then_some(ch as u8)
}

/// Where the chip can tune: a megahertz a step from 2400 MHz, 126 channels.
pub const BAND: (f64, f64) = (2_400_000_000.0, 2_526_000_000.0);

/// The two bit rates an XN297 keys. The chip supports no others.
pub const BAUDS: [f64; 2] = [250_000.0, 1_000_000.0];

/// One bit rate's clock and the bits it has produced but not yet read a
/// frame out of.
pub struct Reader {
    /// Down to four samples a symbol for *this* bit rate.
    ///
    /// The stream both clocks are handed is four samples a symbol at the
    /// faster one, which is sixteen at the slower, and a bit clock's channel
    /// filter is designed against its own baud: at 4 MS/s the 250 kbit
    /// filter is 119 taps where at 1 MS/s it is 31, and it runs over a
    /// quarter as many samples. Fifteen times the arithmetic for the same
    /// bits.
    decim: Option<FirDecim>,
    narrow: Vec<common::C32>,
    sync: BitSync,
    bits: Vec<bool>,
    /// Bits dropped off the front, so a frame's position stays a position in
    /// the stream rather than in what is left of it.
    dropped: u64,
    /// Where the search has reached, counted in the same stream positions.
    /// The tail is kept for a frame that is still arriving, so without this
    /// the frame at the end of one block is read again out of the next.
    read_from: u64,
}

impl Reader {
    pub fn new(rate: f64, baud: f64) -> Self {
        // A GFSK link at modulation index 0.64 occupies about 1.6 times its
        // baud, and the filter in the bit clock is what keeps the rest of
        // the channel's noise out of the discriminator.
        let occupied = 1.6 * baud;
        let factor = (rate / (baud * SPS)).floor().max(1.0) as usize;
        let work = rate / factor as f64;
        Self {
            decim: (factor > 1).then(|| FirDecim::design_hz(rate, factor, occupied / 2.0, 60.0)),
            narrow: Vec::new(),
            sync: BitSync::with_bandwidth(work, baud, occupied),
            bits: Vec::new(),
            dropped: 0,
            read_from: 0,
        }
    }

    /// Demodulate a block and hand back every frame that closed inside it,
    /// each with the bit it started at.
    /// Whether this rate's clock can run on the stream it was given.
    pub fn usable(&self) -> bool {
        self.sync.usable()
    }

    pub fn read(&mut self, iq: &[common::C32], out: &mut Vec<(u64, Packet)>) {
        if !self.sync.usable() {
            return;
        }
        match &mut self.decim {
            Some(d) => {
                self.narrow.clear();
                d.process(iq, &mut self.narrow);
                self.sync.process(&self.narrow, &mut self.bits);
            }
            None => self.sync.process(iq, &mut self.bits),
        }
        let mut from = (self.read_from - self.dropped) as usize;
        while let Some(at) = find_preamble(&self.bits, from) {
            // A preamble too near the end may be a frame still arriving, so
            // leave it for the next block rather than deciding on half of it.
            if self.bits.len() - at < MAX_FRAME_BITS {
                from = at;
                break;
            }
            match decode(&self.bits, at) {
                Some(p) => {
                    from = at + p.bits();
                    out.push((self.dropped + at as u64, p));
                }
                None => from = at + 1,
            }
        }
        self.read_from = self.dropped + from as u64;
        let keep = self.bits.len().min(KEEP_BITS);
        let cut = self.bits.len() - keep;
        if cut > 0 {
            self.bits.drain(..cut);
            self.dropped += cut as u64;
            self.read_from = self.read_from.max(self.dropped);
        }
    }

    pub fn reset(&mut self) {
        if let Some(d) = &mut self.decim {
            d.reset();
        }
        self.narrow.clear();
        self.sync.reset();
        self.bits.clear();
        self.dropped = 0;
        self.read_from = 0;
    }
}

/// Samples a symbol each bit clock is fed, which is where [`BitSync`] stops.
pub const SPS: f64 = 4.0;

/// The longest frame the chip sends: five address bytes, thirty-two of
/// payload and the check, behind the preamble.
pub const MAX_FRAME_BITS: usize = PREAMBLE_BITS + (5 + 32 + 2) * 8;

/// Bits kept behind the search so a frame split across two blocks is still
/// whole when the second arrives.
pub const KEEP_BITS: usize = MAX_FRAME_BITS * 2;

#[cfg(test)]
mod tests {
    use super::*;

    /// A Bayang remote's shape: five byte address, fifteen byte payload,
    /// scrambled, which is what most toy quadcopters send.
    #[test]
    fn a_packet_survives_the_round_trip() {
        let addr = [0xa4, 0x03, 0x55, 0x11, 0x22];
        let payload: Vec<u8> = (0..15).map(|i| i * 17 + 3).collect();
        for scrambled in [true, false] {
            let bits = encode(&addr, &payload, scrambled);
            let p = decode(&bits, 0).unwrap_or_else(|| panic!("scrambled={scrambled}: no packet"));
            assert_eq!(p.address, addr, "the address came back changed");
            assert_eq!(p.payload, payload, "the payload came back changed");
            assert_eq!(p.scrambled, scrambled);
        }
    }

    /// The total length is recovered without being told, which is the part
    /// the CRC can settle: a frame two bytes longer or shorter gets a
    /// different xorout and fails. Where the address ends and the payload
    /// begins it cannot settle, which the test above records.
    #[test]
    fn the_total_length_is_recovered_without_being_told() {
        for addr_len in 3..=5usize {
            for payload_len in [1usize, 7, 15, 32] {
                let addr: Vec<u8> = (0..addr_len).map(|i| 0x11 * (i as u8 + 1)).collect();
                let payload: Vec<u8> = (0..payload_len).map(|i| i as u8 ^ 0x5a).collect();
                let bits = encode(&addr, &payload, true);
                let p = decode(&bits, 0).unwrap_or_else(|| {
                    panic!("{addr_len} byte address, {payload_len} byte payload")
                });
                assert_eq!(
                    p.address.len() + p.payload.len(),
                    addr_len + payload_len,
                    "the frame came back a different length"
                );
                assert_eq!(p.raw.len(), addr_len + payload_len);
            }
        }
    }

    /// A packet inside a longer stream, with noise either side, which is what
    /// a demodulator hands over.
    #[test]
    fn a_packet_is_found_inside_a_stream() {
        let mut seed = 0x9e37_79b9_7f4a_7c15u64;
        let mut noise = |n: usize| -> Vec<bool> {
            (0..n)
                .map(|_| {
                    seed ^= seed << 13;
                    seed ^= seed >> 7;
                    seed ^= seed << 17;
                    seed & 1 == 0
                })
                .collect()
        };
        let mut bits = noise(500);
        let addr = [0xcc, 0xcc, 0xcc, 0xcc, 0xcc];
        bits.extend(encode(&addr, &[1, 2, 3, 4], true));
        bits.extend(noise(500));
        let p = decode(&bits, 0).expect("a packet in the stream");
        assert_eq!(p.address, addr);
        assert_eq!(p.payload, vec![1, 2, 3, 4]);
    }

    /// The CRC covers the address and the payload together and its xorout is
    /// indexed by their sum, so where one ends and the other begins is not in
    /// the signal at all. A decoder that reported the split as fact would be
    /// inventing it.
    #[test]
    fn the_split_between_address_and_payload_is_not_determinable() {
        let addr = [0xa4, 0x03, 0x55, 0x11, 0x22];
        let payload: Vec<u8> = (0..15).map(|i| i * 17 + 3).collect();
        let bits = encode(&addr, &payload, true);
        let p = decode(&bits, 0).expect("a packet");
        // Five bytes is what it guesses, and it is right here because that is
        // what was sent, but the reason is that five is tried first.
        assert_eq!(p.address.len(), 5);
        assert!(p.split_is_a_guess());
        // What is determined is the frame as it was on the air, which does
        // not depend on the split.
        assert_eq!(p.raw.len(), addr.len() + payload.len());
    }

    #[test]
    fn one_wrong_bit_is_refused() {
        let addr = [0xa4, 0x03, 0x55, 0x11, 0x22];
        let payload = vec![9u8; 15];
        for bit in [40usize, 90, 150] {
            let mut bits = encode(&addr, &payload, true);
            let at = PREAMBLE_BITS + bit;
            bits[at] = !bits[at];
            assert!(decode(&bits, 0).is_none(), "bit {bit} was not noticed");
        }
    }

    /// The search tries three address lengths, thirty-two payload lengths and
    /// both scramblings, which is 192 chances for a sixteen bit CRC to pass
    /// by accident: one in 341 rather than one in 65536. Worth measuring
    /// rather than assuming, and worth knowing before treating a single
    /// packet as proof.
    #[test]
    fn noise_behind_a_preamble_rarely_passes() {
        let mut seed = 0x2545_f491_4f6c_dd1du64;
        let mut passed = 0;
        let trials = 5_000;
        for _ in 0..trials {
            let mut bits: Vec<bool> =
                (0..PREAMBLE_BITS).map(|k| PREAMBLE >> (PREAMBLE_BITS - 1 - k) & 1 != 0).collect();
            bits.extend((0..400).map(|_| {
                seed ^= seed << 13;
                seed ^= seed >> 7;
                seed ^= seed << 17;
                seed & 1 == 0
            }));
            if decode(&bits, 0).is_some() {
                passed += 1;
            }
        }
        let rate = passed as f64 / trials as f64;
        assert!(rate < 0.02, "{passed} of {trials} noise packets passed, {rate:.4}");
    }

    /// What a front end puts on the bus is what a reader of the bus decodes
    /// again, check and all.
    #[test]
    fn a_frame_off_the_bus_reads_back_the_same() {
        let addr = [0xa4, 0x03, 0x55, 0x11, 0x22];
        let payload: Vec<u8> = (0..15).map(|i| i * 7 + 1).collect();
        let bits = encode(&addr, &payload, true);
        let p = decode(&bits, 0).expect("a packet");
        assert_eq!(p.bits(), bits.len());
        let again = from_on_air(&p.on_air()).expect("the same packet off the bus");
        assert_eq!(again.address, addr);
        assert_eq!(again.payload, payload);
        assert_eq!(again.crc, p.crc);
        // A byte changed anywhere fails the check rather than reading as
        // some other length.
        let mut bent = p.on_air();
        bent[4] ^= 0x10;
        assert!(from_on_air(&bent).is_none());
    }

    #[test]
    fn the_channels_are_a_megahertz_apart_from_2400() {
        assert_eq!(channel_hz(0), 2_400e6);
        assert_eq!(channel_hz(83), 2_483e6);
    }
}
