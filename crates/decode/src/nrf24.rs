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

use common::Value;

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
            crc = if crc & 0x8000 != 0 {
                (crc << 1) ^ 0x1021
            } else {
                crc << 1
            };
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
        (at + 8 <= bits.len())
            .then(|| (0..8).fold(0u8, |a, k| (a << 1) | u8::from(bits[at + k])))
    };
    for addr_len in (3..=5usize).rev() {
        for payload_len in 1..=32usize {
            let total = addr_len + payload_len + 2;
            let raw: Option<Vec<u8>> = (0..total).map(byte_at).collect();
            let Some(raw) = raw else { continue };
            for scrambled in [true, false] {
                let table = if scrambled {
                    &XOROUT_SCRAMBLED
                } else {
                    &XOROUT_PLAIN
                };
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
                        let b = raw[addr_len + i]
                            ^ if scrambled { SCRAMBLE[addr_len + i] } else { 0 };
                        bit_reverse(b)
                    })
                    .collect();
                return Some(Packet {
                    address,
                    payload,
                    scrambled,
                    raw: raw[..total - 2].to_vec(),
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
        let s = if scrambled {
            SCRAMBLE[address.len() + i]
        } else {
            0
        };
        raw.push(bit_reverse(*b) ^ s);
    }
    let table = if scrambled {
        &XOROUT_SCRAMBLED
    } else {
        &XOROUT_PLAIN
    };
    let crc = crc16(&raw) ^ table[address.len() - 3 + payload.len()];
    raw.push((crc >> 8) as u8);
    raw.push(crc as u8);

    let mut bits: Vec<bool> = (0..PREAMBLE_BITS)
        .map(|k| PREAMBLE >> (PREAMBLE_BITS - 1 - k) & 1 != 0)
        .collect();
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
            let mut bits: Vec<bool> = (0..PREAMBLE_BITS)
                .map(|k| PREAMBLE >> (PREAMBLE_BITS - 1 - k) & 1 != 0)
                .collect();
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
        assert!(
            rate < 0.02,
            "{passed} of {trials} noise packets passed, {rate:.4}"
        );
    }

    #[test]
    fn the_channels_are_a_megahertz_apart_from_2400() {
        assert_eq!(channel_hz(0), 2_400e6);
        assert_eq!(channel_hz(83), 2_483e6);
    }
}
