//! The SX128x long interleaved coding, which is what ExpressLRS 2.4 GHz uses
//! and what no data sheet describes.
//!
//! Semtech names `CR_LI_4_5`, `CR_LI_4_6` and `CR_LI_4_8` and says nothing
//! about them, ExpressLRS writes the register and lets the modem do it, and no
//! open decoder implements them. What is here was measured off a RadioMaster
//! RP2 by transmitting payloads chosen one bit at a time and reading where
//! each bit landed, then confirmed against a TX16S off air. The procedure and
//! the evidence are in `docs/protocols.md`, and the measurement itself is in
//! `testdata/sx1280_cr_li_*_map.json`.
//!
//! # What it turned out to be
//!
//! The code is the same Hamming(8,4) the SX127x uses at 4/8. What differs is
//! the interleaver, and the difference is the whole point of the name: where
//! the SX127x interleaves inside each block of `4 + cr` symbols, this spreads
//! one nibble across the entire packet. For a payload of `K` nibbles the coded
//! stream is `8K` bits, `4K` systematic then `4K` parity, with data bit `j` of
//! nibble `n` at `n + Kj` and its parity at `4K + n + Kj`. A fade that ruins a
//! symbol then costs one bit from each of several nibbles, every one of them
//! correctable, instead of several bits from one nibble, which is not.
//!
//! The symbols are not all the same width: the first eight carry `SF - 2` bits
//! and the rest carry `SF`, in implicit header mode where there is no header
//! to justify a reduced-rate opening. That split is measured per configuration
//! rather than assumed, because at SF5 it is two symbols and not eight, and
//! only SF7 at 4/8 has been confirmed against a real link.
//!
//! # Only one configuration is supported, on purpose
//!
//! SF7 at `CR_LI 4/8` is what has been measured for both ExpressLRS payload
//! lengths and checked against a transmitter that is not ours. The other
//! spreading factors and the 4/5 and 4/6 long rates each need their own walk,
//! and [`decode`] returns `None` for them rather than guessing, because a
//! plausible-looking wrong decode is worse here than no decode: the packet
//! carries no CRC of its own that would catch it.

/// XORed into the payload before coding. Measured as the decode of an all-zero
/// payload, and the eight byte sequence is the first eight of these, which is
/// the evidence that it is one LFSR run rather than a function of the length.
pub const WHITENING: [u8; 13] = [
    0xff, 0xfe, 0xfc, 0xf8, 0xf0, 0xe1, 0xc2, 0x85, 0x0b, 0x17, 0x2f, 0x5e, 0xbc,
];

/// Symbols at the reduced width before the rest run at the full spreading
/// factor. Measured at SF7; see the module note about SF5.
const REDUCED_SYMBOLS: usize = 8;

/// A decoded packet, and how much repair it took to get there.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Decoded {
    pub bytes: Vec<u8>,
    /// Nibbles whose parity disagreed and were corrected. A packet needing
    /// several is a weak one, which is worth knowing when the payload has no
    /// checksum of its own.
    pub corrected: usize,
}

/// Bits each symbol of a packet carries.
pub fn symbol_widths(sf: u8, symbols: usize) -> Vec<u8> {
    (0..symbols)
        .map(|i| if i < REDUCED_SYMBOLS { sf - 2 } else { sf })
        .collect()
}

/// Symbols a packet of this length occupies.
pub fn symbol_count(sf: u8, len: usize) -> usize {
    let coded = 16 * len; // 8 bits out per nibble, two nibbles a byte
    let reduced = REDUCED_SYMBOLS * (sf as usize - 2);
    if coded <= reduced {
        return coded.div_ceil(sf as usize - 2);
    }
    REDUCED_SYMBOLS + (coded - reduced).div_ceil(sf as usize)
}

fn parity(d: [u8; 4]) -> [u8; 4] {
    [
        d[0] ^ d[1] ^ d[2],
        d[1] ^ d[2] ^ d[3],
        d[0] ^ d[1] ^ d[3],
        d[0] ^ d[2] ^ d[3],
    ]
}

/// Read the payload out of a packet's symbols.
///
/// `cr` is the denominator, so 8 for `CR_LI 4/8`, and `len` is the payload
/// length both ends agreed on, since implicit header mode does not transmit
/// it. Returns `None` for a configuration that has not been measured or a
/// symbol run too short to hold the payload.
pub fn decode(symbols: &[u16], sf: u8, cr: u8, len: usize) -> Option<Decoded> {
    if cr != 8 || sf != 7 || len == 0 {
        return None;
    }
    let widths = symbol_widths(sf, symbols.len());
    let mut bits: Vec<u8> = Vec::with_capacity(symbols.len() * sf as usize);
    for (s, w) in symbols.iter().zip(&widths) {
        // The demodulator reports the bin; the codeword is its Gray code one
        // step down, shifted to drop the bits a reduced-rate symbol does not
        // carry.
        let v = (s.wrapping_sub(1)) & ((1 << sf) - 1);
        let c = (v >> (sf - w)) ^ ((v >> (sf - w)) >> 1);
        for k in 0..*w {
            bits.push(((c >> k) & 1) as u8);
        }
    }

    let nibbles = 2 * len;
    if bits.len() < 8 * nibbles {
        return None;
    }

    let mut out = vec![0u8; len];
    let mut corrected = 0;
    for n in 0..nibbles {
        let mut d = [0u8; 4];
        let mut p = [0u8; 4];
        for j in 0..4 {
            d[j] = bits[n + nibbles * j];
            p[j] = bits[4 * nibbles + n + nibbles * j];
        }
        let want = parity(d);
        if want != p {
            // Hamming(8,4): one flipped bit anywhere in the codeword has its
            // own signature, so try all eight and take the one that agrees.
            let syndrome: Vec<usize> = (0..4).filter(|&j| want[j] != p[j]).collect();
            let mut fixed = false;
            for bit in 0..4 {
                let mut t = d;
                t[bit] ^= 1;
                if parity(t) == p {
                    d = t;
                    fixed = true;
                    break;
                }
            }
            // A single parity bit in error leaves the data alone.
            if !fixed && syndrome.len() == 1 {
                fixed = true;
            }
            if !fixed {
                return None; // two errors in one nibble: not repairable
            }
            corrected += 1;
        }
        let nib = d[0] | d[1] << 1 | d[2] << 2 | d[3] << 3;
        if n % 2 == 0 {
            out[n / 2] |= nib;
        } else {
            out[n / 2] |= nib << 4;
        }
    }

    for (i, b) in out.iter_mut().enumerate() {
        *b ^= WHITENING[i % WHITENING.len()];
    }
    Some(Decoded { bytes: out, corrected })
}

/// The symbols a transmitter would send for this payload. Here so the decoder
/// can be tested against something other than itself, and so a bench
/// transmitter can be checked without a radio.
pub fn encode(payload: &[u8], sf: u8, cr: u8) -> Option<Vec<u16>> {
    if cr != 8 || sf != 7 || payload.is_empty() {
        return None;
    }
    let len = payload.len();
    let nibbles = 2 * len;
    let mut bits = vec![0u8; 8 * nibbles];
    for n in 0..nibbles {
        let b = payload[n / 2] ^ WHITENING[(n / 2) % WHITENING.len()];
        let nib = if n % 2 == 0 { b & 0x0f } else { b >> 4 };
        let d = [nib & 1, (nib >> 1) & 1, (nib >> 2) & 1, (nib >> 3) & 1];
        let p = parity(d);
        for j in 0..4 {
            bits[n + nibbles * j] = d[j];
            bits[4 * nibbles + n + nibbles * j] = p[j];
        }
    }
    let symbols = symbol_count(sf, len);
    let widths = symbol_widths(sf, symbols);
    let mut out = Vec::with_capacity(symbols);
    let mut at = 0usize;
    for w in widths {
        let mut c = 0u16;
        for k in 0..w {
            let bit = if at < bits.len() { bits[at] } else { 0 };
            c |= u16::from(bit) << k;
            at += 1;
        }
        // Undo the Gray map, put the reduced-rate bits back at the top, and
        // add the one the demodulator takes off.
        let mut v = c;
        let mut shift = 1;
        while shift < u32::from(w) {
            v ^= c >> shift;
            shift += 1;
        }
        let v = (v & ((1 << w) - 1)) << (sf - w);
        out.push((v + 1) & ((1 << sf) - 1));
    }
    Some(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Symbols lifted from a TX16S at 100 Hz Full, off air at 2.4 GHz, decoded
    /// by the measured map. The evidence that they are real packets and not an
    /// artefact of this code is the CRC below: all three imply the same seed,
    /// which is the binding UID, and a wrong decode would not.
    const OFF_AIR: [([u16; 32], [u8; 13]); 2] = [
        (
            [
                117, 81, 73, 5, 117, 17, 29, 5, 121, 76, 111, 77, 65, 21, 119, 42, 127, 75, 67,
                15, 81, 83, 46, 80, 37, 49, 1, 93, 22, 35, 106, 52,
            ],
            [
                0x00, 0xfa, 0xb5, 0x67, 0xc5, 0x7c, 0x56, 0x58, 0x01, 0x1f, 0x7c, 0x78, 0x73,
            ],
        ),
        (
            [
                117, 81, 73, 5, 73, 17, 29, 5, 121, 72, 106, 77, 64, 20, 120, 42, 34, 118, 67,
                15, 0, 83, 46, 80, 42, 52, 1, 93, 22, 35, 106, 116,
            ],
            [
                0x00, 0xfa, 0xb9, 0x67, 0xc5, 0x7c, 0x56, 0x58, 0x01, 0x1f, 0x7c, 0xe6, 0x39,
            ],
        ),
    ];

    /// The full packet's CRC, repeated here rather than reached for: this test
    /// is about whether two packets agree on a seed, not about `elrs`.
    fn crc16(data: &[u8], init: u16) -> u16 {
        let mut crc = init;
        for &b in data {
            crc ^= u16::from(b) << 8;
            for _ in 0..8 {
                crc = if crc & 0x8000 != 0 { (crc << 1) ^ 0x3d65 } else { crc << 1 };
            }
        }
        crc
    }

    fn seed_of(packet: &[u8]) -> Option<u16> {
        let sent = u16::from_le_bytes([packet[11], packet[12]]);
        (0..=u16::MAX).find(|&s| crc16(&packet[..11], s) == sent)
    }

    #[test]
    fn a_payload_survives_the_round_trip() {
        for len in [8usize, 13] {
            let payload: Vec<u8> = (0..len).map(|i| (i as u8).wrapping_mul(37).wrapping_add(9)).collect();
            let symbols = encode(&payload, 7, 8).expect("encode");
            assert_eq!(symbols.len(), symbol_count(7, len));
            let got = decode(&symbols, 7, 8, len).expect("decode");
            assert_eq!(got.bytes, payload);
            assert_eq!(got.corrected, 0);
        }
    }

    #[test]
    fn one_flipped_symbol_bit_is_repaired() {
        let payload: Vec<u8> = (0..13).map(|i| i as u8 * 11).collect();
        let mut symbols = encode(&payload, 7, 8).expect("encode");
        symbols[20] ^= 0b0010_0000;
        let got = decode(&symbols, 7, 8, 13).expect("decode");
        assert_eq!(got.bytes, payload);
        assert!(got.corrected >= 1, "a flipped bit should have been corrected");
    }

    /// The packets came off a transmitter nobody here configured, and two of
    /// them solve to one seed. A wrong interleaver gives arbitrary bytes and
    /// arbitrary seeds, so agreement is the check.
    #[test]
    fn packets_off_air_decode_and_agree_on_the_binding_uid() {
        let mut seeds = Vec::new();
        for (symbols, want) in OFF_AIR {
            let got = decode(&symbols, 7, 8, 13).expect("decode an off air packet");
            assert_eq!(got.bytes, want, "payload differs from the measured map's answer");
            seeds.push(seed_of(&got.bytes).expect("a seed exists"));
        }
        assert_eq!(seeds[0], seeds[1], "two packets of one link disagree on the seed");
        assert_eq!(seeds[0], 0x6b37, "the seed this link was recorded with");
    }

    #[test]
    fn an_unmeasured_configuration_is_refused_rather_than_guessed() {
        let symbols = [1u16; 40];
        assert!(decode(&symbols, 7, 5, 13).is_none());
        assert!(decode(&symbols, 8, 8, 13).is_none());
    }
}
