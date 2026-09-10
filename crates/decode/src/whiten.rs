//! Data whitening, and reading a frame that has been whitened.
//!
//! A payload of high-entropy bytes with no structure at all is the normal
//! result of looking at an unknown 868 or 915 MHz device, and it is usually
//! not encryption. Sub-GHz transceivers whiten the payload by default so that
//! a long run of identical bytes cannot starve the receiver's clock recovery,
//! and the whitener is not a secret: it is a nine-bit LFSR with a fixed seed,
//! specified in the data sheet. TI's parts (CC1101, CC110L, CC1120, and the
//! CC13xx radios) all use the same PN9, and Silicon Labs' EZRadio parts use a
//! compatible one.
//!
//! So the first thing worth trying on a scrambled payload is XOR with PN9. If
//! a length byte and a valid CRC fall out, the frame format is answered: not
//! the vendor, but the framing, the payload length and where the address sits,
//! which is where reverse engineering can actually start. If nothing falls
//! out, that is evidence too, and it points at genuine encryption or at a
//! vendor-specific scrambler.
//!
//! Whitening is its own inverse, so the same function whitens and unwhitens.

use crate::bits::crc16;

/// TI's PN9 whitening sequence: a nine-bit LFSR, `x^9 + x^5 + 1`, seeded all
/// ones, yielding one byte per eight shifts.
pub struct Pn9(u16);

impl Default for Pn9 {
    fn default() -> Self {
        Self::new()
    }
}

impl Pn9 {
    pub fn new() -> Self {
        Self(0x1ff)
    }

    /// Seed the register explicitly. Every part in common use seeds all ones,
    /// but a few vendors pick their own and the sequence is otherwise the
    /// same.
    pub fn with_seed(seed: u16) -> Self {
        Self(seed & 0x1ff)
    }
}

impl Iterator for Pn9 {
    type Item = u8;

    fn next(&mut self) -> Option<u8> {
        let out = (self.0 & 0xff) as u8;
        for _ in 0..8 {
            let bit = (self.0 ^ (self.0 >> 5)) & 1;
            self.0 = (self.0 >> 1) | (bit << 8);
        }
        Some(out)
    }
}

/// XOR `data` with PN9. Applying this to whitened data recovers the original.
pub fn pn9(data: &[u8]) -> Vec<u8> {
    data.iter().zip(Pn9::new()).map(|(b, k)| b ^ k).collect()
}

/// CRC-16 as the TI parts compute it: `x^16 + x^15 + x^2 + 1`, seeded all
/// ones, MSB first, appended big-endian.
pub fn crc16_ti(data: &[u8]) -> u16 {
    crc16(data, 0x8005, 0xffff)
}

/// A frame that reads as variable-length TI framing: one length byte, that
/// many payload bytes, then a two-byte CRC over both.
#[derive(Clone, Debug, PartialEq)]
pub struct Framed {
    /// Whether the bytes had to be de-whitened to read.
    pub whitened: bool,
    /// Bytes skipped before the length byte, which is the sync word: two to
    /// four bytes on the parts that use this framing.
    pub sync_len: usize,
    /// Payload after the length byte, CRC excluded.
    pub payload: Vec<u8>,
    pub crc: u16,
}

/// Smallest payload worth believing. A one or two byte frame passing a
/// sixteen-bit CRC is possible but the odds of a false hit rise as the frame
/// shrinks, and nothing useful is that short.
const MIN_PAYLOAD: usize = 4;

/// Sync word lengths tried before the length byte. The parts using this
/// framing send two, three or four bytes of sync; zero covers a stream already
/// cut at the length byte.
///
/// The sweep is kept this short on purpose. Each candidate is a fresh chance
/// for a sixteen-bit CRC to pass by luck, at one in 65536, and sweeping every
/// bit offset instead of these few byte ones would put a hundred and
/// twenty-eight candidates behind every burst and report a frame on noise
/// every few hundred of them. Whitening is only tried where the frame is
/// whitened from the length byte on, which is what the parts do.
const SYNC_LENS: [usize; 4] = [0, 2, 3, 4];

/// Try to read `bytes` as a TI-style variable-length frame, de-whitening if
/// that is what it takes.
///
/// The plain reading is tried first at each sync length: a device that
/// transmits with whitening off is common enough, and PN9 of a valid plain
/// frame will not pass a CRC, so trying both costs only the extra candidate.
pub fn read_framed(bytes: &[u8]) -> Option<Framed> {
    SYNC_LENS.into_iter().find_map(|sync| {
        let rest = bytes.get(sync..)?;
        check(rest, false, sync).or_else(|| check(&pn9(rest), true, sync))
    })
}

fn check(bytes: &[u8], whitened: bool, sync_len: usize) -> Option<Framed> {
    let &len = bytes.first()?;
    let len = len as usize;
    if len < MIN_PAYLOAD {
        return None;
    }
    // Length, payload, then the CRC over the two.
    let end = 1 + len;
    let crc_at = bytes.get(end..end + 2)?;
    let crc = u16::from_be_bytes([crc_at[0], crc_at[1]]);
    if crc16_ti(&bytes[..end]) != crc {
        return None;
    }
    Some(Framed { whitened, sync_len, payload: bytes[1..end].to_vec(), crc })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pn9_matches_the_published_sequence() {
        // The first bytes of TI's PN9, as printed in the CC1101 errata and
        // reproduced by every implementation of it. Getting the shift
        // direction wrong still produces a plausible-looking pseudo-random
        // sequence, so this is checked against the published one rather than
        // against itself.
        let got: Vec<u8> = Pn9::new().take(8).collect();
        assert_eq!(got, vec![0xff, 0xe1, 0x1d, 0x9a, 0xed, 0x85, 0x33, 0x24]);
    }

    #[test]
    fn pn9_repeats_after_511_bits_and_not_before() {
        // The register is nine bits and the polynomial is primitive, so the
        // sequence has period 511 bits. A period of 255 or 512 means the
        // feedback tap is in the wrong place.
        let long: Vec<u8> = Pn9::new().take(600).collect();
        assert_ne!(long[0..64], long[64..128], "the sequence repeated far too soon");
        // Eight shifts per byte, so the register comes back to its seed after
        // 511 bytes and the byte stream repeats there.
        assert_eq!(long[0..64], long[511..575]);
    }

    #[test]
    fn whitening_is_its_own_inverse() {
        let data = b"a payload with a long run\x00\x00\x00\x00\x00\x00 in it";
        assert_eq!(pn9(&pn9(data)), data.to_vec());
    }

    #[test]
    fn crc16_ti_matches_a_known_vector() {
        // CRC-16/CMS: poly 0x8005, init 0xffff, no reflection.
        assert_eq!(crc16_ti(b"123456789"), 0xaee7);
    }

    #[test]
    fn a_whitened_frame_reads_back() {
        let payload = [0x1au8, 0x2b, 0x3c, 0x4d, 0x5e, 0x6f];
        let mut frame = vec![payload.len() as u8];
        frame.extend_from_slice(&payload);
        let crc = crc16_ti(&frame);
        frame.extend_from_slice(&crc.to_be_bytes());
        let on_air = pn9(&frame);
        assert_ne!(on_air, frame, "the test frame was not actually whitened");

        let got = read_framed(&on_air).expect("frame");
        assert!(got.whitened);
        assert_eq!(got.sync_len, 0);
        assert_eq!(got.payload, payload);
    }

    #[test]
    fn a_frame_behind_a_sync_word_reads() {
        // What actually comes off the air: the sync word is in front of the
        // length byte, and only what follows it is whitened.
        let payload = [0xdeu8, 0xad, 0xbe, 0xef, 0x01, 0x02, 0x03];
        let mut frame = vec![payload.len() as u8];
        frame.extend_from_slice(&payload);
        frame.extend_from_slice(&crc16_ti(&frame).to_be_bytes());
        let mut on_air = vec![0x2du8, 0xd4];
        on_air.extend_from_slice(&pn9(&frame));

        let got = read_framed(&on_air).expect("frame");
        assert_eq!(got.sync_len, 2);
        assert!(got.whitened);
        assert_eq!(got.payload, payload);
    }

    #[test]
    fn an_unwhitened_frame_reads_back_without_being_called_whitened() {
        let payload = [0x11u8, 0x22, 0x33, 0x44, 0x55];
        let mut frame = vec![payload.len() as u8];
        frame.extend_from_slice(&payload);
        frame.extend_from_slice(&crc16_ti(&frame).to_be_bytes());
        let got = read_framed(&frame).expect("frame");
        assert!(!got.whitened);
        assert_eq!(got.payload, payload);
    }

    #[test]
    fn trailing_bytes_past_the_crc_do_not_stop_a_frame_reading() {
        // The slicer runs past the end of the transmission and pads with the
        // silence that followed, so a real frame nearly always has junk after
        // its CRC.
        let payload = [0x11u8, 0x22, 0x33, 0x44, 0x55];
        let mut frame = vec![payload.len() as u8];
        frame.extend_from_slice(&payload);
        frame.extend_from_slice(&crc16_ti(&frame).to_be_bytes());
        frame.extend_from_slice(&[0, 0, 0, 0, 0, 0]);
        assert_eq!(read_framed(&frame).map(|f| f.payload), Some(payload.to_vec()));
    }

    #[test]
    fn noise_is_not_reported_as_a_frame() {
        // Every 24-byte pseudo-random buffer that passes is a false positive
        // an operator would waste time on. Eight candidate readings at one in
        // 65536 each is about one buffer in eight thousand, so a thousand must
        // essentially all fail.
        let mut seed = 0x2545f491_4f6cdd1du64;
        let mut hits = 0;
        for _ in 0..1000 {
            let bytes: Vec<u8> = (0..24)
                .map(|_| {
                    seed ^= seed << 13;
                    seed ^= seed >> 7;
                    seed ^= seed << 17;
                    (seed >> 24) as u8
                })
                .collect();
            if read_framed(&bytes).is_some() {
                hits += 1;
            }
        }
        assert!(hits <= 1, "{hits} of 1000 noise buffers read as frames");
    }

    #[test]
    fn a_length_byte_running_off_the_end_is_not_a_frame() {
        let mut bytes = vec![0xf0u8];
        bytes.extend_from_slice(&[0x11; 12]);
        assert_eq!(read_framed(&bytes), None);
    }
}
