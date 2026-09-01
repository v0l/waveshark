//! Bit buffers and integrity checks.

/// A packed bit string, MSB first within each byte.
///
/// MSB-first matches how every protocol document writes its frames, so a
/// layout like `ff FI IT TT` can be read straight out of `as_bytes()` without
/// mental reversal. Getting this backwards is a classic source of decoders
/// that almost work.
/// Row starts are carried alongside the bits because they are the only
/// evidence of where a frame begins. A burst holds a transmission repeated ten
/// or twelve times, separated by a gap far longer than any symbol, and that gap
/// is where each copy starts. Without it a decoder has to search every bit
/// offset and trust a checksum to reject the wrong ones, which for the many
/// protocols carrying six or eight bits of checksum it will not reliably do:
/// a misaligned window that happens to sum correctly reports a real-looking
/// device with an invented temperature. rtl_433 avoids that by cutting the
/// burst into rows at its `gap_limit`; this is the same information.
#[derive(Clone, Default)]
pub struct BitBuffer {
    bytes: Vec<u8>,
    len: usize,
    rows: Vec<usize>,
}

/// Rows are metadata about how the bits were found, not part of the value, so
/// two buffers holding the same bits are equal whatever their row structure.
/// Frame comparison depends on this.
impl PartialEq for BitBuffer {
    fn eq(&self, other: &Self) -> bool {
        self.len == other.len && self.as_padded_bytes() == other.as_padded_bytes()
    }
}

impl Eq for BitBuffer {}

impl BitBuffer {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn with_capacity(bits: usize) -> Self {
        Self { bytes: Vec::with_capacity(bits.div_ceil(8)), len: 0, rows: Vec::new() }
    }

    pub fn from_bytes(b: &[u8]) -> Self {
        Self { bytes: b.to_vec(), len: b.len() * 8, rows: Vec::new() }
    }

    /// Number of bits held.
    pub fn len(&self) -> usize {
        self.len
    }

    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// Whole bytes only; a trailing partial byte is excluded.
    pub fn as_bytes(&self) -> &[u8] {
        &self.bytes[..self.len / 8]
    }

    /// All bytes including a zero-padded trailing partial byte.
    pub fn as_padded_bytes(&self) -> &[u8] {
        &self.bytes
    }

    pub fn clear(&mut self) {
        self.bytes.clear();
        self.len = 0;
        self.rows.clear();
    }

    /// Record that a new row starts at the next bit pushed.
    pub fn mark_row(&mut self) {
        if self.rows.last() != Some(&self.len) {
            self.rows.push(self.len);
        }
    }

    /// Bit offsets where a row starts, empty when the slicer found no row
    /// structure. The first row is included only if it was marked.
    pub fn rows(&self) -> &[usize] {
        &self.rows
    }

    pub fn push(&mut self, bit: bool) {
        if self.len % 8 == 0 {
            self.bytes.push(0);
        }
        if bit {
            let i = self.len / 8;
            self.bytes[i] |= 0x80 >> (self.len % 8);
        }
        self.len += 1;
    }

    /// Append `n` copies of `bit`.
    pub fn extend(&mut self, bit: bool, n: usize) {
        for _ in 0..n {
            self.push(bit);
        }
    }

    pub fn get(&self, i: usize) -> Option<bool> {
        if i >= self.len {
            return None;
        }
        Some(self.bytes[i / 8] & (0x80 >> (i % 8)) != 0)
    }

    /// Extract `n` bits starting at `start`, right-aligned into a u32.
    pub fn extract(&self, start: usize, n: usize) -> Option<u32> {
        if n > 32 || start + n > self.len {
            return None;
        }
        let mut v = 0u32;
        for i in 0..n {
            v = (v << 1) | self.get(start + i)? as u32;
        }
        Some(v)
    }

    /// Find `pattern`'s first `pattern_bits` bits, returning the bit offset.
    ///
    /// Searching at bit rather than byte granularity is essential: a slicer
    /// starts wherever the first pulse happened to be detected, so a frame is
    /// almost never byte-aligned to the buffer.
    pub fn find(&self, pattern: &[u8], pattern_bits: usize) -> Option<usize> {
        if pattern_bits == 0 || pattern_bits > self.len {
            return None;
        }
        let pat = BitBuffer { bytes: pattern.to_vec(), len: pattern_bits, rows: Vec::new() };
        'outer: for start in 0..=(self.len - pattern_bits) {
            for i in 0..pattern_bits {
                if self.get(start + i) != pat.get(i) {
                    continue 'outer;
                }
            }
            return Some(start);
        }
        None
    }

    /// Copy `n` bits from `start` into a new buffer, realigning to byte zero.
    pub fn slice(&self, start: usize, n: usize) -> BitBuffer {
        let mut out = BitBuffer::with_capacity(n);
        for i in 0..n {
            match self.get(start + i) {
                Some(b) => out.push(b),
                None => break,
            }
        }
        out
    }

    /// Every bit flipped.
    ///
    /// Several protocols are documented with the opposite polarity to the one
    /// the slicer produces, and rtl_433 handles them by inverting the whole
    /// buffer before parsing. Doing the same keeps a transcribed frame layout
    /// readable against its source.
    pub fn inverted(&self) -> BitBuffer {
        BitBuffer {
            bytes: self.bytes.iter().map(|b| !b).collect(),
            len: self.len,
            rows: self.rows.clone(),
        }
    }

    pub fn to_hex(&self) -> String {
        self.bytes
            .iter()
            .map(|b| format!("{b:02x}"))
            .collect::<Vec<_>>()
            .join(" ")
    }
}

impl std::fmt::Debug for BitBuffer {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "BitBuffer({} bits: {})", self.len, self.to_hex())
    }
}

/// MSB-first CRC-8.
///
/// `poly` is the normal (non-reflected) representation, for example 0x31 for
/// CRC-8/NRSC-5 as used by Fine Offset. A frame that includes its own CRC
/// yields zero when the whole frame is passed in, which is the usual way to
/// check one.
pub fn crc8(data: &[u8], poly: u8, init: u8) -> u8 {
    let mut crc = init;
    for &b in data {
        crc ^= b;
        for _ in 0..8 {
            crc = if crc & 0x80 != 0 { (crc << 1) ^ poly } else { crc << 1 };
        }
    }
    crc
}

/// MSB-first CRC-16, `poly` in its normal representation.
pub fn crc16(data: &[u8], poly: u16, init: u16) -> u16 {
    let mut crc = init;
    for &b in data {
        crc ^= (b as u16) << 8;
        for _ in 0..8 {
            crc = if crc & 0x8000 != 0 { (crc << 1) ^ poly } else { crc << 1 };
        }
    }
    crc
}

/// Simple additive checksum, truncated to 8 bits.
pub fn checksum8(data: &[u8]) -> u8 {
    data.iter().fold(0u8, |a, b| a.wrapping_add(*b))
}

/// XOR of every byte, used by several cheap sensors.
pub fn xor8(data: &[u8]) -> u8 {
    data.iter().fold(0u8, |a, b| a ^ b)
}

/// Reverse the bit order within a byte, for protocols transmitted LSB first.
pub fn reflect8(b: u8) -> u8 {
    b.reverse_bits()
}

/// Even parity of one byte: 1 when an odd number of bits are set.
pub fn parity8(b: u8) -> u8 {
    b.count_ones() as u8 & 1
}

/// True when every byte carries even parity, as Acurite's TXR family requires.
pub fn even_parity(data: &[u8]) -> bool {
    data.iter().all(|b| parity8(*b) == 0)
}

/// Galois LFSR digest, reflected, as rtl_433's `lfsr_digest8_reflect`.
///
/// Used by LaCrosse and several others in place of a CRC. Bytes are processed
/// last to first and bits LSB first, the key rolling left through `gen` at
/// every bit. It is not a CRC and cannot be computed with one.
pub fn lfsr_digest8_reflect(data: &[u8], gen: u8, key: u8) -> u8 {
    let mut sum = 0u8;
    let mut key = key;
    for &byte in data.iter().rev() {
        for i in 0..8 {
            if byte >> i & 1 != 0 {
                sum ^= key;
            }
            key = if key & 0x80 != 0 { (key << 1) ^ gen } else { key << 1 };
        }
    }
    sum
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pushes_msb_first() {
        let mut b = BitBuffer::new();
        for bit in [true, true, true, true, false, false, false, false] {
            b.push(bit);
        }
        assert_eq!(b.as_bytes(), &[0xf0]);
        assert_eq!(b.len(), 8);
    }

    #[test]
    fn extract_reads_across_byte_boundaries() {
        let b = BitBuffer::from_bytes(&[0b1010_1010, 0b1100_0011]);
        assert_eq!(b.extract(0, 8), Some(0b1010_1010));
        assert_eq!(b.extract(4, 8), Some(0b1010_1100));
        assert_eq!(b.extract(12, 4), Some(0b0011));
        assert_eq!(b.extract(12, 8), None, "reading past the end must fail");
    }

    #[test]
    fn find_locates_an_unaligned_pattern() {
        // 0xff preamble starting at bit 3.
        let mut b = BitBuffer::new();
        for bit in [false, true, false] {
            b.push(bit);
        }
        for _ in 0..8 {
            b.push(true);
        }
        b.push(false);
        assert_eq!(b.find(&[0xff], 8), Some(3));
    }

    #[test]
    fn slice_realigns_to_byte_zero() {
        let b = BitBuffer::from_bytes(&[0b0001_1111, 0b1111_0000]);
        let s = b.slice(3, 8);
        assert_eq!(s.as_bytes(), &[0b1111_1111]);
    }

    #[test]
    fn crc8_matches_a_known_vector() {
        // CRC-8/NRSC-5: poly 0x31, init 0xff, "123456789" -> 0xf7.
        assert_eq!(crc8(b"123456789", 0x31, 0xff), 0xf7);
    }

    #[test]
    fn crc8_over_a_frame_including_its_crc_is_zero() {
        let payload = [0xff, 0xa1, 0x23];
        let c = crc8(&payload, 0x31, 0xff);
        let mut framed = payload.to_vec();
        framed.push(c);
        assert_eq!(crc8(&framed, 0x31, 0xff), 0);
    }

    #[test]
    fn inverted_flips_every_bit_and_keeps_the_length() {
        let b = BitBuffer::from_bytes(&[0b1010_0000, 0xff]).slice(0, 12);
        let i = b.inverted();
        assert_eq!(i.len(), 12);
        for n in 0..12 {
            assert_eq!(i.get(n), b.get(n).map(|v| !v));
        }
    }

    #[test]
    fn parity_counts_set_bits() {
        assert_eq!(parity8(0b0000_0000), 0);
        assert_eq!(parity8(0b1000_0001), 0);
        assert_eq!(parity8(0b1000_0000), 1);
        assert!(even_parity(&[0x00, 0x03, 0xff]));
        assert!(!even_parity(&[0x00, 0x01]));
    }

    #[test]
    fn lfsr_digest_matches_rtl_433() {
        // Checked against rtl_433's own lfsr_digest8_reflect compiled and run
        // on the same input, with the LaCrosse TX141TH parameters.
        assert_eq!(lfsr_digest8_reflect(&[0xd4, 0x22, 0xf5, 0x3b], 0x31, 0xf4), 0x5b);
    }

    #[test]
    fn reflect8_reverses_bit_order() {
        assert_eq!(reflect8(0b1000_0001), 0b1000_0001);
        assert_eq!(reflect8(0b1100_0000), 0b0000_0011);
    }
}
