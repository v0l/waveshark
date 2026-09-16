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
        if self.len.is_multiple_of(8) {
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
        self.bytes.iter().map(|b| format!("{b:02x}")).collect::<Vec<_>>().join(" ")
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

/// MSB-first CRC-32, `poly` in its normal representation. 0x04C11DB7 with an
/// all-ones start and no final inversion is the check every MPEG and DVB
/// table carries, and a section including its own check yields zero.
pub fn crc32(data: &[u8], poly: u32, init: u32) -> u32 {
    let mut crc = init;
    for &b in data {
        crc ^= (b as u32) << 24;
        for _ in 0..8 {
            crc = if crc & 0x8000_0000 != 0 { (crc << 1) ^ poly } else { crc << 1 };
        }
    }
    crc
}

/// LSB-first CRC-16, `poly` in its reflected representation: 0x8408 is the
/// CCITT polynomial as X.25, ARINC 618 and a dozen packet radios use it.
pub fn crc16le(data: &[u8], poly: u16, init: u16) -> u16 {
    let mut crc = init;
    for &b in data {
        crc ^= b as u16;
        for _ in 0..8 {
            crc = if crc & 1 != 0 { (crc >> 1) ^ poly } else { crc >> 1 };
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
pub fn lfsr_digest8_reflect(data: &[u8], r#gen: u8, key: u8) -> u8 {
    let mut sum = 0u8;
    let mut key = key;
    for &byte in data.iter().rev() {
        for i in 0..8 {
            if byte >> i & 1 != 0 {
                sum ^= key;
            }
            key = if key & 0x80 != 0 { (key << 1) ^ r#gen } else { key << 1 };
        }
    }
    sum
}

/// Galois LFSR digest, as rtl_433's `lfsr_digest8`.
///
/// The same construction as [`lfsr_digest8_reflect`] with every direction
/// turned around: bytes first to last, bits MSB first, and the key rolling
/// right. Acurite's 606TX uses it where its siblings use a sum.
pub fn lfsr_digest8(data: &[u8], r#gen: u8, key: u8) -> u8 {
    let mut sum = 0u8;
    let mut key = key;
    for &byte in data {
        for i in (0..8).rev() {
            if byte >> i & 1 != 0 {
                sum ^= key;
            }
            key = if key & 1 != 0 { (key >> 1) ^ r#gen } else { key >> 1 };
        }
    }
    sum
}

/// GF(64), built on x^6 + x + 1, as the tables a BCH(63,51) decoder needs.
///
/// Antilog and log: `exp[i]` is the field element a^i and `log[e]` the power
/// that produced it. Built once, because a Meisei radiosonde asks for twelve
/// codewords a second and a fresh table each time is the whole cost.
fn gf64() -> &'static ([u8; 64], [u8; 64]) {
    static TABLES: std::sync::OnceLock<([u8; 64], [u8; 64])> = std::sync::OnceLock::new();
    TABLES.get_or_init(|| {
        let (mut exp, mut log) = ([0u8; 64], [0u8; 64]);
        let mut x = 1u8;
        for (i, e) in exp.iter_mut().enumerate().take(63) {
            *e = x;
            log[x as usize] = i as u8;
            x <<= 1;
            if x & 0x40 != 0 {
                x ^= 0x43;
            }
        }
        exp[63] = exp[0];
        (exp, log)
    })
}

/// BCH(63,51) over GF(64), correcting up to two wrong bits.
///
/// `code` is the codeword as bits, `code[i]` the coefficient of x^i, so the
/// parity is at the low end. A shortened code is the same thing with zeros
/// in the positions that were never sent: the Meisei sondes send (46,34),
/// which is this with seventeen zeros above it.
///
/// Returns how many bits were corrected, or `None` where the syndromes name
/// no pair of positions, which is three wrong bits or more. Two syndromes
/// are enough: for a binary code the even ones follow from the odd, so only
/// `S1` and `S3` have to be computed.
pub fn bch63_51(code: &mut [bool]) -> Option<u32> {
    if code.len() != 63 {
        return None;
    }
    let (exp, log) = gf64();
    let mul = |a: u8, b: u8| match a == 0 || b == 0 {
        true => 0,
        false => exp[(usize::from(log[a as usize]) + usize::from(log[b as usize])) % 63],
    };
    let syndrome = |power: usize| {
        code.iter().enumerate().filter(|(_, b)| **b).fold(0u8, |s, (i, _)| s ^ exp[i * power % 63])
    };
    let (s1, s3) = (syndrome(1), syndrome(3));
    if s1 == 0 {
        // No first syndrome and a third one is a pattern no pair of errors
        // can make.
        return (s3 == 0).then_some(0);
    }
    let s1_cubed = mul(mul(s1, s1), s1);
    if s1_cubed == s3 {
        let at = usize::from(log[s1 as usize]);
        code[at] = !code[at];
        return Some(1);
    }
    // The error locator is 1 + s1 x + ((s1^3 + s3)/s1) x^2, and its roots are
    // the inverses of the two positions. Chien search: try every power.
    let num = s1_cubed ^ s3;
    let sigma2 = exp[(usize::from(log[num as usize]) + 63 - usize::from(log[s1 as usize])) % 63];
    let mut found = Vec::with_capacity(2);
    for at in 0..63usize {
        let x = exp[(63 - at) % 63];
        if 1 ^ mul(s1, x) ^ mul(sigma2, mul(x, x)) == 0 {
            found.push(at);
        }
    }
    if found.len() != 2 {
        return None;
    }
    for at in found {
        code[at] = !code[at];
    }
    Some(2)
}

/// One nibble read back out of a Hamming(8,4) codeword, and whether a bit
/// had to be corrected to get it.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct Hamming84 {
    pub nibble: u8,
    pub corrected: bool,
}

/// Parity check matrix, one row a byte, most significant bit first, which is
/// the order the bits arrive in.
const H84: [u8; 4] = [0b0111_1000, 0b1011_0100, 0b1101_0010, 0b1110_0001];

/// The syndrome a single wrong bit leaves, by position. Reading the columns
/// of [`H84`], so position 0 is `0x7` and the four parity bits are the
/// powers of two.
const H84_SYNDROME: [u8; 8] = [0x7, 0xB, 0xD, 0xE, 0x8, 0x4, 0x2, 0x1];

/// Systematic Hamming(8,4): four data bits, four parity bits, one error
/// corrected and two detected.
///
/// `code` is the eight bits as they arrived, most significant first, with
/// the data in the top nibble. `None` where the syndrome names no single
/// position, which is two bits wrong or worse. The DFM radiosondes protect
/// every nibble of a frame this way.
pub fn hamming84(code: u8) -> Option<Hamming84> {
    let syndrome = H84.iter().fold(0u8, |s, row| s << 1 | (row & code).count_ones() as u8 & 1);
    if syndrome == 0 {
        return Some(Hamming84 { nibble: code >> 4, corrected: false });
    }
    let at = H84_SYNDROME.iter().position(|s| *s == syndrome)?;
    Some(Hamming84 { nibble: (code ^ 0x80 >> at) >> 4, corrected: true })
}

/// LSB-first CRC-8, rtl_433's `crc8le`: the same polynomial division as
/// [`crc8`] run through the byte from the other end, which is what a device
/// that transmits its bits least significant first computes.
pub fn crc8le(data: &[u8], poly: u8, init: u8) -> u8 {
    let poly = reflect8(poly);
    let mut crc = reflect8(init);
    for &b in data {
        crc ^= b;
        for _ in 0..8 {
            crc = if crc & 1 != 0 { (crc >> 1) ^ poly } else { crc >> 1 };
        }
    }
    crc
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Encode a message as a BCH(63,51) codeword: the remainder of the
    /// message shifted up, divided by the generator, put in the low bits.
    /// The generator is the product of the minimal polynomials of a and a^3,
    /// which is x^12+x^10+x^8+x^5+x^4+x^3+1.
    fn bch_encode(message: &[bool]) -> Vec<bool> {
        const GEN: u64 = 0b1_0101_0011_1001;
        let mut acc = 0u64;
        // Highest message bit first, as polynomial division runs.
        for bit in message.iter().rev() {
            acc = acc << 1 | u64::from(*bit);
            if acc >> 12 & 1 != 0 {
                acc ^= GEN;
            }
        }
        for _ in 0..12 {
            acc <<= 1;
            if acc >> 12 & 1 != 0 {
                acc ^= GEN;
            }
        }
        let mut code: Vec<bool> = (0..12).map(|i| acc >> i & 1 != 0).collect();
        code.extend_from_slice(message);
        code
    }

    /// Every message comes back, and so does every message with one or two
    /// wrong bits anywhere in the word. Three is refused rather than
    /// mis-corrected wherever the syndromes can tell.
    #[test]
    fn bch63_51_corrects_two_wrong_bits() {
        let mut seed = 0x1234_5678_9abc_def0u64;
        let mut next = move || {
            seed ^= seed << 13;
            seed ^= seed >> 7;
            seed ^= seed << 17;
            seed
        };
        for _ in 0..64 {
            let message: Vec<bool> = (0..51).map(|_| next() & 1 != 0).collect();
            let code = bch_encode(&message);
            assert_eq!(code.len(), 63);
            let mut clean = code.clone();
            assert_eq!(bch63_51(&mut clean), Some(0), "a clean codeword was corrected");
            assert_eq!(clean, code);

            let (a, b) = ((next() % 63) as usize, (next() % 63) as usize);
            let mut one = code.clone();
            one[a] = !one[a];
            assert_eq!(bch63_51(&mut one), Some(1), "one wrong bit at {a}");
            assert_eq!(one, code);

            if a == b {
                continue;
            }
            let mut two = code.clone();
            two[a] = !two[a];
            two[b] = !two[b];
            assert_eq!(bch63_51(&mut two), Some(2), "two wrong bits at {a} and {b}");
            assert_eq!(two, code);
        }
    }

    /// A word too broken to place is refused. Not every triple can be told
    /// from a correctable pair, which is why the caller checks what it got
    /// as well: the Meisei frame carries its own sum over the words.
    #[test]
    fn bch63_51_refuses_some_triples() {
        let code = bch_encode(&[true; 51]);
        let mut refused = 0;
        for at in 0..20usize {
            let mut bad = code.clone();
            for k in [at, at + 7, at + 19] {
                bad[k % 63] = !bad[k % 63];
            }
            refused += u32::from(bch63_51(&mut bad).is_none() || bad != code);
        }
        assert_eq!(refused, 20, "a triple was silently turned into a codeword");
    }

    /// Every nibble survives a round trip through the code, and every single
    /// wrong bit in all eight positions is put back.
    #[test]
    fn hamming84_corrects_any_one_bit() {
        // The generator, read as the parity rows of H: bits 4..8 are the
        // three-of-four sums the DFM sonde transmits after each nibble.
        let encode = |nib: u8| {
            let mut code = nib << 4;
            for (i, row) in H84.iter().enumerate() {
                let parity = (row & 0xF0 & code).count_ones() as u8 & 1;
                code |= parity << (3 - i);
            }
            code
        };
        for nib in 0..16u8 {
            let code = encode(nib);
            assert_eq!(hamming84(code), Some(Hamming84 { nibble: nib, corrected: false }));
            for bit in 0..8 {
                let got = hamming84(code ^ (0x80 >> bit)).expect("a correctable word");
                assert_eq!(got, Hamming84 { nibble: nib, corrected: true }, "bit {bit} of {nib:X}");
            }
        }
    }

    /// Two wrong bits are refused rather than turned into a wrong nibble,
    /// which is what lets a frame say how much of it was guessed.
    #[test]
    fn hamming84_refuses_two_wrong_bits() {
        let code = 0b0000_0000u8;
        assert_eq!(hamming84(code).map(|h| h.nibble), Some(0));
        assert_eq!(hamming84(code ^ 0b1100_0000), None);
        assert_eq!(hamming84(code ^ 0b0000_0011), None);
    }

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
