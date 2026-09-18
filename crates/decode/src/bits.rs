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

/// Manchester chips to bits, taking the first chip of each pair from
/// `phase`, with the number of pairs that did not alternate.
///
/// A pair of equal chips cannot be a Manchester bit, so the count is how much
/// of the run was not this code at all. Which chip of the pair carries the bit
/// is the transmitter's convention and the caller's business: reading the
/// first chip gives one convention and its complement gives the other, so a
/// caller with a sync word finds the polarity from that.
pub fn manchester(chips: &[bool], phase: usize) -> (Vec<bool>, usize) {
    let mut bits = Vec::with_capacity(chips.len().saturating_sub(phase) / 2);
    let mut violations = 0usize;
    for pair in chips[phase.min(chips.len())..].chunks_exact(2) {
        if pair[0] == pair[1] {
            violations += 1;
        }
        bits.push(pair[0]);
    }
    (bits, violations)
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

/// Generator of the BCH(63,51) code [`bch63_51`] reads, as the coefficients
/// of x^12+x^10+x^8+x^5+x^4+x^3+1, highest power in the top bit.
pub const BCH_63_51_GEN: u64 = 0b1_0101_0011_1001;

/// GF(32), built on x^5 + x^2 + 1, as the tables a BCH(31,21) decoder needs.
/// The same pair as [`gf64`]: `exp[i]` is a^i, `log[e]` the power that made it.
fn gf32() -> &'static ([u8; 32], [u8; 32]) {
    static TABLES: std::sync::OnceLock<([u8; 32], [u8; 32])> = std::sync::OnceLock::new();
    TABLES.get_or_init(|| {
        let (mut exp, mut log) = ([0u8; 32], [0u8; 32]);
        let mut x = 1u8;
        for (i, e) in exp.iter_mut().enumerate().take(31) {
            *e = x;
            log[x as usize] = i as u8;
            x <<= 1;
            if x & 0x20 != 0 {
                x ^= 0x25;
            }
        }
        exp[31] = exp[0];
        (exp, log)
    })
}

/// BCH(31,21) over GF(32), correcting up to two wrong bits.
///
/// The same shape as [`bch63_51`]: `code[i]` is the coefficient of x^i, so
/// the ten parity bits sit at the low end and the twenty-one message bits
/// above them. Paging carries this code twice over: POCSAG sends it most
/// significant bit first, FLEX least significant bit first, and the two are
/// therefore each other's word read backwards.
///
/// Returns how many bits were corrected, or `None` where the syndromes name
/// no pair of positions, which is three wrong bits or more.
pub fn bch31_21(code: &mut [bool]) -> Option<u32> {
    if code.len() != 31 {
        return None;
    }
    let (exp, log) = gf32();
    let mul = |a: u8, b: u8| match a == 0 || b == 0 {
        true => 0,
        false => exp[(usize::from(log[a as usize]) + usize::from(log[b as usize])) % 31],
    };
    let syndrome = |power: usize| {
        code.iter().enumerate().filter(|(_, b)| **b).fold(0u8, |s, (i, _)| s ^ exp[i * power % 31])
    };
    let (s1, s3) = (syndrome(1), syndrome(3));
    if s1 == 0 {
        return (s3 == 0).then_some(0);
    }
    let s1_cubed = mul(mul(s1, s1), s1);
    if s1_cubed == s3 {
        let at = usize::from(log[s1 as usize]);
        code[at] = !code[at];
        return Some(1);
    }
    let num = s1_cubed ^ s3;
    let sigma2 = exp[(usize::from(log[num as usize]) + 31 - usize::from(log[s1 as usize])) % 31];
    let mut found = Vec::with_capacity(2);
    for at in 0..31usize {
        let x = exp[(31 - at) % 31];
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

/// Generator of the BCH(31,21) code [`bch31_21`] reads, the product of the
/// minimal polynomials of a and a^3 over GF(32), which is
/// x^10+x^9+x^8+x^6+x^5+x^3+1. Paging standards give it in octal as 03551.
pub const BCH_31_21_GEN: u64 = 0x769;

/// Generator of the BCH(127,106) code [`bch127_106`] reads: the product of
/// the minimal polynomials of a, a^3 and a^5 over GF(128), which is
/// x^21+x^18+x^17+x^15+x^14+x^12+x^11+x^8+x^7+x^6+x^5+x+1.
pub const BCH_127_106_GEN: u64 = 0b10_0110_1101_1001_1110_0011;

/// The parity a systematic BCH or CRC encoder appends: the message shifted
/// up by the generator's degree, divided by the generator, remainder kept.
///
/// `message` is in transmission order, so the first bit is the highest
/// power. `parity_bits` is that degree, and the generator carries its
/// x^degree term in the bit above the parity.
pub fn bch_parity(message: &[bool], generator: u64, parity_bits: usize) -> u64 {
    let mut acc = 0u64;
    let feed = |acc: &mut u64, bit: bool| {
        *acc = *acc << 1 | u64::from(bit);
        if *acc >> parity_bits & 1 != 0 {
            *acc ^= generator;
        }
    };
    for bit in message {
        feed(&mut acc, *bit);
    }
    for _ in 0..parity_bits {
        feed(&mut acc, false);
    }
    acc & ((1u64 << parity_bits) - 1)
}

/// GF(128), built on x^7 + x^3 + 1, as the tables a BCH(127,106) decoder
/// needs. The same pair as [`gf64`]: `exp[i]` is a^i, `log[e]` the power that
/// produced it.
fn gf128() -> &'static ([u8; 128], [u8; 128]) {
    static TABLES: std::sync::OnceLock<([u8; 128], [u8; 128])> = std::sync::OnceLock::new();
    TABLES.get_or_init(|| {
        let (mut exp, mut log) = ([0u8; 128], [0u8; 128]);
        let mut x = 1u8;
        for (i, e) in exp.iter_mut().enumerate().take(127) {
            *e = x;
            log[x as usize] = i as u8;
            x <<= 1;
            if x & 0x80 != 0 {
                x ^= 0x89;
            }
        }
        exp[127] = exp[0];
        (exp, log)
    })
}

/// BCH(127,106) over GF(128), correcting up to three wrong bits.
///
/// `code` is the codeword as bits, `code[i]` the coefficient of x^i, so the
/// parity is at the low end. A shortened code is the same word with zeros in
/// the positions never sent: a 406 MHz beacon's first protected field is
/// (82,61), which is this with forty-five zeros above it, and a correction
/// landing in that padding is evidence the word was worse than three bits
/// wrong rather than a correction to keep.
///
/// Returns how many bits were corrected, or `None` where the syndromes name
/// no pattern of three or fewer. Berlekamp-Massey for the locator rather
/// than Peterson's closed form, which divides by S1^3 + S3 and so cannot
/// place the triples where that is zero: three errors at positions 3, 4 and
/// 34 of a test word are one such.
pub fn bch127_106(code: &mut [bool]) -> Option<u32> {
    if code.len() != 127 {
        return None;
    }
    let (exp, log) = gf128();
    let mul = |a: u8, b: u8| match a == 0 || b == 0 {
        true => 0,
        false => exp[(usize::from(log[a as usize]) + usize::from(log[b as usize])) % 127],
    };
    let div = |a: u8, b: u8| match a == 0 {
        true => 0,
        false => exp[(usize::from(log[a as usize]) + 127 - usize::from(log[b as usize])) % 127],
    };
    // Syndromes S1 to S6. The even ones follow from the odd for a binary
    // code, but computing all six costs nothing and keeps the recursion
    // below the textbook one.
    let syn: Vec<u8> = (1..=6)
        .map(|power: usize| {
            code.iter()
                .enumerate()
                .filter(|(_, b)| **b)
                .fold(0u8, |s, (i, _)| s ^ exp[i * power % 127])
        })
        .collect();
    if syn.iter().all(|s| *s == 0) {
        return Some(0);
    }

    // Berlekamp-Massey: the shortest register that generates the syndromes
    // is the error locator.
    let (mut sigma, mut prev) = (vec![1u8], vec![1u8]);
    let (mut len, mut shift, mut last_d) = (0usize, 1usize, 1u8);
    for n in 0..6usize {
        let mut d = syn[n];
        for i in 1..=len {
            if i < sigma.len() && i <= n {
                d ^= mul(sigma[i], syn[n - i]);
            }
        }
        if d == 0 {
            shift += 1;
            continue;
        }
        let scale = div(d, last_d);
        let was = sigma.clone();
        sigma.resize(sigma.len().max(prev.len() + shift), 0);
        for (i, p) in prev.iter().enumerate() {
            sigma[i + shift] ^= mul(scale, *p);
        }
        if 2 * len <= n {
            (len, prev, last_d, shift) = (n + 1 - len, was, d, 1);
        } else {
            shift += 1;
        }
    }
    if len > 3 {
        return None;
    }

    // Chien search: the roots are the inverses of the error positions, and
    // there have to be as many as the locator's degree or the word was worse
    // than this code can place.
    let mut found = Vec::with_capacity(len);
    for at in 0..127usize {
        let x = exp[(127 - at) % 127];
        let (mut v, mut power) = (0u8, 1u8);
        for c in &sigma {
            v ^= mul(*c, power);
            power = mul(power, x);
        }
        if v == 0 {
            found.push(at);
        }
    }
    if found.len() != len {
        return None;
    }
    for at in found {
        code[at] = !code[at];
    }
    Some(len as u32)
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

/// Hamming(10,6,3) as P25 keys its hex words (TIA-102.BAAA clause 7.3): six
/// data bits followed by four parity bits, one wrong bit corrected and two
/// detected. The generator's rows are the ones the standard prints, so the
/// parity of data bit `i` is `H10_6_TAILS[i]`.
const H10_6_TAILS: [u8; 6] = [0xE, 0xD, 0xB, 0x7, 0x3, 0xC];

/// The four parity bits of a six-bit hex word.
pub fn hamming10_6_parity(hex: u8) -> u8 {
    (0..6).filter(|i| hex >> (5 - i) & 1 != 0).fold(0u8, |p, i| p ^ H10_6_TAILS[i])
}

/// Read a ten-bit Hamming(10,6,3) word: the six data bits, and how many bits
/// were corrected. `None` where the syndrome names no single position, which
/// is two bits wrong.
///
/// `code` is the word as it arrived, the data in bits 9 to 4 and the parity
/// in bits 3 to 0.
pub fn hamming10_6(code: u16) -> Option<(u8, u32)> {
    let hex = (code >> 4) as u8 & 0x3f;
    let syndrome = hamming10_6_parity(hex) ^ (code as u8 & 0xf);
    if syndrome == 0 {
        return Some((hex, 0));
    }
    if let Some(bit) = H10_6_TAILS.iter().position(|t| *t == syndrome) {
        return Some((hex ^ (1 << (5 - bit)), 1));
    }
    // A syndrome that is one parity bit is that parity bit wrong, and the
    // data stands. Anything else is two bits wrong or more.
    matches!(syndrome, 1 | 2 | 4 | 8).then_some((hex, 1))
}

/// The BCH(63,16,23) code P25 protects its network identifier with
/// (TIA-102.BAAA clause 7.1), as one basis codeword per message bit.
///
/// The generator is the product of the minimal polynomials of a, a^3, ... ,
/// a^21 over GF(64), which is what a t=11 code of length 63 asks for, and
/// comes out at degree 47. Computed rather than written down so the code is
/// its own statement of which roots it has.
fn bch63_16_basis() -> &'static [u64; 16] {
    static BASIS: std::sync::OnceLock<[u64; 16]> = std::sync::OnceLock::new();
    BASIS.get_or_init(|| {
        let (exp, _) = gf64();
        // Multiply two GF(2) polynomials held as bit masks.
        let mul = |a: u64, b: u64| {
            let mut out = 0u64;
            for i in 0..64 {
                if a >> i & 1 != 0 {
                    out ^= b << i;
                }
            }
            out
        };
        // The minimal polynomial of a^power: the product of (x - a^j) over
        // its conjugates, built in GF(64) and coming out with binary
        // coefficients.
        let minimal = |power: usize| {
            let mut roots = Vec::new();
            let mut j = power % 63;
            loop {
                if !roots.contains(&j) {
                    roots.push(j);
                }
                j = j * 2 % 63;
                if j == power % 63 {
                    break;
                }
            }
            // Coefficients in GF(64), lowest power first.
            let mut poly = vec![1u8];
            for r in roots {
                let root = exp[r];
                let mut next = vec![0u8; poly.len() + 1];
                for (i, &c) in poly.iter().enumerate() {
                    next[i + 1] ^= c;
                    next[i] ^= gf64_mul(c, root);
                }
                poly = next;
            }
            poly.iter().enumerate().fold(0u64, |m, (i, &c)| m | u64::from(c & 1) << i)
        };
        let mut generator = 1u64;
        for power in (1..=21).step_by(2) {
            let m = minimal(power);
            // The minimal polynomials of the odd powers are distinct or
            // equal, never partly shared, so a product of the new ones is
            // their least common multiple.
            if !divides(m, generator) {
                generator = mul(generator, m);
            }
        }
        debug_assert_eq!(63 - degree(generator), 16, "the generator is not degree 47");
        let mut basis = [0u64; 16];
        for (i, row) in basis.iter_mut().enumerate() {
            let message = 1u64 << (62 - i);
            *row = message | modulo(message, generator);
        }
        basis
    })
}

fn gf64_mul(a: u8, b: u8) -> u8 {
    let (exp, log) = gf64();
    match a == 0 || b == 0 {
        true => 0,
        false => exp[(usize::from(log[a as usize]) + usize::from(log[b as usize])) % 63],
    }
}

fn degree(poly: u64) -> usize {
    63 - poly.leading_zeros() as usize
}

fn modulo(mut value: u64, generator: u64) -> u64 {
    let d = degree(generator);
    while value != 0 && degree(value) >= d {
        value ^= generator << (degree(value) - d);
    }
    value
}

fn divides(divisor: u64, value: u64) -> bool {
    value != 0 && modulo(value, divisor) == 0
}

/// The BCH(63,16,23) codeword a sixteen-bit message makes, the message in
/// the top bits and the 47 parity bits below it.
pub fn bch63_16_encode(message: u16) -> u64 {
    bch63_16_basis()
        .iter()
        .enumerate()
        .filter(|(i, _)| message >> (15 - i) & 1 != 0)
        .fold(0u64, |cw, (_, row)| cw ^ row)
}

/// Read a BCH(63,16,23) codeword: the message, and how many bits were
/// wrong. `None` where no codeword is within the eleven bits the code
/// corrects, which on P25 is a network identifier read out of noise.
///
/// `code` is the 63 bits as they arrived, the first in bit 62. The search is
/// over all 65536 codewords in Gray code order, one exclusive-or and one
/// population count each, which is both the optimal decoder and quicker to
/// be sure of than a syndrome decoder for eleven errors.
pub fn bch63_16(code: u64) -> Option<(u16, u32)> {
    let basis = bch63_16_basis();
    let (mut message, mut codeword) = (0u16, 0u64);
    let (mut best, mut best_message) = (u32::MAX, 0u16);
    for step in 0..1u32 << 16 {
        let d = (codeword ^ code).count_ones();
        if d < best {
            best = d;
            best_message = message;
        }
        let bit = step.trailing_ones() as usize;
        if bit < 16 {
            message ^= 1 << (15 - bit);
            codeword ^= basis[bit];
        }
    }
    (best <= 11).then_some((best_message, best))
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

    /// The same exercise for BCH(31,21), the paging code, plus the eight
    /// codewords multimon-ng's `bch_flex_encode` produces for the same
    /// messages. Those pin the generator and the bit order together: FLEX
    /// puts its message bits at 0 to 20 and its parity at 21 to 30, least
    /// significant bit first on the air, so a FLEX word is this codeword
    /// read from the top down.
    #[test]
    fn bch31_21_corrects_two_wrong_bits_and_matches_multimon() {
        let encode = |message: &[bool]| {
            let parity = bch_parity(message, BCH_31_21_GEN, 10);
            let mut code: Vec<bool> = (0..10).map(|i| parity >> i & 1 != 0).collect();
            code.extend(message.iter().rev().copied());
            code
        };
        // (21-bit FLEX message, 31-bit FLEX codeword), from multimon-ng's
        // bch.c compiled and run over the messages on the left.
        let reference: [(u32, u32); 6] = [
            (0x00_0001, 0x16E0_0001),
            (0x10_0000, 0x4B70_0000),
            (0x0A_AAAA, 0x270A_AAAA),
            (0x01_2345, 0x3D41_2345),
            (0x1F_FFFF, 0x7FFF_FFFF),
            (0x0F_0F0F, 0x480F_0F0F),
        ];
        for (message, word) in reference {
            // The message in transmission order: FLEX bit 0 goes first.
            let bits: Vec<bool> = (0..21).map(|i| message >> i & 1 != 0).collect();
            let code = encode(&bits);
            let got = (0..31u32).fold(0u32, |w, i| w | u32::from(code[30 - i as usize]) << i);
            assert_eq!(got, word, "message {message:#08x}");
            let mut clean = code.clone();
            assert_eq!(bch31_21(&mut clean), Some(0), "a clean codeword was corrected");
        }

        let mut seed = 0x0bad_c0de_1234_5678u64;
        let mut next = move || {
            seed ^= seed << 13;
            seed ^= seed >> 7;
            seed ^= seed << 17;
            seed
        };
        for _ in 0..64 {
            let message: Vec<bool> = (0..21).map(|_| next() & 1 != 0).collect();
            let code = encode(&message);
            assert_eq!(code.len(), 31);
            let (a, b) = ((next() % 31) as usize, (next() % 31) as usize);
            let mut one = code.clone();
            one[a] = !one[a];
            assert_eq!(bch31_21(&mut one), Some(1), "one wrong bit at {a}");
            assert_eq!(one, code);
            if a == b {
                continue;
            }
            let mut two = code.clone();
            two[a] = !two[a];
            two[b] = !two[b];
            assert_eq!(bch31_21(&mut two), Some(2), "two wrong bits at {a} and {b}");
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

    /// Bits written as a string of ones and zeros, spaces ignored, in the
    /// order the beacon transmits them.
    fn bits_of(s: &str) -> Vec<bool> {
        s.chars().filter(|c| *c == '0' || *c == '1').map(|c| c == '1').collect()
    }

    /// The two worked examples in Annex B of C/S T.001, the 406 MHz beacon
    /// specification: a short message from a float-free EPIRB in the United
    /// States, and the position field of a user-location beacon at
    /// 43 32' N 001 28' E. Both give the parity their own long division
    /// produced, so this pins the generators as well as the division.
    #[test]
    fn the_beacon_specification_parity_examples_come_back() {
        let pdf1 = bits_of("0101011011100110100000000100000000000010001000000010000000001");
        assert_eq!(pdf1.len(), 61);
        let bch1 = bch_parity(&pdf1, BCH_127_106_GEN, 21);
        assert_eq!(bch1, 0b001011001010101001001, "{bch1:021b}");

        let pdf2 = bits_of("10 0101 0111 0000 0000 0001 0111");
        assert_eq!(pdf2.len(), 26);
        let bch2 = bch_parity(&pdf2, BCH_63_51_GEN, 12);
        assert_eq!(bch2, 0b0001_0101_0001, "{bch2:012b}");
    }

    /// Encode 61 data bits as the shortened (82,61) BCH(127,106) codeword a
    /// beacon's first protected field is: parity at the low end, the
    /// forty-five never-sent positions zero at the top.
    fn bch127_shortened(message: &[bool]) -> Vec<bool> {
        let parity = bch_parity(message, BCH_127_106_GEN, 21);
        let mut code: Vec<bool> = (0..21).map(|i| parity >> i & 1 != 0).collect();
        code.extend(message.iter().rev().copied());
        code.resize(127, false);
        code
    }

    /// Every word comes back, and so does every word with one, two or three
    /// wrong bits anywhere in the 82 that were sent.
    #[test]
    fn bch127_106_corrects_three_wrong_bits() {
        let mut seed = 0x0bad_c0de_1234_5678u64;
        let mut next = move || {
            seed ^= seed << 13;
            seed ^= seed >> 7;
            seed ^= seed << 17;
            seed
        };
        for _ in 0..64 {
            let message: Vec<bool> = (0..61).map(|_| next() & 1 != 0).collect();
            let code = bch127_shortened(&message);
            let mut clean = code.clone();
            assert_eq!(bch127_106(&mut clean), Some(0), "a clean codeword was corrected");
            assert_eq!(clean, code);

            let mut at: Vec<usize> = Vec::new();
            while at.len() < 3 {
                let k = (next() % 82) as usize;
                if !at.contains(&k) {
                    at.push(k);
                }
            }
            for n in 1..=3usize {
                let mut bad = code.clone();
                for k in &at[..n] {
                    bad[*k] = !bad[*k];
                }
                assert_eq!(bch127_106(&mut bad), Some(n as u32), "{n} wrong bits at {at:?}");
                assert_eq!(bad, code, "{n} wrong bits at {at:?}");
            }
        }
    }

    /// Four wrong bits are past the code, and what matters is that it says
    /// so rather than handing back a different message. Measured over 200
    /// quadruples in the 82 sent positions: 200 refused or corrected into
    /// the padding, none silently turned into another codeword.
    #[test]
    fn bch127_106_refuses_four_wrong_bits() {
        let code = bch127_shortened(&[true; 61]);
        let mut honest = 0;
        for at in 0..200usize {
            let mut bad = code.clone();
            for k in [at, at + 11, at + 29, at + 53] {
                let k = k % 82;
                bad[k] = !bad[k];
            }
            let placed = bch127_106(&mut bad);
            // Either refused, or the correction landed somewhere the
            // shortened code never sends, which the caller checks.
            let padding_dirty = bad[82..].iter().any(|b| *b);
            honest += u32::from(placed.is_none() || padding_dirty || bad != code);
        }
        assert_eq!(honest, 200, "a quadruple was silently turned into a codeword");
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

    /// Every hex word survives its own parity, and every single wrong bit in
    /// the ten is put back. Two wrong bits in the data are refused or land on
    /// another word: the code's distance is 3, so it cannot do better.
    #[test]
    fn hamming_10_6_corrects_one_bit_of_ten() {
        let mut corrected = 0;
        for hex in 0u8..64 {
            let word = u16::from(hex) << 4 | u16::from(hamming10_6_parity(hex));
            assert_eq!(hamming10_6(word), Some((hex, 0)));
            for bit in 0..10 {
                assert_eq!(hamming10_6(word ^ 1 << bit), Some((hex, 1)), "hex {hex} bit {bit}");
                corrected += 1;
            }
        }
        assert_eq!(corrected, 640);
        // Two wrong bits are past a distance-3 code: of the 45 pairs of
        // positions, 21 leave a syndrome no single bit could and are
        // refused, and the other 24 read back as a different hex word. The
        // Reed-Solomon over the words is what catches those.
        let word = u16::from(0b10_1100u8) << 4 | u16::from(hamming10_6_parity(0b10_1100));
        let mut refused = 0;
        let mut wrong_word = 0;
        for a in 0..10 {
            for b in (a + 1)..10 {
                match hamming10_6(word ^ 1 << a ^ 1 << b) {
                    None => refused += 1,
                    Some((hex, _)) => {
                        assert_ne!(hex, 0b10_1100, "two wrong bits read as the word sent");
                        wrong_word += 1;
                    }
                }
            }
        }
        assert_eq!((refused, wrong_word), (21, 24));
    }

    /// The network identifier code: eleven wrong bits of 63 still read back
    /// the NAC and the DUID, and twelve are past what it can promise.
    #[test]
    fn bch_63_16_corrects_eleven_bits() {
        // 0x293 is the default NAC, and 5 the DUID of a voice frame.
        let message = 0x293 << 4 | 5;
        let code = bch63_16_encode(message);
        assert_eq!(bch63_16(code), Some((message, 0)));
        for errors in 1..=11u32 {
            // Spread the wrong bits across the word rather than bunching
            // them, which is the pattern a fade leaves.
            let mut wrong = code;
            for i in 0..errors {
                wrong ^= 1 << (i * 5 % 63);
            }
            assert_eq!(bch63_16(wrong), Some((message, errors)), "{errors} wrong bits");
        }
        let mut wrong = code;
        for i in 0..12u32 {
            wrong ^= 1 << (i * 5 % 63);
        }
        assert_ne!(bch63_16(wrong), Some((message, 12)), "twelve bits is past the code");
        // Noise is mostly refused rather than read as some other frame: a
        // word is within eleven bits of a codeword one time in two hundred,
        // which is why a P25 framer asks for the sync word as well.
        // Measured over ten thousand random words.
        let mut seed = 0x243f_6a88_85a3_08d3u64;
        let read = (0..10_000)
            .filter(|_| {
                seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
                bch63_16(seed >> 1).is_some()
            })
            .count();
        assert_eq!(read, 52, "a NAC read out of noise");
    }
}
