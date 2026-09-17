//! Baseline JPEG's entropy coding and inverse transform, with no file round
//! it.
//!
//! A satellite that sends pictures as compressed strips sends JPEG's blocks
//! and nothing else: no markers, no headers, no tables, because the tables
//! are the standard ones and the quantisation is a single quality byte in the
//! packet. So what is needed is the part of a JPEG decoder below the file
//! format: the standard luminance Huffman tables, a quantisation table from
//! a quality factor, and an 8x8 block of pixels out of a run of bits.
//!
//! Meteor's LRPT is the first caller (`crate::lrpt`); its high resolution
//! link sends the same blocks.
//!
//! Tables from ITU-T T.81 annex K, which is where every JPEG encoder takes
//! them from. The quality factor mapping is the one the Independent JPEG
//! Group's `libjpeg` uses and the one Meteor's encoder was built against, by
//! way of `mlrpt` (dvdesolve/mlrpt, `src/decoder/met_jpg.c`).

/// Where each coefficient of a block sits once the zigzag is undone.
pub const ZIGZAG: [usize; 64] = [
    0, 1, 5, 6, 14, 15, 27, 28, 2, 4, 7, 13, 16, 26, 29, 42, 3, 8, 12, 17, 25, 30, 41, 43, 9, 11,
    18, 24, 31, 40, 44, 53, 10, 19, 23, 32, 39, 45, 52, 54, 20, 22, 33, 38, 46, 51, 55, 60, 21, 34,
    37, 47, 50, 56, 59, 61, 35, 36, 48, 49, 57, 58, 62, 63,
];

/// The luminance quantisation table of T.81 annex K, which every quality
/// factor is a scaling of.
pub const LUMA_QUANT: [u8; 64] = [
    16, 11, 10, 16, 24, 40, 51, 61, 12, 12, 14, 19, 26, 58, 60, 55, 14, 13, 16, 24, 40, 57, 69, 56,
    14, 17, 22, 29, 51, 87, 80, 62, 18, 22, 37, 56, 68, 109, 103, 77, 24, 35, 55, 64, 81, 104, 113,
    92, 49, 64, 78, 87, 103, 121, 120, 101, 72, 92, 95, 98, 112, 100, 103, 99,
];

/// How many code lengths a Huffman table lists.
const MAX_CODE_BITS: usize = 16;

/// The standard luminance DC table: how many codes of each length, then the
/// categories they name.
const DC_BITS: [u8; MAX_CODE_BITS] = [0, 1, 5, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0];
const DC_VALUES: [u8; 12] = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11];

/// And the standard luminance AC table, whose values are a run of zeros in
/// the top nibble and a coefficient size in the bottom.
const AC_BITS: [u8; MAX_CODE_BITS] = [0, 2, 1, 3, 3, 2, 4, 3, 5, 5, 4, 4, 0, 0, 1, 125];
const AC_VALUES: [u8; 162] = [
    0x01, 0x02, 0x03, 0x00, 0x04, 0x11, 0x05, 0x12, 0x21, 0x31, 0x41, 0x06, 0x13, 0x51, 0x61, 0x07,
    0x22, 0x71, 0x14, 0x32, 0x81, 0x91, 0xa1, 0x08, 0x23, 0x42, 0xb1, 0xc1, 0x15, 0x52, 0xd1, 0xf0,
    0x24, 0x33, 0x62, 0x72, 0x82, 0x09, 0x0a, 0x16, 0x17, 0x18, 0x19, 0x1a, 0x25, 0x26, 0x27, 0x28,
    0x29, 0x2a, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3a, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49,
    0x4a, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5a, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69,
    0x6a, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79, 0x7a, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89,
    0x8a, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98, 0x99, 0x9a, 0xa2, 0xa3, 0xa4, 0xa5, 0xa6, 0xa7,
    0xa8, 0xa9, 0xaa, 0xb2, 0xb3, 0xb4, 0xb5, 0xb6, 0xb7, 0xb8, 0xb9, 0xba, 0xc2, 0xc3, 0xc4, 0xc5,
    0xc6, 0xc7, 0xc8, 0xc9, 0xca, 0xd2, 0xd3, 0xd4, 0xd5, 0xd6, 0xd7, 0xd8, 0xd9, 0xda, 0xe1, 0xe2,
    0xe3, 0xe4, 0xe5, 0xe6, 0xe7, 0xe8, 0xe9, 0xea, 0xf1, 0xf2, 0xf3, 0xf4, 0xf5, 0xf6, 0xf7, 0xf8,
    0xf9, 0xfa,
];

/// One canonical Huffman table: the code of each length, and what it names.
#[derive(Clone, Debug)]
pub struct Table {
    /// `(length, code, value)`, shortest code first, which is the order the
    /// canonical assignment produces.
    codes: Vec<(u8, u16, u8)>,
}

impl Table {
    /// Assign codes the way T.81 annex C does: shortest first, in order,
    /// doubling at each length.
    pub fn new(bits: &[u8; MAX_CODE_BITS], values: &[u8]) -> Self {
        let mut codes = Vec::with_capacity(values.len());
        let mut code = 0u16;
        let mut at = 0usize;
        for (i, &count) in bits.iter().enumerate() {
            let length = i as u8 + 1;
            for _ in 0..count {
                codes.push((length, code, values[at]));
                code = code.wrapping_add(1);
                at += 1;
            }
            code <<= 1;
        }
        Self { codes }
    }

    pub fn luma_dc() -> Self {
        Self::new(&DC_BITS, &DC_VALUES)
    }

    pub fn luma_ac() -> Self {
        Self::new(&AC_BITS, &AC_VALUES)
    }

    /// The code that names `value`, as its length and the code itself.
    /// What an encoder needs, and what a test building a packet by hand
    /// needs.
    pub fn code_for(&self, value: u8) -> Option<(u8, u16)> {
        self.codes.iter().find(|c| c.2 == value).map(|&(length, code, _)| (length, code))
    }

    /// What the next code in `bits` names, and how long it was.
    fn lookup(&self, bits: &mut Bits<'_>) -> Option<(u8, u8)> {
        let word = bits.peek16();
        for &(length, code, value) in &self.codes {
            if word >> (16 - length) == code {
                bits.skip(usize::from(length));
                return Some((value, length));
            }
        }
        None
    }
}

/// A run of bits, most significant first, as JPEG reads them.
pub struct Bits<'a> {
    bytes: &'a [u8],
    at: usize,
}

impl<'a> Bits<'a> {
    pub fn new(bytes: &'a [u8]) -> Self {
        Self { bytes, at: 0 }
    }

    /// Bits read so far.
    pub fn position(&self) -> usize {
        self.at
    }

    /// The next sixteen bits, zero padded past the end, which is what a
    /// Huffman lookup needs and what T.81 allows a decoder to do at the tail
    /// of a scan.
    pub fn peek16(&self) -> u16 {
        let mut word = 0u16;
        for k in 0..16 {
            let at = self.at + k;
            let bit = match self.bytes.get(at / 8) {
                Some(b) => (b >> (7 - at % 8)) & 1,
                None => 0,
            };
            word = (word << 1) | u16::from(bit);
        }
        word
    }

    pub fn skip(&mut self, n: usize) {
        self.at += n;
    }

    /// Take `n` bits as a number.
    pub fn take(&mut self, n: usize) -> u16 {
        let word = self.peek16();
        self.at += n;
        match n {
            0 => 0,
            n => word >> (16 - n),
        }
    }

    /// Whether the reader has run off the end, which for a packet whose tail
    /// was lost is how the block decoder stops.
    pub fn past_end(&self) -> bool {
        self.at > self.bytes.len() * 8
    }
}

/// The signed value a category and its bits name, T.81 table F.2: the top
/// half of the range is positive and the bottom half negative.
pub fn extend(category: u8, raw: u16) -> i32 {
    if category == 0 {
        return 0;
    }
    let bits = u32::from(category);
    let value = i32::from(raw);
    match value >> (bits - 1) != 0 {
        true => value,
        false => value - ((1 << bits) - 1),
    }
}

/// The quantisation table a quality factor names, as `libjpeg` scales it.
///
/// The factor arrives in the packet, one byte, and is the same for every
/// block of that packet.
pub fn quant_table(quality: u8) -> [i32; 64] {
    let q = f64::from(quality);
    let scale = match (20.0..50.0).contains(&q) {
        true => 5000.0 / q,
        false => 200.0 - 2.0 * q,
    };
    let mut table = [1i32; 64];
    for (t, &base) in table.iter_mut().zip(&LUMA_QUANT) {
        *t = ((scale / 100.0 * f64::from(base)).round() as i32).max(1);
    }
    table
}

/// The inverse discrete cosine transform of one 8x8 block, the slow way the
/// standard defines it.
pub fn idct8x8(coefficients: &[f64; 64]) -> [f64; 64] {
    // cos(pi/16 * (2y+1) * x), which is the whole of the transform.
    let cosine = |y: usize, x: usize| {
        (std::f64::consts::PI / 16.0 * (2.0 * y as f64 + 1.0) * x as f64).cos()
    };
    let alpha = |x: usize| match x {
        0 => std::f64::consts::FRAC_1_SQRT_2,
        _ => 1.0,
    };
    let mut out = [0.0f64; 64];
    for y in 0..8 {
        for x in 0..8 {
            let mut sum = 0.0;
            for u in 0..8 {
                for v in 0..8 {
                    sum +=
                        coefficients[v * 8 + u] * alpha(u) * alpha(v) * cosine(x, u) * cosine(y, v);
                }
            }
            out[y * 8 + x] = sum / 4.0;
        }
    }
    out
}

/// The tables a run of blocks is read with.
pub struct Blocks {
    dc: Table,
    ac: Table,
}

impl Default for Blocks {
    fn default() -> Self {
        Self::new()
    }
}

impl Blocks {
    pub fn new() -> Self {
        Self { dc: Table::luma_dc(), ac: Table::luma_ac() }
    }

    /// Read one block: its coefficients, dequantised and zigzagged back into
    /// place, with the DC carried on from the block before it.
    ///
    /// `None` where the bits are not a block, which is what a broken packet
    /// gives and is the end of what can be read from it.
    pub fn coefficients(
        &self,
        bits: &mut Bits<'_>,
        quant: &[i32; 64],
        previous_dc: &mut i32,
    ) -> Option<[f64; 64]> {
        let (category, _) = self.dc.lookup(bits)?;
        if category > 11 {
            return None;
        }
        let raw = bits.take(usize::from(category));
        let mut zigzagged = [0i32; 64];
        *previous_dc += extend(category, raw);
        zigzagged[0] = *previous_dc;

        let mut k = 1usize;
        while k < 64 {
            let (value, _) = self.ac.lookup(bits)?;
            let (run, size) = (usize::from(value >> 4), value & 0x0f);
            if run == 0 && size == 0 {
                break;
            }
            k += run;
            if k >= 64 {
                break;
            }
            if size > 0 {
                let raw = bits.take(usize::from(size));
                zigzagged[k] = extend(size, raw);
            }
            k += 1;
        }
        if bits.past_end() {
            return None;
        }
        let mut coefficients = [0.0f64; 64];
        for (i, c) in coefficients.iter_mut().enumerate() {
            *c = f64::from(zigzagged[ZIGZAG[i]]) * f64::from(quant[i]);
        }
        Some(coefficients)
    }

    /// One block as pixels: the transform, the level shift back up and the
    /// clamp.
    pub fn pixels(
        &self,
        bits: &mut Bits<'_>,
        quant: &[i32; 64],
        previous_dc: &mut i32,
    ) -> Option<[u8; 64]> {
        let coefficients = self.coefficients(bits, quant, previous_dc)?;
        let block = idct8x8(&coefficients);
        let mut out = [0u8; 64];
        for (o, v) in out.iter_mut().zip(&block) {
            *o = (v + 128.0).round().clamp(0.0, 255.0) as u8;
        }
        Some(out)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The canonical assignment against the codes T.81 lists for the
    /// luminance DC table: `00` for category nought, three bits for one to
    /// five, and a one longer for each after that.
    #[test]
    fn the_dc_table_is_the_published_one() {
        let t = Table::luma_dc();
        assert_eq!(t.codes.len(), 12);
        assert_eq!(t.codes[0], (2, 0b00, 0));
        assert_eq!(t.codes[1], (3, 0b010, 1));
        assert_eq!(t.codes[5], (3, 0b110, 5));
        assert_eq!(t.codes[6], (4, 0b1110, 6));
        assert_eq!(t.codes[11], (9, 0b1_1111_1110, 11));
    }

    /// And the AC table's first codes and its end of block.
    #[test]
    fn the_ac_table_is_the_published_one() {
        let t = Table::luma_ac();
        assert_eq!(t.codes.len(), 162);
        // End of block is 1010, and 00 is a run of nothing with a size of
        // one.
        assert_eq!(t.codes[0], (2, 0b00, 0x01));
        let eob = t.codes.iter().find(|c| c.2 == 0x00).expect("an end of block");
        assert_eq!(*eob, (4, 0b1010, 0x00));
        let zrl = t.codes.iter().find(|c| c.2 == 0xf0).expect("a run of sixteen");
        assert_eq!(*zrl, (11, 0b111_1111_1001, 0xf0));
    }

    /// A quality factor either side of where the two scalings meet, and the
    /// floor of one that nothing may quantise to nought.
    #[test]
    fn the_quantisation_table_follows_the_quality() {
        // Quality 100 is the table itself.
        let q100 = quant_table(100);
        assert_eq!(q100[0], 1, "the DC step at quality 100");
        assert_eq!(q100[63], 1);
        let q50 = quant_table(50);
        assert_eq!(q50[0], 16, "quality 50 is the table unscaled");
        assert_eq!(q50[1], 11);
        assert_eq!(q50[63], 99);
        // Below 50 the scaling is the other branch: 5000/40 is 125%.
        let q40 = quant_table(40);
        assert_eq!(q40[0], 20);
        assert!(quant_table(1).iter().all(|&v| v >= 1));
    }

    /// The signed value a category names, at both ends of a category.
    #[test]
    fn a_coefficient_extends_to_its_signed_value() {
        assert_eq!(extend(0, 0), 0);
        assert_eq!(extend(1, 1), 1);
        assert_eq!(extend(1, 0), -1);
        assert_eq!(extend(3, 0b111), 7);
        assert_eq!(extend(3, 0b100), 4);
        assert_eq!(extend(3, 0b011), -4);
        assert_eq!(extend(3, 0b000), -7);
    }

    /// A block of one value: a DC coefficient alone comes back as a flat
    /// block at that level.
    #[test]
    fn a_dc_only_block_is_flat() {
        let mut coefficients = [0.0f64; 64];
        // The transform's DC gain is eight, so a flat block at 64 counts
        // above the level shift is a DC coefficient of 512.
        coefficients[0] = 512.0;
        let block = idct8x8(&coefficients);
        for v in block {
            assert!((v - 64.0).abs() < 1e-9, "{v} is not flat");
        }
    }

    /// The whole of one block, encoded by hand as a transmitter would and
    /// read back: a DC of two steps and one AC coefficient.
    #[test]
    fn a_hand_built_block_reads_back() {
        // DC code 011 is category 2, whose two bits 11 are +3; AC code 00
        // is a run of nothing and a size of one, whose bit 1 is +1; then
        // end of block, 1010.
        let bits = [0b0111_1001, 0b1010_0000];
        let quant = quant_table(50);
        let blocks = Blocks::new();
        let mut reader = Bits::new(&bits);
        let mut dc = 0;
        let coefficients = blocks.coefficients(&mut reader, &quant, &mut dc).expect("a block");
        assert_eq!(dc, 3, "three DC steps");
        assert_eq!(coefficients[0], 3.0 * 16.0, "the DC step is the table's first entry");
        assert_eq!(coefficients[1], 11.0, "one step of the first AC coefficient");
        assert!(coefficients[2..].iter().all(|&c| c == 0.0));
        // The DC carries on into the next block, which is what makes a
        // packet's blocks a run rather than independent.
        let mut reader = Bits::new(&bits);
        blocks.coefficients(&mut reader, &quant, &mut dc).expect("a second block");
        assert_eq!(dc, 6);
    }

    /// Nonsense in: the block decoder says so rather than inventing pixels.
    #[test]
    fn a_broken_block_is_refused() {
        let quant = quant_table(50);
        let blocks = Blocks::new();
        // A run of ones is no DC code in the table.
        let bits = [0xff; 8];
        let mut reader = Bits::new(&bits);
        let mut dc = 0;
        assert!(blocks.pixels(&mut reader, &quant, &mut dc).is_none());
        // And an empty packet is nothing to read.
        let mut reader = Bits::new(&[]);
        assert!(blocks.pixels(&mut reader, &quant, &mut dc).is_none());
    }
}
