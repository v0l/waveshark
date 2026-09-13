//! The variable length codes an MPEG-2 picture is written in.
//!
//! ISO/IEC 13818-2 annex B: the macroblock address increment, the intra
//! macroblock types, the two DC size tables, and the two run and level
//! tables. A code is matched by peeking its longest form and comparing the
//! top bits, which is slower than a lookup tree and is not where the time
//! goes: a picture is a million coefficients and each costs one pass over a
//! table that fits in cache.
//!
//! The tables were generated from the ones in ffmpeg's `mpeg12data.c` rather
//! than typed out of the standard, because a transcription error in a run and
//! level table does not fail: it decodes most of a picture and puts a smear
//! through the rest of it.

/// One entry: the code, how long it is, and the run and level it stands for.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Code {
    pub bits: u32,
    pub len: u8,
    pub run: i8,
    pub level: i8,
}

/// A code with no run or level behind it: a count, a size or a type.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Plain {
    pub bits: u32,
    pub len: u8,
    pub value: u8,
}

/// Read one code out of `bits`, or `None` where the next bits are not in the
/// table, which means the picture is damaged.
pub fn read(bits: &mut super::bits::Bits<'_>, table: &[Code]) -> Option<Code> {
    let peek = bits.peek(16);
    for c in table {
        if peek >> (16 - c.len) == c.bits {
            bits.take(c.len as usize);
            return Some(*c);
        }
    }
    None
}

/// The same for a table that carries a plain value.
pub fn read_plain(bits: &mut super::bits::Bits<'_>, table: &[Plain]) -> Option<u8> {
    let peek = bits.peek(16);
    for c in table {
        if peek >> (16 - c.len) == c.bits {
            bits.take(c.len as usize);
            return Some(c.value);
        }
    }
    None
}

/// Whether the next bits are this code, and take them if so.
pub fn matches(bits: &mut super::bits::Bits<'_>, code: (u32, u8)) -> bool {
    let (v, len) = code;
    if bits.peek(16) >> (16 - len) == v {
        bits.take(len as usize);
        return true;
    }
    false
}

/// Table B.1: how many macroblocks on from the last one. The escape adds 33
/// and is read again.
pub const ADDRESS_INCREMENT: [Plain; 33] = {
    let mut out = [Plain { bits: 0, len: 0, value: 0 }; 33];
    let codes: [(u32, u8); 33] = [
        (0x1, 1),
        (0x3, 3),
        (0x2, 3),
        (0x3, 4),
        (0x2, 4),
        (0x3, 5),
        (0x2, 5),
        (0x7, 7),
        (0x6, 7),
        (0xB, 8),
        (0xA, 8),
        (0x9, 8),
        (0x8, 8),
        (0x7, 8),
        (0x6, 8),
        (0x17, 10),
        (0x16, 10),
        (0x15, 10),
        (0x14, 10),
        (0x13, 10),
        (0x12, 10),
        (0x23, 11),
        (0x22, 11),
        (0x21, 11),
        (0x20, 11),
        (0x1F, 11),
        (0x1E, 11),
        (0x1D, 11),
        (0x1C, 11),
        (0x1B, 11),
        (0x1A, 11),
        (0x19, 11),
        (0x18, 11),
    ];
    let mut i = 0;
    while i < 33 {
        out[i] = Plain { bits: codes[i].0, len: codes[i].1, value: i as u8 + 1 };
        i += 1;
    }
    out
};

/// The escape that adds 33 to the increment, and the stuffing an encoder pads
/// a slice with, which is read and thrown away.
pub const ADDRESS_ESCAPE: (u32, u8) = (0x8, 11);
pub const ADDRESS_STUFFING: (u32, u8) = (0xF, 11);

/// Table B.12: the number of bits in a luminance DC differential.
pub const DC_SIZE_LUMA: [Plain; 12] = dc_table(
    [0x4, 0x0, 0x1, 0x5, 0x6, 0xE, 0x1E, 0x3E, 0x7E, 0xFE, 0x1FE, 0x1FF],
    [3, 2, 2, 3, 3, 4, 5, 6, 7, 8, 9, 9],
);

/// Table B.13: the same for chrominance, which has one bit less to spend on
/// its shortest codes because there is less of it.
pub const DC_SIZE_CHROMA: [Plain; 12] = dc_table(
    [0x0, 0x1, 0x2, 0x6, 0xE, 0x1E, 0x3E, 0x7E, 0xFE, 0x1FE, 0x3FE, 0x3FF],
    [2, 2, 2, 3, 4, 5, 6, 7, 8, 9, 10, 10],
);

const fn dc_table(codes: [u32; 12], lens: [u8; 12]) -> [Plain; 12] {
    let mut out = [Plain { bits: 0, len: 0, value: 0 }; 12];
    let mut i = 0;
    while i < 12 {
        out[i] = Plain { bits: codes[i], len: lens[i], value: i as u8 };
        i += 1;
    }
    out
}

/// EN 300 744's Table B.14: the run and level codes every picture uses unless the intra VLC format says otherwise.
pub const TABLE_ZERO: [Code; 111] = [
    Code { bits: 0x3, len: 2, run: 0, level: 1 },
    Code { bits: 0x4, len: 4, run: 0, level: 2 },
    Code { bits: 0x5, len: 5, run: 0, level: 3 },
    Code { bits: 0x6, len: 7, run: 0, level: 4 },
    Code { bits: 0x26, len: 8, run: 0, level: 5 },
    Code { bits: 0x21, len: 8, run: 0, level: 6 },
    Code { bits: 0xA, len: 10, run: 0, level: 7 },
    Code { bits: 0x1D, len: 12, run: 0, level: 8 },
    Code { bits: 0x18, len: 12, run: 0, level: 9 },
    Code { bits: 0x13, len: 12, run: 0, level: 10 },
    Code { bits: 0x10, len: 12, run: 0, level: 11 },
    Code { bits: 0x1A, len: 13, run: 0, level: 12 },
    Code { bits: 0x19, len: 13, run: 0, level: 13 },
    Code { bits: 0x18, len: 13, run: 0, level: 14 },
    Code { bits: 0x17, len: 13, run: 0, level: 15 },
    Code { bits: 0x1F, len: 14, run: 0, level: 16 },
    Code { bits: 0x1E, len: 14, run: 0, level: 17 },
    Code { bits: 0x1D, len: 14, run: 0, level: 18 },
    Code { bits: 0x1C, len: 14, run: 0, level: 19 },
    Code { bits: 0x1B, len: 14, run: 0, level: 20 },
    Code { bits: 0x1A, len: 14, run: 0, level: 21 },
    Code { bits: 0x19, len: 14, run: 0, level: 22 },
    Code { bits: 0x18, len: 14, run: 0, level: 23 },
    Code { bits: 0x17, len: 14, run: 0, level: 24 },
    Code { bits: 0x16, len: 14, run: 0, level: 25 },
    Code { bits: 0x15, len: 14, run: 0, level: 26 },
    Code { bits: 0x14, len: 14, run: 0, level: 27 },
    Code { bits: 0x13, len: 14, run: 0, level: 28 },
    Code { bits: 0x12, len: 14, run: 0, level: 29 },
    Code { bits: 0x11, len: 14, run: 0, level: 30 },
    Code { bits: 0x10, len: 14, run: 0, level: 31 },
    Code { bits: 0x18, len: 15, run: 0, level: 32 },
    Code { bits: 0x17, len: 15, run: 0, level: 33 },
    Code { bits: 0x16, len: 15, run: 0, level: 34 },
    Code { bits: 0x15, len: 15, run: 0, level: 35 },
    Code { bits: 0x14, len: 15, run: 0, level: 36 },
    Code { bits: 0x13, len: 15, run: 0, level: 37 },
    Code { bits: 0x12, len: 15, run: 0, level: 38 },
    Code { bits: 0x11, len: 15, run: 0, level: 39 },
    Code { bits: 0x10, len: 15, run: 0, level: 40 },
    Code { bits: 0x3, len: 3, run: 1, level: 1 },
    Code { bits: 0x6, len: 6, run: 1, level: 2 },
    Code { bits: 0x25, len: 8, run: 1, level: 3 },
    Code { bits: 0xC, len: 10, run: 1, level: 4 },
    Code { bits: 0x1B, len: 12, run: 1, level: 5 },
    Code { bits: 0x16, len: 13, run: 1, level: 6 },
    Code { bits: 0x15, len: 13, run: 1, level: 7 },
    Code { bits: 0x1F, len: 15, run: 1, level: 8 },
    Code { bits: 0x1E, len: 15, run: 1, level: 9 },
    Code { bits: 0x1D, len: 15, run: 1, level: 10 },
    Code { bits: 0x1C, len: 15, run: 1, level: 11 },
    Code { bits: 0x1B, len: 15, run: 1, level: 12 },
    Code { bits: 0x1A, len: 15, run: 1, level: 13 },
    Code { bits: 0x19, len: 15, run: 1, level: 14 },
    Code { bits: 0x13, len: 16, run: 1, level: 15 },
    Code { bits: 0x12, len: 16, run: 1, level: 16 },
    Code { bits: 0x11, len: 16, run: 1, level: 17 },
    Code { bits: 0x10, len: 16, run: 1, level: 18 },
    Code { bits: 0x5, len: 4, run: 2, level: 1 },
    Code { bits: 0x4, len: 7, run: 2, level: 2 },
    Code { bits: 0xB, len: 10, run: 2, level: 3 },
    Code { bits: 0x14, len: 12, run: 2, level: 4 },
    Code { bits: 0x14, len: 13, run: 2, level: 5 },
    Code { bits: 0x7, len: 5, run: 3, level: 1 },
    Code { bits: 0x24, len: 8, run: 3, level: 2 },
    Code { bits: 0x1C, len: 12, run: 3, level: 3 },
    Code { bits: 0x13, len: 13, run: 3, level: 4 },
    Code { bits: 0x6, len: 5, run: 4, level: 1 },
    Code { bits: 0xF, len: 10, run: 4, level: 2 },
    Code { bits: 0x12, len: 12, run: 4, level: 3 },
    Code { bits: 0x7, len: 6, run: 5, level: 1 },
    Code { bits: 0x9, len: 10, run: 5, level: 2 },
    Code { bits: 0x12, len: 13, run: 5, level: 3 },
    Code { bits: 0x5, len: 6, run: 6, level: 1 },
    Code { bits: 0x1E, len: 12, run: 6, level: 2 },
    Code { bits: 0x14, len: 16, run: 6, level: 3 },
    Code { bits: 0x4, len: 6, run: 7, level: 1 },
    Code { bits: 0x15, len: 12, run: 7, level: 2 },
    Code { bits: 0x7, len: 7, run: 8, level: 1 },
    Code { bits: 0x11, len: 12, run: 8, level: 2 },
    Code { bits: 0x5, len: 7, run: 9, level: 1 },
    Code { bits: 0x11, len: 13, run: 9, level: 2 },
    Code { bits: 0x27, len: 8, run: 10, level: 1 },
    Code { bits: 0x10, len: 13, run: 10, level: 2 },
    Code { bits: 0x23, len: 8, run: 11, level: 1 },
    Code { bits: 0x1A, len: 16, run: 11, level: 2 },
    Code { bits: 0x22, len: 8, run: 12, level: 1 },
    Code { bits: 0x19, len: 16, run: 12, level: 2 },
    Code { bits: 0x20, len: 8, run: 13, level: 1 },
    Code { bits: 0x18, len: 16, run: 13, level: 2 },
    Code { bits: 0xE, len: 10, run: 14, level: 1 },
    Code { bits: 0x17, len: 16, run: 14, level: 2 },
    Code { bits: 0xD, len: 10, run: 15, level: 1 },
    Code { bits: 0x16, len: 16, run: 15, level: 2 },
    Code { bits: 0x8, len: 10, run: 16, level: 1 },
    Code { bits: 0x15, len: 16, run: 16, level: 2 },
    Code { bits: 0x1F, len: 12, run: 17, level: 1 },
    Code { bits: 0x1A, len: 12, run: 18, level: 1 },
    Code { bits: 0x19, len: 12, run: 19, level: 1 },
    Code { bits: 0x17, len: 12, run: 20, level: 1 },
    Code { bits: 0x16, len: 12, run: 21, level: 1 },
    Code { bits: 0x1F, len: 13, run: 22, level: 1 },
    Code { bits: 0x1E, len: 13, run: 23, level: 1 },
    Code { bits: 0x1D, len: 13, run: 24, level: 1 },
    Code { bits: 0x1C, len: 13, run: 25, level: 1 },
    Code { bits: 0x1B, len: 13, run: 26, level: 1 },
    Code { bits: 0x1F, len: 16, run: 27, level: 1 },
    Code { bits: 0x1E, len: 16, run: 28, level: 1 },
    Code { bits: 0x1D, len: 16, run: 29, level: 1 },
    Code { bits: 0x1C, len: 16, run: 30, level: 1 },
    Code { bits: 0x1B, len: 16, run: 31, level: 1 },
];
/// The escape code of TABLE_ZERO: a 6 bit run and a 12 bit level follow.
pub const TABLE_ZERO_ESCAPE: (u32, u8) = (0x1, 6);
/// End of block.
pub const TABLE_ZERO_EOB: (u32, u8) = (0x2, 2);

/// Table B.15: the alternative intra table, which an MPEG-2 picture may use for its intra blocks.
pub const TABLE_ONE: [Code; 111] = [
    Code { bits: 0x2, len: 2, run: 0, level: 1 },
    Code { bits: 0x6, len: 3, run: 0, level: 2 },
    Code { bits: 0x7, len: 4, run: 0, level: 3 },
    Code { bits: 0x1C, len: 5, run: 0, level: 4 },
    Code { bits: 0x1D, len: 5, run: 0, level: 5 },
    Code { bits: 0x5, len: 6, run: 0, level: 6 },
    Code { bits: 0x4, len: 6, run: 0, level: 7 },
    Code { bits: 0x7B, len: 7, run: 0, level: 8 },
    Code { bits: 0x7C, len: 7, run: 0, level: 9 },
    Code { bits: 0x23, len: 8, run: 0, level: 10 },
    Code { bits: 0x22, len: 8, run: 0, level: 11 },
    Code { bits: 0xFA, len: 8, run: 0, level: 12 },
    Code { bits: 0xFB, len: 8, run: 0, level: 13 },
    Code { bits: 0xFE, len: 8, run: 0, level: 14 },
    Code { bits: 0xFF, len: 8, run: 0, level: 15 },
    Code { bits: 0x1F, len: 14, run: 0, level: 16 },
    Code { bits: 0x1E, len: 14, run: 0, level: 17 },
    Code { bits: 0x1D, len: 14, run: 0, level: 18 },
    Code { bits: 0x1C, len: 14, run: 0, level: 19 },
    Code { bits: 0x1B, len: 14, run: 0, level: 20 },
    Code { bits: 0x1A, len: 14, run: 0, level: 21 },
    Code { bits: 0x19, len: 14, run: 0, level: 22 },
    Code { bits: 0x18, len: 14, run: 0, level: 23 },
    Code { bits: 0x17, len: 14, run: 0, level: 24 },
    Code { bits: 0x16, len: 14, run: 0, level: 25 },
    Code { bits: 0x15, len: 14, run: 0, level: 26 },
    Code { bits: 0x14, len: 14, run: 0, level: 27 },
    Code { bits: 0x13, len: 14, run: 0, level: 28 },
    Code { bits: 0x12, len: 14, run: 0, level: 29 },
    Code { bits: 0x11, len: 14, run: 0, level: 30 },
    Code { bits: 0x10, len: 14, run: 0, level: 31 },
    Code { bits: 0x18, len: 15, run: 0, level: 32 },
    Code { bits: 0x17, len: 15, run: 0, level: 33 },
    Code { bits: 0x16, len: 15, run: 0, level: 34 },
    Code { bits: 0x15, len: 15, run: 0, level: 35 },
    Code { bits: 0x14, len: 15, run: 0, level: 36 },
    Code { bits: 0x13, len: 15, run: 0, level: 37 },
    Code { bits: 0x12, len: 15, run: 0, level: 38 },
    Code { bits: 0x11, len: 15, run: 0, level: 39 },
    Code { bits: 0x10, len: 15, run: 0, level: 40 },
    Code { bits: 0x2, len: 3, run: 1, level: 1 },
    Code { bits: 0x6, len: 5, run: 1, level: 2 },
    Code { bits: 0x79, len: 7, run: 1, level: 3 },
    Code { bits: 0x27, len: 8, run: 1, level: 4 },
    Code { bits: 0x20, len: 8, run: 1, level: 5 },
    Code { bits: 0x16, len: 13, run: 1, level: 6 },
    Code { bits: 0x15, len: 13, run: 1, level: 7 },
    Code { bits: 0x1F, len: 15, run: 1, level: 8 },
    Code { bits: 0x1E, len: 15, run: 1, level: 9 },
    Code { bits: 0x1D, len: 15, run: 1, level: 10 },
    Code { bits: 0x1C, len: 15, run: 1, level: 11 },
    Code { bits: 0x1B, len: 15, run: 1, level: 12 },
    Code { bits: 0x1A, len: 15, run: 1, level: 13 },
    Code { bits: 0x19, len: 15, run: 1, level: 14 },
    Code { bits: 0x13, len: 16, run: 1, level: 15 },
    Code { bits: 0x12, len: 16, run: 1, level: 16 },
    Code { bits: 0x11, len: 16, run: 1, level: 17 },
    Code { bits: 0x10, len: 16, run: 1, level: 18 },
    Code { bits: 0x5, len: 5, run: 2, level: 1 },
    Code { bits: 0x7, len: 7, run: 2, level: 2 },
    Code { bits: 0xFC, len: 8, run: 2, level: 3 },
    Code { bits: 0xC, len: 10, run: 2, level: 4 },
    Code { bits: 0x14, len: 13, run: 2, level: 5 },
    Code { bits: 0x7, len: 5, run: 3, level: 1 },
    Code { bits: 0x26, len: 8, run: 3, level: 2 },
    Code { bits: 0x1C, len: 12, run: 3, level: 3 },
    Code { bits: 0x13, len: 13, run: 3, level: 4 },
    Code { bits: 0x6, len: 6, run: 4, level: 1 },
    Code { bits: 0xFD, len: 8, run: 4, level: 2 },
    Code { bits: 0x12, len: 12, run: 4, level: 3 },
    Code { bits: 0x7, len: 6, run: 5, level: 1 },
    Code { bits: 0x4, len: 9, run: 5, level: 2 },
    Code { bits: 0x12, len: 13, run: 5, level: 3 },
    Code { bits: 0x6, len: 7, run: 6, level: 1 },
    Code { bits: 0x1E, len: 12, run: 6, level: 2 },
    Code { bits: 0x14, len: 16, run: 6, level: 3 },
    Code { bits: 0x4, len: 7, run: 7, level: 1 },
    Code { bits: 0x15, len: 12, run: 7, level: 2 },
    Code { bits: 0x5, len: 7, run: 8, level: 1 },
    Code { bits: 0x11, len: 12, run: 8, level: 2 },
    Code { bits: 0x78, len: 7, run: 9, level: 1 },
    Code { bits: 0x11, len: 13, run: 9, level: 2 },
    Code { bits: 0x7A, len: 7, run: 10, level: 1 },
    Code { bits: 0x10, len: 13, run: 10, level: 2 },
    Code { bits: 0x21, len: 8, run: 11, level: 1 },
    Code { bits: 0x1A, len: 16, run: 11, level: 2 },
    Code { bits: 0x25, len: 8, run: 12, level: 1 },
    Code { bits: 0x19, len: 16, run: 12, level: 2 },
    Code { bits: 0x24, len: 8, run: 13, level: 1 },
    Code { bits: 0x18, len: 16, run: 13, level: 2 },
    Code { bits: 0x5, len: 9, run: 14, level: 1 },
    Code { bits: 0x17, len: 16, run: 14, level: 2 },
    Code { bits: 0x7, len: 9, run: 15, level: 1 },
    Code { bits: 0x16, len: 16, run: 15, level: 2 },
    Code { bits: 0xD, len: 10, run: 16, level: 1 },
    Code { bits: 0x15, len: 16, run: 16, level: 2 },
    Code { bits: 0x1F, len: 12, run: 17, level: 1 },
    Code { bits: 0x1A, len: 12, run: 18, level: 1 },
    Code { bits: 0x19, len: 12, run: 19, level: 1 },
    Code { bits: 0x17, len: 12, run: 20, level: 1 },
    Code { bits: 0x16, len: 12, run: 21, level: 1 },
    Code { bits: 0x1F, len: 13, run: 22, level: 1 },
    Code { bits: 0x1E, len: 13, run: 23, level: 1 },
    Code { bits: 0x1D, len: 13, run: 24, level: 1 },
    Code { bits: 0x1C, len: 13, run: 25, level: 1 },
    Code { bits: 0x1B, len: 13, run: 26, level: 1 },
    Code { bits: 0x1F, len: 16, run: 27, level: 1 },
    Code { bits: 0x1E, len: 16, run: 28, level: 1 },
    Code { bits: 0x1D, len: 16, run: 29, level: 1 },
    Code { bits: 0x1C, len: 16, run: 30, level: 1 },
    Code { bits: 0x1B, len: 16, run: 31, level: 1 },
];
/// The escape code of TABLE_ONE: a 6 bit run and a 12 bit level follow.
pub const TABLE_ONE_ESCAPE: (u32, u8) = (0x1, 6);
/// End of block.
pub const TABLE_ONE_EOB: (u32, u8) = (0x6, 4);
