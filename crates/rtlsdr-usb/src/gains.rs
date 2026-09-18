//! What each tuner offers, which `rtl_tcp` also reports as numbers, so the
//! tables live beside the wire values in `common::rtl`.

/// Tuner identification codes, as librtlsdr numbers them and as `rtl_tcp`
/// puts in its greeting
pub mod code {
    pub const E4K: u8 = 1;
    pub const FC0012: u8 = 2;
    pub const FC0013: u8 = 3;
    pub const FC2580: u8 = 4;
    pub const R820T: u8 = 5;
    pub const R828D: u8 = 6;
}

/// Supported gains in tenths of a dB, one table per tuner
pub const E4K: &[i32] = &[-10, 15, 40, 65, 90, 115, 140, 165, 190, 215, 240, 290, 340, 420];
pub const FC0012: &[i32] = &[-99, -40, 71, 179, 192];
pub const FC0013: &[i32] = &[
    -99, -73, -65, -63, -60, -58, -54, 58, 61, 63, 65, 67, 68, 70, 71, 179, 181, 182, 184, 186,
    188, 191, 197,
];
pub const FC2580: &[i32] = &[];
pub const R82XX: &[i32] = &[
    0, 9, 14, 27, 37, 77, 87, 125, 144, 157, 166, 197, 207, 229, 254, 280, 297, 328, 338, 364, 372,
    386, 402, 421, 434, 439, 445, 480, 496,
];
pub const UNKNOWN: &[i32] = &[];

pub fn for_code(code: u8) -> &'static [i32] {
    match code {
        code::E4K => E4K,
        code::FC0012 => FC0012,
        code::FC0013 => FC0013,
        code::FC2580 => FC2580,
        code::R820T | code::R828D => R82XX,
        _ => UNKNOWN,
    }
}
