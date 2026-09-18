//! The FT8 and FT4 message: 77 bits, a CRC-14 and the (174,91) LDPC code.
//!
//! Bytes in, fields out. What a station sends is a fixed 77-bit payload
//! whose first three bits (last on the air) say how the rest is read: two
//! callsigns and a grid square or a signal report, a callsign that will not
//! fit the standard form, thirteen characters of free text, or telemetry.
//! The packing is the one WSJT-X publishes, and this module is a reading of
//! `ft8_lib`'s `message.c` against it.
//!
//! The check is a CRC-14 over the payload zero-extended to 82 bits, and the
//! 91 bits that makes are what the LDPC code protects. Both halves are here
//! because a caller with soft bits off the air needs the code, and a caller
//! with a payload needs the packing, and neither wants the other's DSP.

use crate::bits::{Ldpc, crc_bits};
use std::sync::OnceLock;

/// Bits a station composes.
pub const PAYLOAD_BITS: usize = 77;
/// Payload plus its CRC-14, which is what the code protects.
pub const MESSAGE_BITS: usize = 91;
/// Bits on the air, three to a tone for FT8 and two for FT4.
pub const CODE_BITS: usize = 174;

/// CRC-14, its normal representation with the top term dropped.
const CRC_POLY: u32 = 0x2757;

/// The payload is zero-extended to 82 bits before the check is taken, which
/// is 77 bits of message and five zeros; WSJT-X's own words.
const CRC_OVER_BITS: usize = 82;

/// The parity half of the systematic (174,91) generator, one packed row of
/// 91 message bits per parity bit, MSB first.
///
/// From WSJT-X by way of `ft8_lib`'s `kFTX_LDPC_generator`. The parity check
/// matrix is not carried beside it because a systematic code implies it:
/// check `i` is this row's message bits and parity bit `i`.
#[rustfmt::skip]
const GENERATOR: [[u8; 12]; 83] = [
    [0x83, 0x29, 0xce, 0x11, 0xbf, 0x31, 0xea, 0xf5, 0x09, 0xf2, 0x7f, 0xc0],
    [0x76, 0x1c, 0x26, 0x4e, 0x25, 0xc2, 0x59, 0x33, 0x54, 0x93, 0x13, 0x20],
    [0xdc, 0x26, 0x59, 0x02, 0xfb, 0x27, 0x7c, 0x64, 0x10, 0xa1, 0xbd, 0xc0],
    [0x1b, 0x3f, 0x41, 0x78, 0x58, 0xcd, 0x2d, 0xd3, 0x3e, 0xc7, 0xf6, 0x20],
    [0x09, 0xfd, 0xa4, 0xfe, 0xe0, 0x41, 0x95, 0xfd, 0x03, 0x47, 0x83, 0xa0],
    [0x07, 0x7c, 0xcc, 0xc1, 0x1b, 0x88, 0x73, 0xed, 0x5c, 0x3d, 0x48, 0xa0],
    [0x29, 0xb6, 0x2a, 0xfe, 0x3c, 0xa0, 0x36, 0xf4, 0xfe, 0x1a, 0x9d, 0xa0],
    [0x60, 0x54, 0xfa, 0xf5, 0xf3, 0x5d, 0x96, 0xd3, 0xb0, 0xc8, 0xc3, 0xe0],
    [0xe2, 0x07, 0x98, 0xe4, 0x31, 0x0e, 0xed, 0x27, 0x88, 0x4a, 0xe9, 0x00],
    [0x77, 0x5c, 0x9c, 0x08, 0xe8, 0x0e, 0x26, 0xdd, 0xae, 0x56, 0x31, 0x80],
    [0xb0, 0xb8, 0x11, 0x02, 0x8c, 0x2b, 0xf9, 0x97, 0x21, 0x34, 0x87, 0xc0],
    [0x18, 0xa0, 0xc9, 0x23, 0x1f, 0xc6, 0x0a, 0xdf, 0x5c, 0x5e, 0xa3, 0x20],
    [0x76, 0x47, 0x1e, 0x83, 0x02, 0xa0, 0x72, 0x1e, 0x01, 0xb1, 0x2b, 0x80],
    [0xff, 0xbc, 0xcb, 0x80, 0xca, 0x83, 0x41, 0xfa, 0xfb, 0x47, 0xb2, 0xe0],
    [0x66, 0xa7, 0x2a, 0x15, 0x8f, 0x93, 0x25, 0xa2, 0xbf, 0x67, 0x17, 0x00],
    [0xc4, 0x24, 0x36, 0x89, 0xfe, 0x85, 0xb1, 0xc5, 0x13, 0x63, 0xa1, 0x80],
    [0x0d, 0xff, 0x73, 0x94, 0x14, 0xd1, 0xa1, 0xb3, 0x4b, 0x1c, 0x27, 0x00],
    [0x15, 0xb4, 0x88, 0x30, 0x63, 0x6c, 0x8b, 0x99, 0x89, 0x49, 0x72, 0xe0],
    [0x29, 0xa8, 0x9c, 0x0d, 0x3d, 0xe8, 0x1d, 0x66, 0x54, 0x89, 0xb0, 0xe0],
    [0x4f, 0x12, 0x6f, 0x37, 0xfa, 0x51, 0xcb, 0xe6, 0x1b, 0xd6, 0xb9, 0x40],
    [0x99, 0xc4, 0x72, 0x39, 0xd0, 0xd9, 0x7d, 0x3c, 0x84, 0xe0, 0x94, 0x00],
    [0x19, 0x19, 0xb7, 0x51, 0x19, 0x76, 0x56, 0x21, 0xbb, 0x4f, 0x1e, 0x80],
    [0x09, 0xdb, 0x12, 0xd7, 0x31, 0xfa, 0xee, 0x0b, 0x86, 0xdf, 0x6b, 0x80],
    [0x48, 0x8f, 0xc3, 0x3d, 0xf4, 0x3f, 0xbd, 0xee, 0xa4, 0xea, 0xfb, 0x40],
    [0x82, 0x74, 0x23, 0xee, 0x40, 0xb6, 0x75, 0xf7, 0x56, 0xeb, 0x5f, 0xe0],
    [0xab, 0xe1, 0x97, 0xc4, 0x84, 0xcb, 0x74, 0x75, 0x71, 0x44, 0xa9, 0xa0],
    [0x2b, 0x50, 0x0e, 0x4b, 0xc0, 0xec, 0x5a, 0x6d, 0x2b, 0xdb, 0xdd, 0x00],
    [0xc4, 0x74, 0xaa, 0x53, 0xd7, 0x02, 0x18, 0x76, 0x16, 0x69, 0x36, 0x00],
    [0x8e, 0xba, 0x1a, 0x13, 0xdb, 0x33, 0x90, 0xbd, 0x67, 0x18, 0xce, 0xc0],
    [0x75, 0x38, 0x44, 0x67, 0x3a, 0x27, 0x78, 0x2c, 0xc4, 0x20, 0x12, 0xe0],
    [0x06, 0xff, 0x83, 0xa1, 0x45, 0xc3, 0x70, 0x35, 0xa5, 0xc1, 0x26, 0x80],
    [0x3b, 0x37, 0x41, 0x78, 0x58, 0xcc, 0x2d, 0xd3, 0x3e, 0xc3, 0xf6, 0x20],
    [0x9a, 0x4a, 0x5a, 0x28, 0xee, 0x17, 0xca, 0x9c, 0x32, 0x48, 0x42, 0xc0],
    [0xbc, 0x29, 0xf4, 0x65, 0x30, 0x9c, 0x97, 0x7e, 0x89, 0x61, 0x0a, 0x40],
    [0x26, 0x63, 0xae, 0x6d, 0xdf, 0x8b, 0x5c, 0xe2, 0xbb, 0x29, 0x48, 0x80],
    [0x46, 0xf2, 0x31, 0xef, 0xe4, 0x57, 0x03, 0x4c, 0x18, 0x14, 0x41, 0x80],
    [0x3f, 0xb2, 0xce, 0x85, 0xab, 0xe9, 0xb0, 0xc7, 0x2e, 0x06, 0xfb, 0xe0],
    [0xde, 0x87, 0x48, 0x1f, 0x28, 0x2c, 0x15, 0x39, 0x71, 0xa0, 0xa2, 0xe0],
    [0xfc, 0xd7, 0xcc, 0xf2, 0x3c, 0x69, 0xfa, 0x99, 0xbb, 0xa1, 0x41, 0x20],
    [0xf0, 0x26, 0x14, 0x47, 0xe9, 0x49, 0x0c, 0xa8, 0xe4, 0x74, 0xce, 0xc0],
    [0x44, 0x10, 0x11, 0x58, 0x18, 0x19, 0x6f, 0x95, 0xcd, 0xd7, 0x01, 0x20],
    [0x08, 0x8f, 0xc3, 0x1d, 0xf4, 0xbf, 0xbd, 0xe2, 0xa4, 0xea, 0xfb, 0x40],
    [0xb8, 0xfe, 0xf1, 0xb6, 0x30, 0x77, 0x29, 0xfb, 0x0a, 0x07, 0x8c, 0x00],
    [0x5a, 0xfe, 0xa7, 0xac, 0xcc, 0xb7, 0x7b, 0xbc, 0x9d, 0x99, 0xa9, 0x00],
    [0x49, 0xa7, 0x01, 0x6a, 0xc6, 0x53, 0xf6, 0x5e, 0xcd, 0xc9, 0x07, 0x60],
    [0x19, 0x44, 0xd0, 0x85, 0xbe, 0x4e, 0x7d, 0xa8, 0xd6, 0xcc, 0x7d, 0x00],
    [0x25, 0x1f, 0x62, 0xad, 0xc4, 0x03, 0x2f, 0x0e, 0xe7, 0x14, 0x00, 0x20],
    [0x56, 0x47, 0x1f, 0x87, 0x02, 0xa0, 0x72, 0x1e, 0x00, 0xb1, 0x2b, 0x80],
    [0x2b, 0x8e, 0x49, 0x23, 0xf2, 0xdd, 0x51, 0xe2, 0xd5, 0x37, 0xfa, 0x00],
    [0x6b, 0x55, 0x0a, 0x40, 0xa6, 0x6f, 0x47, 0x55, 0xde, 0x95, 0xc2, 0x60],
    [0xa1, 0x8a, 0xd2, 0x8d, 0x4e, 0x27, 0xfe, 0x92, 0xa4, 0xf6, 0xc8, 0x40],
    [0x10, 0xc2, 0xe5, 0x86, 0x38, 0x8c, 0xb8, 0x2a, 0x3d, 0x80, 0x75, 0x80],
    [0xef, 0x34, 0xa4, 0x18, 0x17, 0xee, 0x02, 0x13, 0x3d, 0xb2, 0xeb, 0x00],
    [0x7e, 0x9c, 0x0c, 0x54, 0x32, 0x5a, 0x9c, 0x15, 0x83, 0x6e, 0x00, 0x00],
    [0x36, 0x93, 0xe5, 0x72, 0xd1, 0xfd, 0xe4, 0xcd, 0xf0, 0x79, 0xe8, 0x60],
    [0xbf, 0xb2, 0xce, 0xc5, 0xab, 0xe1, 0xb0, 0xc7, 0x2e, 0x07, 0xfb, 0xe0],
    [0x7e, 0xe1, 0x82, 0x30, 0xc5, 0x83, 0xcc, 0xcc, 0x57, 0xd4, 0xb0, 0x80],
    [0xa0, 0x66, 0xcb, 0x2f, 0xed, 0xaf, 0xc9, 0xf5, 0x26, 0x64, 0x12, 0x60],
    [0xbb, 0x23, 0x72, 0x5a, 0xbc, 0x47, 0xcc, 0x5f, 0x4c, 0xc4, 0xcd, 0x20],
    [0xde, 0xd9, 0xdb, 0xa3, 0xbe, 0xe4, 0x0c, 0x59, 0xb5, 0x60, 0x9b, 0x40],
    [0xd9, 0xa7, 0x01, 0x6a, 0xc6, 0x53, 0xe6, 0xde, 0xcd, 0xc9, 0x03, 0x60],
    [0x9a, 0xd4, 0x6a, 0xed, 0x5f, 0x70, 0x7f, 0x28, 0x0a, 0xb5, 0xfc, 0x40],
    [0xe5, 0x92, 0x1c, 0x77, 0x82, 0x25, 0x87, 0x31, 0x6d, 0x7d, 0x3c, 0x20],
    [0x4f, 0x14, 0xda, 0x82, 0x42, 0xa8, 0xb8, 0x6d, 0xca, 0x73, 0x35, 0x20],
    [0x8b, 0x8b, 0x50, 0x7a, 0xd4, 0x67, 0xd4, 0x44, 0x1d, 0xf7, 0x70, 0xe0],
    [0x22, 0x83, 0x1c, 0x9c, 0xf1, 0x16, 0x94, 0x67, 0xad, 0x04, 0xb6, 0x80],
    [0x21, 0x3b, 0x83, 0x8f, 0xe2, 0xae, 0x54, 0xc3, 0x8e, 0xe7, 0x18, 0x00],
    [0x5d, 0x92, 0x6b, 0x6d, 0xd7, 0x1f, 0x08, 0x51, 0x81, 0xa4, 0xe1, 0x20],
    [0x66, 0xab, 0x79, 0xd4, 0xb2, 0x9e, 0xe6, 0xe6, 0x95, 0x09, 0xe5, 0x60],
    [0x95, 0x81, 0x48, 0x68, 0x2d, 0x74, 0x8a, 0x38, 0xdd, 0x68, 0xba, 0xa0],
    [0xb8, 0xce, 0x02, 0x0c, 0xf0, 0x69, 0xc3, 0x2a, 0x72, 0x3a, 0xb1, 0x40],
    [0xf4, 0x33, 0x1d, 0x6d, 0x46, 0x16, 0x07, 0xe9, 0x57, 0x52, 0x74, 0x60],
    [0x6d, 0xa2, 0x3b, 0xa4, 0x24, 0xb9, 0x59, 0x61, 0x33, 0xcf, 0x9c, 0x80],
    [0xa6, 0x36, 0xbc, 0xbc, 0x7b, 0x30, 0xc5, 0xfb, 0xea, 0xe6, 0x7f, 0xe0],
    [0x5c, 0xb0, 0xd8, 0x6a, 0x07, 0xdf, 0x65, 0x4a, 0x90, 0x89, 0xa2, 0x00],
    [0xf1, 0x1f, 0x10, 0x68, 0x48, 0x78, 0x0f, 0xc9, 0xec, 0xdd, 0x80, 0xa0],
    [0x1f, 0xbb, 0x53, 0x64, 0xfb, 0x8d, 0x2c, 0x9d, 0x73, 0x0d, 0x5b, 0xa0],
    [0xfc, 0xb8, 0x6b, 0xc7, 0x0a, 0x50, 0xc9, 0xd0, 0x2a, 0x5d, 0x03, 0x40],
    [0xa5, 0x34, 0x43, 0x30, 0x29, 0xea, 0xc1, 0x5f, 0x32, 0x2e, 0x34, 0xc0],
    [0xc9, 0x89, 0xd9, 0xc7, 0xc3, 0xd3, 0xb8, 0xc5, 0x5d, 0x75, 0x13, 0x00],
    [0x7b, 0xb3, 0x8b, 0x2f, 0x01, 0x86, 0xd4, 0x66, 0x43, 0xae, 0x96, 0x20],
    [0x26, 0x44, 0xeb, 0xad, 0xeb, 0x44, 0xb9, 0x46, 0x7d, 0x1f, 0x42, 0xc0],
    [0x60, 0x8c, 0xc8, 0x57, 0x59, 0x4b, 0xfb, 0xb5, 0x5d, 0x69, 0x60, 0x00],];

/// Each parity check, as the codeword bits that must exclusive-or to zero.
///
/// From WSJT-X by way of `ft8_lib`'s `kFTX_LDPC_Nm`, counted from zero here
/// rather than from one. This is the sparse graph the code was designed
/// around: every codeword bit sits in three of these checks, and a dense set
/// derived from the generator instead reads several dB worse.
#[rustfmt::skip]
const CHECKS: [&[u16]; 83] = [
    &[3, 30, 58, 90, 91, 95, 152],
    &[4, 31, 59, 92, 114, 145],
    &[5, 23, 60, 93, 121, 150],
    &[6, 32, 61, 94, 95, 142],
    &[7, 24, 62, 82, 92, 95, 147],
    &[5, 31, 63, 96, 125, 137],
    &[4, 33, 64, 77, 97, 106, 153],
    &[8, 34, 65, 98, 138, 145],
    &[9, 35, 66, 99, 106, 125],
    &[10, 36, 66, 86, 100, 138, 157],
    &[11, 37, 67, 101, 104, 154],
    &[12, 38, 68, 102, 148, 161],
    &[7, 39, 69, 81, 103, 113, 144],
    &[13, 40, 70, 87, 101, 122, 155],
    &[14, 41, 58, 105, 122, 158],
    &[0, 32, 71, 105, 106, 156],
    &[15, 42, 72, 107, 140, 159],
    &[16, 36, 73, 80, 108, 130, 153],
    &[10, 43, 74, 109, 120, 165],
    &[44, 54, 63, 110, 129, 160, 172],
    &[7, 45, 70, 111, 118, 165],
    &[17, 35, 75, 88, 112, 113, 142],
    &[18, 37, 76, 103, 115, 162],
    &[19, 46, 69, 91, 137, 164],
    &[1, 47, 73, 112, 127, 159],
    &[20, 44, 77, 82, 116, 120, 150],
    &[21, 46, 57, 117, 126, 163],
    &[15, 38, 61, 111, 133, 157],
    &[22, 42, 78, 119, 130, 144],
    &[18, 34, 58, 72, 109, 124, 160],
    &[19, 35, 62, 93, 135, 160],
    &[13, 30, 78, 97, 131, 163],
    &[2, 43, 79, 123, 126, 168],
    &[18, 45, 80, 116, 134, 166],
    &[6, 48, 57, 89, 99, 104, 167],
    &[11, 49, 60, 117, 118, 143],
    &[12, 50, 63, 113, 117, 156],
    &[23, 51, 75, 128, 147, 148],
    &[24, 52, 68, 89, 100, 129, 155],
    &[19, 45, 64, 79, 119, 139, 169],
    &[20, 53, 76, 99, 139, 170],
    &[34, 81, 132, 141, 170, 173],
    &[13, 29, 82, 112, 124, 169],
    &[3, 28, 67, 119, 133, 172],
    &[0, 3, 51, 56, 85, 135, 151],
    &[25, 50, 55, 90, 121, 136, 167],
    &[51, 83, 109, 114, 144, 167],
    &[6, 49, 80, 98, 131, 172],
    &[22, 54, 66, 94, 171, 173],
    &[25, 40, 76, 108, 140, 147],
    &[1, 26, 40, 60, 61, 114, 132],
    &[26, 39, 55, 123, 124, 125],
    &[17, 48, 54, 123, 140, 166],
    &[5, 32, 84, 107, 115, 155],
    &[27, 47, 69, 84, 104, 128, 157],
    &[8, 53, 62, 130, 146, 154],
    &[21, 52, 67, 108, 120, 173],
    &[2, 12, 47, 77, 94, 122],
    &[30, 68, 132, 149, 154, 168],
    &[11, 42, 65, 88, 96, 134, 158],
    &[4, 38, 74, 101, 135, 166],
    &[1, 53, 85, 100, 134, 163],
    &[14, 55, 86, 107, 118, 170],
    &[9, 43, 81, 90, 110, 143, 148],
    &[22, 33, 70, 93, 126, 152],
    &[10, 48, 87, 91, 141, 156],
    &[28, 33, 86, 96, 146, 161],
    &[29, 49, 59, 85, 136, 141, 161],
    &[9, 52, 65, 83, 111, 127, 164],
    &[21, 56, 84, 92, 139, 158],
    &[27, 31, 71, 102, 131, 165],
    &[27, 28, 83, 87, 116, 142, 149],
    &[0, 25, 44, 79, 127, 146],
    &[16, 26, 88, 102, 115, 152],
    &[50, 56, 97, 162, 164, 171],
    &[20, 36, 72, 137, 151, 168],
    &[15, 46, 75, 129, 136, 153],
    &[2, 23, 29, 71, 103, 138],
    &[8, 39, 89, 105, 133, 150],
    &[14, 57, 59, 73, 110, 149, 162],
    &[17, 41, 78, 143, 145, 151],
    &[24, 37, 64, 98, 121, 159],
    &[16, 41, 74, 128, 169, 171],
];

/// The code both modes are protected by.
pub fn code() -> &'static Ldpc {
    static CODE: OnceLock<Ldpc> = OnceLock::new();
    CODE.get_or_init(|| Ldpc::new(&CHECKS, CODE_BITS))
}

/// The check over a payload, as the transmitter computed it.
pub fn crc14(payload: &[bool]) -> u16 {
    let mut bits = [false; CRC_OVER_BITS];
    bits[..PAYLOAD_BITS].copy_from_slice(&payload[..PAYLOAD_BITS]);
    crc_bits(&bits, 14, CRC_POLY, 0) as u16
}

/// A payload with its check behind it: the 91 bits the code carries.
pub fn with_crc(payload: &[bool]) -> Vec<bool> {
    let crc = crc14(payload);
    let mut out = payload[..PAYLOAD_BITS].to_vec();
    out.extend((0..14).map(|k| crc >> (13 - k) & 1 != 0));
    out
}

/// Whether a 91-bit message carries the check its payload implies.
pub fn crc_ok(message: &[bool]) -> bool {
    if message.len() < MESSAGE_BITS {
        return false;
    }
    let sent =
        message[PAYLOAD_BITS..MESSAGE_BITS].iter().fold(0u16, |acc, &b| acc << 1 | u16::from(b));
    crc14(message) == sent
}

/// A payload as the 174 bits that go on the air: the 91 it protects, then
/// one parity bit per row of the generator.
pub fn encode(payload: &[bool]) -> Vec<bool> {
    let message = with_crc(payload);
    let mut word = message.clone();
    word.extend(GENERATOR.iter().map(|row| {
        (0..MESSAGE_BITS)
            .filter(|b| row[b / 8] >> (7 - b % 8) & 1 != 0)
            .fold(false, |acc, b| acc ^ message[b])
    }));
    word
}

/// The pseudorandom sequence FT4 exclusive-ors a payload with before the
/// check and the parity are taken, so a CQ full of zeros is not keyed as a
/// steady tone. MSB first, the last five bits unused.
const FT4_SCRAMBLE: [u8; 10] = [0x4A, 0x5E, 0x89, 0xB4, 0xB0, 0x8A, 0x79, 0x55, 0xBE, 0x28];

/// Apply that sequence, which is its own inverse.
pub fn scramble_ft4(payload: &mut [bool]) {
    for (i, b) in payload.iter_mut().enumerate().take(PAYLOAD_BITS) {
        *b ^= FT4_SCRAMBLE[i / 8] >> (7 - i % 8) & 1 != 0;
    }
}

/// How a payload's 77 bits are read, which its top three bits say.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Kind {
    /// Two callsigns and a grid square, a report or nothing: i3 of 1 or 2,
    /// and all but a few per cent of what a band carries.
    Standard,
    /// One callsign too long or too odd for the standard form, with the
    /// other end carried as a 12-bit hash: i3 of 4.
    Nonstandard,
    /// Thirteen characters somebody typed: i3 and n3 both zero.
    FreeText,
    /// 71 bits of whatever a beacon wanted to send: i3 zero, n3 five.
    Telemetry,
}

/// What a station said.
#[derive(Clone, Debug, PartialEq)]
pub struct Message {
    pub kind: Kind,
    /// The line WSJT-X would print, which is what a person reads.
    pub text: String,
    /// Who it was addressed to, where the form names one: a callsign, or a
    /// token like `CQ` or `QRZ`.
    pub to: Option<String>,
    /// Who sent it, where the form names one.
    pub from: Option<String>,
    /// The four-character grid square, where one was sent.
    pub grid: Option<String>,
    /// The signal report in dB, where one was sent rather than a grid.
    pub report: Option<i32>,
}

/// Tokens the first 28-bit field can hold instead of a callsign.
const TOKENS: u32 = 2_063_592;
/// Hashed callsigns, between the tokens and the standard callsigns.
const MAX22: u32 = 4_194_304;
/// Grid squares, above which the field is a report or a token.
const MAXGRID4: u16 = 32_400;

const ALNUM_SPACE: &str = " 0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ";
const ALNUM: &str = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ";
const NUMERIC: &str = "0123456789";
const LETTERS_SPACE: &str = " ABCDEFGHIJKLMNOPQRSTUVWXYZ";
const FULL: &str = " 0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ+-./?";
/// The 38 characters a nonstandard callsign is packed from.
const ALNUM_SPACE_SLASH: &str = " 0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ/";

fn charn(table: &str, n: u64) -> char {
    table.chars().nth(n as usize).unwrap_or('_')
}

fn nchar(table: &str, c: char) -> Option<u64> {
    table.chars().position(|t| t == c).map(|p| p as u64)
}

/// Read a run of bits as a number, the first bit most significant.
fn field(bits: &[bool], at: usize, len: usize) -> u64 {
    bits[at..at + len].iter().fold(0u64, |acc, &b| acc << 1 | u64::from(b))
}

fn put(bits: &mut [bool], at: usize, len: usize, value: u64) {
    for k in 0..len {
        bits[at + k] = value >> (len - 1 - k) & 1 != 0;
    }
}

/// What a payload says, or `None` where it is a message form nothing here
/// reads: the contest and DXpedition forms, which are a few per cent of a
/// band and each pack their fields differently.
pub fn unpack(payload: &[bool]) -> Option<Message> {
    if payload.len() < PAYLOAD_BITS {
        return None;
    }
    let i3 = field(payload, 74, 3) as u8;
    match i3 {
        0 => match field(payload, 71, 3) as u8 {
            // Free text of nothing is what the all-zero codeword unpacks
            // to, and belief propagation settles on that word whenever the
            // soft bits are too weak to pull it anywhere else. Its CRC-14 is
            // zero too, so the check does not catch it: nobody transmits an
            // empty message, and refusing it here is what keeps it off the
            // bus.
            0 => free_text(payload).filter(|m| !m.text.is_empty()),
            5 => Some(telemetry(payload)),
            _ => None,
        },
        1 | 2 => standard(payload, i3),
        4 => nonstandard(payload),
        _ => None,
    }
}

fn standard(payload: &[bool], i3: u8) -> Option<Message> {
    let (n28a, ipa) = (field(payload, 0, 28) as u32, payload[28]);
    let (n28b, ipb) = (field(payload, 29, 28) as u32, payload[57]);
    let ir = payload[58];
    let igrid4 = field(payload, 59, 15) as u16;
    let to = unpack28(n28a, ipa, i3)?;
    let from = unpack28(n28b, ipb, i3)?;
    let (extra, grid, report) = unpack_grid(igrid4, ir);
    let text = [to.clone(), from.clone(), extra]
        .iter()
        .filter(|s| !s.is_empty())
        .cloned()
        .collect::<Vec<_>>()
        .join(" ");
    Some(Message { kind: Kind::Standard, text, to: Some(to), from: Some(from), grid, report })
}

/// A 28-bit callsign field: a token, a hash nothing here can look up, or a
/// callsign in the standard six-character form.
fn unpack28(n28: u32, suffix: bool, i3: u8) -> Option<String> {
    if n28 < TOKENS {
        return match n28 {
            0 => Some("DE".into()),
            1 => Some("QRZ".into()),
            2 => Some("CQ".into()),
            3..=1002 => Some(format!("CQ {:03}", n28 - 3)),
            1003..=532_443 => {
                let mut n = n28 - 1003;
                let mut letters = ['\0'; 4];
                for slot in letters.iter_mut().rev() {
                    *slot = charn(LETTERS_SPACE, (n % 27) as u64);
                    n /= 27;
                }
                let word: String = letters.iter().collect();
                Some(format!("CQ {}", word.trim()))
            }
            _ => None,
        };
    }
    let n28 = n28 - TOKENS;
    if n28 < MAX22 {
        // A 22-bit hash of a callsign somebody sent in full earlier in the
        // exchange. Nothing here remembers the exchange, so it stays a hash.
        return Some("<...>".into());
    }
    let mut n = (n28 - MAX22) as u64;
    let mut call = ['\0'; 6];
    for (slot, table) in call.iter_mut().rev().zip([
        LETTERS_SPACE,
        LETTERS_SPACE,
        LETTERS_SPACE,
        NUMERIC,
        ALNUM,
        ALNUM_SPACE,
    ]) {
        let base = table.chars().count() as u64;
        *slot = charn(table, n % base);
        n /= base;
    }
    let call: String = call.iter().collect();
    let mut call = match call.as_bytes() {
        // Swaziland and Guinea are keyed short and printed in full.
        [b'3', b'D', b'0', rest @ ..] if rest[0] != b' ' => {
            format!("3DA0{}", call[3..].trim())
        }
        [b'Q', b, ..] if b.is_ascii_uppercase() => format!("3X{}", call[1..].trim()),
        _ => call.trim().to_string(),
    };
    if call.len() < 3 {
        return None;
    }
    if suffix {
        match i3 {
            1 => call.push_str("/R"),
            2 => call.push_str("/P"),
            _ => return None,
        }
    }
    Some(call)
}

/// The 15-bit field behind the two callsigns: a grid square, a report, or
/// one of the four tokens that end an exchange.
fn unpack_grid(igrid4: u16, ir: bool) -> (String, Option<String>, Option<i32>) {
    if igrid4 <= MAXGRID4 {
        let mut n = igrid4;
        let mut g = ['\0'; 4];
        g[3] = charn(NUMERIC, (n % 10) as u64);
        n /= 10;
        g[2] = charn(NUMERIC, (n % 10) as u64);
        n /= 10;
        g[1] = (b'A' + (n % 18) as u8) as char;
        n /= 18;
        g[0] = (b'A' + (n % 18) as u8) as char;
        let grid: String = g.iter().collect();
        let text = match ir {
            true => format!("R {grid}"),
            false => grid.clone(),
        };
        return (text, Some(grid), None);
    }
    match igrid4 - MAXGRID4 {
        1 => (String::new(), None, None),
        2 => ("RRR".into(), None, None),
        3 => ("RR73".into(), None, None),
        4 => ("73".into(), None, None),
        irpt => {
            let db = irpt as i32 - 35;
            let text = match ir {
                true => format!("R{db:+03}"),
                false => format!("{db:+03}"),
            };
            (text, None, Some(db))
        }
    }
}

/// A callsign the standard form cannot hold, packed as 58 bits of its own
/// with the other end of the exchange left as a 12-bit hash.
fn nonstandard(payload: &[bool]) -> Option<Message> {
    let n58 = field(payload, 12, 58);
    let flip = payload[70];
    let nrpt = field(payload, 71, 2);
    let cq = payload[73];
    let mut call = ['\0'; 11];
    let mut n = n58;
    for slot in call.iter_mut().rev() {
        *slot = charn(ALNUM_SPACE_SLASH, n % 38);
        n /= 38;
    }
    let call: String = call.iter().collect::<String>().trim().to_string();
    if call.len() < 3 {
        return None;
    }
    // The hashed end is a callsign this receiver was not told, so it is
    // printed the way WSJT-X prints one it cannot look up.
    let other = "<...>".to_string();
    let (to, from) = match (cq, flip) {
        (true, _) => ("CQ".to_string(), call),
        (false, true) => (other, call),
        (false, false) => (call, other),
    };
    let extra = match nrpt {
        1 => "RRR",
        2 => "RR73",
        3 => "73",
        _ => "",
    };
    let text = [to.as_str(), from.as_str(), extra]
        .iter()
        .filter(|s| !s.is_empty())
        .copied()
        .collect::<Vec<_>>()
        .join(" ");
    Some(Message {
        kind: Kind::Nonstandard,
        text,
        to: Some(to),
        from: Some(from),
        grid: None,
        report: None,
    })
}

/// The 71 bits under the type, as a big-endian number in nine bytes.
fn b71(payload: &[bool]) -> [u8; 9] {
    let mut out = [0u8; 9];
    for (i, slot) in out.iter_mut().enumerate() {
        *slot = (0..8).fold(0u8, |acc, k| acc << 1 | u8::from(payload[i * 8 + k]));
    }
    out
}

fn free_text(payload: &[bool]) -> Option<Message> {
    let mut n = b71(payload);
    let mut chars = ['\0'; 13];
    for slot in chars.iter_mut().rev() {
        let mut rem = 0u16;
        for byte in n.iter_mut() {
            rem = (rem << 8) | *byte as u16;
            *byte = (rem / 42) as u8;
            rem %= 42;
        }
        *slot = charn(FULL, rem as u64);
    }
    let text: String = chars.iter().collect::<String>().trim().to_string();
    Some(Message { kind: Kind::FreeText, text, to: None, from: None, grid: None, report: None })
}

fn telemetry(payload: &[bool]) -> Message {
    let n = b71(payload);
    let text = n.iter().map(|b| format!("{b:02X}")).collect::<String>();
    Message { kind: Kind::Telemetry, text, to: None, from: None, grid: None, report: None }
}

/// Pack a standard message: two callsigns or tokens and a grid, a report or
/// one of the closing tokens. `None` where a callsign will not fit the
/// standard form, which is the point at which a station switches to the
/// nonstandard form this does not key.
pub fn pack_standard(to: &str, from: &str, extra: &str) -> Option<[bool; PAYLOAD_BITS]> {
    let mut bits = [false; PAYLOAD_BITS];
    let (a, ipa) = pack28(to)?;
    let (b, ipb) = pack28(from)?;
    let (igrid4, ir) = pack_grid(extra)?;
    put(&mut bits, 0, 28, a as u64);
    bits[28] = ipa;
    put(&mut bits, 29, 28, b as u64);
    bits[57] = ipb;
    bits[58] = ir;
    put(&mut bits, 59, 15, igrid4 as u64);
    // i3 = 1, a standard message whose suffix, where there is one, is /R.
    put(&mut bits, 74, 3, 1);
    Some(bits)
}

fn pack28(call: &str) -> Option<(u32, bool)> {
    match call {
        "DE" => return Some((0, false)),
        "QRZ" => return Some((1, false)),
        "CQ" => return Some((2, false)),
        _ => {}
    }
    let (base, suffix) = match call.strip_suffix("/R") {
        Some(base) => (base, true),
        None => (call, false),
    };
    let mut c6 = [' '; 6];
    let chars: Vec<char> = base.chars().collect();
    // The digit in a callsign decides where it sits in the six characters.
    let at = match chars.get(2) {
        Some(c) if c.is_ascii_digit() && chars.len() <= 6 => 0,
        _ if chars.get(1).is_some_and(|c| c.is_ascii_digit()) && chars.len() <= 5 => 1,
        _ => return None,
    };
    for (k, c) in chars.iter().enumerate() {
        *c6.get_mut(at + k)? = *c;
    }
    let tables = [ALNUM_SPACE, ALNUM, NUMERIC, LETTERS_SPACE, LETTERS_SPACE, LETTERS_SPACE];
    let mut n = 0u64;
    for (c, table) in c6.iter().zip(tables) {
        n = n * table.chars().count() as u64 + nchar(table, *c)?;
    }
    Some((TOKENS + MAX22 + n as u32, suffix))
}

fn pack_grid(extra: &str) -> Option<(u16, bool)> {
    let extra = extra.trim();
    match extra {
        "" => return Some((MAXGRID4 + 1, false)),
        "RRR" => return Some((MAXGRID4 + 2, false)),
        "RR73" => return Some((MAXGRID4 + 3, false)),
        "73" => return Some((MAXGRID4 + 4, false)),
        _ => {}
    }
    let (body, ir) = match extra.strip_prefix("R ") {
        Some(rest) => (rest, true),
        None => match extra.strip_prefix('R') {
            Some(rest) if rest.starts_with(['+', '-']) => (rest, true),
            _ => (extra, false),
        },
    };
    let b = body.as_bytes();
    if b.len() == 4 && b[0].is_ascii_uppercase() && b[1].is_ascii_uppercase() {
        let mut n = (b[0] - b'A') as u16;
        n = n * 18 + (b[1] - b'A') as u16;
        n = n * 10 + (b[2] as char).to_digit(10)? as u16;
        n = n * 10 + (b[3] as char).to_digit(10)? as u16;
        return match n <= MAXGRID4 {
            true => Some((n, ir)),
            false => None,
        };
    }
    let db: i32 = body.parse().ok()?;
    let irpt = 35 + db;
    match (0..=MAXGRID4 as i32).contains(&irpt) {
        true => Some((MAXGRID4 + irpt as u16, ir)),
        false => None,
    }
}

/// Pack thirteen characters of free text.
pub fn pack_free_text(text: &str) -> Option<[bool; PAYLOAD_BITS]> {
    if text.chars().count() > 13 {
        return None;
    }
    let mut n = [0u8; 9];
    for k in 0..13 {
        let c = text.chars().nth(k).unwrap_or(' ');
        let mut rem = nchar(FULL, c.to_ascii_uppercase())? as u16;
        for byte in n.iter_mut().rev() {
            rem += *byte as u16 * 42;
            *byte = rem as u8;
            rem >>= 8;
        }
    }
    let mut bits = [false; PAYLOAD_BITS];
    for (i, byte) in n.iter().enumerate() {
        put(&mut bits, i * 8, 8, *byte as u64);
    }
    Some(bits)
}

/// The middle of a four-character Maidenhead square, in degrees.
///
/// A square is 2 degrees of longitude by 1 of latitude, and a station
/// reporting one is reporting the square rather than a point, so the middle
/// is the honest reading of it.
pub fn grid_position(grid: &str) -> Option<(f64, f64)> {
    let b = grid.as_bytes();
    if b.len() != 4 {
        return None;
    }
    let field_lon = (b[0].to_ascii_uppercase().checked_sub(b'A')?) as f64;
    let field_lat = (b[1].to_ascii_uppercase().checked_sub(b'A')?) as f64;
    if field_lon > 17.0 || field_lat > 17.0 {
        return None;
    }
    let square_lon = (b[2] as char).to_digit(10)? as f64;
    let square_lat = (b[3] as char).to_digit(10)? as f64;
    let lon = -180.0 + field_lon * 20.0 + square_lon * 2.0 + 1.0;
    let lat = -90.0 + field_lat * 10.0 + square_lat * 1.0 + 0.5;
    Some((lat, lon))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The payload a station sends for `CQ MI0ABC IO74`, packed here and
    /// read back: the round trip through both tables and the field layout.
    #[test]
    fn a_cq_packs_and_unpacks() {
        let payload = pack_standard("CQ", "MI0ABC", "IO74").expect("a standard message");
        let m = unpack(&payload).expect("a message");
        assert_eq!(m.kind, Kind::Standard);
        assert_eq!(m.text, "CQ MI0ABC IO74");
        assert_eq!(m.to.as_deref(), Some("CQ"));
        assert_eq!(m.from.as_deref(), Some("MI0ABC"));
        assert_eq!(m.grid.as_deref(), Some("IO74"));
        assert_eq!(m.report, None);
    }

    /// The other three shapes of a standard exchange: a report, a report
    /// acknowledged, and the token that ends it.
    #[test]
    fn a_whole_exchange_round_trips() {
        for (to, from, extra, report) in [
            ("G4ABC", "MI0ABC", "-12", Some(-12)),
            ("G4ABC", "MI0ABC", "R-08", Some(-8)),
            ("G4ABC", "MI0ABC", "+03", Some(3)),
            ("G4ABC", "MI0ABC", "RR73", None),
            ("G4ABC", "MI0ABC", "73", None),
        ] {
            let payload = pack_standard(to, from, extra).expect(extra);
            let m = unpack(&payload).expect(extra);
            assert_eq!(m.text, format!("{to} {from} {extra}"));
            assert_eq!(m.report, report, "{extra}");
        }
    }

    /// Thirteen characters of free text, which is the other form a person
    /// composes rather than a machine.
    #[test]
    fn free_text_round_trips() {
        let payload = pack_free_text("HELLO WORLD").expect("thirteen characters");
        let m = unpack(&payload).expect("a message");
        assert_eq!(m.kind, Kind::FreeText);
        assert_eq!(m.text, "HELLO WORLD");
        assert_eq!(m.to, None, "nobody is addressed by free text");
        assert!(pack_free_text("FOURTEEN CHARS").is_none());
    }

    /// The check is over the payload zero-extended to 82 bits, and one wrong
    /// bit anywhere in the 91 fails it.
    #[test]
    fn the_crc_covers_the_whole_payload() {
        let payload = pack_standard("CQ", "MI0ABC", "IO74").unwrap();
        let message = with_crc(&payload);
        assert_eq!(message.len(), MESSAGE_BITS);
        assert!(crc_ok(&message));
        for k in [0, 37, 76, 77, 90] {
            let mut wrong = message.clone();
            wrong[k] = !wrong[k];
            assert!(!crc_ok(&wrong), "bit {k} slipped past the check");
        }
    }

    /// The code is systematic, so its 174 bits open with the 91 they
    /// protect, and the checks it derives from the generator all pass.
    #[test]
    fn a_codeword_carries_its_message_and_satisfies_every_check() {
        let payload = pack_standard("CQ", "MI0ABC", "IO74").unwrap();
        let word = encode(&payload);
        assert_eq!(word.len(), CODE_BITS);
        assert_eq!(&word[..MESSAGE_BITS], &with_crc(&payload)[..]);
        assert_eq!(code().unsatisfied(&word), 0);
        let mut wrong = word.clone();
        wrong[100] = !wrong[100];
        assert_eq!(code().unsatisfied(&wrong), 3, "each bit sits in three checks");
        assert!((0..CODE_BITS).all(|b| code().degree_of(b) == 3));
    }

    /// What the code is for: wrong bits read back off soft values.
    ///
    /// Two ways, because they measure different things. Hard decisions of
    /// equal confidence are the worst case for the min-sum rule, and ten
    /// wrong bits in 174 is where that stops reading; with real soft values
    /// off a demodulator the code takes far more, and the noisy half of this
    /// reads 49 of 50 words back at a raw error rate of 7.5 per cent.
    #[test]
    fn the_code_repairs_a_word_off_soft_bits() {
        let payload = pack_standard("CQ", "MI0ABC", "IO74").unwrap();
        let word = encode(&payload);
        for wrong_bits in [0usize, 5, 10] {
            let llr: Vec<f32> = word
                .iter()
                .enumerate()
                .map(|(i, &b)| {
                    let flip = i * 7 % CODE_BITS < wrong_bits;
                    let one = b != flip;
                    if one { 4.0 } else { -4.0 }
                })
                .collect();
            let (read, failed) = code().decode(&llr, 30);
            assert_eq!(failed, 0, "{wrong_bits} wrong bits left the word unread");
            assert_eq!(read[..MESSAGE_BITS], word[..MESSAGE_BITS], "{wrong_bits} wrong bits");
            assert!(crc_ok(&read));
        }
        // Eleven, and the rule stalls: it settles on something that fails
        // ten of the 83 checks rather than on the word that was sent.
        let llr: Vec<f32> = word
            .iter()
            .enumerate()
            .map(|(i, &b)| {
                let one = b != (i * 7 % CODE_BITS < 11);
                if one { 4.0 } else { -4.0 }
            })
            .collect();
        assert_eq!(code().decode(&llr, 30).1, 10);

        // The same code read off a noisy channel, which is what a
        // demodulator hands it: 50 words at each noise level, counted.
        let mut seed = 0x1234_5678_9abc_def0u64;
        let mut normal = move || {
            let mut next = || {
                seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
                (seed >> 11) as f64 / (1u64 << 53) as f64
            };
            let (u, v) = (next(), next());
            ((-2.0 * (u + 1e-12).ln()).sqrt() * (std::f64::consts::TAU * v).cos()) as f32
        };
        for (sigma, want) in [(0.7f32, 49), (0.8, 36), (1.0, 1)] {
            let read = (0..50)
                .filter(|_| {
                    let llr: Vec<f32> = word
                        .iter()
                        .map(|&b| {
                            let x = if b { 1.0 } else { -1.0 } + sigma * normal();
                            2.0 * x / (sigma * sigma)
                        })
                        .collect();
                    let (read, failed) = code().decode(&llr, 30);
                    failed == 0 && crc_ok(&read) && read[..MESSAGE_BITS] == word[..MESSAGE_BITS]
                })
                .count();
            assert_eq!(read, want, "words read back at sigma {sigma}");
        }
    }

    /// Soft bits off noise are not a message: the code converges on
    /// something in a handful of cases and the CRC-14 is what refuses it.
    /// Measured over a thousand random words: none of them satisfied every
    /// check, so none of them reached the CRC-14 behind it.
    #[test]
    fn noise_is_not_a_codeword() {
        let mut seed = 0x243f_6a88_85a3_08d3u64;
        let mut rng = move || {
            seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
            (seed >> 40) as f32 / 8_388_608.0 - 1.0
        };
        let (mut coded, mut passed) = (0, 0);
        for _ in 0..1_000 {
            let llr: Vec<f32> = (0..CODE_BITS).map(|_| 4.0 * rng()).collect();
            let (read, failed) = code().decode(&llr, 30);
            if failed == 0 {
                coded += 1;
                passed += usize::from(crc_ok(&read));
            }
        }
        assert_eq!(coded, 0, "noise that settled on a codeword");
        assert_eq!(passed, 0, "noise read as a message");
    }

    /// A grid square is a square, and what is plotted is the middle of it.
    #[test]
    fn a_grid_square_is_a_place() {
        let (lat, lon) = grid_position("IO74").expect("a square");
        assert!((lat - 54.5).abs() < 1e-9, "{lat}");
        assert!((lon - -5.0).abs() < 1e-9, "{lon}");
        let (lat, lon) = grid_position("FN20").unwrap();
        assert!((lat - 40.5).abs() < 1e-9, "{lat}");
        assert!((lon - -75.0).abs() < 1e-9, "{lon}");
        assert_eq!(grid_position("IO7"), None);
        assert_eq!(grid_position("ZZ99"), None);
    }

    /// FT4 keys the payload through a fixed sequence so a message of mostly
    /// zeros is not keyed as a steady tone, and the sequence undoes itself.
    #[test]
    fn the_ft4_sequence_is_its_own_inverse() {
        let payload = pack_standard("CQ", "MI0ABC", "IO74").unwrap();
        let mut scrambled = payload;
        scramble_ft4(&mut scrambled);
        assert_ne!(scrambled, payload);
        scramble_ft4(&mut scrambled);
        assert_eq!(scrambled, payload);
    }
}
