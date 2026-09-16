//! Lockheed Martin LMS6 radiosondes, the 403 MHz model.
//!
//! The most heavily protected sonde here: the data is convolutionally coded
//! at rate a half, then carried in CCSDS Reed-Solomon codewords, and the
//! 223-byte frame inside that ends in a CRC of its own. The coding and the
//! block belong to the waveform, so this reads what comes out the far end:
//! one frame, its serial number, its time of week and its position.
//!
//! Frame layout, the reversed codeword order and the CRC are from zilog80's
//! `rs1729/RS`, `demod/mod/lms6Xmod.c`.

use crate::bits::crc16;
use crate::rs::ReedSolomon;

/// The bytes a frame starts with.
pub const SYNC: [u8; 4] = [0x24, 0x54, 0x00, 0x00];

/// The block's own sync, which the demodulator finds and this does not see.
pub const BLOCK_SYNC: [u8; 5] = [0x00, 0x58, 0xF3, 0x3F, 0xB8];

/// Bytes in a frame, the CRC included.
pub const FRAME: usize = 223;

/// Bytes the CRC covers.
const CHECKED: usize = 221;

/// Bytes in a Reed-Solomon codeword and in its message.
pub const CODEWORD: usize = 255;
pub const MESSAGE: usize = 223;

/// The CCSDS Reed-Solomon code, which is what an LMS6 block is.
///
/// Not the one the RS41 uses: this is the CCSDS parameterisation, GF(256)
/// over 0x187 with sixteen errors corrected from the 112th root and a
/// primitive step of eleven, and the codeword arrives with its symbols the
/// other way round.
pub fn code() -> ReedSolomon {
    ReedSolomon::new(8, 0x187, 112, 11, CODEWORD - MESSAGE, 0)
}

/// Correct a block in place, and say how many symbols were wrong.
///
/// `None` where the codeword is too broken to place, which with thirty-two
/// parity symbols means more than sixteen bytes of it were wrong.
pub fn correct(block: &mut [u8]) -> Option<usize> {
    if block.len() != CODEWORD {
        return None;
    }
    // The sonde sends the codeword highest symbol first, which is the other
    // way round from how the decoder indexes it.
    block.reverse();
    let errors = code().decode(block, &[]);
    block.reverse();
    errors
}

/// Whether a frame's own check holds.
pub fn check_ok(frame: &[u8]) -> bool {
    if frame.len() < FRAME {
        return false;
    }
    let sent = u16::from(frame[CHECKED]) << 8 | u16::from(frame[CHECKED + 1]);
    sent == crc16(&frame[..CHECKED], 0x1021, 0x0000)
}

/// What a frame says.
#[derive(Clone, Debug, PartialEq)]
pub struct Report {
    /// The serial printed on the sonde, which is the low three bytes of the
    /// word it sends.
    pub serial: u32,
    pub frame_no: u16,
    pub lat_deg: f64,
    pub lon_deg: f64,
    /// Height above the ellipsoid.
    pub altitude_m: f64,
    pub speed_kt: f64,
    pub course_deg: f64,
    pub climb_ms: f64,
    /// Seconds into the GPS week, and the milliseconds with it.
    pub gps_sec: u32,
    /// Hours, minutes and seconds of the day.
    pub utc: (u8, u8, f64),
}

impl Report {
    pub fn has_position(&self) -> bool {
        self.lat_deg != 0.0 || self.lon_deg != 0.0
    }

    pub fn summary(&self) -> String {
        format!(
            "LMS6 {} {:.5}, {:.5} at {:.0} m, {:+.1} m/s",
            self.serial, self.lat_deg, self.lon_deg, self.altitude_m, self.climb_ms
        )
    }
}

/// Read a frame.
///
/// `None` where it does not start with the frame sync or its CRC does not
/// hold. Both are checked because a Reed-Solomon codeword that failed to
/// correct still produces 223 bytes, and they are not a frame.
pub fn parse(frame: &[u8]) -> Option<Report> {
    if frame.len() < FRAME || frame[..4] != SYNC || !check_ok(frame) {
        return None;
    }
    let at = 4;
    let be32 = |i: usize| (0..4).fold(0u32, |v, k| v << 8 | u32::from(frame[at + i + k]));
    let be24 = |i: usize| {
        let v = (0..3).fold(0u32, |v, k| v << 8 | u32::from(frame[at + i + k]));
        match v > 0x7F_FFFF {
            true => v as i32 - 0x100_0000,
            false => v as i32,
        }
    };
    // Degrees as the receiver counts them: 2^30ths of ninety.
    const B60B60: f64 = (1u32 << 30) as f64 / 90.0;

    let tow_ms = be32(0x06);
    let sec = tow_ms / 1000;
    let in_day = sec % 86_400;
    let (east, north, up) =
        (f64::from(be24(0x1A)) / 1e3, f64::from(be24(0x1D)) / 1e3, f64::from(be24(0x20)) / 1e3);
    let r = Report {
        serial: be32(0x00) & 0xFF_FFFF,
        frame_no: (u16::from(frame[at + 4]) << 8) | u16::from(frame[at + 5]),
        lat_deg: f64::from(be32(0x0E) as i32) / B60B60,
        lon_deg: f64::from(be32(0x12) as i32) / B60B60,
        altitude_m: f64::from(be32(0x16) as i32) / 1000.0,
        speed_kt: east.hypot(north) * 1.943_844,
        course_deg: east.atan2(north).to_degrees().rem_euclid(360.0),
        climb_ms: up,
        gps_sec: sec,
        utc: (
            (in_day / 3600) as u8,
            (in_day % 3600 / 60) as u8,
            f64::from(in_day % 60) + f64::from(tow_ms % 1000) / 1000.0,
        ),
    };
    // Nothing a balloon does reaches these, and they are what a frame of
    // zeros that happened to check would come to.
    if r.altitude_m < -200.0 || r.altitude_m > 60_000.0 {
        return None;
    }
    Some(r)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A frame with its sync, fields and CRC in place.
    fn frame(fields: &[(usize, Vec<u8>)]) -> Vec<u8> {
        let mut f = vec![0u8; FRAME];
        f[..4].copy_from_slice(&SYNC);
        for (at, bytes) in fields {
            f[4 + at..4 + at + bytes.len()].copy_from_slice(bytes);
        }
        let cs = crc16(&f[..CHECKED], 0x1021, 0x0000).to_be_bytes();
        f[CHECKED..CHECKED + 2].copy_from_slice(&cs);
        f
    }

    fn a_flight() -> Vec<u8> {
        const B60B60: f64 = (1u32 << 30) as f64 / 90.0;
        let deg = |d: f64| ((d * B60B60) as i32).to_be_bytes().to_vec();
        let vel = |ms: f64| ((ms * 1000.0) as i32).to_be_bytes()[1..].to_vec();
        frame(&[
            (0x00, 0x00_A1_B2_C3u32.to_be_bytes().to_vec()),
            (0x04, 4_321u16.to_be_bytes().to_vec()),
            (0x06, 452_540_000u32.to_be_bytes().to_vec()),
            (0x0E, deg(53.35)),
            (0x12, deg(-5.0)),
            (0x16, 4_712_220i32.to_be_bytes().to_vec()),
            (0x1A, vel(9.0)),
            (0x1D, vel(0.0)),
            (0x20, vel(5.0)),
        ])
    }

    /// A frame over the Irish Sea, read back: the serial, the clock, the
    /// Trimble angle scale and the velocity in millimetres a second.
    #[test]
    fn a_frame_reads_as_a_fix() {
        let f = a_flight();
        let r = parse(&f).expect("a report");
        assert_eq!(r.serial, 0xA1_B2C3);
        assert_eq!(r.frame_no, 4_321);
        assert!((r.lat_deg - 53.35).abs() < 1e-6, "{}", r.lat_deg);
        assert!((r.lon_deg + 5.0).abs() < 1e-6, "{}", r.lon_deg);
        assert!((r.altitude_m - 4_712.22).abs() < 0.01, "{}", r.altitude_m);
        assert!((r.speed_kt - 17.49).abs() < 0.05, "{}", r.speed_kt);
        assert!((r.course_deg - 90.0).abs() < 0.01, "{}", r.course_deg);
        assert!((r.climb_ms - 5.0).abs() < 0.01, "{}", r.climb_ms);
        assert_eq!(r.utc, (5, 42, 20.0));
    }

    /// The Reed-Solomon code puts back up to sixteen wrong bytes anywhere in
    /// a block, and the frame's CRC refuses what it could not.
    #[test]
    fn the_block_code_repairs_sixteen_wrong_bytes() {
        let f = a_flight();
        let mut block = vec![0u8; CODEWORD];
        block[..FRAME].copy_from_slice(&f);
        // Parity for this message, computed the way the sonde does.
        let mut cw = block.clone();
        cw.reverse();
        let parity = code().encode(&cw[CODEWORD - MESSAGE..]);
        cw[..CODEWORD - MESSAGE].copy_from_slice(&parity);
        cw.reverse();
        let block = cw;

        let mut clean = block.clone();
        assert_eq!(correct(&mut clean), Some(0), "a clean block was corrected");
        assert_eq!(clean, block);

        let mut broken = block.clone();
        for at in 0..16 {
            broken[at * 7 + 3] ^= 0x5A;
        }
        assert_eq!(correct(&mut broken), Some(16));
        assert_eq!(broken, block);
        assert!(parse(&broken[..FRAME]).is_some());

        // Seventeen is past what the code can place, and the frame's own
        // check is then what refuses it.
        let mut hopeless = block.clone();
        for at in 0..17 {
            hopeless[at * 7 + 3] ^= 0x5A;
        }
        let repaired = correct(&mut hopeless);
        assert!(
            repaired.is_none() || !check_ok(&hopeless[..FRAME]),
            "seventeen wrong bytes were turned into a frame"
        );
    }

    /// Bytes that are not a frame are refused on the sync and the check.
    #[test]
    fn noise_is_not_a_frame() {
        assert_eq!(parse(&[0u8; FRAME]), None);
        let mut bad = a_flight();
        bad[40] ^= 0x01;
        assert_eq!(parse(&bad), None);
    }
}
