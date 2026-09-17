//! Itron ERT meters on 902 to 928 MHz: SCM, SCM+ and IDM.
//!
//! An Encoder Receiver Transmitter is the radio bolted to an electricity, gas
//! or water meter across North America, and it broadcasts the register in the
//! clear every few seconds while hopping the band. Three message types are
//! common enough to be worth reading, and all three are Manchester keyed
//! on-off at 32768 chips per second, which is one 30 us half symbol:
//!
//! - **SCM**, 96 bits: the meter id, the commodity, the tamper flags and the
//!   cumulative reading, behind a 21 bit frame sync and a BCH check.
//! - **SCM+**, 16 bytes: the same reading with a longer id and a CRC-16.
//! - **IDM**, 90 bytes: the last reading plus the 47 five-minute differences
//!   before it, which is what makes a load curve rather than a total.
//!
//! The frame layouts are rtl_433's (`ert_scm.c`, `scmplus.c`, `ert_idm.c`) and
//! rtlamr's, which agree. The three checks are three different parameter sets
//! over the same CRC-16, so what separates the messages is the sync word and
//! the length, and the check is what confirms it.
//!
//! What reports is the meter: its id, what it measures and what it has counted.
//! A meter talking about itself is a packet with fields, so nothing here is
//! written by a person.

use crate::bits::{BitBuffer, crc16};
use crate::protocol::{DecodeError, Protocol, Report};
use crate::slicer::{Coding, Timing};

/// What the meter measures, from the low nibble of the ERT type. The mapping
/// is rtlamr's compatible-meters list, which rtl_433 also carries.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Commodity {
    Electric,
    Gas,
    Water,
    Unknown,
}

impl Commodity {
    fn of(ert_type: u8) -> Self {
        match ert_type & 0x0f {
            4 | 5 | 7 | 8 => Self::Electric,
            0 | 1 | 2 | 9 | 12 => Self::Gas,
            3 | 11 | 13 => Self::Water,
            _ => Self::Unknown,
        }
    }

    fn name(self) -> &'static str {
        match self {
            Self::Electric => "Electric",
            Self::Gas => "Gas",
            Self::Water => "Water",
            Self::Unknown => "unknown",
        }
    }
}

/// The one PHY all three messages share. `reset_us` is the gap that ends a
/// row, and differs only because an IDM is long enough to hold gaps an SCM
/// never would.
fn keying(reset_us: u32) -> Timing {
    Timing {
        coding: Coding::Manchester,
        short_us: 30,
        long_us: 30,
        sync_us: 0,
        tolerance_us: 0,
        reset_us,
    }
}

/// Offsets where `sync` appears, most significant bit first.
fn syncs(bits: &BitBuffer, sync: u32, sync_bits: usize, frame_bits: usize) -> Vec<usize> {
    if bits.len() < frame_bits {
        return Vec::new();
    }
    (0..=bits.len() - frame_bits).filter(|at| bits.extract(*at, sync_bits) == Some(sync)).collect()
}

fn bytes_at(bits: &BitBuffer, at: usize, n: usize) -> Vec<u8> {
    (0..n).filter_map(|i| bits.extract(at + i * 8, 8).map(|v| v as u8)).collect()
}

fn be16(b: &[u8], at: usize) -> u16 {
    u16::from_be_bytes([b[at], b[at + 1]])
}

fn be32(b: &[u8], at: usize) -> u32 {
    u32::from_be_bytes([b[at], b[at + 1], b[at + 2], b[at + 3]])
}

fn hex(b: &[u8]) -> String {
    b.iter().map(|x| format!("{x:02X}")).collect()
}

/// Standard Consumption Message: 96 bits, the oldest and by far the commonest.
pub struct ErtScm;

/// `1 1111 0010 1010 0110 0000`, the 21 bit frame sync, sync bit included.
const SCM_SYNC: u32 = 0x1f_2a60;
const SCM_SYNC_BITS: usize = 21;
const SCM_BYTES: usize = 12;
/// How much of the sync has to be found, counted from its end.
///
/// Not all of it, because the sync is the first thing on the air and a burst
/// detector opens partway through it: on both of rtl_433's SCM recordings the
/// first six chips are gone and the frame is otherwise exact. Twelve bits of
/// sync and the sixteen bit check together are what says this was a frame.
const SCM_SYNC_TAIL: usize = 12;

impl Protocol for ErtScm {
    fn name(&self) -> &'static str {
        "ERT-SCM"
    }

    fn timing(&self) -> Timing {
        keying(64)
    }

    fn decode(&self, bits: &BitBuffer) -> Result<Report, DecodeError> {
        let mut checked = false;
        let skip = SCM_SYNC_BITS - SCM_SYNC_TAIL;
        let tail = SCM_SYNC & ((1 << SCM_SYNC_TAIL) - 1);
        if bits.len() < SCM_BYTES * 8 {
            return Err(DecodeError::NotThisProtocol);
        }
        for at in 0..=bits.len() - SCM_BYTES * 8 {
            if bits.extract(at + skip, SCM_SYNC_TAIL) != Some(tail) {
                continue;
            }
            let f = bytes_at(bits, at, SCM_BYTES);
            if f.len() < SCM_BYTES {
                continue;
            }
            checked = true;
            // The BCH check is a CRC-16 with polynomial 0x6F63 over everything
            // from the tail of the sync on, the check bytes included, so a
            // good frame leaves nothing.
            if crc16(&f[2..12], 0x6f63, 0) != 0 {
                continue;
            }
            let ert_type = (f[3] >> 2) & 0x0f;
            // The two most significant bits of the id ride in the byte the
            // sync ends in; the other 24 are at the back of the frame.
            let id =
                (u32::from(f[2] & 0x06) << 23) | (u32::from(f[7]) << 16) | u32::from(be16(&f, 8));
            let mut r = Report::new(self.name());
            r.crc_valid = Some(true);
            r.raw = f.clone();
            return Ok(r
                .int("id", i64::from(id))
                .int("ert_type", i64::from(ert_type))
                .text("commodity", Commodity::of(ert_type).name())
                .int("consumption", i64::from(u32::from(f[4]) << 16 | u32::from(be16(&f, 5))))
                .int("physical_tamper", i64::from(f[3] >> 6))
                .int("encoder_tamper", i64::from(f[3] & 0x03)));
        }
        Err(if checked { DecodeError::CrcFailed } else { DecodeError::NotThisProtocol })
    }
}

/// Standard Consumption Message Plus: the same reading, a full 32 bit id.
pub struct ErtScmPlus;

const SCMP_SYNC: u32 = 0x16_a31e;
const ERT_SYNC_BITS: usize = 24;
const SCMP_BYTES: usize = 16;

impl Protocol for ErtScmPlus {
    fn name(&self) -> &'static str {
        "ERT-SCM+"
    }

    fn timing(&self) -> Timing {
        keying(64)
    }

    fn decode(&self, bits: &BitBuffer) -> Result<Report, DecodeError> {
        let mut checked = false;
        for at in syncs(bits, SCMP_SYNC, ERT_SYNC_BITS, SCMP_BYTES * 8) {
            let f = bytes_at(bits, at, SCMP_BYTES);
            if f.len() < SCMP_BYTES {
                continue;
            }
            checked = true;
            if crc16(&f[2..14], 0x1021, 0x0971) != be16(&f, 14) {
                continue;
            }
            let ert_type = f[3];
            let mut r = Report::new(self.name());
            r.crc_valid = Some(true);
            r.raw = f.clone();
            return Ok(r
                .int("id", i64::from(be32(&f, 4)))
                .int("ert_type", i64::from(ert_type))
                .text("commodity", Commodity::of(ert_type).name())
                .int("consumption", i64::from(be32(&f, 8)))
                .text("tamper", hex(&f[12..14])));
        }
        Err(if checked { DecodeError::CrcFailed } else { DecodeError::NotThisProtocol })
    }
}

/// Interval Data Message: the reading and the 47 intervals behind it.
pub struct ErtIdm;

const IDM_SYNC: u32 = 0x16_a31c;
/// 90 bytes on the air. The length byte inside says 92, which counts the two
/// preamble bytes a receiver has already consumed.
const IDM_BYTES: usize = 90;
/// Five minutes apart, so the 47 of them are the four hours before the
/// reading.
const IDM_INTERVALS: usize = 47;

impl Protocol for ErtIdm {
    fn name(&self) -> &'static str {
        "ERT-IDM"
    }

    fn timing(&self) -> Timing {
        keying(20_000)
    }

    fn decode(&self, bits: &BitBuffer) -> Result<Report, DecodeError> {
        let mut checked = false;
        for at in syncs(bits, IDM_SYNC, ERT_SYNC_BITS, IDM_BYTES * 8) {
            let f = bytes_at(bits, at, IDM_BYTES);
            if f.len() < IDM_BYTES {
                continue;
            }
            checked = true;
            if crc16(&f[2..88], 0x1021, 0xd895) != be16(&f, 88) {
                continue;
            }
            let ert_type = f[6];
            let mut r = Report::new(self.name());
            r.crc_valid = Some(true);
            r.raw = f.clone();
            return Ok(r
                .int("id", i64::from(be32(&f, 7)))
                .int("ert_type", i64::from(ert_type))
                .text("commodity", Commodity::of(ert_type).name())
                .int("consumption", i64::from(be32(&f, 27)))
                .int("interval_count", i64::from(f[11]))
                .int("app_version", i64::from(f[5]))
                .text("packet_type", format!("{:02X}", f[2]))
                .text("tamper", hex(&f[13..19]))
                .text("outages", hex(&f[21..27]))
                .int("transmit_offset", i64::from(be16(&f, 84)))
                .text("intervals", intervals(&f[31..])));
        }
        Err(if checked { DecodeError::CrcFailed } else { DecodeError::NotThisProtocol })
    }
}

/// The 47 differential intervals, nine bits apiece and not byte aligned.
fn intervals(body: &[u8]) -> String {
    let bit = |i: usize| usize::from(body[i / 8] >> (7 - i % 8) & 1);
    let mut out = String::new();
    for j in 0..IDM_INTERVALS {
        let v = (0..9).fold(0usize, |a, k| a << 1 | bit(j * 9 + k));
        if j > 0 {
            out.push(',');
        }
        out.push_str(&v.to_string());
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protocol::Value;

    fn bits_of(hex: &str) -> BitBuffer {
        let mut b = BitBuffer::new();
        // Leading noise, so nothing here decodes only because the frame starts
        // at bit zero.
        for i in 0..37 {
            b.push(i % 3 == 0);
        }
        for byte in (0..hex.len()).step_by(2).map(|i| u8::from_str_radix(&hex[i..i + 2], 16)) {
            let byte = byte.unwrap();
            for i in 0..8 {
                b.push(byte & (0x80 >> i) != 0);
            }
        }
        b
    }

    /// A frame off the air, from rtl_433's corpus at `tests/ert/scm/01`
    /// (`g001_912.6M_2400k.cu8`). rtl_433 25.02 reads it as id 54585868,
    /// ert_type 12, physical_tamper 3, consumption 562456, and rtlamr's
    /// `values.txt` beside the capture reports the same with CRC 0x101A,
    /// which is the last two bytes here.
    const SCM: &str = "f95306f008951840ea0c101a";

    /// The second capture in the same directory, `g002`: id 56355785,
    /// consumption 727018, physical tamper 2, CRC 0xDBFC.
    const SCM_B: &str = "f95306b00b17ea5bebc9dbfc";

    /// rtl_433's corpus at `tests/scmplus/01/g002_912.6M_2359.3k.cu8`, as
    /// rtl_433 25.02 aligns it: id 68211547, endpoint type 0xAB (water),
    /// consumption 6883, tamper 0x4900, CRC 0x39BE.
    const SCM_PLUS: &str = "16a31eab0410d35b00001ae3490039be";

    /// rtl_433's corpus at `tests/idm/IDM/g002_912.6M_2359.3k.cu8`. rtl_433
    /// 25.02 reads serial 11278109, ERT type 0x17, application version 4,
    /// interval count 246, last consumption 339972, transmit offset 476 and
    /// CRC 0x7C37, with three of the 47 differential intervals equal to 1.
    const IDM: &str = "16a31c5cc6041700ac171df6bc020100ef0900000000000000000000053004000000000000000000000000000000000000000000000000000000008000000000000000000000200000000000000000000008000001dceaba7c37";

    #[test]
    fn an_scm_frame_reads_its_meter_and_reading() {
        let r = ErtScm.decode(&bits_of(SCM)).expect("a frame");
        assert_eq!(r.get("id"), Some(&Value::Int(54585868)));
        assert_eq!(r.get("ert_type"), Some(&Value::Int(12)));
        assert_eq!(r.get("commodity"), Some(&Value::Text("Gas".into())));
        assert_eq!(r.get("consumption"), Some(&Value::Int(562456)));
        assert_eq!(r.get("physical_tamper"), Some(&Value::Int(3)));
        assert_eq!(r.get("encoder_tamper"), Some(&Value::Int(0)));
        assert_eq!(r.crc_valid, Some(true));
        assert_eq!(r.device, Some("54585868".into()));
    }

    #[test]
    fn a_second_scm_frame_reads_a_different_meter() {
        let r = ErtScm.decode(&bits_of(SCM_B)).expect("a frame");
        assert_eq!(r.get("id"), Some(&Value::Int(56355785)));
        assert_eq!(r.get("consumption"), Some(&Value::Int(727018)));
        assert_eq!(r.get("physical_tamper"), Some(&Value::Int(2)));
    }

    #[test]
    fn an_scm_plus_frame_reads_a_water_meter() {
        let r = ErtScmPlus.decode(&bits_of(SCM_PLUS)).expect("a frame");
        assert_eq!(r.get("id"), Some(&Value::Int(68211547)));
        assert_eq!(r.get("ert_type"), Some(&Value::Int(0xab)));
        assert_eq!(r.get("commodity"), Some(&Value::Text("Water".into())));
        assert_eq!(r.get("consumption"), Some(&Value::Int(6883)));
        assert_eq!(r.get("tamper"), Some(&Value::Text("4900".into())));
        assert_eq!(r.crc_valid, Some(true));
    }

    #[test]
    fn an_idm_frame_reads_its_intervals() {
        let r = ErtIdm.decode(&bits_of(IDM)).expect("a frame");
        assert_eq!(r.get("id"), Some(&Value::Int(11278109)));
        assert_eq!(r.get("ert_type"), Some(&Value::Int(0x17)));
        assert_eq!(r.get("commodity"), Some(&Value::Text("Electric".into())));
        assert_eq!(r.get("consumption"), Some(&Value::Int(339972)));
        assert_eq!(r.get("interval_count"), Some(&Value::Int(246)));
        assert_eq!(r.get("app_version"), Some(&Value::Int(4)));
        assert_eq!(r.get("packet_type"), Some(&Value::Text("1C".into())));
        assert_eq!(r.get("transmit_offset"), Some(&Value::Int(476)));
        assert_eq!(r.get("tamper"), Some(&Value::Text("020100EF0900".into())));
        let Some(Value::Text(iv)) = r.get("intervals") else { panic!("no intervals") };
        assert_eq!(iv.split(',').count(), 47);
        assert_eq!(iv.split(',').filter(|v| *v == "1").count(), 3);
        assert_eq!(iv.split(',').position(|v| v == "1"), Some(24));
    }

    #[test]
    fn a_corrupt_scm_frame_fails_its_check() {
        let mut frame = SCM.to_string();
        // One byte of the consumption, which the check covers.
        frame.replace_range(10..12, "ff");
        assert_eq!(ErtScm.decode(&bits_of(&frame)), Err(DecodeError::CrcFailed));
    }

    #[test]
    fn each_message_ignores_the_others() {
        assert_eq!(ErtScmPlus.decode(&bits_of(SCM)), Err(DecodeError::NotThisProtocol));
        assert_eq!(ErtIdm.decode(&bits_of(SCM_PLUS)), Err(DecodeError::NotThisProtocol));
        // Twelve bits of SCM sync turn up by chance inside 720 bits of IDM, so
        // that one reaches the check and is thrown out by it.
        assert_eq!(ErtScm.decode(&bits_of(IDM)), Err(DecodeError::CrcFailed));
    }

    /// Noise the length of a long burst, at every message: a sync word that
    /// turns up by chance still has to get its check past, and 4096 tries do
    /// not manage it.
    #[test]
    fn noise_decodes_as_nothing() {
        let mut state = 0x1234_5678u32;
        for _ in 0..4096 {
            let mut b = BitBuffer::new();
            for _ in 0..1024 {
                state = state.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
                b.push(state & 0x8000 != 0);
            }
            assert!(ErtScm.decode(&b).is_err());
            assert!(ErtScmPlus.decode(&b).is_err());
            assert!(ErtIdm.decode(&b).is_err());
        }
    }
}
