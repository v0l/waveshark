//! OpenEPaperLink and Chroma electronic shelf labels on 868 and 915 MHz.
//!
//! Shelf labels are e-paper price tags: a store's access point holds an image
//! for each tag, the tag wakes every few minutes, asks whether anything has
//! changed, and pulls the new image down in blocks. The sub-GHz families use a
//! CC1101, CC1110 or CC1310, and the link layer is that radio's default
//! framing: preamble, a sync word, a length byte, the payload, and a CRC-16
//! over both, with TI's PN9 whitening applied from the length byte on.
//!
//! Two PHYs are in use, taken from the register dumps OpenEPaperLink's access
//! point firmware carries (`ARM_Tag_FW/OpenEPaperLink_esp32_C6_AP/main/
//! SubGigRadio.c`). The stock Chroma configuration is GFSK at 249.94 kbit/s
//! with 165 kHz deviation on 903 to 923 MHz; OpenEPaperLink's own is GFSK at
//! 38.38 kbit/s with 20.6 kHz deviation on six channels from 864.0 to 869.0
//! MHz and six more from 903 MHz up. Only the bit period differs here, since
//! the framing above it is the same.
//!
//! The sync word is not matched against a constant. The firmware sets one
//! explicitly for its own configuration and inherits the base config's for the
//! stock one, and a tag on a shelf can be running either, so the frame is
//! taken from the end of the preamble and the CRC-16 decides whether it was a
//! frame. What names the family is the payload: an 802.15.4 header carrying
//! PAN id 0x4447 and a packet type in `oepl-proto.h`'s range, with the
//! additive checksum that file's `checkCRC` computes.
//!
//! What reports is who is talking and what about: the tag's MAC, its battery
//! voltage, temperature and firmware version on a check-in, and on the access
//! point's side the size of the image being pushed and when the tag is told to
//! come back. The image blocks themselves are reported as their length only.

use crate::bits::BitBuffer;
use crate::protocol::{DecodeError, Protocol, Report};
use crate::slicer::{Coding, Timing};
use crate::whiten::read_framed;

/// PAN id every OpenEPaperLink and Chroma frame carries, from `oepl-proto.h`.
const PAN: u16 = 0x4447;

/// Alternating bits that must precede the frame. The preamble is 24 bytes on
/// the stock configuration and 4 on OpenEPaperLink's, so this is the shorter
/// one with room for a receiver that opened late.
const PREAMBLE_MIN: usize = 24;

/// Bit offsets tried for the start of the sync word, counted back from where
/// the alternating run stops.
///
/// A sync word whose own leading bits alternate carries the preamble on past
/// where it really ended, so the break is found late by as many bits as the
/// sync word begins with. Each candidate is another chance for the CRC to pass
/// by luck, at one in 65536 apiece, so the sweep stops at four.
const SYNC_SHIFTS: [usize; 4] = [0, 1, 2, 3];

/// Smallest payload reported as a bare frame. A CRC-16 passes by luck once in
/// 65536 candidates and a burst offers a few dozen, so a very short unparsed
/// frame is likelier to be that coincidence than a shelf label.
const MIN_BARE_PAYLOAD: usize = 8;

pub struct Esl {
    name: &'static str,
    bit_us: u32,
}

impl Esl {
    /// OpenEPaperLink's own configuration: 38.3835 kbit/s, 26 us per bit.
    pub fn sub_ghz_38k() -> Self {
        Self { name: "ESL-38k", bit_us: 26 }
    }

    /// The stock Chroma configuration: 249.939 kbit/s, 4 us per bit.
    pub fn sub_ghz_250k() -> Self {
        Self { name: "ESL-250k", bit_us: 4 }
    }
}

impl Protocol for Esl {
    fn name(&self) -> &'static str {
        self.name
    }

    fn timing(&self) -> Timing {
        Timing {
            coding: Coding::Nrz,
            short_us: self.bit_us,
            long_us: self.bit_us,
            sync_us: 0,
            tolerance_us: 0,
            reset_us: self.bit_us * 64,
        }
    }

    fn decode(&self, bits: &BitBuffer) -> Result<Report, DecodeError> {
        for start in preamble_ends(bits) {
            for shift in SYNC_SHIFTS {
                let Some(at) = start.checked_sub(shift) else {
                    continue;
                };
                let bytes = bytes_from(bits, at);
                let Some(frame) = read_framed(&bytes) else {
                    continue;
                };
                let mut r = match parse(self.name, &frame.payload) {
                    Some(r) => r.bool("oepl", true),
                    // A stock tag's payload is Solum's and unpublished, so the
                    // frame reports as bytes rather than not at all.
                    None if frame.payload.len() >= MIN_BARE_PAYLOAD => Report::new(self.name)
                        .bool("oepl", false)
                        .int("length", frame.payload.len() as i64)
                        .text("body", hex(&frame.payload)),
                    None => continue,
                };
                r.crc_valid = Some(true);
                r.raw = frame.payload.clone();
                let sync = hex(&bytes[..frame.sync_len]);
                return Ok(r.text("sync", sync).bool("whitened", frame.whitened));
            }
        }
        Err(DecodeError::NotThisProtocol)
    }
}

/// Bit offsets just past each run of at least `PREAMBLE_MIN` alternating bits.
fn preamble_ends(bits: &BitBuffer) -> Vec<usize> {
    let mut ends = Vec::new();
    let mut run = 1usize;
    for i in 1..bits.len() {
        if bits.get(i) != bits.get(i - 1) {
            run += 1;
            continue;
        }
        if run >= PREAMBLE_MIN {
            ends.push(i);
        }
        run = 1;
    }
    if run >= PREAMBLE_MIN {
        ends.push(bits.len());
    }
    ends
}

fn bytes_from(bits: &BitBuffer, at: usize) -> Vec<u8> {
    let n = (bits.len().saturating_sub(at)) / 8;
    (0..n).filter_map(|i| bits.extract(at + i * 8, 8).map(|v| v as u8)).collect()
}

/// Packet types, from `oepl-proto.h`.
fn type_name(t: u8) -> Option<&'static str> {
    Some(match t {
        0xe1 => "tag-return-data",
        0xe2 => "tag-return-data-ack",
        0xe3 => "avail-data-shortreq",
        0xe4 => "block-request",
        0xe5 => "avail-data-req",
        0xe6 => "avail-data-info",
        0xe7 => "block-partial-request",
        0xe8 => "block-part",
        0xe9 => "block-request-ack",
        0xea => "xfer-complete",
        0xeb => "xfer-complete-ack",
        0xec => "cancel-xfer",
        0xed => "ping",
        0xee => "pong",
        _ => return None,
    })
}

fn hex(b: &[u8]) -> String {
    b.iter().map(|x| format!("{x:02x}")).collect()
}

/// MAC addresses go out least significant byte first; print them the way
/// OpenEPaperLink's own interface shows a tag.
fn mac(b: &[u8]) -> String {
    b.iter().rev().map(|x| format!("{x:02X}")).collect()
}

fn le16(b: &[u8], at: usize) -> u16 {
    u16::from_le_bytes([b[at], b[at + 1]])
}

fn le32(b: &[u8], at: usize) -> u32 {
    u32::from_le_bytes([b[at], b[at + 1], b[at + 2], b[at + 3]])
}

fn le64(b: &[u8], at: usize) -> u64 {
    let mut v = [0u8; 8];
    v.copy_from_slice(&b[at..at + 8]);
    u64::from_le_bytes(v)
}

/// The additive checksum `oepl-proto.h`'s `checkCRC` computes: every byte of
/// the structure but the first, summed into the first.
fn checksum_ok(body: &[u8]) -> bool {
    match body.split_first() {
        Some((&want, rest)) => rest.iter().fold(0u8, |a, &b| a.wrapping_add(b)) == want,
        None => false,
    }
}

fn parse(model: &'static str, p: &[u8]) -> Option<Report> {
    if p.len() < 15 {
        return None;
    }
    let frame_type = p[0] & 7;
    let pan_compressed = (p[0] >> 6) & 1;
    let dest_mode = (p[1] >> 2) & 3;
    let src_mode = (p[1] >> 6) & 3;
    if frame_type != 1 {
        return None;
    }

    let mut r = Report::new(model).int("seq", i64::from(p[2]));
    let head = match (dest_mode, src_mode, pan_compressed) {
        // Broadcast, tag to whichever access point hears it.
        (2, 3, 0) => {
            if le16(p, 3) != PAN || le16(p, 7) != PAN {
                return None;
            }
            r = r.text("from", mac(&p[9..17])).text("to", "broadcast");
            17
        }
        // Tag to access point, or the reply, both addressed long.
        (3, 3, 1) => {
            if p.len() < 21 || le16(p, 3) != PAN {
                return None;
            }
            r = r.text("to", mac(&p[5..13])).text("from", mac(&p[13..21]));
            21
        }
        // Access point to tag: the tag addressed long, the AP by its short id.
        (3, 2, 1) => {
            if le16(p, 3) != PAN {
                return None;
            }
            r = r.text("to", mac(&p[5..13])).text("from", format!("{:04X}", le16(p, 13)));
            15
        }
        _ => return None,
    };

    let kind = type_name(*p.get(head)?)?;
    let body = &p[head + 1..];
    r = r.text("type", kind);
    if !body.is_empty() {
        r = r.bool("checksum_ok", checksum_ok(body));
    }

    Some(match (kind, body.len()) {
        // AvailDataReq, and the shorter one older tags send.
        ("avail-data-req" | "avail-data-shortreq", n) if n >= 9 => {
            let mut r = r
                .int("lqi", i64::from(body[1]))
                .int("rssi_dbm", i64::from(body[2] as i8))
                .int("temp_c", i64::from(body[3] as i8))
                .int("battery_mv", i64::from(le16(body, 4)))
                .int("hw_type", i64::from(body[6]))
                .int("wakeup", i64::from(body[7]));
            if n >= 12 {
                r = r.int("fw", i64::from(le16(body, 9))).int("channel", i64::from(body[11]));
            }
            r
        }
        // AvailDataInfo: what the access point has waiting.
        ("avail-data-info", n) if n >= 17 => r
            .text("data_ver", format!("{:016X}", le64(body, 1)))
            .int("data_size", i64::from(le32(body, 9)))
            .int("data_type", i64::from(body[13]))
            .int("next_checkin_min", i64::from(le16(body, 15))),
        ("block-request", n) if n >= 17 => {
            r.text("data_ver", format!("{:016X}", le64(body, 1))).int("block", i64::from(body[9]))
        }
        ("block-part", n) if n >= 3 => r
            .int("block", i64::from(body[1]))
            .int("part", i64::from(body[2]))
            .int("bytes", n as i64 - 3),
        ("tag-return-data", n) if n >= 10 => {
            r.int("part", i64::from(body[1])).text("data_ver", format!("{:016X}", le64(body, 2)))
        }
        _ => r,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protocol::Value;
    use crate::whiten::{crc16_ti, pn9};

    /// Build what a CC1101 puts on the air: preamble, sync, then the whitened
    /// length, payload and CRC.
    fn on_air(sync: &[u8], payload: &[u8]) -> BitBuffer {
        let mut frame = vec![payload.len() as u8];
        frame.extend_from_slice(payload);
        frame.extend_from_slice(&crc16_ti(&frame).to_be_bytes());
        let mut air = vec![0xaa; 6];
        air.extend_from_slice(sync);
        air.extend_from_slice(&pn9(&frame));

        let mut b = BitBuffer::new();
        for byte in air {
            for i in 0..8 {
                b.push(byte & (0x80 >> i) != 0);
            }
        }
        b
    }

    fn with_checksum(head: &[u8], kind: u8, body: &[u8]) -> Vec<u8> {
        let sum = body.iter().fold(0u8, |a, &b| a.wrapping_add(b));
        let mut p = head.to_vec();
        p.push(kind);
        p.push(sum);
        p.extend_from_slice(body);
        p
    }

    /// A tag's check-in: broadcast, its own MAC as the source, battery and
    /// temperature in the body.
    fn check_in() -> Vec<u8> {
        let mut head = vec![0x01, 0xc8, 0x17];
        head.extend_from_slice(&PAN.to_le_bytes());
        head.extend_from_slice(&0xffffu16.to_le_bytes());
        head.extend_from_slice(&PAN.to_le_bytes());
        head.extend_from_slice(&[0x91, 0x8a, 0x00, 0x00, 0x00, 0x1a, 0x02, 0x00]);
        let mut body = vec![100, (-62i8) as u8, 21];
        body.extend_from_slice(&3012u16.to_le_bytes());
        body.extend_from_slice(&[0xdd, 0x02, 0x01]);
        body.extend_from_slice(&0x0021u16.to_le_bytes());
        body.push(103);
        body.push(0);
        body.extend_from_slice(&[0; 8]);
        with_checksum(&head, 0xe5, &body)
    }

    #[test]
    fn a_check_in_reports_the_tag_and_its_battery() {
        let bits = on_air(&[0xc7, 0x0a, 0xc7, 0x0a], &check_in());
        let r = Esl::sub_ghz_38k().decode(&bits).expect("a frame");
        assert_eq!(r.fields["type"], Value::Text("avail-data-req".into()));
        assert_eq!(r.fields["from"], Value::Text("00021A0000008A91".into()));
        assert_eq!(r.fields["battery_mv"], Value::Int(3012));
        assert_eq!(r.fields["temp_c"], Value::Int(21));
        assert_eq!(r.fields["rssi_dbm"], Value::Int(-62));
        assert_eq!(r.fields["fw"], Value::Int(0x21));
        assert_eq!(r.fields["channel"], Value::Int(103));
        assert_eq!(r.fields["checksum_ok"], Value::Bool(true));
        assert_eq!(r.crc_valid, Some(true));
    }

    /// The sync word is not a constant here, so a frame behind a different one
    /// still reads: the CRC is what says it was a frame.
    #[test]
    fn a_frame_behind_an_unknown_sync_still_reads() {
        let bits = on_air(&[0xd3, 0x91, 0xd3, 0x91], &check_in());
        assert!(Esl::sub_ghz_250k().decode(&bits).is_ok());
    }

    /// The access point's reply: addressed to the tag, its own id short.
    #[test]
    fn an_avail_data_info_reports_the_image_waiting() {
        let mut head = vec![0x41, 0x8c, 0x02];
        head.extend_from_slice(&PAN.to_le_bytes());
        head.extend_from_slice(&[0x91, 0x8a, 0x00, 0x00, 0x00, 0x1a, 0x02, 0x00]);
        head.extend_from_slice(&0x1234u16.to_le_bytes());
        let mut body = 0x0123456789abcdefu64.to_le_bytes().to_vec();
        body.extend_from_slice(&4096u32.to_le_bytes());
        body.push(0x20);
        body.push(0);
        body.extend_from_slice(&15u16.to_le_bytes());
        let bits = on_air(&[0xc7, 0x0a, 0xc7, 0x0a], &with_checksum(&head, 0xe6, &body));

        let r = Esl::sub_ghz_38k().decode(&bits).expect("a frame");
        assert_eq!(r.fields["type"], Value::Text("avail-data-info".into()));
        assert_eq!(r.fields["to"], Value::Text("00021A0000008A91".into()));
        assert_eq!(r.fields["from"], Value::Text("1234".into()));
        assert_eq!(r.fields["data_size"], Value::Int(4096));
        assert_eq!(r.fields["next_checkin_min"], Value::Int(15));
        assert_eq!(r.fields["data_ver"], Value::Text("0123456789ABCDEF".into()));
    }

    /// A stock tag's frame: the framing reads, the payload does not, and it
    /// still reports, because bytes off a real shelf are the point.
    #[test]
    fn a_frame_that_is_not_openepaperlink_reports_as_bytes() {
        let payload: Vec<u8> = (0u8..24).collect();
        let bits = on_air(&[0xc7, 0x0a, 0xc7, 0x0a], &payload);
        let r = Esl::sub_ghz_38k().decode(&bits).expect("a frame");
        assert_eq!(r.fields["oepl"], Value::Bool(false));
        assert_eq!(r.fields["length"], Value::Int(24));
        assert_eq!(r.fields["sync"], Value::Text("c70ac70a".into()));
        assert_eq!(r.raw, payload);
        assert_eq!(r.crc_valid, Some(true));
    }

    /// A payload too short to be a shelf label's is refused: at that length a
    /// CRC-16 passing means less than the odds of it passing by luck.
    #[test]
    fn a_frame_too_short_to_mean_anything_is_refused() {
        let bits = on_air(&[0xc7, 0x0a, 0xc7, 0x0a], &[1, 2, 3, 4, 5]);
        assert!(Esl::sub_ghz_38k().decode(&bits).is_err());
    }

    #[test]
    fn noise_without_a_preamble_is_not_a_frame() {
        let mut b = BitBuffer::new();
        let mut x = 0x1234_5678u32;
        for _ in 0..600 {
            x = x.wrapping_mul(1_103_515_245).wrapping_add(12345);
            b.push(x & 0x1000 != 0);
        }
        assert!(Esl::sub_ghz_38k().decode(&b).is_err());
    }
}
