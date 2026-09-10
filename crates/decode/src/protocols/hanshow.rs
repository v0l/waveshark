//! Hanshow Stellar shelf labels: the heartbeat a shop's own tags send.
//!
//! Hanshow's tags run stock firmware on an A7106 transceiver and talk to the
//! store's base stations on 2.4 GHz, GFSK, 500 kbit/s up and 100 kbit/s down,
//! on channels 500 kHz apart. A tag sleeps almost all the time: it opens a
//! receive window of about 3 ms every 8.2 seconds, and sends a heartbeat of
//! its own roughly every three minutes. The heartbeat is what a receiver can
//! count on hearing without a base station of its own to talk to.
//!
//! The layout is from the off-air reverse engineering in
//! `dustybee/HanshowESL`, which recovered the two heartbeat structures from
//! captures at 2.401 GHz together with the field names the firmware uses. A
//! frame is `AA AA AA AA`, the sync word `52 56 78 53`, a control byte saying
//! which heartbeat it is, the structure, and a CRC-16 whose polynomial and
//! span that work did not pin down.
//!
//! So there is no integrity check to report here, and the corroboration is the
//! frame's shape instead: a 32-bit sync word behind a preamble, a control byte
//! from a known set, and the exact length that control byte implies. The
//! stream is inverted against this slicer's convention, and the sync is
//! searched for both ways round rather than assumed, since which tone a
//! discriminator calls the mark depends on the tuner's side of the carrier.
//!
//! What reports is a tag's identity and its state: the ESL id printed on the
//! label's own barcode, the wakeup and data channels it was assigned, its
//! battery level in the three bits the firmware gives it, whether the link is
//! encrypted, the panel temperature and the display it carries. That is a
//! census of a shop's tags and their health, not the prices: the price is an
//! image pushed down in the other direction.

use crate::bits::BitBuffer;
use crate::protocol::{DecodeError, Protocol, Report};
use crate::slicer::{Coding, Timing};

/// Preamble `AA` then the sync word, as it reads with the inversion already
/// undone.
const SYNC: [u8; 4] = [0x52, 0x56, 0x78, 0x53];
const SYNC_BITS: usize = 32;

/// Alternating preamble bits required in front of the sync. The tags send four
/// bytes of it.
const PREAMBLE_MIN: usize = 16;

const CTRL_NORMAL: u8 = 0x81;
const CTRL_REED: u8 = 0x82;
const CTRL_REQ: u8 = 0x83;
const CTRL_TABLE: u8 = 0x84;

/// `esl_regist`, from the control byte to the end of the CRC.
const NORMAL_LEN: usize = 26;
/// `esl_table_regist`, likewise.
const TABLE_LEN: usize = 25;

pub struct Hanshow {
    name: &'static str,
    bit_us: u32,
}

impl Hanshow {
    /// The uplink rate the radio table gives, 500 kbit/s: 2 us per bit.
    pub fn uplink_500k() -> Self {
        Self { name: "Hanshow-500k", bit_us: 2 }
    }

    /// The rate the capture settings imply, 100 kbit/s at 20 samples per
    /// symbol on 2 MS/s. The published rate table and the published capture
    /// settings disagree about which one carries the heartbeat, so both are
    /// tried rather than picking one.
    pub fn uplink_100k() -> Self {
        Self { name: "Hanshow-100k", bit_us: 10 }
    }
}

impl Protocol for Hanshow {
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
        let normal = bits.find(&SYNC, SYNC_BITS).map(|at| (at, false));
        let inverted = bits.inverted();
        let flipped = inverted.find(&SYNC, SYNC_BITS).map(|at| (at, true));
        let (at, invert) = normal.or(flipped).ok_or(DecodeError::NotThisProtocol)?;
        let bits = if invert { &inverted } else { bits };

        let lead = (1..=PREAMBLE_MIN).all(|k| match at.checked_sub(k + 1) {
            Some(i) => bits.get(i) != bits.get(i + 1),
            None => false,
        });
        if !lead {
            return Err(DecodeError::NotThisProtocol);
        }

        let start = at + SYNC_BITS;
        let avail = (bits.len() - start) / 8;
        let body: Vec<u8> =
            (0..avail).filter_map(|i| bits.extract(start + i * 8, 8).map(|v| v as u8)).collect();
        let want = match body.first() {
            Some(&CTRL_TABLE) => TABLE_LEN,
            Some(&(CTRL_NORMAL | CTRL_REED | CTRL_REQ)) => NORMAL_LEN,
            _ => return Err(DecodeError::NotThisProtocol),
        };
        if body.len() < want {
            return Err(DecodeError::WrongLength { got: body.len(), want });
        }
        let body = &body[..want];

        let mut r = Report::new(self.name).bool("inverted", invert);
        r.raw = body.to_vec();
        // The CRC's polynomial and span are unknown, so it is reported rather
        // than checked, and the frame stays unverified.
        r.crc_valid = None;
        let crc = u16::from_be_bytes([body[want - 2], body[want - 1]]);
        r = r.text("crc", format!("{crc:04x}"));

        Ok(match body[0] {
            CTRL_TABLE => r
                .text("kind", "table-heartbeat")
                .text("product_id", hex(&body[1..5]))
                .int("height", i64::from(u16::from_le_bytes([body[5], body[6]])))
                .int("width", i64::from(u16::from_le_bytes([body[7], body[8]])))
                .int("dpi", i64::from(body[9]))
                .int("pages", i64::from(body[10]))
                .int("flash_bytes", i64::from(u16::from_le_bytes([body[11], body[12]]))),
            ctrl => {
                let info = body[13];
                r.text(
                    "kind",
                    match ctrl {
                        CTRL_REED => "reed-heartbeat",
                        CTRL_REQ => "request-heartbeat",
                        _ => "heartbeat",
                    },
                )
                .text("wakeup_id", hex(&body[1..4]))
                .text("esl_id", hex(&body[4..8]))
                .identified_by("esl_id")
                .int("wakeup_chan", i64::from(body[8]))
                .int("group_chan", i64::from(body[9]))
                .int("data_chan", i64::from(body[10]))
                .int("netmask", i64::from(body[11]))
                .int("battery", i64::from(info & 7))
                .bool("encrypted", info & 0x40 != 0)
                .bool("flash_error", info & 0x80 != 0)
                .text("display_id", hex(&body[14..18]))
                .int("temp_c", i64::from(body[19] as i8))
                .int("rom_version", i64::from(body[20]))
            }
        })
    }
}

fn hex(b: &[u8]) -> String {
    b.iter().map(|x| format!("{x:02x}")).collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protocol::Value;

    /// The normal heartbeat printed in `dustybee/HanshowESL`, byte for byte.
    const NORMAL: [u8; NORMAL_LEN] = [
        0x81, 0x50, 0x81, 0x03, 0x50, 0xc3, 0xcc, 0x62, 0x97, 0x97, 0x97, 0x1d, 0x23, 0x23, 0x20,
        0x26, 0xa3, 0x54, 0x00, 0x04, 0x0c, 0x25, 0xfc, 0x34, 0x00, 0xcb,
    ];

    fn on_air(body: &[u8], invert: bool) -> BitBuffer {
        let mut air = vec![0xaa; 4];
        air.extend_from_slice(&SYNC);
        air.extend_from_slice(body);
        let mut b = BitBuffer::new();
        for byte in air {
            let byte = if invert { !byte } else { byte };
            for i in 0..8 {
                b.push(byte & (0x80 >> i) != 0);
            }
        }
        b
    }

    #[test]
    fn a_heartbeat_reports_the_tag_and_its_channels() {
        let r = Hanshow::uplink_100k().decode(&on_air(&NORMAL, false)).expect("a frame");
        assert_eq!(r.fields["kind"], Value::Text("heartbeat".into()));
        assert_eq!(r.fields["esl_id"], Value::Text("50c3cc62".into()));
        assert_eq!(r.fields["wakeup_chan"], Value::Int(0x97));
        assert_eq!(r.fields["data_chan"], Value::Int(0x97));
        assert_eq!(r.fields["battery"], Value::Int(3));
        assert_eq!(r.fields["encrypted"], Value::Bool(false));
        // Nothing here can be checked: the CRC's span is not known.
        assert_eq!(r.crc_valid, None);
    }

    /// Which tone the discriminator calls the mark depends on which side of
    /// the carrier the tuner sits, so the frame has to read either way round.
    #[test]
    fn an_inverted_capture_reads_the_same() {
        let r = Hanshow::uplink_100k().decode(&on_air(&NORMAL, true)).expect("a frame");
        assert_eq!(r.fields["inverted"], Value::Bool(true));
        assert_eq!(r.fields["esl_id"], Value::Text("50c3cc62".into()));
    }

    #[test]
    fn a_table_heartbeat_reports_the_panel() {
        let body = [
            0x84, 0x20, 0x26, 0xa3, 0x54, 0x7a, 0x00, 0xfa, 0x00, 0x83, 0x04, 0xe8, 0x00, 0xcc,
            0x00, 0x14, 0x01, 0xd5, 0x00, 0x00, 0x20, 0x03, 0x00, 0xe2, 0x1f,
        ];
        let r = Hanshow::uplink_500k().decode(&on_air(&body, false)).expect("a frame");
        assert_eq!(r.fields["kind"], Value::Text("table-heartbeat".into()));
        assert_eq!(r.fields["height"], Value::Int(122));
        assert_eq!(r.fields["width"], Value::Int(250));
        assert_eq!(r.fields["dpi"], Value::Int(131));
    }

    /// The sync word alone is not a frame: without a check inside, an
    /// unexpected control byte is the only thing that can refuse one.
    #[test]
    fn an_unknown_control_byte_is_refused() {
        let mut body = NORMAL;
        body[0] = 0x11;
        assert!(Hanshow::uplink_100k().decode(&on_air(&body, false)).is_err());
    }

    #[test]
    fn a_truncated_frame_is_refused() {
        assert!(Hanshow::uplink_100k().decode(&on_air(&NORMAL[..12], false)).is_err());
    }
}
