//! FrSky ACCST D16, the 2.4 GHz link on most FrSky and Jumper handsets.
//!
//! A CC2500 keying GFSK at 70 kbit/s with 57 kHz of deviation, hopping across
//! 47 channels 1.5 MHz apart on a 9 ms frame. Nothing in it is encrypted:
//! the packet carries the transmitter's id, where it is in the hop sequence,
//! the receiver number and eight channels of stick positions, and the
//! receiver answers with telemetry on the same channel.
//!
//! Layout and hop tables from `pascallanger/DIY-Multiprotocol-TX-Module`
//! (`FrSkyX_cc2500.ino`, `FrSkyDVX_common.ino`), which reproduces what a
//! handset transmits well enough that real receivers bind to it.
//!
//! # What the CRC is worth
//!
//! Twenty-nine bytes under a sixteen bit CRC with no seed, so one packet in
//! 65536 passes by chance and a burst that is not FrSky at all almost never
//! does. That is much stronger evidence than ExpressLRS's fourteen bits, and
//! unlike ExpressLRS the CRC carries no binding id, so a listener needs to
//! know nothing about the link to check it. The id is in the packet instead,
//! in the clear, which is what makes a handset identifiable.

use common::Value;

/// Bytes in a D16 packet, the length byte included. The European variant
/// with listen-before-talk sends 0x20 instead.
pub const PACKET_LEN: usize = 0x1d + 1;

/// Channels in the hop sequence.
pub const HOP_CHANNELS: usize = 47;

/// The CC2500's channel spacing in this configuration, from its MDMCFG
/// registers.
pub const CHANNEL_SPACING_HZ: f64 = 299_927.0;

/// Where channel zero sits, from the FREQ registers.
pub const BASE_HZ: f64 = 2_404_000_000.0;

/// What the modulation is, for a front end that has to be told.
pub const BITRATE: f64 = 70_027.0;
pub const DEVIATION_HZ: f64 = 57_129.0;

const CRC_SHORT: [u16; 16] = [
    0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf, 0x8c48, 0x9dc1, 0xaf5a, 0xbed3,
    0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
];

fn crc_entry(v: u8) -> u16 {
    // A sixteen entry table and a multiply, rather than 256 entries: the
    // firmware does it this way to fit an eight bit micro, and the result is
    // an ordinary CRC-16.
    CRC_SHORT[usize::from(v & 0x0f)] ^ (0x1081u16.wrapping_mul(u16::from(v / 16)))
}

pub fn crc(data: &[u8]) -> u16 {
    data.iter().fold(0u16, |c, &b| {
        (c << 8) ^ crc_entry(((c >> 8) as u8) ^ b)
    })
}

/// A data packet: stick positions and where the link is.
#[derive(Clone, Debug, PartialEq)]
pub struct Packet {
    /// The handset's id, which does not change and is what identifies it.
    pub id: u16,
    /// Third id byte, constant in current firmware.
    pub id_extra: u8,
    /// Which entry of the hop sequence this was sent on.
    pub hop_index: u8,
    /// How far the sequence advances each frame, which with the id fixes the
    /// whole hop pattern.
    pub chanskip: u8,
    /// Receiver number, so one handset can address several models.
    pub rx_number: u8,
    /// Non-zero while the handset is sending failsafe positions or is in
    /// range-check mode.
    pub flags: u8,
    /// Eight channels, as the packet carries them. 0x800 set marks the upper
    /// bank, so a sixteen channel model alternates banks frame by frame.
    pub channels: [u16; 8],
    /// Telemetry sequence nibble the handset is acknowledging.
    pub rx_seq: u8,
    pub tx_seq: u8,
}

impl Packet {
    /// Whether this frame carries channels 9 to 16 rather than 1 to 8.
    pub fn upper_bank(&self) -> bool {
        self.channels.iter().any(|c| c & 0x800 != 0)
    }

    /// A channel as microseconds of servo pulse, the way a handset means it.
    ///
    /// The scaling is the PXX one: 1500 us sits at 1024 and the range either
    /// side is 3/4 of the raw span.
    ///
    /// Bit 11 says which bank the frame carries and is not part of the
    /// position, so it is masked off. Clamping to 0x7ff instead read every
    /// channel of an upper-bank frame as full deflection.
    pub fn microseconds(&self, index: usize) -> Option<f64> {
        let raw = f64::from(self.channels.get(index)? & 0x7ff);
        Some((raw - 1024.0) * 4.0 / 3.0 / 2.0 + 1500.0)
    }

    /// The sticks this packet carried, for the views that draw a control
    /// link.
    ///
    /// Eight of the sixteen, since a frame carries one bank and the next
    /// carries the other. The eight it did not carry are absent, which is
    /// what lets a view merge the two frames instead of watching half the
    /// channels drop to zero every other frame.
    pub fn control(&self) -> common::ReportDetail {
        let mut channels = [None; common::CONTROL_CHANNELS];
        let base = if self.upper_bank() { 8 } else { 0 };
        for i in 0..8 {
            if let (Some(us), Some(slot)) = (self.microseconds(i), channels.get_mut(base + i)) {
                *slot = Some(us.round() as u16);
            }
        }
        common::ReportDetail::Control { channels, armed: None, uplink_power_mw: None }
    }
}

/// Read a packet, refusing anything whose CRC does not agree.
///
/// The CRC covers everything from the id to the end, which is what makes a
/// decode evidence rather than a guess.
pub fn parse(packet: &[u8]) -> Option<Packet> {
    // The first byte counts the bytes after it, so a whole packet is one
    // longer. A length that disagrees with the buffer is a truncated
    // reception.
    let len = usize::from(*packet.first()?);
    if len < 0x1d || packet.len() < len + 1 {
        return None;
    }
    let sent = (u16::from(packet[len - 1]) << 8) | u16::from(packet[len]);
    if crc(&packet[3..len - 1]) != sent {
        return None;
    }
    let mut channels = [0u16; 8];
    for (i, pair) in channels.chunks_mut(2).enumerate() {
        let b = &packet[9 + i * 3..12 + i * 3];
        pair[0] = u16::from(b[0]) | (u16::from(b[1] & 0x0f) << 8);
        pair[1] = u16::from(b[1] >> 4) | (u16::from(b[2]) << 4);
    }
    Some(Packet {
        id: (u16::from(packet[1]) << 8) | u16::from(packet[2]),
        id_extra: packet[3],
        hop_index: packet[4] & 0x3f,
        chanskip: (packet[5] << 2) | (packet[4] >> 6),
        rx_number: packet[6],
        flags: packet[7],
        channels,
        rx_seq: packet[21] >> 4,
        tx_seq: packet[21] & 0x0f,
    })
}

/// The hop sequence a v2 handset generates from its id.
///
/// Two of the exception lists in the firmware are labelled "from dumps",
/// which is a reminder that this is reverse engineered rather than published:
/// the increments 12 and 35 are skipped and a handful of channels are nudged
/// aside, for reasons nobody outside FrSky knows.
pub fn hop_sequence_v2(id: u16, lbt: bool) -> Vec<u8> {
    let mut inc = (id % 46) as u8 + 1;
    if inc == 12 || inc == 35 {
        inc += 1;
    }
    let offset = (id % 5) as u8;
    (0..HOP_CHANNELS)
        .map(|i| {
            let mut ch = 5 * ((u16::from(inc) * i as u16) % 47) as u8 + offset;
            if lbt {
                if ch <= 1 || matches!(ch, 43 | 44 | 87 | 88 | 129 | 130 | 173 | 174) {
                    ch += 2;
                } else if matches!(ch, 216..=218) {
                    ch += 3;
                }
            } else if matches!(ch, 3 | 4 | 46 | 47 | 90 | 91 | 133 | 134 | 176 | 177 | 220 | 221) {
                ch += 2;
            }
            ch
        })
        .collect()
}

/// The hop sequence a v1 handset generates, which walks the band by a spacing
/// its own id sets rather than by a multiplier.
pub fn hop_sequence_v1(id_low: u8, spacing: u8) -> Vec<u8> {
    let mut channel = id_low & 0x07;
    let mut spacing = spacing;
    if spacing < 0x02 {
        spacing += 0x02;
    }
    if spacing > 0xe9 {
        spacing -= 0xe7;
    }
    if spacing.is_multiple_of(0x2f) {
        spacing += 1;
    }
    let mut out = vec![channel];
    for _ in 1..HOP_CHANNELS {
        channel = ((u16::from(channel) + u16::from(spacing)) % 0xeb) as u8;
        // Three channels the firmware refuses to use.
        if matches!(channel, 0x00 | 0x5a | 0xdc) {
            channel += 1;
        }
        out.push(channel);
    }
    out
}

/// Where a CC2500 channel number sits.
pub fn channel_hz(channel: u8) -> f64 {
    BASE_HZ + f64::from(channel) * CHANNEL_SPACING_HZ
}

/// The fields a log or a bus carries.
pub fn fields(p: &Packet) -> Vec<(String, Value)> {
    let mut f: Vec<(String, Value)> = vec![
        ("id".into(), Value::Text(format!("{:04x}", p.id))),
        ("rx_number".into(), Value::Int(i64::from(p.rx_number))),
        ("hop_index".into(), Value::Int(i64::from(p.hop_index))),
        ("chanskip".into(), Value::Int(i64::from(p.chanskip))),
        (
            "bank".into(),
            Value::Text(if p.upper_bank() { "9-16" } else { "1-8" }.into()),
        ),
    ];
    for i in 0..8 {
        if let Some(us) = p.microseconds(i) {
            f.push((format!("ch{}_us", i + 1), Value::Float(us.round())));
        }
    }
    if p.flags != 0 {
        f.push(("flags".into(), Value::Text(format!("0x{:02x}", p.flags))));
    }
    f
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A data packet from the Multiprotocol source's own comment, which is a
    /// dump of what a handset transmits. Not this project's bytes, and the
    /// CRC at the end is the handset's.
    const PACKET: [u8; 30] = [
        0x1d, 0xb3, 0xfd, 0x02, 0x56, 0x07, 0x15, 0x00, 0x00, 0x00, 0x04, 0x40, 0x00, 0x04, 0x40,
        0x00, 0x04, 0x40, 0x00, 0x04, 0x40, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x96, 0x12,
    ];

    #[test]
    fn the_crc_agrees_with_a_transmitted_packet() {
        assert_eq!(crc(&PACKET[3..0x1d - 1]), 0x9612);
    }

    #[test]
    fn a_packet_reports_the_handset_and_its_sticks() {
        let p = parse(&PACKET).expect("a packet");
        assert_eq!(p.id, 0xb3fd);
        assert_eq!(p.rx_number, 0x15);
        assert_eq!(p.hop_index, 22);
        assert_eq!(p.chanskip, 29);
        // Every channel at 1024 is every stick centred, which is what this
        // dump holds.
        assert!(p.channels.iter().all(|c| c & 0x7ff == 0x400));
        for i in 0..8 {
            assert_eq!(p.microseconds(i).unwrap().round(), 1500.0);
        }
        assert!(!p.upper_bank());
    }

    /// One bit changed inside the CRC and the packet is refused. Sixteen
    /// bits over twenty-five is what makes a decode here evidence rather
    /// than a plausible shape.
    ///
    /// The two id bytes in front of it are outside the CRC: the CC2500
    /// filters on them in hardware, so a receiver never sees a packet whose
    /// address is wrong and the firmware does not check them again. A
    /// listener has no such filter, which is worth knowing before treating a
    /// reported id as certain.
    #[test]
    fn a_corrupted_packet_is_refused() {
        for bit in [40usize, 100, 200] {
            let mut p = PACKET;
            p[bit / 8] ^= 1 << (bit % 8);
            assert!(parse(&p).is_none(), "bit {bit} was not noticed");
        }
    }

    #[test]
    fn random_bytes_almost_never_pass() {
        let mut seed = 0x1234_5678_9abc_def0u64;
        let mut passed = 0;
        for _ in 0..20_000 {
            let mut p = [0u8; 30];
            for b in p.iter_mut() {
                seed ^= seed << 13;
                seed ^= seed >> 7;
                seed ^= seed << 17;
                *b = seed as u8;
            }
            p[0] = 0x1d;
            if parse(&p).is_some() {
                passed += 1;
            }
        }
        // 20000/65536 is well under one. Anything above a handful means the
        // CRC is not being computed over the right bytes.
        assert!(passed <= 3, "{passed} of 20000 random packets passed");
    }

    /// The hop sequence has to have the shape the firmware promises: every
    /// channel used once, spread across the band, and a different id hopping
    /// differently.
    #[test]
    fn a_hop_sequence_covers_the_band_without_repeating() {
        let seq = hop_sequence_v2(0xb3fd, false);
        assert_eq!(seq.len(), HOP_CHANNELS);
        let mut seen = seq.clone();
        seen.sort_unstable();
        seen.dedup();
        assert_eq!(seen.len(), HOP_CHANNELS, "a channel repeats");
        assert_ne!(seq, hop_sequence_v2(0x1234, false));
        // The band it walks is the 2.4 GHz ISM one.
        let lo = channel_hz(*seq.iter().min().unwrap());
        let hi = channel_hz(*seq.iter().max().unwrap());
        assert!(lo > 2_400e6 && hi < 2_484e6, "{lo} to {hi}");
    }

    /// The listen-before-talk tables differ, which is a regulatory
    /// difference rather than a cosmetic one: a European handset avoids
    /// channels a north American one uses.
    #[test]
    fn the_european_hop_table_is_not_the_american_one() {
        assert_ne!(hop_sequence_v2(0xb3fd, true), hop_sequence_v2(0xb3fd, false));
    }

    #[test]
    fn the_v1_sequence_refuses_the_channels_the_firmware_refuses() {
        let seq = hop_sequence_v1(0x05, 0x2f * 2);
        assert_eq!(seq.len(), HOP_CHANNELS);
        assert!(!seq.iter().any(|&c| matches!(c, 0x00 | 0x5a | 0xdc)));
    }

    /// Bit 11 marks the bank rather than the position, so a channel of an
    /// upper-bank frame is the same width as the same value in a lower one.
    /// Clamping to 0x7ff instead of masking read every one of them as full
    /// deflection, which on a sixteen channel model is every other frame.
    #[test]
    fn the_bank_bit_is_not_part_of_the_position() {
        let mut p = parse(&PACKET).expect("a packet");
        assert_eq!(p.microseconds(0).unwrap().round(), 1500.0);
        for c in &mut p.channels {
            *c |= 0x800;
        }
        assert!(p.upper_bank());
        assert_eq!(p.microseconds(0).unwrap().round(), 1500.0);
    }

    /// A frame carries one bank of eight, so the other eight are absent and
    /// not zero: a view merges the two frames, and a zero would be a stick
    /// slammed to its stop every other frame.
    #[test]
    fn a_frame_reports_the_bank_it_carried_and_no_more() {
        let mut p = parse(&PACKET).expect("a packet");
        let common::ReportDetail::Control { channels, .. } = p.control() else {
            panic!("not a control report")
        };
        assert!(channels[..8].iter().all(Option::is_some), "{channels:?}");
        assert!(channels[8..].iter().all(Option::is_none), "{channels:?}");
        // Every channel of this packet is centred, which is 1500 us.
        assert_eq!(channels[0], Some(1500));

        for c in &mut p.channels {
            *c |= 0x800;
        }
        let common::ReportDetail::Control { channels, .. } = p.control() else {
            panic!("not a control report")
        };
        assert!(channels[..8].iter().all(Option::is_none), "{channels:?}");
        assert_eq!(channels[8], Some(1500));
    }

}
