//! Meteor-M pictures: the MSU-MR strips inside LRPT's space packets.
//!
//! What arrives from [`crate::ccsds`] is space packets on one application id
//! per instrument channel. Each image packet carries fourteen minimum coded
//! units, which are fourteen JPEG blocks side by side, so a packet is eight
//! rows of 112 pixels and fourteen packets are eight rows of the whole 1568
//! pixel scan. A packet says which column its first block starts at and the
//! quality factor its blocks were quantised with, and that is the whole
//! header: [`crate::jpeg`] does the rest.
//!
//! Nothing in a packet says which row of the picture it belongs to. What
//! there is is the packet counter, which counts every packet the satellite
//! sent on the channel, so the distance between the starts of two strips of
//! one channel is how many packets a strip of the whole downlink takes. That
//! is measured rather than assumed, because it depends on how many channels
//! the satellite is sending: three channels and a telemetry packet make it
//! 43, and nothing says it will stay three.
//!
//! The layout of a packet and the strip cadence are from `mlrpt`
//! (dvdesolve/mlrpt, `src/decoder/met_packet.c` and `met_jpg.c`), which
//! reads Meteor off the air.

use crate::ccsds::SpacePacket;
use crate::jpeg;

/// The application ids the instrument's channels are sent on: MSU-MR has six
/// and the satellite sends three of them.
pub const FIRST_IMAGE_APID: u16 = 64;
pub const LAST_IMAGE_APID: u16 = 69;

/// Where the spacecraft's own housekeeping goes, which is not a picture.
pub const TELEMETRY_APID: u16 = 70;

/// Blocks in one packet.
pub const MCUS_PER_PACKET: usize = 14;

/// Blocks across one scan, and so the width of the picture.
pub const MCU_COLUMNS: usize = 196;
pub const WIDTH: usize = MCU_COLUMNS * 8;

/// Rows one strip of blocks is tall.
pub const STRIP_ROWS: usize = 8;

/// Bytes of packet in front of the picture data: eight of secondary header
/// carrying the time, then the block header.
const MCU_HEADER_AT: usize = 8;
const DATA_AT: usize = 14;

/// The channel of the instrument an application id names.
pub fn channel_of(apid: u16) -> Option<u8> {
    (FIRST_IMAGE_APID..=LAST_IMAGE_APID)
        .contains(&apid)
        .then(|| (apid - FIRST_IMAGE_APID + 1) as u8)
}

/// Eight rows of one channel, at the place down the picture they belong.
#[derive(Clone, Debug, PartialEq)]
pub struct Strip {
    pub apid: u16,
    /// Which channel of the instrument, 1 to 6.
    pub channel: u8,
    /// The row of the picture the strip starts at.
    pub first_row: usize,
    /// Eight rows of [`WIDTH`] pixels, one byte each, row major. What was
    /// not received is black.
    pub gray: Vec<u8>,
    /// Blocks painted into it, out of [`MCU_COLUMNS`].
    pub blocks: usize,
    /// The quality factor the last packet of it was quantised with.
    pub quality: u8,
}

impl Strip {
    /// How much of the strip arrived, 0 to 1.
    pub fn completeness(&self) -> f32 {
        self.blocks as f32 / MCU_COLUMNS as f32
    }
}

/// One channel's picture as it is being built.
struct Pane {
    apid: u16,
    channel: u8,
    strip: Vec<u8>,
    blocks: usize,
    quality: u8,
    /// Which strip down the picture is being painted.
    index: usize,
    /// The packet counter the current strip started at.
    started_at: u16,
    /// Packets between the starts of two strips, as measured: the smallest
    /// distance seen, since losing packets can only make it look longer.
    cadence: Option<u32>,
    last_mcu: usize,
}

impl Pane {
    fn new(apid: u16, channel: u8) -> Self {
        Self {
            apid,
            channel,
            strip: vec![0; STRIP_ROWS * WIDTH],
            blocks: 0,
            quality: 0,
            index: 0,
            started_at: 0,
            cadence: None,
            last_mcu: 0,
        }
    }

    fn take(&mut self) -> Option<Strip> {
        if self.blocks == 0 {
            return None;
        }
        let strip = Strip {
            apid: self.apid,
            channel: self.channel,
            first_row: self.index * STRIP_ROWS,
            gray: std::mem::replace(&mut self.strip, vec![0; STRIP_ROWS * WIDTH]),
            blocks: self.blocks,
            quality: self.quality,
        };
        self.blocks = 0;
        Some(strip)
    }
}

/// The packet counter is fourteen bits.
const SEQUENCE_MODULO: u32 = 1 << 14;

/// Pictures out of a run of space packets.
pub struct Receiver {
    panes: Vec<Pane>,
    blocks: jpeg::Blocks,
    /// Packets whose blocks would not read, which is what a frame the
    /// Reed-Solomon corrected wrongly looks like.
    broken: u64,
}

impl Default for Receiver {
    fn default() -> Self {
        Self::new()
    }
}

impl Receiver {
    pub fn new() -> Self {
        Self { panes: Vec::new(), blocks: jpeg::Blocks::new(), broken: 0 }
    }

    /// Packets that were an image packet and did not decode.
    pub fn broken(&self) -> u64 {
        self.broken
    }

    pub fn reset(&mut self) {
        self.panes.clear();
    }

    /// Whatever is half painted, for the end of a pass.
    pub fn flush(&mut self) -> Vec<Strip> {
        self.panes.iter_mut().filter_map(Pane::take).collect()
    }

    /// Read one packet, and hand back the strip it finished, if it finished
    /// one. A strip is finished when the next one starts, since nothing in
    /// the stream says a strip is over.
    pub fn push(&mut self, packet: &SpacePacket) -> Option<Strip> {
        let channel = channel_of(packet.apid)?;
        if packet.payload.len() <= DATA_AT {
            self.broken += 1;
            return None;
        }
        let mcu = usize::from(packet.payload[MCU_HEADER_AT]);
        let quality = packet.payload[MCU_HEADER_AT + 5];
        // A first block past the end of the scan is a broken packet: the
        // column it names would paint outside the picture.
        if mcu + MCUS_PER_PACKET > MCU_COLUMNS {
            self.broken += 1;
            return None;
        }

        let at = match self.panes.iter().position(|p| p.apid == packet.apid) {
            Some(at) => at,
            None => {
                self.panes.push(Pane::new(packet.apid, channel));
                self.panes.len() - 1
            }
        };
        // Where this packet says its strip began, which is where the strip's
        // own first packet would have been counted whether or not it
        // arrived.
        let began =
            packet.sequence.wrapping_sub((mcu / MCUS_PER_PACKET) as u16) % SEQUENCE_MODULO as u16;

        let mut done = None;
        let pane = &mut self.panes[at];
        let new_strip = pane.blocks > 0 && mcu <= pane.last_mcu;
        if new_strip {
            done = pane.take();
            let step = u32::from(began.wrapping_sub(pane.started_at)) % SEQUENCE_MODULO;
            if step > 0 {
                pane.cadence = Some(pane.cadence.map_or(step, |c| c.min(step)));
            }
            // How many strips of the downlink went by, so that strips lost
            // whole leave a gap rather than pulling the picture up.
            let strips = match pane.cadence {
                Some(cadence) if cadence > 0 => {
                    ((step as f64 / cadence as f64).round() as usize).max(1)
                }
                _ => 1,
            };
            pane.index += strips;
        }
        if pane.blocks == 0 {
            pane.started_at = began;
        }
        pane.last_mcu = mcu;
        pane.quality = quality;

        let quant = jpeg::quant_table(quality);
        let mut bits = jpeg::Bits::new(&packet.payload[DATA_AT..]);
        let mut dc = 0i32;
        let mut read = 0;
        for k in 0..MCUS_PER_PACKET {
            let Some(block) = self.blocks.pixels(&mut bits, &quant, &mut dc) else { break };
            let pane = &mut self.panes[at];
            let left = (mcu + k) * 8;
            for row in 0..8 {
                let into = row * WIDTH + left;
                pane.strip[into..into + 8].copy_from_slice(&block[row * 8..row * 8 + 8]);
            }
            pane.blocks += 1;
            read += 1;
        }
        if read < MCUS_PER_PACKET {
            self.broken += 1;
        }
        done
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A run of bits, most significant first, as a transmitter writes them.
    #[derive(Default)]
    struct Writer {
        bytes: Vec<u8>,
        at: usize,
    }

    impl Writer {
        fn push(&mut self, code: u16, length: u8) {
            for k in (0..length).rev() {
                if self.at.is_multiple_of(8) {
                    self.bytes.push(0);
                }
                let bit = (code >> k) & 1;
                let last = self.bytes.len() - 1;
                self.bytes[last] |= (bit as u8) << (7 - self.at % 8);
                self.at += 1;
            }
        }
    }

    /// One image packet: fourteen flat blocks, all at the level `dc` steps
    /// of the quantisation table's first entry name.
    ///
    /// Flat blocks because what is being tested here is the packet layout,
    /// the geometry and where a strip lands, not the transform, which is
    /// settled where the blocks are tested.
    fn mcu_packet(apid: u16, sequence: u16, mcu: usize, quality: u8, dc: i32) -> SpacePacket {
        let dctab = jpeg::Table::luma_dc();
        let actab = jpeg::Table::luma_ac();
        let mut w = Writer::default();
        for k in 0..MCUS_PER_PACKET {
            // The first block carries the whole level; the rest repeat it,
            // which is a category of nought.
            let step = match k {
                0 => dc,
                _ => 0,
            };
            let category = match step {
                0 => 0u8,
                n => (32 - (n.unsigned_abs()).leading_zeros()) as u8,
            };
            let (length, code) = dctab.code_for(category).expect("a DC code");
            w.push(code, length);
            if category > 0 {
                w.push(step as u16, category);
            }
            // End of block: nothing but the DC coefficient.
            let (length, code) = actab.code_for(0x00).expect("an end of block code");
            w.push(code, length);
        }
        let mut payload = vec![0u8; DATA_AT];
        payload[MCU_HEADER_AT] = mcu as u8;
        payload[MCU_HEADER_AT + 5] = quality;
        payload.extend(w.bytes);
        SpacePacket { apid, sequence, payload }
    }

    /// The level a flat block of `dc` steps comes out at: the transform's DC
    /// gain is eight and the level shift is 128.
    fn level(dc: i32, quality: u8) -> u8 {
        let step = jpeg::quant_table(quality)[0];
        ((dc * step) as f64 / 8.0 + 128.0).round().clamp(0.0, 255.0) as u8
    }

    /// A channel's number from its application id, and what is not a
    /// picture at all.
    #[test]
    fn an_application_id_names_a_channel() {
        assert_eq!(channel_of(64), Some(1));
        assert_eq!(channel_of(66), Some(3));
        assert_eq!(channel_of(69), Some(6));
        assert_eq!(channel_of(TELEMETRY_APID), None);
        assert_eq!(channel_of(70), None);
        assert_eq!(channel_of(2047), None);
    }

    /// Fourteen packets are one strip of the whole scan, and the strip is
    /// handed over when the next one starts.
    #[test]
    fn fourteen_packets_make_a_strip() {
        let mut rx = Receiver::new();
        let mut strips = Vec::new();
        for (i, mcu) in (0..MCU_COLUMNS).step_by(MCUS_PER_PACKET).enumerate() {
            let p = mcu_packet(64, i as u16, mcu, 60, 40);
            strips.extend(rx.push(&p));
        }
        assert_eq!(strips.len(), 0, "the strip is not over until the next one starts");
        // The first packet of the next strip, 43 packets after this one
        // started: three channels of fourteen and one telemetry packet.
        strips.extend(rx.push(&mcu_packet(64, 43, 0, 60, 40)));
        assert_eq!(strips.len(), 1);
        let s = &strips[0];
        assert_eq!(s.blocks, MCU_COLUMNS, "every block of the scan");
        assert_eq!(s.completeness(), 1.0);
        assert_eq!(s.first_row, 0);
        assert_eq!(s.gray.len(), STRIP_ROWS * WIDTH);
        assert_eq!(s.channel, 1);
        assert_eq!(s.quality, 60);
        // Every pixel is the level the blocks were sent at.
        let want = level(40, 60);
        assert_eq!(want, 193, "a level of 40 steps at quality 60");
        assert!(s.gray.iter().all(|&p| p.abs_diff(want) <= 1), "the strip is not flat");
        assert_eq!(rx.broken(), 0);
    }

    /// Three channels through one receiver: each keeps its own picture, and
    /// a strip lands at the same row on all three.
    #[test]
    fn three_channels_build_three_pictures() {
        let mut rx = Receiver::new();
        let mut strips = Vec::new();
        let mut sequence = 0u16;
        for strip in 0..4u16 {
            for (channel, apid) in [64u16, 65, 66].iter().enumerate() {
                for mcu in (0..MCU_COLUMNS).step_by(MCUS_PER_PACKET) {
                    let dc = 20 + 10 * channel as i32;
                    strips.extend(rx.push(&mcu_packet(*apid, sequence, mcu, 60, dc)));
                    sequence += 1;
                }
            }
            // The telemetry packet that goes out with every strip, which is
            // what makes the cadence 43 rather than 42.
            sequence += 1;
            let _ = strip;
        }
        strips.extend(rx.flush());
        assert_eq!(strips.len(), 12, "four strips on each of three channels");
        for apid in [64u16, 65, 66] {
            let mine: Vec<&Strip> = strips.iter().filter(|s| s.apid == apid).collect();
            assert_eq!(mine.len(), 4);
            let rows: Vec<usize> = mine.iter().map(|s| s.first_row).collect();
            assert_eq!(rows, vec![0, 8, 16, 24], "channel {apid} landed at {rows:?}");
            assert!(mine.iter().all(|s| s.blocks == MCU_COLUMNS));
        }
        // Each channel came out at its own level, so nothing was painted
        // into the wrong picture.
        for (apid, dc) in [(64u16, 20i32), (65, 30), (66, 40)] {
            let s = strips.iter().find(|s| s.apid == apid).expect("a strip");
            assert_eq!(s.gray[0], level(dc, 60), "channel {apid}");
        }
    }

    /// A whole strip lost: the rows after it land where they belong rather
    /// than pulling the picture up, because the packet counter says how many
    /// went by.
    #[test]
    fn a_lost_strip_leaves_a_gap() {
        let mut rx = Receiver::new();
        let mut strips = Vec::new();
        // Four strips of one channel at the 43 packet cadence, with the
        // third one never received.
        for strip in 0..4u16 {
            if strip == 2 {
                continue;
            }
            for (k, mcu) in (0..MCU_COLUMNS).step_by(MCUS_PER_PACKET).enumerate() {
                let sequence = strip * 43 + k as u16;
                strips.extend(rx.push(&mcu_packet(65, sequence, mcu, 60, 30)));
            }
        }
        strips.extend(rx.flush());
        let rows: Vec<usize> = strips.iter().map(|s| s.first_row).collect();
        assert_eq!(rows, vec![0, 8, 24], "the rows landed at {rows:?}");
    }

    /// Half a strip: what arrived is painted where it belongs and the rest
    /// is black, and the strip says how much of it there is.
    #[test]
    fn a_half_strip_is_painted_where_it_belongs() {
        let mut rx = Receiver::new();
        let mut strips = Vec::new();
        // The second half of the scan only.
        for (k, mcu) in (98..MCU_COLUMNS).step_by(MCUS_PER_PACKET).enumerate() {
            strips.extend(rx.push(&mcu_packet(66, 7 + k as u16, mcu, 60, 50)));
        }
        strips.extend(rx.flush());
        assert_eq!(strips.len(), 1);
        let s = &strips[0];
        assert_eq!(s.blocks, 98);
        assert_eq!(s.completeness(), 0.5);
        let want = level(50, 60);
        assert_eq!(s.gray[0], 0, "the half that never arrived is black");
        assert_eq!(s.gray[WIDTH - 1], want, "and the half that did is the picture");
        assert_eq!(s.gray[98 * 8], want);
        assert_eq!(s.gray[98 * 8 - 1], 0);
    }

    /// A packet whose bytes are not blocks, and one whose column is off the
    /// end of the scan: counted as broken and painted nowhere.
    #[test]
    fn a_broken_packet_paints_nothing() {
        let mut rx = Receiver::new();
        let mut nonsense = mcu_packet(64, 0, 0, 60, 40);
        nonsense.payload[DATA_AT..].iter_mut().for_each(|b| *b = 0xff);
        assert!(rx.push(&nonsense).is_none());
        let mut off_the_end = mcu_packet(64, 1, 0, 60, 40);
        off_the_end.payload[MCU_HEADER_AT] = 190;
        assert!(rx.push(&off_the_end).is_none());
        let short = SpacePacket { apid: 65, sequence: 2, payload: vec![0; 8] };
        assert!(rx.push(&short).is_none());
        assert_eq!(rx.broken(), 3);
        assert_eq!(rx.flush().len(), 0, "nothing was painted");
    }

    /// Housekeeping is not a picture: a telemetry packet is passed over
    /// without being counted as a fault.
    #[test]
    fn telemetry_is_not_a_picture() {
        let mut rx = Receiver::new();
        let p = SpacePacket { apid: TELEMETRY_APID, sequence: 1, payload: vec![0; 30] };
        assert!(rx.push(&p).is_none());
        assert_eq!(rx.broken(), 0);
        assert_eq!(rx.flush().len(), 0);
    }
}
