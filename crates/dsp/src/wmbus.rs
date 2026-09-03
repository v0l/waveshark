//! Wireless M-Bus: the meter radio of EN 13757-4, modes T and C.
//!
//! A meter keys a carrier at 100 kchip/s with about 50 kHz of deviation,
//! sends a preamble of alternating chips long enough for a receiver to find
//! the clock, a sync word that names the mode, and then the frame. Mode T
//! spreads every byte over twelve chips with the 3-of-6 code, three ones in
//! every six chips, so the line stays balanced and any other pattern is a
//! chip error caught at once; mode C sends the bytes as they are. Either
//! way the frame is blocks of at most sixteen bytes each followed by a
//! CRC-16, so a frame that passes is a frame, not a run of noise that
//! happened to start like one.
//!
//! What this produces is the frame's bytes with the CRCs taken out, from
//! the length field onward. What the bytes mean, the manufacturer, the
//! meter's number, the kind of meter, and where the encrypted part starts,
//! is the decode crate's business.
//!
//! Verified against rtl_433's recordings of four meters in mode T: a Diehl
//! and a Techem water meter, a Bernina/BMeters water meter, and an Itron
//! component behind a repeater. Mode C's frame layout is decoded and its
//! CRC checked, but the C recordings do not yet demodulate here: their
//! signal needs a slicer this one does not have, and a frame the receiver
//! cannot recover is not claimed. Mode S, the older 32.768 kchip/s
//! Manchester mode, is not here either: nothing recorded it.

use common::C32;

/// Chip rate of modes T and C.
pub const CHIP_RATE: f64 = 100_000.0;

/// Where meters transmit: mode T and C uplinks on 868.95 MHz, mode S on
/// 868.3 MHz, with room for a source's centre to sit off nominal.
pub fn is_wmbus_band(hz: f64) -> bool {
    (868_850_000.0..=869_050_000.0).contains(&hz) || (868_250_000.0..=868_350_000.0).contains(&hz)
}

/// CRC-16 of EN 13757-4: polynomial 0x3D65, zero initial value, and the
/// result inverted.
pub fn crc16(data: &[u8]) -> u16 {
    let mut crc: u16 = 0;
    for &b in data {
        crc ^= (b as u16) << 8;
        for _ in 0..8 {
            crc = if crc & 0x8000 != 0 { (crc << 1) ^ 0x3D65 } else { crc << 1 };
        }
    }
    crc ^ 0xffff
}

/// The 3-of-6 code: a nibble to six chips holding exactly three ones.
const THREE_OF_SIX: [u8; 16] = [
    0b010110, 0b001101, 0b001110, 0b001011, 0b011100, 0b011001, 0b011010, 0b010011,
    0b101100, 0b100101, 0b100110, 0b100011, 0b110100, 0b110001, 0b110010, 0b101001,
];

/// Six chips back to a nibble, or `None` for a pattern that is not in the
/// code, which is a chip error.
pub fn from_three_of_six(chips: u8) -> Option<u8> {
    THREE_OF_SIX.iter().position(|c| *c == chips).map(|n| n as u8)
}

pub fn to_three_of_six(nibble: u8) -> u8 {
    THREE_OF_SIX[(nibble & 0xf) as usize]
}

/// Which mode a frame arrived in, and in mode C which frame format.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Mode {
    T,
    /// Format A: blocks of sixteen bytes each with a CRC, like mode T.
    CA,
    /// Format B: one CRC after the first block and one after the rest.
    CB,
}

impl Mode {
    pub fn label(&self) -> &'static str {
        match self {
            Mode::T => "T",
            Mode::CA | Mode::CB => "C",
        }
    }
}

/// One frame off the air, its CRCs checked and removed.
#[derive(Clone, Debug, PartialEq)]
pub struct Frame {
    pub mode: Mode,
    /// From the length field onward, without CRCs.
    pub bytes: Vec<u8>,
    /// Sample index, at the demodulator's rate, where the sync word ended.
    pub at: u64,
}

/// The sync words as chips after the preamble. Mode T's is ten chips; mode
/// C's is 0x543D, then 0x54CD for format A or 0x543D again for format B.
const SYNC_T: &[u8] = &[0, 0, 0, 0, 1, 1, 1, 1, 0, 1];
const SYNC_C: &[u8] = &[0, 1, 0, 1, 0, 1, 0, 0, 0, 0, 1, 1, 1, 1, 0, 1];
const FORMAT_A: &[u8] = &[0, 1, 0, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 1];
const FORMAT_B: &[u8] = SYNC_C;
/// Alternating chips required before a sync word, so that noise which
/// happens to spell one is not searched for a frame.
const PREAMBLE_CHIPS: usize = 16;
/// Longest frame: a 255-byte length field, every block with its CRC, in
/// mode T's twelve chips a byte.
const MAX_CHIPS: usize = (256 + 2 * 17) * 12 + 64;

pub struct Demod {
    rate: f64,
    /// Samples per chip.
    sps: f64,
    prev: C32,
    /// Slow mean of the discriminator, which is the carrier offset.
    dc: f32,
    dc_alpha: f32,
    /// Boxcar over half a chip.
    box_len: usize,
    box_ring: Vec<f32>,
    box_pos: usize,
    box_sum: f32,
    /// The run of one sign in progress: its level and its length.
    level: bool,
    run: usize,
    /// Chips decided so far, oldest first, and the sample index of the
    /// first of them.
    chips: Vec<u8>,
    chips_at: u64,
    /// Chip position the sync search resumes from. Everything before it was
    /// tried and is not the start of a frame, which will not change: chips
    /// are only appended. Rescanning from the front on every run of noise
    /// made the search quadratic, and on a wide stream that never closes it
    /// was most of what the decoder cost.
    scanned: usize,
    sample: u64,
    frames: Vec<Frame>,
}

impl Demod {
    /// Needs at least four samples per chip to tell one chip from two.
    pub fn new(rate: f64) -> Self {
        let sps = rate / CHIP_RATE;
        let box_len = (sps / 2.0).round().max(1.0) as usize;
        Self {
            rate,
            sps,
            prev: C32::new(1.0, 0.0),
            dc: 0.0,
            // Twenty chips: the preamble is balanced and longer than that,
            // so the offset is learned before the sync word arrives.
            dc_alpha: 1.0 / (20.0 * sps as f32),
            box_len,
            box_ring: vec![0.0; box_len],
            box_pos: 0,
            box_sum: 0.0,
            level: false,
            run: 0,
            chips: Vec::new(),
            chips_at: 0,
            scanned: 0,
            sample: 0,
            frames: Vec::new(),
        }
    }

    pub fn usable(&self) -> bool {
        self.sps >= 4.0
    }

    pub fn rate(&self) -> f64 {
        self.rate
    }

    pub fn reset(&mut self) {
        self.prev = C32::new(1.0, 0.0);
        self.dc = 0.0;
        self.box_ring.fill(0.0);
        self.box_sum = 0.0;
        self.run = 0;
        self.chips.clear();
        self.scanned = 0;
        self.frames.clear();
    }

    /// Feed samples and return the frames completed by them.
    pub fn process(&mut self, iq: &[C32]) -> &[Frame] {
        self.frames.clear();
        if !self.usable() {
            return &self.frames;
        }
        for &x in iq {
            // Discriminator: the phase step, which is the frequency.
            let f = (x * self.prev.conj()).arg();
            self.prev = x;
            self.dc += self.dc_alpha * (f - self.dc);
            let v = f - self.dc;
            self.box_sum += v - self.box_ring[self.box_pos];
            self.box_ring[self.box_pos] = v;
            self.box_pos = (self.box_pos + 1) % self.box_len;
            let high = self.box_sum > 0.0;
            if high == self.level {
                self.run += 1;
            } else {
                self.end_run();
                self.level = high;
                self.run = 1;
            }
            self.sample += 1;
        }
        // A run still open is not decided yet, but a long one is silence
        // and the chips behind it are either a frame by now or nothing.
        if self.run as f64 > 64.0 * self.sps {
            self.chips.clear();
            self.scanned = 0;
        }
        &self.frames
    }

    /// A run of one level ended: turn it into chips and look for frames.
    fn end_run(&mut self) {
        if self.run == 0 {
            return;
        }
        let n = (self.run as f64 / self.sps).round() as usize;
        if n == 0 {
            // Shorter than half a chip: a glitch, folded into its
            // neighbours by being ignored.
            return;
        }
        if n > 64 {
            // Idle: nothing keyed holds one level this long.
            self.chips.clear();
            self.scanned = 0;
            return;
        }
        if self.chips.is_empty() {
            self.chips_at = self.sample.saturating_sub(self.run as u64);
        }
        let level = self.level as u8;
        self.chips.extend(std::iter::repeat(level).take(n));
        if self.chips.len() > MAX_CHIPS {
            let drop = self.chips.len() - MAX_CHIPS;
            self.chips.drain(..drop);
            self.chips_at += (drop as f64 * self.sps) as u64;
            self.scanned = self.scanned.saturating_sub(drop);
        }
        self.search();
    }

    /// Look for a sync word behind a preamble, and a whole frame after it.
    fn search(&mut self) {
        let chips = &self.chips;
        let mut start = self.scanned.max(PREAMBLE_CHIPS);
        while start + SYNC_C.len() <= chips.len() {
            let pre = &chips[start - PREAMBLE_CHIPS..start];
            let alternating = pre.windows(2).all(|w| w[0] != w[1]);
            if !alternating {
                start += 1;
                continue;
            }
            let rest = &chips[start..];
            let hit = if rest.starts_with(SYNC_T) {
                Self::frame_t(&rest[SYNC_T.len()..]).map(|(f, used)| (f, SYNC_T.len() + used))
            } else if rest.starts_with(SYNC_C) {
                let body = &rest[SYNC_C.len()..];
                if body.len() < FORMAT_A.len().max(FORMAT_B.len()) {
                    // The sync matched and the format word is not all here
                    // yet. This used to fall through as "not a frame", which
                    // the full rescan on the next run quietly repaired; with
                    // the search resuming where it left off it has to wait
                    // here explicitly.
                    Some((None, 0))
                } else if body.starts_with(FORMAT_A) {
                    Self::frame_ca(&body[FORMAT_A.len()..])
                        .map(|(f, used)| (f, SYNC_C.len() + FORMAT_A.len() + used))
                } else if body.starts_with(FORMAT_B) {
                    Self::frame_cb(&body[FORMAT_B.len()..])
                        .map(|(f, used)| (f, SYNC_C.len() + FORMAT_B.len() + used))
                } else {
                    None
                }
            } else {
                None
            };
            match hit {
                Some((Some(mut f), used)) => {
                    f.at = self.chips_at + ((start + used) as f64 * self.sps) as u64;
                    self.frames.push(f);
                    let end = start + used;
                    self.chips.drain(..end);
                    self.chips_at += (end as f64 * self.sps) as u64;
                    self.scanned = 0;
                    return;
                }
                // A sync word whose frame is not all here yet: wait.
                Some((None, _)) => {
                    self.scanned = start;
                    return;
                }
                None => start += 1,
            }
        }
        self.scanned = start;
    }

    /// Mode T: nibbles from 3-of-6 chips, blocks of up to sixteen bytes each
    /// with a CRC. `None` outside means the chips are not a frame; `Some
    /// (None)` means not enough of them yet.
    fn frame_t(chips: &[u8]) -> Option<(Option<Frame>, usize)> {
        let byte_at = |k: usize| -> Option<Option<u8>> {
            let o = k * 12;
            if o + 12 > chips.len() {
                return Some(None);
            }
            let hi = from_three_of_six(pack(&chips[o..o + 6]))?;
            let lo = from_three_of_six(pack(&chips[o + 6..o + 12]))?;
            Some(Some(hi << 4 | lo))
        };
        let l = match byte_at(0)? {
            Some(l) => l as usize,
            None => return Some((None, 0)),
        };
        if l < 9 {
            return None;
        }
        let mut out = Vec::with_capacity(l + 1);
        let mut k = 0usize;
        let mut block = 10usize;
        let mut left = l + 1;
        while left > 0 {
            let n = block.min(left);
            let mut blk = Vec::with_capacity(n);
            for _ in 0..n {
                match byte_at(k)? {
                    Some(b) => blk.push(b),
                    None => return Some((None, 0)),
                }
                k += 1;
            }
            let crc = match (byte_at(k)?, byte_at(k + 1)?) {
                (Some(a), Some(b)) => (a as u16) << 8 | b as u16,
                _ => return Some((None, 0)),
            };
            k += 2;
            if crc16(&blk) != crc {
                return None;
            }
            out.extend_from_slice(&blk);
            left -= n;
            block = 16;
        }
        Some((Some(Frame { mode: Mode::T, bytes: out, at: 0 }), k * 12))
    }

    /// Mode C format A: bytes as sent, blocks and CRCs as mode T.
    fn frame_ca(chips: &[u8]) -> Option<(Option<Frame>, usize)> {
        let byte_at = |k: usize| -> Option<u8> {
            let o = k * 8;
            (o + 8 <= chips.len()).then(|| pack(&chips[o..o + 8]))
        };
        let Some(l) = byte_at(0) else { return Some((None, 0)) };
        let l = l as usize;
        if l < 9 {
            return None;
        }
        let mut out = Vec::with_capacity(l + 1);
        let mut k = 0usize;
        let mut block = 10usize;
        let mut left = l + 1;
        while left > 0 {
            let n = block.min(left);
            let mut blk = Vec::with_capacity(n);
            for _ in 0..n {
                let Some(b) = byte_at(k) else { return Some((None, 0)) };
                blk.push(b);
                k += 1;
            }
            let (Some(a), Some(b)) = (byte_at(k), byte_at(k + 1)) else { return Some((None, 0)) };
            k += 2;
            if crc16(&blk) != (a as u16) << 8 | b as u16 {
                return None;
            }
            out.extend_from_slice(&blk);
            left -= n;
            block = 16;
        }
        Some((Some(Frame { mode: Mode::CA, bytes: out, at: 0 }), k * 8))
    }

    /// Mode C format B: the length is the on-air byte count after itself,
    /// and the block carries a single CRC-16 over everything before it.
    /// Longer frames carry a second block after 126 bytes; none recorded
    /// here reaches that, so it is not handled until one does. The bytes
    /// come out without the CRC, like every other frame.
    fn frame_cb(chips: &[u8]) -> Option<(Option<Frame>, usize)> {
        let byte_at = |k: usize| -> Option<u8> {
            let o = k * 8;
            (o + 8 <= chips.len()).then(|| pack(&chips[o..o + 8]))
        };
        let Some(l) = byte_at(0) else { return Some((None, 0)) };
        let n = l as usize + 1;
        if !(12..=128).contains(&n) {
            return None;
        }
        let mut all = Vec::with_capacity(n);
        for k in 0..n {
            let Some(b) = byte_at(k) else { return Some((None, 0)) };
            all.push(b);
        }
        let crc = (all[n - 2] as u16) << 8 | all[n - 1] as u16;
        if crc16(&all[..n - 2]) != crc {
            return None;
        }
        all.truncate(n - 2);
        Some((Some(Frame { mode: Mode::CB, bytes: all, at: 0 }), n * 8))
    }
}

/// Chips to a number, first chip most significant.
fn pack(chips: &[u8]) -> u8 {
    chips.iter().fold(0u8, |acc, c| acc << 1 | (*c & 1))
}

/// Chips as a mode T transmitter sends them for a frame, for tests and for
/// anything that wants to make one: preamble, sync, then the blocks with
/// their CRCs in 3-of-6.
pub fn chips_t(frame: &[u8]) -> Vec<u8> {
    // Nineteen pairs is the least a meter sends.
    let mut chips: Vec<u8> = (0..40).map(|i| (i % 2) as u8).collect();
    chips.extend_from_slice(SYNC_T);
    let push_byte = |b: u8, chips: &mut Vec<u8>| {
        for nib in [b >> 4, b & 0xf] {
            let c = to_three_of_six(nib);
            for i in (0..6).rev() {
                chips.push((c >> i) & 1);
            }
        }
    };
    let mut block = 10usize;
    let mut at = 0usize;
    while at < frame.len() {
        let n = block.min(frame.len() - at);
        let blk = &frame[at..at + n];
        for &b in blk {
            push_byte(b, &mut chips);
        }
        let crc = crc16(blk);
        push_byte((crc >> 8) as u8, &mut chips);
        push_byte(crc as u8, &mut chips);
        at += n;
        block = 16;
    }
    chips
}

/// Two-tone keying of chips at the chip rate, for tests.
pub fn modulate(chips: &[u8], rate: f64, deviation_hz: f64, amp: f32) -> Vec<C32> {
    let sps = rate / CHIP_RATE;
    let mut out = Vec::with_capacity((chips.len() as f64 * sps) as usize + 1);
    let mut ph = 0.0f64;
    let mut t = 0.0f64;
    for (i, &c) in chips.iter().enumerate() {
        let f = if c == 1 { deviation_hz } else { -deviation_hz };
        while t < (i + 1) as f64 * sps {
            ph += std::f64::consts::TAU * f / rate;
            out.push(C32::new(amp * ph.cos() as f32, amp * ph.sin() as f32));
            t += 1.0;
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_code_is_balanced_and_round_trips() {
        for n in 0..16u8 {
            let c = to_three_of_six(n);
            assert_eq!(c.count_ones(), 3, "{n:x} -> {c:06b}");
            assert_eq!(from_three_of_six(c), Some(n));
        }
        assert_eq!(from_three_of_six(0b111000), None);
    }

    #[test]
    fn the_crc_matches_the_standard() {
        // The first block of rtl_433's Diehl water meter frame, and the CRC
        // that followed it on the air.
        let block = [0x53, 0x44, 0xa5, 0x11, 0x29, 0x01, 0x85, 0x84, 0x76, 0x07];
        assert_eq!(crc16(&block), 0x00cb);
        assert_ne!(crc16(&block), crc16(&block[..9]));
    }

    #[test]
    fn a_synthetic_mode_t_frame_demodulates() {
        let frame: Vec<u8> = (0..0x2a).map(|i| (i * 37 + 11) as u8).collect();
        let mut frame = frame;
        frame[0] = (frame.len() - 1) as u8;
        let rate = 1_000_000.0;
        let chips = chips_t(&frame);
        let mut iq = vec![C32::new(0.0, 0.0); 20_000];
        // Noise, then the burst 3 kHz off the source's centre, then noise.
        let mut seed = 7u64;
        let mut uniform = move || {
            seed ^= seed << 13;
            seed ^= seed >> 7;
            seed ^= seed << 17;
            (seed >> 11) as f64 / (1u64 << 53) as f64
        };
        let mut noise = move || {
            let r = (-2.0 * uniform().max(1e-12).ln()).sqrt();
            let th = std::f64::consts::TAU * uniform();
            C32::new((r * th.cos()) as f32 * 0.02, (r * th.sin()) as f32 * 0.02)
        };
        for x in iq.iter_mut() {
            *x = noise();
        }
        let burst = modulate(&chips, rate, 50_000.0, 0.5);
        let mut ph = 0.0f64;
        for (i, s) in burst.iter().enumerate() {
            ph += std::f64::consts::TAU * 3_000.0 / rate;
            iq[8_000 + i] += s * C32::new(ph.cos() as f32, ph.sin() as f32) + noise();
        }
        let mut d = Demod::new(rate);
        let mut got = Vec::new();
        for block in iq.chunks(1000) {
            got.extend_from_slice(d.process(block));
        }
        assert_eq!(got.len(), 1, "{got:?}");
        assert_eq!(got[0].mode, Mode::T);
        assert_eq!(got[0].bytes, frame);
    }
}
