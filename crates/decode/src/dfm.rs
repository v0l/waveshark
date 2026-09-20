//! Graw DFM radiosondes: the DFM-06, DFM-09 and DFM-17.
//!
//! A DFM says one thing at a time. Every frame carries one configuration
//! channel and two thirteen-nibble data blocks, each block numbered 0 to 8,
//! and a position needs several of them: the latitude is in block 2, the
//! longitude in block 3, the height in block 4 and the date in block 8. So a
//! transmission is gathered rather than parsed, and what [`parse`] reads is
//! the gathering: a [`RECORD`] of the nine data blocks and the sixteen
//! configuration channels as they arrived.
//!
//! Layout, Hamming code, interleave and the meaning of each block are from
//! zilog80's `rs1729/RS`, `demod/mod/dfm09mod.c`, which is the decoder
//! radiosonde_auto_rx runs. The serial number is the fiddly part: a DFM-09
//! or DFM-17 sends it as two sixteen-bit halves on a configuration channel
//! whose own number says which model it is.

use crate::bits::hamming84;
use common::packet::{Entity, Fact, Id, Named, Proto, ThingKind};

/// The frame header, 16 bits, once the Manchester coding is off.
pub const HEADER: u16 = 0x45CF;

/// Bits in one frame: the header, the configuration channel, and two data
/// blocks.
pub const FRAME_BITS: usize = 280;

const CONF_AT: usize = 16;
const CONF_CODEWORDS: usize = 7;
const DAT_AT: [usize; 2] = [16 + 56, 16 + 160];
const DAT_CODEWORDS: usize = 13;

/// Nibbles in a configuration channel and in a data block, which is one
/// nibble per Hamming codeword.
pub const CONF_NIBBLES: usize = CONF_CODEWORDS;
pub const DAT_NIBBLES: usize = DAT_CODEWORDS;

/// Configuration channels a sonde cycles through. The id is a nibble, so
/// there can be sixteen and a sonde uses as many as its sensors need.
pub const CHANNELS: usize = 16;

/// Slots kept per channel. Two, because the serial number arrives as two
/// configuration frames on one channel that differ only in their last
/// nibble, and one slot per channel would keep only the half that came
/// second.
const HALVES: usize = 2;

/// Data blocks, numbered by the nibble at the end of each.
pub const BLOCKS: usize = 9;

/// Bytes in a gathered record: every configuration channel at four bytes,
/// then every data block at seven, each holding its nibbles most significant
/// first with the odd one in the high half of the last byte.
pub const RECORD: usize = CHANNELS * HALVES * 4 + BLOCKS * 7;

const CONF_BYTES: usize = CHANNELS * HALVES * 4;

/// One frame's blocks, after the interleave and the Hamming code.
#[derive(Clone, Debug, PartialEq)]
pub struct Frame {
    /// The configuration channel this frame carried, its own number first.
    pub conf: [u8; CONF_NIBBLES],
    /// The two data blocks, each ending in the block number it is.
    pub dat: [[u8; DAT_NIBBLES]; 2],
    /// Nibbles the Hamming code had to repair.
    pub corrected: u32,
}

impl Frame {
    /// The number of a data block, which is its last nibble.
    pub fn block_id(dat: &[u8; DAT_NIBBLES]) -> u8 {
        dat[12]
    }

    pub fn channel_id(&self) -> u8 {
        self.conf[0]
    }
}

/// Read one frame's bits, header included, into its blocks.
///
/// `None` where the header is not there or a codeword was too broken to
/// repair. Every nibble is protected, so a frame either comes out whole or
/// is not a frame: there is no checksum to fall back on.
pub fn frame(bits: &[bool]) -> Option<Frame> {
    if bits.len() < FRAME_BITS || value(bits, 0, 16) as u16 != HEADER {
        return None;
    }
    let mut corrected = 0;
    let conf = block::<CONF_CODEWORDS>(bits, CONF_AT, &mut corrected)?;
    let dat = [
        block::<DAT_CODEWORDS>(bits, DAT_AT[0], &mut corrected)?,
        block::<DAT_CODEWORDS>(bits, DAT_AT[1], &mut corrected)?,
    ];
    Some(Frame { conf, dat, corrected })
}

/// One interleaved, Hamming-coded section as its data nibbles.
///
/// The interleave is by column: the eight bits of a codeword are `L` apart
/// in the stream, so a fade takes one bit from each of eight codewords
/// rather than eight bits from one, and the code puts all eight back.
fn block<const L: usize>(bits: &[bool], at: usize, corrected: &mut u32) -> Option<[u8; L]> {
    let mut out = [0u8; L];
    for (i, nib) in out.iter_mut().enumerate() {
        let mut code = 0u8;
        for j in 0..8 {
            code = code << 1 | bits[at + L * j + i] as u8;
        }
        let read = hamming84(code)?;
        *corrected += read.corrected as u32;
        *nib = read.nibble;
    }
    Some(out)
}

/// `len` bits from `at`, most significant first.
fn value(bits: &[bool], at: usize, len: usize) -> u32 {
    (0..len).fold(0u32, |v, k| v << 1 | bits[at + k] as u32)
}

/// The same over a block's nibbles, which is how every field in a DFM is
/// addressed: a bit offset into the nibbles, not a byte offset.
fn nib_value(nibs: &[u8], at: usize, len: usize) -> u32 {
    (0..len).fold(0u32, |v, k| {
        let i = at + k;
        v << 1 | (nibs[i / 4] >> (3 - i % 4)) as u32 & 1
    })
}

/// What a sonde has said so far, and the record it becomes.
///
/// A block arrives twice a second and the set of nine takes about two
/// seconds to come round, so this holds what has been heard until block 8
/// closes a set. Blocks are kept by number rather than in order, because the
/// two in a frame are not consecutive and a fade loses whichever it likes.
#[derive(Default)]
pub struct Gather {
    conf: [[u8; CONF_NIBBLES]; CHANNELS * HALVES],
    seen_conf: u32,
    dat: [[u8; DAT_NIBBLES]; BLOCKS],
    seen_dat: u16,
}

/// The blocks a record is not worth emitting without: the frame number and
/// mode, the time, the latitude, the longitude and the height.
const NEEDED: u16 = 0b1_1111;

impl Gather {
    pub fn new() -> Self {
        Self::default()
    }

    /// Take a frame, and hand back a record where that frame completed one.
    ///
    /// Block 8 carries the date and is the last of a set, so it is what
    /// closes the record, and only where the blocks that carry a fix have
    /// arrived since the last one. The data blocks are then forgotten and
    /// the configuration channels are not: the channels cycle far more
    /// slowly, and the serial number is spread over two of them.
    pub fn take(&mut self, f: &Frame) -> Option<Vec<u8>> {
        let slot = f.channel_id() as usize * HALVES + (f.conf[6] as usize & 1);
        self.conf[slot] = f.conf;
        self.seen_conf |= 1 << slot;
        let mut closed = false;
        for dat in &f.dat {
            let id = Frame::block_id(dat) as usize;
            if id >= BLOCKS {
                continue;
            }
            self.dat[id] = *dat;
            self.seen_dat |= 1 << id;
            closed |= id == 8;
        }
        if !closed || self.seen_dat & NEEDED != NEEDED {
            return None;
        }
        let record = self.record();
        self.dat = [[0; DAT_NIBBLES]; BLOCKS];
        self.seen_dat = 0;
        Some(record)
    }

    fn record(&self) -> Vec<u8> {
        let mut out = Vec::with_capacity(RECORD);
        for (i, conf) in self.conf.iter().enumerate() {
            match self.seen_conf >> i & 1 {
                1 => pack(conf, &mut out),
                _ => out.extend([0u8; 4]),
            }
        }
        for dat in &self.dat {
            pack(dat, &mut out);
        }
        out
    }
}

/// Nibbles into bytes, high nibble first, the odd one left in the high half.
fn pack(nibs: &[u8], out: &mut Vec<u8>) {
    for pair in nibs.chunks(2) {
        out.push(pair[0] << 4 | pair.get(1).copied().unwrap_or(0));
    }
}

fn unpack<const N: usize>(bytes: &[u8]) -> [u8; N] {
    let mut nibs = [0u8; N];
    for (i, nib) in nibs.iter_mut().enumerate() {
        *nib = match i % 2 {
            0 => bytes[i / 2] >> 4,
            _ => bytes[i / 2] & 0xF,
        };
    }
    nibs
}

/// Which instrument sent a record, read from the configuration channel its
/// serial number arrived on. The nibble is the model, which is why a DFM
/// names itself without saying so anywhere else.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
pub enum Model {
    Dfm06,
    Dfm09,
    Dfm09p,
    Dfm17,
    Dfm17p,
    /// A pilot sonde: position only, no weather sensors.
    Ps15,
    #[default]
    Unknown,
}

impl Model {
    fn from_channel(ch: u8) -> Model {
        match ch {
            0x7 | 0x8 => Model::Ps15,
            0xA => Model::Dfm09,
            0xB => Model::Dfm17,
            0xC => Model::Dfm09p,
            0xD => Model::Dfm17p,
            _ => Model::Unknown,
        }
    }

    pub fn label(self) -> &'static str {
        match self {
            Model::Dfm06 => "DFM-06",
            Model::Dfm09 => "DFM-09",
            Model::Dfm09p => "DFM-09P",
            Model::Dfm17 => "DFM-17",
            Model::Dfm17p => "DFM-17P",
            Model::Ps15 => "PS-15",
            Model::Unknown => "DFM",
        }
    }
}

/// How the data blocks are laid out, which the sonde states in block 0 and
/// which moves every field when it changes.
///
/// The older firmwares put the time in block 1 and the position in 2, 3 and
/// 4; the newer ones start at block 0 and carry a second fix further down,
/// and the XDATA one uses the blocks past 3 for an instrument hanging off
/// the sonde rather than for the sonde itself.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Layout {
    Classic,
    Newer,
    Xdata,
}

impl Layout {
    fn of(mode: u8) -> Layout {
        match mode {
            3 => Layout::Newer,
            4 => Layout::Xdata,
            _ => Layout::Classic,
        }
    }
}

/// What a gathered record says.
#[derive(Clone, Debug, PartialEq)]
pub struct Report {
    pub model: Model,
    /// The serial as it is printed on the sonde, or empty where the
    /// configuration channels carrying it have not both arrived.
    pub serial: String,
    pub frame_no: u32,
    pub layout: Layout,
    pub lat_deg: f64,
    pub lon_deg: f64,
    /// Height above the ellipsoid on the older firmwares and above mean sea
    /// level on the newer ones, which is what the sonde itself reports.
    pub altitude_m: f64,
    pub speed_kt: f64,
    pub course_deg: f64,
    pub climb_ms: f64,
    /// Satellites in the fix, where the sonde said.
    pub satellites: u8,
    /// Seconds into the UTC day.
    pub sec_of_day: f64,
    /// Year, month, day, hour and minute from block 8, where it was filed.
    pub date: Option<(u16, u8, u8, u8, u8)>,
}

impl Report {
    pub fn has_position(&self) -> bool {
        self.lat_deg != 0.0 || self.lon_deg != 0.0
    }

    pub fn summary(&self) -> String {
        let who = match self.serial.is_empty() {
            true => self.model.label().to_string(),
            false => format!("{} {}", self.model.label(), self.serial),
        };
        match self.has_position() {
            true => format!(
                "{who} {:.5}, {:.5} at {:.0} m, {:+.1} m/s",
                self.lat_deg, self.lon_deg, self.altitude_m, self.climb_ms
            ),
            false => format!("{who} frame {}", self.frame_no),
        }
    }
}

/// Read a gathered record.
///
/// `None` where the bytes are not one: every data block ends in its own
/// number and every configuration channel begins with its own, which is nine
/// and up to sixteen checks that no other protocol's frame will pass.
pub fn parse(record: &[u8]) -> Option<Report> {
    if record.len() != RECORD {
        return None;
    }
    let conf: Vec<[u8; CONF_NIBBLES]> =
        (0..CHANNELS * HALVES).map(|c| unpack::<CONF_NIBBLES>(&record[c * 4..c * 4 + 4])).collect();
    let dat: Vec<[u8; DAT_NIBBLES]> = (0..BLOCKS)
        .map(|b| unpack::<DAT_NIBBLES>(&record[CONF_BYTES + b * 7..CONF_BYTES + b * 7 + 7]))
        .collect();
    for (slot, nibs) in conf.iter().enumerate() {
        let empty = nibs.iter().all(|n| *n == 0);
        if !empty && nibs[0] as usize != slot / HALVES {
            return None;
        }
    }
    for (b, nibs) in dat.iter().enumerate() {
        if nibs[12] as usize != b {
            return None;
        }
    }

    let mode = nib_value(&dat[0], 16, 8) as u8;
    let layout = Layout::of(mode);
    let (model, serial) = identity(&conf);
    let mut r = Report {
        model,
        serial,
        frame_no: nib_value(&dat[0], 24, 8),
        layout,
        lat_deg: 0.0,
        lon_deg: 0.0,
        altitude_m: 0.0,
        speed_kt: 0.0,
        course_deg: 0.0,
        climb_ms: 0.0,
        satellites: nib_value(&dat[8], 32, 8) as u8,
        sec_of_day: 0.0,
        date: None,
    };

    // Metres a second across the ground, as the sonde sends it, read as
    // knots because that is what a track carries.
    let kt = |ms: f64| ms * 1.943_844;
    match layout {
        Layout::Classic => {
            r.sec_of_day = nib_value(&dat[1], 32, 16) as f64 / 1000.0;
            r.lat_deg = nib_value(&dat[2], 0, 32) as i32 as f64 / 1e7;
            r.speed_kt = kt(nib_value(&dat[2], 32, 16) as i16 as f64 / 100.0);
            r.lon_deg = nib_value(&dat[3], 0, 32) as i32 as f64 / 1e7;
            r.course_deg = nib_value(&dat[3], 32, 16) as f64 / 100.0;
            r.altitude_m = nib_value(&dat[4], 0, 32) as i32 as f64 / 100.0;
            r.climb_ms = nib_value(&dat[4], 32, 16) as i16 as f64 / 100.0;
        }
        Layout::Newer | Layout::Xdata => {
            r.sec_of_day = nib_value(&dat[0], 0, 16) as f64 / 1000.0;
            r.speed_kt = kt(nib_value(&dat[0], 32, 16) as i16 as f64 / 100.0);
            r.lat_deg = nib_value(&dat[1], 0, 32) as i32 as f64 / 1e7;
            r.course_deg = nib_value(&dat[1], 32, 16) as f64 / 100.0;
            r.lon_deg = nib_value(&dat[2], 0, 32) as i32 as f64 / 1e7;
            r.climb_ms = nib_value(&dat[2], 32, 16) as i16 as f64 / 100.0;
            r.altitude_m = nib_value(&dat[3], 0, 32) as i32 as f64 / 100.0;
        }
    }

    let year = nib_value(&dat[8], 0, 12) as u16;
    if year > 2000 {
        r.date = Some((
            year,
            nib_value(&dat[8], 12, 4) as u8,
            nib_value(&dat[8], 16, 5) as u8,
            nib_value(&dat[8], 21, 5) as u8,
            nib_value(&dat[8], 26, 6) as u8,
        ));
    }
    Some(r)
}

/// Which sonde this is, from the configuration channels.
///
/// A DFM-09 or DFM-17 sends its serial on one channel as two sixteen-bit
/// halves: the channel's own number is the model, the byte after it is
/// `0xsC` or `0xs0` with `s` the same number again, and the last nibble of
/// the five that follow says which half this is. Both halves are needed, so
/// a sonde names itself a second or two after it is first heard.
///
/// A DFM-06 numbers itself differently and is not read here: they have all
/// but left the air, and guessing at one would put a wrong serial on a
/// track.
fn identity(conf: &[[u8; CONF_NIBBLES]]) -> (Model, String) {
    let mut halves: [Option<u16>; 2] = [None, None];
    let mut model = Model::Unknown;
    for (slot, nibs) in conf.iter().enumerate() {
        let channel = slot / HALVES;
        if channel < 6 || nibs[0] as usize != channel {
            continue;
        }
        if !matches!(nibs[1], 0xC | 0x0) {
            continue;
        }
        let half = nibs[6] as usize;
        if half > 1 {
            continue;
        }
        model = Model::from_channel(nibs[0]);
        halves[half] = Some(
            ((nibs[2] as u16) << 12)
                | ((nibs[3] as u16) << 8)
                | ((nibs[4] as u16) << 4)
                | nibs[5] as u16,
        );
    }
    match (halves[0], halves[1]) {
        (Some(hi), Some(lo)) => (model, format!("{}", (u32::from(hi) << 16) | u32::from(lo))),
        _ => (model, String::new()),
    }
}

/// What a Graw DFM frame says.
///
/// The serial is what a chaser follows and what SondeHub files a flight
/// under; a sonde that has not sent both halves of it yet is still a sonde,
/// so it is reported without an identity rather than under a made-up one.
pub fn read(bytes: &[u8]) -> Option<Proto> {
    let r = parse(bytes)?;
    let mut p = Proto::new("dfm", "frame");
    if !r.serial.is_empty() {
        p = p
            .by(Entity::new("graw", Id::Text(r.serial.clone())).made_by("Graw"))
            .saying(Fact::Named(Named::new(r.serial.clone(), ThingKind::Sonde)));
    }
    if r.has_position() {
        for fact in crate::facts::of_flight(
            r.lat_deg,
            r.lon_deg,
            r.altitude_m,
            r.climb_ms,
            r.speed_kt,
            r.course_deg,
            None,
        ) {
            p = p.saying(fact);
        }
    }
    Some(p)
}

/// Chips in one frame.
pub const FRAME_CHIPS: usize = FRAME_BITS * 2;

/// Chips held while looking for a header: three frames, so a frame
/// straddling two blocks is never lost and a quiet channel cannot grow.
pub const MAX_CHIPS: usize = FRAME_CHIPS * 3;

/// Chips of the header allowed to be wrong. The header is 32 chips and
/// every nibble behind it is protected, so a false header costs one Hamming
/// failure and nothing else.
pub const HEADER_SLACK: u32 = 4;

/// Whether the header sits at `at`, and which way up it is. `Some(true)`
/// where the chips are inverted, which is how a DFM-06 and a DFM-09 differ.
pub fn matches(chips: &[bool], at: usize, header: &[bool]) -> Option<bool> {
    let mut wrong = [0u32; 2];
    for (k, &want) in header.iter().enumerate() {
        let got = chips[at + k];
        wrong[(got == want) as usize] += 1;
        if wrong[0] > HEADER_SLACK && wrong[1] > HEADER_SLACK {
            return None;
        }
    }
    match (wrong[0] <= HEADER_SLACK, wrong[1] <= HEADER_SLACK) {
        (true, _) => Some(false),
        (_, true) => Some(true),
        _ => None,
    }
}

/// The header as it arrives: every bit as two chips, `01` for a one.
pub fn header_chips() -> Vec<bool> {
    (0..16)
        .flat_map(|k| {
            let bit = HEADER >> (15 - k) & 1 != 0;
            [!bit, bit]
        })
        .collect()
}

/// Manchester chips back to bits. The second chip of each pair is the bit,
/// and a pair that is not a transition is left to the Hamming code: half a
/// wrong pair is one wrong bit, which is what that code is for.
pub fn manchester(chips: &[bool], inverted: bool) -> Vec<bool> {
    chips.chunks(2).map(|c| c[c.len() - 1] != inverted).collect()
}

/// Frames cut out of a stream of chips.
///
/// Above the waveform and below the payload: the chips come from any FSK
/// demodulator at this sonde's baud, and what leaves is what the payload
/// reader takes.
pub struct Framer {
    chips: Vec<bool>,
    /// Frames whose every nibble came through the Hamming code.
    frames: u64,
    /// Records gathered, which is what a reader takes.
    records: u64,
    /// Positions already searched and known not to start a header.
    scanned: usize,
    gather: Gather,
}

impl Default for Framer {
    fn default() -> Self {
        Self::new()
    }
}

impl Framer {
    pub fn new() -> Self {
        Self { chips: Vec::new(), scanned: 0, frames: 0, records: 0, gather: Gather::new() }
    }

    /// The buffer a bit clock appends into.
    pub fn sink(&mut self) -> &mut Vec<bool> {
        &mut self.chips
    }

    /// Frames whose every nibble came through the Hamming code.
    pub fn frames(&self) -> u64 {
        self.frames
    }

    /// Records gathered out of those frames.
    pub fn records(&self) -> u64 {
        self.records
    }

    /// Look for headers in the chips held, returning every record a frame
    /// behind one completed.
    pub fn take(&mut self) -> Vec<Vec<u8>> {
        let mut out = Vec::new();
        let header = header_chips();
        let mut at = self.scanned;
        while at + header.len() <= self.chips.len() {
            let Some(inverted) = matches(&self.chips, at, &header) else {
                at += 1;
                continue;
            };
            if at + FRAME_CHIPS > self.chips.len() {
                // Not enough of the frame has arrived. Waiting here rather
                // than walking past it is what keeps a frame that straddles
                // two blocks.
                self.scanned = at;
                return out;
            }
            let bits = manchester(&self.chips[at..at + FRAME_CHIPS], inverted);
            match frame(&bits) {
                Some(frame) => {
                    self.frames += 1;
                    if let Some(record) = self.gather.take(&frame) {
                        self.records += 1;
                        out.push(record);
                    }
                    self.chips.drain(..at + FRAME_CHIPS);
                    at = 0;
                    self.scanned = 0;
                }
                None => at += 1,
            }
        }
        self.scanned = at;
        out
    }

    /// Drop what has been searched and found wanting.
    pub fn trim(&mut self) {
        if self.chips.len() > MAX_CHIPS {
            let drop = self.chips.len() - MAX_CHIPS;
            self.chips.drain(..drop);
            self.scanned = self.scanned.saturating_sub(drop);
        }
    }

    pub fn reset(&mut self) {
        self.chips.clear();
        self.scanned = 0;
        self.gather = Gather::new();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Encode a nibble the way the sonde does, so a frame can be built here
    /// and read back: four data bits, then the four parity sums of the
    /// check matrix.
    fn encode(nib: u8) -> u8 {
        let rows: [u8; 4] = [0b0111_0000, 0b1011_0000, 0b1101_0000, 0b1110_0000];
        let mut code = nib << 4;
        for (i, row) in rows.iter().enumerate() {
            code |= ((row & code).count_ones() as u8 & 1) << (3 - i);
        }
        code
    }

    /// One section's nibbles, interleaved and coded, written into `bits`.
    fn put(bits: &mut [bool], at: usize, nibs: &[u8]) {
        let l = nibs.len();
        for (i, nib) in nibs.iter().enumerate() {
            let code = encode(*nib);
            for j in 0..8 {
                bits[at + l * j + i] = code >> (7 - j) & 1 != 0;
            }
        }
    }

    fn nibbles(v: u64, count: usize) -> Vec<u8> {
        (0..count).map(|i| (v >> (4 * (count - 1 - i))) as u8 & 0xF).collect()
    }

    /// A data block: twelve nibbles of payload and the block number.
    fn dat(id: u8, payload: u64) -> [u8; DAT_NIBBLES] {
        let mut nibs = [0u8; DAT_NIBBLES];
        for (i, n) in nibbles(payload, 12).into_iter().enumerate() {
            nibs[i] = n;
        }
        nibs[12] = id;
        nibs
    }

    fn frame_bits(
        conf: [u8; CONF_NIBBLES],
        a: [u8; DAT_NIBBLES],
        b: [u8; DAT_NIBBLES],
    ) -> Vec<bool> {
        let mut bits = vec![false; FRAME_BITS];
        for (k, bit) in bits.iter_mut().take(16).enumerate() {
            *bit = HEADER >> (15 - k) & 1 != 0;
        }
        put(&mut bits, CONF_AT, &conf);
        put(&mut bits, DAT_AT[0], &a);
        put(&mut bits, DAT_AT[1], &b);
        bits
    }

    /// A serial channel: its own number, the 0xsC marker, four nibbles of
    /// the half, and which half it is.
    fn serial_channel(ch: u8, half: u8, value: u16) -> [u8; CONF_NIBBLES] {
        let mut nibs = [0u8; CONF_NIBBLES];
        nibs[0] = ch;
        nibs[1] = 0xC;
        for (i, n) in nibbles(u64::from(value), 4).into_iter().enumerate() {
            nibs[2 + i] = n;
        }
        nibs[6] = half;
        nibs
    }

    /// A whole transmission of a DFM-17 over the Irish Sea, built block by
    /// block and read back: the interleave, the Hamming code, the gathering
    /// across frames, and the fields of the classic layout.
    #[test]
    fn a_gathered_transmission_reads_as_a_fix() {
        let mut g = Gather::new();
        // The numbers a sonde would be sending over the Irish Sea, in the
        // units it sends them in: degrees at 1e-7, metres and metres a
        // second at 1e-2, the heading in hundredths of a degree.
        let (lat, lon, alt) = (533_500_000i32, -50_000_000i32, 471_222i32);
        let (ground_cms, climb_cms, course_cdeg) = (900i16, 500i16, 20_000u16);
        let (frame_no, mode, msek) = (0x71u64, 2u64, 40_000u64);
        let u32b = |v: i32| v as u32 as u64;
        let blocks = [
            dat(0, mode << 24 | frame_no << 16),
            dat(1, msek),
            dat(2, u32b(lat) << 16 | ground_cms as u64),
            dat(3, u32b(lon) << 16 | course_cdeg as u64),
            dat(4, u32b(alt) << 16 | climb_cms as u64),
            dat(5, 0),
            dat(6, 0),
            dat(7, 0),
            // Year, month, day, hour, minute, then the satellite count.
            dat(8, 0x7E9 << 36 | 3 << 32 | 13 << 27 | 5 << 22 | 42 << 16 | 11 << 8),
        ];
        // The serial arrives on channel 0xB, which is what says DFM-17.
        let conf = [serial_channel(0xB, 0, 0x0163), serial_channel(0xB, 1, 0x4A21)];
        let mut record = None;
        for (i, pair) in blocks.chunks(2).enumerate() {
            let b = pair.get(1).copied().unwrap_or(pair[0]);
            let bits = frame_bits(conf[i.min(1)], pair[0], b);
            let f = frame(&bits).expect("a frame");
            assert_eq!(f.corrected, 0, "a frame built here needed correcting");
            record = g.take(&f).or(record);
        }
        let record = record.expect("block 8 closes a record");
        assert_eq!(record.len(), RECORD);

        let r = parse(&record).expect("a report");
        assert_eq!(r.model, Model::Dfm17);
        assert_eq!(r.serial, format!("{}", 0x0163_4A21u32));
        assert_eq!(r.layout, Layout::Classic);
        assert_eq!(r.frame_no, 0x71);
        assert!((r.lat_deg - 53.35).abs() < 1e-6, "{}", r.lat_deg);
        assert!((r.lon_deg + 5.0).abs() < 1e-6, "{}", r.lon_deg);
        assert!((r.altitude_m - 4_712.22).abs() < 0.01, "{}", r.altitude_m);
        assert!((r.climb_ms - 5.0).abs() < 0.01, "{}", r.climb_ms);
        // Nine metres a second across the ground is 17.5 knots.
        assert!((r.speed_kt - 17.49).abs() < 0.05, "{}", r.speed_kt);
        assert!((r.course_deg - 200.0).abs() < 0.01, "{}", r.course_deg);
        assert_eq!(r.sec_of_day, 40.0);
        assert_eq!(r.date, Some((2025, 3, 13, 5, 42)));
        assert_eq!(r.satellites, 11);
        assert!(r.has_position());
    }

    /// A record is not emitted until the blocks that carry a fix have all
    /// arrived, so a sonde heard from block 7 onwards does not produce a
    /// position of nought degrees off the coast of Africa.
    #[test]
    fn a_partial_set_produces_nothing() {
        let mut g = Gather::new();
        for id in [6u8, 7, 8] {
            let bits = frame_bits([0; CONF_NIBBLES], dat(id, 0), dat(id, 0));
            let f = frame(&bits).expect("a frame");
            assert_eq!(g.take(&f), None, "block {id} closed a record on its own");
        }
    }

    /// Anything that is not a record is refused: every block has to end in
    /// its own number, which is what keeps another protocol's frame of the
    /// same length from being read as a sonde.
    #[test]
    fn a_record_is_recognised_by_its_block_numbers() {
        assert_eq!(parse(&[0u8; RECORD]).map(|r| r.frame_no), None);
        assert_eq!(parse(&[0u8; RECORD - 1]), None);
    }

    /// A single wrong bit anywhere in a frame is put back by the code, and
    /// the frame says it happened.
    #[test]
    fn one_wrong_bit_is_repaired() {
        let mut bits =
            frame_bits(serial_channel(0xA, 0, 0x1234), dat(2, 0x1234_5678_9ABC), dat(3, 0));
        bits[100] = !bits[100];
        let f = frame(&bits).expect("a frame");
        assert_eq!(f.corrected, 1);
        assert_eq!(Frame::block_id(&f.dat[0]), 2);
        assert_eq!(f.dat[0][..12], nibbles(0x1234_5678_9ABC, 12)[..]);
    }
}
