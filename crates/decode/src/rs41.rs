//! Vaisala RS41 radiosonde frames: bytes in, a weather balloon's report out.
//!
//! A sonde sends one frame a second on a channel between 400 and 406 MHz.
//! Behind the header the whole frame is XORed with a repeating 64-byte mask
//! ([`whiten::vaisala`]), then protected by two interleaved RS(255,231)
//! codewords over GF(256), and only then does it hold anything: a run of
//! typed blocks, each `id`, `len`, `len` bytes and a CRC-16.
//!
//! Three of those blocks are worth reading here. `0x79` names the sonde and
//! counts its frames, `0x7B` carries the position the u-blox receiver
//! computed in earth-centred coordinates, and `0x7C` carries the GPS week and
//! time of week the fix was taken at. The rest are raw pseudorange
//! observations and uncalibrated sensor counts, which need the calibration
//! subframe the sonde spreads over 51 frames before they mean anything.
//!
//! Field offsets follow Vaisala's frame as reverse engineered by Josef
//! Bazant (`bazjo/RS41_Decoding`) and by zilog80 (`rs1729/RS`, `rs41/rs41.txt`);
//! the tests are the two frames published in the latter, with the serials and
//! the position they were stated to decode to.

use crate::bits::crc16;
use crate::geo::{ecef_to_geodetic, ecef_velocity_to_enu};
use crate::rs::ReedSolomon;
use crate::whiten;
use common::Decoded;

/// Header the sonde keys before anything else, as it arrives, least
/// significant bit first. This is what a receiver correlates against: the
/// scrambler covers the header too, so the constant on the air and the
/// constant in a parsed frame are two different eight-byte strings.
pub const HEADER_AIR: [u8; 8] = [0x10, 0xB6, 0xCA, 0x11, 0x22, 0x96, 0x12, 0xF8];

/// The same header once the scrambler is off, which is what [`parse`]
/// checks.
pub const HEADER: [u8; 8] = [0x86, 0x35, 0xF4, 0x40, 0x93, 0xDF, 0x1A, 0x60];

/// Bytes before the Reed-Solomon parity, which is the header alone.
const PARITY_AT: usize = HEADER.len();
/// Parity symbols, 24 for each of the two interleaved codewords.
const PARITY_LEN: usize = 48;
/// Where the data the blocks live in starts, which is also where the length
/// marker is.
pub const DATA_AT: usize = PARITY_AT + PARITY_LEN;

/// The whole frame, in the two lengths a sonde sends.
///
/// The length is declared by the first data byte rather than by anything a
/// receiver measures: `0x0F` for the standard frame and `0xF0` for the long
/// one, which carries the extra `0x7E` block of ground-station text.
pub const FRAME_STD: usize = 320;
pub const FRAME_AUX: usize = 518;

const LEN_STD: u8 = 0x0F;
const LEN_AUX: u8 = 0xF0;

/// Frame length in bytes, given the byte at [`DATA_AT`].
pub fn frame_len(marker: u8) -> Option<usize> {
    match marker {
        LEN_STD => Some(FRAME_STD),
        LEN_AUX => Some(FRAME_AUX),
        _ => None,
    }
}

/// CRC-16/CCITT-FALSE, which every block inside a frame carries, sent little
/// endian.
pub fn block_crc(data: &[u8]) -> u16 {
    crc16(data, 0x1021, 0xFFFF)
}

/// Take the scrambler off a whole frame, header included.
///
/// The header is inside the scrambled region, and descrambles to a different
/// constant rather than to nothing. That is why there are two of them: a
/// receiver correlates [`HEADER_AIR`] in the bit stream and a parser checks
/// [`HEADER`] in what comes out.
pub fn descramble(frame: &mut [u8]) {
    whiten::vaisala(frame, 0);
}

/// The code the sonde protects a frame with: RS(255,231) over GF(256) with
/// field polynomial 0x11D and the 24 roots from alpha^0, shortened to
/// however many data symbols this frame length gives each codeword.
fn code(data_len: usize) -> ReedSolomon {
    let k = data_len / 2;
    ReedSolomon::new(8, 0x11D, 0, 1, 24, 255 - 24 - k)
}

/// Correct a descrambled frame in place, returning how many symbols the code
/// changed, or `None` where either codeword is beyond it.
///
/// The frame's data bytes alternate between the two codewords, and each
/// codeword holds them in reverse: the sonde sends the lowest-degree
/// coefficient first, where this decoder, like Karn's, takes the highest
/// first. The parity runs the same way about, the second codeword's 24
/// symbols first.
pub fn correct(frame: &mut [u8]) -> Option<usize> {
    if frame.len() <= DATA_AT {
        return None;
    }
    let n = frame.len() - DATA_AT;
    let k = n / 2;
    let rs = code(n);
    let mut fixed = 0;
    for c in 0..2 {
        let mut block = Vec::with_capacity(k + 24);
        for i in (0..k).rev() {
            block.push(frame[DATA_AT + 2 * i + c]);
        }
        for j in 0..24 {
            block.push(frame[PARITY_AT + (c + 1) * 24 - 1 - j]);
        }
        fixed += rs.decode(&mut block, &[])?;
        for i in 0..k {
            frame[DATA_AT + 2 * i + c] = block[k - 1 - i];
        }
        for j in 0..24 {
            frame[PARITY_AT + (c + 1) * 24 - 1 - j] = block[k + j];
        }
    }
    Some(fixed)
}

/// Write the parity for a descrambled frame whose data is already in place,
/// which is what a test that makes a frame needs.
pub fn protect(frame: &mut [u8]) {
    let n = frame.len() - DATA_AT;
    let k = n / 2;
    let rs = code(n);
    for c in 0..2 {
        let data: Vec<u8> = (0..k).rev().map(|i| frame[DATA_AT + 2 * i + c]).collect();
        let parity = rs.encode(&data);
        for j in 0..24 {
            frame[PARITY_AT + (c + 1) * 24 - 1 - j] = parity[j];
        }
    }
}

/// One typed block inside a frame.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Block {
    /// Frame counter, serial, battery, flight state.
    Status,
    /// Uncalibrated temperature, humidity and pressure counts.
    Meas,
    /// Position and velocity, earth-centred.
    GpsPos,
    /// GPS week, time of week, and which satellites were tracked.
    GpsInfo,
    /// Raw pseudorange and doppler observations.
    GpsRaw,
    /// Padding.
    Empty,
    /// Text the ground station put in the sonde before launch.
    Xdata,
    /// The encrypted block a military sonde sends instead of the GPS ones.
    Crypto,
    Other(u8),
}

impl Block {
    pub fn from_id(id: u8) -> Self {
        match id {
            0x79 => Block::Status,
            0x7A | 0x7F => Block::Meas,
            0x7B => Block::GpsPos,
            0x7C => Block::GpsInfo,
            0x7D => Block::GpsRaw,
            0x76 => Block::Empty,
            0x7E => Block::Xdata,
            0x80 => Block::Crypto,
            other => Block::Other(other),
        }
    }
}

/// Whether the sonde's GPS and sensor blocks are readable at all.
///
/// A military RS41-SGM can be configured to encrypt them, and then the only
/// honest thing to report is that a sonde is there and enciphered.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum Crypto {
    #[default]
    Standard,
    /// Unencrypted, but keyed as a military sonde: all three GPS blocks.
    Military,
    Encrypted,
    Unknown(u8),
}

impl Crypto {
    fn from_byte(b: u8) -> Self {
        match b {
            0 => Crypto::Standard,
            1 | 2 => Crypto::Military,
            3 | 4 => Crypto::Encrypted,
            other => Crypto::Unknown(other),
        }
    }

    pub fn encrypted(&self) -> bool {
        matches!(self, Crypto::Encrypted)
    }
}

/// Where the sonde is in its flight, as the status block's bit field says.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum Flight {
    #[default]
    Ground,
    Ascent,
    Descent,
}

impl Flight {
    pub fn label(&self) -> &'static str {
        match self {
            Flight::Ground => "on the ground",
            Flight::Ascent => "ascending",
            Flight::Descent => "descending",
        }
    }
}

/// What one frame said.
#[derive(Clone, Debug, PartialEq, Default)]
pub struct Frame {
    /// Vaisala serial, eight printable characters: "K1930293".
    pub serial: String,
    /// Frames since the sonde was switched on, which is its age in seconds.
    pub frame_no: u16,
    pub battery_v: f32,
    /// Temperature of the reference cut-out on the board, in degrees.
    pub pcb_temp_c: i8,
    pub flight: Flight,
    pub crypto: Crypto,
    /// Transmit power setting, 0 to 7.
    pub tx_power: u8,
    pub lat_deg: f64,
    pub lon_deg: f64,
    /// Height above the WGS84 ellipsoid, in metres. Not above the geoid: the
    /// sonde reports what the receiver solved for, and the difference is tens
    /// of metres in Europe.
    pub altitude_m: f64,
    /// Over the ground, in knots, which is the unit the map wants.
    pub speed_kt: f64,
    pub course_deg: f64,
    /// Positive upward, in metres a second.
    pub climb_ms: f64,
    pub satellites: u8,
    /// GPS week and time of week in milliseconds, where the frame carried
    /// them. The week is the full count since 6 January 1980.
    pub gps_week: Option<u16>,
    pub gps_tow_ms: Option<u32>,
    /// The twelve 24-bit counts the sensor block carries, in the order it
    /// sends them: the temperature sensor and its two references, the
    /// humidity sensor and its two, the humidity sensor's own thermometer
    /// and its two, then the pressure sensor and its two. They are ratios
    /// against reference resistors and capacitors and mean nothing without
    /// the calibration the sonde spreads over 51 frames; see [`Calibration`].
    pub meas: Option<[u32; 12]>,
    /// One sixteenth of the calibration, and which sixteenth it is.
    pub subframe: Option<(u8, [u8; SUBFRAME_PIECE])>,
    /// How many pieces the sonde says there are, which is 50 on every one
    /// seen so far, the 51st being the run-time block.
    pub subframe_max: u8,
    /// Blocks that were present and passed their CRC.
    pub blocks: Vec<Block>,
    /// A block whose CRC failed. One bad block does not throw the frame
    /// away, because the others are checked separately.
    pub bad_blocks: usize,
}

impl Frame {
    /// Whether the frame said where the sonde is. A frame with no GPS block,
    /// or one whose GPS block failed its CRC, has not.
    pub fn has_position(&self) -> bool {
        self.blocks.contains(&Block::GpsPos)
    }

    /// One line for a list: who it is, where, and how high.
    pub fn summary(&self) -> String {
        if self.crypto.encrypted() {
            return format!("{} frame {} (encrypted)", self.serial, self.frame_no);
        }
        if !self.has_position() {
            return format!("{} frame {}, no fix", self.serial, self.frame_no);
        }
        format!(
            "{} {:.5} {:.5} {:.0} m {}",
            self.serial,
            self.lat_deg,
            self.lon_deg,
            self.altitude_m,
            self.flight.label()
        )
    }
}

fn le16(b: &[u8]) -> u16 {
    u16::from_le_bytes([b[0], b[1]])
}

fn le32(b: &[u8]) -> u32 {
    u32::from_le_bytes([b[0], b[1], b[2], b[3]])
}

/// Bits held while looking for a header. Two long frames and the gap
/// between them: enough that a frame straddling two blocks is never lost,
/// and bounded so a channel with nothing on it cannot grow.
const MAX_BITS: usize = FRAME_AUX * 8 * 3;

/// Header bit errors tolerated. The header is 64 bits and a sonde at the
/// edge of reception loses a few; more than this and it is not a header.
/// Four leaves a false alarm rate of about one in a million bit positions,
/// which at 4800 baud is one spurious search every three minutes and costs
/// nothing, because the Reed-Solomon code then refuses it.
const HEADER_SLACK: u32 = 4;

/// A sonde's frames cut out of a stream of bits
///
/// Above the waveform and below the payload: the bits come from any 4800
/// baud FSK demodulator, and what leaves is a descrambled, Reed-Solomon
/// corrected frame ready for [`parse`]. The receiver's node and anything
/// naming a recording read the same one, so a correction made here reaches
/// both.
#[derive(Default)]
pub struct Framer {
    bits: Vec<bool>,
    /// Bits already searched and known not to start a header. Only appended
    /// to, so what was rejected stays rejected.
    scanned: usize,
}

impl Framer {
    pub fn new() -> Self {
        Self::default()
    }

    /// The buffer a bit clock appends into.
    pub fn sink(&mut self) -> &mut Vec<bool> {
        &mut self.bits
    }

    /// Every frame behind a header in the bits held, taken out of the buffer
    /// as they are read.
    pub fn take(&mut self) -> Vec<Vec<u8>> {
        let mut out = Vec::new();
        let header = header_bits();
        let mut at = self.scanned;
        while at + header.len() <= self.bits.len() {
            let mut wrong = 0u32;
            for (k, &want) in header.iter().enumerate() {
                if self.bits[at + k] != want {
                    wrong += 1;
                    if wrong > HEADER_SLACK {
                        break;
                    }
                }
            }
            if wrong > HEADER_SLACK {
                at += 1;
                continue;
            }
            match self.frame_at(at) {
                // A header with a frame behind it: take both out of the
                // buffer so the search does not walk back into them.
                Some(Some(frame)) => {
                    let used = frame.len() * 8;
                    out.push(frame);
                    self.bits.drain(..at + used);
                    at = 0;
                    self.scanned = 0;
                }
                // A header whose frame has not all arrived: wait here.
                Some(None) => {
                    self.scanned = at;
                    return out;
                }
                None => at += 1,
            }
        }
        self.scanned = at;
        out
    }

    /// Drop what has been searched and found wanting, so a channel with
    /// nothing on it holds a bounded buffer.
    pub fn trim(&mut self) {
        if self.bits.len() > MAX_BITS {
            let drop = self.bits.len() - MAX_BITS;
            self.bits.drain(..drop);
            self.scanned = self.scanned.saturating_sub(drop);
        }
    }

    pub fn reset(&mut self) {
        self.bits.clear();
        self.scanned = 0;
    }

    /// Read the frame starting at bit `at`. `None` where those bits are not
    /// a frame, `Some(None)` where not enough of them have arrived.
    fn frame_at(&self, at: usize) -> Option<Option<Vec<u8>>> {
        // Not enough bits yet is not the same answer as not a frame: the
        // search resumes where it stopped, so treating a short buffer as a
        // rejection walks the cursor past the header and loses the frame
        // that was about to arrive.
        let Some(mut probe) = pack(&self.bits, at, FRAME_STD) else { return Some(None) };
        descramble(&mut probe);
        // The length marker sits in the data, so it has to be read before
        // the code has passed on it; a wrong bit here costs one frame.
        let len = frame_len(probe[DATA_AT])?;
        let mut frame = if len == FRAME_STD {
            probe
        } else {
            let Some(mut long) = pack(&self.bits, at, len) else { return Some(None) };
            descramble(&mut long);
            long
        };
        correct(&mut frame)?;
        Some(Some(frame))
    }
}

/// The header as it arrives: eight bytes, least significant bit first.
fn header_bits() -> Vec<bool> {
    HEADER_AIR.iter().flat_map(|b| (0..8).map(move |k| b >> k & 1 != 0)).collect()
}

/// `len` bytes from bit `at`, least significant bit first, or `None` where
/// the bits are not all there.
fn pack(bits: &[bool], at: usize, len: usize) -> Option<Vec<u8>> {
    if at + len * 8 > bits.len() {
        return None;
    }
    Some(
        bits[at..at + len * 8]
            .chunks(8)
            .map(|c| c.iter().enumerate().fold(0u8, |b, (k, &s)| b | (s as u8) << k))
            .collect(),
    )
}

/// Read a descrambled, corrected frame.
///
/// `None` where it is not a frame at all: the header is wrong, the length
/// marker is not one of the two, or nothing inside passed a CRC. A frame
/// whose blocks all failed is noise that got through the code, and reporting
/// it as a sonde with an empty serial would be worse than reporting nothing.
pub fn parse(frame: &[u8]) -> Option<Frame> {
    if frame.len() < DATA_AT + 1 || frame[..HEADER.len()] != HEADER {
        return None;
    }
    if frame_len(frame[DATA_AT])? != frame.len() {
        return None;
    }
    let mut out = Frame::default();
    let mut at = DATA_AT + 1;
    while at + 4 <= frame.len() {
        let id = frame[at];
        let len = frame[at + 1] as usize;
        if id == 0 || at + 2 + len + 2 > frame.len() {
            break;
        }
        let body = &frame[at + 2..at + 2 + len];
        let want = le16(&frame[at + 2 + len..at + 4 + len]);
        let kind = Block::from_id(id);
        if block_crc(body) == want {
            read_block(kind, body, &mut out);
            out.blocks.push(kind);
        } else {
            out.bad_blocks += 1;
        }
        at += 4 + len;
    }
    (!out.blocks.is_empty()).then_some(out)
}

fn read_block(kind: Block, b: &[u8], out: &mut Frame) {
    match kind {
        Block::Status if b.len() >= 0x18 => {
            out.frame_no = le16(b);
            out.serial = b[2..10]
                .iter()
                .map(|&c| if c.is_ascii_graphic() { c as char } else { '?' })
                .collect();
            out.battery_v = b[0x0A] as f32 / 10.0;
            let flags = le16(&b[0x0D..]);
            out.flight = match (flags & 1 != 0, flags & 2 != 0) {
                (false, _) => Flight::Ground,
                (true, false) => Flight::Ascent,
                (true, true) => Flight::Descent,
            };
            out.crypto = Crypto::from_byte(b[0x0F]);
            out.pcb_temp_c = b[0x10] as i8;
            out.tx_power = b[0x15] & 0x7;
            out.subframe_max = b[0x16];
            if b.len() >= 0x18 + SUBFRAME_PIECE {
                let mut piece = [0u8; SUBFRAME_PIECE];
                piece.copy_from_slice(&b[0x18..0x18 + SUBFRAME_PIECE]);
                out.subframe = Some((b[0x17], piece));
            }
        }
        Block::Meas if b.len() >= 36 => {
            let mut m = [0u32; 12];
            for (i, v) in m.iter_mut().enumerate() {
                *v = u32::from_le_bytes([b[i * 3], b[i * 3 + 1], b[i * 3 + 2], 0]);
            }
            out.meas = Some(m);
        }
        Block::GpsInfo if b.len() >= 6 => {
            out.gps_week = Some(le16(b));
            out.gps_tow_ms = Some(le32(&b[2..]));
        }
        Block::GpsPos if b.len() >= 0x13 => {
            // Centimetres, earth-centred earth-fixed, as the u-blox NAV-SOL
            // message gave them.
            let x = le32(b) as i32 as f64 / 100.0;
            let y = le32(&b[4..]) as i32 as f64 / 100.0;
            let z = le32(&b[8..]) as i32 as f64 / 100.0;
            let vx = le16(&b[0x0C..]) as i16 as f64 / 100.0;
            let vy = le16(&b[0x0E..]) as i16 as f64 / 100.0;
            let vz = le16(&b[0x10..]) as i16 as f64 / 100.0;
            let (lat, lon, alt) = ecef_to_geodetic(x, y, z);
            out.lat_deg = lat;
            out.lon_deg = lon;
            out.altitude_m = alt;
            let (e, n, u) = ecef_velocity_to_enu(lat, lon, vx, vy, vz);
            out.climb_ms = u;
            out.speed_kt = (e * e + n * n).sqrt() * 3600.0 / 1852.0;
            out.course_deg = e.atan2(n).to_degrees().rem_euclid(360.0);
            out.satellites = b[0x12];
        }
        _ => {}
    }
}

/// Bytes of calibration in each frame, and how many frames it takes.
///
/// A sonde has no room to send its calibration with every reading, so it
/// sends a sixteenth of it a second and repeats: 51 pieces, of which the
/// first 50 are constants burnt in at the factory and the 51st is what the
/// sonde is doing now.
pub const SUBFRAME_PIECE: usize = 16;
pub const SUBFRAME_PIECES: usize = 51;
/// Pieces the factory constants occupy, which is what the CRC covers.
const SUBFRAME_FIXED: usize = 50;

/// The calibration a sonde spreads over its frames, as much of it as has
/// arrived.
///
/// Kept per sonde and not per frame: this is the one thing a receiver has to
/// accumulate to read a radiosonde at all, because the sensor block carries
/// ratios and the constants that turn a ratio into degrees come separately.
/// Until the right pieces have arrived the temperature is not unknown to
/// within some error, it is not there, and reporting a number would be
/// inventing one.
#[derive(Clone)]
pub struct Calibration {
    bytes: [u8; SUBFRAME_PIECES * SUBFRAME_PIECE],
    have: [bool; SUBFRAME_PIECES],
}

impl Default for Calibration {
    fn default() -> Self {
        Self::new()
    }
}

impl Calibration {
    pub fn new() -> Self {
        Self { bytes: [0; SUBFRAME_PIECES * SUBFRAME_PIECE], have: [false; SUBFRAME_PIECES] }
    }

    /// Take the piece a frame carried.
    pub fn feed(&mut self, index: u8, piece: &[u8; SUBFRAME_PIECE]) {
        let i = index as usize;
        if i >= SUBFRAME_PIECES {
            return;
        }
        self.bytes[i * SUBFRAME_PIECE..(i + 1) * SUBFRAME_PIECE].copy_from_slice(piece);
        self.have[i] = true;
    }

    /// One piece as it arrived, for a test that wants to write it down.
    pub fn piece(&self, index: u8) -> Option<&[u8; SUBFRAME_PIECE]> {
        let i = index as usize;
        if i >= SUBFRAME_PIECES || !self.have[i] {
            return None;
        }
        self.bytes[i * SUBFRAME_PIECE..(i + 1) * SUBFRAME_PIECE].try_into().ok()
    }

    pub fn pieces(&self) -> usize {
        self.have.iter().filter(|h| **h).count()
    }

    /// Whether every piece has arrived and the whole of it checks.
    ///
    /// The sonde puts a CRC-16 over its own constants in the first two bytes
    /// of the first piece, covering the 50 fixed pieces and not the 51st,
    /// which changes as it flies. Worth checking rather than trusting the
    /// count: a piece can arrive from a frame the Reed-Solomon code repaired
    /// wrongly, and a bad calibration constant is a plausible temperature
    /// that is quietly several degrees out.
    pub fn complete(&self) -> bool {
        if !self.have.iter().all(|h| *h) {
            return false;
        }
        let want = le16(&self.bytes);
        block_crc(&self.bytes[2..SUBFRAME_FIXED * SUBFRAME_PIECE]) == want
    }

    /// A little-endian float at a byte offset, where every piece it spans
    /// has arrived.
    fn f32_at(&self, at: usize) -> Option<f32> {
        if at + 4 > self.bytes.len() {
            return None;
        }
        for i in at / SUBFRAME_PIECE..=(at + 3) / SUBFRAME_PIECE {
            if !self.have[i] {
                return None;
            }
        }
        Some(f32::from_le_bytes([
            self.bytes[at],
            self.bytes[at + 1],
            self.bytes[at + 2],
            self.bytes[at + 3],
        ]))
    }

    fn f32s<const N: usize>(&self, at: usize) -> Option<[f32; N]> {
        let mut out = [0.0; N];
        for (i, v) in out.iter_mut().enumerate() {
            *v = self.f32_at(at + 4 * i)?;
        }
        Some(out)
    }

    /// The two reference resistors the temperature ratios are against, in
    /// ohms. Nominally 750 and 1100 on every sonde.
    pub fn reference_ohms(&self) -> Option<(f32, f32)> {
        Some((self.f32_at(61)?, self.f32_at(65)?))
    }

    /// The model this sonde says it is: "RS41-SG", "RS41-SGP".
    pub fn model(&self) -> Option<String> {
        for i in 0x21..=0x22 {
            if !self.have[i] {
                return None;
            }
        }
        let s: String =
            self.bytes[0x218..0x222].iter().take_while(|b| **b != 0).map(|b| *b as char).collect();
        let s = s.trim().to_string();
        (!s.is_empty() && s.is_ascii()).then_some(s)
    }

    /// Air temperature in degrees, from the sensor count and its two
    /// references.
    ///
    /// A PT1000 read as a ratio against two reference resistors: the two
    /// references fix the gain and offset of whatever the counter is doing
    /// this second, which is what makes the reading independent of the
    /// sonde's own temperature and supply. The polynomial and the three
    /// per-sonde corrections after it are Vaisala's, measured on that
    /// sonde in the factory, and are the reason the calibration has to be
    /// collected before a reading means anything.
    pub fn air_temperature_c(&self, meas: &[u32; 12]) -> Option<f32> {
        self.temperature(meas[0], meas[1], meas[2], 77, 89)
    }

    /// The thermometer on the humidity sensor itself, which runs warmer
    /// than the air because the sensor is heated.
    pub fn humidity_sensor_temperature_c(&self, meas: &[u32; 12]) -> Option<f32> {
        self.temperature(meas[6], meas[7], meas[8], 293, 305)
    }

    fn temperature(&self, f: u32, f1: u32, f2: u32, co_at: usize, cal_at: usize) -> Option<f32> {
        let (rf1, rf2) = self.reference_ohms()?;
        let p: [f32; 3] = self.f32s(co_at)?;
        let c: [f32; 3] = self.f32s(cal_at)?;
        if f2 == f1 {
            return None;
        }
        let d = (f2 as f32) - (f1 as f32);
        let gain = d / (rf2 - rf1);
        let offset = (f1 as f32 * rf2 - f2 as f32 * rf1) / d;
        let r = (f as f32 / gain - offset) * c[0];
        let t = (p[0] + p[1] * r + p[2] * r * r + c[1]) * (1.0 + c[2]);
        t.is_finite().then_some(t)
    }

    /// Relative humidity as a percentage, by the empirical fit.
    ///
    /// Not Vaisala's own reduction, which needs the whole 7 by 6 correction
    /// matrix and the sensor's time lag: this is the fit zilog80 arrived at
    /// against the German weather service's published ascents, and it is
    /// what every hobby decoder reports. Good to a few percent near the
    /// ground and worse as the sensor gets cold, which is why the
    /// temperature corrections below it are there at all.
    pub fn humidity_pct(&self, meas: &[u32; 12], air_temp_c: f32) -> Option<f32> {
        let c0 = self.f32_at(117)?;
        if c0 == 0.0 || meas[5] == meas[4] {
            return None;
        }
        let a1 = 350.0 / c0;
        let fh = (meas[3] as f32 - meas[4] as f32) / (meas[5] as f32 - meas[4] as f32);
        let mut rh = 100.0 * (a1 * fh - 7.5);
        rh -= air_temp_c / 5.5;
        if air_temp_c < -20.0 {
            rh *= 1.0 + (-20.0 - air_temp_c) / 100.0;
        }
        if air_temp_c < -40.0 {
            rh *= 1.0 + (-40.0 - air_temp_c) / 120.0;
        }
        rh.is_finite().then(|| rh.clamp(0.0, 100.0))
    }
}

/// What the sonde's sensors read, once there is enough calibration to say.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Ptu {
    pub temperature_c: Option<f32>,
    pub humidity_pct: Option<f32>,
    /// The heated humidity sensor's own temperature, which is not the air's.
    pub sensor_temp_c: Option<f32>,
}

impl Frame {
    /// What this frame's sensor counts mean, given what has been collected
    /// of the sonde's calibration. Empty until the right pieces arrive.
    pub fn ptu(&self, cal: &Calibration) -> Ptu {
        let Some(meas) = &self.meas else { return Ptu::default() };
        let temperature_c = cal.air_temperature_c(meas);
        Ptu {
            temperature_c,
            humidity_pct: temperature_c.and_then(|t| cal.humidity_pct(meas, t)),
            sensor_temp_c: cal.humidity_sensor_temperature_c(meas),
        }
    }
}

/// What the protocols node makes of a sonde frame: which balloon it is,
/// where, and how it is flying.
pub fn decoded(bytes: &[u8], center: common::Hz) -> Option<Decoded> {
    let f = parse(bytes)?;
    let mut fields: Vec<(String, common::Value)> = vec![
        ("serial".into(), common::Value::Text(f.serial.clone())),
        ("frame".into(), common::Value::Int(f.frame_no as i64)),
        ("battery_v".into(), common::Value::Float(f.battery_v as f64)),
        ("state".into(), common::Value::Text(f.flight.label().into())),
    ];
    if f.has_position() {
        fields.push(("altitude_m".into(), common::Value::Float(f.altitude_m)));
        fields.push(("climb_ms".into(), common::Value::Float(f.climb_ms)));
        fields.push(("speed_kt".into(), common::Value::Float(f.speed_kt)));
        fields.push(("course_deg".into(), common::Value::Float(f.course_deg)));
        fields.push(("satellites".into(), common::Value::Int(f.satellites as i64)));
    }
    if let (Some(w), Some(t)) = (f.gps_week, f.gps_tow_ms) {
        fields.push(("gps_week".into(), common::Value::Int(w as i64)));
        fields.push(("gps_tow_ms".into(), common::Value::Int(t as i64)));
    }
    fields.push(("pcb_temp_c".into(), common::Value::Int(f.pcb_temp_c as i64)));
    if f.bad_blocks > 0 {
        fields.push(("bad_blocks".into(), common::Value::Int(f.bad_blocks as i64)));
    }

    let mut d = Decoded::bytes("rs41", center, 0.0, bytes.to_vec())
        .with_modulation(common::Modulation::Fsk2)
        .with_crc(Some(f.bad_blocks == 0))
        .with_text(f.summary())
        .with_detail(format!("frame {}, {:.1} V, {}", f.frame_no, f.battery_v, f.flight.label()))
        .with_fields(fields)
        .by(common::Identity::new("vaisala", f.serial.clone()).made_by("Vaisala"));
    if f.has_position() {
        d = d
            .reporting(common::ReportDetail::Sonde {
                altitude_m: f.altitude_m,
                climb_ms: f.climb_ms,
                battery_v: f.battery_v,
                satellites: f.satellites,
                descending: f.flight == Flight::Descent,
                sensors: f.meas.map(|meas| common::SondeSensors { meas, calibration: f.subframe }),
            })
            .at_position(common::Position {
                lat: f.lat_deg,
                lon: f.lon_deg,
                altitude_m: Some(f.altitude_m),
                speed_kt: Some(f.speed_kt),
                course_deg: Some(f.course_deg),
            });
    }
    Some(d)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The two frames published in `rs1729/RS`, `rs41/rs41.txt`, as they came
    /// off the air: header, parity and scrambled data. The first is stated
    /// there to be sonde K1930293 over Zagreb, the second sonde L4020244.
    const FRAME_K1930293: &str = "8635f44093df1a602c87e0fa0521e8943d9cef4c7a67393f6d39fb546461f2111b6447ab79a746c80350cda5344157f8c0c12234f46902220f792816174b313933303239331a00000300000a00002f0007322ce53e31991abf12dada3eb68468c16755d51c7a2a15310216060245f302000d08a31607821e08bb210219060243f302000000000000000000000000000000220d7c1e0807d03cdc071fd81ddb19d70a8d0eb602b60cb518d40692ff00ff00ff001c277d59b8d83301ff0f881f0f38f4fe18b283038735ff000000003eb8ff4947201e6e3aff55415f13fc6e005440440cf100009e9f7406f85800832b631719d70010bebc172a8b00000000000000000000000000000000000000000000a48b7b15366181193ef05d07e1245b1be0f721f801f60804107b0b76110000000000000000000000000000000000ecc7";

    /// The same file's long frame, two symbols of which arrived wrong. The
    /// text quotes both the received bytes and what the code made of them,
    /// so this pins the Reed-Solomon decoder as well as the parse.
    const FRAME_K4020244: &str = "8635f44093df1a608f9b1025bf8ec9e28ad68413c31788307e9881c5cb2f37f754fa09b711c5c39977ed8fbf22377b3e5e1cee59fc644b19f0792896134b343032303234341c00000100000c00007a0007320f00000000008920bac20000000000000092697a2ae9030226fd015de502363208522a075f330874040228fd015de502000000000000000000000000000000e7917c1e4d0750f1921703fb01f8068d1fd811f70bd604d50afa17f913d90c8b20f9a16a7d5921103501ff440000006c1f00cd977e059ab7009566fd191d1affd82fbf143fb8ff5277180991faff9ca1d10d441b01927bf211dd190190999f0553a1ff9120b10c3847ff06eeee0e571301a2c0891c000000cddd1a0882d10011167b153c154217941930005fc50b1eb9fde107d2050902115a537ea6ed343030313030303120313037393020202033312e37203036373520303334392030373030203132383636203630303520313339333120363031342031343038322035383830203738313420383032372031303039203930392039353631353632203935303839323220343238383339313633382032393335383636203539343238203335323439203636393920333738332034363837203637303120363930312037393939049a762d000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000f35a";

    fn hex(s: &str) -> Vec<u8> {
        (0..s.len() / 2).map(|i| u8::from_str_radix(&s[i * 2..i * 2 + 2], 16).unwrap()).collect()
    }

    /// The text prints the descrambled frame, which is what everything here
    /// works on, so a test only has to check the header came out right.
    fn descrambled(f: Vec<u8>) -> Vec<u8> {
        assert_eq!(f[..HEADER.len()], HEADER, "the published frame is the descrambled one");
        f
    }

    /// And the other way about, to what a receiver would have off the air.
    fn on_air(mut f: Vec<u8>) -> Vec<u8> {
        descramble(&mut f);
        assert_eq!(f[..HEADER_AIR.len()], HEADER_AIR);
        f
    }

    #[test]
    fn the_short_frame_names_its_sonde_over_zagreb() {
        let f = descrambled(hex(FRAME_K1930293));
        assert_eq!(f.len(), FRAME_STD);
        let mut f = f;
        // Nothing was wrong with it on the way to print, so the code has
        // nothing to do.
        assert_eq!(correct(&mut f), Some(0));
        let r = parse(&f).expect("a frame");
        assert_eq!(r.serial, "K1930293");
        assert_eq!(r.frame_no, 5910);
        assert_eq!(r.battery_v, 2.6);
        assert_eq!(r.crypto, Crypto::Standard);
        assert_eq!(r.flight, Flight::Descent);
        assert_eq!(r.bad_blocks, 0);
        assert_eq!(
            r.blocks,
            vec![
                Block::Status,
                Block::Meas,
                Block::GpsInfo,
                Block::GpsRaw,
                Block::GpsPos,
                Block::Empty
            ]
        );
        // The text prints one line of this flight's telemetry, for frame
        // 5808: 46.04934 N, 16.13034 E, 32347.21 m. This frame is 102 later,
        // and everything about the two agrees. It has fallen 3937 m, which is
        // 38.6 m/s averaged over those 102 seconds against the 36.2 m/s this
        // frame reports instantaneously, and it has drifted 1.3 km west,
        // which is the course it reports. Nothing but a correct ECEF
        // conversion puts an independently published fix and this one on the
        // same trajectory.
        assert!((r.lat_deg - 46.0503).abs() < 1e-4, "{}", r.lat_deg);
        assert!((r.lon_deg - 16.1108).abs() < 1e-4, "{}", r.lon_deg);
        assert!((r.altitude_m - 28_410.0).abs() < 1.0, "{}", r.altitude_m);
        assert!((r.climb_ms + 36.17).abs() < 0.01, "{}", r.climb_ms);
        assert!((r.course_deg - 272.7).abs() < 0.1, "west: {}", r.course_deg);
        assert!((r.speed_kt - 26.4).abs() < 0.1, "{}", r.speed_kt);
        // Week 1800 day 1, 131874000 ms: Monday 7 July 2014, 12:37:54 UTC,
        // which is the date the text gives the flight.
        assert_eq!(r.gps_week, Some(1800));
        assert_eq!(r.gps_tow_ms, Some(131_874_000));
        assert_eq!(r.satellites, 8);
        assert_eq!(r.pcb_temp_c, 10);
        assert_eq!(r.tx_power, 7);
    }

    /// The long frame carries two symbol errors the text says the code
    /// repaired, at positions 234 and 252 of the second codeword.
    #[test]
    fn the_long_frame_is_repaired_and_names_its_sonde() {
        let mut f = descrambled(hex(FRAME_K4020244));
        assert_eq!(f.len(), FRAME_AUX);
        assert_eq!(correct(&mut f), Some(2));
        let r = parse(&f).expect("a frame");
        assert_eq!(r.serial, "K4020244");
        assert_eq!(r.frame_no, 5014);
        assert_eq!(r.bad_blocks, 0);
        assert_eq!(r.crypto, Crypto::Standard);
        assert_eq!(r.flight, Flight::Ascent);
        assert!(r.blocks.contains(&Block::Xdata), "the long frame is the one carrying text");
        assert_eq!(r.blocks.len(), 7);
        // Over East Anglia at 10 km, climbing. Week 1869, 395506000 ms, is
        // Thursday 8 October 2015 at 21:51:46 UTC, which is a sonde on its
        // way up to a 23:00 sounding.
        assert!((r.lat_deg - 52.4420).abs() < 1e-4, "{}", r.lat_deg);
        assert!((r.lon_deg - 0.4629).abs() < 1e-4, "{}", r.lon_deg);
        assert!((r.altitude_m - 10_021.7).abs() < 1.0, "{}", r.altitude_m);
        assert!((r.climb_ms - 8.36).abs() < 0.01, "{}", r.climb_ms);
        assert_eq!(r.gps_week, Some(1869));
        assert_eq!(r.gps_tow_ms, Some(395_506_000));
        assert_eq!(r.satellites, 9);
    }

    /// The long frame's two bad symbols both landed in the parity, which the
    /// text remarks on, so it reads without the code being run at all. That
    /// is what makes it a test of the code rather than of the parse: the
    /// fields have to come out the same either way, and [`correct`] still has
    /// to find exactly two.
    #[test]
    fn the_errors_in_the_long_frame_are_in_its_parity() {
        let raw = descrambled(hex(FRAME_K4020244));
        let mut fixed = raw.clone();
        assert_eq!(correct(&mut fixed), Some(2));
        assert_eq!(raw[DATA_AT..], fixed[DATA_AT..], "no data byte was wrong");
        assert_eq!(parse(&raw).map(|f| f.serial), parse(&fixed).map(|f| f.serial));
        assert_eq!(parse(&raw).expect("a frame").bad_blocks, 0);
    }

    /// A byte lost past what the code can carry falls to the block CRC,
    /// which is the check that stops a broken block being reported as a
    /// reading. The frame is still a frame: its other blocks stand.
    #[test]
    fn a_block_whose_crc_fails_is_counted_and_not_read() {
        let mut f = descrambled(hex(FRAME_K1930293));
        let whole = parse(&f).expect("a frame");
        // Into the GPS position block, which the last test showed is the
        // fifth one and starts 21 bytes plus its header from the end of the
        // padding.
        let at = f.len() - 24;
        f[at] ^= 0x40;
        let r = parse(&f).expect("still a frame");
        assert_eq!(r.bad_blocks, 1);
        assert_eq!(r.serial, whole.serial, "the status block is untouched");
        assert!(!r.has_position(), "a block that failed its CRC reports no fix");
        assert_eq!(r.blocks.len(), whole.blocks.len() - 1);
    }

    /// One bit flipped anywhere in a frame's data is one symbol wrong in one
    /// codeword, which the code takes out; twenty-six spread across it are
    /// past the thirteen either codeword can carry.
    #[test]
    fn the_code_carries_twelve_errors_a_codeword_and_not_a_hundred() {
        let good = descrambled(hex(FRAME_K1930293));
        for bad in [1usize, 12, 24] {
            let mut f = good.clone();
            for i in 0..bad {
                f[DATA_AT + i * 7] ^= 0x80;
            }
            assert_eq!(correct(&mut f), Some(bad), "{bad} errors");
            assert_eq!(f, good, "{bad} errors left the frame changed");
        }
        let mut f = good.clone();
        for i in 0..100 {
            f[DATA_AT + i] ^= 0x5a;
        }
        assert_eq!(correct(&mut f), None, "a hundred errors is not correctable");
    }

    /// A frame built here reads back, so a test elsewhere can make one.
    #[test]
    fn a_made_frame_round_trips() {
        let mut f = vec![0u8; FRAME_STD];
        f[..HEADER.len()].copy_from_slice(&HEADER);
        f[DATA_AT] = LEN_STD;
        let mut body = vec![0u8; 0x28];
        body[..2].copy_from_slice(&1234u16.to_le_bytes());
        body[2..10].copy_from_slice(b"V1234567");
        body[0x0A] = 29;
        body[0x0D] = 0x03;
        f[DATA_AT + 1] = 0x79;
        f[DATA_AT + 2] = body.len() as u8;
        f[DATA_AT + 3..DATA_AT + 3 + body.len()].copy_from_slice(&body);
        let crc = block_crc(&body);
        f[DATA_AT + 3 + body.len()..DATA_AT + 5 + body.len()].copy_from_slice(&crc.to_le_bytes());
        protect(&mut f);
        assert_eq!(correct(&mut f.clone()), Some(0), "the parity it was given is the right parity");
        // And the scrambler takes it to the constant a receiver correlates.
        let mut air = on_air(f.clone());
        descramble(&mut air);
        assert_eq!(air, f, "the scrambler is its own inverse");
        let r = parse(&f).expect("a frame");
        assert_eq!(r.serial, "V1234567");
        assert_eq!(r.frame_no, 1234);
        assert_eq!(r.battery_v, 2.9);
        assert_eq!(r.flight, Flight::Descent);
        assert!(!r.has_position());
    }

    /// The whole factory calibration of sonde S1720982, as it arrived over
    /// the 500 second recording the corpus capture is cut from, and one
    /// frame's sensor counts to read against it.
    ///
    /// Written down rather than fetched because it is the only way to test
    /// the reduction offline: the pieces needed for temperature arrive
    /// within any 51 frames, but the CRC over the whole of it needs all 51.
    const CAL_S1720982: [&str; SUBFRAME_PIECES] = [
        "976700910300000e0000000000533137",
        "3230393832f74e000058021205b43ca4",
        "06148732000000ffff00000000031923",
        "e8030004000700bf0291b3000600803b",
        "44008089440000000000003c422ae973",
        "c35f28403ebb9209372b68a13f586b8a",
        "bd4fd3783b0000000000000000000000",
        "000000000093a0374260bc9d40e17929",
        "bb52980fc05fc41e41c39f67c0e96b59",
        "42339abac28ed24e42c37b1b42f86f51",
        "43f037bdc3a8c51241933d9c41eb4116",
        "4314e816c345288cc3094b36434ff64a",
        "456f3a7f45869169c3f1afac438d3748",
        "437b1fc2c3871a62c50000000054d761",
        "43f40c69c30000000000000000000000",
        "00000000008920bac200000000000000",
        "00000000000000000000000000000000",
        "00000000000000000000000000000000",
        "00000000002ae973c35f28403ebb9209",
        "374b02a53fed12a3be31ed863c000000",
        "00000000000000000000000000000000",
        "0000ffffffc600289237420000000000",
        "cdcccc3dbdff4bbf47499ebd6636b133",
        "5b398bb71b8af13900e0aa44f085493c",
        "0000003f000090400000a03f00000000",
        "3333333f68912d3f0000803f00000000",
        "00000000e6967e3f97829bb8aa392330",
        "e416cd29b5265aa2fdeb021aec51383e",
        "3333333f000000000000000000000000",
        "00000000000000000000000000000000",
        "f67f74403b3682bfe52f983d00010001",
        "4b69a2be19d2d03d00000040d517a341",
        "6b5da24100004040ffffffc6ffffffc6",
        "ffffffc6ffffffc6525334312d534700",
        "000052534d3431320000000053313731",
        "31343131003030303030303030303000",
        "00000030303030303030300000812300",
        "001a020002e500b63faa000000000000",
        "00000000000000000000000000000000",
        "00000000000000000000000000000000",
        "00000000000000000000000000000000",
        "00000000000000000000000000000000",
        "000000000000d5caa43d5da365397f87",
        "2239000000000000000009feb7bcc896",
        "e53e31991abf12dada3eb68468c16755",
        "5742d6c5aac1849ec7c1fdbc3e411e16",
        "4cc27cb88b41bb32f441000000000000",
        "00000000000000000000030001001400",
        "c80046003c0005003c0018019e62d5b8",
        "6c9c07b1003c88770000000000000000",
        "ffff32ea5b020700fbfde901161b0000",
    ];

    /// Frame 3403 of that flight, at 10299.3 m: the temperature sensor and
    /// its two references, the humidity sensor and its two, the humidity
    /// sensor's thermometer and its two, then three zeros where a pressure
    /// sensor would be on an RS41-SGP.
    const MEAS_3403: [u32; 12] =
        [136587, 135247, 196226, 552247, 483095, 551578, 137891, 135248, 196223, 0, 0, 0];

    fn calibration() -> Calibration {
        let mut c = Calibration::new();
        for (i, line) in CAL_S1720982.iter().enumerate() {
            let b = hex(line);
            c.feed(i as u8, &b.try_into().expect("sixteen bytes"));
        }
        c
    }

    /// The sonde's own CRC over its constants, which is the only check there
    /// is that a piece did not arrive from a frame the code repaired wrongly.
    #[test]
    fn the_calibration_checks_against_its_own_crc() {
        let c = calibration();
        assert_eq!(c.pieces(), SUBFRAME_PIECES);
        assert!(c.complete());
        // It says what it is, and it is the model with no pressure sensor,
        // which is why the last three counts of every frame are zero.
        assert_eq!(c.model().as_deref(), Some("RS41-SG"));
        // The two reference resistors, which are 750 and 1100 ohms on every
        // sonde: a transcription of the offsets that was a few bytes out
        // would not land on round numbers.
        assert_eq!(c.reference_ohms(), Some((750.0, 1100.0)));
        // A single wrong byte anywhere in the constants is caught.
        let mut bad = calibration();
        let mut piece = *bad.piece(20).unwrap();
        piece[7] ^= 0x01;
        bad.feed(20, &piece);
        assert!(!bad.complete(), "a flipped bit passed the calibration CRC");
    }

    /// The reduction, against the profile the Met Office published for this
    /// same ascent: Herstmonceux (03882), 00 UTC on 27 December 2021, which
    /// reads -57.2 C at 10219 m and -58.5 C at 10358 m. This frame is at
    /// 10299 m, between them.
    #[test]
    fn the_thermometer_reads_what_the_met_office_published() {
        let c = calibration();
        let t = c.air_temperature_c(&MEAS_3403).expect("no temperature");
        assert!((t + 57.42).abs() < 0.01, "{t}");
        // The humidity sensor runs warmer than the air, because it is
        // heated to keep ice off it. It must not be reported as the air
        // temperature, which is what reading the wrong three counts would
        // do.
        let th = c.humidity_sensor_temperature_c(&MEAS_3403).expect("no sensor temperature");
        assert!((th + 52.0).abs() < 0.1, "{th}");
        assert!(th > t + 4.0, "the heated sensor is not warmer than the air: {th} against {t}");
        // Relative humidity by the empirical fit. The published profile says
        // 45% at 10219 m and 52% at 10358 m.
        let rh = c.humidity_pct(&MEAS_3403, t).expect("no humidity");
        assert!((rh - 47.7).abs() < 0.1, "{rh}");
    }

    /// Until the pieces holding the constants have arrived there is no
    /// temperature, rather than a plausible wrong one. The sonde sends one
    /// piece a second, so this is the first minute of every flight.
    #[test]
    fn a_reading_waits_for_the_pieces_it_needs() {
        let full = calibration();
        let mut c = Calibration::new();
        assert_eq!(c.air_temperature_c(&MEAS_3403), None, "nothing collected");
        // The references are in pieces 3 and 4, the polynomial in 4 and 5,
        // and the per-sonde corrections in 5 and 6.
        for i in [3u8, 4, 5] {
            c.feed(i, full.piece(i).unwrap());
            assert_eq!(c.air_temperature_c(&MEAS_3403), None, "read with only {i} pieces in");
        }
        c.feed(6, full.piece(6).unwrap());
        let t = c.air_temperature_c(&MEAS_3403).expect("four pieces is enough");
        assert!((t + 57.42).abs() < 0.01, "{t}");
        // Humidity needs one more, and the frame's own reading of the
        // heated sensor needs pieces from much later in the cycle.
        assert_eq!(c.humidity_pct(&MEAS_3403, t), None);
        c.feed(7, full.piece(7).unwrap());
        assert!(c.humidity_pct(&MEAS_3403, t).is_some());
        assert_eq!(c.humidity_sensor_temperature_c(&MEAS_3403), None);
    }

    /// A frame carries its piece of the calibration and its sensor counts,
    /// and the short frame in the corpus carries both.
    #[test]
    fn a_frame_carries_its_counts_and_its_piece_of_the_calibration() {
        let mut f = descrambled(hex(FRAME_K1930293));
        assert_eq!(correct(&mut f), Some(0));
        let r = parse(&f).expect("a frame");
        let (index, _) = r.subframe.expect("no calibration piece");
        assert_eq!(index, 44);
        assert_eq!(r.subframe_max, 0x32, "fifty pieces and the run-time one");
        let meas = r.meas.expect("no sensor counts");
        // A count is a 24-bit ratio, and the three references are close to
        // each other because they are the same kind of part.
        assert_eq!(meas[0], 143_637);
        assert!(meas.iter().take(9).all(|m| *m > 100_000 && *m < 700_000), "{meas:?}");
        // Nothing can be made of them, because this frame is all there is.
        assert_eq!(r.ptu(&Calibration::new()), Ptu::default());
    }

    #[test]
    fn noise_is_not_a_frame() {
        let mut seed = 0x9e37_79b9_7f4a_7c15u64;
        let mut hits = 0;
        for _ in 0..500 {
            let mut f: Vec<u8> = (0..FRAME_STD)
                .map(|_| {
                    seed ^= seed << 13;
                    seed ^= seed >> 7;
                    seed ^= seed << 17;
                    (seed >> 32) as u8
                })
                .collect();
            f[..HEADER.len()].copy_from_slice(&HEADER);
            if parse(&f).is_some() {
                hits += 1;
            }
        }
        assert_eq!(hits, 0, "{hits} of 500 noise frames parsed");
    }
}
