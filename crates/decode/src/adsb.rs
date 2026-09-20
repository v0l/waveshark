//! Mode S and ADS-B 1090ES frames.
//!
//! This is the frame layer only: bytes in, aircraft state out. The 1090 MHz
//! demodulator that produces those bytes is a separate problem, because Mode S
//! does not fit the pulse front end the ISM protocols share. Its bits are 1 us
//! wide with 0.5 us half-chips, so the mark/gap timings an envelope detector
//! produces at 31 kHz channel bandwidth are three orders of magnitude too
//! coarse. Keeping the two apart also means the parsing can be checked against
//! published frames without a radio in the room.
//!
//! A frame is 56 or 112 bits. The first five are the downlink format, and the
//! last 24 are a CRC that doubles as an address in some formats:
//!
//! ```text
//! DF5 CA3 ICAO24 ME56 PI24     (DF17, 112 bits, the ADS-B one)
//! DF5 ...       ...   AP24     (DF0, DF4, DF5, DF11, 56 bits)
//! ```
//!
//! Only DF17 and DF18 carry position and identity in the clear. The short
//! formats are interrogation replies whose CRC is overlaid with the aircraft
//! address, so they can be recognised but not attributed without a list of
//! addresses seen recently, which is what `IcaoSeen` in a receiver would be.
//!
//! Position is the awkward part. ADS-B sends compact position reporting, a
//! pair of ambiguous coordinates that only resolve when an even and an odd
//! frame are combined, or when a reference position within 180 nautical miles
//! is already known. Both are implemented here: [`cpr_global`] for a cold
//! start from two frames, [`cpr_local`] for the cheap path afterwards.

use common::Cpr;
use common::packet::{Entity, Fact, Id, Link, Motion, Named, Party, Proto, Quantity, ThingKind};
use std::fmt;

/// A parsed 1090 MHz frame.
#[derive(Clone, Debug, PartialEq)]
pub struct Frame {
    /// Downlink format, the top five bits.
    pub df: u8,
    /// 24 bit ICAO address, when the format carries one in the clear.
    pub icao: Option<u32>,
    pub kind: Message,
    /// Bytes as received, for logging and for the formats not parsed here.
    pub raw: Vec<u8>,
}

/// What a frame says, for the formats worth parsing.
#[derive(Clone, Debug, PartialEq)]
pub enum Message {
    /// Callsign, or the tail number when no flight number is filed.
    Identification {
        callsign: String,
        category: u8,
    },
    /// One half of a position pair. Useless alone; see [`cpr_global`].
    AirbornePosition {
        /// Barometric or GNSS altitude in feet, absent when the aircraft is
        /// not reporting one.
        altitude_ft: Option<i32>,
        /// True when this is the odd frame of the pair.
        odd: bool,
        /// Encoded latitude, 17 bits.
        lat_cpr: u32,
        /// Encoded longitude, 17 bits.
        lon_cpr: u32,
    },
    SurfacePosition {
        odd: bool,
        lat_cpr: u32,
        lon_cpr: u32,
    },
    /// Ground velocity, from the two subtypes that report it that way.
    Velocity {
        /// Knots over the ground.
        ground_speed_kt: f64,
        /// Degrees true.
        track_deg: f64,
        /// Feet per minute, positive up.
        vertical_rate_fpm: i32,
    },
    /// A format this decoder does not parse, named by its type code.
    Unsupported {
        type_code: u8,
    },
    /// A reply to an interrogation, carrying a Comm-B register.
    ///
    /// DF20 answers with an altitude and DF21 with a squawk, and both attach
    /// 56 bits of whichever register the radar asked for. Nothing in the frame
    /// says which register that was, so [`crate::bds::infer`] works it out and
    /// reports nothing when the answer is not clear.
    CommB {
        /// Barometric altitude, from a DF20.
        altitude_ft: Option<i32>,
        /// Mode A code, from a DF21.
        squawk: Option<u16>,
        report: Option<crate::bds::Report>,
    },
    /// A short reply, recognisable but not attributable on its own.
    ShortReply,
}

/// Why a frame was rejected.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FrameError {
    /// Not 56 or 112 bits.
    WrongLength(usize),
    /// The 24 bit CRC did not come to zero.
    CrcFailed,
}

impl fmt::Display for FrameError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::WrongLength(n) => write!(f, "{n} bytes is not a Mode S frame"),
            Self::CrcFailed => write!(f, "CRC failed"),
        }
    }
}

/// Mode S CRC-24, polynomial 0xFFF409, no reflection, zero init.
///
/// Over a whole frame including its parity field the result is zero for the
/// formats that do not overlay an address. That is the only integrity check
/// 1090 MHz has, and it is doing more work here than a CRC usually does: at
/// this bit rate a receiver sees far more noise than aircraft, so anything
/// that fails it has to be dropped without a second thought.
pub fn crc24(data: &[u8]) -> u32 {
    // The demodulator frames on this polynomial as well as checking it, so it
    // is declared beside the waveform.
    const POLY: u32 = dsp::modes::CRC24_POLY;
    let mut rem: u32 = 0;
    for &b in data {
        rem ^= (b as u32) << 16;
        for _ in 0..8 {
            rem = if rem & 0x0080_0000 != 0 { (rem << 1) ^ POLY } else { rem << 1 };
            rem &= 0x00ff_ffff;
        }
    }
    rem
}

/// Correct a single flipped bit, when the CRC says exactly one is wrong.
///
/// The parity field is a CRC, so the remainder over a corrupt frame depends
/// only on which bits are wrong, not on what the frame said. One flipped bit
/// therefore gives one particular remainder, and the map from remainder back
/// to bit position can be built once and looked up.
///
/// This is worth doing because 1090 MHz is a shared, uncoordinated channel:
/// most losses are a single chip clipped by another aircraft's transmission
/// rather than a frame lost to noise. It only applies to frames that carry a
/// plain CRC, since an overlaid address is indistinguishable from an error.
pub fn fix_single_bit(bytes: &[u8]) -> Option<Vec<u8>> {
    let syndrome = crc24(bytes);
    if syndrome == 0 {
        return Some(bytes.to_vec());
    }
    let bit = *syndromes(bytes.len())?.get(&syndrome)?;
    let mut fixed = bytes.to_vec();
    fixed[bit / 8] ^= 0x80 >> (bit % 8);
    Some(fixed)
}

/// What a demodulated reply is, with the one bit error an extended
/// squitter's parity can locate put back first.
///
/// The acceptance test, in one place: the receiver's node and anything
/// naming a recording both take a frame from the demodulator and have to
/// make the same decision about it, and a repair applied in one and not the
/// other is two receivers.
pub fn accept(bytes: &[u8]) -> Option<(Vec<u8>, Frame)> {
    let df = bytes.first()? >> 3;
    // Only a frame carrying a plain CRC can be repaired: an overlaid address
    // is indistinguishable from an error.
    let fixed = match df {
        17 | 18 => fix_single_bit(bytes).unwrap_or_else(|| bytes.to_vec()),
        _ => bytes.to_vec(),
    };
    let frame = parse(&fixed).ok()?;
    Some((fixed, frame))
}

/// Remainder to bit position, for each frame length, built on first use.
fn syndromes(len: usize) -> Option<&'static std::collections::HashMap<u32, usize>> {
    use std::collections::HashMap;
    use std::sync::OnceLock;
    static SHORT: OnceLock<HashMap<u32, usize>> = OnceLock::new();
    static LONG: OnceLock<HashMap<u32, usize>> = OnceLock::new();
    let build = |len: usize| {
        let mut m = HashMap::with_capacity(len * 8);
        for bit in 0..len * 8 {
            let mut f = vec![0u8; len];
            f[bit / 8] = 0x80 >> (bit % 8);
            m.insert(crc24(&f), bit);
        }
        m
    };
    match len {
        7 => Some(SHORT.get_or_init(|| build(7))),
        14 => Some(LONG.get_or_init(|| build(14))),
        _ => None,
    }
}

/// The address a short reply was addressed to, if this frame is one.
///
/// DF0, 4, 5, 16, 20, 21 and 24 overlay the aircraft's address on their parity
/// field, so the CRC remainder over the whole frame *is* the address rather
/// than zero. That makes them unverifiable on their own: any 56 bits of noise
/// yields some remainder, and reporting it as an aircraft invents a different
/// one every time. They are only worth believing when the address is one an
/// ADS-B frame has already proved is out there, which is what [`AddressBook`]
/// is for.
pub fn overlaid_address(bytes: &[u8]) -> Option<u32> {
    match bytes.len() {
        7 | 14 => Some(syndrome(bytes)),
        _ => None,
    }
}

/// What a frame leaves over its parity field: zero, an address or an
/// interrogator's id.
///
/// The checksum of everything but the parity, exclusive-ored with the parity
/// as transmitted, which is how the format defines it. Running the parity
/// bytes through the register instead also comes to zero for a clean frame,
/// so the two agree on whether a DF17 checks out, but they do not agree on
/// anything else: feeding them through leaves the address multiplied by x^24,
/// a different 24 bit number, which no ADS-B frame will ever match.
pub fn syndrome(bytes: &[u8]) -> u32 {
    let Some(split) = bytes.len().checked_sub(3) else {
        return 0;
    };
    let parity = bytes[split..].iter().fold(0u32, |a, b| (a << 8) | *b as u32);
    crc24(&bytes[..split]) ^ parity
}

/// Addresses seen in frames that carried their own CRC.
///
/// A short reply is accepted only when it names one of these. The window is in
/// frames rather than seconds because the point is corroboration, not liveness:
/// an aircraft that transmitted a verifiable position a moment ago is still
/// overhead when its altitude reply arrives.
#[derive(Clone, Debug, Default)]
pub struct AddressBook {
    seen: std::collections::HashSet<u32>,
}

impl AddressBook {
    pub fn new() -> Self {
        Self::default()
    }

    /// Record the address of a frame that verified on its own.
    pub fn insert(&mut self, icao: u32) {
        self.seen.insert(icao);
    }

    pub fn contains(&self, icao: u32) -> bool {
        self.seen.contains(&icao)
    }

    pub fn len(&self) -> usize {
        self.seen.len()
    }

    pub fn is_empty(&self) -> bool {
        self.seen.is_empty()
    }

    /// Whether this frame is worth believing, and remember it if it proves
    /// itself. Suitable as the validator a demodulator drives its search with.
    ///
    /// Nothing here weighs how cleanly the bits were read: a frame either
    /// proves itself by its CRC, or names an aircraft one already has.
    pub fn accept(&mut self, bytes: &[u8]) -> bool {
        let Some(df) = bytes.first().map(|b| b >> 3) else {
            return false;
        };
        match df {
            // Carries its own CRC over the whole frame.
            17 | 18 if bytes.len() == 14 => {
                // A frame with one bit wrong is still that aircraft's frame,
                // and on this band a single clipped chip is the usual damage.
                let Some(fixed) = fix_single_bit(bytes) else {
                    return false;
                };
                let icao = ((fixed[1] as u32) << 16) | ((fixed[2] as u32) << 8) | fixed[3] as u32;
                self.insert(icao);
                true
            }
            // An all-call reply, whose remainder is the interrogator it is
            // answering rather than a fault.
            11 if bytes.len() == 7 => {
                let icao = ((bytes[1] as u32) << 16) | ((bytes[2] as u32) << 8) | bytes[3] as u32;
                match syndrome(bytes) {
                    // Nobody's interrogation: the frame proves itself, so it
                    // may name a new aircraft.
                    0 => {
                        self.insert(icao);
                        true
                    }
                    // A reply to a ground station, carrying that station's id
                    // in the low seven bits of the remainder. Over 10 seconds
                    // of Dublin approach, 825 of 1181 all-call replies were
                    // these, so reading only the zero ones is a third of the
                    // DF11 an mlat client has to synchronise on. Nothing here
                    // is checked, though: an id is seven bits and noise lands
                    // inside that once in 128, which is why this corroborates
                    // rather than proves and cannot propose an address.
                    iid if iid < 128 => self.contains(icao),
                    _ => false,
                }
            }
            // Everything else is only as trustworthy as the address it names,
            // and an address nothing has proved is not an aircraft. Letting an
            // unproved one in after three sightings was worth 5 frames in
            // 3727 off radarpi, which does not buy the chance of inventing an
            // aircraft out of a repeated misread.
            0 | 4 | 5 | 16 | 20 | 21 | 24 => {
                let Some(a) = overlaid_address(bytes) else {
                    return false;
                };
                self.contains(a)
            }
            // A downlink format nothing transmits.
            _ => false,
        }
    }
}

/// Parse a frame, verifying the CRC.
pub fn parse(bytes: &[u8]) -> Result<Frame, FrameError> {
    if bytes.len() != 7 && bytes.len() != 14 {
        return Err(FrameError::WrongLength(bytes.len()));
    }
    let df = bytes[0] >> 3;
    // DF17 and DF18 carry a plain CRC. The short replies overlay the aircraft
    // address on theirs, so the remainder is the address rather than zero and
    // cannot be checked without knowing which aircraft is expected.
    let extended = matches!(df, 17 | 18);
    if extended && crc24(bytes) != 0 {
        return Err(FrameError::CrcFailed);
    }
    // A Comm-B reply. The address is the CRC remainder rather than a field,
    // so it is only as good as whatever decided to let this frame through:
    // [`AddressBook`] accepts one when an ADS-B frame has already proved that
    // aircraft is out there, and everything downstream inherits that judgment.
    if !extended && matches!(df, 20 | 21) && bytes.len() == 14 {
        let ac = (((bytes[2] & 0x1f) as u16) << 8) | bytes[3] as u16;
        return Ok(Frame {
            df,
            icao: overlaid_address(bytes),
            kind: Message::CommB {
                altitude_ft: (df == 20).then(|| altitude_13(ac)).flatten(),
                squawk: (df == 21).then(|| squawk_13(ac)).flatten(),
                report: crate::bds::infer(&bytes[4..11]),
            },
            raw: bytes.to_vec(),
        });
    }
    if !extended {
        return Ok(Frame { df, icao: None, kind: Message::ShortReply, raw: bytes.to_vec() });
    }

    let icao = ((bytes[1] as u32) << 16) | ((bytes[2] as u32) << 8) | bytes[3] as u32;
    let me = &bytes[4..11];
    let tc = me[0] >> 3;
    let kind = match tc {
        1..=4 => Message::Identification { callsign: callsign(me), category: me[0] & 0x07 },
        5..=8 => Message::SurfacePosition {
            odd: me[2] & 0x04 != 0,
            lat_cpr: cpr_lat(me),
            lon_cpr: cpr_lon(me),
        },
        9..=18 | 20..=22 => Message::AirbornePosition {
            altitude_ft: altitude(me),
            odd: me[2] & 0x04 != 0,
            lat_cpr: cpr_lat(me),
            lon_cpr: cpr_lon(me),
        },
        19 => match velocity(me) {
            Some((ground_speed_kt, track_deg, vertical_rate_fpm)) => {
                Message::Velocity { ground_speed_kt, track_deg, vertical_rate_fpm }
            }
            None => Message::Unsupported { type_code: tc },
        },
        _ => Message::Unsupported { type_code: tc },
    };
    Ok(Frame { df, icao: Some(icao), kind, raw: bytes.to_vec() })
}

/// The six bit character set callsigns are packed in, index by code.
const CHARSET: &[u8; 64] = b"#ABCDEFGHIJKLMNOPQRSTUVWXYZ##### ###############0123456789######";

fn callsign(me: &[u8]) -> String {
    // Eight six-bit characters packed into the 48 bits after the type code
    // and category, so most of them straddle a byte boundary and the last one
    // ends exactly at the end of the field. Assembling the whole ME into one
    // integer first is what keeps that last character from reading off the
    // end of the slice.
    let v = me.iter().take(7).fold(0u64, |acc, b| (acc << 8) | *b as u64);
    let mut s = String::with_capacity(8);
    for i in 0..8 {
        s.push(CHARSET[((v >> (42 - 6 * i)) & 0x3f) as usize] as char);
    }
    s.trim_end().replace('#', "")
}

fn cpr_lat(me: &[u8]) -> u32 {
    (((me[2] as u32) & 0x03) << 15) | ((me[3] as u32) << 7) | ((me[4] as u32) >> 1)
}

fn cpr_lon(me: &[u8]) -> u32 {
    (((me[4] as u32) & 0x01) << 16) | ((me[5] as u32) << 8) | me[6] as u32
}

/// Barometric altitude from an airborne position message.
///
/// The Q bit picks the encoding: set means 25 foot steps, clear means the
/// 100 foot Gillham code, which is Gray coded and is not decoded here. All
/// zeros means the aircraft is not reporting an altitude at all, which is not
/// the same as reporting zero and must not be shown as ground level.
fn altitude(me: &[u8]) -> Option<i32> {
    // Twelve bits: all of ME byte 1 and the top half of byte 2.
    let raw = ((me[1] as u32) << 4) | ((me[2] as u32 & 0xf0) >> 4);
    if raw == 0 {
        return None;
    }
    let q = raw & 0x10 != 0;
    if !q {
        return None;
    }
    let n = ((raw & 0x0fe0) >> 1) | (raw & 0x000f);
    Some(n as i32 * 25 - 1000)
}

/// The 13 bit altitude field of a DF4, DF20 or all-call reply.
///
/// The M bit picks metres and the Q bit picks 25 foot steps. Neither the
/// metric encoding nor the 100 foot Gillham code is decoded here, and both are
/// reported as no altitude rather than as a wrong one: a Gillham code read as
/// binary is a plausible number at the wrong height.
pub fn altitude_13(ac: u16) -> Option<i32> {
    if ac == 0 || ac & 0x0040 != 0 || ac & 0x0010 == 0 {
        return None;
    }
    let n = ((ac & 0x1f80) >> 2) | ((ac & 0x0020) >> 1) | (ac & 0x000f);
    Some(n as i32 * 25 - 1000)
}

/// The 13 bit identity field of a DF5 or DF21, as the four digit squawk the
/// crew set.
///
/// The bits are interleaved by pulse position rather than by digit: reading
/// the field as a number gives something that looks like a squawk and is not.
pub fn squawk_13(id: u16) -> Option<u16> {
    if id == 0 {
        return None;
    }
    // C1 A1 C2 A2 C4 A4 X B1 D1 B2 D2 B4 D4, from the top of the field, and
    // each digit's own bits run 4, 2, 1 rather than in field order.
    let at = |i: u32| (id >> (12 - i)) & 1;
    let a = (at(5) << 2) | (at(3) << 1) | at(1);
    let b = (at(11) << 2) | (at(9) << 1) | at(7);
    let c = (at(4) << 2) | (at(2) << 1) | at(0);
    let d = (at(12) << 2) | (at(10) << 1) | at(8);
    Some(a * 1000 + b * 100 + c * 10 + d)
}

/// Ground speed, track and vertical rate from a type 19 subtype 1 or 3.
fn velocity(me: &[u8]) -> Option<(f64, f64, i32)> {
    let subtype = me[0] & 0x07;
    if subtype != 1 && subtype != 2 {
        // Subtypes 3 and 4 report airspeed and heading instead, which is a
        // different quantity and must not be passed off as ground track.
        return None;
    }
    // Supersonic subtypes report in four knot units.
    let scale = if subtype == 2 { 4.0 } else { 1.0 };
    let ew_sign = if me[1] & 0x04 != 0 { -1.0 } else { 1.0 };
    let ew = (((me[1] as u32 & 0x03) << 8) | me[2] as u32) as f64;
    let ns_sign = if me[3] & 0x80 != 0 { -1.0 } else { 1.0 };
    let ns = (((me[3] as u32 & 0x7f) << 3) | ((me[4] as u32 & 0xe0) >> 5)) as f64;
    if ew == 0.0 || ns == 0.0 {
        // Zero means "no value", not "not moving".
        return None;
    }
    let vx = ew_sign * (ew - 1.0) * scale;
    let vy = ns_sign * (ns - 1.0) * scale;
    let speed = (vx * vx + vy * vy).sqrt();
    let mut track = vx.atan2(vy).to_degrees();
    if track < 0.0 {
        track += 360.0;
    }

    // The vertical rate straddles ME bytes 4 and 5: three bits at the bottom
    // of one and six at the top of the next, with its sign the bit above.
    let vr_raw = (((me[4] as u32 & 0x07) << 6) | ((me[5] as u32 & 0xfc) >> 2)) as i32;
    let vr_sign = if me[4] & 0x08 != 0 { -1 } else { 1 };
    let vertical_rate = if vr_raw == 0 { 0 } else { vr_sign * (vr_raw - 1) * 64 };
    Some((speed, track, vertical_rate))
}

/// Latitude zones, fixed by the standard.
const NZ: f64 = 15.0;

/// Number of longitude zones at a given latitude.
fn cpr_nl(lat: f64) -> f64 {
    let lat = lat.abs();
    if lat >= 87.0 {
        return 1.0;
    }
    if lat < 10.0 {
        return 59.0;
    }
    let a = 1.0 - (std::f64::consts::PI / (2.0 * NZ)).cos();
    let b = (std::f64::consts::PI / 180.0 * lat).cos().powi(2);
    let nl = (std::f64::consts::TAU / (1.0 - a / b).acos()).floor();
    nl.max(1.0)
}

/// Position from an even and an odd frame, with no prior knowledge.
///
/// `even` and `odd` are the encoded pairs, and `odd_is_newer` says which
/// arrived last, because the result is reported at that frame's time and using
/// the wrong one puts the aircraft a few seconds behind itself.
///
/// Returns `None` when the two frames disagree about which latitude zone they
/// are in, which happens when they were transmitted either side of a zone
/// boundary. That is a real ambiguity, not an error: the fix is the next pair.
pub fn cpr_global(even: (u32, u32), odd: (u32, u32), odd_is_newer: bool) -> Option<(f64, f64)> {
    let (lat_e, lon_e) = (even.0 as f64 / 131_072.0, even.1 as f64 / 131_072.0);
    let (lat_o, lon_o) = (odd.0 as f64 / 131_072.0, odd.1 as f64 / 131_072.0);
    let d_lat_e = 360.0 / (4.0 * NZ);
    let d_lat_o = 360.0 / (4.0 * NZ - 1.0);

    // Latitude index: which of the fifteen zone-pairs the aircraft is in.
    let j = (59.0 * lat_e - 60.0 * lat_o + 0.5).floor();
    let mut rlat_e = d_lat_e * (j.rem_euclid(60.0) + lat_e);
    let mut rlat_o = d_lat_o * (j.rem_euclid(59.0) + lat_o);
    if rlat_e >= 270.0 {
        rlat_e -= 360.0;
    }
    if rlat_o >= 270.0 {
        rlat_o -= 360.0;
    }
    if cpr_nl(rlat_e) != cpr_nl(rlat_o) {
        return None;
    }

    let (lat, nl) = if odd_is_newer { (rlat_o, cpr_nl(rlat_o)) } else { (rlat_e, cpr_nl(rlat_e)) };
    let ni = if odd_is_newer { (nl - 1.0).max(1.0) } else { nl.max(1.0) };
    let m = (lon_e * (nl - 1.0) - lon_o * nl + 0.5).floor();
    let lon_cpr = if odd_is_newer { lon_o } else { lon_e };
    let mut lon = (360.0 / ni) * (m.rem_euclid(ni) + lon_cpr);
    if lon >= 180.0 {
        lon -= 360.0;
    }
    Some((lat, lon))
}

/// Position from one frame plus a reference within 180 nautical miles.
///
/// This is the path a receiver uses once it has a fix: one frame is enough, so
/// position updates at the rate the aircraft transmits rather than at the rate
/// even and odd frames happen to pair up.
pub fn cpr_local(reference: (f64, f64), cpr: (u32, u32), odd: bool) -> (f64, f64) {
    let (lat_ref, lon_ref) = reference;
    let (lat_cpr, lon_cpr) = (cpr.0 as f64 / 131_072.0, cpr.1 as f64 / 131_072.0);
    let d_lat = 360.0 / if odd { 4.0 * NZ - 1.0 } else { 4.0 * NZ };
    let j =
        (lat_ref / d_lat).floor() + ((lat_ref.rem_euclid(d_lat)) / d_lat - lat_cpr + 0.5).floor();
    let lat = d_lat * (j + lat_cpr);

    let nl = cpr_nl(lat);
    let ni = if odd { (nl - 1.0).max(1.0) } else { nl.max(1.0) };
    let d_lon = 360.0 / ni;
    let m =
        (lon_ref / d_lon).floor() + ((lon_ref.rem_euclid(d_lon)) / d_lon - lon_cpr + 0.5).floor();
    let lon = d_lon * (m + lon_cpr);
    (lat, lon)
}

/// What a Mode S frame says.
///
/// A position frame carries half a place, so it states the compact halves and
/// the tracker resolves them: a decoder that answered with a latitude would be
/// inventing one. Height is a reading rather than part of a place, because an
/// aircraft sends one in frames that say nothing about where it is.
pub fn read(frame: &Frame) -> Proto {
    let mut p = Proto::new("adsb", kind_of(&frame.kind));
    if let Some(icao) = frame.icao {
        let id = format!("{icao:06x}");
        p = p
            .by(Entity::new("adsb", Id::Hex(u64::from(icao))))
            .between(Link::beacon(Party::unit(id)));
    }
    match &frame.kind {
        Message::Identification { callsign, .. } => {
            p = p.saying(Fact::Named(Named::new(callsign.clone(), ThingKind::Aircraft)));
            if let Some(e) = p.subject.as_mut() {
                e.name = Some(callsign.clone());
            }
        }
        Message::AirbornePosition { altitude_ft, odd, lat_cpr, lon_cpr } => {
            p = p.saying(Fact::PartialPosition(Cpr { odd: *odd, lat: *lat_cpr, lon: *lon_cpr }));
            p = p.maybe(altitude_ft.map(feet));
        }
        Message::SurfacePosition { odd, lat_cpr, lon_cpr } => {
            // On the ground, so the height that goes with this position is
            // zero and not whatever it was reporting on the way down.
            p = p
                .saying(Fact::PartialPosition(Cpr { odd: *odd, lat: *lat_cpr, lon: *lon_cpr }))
                .saying(feet(0));
        }
        Message::Velocity { ground_speed_kt, track_deg, vertical_rate_fpm } => {
            p = p.saying(Fact::Motion(Motion {
                speed_kt: Some(*ground_speed_kt),
                course_deg: Some(*track_deg),
                climb_ms: Some(f64::from(*vertical_rate_fpm) * FPM_TO_MS),
                heading_deg: None,
            }));
        }
        Message::CommB { altitude_ft, report, .. } => {
            p = p.maybe(altitude_ft.map(feet));
            match report {
                Some(crate::bds::Report::Meteo(m)) => {
                    if let (Some(kt), Some(deg)) = (m.wind_kt, m.wind_dir_deg) {
                        p = p
                            .saying(Fact::sensed(Quantity::WindSpeed, kt, common::Unit::Knot))
                            .saying(Fact::sensed(
                                Quantity::WindDirection,
                                deg,
                                common::Unit::Degree,
                            ));
                    }
                    p = p.saying(Fact::sensed(
                        Quantity::Temperature,
                        m.temp_c,
                        common::Unit::Celsius,
                    ));
                }
                Some(crate::bds::Report::TrackTurn { track_deg, ground_speed_kt, .. }) => {
                    p = p.saying(Fact::Motion(Motion {
                        speed_kt: *ground_speed_kt,
                        course_deg: *track_deg,
                        climb_ms: None,
                        heading_deg: None,
                    }));
                }
                Some(crate::bds::Report::Identification { callsign }) => {
                    // The same aircraft naming itself, in answer to a radar
                    // rather than in a broadcast, and the only name some of
                    // them ever give.
                    p = p.saying(Fact::Named(Named::new(callsign.clone(), ThingKind::Aircraft)));
                    if let Some(e) = p.subject.as_mut() {
                        e.name = Some(callsign.clone());
                    }
                }
                _ => {}
            }
        }
        Message::Unsupported { .. } | Message::ShortReply => {}
    }
    p
}

/// A height the frame reported, in the unit every reading is stated in
fn feet(ft: i32) -> Fact {
    Fact::sensed(Quantity::Altitude, f64::from(ft) * 0.3048, common::Unit::Metre)
}

/// Feet a minute as metres a second, which is the unit a climb is stated in
const FPM_TO_MS: f64 = 0.00508;

/// Which message it is, as the name a row matches on
fn kind_of(m: &Message) -> &'static str {
    match m {
        Message::Identification { .. } => "identification",
        Message::AirbornePosition { .. } => "airborne_position",
        Message::SurfacePosition { .. } => "surface_position",
        Message::Velocity { .. } => "velocity",
        Message::Unsupported { .. } => "other",
        Message::ShortReply => "reply",
        Message::CommB { report, .. } => match report {
            Some(crate::bds::Report::Meteo(_)) => "weather",
            Some(crate::bds::Report::Identification { .. }) => "identification",
            Some(crate::bds::Report::TrackTurn { .. }) => "track",
            Some(crate::bds::Report::HeadingSpeed { .. }) => "heading_speed",
            Some(crate::bds::Report::VerticalIntent { .. }) => "vertical_intent",
            Some(crate::bds::Report::Capability { .. }) => "capability",
            None => "comm_b",
        },
    }
}

/// The fields of one Comm-B register, named the way the rest of the log names
/// things: units in the key, so a row reads without a legend.
pub fn commb_fields(r: &crate::bds::Report, fields: &mut Vec<(String, common::Value)>) {
    use common::Value;
    fn num(fields: &mut Vec<(String, Value)>, k: &str, v: Option<f64>) {
        if let Some(v) = v {
            fields.push((k.to_string(), Value::Float(round1(v))));
        }
    }
    let num = |fields: &mut Vec<(String, Value)>, k: &str, v: Option<f64>| num(fields, k, v);
    match r {
        crate::bds::Report::Meteo(m) => {
            num(fields, "wind_kt", m.wind_kt);
            num(fields, "wind_dir_deg", m.wind_dir_deg);
            num(fields, "temperature_c", Some(m.temp_c));
            num(fields, "humidity_pct", m.humidity_pct);
            if let Some(p) = m.pressure_hpa {
                fields.push(("pressure_hpa".into(), Value::Int(p as i64)));
            }
            if let Some(t) = m.turbulence {
                fields.push(("turbulence".into(), Value::Int(t as i64)));
            }
        }
        crate::bds::Report::Identification { callsign } => {
            fields.push(("callsign".into(), Value::Text(callsign.clone())));
        }
        crate::bds::Report::TrackTurn {
            roll_deg,
            track_deg,
            ground_speed_kt,
            track_rate_deg_s,
            true_airspeed_kt,
        } => {
            num(fields, "roll_deg", *roll_deg);
            num(fields, "track_deg", *track_deg);
            num(fields, "ground_speed_kt", *ground_speed_kt);
            num(fields, "track_rate_deg_s", *track_rate_deg_s);
            num(fields, "true_airspeed_kt", *true_airspeed_kt);
        }
        crate::bds::Report::HeadingSpeed {
            heading_deg,
            indicated_airspeed_kt,
            mach,
            baro_vertical_rate_fpm,
            inertial_vertical_rate_fpm,
        } => {
            num(fields, "heading_deg", *heading_deg);
            num(fields, "indicated_airspeed_kt", *indicated_airspeed_kt);
            if let Some(m) = mach {
                fields.push(("mach".into(), Value::Float((m * 100.0).round() / 100.0)));
            }
            for (k, v) in [
                ("vertical_rate_fpm", baro_vertical_rate_fpm),
                ("inertial_rate_fpm", inertial_vertical_rate_fpm),
            ] {
                if let Some(v) = v {
                    fields.push((k.to_string(), Value::Int(*v as i64)));
                }
            }
        }
        crate::bds::Report::VerticalIntent { selected_altitude_ft, fms_altitude_ft, qnh_mb } => {
            for (k, v) in [
                ("selected_altitude_ft", selected_altitude_ft),
                ("fms_altitude_ft", fms_altitude_ft),
            ] {
                if let Some(v) = v {
                    fields.push((k.to_string(), Value::Int(*v as i64)));
                }
            }
            num(fields, "qnh_mb", *qnh_mb);
        }
        crate::bds::Report::Capability { subnetwork_version } => {
            fields.push(("subnetwork".into(), Value::Int(*subnetwork_version as i64)));
        }
    }
}

pub fn round1(v: f64) -> f64 {
    (v * 10.0).round() / 10.0
}

#[cfg(test)]
mod tests {
    use super::*;

    fn hex(s: &str) -> Vec<u8> {
        (0..s.len() / 2).map(|i| u8::from_str_radix(&s[i * 2..i * 2 + 2], 16).unwrap()).collect()
    }

    /// Frames from published worked examples. Every one of them ends in a
    /// 24 bit CRC over the other thirteen bytes, so a decoder that gets the
    /// CRC wrong cannot accidentally agree with them: the check and the
    /// vectors corroborate each other.
    /// All-call replies off radarpi, in the capture the DF11 counts come
    /// from. `ZERO_IID` answers nobody and checks to zero; `IID` answers a
    /// ground station and leaves 88 behind, which is that station's id.
    const ZERO_IID: &str = "5d4ca624556ce7";
    const IID: &str = "5d3c66b6c5cee1";

    /// An all-call reply answering a ground station is still that aircraft.
    ///
    /// The remainder of a DF11 is the interrogator it replied to, not a
    /// fault, and only a reply to nobody comes out zero. Reading the zero
    /// ones alone threw away 825 of the 1181 all-call replies in ten seconds
    /// off radarpi, and DF11 is half of what an mlat client synchronises on.
    /// dump1090 ranks the same frame `SR_DF11_IID_KNOWN`, above its accept
    /// threshold, and `SR_DF11_IID_UNKNOWN` below it.
    #[test]
    fn an_all_call_answering_a_ground_station_needs_the_aircraft_known_first() {
        let mut book = AddressBook::new();
        assert_eq!(syndrome(&hex(IID)), 88, "the interrogator's id");
        // Seven bits is a one in 128 chance for noise, so an aircraft nothing
        // has verified stays out however often it is proposed.
        assert!(!book.accept(&hex(IID)));
        assert!(!book.accept(&hex(IID)));
        assert!(!book.accept(&hex(IID)), "an unproved address got in by repetition");

        // A reply to nobody proves itself and names its aircraft.
        assert_eq!(syndrome(&hex(ZERO_IID)), 0);
        assert!(book.accept(&hex(ZERO_IID)));
        assert!(book.contains(0x4c_a624));

        // And once an ADS-B frame has proved that aircraft, its interrogated
        // replies are believed too.
        book.insert(0x3c_66b6);
        assert!(book.accept(&hex(IID)));

        // A remainder too big to be an interrogator id is a frame read wrong.
        let mut damaged = hex(IID);
        damaged[4] ^= 0x40;
        assert!(syndrome(&damaged) >= 128);
        assert!(!book.accept(&damaged));
    }

    const IDENT: &str = "8D4840D6202CC371C32CE0576098";
    const POS_EVEN: &str = "8D40621D58C382D690C8AC2863A7";
    const POS_ODD: &str = "8D40621D58C386435CC412692AD6";
    const VELOCITY: &str = "8D485020994409940838175B284F";

    #[test]
    fn the_crc_of_a_real_frame_comes_to_zero() {
        for f in [IDENT, POS_EVEN, POS_ODD, VELOCITY] {
            assert_eq!(crc24(&hex(f)), 0, "{f} failed its CRC");
        }
    }

    #[test]
    fn a_single_bit_error_is_caught() {
        let mut b = hex(IDENT);
        b[6] ^= 0x01;
        assert_eq!(parse(&b), Err(FrameError::CrcFailed));
    }

    #[test]
    fn a_callsign_decodes_from_its_six_bit_packing() {
        let f = parse(&hex(IDENT)).unwrap();
        assert_eq!(f.df, 17);
        assert_eq!(f.icao, Some(0x4840d6));
        match f.kind {
            Message::Identification { callsign, category } => {
                assert_eq!(callsign, "KLM1023");
                assert_eq!(category, 0);
            }
            other => panic!("expected an identification message, got {other:?}"),
        }
    }

    #[test]
    fn an_airborne_position_carries_its_altitude_and_parity() {
        let e = parse(&hex(POS_EVEN)).unwrap();
        let o = parse(&hex(POS_ODD)).unwrap();
        let Message::AirbornePosition { altitude_ft: ae, odd: oe, .. } = e.kind else {
            panic!("not a position")
        };
        let Message::AirbornePosition { odd: oo, .. } = o.kind else { panic!("not a position") };
        assert_eq!(ae, Some(38_000), "altitude is 38000 ft in the worked example");
        assert!(!oe, "the first frame is the even one");
        assert!(oo, "the second frame is the odd one");
    }

    #[test]
    fn two_frames_resolve_to_a_position() {
        // The worked example puts this aircraft over the Netherlands at
        // 52.2572 N, 3.91937 E.
        let Message::AirbornePosition { lat_cpr: le, lon_cpr: ne, .. } =
            parse(&hex(POS_EVEN)).unwrap().kind
        else {
            panic!()
        };
        let Message::AirbornePosition { lat_cpr: lo, lon_cpr: no, .. } =
            parse(&hex(POS_ODD)).unwrap().kind
        else {
            panic!()
        };
        // The even frame is the later of the two in the worked example, so
        // the fix is reported at its position.
        let (lat, lon) = cpr_global((le, ne), (lo, no), false).expect("same latitude zone");
        assert!((lat - 52.2572).abs() < 0.001, "latitude came out as {lat}");
        assert!((lon - 3.91937).abs() < 0.001, "longitude came out as {lon}");
    }

    #[test]
    fn one_frame_and_a_reference_resolve_to_the_same_place() {
        // The cheap path, once a fix exists. A receiver at Schiphol is well
        // within the 180 nautical mile limit of the aircraft above.
        let Message::AirbornePosition { lat_cpr, lon_cpr, odd, .. } =
            parse(&hex(POS_EVEN)).unwrap().kind
        else {
            panic!()
        };
        let (lat, lon) = cpr_local((52.258, 3.918), (lat_cpr, lon_cpr), odd);
        assert!((lat - 52.2572).abs() < 0.001, "latitude came out as {lat}");
        assert!((lon - 3.91937).abs() < 0.001, "longitude came out as {lon}");
    }

    #[test]
    fn velocity_decodes_to_ground_speed_track_and_climb() {
        // Worked example: 159 kt on a track of 182.88 degrees, descending at
        // 832 feet per minute.
        let f = parse(&hex(VELOCITY)).unwrap();
        match f.kind {
            Message::Velocity { ground_speed_kt, track_deg, vertical_rate_fpm } => {
                assert!((ground_speed_kt - 159.20).abs() < 0.1, "speed {ground_speed_kt}");
                assert!((track_deg - 182.88).abs() < 0.01, "track {track_deg}");
                assert_eq!(vertical_rate_fpm, -832);
            }
            other => panic!("expected a velocity message, got {other:?}"),
        }
    }

    #[test]
    fn a_short_reply_is_recognised_but_not_attributed() {
        // 56 bit formats overlay the aircraft address on the CRC, so the
        // remainder is the address rather than zero. Reporting one as an
        // aircraft would invent a different aircraft for every reply.
        let f = parse(&hex("02E19838ADB7C4")).unwrap();
        assert_eq!(f.df, 0);
        assert_eq!(f.icao, None);
        assert_eq!(f.kind, Message::ShortReply);
    }

    #[test]
    fn a_comm_b_reply_carries_its_altitude_and_its_register() {
        // A DF20 from pyModeS's test suite: the radar asked for the callsign
        // register and the aircraft answered with EXS2MF.
        let f = parse(&hex("A0001838201584F23468207CDFA5")).unwrap();
        assert_eq!(f.df, 20);
        // The address is what the frame leaves over its parity, believable
        // only because something upstream matched it against an aircraft
        // already seen. 40655A is in the United Kingdom's allocation, which a
        // Jet2 flight should be; running the parity through the register
        // instead gave FD0006, which is in nobody's.
        assert_eq!(f.icao, Some(0x40_655a));
        let Message::CommB { altitude_ft, squawk, report } = f.kind else {
            panic!("not read as a Comm-B reply")
        };
        assert_eq!(altitude_ft, Some(38_000), "altitude {altitude_ft:?}");
        assert_eq!(squawk, None, "a DF20 has no squawk in it");
        assert_eq!(report, Some(crate::bds::Report::Identification { callsign: "EXS2MF".into() }));
    }

    #[test]
    fn an_altitude_field_only_decodes_the_encoding_it_understands() {
        // Q set, 25 foot steps.
        assert_eq!(altitude_13(0x1838), Some(38_000));
        assert_eq!(altitude_13(0x00b0), Some(200));
        // Q clear is the Gillham code, which is not decoded, and M set is
        // metric. Both must report nothing rather than a number: a Gillham
        // code read as binary is a plausible height that is not the one the
        // aircraft is at.
        assert_eq!(altitude_13(0x1828), None);
        assert_eq!(altitude_13(0x1878), None);
        assert_eq!(altitude_13(0), None);
    }

    #[test]
    fn a_squawk_is_read_by_pulse_position_rather_than_as_a_number() {
        // A1 alone is the thousands digit of 1000, and it sits in bit 1 of
        // the field rather than anywhere a plain integer would put it.
        assert_eq!(squawk_13(1 << 11), Some(1000));
        // The emergency code 7700: A=7, B=7, C=0, D=0.
        let bits = (1 << 11) | (1 << 9) | (1 << 7) | (1 << 5) | (1 << 3) | (1 << 1);
        assert_eq!(squawk_13(bits), Some(7700));
        assert_eq!(squawk_13(0), None);
    }

    #[test]
    fn a_frame_of_the_wrong_length_is_refused() {
        assert_eq!(parse(&hex("8D4840D620")), Err(FrameError::WrongLength(5)));
    }

    #[test]
    fn longitude_zones_narrow_towards_the_poles() {
        assert_eq!(cpr_nl(0.0), 59.0);
        assert_eq!(cpr_nl(52.0), 36.0);
        assert_eq!(cpr_nl(87.5), 1.0);
        assert_eq!(cpr_nl(-52.0), 36.0, "zones are symmetric about the equator");
    }
}
