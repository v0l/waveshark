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
    Identification { callsign: String, category: u8 },
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
    SurfacePosition { odd: bool, lat_cpr: u32, lon_cpr: u32 },
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
    Unsupported { type_code: u8 },
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
    const POLY: u32 = 0x00ff_f409;
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
        7 | 14 => Some(crc24(bytes)),
        _ => None,
    }
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
    /// Addresses proposed by frames that cannot prove themselves, and how many
    /// times each has been proposed.
    pending: std::collections::HashMap<u32, u32>,
}

/// Sightings before an address that never proved itself is believed.
///
/// A receiver hears aircraft that only ever answer interrogations, never
/// broadcasting a position, so refusing every unprovable address loses them
/// entirely. The way back in is repetition: noise proposes a uniformly random
/// 24 bit address each time, so the chance of the same one arriving three
/// times is negligible, while an aircraft answering a radar sends dozens a
/// minute. Two would not do: across the hundreds of thousands of noise
/// candidates a busy band produces, coincidental pairs are common.
const SIGHTINGS: u32 = 3;

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
    /// `confident` says the demodulator had no marginal bits in this frame.
    /// Only a confident frame may propose a new address: a frame read out of
    /// noise has bits that were a coin toss, and letting those vote is how a
    /// receiver invents aircraft.
    pub fn accept(&mut self, bytes: &[u8], confident: bool) -> bool {
        let Some(df) = bytes.first().map(|b| b >> 3) else { return false };
        match df {
            // Carries its own CRC over the whole frame.
            17 | 18 if bytes.len() == 14 => {
                // A frame with one bit wrong is still that aircraft's frame,
                // and on this band a single clipped chip is the usual damage.
                let Some(fixed) = fix_single_bit(bytes) else { return false };
                let icao = ((fixed[1] as u32) << 16)
                    | ((fixed[2] as u32) << 8)
                    | fixed[3] as u32;
                self.insert(icao);
                true
            }
            // An all-call reply: the remainder is the interrogator id, which
            // is zero for the ones a receiver overhears.
            11 if bytes.len() == 7 => {
                // DF11 overlays the interrogator id, which is zero for the
                // all-call replies a listener overhears.
                if crc24(bytes) == 0 {
                    let icao = ((bytes[1] as u32) << 16)
                        | ((bytes[2] as u32) << 8)
                        | bytes[3] as u32;
                    self.insert(icao);
                    return true;
                }
                false
            }
            // Everything else is only as trustworthy as the address it names.
            // An unknown one is held until it has been proposed enough times
            // to be more than a coincidence.
            0 | 4 | 5 | 16 | 20 | 21 | 24 => {
                let Some(a) = overlaid_address(bytes) else { return false };
                if self.contains(a) {
                    return true;
                }
                if !confident {
                    return false;
                }
                let n = self.pending.entry(a).or_insert(0);
                *n += 1;
                if *n >= SIGHTINGS {
                    self.pending.remove(&a);
                    self.insert(a);
                    return true;
                }
                false
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
pub fn cpr_global(
    even: (u32, u32),
    odd: (u32, u32),
    odd_is_newer: bool,
) -> Option<(f64, f64)> {
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
    let j = (lat_ref / d_lat).floor() + ((lat_ref.rem_euclid(d_lat)) / d_lat - lat_cpr + 0.5).floor();
    let lat = d_lat * (j + lat_cpr);

    let nl = cpr_nl(lat);
    let ni = if odd { (nl - 1.0).max(1.0) } else { nl.max(1.0) };
    let d_lon = 360.0 / ni;
    let m = (lon_ref / d_lon).floor() + ((lon_ref.rem_euclid(d_lon)) / d_lon - lon_cpr + 0.5).floor();
    let lon = d_lon * (m + lon_cpr);
    (lat, lon)
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
        let Message::AirbornePosition { odd: oo, .. } = o.kind else {
            panic!("not a position")
        };
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
