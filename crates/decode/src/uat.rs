//! UAT at 978 MHz: the other ADS-B link, and the ground stations answering it.
//!
//! Light aircraft in the United States that are not on 1090 MHz broadcast
//! here, and the ground stations broadcast back: traffic the aircraft cannot
//! see itself (TIS-B) and weather (FIS-B). Bytes in, fields out; the waveform
//! is `dsp::fsk` and the wiring is `nodes::uat_nodes`.
//!
//! The field layout is DO-282B, read here as dump978 reads it
//! (<https://github.com/mutability/dump978>, `uat_decode.c`), which is the
//! implementation every published UAT decode agrees with.

use crate::rs::ReedSolomon;
use common::Decoded;
use dsp::fsk::{SyncBurst, SyncPattern};

/// 978 MHz, the one channel.
pub const CHANNEL_HZ: f64 = 978_000_000.0;
/// 1.041667 Mbit/s, DO-282B.
pub const BAUD: f64 = 1_041_667.0;
/// Peak deviation either side of the carrier.
pub const DEVIATION_HZ: f64 = 312_500.0;
/// The 36-bit sync word an aircraft sends.
pub const ADSB_SYNC: u64 = 0xEAC_DDA_4E2;
/// The ground station's, which is its bitwise complement.
pub const UPLINK_SYNC: u64 = 0x153_225_B1D;
pub const SYNC_BITS: u32 = 36;

/// A basic ADS-B message: 18 bytes of data in a 30-byte codeword.
pub const SHORT_DATA_BYTES: usize = 18;
pub const SHORT_BYTES: usize = 30;
/// A long one: 34 bytes of data in 48.
pub const LONG_DATA_BYTES: usize = 34;
pub const LONG_BYTES: usize = 48;
/// The uplink: six interleaved RS(92,72) blocks.
pub const UPLINK_BLOCKS: usize = 6;
pub const UPLINK_BLOCK_BYTES: usize = 92;
pub const UPLINK_BLOCK_DATA_BYTES: usize = 72;
pub const UPLINK_BYTES: usize = UPLINK_BLOCKS * UPLINK_BLOCK_BYTES;
pub const UPLINK_DATA_BYTES: usize = UPLINK_BLOCKS * UPLINK_BLOCK_DATA_BYTES;

/// Whether a receiver tuned here could be hearing UAT, within a channel of
/// the allocation.
pub fn is_uat_band(hz: f64) -> bool {
    (hz - CHANNEL_HZ).abs() < 1_000_000.0
}

/// The codes, all three over GF(256) with the same field polynomial and
/// first root: a shortened RS(255,243), RS(255,241) and RS(255,235).
fn rs_short() -> ReedSolomon {
    ReedSolomon::new(8, 0x187, 120, 1, 12, 225)
}

fn rs_long() -> ReedSolomon {
    ReedSolomon::new(8, 0x187, 120, 1, 14, 207)
}

fn rs_uplink() -> ReedSolomon {
    ReedSolomon::new(8, 0x187, 120, 1, 20, 163)
}

/// A codeword that corrected, and what it cost.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Corrected {
    pub data: Vec<u8>,
    pub errors: usize,
}

/// Correct an ADS-B codeword demodulated as 48 bytes, long form first.
///
/// Long first because a short frame is the first 30 bytes of what the
/// demodulator hands over and the long code would otherwise be tried against
/// 18 bytes of noise. The payload type code says which form it is, so a
/// codeword that corrects but disagrees with its own header is rejected:
/// dump978 refuses the same pair, since the alternative is a long decode of a
/// short frame's trailing air.
pub fn correct_adsb(raw: &[u8]) -> Option<Corrected> {
    if raw.len() >= LONG_BYTES {
        let mut block = raw[..LONG_BYTES].to_vec();
        if let Some(errors) = rs_long().decode(&mut block, &[])
            && block[0] >> 3 != 0
        {
            block.truncate(LONG_DATA_BYTES);
            return Some(Corrected { data: block, errors });
        }
    }
    if raw.len() >= SHORT_BYTES {
        let mut block = raw[..SHORT_BYTES].to_vec();
        if let Some(errors) = rs_short().decode(&mut block, &[])
            && block[0] >> 3 == 0
        {
            block.truncate(SHORT_DATA_BYTES);
            return Some(Corrected { data: block, errors });
        }
    }
    None
}

/// Correct an uplink frame: de-interleave the six blocks, correct each, and
/// return the 432 bytes they carry.
pub fn correct_uplink(raw: &[u8]) -> Option<Corrected> {
    if raw.len() < UPLINK_BYTES {
        return None;
    }
    let rs = rs_uplink();
    let mut data = Vec::with_capacity(UPLINK_DATA_BYTES);
    let mut errors = 0;
    for block in 0..UPLINK_BLOCKS {
        let mut buf: Vec<u8> =
            (0..UPLINK_BLOCK_BYTES).map(|i| raw[i * UPLINK_BLOCKS + block]).collect();
        errors += rs.decode(&mut buf, &[])?;
        buf.truncate(UPLINK_BLOCK_DATA_BYTES);
        data.extend_from_slice(&buf);
    }
    Some(Corrected { data, errors })
}

/// What an address in a header means.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AddressQualifier {
    /// An ICAO address, broadcast by the aircraft itself.
    IcaoAdsb,
    /// An ICAO address a ground station is relaying.
    IcaoTisb,
    /// A ground station's own track file number, which is not an address.
    TisbTrackFile,
    /// A surface vehicle.
    Vehicle,
    /// A fixed beacon on the ground.
    FixedBeacon,
    Reserved(u8),
}

impl AddressQualifier {
    pub fn from_code(c: u8) -> Self {
        match c {
            0 => Self::IcaoAdsb,
            2 => Self::IcaoTisb,
            3 => Self::TisbTrackFile,
            4 => Self::Vehicle,
            5 => Self::FixedBeacon,
            other => Self::Reserved(other),
        }
    }

    pub fn name(&self) -> &'static str {
        match self {
            Self::IcaoAdsb => "ICAO via ADS-B",
            Self::IcaoTisb => "ICAO via TIS-B",
            Self::TisbTrackFile => "TIS-B track file",
            Self::Vehicle => "vehicle",
            Self::FixedBeacon => "fixed beacon",
            Self::Reserved(_) => "reserved",
        }
    }

    /// Whether the address names an aircraft rather than a ground station's
    /// bookkeeping.
    pub fn is_icao(&self) -> bool {
        matches!(self, Self::IcaoAdsb | Self::IcaoTisb)
    }
}

/// Which altimeter an altitude or a vertical rate came from.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AltitudeSource {
    Baro,
    Geo,
}

impl AltitudeSource {
    pub fn name(&self) -> &'static str {
        match self {
            Self::Baro => "barometric",
            Self::Geo => "geometric",
        }
    }
}

/// Whether the transmitter says it is flying.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AirGround {
    Subsonic,
    Supersonic,
    Ground,
    Reserved,
}

/// What an angle in a state vector is an angle of.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TrackKind {
    /// The direction it is moving.
    Track,
    MagneticHeading,
    TrueHeading,
}

impl TrackKind {
    pub fn name(&self) -> &'static str {
        match self {
            Self::Track => "track",
            Self::MagneticHeading => "magnetic heading",
            Self::TrueHeading => "true heading",
        }
    }
}

/// What the aircraft is.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Emitter {
    Unknown,
    Light,
    Medium,
    Large,
    HighVortexLarge,
    Heavy,
    HighlyManoeuvrable,
    Rotorcraft,
    Glider,
    LighterThanAir,
    Parachutist,
    UltraLight,
    Uav,
    Space,
    EmergencyVehicle,
    ServiceVehicle,
    PointObstacle,
    ClusterObstacle,
    LineObstacle,
    Reserved(u8),
}

impl Emitter {
    pub fn from_code(c: u8) -> Self {
        match c {
            0 => Self::Unknown,
            1 => Self::Light,
            2 => Self::Medium,
            3 => Self::Large,
            4 => Self::HighVortexLarge,
            5 => Self::Heavy,
            6 => Self::HighlyManoeuvrable,
            7 => Self::Rotorcraft,
            9 => Self::Glider,
            10 => Self::LighterThanAir,
            11 => Self::Parachutist,
            12 => Self::UltraLight,
            14 => Self::Uav,
            15 => Self::Space,
            17 => Self::EmergencyVehicle,
            18 => Self::ServiceVehicle,
            19 => Self::PointObstacle,
            20 => Self::ClusterObstacle,
            21 => Self::LineObstacle,
            other => Self::Reserved(other),
        }
    }

    pub fn name(&self) -> &'static str {
        match self {
            Self::Unknown => "unknown",
            Self::Light => "light",
            Self::Medium => "medium",
            Self::Large => "large",
            Self::HighVortexLarge => "large, high vortex",
            Self::Heavy => "heavy",
            Self::HighlyManoeuvrable => "highly manoeuvrable",
            Self::Rotorcraft => "rotorcraft",
            Self::Glider => "glider",
            Self::LighterThanAir => "lighter than air",
            Self::Parachutist => "parachutist",
            Self::UltraLight => "ultralight",
            Self::Uav => "uav",
            Self::Space => "spacecraft",
            Self::EmergencyVehicle => "emergency vehicle",
            Self::ServiceVehicle => "service vehicle",
            Self::PointObstacle => "point obstacle",
            Self::ClusterObstacle => "cluster obstacle",
            Self::LineObstacle => "line obstacle",
            Self::Reserved(_) => "reserved",
        }
    }
}

/// What the crew has told the transponder is wrong.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Emergency {
    None,
    General,
    Medical,
    MinimumFuel,
    NoCommunications,
    Unlawful,
    Downed,
    Reserved,
}

impl Emergency {
    pub fn from_code(c: u8) -> Self {
        match c {
            0 => Self::None,
            1 => Self::General,
            2 => Self::Medical,
            3 => Self::MinimumFuel,
            4 => Self::NoCommunications,
            5 => Self::Unlawful,
            6 => Self::Downed,
            _ => Self::Reserved,
        }
    }

    pub fn name(&self) -> &'static str {
        match self {
            Self::None => "none",
            Self::General => "general emergency",
            Self::Medical => "medical",
            Self::MinimumFuel => "minimum fuel",
            Self::NoCommunications => "no communications",
            Self::Unlawful => "unlawful interference",
            Self::Downed => "downed aircraft",
            Self::Reserved => "reserved",
        }
    }
}

/// Where the transmitter is and how it is moving.
#[derive(Clone, Debug, PartialEq)]
pub struct StateVector {
    pub position: Option<(f64, f64)>,
    pub altitude_ft: Option<i32>,
    pub altitude_source: Option<AltitudeSource>,
    /// Navigation integrity category: how far the position may be out.
    pub nic: u8,
    pub air_ground: AirGround,
    pub ground_speed_kt: Option<f64>,
    pub track_deg: Option<f64>,
    pub track_kind: Option<TrackKind>,
    pub vertical_rate_fpm: Option<i32>,
    pub vertical_rate_source: Option<AltitudeSource>,
    /// On the ground, the airframe's size in metres.
    pub dimensions_m: Option<(f64, f64)>,
    pub utc_coupled: bool,
    pub tisb_site_id: u8,
}

/// What the transmitter says it is and what it can do.
#[derive(Clone, Debug, PartialEq)]
pub struct ModeStatus {
    pub callsign: Option<String>,
    /// The eight characters are a squawk code rather than a callsign.
    pub callsign_is_squawk: bool,
    pub emitter: Emitter,
    pub emergency: Emergency,
    pub uat_version: u8,
    pub nac_p: u8,
    pub nac_v: u8,
    pub sil: u8,
    /// The crew has pressed IDENT.
    pub ident_active: bool,
    pub atc_services: bool,
}

/// One aircraft message.
#[derive(Clone, Debug, PartialEq)]
pub struct Adsb {
    pub mdb_type: u8,
    pub address: u32,
    pub qualifier: AddressQualifier,
    pub state: Option<StateVector>,
    pub status: Option<ModeStatus>,
    pub secondary_altitude_ft: Option<i32>,
}

/// One ground station message.
#[derive(Clone, Debug, PartialEq)]
pub struct Uplink {
    pub position: Option<(f64, f64)>,
    /// The station vouches for the position above.
    pub position_valid: bool,
    pub utc_coupled: bool,
    pub slot_id: u8,
    pub tisb_site_id: u8,
    pub frames: Vec<InfoFrame>,
}

/// What an information frame in an uplink carries.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum InfoFrameKind {
    /// Weather and notices: FIS-B.
    Fisb,
    Developmental,
    /// Whether the station is relaying traffic, and from where.
    TisbAdsrStatus,
    Reserved(u8),
}

impl InfoFrameKind {
    pub fn from_code(c: u8) -> Self {
        match c {
            0 => Self::Fisb,
            1 => Self::Developmental,
            15 => Self::TisbAdsrStatus,
            other => Self::Reserved(other),
        }
    }

    pub fn name(&self) -> &'static str {
        match self {
            Self::Fisb => "FIS-B",
            Self::Developmental => "developmental",
            Self::TisbAdsrStatus => "TIS-B/ADS-R status",
            Self::Reserved(_) => "reserved",
        }
    }
}

/// How a FIS-B product's payload is encoded.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ProductFormat {
    Text,
    TextDlac,
    TextGraphic,
    Graphic,
    GraphicDlac,
    Proprietary,
    Developmental,
    Unknown,
}

impl ProductFormat {
    pub fn name(&self) -> &'static str {
        match self {
            Self::Text => "text",
            Self::TextDlac => "text (DLAC)",
            Self::TextGraphic => "text/graphic",
            Self::Graphic => "graphic",
            Self::GraphicDlac => "graphic (DLAC)",
            Self::Proprietary => "proprietary",
            Self::Developmental => "developmental",
            Self::Unknown => "unknown",
        }
    }

    /// Whether the payload is DLAC characters this module can read.
    pub fn is_dlac_text(&self) -> bool {
        matches!(self, Self::TextDlac)
    }
}

/// One FIS-B product, with the time it was issued.
#[derive(Clone, Debug, PartialEq)]
pub struct Fisb {
    /// The product number from the FIS-B registry, which is open: a number
    /// with no name here is still a product.
    pub product_id: u16,
    pub format: ProductFormat,
    pub month_day: Option<(u8, u8)>,
    pub hours: u8,
    pub minutes: u8,
    pub seconds: Option<u8>,
    /// The payload is one segment of a product sent in several.
    pub segmented: bool,
    /// The text, where the product is DLAC characters.
    pub text: Option<String>,
    pub data: Vec<u8>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct InfoFrame {
    pub kind: InfoFrameKind,
    pub fisb: Option<Fisb>,
    pub data: Vec<u8>,
}

/// A corrected UAT payload, told apart by its length: 18 or 34 bytes is an
/// aircraft, 432 a ground station.
#[derive(Clone, Debug, PartialEq)]
pub enum Frame {
    Adsb(Adsb),
    Uplink(Uplink),
}

pub fn parse(data: &[u8]) -> Option<Frame> {
    match data.len() {
        SHORT_DATA_BYTES | LONG_DATA_BYTES => Some(Frame::Adsb(parse_adsb(data))),
        UPLINK_DATA_BYTES => Some(Frame::Uplink(parse_uplink(data))),
        _ => None,
    }
}

/// An angle in 24 bits of a turn, as both halves of a position are keyed.
fn angle(raw: u32) -> f64 {
    raw as f64 * 360.0 / 16_777_216.0
}

pub fn parse_adsb(f: &[u8]) -> Adsb {
    let mdb_type = (f[0] >> 3) & 0x1f;
    let qualifier = AddressQualifier::from_code(f[0] & 7);
    let address = (u32::from(f[1]) << 16) | (u32::from(f[2]) << 8) | u32::from(f[3]);
    // Which elements follow the header is the payload type code's to say,
    // DO-282B table 3-5.
    let has_sv = matches!(mdb_type, 0..=10);
    let has_ms = matches!(mdb_type, 1 | 3);
    let has_aux = matches!(mdb_type, 1 | 2 | 5 | 6);
    Adsb {
        mdb_type,
        address,
        qualifier,
        state: has_sv.then(|| state_vector(f)),
        status: (has_ms && f.len() >= LONG_DATA_BYTES).then(|| mode_status(f)),
        secondary_altitude_ft: (has_aux && f.len() >= LONG_DATA_BYTES)
            .then(|| altitude_ft(((u32::from(f[29]) << 4) | u32::from(f[30] >> 4)) as u16))
            .flatten(),
    }
}

/// The 12-bit altitude both the state vector and the auxiliary one carry:
/// zero is no altitude, and the rest is 25-foot steps from 1000 feet below
/// sea level.
fn altitude_ft(raw: u16) -> Option<i32> {
    (raw != 0).then(|| (i32::from(raw) - 1) * 25 - 1000)
}

fn state_vector(f: &[u8]) -> StateVector {
    let nic = f[11] & 15;
    let raw_lat = (u32::from(f[4]) << 15) | (u32::from(f[5]) << 7) | u32::from(f[6] >> 1);
    let raw_lon = (u32::from(f[6] & 1) << 23)
        | (u32::from(f[7]) << 15)
        | (u32::from(f[8]) << 7)
        | u32::from(f[9] >> 1);
    let position = (nic != 0 || raw_lat != 0 || raw_lon != 0).then(|| {
        let lat = angle(raw_lat);
        let lon = angle(raw_lon);
        (if lat > 90.0 { lat - 180.0 } else { lat }, if lon > 180.0 { lon - 360.0 } else { lon })
    });

    let raw_alt = ((u32::from(f[10]) << 4) | u32::from(f[11] >> 4)) as u16;
    let altitude_ft = altitude_ft(raw_alt);
    let altitude_source =
        altitude_ft.map(|_| if f[9] & 1 == 1 { AltitudeSource::Geo } else { AltitudeSource::Baro });

    let air_ground = match (f[12] >> 6) & 3 {
        0 => AirGround::Subsonic,
        1 => AirGround::Supersonic,
        2 => AirGround::Ground,
        _ => AirGround::Reserved,
    };
    let mut sv = StateVector {
        position,
        altitude_ft,
        altitude_source,
        nic,
        air_ground,
        ground_speed_kt: None,
        track_deg: None,
        track_kind: None,
        vertical_rate_fpm: None,
        vertical_rate_source: None,
        dimensions_m: None,
        utc_coupled: false,
        tisb_site_id: 0,
    };

    let field_a = (u32::from(f[12] & 0x1f) << 6) | u32::from(f[13] >> 2);
    let field_b = (u32::from(f[13] & 3) << 9) | (u32::from(f[14]) << 1) | u32::from(f[15] >> 7);
    match air_ground {
        AirGround::Subsonic | AirGround::Supersonic => {
            // North/south and east/west components, each an eleventh bit of
            // sign and a magnitude one knot above zero. Supersonic keys the
            // same number in four-knot steps.
            let scale = if air_ground == AirGround::Supersonic { 4.0 } else { 1.0 };
            let component = |raw: u32| {
                ((raw & 0x3ff) != 0).then(|| {
                    let v = ((raw & 0x3ff) - 1) as f64 * scale;
                    if raw & 0x400 != 0 { -v } else { v }
                })
            };
            if let (Some(ns), Some(ew)) = (component(field_a), component(field_b)) {
                if ns != 0.0 || ew != 0.0 {
                    // The bearing of the velocity, clockwise from north, so
                    // the north component is atan2's first argument.
                    sv.track_kind = Some(TrackKind::Track);
                    sv.track_deg = Some(ew.atan2(ns).to_degrees().rem_euclid(360.0));
                }
                sv.ground_speed_kt = Some((ns * ns + ew * ew).sqrt().floor());
            }
            let raw_vv = (u32::from(f[15] & 0x7f) << 4) | u32::from(f[16] >> 4);
            if raw_vv & 0x1ff != 0 {
                sv.vertical_rate_source = Some(if raw_vv & 0x400 != 0 {
                    AltitudeSource::Baro
                } else {
                    AltitudeSource::Geo
                });
                let v = ((raw_vv & 0x1ff) as i32 - 1) * 64;
                sv.vertical_rate_fpm = Some(if raw_vv & 0x200 != 0 { -v } else { v });
            }
        }
        AirGround::Ground => {
            if field_a != 0 {
                sv.ground_speed_kt = Some(((field_a & 0x3ff) - 1) as f64);
            }
            sv.track_kind = match (field_b & 0x600) >> 9 {
                1 => Some(TrackKind::Track),
                2 => Some(TrackKind::MagneticHeading),
                3 => Some(TrackKind::TrueHeading),
                _ => None,
            };
            if sv.track_kind.is_some() {
                sv.track_deg = Some((field_b & 0x1ff) as f64 * 360.0 / 512.0);
            }
            // Length in ten-metre steps, width off a table of the sizes an
            // airframe of that length comes in (DO-282B table 3-9).
            const WIDTHS_M: [f64; 16] = [
                11.5, 23.0, 28.5, 34.0, 33.0, 38.0, 39.5, 45.0, 45.0, 52.0, 59.5, 67.0, 72.5, 80.0,
                80.0, 90.0,
            ];
            let length = 15.0 + 10.0 * f64::from((f[15] & 0x38) >> 3);
            sv.dimensions_m = Some((length, WIDTHS_M[((f[15] & 0x78) >> 3) as usize]));
        }
        AirGround::Reserved => {}
    }

    // A ground station relaying somebody else's position says which site it
    // is; an aircraft says whether its clock is coupled to UTC.
    if matches!(f[0] & 7, 2 | 3) {
        sv.tisb_site_id = f[16] & 0x0f;
    } else {
        sv.utc_coupled = f[16] & 0x08 != 0;
    }
    sv
}

/// The eight-character callsign alphabet: three characters in every two
/// bytes, base 40.
const BASE40: [char; 40] = [
    '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I',
    'J', 'K', 'L', 'M', 'N', 'O', 'P', 'Q', 'R', 'S', 'T', 'U', 'V', 'W', 'X', 'Y', 'Z', ' ', ' ',
    '.', '.',
];

fn mode_status(f: &[u8]) -> ModeStatus {
    let mut chars = String::new();
    let v = (u16::from(f[17]) << 8) | u16::from(f[18]);
    let emitter = Emitter::from_code(((v / 1600) % 40) as u8);
    chars.push(BASE40[((v / 40) % 40) as usize]);
    chars.push(BASE40[(v % 40) as usize]);
    for pair in [(f[19], f[20]), (f[21], f[22])] {
        let v = (u16::from(pair.0) << 8) | u16::from(pair.1);
        chars.push(BASE40[((v / 1600) % 40) as usize]);
        chars.push(BASE40[((v / 40) % 40) as usize]);
        chars.push(BASE40[(v % 40) as usize]);
    }
    let callsign = chars.trim_end().to_string();
    ModeStatus {
        callsign_is_squawk: !callsign.is_empty() && f[26] & 0x02 == 0,
        callsign: (!callsign.is_empty()).then_some(callsign),
        emitter,
        emergency: Emergency::from_code((f[23] >> 5) & 7),
        uat_version: (f[23] >> 2) & 7,
        sil: f[23] & 3,
        nac_p: (f[25] >> 4) & 15,
        nac_v: (f[25] >> 1) & 7,
        ident_active: f[26] & 0x10 != 0,
        atc_services: f[26] & 0x08 != 0,
    }
}

pub fn parse_uplink(f: &[u8]) -> Uplink {
    let raw_lat = (u32::from(f[0]) << 15) | (u32::from(f[1]) << 7) | u32::from(f[2] >> 1);
    let raw_lon = (u32::from(f[2] & 1) << 23)
        | (u32::from(f[3]) << 15)
        | (u32::from(f[4]) << 7)
        | u32::from(f[5] >> 1);
    let lat = angle(raw_lat);
    let lon = angle(raw_lon);
    let position = Some((
        if lat > 90.0 { lat - 180.0 } else { lat },
        if lon > 180.0 { lon - 360.0 } else { lon },
    ));
    let app_data_valid = f[6] & 0x20 != 0;
    Uplink {
        position,
        position_valid: f[5] & 1 != 0,
        utc_coupled: f[6] & 0x80 != 0,
        slot_id: f[6] & 0x1f,
        tisb_site_id: f[7] >> 4,
        frames: if app_data_valid { info_frames(&f[8..]) } else { Vec::new() },
    }
}

fn info_frames(mut data: &[u8]) -> Vec<InfoFrame> {
    let mut out = Vec::new();
    while data.len() >= 2 {
        let length = ((usize::from(data[0]) << 1) | usize::from(data[1] >> 7)) & 0x1ff;
        let kind = InfoFrameKind::from_code(data[1] & 0x0f);
        if length == 0 && kind == InfoFrameKind::Fisb {
            // The padding that follows the last frame of an uplink.
            break;
        }
        if data.len() < length + 2 {
            break;
        }
        let body = &data[2..2 + length];
        out.push(InfoFrame {
            kind,
            fisb: (kind == InfoFrameKind::Fisb).then(|| fisb(body)).flatten(),
            data: body.to_vec(),
        });
        data = &data[2 + length..];
    }
    out
}

fn fisb(body: &[u8]) -> Option<Fisb> {
    if body.len() < 4 {
        return None;
    }
    // Two bits say which of the four time stamps follows the header, and so
    // where the product's own payload starts.
    let t_opt = ((body[1] & 0x01) << 1) | (body[2] >> 7);
    let (month_day, hours, minutes, seconds, at) = match t_opt {
        0 => (None, (body[2] & 0x7c) >> 2, ((body[2] & 3) << 4) | (body[3] >> 4), None, 4),
        1 => {
            if body.len() < 5 {
                return None;
            }
            (
                None,
                (body[2] & 0x7c) >> 2,
                ((body[2] & 3) << 4) | (body[3] >> 4),
                Some(((body[3] & 0x0f) << 2) | (body[4] >> 6)),
                5,
            )
        }
        2 => {
            if body.len() < 5 {
                return None;
            }
            (
                Some(((body[2] & 0x78) >> 3, ((body[2] & 7) << 2) | (body[3] >> 6))),
                (body[3] & 0x3e) >> 1,
                ((body[3] & 1) << 5) | (body[4] >> 3),
                None,
                5,
            )
        }
        _ => {
            if body.len() < 6 {
                return None;
            }
            (
                Some(((body[2] & 0x78) >> 3, ((body[2] & 7) << 2) | (body[3] >> 6))),
                (body[3] & 0x3e) >> 1,
                ((body[3] & 1) << 5) | (body[4] >> 3),
                Some(((body[4] & 3) << 3) | (body[5] >> 5)),
                6,
            )
        }
    };
    let product_id = (u16::from(body[0] & 0x1f) << 6) | u16::from(body[1] >> 2);
    let format = product_format(product_id);
    let data = body[at..].to_vec();
    Some(Fisb {
        product_id,
        format,
        month_day,
        hours,
        minutes,
        seconds,
        segmented: body[1] & 0x02 != 0,
        text: format.is_dlac_text().then(|| dlac(&data)),
        data,
    })
}

/// The name the FIS-B registry gives a product number, where it has one.
pub fn product_name(id: u16) -> &'static str {
    match id {
        0 | 20 => "METAR",
        1 | 21 => "TAF",
        2 | 22 => "SIGMET",
        3 | 23 => "convective SIGMET",
        4 | 24 => "AIRMET",
        5 | 25 => "PIREP",
        6 | 26 => "severe weather warning",
        7 | 27 => "winds and temperatures aloft",
        8 => "NOTAM",
        9 => "D-ATIS",
        10 => "TWIP",
        11 => "airspace AIRMET",
        12 => "airspace SIGMET",
        13 => "special use airspace status",
        51..=54 => "national NEXRAD",
        55..=58 => "regional NEXRAD",
        59..=62 => "individual NEXRAD",
        63 | 64 => "block NEXRAD",
        81 | 82 => "radar echo tops",
        83 => "storm tops and velocity",
        101 | 102 => "lightning strikes",
        151 => "point phenomena",
        201 => "surface conditions",
        202 => "surface weather systems",
        254 => "AIRMET/SIGMET bitmap",
        351 => "system time",
        352 => "operational status",
        353 => "ground station status",
        401 => "generic raster",
        402 | 405 | 411 | 413 => "generic text",
        403 => "generic vector",
        404 | 412 => "generic symbolic",
        600 | 2004 => "proprietary",
        2000..=2003 | 2005 => "developmental",
        _ => "unknown",
    }
}

fn product_format(id: u16) -> ProductFormat {
    match id {
        0..=7 | 351..=353 | 402 | 405 => ProductFormat::Text,
        8..=13 => ProductFormat::TextGraphic,
        20..=27 | 411 | 413 => ProductFormat::TextDlac,
        51..=64 | 81..=83 | 101 | 102 | 151 | 201 | 202 | 254 | 401 | 403 | 404 => {
            ProductFormat::Graphic
        }
        412 => ProductFormat::GraphicDlac,
        600 | 2004 => ProductFormat::Proprietary,
        2000..=2003 | 2005 => ProductFormat::Developmental,
        _ => ProductFormat::Unknown,
    }
}

/// The six-bit alphabet FIS-B writes its reports in, three characters to
/// four bytes. Character 28 is a tab whose argument is the number of spaces.
pub fn dlac(data: &[u8]) -> String {
    const ALPHABET: [char; 64] = [
        '\u{3}', 'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J', 'K', 'L', 'M', 'N', 'O', 'P',
        'Q', 'R', 'S', 'T', 'U', 'V', 'W', 'X', 'Y', 'Z', '\u{1a}', '\t', '\u{1e}', '\n', '|', ' ',
        '!', '"', '#', '$', '%', '&', '\'', '(', ')', '*', '+', ',', '-', '.', '/', '0', '1', '2',
        '3', '4', '5', '6', '7', '8', '9', ':', ';', '<', '=', '>', '?',
    ];
    let mut out = String::new();
    let mut tab = false;
    for chunk in data.chunks(3) {
        if chunk.len() < 3 {
            break;
        }
        let word = (u32::from(chunk[0]) << 16) | (u32::from(chunk[1]) << 8) | u32::from(chunk[2]);
        for shift in [18, 12, 6, 0] {
            let ch = ((word >> shift) & 0x3f) as usize;
            if tab {
                for _ in 0..ch {
                    out.push(' ');
                }
                tab = false;
            } else if ch == 28 {
                tab = true;
            } else {
                out.push(ALPHABET[ch]);
            }
        }
    }
    out
}

/// The rows a corrected UAT payload becomes: one for an aircraft, and for a
/// ground station one for the station and one per product it sent.
pub fn decoded(frame: &Frame, bytes: &[u8], center: common::Hz) -> Vec<Decoded> {
    match frame {
        Frame::Adsb(a) => vec![adsb_decoded(a, bytes, center)],
        Frame::Uplink(u) => uplink_decoded(u, bytes, center),
    }
}

pub fn adsb_decoded(a: &Adsb, bytes: &[u8], center: common::Hz) -> Decoded {
    use common::Value;
    let mut fields: Vec<(String, Value)> = Vec::new();
    let address = format!("{:06x}", a.address);
    fields.push(("address".into(), Value::Text(address.clone())));
    fields.push(("address_type".into(), Value::Text(a.qualifier.name().into())));

    let mut position = None;
    let mut altitude_ft = None;
    let mut ground_speed_kt = None;
    let mut track_deg = None;
    let mut vertical_rate_fpm = None;
    if let Some(sv) = &a.state {
        if let Some((lat, lon)) = sv.position {
            fields.push(("lat".into(), Value::Float(round5(lat))));
            fields.push(("lon".into(), Value::Float(round5(lon))));
        }
        if let Some(alt) = sv.altitude_ft {
            altitude_ft = Some(alt);
            fields.push(("altitude_ft".into(), Value::Int(i64::from(alt))));
        }
        if let Some(src) = sv.altitude_source {
            fields.push(("altitude_source".into(), Value::Text(src.name().into())));
        }
        if let Some(v) = sv.ground_speed_kt {
            ground_speed_kt = Some(v);
            fields.push(("ground_speed_kt".into(), Value::Float(round1(v))));
        }
        if let (Some(d), Some(k)) = (sv.track_deg, sv.track_kind) {
            track_deg = Some(d);
            fields.push((k.name().replace(' ', "_"), Value::Float(round1(d))));
        }
        if let Some(v) = sv.vertical_rate_fpm {
            vertical_rate_fpm = Some(v);
            fields.push(("vertical_rate_fpm".into(), Value::Int(i64::from(v))));
        }
        fields.push(("nic".into(), Value::Int(i64::from(sv.nic))));
        position = sv.position.map(|(lat, lon)| common::Position {
            lat,
            lon,
            altitude_m: sv.altitude_ft.map(|ft| f64::from(ft) * 0.3048),
            speed_kt: sv.ground_speed_kt,
            course_deg: sv.track_deg,
        });
    }

    let mut name = None;
    if let Some(ms) = &a.status {
        if let Some(cs) = &ms.callsign {
            let key = if ms.callsign_is_squawk { "squawk" } else { "callsign" };
            if !ms.callsign_is_squawk {
                name = Some(cs.clone());
            }
            fields.push((key.into(), Value::Text(cs.clone())));
        }
        fields.push(("emitter".into(), Value::Text(ms.emitter.name().into())));
        if ms.emergency != Emergency::None {
            fields.push(("emergency".into(), Value::Text(ms.emergency.name().into())));
        }
        if ms.ident_active {
            fields.push(("ident".into(), Value::Bool(true)));
        }
    }
    if let Some(alt) = a.secondary_altitude_ft {
        fields.push(("secondary_altitude_ft".into(), Value::Int(i64::from(alt))));
    }

    // Named for what the frame says, not for its type code: a frame with a
    // position is a position report whichever of the eleven forms carried it.
    let protocol = match (position.is_some(), a.qualifier) {
        (_, AddressQualifier::IcaoTisb | AddressQualifier::TisbTrackFile) => "UAT-TISB",
        (true, _) => "UAT-Position",
        (false, _) => "UAT-Status",
    };
    let detail = fields.iter().map(|(k, v)| format!("{k}={v}")).collect::<Vec<_>>().join(" ");
    let mut d = Decoded::bytes(protocol, center, 0.0, bytes.to_vec())
        .with_detail(detail)
        .with_fields(fields)
        .with_modulation(common::Modulation::Fsk2)
        // Every frame here corrected under its Reed-Solomon code, which is a
        // real integrity check.
        .with_crc(Some(true))
        .reporting(common::ReportDetail::Aircraft {
            altitude_ft,
            ground_speed_kt,
            track_deg,
            vertical_rate_fpm,
            squawk: None,
            wind: None,
            temp_c: None,
            // UAT sends the position itself: nothing to pair up across
            // frames the way 1090 MHz needs.
            cpr: None,
        });
    d.position = position;
    // A track file number is the ground station's bookkeeping and not an
    // address, so it names nobody.
    if a.qualifier.is_icao() || a.qualifier == AddressQualifier::Vehicle {
        d.link = Some(common::Link::beacon(common::Party::unit(address.clone())));
        let mut who = common::Identity::new("uat", address);
        who.name = name;
        d.identity = Some(who);
    }
    d
}

pub fn uplink_decoded(u: &Uplink, bytes: &[u8], center: common::Hz) -> Vec<Decoded> {
    use common::Value;
    let mut fields: Vec<(String, Value)> = Vec::new();
    if let Some((lat, lon)) = u.position {
        fields.push(("lat".into(), Value::Float(round5(lat))));
        fields.push(("lon".into(), Value::Float(round5(lon))));
    }
    fields.push(("position_valid".into(), Value::Bool(u.position_valid)));
    fields.push(("slot".into(), Value::Int(i64::from(u.slot_id))));
    fields.push(("tisb_site".into(), Value::Int(i64::from(u.tisb_site_id))));
    fields.push(("frames".into(), Value::Int(u.frames.len() as i64)));
    let detail = fields.iter().map(|(k, v)| format!("{k}={v}")).collect::<Vec<_>>().join(" ");
    let station = format!("gs{:02}", u.tisb_site_id);
    let mut d = Decoded::bytes("UAT-Uplink", center, 0.0, bytes.to_vec())
        .with_detail(detail)
        .with_fields(fields)
        .with_modulation(common::Modulation::Fsk2)
        .with_crc(Some(true))
        .reporting(common::ReportDetail::Station { aid: false });
    // The station vouches for its own position, so a doubtful one is not
    // plotted.
    if u.position_valid {
        d.position =
            u.position.map(|(lat, lon)| common::Position { lat, lon, ..Default::default() });
    }
    d.identity = Some(common::Identity::new("uat-gs", station));
    let mut out = vec![d];
    out.extend(u.frames.iter().filter_map(|f| product_decoded(f, center)));
    out
}

pub fn product_decoded(f: &InfoFrame, center: common::Hz) -> Option<Decoded> {
    use common::Value;
    let fisb = f.fisb.as_ref()?;
    let mut fields: Vec<(String, Value)> = vec![
        ("product".into(), Value::Text(product_name(fisb.product_id).into())),
        ("product_id".into(), Value::Int(i64::from(fisb.product_id))),
        ("format".into(), Value::Text(fisb.format.name().into())),
        (
            "issued".into(),
            Value::Text(match (fisb.month_day, fisb.seconds) {
                (Some((m, day)), Some(s)) => {
                    format!("{m:02}-{day:02} {:02}:{:02}:{s:02}", fisb.hours, fisb.minutes)
                }
                (Some((m, day)), None) => {
                    format!("{m:02}-{day:02} {:02}:{:02}", fisb.hours, fisb.minutes)
                }
                (None, Some(s)) => format!("{:02}:{:02}:{s:02}", fisb.hours, fisb.minutes),
                (None, None) => format!("{:02}:{:02}", fisb.hours, fisb.minutes),
            }),
        ),
        ("bytes".into(), Value::Int(fisb.data.len() as i64)),
    ];
    let text = fisb.text.as_ref().map(|t| t.trim_end().to_string()).filter(|t| !t.is_empty());
    if let Some(t) = &text {
        fields.push(("text".into(), Value::Text(t.clone())));
    }
    let detail = fields.iter().map(|(k, v)| format!("{k}={v}")).collect::<Vec<_>>().join(" ");
    let mut d = Decoded::bytes("FISB", center, 0.0, fisb.data.clone())
        .with_detail(detail)
        .with_fields(fields)
        .with_modulation(common::Modulation::Fsk2)
        .with_crc(Some(true));
    if let Some(t) = text {
        d.media_type = common::media::TEXT;
        d.text = Some(t);
        // A weather report a machine composed and broadcast to everybody in
        // range. Nobody wrote it and it is addressed to nobody, so it
        // belongs in the packet list and not in the messages.
        d.written = false;
    }
    Some(d)
}

pub fn round1(v: f64) -> f64 {
    (v * 10.0).round() / 10.0
}

pub fn round5(v: f64) -> f64 {
    (v * 100_000.0).round() / 100_000.0
}

/// The bits of a burst as bytes, most significant bit first, which is the
/// order UAT keys them in.
pub fn pack(bits: &[bool]) -> Vec<u8> {
    bits.chunks(8)
        .map(|c| c.iter().enumerate().fold(0u8, |b, (i, &v)| b | (u8::from(v) << (7 - i))))
        .collect()
}

/// The payload a burst carries once its code has corrected it, or nothing
/// where the sync word was noise.
pub fn correct(b: &SyncBurst) -> Option<Corrected> {
    let bytes = pack(&b.bits);
    match b.pattern {
        ADSB => correct_adsb(&bytes),
        UPLINK => correct_uplink(&bytes),
        _ => None,
    }
}

pub fn patterns() -> Vec<SyncPattern> {
    vec![
        SyncPattern {
            word: ADSB_SYNC,
            bits: SYNC_BITS,
            // The long form always: a basic message is the first 30 bytes of
            // it, and which one it was is the payload type code's to say.
            payload_bits: LONG_BYTES * 8,
            max_errors: MAX_SYNC_ERRORS,
        },
        SyncPattern {
            word: UPLINK_SYNC,
            bits: SYNC_BITS,
            payload_bits: UPLINK_BYTES * 8,
            max_errors: MAX_SYNC_ERRORS,
        },
    ]
}

/// Which of the detector's two patterns matched.
pub const ADSB: usize = 0;

pub const UPLINK: usize = 1;

/// How many sync bits may be wrong and the word still be this one.
///
/// Four of 36. The cost of raising it is candidates the Reed-Solomon decode
/// then throws away: measured on synthesised noise, four allowed gives 4
/// candidates in 4.8 million samples and none of them corrects.
pub const MAX_SYNC_ERRORS: u32 = 4;

#[cfg(test)]
mod tests {
    use super::*;

    /// Data followed by its parity, which is what goes on the air: the code
    /// itself returns only the parity symbols.
    fn codeword(rs: &ReedSolomon, data: &[u8]) -> Vec<u8> {
        let mut out = data.to_vec();
        out.extend_from_slice(&rs.encode(data));
        out
    }

    /// Wrap 18 bytes of payload in the short code and read them back, which
    /// is the only way to know the code's parameters are the ones the
    /// standard names.
    #[test]
    fn a_short_codeword_corrects_up_to_six_bytes() {
        let payload: Vec<u8> = (0..SHORT_DATA_BYTES as u8).collect();
        let coded = codeword(&rs_short(), &payload);
        assert_eq!(coded.len(), SHORT_BYTES);
        let mut raw = coded.clone();
        raw.resize(LONG_BYTES, 0);
        let c = correct_adsb(&raw).expect("a clean codeword");
        assert_eq!(c.data, payload);
        assert_eq!(c.errors, 0);

        // Six symbols wrong is the most twelve parity symbols can mend.
        let mut broken = raw.clone();
        for b in broken.iter_mut().take(6) {
            *b ^= 0xff;
        }
        // The header is one of the bytes hit, so what comes back has to be
        // the payload again and not merely something.
        let c = correct_adsb(&broken).expect("six wrong bytes still correct");
        assert_eq!(c.data, payload);
        assert_eq!(c.errors, 6);

        let mut lost = raw.clone();
        for b in lost.iter_mut().take(9) {
            *b ^= 0xff;
        }
        assert_eq!(correct_adsb(&lost), None, "nine wrong bytes is past the code");
    }

    #[test]
    fn a_long_codeword_carries_thirty_four_bytes() {
        let mut payload: Vec<u8> = (0..LONG_DATA_BYTES as u8).collect();
        // A long frame says so in its payload type code.
        payload[0] = 1 << 3;
        let coded = codeword(&rs_long(), &payload);
        assert_eq!(coded.len(), LONG_BYTES);
        let c = correct_adsb(&coded).expect("a clean codeword");
        assert_eq!(c.data, payload);
        assert_eq!(c.errors, 0);
    }

    /// The uplink's six blocks are interleaved byte by byte, so a burst of
    /// errors is spread six ways before any of them is corrected.
    #[test]
    fn an_uplink_frame_deinterleaves_into_six_blocks() {
        let data: Vec<u8> = (0..UPLINK_DATA_BYTES).map(|i| (i % 251) as u8).collect();
        let raw = encode_uplink(&data);
        assert_eq!(raw.len(), UPLINK_BYTES);
        let c = correct_uplink(&raw).expect("a clean frame");
        assert_eq!(c.data, data);
        assert_eq!(c.errors, 0);

        // Sixty consecutive bytes wrong is ten per block, which is what the
        // twenty parity symbols of each can mend.
        let mut burst = raw.clone();
        for b in burst.iter_mut().skip(100).take(60) {
            *b ^= 0x5a;
        }
        let c = correct_uplink(&burst).expect("a burst the interleave spread");
        assert_eq!(c.data, data);
        assert_eq!(c.errors, 60);

        let mut too_much = raw.clone();
        for b in too_much.iter_mut().skip(100).take(140) {
            *b ^= 0x5a;
        }
        assert_eq!(correct_uplink(&too_much), None);
    }

    /// Interleave and encode, the inverse of [`correct_uplink`], for tests
    /// and for anything that wants to make an uplink frame.
    pub(crate) fn encode_uplink(data: &[u8]) -> Vec<u8> {
        let rs = rs_uplink();
        let mut out = vec![0u8; UPLINK_BYTES];
        for block in 0..UPLINK_BLOCKS {
            let from = block * UPLINK_BLOCK_DATA_BYTES;
            let coded = codeword(&rs, &data[from..from + UPLINK_BLOCK_DATA_BYTES]);
            for (i, b) in coded.iter().enumerate() {
                out[i * UPLINK_BLOCKS + block] = *b;
            }
        }
        out
    }

    /// A frame built to the field layout, read back as the aircraft that
    /// built it: type 1 carries the header, the state vector and the mode
    /// status together.
    #[test]
    fn a_long_frame_gives_a_position_an_altitude_and_a_callsign() {
        let f = long_frame();
        let Frame::Adsb(a) = parse(&f).expect("a long frame") else { panic!() };
        assert_eq!(a.mdb_type, 1);
        assert_eq!(a.address, 0xa0_de_ad);
        assert_eq!(a.qualifier, AddressQualifier::IcaoAdsb);
        let sv = a.state.expect("a state vector");
        let (lat, lon) = sv.position.expect("a position");
        assert!((lat - 40.0).abs() < 0.001, "latitude {lat}");
        assert!((lon + 105.0).abs() < 0.001, "longitude {lon}");
        assert_eq!(sv.altitude_ft, Some(9_500));
        assert_eq!(sv.altitude_source, Some(AltitudeSource::Baro));
        assert_eq!(sv.nic, 8);
        assert_eq!(sv.air_ground, AirGround::Subsonic);
        assert_eq!(sv.ground_speed_kt, Some(141.0));
        // 100 knots north and 100 east is northeast.
        assert_eq!(sv.track_deg.map(|d| d.round()), Some(45.0));
        assert_eq!(sv.vertical_rate_fpm, Some(640));
        let ms = a.status.expect("a mode status");
        assert_eq!(ms.callsign.as_deref(), Some("N172SP"));
        assert!(!ms.callsign_is_squawk);
        assert_eq!(ms.emitter, Emitter::Light);
        assert_eq!(ms.emergency, Emergency::None);
    }

    /// The same payload as a basic message: type 0 is the header and the
    /// state vector and stops at 18 bytes.
    #[test]
    fn a_short_frame_is_a_header_and_a_state_vector() {
        let mut f = long_frame();
        f[0] = 0;
        f.truncate(SHORT_DATA_BYTES);
        let Frame::Adsb(a) = parse(&f).expect("a short frame") else { panic!() };
        assert_eq!(a.mdb_type, 0);
        assert!(a.state.is_some());
        assert_eq!(a.status, None, "a basic message has no mode status");
        assert_eq!(a.secondary_altitude_ft, None);
    }

    /// An aircraft: 40 N, 105 W at 9500 feet, 100 knots north and east,
    /// climbing at 640 feet a minute, calling itself N172SP.
    fn long_frame() -> Vec<u8> {
        let mut f = vec![0u8; LONG_DATA_BYTES];
        f[0] = 1 << 3; // type 1, ICAO address via ADS-B
        f[1] = 0xa0;
        f[2] = 0xde;
        f[3] = 0xad;
        let lat = (40.0f64 * 16_777_216.0 / 360.0).round() as u32;
        let lon = ((360.0f64 - 105.0) * 16_777_216.0 / 360.0).round() as u32;
        f[4] = (lat >> 15) as u8;
        f[5] = (lat >> 7) as u8;
        f[6] = ((lat << 1) as u8) | ((lon >> 23) as u8 & 1);
        f[7] = (lon >> 15) as u8;
        f[8] = (lon >> 7) as u8;
        f[9] = (lon << 1) as u8; // barometric altitude
        let alt = ((9_500 + 1000) / 25 + 1) as u16;
        f[10] = (alt >> 4) as u8;
        f[11] = ((alt << 4) as u8) | 8; // NIC 8
        let ns = 100u32 + 1;
        let ew = 100u32 + 1;
        f[12] = (ns >> 6) as u8; // airborne subsonic
        f[13] = ((ns << 2) as u8) | (ew >> 9) as u8;
        f[14] = (ew >> 1) as u8;
        f[15] = (ew << 7) as u8;
        let vv = 640 / 64 + 1;
        f[15] |= (vv >> 4) as u8;
        f[16] = (vv << 4) as u8;
        // Three characters to every two bytes, base 40, and the first of
        // the three in the first pair is the emitter category instead.
        let b40 = |c: char| BASE40.iter().position(|b| *b == c).unwrap() as u16;
        let triple = |a: u16, b: char, c: char| a * 1600 + b40(b) * 40 + b40(c);
        for (at, v) in [
            (17, triple(1, 'N', '1')),
            (19, triple(b40('7'), '2', 'S')),
            (21, triple(b40('P'), ' ', ' ')),
        ] {
            f[at] = (v >> 8) as u8;
            f[at + 1] = v as u8;
        }
        f[26] = 0x02; // the eight characters are a callsign, not a squawk
        f
    }

    /// A ground station's uplink: where it is, and one FIS-B text product.
    #[test]
    fn an_uplink_gives_a_site_a_slot_and_a_metar() {
        let text = "METAR KDEN 041653Z 27012KT 10SM CLR 24/M07 A3002";
        let mut data = vec![0u8; UPLINK_DATA_BYTES];
        let lat = (39.861_67f64 * 16_777_216.0 / 360.0).round() as u32;
        let lon = ((360.0f64 - 104.673) * 16_777_216.0 / 360.0).round() as u32;
        data[0] = (lat >> 15) as u8;
        data[1] = (lat >> 7) as u8;
        data[2] = ((lat << 1) as u8) | ((lon >> 23) as u8 & 1);
        data[3] = (lon >> 15) as u8;
        data[4] = (lon >> 7) as u8;
        data[5] = ((lon << 1) as u8) | 1; // position valid
        data[6] = 0x80 | 0x20 | 9; // UTC coupled, app data valid, slot 9
        data[7] = 2 << 4;

        let payload = dlac_encode(text);
        // Product 413 is generic text in DLAC, timed in hours and minutes.
        let mut body = vec![0u8; 4];
        body[0] = (413 >> 6) as u8;
        body[1] = ((413 & 0x3f) << 2) as u8;
        // 16:53, the six bits of the minute split across the two bytes.
        body[2] = (16 << 2) | (53 >> 4);
        body[3] = (53 & 0x0f) << 4;
        body.extend_from_slice(&payload);
        let length = body.len();
        data[8] = (length >> 1) as u8;
        data[9] = (length << 7) as u8; // FIS-B
        data[10..10 + length].copy_from_slice(&body);

        let raw = encode_uplink(&data);
        let c = correct_uplink(&raw).expect("a clean uplink");
        let Frame::Uplink(u) = parse(&c.data).expect("an uplink") else { panic!() };
        assert!(u.position_valid);
        let (lat, lon) = u.position.expect("a site position");
        assert!((lat - 39.861_67).abs() < 0.001, "latitude {lat}");
        assert!((lon + 104.673).abs() < 0.001, "longitude {lon}");
        assert!(u.utc_coupled);
        assert_eq!(u.slot_id, 9);
        assert_eq!(u.tisb_site_id, 2);
        assert_eq!(u.frames.len(), 1);
        let fisb = u.frames[0].fisb.as_ref().expect("a FIS-B frame");
        assert_eq!(fisb.product_id, 413);
        assert_eq!(fisb.format, ProductFormat::TextDlac);
        assert_eq!(product_name(fisb.product_id), "generic text");
        assert_eq!((fisb.hours, fisb.minutes), (16, 53));
        assert_eq!(fisb.text.as_deref().map(str::trim_end), Some(text));
    }

    /// Pack characters into the six-bit alphabet, the inverse of [`dlac`].
    fn dlac_encode(s: &str) -> Vec<u8> {
        let code = |c: char| -> u32 {
            match c {
                'A'..='Z' => c as u32 - 'A' as u32 + 1,
                ' ' => 32,
                '!'..='?' => c as u32 - ' ' as u32 + 32,
                _ => 32,
            }
        };
        let mut chars: Vec<u32> = s.chars().map(code).collect();
        while !chars.len().is_multiple_of(4) {
            chars.push(32);
        }
        let mut out = Vec::new();
        for w in chars.chunks(4) {
            let word = (w[0] << 18) | (w[1] << 12) | (w[2] << 6) | w[3];
            out.push((word >> 16) as u8);
            out.push((word >> 8) as u8);
            out.push(word as u8);
        }
        out
    }

    /// Nothing is a frame: a length the standard does not key is not read as
    /// the nearest one it does.
    #[test]
    fn a_payload_of_the_wrong_length_is_not_a_frame() {
        assert_eq!(parse(&[0u8; 17]), None);
        assert_eq!(parse(&[0u8; 20]), None);
        assert_eq!(parse(&[0u8; 431]), None);
    }
}
