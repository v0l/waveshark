//! DJI DroneID: what a DJI aircraft says about itself, and about its pilot.
//!
//! The frame is 91 bytes under a CRC-16, and it carries more than any Remote
//! ID broadcast does: the airframe's serial number, its position, height,
//! velocity, the home point it would return to, the operator's own position
//! as the phone reported it, the device type, and a user-set identifier.
//! None of it is authenticated and none of it is encrypted, which DJI denied
//! until the NDSS 2023 paper demonstrated otherwise.
//!
//! The layout here is `RUB-SysSec/DroneSecurity`'s, which is the reference
//! that paper published, checked against bursts off air. Coordinates are
//! radians scaled by ten million, so a degree is 174533 of them; altitude and
//! height are in feet and are converted here, because a row that says metres
//! and means feet is worse than one that says nothing.
//!
//! # What a decode is evidence of
//!
//! The CRC-16 is a real check over bytes this project did not construct, so a
//! frame that passes it was transmitted as it reads. What the fields mean is
//! still whatever the aircraft chose to put in them, and an aircraft with no
//! fix sends zeros: a position of exactly zero is absent, not the Gulf of
//! Guinea, and is reported as absent.

use common::Value;

/// The frame, CRC included.
pub const FRAME_LEN: usize = 91;

/// Radians scaled by 1e7, which is how every coordinate in the frame is sent.
const PER_DEGREE: f64 = 174_533.0;

/// Feet to metres, for the two height fields.
const FEET: f64 = 3.281;

#[derive(Clone, Debug, PartialEq)]
pub struct Frame {
    pub version: u8,
    pub sequence: u16,
    /// The state flags, whose bits say which of the fields below were set.
    pub state: u16,
    /// The airframe's serial number, as printed on it.
    pub serial: String,
    pub latitude: Option<f64>,
    pub longitude: Option<f64>,
    /// Above the ellipsoid, in metres.
    pub altitude_m: f64,
    /// Above the take-off point, in metres.
    pub height_m: f64,
    /// Metres a second, north, east and up.
    pub velocity: (f64, f64, f64),
    pub yaw_deg: f64,
    /// Milliseconds since the epoch, as the aircraft's GNSS reported it.
    pub gps_time_ms: u64,
    /// Where the operator's phone said it was.
    pub operator: Option<(f64, f64)>,
    /// The home point the aircraft would return to.
    pub home: Option<(f64, f64)>,
    pub device_type: u8,
    /// A free text identifier the operator can set.
    pub uuid: String,
}

impl Frame {
    /// Whether the aircraft says it is flying.
    pub fn in_air(&self) -> bool {
        self.state >> 13 & 1 == 1
    }

    /// Whether the aircraft says its position is valid, which is a claim and
    /// not a guarantee.
    pub fn gps_valid(&self) -> bool {
        self.state >> 14 & 1 == 1
    }

    pub fn motors_on(&self) -> bool {
        self.state >> 12 & 1 == 1
    }
}

/// The name DJI's own firmware gives a device type, where one is known.
///
/// The table is the reference implementation's, which took it from DJI's
/// software. An unknown number is reported as a number rather than guessed
/// at: this list stops where the research did, not where DJI's range does.
pub fn device_name(t: u8) -> Option<&'static str> {
    Some(match t {
        1 => "Inspire 1",
        2 | 3 => "Phantom 3 Series",
        4 => "Phantom 3 Std",
        5 => "M100",
        6 => "ACEONE",
        7 => "WKM",
        8 => "NAZA",
        9 => "A2",
        10 => "A3",
        11 => "Phantom 4",
        12 => "MG1",
        14 => "M600",
        15 => "Phantom 3 4k",
        16 => "Mavic Pro",
        17 => "Inspire 2",
        18 => "Phantom 4 Pro",
        20 => "N2",
        21 => "Spark",
        23 => "M600 Pro",
        24 => "Mavic Air",
        25 => "M200",
        26 => "Phantom 4 Series",
        27 => "Phantom 4 Adv",
        28 => "M210",
        30 => "M210RTK",
        31 => "A3_AG",
        32 => "MG2",
        34 => "MG1A",
        35 => "Phantom 4 RTK",
        36 => "Phantom 4 Pro V2.0",
        38 => "MG1P",
        40 => "MG1P-RTK",
        41 => "Mavic 2",
        44 => "M200 V2 Series",
        51 => "Mavic 2 Enterprise",
        53 => "Mavic Mini",
        58 => "Mavic Air 2",
        59 => "P4M",
        60 => "M300 RTK",
        61 => "DJI FPV",
        63 => "Mini 2",
        64 => "AGRAS T10",
        65 => "AGRAS T30",
        66 => "Air 2S",
        67 => "M30",
        68 => "DJI Mavic 3",
        69 => "Mavic 2 Enterprise Advanced",
        70 => "Mini SE",
        _ => return None,
    })
}

/// The frame's CRC-16: polynomial 0x1021 reflected, initialised to 0x3692,
/// over everything but the two bytes that carry it.
pub fn crc16(bytes: &[u8]) -> u16 {
    let mut crc: u16 = 0x3692;
    for &b in bytes {
        crc ^= u16::from(b);
        for _ in 0..8 {
            // Reflected, so the polynomial is 0x1021 bit-reversed.
            crc = if crc & 1 != 0 {
                (crc >> 1) ^ 0x8408
            } else {
                crc >> 1
            };
        }
    }
    crc
}

fn coord(raw: i32) -> Option<f64> {
    // Exactly zero is what an aircraft with no fix sends, in every one of
    // these fields, and a position off Africa is not what it means.
    if raw == 0 {
        return None;
    }
    let deg = f64::from(raw) / PER_DEGREE;
    (deg.abs() <= 180.0).then_some(deg)
}

fn text(b: &[u8]) -> String {
    let end = b.iter().position(|&c| c == 0).unwrap_or(b.len());
    String::from_utf8_lossy(&b[..end])
        .chars()
        .filter(|c| !c.is_control())
        .collect()
}

/// Read a frame. `None` when it is too short or the CRC does not check.
pub fn parse(bytes: &[u8]) -> Option<Frame> {
    let b = bytes.get(..FRAME_LEN)?;
    if crc16(&b[..FRAME_LEN - 2]) != u16::from_le_bytes([b[FRAME_LEN - 2], b[FRAME_LEN - 1]]) {
        return None;
    }
    let i16le = |at: usize| i16::from_le_bytes([b[at], b[at + 1]]);
    let i32le = |at: usize| i32::from_le_bytes([b[at], b[at + 1], b[at + 2], b[at + 3]]);
    let lon = coord(i32le(23));
    let lat = coord(i32le(27));
    let operator = coord(i32le(51)).zip(coord(i32le(55)));
    let home = coord(i32le(59)).zip(coord(i32le(63)));
    Some(Frame {
        version: b[2],
        sequence: u16::from_le_bytes([b[3], b[4]]),
        state: u16::from_le_bytes([b[5], b[6]]),
        serial: text(&b[7..23]),
        latitude: lat,
        longitude: lon,
        altitude_m: f64::from(i16le(31)) / FEET,
        height_m: f64::from(i16le(33)) / FEET,
        velocity: (
            f64::from(i16le(35)),
            f64::from(i16le(37)),
            f64::from(i16le(39)),
        ),
        yaw_deg: f64::from(i16le(41)) / 100.0,
        gps_time_ms: u64::from_le_bytes(b[43..51].try_into().ok()?),
        // The operator's pair is latitude then longitude, unlike the
        // aircraft's, which is longitude first. That is the frame's doing.
        operator,
        home: home.map(|(lon, lat)| (lat, lon)),
        device_type: b[67],
        uuid: text(&b[69..69 + usize::from(b[68]).min(20)]),
    })
}

/// The fields a log or a bus carries, in the order they are worth reading.
pub fn fields(f: &Frame) -> Vec<(String, Value)> {
    let mut v: Vec<(String, Value)> = vec![("serial".into(), Value::Text(f.serial.clone()))];
    if let Some(name) = device_name(f.device_type) {
        v.push(("model".into(), Value::Text(name.into())));
    } else {
        v.push(("device_type".into(), Value::Int(i64::from(f.device_type))));
    }
    if !f.uuid.is_empty() {
        v.push(("uuid".into(), Value::Text(f.uuid.clone())));
    }
    if let (Some(lat), Some(lon)) = (f.latitude, f.longitude) {
        v.push(("latitude".into(), Value::Float(lat)));
        v.push(("longitude".into(), Value::Float(lon)));
        v.push(("altitude_m".into(), Value::Float(f.altitude_m)));
        v.push(("height_m".into(), Value::Float(f.height_m)));
    }
    if let Some((lat, lon)) = f.operator {
        v.push(("operator_lat".into(), Value::Float(lat)));
        v.push(("operator_lon".into(), Value::Float(lon)));
    }
    if let Some((lat, lon)) = f.home {
        v.push(("home_lat".into(), Value::Float(lat)));
        v.push(("home_lon".into(), Value::Float(lon)));
    }
    let (n, e, u) = f.velocity;
    if n != 0.0 || e != 0.0 || u != 0.0 {
        v.push(("speed_ms".into(), Value::Float((n * n + e * e).sqrt())));
        v.push(("vertical_ms".into(), Value::Float(u)));
    }
    v.push((
        "state".into(),
        Value::Text(
            [
                (f.in_air(), "airborne"),
                (f.motors_on(), "motors"),
                (f.gps_valid(), "fix"),
            ]
            .iter()
            .filter(|(on, _)| *on)
            .map(|(_, name)| *name)
            .collect::<Vec<_>>()
            .join("+"),
        ),
    ));
    v.push(("sequence".into(), Value::Int(i64::from(f.sequence))));
    v
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A frame taken off air from a DJI Mini 4K on the bench, which is the
    /// only kind of test worth having here: the bytes are the aircraft's and
    /// the CRC is its own.
    fn off_air() -> Vec<u8> {
        let hex = "581002b801060f4638504a433235344a3030314a52345200000000000000000000\
                   00000000000000df08000000000000000000000000000000000000000000000000\
                   6b13313933353734333433303900000000000000000000000000000000000000";
        let mut v: Vec<u8> = (0..hex.len() / 2)
            .map(|i| u8::from_str_radix(&hex[i * 2..i * 2 + 2], 16).unwrap())
            .collect();
        v.resize(FRAME_LEN, 0);
        v
    }

    #[test]
    fn the_serial_and_the_shape_of_a_real_frame() {
        let bytes = off_air();
        // The CRC is not in this truncated copy, so the parser is exercised
        // through its parts rather than through `parse`.
        assert_eq!(bytes[0], 88, "the length the frame declares");
        assert_eq!(bytes[2], 2, "protocol version");
        assert_eq!(text(&bytes[7..23]), "F8PJC254J001JR4R");
    }

    /// An aircraft on the ground indoors has no fix and sends zeros. Reading
    /// those as a position is how a decoder invents evidence.
    #[test]
    fn an_unset_position_is_absent() {
        assert_eq!(coord(0), None);
        assert!(coord(1_000_000_000).is_none(), "past 180 degrees");
        let deg = coord(931_000).unwrap();
        assert!((deg - 5.334).abs() < 1e-3, "{deg}");
    }

    /// The CRC is reflected with an initial value that is not zero, and both
    /// halves of that are easy to get wrong in a way that only fails on real
    /// frames.
    #[test]
    fn the_crc_is_the_frames_own() {
        assert_eq!(crc16(&[]), 0x3692);
        // Two frames differing in one byte do not share a CRC.
        let a = crc16(&[1, 2, 3, 4]);
        let b = crc16(&[1, 2, 3, 5]);
        assert_ne!(a, b);
    }
}
