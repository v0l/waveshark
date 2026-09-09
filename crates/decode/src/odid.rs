//! Open Drone ID: what an aircraft is required to say about itself.
//!
//! ASTM F3411 and ASD-STAN EN 4709-002 define the same message set, and a
//! drone flying under either rule broadcasts it in the clear over Bluetooth
//! advertising or Wi-Fi. The layout here is from `opendroneid/opendroneid-core-c`,
//! which is the reference implementation both specifications point at.
//!
//! A message is exactly 25 bytes: one header byte holding the type and the
//! protocol version, then 24 bytes whose meaning the type decides. A Bluetooth
//! legacy advertisement carries one message inside service data for UUID
//! 0xFFFA behind the application code 0x0D; Bluetooth 5 Long Range and Wi-Fi
//! carry a message pack, which is the same 25 byte messages one after another
//! behind two length bytes. This file reads all of that and nothing about how
//! the bits arrived.
//!
//! # What a decode is evidence of
//!
//! Nothing here is authenticated. There is no CRC of its own (the link layer's
//! is the only integrity check), no signature on an ordinary message, and the
//! serial number, the position and the operator's position are whatever the
//! transmitter chose to put in them. F3411 has an authentication message and
//! the DRIP work builds on it, but almost nothing transmits one, so a row
//! saying an aircraft is at a position means a transmitter claimed it.
//!
//! What keeps that honest is refusing to report a field the specification
//! marks as absent rather than printing its encoded value: an invalid
//! position is exactly 0, an unknown altitude is -1000 m, and both appear
//! constantly because a drone on the bench has no fix. Printing those as a
//! position off the Gulf of Guinea is how a decoder invents evidence.

use common::Value;

/// Every message is this long, in every transport.
pub const MESSAGE_LEN: usize = 25;

/// Service UUID the specification assigns, little endian on air.
pub const SERVICE_UUID: u16 = 0xfffa;

/// The first byte of the service data, which says the rest is Open Drone ID
/// rather than anything else using the same UUID.
pub const APP_CODE: u8 = 0x0d;

/// The OUI ASD-STAN registered, which is what a Wi-Fi beacon's vendor
/// specific element opens with, and the type byte behind it.
///
/// Not to be confused with 6A:5C:35 type 0x01, which is the French national
/// scheme: a different OUI carrying a different, TLV encoded, message set.
pub const WIFI_OUI: [u8; 3] = [0xfa, 0x0b, 0xbc];
pub const WIFI_OUI_TYPE: u8 = 0x0d;

/// The Wi-Fi Alliance OUI and the type that says Neighbour Aware Networking,
/// which is what carries a pack in a public action frame.
const NAN_OUI: [u8; 3] = [0x50, 0x6f, 0x9a];
const NAN_OUI_TYPE: u8 = 0x13;

/// "org.opendroneid.remoteid" hashed. A NAN service descriptor names the
/// service by this rather than by any text, so it is what tells a Remote ID
/// frame from every other NAN service sharing the frame format.
pub const NAN_SERVICE_ID: [u8; 6] = [0x88, 0x69, 0x19, 0x9d, 0x92, 0x09];

/// The attribute id of a NAN service descriptor.
const NAN_SERVICE_DESCRIPTOR: u8 = 0x03;

/// Altitude and height are sent as a half-metre count above this floor, and
/// this exact value is the specification's "unknown".
const ALT_INVALID: f64 = -1000.0;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum IdType {
    None,
    SerialNumber,
    CaaRegistration,
    Utm,
    SessionId,
    Other(u8),
}

impl IdType {
    fn from(v: u8) -> Self {
        match v {
            0 => Self::None,
            1 => Self::SerialNumber,
            2 => Self::CaaRegistration,
            3 => Self::Utm,
            4 => Self::SessionId,
            other => Self::Other(other),
        }
    }

    pub fn name(&self) -> &'static str {
        match self {
            Self::None => "none",
            // ANSI/CTA-2063-A, so a manufacturer code and a serial the
            // airframe carries printed on it.
            Self::SerialNumber => "serial",
            Self::CaaRegistration => "caa registration",
            Self::Utm => "utm uuid",
            Self::SessionId => "session id",
            Self::Other(_) => "unknown",
        }
    }
}

/// What kind of aircraft, as the aircraft says.
pub fn ua_type_name(v: u8) -> &'static str {
    match v {
        0 => "none",
        1 => "aeroplane",
        2 => "multirotor",
        3 => "gyroplane",
        4 => "hybrid lift",
        5 => "ornithopter",
        6 => "glider",
        7 => "kite",
        8 => "free balloon",
        9 => "captive balloon",
        10 => "airship",
        11 => "parachute",
        12 => "rocket",
        13 => "tethered powered",
        14 => "ground obstacle",
        _ => "other",
    }
}

fn status_name(v: u8) -> &'static str {
    match v {
        0 => "undeclared",
        1 => "ground",
        2 => "airborne",
        3 => "emergency",
        4 => "system failure",
        _ => "reserved",
    }
}

#[derive(Clone, Debug, PartialEq)]
pub enum Message {
    /// Who the aircraft is.
    BasicId {
        id_type: IdType,
        ua_type: u8,
        id: String,
    },
    /// Where it is and what it is doing. The one message that repeats at 1 Hz.
    Location(Location),
    /// A signature over earlier messages, in pages. Carried, not checked.
    Authentication {
        auth_type: u8,
        page: u8,
        data: Vec<u8>,
    },
    /// Free text the operator set, such as a flight description.
    SelfId { description_type: u8, text: String },
    /// The operator's own position, and the area a swarm occupies.
    System(System),
    /// The operator's registration, which is the number a regulator can
    /// resolve to a person.
    OperatorId { id_type: u8, id: String },
    /// A type this does not read, kept so the row still says something
    /// arrived.
    Unknown { kind: u8, body: Vec<u8> },
}

impl Message {
    pub fn name(&self) -> &'static str {
        match self {
            Self::BasicId { .. } => "basic id",
            Self::Location(_) => "location",
            Self::Authentication { .. } => "authentication",
            Self::SelfId { .. } => "self id",
            Self::System(_) => "system",
            Self::OperatorId { .. } => "operator id",
            Self::Unknown { .. } => "unknown",
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Location {
    pub status: u8,
    /// Degrees, absent when the transmitter had no fix.
    pub latitude: Option<f64>,
    pub longitude: Option<f64>,
    /// Degrees true, 0-359.
    pub track_deg: Option<u16>,
    pub speed_ms: Option<f64>,
    pub vertical_speed_ms: Option<f64>,
    pub pressure_alt_m: Option<f64>,
    pub geodetic_alt_m: Option<f64>,
    /// Height above the take-off point or the ground, whichever
    /// `height_above_takeoff` says.
    pub height_m: Option<f64>,
    pub height_above_takeoff: bool,
    /// Tenths of a second since the top of the hour, which is all the time
    /// reference a location message carries.
    pub timestamp_tenths: Option<u16>,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct System {
    pub operator_latitude: Option<f64>,
    pub operator_longitude: Option<f64>,
    pub operator_alt_m: Option<f64>,
    pub area_count: u16,
    pub area_radius_m: u32,
    /// The EU category and class from the aircraft's C label, when it carries
    /// one.
    pub classification: Option<(u8, u8)>,
    /// Seconds since 1 January 2019 00:00:00 UTC, the specification's epoch.
    pub timestamp: Option<u32>,
}

/// One message, header byte included.
#[derive(Clone, Debug, PartialEq)]
pub struct Parsed {
    pub version: u8,
    pub message: Message,
}

fn text(bytes: &[u8]) -> String {
    // Padded with NULs by the specification and with spaces by some
    // transmitters. Anything unprintable is a field that was not set.
    let end = bytes.iter().position(|&b| b == 0).unwrap_or(bytes.len());
    String::from_utf8_lossy(&bytes[..end])
        .trim()
        .chars()
        .filter(|c| !c.is_control())
        .collect()
}

fn coord(raw: i32, limit: f64) -> Option<f64> {
    // Exactly zero is the specification's "no fix". A real position that near
    // null island is in the Atlantic and is not what a drone on a bench means.
    if raw == 0 {
        return None;
    }
    let deg = f64::from(raw) * 1e-7;
    // The encoding can hold values no coordinate reaches, and a field left as
    // filler often lands out there.
    (deg.abs() <= limit).then_some(deg)
}

fn altitude(raw: u16) -> Option<f64> {
    let m = f64::from(raw) * 0.5 - 1000.0;
    (m != ALT_INVALID).then_some(m)
}

/// Read one 25 byte message.
pub fn parse_message(m: &[u8]) -> Option<Parsed> {
    if m.len() < MESSAGE_LEN {
        return None;
    }
    let kind = m[0] >> 4;
    let version = m[0] & 0x0f;
    // Versions 0 to 2 are F3411-19 through -22a. A version this has never
    // heard of means the layout below is a guess, so refuse rather than
    // report fields read at the wrong offsets.
    if version > 2 {
        return None;
    }
    let b = &m[1..MESSAGE_LEN];
    let message = match kind {
        0 => Message::BasicId {
            id_type: IdType::from(b[0] >> 4),
            ua_type: b[0] & 0x0f,
            id: text(&b[1..21]),
        },
        1 => {
            let status = b[0] >> 4;
            let ew_segment = b[0] & 0x02 != 0;
            let speed_multiplier = b[0] & 0x01 != 0;
            let track = u16::from(b[1]) + if ew_segment { 180 } else { 0 };
            // 255 is the specification's "unknown" for both of these.
            let speed = (b[2] != 0xff).then(|| {
                if speed_multiplier {
                    f64::from(b[2]) * 0.75 + 255.0 * 0.25
                } else {
                    f64::from(b[2]) * 0.25
                }
            });
            // 63 m/s is the reference library's INV_SPEED_V, not a climb
            // rate: a module with no flight controller sends it constantly.
            let vspeed = (b[3] != 126 && b[3] != 0x7f).then(|| f64::from(b[3] as i8) * 0.5);
            let lat = i32::from_le_bytes([b[4], b[5], b[6], b[7]]);
            let lon = i32::from_le_bytes([b[8], b[9], b[10], b[11]]);
            let ts = u16::from_le_bytes([b[20], b[21]]);
            Message::Location(Location {
                status,
                latitude: coord(lat, 90.0),
                longitude: coord(lon, 180.0),
                track_deg: (track < 360).then_some(track),
                speed_ms: speed,
                vertical_speed_ms: vspeed,
                pressure_alt_m: altitude(u16::from_le_bytes([b[12], b[13]])),
                geodetic_alt_m: altitude(u16::from_le_bytes([b[14], b[15]])),
                height_m: altitude(u16::from_le_bytes([b[16], b[17]])),
                height_above_takeoff: b[0] & 0x04 == 0,
                timestamp_tenths: (ts != 0xffff && ts < 36_000).then_some(ts),
            })
        }
        2 => Message::Authentication {
            auth_type: b[0] >> 4,
            page: b[0] & 0x0f,
            data: b[1..].to_vec(),
        },
        3 => Message::SelfId {
            description_type: b[0],
            text: text(&b[1..24]),
        },
        4 => {
            let lat = i32::from_le_bytes([b[1], b[2], b[3], b[4]]);
            let lon = i32::from_le_bytes([b[5], b[6], b[7], b[8]]);
            let class = b[16];
            let ts = u32::from_le_bytes([b[19], b[20], b[21], b[22]]);
            Message::System(System {
                operator_latitude: coord(lat, 90.0),
                operator_longitude: coord(lon, 180.0),
                operator_alt_m: altitude(u16::from_le_bytes([b[17], b[18]])),
                area_count: u16::from_le_bytes([b[9], b[10]]),
                area_radius_m: u32::from(b[11]) * 10,
                // The low three bits of the flags say which classification
                // scheme is in use; 1 is the EU's, and nothing else has one.
                classification: ((b[0] >> 2) & 0x07 == 1).then_some((class >> 4, class & 0x0f)),
                timestamp: (ts != 0).then_some(ts),
            })
        }
        5 => Message::OperatorId {
            id_type: b[0],
            id: text(&b[1..21]),
        },
        other => Message::Unknown {
            kind: other,
            body: b.to_vec(),
        },
    };
    Some(Parsed { version, message })
}

/// Read a message pack: a header byte, the size of each message, how many
/// there are, then that many messages.
pub fn parse_pack(body: &[u8]) -> Option<Vec<Parsed>> {
    if body.len() < 3 || body[0] >> 4 != 0x0f {
        return None;
    }
    let size = body[1] as usize;
    let count = body[2] as usize;
    // The specification fixes the size and caps the count at nine. A pack
    // saying anything else is a pack this should not walk.
    if size != MESSAGE_LEN || count == 0 || count > 9 || body.len() < 3 + size * count {
        return None;
    }
    (0..count)
        .map(|i| parse_message(&body[3 + i * size..3 + (i + 1) * size]))
        .collect()
}

/// Read whatever an advertisement's service data holds: a single message or a
/// pack, behind the UUID and the application code.
///
/// `value` is the advertising data structure's value for type 0x16, so it
/// opens with the UUID little endian.
pub fn from_service_data(value: &[u8]) -> Option<Vec<Parsed>> {
    if value.len() < 4 {
        return None;
    }
    if u16::from_le_bytes([value[0], value[1]]) != SERVICE_UUID || value[2] != APP_CODE {
        return None;
    }
    // A message counter the transmitter increments, which is not part of the
    // message and is dropped here.
    let body = &value[4..];
    parse_pack(body).or_else(|| parse_message(body).map(|m| vec![m]))
}

/// Read a Wi-Fi beacon's vendor specific element, given the OUI, the vendor's
/// type byte and the bytes behind them.
///
/// A beacon carries either a single message or a pack, so both are tried:
/// the reference transmitter builds a pack and the sample configuration
/// shipped with it broadcasts one location message on its own.
pub fn from_vendor_element(oui: [u8; 3], kind: u8, data: &[u8]) -> Option<Vec<Parsed>> {
    if oui != WIFI_OUI || kind != WIFI_OUI_TYPE || data.is_empty() {
        return None;
    }
    // The first byte is the transmitter's own message counter, which is not
    // part of any message and is dropped here, as it is on Bluetooth.
    let body = &data[1..];
    parse_pack(body).or_else(|| parse_message(body).map(|m| vec![m]))
}

/// Read a message pack out of a Wi-Fi NAN public action frame, given the
/// action frame's category, its code, and the bytes after them.
///
/// The attributes are walked rather than assumed to be in the reference
/// implementation's order, since what has to be found is one service
/// descriptor whose service id is Remote ID's and nothing about the rest.
pub fn from_nan_action(category: u8, code: u8, body: &[u8]) -> Option<Vec<Parsed>> {
    // 0x04 is a public action frame and 0x09 is its vendor specific code.
    if category != 0x04 || code != 0x09 {
        return None;
    }
    if body.len() < 4 || body[..3] != NAN_OUI || body[3] != NAN_OUI_TYPE {
        return None;
    }
    let mut b = &body[4..];
    while b.len() >= 3 {
        let len = u16::from_le_bytes([b[1], b[2]]) as usize;
        let value = b.get(3..3 + len)?;
        // A descriptor is the service id, an instance id, the instance it
        // answers, the service control byte and the length of the service
        // info; the info itself opens with a message counter.
        if b[0] == NAN_SERVICE_DESCRIPTOR && value.len() > 11 && value[..6] == NAN_SERVICE_ID {
            if let Some(pack) = parse_pack(&value[11..]) {
                return Some(pack);
            }
        }
        b = &b[3 + len..];
    }
    None
}

/// The fields a log or a bus carries, in the order they are worth reading.
pub fn fields(messages: &[Parsed]) -> Vec<(String, Value)> {
    let mut f: Vec<(String, Value)> = Vec::new();
    for p in messages {
        // A message whose fields are all unset is still evidence that an
        // aircraft is transmitting, so every message names itself and a row
        // is never blank.
        f.push(("message".into(), Value::Text(p.message.name().into())));
        match &p.message {
            Message::BasicId {
                id_type,
                ua_type,
                id,
            } => {
                f.push(("id_type".into(), Value::Text(id_type.name().into())));
                f.push(("uas_id".into(), Value::Text(id.clone())));
                f.push(("ua_type".into(), Value::Text(ua_type_name(*ua_type).into())));
            }
            Message::Location(l) => {
                f.push(("status".into(), Value::Text(status_name(l.status).into())));
                if let (Some(lat), Some(lon)) = (l.latitude, l.longitude) {
                    f.push(("latitude".into(), Value::Float(lat)));
                    f.push(("longitude".into(), Value::Float(lon)));
                }
                if let Some(a) = l.geodetic_alt_m {
                    f.push(("altitude_m".into(), Value::Float(a)));
                }
                if let Some(h) = l.height_m {
                    let k = if l.height_above_takeoff {
                        "height_above_takeoff_m"
                    } else {
                        "height_agl_m"
                    };
                    f.push((k.into(), Value::Float(h)));
                }
                if let Some(s) = l.speed_ms {
                    f.push(("speed_ms".into(), Value::Float(s)));
                }
                if let Some(v) = l.vertical_speed_ms {
                    f.push(("vertical_speed_ms".into(), Value::Float(v)));
                }
                if let Some(t) = l.track_deg {
                    f.push(("track_deg".into(), Value::Int(i64::from(t))));
                }
            }
            Message::SelfId { text, .. } if !text.is_empty() => {
                f.push(("self_id".into(), Value::Text(text.clone())));
            }
            Message::SelfId { .. } => {}
            Message::System(s) => {
                if let (Some(lat), Some(lon)) = (s.operator_latitude, s.operator_longitude) {
                    f.push(("operator_latitude".into(), Value::Float(lat)));
                    f.push(("operator_longitude".into(), Value::Float(lon)));
                }
                if let Some(a) = s.operator_alt_m {
                    f.push(("operator_altitude_m".into(), Value::Float(a)));
                }
                if s.area_count > 1 {
                    f.push(("area_count".into(), Value::Int(i64::from(s.area_count))));
                    f.push((
                        "area_radius_m".into(),
                        Value::Int(i64::from(s.area_radius_m)),
                    ));
                }
                if let Some((cat, class)) = s.classification {
                    f.push(("eu_category".into(), Value::Int(i64::from(cat))));
                    f.push(("eu_class".into(), Value::Int(i64::from(class))));
                }
            }
            Message::OperatorId { id, .. } if !id.is_empty() => {
                f.push(("operator_id".into(), Value::Text(id.clone())));
            }
            Message::OperatorId { .. } => {}
            Message::Authentication {
                auth_type, page, ..
            } => {
                f.push(("auth_type".into(), Value::Int(i64::from(*auth_type))));
                f.push(("auth_page".into(), Value::Int(i64::from(*page))));
            }
            Message::Unknown { kind, .. } => {
                f.push(("message_type".into(), Value::Int(i64::from(*kind))));
            }
        }
    }
    f
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Build a message the way a transmitter does, so the offsets under test
    /// are the specification's rather than this file's own.
    fn message(kind: u8, body: &[u8]) -> Vec<u8> {
        let mut m = vec![(kind << 4) | 2];
        m.extend_from_slice(body);
        m.resize(MESSAGE_LEN, 0);
        m
    }

    fn basic_id(serial: &str) -> Vec<u8> {
        let mut b = vec![(1 << 4) | 2];
        let mut id = serial.as_bytes().to_vec();
        id.resize(20, 0);
        b.extend_from_slice(&id);
        message(0, &b)
    }

    /// A location message at a position with a known value, so a misread
    /// offset moves it rather than rounding it.
    fn location() -> Vec<u8> {
        let mut b = vec![(2 << 4) | 0x01]; // airborne, speed multiplier set
        b.push(90); // track, no east/west segment
        b.push(20); // speed: 20 * 0.75 + 63.75
        b.push(4i8 as u8); // vertical speed 2 m/s
        b.extend_from_slice(&533_500_000i32.to_le_bytes()); // 53.35 N
        b.extend_from_slice(&(-62_600_000i32).to_le_bytes()); // 6.26 W
        b.extend_from_slice(&2100u16.to_le_bytes()); // pressure 50 m
        b.extend_from_slice(&2200u16.to_le_bytes()); // geodetic 100 m
        b.extend_from_slice(&2100u16.to_le_bytes()); // height 50 m
        b.push(0x22); // accuracies
        b.push(0x22);
        b.extend_from_slice(&12_345u16.to_le_bytes());
        message(1, &b)
    }

    #[test]
    fn a_basic_id_message_reports_the_serial_the_airframe_carries() {
        let p = parse_message(&basic_id("1596F3AAAAAAAAAAAAAA")).expect("a message");
        assert_eq!(p.version, 2);
        match p.message {
            Message::BasicId {
                id_type,
                ua_type,
                id,
            } => {
                assert_eq!(id_type, IdType::SerialNumber);
                assert_eq!(ua_type_name(ua_type), "multirotor");
                assert_eq!(id, "1596F3AAAAAAAAAAAAAA");
            }
            other => panic!("read as {other:?}"),
        }
    }

    #[test]
    fn a_location_message_reports_where_and_how_fast() {
        let p = parse_message(&location()).expect("a message");
        let Message::Location(l) = p.message else {
            panic!("not a location")
        };
        assert_eq!(l.status, 2);
        assert!((l.latitude.unwrap() - 53.35).abs() < 1e-6);
        assert!((l.longitude.unwrap() + 6.26).abs() < 1e-6);
        assert!((l.speed_ms.unwrap() - 78.75).abs() < 1e-9);
        assert!((l.vertical_speed_ms.unwrap() - 2.0).abs() < 1e-9);
        assert!((l.geodetic_alt_m.unwrap() - 100.0).abs() < 1e-9);
        assert_eq!(l.track_deg, Some(90));
        assert_eq!(l.timestamp_tenths, Some(12_345));
    }

    /// A drone with no fix sends zeros and an altitude of -1000 m, which is
    /// the specification saying "unset". Reporting those as a position in the
    /// Atlantic a kilometre underground is a decoder inventing evidence.
    #[test]
    fn an_unset_position_is_absent_rather_than_null_island() {
        let mut b = vec![0x10];
        b.resize(24, 0);
        let p = parse_message(&message(1, &b)).expect("a message");
        let Message::Location(l) = p.message else {
            panic!("not a location")
        };
        assert_eq!(l.latitude, None);
        assert_eq!(l.longitude, None);
        assert_eq!(l.geodetic_alt_m, None);
        assert_eq!(l.height_m, None);
    }

    /// The EU classification only means something when the flags say the EU
    /// scheme is in use, so the same byte under another scheme is not read as
    /// a C class.
    #[test]
    fn a_system_message_reports_the_operator_and_the_eu_class() {
        let mut b = vec![0x04]; // classification type 1, the EU's
        b.extend_from_slice(&533_400_000i32.to_le_bytes());
        b.extend_from_slice(&(-62_500_000i32).to_le_bytes());
        b.extend_from_slice(&1u16.to_le_bytes()); // area count
        b.push(0); // area radius
        b.extend_from_slice(&0u16.to_le_bytes());
        b.extend_from_slice(&0u16.to_le_bytes());
        b.push(0x21); // category 2, class 1
        b.extend_from_slice(&2020u16.to_le_bytes()); // operator altitude 10 m
        b.extend_from_slice(&100u32.to_le_bytes());
        let p = parse_message(&message(4, &b)).expect("a message");
        let Message::System(s) = p.message else {
            panic!("not a system message")
        };
        assert!((s.operator_latitude.unwrap() - 53.34).abs() < 1e-6);
        assert!((s.operator_alt_m.unwrap() - 10.0).abs() < 1e-9);
        assert_eq!(s.classification, Some((2, 1)));

        let mut other = b.clone();
        other[0] = 0x00;
        let p = parse_message(&message(4, &other)).expect("a message");
        let Message::System(s) = p.message else {
            panic!("not a system message")
        };
        assert_eq!(s.classification, None, "no scheme declared, so no class");
    }

    #[test]
    fn service_data_carries_one_message_behind_the_uuid_and_app_code() {
        let mut v = vec![0xfa, 0xff, APP_CODE, 7];
        v.extend_from_slice(&basic_id("OPDRONE1"));
        let msgs = from_service_data(&v).expect("a message");
        assert_eq!(msgs.len(), 1);

        // The same bytes under another service UUID are somebody else's.
        let mut wrong = v.clone();
        wrong[0] = 0xfb;
        assert!(from_service_data(&wrong).is_none());

        // And the UUID without the application code is not enough either.
        let mut no_app = v.clone();
        no_app[2] = 0x00;
        assert!(from_service_data(&no_app).is_none());
    }

    #[test]
    fn a_message_pack_holds_the_whole_set() {
        let mut body = vec![(0x0f << 4) | 2, MESSAGE_LEN as u8, 2];
        body.extend_from_slice(&basic_id("OPDRONE1"));
        body.extend_from_slice(&location());
        let mut v = vec![0xfa, 0xff, APP_CODE, 1];
        v.extend_from_slice(&body);
        let msgs = from_service_data(&v).expect("a pack");
        assert_eq!(msgs.len(), 2);
        let f = fields(&msgs);
        assert!(f.iter().any(|(k, _)| k == "uas_id"));
        assert!(f.iter().any(|(k, _)| k == "latitude"));
    }

    /// A pack claiming more messages than it carries is truncated reception,
    /// and walking it reads whatever follows in memory as a message.
    #[test]
    fn a_pack_that_overruns_is_refused() {
        let mut body = vec![(0x0f << 4) | 2, MESSAGE_LEN as u8, 4];
        body.extend_from_slice(&basic_id("OPDRONE1"));
        assert!(parse_pack(&body).is_none());
    }

    /// The vendor specific element from `beacon.conf` in
    /// `opendroneid/transmitter-linux`, which is what that project feeds to
    /// hostapd to broadcast one location message. Read against their bytes
    /// rather than against bytes written here, so a wrong offset fails.
    #[test]
    fn a_wifi_beacon_element_carries_a_single_message() {
        let ie: Vec<u8> = (0..)
            .step_by(2)
            .take_while(|i| i + 2 <= 60)
            .map(|i| {
                u8::from_str_radix(
                    &"FA0BBC0D00102038000058D6DF1D9055A308820DC10ACF072803D20F0100"[i..i + 2],
                    16,
                )
                .unwrap()
            })
            .collect();
        let msgs = from_vendor_element([ie[0], ie[1], ie[2]], ie[3], &ie[4..]).expect("a message");
        assert_eq!(msgs.len(), 1);
        let Message::Location(l) = &msgs[0].message else {
            panic!("read as {:?}", msgs[0].message)
        };
        assert_eq!(msgs[0].version, 0);
        assert_eq!(l.status, 2);
        assert_eq!(l.track_deg, Some(56));
        assert!((l.latitude.unwrap() - 50.1208664).abs() < 1e-7);
        assert!((l.longitude.unwrap() - 14.4922).abs() < 1e-7);
        assert!((l.geodetic_alt_m.unwrap() - 376.5).abs() < 1e-9);
        assert_eq!(l.timestamp_tenths, Some(4050));
    }

    /// The OUI is what says whose element it is. The French national scheme
    /// uses element 221 too, under 6A:5C:35, and its bytes are not these.
    #[test]
    fn a_vendor_element_under_another_oui_is_not_read() {
        let mut data = vec![0u8];
        data.extend_from_slice(&basic_id("OPDRONE1"));
        assert!(from_vendor_element([0x6a, 0x5c, 0x35], 0x01, &data).is_none());
        assert!(from_vendor_element(WIFI_OUI, 0x01, &data).is_none());
        assert!(from_vendor_element(WIFI_OUI, WIFI_OUI_TYPE, &data).is_some());
    }

    /// A NAN action frame as `odid_wifi_build_message_pack_nan_action_frame`
    /// builds it, from the category byte onward.
    fn nan_action() -> Vec<u8> {
        let mut pack = vec![(0x0f << 4) | 2, MESSAGE_LEN as u8, 2];
        pack.extend_from_slice(&basic_id("OPDRONE1"));
        pack.extend_from_slice(&location());

        let mut sda = NAN_SERVICE_ID.to_vec();
        sda.push(0x01); // instance id
        sda.push(0x00); // the instance this answers, none
        sda.push(0x10); // service control: a follow-up
        sda.push((1 + pack.len()) as u8); // service info length
        sda.push(0x07); // the transmitter's message counter
        sda.extend_from_slice(&pack);

        let mut body = vec![0x50, 0x6f, 0x9a, 0x13];
        body.push(NAN_SERVICE_DESCRIPTOR);
        body.extend_from_slice(&(sda.len() as u16).to_le_bytes());
        body.extend_from_slice(&sda);
        // The service descriptor extension attribute the sender appends,
        // which is walked past rather than assumed to be absent.
        body.extend_from_slice(&[0x0e, 0x04, 0x00, 0x01, 0x00, 0x02, 0x07]);
        body
    }

    #[test]
    fn a_nan_action_frame_carries_a_pack() {
        let msgs = from_nan_action(0x04, 0x09, &nan_action()).expect("a pack");
        assert_eq!(msgs.len(), 2);
        let f = fields(&msgs);
        assert!(f
            .iter()
            .any(|(k, v)| k == "uas_id" && v.to_string() == "OPDRONE1"));
        assert!(f.iter().any(|(k, _)| k == "latitude"));
    }

    /// Every other NAN service uses the same frame and the same attributes,
    /// so the service id is the only thing that says the info behind it is a
    /// message pack and not somebody's file sharing.
    #[test]
    fn another_nan_service_is_not_read_as_a_pack() {
        let mut body = nan_action();
        body[7] ^= 0xff;
        assert!(from_nan_action(0x04, 0x09, &body).is_none());
        assert!(from_nan_action(0x04, 0x08, &nan_action()).is_none());
        assert!(from_nan_action(0x0d, 0x09, &nan_action()).is_none());
    }

    /// An attribute length past the end of the frame is truncated reception,
    /// and walking it reads whatever follows as an attribute.
    #[test]
    fn a_nan_attribute_that_overruns_is_refused() {
        let mut body = nan_action();
        let len = (body.len() as u16) + 8;
        body[5..7].copy_from_slice(&len.to_le_bytes());
        assert!(from_nan_action(0x04, 0x09, &body).is_none());
    }

    /// A version past what is published means the offsets below are a guess.
    #[test]
    fn an_unknown_protocol_version_is_refused() {
        let mut m = basic_id("OPDRONE1");
        m[0] |= 0x0f;
        assert!(parse_message(&m).is_none());
    }
}
