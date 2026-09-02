//! Comm-B registers: what a Mode S transponder answers a radar with.
//!
//! DF20 and DF21 carry a 56 bit MB field holding one BDS register, and which
//! register it is appears nowhere in the frame. The interrogator knows because
//! it asked; a listener has to work it out from the contents. Two of them say
//! so themselves in their first byte (1,0 and 2,0); the rest are identified by
//! whether every status bit, sign bit and range in the frame is consistent
//! with that register's layout and with an aircraft that could exist.
//!
//! That is a guess, so this module only reports one when exactly one register
//! fits. Where two do, the frame is dropped rather than attributed: a wind
//! reading filed under the wrong register is worse than no wind reading, and
//! the aircraft will send another in a few seconds.
//!
//! The layouts and the plausibility limits follow ICAO Doc 9871 and pyModeS,
//! which is the reference every open implementation is checked against. The
//! test vectors below come from its own test suite.
//!
//! # What this is for
//!
//! BDS 4,4 is the reason. It is a Meteorological Routine Air Report: wind
//! speed and direction, static air temperature, sometimes pressure and
//! humidity, measured by an aircraft at altitude and sent in the clear. A
//! receiver on the ground gets a wind and temperature profile of the sky above
//! it for the price of decoding frames it was already hearing.

/// A decoded Comm-B register.
#[derive(Clone, Debug, PartialEq)]
pub enum Report {
    /// BDS 1,0. Says what the transponder can do, and nothing about the
    /// flight. Worth identifying so it is not mistaken for something else.
    Capability { subnetwork_version: u8 },
    /// BDS 2,0. The callsign, from an aircraft that may never broadcast one.
    Identification { callsign: String },
    /// BDS 4,0, what the crew has set the autopilot to.
    VerticalIntent {
        selected_altitude_ft: Option<i32>,
        fms_altitude_ft: Option<i32>,
        /// Pressure set on the altimeter, in millibars.
        qnh_mb: Option<f64>,
    },
    /// BDS 4,4, the meteorological report.
    Meteo(Meteo),
    /// BDS 5,0, track and turn.
    TrackTurn {
        roll_deg: Option<f64>,
        track_deg: Option<f64>,
        ground_speed_kt: Option<f64>,
        track_rate_deg_s: Option<f64>,
        true_airspeed_kt: Option<f64>,
    },
    /// BDS 6,0, heading and speed.
    HeadingSpeed {
        heading_deg: Option<f64>,
        indicated_airspeed_kt: Option<f64>,
        mach: Option<f64>,
        baro_vertical_rate_fpm: Option<i32>,
        inertial_vertical_rate_fpm: Option<i32>,
    },
}

/// Weather at the aircraft, from BDS 4,4.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Meteo {
    /// How the position the report belongs to was fixed: 1 INS, 2 GNSS,
    /// 3 DME/DME, 4 VOR/DME. Zero means the report has no position source.
    pub figure_of_merit: u8,
    pub wind_kt: Option<f64>,
    /// Degrees true, the direction the wind is coming from.
    pub wind_dir_deg: Option<f64>,
    /// Static air temperature in degrees Celsius. Always present: the bit
    /// that would be its status is the sign.
    pub temp_c: f64,
    pub pressure_hpa: Option<u16>,
    /// 0 none, 1 light, 2 moderate, 3 severe.
    pub turbulence: Option<u8>,
    pub humidity_pct: Option<f64>,
}

impl Report {
    /// The register this came from, as it is written in the standard.
    pub fn bds(&self) -> &'static str {
        match self {
            Self::Capability { .. } => "1,0",
            Self::Identification { .. } => "2,0",
            Self::VerticalIntent { .. } => "4,0",
            Self::Meteo(_) => "4,4",
            Self::TrackTurn { .. } => "5,0",
            Self::HeadingSpeed { .. } => "6,0",
        }
    }
}

/// Work out which register a 56 bit MB field holds, and decode it.
///
/// `mb` is the seven bytes after the frame's header, which for DF20 and DF21
/// is bytes 4 to 10 inclusive. Returns `None` for an empty field, for one that
/// fits no register, and for one that fits more than one.
pub fn infer(mb: &[u8]) -> Option<Report> {
    if mb.len() < 7 {
        return None;
    }
    let p = mb.iter().take(7).fold(0u64, |acc, b| (acc << 8) | *b as u64);
    if p == 0 {
        return None;
    }

    // Two registers name themselves in their first byte. Believing that is
    // still a guess, because any 56 bits can start with 0x20, so the rest of
    // the register has to check out as well.
    if let Some(r) = bds10(p) {
        return Some(r);
    }
    if let Some(r) = bds20(p) {
        return Some(r);
    }

    // The rest are told apart by whether they are self-consistent. Several
    // frames satisfy two of these, and there is no way to choose from one
    // frame alone.
    let mut found = None;
    for r in [bds40(p), bds44(p), bds50(p), bds60(p)].into_iter().flatten() {
        if found.is_some() {
            return None;
        }
        found = Some(r);
    }
    found
}

/// Bits `[from, to]` of the 56 bit field, numbered from zero at the top, the
/// way the register tables in Doc 9871 are.
fn bits(p: u64, from: u32, to: u32) -> u64 {
    let width = to - from + 1;
    (p >> (55 - to)) & ((1u64 << width) - 1)
}

fn bit(p: u64, at: u32) -> bool {
    bits(p, at, at) != 0
}

/// Sign and magnitude, which is what these registers use rather than two's
/// complement: a set sign bit with a zero magnitude is the largest negative
/// value, not minus zero.
fn signed(mag: u64, width: u32, negative: bool) -> f64 {
    if negative {
        mag as f64 - (1u64 << width) as f64
    } else {
        mag as f64
    }
}

/// A status bit that is clear while the field it guards is not means this is
/// not the register it looks like.
fn wrong_status(p: u64, status: u32, from: u32, to: u32) -> bool {
    !bit(p, status) && bits(p, from, to) != 0
}

fn wrap360(deg: f64) -> f64 {
    deg.rem_euclid(360.0)
}

fn bds10(p: u64) -> Option<Report> {
    if bits(p, 0, 7) != 0x10 || bits(p, 9, 13) != 0 {
        return None;
    }
    let overlay = bit(p, 14);
    let version = bits(p, 16, 22) as u8;
    // Overlay command capability arrived with subnetwork 5, so the two
    // disagreeing means this is not a capability report.
    if overlay != (version >= 5) {
        return None;
    }
    Some(Report::Capability { subnetwork_version: version })
}

/// The six bit alphabet callsigns are packed in, shared with ADS-B.
const CHARSET: &[u8; 64] = b"#ABCDEFGHIJKLMNOPQRSTUVWXYZ##### ###############0123456789######";

fn bds20(p: u64) -> Option<Report> {
    if bits(p, 0, 7) != 0x20 {
        return None;
    }
    let mut s = String::with_capacity(8);
    for i in 0..8 {
        let c = CHARSET[bits(p, 8 + i * 6, 13 + i * 6) as usize];
        // A character outside the alphabet means the field is not a callsign,
        // which is the main thing keeping noise out of this register.
        if c == b'#' {
            return None;
        }
        s.push(c as char);
    }
    Some(Report::Identification { callsign: s.trim_end().to_string() })
}

fn bds40(p: u64) -> Option<Report> {
    if wrong_status(p, 0, 1, 12)
        || wrong_status(p, 13, 14, 25)
        || wrong_status(p, 26, 27, 38)
        || wrong_status(p, 47, 48, 50)
        || wrong_status(p, 53, 54, 55)
        || bits(p, 39, 46) != 0
        || bits(p, 51, 52) != 0
    {
        return None;
    }
    let mcp = bit(p, 0).then(|| bits(p, 1, 12) as i32 * 16);
    let fms = bit(p, 13).then(|| bits(p, 14, 25) as i32 * 16);
    let qnh = bit(p, 26).then(|| bits(p, 27, 38) as f64 * 0.1 + 800.0);
    if mcp.is_none() && fms.is_none() && qnh.is_none() {
        return None;
    }
    Some(Report::VerticalIntent { selected_altitude_ft: mcp, fms_altitude_ft: fms, qnh_mb: qnh })
}

fn bds44(p: u64) -> Option<Report> {
    let fom = bits(p, 0, 3) as u8;
    // Only five sources are defined, and a report with no wind in it is not
    // worth the risk of having guessed the register wrong.
    if fom > 4 || !bit(p, 4) {
        return None;
    }
    if wrong_status(p, 34, 35, 45) || wrong_status(p, 46, 47, 48) || wrong_status(p, 49, 50, 55) {
        return None;
    }
    let wind = bits(p, 5, 13);
    let dir_raw = bits(p, 14, 22);
    let temp_raw = bits(p, 24, 33);
    if wind > 250 {
        return None;
    }
    // Bit 23 is the temperature's sign, not a status bit: the field is always
    // present, which is why the range check below is the only thing standing
    // between a misread frame and a plausible-looking temperature.
    let temp_c = signed(temp_raw, 10, bit(p, 23)) * 0.25;
    if !(-80.0..=60.0).contains(&temp_c) {
        return None;
    }
    if wind == 0 && dir_raw == 0 && temp_raw == 0 {
        return None;
    }
    Some(Report::Meteo(Meteo {
        figure_of_merit: fom,
        wind_kt: Some(wind as f64),
        wind_dir_deg: Some(dir_raw as f64 * (180.0 / 256.0)),
        temp_c,
        pressure_hpa: bit(p, 34).then(|| bits(p, 35, 45) as u16),
        turbulence: bit(p, 46).then(|| bits(p, 47, 48) as u8),
        humidity_pct: bit(p, 49).then(|| bits(p, 50, 55) as f64 * (100.0 / 64.0)),
    }))
}

fn bds50(p: u64) -> Option<Report> {
    if wrong_status(p, 0, 1, 10)
        || wrong_status(p, 11, 12, 22)
        || wrong_status(p, 23, 24, 33)
        || wrong_status(p, 34, 35, 44)
        || wrong_status(p, 45, 46, 55)
    {
        return None;
    }
    let roll = bit(p, 0).then(|| signed(bits(p, 2, 10), 9, bit(p, 1)) * 45.0 / 256.0);
    let gs = bit(p, 23).then(|| bits(p, 24, 33) as f64 * 2.0);
    let tas = bit(p, 45).then(|| bits(p, 46, 55) as f64 * 2.0);
    // An airliner past 35 degrees of bank, or past 600 knots, is a frame that
    // belongs to another register. The wire format allows both.
    if roll.is_some_and(|r| r.abs() > 35.0)
        || gs.is_some_and(|v| v > 600.0)
        || tas.is_some_and(|v| v > 600.0)
    {
        return None;
    }
    // True airspeed and ground speed differ by the wind, which is tens of
    // knots, not hundreds.
    if let (Some(g), Some(t)) = (gs, tas) {
        if (t - g).abs() > 200.0 {
            return None;
        }
    }
    if roll.is_none() && gs.is_none() && tas.is_none() {
        return None;
    }
    Some(Report::TrackTurn {
        roll_deg: roll,
        track_deg: bit(p, 11)
            .then(|| wrap360(signed(bits(p, 13, 22), 10, bit(p, 12)) * 90.0 / 512.0)),
        ground_speed_kt: gs,
        track_rate_deg_s: bit(p, 34)
            .then(|| signed(bits(p, 36, 44), 9, bit(p, 35)) * 8.0 / 256.0),
        true_airspeed_kt: tas,
    })
}

fn bds60(p: u64) -> Option<Report> {
    if wrong_status(p, 0, 1, 11)
        || wrong_status(p, 12, 13, 22)
        || wrong_status(p, 23, 24, 33)
        || wrong_status(p, 34, 35, 44)
        || wrong_status(p, 45, 46, 55)
    {
        return None;
    }
    let ias = bit(p, 12).then(|| bits(p, 13, 22) as f64);
    let mach = bit(p, 23).then(|| bits(p, 24, 33) as f64 * (2.048 / 512.0));
    let baro = bit(p, 34).then(|| (signed(bits(p, 36, 44), 9, bit(p, 35)) * 32.0) as i32);
    let inertial = bit(p, 45).then(|| (signed(bits(p, 47, 55), 9, bit(p, 46)) * 32.0) as i32);
    if ias.is_some_and(|v| v > 500.0)
        || mach.is_some_and(|m| m > 1.0)
        || baro.is_some_and(|v| v.abs() > 6000)
        || inertial.is_some_and(|v| v.abs() > 6000)
    {
        return None;
    }
    if ias.is_none() && mach.is_none() && baro.is_none() && inertial.is_none() {
        return None;
    }
    Some(Report::HeadingSpeed {
        heading_deg: bit(p, 0)
            .then(|| wrap360(signed(bits(p, 2, 11), 10, bit(p, 1)) * 90.0 / 512.0)),
        indicated_airspeed_kt: ias,
        mach,
        baro_vertical_rate_fpm: baro,
        inertial_vertical_rate_fpm: inertial,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The MB field of a 112 bit frame: bytes 4 to 10.
    fn mb(hex: &str) -> Vec<u8> {
        let bytes: Vec<u8> = (0..hex.len() / 2)
            .map(|i| u8::from_str_radix(&hex[i * 2..i * 2 + 2], 16).unwrap())
            .collect();
        bytes[4..11].to_vec()
    }

    /// Frames from pyModeS's own test suite, with the values it decodes.
    #[test]
    fn a_meteorological_report_gives_wind_and_temperature() {
        let Some(Report::Meteo(m)) = infer(&mb("A0001692185BD5CF400000DFC696")) else {
            panic!("not read as a meteorological report")
        };
        assert_eq!(m.wind_kt, Some(22.0));
        assert!((m.wind_dir_deg.unwrap() - 344.5).abs() < 0.5);
        assert!((m.temp_c + 48.75).abs() < 0.1, "temperature {}", m.temp_c);
        assert_eq!(m.figure_of_merit, 1);
        assert_eq!(m.pressure_hpa, None);
        assert_eq!(m.humidity_pct, None);
    }

    #[test]
    fn a_track_and_turn_report_decodes() {
        let Some(Report::TrackTurn {
            roll_deg,
            track_deg,
            ground_speed_kt,
            track_rate_deg_s,
            true_airspeed_kt,
        }) = infer(&mb("A000139381951536E024D4CCF6B5"))
        else {
            panic!("not read as a track and turn report")
        };
        assert!((roll_deg.unwrap() - 2.1).abs() < 0.1);
        assert!((track_deg.unwrap() - 114.258).abs() < 0.01);
        assert_eq!(ground_speed_kt, Some(438.0));
        assert!((track_rate_deg_s.unwrap() - 0.125).abs() < 0.01);
        assert_eq!(true_airspeed_kt, Some(424.0));
    }

    #[test]
    fn a_heading_and_speed_report_decodes() {
        let Some(Report::HeadingSpeed {
            heading_deg,
            indicated_airspeed_kt,
            mach,
            baro_vertical_rate_fpm,
            inertial_vertical_rate_fpm,
        }) = infer(&mb("A00004128F39F91A7E27C46ADC21"))
        else {
            panic!("not read as a heading and speed report")
        };
        assert!((heading_deg.unwrap() - 42.715).abs() < 0.01);
        assert_eq!(indicated_airspeed_kt, Some(252.0));
        assert!((mach.unwrap() - 0.42).abs() < 0.005);
        assert_eq!(baro_vertical_rate_fpm, Some(-1920));
        assert_eq!(inertial_vertical_rate_fpm, Some(-1920));
    }

    #[test]
    fn a_callsign_register_names_the_aircraft() {
        assert_eq!(
            infer(&mb("A0001838201584F23468207CDFA5")),
            Some(Report::Identification { callsign: "EXS2MF".into() })
        );
    }

    #[test]
    fn a_selected_altitude_register_decodes() {
        let Some(Report::VerticalIntent { selected_altitude_ft, qnh_mb, .. }) =
            infer(&mb("A000029C85E42F313000007047D3"))
        else {
            panic!("not read as a vertical intent report")
        };
        assert_eq!(selected_altitude_ft, Some(3008));
        assert!((qnh_mb.unwrap() - 1020.0).abs() < 0.05);
    }

    #[test]
    fn an_empty_field_is_not_a_register() {
        assert_eq!(infer(&[0u8; 7]), None);
        assert_eq!(infer(&[0xff]), None);
    }

    #[test]
    fn a_field_that_fits_two_registers_is_not_guessed_at() {
        // Only the heading and roll status bits set, with everything they
        // guard left at zero. That is consistent with 5,0 and with 6,0, and
        // nothing in the frame says which. Reporting a heading of 0 degrees
        // as a roll of 0 degrees is the kind of mistake that ends up in a
        // wind field.
        let mut p = [0u8; 7];
        p[0] = 0x80;
        assert_eq!(infer(&p), None);
    }

    #[test]
    fn a_status_bit_that_contradicts_its_field_rejects_the_register() {
        // Ground speed of 200 kt with its status bit clear. The layout says
        // that cannot happen, so this is not a 5,0.
        let p = (100u64) << (55 - 33);
        assert_eq!(bds50(p), None);
    }
}
