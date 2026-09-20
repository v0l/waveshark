//! Meteo-Radiy MRZ radiosondes, the MP3-H1 airframe.
//!
//! The Russian sonde, and the only one here that sends its position the way
//! its receiver computes it: metres from the centre of the earth, which
//! [`crate::geo`] turns into a latitude and a longitude. Some firmwares send
//! degrees instead, and which one it is shows in the frame: the older
//! layout leaves `0xFFFF` where the newer one has satellites, and the frame
//! is three bytes shorter.
//!
//! A frame carries one word of the sonde's configuration, numbered by a
//! counter, so the serial number and the date arrive a word at a time over
//! sixteen seconds and are gathered the way a DFM's are.
//!
//! Frame layout, the reversed CRC and the configuration numbering are from
//! zilog80's `rs1729/RS`, `demod/mod/mp3h1mod.c`.

use crate::bits::crc16le;
use crate::geo::{ecef_to_geodetic, ecef_velocity_to_enu};
use common::packet::{Entity, Fact, Id, Named, Proto, ThingKind};

/// The bytes every frame starts with: a marker, a subtype and a length.
pub const SYNC: [u8; 3] = [0xAA, 0xBF, 0x35];

/// Bytes the check covers, in each layout, counted from the counter nibble.
const CHECKED_ECEF: usize = 45;
const CHECKED_LATLON: usize = 42;

/// Frame length in each layout: what the check covers, the three bytes in
/// front of it, the two of check and the tail.
pub const FRAME_ECEF: usize = CHECKED_ECEF + 6;
pub const FRAME_LATLON: usize = CHECKED_LATLON + 6;

/// Configuration words gathered beside a frame: the two serial numbers and
/// the date.
const EXTRA: usize = 3 * 4;

/// A gathered record: the frame as it arrived, then the configuration words
/// that name the sonde.
pub const RECORD_ECEF: usize = FRAME_ECEF + EXTRA;
pub const RECORD_LATLON: usize = FRAME_LATLON + EXTRA;

/// Which layout a frame is in.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Layout {
    /// Position as earth-centred metres, with a velocity and a satellite
    /// count.
    Ecef,
    /// Position as degrees, three bytes shorter and with no vertical speed.
    LatLon,
}

impl Layout {
    fn of(frame: &[u8]) -> Layout {
        // The older layout has nothing to say where the newer one counts
        // satellites, and says it as two bytes of ones.
        match frame.get(30..32) {
            Some([0xFF, 0xFF]) => Layout::LatLon,
            _ => Layout::Ecef,
        }
    }

    fn frame_len(self) -> usize {
        match self {
            Layout::Ecef => FRAME_ECEF,
            Layout::LatLon => FRAME_LATLON,
        }
    }

    fn checked(self) -> usize {
        match self {
            Layout::Ecef => CHECKED_ECEF,
            Layout::LatLon => CHECKED_LATLON,
        }
    }
}

/// Whether a frame's own check holds, and in which layout.
///
/// The check is a CRC-16 run from the least significant end, over everything
/// from the counter nibble to the two bytes that carry it. There is no error
/// correction in this sonde, so a frame that fails is dropped.
pub fn check(frame: &[u8]) -> Option<Layout> {
    if frame.len() < FRAME_LATLON || frame[..3] != SYNC {
        return None;
    }
    let layout = Layout::of(frame);
    let len = layout.checked();
    if frame.len() < len + 5 {
        return None;
    }
    let sent = u16::from(frame[len + 3]) | u16::from(frame[len + 4]) << 8;
    (sent == crc16le(&frame[3..3 + len], 0x8005, 0xFFFF)).then_some(layout)
}

/// What a sonde has said so far, and the record it becomes.
#[derive(Default)]
pub struct Gather {
    /// The sixteen configuration words, by the counter that numbered them.
    cfg: [u32; 16],
    seen: u16,
}

impl Gather {
    pub fn new() -> Self {
        Self::default()
    }

    /// Take a frame that has already passed its check, and hand back the
    /// record it makes with what has been gathered.
    pub fn take(&mut self, frame: &[u8]) -> Option<Vec<u8>> {
        let layout = check(frame)?;
        let len = layout.frame_len();
        // The configuration word sits at the end of the frame, before the
        // check, and its number is the low nibble of the counter byte.
        let at = match layout {
            Layout::Ecef => 44,
            Layout::LatLon => 41,
        };
        let slot = usize::from(frame[3] & 0xF);
        self.cfg[slot] =
            u32::from_le_bytes([frame[at], frame[at + 1], frame[at + 2], frame[at + 3]]);
        self.seen |= 1 << slot;

        let mut out = Vec::with_capacity(len + EXTRA);
        out.extend_from_slice(&frame[..len]);
        // The receiver's serial, the sensor boom's, and the date.
        for slot in [0xC, 0xD, 0xF] {
            out.extend(self.cfg[slot].to_le_bytes());
        }
        Some(out)
    }
}

/// What a gathered record says.
#[derive(Clone, Debug, PartialEq)]
pub struct Report {
    pub layout: Layout,
    /// `MRZ-<receiver>-<boom>`, or empty until both configuration words have
    /// come round. A sonde is named after the two serial numbers inside it
    /// because it carries no other name.
    pub serial: String,
    pub lat_deg: f64,
    pub lon_deg: f64,
    /// Height above the ellipsoid.
    pub altitude_m: f64,
    pub speed_kt: f64,
    pub course_deg: f64,
    pub climb_ms: f64,
    pub satellites: u8,
    /// Hours, minutes and seconds UTC.
    pub utc: (u8, u8, u8),
    /// Year, month and day, from the configuration word that carries them.
    pub date: Option<(u16, u8, u8)>,
}

impl Report {
    pub fn has_position(&self) -> bool {
        self.lat_deg != 0.0 || self.lon_deg != 0.0
    }

    pub fn summary(&self) -> String {
        let who = match self.serial.is_empty() {
            true => "MRZ".to_string(),
            false => self.serial.clone(),
        };
        format!(
            "{who} {:.5}, {:.5} at {:.0} m, {:+.1} m/s",
            self.lat_deg, self.lon_deg, self.altitude_m, self.climb_ms
        )
    }
}

/// Read a gathered record.
///
/// `None` where the frame's check does not hold or the position it works out
/// to is not one a balloon could be at, which is what a frame of zeros comes
/// to and what keeps another protocol's bytes out.
pub fn parse(record: &[u8]) -> Option<Report> {
    let layout = match record.len() {
        RECORD_ECEF => Layout::Ecef,
        RECORD_LATLON => Layout::LatLon,
        _ => return None,
    };
    let frame = &record[..layout.frame_len()];
    if check(frame)? != layout {
        return None;
    }
    let le32 =
        |at: usize| i32::from_le_bytes([frame[at], frame[at + 1], frame[at + 2], frame[at + 3]]);
    let le16 = |at: usize| i16::from_le_bytes([frame[at], frame[at + 1]]);

    let mut r = Report {
        layout,
        serial: String::new(),
        lat_deg: 0.0,
        lon_deg: 0.0,
        altitude_m: 0.0,
        speed_kt: 0.0,
        course_deg: 0.0,
        climb_ms: 0.0,
        satellites: 0,
        utc: (frame[4], frame[5], frame[6]),
        date: None,
    };
    match layout {
        Layout::Ecef => {
            let (x, y, z) = (
                f64::from(le32(8)) / 100.0,
                f64::from(le32(12)) / 100.0,
                f64::from(le32(16)) / 100.0,
            );
            let (lat, lon, alt) = ecef_to_geodetic(x, y, z);
            let (vx, vy, vz) = (
                f64::from(le16(20)) / 100.0,
                f64::from(le16(22)) / 100.0,
                f64::from(le16(24)) / 100.0,
            );
            let (e, n, u) = ecef_velocity_to_enu(lat, lon, vx, vy, vz);
            r.lat_deg = lat;
            r.lon_deg = lon;
            r.altitude_m = alt;
            r.speed_kt = e.hypot(n) * 1.943_844;
            r.course_deg = e.atan2(n).to_degrees().rem_euclid(360.0);
            r.climb_ms = u;
            r.satellites = frame[26];
        }
        Layout::LatLon => {
            r.lat_deg = f64::from(le32(7)) / 1e6;
            r.lon_deg = f64::from(le32(11)) / 1e6;
            r.altitude_m = f64::from(le32(15)) / 100.0;
            r.speed_kt = f64::from(le16(19)) / 100.0 * 1.943_844;
            r.course_deg = f64::from(u16::from_le_bytes([frame[21], frame[22]])) / 100.0;
        }
    }
    // Nothing a balloon does reaches these, and a frame of zeros lands at
    // the centre of the earth.
    if r.altitude_m < -1_000.0 || r.altitude_m > 80_000.0 {
        return None;
    }

    let extra = |i: usize| {
        let at = layout.frame_len() + i * 4;
        u32::from_le_bytes([record[at], record[at + 1], record[at + 2], record[at + 3]])
    };
    let (receiver, boom, date) = (extra(0), extra(1), extra(2));
    if receiver > 0 && boom > 0 {
        r.serial = format!("MRZ-{receiver}-{boom}");
    }
    if date > 0 {
        r.date = Some((
            2000 + (date % 100) as u16,
            ((date / 100) % 100) as u8,
            ((date / 10_000) % 100) as u8,
        ));
    }
    Some(r)
}

/// What a Meteo-Radiy MRZ frame says.
pub fn read(bytes: &[u8]) -> Option<Proto> {
    let r = parse(bytes)?;
    let mut p = Proto::new("mrz", "frame");
    if !r.serial.is_empty() {
        p = p
            .by(Entity::new("mrz", Id::Text(r.serial.clone())).made_by("Meteo-Radiy"))
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

/// Chips in the longest frame.
pub const FRAME_CHIPS: usize = FRAME_ECEF * 8 * 2;

/// Chips held while looking for a frame: three of them.
pub const MAX_CHIPS: usize = FRAME_CHIPS * 3;

/// Chips of the three sync bytes allowed to be wrong. The CRC behind them
/// refuses what a false sync would produce.
pub const SYNC_SLACK: u32 = 4;

/// The three sync bytes as chips, each way up. Manchester: a one is a fall
/// and a zero a rise, or the other way about when the receiver has the
/// signal over.
pub fn sync_chips() -> [Vec<bool>; 2] {
    let upright: Vec<bool> = SYNC
        .iter()
        .flat_map(|b| (0..8).rev().map(move |k| b >> k & 1 != 0))
        .flat_map(|bit| [bit, !bit])
        .collect();
    let inverted = upright.iter().map(|c| !c).collect();
    [upright, inverted]
}

pub fn matches(chips: &[bool], at: usize, want: &[bool]) -> bool {
    let mut wrong = 0;
    for (k, &w) in want.iter().enumerate() {
        wrong += u32::from(chips[at + k] != w);
        if wrong > SYNC_SLACK {
            return false;
        }
    }
    true
}

/// Manchester chips back to bytes, most significant bit first.
pub fn manchester(chips: &[bool], inverted: bool) -> Vec<u8> {
    let bits: Vec<bool> = chips.chunks(2).map(|c| (c[0] && !c[c.len() - 1]) != inverted).collect();
    bits.chunks(8).map(|b| b.iter().fold(0u8, |v, bit| v << 1 | u8::from(*bit))).collect()
}

/// Frames cut out of a stream of chips.
///
/// Above the waveform and below the payload: the chips come from any FSK
/// demodulator at this sonde's baud, and what leaves is what the payload
/// reader takes.
pub struct Framer {
    chips: Vec<bool>,
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
        Self { chips: Vec::new(), scanned: 0, gather: Gather::new() }
    }

    /// The buffer a bit clock appends into.
    pub fn sink(&mut self) -> &mut Vec<bool> {
        &mut self.chips
    }

    pub fn take(&mut self) -> Vec<Vec<u8>> {
        let mut out = Vec::new();
        let sync = sync_chips();
        let mut at = self.scanned;
        while at + sync[0].len() <= self.chips.len() {
            let Some(inverted) = sync.iter().position(|s| matches(&self.chips, at, s)) else {
                at += 1;
                continue;
            };
            if at + FRAME_CHIPS > self.chips.len() {
                self.scanned = at;
                return out;
            }
            let frame = manchester(&self.chips[at..at + FRAME_CHIPS], inverted == 1);
            match self.gather.take(&frame) {
                Some(record) => {
                    let used = at + record.len().saturating_sub(12) * 16;
                    out.push(record);
                    self.chips.drain(..used.min(self.chips.len()));
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

    /// A frame with its sync, counter and check in place.
    fn frame(layout: Layout, counter: u8, fields: &[(usize, Vec<u8>)]) -> Vec<u8> {
        let mut f = vec![0u8; layout.frame_len()];
        f[..3].copy_from_slice(&SYNC);
        f[3] = 0x80 | counter;
        if layout == Layout::LatLon {
            f[30..32].copy_from_slice(&[0xFF, 0xFF]);
        }
        for (at, bytes) in fields {
            f[*at..*at + bytes.len()].copy_from_slice(bytes);
        }
        let len = layout.checked();
        let cs = crc16le(&f[3..3 + len], 0x8005, 0xFFFF).to_le_bytes();
        f[len + 3..len + 5].copy_from_slice(&cs);
        f
    }

    /// Earth-centred metres for a place, which is what this sonde's
    /// receiver reports and what the frame carries.
    fn ecef(lat: f64, lon: f64, alt: f64) -> (f64, f64, f64) {
        const A: f64 = 6_378_137.0;
        const E2: f64 = 6.694_379_990_141_32e-3;
        let (sla, cla) = lat.to_radians().sin_cos();
        let (slo, clo) = lon.to_radians().sin_cos();
        let n = A / (1.0 - E2 * sla * sla).sqrt();
        ((n + alt) * cla * clo, (n + alt) * cla * slo, (n * (1.0 - E2) + alt) * sla)
    }

    /// A flight over the Irish Sea, gathered with the configuration words
    /// that name the sonde.
    #[test]
    fn an_ecef_frame_reads_as_a_fix() {
        let (x, y, z) = ecef(53.35, -5.0, 4_712.22);
        let cm = |v: f64| ((v * 100.0).round() as i32).to_le_bytes().to_vec();
        let mut g = Gather::new();
        // The configuration words: the receiver's serial on counter 0xC,
        // the boom's on 0xD, the date on 0xF.
        for (counter, word) in [(0xCu8, 5_601_u32), (0xD, 4_210), (0xF, 130_325)] {
            let f = frame(Layout::Ecef, counter, &[(44, word.to_le_bytes().to_vec())]);
            assert!(g.take(&f).is_some(), "a frame with a config word");
        }
        let f = frame(
            Layout::Ecef,
            1,
            &[
                (4, vec![5, 42, 20]),
                (8, cm(x)),
                (12, cm(y)),
                (16, cm(z)),
                // Nine metres a second east and five up, as the receiver
                // sends them: earth-centred, in centimetres a second.
                (
                    20,
                    ((-9.0f64 * (-5.0f64).to_radians().sin() * 100.0) as i16)
                        .to_le_bytes()
                        .to_vec(),
                ),
                (26, vec![11]),
            ],
        );
        let record = g.take(&f).expect("a record");
        assert_eq!(record.len(), RECORD_ECEF);

        let r = parse(&record).expect("a report");
        assert_eq!(r.layout, Layout::Ecef);
        assert!((r.lat_deg - 53.35).abs() < 1e-6, "{}", r.lat_deg);
        assert!((r.lon_deg + 5.0).abs() < 1e-6, "{}", r.lon_deg);
        assert!((r.altitude_m - 4_712.22).abs() < 0.05, "{}", r.altitude_m);
        assert_eq!(r.satellites, 11);
        assert_eq!(r.utc, (5, 42, 20));
        assert_eq!(r.serial, "MRZ-5601-4210");
        assert_eq!(r.date, Some((2025, 3, 13)));
    }

    /// The older layout puts degrees in the frame instead, and is three
    /// bytes shorter. Which it is comes out of the frame itself.
    #[test]
    fn a_latlon_frame_reads_as_a_fix() {
        let f = frame(
            Layout::LatLon,
            2,
            &[
                (4, vec![5, 42, 20]),
                (7, 53_350_000i32.to_le_bytes().to_vec()),
                (11, (-5_000_000i32).to_le_bytes().to_vec()),
                (15, 471_222i32.to_le_bytes().to_vec()),
                (19, 900i16.to_le_bytes().to_vec()),
                (21, 20_000u16.to_le_bytes().to_vec()),
            ],
        );
        let mut g = Gather::new();
        let record = g.take(&f).expect("a record");
        assert_eq!(record.len(), RECORD_LATLON);
        let r = parse(&record).expect("a report");
        assert_eq!(r.layout, Layout::LatLon);
        assert!((r.lat_deg - 53.35).abs() < 1e-6, "{}", r.lat_deg);
        assert!((r.lon_deg + 5.0).abs() < 1e-6, "{}", r.lon_deg);
        assert!((r.altitude_m - 4_712.22).abs() < 0.01, "{}", r.altitude_m);
        assert!((r.speed_kt - 17.49).abs() < 0.05, "{}", r.speed_kt);
        assert!((r.course_deg - 200.0).abs() < 0.01, "{}", r.course_deg);
        // No configuration word has come round, so it has no name yet.
        assert_eq!(r.serial, "");
    }

    /// A wrong bit fails the check, and a frame that checks but places the
    /// sonde at the centre of the earth is not a position.
    #[test]
    fn a_broken_frame_is_refused() {
        let (x, y, z) = ecef(53.35, -5.0, 4_712.22);
        let cm = |v: f64| ((v * 100.0).round() as i32).to_le_bytes().to_vec();
        let f = frame(Layout::Ecef, 1, &[(8, cm(x)), (12, cm(y)), (16, cm(z))]);
        let mut g = Gather::new();
        assert!(g.take(&f).is_some());
        let mut bad = f.clone();
        bad[10] ^= 0x01;
        assert_eq!(check(&bad), None);
        // All zeros checks against nothing and is at the centre of the
        // earth even if it did.
        let empty = frame(Layout::Ecef, 0, &[]);
        let record = Gather::new().take(&empty).expect("a record");
        assert_eq!(parse(&record), None);
    }
}
