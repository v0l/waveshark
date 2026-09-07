//! Export a survey as WiGLE CSV.
//!
//! Not because wigle.net is the destination, but because every wardriving
//! tool reads this file and nothing reads a format invented here. Kismet,
//! WiGLE's own apps and a dozen scripts all take these eleven columns, so a
//! survey in this shape can be merged with somebody else's, plotted by
//! something that already draws coverage, or uploaded.
//!
//! The format is `WigleWifi-1.6`: a header line of key=value pairs, a column
//! header, then a row per observation. It was written for Wi-Fi access points
//! and grew a `Type` column when Bluetooth arrived, which is the column that
//! makes it usable for everything here: `WIFI`, `BLE`, `BT`, `GSM`, `CDMA`,
//! `LTE`, `NR`. A protocol with no type of its own is exported as `BLE`
//! rather than invented, since the readers reject an unknown type and drop
//! the row, and a row in the wrong bucket is at least a row.
//!
//! # What does not survive the round trip
//!
//! Everything the format has no column for: the frequency in hertz, the SNR,
//! how many packets, the first and last times separately. WiGLE has one time
//! per row and a channel number rather than a frequency. So this is an export
//! and not a backup: the survey file is the record, and this is a copy of it
//! shaped for other people's tools.

use crate::{Db, Device, Query};
use common::Result;
use std::io::Write;

/// The `Type` column, from the protocol a device was heard on.
fn kind(protocol: &str) -> &'static str {
    match protocol {
        "ble" => "BLE",
        "wifi" => "WIFI",
        _ => "BLE",
    }
}

/// The `Channel` column. WiGLE wants a channel number and every device here
/// has a frequency, so what goes in is the BLE channel where that is what it
/// is, and the frequency in megahertz otherwise: a number that identifies
/// where it was heard, in the only column there is for one.
fn channel(protocol: &str, center_hz: u64) -> i64 {
    match (protocol, center_hz) {
        ("ble", 2_402_000_000) => 37,
        ("ble", 2_426_000_000) => 38,
        ("ble", 2_480_000_000) => 39,
        _ => (center_hz / 1_000_000) as i64,
    }
}

/// `YYYY-MM-DD HH:MM:SS`, which is what the format's readers expect.
fn stamp(us: u64) -> String {
    let secs = (us / 1_000_000) as i64;
    let days = secs.div_euclid(86_400);
    let rem = secs.rem_euclid(86_400);
    let (mut y, mut d) = (1970i64, days);
    let leap = |y: i64| y % 4 == 0 && (y % 100 != 0 || y % 400 == 0);
    loop {
        let len = if leap(y) { 366 } else { 365 };
        if d < len {
            break;
        }
        d -= len;
        y += 1;
    }
    const LENGTHS: [i64; 12] = [31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31];
    let mut m = 0usize;
    while m < 12 {
        let len = LENGTHS[m] + i64::from(m == 1 && leap(y));
        if d < len {
            break;
        }
        d -= len;
        m += 1;
    }
    format!(
        "{y:04}-{:02}-{:02} {:02}:{:02}:{:02}",
        m + 1,
        d + 1,
        rem / 3600,
        rem % 3600 / 60,
        rem % 60
    )
}

/// A CSV field: quoted when it holds anything that would break the row.
///
/// Device names come off the air and are whatever the vendor typed, which has
/// included commas, quotes and inch marks. A survey that corrupts its own
/// export on one badly named printer is worse than one with no export.
fn field(s: &str) -> String {
    let clean: String = s.chars().filter(|c| !c.is_control()).collect();
    if clean.contains([',', '"', '\n']) {
        format!("\"{}\"", clean.replace('"', "\"\""))
    } else {
        clean
    }
}

/// Write the whole survey, one row per sighting that has a position.
///
/// Sightings without a fix are dropped rather than written at zero: the
/// format has no way to say "heard, position unknown", and a row at the
/// island of null is a lie the reader cannot detect.
pub fn write_wigle(db: &Db, q: Query, out: &mut impl Write) -> Result<usize> {
    let io = |e: std::io::Error| common::Error::other(format!("wigle export: {e}"));
    writeln!(
        out,
        "WigleWifi-1.6,appRelease={},model=waveshark,release={},device=waveshark,\
display=,board=,brand=waveshark,star=Sol,body=3,subBody=0",
        env!("CARGO_PKG_VERSION"),
        env!("CARGO_PKG_VERSION")
    )
    .map_err(io)?;
    writeln!(
        out,
        "MAC,SSID,AuthMode,FirstSeen,Channel,Frequency,RSSI,CurrentLatitude,\
CurrentLongitude,AltitudeMeters,AccuracyMeters,RCOIs,MfgrId,Type"
    )
    .map_err(io)?;

    let mut rows = 0usize;
    for d in db.devices(q)? {
        for s in db.sightings(d.id)? {
            let (Some(lat), Some(lon)) = (s.lat, s.lon) else { continue };
            writeln!(
                out,
                "{},{},{},{},{},{},{},{lat},{lon},{},{},,{},{}",
                field(&d.ident),
                field(d.name.as_deref().unwrap_or("")),
                // The column is Wi-Fi's encryption, and nothing here has one.
                // Vendor goes in as a tag rather than left blank, since it is
                // the one thing readers show beside a name.
                field(&vendor_tag(&d)),
                stamp(s.at_us),
                channel(&d.protocol, s.center_hz),
                s.center_hz / 1_000,
                s.rssi_dbfs.map(|v| v.round() as i64).unwrap_or(0),
                s.alt_m.unwrap_or(0.0),
                s.accuracy_m.unwrap_or(0.0),
                field(d.vendor.as_deref().unwrap_or("")),
                kind(&d.protocol),
            )
            .map_err(io)?;
            rows += 1;
        }
    }
    Ok(rows)
}

/// The `AuthMode` column carries the protocol, in brackets the way WiGLE's
/// own Bluetooth rows carry capability flags. Without it every row in a
/// mixed survey looks like the same kind of thing.
fn vendor_tag(d: &Device) -> String {
    format!("[{}]", d.protocol.to_uppercase())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{Report, Sighting};

    fn db_with_one() -> Db {
        let mut db = Db::in_memory().unwrap();
        db.record(&Report {
            protocol: "ble".into(),
            ident: "6C:70:CB:EF:72:4D".into(),
            name: Some("49\" Odyssey, OLED".into()),
            vendor: Some("Samsung".into()),
            sighting: Sighting {
                at_us: 1_788_774_170_000_000,
                lat: Some(53.6369),
                lon: Some(-6.6528),
                alt_m: Some(42.5),
                accuracy_m: Some(5.0),
                rssi_dbfs: Some(-46.4),
                snr_db: Some(20.0),
                center_hz: 2_426_000_000,
            },
        })
        .unwrap();
        db
    }

    #[test]
    fn the_export_has_the_header_the_readers_expect() {
        let mut buf = Vec::new();
        let n = write_wigle(&db_with_one(), Query::default(), &mut buf).unwrap();
        let text = String::from_utf8(buf).unwrap();
        let mut lines = text.lines();
        assert!(lines.next().unwrap().starts_with("WigleWifi-1.6,"));
        assert!(lines.next().unwrap().starts_with("MAC,SSID,AuthMode,FirstSeen,Channel,"));
        assert_eq!(n, 1);
    }

    #[test]
    fn a_row_carries_the_position_the_level_and_the_channel() {
        let mut buf = Vec::new();
        write_wigle(&db_with_one(), Query::default(), &mut buf).unwrap();
        let row = String::from_utf8(buf).unwrap().lines().nth(2).unwrap().to_string();
        assert!(row.starts_with("6C:70:CB:EF:72:4D,"), "{row}");
        assert!(row.contains("53.6369,-6.6528"), "{row}");
        assert!(row.contains(",38,"), "channel 38 is not in {row}");
        assert!(row.ends_with(",Samsung,BLE"), "{row}");
        assert!(row.contains("2026-09-07 09:42:50"), "{row}");
    }

    /// A name off the air can hold a comma or a quote, and one badly named
    /// device must not corrupt every row after it.
    #[test]
    fn a_name_with_a_comma_is_quoted() {
        let mut buf = Vec::new();
        write_wigle(&db_with_one(), Query::default(), &mut buf).unwrap();
        let row = String::from_utf8(buf).unwrap().lines().nth(2).unwrap().to_string();
        assert!(row.contains("\"49\"\" Odyssey, OLED\""), "{row}");
        assert_eq!(row.matches(',').count() - 1, 13, "the row grew or lost a column: {row}");
    }

    /// The format cannot say "heard here, position unknown", so those
    /// sightings are left out rather than written at zero.
    #[test]
    fn a_sighting_without_a_fix_is_not_exported() {
        let mut db = Db::in_memory().unwrap();
        db.record(&Report {
            protocol: "ble".into(),
            ident: "AA:BB".into(),
            name: None,
            vendor: None,
            sighting: Sighting { at_us: 1, center_hz: 2_426_000_000, ..Default::default() },
        })
        .unwrap();
        let mut buf = Vec::new();
        assert_eq!(write_wigle(&db, Query::default(), &mut buf).unwrap(), 0);
    }

    #[test]
    fn a_timestamp_is_the_civil_time_the_format_wants() {
        // 2026-09-07 09:42:50 UTC.
        assert_eq!(stamp(1_788_774_170_000_000), "2026-09-07 09:42:50");
        assert_eq!(stamp(0), "1970-01-01 00:00:00");
        // A leap day, which is where a hand-written calendar goes wrong.
        assert_eq!(stamp(1_709_164_800_000_000), "2024-02-29 00:00:00");
    }
}
