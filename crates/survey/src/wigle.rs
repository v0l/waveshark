//! WiGLE CSV: the rows, and the upload.
//!
//! Not because wigle.net is the destination, but because every wardriving
//! tool reads this file and nothing reads a format invented here. Kismet,
//! WiGLE's own apps and a dozen scripts all take these fourteen columns, so a
//! survey in this shape can be merged with somebody else's, plotted by
//! something that already draws coverage, or sent to WiGLE itself.
//!
//! The format is `WigleWifi-1.6`: a header line of key=value pairs, a column
//! header, then a row per observation. It was written for Wi-Fi access points
//! and grew a `Type` column when Bluetooth and cells arrived, and the columns
//! mean different things per type: a Bluetooth row's `Frequency` is a device
//! class, a cell row's is an ARFCN, and a cell's `MAC` is not an address at
//! all but `MCCMNC_LAC_CID`.
//!
//! # Only what the format has a type for
//!
//! A row is written for a device WiGLE can hold: a BLE or Bluetooth address,
//! and a GSM cell that named itself. Everything else the survey records,
//! aircraft, vessels, pagers, tyre sensors, has no type here, and an earlier
//! version of this exported those as `BLE` on the grounds that a row in the
//! wrong bucket is better than no row. It is not: an ICAO address filed as a
//! Bluetooth device is wrong in somebody else's database forever, and nothing
//! downstream can tell it from a real one. Those stay in the survey file,
//! which is the record.
//!
//! # What does not survive the round trip
//!
//! Everything the format has no column for: the SNR, how many packets, the
//! first and last times separately. So this is an export and not a backup:
//! the survey file is the record, and this is a copy of it shaped for other
//! people's tools.

use crate::{Db, Device, Query, Sighting};
use common::Result;
use std::io::Write;

mod upload;
pub use upload::{upload, Account, Receipt};

/// What WiGLE calls a device of this kind, or `None` for something it has no
/// bucket for.
///
/// Keyed on the survey's identity space, which is what a decoder said the
/// identifier means. Adding a kind is a line here plus the columns it fills
/// in [`row`]; a protocol missing from this table is not exported, which is
/// the intended answer for everything that is not a radio LAN or a cell.
pub fn kind(protocol: &str) -> Option<&'static str> {
    match protocol {
        "ble" => Some("BLE"),
        "bt" => Some("BT"),
        "gsm" => Some("GSM"),
        _ => None,
    }
}

/// A cell's key, network name and capability string, from the identity the
/// GSM decoder gives a cell: `MCC-MNC-LAC-CID`.
///
/// WiGLE wants the operator as `MCC` and `MNC` run together, then the
/// location area and the cell, underscore separated. The MNC keeps the number
/// of digits it was broadcast with: two and three digit codes are different
/// networks in the same country, and padding one to look like the other files
/// a cell under an operator that does not exist.
fn cell_key(ident: &str) -> Option<(String, String)> {
    let mut parts = ident.split('-');
    let (mcc, mnc, lac, cid) = (parts.next()?, parts.next()?, parts.next()?, parts.next()?);
    if parts.next().is_some() || [mcc, mnc, lac, cid].iter().any(|p| p.is_empty()) {
        return None;
    }
    let operator = format!("{mcc}{mnc}");
    Some((format!("{operator}_{lac}_{cid}"), operator))
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

/// The two lines every file starts with: what wrote it, and the columns.
pub fn header() -> String {
    format!(
        "WigleWifi-1.6,appRelease={v},model=waveshark,release={v},device=waveshark,\
display=,board=,brand=waveshark,star=Sol,body=3,subBody=0\n\
MAC,SSID,AuthMode,FirstSeen,Channel,Frequency,RSSI,CurrentLatitude,\
CurrentLongitude,AltitudeMeters,AccuracyMeters,RCOIs,MfgrId,Type\n",
        v = env!("CARGO_PKG_VERSION")
    )
}

/// One observation as a row, or `None` when it is not one WiGLE can hold.
///
/// A sighting with no position is not a row: the format has no way to say
/// "heard, position unknown", and a row at the island of null is a lie the
/// reader cannot detect.
pub fn row(
    protocol: &str,
    ident: &str,
    name: Option<&str>,
    vendor: Option<&str>,
    s: &Sighting,
) -> Option<String> {
    let (lat, lon) = (s.lat?, s.lon?);
    let ty = kind(protocol)?;
    // Channel and Frequency carry different things per type, and a reader
    // takes them at their word: a Bluetooth row's frequency is a device class
    // code, which this has no way to know, so it is left empty rather than
    // filled with hertz.
    let (mac, ssid, caps, channel, frequency) = match ty {
        "GSM" => {
            let (key, operator) = cell_key(ident)?;
            let arfcn = dsp::gsm::arfcn(s.center_hz as f64)
                .map(|n| n.to_string())
                .unwrap_or_default();
            (key, name.unwrap_or("").to_string(), format!("GSM;{operator}"), String::new(), arfcn)
        }
        _ => (
            ident.to_string(),
            name.unwrap_or("").to_string(),
            // The capability column is Wi-Fi's encryption and Bluetooth's
            // class list; what is known here is which scan saw it.
            format!("[{ty}]"),
            "0".to_string(),
            String::new(),
        ),
    };
    Some(format!(
        "{},{},{},{},{channel},{frequency},{},{lat},{lon},{},{},,{},{ty}",
        field(&mac),
        field(&ssid),
        field(&caps),
        stamp(s.at_us),
        s.rssi_dbfs.map(|v| v.round() as i64).unwrap_or(0),
        s.alt_m.unwrap_or(0.0).round() as i64,
        s.accuracy_m.unwrap_or(0.0),
        field(vendor.unwrap_or("")),
    ))
}

/// Write the whole survey, one row per sighting that has a position and a
/// type. Returns how many rows were written.
pub fn write_wigle(db: &Db, q: Query, out: &mut impl Write) -> Result<usize> {
    let io = |e: std::io::Error| common::Error::other(format!("wigle export: {e}"));
    out.write_all(header().as_bytes()).map_err(io)?;
    let mut rows = 0usize;
    for d in db.devices(q)? {
        if kind(&d.protocol).is_none() {
            continue;
        }
        for s in db.sightings(d.id)? {
            let Some(line) = row_of(&d, &s) else { continue };
            writeln!(out, "{line}").map_err(io)?;
            rows += 1;
        }
    }
    Ok(rows)
}

/// A device row from the survey's own record of it.
pub fn row_of(d: &Device, s: &Sighting) -> Option<String> {
    row(&d.protocol, &d.ident, d.name.as_deref(), d.vendor.as_deref(), s)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Report;

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

    fn export(db: &Db) -> String {
        let mut buf = Vec::new();
        write_wigle(db, Query::default(), &mut buf).unwrap();
        String::from_utf8(buf).unwrap()
    }

    #[test]
    fn the_export_has_the_header_the_readers_expect() {
        let text = export(&db_with_one());
        let mut lines = text.lines();
        assert!(lines.next().unwrap().starts_with("WigleWifi-1.6,"));
        assert!(lines.next().unwrap().starts_with("MAC,SSID,AuthMode,FirstSeen,Channel,"));
        assert_eq!(lines.count(), 1);
    }

    #[test]
    fn a_bluetooth_row_carries_the_position_and_the_level() {
        let text = export(&db_with_one());
        let row = text.lines().nth(2).unwrap();
        assert!(row.starts_with("6C:70:CB:EF:72:4D,"), "{row}");
        assert!(row.contains("53.6369,-6.6528"), "{row}");
        assert!(row.contains(",-46,"), "the level is missing from {row}");
        assert!(row.ends_with(",Samsung,BLE"), "{row}");
        assert!(row.contains("2026-09-07 09:42:50"), "{row}");
        // Channel 0 and an empty frequency: the column means a Bluetooth
        // device class here, and hertz in it is read as one.
        assert!(row.contains(",0,,"), "{row}");
    }

    /// A cell is not filed by address but by the operator, the location area
    /// and the cell number, and the carrier it was heard on goes in as an
    /// ARFCN. Getting the key wrong files the cell under a network that does
    /// not exist.
    #[test]
    fn a_cell_is_a_row_keyed_by_operator_and_cell() {
        let mut db = Db::in_memory().unwrap();
        db.record(&Report {
            protocol: "gsm".into(),
            ident: "272-01-1234-56789".into(),
            name: Some("Vodafone IE".into()),
            vendor: None,
            sighting: Sighting {
                at_us: 1_788_774_170_000_000,
                lat: Some(53.3),
                lon: Some(-6.2),
                rssi_dbfs: Some(-81.0),
                center_hz: 947_400_000,
                ..Default::default()
            },
        })
        .unwrap();
        let row = export(&db).lines().nth(2).unwrap().to_string();
        assert!(row.starts_with("27201_1234_56789,Vodafone IE,GSM;27201,"), "{row}");
        assert!(row.ends_with(",GSM"), "{row}");
        // ARFCN 62 is 947.4 MHz downlink, and the channel column stays empty.
        assert!(row.contains(",,62,"), "{row}");
    }

    /// A row in the wrong bucket is wrong in somebody else's database
    /// forever, so a protocol WiGLE has no type for is not exported.
    #[test]
    fn an_aircraft_is_not_a_bluetooth_device() {
        let mut db = Db::in_memory().unwrap();
        db.record(&Report {
            protocol: "adsb".into(),
            ident: "4ca1fb".into(),
            name: None,
            vendor: None,
            sighting: Sighting {
                at_us: 1,
                lat: Some(53.3),
                lon: Some(-6.2),
                center_hz: 1_090_000_000,
                ..Default::default()
            },
        })
        .unwrap();
        assert_eq!(export(&db).lines().count(), 2, "header only");
        assert_eq!(kind("adsb"), None);
    }

    /// A name off the air can hold a comma or a quote, and one badly named
    /// device must not corrupt every row after it.
    #[test]
    fn a_name_with_a_comma_is_quoted() {
        let text = export(&db_with_one());
        let row = text.lines().nth(2).unwrap();
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

    #[test]
    fn a_cell_identity_that_is_not_four_parts_is_not_a_key() {
        assert_eq!(cell_key("272-01-1234"), None);
        assert_eq!(cell_key("272-01-1234-5-6"), None);
        assert_eq!(cell_key("272--1234-5"), None);
    }
}
