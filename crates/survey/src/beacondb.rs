//! beaconDB: observations submitted, and a position asked for.
//!
//! Ichnaea's API as <https://beacondb.net> serves it, and no credential at
//! all: `/v2/geosubmit` takes what was heard and where it was heard from,
//! `/v1/geolocate` answers where a receiver seeing those beacons probably is.
//! What identifies a client is the user agent, which is why every request
//! here goes through `httpc` like every other request this program makes.
//!
//! # Not a dataset
//!
//! OpenCelliD is a file: a country's cells downloaded once and read from
//! disc. beaconDB has no bulk export yet, so a position is asked for one
//! beacon at a time and only when something wants it. That difference is why
//! this is not in `datasets`, which is a cache of other people's files.
//!
//! # Only what was measured, and only what is safe to give away
//!
//! An observation is submitted with the receiver's own position, so nothing
//! goes without a fix. Two things are deliberately left out. The level: this
//! receiver measures dBFS against a tuner's full scale, and Ichnaea's
//! `signalStrength` is dBm, so filling it in would put a number into somebody
//! else's database that means something else, and a wrong absolute level is
//! worse than a missing optional field. And anything that is not a cell or a
//! Bluetooth device: an aircraft, a pager or a tyre sensor has no beacon type
//! here, and a survey row filed under one that happens to fit is wrong in a
//! stranger's database forever.

use crate::Sighting;
use std::time::Duration;

/// Where observations go.
pub const SUBMIT_URL: &str = "https://api.beacondb.net/v2/geosubmit";

/// Where a position is asked for.
pub const LOCATE_URL: &str = "https://api.beacondb.net/v1/geolocate";

/// A submission is a few kilobytes and happens from a car on a tether.
const TIMEOUT: Duration = Duration::from_secs(60);

/// A lookup is drawn on somebody's screen, so it gives up quickly.
const LOOKUP_TIMEOUT: Duration = Duration::from_secs(15);

/// What beaconDB has a beacon type for, keyed on the survey's identity space.
///
/// Adding a kind is a line here and the fields it fills in [`item`]. A
/// protocol missing from this table is not submitted, which is the intended
/// answer for everything that is not a cell or a Bluetooth device.
pub fn kind(protocol: &str) -> Option<&'static str> {
    match protocol {
        "gsm" => Some("cellTowers"),
        "ble" | "bt" => Some("bluetoothBeacons"),
        _ => None,
    }
}

/// A cell as beaconDB names it, from the identity the GSM decoder gives:
/// `MCC-MNC-LAC-CID`.
///
/// The MNC keeps the number of digits it was broadcast with. Two and three
/// digit codes are different networks in the same country, and padding one to
/// look like the other files a cell under an operator that does not exist.
pub fn cell(ident: &str) -> Option<Cell> {
    let mut parts = ident.split('-');
    let (mcc, mnc, lac, cid) = (parts.next()?, parts.next()?, parts.next()?, parts.next()?);
    if parts.next().is_some() {
        return None;
    }
    Some(Cell {
        mcc: mcc.parse().ok()?,
        mnc: mnc.parse().ok()?,
        lac: lac.parse().ok()?,
        cid: cid.parse().ok()?,
    })
}

/// One cell, in the numbers the API asks for.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct Cell {
    pub mcc: u16,
    pub mnc: u16,
    pub lac: u32,
    pub cid: u64,
}

impl Cell {
    fn json(&self, radio: &str) -> serde_json::Value {
        serde_json::json!({
            "radioType": radio,
            "mobileCountryCode": self.mcc,
            "mobileNetworkCode": self.mnc,
            "locationAreaCode": self.lac,
            "cellId": self.cid,
        })
    }
}

/// One observation, or `None` when it is not one beaconDB can hold.
///
/// A sighting with no position is not an observation: the whole point of a
/// submission is that this receiver was at that place and heard that beacon,
/// and there is nothing to say without the place.
pub fn item(protocol: &str, ident: &str, s: &Sighting) -> Option<serde_json::Value> {
    let (lat, lon) = (s.lat?, s.lon?);
    let field = kind(protocol)?;
    let beacon = match field {
        "cellTowers" => cell(ident)?.json("gsm"),
        _ => serde_json::json!({ "macAddress": ident.to_ascii_lowercase() }),
    };
    let mut position = serde_json::json!({
        "latitude": lat,
        "longitude": lon,
        "source": "gps",
    });
    // Optional and left out rather than guessed: a fix with no stated
    // accuracy is not a fix accurate to zero metres.
    if let (Some(m), Some(o)) = (s.accuracy_m, position.as_object_mut()) {
        o.insert("accuracy".into(), serde_json::json!(m));
    }
    if let (Some(m), Some(o)) = (s.alt_m, position.as_object_mut()) {
        o.insert("altitude".into(), serde_json::json!(m));
    }
    Some(serde_json::json!({
        "timestamp": s.at_us / 1000,
        "position": position,
        field: [beacon],
    }))
}

/// One observation as the text that goes in a spool file.
///
/// Text rather than a value because what collects these has no reason to
/// hold a JSON library: it writes them out and posts what it wrote.
pub fn item_json(protocol: &str, ident: &str, s: &Sighting) -> Option<String> {
    item(protocol, ident, s).map(|v| v.to_string())
}

/// A submission body around observations already in JSON.
pub fn body(items: &[String]) -> String {
    format!("{{\"items\":[{}]}}", items.join(","))
}

/// Send observations. Blocking, so it belongs on a thread of its own.
///
/// An error is a submission that has not been taken and should be tried
/// again: the caller keeps its spool file until this returns `Ok`.
pub fn submit(body: &[u8]) -> Result<(), String> {
    let http = httpc::blocking(TIMEOUT).map_err(|e| e.to_string())?;
    let resp = http
        .post(SUBMIT_URL)
        .header("Content-Type", "application/json")
        .body(body.to_vec())
        .send()
        .map_err(|e| e.to_string())?;
    let status = resp.status();
    if status.is_success() {
        return Ok(());
    }
    let body = resp.text().unwrap_or_default();
    Err(format!("beaconDB answered {}: {}", status.as_u16(), first_line(&body)))
}

/// Where a receiver seeing this cell probably is, and how far out that may
/// be, in metres.
///
/// Ichnaea answers the question "where am I", not "where is that mast". With
/// one cell and nothing else the answer is the cell's own estimated position,
/// which is the nearest thing to a tower position beaconDB can give and is
/// worth no more precision than the accuracy it comes with. `Ok(None)` is a
/// cell the database has never heard of, which is the ordinary answer and not
/// a failure.
pub fn locate_cell(c: Cell, radio: &str) -> Result<Option<(f64, f64, f64)>, String> {
    let body = serde_json::json!({
        "considerIp": false,
        "cellTowers": [c.json(radio)],
    });
    let http = httpc::blocking(LOOKUP_TIMEOUT).map_err(|e| e.to_string())?;
    let resp = http
        .post(LOCATE_URL)
        .header("Content-Type", "application/json")
        .body(body.to_string())
        .send()
        .map_err(|e| e.to_string())?;
    let status = resp.status();
    let text = resp.text().unwrap_or_default();
    if status == reqwest::StatusCode::NOT_FOUND {
        return Ok(None);
    }
    if !status.is_success() {
        return Err(format!("beaconDB answered {}: {}", status.as_u16(), first_line(&text)));
    }
    let v: serde_json::Value =
        serde_json::from_str(&text).map_err(|_| "beaconDB sent something that is not JSON".to_string())?;
    let loc = v.get("location").ok_or_else(|| "no location in the answer".to_string())?;
    let (Some(lat), Some(lon)) = (
        loc.get("lat").and_then(serde_json::Value::as_f64),
        loc.get("lng").and_then(serde_json::Value::as_f64),
    ) else {
        return Ok(None);
    };
    let acc = v.get("accuracy").and_then(serde_json::Value::as_f64).unwrap_or(0.0);
    Ok(Some((lat, lon, acc)))
}

fn first_line(s: &str) -> String {
    s.lines().next().unwrap_or("").chars().take(160).collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sighting() -> Sighting {
        Sighting {
            at_us: 1_700_000_000_000_000,
            lat: Some(53.6369),
            lon: Some(-6.6528),
            accuracy_m: Some(8.0),
            rssi_dbfs: Some(-46.0),
            ..Default::default()
        }
    }

    #[test]
    fn a_cell_reads_as_the_four_numbers_the_api_wants() {
        assert_eq!(cell("272-1-1234-56789"), Some(Cell { mcc: 272, mnc: 1, lac: 1234, cid: 56789 }));
        // A three digit MNC is a different network from the same digits
        // padded, so nothing here rewrites one.
        assert_eq!(cell("310-260-1-2").map(|c| c.mnc), Some(260));
        assert_eq!(cell("272-1-1234"), None);
        assert_eq!(cell("272-1-1234-5-6"), None);
    }

    #[test]
    fn an_observation_carries_the_place_it_was_heard_from() {
        let v = item("gsm", "272-1-1234-56789", &sighting()).expect("an item");
        assert_eq!(v["position"]["latitude"], 53.6369);
        assert_eq!(v["position"]["accuracy"], 8.0);
        assert_eq!(v["timestamp"], 1_700_000_000_000u64);
        assert_eq!(v["cellTowers"][0]["mobileCountryCode"], 272);
        assert_eq!(v["cellTowers"][0]["cellId"], 56789);
        // dBFS is not dBm, so the level is left out rather than sent as one.
        assert!(v["cellTowers"][0].get("signalStrength").is_none());
    }

    #[test]
    fn a_bluetooth_address_is_submitted_lower_case() {
        let v = item("ble", "E8:31:CD:0A:F5:3A", &sighting()).expect("an item");
        assert_eq!(v["bluetoothBeacons"][0]["macAddress"], "e8:31:cd:0a:f5:3a");
    }

    /// Everything a survey records that beaconDB has no type for stays in the
    /// survey file rather than being filed under one that happens to fit.
    #[test]
    fn an_aircraft_is_not_an_observation() {
        assert!(item("adsb", "4CA1FB", &sighting()).is_none());
        assert!(item("tpms", "1234", &sighting()).is_none());
    }

    #[test]
    fn a_body_holds_the_observations_it_was_given() {
        let one = item_json("ble", "E8:31:CD:0A:F5:3A", &sighting()).unwrap();
        let v: serde_json::Value = serde_json::from_str(&body(&[one.clone(), one])).unwrap();
        assert_eq!(v["items"].as_array().map(Vec::len), Some(2));
    }

    /// Nothing is submitted that cannot say where it was heard from.
    #[test]
    fn without_a_fix_there_is_nothing_to_submit() {
        let s = Sighting { lat: None, lon: None, ..sighting() };
        assert!(item("ble", "E8:31:CD:0A:F5:3A", &s).is_none());
    }
}
