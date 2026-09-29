//! Devices heard, and where they were heard from.
//!
//! The packet log holds every burst as evidence and is written to be read
//! again by a better decoder. This is the other thing a survey wants: not the
//! transmissions but the transmitters, one row each, with the strongest place
//! each was heard from and the times it was first and last seen. Drive around
//! for an hour with the packet log on and you have four gigabytes of bursts;
//! what you wanted to know is which four hundred things were out there and
//! roughly where.
//!
//! # Identity is a pair, never a number
//!
//! `tracks` learned this when AIS arrived beside ADS-B: an ICAO address and
//! an MMSI are both integers and are not comparable. So a device here is
//! `(protocol, ident)`, the protocol naming the space the identifier lives
//! in, and the identifier kept as the text a person would recognise, an
//! address as `6C:70:CB:EF:72:4D` rather than as forty-eight bits. Rows are
//! read by people, exported to tools that expect text, and compared against
//! what another receiver saw.
//!
//! # What a sighting is, and what it is not
//!
//! A sighting is one reception: when, from where, at what level, on what
//! frequency. It says the receiver was at that position and heard that
//! device, which is all a moving receiver can honestly claim. It is not the
//! device's position. What is stored is the measurement; where the device
//! is, inferred from several of them, is [`locate`]'s conclusion drawn
//! later, and it says how sure it is.
//!
//! Sightings are thinned rather than kept in full. A beacon advertising ten
//! times a second for an hour is thirty-six thousand rows that all say the
//! same thing, so a new row is written only from a place the device has not
//! been heard from before, `MOVED_M` from every row it already has. A
//! receiver that is not moving writes nothing new. What is never thinned
//! away is the first and last time a device was heard, because those are on
//! the device row.
//!
//! # SQLite, and why a file rather than a format
//!
//! The packet log is hand-written binary because it is a stream of bursts
//! nobody queries until later. This is queried constantly by whatever is
//! drawing it, wants indexes on time and on identity, has to survive the
//! laptop being closed mid-drive, and is small enough that the storage cost
//! of a real database does not matter. `rusqlite` is already in the tree for
//! the Artemis signal database, bundled, so this costs no new dependency.

pub mod beacondb;
#[cfg(feature = "db")]
mod db;
#[cfg(not(feature = "db"))]
#[path = "db_offline.rs"]
mod db;
mod locate;
mod located;
mod places;
pub mod wigle;
pub use db::Db;
pub use locate::{Estimate, locate};
pub use located::{Located, Locator};
pub use places::{Places, distinct};
pub use wigle::{Account, Receipt, write_wigle};

/// A new sighting row is written once the receiver has moved this far from
/// the last one, in metres. Below it the two sightings say the same thing
/// about the same place.
pub const MOVED_M: f64 = 25.0;

/// One thing that transmits.
#[derive(Clone, Debug, PartialEq)]
pub struct Device {
    pub id: i64,
    /// The identifier space: "ble", "adsb", "ais", "aprs", "tpms".
    pub protocol: String,
    /// The identifier itself, as a person would read it.
    pub ident: String,
    /// Microseconds since the epoch.
    pub first_us: u64,
    pub last_us: u64,
    /// How many receptions were recorded, before thinning.
    pub packets: u64,
    /// What it called itself, where it says so.
    pub name: Option<String>,
    /// Who made it, where that is knowable: a company identifier's name, an
    /// OUI's owner, an aircraft's operator.
    pub vendor: Option<String>,
    /// The strongest level it was ever heard at, and where the receiver was
    /// standing at the time. The closest approach, which is the most useful
    /// single place to draw a device.
    pub best_rssi_dbfs: Option<f32>,
    pub best_lat: Option<f64>,
    pub best_lon: Option<f64>,
    /// Where it was last heard, in hertz. A device that moves band is worth
    /// seeing as one that did.
    pub center_hz: u64,
}

/// One reception.
#[derive(Clone, Copy, Debug, PartialEq, Default)]
pub struct Sighting {
    /// Microseconds since the epoch.
    pub at_us: u64,
    /// Where the receiver was, when it knew.
    pub lat: Option<f64>,
    pub lon: Option<f64>,
    pub alt_m: Option<f64>,
    /// Metres of horizontal uncertainty, from the fix's dilution where the
    /// source gave one. Kept because a sighting from a fix with an HDOP of 9
    /// and one from an HDOP of 0.8 are not the same evidence.
    pub accuracy_m: Option<f64>,
    pub rssi_dbfs: Option<f32>,
    pub snr_db: Option<f32>,
    pub center_hz: u64,
}

/// What a node hands over: who, and the reception.
#[derive(Clone, Debug)]
pub struct Report {
    pub protocol: String,
    pub ident: String,
    pub name: Option<String>,
    pub vendor: Option<String>,
    pub sighting: Sighting,
}

/// How much of a survey to return, so a pane asking for rows cannot be handed
/// a million of them.
#[derive(Clone, Copy, Debug)]
pub struct Query {
    /// Only devices heard since this time, in microseconds since the epoch.
    pub since_us: u64,
    pub limit: usize,
}

impl Default for Query {
    fn default() -> Self {
        Self { since_us: 0, limit: 5_000 }
    }
}

/// Whether a reception says something the last one did not.
pub fn worth_keeping(prev: &Sighting, now: &Sighting) -> bool {
    match ((prev.lat, prev.lon), (now.lat, now.lon)) {
        ((Some(alat), Some(alon)), (Some(blat), Some(blon))) => {
            metres(alat, alon, blat, blon) >= MOVED_M
        }
        // A position appearing where there was none is new information; both
        // missing says nothing new.
        ((None, _) | (_, None), (Some(_), Some(_))) => true,
        _ => false,
    }
}

/// Distance between two positions, flat-earth on a local scale.
///
/// Good to a fraction of a percent over the tens of metres this compares,
/// which is what decides whether a sighting is a new row. Anything that
/// wanted real distances would use the haversine formula and would not be
/// comparing against 25 metres.
pub fn metres(alat: f64, alon: f64, blat: f64, blon: f64) -> f64 {
    const DEG_M: f64 = 111_320.0;
    let dlat = (blat - alat) * DEG_M;
    let dlon = (blon - alon) * DEG_M * alat.to_radians().cos();
    (dlat * dlat + dlon * dlon).sqrt()
}
