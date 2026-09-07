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
//! same thing, so a new row is written when the receiver has moved, when the
//! level has changed enough to be worth having, or when enough time has
//! passed; otherwise the existing row's time and level are updated. What is
//! never thinned away is the first and last time a device was heard, because
//! those are on the device row.
//!
//! # SQLite, and why a file rather than a format
//!
//! The packet log is hand-written binary because it is a stream of bursts
//! nobody queries until later. This is queried constantly by whatever is
//! drawing it, wants indexes on time and on identity, has to survive the
//! laptop being closed mid-drive, and is small enough that the storage cost
//! of a real database does not matter. `rusqlite` is already in the tree for
//! the Artemis signal database, bundled, so this costs no new dependency.

use common::{Error, Result};
use rusqlite::{params, Connection, OptionalExtension};
use std::path::{Path, PathBuf};

mod locate;
mod wigle;
pub use locate::{locate, Estimate};
pub use wigle::write_wigle;

/// Schema version, written into the file. A file from a newer version is
/// refused rather than half read.
const VERSION: i64 = 1;

/// A new sighting row is written once the receiver has moved this far from
/// the last one, in metres. Below it the two sightings say the same thing
/// about the same place.
pub const MOVED_M: f64 = 25.0;

/// Or once the level has changed by this much, which is what says the
/// distance changed even when the position did not: a device driving past a
/// parked receiver.
pub const LEVEL_DB: f32 = 6.0;

/// Or once this many seconds have passed, so a long stationary survey still
/// records that something kept transmitting.
pub const INTERVAL_S: u64 = 60;

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

pub struct Db {
    conn: Connection,
    path: PathBuf,
    /// The last sighting written per device, for the thinning decision. Held
    /// here rather than read back per packet: a busy band is thousands of
    /// packets a second and each would otherwise be a query.
    last: std::collections::HashMap<i64, Sighting>,
    devices: std::collections::HashMap<(String, String), i64>,
    written: u64,
}

impl Db {
    /// Open or create a survey at `path`.
    pub fn open(path: impl AsRef<Path>) -> Result<Self> {
        let path = path.as_ref().to_path_buf();
        if let Some(dir) = path.parent() {
            std::fs::create_dir_all(dir).map_err(|e| Error::other(format!("survey dir: {e}")))?;
        }
        let conn = Connection::open(&path).map_err(sql)?;
        Self::from_conn(conn, path)
    }

    /// Open a survey to read it, without creating one.
    ///
    /// The interface draws a survey the radio thread is writing, and the two
    /// are different connections to the same file: SQLite in write-ahead mode
    /// lets a reader run while the writer appends, which is the whole reason
    /// this is a database rather than a table held in the node. A handle
    /// opened this way must not record; `record` on it would fail on the
    /// first write.
    pub fn open_read(path: impl AsRef<Path>) -> Result<Self> {
        let path = path.as_ref().to_path_buf();
        let conn = Connection::open_with_flags(
            &path,
            rusqlite::OpenFlags::SQLITE_OPEN_READ_ONLY | rusqlite::OpenFlags::SQLITE_OPEN_NO_MUTEX,
        )
        .map_err(sql)?;
        Ok(Self {
            conn,
            path,
            last: Default::default(),
            devices: Default::default(),
            written: 0,
        })
    }

    /// A survey held in memory, for a test or a replay whose result nobody
    /// wants on disk.
    pub fn in_memory() -> Result<Self> {
        Self::from_conn(Connection::open_in_memory().map_err(sql)?, PathBuf::new())
    }

    fn from_conn(conn: Connection, path: PathBuf) -> Result<Self> {
        // A survey is written continuously by one process and read by the
        // same one. WAL keeps a reader from blocking the writer, and normal
        // synchronisation is the trade every logger makes: a power cut can
        // cost the last transaction, and the alternative is an fsync per
        // packet on a band that produces thousands.
        conn.pragma_update(None, "journal_mode", "WAL").map_err(sql)?;
        conn.pragma_update(None, "synchronous", "NORMAL").map_err(sql)?;
        conn.execute_batch(
            "CREATE TABLE IF NOT EXISTS meta (key TEXT PRIMARY KEY, value TEXT NOT NULL);
             CREATE TABLE IF NOT EXISTS devices (
                 id INTEGER PRIMARY KEY,
                 protocol TEXT NOT NULL,
                 ident TEXT NOT NULL,
                 first_us INTEGER NOT NULL,
                 last_us INTEGER NOT NULL,
                 packets INTEGER NOT NULL DEFAULT 0,
                 name TEXT,
                 vendor TEXT,
                 best_rssi_dbfs REAL,
                 best_lat REAL,
                 best_lon REAL,
                 center_hz INTEGER NOT NULL DEFAULT 0,
                 UNIQUE(protocol, ident));
             CREATE TABLE IF NOT EXISTS sightings (
                 device INTEGER NOT NULL REFERENCES devices(id),
                 at_us INTEGER NOT NULL,
                 lat REAL, lon REAL, alt_m REAL, accuracy_m REAL,
                 rssi_dbfs REAL, snr_db REAL,
                 center_hz INTEGER NOT NULL);
             CREATE INDEX IF NOT EXISTS sightings_device ON sightings(device, at_us);
             CREATE INDEX IF NOT EXISTS devices_last ON devices(last_us);",
        )
        .map_err(sql)?;
        let have: Option<String> = conn
            .query_row("SELECT value FROM meta WHERE key = 'version'", [], |r| r.get(0))
            .optional()
            .map_err(sql)?;
        match have.as_deref().map(str::parse::<i64>) {
            Some(Ok(v)) if v > VERSION => {
                return Err(Error::other(format!(
                    "survey at {} is version {v}, this build reads {VERSION}",
                    path.display()
                )))
            }
            Some(_) => {}
            None => {
                conn.execute(
                    "INSERT INTO meta(key, value) VALUES ('version', ?1)",
                    params![VERSION.to_string()],
                )
                .map_err(sql)?;
            }
        }
        Ok(Self {
            conn,
            path,
            last: Default::default(),
            devices: Default::default(),
            written: 0,
        })
    }

    pub fn path(&self) -> &Path {
        &self.path
    }

    /// Sighting rows written since this was opened.
    pub fn written(&self) -> u64 {
        self.written
    }

    /// Record a reception, thinning it against the last one for the same
    /// device. Returns whether a sighting row was written.
    pub fn record(&mut self, r: &Report) -> Result<bool> {
        let id = self.device_id(r)?;
        let s = r.sighting;
        let fresh = match self.last.get(&id) {
            None => true,
            Some(prev) => worth_keeping(prev, &s),
        };
        self.conn
            .execute(
                "UPDATE devices SET last_us = MAX(last_us, ?2), packets = packets + 1,
                     center_hz = ?3,
                     name = COALESCE(?4, name), vendor = COALESCE(?5, vendor)
                 WHERE id = ?1",
                params![id, s.at_us as i64, s.center_hz as i64, r.name, r.vendor],
            )
            .map_err(sql)?;
        // The strongest reception and where it was heard from, which is the
        // one place worth drawing a device at.
        if let Some(rssi) = s.rssi_dbfs {
            self.conn
                .execute(
                    "UPDATE devices SET best_rssi_dbfs = ?2, best_lat = ?3, best_lon = ?4
                     WHERE id = ?1 AND (best_rssi_dbfs IS NULL OR best_rssi_dbfs < ?2)",
                    params![id, rssi, s.lat, s.lon],
                )
                .map_err(sql)?;
        }
        // A front end that reads bits rather than power reports no level at
        // all, which used to leave those devices with no place on the map
        // even though every sighting had one. Without a level to compare, the
        // first position heard is the one kept: it is the only claim there is.
        if s.lat.is_some() {
            self.conn
                .execute(
                    "UPDATE devices SET best_lat = ?2, best_lon = ?3
                     WHERE id = ?1 AND best_lat IS NULL",
                    params![id, s.lat, s.lon],
                )
                .map_err(sql)?;
        }
        if !fresh {
            return Ok(false);
        }
        self.conn
            .execute(
                "INSERT INTO sightings(device, at_us, lat, lon, alt_m, accuracy_m,
                                       rssi_dbfs, snr_db, center_hz)
                 VALUES (?1, ?2, ?3, ?4, ?5, ?6, ?7, ?8, ?9)",
                params![
                    id,
                    s.at_us as i64,
                    s.lat,
                    s.lon,
                    s.alt_m,
                    s.accuracy_m,
                    s.rssi_dbfs,
                    s.snr_db,
                    s.center_hz as i64
                ],
            )
            .map_err(sql)?;
        self.last.insert(id, s);
        self.written += 1;
        Ok(true)
    }

    fn device_id(&mut self, r: &Report) -> Result<i64> {
        let key = (r.protocol.clone(), r.ident.clone());
        if let Some(id) = self.devices.get(&key) {
            return Ok(*id);
        }
        self.conn
            .execute(
                "INSERT INTO devices(protocol, ident, first_us, last_us, name, vendor, center_hz)
                 VALUES (?1, ?2, ?3, ?3, ?4, ?5, ?6)
                 ON CONFLICT(protocol, ident) DO NOTHING",
                params![
                    r.protocol,
                    r.ident,
                    r.sighting.at_us as i64,
                    r.name,
                    r.vendor,
                    r.sighting.center_hz as i64
                ],
            )
            .map_err(sql)?;
        let id: i64 = self
            .conn
            .query_row(
                "SELECT id FROM devices WHERE protocol = ?1 AND ident = ?2",
                params![r.protocol, r.ident],
                |row| row.get(0),
            )
            .map_err(sql)?;
        self.devices.insert(key, id);
        Ok(id)
    }

    /// Devices heard, most recently heard first.
    pub fn devices(&self, q: Query) -> Result<Vec<Device>> {
        let mut st = self
            .conn
            .prepare(
                "SELECT id, protocol, ident, first_us, last_us, packets, name, vendor,
                        best_rssi_dbfs, best_lat, best_lon, center_hz
                 FROM devices WHERE last_us >= ?1 ORDER BY last_us DESC LIMIT ?2",
            )
            .map_err(sql)?;
        let rows = st
            .query_map(params![q.since_us as i64, q.limit as i64], |r| {
                Ok(Device {
                    id: r.get(0)?,
                    protocol: r.get(1)?,
                    ident: r.get(2)?,
                    first_us: r.get::<_, i64>(3)? as u64,
                    last_us: r.get::<_, i64>(4)? as u64,
                    packets: r.get::<_, i64>(5)? as u64,
                    name: r.get(6)?,
                    vendor: r.get(7)?,
                    best_rssi_dbfs: r.get(8)?,
                    best_lat: r.get(9)?,
                    best_lon: r.get(10)?,
                    center_hz: r.get::<_, i64>(11)? as u64,
                })
            })
            .map_err(sql)?;
        rows.collect::<std::result::Result<Vec<_>, _>>().map_err(sql)
    }

    /// Every sighting of one device, oldest first, which is the trail it was
    /// heard along.
    pub fn sightings(&self, device: i64) -> Result<Vec<Sighting>> {
        let mut st = self
            .conn
            .prepare(
                "SELECT at_us, lat, lon, alt_m, accuracy_m, rssi_dbfs, snr_db, center_hz
                 FROM sightings WHERE device = ?1 ORDER BY at_us",
            )
            .map_err(sql)?;
        let rows = st
            .query_map(params![device], |r| {
                Ok(Sighting {
                    at_us: r.get::<_, i64>(0)? as u64,
                    lat: r.get(1)?,
                    lon: r.get(2)?,
                    alt_m: r.get(3)?,
                    accuracy_m: r.get(4)?,
                    rssi_dbfs: r.get(5)?,
                    snr_db: r.get(6)?,
                    center_hz: r.get::<_, i64>(7)? as u64,
                })
            })
            .map_err(sql)?;
        rows.collect::<std::result::Result<Vec<_>, _>>().map_err(sql)
    }

    /// How many devices and sightings the survey holds.
    pub fn counts(&self) -> Result<(u64, u64)> {
        let d: i64 =
            self.conn.query_row("SELECT COUNT(*) FROM devices", [], |r| r.get(0)).map_err(sql)?;
        let s: i64 =
            self.conn.query_row("SELECT COUNT(*) FROM sightings", [], |r| r.get(0)).map_err(sql)?;
        Ok((d as u64, s as u64))
    }
}

/// Whether a reception says something the last one did not.
fn worth_keeping(prev: &Sighting, now: &Sighting) -> bool {
    if now.at_us.saturating_sub(prev.at_us) >= INTERVAL_S * 1_000_000 {
        return true;
    }
    if let (Some(a), Some(b)) = (prev.rssi_dbfs, now.rssi_dbfs) {
        if (a - b).abs() >= LEVEL_DB {
            return true;
        }
    }
    match ((prev.lat, prev.lon), (now.lat, now.lon)) {
        ((Some(alat), Some(alon)), (Some(blat), Some(blon))) => {
            metres(alat, alon, blat, blon) >= MOVED_M
        }
        // A position appearing where there was none is new information; both
        // missing is the stationary case the interval covers.
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

fn sql(e: rusqlite::Error) -> Error {
    Error::other(format!("survey: {e}"))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn report(ident: &str, at_s: u64, lat: f64, lon: f64, rssi: f32) -> Report {
        Report {
            protocol: "ble".into(),
            ident: ident.into(),
            name: None,
            vendor: None,
            sighting: Sighting {
                at_us: at_s * 1_000_000,
                lat: Some(lat),
                lon: Some(lon),
                rssi_dbfs: Some(rssi),
                center_hz: 2_426_000_000,
                ..Default::default()
            },
        }
    }

    #[test]
    fn a_device_is_created_once_and_its_times_span_what_was_heard() {
        let mut db = Db::in_memory().unwrap();
        db.record(&report("AA:BB", 100, 53.0, -6.0, -50.0)).unwrap();
        db.record(&report("AA:BB", 400, 53.0, -6.0, -50.0)).unwrap();
        let rows = db.devices(Query::default()).unwrap();
        assert_eq!(rows.len(), 1, "one device, heard twice");
        assert_eq!(rows[0].first_us, 100_000_000);
        assert_eq!(rows[0].last_us, 400_000_000);
        assert_eq!(rows[0].packets, 2, "every reception counts even when thinned");
    }

    /// A beacon transmitting ten times a second from a parked car must not
    /// write ten rows a second.
    #[test]
    fn a_stationary_repeat_is_thinned_away() {
        let mut db = Db::in_memory().unwrap();
        assert!(db.record(&report("AA:BB", 0, 53.0, -6.0, -50.0)).unwrap());
        for i in 1..50 {
            assert!(
                !db.record(&report("AA:BB", i, 53.0, -6.0, -50.0)).unwrap(),
                "second {i} wrote a row saying the same thing"
            );
        }
        assert_eq!(db.counts().unwrap(), (1, 1));
    }

    /// Moving is the whole point of a survey, so movement always writes.
    #[test]
    fn moving_writes_a_new_sighting() {
        let mut db = Db::in_memory().unwrap();
        db.record(&report("AA:BB", 0, 53.0, -6.0, -50.0)).unwrap();
        // About 33 metres north.
        assert!(db.record(&report("AA:BB", 1, 53.0003, -6.0, -50.0)).unwrap());
        assert_eq!(db.counts().unwrap().1, 2);
    }

    /// A device driving past a parked receiver never moves the receiver, and
    /// the level is the only thing that says it happened.
    #[test]
    fn a_level_change_writes_even_without_movement() {
        let mut db = Db::in_memory().unwrap();
        db.record(&report("AA:BB", 0, 53.0, -6.0, -70.0)).unwrap();
        assert!(!db.record(&report("AA:BB", 1, 53.0, -6.0, -68.0)).unwrap(), "2 dB is noise");
        assert!(db.record(&report("AA:BB", 2, 53.0, -6.0, -55.0)).unwrap(), "15 dB is an approach");
    }

    /// The closest approach is where a device is worth drawing, so the
    /// strongest reception and the position it came from are kept together.
    #[test]
    fn the_strongest_sighting_and_its_position_are_remembered() {
        let mut db = Db::in_memory().unwrap();
        db.record(&report("AA:BB", 0, 53.0, -6.0, -80.0)).unwrap();
        db.record(&report("AA:BB", 100, 53.1, -6.1, -40.0)).unwrap();
        db.record(&report("AA:BB", 200, 53.2, -6.2, -75.0)).unwrap();
        let d = &db.devices(Query::default()).unwrap()[0];
        assert_eq!(d.best_rssi_dbfs, Some(-40.0));
        assert_eq!(d.best_lat, Some(53.1), "the position of the strongest, not of the last");
    }

    /// A demodulator that reads bits reports no level, and those devices
    /// still have to be somewhere on the map.
    #[test]
    fn a_device_heard_without_a_level_still_keeps_a_position() {
        let mut db = Db::in_memory().unwrap();
        let mut r = report("AA:BB", 0, 53.0, -6.0, 0.0);
        r.sighting.rssi_dbfs = None;
        db.record(&r).unwrap();
        let d = &db.devices(Query::default()).unwrap()[0];
        assert_eq!(d.best_lat, Some(53.0));
        assert_eq!(d.best_rssi_dbfs, None, "and no level is invented for it");
    }

    /// Two protocols can hand over the same string and mean different things,
    /// which is why identity is a pair.
    #[test]
    fn the_same_identifier_in_two_protocols_is_two_devices() {
        let mut db = Db::in_memory().unwrap();
        db.record(&report("1234", 0, 53.0, -6.0, -50.0)).unwrap();
        let mut other = report("1234", 0, 53.0, -6.0, -50.0);
        other.protocol = "pocsag".into();
        db.record(&other).unwrap();
        assert_eq!(db.counts().unwrap().0, 2);
    }

    /// A name arriving later fills in a device heard before it said one, and
    /// a packet without a name does not erase it.
    #[test]
    fn a_name_learned_later_is_kept() {
        let mut db = Db::in_memory().unwrap();
        db.record(&report("AA:BB", 0, 53.0, -6.0, -50.0)).unwrap();
        let mut named = report("AA:BB", 100, 53.0, -6.0, -50.0);
        named.name = Some("EVCS".into());
        named.vendor = Some("Victron Energy".into());
        db.record(&named).unwrap();
        db.record(&report("AA:BB", 200, 53.0, -6.0, -50.0)).unwrap();
        let d = &db.devices(Query::default()).unwrap()[0];
        assert_eq!(d.name.as_deref(), Some("EVCS"));
        assert_eq!(d.vendor.as_deref(), Some("Victron Energy"));
    }

    /// A survey is a file that outlives the run that wrote it.
    #[test]
    fn a_survey_reopens_with_what_was_recorded() {
        let dir = std::env::temp_dir().join(format!("survey-test-{}", std::process::id()));
        let path = dir.join("survey.sqlite");
        let _ = std::fs::remove_file(&path);
        {
            let mut db = Db::open(&path).unwrap();
            db.record(&report("AA:BB", 0, 53.0, -6.0, -50.0)).unwrap();
        }
        let db = Db::open(&path).unwrap();
        assert_eq!(db.counts().unwrap(), (1, 1));
        let _ = std::fs::remove_dir_all(&dir);
    }

    /// A sighting with no fix is still a sighting: the receiver heard it, and
    /// indoors or before a lock there is no position to attach.
    #[test]
    fn a_sighting_without_a_position_is_recorded() {
        let mut db = Db::in_memory().unwrap();
        let mut r = report("AA:BB", 0, 0.0, 0.0, -50.0);
        r.sighting.lat = None;
        r.sighting.lon = None;
        assert!(db.record(&r).unwrap());
        let s = &db.sightings(1).unwrap()[0];
        assert_eq!((s.lat, s.lon), (None, None));
    }
}
