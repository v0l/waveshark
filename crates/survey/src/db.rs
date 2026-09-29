use crate::{Device, Places, Query, Report, Sighting};
use common::{Error, Result};
use rusqlite::{Connection, OptionalExtension, params};
use std::path::{Path, PathBuf};

/// Schema version, written into the file. A file from a newer version is
/// refused rather than half read.
const VERSION: i64 = 1;

pub struct Db {
    conn: Connection,
    path: PathBuf,
    /// The last sighting written per device, for the thinning decision. Held
    /// here rather than read back per packet: a busy band is thousands of
    /// packets a second and each would otherwise be a query.
    kept: std::collections::HashMap<i64, Kept>,
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
        Ok(Self { conn, path, kept: Default::default(), devices: Default::default(), written: 0 })
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
                )));
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
        Ok(Self { conn, path, kept: Default::default(), devices: Default::default(), written: 0 })
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
        if !self.kept.contains_key(&id) {
            let had = self.sightings(id)?;
            let mut places = Places::default();
            for h in &had {
                if let (Some(lat), Some(lon)) = (h.lat, h.lon) {
                    places.add(lat, lon);
                }
            }
            self.kept.insert(id, Kept { places, any: !had.is_empty() });
        }
        let kept = self.kept.get_mut(&id).expect("just inserted");
        let fresh = match (s.lat, s.lon) {
            (Some(lat), Some(lon)) => kept.places.add(lat, lon),
            _ => !kept.any,
        };
        kept.any |= fresh;
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

    pub fn sighting_counts(&self) -> Result<std::collections::HashMap<i64, u64>> {
        let mut st = self
            .conn
            .prepare("SELECT device, COUNT(*) FROM sightings GROUP BY device")
            .map_err(sql)?;
        let rows = st
            .query_map([], |r| Ok((r.get::<_, i64>(0)?, r.get::<_, i64>(1)? as u64)))
            .map_err(sql)?;
        rows.collect::<std::result::Result<_, _>>().map_err(sql)
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

struct Kept {
    places: Places,
    any: bool,
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

    #[test]
    fn a_parked_receiver_writes_nothing_however_the_level_or_the_hour_changes() {
        let mut db = Db::in_memory().unwrap();
        assert!(db.record(&report("AA:BB", 0, 53.0, -6.0, -70.0)).unwrap());
        assert!(!db.record(&report("AA:BB", 1, 53.0, -6.0, -40.0)).unwrap(), "30 dB louder");
        assert!(!db.record(&report("AA:BB", 7_200, 53.0001, -6.0, -70.0)).unwrap(), "2 h, 11 m");
        assert_eq!(db.counts().unwrap(), (1, 1));
        let d = &db.devices(Query::default()).unwrap()[0];
        assert_eq!(d.last_us, 7_200_000_000, "the device row still says when it was last heard");
        assert_eq!(d.packets, 3);
    }

    #[test]
    fn coming_back_to_a_place_already_kept_writes_nothing_even_after_a_restart() {
        let dir =
            common::platform::scratch_dir().join(format!("survey-places-{}", std::process::id()));
        let path = dir.join("survey.sqlite");
        let _ = std::fs::remove_file(&path);
        {
            let mut db = Db::open(&path).unwrap();
            assert!(db.record(&report("AA:BB", 0, 53.0, -6.0, -50.0)).unwrap());
            assert!(db.record(&report("AA:BB", 10, 53.001, -6.0, -50.0)).unwrap());
            assert!(
                !db.record(&report("AA:BB", 20, 53.0, -6.0, -50.0)).unwrap(),
                "back at the first"
            );
        }
        let mut db = Db::open(&path).unwrap();
        assert!(!db.record(&report("AA:BB", 30, 53.00005, -6.0, -50.0)).unwrap(), "6 m from it");
        assert!(!db.record(&report("AA:BB", 40, 53.001, -6.0, -50.0)).unwrap(), "at the second");
        assert!(db.record(&report("AA:BB", 50, 53.002, -6.0, -50.0)).unwrap(), "a third place");
        assert_eq!(db.counts().unwrap(), (1, 3));
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn a_device_never_given_a_position_keeps_one_sighting_until_it_has_one() {
        let mut db = Db::in_memory().unwrap();
        let mut r = report("AA:BB", 0, 0.0, 0.0, -50.0);
        r.sighting.lat = None;
        r.sighting.lon = None;
        assert!(db.record(&r).unwrap());
        r.sighting.at_us = 600_000_000;
        assert!(!db.record(&r).unwrap(), "a second sighting from nowhere says nothing");
        assert!(db.record(&report("AA:BB", 700, 53.0, -6.0, -50.0)).unwrap(), "a fix is a place");
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
        let dir =
            common::platform::scratch_dir().join(format!("survey-test-{}", std::process::id()));
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
