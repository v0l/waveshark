//! The files dump1090 writes for a web map: aircraft.json, receiver.json,
//! the rolling history and stats.json.
//!
//! tar1090 and graphs1090 parse these directly, so the names of the keys are
//! dump1090's and not this receiver's. Each file is written beside its place
//! and renamed over it, because a map reading half a file draws nothing.

use crate::stats::{Counts, Stats};
use crate::track::Tracker;
use serde_json::{Map, Value, json};
use std::path::{Path, PathBuf};
use std::time::{Duration, Instant};

/// How many copies of aircraft.json tar1090 loads to draw a track laid down
/// before the page was opened, which is dump1090's number.
pub const HISTORY_FILES: usize = 120;

/// How far apart those copies are taken.
pub const HISTORY_EVERY: Duration = Duration::from_secs(30);

/// How long an aircraft stays in the file after its last frame.
///
/// The table holds one for five minutes, so a position pair still resolves
/// after a quiet spell, but a map showing an aeroplane a minute after it was
/// last heard is showing where it was, not where it is.
pub const RECENT: Duration = Duration::from_secs(60);

pub struct Writer {
    dir: PathBuf,
    every: Duration,
    here: Option<(f64, f64)>,
    wrote: Option<Instant>,
    copied: Option<Instant>,
    next: usize,
    files: usize,
    stats: bool,
}

impl Writer {
    pub fn new(dir: &Path, every: Duration, here: Option<(f64, f64)>) -> std::io::Result<Self> {
        std::fs::create_dir_all(dir)?;
        let out = Self {
            dir: dir.into(),
            every,
            here,
            wrote: None,
            copied: None,
            next: 0,
            files: 0,
            stats: true,
        };
        out.write("receiver.json", &receiver(here, every, 0))?;
        Ok(out)
    }

    /// Whatever is due, given the time that has passed since the last one.
    pub fn tick(
        &mut self,
        track: &Tracker,
        stats: &Stats,
        unix: f64,
        now: Instant,
    ) -> std::io::Result<()> {
        if self.wrote.is_some_and(|at| now.saturating_duration_since(at) < self.every) {
            return Ok(());
        }
        self.wrote = Some(now);
        let doc = aircraft(track, stats.total.messages, unix, now);
        self.write("aircraft.json", &doc)?;
        if self.stats {
            self.stats = false;
            self.write("stats.json", &self::stats(stats))?;
        }
        if self.copied.is_some_and(|at| now.saturating_duration_since(at) < HISTORY_EVERY) {
            return Ok(());
        }
        self.copied = Some(now);
        self.write(&format!("history_{}.json", self.next), &doc)?;
        self.next = (self.next + 1) % HISTORY_FILES;
        self.files = (self.files + 1).min(HISTORY_FILES);
        self.write("receiver.json", &receiver(self.here, self.every, self.files))
    }

    /// Write now, whatever is left of the interval.
    pub fn flush(
        &mut self,
        track: &Tracker,
        stats: &Stats,
        unix: f64,
        now: Instant,
    ) -> std::io::Result<()> {
        self.wrote = None;
        self.stats = true;
        self.tick(track, stats, unix, now)
    }

    /// The minute has rolled, so stats.json is due with the next write.
    pub fn minute(&mut self) {
        self.stats = true;
    }

    fn write(&self, name: &str, doc: &Value) -> std::io::Result<()> {
        let tmp = self.dir.join(format!("{name}.tmp"));
        std::fs::write(&tmp, serde_json::to_vec(doc)?)?;
        std::fs::rename(tmp, self.dir.join(name))
    }
}

pub fn receiver(here: Option<(f64, f64)>, every: Duration, history: usize) -> Value {
    let mut out = json!({
        "version": env!("CARGO_PKG_VERSION"),
        "refresh": every.as_millis() as u64,
        "history": history,
    });
    if let Some((lat, lon)) = here {
        out["lat"] = json!(lat);
        out["lon"] = json!(lon);
    }
    out
}

pub fn aircraft(track: &Tracker, messages: u64, unix: f64, now: Instant) -> Value {
    let list: Vec<Value> = track
        .recent(now, RECENT)
        .into_iter()
        .map(|(icao, a)| {
            let mut o = Map::new();
            o.insert("hex".into(), json!(format!("{icao:06x}")));
            o.insert("type".into(), json!("adsb_icao"));
            if let Some(call) = &a.callsign {
                o.insert("flight".into(), json!(call));
            }
            match (a.ground, a.altitude_ft) {
                (true, _) => o.insert("alt_baro".into(), json!("ground")),
                (false, Some(alt)) => o.insert("alt_baro".into(), json!(alt)),
                (false, None) => None,
            };
            if let Some(gs) = a.ground_speed_kt {
                o.insert("gs".into(), json!(round(gs, 1)));
            }
            if let Some(t) = a.track_deg {
                o.insert("track".into(), json!(round(t, 1)));
            }
            if let Some(r) = a.vertical_rate_fpm {
                o.insert("baro_rate".into(), json!(r));
            }
            if let Some(sq) = a.squawk {
                o.insert("squawk".into(), json!(format!("{sq:04}")));
            }
            if let (Some((lat, lon)), Some(age)) = (a.at, a.seen_pos(now)) {
                o.insert("lat".into(), json!(round(lat, 5)));
                o.insert("lon".into(), json!(round(lon, 5)));
                o.insert("seen_pos".into(), json!(round(age, 1)));
            }
            if let Some(rssi) = a.rssi_dbfs() {
                o.insert("rssi".into(), json!(round(rssi as f64, 1)));
            }
            o.insert("messages".into(), json!(a.messages));
            o.insert("seen".into(), json!(round(a.seen(now), 1)));
            Value::Object(o)
        })
        .collect();
    json!({ "now": round(unix, 1), "messages": messages, "aircraft": list })
}

pub fn stats(s: &Stats) -> Value {
    json!({ "total": period(&s.total), "last1min": period(&s.last_minute) })
}

fn period(c: &Counts) -> Value {
    let mut local = json!({
        "samples_processed": c.samples_processed,
        "samples_dropped": c.samples_dropped,
        "modeac": 0,
        "accepted": c.accepted.to_vec(),
        "strong_signals": c.strong_signals,
    });
    // Left out rather than sent as zero where nothing measured it: the
    // collectd plugin draws a graph for a key that is there.
    if let Some(signal) = c.signal_dbfs() {
        local["signal"] = json!(round(signal, 1));
    }
    if let Some(peak) = c.peak_dbfs() {
        local["peak_signal"] = json!(round(peak, 1));
    }
    json!({
        "start": round(c.start, 1),
        "end": round(c.end, 1),
        "local": local,
        "remote": { "accepted": [c.remote_accepted] },
        "cpr": {
            "global_ok": c.cpr_global_ok,
            "local_ok": c.cpr_local_ok,
        },
        "tracks": { "all": c.tracks, "single_message": c.single_message_tracks },
        "cpu": {},
        "messages": c.messages,
        "messages_by_df": c.messages_by_df.to_vec(),
    })
}

fn round(v: f64, places: u32) -> f64 {
    let scale = 10f64.powi(places as i32);
    (v * scale).round() / scale
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::stats::{Source, Stats};

    fn scratch(name: &str) -> PathBuf {
        let dir = std::env::temp_dir().join(format!("wave1090-json-{name}-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        dir
    }

    fn read(dir: &Path, name: &str) -> Value {
        let bytes = std::fs::read(dir.join(name)).unwrap_or_else(|e| panic!("{name}: {e}"));
        serde_json::from_slice(&bytes).unwrap_or_else(|e| panic!("{name} is not JSON: {e}"))
    }

    /// tar1090 asks receiver.json how many history files it may load, so a
    /// receiver left running may not answer with more than it wrote.
    #[test]
    fn the_history_stops_at_a_hundred_and_twenty_files_and_writes_over_the_oldest() {
        let dir = scratch("history");
        let (track, stats) = (Tracker::default(), Stats::new(0.0));
        let mut w = Writer::new(&dir, Duration::from_millis(1), None).expect("a writer");
        let base = Instant::now();
        for n in 0..130u32 {
            w.tick(&track, &stats, n as f64, base + HISTORY_EVERY * n).expect("a write");
        }
        assert_eq!(read(&dir, "receiver.json")["history"], 120, "history files offered");
        let written = std::fs::read_dir(&dir)
            .unwrap()
            .filter_map(|e| Some(e.ok()?.file_name().to_str()?.to_string()))
            .filter(|n| n.starts_with("history_"))
            .count();
        assert_eq!(written, 120, "files on disk");
        assert_eq!(read(&dir, "history_119.json")["now"], 119.0, "the last of the cycle");
        assert_eq!(read(&dir, "history_9.json")["now"], 129.0, "where the cycle got back to");
        // The hundred and twenty-first went over history_0, which is why
        // tar1090 sorts the files by their own `now` rather than by name.
        assert_eq!(read(&dir, "history_0.json")["now"], 120.0, "the oldest was written over");
        assert!(!dir.join("history_120.json").exists());
    }

    /// A map reading a file being written draws nothing, so nothing is ever
    /// written in place.
    #[test]
    fn no_half_written_file_is_left_where_a_map_would_read_it() {
        let dir = scratch("atomic");
        let (track, mut stats) = (Tracker::default(), Stats::new(0.0));
        stats.frame(17, -12.0, Source::Air, 0);
        let mut w = Writer::new(&dir, Duration::ZERO, Some((53.4, -6.2))).expect("a writer");
        let base = Instant::now();
        for n in 0..3u32 {
            w.tick(&track, &stats, n as f64, base + HISTORY_EVERY * n).expect("a write");
        }
        let mut names: Vec<String> = std::fs::read_dir(&dir)
            .unwrap()
            .map(|e| e.unwrap().file_name().to_str().unwrap().to_string())
            .collect();
        names.sort();
        assert_eq!(
            names,
            [
                "aircraft.json",
                "history_0.json",
                "history_1.json",
                "history_2.json",
                "receiver.json",
                "stats.json"
            ],
            "no .tmp left behind"
        );
        for name in &names {
            read(&dir, name);
        }
        let recv = read(&dir, "receiver.json");
        assert_eq!((recv["lat"].as_f64(), recv["lon"].as_f64()), (Some(53.4), Some(-6.2)));
        assert_eq!(recv["refresh"], 0, "how often a map should come back, in ms");
    }

    /// stats.json is written once a minute, as dump1090 writes it, and not
    /// with every copy of aircraft.json.
    #[test]
    fn stats_json_is_rewritten_when_the_minute_rolls_and_not_before() {
        let dir = scratch("stats");
        let (track, stats) = (Tracker::default(), Stats::new(0.0));
        let mut w = Writer::new(&dir, Duration::ZERO, None).expect("a writer");
        let base = Instant::now();
        w.tick(&track, &stats, 1.0, base).expect("a write");
        assert_eq!(read(&dir, "stats.json")["total"]["end"], 0.0, "written at startup");

        let mut later = Stats::new(0.0);
        later.tick(90.0, base + Duration::from_secs(90));
        w.tick(&track, &later, 90.0, base + Duration::from_secs(90)).expect("a write");
        assert_eq!(read(&dir, "stats.json")["total"]["end"], 0.0, "no minute said to");
        w.minute();
        w.tick(&track, &later, 91.0, base + Duration::from_secs(91)).expect("a write");
        assert_eq!(read(&dir, "stats.json")["total"]["end"], 90.0, "the minute that rolled");
    }
}
