use crate::{Dial, Listing, Query};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::net::SocketAddr;
use std::path::Path;
use std::sync::Mutex;
use std::sync::atomic::{AtomicUsize, Ordering};

pub const WORKERS: usize = 10;

pub const ANSWERED_FOR: u64 = 60 * 60;

pub const SILENT_FOR: u64 = 4 * 60 * 60;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum Reached {
    Listed,
    Ours(SocketAddr),
    Near(SocketAddr),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct Said {
    pub reached: Reached,
    pub center_hz: Option<u64>,
    pub dial: Option<Dial>,
    pub clients: Option<u32>,
}

impl Said {
    pub const LISTED: Said = Said::at(Reached::Listed);

    pub const fn at(reached: Reached) -> Said {
        Said { reached, center_hz: None, dial: None, clients: None }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum Heard {
    Answered(Said),
    Silent,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct Probed {
    pub at: u64,
    pub heard: Heard,
}

impl Probed {
    pub fn stale(&self, now: u64) -> bool {
        let lasts = match self.heard {
            Heard::Answered(_) => ANSWERED_FOR,
            Heard::Silent => SILENT_FOR,
        };
        now.saturating_sub(self.at) >= lasts
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Tally {
    pub listed: usize,
    pub checked: usize,
    pub answering: usize,
}

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Probes(HashMap<String, Probed>);

impl Probes {
    pub fn read(path: &Path) -> Self {
        std::fs::read(path)
            .ok()
            .and_then(|raw| serde_json::from_slice(&raw).ok())
            .unwrap_or_default()
    }

    pub fn write(&self, path: &Path) -> std::io::Result<()> {
        let raw = serde_json::to_vec(self).map_err(std::io::Error::other)?;
        let part = path.with_extension("json.part");
        std::fs::write(&part, raw)?;
        std::fs::rename(&part, path)
    }

    pub fn get(&self, l: &Listing) -> Option<Probed> {
        self.0.get(&l.entry.addr()).copied()
    }

    pub fn heard(&self, l: &Listing) -> Option<Heard> {
        self.get(l).map(|p| p.heard)
    }

    pub fn insert(&mut self, l: &Listing, probed: Probed) {
        self.0.insert(l.entry.addr(), probed);
    }

    pub fn len(&self) -> usize {
        self.0.len()
    }

    pub fn is_empty(&self) -> bool {
        self.0.is_empty()
    }

    pub fn due<'a>(&self, listings: &'a [Listing], now: u64) -> Vec<&'a Listing> {
        listings.iter().filter(|l| self.get(l).is_none_or(|p| p.stale(now))).collect()
    }

    pub fn tally(&self, listings: &[Listing]) -> Tally {
        listings.iter().fold(Tally::default(), |mut t, l| {
            t.listed += 1;
            match self.heard(l) {
                Some(Heard::Answered(_)) => {
                    t.checked += 1;
                    t.answering += 1;
                }
                Some(Heard::Silent) => t.checked += 1,
                None => {}
            }
            t
        })
    }

    fn keep_listed(&mut self, listings: &[Listing]) {
        self.0.retain(|addr, _| listings.iter().any(|l| l.entry.addr() == *addr));
    }
}

pub fn sweep(
    probes: &Mutex<Probes>,
    listings: &[Listing],
    now: u64,
    workers: usize,
    probe: impl Fn(&Listing) -> Heard + Sync,
) -> usize {
    let due = {
        let mut held = probes.lock().unwrap_or_else(|e| e.into_inner());
        held.keep_listed(listings);
        held.due(listings, now)
    };
    let next = AtomicUsize::new(0);
    std::thread::scope(|scope| {
        for _ in 0..workers.min(due.len()) {
            scope.spawn(|| {
                while let Some(l) = due.get(next.fetch_add(1, Ordering::Relaxed)) {
                    let heard = probe(l);
                    probes
                        .lock()
                        .unwrap_or_else(|e| e.into_inner())
                        .insert(l, Probed { at: now, heard });
                }
            });
        }
    });
    due.len()
}

impl Listing {
    pub fn as_heard(&self, said: &Said) -> Listing {
        let mut l = self.clone();
        let s = &mut l.entry.station;
        if let Some(c) = said.clients {
            s.clients = c;
        }
        if let [t] = s.tuners.as_mut_slice() {
            if let Some(hz) = said.center_hz {
                t.center_hz = hz;
            }
            if let Some(d) = said.dial {
                t.dial = d;
            }
        }
        l
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct Found {
    pub listing: Listing,
    pub heard: Option<Heard>,
}

impl Found {
    pub fn said(&self) -> Option<&Said> {
        match &self.heard {
            Some(Heard::Answered(s)) => Some(s),
            Some(Heard::Silent) | None => None,
        }
    }

    pub fn answered(&self) -> bool {
        self.said().is_some()
    }

    pub fn addr(&self) -> String {
        match self.said().map(|s| s.reached) {
            Some(Reached::Ours(at) | Reached::Near(at)) => at.to_string(),
            Some(Reached::Listed) | None => self.listing.entry.addr(),
        }
    }

    fn rank(&self) -> u8 {
        match (&self.heard, self.said().map(|s| s.reached)) {
            (_, Some(Reached::Ours(_))) => 0,
            (_, Some(Reached::Near(_))) => 1,
            (_, Some(Reached::Listed)) => 2,
            (None, None) => 3,
            (Some(_), None) => 4,
        }
    }
}

pub fn shown(listings: &[Listing], probes: &Probes, query: &Query, now: u64) -> Vec<Found> {
    let mut out: Vec<Found> = listings
        .iter()
        .map(|l| {
            let heard = probes.heard(l);
            let listing = match &heard {
                Some(Heard::Answered(said)) => l.as_heard(said),
                Some(Heard::Silent) | None => l.clone(),
            };
            Found { listing, heard }
        })
        .filter(|f| f.heard != Some(Heard::Silent) && query.keeps(&f.listing, now))
        .collect();
    out.sort_by_cached_key(|f| {
        let s = &f.listing.entry.station;
        (f.rank(), !s.has_slot(), !s.tuners.iter().any(|t| t.tunable()), s.name.to_lowercase())
    });
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::model::fixtures::{airband, entry, hf};
    use crate::{Author, Station};
    use std::time::Duration;

    fn listed(name: &str, port: u16, tuners: Vec<crate::Tuner>) -> Listing {
        let mut entry = entry("203.0.113.9", tuners);
        entry.port = port;
        entry.station.name = name.into();
        Listing { author: Author(name.into()), seen: 1000, entry }
    }

    fn listings() -> Vec<Listing> {
        vec![
            listed("answers", 1, vec![airband()]),
            listed("silent", 2, vec![hf()]),
            listed("mute", 3, vec![airband()]),
        ]
    }

    fn by_name(l: &Listing) -> Heard {
        match l.entry.station.name.as_str() {
            "answers" => Heard::Answered(Said {
                center_hz: Some(145_800_000),
                dial: Some(Dial::Tunable { min_hz: Some(24_000_000), max_hz: Some(1_700_000_000) }),
                clients: Some(3),
                ..Said::LISTED
            }),
            _ => Heard::Silent,
        }
    }

    fn swept(now: u64) -> (Mutex<Probes>, Vec<Listing>) {
        let (probes, v) = (Mutex::new(Probes::default()), listings());
        assert_eq!(sweep(&probes, &v, now, WORKERS, by_name), 3);
        (probes, v)
    }

    fn names(found: &[Found]) -> Vec<&str> {
        found.iter().map(|f| f.listing.entry.station.name.as_str()).collect()
    }

    #[test]
    fn of_one_answering_and_two_silent_exactly_the_one_that_answered_is_shown() {
        let (probes, v) = swept(1000);
        let p = probes.lock().unwrap();
        assert_eq!(names(&shown(&v, &p, &Query::default(), 1000)), ["answers"]);
        assert_eq!(p.tally(&v), Tally { listed: 3, checked: 3, answering: 1 });
    }

    #[test]
    fn what_the_server_said_overrides_what_the_directory_said() {
        let (probes, v) = swept(1000);
        let p = probes.lock().unwrap();
        let q = Query { hz: Some(435_000_000), tunable: true, ..Query::default() };
        let found = shown(&v, &p, &q, 1000);
        assert_eq!(names(&found), ["answers"], "the listing says a fixed airband span");
        let t = &found[0].listing.entry.station.tuners[0];
        assert_eq!(t.center_hz, 145_800_000);
        assert_eq!(found[0].listing.entry.station.clients, 3);
        assert_eq!(v[0].entry.station.tuners[0].center_hz, 125_000_000);
    }

    #[test]
    fn this_receiver_then_its_network_then_answering_then_unchecked() {
        let v: Vec<Listing> = ["zed", "unchecked", "near", "ours"]
            .iter()
            .enumerate()
            .map(|(i, n)| listed(n, i as u16 + 1, vec![airband()]))
            .collect();
        let lan: SocketAddr = "192.168.1.9:5555".parse().unwrap();
        let home: SocketAddr = "127.0.0.1:5555".parse().unwrap();
        let mut p = Probes::default();
        p.insert(&v[0], Probed { at: 0, heard: Heard::Answered(Said::LISTED) });
        p.insert(&v[2], Probed { at: 0, heard: Heard::Answered(Said::at(Reached::Near(lan))) });
        p.insert(&v[3], Probed { at: 0, heard: Heard::Answered(Said::at(Reached::Ours(home))) });
        let found = shown(&v, &p, &Query::default(), 1000);
        assert_eq!(names(&found), ["ours", "near", "zed", "unchecked"]);
        let at: Vec<String> = found.iter().map(Found::addr).collect();
        assert_eq!(at, ["127.0.0.1:5555", "192.168.1.9:5555", "203.0.113.9:1", "203.0.113.9:2"]);
    }

    #[test]
    fn among_the_answering_a_free_slot_then_a_free_dial_comes_first() {
        let mut full = listed("a", 1, vec![hf()]);
        full.entry.station = Station { clients: 4, ..full.entry.station };
        let v = vec![full, listed("b", 2, vec![airband()]), listed("c", 3, vec![hf()])];
        let found = shown(&v, &Probes::default(), &Query::default(), 1000);
        assert_eq!(names(&found), ["c", "b", "a"]);
    }

    #[test]
    fn a_second_sweep_asks_again_only_what_has_gone_stale() {
        let (probes, v) = swept(1000);
        let asked = AtomicUsize::new(0);
        let count = |l: &Listing| {
            asked.fetch_add(1, Ordering::Relaxed);
            by_name(l)
        };
        assert_eq!(sweep(&probes, &v, 1000 + ANSWERED_FOR - 1, WORKERS, count), 0);
        assert_eq!(sweep(&probes, &v, 1000 + SILENT_FOR - 1, WORKERS, count), 1, "the live one");
        assert_eq!(sweep(&probes, &v, 1000 + SILENT_FOR, WORKERS, count), 2, "the silent two");
        assert_eq!(asked.load(Ordering::Relaxed), 3);
    }

    #[test]
    fn ten_servers_are_asked_at_a_time() {
        let v: Vec<Listing> = (0..30u16).map(|i| listed("x", 5000 + i, vec![airband()])).collect();
        let (now, peak) = (AtomicUsize::new(0), AtomicUsize::new(0));
        let started = std::time::Instant::now();
        let probes = Mutex::new(Probes::default());
        let asked = sweep(&probes, &v, 0, WORKERS, |_| {
            peak.fetch_max(now.fetch_add(1, Ordering::SeqCst) + 1, Ordering::SeqCst);
            std::thread::sleep(Duration::from_millis(100));
            now.fetch_sub(1, Ordering::SeqCst);
            Heard::Silent
        });
        assert_eq!((asked, peak.load(Ordering::SeqCst)), (30, 10));
        let took = started.elapsed();
        assert!(took >= Duration::from_millis(300), "floor, three rounds of ten: {took:?}");
        assert!(took < Duration::from_millis(900), "ceiling, not one at a time: {took:?}");
    }

    #[test]
    fn the_probes_are_kept_on_disk_and_a_torn_file_reads_as_none() {
        let dir = std::env::temp_dir().join(format!("sdrprobe-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("probed.json");
        let (probes, v) = swept(1000);
        probes.lock().unwrap().write(&path).unwrap();
        let back = Probes::read(&path);
        assert_eq!(back, *probes.lock().unwrap());
        assert_eq!(back.tally(&v), Tally { listed: 3, checked: 3, answering: 1 });
        std::fs::write(&path, b"{\"torn").unwrap();
        assert_eq!(Probes::read(&path), Probes::default());
        std::fs::remove_dir_all(&dir).unwrap();
    }

    #[test]
    fn a_server_gone_from_the_directory_is_forgotten_at_the_next_sweep() {
        let (probes, mut v) = swept(1000);
        v.retain(|l| l.entry.station.name != "mute");
        assert_eq!(sweep(&probes, &v, 1000, WORKERS, by_name), 0);
        assert_eq!(probes.lock().unwrap().len(), 2);
    }
}
