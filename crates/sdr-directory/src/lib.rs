pub mod airspy;
pub mod lister;
pub mod model;
pub mod portmap;
pub mod probe;

pub use model::{Accuracy, Dial, Entry, Hardware, Location, Protocol, Station, Tuner, Version};

use std::collections::HashMap;
use std::fmt;
use std::time::{Duration, SystemTime, UNIX_EPOCH};

pub const STALE_AFTER_SECS: u64 = 24 * 60 * 60;
pub const ANNOUNCE_EVERY_SECS: u64 = STALE_AFTER_SECS;

pub trait SdrDirectory: Sized + Send {
    type Config: Clone + PartialEq + fmt::Debug + Send + Sync + 'static;

    fn open(config: &Self::Config, wait: Duration) -> Result<Self, Error>;
    fn author(&self) -> Option<Author>;
    fn announce(&self, entry: &Entry) -> Result<Published, Error>;
    fn every(&self) -> Duration {
        Duration::from_secs(ANNOUNCE_EVERY_SECS)
    }
    fn withdraw(&self) -> Result<Published, Error>;
    fn list(&self, wait: Duration) -> Result<Vec<Listing>, Error>;
    fn list_near(&self, geohash: &str, wait: Duration) -> Result<Vec<Listing>, Error>;
    fn close(self);
}

#[derive(Debug, thiserror::Error)]
pub enum Error {
    #[error("{0}")]
    Unreachable(String),
    #[error("nobody took the listing: {}", refusals(.0))]
    Refused(Vec<(String, String)>),
    #[error("{0}")]
    Identity(String),
}

fn refusals(r: &[(String, String)]) -> String {
    match r.is_empty() {
        true => "no server answered".into(),
        false => r.iter().map(|(at, why)| format!("{at}: {why}")).collect::<Vec<_>>().join("; "),
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct Author(pub String);

impl fmt::Display for Author {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.0)
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Published {
    pub accepted: usize,
    pub refused: Vec<(String, String)>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Listing {
    pub author: Author,
    pub seen: u64,
    pub entry: Entry,
}

impl Listing {
    pub fn online(&self, now: u64) -> bool {
        now.saturating_sub(self.seen) < STALE_AFTER_SECS
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Query {
    pub hz: Option<u64>,
    pub tunable: bool,
    pub free: bool,
}

impl Query {
    pub fn keeps(&self, l: &Listing, now: u64) -> bool {
        let s = &l.entry.station;
        l.online(now)
            && s.protocol.speaks_with(&Version::OURS)
            && (!self.free || s.has_slot())
            && s.tuners
                .iter()
                .any(|t| self.hz.is_none_or(|hz| t.hears(hz)) && (!self.tunable || t.tunable()))
    }
}

pub fn newest_per_author(listings: impl Iterator<Item = Listing>, now: u64) -> Vec<Listing> {
    let mut by: HashMap<Author, Listing> = HashMap::new();
    for l in listings.filter(|l| l.online(now)) {
        match by.get(&l.author) {
            Some(held) if held.seen >= l.seen => {}
            _ => {
                by.insert(l.author.clone(), l);
            }
        }
    }
    let mut out: Vec<Listing> = by.into_values().collect();
    out.sort_by(|a, b| b.seen.cmp(&a.seen).then_with(|| a.entry.addr().cmp(&b.entry.addr())));
    out
}

pub fn distinct(listings: &mut Vec<Listing>) {
    let mut held = std::collections::HashSet::new();
    listings.retain(|l| held.insert(l.entry.addr()));
}

pub fn now() -> u64 {
    SystemTime::now().duration_since(UNIX_EPOCH).map_or(0, |d| d.as_secs())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::model::fixtures::{airband, entry, hf, station};

    fn listed(author: &str, seen: u64, entry: Entry) -> Listing {
        Listing { author: Author(author.into()), seen, entry }
    }

    #[test]
    fn a_query_keeps_servers_announced_recently_that_hear_the_frequency() {
        let now = 2_000 + STALE_AFTER_SECS - 1;
        let listed = [
            listed("a", 2_000, entry("a.example", vec![airband()])),
            listed("b", 2_000, entry("b.example", vec![airband(), hf()])),
            listed("d", 1, entry("d.example", vec![hf()])),
            listed(
                "e",
                2_000,
                Entry {
                    station: Station { clients: 4, ..station(vec![hf()]) },
                    ..entry("e.example", vec![])
                },
            ),
            listed(
                "f",
                2_000,
                Entry {
                    station: Station {
                        protocol: Protocol::IqStream(Version { major: 2, minor: 0 }),
                        ..station(vec![hf()])
                    },
                    ..entry("f.example", vec![])
                },
            ),
        ];
        let kept = |q: Query| -> Vec<&str> {
            listed.iter().filter(|l| q.keeps(l, now)).map(|l| l.entry.host.as_str()).collect()
        };
        assert_eq!(kept(Query::default()), ["a.example", "b.example", "e.example"]);
        assert_eq!(
            kept(Query { hz: Some(14_074_000), ..Query::default() }),
            ["b.example", "e.example"]
        );
        assert_eq!(kept(Query { tunable: true, free: true, ..Query::default() }), ["b.example"]);
        assert_eq!(
            kept(Query { hz: Some(125_000_000), tunable: true, ..Query::default() }),
            Vec::<&str>::new(),
            "the tuner hearing 125 MHz is not the tunable one"
        );
    }

    #[test]
    fn a_server_listed_twice_at_one_address_is_kept_once_where_it_first_appeared() {
        let mut v = vec![
            listed("a", 1, entry("x.example", vec![airband()])),
            listed("b", 1, entry("y.example", vec![hf()])),
            listed("c", 1, entry("x.example", vec![hf()])),
        ];
        distinct(&mut v);
        let kept: Vec<&str> = v.iter().map(|l| l.author.0.as_str()).collect();
        assert_eq!(kept, ["a", "b"]);
    }

    #[test]
    fn of_two_listings_by_one_author_the_later_is_kept_and_a_stale_one_is_dropped() {
        let now = 1_750_000_000;
        let l = |author, seen, host| listed(author, seen, entry(host, vec![airband()]));
        let kept = newest_per_author(
            [
                l("a", now - 500, "old.a"),
                l("a", now - 100, "new.a"),
                l("a", now - 300, "mid.a"),
                l("b", now - STALE_AFTER_SECS, "stale.b"),
            ]
            .into_iter(),
            now,
        );
        let hosts: Vec<&str> = kept.iter().map(|l| l.entry.host.as_str()).collect();
        assert_eq!(hosts, ["new.a"]);
    }
}
