use nostr_directory::{Config, NostrDirectory};
use parking_lot::Mutex;
use sdr_directory::probe::{Heard, Reached, Said};
use sdr_directory::{Author, Listing, SdrDirectory};
use std::net::{IpAddr, Ipv4Addr, Ipv6Addr, SocketAddr};
use std::sync::Arc;
use std::time::Duration;

const RELAY_WAIT: Duration = Duration::from_secs(10);

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Own {
    pub author: Author,
    pub local: SocketAddr,
}

impl Own {
    pub fn of(s: &crate::session::Session) -> Option<Own> {
        let author = Author::from(nostr_directory::identity(&s.iqstream_nsec)?.public_key());
        let (mut local, _) = s.iqstream()?;
        if local.ip().is_unspecified() {
            local.set_ip(match local.ip() {
                IpAddr::V4(_) => IpAddr::V4(Ipv4Addr::LOCALHOST),
                IpAddr::V6(_) => IpAddr::V6(Ipv6Addr::LOCALHOST),
            });
        }
        Some(Own { author, local })
    }
}

#[derive(Default)]
struct Finder {
    listings: Option<Arc<Vec<Listing>>>,
    attempted: bool,
    busy: bool,
    error: Option<String>,
}

fn finder() -> &'static Mutex<Finder> {
    static FINDER: std::sync::OnceLock<Mutex<Finder>> = std::sync::OnceLock::new();
    FINDER.get_or_init(Default::default)
}

pub fn listings() -> Option<Arc<Vec<Listing>>> {
    let (held, attempted) = {
        let f = finder().lock();
        (f.listings.clone(), f.attempted)
    };
    if held.is_none() && !attempted {
        refresh();
    }
    held
}

pub fn busy() -> bool {
    finder().lock().busy
}

pub fn failed() -> Option<String> {
    finder().lock().error.clone()
}

pub fn check() {
    if finder().lock().listings.is_none() {
        refresh();
    }
}

pub fn refresh() {
    {
        let mut f = finder().lock();
        if f.busy {
            return;
        }
        f.busy = true;
        f.attempted = true;
        f.error = None;
    }
    let started = std::thread::Builder::new().name("iqstream-find".into()).spawn(move || {
        let listed = read_directory();
        let mut f = finder().lock();
        match listed {
            Ok(l) => f.listings = Some(Arc::new(l)),
            Err(e) => f.error = Some(e),
        }
        f.busy = false;
    });
    if started.is_err() {
        finder().lock().busy = false;
    }
}

fn read_directory() -> Result<Vec<Listing>, String> {
    let dir = NostrDirectory::open(&Config::reader(&nostr_directory::RELAYS), RELAY_WAIT)
        .map_err(|e| e.to_string())?;
    let listed = dir.list(RELAY_WAIT).map_err(|e| e.to_string());
    dir.close();
    listed
}

pub fn home(listings: &[Listing], own: Option<&Own>) -> Option<String> {
    let own = own?;
    listings.iter().find(|l| l.author == own.author).map(|l| l.entry.host.clone())
}

pub fn heard(
    l: &Listing,
    own: Option<&Own>,
    home: Option<&str>,
    answers: impl Fn(&str) -> bool,
) -> Heard {
    let tries: Vec<(String, Reached)> = match own.filter(|o| o.author == l.author) {
        Some(o) => vec![(o.local.to_string(), Reached::Ours(o.local))],
        None if home == Some(l.entry.host.as_str()) => {
            l.entry.also.iter().map(|a| (a.to_string(), Reached::Near(*a))).collect()
        }
        None => vec![(l.entry.addr(), Reached::Listed)],
    };
    tries
        .into_iter()
        .find(|(at, _)| answers(at))
        .map_or(Heard::Silent, |(_, reached)| Heard::Answered(Said::at(reached)))
}

#[cfg(test)]
mod tests {
    use super::*;
    use nostr_directory::Keys;
    use sdr_directory::probe::{self, Probes, Tally};
    use sdr_directory::{Dial, Entry, Hardware, Query, Station, Tuner, Version};

    fn listing(name: &str, host: &str, center_hz: u64, dial: Dial) -> Listing {
        Listing {
            author: Author::from(Keys::generate().public_key()),
            seen: 1_000,
            entry: Entry {
                host: host.into(),
                port: 1234,
                data_port: None,
                also: Vec::new(),
                station: Station {
                    name: name.into(),
                    description: String::new(),
                    location: None,
                    protocol: sdr_directory::Protocol::IqStream(Version::OURS),
                    clients: 0,
                    max_clients: None,
                    session_limit_secs: None,
                    tuners: vec![Tuner {
                        id: 0,
                        name: "span".into(),
                        hardware: Hardware::HackRf,
                        antenna: String::new(),
                        center_hz,
                        sample_rate: 2_000_000,
                        dial,
                    }],
                },
            },
        }
    }

    fn sweep(
        listings: &[Listing],
        own: Option<&Own>,
        answers: impl Fn(&str) -> bool + Sync,
    ) -> Probes {
        let probes = std::sync::Mutex::new(Probes::default());
        let home = home(listings, own);
        let asked = probe::sweep(&probes, listings, 1_000, probe::WORKERS, |l| {
            heard(l, own, home.as_deref(), &answers)
        });
        assert_eq!(asked, listings.len(), "every listing asked once");
        probes.into_inner().unwrap()
    }

    fn names(listings: &[Listing], probes: &Probes) -> Vec<String> {
        probe::shown(listings, probes, &Query::default(), 2_000)
            .into_iter()
            .map(|f| f.listing.entry.station.name)
            .collect()
    }

    #[test]
    fn of_twenty_five_listed_stations_the_eleven_that_answer_are_shown() {
        let listings: Vec<Listing> = (0..25)
            .map(|i| listing(&format!("s{i}"), &format!("h{i}.example"), 1, Dial::Fixed))
            .collect();
        let probes = sweep(&listings, None, |addr| addr.starts_with("h1"));
        assert_eq!(probes.tally(&listings), Tally { listed: 25, checked: 25, answering: 11 });
        assert_eq!(names(&listings, &probes).len(), 11, "h1 and h10 to h19");
    }

    #[test]
    fn this_receivers_own_listing_is_asked_on_the_local_address_and_listed_first() {
        let mine = listing("mine", "83.71.105.199", 433_920_000, Dial::Fixed);
        let other = listing("other", "203.0.113.9", 433_920_000, Dial::Fixed);
        let local: SocketAddr = "127.0.0.1:1234".parse().unwrap();
        let own = Own { author: mine.author.clone(), local };
        let asked = Mutex::new(Vec::new());
        let all = [other.clone(), mine.clone()];
        let probes = sweep(&all, Some(&own), |addr| {
            asked.lock().push(addr.to_string());
            addr != "83.71.105.199:1234"
        });
        let mut asked = asked.into_inner();
        asked.sort();
        assert_eq!(asked, ["127.0.0.1:1234", "203.0.113.9:1234"], "never its own public address");
        assert_eq!(probes.heard(&other), Some(Heard::Answered(Said::LISTED)));
        assert_eq!(probes.heard(&mine), Some(Heard::Answered(Said::at(Reached::Ours(local)))));
        assert_eq!(names(&all, &probes), ["mine", "other"]);
        assert_eq!(probes.tally(&all), Tally { listed: 2, checked: 2, answering: 2 });
    }

    #[test]
    fn a_station_behind_the_same_router_is_asked_on_its_other_addresses() {
        let mine = listing("mine", "83.71.105.199", 1, Dial::Fixed);
        let radarpi = Listing {
            entry: Entry {
                also: vec!["10.9.9.9:1234".parse().unwrap(), "10.100.2.249:1234".parse().unwrap()],
                ..listing("radarpi", "83.71.105.199", 1, Dial::Fixed).entry
            },
            ..listing("radarpi", "83.71.105.199", 1, Dial::Fixed)
        };
        let elsewhere = Listing {
            entry: Entry {
                also: vec!["10.100.2.249:1234".parse().unwrap()],
                ..listing("elsewhere", "203.0.113.9", 1, Dial::Fixed).entry
            },
            ..listing("elsewhere", "203.0.113.9", 1, Dial::Fixed)
        };
        let own = Own { author: mine.author.clone(), local: "127.0.0.1:1234".parse().unwrap() };
        let asked = Mutex::new(Vec::new());
        let all = [mine.clone(), radarpi.clone(), elsewhere.clone()];
        let home = home(&all, Some(&own));
        let heard_all: Vec<Heard> = all
            .iter()
            .map(|l| {
                heard(l, Some(&own), home.as_deref(), |addr| {
                    asked.lock().push(addr.to_string());
                    addr != "10.9.9.9:1234"
                })
            })
            .collect();
        let mut asked = asked.into_inner();
        asked.sort();
        assert_eq!(
            asked,
            ["10.100.2.249:1234", "10.9.9.9:1234", "127.0.0.1:1234", "203.0.113.9:1234"],
            "a stranger's other addresses are never asked, however private they look"
        );
        let near: SocketAddr = "10.100.2.249:1234".parse().unwrap();
        assert_eq!(
            heard_all,
            [
                Heard::Answered(Said::at(Reached::Ours(own.local))),
                Heard::Answered(Said::at(Reached::Near(near))),
                Heard::Answered(Said::LISTED)
            ]
        );
        let alone = heard(&radarpi, None, None, |addr| addr == "83.71.105.199:1234");
        assert_eq!(alone, Heard::Answered(Said::LISTED), "without its own listing it cannot tell");
    }

    #[test]
    fn a_server_on_every_interface_is_reached_here_on_loopback() {
        let mut s = crate::session::Session { iqstream_on: true, ..Default::default() };
        assert_eq!(Own::of(&s), None, "no key yet");
        let author = Author::from(s.directory_keys().public_key());
        s.iqstream_addr = "0.0.0.0:1234".into();
        assert_eq!(Own::of(&s), Some(Own { author, local: "127.0.0.1:1234".parse().unwrap() }));
        s.iqstream_addr = "[::]:1234".into();
        assert_eq!(Own::of(&s).map(|o| o.local), Some("[::1]:1234".parse().unwrap()));
        s.iqstream_addr = "10.100.2.34:1234".into();
        assert_eq!(Own::of(&s).map(|o| o.local), Some("10.100.2.34:1234".parse().unwrap()));
    }
}
