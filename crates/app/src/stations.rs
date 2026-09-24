use iqdirectory::{Directory, Listing, PublicKey, Query};
use parking_lot::Mutex;
use std::net::{IpAddr, Ipv4Addr, Ipv6Addr, SocketAddr};
use std::time::Duration;

const RELAY_WAIT: Duration = Duration::from_secs(10);
const PROBERS: usize = 10;

#[derive(Clone, Debug, PartialEq)]
pub enum Heard {
    Unchecked,
    Ours(SocketAddr),
    Near(SocketAddr),
    Answered,
    Silent,
}

impl Heard {
    pub fn answered(&self) -> bool {
        match self {
            Heard::Ours(_) | Heard::Near(_) | Heard::Answered => true,
            Heard::Unchecked | Heard::Silent => false,
        }
    }

    fn rank(&self) -> u8 {
        match self {
            Heard::Ours(_) => 0,
            Heard::Near(_) => 1,
            Heard::Answered => 2,
            Heard::Unchecked => 3,
            Heard::Silent => 4,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Own {
    pub author: PublicKey,
    pub local: SocketAddr,
}

impl Own {
    pub fn of(s: &crate::session::Session) -> Option<Own> {
        let author = iqdirectory::identity(&s.iqstream_nsec)?.public_key();
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

#[derive(Clone, Debug)]
pub struct Found {
    pub listing: Listing,
    pub heard: Heard,
}

#[derive(Default)]
struct Finder {
    found: Option<Vec<Found>>,
    busy: bool,
    error: Option<String>,
}

fn finder() -> &'static Mutex<Finder> {
    static FINDER: std::sync::OnceLock<Mutex<Finder>> = std::sync::OnceLock::new();
    FINDER.get_or_init(Default::default)
}

pub fn found() -> Option<Vec<Found>> {
    finder().lock().found.clone()
}

pub fn busy() -> bool {
    finder().lock().busy
}

pub fn failed() -> Option<String> {
    finder().lock().error.clone()
}

pub fn check(own: Option<Own>) {
    if finder().lock().found.is_none() {
        refresh(own);
    }
}

pub fn refresh(own: Option<Own>) {
    {
        let mut f = finder().lock();
        if f.busy {
            return;
        }
        f.busy = true;
        f.error = None;
    }
    let started = std::thread::Builder::new().name("iqstream-find".into()).spawn(move || {
        let listed = read_directory();
        let listings = match listed {
            Ok(l) => l,
            Err(e) => {
                let mut f = finder().lock();
                (f.error, f.busy) = (Some(e), false);
                return;
            }
        };
        finder().lock().found = Some(
            listings
                .iter()
                .map(|l| Found { listing: l.clone(), heard: Heard::Unchecked })
                .collect(),
        );
        let heard = probe(&listings, own, |addr| remote::iqstream::probe_all(addr).is_ok());
        let mut f = finder().lock();
        if let Some(found) = f.found.as_mut() {
            for (fd, heard) in found.iter_mut().zip(heard) {
                fd.heard = heard;
            }
        }
        f.busy = false;
    });
    if started.is_err() {
        finder().lock().busy = false;
    }
}

fn read_directory() -> Result<Vec<Listing>, String> {
    let rt = tokio::runtime::Builder::new_current_thread()
        .enable_all()
        .build()
        .map_err(|e| format!("tokio runtime: {e}"))?;
    rt.block_on(async {
        let dir = Directory::connect(&iqdirectory::RELAYS, RELAY_WAIT)
            .await
            .map_err(|e| e.to_string())?;
        let listed = dir.list(RELAY_WAIT).await.map_err(|e| e.to_string());
        dir.shutdown().await;
        listed
    })
}

fn probe(
    listings: &[Listing],
    own: Option<Own>,
    answers: impl Fn(&str) -> bool + Sync,
) -> Vec<Heard> {
    let next = std::sync::atomic::AtomicUsize::new(0);
    let heard: Vec<Mutex<Heard>> = listings.iter().map(|_| Mutex::new(Heard::Silent)).collect();
    let home = own.and_then(|o| listings.iter().find(|l| l.author == o.author));
    let home = home.map(|l| l.entry.host.clone());
    std::thread::scope(|scope| {
        for _ in 0..PROBERS.min(listings.len()) {
            scope.spawn(|| {
                loop {
                    let i = next.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
                    let Some(l) = listings.get(i) else { break };
                    let tries: Vec<(String, Heard)> = match own.filter(|o| o.author == l.author) {
                        Some(o) => vec![(o.local.to_string(), Heard::Ours(o.local))],
                        None if home.as_ref() == Some(&l.entry.host) => {
                            l.entry.also.iter().map(|a| (a.to_string(), Heard::Near(*a))).collect()
                        }
                        None => vec![(l.entry.addr(), Heard::Answered)],
                    };
                    if let Some((_, found)) = tries.into_iter().find(|(at, _)| answers(at)) {
                        *heard[i].lock() = found;
                    }
                }
            });
        }
    });
    heard.into_iter().map(|h| h.into_inner()).collect()
}

pub fn shown(found: &[Found], query: &Query, now: u64) -> Vec<Found> {
    let mut out: Vec<Found> = found
        .iter()
        .filter(|f| f.heard != Heard::Silent && query.keeps(&f.listing, now))
        .cloned()
        .collect();
    out.sort_by_key(|f| (f.heard.rank(), f.listing.entry.station.name.to_lowercase()));
    out
}

pub fn tally(found: &[Found]) -> (usize, usize, usize) {
    let checked = found.iter().filter(|f| f.heard != Heard::Unchecked).count();
    let answering = found.iter().filter(|f| f.heard.answered()).count();
    (found.len(), checked, answering)
}

#[cfg(test)]
mod tests {
    use super::*;
    use iqdirectory::model::{Dial, Hardware, Station, Tuner, Version};
    use iqdirectory::{Entry, Keys};

    fn listing(name: &str, host: &str, center_hz: u64, dial: Dial) -> Listing {
        Listing {
            author: Keys::generate().public_key(),
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
                    version: Version::OURS,
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

    #[test]
    fn stations_that_answered_come_first_and_silent_ones_are_hidden() {
        let fixed = Dial::Fixed;
        let found = vec![
            Found {
                listing: listing("zulu", "z.example", 433_920_000, fixed),
                heard: Heard::Answered,
            },
            Found {
                listing: listing("alpha", "a.example", 433_920_000, fixed),
                heard: Heard::Unchecked,
            },
            Found {
                listing: listing("mike", "m.example", 433_920_000, fixed),
                heard: Heard::Silent,
            },
            Found {
                listing: listing("bravo", "b.example", 433_920_000, fixed),
                heard: Heard::Answered,
            },
        ];
        let names = |q: Query| -> Vec<String> {
            shown(&found, &q, 2_000).into_iter().map(|f| f.listing.entry.station.name).collect()
        };
        assert_eq!(names(Query::default()), ["bravo", "zulu", "alpha"]);
        assert_eq!(
            names(Query { hz: Some(145_800_000), ..Query::default() }),
            Vec::<String>::new()
        );
        assert_eq!(tally(&found), (4, 3, 2));
    }

    #[test]
    fn every_listed_station_is_asked_once_and_ten_at_a_time() {
        let listings: Vec<Listing> = (0..25)
            .map(|i| listing(&format!("s{i}"), &format!("h{i}.example"), 1, Dial::Fixed))
            .collect();
        let (now, peak, asked) = (
            std::sync::atomic::AtomicUsize::new(0),
            std::sync::atomic::AtomicUsize::new(0),
            std::sync::atomic::AtomicUsize::new(0),
        );
        use std::sync::atomic::Ordering::SeqCst;
        let heard = probe(&listings, None, |addr| {
            peak.fetch_max(now.fetch_add(1, SeqCst) + 1, SeqCst);
            asked.fetch_add(1, SeqCst);
            std::thread::sleep(Duration::from_millis(20));
            now.fetch_sub(1, SeqCst);
            addr.starts_with("h1")
        });
        assert_eq!((asked.load(SeqCst), peak.load(SeqCst)), (25, PROBERS));
        assert_eq!(heard.iter().filter(|h| h.answered()).count(), 11, "h1 and h10 to h19");
    }

    #[test]
    fn this_receivers_own_listing_is_asked_on_the_local_address_and_listed_first() {
        let mine = listing("mine", "83.71.105.199", 433_920_000, Dial::Fixed);
        let other = listing("other", "203.0.113.9", 433_920_000, Dial::Fixed);
        let local: SocketAddr = "127.0.0.1:1234".parse().unwrap();
        let own = Own { author: mine.author, local };
        let asked = Mutex::new(Vec::new());
        let heard = probe(&[other.clone(), mine.clone()], Some(own), |addr| {
            asked.lock().push(addr.to_string());
            addr != "83.71.105.199:1234"
        });
        let mut asked = asked.into_inner();
        asked.sort();
        assert_eq!(asked, ["127.0.0.1:1234", "203.0.113.9:1234"], "never its own public address");
        assert_eq!(heard, [Heard::Answered, Heard::Ours(local)]);
        let found: Vec<Found> = [other, mine]
            .into_iter()
            .zip(heard)
            .map(|(listing, heard)| Found { listing, heard })
            .collect();
        let names: Vec<String> = shown(&found, &Query::default(), 2_000)
            .into_iter()
            .map(|f| f.listing.entry.station.name)
            .collect();
        assert_eq!(names, ["mine", "other"]);
        assert_eq!(tally(&found), (2, 2, 2));
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
        let own = Own { author: mine.author, local: "127.0.0.1:1234".parse().unwrap() };
        let asked = Mutex::new(Vec::new());
        let all = [mine.clone(), radarpi.clone(), elsewhere.clone()];
        let heard = probe(&all, Some(own), |addr| {
            asked.lock().push(addr.to_string());
            addr != "10.9.9.9:1234"
        });
        let mut asked = asked.into_inner();
        asked.sort();
        assert_eq!(
            asked,
            ["10.100.2.249:1234", "10.9.9.9:1234", "127.0.0.1:1234", "203.0.113.9:1234"],
            "a stranger's other addresses are never asked, however private they look"
        );
        assert_eq!(
            heard,
            [
                Heard::Ours(own.local),
                Heard::Near("10.100.2.249:1234".parse().unwrap()),
                Heard::Answered
            ]
        );
        let alone = probe(&[radarpi], None, |addr| addr == "83.71.105.199:1234");
        assert_eq!(alone, [Heard::Answered], "without a listing of its own it cannot tell");
    }

    #[test]
    fn a_server_on_every_interface_is_reached_here_on_loopback() {
        let mut s = crate::session::Session { iqstream_on: true, ..Default::default() };
        assert_eq!(Own::of(&s), None, "no key yet");
        let author = s.directory_keys().public_key();
        s.iqstream_addr = "0.0.0.0:1234".into();
        assert_eq!(Own::of(&s), Some(Own { author, local: "127.0.0.1:1234".parse().unwrap() }));
        s.iqstream_addr = "[::]:1234".into();
        assert_eq!(Own::of(&s).map(|o| o.local), Some("[::1]:1234".parse().unwrap()));
        s.iqstream_addr = "10.100.2.34:1234".into();
        assert_eq!(Own::of(&s).map(|o| o.local), Some("10.100.2.34:1234".parse().unwrap()));
    }
}
