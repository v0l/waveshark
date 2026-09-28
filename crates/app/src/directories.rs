use crate::stations::Own;
use sdr_directory::probe::{self, Found, Heard, Probes, Said, Tally};
use sdr_directory::{Dial, Listing, Query};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, LazyLock, Mutex, MutexGuard};
use std::time::Duration;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Directory {
    IqStream,
    SpyServer,
    KiwiSdr,
}

struct Swept {
    probes: Mutex<Probes>,
    sweeping: AtomicBool,
}

impl Swept {
    fn read(d: Directory) -> Swept {
        let probes = crate::data::cache().map(|c| Probes::read(&d.probes_path(c)));
        Swept { probes: Mutex::new(probes.unwrap_or_default()), sweeping: AtomicBool::new(false) }
    }
}

static IQSTREAM: LazyLock<Swept> = LazyLock::new(|| Swept::read(Directory::IqStream));
static SPYSERVER: LazyLock<Swept> = LazyLock::new(|| Swept::read(Directory::SpyServer));
static KIWISDR: LazyLock<Swept> = LazyLock::new(|| Swept::read(Directory::KiwiSdr));

impl Directory {
    pub const ALL: [Directory; 3] = [Directory::IqStream, Directory::SpyServer, Directory::KiwiSdr];

    pub fn proto(self) -> remote::Proto {
        match self {
            Directory::IqStream => remote::Proto::IqStream,
            Directory::SpyServer => remote::Proto::SpyServer,
            Directory::KiwiSdr => remote::Proto::KiwiSdr,
        }
    }

    pub fn of(proto: remote::Proto) -> Option<Directory> {
        Directory::ALL.into_iter().find(|d| d.proto() == proto)
    }

    pub fn title(self) -> &'static str {
        match self {
            Directory::IqStream => "Public IQStream servers",
            Directory::SpyServer => "Public SpyServers",
            Directory::KiwiSdr => "Public KiwiSDRs",
        }
    }

    pub fn place(self) -> &'static str {
        match self {
            Directory::IqStream => "on nostr",
            Directory::SpyServer => "in the Airspy directory",
            Directory::KiwiSdr => "on kiwisdr.com",
        }
    }

    pub fn control_help(self) -> &'static str {
        match self {
            Directory::IqStream => {
                "A server offering its dial lets a subscriber retune it anywhere its tuner \
                 reaches. One that does not is fixed to the span it is serving."
            }
            Directory::SpyServer => {
                "A server granting control lets the dial go anywhere in its range. One that \
                 does not lets it move only inside the span it is already on."
            }
            Directory::KiwiSdr => {
                "Every KiwiSDR gives each listener a dial of its own across the bands it \
                 covers, so this keeps them all."
            }
        }
    }

    fn key(self) -> &'static str {
        match self {
            Directory::IqStream => "iqstream",
            Directory::SpyServer => "spyservers",
            Directory::KiwiSdr => "kiwisdrs",
        }
    }

    fn probes_path(self, cache: &datasets::Cache) -> std::path::PathBuf {
        cache.dir().join(format!("{}-probed.json", self.key()))
    }

    fn swept(self) -> &'static Swept {
        match self {
            Directory::IqStream => &IQSTREAM,
            Directory::SpyServer => &SPYSERVER,
            Directory::KiwiSdr => &KIWISDR,
        }
    }

    fn which(self) -> Option<crate::data::Which> {
        match self {
            Directory::IqStream => None,
            Directory::SpyServer => Some(crate::data::Which::SpyServers),
            Directory::KiwiSdr => Some(crate::data::Which::KiwiSdrs),
        }
    }

    pub fn credit(self) -> Option<crate::data::Credit> {
        self.which().map(|w| w.credit())
    }

    pub fn listings(self) -> Option<Arc<Vec<Listing>>> {
        match self {
            Directory::IqStream => crate::stations::listings(),
            Directory::SpyServer => crate::data::spyservers(),
            Directory::KiwiSdr => crate::data::kiwisdrs(),
        }
    }

    pub fn open(self) {
        match self.which() {
            Some(w) => crate::data::check(w),
            None => crate::stations::check(),
        }
    }

    pub fn refresh(self) {
        match self.which() {
            Some(w) => crate::data::refresh(w),
            None => crate::stations::refresh(),
        }
    }

    pub fn busy(self) -> bool {
        match self.which() {
            Some(w) => crate::data::busy(w),
            None => crate::stations::busy(),
        }
    }

    pub fn failed(self) -> Option<String> {
        match self.which() {
            Some(w) => crate::data::failed(w),
            None => crate::stations::failed(),
        }
    }

    pub fn probes(self) -> MutexGuard<'static, Probes> {
        self.swept().probes.lock().unwrap_or_else(|e| e.into_inner())
    }

    pub fn sweeping(self) -> bool {
        self.swept().sweeping.load(Ordering::Acquire)
    }

    pub fn tally(self) -> Option<Tally> {
        self.listings().map(|l| self.probes().tally(&l))
    }

    pub fn shown(self, query: &Query) -> Vec<Found> {
        let Some(listings) = self.listings() else { return Vec::new() };
        probe::shown(&listings, &self.probes(), query, sdr_directory::now())
    }

    pub fn sweep(self, own: Option<Own>) {
        let Some(listings) = self.listings() else { return };
        let now = sdr_directory::now();
        if self.sweeping() || self.probes().due(&listings, now).is_empty() {
            return;
        }
        let swept = self.swept();
        if swept.sweeping.swap(true, Ordering::AcqRel) {
            return;
        }
        let name = format!("{}-probe", self.key());
        let started = std::thread::Builder::new().name(name).spawn(move || {
            let home = crate::stations::home(&listings, own.as_ref());
            let asked = probe::sweep(&swept.probes, &listings, now, probe::WORKERS, |l| {
                let heard = self.ask(l, own.as_ref(), home.as_deref());
                crate::data::repaint();
                heard
            });
            let t = self.probes().tally(&listings);
            tracing::info!(asked, answering = t.answering, "{} directory probed", self.key());
            if let Some(c) = crate::data::cache()
                && let Err(e) = self.probes().write(&self.probes_path(c))
            {
                tracing::warn!("{} probes not saved: {e}", self.key());
            }
            swept.sweeping.store(false, Ordering::Release);
            crate::data::repaint();
        });
        if started.is_err() {
            swept.sweeping.store(false, Ordering::Release);
        }
    }

    fn ask(self, l: &Listing, own: Option<&Own>, home: Option<&str>) -> Heard {
        match self {
            Directory::IqStream => crate::stations::heard(l, own, home, |addr| {
                remote::iqstream::probe_all(addr).is_ok()
            }),
            Directory::SpyServer => spyserver(&l.entry.addr(), remote::CONNECT_TIMEOUT),
            Directory::KiwiSdr => kiwisdr(&l.entry.addr()),
        }
    }
}

fn spyserver(addr: &str, within: Duration) -> Heard {
    match remote::spyserver::probe_within(addr, within) {
        Ok(p) => Heard::Answered(Said {
            center_hz: p.center.map(|c| c.0),
            dial: Some(match (p.tunable, p.tune_range) {
                (false, _) => Dial::Fixed,
                (true, Some(r)) => {
                    Dial::Tunable { min_hz: Some(r.start().0), max_hz: Some(r.end().0) }
                }
                (true, None) => Dial::Tunable { min_hz: None, max_hz: None },
            }),
            ..Said::LISTED
        }),
        Err(_) => Heard::Silent,
    }
}

fn kiwisdr(addr: &str) -> Heard {
    match remote::kiwisdr::fetch_status(addr) {
        Ok(s) if !s.offline => Heard::Answered(Said { clients: Some(s.users), ..Said::LISTED }),
        Ok(_) | Err(_) => Heard::Silent,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use sdr_directory::{Author, Entry, Hardware, Protocol, Station, Tuner};
    use std::io::{Read, Write};
    use std::net::TcpListener;
    use std::sync::atomic::AtomicUsize;
    use std::time::Instant;
    fn spyserver_message(kind: u32, body: &[u32]) -> Vec<u8> {
        [(2 << 24) | 1921, kind, 0, 0, (body.len() * 4) as u32]
            .iter()
            .chain(body)
            .flat_map(|w| w.to_le_bytes())
            .collect()
    }

    fn fake_spyserver(answers: bool) -> (String, Arc<AtomicUsize>) {
        let l = TcpListener::bind("127.0.0.1:0").unwrap();
        let addr = l.local_addr().unwrap().to_string();
        let accepted = Arc::new(AtomicUsize::new(0));
        let counted = accepted.clone();
        std::thread::spawn(move || {
            let mut held = Vec::new();
            for mut sock in l.incoming().flatten() {
                counted.fetch_add(1, Ordering::SeqCst);
                if answers {
                    let mut hello = [0u8; 8 + 13];
                    let _ = sock.read_exact(&mut hello);
                    let info = [
                        1,
                        7,
                        3_000_000,
                        2_400_000,
                        10,
                        0,
                        21,
                        24_000_000,
                        1_800_000_000,
                        12,
                        0,
                        0,
                    ];
                    let sync = [1, 0, 145_800_000, 145_800_000, 0, 144_600_000, 147_000_000, 0, 0];
                    let _ = sock.write_all(&spyserver_message(0, &info));
                    let _ = sock.write_all(&spyserver_message(1, &sync));
                }
                held.push(sock);
            }
        });
        (addr, accepted)
    }

    fn listed_at(name: &str, addr: &str) -> Listing {
        let (host, port) = addr.rsplit_once(':').unwrap();
        let entry = Entry {
            host: host.into(),
            port: port.parse().unwrap(),
            data_port: None,
            also: Vec::new(),
            station: Station {
                name: name.into(),
                description: String::new(),
                location: None,
                protocol: Protocol::SpyServer,
                clients: 0,
                max_clients: Some(5),
                session_limit_secs: None,
                tuners: vec![Tuner {
                    id: 0,
                    name: String::new(),
                    hardware: Hardware::Airspy,
                    antenna: String::new(),
                    center_hz: 93_600_000,
                    sample_rate: 2_000_000,
                    dial: Dial::Fixed,
                }],
            },
        };
        Listing { author: Author(entry.addr()), seen: 1_000, entry }
    }

    #[test]
    fn of_three_listed_servers_only_the_one_that_answers_is_kept_and_asked_once() {
        let (answering, answered) = fake_spyserver(true);
        let (silent, held) = fake_spyserver(false);
        let refusing = TcpListener::bind("127.0.0.1:0").unwrap().local_addr().unwrap().to_string();
        let servers = [
            listed_at("answers", &answering),
            listed_at("refuses", &refusing),
            listed_at("says nothing", &silent),
        ];
        let probes = Mutex::new(Probes::default());
        let within = Duration::from_millis(400);
        let started = Instant::now();
        let ask = |l: &Listing| spyserver(&l.entry.addr(), within);
        assert_eq!(probe::sweep(&probes, &servers, 1_000, 10, ask), 3);
        assert!(started.elapsed() < within * 2, "ceiling: {:?}", started.elapsed());
        let p = probes.lock().unwrap();
        let q = Query { tunable: true, ..Query::default() };
        let kept = probe::shown(&servers, &p, &q, 2_000);
        assert_eq!(kept.len(), 1, "the directory says none of them grants control");
        assert_eq!(kept[0].listing.entry.station.name, "answers");
        let t = &kept[0].listing.entry.station.tuners[0];
        assert_eq!(t.center_hz, 145_800_000);
        assert_eq!(
            t.dial,
            Dial::Tunable { min_hz: Some(24_000_000), max_hz: Some(1_800_000_000) },
            "the server grants what the directory denied"
        );
        drop(p);
        assert_eq!(probe::sweep(&probes, &servers, 1_060, 10, ask), 0);
        assert_eq!(
            (answered.load(Ordering::SeqCst), held.load(Ordering::SeqCst)),
            (1, 1),
            "the second sweep read the cache and connected to nothing"
        );
    }

    #[test]
    fn a_kiwisdr_that_answers_its_status_page_says_how_many_are_listening() {
        let l = TcpListener::bind("127.0.0.1:0").unwrap();
        let addr = l.local_addr().unwrap().to_string();
        std::thread::spawn(move || {
            for (i, sock) in l.incoming().flatten().enumerate() {
                let mut sock = sock;
                let mut req = [0u8; 512];
                let _ = sock.read(&mut req);
                let offline = if i == 0 { "no" } else { "yes" };
                let _ = write!(
                    sock,
                    "HTTP/1.1 200 OK\r\nContent-Type: text/plain\r\n\r\n\
                     status=active\noffline={offline}\nusers=3\nusers_max=8\n"
                );
            }
        });
        assert_eq!(kiwisdr(&addr), Heard::Answered(Said { clients: Some(3), ..Said::LISTED }));
        assert_eq!(kiwisdr(&addr), Heard::Silent, "an offline receiver is not offered");
        let refusing = TcpListener::bind("127.0.0.1:0").unwrap().local_addr().unwrap().to_string();
        assert_eq!(kiwisdr(&refusing), Heard::Silent);
    }

    #[test]
    fn every_directory_is_found_again_from_its_protocol() {
        for d in Directory::ALL {
            assert_eq!(Directory::of(d.proto()), Some(d));
        }
        assert_eq!(Directory::of(remote::Proto::RtlTcp), None);
    }
}
