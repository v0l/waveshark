//! The cached reference datasets, and the threads that keep them current.
//!
//! `datasets` knows how to fetch and parse; this decides when, and holds what
//! came back where a frame can read it without waiting. Nothing here blocks
//! the UI: a dataset is absent until it is not, and every view that uses one
//! has to draw without it.
//!
//! Airports load at startup because the map wants them within a second of
//! opening. The radioid registries do not, because the DMR user dump is 85 MB
//! and nothing asks it a question until a digital voice frame arrives, so
//! they load on first use and stay loaded.

use datasets::airports::Airport;
use datasets::cells::{Cells, Operators};
use datasets::gateways::{Gateway, HostFile};
use datasets::radioid::{Repeater, Users};
use datasets::sigid;
use datasets::tle;
use datasets::{Cache, When};
use parking_lot::RwLock;
use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::sync::{LazyLock, OnceLock};
use std::time::{SystemTime, UNIX_EPOCH};

/// The zoom at which airports first appear on the map. Below this the view is
/// wide enough that every marker would be a blob under a handful of aircraft,
/// and the range rings already say where the interesting things are.
pub const SHOW_ZOOM: f64 = 9.0;

/// The airports the map draws.
///
/// Handed out as `&'static [Airport]` because a tooltip borrows an airport
/// across a frame and the alternative is cloning the row on every hover. The
/// snapshot is leaked rather than freed: it is replaced only when a refresh
/// finds a new file, and a reader holding the old one has no way to say when
/// it is done with it.
static AIRPORTS: RwLock<&'static [Airport]> = RwLock::new(&[]);

pub fn airports() -> &'static [Airport] {
    *AIRPORTS.read()
}

static USERS: RwLock<Option<Arc<Users>>> = RwLock::new(None);
static NXDN: RwLock<Option<Arc<Users>>> = RwLock::new(None);
static REPEATERS: RwLock<Option<Arc<Vec<Repeater>>>> = RwLock::new(None);
/// One slot per host file, in [`datasets::gateways::HOST_FILES`] order. The
/// networks are published separately, refreshed separately and fail
/// separately, so holding them as one list meant one slow publisher held up
/// the other four and one failure was reported against all of them.
static GATEWAYS: LazyLock<Vec<GatewaySlot>> =
    LazyLock::new(|| datasets::gateways::HOST_FILES.iter().map(|_| RwLock::new(None)).collect());

type GatewaySlot = RwLock<Option<Arc<Vec<Gateway>>>>;
/// One slot per CelesTrak group, in [`datasets::tle::GROUPS`] order, for the
/// same reason the gateways have one each.
static SATS: LazyLock<Vec<SatSlot>> =
    LazyLock::new(|| datasets::tle::GROUPS.iter().map(|_| RwLock::new(None)).collect());

type SatSlot = RwLock<Option<Arc<datasets::tle::Sats>>>;
static TRANSMITTERS: RwLock<Option<Arc<datasets::satnogs::Transmitters>>> = RwLock::new(None);
static OPERATORS: RwLock<Option<Arc<Operators>>> = RwLock::new(None);
static CELLS: RwLock<Option<Arc<Cells>>> = RwLock::new(None);
/// The two halves of the wiki, held apart because they are two files from
/// two publishers refreshed on their own, and joined on read.
static ARTEMIS: RwLock<Option<Arc<Vec<sigid::Signal>>>> = RwLock::new(None);
static UNID: RwLock<Option<Arc<Vec<sigid::Signal>>>> = RwLock::new(None);
static SIGID: RwLock<Option<Arc<sigid::Db>>> = RwLock::new(None);

/// The DMR ID registry: what the number in a DMR frame belongs to.
///
/// Asking starts the load and returns nothing; the answer is there a few
/// seconds later. Nothing decodes DMR yet, so this and the two below have no
/// caller in the tree: they are the half of the lookup that does not depend
/// on the decoder.
#[allow(dead_code)]
pub fn dmr_users() -> Option<Arc<Users>> {
    on_demand(Which::DmrIds, &USERS)
}

/// The NXDN ID registry, the same question for NXDN.
#[allow(dead_code)]
pub fn nxdn_users() -> Option<Arc<Users>> {
    on_demand(Which::NxdnIds, &NXDN)
}

/// Registered DMR repeaters, with their output frequency and colour code.
#[allow(dead_code)]
pub fn dmr_repeaters() -> Option<Arc<Vec<Repeater>>> {
    on_demand(Which::Repeaters, &REPEATERS)
}

/// Where one digital voice network can be reached: an address, a port, and
/// the channels within it that carry the mode spoken there.
#[allow(dead_code)]
pub fn gateways_of(h: &'static HostFile) -> Option<Arc<Vec<Gateway>>> {
    let slot = &GATEWAYS[host_file_index(h)];
    if let Some(v) = slot.read().clone() {
        return Some(v);
    }
    if !Which::Gateway(h).attempted() {
        load(Which::Gateway(h), When::IfDue);
    }
    None
}

/// Every gateway of every network whose file has landed, for a view that
/// does not care which publisher a row came from.
#[allow(dead_code)]
pub fn gateways() -> Vec<Gateway> {
    datasets::gateways::HOST_FILES
        .iter()
        .copied()
        .filter_map(gateways_of)
        .flat_map(|g| g.iter().cloned().collect::<Vec<_>>())
        .collect()
}

fn host_file_index(h: &'static HostFile) -> usize {
    datasets::gateways::HOST_FILES.iter().position(|o| *o == h).unwrap_or(0)
}

/// The orbital elements of one group, downloading them if they are not held.
///
/// Asking is what fetches: nothing propagates orbits yet, so the file is
/// downloaded when something wants it rather than at every start.
#[allow(dead_code)]
pub fn satellites(g: &'static datasets::tle::Group) -> Option<Arc<datasets::tle::Sats>> {
    let slot = &SATS[group_index(g)];
    if let Some(v) = slot.read().clone() {
        return Some(v);
    }
    if !Which::Satellites(g).attempted() {
        load(Which::Satellites(g), When::IfDue);
    }
    None
}

/// What the satellites transmit on, downloading the table if it is not held.
pub fn transmitters() -> Option<Arc<datasets::satnogs::Transmitters>> {
    on_demand(Which::Transmitters, &TRANSMITTERS)
}

fn group_index(g: &'static datasets::tle::Group) -> usize {
    datasets::tle::GROUPS.iter().position(|o| *o == g).unwrap_or(0)
}

/// What an MCC and MNC off a GSM beacon belong to.
#[allow(dead_code)]
pub fn cell_operators() -> Option<Arc<Operators>> {
    on_demand(Which::CellOperators, &OPERATORS)
}

/// A credential one dataset needs, described well enough for its row to ask
/// for it: what to call the field, the shape of an answer, and why.
#[derive(Clone, Copy)]
pub struct Key {
    pub label: &'static str,
    /// An example, as placeholder text. Never a real one.
    pub hint: &'static str,
    pub help: &'static str,
    /// Whether what is typed is hidden as it is typed. A password is; a
    /// user name is not, and masking it only stops an operator seeing which
    /// account a refusal belongs to.
    pub secret: bool,
}

/// A name to show and a page to open, for the credit drawn wherever data
/// somebody else published is used. Not only a dataset's: the map tiles are
/// somebody's too, and are credited the same way.
#[derive(Clone, Copy, PartialEq, Eq)]
pub struct Credit {
    /// The name the publisher asks to be called by.
    pub name: &'static str,
    /// The licence in brief, or who the contributors are where there is no
    /// licence to name.
    pub licence: &'static str,
    /// A page a person can read, never a fetch URL: the cell export's
    /// carries the operator's token.
    pub url: &'static str,
}

/// Where the cells of the configured country have been heard, when a token
/// has been given and the export downloaded.
#[allow(dead_code)]
pub fn cell_towers() -> Option<Arc<Cells>> {
    if cell_mcc().is_none() || opencellid_token().is_empty() {
        return None;
    }
    on_demand(Which::CellTowers, &CELLS)
}

/// The OpenCelliD download token, which is the operator's own: the export
/// URL carries it, and without one that dataset cannot be fetched at all.
static TOKEN: RwLock<String> = RwLock::new(String::new());
/// ISO 3166-1 country, which picks the country export to fetch. The world
/// file is hundreds of megabytes of cells on other continents.
static COUNTRY: RwLock<String> = RwLock::new(String::new());

pub fn opencellid_token() -> String {
    TOKEN.read().clone()
}

pub fn set_opencellid_token(token: &str) {
    *TOKEN.write() = token.trim().to_string();
}

pub fn set_country(iso: &str) {
    *COUNTRY.write() = iso.trim().to_ascii_lowercase();
}

/// The configured country, as an ISO 3166-1 code.
pub fn country() -> String {
    COUNTRY.read().clone()
}

/// The MCC the tower export is fetched for: the country's, read out of the
/// operator table that is already cached for naming networks.
pub fn cell_mcc() -> Option<u16> {
    let iso = COUNTRY.read().clone();
    if iso.is_empty() {
        return None;
    }
    OPERATORS.read().as_ref()?.mcc_for_country(&iso)
}

/// The signal identification wiki, both halves, for the inspector's guess
/// at what an unknown burst is.
pub fn sigid() -> Option<Arc<sigid::Db>> {
    let a = on_demand(Which::Artemis, &ARTEMIS);
    let u = on_demand(Which::SigIdUnid, &UNID);
    if let Some(db) = SIGID.read().clone() {
        return Some(db);
    }
    // Whichever half has landed is worth answering from. The join is
    // redone when the other arrives, or either is refreshed.
    let a = a?;
    let mut signals = (*a).clone();
    if let Some(u) = u {
        signals.extend(u.iter().cloned());
    }
    let db = Arc::new(sigid::Db { signals });
    *SIGID.write() = Some(db.clone());
    Some(db)
}

fn on_demand<T>(which: Which, held: &'static RwLock<Option<Arc<T>>>) -> Option<Arc<T>> {
    if let Some(v) = held.read().clone() {
        return Some(v);
    }
    // A dataset that failed to download is not going to download on the next
    // frame either, so the attempt is made once and then only on request.
    if !which.attempted() {
        load(which, When::IfDue);
    }
    None
}

/// One cached dataset, as the settings pane lists them.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Which {
    Airports,
    Repeaters,
    DmrIds,
    NxdnIds,
    /// One digital voice network's host file. A row each, because that is
    /// what a publisher, a refresh and a failure belong to.
    Gateway(&'static HostFile),
    CellOperators,
    CellTowers,
    Artemis,
    SigIdUnid,
    /// One CelesTrak group's orbital elements. A row each, for the same
    /// reason the gateways get one: they are published, refreshed and go
    /// stale on their own.
    Satellites(&'static datasets::tle::Group),
    /// What those satellites transmit on.
    Transmitters,
}

impl Which {
    /// Every dataset, in the order the pane lists them. A slice built once
    /// rather than a constant array: the gateway rows come from the host
    /// file table, so adding a network adds a row here without touching
    /// this.
    pub fn all() -> &'static [Which] {
        static ALL: OnceLock<Vec<Which>> = OnceLock::new();
        ALL.get_or_init(|| {
            let mut v = vec![Which::Airports, Which::Repeaters, Which::DmrIds, Which::NxdnIds];
            v.extend(datasets::gateways::HOST_FILES.iter().copied().map(Which::Gateway));
            v.extend([Which::CellOperators, Which::CellTowers, Which::Artemis, Which::SigIdUnid]);
            v.extend(datasets::tle::GROUPS.iter().copied().map(Which::Satellites));
            v.push(Which::Transmitters);
            v
        })
    }

    pub fn label(self) -> String {
        match self {
            Which::Airports => "Airports".into(),
            Which::Repeaters => "DMR repeaters".into(),
            Which::DmrIds => "DMR IDs".into(),
            Which::NxdnIds => "NXDN IDs".into(),
            Which::Gateway(h) => format!("{} gateways", h.name),
            Which::Satellites(g) => format!("{} satellites", g.name),
            Which::Transmitters => "Satellite transmitters".into(),
            Which::CellOperators => "Mobile networks".into(),
            Which::CellTowers => "Cell towers".into(),
            Which::Artemis => "Identified signals".into(),
            Which::SigIdUnid => "Unidentified signals".into(),
        }
    }

    /// Where it comes from, for the line under the name.
    pub fn publisher(self) -> &'static str {
        match self {
            Which::Airports => "ourairports.com",
            Which::Gateway(h) => h.publisher,
            Which::Satellites(g) => g.publisher,
            Which::Transmitters => "db.satnogs.org",
            Which::CellOperators => "github.com/pbakondy/mcc-mnc-list",
            Which::CellTowers => "opencellid.org",
            Which::Artemis => "github.com/AresValley/Artemis-DB",
            Which::SigIdUnid => "sigidwiki.com",
            _ => "radioid.net",
        }
    }

    /// The page the data comes from, for the link beside the row.
    ///
    /// A page a person can read rather than the file the fetch uses: the
    /// terms, the credit and the contact are on the page, and the CSV is
    /// not. Never the tower export's own URL, which carries the token.
    pub fn page(self) -> &'static str {
        match self {
            Which::Airports => "https://ourairports.com/data/",
            Which::Gateway(h) => h.page,
            Which::Satellites(g) => g.page,
            Which::Transmitters => "https://db.satnogs.org/",
            Which::CellOperators => "https://github.com/pbakondy/mcc-mnc-list",
            Which::CellTowers => "https://opencellid.org/",
            Which::Artemis => "https://github.com/AresValley/Artemis-DB",
            Which::SigIdUnid => "https://www.sigidwiki.com/",
            _ => "https://radioid.net/",
        }
    }

    /// The credential this dataset cannot be fetched without, or `None` for
    /// the ones anybody can download.
    ///
    /// Declared here so the row asks for it. A token is not a preference:
    /// it is the one thing standing between that row and a download, and a
    /// field for it under the list reads as a setting of the pane.
    pub fn keys(self) -> &'static [Key] {
        match self {
            Which::CellTowers => &[Key {
                label: "token",
                hint: "pk.0123456789abcdef",
                help: "An account at opencellid.org gives a download token. It goes in the \
                       URL of the export, so without one this row cannot be fetched.",
                secret: true,
            }],
            Which::Satellites(g) if g.needs_login() => &[
                Key {
                    label: "identity",
                    hint: "you@example.com",
                    help: "The e-mail a Space-Track account is registered under. The query \
                           is answered only while logged in, so without an account this row \
                           cannot be fetched.",
                    secret: false,
                },
                Key {
                    label: "password",
                    hint: "the account's password",
                    help: "Posted to Space-Track's login once per refresh and held in \
                           memory and in the session file. Nothing else is done with it.",
                    secret: true,
                },
            ],
            _ => &[],
        }
    }

    /// What is held for the key at `index`, and where a new one is put.
    /// Empty for a dataset that needs none.
    pub fn key_value(self, index: usize) -> String {
        match (self, index) {
            (Which::CellTowers, 0) => opencellid_token(),
            (Which::Satellites(g), i) if g.needs_login() => {
                let a = datasets::spacetrack::account().unwrap_or_default();
                match i {
                    0 => a.identity,
                    _ => a.password,
                }
            }
            _ => String::new(),
        }
    }

    pub fn set_key(self, index: usize, value: &str) {
        match (self, index) {
            (Which::CellTowers, 0) => set_opencellid_token(value),
            (Which::Satellites(g), i) if g.needs_login() => {
                let mut a = datasets::spacetrack::account().unwrap_or_default();
                match i {
                    0 => a.identity = value.trim().to_string(),
                    _ => a.password = value.to_string(),
                }
                datasets::spacetrack::set_account(Some(a));
            }
            _ => {}
        }
    }

    /// Who to name, and under what, where this data is drawn.
    ///
    /// Shorter than [`Self::terms`], which is a line in a settings pane and
    /// can afford a sentence. This one goes in the corner of a map, where
    /// the name the publisher asks for and the licence in brief are all
    /// there is room for and all the licences ask for.
    pub fn credit(self) -> Credit {
        let (name, licence) = match self {
            Which::Airports => ("OurAirports", "public domain"),
            Which::CellOperators => ("mcc-mnc-list", "MIT"),
            Which::CellTowers => ("OpenCelliD", "CC BY-SA 4.0"),
            Which::Artemis => ("Artemis-DB", "sigidwiki.com"),
            Which::SigIdUnid => ("sigidwiki.com", "contributors"),
            Which::Gateway(h) => (h.name, h.publisher),
            Which::Satellites(g) => (g.credit_name, g.credit_licence),
            Which::Transmitters => ("SatNOGS DB", "CC BY-SA 4.0"),
            _ => ("radioid.net", "amateur use"),
        };
        Credit { name, licence, url: self.page() }
    }

    /// The terms the copy on this machine is held under, as the publisher
    /// states them.
    ///
    /// Shown on every row rather than buried in a document, because two of
    /// these require credit wherever the data is used and one of them says
    /// so in writing: OpenCelliD asks for a visible "OpenCelliD" and a link
    /// to opencellid.org. A receiver that draws somebody's masts on a map
    /// and says nothing about where they came from is not complying with
    /// that, and the person running it cannot comply either if the program
    /// never told them.
    pub fn terms(self) -> &'static str {
        match self {
            Which::Airports => "public domain (OurAirports)",
            Which::CellOperators => "MIT (pbakondy/mcc-mnc-list)",
            Which::CellTowers => "CC BY-SA 4.0, credit OpenCelliD and link opencellid.org",
            Which::Artemis => "Artemis-DB, from the Signal Identification Wiki",
            Which::SigIdUnid => "sigidwiki.com contributors",
            Which::Gateway(h) => h.terms,
            Which::Satellites(g) => g.terms,
            Which::Transmitters => "CC BY-SA 4.0, credit the SatNOGS project",
            // radioid.net publishes the registry for amateur use and states
            // no licence, so the honest line is who it belongs to.
            _ => "radioid.net, for amateur radio use",
        }
    }

    /// What the dataset is for, so the pane says why it is being downloaded.
    pub fn about(self) -> &'static str {
        match self {
            Which::Airports => {
                "Airfields and their tower, ground and ATIS frequencies, drawn on the map \
                 under the aircraft."
            }
            Which::Repeaters => {
                "Registered DMR repeaters with their output frequency, offset and colour code."
            }
            Which::DmrIds => {
                "Every registered DMR ID. A digital voice frame carries a number, and this \
                 is what turns it into a callsign without asking anybody over the network."
            }
            Which::NxdnIds => "The same registry for NXDN.",
            Which::Gateway(h) => h.about,
            Which::Satellites(g) => g.about,
            Which::Transmitters => {
                "What each satellite transmits on, from the SatNOGS database: the downlink, \
                 the mode and the baud rate, corrected against what their ground stations \
                 actually hear. Elements say where a satellite will be; this says where to \
                 tune when it gets there."
            }
            Which::CellOperators => {
                "Which network an MCC and MNC belong to, so a decoded GSM beacon reads as an \
                 operator and a country rather than two numbers. Also what picks the cell \
                 export below for the country this receiver is in."
            }
            Which::CellTowers => {
                "Where the cells of this country have been heard, from the OpenCelliD \
                 community export: a position and a rough radius for a decoded cell \
                 identity. It needs a download token of your own, and OpenCelliD allows two \
                 downloads of a file a day."
            }
            Which::Artemis => {
                "Every signal the Signal Identification Wiki names, with frequency, keying \
                 and width, as the Artemis crawler packages it: the SQLite file at the front \
                 of its release, without the waterfalls and audio behind it."
            }
            Which::SigIdUnid => {
                "The signals the wiki is still asking about, from its own database query. \
                 A burst that matches one of these has been heard by somebody else too."
            }
        }
    }

    fn sources(self) -> Vec<datasets::Source> {
        use datasets::{airports, radioid};
        match self {
            Which::Airports => vec![airports::airports_source(), airports::frequencies_source()],
            Which::Repeaters => vec![radioid::repeaters_source()],
            Which::DmrIds => vec![radioid::users_source()],
            Which::NxdnIds => vec![radioid::nxdn_source()],
            Which::Gateway(h) => vec![h.source()],
            Which::Satellites(g) => vec![g.source()],
            Which::Transmitters => vec![datasets::satnogs::source()],
            Which::CellOperators => vec![datasets::cells::operators_source()],
            // Nothing to fetch until both halves of the URL exist. An empty
            // list reads as nothing held, which is the truth.
            Which::CellTowers => match (cell_mcc(), opencellid_token()) {
                (Some(mcc), t) if !t.is_empty() => vec![datasets::cells::towers_source(mcc, &t)],
                _ => Vec::new(),
            },
            Which::Artemis => vec![sigid::artemis_source()],
            Which::SigIdUnid => vec![sigid::unid_source()],
        }
    }

    /// Why this dataset cannot be fetched yet, for the pane to say instead of
    /// offering a refresh that would fail.
    pub fn blocked(self) -> Option<&'static str> {
        if let Which::Satellites(g) = self {
            return match g.needs_login() && !datasets::spacetrack::has_account() {
                true => Some("needs a Space-Track login"),
                false => None,
            };
        }
        if self != Which::CellTowers {
            return None;
        }
        if opencellid_token().is_empty() {
            return Some("needs an OpenCelliD token");
        }
        if country().is_empty() {
            return Some("needs a country, set in Setup");
        }
        // The MCC comes out of the operator table, which the fetch downloads
        // for itself before it needs one. Waiting on that table is not the
        // same as having no country, and saying so sent operators who had set
        // one back to Setup to set it again.
        if OPERATORS.read().is_some() && cell_mcc().is_none() {
            return Some("no mobile network is registered in this country");
        }
        None
    }

    fn index(self) -> usize {
        Self::all().iter().position(|w| *w == self).unwrap_or(0)
    }

    /// How many rows are held, or `None` when it is not loaded.
    fn rows(self) -> Option<usize> {
        match self {
            Which::Airports => match airports().len() {
                0 => None,
                n => Some(n),
            },
            Which::Repeaters => REPEATERS.read().as_ref().map(|r| r.len()),
            Which::DmrIds => USERS.read().as_ref().map(|u| u.len()),
            Which::NxdnIds => NXDN.read().as_ref().map(|u| u.len()),
            Which::Gateway(h) => GATEWAYS[host_file_index(h)].read().as_ref().map(|g| g.len()),
            Which::Satellites(g) => SATS[group_index(g)].read().as_ref().map(|s| s.len()),
            Which::Transmitters => TRANSMITTERS.read().as_ref().map(|t| t.len()),
            Which::CellOperators => OPERATORS.read().as_ref().map(|o| o.len()),
            Which::CellTowers => CELLS.read().as_ref().map(|c| c.len()),
            Which::Artemis => ARTEMIS.read().as_ref().map(|s| s.len()),
            Which::SigIdUnid => UNID.read().as_ref().map(|s| s.len()),
        }
    }

    fn attempted(self) -> bool {
        work_slot(self).attempted.load(Ordering::Acquire)
    }
}

/// What a load or refresh is doing, for the settings pane to draw. One slot
/// per dataset, in [`Which::all`] order.
struct Work {
    busy: AtomicBool,
    attempted: AtomicBool,
    error: RwLock<Option<String>>,
}

impl Work {
    const fn new() -> Self {
        Self {
            busy: AtomicBool::new(false),
            attempted: AtomicBool::new(false),
            error: RwLock::new(None),
        }
    }
}

static WORK: LazyLock<Vec<Work>> =
    LazyLock::new(|| Which::all().iter().map(|_| Work::new()).collect());

fn work_slot(which: Which) -> &'static Work {
    &WORK[which.index()]
}

/// A dataset as the settings pane shows it: what is held, how big it is on
/// disk, when it was last checked, and whatever went wrong last time.
pub struct Row {
    pub which: Which,
    pub rows: Option<usize>,
    /// Bytes on disk across the dataset's files. Zero when nothing is cached.
    pub bytes: u64,
    /// Seconds since the last successful check, or `None` if never checked.
    pub checked_ago: Option<u64>,
    pub busy: bool,
    pub error: Option<String>,
    /// Set when the dataset cannot be fetched as things stand, such as a
    /// token that has not been given. The pane says this instead of offering
    /// a refresh that would only fail.
    pub blocked: Option<&'static str>,
}

pub fn status() -> Vec<Row> {
    let cache = cache();
    Which::all()
        .iter()
        .copied()
        .map(|which| {
            let mut bytes = 0;
            let mut oldest: Option<u64> = None;
            let mut present = true;
            for src in which.sources() {
                let s = cache.map(|c| c.status(&src)).unwrap_or_default();
                match s.bytes {
                    Some(b) => bytes += b,
                    None => present = false,
                }
                // A dataset of two files is as fresh as its stalest half. A
                // stamp of zero is a clock that was not readable when the
                // file landed, which is not a check in 1970.
                if let Some(c) = s.checked.filter(|c| *c > 0) {
                    oldest = Some(oldest.map_or(c, |o: u64| o.min(c)));
                }
            }
            let w = work_slot(which);
            Row {
                which,
                rows: which.rows(),
                bytes,
                checked_ago: present.then(|| oldest.map(|c| now().saturating_sub(c))).flatten(),
                busy: w.busy.load(Ordering::Acquire),
                error: w.error.read().clone(),
                blocked: which.blocked(),
            }
        })
        .collect()
}

pub fn cache_dir() -> Option<PathBuf> {
    cache().map(|c| c.dir().to_path_buf())
}

/// Check now, whatever the age of the last check, and reload what changed.
/// This is the refresh button.
pub fn refresh(which: Which) {
    load(which, When::Now);
}

/// Load or refresh one dataset on a thread of its own, so a slow 85 MB
/// download does not hold up the three small ones or the frame.
fn load(which: Which, when: When) {
    let w = work_slot(which);
    if w.busy.swap(true, Ordering::AcqRel) {
        return;
    }
    w.attempted.store(true, Ordering::Release);
    let name = which.label();
    let started = std::thread::Builder::new()
        .name(format!("dataset-{}", which.index()))
        .spawn(move || {
            let outcome = cache().map_or_else(
                || Err("no cache directory".to_string()),
                |c| work(which, c, when).map_err(|e| e.to_string()),
            );
            match &outcome {
                Ok(()) => tracing::info!(dataset = name, "dataset ready"),
                Err(e) => tracing::warn!(dataset = name, "dataset unavailable: {e}"),
            }
            let w = work_slot(which);
            *w.error.write() = outcome.err();
            w.busy.store(false, Ordering::Release);
        })
        .is_ok();
    if !started {
        w.busy.store(false, Ordering::Release);
    }
}

/// Read what is cached, then ask whether it changed. On a cold cache the
/// first step is the download and the second answers 304 straight away; on a
/// warm one the first step is a file read.
fn work(which: Which, cache: &Cache, when: When) -> Result<(), datasets::Error> {
    use datasets::{airports, cells, gateways, radioid};
    match which {
        Which::Airports => {
            if airports().is_empty() {
                publish_airports(airports::load(cache)?);
            }
            if let Some(a) = airports::refresh(cache, when)? {
                publish_airports(a);
            }
        }
        Which::Repeaters => {
            if REPEATERS.read().is_none() {
                *REPEATERS.write() = Some(Arc::new(radioid::load_repeaters(cache)?));
            }
            if let Some(r) = radioid::refresh_repeaters(cache, when)? {
                *REPEATERS.write() = Some(Arc::new(r));
            }
        }
        Which::DmrIds => {
            if USERS.read().is_none() {
                *USERS.write() = Some(Arc::new(radioid::load_users(cache)?));
            }
            if let Some(u) = radioid::refresh_users(cache, when)? {
                *USERS.write() = Some(Arc::new(u));
            }
        }
        Which::NxdnIds => {
            if NXDN.read().is_none() {
                *NXDN.write() = Some(Arc::new(radioid::load_nxdn(cache)?));
            }
            if let Some(u) = radioid::refresh_nxdn(cache, when)? {
                *NXDN.write() = Some(Arc::new(u));
            }
        }
        Which::Gateway(h) => {
            let slot = &GATEWAYS[host_file_index(h)];
            if slot.read().is_none() {
                *slot.write() = Some(Arc::new(gateways::load_one(cache, h)?));
            }
            if let Some(g) = gateways::refresh_one(cache, h, when)? {
                *slot.write() = Some(Arc::new(g));
            }
        }
        Which::Transmitters => {
            if TRANSMITTERS.read().is_none() {
                *TRANSMITTERS.write() = Some(Arc::new(datasets::satnogs::load(cache)?));
            }
            if let Some(t) = datasets::satnogs::refresh(cache, when)? {
                *TRANSMITTERS.write() = Some(Arc::new(t));
            }
        }
        Which::Satellites(g) => {
            let slot = &SATS[group_index(g)];
            if slot.read().is_none() {
                *slot.write() = Some(Arc::new(tle::load(cache, g)?));
            }
            if let Some(s) = tle::refresh(cache, g, when)? {
                *slot.write() = Some(Arc::new(s));
            }
        }
        Which::CellOperators => {
            if OPERATORS.read().is_none() {
                *OPERATORS.write() = Some(Arc::new(cells::load_operators(cache)?));
            }
            if let Some(o) = cells::refresh_operators(cache, when)? {
                *OPERATORS.write() = Some(Arc::new(o));
            }
        }
        Which::CellTowers => {
            // The country's MCC is read out of the operator table, so that
            // has to be here first. Fetching it is the cheap half.
            if OPERATORS.read().is_none() {
                *OPERATORS.write() = Some(Arc::new(cells::load_operators(cache)?));
            }
            let token = opencellid_token();
            let (Some(mcc), false) = (cell_mcc(), token.is_empty()) else {
                return Err(datasets::Error::Parse(
                    "opencellid".into(),
                    Which::CellTowers.blocked().unwrap_or("not configured").into(),
                ));
            };
            if CELLS.read().is_none() {
                *CELLS.write() = Some(Arc::new(cells::load_towers(cache, mcc, &token)?));
            }
            if let Some(c) = cells::refresh_towers(cache, mcc, &token, when)? {
                *CELLS.write() = Some(Arc::new(c));
            }
        }
        Which::Artemis => {
            if ARTEMIS.read().is_none() {
                *ARTEMIS.write() = Some(Arc::new(sigid::load_artemis(cache)?));
                *SIGID.write() = None;
            }
            if let Some(s) = sigid::refresh_artemis(cache, when)? {
                *ARTEMIS.write() = Some(Arc::new(s));
                *SIGID.write() = None;
            }
        }
        Which::SigIdUnid => {
            if UNID.read().is_none() {
                *UNID.write() = Some(Arc::new(sigid::load_unid(cache)?));
                *SIGID.write() = None;
            }
            if let Some(s) = sigid::refresh_unid(cache, when)? {
                *UNID.write() = Some(Arc::new(s));
                *SIGID.write() = None;
            }
        }
    }
    Ok(())
}

fn publish_airports(v: Vec<Airport>) {
    tracing::info!(count = v.len(), "airports loaded");
    *AIRPORTS.write() = Vec::leak(v);
}

fn cache() -> Option<&'static Cache> {
    static CACHE: OnceLock<Option<Cache>> = OnceLock::new();
    CACHE
        .get_or_init(|| match Cache::at_default_dir() {
            Ok(c) => Some(c),
            Err(e) => {
                tracing::warn!("no dataset cache: {e}");
                None
            }
        })
        .as_ref()
}

fn now() -> u64 {
    SystemTime::now().duration_since(UNIX_EPOCH).map(|d| d.as_secs()).unwrap_or(0)
}

/// Load what the map needs. The registries are left alone: they are large,
/// and nothing has asked them a question yet.
pub fn start() {
    load(Which::Airports, When::IfDue);
}

/// Download or revalidate every dataset and report what happened, for
/// `--fetch-data`. Warming the cache before going somewhere without a
/// connection is the point, so this waits and prints rather than logging.
pub fn fetch_all() {
    let Ok(cache) = Cache::at_default_dir() else {
        eprintln!("no cache directory");
        return;
    };
    println!("cache: {}", cache.dir().display());
    // Nothing has drawn a window here, so the country and the token the cell
    // export needs are still only in the saved settings.
    let s = crate::session::Session::load();
    if country().is_empty() {
        set_country(&s.country);
    }
    if opencellid_token().is_empty() {
        set_opencellid_token(&s.opencellid_token);
    }
    // Each pair is written refresh first, and arguments evaluate in order, so
    // the count reported is of the file after any update rather than before.
    each(
        "airports",
        datasets::airports::refresh(&cache, When::Now).map(|u| u.is_some()),
        datasets::airports::load(&cache).map(|a| a.len()),
    );
    each(
        "dmr repeaters",
        datasets::radioid::refresh_repeaters(&cache, When::Now).map(|u| u.is_some()),
        datasets::radioid::load_repeaters(&cache).map(|r| r.len()),
    );
    each(
        "nxdn ids",
        datasets::radioid::refresh_nxdn(&cache, When::Now).map(|u| u.is_some()),
        datasets::radioid::load_nxdn(&cache).map(|u| u.len()),
    );
    each(
        "dmr ids",
        datasets::radioid::refresh_users(&cache, When::Now).map(|u| u.is_some()),
        datasets::radioid::load_users(&cache).map(|u| u.len()),
    );
    for h in datasets::gateways::HOST_FILES.iter().copied() {
        each(
            &format!("{} gateways", h.name),
            datasets::gateways::refresh_one(&cache, h, When::Now).map(|u| u.is_some()),
            datasets::gateways::load_one(&cache, h).map(|g| g.len()),
        );
    }
    each(
        "mobile networks",
        datasets::cells::refresh_operators(&cache, When::Now).map(|u| u.is_some()),
        datasets::cells::load_operators(&cache).map(|o| {
            // Held, because the cell export below reads its MCC out of it.
            let n = o.len();
            *OPERATORS.write() = Some(Arc::new(o));
            n
        }),
    );
    // The cell export is the one dataset warming the cache cannot decide to
    // fetch on its own: it costs a token and a choice of country.
    match (cell_mcc(), opencellid_token()) {
        (Some(mcc), t) if !t.is_empty() => each(
            "cell towers",
            datasets::cells::refresh_towers(&cache, mcc, &t, When::Now).map(|u| u.is_some()),
            datasets::cells::load_towers(&cache, mcc, &t).map(|c| c.len()),
        ),
        _ => println!("cell towers: {}", Which::CellTowers.blocked().unwrap_or("skipped")),
    }
    each(
        "identified signals",
        datasets::sigid::refresh_artemis(&cache, When::Now).map(|u| u.is_some()),
        datasets::sigid::load_artemis(&cache).map(|d| d.len()),
    );
    each(
        "unidentified signals",
        datasets::sigid::refresh_unid(&cache, When::Now).map(|u| u.is_some()),
        datasets::sigid::load_unid(&cache).map(|d| d.len()),
    );
    each(
        "satellite transmitters",
        datasets::satnogs::refresh(&cache, When::Now).map(|u| u.is_some()),
        datasets::satnogs::load(&cache).map(|t| t.len()),
    );
    for g in datasets::tle::GROUPS.iter().copied() {
        each(
            &format!("{} satellites", g.name),
            tle::refresh(&cache, g, When::Now).map(|u| u.is_some()),
            tle::load(&cache, g).map(|s| s.len()),
        );
    }
}

fn each(
    what: &str,
    // Both halves are results because a publisher being down must report
    // against that dataset alone and let the rest run.
    refreshed: Result<bool, datasets::Error>,
    count: Result<usize, datasets::Error>,
) {
    let state = match refreshed {
        Ok(true) => "updated".to_string(),
        Ok(false) => "current".to_string(),
        Err(e) => format!("not refreshed: {e}"),
    };
    match count {
        Ok(n) => println!("{what}: {n}, {state}"),
        Err(e) => println!("{what}: {e}"),
    }
}

/// Sizes in the settings pane, where a byte count is noise and a rounded
/// number is the whole point.
pub fn fmt_bytes(b: u64) -> String {
    match b {
        0 => "—".into(),
        b if b < 1 << 20 => format!("{} kB", b >> 10),
        b => format!("{:.1} MB", b as f64 / (1u64 << 20) as f64),
    }
}

/// How long ago, in the coarsest unit that still says something.
pub fn fmt_ago(secs: u64) -> String {
    match secs {
        s if s < 90 => "just now".into(),
        s if s < 5400 => format!("{} min ago", s / 60),
        s if s < 172_800 => format!("{} h ago", s / 3600),
        s => format!("{} days ago", s / 86_400),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sizes_round_to_something_readable() {
        assert_eq!(fmt_bytes(0), "—");
        assert_eq!(fmt_bytes(930_667), "908 kB");
        assert_eq!(fmt_bytes(84_506_836), "80.6 MB");
    }

    #[test]
    fn ages_read_as_a_person_would_say_them() {
        assert_eq!(fmt_ago(5), "just now");
        assert_eq!(fmt_ago(600), "10 min ago");
        assert_eq!(fmt_ago(7200), "2 h ago");
        assert_eq!(fmt_ago(400_000), "4 days ago");
    }

    #[test]
    fn every_dataset_has_at_least_one_source_and_its_own_slot() {
        for w in Which::all().iter().copied() {
            // The cell export is the exception, and deliberately: its URL
            // carries a token and an MCC, so there is nothing to fetch until
            // both exist and the pane says which one is missing.
            if w == Which::CellTowers {
                continue;
            }
            assert!(!w.sources().is_empty(), "{} has no source", w.label());
        }
        let idx: Vec<usize> = Which::all().iter().map(|w| w.index()).collect();
        assert_eq!(idx, (0..Which::all().len()).collect::<Vec<_>>(), "slots must be distinct");
        assert_eq!(WORK.len(), Which::all().len());
    }

    #[test]
    fn a_gateway_row_exists_for_every_published_host_file() {
        for h in datasets::gateways::HOST_FILES.iter().copied() {
            let w = Which::Gateway(h);
            assert!(Which::all().contains(&w), "{} has no dataset row", h.name);
            // One file each, or a refresh of one network would overwrite
            // another's cache entry and report its rows.
            assert_eq!(w.sources().len(), 1);
            assert_eq!(w.publisher(), h.publisher);
        }
    }

    /// One row, one cache file and one slot per group, or refreshing one
    /// would overwrite another's file and report its count.
    #[test]
    fn a_satellite_row_exists_for_every_published_group() {
        for g in datasets::tle::GROUPS.iter().copied() {
            let w = Which::Satellites(g);
            assert!(Which::all().contains(&w), "{} has no dataset row", g.name);
            assert_eq!(w.sources().len(), 1);
            // The row says who published it, which is not always CelesTrak:
            // a group naming them for a file fetched from somewhere else is
            // a credit given to the wrong project.
            assert_eq!(w.publisher(), g.publisher);
            assert_eq!(w.page(), g.page);
        }
        let files: Vec<usize> = datasets::tle::GROUPS.iter().copied().map(group_index).collect();
        assert_eq!(files, (0..datasets::tle::GROUPS.len()).collect::<Vec<_>>());
    }

    #[test]
    fn the_cell_export_says_what_it_is_waiting_for() {
        set_opencellid_token("");
        set_country("gb");
        assert_eq!(Which::CellTowers.blocked(), Some("needs an OpenCelliD token"));
        set_opencellid_token("pk.test");
        set_country("");
        assert_eq!(Which::CellTowers.blocked(), Some("needs a country, set in Setup"));
        // A country with no operator table loaded yet is not blocked: the
        // fetch downloads that table itself.
        set_country("gb");
        assert_eq!(Which::CellTowers.blocked(), None);
        set_opencellid_token("");
        set_country("");
    }

    /// Space-Track answers nothing without a session, so the row asks for
    /// both halves of a login and says so until it has them. Two fields
    /// rather than one because an identity is an e-mail a person needs to
    /// see and a password is not.
    #[test]
    fn the_catalogue_row_asks_for_a_login_and_says_so_until_it_has_one() {
        let w = Which::Satellites(&datasets::tle::SPACE_TRACK);
        let keys = w.keys();
        assert_eq!(keys.len(), 2);
        assert_eq!((keys[0].label, keys[0].secret), ("identity", false));
        assert_eq!((keys[1].label, keys[1].secret), ("password", true));

        datasets::spacetrack::set_account(None);
        assert_eq!(w.blocked(), Some("needs a Space-Track login"));
        w.set_key(0, "someone@example.com");
        assert_eq!(w.blocked(), Some("needs a Space-Track login"), "half a login is none");
        w.set_key(1, "hunter2");
        assert_eq!(w.blocked(), None);
        assert_eq!(w.key_value(0), "someone@example.com");
        assert_eq!(w.key_value(1), "hunter2");

        // A group anybody may fetch asks for nothing and is never blocked.
        let open = Which::Satellites(&datasets::tle::AMATEUR);
        assert!(open.keys().is_empty());
        assert_eq!(open.blocked(), None);
        datasets::spacetrack::set_account(None);
    }
}
