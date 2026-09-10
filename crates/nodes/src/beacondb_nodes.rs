//! Feeding beaconDB, as a node on the packet bus.
//!
//! The same shape as the WiGLE feed and for the same reasons: a spool on
//! disc, one process-wide thread sending the closed files oldest first, and
//! nothing deleted until the far end has said it took it. A drive happens
//! where the network does not, and a laptop that comes home to a wireless
//! network sends the whole drive without being asked.
//!
//! What is different is that there is no account. beaconDB takes submissions
//! from anybody, identifies clients by their user agent, and publishes what
//! it collects into the public domain, so the switch is a switch and not a
//! credential. That makes the consent question sharper rather than softer:
//! this is off until an operator turns it on, and what it sends is a list of
//! places this receiver has been.

use common::Result;
use pipeline::node::{NodeCtx, PortSpec, Simple};
use pipeline::port::{Payload, PortKind, StreamSpec};
use std::collections::HashMap;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex, OnceLock};
use std::time::{Duration, Instant};
use pipeline::registry::{Category, Settings, StageDesc};

/// A spool file is closed once it holds this many observations.
const ITEMS_PER_FILE: usize = 500;

/// Or once it is this old, so a quiet evening's observations are not still in
/// a part file when the laptop is closed.
const MAX_AGE: Duration = Duration::from_secs(300);

/// How often the sender looks for something to send.
const POLL: Duration = Duration::from_secs(10);

/// After a failure, how long before the same file is tried again, and the
/// ceiling the backoff climbs to.
const RETRY: Duration = Duration::from_secs(30);
const RETRY_MAX: Duration = Duration::from_secs(15 * 60);

/// What the feed is doing, for the interface to draw.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct BeaconDbStatus {
    /// Whether the operator has turned the feed on.
    pub on: bool,
    /// Files written and not yet accepted, and the observations in them.
    pub queued_files: u64,
    pub queued_items: u64,
    /// Collected since the receiver started but not yet in a file.
    pub pending_items: u64,
    pub sent_files: u64,
    pub sent_items: u64,
    /// Why the last attempt failed, when it did.
    pub error: Option<String>,
    pub spool: PathBuf,
}

/// The thread that sends files, and everything it reports.
pub struct Sender {
    dir: Mutex<PathBuf>,
    on: AtomicBool,
    sent_files: AtomicU64,
    sent_items: AtomicU64,
    error: Mutex<Option<String>>,
    started: AtomicBool,
}

impl Sender {
    /// A sender with no thread behind it, which is what a test wants: the
    /// observations and the spool are testable, the network is not.
    pub fn inert(dir: &Path) -> Arc<Self> {
        Arc::new(Sender {
            dir: Mutex::new(dir.to_path_buf()),
            on: AtomicBool::new(false),
            sent_files: AtomicU64::new(0),
            sent_items: AtomicU64::new(0),
            error: Mutex::new(None),
            started: AtomicBool::new(false),
        })
    }

    /// The one sender, started the first time a node asks for it.
    pub fn shared(dir: &Path) -> Arc<Self> {
        static SENDER: OnceLock<Arc<Sender>> = OnceLock::new();
        let s = SENDER.get_or_init(|| Sender::inert(dir));
        s.set_dir(dir);
        s.start();
        s.clone()
    }

    fn start(self: &Arc<Self>) {
        if self.started.swap(true, Ordering::SeqCst) {
            return;
        }
        let s = self.clone();
        let _ = std::thread::Builder::new().name("beacondb-submit".into()).spawn(move || s.run());
    }

    pub fn set_on(&self, on: bool) {
        self.on.store(on, Ordering::Relaxed);
    }

    pub fn is_on(&self) -> bool {
        self.on.load(Ordering::Relaxed)
    }

    fn set_dir(&self, dir: &Path) {
        if let Ok(mut d) = self.dir.lock() {
            if d.as_path() != dir {
                *d = dir.to_path_buf();
            }
        }
    }

    pub fn dir(&self) -> PathBuf {
        self.dir.lock().map(|d| d.clone()).unwrap_or_default()
    }

    pub fn status(&self) -> BeaconDbStatus {
        let (queued_files, queued_items) = queued(&self.dir());
        BeaconDbStatus {
            on: self.is_on(),
            queued_files,
            queued_items,
            pending_items: 0,
            sent_files: self.sent_files.load(Ordering::Relaxed),
            sent_items: self.sent_items.load(Ordering::Relaxed),
            error: self.error.lock().ok().and_then(|e| e.clone()),
            spool: self.dir(),
        }
    }

    fn run(&self) {
        let mut wait = RETRY;
        loop {
            std::thread::sleep(POLL);
            // A spool that keeps being sent after the switch was turned off
            // is a feed the operator cannot stop. What is already written
            // waits on disc until it is turned back on.
            if !self.is_on() {
                continue;
            }
            let Some(path) = oldest(&self.dir()) else { continue };
            let Ok(body) = std::fs::read(&path) else {
                self.say(Some(format!("{}: unreadable", path.display())));
                continue;
            };
            match survey::beacondb::submit(&body) {
                Ok(()) => {
                    let items = items_in(&path);
                    // Deleted only now: a file sent but not acknowledged is
                    // one this will send again, which is the failure worth
                    // having.
                    let _ = std::fs::remove_file(&path);
                    self.sent_files.fetch_add(1, Ordering::Relaxed);
                    self.sent_items.fetch_add(items, Ordering::Relaxed);
                    self.say(None);
                    wait = RETRY;
                }
                Err(e) => {
                    self.say(Some(e));
                    std::thread::sleep(wait);
                    wait = (wait * 2).min(RETRY_MAX);
                }
            }
        }
    }

    fn say(&self, msg: Option<String>) {
        if let Ok(mut e) = self.error.lock() {
            *e = msg;
        }
    }
}

/// How many observations a spool file holds, taken from its name, so what is
/// waiting can be reported from a directory listing rather than by reading
/// every file at the display's rate.
fn items_in(path: &Path) -> u64 {
    path.file_stem()
        .and_then(|n| n.to_str())
        .and_then(|n| n.rsplit('-').next())
        .and_then(|n| n.parse().ok())
        .unwrap_or(0)
}

/// Closed spool files, oldest first by name, which is the order they were
/// written in: the name carries the time.
fn spooled(dir: &Path) -> Vec<PathBuf> {
    let mut files: Vec<PathBuf> = std::fs::read_dir(dir)
        .into_iter()
        .flatten()
        .filter_map(|e| e.ok().map(|e| e.path()))
        .filter(|p| p.extension().is_some_and(|e| e == "json"))
        .collect();
    files.sort();
    files
}

fn oldest(dir: &Path) -> Option<PathBuf> {
    spooled(dir).into_iter().next()
}

fn queued(dir: &Path) -> (u64, u64) {
    let files = spooled(dir);
    (files.len() as u64, files.iter().map(|p| items_in(p)).sum())
}

/// The bus consumer that turns decodes into beaconDB observations.
pub struct BeaconDbNode {
    sender: Arc<Sender>,
    /// Where this node's observations are written. The sender has its own
    /// copy; a node writing where nobody reads is a test, not a fault.
    spool: PathBuf,
    /// Where the receiver is. Without one there is nothing to submit: an
    /// observation is a claim about a place.
    station: Option<gps::Fix>,
    pending: Vec<String>,
    /// The last sighting submitted per device, for the same thinning the
    /// survey does.
    last: HashMap<(String, String), survey::Sighting>,
    opened: Instant,
    seq: u64,
    counted: std::cell::RefCell<(Option<Instant>, (u64, u64))>,
}

impl Default for BeaconDbNode {
    fn default() -> Self {
        Self::new(default_spool_dir())
    }
}

impl BeaconDbNode {
    pub fn new(spool: PathBuf) -> Self {
        Self::with(Sender::shared(&spool), spool)
    }

    /// A node on a sender somebody else made, which is how a test runs the
    /// whole path without a thread trying to reach beacondb.net.
    pub fn with(sender: Arc<Sender>, spool: PathBuf) -> Self {
        Self {
            sender,
            spool,
            station: None,
            pending: Vec::new(),
            last: HashMap::new(),
            opened: Instant::now(),
            seq: 0,
            counted: std::cell::RefCell::new((None, (0, 0))),
        }
    }

    /// Start or stop feeding.
    ///
    /// Observations are only collected while there is somewhere for them to
    /// go: spooling for an operator who never turned it on is a directory
    /// that fills with files nobody will ever send.
    pub fn set_on(&mut self, on: bool) {
        self.sender.set_on(on);
        if !on {
            self.pending.clear();
        }
    }

    pub fn is_on(&self) -> bool {
        self.sender.is_on()
    }

    pub fn set_station(&mut self, at: Option<gps::Fix>) {
        self.station = at;
    }

    pub fn status(&self) -> BeaconDbStatus {
        let (queued_files, queued_items) = self.queued();
        BeaconDbStatus {
            pending_items: self.pending.len() as u64,
            queued_files,
            queued_items,
            spool: self.spool.clone(),
            ..self.sender.status()
        }
    }

    /// What is waiting in the spool, recounted at most once a second.
    fn queued(&self) -> (u64, u64) {
        let mut held = self.counted.borrow_mut();
        if held.0.is_none_or(|t| t.elapsed() >= Duration::from_secs(1)) {
            *held = (Some(Instant::now()), queued(&self.spool));
        }
        held.1
    }

    /// Close the current file so what has been collected can go, whether or
    /// not it is full. This is what a rebuild and a shutdown do.
    pub fn flush(&mut self) {
        if self.pending.is_empty() {
            return;
        }
        let dir = self.spool.clone();
        if std::fs::create_dir_all(&dir).is_err() {
            return;
        }
        self.seq += 1;
        let items: Vec<String> = self.pending.drain(..).collect();
        let name = format!("waveshark-{}-{:04}-{}.json", now_s(), self.seq, items.len());
        let body = survey::beacondb::body(&items);
        // Written whole and then moved into place, so the sender never reads
        // a file that is still being written.
        let part = dir.join(format!("{name}.part"));
        if std::fs::write(&part, body).is_ok() {
            let _ = std::fs::rename(&part, dir.join(name));
        }
        self.counted.borrow_mut().0 = None;
        self.opened = Instant::now();
    }

}

impl Drop for BeaconDbNode {
    fn drop(&mut self) {
        self.flush();
    }
}

impl Simple for BeaconDbNode {
    fn name(&self) -> &str {
        "beacondb"
    }

    fn is_sink(&self) -> bool {
        true
    }

    fn negotiate(&mut self, i: &PortSpec) -> Result<StreamSpec> {
        if i.spec.kind != PortKind::Packets {
            return Err(common::Error::other("beacondb reads the packet bus"));
        }
        Ok(i.spec)
    }

    fn process(&mut self, i: &Payload, _o: &mut Payload, _c: &mut NodeCtx<'_>) -> Result<()> {
        if !self.is_on() {
            return Ok(());
        }
        for p in i.as_packets().unwrap_or(&[]) {
            for d in p.decodes.iter() {
                let Some((protocol, ident)) = crate::survey_nodes::identity(d) else { continue };
                if survey::beacondb::kind(&protocol).is_none() {
                    break;
                }
                let s = crate::survey_nodes::sighting(p, d, self.station);
                let key = (protocol.clone(), ident.clone());
                let fresh = match self.last.get(&key) {
                    None => true,
                    Some(prev) => survey::worth_keeping(prev, &s),
                };
                if !fresh {
                    break;
                }
                if let Some(item) = survey::beacondb::item_json(&protocol, &ident, &s) {
                    self.last.insert(key, s);
                    self.pending.push(item);
                }
                break;
            }
        }
        if self.pending.len() >= ITEMS_PER_FILE || self.opened.elapsed() >= MAX_AGE {
            self.flush();
        }
        Ok(())
    }
}

/// `$XDG_DATA_HOME/waveshark/beacondb`, beside the packet log and the survey.
pub fn default_spool_dir() -> PathBuf {
    std::env::var_os("XDG_DATA_HOME")
        .map(PathBuf::from)
        .or_else(|| std::env::var_os("HOME").map(|h| PathBuf::from(h).join(".local/share")))
        .unwrap_or_else(std::env::temp_dir)
        .join("waveshark/beacondb")
}

fn now_s() -> u64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_secs())
        .unwrap_or(0)
}

#[cfg(test)]
mod tests {
    use super::*;
    use common::{Hz, Packet};

    fn packet(bytes: Vec<u8>, center_hz: u64) -> Packet {
        Packet::of_frame(
            1_000_000,
            2_000_000,
            common::Frame::measured(bytes, -46.0, 20.0).at(center_hz),
        )
    }

    fn run(node: &mut BeaconDbNode, packets: Vec<Packet>) {
        let mut packets = packets;
        crate::PacketDecodeNode::default().annotate(&mut packets);
        let mut s = pipeline::StreamSpec::iq(0.0, Hz(2_426_000_000));
        s.kind = PortKind::Packets;
        let ins = [PortSpec { spec: s, latency: 0 }];
        let (tags, mut events, mut new_tags) = (Vec::new(), Vec::new(), Vec::new());
        let mut out = Payload::Packets(Vec::new());
        let mut ctx = NodeCtx::new(0, &ins, &tags, &mut events, &mut new_tags);
        node.process(&Payload::Packets(packets), &mut out, &mut ctx).unwrap();
    }

    fn spool() -> PathBuf {
        let dir = std::env::temp_dir().join(format!(
            "waveshark-beacondb-{}-{:?}",
            std::process::id(),
            std::thread::current().id()
        ));
        let _ = std::fs::remove_dir_all(&dir);
        dir
    }

    fn advertisement() -> Packet {
        // A Samsung monitor's ADV_IND, dewhitened and CRC checked.
        packet(
            vec![
                0x00, 0x11, 0x3a, 0xf5, 0x0a, 0xcd, 0x31, 0xe8, 0x02, 0x01, 0x06, 0x07, 0xff,
                0xe1, 0x02, 0x10, 0x00, 0x26, 0xc0,
            ],
            2_426_000_000,
        )
    }

    fn node_on(dir: PathBuf) -> BeaconDbNode {
        let mut n = BeaconDbNode::with(Sender::inert(&dir), dir);
        n.set_on(true);
        n.set_station(Some(gps::Fix { lat: 53.6369, lon: -6.6528, ..Default::default() }));
        n
    }

    /// The whole path short of the network: an advertisement off the bus
    /// becomes a spooled submission body.
    #[test]
    fn an_advertisement_becomes_an_observation_in_a_spool_file() {
        let dir = spool();
        let mut node = node_on(dir.clone());
        run(&mut node, vec![advertisement()]);
        assert_eq!(node.status().pending_items, 1);
        node.flush();
        let files = spooled(&dir);
        assert_eq!(files.len(), 1, "expected one spool file, got {files:?}");
        // Read as text: this crate has no JSON library, and what matters is
        // that the body carries the address and the place it was heard from.
        let body = std::fs::read_to_string(&files[0]).unwrap();
        assert!(body.starts_with("{\"items\":["), "{body}");
        assert!(body.contains("\"macAddress\":\"e8:31:cd:0a:f5:3a\""), "{body}");
        assert!(body.contains("\"latitude\":53.6369"), "{body}");
        assert_eq!(queued(&dir), (1, 1));
        let _ = std::fs::remove_dir_all(&dir);
    }

    /// Off is off: nothing is collected, so a spool does not fill up for an
    /// operator who never agreed to send anything.
    #[test]
    fn nothing_is_collected_while_the_feed_is_off() {
        let dir = spool();
        let mut node = BeaconDbNode::with(Sender::inert(&dir), dir.clone());
        node.set_station(Some(gps::Fix { lat: 53.6, lon: -6.6, ..Default::default() }));
        run(&mut node, vec![advertisement()]);
        assert_eq!(node.status().pending_items, 0);
        assert!(!node.is_on());
    }

    /// An observation is a claim about a place, so a receiver with no fix
    /// submits nothing rather than submitting zeroes.
    #[test]
    fn without_a_fix_there_is_nothing_to_submit() {
        let dir = spool();
        let mut node = node_on(dir);
        node.set_station(None);
        run(&mut node, vec![advertisement()]);
        assert_eq!(node.status().pending_items, 0);
    }

    /// The same beacon heard again from the same place is the same
    /// observation.
    #[test]
    fn a_stationary_repeat_is_thinned_away() {
        let dir = spool();
        let mut node = node_on(dir);
        run(&mut node, vec![advertisement(), advertisement(), advertisement()]);
        assert_eq!(node.status().pending_items, 1);
    }

    /// A rebuild drops the node, and what it collected has to survive that:
    /// the next graph's sender finds it on disc.
    #[test]
    fn dropping_the_node_writes_what_it_had() {
        let dir = spool();
        {
            let mut node = node_on(dir.clone());
            run(&mut node, vec![advertisement()]);
        }
        assert_eq!(spooled(&dir).len(), 1);
        let _ = std::fs::remove_dir_all(&dir);
    }
}

pub const DESC: StageDesc = StageDesc {
    name: "beacondb",
    summary: "Feed what was heard to beacondb.net: observations, spooled and submitted",
    category: Category::Sink,
    feeds_bus: false,
};

pub fn build(s: &Settings) -> Result<Box<dyn pipeline::node::Node>> {
    Ok(Box::new(BeaconDbNode::new(crate::spool_dir(
        s,
        default_spool_dir,
    ))))
}
