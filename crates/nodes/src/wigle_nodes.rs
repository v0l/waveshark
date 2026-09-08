//! Feeding WiGLE, as a node on the packet bus.
//!
//! A wardrive is worth more shared than kept, and WiGLE is where the sharing
//! happens: a BLE address or a cell heard from a moving car becomes a row in
//! a database anybody can search. This is the path out. It sits on the bus
//! beside the survey, reads the same decodes, and writes the ones WiGLE has a
//! type for as CSV rows.
//!
//! # Why a spool rather than a request per sighting
//!
//! WiGLE takes files, not observations, and a drive happens where the network
//! does not: the reason to record a survey in the first place is that the
//! interesting roads have no coverage. So rows go to a file under the spool
//! directory, the file is closed when it is big enough or old enough, and a
//! thread uploads closed files whenever there is a network to upload over,
//! oldest first, deleting each only once WiGLE has said it took it. Nothing
//! is held in memory that a power cut would cost, and a laptop that comes
//! home to a wireless network sends the whole drive without being asked.
//!
//! # One uploader for the process
//!
//! The graph is rebuilt on every retune, so a node lives for seconds and an
//! upload can take minutes. The uploader is therefore a process-wide thread
//! that the node points at, not something the node owns: a rebuild during an
//! upload neither interrupts it nor lets a second thread pick the same file
//! up and send it twice. What the node owns is the rows it has collected and
//! not yet written, and it writes those out when it is dropped.

use common::{Packet, Result};
use pipeline::event::Decoded;
use pipeline::node::{NodeCtx, PortSpec, Simple};
use pipeline::port::{Payload, PortKind, StreamSpec};
use std::collections::HashMap;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex, OnceLock};
use std::time::{Duration, Instant};

pub use survey::Account;

/// A spool file is closed once it holds this many rows. Small enough that a
/// failed upload costs one short retry, large enough that a busy band is not
/// a request a second.
const ROWS_PER_FILE: usize = 2_000;

/// Or once it is this old, so a quiet evening's rows are not still sitting in
/// a part file when the laptop is closed.
const MAX_AGE: Duration = Duration::from_secs(300);

/// How often the uploader looks for something to send.
const POLL: Duration = Duration::from_secs(10);

/// After a failure, how long to wait before trying the same file again, and
/// the ceiling that backoff climbs to. A refused token fails every time and
/// must not become a request every ten seconds.
const RETRY: Duration = Duration::from_secs(30);
const RETRY_MAX: Duration = Duration::from_secs(15 * 60);

/// What the uploader is doing, for the interface to draw.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct WigleStatus {
    /// Whether an API name and token have been set.
    pub configured: bool,
    /// Whether the rows may be licensed on by WiGLE.
    pub donate: bool,
    /// Files written and not yet accepted, and the rows in them.
    pub queued_files: u64,
    pub queued_rows: u64,
    /// Rows collected since the receiver started but not yet in a file.
    pub pending_rows: u64,
    pub sent_files: u64,
    pub sent_rows: u64,
    /// The last transaction WiGLE handed back, which is how an operator finds
    /// the upload on their account page.
    pub transaction: Option<String>,
    /// Why the last attempt failed, when it did.
    pub error: Option<String>,
    pub spool: PathBuf,
}

/// The thread that sends files, and everything it reports.
pub struct Uploader {
    dir: Mutex<PathBuf>,
    account: Mutex<Option<Account>>,
    sent_files: AtomicU64,
    sent_rows: AtomicU64,
    error: Mutex<Option<String>>,
    transaction: Mutex<Option<String>>,
    started: AtomicBool,
}

impl Uploader {
    /// An uploader with no thread behind it, which is what a test wants: the
    /// rows and the spool are testable, the network is not.
    pub fn inert(dir: &Path) -> Arc<Self> {
        Arc::new(Uploader {
            dir: Mutex::new(dir.to_path_buf()),
            account: Mutex::new(None),
            sent_files: AtomicU64::new(0),
            sent_rows: AtomicU64::new(0),
            error: Mutex::new(None),
            transaction: Mutex::new(None),
            started: AtomicBool::new(false),
        })
    }

    /// The one uploader, started the first time a node asks for it.
    pub fn shared(dir: &Path) -> Arc<Self> {
        static UPLOADER: OnceLock<Arc<Uploader>> = OnceLock::new();
        let up = UPLOADER.get_or_init(|| Uploader::inert(dir));
        up.set_dir(dir);
        up.start();
        up.clone()
    }

    fn start(self: &Arc<Self>) {
        if self.started.swap(true, Ordering::SeqCst) {
            return;
        }
        let up = self.clone();
        let _ = std::thread::Builder::new()
            .name("wigle-upload".into())
            .spawn(move || up.run());
    }

    pub fn set_account(&self, account: Option<Account>) {
        if let Ok(mut a) = self.account.lock() {
            *a = account.filter(Account::is_complete);
        }
    }

    pub fn account(&self) -> Option<Account> {
        self.account.lock().ok().and_then(|a| a.clone())
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

    pub fn status(&self) -> WigleStatus {
        let account = self.account();
        let (queued_files, queued_rows) = queued(&self.dir());
        WigleStatus {
            configured: account.is_some(),
            donate: account.is_some_and(|a| a.donate),
            queued_files,
            queued_rows,
            pending_rows: 0,
            sent_files: self.sent_files.load(Ordering::Relaxed),
            sent_rows: self.sent_rows.load(Ordering::Relaxed),
            transaction: self.transaction.lock().ok().and_then(|t| t.clone()),
            error: self.error.lock().ok().and_then(|e| e.clone()),
            spool: self.dir(),
        }
    }

    fn run(&self) {
        let mut wait = RETRY;
        loop {
            std::thread::sleep(POLL);
            let Some(account) = self.account() else { continue };
            let Some(path) = oldest(&self.dir()) else { continue };
            match std::fs::read(&path) {
                Ok(bytes) => {
                    let rows = rows_in(&path);
                    let name = path
                        .file_name()
                        .map(|n| n.to_string_lossy().to_string())
                        .unwrap_or_else(|| "waveshark.csv".into());
                    match survey::wigle::upload(&account, &name, bytes) {
                        Ok(receipt) => {
                            // Deleted only now: a file that was sent but not
                            // acknowledged is one this will send again, which
                            // is the failure worth having.
                            let _ = std::fs::remove_file(&path);
                            self.sent_files.fetch_add(1, Ordering::Relaxed);
                            self.sent_rows.fetch_add(rows, Ordering::Relaxed);
                            self.say(None);
                            if let (Ok(mut t), Some(id)) =
                                (self.transaction.lock(), receipt.transaction)
                            {
                                *t = Some(id);
                            }
                            wait = RETRY;
                        }
                        Err(e) => {
                            self.say(Some(e));
                            std::thread::sleep(wait);
                            wait = (wait * 2).min(RETRY_MAX);
                        }
                    }
                }
                Err(e) => self.say(Some(format!("{}: {e}", path.display()))),
            }
        }
    }

    fn say(&self, msg: Option<String>) {
        if let Ok(mut e) = self.error.lock() {
            *e = msg;
        }
    }
}

/// How many rows a spool file holds, taken from its name.
///
/// In the name rather than counted out of the file, because what is waiting
/// is drawn at the display's rate and reading every spooled file to answer
/// would be megabytes a second for two numbers.
fn rows_in(path: &Path) -> u64 {
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
        .filter(|p| p.extension().is_some_and(|e| e == "csv"))
        .collect();
    files.sort();
    files
}

fn oldest(dir: &Path) -> Option<PathBuf> {
    spooled(dir).into_iter().next()
}

/// How much is waiting, counted off the files themselves so a restart reports
/// what is really there rather than what this run happens to remember.
fn queued(dir: &Path) -> (u64, u64) {
    let files = spooled(dir);
    (files.len() as u64, files.iter().map(|p| rows_in(p)).sum())
}

/// The bus consumer that turns decodes into WiGLE rows.
pub struct WigleNode {
    up: Arc<Uploader>,
    /// Where this node's rows are written. The uploader has its own copy; a
    /// node writing to a directory nobody is reading is a test, not a fault.
    spool: PathBuf,
    /// Where the receiver is. Without one there are no rows: the format has
    /// no way to say "heard, position unknown".
    station: Option<gps::Fix>,
    /// Rows collected and not yet written to a file.
    pending: Vec<String>,
    /// The last sighting written per device, for the same thinning the survey
    /// does: a beacon advertising ten times a second is one row, not
    /// thirty-six thousand.
    last: HashMap<(String, String), survey::Sighting>,
    opened: Instant,
    seq: u64,
    /// What was last counted in the spool, and when. The interface asks for
    /// this at the display's rate and the answer is a directory listing.
    counted: std::cell::RefCell<(Option<Instant>, (u64, u64))>,
}

impl Default for WigleNode {
    fn default() -> Self {
        Self::new(default_spool_dir())
    }
}

impl WigleNode {
    pub fn new(spool: PathBuf) -> Self {
        Self::with(Uploader::shared(&spool), spool)
    }

    /// A node on an uploader somebody else made, which is how a test runs the
    /// whole path without a thread trying to reach wigle.net.
    pub fn with(up: Arc<Uploader>, spool: PathBuf) -> Self {
        Self {
            up,
            spool,
            station: None,
            pending: Vec::new(),
            last: HashMap::new(),
            opened: Instant::now(),
            seq: 0,
            counted: std::cell::RefCell::new((None, (0, 0))),
        }
    }

    /// Start or stop feeding: an account, or `None` to collect nothing.
    ///
    /// Rows are only collected while there is somewhere for them to go.
    /// Spooling for an operator who never set a token is a directory that
    /// fills up with files nobody will ever send.
    pub fn set_account(&mut self, account: Option<Account>) {
        self.up.set_account(account);
        if self.up.account().is_none() {
            self.pending.clear();
        }
    }

    pub fn is_on(&self) -> bool {
        self.up.account().is_some()
    }

    pub fn set_station(&mut self, at: Option<gps::Fix>) {
        self.station = at;
    }

    pub fn status(&self) -> WigleStatus {
        let (queued_files, queued_rows) = self.queued();
        WigleStatus {
            pending_rows: self.pending.len() as u64,
            queued_files,
            queued_rows,
            spool: self.spool.clone(),
            ..self.up.status()
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
        // The row count is part of the name so that what is waiting can be
        // reported from a directory listing.
        let name = format!("waveshark-{}-{:04}-{}.csv", now_s(), self.seq, self.pending.len());
        let mut text = survey::wigle::header();
        for row in self.pending.drain(..) {
            text.push_str(&row);
            text.push('\n');
        }
        // Written whole and then moved into place, so the uploader never
        // reads a file that is still being written.
        let part = dir.join(format!("{name}.part"));
        if std::fs::write(&part, text).is_ok() {
            let _ = std::fs::rename(&part, dir.join(name));
        }
        // The count is stale the moment a file lands.
        self.counted.borrow_mut().0 = None;
        self.opened = Instant::now();
    }

    fn sighting(&self, p: &Packet, d: &Decoded) -> survey::Sighting {
        let fix = self.station;
        survey::Sighting {
            at_us: p.at_us,
            lat: fix.map(|f| f.lat),
            lon: fix.map(|f| f.lon),
            alt_m: fix.and_then(|f| f.alt_m),
            accuracy_m: fix.and_then(|f| f.accuracy_m()),
            rssi_dbfs: d.rssi_dbfs.or(p.rssi_dbfs().is_finite().then_some(p.rssi_dbfs())),
            snr_db: d.snr_db.or(p.snr_db().is_finite().then_some(p.snr_db())),
            center_hz: d.center.0,
        }
    }
}

impl Drop for WigleNode {
    fn drop(&mut self) {
        self.flush();
    }
}

impl Simple for WigleNode {
    fn name(&self) -> &str {
        "wigle"
    }

    fn is_sink(&self) -> bool {
        true
    }

    fn negotiate(&mut self, i: &PortSpec) -> Result<StreamSpec> {
        if i.spec.kind != PortKind::Packets {
            return Err(common::Error::other("wigle reads the packet bus"));
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
                // Everything else a survey records, an aircraft, a pager, a
                // tyre sensor, has no type in this format and is not filed
                // under one that happens to fit.
                if survey::wigle::kind(&protocol).is_none() {
                    break;
                }
                let s = self.sighting(p, d);
                let key = (protocol.clone(), ident.clone());
                let fresh = match self.last.get(&key) {
                    None => true,
                    Some(prev) => survey::worth_keeping(prev, &s),
                };
                if !fresh {
                    break;
                }
                let row = survey::wigle::row(
                    &protocol,
                    &ident,
                    crate::survey_nodes::name_of(d).as_deref(),
                    crate::survey_nodes::vendor_of(d).as_deref(),
                    &s,
                );
                if let Some(row) = row {
                    self.last.insert(key, s);
                    self.pending.push(row);
                }
                break;
            }
        }
        if self.pending.len() >= ROWS_PER_FILE || self.opened.elapsed() >= MAX_AGE {
            self.flush();
        }
        Ok(())
    }
}

/// `$XDG_DATA_HOME/waveshark/wigle`, beside the packet log and the survey.
pub fn default_spool_dir() -> PathBuf {
    std::env::var_os("XDG_DATA_HOME")
        .map(PathBuf::from)
        .or_else(|| std::env::var_os("HOME").map(|h| PathBuf::from(h).join(".local/share")))
        .unwrap_or_else(std::env::temp_dir)
        .join("waveshark/wigle")
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
    use common::Hz;

    fn packet(bytes: Vec<u8>, center_hz: u64) -> Packet {
        Packet::of_frame(
            1_000_000,
            2_000_000,
            common::Frame::measured(bytes, -46.0, 20.0).at(center_hz),
        )
    }

    /// Through the protocols, the way the graph runs it.
    fn run(node: &mut WigleNode, packets: Vec<Packet>) {
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
            "waveshark-wigle-{}-{:?}",
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

    fn node_with_account(dir: PathBuf) -> WigleNode {
        let mut n = WigleNode::with(Uploader::inert(&dir), dir);
        n.set_account(Some(Account {
            name: "AID0000".into(),
            token: "token".into(),
            donate: false,
        }));
        n.set_station(Some(gps::Fix { lat: 53.6369, lon: -6.6528, ..Default::default() }));
        n
    }

    /// The whole path, short of the network: an advertisement off the bus
    /// becomes a spooled file with one row in it.
    #[test]
    fn an_advertisement_becomes_a_row_in_a_spool_file() {
        let dir = spool();
        let mut node = node_with_account(dir.clone());
        run(&mut node, vec![advertisement()]);
        assert_eq!(node.status().pending_rows, 1);
        node.flush();
        let files = spooled(&dir);
        assert_eq!(files.len(), 1, "expected one spool file, got {files:?}");
        let text = std::fs::read_to_string(&files[0]).unwrap();
        let mut lines = text.lines();
        assert!(lines.next().unwrap().starts_with("WigleWifi-1.6,"));
        lines.next();
        let row = lines.next().expect("a row");
        assert!(row.starts_with("E8:31:CD:0A:F5:3A,"), "{row}");
        assert!(row.ends_with(",BLE"), "{row}");
        assert_eq!(queued(&dir), (1, 1));
        let _ = std::fs::remove_dir_all(&dir);
    }

    /// Without an account nothing is collected: a spool that fills up for an
    /// operator who never set a token is a disc leak, not a queue.
    #[test]
    fn nothing_is_collected_without_an_account() {
        let dir = spool();
        let mut node = WigleNode::with(Uploader::inert(&dir), dir.clone());
        node.set_account(None);
        node.set_station(Some(gps::Fix { lat: 53.6, lon: -6.6, ..Default::default() }));
        run(&mut node, vec![advertisement()]);
        assert_eq!(node.status().pending_rows, 0);
        assert!(!node.is_on());
    }

    /// The format has no way to say "heard, position unknown", so a receiver
    /// with no fix feeds nothing rather than feeding zeroes.
    #[test]
    fn without_a_fix_there_is_nothing_to_upload() {
        let dir = spool();
        let mut node = node_with_account(dir);
        node.set_station(None);
        run(&mut node, vec![advertisement()]);
        assert_eq!(node.status().pending_rows, 0);
    }

    /// The same beacon heard again from the same place is the same row.
    #[test]
    fn a_stationary_repeat_is_thinned_away() {
        let dir = spool();
        let mut node = node_with_account(dir);
        run(&mut node, vec![advertisement(), advertisement(), advertisement()]);
        assert_eq!(node.status().pending_rows, 1);
    }

    /// A rebuild drops the node, and what it had collected has to survive
    /// that: the next graph's uploader finds it on disc.
    #[test]
    fn dropping_the_node_writes_what_it_had() {
        let dir = spool();
        {
            let mut node = node_with_account(dir.clone());
            run(&mut node, vec![advertisement()]);
        }
        assert_eq!(spooled(&dir).len(), 1);
        let _ = std::fs::remove_dir_all(&dir);
    }
}
