//! Walking the dial past the span, and keeping what was heard on the way.
//!
//! Everything else in the receiver reads the one span the radio is sampling.
//! This steps the dial across a band, waits, and reads the packet bus to find
//! out whether the step was worth anything, which makes it a consumer of the
//! bus like the survey and the tracker: what counts as a signal is whatever
//! the detector opened and the protocols claimed, so a step's result is the
//! same question the packet list answers and not a second energy search.
//!
//! Three things the walk has to decide, and the answers are here rather than
//! spread through the receiver:
//!
//! - **It does not move the dial itself.** It asks, with
//!   [`pipeline::Request::Retune`], because only the thing holding the device
//!   can retune it, and the ask is what the receiver already has a channel
//!   for.
//! - **A hit is keyed by identity where a decode names one, and by frequency
//!   where none does.** A frequency alone is wrong for a transmitter that
//!   hops and is the only key available for an unclaimed carrier, so both
//!   exist and the decoder decides which applies.
//! - **What to do with a hit is the operator's.** [`OnHit::Hold`] stops the
//!   walk on the step so it can be listened to; [`OnHit::Log`] records it and
//!   carries on after [`Linger`].
//!
//! A step is called busy by a [`Lock`], not by the first packet: one decode
//! can be a CRC-less protocol reading noise, and a burst arriving as the
//! dial settles belongs to the step before. Mayhem's Recon draws the same
//! distinction and offers the same two ways of counting.

use common::Result;
use common::packet::Packet;
use pipeline::event::{Event, Request};
use pipeline::node::{NodeCtx, PortSpec, Simple};
use pipeline::param::{Param, ParamValue};
use pipeline::port::{Payload, PortKind, StreamSpec};
use pipeline::registry::{Category, Settings, SettingsExt, StageDesc};

/// How long after asking for a retune the walk ignores the bus.
///
/// A retune is held until the receiver can afford one, which is 120 ms
/// (`MIN_TUNE_GAP` in `crates/app/src/radio.rs`), and the graph is rebuilt
/// after it, so packets keep arriving from the old dial for a while after the
/// ask. At twice the tuning gap a burst decoded before the dial moved is
/// still filed under the frequency it was heard at, but the step it belongs
/// to is the previous one and the walk does not hold on it.
const SETTLE_S: f64 = 0.24;

/// How many packets of a step make it busy, and whether they have to arrive
/// one after another.
///
/// [`Lock::Continuous`] wants the packets in consecutive blocks, so a gap
/// starts the count again: fast, and it drops a transmitter heard through bad
/// reception. [`Lock::Sparse`] wants them any time before the step's listen
/// runs out, which survives the gap and takes the whole listen to say no.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Lock {
    #[default]
    Sparse,
    Continuous,
}

impl Lock {
    pub fn label(self) -> &'static str {
        match self {
            Lock::Sparse => "sparse",
            Lock::Continuous => "continuous",
        }
    }

    pub fn parse(s: &str) -> Option<Self> {
        match s.trim().to_ascii_lowercase().as_str() {
            "sparse" | "any" => Some(Lock::Sparse),
            "continuous" | "consecutive" => Some(Lock::Continuous),
            _ => None,
        }
    }
}

/// How long a logging walk stays on a step it has locked, which the operator
/// sets as one signed number of seconds.
///
/// Zero is what makes a fast map of a band: note it and move on. A positive
/// number is a fixed stay. A negative one stays until that long passes with
/// nothing further heard, each packet starting the count again, which is the
/// only one of the three that follows a conversation.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub enum Linger {
    #[default]
    Now,
    For(f64),
    UntilQuiet(f64),
}

impl Linger {
    pub fn seconds(self) -> f64 {
        match self {
            Linger::Now => 0.0,
            Linger::For(s) => s,
            Linger::UntilQuiet(s) => -s,
        }
    }

    fn of_seconds(s: f64) -> Self {
        match s {
            s if s > 0.0 => Linger::For(s),
            s if s < 0.0 => Linger::UntilQuiet(-s),
            _ => Linger::Now,
        }
    }
}

/// Closest two hits can be and still be the same transmitter, where neither
/// carries an identity.
///
/// A narrowband channel is 12.5 kHz, so two carriers within half of one are
/// the same one measured twice: the detector's centre moves by a few
/// kilohertz between bursts as the burst's own shape changes.
const SAME_HZ: f64 = 6_250.0;

/// What the walk does when a step turns something up.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum OnHit {
    /// Stay on the step, so it can be listened to. The walk resumes when the
    /// operator says so.
    #[default]
    Hold,
    /// Write it down and keep walking.
    Log,
}

impl OnHit {
    pub fn label(self) -> &'static str {
        match self {
            OnHit::Hold => "hold",
            OnHit::Log => "log",
        }
    }

    pub fn parse(s: &str) -> Option<Self> {
        match s.trim().to_ascii_lowercase().as_str() {
            "hold" | "dwell" => Some(OnHit::Hold),
            "log" | "move" => Some(OnHit::Log),
            _ => None,
        }
    }
}

/// What a hit is remembered by.
///
/// The identity a decoder named, where there is one: a transmitter that hops
/// is one transmitter, and ignoring it by the frequency it happened to be on
/// ignores a channel rather than a device. Where nothing named it, all the
/// walk has is where it was, which is enough to stop coming back to the same
/// carrier.
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum Key {
    Identity { space: String, id: String },
    Frequency(u64),
}

impl Key {
    /// How an ignore list writes it: `ble:AA:BB:CC:DD:EE:FF`, or `433.920M`.
    pub fn label(&self) -> String {
        match self {
            Key::Identity { space, id } => format!("{space}:{id}"),
            Key::Frequency(hz) => format!("{:.4}M", *hz as f64 / 1e6),
        }
    }

    /// Read one back, as an operator's ignore list holds it.
    pub fn parse(s: &str) -> Option<Self> {
        let s = s.trim();
        if s.is_empty() {
            return None;
        }
        if let Some(Ok(f)) = s.strip_suffix(['M', 'm']).map(|mhz| mhz.trim().parse::<f64>()) {
            return Some(Key::Frequency((f * 1e6).round() as u64));
        }
        let (space, id) = s.split_once(':')?;
        match space.is_empty() || id.is_empty() {
            true => None,
            false => Some(Key::Identity { space: space.into(), id: id.into() }),
        }
    }

    /// Whether this key covers a hit at that key: the same identity, or a
    /// frequency within half a narrowband channel.
    fn covers(&self, other: &Key) -> bool {
        match (self, other) {
            (Key::Frequency(a), Key::Frequency(b)) => (*a as f64 - *b as f64).abs() <= SAME_HZ,
            (a, b) => a == b,
        }
    }
}

/// One transmitter the walk turned up.
#[derive(Clone, Debug, PartialEq)]
pub struct Found {
    pub key: Key,
    /// Where it was heard, which is the transmitter's own frequency and not
    /// the step's centre.
    pub center_hz: u64,
    pub bandwidth_hz: u32,
    pub snr_db: f32,
    pub rssi_dbfs: f32,
    /// What claimed it, where anything did.
    pub protocol: Option<String>,
    /// When it was last heard, in microseconds since the epoch, as the
    /// packet carried it.
    pub at_us: u64,
    /// How many packets of it the walk has seen.
    pub heard: u32,
    /// The step it was first heard on.
    pub step_hz: u64,
}

/// What the walk is doing, for whatever is showing it.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct ScanStatus {
    pub running: bool,
    pub holding: bool,
    /// Where the walk believes the dial is.
    pub center_hz: Option<f64>,
    pub steps: u64,
    /// Centres one pass over the band takes.
    pub stops: u64,
    pub found: Vec<Found>,
    pub ignore: Vec<Key>,
}

pub struct BandScanNode {
    running: bool,
    lo_hz: f64,
    hi_hz: f64,
    step_hz: f64,
    dwell_s: f64,
    on_hit: OnHit,
    linger: Linger,
    lock: Lock,
    locks: u32,
    /// Where the walk believes the dial is. Zero until it has asked for
    /// anything, which is what makes the first block of a run a step.
    center_hz: f64,
    dwelt_s: f64,
    settling_s: f64,
    /// Packets heard on this step, and how many blocks in a row have carried
    /// one: the sparse count and the continuous one.
    seen: u32,
    run: u32,
    /// Whether this step has turned up a transmitter the run had not heard.
    fresh: bool,
    /// Seconds since the step locked, and since the last packet on it.
    locked_s: Option<f64>,
    quiet_s: f64,
    /// Stopped on a hit, waiting to be let go.
    holding: bool,
    found: Vec<Found>,
    ignore: Vec<Key>,
    steps: u64,
}

impl Default for BandScanNode {
    fn default() -> Self {
        Self::new()
    }
}

impl BandScanNode {
    pub fn new() -> Self {
        Self {
            running: false,
            lo_hz: 0.0,
            hi_hz: 0.0,
            step_hz: 0.0,
            dwell_s: 2.0,
            on_hit: OnHit::default(),
            linger: Linger::default(),
            lock: Lock::default(),
            locks: 1,
            center_hz: 0.0,
            dwelt_s: 0.0,
            settling_s: 0.0,
            seen: 0,
            run: 0,
            fresh: false,
            locked_s: None,
            quiet_s: 0.0,
            holding: false,
            found: Vec::new(),
            ignore: Vec::new(),
            steps: 0,
        }
    }

    /// Where the walk believes the dial is, and nothing if it has not asked
    /// for anything yet.
    pub fn center_hz(&self) -> Option<f64> {
        (self.center_hz > 0.0).then_some(self.center_hz)
    }

    pub fn is_running(&self) -> bool {
        self.running
    }

    /// Whether the walk has stopped on something.
    pub fn holding(&self) -> bool {
        self.holding
    }

    /// What has been heard since the walk started, newest last.
    pub fn found(&self) -> &[Found] {
        &self.found
    }

    /// What the operator has told the walk not to stop for.
    pub fn ignored(&self) -> &[Key] {
        &self.ignore
    }

    /// Steps taken, which is retunes asked for.
    pub fn steps(&self) -> u64 {
        self.steps
    }

    /// Everything a pane showing the walk needs, in one read.
    pub fn status(&self) -> ScanStatus {
        ScanStatus {
            running: self.running,
            holding: self.holding,
            center_hz: self.center_hz(),
            steps: self.steps,
            stops: self.stops(),
            found: self.found.clone(),
            ignore: self.ignore.clone(),
        }
    }

    /// How many centres one pass over the band takes.
    fn stops(&self) -> u64 {
        let width = (self.hi_hz - self.lo_hz).max(0.0);
        match self.step_hz > 0.0 {
            true => (width / self.step_hz).floor().max(1.0) as u64,
            false => 1,
        }
    }

    fn ignored_key(&self, k: &Key) -> bool {
        self.ignore.iter().any(|i| i.covers(k))
    }

    /// The centre of the first step, which is a step's width inside the low
    /// edge: the band is covered by the spans, not by the centres.
    fn first(&self) -> f64 {
        self.lo_hz + self.step_hz / 2.0
    }

    /// Whether the step the walk is on has been called busy.
    pub fn locked(&self) -> bool {
        self.locked_s.is_some()
    }

    /// Ask for the next centre, wrapping at the top edge.
    fn step(&mut self, c: &mut NodeCtx<'_>) {
        let next = match self.center_hz > 0.0 {
            false => self.first(),
            true => {
                let n = self.center_hz + self.step_hz;
                match n + self.step_hz / 2.0 > self.hi_hz {
                    true => self.first(),
                    false => n,
                }
            }
        };
        self.center_hz = next;
        self.dwelt_s = 0.0;
        self.settling_s = SETTLE_S;
        self.seen = 0;
        self.run = 0;
        self.fresh = false;
        self.locked_s = None;
        self.quiet_s = 0.0;
        self.steps += 1;
        c.emit(Event::Request(Request::Retune { center_hz: next }));
    }

    /// File one packet under whatever names it, and say what it was worth.
    fn hit(&mut self, p: &Packet) -> Heard {
        let named = p
            .subject()
            .map(|who| Key::Identity { space: who.space.to_string(), id: who.id.to_string() });
        let key = named.unwrap_or(Key::Frequency(p.center_hz()));
        if self.ignored_key(&key) {
            return Heard::Ignored;
        }
        let protocol = p.innermost().map(|l| l.id.to_string());
        if let Some(f) = self.found.iter_mut().find(|f| f.key.covers(&key)) {
            f.heard += 1;
            f.at_us = p.carrier.at_us;
            f.snr_db = p.carrier.snr_db;
            f.rssi_dbfs = p.carrier.rssi_dbfs;
            if f.protocol.is_none() {
                f.protocol = protocol;
            }
            // A transmitter heard again on a step already walked is not a
            // reason to stop a second time: what the operator has not seen
            // is what is worth holding for. It still counts towards the
            // lock, because it is the step being busy that is in question.
            return Heard::Again;
        }
        self.found.push(Found {
            key,
            center_hz: p.center_hz(),
            bandwidth_hz: p.carrier.bandwidth_hz,
            snr_db: p.carrier.snr_db,
            rssi_dbfs: p.carrier.rssi_dbfs,
            protocol,
            at_us: p.carrier.at_us,
            heard: 1,
            step_hz: self.center_hz as u64,
        });
        Heard::New
    }

    /// Let a held walk go on, without forgetting what it found.
    pub fn resume(&mut self) {
        self.holding = false;
        self.dwelt_s = self.dwell_s;
        // The step has had its hearing: leaving the lock standing would hold
        // the walk again on the next block that carries a packet.
        self.locked_s = None;
        self.fresh = false;
        self.seen = 0;
        self.run = 0;
    }

    /// Stop coming back to this one.
    pub fn ignore(&mut self, key: Key) {
        if !self.ignore.contains(&key) {
            self.ignore.push(key);
        }
        self.found.retain(|f| !key_ignored(&self.ignore, &f.key));
    }

    fn set_ignore_list(&mut self, list: &str) {
        self.ignore = list.split(',').filter_map(Key::parse).collect();
        self.found.retain(|f| !key_ignored(&self.ignore, &f.key));
    }

    fn ignore_list(&self) -> String {
        self.ignore.iter().map(Key::label).collect::<Vec<_>>().join(",")
    }
}

/// What one packet on a step was worth to the walk.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Heard {
    /// On the operator's ignore list.
    Ignored,
    /// A transmitter this run has already listed.
    Again,
    /// One the operator has not seen yet.
    New,
}

fn key_ignored(list: &[Key], k: &Key) -> bool {
    list.iter().any(|i| i.covers(k))
}

impl Simple for BandScanNode {
    fn name(&self) -> &str {
        DESC.name
    }

    fn is_sink(&self) -> bool {
        true
    }

    fn negotiate(&mut self, i: &PortSpec) -> Result<StreamSpec> {
        if i.spec.kind != PortKind::Packets {
            return Err(common::Error::other("the band scan reads the packet bus"));
        }
        // A step is as wide as the span the receiver is sampling, unless the
        // operator asked for something else: stepping by more leaves gaps
        // nothing was ever tuned to.
        if self.step_hz <= 0.0 && i.spec.bandwidth > 0.0 {
            self.step_hz = i.spec.bandwidth;
        }
        Ok(i.spec)
    }

    fn params(&self) -> Vec<Param> {
        vec![
            Param::bool("running", self.running).label("Walk the band"),
            Param::float("lo_hz", self.lo_hz, 0.0..=6e9).unit("Hz").label("From"),
            Param::float("hi_hz", self.hi_hz, 0.0..=6e9).unit("Hz").label("To"),
            Param::float("step_hz", self.step_hz, 0.0..=1e8).unit("Hz").label("Step"),
            Param::float("dwell_s", self.dwell_s, 0.1..=60.0).unit("s").label("Dwell"),
            Param::int("locks", self.locks as i64, 1..=16).label("Packets to lock"),
            Param::choice(
                "lock",
                match self.lock {
                    Lock::Sparse => 0,
                    Lock::Continuous => 1,
                },
                vec!["sparse".into(), "continuous".into()],
            )
            .label("Counted"),
            Param::float("linger_s", self.linger.seconds(), -60.0..=60.0).unit("s").label("Linger"),
            Param::choice(
                "on_hit",
                match self.on_hit {
                    OnHit::Hold => 0,
                    OnHit::Log => 1,
                },
                vec!["hold".into(), "log".into()],
            )
            .label("On a hit"),
            // Settable, because clearing it is how a held walk is let go:
            // there is no such thing as a parameter that is an action.
            Param::bool("holding", self.holding).label("Held on a hit"),
            Param::text("ignore", self.ignore_list()).label("Ignore"),
        ]
    }

    fn set_param(&mut self, name: &str, v: ParamValue) -> Result<()> {
        match name {
            "running" => {
                let want = v.as_bool().unwrap_or(false);
                // Starting is starting over: the found list is what this run
                // turned up, and a walk resumed after an hour off would
                // otherwise show what the last one heard.
                if want && !self.running {
                    self.found.clear();
                    self.center_hz = 0.0;
                    self.steps = 0;
                }
                self.running = want;
                self.holding = false;
                self.dwelt_s = 0.0;
                self.settling_s = 0.0;
                self.seen = 0;
                self.run = 0;
                self.fresh = false;
                self.locked_s = None;
                self.quiet_s = 0.0;
            }
            "lo_hz" => self.lo_hz = v.as_f64().unwrap_or(self.lo_hz),
            "hi_hz" => self.hi_hz = v.as_f64().unwrap_or(self.hi_hz),
            "step_hz" => self.step_hz = v.as_f64().unwrap_or(self.step_hz).max(0.0),
            "dwell_s" => self.dwell_s = v.as_f64().unwrap_or(self.dwell_s).max(0.1),
            "locks" => {
                self.locks = v.as_f64().unwrap_or(self.locks as f64).round().clamp(1.0, 16.0) as u32
            }
            // A choice arrives as an index from the chain view and as a
            // label from a patch, and both have to land on the same variant.
            "lock" => {
                self.lock = match &v {
                    ParamValue::Int(_) | ParamValue::Choice(_) => match v.as_i64() {
                        Some(0) => Lock::Sparse,
                        _ => Lock::Continuous,
                    },
                    _ => v.as_str().and_then(Lock::parse).unwrap_or(self.lock),
                }
            }
            "linger_s" => {
                self.linger = Linger::of_seconds(
                    v.as_f64().unwrap_or(self.linger.seconds()).clamp(-60.0, 60.0),
                )
            }
            "on_hit" => {
                self.on_hit = match &v {
                    ParamValue::Int(_) | ParamValue::Choice(_) => match v.as_i64() {
                        Some(0) => OnHit::Hold,
                        _ => OnHit::Log,
                    },
                    _ => v.as_str().and_then(OnHit::parse).unwrap_or(self.on_hit),
                }
            }
            "holding" => match v.as_bool().unwrap_or(false) {
                true => self.holding = true,
                false => self.resume(),
            },
            "ignore" => self.set_ignore_list(v.as_str().unwrap_or_default()),
            _ => return Err(common::Error::other(format!("no parameter {name}"))),
        }
        Ok(())
    }

    fn readings(&self) -> Vec<(String, String)> {
        let mut out = vec![("found".into(), self.found.len().to_string())];
        if let Some(c) = self.center_hz() {
            out.push(("at".into(), format!("{:.4} MHz", c / 1e6)));
        }
        if self.running {
            out.push(("steps".into(), self.steps.to_string()));
            out.push(("of".into(), self.stops().to_string()));
        }
        if self.holding {
            out.push(("holding".into(), "yes".into()));
        } else if self.locked() {
            out.push(("locked".into(), "yes".into()));
        }
        out
    }

    fn process(&mut self, i: &Payload, _o: &mut Payload, c: &mut NodeCtx<'_>) -> Result<()> {
        if !self.running || self.hi_hz - self.lo_hz <= 0.0 || self.step_hz <= 0.0 {
            return Ok(());
        }
        if self.center_hz == 0.0 {
            self.step(c);
            return Ok(());
        }
        if self.settling_s > 0.0 {
            // What arrives while the dial is still moving was heard on the
            // step before this one, which has already had its chance to hold
            // the walk. The dwell starts after it: a dwell is how long the
            // step is listened to, not how long since it was asked for.
            self.settling_s -= c.block_seconds;
            return Ok(());
        }
        self.dwelt_s += c.block_seconds;
        self.quiet_s += c.block_seconds;
        if let Some(since) = self.locked_s.as_mut() {
            *since += c.block_seconds;
        }
        let mut counted = 0u32;
        for p in i.as_packets().unwrap_or(&[]) {
            match self.hit(p) {
                Heard::Ignored => {}
                Heard::Again => counted += 1,
                Heard::New => {
                    counted += 1;
                    self.fresh = true;
                }
            }
        }
        match counted > 0 {
            // A continuous count is consecutive blocks carrying a packet, so
            // the gap it will not tolerate is one block of the graph rather
            // than a number of its own.
            true => {
                self.seen += counted;
                self.run += 1;
                self.quiet_s = 0.0;
            }
            false => self.run = 0,
        }
        let enough = match self.lock {
            Lock::Sparse => self.seen >= self.locks,
            Lock::Continuous => self.run >= self.locks,
        };
        if self.locked_s.is_none() && self.fresh && enough {
            self.locked_s = Some(0.0);
            if self.on_hit == OnHit::Hold {
                self.holding = true;
            }
        }
        if self.holding {
            return Ok(());
        }
        let go = match self.locked_s {
            Some(since) => match self.linger {
                Linger::Now => true,
                Linger::For(s) => since >= s,
                Linger::UntilQuiet(s) => self.quiet_s >= s,
            },
            None => self.dwelt_s >= self.dwell_s,
        };
        if go {
            self.step(c);
        }
        Ok(())
    }
}

pub const DESC: StageDesc = StageDesc {
    name: "band_scan",
    summary: "Walk the dial across a band and keep what was heard",
    category: Category::Sink,
    feeds_bus: false,
};

pub fn build(s: &Settings) -> Result<Box<dyn pipeline::node::Node>> {
    let mut n = BandScanNode::new();
    n.running = s.bool_or("running", false);
    n.lo_hz = s.f64_or("lo_hz", 0.0);
    n.hi_hz = s.f64_or("hi_hz", 0.0);
    n.step_hz = s.f64_or("step_hz", 0.0);
    n.dwell_s = s.f64_or("dwell_s", 2.0);
    n.locks = s.f64_or("locks", 1.0).round().clamp(1.0, 16.0) as u32;
    n.lock = Lock::parse(s.str_or("lock", "sparse")).unwrap_or_default();
    n.linger = Linger::of_seconds(s.f64_or("linger_s", 0.0).clamp(-60.0, 60.0));
    n.on_hit = OnHit::parse(s.str_or("on_hit", "hold")).unwrap_or_default();
    let ignore = s.str_or("ignore", "").to_string();
    n.set_ignore_list(&ignore);
    Ok(Box::new(n))
}

#[cfg(test)]
mod tests {
    use super::*;
    use common::{Frame, Hz, Identity};

    /// One block through the node, a tenth of a second of run time, with the
    /// packets that arrived during it.
    fn block(node: &mut BandScanNode, packets: Vec<Packet>) -> Vec<f64> {
        blocks(node, packets, 0.1)
    }

    fn blocks(node: &mut BandScanNode, packets: Vec<Packet>, seconds: f64) -> Vec<f64> {
        let mut s = StreamSpec::iq(2_400_000.0, Hz(433_000_000));
        s.kind = PortKind::Packets;
        s.bandwidth = 2_400_000.0;
        let ins = [PortSpec { spec: s, latency: 0 }];
        let (tags, mut events, mut new_tags) = (Vec::new(), Vec::new(), Vec::new());
        let mut out = Payload::Packets(Vec::new());
        let mut ctx = NodeCtx::new(0, &ins, &tags, &mut events, &mut new_tags);
        ctx.block_seconds = seconds;
        node.process(&Payload::Packets(packets), &mut out, &mut ctx).unwrap();
        events
            .iter()
            .filter_map(|e| match e {
                Event::Request(Request::Retune { center_hz }) => Some(*center_hz),
                _ => None,
            })
            .collect()
    }

    fn walking(lo_mhz: f64, hi_mhz: f64, step_mhz: f64, dwell_s: f64, on: OnHit) -> BandScanNode {
        let mut n = BandScanNode::new();
        n.lo_hz = lo_mhz * 1e6;
        n.hi_hz = hi_mhz * 1e6;
        n.step_hz = step_mhz * 1e6;
        n.dwell_s = dwell_s;
        n.on_hit = on;
        n.set_param("running", ParamValue::Bool(true)).unwrap();
        n
    }

    fn burst(center_hz: u64) -> Packet {
        let mut p = crate::measured(center_hz, 25_000, vec![0x01, 0x02, 0x03], -60.0, 14.0);
        p.carrier.at_us = 1_000_000;
        p
    }

    fn named(center_hz: u64, space: &'static str, id: &str) -> Packet {
        burst(center_hz).decoded(
            common::packet::Proto::new("ble", "adv")
                .by(common::packet::Entity::new(space, common::packet::Id::Text(id.to_string()))),
        )
    }

    /// An empty band is walked end to end and starts again, at the centres a
    /// step apart from the low edge inwards.
    #[test]
    fn a_quiet_band_is_walked_step_by_step_and_wraps() {
        // 430 to 440 MHz in 2 MHz steps is five stops: 431, 433, 435, 437,
        // 439. A sixth would centre at 441 and reach past the top edge.
        let mut n = walking(430.0, 440.0, 2.0, 1.0, OnHit::Hold);
        assert_eq!(n.stops(), 5);
        let mut asked: Vec<f64> = Vec::new();
        // 60 blocks of 0.1 s: enough for eleven steps at a second each plus
        // the settle after each.
        for _ in 0..60 {
            asked.extend(block(&mut n, Vec::new()));
        }
        assert_eq!(asked.len(), 5, "asked for {asked:?}");
        assert_eq!(
            asked,
            vec![431e6, 433e6, 435e6, 437e6, 439e6],
            "the walk did not cover the band in order"
        );
        assert_eq!(n.found().len(), 0, "a quiet band turned something up");
        // A sixth step wraps to the bottom rather than running off the top.
        for _ in 0..20 {
            asked.extend(block(&mut n, Vec::new()));
        }
        assert_eq!(asked.len(), 6);
        assert_eq!(asked[5], 431e6, "the walk did not wrap at the top edge");
        assert_eq!(n.steps(), 6);
    }

    /// Minutes of an empty band, which is what a walk over a dead band is:
    /// nothing found, and only the steps to show for it.
    #[test]
    fn minutes_of_nothing_find_nothing() {
        let mut n = walking(144.0, 146.0, 0.5, 2.0, OnHit::Hold);
        let mut steps = 0usize;
        // 120 seconds at 0.1 s a block.
        for _ in 0..1_200 {
            steps += block(&mut n, Vec::new()).len();
        }
        assert_eq!(n.found().len(), 0);
        assert!(!n.holding(), "a walk held on an empty band");
        // A step is the settle plus the dwell: 0.24 s of dial and rebuild
        // then 2.0 s listening, so 23 blocks of 0.1 s, and the first block of
        // the run is spent asking for the first centre.
        assert_eq!(steps, 53, "expected 53 steps in two minutes, got {steps}");
        assert_eq!(n.steps(), 53);
    }

    /// A hit stops the walk where the operator asked it to, and letting it go
    /// carries on from there.
    #[test]
    fn a_hit_holds_the_walk_until_it_is_let_go() {
        let mut n = walking(433.0, 435.0, 1.0, 1.0, OnHit::Hold);
        assert_eq!(block(&mut n, Vec::new()), vec![433.5e6]);
        // Nothing is attributed while the dial is still moving: 0.24 s of
        // settle is three blocks of a tenth of a second.
        for _ in 0..3 {
            assert_eq!(block(&mut n, vec![burst(433_920_000)]), Vec::<f64>::new());
        }
        assert_eq!(n.found().len(), 0, "a packet from the previous step was counted");
        assert!(block(&mut n, vec![burst(433_920_000)]).is_empty());
        assert!(n.holding(), "the walk did not hold on a hit");
        assert_eq!(n.found().len(), 1);
        assert_eq!(n.found()[0].center_hz, 433_920_000);
        assert_eq!(n.found()[0].heard, 1);
        assert_eq!(n.found()[0].step_hz, 433_500_000);
        // Held is held, however long the dwell runs on.
        for _ in 0..50 {
            assert!(block(&mut n, Vec::new()).is_empty());
        }
        assert_eq!(n.steps(), 1);
        n.resume();
        assert_eq!(block(&mut n, Vec::new()), vec![434.5e6]);
        assert!(!n.holding());
    }

    /// The other policy: write it down and keep going, which with the
    /// default linger of zero is the same block it heard it on.
    #[test]
    fn logging_a_hit_keeps_the_walk_moving() {
        let mut n = walking(433.0, 435.0, 1.0, 1.0, OnHit::Log);
        assert_eq!(block(&mut n, Vec::new()), vec![433.5e6]);
        for _ in 0..3 {
            block(&mut n, Vec::new());
        }
        assert_eq!(block(&mut n, vec![burst(433_920_000)]), vec![434.5e6]);
        assert!(!n.holding());
        assert_eq!(n.found().len(), 1);
        assert_eq!(n.found()[0].heard, 1);
        // The same transmitter on the next step is one the operator has
        // seen, so it does not lock a second time: that step runs its dwell
        // out, which is the settle and ten blocks of a tenth of a second.
        let mut asked = Vec::new();
        for _ in 0..20 {
            asked.extend(block(&mut n, vec![burst(433_920_000)]));
        }
        assert_eq!(asked, vec![433.5e6], "the walk did not step on the dwell");
        assert_eq!(n.found().len(), 1, "one transmitter heard again is one row");
        // 21 blocks carried it, and the six spent settling after the two
        // steps are not attributed to the step being tuned to.
        assert_eq!(n.found()[0].heard, 15);
    }

    /// A step is busy when the packets asked for have arrived, and a lone
    /// packet is not enough where the operator asked for three: the same
    /// stream locks a sparse count and never locks a continuous one, because
    /// every other block is empty.
    #[test]
    fn a_gappy_transmitter_locks_a_sparse_count_and_not_a_continuous_one() {
        for (lock, want) in [(Lock::Sparse, true), (Lock::Continuous, false)] {
            let mut n = walking(433.0, 435.0, 1.0, 5.0, OnHit::Hold);
            n.lock = lock;
            n.locks = 3;
            block(&mut n, Vec::new());
            for _ in 0..3 {
                block(&mut n, Vec::new());
            }
            // Packets in every other block: eight blocks, four packets.
            for i in 0..8 {
                let p = match i % 2 {
                    0 => vec![burst(433_920_000)],
                    _ => Vec::new(),
                };
                block(&mut n, p);
            }
            assert_eq!(n.holding(), want, "{} counted it wrong", lock.label());
            assert_eq!(n.locked(), want);
            // Either way the transmitter is written down on the first packet.
            assert_eq!(n.found().len(), 1);
            assert_eq!(n.found()[0].heard, 4);
        }
    }

    /// Consecutive blocks do lock a continuous count, and three packets are
    /// three blocks: the first two leave it walking.
    #[test]
    fn a_continuous_count_locks_on_consecutive_blocks() {
        let mut n = walking(433.0, 435.0, 1.0, 5.0, OnHit::Hold);
        n.lock = Lock::Continuous;
        n.locks = 3;
        block(&mut n, Vec::new());
        for _ in 0..3 {
            block(&mut n, Vec::new());
        }
        for _ in 0..2 {
            block(&mut n, vec![burst(433_920_000)]);
        }
        assert!(!n.locked(), "two of three packets locked the step");
        block(&mut n, vec![burst(433_920_000)]);
        assert!(n.locked());
        assert!(n.holding());
    }

    /// A positive linger is a fixed stay after the lock, counted from the
    /// lock and not from the step.
    #[test]
    fn a_positive_linger_stays_for_that_long_and_then_steps() {
        let mut n = walking(433.0, 435.0, 1.0, 5.0, OnHit::Log);
        n.linger = Linger::For(0.5);
        block(&mut n, Vec::new());
        for _ in 0..3 {
            block(&mut n, Vec::new());
        }
        assert!(block(&mut n, vec![burst(433_920_000)]).is_empty());
        assert!(n.locked());
        // Half a second is five blocks of a tenth, and the fifth steps.
        for _ in 0..4 {
            assert!(block(&mut n, Vec::new()).is_empty());
        }
        assert_eq!(block(&mut n, Vec::new()), vec![434.5e6]);
    }

    /// A negative linger stays until that long passes with nothing further,
    /// and every packet starts the count again, so a transmitter talking
    /// every quarter second holds a walk set to half a second.
    #[test]
    fn a_negative_linger_stays_until_the_step_goes_quiet() {
        let mut n = walking(433.0, 435.0, 1.0, 5.0, OnHit::Log);
        n.linger = Linger::UntilQuiet(0.5);
        block(&mut n, Vec::new());
        for _ in 0..3 {
            block(&mut n, Vec::new());
        }
        let mut asked = Vec::new();
        // Four seconds of a packet every other block, which is past the five
        // second dwell's worth of blocks only because the lock replaces it.
        for i in 0..40 {
            let p = match i % 2 {
                0 => vec![burst(433_920_000)],
                _ => Vec::new(),
            };
            asked.extend(block(&mut n, p));
        }
        assert!(asked.is_empty(), "a talking transmitter did not hold the walk: {asked:?}");
        assert_eq!(n.found()[0].heard, 20);
        // Half a second of quiet from the last packet, which the block that
        // ended the loop has a tenth of already.
        for _ in 0..3 {
            assert!(block(&mut n, Vec::new()).is_empty());
        }
        assert_eq!(block(&mut n, Vec::new()), vec![434.5e6]);
    }

    /// The signed number the operator sets and the three things it means.
    #[test]
    fn the_linger_is_one_signed_number() {
        assert_eq!(Linger::of_seconds(0.0), Linger::Now);
        assert_eq!(Linger::of_seconds(2.5), Linger::For(2.5));
        assert_eq!(Linger::of_seconds(-2.5), Linger::UntilQuiet(2.5));
        assert_eq!(Linger::of_seconds(-2.5).seconds(), -2.5);
        assert_eq!(Linger::Now.seconds(), 0.0);
        let mut n = BandScanNode::new();
        n.set_param("linger_s", ParamValue::Float(-3.0)).unwrap();
        assert_eq!(n.linger, Linger::UntilQuiet(3.0));
        n.set_param("linger_s", ParamValue::Float(4.0)).unwrap();
        assert_eq!(n.linger, Linger::For(4.0));
        n.set_param("lock", ParamValue::Text("continuous".into())).unwrap();
        assert_eq!(n.lock, Lock::Continuous);
        // The chain view sends the position in the list rather than the word.
        n.set_param("lock", ParamValue::Choice(0)).unwrap();
        assert_eq!(n.lock, Lock::Sparse);
        n.set_param("on_hit", ParamValue::Choice(1)).unwrap();
        assert_eq!(n.on_hit, OnHit::Log);
        n.set_param("locks", ParamValue::Int(40)).unwrap();
        assert_eq!(n.locks, 16, "the count is clamped to what the parameter offers");
    }

    /// The same transmitter heard twice is one row, and two carriers further
    /// apart than half a narrowband channel are two.
    #[test]
    fn hits_are_merged_by_identity_first_and_by_frequency_otherwise() {
        let mut n = walking(433.0, 435.0, 1.0, 5.0, OnHit::Log);
        block(&mut n, Vec::new());
        for _ in 0..3 {
            block(&mut n, Vec::new());
        }
        block(
            &mut n,
            vec![
                named(433_920_000, "ble", "AA:BB:CC:DD:EE:FF"),
                // The same device 400 kHz away: an identity outranks where it
                // was heard.
                named(434_320_000, "ble", "AA:BB:CC:DD:EE:FF"),
                // Two carriers 5 kHz apart are one detector wobbling.
                burst(433_100_000),
                burst(433_105_000),
                // And 20 kHz away is a second one.
                burst(433_125_000),
            ],
        );
        let keys: Vec<String> = n.found().iter().map(|f| f.key.label()).collect();
        assert_eq!(keys.len(), 3, "expected three transmitters, got {keys:?}");
        assert_eq!(keys[0], "ble:AA:BB:CC:DD:EE:FF");
        assert_eq!(keys[1], "433.1000M");
        assert_eq!(keys[2], "433.1250M");
        assert_eq!(n.found()[0].heard, 2);
        assert_eq!(n.found()[0].protocol.as_deref(), Some("ble"));
    }

    /// What the operator has told it to ignore neither holds the walk nor
    /// appears in the list, whether it is named by identity or by frequency.
    #[test]
    fn an_ignored_transmitter_does_not_stop_the_walk() {
        let mut n = walking(433.0, 435.0, 1.0, 1.0, OnHit::Hold);
        n.set_param("ignore", ParamValue::Text("ble:AA:BB:CC:DD:EE:FF,433.9200M".into())).unwrap();
        assert_eq!(n.ignored().len(), 2);
        block(&mut n, Vec::new());
        for _ in 0..3 {
            block(&mut n, vec![named(434_000_000, "ble", "AA:BB:CC:DD:EE:FF")]);
        }
        // 433.9215 MHz is within half a narrowband channel of the ignored
        // frequency and is the same carrier.
        block(&mut n, vec![burst(433_921_500)]);
        assert!(!n.holding(), "an ignored transmitter held the walk");
        assert_eq!(n.found().len(), 0);
        // Something else on the same step still holds it.
        block(&mut n, vec![burst(433_700_000)]);
        assert!(n.holding());
        assert_eq!(n.found().len(), 1);
        assert_eq!(n.found()[0].key, Key::Frequency(433_700_000));
    }

    /// Ignoring something already found takes it off the list, so the pane
    /// does not keep showing what the operator has dismissed.
    #[test]
    fn ignoring_a_found_transmitter_drops_the_row() {
        let mut n = walking(433.0, 435.0, 1.0, 1.0, OnHit::Log);
        block(&mut n, Vec::new());
        for _ in 0..3 {
            block(&mut n, Vec::new());
        }
        block(&mut n, vec![burst(433_920_000), burst(433_700_000)]);
        assert_eq!(n.found().len(), 2);
        n.ignore(Key::Frequency(433_920_000));
        assert_eq!(n.found().len(), 1);
        assert_eq!(n.found()[0].key, Key::Frequency(433_700_000));
        assert_eq!(n.ignore_list(), "433.9200M");
    }

    /// An ignore list survives the round trip through a stage setting, which
    /// is how it is saved with the rest of what the operator set.
    #[test]
    fn the_ignore_list_round_trips_through_its_setting() {
        let mut n = BandScanNode::new();
        n.set_ignore_list("ble:AA:BB, adsb:4ca1fb, 1090.0000M, nonsense");
        assert_eq!(
            n.ignored(),
            &[
                Key::Identity { space: "ble".into(), id: "AA:BB".into() },
                Key::Identity { space: "adsb".into(), id: "4ca1fb".into() },
                Key::Frequency(1_090_000_000),
            ]
        );
        assert_eq!(n.ignore_list(), "ble:AA:BB,adsb:4ca1fb,1090.0000M");
    }

    /// Switched off, it is a stage that does nothing at all: no retune, no
    /// rows, whatever is on the bus.
    #[test]
    fn a_walk_that_is_not_running_asks_for_nothing() {
        let mut n = BandScanNode::new();
        n.lo_hz = 433e6;
        n.hi_hz = 435e6;
        n.step_hz = 1e6;
        for _ in 0..20 {
            assert!(block(&mut n, vec![burst(433_920_000)]).is_empty());
        }
        assert_eq!(n.found().len(), 0);
        assert_eq!(n.steps(), 0);
        assert_eq!(n.center_hz(), None);
    }

    /// Starting a walk starts it over, rather than carrying last hour's list.
    #[test]
    fn starting_again_forgets_what_the_last_run_found() {
        let mut n = walking(433.0, 435.0, 1.0, 1.0, OnHit::Log);
        block(&mut n, Vec::new());
        for _ in 0..3 {
            block(&mut n, Vec::new());
        }
        block(&mut n, vec![burst(433_920_000)]);
        assert_eq!(n.found().len(), 1);
        n.set_param("running", ParamValue::Bool(false)).unwrap();
        n.set_param("running", ParamValue::Bool(true)).unwrap();
        assert_eq!(n.found().len(), 0);
        assert_eq!(n.steps(), 0);
        // The ignore list is the operator's and is not a run's.
        assert_eq!(n.ignored().len(), 0);
    }

    /// A step is as wide as the span the receiver is sampling unless it was
    /// told otherwise, so nothing between two centres goes untuned.
    #[test]
    fn the_step_defaults_to_the_span_the_receiver_samples() {
        let mut n = BandScanNode::new();
        let mut s = StreamSpec::iq(2_400_000.0, Hz(433_000_000));
        s.kind = PortKind::Packets;
        s.bandwidth = 2_400_000.0;
        n.negotiate(&PortSpec { spec: s, latency: 0 }).unwrap();
        assert_eq!(n.step_hz, 2_400_000.0);
    }
}
