//! What to run, and where.
//!
//! The receiver used to answer this with three booleans and two hand-written
//! band gates: sweep the span with the ISM banks unless the dial is on 1090,
//! in which case run Mode S, or on 162, in which case run AIS. That was wrong
//! in both directions. Adding a protocol meant editing a plan struct, a role
//! enum, an assembly function and a module of its own, and the default was to
//! sweep, so tuning to FM broadcast split the span into a hundred channels
//! and ran the whole protocol table over all of them looking for weather
//! sensors. Measured on a HackRF at 2.4 MS/s, that was 436% of one core
//! against 109% for Mode S and 71% for AIS.
//!
//! So it is a table now, and the table is a file rather than a constant. The
//! shipped defaults catch what a receiver is nearly always pointed at, and
//! anything else is a block somebody added: a protocol on a frequency this
//! author never thought of is four lines, not a patch.
//!
//! # What decides that a block runs
//!
//! The span, not the dial. A block runs when the frequencies it demodulates
//! are inside the sampled bandwidth, and every block that qualifies runs at
//! once. The dial is where somebody is looking; the span is what the receiver
//! actually has, and at 2.4 MS/s that is a couple of megahertz of spectrum
//! arriving whether or not anything is pointed at it.
//!
//! This used to test the dial against the block's range and take the first
//! block that matched, which was wrong twice over. A receiver at 153.4 MHz
//! with a pager channel 50 kHz away heard nothing, because the dial was
//! outside a range written narrowly around the channel. And a span holding
//! two protocols ran whichever block was written higher in the file, which is
//! not a decision anybody made.
//!
//! # Why it is not in the session file
//!
//! `session.rs` is rewritten whole every couple of seconds as settings
//! change, which would eat comments and formatting. A file a person is
//! expected to edit cannot be a file the program rewrites, so this one is
//! written once when it is missing and only read afterwards.

use std::path::PathBuf;

/// The bank tiers, which is what "banks" means unless a block says otherwise.
///
/// Four channelizers over the same span, because a channel width is a
/// trade-off with no single right answer: too narrow and it cuts a tone or a
/// sideband off, too wide and it integrates noise the signal never occupied.
/// Every tier hears every burst; what differs is how much of the burst
/// survives and how much noise arrives with it.
///
/// The middle two are measured rather than chosen. A 1.5 kbit/s OOK sensor
/// survives to 12.3 dB peak-to-noise in a 31 kHz channel and needs 22.9 dB in
/// a 125 kHz one, because a wide channel integrates noise across its whole
/// width while the signal occupies a sliver. FSK wants the opposite, since its
/// two tones are tens of kHz apart and a narrow channel cuts one off.
///
/// The outer two are for what the middle two cannot hold. 12.5 kHz is the
/// channel spacing the four-level voice protocols use, and the width a slow
/// narrowband signal wants for the same reason the OOK tier is narrower than
/// the FSK one. 500 kHz is what a chirp needs: LoRa occupies 125 to 500 kHz by
/// spreading factor, and a signal wider than its channel is measured through a
/// filter that removed most of it.
///
/// The cost is the channel count, and it is not free: at 2.4 MS/s these four
/// are 192 + 78 + 20 + 5 channels against the 78 + 20 that came before.
/// Measured by `the_scanner_keeps_up_with_the_stream` on a 48 core machine,
/// that is 6.1x real time against 10.7x for the two tiers, so half the
/// headroom buys the narrow and wide ends of the band. A slower receiver
/// should drop a tier in this file rather than run out of headroom.
pub const DEFAULT_WIDTHS: [f64; 4] = [12_500.0, 31_250.0, 125_000.0, 500_000.0];

/// Which demodulator a block asks for.
#[derive(Clone, PartialEq, Debug)]
pub enum Front {
    /// Find and decode everything in the band on its own: sources wherever
    /// something transmits, each read as its own stream, and the span-wide
    /// decoders where the span reaches what they are for.
    ///
    /// What `banks` became. A bank decided a signal's width before it had
    /// seen the signal, which is why it took four of them; this measures
    /// the width instead and needs one of itself. It also makes the other
    /// fronts unnecessary as defaults: a pager channel is found wherever it
    /// is, and Mode S runs when the span covers 1090 MHz.
    Auto,
    /// Channelize the span and run the protocol tables over every channel.
    Banks(Vec<f64>),
    /// One protocol's decoder, pinned where the block says. A protocol that
    /// reads one channel carries the channel, since paging allocations are
    /// national and M17 runs wherever an amateur puts it; one that reads a
    /// span carries the middle of what it reads. Nothing here knows which
    /// protocols exist: the registry in `nodes::protocol` does.
    Protocol { id: &'static str, hz: f64 },
}

impl Front {
    /// A protocol by registry name, at its own default frequency.
    #[cfg_attr(not(test), allow(dead_code))]
    pub fn named(id: &str) -> Option<Front> {
        let p = nodes::protocol::by_id(id)?;
        Some(Front::Protocol { id: p.id(), hz: p.default_hz() })
    }

    /// A protocol by registry name, on a channel.
    #[cfg_attr(not(test), allow(dead_code))]
    pub fn protocol(id: &str, hz: f64) -> Front {
        let p = nodes::protocol::by_id(id).unwrap_or_else(|| panic!("no protocol {id:?}"));
        Front::Protocol { id: p.id(), hz }
    }

    /// The protocol behind this front end, for one that is one.
    pub fn proto(&self) -> Option<&'static dyn nodes::Protocol> {
        match self {
            Front::Protocol { id, .. } => nodes::protocol::by_id(id),
            _ => None,
        }
    }

    /// The word this front end is written as in the file.
    pub fn key(&self) -> &'static str {
        match self {
            Front::Auto => "auto",
            Front::Banks(_) => "banks",
            Front::Protocol { id, .. } => id,
        }
    }

    /// What it is called where a person reads it.
    pub fn label(&self) -> &'static str {
        match self {
            Front::Auto => "auto",
            Front::Banks(_) => "banks",
            Front::Protocol { .. } => self.proto().map_or("?", |p| p.label()),
        }
    }

    /// Every front end, for a control that offers a choice of them.
    pub fn all() -> Vec<Front> {
        let mut out = vec![Front::Auto];
        out.extend(
            nodes::protocol::all()
                .iter()
                .map(|p| Front::Protocol { id: p.id(), hz: p.default_hz() }),
        );
        out.push(Front::Banks(DEFAULT_WIDTHS.to_vec()));
        out
    }

    /// Whether this front end demodulates one named channel, so a block
    /// listing several means one of these per channel.
    ///
    /// The shape answers it, because the shape is what the chain is built
    /// from: a decoder that is not span wide is placed at a frequency and
    /// reads that channel and no other. The others are about a band. `auto`
    /// searches it, a bank channelizes it, and AIS mixes its two channels
    /// itself, so each of those is one decoder over the whole of it.
    ///
    /// Asked of the placement before, which is a different question: where in
    /// the world the protocol is allowed to be. ACARS and VDL Mode 2 are
    /// licensed by band and read one channel at a time, so both blocks listed
    /// their channels and got a single decoder on whichever frequency the
    /// registry called the default. Four ACARS channels in the table, one
    /// demodulated.
    pub fn reads_one_channel(&self) -> bool {
        self.proto().is_some_and(|p| !p.shape().span_wide)
    }

    /// The same front end moved to another frequency.
    ///
    /// Every protocol front end has one, span wide or not: a camera reads
    /// twenty megahertz and still has to be told which twenty. `auto` and the
    /// banks are about a band and have none, so they do not move.
    fn at(&self, hz: f64) -> Front {
        match self {
            Front::Protocol { id, .. } => Front::Protocol { id, hz },
            other => other.clone(),
        }
    }

    fn parse(s: &str) -> Option<Self> {
        let s = s.trim().to_ascii_lowercase();
        match s.as_str() {
            "auto" | "sources" | "scan" => return Some(Front::Auto),
            "banks" => return Some(Front::Banks(DEFAULT_WIDTHS.to_vec())),
            _ => {}
        }
        let p = nodes::protocol::by_word(&s)?;
        Some(Front::Protocol { id: p.id(), hz: p.default_hz() })
    }
}

/// A front end together with the band its block was written about.
///
/// The band is what a channel bank is built over. Without it a bank
/// channelizes the whole span, which at 60 MS/s means channels wider than the
/// signals in an ISM band and a channel grid that slides under the receiver
/// every time the dial moves.
#[derive(Clone, PartialEq, Debug)]
pub struct FrontAt {
    pub front: Front,
    pub band: (f64, f64),
}

impl FrontAt {
    /// The part of this band the span actually covers, or `None` when the two
    /// do not overlap.
    pub fn covered(&self, center: f64, rate: f64) -> Option<(f64, f64)> {
        let (lo, hi) = (self.band.0.max(center - rate / 2.0), self.band.1.min(center + rate / 2.0));
        (hi > lo).then_some((lo, hi))
    }
}

/// One scanner: where it applies and what it runs.
#[derive(Clone, PartialEq, Debug)]
pub struct Scanner {
    pub name: String,
    /// The band this block is about. A block with no `channels` runs when any
    /// part of this is inside the span; a block with channels is decided by
    /// those instead, since they are the frequencies it actually demodulates.
    pub lo: f64,
    pub hi: f64,
    /// Narrowest span the front end works in.
    pub min_rate: f64,
    /// Frequencies that must all be inside the span, with a channel's margin.
    ///
    /// This is what the hand-written gates used to say in code. AIS needs
    /// both of its channels, because stations alternate and a receiver that
    /// clips one hears half the traffic while looking like a quiet band. Mode
    /// S needs none, since its envelope detector does not care where in the
    /// span the signal sits. Expressing it as data is what lets somebody move
    /// APRS to 144.390 for North America by editing one line.
    pub channels: Vec<f64>,
    /// How far inside the span edge a channel must fall. A channel sitting on
    /// the edge is one being demodulated through the anti-alias filter's
    /// skirt, which reads as silence and looks exactly like an empty band.
    pub margin_hz: f64,
    pub front: Front,
    /// The regions this block is about, or empty for everywhere.
    ///
    /// The spectrum is divided differently by each regulator, so a block can
    /// be right in Dublin and wrong in Denver: 902 to 928 is the American
    /// licence-free band and the European GSM uplink, and 315 MHz is key fobs
    /// in the Americas and Japan and nothing in Europe. A block naming its
    /// regions runs only under the plan the operator picked, so nobody has to
    /// go through the table turning off what their regulator gave to somebody
    /// else.
    pub regions: Vec<crate::bands::Plan>,
    /// Whether this block runs. A block switched off stays in the table with
    /// everything it was configured with, so turning `auto` off once a few
    /// channels are pinned does not mean losing it: it is one click back.
    /// Written to the file as `enabled = false`; absent means on, so every
    /// block that predates this field keeps running.
    pub enabled: bool,
}

impl Scanner {
    /// Whether this scanner applies to a tuning.
    ///
    /// What decides it is whether the span covers what the block needs, not
    /// where the dial happens to sit. A receiver on 20 MS/s at 440 MHz has
    /// the pager channel in front of it whether or not the dial is parked on
    /// it, and a front end that waits to be tuned to a frequency it is
    /// already sampling is throwing the signal away.
    /// Whether this block is about where the operator says they are.
    pub fn here_in(&self, plan: crate::bands::Plan) -> bool {
        self.regions.is_empty() || self.regions.contains(&plan)
    }

    pub fn applies(&self, center: f64, rate: f64) -> bool {
        self.applies_in(crate::bands::plan(), center, rate)
    }

    pub fn applies_in(&self, plan: crate::bands::Plan, center: f64, rate: f64) -> bool {
        if !self.here_in(plan) {
            return false;
        }
        if rate < self.min_rate {
            return false;
        }
        if self.channels.is_empty() {
            // Nothing specific to demodulate, so the band is the test: any
            // overlap with the span, since a bank channelizes whatever it is
            // handed and a wideband detector does not care where in the span
            // a signal sits.
            return self.lo < center + rate / 2.0 && self.hi > center - rate / 2.0;
        }
        // Every channel has to clear the span edge by its margin. A channel
        // on the edge is one being demodulated through the anti-alias
        // filter's skirt, which reads as silence.
        let edge = rate / 2.0 - self.margin_hz;
        // A block whose front end is one demodulator per channel runs for
        // whichever channels are in the span. AIS is the other case, where
        // both channels are one front end and half of them is half the
        // traffic, so there it is all of them or none.
        if self.front.reads_one_channel() {
            return self.channels.iter().any(|c| (c - center).abs() <= edge);
        }
        self.channels.iter().all(|c| (c - center).abs() <= edge)
    }

    /// The channels of this block that the span actually covers.
    fn covered(&self, center: f64, rate: f64) -> Vec<f64> {
        let edge = rate / 2.0 - self.margin_hz;
        self.channels.iter().copied().filter(|c| (c - center).abs() <= edge).collect()
    }
}

/// Which shipped table a file was written from.
///
/// Bumped whenever a block is added to [`DEFAULT_TEXT`], so that a receiver
/// somebody has been running since before a protocol landed picks the new
/// block up. Without it the file is written once, on the first run, and a
/// front end added later never runs for anybody who already had one: BLE
/// shipped, and every existing installation quietly had no Bluetooth.
pub const VERSION: u32 = 4;

/// The scanners, in the order they are consulted.
#[derive(Clone, PartialEq, Debug)]
pub struct Scanners {
    pub list: Vec<Scanner>,
    /// The version the file carried, or zero for one written before this
    /// existed.
    pub version: u32,
}

impl Default for Scanners {
    fn default() -> Self {
        Self::parse(DEFAULT_TEXT)
    }
}

impl Scanners {
    /// `$XDG_CONFIG_HOME/waveshark/scanners`, beside the session.
    pub fn path() -> Option<PathBuf> {
        let base = std::env::var_os("XDG_CONFIG_HOME")
            .map(PathBuf::from)
            .or_else(|| std::env::var_os("HOME").map(|h| PathBuf::from(h).join(".config")))?;
        Some(base.join("waveshark").join("scanners"))
    }

    /// Load, writing the defaults out first if there is no file yet, and
    /// taking in any block a later version added.
    ///
    /// Writing them is the point: a table nobody can see is not configurable,
    /// and the shipped blocks are the worked examples for adding another.
    pub fn load() -> Self {
        let Some(path) = Self::path() else {
            return Self::default();
        };
        match std::fs::read_to_string(&path) {
            Ok(text) => {
                let mut t = Self::parse(&text);
                if t.take_new_blocks() {
                    let _ = t.save();
                }
                t
            }
            Err(_) => {
                if let Some(dir) = path.parent() {
                    let _ = std::fs::create_dir_all(dir);
                }
                let _ = std::fs::write(&path, DEFAULT_TEXT);
                Self::default()
            }
        }
    }

    /// Add every shipped block this file has never seen, and say whether
    /// anything changed.
    ///
    /// By name, and only when the file is older than the shipped table, so a
    /// block the operator edited keeps their version and one they deleted
    /// stays deleted from the next load on. A block deleted before the file
    /// carried a version comes back once, which is the price of not having
    /// recorded the deletion.
    pub fn take_new_blocks(&mut self) -> bool {
        if self.version >= VERSION {
            return false;
        }
        self.version = VERSION;
        for sc in Self::default().list {
            if !self.list.iter().any(|s| s.name == sc.name) {
                self.list.push(sc);
            }
        }
        // Written back even when no block was missing, so the version is
        // recorded and the next load has nothing to do.
        true
    }

    /// Every scanner the span covers, in file order.
    ///
    /// All of them rather than the first, because a span is wide and what is
    /// in it is a fact rather than a preference: at 2.4 MS/s a receiver in
    /// the middle of VHF can hear a pager channel and a packet channel at
    /// once, and hearing one of them because its block is written higher up
    /// the file is not a decision anybody made.
    ///
    /// The cost of an extra front end is what it demodulates. The narrowband
    /// ones are one channel each and cost almost nothing; the banks are the
    /// expensive one, and they are still bounded by the block's own range
    /// overlapping the span at all.
    pub fn active(&self, center: f64, rate: f64) -> Vec<&Scanner> {
        self.active_in(crate::bands::plan(), center, rate)
    }

    pub fn active_in(&self, plan: crate::bands::Plan, center: f64, rate: f64) -> Vec<&Scanner> {
        self.list.iter().filter(|s| s.enabled && s.applies_in(plan, center, rate)).collect()
    }

    /// The front ends the span covers, deduplicated.
    ///
    /// Two blocks that ask for the same thing are one front end: a duplicate
    /// would be a second demodulator on the same channel producing the same
    /// packets twice.
    pub fn fronts(&self, center: f64, rate: f64) -> Vec<FrontAt> {
        self.fronts_in(crate::bands::plan(), center, rate)
    }

    /// The same, under a plan named rather than the one in force. The band
    /// tables are read this way too: what runs where is a fact about a
    /// regulator, and a test should not have to move the whole receiver to
    /// another continent to ask about it.
    pub fn fronts_in(&self, plan: crate::bands::Plan, center: f64, rate: f64) -> Vec<FrontAt> {
        let mut out: Vec<FrontAt> = Vec::new();
        for s in self.active_in(plan, center, rate) {
            let band = (s.lo, s.hi);
            let touching = |a: (f64, f64), b: (f64, f64)| a.0 <= b.1 && b.0 <= a.1;
            // One demodulator per listed channel, which is what lets a block
            // watch a calling channel and the two repeaters beside it rather
            // than only whichever was written first.
            if s.front.reads_one_channel() && s.channels.len() > 1 {
                for hz in s.covered(center, rate) {
                    let front = s.front.at(hz);
                    if !out.iter().any(|e| e.front == front) {
                        out.push(FrontAt { front, band });
                    }
                }
                continue;
            }
            if matches!(s.front, Front::Banks(_) | Front::Auto) {
                // Two blocks asking for the same channel width in bands that
                // meet are one bank over both, not two banks decoding the
                // overlap twice. Bands that do not meet stay separate, which
                // is the case the band exists for: 433 and 868 are the same
                // front end in two different places.
                if let Some(e) =
                    out.iter_mut().find(|e| e.front == s.front && touching(e.band, band))
                {
                    e.band.0 = e.band.0.min(band.0);
                    e.band.1 = e.band.1.max(band.1);
                    continue;
                }
            } else if out.iter().any(|e| e.front == s.front) {
                continue;
            }
            out.push(FrontAt { front: s.front.clone(), band });
        }
        out
    }

    /// The table as the file, which is what the interface writes.
    ///
    /// Generated rather than edited in place, so the comments are the ones
    /// this version ships and a block removed in the interface really is
    /// gone. A file somebody hand-edited round trips through here with its
    /// blocks intact and its own comments replaced by the standard header,
    /// which is the price of the table being editable in two places.
    pub fn render(&self) -> String {
        let mut s = String::from(HEADER);
        s.push_str(&format!("\nversion = {VERSION}\n"));
        for sc in &self.list {
            s.push_str(&format!("\n[{}]\n", sc.name));
            s.push_str(&format!("range = {} - {} MHz\n", num(sc.lo / 1e6), num(sc.hi / 1e6)));
            s.push_str(&format!("span  = {} kHz\n", num(sc.min_rate / 1e3)));
            s.push_str(&format!("front = {}\n", sc.front.key()));
            if let Front::Banks(w) = &sc.front {
                let widths: Vec<String> =
                    w.iter().map(|x| format!("{} kHz", num(x / 1e3))).collect();
                s.push_str(&format!("widths = {}\n", widths.join(", ")));
            }
            if !sc.channels.is_empty() {
                let ch: Vec<String> =
                    sc.channels.iter().map(|x| format!("{} MHz", num(x / 1e6))).collect();
                s.push_str(&format!("channels = {}\n", ch.join(", ")));
            }
            if sc.margin_hz > 0.0 {
                s.push_str(&format!("margin = {} kHz\n", num(sc.margin_hz / 1e3)));
            }
            if !sc.regions.is_empty() {
                let r: Vec<&str> = sc.regions.iter().map(|p| p.id()).collect();
                s.push_str(&format!("region = {}\n", r.join(", ")));
            }
            // Only written when off: the absence of the key is the common
            // case and reads as running, so a file stays terse.
            if !sc.enabled {
                s.push_str("enabled = false\n");
            }
        }
        s
    }

    /// Write the file, creating its directory.
    pub fn save(&self) -> std::io::Result<()> {
        let Some(path) = Self::path() else {
            return Err(std::io::Error::other("no config directory"));
        };
        if let Some(dir) = path.parent() {
            std::fs::create_dir_all(dir)?;
        }
        std::fs::write(path, self.render())
    }

    /// Blocks of `key = value` under a `[name]` heading. Anything unparsable
    /// is skipped rather than fatal, for the reason the session file gives:
    /// a config written by a later version has to load in an earlier one.
    pub fn parse(text: &str) -> Self {
        let mut list = Vec::new();
        let mut version = 0u32;
        let mut cur: Option<Scanner> = None;
        for line in joined(text) {
            let line = line.as_str();
            if let Some(name) = line.strip_prefix('[').and_then(|l| l.strip_suffix(']')) {
                if let Some(mut s) = cur.take().filter(|s| !s.channels_unset()) {
                    s.settle();
                    list.push(s);
                }
                cur = Some(Scanner {
                    name: name.trim().to_string(),
                    lo: 0.0,
                    hi: 0.0,
                    min_rate: 0.0,
                    channels: Vec::new(),
                    margin_hz: 0.0,
                    front: Front::Auto,
                    regions: Vec::new(),
                    enabled: true,
                });
                continue;
            }
            let Some((k, v)) = line.split_once('=') else {
                continue;
            };
            if cur.is_none() {
                if k.trim() == "version" {
                    version = v.trim().parse().unwrap_or(0);
                }
                continue;
            }
            let Some(s) = cur.as_mut() else { continue };
            let (k, v) = (k.trim(), v.trim());
            match k {
                "range" => {
                    if let Some((a, b)) = v.split_once('-') {
                        // The unit is usually written once, on the upper
                        // bound: "433.05 - 434.79 MHz".
                        let unit = unit_of(b);
                        if let (Some(lo), Some(hi)) = (hz_with(a, unit), hz(b)) {
                            s.lo = lo;
                            s.hi = hi;
                        }
                    }
                }
                "span" => s.min_rate = hz(v).unwrap_or(0.0),
                "front" => {
                    if let Some(f) = Front::parse(v) {
                        s.front = f;
                    }
                }
                "channels" => s.channels = list_hz(v),
                "margin" => s.margin_hz = hz(v).unwrap_or(0.0),
                // A known set of values, read once into the enum the rest of
                // the receiver decides by. A word nobody recognises is left
                // out rather than kept as text, so a typo means the block
                // runs everywhere instead of nowhere.
                "region" | "regions" => {
                    s.regions =
                        v.split(',').filter_map(|w| crate::bands::Plan::from_id(w.trim())).collect()
                }
                "enabled" => {
                    s.enabled =
                        !matches!(v.to_ascii_lowercase().as_str(), "false" | "no" | "0" | "off")
                }
                "widths" => {
                    let w: Vec<f64> = v.split(',').filter_map(hz).collect();
                    if !w.is_empty() {
                        s.front = Front::Banks(w);
                    }
                }
                _ => {}
            }
        }
        if let Some(mut s) = cur.filter(|s| !s.channels_unset()) {
            s.settle();
            list.push(s);
        }
        Self { list, version }
    }
}

/// The widths the shipped ISM blocks carried before `sources` existed: two
/// tiers at first, then four.
const LEGACY_WIDTHS: [&[f64]; 2] =
    [&[31_250.0, 125_000.0], &[12_500.0, 31_250.0, 125_000.0, 500_000.0]];

impl Scanner {
    /// Fold what the block said into the front end, now that the whole block
    /// has been read.
    ///
    /// The single-channel front ends need this: what they demodulate is one
    /// frequency, that frequency is regional, and the block already names it.
    /// Without this the channel decides only whether the block matches, and a
    /// receiver told to listen on 144.390 would gate on that and then
    /// demodulate 144.800, which is silence that looks like a quiet band.
    /// Done at the end of the block rather than as the keys arrive, so that
    /// `channels` and `front` can be written in either order.
    ///
    /// A `banks` block at exactly the widths the file used to ship with is
    /// the shipped default nobody edited, and the shipped default for that
    /// band is now `auto` over the same range. The file is written once and only read afterwards, so
    /// this is the only place the change can reach a file that already
    /// exists. A block with any other widths was somebody's decision and is
    /// left alone.
    fn settle(&mut self) {
        if let Front::Banks(w) = &self.front {
            let shipped = LEGACY_WIDTHS.iter().any(|l| {
                w.len() == l.len() && w.iter().zip(l.iter()).all(|(a, b)| (a - b).abs() < 1.0)
            });
            if shipped {
                self.front = Front::Auto;
            }
        }
        self.pin_to_channel();
    }

    /// A single-channel front end takes its frequency from the block. With
    /// several listed it is the first, and the rest become their own front
    /// ends when the span covers them.
    ///
    /// Called from the file's [`Self::settle`] and from the interface's row
    /// editor, which is the whole of it: a block added in the interface used
    /// to keep the protocol's default channel whatever was typed into the
    /// channels field, so a video front end asked for on 5800 was built on
    /// 5865 and read a part of the band the span did not cover.
    pub(crate) fn pin_to_channel(&mut self) {
        let Some(&c) = self.channels.first() else {
            return;
        };
        // A decoder that reads one channel goes on the first listed, and a
        // span wide one goes where it was asked only when one channel was
        // asked for: AIS covers both of its and belongs between them, not on
        // the lower one.
        if self.channels.len() == 1 || self.front.reads_one_channel() {
            self.front = self.front.at(c);
        }
    }
}

impl Scanner {
    /// A block with no usable range is a block that would match everything or
    /// nothing, and both are worse than dropping it.
    fn channels_unset(&self) -> bool {
        self.hi <= self.lo
    }
}

/// The unit suffix of a value, so `433.05 - 434.79 MHz` can write it once.
pub(crate) fn unit_of(s: &str) -> f64 {
    let s = s.trim().to_ascii_lowercase();
    if s.ends_with("ghz") {
        1e9
    } else if s.ends_with("mhz") {
        1e6
    } else if s.ends_with("khz") {
        1e3
    } else {
        1.0
    }
}

/// `162.025 MHz`, `150 kHz`, or a bare number in hertz.
/// The lines of the file, with a wrapped one joined to the line it belongs
/// to, comments stripped and blanks dropped.
///
/// A long list of channels does not fit on one line and a reader will wrap
/// it. The continuation used to be a line with no `=` in it, which was
/// skipped in silence: ten frequencies were written, five were read, and
/// nothing said which five.
fn joined(text: &str) -> Vec<String> {
    let mut out: Vec<String> = Vec::new();
    for line in text.lines() {
        let line = line.split('#').next().unwrap_or("").trim();
        if line.is_empty() {
            continue;
        }
        // A heading or a `key = value` starts a line; anything else carries
        // on the one before it, which is what an indented wrap looks like.
        let starts = line.starts_with('[') || line.contains('=');
        match (starts, out.last_mut()) {
            (false, Some(prev)) => {
                if !prev.ends_with(',') {
                    prev.push(',');
                }
                prev.push(' ');
                prev.push_str(line);
            }
            _ => out.push(line.to_string()),
        }
    }
    out
}

/// A comma separated list of frequencies, where the unit may be written on
/// every one or once at the end.
///
/// "136.1, 136.65 MHz" is ten megahertz written the way a person writes it,
/// and reading the first as 136.1 Hz drops it out of the band silently. The
/// same rule the `range` key already follows.
fn list_hz(v: &str) -> Vec<f64> {
    let unit = v.rsplit(',').next().map(unit_of).unwrap_or(1.0);
    v.split(',').filter_map(|p| hz_with(p, unit)).collect()
}

pub(crate) fn hz(s: &str) -> Option<f64> {
    hz_with(s, unit_of(s))
}

fn hz_with(s: &str, unit: f64) -> Option<f64> {
    let t = s.trim().trim_end_matches(|c: char| c.is_ascii_alphabetic()).trim();
    let v: f64 = t.parse().ok()?;
    Some(v * if unit_of(s) == 1.0 { unit } else { unit_of(s) })
}

/// Print a frequency without trailing zeros: 162.025, not 162.025000.
pub(crate) fn num(v: f64) -> String {
    let s = format!("{v:.4}");
    let s = s.trim_end_matches('0').trim_end_matches('.');
    s.to_string()
}

/// The header the interface writes above the blocks, which is also the
/// format's documentation.
pub const HEADER: &str = "\
# waveshark scanners: what to run, and where.
#
# Every block the tuned span covers runs, and a block is covered when the
# frequencies it demodulates are inside the span. Edit here or in the
# interface; the interface rewrites this file from its own blocks, so comments
# below this header are not kept.
#
#   range     the band this block is about; with no channels, any overlap
#             with the span runs it
#   span      narrowest span the front end works in
#   front     auto | banks | a protocol: modes, ais, aprs, pocsag, m17, dmr,
#             tetra, gsm, ble, lora, wmbus, video
#   channels  frequencies to demodulate (optional). A protocol that reads one
#             channel is one demodulator per channel, and runs for each
#             channel the span covers; ais needs both of its, so it runs only
#             when the span holds both
#   margin    how far inside the span edge they must fall (optional)
#   region    europe | americas | asia-pacific, for a block that is only
#             right in some of them; absent means everywhere
#   widths    channel widths, for front = banks
#   version   which shipped table this file was written from; blocks added
#             by a later one are taken in on load
";

/// The defaults, written out when there is no file.
///
/// This doubles as the documentation for the format, which is why it carries
/// its own comments rather than being built from struct literals.
pub const DEFAULT_TEXT: &str = "\
# waveshark scanners: what to run, and where.
#
# Written once when this file was missing, and only read afterwards, so it is
# safe to edit. Delete a block to stop it running; add one to scan somewhere
# new. Every block the tuned span covers runs, and a block is covered when the
# frequencies it demodulates are inside the span, wherever the dial sits.
#
#   range     the band this block is about; with no channels, any overlap
#             with the span runs it
#   span      narrowest span the front end works in
#   front     auto | banks | a protocol: modes, ais, aprs, pocsag, m17, dmr,
#             tetra, gsm, ble, lora, wmbus, video
#   channels  frequencies to demodulate (optional). A protocol that reads one
#             channel is one demodulator per channel, and runs for each
#             channel the span covers; ais needs both of its, so it runs only
#             when the span holds both
#   margin    how far inside the span edge they must fall (optional)
#   region    europe | americas | asia-pacific, for a block that is only
#             right in some of them; absent means everywhere
#   widths    channel widths, for front = banks
#   version   which shipped table this file was written from; blocks added
#             by a later one are taken in on load
#
# `auto` finds and decodes everything in the block's band on its own: sources
# wherever something transmits, each measured for centre and width and read
# as its own stream, plus the span-wide decoders (Mode S, AIS) where the band
# reaches the frequency they are for. That is why there is no M17 block: M17
# runs wherever an amateur puts it, so a block naming one channel would read
# that channel and miss every other, while `auto` reads it wherever it is.
# It costs what is transmitting in the band, so the band is what keeps it
# real-time: an ISM allocation, not the whole of what a wideband radio
# samples. `banks` is the older fixed grid of channels at the widths listed,
# kept for comparison.

[ADS-B]
range = 1089.9 - 1090.1 MHz
span  = 2 MHz
front = modes

[AIS]
# Stations alternate between the two channels, so a span holding only one
# hears half the traffic while looking like a quiet band.
range    = 161.9 - 162.1 MHz
span     = 150 kHz
front    = ais
channels = 161.975 MHz, 162.025 MHz
margin   = 25 kHz

[APRS]
# 144.800 across Europe. North America is 144.390, Japan 144.640: change the
# range and the channel together.
range    = 144.79 - 144.81 MHz
span     = 48 kHz
front    = aprs
channels = 144.800 MHz
margin   = 8 kHz

[ACARS]
# The aircraft datalink channels, which differ by region: 131.725 is the
# European primary and 131.525 and 131.825 the other two European operational
# control channels, while 131.550 is the ARINC primary in North America and
# the worldwide fallback. All four fit in one span, so all four are watched.
# Most European airline traffic has moved to VDL Mode 2 up at 136.675 to
# 136.975, which is a different waveform and is not read here.
range    = 131.5 - 131.85 MHz
span     = 400 kHz
front    = acars
channels = 131.525 MHz, 131.550 MHz, 131.725 MHz, 131.825 MHz
margin   = 15 kHz

[SSTV]
# The two metre calling frequency, which is where a picture is sent across
# Europe. The shortwave ones (14.230 and 7.171) need a sideband demodulator
# in front and a receiver that reaches them.
range    = 144.49 - 144.51 MHz
span     = 48 kHz
front    = sstv
channels = 144.500 MHz
margin   = 12.5 kHz

[VDL2]
# Every VDL Mode 2 channel in use. 136.975 is the common signalling channel
# every ground station carries, worldwide; the rest are split between ARINC
# and SITA and between regions, and are listed here under both so a receiver
# picks up whichever its span reaches. European traffic moved onto these as
# ACARS emptied out.
range    = 136.6 - 137.0 MHz
span     = 400 kHz
front    = vdl2
channels = 136.675, 136.725, 136.775, 136.825, 136.875, 136.975 MHz
margin   = 25 kHz
region   = europe, asia-pacific

[VDL2 Americas]
# The ARINC and SITA channels of North America. 136.1 is nearly a megahertz
# below the others, so it is only ever in the span on its own or on a wide
# one.
range    = 136.05 - 137.0 MHz
span     = 400 kHz
front    = vdl2
channels = 136.1, 136.65, 136.7, 136.8, 136.975 MHz
margin   = 25 kHz
region   = americas

[POCSAG]
# The amateur DAPNET network, which runs POCSAG at 1200 baud and is the one
# paging channel that is the same across Europe. Commercial paging is
# national: 138 to 153 MHz in the UK, 929 to 932 MHz in the United States,
# 450 to 470 MHz in much of Europe. Point this at a channel you can hear, and
# know that pager traffic carries names, addresses and medical detail before
# logging it.
range    = 439.9 - 440.1 MHz
span     = 100 kHz
front    = pocsag
channels = 439.9875 MHz
margin   = 12.5 kHz

[FLEX]
# The Dutch P2000 network, the one FLEX channel with a fixed frequency across
# a whole country and the busiest in Europe: fire, ambulance and police
# dispatch at 1600 baud. Elsewhere FLEX is national and commercial, in the
# same allocations POCSAG uses. The same warning applies: these pages carry
# names, addresses and medical detail before anybody logs them.
range    = 169.4 - 169.8 MHz
span     = 100 kHz
front    = flex
channels = 169.65 MHz
margin   = 12.5 kHz
region   = europe

[GSM]
# One GSM carrier's beacon: the frequency correction tone, and the cell
# identity and frame number in the synchronisation burst a frame later. Both
# are broadcast in the clear; everything above them is ciphered.
#
# Off by default, and pointed at nothing in particular, because a beacon has
# no frequency worth shipping: carriers are licensed per operator and per
# country, and this front end reads one at a time. The blocks below find them
# instead; this is where to pin one carrier and watch only it. The 900
# downlink raster starts at 935.2 MHz and steps 200 kHz; E-GSM starts at
# 925.2.
range    = 925 - 960 MHz
span     = 2 MHz
front    = gsm
channels = 947.4 MHz
enabled  = false

# The GSM downlinks, one block per allocation. `auto` finds the carriers and
# puts the front end above on each one that measures 200 kHz, so a cell
# decodes without anybody naming its channel. Only the base station halves
# are here: a handset transmits in bursts on the uplink and broadcasts no
# identity, so there is nothing on that half to read.
#
# These are dense allocations and a span over one opens a source per carrier,
# so turn off the ones your region does not use.

[GSM 850]
# The Americas, ARFCN 128 up from 869.2 MHz.
range  = 869 - 894 MHz
span   = 1 MHz
front  = auto
region = americas

[GSM 900]
# Europe and most of the world: GSM-R from 921, then E-GSM and P-GSM to 960.
range  = 921 - 960 MHz
span   = 1 MHz
front  = auto
region = europe, asia-pacific

[DCS 1800]
range  = 1805 - 1880 MHz
span   = 1 MHz
front  = auto
region = europe, asia-pacific

[PCS 1900]
# The American 1900 downlink, which reuses the DCS channel numbers.
range  = 1930 - 1990 MHz
span   = 1 MHz
front  = auto
region = americas

[TETRA]
# Base station downlinks, which is the half of a TETRA network a listener
# hears: 390 to 400 MHz across Europe for the emergency services, with the
# handsets answering 10 MHz lower. Commercial networks sit at 415 to 430 MHz,
# so move the range if that is what is above you.
#
# The carriers are pi/4-DQPSK at 18 kbaud on a 25 kHz raster. Each one is
# found, measured, and read: the control channels decode, so a carrier is
# logged as who it is, MCC, MNC, colour code and location area, beside the
# measurement rows that say it is still on the air. Traffic is not decoded,
# and on these networks it is encrypted anyway.
range = 390 - 400 MHz
span  = 250 kHz
front = auto

[Radiosonde]
# Weather balloons. Every upper-air station in the world launches one at 00
# and 12 UTC and it climbs for two hours, so there is nearly always one
# overhead somewhere in this band. A Vaisala RS41, a Graw DFM, a Meteomodem
# M10, an InterMet iMet, a Meisei iMS-100, a Meteo-Radiy MRZ or a Lockheed
# LMS6 is tuned
# anywhere in it on a 10 kHz raster and the frequency is decided at the
# station rather than published, so the band is scanned rather than a channel
# list watched.
range = 400 - 406 MHz
span  = 25 kHz
front = auto

# The licence-free allocations, one block each, so a receiver tuned into any
# of them decodes what is there without being told. They are all shipped
# enabled because a block only costs anything when the span covers it, and a
# span covers at most one or two of these at a time. The ones that mean
# something else elsewhere name their region and run only under the plan in
# the settings: 902-928 is the American licence-free band and the European GSM
# uplink, and 315 is key fobs in the Americas and Japan and nothing in Europe.
# Drop the region line from a block to run it wherever you are.

[ISM 27]
# RC models, telemetry and CB data, under the amateur 10 m band. Needs a radio
# that tunes below 24 MHz.
range = 26.957 - 27.283 MHz
span  = 250 kHz
front = auto

[ISM 40]
# The other RC allocation, plus older sensors and garage doors.
range = 40.66 - 40.7 MHz
span  = 250 kHz
front = auto

[ISM 169]
# European wireless M-Bus, which is where smart meters report at long range.
range  = 169.4 - 169.475 MHz
span   = 250 kHz
front  = auto
region = europe

[ISM 315]
# Key fobs and tyre pressure sensors in the Americas and Japan.
range  = 314 - 316 MHz
span   = 250 kHz
front  = auto
region = americas, asia-pacific

[SLP 426]
# Japan's specified low power band: telemetry, alarms and short range voice.
range  = 426 - 426.1 MHz
span   = 250 kHz
front  = auto
region = asia-pacific

[ISM 433]
range = 433.05 - 434.79 MHz
span  = 250 kHz
front = auto

[ISM 868]
# The European short range band, 863 up: LoRaWAN, wireless M-Bus, alarms and
# most of what a weather sensor here transmits on.
range  = 862 - 876 MHz
span   = 250 kHz
front  = auto
region = europe

[ISM 915]
# The American licence-free band. In Europe this is the GSM 900 uplink and in
# Japan the top of it is the 920 band below, so what runs here is a handset
# rather than a sensor unless the FCC is your regulator.
range  = 902 - 928 MHz
span   = 250 kHz
front  = auto
region = americas

[ISM 920]
# Japan and much of Region 3, inside the American band above.
range  = 920 - 928 MHz
span   = 250 kHz
front  = auto
region = asia-pacific

[ISM 2.4]
# Wi-Fi, Bluetooth, video links and RC. Crowded, wide, and mostly signals far
# wider than the span a receiver samples, so expect measurements rather than
# decodes. Bluetooth advertising needs no block of its own: `auto` runs it
# across the span wherever one of the three advertising channels is inside
# it, the same way it runs Mode S and AIS.
range = 2400 - 2483.5 MHz
span  = 250 kHz
front = auto

[ISM 5.8]
# Wi-Fi and the analogue video links that share the band. Above what most
# receivers tune, so the block simply
# never matches on those.
range = 5725 - 5875 MHz
span  = 250 kHz
front = auto
";

#[cfg(test)]
mod tests {
    /// The front ends alone, for the tests that are about which front end runs
    /// rather than over what band.
    fn kinds(v: &[FrontAt]) -> Vec<Front> {
        v.iter().map(|f| f.front.clone()).collect()
    }

    use super::*;

    #[test]
    fn a_file_from_before_a_block_shipped_takes_it_in() {
        // A table written by an older build: no version line, and none of
        // the ISM blocks. Every receiver in the field has a file like this,
        // and until the merge a block added after their first run never
        // reached them.
        let old = "[ADS-B]\nrange = 1089.9 - 1090.1 MHz\nspan = 2000 kHz\nfront = modes\n";
        let mut t = Scanners::parse(old);
        assert_eq!(t.version, 0);
        assert_eq!(t.list.len(), 1);
        assert!(t.take_new_blocks());
        assert_eq!(t.version, VERSION);
        assert!(t.list.iter().any(|s| s.name == "ISM 2.4"), "nothing added: {:?}", t.list);
        // The operator's own block keeps its place at the front.
        assert_eq!(t.list[0].name, "ADS-B");
        // And a second pass has nothing to do, so a block deleted now stays
        // deleted.
        let mut again = Scanners::parse(&t.render());
        assert_eq!(again.version, VERSION);
        again.list.retain(|s| s.name != "ISM 2.4");
        assert!(!again.take_new_blocks());
        assert!(!again.list.iter().any(|s| s.name == "ISM 2.4"));
    }

    #[test]
    fn a_disabled_block_does_not_run_but_survives_the_file() {
        // Turning auto off after pinning channels must keep it in the table:
        // a switch, not a delete. And an absent `enabled` key is on, so a
        // file that predates the field keeps running.
        let s = Scanners::parse(
            "[Auto]\nrange = 400 - 470 MHz\nspan = 250 kHz\nfront = auto\nenabled = false\n\
             [Pager]\nrange = 439.9 - 440.1 MHz\nspan = 25 kHz\nfront = pocsag\n\
             channels = 439.9875 MHz\n",
        );
        assert_eq!(s.list.len(), 2);
        assert!(!s.list[0].enabled, "the auto block is off");
        assert!(s.list[1].enabled, "an absent enabled key is on");
        // At a tuning both cover, only the pager runs.
        let running: Vec<&str> =
            s.active(439_987_500.0, 2_400_000.0).iter().map(|b| b.name.as_str()).collect();
        assert_eq!(running, ["Pager"]);
        // The off state round-trips through the file.
        assert!(!Scanners::parse(&s.render()).list[0].enabled);
    }

    #[test]
    fn a_file_written_from_the_old_defaults_is_read_as_sources() {
        // The file is written once and only read afterwards, so a receiver
        // that wrote it before `sources` existed has ISM blocks saying
        // `banks` at the four widths that used to ship. Those blocks are the
        // default, and the default changed.
        let s = Scanners::parse(
            "[ISM 433]\nrange = 433.05 - 434.79 MHz\nspan = 250 kHz\nfront = banks\n\
             widths = 12.5 kHz, 31.25 kHz, 125 kHz, 500 kHz\n\
             [Mine]\nrange = 433.05 - 434.79 MHz\nspan = 250 kHz\nfront = banks\n\
             widths = 20 kHz\n",
        );
        assert_eq!(s.list[0].front, Front::Auto);
        assert_eq!(s.list[1].front, Front::Banks(vec![20_000.0]), "a chosen width is kept");
        let older = Scanners::parse(
            "[ISM 433]\nrange = 433.05 - 434.79 MHz\nspan = 250 kHz\nfront = banks\n\
             widths = 31.25 kHz, 125 kHz\n",
        );
        assert_eq!(older.list[0].front, Front::Auto, "the two-tier default before that");
    }

    #[test]
    fn the_shipped_defaults_parse() {
        let s = Scanners::default();
        let names: Vec<&str> = s.list.iter().map(|x| x.name.as_str()).collect();
        assert_eq!(
            names,
            [
                "ADS-B",
                "AIS",
                "APRS",
                "ACARS",
                "SSTV",
                "VDL2",
                "VDL2 Americas",
                "POCSAG",
                "FLEX",
                "GSM",
                "GSM 850",
                "GSM 900",
                "DCS 1800",
                "PCS 1900",
                "TETRA",
                "Radiosonde",
                "ISM 27",
                "ISM 40",
                "ISM 169",
                "ISM 315",
                "SLP 426",
                "ISM 433",
                "ISM 868",
                "ISM 915",
                "ISM 920",
                "ISM 2.4",
                "ISM 5.8"
            ]
        );
        // The GSM block ships off: it names a carrier nobody can know from
        // here, so it is a place to put one rather than a place one is.
        let gsm = s.list.iter().find(|x| x.name == "GSM").unwrap();
        assert!(!gsm.enabled);
        assert_eq!(gsm.front, Front::protocol("gsm", 947_400_000.0));
    }

    /// Every licence-free allocation the ribbon draws has a block that scans
    /// it, in every region. The two tables are the same set seen twice: a
    /// band named on screen and then not scanned is a receiver that knows
    /// what it is looking at and does nothing about it.
    #[test]
    fn every_ism_band_in_every_plan_has_a_scanner() {
        use crate::bands::Plan;
        let s = Scanners::default();
        for p in Plan::ALL {
            for b in p.bands().iter().filter(|b| b.is_ism()) {
                let covered = s.list.iter().any(|sc| {
                    sc.front == Front::Auto && sc.lo <= b.lo + 1.0 && sc.hi >= b.hi - 1.0
                });
                assert!(covered, "{} in {} has no auto block", b.name, p.id());
            }
        }
    }

    /// Tuning into a licence-free band runs `auto` over it and nothing else.
    #[test]
    fn the_ism_blocks_run_where_they_belong() {
        use crate::bands::Plan;
        let s = Scanners::default();
        let at = |plan: Plan, hz: f64| kinds(&s.fronts_in(plan, hz, 2_400_000.0));
        // The allocations that are the same the world over.
        for hz in [40_680_000.0, 433_920_000.0, 2_437_000_000.0, 5_800_000_000.0] {
            for plan in Plan::ALL {
                assert_eq!(at(plan, hz), [Front::Auto], "nothing runs at {hz} in {plan:?}");
            }
        }
        // And the ones that are somebody else's band elsewhere. 169 and 868
        // are European, 315 is key fobs in the Americas and Japan, 426 is
        // Japan's, and 902-928 is American licence-free and the European GSM
        // uplink: a European pointed at 915 is hearing a handset, and running
        // a sensor scanner over it measures carriers they cannot use.
        // 169.4 is the European sensor band and 169.65 the P2000 paging
        // channel, which a 2.4 MHz span reaches from it, so both run.
        assert_eq!(
            at(Plan::Europe, 169_437_500.0),
            [Front::protocol("flex", 169_650_000.0), Front::Auto]
        );
        assert_eq!(at(Plan::Americas, 169_437_500.0), []);
        assert_eq!(at(Plan::Americas, 315_000_000.0), [Front::Auto]);
        assert_eq!(at(Plan::AsiaPacific, 315_000_000.0), [Front::Auto]);
        assert_eq!(at(Plan::Europe, 315_000_000.0), []);
        assert_eq!(at(Plan::AsiaPacific, 426_050_000.0), [Front::Auto]);
        assert_eq!(at(Plan::Europe, 426_050_000.0), []);
        assert_eq!(at(Plan::Europe, 868_300_000.0), [Front::Auto]);
        // On a narrow span, because a 2.4 MHz one at 868.3 clips the bottom
        // of GSM 850, which an American is entitled to scan.
        assert_eq!(kinds(&s.fronts_in(Plan::Americas, 868_300_000.0, 250_000.0)), []);
        assert_eq!(at(Plan::Americas, 915_000_000.0), [Front::Auto]);
        assert_eq!(at(Plan::Europe, 915_000_000.0), [], "that is the GSM uplink here");
        // 920 sits inside 902-928, and in Region 3 both blocks are about the
        // same band: two auto blocks over bands that meet are one front end
        // rather than the same sources decoded twice.
        assert_eq!(at(Plan::AsiaPacific, 923_000_000.0), [Front::Auto]);
    }

    /// Every GSM downlink is scanned, and no uplink is.
    ///
    /// A carrier is found rather than named: the operator's channels are not
    /// knowable from here, so a block per allocation and the detector is the
    /// only arrangement that decodes a cell nobody typed in.
    #[test]
    fn the_gsm_downlinks_are_scanned_and_the_uplinks_are_not() {
        use crate::bands::Plan;
        let s = Scanners::default();
        let at = |plan: Plan, hz: f64| kinds(&s.fronts_in(plan, hz, 2_400_000.0));
        // Each allocation under the regulator that granted it. 850 and 1900
        // are the American pair; 900 and 1800 are used across Europe and
        // Region 3.
        for (plan, hz) in [
            (Plan::Americas, 881_000_000.0),   // GSM 850, ARFCN 190 or so
            (Plan::Americas, 1_960_000_000.0), // PCS 1900
            (Plan::Europe, 923_000_000.0),     // GSM-R
            (Plan::Europe, 947_400_000.0),     // E-GSM / P-GSM 900
            (Plan::Europe, 1_842_000_000.0),   // DCS 1800
            (Plan::AsiaPacific, 947_400_000.0),
        ] {
            assert_eq!(at(plan, hz), [Front::Auto], "nothing runs at {hz} in {plan:?}");
        }
        // And not under one that gave the band to somebody else: 881 is the
        // American downlink and inside nothing European at all.
        assert_eq!(at(Plan::Europe, 881_000_000.0), []);
        assert_eq!(at(Plan::Americas, 1_842_000_000.0), []);
        // The halves the handsets transmit in, where there is no beacon.
        assert_eq!(at(Plan::Europe, 897_000_000.0), [], "GSM 900 uplink");
        assert_eq!(at(Plan::Europe, 1_750_000_000.0), [], "DCS 1800 uplink");
        // And a span too narrow for the carrier's own rate does not match.
        assert!(s.fronts_in(Plan::Europe, 947_400_000.0, 500_000.0).is_empty());
    }

    /// The behaviour the old hand-written gates had, now as table lookups.
    #[test]
    fn the_defaults_put_each_front_end_where_it_belongs() {
        let s = Scanners::default();
        let fronts = |c: f64, r: f64| -> Vec<Front> {
            s.fronts(c, r).into_iter().map(|f| f.front).collect()
        };
        assert_eq!(fronts(1_090_000_000.0, 2_400_000.0), [Front::named("mode_s").unwrap()]);
        assert_eq!(fronts(162_000_000.0, 2_400_000.0), [Front::named("ais").unwrap()]);
        assert_eq!(
            fronts(144_800_000.0, 2_400_000.0),
            [Front::protocol("aprs", 144_800_000.0), Front::protocol("sstv", 144_500_000.0)],
            "a 2.4 MHz span over 144.8 reaches the SSTV calling frequency too"
        );
        assert_eq!(fronts(439_987_500.0, 500_000.0), [Front::protocol("pocsag", 439_987_500.0)]);
        // M17 has no channel of its own to list: it runs wherever an amateur
        // puts it, and `auto` finds it there. A block naming one frequency
        // would decode that frequency and miss every other.
        assert_eq!(fronts(433_920_000.0, 2_400_000.0), [Front::Auto]);
        assert_eq!(fronts(433_920_000.0, 250_000.0), [Front::Auto]);
        assert_eq!(fronts(868_300_000.0, 2_400_000.0), [Front::Auto]);
        // The TETRA downlinks: the detector finds the carriers and the
        // auto node gives each one the decoder that reads its identity.
        assert_eq!(fronts(395_000_000.0, 2_400_000.0), [Front::Auto]);
        assert_eq!(fronts(380_000_000.0, 250_000.0), [], "the uplink half is not covered");
    }

    /// A block that lists channels gets a decoder on each one in the span.
    ///
    /// ACARS and VDL Mode 2 are the two blocks with more than one channel and
    /// a decoder that reads one channel at a time. Both listed their
    /// frequencies and both got a single decoder, on whichever one the
    /// registry called the default, because whether a front end is
    /// per-channel was asked of the protocol's placement: they are licensed
    /// by band, so they were treated as one decoder over the band, which is
    /// what AIS is and what they are not. Four ACARS channels in the table,
    /// one demodulated, and nothing anywhere saying so.
    #[test]
    fn every_channel_a_block_lists_gets_its_own_decoder() {
        let s = Scanners::default();
        let on = |id: &str, c: f64, r: f64| -> Vec<f64> {
            let mut out: Vec<f64> = s
                .fronts(c, r)
                .into_iter()
                .filter_map(|f| match f.front {
                    Front::Protocol { id: got, hz } if got == id => Some(hz.round()),
                    _ => None,
                })
                .collect();
            out.sort_by(f64::total_cmp);
            out
        };
        assert_eq!(
            on("acars", 131_675_000.0, 400_000.0),
            [131_525_000.0, 131_550_000.0, 131_725_000.0, 131_825_000.0],
            "the airband channels the table lists"
        );
        assert_eq!(
            on("vdl2", 136_825_000.0, 400_000.0),
            [
                136_675_000.0,
                136_725_000.0,
                136_775_000.0,
                136_825_000.0,
                136_875_000.0,
                136_975_000.0
            ],
            "the European VHF datalink channels the table lists"
        );
        // Only the ones the span reaches, with their margin: a channel on the
        // edge is demodulated through the anti-alias skirt and reads as
        // silence.
        assert_eq!(on("acars", 131_537_500.0, 400_000.0), [131_525_000.0, 131_550_000.0]);

        // AIS is the other case and is unchanged: its two channels are one
        // decoder, which mixes both itself, so there is one of it and it
        // needs both channels in the span.
        assert_eq!(on("ais", 162_000_000.0, 150_000.0).len(), 1);
        assert_eq!(on("ais", 161_975_000.0, 40_000.0).len(), 0, "half the traffic is not enough");
    }

    /// A block naming its region runs only there, and survives a rewrite.
    ///
    /// The table is rewritten from the interface's own rows, so a field the
    /// editor drops is a field an operator loses the moment they touch any
    /// block: a regional block would quietly become a worldwide one.
    #[test]
    fn a_block_is_gated_by_the_region_and_keeps_it_through_a_rewrite() {
        use crate::bands::Plan;
        let t = Scanners::parse(
            "[Fobs]\nrange = 314 - 316 MHz\nspan = 250 kHz\nfront = auto\nregion = americas, asia-pacific\n\
             [Meters]\nrange = 169.4 - 169.475 MHz\nspan = 250 kHz\nfront = auto\nregion = europe\n\
             [Everywhere]\nrange = 433.05 - 434.79 MHz\nspan = 250 kHz\nfront = auto\n",
        );
        assert_eq!(t.list.len(), 3);
        assert_eq!(t.list[0].regions, [Plan::Americas, Plan::AsiaPacific]);
        assert_eq!(t.list[1].regions, [Plan::Europe]);
        assert_eq!(t.list[2].regions, [], "absent means everywhere");

        let runs = |plan: Plan, hz: f64| !t.fronts_in(plan, hz, 250_000.0).is_empty();
        assert!(runs(Plan::Americas, 315e6));
        assert!(runs(Plan::AsiaPacific, 315e6));
        assert!(!runs(Plan::Europe, 315e6));
        assert!(runs(Plan::Europe, 169.4375e6));
        assert!(!runs(Plan::AsiaPacific, 169.4375e6));
        for plan in Plan::ALL {
            assert!(runs(plan, 433.92e6), "an ungated block runs in {plan:?}");
        }

        // Through the file and back, which is what the interface does every
        // time a row is edited.
        assert_eq!(Scanners::parse(&t.render()).list, t.list);
        // A region nobody recognises is left out rather than kept as text, so
        // a typo runs the block everywhere instead of nowhere.
        let typo = Scanners::parse(
            "[X]\nrange = 1 - 2 MHz\nspan = 1 kHz\nfront = auto\nregion = narnia\n",
        );
        assert_eq!(typo.list[0].regions, []);
    }

    /// A list too long for a line, and a unit written once at the end.
    ///
    /// Both are how a person writes ten frequencies, and both used to be read
    /// wrong in silence: the wrapped half was skipped as a line with no `=`
    /// in it, and everything before the unit was read as hertz and fell out
    /// of the band. Ten channels in, five out, nothing saying which five.
    #[test]
    fn a_wrapped_line_and_a_unit_at_the_end_are_read_whole() {
        let t = Scanners::parse(
            "[VDL2]\n\
             range    = 136.05 - 137 MHz\n\
             span     = 400 kHz\n\
             front    = vdl2\n\
             channels = 136.1 MHz, 136.65 MHz, 136.675 MHz, 136.7 MHz, 136.725 MHz,\n\
                        136.775 MHz, 136.8 MHz, 136.825 MHz, 136.875 MHz, 136.975 MHz\n\
             margin   = 25 kHz\n",
        );
        assert_eq!(t.list.len(), 1);
        let mhz: Vec<f64> = t.list[0].channels.iter().map(|c| (c / 1e3).round() / 1e3).collect();
        assert_eq!(
            mhz,
            [136.1, 136.65, 136.675, 136.7, 136.725, 136.775, 136.8, 136.825, 136.875, 136.975]
        );

        // The unit once at the end, the way `range` already takes it.
        let terse = Scanners::parse(
            "[X]\nrange = 136.05 - 137 MHz\nspan = 400 kHz\nfront = vdl2\n\
             channels = 136.1, 136.65, 136.975 MHz\n",
        );
        assert_eq!(terse.list[0].channels, [136_100_000.0, 136_650_000.0, 136_975_000.0]);

        // And a wrap with the comma at the start of the next line, which is
        // the other way a person breaks a list.
        let other = Scanners::parse(
            "[X]\nrange = 136.05 - 137 MHz\nspan = 400 kHz\nfront = vdl2\n\
             channels = 136.1 MHz\n             , 136.975 MHz\n",
        );
        assert_eq!(other.list[0].channels, [136_100_000.0, 136_975_000.0]);
    }

    /// Every VDL Mode 2 channel in use, under the regulator that uses it.
    ///
    /// The wiki lists ten: five European, four North American and the common
    /// signalling channel every ground station carries. Five were shipped and
    /// the rest of the plan was not written down anywhere, so an American
    /// receiver decoded one channel of the five it has.
    #[test]
    fn the_vdl2_plan_is_written_down_for_both_sides_of_the_atlantic() {
        use crate::bands::Plan;
        let s = Scanners::default();
        let on = |plan: Plan, c: f64, r: f64| -> Vec<f64> {
            let mut out: Vec<f64> = s
                .fronts_in(plan, c, r)
                .into_iter()
                .filter_map(|f| match f.front {
                    Front::Protocol { id, hz } if id == "vdl2" => Some((hz / 1e3).round() / 1e3),
                    _ => None,
                })
                .collect();
            out.sort_by(f64::total_cmp);
            out.dedup();
            out
        };
        // A wide span over the whole plan, in each region.
        assert_eq!(
            on(Plan::Europe, 136_800_000.0, 1_000_000.0),
            [136.675, 136.725, 136.775, 136.825, 136.875, 136.975]
        );
        assert_eq!(
            on(Plan::Americas, 136_500_000.0, 1_800_000.0),
            [136.1, 136.65, 136.7, 136.8, 136.975]
        );
        // The common signalling channel is in both, so it is decoded
        // wherever the receiver is.
        for plan in [Plan::Europe, Plan::Americas, Plan::AsiaPacific] {
            assert!(on(plan, 136_975_000.0, 400_000.0).contains(&136.975), "{plan:?}");
        }
        // 136.1 is nearly a megahertz below the rest and is only ever reached
        // on its own or on a wide span.
        assert_eq!(on(Plan::Americas, 136_100_000.0, 400_000.0), [136.1]);
        assert!(on(Plan::Europe, 136_100_000.0, 400_000.0).is_empty(), "not a European channel");
    }

    /// The point of the change: a band nobody declared runs nothing, instead
    /// of sweeping the span for sensors that are not there.
    #[test]
    fn a_band_with_no_scanner_runs_nothing() {
        let s = Scanners::default();
        assert!(s.fronts(95_800_000.0, 2_400_000.0).is_empty(), "FM broadcast");
        assert!(s.fronts(124_000_000.0, 2_400_000.0).is_empty(), "airband");
        assert!(s.fronts(145_500_000.0, 200_000.0).is_empty(), "2 m voice");
        // Widen that last span until it reaches the packet channel 700 kHz
        // away, though, and APRS runs: the receiver is sampling it either
        // way, and the dial is only where somebody is looking.
        assert_eq!(
            kinds(&s.fronts(145_500_000.0, 2_400_000.0)),
            [Front::protocol("aprs", 144_800_000.0), Front::protocol("sstv", 144_500_000.0)]
        );
    }

    /// A span too narrow for the front end is not that front end.
    #[test]
    fn a_span_the_front_end_cannot_work_in_does_not_match() {
        let s = Scanners::default();
        // Mode S bits are 1 us wide and need 2 MS/s.
        assert!(s.fronts(1_090_000_000.0, 1_024_000.0).is_empty());
        assert_eq!(
            kinds(&s.fronts(1_090_000_000.0, 2_048_000.0)),
            [Front::named("mode_s").unwrap()]
        );
    }

    /// The channel test is what the AIS gate used to be: both channels have to
    /// clear the span edge, not merely be nearer than half the span.
    #[test]
    fn a_span_holding_only_one_ais_channel_does_not_match() {
        let s = Scanners::default();
        // Centred on one channel with 60 kHz: the other is 50 kHz away and
        // the span reaches only 30 kHz, so it is outside.
        assert!(s.fronts(161_975_000.0, 60_000.0).is_empty());
        assert_eq!(kinds(&s.fronts(162_000_000.0, 200_000.0)), [Front::named("ais").unwrap()]);
    }

    /// The span decides, not the dial. A pager channel 200 kHz off the
    /// centre of a 2.4 MS/s span is being sampled, and a front end that
    /// waits to be tuned to it is discarding a signal it already has.
    #[test]
    fn a_channel_off_the_centre_but_inside_the_span_still_runs() {
        let s = Scanners::default();
        // Tuned 200 kHz below the DAPNET channel, which the old rule would
        // have refused because the dial sits outside the block's range.
        assert_eq!(
            kinds(&s.fronts(439_787_500.0, 2_400_000.0)),
            [Front::protocol("pocsag", 439_987_500.0)]
        );
        // And AIS from a dial parked on marine voice a megahertz away.
        assert_eq!(kinds(&s.fronts(161_000_000.0, 2_400_000.0)), [Front::named("ais").unwrap()]);
    }

    /// Everything the span covers runs. Which of two protocols a receiver
    /// hears should not depend on which block was written first.
    #[test]
    fn a_span_holding_two_blocks_runs_both() {
        let s = Scanners::parse(
            "[Pagers]\nrange = 153 - 154 MHz\nspan = 100 kHz\nfront = pocsag\n\
             channels = 153.35 MHz\nmargin = 12.5 kHz\n\
             [Packet]\nrange = 153.5 - 153.6 MHz\nspan = 48 kHz\nfront = aprs\n\
             channels = 153.55 MHz\nmargin = 8 kHz\n",
        );
        let fronts = kinds(&s.fronts(153_450_000.0, 1_000_000.0));
        assert_eq!(
            fronts,
            [Front::protocol("pocsag", 153_350_000.0), Front::protocol("aprs", 153_550_000.0)]
        );
    }

    /// Two blocks asking for the same thing are one front end. A duplicate
    /// would be a second demodulator on the same channel, reporting every
    /// packet twice.
    #[test]
    fn identical_front_ends_are_not_built_twice() {
        let s = Scanners::parse(
            "[A]\nrange = 433 - 435 MHz\nspan = 250 kHz\nfront = banks\nwidths = 20 kHz\n\
             [B]\nrange = 433.5 - 434 MHz\nspan = 250 kHz\nfront = banks\nwidths = 20 kHz\n",
        );
        assert_eq!(s.active(433_900_000.0, 1_000_000.0).len(), 2, "both blocks match");
        let got = s.fronts(433_900_000.0, 1_000_000.0);
        assert_eq!(got.len(), 1, "one bank, not two decoding the same signals");
        assert_eq!(got[0].front, Front::Banks(vec![20_000.0]));
        // The nested block widens nothing: the union is the outer range.
        assert_eq!(got[0].band, (433e6, 435e6));
    }

    /// The case the file exists for.
    #[test]
    fn a_hand_written_block_scans_somewhere_new() {
        let s = Scanners::parse(
            "[Doorbells]\nrange = 314 - 316 MHz\nspan = 250 kHz\nfront = banks\nwidths = 20 kHz\n",
        );
        assert_eq!(s.list.len(), 1);
        let hit = *s.active(315_000_000.0, 1_000_000.0).first().expect("the block should match");
        assert_eq!(hit.name, "Doorbells");
        assert_eq!(hit.front, Front::Banks(vec![20_000.0]));
    }

    /// Moving APRS to North America is one line, which is the test that says
    /// the frequency really is data and not code.
    ///
    /// The front end has to carry the channel too, not only gate on it. A
    /// block that matches at 144.390 and then demodulates 144.800 hears
    /// nothing, and an empty packet list looks exactly like a quiet band.
    #[test]
    fn aprs_can_be_moved_to_another_region() {
        let s = Scanners::parse(
            "[APRS]\nrange = 144.38 - 144.40 MHz\nspan = 48 kHz\nfront = aprs\n\
             channels = 144.390 MHz\nmargin = 8 kHz\n",
        );
        assert_eq!(
            kinds(&s.fronts(144_390_000.0, 500_000.0)),
            [Front::protocol("aprs", 144_390_000.0)]
        );
        assert!(s.fronts(144_800_000.0, 500_000.0).is_empty(), "the European one is gone");
    }

    /// The channel a POCSAG block names is the channel the demodulator tunes,
    /// because paging allocations are national and nothing sensible can be
    /// compiled in. This is the test that says the frequency really is data.
    #[test]
    fn a_pocsag_block_carries_its_channel_into_the_front_end() {
        let s = Scanners::parse(
            "[Pagers]\nrange = 153 - 154 MHz\nspan = 100 kHz\nfront = pocsag\n\
             channels = 153.35 MHz\nmargin = 12.5 kHz\n",
        );
        assert_eq!(s.list[0].front, Front::protocol("pocsag", 153_350_000.0));
        // And in the other order, since a hand-written block may write
        // either key first.
        let s = Scanners::parse(
            "[Pagers]\nchannels = 153.35 MHz\nrange = 153 - 154 MHz\nspan = 100 kHz\n\
             front = pocsag\n",
        );
        assert_eq!(s.list[0].front, Front::protocol("pocsag", 153_350_000.0));
    }

    #[test]
    fn units_are_read_on_either_side_or_once_at_the_end() {
        assert_eq!(hz("162.025 MHz"), Some(162_025_000.0));
        assert_eq!(hz("150 kHz"), Some(150_000.0));
        assert_eq!(hz("48000"), Some(48_000.0));
        let s = Scanners::parse("[x]\nrange = 433.05 - 434.79 MHz\nspan = 250 kHz\n");
        assert_eq!(s.list[0].lo, 433_050_000.0, "the unit carries to the lower bound");
        assert_eq!(s.list[0].hi, 434_790_000.0);
    }

    #[test]
    fn a_broken_or_empty_file_is_not_fatal() {
        assert!(Scanners::parse("").list.is_empty());
        assert!(Scanners::parse("nonsense\n[unclosed\nrange = banana").list.is_empty());
        // A block with no range would match everything or nothing.
        assert!(Scanners::parse("[x]\nfront = ais\n").list.is_empty());
        // An unknown key is ignored, so a later version's file still loads.
        let s = Scanners::parse("[x]\nrange = 1 - 2 MHz\nfuture = 7\nfront = ais\n");
        assert_eq!(s.list.len(), 1);
    }

    /// The interface writes this file, so what it writes has to read back as
    /// what it had. Without this a block edited in the interface can come
    /// back subtly different, or not at all.
    #[test]
    fn the_table_round_trips_through_the_file_it_writes() {
        let s = Scanners::default();
        // The blocks, not the version: what is written is always this
        // build's, which is how the file records that it has seen the
        // shipped table.
        assert_eq!(Scanners::parse(&s.render()).list, s.list);
        assert_eq!(Scanners::parse(&s.render()).version, VERSION);
    }

    #[test]
    fn a_hand_written_block_survives_being_rewritten() {
        let s = Scanners::parse(
            "[Doorbells]\nrange = 314 - 316 MHz\nspan = 250 kHz\nfront = banks\n\
             widths = 20 kHz\n[Weather]\nrange = 868 - 869 MHz\nspan = 250 kHz\n\
             front = ais\nchannels = 868.3 MHz\nmargin = 12.5 kHz\n",
        );
        assert_eq!(Scanners::parse(&s.render()).list, s.list);
        assert_eq!(s.list.len(), 2);
    }

    /// Blocks are reported in file order, which is the order the front ends
    /// are built in and the order the interface lists them.
    #[test]
    fn matching_blocks_come_back_in_the_order_they_were_written() {
        let s = Scanners::parse(
            "[first]\nrange = 100 - 200 MHz\nspan = 1 kHz\nfront = aprs\n\
             channels = 150 MHz\n\
             [second]\nrange = 100 - 200 MHz\nspan = 1 kHz\nfront = ais\n",
        );
        let names: Vec<&str> =
            s.active(150_000_000.0, 1_000_000.0).iter().map(|x| x.name.as_str()).collect();
        assert_eq!(names, ["first", "second"]);
    }
}
