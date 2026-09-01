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
    /// Channelize the span and run the protocol tables over every channel.
    Banks(Vec<f64>),
    /// The 1090 MHz wideband envelope demodulator.
    ModeS,
    /// Both 162 MHz channels, GMSK.
    Ais,
    /// Narrowband FM into Bell 202 AFSK, and AX.25 above it. Carries the
    /// channel, since APRS is on a different frequency in each region.
    Aprs(f64),
    /// One paging channel, NRZ FSK at whichever of the three POCSAG rates it
    /// turns out to be. Carries the channel, because paging allocations are
    /// national and there is no frequency worth compiling in.
    Pocsag(f64),
}

/// The amateur DAPNET channel, used until a block says otherwise. Amateur
/// rather than commercial because it is the one paging frequency that is the
/// same across Europe.
pub const DEFAULT_POCSAG_HZ: f64 = 439_987_500.0;

/// Where APRS is across Europe, used until a block says otherwise. North
/// America is 144.390 and Japan 144.640.
pub const DEFAULT_APRS_HZ: f64 = 144_800_000.0;

impl Front {
    /// The word this front end is written as in the file.
    pub fn key(&self) -> &'static str {
        match self {
            Front::Banks(_) => "banks",
            Front::ModeS => "modes",
            Front::Ais => "ais",
            Front::Aprs(_) => "aprs",
            Front::Pocsag(_) => "pocsag",
        }
    }

    /// What it is called where a person reads it.
    pub fn label(&self) -> &'static str {
        match self {
            Front::Banks(_) => "banks",
            Front::ModeS => "mode s",
            Front::Ais => "ais",
            Front::Aprs(_) => "aprs",
            Front::Pocsag(_) => "pager",
        }
    }

    /// Every front end, for a control that offers a choice of them.
    pub fn all() -> [Front; 5] {
        [
            Front::ModeS,
            Front::Ais,
            Front::Aprs(DEFAULT_APRS_HZ),
            Front::Pocsag(DEFAULT_POCSAG_HZ),
            Front::Banks(DEFAULT_WIDTHS.to_vec()),
        ]
    }

    fn parse(s: &str) -> Option<Self> {
        match s.trim().to_ascii_lowercase().as_str() {
            "banks" | "scan" => Some(Front::Banks(DEFAULT_WIDTHS.to_vec())),
            "modes" | "mode-s" | "adsb" => Some(Front::ModeS),
            "ais" => Some(Front::Ais),
            "aprs" => Some(Front::Aprs(DEFAULT_APRS_HZ)),
            "pocsag" | "pager" => Some(Front::Pocsag(DEFAULT_POCSAG_HZ)),
            _ => None,
        }
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
        let (lo, hi) =
            (self.band.0.max(center - rate / 2.0), self.band.1.min(center + rate / 2.0));
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
}

impl Scanner {
    /// Whether this scanner applies to a tuning.
    ///
    /// What decides it is whether the span covers what the block needs, not
    /// where the dial happens to sit. A receiver on 20 MS/s at 440 MHz has
    /// the pager channel in front of it whether or not the dial is parked on
    /// it, and a front end that waits to be tuned to a frequency it is
    /// already sampling is throwing the signal away.
    pub fn applies(&self, center: f64, rate: f64) -> bool {
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
        self.channels.iter().all(|c| (c - center).abs() <= edge)
    }
}

/// The scanners, in the order they are consulted.
#[derive(Clone, PartialEq, Debug)]
pub struct Scanners {
    pub list: Vec<Scanner>,
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

    /// Load, writing the defaults out first if there is no file yet.
    ///
    /// Writing them is the point: a table nobody can see is not configurable,
    /// and the shipped blocks are the worked examples for adding another.
    pub fn load() -> Self {
        let Some(path) = Self::path() else { return Self::default() };
        match std::fs::read_to_string(&path) {
            Ok(text) => Self::parse(&text),
            Err(_) => {
                if let Some(dir) = path.parent() {
                    let _ = std::fs::create_dir_all(dir);
                }
                let _ = std::fs::write(&path, DEFAULT_TEXT);
                Self::default()
            }
        }
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
        self.list.iter().filter(|s| s.applies(center, rate)).collect()
    }

    /// The front ends the span covers, deduplicated.
    ///
    /// Two blocks that ask for the same thing are one front end: a duplicate
    /// would be a second demodulator on the same channel producing the same
    /// packets twice.
    pub fn fronts(&self, center: f64, rate: f64) -> Vec<FrontAt> {
        let mut out: Vec<FrontAt> = Vec::new();
        for s in self.active(center, rate) {
            let band = (s.lo, s.hi);
            let touching = |a: (f64, f64), b: (f64, f64)| a.0 <= b.1 && b.0 <= a.1;
            if matches!(s.front, Front::Banks(_)) {
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
        for sc in &self.list {
            s.push_str(&format!("\n[{}]\n", sc.name));
            s.push_str(&format!(
                "range = {} - {} MHz\n",
                num(sc.lo / 1e6),
                num(sc.hi / 1e6)
            ));
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
        let mut cur: Option<Scanner> = None;
        for line in text.lines() {
            let line = line.split('#').next().unwrap_or("").trim();
            if line.is_empty() {
                continue;
            }
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
                    front: Front::Banks(DEFAULT_WIDTHS.to_vec()),
                });
                continue;
            }
            let (Some(s), Some((k, v))) = (cur.as_mut(), line.split_once('=')) else { continue };
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
                "channels" => {
                    s.channels = v.split(',').filter_map(hz).collect();
                }
                "margin" => s.margin_hz = hz(v).unwrap_or(0.0),
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
        Self { list }
    }
}

impl Scanner {
    /// A block with no usable range is a block that would match everything or
    /// nothing, and both are worse than dropping it.
    fn channels_unset(&self) -> bool {
        self.hi <= self.lo
    }

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
    fn settle(&mut self) {
        let Some(&c) = self.channels.first() else { return };
        match &mut self.front {
            Front::Aprs(f) | Front::Pocsag(f) => *f = c,
            _ => {}
        }
    }
}

/// The unit suffix of a value, so `433.05 - 434.79 MHz` can write it once.
fn unit_of(s: &str) -> f64 {
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
fn hz(s: &str) -> Option<f64> {
    hz_with(s, unit_of(s))
}

fn hz_with(s: &str, unit: f64) -> Option<f64> {
    let t = s.trim().trim_end_matches(|c: char| c.is_ascii_alphabetic()).trim();
    let v: f64 = t.parse().ok()?;
    Some(v * if unit_of(s) == 1.0 { unit } else { unit_of(s) })
}

/// Print a frequency without trailing zeros: 162.025, not 162.025000.
fn num(v: f64) -> String {
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
#   front     modes | ais | aprs | pocsag | banks
#   channels  frequencies that must all be inside the span (optional)
#   margin    how far inside the span edge they must fall (optional)
#   widths    channel widths, for front = banks
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
#   front     modes | ais | aprs | pocsag | banks
#   channels  frequencies that must all be inside the span (optional)
#   margin    how far inside the span edge they must fall (optional)
#   widths    channel widths, for front = banks

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

[POCSAG]
# The amateur DAPNET network, which runs POCSAG at 1200 baud and is the one
# paging channel that is the same across Europe. Commercial paging is
# national: 138 to 153 MHz in the UK, 929 to 932 MHz in the United States,
# 450 to 470 MHz in much of Europe. Point this at a channel you can hear, and
# read the note about pager traffic in docs/protocols.md before logging it.
range    = 439.9 - 440.1 MHz
span     = 100 kHz
front    = pocsag
channels = 439.9875 MHz
margin   = 12.5 kHz

[ISM 433]
range  = 433.05 - 434.79 MHz
span   = 250 kHz
front  = banks
widths = 12.5 kHz, 31.25 kHz, 125 kHz, 500 kHz

[ISM 868]
range  = 862 - 876 MHz
span   = 250 kHz
front  = banks
widths = 12.5 kHz, 31.25 kHz, 125 kHz, 500 kHz

[ISM 315]
range  = 314 - 316 MHz
span   = 250 kHz
front  = banks
widths = 12.5 kHz, 31.25 kHz, 125 kHz, 500 kHz
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
    fn the_shipped_defaults_parse() {
        let s = Scanners::default();
        let names: Vec<&str> = s.list.iter().map(|x| x.name.as_str()).collect();
        assert_eq!(names, ["ADS-B", "AIS", "APRS", "POCSAG", "ISM 433", "ISM 868", "ISM 315"]);
    }

    /// The behaviour the old hand-written gates had, now as table lookups.
    #[test]
    fn the_defaults_put_each_front_end_where_it_belongs() {
        let s = Scanners::default();
        let fronts = |c: f64, r: f64| -> Vec<Front> {
            s.fronts(c, r).into_iter().map(|f| f.front).collect()
        };
        assert_eq!(fronts(1_090_000_000.0, 2_400_000.0), [Front::ModeS]);
        assert_eq!(fronts(162_000_000.0, 2_400_000.0), [Front::Ais]);
        assert_eq!(fronts(144_800_000.0, 2_400_000.0), [Front::Aprs(144_800_000.0)]);
        assert_eq!(fronts(439_987_500.0, 500_000.0), [Front::Pocsag(439_987_500.0)]);
        assert!(matches!(fronts(433_920_000.0, 2_400_000.0)[..], [Front::Banks(_)]));
        assert!(matches!(fronts(868_300_000.0, 2_400_000.0)[..], [Front::Banks(_)]));
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
        assert_eq!(kinds(&s.fronts(145_500_000.0, 2_400_000.0)), [Front::Aprs(144_800_000.0)]);
    }

    /// A span too narrow for the front end is not that front end.
    #[test]
    fn a_span_the_front_end_cannot_work_in_does_not_match() {
        let s = Scanners::default();
        // Mode S bits are 1 us wide and need 2 MS/s.
        assert!(s.fronts(1_090_000_000.0, 1_024_000.0).is_empty());
        assert_eq!(kinds(&s.fronts(1_090_000_000.0, 2_048_000.0)), [Front::ModeS]);
    }

    /// The channel test is what the AIS gate used to be: both channels have to
    /// clear the span edge, not merely be nearer than half the span.
    #[test]
    fn a_span_holding_only_one_ais_channel_does_not_match() {
        let s = Scanners::default();
        // Centred on one channel with 60 kHz: the other is 50 kHz away and
        // the span reaches only 30 kHz, so it is outside.
        assert!(s.fronts(161_975_000.0, 60_000.0).is_empty());
        assert_eq!(kinds(&s.fronts(162_000_000.0, 200_000.0)), [Front::Ais]);
    }

    /// The span decides, not the dial. A pager channel 200 kHz off the
    /// centre of a 2.4 MS/s span is being sampled, and a front end that
    /// waits to be tuned to it is discarding a signal it already has.
    #[test]
    fn a_channel_off_the_centre_but_inside_the_span_still_runs() {
        let s = Scanners::default();
        // Tuned 200 kHz below the DAPNET channel, which the old rule would
        // have refused because the dial sits outside the block's range.
        assert_eq!(kinds(&s.fronts(439_787_500.0, 2_400_000.0)), [Front::Pocsag(439_987_500.0)]);
        // And AIS from a dial parked on marine voice a megahertz away.
        assert_eq!(kinds(&s.fronts(161_000_000.0, 2_400_000.0)), [Front::Ais]);
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
        assert_eq!(fronts, [Front::Pocsag(153_350_000.0), Front::Aprs(153_550_000.0)]);
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
        assert_eq!(kinds(&s.fronts(144_390_000.0, 500_000.0)), [Front::Aprs(144_390_000.0)]);
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
        assert_eq!(s.list[0].front, Front::Pocsag(153_350_000.0));
        // And in the other order, since a hand-written block may write
        // either key first.
        let s = Scanners::parse(
            "[Pagers]\nchannels = 153.35 MHz\nrange = 153 - 154 MHz\nspan = 100 kHz\n\
             front = pocsag\n",
        );
        assert_eq!(s.list[0].front, Front::Pocsag(153_350_000.0));
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
        assert_eq!(Scanners::parse(&s.render()), s);
    }

    #[test]
    fn a_hand_written_block_survives_being_rewritten() {
        let s = Scanners::parse(
            "[Doorbells]\nrange = 314 - 316 MHz\nspan = 250 kHz\nfront = banks\n\
             widths = 20 kHz\n[Weather]\nrange = 868 - 869 MHz\nspan = 250 kHz\n\
             front = ais\nchannels = 868.3 MHz\nmargin = 12.5 kHz\n",
        );
        assert_eq!(Scanners::parse(&s.render()), s);
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
