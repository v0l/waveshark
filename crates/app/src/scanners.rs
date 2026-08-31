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
//! # Why it is not in the session file
//!
//! `session.rs` is rewritten whole every couple of seconds as settings
//! change, which would eat comments and formatting. A file a person is
//! expected to edit cannot be a file the program rewrites, so this one is
//! written once when it is missing and only read afterwards.

use std::path::PathBuf;

/// The 31.25 kHz OOK bank and the 125 kHz FSK one, which is what "banks"
/// means unless a block says otherwise.
///
/// The widths are measured rather than chosen: a 1.5 kbit/s OOK sensor
/// survives to 12.3 dB peak-to-noise in a 31 kHz channel and needs 22.9 dB in
/// a 125 kHz one, because a wide channel integrates noise across its whole
/// width while the signal occupies a sliver. FSK wants the opposite, since
/// its two tones are tens of kHz apart and a narrow channel cuts one off.
pub const DEFAULT_WIDTHS: [f64; 2] = [31_250.0, 125_000.0];

/// Which demodulator a block asks for.
#[derive(Clone, PartialEq, Debug)]
pub enum Front {
    /// Channelize the span and run the protocol tables over every channel.
    Banks(Vec<f64>),
    /// The 1090 MHz wideband envelope demodulator.
    ModeS,
    /// Both 162 MHz channels, GMSK.
    Ais,
    /// Narrowband FM into Bell 202 AFSK, and AX.25 above it.
    Aprs,
}

impl Front {
    /// The word this front end is written as in the file.
    pub fn key(&self) -> &'static str {
        match self {
            Front::Banks(_) => "banks",
            Front::ModeS => "modes",
            Front::Ais => "ais",
            Front::Aprs => "aprs",
        }
    }

    /// What it is called where a person reads it.
    pub fn label(&self) -> &'static str {
        match self {
            Front::Banks(_) => "banks",
            Front::ModeS => "mode s",
            Front::Ais => "ais",
            Front::Aprs => "aprs",
        }
    }

    /// Every front end, for a control that offers a choice of them.
    pub fn all() -> [Front; 4] {
        [
            Front::ModeS,
            Front::Ais,
            Front::Aprs,
            Front::Banks(DEFAULT_WIDTHS.to_vec()),
        ]
    }

    fn parse(s: &str) -> Option<Self> {
        match s.trim().to_ascii_lowercase().as_str() {
            "banks" | "scan" => Some(Front::Banks(DEFAULT_WIDTHS.to_vec())),
            "modes" | "mode-s" | "adsb" => Some(Front::ModeS),
            "ais" => Some(Front::Ais),
            "aprs" => Some(Front::Aprs),
            _ => None,
        }
    }
}

/// One scanner: where it applies and what it runs.
#[derive(Clone, PartialEq, Debug)]
pub struct Scanner {
    pub name: String,
    /// The dial must sit inside this for the scanner to apply.
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
    pub fn applies(&self, center: f64, rate: f64) -> bool {
        if rate < self.min_rate || center < self.lo || center > self.hi {
            return false;
        }
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
    /// `$XDG_CONFIG_HOME/super-radio/scanners`, beside the session.
    pub fn path() -> Option<PathBuf> {
        let base = std::env::var_os("XDG_CONFIG_HOME")
            .map(PathBuf::from)
            .or_else(|| std::env::var_os("HOME").map(|h| PathBuf::from(h).join(".config")))?;
        Some(base.join("super-radio").join("scanners"))
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

    /// The first scanner that applies, or none.
    ///
    /// First rather than all: two front ends on one span is a thing to say
    /// deliberately, and quietly running both because two blocks overlap is
    /// how the old default came to sweep every band it was not told about.
    pub fn resolve(&self, center: f64, rate: f64) -> Option<&Scanner> {
        self.list.iter().find(|s| s.applies(center, rate))
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
                if let Some(s) = cur.take().filter(|s| !s.channels_unset()) {
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
        if let Some(s) = cur.filter(|s| !s.channels_unset()) {
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
# super-radio scanners: what to run, and where.
#
# The first block whose range holds the dial, and whose span is wide enough,
# is the one that runs. Edit here or in the interface; the interface rewrites
# this file from its own blocks, so comments below this header are not kept.
#
#   range     the dial must sit inside this
#   span      narrowest span the front end works in
#   front     modes | ais | aprs | banks
#   channels  frequencies that must all be inside the span (optional)
#   margin    how far inside the span edge they must fall (optional)
#   widths    channel widths, for front = banks
";

/// The defaults, written out when there is no file.
///
/// This doubles as the documentation for the format, which is why it carries
/// its own comments rather than being built from struct literals.
pub const DEFAULT_TEXT: &str = "\
# super-radio scanners: what to run, and where.
#
# Written once when this file was missing, and only read afterwards, so it is
# safe to edit. Delete a block to stop it running; add one to scan somewhere
# new. The first block whose range holds the dial, and whose span is wide
# enough, is the one that runs.
#
#   range     the dial must sit inside this
#   span      narrowest span the front end works in
#   front     modes | ais | aprs | banks
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

[ISM 433]
range  = 433.05 - 434.79 MHz
span   = 250 kHz
front  = banks
widths = 31.25 kHz, 125 kHz

[ISM 868]
range  = 862 - 876 MHz
span   = 250 kHz
front  = banks
widths = 31.25 kHz, 125 kHz

[ISM 315]
range  = 314 - 316 MHz
span   = 250 kHz
front  = banks
widths = 31.25 kHz, 125 kHz
";

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_shipped_defaults_parse() {
        let s = Scanners::default();
        let names: Vec<&str> = s.list.iter().map(|x| x.name.as_str()).collect();
        assert_eq!(names, ["ADS-B", "AIS", "APRS", "ISM 433", "ISM 868", "ISM 315"]);
    }

    /// The behaviour the old hand-written gates had, now as table lookups.
    #[test]
    fn the_defaults_put_each_front_end_where_it_belongs() {
        let s = Scanners::default();
        let front = |c: f64, r: f64| s.resolve(c, r).map(|x| x.front.clone());
        assert_eq!(front(1_090_000_000.0, 2_400_000.0), Some(Front::ModeS));
        assert_eq!(front(162_000_000.0, 2_400_000.0), Some(Front::Ais));
        assert_eq!(front(144_800_000.0, 2_400_000.0), Some(Front::Aprs));
        assert!(matches!(front(433_920_000.0, 2_400_000.0), Some(Front::Banks(_))));
        assert!(matches!(front(868_300_000.0, 2_400_000.0), Some(Front::Banks(_))));
    }

    /// The point of the change: a band nobody declared runs nothing, instead
    /// of sweeping the span for sensors that are not there.
    #[test]
    fn a_band_with_no_scanner_runs_nothing() {
        let s = Scanners::default();
        assert_eq!(s.resolve(95_800_000.0, 2_400_000.0), None, "FM broadcast");
        assert_eq!(s.resolve(124_000_000.0, 2_400_000.0), None, "airband");
        assert_eq!(s.resolve(145_500_000.0, 2_400_000.0), None, "2 m voice");
    }

    /// A span too narrow for the front end is not that front end.
    #[test]
    fn a_span_the_front_end_cannot_work_in_does_not_match() {
        let s = Scanners::default();
        // Mode S bits are 1 us wide and need 2 MS/s.
        assert_eq!(s.resolve(1_090_000_000.0, 1_024_000.0), None);
        assert!(s.resolve(1_090_000_000.0, 2_048_000.0).is_some());
    }

    /// The channel test is what the AIS gate used to be: both channels have to
    /// clear the span edge, not merely be nearer than half the span.
    #[test]
    fn a_span_holding_only_one_ais_channel_does_not_match() {
        let s = Scanners::default();
        // Centred on one channel with 60 kHz: the other is 50 kHz away and
        // the span reaches only 30 kHz, so it is outside.
        assert_eq!(s.resolve(161_975_000.0, 60_000.0), None);
        assert!(s.resolve(162_000_000.0, 200_000.0).is_some());
    }

    /// The case the file exists for.
    #[test]
    fn a_hand_written_block_scans_somewhere_new() {
        let s = Scanners::parse(
            "[Doorbells]\nrange = 314 - 316 MHz\nspan = 250 kHz\nfront = banks\nwidths = 20 kHz\n",
        );
        assert_eq!(s.list.len(), 1);
        let hit = s.resolve(315_000_000.0, 1_000_000.0).expect("the block should match");
        assert_eq!(hit.name, "Doorbells");
        assert_eq!(hit.front, Front::Banks(vec![20_000.0]));
    }

    /// Moving APRS to North America is one line, which is the test that says
    /// the frequency really is data and not code.
    #[test]
    fn aprs_can_be_moved_to_another_region() {
        let s = Scanners::parse(
            "[APRS]\nrange = 144.38 - 144.40 MHz\nspan = 48 kHz\nfront = aprs\n\
             channels = 144.390 MHz\nmargin = 8 kHz\n",
        );
        assert!(s.resolve(144_390_000.0, 500_000.0).is_some());
        assert_eq!(s.resolve(144_800_000.0, 500_000.0), None, "the European one is gone");
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

    #[test]
    fn the_first_matching_block_wins() {
        let s = Scanners::parse(
            "[first]\nrange = 100 - 200 MHz\nspan = 1 kHz\nfront = aprs\n\
             [second]\nrange = 100 - 200 MHz\nspan = 1 kHz\nfront = ais\n",
        );
        assert_eq!(s.resolve(150_000_000.0, 1_000_000.0).unwrap().name, "first");
    }
}
