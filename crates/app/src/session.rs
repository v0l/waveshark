//! What the receiver was set to last time, so it comes back that way.
//!
//! Tuning a receiver is a dozen small decisions: which radio, where it points,
//! how wide, how much gain in each stage, how far out the crystal is. Making
//! the operator take all of them again at every start is the difference
//! between a instrument and a demo.
//!
//! Written as plain `key = value` lines rather than through a serialisation
//! crate. The file is a dozen scalars, it wants to survive a version bump
//! without a migration, and it is worth being editable by hand when something
//! about the saved state is what is broken. Unknown keys are ignored and
//! missing ones keep their defaults, which is what makes both of those true.

use common::GainMode;
use std::collections::BTreeMap;
use std::path::PathBuf;

/// Where the receiver starts with nothing saved.
///
/// 433.92 MHz rather than an FM broadcast station: the point of the thing is
/// digital modes, and this is where they are.
pub const DEFAULT_CENTER: f64 = 433_920_000.0;

#[derive(Clone, Debug, PartialEq)]
pub struct Session {
    /// Device label, matched against what is attached at startup. A label
    /// rather than an index, because plug order is not stable.
    pub device: Option<String>,
    pub center: f64,
    pub rate: f64,
    pub zoom: usize,
    pub fft: usize,
    /// Gain stages by driver name, re-applied once the radio is running.
    pub gains: Vec<(String, GainMode)>,
    /// Named switches: bias tee, digital AGC and so on.
    pub toggles: Vec<(String, bool)>,
    /// List settings by driver name and the option chosen, such as which
    /// antenna port the cable is in.
    pub choices: Vec<(String, String)>,
    pub ppm: f64,
    /// Where the receiver is, in degrees. Used to resolve an aircraft's
    /// position from a single frame instead of waiting for a matching pair.
    pub location: Option<(f64, f64)>,
    /// Interface language as a BCP 47 code, empty for the system default.
    pub language: String,
    /// ISO 3166-1 country code, empty when it has never been set.
    pub country: String,
    /// Band plan identifier. Held separately from the country because it is
    /// overridable: a country sets it once and then stops having an opinion.
    pub band_plan: String,
    pub dc_block: bool,
    pub decode_on: bool,
    pub volume: f32,
    /// Packet feeds from other receivers, as `format host:port`.
    pub feeds: Vec<nodes::FeedSpec>,
    /// Whether the operator owns the shape of the graph. The graph itself is
    /// in its own file: it is a drawing, not a setting.
    pub manual_chain: bool,
}

impl Default for Session {
    fn default() -> Self {
        Self {
            device: None,
            center: DEFAULT_CENTER,
            rate: 2_304_000.0,
            zoom: 1,
            fft: 2048,
            gains: Vec::new(),
            toggles: Vec::new(),
            choices: Vec::new(),
            ppm: 0.0,
            location: None,
            language: String::new(),
            country: String::new(),
            band_plan: String::new(),
            dc_block: true,
            decode_on: true,
            volume: 0.5,
            feeds: Vec::new(),
            manual_chain: false,
        }
    }
}

impl Session {
    /// `$XDG_CONFIG_HOME/waveshark/session`, or `~/.config` when unset.
    pub fn path() -> Option<PathBuf> {
        let base = std::env::var_os("XDG_CONFIG_HOME")
            .map(PathBuf::from)
            .or_else(|| std::env::var_os("HOME").map(|h| PathBuf::from(h).join(".config")))?;
        Some(base.join("waveshark").join("session"))
    }

    /// Load, falling back to defaults for anything missing or unreadable.
    ///
    /// A corrupt session file must never stop the receiver starting: the whole
    /// file is a convenience, and refusing to run because of one is worse than
    /// any setting it could restore.
    pub fn load() -> Self {
        Self::path()
            .and_then(|p| std::fs::read_to_string(p).ok())
            .map(|s| Self::parse(&s))
            .unwrap_or_default()
    }

    pub fn save(&self) {
        let Some(path) = Self::path() else { return };
        if let Some(dir) = path.parent() {
            let _ = std::fs::create_dir_all(dir);
        }
        let _ = std::fs::write(path, self.render());
    }

    pub fn parse(text: &str) -> Self {
        let mut kv: BTreeMap<&str, &str> = BTreeMap::new();
        let mut gains = Vec::new();
        let mut toggles = Vec::new();
        let mut choices = Vec::new();
        let mut feeds = Vec::new();
        for line in text.lines() {
            let line = line.trim();
            if line.is_empty() || line.starts_with('#') {
                continue;
            }
            let Some((k, v)) = line.split_once('=') else { continue };
            let (k, v) = (k.trim(), v.trim());
            if let Some(name) = k.strip_prefix("gain.") {
                if let Some(m) = parse_gain(v) {
                    gains.push((name.to_string(), m));
                }
            } else if let Some(name) = k.strip_prefix("toggle.") {
                toggles.push((name.to_string(), v == "true"));
            } else if let Some(name) = k.strip_prefix("choice.") {
                choices.push((name.to_string(), v.to_string()));
            } else if k == "feed" {
                if let Some(f) = parse_feed(v) {
                    feeds.push(f);
                }
            } else {
                kv.insert(k, v);
            }
        }
        let d = Session::default();
        let f = |k: &str, or: f64| kv.get(k).and_then(|v| v.parse().ok()).unwrap_or(or);
        Session {
            device: kv.get("device").map(|v| v.to_string()).filter(|v| !v.is_empty()),
            center: f("center", d.center),
            rate: f("rate", d.rate),
            zoom: kv.get("zoom").and_then(|v| v.parse().ok()).unwrap_or(d.zoom),
            fft: kv.get("fft").and_then(|v| v.parse().ok()).unwrap_or(d.fft),
            gains,
            toggles,
            choices,
            ppm: f("ppm", d.ppm),
            location: match (kv.get("lat"), kv.get("lon")) {
                (Some(a), Some(o)) => a.parse().ok().zip(o.parse().ok()),
                _ => None,
            },
            language: kv.get("language").map(|v| v.to_string()).unwrap_or_default(),
            country: kv.get("country").map(|v| v.to_string()).unwrap_or_default(),
            band_plan: kv.get("band_plan").map(|v| v.to_string()).unwrap_or_default(),
            dc_block: kv.get("dc_block").map(|v| *v == "true").unwrap_or(d.dc_block),
            decode_on: kv.get("decode").map(|v| *v == "true").unwrap_or(d.decode_on),
            volume: f("volume", d.volume as f64) as f32,
            feeds,
            manual_chain: kv.get("manual_chain").map(|v| *v == "true").unwrap_or(false),
        }
    }

    pub fn render(&self) -> String {
        let mut s = String::from("# waveshark session, rewritten as settings change\n");
        if let Some(d) = &self.device {
            s.push_str(&format!("device = {d}\n"));
        }
        s.push_str(&format!("center = {:.0}\n", self.center));
        s.push_str(&format!("rate = {:.0}\n", self.rate));
        s.push_str(&format!("zoom = {}\n", self.zoom));
        s.push_str(&format!("fft = {}\n", self.fft));
        s.push_str(&format!("ppm = {}\n", self.ppm));
        if let Some((lat, lon)) = self.location {
            s.push_str(&format!("lat = {lat}\nlon = {lon}\n"));
        }
        for (k, v) in [
            ("language", &self.language),
            ("country", &self.country),
            ("band_plan", &self.band_plan),
        ] {
            if !v.is_empty() {
                s.push_str(&format!("{k} = {v}\n"));
            }
        }
        s.push_str(&format!("dc_block = {}\n", self.dc_block));
        s.push_str(&format!("decode = {}\n", self.decode_on));
        s.push_str(&format!("volume = {}\n", self.volume));
        if self.manual_chain {
            s.push_str("manual_chain = true\n");
        }
        for (name, mode) in &self.gains {
            s.push_str(&format!("gain.{name} = {}\n", render_gain(*mode)));
        }
        for (name, on) in &self.toggles {
            s.push_str(&format!("toggle.{name} = {on}\n"));
        }
        for (name, value) in &self.choices {
            s.push_str(&format!("choice.{name} = {value}\n"));
        }
        for f in &self.feeds {
            s.push_str(&format!("feed = {} {}\n", f.kind.name, f.address()));
        }
        s
    }
}

/// `beast host:port`, as written by `render`. An unknown kind is dropped
/// rather than fatal: a session written by a later version has to load.
fn parse_feed(v: &str) -> Option<nodes::FeedSpec> {
    let (kind, addr) = v.split_once(char::is_whitespace)?;
    let kind = nodes::feed_kind(kind.trim())?;
    let (host, port) = addr.trim().rsplit_once(':')?;
    Some(nodes::FeedSpec::new(host, port.parse().ok()?, kind))
}

fn parse_gain(v: &str) -> Option<GainMode> {
    if v == "auto" {
        return Some(GainMode::Auto);
    }
    v.parse().ok().map(GainMode::Manual)
}

fn render_gain(m: GainMode) -> String {
    match m {
        GainMode::Auto => "auto".into(),
        GainMode::Manual(v) => format!("{v}"),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_session_round_trips_through_the_file_format() {
        let s = Session {
            device: Some("RTL2838 #00000001".into()),
            center: 433_920_000.0,
            rate: 2_048_000.0,
            zoom: 4,
            fft: 4096,
            gains: vec![
                ("tuner".into(), GainMode::Manual(29.7)),
                ("lna".into(), GainMode::Auto),
            ],
            toggles: vec![("bias_tee".into(), true)],
            choices: vec![("antenna".into(), "LNAH".into())],
            ppm: -3.5,
            location: Some((53.5137, -6.2431)),
            language: "en".into(),
            country: "IE".into(),
            band_plan: "europe".into(),
            dc_block: false,
            decode_on: false,
            volume: 0.25,
            manual_chain: true,
            feeds: vec![
                nodes::FeedSpec::new("10.100.2.249", 30005, &nodes::feed_nodes::BEAST),
                nodes::FeedSpec::new("pi.local", 30002, &nodes::feed_nodes::AVR),
            ],
        };
        assert_eq!(Session::parse(&s.render()), s);
    }

    #[test]
    fn an_empty_or_broken_file_gives_the_defaults() {
        // The session is a convenience. Refusing to start because of it, or
        // starting somewhere unexpected, are both worse than ignoring it.
        assert_eq!(Session::parse(""), Session::default());
        assert_eq!(Session::parse("nonsense\n\x00\ncenter = banana"), Session::default());
        assert_eq!(Session::parse("").center, DEFAULT_CENTER);
    }

    #[test]
    fn unknown_keys_are_ignored_rather_than_fatal() {
        // A file written by a later version has to load in an earlier one,
        // or a downgrade loses every setting rather than the new ones.
        let s = Session::parse("center = 868300000\nfuture_setting = 7\n");
        assert_eq!(s.center, 868_300_000.0);
        assert_eq!(s.rate, Session::default().rate);
    }

    #[test]
    fn gain_stages_keep_their_names_and_modes() {
        let s = Session::parse("gain.tuner = 29.7\ngain.if = auto\n");
        assert_eq!(
            s.gains,
            vec![("tuner".into(), GainMode::Manual(29.7)), ("if".into(), GainMode::Auto)]
        );
    }
}
