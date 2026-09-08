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

/// What the radio is set to, as the operator set it.
///
/// One record, owned by the interface, that every route to a radio setting
/// goes through: the settings pane changes a field and applies, a restore
/// applies the whole thing, a device change or a reset applies it again. The
/// radio's own report of its state is for display and is never the source,
/// because it lags what was asked for: on connect the session was once
/// written from a status that still said zero, before the restore had
/// reached the driver, and the saved values went with it.
#[derive(Clone, Debug, PartialEq, Default)]
pub struct RadioSettings {
    /// Gain stages by driver name.
    pub gains: Vec<(String, GainMode)>,
    /// Named switches: bias tee, digital AGC and so on.
    pub toggles: Vec<(String, bool)>,
    /// List settings by driver name and the option chosen, such as which
    /// antenna port the cable is in.
    pub choices: Vec<(String, String)>,
    pub ppm: f64,
    /// Transmit gain in dB, which the radio's transmit stages are set to
    /// when a channel is keyed.
    pub tx_gain_db: f32,
}

impl RadioSettings {
    /// Set one gain stage, replacing any earlier setting of the same stage.
    pub fn set_gain(&mut self, name: &str, mode: GainMode) {
        self.gains.retain(|(n, _)| n != name);
        self.gains.push((name.to_string(), mode));
    }

    pub fn set_toggle(&mut self, name: &str, on: bool) {
        self.toggles.retain(|(n, _)| n != name);
        self.toggles.push((name.to_string(), on));
    }

    pub fn set_choice(&mut self, name: &str, value: &str) {
        self.choices.retain(|(n, _)| n != name);
        self.choices.push((name.to_string(), value.to_string()));
    }
}

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
    /// Transmit gain in dB, which the radio's own transmit stages are set to
    /// when a channel is keyed.
    pub tx_gain_db: f32,
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
    /// OpenCelliD download token, empty when none has been given. Held with
    /// the settings rather than in the cache directory: deleting the cached
    /// data must not lose the credential that fetches it again.
    pub opencellid_token: String,
    pub dc_block: bool,
    pub decode_on: bool,
    pub volume: f32,
    /// Sound devices by name, empty for the system default.
    ///
    /// A name rather than an index for the same reason the radio is a label:
    /// what the host calls device 2 changes when a headset is plugged in, and
    /// a receiver that starts talking to the wrong output after a reboot is
    /// indistinguishable from one that has stopped working.
    pub audio_out: String,
    pub audio_in: String,
    /// What the packet log folder and the raw capture folder may take, in
    /// megabytes, or `None` for no limit. Absent from an older file means
    /// the default, and a limit set once should not need setting again.
    pub log_cap_mb: Option<u64>,
    pub capture_cap_mb: Option<u64>,
    /// Where the receiver's own position is read from, as it was written on
    /// the command line: a serial port or a gpsd address. A survey started
    /// once should not need its GPS naming again at every start.
    pub gps: String,
    /// The wigle.net account the wardriving feed uploads as, and whether it
    /// is running. The token is a credential in a plain text file, which is
    /// said in the dialog that takes it: the alternative is a keyring this
    /// program has no other use for.
    pub wigle_name: String,
    pub wigle_token: String,
    pub wigle_donate: bool,
    pub wigle_on: bool,
    /// Whether observations are submitted to beacondb.net. No credential:
    /// beaconDB takes them from anybody, so this is the whole setting.
    pub beacondb_on: bool,
    /// Whether the map may ask beaconDB where a decoded cell is. Apart from
    /// the feed: asking tells beaconDB which cells this receiver heard, and
    /// giving is not the same decision as asking.
    pub beacondb_lookup: bool,
    /// How the spectrum and the waterfall are drawn.
    ///
    /// Kept here with the rest of it because they are settings in the same
    /// sense the gain is: an operator picks a scroll rate and a scale to suit
    /// the band being watched, and having to pick them again at every start is
    /// the difference between an instrument and a demo.
    pub view: ViewPrefs,
    /// Packet feeds from other receivers, as `format host:port`.
    pub feeds: Vec<nodes::FeedSpec>,
    /// iqstream servers to offer as radios, as `host:port` and the name given
    /// to that receiver. Configuration rather than discovery: nothing on the
    /// bus says a tuner is on the network.
    pub streams: Vec<(String, String)>,
    /// Whether the operator owns the shape of the graph. The graph itself is
    /// in its own file: it is a drawing, not a setting.
    pub manual_chain: bool,
    /// Map reference layers by name, and whether each is drawn. Named rather
    /// than positional so a layer added later keeps its own default instead
    /// of inheriting a stale flag.
    pub map_layers: Vec<(String, bool)>,
}

/// Spectrum and waterfall settings, as the two panes' own panels set them.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ViewPrefs {
    /// Waterfall rows drawn per second.
    pub rows_per_sec: f32,
    /// Rows of history the waterfall keeps.
    pub wf_rows: usize,
    /// How far below the trace ceiling the hottest colour sits, in dB.
    pub wf_top_offset: f32,
    /// Whether the scale follows the signal, and where it sits when it does
    /// not. Both are saved: an operator who turned auto off did so to hold a
    /// particular window on the noise floor.
    pub auto_scale: bool,
    pub floor: f32,
    pub ceil: f32,
    /// Spectrum frames per second, and the averaging applied to them.
    pub refresh: f32,
    pub smoothing: f32,
}

impl Default for ViewPrefs {
    fn default() -> Self {
        Self {
            rows_per_sec: 20.0,
            wf_rows: 512,
            wf_top_offset: 5.0,
            auto_scale: true,
            floor: -90.0,
            ceil: -20.0,
            refresh: 30.0,
            smoothing: 0.35,
        }
    }
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
            tx_gain_db: 0.0,
            location: None,
            language: String::new(),
            country: String::new(),
            band_plan: String::new(),
            opencellid_token: String::new(),
            dc_block: true,
            decode_on: true,
            volume: 0.5,
            audio_out: String::new(),
            audio_in: String::new(),
            log_cap_mb: Some(crate::packetlog::DEFAULT_MAX_BYTES >> 20),
            capture_cap_mb: Some(nodes::capture_nodes::DEFAULT_BUDGET >> 20),
            gps: String::new(),
            wigle_name: String::new(),
            wigle_token: String::new(),
            wigle_donate: false,
            wigle_on: false,
            beacondb_on: false,
            beacondb_lookup: false,
            view: ViewPrefs::default(),
            feeds: Vec::new(),
            streams: Vec::new(),
            manual_chain: false,
            map_layers: Vec::new(),
        }
    }
}

impl Session {
    /// The radio's part of the session.
    pub fn radio(&self) -> RadioSettings {
        RadioSettings {
            gains: self.gains.clone(),
            toggles: self.toggles.clone(),
            choices: self.choices.clone(),
            ppm: self.ppm,
            tx_gain_db: self.tx_gain_db,
        }
    }

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
        let mut streams = Vec::new();
        let mut map_layers = Vec::new();
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
            } else if let Some(name) = k.strip_prefix("map_layer.") {
                map_layers.push((name.to_string(), v == "true"));
            } else if k == "feed" {
                if let Some(f) = parse_feed(v) {
                    feeds.push(f);
                }
            } else if k == "stream" {
                // `host:port name of the receiver`, the name being everything
                // after the first space and often absent.
                match v.split_once(char::is_whitespace) {
                    Some((addr, name)) => {
                        streams.push((addr.to_string(), name.trim().to_string()))
                    }
                    None if !v.is_empty() => streams.push((v.to_string(), String::new())),
                    None => {}
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
            tx_gain_db: f("tx_gain_db", d.tx_gain_db as f64) as f32,
            location: match (kv.get("lat"), kv.get("lon")) {
                (Some(a), Some(o)) => a.parse().ok().zip(o.parse().ok()),
                _ => None,
            },
            language: kv.get("language").map(|v| v.to_string()).unwrap_or_default(),
            country: kv.get("country").map(|v| v.to_string()).unwrap_or_default(),
            band_plan: kv.get("band_plan").map(|v| v.to_string()).unwrap_or_default(),
            opencellid_token: kv.get("opencellid_token").map(|v| v.to_string()).unwrap_or_default(),
            dc_block: kv.get("dc_block").map(|v| *v == "true").unwrap_or(d.dc_block),
            decode_on: kv.get("decode").map(|v| *v == "true").unwrap_or(d.decode_on),
            volume: f("volume", d.volume as f64) as f32,
            audio_out: kv.get("audio_out").map(|v| v.to_string()).unwrap_or_default(),
            audio_in: kv.get("audio_in").map(|v| v.to_string()).unwrap_or_default(),
            log_cap_mb: cap(kv.get("log_cap_mb").copied(), d.log_cap_mb),
            capture_cap_mb: cap(kv.get("capture_cap_mb").copied(), d.capture_cap_mb),
            gps: kv.get("gps").map(|v| v.to_string()).unwrap_or_default(),
            wigle_name: kv.get("wigle_name").map(|v| v.to_string()).unwrap_or_default(),
            wigle_token: kv.get("wigle_token").map(|v| v.to_string()).unwrap_or_default(),
            wigle_donate: kv.get("wigle_donate").map(|v| *v == "true").unwrap_or(false),
            wigle_on: kv.get("wigle_on").map(|v| *v == "true").unwrap_or(false),
            beacondb_on: kv.get("beacondb_on").map(|v| *v == "true").unwrap_or(false),
            beacondb_lookup: kv.get("beacondb_lookup").map(|v| *v == "true").unwrap_or(false),
            view: ViewPrefs {
                rows_per_sec: f("rows_per_sec", d.view.rows_per_sec as f64).clamp(1.0, 200.0)
                    as f32,
                wf_rows: kv
                    .get("wf_rows")
                    .and_then(|v| v.parse().ok())
                    .unwrap_or(d.view.wf_rows)
                    .clamp(64, 4096),
                wf_top_offset: f("wf_contrast", d.view.wf_top_offset as f64).clamp(0.0, 20.0)
                    as f32,
                auto_scale: kv.get("auto_scale").map(|v| *v == "true").unwrap_or(d.view.auto_scale),
                floor: f("floor", d.view.floor as f64).clamp(-140.0, 20.0) as f32,
                ceil: f("ceiling", d.view.ceil as f64).clamp(-140.0, 20.0) as f32,
                refresh: f("refresh", d.view.refresh as f64).clamp(1.0, 120.0) as f32,
                smoothing: f("smoothing", d.view.smoothing as f64).clamp(0.01, 1.0) as f32,
            },
            feeds,
            streams,
            manual_chain: kv.get("manual_chain").map(|v| *v == "true").unwrap_or(false),
            map_layers,
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
        s.push_str(&format!("tx_gain_db = {}\n", self.tx_gain_db));
        if let Some((lat, lon)) = self.location {
            s.push_str(&format!("lat = {lat}\nlon = {lon}\n"));
        }
        for (k, v) in [
            ("language", &self.language),
            ("country", &self.country),
            ("band_plan", &self.band_plan),
            ("opencellid_token", &self.opencellid_token),
            ("audio_out", &self.audio_out),
            ("audio_in", &self.audio_in),
            ("gps", &self.gps),
            ("wigle_name", &self.wigle_name),
            ("wigle_token", &self.wigle_token),
        ] {
            if !v.is_empty() {
                s.push_str(&format!("{k} = {v}\n"));
            }
        }
        s.push_str(&format!("dc_block = {}\n", self.dc_block));
        s.push_str(&format!("decode = {}\n", self.decode_on));
        s.push_str(&format!("volume = {}\n", self.volume));
        s.push_str(&format!("log_cap_mb = {}\n", render_cap(self.log_cap_mb)));
        s.push_str(&format!("capture_cap_mb = {}\n", render_cap(self.capture_cap_mb)));
        let v = &self.view;
        s.push_str(&format!("rows_per_sec = {}\n", v.rows_per_sec));
        s.push_str(&format!("wf_rows = {}\n", v.wf_rows));
        s.push_str(&format!("wf_contrast = {}\n", v.wf_top_offset));
        s.push_str(&format!("auto_scale = {}\n", v.auto_scale));
        s.push_str(&format!("floor = {}\n", v.floor));
        s.push_str(&format!("ceiling = {}\n", v.ceil));
        s.push_str(&format!("refresh = {}\n", v.refresh));
        s.push_str(&format!("smoothing = {}\n", v.smoothing));
        if self.manual_chain {
            s.push_str("manual_chain = true\n");
        }
        if self.wigle_donate {
            s.push_str("wigle_donate = true\n");
        }
        if self.wigle_on {
            s.push_str("wigle_on = true\n");
        }
        if self.beacondb_on {
            s.push_str("beacondb_on = true\n");
        }
        if self.beacondb_lookup {
            s.push_str("beacondb_lookup = true\n");
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
        for (name, on) in &self.map_layers {
            s.push_str(&format!("map_layer.{name} = {on}\n"));
        }
        for f in &self.feeds {
            s.push_str(&format!("feed = {} {}\n", f.kind.name, f.address()));
        }
        for (addr, name) in &self.streams {
            if name.is_empty() {
                s.push_str(&format!("stream = {addr}\n"));
            } else {
                s.push_str(&format!("stream = {addr} {name}\n"));
            }
        }
        s
    }
}

/// A folder limit as written: megabytes, or `none`.
fn cap(v: Option<&str>, default: Option<u64>) -> Option<u64> {
    match v {
        None => default,
        Some("none") => None,
        Some(s) => s.parse().ok().or(default),
    }
}

fn render_cap(c: Option<u64>) -> String {
    c.map(|mb| mb.to_string()).unwrap_or_else(|| "none".into())
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
            tx_gain_db: 12.0,
            location: Some((53.6369, -6.6528)),
            language: "en".into(),
            audio_out: "Scarlett 2i2 Analogue".into(),
            audio_in: "Scarlett 2i2 Analogue".into(),
            country: "IE".into(),
            band_plan: "europe".into(),
            opencellid_token: "pk.0123456789".into(),
            dc_block: false,
            decode_on: false,
            volume: 0.25,
            gps: "/dev/ttyACM0@9600".into(),
            wigle_name: "AID0000".into(),
            wigle_token: "hunter2".into(),
            wigle_donate: true,
            wigle_on: true,
            beacondb_on: true,
            beacondb_lookup: true,
            log_cap_mb: None,
            capture_cap_mb: Some(16_384),
            view: ViewPrefs {
                rows_per_sec: 40.0,
                wf_rows: 1024,
                wf_top_offset: 3.0,
                auto_scale: false,
                floor: -102.5,
                ceil: -14.0,
                refresh: 60.0,
                smoothing: 0.5,
            },
            manual_chain: true,
            feeds: vec![
                nodes::FeedSpec::new("10.100.2.249", 30005, &nodes::feed_nodes::BEAST),
                nodes::FeedSpec::new("pi.local", 30002, &nodes::feed_nodes::AVR),
            ],
            streams: vec![
                ("radarpi:1234".into(), "Loft dongle".into()),
                ("10.0.0.5:1234".into(), String::new()),
            ],
            map_layers: vec![("rings".into(), true), ("airports".into(), false)],
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
