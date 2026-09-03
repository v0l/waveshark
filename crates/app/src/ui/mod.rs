//! Instrument front panel: readout, spectrum, waterfall, channel strips.

mod calls_pane;
mod chain_pane;
mod map_pane;
mod packets;
mod scope;
mod settings;
mod state;
mod widgets;

use crate::bands;
use crate::dial::Dial;
use crate::radio::{
    ChannelSpec, ChannelState, Cmd, DecodeRecord, Demod, Frame, Radio, StationInfo,
};
use crate::theme::{self, legend, value};
use widgets::{Fader, Squelch, Vu};
use common::{GainMode, Hz, Sps};
use egui::containers::{CentralPanel, Panel};
use egui::{Align2, Color32, ColorImage, FontFamily, FontId, Pos2, Rect, Sense, Stroke, StrokeKind, TextureOptions, Vec2};

pub struct App {
    /// What each view remembers. A pane is handed its own and nothing else,
    /// which is what stops one view reaching into another's business.
    scope: state::ScopeState,
    chain: state::ChainState,
    log: state::LogState,
    map: state::MapState,
    calls: state::CallsState,
    audio: state::AudioState,

    radio: Option<Radio>,
    err: Option<String>,

    center: f64,
    rate: f64,


    dial: Dial,
    open: Option<Settings>,
    devices: Vec<crate::devices::Entry>,
    device: Option<crate::devices::Entry>,
    spans: Vec<crate::devices::Span>,
    /// Software decimation currently applied, 1 for none.
    zoom: usize,
    /// Run for this many seconds, report CPU used, then quit.
    pub soak: Option<f32>,
    /// Save a PNG to this path once the radio has settled, then quit.
    pub shot: Option<String>,
    /// Seconds of running before the screenshot is taken.
    pub shot_after: f32,
    /// Where bursts are being written and how much may be written, when
    /// recording.
    record_dir: Option<(std::path::PathBuf, Option<u64>)>,
    shot_at: Option<std::time::Instant>,
    shot_sent: bool,
    /// Remove the direct-conversion centre spur. On by default: it is an
    /// artefact of the receiver, not something being received.
    dc_block: bool,
    view: View,
    /// Decoding every channel is on by default and can be turned off; it is
    /// the most expensive thing the app does.
    decode_on: bool,
    /// Whether the raw span capture is wanted. Held rather than sent once:
    /// choosing a device starts a new radio thread with a new graph, and a
    /// capture that quietly stopped there would be worse than none.
    capture: bool,
    /// Where the receiver is, when it has been told.
    location: Option<(f64, f64)>,
    /// ISO country code, or empty when nothing has chosen one.
    country: String,
    /// Packet feeds from other receivers, as configured here and saved in
    /// the session.
    feeds: Vec<nodes::FeedSpec>,
    /// The feed being typed into the settings modal.
    feed_host: String,
    /// The remote radio being created, while that dialog is open.
    remote: Option<RemoteEdit>,
    /// Where the packet log writes, as typed, and the size limit in
    /// megabytes per day, or `None` for no limit.
    /// The scanner file as text, while it is being edited. Held apart from
    /// the live table so a half-typed block does not retune the receiver.
    scanner_edit: Option<Vec<ScannerRow>>,
    /// The live table, as the radio thread has it.
    scanners: crate::scanners::Scanners,
    log_dir_edit: String,
    log_dir: Option<std::path::PathBuf>,
    log_cap_mb: Option<u64>,
    feed_kind: &'static nodes::FeedKind,
    /// The station position being typed, while it is being typed. Kept apart
    /// from the real one so a half-finished latitude does not move the map.
    station_edit: Option<String>,
    saved: crate::session::Session,
    saved_at: Option<std::time::Instant>,
    /// Gain stages and switches restored from the session, waiting for the
    /// radio to report its controls so they can be applied to it.
    pending_radio: Option<crate::session::Session>,
}

/// Which settings panel is open. Each pane owns its own, because spectrum and
/// waterfall settings are unrelated and lumping them together makes both
/// harder to find.
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Settings {
    Spectrum,
    Waterfall,
    /// The radio's own controls: gain stages, its switches, and the
    /// corrections applied to what comes off it.
    Radio,
    /// The packet log: where it is written, and what else feeds it.
    PacketLog,
    /// The scanner table: which front end runs on which frequency.
    Scanners,
    /// Everything about where this receiver is rather than what it is doing:
    /// language, country, band plan, station position.
    App,
}

/// A radio reached over the network, as the dialog that creates one asks for
/// it.
///
/// The protocol is a choice rather than an assumption: rtl_tcp and airspy's
/// own network server are the same shape of thing, and the dialog is where
/// they will be offered.
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum RemoteKind {
    IqStream,
}

impl RemoteKind {
    pub const ALL: &'static [RemoteKind] = &[RemoteKind::IqStream];

    fn label(self) -> &'static str {
        match self {
            Self::IqStream => "iqstream",
        }
    }

    fn help(self) -> &'static str {
        match self {
            Self::IqStream => {
                "One tuner shared with many readers, so a dongle already feeding a decoder \
                 elsewhere can still be listened to here. The frequency and the span belong \
                 to whoever owns that tuner and cannot be changed from this end."
            }
        }
    }

    fn placeholder(self) -> &'static str {
        match self {
            Self::IqStream => "host, or host:port (1234)",
        }
    }
}

#[derive(Clone)]
pub struct RemoteEdit {
    kind: RemoteKind,
    host: String,
    /// What to call it in the radio list. Optional, and worth having: an
    /// address says which machine and nothing about which aerial.
    label: String,
    /// Why the last attempt was refused, kept beside the field it belongs to
    /// rather than in the status line under the dial.
    err: Option<String>,
}

impl Default for RemoteEdit {
    fn default() -> Self {
        Self {
            kind: RemoteKind::IqStream,
            host: String::new(),
            label: String::new(),
            err: None,
        }
    }
}

/// A decode, as shown in the packet log.
pub struct Logged {
    /// Position in the capture, counted from the first packet and never
    /// reused, so a row keeps its number as the list scrolls.
    id: u64,
    rec: DecodeRecord,
}

pub struct Channel {
    /// Stable for the life of the channel, so the radio thread can keep its
    /// chain when a different channel is removed.
    id: u64,
    freq: f64,
    demod: Demod,
    label: String,
    /// Whether this channel is being demodulated into the mix.
    on: bool,
    /// Its own level in the mix, before the master volume.
    volume: f32,
    muted: bool,
    /// Where the squelch opens. None means the mode's own default, which is
    /// what an operator who has never touched the control should get.
    squelch_db: Option<f32>,
    agc: bool,
}

const FFTS: [usize; 6] = [512, 1024, 2048, 4096, 8192, 16384];
/// Spectrum refresh rates in frames per second.
const REFRESH: [(&str, f32); 4] = [("10", 10.0), ("20", 20.0), ("30", 30.0), ("60", 60.0)];
/// Waterfall scroll rates in rows per second.
const SPEEDS: [(&str, f32); 5] = [
    ("5", 5.0),
    ("10", 10.0),
    ("20", 20.0),
    ("40", 40.0),
    ("80", 80.0),
];

/// What the main pane shows.
#[derive(Clone, Copy, PartialEq, Eq)]
enum View {
    Spectrum,
    Chain,
    Map,
    Calls,
}

impl View {
    fn label(self) -> &'static str {
        match self {
            View::Spectrum => "Spectrum",
            View::Chain => "Signal chain",
            View::Map => "Map",
            View::Calls => "Calls",
        }
    }
}

/// Packets kept in the log. About a screenful of scrollback at any plausible
/// reading speed, and bounded memory on a band that never goes quiet.
const DECODE_LOG_MAX: usize = 500;
/// How many of the newest rows keep their burst's samples for the view.
const IQ_KEEP: usize = 64;
/// Height of the burst view in the packet detail, in pixels.
const BURST_VIEW_H: f32 = 120.0;
/// The least the inspector may be dragged to, and the drag handle's height.
const INSPECTOR_MIN_H: f32 = 64.0;

/// Tallest the inspector may be drawn, leaving the list a usable strip and the
/// column layout its two gaps of spacing.
fn inspector_max(avail: f32, gap: f32) -> f32 {
    (avail - 40.0 - gap * 2.0).max(INSPECTOR_MIN_H)
}

const HANDLE_H: f32 = 7.0;

/// Where the flight map opens. Zoom 8 is roughly a 150 nm view on a laptop
/// screen, which is about what a rooftop antenna hears.
const DEFAULT_MAP_ZOOM: f64 = 8.0;

/// Bytes, in whatever unit keeps the number readable.
fn human_bytes(n: u64) -> String {
    const UNITS: [(&str, u64); 4] =
        [("GB", 1 << 30), ("MB", 1 << 20), ("kB", 1 << 10), ("B", 1)];
    for (name, size) in UNITS {
        if n >= size {
            return format!("{:.1} {name}", n as f64 / size as f64);
        }
    }
    "0 B".into()
}

/// `host` or `host:port`, with the format's usual port when none is given.
fn parse_feed(text: &str, kind: &'static nodes::FeedKind) -> Option<nodes::FeedSpec> {
    let text = text.trim();
    if text.is_empty() {
        return None;
    }
    let (host, port) = match text.rsplit_once(':') {
        Some((h, p)) => (h, p.parse().ok()?),
        None => (text, kind.default_port),
    };
    let host = host.trim();
    if host.is_empty() {
        return None;
    }
    Some(nodes::FeedSpec::new(host, port, kind))
}

/// The average of the positions known, for opening the map somewhere useful
/// when the receiver has not been told where it is.
fn mean_position(active: &[&crate::tracks::Track]) -> Option<(f64, f64)> {
    let fixes: Vec<(f64, f64)> = active.iter().filter_map(|a| a.position).collect();
    if fixes.is_empty() {
        return None;
    }
    let n = fixes.len() as f64;
    Some((
        fixes.iter().map(|f| f.0).sum::<f64>() / n,
        fixes.iter().map(|f| f.1).sum::<f64>() / n,
    ))
}

/// Where the map is looking. `center` is `None` until something has been
/// heard, so the first track decides where the map opens rather than the map
/// opening on the ocean.
#[derive(Clone, Copy)]
struct MapView {
    center: Option<(f64, f64)>,
    /// Continuous, not a tile level: the tile level is where the pictures
    /// come from, and rounding the view to it would make most scroll notches
    /// do nothing.
    zoom: f64,
}

impl Default for MapView {
    fn default() -> Self {
        Self { center: None, zoom: DEFAULT_MAP_ZOOM }
    }
}

/// Colour of a packet whose integrity check passed.
const CRC_OK: Color32 = Color32::from_rgb(0x6F, 0xD1, 0x8A);

/// How wide a fader is drawn. The channel panel is a fixed width and every
/// one of these rows ends in a mute button, which needs the rest of it.
const VU_W: f32 = 130.0;

/// How far the auto scale keeps its ceiling above the loudest bin, and the
/// least range it will show whatever the band is doing.
///
/// A quiet band with nothing in it used to be scaled to a twenty decibel
/// window, which turns the noise floor's own wobble into a trace filling half
/// the plot and leaves a signal arriving on top of it nowhere to go. Thirty
/// five keeps the floor down where it belongs and leaves room above it for
/// something to appear in.
const PEAK_HEADROOM_DB: f32 = 8.0;
const MIN_SPAN_DB: f32 = 35.0;

/// Share of the scope pane the spectrum gets by default.
const DEFAULT_PLOT_FRAC: f32 = 0.34;
/// Range the split can be dragged to. Neither pane may be squeezed to nothing:
/// a two pixel waterfall is not a smaller waterfall, it is a broken one.
const PLOT_FRAC_RANGE: std::ops::RangeInclusive<f32> = 0.12..=0.85;
/// Height of the drag handle between the two, in pixels.
const SPLIT_GRIP_H: f32 = 7.0;

/// How near the pointer must be to a channel marker to grab it, in pixels.
///
/// Must exceed egui's drag threshold, or the pointer leaves the marker before
/// the drag is reported and the grab is never seen.
const GRAB_PX: f64 = 10.0;

/// Rate limits for a device, used to build the span list before it is opened.
/// The driver reports the same numbers through `DeviceInfo` once it is.
fn device_rates(e: &crate::devices::Entry) -> std::ops::RangeInclusive<Sps> {
    e.rates.clone()
}

impl Default for App {
    fn default() -> Self {
        Self {
            scope: state::ScopeState::default(),
            chain: state::ChainState::default(),
            log: state::LogState::default(),
            map: state::MapState::default(),
            calls: state::CallsState::default(),
            audio: state::AudioState::default(),
            record_dir: None,
            radio: None,
            err: None,
            center: crate::session::DEFAULT_CENTER,
            rate: 2_304_000.0,
            dial: Dial::new(),
            open: None,
            devices: Vec::new(),
            device: None,
            spans: Vec::new(),
            zoom: 1,
            soak: None,
            shot: None,
            shot_after: 6.0,
            decode_on: true,
            capture: false,
            shot_at: None,
            shot_sent: false,
            dc_block: true,
            view: View::Spectrum,
            location: None,
            country: String::new(),
            feeds: Vec::new(),
            feed_host: String::new(),
            remote: None,
            feed_kind: nodes::FEED_KINDS[0],
            scanner_edit: None,
            scanners: crate::scanners::Scanners::default(),
            log_dir_edit: String::new(),
            log_dir: None,
            log_cap_mb: Some(crate::packetlog::DEFAULT_MAX_BYTES >> 20),
            station_edit: None,
            saved: crate::session::Session::default(),
            saved_at: None,
            pending_radio: None,
        }
    }
}

impl App {
    pub fn new(cc: &eframe::CreationContext<'_>) -> Self {
        theme::install(&cc.egui_ctx);
        crate::shutdown::install(cc.egui_ctx.clone());
        let mut s = crate::session::Session::load();
        apply_locale(&mut s);
        // A radio on the network cannot be found by looking at the bus, so the
        // saved servers have to be registered before the list is built. Added
        // rather than set: the command line may already have put one there.
        for (addr, name) in &s.streams {
            crate::devices::add_stream(addr, name);
        }
        let devices = crate::devices::list();
        // The saved radio may not be plugged in any more, in which case the
        // rest of the session still applies to whatever is.
        let device = s
            .device
            .as_deref()
            .and_then(|want| devices.iter().find(|d| d.label == want).cloned())
            .or_else(|| devices.first().cloned());
        let mut app = Self {
            devices,
            device,
            center: s.center,
            // The file holds the device's own rate; the app works in the
            // effective one, which zoom divides.
            rate: s.rate / s.zoom.max(1) as f64,
            zoom: s.zoom,
            dc_block: s.dc_block,
            decode_on: s.decode_on,
            location: s.location,
            country: s.country.clone(),
            feeds: s.feeds.clone(),
            scanners: crate::scanners::Scanners::load(),
            saved: s.clone(),
            ..Default::default()
        };
        app.scope.restore(&s.view, s.fft);
        app.scope.db_center = s.center;
        app.scope.wf_center = s.center;
        app.audio.volume = s.volume;
        app.log.path = crate::packetlog::PacketLog::default_dir();
        app.pending_radio = Some(s);
        // The graph as it was left, if it was ever drawn by hand. Loaded
        // whether or not manual mode is on, so that turning it on comes back
        // to the drawing rather than to the automatic chain.
        if let Some((patch, places)) = crate::patch::Patch::load() {
            app.chain.drawn = Some(patch);
            app.chain.edit.pos =
                places.iter().map(|(k, (x, y))| (*k, egui::Pos2::new(*x, *y))).collect();
            app.chain.places = places;
        }
        // The settings show where the log is going, so they start from where
        // it is actually going.
        app.log_dir = app.log.path.clone();
        app.log_dir_edit =
            app.log_dir.as_ref().map(|d| d.display().to_string()).unwrap_or_default();
        app.connect(&cc.egui_ctx);
        app
    }

    /// The live settings, in the form they are stored in.
    fn session(&self) -> crate::session::Session {
        let radio = self.radio.as_ref().map(|r| r.status.radio());
        crate::session::Session {
            device: self.device.as_ref().map(|d| d.label.clone()),
            center: self.center,
            rate: self.rate * self.zoom.max(1) as f64,
            zoom: self.zoom,
            fft: self.scope.fft,
            // Read back from the driver rather than from what was asked for,
            // so the file holds the gain the hardware actually took.
            gains: radio
                .as_ref()
                .map(|r| r.stages.iter().map(|(s, m)| (s.name.clone(), *m)).collect())
                .unwrap_or_else(|| self.saved.gains.clone()),
            toggles: radio
                .as_ref()
                .map(|r| r.toggles.iter().map(|t| (t.name.clone(), t.on)).collect())
                .unwrap_or_else(|| self.saved.toggles.clone()),
            choices: radio
                .as_ref()
                .map(|r| r.choices.iter().map(|c| (c.name.clone(), c.selected.clone())).collect())
                .unwrap_or_else(|| self.saved.choices.clone()),
            ppm: radio.as_ref().map(|r| r.ppm).unwrap_or(self.saved.ppm),
            location: self.location,
            language: crate::i18n::language().code().to_string(),
            country: self.country.clone(),
            band_plan: crate::bands::plan().id().to_string(),
            view: self.scope.prefs(),
            feeds: self.feeds.clone(),
            streams: crate::devices::streams()
                .into_iter()
                .map(|r| (r.addr, r.label))
                .collect(),
            dc_block: self.dc_block,
            decode_on: self.decode_on,
            volume: self.audio.volume,
            manual_chain: self.chain.edit.manual,
        }
    }

    /// Write the session out when it has changed and settled.
    ///
    /// Debounced because dragging the dial changes the centre on every frame,
    /// and a file written sixty times a second to record a frequency nobody
    /// stopped on is a lot of writes for no information.
    fn save_session(&mut self) {
        let now = self.session();
        if now == self.saved {
            return;
        }
        let due = self.saved_at.is_none_or(|t| t.elapsed().as_secs_f32() >= 2.0);
        if !due {
            return;
        }
        now.save();
        self.saved = now;
        self.saved_at = Some(std::time::Instant::now());
    }

    /// Push restored gain stages and switches at the radio once it is up.
    ///
    /// Deferred rather than sent with the tuning: the stage names come from
    /// the driver, and until it has reported them there is nothing to check a
    /// saved name against.
    fn restore_radio_settings(&mut self) {
        let Some(want) = self.pending_radio.clone() else { return };
        let Some(radio) = self.radio.as_ref() else { return };
        let controls = radio.status.radio();
        if controls.stages.is_empty() && controls.toggles.is_empty() && controls.choices.is_empty()
        {
            return;
        }
        self.pending_radio = None;
        for (name, mode) in &want.gains {
            if controls.stages.iter().any(|(s, _)| &s.name == name) {
                self.send(Cmd::GainStage(name.clone(), *mode));
            }
        }
        for (name, on) in &want.toggles {
            if controls.toggles.iter().any(|t| &t.name == name) {
                self.send(Cmd::Toggle(name.clone(), *on));
            }
        }
        for (name, value) in &want.choices {
            // Saved by name rather than by position, so a driver that gains an
            // option does not silently move every setting along one.
            if controls.choices.iter().any(|c| &c.name == name && c.options.contains(value)) {
                self.send(Cmd::Choice(name.clone(), value.clone()));
            }
        }
        if want.ppm != 0.0 {
            self.send(Cmd::Ppm(want.ppm));
        }
        if !want.dc_block {
            self.send(Cmd::DcBlock(false));
        }
        if !want.decode_on {
            self.send(Cmd::Decode(false));
        }
        // A graph that was drawn by hand is the receiver's shape, so it has
        // to go back before anything else settles: the alternative is a
        // receiver that runs the automatic chain for a moment and then
        // rebuilds into the one that was saved.
        if want.manual_chain && self.chain.drawn.is_some() {
            self.set_manual_chain(true);
        }
    }

    /// Tell the tracker where the receiver is, so a single position frame
    /// resolves instead of waiting for a matching pair.
    pub fn set_location(&mut self, lat: f64, lon: f64) {
        self.location = Some((lat, lon));
        self.send(Cmd::Location(lat, lon));
    }

    /// Turn the packet log off, or point it somewhere other than the default.
    pub fn set_packet_log(&mut self, off: bool, dir: Option<std::path::PathBuf>) {
        self.log.path = if off { None } else { dir.or_else(crate::packetlog::PacketLog::default_dir) };
        let dir = self.log.path.clone();
        self.send(Cmd::PacketLog(dir));
    }

    /// Record every burst that decodes into a directory of captures.
    ///
    /// Held rather than sent once: choosing a device, or changing the span,
    /// starts a new radio thread, and recording that quietly stopped when the
    /// UI reconnected would be worse than not recording at all.
    pub fn record_to(&mut self, dir: std::path::PathBuf, budget_mb: Option<u64>) {
        self.record_dir = Some((dir.clone(), budget_mb));
        self.send(Cmd::Record(Some((dir, budget_mb))));
    }

    /// Print every packet to standard output as it is logged.
    pub fn set_print_log(&mut self, on: bool) {
        self.log.print = on;
    }

    /// Start or stop writing the raw span to a file.
    pub fn set_capture(&mut self, on: bool) {
        self.capture = on;
        self.send(Cmd::CaptureIq(on));
    }

    /// Pick the span closest to `hz`, narrowing in software if the radio
    /// cannot sample that slowly.
    pub fn set_span(&mut self, hz: f64) {
        let Some(sp) = self
            .spans
            .iter()
            .min_by(|a, b| (a.effective() - hz).abs().total_cmp(&(b.effective() - hz).abs()))
            .cloned()
        else {
            return;
        };
        self.send(Cmd::Rate(common::Sps(sp.rate as u64)));
        self.send(Cmd::Zoom(sp.zoom));
        self.rate = sp.effective();
        self.zoom = sp.zoom;
        self.reset_waterfall();
        self.retune_listener();
    }

    /// Start tuned to a station and listening to it.
    ///
    /// Useful for screenshots and for checking a change against real RF
    /// without a dozen clicks first.
    pub fn tune_to(&mut self, mhz: f64, demod: Demod) {
        let freq = mhz * 1e6;
        // The first channel sets where the receiver points; later ones are
        // added around it, since a second call moving the centre would drag
        // the first channel to the edge of the span or out of it.
        if self.audio.channels.is_empty() {
            self.center = freq;
            self.send(Cmd::Center(common::Hz(freq as u64)));
        }
        self.audio.channels.push(Channel {
            id: self.audio.next_id as u64,
            freq,
            demod,
            label: format!("{mhz:.1}"),
            on: true,
            volume: 0.8,
            muted: false,
            squelch_db: None,
            agc: true,
        });
        self.audio.next_id += 1;
        self.listen(self.audio.channels.len() - 1);
    }

    fn connect(&mut self, ctx: &egui::Context) {
        // Dropping the old Radio stops its thread and releases the USB claim
        // before the next one tries to take it.
        self.radio = None;
        self.err = None;
        let Some(entry) = self.device.clone() else {
            self.err = Some("no radio found. plug one in, then press RESCAN.".into());
            return;
        };
        self.spans = crate::devices::spans_with_zoom(&device_rates(&entry));
        if !self.spans.iter().any(|s| (s.effective() - self.rate).abs() < 1.0) {
            self.rate = self.spans.last().map(|s| s.effective()).unwrap_or(self.rate);
            self.zoom = 1;
        }
        let c = ctx.clone();
        self.radio = Some(Radio::start(
            entry,
            Hz(self.center as u64),
            // The radio samples at the full rate and the zoom narrows it in
            // software, so it is started at the rate before that division.
            Sps((self.rate * self.zoom.max(1) as f64).round() as u64),
            self.scope.fft,
            move || c.request_repaint(),
        ));
        if self.zoom > 1 {
            self.send(Cmd::Zoom(self.zoom));
        }
        if let Some(r) = self.record_dir.clone() {
            self.send(Cmd::Record(Some(r)));
        }
        if self.capture {
            self.send(Cmd::CaptureIq(true));
        }
        // The log is a node in the graph, so a new radio thread means a new
        // graph and it has to be told where to write again.
        if let Some(d) = self.log.path.clone() {
            self.send(Cmd::PacketLog(Some(d)));
        }
        // Same for the feeds and the station position: they belong to the
        // graph, and a new radio thread has built a new one.
        if !self.feeds.is_empty() {
            self.send(Cmd::Feeds(self.feeds.clone()));
        }
        if let Some((lat, lon)) = self.location {
            self.send(Cmd::Location(lat, lon));
        }
        // The spectrum's frame rate and averaging live in the graph, so a new
        // radio thread has them at their defaults until it is told otherwise.
        self.send(Cmd::Refresh(self.scope.refresh));
        self.send(Cmd::Smoothing(self.scope.smoothing));
        // Same for the call bus: a new thread has an empty one.
        self.send(Cmd::CallVolume { volume: self.audio.call_volume, muted: self.audio.call_muted });
        self.send(Cmd::CallAgc(self.audio.call_agc));
        if !self.calls.subs.is_empty() {
            self.send(Cmd::CallSubs(self.calls.subs.clone()));
        }
        // Whatever the radio was set to last time has to be pushed at it
        // again: a new thread means a freshly opened device at its defaults.
        self.pending_radio = Some(self.saved.clone());
        self.reset_waterfall();
    }

    /// Release the radio without quitting.
    ///
    /// Dropping it stops the thread and gives up the USB claim; a stale
    /// process holding that claim is why a second program fails to open the
    /// device at all.
    fn stop(&mut self) {
        self.radio = None;
        self.audio.listening = None;
        self.err = None;
    }

    fn select_device(&mut self, ctx: &egui::Context, e: crate::devices::Entry) {
        if self.device.as_ref() == Some(&e) {
            return;
        }
        // A remote tuner is pinned to one frequency, so the dial goes there
        // rather than the samples arriving under whatever it was last on.
        if let Some(f) = e.pinned {
            self.center = f.as_f64();
            self.scope.wf_center = self.center;
            self.scope.db_center = self.center;
        }
        self.device = Some(e);
        self.audio.listening = None;
        self.connect(ctx);
    }

    fn send(&self, c: Cmd) {
        if let Some(r) = &self.radio {
            r.send(c);
        }
    }

    fn drain(&mut self) {
        let Some(radio) = &self.radio else { return };
        // The flight tracker lives in the graph; this is the table it
        // published on the last frame.
        if self.view == View::Map || !self.map.tracks.is_empty() {
            self.map.tracks = radio.status.track_list.lock().clone();
        }
        if let Some(e) = radio.status.error.lock().take() {
            self.err = Some(e);
        }
        let mut frames: Vec<Frame> = Vec::new();
        while let Ok(f) = radio.frames.try_recv() {
            frames.push(f);
        }
        // A pinned radio cannot be retuned, and a dial left wherever it was
        // dragged would label every frequency on screen wrongly.
        if let Some(f) = self.device.as_ref().and_then(|d| d.pinned) {
            self.center = f.as_f64();
        }
        // Every frame is peak-held into the pending row, not just the one that
        // happens to be last in the queue. Folding only the last of each batch
        // tied a waterfall row's content to how often the interface repainted:
        // dragging a slider repaints continuously, fewer frames were thrown
        // away between drains, and the history visibly changed contrast for as
        // long as the drag lasted.
        self.chain.topo = radio.status.chain();
        self.chain.latency = radio.status.chain_latency();
        // An edit that will not build is refused and the last one that did
        // goes back, so what is on screen has to be what the receiver is
        // running rather than what was last asked for.
        let (rev, running) = radio.status.patch();
        if rev != self.chain.patch_rev {
            self.chain.patch_rev = rev;
            let running = running.unwrap_or_default();
            // Only when it is not the edit that was just sent. Adopting our
            // own patch back would undo anything drawn in the meantime, since
            // the receiver is a rebuild behind the pointer.
            if self.chain.patch_sent.as_ref() != Some(&running) {
                self.chain.patch = running;
            }
        }
        let mut batches = Vec::new();
        while let Ok(batch) = radio.decodes.try_recv() {
            batches.push(batch);
        }
        for f in &frames {
            self.hold_peak(f);
        }
        let latest: Option<Frame> = frames.pop();
        for b in batches {
            self.log_decodes(b);
        }
        if let Some(f) = latest {
            // The requested centre is not overwritten by the frame's. Retunes
            // are spaced out because each blocks the radio thread, so frames
            // arrive from the old frequency for a while after a drag moves the
            // view. Adopting their centre would drag the view back under the
            // pointer every time one landed.
            self.scope.db_center = f.center;
            self.rate = f.rate;
            if self.scope.auto_scale {
                self.rescale(&f.db);
            }
            self.slide_waterfall(f.center, f.db.len());

            let due = self.scope
                .wf_last
                .map(|t| t.elapsed().as_secs_f32() >= 1.0 / self.scope.rows_per_sec)
                .unwrap_or(true);
            if due {
                // The waterfall tops out below the trace's ceiling: the plot
                // wants headroom so peaks are not clipped flat, the colour
                // ramp wants the opposite or its hottest colours go unused.
                let pending = std::mem::take(&mut self.scope.wf_pending);
                    self.scope.wf.push(&pending, self.scope.floor, self.scope.ceil - self.scope.wf_top_offset);
                self.scope.wf_pending = pending;
                self.scope.wf_pending.fill(f32::MIN);
                self.scope.wf_last = Some(std::time::Instant::now());
            }
            self.scope.db = f.db;
            self.scope.extra = f.extra;
        }
    }

    /// Add decoded packets to the on-screen list, oldest first.
    ///
    /// Nothing is written here. The packet log is a node in the graph and
    /// stores what the demodulators produced, which is a better record than
    /// this list: these are conclusions, and they are bounded.
    fn log_decodes(&mut self, batch: Vec<DecodeRecord>) {
        for rec in batch {
            if self.log.print {
                println!("{}", rec.line(self.log.print_since));
            }
            // A transmission that names who it is for is also a call, and
            // the call list outlives the packet log: a group heard an hour
            // ago has scrolled out of the log long before it is forgotten
            // here.
            self.calls.list.update(&rec, rec.at);
            let id = self.log.next_packet;
            self.log.next_packet += 1;
            self.log.decodes.push(Logged { id, rec });
        }
        // Listening does not depend on which pane is on screen. The
        // subscriptions used to be made where the call list is drawn, so a
        // receiver sitting on the spectrum heard nothing however much it
        // decoded, and the fault looked like a broken vocoder.
        let heard: Vec<crate::calls::Call> =
            self.calls.list.active(std::time::Instant::now()).into_iter().cloned().collect();
        self.subscribe_new_groups(&heard);

        // A busy band produces packets faster than anyone reads them, and an
        // unbounded log is a slow memory leak with a scrollbar.
        // The samples of a burst are kept for the newest rows only. A row
        // is a few hundred bytes; its burst is a few hundred kilobytes, and
        // five hundred of those is the receiver's memory spent on a scroll
        // nobody reads that far back.
        let keep_from = self.log.decodes.len().saturating_sub(IQ_KEEP);
        for l in &mut self.log.decodes[..keep_from] {
            l.rec.iq = None;
        }
        if self.log.decodes.len() > DECODE_LOG_MAX {
            let drop = self.log.decodes.len() - DECODE_LOG_MAX;
            self.log.decodes.drain(..drop);
            // A selection that has aged out of the list must not leave the
            // dump showing bytes with no row above them.
            if self.log.selected.is_some_and(|id| !self.log.decodes.iter().any(|l| l.id == id)) {
                self.log.selected = None;
            }
        }
    }

    /// Slide the waterfall to match a new centre frequency.
    /// Fold one spectrum frame into the row being built.
    ///
    /// Peak-held rather than averaged or sampled: a burst shorter than the row
    /// interval lands between rows otherwise and is never drawn at all, which
    /// on a band of short transmissions is most of them.
    fn hold_peak(&mut self, f: &Frame) {
        // A frame from a different place is not the same row. Peak-holding
        // across a retune would smear the old span's carriers onto the new
        // one's frequencies.
        if self.scope.wf_pending.len() != f.db.len() || self.scope.wf_pending_center != f.center {
            self.scope.wf_pending = f.db.clone();
            self.scope.wf_pending_center = f.center;
            return;
        }
        for (a, b) in self.scope.wf_pending.iter_mut().zip(&f.db) {
            *a = a.max(*b);
        }
    }

    fn slide_waterfall(&mut self, center: f64, bins: usize) {
        if bins == 0 {
            return;
        }
        // Aligned to where the rows' data actually is, not to where the view
        // has been moved to, or the history smears as the two drift apart.
        let hz_per_bin = self.rate / bins as f64;
        let d = ((center - self.scope.wf_center) / hz_per_bin).round();
        if d != 0.0 {
            self.scope.wf.shift(d as i32);
            self.scope.wf_center += d * hz_per_bin;
        }
    }

    /// Track percentiles, not extremes: one strong carrier would otherwise
    /// flatten everything else in the span.
    fn rescale(&mut self, db: &[f32]) {
        let mut v: Vec<f32> = db.iter().copied().filter(|x| x.is_finite()).collect();
        if v.is_empty() {
            return;
        }
        v.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let pct = |p: f32| v[((v.len() - 1) as f32 * p) as usize];
        let (lo, hi) = (pct(0.10) - 6.0, pct(0.999) + PEAK_HEADROOM_DB);
        self.scope.floor += (lo - self.scope.floor) * 0.05;
        self.scope.ceil += (hi.max(lo + MIN_SPAN_DB) - self.scope.ceil) * 0.05;
    }

    /// Draw the scope, then do what it asked for.
    ///
    /// The pane cannot reach the radio, so a click that tunes or a marker
    /// that was dragged comes back as an action and is carried out here,
    /// which is the only place that knows how to send anything.
    fn scope_view(&mut self, ui: &mut egui::Ui) {
        let acts = scope::Scope {
            st: &mut self.scope,
            channels: &mut self.audio.channels,
            listening: self.audio.listening,
            center: self.center,
            rate: self.rate,
            radio: self.radio.as_ref(),
            scanners: &self.scanners,
            patch: &self.chain.patch,
            decode_on: self.decode_on,
            acts: Vec::new(),
        }
        .show(ui);
        for a in acts {
            match a {
                scope::Action::Listen(i) => self.listen(i),
                scope::Action::Add(hz) => self.add_channel(hz),
                scope::Action::Retune(hz) => self.retune(hz),
                scope::Action::Moved(i) => {
                    if self.audio.listening == Some(i) {
                        self.listen(i);
                    }
                }
                scope::Action::Open(w) => self.open = Some(w),
            }
        }
    }

    fn retune(&mut self, hz: f64) {
        self.center = hz.clamp(24e6, 1766e6);
        self.send(Cmd::Center(Hz(self.center as u64)));
        self.retune_listener();
    }

    /// The span or bin count changed, so old rows no longer line up.
    fn reset_waterfall(&mut self) {
        self.scope.wf.clear();
        self.scope.wf_center = self.center;
        self.scope.wf_pending.clear();
    }

    fn add_channel(&mut self, freq: f64) {
        let id = self.audio.next_id;
        self.audio.next_id += 1;
        self.audio.channels.push(Channel {
            id: id as u64,
            freq,
            demod: bands::demod_at(freq),
            label: format!("CH{id}"),
            on: true,
            volume: 0.8,
            muted: false,
            squelch_db: None,
            agc: true,
        });
        self.audio.listening = Some(self.audio.channels.len() - 1);
        self.send_channels();
    }

    /// Hand the radio the whole channel list.
    ///
    /// The whole list rather than an edit, because the radio thread is the
    /// one that knows which chains it already has: sending it the state it
    /// should be in leaves no way for the two to disagree, and it keeps the
    /// chains of channels that did not change.
    fn send_channels(&mut self) {
        let specs = self.channel_specs();
        self.send(Cmd::Channels(specs));
    }

    fn channel_specs(&self) -> Vec<ChannelSpec> {
        let center = self.center;
        self.audio.channels
            .iter()
            .filter(|c| c.on)
            .map(|c| ChannelSpec {
                id: c.id,
                offset_hz: c.freq - center,
                demod: c.demod,
                volume: c.volume,
                muted: c.muted,
                squelch_db: c.squelch_db,
                agc: c.agc,
            })
            .collect()
    }

    fn listen(&mut self, idx: usize) {
        if let Some(ch) = self.audio.channels.get_mut(idx) {
            ch.on = true;
        }
        self.audio.listening = Some(idx);
        self.send_channels();
    }

    fn retune_listener(&mut self) {
        if self.audio.listening.is_some_and(|i| i >= self.audio.channels.len()) {
            self.audio.listening = None;
        }
        self.send_channels();
    }
}

/// One scanner as the interface edits it.
///
/// Frequencies are held in the units they are typed in, and the lists stay as
/// text while they are being typed: a half-finished "161.9" must not be
/// parsed into a channel and used to decide what runs. Converting happens at
/// [`ScannerRow::to_scanner`], and a row that does not convert is shown as
/// incomplete rather than silently dropped.
pub struct ScannerRow {
    name: String,
    lo_mhz: f64,
    hi_mhz: f64,
    span_khz: f64,
    margin_khz: f64,
    front: crate::scanners::Front,
    channels: String,
    widths: String,
}

impl ScannerRow {
    fn from_scanner(s: &crate::scanners::Scanner) -> Self {
        let widths = match &s.front {
            crate::scanners::Front::Banks(w) => {
                w.iter().map(|x| trim_num(x / 1e3)).collect::<Vec<_>>().join(", ")
            }
            _ => String::new(),
        };
        Self {
            name: s.name.clone(),
            lo_mhz: s.lo / 1e6,
            hi_mhz: s.hi / 1e6,
            span_khz: s.min_rate / 1e3,
            margin_khz: s.margin_hz / 1e3,
            front: s.front.clone(),
            channels: s.channels.iter().map(|c| trim_num(c / 1e6)).collect::<Vec<_>>().join(", "),
            widths,
        }
    }

    /// A new block around the frequency being looked at, which is why
    /// somebody is adding one.
    fn new_at(center: f64, rate: f64) -> Self {
        let mhz = center / 1e6;
        let half = (rate / 2e6).max(0.01);
        Self {
            name: "New scanner".into(),
            lo_mhz: (mhz - half).max(0.0),
            hi_mhz: mhz + half,
            span_khz: (rate / 1e3).max(1.0),
            margin_khz: 0.0,
            front: crate::scanners::Front::Banks(crate::scanners::DEFAULT_WIDTHS.to_vec()),
            channels: String::new(),
            widths: crate::scanners::DEFAULT_WIDTHS
                .iter()
                .map(|x| trim_num(x / 1e3))
                .collect::<Vec<_>>()
                .join(", "),
        }
    }

    /// The banks front end carrying whatever widths are currently typed, so
    /// switching away and back does not lose them.
    fn banks_with_current_widths(&self) -> crate::scanners::Front {
        let w: Vec<f64> = parse_list(&self.widths, 1e3);
        crate::scanners::Front::Banks(if w.is_empty() {
            crate::scanners::DEFAULT_WIDTHS.to_vec()
        } else {
            w
        })
    }

    fn to_scanner(&self) -> Option<crate::scanners::Scanner> {
        let name = self.name.trim();
        if name.is_empty() || self.hi_mhz <= self.lo_mhz {
            return None;
        }
        let front = match self.front {
            crate::scanners::Front::Banks(_) => self.banks_with_current_widths(),
            ref f => f.clone(),
        };
        Some(crate::scanners::Scanner {
            name: name.to_string(),
            lo: self.lo_mhz * 1e6,
            hi: self.hi_mhz * 1e6,
            min_rate: self.span_khz * 1e3,
            channels: parse_list(&self.channels, 1e6),
            margin_hz: self.margin_khz * 1e3,
            front,
        })
    }
}

/// A comma separated list of numbers, scaled to hertz.
fn parse_list(text: &str, unit: f64) -> Vec<f64> {
    text.split(',').filter_map(|p| p.trim().parse::<f64>().ok()).map(|v| v * unit).collect()
}

/// A number without trailing zeros, for putting one back in a text field.
fn trim_num(v: f64) -> String {
    let s = format!("{v:.4}");
    s.trim_end_matches('0').trim_end_matches('.').to_string()
}

/// A megahertz field, typed to enough places for a 25 kHz channel raster.
fn mhz_field(ui: &mut egui::Ui, v: &mut f64) {
    ui.add(egui::DragValue::new(v).speed(0.01).range(0.0..=6000.0).max_decimals(4));
}

/// A line of explanation under a control.
///
/// Added through `Label` with wrapping asked for explicitly: inside a modal
/// the surrounding layout justifies text, which spreads a wrapped sentence
/// across the full width and leaves holes in the middle of it.
/// Put the saved language, country and band plan into effect, filling in what
/// has never been chosen.
///
/// Done once at startup rather than read from the session on every lookup:
/// naming the band a frequency falls in happens from drawing code that has no
/// settings object to consult.
fn apply_locale(s: &mut crate::session::Session) {
    if let Some(l) = crate::i18n::Language::from_code(&s.language) {
        crate::i18n::set_language(l);
    }
    // A first run has nothing saved, and the environment already knows: a
    // locale of en_IE means the European plan, and guessing wrong puts an
    // American on a table where 915 MHz is a phone.
    if s.country.is_empty() {
        if let Some(c) = crate::locale::from_environment() {
            s.country = c.code.to_string();
            if s.band_plan.is_empty() {
                s.band_plan = c.plan.id().to_string();
            }
        }
    }
    if let Some(p) = crate::bands::Plan::from_id(&s.band_plan) {
        crate::bands::set_plan(p);
    }
}

fn hint(ui: &mut egui::Ui, text: &str) {
    ui.add(egui::Label::new(egui::RichText::new(text).small().color(theme::LEGEND)).wrap());
}

/// The heading every modal wears, so one dialog does not announce itself in a
/// different voice from the next.
fn modal_title(ui: &mut egui::Ui, text: &str) {
    ui.label(legend(text));
    ui.add_space(10.0);
}

/// A labelled settings row: legend on the left, control on the right, so the
/// modal reads as a column of settings rather than a wall of widgets.
fn row(ui: &mut egui::Ui, label: &str, add: impl FnOnce(&mut egui::Ui)) {
    ui.horizontal(|ui| {
        ui.add_sized([90.0, 18.0], egui::Label::new(legend(label)));
        add(ui);
    });
}

/// Resolution bandwidth, which is what the bin count actually buys you.
fn bin_hint(rate: f64, bins: usize) -> String {
    let hz = rate / bins as f64;
    if hz >= 1000.0 {
        format!("{:.1} kHz per bin", hz / 1e3)
    } else {
        format!("{hz:.0} Hz per bin")
    }
}

/// Where each line of the airport card sits, and how big the card has to be
/// to hold them all.
struct CardLayout {
    size: Vec2,
    /// Top of each line from the card's top edge: the head lines, then the
    /// rows below the rule, in the order they are drawn.
    ys: Vec<f32>,
    rule_y: f32,
    text_x: f32,
}

/// Lay the card out from the measured size of every line it will draw.
///
/// Pure arithmetic, and separate from the drawing, because the two ways this
/// went wrong were both a line drawn that the size had not accounted for: the
/// width left no room for the margin the text was drawn at, and the "+N more"
/// row was painted below a card measured without it. Measuring and drawing
/// from one list is what stops that, and it can be checked without a font.
fn card_layout(head: &[Vec2], rows: &[Vec2], pad: f32, sep: f32, rule_gap: f32) -> CardLayout {
    let text_w = head
        .iter()
        .chain(rows)
        .map(|s| s.x)
        .fold(0.0f32, f32::max);
    let mut ys = Vec::with_capacity(head.len() + rows.len());
    let mut y = pad;
    for (i, s) in head.iter().enumerate() {
        if i > 0 {
            y += sep;
        }
        ys.push(y);
        y += s.y;
    }
    y += rule_gap;
    let rule_y = y;
    y += 1.0 + rule_gap;
    for (i, s) in rows.iter().enumerate() {
        if i > 0 {
            y += sep;
        }
        ys.push(y);
        y += s.y;
    }
    CardLayout {
        size: Vec2::new(text_w + pad * 2.0, y + pad),
        ys,
        rule_y,
        text_x: pad,
    }
}

/// Settings affordance in a pane corner.
fn cog_rect(pane: &Rect) -> Rect {
    let s = 18.0;
    Rect::from_min_size(Pos2::new(pane.right() - s - 6.0, pane.top() + 6.0), Vec2::splat(s))
}

fn cog(p: &egui::Painter, r: &Rect, hot: bool) {
    let col = if hot { theme::READOUT } else { Color32::from_rgb(0x6A, 0x72, 0x7C) };
    crate::icons::Icon::Setup.paint(p, *r, col);
}

fn fmt_hz(hz: f64) -> String {
    if hz.abs() >= 1e6 {
        format!("{:.4} MHz", hz / 1e6)
    } else if hz.abs() >= 1e3 {
        format!("{:.1} kHz", hz / 1e3)
    } else {
        format!("{hz:.0} Hz")
    }
}

impl eframe::App for App {
    fn ui(&mut self, ui: &mut egui::Ui, _f: &mut eframe::Frame) {
        let _f = tracing::info_span!("frame").entered();
        {
            let _s = tracing::info_span!("drain").entered();
            self.drain();
        }
        self.screenshot(ui.ctx());
        self.soak_check(ui.ctx());
        if crate::shutdown::asked() {
            // Closing rather than exiting, so the session is saved and the
            // radio and the log are dropped the way a click on the close
            // button drops them.
            ui.ctx().send_viewport_cmd(egui::ViewportCommand::Close);
        }
        {
            let _s = tracing::info_span!("head").entered();
            self.head(ui);
        }
        {
            let _s = tracing::info_span!("strip").entered();
            self.strip(ui);
        }
        {
            let _s = tracing::info_span!("log").entered();
            self.decode_log(ui);
        }
        {
            let _s = tracing::info_span!("scope").entered();
            CentralPanel::default()
                .frame(egui::Frame::NONE.fill(theme::CHASSIS))
                .show(ui, |ui| match self.view {
                    View::Spectrum => self.scope_view(ui),
                    View::Chain => self.chain(ui),
                    View::Map => self.map_view(ui),
                    View::Calls => self.call_view(ui),
                });
        }
        self.settings_modal(ui.ctx());
        self.remote_modal(ui.ctx());
        self.restore_radio_settings();
        self.save_session();
    }

    fn on_exit(&mut self) {
        // The periodic save is debounced, so a change made in the last couple
        // of seconds before quitting is still only in memory.
        self.saved_at = None;
        self.save_session();
    }
}

impl App {
    /// Self-measured CPU, because a GUI process is awkward to sample from a
    /// shell and the number that matters is over a steady-state window.
    fn soak_check(&mut self, ctx: &egui::Context) {
        let Some(secs) = self.soak else { return };
        if self.shot_sent {
            return;
        }
        // Deliberately does not request repaints: the point is to measure how
        // often the app redraws on its own.
        let t0 = *self.shot_at.get_or_insert_with(std::time::Instant::now);
        let el = t0.elapsed().as_secs_f32();
        if el < secs {
            return;
        }
        let cpu = std::fs::read_to_string("/proc/self/stat")
            .ok()
            .and_then(|s| {
                // Fields are offset by the comm field, which can contain
                // spaces and parentheses, so start after the last ')'.
                let tail = &s[s.rfind(')')? + 1..];
                let f: Vec<&str> = tail.split_whitespace().collect();
                let u: f64 = f.get(11)?.parse().ok()?;
                let k: f64 = f.get(12)?.parse().ok()?;
                Some((u + k) / 100.0)
            })
            .unwrap_or(0.0);
        println!(
            "ran {el:.1}s, used {cpu:.2}s CPU = {:.0}% of one core",
            cpu / el as f64 * 100.0
        );
        crate::prof::report(std::time::Duration::from_secs_f32(el));
        self.shot_sent = true;
        ctx.send_viewport_cmd(egui::ViewportCommand::Close);
    }

    fn screenshot(&mut self, ctx: &egui::Context) {
        let Some(path) = self.shot.clone() else { return };
        ctx.request_repaint();
        let t0 = *self.shot_at.get_or_insert_with(std::time::Instant::now);
        // Wait for the tuner to lock and the waterfall to fill; a screenshot
        // taken before that reviews an empty screen, not the design.
        if !self.shot_sent && t0.elapsed().as_secs_f32() > self.shot_after {
            if self.audio.channels.is_empty() {
                self.add_channel(95.8e6);
                self.add_channel(95.35e6);
            }
            self.shot_sent = true;
            ctx.send_viewport_cmd(egui::ViewportCommand::Screenshot(Default::default()));
        }
        let img = ctx.input(|i| {
            i.events.iter().find_map(|e| match e {
                egui::Event::Screenshot { image, .. } => Some(image.clone()),
                _ => None,
            })
        });
        if let Some(img) = img {
            let (w, h) = (img.width() as u32, img.height() as u32);
            let buf: Vec<u8> = img.pixels.iter().flat_map(|p| [p.r(), p.g(), p.b(), p.a()]).collect();
            if let Some(b) = image::RgbaImage::from_raw(w, h, buf) {
                let _ = b.save(&path);
                println!("wrote {path} ({w}x{h})");
            }
            ctx.send_viewport_cmd(egui::ViewportCommand::Close);
        }
    }

    /// The readout and the controls that set it.
    fn head(&mut self, ui: &mut egui::Ui) {
        Panel::top("head")
            .frame(
                egui::Frame::NONE
                    .fill(theme::PANEL)
                    .inner_margin(egui::Margin::symmetric(14, 10)),
            )
            .show(ui, |ui| {
                ui.horizontal(|ui| {
                    let out = self.dial.show(ui, self.center, 34.0);
                    if out.changed {
                        self.retune(out.hz);
                    }

                    ui.add_space(16.0);
                    ui.vertical(|ui| {
                        ui.add_space(2.0);
                        ui.label(legend("band"));
                        ui.label(
                            value(bands::name_at(self.center))
                                .color(theme::TRACE)
                                .size(14.0),
                        );
                    });

                    ui.add_space(18.0);
                    self.divider(ui);
                    ui.add_space(18.0);

                    ui.vertical(|ui| {
                        ui.horizontal(|ui| {
                            ui.label(legend("radio"));
                            // Beside the device it describes. Off in the
                            // corner it was a readout of something, and which
                            // something was anyone's guess.
                            self.status_lamp(ui);
                        });
                        let cur = self
                            .device
                            .as_ref()
                            .map(|d| d.label.clone())
                            .unwrap_or_else(|| "none".into());
                        let mut pick = None;
                        let mut rescan = false;
                        let mut forget = None;
                        let mut add_remote = false;
                        egui::ComboBox::from_id_salt("device")
                            .selected_text(cur)
                            .width(190.0)
                            .show_ui(ui, |ui| {
                                for d in &self.devices {
                                    let on = self.device.as_ref() == Some(d);
                                    // A remote radio was created here rather
                                    // than plugged in, so it is dropped here
                                    // too: nothing else in the interface knows
                                    // it exists.
                                    match &d.addr {
                                        Some(addr) => {
                                            ui.horizontal(|ui| {
                                                if ui.selectable_label(on, &d.label).clicked() {
                                                    pick = Some(d.clone());
                                                }
                                                ui.with_layout(
                                                    egui::Layout::right_to_left(
                                                        egui::Align::Center,
                                                    ),
                                                    |ui| {
                                                        if ui.small_button("×").clicked() {
                                                            forget = Some(addr.clone());
                                                        }
                                                    },
                                                );
                                            });
                                        }
                                        None => {
                                            if ui.selectable_label(on, &d.label).clicked() {
                                                pick = Some(d.clone());
                                            }
                                        }
                                    }
                                }
                                ui.separator();
                                if ui.selectable_label(false, "Rescan").clicked() {
                                    rescan = true;
                                }
                                if ui.selectable_label(false, "Add remote…").clicked() {
                                    add_remote = true;
                                }
                            });
                        if add_remote {
                            self.remote = Some(RemoteEdit::default());
                        }
                        if let Some(addr) = forget {
                            crate::devices::remove_stream(&addr);
                            let c = ui.ctx().clone();
                            self.rescan(&c);
                        }
                        if rescan {
                            let c = ui.ctx().clone();
                            self.rescan(&c);
                        }
                        if let Some(d) = pick {
                            let c = ui.ctx().clone();
                            self.select_device(&c, d);
                        }

                        ui.add_space(4.0);
                        ui.horizontal(|ui| {
                            // Stopping releases the USB claim, which is the
                            // only way to hand the radio to another program
                            // without quitting.
                            use crate::icons::{icon_button, Icon};
                            let on = self.radio.is_some();
                            let t = crate::i18n::t;
                            if icon_button(ui, Icon::Play, t("ui.start"), !on, false).clicked() {
                                let c = ui.ctx().clone();
                                self.connect(&c);
                            }
                            if icon_button(ui, Icon::Stop, t("ui.stop"), on, false).clicked() {
                                self.stop();
                            }
                            // Not gain alone: the pane behind it also holds
                            // the radio's switches, its antenna and channel
                            // choices, and the crystal correction.
                            if icon_button(ui, Icon::Sliders, t("ui.settings"), on, false)
                                .clicked()
                            {
                                self.open = Some(Settings::Radio);
                            }
                            // Beside the transport, because that is what it
                            // is: the span is running and this writes it
                            // down. A signal nothing decodes is worth
                            // capturing while it is still transmitting, and
                            // anything behind a modal is too slow for that.
                            let capturing = self
                                .radio
                                .as_ref()
                                .is_some_and(|r| r.status.capture_on.load(std::sync::atomic::Ordering::Relaxed));
                            let tip = if capturing { t("ui.capture_stop") } else { t("ui.capture") };
                            if icon_button(ui, Icon::Capture, tip, on, capturing).clicked() {
                                self.set_capture(!capturing);
                            }
                        });
                    });

                    ui.add_space(18.0);
                    self.divider(ui);
                    ui.add_space(18.0);

                    ui.vertical(|ui| {
                        ui.label(legend("bandwidth"));
                        let cur = self
                            .spans
                            .iter()
                            .find(|s| (self.rate - s.effective()).abs() < 1.0)
                            .map(|s| s.label.clone())
                            .unwrap_or_else(|| "custom".into());
                        let mut pick = None;
                        egui::ComboBox::from_id_salt("span")
                            .selected_text(cur)
                            .width(96.0)
                            .show_ui(ui, |ui| {
                                for sp in &self.spans {
                                    let on = (self.rate - sp.effective()).abs() < 1.0
                                        && self.zoom == sp.zoom;
                                    let text = if sp.zoom > 1 {
                                        format!("{}  /{}", sp.label, sp.zoom)
                                    } else {
                                        sp.label.clone()
                                    };
                                    if ui.selectable_label(on, text).clicked() && !on {
                                        pick = Some(sp.clone());
                                    }
                                }
                            });
                        if let Some(sp) = pick {
                            // Rate first: the radio rebuilds everything on a
                            // rate change, and a zoom sent before it would be
                            // applied to a chain about to be replaced.
                            self.send(Cmd::Rate(Sps(sp.rate as u64)));
                            self.send(Cmd::Zoom(sp.zoom));
                            self.rate = sp.effective();
                            self.zoom = sp.zoom;
                            self.reset_waterfall();
                            self.retune_listener();
                        }
                    });

                    ui.add_space(18.0);
                    self.divider(ui);
                    ui.add_space(18.0);

                    ui.vertical(|ui| {
                        ui.label(legend("view"));
                        let mut v = self.view;
                        egui::ComboBox::from_id_salt("view")
                            .selected_text(v.label())
                            .width(120.0)
                            .show_ui(ui, |ui| {
                                for opt in [View::Spectrum, View::Chain, View::Map, View::Calls] {
                                    ui.selectable_value(&mut v, opt, opt.label());
                                }
                            });
                        self.view = v;
                    });

                    ui.add_space(18.0);
                    self.divider(ui);
                    ui.add_space(18.0);

                    ui.vertical(|ui| {
                        ui.label(legend("packets"));
                        ui.horizontal(|ui| {
                            // Only the switch that opens the log. What decodes
                            // and what runs where are questions about the
                            // packets, so they are asked in the window that
                            // shows them rather than up here.
                            if crate::icons::icon_button(
                                ui,
                                crate::icons::Icon::Log,
                                crate::i18n::t("ui.log"),
                                true,
                                self.log.open,
                            )
                            .clicked()
                            {
                                self.log.open = !self.log.open;
                            }
                        });
                    });

                    // Pinned to the far end, and built like every other group
                    // on this row: a legend with its control under it. Floating
                    // loose against the right edge, it read as a lamp or a
                    // dismiss button, because nothing said what it belonged to.
                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Min), |ui| {
                        ui.vertical(|ui| {
                            ui.label(legend("setup"));
                            ui.horizontal(|ui| {
                                let open = crate::icons::icon_button(
                                    ui,
                                    crate::icons::Icon::Setup,
                                    crate::i18n::t("ui.setup"),
                                    true,
                                    self.open == Some(Settings::App),
                                );
                                if open.clicked() {
                                    self.open = Some(Settings::App);
                                }
                            });
                        });
                        ui.add_space(18.0);
                        self.divider(ui);
                        ui.add_space(18.0);
                    });
                });

            if let Some(e) = &self.err {
                    ui.add_space(4.0);
                    ui.label(
                        egui::RichText::new(e)
                            .color(theme::FAULT)
                            .font(FontId::proportional(12.0)),
                    );
                }
            });
    }

    fn divider(&self, ui: &mut egui::Ui) {
        let h = 40.0;
        let (rect, _) = ui.allocate_exact_size(Vec2::new(1.0, h), Sense::hover());
        ui.painter().line_segment(
            [
                Pos2::new(rect.center().x, rect.top()),
                Pos2::new(rect.center().x, rect.bottom()),
            ],
            Stroke::new(1.0, theme::ETCH),
        );
    }

    /// Status lamps. Dark is good; an unlit lamp means nothing is wrong.
    /// One lamp for the whole receive path.
    ///
    /// Two lamps and two numbers used to say this, in the far corner, and the
    /// numbers were the wrong thing to print: a sample count nobody can act on
    /// is noise, while its colour is the one thing worth seeing across a room.
    /// Green is receiving cleanly. Red is either stopped or dropping, which
    /// are the same news, and the hover text says which.
    fn status_lamp(&self, ui: &mut egui::Ui) {
        use std::sync::atomic::Ordering;
        let (running, dropped) = match &self.radio {
            Some(r) => (
                r.status.running.load(Ordering::Relaxed),
                r.status.dropped.load(Ordering::Relaxed),
            ),
            None => (false, 0),
        };
        let good = running && dropped == 0;
        let col = if good { theme::OK } else { theme::FAULT };

        let (rect, resp) = ui.allocate_exact_size(Vec2::splat(10.0), Sense::hover());
        let p = ui.painter();
        p.circle_filled(rect.center(), 3.5, col);
        // A halo, so it reads as a lit lamp rather than a printed dot.
        p.circle_filled(
            rect.center(),
            6.0,
            Color32::from_rgba_unmultiplied(col.r(), col.g(), col.b(), 34),
        );
        resp.on_hover_text(if !running {
            "Stopped. The device is free for another program.".to_string()
        } else if dropped == 0 {
            "Receiving, no samples dropped.".to_string()
        } else {
            format!(
                "Receiving, but {} samples were dropped: the host is not keeping up with this span.",
                thousands(dropped)
            )
        });
    }

    /// Gain and squelch, for the modes that have them.
    ///
    /// Worth a line of its own because on a weak signal these two are the
    /// difference between a band that is dead and a receiver that is muted,
    /// and without them both look and sound identical.
    fn channel_audio(ui: &mut egui::Ui, ch: &mut Channel, st: ChannelState) -> bool {
        let (gain_db, open, measured) = (st.agc_gain_db, st.squelch_open, st.squelch_db);
        let mut changed = false;
        ui.add_space(4.0);
        if ch.demod != Demod::Wfm {
            ui.horizontal(|ui| {
                ui.label(legend("agc"));
                if ui.selectable_label(ch.agc, if ch.agc { "ON" } else { "OFF" }).clicked() {
                    ch.agc = !ch.agc;
                    changed = true;
                }
                if ch.agc {
                    ui.label(value(format!("{gain_db:+.0} dB")).size(10.0));
                }
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    if !open {
                        ui.label(value("MUTED").size(10.0).color(theme::LEGEND));
                    }
                });
            });
        }
        if let Some(default) = ch.demod.default_squelch_db() {
            let (lo, hi, ratio) = ch.demod.squelch_range();
            let mut db = ch.squelch_db.unwrap_or(default);
            ui.horizontal(|ui| {
                ui.label(legend("sql"));
                if ui.add(Squelch::new(&mut db, lo, hi, measured, open)).changed() {
                    ch.squelch_db = Some(db);
                    changed = true;
                }
                // At the bottom of its range the squelch passes everything,
                // and saying so is more use than printing the number that
                // happens to be there.
                let text = if db <= lo + 0.5 {
                    "off".to_string()
                } else {
                    format!("{db:.0}{}", if ratio { "" } else { " dBFS" })
                };
                ui.label(value(text).size(10.0));
            });
            // The reading the threshold is being set against. Without it the
            // control is a number to guess at, and the right number differs
            // by mode and moves with the RF gain.
            ui.horizontal(|ui| {
                ui.add_space(28.0);
                hint(ui, &format!("now {measured:.0} dB"));
            });
        }
        changed
    }

    /// What the radio is hearing on the channel being listened to.
    ///
    /// This belongs inside the channel rather than beside the list: a station
    /// name is a property of one tuned frequency, and with several channels
    /// configured a panel-level readout gives no clue which one it describes.
    fn channel_rds(ui: &mut egui::Ui, st: &StationInfo, blend: f32) {
        if st.is_empty() && blend <= 0.01 {
            return;
        }
        ui.add_space(6.0);
        ui.separator();
        ui.horizontal(|ui| {
            ui.label(legend("rds"));
            if let Some(pi) = st.pi {
                ui.label(legend(&format!("PI {pi:04X}")));
            }
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                // Stereo belongs here too: it is a property of this station,
                // and it fades with the blend because the audio does.
                let t = blend.clamp(0.0, 1.0);
                if t > 0.01 {
                    let c = theme::TRACE.gamma_multiply(0.35 + 0.65 * t);
                    ui.label(value(if t > 0.99 { "STEREO" } else { "BLEND" }).size(10.0).color(c));
                }
            });
        });
        if let Some(n) = &st.name {
            // Cyan, not amber: this is what the radio heard, not something the
            // operator set.
            ui.label(value(n).size(15.0).color(theme::TRACE));
        }
        if let Some(p) = st.pty {
            ui.label(legend(p));
        }
        if let Some(rt) = &st.radiotext {
            ui.add_space(2.0);
            // Radiotext is up to 64 characters and the strip is narrow, so let
            // it wrap rather than truncating a song title mid-word.
            ui.label(egui::RichText::new(rt).color(theme::LEGEND).size(11.0));
        }
    }

    fn strip(&mut self, ui: &mut egui::Ui) {
        Panel::right("channels")
            .default_size(285.0)
            .frame(
                egui::Frame::NONE
                    .fill(theme::PANEL)
                    .inner_margin(egui::Margin::symmetric(12, 10)),
            )
            .show(ui, |ui| {
                ui.label(legend("channels"));
                ui.add_space(6.0);

                // The master, which every channel's own level runs into.
                let out_level = self.radio.as_ref().map(|r| r.status.out_level()).unwrap_or(0.0);
                let call_level =
                    self.radio.as_ref().map(|r| r.status.call_level()).unwrap_or(0.0);
                ui.horizontal(|ui| {
                    ui.label(legend("master"));
                    if ui.add(Fader::new(&mut self.audio.volume, out_level).width(VU_W)).changed() {
                        self.send(Cmd::Volume(self.audio.volume));
                    }
                    let all_muted = !self.audio.channels.is_empty()
                        && self.audio.channels.iter().all(|c| c.muted || !c.on);
                    if crate::icons::icon_button(
                        ui,
                        if all_muted { crate::icons::Icon::Mute } else { crate::icons::Icon::Sound },
                        "Mute every channel",
                        true,
                        all_muted,
                    )
                    .clicked()
                    {
                        for c in &mut self.audio.channels {
                            c.muted = !all_muted;
                        }
                        self.send_channels();
                    }
                });

                // Call audio has one level for the lot, beside the master:
                // it is not a channel anybody tuned, it is whatever the front
                // ends decode, and mixing it belongs where every other level
                // in the receiver is set.
                ui.horizontal(|ui| {
                    ui.label(legend("calls"));
                    let mut changed = ui
                        .add(Fader::new(&mut self.audio.call_volume, call_level).width(VU_W))
                        .changed();
                    if crate::icons::icon_button(
                        ui,
                        if self.audio.call_muted {
                            crate::icons::Icon::Mute
                        } else {
                            crate::icons::Icon::Sound
                        },
                        "Mute call audio",
                        true,
                        self.audio.call_muted,
                    )
                    .clicked()
                    {
                        self.audio.call_muted = !self.audio.call_muted;
                        changed = true;
                    }
                    if changed {
                        self.send(Cmd::CallVolume {
                            volume: self.audio.call_volume,
                            muted: self.audio.call_muted,
                        });
                    }
                });
                // The gain control, with what it is doing beside it: a call
                // arrives at whatever level the transmitting radio's
                // microphone was set to, which is not something a listener
                // can fix at the far end.
                ui.horizontal(|ui| {
                    ui.add_space(4.0);
                    if ui.checkbox(&mut self.audio.call_agc, "AGC").changed() {
                        self.send(Cmd::CallAgc(self.audio.call_agc));
                    }
                    let db = self
                        .radio
                        .as_ref()
                        .map(|r| r.status.call_gain_db())
                        .unwrap_or(0.0);
                    if self.audio.call_agc && db.abs() > 0.1 {
                        ui.label(value(format!("{db:+.0} dB")).size(11.0));
                    }
                });

                ui.add_space(8.0);

                if self.audio.channels.is_empty() {
                    ui.label(
                        egui::RichText::new("Click the spectrum to tune a channel.")
                            .color(theme::LEGEND)
                            .size(12.0),
                    );
                }

                let states: Vec<ChannelState> =
                    self.radio.as_ref().map(|r| r.status.channel_states()).unwrap_or_default();
                let mut remove = None;
                let mut tune = None;
                for (i, ch) in self.audio.channels.iter_mut().enumerate() {
                    let active = self.audio.listening == Some(i);
                    // Both strips take the panel fill. The selected one used a
                    // lighter wash, which was the exact colour of a slider's
                    // handle and trough, so the volume control disappeared
                    // into the strip it sat on. Selection is carried by the
                    // amber edge and the lit bar instead, which is how it is
                    // marked on a mixing desk: a lamp, not a change of paint.
                    egui::Frame::NONE
                        .fill(theme::PANEL)
                        .stroke(Stroke::new(
                            if active { 1.5 } else { 1.0 },
                            if active { theme::READOUT } else { theme::ETCH },
                        ))
                        .corner_radius(2.0)
                        .inner_margin(egui::Margin::same(8))
                        .show(ui, |ui| {
                            ui.horizontal(|ui| {
                                // A lit bar marks the channel you are hearing.
                                let (r, _) = ui.allocate_exact_size(Vec2::new(3.0, 16.0), Sense::hover());
                                ui.painter().rect_filled(
                                    r,
                                    1.0,
                                    if active { theme::READOUT } else { theme::ETCH },
                                );
                                ui.add(
                                    egui::TextEdit::singleline(&mut ch.label)
                                        .desired_width(90.0)
                                        .frame(egui::Frame::NONE),
                                );
                                ui.with_layout(
                                    egui::Layout::right_to_left(egui::Align::Center),
                                    |ui| {
                                        if ui.small_button("REMOVE").clicked() {
                                            remove = Some(i);
                                        }
                                    },
                                );
                            });
                            // Per-digit, like the main tuner: the wheel over a
                            // digit steps that decade, so tuning is repeatable
                            // rather than depending on pointer speed.
                            let d = self.audio.dial.compact(ui, ch.freq, 23.0);
                            if d.changed {
                                ch.freq = d.hz;
                                tune = Some(i);
                            }
                            ui.label(legend(bands::name_at(ch.freq)));
                            ui.add_space(4.0);
                            // Two rows: broadcast modes, then the ones an
                            // amateur band needs. Six across is narrower than
                            // the panel gets on a laptop.
                            ui.horizontal(|ui| {
                                for m in [Demod::Wfm, Demod::Nfm, Demod::Am] {
                                    if ui.selectable_label(ch.demod == m, m.label()).clicked() {
                                        ch.demod = m;
                                        tune = Some(i);
                                    }
                                }
                                ui.with_layout(
                                    egui::Layout::right_to_left(egui::Align::Center),
                                    |ui| {
                                        // Every channel can be on at once and
                                        // they mix, so this is a per channel
                                        // switch rather than a choice of one.
                                        let text = if ch.on { "ON" } else { "OFF" };
                                        if ui.selectable_label(ch.on, text).clicked() {
                                            ch.on = !ch.on;
                                            tune = Some(i);
                                        }
                                    },
                                );
                            });
                            ui.horizontal(|ui| {
                                for m in [Demod::Usb, Demod::Lsb, Demod::Cw] {
                                    if ui.selectable_label(ch.demod == m, m.label()).clicked() {
                                        ch.demod = m;
                                        tune = Some(i);
                                    }
                                }
                            });
                            if ch.on {
                                // Its own level, which runs into the master,
                                // read against what it is contributing.
                                let st = states.iter().find(|s| s.id == ch.id).copied();
                                ui.add_space(4.0);
                                ui.horizontal(|ui| {
                                    ui.label(legend("vol"));
                                    let level = st.map(|s| s.level).unwrap_or(0.0);
                                    if ui.add(Fader::new(&mut ch.volume, level).width(VU_W)).changed() {
                                        tune = Some(i);
                                    }
                                    if ui.selectable_label(ch.muted, "M").clicked() {
                                        ch.muted = !ch.muted;
                                        tune = Some(i);
                                    }
                                });
                                if ch.demod == Demod::Wfm {
                                    // Each channel's own RDS, not the first
                                    // channel's: two WFM channels are usually
                                    // two different stations.
                                    let station = self
                                        .radio
                                        .as_ref()
                                        .and_then(|r| r.status.station_for(ch.id));
                                    if let Some(station) = station {
                                        let blend = st.map(|s| s.stereo_blend).unwrap_or(0.0);
                                        Self::channel_rds(ui, &station, blend);
                                    }
                                }
                                if let Some(st) = st {
                                    if Self::channel_audio(ui, ch, st) {
                                        tune = Some(i);
                                    }
                                }
                            }
                        });
                    ui.add_space(6.0);
                }

                if let Some(i) = remove {
                    self.audio.channels.remove(i);
                    match self.audio.listening {
                        Some(l) if l == i => self.audio.listening = None,
                        Some(l) if l > i => self.audio.listening = Some(l - 1),
                        _ => {}
                    }
                    self.retune_listener();
                }
                if tune.is_some() {
                    if let Some(i) = tune {
                        self.audio.listening = Some(i);
                    }
                    self.send_channels();
                }

            });
    }

    /// Open on the chain view, for screenshots and for starting where the
    /// operator left off.
    /// Open the radio's own controls, for a screenshot or a quick check.
    pub fn show_radio_settings(&mut self) {
        self.open = Some(Settings::Radio);
    }

    pub fn show_chain(&mut self) {
        self.view = View::Chain;
    }

    /// Open the scanner table.
    pub fn show_scanner_settings(&mut self) {
        self.open = Some(Settings::Scanners);
    }

    /// Open setup: language, country, band plan, position, cached data.
    pub fn show_setup(&mut self) {
        self.open = Some(Settings::App);
    }

    pub fn show_map(&mut self) {
        self.view = View::Map;
    }

    pub fn show_calls(&mut self) {
        self.view = View::Calls;
    }

    /// Point the receiver at a frequency without opening a channel on it.
    ///
    /// Distinct from [`Self::tune_to`], which also starts demodulating: ADS-B
    /// and the band scanners want the dial moved and nothing listening, since
    /// there is no audio to be had at 1090 MHz.
    pub fn set_center(&mut self, mhz: f64) {
        self.center = mhz * 1e6;
        self.send(Cmd::Center(Hz(self.center as u64)));
        self.reset_waterfall();
    }

}

/// The handle between the spectrum and the waterfall.
///
/// Drawn as a short bar rather than a full-width line: a line reads as a
/// border, and a border is not something anyone tries to drag.
fn split_grip(p: &egui::Painter, r: &Rect, hot: bool) {
    p.rect_filled(*r, 0.0, theme::CHASSIS);
    let col = if hot { theme::READOUT } else { theme::ETCH };
    let w = 46.0;
    let y = r.center().y;
    let x0 = r.center().x - w / 2.0;
    for dy in [-2.0f32, 1.0] {
        p.line_segment(
            [Pos2::new(x0, y + dy), Pos2::new(x0 + w, y + dy)],
            Stroke::new(1.0, col),
        );
    }
}

/// A level in dB, or blank when the decoder did not measure one. Blank rather
/// than a zero: a missing measurement and a strong signal must not look alike.
fn fmt_db(v: f32) -> String {
    if v.is_finite() {
        format!("{v:6.1}")
    } else {
        " -".into()
    }
}

/// Amber when a packet is loud enough to be clipping the front end, which is
/// worth seeing: a decode can fail from too much gain as easily as too little.
fn level_color(rssi_dbfs: f32) -> Color32 {
    if rssi_dbfs > -3.0 {
        theme::FAULT
    } else if rssi_dbfs > -12.0 {
        theme::READOUT
    } else {
        theme::LEGEND
    }
}

/// Fixed-width text, so columns of numbers line up and a hex dump reads as one.
fn mono(text: &str, col: Color32) -> egui::RichText {
    egui::RichText::new(text)
        .font(FontId::new(11.0, FontFamily::Name(theme::READOUT_FONT.into())))
        .color(col)
}

/// Green for a verified packet, amber for one with no check to verify, red for
/// a failed one, grey for a burst nothing claimed. The same colours are used
/// on the waterfall.
fn row_color(rec: &DecodeRecord) -> Color32 {
    if !rec.is_known() {
        return theme::LEGEND;
    }
    match rec.crc {
        Some(true) => CRC_OK,
        Some(false) => theme::FAULT,
        None => theme::READOUT,
    }
}

/// What a selected packet holds: its fields, then its bytes.
///
/// The fields come first because they are the answer; the bytes are there for
/// when the answer is wrong, or when the protocol is unknown and the bytes are
/// all there is. Both are also what a view widget would consume: a map reads
/// the fields, an image pane reads the bytes and the media type.
/// The detail pane under the packet list.
///
/// Returns whether the operator asked to hear the transmission again, which
/// the caller turns into a command: the audio device belongs to the radio
/// thread, and a view does not get to open its own.
fn packet_detail(ui: &mut egui::Ui, rec: &DecodeRecord) -> bool {
    // The burst view takes up to half the room the inspector was dragged
    // to, never less than its natural height, so dragging the divider up
    // grows the RF view and the bytes together rather than only the
    // scrollback under them. A packet without samples gets the same area,
    // blank, so the bytes sit where they did for the last packet.
    let h = (ui.available_height() * 0.5).clamp(BURST_VIEW_H, 320.0);
    match &rec.iq {
        Some(iq) => burst_view(ui, iq, h),
        None => {
            ui.label(legend("burst  no samples kept for this packet"));
            let (rect, _) = ui.allocate_exact_size(Vec2::new(ui.available_width().max(200.0), h), Sense::hover());
            ui.painter().rect_filled(rect, 2.0, theme::WELL);
        }
    }
    ui.add_space(4.0);
    // A voice transmission's payload is what was said, so the row offers to
    // say it again. The bytes below are the vocoder's, and nobody reads those.
    let mut play = false;
    if let Some(a) = &rec.audio {
        let (peak, rms) = crate::callbus::levels_db(a);
        ui.horizontal(|ui| {
            play = ui.button("PLAY").clicked();
            ui.label(legend(&format!(
                "{:.1} s of speech   peak {peak:.0} dBFS   rms {rms:.0} dBFS",
                a.seconds()
            )));
        });
        ui.add_space(4.0);
    }
    if !rec.fields.is_empty() {
        ui.horizontal_wrapped(|ui| {
            ui.spacing_mut().item_spacing.x = 14.0;
            for (k, v) in &rec.fields {
                ui.horizontal(|ui| {
                    ui.spacing_mut().item_spacing.x = 4.0;
                    ui.label(legend(k));
                    ui.label(mono(&v.to_string(), theme::VALUE));
                });
            }
        });
        ui.add_space(4.0);
    }
    hex_dump(ui, &rec.bytes);
    play
}

/// One column of the burst view: the loudest sample in the column, and the
/// mean instantaneous frequency across it, in hertz.
///
/// A burst is thousands of samples and the view a few hundred pixels wide,
/// so each column stands for a run of samples. The envelope keeps the peak
/// of the run, since a mark a few samples long has to stay visible, and the
/// frequency is the mean phase step across it, which for a keyed carrier
/// sits at its offset during a mark and wanders during a gap.
fn burst_columns(samples: &[common::C32], rate: f64, cols: usize) -> Vec<(f32, f32)> {
    let cols = cols.max(1);
    let n = samples.len();
    (0..cols)
        .map(|c| {
            let a = c * n / cols;
            let b = ((c + 1) * n / cols).max(a + 1).min(n);
            let env = samples[a..b].iter().map(|x| x.norm()).fold(0.0f32, f32::max);
            let mut acc = common::C32::new(0.0, 0.0);
            for i in a.max(1)..b {
                acc += samples[i] * samples[i - 1].conj();
            }
            let hz = if acc.norm_sqr() > 0.0 {
                acc.arg() / std::f32::consts::TAU * rate as f32
            } else {
                0.0
            };
            (env, hz)
        })
        .collect()
}

/// The burst as the front end saw it: its envelope filled from the floor,
/// and its instantaneous frequency drawn over it, against time. What
/// Universal Radio Hacker shows beside a burst's bits, and the view an
/// unknown device is worked out from: a keyed carrier's marks and gaps, a
/// two-tone signal's frequency stepping between its tones, a chirp's
/// frequency ramping across the width.
fn burst_view(ui: &mut egui::Ui, iq: &common::IqBurst, height: f32) {
    let secs = iq.samples.len() as f64 / iq.rate.max(1.0);
    let half_span = iq.rate / 2.0;
    ui.label(legend(&format!(
        "burst  {:.2} ms  {} samples at {:.0} kS/s  {:.4} MHz +/-{:.0} kHz",
        secs * 1e3,
        iq.samples.len(),
        iq.rate / 1e3,
        iq.center_hz as f64 / 1e6,
        half_span / 1e3,
    )));
    let width = ui.available_width().max(200.0);
    // A strip of envelope under the spectrogram: the two together are the
    // amplitude and the frequency of the burst, which between them show what
    // any of the classes looks like.
    let env_h = (height * 0.22).clamp(18.0, 48.0);
    let spec_h = (height - env_h - 2.0).max(24.0);
    let (resp, p) = ui.allocate_painter(Vec2::new(width, spec_h), Sense::hover());
    let rect = resp.rect;
    p.rect_filled(rect, 2.0, theme::WELL);
    let cols = (rect.width() as usize).max(1);
    // A short transform window: 128 samples, so the time axis resolves the
    // keying rather than averaging a spectrum over many symbols, which is
    // what a long window did and what smeared an on-off burst into a solid
    // band. 128 bins over the extraction's span is ample frequency detail
    // for what this shows. The columns overlap, one per pixel, so the time
    // detail is the panel's width rather than the window.
    let rows = 256usize;
    // A window that is a fixed fraction of the burst, so a fast and a slow
    // extraction of the same signal read alike rather than one crisp and one
    // smeared. About three hundred resolvable time cells across the burst.
    let win = (iq.samples.len() / 300).clamp(8, rows);
    let img = dsp::spectrum::spectrogram(&iq.samples, cols, rows, win);
    let n = img.len() / cols;
    // The floor is the median cell; the top is the peak. A fixed range would
    // wash out a weak burst or clip a strong one, and the burst is all there
    // is on screen so its own range is the right one.
    let mut sorted: Vec<f32> = img.iter().copied().filter(|v| v.is_finite()).collect();
    sorted.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let floor = sorted.get(sorted.len() / 2).copied().unwrap_or(-60.0);
    let span = (0.0 - floor).max(6.0);
    let mut pixels = vec![Color32::BLACK; cols * n];
    for r in 0..n {
        // Row zero of the transform is the lowest frequency; the screen has
        // the highest at the top, so the image is filled upside down.
        let dst = (n - 1 - r) * cols;
        for c in 0..cols {
            let v = ((img[r * cols + c] - floor) / span).clamp(0.0, 1.0);
            pixels[dst + c] = crate::waterfall::colormap(v);
        }
    }
    let image = ColorImage {
        size: [cols, n],
        pixels,
        source_size: egui::Vec2::new(cols as f32, n as f32),
    };
    // Linear filtering fills the panel height smoothly from the transform's
    // rows, which reads as a spectrogram rather than a grid of cells.
    let tex = ui.ctx().load_texture("burst_spectrogram", image, TextureOptions::LINEAR);
    p.image(
        tex.id(),
        rect,
        Rect::from_min_max(Pos2::ZERO, Pos2::new(1.0, 1.0)),
        Color32::WHITE,
    );

    let font = FontId::new(9.0, FontFamily::Name(theme::LEGEND_FONT.into()));
    p.text(
        Pos2::new(rect.left() + 4.0, rect.top() + 2.0),
        Align2::LEFT_TOP,
        format!("+{:.0} kHz", half_span / 1e3),
        font.clone(),
        theme::LEGEND,
    );
    p.text(
        Pos2::new(rect.left() + 4.0, rect.bottom() - 2.0),
        Align2::LEFT_BOTTOM,
        format!("-{:.0} kHz", half_span / 1e3),
        font.clone(),
        theme::LEGEND,
    );
    p.text(
        Pos2::new(rect.right() - 4.0, rect.top() + 2.0),
        Align2::RIGHT_TOP,
        format!("{:.2} ms", secs * 1e3),
        font.clone(),
        theme::LEGEND,
    );
    // DC line, so a reader knows where zero frequency sits.
    let mid = rect.center().y;
    p.line_segment(
        [Pos2::new(rect.left(), mid), Pos2::new(rect.right(), mid)],
        Stroke::new(1.0, Color32::from_rgba_unmultiplied(0x8B, 0x92, 0x9C, 40)),
    );

    // The envelope beneath, filled from its own floor.
    ui.add_space(2.0);
    let (eresp, ep) = ui.allocate_painter(Vec2::new(width, env_h), Sense::hover());
    let erect = eresp.rect;
    ep.rect_filled(erect, 2.0, theme::WELL);
    let env = burst_columns(&iq.samples, iq.rate, erect.width() as usize);
    let peak = env.iter().map(|(e, _)| *e).fold(1e-6f32, f32::max);
    for (c, (e, _)) in env.iter().enumerate() {
        let h = (e / peak) * (erect.height() - 3.0);
        let x = erect.left() + c as f32 + 0.5;
        ep.line_segment(
            [Pos2::new(x, erect.bottom() - 1.0), Pos2::new(x, erect.bottom() - 1.0 - h)],
            Stroke::new(
                1.0,
                Color32::from_rgba_unmultiplied(theme::TRACE.r(), theme::TRACE.g(), theme::TRACE.b(), 150),
            ),
        );
    }
    ep.text(
        Pos2::new(erect.left() + 4.0, erect.top() + 1.0),
        Align2::LEFT_TOP,
        "envelope",
        font,
        theme::LEGEND,
    );

    if let Some(pos) = resp.hover_pos() {
        let t = ((pos.x - rect.left()) / rect.width()).clamp(0.0, 1.0) as f64 * secs;
        let hz = (mid - pos.y) / (rect.height() / 2.0) * half_span as f32;
        resp.on_hover_text(format!("{:.3} ms   {:+.1} kHz", t * 1e3, hz / 1e3));
    }
}

/// Offset, hex, and printable ASCII, sixteen bytes to the line.
///
/// The bytes are what a protocol is worked out from, so they are shown as they
/// are rather than summarised. For an unknown burst these are the bits sliced
/// under a guessed coding, which is a guess about the framing and not about
/// the reception.
fn hex_dump(ui: &mut egui::Ui, bytes: &[u8]) {
    if bytes.is_empty() {
        ui.label(legend("no bits could be read from this burst"));
        return;
    }
    egui::ScrollArea::vertical()
        .auto_shrink([false, false])
        .id_salt("hex")
        .show(ui, |ui| {
            for (i, row) in bytes.chunks(16).enumerate() {
                let hex: String = row
                    .iter()
                    .enumerate()
                    .map(|(k, b)| if k == 7 { format!("{b:02x}  ") } else { format!("{b:02x} ") })
                    .collect();
                let ascii: String = row
                    .iter()
                    .map(|b| if b.is_ascii_graphic() || *b == b' ' { *b as char } else { '.' })
                    .collect();
                ui.horizontal(|ui| {
                    ui.spacing_mut().item_spacing.x = 8.0;
                    ui.label(mono(&format!("{:04x}", i * 16), theme::LEGEND));
                    ui.label(mono(&format!("{hex:<49}"), theme::VALUE));
                    ui.label(mono(&ascii, theme::TRACE));
                });
            }
        });
}

/// Group a large count so it can be read at a glance rather than counted.
fn thousands(n: u64) -> String {
    let s = n.to_string();
    let mut out = String::with_capacity(s.len() + s.len() / 3);
    for (i, c) in s.chars().enumerate() {
        if i > 0 && (s.len() - i).is_multiple_of(3) {
            out.push(' ');
        }
        out.push(c);
    }
    out
}


#[cfg(test)]
mod tests {
    use super::*;

    fn app() -> App {
        let mut a = App {
            center: 100_000_000.0,
            rate: 2_000_000.0,
            ..Default::default()
        };
        // The waterfall holds history from where the radio actually is,
        // which after a settled tune is the same place.
        a.scope.wf_center = 100_000_000.0;
        a.scope.db_center = 100_000_000.0;
        a
    }

    fn channel(app: &mut App, offset: f64, on: bool, volume: f32) {
        let freq = app.center + offset;
        let id = app.audio.next_id as u64;
        app.audio.next_id += 1;
        app.audio.channels.push(Channel {
            id,
            freq,
            demod: Demod::Nfm,
            label: format!("CH{id}"),
            on,
            volume,
            muted: false,
            squelch_db: None,
            agc: true,
        });
    }

    #[test]
    fn every_channel_that_is_on_goes_to_the_mixer() {
        // The point of the mixer: several channels at once, each with its own
        // level, not one at a time.
        let mut a = app();
        channel(&mut a, 100_000.0, true, 0.8);
        channel(&mut a, -250_000.0, true, 0.3);
        channel(&mut a, 400_000.0, false, 1.0);

        let specs = a.channel_specs();
        assert_eq!(specs.len(), 2, "a channel that is off should not be demodulated");
        assert_eq!(specs[0].offset_hz, 100_000.0);
        assert_eq!(specs[1].offset_hz, -250_000.0);
        assert_eq!(specs[1].volume, 0.3, "each channel keeps its own level");
    }

    #[test]
    fn a_channel_keeps_its_identity_when_a_neighbour_is_removed() {
        // Chains are matched by id on the radio thread. If these were
        // positions, removing the first channel would silently hand its
        // running chain to the second.
        let mut a = app();
        channel(&mut a, 100_000.0, true, 1.0);
        channel(&mut a, 200_000.0, true, 1.0);
        let second = a.channel_specs()[1].id;
        a.audio.channels.remove(0);
        assert_eq!(a.channel_specs()[0].id, second);
    }

    #[test]
    fn muting_everything_leaves_the_channels_running() {
        // Mute is a level, not a teardown: unmuting should not have to wait
        // for chains to be rebuilt and AGCs to settle again.
        let mut a = app();
        channel(&mut a, 100_000.0, true, 1.0);
        for c in &mut a.audio.channels {
            c.muted = true;
        }
        let specs = a.channel_specs();
        assert_eq!(specs.len(), 1);
        assert!(specs[0].muted);
    }

    fn rect() -> Rect {
        Rect::from_min_size(Pos2::new(0.0, 0.0), Vec2::new(1000.0, 400.0))
    }


    fn record(freq: f64, crc: Option<bool>) -> DecodeRecord {
        DecodeRecord {
            at: std::time::Instant::now(),
            freq,
            model: "Fineoffset-WHx080".into(),
            channel_hz: 31_250.0,
            modulation: "OOK",
            detail: "temperature_c=16.2 humidity_pct=89".into(),
            fields: vec![
                ("temperature_c".into(), common::Value::Float(16.2)),
                ("humidity_pct".into(), common::Value::Int(89)),
            ],
            media_type: pipeline::event::media::BYTES,
            rssi_dbfs: -18.0,
            snr_db: 21.5,
            bytes: vec![0xab, 0xcd],
            crc,
            iq: None,
            audio: None,
        }
    }

    #[test]
    fn the_scope_split_is_adjustable_and_bounded() {
        let mut a = app();
        assert_eq!(a.scope.plot_frac, DEFAULT_PLOT_FRAC);
        // Dragging past either end clamps rather than collapsing a pane: a
        // two pixel waterfall is not a smaller waterfall, it is a broken one.
        for want in [0.0f32, 1.0, 0.6] {
            a.scope.plot_frac = want.clamp(*PLOT_FRAC_RANGE.start(), *PLOT_FRAC_RANGE.end());
            assert!(PLOT_FRAC_RANGE.contains(&a.scope.plot_frac), "{want} left {}", a.scope.plot_frac);
        }
        assert!(*PLOT_FRAC_RANGE.start() > 0.0 && *PLOT_FRAC_RANGE.end() < 1.0);
    }

    #[test]
    fn the_split_moves_the_boundary_the_way_the_pointer_went() {
        // The mapping the drag uses: pointer y within the pane becomes the
        // spectrum's share of it.
        let full = Rect::from_min_size(Pos2::new(0.0, 100.0), Vec2::new(1000.0, 800.0));
        let usable = full.height() - 16.0 - SPLIT_GRIP_H;
        let frac_at = |y: f32| ((y - full.top() - SPLIT_GRIP_H / 2.0) / usable)
            .clamp(*PLOT_FRAC_RANGE.start(), *PLOT_FRAC_RANGE.end());

        let up = frac_at(300.0);
        let down = frac_at(700.0);
        assert!(down > up, "dragging down must grow the spectrum");
        // A quarter of the way down the pane is about a quarter of the split.
        assert!((frac_at(full.top() + usable * 0.25) - 0.25).abs() < 0.02);
    }


    #[test]
    fn logged_packets_are_numbered_in_arrival_order() {
        let mut a = app();
        a.log_decodes(vec![record(a.center, None), record(a.center, None)]);
        assert_eq!(a.log.decodes[0].id, 1);
        assert_eq!(a.log.decodes[1].id, 2);
    }

    #[test]
    fn hiding_unknowns_does_not_discard_them() {
        // The filter is a view, not a policy: turning it back on must show the
        // bursts that arrived while it was off.
        let mut a = app();
        let mut unknown = record(a.center, None);
        unknown.model = "unknown".into();
        a.log_decodes(vec![unknown, record(a.center, Some(true))]);
        a.log.show_unknown = false;
        assert_eq!(a.log.decodes.len(), 2, "hiding must not drop anything");
        assert_eq!(a.log.decodes.iter().filter(|l| !l.rec.is_known()).count(), 1);
    }

    #[test]
    fn a_burst_reduces_to_columns_that_keep_its_marks_and_its_frequency() {
        // A tone 10 kHz up, keyed on for the middle third: one column per
        // third, and the middle one is loud at +10 kHz.
        let rate = 100_000.0;
        let mut iq = vec![common::C32::new(0.0, 0.0); 3000];
        for (i, x) in iq.iter_mut().enumerate().take(2000).skip(1000) {
            let ph = std::f64::consts::TAU * 10_000.0 * i as f64 / rate;
            *x = common::C32::new(ph.cos() as f32, ph.sin() as f32);
        }
        let cols = burst_columns(&iq, rate, 3);
        assert!(cols[0].0 < 0.01 && cols[2].0 < 0.01, "{cols:?}");
        assert!(cols[1].0 > 0.99, "{cols:?}");
        assert!((cols[1].1 - 10_000.0).abs() < 50.0, "{cols:?}");
    }

    #[test]
    fn the_packet_log_is_bounded() {
        let mut a = app();
        for i in 0..(DECODE_LOG_MAX + 120) {
            a.log_decodes(vec![record(100_000_000.0 + i as f64, Some(true))]);
        }
        assert_eq!(a.log.decodes.len(), DECODE_LOG_MAX);
        // The oldest are the ones dropped, so the newest packet is still there.
        let newest = 100_000_000.0 + (DECODE_LOG_MAX + 119) as f64;
        assert_eq!(a.log.decodes.last().unwrap().rec.freq, newest);
        // Numbers keep counting past what the list holds, so a row keeps the
        // number it was given.
        assert_eq!(a.log.decodes.last().unwrap().id, (DECODE_LOG_MAX + 120) as u64);
    }

    /// The scope pane over an app's state, for the geometry tests.
    fn scope_of(a: &mut App) -> scope::Scope<'_> {
        scope::Scope {
            st: &mut a.scope,
            channels: &mut a.audio.channels,
            listening: a.audio.listening,
            center: a.center,
            rate: a.rate,
            radio: None,
            scanners: &a.scanners,
            patch: &a.chain.patch,
            decode_on: a.decode_on,
            acts: Vec::new(),
        }
    }

    #[test]
    fn frequency_mapping_round_trips() {
        let mut a = app();
        let a = scope_of(&mut a);
        let r = rect();
        for hz in [99_000_000.0, 100_000_000.0, 100_750_000.0] {
            let back = a.hz_at(&r, a.x_of(&r, hz));
            assert!((back - hz).abs() < 1.0, "{hz} came back as {back}");
        }
    }

    fn with_channels(freqs: &[f64]) -> App {
        let mut a = app();
        a.center = 95_000_000.0;
        a.rate = 2_400_000.0;
        for f in freqs {
            a.audio.channels.push(Channel {
                id: 1,
                freq: *f,
                demod: Demod::Wfm,
                label: "t".into(),
                on: true,
                volume: 0.8,
                muted: false,
                squelch_db: None,
                agc: true,
            });
        }
        a
    }

    /// The band plan is process-wide, so the tests that move it cannot run
    /// beside each other: one would assert on the other's value.
    static PLAN_LOCK: std::sync::Mutex<()> = std::sync::Mutex::new(());

    #[test]
    fn a_saved_locale_is_put_into_effect_at_startup() {
        let _g = PLAN_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let mut s = crate::session::Session {
            country: "US".into(),
            band_plan: "americas".into(),
            language: "en".into(),
            ..Default::default()
        };
        apply_locale(&mut s);
        assert_eq!(crate::bands::plan(), crate::bands::Plan::Americas);
        assert_eq!(crate::i18n::language(), crate::i18n::Language::English);
        // Put it back: the plan is global, and a test that leaves it changed
        // renames every band for whatever runs next.
        crate::bands::set_plan(crate::bands::Plan::Europe);
    }

    #[test]
    fn a_first_run_takes_its_plan_from_the_country_it_infers() {
        let _g = PLAN_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let mut s = crate::session::Session { country: "JP".into(), ..Default::default() };
        // Nothing saved for the plan, but the country is known, so the plan
        // follows it rather than defaulting to wherever the author lives.
        s.band_plan = crate::locale::by_code(&s.country).unwrap().plan.id().to_string();
        apply_locale(&mut s);
        assert_eq!(crate::bands::plan(), crate::bands::Plan::AsiaPacific);
        crate::bands::set_plan(crate::bands::Plan::Europe);
    }

    #[test]
    fn a_marker_can_be_grabbed_from_further_than_the_drag_threshold() {
        // egui only reports a drag once the pointer has moved about 6 px, and
        // the hit test runs at that moment. A tolerance at or under the
        // threshold means the pointer has always already left the marker, and
        // every attempt to drag a channel pans the view instead.
        assert!(GRAB_PX > 6.0);
    }

    #[test]
    fn grabbing_is_a_fixed_distance_on_screen_at_any_span() {
        let rect = Rect::from_min_size(Pos2::ZERO, Vec2::new(1000.0, 400.0));
        for rate in [250_000.0, 2_400_000.0, 20_000_000.0] {
            let mut a = with_channels(&[95_000_000.0]);
            a.rate = rate;
            let a = scope_of(&mut a);
            let x = a.x_of(&rect, 95_000_000.0);
            assert_eq!(a.channel_at(&rect, x), Some(0), "rate {rate}");
            // Just inside the grab distance, and just outside it.
            assert_eq!(a.channel_at(&rect, x + (GRAB_PX as f32) * 0.8), Some(0), "rate {rate}");
            assert_eq!(a.channel_at(&rect, x + (GRAB_PX as f32) * 1.5), None, "rate {rate}");
        }
    }

    #[test]
    fn the_nearest_marker_wins_when_two_are_close() {
        // Taking the first match would grab whichever was added earlier rather
        // than the one being pointed at.
        let rect = Rect::from_min_size(Pos2::ZERO, Vec2::new(1000.0, 400.0));
        let mut a = with_channels(&[95_000_000.0, 95_009_000.0]);
        let a = scope_of(&mut a);
        let x = a.x_of(&rect, 95_009_000.0);
        assert_eq!(a.channel_at(&rect, x), Some(1));
        let x = a.x_of(&rect, 95_000_000.0);
        assert_eq!(a.channel_at(&rect, x), Some(0));
    }

    #[test]
    fn empty_space_grabs_nothing() {
        let rect = Rect::from_min_size(Pos2::ZERO, Vec2::new(1000.0, 400.0));
        let mut a = with_channels(&[95_000_000.0]);
        let a = scope_of(&mut a);
        assert_eq!(a.channel_at(&rect, a.x_of(&rect, 94_500_000.0)), None);
    }

    #[test]
    fn the_trace_covers_the_whole_pane_when_nothing_is_pending() {
        let rect = Rect::from_min_size(Pos2::ZERO, Vec2::new(1000.0, 400.0));
        let mut a = app();
        a.center = 95_000_000.0;
        a.scope.db_center = 95_000_000.0;
        a.rate = 2_400_000.0;
        let a = scope_of(&mut a);
        assert_eq!(a.column_bins(&rect, 0, 1000, 2048).map(|x| x.0), Some(0));
        assert_eq!(a.column_bins(&rect, 999, 1000, 2048).map(|x| x.0), Some(2045));
    }

    #[test]
    fn a_pending_retune_slides_the_trace_instead_of_stretching_it() {
        // The held spectrum belongs to the old centre. Drawing it across the
        // whole pane would put every signal at the wrong frequency; it has to
        // move with the drag, because that is where its data is.
        let rect = Rect::from_min_size(Pos2::ZERO, Vec2::new(1000.0, 400.0));
        let mut a = app();
        a.rate = 2_400_000.0;
        a.scope.db_center = 95_000_000.0;
        // View dragged a quarter span right, data not yet caught up.
        a.center = 95_600_000.0;
        let a = scope_of(&mut a);
        // A quarter of a 2.4 MHz span is 512 bins of 2048, so the left of the
        // pane now shows what was a quarter of the way in.
        assert_eq!(a.column_bins(&rect, 0, 1000, 2048).map(|x| x.0), Some(512));
        // And the right quarter has no data at all yet.
        assert_eq!(a.column_bins(&rect, 900, 1000, 2048), None);
    }

    #[test]
    fn dragging_the_other_way_leaves_the_left_empty() {
        let rect = Rect::from_min_size(Pos2::ZERO, Vec2::new(1000.0, 400.0));
        let mut a = app();
        a.rate = 2_400_000.0;
        a.scope.db_center = 95_000_000.0;
        a.center = 94_400_000.0;
        let a = scope_of(&mut a);
        assert_eq!(a.column_bins(&rect, 0, 1000, 2048), None);
        assert_eq!(a.column_bins(&rect, 999, 1000, 2048).map(|x| x.0), Some(1533));
    }

    #[test]
    fn the_edges_are_the_ends_of_the_span() {
        let mut a = app();
        let a = scope_of(&mut a);
        let r = rect();
        assert!((a.hz_at(&r, r.left()) - 99_000_000.0).abs() < 1.0);
        assert!((a.hz_at(&r, r.right()) - 101_000_000.0).abs() < 1.0);
    }

    #[test]
    fn clicks_outside_the_pane_clamp_to_the_span() {
        let mut a = app();
        let a = scope_of(&mut a);
        let r = rect();
        assert!((a.hz_at(&r, -500.0) - 99_000_000.0).abs() < 1.0);
        assert!((a.hz_at(&r, 5000.0) - 101_000_000.0).abs() < 1.0);
    }

    #[test]
    fn new_channels_take_the_mode_of_their_band() {
        let mut a = app();
        a.add_channel(95.8e6);
        a.add_channel(124.0e6);
        assert_eq!(a.audio.channels[0].demod, Demod::Wfm);
        assert_eq!(a.audio.channels[1].demod, Demod::Am);
    }

    #[test]
    fn auto_scale_ignores_a_single_strong_carrier() {
        let mut a = app();
        a.scope.floor = -90.0;
        a.scope.ceil = -20.0;
        let mut db = vec![-95.0f32; 1024];
        db[500] = 0.0;
        for _ in 0..200 {
            a.rescale(&db);
        }
        assert!(a.scope.floor < -95.0, "floor tracked the carrier: {}", a.scope.floor);
        assert!(a.scope.floor > -110.0, "floor ran away: {}", a.scope.floor);
    }

    #[test]
    fn auto_scale_keeps_room_above_an_empty_band() {
        let mut a = app();
        a.scope.floor = -90.0;
        a.scope.ceil = -20.0;
        let db = vec![-95.0f32; 1024];
        for _ in 0..400 {
            a.rescale(&db);
        }
        assert!(
            a.scope.ceil - a.scope.floor >= MIN_SPAN_DB - 0.5,
            "a flat band was squeezed to {} dB",
            a.scope.ceil - a.scope.floor
        );
    }

    #[test]
    fn retuning_stays_inside_what_the_tuner_can_reach() {
        let mut a = app();
        a.retune(1.0);
        assert_eq!(a.center, 24e6);
        a.retune(9e9);
        assert_eq!(a.center, 1766e6);
    }

    #[test]
    fn fmt_hz_scales_units() {
        assert_eq!(fmt_hz(95_800_000.0), "95.8000 MHz");
        assert_eq!(fmt_hz(12_500.0), "12.5 kHz");
        assert_eq!(fmt_hz(400.0), "400 Hz");
    }

    /// The airport card must be big enough for every line it draws.
    ///
    /// It twice was not: the width was the widest line with no room for the
    /// margin the text is drawn at, so every row ran over the right edge, and
    /// the "+N more" row was drawn below a card measured without it. Both are
    /// a line outside the box, so that is what this checks.
    #[test]
    fn the_airport_card_holds_every_line_it_draws() {
        let (pad, sep, rule_gap) = (8.0, 4.0, 6.0);
        let head = [
            Vec2::new(180.0, 15.0),
            Vec2::new(90.0, 11.0),
            Vec2::new(40.0, 13.0),
        ];
        // Eleven rows: ten frequencies and the "+N more" that follows them.
        let rows: Vec<Vec2> = (0..11).map(|_| Vec2::new(150.0, 13.0)).collect();
        let l = card_layout(&head, &rows, pad, sep, rule_gap);

        for (i, s) in head.iter().chain(rows.iter()).enumerate() {
            let right = l.text_x + s.x;
            assert!(
                right <= l.size.x - pad + 1e-3,
                "line {i} ends at {right}, past the {} the card is wide",
                l.size.x
            );
            let bottom = l.ys[i] + s.y;
            assert!(
                bottom <= l.size.y - pad + 1e-3,
                "line {i} ends at {bottom}, past the {} the card is tall",
                l.size.y
            );
        }
        // The rule sits between the head and the rows, not on top of either.
        assert!(l.rule_y > l.ys[head.len() - 1]);
        assert!(l.rule_y < l.ys[head.len()]);
    }
}
