//! Instrument front panel: readout, spectrum, waterfall, channel strips.
//!
//! Three layers, and which one a thing belongs in is decided by what it needs
//! to see.
//!
//! [`widgets`] holds the controls: a meter, a fader, a squelch, a table cell.
//! Each is an `egui::Widget` over the one value it edits and knows nothing
//! about the receiver, so it can be used by any pane, twice on a row, or in a
//! test.
//!
//! Then the panes. Each is a struct that borrows its own slice of [`state`]
//! and nothing else: [`scope::Scope`], [`strip::Strip`], [`packets::Log`],
//! [`map_pane::Map`], [`chain_pane::Chain`], [`calls_pane::CallList`],
//! [`messages_pane::Msgs`]. A pane
//! cannot reach the radio. What it wants done it either pushes into the
//! command queue, for the things the receiver does, or returns as its own
//! `Action`, for the things the application does. That is what keeps a view
//! from quietly depending on another view's field, which is how this file
//! grew to three thousand lines the first time.
//!
//! `App` is the third layer: it owns the state, hands each pane its part,
//! carries out the actions, and drains the queue once a frame in
//! [`App::flush_cmds`]. Two things stay on it rather than becoming panes,
//! [`head`] and [`settings`], because neither is a view of anything: both set
//! the receiver itself, so what they borrow is most of the application.

mod burst;
mod calls_pane;
mod chain_pane;
mod head;
mod keys_pane;
mod map_pane;
mod mapview;
mod messages_pane;
mod packets;
mod scope;
mod scope_settings;
mod settings;
mod settings_rows;
mod state;
mod strip;
mod widgets;

use crate::bands;
use crate::dial::Dial;
use crate::radio::{
    ChanMode, ChannelSpec, ChannelState, Cmd, DecodeRecord, Demod, Frame, Radio, StationInfo,
};
use crate::theme::{self, legend, value};
use burst::*;
use settings::RemoteEdit;
use settings_rows::{mhz_field, ScannerRow};
use state::{Channel, Logged};
use widgets::{bin_hint, cog, cog_rect, hint, modal_title, reading, row, Fader, Squelch, Vu};
use common::{GainMode, Hz, Sps};
use egui::containers::{CentralPanel, Panel};
use egui::{Align2, Color32, ColorImage, FontFamily, FontId, Pos2, Rect, Sense, Stroke, StrokeKind, TextureOptions, Vec2};

pub struct App {
    /// What each view remembers. A pane is handed its own and nothing else,
    /// which is what stops one view reaching into another's business.
    scope: state::ScopeState,
    chain: state::ChainState,
    log: state::LogState,
    map: map_pane::MapState,
    calls: state::CallsState,
    messages: state::MessagesState,
    #[allow(dead_code)]
    keys: state::KeysState,
    audio: state::AudioState,
    /// Where the interface's waiting work runs: tile fetches now, anything
    /// else that waits on a network later. One per application rather than
    /// one per view, so a second view that needs it borrows a handle instead
    /// of standing up threads of its own.
    rt: tokio::runtime::Runtime,
    /// What the panes asked the receiver for this frame, sent once drawing
    /// is over.
    cmds: Vec<Cmd>,

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
    /// How large the raw capture folder may get, in megabytes, or `None` for
    /// no limit.
    capture_cap_mb: Option<u64>,
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
    Messages,
    Keys,
}

impl View {
    fn label(self) -> &'static str {
        match self {
            View::Spectrum => "Spectrum",
            View::Chain => "Signal chain",
            View::Map => "Map",
            View::Calls => "Calls",
            View::Messages => "Messages",
            View::Keys => "Keys",
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

/// Two workers, matching the two tile requests allowed in flight. Everything
/// this runtime carries is waiting on a network rather than computing, so
/// sizing it to the core count would buy nothing.
fn background_runtime() -> tokio::runtime::Runtime {
    tokio::runtime::Builder::new_multi_thread()
        .worker_threads(2)
        .thread_name("net")
        .enable_all()
        .build()
        .expect("background runtime")
}

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

/// Colour of a packet whose integrity check passed.
const CRC_OK: Color32 = Color32::from_rgb(0x6F, 0xD1, 0x8A);

/// How wide a fader is drawn. The channel panel is a fixed width and every
/// one of these rows ends in a mute button, which needs the rest of it.
const VU_W: f32 = 130.0;

/// How far the auto scale keeps its ceiling above the loudest bin, and the
/// least range it will show whatever the band is doing.
///
/// The floor sits just under the noise and stays there. A quiet band with
/// nothing in it used to be scaled to a twenty decibel window, which turns
/// the noise floor's own wobble into a trace filling half the plot and
/// leaves a signal arriving on top of it nowhere to go, so the ceiling is
/// held at least this far up. It was eighty, with the ceiling never under
/// -20 dB, and that pushed the floor down to -100 dB under an -85 dB noise
/// floor: noise then sat a fifth of the way up the colour ramp and a 20 dB
/// signal barely a third, and the waterfall had no contrast left. Fifty
/// keeps the grass at a tenth of the ramp and gives a signal the rest.
const PEAK_HEADROOM_DB: f32 = 12.0;
const MIN_SPAN_DB: f32 = 50.0;

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
            map: map_pane::MapState::default(),
            rt: background_runtime(),
            calls: state::CallsState::default(),
            messages: state::MessagesState::default(),
            keys: state::KeysState::default(),
            audio: state::AudioState::default(),
            cmds: Vec::new(),
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
            capture_cap_mb: Some(nodes::capture_nodes::DEFAULT_BUDGET >> 20),
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
            log_cap_mb: s.log_cap_mb,
            capture_cap_mb: s.capture_cap_mb,
            scanners: crate::scanners::Scanners::load(),
            saved: s.clone(),
            ..Default::default()
        };
        app.map.map.layers.restore(&s.map_layers);
        app.scope.restore(&s.view, s.fft);
        app.scope.db_center = s.center;
        app.scope.wf_center = s.center;
        app.audio.volume = s.volume;
        app.log.path = crate::packetlog::PacketLog::default_dir();
        app.pending_radio = Some(s);
        // What was changed about the graph, if anything was. Applied
        // whether or not manual mode is on: the mode only says whether the
        // graph can be edited now.
        if let Some((edits, places)) = crate::patch::Edits::load() {
            app.chain.edits = edits;
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
            log_cap_mb: self.log_cap_mb,
            capture_cap_mb: self.capture_cap_mb,
            manual_chain: self.chain.edit.manual,
            map_layers: self.map.map.layers.saved(),
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
            // "tuner" is not a stage the radio lists: it is the one number a
            // driver distributes across the stages it does have.
            if name == "tuner" || controls.stages.iter().any(|(s, _)| &s.name == name) {
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
        if want.manual_chain {
            let mut cmds = std::mem::take(&mut self.cmds);
            self.chain.set_manual(true, &mut cmds);
            self.cmds = cmds;
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

    /// Ask the tuner for a total gain, distributed across whatever stages the
    /// radio has. Applied once the device reports its controls, the same way
    /// a saved setting is.
    pub fn set_rf_gain(&mut self, db: f32) {
        let s = self.pending_radio.get_or_insert_with(|| self.saved.clone());
        s.gains.retain(|(n, _)| n != "tuner");
        s.gains.push(("tuner".into(), common::GainMode::Manual(db)));
    }

    /// Start on the radio whose label contains `want`, for when several are
    /// plugged in and the saved one is not the one wanted.
    pub fn set_device(&mut self, want: &str) {
        let w = want.to_lowercase();
        match self.devices.iter().find(|d| d.label.to_lowercase().contains(&w)) {
            Some(d) => self.device = Some(d.clone()),
            None => {
                let have: Vec<&str> = self.devices.iter().map(|d| d.label.as_str()).collect();
                eprintln!("no radio matching {want:?}; attached: {have:?}");
            }
        }
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
            mode: ChanMode::Audio(demod),
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
        // A new radio thread has a new graph, whose log and capture are at
        // their defaults until they are told otherwise.
        self.send(Cmd::PacketLogCap(self.log_cap_mb.map(|mb| mb << 20)));
        self.send(Cmd::CaptureCap(self.capture_cap_mb.map(|mb| mb << 20).unwrap_or(0)));
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
        // Same for the bus: a new thread has one at its defaults.
        self.send(Cmd::Volume { volume: self.audio.volume, muted: self.audio.muted });
        self.send(Cmd::CallVolume { volume: self.audio.call_volume, muted: self.audio.call_muted });
        self.send(Cmd::CallAgc(self.audio.call_agc));
        // And what was changed about the graph goes back on top of it
        // before anything else settles: the alternative is a receiver that
        // runs the automatic chain for a moment and then rebuilds into the
        // edited one.
        if !self.chain.edits.is_empty() {
            self.send(Cmd::Edits(self.chain.edits.clone()));
        }
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

    /// Hand the radio everything the panes asked for this frame.
    ///
    /// A pane cannot reach the radio: it pushes commands into a queue and
    /// this is where they leave. That is what lets a pane borrow only its own
    /// state and still change what the receiver is doing.
    fn flush_cmds(&mut self) {
        for c in std::mem::take(&mut self.cmds) {
            self.send(c);
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
            let (running, base) = running.unwrap_or_default();
            // The graph the receiver drew underneath the edits is always
            // taken: it is what the next edit is read against. The running
            // graph only when it is not the edit that was just sent, since
            // adopting our own patch back would undo anything drawn in the
            // meantime, the receiver being a rebuild behind the pointer.
            self.chain.base = base;
            if self.chain.patch_sent.as_ref() != Some(&running) {
                self.chain.patch = running;
            }
        }
        // A level set in the chain view lands on the node, and the strip
        // has to follow or the next thing it sends puts the level back.
        let (rev, audio, chans) = radio.status.levels();
        if rev != self.audio.levels_rev {
            self.audio.levels_rev = rev;
            if rev > 0 {
                self.audio.volume = audio.master;
                self.audio.muted = audio.muted;
                self.audio.call_volume = audio.calls;
                self.audio.call_muted = audio.calls_muted;
                self.audio.call_agc = audio.agc;
                for spec in chans {
                    if let Some(c) = self.audio.channels.iter_mut().find(|c| c.id == spec.id) {
                        c.volume = spec.volume;
                        c.muted = spec.muted;
                        c.squelch_db = spec.squelch_db;
                        c.agc = spec.agc;
                        if !spec.label.is_empty() {
                            c.label = spec.label;
                        }
                    }
                }
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
            self.scope.adc = f.adc;
            self.scope.adc_bad_frames = if f.adc.starved() || f.adc.clipping() {
                self.scope.adc_bad_frames.saturating_add(1)
            } else {
                0
            };
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
            // Text outlives the log for the same reason a call does: a page
            // read half an hour later is still the page that was sent.
            self.messages.list.update(&rec, rec.at);
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
        let mut cmds = std::mem::take(&mut self.cmds);
        self.calls.subscribe_new(&heard, &mut cmds);
        self.cmds = cmds;

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
        let lo = pct(0.10) - 6.0;
        let hi = (pct(0.999) + PEAK_HEADROOM_DB).max(lo + MIN_SPAN_DB);
        self.scope.floor += (lo - self.scope.floor) * 0.05;
        self.scope.ceil += (hi - self.scope.ceil) * 0.05;
    }

    /// Draw the channel strip, then hand the radio what it changed.
    fn strip_view(&mut self, ui: &mut egui::Ui) {
        let acts = strip::Strip {
            st: &mut self.audio,
            radio: self.radio.as_ref(),
            acts: Vec::new(),
            cmds: &mut self.cmds,
        }
        .show(ui);
        for a in acts {
            match a {
                strip::Action::Channels => self.send_channels(),
            }
        }
    }

    /// Draw the chain view over the graph it edits.
    fn chain_view(&mut self, ui: &mut egui::Ui) {
        chain_pane::Chain { st: &mut self.chain, cmds: &mut self.cmds }.show(ui);
    }

    /// Draw the map, and take the station position it was given.
    fn map_view(&mut self, ui: &mut egui::Ui) {
        let mut edit = self.station_edit.take();
        // Cloned rather than borrowed, so holding the runtime does not hold
        // the application while the pane borrows its own state out of it.
        let rt = self.rt.handle().clone();
        let place = map_pane::Map { st: &mut self.map, home: self.location, edit: &mut edit, rt }
            .show(ui);
        self.station_edit = edit;
        if let Some((lat, lon)) = place {
            self.set_location(lat, lon);
        }
    }

    /// Draw the packet log, then do what its buttons asked for.
    fn log_view(&mut self, ui: &mut egui::Ui) {
        let acts = packets::Log {
            st: &mut self.log,
            radio: self.radio.as_ref(),
            scanners: &self.scanners,
            center: self.center,
            rate: self.rate,
            decode_on: self.decode_on,
            cmds: &mut self.cmds,
            acts: Vec::new(),
        }
        .show(ui);
        for a in acts {
            match a {
                packets::Action::Decode(on) => {
                    self.decode_on = on;
                    self.send(Cmd::Decode(on));
                }
                packets::Action::Open(w) => self.open = Some(w),
                packets::Action::Pin { freq, model } => self.pin_channel(freq, &model),
            }
        }
    }

    /// Put a decode channel on the strip from a packet, the (+) on a log row:
    /// the frequency it arrived on, and the front end that reads it.
    ///
    /// This used to prefill a scanner block, which was a heavier answer than
    /// the question. A block sweeps the span it covers whether or not anything
    /// else in it is wanted, so keeping one frequency meant keeping the search
    /// that found it. A channel is the front end alone, at a fixed centre and
    /// width, and it runs with the scanner switched off.
    fn pin_channel(&mut self, freq: f64, model: &str) {
        let Some(kind) = front_for(model) else {
            // Nothing here reads it on its own, so the search that found it is
            // the only thing that can: leave the scanner table to say so.
            self.open = Some(Settings::Scanners);
            return;
        };
        let label = format!("{} {:.4}", crate::chain::front_label(kind), freq / 1e6);
        self.push_channel(freq, ChanMode::Decode(kind.to_string()), Some(label));
    }

    /// Draw the call list, then do what its buttons asked for.
    fn call_view(&mut self, ui: &mut egui::Ui) {
        let act = calls_pane::CallList {
            st: &mut self.calls,
            audio: &mut self.audio,
            radio: self.radio.as_ref(),
            cmds: &mut self.cmds,
        }
        .show(ui);
        match act {
            Some(calls_pane::Action::Tune(hz)) => self.set_center(hz / 1e6),
            Some(calls_pane::Action::Clear) => self.calls.list.clear(),
            None => {}
        }
    }

    /// Draw the message list, then do what its buttons asked for.
    fn message_view(&mut self, ui: &mut egui::Ui) {
        let act = messages_pane::Msgs { st: &mut self.messages }.show(ui);
        match act {
            Some(messages_pane::Action::Tune(hz)) => self.set_center(hz / 1e6),
            Some(messages_pane::Action::Clear) => self.messages.list.clear(),
            None => {}
        }
    }

    /// Draw the key manager.
    fn keys_view(&mut self, ui: &mut egui::Ui) {
        keys_pane::Keys {
            st: &mut self.keys,
            radio: self.radio.as_ref(),
            cmds: &mut self.cmds,
            rt: self.rt.handle().clone(),
        }
        .show(ui);
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
        self.push_channel(freq, ChanMode::Audio(bands::demod_at(freq)), None);
    }

    /// A channel on the strip, tuned to `freq` and doing `mode` with it.
    fn push_channel(&mut self, freq: f64, mode: ChanMode, label: Option<String>) {
        let id = self.audio.next_id;
        self.audio.next_id += 1;
        self.audio.channels.push(Channel {
            id: id as u64,
            freq,
            mode,
            label: label.unwrap_or_else(|| format!("CH{id}")),
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
                label: c.label.clone(),
                offset_hz: c.freq - center,
                mode: c.mode.clone(),
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

/// The front end that reads a protocol, matched by the name the log gives it.
///
/// Against the registry's own channel front ends rather than a table of
/// protocol names, so a decoder added to the registry is pinnable the day it
/// arrives. AX.25 is the exception: it is the frame format APRS carries
/// rather than a front end of its own.
fn front_for(model: &str) -> Option<&'static str> {
    let system = model.split('-').next().unwrap_or(model).to_ascii_lowercase();
    let system = if system == "ax25" { "aprs" } else { system.as_str() };
    crate::chain::channel_fronts().iter().map(|(k, _)| *k).find(|k| *k == system)
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
            self.strip_view(ui);
        }
        {
            let _s = tracing::info_span!("log").entered();
            self.log_view(ui);
        }
        {
            let _s = tracing::info_span!("scope").entered();
            CentralPanel::default()
                .frame(egui::Frame::NONE.fill(theme::CHASSIS))
                .show(ui, |ui| match self.view {
                    View::Spectrum => self.scope_view(ui),
                    View::Chain => self.chain_view(ui),
                    View::Map => self.map_view(ui),
                    View::Calls => self.call_view(ui),
                    View::Messages => self.message_view(ui),
                    View::Keys => self.keys_view(ui),
                });
        }
        self.settings_modal(ui.ctx());
        self.remote_modal(ui.ctx());
        self.flush_cmds();
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

    pub fn show_messages(&mut self) {
        self.view = View::Messages;
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
            mode: ChanMode::Audio(Demod::Nfm),
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
                mode: ChanMode::Audio(Demod::Wfm),
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
        assert_eq!(a.audio.channels[0].mode, ChanMode::Audio(Demod::Wfm));
        assert_eq!(a.audio.channels[1].mode, ChanMode::Audio(Demod::Am));
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
        assert!(a.scope.floor > -105.0, "floor ran away: {}", a.scope.floor);
    }

    #[test]
    fn auto_scale_keeps_the_floor_just_under_the_noise() {
        let mut a = app();
        a.scope.floor = -120.0;
        a.scope.ceil = -20.0;
        // An -85 dB floor with a 20 dB signal standing in it: the noise has
        // to sit near the bottom of the ramp or the signal has no contrast.
        let mut db = vec![-85.0f32; 1024];
        for x in db[300..306].iter_mut() {
            *x = -65.0;
        }
        for _ in 0..400 {
            a.rescale(&db);
        }
        assert!((a.scope.floor + 91.0).abs() < 1.0, "floor {}", a.scope.floor);
        let span = a.scope.ceil - a.scope.floor;
        assert!((span - MIN_SPAN_DB).abs() < 1.0, "span {span}");
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

}
