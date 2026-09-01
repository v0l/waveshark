//! Instrument front panel: readout, spectrum, waterfall, channel strips.

use crate::bands;
use crate::dial::Dial;
use crate::radio::{
    ChannelSpec, ChannelState, Cmd, DecodeRecord, Demod, Frame, Radio, StationInfo,
};
use crate::theme::{self, legend, value};
use crate::waterfall::Waterfall;
use crate::wheel::Wheel;
use common::{GainMode, Hz, Sps};
use egui::containers::{CentralPanel, Panel};
use egui::{Align2, Color32, FontFamily, FontId, Pos2, Rect, Sense, Stroke, StrokeKind, Vec2};

pub struct App {
    radio: Option<Radio>,
    err: Option<String>,

    center: f64,
    rate: f64,

    db: Vec<f32>,
    wf: Waterfall,
    floor: f32,
    ceil: f32,
    auto_scale: bool,

    dial: Dial,
    /// Centre the waterfall history currently corresponds to, so a retune can
    /// slide it instead of throwing it away.
    wf_center: f64,
    /// Centre frequency of the spectrum currently held in `db`, which lags the
    /// requested centre while a retune is pending.
    db_center: f64,
    wf_pending: Vec<f32>,
    /// Where the frames feeding `wf_pending` were tuned, so a retune starts a
    /// fresh row instead of mixing two spans into one.
    wf_pending_center: f64,
    wf_last: Option<std::time::Instant>,
    rows_per_sec: f32,
    refresh: f32,
    fft_size: usize,
    scrub: Wheel,
    open: Option<Settings>,
    smoothing: f32,
    wf_top_offset: f32,
    wf_rows: usize,
    channels: Vec<Channel>,
    /// The channel whose chain the signal chain view shows.
    listening: Option<usize>,
    volume: f32,
    next_id: u32,
    fft: usize,
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
    /// Shape of the running chain, republished by the radio thread whenever it
    /// rebuilds one. Cloned rather than shared so drawing never blocks the
    /// thread that has to keep draining USB.
    chain_topo: Option<pipeline::graph::Topology>,
    chain_latency: f64,
    /// Packets decoded anywhere in the span, oldest first.
    decodes: Vec<Logged>,
    /// Number given to the next packet.
    next_packet: u64,
    /// Packet whose bytes are shown in the dump.
    selected: Option<u64>,
    /// Show bursts no protocol claimed.
    show_unknown: bool,
    /// Decoding every channel is on by default and can be turned off; it is
    /// the most expensive thing the app does.
    decode_on: bool,
    /// Whether the packet log is showing.
    log_open: bool,
    /// Share of the scope pane given to the spectrum, the rest going to the
    /// waterfall. Dragged rather than fixed: which of the two matters depends
    /// entirely on what is being looked for.
    plot_frac: f32,
    /// The split between spectrum and waterfall is being dragged.
    splitting: bool,
    /// Channel whose marker is being dragged in the spectrum.
    drag_ch: Option<usize>,
    /// Shared per-digit readout for the channel strip. Only one channel can be
    /// under the pointer, so one is enough.
    chan_dial: crate::dial::Dial,
    /// Settings as last written to disk, and when. Compared against the live
    /// ones each frame rather than tracked with a dirty flag, because every
    /// control that changes one would otherwise have to remember to set it.
    /// Every packet, appended to disk as it arrives. On unless the receiver
    /// has nowhere to write, which is the case in tests.
    /// Where the packet log is being written, for the status line. The log
    /// itself lives in the graph, on the radio thread.
    packet_log: Option<std::path::PathBuf>,
    /// Tracks, folded together from whatever on the bus reports a position:
    /// aircraft from ADS-B, vessels and marks from AIS. The most recent table
    /// published by the receiver.
    tracks: Vec<crate::tracks::Track>,
    /// Where the receiver is, when it has been told.
    location: Option<(f64, f64)>,
    /// The stage whose settings the chain view is showing, by node id.
    chain_sel: Option<usize>,
    /// Manual mode and where the stages have been dragged to.
    chain_edit: crate::chainview::Edit,
    /// Spectrum stages the operator added, from the last frame. Each covers
    /// whatever was wired into it rather than the span.
    extra_spectra: Vec<crate::radio::Spectrum>,
    /// The graph the operator has drawn, when manual mode is on.
    chain_patch: crate::patch::Patch,
    /// Which revision of it the radio thread last published, so an edit it
    /// refused can be noticed and taken back.
    chain_patch_rev: u64,
    /// The last patch handed to the radio thread. What comes back matches it
    /// when the edit built, and is the previous graph when it did not.
    chain_patch_sent: Option<crate::patch::Patch>,
    /// The stage the palette will add next.
    chain_add: String,
    /// The operator's own stage that is selected, by patch id.
    chain_pick: Option<u64>,
    /// ISO country code, or empty when nothing has chosen one.
    country: String,
    /// Where the flight map is looking, and how far it reaches.
    map: MapView,
    /// OSM tiles under it, fetched in the background.
    tiles: crate::map::Tiles,
    /// Packet feeds from other receivers, as configured here and saved in
    /// the session.
    feeds: Vec<nodes::FeedSpec>,
    /// The feed being typed into the settings modal.
    feed_host: String,
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
}

impl View {
    fn label(self) -> &'static str {
        match self {
            View::Spectrum => "Spectrum",
            View::Chain => "Signal chain",
            View::Map => "Map",
        }
    }
}

/// Packets kept in the log. About a screenful of scrollback at any plausible
/// reading speed, and bounded memory on a band that never goes quiet.
const DECODE_LOG_MAX: usize = 500;

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
            record_dir: None,
            radio: None,
            err: None,
            center: crate::session::DEFAULT_CENTER,
            rate: 2_304_000.0,
            db: Vec::new(),
            wf: Waterfall::new(512),
            floor: -90.0,
            ceil: -20.0,
            auto_scale: true,
            dial: Dial::new(),
            wf_center: crate::session::DEFAULT_CENTER,
            db_center: crate::session::DEFAULT_CENTER,
            wf_pending: Vec::new(),
            wf_pending_center: 0.0,
            wf_last: None,
            rows_per_sec: 20.0,
            refresh: 30.0,
            fft_size: 2048,
            scrub: Wheel::default(),
            open: None,
            smoothing: 0.35,
            wf_top_offset: 5.0,
            wf_rows: 512,
            channels: Vec::new(),
            listening: None,
            volume: 0.5,
            next_id: 1,
            fft: 2048,
            devices: Vec::new(),
            device: None,
            spans: Vec::new(),
            zoom: 1,
            soak: None,
            shot: None,
            shot_after: 6.0,
            decodes: Vec::new(),
            next_packet: 1,
            selected: None,
            show_unknown: true,
            decode_on: true,
            log_open: true,
            plot_frac: DEFAULT_PLOT_FRAC,
            splitting: false,
            drag_ch: None,
            chan_dial: crate::dial::Dial::new(),
            shot_at: None,
            shot_sent: false,
            dc_block: true,
            view: View::Spectrum,
            chain_topo: None,
            chain_latency: 0.0,
            packet_log: None,
            tracks: Vec::new(),
            location: None,
            chain_sel: None,
            chain_edit: crate::chainview::Edit::default(),
            extra_spectra: Vec::new(),
            chain_patch: crate::patch::Patch::default(),
            chain_patch_rev: 0,
            chain_patch_sent: None,
            chain_add: String::new(),
            chain_pick: None,
            country: String::new(),
            map: MapView::default(),
            tiles: crate::map::Tiles::new(),
            feeds: Vec::new(),
            feed_host: String::new(),
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
        let devices = crate::devices::list();
        let mut s = crate::session::Session::load();
        apply_locale(&mut s);
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
            packet_log: crate::packetlog::PacketLog::default_dir(),
            tracks: Vec::new(),
            center: s.center,
            wf_center: s.center,
            db_center: s.center,
            // The file holds the device's own rate; the app works in the
            // effective one, which zoom divides.
            rate: s.rate / s.zoom.max(1) as f64,
            zoom: s.zoom,
            fft: s.fft,
            fft_size: s.fft,
            dc_block: s.dc_block,
            decode_on: s.decode_on,
            location: s.location,
            country: s.country.clone(),
            map: MapView::default(),
            tiles: crate::map::Tiles::new(),
            feeds: s.feeds.clone(),
            feed_host: String::new(),
            feed_kind: nodes::FEED_KINDS[0],
            scanner_edit: None,
            scanners: crate::scanners::Scanners::load(),
            log_dir_edit: String::new(),
            log_dir: None,
            log_cap_mb: Some(crate::packetlog::DEFAULT_MAX_BYTES >> 20),
            station_edit: None,
            volume: s.volume,
            saved: s.clone(),
            pending_radio: Some(s),
            ..Default::default()
        };
        // The settings show where the log is going, so they start from where
        // it is actually going.
        app.log_dir = app.packet_log.clone();
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
            fft: self.fft,
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
            feeds: self.feeds.clone(),
            dc_block: self.dc_block,
            decode_on: self.decode_on,
            volume: self.volume,
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
    }

    /// Tell the tracker where the receiver is, so a single position frame
    /// resolves instead of waiting for a matching pair.
    pub fn set_location(&mut self, lat: f64, lon: f64) {
        self.location = Some((lat, lon));
        self.send(Cmd::Location(lat, lon));
    }

    /// Turn the packet log off, or point it somewhere other than the default.
    pub fn set_packet_log(&mut self, off: bool, dir: Option<std::path::PathBuf>) {
        self.packet_log = if off { None } else { dir.or_else(crate::packetlog::PacketLog::default_dir) };
        let dir = self.packet_log.clone();
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
        if self.channels.is_empty() {
            self.center = freq;
            self.send(Cmd::Center(common::Hz(freq as u64)));
        }
        self.channels.push(Channel {
            id: self.next_id as u64,
            freq,
            demod,
            label: format!("{mhz:.1}"),
            on: true,
            volume: 0.8,
            muted: false,
            squelch_db: None,
            agc: true,
        });
        self.next_id += 1;
        self.listen(self.channels.len() - 1);
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
            self.fft,
            move || c.request_repaint(),
        ));
        if self.zoom > 1 {
            self.send(Cmd::Zoom(self.zoom));
        }
        if let Some(r) = self.record_dir.clone() {
            self.send(Cmd::Record(Some(r)));
        }
        // The log is a node in the graph, so a new radio thread means a new
        // graph and it has to be told where to write again.
        if let Some(d) = self.packet_log.clone() {
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
        self.listening = None;
        self.err = None;
    }

    fn select_device(&mut self, ctx: &egui::Context, e: crate::devices::Entry) {
        if self.device.as_ref() == Some(&e) {
            return;
        }
        self.device = Some(e);
        self.listening = None;
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
        if self.view == View::Map || !self.tracks.is_empty() {
            self.tracks = radio.status.track_list.lock().clone();
        }
        if let Some(e) = radio.status.error.lock().take() {
            self.err = Some(e);
        }
        let mut frames: Vec<Frame> = Vec::new();
        while let Ok(f) = radio.frames.try_recv() {
            frames.push(f);
        }
        // Every frame is peak-held into the pending row, not just the one that
        // happens to be last in the queue. Folding only the last of each batch
        // tied a waterfall row's content to how often the interface repainted:
        // dragging a slider repaints continuously, fewer frames were thrown
        // away between drains, and the history visibly changed contrast for as
        // long as the drag lasted.
        self.chain_topo = radio.status.chain();
        self.chain_latency = radio.status.chain_latency();
        // An edit that will not build is refused and the last one that did
        // goes back, so what is on screen has to be what the receiver is
        // running rather than what was last asked for.
        let (rev, running) = radio.status.patch();
        if rev != self.chain_patch_rev {
            self.chain_patch_rev = rev;
            let running = running.unwrap_or_default();
            // Only when it is not the edit that was just sent. Adopting our
            // own patch back would undo anything drawn in the meantime, since
            // the receiver is a rebuild behind the pointer.
            if self.chain_patch_sent.as_ref() != Some(&running) {
                self.chain_patch = running;
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
            self.db_center = f.center;
            self.rate = f.rate;
            if self.auto_scale {
                self.rescale(&f.db);
            }
            self.slide_waterfall(f.center, f.db.len());

            let due = self
                .wf_last
                .map(|t| t.elapsed().as_secs_f32() >= 1.0 / self.rows_per_sec)
                .unwrap_or(true);
            if due {
                // The waterfall tops out below the trace's ceiling: the plot
                // wants headroom so peaks are not clipped flat, the colour
                // ramp wants the opposite or its hottest colours go unused.
                let pending = std::mem::take(&mut self.wf_pending);
                    self.wf.push(&pending, self.floor, self.ceil - self.wf_top_offset);
                self.wf_pending = pending;
                self.wf_pending.fill(f32::MIN);
                self.wf_last = Some(std::time::Instant::now());
            }
            self.db = f.db;
            self.extra_spectra = f.extra;
        }
    }

    /// Add decoded packets to the on-screen list, oldest first.
    ///
    /// Nothing is written here. The packet log is a node in the graph and
    /// stores what the demodulators produced, which is a better record than
    /// this list: these are conclusions, and they are bounded.
    fn log_decodes(&mut self, batch: Vec<DecodeRecord>) {
        for rec in batch {
            let id = self.next_packet;
            self.next_packet += 1;
            self.decodes.push(Logged { id, rec });
        }
        // A busy band produces packets faster than anyone reads them, and an
        // unbounded log is a slow memory leak with a scrollbar.
        if self.decodes.len() > DECODE_LOG_MAX {
            let drop = self.decodes.len() - DECODE_LOG_MAX;
            self.decodes.drain(..drop);
            // A selection that has aged out of the list must not leave the
            // dump showing bytes with no row above them.
            if self.selected.is_some_and(|id| !self.decodes.iter().any(|l| l.id == id)) {
                self.selected = None;
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
        if self.wf_pending.len() != f.db.len() || self.wf_pending_center != f.center {
            self.wf_pending = f.db.clone();
            self.wf_pending_center = f.center;
            return;
        }
        for (a, b) in self.wf_pending.iter_mut().zip(&f.db) {
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
        let d = ((center - self.wf_center) / hz_per_bin).round();
        if d != 0.0 {
            self.wf.shift(d as i32);
            self.wf_center += d * hz_per_bin;
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
        let (lo, hi) = (pct(0.10) - 6.0, pct(0.999) + 3.0);
        self.floor += (lo - self.floor) * 0.05;
        self.ceil += (hi.max(lo + 20.0) - self.ceil) * 0.05;
    }

    /// Frequency under the pointer, snapped to the band's channel plan while
    /// shift is held.
    ///
    /// Snapping is opt-in rather than always on because most of the spectrum
    /// has no legal raster, and a band that does still carries signals off it.
    fn hz_at_snapped(&self, rect: &Rect, x: f32, ui: &egui::Ui) -> f64 {
        let hz = self.hz_at(rect, x);
        if ui.input(|i| i.modifiers.shift) {
            bands::snap(hz)
        } else {
            hz
        }
    }

    fn hz_at(&self, rect: &Rect, x: f32) -> f64 {
        let t = ((x - rect.left()) / rect.width()).clamp(0.0, 1.0) as f64;
        self.center - self.rate / 2.0 + t * self.rate
    }

    /// Index of the channel marker within grabbing distance of `x`.
    ///
    /// Tolerance is in pixels, not Hz: the marker is a line on screen and the
    /// pointer is aiming at that line, so how close a grab counts as a hit must
    /// not change with the span.
    fn channel_at(&self, rect: &Rect, x: f32) -> Option<usize> {
        let tol = GRAB_PX * self.rate / rect.width().max(1.0) as f64;
        let hz = self.hz_at(rect, x);
        self.channels
            .iter()
            .enumerate()
            .filter(|(_, c)| (c.freq - hz).abs() < tol)
            .min_by(|a, b| {
                (a.1.freq - hz).abs().partial_cmp(&(b.1.freq - hz).abs()).unwrap()
            })
            .map(|(i, _)| i)
    }

    fn x_of(&self, rect: &Rect, hz: f64) -> f32 {
        let t = (hz - (self.center - self.rate / 2.0)) / self.rate;
        rect.left() + (t as f32) * rect.width()
    }

    fn retune(&mut self, hz: f64) {
        self.center = hz.clamp(24e6, 1766e6);
        self.send(Cmd::Center(Hz(self.center as u64)));
        self.retune_listener();
    }

    /// The span or bin count changed, so old rows no longer line up.
    fn reset_waterfall(&mut self) {
        self.wf.clear();
        self.wf_center = self.center;
        self.wf_pending.clear();
    }

    fn add_channel(&mut self, freq: f64) {
        let id = self.next_id;
        self.next_id += 1;
        self.channels.push(Channel {
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
        self.listening = Some(self.channels.len() - 1);
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
        self.channels
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
        if let Some(ch) = self.channels.get_mut(idx) {
            ch.on = true;
        }
        self.listening = Some(idx);
        self.send_channels();
    }

    fn retune_listener(&mut self) {
        if self.listening.is_some_and(|i| i >= self.channels.len()) {
            self.listening = None;
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

/// A squelch control that shows what it is deciding against.
///
/// A threshold with no meter beside it is a number to guess at: the operator
/// cannot tell whether 9 dB is one above the noise or ten below the station.
/// The bar is what the squelch is measuring right now, the marker is where it
/// opens, and dragging moves the marker.
fn squelch_meter(
    ui: &mut egui::Ui,
    lo: f32,
    hi: f32,
    measured: f32,
    threshold: &mut f32,
    open: bool,
) -> bool {
    let (rect, resp) =
        ui.allocate_exact_size(egui::vec2(120.0, 12.0), egui::Sense::click_and_drag());
    let at = |v: f32| {
        let t = ((v - lo) / (hi - lo)).clamp(0.0, 1.0);
        rect.left() + t * rect.width()
    };
    let p = ui.painter();
    p.rect_filled(rect, 2.0, theme::PANEL);
    let fill = egui::Rect::from_min_max(rect.min, egui::pos2(at(measured), rect.max.y));
    // Coloured by the decision rather than by the level, so a glance says
    // whether audio is getting through without reading the numbers.
    p.rect_filled(fill, 2.0, if open { theme::TRACE } else { theme::LEGEND });
    let x = at(*threshold);
    p.line_segment(
        [egui::pos2(x, rect.top()), egui::pos2(x, rect.bottom())],
        egui::Stroke::new(1.5, theme::VALUE),
    );

    let mut changed = false;
    if let Some(pos) = resp.interact_pointer_pos() {
        if resp.dragged() || resp.clicked() {
            let t = ((pos.x - rect.left()) / rect.width()).clamp(0.0, 1.0);
            *threshold = lo + t * (hi - lo);
            changed = true;
        }
    }
    changed
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
                    View::Spectrum => self.scope(ui),
                    View::Chain => self.chain(ui),
                    View::Map => self.map_view(ui),
                });
        }
        self.settings_modal(ui.ctx());
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
            if self.channels.is_empty() {
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
                        egui::ComboBox::from_id_salt("device")
                            .selected_text(cur)
                            .width(190.0)
                            .show_ui(ui, |ui| {
                                for d in &self.devices {
                                    let on = self.device.as_ref() == Some(d);
                                    if ui.selectable_label(on, &d.label).clicked() {
                                        pick = Some(d.clone());
                                    }
                                }
                                ui.separator();
                                if ui.selectable_label(false, "Rescan").clicked() {
                                    rescan = true;
                                }
                            });
                        if rescan {
                            self.devices = crate::devices::list();
                            if self.device.is_none() {
                                self.device = self.devices.first().cloned();
                                let c = ui.ctx().clone();
                                self.connect(&c);
                            }
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
                                for opt in [View::Spectrum, View::Chain, View::Map] {
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
                                self.log_open,
                            )
                            .clicked()
                            {
                                self.log_open = !self.log_open;
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

    fn settings_modal(&mut self, ctx: &egui::Context) {
        let Some(which) = self.open else { return };
        let title = match which {
            Settings::Spectrum => "Spectrum",
            Settings::Waterfall => "Waterfall",
            Settings::Radio => "Radio",
            Settings::PacketLog => "Packet log",
            Settings::Scanners => "Scanners",
            Settings::App => crate::i18n::t("settings.title"),
        };
        let r = egui::containers::Modal::new(egui::Id::new(title))
            .backdrop_color(Color32::from_black_alpha(150))
            .show(ctx, |ui| {
                ui.set_width(match which {
                    Settings::Radio | Settings::PacketLog | Settings::App => 420.0,
                    Settings::Scanners => 560.0,
                    _ => 320.0,
                });
                ui.label(legend(title));
                ui.add_space(10.0);
                match which {
                    Settings::Spectrum => self.spectrum_settings(ui),
                    Settings::Waterfall => self.waterfall_settings(ui),
                    Settings::Radio => self.radio_settings(ui),
                    Settings::PacketLog => self.packet_log_settings(ui),
                    Settings::Scanners => self.scanner_settings(ui),
                    Settings::App => self.app_settings(ui),
                }
                ui.add_space(12.0);
                ui.separator();
                ui.add_space(6.0);
                ui.horizontal(|ui| {
                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        if ui.button(crate::i18n::t("ui.close")).clicked() {
                            self.open = None;
                        }
                    });
                });
            });
        if r.should_close() {
            self.open = None;
        }
    }

    /// The scanner table: which front end runs on which frequency.
    ///
    /// A block per scanner rather than a text box over the file. The file is
    /// still the format of record and is still worth hand-editing, but the
    /// question this pane answers is "why is nothing decoding here", and the
    /// answer is a frequency compared against a list of ranges. That is a
    /// thing to show, not a thing to make somebody read.
    fn scanner_settings(&mut self, ui: &mut egui::Ui) {
        let (center, rate) = (self.center, self.rate);
        // Taken out of `self` so the closures below can borrow the rest of
        // it, and put back at the end.
        let mut rows = self
            .scanner_edit
            .take()
            .unwrap_or_else(|| self.scanners.list.iter().map(ScannerRow::from_scanner).collect());

        let live: Vec<crate::scanners::Scanner> =
            rows.iter().filter_map(ScannerRow::to_scanner).collect();
        let table = crate::scanners::Scanners { list: live };
        let active: Vec<String> =
            table.active(center, rate).into_iter().map(|s| s.name.clone()).collect();

        ui.horizontal(|ui| {
            ui.label(legend("tuned to"));
            ui.label(value(format!("{:.4} MHz", center / 1e6)).size(12.0));
            ui.add_space(8.0);
            ui.label(legend("span"));
            ui.label(value(format!("{:.0} kHz", rate / 1e3)).size(12.0));
            ui.add_space(8.0);
            ui.label(legend("running"));
            match active.is_empty() {
                false => ui.label(
                    egui::RichText::new(active.join(", ")).color(theme::TRACE).size(13.0),
                ),
                true => ui.label(egui::RichText::new("nothing").color(theme::FAULT).size(13.0)),
            };
        });
        if active.is_empty() {
            hint(ui, "No block covers this frequency and span, so nothing is decoded here. Add one, or widen a range.");
        }
        ui.add_space(8.0);

        let mut remove = None;
        let mut tune_to = None;
        egui::ScrollArea::vertical().max_height(360.0).id_salt("scanrows").show(ui, |ui| {
            for (i, r) in rows.iter_mut().enumerate() {
                let on = active.iter().any(|n| n == &r.name);
                // Running blocks are framed, so which of them the span covers
                // is visible without reading every range.
                let frame = egui::Frame::NONE
                    .fill(if on { theme::WELL } else { theme::CHASSIS })
                    .stroke(Stroke::new(1.0, if on { theme::TRACE } else { theme::ETCH }))
                    .inner_margin(egui::Margin::symmetric(8, 6))
                    .corner_radius(2);
                frame.show(ui, |ui| {
                    ui.horizontal(|ui| {
                        ui.add(
                            egui::TextEdit::singleline(&mut r.name)
                                .desired_width(120.0)
                                .hint_text("name"),
                        );
                        ui.add_space(4.0);
                        ui.label(legend("front"));
                        egui::ComboBox::from_id_salt(("front", i))
                            .selected_text(r.front.label())
                            .width(84.0)
                            .show_ui(ui, |ui| {
                                for f in crate::scanners::Front::all() {
                                    let label = f.label();
                                    // Keep the widths already typed when
                                    // switching back to banks.
                                    let pick = if matches!(f, crate::scanners::Front::Banks(_)) {
                                        r.banks_with_current_widths()
                                    } else {
                                        f
                                    };
                                    if ui
                                        .selectable_label(r.front.key() == pick.key(), label)
                                        .clicked()
                                    {
                                        r.front = pick;
                                    }
                                }
                            });
                        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                            if ui.small_button("REMOVE").clicked() {
                                remove = Some(i);
                            }
                            if ui.add_enabled(!on, egui::Button::new("TUNE")).clicked() {
                                tune_to = Some((r.lo_mhz + r.hi_mhz) / 2.0);
                            }
                        });
                    });
                    ui.horizontal(|ui| {
                        ui.label(legend("range"));
                        mhz_field(ui, &mut r.lo_mhz);
                        ui.label(legend("to"));
                        mhz_field(ui, &mut r.hi_mhz);
                        ui.label(legend("MHz"));
                        ui.add_space(8.0);
                        ui.label(legend("span"));
                        ui.add(
                            egui::DragValue::new(&mut r.span_khz)
                                .speed(10.0)
                                .range(1.0..=20_000.0)
                                .suffix(" kHz"),
                        );
                    });
                    ui.horizontal(|ui| {
                        match &mut r.front {
                            // A bank front end is defined by its channel
                            // widths; everything else by the channels that
                            // have to be inside the span.
                            crate::scanners::Front::Banks(_) => {
                                ui.label(legend("widths"));
                                ui.add(
                                    egui::TextEdit::singleline(&mut r.widths)
                                        .desired_width(180.0)
                                        .hint_text("31.25, 125 kHz"),
                                );
                                ui.label(legend("kHz"));
                            }
                            _ => {
                                ui.label(legend("channels"));
                                ui.add(
                                    egui::TextEdit::singleline(&mut r.channels)
                                        .desired_width(180.0)
                                        // Not an example of a value: a hint
                                        // that looks like data reads as data
                                        // on a row that needs none.
                                        .hint_text("none needed"),
                                );
                                ui.label(legend("MHz"));
                                ui.add_space(6.0);
                                ui.label(legend("margin"));
                                ui.add(
                                    egui::DragValue::new(&mut r.margin_khz)
                                        .speed(1.0)
                                        .range(0.0..=1000.0)
                                        .suffix(" kHz"),
                                );
                            }
                        }
                    });
                    if r.to_scanner().is_none() {
                        ui.label(
                            egui::RichText::new("needs a name and a range that goes upwards")
                                .color(theme::FAULT)
                                .size(10.0),
                        );
                    }
                });
                ui.add_space(4.0);
            }
        });

        if let Some(i) = remove {
            rows.remove(i);
        }
        ui.add_space(6.0);
        ui.horizontal(|ui| {
            if ui.button("ADD").clicked() {
                // Starts on the frequency being looked at, since wanting a
                // scanner here is why the pane is open.
                rows.push(ScannerRow::new_at(center, rate));
            }
            if ui.button("DEFAULTS").clicked() {
                rows = crate::scanners::Scanners::default()
                    .list
                    .iter()
                    .map(ScannerRow::from_scanner)
                    .collect();
            }
            let dirty = table != self.scanners;
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                // Saving writes the file and hands the table to the radio
                // thread, which rebuilds: a change to what runs on this
                // frequency has to take effect without a retune.
                // Saved either way, since the table is configuration. It only
                // reaches the graph when the graph is the table's to build.
                if ui.add_enabled(dirty, egui::Button::new("SAVE")).clicked() {
                    let _ = table.save();
                    self.scanners = table.clone();
                    self.send(Cmd::Scanners(table.clone()));
                }
                if self.chain_edit.manual {
                    ui.label(
                        egui::RichText::new(crate::i18n::t("ui.manual_locked"))
                            .color(theme::LEGEND)
                            .size(11.0),
                    );
                }
                if ui.add_enabled(dirty, egui::Button::new("REVERT")).clicked() {
                    rows = self.scanners.list.iter().map(ScannerRow::from_scanner).collect();
                }
                if dirty {
                    ui.label(egui::RichText::new("unsaved").color(theme::READOUT).size(11.0));
                }
            });
        });
        if let Some(p) = crate::scanners::Scanners::path() {
            ui.add_space(4.0);
            hint(ui, &p.display().to_string());
        }

        self.scanner_edit = Some(rows);
        if let Some(mhz) = tune_to {
            self.retune(mhz * 1e6);
        }
    }

    /// The packet log, and everything else that feeds the bus.
    ///
    /// Feeds live here rather than beside the tuner because that is what they
    /// are: another front end putting packets on the same bus, whose frames
    /// reach the packet list, the log and the flight list exactly like the
    /// ones this receiver demodulated itself.
    fn packet_log_settings(&mut self, ui: &mut egui::Ui) {
        let (logged, bytes, full) = match &self.radio {
            Some(r) => {
                use std::sync::atomic::Ordering;
                (
                    r.status.logged.load(Ordering::Relaxed),
                    r.status.log_bytes.load(Ordering::Relaxed),
                    r.status.log_full.load(Ordering::Relaxed),
                )
            }
            None => (0, 0, false),
        };

        // The log is on by default and stays on: the transmission worth
        // having is always the one before somebody thought to press record.
        // What is settable is where it goes and how large it may get.
        let mut on = self.packet_log.is_some();
        if ui.checkbox(&mut on, "Write every packet to disk").changed() {
            let dir = if on {
                self.log_dir
                    .clone()
                    .or_else(crate::packetlog::PacketLog::default_dir)
            } else {
                None
            };
            self.packet_log = dir.clone();
            self.send(Cmd::PacketLog(dir));
        }
        hint(ui, "Timings and frames as demodulated, a day per file, replayable.");
        ui.add_space(8.0);

        // What the list shows, rather than what the receiver does. An
        // unrecognised burst is still reported, logged and replayable with
        // this off; it is only kept out of the table.
        let mut unknown = self.show_unknown;
        if ui.checkbox(&mut unknown, "Show unrecognised bursts").changed() {
            self.show_unknown = unknown;
        }
        hint(
            ui,
            "Bursts that decoded to no known protocol. They are the point of scanning an unfamiliar band, and on a noisy one they bury the decodes.",
        );
        ui.add_space(8.0);

        row(ui, "directory", |ui| {
            let r = ui.add(
                egui::TextEdit::singleline(&mut self.log_dir_edit)
                    .desired_width(240.0)
                    .hint_text("where the files go"),
            );
            let typed = r.lost_focus() && ui.input(|i| i.key_pressed(egui::Key::Enter));
            if typed || ui.small_button("SET").clicked() {
                let dir = std::path::PathBuf::from(self.log_dir_edit.trim());
                if !self.log_dir_edit.trim().is_empty() {
                    self.log_dir = Some(dir.clone());
                    self.packet_log = Some(dir.clone());
                    self.send(Cmd::PacketLog(Some(dir)));
                }
            }
        });

        row(ui, "size limit", |ui| {
            let mut cap = self.log_cap_mb;
            egui::ComboBox::from_id_salt("log_cap")
                .selected_text(match cap {
                    Some(mb) => format!("{mb} MB per day"),
                    None => "no limit".into(),
                })
                .width(160.0)
                .show_ui(ui, |ui| {
                    for opt in [Some(128u64), Some(512), Some(2048), Some(8192), None] {
                        let label = match opt {
                            Some(mb) => format!("{mb} MB per day"),
                            None => "no limit".into(),
                        };
                        ui.selectable_value(&mut cap, opt, label);
                    }
                });
            if cap != self.log_cap_mb {
                self.log_cap_mb = cap;
                self.send(Cmd::PacketLogCap(cap.map(|mb| mb << 20)));
            }
        });
        hint(ui, "A runaway guard, not a budget.");
        ui.add_space(10.0);

        row(ui, "today", |ui| {
            ui.label(value(format!("{} in {logged} packets", human_bytes(bytes))).size(11.0));
        });
        if full {
            ui.add(
                egui::Label::new(
                    egui::RichText::new(
                        "The log has stopped: the day's file reached the limit. \
                         Raise it here to start again.",
                    )
                    .small()
                    .color(theme::FAULT),
                )
                .wrap(),
            );
        }

        ui.add_space(12.0);
        ui.separator();
        ui.add_space(6.0);
        ui.label(legend("feeds"));
        hint(ui, "Packets from another receiver, over TCP.");
        ui.add_space(8.0);

        let status = self.radio.as_ref().map(|r| r.status.feeds.lock().clone()).unwrap_or_default();
        let mut remove = None;
        for (i, f) in self.feeds.iter().enumerate() {
            let live = status.iter().find(|s| s.spec == *f);
            ui.horizontal(|ui| {
                let (r, _) = ui.allocate_exact_size(Vec2::new(3.0, 16.0), Sense::hover());
                ui.painter().rect_filled(
                    r,
                    1.0,
                    match live {
                        Some(s) if s.connected => CRC_OK,
                        Some(_) => theme::FAULT,
                        None => theme::ETCH,
                    },
                );
                ui.label(value(f.address()).size(11.0));
                ui.label(legend(f.kind.name));
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    if ui.small_button("REMOVE").clicked() {
                        remove = Some(i);
                    }
                    if let Some(s) = live {
                        ui.label(legend(&format!("{} frames", s.frames)));
                    }
                });
            });
            // A feed that is down says why. The alternative is a dark lamp and
            // a guess about whether it is the network, the port, or a receiver
            // somebody turned off.
            if let Some(e) = live.and_then(|s| s.error.clone()) {
                ui.add(egui::Label::new(egui::RichText::new(e).small().color(theme::FAULT)).wrap());
            }
            ui.add_space(6.0);
        }
        if let Some(i) = remove {
            self.feeds.remove(i);
            self.send(Cmd::Feeds(self.feeds.clone()));
        }

        ui.add_space(4.0);
        ui.horizontal(|ui| {
            ui.add(
                egui::TextEdit::singleline(&mut self.feed_host)
                    .desired_width(170.0)
                    .hint_text("host, or host:port"),
            );
            egui::ComboBox::from_id_salt("feed_kind")
                .selected_text(self.feed_kind.name)
                .width(90.0)
                .show_ui(ui, |ui| {
                    for k in nodes::FEED_KINDS {
                        let on = self.feed_kind.name == k.name;
                        if ui.selectable_label(on, k.name).clicked() {
                            self.feed_kind = k;
                        }
                    }
                });
            if ui.button("ADD").clicked() {
                match parse_feed(&self.feed_host, self.feed_kind) {
                    Some(spec) if !self.feeds.contains(&spec) => {
                        self.feeds.push(spec);
                        self.send(Cmd::Feeds(self.feeds.clone()));
                        self.feed_host.clear();
                    }
                    Some(_) => self.err = Some("that feed is already attached".into()),
                    None => self.err = Some("expected host or host:port".into()),
                }
            }
        });
    }

    /// Everything the radio itself can be set to.
    ///
    /// Where this receiver is, rather than what it is doing.
    ///
    /// One pane for the settings that are true of the installation and not of
    /// the session: they survive changing radio, they are asked once, and
    /// none of them belong under a cog on the spectrum.
    fn app_settings(&mut self, ui: &mut egui::Ui) {
        let t = crate::i18n::t;

        ui.label(legend(t("settings.language")));
        let mut lang = crate::i18n::language();
        egui::ComboBox::from_id_salt("app-language")
            .selected_text(lang.label())
            .width(ui.available_width())
            .show_ui(ui, |ui| {
                for l in crate::i18n::Language::ALL {
                    ui.selectable_value(&mut lang, l, l.label());
                }
            });
        crate::i18n::set_language(lang);
        hint(ui, t("settings.language.help"));
        ui.add_space(10.0);

        ui.label(legend(t("settings.country")));
        let current = crate::locale::by_code(&self.country);
        let mut pick: Option<&'static crate::locale::Country> = None;
        egui::ComboBox::from_id_salt("app-country")
            .selected_text(current.map(|c| c.name).unwrap_or("—"))
            .width(ui.available_width())
            .show_ui(ui, |ui| {
                for c in crate::locale::COUNTRIES {
                    let on = current.is_some_and(|s| s.code == c.code);
                    if ui.selectable_label(on, c.name).clicked() && !on {
                        pick = Some(c);
                    }
                }
            });
        if let Some(c) = pick {
            self.country = c.code.to_string();
            // A country decides the plan the first time and then stops having
            // an opinion, so choosing one after overriding the plan puts the
            // override back rather than leaving a mismatch nobody asked for.
            crate::bands::set_plan(c.plan);
            // The map has to open somewhere. A capital city is wrong by a
            // couple of hundred miles, which is close enough to draw with and
            // is replaced the moment a real position is typed in.
            if self.location.is_none() {
                self.set_location(c.centre.0, c.centre.1);
                self.station_edit = None;
            }
        }
        hint(ui, t("settings.country.help"));
        ui.add_space(10.0);

        ui.label(legend(t("settings.band_plan")));
        let mut plan = crate::bands::plan();
        egui::ComboBox::from_id_salt("app-band-plan")
            .selected_text(plan.label())
            .width(ui.available_width())
            .show_ui(ui, |ui| {
                for p in crate::bands::Plan::ALL {
                    ui.selectable_value(&mut plan, p, p.label());
                }
            });
        crate::bands::set_plan(plan);
        hint(ui, t("settings.band_plan.help"));
        ui.add_space(4.0);
        // The plan is abstract until it is applied to the frequency in front
        // of you, and this is the one line that makes the choice concrete.
        hint(
            ui,
            &format!(
                "{} here is {}",
                fmt_hz(self.center),
                crate::bands::name_at_in(plan, self.center)
            ),
        );
        ui.add_space(10.0);

        ui.separator();
        ui.add_space(6.0);
        ui.label(legend(t("settings.position")));
        let mut edit = self.station_edit.take();
        let set = Self::station_row(ui, self.location, &mut edit);
        self.station_edit = edit;
        if let Some((lat, lon)) = set {
            self.set_location(lat, lon);
            self.station_edit = None;
        }
        hint(ui, t("settings.position.help"));
    }

    /// Separate from the spectrum and waterfall settings because it is a
    /// different kind of thing: those change what you see, these change what
    /// the receiver does, and getting them wrong costs sensitivity or
    /// intermodulation rather than a prettier display.
    fn radio_settings(&mut self, ui: &mut egui::Ui) {
        let Some(radio) = self.radio.as_ref() else {
            ui.label(legend("no radio running"));
            return;
        };
        let controls = radio.status.radio();
        if controls.stages.is_empty() && controls.toggles.is_empty() && controls.choices.is_empty()
        {
            ui.label(legend("this device has no adjustable stages"));
            return;
        }

        for (stage, mode) in &controls.stages {
            let auto = *mode == GainMode::Auto;
            let mut db = match mode {
                GainMode::Auto => *stage.range.start(),
                GainMode::Manual(v) => *v,
            };
            ui.horizontal(|ui| {
                ui.label(legend(&stage.label));
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    if stage.auto {
                        let mut on = auto;
                        if ui.checkbox(&mut on, "Auto").changed() {
                            self.send(Cmd::GainStage(
                                stage.name.clone(),
                                if on { GainMode::Auto } else { GainMode::Manual(db) },
                            ));
                        }
                    }
                    // Under AUTO the number is the hardware's business and
                    // showing a stale one invites the operator to believe it.
                    let text =
                        if auto { "auto".to_string() } else { format!("{db:.1} dB") };
                    ui.label(value(text).size(11.0));
                });
            });
            let lo = *stage.range.start();
            let hi = *stage.range.end();
            // Snapped as it is dragged, because the hardware does it anyway:
            // a slider that glides between values the tuner cannot reach shows
            // a number the receiver is not using.
            let slider = egui::Slider::new(&mut db, lo..=hi).show_value(false);
            if ui.add_enabled(!auto, slider).changed() {
                let want = stage.quantise(db);
                self.send(Cmd::GainStage(stage.name.clone(), GainMode::Manual(want)));
            }
            if !stage.values.is_empty() {
                hint(ui, &format!("{} steps, {lo:.0} to {hi:.0} dB", stage.values.len()));
            } else if stage.step > 0.0 {
                hint(ui, &format!("{:.0} dB steps, {lo:.0} to {hi:.0} dB", stage.step));
            }
            ui.add_space(10.0);
        }

        if !controls.choices.is_empty() {
            ui.separator();
            ui.add_space(6.0);
            for c in &controls.choices {
                ui.label(legend(&c.label));
                let mut picked = c.selected.clone();
                egui::ComboBox::from_id_salt(format!("radio-choice-{}", c.name))
                    .selected_text(&picked)
                    .width(ui.available_width())
                    .show_ui(ui, |ui| {
                        for opt in &c.options {
                            ui.selectable_value(&mut picked, opt.clone(), opt);
                        }
                    });
                if picked != c.selected {
                    self.send(Cmd::Choice(c.name.clone(), picked));
                }
                hint(ui, &c.help);
                ui.add_space(8.0);
            }
        }

        if !controls.toggles.is_empty() {
            ui.separator();
            ui.add_space(6.0);
            for t in &controls.toggles {
                let mut on = t.on;
                if ui.checkbox(&mut on, &t.label).changed() {
                    self.send(Cmd::Toggle(t.name.clone(), on));
                }
                hint(ui, &t.help);
                ui.add_space(8.0);
            }
        }

        ui.separator();
        ui.add_space(6.0);
        row(ui, "Correction", |ui| {
            let mut ppm = controls.ppm;
            if ui
                .add(egui::DragValue::new(&mut ppm).speed(0.5).range(-200.0..=200.0).suffix(" ppm"))
                .changed()
            {
                self.send(Cmd::Ppm(ppm));
            }
        });
        ui.label(
            egui::RichText::new(
                "The reference oscillator is a few tens of parts per million out on a cheap dongle, which is a kilohertz or two at 145 MHz and rather more higher up. Tune a known carrier and correct until it sits on its nominal frequency.",
            )
            .small()
            .color(theme::LEGEND),
        );
        ui.add_space(10.0);

        let mut dc = self.dc_block;
        if ui.checkbox(&mut dc, "Remove the DC spur").changed() {
            self.dc_block = dc;
            self.send(Cmd::DcBlock(dc));
        }
        ui.label(
            egui::RichText::new(
                "A direct conversion receiver leaks its own local oscillator into the middle of the span, where it looks exactly like a carrier on the frequency you are tuned to. This measures the offset and subtracts it.",
            )
            .small()
            .color(theme::LEGEND),
        );
    }

    fn spectrum_settings(&mut self, ui: &mut egui::Ui) {
        row(ui, "FFT bins", |ui| {
            let mut n = self.fft_size;
            egui::ComboBox::from_id_salt("fft")
                .selected_text(n.to_string())
                .width(120.0)
                .show_ui(ui, |ui| {
                    for v in FFTS {
                        ui.selectable_value(&mut n, v, v.to_string());
                    }
                });
            if n != self.fft_size {
                self.fft_size = n;
                self.send(Cmd::Fft(n));
                self.reset_waterfall();
            }
        });
        ui.label(
            egui::RichText::new(bin_hint(self.rate, self.fft_size))
                .small()
                .color(theme::LEGEND),
        );
        ui.add_space(8.0);

        row(ui, "Refresh", |ui| {
            let mut v = self.refresh;
            egui::ComboBox::from_id_salt("fps")
                .selected_text(format!("{} fps", v as i32))
                .width(120.0)
                .show_ui(ui, |ui| {
                    for (n, f) in REFRESH {
                        ui.selectable_value(&mut v, f, format!("{n} fps"));
                    }
                });
            if (v - self.refresh).abs() > 0.01 {
                self.refresh = v;
                self.send(Cmd::Refresh(v));
            }
        });
        ui.add_space(8.0);

        row(ui, "Averaging", |ui| {
            if ui
                .add(egui::Slider::new(&mut self.smoothing, 0.02..=1.0).show_value(false))
                .changed()
            {
                self.send(Cmd::Smoothing(self.smoothing));
            }
            ui.label(value(if self.smoothing > 0.95 {
                "off".to_string()
            } else {
                format!("{:.0}%", (1.0 - self.smoothing) * 100.0)
            }));
        });
        ui.add_space(8.0);

        row(ui, "Centre spur", |ui| {
            if ui.checkbox(&mut self.dc_block, "Remove").changed() {
                self.send(Cmd::DcBlock(self.dc_block));
            }
            ui.label(
                egui::RichText::new("LO leakage at the tuned frequency")
                    .color(theme::LEGEND)
                    .size(10.0),
            );
        });
        ui.add_space(8.0);
        self.scale_settings(ui);
    }

    fn waterfall_settings(&mut self, ui: &mut egui::Ui) {
        row(ui, "Scroll rate", |ui| {
            let mut v = self.rows_per_sec;
            egui::ComboBox::from_id_salt("rows")
                .selected_text(format!("{} rows/s", v as i32))
                .width(130.0)
                .show_ui(ui, |ui| {
                    for (n, f) in SPEEDS {
                        ui.selectable_value(&mut v, f, format!("{n} rows/s"));
                    }
                });
            self.rows_per_sec = v;
        });
        ui.add_space(8.0);

        row(ui, "History", |ui| {
            let mut n = self.wf_rows;
            egui::ComboBox::from_id_salt("hist")
                .selected_text(format!("{n} rows"))
                .width(130.0)
                .show_ui(ui, |ui| {
                    for v in [256usize, 512, 1024, 2048] {
                        ui.selectable_value(&mut n, v, format!("{v} rows"));
                    }
                });
            if n != self.wf_rows {
                self.wf_rows = n;
                self.wf.set_height(n);
            }
        });
        ui.label(
            egui::RichText::new(format!(
                "{:.0} s of history at {:.0} rows/s",
                self.wf.height() as f32 / self.rows_per_sec,
                self.rows_per_sec
            ))
            .small()
            .color(theme::LEGEND),
        );
        ui.add_space(8.0);

        row(ui, "Contrast", |ui| {
            ui.add(egui::Slider::new(&mut self.wf_top_offset, 0.0..=20.0).show_value(false));
            ui.label(value(format!("{:.0} dB", self.wf_top_offset)));
        });
        ui.label(
            egui::RichText::new("How far below the trace ceiling the hottest colour sits.")
                .small()
                .color(theme::LEGEND),
        );
        ui.add_space(8.0);
        self.scale_settings(ui);
    }

    fn scale_settings(&mut self, ui: &mut egui::Ui) {
        row(ui, "Scale", |ui| {
            ui.checkbox(&mut self.auto_scale, "Auto");
        });
        ui.add_enabled_ui(!self.auto_scale, |ui| {
            row(ui, "Floor", |ui| {
                ui.add(egui::Slider::new(&mut self.floor, -140.0..=0.0).suffix(" dB"));
            });
            row(ui, "Ceiling", |ui| {
                ui.add(egui::Slider::new(&mut self.ceil, -140.0..=20.0).suffix(" dB"));
            });
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
                if squelch_meter(ui, lo, hi, measured, &mut db, open) {
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
                ui.horizontal(|ui| {
                    ui.label(legend("master"));
                    if ui
                        .add(egui::Slider::new(&mut self.volume, 0.0..=1.0).show_value(false))
                        .changed()
                    {
                        self.send(Cmd::Volume(self.volume));
                    }
                    let all_muted = !self.channels.is_empty()
                        && self.channels.iter().all(|c| c.muted || !c.on);
                    if ui.selectable_label(all_muted, "MUTE").clicked() {
                        for c in &mut self.channels {
                            c.muted = !all_muted;
                        }
                        self.send_channels();
                    }
                });

                ui.add_space(8.0);

                if self.channels.is_empty() {
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
                for (i, ch) in self.channels.iter_mut().enumerate() {
                    let active = self.listening == Some(i);
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
                            let d = self.chan_dial.compact(ui, ch.freq, 23.0);
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
                                // Its own level, which runs into the master.
                                ui.add_space(4.0);
                                ui.horizontal(|ui| {
                                    ui.label(legend("vol"));
                                    if ui
                                        .add(
                                            egui::Slider::new(&mut ch.volume, 0.0..=1.0)
                                                .show_value(false),
                                        )
                                        .changed()
                                    {
                                        tune = Some(i);
                                    }
                                    if ui.selectable_label(ch.muted, "M").clicked() {
                                        ch.muted = !ch.muted;
                                        tune = Some(i);
                                    }
                                });
                                let st = states.iter().find(|s| s.id == ch.id).copied();
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
                    self.channels.remove(i);
                    match self.listening {
                        Some(l) if l == i => self.listening = None,
                        Some(l) if l > i => self.listening = Some(l - 1),
                        _ => {}
                    }
                    self.retune_listener();
                }
                if tune.is_some() {
                    if let Some(i) = tune {
                        self.listening = Some(i);
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

    pub fn show_map(&mut self) {
        self.view = View::Map;
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

    /// The signal chain the listening channel is running.
    fn chain(&mut self, ui: &mut egui::Ui) {
        let Some(topo) = self.chain_topo.clone() else {
            ui.centered_and_justified(|ui| {
                ui.label(
                    egui::RichText::new("The radio is stopped, so no chain is running.")
                        .color(theme::LEGEND),
                );
            });
            return;
        };
        // Node ids are positions in the built graph, so a rebuild can leave
        // the selection pointing at a stage that is no longer there.
        if self.chain_sel.is_some_and(|s| !topo.nodes.iter().any(|n| n.id.0 == s)) {
            self.chain_sel = None;
        }
        // The inspector takes a column on the right when a stage is selected,
        // rather than floating over the graph: what a stage is set to is read
        // against where it sits in the chain, and a panel covering the chain
        // hides half of that.
        let mut act = crate::chainview::Interaction {
            selected: self.chain_sel,
            ..Default::default()
        };
        if self.chain_sel.is_some() {
            Panel::right("chain-inspector")
                .default_size(260.0)
                .frame(
                    egui::Frame::NONE
                        .fill(theme::PANEL)
                        .inner_margin(egui::Margin::symmetric(12, 10)),
                )
                .show(ui, |ui| {
                    egui::ScrollArea::vertical().show(ui, |ui| {
                        if let Some(sel) = self.chain_sel {
                            act.changed = crate::chainview::inspector(ui, &topo, sel);
                        }
                    });
                });
        }
        self.chain_header(ui);
        // Dragged, not only scrolled: the graph is wider and taller than the
        // pane on any real chain, and reaching for a scrollbar to see a branch
        // is not how anyone reads a diagram. In manual mode a drag moves a
        // stage instead, since dragging is how the graph is edited and the
        // two cannot both own the gesture.
        let manual = self.chain_edit.manual;
        let drawn = egui::ScrollArea::both()
            .scroll_source(egui::containers::scroll_area::ScrollSource {
                drag: if manual {
                    egui::containers::scroll_area::DragScroll::Never
                } else {
                    egui::containers::scroll_area::DragScroll::Always
                },
                ..Default::default()
            })
            .show(ui, |ui| {
                crate::chainview::draw(
                    ui,
                    &topo,
                    self.chain_latency,
                    self.chain_sel,
                    &mut self.chain_edit,
                    Some(&self.chain_patch),
                )
            })
            .inner;
        self.chain_sel = drawn.selected;
        if manual {
            if drawn.picked.is_some() {
                self.chain_pick = drawn.picked;
            }
            // Unwiring first: taking hold of a wire reports both in the same
            // frame when the drag is short, and doing it the other way round
            // would drop the wire that was just drawn.
            let mut edited = false;
            if let Some(to) = drawn.unlink {
                self.chain_patch.disconnect(to);
                edited = true;
            }
            if let Some(from) = drawn.unlink_out {
                self.chain_patch.disconnect_from(from);
                edited = true;
            }
            if let Some((from, to, port)) = drawn.link {
                self.chain_patch.connect(from, (to, port));
                edited = true;
            }
            if edited {
                self.send_patch();
            }
        }
        if let Some((id, name, value)) = act.changed.or(drawn.changed) {
            self.send(Cmd::NodeParam(id, name, value));
        }
    }

    /// The switch that decides who owns the shape of the graph, and the
    /// palette that edits it once it does.
    fn chain_header(&mut self, ui: &mut egui::Ui) {
        ui.horizontal(|ui| {
            let mut manual = self.chain_edit.manual;
            if ui.checkbox(&mut manual, "MANUAL").clicked() {
                self.set_manual_chain(manual);
            }
            if self.chain_edit.manual {
                self.chain_palette(ui);
            } else {
                ui.label(legend("built from the scanner table for this span"));
            }
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                if ui
                    .add_enabled(self.chain_edit.moved(), egui::Button::new("ARRANGE"))
                    .on_hover_text("Lay the stages out again from the graph")
                    .clicked()
                {
                    self.chain_edit.arrange();
                }
            });
        });
        ui.add_space(4.0);
    }

    /// Which stages can be added, and what to do with the one selected.
    ///
    /// The list comes from the node registry rather than from anything
    /// written here, so a decoder added to the build appears in it without
    /// this file being touched.
    fn chain_palette(&mut self, ui: &mut egui::Ui) {
        let reg = nodes::registry();
        let kinds: Vec<(String, String)> =
            reg.list().map(|d| (d.name.to_string(), d.summary.to_string())).collect();
        if self.chain_add.is_empty() {
            if let Some((name, _)) = kinds.first() {
                self.chain_add = name.clone();
            }
        }
        egui::ComboBox::from_id_salt("chain-add")
            .selected_text(self.chain_add.clone())
            .width(150.0)
            .show_ui(ui, |ui| {
                for (name, summary) in &kinds {
                    ui.selectable_value(&mut self.chain_add, name.clone(), name)
                        .on_hover_text(summary);
                }
            });
        if ui.button("ADD").clicked() && !self.chain_add.is_empty() {
            let id = self.chain_patch.add(&self.chain_add.clone());
            self.chain_pick = Some(id);
            self.send_patch();
        }
        let picked = self.chain_pick.filter(|id| self.chain_patch.stage(*id).is_some());
        if ui.add_enabled(picked.is_some(), egui::Button::new("REMOVE")).clicked() {
            if let Some(id) = picked {
                self.chain_patch.remove(id);
                self.chain_pick = None;
                self.chain_sel = None;
                self.send_patch();
            }
        }
        // Which gestures exist, and the one thing that is not editable. Only
        // the stages added here have live ports: the head of the chain, the
        // spectrum and the listening channels are the receiver's own wiring.
        ui.label(legend(&match picked.and_then(|id| self.chain_patch.stage(id)) {
            Some(s) => format!("{} selected; drag its ports to wire it up", s.kind),
            None if self.chain_patch.stages().is_empty() => {
                "add a stage: only stages added here can be wired".to_string()
            }
            None => "drag a port to wire, drag a wire off an input to move it".to_string(),
        }));
    }

    /// Hand the patch to the radio thread, remembering what was sent so that
    /// one handed back after a refusal can be told apart from an echo.
    fn send_patch(&mut self) {
        self.chain_patch_sent = Some(self.chain_patch.clone());
        self.send(Cmd::Patch(self.chain_patch.clone()));
    }

    /// Hand the shape of the graph to the operator, or give it back to the
    /// scanner table.
    pub fn set_manual_chain(&mut self, on: bool) {
        self.chain_edit.manual = on;
        if !on {
            self.chain_edit.arrange();
            self.chain_pick = None;
        }
        // Not followed by a patch of our own: the radio thread answers with
        // the graph it is running, which is what taking it over means.
        self.chain_patch_sent = None;
        self.send(Cmd::Manual(on));
    }

    fn scope(&mut self, ui: &mut egui::Ui) {
        let mut full = ui.available_rect_before_wrap();
        // A spectrum stage the operator added gets a strip of its own under
        // everything else. They cover a band rather than the span, so they
        // cannot share the main plot's axis, and stacking them is what makes
        // watching a decimated band and the whole span at once worth the
        // stage.
        if !self.extra_spectra.is_empty() {
            let each = (full.height() * 0.22).clamp(60.0, 140.0);
            let n = self.extra_spectra.len().min(3);
            let strips = Rect::from_min_max(
                Pos2::new(full.left(), full.bottom() - each * n as f32),
                full.max,
            );
            full = Rect::from_min_max(full.min, Pos2::new(full.right(), strips.top()));
            let p = ui.painter_at(strips).to_owned();
            for (i, s) in self.extra_spectra.iter().take(n).enumerate() {
                let r = Rect::from_min_size(
                    Pos2::new(strips.left(), strips.top() + each * i as f32),
                    Vec2::new(strips.width(), each),
                );
                self.extra_plot(&p, &r, s);
            }
        }
        let ribbon_h = 16.0;
        let usable = (full.height() - ribbon_h - SPLIT_GRIP_H).max(1.0);
        let plot_h = usable * self.plot_frac.clamp(*PLOT_FRAC_RANGE.start(), *PLOT_FRAC_RANGE.end());
        let plot = Rect::from_min_max(full.min, Pos2::new(full.right(), full.top() + plot_h));
        let ribbon = Rect::from_min_max(
            Pos2::new(full.left(), plot.bottom()),
            Pos2::new(full.right(), plot.bottom() + ribbon_h),
        );
        let grip = Rect::from_min_max(
            Pos2::new(full.left(), ribbon.bottom()),
            Pos2::new(full.right(), ribbon.bottom() + SPLIT_GRIP_H),
        );
        let fall = Rect::from_min_max(Pos2::new(full.left(), grip.bottom()), full.max);

        let resp = ui.allocate_rect(full, Sense::click_and_drag());
        let p = ui.painter_at(full).to_owned();
        let plot_cog = cog_rect(&plot);
        let fall_cog = cog_rect(&fall);
        p.rect_filled(plot, 0.0, theme::WELL);

        self.grid(&p, &plot);
        {
            let _s = tracing::info_span!("trace").entered();
            self.trace(&p, &plot);
        }
        // Over the trace, because the trace is filled to the floor of the
        // plot and anything drawn under it there is washed out to the fill's
        // own colour. Kept to four pixels and a low alpha so it reads as a
        // margin note rather than as a signal.
        self.scan_marks(&p, &plot);
        self.ribbon(&p, &ribbon);

        p.rect_filled(fall, 0.0, theme::CHASSIS);
        {
            let _wf = tracing::info_span!("wf_texture").entered();
            self.wf.draw(ui.ctx(), &p, fall);
        }

        self.markers(&p, &full);

        let hover = resp.hover_pos();
        let grip_hot = self.splitting || hover.is_some_and(|h| grip.contains(h));
        split_grip(&p, &grip, grip_hot);
        let plot_hot = hover.is_some_and(|h| plot_cog.contains(h));
        let fall_hot = hover.is_some_and(|h| fall_cog.contains(h));

        // Reaching for the cog is not reading the spectrum, so the crosshair
        // and its readout get out of the way rather than sitting under the
        // pointer while it is over a button.
        cog(&p, &plot_cog, plot_hot);
        cog(&p, &fall_cog, fall_hot);

        if !plot_hot && !fall_hot {
            let shift = ui.input(|i| i.modifiers.shift);
            self.cursor(&p, &full, &resp, shift);
        }

        if resp.clicked() && self.drag_ch.is_none() {
            if let Some(pos) = resp.interact_pointer_pos() {
                // Cogs sit inside the pane, so they get first refusal on a
                // click; otherwise opening settings would also drop a channel.
                if plot_cog.contains(pos) {
                    self.open = Some(Settings::Spectrum);
                } else if fall_cog.contains(pos) {
                    self.open = Some(Settings::Waterfall);
                } else if grip.contains(pos) {
                    // Dropping a channel on the divider is never what was
                    // meant; a double click there restores the default split.
                } else {
                    // Hit testing uses the true position; only the frequency a
                    // new channel lands on is snapped, so shift-clicking an
                    // existing channel still selects it.
                    match self.channel_at(&full, pos.x) {
                        Some(i) => self.listen(i),
                        None => self.add_channel(self.hz_at_snapped(&full, pos.x, ui)),
                    }
                }
            }
        }
        // Grabbing a channel marker moves that channel; grabbing anywhere else
        // pans the view. Deciding once when the drag starts means the gesture
        // cannot change meaning halfway through as the pointer moves off the
        // line it grabbed.
        if resp.drag_started() {
            // Test where the button went down, not where the pointer is now.
            // egui only reports a drag once the pointer has passed its drag
            // threshold, which is about the same distance as the grab
            // tolerance, so by this point the pointer has already left the
            // marker it grabbed and every drag looked like a pan.
            let origin = ui
                .input(|i| i.pointer.press_origin())
                .or_else(|| resp.interact_pointer_pos());
            self.splitting = origin.is_some_and(|pos| grip.contains(pos));
            self.drag_ch = origin.and_then(|pos| {
                if plot_cog.contains(pos) || fall_cog.contains(pos) || grip.contains(pos) {
                    return None;
                }
                self.channel_at(&full, pos.x)
            });
        }
        if resp.dragged() && self.splitting {
            if let Some(pos) = resp.interact_pointer_pos() {
                // Follow the pointer rather than accumulating deltas, so the
                // divider cannot drift away from the cursor over a long drag.
                let f = (pos.y - full.top() - SPLIT_GRIP_H / 2.0) / usable;
                self.plot_frac =
                    f.clamp(*PLOT_FRAC_RANGE.start(), *PLOT_FRAC_RANGE.end());
            }
        } else if resp.dragged() {
            match self.drag_ch {
                Some(i) if i < self.channels.len() => {
                    if let Some(pos) = resp.interact_pointer_pos() {
                        // Follow the pointer rather than accumulating deltas,
                        // so the marker cannot drift away from the cursor.
                        self.channels[i].freq = self.hz_at_snapped(&full, pos.x, ui);
                        if self.listening == Some(i) {
                            self.listen(i);
                        }
                    }
                }
                _ => {
                    let dx = resp.drag_delta().x as f64;
                    if dx.abs() > 0.0 {
                        self.retune(self.center - dx * self.rate / full.width() as f64);
                    }
                }
            }
        }
        if resp.drag_stopped() {
            self.drag_ch = None;
            self.splitting = false;
        }

        if resp.double_clicked() && hover.is_some_and(|h| grip.contains(h)) {
            self.plot_frac = DEFAULT_PLOT_FRAC;
        }

        // A marker under the pointer is draggable, so say so.
        if grip_hot {
            ui.ctx().set_cursor_icon(egui::CursorIcon::ResizeVertical);
        } else if self.drag_ch.is_some() {
            ui.ctx().set_cursor_icon(egui::CursorIcon::ResizeHorizontal);
        } else if let Some(h) = hover {
            if !plot_hot && !fall_hot && self.channel_at(&full, h.x).is_some() {
                ui.ctx().set_cursor_icon(egui::CursorIcon::ResizeHorizontal);
            }
        }

        // Wheel over the pane scrubs the centre frequency. A notch moves a
        // twentieth of the span, so the gesture means the same thing at every
        // zoom level.
        if resp.hovered() && !plot_hot && !fall_hot {
            let n = self.scrub.notches(ui);
            if n != 0 {
                self.retune(self.center - n as f64 * self.rate / 20.0);
            }
        }
    }

    /// Where the scanner table is listening, along the foot of the spectrum.
    ///
    /// A bank has hundreds of channels and drawing a line per channel is a
    /// grey wash that hides the spectrum it is describing, so the band is a
    /// strip and the channel grid appears as ticks only when the ticks are far
    /// enough apart to be counted. Below that the strip alone is the honest
    /// drawing: the channels are narrower than a pixel.
    fn scan_marks(&self, p: &egui::Painter, plot: &Rect) {
        if !self.decode_on {
            return;
        }
        let marks = crate::chain::scan_marks(&self.scanners, self.center, self.rate);
        if marks.is_empty() {
            return;
        }
        let col = theme::OK;
        let dim = |a: u8| Color32::from_rgba_unmultiplied(col.r(), col.g(), col.b(), a);
        // Read against the trace's own fill, which is what is behind this.
        let (fill_a, tick_a, text_a) = (110u8, 190u8, 220u8);
        // A strip at the foot of the plot, clear of the trace's baseline.
        let floor = plot.bottom() - 1.0;
        let font = FontId::new(9.0, FontFamily::Name(theme::LEGEND_FONT.into()));
        // Banks stack upward. Two of them cover the same band at different
        // channel widths, and drawn on one row they were one strip with two
        // labels printed over each other.
        let mut row = 0usize;

        for m in &marks {
            match m {
                crate::chain::ScanMark::Band { lo, hi, origin, spacing, label } => {
                    let y1 = floor - row as f32 * 6.0;
                    let y0 = y1 - 4.0;
                    row += 1;
                    let (x0, x1) = (self.x_of(plot, *lo), self.x_of(plot, *hi));
                    let (cx0, cx1) = (x0.max(plot.left()), x1.min(plot.right()));
                    if cx1 - cx0 < 1.0 {
                        continue;
                    }
                    p.rect_filled(
                        Rect::from_min_max(Pos2::new(cx0, y0), Pos2::new(cx1, y1)),
                        1.0,
                        dim(fill_a),
                    );
                    let step_px = (x1 - x0) * (*spacing as f32) / (*hi - *lo).max(1.0) as f32;
                    if step_px >= 7.0 {
                        // Stepped from a real channel centre rather than from
                        // the band edge, which is where the grid happens to be
                        // cut. Half a channel of error in a drawing of where
                        // the channels are is the whole of what it says.
                        let first =
                            ((lo - origin) / spacing).ceil() * spacing + origin - spacing / 2.0;
                        let mut hz = first;
                        while hz <= *hi + spacing {
                            let x = self.x_of(plot, hz);
                            if plot.x_range().contains(x) && x >= x0 && x <= x1 {
                                p.line_segment(
                                    [Pos2::new(x, y0 - 2.0), Pos2::new(x, y1)],
                                    Stroke::new(1.0, dim(tick_a)),
                                );
                            }
                            hz += spacing;
                        }
                    }
                    // Named at the left edge of its own strip, where it cannot
                    // be mistaken for a label on the band beside it.
                    if cx1 - cx0 > 46.0 {
                        p.text(
                            Pos2::new(cx0 + 3.0, y0 - 3.0),
                            Align2::LEFT_BOTTOM,
                            label,
                            font.clone(),
                            dim(text_a),
                        );
                    }
                }
                crate::chain::ScanMark::Channel { hz, width, label } => {
                    let (y1, y0) = (floor, floor - 4.0);
                    let x = self.x_of(plot, *hz);
                    if !plot.x_range().contains(x) {
                        continue;
                    }
                    let (x0, x1) =
                        (self.x_of(plot, hz - width / 2.0), self.x_of(plot, hz + width / 2.0));
                    if x1 - x0 >= 2.0 {
                        p.rect_filled(
                            Rect::from_min_max(
                                Pos2::new(x0.max(plot.left()), y0),
                                Pos2::new(x1.min(plot.right()), y1),
                            ),
                            1.0,
                            dim(fill_a),
                        );
                    }
                    p.line_segment(
                        [Pos2::new(x, y1 - 9.0), Pos2::new(x, y1)],
                        Stroke::new(1.0, dim(tick_a)),
                    );
                    p.text(
                        Pos2::new(x + 3.0, y1 - 9.0),
                        Align2::LEFT_BOTTOM,
                        label,
                        font.clone(),
                        dim(text_a),
                    );
                }
            }
        }
    }

    fn grid(&self, p: &egui::Painter, plot: &Rect) {
        for i in 0..=10 {
            let x = plot.left() + plot.width() * i as f32 / 10.0;
            p.line_segment(
                [Pos2::new(x, plot.top()), Pos2::new(x, plot.bottom())],
                Stroke::new(1.0, Color32::from_rgb(0x24, 0x28, 0x2E)),
            );
        }
        // Amplitude graticule, labelled in dBFS so the numbers mean something.
        for i in 1..4 {
            let y = plot.top() + plot.height() * i as f32 / 4.0;
            p.line_segment(
                [Pos2::new(plot.left(), y), Pos2::new(plot.right(), y)],
                Stroke::new(1.0, Color32::from_rgb(0x22, 0x26, 0x2B)),
            );
            let db = self.ceil - (self.ceil - self.floor) * i as f32 / 4.0;
            // The unit once, on the top line. Repeating it on every gridline
            // is three times the ink for the same fact.
            let text = if i == 1 { format!("{db:.0} dBFS") } else { format!("{db:.0}") };
            // Set at 11 rather than 9, and in the panel's legend grey rather
            // than a shade above the background. These are the numbers that
            // say what the trace is worth, and they were unreadable.
            let font = FontId::new(11.0, FontFamily::Name(theme::LEGEND_FONT.into()));
            let at = Pos2::new(plot.right() - 5.0, y - 1.0);
            let galley = p.layout_no_wrap(text, font, theme::LEGEND);
            let rect = Align2::RIGHT_BOTTOM.anchor_size(at, galley.size());
            // A backing, because the trace runs behind these and a number
            // crossed by a carrier is not a number any more.
            p.rect_filled(
                rect.expand2(Vec2::new(3.0, 1.0)),
                2.0,
                Color32::from_rgba_unmultiplied(0x14, 0x16, 0x19, 190),
            );
            p.galley(rect.min, galley, theme::LEGEND);
        }
    }

    /// Which bins of the held spectrum belong under screen column `c`.
    ///
    /// `None` where the column falls outside the data, which happens while a
    /// retune is pending and the view has moved past what has been received.
    fn column_bins(
        &self,
        plot: &Rect,
        c: usize,
        _cols: usize,
        n: usize,
    ) -> Option<(usize, usize)> {
        let lo = self.db_center - self.rate / 2.0;
        let bin = |f: f64| ((f - lo) / self.rate * n as f64).floor();
        let a = bin(self.hz_at(plot, plot.left() + c as f32));
        let b = bin(self.hz_at(plot, plot.left() + c as f32 + 1.0));
        if a < 0.0 || a >= n as f64 {
            return None;
        }
        let a = a as usize;
        Some((a, (b.clamp(0.0, n as f64) as usize).max(a + 1).min(n)))
    }

    /// One extra spectrum, with its own band under it.
    ///
    /// Drawn from its own centre and rate rather than the dial's: what is
    /// wired into it decides what it covers, and that is the whole point of
    /// having more than one.
    fn extra_plot(&self, p: &egui::Painter, r: &Rect, s: &crate::radio::Spectrum) {
        p.rect_filled(*r, 0.0, theme::WELL);
        p.line_segment(
            [r.left_top(), r.right_top()],
            Stroke::new(1.0, theme::ETCH),
        );
        let plot = Rect::from_min_max(Pos2::new(r.left(), r.top() + 12.0), r.max);
        let span = (self.ceil - self.floor).max(1.0);
        let n = s.db.len();
        if n >= 2 {
            let cols = plot.width().max(1.0) as usize;
            let mut pts = Vec::with_capacity(cols);
            for c in 0..cols {
                let a = c * n / cols.max(1);
                let b = (((c + 1) * n) / cols.max(1)).max(a + 1).min(n);
                let v = s.db[a..b].iter().copied().fold(f32::MIN, f32::max);
                let t = ((v - self.floor) / span).clamp(0.0, 1.0);
                pts.push(Pos2::new(plot.left() + c as f32, plot.bottom() - t * plot.height()));
            }
            p.add(egui::Shape::line(pts, Stroke::new(1.0, theme::TRACE)));
        }
        let name = self
            .chain_patch
            .stage(s.tag)
            .map(|st| st.kind.clone())
            .unwrap_or_else(|| "spectrum".into());
        p.text(
            Pos2::new(r.left() + 6.0, r.top() + 1.0),
            egui::Align2::LEFT_TOP,
            format!(
                "{name}   {:.4} MHz   {:.3} MS/s",
                s.center / 1e6,
                s.rate / 1e6
            ),
            FontId::new(10.0, FontFamily::Name(theme::READOUT_FONT.into())),
            theme::LEGEND,
        );
    }

    fn trace(&self, p: &egui::Painter, plot: &Rect) {
        if self.db.is_empty() {
            return;
        }
        let span = (self.ceil - self.floor).max(1.0);
        let n = self.db.len();
        let cols = plot.width().max(1.0) as usize;
        // Columns are placed by frequency rather than by bin index. While a
        // retune is pending the held spectrum belongs to a different centre,
        // and drawing it stretched across the pane would put every signal at
        // the wrong frequency. Positioning it by its own centre slides it under
        // the drag instead, which is where its data really is.
        let mut pts = Vec::with_capacity(cols);
        for c in 0..cols {
            let Some((a, b)) = self.column_bins(plot, c, cols, n) else { continue };
            // Max, not mean: averaging hides the narrow carriers that matter.
            let v = self.db[a..b].iter().copied().fold(f32::MIN, f32::max);
            let t = ((v - self.floor) / span).clamp(0.0, 1.0);
            pts.push(Pos2::new(plot.left() + c as f32, plot.bottom() - t * plot.height()));
        }
        if pts.len() < 2 {
            return;
        }
        // Fill under the trace so occupied spectrum reads as mass. Built as a
        // quad strip: a spectrum outline is wildly concave, and asking for a
        // convex polygon fill turns it into fan-shaped wedges.
        let mut mesh = egui::Mesh::default();
        let fill = Color32::from_rgba_unmultiplied(0x5C, 0xD0, 0xE8, 26);
        for w in pts.windows(2) {
            let i = mesh.vertices.len() as u32;
            for v in [
                w[0],
                w[1],
                Pos2::new(w[1].x, plot.bottom()),
                Pos2::new(w[0].x, plot.bottom()),
            ] {
                mesh.colored_vertex(v, fill);
            }
            mesh.add_triangle(i, i + 1, i + 2);
            mesh.add_triangle(i, i + 2, i + 3);
        }
        p.add(egui::Shape::mesh(mesh));
        p.add(egui::Shape::line(pts, Stroke::new(1.2, theme::TRACE)));
    }

    fn ribbon(&self, p: &egui::Painter, r: &Rect) {
        p.rect_filled(*r, 0.0, theme::CHASSIS);
        let (lo, hi) = (self.center - self.rate / 2.0, self.center + self.rate / 2.0);
        for b in bands::in_span(lo, hi) {
            let x0 = self.x_of(r, b.lo.max(lo)).max(r.left());
            let x1 = self.x_of(r, b.hi.min(hi)).min(r.right());
            if x1 - x0 < 1.0 {
                continue;
            }
            let cell = Rect::from_min_max(Pos2::new(x0, r.top() + 2.0), Pos2::new(x1, r.bottom() - 2.0));
            p.rect_filled(cell, 1.0, b.color);
            if x1 - x0 > 60.0 {
                p.text(
                    cell.center(),
                    Align2::CENTER_CENTER,
                    b.name,
                    FontId::new(9.0, FontFamily::Name(theme::LEGEND_FONT.into())),
                    Color32::from_rgb(0xE8, 0xEC, 0xF0),
                );
            }
        }
    }

    /// Everything the tracker in the graph is holding: aircraft from ADS-B,
    /// vessels and navigation marks from AIS.
    ///
    /// Read from the receiver rather than assembled here: the tracker is a
    /// node fed by the bus, so it sees every frame rather than the ones still
    /// in the on-screen packet list.
    fn map_view(&mut self, ui: &mut egui::Ui) {
        let now = std::time::Instant::now();
        // The pane runs to the window edge, and a table that starts there is
        // unreadable.
        let margin = egui::Frame::NONE.inner_margin(egui::Margin::symmetric(12, 8));
        self.tiles.poll(ui.ctx());
        let mut view = self.map;
        let mut place = None;
        let mut edit = self.station_edit.take();
        {
            let tiles = &mut self.tiles;
            let active: Vec<&crate::tracks::Track> = self.tracks.iter().collect();
            let home = self.location;
            let body = |ui: &mut egui::Ui| {
                place = Self::station_row(ui, home, &mut edit);
                ui.add_space(6.0);
                // Half the pane each, roughly: the map is the view worth
                // having and the table is what you read once something on it
                // is interesting.
                let h = (ui.available_height() * 0.55).clamp(160.0, 1200.0);
                let (v, dropped) = Self::map_pane(ui, tiles, &active, now, home, view, h);
                view = v;
                place = place.or(dropped);
                ui.add_space(10.0);
                Self::track_rows(ui, &active, now);
            };
            margin.show(ui, body);
        }
        self.map = view;
        self.station_edit = edit;
        if let Some((lat, lon)) = place {
            self.set_location(lat, lon);
            self.station_edit = None;
        }
    }

    /// The station position, shown and editable.
    ///
    /// Worth a control rather than only a command line flag: it is what makes
    /// a single position frame resolve instead of waiting for a matching
    /// pair, and it is the point the range rings are drawn around. Anything
    /// within a couple of hundred miles of the truth does the job.
    fn station_row(
        ui: &mut egui::Ui,
        home: Option<(f64, f64)>,
        edit: &mut Option<String>,
    ) -> Option<(f64, f64)> {
        let mut set = None;
        ui.horizontal(|ui| {
            ui.label(legend("station"));
            let text = edit.get_or_insert_with(|| match home {
                Some((lat, lon)) => format!("{lat:.4}, {lon:.4}"),
                None => String::new(),
            });
            let r = ui.add(
                egui::TextEdit::singleline(text)
                    .desired_width(150.0)
                    .hint_text("lat, lon")
                    .font(FontId::new(12.0, FontFamily::Name(theme::READOUT_FONT.into()))),
            );
            let typed = r.lost_focus() && ui.input(|i| i.key_pressed(egui::Key::Enter));
            if typed || ui.small_button("SET").clicked() {
                if let Ok(p) = crate::parse_location(text) {
                    set = Some(p);
                }
            }
            ui.add_space(8.0);
            ui.label(legend(if home.is_some() {
                "right-click the map to move it"
            } else {
                "type it, or right-click the map"
            }));
        });
        set
    }

    /// Where the tracks are, on OpenStreetMap tiles.
    ///
    /// The tiles come from our own fetcher rather than a map crate: slippy
    /// tiles are a URL template and a Mercator projection, and what a map
    /// widget adds on top is a way to draw things over them, which is the
    /// part this view has to write anyway.
    ///
    /// Returns the view to use next frame, since dragging and scrolling
    /// change it, and a position if the station was dropped somewhere.
    fn map_pane(
        ui: &mut egui::Ui,
        tiles: &mut crate::map::Tiles,
        active: &[&crate::tracks::Track],
        now: std::time::Instant,
        home: Option<(f64, f64)>,
        view: MapView,
        height: f32,
    ) -> (MapView, Option<(f64, f64)>) {
        let w = ui.available_width();
        let (rect, resp) = ui.allocate_exact_size(Vec2::new(w, height), Sense::click_and_drag());
        let p = ui.painter_at(rect);
        p.rect_filled(rect, 2.0, theme::WELL);
        let font = FontId::new(10.0, FontFamily::Name(theme::READOUT_FONT.into()));

        // Centre on the receiver, or on what it can hear, until someone drags
        // the map somewhere else. After that it stays where it was put: a map
        // that recentres itself is a map you cannot read.
        let mut view = view;
        if view.center.is_none() {
            view.center = home.or_else(|| mean_position(active));
        }
        let Some((clat, clon)) = view.center else {
            p.rect_stroke(rect, 2.0, Stroke::new(1.0, theme::ETCH), StrokeKind::Inside);
            p.text(rect.center(), Align2::CENTER_CENTER, "no positions yet", font, theme::LEGEND);
            return (view, None);
        };

        let mid = rect.center();
        let mut center = (clat, clon);
        let offset = |pos: Pos2| (f64::from(pos.x - mid.x), f64::from(pos.y - mid.y));

        if resp.dragged() {
            let d = resp.drag_delta();
            center = crate::map::screen_to_ll(
                center,
                view.zoom,
                (f64::from(-d.x), f64::from(-d.y)),
            );
        }
        if let (true, Some(pos)) = (resp.hovered(), resp.hover_pos()) {
            let d = ui.input(|i| i.smooth_scroll_delta.y);
            if d != 0.0 {
                let next = (view.zoom + f64::from(d) * 0.004).clamp(2.0, 19.0);
                center = crate::map::anchored_zoom(center, view.zoom, next, offset(pos));
                view.zoom = next;
            }
        }
        view.center = Some(center);

        let (z, scale) = (crate::map::level(view.zoom), crate::map::tile_scale(view.zoom));
        let (cx, cy) = crate::map::project(center.0, center.1, z);
        let to_screen = |lat: f64, lon: f64| {
            let (x, y) = crate::map::ll_to_screen(center, view.zoom, (lat, lon));
            Pos2::new(mid.x + x as f32, mid.y + y as f32)
        };

        let clip = p.with_clip_rect(rect);
        Self::draw_tiles(&clip, tiles, rect, mid, (cx, cy), z, scale);

        // Range rings are centred on the receiver, not on the view. They say
        // how far away something is from the antenna, which does not change
        // when the map is dragged.
        let m_px = crate::map::resolution(center.0, z) * crate::map::TILE_PX / scale;
        let nm_px = 1852.0 / m_px;
        if let Some((lat, lon)) = home {
            let at = to_screen(lat, lon);
            // Rings at a round distance that fits the window, rather than a
            // fraction of a zoom: 25 nm is 25 nm at every scale.
            let span = f64::from(rect.width().min(rect.height())) / 2.0 / nm_px;
            let step = [1.0, 2.0, 5.0, 10.0, 25.0, 50.0, 100.0, 200.0, 500.0]
                .into_iter()
                .find(|s| s * 2.0 >= span)
                .unwrap_or(1000.0);
            for k in 1..=3 {
                let r = (step * f64::from(k) * nm_px) as f32;
                clip.circle_stroke(at, r, Stroke::new(1.0, theme::READOUT.gamma_multiply(0.30)));
                clip.text(
                    Pos2::new(at.x + 4.0, at.y - r),
                    Align2::LEFT_CENTER,
                    format!("{:.0} nm", step * f64::from(k)),
                    font.clone(),
                    theme::READOUT.gamma_multiply(0.55),
                );
            }
            clip.circle_stroke(at, 5.0, Stroke::new(1.5, theme::READOUT));
            clip.circle_filled(at, 1.5, theme::READOUT);
        }

        // Airports are fixed things under the traffic: drawn behind it so the
        // cyan of what is moving stays the brightest thing on screen.
        let airports_shown = Self::draw_airports(&clip, rect, view.zoom, to_screen);

        for a in active {
            let Some((lat, lon)) = a.position else { continue };
            // Faded by age against its own kind's memory: a minute of silence
            // means an aircraft is gone and means nothing at all for a vessel,
            // so fading both on the same clock would grey out half the
            // shipping while it was still there.
            let stale = a.kind().forget().as_secs_f32();
            let fade = 1.0 - (a.age(now).as_secs_f32() / stale).clamp(0.0, 0.75);
            // Drawn in segments, brightening towards the track: a line of one
            // colour says nothing about which end of it is now, and over map
            // tiles a thin one is lost in the roads.
            if a.trail.len() > 1 {
                let pts: Vec<Pos2> = a.trail.iter().map(|(la, lo)| to_screen(*la, *lo)).collect();
                let n = pts.len() as f32;
                for (k, seg) in pts.windows(2).enumerate() {
                    let along = (k as f32 + 1.0) / n;
                    clip.line_segment(
                        [seg[0], seg[1]],
                        Stroke::new(
                            2.5,
                            theme::TRACE.gamma_multiply((0.25 + 0.65 * along) * fade),
                        ),
                    );
                }
            }
            let at = to_screen(lat, lon);
            let col = theme::TRACE.gamma_multiply(fade);
            // An unconfirmed position came from one ADS-B frame read against
            // the receiver, which is right for anything in ordinary range and
            // a whole zone out beyond it. Drawn hollow so it does not claim
            // more than it knows. Nothing else can be unconfirmed: an AIS
            // position is absolute.
            if a.confirmed {
                Self::track_mark(&clip, at, a.kind(), a.course_deg, col);
            } else {
                clip.circle_stroke(at, 3.5, Stroke::new(1.0, col.gamma_multiply(0.7)));
            }
            let label = a.label.clone().unwrap_or_else(|| a.id.text());
            Self::map_label(&clip, Pos2::new(at.x + 9.0, at.y - 5.0), &label, theme::VALUE, fade);
            // The second line is whatever that kind is measured by: an
            // aircraft by its altitude, a vessel by its speed. A station is
            // fixed and has neither.
            let under = match a.kind() {
                crate::tracks::Kind::Aircraft => a.altitude_ft().map(|ft| format!("{ft} ft")),
                crate::tracks::Kind::Vessel | crate::tracks::Kind::Vehicle => {
                    a.speed_kt.filter(|v| *v > 0.0).map(|kt| format!("{kt:.0} kt"))
                }
                crate::tracks::Kind::Station => None,
            };
            if let Some(t) = under {
                Self::map_label(&clip, Pos2::new(at.x + 9.0, at.y + 5.0), &t, theme::LEGEND, fade);
            }
        }

        // Hover over an airport to read its frequencies. The card is drawn on
        // the map painter after everything else, so it sits over the tiles and
        // the aircraft instead of vanishing behind them.
        if view.zoom >= crate::airports::SHOW_ZOOM && !resp.dragged() {
            if let Some(pos) = resp.hover_pos() {
                if let Some((at, a)) = Self::hovered_airport(&airports_shown, pos) {
                    Self::airport_card(&clip, rect, at, a);
                }
            }
        }

        let shown = active.iter().filter(|a| a.position.is_some()).count();
        let mut status = format!(
            "{shown} plotted    {:.1} nm/cm    z{:.1}    drag to pan, scroll to zoom",
            nm_px.recip() * 37.8,
            view.zoom
        );
        if !airports_shown.is_empty() {
            status.push_str(&format!("    {} airports", airports_shown.len()));
        }
        Self::map_label(
            &clip,
            Pos2::new(rect.left() + 8.0, rect.top() + 10.0),
            &status,
            theme::LEGEND,
            1.0,
        );
        // Required by the tile usage policy, and by the licence the map data
        // is under.
        Self::map_label(
            &clip,
            Pos2::new(rect.right() - 150.0, rect.bottom() - 10.0),
            "(c) OpenStreetMap contributors",
            theme::LEGEND,
            1.0,
        );

        // A map with no tiles under it still shows tracks, and would quietly
        // look like empty sky and empty sea. Say what failed instead.
        if let Some((err, n)) = tiles.error() {
            let short: String = err.chars().take(110).collect();
            clip.rect_filled(
                Rect::from_min_max(
                    Pos2::new(rect.left(), rect.bottom() - 30.0),
                    Pos2::new(rect.right(), rect.bottom()),
                ),
                0.0,
                theme::WELL,
            );
            Self::map_label(
                &clip,
                Pos2::new(rect.left() + 8.0, rect.bottom() - 20.0),
                &format!("{n} tile(s) failed, map is not showing terrain"),
                theme::FAULT,
                1.0,
            );
            Self::map_label(
                &clip,
                Pos2::new(rect.left() + 8.0, rect.bottom() - 8.0),
                &short,
                theme::LEGEND,
                1.0,
            );
        }
        p.rect_stroke(rect, 2.0, Stroke::new(1.0, theme::ETCH), StrokeKind::Inside);

        // Right-click puts the station where the pointer is. A receiver knows
        // where it is on a map long before it knows its coordinates.
        let mut place = None;
        if resp.secondary_clicked() {
            if let Some(pos) = resp.interact_pointer_pos() {
                place = Some(crate::map::screen_to_ll(center, view.zoom, offset(pos)));
            }
        }
        (view, place)
    }

    /// Every tile the view touches, asked for as it is drawn.
    fn draw_tiles(
        p: &egui::Painter,
        tiles: &mut crate::map::Tiles,
        rect: Rect,
        mid: Pos2,
        center: (f64, f64),
        z: u8,
        scale: f64,
    ) {
        let (cx, cy) = center;
        let half_w = f64::from(rect.width()) / 2.0 / scale;
        let half_h = f64::from(rect.height()) / 2.0 / scale;
        let n = 1i64 << z;
        let (x0, x1) = ((cx - half_w).floor() as i64, (cx + half_w).floor() as i64);
        let (y0, y1) = ((cy - half_h).floor() as i64, (cy + half_h).floor() as i64);
        for ty in y0..=y1 {
            // The world does not wrap north to south, so a tile above the
            // pole is nothing rather than a tile from the other end.
            if ty < 0 || ty >= n {
                continue;
            }
            for tx in x0..=x1 {
                // Longitude does wrap, so panning past the date line shows
                // the far side of the world rather than a hole.
                let wrapped = tx.rem_euclid(n);
                let id = crate::map::TileId { z, x: wrapped as u32, y: ty as u32 };
                let Some(tex) = tiles.get(id) else { continue };
                let min = Pos2::new(
                    mid.x + ((tx as f64 - cx) * scale) as f32,
                    mid.y + ((ty as f64 - cy) * scale) as f32,
                );
                let at = Rect::from_min_size(min, Vec2::splat(scale as f32));
                p.image(
                    tex.id(),
                    at,
                    Rect::from_min_max(Pos2::ZERO, Pos2::new(1.0, 1.0)),
                    // Held back so the tracks over it stay the brightest
                    // thing on screen, and so it sits in the panel's palette
                    // rather than glowing white in a dark interface.
                    Color32::from_gray(150),
                );
            }
        }
    }

    /// The backing box a map label would occupy, before it is drawn. Kept
    /// separate from [`Self::map_label`] so airport labels can refuse to draw
    /// where another one already is, instead of covering it.
    fn map_label_rect(p: &egui::Painter, at: Pos2, text: &str, col: Color32, fade: f32) -> Rect {
        let font = FontId::new(10.0, FontFamily::Name(theme::READOUT_FONT.into()));
        let g = p.layout_no_wrap(text.to_string(), font, col.gamma_multiply(fade));
        Rect::from_min_size(at - Vec2::new(2.0, g.size().y / 2.0), g.size() + Vec2::new(4.0, 0.0))
    }

    /// Text with a dark backing, since map tiles are busy and unbacked labels
    /// vanish over a town.
    fn map_label(p: &egui::Painter, at: Pos2, text: &str, col: Color32, fade: f32) -> Rect {
        let r = Self::map_label_rect(p, at, text, col, fade);
        p.rect_filled(r, 2.0, Color32::from_black_alpha((190.0 * fade) as u8));
        let font = FontId::new(10.0, FontFamily::Name(theme::READOUT_FONT.into()));
        let g = p.layout_no_wrap(text.to_string(), font, col.gamma_multiply(fade));
        p.galley(Pos2::new(at.x, at.y - g.size().y / 2.0), g, col);
        r
    }

    /// Airport markers and their ident labels, in the map's amber so a fixed
    /// facility is not mistaken for the cyan of something in the air.
    ///
    /// Returns the on-screen markers, so the caller can hit-test the pointer
    /// against them for the frequency tooltip. Airports appear only once the
    /// map is zoomed in past [`crate::airports::SHOW_ZOOM`]; at the default
    /// wide view a marker is a blob under the traffic, and the range rings
    /// already say where the interesting things are.
    fn draw_airports(
        clip: &egui::Painter,
        rect: Rect,
        zoom: f64,
        to_screen: impl Fn(f64, f64) -> Pos2,
    ) -> Vec<(Pos2, &'static crate::airports::Airport)> {
        if zoom < crate::airports::SHOW_ZOOM {
            return Vec::new();
        }
        // Cull to the window, plus room for a label hanging over an edge.
        let near = rect.expand(30.0);
        let mut shown: Vec<(&'static crate::airports::Airport, Pos2)> = crate::airports::all()
            .iter()
            .filter_map(|a| {
                let at = to_screen(a.lat, a.lon);
                near.contains(at).then_some((a, at))
            })
            .collect();
        for (a, at) in &shown {
            let (r, bright) = match a.kind {
                crate::airports::Kind::Large => (4.5, 1.0),
                crate::airports::Kind::Medium => (3.5, 0.85),
                crate::airports::Kind::Small => (2.6, 0.7),
            };
            let col = theme::READOUT.gamma_multiply(bright);
            clip.circle_filled(*at, r, col);
            clip.circle_stroke(*at, r + 1.0, Stroke::new(1.0, col.gamma_multiply(0.55)));
        }
        // Ident labels appear as the map zooms in, large airports first, and
        // drop where they would cover one already drawn: a city full of
        // strips must not become a wall of text. Larger first so a big field
        // wins its label against smaller neighbours.
        shown.sort_by_key(|(a, _)| match a.kind {
            crate::airports::Kind::Large => 0,
            crate::airports::Kind::Medium => 1,
            crate::airports::Kind::Small => 2,
        });
        let mut labels: Vec<Rect> = Vec::new();
        for (a, at) in &shown {
            let at_zoom = match a.kind {
                crate::airports::Kind::Large => 9.0,
                crate::airports::Kind::Medium => 10.0,
                crate::airports::Kind::Small => 11.0,
            };
            if zoom < at_zoom {
                continue;
            }
            let at = Pos2::new(at.x + 8.0, at.y - 5.0);
            let r = Self::map_label_rect(clip, at, &a.ident, theme::VALUE, 1.0);
            if labels.iter().any(|l| l.intersects(r.expand(3.0))) {
                continue;
            }
            labels.push(r);
            Self::map_label(clip, at, &a.ident, theme::VALUE, 1.0);
        }
        shown.into_iter().map(|(a, at)| (at, a)).collect()
    }

    /// The airport nearest the pointer within a marker's grabbing distance, if
    /// any, with where its marker sits so the card can be anchored to it. The
    /// card belongs on a hover, so the threshold is a small screen distance
    /// rather than a whole map.
    fn hovered_airport<'a>(
        shown: &[(Pos2, &'a crate::airports::Airport)],
        pos: Pos2,
    ) -> Option<(Pos2, &'a crate::airports::Airport)> {
        const PX: f32 = 12.0;
        let mut best: Option<(f32, Pos2, &'a crate::airports::Airport)> = None;
        for (at, a) in shown {
            let d = at.distance(pos);
            if d <= PX && best.is_none_or(|(b, _, _)| d < b) {
                best = Some((d, *at, a));
            }
        }
        best.map(|(_, at, a)| (at, a))
    }

    /// The frequency card shown when an airport is hovered: name, code,
    /// elevation and the air traffic frequencies, primary ones first.
    ///
    /// Drawn by hand rather than as an egui tooltip so it stays in the map's
    /// language and is clipped with the pane, and so it does not depend on the
    /// tooltip API changing under us.
    fn airport_card(p: &egui::Painter, rect: Rect, anchor: Pos2, a: &crate::airports::Airport) {
        // At most this many frequency rows before the card says how many it
        // is not showing. A field with thirty listed frequencies would
        // otherwise cover the map it is annotating.
        const MAX_ROWS: usize = 10;
        let (pad, sep, rule_gap) = (8.0, 4.0, 6.0);
        let font = |sz: f32| FontId::new(sz, FontFamily::Name(theme::READOUT_FONT.into()));

        // The name wraps rather than setting the card's width: "Charles de
        // Gaulle International Airport" is wider than anything else on the
        // card and would drag the whole thing across the map.
        let text_max = (f64::from(rect.width()) - 2.0 * f64::from(pad) - 8.0).clamp(80.0, 240.0) as f32;
        let name = p.layout(a.name.clone(), font(13.0), theme::VALUE, text_max);
        let class = match a.kind {
            crate::airports::Kind::Large => "LARGE",
            crate::airports::Kind::Medium => "MEDIUM",
            crate::airports::Kind::Small => "SMALL",
        };
        let mut meta = format!("{class} AIRPORT");
        if let Some(el) = a.elev_ft {
            meta.push_str(&format!("   {el} FT"));
        }
        let meta = p.layout_no_wrap(meta, font(9.0), theme::LEGEND);
        let ident = p.layout_no_wrap(a.ident.clone(), font(11.0), theme::READOUT);

        // Every row below the rule, built once and then both measured and
        // drawn from this list. A row that is drawn without being measured is
        // a row that hangs off the bottom of the card.
        let mut rows: Vec<(std::sync::Arc<egui::Galley>, Color32)> = Vec::new();
        if a.freqs.is_empty() {
            let g = p.layout_no_wrap(
                "no published frequencies".to_string(),
                font(10.0),
                theme::LEGEND,
            );
            rows.push((g, theme::LEGEND));
        } else {
            for f in a.freqs.iter().take(MAX_ROWS) {
                let label = if f.kind == crate::airports::FreqKind::Other {
                    f.desc.as_str()
                } else {
                    f.kind.label()
                };
                // The role is padded to a fixed width so the numbers line up
                // down the column, and truncated rather than let a long
                // description push the frequency off the card.
                let label: String = label.chars().take(16).collect();
                let g = p.layout_no_wrap(
                    format!("{label:<16}{}", crate::airports::fmt_mhz(f.mhz)),
                    font(11.0),
                    theme::VALUE,
                );
                rows.push((g, theme::VALUE));
            }
            if a.freqs.len() > MAX_ROWS {
                let g = p.layout_no_wrap(
                    format!("+{} more", a.freqs.len() - MAX_ROWS),
                    font(10.0),
                    theme::LEGEND,
                );
                rows.push((g, theme::LEGEND));
            }
        }

        let head = [
            (name, theme::VALUE),
            (meta, theme::LEGEND),
            (ident, theme::READOUT),
        ];
        let head_sizes: Vec<Vec2> = head.iter().map(|(g, _)| g.size()).collect();
        let row_sizes: Vec<Vec2> = rows.iter().map(|(g, _)| g.size()).collect();
        let l = card_layout(&head_sizes, &row_sizes, pad, sep, rule_gap);

        // Beside the marker, or to its left when that would run off the right
        // edge, and clamped so the card stays on the map. The upper bound is
        // held above the lower one because a card wider than the pane would
        // otherwise clamp with a reversed range.
        let mut at = anchor + Vec2::new(14.0, -8.0);
        if anchor.x + 14.0 + l.size.x > rect.right() - 4.0 {
            at.x = anchor.x - 14.0 - l.size.x;
        }
        let max_x = (rect.right() - 4.0 - l.size.x).max(rect.left() + 4.0);
        let max_y = (rect.bottom() - 4.0 - l.size.y).max(rect.top() + 4.0);
        at.x = at.x.clamp(rect.left() + 4.0, max_x);
        at.y = at.y.clamp(rect.top() + 4.0, max_y);
        let card = Rect::from_min_size(at, l.size);
        p.rect_filled(card, 3.0, theme::PANEL);
        p.rect_stroke(card, 3.0, Stroke::new(1.0, theme::ETCH), StrokeKind::Inside);

        // Drawn entirely from the measured layout, so a line cannot be placed
        // somewhere the card was never sized for.
        let x = card.left() + l.text_x;
        for ((g, col), y) in head.iter().chain(rows.iter()).zip(&l.ys) {
            p.galley(Pos2::new(x, card.top() + y), g.clone(), *col);
        }
        let rule_y = card.top() + l.rule_y;
        p.line_segment(
            [Pos2::new(x, rule_y), Pos2::new(card.right() - pad, rule_y)],
            Stroke::new(1.0, theme::ETCH),
        );
    }

    /// A mark pointing where the track is going, shaped by what it is.
    ///
    /// The shapes have to be tellable apart at a glance and at a few pixels,
    /// because a busy estuary puts aircraft and shipping on the same screen.
    /// An aircraft is a swept arrowhead, a vessel a longer hull with a bow,
    /// and a station a fixed diamond that does not point anywhere because it
    /// is not going anywhere.
    fn track_mark(
        p: &egui::Painter,
        at: Pos2,
        kind: crate::tracks::Kind,
        course_deg: Option<f64>,
        col: Color32,
    ) {
        use crate::tracks::Kind;
        if kind == Kind::Station {
            let d = 4.0;
            p.add(egui::Shape::convex_polygon(
                vec![
                    Pos2::new(at.x, at.y - d),
                    Pos2::new(at.x + d, at.y),
                    Pos2::new(at.x, at.y + d),
                    Pos2::new(at.x - d, at.y),
                ],
                Color32::TRANSPARENT,
                Stroke::new(1.5, col),
            ));
            return;
        }
        let Some(track) = course_deg else {
            p.circle_filled(at, 3.0, col);
            return;
        };
        let t = (track as f32).to_radians();
        let (s, c) = (t.sin(), t.cos());
        // Course is clockwise from north, and north is up, so a point ahead of
        // the track is (sin, -cos) in screen coordinates.
        let rot = |x: f32, y: f32| Pos2::new(at.x + x * c + y * s, at.y + x * s - y * c);
        let shape = match kind {
            Kind::Aircraft => {
                vec![rot(0.0, 6.0), rot(-4.0, -4.0), rot(0.0, -1.5), rot(4.0, -4.0)]
            }
            // Longer and narrower, with a squared stern: a hull rather than a
            // wing.
            Kind::Vessel => vec![
                rot(0.0, 7.0),
                rot(-2.5, 2.0),
                rot(-2.5, -5.0),
                rot(2.5, -5.0),
                rot(2.5, 2.0),
            ],
            // Short and blunt, which is neither of the other two at a glance.
            _ => vec![rot(0.0, 4.5), rot(-3.0, 1.0), rot(-3.0, -3.0), rot(3.0, -3.0), rot(3.0, 1.0)],
        };
        p.add(egui::Shape::convex_polygon(shape, col, Stroke::NONE));
    }

    fn track_rows(
        ui: &mut egui::Ui,
        active: &[&crate::tracks::Track],
        now: std::time::Instant,
    ) {
        use crate::tracks::Kind;
        let count = |k: Kind| active.iter().filter(|t| t.kind() == k).count();
        ui.horizontal(|ui| {
            ui.label(legend("tracks"));
            ui.label(value(active.len().to_string()).size(12.0));
            // Broken down by kind, because "14 tracks" on a coast says nothing
            // about whether the aircraft or the shipping is being heard.
            for (k, name) in
                [
                    (Kind::Aircraft, "aircraft"),
                    (Kind::Vessel, "vessels"),
                    (Kind::Vehicle, "vehicles"),
                    (Kind::Station, "stations"),
                ]
            {
                let n = count(k);
                if n > 0 {
                    ui.add_space(8.0);
                    ui.label(legend(name));
                    ui.label(value(n.to_string()).size(12.0));
                }
            }
            if active.is_empty() {
                ui.add_space(10.0);
                ui.label(legend(
                    "tune to 1090 for aircraft, 162 for shipping, 144.8 for APRS",
                ));
            }
        });
        ui.add_space(6.0);

        // One table for both, so the columns are what the two have in common
        // and the one column that differs is named for what it holds. An
        // altitude column full of dashes beside every vessel is worse than a
        // column that says "altitude / status".
        const COLS: [(&str, f32); 8] = [
            ("name", 100.0),
            ("id", 80.0),
            ("kind", 62.0),
            ("alt / status", 96.0),
            ("speed", 70.0),
            ("course", 60.0),
            ("position", 170.0),
            ("msgs", 56.0),
        ];
        // Wide enough that the last column is not clipped when the channel
        // strip is open, and scrolled sideways rather than squeezed when the
        // window is narrower than that.
        let width: f32 = COLS.iter().map(|(_, w)| w).sum::<f32>() + 60.0;
        egui::ScrollArea::horizontal().auto_shrink([false, false]).show(ui, |ui| {
        ui.set_min_width(width);
        let (rect, _) = ui.allocate_exact_size(Vec2::new(width, Self::ROW_H), Sense::hover());
        let p = ui.painter_at(rect);
        let mut x = rect.left();
        for (name, w) in COLS {
            Self::cell(&p, rect, x, w, name, theme::LEGEND);
            x += w;
        }
        Self::cell(&p, rect, x, rect.right() - x, "age", theme::LEGEND);
        p.line_segment(
            [Pos2::new(rect.left(), rect.bottom()), Pos2::new(rect.right(), rect.bottom())],
            Stroke::new(1.0, theme::ETCH),
        );

        egui::ScrollArea::vertical().auto_shrink([false, false]).show(ui, |ui| {
            for (n, a) in active.iter().enumerate() {
                let (rect, _) =
                    ui.allocate_exact_size(Vec2::new(width, Self::ROW_H), Sense::hover());
                if !ui.is_rect_visible(rect) {
                    continue;
                }
                let p = ui.painter_at(rect);
                if n % 2 == 1 {
                    p.rect_filled(rect, 0.0, Color32::from_rgb(0x24, 0x27, 0x2D));
                }
                let dash = "-".to_string();
                // The one column that differs by kind: an aircraft is placed
                // vertically by its altitude, a vessel by what it is doing.
                let (state, state_col) = match &a.detail {
                    crate::tracks::Detail::Aircraft { altitude_ft, vertical_rate_fpm } => (
                        altitude_ft.map(|v| format!("{v} ft")).unwrap_or_else(|| dash.clone()),
                        // Climb and descent are worth telling apart at a
                        // glance; level flight is not worth colouring at all.
                        match vertical_rate_fpm {
                            Some(v) if *v > 128 => CRC_OK,
                            Some(v) if *v < -128 => theme::READOUT,
                            _ => theme::VALUE,
                        },
                    ),
                    crate::tracks::Detail::Vessel { nav_status, ship_type, .. } => (
                        nav_status
                            .or(*ship_type)
                            .map(str::to_string)
                            .unwrap_or_else(|| dash.clone()),
                        theme::LEGEND,
                    ),
                    crate::tracks::Detail::Station { aid } => (
                        if *aid { "navigation mark".into() } else { "shore station".into() },
                        theme::LEGEND,
                    ),
                    // An APRS station says what it is in a comment more often
                    // than in any field, so that is what the column shows.
                    crate::tracks::Detail::Aprs { comment, altitude_ft, .. } => (
                        comment
                            .clone()
                            .or_else(|| altitude_ft.map(|v| format!("{v} ft")))
                            .unwrap_or_else(|| dash.clone()),
                        theme::LEGEND,
                    ),
                };
                let kind = match a.kind() {
                    Kind::Aircraft => "air",
                    Kind::Vessel => "sea",
                    Kind::Vehicle => "land",
                    Kind::Station => "fixed",
                };
                let text = [
                    (a.label.clone().unwrap_or_else(|| dash.clone()), theme::TRACE),
                    (a.id.text(), theme::VALUE),
                    (kind.to_string(), theme::LEGEND),
                    (state, state_col),
                    (
                        a.speed_kt.map(|v| format!("{v:.0} kt")).unwrap_or_else(|| dash.clone()),
                        theme::VALUE,
                    ),
                    (
                        a.course_deg.map(|v| format!("{v:.0}")).unwrap_or_else(|| dash.clone()),
                        theme::LEGEND,
                    ),
                    (
                        a.position
                            .map(|(lat, lon)| format!("{lat:.4}, {lon:.4}"))
                            .unwrap_or_else(|| dash.clone()),
                        theme::TRACE,
                    ),
                    (a.messages.to_string(), theme::LEGEND),
                ];
                let mut x = rect.left();
                for ((t, c), (_, w)) in text.iter().zip(COLS) {
                    Self::cell(&p, rect, x, w, t, *c);
                    x += w;
                }
                let age = a.age(now).as_secs();
                Self::cell(&p, rect, x, rect.right() - x, &format!("{age}s"), theme::LEGEND);
            }
        });
        });
    }

    /// The packet log: everything decoded anywhere in the span.
    fn decode_log(&mut self, ui: &mut egui::Ui) {
        if !self.log_open {
            return;
        }
        Panel::bottom("decodes")
            .default_size(230.0)
            // Drag the top edge to resize. A band that is busy wants a tall
            // list; one that is quiet wants the waterfall back.
            .resizable(true)
            .min_size(64.0)
            .max_size(720.0)
            .show_separator_line(true)
            .frame(
                egui::Frame::NONE
                    .fill(theme::PANEL)
                    .inner_margin(egui::Margin::symmetric(12, 8)),
            )
            .show(ui, |ui| {
                self.log_header(ui);
                ui.add_space(4.0);
                let selected = self
                    .selected
                    .and_then(|id| self.decodes.iter().find(|l| l.id == id))
                    .map(|l| l.rec.clone());
                let dump_h = if selected.is_some() { 116.0 } else { 0.0 };
                let list_h = (ui.available_height() - dump_h).max(24.0);
                // Two nested scroll areas so the headings stay above the rows
                // vertically but travel with them sideways, which is the only
                // arrangement where a narrow window can still reach the last
                // column and the headings never leave the top.
                // Both areas are given an explicit height. Without it the
                // content asks for as much room as it has rows, the panel
                // grows to match, and the headings are pushed off the top of
                // the window they are supposed to be pinned to.
                egui::ScrollArea::horizontal()
                    .auto_shrink([false, false])
                    .max_height(list_h)
                    .show(ui, |ui| {
                    let w = ui.available_width().max(Self::table_width());
                    ui.set_min_width(w);
                    if !self.decodes.is_empty() {
                        self.log_header_row(ui, w);
                    }
                    egui::ScrollArea::vertical()
                        .auto_shrink([false, false])
                        .max_height((list_h - Self::ROW_H).max(16.0))
                        .stick_to_bottom(true)
                        .id_salt("packet_rows")
                        .show(ui, |ui| self.log_rows(ui, w));
                });
                if let Some(rec) = selected {
                    ui.separator();
                    packet_detail(ui, &rec);
                }
            });
    }

    fn log_header(&mut self, ui: &mut egui::Ui) {
        ui.horizontal(|ui| {
            // The switch that produces everything below it. Decoding the whole
            // span at once is the expensive thing this app does, so it stays a
            // one-click switch rather than a line in a settings modal, and it
            // sits beside the legend that reports what it did.
            // Off while the graph is the operator's: the switch rebuilds the
            // whole front end, which is the one thing manual mode promises
            // will not happen behind your back.
            let auto = !self.chain_edit.manual;
            if crate::icons::icon_button(
                ui,
                crate::icons::Icon::Decode,
                crate::i18n::t(if auto { "ui.decode_all" } else { "ui.manual_locked" }),
                auto,
                self.decode_on,
            )
            .clicked()
            {
                self.decode_on = !self.decode_on;
                self.send(Cmd::Decode(self.decode_on));
            }
            ui.add_space(6.0);
            // No row count here. The list keeps the last 500 and drops the
            // rest, so the number stops meaning anything the moment a band
            // gets busy, which is exactly when it would be looked at. The
            // frame total below is a real total and is worth printing.
            if let Some(r) = &self.radio {
                use std::sync::atomic::Ordering;
                let narrow = r.status.scan_channels.load(Ordering::Relaxed);
                let wide = r.status.scan_channels_wide.load(Ordering::Relaxed);
                let total = r.status.decoded.load(Ordering::Relaxed);
                let aircraft = r.status.aircraft.load(Ordering::Relaxed);
                ui.add_space(10.0);
                // Several front ends can run on one span now, so this names
                // all of them rather than the first that happens to be on.
                let mut running: Vec<String> = Vec::new();
                if r.status.modes_on.load(Ordering::Relaxed) {
                    running.push("mode s".into());
                }
                if r.status.ais_on.load(Ordering::Relaxed) {
                    running.push("ais".into());
                }
                if r.status.aprs_on.load(Ordering::Relaxed) {
                    running.push("aprs".into());
                }
                if r.status.pocsag_on.load(Ordering::Relaxed) {
                    running.push("pocsag".into());
                }
                if narrow > 0 || wide > 0 {
                    running.push(format!("{narrow} ook + {wide} fsk channels"));
                }
                let tracking = r.status.modes_on.load(Ordering::Relaxed)
                    || r.status.ais_on.load(Ordering::Relaxed)
                    || r.status.aprs_on.load(Ordering::Relaxed);
                ui.label(legend(&if running.is_empty() {
                    "decoding off".to_string()
                } else if tracking {
                    format!("{}, {aircraft} tracks, {total} frames", running.join(" + "))
                } else {
                    format!("{}, {total} frames", running.join(" + "))
                }));
            }
            let logged = self
                .radio
                .as_ref()
                .map(|r| r.status.logged.load(std::sync::atomic::Ordering::Relaxed))
                .unwrap_or(0);
            if logged > 0 {
                ui.add_space(10.0);
                ui.label(legend(&format!("{logged} saved")))
                    .on_hover_text(match &self.packet_log {
                        Some(d) => format!("appended to {}", d.display()),
                        None => "appended to the packet log".into(),
                    });
            }
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                if ui.button("CLEAR").clicked() {
                    self.decodes.clear();
                    self.selected = None;
                }
                if ui.button("SETTINGS").clicked() {
                    self.open = Some(Settings::PacketLog);
                }

                // Which front end runs on which frequency. Named rather than
                // drawn: it sits in a row of named buttons with room for a
                // word, and no glyph says "the table deciding what decodes
                // where" without being learned first.
                if ui
                    .add_enabled(auto, egui::Button::new("SCANNERS"))
                    .on_disabled_hover_text(crate::i18n::t("ui.manual_locked"))
                    .clicked()
                {
                    self.open = Some(Settings::Scanners);
                }
            });
        });
    }

    /// Column headings and their widths in pixels. Fixed rather than sized to
    /// the content: a table whose columns resize as packets arrive is a table
    /// that moves under the pointer, and the last column absorbs the slack.
    const COLS: [(&'static str, f32); 8] = [
        ("no", 40.0),
        ("time", 64.0),
        ("frequency", 96.0),
        ("mod", 34.0),
        ("rssi", 48.0),
        ("snr", 44.0),
        ("protocol", 140.0),
        ("len", 38.0),
    ];
    const ROW_H: f32 = 16.0;

    /// Width the table needs before the info column starts being squeezed.
    fn table_width() -> f32 {
        Self::COLS.iter().map(|(_, w)| w).sum::<f32>() + 340.0
    }

    /// The heading strip, above the rows and outside their vertical scroll, so
    /// it cannot scroll away from what it labels.
    fn log_header_row(&self, ui: &mut egui::Ui, w: f32) {
        let (rect, _) = ui.allocate_exact_size(Vec2::new(w, Self::ROW_H), Sense::hover());
        let p = ui.painter_at(rect);
        let mut x = rect.left();
        for (name, cw) in Self::COLS {
            Self::cell(&p, rect, x, cw, name, theme::LEGEND);
            x += cw;
        }
        Self::cell(&p, rect, x, rect.right() - x, "info", theme::LEGEND);
        p.line_segment(
            [Pos2::new(rect.left(), rect.bottom()), Pos2::new(rect.right(), rect.bottom())],
            Stroke::new(1.0, theme::ETCH),
        );
    }

    /// One cell of text, clipped to its column so a long field cannot push the
    /// ones after it sideways.
    fn cell(p: &egui::Painter, row: Rect, x: f32, w: f32, text: &str, col: Color32) {
        let r = Rect::from_min_max(Pos2::new(x, row.top()), Pos2::new(x + w - 6.0, row.bottom()));
        p.with_clip_rect(r.intersect(p.clip_rect())).text(
            Pos2::new(r.left(), r.center().y),
            Align2::LEFT_CENTER,
            text,
            FontId::new(11.0, FontFamily::Name(theme::READOUT_FONT.into())),
            col,
        );
    }

    fn log_rows(&mut self, ui: &mut egui::Ui, width: f32) {
        if self.decodes.is_empty() {
            // What is actually running here, rather than a claim about
            // sweeping the span that has not been true since the front end
            // became a table lookup.
            let running = self.scanners.active(self.center, self.rate);
            let waiting = match (self.decode_on, running.as_slice()) {
                (false, _) => "decoding is off".to_string(),
                (true, []) => {
                    "no scanner covers this span: press SCAN to add one".to_string()
                }
                (true, blocks) => {
                    let names: Vec<&str> = blocks.iter().map(|s| s.name.as_str()).collect();
                    format!("{} running, nothing heard yet", names.join(", "))
                }
            };
            ui.label(legend(&waiting));
            return;
        }
        let t0 = self.decodes.first().map(|l| l.rec.at);
        let mut clicked = None;

        for (n, log) in self.decodes.iter().enumerate() {
            let rec = &log.rec;
            if !self.show_unknown && !rec.is_known() {
                continue;
            }
            // Every row is the same height and every column the same width, so
            // nothing reflows as packets arrive or the pointer moves over
            // them. The whole row is one hit target, painted rather than built
            // from widgets, which is also what keeps a five hundred row list
            // cheap to draw.
            let (rect, resp) =
                ui.allocate_exact_size(Vec2::new(width, Self::ROW_H), Sense::click());
            if !ui.is_rect_visible(rect) {
                continue;
            }
            let on = self.selected == Some(log.id);
            let p = ui.painter_at(rect);
            if on {
                p.rect_filled(rect, 0.0, theme::ETCH);
            } else if resp.hovered() {
                p.rect_filled(rect, 0.0, Color32::from_rgb(0x2A, 0x2E, 0x35));
            } else if n % 2 == 1 {
                p.rect_filled(rect, 0.0, Color32::from_rgb(0x24, 0x27, 0x2D));
            }
            if resp.clicked() {
                clicked = Some(log.id);
            }
            if resp.hovered() {
                ui.ctx().set_cursor_icon(egui::CursorIcon::PointingHand);
            }

            let col = row_color(rec);
            // Seconds since the first packet in the list, the way a capture is
            // timed rather than a wall clock, so two transmissions can be
            // compared without arithmetic.
            let secs = t0
                .map(|t0| rec.at.saturating_duration_since(t0).as_secs_f64())
                .unwrap_or(0.0);
            let text = [
                (format!("{:>4}", log.id), col),
                (format!("{secs:8.3}"), theme::LEGEND),
                (fmt_hz(rec.freq), theme::TRACE),
                (rec.modulation.to_string(), theme::LEGEND),
                (fmt_db(rec.rssi_dbfs), level_color(rec.rssi_dbfs)),
                (fmt_db(rec.snr_db), theme::LEGEND),
                (rec.model.clone(), col),
                (format!("{:>4}", rec.bytes.len()), theme::LEGEND),
            ];
            let mut x = rect.left();
            for ((t, c), (_, cw)) in text.iter().zip(Self::COLS) {
                Self::cell(&p, rect, x, cw, t, *c);
                x += cw;
            }
            Self::cell(&p, rect, x, rect.right() - x, &rec.detail, theme::VALUE);
        }

        if let Some(id) = clicked {
            // Clicking the selected packet again closes the dump.
            self.selected = (self.selected != Some(id)).then_some(id);
        }
    }

    fn markers(&self, p: &egui::Painter, full: &Rect) {
        let (lo, hi) = (self.center - self.rate / 2.0, self.center + self.rate / 2.0);
        for (i, ch) in self.channels.iter().enumerate() {
            if ch.freq < lo || ch.freq > hi {
                continue;
            }
            let x = self.x_of(full, ch.freq);
            let active = self.listening == Some(i);
            let col = if active { theme::READOUT } else { Color32::from_rgb(0x6E, 0x7A, 0x88) };

            // Show what the demodulator actually takes in, not just where it
            // is centred: an NFM channel and a WFM channel at the same spot
            // are wildly different slices of spectrum.
            let half = ch.demod.bandwidth() / 2.0;
            let (bx0, bx1) = (self.x_of(full, ch.freq - half), self.x_of(full, ch.freq + half));
            if bx1 - bx0 >= 1.0 {
                let band = Rect::from_min_max(
                    Pos2::new(bx0.max(full.left()), full.top()),
                    Pos2::new(bx1.min(full.right()), full.bottom()),
                );
                p.rect_filled(
                    band,
                    0.0,
                    Color32::from_rgba_unmultiplied(col.r(), col.g(), col.b(), if active { 34 } else { 18 }),
                );
                for ex in [bx0, bx1] {
                    if full.x_range().contains(ex) {
                        p.line_segment(
                            [Pos2::new(ex, full.top()), Pos2::new(ex, full.bottom())],
                            Stroke::new(1.0, Color32::from_rgba_unmultiplied(col.r(), col.g(), col.b(), 120)),
                        );
                    }
                }
            }
            p.line_segment(
                [Pos2::new(x, full.top()), Pos2::new(x, full.bottom())],
                Stroke::new(if active { 1.5 } else { 1.0 }, col),
            );
            // Flag the label off the line so it never sits on the trace.
            let t = p.layout_no_wrap(
                ch.label.clone(),
                FontId::new(10.0, FontFamily::Name(theme::LEGEND_FONT.into())),
                Color32::BLACK,
            );
            let flag = Rect::from_min_size(
                Pos2::new(x + 1.0, full.top() + 2.0),
                Vec2::new(t.size().x + 8.0, t.size().y + 4.0),
            );
            p.rect_filled(flag, 1.0, col);
            p.galley(Pos2::new(flag.left() + 4.0, flag.top() + 2.0), t, Color32::BLACK);
        }
    }

    fn cursor(&self, p: &egui::Painter, full: &Rect, resp: &egui::Response, shift: bool) {
        let Some(pos) = resp.hover_pos() else { return };
        let raw = self.hz_at(full, pos.x);
        let raster = bands::raster_at(raw);
        // With shift held the readout shows where a channel would actually
        // land, not where the pointer is, so the snap is visible before
        // committing to it.
        let hz = if shift { bands::snap(raw) } else { raw };
        p.line_segment(
            [Pos2::new(pos.x, full.top()), Pos2::new(pos.x, full.bottom())],
            Stroke::new(1.0, Color32::from_rgb(0x55, 0x5E, 0x69)),
        );
        if shift {
            // Mark the snapped frequency when it differs from the pointer.
            let sx = self.x_of(full, hz);
            if (sx - pos.x).abs() > 1.0 && full.x_range().contains(sx) {
                p.line_segment(
                    [Pos2::new(sx, full.top()), Pos2::new(sx, full.bottom())],
                    Stroke::new(1.0, theme::READOUT),
                );
            }
        }
        let text = match (shift, raster) {
            (true, Some(r)) => {
                format!("{} {} snap {}", fmt_hz(hz), bands::name_at(hz), fmt_hz(r.step))
            }
            (true, None) => format!("{} {} no channel plan", fmt_hz(hz), bands::name_at(hz)),
            _ => match raster {
                // Advertise the gesture only where it would do something.
                Some(_) => format!("{} {} shift to snap", fmt_hz(hz), bands::name_at(hz)),
                None => format!("{} {}", fmt_hz(hz), bands::name_at(hz)),
            },
        };
        let g = p.layout_no_wrap(
            text,
            FontId::new(11.0, FontFamily::Name(theme::READOUT_FONT.into())),
            theme::VALUE,
        );
        let left = (pos.x + 8.0).min(full.right() - g.size().x - 10.0);
        let box_r = Rect::from_min_size(
            Pos2::new(left - 5.0, full.top() + 5.0),
            g.size() + Vec2::new(10.0, 6.0),
        );
        p.rect(box_r, 2.0, theme::WELL, Stroke::new(1.0, theme::ETCH), StrokeKind::Inside);
        p.galley(Pos2::new(left, full.top() + 8.0), g, theme::VALUE);
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
fn packet_detail(ui: &mut egui::Ui, rec: &DecodeRecord) {
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
        App {
            center: 100_000_000.0,
            rate: 2_000_000.0,
            // The waterfall holds history from where the radio actually is,
            // which after a settled tune is the same place.
            wf_center: 100_000_000.0,
            db_center: 100_000_000.0,
            ..Default::default()
        }
    }

    fn channel(app: &mut App, offset: f64, on: bool, volume: f32) {
        let freq = app.center + offset;
        let id = app.next_id as u64;
        app.next_id += 1;
        app.channels.push(Channel {
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
        a.channels.remove(0);
        assert_eq!(a.channel_specs()[0].id, second);
    }

    #[test]
    fn muting_everything_leaves_the_channels_running() {
        // Mute is a level, not a teardown: unmuting should not have to wait
        // for chains to be rebuilt and AGCs to settle again.
        let mut a = app();
        channel(&mut a, 100_000.0, true, 1.0);
        for c in &mut a.channels {
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
        }
    }

    #[test]
    fn the_scope_split_is_adjustable_and_bounded() {
        let mut a = app();
        assert_eq!(a.plot_frac, DEFAULT_PLOT_FRAC);
        // Dragging past either end clamps rather than collapsing a pane: a
        // two pixel waterfall is not a smaller waterfall, it is a broken one.
        for want in [0.0f32, 1.0, 0.6] {
            a.plot_frac = want.clamp(*PLOT_FRAC_RANGE.start(), *PLOT_FRAC_RANGE.end());
            assert!(PLOT_FRAC_RANGE.contains(&a.plot_frac), "{want} left {}", a.plot_frac);
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
        assert_eq!(a.decodes[0].id, 1);
        assert_eq!(a.decodes[1].id, 2);
    }

    #[test]
    fn hiding_unknowns_does_not_discard_them() {
        // The filter is a view, not a policy: turning it back on must show the
        // bursts that arrived while it was off.
        let mut a = app();
        let mut unknown = record(a.center, None);
        unknown.model = "unknown".into();
        a.log_decodes(vec![unknown, record(a.center, Some(true))]);
        a.show_unknown = false;
        assert_eq!(a.decodes.len(), 2, "hiding must not drop anything");
        assert_eq!(a.decodes.iter().filter(|l| !l.rec.is_known()).count(), 1);
    }

    #[test]
    fn the_packet_log_is_bounded() {
        let mut a = app();
        for i in 0..(DECODE_LOG_MAX + 120) {
            a.log_decodes(vec![record(100_000_000.0 + i as f64, Some(true))]);
        }
        assert_eq!(a.decodes.len(), DECODE_LOG_MAX);
        // The oldest are the ones dropped, so the newest packet is still there.
        let newest = 100_000_000.0 + (DECODE_LOG_MAX + 119) as f64;
        assert_eq!(a.decodes.last().unwrap().rec.freq, newest);
        // Numbers keep counting past what the list holds, so a row keeps the
        // number it was given.
        assert_eq!(a.decodes.last().unwrap().id, (DECODE_LOG_MAX + 120) as u64);
    }

    #[test]
    fn frequency_mapping_round_trips() {
        let a = app();
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
            a.channels.push(Channel {
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
        let a = with_channels(&[95_000_000.0, 95_009_000.0]);
        let x = a.x_of(&rect, 95_009_000.0);
        assert_eq!(a.channel_at(&rect, x), Some(1));
        let x = a.x_of(&rect, 95_000_000.0);
        assert_eq!(a.channel_at(&rect, x), Some(0));
    }

    #[test]
    fn empty_space_grabs_nothing() {
        let rect = Rect::from_min_size(Pos2::ZERO, Vec2::new(1000.0, 400.0));
        let a = with_channels(&[95_000_000.0]);
        assert_eq!(a.channel_at(&rect, a.x_of(&rect, 94_500_000.0)), None);
    }

    #[test]
    fn the_trace_covers_the_whole_pane_when_nothing_is_pending() {
        let rect = Rect::from_min_size(Pos2::ZERO, Vec2::new(1000.0, 400.0));
        let mut a = app();
        a.center = 95_000_000.0;
        a.db_center = 95_000_000.0;
        a.rate = 2_400_000.0;
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
        a.db_center = 95_000_000.0;
        // View dragged a quarter span right, data not yet caught up.
        a.center = 95_600_000.0;
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
        a.db_center = 95_000_000.0;
        a.center = 94_400_000.0;
        assert_eq!(a.column_bins(&rect, 0, 1000, 2048), None);
        assert_eq!(a.column_bins(&rect, 999, 1000, 2048).map(|x| x.0), Some(1533));
    }

    #[test]
    fn the_edges_are_the_ends_of_the_span() {
        let a = app();
        let r = rect();
        assert!((a.hz_at(&r, r.left()) - 99_000_000.0).abs() < 1.0);
        assert!((a.hz_at(&r, r.right()) - 101_000_000.0).abs() < 1.0);
    }

    #[test]
    fn clicks_outside_the_pane_clamp_to_the_span() {
        let a = app();
        let r = rect();
        assert!((a.hz_at(&r, -500.0) - 99_000_000.0).abs() < 1.0);
        assert!((a.hz_at(&r, 5000.0) - 101_000_000.0).abs() < 1.0);
    }

    #[test]
    fn new_channels_take_the_mode_of_their_band() {
        let mut a = app();
        a.add_channel(95.8e6);
        a.add_channel(124.0e6);
        assert_eq!(a.channels[0].demod, Demod::Wfm);
        assert_eq!(a.channels[1].demod, Demod::Am);
    }

    #[test]
    fn auto_scale_ignores_a_single_strong_carrier() {
        let mut a = app();
        a.floor = -90.0;
        a.ceil = -20.0;
        let mut db = vec![-95.0f32; 1024];
        db[500] = 0.0;
        for _ in 0..200 {
            a.rescale(&db);
        }
        assert!(a.floor < -95.0, "floor tracked the carrier: {}", a.floor);
        assert!(a.floor > -110.0, "floor ran away: {}", a.floor);
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
