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
mod control_pane;
mod dashboard_pane;
mod devices_pane;
mod head;
mod keys_pane;
mod links_pane;
mod map_pane;
mod mapview;
mod messages_pane;
mod packets;
mod sats_pane;
mod scope;
mod scope_settings;
mod settings;
mod settings_rows;
mod state;
mod strip;
mod transcript_pane;
mod video_pane;
pub(crate) mod widgets;

use crate::bands;
use crate::dial::Dial;
use crate::radio::{
    ChanMode, ChannelSpec, ChannelState, Cmd, DecodeRecord, Demod, Frame, Radio, StationInfo,
};
use crate::theme::{self, legend, value};
use burst::*;
use common::{GainMode, Hz, Sps};
use egui::containers::{CentralPanel, Panel};
use egui::{
    Align2, Color32, ColorImage, FontFamily, FontId, Pos2, Rect, Sense, Stroke, StrokeKind,
    TextureOptions, Vec2,
};
use settings::RemoteEdit;
use settings_rows::{mhz_field, ScannerRow};
use state::{Channel, Logged};
use widgets::{
    bin_hint, check_help, cog, cog_rect, help, hint, legend_help, modal_title, reading, row,
    row_help, Fader, Squelch, Vu,
};

pub struct App {
    /// What each view remembers. A pane is handed its own and nothing else,
    /// which is what stops one view reaching into another's business.
    scope: state::ScopeState,
    chain: state::ChainState,
    log: state::LogState,
    survey: state::SurveyState,
    map: map_pane::MapState,
    sats: state::SatsState,
    calls: state::CallsState,
    transcript: state::TranscriptState,
    messages: state::MessagesState,
    links: state::LinksState,
    control: state::ControlState,
    video: video_pane::VideoState,
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
    /// When `err` was set, so it can fade rather than stay until the next
    /// one replaces it.
    err_at: Option<std::time::Instant>,

    center: f64,
    rate: f64,

    dial: Dial,
    open: Option<Settings>,
    devices: Vec<crate::devices::Entry>,
    device: Option<crate::devices::Entry>,
    /// Where the connected tuner reaches and whether it can be moved, from
    /// the radio thread's reading of the device.
    reach: (f64, f64),
    tunable: bool,
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
    /// Start the radio on the first frame, rather than waiting for a click.
    autostart: bool,
    /// Remove the direct-conversion centre spur. On by default: it is an
    /// artefact of the receiver, not something being received.
    dc_block: bool,
    view: View,
    /// What was open before it. A look at the map and back is then one key,
    /// which is the thing an operator does most often with these views.
    prev_view: View,
    /// Whether the dashboard is one of the views. Off takes its tab away and
    /// opens the receiver on the spectrum, for an operator who knows what the
    /// thing does and wants the band instead.
    dashboard: bool,
    /// How much each view held when it was last looked at, by
    /// [`View::slot`]. A tab's dot is on when its view has more than this.
    view_seen: [u64; View::COUNT],
    /// Video transmissions that have ended, and whether one is running.
    /// Counted because the video pane has no list to take a length of.
    video_seen: u64,
    video_live_was: bool,
    /// Decoding every channel is on by default and can be turned off; it is
    /// the most expensive thing the app does.
    decode_on: bool,
    /// Whether the raw span capture is wanted. Held rather than sent once:
    /// choosing a device starts a new radio thread with a new graph, and a
    /// capture that quietly stopped there would be worse than none.
    capture: bool,
    /// Where the receiver is, when it has been told.
    location: Option<(f64, f64)>,
    /// How far out that position may be, in metres, when a fix said. `None`
    /// for a position typed in or taken from the country, which is a claim
    /// with no error bar rather than a perfect one.
    accuracy_m: Option<f64>,
    /// When the radio last delivered a spectrum, for noticing that it has
    /// stopped.
    last_frame: Option<std::time::Instant>,
    /// ISO country code, or empty when nothing has chosen one.
    country: String,
    /// OpenCelliD download token, as typed in the datasets pane.
    opencellid_token: String,
    /// The Space-Track login, as typed in the same pane. Its catalogue query
    /// is answered only while logged in.
    spacetrack_identity: String,
    spacetrack_password: String,
    /// Sound devices by name, empty for the system default. The speaker the
    /// mix comes out of, and the microphone a keyed channel transmits from.
    audio_out: String,
    audio_in: String,
    /// What the radio is set to, as the operator set it. The one record every
    /// route to a radio setting writes and reads; see
    /// [`crate::session::RadioSettings`].
    radio_settings: crate::session::RadioSettings,
    /// Reference correction by device label, as saved. The live figure is the
    /// one in `radio_settings`; this is where the radios not in use keep
    /// theirs, because a correction is a property of one crystal.
    ppm_by_device: std::collections::BTreeMap<String, f64>,
    /// Whether the radio has a freshly opened device that has not yet been
    /// given the settings. Set on connect and on reset, cleared once the
    /// driver has reported its controls and the settings have gone to it.
    radio_dirty: bool,
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
    /// The memory bank, and the group the next save goes into.
    memory: crate::memory::Memory,
    memory_group: String,
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
    /// The memory bank: saved channels, in groups.
    Memory,
    /// The dataset cache: what is held on disc, how old it is, and refresh.
    Data,
    /// Everything about where this receiver is rather than what it is doing:
    /// language, country, band plan, station position.
    App,
}

const FFTS: [usize; 6] = [512, 1024, 2048, 4096, 8192, 16384];
/// Spectrum refresh rates in frames per second.
const REFRESH: [(&str, f32); 4] = [("10", 10.0), ("20", 20.0), ("30", 30.0), ("60", 60.0)];
/// Waterfall scroll rates in rows per second.
const SPEEDS: [(&str, f32); 5] =
    [("5", 5.0), ("10", 10.0), ("20", 20.0), ("40", 40.0), ("80", 80.0)];

/// What the main pane shows.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum View {
    /// What the receiver can do, and what it is doing.
    Dashboard,
    Spectrum,
    Chain,
    Map,
    Calls,
    /// What was said, as the model on the audio bus read it.
    Transcript,
    Messages,
    Links,
    Devices,
    Satellites,
    Video,
    Keys,
    /// Where the sticks are, on every model control link in earshot.
    Control,
}

impl View {
    fn label(self) -> &'static str {
        match self {
            View::Dashboard => "Dashboard",
            View::Spectrum => "Spectrum",
            View::Chain => "Signal chain",
            View::Map => "Map",
            View::Calls => "Calls",
            View::Transcript => "Transcript",
            View::Messages => "Messages",
            View::Links => "Data links",
            View::Devices => "Devices",
            View::Satellites => "Satellites",
            View::Video => "Video",
            View::Keys => "Keys",
            View::Control => "Control",
        }
    }

    /// What the tab draws. One glyph per view, no two alike at 22 points.
    fn icon(self) -> crate::icons::Icon {
        use crate::icons::Icon;
        match self {
            View::Dashboard => Icon::Dashboard,
            View::Spectrum => Icon::Spectrum,
            View::Chain => Icon::Chain,
            View::Calls => Icon::Calls,
            View::Transcript => Icon::Transcript,
            View::Messages => Icon::Messages,
            View::Video => Icon::Video,
            View::Map => Icon::Map,
            View::Links => Icon::Links,
            View::Devices => Icon::Devices,
            View::Satellites => Icon::Satellite,
            View::Keys => Icon::Key,
            View::Control => Icon::Control,
        }
    }

    /// One line about what the view holds, under the name in the hover text.
    /// An icon alone is a rebus, and a strip of them needs more than a noun.
    fn about(self) -> &'static str {
        match self {
            View::Dashboard => "What the receiver can do, and what it is doing",
            View::Spectrum => "The span, and the waterfall under it",
            View::Chain => "The graph the receiver is running",
            View::Calls => "Who is talking, from every voice decoder",
            View::Transcript => "What was said, read by the local model",
            View::Messages => "Text sent over the air",
            View::Video => "Pictures, while something is sending them",
            View::Map => "Everything that reported a position",
            View::Links => "Who is talking to whom",
            View::Devices => "Transmitters seen, and where they were",
            View::Satellites => "Passes overhead, and what they send",
            View::Keys => "Encryption seen, and the keys held",
            View::Control => "Where the sticks are, on every handset heard",
        }
    }

    /// The strip, in two rows: what the receiver can do and what it heard on
    /// the top row, who is out there on the bottom. The dashboard leads,
    /// because it is where a receiver that has just been started is.
    const ROWS: [&'static [View]; 2] = [
        &[
            View::Dashboard,
            View::Spectrum,
            View::Chain,
            View::Calls,
            View::Transcript,
            View::Messages,
            View::Video,
        ],
        &[View::Map, View::Links, View::Devices, View::Control, View::Satellites, View::Keys],
    ];

    const COUNT: usize = View::ROWS[0].len() + View::ROWS[1].len();

    /// Where the view keeps what it has been seen holding. The strip's own
    /// order, so a reader of one is a reader of the other.
    fn slot(self) -> usize {
        View::ROWS.into_iter().flatten().position(|v| *v == self).unwrap_or(0)
    }
}

/// The digit that selects the `i`th tab on the strip, held with the modifier
/// key.
///
/// Positional rather than a property of the view, because which tabs are on
/// the strip depends on whether the dashboard is wanted: the number has to be
/// where the tab is, so hiding the dashboard puts the spectrum back on 1.
/// There are ten digits and thirteen views, so the last three tabs have no
/// key. Those are the control links, the satellites and the keys, which are
/// the ones nobody reaches for in a hurry.
fn tab_digit(i: usize) -> Option<(egui::Key, &'static str)> {
    use egui::Key::*;
    const KEYS: [(egui::Key, &str); 10] = [
        (Num1, "1"),
        (Num2, "2"),
        (Num3, "3"),
        (Num4, "4"),
        (Num5, "5"),
        (Num6, "6"),
        (Num7, "7"),
        (Num8, "8"),
        (Num9, "9"),
        (Num0, "0"),
    ];
    KEYS.get(i).copied()
}

/// Log segments a load reads back, newest first. At 256 MB each this is a
/// bounded amount of reading for a directory that is meant to open at once.
const LINK_LOG_SEGMENTS: usize = 2;

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
    const UNITS: [(&str, u64); 4] = [("GB", 1 << 30), ("MB", 1 << 20), ("kB", 1 << 10), ("B", 1)];
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
    Some((fixes.iter().map(|f| f.0).sum::<f64>() / n, fixes.iter().map(|f| f.1).sum::<f64>() / n))
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

/// How long a fault stays over the spectrum.
const ERR_SHOWN_FOR: std::time::Duration = std::time::Duration::from_secs(8);

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
            survey: state::SurveyState::default(),
            sats: state::SatsState::default(),
            map: map_pane::MapState::default(),
            rt: background_runtime(),
            calls: state::CallsState::default(),
            transcript: state::TranscriptState::default(),
            messages: state::MessagesState::default(),
            links: state::LinksState::default(),
            control: state::ControlState::default(),
            video: video_pane::VideoState::default(),
            keys: state::KeysState::default(),
            audio: state::AudioState::default(),
            cmds: Vec::new(),
            record_dir: None,
            radio: None,
            err: None,
            err_at: None,
            center: crate::session::DEFAULT_CENTER,
            rate: 2_304_000.0,
            dial: Dial::new(),
            open: None,
            devices: Vec::new(),
            device: None,
            reach: (24e6, 1766e6),
            tunable: true,
            spans: Vec::new(),
            zoom: 1,
            soak: None,
            shot: None,
            shot_after: 6.0,
            decode_on: true,
            capture: false,
            shot_at: None,
            shot_sent: false,
            autostart: false,
            dc_block: true,
            view: View::Dashboard,
            prev_view: View::Spectrum,
            dashboard: true,
            view_seen: [0; View::COUNT],
            video_seen: 0,
            video_live_was: false,
            location: None,
            accuracy_m: None,
            last_frame: None,
            country: String::new(),
            opencellid_token: String::new(),
            spacetrack_identity: String::new(),
            spacetrack_password: String::new(),
            audio_out: String::new(),
            audio_in: String::new(),
            radio_settings: Default::default(),
            ppm_by_device: Default::default(),
            radio_dirty: false,
            feeds: Vec::new(),
            feed_host: String::new(),
            remote: None,
            feed_kind: nodes::FEED_KINDS[0],
            scanner_edit: None,
            scanners: crate::scanners::Scanners::default(),
            memory: Default::default(),
            memory_group: crate::memory::UNGROUPED.into(),
            log_dir_edit: String::new(),
            log_dir: None,
            log_cap_mb: Some(crate::packetlog::DEFAULT_MAX_BYTES >> 20),
            capture_cap_mb: Some(nodes::capture_nodes::DEFAULT_BUDGET >> 20),
            station_edit: None,
            saved: crate::session::Session::default(),
            saved_at: None,
        }
    }
}

impl App {
    pub fn new(cc: &eframe::CreationContext<'_>) -> Self {
        theme::install(&cc.egui_ctx);
        crate::shutdown::install(cc.egui_ctx.clone());
        let mut s = crate::session::Session::load();
        apply_locale(&mut s);
        // The dataset cache is asked questions from drawing code with no
        // session to consult, so what it needs is handed to it once here:
        // the country picks which cell export applies, and the token is what
        // fetches it.
        crate::data::set_country(&s.country);
        crate::data::set_opencellid_token(&s.opencellid_token);
        datasets::spacetrack::set_account(Some(datasets::spacetrack::Account {
            identity: s.spacetrack_identity.clone(),
            password: s.spacetrack_password.clone(),
        }));
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
        let radio_settings = s.radio(device.as_ref().map(|d| d.label.as_str()));
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
            accuracy_m: None,
            country: s.country.clone(),
            opencellid_token: s.opencellid_token.clone(),
            spacetrack_identity: s.spacetrack_identity.clone(),
            spacetrack_password: s.spacetrack_password.clone(),
            audio_out: s.audio_out.clone(),
            audio_in: s.audio_in.clone(),
            radio_settings,
            ppm_by_device: s.ppm.clone(),
            feeds: s.feeds.clone(),
            log_cap_mb: s.log_cap_mb,
            capture_cap_mb: s.capture_cap_mb,
            scanners: crate::scanners::Scanners::load(),
            memory: crate::memory::Memory::load(),
            saved: s.clone(),
            dashboard: s.dashboard,
            view: if s.dashboard { View::Dashboard } else { View::Spectrum },
            ..Default::default()
        };
        app.map.map.layers.restore(&s.map_layers);
        app.scope.restore(&s.view, s.fft);
        app.scope.db_center = s.center;
        app.scope.wf_center = s.center;
        app.audio.volume = s.volume;
        // What the receiver writes down, as the operator last left it. Off
        // until asked: see `Session::packet_log_on`.
        app.log.path = s.packet_log_on.then(crate::packetlog::PacketLog::default_dir).flatten();
        app.survey.path =
            s.survey_on.then(crate::packetlog::PacketLog::default_survey_path).flatten();
        // A GPS named once stays named: a survey is usually the same drive
        // with the same receiver, and typing the port again every start is
        // the difference between a tool and a demonstration.
        app.survey.gps = gps::Transport::parse(&s.gps);
        app.survey.wigle.name = s.wigle_name.clone();
        app.survey.wigle.token = s.wigle_token.clone();
        app.survey.wigle.donate = s.wigle_donate;
        app.survey.wigle.on = s.wigle_on;
        app.survey.beacondb.on = s.beacondb_on;
        app.survey.beacondb.lookup = s.beacondb_lookup;
        crate::beacondb::set_lookup(s.beacondb_lookup);
        crate::beacondb::start();
        app.radio_dirty = true;
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
        let rs = &self.radio_settings;
        crate::session::Session {
            device: self.device.as_ref().map(|d| d.label.clone()),
            center: self.center,
            rate: self.rate * self.zoom.max(1) as f64,
            zoom: self.zoom,
            fft: self.scope.fft,
            gains: rs.gains.clone(),
            toggles: rs.toggles.clone(),
            choices: rs.choices.clone(),
            ppm: self.ppm_by_device.clone(),
            tx_gain_db: rs.tx_gain_db,
            location: self.location,
            language: crate::i18n::language().code().to_string(),
            country: self.country.clone(),
            opencellid_token: self.opencellid_token.clone(),
            spacetrack_identity: self.spacetrack_identity.clone(),
            spacetrack_password: self.spacetrack_password.clone(),
            audio_out: self.audio_out.clone(),
            audio_in: self.audio_in.clone(),
            band_plan: crate::bands::plan().id().to_string(),
            view: self.scope.prefs(),
            feeds: self.feeds.clone(),
            streams: crate::devices::streams().into_iter().map(|r| (r.addr, r.label)).collect(),
            dc_block: self.dc_block,
            decode_on: self.decode_on,
            volume: self.audio.volume,
            log_cap_mb: self.log_cap_mb,
            gps: self.survey.gps.as_ref().map(|t| t.to_string()).unwrap_or_default(),
            wigle_name: self.survey.wigle.name.clone(),
            wigle_token: self.survey.wigle.token.clone(),
            wigle_donate: self.survey.wigle.donate,
            wigle_on: self.survey.wigle.on,
            beacondb_on: self.survey.beacondb.on,
            beacondb_lookup: self.survey.beacondb.lookup,
            capture_cap_mb: self.capture_cap_mb,
            manual_chain: self.chain.edit.manual,
            packet_log_on: self.log.path.is_some(),
            survey_on: self.survey.path.is_some(),
            map_layers: self.map.map.layers.saved(),
            dashboard: self.dashboard,
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

    /// Put the radio settings on the radio.
    ///
    /// The one path. The settings pane calls it after changing a field, a
    /// connect or a reset calls it once the driver has reported its controls,
    /// and a headless start calls it the same way: there is no second copy of
    /// this logic that could disagree with the first.
    ///
    /// Deferred until the driver has reported its controls, because the stage
    /// names come from the driver and until then there is nothing to check a
    /// saved name against. A name the radio does not have is left alone
    /// rather than refused, so a session saved on one radio applies what it
    /// can to another.
    fn apply_radio_settings(&mut self) {
        let Some(radio) = self.radio.as_ref() else {
            return;
        };
        let controls = radio.status.radio();
        if controls.stages.is_empty() && controls.toggles.is_empty() && controls.choices.is_empty()
        {
            return;
        }
        self.radio_dirty = false;
        let want = self.radio_settings.clone();
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
        self.send(Cmd::Ppm(want.ppm));
        if !controls.tx_stages.is_empty() {
            self.send(Cmd::TxGain(want.tx_gain_db));
        }
    }

    /// The rest of what a freshly opened radio has to be told, which is not
    /// a radio setting but travels with them on connect.
    fn restore_radio_settings(&mut self) {
        if !self.radio_dirty {
            return;
        }
        let Some(radio) = self.radio.as_ref() else {
            return;
        };
        let controls = radio.status.radio();
        if controls.stages.is_empty() && controls.toggles.is_empty() && controls.choices.is_empty()
        {
            return;
        }
        self.apply_radio_settings();
        let want = self.saved.clone();
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
        self.accuracy_m = None;
        self.send(Cmd::Location(lat, lon));
    }

    /// Start or stop recording the survey, or point it at another file.
    pub fn set_survey(&mut self, off: bool, path: Option<std::path::PathBuf>) {
        self.survey.path =
            if off { None } else { path.or_else(crate::packetlog::PacketLog::default_survey_path) };
        let p = self.survey.path.clone();
        self.send(Cmd::Survey(p));
        // The pane reads the same file through a connection of its own. It
        // does not exist until the radio thread has created it, so opening
        // here is allowed to fail and is retried while the pane is drawn.
        self.survey.db = None;
        self.survey.refreshed = None;
    }

    /// Read the receiver's own position from a named GPS, or `None` to go
    /// back to the local gpsd the reader finds on its own.
    ///
    /// Set here rather than sent to the radio, since the reader is not the
    /// radio's: choosing a GPS works with no device connected, and the radio
    /// thread reads the same fixes when there is one.
    pub fn set_gps(&mut self, transport: Option<gps::Transport>) {
        self.survey.gps = transport.clone();
        crate::station::set_source(transport);
    }

    /// Move the station to wherever the GPS last said, whether or not a radio
    /// is running.
    ///
    /// The station is one position for the whole receiver, so this is also
    /// what the map, the range rings and anything resolving a bearing against
    /// the receiver follow.
    fn follow_gps(&mut self) {
        let Some(f) = crate::station::fix() else {
            return;
        };
        self.accuracy_m = f.accuracy_m();
        let moved = self
            .location
            .is_none_or(|(lat, lon)| (lat - f.lat).abs() > 1e-5 || (lon - f.lon).abs() > 1e-5);
        if moved {
            self.location = Some((f.lat, f.lon));
            // The box in settings shows the position; a stale string in it
            // would sit there claiming the receiver had not moved.
            self.station_edit = None;
        }
    }

    /// Turn the packet log off, or point it somewhere other than the default.
    pub fn set_packet_log(&mut self, off: bool, dir: Option<std::path::PathBuf>) {
        self.log.path =
            if off { None } else { dir.or_else(crate::packetlog::PacketLog::default_dir) };
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
        self.radio_settings.set_gain("tuner", common::GainMode::Manual(db));
        self.radio_dirty = true;
    }

    /// Correct the reference of the radio in use, and record the figure
    /// against that radio so it is not applied to the next one.
    pub fn set_ppm(&mut self, ppm: f64) {
        self.radio_settings.ppm = ppm;
        if let Some(d) = self.device.as_ref() {
            self.ppm_by_device.insert(d.label.clone(), ppm);
        }
    }

    /// Start on the radio whose label contains `want`, for when several are
    /// plugged in and the saved one is not the one wanted.
    /// Start the radio without waiting for the play button, which is what a
    /// capture being replayed usually wants and what a screenshot needs.
    pub fn start_on_open(&mut self) {
        self.autostart = true;
    }

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
            bandwidth_hz: None,
            label: format!("{mhz:.1}"),
            on: true,
            volume: 0.8,
            muted: false,
            squelch_db: None,
            agc: true,
            voice: speaks(&ChanMode::Audio(demod)),
            tx: None,
            doppler: false,
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
            self.err_at = Some(std::time::Instant::now());
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
        // The thread opens the default speaker at startup; this puts the one
        // the session asked for in its place, and hands over the microphone
        // to use when a channel is keyed.
        if !self.audio_out.is_empty() || !self.audio_in.is_empty() {
            self.send_audio();
        }
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
        // The survey and the GPS are the same: both belong to the graph and
        // the thread that had them is gone.
        if let Some(p) = self.survey.path.clone() {
            self.send(Cmd::Survey(Some(p)));
        }
        if let Some(t) = self.survey.gps.clone() {
            self.send(Cmd::Gps(Some(t)));
        }
        if self.survey.wigle.on {
            self.apply_wigle();
        }
        if self.survey.beacondb.on {
            self.apply_beacondb();
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
        // Whatever the radio was set to has to be pushed at it again: a new
        // thread means a freshly opened device at its defaults. Start and
        // reset are the same path through here.
        self.radio_dirty = true;
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
        // The correction belongs to the crystal, not to the receiver: the
        // radio being put down keeps its figure and the one picked up brings
        // its own, which is zero until somebody has calibrated it.
        self.radio_settings.ppm = self.ppm_by_device.get(&e.label).copied().unwrap_or(0.0);
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
        self.follow_gps();
        let Some(radio) = &self.radio else { return };
        // The flight tracker lives in the graph; this is the table it
        // published on the last frame.
        if self.view == View::Map || !self.map.tracks.is_empty() {
            self.map.tracks = radio.status.track_list.lock().clone();
        }
        // A fix moves the station. The receiver already has it, since the
        // station is moved on the radio thread, where the survey and the
        // tracker read it; this is the interface following the same position,
        // so the map, the range rings and anything else that resolves against
        // the station are where the receiver actually is rather than where it
        // was parked this morning.
        // A fault first, then whatever the last rebuild could not put in the
        // graph. Two slots because they are different in kind, and one banner
        // because there is one place to read a sentence: what went wrong wins
        // over a standing verdict on the chain.
        let fault = radio
            .status
            .error
            .lock()
            .take()
            .or_else(|| radio.status.refused.lock().take());
        if let Some(e) = fault {
            self.err = Some(e);
            self.err_at = Some(std::time::Instant::now());
        }
        // A fault is worth a look, not a permanent fixture: the radio going
        // quiet re-raises itself every frame for as long as it is true, and
        // anything else was true once.
        if self.err_at.is_some_and(|t| t.elapsed() > ERR_SHOWN_FOR) {
            self.err = None;
            self.err_at = None;
        }
        let mut frames: Vec<Frame> = Vec::new();
        while let Ok(f) = radio.frames.try_recv() {
            frames.push(f);
        }
        // A radio that has stopped delivering says so. Without this the
        // window carries on drawing the last spectrum it was given, which is
        // indistinguishable from a very quiet band: the receiver looks like
        // it is working right up until somebody notices the waterfall has not
        // moved in a minute.
        if !frames.is_empty() {
            self.last_frame = Some(std::time::Instant::now());
        } else if radio.status.running.load(std::sync::atomic::Ordering::Relaxed) {
            let since = self.last_frame.get_or_insert_with(std::time::Instant::now).elapsed();
            if since > std::time::Duration::from_secs(3) {
                self.err = Some(format!(
                    "the radio has sent nothing for {:.0} s; it may need unplugging",
                    since.as_secs_f32()
                ));
                self.err_at = Some(std::time::Instant::now());
            }
        }
        // A pinned radio cannot be retuned, and a dial left wherever it was
        // dragged would label every frequency on screen wrongly.
        if let Some(f) = self.device.as_ref().and_then(|d| d.pinned) {
            self.center = f.as_f64();
        }
        {
            let c = radio.status.radio();
            self.reach = c.reach;
            self.tunable = c.tunable;
        }
        // Every frame is peak-held into the pending row, not just the one that
        // happens to be last in the queue. Folding only the last of each batch
        // tied a waterfall row's content to how often the interface repainted:
        // dragging a slider repaints continuously, fewer frames were thrown
        // away between drains, and the history visibly changed contrast for as
        // long as the drag lasted.
        self.chain.topo = radio.status.chain();
        self.chain.latency = radio.status.chain_latency();
        self.chain.scopes = radio.status.scopes();
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
            // A setting changed by parameter, from the chain inspector or
            // the transcript card, is an edit the receiver made on the
            // interface's behalf: it comes back here as the running graph
            // and has to be written out like one drawn by hand, or the
            // model picked is the model until the program is restarted.
            let edits = crate::patch::Edits::diff(
                &self.chain.patch,
                &self.chain.base,
                crate::chain::operator_owns,
            );
            if edits != self.chain.edits {
                self.chain.edits = edits;
                self.chain.save_patch();
            }
        }
        // A level set in the chain view lands on the node, and the strip
        // has to follow or the next thing it sends puts the level back.
        let levels = radio.status.levels();
        if levels.rev != self.audio.levels_rev {
            self.audio.levels_rev = levels.rev;
            if levels.rev > 0 {
                self.audio.volume = levels.audio.master;
                self.audio.muted = levels.audio.muted;
                self.audio.call_volume = levels.audio.calls;
                self.audio.call_muted = levels.audio.calls_muted;
                self.audio.call_agc = levels.audio.agc;
                for spec in levels.channels {
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

            let due = self
                .scope
                .wf_last
                .map(|t| t.elapsed().as_secs_f32() >= 1.0 / self.scope.rows_per_sec)
                .unwrap_or(true);
            if due {
                // The waterfall tops out below the trace's ceiling: the plot
                // wants headroom so peaks are not clipped flat, the colour
                // ramp wants the opposite or its hottest colours go unused.
                let pending = std::mem::take(&mut self.scope.wf_pending);
                self.scope.wf.push(
                    &pending,
                    self.scope.floor,
                    self.scope.ceil - self.scope.wf_top_offset,
                );
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
            // And a transmission that names an end is a link, whether or not
            // anybody spoke or wrote: a meter, an advertiser and a pager
            // capcode all belong in the directory.
            self.links.list.update(&rec, rec.at);
            // A handset's sticks are state rather than a stream: the control
            // view holds the last of each channel per transmitter.
            self.control.list.update(&rec, rec.at);
            let id = self.log.next_packet;
            self.log.next_packet += 1;
            self.log.decodes.push(Logged { id, rec });
        }
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
            center: self.center,
            rate: self.rate,
            memory: &mut self.memory,
            memory_group: &mut self.memory_group,
            acts: Vec::new(),
            cmds: &mut self.cmds,
        }
        .show(ui);
        for a in acts {
            match a {
                strip::Action::Channels => self.send_channels(),
                strip::Action::Open(w) => self.open = Some(w),
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
        // The trail is whatever the device list has selected, so choosing a
        // device in one view and looking at the map in the other shows the
        // same device.
        let ident = self
            .survey
            .selected
            .and_then(|id| self.survey.rows.iter().find(|d| d.id == id))
            .map(|d| d.ident.clone());
        let trail = map_pane::Trail {
            points: &self.survey.trail,
            ident: ident.as_deref(),
            estimate: self.survey.estimate,
        };
        let place = map_pane::Map {
            st: &mut self.map,
            home: self.location,
            accuracy_m: self.accuracy_m,
            edit: &mut edit,
            trail,
            heard: &self.survey.rows,
            sat: self.sats.selected,
            sat_group: self.sats.group,
            rt,
        }
        .show(ui);
        self.station_edit = edit;
        // Clicking a satellite selects it, and clicking it again lets it go:
        // the selection is what draws its ground track and its footprint,
        // and there has to be a way to stop drawing them.
        if let Some(norad) = self.map.sat_hit {
            self.sats.selected = (self.sats.selected != Some(norad)).then_some(norad);
        }
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
            said: &self.transcript.log,
            cmds: &mut self.cmds,
        }
        .show(ui);
        match act {
            Some(calls_pane::Action::Tune(hz)) => self.set_center(hz / 1e6),
            Some(calls_pane::Action::Clear) => self.calls.list.clear(),
            Some(calls_pane::Action::Transcript(key)) => self.show_transcript(Some(key)),
            None => {}
        }
    }

    /// Draw whatever the video bus is publishing.
    fn video_view(&mut self, ui: &mut egui::Ui) {
        let (frame, inputs) = match self.radio.as_ref() {
            Some(r) => (r.status.video(), r.status.video_inputs()),
            None => (None, Vec::new()),
        };
        video_pane::VideoPane { st: &mut self.video, frame, inputs, cmds: &mut self.cmds }.show(ui);
    }

    /// Fold in who the audio bus is hearing, and subscribe to anything new.
    ///
    /// The call list is fed from here for everything that talks, analogue or
    /// digital, because every demodulator's audio goes through the bus first
    /// and the bus is the one thing that knows who is on the air now. What a
    /// decoder knows besides, the cipher, the codec, arrives on the packet
    /// side through `log_decodes` and lands on the same row.
    ///
    /// The one place anything is subscribed to, for the same reason: a group
    /// worth listening to is one the bus has heard speech on. Subscribing
    /// where the list is drawn meant a receiver sitting on the spectrum
    /// heard nothing however much it decoded, and subscribing from a decode
    /// meant subscribing to calls the receiver cannot play.
    ///
    /// Every frame, whichever pane is on screen.
    fn read_heard(&mut self) {
        let Some(r) = &self.radio else {
            return;
        };
        let heard = std::mem::take(&mut *r.status.heard.lock());
        if heard.is_empty() {
            return;
        }
        for c in &heard {
            self.calls.list.hear(c);
        }
        let active: Vec<crate::calls::Call> =
            self.calls.list.active(std::time::Instant::now()).into_iter().cloned().collect();
        let mut cmds = std::mem::take(&mut self.cmds);
        self.calls.subscribe_new(&active, &mut cmds);
        self.cmds = cmds;
    }

    /// Take a fresh copy of the transcript when it has changed, and give the
    /// call list the newest line for each call.
    ///
    /// The transcript belongs to the receiver and the node writes into it
    /// from the radio thread; the view draws from a copy so it never holds
    /// the lock while drawing. Every frame rather than when a packet
    /// arrives, because speech is not a packet.
    fn read_said(&mut self) {
        let Some(shared) = self.radio.as_ref().map(|r| r.status.transcript()) else {
            return;
        };
        let seq = shared.lock().seq();
        if seq == self.transcript.seq {
            return;
        }
        self.transcript.seq = seq;
        self.transcript.log = shared.lock().snapshot();
        let said = self.transcript.log.recent(256).into_iter().cloned().collect::<Vec<_>>();
        // A call that produced no speech the model would read keeps whatever
        // text its own decoder gave it.
        self.calls.list.read_transcripts(&said);
    }

    /// Draw the transcript, then do what its buttons asked for.
    fn transcript_view(&mut self, ui: &mut egui::Ui) {
        let engine = self.radio.as_ref().and_then(|r| r.status.transcriber.lock().clone());
        let act =
            transcript_pane::Transcript { st: &mut self.transcript, engine, cmds: &mut self.cmds }
                .show(ui);
        match act {
            Some(transcript_pane::Action::Clear) => {
                if let Some(r) = self.radio.as_ref() {
                    r.status.transcript().lock().clear();
                }
                self.transcript.log.clear();
                self.transcript.only = None;
            }
            None => {}
        }
    }

    /// Draw the message list, then do what its buttons asked for.
    fn message_view(&mut self, ui: &mut egui::Ui) {
        let act = messages_pane::Msgs { st: &mut self.messages }.show(ui);
        match act {
            Some(messages_pane::Action::Clear) => self.messages.list.clear(),
            None => {}
        }
    }

    /// Draw the data links directory, and the link being followed.
    fn links_view(&mut self, ui: &mut egui::Ui) {
        // The packet list is where a followed link's packets come from, so
        // the two views cannot disagree about a packet they have both seen.
        let packets: Vec<crate::radio::DecodeRecord> =
            self.log.decodes.iter().map(|l| l.rec.clone()).collect();
        match (links_pane::LinksView { st: &mut self.links, packets: &packets }).show(ui) {
            Some(links_pane::Action::Clear) => self.links.list = crate::links::Links::new(),
            Some(links_pane::Action::LoadLog) => self.load_links_from_log(),
            None => {}
        }
    }

    /// Draw the device database, then do what a click asked for.
    fn devices_view(&mut self, ui: &mut egui::Ui) {
        let counts = match self.radio.as_ref() {
            Some(r) => (
                r.status.survey_devices.load(std::sync::atomic::Ordering::Relaxed),
                r.status.survey_sightings.load(std::sync::atomic::Ordering::Relaxed),
                r.status.survey_heard.load(std::sync::atomic::Ordering::Relaxed),
            ),
            None => (0, 0, 0),
        };
        self.refresh_survey();
        let act = devices_pane::Devices {
            st: &mut self.survey,
            counts,
            fix: crate::station::fix(),
            gps_connected: crate::station::connected(),
        }
        .show(ui);
        match act {
            Some(devices_pane::Action::Select(id)) => {
                self.survey.selected = id;
                // The trail is fetched on the change rather than per frame:
                // a device heard all afternoon has thousands of sightings and
                // the map only redraws when one of them moves.
                self.survey.trail = match (id, self.survey.db.as_ref()) {
                    (Some(id), Some(db)) => db.sightings(id).unwrap_or_default(),
                    _ => Vec::new(),
                };
                self.survey.estimate = survey::locate(&self.survey.trail);
            }
            Some(devices_pane::Action::Export) => self.export_survey(),
            Some(devices_pane::Action::Wigle) => self.survey.wigle.open = true,
            Some(devices_pane::Action::BeaconDb) => self.survey.beacondb.open = true,
            None => {}
        }
    }

    /// Read the packet log back into the links directory.
    ///
    /// Newest segments first and bounded, because a folder holds gigabytes
    /// and the directory is a view: what an operator wants on opening it is
    /// the recent past, not the whole disk.
    fn load_links_from_log(&mut self) {
        let Some(dir) = crate::packetlog::PacketLog::default_dir() else {
            self.links.error = Some("no packet log folder".into());
            return;
        };
        let segments = crate::links::segments(&dir);
        let recent: Vec<_> = segments.iter().rev().take(LINK_LOG_SEGMENTS).rev().collect();
        if recent.is_empty() {
            self.links.error = Some(format!("no log segments in {}", dir.display()));
            return;
        }
        self.links.error = None;
        for path in recent {
            match crate::links::from_log(path) {
                Ok(l) => self.links.list.absorb(l),
                Err(e) => self.links.error = Some(format!("{}: {e}", path.display())),
            }
        }
    }

    /// Read the survey's rows again, at most once a second.
    ///
    /// The radio thread owns the file and appends to it continuously; this is
    /// a second connection reading the same one. Whatever it last read is
    /// what the pane and the map draw, so a survey being written while it is
    /// being looked at lags by up to a second and never blocks the writer.
    fn refresh_survey(&mut self) {
        let Some(path) = self.survey.path.clone() else {
            self.survey.rows.clear();
            return;
        };
        if self.survey.db.is_none() {
            self.survey.db = survey::Db::open_read(&path).ok();
        }
        let due =
            self.survey.refreshed.is_none_or(|t| t.elapsed() >= std::time::Duration::from_secs(1));
        if !due {
            return;
        }
        if let Some(db) = self.survey.db.as_ref() {
            if let Ok(rows) = db.devices(survey::Query::default()) {
                self.survey.rows = rows;
            }
            if let Some(id) = self.survey.selected {
                self.survey.trail = db.sightings(id).unwrap_or_default();
                self.survey.estimate = survey::locate(&self.survey.trail);
            }
        }
        self.survey.refreshed = Some(std::time::Instant::now());
    }

    /// Start or stop feeding wigle.net with what has been typed into the
    /// modal.
    ///
    /// The account goes to the radio thread rather than being kept here: the
    /// feed is a node on the packet bus, and the interface holding an account
    /// the node had not been told about would be a switch that reports on.
    fn apply_wigle(&mut self) {
        let account = survey::Account {
            name: self.survey.wigle.name.trim().to_string(),
            token: self.survey.wigle.token.trim().to_string(),
            donate: self.survey.wigle.donate,
        };
        self.survey.wigle.on = self.survey.wigle.on && account.is_complete();
        let on = self.survey.wigle.on;
        self.send(Cmd::Wigle(on.then_some(account)));
    }

    /// Start or stop submitting to beaconDB.
    ///
    /// The switch goes to the radio thread rather than being kept here: the
    /// feed is a node on the packet bus, and an interface holding a switch
    /// the node had not been told about would be a switch that reports on.
    fn apply_beacondb(&mut self) {
        self.send(Cmd::BeaconDb(self.survey.beacondb.on));
        // The lookup runs here rather than in the receiver: it answers the
        // map, not the graph.
        crate::beacondb::set_lookup(self.survey.beacondb.lookup);
    }

    /// Write the survey out as WiGLE CSV, beside the survey file.
    fn export_survey(&mut self) {
        let (Some(path), Some(db)) = (self.survey.path.clone(), self.survey.db.as_ref()) else {
            return;
        };
        let out = path.with_extension("wigle.csv");
        let written = std::fs::File::create(&out).map_err(|e| e.to_string()).and_then(|mut f| {
            survey::write_wigle(db, survey::Query::default(), &mut f).map_err(|e| e.to_string())
        });
        // The same banner a radio fault uses: an export is a thing that
        // happened once, and it should say so and then get out of the way.
        self.err = Some(match written {
            Ok(n) => format!("wrote {n} rows to {}", out.display()),
            Err(e) => format!("survey export failed: {e}"),
        });
        self.err_at = Some(std::time::Instant::now());
    }

    /// Draw the dashboard, then do what it asked for.
    fn dashboard_view(&mut self, ui: &mut egui::Ui) {
        let acts = dashboard_pane::Dashboard {
            radio: self.radio.as_ref(),
            device: self.device.as_ref().map(|d| d.label.as_str()),
            center: self.center,
            rate: self.rate,
            zoom: self.zoom,
            decode_on: self.decode_on,
            counts: dashboard_pane::Counts {
                tracks: self.map.tracks.len(),
                calls: self.calls.list.len(),
                messages: self.messages.list.len(),
                links: self.links.list.len(),
                fix: self.accuracy_m.is_some(),
            },
        }
        .show(ui);
        for a in acts {
            match a {
                dashboard_pane::Action::Open(v) => self.set_view(v),
                dashboard_pane::Action::Panel(p) => self.open = Some(p),
                dashboard_pane::Action::Start => {
                    let c = ui.ctx().clone();
                    self.connect(&c);
                }
                dashboard_pane::Action::Hide => self.hide_dashboard(),
            }
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

    /// The pass table, and what its buttons asked for.
    fn sats_view(&mut self, ui: &mut egui::Ui) {
        let acts = sats_pane::Sats { st: &mut self.sats, home: self.location }.show(ui);
        for a in acts {
            match a {
                sats_pane::Action::ShowOnMap => self.set_view(View::Map),
                sats_pane::Action::Track(d) => self.listen_to_satellite(&d),
                sats_pane::Action::Untrack => self.stop_tracking(),
            }
        }
    }

    /// Put a channel on a satellite's downlink and follow it down.
    ///
    /// Always following, because there is no useful other kind: a downlink
    /// tuned once is off the transmission within the minute, so a control
    /// that tuned without correcting would be a control that stops working
    /// while you watch it.
    ///
    /// A channel of its own rather than moving whatever was being listened
    /// to: the correction only makes sense on that one downlink, and moving
    /// a channel somebody put on a repeater would be a control that eats
    /// another one's work.
    fn listen_to_satellite(&mut self, d: &sats_pane::Downlink) {
        let shifted = self.doppler_at(d.norad, d.hz).unwrap_or(d.hz);
        // The dial has to be able to reach it. A satellite channel is the
        // one case where what to listen to is chosen before the receiver is
        // pointed anywhere near it.
        if !self.audio.channels.iter().any(|c| c.on)
            || (shifted - self.center).abs() > self.rate / 2.0
        {
            self.set_center(shifted / 1e6);
        }
        let mode = sat_mode(d.kind);
        self.push_channel(shifted, mode, Some(sat_label(d)));
        let Some(c) = self.audio.channels.last_mut() else {
            return;
        };
        // SatNOGS quotes a LoRa transmitter's bandwidth in the baud field,
        // so the chirp is built at the width it was sent at rather than at
        // the front end's default.
        if d.kind == datasets::satnogs::Mode::Lora {
            c.bandwidth_hz = d.baud.filter(|b| *b >= 1_000.0);
        }
        c.doppler = true;
        let channel = c.id;
        self.sats.tracking =
            Some(state::Tracking { norad: d.norad, downlink_hz: d.hz, channel, tuned_hz: shifted });
        self.send_channels();
    }

    /// Take a channel out of the strip by its id, keeping the listening
    /// selection pointed at whatever it was pointed at.
    fn close_channel(&mut self, id: u64) {
        let Some(i) = self.audio.channels.iter().position(|c| c.id == id) else {
            return;
        };
        self.audio.channels.remove(i);
        match self.audio.listening {
            Some(l) if l == i => self.audio.listening = None,
            Some(l) if l > i => self.audio.listening = Some(l - 1),
            _ => {}
        }
        self.send_channels();
    }

    /// Stop following, and give the channel its dial back rather than
    /// leaving one nobody is allowed to tune.
    fn stop_tracking(&mut self) {
        let Some(t) = self.sats.tracking.take() else {
            return;
        };
        if let Some(c) = self.audio.channels.iter_mut().find(|c| c.id == t.channel) {
            c.doppler = false;
        }
    }

    /// Where a satellite is from here, now.
    fn look_at(&self, norad: u64) -> Option<orbit::Look> {
        let (lat, lon) = self.location?;
        let sky = crate::sats::sky(self.sats.group)?;
        sky.get(norad)?.look(orbit::Station::new(lat, lon), crate::sats::now_s())
    }

    /// Where a satellite's downlink is arriving right now, or `None` when
    /// there is no station or no elements.
    fn doppler_at(&self, norad: u64, downlink_hz: f64) -> Option<f64> {
        Some(self.look_at(norad)?.doppler_hz(downlink_hz))
    }

    /// Move the tracking channel to where the downlink is now.
    ///
    /// Once a frame, which at any refresh rate is far oftener than the shift
    /// changes by anything a receiver can act on, so the retune is gated on
    /// a threshold rather than on a clock: a command per frame would be a
    /// channel rebuilt sixty times a second for a few hertz.
    fn follow_doppler(&mut self) {
        let Some(t) = self.sats.tracking else { return };
        // A channel the operator closed ends the tracking with it, rather
        // than leaving a control that is following nothing.
        if !self.audio.channels.iter().any(|c| c.id == t.channel) {
            self.sats.tracking = None;
            return;
        }
        let Some(look) = self.look_at(t.norad) else {
            return;
        };
        // The pass ends and the channel goes with it. A channel left on a
        // frequency nothing is transmitting on is a strip of noise the
        // operator has to notice and close, and the next pass makes another
        // one: an hour of watching leaves a dozen.
        if look.el_deg <= 0.0 {
            self.close_channel(t.channel);
            self.sats.tracking = None;
            return;
        }
        let now_hz = look.doppler_hz(t.downlink_hz);
        // A hundred hertz is inside the narrowest channel this receiver
        // demodulates and is about a second of drift on a two-metre pass.
        if (now_hz - t.tuned_hz).abs() < 100.0 {
            return;
        }
        if let Some(c) = self.audio.channels.iter_mut().find(|c| c.id == t.channel) {
            c.freq = now_hz;
        }
        self.sats.tracking = Some(state::Tracking { tuned_hz: now_hz, ..t });
        self.send_channels();
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
            err: self.err.as_deref(),
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
        let (lo, hi) = self.reach;
        self.center = hz.clamp(lo, hi);
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

    /// A saved channel onto the strip, and the dial to it if the span does
    /// not reach it: recalling a channel is asking to hear it.
    fn recall(&mut self, s: &crate::memory::Saved) {
        if (s.freq - self.center).abs() > self.rate / 2.0 {
            self.retune(s.freq);
        }
        let label = if s.label.trim().is_empty() { None } else { Some(s.label.clone()) };
        self.push_channel(s.freq, s.mode.clone(), label);
        if let Some(c) = self.audio.channels.last_mut() {
            c.bandwidth_hz = s.bandwidth_hz;
        }
        self.send_channels();
    }

    /// A channel on the strip, tuned to `freq` and doing `mode` with it.
    fn push_channel(&mut self, freq: f64, mode: ChanMode, label: Option<String>) {
        let id = self.audio.next_id;
        self.audio.next_id += 1;
        self.audio.channels.push(Channel {
            id: id as u64,
            freq,
            voice: speaks(&mode),
            mode,
            bandwidth_hz: None,
            label: label.unwrap_or_else(|| format!("CH{id}")),
            on: true,
            volume: 0.8,
            muted: false,
            squelch_db: None,
            agc: true,
            tx: None,
            doppler: false,
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
    /// Tell the radio thread which sound devices to use.
    fn send_audio(&mut self) {
        let (out, input) = (self.audio_out.clone(), self.audio_in.clone());
        self.send(Cmd::Audio { out, input });
    }

    fn send_channels(&mut self) {
        let specs = self.channel_specs();
        self.send(Cmd::Channels(specs));
    }

    fn channel_specs(&self) -> Vec<ChannelSpec> {
        let center = self.center;
        self.audio
            .channels
            .iter()
            .filter(|c| c.on)
            .map(|c| ChannelSpec {
                id: c.id,
                label: c.label.clone(),
                offset_hz: c.freq - center,
                mode: c.mode.clone(),
                bandwidth_hz: c.bandwidth_hz,
                volume: c.volume,
                muted: c.muted,
                squelch_db: c.squelch_db,
                agc: c.agc,
                voice: c.voice,
                tx: c.tx,
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
/// Whether a new channel in this mode is speech somebody wants a record of.
///
/// On for the modes people talk on, so a channel added to listen to a
/// repeater is in the call list and transcribed without a second switch being
/// found first. Off for broadcast FM, which would otherwise transcribe a
/// music station for as long as the receiver is on, and off for anything that
/// is not audio: a decoder's channel produces its own calls.
fn speaks(mode: &ChanMode) -> bool {
    matches!(mode, ChanMode::Audio(Demod::Nfm | Demod::Am | Demod::Usb | Demod::Lsb))
}

fn front_for(model: &str) -> Option<&'static str> {
    let system = model.split('-').next().unwrap_or(model).to_ascii_lowercase();
    let system = if system == "ax25" { "aprs" } else { system.as_str() };
    crate::chain::channel_fronts().iter().map(|(k, _)| *k).find(|k| *k == system)
}

/// What to build for a downlink, from what SatNOGS says its mode is.
///
/// Speech and Morse get the demodulator for them. A mode this receiver has a
/// front end for gets that front end, asked for by name against the registry
/// so a decoder added there is used the day it arrives: a LoRa downlink was
/// given the auto node while `lora` sat in the registry, which meant the
/// spreading factor and bandwidth had to be found again by measurement.
/// Everything else, the phase-keyed telemetry and the framings with no
/// decoder here, gets the auto front end, which measures what is in the
/// channel and places whatever reads it.
fn sat_mode(mode: datasets::satnogs::Mode) -> ChanMode {
    use datasets::satnogs::Mode as M;
    let front = |id: &str| {
        crate::chain::front_kind(id).map_or(ChanMode::Auto, |k| ChanMode::Decode(k.into()))
    };
    match mode {
        M::Fm => ChanMode::Audio(Demod::Nfm),
        M::Am => ChanMode::Audio(Demod::Am),
        M::Usb => ChanMode::Audio(Demod::Usb),
        M::Lsb => ChanMode::Audio(Demod::Lsb),
        M::Cw => ChanMode::Audio(Demod::Cw),
        M::Lora => front("lora"),
        M::Dmr => front("dmr"),
        // AX.25 at 1200 baud is what AFSK on a satellite nearly always is,
        // and the APRS front end is the one that reads it.
        M::Afsk => front("aprs"),
        M::Fsk
        | M::Gfsk
        | M::Gmsk
        | M::Msk
        | M::Bpsk
        | M::Qpsk
        | M::Psk
        | M::Ask
        | M::Dvb
        | M::Sstv
        | M::Apt
        | M::Lrpt
        | M::Hrpt
        | M::Duv
        | M::Dstar
        // C4FM here is System Fusion, which this does not read: it is not
        // M17, whatever the keying looks like.
        | M::C4fm
        | M::Other => ChanMode::Auto,
    }
}

/// What the strip calls a satellite channel: the satellite and which of its
/// transmitters this is, because a bird with a voice repeater and a
/// telemetry beacon is two channels that are otherwise identical.
fn sat_label(d: &sats_pane::Downlink) -> String {
    let what = match (d.what.is_empty(), d.mode.is_empty()) {
        (false, _) => d.what.clone(),
        (true, false) => d.mode.clone(),
        (true, true) => format!("{:.3} MHz", d.hz / 1e6),
    };
    format!("{}:{what}", d.sat)
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
        if self.autostart {
            self.autostart = false;
            self.connect(ui.ctx());
        }
        self.screenshot(ui.ctx());
        // Who is talking and what they said, every frame and whichever view
        // is open. Both used to be read only under --soak, so the call list
        // and the transcript filled in a soak run and stayed empty in use.
        self.read_heard();
        self.read_said();
        self.soak_check(ui.ctx());
        // Read once a frame rather than where it is drawn: the pane's button
        // and the modal both show it, and only one of them is ever open.
        self.survey.wigle.status = self.radio.as_ref().and_then(|r| r.status.wigle.lock().clone());
        self.survey.beacondb.status =
            self.radio.as_ref().and_then(|r| r.status.beacondb.lock().clone());
        // Whatever view is open: a pass does not stop moving because the
        // operator went to look at the spectrum.
        self.follow_doppler();
        if crate::shutdown::asked() {
            // Closing rather than exiting, so the session is saved and the
            // radio and the log are dropped the way a click on the close
            // button drops them.
            ui.ctx().send_viewport_cmd(egui::ViewportCommand::Close);
        }
        self.view_keys(ui.ctx());
        self.read_views();
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
            CentralPanel::default().frame(egui::Frame::NONE.fill(theme::CHASSIS)).show(ui, |ui| {
                match self.view {
                    View::Dashboard => self.dashboard_view(ui),
                    View::Spectrum => self.scope_view(ui),
                    View::Chain => self.chain_view(ui),
                    View::Map => self.map_view(ui),
                    View::Calls => self.call_view(ui),
                    View::Transcript => self.transcript_view(ui),
                    View::Messages => self.message_view(ui),
                    View::Links => self.links_view(ui),
                    View::Devices => self.devices_view(ui),
                    View::Control => {
                        control_pane::ControlView { st: &mut self.control }.show(ui)
                    }
                    View::Satellites => self.sats_view(ui),
                    View::Video => self.video_view(ui),
                    View::Keys => self.keys_view(ui),
                }
            });
        }
        self.settings_modal(ui.ctx());
        self.remote_modal(ui.ctx());
        self.wigle_modal(ui.ctx());
        self.beacondb_modal(ui.ctx());
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
        println!("ran {el:.1}s, used {cpu:.2}s CPU = {:.0}% of one core", cpu / el as f64 * 100.0);
        crate::prof::report(std::time::Duration::from_secs_f32(el));
        self.shot_sent = true;
        ctx.send_viewport_cmd(egui::ViewportCommand::Close);
    }

    fn screenshot(&mut self, ctx: &egui::Context) {
        let Some(path) = self.shot.clone() else {
            return;
        };
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
            let buf: Vec<u8> =
                img.pixels.iter().flat_map(|p| [p.r(), p.g(), p.b(), p.a()]).collect();
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
        self.set_view(View::Chain);
    }

    /// Open a view, remembering the one being left.
    fn set_view(&mut self, v: View) {
        if v != self.view {
            self.prev_view = self.view;
            self.view = v;
        }
    }

    /// How much a view is holding, as one number that moves when something
    /// arrives in it.
    ///
    /// Cheap enough to ask for every view on every frame: each answer is a
    /// length already in hand. The spectrum and the chain answer nothing,
    /// because they are never a place traffic collects.
    fn view_mark(&self, v: View) -> u64 {
        match v {
            View::Dashboard | View::Spectrum | View::Chain => 0,
            View::Calls => self.calls.list.len() as u64,
            View::Transcript => self.transcript.log.len() as u64,
            View::Messages => self.messages.list.len() as u64,
            View::Video => self.video_seen,
            View::Map => self.map.tracks.len() as u64,
            View::Links => self.links.list.len() as u64,
            View::Devices => self.survey.rows.len() as u64,
            View::Control => self.control.list.len() as u64,
            View::Satellites => u64::from(self.sats.tracking.is_some()),
            View::Keys => self.keys.store.channels().len() as u64,
        }
    }

    /// Whether a view has taken something in since it was last looked at,
    /// which is what its tab's dot says.
    ///
    /// Not "has anything": on a busy band every list is non-empty a minute
    /// after the radio starts, so a dot meaning that is a lamp that is always
    /// lit and tells nobody anything. What is worth a glance is the view that
    /// has grown while you were somewhere else.
    fn view_live(&self, v: View) -> bool {
        self.view_mark(v) > self.view_seen[v.slot()]
    }

    /// Mark the open view as read, and follow a list that shrank down so a
    /// call forgotten and heard again still lights its tab.
    fn read_views(&mut self) {
        // Pictures come and go without a list to count, so the arrivals are
        // counted instead: a second transmission after you looked is a dot,
        // the same one still sending is not.
        // From the bus rather than from the pane: the pane's own idea of
        // whether a picture is live only moves while it is being drawn, so a
        // transmission that came and went while the spectrum was open would
        // never have been counted.
        let sending = self.radio.as_ref().is_some_and(|r| !r.status.video_inputs().is_empty());
        if sending {
            self.video_live_was = true;
        } else if std::mem::take(&mut self.video_live_was) {
            self.video_seen += 1;
        }
        for v in View::ROWS.into_iter().flatten().copied() {
            let mark = self.view_mark(v);
            let seen = &mut self.view_seen[v.slot()];
            if v == self.view || mark < *seen {
                *seen = mark;
            }
        }
    }

    /// The tabs on the strip, in order, without the dashboard when it is not
    /// wanted. The dashboard leads the top row, so leaving it out is a slice.
    fn tabs(&self) -> [&'static [View]; 2] {
        let mut rows = View::ROWS;
        if !self.dashboard {
            rows[0] = &View::ROWS[0][1..];
        }
        rows
    }

    /// Stop showing the dashboard, from its own corner or from settings. The
    /// view it was open on has to go somewhere, and that is the spectrum.
    fn hide_dashboard(&mut self) {
        self.dashboard = false;
        if self.view == View::Dashboard {
            self.set_view(View::Spectrum);
        }
        if self.prev_view == View::Dashboard {
            self.prev_view = View::Spectrum;
        }
    }

    /// The keyboard route to the views: the modifier and a digit for each,
    /// and the modifier and a backtick to swap with the last one.
    fn view_keys(&mut self, ctx: &egui::Context) {
        if ctx.egui_wants_keyboard_input() {
            return;
        }
        let back = self.prev_view;
        let tabs = self.tabs();
        let mut pick = None;
        ctx.input_mut(|i| {
            for (n, v) in tabs.into_iter().flatten().copied().enumerate() {
                let Some((key, _)) = tab_digit(n) else {
                    continue;
                };
                if i.consume_key(egui::Modifiers::COMMAND, key) {
                    pick = Some(v);
                }
            }
            if i.consume_key(egui::Modifiers::COMMAND, egui::Key::Backtick) {
                pick = Some(back);
            }
        });
        if let Some(v) = pick {
            self.set_view(v);
        }
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
        self.set_view(View::Map);
    }

    pub fn show_calls(&mut self) {
        self.set_view(View::Calls);
    }

    pub fn show_messages(&mut self) {
        self.set_view(View::Messages);
    }

    /// Open the transcript, on one conversation or on everything heard.
    pub fn show_transcript(&mut self, only: Option<common::ConversationKey>) {
        self.transcript.only = only;
        self.set_view(View::Transcript);
    }

    /// Open on the video pane, for a receiver pointed at a camera.
    pub fn show_video(&mut self) {
        self.set_view(View::Video);
    }

    pub fn show_links(&mut self) {
        self.set_view(View::Links);
    }

    pub fn show_control(&mut self) {
        self.set_view(View::Control);
    }

    pub fn show_devices(&mut self) {
        self.set_view(View::Devices);
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
        p.line_segment([Pos2::new(x0, y + dy), Pos2::new(x0 + w, y + dy)], Stroke::new(1.0, col));
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn downlink(mode: &str, what: &str) -> sats_pane::Downlink {
        sats_pane::Downlink {
            norad: 25544,
            sat: "ISS (ZARYA)".into(),
            hz: 145_800_000.0,
            mode: mode.into(),
            kind: datasets::satnogs::Mode::parse(mode),
            baud: None,
            what: what.into(),
        }
    }

    /// A bird with a voice repeater and a telemetry beacon is two channels
    /// that are otherwise identical, so the strip has to say which is which.
    #[test]
    fn a_satellite_channel_is_named_after_the_transmitter_not_the_satellite() {
        assert_eq!(
            sat_label(&downlink("FM", "Mode V/U FM voice")),
            "ISS (ZARYA):Mode V/U FM voice"
        );
        // A transmitter SatNOGS has not described is named by its mode.
        assert_eq!(sat_label(&downlink("BPSK", "")), "ISS (ZARYA):BPSK");
        // And one with neither still says something a person can tell apart.
        assert_eq!(sat_label(&downlink("", "")), "ISS (ZARYA):145.800 MHz");
    }

    /// A pass ends and its channel goes with it, or an hour of watching
    /// leaves a strip of dead channels the operator has to close by hand.
    #[test]
    fn closing_a_channel_keeps_the_listening_selection_where_it_was() {
        let mut a = app();
        channel(&mut a, 100_000.0, true, 1.0);
        channel(&mut a, 200_000.0, true, 1.0);
        channel(&mut a, 300_000.0, true, 1.0);
        let (first, third) = (a.audio.channels[0].id, a.audio.channels[2].id);
        a.audio.listening = Some(2);
        a.close_channel(first);
        assert_eq!(a.audio.channels.len(), 2);
        assert_eq!(a.audio.listening, Some(1), "the selection followed the wrong channel");
        assert_eq!(a.audio.channels[1].id, third);
        // Closing what is being listened to leaves nothing selected rather
        // than the channel that slid into its place.
        a.close_channel(third);
        assert_eq!(a.audio.listening, None);
        // And an id that is not there is not a panic.
        a.close_channel(999);
        assert_eq!(a.audio.channels.len(), 1);
    }

    /// A mode with a demodulator here gets it, a mode with a front end here
    /// gets that, and anything else gets the auto front end, which measures
    /// the channel rather than guessing at it.
    #[test]
    fn a_downlink_gets_a_demodulator_a_front_end_or_the_auto_one() {
        use datasets::satnogs::Mode as M;
        assert_eq!(sat_mode(M::parse("FM")), ChanMode::Audio(Demod::Nfm));
        assert_eq!(sat_mode(M::parse("usb")), ChanMode::Audio(Demod::Usb));
        assert_eq!(sat_mode(M::parse("CW")), ChanMode::Audio(Demod::Cw));
        // The three this receiver reads with a decoder of its own.
        assert_eq!(sat_mode(M::Lora), ChanMode::Decode("lora".into()));
        assert_eq!(sat_mode(M::Afsk), ChanMode::Decode("aprs".into()));
        assert_eq!(sat_mode(M::Dmr), ChanMode::Decode("dmr".into()));
        for digital in ["BPSK", "GMSK", "FSK", "", "LRPT"] {
            assert_eq!(sat_mode(M::parse(digital)), ChanMode::Auto, "{digital}");
        }
    }

    fn app() -> App {
        let mut a = App { center: 100_000_000.0, rate: 2_000_000.0, ..Default::default() };
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
            bandwidth_hz: None,
            label: format!("CH{id}"),
            on,
            volume,
            muted: false,
            squelch_db: None,
            agc: true,
            voice: false,
            tx: None,
            doppler: false,
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
            model: Some("Fineoffset-WHx080"),
            channel_hz: 31_250.0,
            modulation: common::Modulation::Ook,
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
            link: None,
            report: common::ReportDetail::Bare,
            identity: None,
            iq: None,
            audio: None,
            airtime: None,
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
            assert!(
                PLOT_FRAC_RANGE.contains(&a.scope.plot_frac),
                "{want} left {}",
                a.scope.plot_frac
            );
        }
        assert!(*PLOT_FRAC_RANGE.start() > 0.0 && *PLOT_FRAC_RANGE.end() < 1.0);
    }

    #[test]
    fn the_split_moves_the_boundary_the_way_the_pointer_went() {
        // The mapping the drag uses: pointer y within the pane becomes the
        // spectrum's share of it.
        let full = Rect::from_min_size(Pos2::new(0.0, 100.0), Vec2::new(1000.0, 800.0));
        let usable = full.height() - 16.0 - SPLIT_GRIP_H;
        let frac_at = |y: f32| {
            ((y - full.top() - SPLIT_GRIP_H / 2.0) / usable)
                .clamp(*PLOT_FRAC_RANGE.start(), *PLOT_FRAC_RANGE.end())
        };

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
        unknown.model = None;
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
            err: None,
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
                bandwidth_hz: None,
                label: "t".into(),
                on: true,
                volume: 0.8,
                muted: false,
                squelch_db: None,
                agc: true,
                voice: false,
                tx: None,
                doppler: false,
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

    /// The clamp follows the radio that is connected, not the dongle the
    /// numbers were written for: a HackRF reaches 6 GHz and 1 MHz, and both
    /// were unreachable while this was the RTL-SDR's range.
    #[test]
    fn retuning_stays_inside_what_the_tuner_can_reach() {
        let mut a = app();
        a.retune(1.0);
        assert_eq!(a.center, 24e6);
        a.retune(9e9);
        assert_eq!(a.center, 1766e6);

        a.reach = (1e6, 6e9);
        a.retune(1.0);
        assert_eq!(a.center, 1e6);
        a.retune(9e9);
        assert_eq!(a.center, 6e9);
    }

    /// The strip is the only route to a view by pointer, so a view missing
    /// from it is a view that cannot be opened, and two views sharing a
    /// glyph or a digit is a tab that opens the wrong one.
    #[test]
    fn every_view_has_a_tab_of_its_own() {
        let tabs: Vec<View> = View::ROWS.into_iter().flatten().copied().collect();
        assert_eq!(tabs.len(), 13);
        for v in [
            View::Dashboard,
            View::Spectrum,
            View::Chain,
            View::Map,
            View::Calls,
            View::Transcript,
            View::Messages,
            View::Links,
            View::Devices,
            View::Satellites,
            View::Video,
            View::Keys,
            View::Control,
        ] {
            assert!(tabs.contains(&v), "{} has no tab", v.label());
        }
        for (i, a) in tabs.iter().enumerate() {
            for b in &tabs[i + 1..] {
                assert!(a.icon() != b.icon(), "{} and {} share a glyph", a.label(), b.label());
            }
        }
    }

    /// The digit is where the tab is, whichever tabs are on the strip.
    /// Thirteen views and ten digits, so the last tabs go without one; what may
    /// never happen is two tabs answering to the same key.
    #[test]
    fn the_shortcuts_follow_the_strip() {
        let mut a = app();
        let with: Vec<View> = a.tabs().into_iter().flatten().copied().collect();
        assert_eq!(with.first(), Some(&View::Dashboard));
        assert_eq!(tab_digit(0).map(|(_, d)| d), Some("1"));

        a.hide_dashboard();
        let without: Vec<View> = a.tabs().into_iter().flatten().copied().collect();
        assert_eq!(without.len(), with.len() - 1);
        assert!(!without.contains(&View::Dashboard), "a hidden view keeps its tab");
        // The spectrum is back on 1, which is where it was before there was a
        // dashboard to put in front of it.
        assert_eq!(without.first(), Some(&View::Spectrum));

        let keys: Vec<_> = (0..with.len()).filter_map(tab_digit).collect();
        assert_eq!(keys.len(), 10, "ten digits for thirteen tabs");
        for (i, x) in keys.iter().enumerate() {
            for y in &keys[i + 1..] {
                assert_ne!(x.0, y.0, "two tabs answer to the same key");
            }
        }
    }

    /// Hiding the dashboard while it is open has to leave the operator
    /// somewhere, and the way back to it may not point at a view with no tab.
    #[test]
    fn hiding_the_dashboard_leaves_the_spectrum_open() {
        let mut a = app();
        assert_eq!(a.view, View::Dashboard);
        a.hide_dashboard();
        assert_eq!(a.view, View::Spectrum);
        assert_ne!(a.prev_view, View::Dashboard);
    }

    /// The call list's route into the transcript: the key travels, so the
    /// view opens on that conversation and not on the whole afternoon.
    #[test]
    fn a_call_opens_the_transcript_on_its_own_conversation() {
        let mut a = app();
        let key = common::ConversationKey::new("DMR", 435_000_000.0)
            .to(Some("9".into()))
            .from(Some("1234567".into()));
        a.read_views();
        assert!(!a.view_live(View::Transcript), "nothing has been said yet");
        assert!(!a.transcript.log.has(&key), "and so no row would offer a way in");

        for (n, text) in ["go ahead", "received, out"].into_iter().enumerate() {
            a.transcript.log.push(crate::transcripts::Utterance {
                key: key.clone(),
                at: std::time::Instant::now() + std::time::Duration::from_secs(n as u64),
                seconds: 1.0,
                text: text.into(),
                settled: true,
                confidence: -0.3,
                credible: true,
            });
        }
        assert!(a.transcript.log.has(&key));
        assert!(a.view_live(View::Transcript), "two lines arrived while elsewhere");

        a.show_transcript(Some(key.clone()));
        assert_eq!(a.view, View::Transcript);
        assert_eq!(a.transcript.only.as_ref(), Some(&key));
        assert_eq!(a.transcript.log.of(&key).len(), 2);
        a.read_views();
        assert!(!a.view_live(View::Transcript), "the view has been looked at");
    }

    /// Going back is one key, which is the whole reason the previous view is
    /// kept: a look at the map and back should not be a hunt.
    #[test]
    fn a_view_remembers_the_one_before_it() {
        let mut a = app();
        a.set_view(View::Spectrum);
        a.set_view(View::Map);
        assert_eq!(a.prev_view, View::Spectrum);
        // Choosing the view already open is not a move, or the way back
        // would point at itself.
        a.set_view(View::Map);
        assert_eq!(a.prev_view, View::Spectrum);
        a.set_view(a.prev_view);
        assert_eq!(a.view, View::Spectrum);
        assert_eq!(a.prev_view, View::Map);
    }

    /// The dot is worth its ink only while it means "this grew since you
    /// were last there". A dot for "has anything" is lit for good a minute
    /// into a busy band, which is the state this test exists to catch.
    #[test]
    fn a_tab_lights_for_what_arrived_while_you_were_elsewhere() {
        let mut a = app();
        a.read_views();
        assert!(!a.view_live(View::Links), "nothing has arrived yet");

        let heard = || survey::Device {
            id: 1,
            protocol: "ble".into(),
            ident: "aa:bb:cc:dd:ee:ff".into(),
            first_us: 0,
            last_us: 0,
            packets: 1,
            name: None,
            vendor: None,
            best_rssi_dbfs: None,
            best_lat: None,
            best_lon: None,
            center_hz: 2_440_000_000,
        };
        a.survey.rows.push(heard());
        assert!(a.view_live(View::Devices));

        a.set_view(View::Devices);
        a.read_views();
        assert!(!a.view_live(View::Devices), "the view has been looked at");

        a.survey.rows.push(heard());
        assert!(a.view_live(View::Devices), "but a second device is new again");

        // A list that was cleared must not leave its tab dark for the next
        // thing that arrives.
        a.set_view(View::Spectrum);
        a.read_views();
        a.survey.rows.clear();
        a.read_views();
        a.survey.rows.push(heard());
        assert!(a.view_live(View::Devices));

        // The dashboard, the spectrum and the chain are never a place traffic
        // collects, so they never carry one.
        assert!(!a.view_live(View::Dashboard));
        assert!(!a.view_live(View::Spectrum));
        assert!(!a.view_live(View::Chain));
    }

    #[test]
    fn fmt_hz_scales_units() {
        assert_eq!(fmt_hz(95_800_000.0), "95.8000 MHz");
        assert_eq!(fmt_hz(12_500.0), "12.5 kHz");
        assert_eq!(fmt_hz(400.0), "400 Hz");
    }
}
