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

mod agent;
mod agent_pane;
mod agent_settings;
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
mod scripts_pane;
mod settings;
mod settings_rows;
mod state;
mod strip;
mod timeline;
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
use settings_rows::{ScannerRow, mhz_field};
use state::{Channel, Logged};
use widgets::{
    Fader, Squelch, bin_hint, cog, cog_rect, help, hint, modal_title, reading, row, row_help,
};

pub struct App {
    /// What the operator has set, as one record. Every pane holds a clone of
    /// this handle and writes its setting through it; nothing keeps a second
    /// copy of a setting in a field of its own, because a second copy is a
    /// copy that can be stale.
    settings: crate::session::Settings,
    /// The record as the receiver was last told it, or `None` for a radio
    /// thread that has been told nothing yet. [`App::apply_settings`] sends
    /// what differs, so a setting cannot be stored without being applied.
    applied: Option<crate::session::Session>,
    /// Which version of the record that was, so a frame that changed nothing
    /// costs one atomic read.
    applied_rev: u64,
    /// What each view remembers. A pane is handed its own and nothing else,
    /// which is what stops one view reaching into another's business.
    scope: state::ScopeState,
    chain: state::ChainState,
    log: state::LogState,
    survey: state::SurveyState,
    map: map_pane::MapState,
    /// The .sub files this machine holds, as the panel lists them.
    scripts: scripts_pane::ScriptsState,
    /// When the `.sub` file being keyed from the scripts panel has played,
    /// so the key comes back up without the operator holding anything.
    sub_until: Option<std::time::Instant>,
    /// Where the radio transmits, or `None` for one that does not.
    tx_reach: Option<(f64, f64)>,
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
    /// The open-a-capture dialog, while it is up. It runs on its own thread
    /// so the receiver keeps painting behind it.
    picking: Option<poll_promise::Promise<Option<std::path::PathBuf>>>,
    /// The open-a-file dialog for a stage's own setting, wherever it was
    /// asked for: the chain view's inspector or the channel strip.
    pick_file: state::FilePick,
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
    view: View,
    /// What was open before it. A look at the map and back is then one key,
    /// which is the thing an operator does most often with these views.
    prev_view: View,
    /// How much each view held when it was last looked at, by
    /// [`View::slot`]. A tab's dot is on when its view has more than this.
    view_seen: [u64; View::COUNT],
    /// Video transmissions that have ended, and whether one is running.
    /// Counted because the video pane has no list to take a length of.
    video_seen: u64,
    video_live_was: bool,
    /// How far out the station position may be, in metres, when a fix said. `None`
    /// for a position typed in or taken from the country, which is a claim
    /// with no error bar rather than a perfect one.
    accuracy_m: Option<f64>,
    /// When the radio last delivered a spectrum, for noticing that it has
    /// stopped.
    last_frame: Option<std::time::Instant>,
    /// What the radio is set to, as the operator set it. The one record every
    /// route to a radio setting writes and reads; see
    /// [`crate::session::RadioSettings`].
    radio_settings: crate::session::RadioSettings,
    /// Reference correction by device label, as saved. The live figure is the
    /// one in `radio_settings`; this is where the radios not in use keep
    /// theirs, because a correction is a property of one crystal.
    ppm_by_device: std::collections::BTreeMap<String, f64>,
    /// What the dial reads above the tuner, in hertz, by device label, kept
    /// for the same reason: the dish is on one radio's cable.
    offset_by_device: std::collections::BTreeMap<String, f64>,
    /// Whether the radio has a freshly opened device that has not yet been
    /// given the settings. Set on connect and on reset, cleared once the
    /// driver has reported its controls and the settings have gone to it.
    radio_dirty: bool,
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
    /// Where the packet log is being pointed, while it is being typed. Apart
    /// from the setting so a half-written path does not move the log on every
    /// keystroke.
    log_dir_edit: String,
    feed_kind: &'static nodes::FeedKind,
    /// The station position being typed, while it is being typed. Kept apart
    /// from the real one so a half-finished latitude does not move the map.
    station_edit: Option<String>,
    /// What an agent has asked for, whether it came over MCP or from the
    /// chat in the Agent view. One queue: both front ends hold the same desk.
    agent: crossbeam_channel::Receiver<crate::agent::Ask>,
    /// The counter they put work on, handed to whatever wants to drive this
    /// receiver.
    desk: crate::agent::Desk,
    /// Whether the desk's bell has been given this window's repaint.
    desk_rung: bool,
    /// The conversation in the Agent view, and the model behind it.
    chat: crate::agent::chat::Chat,
    /// The agent on the air: the channel it answers on, and what it has to
    /// say. Off until a channel is set to transmit from it.
    air: crate::agent::channel::AgentChannel,
    /// Agents waiting for a picture of the window. Held rather than answered
    /// on the spot because egui hands the image back on a later frame.
    agent_shots: Vec<tokio::sync::oneshot::Sender<crate::agent::Reply>>,
    /// Graph edits an agent made, waiting for the rebuild that takes them:
    /// an edit that will not build is refused, and answering before the
    /// receiver has tried would be answering the wrong question.
    agent_edits: Vec<agent::PendingEdit>,
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
    /// The walk over a band: where the dial goes when it is let off the span,
    /// and what it heard on the way.
    BandWalk,
    /// The memory bank: saved channels, in groups.
    Memory,
    /// The dataset cache: what is held on disc, how old it is, and refresh.
    Data,
    /// Which model the Agent view talks to.
    Agent,
    /// Everything about where this receiver is rather than what it is doing:
    /// language, country, band plan, station position.
    App,
}

impl Settings {
    /// A dialog by the name the command line gives it.
    pub fn parse(name: &str) -> Option<Self> {
        Some(match name.trim().to_ascii_lowercase().as_str() {
            "spectrum" => Self::Spectrum,
            "waterfall" => Self::Waterfall,
            "radio" => Self::Radio,
            "log" | "packet_log" => Self::PacketLog,
            "scanners" => Self::Scanners,
            "walk" | "band_walk" => Self::BandWalk,
            "memory" => Self::Memory,
            "data" => Self::Data,
            "agent" => Self::Agent,
            "app" => Self::App,
            _ => return None,
        })
    }
}

const FFTS: [usize; 6] = [512, 1024, 2048, 4096, 8192, 16384];
/// Spectrum refresh rates in frames per second.
const REFRESH: [(&str, f32); 4] = [("10", 10.0), ("20", 20.0), ("30", 30.0), ("60", 60.0)];
/// Waterfall scroll rates in rows per second.
const SPEEDS: [(&str, f32); 5] =
    [("5", 5.0), ("10", 10.0), ("20", 20.0), ("40", 40.0), ("80", 80.0)];
/// Heatmap row rates. Slower than the waterfall by design: what is exported
/// is hours rather than the last half minute.
const HEAT_ROWS: [(&str, f32); 5] =
    [("4/s", 4.0), ("2/s", 2.0), ("1/s", 1.0), ("every 2 s", 0.5), ("every 10 s", 0.1)];
/// What the readings may take, in megabytes.
const HEAT_CAPS: [u64; 5] = [8, 32, 128, 512, 2048];

/// What the channel the scripts panel keys is called. Found by its name, so
/// keying a second file reuses the one channel rather than leaving a strip of
/// identical ones behind.
const SUB_CHANNEL: &str = "SUB";

/// Carrier held past the end of the file, so the last gap is played out
/// rather than cut off by the key coming up on the final mark.
const SUB_TAIL: std::time::Duration = std::time::Duration::from_millis(120);

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
    /// A model driving this receiver, and what it was asked.
    Agent,
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
            View::Agent => "Agent",
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
            View::Agent => Icon::Agent,
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
            View::Agent => "A model with the run of the receiver, and what you asked it",
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
        &[
            View::Map,
            View::Links,
            View::Devices,
            View::Control,
            View::Satellites,
            View::Keys,
            View::Agent,
        ],
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
        let (desk, asks) = crate::agent::Desk::new();
        Self {
            settings: crate::session::Settings::new(crate::session::Session::default()),
            applied: None,
            applied_rev: 0,
            scope: state::ScopeState::default(),
            chain: state::ChainState::default(),
            log: state::LogState::default(),
            survey: state::SurveyState::default(),
            sats: state::SatsState::default(),
            map: map_pane::MapState::default(),
            scripts: scripts_pane::ScriptsState::default(),
            sub_until: None,
            tx_reach: None,
            rt: background_runtime(),
            calls: state::CallsState::default(),
            transcript: state::TranscriptState::default(),
            // What was written before this receiver started, so the view
            // opens on last night rather than on nothing.
            messages: state::MessagesState::loaded(),
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
            picking: None,
            pick_file: state::FilePick::default(),
            device: None,
            reach: (24e6, 1766e6),
            tunable: true,
            spans: Vec::new(),
            zoom: 1,
            soak: None,
            shot: None,
            shot_after: 6.0,
            shot_at: None,
            shot_sent: false,
            autostart: false,
            view: View::Dashboard,
            prev_view: View::Spectrum,
            view_seen: [0; View::COUNT],
            video_seen: 0,
            video_live_was: false,
            accuracy_m: None,
            last_frame: None,
            radio_settings: Default::default(),
            ppm_by_device: Default::default(),
            offset_by_device: Default::default(),
            radio_dirty: false,
            feed_host: String::new(),
            remote: None,
            feed_kind: nodes::FEED_KINDS[0],
            scanner_edit: None,
            scanners: crate::scanners::Scanners::default(),
            memory: Default::default(),
            memory_group: crate::memory::UNGROUPED.into(),
            log_dir_edit: String::new(),
            station_edit: None,
            agent: asks,
            desk,
            desk_rung: false,
            chat: crate::agent::chat::Chat::default(),
            air: crate::agent::channel::AgentChannel::default(),
            agent_shots: Vec::new(),
            agent_edits: Vec::new(),
        }
    }
}

impl App {
    pub fn new(cc: &eframe::CreationContext<'_>) -> Self {
        theme::install(&cc.egui_ctx);
        crate::shutdown::install(cc.egui_ctx.clone());
        let settings = crate::session::Settings::load();
        settings.edit(apply_locale);
        let s = settings.get();
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
            radio_settings,
            ppm_by_device: s.ppm.clone(),
            offset_by_device: s.offset.clone(),
            scanners: crate::scanners::Scanners::load(),
            memory: crate::memory::Memory::load(),
            view: if s.dashboard { View::Dashboard } else { View::Spectrum },
            settings,
            ..Default::default()
        };
        app.map.map.layers.restore(&s.map_layers);
        app.scope.restore(&s.view, s.fft);
        app.scope.db_center = s.center;
        app.scope.wf_center = s.center;
        // The field shows where the log would go, not where it is going: an
        // empty box beside a switch nobody has thrown says nothing.
        app.log_dir_edit = match s.log_dir.is_empty() {
            true => crate::packetlog::PacketLog::default_dir()
                .map(|d| d.display().to_string())
                .unwrap_or_default(),
            false => s.log_dir.clone(),
        };
        crate::beacondb::start();
        app.radio_dirty = true;
        // Everything the record reaches outside the radio thread: the GPS
        // source, the beaconDB lookup and what the dataset cache is asked
        // from drawing code. Before the connect and whether or not one
        // happens, since a receiver with nothing plugged in still draws a map
        // and still downloads a catalogue.
        app.apply_settings();
        // What was changed about the graph, if anything was. Applied
        // whether or not manual mode is on: the mode only says whether the
        // graph can be edited now.
        if let Some((edits, places)) = crate::patch::Edits::load() {
            app.chain.edits = edits;
            app.chain.edit.pos =
                places.iter().map(|(k, (x, y))| (*k, egui::Pos2::new(*x, *y))).collect();
            app.chain.places = places;
        }
        // What was already there when the window opened did not arrive while
        // you were somewhere else, so no tab lights for it. Without this the
        // keys tab was lit on every start for anybody with a key saved: its
        // list is read off disk and is the only one that is not empty on the
        // first frame.
        app.forget_what_was_already_here();
        app.connect(&cc.egui_ctx);
        app
    }

    /// One setting, read off the record.
    fn setting<R>(&self, f: impl FnOnce(&crate::session::Session) -> R) -> R {
        self.settings.read(f)
    }

    /// Where the tuner is pointed and what the radio is set to, which the
    /// dial and the strip change directly rather than through a switch.
    ///
    /// Put back into the record once a frame instead of on every drag, since
    /// this is the part of the settings the interface owns rather than the
    /// operator's switches.
    fn sync_settings(&mut self) {
        let rs = self.radio_settings.clone();
        let device = self.device.as_ref().map(|d| d.label.clone());
        let (center, rate, zoom, fft) = (self.center, self.rate, self.zoom, self.scope.fft);
        let prefs = self.scope.prefs();
        let layers = self.map.map.layers.saved();
        let manual = self.chain.edit.manual;
        let streams: Vec<(String, String)> =
            crate::devices::streams().into_iter().map(|r| (r.addr, r.label)).collect();
        let ppm = self.ppm_by_device.clone();
        let offset = self.offset_by_device.clone();
        self.settings.edit(|s| {
            s.device = device;
            s.center = center;
            s.rate = rate * zoom.max(1) as f64;
            s.zoom = zoom;
            s.fft = fft;
            s.gains = rs.gains;
            s.toggles = rs.toggles;
            s.choices = rs.choices;
            s.tx_gain_db = rs.tx_gain_db;
            s.ppm = ppm;
            s.offset = offset;
            s.language = crate::i18n::language().code().to_string();
            s.band_plan = crate::bands::plan().id().to_string();
            s.view = prefs;
            s.streams = streams;
            s.map_layers = layers;
            s.manual_chain = manual;
        });
    }

    /// Put the record into effect: the one path from a setting to the thing
    /// it controls.
    ///
    /// Run whenever the record has moved, and in full for every radio thread
    /// that has been told nothing yet, which is what a fresh one is: a thread
    /// builds its graph from its own defaults, so a source stopped and
    /// started came back decoding a band nobody asked for and writing nothing
    /// the operator had switched on.
    ///
    /// Only what differs is sent. Dragging the dial changes the record sixty
    /// times a second, and a feed reconnected on each of those frames would
    /// be worse than the fault this replaces.
    fn apply_settings(&mut self) {
        self.applied_rev = self.settings.revision();
        let was = self.applied.take();
        // A feed with half an account typed into it is off, whatever the
        // switch says. Written back rather than only obeyed, so the switch on
        // screen says what the receiver is doing.
        self.settings.edit(|s| {
            s.wigle_on = s.wigle_on && s.wigle_account().is_complete();
            s.ha_on = s.ha_on && s.publish().broker.is_complete();
        });
        let now = self.settings.get();
        for c in settings_cmds(&now, was.as_ref()) {
            self.send(c);
        }
        let all = was.is_none();
        let blank = crate::session::Session::default();
        let before = was.as_ref().unwrap_or(&blank);
        if all || (now.survey_on, &now.survey_path) != (before.survey_on, &before.survey_path) {
            // The pane reads the survey through a connection of its own,
            // which does not exist until the radio thread has made the file.
            self.survey.db = None;
            self.survey.refreshed = None;
        }
        // Read from where there is no record to consult: the reader runs
        // whether or not a radio does, and the datasets are asked from
        // drawing code.
        if all || now.gps != before.gps {
            crate::station::set_source(now.gps_source());
        }
        if all || now.beacondb_lookup != before.beacondb_lookup {
            // The lookup answers the map rather than the graph, so it is set
            // here rather than sent.
            crate::beacondb::set_lookup(now.beacondb_lookup);
        }
        if all || now.country != before.country {
            crate::data::set_country(&now.country);
        }
        if all || now.opencellid_token != before.opencellid_token {
            crate::data::set_opencellid_token(&now.opencellid_token);
        }
        if all
            || (&now.spacetrack_identity, &now.spacetrack_password)
                != (&before.spacetrack_identity, &before.spacetrack_password)
        {
            datasets::spacetrack::set_account(Some(datasets::spacetrack::Account {
                identity: now.spacetrack_identity.clone(),
                password: now.spacetrack_password.clone(),
            }));
        }
        self.applied = Some(now);
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
        self.send(Cmd::Offset(want.offset));
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
    }

    /// Tell the tracker where the receiver is, so a single position frame
    /// resolves instead of waiting for a matching pair.
    pub fn set_location(&mut self, lat: f64, lon: f64) {
        self.accuracy_m = None;
        self.settings.edit(|s| s.location = Some((lat, lon)));
    }

    /// Start or stop recording the survey, or point it at another file.
    pub fn set_survey(&mut self, off: bool, path: Option<std::path::PathBuf>) {
        self.settings.edit(|s| {
            s.survey_on = !off;
            if let Some(p) = path {
                s.survey_path = p.display().to_string();
            }
        });
    }

    /// Read the receiver's own position from a named GPS, or `None` to go
    /// back to the local gpsd the reader finds on its own.
    pub fn set_gps(&mut self, transport: Option<gps::Transport>) {
        let named = transport.map(|t| t.to_string()).unwrap_or_default();
        self.settings.edit(|s| s.gps = named);
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
            .setting(|s| s.location)
            .is_none_or(|(lat, lon)| (lat - f.lat).abs() > 1e-5 || (lon - f.lon).abs() > 1e-5);
        if moved {
            self.settings.edit(|s| s.location = Some((f.lat, f.lon)));
            // The box in settings shows the position; a stale string in it
            // would sit there claiming the receiver had not moved.
            self.station_edit = None;
        }
    }

    /// Turn the packet log off, or point it somewhere other than the default.
    pub fn set_packet_log(&mut self, off: bool, dir: Option<std::path::PathBuf>) {
        self.settings.edit(|s| {
            s.packet_log_on = !off;
            if let Some(d) = dir {
                s.log_dir = d.display().to_string();
            }
        });
        self.log_dir_edit = self.setting(|s| s.log_dir.clone());
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
        self.settings.edit(|s| s.capture_on = on);
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
        self.radio_dirty = true;
        if let Some(d) = self.device.as_ref() {
            self.ppm_by_device.insert(d.label.clone(), ppm);
        }
    }

    /// Tell the receiver what sits between the aerial and the radio in use,
    /// and keep the figure against that radio: an LNB is on one cable.
    pub fn set_offset(&mut self, hz: f64) {
        // The dial reads on the aerial's side of the converter, so it moves
        // with the offset: the radio stays where it was and the number over
        // it changes, rather than the dial staying put and the radio being
        // asked for a frequency that is now nine gigahertz away.
        let moved = hz - self.radio_settings.offset;
        self.radio_settings.offset = hz;
        // Applied now if a radio is running, and again when one reports its
        // controls: an offset typed before the radio was started otherwise
        // stayed in the settings pane, the dial read the dish's frequency,
        // and the tuner was asked for it whole.
        self.radio_dirty = true;
        self.reach = (self.reach.0 + moved, self.reach.1 + moved);
        self.center = (self.center + moved).max(0.0);
        if let Some(d) = self.device.as_ref() {
            self.offset_by_device.insert(d.label.clone(), hz);
        }
    }

    /// Start on the radio whose label contains `want`, for when several are
    /// plugged in and the saved one is not the one wanted.
    /// Publish every device heard to this broker, from the command line.
    pub fn publish_to(&mut self, publish: nodes::Publish) {
        self.settings.edit(|s| {
            s.ha_host = publish.broker.host.clone();
            s.ha_port = publish.broker.port.to_string();
            s.ha_user = publish.broker.username.clone();
            s.ha_password = publish.broker.password.clone();
            s.ha_prefix = publish.broker.prefix.clone();
            s.ha_topic = publish.broker.topic.clone();
            s.ha_spaces = publish.spaces.clone();
            s.ha_on = true;
        });
    }

    /// Start the radio without waiting for the play button, which is what a
    /// capture being replayed usually wants and what a screenshot needs.
    /// Open with a settings dialog up, for a screenshot of it.
    pub fn open_settings(&mut self, which: Settings) {
        self.open = Some(which);
    }

    pub fn start_on_open(&mut self) {
        self.autostart = true;
    }

    /// Serve MCP on `addr`, so an agent drives this receiver rather than one
    /// of its own. The desk is the one the chat uses, so a tool call from
    /// either arrives the same way.
    #[cfg(feature = "mcp")]
    pub fn serve_mcp(&mut self, addr: std::net::SocketAddr) -> anyhow::Result<()> {
        crate::agent::serve(addr, self.rt.handle(), self.desk.clone())
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
            self.radio_settings.offset,
            self.scope.fft,
            move || c.request_repaint(),
        ));
        for cmd in self.startup_cmds() {
            self.send(cmd);
        }
        // The queue the agent speaks into, handed over once: a channel set to
        // transmit from the agent reads it when it is keyed.
        self.send(Cmd::Voice(self.air.speaker()));
        // A fresh thread has been told nothing, so the whole record goes to
        // it rather than whatever has changed since the last one.
        self.applied = None;
        self.apply_settings();
        // Whatever the radio was set to has to be pushed at it again: a new
        // thread means a freshly opened device at its defaults. Start and
        // reset are the same path through here.
        self.radio_dirty = true;
        self.reset_waterfall();
    }

    /// Everything a freshly started radio thread has to be told.
    ///
    /// A thread builds its graph from its own defaults: no channels, the
    /// scanner table running, the DC blocker in, nothing being recorded and
    /// no picture watched. Every one of those is the operator's, so a source
    /// stopped and started came back decoding a band nobody asked for and
    /// without the channel that was on the strip a second earlier.
    ///
    /// What is not a setting but still has to be sent: the channels, the
    /// subscriptions, the recorder and the graph's edits. Everything the
    /// operator set goes with [`App::apply_settings`], which the connect
    /// runs in full for a thread that has been told nothing.
    fn startup_cmds(&self) -> Vec<Cmd> {
        let mut cmds = vec![
            // The channels first: everything below is about a graph that
            // has them in it.
            Cmd::Channels(self.channel_specs()),
            Cmd::Zoom(self.zoom),
            Cmd::CallSubs(self.calls.subs.clone()),
            Cmd::WatchVideo(self.video.rules()),
            Cmd::Record(self.record_dir.clone()),
        ];
        // Last, so the edited graph is the first one that settles rather
        // than the automatic one rebuilt a moment later.
        if !self.chain.edits.is_empty() {
            cmds.push(Cmd::Edits(self.chain.edits.clone()));
        }
        cmds
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
        self.radio_settings.offset = self.offset_by_device.get(&e.label).copied().unwrap_or(0.0);
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
        let fault = radio.status.error.lock().take().or_else(|| radio.status.refused.lock().take());
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
            self.tx_reach = c.tx_reach;
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
            self.chain.take_edits_from_the_running_graph();
        }
        self.chain.flush_edits(false);
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
    ///
    /// Nothing at all when it is hidden: the panel is the strip, so putting
    /// it away is not drawing it. What it was set to is still applied, since
    /// every level lives on its node rather than in the panel.
    fn strip_view(&mut self, ui: &mut egui::Ui) {
        if !self.settings.read(|s| s.strip) {
            return;
        }
        let acts = strip::Strip {
            st: &mut self.audio,
            radio: self.radio.as_ref(),
            center: self.center,
            rate: self.rate,
            memory: &mut self.memory,
            memory_group: &mut self.memory_group,
            acts: Vec::new(),
            cmds: &mut self.cmds,
            chain: self.chain.topo.as_ref(),
            files: &mut self.pick_file,
            air: &self.air,
            air_fault: self.chat.config.voice_fault(),
            settings: &self.settings,
        }
        .show(ui);
        for a in acts {
            match a {
                strip::Action::Channels => self.send_channels(),
                strip::Action::Open(w) => self.open = Some(w),
            }
        }
    }

    /// Draw the scripts panel, and act on what it asked for.
    ///
    /// Nothing at all when it is hidden, like the strip: the panel is the
    /// list, so putting it away is not drawing it.
    fn scripts_view(&mut self, ui: &mut egui::Ui) {
        if !self.settings.read(|s| s.scripts) {
            return;
        }
        let acts = scripts_pane::Scripts {
            st: &mut self.scripts,
            tx_range: self.tx_reach,
            keyed: self.audio.keying.at.is_some() || self.sub_until.is_some(),
            acts: Vec::new(),
        }
        .show(ui);
        for a in acts {
            match a {
                scripts_pane::Action::Hide => self.settings.edit(|s| s.scripts = false),
                scripts_pane::Action::Open(w) => self.open = Some(w),
                scripts_pane::Action::Transmit(f) => self.transmit_sub(f),
            }
        }
    }

    /// Key a `.sub` file, at the frequency the file names.
    ///
    /// Everything the operator would otherwise do by hand: the dial to the
    /// file's frequency, a channel there set to transmit the file, the file
    /// itself to the radio thread, and the key down. The key comes back up
    /// on its own in [`Self::sub_key`], because the length of the over is
    /// the length of the file and nothing else decides it.
    fn transmit_sub(&mut self, f: crate::radio::SubFile) {
        let hz = f.file.frequency as f64;
        // Within the span, or the dial moves: a channel outside what the
        // radio is sampling has nothing to key through.
        if (hz - self.center).abs() > self.rate / 2.0 {
            self.retune(hz);
        }
        // One channel for this, reused: keying a file twice must not leave a
        // strip of identical channels behind. Its mode is NFM because the
        // chain reads the pulses through the OOK keyer whatever the channel
        // says, and a mode that cannot transmit would refuse the key.
        let id = match self.audio.channels.iter_mut().find(|c| c.label == SUB_CHANNEL) {
            Some(c) => {
                c.freq = hz;
                c.id
            }
            None => {
                let id = self.audio.next_id as u64;
                self.audio.next_id += 1;
                // Auto on the receive side: a channel replaying a remote is
                // pointed at a band of remotes, so what it hears is worth
                // classifying and decoding. Transmitting is not the mode's
                // question here, because a `.sub` file is keyed carrier
                // whatever the channel listens in; see `tx_mode_for`.
                let mut c = fresh(id, hz, ChanMode::Auto, Some(SUB_CHANNEL.into()));
                // On, because the radio is only sent the channels that are:
                // an off channel is not in the list and the key came back
                // "there is no such channel to key". Muted instead, since
                // nobody wants a remote's pulses through the speaker.
                c.on = true;
                c.muted = true;
                self.audio.channels.push(c);
                id
            }
        };
        if let Some(c) = self.audio.channels.iter_mut().find(|c| c.id == id) {
            c.tx = Some(crate::radio::TxSpec {
                source: crate::radio::TxSource::Sub,
                shift_hz: 0.0,
                ..Default::default()
            });
        }
        self.send_channels();
        // The parsed file, then the key. The strip shows what is loaded, so
        // it is the interface's copy as well as the radio's.
        self.audio.sub_pick.file = Some(f.clone());
        let over = f.file.duration();
        self.cmds.push(Cmd::SubFile(Some(f)));
        self.cmds.push(Cmd::Key(Some(id)));
        // A tail beyond the file: the last gap is silence the transmitter
        // still has to play out before the carrier drops.
        self.sub_until = Some(std::time::Instant::now() + over + SUB_TAIL);
    }

    /// Let the key go once the file has played.
    fn sub_key(&mut self, ctx: &egui::Context) {
        let Some(until) = self.sub_until else { return };
        if std::time::Instant::now() < until {
            // Nothing else may be drawing, and a carrier that stays up
            // because the window went idle is the worst fault this has.
            ctx.request_repaint_after(std::time::Duration::from_millis(50));
            return;
        }
        self.sub_until = None;
        self.cmds.push(Cmd::Key(None));
    }

    /// Draw the chain view over the graph it edits.
    fn chain_view(&mut self, ui: &mut egui::Ui) {
        chain_pane::Chain { st: &mut self.chain, cmds: &mut self.cmds, files: &mut self.pick_file }
            .show(ui);
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
        let home = self.setting(|s| s.location);
        let place = map_pane::Map {
            st: &mut self.map,
            home,
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
        let decode_on = self.setting(|s| s.decode_on);
        let show_unknown = self.setting(|s| s.list_unknown);
        let log_dir = self.setting(|s| s.log_path());
        let acts = packets::Log {
            st: &mut self.log,
            show_unknown,
            log_dir,
            radio: self.radio.as_ref(),
            scanners: &self.scanners,
            center: self.center,
            rate: self.rate,
            decode_on,
            cmds: &mut self.cmds,
            acts: Vec::new(),
        }
        .show(ui);
        for a in acts {
            match a {
                packets::Action::Decode(on) => self.settings.edit(|s| s.decode_on = on),
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
            settings: &self.settings,
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
        let (frame, inputs, saved, muxes) = match self.radio.as_ref() {
            Some(r) => (
                r.status.video(),
                r.status.video_inputs(),
                r.status.pictures(),
                r.status.multiplexes(),
            ),
            None => (None, Vec::new(), Vec::new(), Vec::new()),
        };
        video_pane::VideoPane {
            st: &mut self.video,
            frame,
            inputs,
            saved,
            muxes,
            cmds: &mut self.cmds,
        }
        .show(ui);
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
        let act = transcript_pane::Transcript {
            st: &mut self.transcript,
            engine,
            cmds: &mut self.cmds,
            settings: &self.settings,
        }
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

    /// The agent on the air: what was said on its channel, and whether it is
    /// its turn to talk.
    ///
    /// Every frame and whichever view is open, for the reason the transcript
    /// is read every frame: a receiver that only answers while somebody is
    /// looking at the Agent pane is not a station.
    fn agent_air(&mut self) {
        use crate::agent::channel::Move;
        let agent = self.agent_tx_channel();
        match (agent, self.air.on) {
            (None, None) => return,
            // The channel was closed or given back to a person mid-over.
            (None, Some(_)) => {
                self.air.on = None;
                if let Some(Move::Unkey) = self.air.stand_down() {
                    self.audio.keying = Default::default();
                    self.send(Cmd::Key(None));
                }
                return;
            }
            (Some((id, _)), _) => self.air.on = Some(id),
        }
        let Some((id, freq)) = agent else { return };

        // What was said on that channel, by frequency: the transcript files
        // speech under the conversation it was heard in, and an analogue
        // channel's conversation is its frequency.
        let heard: Vec<(std::time::Instant, Option<String>, String)> = self
            .transcript
            .log
            .recent(16)
            .into_iter()
            .filter(|u| {
                u.settled
                    && u.credible
                    && (u.key.channel_hz as f64 - freq).abs() < common::CHANNEL_MATCH_HZ
            })
            // Who said it, where the radio said so: an analogue PTT-ID or a
            // digital call's own caller. The agent is told, so it can answer
            // a station by number and know when the other side changed.
            .map(|u| (u.at, u.key.from.clone(), u.text.clone()))
            .collect();
        let config = self.chat.config.clone();
        for (at, from, text) in heard {
            let desk = self.desk.clone();
            self.air.heard(&config, &desk, self.rt.handle(), at, from.as_deref(), &text);
        }

        let busy = self
            .radio
            .as_ref()
            .map(|r| r.status.channel_states())
            .unwrap_or_default()
            .iter()
            .any(|s| s.id == id && s.squelch_open);
        match self.air.poll(&config, std::time::Instant::now(), busy) {
            Some(Move::Key(id)) => {
                self.audio.keying = state::Keying { at: Some(id), latched: true };
                self.send(Cmd::Key(Some(id)));
            }
            Some(Move::Unkey) => {
                self.audio.keying = Default::default();
                self.send(Cmd::Key(None));
            }
            None => {}
        }
    }

    /// The channel the agent answers on, and where it is: the one whose
    /// transmit source is the agent.
    ///
    /// Read off the strip rather than held, because giving a channel to the
    /// agent is a transmit setting like any other and can be changed on the
    /// strip, in a bank recall or by a tool.
    pub(crate) fn agent_tx_channel(&self) -> Option<(u64, f64)> {
        self.audio
            .channels
            .iter()
            .find(|c| c.tx.is_some_and(|t| t.source == crate::radio::TxSource::Agent))
            .map(|c| (c.id, c.freq))
    }

    /// Draw the conversation, then do what it asked for.
    fn agent_view(&mut self, ui: &mut egui::Ui) {
        let running = self.radio.is_some();
        let air_fault = self.chat.config.voice_fault();
        let wake = self.chat.config.wake.clone();
        let act = agent_pane::AgentView {
            chat: &mut self.chat,
            running,
            air: &self.air,
            voice: crate::agent::voice::health(),
            air_fault,
            wake,
        }
        .show(ui);
        match act {
            Some(agent_pane::Action::Ask(text)) => {
                let desk = self.desk.clone();
                self.chat.ask(&text, desk, self.rt.handle());
            }
            Some(agent_pane::Action::Clear) => self.chat.clear(),
            Some(agent_pane::Action::Interrupt) => self.chat.interrupt(),
            // The key follows the queue, so this is the whole of it: the
            // samples go back into the speaker and the channel is taken when
            // it next goes quiet, exactly as the first over was.
            Some(agent_pane::Action::SayAgain(nth)) => {
                self.air.repeat(nth);
            }
            Some(agent_pane::Action::Settings) => self.open = Some(Settings::Agent),
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
        let settings = self.settings.clone();
        let act = devices_pane::Devices {
            st: &mut self.survey,
            settings,
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
            Some(devices_pane::Action::Record(on)) => self.set_survey(!on, None),
            Some(devices_pane::Action::Export) => self.export_survey(),
            Some(devices_pane::Action::Wigle) => self.open_wigle(),
            Some(devices_pane::Action::BeaconDb) => self.survey.beacondb.open = true,
            Some(devices_pane::Action::HomeAssistant) => self.open_homeassistant(),
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
        let Some(path) = self.setting(|s| s.survey_file()) else {
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

    /// Fill a feed's dialog from the record, and open it.
    ///
    /// The dialog's fields are a draft rather than the setting: an account
    /// name reaches the record when APPLY is pressed, because a broker
    /// reconnected on every keystroke would spend a typed hostname's worth of
    /// connections failing.
    fn open_wigle(&mut self) {
        let s = self.settings.get();
        let w = &mut self.survey.wigle;
        (w.name, w.token, w.donate, w.on, w.open) =
            (s.wigle_name, s.wigle_token, s.wigle_donate, s.wigle_on, true);
    }

    fn open_homeassistant(&mut self) {
        let s = self.settings.get();
        let h = &mut self.survey.homeassistant;
        h.host = s.ha_host;
        h.port = s.ha_port;
        h.username = s.ha_user;
        h.password = s.ha_password;
        h.prefix = s.ha_prefix;
        h.topic = s.ha_topic;
        h.spaces = s.ha_spaces;
        h.on = s.ha_on;
        h.buses = s.ha_buses;
        h.open = true;
    }

    /// Put the wigle.net dialog's draft into the record.
    fn apply_wigle(&mut self) {
        let w = &self.survey.wigle;
        let (name, token, donate, on) = (w.name.clone(), w.token.clone(), w.donate, w.on);
        self.settings.edit(|s| {
            s.wigle_name = name;
            s.wigle_token = token;
            s.wigle_donate = donate;
            s.wigle_on = on;
        });
        // An account with half of it typed cannot upload, so the switch has
        // to come back off before the dialog draws it as on.
        self.apply_settings();
        self.survey.wigle.on = self.setting(|s| s.wigle_on);
    }

    /// Put the Home Assistant dialog's draft into the record.
    fn apply_homeassistant(&mut self) {
        let h = &self.survey.homeassistant;
        let (host, port, user) = (h.host.clone(), h.port.clone(), h.username.clone());
        let (password, prefix, topic) = (h.password.clone(), h.prefix.clone(), h.topic.clone());
        let (spaces, on, buses) = (h.spaces.clone(), h.on, h.buses);
        self.settings.edit(|s| {
            s.ha_host = host;
            s.ha_port = port;
            s.ha_user = user;
            s.ha_password = password;
            s.ha_prefix = prefix;
            s.ha_topic = topic;
            s.ha_spaces = spaces;
            s.ha_on = on;
            s.ha_buses = buses;
        });
        self.apply_settings();
        self.survey.homeassistant.on = self.setting(|s| s.ha_on);
    }

    /// Write the survey out as WiGLE CSV, beside the survey file.
    fn export_survey(&mut self) {
        let (Some(path), Some(db)) = (self.setting(|s| s.survey_file()), self.survey.db.as_ref())
        else {
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
        let decode_on = self.setting(|s| s.decode_on);
        let acts = dashboard_pane::Dashboard {
            radio: self.radio.as_ref(),
            device: self.device.as_ref().map(|d| d.label.as_str()),
            center: self.center,
            rate: self.rate,
            zoom: self.zoom,
            decode_on,
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
        let home = self.setting(|s| s.location);
        let acts = sats_pane::Sats { st: &mut self.sats, home }.show(ui);
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
        let (lat, lon) = self.setting(|s| s.location)?;
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
        let decode_on = self.setting(|s| s.decode_on);
        let acts = scope::Scope {
            st: &mut self.scope,
            channels: &mut self.audio.channels,
            listening: self.audio.listening,
            center: self.center,
            rate: self.rate,
            radio: self.radio.as_ref(),
            scanners: &self.scanners,
            patch: &self.chain.patch,
            decode_on,
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
        let id = self.audio.next_id;
        self.audio.next_id += 1;
        self.audio.channels.push(recalled(id as u64, s));
        self.audio.listening = Some(self.audio.channels.len() - 1);
        self.send_channels();
    }

    /// A channel on the strip, tuned to `freq` and doing `mode` with it.
    fn push_channel(&mut self, freq: f64, mode: ChanMode, label: Option<String>) {
        let id = self.audio.next_id;
        self.audio.next_id += 1;
        self.audio.channels.push(fresh(id as u64, freq, mode, label));
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
        specs_of(&self.audio.channels, self.center)
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
/// The record as commands to the receiver: what changed since `was`, or the
/// whole of it for a thread that has been told nothing.
///
/// A free function so a test can read the whole list. Every setting the radio
/// thread has to know is named here exactly once, which is what stops a
/// switch from being stored without being applied.
fn settings_cmds(now: &crate::session::Session, was: Option<&crate::session::Session>) -> Vec<Cmd> {
    let all = was.is_none();
    let blank = crate::session::Session::default();
    let was = was.unwrap_or(&blank);
    let mut cmds = Vec::new();
    let mut when = |differs: bool, c: Cmd| {
        if all || differs {
            cmds.push(c);
        }
    };
    when(now.decode_on != was.decode_on, Cmd::Decode(now.decode_on));
    when(now.dc_block != was.dc_block, Cmd::DcBlock(now.dc_block));
    when(now.manual_chain != was.manual_chain, Cmd::Manual(now.manual_chain));
    // The spectrum. The levels are not sent: each lives on its node and comes
    // back as an edit with the rest of the graph.
    when(now.view.refresh != was.view.refresh, Cmd::Refresh(now.view.refresh));
    when(now.view.smoothing != was.view.smoothing, Cmd::Smoothing(now.view.smoothing));
    // What writes to disk.
    when(now.log_cap_mb != was.log_cap_mb, Cmd::PacketLogCap(now.log_cap_mb.map(|mb| mb << 20)));
    when(
        now.capture_cap_mb != was.capture_cap_mb,
        Cmd::CaptureCap(now.capture_cap_mb.map(|mb| mb << 20).unwrap_or(0)),
    );
    when(now.capture_on != was.capture_on, Cmd::CaptureIq(now.capture_on));
    when(now.capture_arm != was.capture_arm, Cmd::CaptureTrigger(now.capture_arm));
    when(
        (now.packet_log_on, &now.log_dir) != (was.packet_log_on, &was.log_dir),
        Cmd::PacketLog(now.log_path()),
    );
    when(
        (now.survey_on, &now.survey_path) != (was.survey_on, &was.survey_path),
        Cmd::Survey(now.survey_file()),
    );
    when(
        (now.calls_on, &now.calls_dir) != (was.calls_on, &was.calls_dir),
        Cmd::RecordCalls(now.calls_path()),
    );
    when(
        (now.transcribe_on, &now.transcribe_model, &now.transcribe_device)
            != (was.transcribe_on, &was.transcribe_model, &was.transcribe_device),
        Cmd::Transcribe {
            on: now.transcribe_on,
            model: now.transcribe_model.clone(),
            device: now.transcribe_device.clone(),
        },
    );
    when(now.gps != was.gps, Cmd::Gps(now.gps_source()));
    when(now.feeds != was.feeds, Cmd::Feeds(now.feeds.clone()));
    if let Some((lat, lon)) = now.location {
        when(now.location != was.location, Cmd::Location(lat, lon));
    }
    // A thread opens the default speaker on its own, so silence is not a
    // setting to send.
    if !now.audio_out.is_empty() || !now.audio_in.is_empty() {
        when(
            (&now.audio_out, &now.audio_in) != (&was.audio_out, &was.audio_in),
            Cmd::Audio { out: now.audio_out.clone(), input: now.audio_in.clone() },
        );
    }
    // The uploads carry an account or a broker rather than a switch, so each
    // goes again when anything about it is typed.
    let account = now.wigle_account();
    when(
        now.wigle_on != was.wigle_on || account != was.wigle_account(),
        Cmd::Wigle(now.wigle_on.then(|| account.clone())),
    );
    when(now.beacondb_on != was.beacondb_on, Cmd::BeaconDb(now.beacondb_on));
    when(now.band_scan() != was.band_scan(), Cmd::BandScan(now.band_scan()));
    when(now.heat_plan() != was.heat_plan(), Cmd::Heatmap(now.heat_plan()));
    let publish = now.publish();
    when(
        now.ha_on != was.ha_on || publish != was.publish(),
        Cmd::HomeAssistant(now.ha_on.then(|| publish.clone())),
    );
    // The transform size is the one thing not swept in for a fresh thread:
    // the radio is started at the size the record holds, so sending it would
    // be a rebuild to the size it already is.
    if now.fft != was.fft {
        cmds.push(Cmd::Fft(now.fft));
    }
    cmds
}

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

/// A channel nobody has touched yet.
fn fresh(id: u64, freq: f64, mode: ChanMode, label: Option<String>) -> Channel {
    Channel {
        id,
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
    }
}

/// A channel out of the memory bank.
///
/// Everything the bank kept comes with it, the transmit side included: a
/// repeater recalled without its shift is a channel working simplex on the
/// repeater's output, where nobody is listening. Levels and squelch are the
/// strip's and are set against the signal on the day.
fn recalled(id: u64, s: &crate::memory::Saved) -> Channel {
    let label = match s.label.trim().is_empty() {
        true => None,
        false => Some(s.label.clone()),
    };
    Channel { bandwidth_hz: s.bandwidth_hz, tx: s.tx, ..fresh(id, s.freq, s.mode.clone(), label) }
}

/// The whole channel list as the radio takes it: offsets from wherever the
/// receiver is now, and nothing the strip keeps for itself.
fn specs_of(channels: &[Channel], center: f64) -> Vec<ChannelSpec> {
    channels
        .iter()
        .filter(|c| c.on)
        .map(|c| ChannelSpec {
            id: c.id,
            label: c.label.clone(),
            offset_hz: c.freq - center,
            mode: c.mode.clone(),
            bandwidth_hz: c.bandwidth_hz,
            squelch_db: c.squelch_db,
            agc: c.agc,
            voice: c.voice,
            // Only what an operator changed about transmitting. Whether the
            // channel transmits at all is its mode's question, asked by the
            // receiver: see `ChannelSpec::spec_to_transmit`.
            tx: c.tx,
        })
        .collect()
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
        // The picture the weather birds send, which is the one satellite
        // mode with a decoder of its own here.
        M::Apt => front("apt"),
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
        // Before the panes draw, so what an agent changed is on the screen in
        // the same frame it asked for it and what it reads back is what the
        // frame is about to show.
        self.agent_serve(ui.ctx());
        // Whatever the model has said since the last frame, whichever view is
        // open: a conversation that only advances while its pane is showing
        // is one that stops when an operator looks at the spectrum.
        self.chat.poll();
        self.agent_air();
        self.screenshot(ui.ctx());
        // Who is talking and what they said, every frame and whichever view
        // is open. Both used to be read only under --soak, so the call list
        // and the transcript filled in a soak run and stayed empty in use.
        self.poll_capture(ui.ctx());
        self.pick_file.poll(&mut self.cmds);
        // The `.sub` dialog lands its file as a command like any other.
        self.audio.sub_pick.poll(&mut self.cmds);
        // And the save dialog writes the file it was given a name for.
        self.log.sub_save.poll();
        self.read_heard();
        self.read_said();
        self.soak_check(ui.ctx());
        // Read once a frame rather than where it is drawn: the pane's button
        // and the modal both show it, and only one of them is ever open.
        self.survey.wigle.status = self.radio.as_ref().and_then(|r| r.status.wigle.lock().clone());
        self.survey.beacondb.status =
            self.radio.as_ref().and_then(|r| r.status.beacondb.lock().clone());
        self.survey.homeassistant.status =
            self.radio.as_ref().and_then(|r| r.status.homeassistant.lock().clone());
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
            let _s = tracing::info_span!("scripts").entered();
            self.scripts_view(ui);
        }
        self.sub_key(ui.ctx());
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
                    View::Control => control_pane::ControlView { st: &mut self.control }.show(ui),
                    View::Satellites => self.sats_view(ui),
                    View::Video => self.video_view(ui),
                    View::Keys => self.keys_view(ui),
                    View::Agent => self.agent_view(ui),
                }
            });
        }
        self.settings_modal(ui.ctx());
        self.remote_modal(ui.ctx());
        self.wigle_modal(ui.ctx());
        self.beacondb_modal(ui.ctx());
        self.homeassistant_modal(ui.ctx());
        self.flush_cmds();
        self.restore_radio_settings();
        // The dial and the strip put what they changed back into the record,
        // then anything that moved reaches the receiver and the disc. One
        // order, once a frame, whichever pane was drawn.
        self.sync_settings();
        if self.applied_rev != self.settings.revision() {
            self.apply_settings();
        }
        self.settings.flush(false);
    }

    fn on_exit(&mut self) {
        // The periodic write is debounced, so a change made in the last
        // couple of seconds before quitting is still only in memory.
        self.sync_settings();
        self.settings.flush(true);
        self.chain.flush_edits(true);
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
            View::Agent => self.chat.turns() as u64,
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

    /// Take every view as read, whatever is in it.
    ///
    /// For the moment the window opens. A dot means a view grew while you
    /// were elsewhere, and you were not elsewhere before the program was
    /// running: a key saved last week is not news this morning.
    fn forget_what_was_already_here(&mut self) {
        for v in View::ROWS.into_iter().flatten().copied() {
            self.view_seen[v.slot()] = self.view_mark(v);
        }
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
        if !self.setting(|s| s.dashboard) {
            rows[0] = &View::ROWS[0][1..];
        }
        rows
    }

    /// Stop showing the dashboard, from its own corner or from settings. The
    /// view it was open on has to go somewhere, and that is the spectrum.
    fn hide_dashboard(&mut self) {
        self.settings.edit(|s| s.dashboard = false);
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

/// The handle between two halves of a pane, and the drag that moves it.
///
/// The same gesture wherever a pane is split two ways and which half matters
/// changes with what is being watched: the map over its tracks, the call list
/// over its recordings. Follows the pointer rather than accumulating deltas,
/// so a long drag cannot leave the divider behind the cursor, and a double
/// click puts it back where it started.
fn split_divider(
    ui: &mut egui::Ui,
    top: f32,
    usable: f32,
    frac: f32,
    splitting: &mut bool,
    range: std::ops::RangeInclusive<f32>,
    default: f32,
) -> f32 {
    let (grip, resp) = ui.allocate_exact_size(
        Vec2::new(ui.available_width(), SPLIT_GRIP_H),
        Sense::click_and_drag(),
    );
    let hot = *splitting || resp.hovered();
    split_grip(&ui.painter_at(grip), &grip, hot);
    if hot {
        ui.ctx().set_cursor_icon(egui::CursorIcon::ResizeVertical);
    }
    if resp.drag_started() {
        *splitting = true;
    }
    let mut frac = frac;
    if *splitting {
        if let Some(pos) = resp.interact_pointer_pos() {
            let f = (pos.y - top - SPLIT_GRIP_H / 2.0) / usable;
            frac = f.clamp(*range.start(), *range.end());
        }
    }
    if resp.drag_stopped() {
        *splitting = false;
    }
    if resp.double_clicked() {
        frac = default;
    }
    frac
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

    /// A channel recalled from the bank arrives at the radio able to key.
    ///
    /// This is the whole path a person takes to transmit on a saved channel:
    /// the bank file, the strip, the spec sent to the radio thread, the plan,
    /// and the chain drawn from it. Every part of it was right on its own and
    /// the join was not, so the receiver drew no transmitter at all and the
    /// first key was refused for want of one.
    #[test]
    fn a_recalled_repeater_arrives_at_the_radio_able_to_key() {
        let bank = crate::memory::Memory::parse(
            "[Repeaters]\n145.7375 MHz NFM 12.5 kHz shift:-600kHz src:mic GB3XX\n",
        );
        let saved = &bank.list[0];

        let ch = recalled(7, saved);
        assert_eq!(ch.freq, 145_737_500.0);
        assert_eq!(ch.label, "GB3XX");
        assert_eq!(ch.bandwidth_hz, Some(12_500.0));

        let center = 145_500_000.0;
        let specs = specs_of(&[ch], center);
        assert_eq!(specs.len(), 1);
        let spec = &specs[0];
        assert_eq!(spec.offset_hz, 237_500.0);

        // The radio's question, asked of the spec the strip actually sent.
        let tx = spec.spec_to_transmit();
        assert_eq!(tx.shift_hz, -600_000.0);
        assert_eq!(tx.source, crate::radio::TxSource::Mic);

        let mut plan = crate::chain::tests::plan(2_400_000.0, Hz(center as u64));
        plan.channels = specs;
        let drawn = crate::radio::tests::transmit_plan(&plan).expect("a transmit chain to draw");
        assert_eq!(drawn.on_air, Hz(145_137_500), "it keys up on the repeater input");
        assert_eq!(drawn.spec.source, crate::radio::TxSource::Mic);
    }

    /// A channel nobody has said anything about still transmits.
    ///
    /// Whether a channel can transmit is its mode's question: it transmits in
    /// the mode it receives. Waiting for an operator to touch a control first
    /// is what left a key on screen over a receiver with no transmitter in
    /// its graph, because the panel that invented the transmit side was drawn
    /// after the spec had already gone to the radio.
    #[test]
    fn a_channel_just_added_transmits_without_anybody_saying_so() {
        let center = 446_000_000.0;
        let nfm = fresh(1, 446_050_000.0, ChanMode::Audio(Demod::Nfm), None);
        let usb = fresh(2, 446_050_000.0, ChanMode::Audio(Demod::Usb), None);
        assert_eq!(nfm.tx, None, "nothing has been said about it yet");

        let specs = specs_of(&[nfm, usb], center);
        let mut plan = crate::chain::tests::plan(2_400_000.0, Hz(center as u64));

        plan.channels = vec![specs[0].clone()];
        let nfm = crate::radio::tests::transmit_plan(&plan).expect("NFM transmits");
        assert_eq!(nfm.on_air, Hz(446_050_000), "simplex, on the channel's own frequency");
        assert_eq!(nfm.spec.source, crate::radio::TxSource::Tone, "the safe default");

        // And a mode with no modulator behind it does not, however much the
        // radio can transmit.
        plan.channels = vec![specs[1].clone()];
        assert_eq!(crate::radio::tests::transmit_plan(&plan), None, "nothing transmits USB");
    }

    /// What an operator set on the strip is what goes out.
    #[test]
    fn what_the_operator_changed_about_transmitting_reaches_the_radio() {
        let center = 145_000_000.0;
        let mut ch = fresh(1, 145_600_000.0, ChanMode::Audio(Demod::Nfm), None);
        ch.tx = Some(crate::radio::TxSpec {
            source: crate::radio::TxSource::Mic,
            shift_hz: -600_000.0,
            trim_db: -6.0,
            ..Default::default()
        });
        let mut plan = crate::chain::tests::plan(2_400_000.0, Hz(center as u64));
        plan.channels = specs_of(&[ch], center);
        let drawn = crate::radio::tests::transmit_plan(&plan).expect("it transmits");
        assert_eq!(drawn.on_air, Hz(145_000_000));
        assert_eq!(drawn.spec.trim_db, -6.0);
        assert_eq!(drawn.spec.source, crate::radio::TxSource::Mic);
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

    /// What the name of a command is, for asserting which ones a change
    /// produced without writing out their contents.
    fn named(c: &Cmd) -> &'static str {
        match c {
            Cmd::Decode(_) => "decode",
            Cmd::DcBlock(_) => "dc_block",
            Cmd::Manual(_) => "manual",
            Cmd::Refresh(_) => "refresh",
            Cmd::Smoothing(_) => "smoothing",
            Cmd::PacketLogCap(_) => "log_cap",
            Cmd::CaptureCap(_) => "capture_cap",
            Cmd::CaptureIq(_) => "capture",
            Cmd::CaptureTrigger(_) => "capture_trigger",
            Cmd::PacketLog(_) => "packet_log",
            Cmd::Survey(_) => "survey",
            Cmd::Gps(_) => "gps",
            Cmd::Feeds(_) => "feeds",
            Cmd::Location(..) => "location",
            Cmd::Audio { .. } => "audio",
            Cmd::Wigle(_) => "wigle",
            Cmd::BeaconDb(_) => "beacondb",
            Cmd::BandScan(_) => "band_scan",
            Cmd::Heatmap(_) => "heatmap",
            Cmd::ExportHeatmap { .. } => "export_heatmap",
            Cmd::HomeAssistant(_) => "homeassistant",
            Cmd::RecordCalls(_) => "record_calls",
            Cmd::Transcribe { .. } => "transcribe",
            _ => "something else",
        }
    }

    /// A radio thread that has been told nothing is told the whole record.
    ///
    /// A thread builds its graph from its own defaults: the scanner table
    /// running, the DC blocker in, nothing being recorded. Every one of those
    /// is the operator's, so a source stopped and started came back decoding
    /// a band nobody asked for.
    #[test]
    fn a_fresh_radio_is_told_every_setting_it_has_to_know() {
        let mut s = crate::session::Session {
            packet_log_on: true,
            survey_on: true,
            capture_on: true,
            gps: "/dev/ttyACM0@9600".into(),
            location: Some((53.5137, -6.2431)),
            audio_out: "Scarlett 2i2".into(),
            ..Default::default()
        };
        s.feeds.push(nodes::FeedSpec::new("10.0.0.5", 30005, &nodes::feed_nodes::BEAST));
        let cmds = settings_cmds(&s, None);
        let mut said: Vec<&str> = cmds.iter().map(named).collect();
        said.sort_unstable();
        assert_eq!(
            said,
            [
                "audio",
                "band_scan",
                "beacondb",
                "capture",
                "capture_cap",
                "capture_trigger",
                "dc_block",
                "decode",
                "feeds",
                "gps",
                "heatmap",
                "homeassistant",
                "location",
                "log_cap",
                "manual",
                "packet_log",
                "record_calls",
                "refresh",
                "smoothing",
                "survey",
                "transcribe",
                "wigle",
            ],
            "a setting the thread is not told is a setting that does not apply"
        );
    }

    /// One switch changed sends one command.
    ///
    /// The dial writes the record on every frame it moves, so applying the
    /// whole of it each time would reconnect the feeds sixty times a second.
    #[test]
    fn a_change_sends_only_what_changed() {
        let was = crate::session::Session::default();
        let mut now = was.clone();
        now.capture_on = true;
        let cmds = settings_cmds(&now, Some(&was));
        assert_eq!(cmds.iter().map(named).collect::<Vec<_>>(), ["capture"]);

        // The centre is in the same record and is not the radio thread's to
        // hear about this way: it is retuned where it is dragged.
        let mut moved = was.clone();
        moved.center = 868_300_000.0;
        assert!(settings_cmds(&moved, Some(&was)).is_empty(), "a drag reapplied the settings");
        assert!(settings_cmds(&was, Some(&was)).is_empty());
    }

    /// A feed with half an account typed into it cannot upload, and the
    /// switch has to say so rather than sitting on while nothing goes.
    #[test]
    fn a_feed_with_no_account_comes_back_off() {
        let mut a = app();
        a.settings.edit(|s| {
            s.wigle_on = true;
            s.wigle_name = "AID0000".into();
            s.ha_on = true;
        });
        a.apply_settings();
        assert!(!a.setting(|s| s.wigle_on), "uploading with no token");
        assert!(!a.setting(|s| s.ha_on), "publishing to no broker");

        a.settings.edit(|s| {
            s.wigle_token = "hunter2".into();
            s.wigle_on = true;
            s.ha_host = "homeassistant.local".into();
            s.ha_on = true;
        });
        a.apply_settings();
        assert!(a.setting(|s| s.wigle_on));
        assert!(a.setting(|s| s.ha_on));
    }

    /// What a pane writes is what is saved, and what is saved is what is
    /// applied: the fault this replaces was a switch kept in three places,
    /// where missing one of them left it on screen and nowhere else.
    #[test]
    fn a_switch_thrown_in_a_pane_is_in_the_record_and_survives_the_file() {
        let mut a = app();
        a.settings.edit(|s| {
            s.packet_log_on = true;
            s.log_dir = "/tmp/waveshark-log".into();
            s.list_unknown = false;
            s.capture_on = true;
            s.beacondb_on = true;
        });
        a.apply_settings();
        let back = crate::session::Session::parse(&a.settings.get().render());
        assert!(back.packet_log_on && back.capture_on && back.beacondb_on);
        assert!(!back.list_unknown);
        assert_eq!(back.log_path(), Some(std::path::PathBuf::from("/tmp/waveshark-log")));
        // And applying it again sends nothing: it is already what it is.
        let now = a.settings.get();
        assert!(settings_cmds(&now, Some(&now)).is_empty());
    }

    /// The scripts panel's TX button, which is the whole of what an
    /// operator does to send a file: the dial, one channel keying the file,
    /// and a key that comes back up when the file has played.
    #[test]
    fn keying_a_sub_file_tunes_a_channel_and_lets_the_key_go_by_itself() {
        let mut a = app();
        let text = "Filetype: Flipper SubGhz Key File\nVersion: 1\nFrequency: 433920000\n\
            Preset: FuriHalSubGhzPresetOok650Async\nProtocol: Princeton\nBit: 24\n\
            Key: 00 00 00 00 00 95 D5 D4\nTE: 400\n";
        let path = std::env::temp_dir().join(format!("waveshark-tx-{}.sub", std::process::id()));
        std::fs::write(&path, text).unwrap();
        let f = crate::radio::SubFile::open(&path).unwrap();
        let over = f.file.duration();

        a.transmit_sub(f);
        // The dial moved to the file, because 433.92 MHz is nowhere near the
        // 100 MHz span this receiver was on.
        assert_eq!(a.center, 433_920_000.0);
        let ch: Vec<&Channel> = a.audio.channels.iter().collect();
        assert_eq!(ch.len(), 1, "one channel, not one per press");
        assert_eq!(ch[0].freq, 433_920_000.0);
        assert!(ch[0].on, "the radio is only sent the channels that are on");
        assert!(ch[0].muted, "keyed, not listened to");
        assert_eq!(ch[0].tx.as_ref().map(|t| t.source), Some(crate::radio::TxSource::Sub));
        let id = ch[0].id;
        let keyed = a.cmds.iter().filter(|c| matches!(c, Cmd::Key(Some(k)) if *k == id)).count();
        assert_eq!(keyed, 1, "keyed once");
        assert_eq!(a.cmds.iter().filter(|c| matches!(c, Cmd::SubFile(Some(_)))).count(), 1);
        assert_eq!(a.audio.sub_pick.file.as_ref().map(|f| f.label()), Some("Princeton".into()));

        // A second file reuses the channel rather than adding another.
        let f2 = crate::radio::SubFile::open(&path).unwrap();
        a.transmit_sub(f2);
        assert_eq!(a.audio.channels.len(), 1);

        // The key is still down while the file plays, and comes up after it
        // with the tail the transmitter needs to play the last gap out.
        let until = a.sub_until.expect("the over has an end");
        assert!(until > std::time::Instant::now() + over, "the tail is past the file");
        assert!(until <= std::time::Instant::now() + over + SUB_TAIL);
        a.sub_until = Some(std::time::Instant::now() - std::time::Duration::from_millis(1));
        a.cmds.clear();
        a.sub_key(&egui::Context::default());
        assert!(a.sub_until.is_none());
        assert_eq!(a.cmds.iter().filter(|c| matches!(c, Cmd::Key(None))).count(), 1);
        let _ = std::fs::remove_file(&path);
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
            written: false,
            rssi_dbfs: -18.0,
            snr_db: 21.5,
            bytes: vec![0xab, 0xcd],
            crc,
            link: None,
            report: common::ReportDetail::Bare,
            identity: None,
            iq: None,
            pulses: None,
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
        a.settings.edit(|s| s.list_unknown = false);
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
        let decode_on = a.setting(|s| s.decode_on);
        scope::Scope {
            st: &mut a.scope,
            channels: &mut a.audio.channels,
            listening: a.audio.listening,
            center: a.center,
            rate: a.rate,
            radio: None,
            scanners: &a.scanners,
            patch: &a.chain.patch,
            decode_on,
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

    /// Setting an offset moves the dial and what it can reach by the same
    /// amount, so the receiver stays on the signal it was on and the dial is
    /// not stranded below everything the radio can do. Taking the offset off
    /// again puts it back.
    #[test]
    fn an_offset_carries_the_dial_and_its_limits_with_it() {
        let mut a = app();
        a.reach = (1e6, 6e9);
        a.retune(739_500_000.0);
        a.set_offset(9_750_000_000.0);
        assert_eq!(a.center, 10_489_500_000.0, "the dial reads the frequency at the dish");
        assert_eq!(a.reach, (9_751_000_000.0, 15_750_000_000.0));
        a.retune(10_714_000_000.0);
        assert_eq!(a.center, 10_714_000_000.0, "and a transponder is now reachable");
        a.set_offset(0.0);
        assert_eq!(a.center, 964_000_000.0, "taking it off leaves the tuner where it was");
        assert_eq!(a.reach, (1e6, 6e9));
    }

    /// The strip is the only route to a view by pointer, so a view missing
    /// from it is a view that cannot be opened, and two views sharing a
    /// glyph or a digit is a tab that opens the wrong one.
    #[test]
    fn every_view_has_a_tab_of_its_own() {
        let tabs: Vec<View> = View::ROWS.into_iter().flatten().copied().collect();
        assert_eq!(tabs.len(), 14);
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
            View::Agent,
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

    /// A key saved last week is not news this morning.
    ///
    /// Every other list starts empty and fills as things arrive, so a tab
    /// that lights for "has anything" is only ever wrong about the keys:
    /// theirs is read off disk before the first frame is drawn. Anybody with
    /// one key saved got the dot on every single start, which is a lamp that
    /// is always lit and tells nobody anything.
    #[test]
    fn a_tab_does_not_light_for_what_was_already_there_when_the_window_opened() {
        let key = |name: &str| decode::channel_keys::ChannelKey {
            system: decode::channel_keys::System::Meshtastic,
            name: name.into(),
            key: vec![0x01; 16],
        };
        let mut a = app();
        // As the store comes back from disk, before anybody has looked.
        a.keys.store = crate::keystore::KeyStore::default();
        a.keys.store.insert_channel(key("LongFast"));
        a.forget_what_was_already_here();
        assert!(!a.view_live(View::Keys), "a key saved before the window opened is not news");

        // And one that turns up while you are looking elsewhere still lights
        // the tab, which is the whole point of the dot.
        a.keys.store.insert_channel(key("PrivateRoom"));
        assert!(a.view_live(View::Keys));
        a.set_view(View::Keys);
        a.read_views();
        assert!(!a.view_live(View::Keys), "the tab has been looked at");
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
