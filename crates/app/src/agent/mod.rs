//! The receiver, driven by something that is not a person.
//!
//! An agent gets the same receiver the operator has rather than a second one
//! of its own: the tools here queue an [`Action`] onto the interface, which
//! applies it at the top of a frame and answers with what the interface then
//! holds. So an agent that tunes moves the dial on screen, and an agent that
//! asks what is on the air reads the same packet list a person is looking at.
//!
//! The alternative, a headless receiver behind the same protocol, was
//! rejected for that reason: two receivers means two graphs, two USB claims
//! and an interface that cannot be asked what the agent just did.
//!
//! Two front ends read the same desk. MCP over streamable HTTP, bound to a
//! loopback address given on the command line and serving nothing unless
//! `--mcp-listen` was passed; and the chat in the Agent view, which drives a
//! model over the same catalogue of tools. Neither knows about the other, and
//! nothing here knows about egui: what wakes the interface is a closure it
//! hangs on the [`Bell`], so an agent with no window open still gets served.

pub mod catalog;
pub mod channel;
pub mod chat;
pub mod config;
pub mod served;
#[cfg(feature = "mcp")]
mod tools;
pub mod voice;

use std::sync::Arc;

/// One thing an agent asked for, with somewhere to put the answer.
pub struct Ask {
    pub action: Action,
    pub reply: tokio::sync::oneshot::Sender<Reply>,
}

/// What a tool call comes back with: a value for the agent, or why not.
pub type Reply = Result<serde_json::Value, String>;

/// How long a tool waits for the interface to answer.
///
/// Longer than a frame by a wide margin, since a retune, a rebuild of the
/// graph or a screenshot all land on frames after the one that took the
/// request. Short enough that an interface which has stopped drawing is
/// reported as such rather than hanging the agent.
const PATIENCE: std::time::Duration = std::time::Duration::from_secs(20);

/// Whoever has to be nudged when a job lands on the desk.
///
/// The interface only draws when something happens, and a request arriving
/// over a socket or from a model is not something egui knows about. The bell
/// is empty until a front end hangs its own repaint on it, so the desk can be
/// built before there is a window and works when there is none.
type Ring = Arc<dyn Fn() + Send + Sync>;

#[derive(Clone, Default)]
pub struct Bell {
    ring: Arc<parking_lot::Mutex<Option<Ring>>>,
}

impl Bell {
    pub fn answered_by(&self, f: impl Fn() + Send + Sync + 'static) {
        *self.ring.lock() = Some(Arc::new(f));
    }

    fn ring(&self) {
        let held = self.ring.lock().clone();
        if let Some(f) = held {
            f();
        }
    }
}

/// The counter an agent puts its work on: a queue into the interface and the
/// bell to wake it with.
#[derive(Clone)]
pub struct Desk {
    jobs: crossbeam_channel::Sender<Ask>,
    bell: Bell,
}

impl Desk {
    /// A desk, and the queue whoever holds the receiver drains.
    pub fn new() -> (Self, crossbeam_channel::Receiver<Ask>) {
        let (jobs, asks) = crossbeam_channel::unbounded();
        (Self { jobs, bell: Bell::default() }, asks)
    }

    pub fn bell(&self) -> Bell {
        self.bell.clone()
    }

    /// Queue an action and wait for the interface to answer it.
    pub async fn ask(&self, action: Action) -> Reply {
        let (reply, answer) = tokio::sync::oneshot::channel();
        self.jobs.send(Ask { action, reply }).map_err(|_| "the interface has gone".to_string())?;
        self.bell.ring();
        match tokio::time::timeout(PATIENCE, answer).await {
            Ok(Ok(r)) => r,
            Ok(Err(_)) => Err("the interface dropped the request".into()),
            Err(_) => Err(format!("the interface did not answer in {:?}", PATIENCE)),
        }
    }
}

/// Everything an agent can ask the receiver to do or to say.
///
/// One enum rather than a method per tool: the interface applies these in one
/// place, so what an agent can reach is a list somebody can read, and a tool
/// is a name and a parameter type in front of a variant.
pub enum Action {
    // What the receiver is doing.
    Status,
    Devices,
    Spectrum(args::Spectrum),
    Channels,
    Packets(args::Packets),
    Calls(args::Limit),
    Transcript(args::Limit),
    Messages(args::Limit),
    Links(args::Limit),
    ControlLinks,
    Tracks(args::Limit),
    Satellites(args::Limit),
    Chain,
    Patch,
    StageKinds,
    Scanners,
    Memory,
    Protocols,
    Datasets,
    Screenshot,

    // What it is set to.
    Start,
    Stop,
    SelectDevice(args::Device),
    Tune(args::Tune),
    Span(args::Span),
    Gain(args::Gain),
    Toggle(args::Toggle),
    Choice(args::Choice),
    Ppm(args::Ppm),
    Location(args::Location),

    // Channels and what is heard.
    AddChannel(args::AddChannel),
    SetChannel(args::SetChannel),
    RemoveChannel(args::Channel),
    Listen(args::Channel),
    Volume(args::Volume),

    // Transmitting. Half duplex: one channel is keyed or none is, which is
    // why keying takes the channel and unkeying takes nothing.
    Key(args::Channel),
    Unkey,
    Transmit(args::Transmit),
    TxGain(args::TxGain),
    Say(args::Say),
    TransmitModes,

    // What it watches, writes and shows.
    Decode(args::Switch),
    DcBlock(args::Switch),
    View(args::View),
    Record(args::Record),
    CaptureIq(args::Switch),
    PacketLog(args::PacketLog),
    NodeParam(args::NodeParam),

    // What the receiver is, rather than what it is doing: the tables, the
    // installation and the feeds. Each of these is a file or a command the
    // settings panes write, reached here by the same route a click takes.
    AddScanner(args::AddScanner),
    SetScanner(args::SetScanner),
    RemoveScanner(args::Named),
    AddMemory(args::AddMemory),
    RemoveMemory(args::FindMemory),
    RecallMemory(args::FindMemory),
    SetVoice(args::Voice),
    SetTranscriber(args::Transcriber),
    SetStation(args::Station),
    SetSound(args::Sound),
    SetSurvey(args::Survey),
    SetWigle(args::Wigle),
    SetBeaconDb(args::BeaconDb),
    SetHomeAssistant(args::HomeAssistant),
    AddFeed(args::AddFeed),
    RemoveFeed(args::Named),
    SetCalls(args::Calls),
    SetWatching(args::Watching),
    RefreshDataset(args::Named),
    SetDatasetKey(args::DatasetKey),
    SetDisplay(args::Display),
    SetCallLog(args::CallLog),

    // Drawing the graph. Every one of these is an edit on top of the graph
    // the receiver derives, so a retune keeps it, and every one of them is
    // answered by the rebuild that took it rather than by the frame that
    // asked for it: an edit that will not build is refused and the previous
    // graph goes back.
    AddStage(args::StageKind),
    RemoveStage(args::StageId),
    Connect(args::Connect),
    Disconnect(args::Disconnect),
    UndoEdit,
    RedoEdit,
    ResetGraph,
    Manual(args::Switch),
}

pub mod args {
    //! What each tool takes, and the closed sets it takes them from.
    //!
    //! These are the schemas an agent reads, so a field's name and its unit
    //! are the whole of its documentation: `mhz`, `khz`, `db`.

    use schemars::JsonSchema;
    use serde::Deserialize;

    #[derive(Debug, Deserialize, JsonSchema)]
    pub struct Limit {
        /// Rows to return, newest first. Defaults to 50.
        pub limit: Option<usize>,
    }

    #[derive(Debug, Deserialize, JsonSchema)]
    pub struct Spectrum {
        /// Bins to reduce the span to, by peak. Defaults to 64.
        pub bins: Option<usize>,
        /// Strongest peaks to name, by frequency. Defaults to 8.
        pub peaks: Option<usize>,
    }

    #[derive(Debug, Deserialize, JsonSchema)]
    pub struct Packets {
        /// Rows to return, newest first. Defaults to 50.
        pub limit: Option<usize>,
        /// Only packets this protocol claimed, by its id or its label.
        pub protocol: Option<String>,
        /// Only packets decoded in the last this many seconds.
        pub within_seconds: Option<f64>,
        /// Include the payload bytes as hex. Off by default: a busy band
        /// answers with more hex than an agent has room for.
        pub bytes: Option<bool>,
    }

    #[derive(Debug, Deserialize, JsonSchema)]
    pub struct Device {
        /// Any part of the radio's label, as `list_devices` reports it.
        pub name: String,
    }

    #[derive(Debug, Deserialize, JsonSchema)]
    pub struct Tune {
        /// Where to point the receiver, in MHz. This is the centre of the
        /// span, not a station: a channel is opened with `add_channel`.
        pub mhz: f64,
    }

    #[derive(Debug, Deserialize, JsonSchema)]
    pub struct Span {
        /// How wide a span to work in, in kHz. The nearest the radio can do
        /// is used, narrowed in software where it cannot sample that slowly.
        pub khz: f64,
    }

    /// How one gain stage is driven.
    #[derive(Debug, Deserialize, JsonSchema)]
    #[serde(tag = "how", rename_all = "lowercase")]
    pub enum GainMode {
        /// Let the hardware or the driver pick.
        Auto,
        /// Fixed gain. The hardware snaps to its nearest step.
        Manual { db: f32 },
    }

    #[derive(Debug, Deserialize, JsonSchema)]
    pub struct Gain {
        /// The stage's driver name, from `status`. "tuner" spreads a total
        /// across whatever stages the radio has.
        pub stage: String,
        pub gain: GainMode,
    }

    #[derive(Debug, Deserialize, JsonSchema)]
    pub struct Toggle {
        /// The switch's driver name, from `status`.
        pub name: String,
        pub on: bool,
    }

    #[derive(Debug, Deserialize, JsonSchema)]
    pub struct Choice {
        /// The setting's driver name, from `status`.
        pub name: String,
        /// One of the options `status` lists for it.
        pub value: String,
    }

    #[derive(Debug, Deserialize, JsonSchema)]
    pub struct Ppm {
        /// Reference correction for the radio in use, in parts per million.
        pub ppm: f64,
    }

    #[derive(Debug, Deserialize, JsonSchema)]
    pub struct Location {
        pub lat: f64,
        pub lon: f64,
    }

    #[derive(Debug, Deserialize, JsonSchema)]
    pub struct AddChannel {
        /// Where the channel sits, in MHz. It has to be inside the span.
        pub mhz: f64,
        /// What the channel does with that frequency: a demodulator (wfm,
        /// nfm, am, usb, lsb, cw), "auto" to find and decode whatever
        /// transmits in it, or a protocol id from `list_protocols`.
        /// Defaults to what the band plan suggests.
        pub mode: Option<String>,
        /// Width in kHz. Defaults to what the mode asks for.
        pub bandwidth_khz: Option<f64>,
        /// What to call it on the strip.
        pub label: Option<String>,
    }

    #[derive(Debug, Deserialize, JsonSchema)]
    pub struct SetChannel {
        /// From `list_channels`.
        pub id: u64,
        pub mhz: Option<f64>,
        /// As in `add_channel`.
        pub mode: Option<String>,
        pub bandwidth_khz: Option<f64>,
        pub label: Option<String>,
        /// Whether the channel runs at all.
        pub on: Option<bool>,
        /// Its own level in the mix, 0 to 1.
        pub volume: Option<f32>,
        pub muted: Option<bool>,
        /// Where the squelch opens, in dB. The scale depends on the mode:
        /// `list_channels` reports the range and what it is measuring.
        pub squelch_db: Option<f32>,
        pub agc: Option<bool>,
        /// Treat what is heard here as speech: a row in the call list, and a
        /// transcript where a model is installed.
        pub voice: Option<bool>,
    }

    #[derive(Debug, Deserialize, JsonSchema)]
    pub struct Channel {
        /// From `list_channels`.
        pub id: u64,
    }

    /// What a keyed channel puts through the modulator.
    #[derive(Debug, Clone, Copy, Deserialize, JsonSchema)]
    #[serde(rename_all = "lowercase")]
    pub enum Source {
        /// A steady tone, which is what a deviation or power check wants.
        Tone,
        /// The microphone, opened while the channel is keyed.
        Mic,
        /// What the agent says: the queue `say` fills, and the voice it
        /// answers an over with. A channel set to this keys itself when
        /// there is something to say.
        Agent,
    }

    #[derive(Debug, Deserialize, JsonSchema)]
    pub struct Transmit {
        /// From `list_channels`. The channel's own mode decides how it is
        /// modulated, so a transmission is set up by setting the channel.
        pub id: u64,
        pub source: Option<Source>,
        /// The tone fed to the modulator under `tone`, in Hz.
        pub tone_hz: Option<f32>,
        /// Microphone gain, as a multiplier on what the capture delivers.
        pub mic_gain: Option<f32>,
        /// Level into the modulator in dB, for trimming deviation.
        pub trim_db: Option<f32>,
        /// What a digital mode transmits, as a path on this machine: a
        /// transport stream for DVB-T, or anything ffmpeg can open, which is
        /// re-encoded into one. Empty goes back to the test card.
        pub file: Option<String>,
    }

    #[derive(Debug, Deserialize, JsonSchema)]
    pub struct Say {
        /// What to say, as a person would read it aloud. Cut to what fits in
        /// one over.
        pub text: String,
        /// The channel to say it on, from `list_channels`. Its transmit
        /// source is set to the agent. Omit to use the channel already set
        /// to AGENT.
        pub channel: Option<u64>,
        /// The voice to say it in: a voice the speech server names, or, for
        /// the model on this machine, a sentence describing how it should
        /// sound. Omit for the one in the Agent settings.
        pub voice: Option<String>,
    }

    #[derive(Debug, Deserialize, JsonSchema)]
    pub struct TxGain {
        /// The radio's transmit gain, in dB. What that buys in power is the
        /// radio's business, and an amplifier's beyond it.
        pub db: f32,
    }

    #[derive(Debug, Deserialize, JsonSchema)]
    pub struct Volume {
        /// Master level, 0 to 1.
        pub volume: Option<f32>,
        /// Whether the mix leaves the bus at all.
        pub muted: Option<bool>,
    }

    #[derive(Debug, Deserialize, JsonSchema)]
    pub struct Switch {
        pub on: bool,
    }

    /// Which view the window shows.
    #[derive(Debug, Clone, Copy, Deserialize, JsonSchema)]
    #[serde(rename_all = "lowercase")]
    pub enum ViewName {
        Dashboard,
        Spectrum,
        Chain,
        Map,
        Calls,
        Transcript,
        Messages,
        Links,
        Devices,
        Satellites,
        Video,
        Keys,
        Control,
        Agent,
    }

    #[derive(Debug, Deserialize, JsonSchema)]
    pub struct View {
        pub view: ViewName,
    }

    #[derive(Debug, Deserialize, JsonSchema)]
    pub struct Record {
        /// Where to write the bursts that decode, or omit for the default
        /// folder. Ignored when `on` is false.
        pub dir: Option<String>,
        /// How much may be written before recording stops, in megabytes.
        pub budget_mb: Option<u64>,
        pub on: bool,
    }

    #[derive(Debug, Deserialize, JsonSchema)]
    pub struct PacketLog {
        /// Where to write the log, or omit for the default folder.
        pub dir: Option<String>,
        pub on: bool,
    }

    #[derive(Debug, Deserialize, JsonSchema)]
    pub struct StageKind {
        /// A stage type from `list_stage_kinds`.
        pub kind: String,
    }

    #[derive(Debug, Deserialize, JsonSchema)]
    pub struct StageId {
        /// A stage id from `patch`, not a node id from `chain`.
        pub stage: u64,
    }

    /// Where a wire starts.
    #[derive(Debug, Deserialize, JsonSchema)]
    #[serde(tag = "from", rename_all = "lowercase")]
    pub enum Tap {
        /// The receiver's own samples after the DC block and the zoom, which
        /// is what every automatic branch reads.
        Span,
        /// An output port of another stage in the patch.
        Stage { id: u64, port: usize },
    }

    #[derive(Debug, Deserialize, JsonSchema)]
    pub struct Connect {
        pub source: Tap,
        /// The stage being fed, by its patch id.
        pub to_stage: u64,
        /// Which of its inputs. An input takes one producer, so this
        /// replaces whatever was feeding it.
        pub to_port: usize,
    }

    #[derive(Debug, Deserialize, JsonSchema)]
    pub struct Disconnect {
        /// The stage whose input is being freed, by its patch id.
        pub stage: u64,
        pub port: usize,
    }

    #[derive(Debug, Deserialize, JsonSchema)]
    pub struct NodeParam {
        /// Node id, from `chain`.
        pub node: usize,
        /// Parameter name, from that node's entry in `chain`.
        pub name: String,
        /// The value, as JSON: a number, a boolean, a string, or a list of
        /// numbers. It has to match the parameter's own kind.
        pub value: serde_json::Value,
    }

    /// One thing named, where the name is the whole of the argument: a
    /// scanner block, a feed's address, a dataset.
    #[derive(Debug, Deserialize, JsonSchema)]
    pub struct Named {
        pub name: String,
    }

    #[derive(Debug, Deserialize, JsonSchema)]
    pub struct AddScanner {
        /// What the block is called, which is also what names it afterwards.
        pub name: String,
        /// The front end it runs: `auto`, `banks`, or a protocol id from
        /// `list_protocols`.
        pub front: String,
        /// The band the block is about, in MHz.
        pub lo_mhz: f64,
        pub hi_mhz: f64,
        /// Frequencies that must be inside the span for it to run, in MHz.
        /// Omit for a block decided by its band alone.
        pub channels_mhz: Option<Vec<f64>>,
        /// Channel widths for a `banks` front end, in kHz.
        pub widths_khz: Option<Vec<f64>>,
        /// Narrowest span the front end works in, in kHz.
        pub min_span_khz: Option<f64>,
        /// How far inside the span edge a channel must fall, in kHz.
        pub margin_khz: Option<f64>,
        /// Band plans this block is for: europe, americas, asia-pacific.
        /// Omit for everywhere.
        pub regions: Option<Vec<String>>,
        pub enabled: Option<bool>,
    }

    #[derive(Debug, Deserialize, JsonSchema)]
    pub struct SetScanner {
        /// The block to change, as `scanners` names it.
        pub name: String,
        /// What to call it instead.
        pub rename: Option<String>,
        pub front: Option<String>,
        pub lo_mhz: Option<f64>,
        pub hi_mhz: Option<f64>,
        pub channels_mhz: Option<Vec<f64>>,
        pub widths_khz: Option<Vec<f64>>,
        pub min_span_khz: Option<f64>,
        pub margin_khz: Option<f64>,
        pub regions: Option<Vec<String>>,
        /// Whether the block runs. A block switched off keeps everything it
        /// was configured with.
        pub enabled: Option<bool>,
    }

    #[derive(Debug, Deserialize, JsonSchema)]
    pub struct AddMemory {
        /// Where the channel is, in MHz.
        pub mhz: f64,
        /// What to call it.
        pub label: String,
        /// The group it goes in. Defaults to "Channels".
        pub group: Option<String>,
        /// As in `add_channel`. Defaults to what the band plan suggests.
        pub mode: Option<String>,
        pub bandwidth_khz: Option<f64>,
    }

    /// A saved channel, by what tells it from the others.
    #[derive(Debug, Deserialize, JsonSchema)]
    pub struct FindMemory {
        /// Its label, as `memory` reports it. Matched without case.
        pub label: Option<String>,
        /// Where it is, in MHz, for a channel with no label or two with the
        /// same one.
        pub mhz: Option<f64>,
    }

    /// Where the agent's voice is made.
    #[derive(Debug, Clone, Copy, Deserialize, JsonSchema)]
    #[serde(rename_all = "lowercase")]
    pub enum VoiceFrom {
        /// A model on this machine. Nothing is sent anywhere.
        Local,
        /// The chat model's own server, at its /audio/speech.
        Chat,
        /// A speech server with an address of its own.
        Server,
    }

    #[derive(Debug, Deserialize, JsonSchema)]
    pub struct Voice {
        /// Where speech is made. Called with nothing, this reports what is
        /// set and changes nothing.
        pub source: Option<VoiceFrom>,
        /// For a model here: the catalogue id, or any repository name.
        pub model: Option<String>,
        /// For a model here: `auto`, `cpu`, `cuda:0`, `metal`.
        pub device: Option<String>,
        /// For a model here: `full` or `half`.
        pub precision: Option<String>,
        /// For a model here: the sentence that says how it should sound.
        pub description: Option<String>,
        /// Where the weights are kept. Empty for the usual place.
        pub dir: Option<String>,
        /// For a speech server: its base address.
        pub url: Option<String>,
        /// For a server: the speech model to ask it for.
        pub server_model: Option<String>,
        /// For a server: the voice as it names it.
        pub voice: Option<String>,
        /// What the agent answers to on the air.
        pub wake: Option<String>,
        /// How long after the channel goes quiet before it keys, in seconds.
        pub hang_s: Option<f64>,
        /// How long after its own over it answers without being named, in
        /// seconds. Zero wants the name every time.
        pub follow_s: Option<f64>,
    }

    #[derive(Debug, Deserialize, JsonSchema)]
    pub struct Transcriber {
        /// Whether what is heard is read back into words at all.
        pub enabled: Option<bool>,
        /// Where it is read: local, chat or server.
        pub source: Option<VoiceFrom>,
        /// For the model here: which weights, as the Transcript pane names
        /// them.
        pub model: Option<String>,
        /// For the model here: `auto`, `cpu`, `cuda:0`, `metal`.
        pub device: Option<String>,
        /// Shortest speech worth reading, in seconds.
        pub min_speech_s: Option<f64>,
        /// For a reading server: its base address.
        pub url: Option<String>,
        /// For a server: the model to ask it for, such as whisper-1.
        pub server_model: Option<String>,
    }

    #[derive(Debug, Deserialize, JsonSchema)]
    pub struct Station {
        /// ISO country code. Sets the band plan and the cell export with it.
        pub country: Option<String>,
        /// europe, americas or asia-pacific, to override the country's.
        pub band_plan: Option<String>,
    }

    #[derive(Debug, Deserialize, JsonSchema)]
    pub struct Sound {
        /// Where the mix comes out, by device name. Empty for the system
        /// default.
        pub speaker: Option<String>,
        /// What a keyed channel transmits, by device name.
        pub microphone: Option<String>,
    }

    #[derive(Debug, Deserialize, JsonSchema)]
    pub struct Survey {
        /// Whether every device heard is recorded to a database.
        pub on: Option<bool>,
        /// Where that database is. Omit for the usual place.
        pub path: Option<String>,
        /// A GPS to take the position from, as host:port or a serial port.
        /// Empty for the local gpsd.
        pub gps: Option<String>,
    }

    #[derive(Debug, Deserialize, JsonSchema)]
    pub struct Wigle {
        /// Whether what is heard is uploaded to wigle.net.
        pub on: Option<bool>,
        /// The account name from wigle.net/account.
        pub name: Option<String>,
        pub token: Option<String>,
        /// Whether wigle.net may licence what is uploaded commercially.
        pub donate: Option<bool>,
    }

    #[derive(Debug, Deserialize, JsonSchema)]
    pub struct BeaconDb {
        /// Whether what is heard is submitted to beacondb.net.
        pub on: Option<bool>,
        /// Whether the map may ask it where a decoded cell is.
        pub lookup: Option<bool>,
    }

    #[derive(Debug, Deserialize, JsonSchema)]
    pub struct HomeAssistant {
        /// Whether every device heard is published for Home Assistant.
        pub on: Option<bool>,
        pub host: Option<String>,
        pub port: Option<u16>,
        pub username: Option<String>,
        pub password: Option<String>,
        /// What Home Assistant listens under. Empty means homeassistant.
        pub prefix: Option<String>,
        /// The topic the readings go to.
        pub topic: Option<String>,
        /// Identity spaces worth publishing, comma separated: `ism,wmbus` is
        /// a house's own sensors without the street's handsets.
        pub spaces: Option<String>,
        /// Whether what people say and write goes with the readings.
        pub buses: Option<bool>,
    }

    #[derive(Debug, Deserialize, JsonSchema)]
    pub struct AddFeed {
        /// Another receiver, as host or host:port.
        pub host: String,
        /// What it speaks, from what `add_feed` lists when it refuses.
        pub kind: String,
    }

    /// What a standing instruction on the audio bus names.
    #[derive(Debug, Clone, Copy, Deserialize, JsonSchema)]
    #[serde(rename_all = "lowercase")]
    pub enum CallRule {
        /// Every call any decoder reads.
        Everything,
        /// One talkgroup, reflector or destination.
        Group,
        /// One caller, wherever they transmit.
        Caller,
        /// Whatever is heard on one channel.
        Channel,
        /// One system: every M17 call, every DMR call.
        System,
    }

    #[derive(Debug, Deserialize, JsonSchema)]
    pub struct Subscription {
        pub rule: CallRule,
        /// The group, caller or system named. Ignored by `everything`.
        pub value: Option<String>,
        /// Where the channel is, in MHz, for a `channel` rule.
        pub mhz: Option<f64>,
        /// Its level in the mix, 0 to 2.
        pub volume: Option<f32>,
        pub muted: Option<bool>,
    }

    #[derive(Debug, Deserialize, JsonSchema)]
    pub struct Calls {
        /// The whole set of standing instructions, replacing what is there.
        /// An empty list hears nothing; omit it to read what is set.
        pub subscriptions: Option<Vec<Subscription>>,
    }

    #[derive(Debug, Deserialize, JsonSchema)]
    pub struct Watching {
        /// The transmission to watch, by the key the video bus keeps it
        /// under. Omit with `everything` false to read what is set.
        pub key: Option<String>,
        /// Watch whatever comes, which is what a scanning receiver wants.
        pub everything: Option<bool>,
    }

    #[derive(Debug, Deserialize, JsonSchema)]
    pub struct DatasetKey {
        /// The dataset, as `datasets` names it.
        pub name: String,
        /// Which of its keys, as `datasets` names them.
        pub key: String,
        pub value: String,
    }

    #[derive(Debug, Deserialize, JsonSchema)]
    pub struct Display {
        /// Bins the transform produces: 512 to 16384.
        pub fft: Option<usize>,
        /// Spectrum frames a second.
        pub refresh_hz: Option<f32>,
        /// How much of the last frame the next one keeps, 0 to 1.
        pub smoothing: Option<f32>,
        /// Waterfall rows a second.
        pub rows_per_sec: Option<f32>,
        /// Whether the scale follows the noise floor.
        pub auto_scale: Option<bool>,
        /// Bottom of the scale, in dBFS. Ignored while auto_scale is on.
        pub floor_dbfs: Option<f32>,
        /// Top of the scale, in dBFS.
        pub ceil_dbfs: Option<f32>,
    }

    #[derive(Debug, Deserialize, JsonSchema)]
    pub struct CallLog {
        /// Whether every over heard is kept as Opus.
        pub on: bool,
    }
}

/// Start serving MCP on `addr`, against a desk the interface is already
/// draining.
///
/// The socket is bound here rather than in the task, so a port already in use
/// is reported at startup instead of into a log nobody is reading.
#[cfg(feature = "mcp")]
pub fn serve(
    addr: std::net::SocketAddr,
    rt: &tokio::runtime::Handle,
    desk: Desk,
) -> anyhow::Result<()> {
    let listener = std::net::TcpListener::bind(addr)?;
    listener.set_nonblocking(true)?;
    let bound = listener.local_addr()?;
    rt.spawn(async move {
        let listener = match tokio::net::TcpListener::from_std(listener) {
            Ok(l) => l,
            Err(e) => {
                tracing::error!("mcp: {e}");
                return;
            }
        };
        let service = rmcp::transport::streamable_http_server::StreamableHttpService::new(
            move || Ok(tools::Tools::new(desk.clone())),
            std::sync::Arc::new(
                rmcp::transport::streamable_http_server::session::local::LocalSessionManager::default(),
            ),
            rmcp::transport::streamable_http_server::StreamableHttpServerConfig::default()
                .with_json_response(true)
                .with_legacy_session_mode(false),
        );
        let router = axum::Router::new().nest_service("/mcp", service);
        if let Err(e) = axum::serve(listener, router).await {
            tracing::error!("mcp: {e}");
        }
    });
    println!("mcp on http://{bound}/mcp");
    Ok(())
}
