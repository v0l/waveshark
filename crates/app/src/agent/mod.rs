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
//! Transport is MCP over streamable HTTP, bound to a loopback address given
//! on the command line. Nothing is served unless `--mcp-listen` was passed.

mod tools;

use std::net::SocketAddr;

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

/// The counter an agent puts its work on: a queue into the interface and the
/// context to wake it with.
#[derive(Clone)]
pub struct Desk {
    jobs: crossbeam_channel::Sender<Ask>,
    ctx: egui::Context,
}

impl Desk {
    /// Queue an action and wait for the interface to answer it.
    pub async fn ask(&self, action: Action) -> Reply {
        let (reply, answer) = tokio::sync::oneshot::channel();
        self.jobs.send(Ask { action, reply }).map_err(|_| "the interface has gone".to_string())?;
        // The interface only draws when something happens, and a request
        // arriving over a socket is not something egui knows about.
        self.ctx.request_repaint();
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

    // What it watches, writes and shows.
    Decode(args::Switch),
    DcBlock(args::Switch),
    View(args::View),
    Record(args::Record),
    CaptureIq(args::Switch),
    PacketLog(args::PacketLog),
    NodeParam(args::NodeParam),

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
}

/// Start serving MCP on `addr`, and hand back the queue the interface drains.
///
/// The socket is bound here rather than in the task, so a port already in use
/// is reported at startup instead of into a log nobody is reading.
pub fn serve(
    addr: SocketAddr,
    rt: &tokio::runtime::Handle,
    ctx: egui::Context,
) -> anyhow::Result<crossbeam_channel::Receiver<Ask>> {
    let (jobs, asks) = crossbeam_channel::unbounded();
    let desk = Desk { jobs, ctx };
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
    Ok(asks)
}
