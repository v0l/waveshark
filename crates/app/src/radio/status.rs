use super::*;

/// What one running channel is doing, for its controls to show.
#[derive(Clone, Copy, Debug, Default)]
pub struct ChannelState {
    pub id: u64,
    pub agc_gain_db: f32,
    pub blanked: f32,
    pub squelch_open: bool,
    pub squelch_db: f32,
    /// The coded squelch heard on the channel now, whatever it is set to:
    /// what a control offers to programme the channel with.
    pub code: Option<dsp::squelch::Coded>,
    pub stereo_blend: f32,
    /// What it is putting into the mix, at its own fader setting, for the
    /// meter beside that fader.
    pub level: f32,
}

/// One spectrum update.
pub struct Frame {
    pub db: Vec<f32>,
    /// The same frame as the waterfall reads it, which is a different
    /// detector: a trace is watched to judge a level and a waterfall to
    /// notice that something happened.
    pub wf: Vec<f32>,
    pub adc: nodes::AdcHealth,
    pub center: f64,
    pub rate: f64,
    /// Spectrum stages the operator added, each covering whatever was wired
    /// into it rather than the span.
    pub extra: Vec<Spectrum>,
}

/// One extra spectrum, as the interface draws it.
#[derive(Clone, Debug, PartialEq)]
pub struct Spectrum {
    /// The patch stage it belongs to.
    pub tag: u64,
    pub db: Vec<f32>,
    pub center: f64,
    pub rate: f64,
}

/// A source the detector has, or recently had, open.
#[derive(Clone, Copy, Debug)]
pub struct SeenSource {
    pub source: crate::chain::LiveSource,
    pub last_seen: std::time::Instant,
    pub live: bool,
}

/// How long a closed source stays on the waterfall.
pub const SOURCE_LINGER: std::time::Duration = std::time::Duration::from_secs(6);

/// Blocks of speed history kept for the sparkline in the head. At a few
/// hundred blocks a second this is a second or two of the recent past, which
/// is as far back as a reading anybody can act on goes.
pub const SPEED_HISTORY: usize = 96;

/// The levels as the nodes hold them, with the revision that moves whenever
/// something other than the strip changed one.
///
/// A revision rather than a comparison: the strip sends its own levels down
/// and reads these back, and only a change it did not make should move its
/// faders.
#[derive(Clone, Debug, Default)]
pub struct Levels {
    pub rev: u64,
    pub audio: crate::chain::MixLevels,
    pub channels: Vec<crate::chain::ChannelLevels>,
}

/// Every fader in the running graph, as the strip draws them. A level is
/// set on one by its stage id, the same route the chain view uses.
#[derive(Clone, Debug, Default)]
pub struct Strips {
    pub inputs: Vec<crate::chain::StripState>,
}

pub struct Status {
    pub dropped: AtomicU64,
    pub running: AtomicBool,
    pub audio_backlog: AtomicU64,
    /// How fast the graph ran each block against the time that block
    /// covered, newest last. One is exactly real time: below it the receiver
    /// cannot keep up and the radio will start dropping samples, and how far
    /// above it the trace sits is the headroom left for another channel.
    pub(super) speed: parking_lot::Mutex<std::collections::VecDeque<f32>>,
    pub error: parking_lot::Mutex<Option<String>>,
    /// What the last rebuild could not put in the graph: a front end the span
    /// cannot hold, a channel too near its edge.
    ///
    /// Its own slot rather than a second use of `error`, because the two are
    /// different in kind. A fault happened once; this is a standing verdict on
    /// the graph that is running, republished by every rebuild, and it used to
    /// be written over `error` a few lines after a refused edit had been
    /// reported there, so the operator never saw why their edit went back.
    pub refused: parking_lot::Mutex<Option<String>>,
    /// Stereo separation currently applied, as f32 bits.
    pub(super) blend: AtomicU32,

    /// The radio's own controls, republished whenever one of them moves.
    pub(super) radio: parking_lot::Mutex<RadioControls>,
    /// What each running channel is doing, one entry per channel.
    pub(super) channels: parking_lot::Mutex<Vec<ChannelState>>,
    /// Station name, programme type and radiotext per channel, for the WFM
    /// channels that are decoding RDS. Keyed by channel id: two channels on
    /// two stations each have their own, and sharing one would print the
    /// first channel's name over every other.
    pub(super) stations: parking_lot::Mutex<Vec<(u64, StationInfo)>>,
    pub(super) decoding: parking_lot::Mutex<Vec<(u64, crate::chain::Decoding)>>,
    /// The picture the video bus is publishing, when anything is producing
    /// one.
    ///
    /// One field rather than a stream: a field is half a megabyte, the
    /// interface redraws when it likes, and a viewer that is two fields
    /// behind is showing something that was true 40 ms ago. The frame is the
    /// `Arc` the front end made, so republishing it copies a pointer.
    pub(super) video: parking_lot::Mutex<Option<common::VideoFrame>>,
    /// Every input of the video bus: which one, what it is called, and how
    /// complete its last picture was. What a pane offers to switch between.
    pub(super) video_inputs: parking_lot::Mutex<Vec<crate::chain::VideoInput>>,
    /// The television multiplexes being decoded, and the services on them.
    pub(super) programmes: parking_lot::Mutex<Vec<crate::videobus::Offered>>,
    /// Pictures written to disk, newest last, so a view can say where they
    /// went without watching the directory itself.
    pub(super) pictures: parking_lot::Mutex<Vec<std::path::PathBuf>>,
    /// Shape of the chain currently demodulating, republished on every rebuild.
    pub(super) chain: parking_lot::Mutex<Option<pipeline::graph::Topology>>,
    pub(super) waiting: parking_lot::Mutex<Vec<crate::chain::Waiting>>,
    /// What each scope stage in the chain is seeing, by node id.
    pub(super) scopes: parking_lot::Mutex<Vec<(usize, nodes::ScopeFrame)>>,
    /// Delay through that chain in milliseconds, as f32 bits.
    pub(super) chain_latency: AtomicU32,
    /// Packets decoded across the whole span since the radio started.
    pub decoded: AtomicU64,
    /// Channels each bank is splitting the span into, zero when decoding is
    /// off. Narrow ones run the OOK front end, wide ones the FSK front end.
    pub scan_channels: AtomicU64,
    pub scan_channels_wide: AtomicU64,
    /// Whether a band is being watched for sources, and the sources open
    /// right now or closed within the last few seconds, republished every
    /// block for the waterfall to mark.
    ///
    /// The recently closed ones are the point. A sensor's burst lasts tens
    /// of milliseconds, and a mark that only lasts as long as the source is
    /// open is a flash nobody can read.
    pub sources_on: AtomicBool,
    pub sources: parking_lot::Mutex<Vec<SeenSource>>,
    /// Aircraft whose address has proved itself, when tuned to 1090 MHz.
    pub aircraft: AtomicU64,
    /// The aircraft the tracker in the graph is holding, republished at the
    /// display's frame rate.
    pub track_list: parking_lot::Mutex<Vec<crate::tracks::Track>>,
    /// The transcriber itself: which model, where it is, what it is running
    /// on and whether it is reading anything. `None` where the graph has no
    /// transcriber, which is every build made without the `stt` feature.
    pub transcriber: parking_lot::Mutex<Option<crate::transcripts::Engine>>,
    /// The call recorder: whether it is on, what it has written, and where.
    /// `None` until a receiver is built.
    pub recorder: parking_lot::Mutex<Option<crate::calllog::Recorder>>,
    /// What has been said, as the receiver's own transcript rather than a
    /// copy of it: the node writes into this from the radio thread and the
    /// view takes a snapshot when its sequence number moves. Empty until a
    /// receiver is built, which is what an interface with no radio shows.
    pub transcript: parking_lot::Mutex<crate::transcripts::SharedLog>,
    /// What the raw span capture has written, and where. Off unless somebody
    /// switched it on, which is the usual state.
    pub capture_on: AtomicBool,
    pub capture_bytes: AtomicU64,
    /// What the whole capture folder holds, which is what the limit is on.
    pub capture_folder: AtomicU64,
    pub capture_full: AtomicBool,
    pub capture_file: parking_lot::Mutex<Option<String>>,
    /// An armed capture: waiting for a signal, how many files it has opened,
    /// and what its threshold and the span come to right now. The levels are
    /// dBFS as `f32` bits, and the threshold is `f32::NEG_INFINITY` until a
    /// relative one has a floor to be relative to.
    pub capture_armed: AtomicBool,
    pub capture_bursts: AtomicU64,
    pub capture_level_db: std::sync::atomic::AtomicU32,
    pub capture_threshold_db: std::sync::atomic::AtomicU32,
    /// Size of the day's log file, and whether it has stopped growing.
    pub log_bytes: AtomicU64,
    pub log_full: std::sync::atomic::AtomicBool,
    /// What each packet feed is doing, for the packet log settings.
    pub feeds: parking_lot::Mutex<Vec<crate::chain::FeedStatus>>,
    /// Whether anything the tracker can resolve a position from is running,
    /// locally or from a feed.
    pub tracking: AtomicBool,
    /// Software zoom currently applied, 1 for none.
    pub zoom: AtomicU64,
    /// Whether the operator owns the shape of the graph.
    pub manual: AtomicBool,
    /// Whether this radio can transmit at all, so the strip knows whether to
    /// offer a key at all rather than offering one that always fails.
    pub can_transmit: AtomicBool,
    /// The channel being transmitted on, or zero for none. Half duplex, so
    /// there is one of these and not one per channel: the radio cannot key
    /// two channels at once and the interface should not be able to say so.
    pub keyed: AtomicU64,
    pub talk_ready: AtomicBool,
    /// Transfers the radio sent as silence during the last transmission.
    pub tx_underruns: AtomicU64,
    /// What the microphone is hearing, as f32 bits.
    pub mic_level: AtomicU32,
    /// The microphone is arriving clipped from the capture side.
    pub mic_clipped: AtomicBool,
    /// What a vox is deciding on, as f32 bits, whether it says the key
    /// should be down, and whether it is being held up by the receiver's own
    /// audio. The level is not the microphone's: it is the audio that would
    /// go on air, which is the number the threshold is compared with.
    pub vox_level: AtomicU32,
    pub vox_open: AtomicBool,
    pub vox_held: AtomicBool,
    /// The radio's transmit gain, in dB, as the device took it.
    pub tx_gain_db: AtomicU32,
    /// The levels as the nodes hold them, republished when a setting made
    /// through the chain view changed one, so the strip can follow.
    pub(super) levels: parking_lot::Mutex<Levels>,
    /// The patch the receiver is actually running, which is not always the
    /// one last sent: an edit that will not build is refused and the previous
    /// one goes back.
    pub(super) patch: parking_lot::Mutex<Option<(crate::patch::Patch, crate::patch::Patch)>>,
    /// Bumped whenever the radio thread replaces it, so the interface can
    /// tell its own edit from one being handed back.
    pub patch_rev: AtomicU64,
    /// Bursts written to the packet log since the receiver started.
    pub logged: AtomicU64,
    /// What the survey holds, and how many receptions have been attributed to
    /// a device since the receiver started. Zero when nothing is recording.
    pub survey_devices: AtomicU64,
    pub survey_sightings: AtomicU64,
    pub survey_heard: AtomicU64,
    /// What the WiGLE feed is doing: what is spooled, what has been sent, and
    /// why the last attempt failed.
    pub wigle: parking_lot::Mutex<Option<nodes::WigleStatus>>,
    /// The same for the beaconDB feed.
    pub beacondb: parking_lot::Mutex<Option<nodes::BeaconDbStatus>>,
    /// The walk over a band: where it is, what it has heard and whether it
    /// has stopped on something.
    pub band_scan: parking_lot::Mutex<Option<nodes::ScanStatus>>,
    /// What is on each channel: one row per transmitter and one per channel.
    pub channel_map: parking_lot::Mutex<Option<nodes::ChannelStatus>>,
    /// What the heatmap holds and where the last export went.
    pub heatmap: parking_lot::Mutex<Option<crate::heatmap::HeatmapStatus>>,
    /// And for the feed into the house: the broker, whether it is up, and
    /// how many devices have been announced to it.
    pub homeassistant: parking_lot::Mutex<Option<nodes::HomeAssistantStatus>>,
    /// Every input of the bus, and where the bus is in the graph, so a strip
    /// the operator drew can be given a level by the same route the chain
    /// view uses.
    pub(super) strips: parking_lot::Mutex<Strips>,
    /// What each voice source put into the mix last block, by the
    /// conversation it belongs to. The meter on a call's own row, which
    /// separates "nothing was decoded" from "it was decoded and you still
    /// cannot hear it": two different faults that sound identical.
    pub(super) call_levels: parking_lot::Mutex<Vec<(common::ConversationKey, f32)>>,
    /// What the bus mixed last block, labelled: what is being heard now.
    pub(super) playing: parking_lot::Mutex<Vec<crate::mix::bus::Playing>>,
    /// Who the bus is hearing, and who it has just stopped hearing, since the
    /// interface last took them. Appended by the radio thread every block
    /// and drained by the interface every frame: the ending of a call is
    /// reported once and must not be lost between two frames.
    pub heard: parking_lot::Mutex<Vec<crate::mix::heard::LiveCall>>,
    /// The TETRA cells heard and their key state, for the key manager.
    pub(super) tetra_keys: parking_lot::Mutex<Vec<nodes::tetra_nodes::KeyStatus>>,
    /// Peak of the whole mix as it left for the speaker, and of the call
    /// bus's share of it, for the meters beside the master and call faders.
    pub(super) out_level: AtomicU32,
    pub(super) call_level: AtomicU32,
    /// What the call bus's gain control is adding, in dB, as f32 bits.
    pub(super) call_gain_db: AtomicU32,
    /// Seconds of playback left in the replay stage, as f32 bits: what the
    /// strip shows a playback by, since a replay is on no channel.
    pub(super) replay_left_s: AtomicU32,
}

/// Everything the radio itself can be set to, and what it is set to now.
///
/// Read back from the driver rather than remembered by the interface, because
/// the hardware quantises: ask an R820T for 30 dB and it gives 29.7, ask a
/// HackRF's LNA for 20 and it gives 16. A control showing the request rather
/// than the result is lying about the receiver.
#[derive(Clone, Debug)]
pub struct RadioControls {
    pub stages: Vec<(common::GainStage, GainMode)>,
    /// The transmit gain stages, empty on a receiver. Kept apart from the
    /// receive ones because they are different hardware: a HackRF's transmit
    /// chain shares nothing with its LNA and baseband VGA.
    pub tx_stages: Vec<common::GainStage>,
    pub toggles: Vec<common::Toggle>,
    pub choices: Vec<common::Choice>,
    /// Plain numbers the driver takes, such as a per-tuner frequency trim on
    /// a stitched receiver.
    pub numbers: Vec<common::Number>,
    /// Read by the agent surface, which reports the whole control set; the
    /// settings modal keeps its own copy.
    #[cfg_attr(not(feature = "mcp"), allow(dead_code))]
    pub ppm: f64,
    /// What the dial reads above the tuner, in hertz, and zero for an aerial
    /// straight into the radio.
    #[cfg_attr(not(feature = "mcp"), allow(dead_code))]
    pub offset: f64,
    /// Where the tuner reaches, in hertz: the lowest and highest of its
    /// ranges. What the dial is clamped to, which used to be the RTL-SDR's
    /// 24 to 1766 MHz whatever radio was connected.
    pub reach: (f64, f64),
    /// Where it transmits, or `None` for a radio that does not. Published
    /// beside the receive reach because a control that offers to key a
    /// frequency the radio cannot reach is a control that fails when it is
    /// pressed.
    pub tx_reach: Option<(f64, f64)>,
    /// Whether the tuner can be moved at all. A network stream is pinned by
    /// whoever feeds it, and its dial is a readout.
    pub tunable: bool,
    /// Where one tuner's span ends and the next begins, on a receiver made of
    /// several. Empty for one radio.
    pub seams: Vec<f64>,
}

impl Default for RadioControls {
    fn default() -> Self {
        Self {
            stages: Vec::new(),
            tx_stages: Vec::new(),
            toggles: Vec::new(),
            choices: Vec::new(),
            numbers: Vec::new(),
            ppm: 0.0,
            offset: 0.0,
            reach: (24e6, 1766e6),
            tx_reach: None,
            tunable: true,
            seams: Vec::new(),
        }
    }
}

impl RadioControls {
    /// The correction and the converter come off the device rather than out
    /// of the driver: a driver that cannot correct itself reports zero, which
    /// would throw away what was just typed, and the reach on the aerial's
    /// side is not the reach of the tuner.
    pub(super) fn read(dev: &dyn common::Device) -> Self {
        let (ppm, offset) = (dev.asked_ppm(), dev.offset());
        let now = dev.gains();
        let stages = dev
            .info()
            .gain_stages
            .iter()
            .map(|st| {
                let mode = now
                    .iter()
                    .find(|(n, _)| *n == st.name)
                    .map(|(_, m)| *m)
                    .unwrap_or(GainMode::Manual(*st.range.start()));
                (st.clone(), mode)
            })
            .collect();
        let tx_stages = dev.info().tx.as_ref().map(|t| t.gain_stages.clone()).unwrap_or_default();
        Self {
            stages,
            tx_stages,
            toggles: dev.toggles(),
            choices: dev.choices(),
            numbers: dev.numbers(),
            ppm,
            offset,
            // Already on the aerial's side of the converter: the front end
            // moves the ranges it reports with the offset.
            reach: dev.reach(),
            tx_reach: dev.info().tx.as_ref().and_then(|t| {
                let offset = dev.tuning().offset;
                let lo =
                    t.ranges.iter().map(|r| r.range.start().as_f64()).fold(f64::INFINITY, f64::min);
                let hi = t.ranges.iter().map(|r| r.range.end().as_f64()).fold(0.0f64, f64::max);
                (lo.is_finite() && hi > lo)
                    .then(|| ((lo + offset).max(0.0), (hi + offset).max(0.0)))
            }),
            // The driver's own answer: a capture is pinned by whoever
            // recorded it and a shared network tuner by whoever feeds it,
            // where an rtl_tcp server on the same kind of socket retunes.
            tunable: dev.info().tunable,
            seams: dev.seams().iter().map(|h| h.as_f64()).collect(),
        }
    }
}

/// What the UI shows about the tuned station.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct StationInfo {
    pub pi: Option<u16>,
    pub name: Option<String>,
    pub pty: Option<&'static str>,
    pub radiotext: Option<String>,
    /// Groups accepted and blocks rejected. The ratio is the honest measure of
    /// RDS reception: a station can be loud and still undecodable.
    pub groups: u64,
    pub block_errors: u64,
    pub synced: bool,
}

impl StationInfo {
    pub fn is_empty(&self) -> bool {
        self.pi.is_none() && self.name.is_none() && self.radiotext.is_none()
    }
}

impl Default for Status {
    fn default() -> Self {
        Self {
            dropped: AtomicU64::new(0),
            running: AtomicBool::new(false),
            audio_backlog: AtomicU64::new(0),
            speed: parking_lot::Mutex::new(std::collections::VecDeque::with_capacity(
                SPEED_HISTORY,
            )),
            survey_devices: AtomicU64::new(0),
            survey_sightings: AtomicU64::new(0),
            survey_heard: AtomicU64::new(0),
            wigle: parking_lot::Mutex::new(None),
            beacondb: parking_lot::Mutex::new(None),
            band_scan: parking_lot::Mutex::new(None),
            channel_map: parking_lot::Mutex::new(None),
            heatmap: parking_lot::Mutex::new(None),
            homeassistant: parking_lot::Mutex::new(None),
            strips: parking_lot::Mutex::new(Strips::default()),
            call_levels: parking_lot::Mutex::new(Vec::new()),
            playing: parking_lot::Mutex::new(Vec::new()),
            heard: parking_lot::Mutex::new(Vec::new()),
            tetra_keys: parking_lot::Mutex::new(Vec::new()),
            out_level: AtomicU32::new(0),
            call_level: AtomicU32::new(0),
            call_gain_db: AtomicU32::new(0),
            replay_left_s: AtomicU32::new(0),
            error: parking_lot::Mutex::new(None),
            refused: parking_lot::Mutex::new(None),
            blend: AtomicU32::new(0),

            radio: parking_lot::Mutex::new(RadioControls::default()),
            channels: parking_lot::Mutex::new(Vec::new()),
            stations: parking_lot::Mutex::new(Vec::new()),
            decoding: parking_lot::Mutex::new(Vec::new()),
            video: parking_lot::Mutex::new(None),
            video_inputs: parking_lot::Mutex::new(Vec::new()),
            programmes: parking_lot::Mutex::new(Vec::new()),
            pictures: parking_lot::Mutex::new(Vec::new()),
            chain: parking_lot::Mutex::new(None),
            waiting: parking_lot::Mutex::new(Vec::new()),
            scopes: parking_lot::Mutex::new(Vec::new()),
            chain_latency: AtomicU32::new(0),
            decoded: AtomicU64::new(0),
            scan_channels: AtomicU64::new(0),
            scan_channels_wide: AtomicU64::new(0),
            sources_on: AtomicBool::new(false),
            sources: parking_lot::Mutex::new(Vec::new()),
            aircraft: AtomicU64::new(0),
            logged: AtomicU64::new(0),
            track_list: parking_lot::Mutex::new(Vec::new()),
            transcriber: parking_lot::Mutex::new(None),
            recorder: parking_lot::Mutex::new(None),
            transcript: parking_lot::Mutex::new(Default::default()),
            capture_on: AtomicBool::new(false),
            capture_bytes: AtomicU64::new(0),
            capture_folder: AtomicU64::new(0),
            capture_full: AtomicBool::new(false),
            capture_file: parking_lot::Mutex::new(None),
            capture_armed: AtomicBool::new(false),
            capture_bursts: AtomicU64::new(0),
            capture_level_db: std::sync::atomic::AtomicU32::new(f32::NEG_INFINITY.to_bits()),
            capture_threshold_db: std::sync::atomic::AtomicU32::new(f32::NEG_INFINITY.to_bits()),
            log_bytes: AtomicU64::new(0),
            log_full: std::sync::atomic::AtomicBool::new(false),
            feeds: parking_lot::Mutex::new(Vec::new()),
            tracking: AtomicBool::new(false),
            zoom: AtomicU64::new(1),
            manual: AtomicBool::new(false),
            can_transmit: AtomicBool::new(false),
            keyed: AtomicU64::new(0),
            talk_ready: AtomicBool::new(true),
            tx_underruns: AtomicU64::new(0),
            mic_level: AtomicU32::new(0),
            mic_clipped: AtomicBool::new(false),
            vox_level: AtomicU32::new(0),
            vox_open: AtomicBool::new(false),
            vox_held: AtomicBool::new(false),
            tx_gain_db: AtomicU32::new(0),
            patch: parking_lot::Mutex::new(None),
            levels: parking_lot::Mutex::new(Levels::default()),
            patch_rev: AtomicU64::new(0),
        }
    }
}

impl Status {
    /// Publish the patch the receiver is running, and the one it drew for
    /// itself underneath the operator's edits.
    pub(super) fn set_patch(&self, rx: &crate::chain::Receiver) {
        *self.patch.lock() = Some((rx.patch().clone(), rx.base().clone()));
        self.patch_rev.fetch_add(1, Ordering::Relaxed);
    }

    /// The patch the receiver is running, the one it drew before the edits,
    /// and which revision they are.
    pub fn patch(&self) -> (u64, Option<(crate::patch::Patch, crate::patch::Patch)>) {
        (self.patch_rev.load(Ordering::Relaxed), self.patch.lock().clone())
    }

    /// The receiver's transcript, for a view that wants to read or clear it.
    pub fn transcript(&self) -> crate::transcripts::SharedLog {
        self.transcript.lock().clone()
    }

    /// The levels as the graph holds them, and a revision that moves only
    /// when something other than the strip changed one.
    pub fn levels(&self) -> Levels {
        self.levels.lock().clone()
    }

    pub(super) fn set_levels(
        &self,
        audio: crate::chain::MixLevels,
        channels: Vec<crate::chain::ChannelLevels>,
    ) {
        let mut held = self.levels.lock();
        *held = Levels { rev: held.rev + 1, audio, channels };
    }

    pub fn blend(&self) -> f32 {
        f32::from_bits(self.blend.load(Ordering::Relaxed))
    }

    /// The radio's gain stages and switches, as they currently are.
    /// What the call bus's gain control is adding, in dB.
    pub fn call_gain_db(&self) -> f32 {
        f32::from_bits(self.call_gain_db.load(Ordering::Relaxed))
    }

    /// The TETRA cells heard and their key state, for the key manager.
    pub fn tetra_keys(&self) -> Vec<nodes::tetra_nodes::KeyStatus> {
        self.tetra_keys.lock().clone()
    }

    /// What each voice source last put into the mix, for the meters.
    pub fn call_levels(&self) -> Vec<(common::ConversationKey, f32)> {
        self.call_levels.lock().clone()
    }

    /// What is being heard now: everything the bus mixed last block, by
    /// system, frequency, group and caller, with its level.
    /// What the audio bus is mixing, for anything that wants to ask. No pane
    /// draws it: the call list is where a conversation appears and the strip
    /// is where a channel does.
    #[cfg_attr(not(test), allow(dead_code))]
    pub fn playing(&self) -> Vec<crate::mix::bus::Playing> {
        self.playing.lock().clone()
    }

    /// Every fader as the strip draws it.
    pub fn strips(&self) -> Strips {
        self.strips.lock().clone()
    }

    pub(super) fn set_strips(&self, mut inputs: Vec<crate::chain::StripState>) {
        let mut held = self.strips.lock();
        for s in &mut inputs {
            if let Some(prev) = held.inputs.iter().find(|p| p.stage == s.stage) {
                s.level = s.level.max(prev.level * METER_FALL);
            }
        }
        *held = Strips { inputs };
    }

    /// The mix's own level, and the call bus's share of it.
    pub fn out_level(&self) -> f32 {
        f32::from_bits(self.out_level.load(Ordering::Relaxed))
    }

    pub fn call_level(&self) -> f32 {
        f32::from_bits(self.call_level.load(Ordering::Relaxed))
    }

    /// Seconds of a played-back over still to come, or zero.
    pub fn replay_left_s(&self) -> f32 {
        f32::from_bits(self.replay_left_s.load(Ordering::Relaxed))
    }

    /// Rise instantly, fall slowly. A meter that tracked the block peak both
    /// ways flickers at the block rate and reads as noise; speech is mostly
    /// gaps, and the gaps are not what anybody is trying to see.
    pub(super) fn set_level(cell: &AtomicU32, peak: f32) {
        let prev = f32::from_bits(cell.load(Ordering::Relaxed));
        cell.store(peak.max(prev * METER_FALL).to_bits(), Ordering::Relaxed);
    }

    pub fn radio(&self) -> RadioControls {
        self.radio.lock().clone()
    }

    pub(super) fn set_radio(&self, c: RadioControls) {
        *self.radio.lock() = c;
    }

    /// What every running channel is doing.
    pub fn channel_states(&self) -> Vec<ChannelState> {
        self.channels.lock().clone()
    }

    /// One channel's state by id, for the controls that belong to it.
    pub fn channel_state(&self, id: u64) -> Option<ChannelState> {
        self.channels.lock().iter().find(|c| c.id == id).copied()
    }

    pub(super) fn set_channel_states(&self, mut states: Vec<ChannelState>) {
        let mut held = self.channels.lock();
        for s in &mut states {
            if let Some(prev) = held.iter().find(|p| p.id == s.id) {
                s.level = s.level.max(prev.level * METER_FALL);
            }
        }
        *held = states;
    }

    pub(super) fn set_blend(&self, v: f32) {
        self.blend.store(v.to_bits(), Ordering::Relaxed);
    }

    pub fn chain(&self) -> Option<pipeline::graph::Topology> {
        self.chain.lock().clone()
    }

    pub fn waiting(&self) -> Vec<crate::chain::Waiting> {
        self.waiting.lock().clone()
    }

    pub(super) fn push_speed(&self, x: f32) {
        let mut h = self.speed.lock();
        if h.len() == SPEED_HISTORY {
            h.pop_front();
        }
        h.push_back(x);
    }

    /// The recent speed trace, oldest first.
    pub fn speed_history(&self) -> Vec<f32> {
        self.speed.lock().iter().copied().collect()
    }

    pub fn chain_latency(&self) -> f64 {
        f64::from(f32::from_bits(self.chain_latency.load(Ordering::Relaxed)))
    }

    /// The scopes' latest frames, for the inspector.
    pub fn scopes(&self) -> Vec<(usize, nodes::ScopeFrame)> {
        self.scopes.lock().clone()
    }

    pub(super) fn set_chain(&self, t: Option<pipeline::graph::Topology>, latency_ms: f64) {
        *self.chain.lock() = t;
        self.chain_latency.store((latency_ms as f32).to_bits(), Ordering::Relaxed);
    }

    /// What one channel is receiving, or nothing when it is not decoding RDS.
    pub fn station_for(&self, id: u64) -> Option<StationInfo> {
        self.stations.lock().iter().find(|(k, _)| *k == id).map(|(_, s)| s.clone())
    }

    pub fn decoding_for(&self, id: u64) -> Option<crate::chain::Decoding> {
        self.decoding.lock().iter().find(|(k, _)| *k == id).map(|(_, d)| d.clone())
    }

    pub(super) fn set_decoding(&self, now: Vec<(u64, crate::chain::Decoding)>) {
        let mut cur = self.decoding.lock();
        if *cur != now {
            *cur = now;
        }
    }

    /// The first channel's station, for the headless probe, which runs one.
    pub fn station(&self) -> StationInfo {
        self.stations.lock().first().map(|(_, s)| s.clone()).unwrap_or_default()
    }

    pub(super) fn set_station(
        &self,
        id: u64,
        s: &dsp::rds::Station,
        groups: u64,
        errors: u64,
        synced: bool,
    ) {
        let next = StationInfo {
            pi: s.pi,
            name: s.name.clone(),
            pty: s.pty_name(),
            radiotext: s.radiotext.clone(),
            groups,
            block_errors: errors,
            synced,
        };
        let mut cur = self.stations.lock();
        // Only take the write cost when something actually changed; this runs
        // on every audio block, for every channel.
        match cur.iter_mut().find(|(k, _)| *k == id) {
            Some((_, cur)) if *cur != next => *cur = next,
            Some(_) => {}
            None => cur.push((id, next)),
        }
    }

    /// The picture being received, if any.
    pub fn video(&self) -> Option<common::VideoFrame> {
        self.video.lock().clone()
    }

    /// What the video bus is receiving, whether or not it is being watched.
    pub fn video_inputs(&self) -> Vec<crate::chain::VideoInput> {
        self.video_inputs.lock().clone()
    }

    /// The television multiplexes being decoded, with their services.
    pub fn programmes(&self) -> Vec<crate::videobus::Offered> {
        self.programmes.lock().clone()
    }

    /// Pictures written to disk this session, newest last.
    pub fn pictures(&self) -> Vec<std::path::PathBuf> {
        self.pictures.lock().clone()
    }

    pub(super) fn set_video_inputs(&self, inputs: Vec<crate::chain::VideoInput>) {
        let mut cur = self.video_inputs.lock();
        if *cur != inputs {
            *cur = inputs;
        }
    }

    /// Publish a field, or clear the pane when the receiver stops producing
    /// them: a still picture left on the screen after the transmitter went
    /// away is the worst thing a video pane can do.
    ///
    /// A picture is new when anything about it is, not when its number is.
    /// The number counts fields for a camera, but names the picture for a
    /// still, so an SSTV transmission keeps one number for two minutes while
    /// its lines fill in: comparing numbers alone published the first line
    /// and nothing after it.
    pub(super) fn set_video(&self, frame: Option<common::VideoFrame>) -> bool {
        let mut cur = self.video.lock();
        let same = match (cur.as_ref(), frame.as_ref()) {
            (Some(a), Some(b)) => {
                a.sequence == b.sequence
                    && a.lines_seen == b.lines_seen
                    && a.channel_hz == b.channel_hz
                    && (a.width, a.height) == (b.width, b.height)
            }
            (None, None) => true,
            _ => false,
        };
        if !same {
            *cur = frame;
        }
        !same
    }

    /// Drop the stations of channels that are no longer running, so a name
    /// cannot linger over a channel that has been retuned or removed.
    pub(super) fn keep_stations(&self, ids: &[u64]) {
        let mut cur = self.stations.lock();
        cur.retain(|(id, _)| ids.contains(id));
        if cur.is_empty() {
            self.set_blend(0.0);
        }
    }
}
