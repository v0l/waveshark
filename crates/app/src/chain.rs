//! The receiver as one graph.
//!
//! Everything the radio thread does to a block of samples is a node in here:
//! the spectrum behind the waterfall, the recorder's ring, the channel banks
//! that sweep a whole span, the Mode S decoder on 1090, and a branch per
//! channel being listened to. The alternative, which this replaces, was a
//! handful of independent objects each fed the same buffer by hand. Every one
//! of them was a chain the chain view could not draw and a set of parameters
//! nothing generic could reach.
//!
//! # Rebuilding
//!
//! A graph is fixed once built, and what the receiver is doing is not: a
//! channel appears, the dial moves onto 1090, the span doubles. So the shape
//! changes by building a new graph out of the *same nodes*, through
//! [`pipeline::Graph::into_parts`]. That distinction matters more than it
//! looks: rebuilding from fresh nodes would reset every branch that was left
//! alone, so adding a second channel would cost the first one its RDS
//! station, its AGC convergence and its detector's noise floor.
//!
//! A node is only reused where reusing it is meaningful. Anything whose
//! coefficients depend on the span, or a channel whose offset or mode
//! changed, is built again, because a filter designed for the old rate is not
//! the same filter.

use std::collections::HashMap;

use crate::audiobus::StripParam;
use crate::scanners::Front;
use common::{Hz, Result, C32};
use dsp::rds::Station;
use nodes::{AgcNode, BankNode, SpectrumNode, SquelchNode, WfmDemodNode};
use pipeline::graph::{NodePart, Topology};
use pipeline::{Graph, GraphBuilder, NodeId, Out, PortKind, StreamSpec};

use crate::radio::{ChanMode, ChannelSpec, DecodeRecord, Demod};
use crate::record::Recorder;
use std::path::PathBuf;

/// Channel width for the OOK bank. Below this the measurements show no further
/// gain, because the sensor's own bandwidth and its carrier offset start to
/// matter more than the noise saved.
pub const OOK_CHANNEL_HZ: f64 = 31_250.0;

/// Audio sample rate every channel branch aims for.
const AUDIO_HZ: f64 = 48_000.0;

/// How much of the last spectrum frame the next one keeps, until the
/// interface says otherwise.
pub const DEFAULT_SMOOTHING: f32 = 0.35;

/// Grid the extracted band's centre is snapped to.
///
/// The band a bank works in has to be the same band from one retune to the
/// next, or the channel grid slides under the signals and every bank rebuilds
/// itself. Snapping the centre means a band clipped slightly differently by
/// the span edge still resolves to the same extraction, and it only moves when
/// the clipping moves it by a whole step.
const SUBBAND_GRID_HZ: f64 = 100_000.0;

/// Room left above the wanted bandwidth for the decimator's transition band.
const SUBBAND_HEADROOM: f64 = 1.15;

/// A band cut out of the span for a bank to channelize.
///
/// Without this a bank divides the whole span, and the span is the wrong
/// number twice over: at 60 MS/s the 1024 channel ceiling gives 60 kHz
/// channels where a sensor needs 25, and the grid is anchored to the dial, so
/// scrubbing moves every channel and resets every detector.
#[derive(Clone, Copy, Debug, PartialEq)]
struct SubBand {
    /// Centre of the extracted band, snapped to the grid.
    center: f64,
    /// Decimation from the span's rate. A power of two, so that a band whose
    /// clipping changes slightly keeps the same rate.
    factor: usize,
    /// Bandwidth that has to survive the decimator.
    need: f64,
}

impl SubBand {
    /// `min_rate` is the slowest the front end behind this can work at: Mode S
    /// needs 2 MS/s for its one microsecond bits, an FM channel needs enough
    /// left for its own audio decimation to land on a whole number.
    fn plan(band: (f64, f64), span_rate: f64, min_rate: f64) -> Self {
        let (lo, hi) = band;
        let center = ((lo + hi) / 2.0 / SUBBAND_GRID_HZ).round() * SUBBAND_GRID_HZ;
        // Measured from the snapped centre, so the snap cannot push an edge
        // of the wanted band outside what is kept.
        let need = 2.0 * (lo - center).abs().max((hi - center).abs());
        let floor = need * SUBBAND_HEADROOM.max(min_rate / need.max(1.0));
        let mut factor = 1usize;
        while span_rate / (factor * 2) as f64 >= floor && factor < 4096 {
            factor *= 2;
        }
        Self { center, factor, need }
    }

    /// Rate the banks will see.
    fn rate(&self, span_rate: f64) -> f64 {
        span_rate / self.factor as f64
    }

    /// Identity for node reuse: the band, not the tuning.
    fn key(&self) -> u64 {
        self.center.max(0.0) as u64
    }

    /// Whether extracting this band is worth any nodes at all.
    fn is_whole_span(&self, span_center: f64) -> bool {
        self.factor == 1 && (self.center - span_center).abs() < 1.0
    }
}

/// The narrow CW filter, in Hz.
const CW_FILTER_HZ: f64 = 500.0;

/// What a channel branch was built for. A branch is only reused while all of
/// this is unchanged, since every one of these decides a filter's
/// coefficients or a mixer's shift.
#[derive(Clone, Copy, PartialEq, Debug)]
struct ChanKey {
    mode: u64,
    /// The width every filter was designed at. A change to it is a rebuild,
    /// not a parameter: without this here a width set on the strip was
    /// applied as a level change and reached nothing.
    width_bits: u64,
    rate_bits: u64,
}

impl ChanKey {
    fn new(spec: &ChannelSpec, rate: f64) -> Self {
        Self {
            mode: spec.mode.key(),
            width_bits: spec.bandwidth().to_bits(),
            rate_bits: rate.to_bits(),
        }
    }
}

/// One listening channel inside the graph.
pub struct Chan {
    pub spec: ChannelSpec,
    /// Whether this channel came through the rebuild with its nodes intact.
    /// A channel built from scratch has forgotten its station and its gain.
    pub kept: bool,
    key: ChanKey,
    tail: Out,
    /// The bus input its audio goes into, which is where its level lives.
    pub port: Option<usize>,
    agc: Option<NodeId>,
    squelch: Option<NodeId>,
    wfm: Option<NodeId>,
    pub audio_rate: f64,
    pub channels: usize,
    /// What the chain cost, for the status line.
    pub detail: String,
    pub agc_gain_db: f32,
    pub squelch_open: bool,
    pub squelch_db: f32,
    pub blend: f32,
    pub station: Station,
    pub rds_stats: (u64, u64, bool),
}

/// One bank sweeping the span.
pub struct Bank {
    pub channels: usize,
}

/// A transmitter the source detector has open right now.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LiveSource {
    pub center_hz: f64,
    pub bandwidth_hz: f64,
    /// How strongly the detector heard it, or `None` for a locked channel,
    /// which is a decision the auto node made rather than a measurement.
    pub snr_db: Option<f32>,
    /// The front end that owns this channel for the rest of the session,
    /// or `None` for a source the detector has open right now.
    ///
    /// The two are different things and the spectrum draws them
    /// differently: a detection is a measurement that lasts as long as the
    /// transmission, and a locked channel is a decision that outlives it.
    pub locked_to: Option<&'static str>,
}

pub struct Receiver {
    graph: Graph,
    /// What the parts of the receiver that are not drawn yet read.
    head: Out,
    /// The graph as a description: what is running, in the operator's terms.
    ///
    /// Also how every node the receiver has to talk to is found: a stage's
    /// id is carried into the graph as a node's tag, so [`Receiver::stage`]
    /// reads the spectrum, the bus or the recorder out of the running graph
    /// by the id the patch derived it under. A field per node held that
    /// answer across rebuilds instead, twenty of them, each cleared and
    /// reassigned by hand.
    patch: crate::patch::Patch,
    /// The graph as the receiver drew it before the operator's edits, which
    /// is what an edited copy of `patch` is read against to find them.
    base: crate::patch::Patch,
    banks: Vec<Bank>,
    /// The source detectors, one per band watched.
    sources: Vec<NodeId>,
    chans: Vec<Chan>,
    /// A recorder waiting for the next rebuild to become a node.
    pending_record: Option<RecordRing>,
    /// A transmitter waiting for its place in the graph, from the key-up that
    /// opened it. Like the recorder's ring, it is handed in once and then
    /// survives rebuilds by coming back out of the pool.
    pending_tx: Option<TxSinks>,
    /// The microphone, for every rebuild that wants one.
    ///
    /// Kept rather than handed in per key, because the microphone stage is
    /// in the graph whether or not anything is keyed and a rebuild that has
    /// no microphone to give it skips the stage: the chain after it was then
    /// built unfed, and keying transmitted a carrier with nothing on it.
    mic: Option<std::sync::Arc<dyn audio::AudioSource>>,
    /// Where the packet log is written, if it is. Held as a directory rather
    /// than an open file so that a rebuild has something to reopen when the
    /// bus itself had to be built again.
    log_dir: Option<PathBuf>,
    /// Size the packet log's folder may reach, or `None` for no limit.
    log_cap: Option<u64>,
    /// What the log folder holds while nothing is writing to it, and when it
    /// was last added up.
    log_folder: u64,
    log_measured: Option<std::time::Instant>,
    /// Where the receiver is: one position, whether it was typed in or came
    /// from a GPS, carrying the quality fields when a fix supplied it.
    ///
    /// One station rather than a position and a fix beside it. The survey
    /// used to record only what the GPS said, so a receiver whose position
    /// was typed in wrote every sighting with an empty position while the map
    /// drew the same receiver on its aerial.
    station: Option<gps::Fix>,
    /// Bursts logged before the last rebuild, since the node holding the
    /// count is replaced by each one.
    logged: u64,
    center: Hz,
    rate: f64,
    /// Rate at the spectrum's own input, which is the span's unless the
    /// operator has put something in front of it.
    spectrum_rate: f64,
    /// What a node said went wrong this block. A node reports once and
    /// stops, so a warning left in the graph's event list is a warning
    /// nobody sees: the capture that would not start said why, to nobody.
    warnings: Vec<String>,
    /// What a decoder asked for that nothing between it and here could
    /// give: a channel outside the span, a retune. The auto node answers
    /// what it can for the decoders it built; these reached the top.
    requests: Vec<(String, pipeline::Request)>,
    /// Spectrum stages the operator added, by patch id. Each is a display of
    /// its own: a patch can watch a decimated band and the whole span at the
    /// same time, which is most of the reason to draw one.
    patch_spectra: Vec<(u64, NodeId)>,
    /// Channels that could not be built, for the status line.
    pub refused: Option<String>,
    /// What was said, which outlives every graph that heard it. Held here
    /// and lent to the transcriber on each rebuild: the node is a stage on
    /// the audio bus, and the bus is rebuilt whenever a channel comes or
    /// goes.
    transcript: crate::transcripts::SharedLog,
}

/// What the receiver should be doing, as opposed to what it is.
pub struct Plan {
    pub center: Hz,
    /// The rate the device is delivering, before zoom.
    pub rate: f64,
    /// Software zoom: the radio keeps sampling at its own rate and everything
    /// downstream sees a decimated copy.
    pub zoom: usize,
    pub dc_block: bool,
    /// Frames a second the spectrum is worth producing.
    pub refresh_hz: f32,
    /// How much of the last spectrum frame the next one keeps.
    pub smoothing: f32,
    pub fft: usize,
    pub channels: Vec<ChannelSpec>,
    /// The levels on the bus that are nobody's channel.
    pub audio: AudioPlan,
    /// The front ends to run, from the scanner table for this span. Empty is
    /// a span nothing is configured for, which costs nothing rather than
    /// sweeping it for sensors that are not there.
    ///
    /// Several, because a span is wide: a couple of megahertz of VHF can hold
    /// a pager channel and a packet channel at once, and both are one
    /// narrowband demodulator each.
    pub fronts: Vec<crate::scanners::FrontAt>,
    /// What the operator changed about the graph, put on top of the one
    /// the receiver draws for itself. Applied whether or not the graph is
    /// being edited: manual mode is a lock on editing, not a different
    /// receiver.
    pub edits: crate::patch::Edits,
    pub record: bool,
    /// Whether the raw span is being written to disk. The stage is always in
    /// the graph and almost always switched off, so this is the switch and
    /// not the presence of a stage.
    pub capture: bool,
    /// Where a raw span capture is written when one is switched on. The
    /// stage is always in the graph, so this is always needed.
    pub capture_dir: PathBuf,
    /// The sample format a capture is written in: the device's own depth.
    /// A twelve bit converter written as bytes throws away its bottom
    /// four bits, and on a quiet band those were the whole signal.
    pub capture_format: common::SampleFormat,
    /// Log every burst the front ends detect.
    pub log: bool,
    /// Other receivers feeding the same packet bus.
    pub feeds: Vec<nodes::FeedSpec>,
    /// The channel being transmitted on, if any, and what it transmits.
    ///
    /// In the plan because the transmitter is part of what the receiver is
    /// doing, so it belongs in the graph the receiver draws: keying adds
    /// stages to the chain view, and they can be tapped and parameterised
    /// like every other stage rather than living in a second graph the
    /// interface never sees.
    pub tx: Option<TxPlan>,
    /// Which calls are heard, which pictures are watched, where the survey
    /// is written and who it is uploaded as.
    ///
    /// In the plan like everything else the receiver is doing, but apart
    /// from the stage settings above because none of these fits in a
    /// `ParamValue`: a subscription is a rule, an account holds a secret,
    /// and a survey is an open file. [`Receiver::apply_settings`] is the one
    /// thing that hands them to the nodes.
    pub settings: PlanSettings,
}

/// What the receiver is doing that no stage setting can carry.
#[derive(Clone, PartialEq)]
pub struct PlanSettings {
    /// Which calls the audio bus mixes.
    pub calls: Vec<crate::audiobus::Subscription>,
    /// Which pictures the video bus publishes.
    pub watching: Vec<crate::videobus::Rule>,
    /// Where the survey is written, if it is.
    pub survey_path: Option<PathBuf>,
    /// Who the receiver uploads to wigle.net as, when it does.
    pub wigle: Option<survey::Account>,
    /// Whether the beaconDB feed is collecting.
    pub beacondb: bool,
    /// Where every device heard is published for Home Assistant to build,
    /// when anywhere.
    pub homeassistant: Option<nodes::Publish>,
}

impl Default for PlanSettings {
    fn default() -> Self {
        Self {
            calls: Vec::new(),
            // Whatever is being received, which is what the video bus itself
            // starts at: a camera the receiver finds should appear without
            // anybody having to ask for it by name first.
            watching: vec![crate::videobus::Rule::Everything],
            survey_path: None,
            wigle: None,
            beacondb: false,
            homeassistant: None,
        }
    }
}

/// The transmit chain the receiver should be drawing.
///
/// Present whenever the radio can transmit and a channel says what it would
/// send, not only while a key is down: the stages are in the graph the whole
/// time, off, so the chain can be read and set up before anything is
/// radiated, and so keying does not rebuild the graph.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TxPlan {
    pub spec: crate::radio::TxSpec,
    pub mode: crate::radio::TxMode,
    /// Where it would transmit: the channel plus its shift.
    pub on_air: Hz,
}

/// The levels on the bus that belong to no one channel: the master every
/// strip runs into, and the one level every call is heard at.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AudioPlan {
    pub master: f32,
    pub muted: bool,
    pub calls: f32,
    pub calls_muted: bool,
    /// Whether calls are levelled before they are mixed.
    pub agc: bool,
}

impl Default for AudioPlan {
    fn default() -> Self {
        Self { master: 0.5, muted: false, calls: 0.8, calls_muted: false, agc: true }
    }
}

/// One channel's settings as the nodes hold them: what the strip owns and an
/// operator can move from the chain view as well as from the strip.
///
/// The line between this and the rest of a [`ChannelSpec`] is the one
/// [`operator_owns`] draws. What is here is read back
/// into the plan on every block; what is not is the plan's alone, and a node
/// that was handed a stale copy of it must not write it back.
#[derive(Clone, Debug, PartialEq)]
pub struct ChannelLevels {
    pub id: u64,
    pub label: String,
    pub volume: f32,
    pub muted: bool,
    pub squelch_db: Option<f32>,
    pub agc: bool,
}

/// One input of the bus, as the strip draws it.
#[derive(Clone, Debug, PartialEq)]
pub struct StripState {
    pub port: usize,
    pub label: String,
    pub volume: f32,
    pub muted: bool,
    /// What it put into the mix last block, after its fader.
    pub level: f32,
    /// Whether it carries speech, which the call list handles, rather than
    /// audio.
    pub voice: bool,
    /// The listening channel feeding it, when one does. A strip with none
    /// is a chain the operator drew.
    pub channel: Option<u64>,
}

/// One input of the video bus, as a pane offers it.
#[derive(Clone, Debug, PartialEq)]
pub struct VideoInput {
    /// What the bus keeps it under, which is what a pane asks to watch.
    pub key: String,
    pub label: String,
    /// How much of its last picture arrived, from nothing to one.
    pub completeness: f32,
}

/// What the transmitter has done since the graph was built: samples handed
/// to the radio, transfers the radio had to fill with silence itself, and
/// what the microphone is hearing.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TxState {
    pub written: u64,
    pub underruns: u64,
    pub mic_peak: f32,
}

/// One feed, as the interface sees it.
#[derive(Clone, Debug)]
pub struct FeedStatus {
    pub spec: nodes::FeedSpec,
    pub connected: bool,
    pub frames: u64,
    pub error: Option<String>,
}

/// Where a receiver's output goes, beyond the audio and the screen.
///
/// Both of these own an open file, so they are handed to the graph rather
/// than created by it, and they survive a rebuild by being carried across.
#[derive(Default)]
pub struct Sinks {
    pub recorder: Option<Recorder>,
    /// Where to write the packet log, if it is being written at all.
    pub packet_log: Option<PathBuf>,
    /// The transmitter, while a channel is keyed.
    ///
    /// Handed in rather than built from a description for the same reason the
    /// recorder is: it owns a radio, and a description cannot carry one.
    pub tx: Option<TxSinks>,
}

/// What a keyed transmission needs from outside the graph.
pub struct TxSinks {
    pub stream: Option<Box<dyn common::TxStream>>,
    /// The microphone, when the channel transmits from one.
    pub mic: Option<std::sync::Arc<dyn audio::AudioSource>>,
}

impl Plan {
    /// The rate everything downstream of the zoom decimator sees.
    pub fn eff_rate(&self) -> f64 {
        self.rate / self.zoom.max(1) as f64
    }
}

impl Receiver {
    pub fn build(plan: &Plan, sinks: Sinks) -> Result<Self> {
        let mut rx = Self {
            // Placeholder, replaced immediately. A graph cannot be built
            // empty and then filled, which is the same constraint that makes
            // rebuilding the interesting case.
            graph: Graph::builder(StreamSpec::iq(plan.rate, plan.center)).build()?,
            head: pipeline::graph::GRAPH_INPUT,
            patch: crate::patch::Patch::default(),
            base: crate::patch::Patch::default(),
            banks: Vec::new(),
            sources: Vec::new(),
            chans: Vec::new(),
            pending_record: None,
            pending_tx: None,
            mic: None,
            log_dir: sinks.packet_log,
            log_cap: Some(crate::packetlog::DEFAULT_MAX_BYTES),
            log_folder: 0,
            log_measured: None,
            station: None,
            logged: 0,
            center: plan.center,
            rate: plan.rate,
            spectrum_rate: plan.eff_rate(),
            warnings: Vec::new(),
            requests: Vec::new(),
            patch_spectra: Vec::new(),
            refused: None,
            transcript: Default::default(),
        };
        // Nothing is in the pool to be reused, so whether the span moved is
        // not a question anything asks of this build.
        rx.assemble(plan, HashMap::new(), sinks.recorder.map(RecordRing::new), sinks.tx, false)?;
        Ok(rx)
    }

    /// Hand over an open transmitter, for the next rebuild to place.
    pub fn set_transmitter(&mut self, tx: Option<TxSinks>) {
        self.pending_tx = tx;
    }

    /// The microphone every rebuild from now on builds the mic stage from.
    pub fn set_microphone(&mut self, mic: Option<std::sync::Arc<dyn audio::AudioSource>>) {
        self.mic = mic;
    }

    /// Key: give the transmit stage a radio, without rebuilding the graph.
    ///
    /// The stages are there whether or not anything is transmitting, so
    /// keying is one node being handed a device rather than a new graph. A
    /// rebuild would restart the spectrum's averaging and every decoder
    /// mid-frame, twice per over.
    pub fn key(&mut self, stream: Box<dyn common::TxStream>) -> bool {
        match self.tx_sink_mut() {
            Some(s) => {
                s.attach(stream);
                true
            }
            None => false,
        }
    }

    /// Unkey: let the queue out and give the radio back.
    pub fn unkey(&mut self) -> u64 {
        match self.tx_sink_mut() {
            Some(s) => {
                let idle = s.underruns();
                s.finish(std::time::Duration::from_secs(1));
                idle
            }
            None => 0,
        }
    }

    pub fn keyed(&self) -> bool {
        self.tx_sink().is_some_and(|s| s.keyed())
    }

    /// A stage of the running graph, read as what it is.
    ///
    /// The stage's patch id is the node's identity across rebuilds, so this
    /// asks the graph rather than a field the last rebuild filled in. A
    /// stage that is not in the graph, or is not the kind asked for, is
    /// absent: the transmit source is a microphone or a tone, and asking for
    /// the microphone answers only when it is one.
    fn stage<T: 'static>(&self, tag: u64) -> Option<&T> {
        self.graph.by_tag(tag).and_then(|id| downcast::<T>(&self.graph, id))
    }

    fn stage_mut<T: 'static>(&mut self, tag: u64) -> Option<&mut T> {
        let id = self.graph.by_tag(tag)?;
        self.graph.node_mut(id)?.as_any_mut().downcast_mut::<T>()
    }

    fn tx_sink_mut(&mut self) -> Option<&mut nodes::TxSinkNode> {
        self.stage_mut::<nodes::TxSinkNode>(derived::TX_RADIO)
    }

    /// What the transmitter has done, for the interface.
    pub fn tx_state(&self) -> Option<TxState> {
        let sink = self.tx_sink()?;
        Some(TxState {
            written: sink.written(),
            underruns: sink.underruns(),
            mic_peak: self.tx_mic().map(|m| m.peak()).unwrap_or(0.0),
        })
    }

    /// Whether the microphone's signal is arriving already clipped.
    pub fn mic_clipped(&self) -> bool {
        self.tx_mic().is_some_and(|m| m.input_clipped())
    }

    fn tx_mic(&self) -> Option<&nodes::MicNode> {
        self.stage::<nodes::MicNode>(derived::TX_SOURCE)
    }

    /// Draw what is going out on the receiver's own span, or stop.
    ///
    /// Only while the radio is deaf, which is the radio thread's to know: a
    /// full duplex radio hears its own transmission for real and mirroring on
    /// top of that would draw it twice.
    pub fn set_tx_monitor(&mut self, on: bool) {
        if let Some(n) = self.stage_mut::<nodes::TxMonitorNode>(derived::TX_MONITOR) {
            n.set_enabled(on);
        }
    }

    fn tx_sink(&self) -> Option<&nodes::TxSinkNode> {
        self.stage::<nodes::TxSinkNode>(derived::TX_RADIO)
    }

    /// Change what the receiver is doing, keeping every node that still means
    /// the same thing.
    pub fn rebuild(&mut self, plan: &Plan) -> Result<()> {
        // Where each channel was listening, absolutely. A channel whose
        // stages come back from the pool but whose frequency moved has to
        // forget its station and its gain: they belong to what it was on.
        let old_freq: HashMap<u64, f64> = self
            .chans
            .iter()
            .map(|c| (c.spec.id, self.center.as_f64() + c.spec.offset_hz))
            .collect();
        let old_rate = self.rate;
        let old_center = self.center;

        // The node holding the count is about to be replaced, and a counter
        // that restarted on every retune would be worse than no counter.
        self.logged = self.logged();
        let graph = std::mem::replace(
            &mut self.graph,
            Graph::builder(StreamSpec::iq(plan.rate, plan.center)).build()?,
        );
        // Keyed by the tag every node went in under, which is the id of the
        // stage it was built for: a `NodeId` is a position in the graph that
        // is being taken apart, and the position is what a rebuild changes.
        let mut pool: HashMap<u64, NodePart> =
            graph.into_parts().into_iter().filter_map(|p| p.tag.map(|t| (t, p))).collect();

        // A channel whose mixer shift or filter design would differ is not
        // the same channel, and it does not have to be caught here any more:
        // everything a filter was designed against is in the id the stage is
        // derived under, so a channel that changed asks for stages that were
        // never in the pool. Whether a node that is still asked for can be
        // reused is the node's own question, put to it as the graph is
        // rebuilt: a bank keeps its several hundred chains across a retune,
        // and a spectrum cannot.
        let retuned = plan.center != old_center || plan.rate != old_rate;

        // The recorder is a file being written; it survives every rebuild
        // short of being switched off, and a newly started one is waiting
        // here for its place in the graph.
        let ring = self.pending_record.take().or_else(|| {
            // The ring is a stage like any other, so it comes back out of the
            // pool by the same name it went in under.
            pool.remove(&derived::RING).and_then(|p| RecordRing::from_part(p.node))
        });

        self.banks.clear();
        self.sources.clear();
        self.center = plan.center;
        self.rate = plan.rate;
        let mut tx = self.pending_tx.take();
        // The microphone the receiver holds stands in wherever a key-up did
        // not bring one, which is every rebuild but that one.
        if let Some(mic) = &self.mic {
            match tx.as_mut() {
                Some(t) if t.mic.is_none() => t.mic = Some(mic.clone()),
                Some(_) => {}
                None => tx = Some(TxSinks { stream: None, mic: Some(mic.clone()) }),
            }
        }
        self.assemble(plan, pool, ring, tx, retuned)?;
        // The stages are keyed by mode and rate, so a channel moved to
        // another frequency comes back holding the nodes it had. The dial
        // moving under every channel is not that: their offsets change and
        // their frequencies do not, and that is the case a rebuild is meant
        // to survive without a sound.
        let moved: Vec<u64> = self
            .chans
            .iter()
            .filter(|c| {
                let now = plan.center.as_f64() + c.spec.offset_hz;
                old_freq.get(&c.spec.id).is_some_and(|was| (was - now).abs() > 0.5)
            })
            .map(|c| c.spec.id)
            .collect();
        for id in moved {
            self.reset_channel(id);
        }
        Ok(())
    }

    /// Drop everything one channel's stages have learned: the station, the
    /// gain, the squelch's floor. Called when it is retuned, since all of
    /// those belong to the frequency it was on.
    fn reset_channel(&mut self, id: u64) {
        let stages: Vec<u64> = self
            .patch
            .stages()
            .iter()
            .filter(|s| s.settings.get("channel").and_then(|v| v.as_i64()) == Some(id as i64))
            .map(|s| s.id)
            .collect();
        for tag in stages {
            if let Some(n) = self.graph.by_tag(tag).and_then(|nid| self.graph.node_mut(nid)) {
                n.reset();
            }
        }
        if let Some(c) = self.chans.iter_mut().find(|c| c.spec.id == id) {
            c.kept = false;
            c.station = Station::default();
        }
    }

    fn fft_size(&self) -> usize {
        main_spectrum(&self.patch)
            .and_then(|id| self.stage::<SpectrumNode>(id))
            .map(|s| s.size())
            .unwrap_or(0)
    }

    /// Build the graph the plan describes, reusing what `pool` holds.
    /// `retuned` says the span moved under those nodes, which is one of the
    /// two things each of them is asked before it is reused.
    fn assemble(
        &mut self,
        plan: &Plan,
        mut pool: HashMap<u64, NodePart>,
        ring: Option<RecordRing>,
        sinks_tx: Option<TxSinks>,
        retuned: bool,
    ) -> Result<()> {
        let input = StreamSpec::iq(plan.rate, plan.center);
        let mut b = Graph::builder(input);

        // Everything at the head of the chain is a stage in a patch now: the
        // DC block, the zoom decimator, the spectrum and the recorder's ring.
        // The receiver draws that patch for itself from what it is doing,
        // unless the operator has taken it over, and then it is theirs.
        let mut refused = None;
        let mut ring = ring;
        // The receiver draws its own graph from the plan, the operator's
        // edits go on top, and the strip's stages are put in step with the
        // result: a wire the operator drew into the bus lands on an input
        // the bus has to be told about.
        let base = derived_patch(plan);
        let mut patch = base.clone();
        plan.edits.apply(&mut patch);
        sync_audio(&mut patch, plan);
        sync_video(&mut patch);
        let mut tx_sinks = sinks_tx;
        // `self.patch` is still the one the pooled nodes were built from,
        // which is what says whether a stage that kept its id still asks for
        // the same kind of node.
        let (patch_packets, patch_ids, reused) = match add_patch(
            &mut b,
            &mut pool,
            &self.patch,
            pipeline::graph::GRAPH_INPUT,
            &patch,
            &mut ring,
            &mut tx_sinks,
            retuned,
        ) {
            Ok(v) => v,
            Err(e) => {
                refused = Some(format!("the patch cannot be built: {e}"));
                (Vec::new(), HashMap::new(), Vec::new())
            }
        };

        // What the parts that are not drawn yet read. They followed the DC
        // block when it was built here; now they follow whatever the patch
        // says is the head, and the raw span if it says nothing.
        let head: Out = patch
            .tap(crate::patch::builtin::HEAD)
            .and_then(|s| match s {
                crate::patch::Source::Span => None,
                crate::patch::Source::Stage(f, port) => patch_ids.get(&f).map(|n| n.out(port)),
            })
            .unwrap_or(pipeline::graph::GRAPH_INPUT);
        let spectrum = main_spectrum(&patch).and_then(|id| patch_ids.get(&id)).copied();
        let audio = patch_ids.get(&derived::AUDIO).copied();

        // The front ends are stages in the patch now, so what runs is what
        // the graph says rather than a second reading of the scanner table.
        let mut narrowband: Vec<NodeId> = Vec::new();
        let of_kind = |kind: &str| -> Vec<NodeId> {
            patch
                .stages()
                .iter()
                .filter(|s| s.kind == kind)
                .filter_map(|s| patch_ids.get(&s.id))
                .copied()
                .collect()
        };
        // A front end the table asked for and the span cannot hold is left
        // out of the derived graph, and the interface has to be told why
        // rather than left wondering where its pager channel went.
        for at in &plan.fronts {
            let (Front::Protocol { hz, .. }, Some(proto)) = (&at.front, at.front.proto()) else {
                continue;
            };
            let shape = proto.shape();
            if shape.span_wide {
                // One that reads the span needs the span to be wide enough
                // for what it reads. Built anyway, the node refuses its own
                // input and takes the whole graph down with it, so the
                // receiver a person asked for a camera on came up with
                // nothing in it at all.
                if plan.eff_rate() < shape.min_rate_hz {
                    refused = Some(format!(
                        "{} needs {:.1} MS/s and the span is {:.1}",
                        proto.label(),
                        shape.min_rate_hz / 1e6,
                        plan.eff_rate() / 1e6
                    ));
                }
                continue;
            }
            if (hz - plan.center.as_f64()).abs() > plan.eff_rate() / 2.0 - shape.widths[0] {
                refused = Some(format!(
                    "{:.4} MHz is too near the span edge for {}",
                    hz / 1e6,
                    proto.label()
                ));
            }
        }
        let banks: Vec<NodeId> = of_kind("bank");
        let mut sources: Vec<NodeId> = of_kind("source_detect");
        sources.extend(of_kind("auto"));

        narrowband.extend(patch_packets);

        // The listening channels are stages in the patch too, so this is a
        // matter of finding them rather than building them. What each one is
        // doing still has to be gathered up: the interface asks a channel for
        // its gain, its squelch and its station, not the graph for a node.
        let mut chans: Vec<Chan> = Vec::new();
        for spec in &plan.channels {
            // A channel the span does not cover, or cannot hold at its
            // width, is left out and the strip shows it as out of reach. It
            // used to be a fault, and restoring a session tuned elsewhere
            // raised one per channel for something the dial fixes.
            if spec.offset_hz.abs() > plan.eff_rate() / 2.0 || plan.eff_rate() < spec.min_rate() {
                continue;
            }
            let of = |what: &str| -> Option<NodeId> {
                patch_ids.get(&chan_stage_id(what, spec, plan.eff_rate())).copied()
            };
            // A played channel ends in the blend; a decoded one ends in its
            // front end, which is heard only if it has speech to give.
            let last = match () {
                _ if spec.mode.is_decode() => "chan_front",
                _ => "chan_blend",
            };
            let Some(tail) = of(last) else { continue };
            // The bus input its tail is wired into, which is where its level
            // and its meter are.
            let tail_id = chan_stage_id(last, spec, plan.eff_rate());
            let port = patch
                .links()
                .iter()
                .find(|l| {
                    l.to.0 == derived::AUDIO
                        && matches!(l.from, crate::patch::Source::Stage(f, _) if f == tail_id)
                })
                .map(|l| l.to.1);
            let stereo = patch
                .stage(chan_stage_id("chan_demod", spec, plan.eff_rate()))
                .is_some_and(|s| s.kind == "wfm_demod");
            chans.push(Chan {
                spec: spec.clone(),
                // A channel came through intact when every stage of it did.
                kept: ["chan_mix", "chan_ifdec", last]
                    .iter()
                    .all(|w| reused.contains(&chan_stage_id(w, spec, plan.eff_rate()))),
                key: ChanKey::new(spec, plan.eff_rate()),
                // A decode channel is read at its voice port when it has
                // one, which is the output the strip listens to; its packets
                // leave on port 0 and go to the bus like any front end's.
                tail: match &spec.mode {
                    ChanMode::Decode(kind) => tail.out(voice_port(kind).unwrap_or(0)),
                    ChanMode::Auto => tail.out(voice_port("auto").unwrap_or(0)),
                    ChanMode::Audio(_) => tail.o(),
                },
                port,
                agc: of("chan_agc"),
                squelch: of("chan_squelch"),
                wfm: stereo.then(|| of("chan_demod")).flatten(),
                audio_rate: AUDIO_HZ,
                channels: if stereo { 2 } else { 1 },
                detail: String::new(),
                agc_gain_db: 0.0,
                squelch_open: false,
                squelch_db: 0.0,
                blend: 0.0,
                station: Station::default(),
                rds_stats: (0, 0, false),
            });
        }

        // The bus is the output: everything that is heard leaves through it.
        // Everything else that leaves the graph is read by the port it is
        // asked for by name.
        if let Some(a) = audio {
            b.output(a.o());
        }

        // Every spectrum stage except the one already behind the waterfall.
        // That stage is the main plot, and reporting it here as well drew the
        // same trace twice: a manual graph with a single spectrum in it came
        // up with a strip underneath showing exactly what was above it.
        self.patch_spectra = patch
            .stages()
            .iter()
            .filter(|s| s.kind == "spectrum")
            .filter_map(|s| patch_ids.get(&s.id).map(|id| (s.id, *id)))
            .filter(|(_, id)| Some(*id) != spectrum)
            .collect();
        let spectrum_src = spectrum.map(|s| s.o());
        let mut graph = b.build()?;
        // What the spectrum is actually seeing, which is the head unless a
        // patch stage was put in front of it. The axis is drawn from this, so
        // a decimator between the two has to narrow the span on screen as
        // well as in the arithmetic.
        // What the spectrum is seeing, which is whatever was wired into it.
        self.spectrum_rate = spectrum
            .and_then(|id| {
                graph
                    .node(id)
                    .map(|n| n.as_any())
                    .and_then(|a| a.downcast_ref::<SpectrumNode>())
                    .map(|s| s.rate())
            })
            .unwrap_or(0.0);
        let _ = spectrum_src;
        for c in chans.iter_mut() {
            let spec = graph.spec_of(c.tail).unwrap_or(input);
            c.audio_rate = spec.frame_rate();
            c.channels = spec.channels;
        }
        // A bus that was carried over from the last graph still holds its
        // open file; one that had to be built again needs it reopened. The
        // file is opened in append mode, so reopening costs nothing but a
        // syscall and never loses what is already in it.
        if let Some(id) = patch_ids.get(&derived::BUS) {
            let want = self.log_dir.is_some();
            if let Some(n) = graph
                .node_mut(*id)
                .map(|n| n.as_any_mut())
                .and_then(|a| a.downcast_mut::<nodes::PacketBusNode>())
            {
                if want != n.has_sink() {
                    n.set_sink(self.new_sink());
                }
            }
        }

        // Settings that live on a node rather than in its wiring, applied
        // once the graph they belong to exists. A reused node arrives holding
        // whatever it was last told, which is not necessarily what the plan
        // now says.
        if let Some(n) = patch_ids
            .get(&derived::DC)
            .and_then(|id| graph.node_mut(*id))
            .map(|n| n.as_any_mut())
            .and_then(|a| a.downcast_mut::<nodes::DcBlockNode>())
        {
            n.set_enabled(plan.dc_block);
        }
        self.graph = graph;
        self.head = head;
        self.patch = patch;
        self.base = base;
        // The nodes a rebuild replaced come back empty: no subscriptions, no
        // account, no survey file. What the plan says they are doing goes
        // back onto them here.
        self.apply_settings(plan);
        // A fresh tracker has to be told where the receiver is: it resolves a
        // position from a single frame with it, and without it a rebuild in
        // the middle of a drive silently stops recording where anything was
        // heard.
        if let Some(at) = self.station {
            self.set_station(at);
        }
        self.banks = banks
            .into_iter()
            .map(|id| {
                let channels = self
                    .graph
                    .node(id)
                    .map(|n| n.as_any())
                    .and_then(|a| a.downcast_ref::<BankNode>())
                    // What is decoding, not what the channelizer produces:
                    // the channels outside the wanted band have no decoder on
                    // them and reporting them overstates what is being heard.
                    .map(|b| b.active_channels())
                    .unwrap_or(0);
                Bank { channels }
            })
            .collect();
        self.sources = sources;
        self.chans = chans;
        self.refused = refused;
        Ok(())
    }

    /// Run one block through everything.
    pub fn process(&mut self, iq: &[C32]) -> Result<()> {
        let buf = self.graph.input_buf();
        buf.clear();
        buf.iq_mut().extend_from_slice(iq);
        // Kept and read back after the run rather than during it, because
        // saying which node spoke means asking the graph, and the run holds
        // it. Both kinds are rare; the packets and the speech are not, and
        // they are read from the ports.
        let mut said: Vec<(NodeId, pipeline::event::Event)> = Vec::new();
        for e in self.graph.run()? {
            match &e.event {
                pipeline::event::Event::Warning { .. } | pipeline::event::Event::Request(_) => {
                    said.push((e.node, e.event.clone()));
                }
                _ => {}
            }
        }
        for (node, event) in said {
            // The stage as the chain view labels it, which is what an
            // operator has on screen: a node no longer names itself in what
            // it emits, and a label of the graph's own cannot be misspelt.
            let stage = self.graph.label(node).unwrap_or("a stage").to_string();
            match event {
                pipeline::event::Event::Warning { message } => {
                    self.warnings.push(format!("{stage}: {message}"));
                }
                // Nothing here moves the dial or opens a channel on a
                // decoder's say-so yet; what was asked is kept where the
                // interface can read it, and said out loud so it is not
                // silently dropped.
                pipeline::event::Event::Request(request) => {
                    self.warnings.push(format!("{stage} asks: {}", describe(&request)));
                    self.requests.push((stage, request));
                }
                _ => {}
            }
        }
        self.read_back();
        Ok(())
    }

    /// The warnings nodes raised since the last call.
    pub fn take_warnings(&mut self) -> Vec<String> {
        std::mem::take(&mut self.warnings)
    }

    /// What decoders asked of the receiver since the last call.
    #[cfg_attr(not(test), allow(dead_code))]
    pub fn take_requests(&mut self) -> Vec<(String, pipeline::Request)> {
        std::mem::take(&mut self.requests)
    }

    /// Copy out the state a display wants on every frame.
    ///
    /// Read here rather than through events because these are things that
    /// *are* rather than things that happened: an AGC's gain has a current
    /// value whether or not it changed, and a panel wants it either way.
    fn read_back(&mut self) {
        for c in &mut self.chans {
            if let Some(a) = c.agc.and_then(|id| downcast::<AgcNode>(&self.graph, id)) {
                c.agc_gain_db = a.gain_db();
            }
            if let Some(sq) = c.squelch.and_then(|id| downcast::<SquelchNode>(&self.graph, id)) {
                c.squelch_open = sq.is_open();
                c.squelch_db = sq.measured_db();
            }
            if let Some(w) = c.wfm.and_then(|id| downcast::<WfmDemodNode>(&self.graph, id)) {
                c.station = w.station().clone();
                c.rds_stats = w.rds_stats();
                c.blend = w.blend();
            }
        }
    }

    pub fn channels(&self) -> &[Chan] {
        &self.chans
    }

    /// Whether a plan differs from what is running only in settings that can
    /// be handed to the nodes already there.
    ///
    /// Squelch, gain control and volume are numbers on existing nodes;
    /// frequency, mode and rate are a different chain. Telling them apart
    /// matters because a rebuild is not free and, dragged, it is not rare:
    /// a slider sends one of these per displayed frame.
    pub fn params_only(&self, plan: &Plan) -> bool {
        self.rate == plan.rate
            && self.center == plan.center
            && self.fft_size() == plan.fft
            && self.chans.len() == plan.channels.len()
            && plan.channels.iter().zip(&self.chans).all(|(want, have)| {
                want.id == have.spec.id && ChanKey::new(want, plan.eff_rate()) == have.key
            })
    }

    /// Apply those settings in place. Only valid where [`Self::params_only`]
    /// holds; anything else needs the graph rebuilt around it.
    ///
    /// The strip's stages are drawn again from the plan, exactly as a rebuild
    /// would draw them, and whatever that changed is handed to the nodes
    /// through the one call that keeps the description in step with them. One
    /// path rather than a second copy of what a channel's squelch, gain
    /// control and fader mean, which is how they came to be applied three
    /// different ways and to disagree.
    ///
    /// A stage the drawing would add or remove is left to the rebuild: the
    /// shape of a graph is fixed once it is built.
    pub fn apply_params(&mut self, plan: &Plan) {
        for (want, have) in plan.channels.iter().zip(self.chans.iter_mut()) {
            have.spec = want.clone();
        }
        let mut drawn = self.patch.clone();
        sync_audio(&mut drawn, plan);
        let mut changed: Vec<(u64, String, pipeline::ParamValue)> = Vec::new();
        for st in drawn.stages() {
            // The stages the strip owns: one chain per channel, and the bus
            // every chain ends at. Nothing else in the patch follows the
            // channel list.
            if !st.settings.contains_key("channel") && st.id != derived::AUDIO {
                continue;
            }
            let Some(was) = self.patch.stage(st.id) else { continue };
            for (name, v) in &st.settings {
                if was.settings.get(name) != Some(v) {
                    changed.push((st.id, name.clone(), v.clone()));
                }
            }
        }
        for (id, name, v) in changed {
            self.set_derived_param(id, &name, v);
        }
    }

    /// Audio from one channel, as its chain left it, before the bus.
    pub fn channel_audio(&self, i: usize) -> &[f32] {
        self.chans
            .get(i)
            .and_then(|c| self.graph.buf(c.tail))
            .and_then(|p| p.as_real())
            .unwrap_or(&[])
    }

    /// Whether the spectrum completed a frame this block.
    pub fn spectrum_ready(&self) -> bool {
        main_spectrum(&self.patch)
            .and_then(|id| self.stage::<SpectrumNode>(id))
            .map(|s| s.is_fresh())
            .unwrap_or(false)
    }

    pub fn power_db(&mut self) -> &[f32] {
        self.spectrum_mut().map(|s| s.power_db()).unwrap_or(&[])
    }

    pub fn adc(&mut self) -> nodes::AdcHealth {
        self.spectrum_mut().map(|s| s.adc()).unwrap_or_default()
    }

    /// The audio bus, for the subscriptions, the levels, the meters and what
    /// it is playing. `None` only when the patch could not be built at all.
    pub fn audio(&self) -> Option<&crate::audiobus::AudioBusNode> {
        self.stage::<crate::audiobus::AudioBusNode>(derived::AUDIO)
    }

    pub fn audio_mut(&mut self) -> Option<&mut crate::audiobus::AudioBusNode> {
        self.stage_mut::<crate::audiobus::AudioBusNode>(derived::AUDIO)
    }

    /// The video bus, for what is being watched and what else is being
    /// received. `None` when nothing in the graph produces pictures.
    pub fn video(&self) -> Option<&crate::videobus::VideoBusNode> {
        self.stage::<crate::videobus::VideoBusNode>(derived::VIDEO)
    }

    pub fn video_mut(&mut self) -> Option<&mut crate::videobus::VideoBusNode> {
        self.stage_mut::<crate::videobus::VideoBusNode>(derived::VIDEO)
    }

    /// The picture the bus is publishing, if any.
    pub fn watched_video(&self) -> Option<common::VideoFrame> {
        self.video().and_then(|n| n.bus().watched().cloned())
    }

    /// Every transmission the video bus has seen.
    pub fn video_inputs(&self) -> Vec<VideoInput> {
        let Some(bus) = self.video().map(|n| n.bus()) else {
            return Vec::new();
        };
        bus.channels()
            .iter()
            .filter(|c| c.live())
            .filter_map(|c| {
                c.last.as_ref().map(|f| VideoInput {
                    key: c.key.clone(),
                    label: c.label.clone(),
                    completeness: f.completeness(),
                })
            })
            .collect()
    }

    /// The bus's position in the running graph, for setting its parameters
    /// by the same route the chain view uses.
    pub fn audio_node_id(&self) -> Option<usize> {
        self.node_of_stage(derived::AUDIO).map(|id| id.0)
    }

    /// This block's mix as it leaves for the speaker: stereo, interleaved,
    /// and the frame rate it is at.
    pub fn audio_out(&self) -> (&[f32], f64) {
        let out = self.node_of_stage(derived::AUDIO).map(|id| id.o());
        let pcm = out
            .and_then(|o| self.graph.buf(o))
            .and_then(|p| p.as_real())
            .unwrap_or(&[]);
        let rate = out
            .and_then(|o| self.graph.spec_of(o))
            .map(|s| s.frame_rate())
            .unwrap_or(crate::audiobus::OUT_HZ);
        (pcm, rate)
    }

    /// Every input of the bus, as the strip draws it.
    pub fn strips(&self) -> Vec<StripState> {
        let Some(bus) = self.audio().map(|n| n.bus()) else {
            return Vec::new();
        };
        bus.strips()
            .iter()
            .enumerate()
            .filter(|(_, s)| s.is_fed())
            .map(|(k, s)| StripState {
                port: k,
                label: s.label.clone(),
                volume: s.volume,
                muted: s.muted,
                level: s.peak,
                voice: s.is_voice(),
                channel: self.chans.iter().find(|c| c.port == Some(k)).map(|c| c.spec.id),
            })
            .collect()
    }

    /// What every listening channel is doing, for its controls to show.
    pub fn channel_states(&self) -> Vec<crate::radio::ChannelState> {
        let bus = self.audio().map(|n| n.bus());
        self.chans
            .iter()
            .map(|c| crate::radio::ChannelState {
                id: c.spec.id,
                agc_gain_db: c.agc_gain_db,
                squelch_open: c.squelch_open,
                squelch_db: c.squelch_db,
                stereo_blend: c.blend,
                level: c
                    .port
                    .and_then(|k| bus.and_then(|b| b.strips().get(k)))
                    .map(|s| s.peak)
                    .unwrap_or(0.0),
            })
            .collect()
    }

    /// Every voice front end running, talking or not, read off the ports
    /// they publish on.
    pub(crate) fn voices(&self) -> Vec<common::Voice> {
        self.graph
            .order()
            .flat_map(|(id, _)| {
                let out = id.o();
                let voice = self.graph.spec_of(out).map(|s| s.kind) == Some(PortKind::Voice);
                let outs = self.graph.node(id).map(|n| n.num_outputs()).unwrap_or(1);
                let mut ports: Vec<Out> = voice.then_some(out).into_iter().collect();
                ports.extend(
                    (1..outs).map(|p| id.out(p)).filter(|o| {
                        self.graph.spec_of(*o).map(|s| s.kind) == Some(PortKind::Voice)
                    }),
                );
                ports
            })
            .filter_map(|o| self.graph.buf(o).and_then(|p| p.as_voice()))
            .flat_map(|v| v.iter().cloned())
            .collect()
    }

    /// Whether anything is tracking aircraft, from the local demodulator or
    /// from a feed.
    pub fn tracking(&self) -> bool {
        self.stage::<crate::tracks::TracksNode>(derived::TRACKS).is_some()
    }

    /// Channels in each bank, in the order the banks were added.
    pub fn bank_channels(&self) -> Vec<usize> {
        self.banks.iter().map(|b| b.channels).collect()
    }

    /// Whether any band is being watched for sources.
    pub fn has_sources(&self) -> bool {
        !self.sources.is_empty()
    }

    /// Every source open right now, across every band watched: RF centre,
    /// width and peak SNR.
    pub fn live_sources(&self) -> Vec<LiveSource> {
        let mut out = Vec::new();
        for &id in &self.sources {
            let Some(spec) = self.graph.spec_of(id.o()) else {
                continue;
            };
            let c = spec.center.as_f64();
            let live = if let Some(n) = downcast::<nodes::SourceDetectNode>(&self.graph, id) {
                n.live()
            } else if let Some(n) = downcast::<nodes::AutoNode>(&self.graph, id) {
                n.live()
            } else {
                continue;
            };
            for s in live {
                out.push(LiveSource {
                    center_hz: c + s.center_hz,
                    bandwidth_hz: s.bandwidth_hz(),
                    snr_db: Some(s.peak_snr_db),
                    locked_to: None,
                });
            }
            if let Some(n) = downcast::<nodes::AutoNode>(&self.graph, id) {
                for (name, center_hz, width_hz) in n.locked_channels() {
                    out.push(LiveSource {
                        center_hz,
                        bandwidth_hz: width_hz,
                        snr_db: None,
                        locked_to: Some(name),
                    });
                }
            }
        }
        out
    }

    /// The key status of every keyed front end in the graph, placed by hand,
    /// by a scanner, or by an auto node for a source it found: what the key
    /// manager shows, and how a recovered key reaches persistence.
    /// Deduplicated by cell, since two front ends on the same carrier report
    /// the same cell.
    pub fn tetra_key_status(&self) -> Vec<nodes::tetra_nodes::KeyStatus> {
        let mut out: Vec<nodes::tetra_nodes::KeyStatus> = Vec::new();
        for (id, _) in self.graph.order() {
            let Some(n) = self.graph.node(id) else { continue };
            pipeline::node::walk(n, &mut |n| {
                let Some(s) = nodes::keyed(n).and_then(|k| k.key_status()) else {
                    return;
                };
                if !out.iter().any(|e| (e.mcc, e.mnc, e.colour) == (s.mcc, s.mnc, s.colour)) {
                    out.push(s);
                }
            });
        }
        out
    }

    /// Install a key for a cell colour on every keyed front end, so traffic
    /// on that cell decodes. From the key manager, for a manual key.
    #[cfg(feature = "tea")]
    pub fn set_tetra_key(&mut self, colour: u8, key: decode::tea::Key) {
        self.each_keyed(&mut |k| k.add_key(colour, key));
    }

    /// Install a TA61 identity secret for a cell colour on every keyed front
    /// end, so its encrypted identities show as real subscribers.
    #[cfg(feature = "tea")]
    pub fn set_tetra_id_secret(&mut self, colour: u8, c: [u8; 8]) {
        self.each_keyed(&mut |k| k.add_id_secret(colour, c));
    }

    /// Every keyed front end in the graph, wherever it sits: on the span, on
    /// a channel of a bank, or inside an auto node.
    #[cfg(feature = "tea")]
    fn each_keyed(&mut self, f: &mut dyn FnMut(&mut dyn nodes::Keyed)) {
        let ids: Vec<_> = self.graph.order().map(|(id, _)| id).collect();
        for id in ids {
            let Some(n) = self.graph.node_mut(id) else { continue };
            pipeline::node::walk_mut(n, &mut |n| {
                if let Some(k) = nodes::keyed_mut(n) {
                    f(k);
                }
            });
        }
    }

    /// The raw span capture, for switching on and for reading how far it has
    /// got.
    pub fn capture(&self) -> Option<&nodes::IqCaptureNode> {
        self.stage::<nodes::IqCaptureNode>(derived::CAPTURE)
    }

    /// Start or stop writing the span to disk.
    ///
    /// A parameter rather than a rebuild: the point of a capture is the
    /// transmission happening right now, and rebuilding the graph to add a
    /// stage would drop every source the auto node has open.
    pub fn set_capture(&mut self, on: bool) {
        self.set_derived_param(derived::CAPTURE, "enabled", pipeline::ParamValue::Bool(on));
    }

    pub fn capturing(&self) -> bool {
        self.capture().is_some_and(|n| n.is_enabled() && !n.is_full())
    }

    /// Add the capture folder up again, for the status that reports it
    /// against the limit. Throttled inside the node.
    pub fn refresh_capture_folder(&mut self) {
        if let Some(n) = self.stage_mut::<nodes::IqCaptureNode>(derived::CAPTURE) {
            n.refresh_folder();
        }
    }

    /// How large the capture folder may get. Raising it lets a capture that
    /// stopped be started again, which is what pressing the button after
    /// reading why it stopped is asking for.
    pub fn set_capture_cap(&mut self, bytes: u64) {
        let Some(id) = self.node_of_stage(derived::CAPTURE) else { return };
        let mb = bytes as f64 / (1u64 << 20) as f64;
        let _ = self.set_node_param(id.0, "budget_mb", pipeline::ParamValue::Float(mb));
    }

    pub fn recorder_mut(&mut self) -> Option<&mut Recorder> {
        self.stage_mut::<nodes::RingNode<RecordRing>>(derived::RING)
            .and_then(|r| r.ring_mut().rec.as_mut())
    }

    /// Shape of everything running, for the chain view.
    pub fn topology(&self) -> Topology {
        self.graph.topology()
    }

    /// Microseconds spent in each top-level node since the graph was built,
    /// in execution order.
    ///
    /// Cumulative rather than per call, so a caller can difference it across
    /// one block and say which stage a slow block was spent in. A composite
    /// node reports the whole of its inner graph.
    #[cfg_attr(test, allow(dead_code))]
    pub fn node_costs(&self) -> Vec<(String, u64)> {
        self.graph.total_costs().into_iter().map(|(l, us)| (l.to_string(), us)).collect()
    }

    /// The latest readings of every scope in the graph, by node id, for the
    /// inspector to draw. Reading takes the fresh flag, so a caller polling
    /// faster than a scope refreshes sees the same frame again unchanged.
    pub fn scopes(&mut self) -> Vec<(usize, nodes::ScopeFrame)> {
        let ids: Vec<NodeId> = self.graph.order().map(|(id, _)| id).collect();
        let mut out = Vec::new();
        for id in ids {
            let Some(scope) = self
                .graph
                .node_mut(id)
                .map(|n| n.as_any_mut())
                .and_then(|a| a.downcast_mut::<nodes::ScopeNode>())
            else {
                continue;
            };
            let (frame, _) = scope.frame();
            if !frame.spectrum.is_empty() || frame.peak > 0.0 {
                out.push((id.0, frame.clone()));
            }
        }
        out
    }

    /// The waves the graph runs in, for debugging what runs beside what.
    pub fn run_levels(&self) -> Vec<Vec<&str>> {
        self.graph.run_levels()
    }

    /// Each node's smoothed cost per call, for finding where the time goes.
    pub fn run_costs(&self) -> Vec<(&str, f32)> {
        self.graph.run_costs()
    }

    /// Total microseconds each node has cost since the build.
    pub fn total_costs(&self) -> Vec<(&str, u64)> {
        self.graph.total_costs()
    }

    /// Set one node's own parameter, by the id the topology gave it.
    ///
    /// Returns whether the change alters the stream's shape, which the caller
    /// has to renegotiate around: a decimation factor is not a knob that can
    /// be turned while everything downstream keeps its rate.
    pub fn set_node_param(
        &mut self,
        id: usize,
        name: &str,
        value: pipeline::param::ParamValue,
    ) -> Result<bool> {
        let node = self
            .graph
            .node_mut(pipeline::graph::NodeId(id))
            .ok_or_else(|| common::Error::other(format!("no node {id} in this chain")))?;
        let affects_rate =
            node.params().into_iter().find(|p| p.name == name).is_some_and(|p| p.affects_rate);
        node.set_param(name, value.clone())?;
        // Into the description too, or the next rebuild puts the stage back
        // the way the patch had it and the setting was a slider that sprang
        // back.
        if let Some(st) =
            self.graph.tag_of(pipeline::graph::NodeId(id)).and_then(|tag| self.patch.stage_mut(tag))
        {
            st.settings.insert(name.to_string(), value);
        }
        Ok(affects_rate)
    }

    /// The node a stage of the patch became, for a setting applied without a
    /// rebuild.
    fn node_of_stage(&self, stage: u64) -> Option<NodeId> {
        self.graph.by_tag(stage)
    }

    /// Put a value the plan owns onto a derived stage: the node, the running
    /// description, and the graph the receiver drew alike.
    ///
    /// The drawn graph moves too because it is what the operator's edits are
    /// read against. Left behind, a switch the plan carries would read as
    /// something the operator changed, and be saved as an edit that outlived
    /// the switch that set it.
    fn set_derived_param(&mut self, stage: u64, name: &str, value: pipeline::ParamValue) {
        let Some(id) = self.node_of_stage(stage) else { return };
        if self.set_node_param(id.0, name, value.clone()).is_err() {
            return;
        }
        if let Some(st) = self.base.stage_mut(stage) {
            st.settings.insert(name.to_string(), value);
        }
    }

    /// Delay to a channel's audio, in milliseconds.
    /// The graph as a description: what is running, in the terms the view
    /// draws and the operator edits.
    pub fn patch(&self) -> &crate::patch::Patch {
        &self.patch
    }

    /// The graph as the receiver drew it before the operator's edits. An
    /// edited copy of [`Self::patch`] is read against this to find them.
    pub fn base(&self) -> &crate::patch::Patch {
        &self.base
    }

    /// The operator's edits, as the running graph now differs from the one
    /// the receiver drew: a parameter set on a derived stage by hand is in
    /// here, and has to be, or the next rebuild would put the stage back.
    pub fn edits(&self) -> crate::patch::Edits {
        crate::patch::Edits::diff(&self.patch, &self.base, operator_owns)
    }

    /// The levels as the nodes hold them, for the plan to follow.
    ///
    /// A fader or a squelch set through the chain view lands on the node,
    /// and the strip has to learn of it or the next thing the strip sends
    /// puts it back. Returns the bus levels and, per running channel, only
    /// what the strip owns: the rest of a channel is the plan's and a node
    /// has nothing to say about it.
    pub fn levels(&self) -> (AudioPlan, Vec<ChannelLevels>) {
        let bus = self.audio().map(|n| n.bus());
        let mut audio = AudioPlan::default();
        if let Some(b) = bus {
            let (master, muted) = b.master();
            let (calls, calls_muted) = b.calls();
            audio = AudioPlan { master, muted, calls, calls_muted, agc: b.agc_on() };
        }
        let chans = self
            .chans
            .iter()
            .map(|c| {
                let mut own = ChannelLevels {
                    id: c.spec.id,
                    label: c.spec.label.clone(),
                    volume: c.spec.volume,
                    muted: c.spec.muted,
                    squelch_db: c.spec.squelch_db,
                    agc: c.spec.agc,
                };
                if let Some(s) = c.port.and_then(|k| bus.and_then(|b| b.strips().get(k))) {
                    own.volume = s.volume;
                    own.muted = s.muted;
                    if !s.label.is_empty() {
                        own.label = s.label.clone();
                    }
                }
                if let Some(sq) = c.squelch.and_then(|id| downcast::<SquelchNode>(&self.graph, id))
                {
                    own.squelch_db = Some(sq.threshold_db());
                }
                if let Some(a) = c.agc.and_then(|id| downcast::<AgcNode>(&self.graph, id)) {
                    own.agc = a.is_enabled();
                }
                own
            })
            .collect();
        (audio, chans)
    }

    /// The rate the spectrum's frames cover, for the axis under them.
    pub fn spectrum_rate(&self) -> f64 {
        self.spectrum_rate
    }

    /// What every spectrum stage the operator added is seeing: its patch id,
    /// its powers in dBFS, and the band they cover.
    pub fn patch_spectra(&mut self) -> Vec<crate::radio::Spectrum> {
        let ids = self.patch_spectra.clone();
        let mut out = Vec::with_capacity(ids.len());
        for (tag, id) in ids {
            let Some(n) = self
                .graph
                .node_mut(id)
                .map(|n| n.as_any_mut())
                .and_then(|a| a.downcast_mut::<SpectrumNode>())
            else {
                continue;
            };
            out.push(crate::radio::Spectrum {
                tag,
                db: n.power_db().to_vec(),
                center: n.center().as_f64(),
                rate: n.rate(),
            });
        }
        out
    }

    pub fn latency_ms(&self, i: usize) -> f64 {
        let Some(c) = self.chans.get(i) else {
            return 0.0;
        };
        self.graph.latency_of(c.tail) as f64 / c.audio_rate.max(1.0) * 1e3
    }

    pub fn set_refresh(&mut self, hz: f32) {
        self.set_derived_param(
            derived::SPECTRUM,
            "refresh",
            pipeline::ParamValue::Float(hz as f64),
        );
    }

    pub fn set_smoothing(&mut self, v: f32) {
        self.set_derived_param(
            derived::SPECTRUM,
            "smoothing",
            pipeline::ParamValue::Float(v as f64),
        );
    }

    fn spectrum_mut(&mut self) -> Option<&mut SpectrumNode> {
        let id = main_spectrum(&self.patch)?;
        self.stage_mut::<SpectrumNode>(id)
    }

    pub fn set_dc_block(&mut self, on: bool) {
        if let Some(d) = self.dc_mut() {
            d.set_enabled(on);
        }
    }

    /// Forget the measured DC offset, after anything that moves it.
    pub fn remeasure_dc(&mut self) {
        if let Some(d) = self.dc_mut() {
            d.remeasure();
        }
    }

    fn dc_mut(&mut self) -> Option<&mut nodes::DcBlockNode> {
        self.stage_mut::<nodes::DcBlockNode>(derived::DC)
    }

    /// Start or stop recording. Takes effect on the next rebuild, since a
    /// recorder is a node and the graph's shape is fixed once built.
    pub fn set_recorder(&mut self, rec: Option<Recorder>) {
        self.pending_record = rec.map(RecordRing::new);
    }

    /// Everything that decoded this block, as packet list rows.
    ///
    /// One place, because there is one decoder: whatever the front end, a
    /// packet went onto the bus and came off it as a row.
    pub fn decodes(&self, at: std::time::Instant) -> Vec<DecodeRecord> {
        // Read off the bus rather than out of the node, and from the far side
        // of the dedupe: the packets there carry what they decoded to and one
        // row per burst, so the list sees exactly what the map and the device
        // database see. The protocols are the fallback for a graph whose
        // dedupe the operator took out.
        let node =
            self.node_of_stage(derived::DEDUPE).or_else(|| self.node_of_stage(derived::PROTOCOLS));
        let Some(out) = node.and_then(|id| self.graph.buf(id.o())) else {
            return Vec::new();
        };
        out.as_packets()
            .unwrap_or(&[])
            .iter()
            .flat_map(|p| p.decodes.iter().map(move |d| record(at, p, d)))
            .collect()
    }

    /// Forget every burst already reported.
    ///
    /// For a rebuild: every channel covers a different frequency afterwards,
    /// so nothing already reported can be the same burst as anything arriving.
    pub fn reset_dedupe(&mut self) {
        let Some(id) = self.node_of_stage(derived::DEDUPE) else { return };
        if let Some(n) = self.graph.node_mut(id) {
            n.reset();
        }
    }

    /// Point the log at a directory, or stop writing one.
    ///
    /// The bus stays either way: turning the log off should stop writing to
    /// disk, not disconnect every view from the traffic.
    /// What each feed is doing, for the settings modal: where it points, and
    /// whether anything is coming from it.
    pub fn feed_status(&self) -> Vec<FeedStatus> {
        // Found by what they are rather than by a role of their own: a feed
        // is a stage in the graph like everything else now.
        self.patch
            .stages()
            .iter()
            .filter(|s| s.kind == "feed")
            .filter_map(|s| self.stage::<nodes::FeedNode>(s.id))
            .map(|n| FeedStatus {
                spec: n.spec().clone(),
                connected: n.connected(),
                frames: n.frames(),
                error: n.error(),
            })
            .collect()
    }

    /// Size the log's folder may reach. Changing it takes effect on the file
    /// being written, so raising it restarts a log that stopped.
    pub fn set_log_cap(&mut self, cap: Option<u64>) {
        self.log_cap = cap;
        let sink = self.new_sink();
        if let Some(bus) = self.bus_mut() {
            bus.set_sink(sink);
        }
    }

    /// What the log has written, and whether it has given up.
    ///
    /// The reading is about the folder, so a receiver with nothing writing to
    /// it, because the log is off or because no front end on this span
    /// produces packets, still reports what is on the disk rather than zero.
    pub fn log_bytes(&self) -> u64 {
        self.stage::<nodes::PacketBusNode>(derived::BUS)
            .filter(|b| b.has_sink())
            .map(|b| b.sink_bytes())
            .unwrap_or(self.log_folder)
    }

    /// Add the log folder up again when nothing is writing to it. Throttled,
    /// and skipped entirely while a sink is counting its own writes.
    pub fn refresh_log_folder(&mut self) {
        const EVERY: std::time::Duration = std::time::Duration::from_secs(2);
        let writing =
            self.stage::<nodes::PacketBusNode>(derived::BUS).is_some_and(|b| b.has_sink());
        if writing || self.log_measured.is_some_and(|t| t.elapsed() < EVERY) {
            return;
        }
        self.log_measured = Some(std::time::Instant::now());
        let dir = self.log_dir.clone().or_else(crate::packetlog::PacketLog::default_dir);
        self.log_folder = dir.map(|d| crate::packetlog::folder_bytes(&d)).unwrap_or(0);
    }

    pub fn log_full(&self) -> bool {
        self.stage::<nodes::PacketBusNode>(derived::BUS).is_some_and(|b| b.sink_full())
    }

    pub fn set_packet_log(&mut self, dir: Option<PathBuf>) {
        self.log_dir = dir;
        let sink = self.new_sink();
        if let Some(bus) = self.bus_mut() {
            bus.set_sink(sink);
        }
    }

    fn new_sink(&self) -> Option<Box<dyn nodes::PacketSink>> {
        let cap = self.log_cap;
        self.log_dir.clone().map(|d| {
            Box::new(crate::packetlog::PacketLog::new(d).with_cap(cap))
                as Box<dyn nodes::PacketSink>
        })
    }

    fn bus_mut(&mut self) -> Option<&mut nodes::PacketBusNode> {
        self.stage_mut::<nodes::PacketBusNode>(derived::BUS)
    }

    /// Tracks heard recently, in the order they were first heard.
    pub fn tracks(&self, now: std::time::Instant) -> Vec<crate::tracks::Track> {
        self.stage::<crate::tracks::TracksNode>(derived::TRACKS)
            .map(|n| n.rows(now))
            .unwrap_or_default()
    }

    /// What the transcriber is, where its model is, and what it is doing.
    ///
    /// `None` when there is no transcriber in the graph at all, which is a
    /// different thing from one that has read nothing and has to read as one
    /// on screen.
    #[cfg(feature = "stt")]
    pub fn transcriber(&self) -> Option<crate::transcripts::Engine> {
        let id = self.node_of_stage(derived::TRANSCRIBE)?;
        let n = downcast::<crate::transcripts::LiveTranscribeNode>(&self.graph, id)?;
        let mut e = n.engine();
        e.node = id.0;
        Some(e)
    }

    /// Hand the nodes what the plan says that no stage setting could carry.
    ///
    /// A subscription is a rule, an account holds a secret, and a survey is
    /// an open file, so none of these survives the round trip through a
    /// stage's settings that every other plan value takes. This is the one
    /// place they are applied: on every rebuild, because the nodes come back
    /// empty, and whenever one of them changes, because a rebuild is not
    /// what a change to any of them is worth.
    pub fn apply_settings(&mut self, plan: &Plan) {
        let want = &plan.settings;
        // Reopened only when it is a different file. A rebuild is frequent
        // and opening the database again on each one costs a connection for
        // nothing.
        let open = self.survey_node().and_then(|n| n.db().map(|d| d.path().to_path_buf()));
        if open.as_deref() != want.survey_path.as_deref() {
            let db = match &want.survey_path {
                Some(p) => match survey::Db::open(p) {
                    Ok(db) => Some(db),
                    Err(e) => {
                        self.warnings.push(format!("survey: {e}"));
                        None
                    }
                },
                None => None,
            };
            if let Some(n) = self.survey_node_mut() {
                n.set_db(db);
            }
        }
        // An account short of what wigle.net needs to accept an upload is no
        // account: the feed would collect all day and be refused at the end
        // of it.
        let account = want.wigle.clone().filter(survey::Account::is_complete);
        if let Some(n) = self.wigle_node_mut() {
            n.set_account(account);
        }
        let beacondb = want.beacondb;
        if let Some(n) = self.beacondb_node_mut() {
            n.set_on(beacondb);
        }
        // A broker short of an address is no broker: the feed would announce
        // devices into a connection that cannot be made.
        let publish = want.homeassistant.clone().filter(|p| p.broker.is_complete());
        if let Some(n) = self.homeassistant_node_mut() {
            n.set_spaces(publish.as_ref().map(|p| p.spaces.as_str()).unwrap_or(""));
            n.set_broker(publish.map(|p| p.broker));
        }
        let calls = want.calls.clone();
        if let Some(n) = self.audio_mut() {
            n.bus_mut().set_subscriptions(calls);
        }
        let watching = want.watching.clone();
        if let Some(n) = self.video_mut() {
            n.bus_mut().set_rules(watching);
        }
        // Every transcriber in the graph, not only the one the receiver
        // draws: a stage the operator placed by hand writes what it read
        // into the same transcript, or its lines would go somewhere nobody
        // is reading.
        #[cfg(feature = "stt")]
        {
            let log = self.transcript.clone();
            let ids: Vec<_> = self.graph.order().map(|(id, _)| id).collect();
            for id in ids {
                if let Some(t) = self
                    .graph
                    .node_mut(id)
                    .map(|n| n.as_any_mut())
                    .and_then(|a| a.downcast_mut::<crate::transcripts::LiveTranscribeNode>())
                {
                    t.set_log(log.clone());
                }
            }
        }
    }

    /// What has been said on everything the receiver heard.
    pub fn transcript(&self) -> &crate::transcripts::SharedLog {
        &self.transcript
    }

    /// What the feed has sent, what is waiting, and why the last attempt
    /// failed. `None` when there is no node, which is a graph with no bus.
    pub fn wigle_status(&self) -> Option<nodes::WigleStatus> {
        Some(self.wigle_node()?.status())
    }

    pub fn beacondb_status(&self) -> Option<nodes::BeaconDbStatus> {
        Some(self.beacondb_node()?.status())
    }

    pub fn homeassistant_status(&self) -> Option<nodes::HomeAssistantStatus> {
        Some(self.homeassistant_node()?.status())
    }

    fn homeassistant_node(&self) -> Option<&nodes::HomeAssistantNode> {
        self.stage::<nodes::HomeAssistantNode>(derived::HOMEASSISTANT)
    }

    fn homeassistant_node_mut(&mut self) -> Option<&mut nodes::HomeAssistantNode> {
        self.stage_mut::<nodes::HomeAssistantNode>(derived::HOMEASSISTANT)
    }

    fn beacondb_node(&self) -> Option<&nodes::BeaconDbNode> {
        self.stage::<nodes::BeaconDbNode>(derived::BEACONDB)
    }

    fn beacondb_node_mut(&mut self) -> Option<&mut nodes::BeaconDbNode> {
        self.stage_mut::<nodes::BeaconDbNode>(derived::BEACONDB)
    }

    fn wigle_node(&self) -> Option<&nodes::WigleNode> {
        self.stage::<nodes::WigleNode>(derived::WIGLE)
    }

    fn wigle_node_mut(&mut self) -> Option<&mut nodes::WigleNode> {
        self.stage_mut::<nodes::WigleNode>(derived::WIGLE)
    }

    /// A fix from the GPS, which moves the station.
    ///
    /// `None` is a fix that went stale rather than a receiver that stopped
    /// being anywhere, so the station keeps the last position it was known to
    /// be at. Everything that needs to know where the receiver is reads the
    /// station, so there is one answer rather than one per consumer.
    pub fn set_fix(&mut self, fix: Option<gps::Fix>) {
        if let Some(f) = fix {
            self.set_station(f);
        }
    }

    /// Where the receiver is, with whatever the last fix said about how well
    /// that is known.
    pub fn fix(&self) -> Option<gps::Fix> {
        self.station
    }

    pub fn location(&self) -> Option<(f64, f64)> {
        self.station.map(|f| (f.lat, f.lon))
    }

    /// Devices and sightings the survey holds, and how many receptions were
    /// attributed to a device since the receiver started.
    pub fn survey_counts(&self) -> Option<(u64, u64, u64)> {
        let n = self.survey_node()?;
        let db = n.db()?;
        let (devices, sightings) = db.counts().ok()?;
        Some((devices, sightings, n.heard()))
    }

    /// The survey's rows, for a pane that draws them.
    pub fn survey_devices(&self, q: survey::Query) -> Vec<survey::Device> {
        self.survey_node()
            .and_then(|n| n.db())
            .and_then(|db| db.devices(q).ok())
            .unwrap_or_default()
    }

    /// Every sighting of one device, which is the trail it was heard along.
    pub fn survey_sightings(&self, device: i64) -> Vec<survey::Sighting> {
        self.survey_node()
            .and_then(|n| n.db())
            .and_then(|db| db.sightings(device).ok())
            .unwrap_or_default()
    }

    fn survey_node(&self) -> Option<&nodes::SurveyNode> {
        self.stage::<nodes::SurveyNode>(derived::SURVEY)
    }

    fn survey_node_mut(&mut self) -> Option<&mut nodes::SurveyNode> {
        self.stage_mut::<nodes::SurveyNode>(derived::SURVEY)
    }

    /// A position typed in or taken from the country, which is a station with
    /// nothing said about its quality.
    pub fn set_location(&mut self, lat: f64, lon: f64) {
        self.set_station(gps::Fix { lat, lon, ..Default::default() });
    }

    /// Move the station, and everything that resolves a position against it.
    pub fn set_station(&mut self, at: gps::Fix) {
        self.station = Some(at);
        if let Some(n) = self.stage_mut::<crate::tracks::TracksNode>(derived::TRACKS) {
            n.set_reference(at.lat, at.lon);
        }
        if let Some(n) = self.survey_node_mut() {
            n.set_station(Some(at));
        }
        if let Some(n) = self.wigle_node_mut() {
            n.set_station(Some(at));
        }
        if let Some(n) = self.beacondb_node_mut() {
            n.set_station(Some(at));
        }
    }

    /// Packets written to the log since the receiver started.
    pub fn logged(&self) -> u64 {
        self.logged
            + self.stage::<nodes::PacketBusNode>(derived::BUS).map(|n| n.written()).unwrap_or(0)
    }

    /// Take the recorder back out, after a replay that wrote one.
    #[cfg_attr(not(test), allow(dead_code))]
    pub fn take_recorder(&mut self) -> Option<Recorder> {
        self.stage_mut::<nodes::RingNode<RecordRing>>(derived::RING)
            .and_then(|r| r.ring_mut().rec.take())
    }

    /// What the head of the chain handed downstream this block: the samples
    /// after the DC notch and the zoom decimator, which is what every branch
    /// actually sees.
    #[cfg_attr(not(test), allow(dead_code))]
    pub fn zoomed_samples(&self) -> &[C32] {
        self.graph.buf(self.head).and_then(|p| p.as_iq()).unwrap_or(&[])
    }
}

/// What a bank tier is called, which is its channel width.
pub fn bank_label(width_hz: f64) -> String {
    if width_hz >= 1e6 {
        format!("{:.1} MHz bank", width_hz / 1e6)
    } else {
        format!("{:.0} kHz bank", width_hz / 1e3)
    }
}

/// The protocols a strip channel can be set to, with the channel each
/// expects: every registered one, span-wide or not. A decoder that reads a
/// span is put on a channel as wide as what it reads.
pub fn channel_fronts() -> Vec<(&'static str, f64)> {
    nodes::protocol::all().iter().map(|p| (p.id(), p.shape().widths[0])).collect()
}

/// The channel a protocol's decoder expects, or None if there is no such
/// protocol.
pub fn front_width(kind: &str) -> Option<f64> {
    nodes::protocol::by_id(kind).map(|p| p.shape().widths[0])
}

/// The protocol a word names: its registry id, or its label as a strip
/// button or a saved channel writes it.
pub fn front_kind(word: &str) -> Option<&'static str> {
    nodes::protocol::by_word(word).map(|p| p.id())
}

/// What a front end is called on a strip button.
pub fn front_label(kind: &str) -> String {
    nodes::protocol::by_id(kind).map_or_else(|| kind.to_uppercase(), |p| p.label().to_uppercase())
}

/// The band a front end needs, and the slowest rate it can be handed.
///
/// Every front end used to cut its own channel out of the full span with a
/// mixer and a filter of its own, inside `process`, where nothing could see
/// it: the chain view showed a pager node being fed 40 MS/s, which was true
/// and told you nothing about what it did with them. Declaring the band here
/// puts the extraction in the graph, lets two front ends in one band share it,
/// and stops Mode S running its envelope detector across a whole 40 MHz span.
fn front_band(front: &Front, at: &crate::scanners::FrontAt) -> Option<((f64, f64), f64)> {
    match front {
        Front::Protocol { hz, .. } => {
            let p = front.proto()?;
            let shape = p.shape();
            let w = shape.widths[0];
            // A span-wide decoder is handed the band it owns; one that reads
            // a channel is handed twice the channel, and mixes and filters
            // its own out of that.
            let band =
                if shape.span_wide { (hz - w / 2.0, hz + w / 2.0) } else { (hz - w, hz + w) };
            Some((band, shape.feed_rate_hz))
        }
        Front::Banks(widths) => {
            let band = at.band;
            // Two channels is the least a channelizer will build, so the band
            // has to arrive at least that wide.
            let widest = widths.iter().cloned().fold(0.0f64, f64::max);
            Some((band, widest * 2.0))
        }
        // Detection works at whatever rate the band arrives at.
        Front::Auto => Some((at.band, 0.0)),
    }
}

/// Whether a stage of this kind puts what it reads on the packet bus.
///
/// Every front end that produces bursts, frames or packets, whether it was
/// placed on a channel, on a band or by an operator, and a feed from another
/// receiver. Asked of the stage registry, which is where each stage says so
/// for itself: a decoder nobody wired to the bus decodes into silence, and a
/// list here would have to be remembered.
fn feeds_bus(kind: &str) -> bool {
    stages().desc(kind).is_some_and(|d| d.feeds_bus)
}

/// The port a stage of this kind puts speech on, if it has one.
///
/// The patch is a description, written before any node exists to be asked,
/// so this is answered from what each protocol declares of its chain. The
/// auto node puts whatever front end it placed on a source on a port of its
/// own, and a voice channel's decoder on its second.
fn voice_port(kind: &str) -> Option<usize> {
    match kind {
        "auto" => Some(1),
        _ => nodes::protocol::by_id(kind)?.outputs().iter().position(|k| *k == PortKind::Voice),
    }
}

/// The port a stage of this kind puts pictures on, if it has one. Read the
/// way [`voice_port`] is, and for the same reason.
fn video_port(kind: &str) -> Option<usize> {
    match kind {
        "auto" => Some(2),
        _ => nodes::protocol::by_id(kind)?.outputs().iter().position(|k| *k == PortKind::Video),
    }
}

/// Whether a stage of this kind can put something on the bus that the
/// tracker resolves a position from.
///
/// Asked of each protocol rather than kept as a list here, so a protocol
/// that starts reporting positions is tracked without this being touched.
fn reports_position(kind: &str) -> bool {
    match kind {
        // Whatever front end it placed on the source it found, which can be
        // any of them.
        "auto" => true,
        // Another receiver's packets, which is usually the reason to run a
        // tracker on a band that is neither 1090 nor 162.
        "feed" => true,
        _ => nodes::protocol::by_id(kind).is_some_and(|p| p.reports_position()),
    }
}

/// The recorder's ring, which is a stage in the graph but owns an open file
/// and so cannot be built from a description alone.
const RING: &str = "ring";
/// The stage that hands samples to the radio. Named here because, like the
/// recorder, it is handed a thing the patch cannot describe.
const TX_RADIO: &str = "radio_tx";

/// What the audio is limited to for a mode, in hertz.
///
/// Deviation is only half of Carson and the other half is the highest note
/// the modulator is given, so this is what keeps a transmission inside its
/// channel: speech to 15 kHz through a 2.5 kHz deviation is 35 kHz wide
/// where the band plan allows 12.5.
pub fn tx_audio_band(mode: crate::radio::TxMode) -> (f64, f64) {
    use crate::radio::TxMode;
    match mode {
        // Communications speech, out to where intelligibility lives. The low
        // cut is high on purpose: measured off air, a handheld puts 7 to
        // 12 dB less into the octave under 630 Hz than a flat microphone
        // does, and audio with that octave left in sounds muddy beside it
        // whatever the level. A 400 Hz cut through the 255 tap filter is
        // about 10 dB down at 300 and 3 dB at 500, which is the shape the
        // radio has.
        TxMode::Nfm | TxMode::Carrier => (400.0, 3_400.0),
        TxMode::Fm => (400.0, 4_000.0),
        TxMode::Am => (300.0, 4_000.0),
        // Broadcast, where 15 kHz is the standard and the pilot is above it.
        TxMode::Wfm => (30.0, 15_000.0),
    }
}

/// Ids the receiver gives the stages it derives for itself.
///
/// Fixed for the stages there is only ever one of, and computed from what it
/// is for otherwise: a bank keeps its channels and a detector its noise floor
/// across a rebuild only if the same stage comes back under the same name.
pub mod derived {
    use crate::patch::Patch;

    pub const DC: u64 = Patch::DERIVED_BASE + 1;
    pub const ZOOM: u64 = Patch::DERIVED_BASE + 2;
    pub const SPECTRUM: u64 = Patch::DERIVED_BASE + 3;
    pub const RING: u64 = Patch::DERIVED_BASE + 4;
    pub const BUS: u64 = Patch::DERIVED_BASE + 5;
    pub const PROTOCOLS: u64 = Patch::DERIVED_BASE + 6;
    pub const TRACKS: u64 = Patch::DERIVED_BASE + 7;
    pub const CAPTURE: u64 = Patch::DERIVED_BASE + 8;
    pub const SURVEY: u64 = Patch::DERIVED_BASE + 14;
    pub const TRANSCRIBE: u64 = Patch::DERIVED_BASE + 15;
    pub const AUDIO: u64 = Patch::DERIVED_BASE + 9;
    /// The transmit chain: its clock, what is modulated, the modulator, and
    /// the radio at the end of it.
    pub const TX_CLOCK: u64 = Patch::DERIVED_BASE + 10;
    pub const TX_SOURCE: u64 = Patch::DERIVED_BASE + 11;
    pub const TX_MOD: u64 = Patch::DERIVED_BASE + 12;
    pub const TX_RADIO: u64 = Patch::DERIVED_BASE + 13;
    /// What is going out, drawn on the span the receiver is deaf to while it
    /// goes out. In front of the head, so everything downstream sees it.
    pub const TX_MONITOR: u64 = Patch::DERIVED_BASE + 19;
    /// The video bus, where every picture the receiver has meets.
    pub const VIDEO: u64 = Patch::DERIVED_BASE + 16;
    /// The wardriving feed: what was heard, on its way to wigle.net.
    pub const WIGLE: u64 = Patch::DERIVED_BASE + 17;
    /// The same, on its way to beacondb.net.
    pub const BEACONDB: u64 = Patch::DERIVED_BASE + 18;
    /// One row per burst, after the protocols and before everything that
    /// reads them.
    pub const DEDUPE: u64 = Patch::DERIVED_BASE + 20;
    /// What is heard, on its way into the house over MQTT.
    pub const HOMEASSISTANT: u64 = Patch::DERIVED_BASE + 21;

    /// A stage that belongs to one band or one channel: the extraction in
    /// front of a front end, the front end itself, one bank of a set.
    pub fn at(what: &str, key: u64, nth: u64) -> u64 {
        use std::hash::{Hash, Hasher};
        let mut h = std::collections::hash_map::DefaultHasher::new();
        what.hash(&mut h);
        key.hash(&mut h);
        nth.hash(&mut h);
        // Clear of the fixed ids above and of the markers at the top of the
        // range, which are what `builtin` uses.
        Patch::DERIVED_BASE + 16 + h.finish() % ((1 << 39) - 16)
    }
}

/// The graph the receiver draws for itself, from what it is doing.
///
/// This is what runs, with the operator's edits put on top of it: an
/// operator editing the chain begins from the chain that is running, not
/// from an empty canvas, and what they change stays changed while the rest
/// of it follows the dial.
pub fn derived_patch(plan: &Plan) -> crate::patch::Patch {
    use crate::patch::{builtin, Source};
    use pipeline::registry::Settings;
    let mut p = crate::patch::Patch::default();

    // The head of the chain: what every branch downstream agrees the samples
    // are. A branch that saw the spur or the full rate would disagree with
    // the others about what arrived.
    let mut head = Source::Span;
    // What is being transmitted, drawn on the span. In front of everything
    // else at the head, so the spectrum, the waterfall and the raw capture
    // all see the over: a half duplex radio hears nothing while it keys up,
    // and a flat floor for the length of a transmission is the one moment an
    // operator most wants a display.
    if let Some(tx) = &plan.tx {
        let mut s = Settings::new();
        s.insert(
            "shift_hz".into(),
            pipeline::ParamValue::Float(tx.on_air.as_f64() - plan.center.as_f64()),
        );
        // Switched on by the radio thread, which is the only thing that knows
        // whether the receive stream has gone deaf, and switched off here on
        // every rebuild the way the raw capture is.
        s.insert("enabled".into(), pipeline::ParamValue::Bool(false));
        p.add_derived(derived::TX_MONITOR, "tx_monitor", s);
        p.connect(Source::Span, (derived::TX_MONITOR, 0));
        head = Source::Stage(derived::TX_MONITOR, 0);
    }
    if plan.dc_block {
        p.add_derived(derived::DC, "dc_block", Settings::new());
        p.connect(head, (derived::DC, 0));
        head = Source::Stage(derived::DC, 0);
    }
    if plan.zoom > 1 {
        let mut zoom = Settings::new();
        zoom.insert("factor".into(), pipeline::ParamValue::Int(plan.zoom as i64));
        // Passband just inside the new Nyquist: the whole point is that what
        // is left is clean, since anything folded in cannot be told from a
        // signal afterwards.
        zoom.insert("passband_hz".into(), pipeline::ParamValue::Float(plan.eff_rate() * 0.45));
        zoom.insert("input_rate_hz".into(), pipeline::ParamValue::Float(plan.rate));
        p.add_derived(derived::ZOOM, "decimate", zoom);
        p.connect(head, (derived::ZOOM, 0));
        head = Source::Stage(derived::ZOOM, 0);
    }
    // What the parts of the receiver that are not drawn yet read. They are
    // not boxes, so they follow a marker rather than a wire.
    p.connect(head, (builtin::HEAD, 0));

    let mut spectrum = Settings::new();
    spectrum.insert("size".into(), pipeline::ParamValue::Int(plan.fft as i64));
    // Both of these are the plan's, so the stage carries them: the spectrum
    // node is dropped from the pool on every retune, and a setting only the
    // node held came back at its default every time the dial moved.
    spectrum.insert("refresh".into(), pipeline::ParamValue::Float(plan.refresh_hz as f64));
    spectrum.insert("smoothing".into(), pipeline::ParamValue::Float(plan.smoothing as f64));
    p.add_derived(derived::SPECTRUM, "spectrum", spectrum);
    p.connect(head, (derived::SPECTRUM, 0));

    if plan.record {
        p.add_derived(derived::RING, RING, Settings::new());
        p.connect(head, (derived::RING, 0));
    }

    // The transmitter. Four stages: the clock it takes from the receiver,
    // what is being modulated, the modulator, and the radio. Derived rather
    // than drawn by hand because it follows the strip, and in the graph
    // rather than beside it because a transmission is something the receiver
    // is doing and the chain view is what the receiver is doing.
    //
    // Drawn whether or not a key is down, like the raw capture: a chain that
    // only exists while transmitting cannot be looked at before transmitting,
    // which is exactly when an operator wants to look at it, and building it
    // at key-up would rebuild the graph twice an over.
    if let Some(tx) = &plan.tx {
        use crate::radio::{TxMode, TxSource};
        p.add_derived(derived::TX_CLOCK, "tx_clock", Settings::new());
        // From the raw span, not from the head. The head is downstream of the
        // zoom decimator, so a chain taken from there runs at the zoomed rate
        // and hands the radio blocks at a rate it is not sampling: the device
        // refuses every one of them and transmits its own idle filler, which
        // on air is a carrier full of holes and nothing else.
        p.connect(Source::Span, (derived::TX_CLOCK, 0));

        let band = tx_audio_band(tx.mode);
        let (kind, mut settings) = match tx.spec.source {
            TxSource::Mic => {
                let mut s = Settings::new();
                s.insert("level".into(), pipeline::ParamValue::Float(tx.spec.mic_gain as f64));
                // What the receiving radio de-emphasises by: 750 us on a
                // voice channel, 50 us on broadcast FM in Europe, nothing
                // on AM.
                let emphasis = match tx.mode {
                    TxMode::Nfm | TxMode::Fm | TxMode::Carrier => 750.0,
                    TxMode::Wfm => 50.0,
                    TxMode::Am => 0.0,
                };
                s.insert("emphasis_us".into(), pipeline::ParamValue::Float(emphasis));
                ("mic", s)
            }
            TxSource::Tone => {
                let mut s = Settings::new();
                s.insert("hz".into(), pipeline::ParamValue::Float(tx.spec.tone_hz.max(1.0)));
                // A carrier is a tone at nothing: the modulator sees silence
                // and leaves the carrier where it is.
                let level = match tx.mode {
                    TxMode::Carrier => 0.0,
                    _ => 0.8,
                };
                s.insert("level".into(), pipeline::ParamValue::Float(level));
                ("tone", s)
            }
        };
        settings.insert("low_hz".into(), pipeline::ParamValue::Float(band.0));
        settings.insert("high_hz".into(), pipeline::ParamValue::Float(band.1));
        p.add_derived(derived::TX_SOURCE, kind, settings);
        p.connect(Source::Stage(derived::TX_CLOCK, 0), (derived::TX_SOURCE, 0));

        let (mod_kind, deviation) = match tx.mode {
            TxMode::Nfm | TxMode::Carrier => ("fm_mod", nodes::NBFM_DEVIATION_HZ),
            TxMode::Fm => ("fm_mod", nodes::FM_DEVIATION_HZ),
            TxMode::Wfm => ("fm_mod", nodes::WBFM_DEVIATION_HZ),
            TxMode::Am => ("am_mod", 0.0),
        };
        let mut m = Settings::new();
        if deviation > 0.0 {
            m.insert("deviation_hz".into(), pipeline::ParamValue::Float(deviation));
        }
        p.add_derived(derived::TX_MOD, mod_kind, m);
        p.connect(Source::Stage(derived::TX_SOURCE, 0), (derived::TX_MOD, 0));

        p.add_derived(derived::TX_RADIO, TX_RADIO, Settings::new());
        p.connect(Source::Stage(derived::TX_MOD, 0), (derived::TX_RADIO, 0));
        // What went to the antenna, back to the monitor at the head of the
        // receive chain. This is the only place holding those samples.
        p.connect(Source::Stage(derived::TX_RADIO, 0), (derived::TX_MONITOR, 1));
    }

    // The raw capture is always in the graph and usually switched off,
    // because the transmission worth having is the one already on the screen.
    // Adding the stage when somebody asks for it would rebuild the graph
    // first, and a rebuild loses the source the auto node has open, which is
    // exactly the signal they were trying to capture. Switched on it costs a
    // parameter; switched off it costs a memcpy of nothing.
    {
        let mut s = Settings::new();
        s.insert("dir".into(), pipeline::ParamValue::Text(plan.capture_dir.display().to_string()));
        // A capture running across a retune keeps running, and starts a new
        // file: the old one's name says which frequency and rate every
        // sample in it was taken at.
        s.insert("enabled".into(), pipeline::ParamValue::Bool(plan.capture));
        s.insert(
            "format".into(),
            pipeline::ParamValue::Text(plan.capture_format.extension().into()),
        );
        p.add_derived(derived::CAPTURE, "iq_capture", s);
        p.connect(head, (derived::CAPTURE, 0));
    }

    // The front ends the scanner table put on this span. Which demodulator
    // belongs on which frequency is configuration rather than structure, so
    // the table decides what is drawn here and the drawing is what runs.
    let mut extracts: HashMap<(u64, usize), Source> = HashMap::new();
    for at in &plan.fronts {
        let front = &at.front;
        // Everything the table puts on the span is fed a band cut out for it
        // rather than the whole span. What each front end then does inside
        // itself is a small residual shift at a low rate, instead of a mixer
        // and a several-thousand-tap filter running at the radio's own rate
        // where nothing could see them.
        let want = front_band(front, at).and_then(|(band, min_rate)| {
            let band = match front {
                Front::Banks(_) | Front::Auto => {
                    at.covered(plan.center.as_f64(), plan.eff_rate())?
                }
                _ => band,
            };
            (band.1 > band.0).then(|| SubBand::plan(band, plan.eff_rate(), min_rate))
        });
        let src = match want {
            Some(sub) => extract_stages(&mut p, &mut extracts, head, plan, sub),
            None => head,
        };
        // A single-channel front end whose channel does not clear the span
        // edge by its own bandwidth is left out rather than drawn: the node
        // would refuse it at negotiation and take the whole graph down with
        // it, and one badly placed block should cost its own front end rather
        // than the receiver.
        let fits = |hz: f64, width: f64| {
            (hz - plan.center.as_f64()).abs() <= plan.eff_rate() / 2.0 - width
        };
        match front {
            Front::Protocol { hz, .. } => {
                let Some(proto) = front.proto() else { continue };
                let shape = proto.shape();
                if !shape.span_wide && !fits(*hz, shape.widths[0]) {
                    continue;
                }
                if shape.span_wide && plan.eff_rate() < shape.min_rate_hz {
                    continue;
                }
                let at = nodes::Placed {
                    center_hz: *hz,
                    width_hz: shape.widths[0],
                    rate: plan.eff_rate(),
                    snr_db: f32::NAN,
                    origin: None,
                };
                // A span-wide decoder is one stage wherever it is placed;
                // one on a channel is keyed by the channel, so two blocks
                // pinning two pager channels are two decoders.
                let key = if shape.span_wide { 0 } else { *hz as u64 };
                let mut from = src;
                let chain = proto.chain(at);
                let last = chain.len() - 1;
                for (i, stage) in chain.into_iter().enumerate() {
                    let mut settings = stage.settings;
                    if i == last {
                        settings.insert(
                            "label".into(),
                            pipeline::ParamValue::Text(proto.stage_label(*hz)),
                        );
                    }
                    let id = p.add_derived(
                        derived::at(proto.id(), key, i as u64),
                        &stage.kind,
                        settings,
                    );
                    p.connect(from, (id, 0));
                    from = Source::Stage(id, 0);
                }
            }
            Front::Auto => {
                // One node over the band, whatever the band holds. The band
                // is passed on so it ignores the margin the power-of-two
                // extraction leaves either side, as a bank does.
                let Some(band) = at.covered(plan.center.as_f64(), plan.eff_rate()) else {
                    continue;
                };
                let sub = SubBand::plan(band, plan.eff_rate(), 0.0);
                let mut s = Settings::new();
                s.insert("band_lo_hz".into(), pipeline::ParamValue::Float(band.0));
                s.insert("band_hi_hz".into(), pipeline::ParamValue::Float(band.1));
                // The tuner's own centre, where the DC offset's movement
                // under a strong signal reads as a burst.
                s.insert("spur_hz".into(), pipeline::ParamValue::Float(plan.center.as_f64()));
                // The channel plan for the band, so a source found on a
                // channel is locked to it rather than measured afresh.
                if let Some(r) = crate::bands::raster_at((band.0 + band.1) / 2.0) {
                    s.insert("raster_hz".into(), pipeline::ParamValue::Float(r.step));
                    s.insert("raster_origin_hz".into(), pipeline::ParamValue::Float(r.origin));
                }
                let id = p.add_derived(derived::at("auto", sub.key(), 0), "auto", s);
                p.connect(src, (id, 0));
            }
            Front::Banks(widths) => {
                // The band the block was written about, not the whole span. A
                // bank handed 60 MS/s divides it into 1024 channels at best,
                // which is 60 kHz each: far wider than the 25 kHz an OOK
                // sensor occupies, so several devices share a channel and the
                // detector sees one long burst instead of packets. The
                // extraction above buys that resolution back, and costs less,
                // because the channelizer then runs at the band's rate.
                let Some(band) = at.covered(plan.center.as_f64(), plan.eff_rate()) else {
                    continue;
                };
                let sub = SubBand::plan(band, plan.eff_rate(), 0.0);
                // Two tiers that come out the same width are one tier. A
                // channelizer has a floor of two channels, so every tier
                // wider than half the band degenerates to that floor and
                // duplicates whichever tier got there first: on a 250 kHz
                // capture the 125 kHz tier and the 500 kHz one are both two
                // channels of 125 kHz, and the burst is then decoded twice,
                // identically, and logged as two receptions of one
                // transmission.
                let mut built: Vec<usize> = Vec::new();
                for &width in widths {
                    let channels = nodes::BankNode::channels_for(sub.rate(plan.eff_rate()), width);
                    if built.contains(&channels) {
                        continue;
                    }
                    built.push(channels);
                    let mut s = Settings::new();
                    s.insert("channel_hz".into(), pipeline::ParamValue::Float(width));
                    s.insert("band_lo_hz".into(), pipeline::ParamValue::Float(band.0));
                    s.insert("band_hi_hz".into(), pipeline::ParamValue::Float(band.1));
                    let id = p.add_derived(derived::at("bank", sub.key(), width as u64), "bank", s);
                    p.connect(src, (id, 0));
                }
            }
        }
    }

    // A feed from another receiver is a front end like any other: it produces
    // packets, so it belongs upstream of the bus rather than beside it.
    for spec in &plan.feeds {
        let mut s = Settings::new();
        s.insert("format".into(), pipeline::ParamValue::Text(spec.kind.name.into()));
        s.insert("host".into(), pipeline::ParamValue::Text(spec.host.clone()));
        s.insert("port".into(), pipeline::ParamValue::Int(spec.port as i64));
        s.insert(
            "label".into(),
            pipeline::ParamValue::Text(format!("{} {}", spec.kind.name, spec.address())),
        );
        let key = fnv(&spec.address());
        p.add_derived(derived::at("feed", key, 0), "feed", s);
    }

    // The strip's channels are drawn before the bus rather than after it,
    // because a channel that decodes is a front end like any other and has to
    // be on the bus with the rest. Drawn afterwards, its packets went
    // nowhere: nothing was wired to it and the log stayed empty.
    sync_audio(&mut p, plan);
    // And everything that produces a picture meets at the video bus, for the
    // same reason and in the same place.
    sync_video(&mut p);

    // Everything that produces packets meets at the bus, and everything that
    // consumes them hangs off the far side. One input per source: the bus is
    // the only stage whose shape follows the rest of the graph rather than
    // its own settings.
    let sources: Vec<u64> =
        p.stages().iter().filter(|s| feeds_bus(&s.kind)).map(|s| s.id).collect();
    if !sources.is_empty() {
        let mut s = Settings::new();
        s.insert("inputs".into(), pipeline::ParamValue::Int(sources.len() as i64));
        s.insert("label".into(), pipeline::ParamValue::Text("Packet log".into()));
        let bus = p.add_derived(derived::BUS, "packet_bus", s);
        for (k, from) in sources.iter().enumerate() {
            p.connect(Source::Stage(*from, 0), (bus, k));
        }

        // The protocols run here, once, over everything on the bus. They used
        // to run inside every channel of every bank, which meant a hundred
        // copies of the same tables and no decoding at all for a packet that
        // arrived by any other route.
        let decode = p.add_derived(derived::PROTOCOLS, "protocols", Settings::new());
        p.connect(Source::Stage(bus, 0), (decode, 0));

        // And the copies of a burst that neighbouring channels also read are
        // dropped once, here, rather than by each consumer for itself. The
        // packet list used to do it after the graph, so the map, the device
        // database and the feeds saw rows it had rejected.
        let rows = p.add_derived(derived::DEDUPE, "dedupe", Settings::new());
        p.connect(Source::Stage(decode, 0), (rows, 0));

        // The tracker is a consumer of the bus like any other, which is what
        // stops every view being wired to the demodulator it happens to care
        // about. Attached whenever anything could produce a frame it can
        // resolve a position from: a feed is usually the reason to run one at
        // all on a band that is neither 1090 nor 162.
        let makes_tracks = p.stages().iter().any(|s| reports_position(&s.kind));
        if makes_tracks {
            let t = p.add_derived(derived::TRACKS, "tracks", Settings::new());
            // Downstream of the protocols and not beside them: the packets
            // that arrive here carry what they decoded to, so the map reads
            // one decode rather than parsing the frame a second time.
            p.connect(Source::Stage(rows, 0), (t, 0));
        }

        // The device database is another consumer of the bus, and it is in
        // the graph whether or not a survey is being recorded: opening the
        // file is a setting on a node that is already there, so turning it on
        // mid-drive does not rebuild the receiver under the packets.
        let survey = p.add_derived(derived::SURVEY, "survey", Settings::new());
        p.connect(Source::Stage(rows, 0), (survey, 0));

        // The feed to wigle.net is a second consumer of the same decodes,
        // and it is drawn whether or not an account has been set: turning
        // wardriving on is a setting on a node that is already there, the
        // way the survey's file is.
        let wigle = p.add_derived(derived::WIGLE, "wigle", Settings::new());
        p.connect(Source::Stage(rows, 0), (wigle, 0));

        // beaconDB is a third consumer of the same decodes, drawn whether or
        // not it is on for the same reason.
        let beacondb = p.add_derived(derived::BEACONDB, "beacondb", Settings::new());
        p.connect(Source::Stage(rows, 0), (beacondb, 0));

        // And the house is a fourth. Drawn with no broker set for the same
        // reason again: pointing it at one is a setting on a stage that is
        // already there, not a rebuild under the packets.
        let ha = p.add_derived(derived::HOMEASSISTANT, "homeassistant", Settings::new());
        p.connect(Source::Stage(rows, 0), (ha, 0));
    }

    p
}

/// The stages of one strip channel, in the order they are built. A decode
/// channel uses the first two and then its front end; an audio one uses the
/// rest.
const CHAN_STAGES: [&str; 10] = [
    "chan_mix",
    "chan_ifdec",
    "chan_front",
    "chan_demod",
    "chan_scope",
    "chan_squelch",
    "chan_audiodec",
    "chan_deemph",
    "chan_agc",
    "chan_blend",
];

/// The video bus, and what feeds it.
///
/// Drawn whenever something in the patch can produce pictures, which the auto
/// node always can: it publishes whatever front end it placed on a source, so
/// a camera it finds reaches the bus without anything here knowing which
/// front end read it. Exactly the arrangement the audio bus has with voice
/// ports, and for the same reason: a picture that arrives somewhere other
/// than the bus is a picture no view can find.
fn sync_video(p: &mut crate::patch::Patch) {
    use crate::patch::Source;
    use pipeline::registry::Settings;
    use pipeline::ParamValue as V;

    let feeds: Vec<(u64, usize, String)> = p
        .stages()
        .iter()
        .filter_map(|st| {
            let port = video_port(&st.kind)?;
            Some((st.id, port, stage_label(&st.kind, &st.settings)))
        })
        .collect();
    if feeds.is_empty() {
        p.remove(derived::VIDEO);
        return;
    }
    let bus = derived::VIDEO;
    let mut s: Settings = p.stage(bus).map(|s| s.settings.clone()).unwrap_or_default();
    s.insert("label".into(), V::Text("Video".into()));
    // The stage has to exist before anything can be wired into it, the way
    // the audio bus is added before its inputs are drawn.
    p.add_derived(bus, "video_bus", s.clone());
    for (k, (id, port, label)) in feeds.iter().enumerate() {
        p.connect(Source::Stage(*id, *port), (bus, k));
        s.entry(format!("label{k}")).or_insert(V::Text(label.clone()));
    }
    // One spare, the way the audio bus keeps one, so a chain drawn by hand
    // has an input to land on.
    s.insert("inputs".into(), V::Int(feeds.len() as i64 + 1));
    p.add_derived(bus, "video_bus", s);
}

/// Whether a setting the operator changed on a derived stage is an edit of
/// theirs, or one the plan owns and writes again on the next rebuild.
///
/// A listening channel's stages and the audio bus's per-strip levels are the
/// strip's: what the operator sets on them by hand goes back into the strip
/// rather than sitting in the edits as an override the strip would fight.
/// The level of a bus input the strip did not set, which is a chain the
/// operator drew, is the exception, since the strip has no other place to
/// keep it.
pub fn operator_owns(st: &crate::patch::Stage, name: &str, base: &crate::patch::Stage) -> bool {
    if st.settings.contains_key("channel") {
        return false;
    }
    if st.kind == "audio_bus" {
        let level =
            matches!(StripParam::parse(name), Some((StripParam::Vol | StripParam::Mute, _)));
        return level && !base.settings.contains_key(name);
    }
    true
}

/// The stages the strip owns, drawn into a patch: one chain per listening
/// channel, and the bus every chain and every voice front end ends at.
///
/// Run over the derived patch and over the operator's alike, on every
/// rebuild. The channels are not the patch's to remove and the bus is where
/// the speaker is, so manual mode keeps them in step with the strip the same
/// way automatic mode draws them: what changes with the mode is who owns the
/// front ends, not whether the receiver can be listened to. Before this,
/// manual mode froze the channels as they were when it was switched on, and
/// a channel added or retuned afterwards was silent.
fn sync_audio(p: &mut crate::patch::Patch, plan: &Plan) {
    use crate::patch::{builtin, Source};
    use pipeline::registry::Settings;
    use pipeline::ParamValue as V;
    let rate = plan.eff_rate();
    let head = p.tap(builtin::HEAD).unwrap_or(Source::Span);

    // The chains, drawn again from the channel list every time, which is
    // what keeps a mixer's shift following the dial. A channel the span no
    // longer covers cannot be demodulated: the mixer would shift a frequency
    // the radio never sampled down to baseband, and the chain would produce
    // noise that sounds like a dead station rather than silence.
    let mut want: Vec<u64> = Vec::new();
    let mut tails: Vec<(Source, &ChannelSpec)> = Vec::new();
    let mut fronts: Vec<u64> = Vec::new();
    for spec in &plan.channels {
        if spec.offset_hz.abs() > rate / 2.0 || rate < spec.min_rate() {
            continue;
        }
        let tail = channel_stages(p, head, spec, plan.center.as_f64(), rate);
        want.extend(CHAN_STAGES.iter().map(|w| chan_stage_id(w, spec, rate)));
        // Where the strip listens to it. A played channel ends in audio; a
        // decoded one is heard only if its front end has speech to give, and
        // a pager does not.
        let port = match &spec.mode {
            // A played channel ends in audio, whether or not it is speech:
            // the bus is the first stop for every demodulator's audio, and a
            // channel marked as voice is played through its fader like any
            // other and named as a conversation on the tap. There is no
            // packet in analogue speech, so there is nothing to put anywhere
            // else.
            ChanMode::Audio(_) => Some(0),
            ChanMode::Decode(kind) => voice_port(kind),
            ChanMode::Auto => voice_port("auto"),
        };
        if let Some(port) = port {
            tails.push((Source::Stage(tail, port), spec));
        }
        if spec.mode.is_decode() {
            fronts.push(tail);
        }
    }
    // A stage left over from a channel that changed mode or went away.
    let stale: Vec<u64> = p
        .stages()
        .iter()
        .filter(|s| s.settings.contains_key("channel") && !want.contains(&s.id))
        .map(|s| s.id)
        .collect();
    for id in stale {
        p.remove(id);
    }

    // A decode channel is a front end, so its packets belong on the packet
    // bus with everything else's. The derived pass draws the channels before
    // the bus and wires them there; this is for the pass over an edited
    // patch, where the bus was drawn before the channel existed.
    if p.stage(derived::BUS).is_some() {
        for id in fronts {
            let from = Source::Stage(id, 0);
            if p.links().iter().any(|l| l.to.0 == derived::BUS && l.from == from) {
                continue;
            }
            let k = (0..).find(|k| p.feeding((derived::BUS, *k)).is_none()).unwrap_or(0);
            p.connect(from, (derived::BUS, k));
        }
        let inputs = p.links().iter().filter(|l| l.to.0 == derived::BUS).map(|l| l.to.1 + 1).max();
        if let (Some(n), Some(st)) = (inputs, p.stage(derived::BUS)) {
            let mut s = st.settings.clone();
            s.insert("inputs".into(), V::Int(n as i64));
            p.add_derived(derived::BUS, "packet_bus", s);
        }
    }

    // The bus, carrying the levels that are nobody's channel. Whatever it
    // was set to per input is kept, so a fader on a chain the operator drew
    // survives the channels around it changing.
    let bus = derived::AUDIO;
    let mut s = p.stage(bus).map(|s| s.settings.clone()).unwrap_or_default();
    s.insert("label".into(), V::Text("Audio".into()));
    s.insert("master".into(), V::Float(plan.audio.master as f64));
    s.insert("muted".into(), V::Bool(plan.audio.muted));
    s.insert("calls".into(), V::Float(plan.audio.calls as f64));
    s.insert("calls_muted".into(), V::Bool(plan.audio.calls_muted));
    s.insert("agc".into(), V::Bool(plan.audio.agc));
    p.add_derived(bus, "audio_bus", s.clone());

    // What feeds it: every chain's tail and every voice port, on the input
    // it already has or else the first free one.
    let mut owned: Vec<(Source, Option<&ChannelSpec>, String)> =
        tails.iter().map(|(tail, spec)| (*tail, Some(*spec), spec.label.clone())).collect();
    for st in p.stages() {
        if let Some(port) = voice_port(&st.kind) {
            let from = Source::Stage(st.id, port);
            // A front end the strip owns is already here, with the fader and
            // the name the operator gave it. Adding it again as a loose voice
            // port would put the same speech into the mix twice.
            if owned.iter().any(|(o, ..)| *o == from) {
                continue;
            }
            owned.push((from, None, stage_label(&st.kind, &st.settings)));
        }
    }
    for (from, ..) in &owned {
        let wired = p.links().iter().any(|l| l.to.0 == bus && l.from == *from);
        if !wired {
            let k = (0..).find(|k| p.feeding((bus, *k)).is_none()).unwrap_or(0);
            p.connect(*from, (bus, k));
        }
    }

    // Inputs in order with no gaps, each carrying its own settings with it,
    // and one spare on the end for the next chain to be wired into. A gap
    // is an input nothing feeds, which is what the spare is, and two of
    // them is a mixer with a hole in it.
    let mut wired: Vec<(usize, Source)> =
        p.links().iter().filter(|l| l.to.0 == bus).map(|l| (l.to.1, l.from)).collect();
    wired.sort_by_key(|(k, _)| *k);
    let per_port: Vec<Vec<(StripParam, V)>> = wired
        .iter()
        .map(|(k, _)| {
            StripParam::ALL
                .into_iter()
                .filter_map(|what| s.get(&what.name(*k)).map(|v| (what, v.clone())))
                .collect()
        })
        .collect();
    s.retain(|name, _| StripParam::parse(name).is_none());
    for (k, _) in &wired {
        p.disconnect((bus, *k));
    }
    for (k, ((_, from), own)) in wired.iter().zip(per_port).enumerate() {
        p.connect(*from, (bus, k));
        for (what, v) in own {
            s.insert(what.name(k), v);
        }
        match owned.iter().find(|(o, ..)| o == from) {
            // A channel's level is the strip's to say.
            Some((_, Some(spec), label)) => {
                strip_settings(&mut s, k, spec.volume, spec.muted, label);
                s.insert(
                    StripParam::Speech.name(k),
                    V::Bool(spec.voice && !spec.mode.is_decode()),
                );
            }
            // A voice port's level is the subscriptions' business; the strip
            // itself passes it whole.
            Some((_, None, label)) => {
                s.insert(StripParam::Label.name(k), V::Text(label.clone()));
                s.entry(StripParam::Vol.name(k)).or_insert(V::Float(1.0));
            }
            // A chain the operator drew, named after what feeds it.
            None => {
                if let Source::Stage(f, _) = from {
                    if let Some(st) = p.stage(*f) {
                        s.entry(StripParam::Label.name(k))
                            .or_insert(V::Text(stage_label(&st.kind, &st.settings)));
                    }
                }
            }
        }
    }
    s.insert("inputs".into(), V::Int(wired.len() as i64 + 1));
    p.add_derived(bus, "audio_bus", s);

    // The transcriber hangs off the bus's tap, which carries every strip
    // before the faders and the subscriptions: what the receiver heard, not
    // what the operator chose to listen to. On the audio and not on the
    // packets because speech is not a packet, and because a partial reading
    // of a transmission still in progress has nowhere to live on one.
    #[cfg(feature = "stt")]
    {
        let mut t = Settings::new();
        t.insert("root".into(), V::Text(models_root().display().to_string()));
        // Off in the graph the receiver draws: writing down what people said
        // is not something to start doing because nobody said otherwise.
        // Turning it on is an edit, which is how it is remembered.
        t.insert("enabled".into(), V::Bool(false));
        let id = p.add_derived(derived::TRANSCRIBE, "transcribe_live", t);
        p.connect(Source::Stage(bus, 1), (id, 0));
    }
}

/// One strip's settings on the bus, as the patch carries them.
fn strip_settings(
    s: &mut pipeline::registry::Settings,
    k: usize,
    volume: f32,
    muted: bool,
    label: &str,
) {
    use pipeline::ParamValue as V;
    s.insert(StripParam::Vol.name(k), V::Float(volume as f64));
    s.insert(StripParam::Mute.name(k), V::Bool(muted));
    s.insert(StripParam::Label.name(k), V::Text(label.to_string()));
}

/// One listening channel, as stages.
///
/// The same arithmetic the hand-built chain used, saying what to build rather
/// than building it. Every stage of it is a box in the view now, so a channel
/// is something an operator can look inside, retune a filter in, or take
/// apart, rather than eight nodes that only existed as a side effect of
/// asking for a frequency.
fn channel_stages(
    p: &mut crate::patch::Patch,
    head: crate::patch::Source,
    spec: &ChannelSpec,
    center: f64,
    rate: f64,
) -> u64 {
    match &spec.mode {
        ChanMode::Audio(mode) => audio_channel_stages(p, head, spec, *mode, rate),
        ChanMode::Decode(kind) => decode_channel_stages(p, head, spec, kind, center, rate),
        ChanMode::Auto => auto_channel_stages(p, head, spec, center, rate),
    }
}

/// One channel watched by the auto front end: the band cut out around the
/// frequency the channel is tuned to, at the width the channel is set to,
/// and the auto node reading whatever is inside it.
///
/// The scanner table's own auto blocks are bands somebody wrote down in
/// advance. This is the same node over a band pointed at on the spectrum,
/// which is what an operator wants when the interesting thing is 40 kHz wide
/// and nowhere near an allocation anybody named.
fn auto_channel_stages(
    p: &mut crate::patch::Patch,
    head: crate::patch::Source,
    spec: &ChannelSpec,
    center: f64,
    rate: f64,
) -> u64 {
    use crate::patch::Source;
    use pipeline::registry::Settings;
    use pipeline::ParamValue as V;

    let hz = spec.offset_hz;
    let width = spec.bandwidth();
    let at = |p: &mut crate::patch::Patch, what: &str, kind: &str, mut s: Settings| -> u64 {
        s.insert("channel".into(), V::Int(spec.id as i64));
        p.add_derived(chan_stage_id(what, spec, rate), kind, s)
    };

    let mut mix = Settings::new();
    mix.insert("shift_hz".into(), V::Float(chan_shift(spec)));
    let m = at(p, "chan_mix", "mixer", mix);
    p.connect(head, (m, 0));

    // Decimated to the band and a little either side, because the detector's
    // resolution is what it can measure a source's width with: handed the
    // whole span it would spend its bins on spectrum this channel is not
    // about.
    let target = width * crate::radio::IF_HEADROOM;
    let dec = ((rate / target).floor() as usize).max(1);
    let band_rate = rate / dec as f64;
    let mut ifd = Settings::new();
    ifd.insert("factor".into(), V::Int(dec as i64));
    ifd.insert("passband_hz".into(), V::Float((width / 2.0).min(band_rate * 0.45)));
    ifd.insert("input_rate_hz".into(), V::Float(rate));
    ifd.insert("label".into(), V::Text(format!("/{dec} to {}", hz_label(band_rate))));
    let i = at(p, "chan_ifdec", "decimate", ifd);
    p.connect(Source::Stage(m, 0), (i, 0));

    // The band in absolute frequencies, as the scanner table's auto blocks
    // are given it, so a source is reported where it is on the dial and not
    // where it is in this stream.
    let (lo, hi) = (center + hz - width / 2.0, center + hz + width / 2.0);
    let mut s = Settings::new();
    s.insert("band_lo_hz".into(), V::Float(lo));
    s.insert("band_hi_hz".into(), V::Float(hi));
    // The tuner's own centre, where the DC offset moving under a strong
    // signal reads as a burst. Only when this channel actually covers it.
    if lo < center && center < hi {
        s.insert("spur_hz".into(), V::Float(center));
    }
    if let Some(r) = crate::bands::raster_at((lo + hi) / 2.0) {
        s.insert("raster_hz".into(), V::Float(r.step));
        s.insert("raster_origin_hz".into(), V::Float(r.origin));
    }
    s.insert("label".into(), V::Text(spec.label.clone()));
    let f = at(p, "chan_front", "auto", s);
    p.connect(Source::Stage(i, 0), (f, 0));
    f
}

/// One channel that is decoded rather than played: the band cut out around
/// the frequency it is tuned to, and the front end reading it.
///
/// The same three boxes the scanner table draws for a pinned front end, drawn
/// for a channel somebody put on the strip instead. That is the whole point
/// of it: one channel at a fixed centre and width can be read with the
/// scanner switched off, where before the only way to decode a frequency was
/// a block that swept the span it was in.
fn decode_channel_stages(
    p: &mut crate::patch::Patch,
    head: crate::patch::Source,
    spec: &ChannelSpec,
    kind: &str,
    center: f64,
    rate: f64,
) -> u64 {
    use crate::patch::Source;
    use pipeline::registry::Settings;
    use pipeline::ParamValue as V;

    let hz = spec.offset_hz;
    let width = spec.bandwidth();
    let at = |p: &mut crate::patch::Patch, what: &str, kind: &str, mut s: Settings| -> u64 {
        s.insert("channel".into(), V::Int(spec.id as i64));
        p.add_derived(chan_stage_id(what, spec, rate), kind, s)
    };

    let mut mix = Settings::new();
    mix.insert("shift_hz".into(), V::Float(chan_shift(spec)));
    let m = at(p, "chan_mix", "mixer", mix);
    p.connect(head, (m, 0));

    // The front end mixes and filters its own channel out of what it is
    // handed, so this only has to bring the rate down far enough that it is
    // not doing that at the radio's. Decimating to the channel itself would
    // leave the node no transition band and no room for the tuning error the
    // dial has, so the target is well above it: what the protocol asks to
    // be fed, or a multiple of the channel where it asks for nothing.
    let proto = nodes::protocol::by_id(kind);
    let feed = proto.map_or(0.0, |p| p.shape().feed_rate_hz);
    let target =
        if feed > 0.0 { feed } else { (width * DECODE_RATE_RATIO).max(DECODE_MIN_RATE_HZ) };
    let dec = ((rate / target).floor() as usize).max(1);
    let mut ifd = Settings::new();
    ifd.insert("factor".into(), V::Int(dec as i64));
    ifd.insert("passband_hz".into(), V::Float((rate / dec as f64) * 0.45));
    ifd.insert("input_rate_hz".into(), V::Float(rate));
    ifd.insert("label".into(), V::Text(format!("/{dec} to {}", hz_label(rate / dec as f64))));
    let i = at(p, "chan_ifdec", "decimate", ifd);
    p.connect(Source::Stage(m, 0), (i, 0));

    // The mixer moved the channel to the middle of the stream and said so, so
    // the front end is told the frequency it is really on: it reads its own
    // channel out of the stream's centre, and every packet it puts on the bus
    // is labelled with where it came from. The protocol says what stages
    // that takes; the last of them is the channel's front.
    let placed = nodes::Placed {
        center_hz: center + hz,
        width_hz: width,
        rate: rate / dec as f64,
        snr_db: f32::NAN,
        origin: None,
    };
    let chain = match proto {
        Some(p) => p.chain(placed),
        None => vec![nodes::NodeSpec::new(kind).f("channel_hz", center + hz)],
    };
    let last = chain.len() - 1;
    let mut from = Source::Stage(i, 0);
    let mut f = 0;
    for (n, stage) in chain.into_iter().enumerate() {
        let mut s = stage.settings;
        let what = if n == last { "chan_front".to_string() } else { format!("chan_front_{n}") };
        if n == last {
            s.insert("label".into(), V::Text(spec.label.clone()));
        }
        f = at(p, &what, &stage.kind, s);
        p.connect(from, (f, 0));
        from = Source::Stage(f, 0);
    }
    f
}

/// How much wider than its channel a decode channel's front end is fed, and
/// the floor under that. A 12.5 kHz channel lands on the 192 kHz the scanner
/// table's own pinned front ends have always been given.
const DECODE_RATE_RATIO: f64 = 12.0;
const DECODE_MIN_RATE_HZ: f64 = 192_000.0;

/// One channel that is played: eight stages from the span to the bus.
fn audio_channel_stages(
    p: &mut crate::patch::Patch,
    head: crate::patch::Source,
    spec: &ChannelSpec,
    mode: Demod,
    rate: f64,
) -> u64 {
    use crate::patch::Source;
    use pipeline::registry::Settings;
    use pipeline::ParamValue as V;

    // The channel's own width decides the IF rate when it is wider than the
    // mode's: a 25 kHz repeater set by hand on an NFM channel has to survive
    // the decimation before any filter can be built around it.
    let width = spec.bandwidth();
    let if_dec =
        ((rate / mode.if_rate().max(width * crate::radio::IF_HEADROOM)).round() as usize).max(1);
    let if_rate = rate / if_dec as f64;
    let au_dec = ((if_rate / AUDIO_HZ).round() as usize).max(1);
    // Every stage says which channel it belongs to, so the ones a channel
    // leaves behind can be found without inverting a hash.
    let at = |p: &mut crate::patch::Patch, what: &str, kind: &str, mut s: Settings| -> u64 {
        s.insert("channel".into(), V::Int(spec.id as i64));
        p.add_derived(chan_stage_id(what, spec, rate), kind, s)
    };

    // CW is tuned low by the pitch so the dial reads the carrier rather than
    // the note; every other mode is tuned to what it listens to.
    let mut mix = Settings::new();
    mix.insert("shift_hz".into(), V::Float(chan_shift(spec)));
    let m = at(p, "chan_mix", "mixer", mix);
    p.connect(head, (m, 0));

    // Sized from the signal's bandwidth, not from the decimation factor: the
    // stopband has to land where the first alias folds down.
    let mut ifd = Settings::new();
    ifd.insert("factor".into(), V::Int(if_dec as i64));
    ifd.insert("passband_hz".into(), V::Float(width / 2.0));
    ifd.insert("input_rate_hz".into(), V::Float(rate));
    ifd.insert("label".into(), V::Text("IF decimator".into()));
    let i = at(p, "chan_ifdec", "decimate", ifd);
    p.connect(Source::Stage(m, 0), (i, 0));

    let stereo = mode == Demod::Wfm && if_rate >= 130_000.0;
    let mut d = Settings::new();
    let demod_kind = if stereo {
        d.insert("label".into(), V::Text("WFM demod".into()));
        "wfm_demod"
    } else if mode == Demod::Am {
        d.insert("label".into(), V::Text("AM envelope".into()));
        "envelope"
    } else if mode.is_ssb() {
        d.insert("sideband".into(), V::Text(mode.sideband().to_string()));
        if mode == Demod::Cw {
            d.insert("pitch_hz".into(), V::Float(mode.cw_pitch()));
            // On CW the width control is the filter itself, which is the
            // whole reason to reach for it: 500 Hz on a quiet band, 150 in a
            // pile-up.
            d.insert("width_hz".into(), V::Float(spec.bandwidth_hz.unwrap_or(CW_FILTER_HZ)));
            d.insert("label".into(), V::Text("CW filter".into()));
        } else {
            // Half the channel is one sideband, which is what the demodulator
            // passes: the control narrows the audio with the channel rather
            // than leaving a filter open wider than the IF in front of it.
            if let Some(bw) = spec.bandwidth_hz {
                d.insert("high_hz".into(), V::Float((bw / 2.0).max(400.0)));
            }
            d.insert("label".into(), V::Text("Sideband filter".into()));
        }
        "ssb_demod"
    } else {
        d.insert("deviation_hz".into(), V::Float(mode.deviation()));
        "fm_demod"
    };
    let dem = at(p, "chan_demod", demod_kind, d);
    p.connect(Source::Stage(i, 0), (dem, 0));

    // The squelch goes here, on the demodulator's raw output, and not later
    // where the audio is. An FM noise squelch works by measuring the hiss
    // above the speech band, and the audio filter's whole job is to remove
    // that: measured on an empty 2 m channel, a squelch after the filter saw
    // a clean signal and held itself open on pure noise.
    let mut tail = Source::Stage(dem, 0);

    // A scope on the demodulator's raw output, in every channel the strip
    // builds. It is where the questions about a channel are answered: what
    // the discriminator is putting out, how much of it is hiss, and what the
    // squelch behind it is deciding against. It passes the stream through
    // and costs a small transform thirty times a second.
    let mut sc = Settings::new();
    sc.insert("label".into(), V::Text("Demod scope".into()));
    let scope = at(p, "chan_scope", "scope", sc);
    p.connect(tail, (scope, 0));
    tail = Source::Stage(scope, 0);

    if let Some(db) = spec.squelch_db.or_else(|| mode.default_squelch_db()) {
        let mut s = Settings::new();
        let measure = if mode == Demod::Nfm {
            nodes::SquelchKind::Noise
        } else {
            nodes::SquelchKind::Level
        };
        s.insert("kind".into(), V::Text(measure.to_string()));
        s.insert("threshold_db".into(), V::Float(db as f64));
        let sq = at(p, "chan_squelch", "squelch", s);
        p.connect(tail, (sq, 0));
        tail = Source::Stage(sq, 0);
    }

    let mut ad = Settings::new();
    ad.insert("factor".into(), V::Int(au_dec as i64));
    // Never wider than the channel itself: a filter passing 4 kHz of audio
    // out of a 5 kHz channel is passing the skirt as well as the signal.
    ad.insert("passband_hz".into(), V::Float(mode.audio_bw().min(width / 2.0)));
    ad.insert("input_rate_hz".into(), V::Float(if_rate));
    ad.insert("label".into(), V::Text("Audio decimator".into()));
    let aud = at(p, "chan_audiodec", "real_decimate", ad);
    p.connect(tail, (aud, 0));
    tail = Source::Stage(aud, 0);

    if !(mode == Demod::Am || mode.is_ssb()) {
        // De-emphasis is an FM thing: it undoes the pre-emphasis the
        // transmitter applied. Applying it to AM or SSB would just be a
        // treble cut nobody asked for.
        let mut de = Settings::new();
        de.insert("tau_us".into(), V::Float(50.0));
        let d = at(p, "chan_deemph", "deemphasis", de);
        p.connect(tail, (d, 0));
        tail = Source::Stage(d, 0);
    }

    // The gain control comes after the squelch, so what it sees is either a
    // signal or silence. The other order lets the AGC lift the noise on a
    // dead channel up to the threshold and hold the squelch open.
    let preset = match mode {
        Demod::Cw => Some(nodes::AgcPreset::Cw),
        Demod::Nfm | Demod::Am | Demod::Usb | Demod::Lsb => Some(nodes::AgcPreset::Voice),
        Demod::Wfm => None,
    };
    if let Some(preset) = preset {
        let mut a = Settings::new();
        a.insert("preset".into(), V::Text(preset.to_string()));
        a.insert("enabled".into(), V::Bool(spec.agc));
        let agc = at(p, "chan_agc", "agc", a);
        p.connect(tail, (agc, 0));
        tail = Source::Stage(agc, 0);
    }

    let hb = at(p, "chan_blend", "high_blend", Settings::new());
    p.connect(tail, (hb, 0));
    hb
}

/// The id one stage of one channel is derived under.
///
/// Everything a filter in this chain was designed against goes into it: a
/// channel whose mode or rate changed is not the same channel, and reusing a
/// filter designed for the old one would be reusing the wrong coefficients
/// rather than saving work. The offset is not in it: that is the mixer's
/// shift, a setting the stage is brought up to date with, and keying on it
/// meant every channel was built afresh whenever the dial moved under it.
/// What a channel's mixer is set to, in one place.
///
/// Written once because it is applied twice: when the chain is built and
/// again whenever the channel moves without one. CW is tuned low by the
/// pitch so the dial reads the carrier rather than the note.
fn chan_shift(spec: &ChannelSpec) -> f64 {
    let pitch = match &spec.mode {
        ChanMode::Audio(m) => m.cw_pitch(),
        _ => 0.0,
    };
    -(spec.offset_hz - pitch)
}

fn chan_stage_id(what: &str, spec: &ChannelSpec, rate: f64) -> u64 {
    use std::hash::{Hash, Hasher};
    let mut h = std::collections::hash_map::DefaultHasher::new();
    spec.mode.key().hash(&mut h);
    // The width is in the key for the same reason the mode is: every filter
    // in the chain is designed around it, and a channel that changed width
    // has to be built again rather than keep coefficients for the old one.
    spec.bandwidth().to_bits().hash(&mut h);
    rate.to_bits().hash(&mut h);
    derived::at(what, spec.id, h.finish() ^ fnv(what))
}

/// A small stable number from a name, to keep one channel's stages apart.
fn fnv(s: &str) -> u64 {
    s.bytes().fold(0xcbf2_9ce4_8422_2325u64, |h, b| (h ^ b as u64).wrapping_mul(0x100_0000_01b3))
}

/// The mixer and decimator that cut one band out of the span, as stages.
///
/// Shared by band and factor, so two front ends listening in the same place
/// read one extraction instead of each running its own mixer over every
/// sample.
fn extract_stages(
    p: &mut crate::patch::Patch,
    cache: &mut HashMap<(u64, usize), crate::patch::Source>,
    head: crate::patch::Source,
    plan: &Plan,
    sub: SubBand,
) -> crate::patch::Source {
    use crate::patch::Source;
    use pipeline::registry::Settings;
    if sub.is_whole_span(plan.center.as_f64()) {
        return head;
    }
    if let Some(src) = cache.get(&(sub.key(), sub.factor)) {
        return *src;
    }
    let mut mix = Settings::new();
    mix.insert("shift_hz".into(), pipeline::ParamValue::Float(plan.center.as_f64() - sub.center));
    mix.insert(
        "label".into(),
        pipeline::ParamValue::Text(format!("{:.4} MHz mixer", sub.center / 1e6)),
    );
    let m = p.add_derived(derived::at("submix", sub.key(), 0), "mixer", mix);
    p.connect(head, (m, 0));

    let mut dec = Settings::new();
    dec.insert("factor".into(), pipeline::ParamValue::Int(sub.factor as i64));
    dec.insert("passband_hz".into(), pipeline::ParamValue::Float(sub.need / 2.0));
    dec.insert("input_rate_hz".into(), pipeline::ParamValue::Float(plan.eff_rate()));
    dec.insert(
        "label".into(),
        pipeline::ParamValue::Text(format!(
            "/{} to {}",
            sub.factor,
            hz_label(sub.rate(plan.eff_rate()))
        )),
    );
    let d = p.add_derived(derived::at("subdec", sub.key(), sub.factor as u64), "decimate", dec);
    p.connect(Source::Stage(m, 0), (d, 0));

    let out = Source::Stage(d, 0);
    cache.insert((sub.key(), sub.factor), out);
    out
}

/// What a stage is called on screen and in the latency accounting.
///
/// A registry name is a key, not a label: "dc_block" and "/8 to 300.0 kHz"
/// are the same kind of thing to the builder and not to a reader.
fn stage_label(kind: &str, settings: &pipeline::registry::Settings) -> String {
    use pipeline::registry::SettingsExt;
    // A derived stage can carry the name the old hand-written code gave it,
    // which says which band or which frequency it belongs to.
    if let Some(l) = settings.get("label").and_then(|v| v.as_str()) {
        return l.to_string();
    }
    match kind {
        "dc_block" => "DC block".into(),
        "spectrum" => "Spectrum".into(),
        RING => "Recorder".into(),
        "decimate" | "real_decimate" => {
            let n = settings.i64_or("factor", 1).max(1);
            if settings.get("passband_hz").is_some() {
                format!("Zoom /{n}")
            } else {
                format!("Decimate /{n}")
            }
        }
        "mixer" => "Mixer".into(),
        TX_RADIO => "Transmitter".into(),
        "tx_clock" => "Transmit clock".into(),
        "tx_monitor" => "Transmit monitor".into(),
        "mic" => "Microphone".into(),
        "tone" => "Test tone".into(),
        "fm_mod" => "FM modulator".into(),
        "am_mod" => "AM modulator".into(),
        "ook_mod" => "OOK keyer".into(),
        "fsk_mod" => "FSK keyer".into(),
        "fir_filter" | "iir_filter" => {
            let what = settings.str_or("response", "lowpass");
            let hz = settings.f64_or("freq_hz", 0.0);
            let how = if kind == "fir_filter" { "FIR" } else { "IIR" };
            format!("{how} {what} {}", hz_label(hz))
        }
        "envelope" => "Envelope".into(),
        "fm_demod" => "FM discriminator".into(),
        "deemphasis" => "De-emphasis".into(),
        "agc" => "AGC".into(),
        "squelch" => "Squelch".into(),
        "scope" => "Scope".into(),
        "pulse_detect" => "OOK pulses".into(),
        "ask_detect" => "ASK pulses".into(),
        "fsk_detect" => "FSK pulses".into(),
        "protocol_decode" => "Protocols".into(),
        "high_blend" => "High blend".into(),
        "protocols" => "Protocols".into(),
        "dedupe" => "Dedupe".into(),
        "tracks" => "Tracks".into(),
        "transcribe_live" => "Transcribe".into(),
        "survey" => "Devices".into(),
        "wigle" => "WiGLE".into(),
        "homeassistant" => "Home Assistant".into(),
        "beacondb" => "beaconDB".into(),
        "packet_bus" => "Packet log".into(),
        "audio_bus" => "Audio".into(),
        "video_bus" => "Video".into(),
        "wfm_demod" => "WFM demod".into(),
        "ssb_demod" => "SSB demodulator".into(),
        "mode_s" => "1090 Mode S".into(),
        "ais" => "162 AIS".into(),
        "aprs" => "APRS".into(),
        "pocsag" => "Pager".into(),
        "m17" => "M17".into(),
        "dmr" => "DMR".into(),
        "bank" => bank_label(settings.f64_or("channel_hz", 0.0)),
        "source_detect" => "Sources".into(),
        "source_decode" => "Source decoders".into(),
        "auto" => "Auto".into(),
        other => other.to_string(),
    }
}

/// A request in a sentence, for the log.
fn describe(r: &pipeline::Request) -> String {
    use pipeline::Request;
    match r {
        Request::Reshape { lo_hz, hi_hz } => {
            format!("a band of {:.4} to {:.4} MHz", lo_hz / 1e6, hi_hz / 1e6)
        }
        Request::OpenChannel { protocol, center_hz, role, .. } => {
            format!("a {role} channel at {:.4} MHz for {protocol}", center_hz / 1e6)
        }
        Request::Claim { lo_hz, hi_hz } => {
            format!("{:.4} to {:.4} MHz for itself", lo_hz / 1e6, hi_hz / 1e6)
        }
        Request::Release => "to be dropped".into(),
        Request::Retune { center_hz } => format!("a retune to {:.4} MHz", center_hz / 1e6),
    }
}

/// What building a patch produced: the stages that put packets on the bus,
/// where every stage ended up, and which of them kept the node they had.
type Built = (Vec<NodeId>, HashMap<u64, NodeId>, Vec<u64>);

/// The same registry, built once, for the questions a patch answers about a
/// stage before any node of it exists.
fn stages() -> &'static pipeline::registry::Registry {
    static REG: std::sync::OnceLock<pipeline::registry::Registry> = std::sync::OnceLock::new();
    REG.get_or_init(registry)
}

/// Every stage type this receiver can build.
///
/// The node registry plus the ones that only make sense inside the
/// application: the tracker folds positions together for the map, which is a
/// view rather than a signal path, so it lives here.
pub fn registry() -> pipeline::registry::Registry {
    use pipeline::registry::{Category, StageDesc};
    let mut r = nodes::registry();
    r.register(
        StageDesc {
            name: "tracks",
            summary: "Fold reported positions into tracks: aircraft, vessels and marks",
            category: Category::Sink,
            feeds_bus: false,
        },
        |_s| Ok(Box::new(crate::tracks::TracksNode::new()) as Box<dyn pipeline::node::Node>),
    );
    #[cfg(feature = "stt")]
    r.register(
        StageDesc {
            name: "transcribe_live",
            summary: "Read what is being said on everything the receiver hears, \
                      as it is said, with a local Whisper model",
            category: Category::Sink,
            feeds_bus: false,
        },
        |s: &pipeline::registry::Settings| {
            use pipeline::SettingsExt;
            let root = std::path::PathBuf::from(s.str_or("root", ""));
            let fallback = stt::default_model_in(&root);
            let mut n = crate::transcripts::LiveTranscribeNode::new()
                .under(root)
                .model(s.str_or("model", &fallback))
                .on(stt::DeviceChoice::parse(s.str_or("device", "auto")));
            if let Some(dir) = s.get("dir").and_then(|v| v.as_str()).filter(|d| !d.is_empty()) {
                n = n.in_dir(dir);
            }
            pipeline::node::Node::set_param(
                &mut n,
                "enabled",
                pipeline::ParamValue::Bool(s.bool_or("enabled", true)),
            )?;
            pipeline::node::Node::set_param(
                &mut n,
                "min_speech_s",
                pipeline::ParamValue::Float(s.f64_or("min_speech_s", 0.6)),
            )?;
            Ok(Box::new(n) as Box<dyn pipeline::node::Node>)
        },
    );
    r.register(
        StageDesc {
            name: "video_bus",
            summary: "Every picture the receiver has in one place: pictures do \
                      not sum, so this one selects what is watched and keeps \
                      the last field of everything else",
            category: Category::Video,
            feeds_bus: false,
        },
        |s: &pipeline::registry::Settings| {
            let mut n = crate::videobus::VideoBusNode::new();
            for (name, value) in s {
                let _ = pipeline::node::Node::set_param(&mut n, name, value.clone());
            }
            Ok(Box::new(n) as Box<dyn pipeline::node::Node>)
        },
    );
    r.register(
        StageDesc {
            name: "audio_bus",
            summary: "Every channel and every voice front end in one place: \
                      what reaches the speaker is what is wired in here, at \
                      the level its strip says",
            category: Category::Audio,
            feeds_bus: false,
        },
        |s: &pipeline::registry::Settings| {
            let mut n = crate::audiobus::AudioBusNode::new(crate::audiobus::OUT_HZ);
            // Every level is a parameter, and the label is not one.
            for (name, value) in s {
                let _ = pipeline::node::Node::set_param(&mut n, name, value.clone());
            }
            Ok(Box::new(n) as Box<dyn pipeline::node::Node>)
        },
    );
    r
}

/// Build every stage in a patch, wired as the patch says.
///
/// Returns the ones that produce packets, for the bus to collect, and where
/// each stage ended up, so the rest of the receiver can find the ones it has
/// to talk to.
///
/// A stage whose inputs are not all fed is left out rather than built. The
/// graph refuses an unconnected input port, and refusing the whole receiver
/// because a stage has just been dropped on the canvas and not yet wired up
/// would make the obvious way to work impossible: nobody draws a chain
/// backwards from its last wire.
fn add_patch(
    b: &mut GraphBuilder,
    pool: &mut HashMap<u64, NodePart>,
    was: &crate::patch::Patch,
    span: Out,
    patch: &crate::patch::Patch,
    ring: &mut Option<RecordRing>,
    tx: &mut Option<TxSinks>,
    retuned: bool,
) -> Result<Built> {
    use crate::patch::Source;
    use pipeline::registry::SettingsExt;
    let reg = registry();
    let mut made: Vec<(u64, String, Box<dyn pipeline::node::Node>)> = Vec::new();
    // Which stages came through the rebuild with the node they had. A channel
    // built from scratch has forgotten its station and its gain, and the
    // interface has to know not to keep showing them.
    let mut reused: Vec<u64> = Vec::new();
    for st in patch.stages() {
        // Reused where it can be, so editing one wire does not reset the
        // detector's noise floor on every other stage in the graph. Two
        // things stop it. A stage that kept its id and changed kind is not
        // the same stage, and `was` is what says which kind the pooled node
        // was built as: reusing an envelope detector as a mixer would hand
        // the graph a node of the wrong type entirely. And a node that
        // cannot be reused for what the stage now asks for is dropped here
        // rather than kept and half corrected: a spectrum cannot resize, and
        // one holding an average of another band is worse than one starting
        // empty.
        let pooled = pool
            .remove(&st.id)
            .filter(|_| was.stage(st.id).is_some_and(|s| s.kind == st.kind))
            .filter(|p| p.node.survives_rebuild(retuned, &st.settings));
        let mut node = match pooled {
            Some(p) => {
                reused.push(st.id);
                p.node
            }
            // The recorder owns an open file, so it is handed in rather than
            // constructed from a description. A patch that asks for one when
            // nothing is recording gets nothing, and the stage waits.
            None if st.kind == RING => match ring.take() {
                Some(r) => Box::new(nodes::RingNode::new(r)) as Box<dyn pipeline::node::Node>,
                None => continue,
            },
            // The radio is handed in at key-up rather than built from a
            // description, so the stage starts idle: it is in the graph
            // whether or not anything is transmitting, exactly as the raw
            // capture is.
            None if st.kind == TX_RADIO => match tx.as_mut().and_then(|t| t.stream.take()) {
                Some(s) => Box::new(nodes::TxSinkNode::new(s)) as Box<dyn pipeline::node::Node>,
                None => Box::new(nodes::TxSinkNode::idle()) as Box<dyn pipeline::node::Node>,
            },
            None if st.kind == "mic" => {
                let src = tx.as_ref().and_then(|t| t.mic.clone());
                match src {
                    // A microphone stage with no microphone is a stage that
                    // cannot say what it would transmit, so it waits.
                    Some(src) => {
                        let level = st.settings.f64_or("level", 1.0) as f32;
                        let band = (
                            st.settings.f64_or("low_hz", 200.0),
                            st.settings.f64_or("high_hz", 3_400.0),
                        );
                        Box::new(nodes::MicNode::with_band(src, level, band))
                            as Box<dyn pipeline::node::Node>
                    }
                    None => continue,
                }
            }
            None => reg.build(&st.kind, &st.settings)?,
        };
        // A stage the receiver derived is described by its settings, so one
        // that came back out of the pool is brought up to date rather than
        // left holding what the last rebuild wanted. A mixer whose shift
        // still followed the old dial put a whole band in the wrong place.
        // Settings a node cannot take as a parameter, such as a filter's
        // designed passband, are refused here and belong to the id instead.
        if crate::patch::Patch::is_derived(st.id) {
            for (name, value) in &st.settings {
                let _ = node.set_param(name, value.clone());
            }
        }
        // What a stage's description says that a parameter cannot carry one
        // value at a time: the band a bank or a detector is limited to, how
        // many things feed the bus, the passband a decimator designs. Every
        // node is handed the description and takes what it understands, on a
        // node that came through the rebuild as much as on a fresh one.
        node.configure(&st.settings);
        made.push((st.id, st.kind.clone(), node));
    }

    // Dropping one stage can leave the next with nothing feeding it, so this
    // settles rather than passing over the list once.
    let mut live: Vec<u64> = made.iter().map(|(id, ..)| *id).collect();
    loop {
        // A mixer's spare input is meant to be empty, so it is not the
        // half-drawn stage this is for.
        let fed = |id: &u64, ins: usize, optional: bool, live: &Vec<u64>| {
            (0..ins).all(|p| match patch.feeding((*id, p)) {
                Some(Source::Span) => true,
                Some(Source::Stage(f, _)) => live.contains(&f),
                None => optional,
            })
        };
        let drop: Vec<u64> = made
            .iter()
            .filter(|(id, ..)| live.contains(id))
            .filter(|(id, _, n)| !fed(id, n.num_inputs(), n.optional_inputs(), &live))
            .map(|(id, ..)| *id)
            .collect();
        if drop.is_empty() {
            break;
        }
        live.retain(|id| !drop.contains(id));
    }

    let mut ids: HashMap<u64, NodeId> = HashMap::new();
    let mut packets = Vec::new();
    for (id, kind, node) in made.into_iter().filter(|(id, ..)| live.contains(id)) {
        let ins = node.num_inputs();
        let label = patch
            .stage(id)
            .map(|s| stage_label(&s.kind, &s.settings))
            .unwrap_or_else(|| kind.clone());
        let nid = b.add_labeled(label, node);
        // Tagged with the patch's own id, which is how the view knows which
        // box on screen is the stage that asked for it, and how the next
        // rebuild finds the node again.
        b.set_tag(nid, id);
        ids.insert(id, nid);
        for p in 0..ins {
            match patch.feeding((id, p)) {
                Some(Source::Span) => {
                    b.connect(span, nid.input(p));
                }
                Some(Source::Stage(f, port)) => {
                    if let Some(from) = ids.get(&f) {
                        b.connect(from.out(port), nid.input(p));
                    }
                }
                None => {}
            }
        }
        // Everything that produces packets meets at the bus, where the
        // protocols run once over all of it. Anything else at the end of a
        // chain is one the operator has not finished, and wiring it to the
        // bus would hand the bus a stream of the wrong type.
        if feeds_bus(kind.as_str()) && patch.is_tail(id) {
            packets.push(nid);
        }
    }

    // A stage added out of dependency order is fed by one that has not been
    // given its `NodeId` yet, so the wires are made again once every stage
    // has one. Connecting an input twice replaces the earlier edge, which is
    // exactly what is wanted here.
    for st in patch.stages().iter().filter(|s| live.contains(&s.id)) {
        let Some(&nid) = ids.get(&st.id) else {
            continue;
        };
        for l in patch.links().iter().filter(|l| l.to.0 == st.id) {
            if let Source::Stage(f, port) = l.from {
                if let Some(from) = ids.get(&f) {
                    b.connect(from.out(port), nid.input(l.to.1));
                }
            }
        }
    }
    Ok((packets, ids, reused))
}

/// A rate as a person reads it, for a node label.
fn hz_label(hz: f64) -> String {
    if hz >= 1e6 {
        format!("{:.3} MHz", hz / 1e6)
    } else {
        format!("{:.1} kHz", hz / 1e3)
    }
}

/// Where a front end is listening inside the current span, for drawing.
///
/// Derived from the same `SubBand` arithmetic the graph is built with rather
/// than from a second guess at it: a marker that says the scanner is somewhere
/// it is not is worse than no marker, because it is believed.
#[derive(Clone, Debug, PartialEq)]
pub enum ScanMark {
    /// One frequency a single-channel front end demodulates.
    Channel { hz: f64, width: f64, label: String },
    /// A band a bank channelizes, and the grid it channelizes it on; or a
    /// band watched for sources, which has no grid and says so with a
    /// spacing of zero.
    ///
    /// `origin` is a real channel centre, not the band edge: the grid is
    /// anchored to the extraction's centre and the band is a window onto it,
    /// so ticks stepped from `lo` would be up to half a channel out.
    Band { lo: f64, hi: f64, origin: f64, spacing: f64, label: String },
}

/// What the scanner table is listening to on this span.
pub fn scan_marks(scanners: &crate::scanners::Scanners, center: f64, rate: f64) -> Vec<ScanMark> {
    let mut out = Vec::new();
    for at in scanners.fronts(center, rate) {
        match &at.front {
            Front::Protocol { hz, .. } => {
                let Some(proto) = at.front.proto() else {
                    continue;
                };
                for m in proto.marks(*hz) {
                    out.push(ScanMark::Channel { hz: m.hz, width: m.width_hz, label: m.label });
                }
            }
            Front::Auto => {
                // No grid to draw: the band is watched whole and whatever
                // is in it is found where it is.
                let Some(band) = at.covered(center, rate) else {
                    continue;
                };
                out.push(ScanMark::Band {
                    lo: band.0,
                    hi: band.1,
                    origin: (band.0 + band.1) / 2.0,
                    spacing: 0.0,
                    label: "auto".into(),
                });
            }
            Front::Banks(widths) => {
                let Some(band) = at.covered(center, rate) else {
                    continue;
                };
                let sub = SubBand::plan(band, rate, 0.0);
                let sub_rate = sub.rate(rate);
                for &width in widths {
                    let n = nodes::BankNode::channels_for(sub_rate, width);
                    let spacing = sub_rate / n as f64;
                    // The band asked for, not the band extracted. Decimation
                    // is by powers of two, so what the bank is handed is up to
                    // twice as wide; the channels out there have their
                    // decoders taken off, and a marker over them would be
                    // saying the receiver listens where it does not.
                    let live = (band.1 - band.0).max(spacing);
                    out.push(ScanMark::Band {
                        lo: band.0,
                        hi: band.1,
                        // Channel 0 sits on the extraction's centre, so every
                        // channel centre is that plus a whole number of
                        // spacings, and a boundary is half a spacing off it.
                        origin: sub.center,
                        spacing,
                        label: {
                            let count = (live / spacing).round() as usize;
                            if width <= OOK_CHANNEL_HZ {
                                format!("OOK x{count}")
                            } else {
                                format!("FSK x{count}")
                            }
                        },
                    });
                }
            }
        }
    }
    out
}

/// A decode as the packet list holds it. Public because a directory rebuilt
/// from the log has to make the same rows the live receiver makes.
pub fn record_of(
    at: std::time::Instant,
    p: &common::Packet,
    d: &pipeline::event::Decoded,
) -> DecodeRecord {
    record(at, p, d)
}

/// One row from a packet and one of the conclusions on it.
///
/// The packet is what carries the evidence: how strongly it was heard, the
/// samples it was read from, the speech it brought and the width it came
/// through. The decode says only what it was. They used to be copied onto the
/// conclusion as well, filled in by four different fallbacks, and the two
/// copies disagreed as soon as one of them was missed.
fn record(
    at: std::time::Instant,
    p: &common::Packet,
    d: &pipeline::event::Decoded,
) -> DecodeRecord {
    DecodeRecord {
        at,
        freq: d.center.as_f64(),
        // The width the packet was heard through, as the front end that
        // produced it declared. This used to be read off the keying where a
        // decode carried no width of its own, which meant a table here of
        // which protocol is heard through what: a guess that had to be kept
        // in step with every front end, and one the packet has always been
        // able to answer for itself.
        channel_hz: f64::from(p.bandwidth_hz),
        model: (d.protocol != nodes::UNKNOWN).then_some(d.protocol),
        modulation: d.modulation.unwrap_or(common::Modulation::Unknown),
        detail: d.detail.clone().or_else(|| d.text.clone()).unwrap_or_default(),
        fields: d.fields.clone(),
        media_type: d.media_type,
        rssi_dbfs: p.rssi_dbfs(),
        snr_db: p.snr_db(),
        bytes: d.payload.clone(),
        crc: d.crc_ok,
        link: d.link.clone(),
        report: d.report.clone(),
        identity: d.identity.clone(),
        iq: p.samples().cloned(),
        audio: p.audio.clone(),
        airtime: d.airtime.clone(),
    }
}

/// Where raw span captures go when nobody says otherwise: beside the packet
/// log, since both are recordings of what was on the air.
/// Where speech models are kept: `models` beside the packet log, one
/// directory per model.
#[cfg(feature = "stt")]
pub fn models_root() -> PathBuf {
    crate::packetlog::PacketLog::default_dir()
        .map(|d| d.with_file_name("models"))
        .unwrap_or_else(|| std::env::temp_dir().join("waveshark-models"))
}

/// Where the files of the model that runs when none was chosen are, or
/// would be fetched to.
#[cfg(feature = "stt")]
pub fn default_model_dir() -> PathBuf {
    let root = models_root();
    stt::model_dir(&root, &stt::default_model_in(&root))
}

pub fn default_capture_dir() -> PathBuf {
    crate::packetlog::PacketLog::default_dir()
        .map(|d| d.with_file_name("captures"))
        .unwrap_or_else(|| std::env::temp_dir().join("waveshark-captures"))
}

/// The spectrum stage the waterfall is drawn from: the receiver's own,
/// or the first the operator drew if they took it out.
///
/// A graph with a spectrum in it draws it where a person is looking,
/// whoever put it there; the rest are strips of their own underneath.
fn main_spectrum(patch: &crate::patch::Patch) -> Option<u64> {
    patch.stages().iter().find(|s| s.kind == "spectrum").map(|s| s.id)
}

fn downcast<T: 'static>(g: &Graph, id: NodeId) -> Option<&T> {
    g.node(id).and_then(|n| n.as_any().downcast_ref::<T>())
}

/// The recorder, as a node.
///
/// It only pushes here. What to keep is decided from decoded events, which do
/// not exist until the decoders downstream have run, so the host makes that
/// call between blocks. A node cannot read the future and should not pretend
/// to.
struct RecordRing {
    /// An `Option` only so the recorder can be taken back out of a graph that
    /// is still running, which a replay does once it has finished writing.
    rec: Option<Recorder>,
}

impl RecordRing {
    fn new(rec: Recorder) -> Self {
        Self { rec: Some(rec) }
    }

    /// Recover the recorder from a node lifted out of a graph, so a rebuild
    /// keeps writing the same file.
    fn from_part(node: Box<dyn pipeline::node::Node>) -> Option<Self> {
        node.into_any().downcast::<nodes::RingNode<Self>>().ok().map(|n| n.into_ring())
    }
}

impl nodes::Ring for RecordRing {
    fn push(&mut self, iq: &[C32]) {
        if let Some(r) = self.rec.as_mut() {
            r.push(iq);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Whether one stage's output is wired into another's input.
    fn feeds(from: &pipeline::graph::TopoNode, to: &pipeline::graph::TopoNode) -> bool {
        from.outputs.iter().any(|(slot, _)| to.inputs.iter().any(|(want, _)| want == slot))
    }

    /// Whether a front end of this kind is in the running graph.
    fn running(rx: &Receiver, kind: &str) -> bool {
        rx.topology().nodes.iter().any(|n| n.kind == kind)
    }

    /// A front end with no band of its own, which is what every front end
    /// except a bank has: only a bank is built over a range.
    pub(super) fn anywhere(front: Front) -> crate::scanners::FrontAt {
        crate::scanners::FrontAt { front, band: (0.0, f64::INFINITY) }
    }

    pub(super) fn plan(rate: f64, center: Hz) -> Plan {
        Plan {
            edits: Default::default(),
            center,
            rate,
            zoom: 1,
            dc_block: true,
            refresh_hz: 30.0,
            smoothing: DEFAULT_SMOOTHING,
            fft: 1024,
            channels: Vec::new(),
            audio: AudioPlan::default(),
            fronts: vec![crate::scanners::FrontAt {
                front: Front::Banks(crate::scanners::DEFAULT_WIDTHS.to_vec()),
                band: (0.0, f64::INFINITY),
            }],
            record: false,
            capture: false,
            capture_dir: crate::chain::default_capture_dir(),
            capture_format: common::SampleFormat::Cu8,
            log: false,
            feeds: Vec::new(),
            tx: None,
            settings: Default::default(),
        }
    }

    /// A plan with nothing on the span but what the operator drew: the
    /// drawing is read against the graph the receiver would draw, and the
    /// difference is what runs on top of it.
    fn manual(patch: crate::patch::Patch) -> Plan {
        let mut p = plan(2_400_000.0, Hz::mhz(433));
        p.fronts.clear();
        p.edits = crate::patch::Edits::diff(&patch, &derived_patch(&p), operator_owns);
        p
    }

    #[test]
    fn a_patch_stage_that_is_not_fed_yet_is_left_out_rather_than_fatal() {
        // A chain is drawn one wire at a time, so a stage with nothing into
        // it is the ordinary state of an edit in progress. The graph refuses
        // an unconnected input port, and refusing the whole receiver for one
        // would make the obvious way to work impossible.
        use crate::patch::Source;
        let mut patch = crate::patch::Patch::default();
        let mix = patch.add("mixer");
        let env = patch.add("envelope");
        patch.connect(Source::Span, (mix, 0));
        let rx = Receiver::build(&manual(patch), Default::default()).expect("a half-drawn patch");
        let topo = rx.topology();
        assert!(topo.nodes.iter().any(|n| n.tag == Some(mix)), "the wired stage runs");
        assert!(!topo.nodes.iter().any(|n| n.tag == Some(env)), "the unwired one waits");
    }

    /// A channel marked as voice is audio like any other, played through its
    /// fader, and named as a conversation on the bus's tap. It is not a front
    /// end: there is no packet in analogue speech, so nothing of it reaches
    /// the packet bus, and the call list and the transcriber read it off the
    /// audio bus, which every demodulator's audio goes through first.
    #[test]
    fn a_channel_marked_as_voice_is_audio_named_on_the_bus() {
        let mut plan = plan(2_400_000.0, Hz::mhz(145));
        plan.fronts.clear();
        let mut ch = chan(1, 25_000.0, Demod::Nfm);
        ch.voice = true;
        plan.channels = vec![ch];
        let rx = Receiver::build(&plan, Default::default()).expect("a voice channel");
        let topo = rx.topology();
        assert!(!topo.nodes.iter().any(|n| n.kind == "voice"), "speech is not a front end");
        assert!(
            !topo.nodes.iter().any(|n| n.kind == "packet_bus"),
            "an analogue channel puts nothing on the packet bus"
        );
        let strips = rx.audio().expect("the bus").bus().strips();
        let fed: Vec<_> = strips.iter().filter(|s| s.is_fed()).collect();
        assert_eq!(fed.len(), 1);
        assert!(fed[0].speech, "the strip is named as a conversation");
        assert_eq!(fed[0].label, "CH1");
    }

    /// The transcript outlives the graph. The transcriber is a stage wired
    /// off the audio bus, and the bus is rebuilt whenever a channel comes or
    /// goes, so a transcript kept inside the node was emptied by adding a
    /// channel: on screen, three reads and no lines.
    #[cfg(feature = "stt")]
    #[test]
    fn the_transcript_survives_a_rebuild() {
        let key = common::ConversationKey::new(crate::audiobus::ANALOGUE, 145_000_000.0)
            .to(Some("CH-rebuild".into()));
        let mut plan = plan(2_400_000.0, Hz::mhz(145));
        plan.fronts.clear();
        plan.channels = vec![chan(1, 25_000.0, Demod::Nfm)];
        let mut rx = Receiver::build(&plan, Default::default()).expect("a receiver");
        let log = rx.transcript().clone();
        log.lock().push(crate::transcripts::Utterance {
            key: key.clone(),
            at: std::time::Instant::now(),
            seconds: 1.0,
            text: "still here".into(),
            settled: true,
            confidence: -0.2,
            credible: true,
        });
        let first = rx.transcriber().expect("a transcriber");
        plan.channels.push(chan(2, -25_000.0, Demod::Nfm));
        rx.rebuild(&plan).expect("a rebuild");
        let second = rx.transcriber().expect("a transcriber");
        // The node's own state did not survive: this is the rebuild that
        // used to take the transcript with it.
        assert_eq!(second.health.reads, 0);
        assert_eq!(first.health.reads, 0);
        assert_eq!(log.lock().latest(&key).map(|u| u.text.as_str()), Some("still here"));
        // And the node the rebuild put there writes into the same one.
        assert!(std::sync::Arc::ptr_eq(&log, rx.transcript()));
    }

    /// Without the mark it is audio and nothing else, which is what an
    /// operator listening to a data channel wants.
    #[test]
    fn an_unmarked_channel_is_audio_and_nothing_else() {
        let mut plan = plan(2_400_000.0, Hz::mhz(145));
        plan.fronts.clear();
        plan.channels = vec![chan(1, 25_000.0, Demod::Nfm)];
        let rx = Receiver::build(&plan, Default::default()).expect("a plain channel");
        let strips = rx.audio().expect("the bus").bus().strips();
        assert!(strips.iter().filter(|s| s.is_fed()).all(|s| !s.speech));
    }

    #[test]
    fn a_patch_that_decodes_reaches_the_packet_bus() {
        // Everything that produces packets meets at the bus, and a decoder
        // the operator wired up by hand is not a special case: without this
        // it would decode into nothing and the packet log would stay empty.
        use crate::patch::Source;
        let mut patch = crate::patch::Patch::default();
        let env = patch.add("envelope");
        let det = patch.add("pulse_detect");
        // The bus is a stage too, so a graph drawn by hand carries its own.
        let bus = patch.add("packet_bus");
        patch.connect(Source::Span, (env, 0));
        patch.connect(Source::Stage(env, 0), (det, 0));
        patch.connect(Source::Stage(det, 0), (bus, 0));
        let rx = Receiver::build(&manual(patch), Default::default()).expect("an OOK patch");
        let topo = rx.topology();
        for id in [env, det] {
            assert!(topo.nodes.iter().any(|n| n.tag == Some(id)), "{id} should be running");
        }
        let bus = topo
            .nodes
            .iter()
            .find(|n| n.tag == Some(bus))
            .expect("a detector needs a bus to detect into");
        let detector = topo.nodes.iter().find(|n| n.tag == Some(det)).unwrap();
        assert!(
            bus.inputs.iter().any(|(s, _)| detector.outputs.iter().any(|(o, _)| o == s)),
            "the bursts have to arrive somewhere"
        );
    }

    #[test]
    fn the_capture_is_always_there_and_always_off() {
        // Switching it on must not rebuild the graph: a rebuild drops every
        // source the auto node has open, which is the transmission somebody
        // pressed the button for. So the stage is in every graph, doing
        // nothing, until it is told otherwise.
        let plan = plan(2_400_000.0, Hz::mhz(433));
        let mut rx = Receiver::build(&plan, Default::default()).expect("a receiver");
        let cap = rx.capture().expect("a capture stage");
        assert!(!cap.is_enabled(), "a capture nobody asked for was running");
        assert!(cap.path().is_none());
        assert!(!rx.capturing());
        rx.set_capture(true);
        assert!(rx.capturing());
        // And it survives a rebuild only because the caller says so again,
        // which is what the radio thread does.
        rx.rebuild(&plan).expect("rebuilt");
        assert!(!rx.capturing(), "the stage comes back as the graph draws it");
        rx.set_capture(true);
        assert!(rx.capturing());
    }

    #[test]
    fn a_stage_can_be_put_between_the_head_and_the_spectrum() {
        // The receiver's own stages read the head of the chain by default,
        // and the reason to draw a graph at all is usually to put something
        // in that gap. A patch that could only hang off the side would leave
        // the one edit anybody wants impossible.
        use crate::patch::Source;
        let mut patch = derived_patch(&manual(crate::patch::Patch::default()));
        let view = derived::SPECTRUM;
        let dec = patch.add("decimate");
        let n = patch.stages().iter().position(|s| s.id == dec).unwrap();
        patch.stages_mut()[n].settings.insert("factor".into(), pipeline::ParamValue::Int(4));
        patch.connect(Source::Span, (dec, 0));
        patch.connect(Source::Stage(dec, 0), (view, 0));
        let plan = manual(patch);
        let rx = Receiver::build(&plan, Default::default()).expect("a tapped spectrum");
        let topo = rx.topology();
        let decim = topo.nodes.iter().find(|n| n.tag == Some(dec)).expect("the stage runs");
        let spectrum = topo.nodes.iter().find(|n| n.tag == Some(view)).expect("a spectrum");
        assert!(
            spectrum.inputs.iter().any(|(s, _)| decim.outputs.iter().any(|(o, _)| o == s)),
            "the spectrum should read the stage, not the head"
        );
        // And the axis has to follow it, or every signal is drawn at four
        // times the offset it arrived on.
        assert_eq!(rx.spectrum_rate(), plan.eff_rate() / 4.0);
    }

    #[test]
    fn a_patch_can_carry_a_spectrum_of_its_own() {
        // Watching a decimated band and the whole span at once is most of
        // the reason to draw a graph rather than read one.
        use crate::patch::Source;
        let mut patch = crate::patch::Patch::default();
        let span = patch.add("spectrum");
        let dec = patch.add("decimate");
        let view = patch.add("spectrum");
        let n = patch.stages().iter().position(|s| s.id == dec).unwrap();
        patch.stages_mut()[n].settings.insert("factor".into(), pipeline::ParamValue::Int(8));
        patch.connect(Source::Span, (span, 0));
        patch.connect(Source::Span, (dec, 0));
        patch.connect(Source::Stage(dec, 0), (view, 0));
        let plan = manual(patch);
        let mut rx = Receiver::build(&plan, Default::default()).expect("a second spectrum");
        let seen = rx.patch_spectra();
        // One strip, not two: the first spectrum is the main plot, and only
        // the other one is a band of its own worth a strip underneath.
        assert_eq!(seen.len(), 1, "the stage should report a spectrum of its own");
        assert_eq!(seen[0].tag, view);
        // Its own band, not the span's: a strip drawn from the dial's rate
        // would put every signal in it at eight times the offset.
        assert_eq!(seen[0].rate, plan.eff_rate() / 8.0);
    }

    #[test]
    fn one_spectrum_is_drawn_once() {
        // The plot behind the waterfall is a spectrum stage like any other,
        // so a graph holding a single one has nothing left to put in a strip.
        use crate::patch::Source;
        let mut patch = crate::patch::Patch::default();
        let view = patch.add("spectrum");
        patch.connect(Source::Span, (view, 0));
        let plan = manual(patch);
        let mut rx = Receiver::build(&plan, Default::default()).expect("one spectrum");
        assert!(rx.patch_spectra().is_empty(), "the main plot was reported as an extra too");
    }

    #[test]
    fn a_patch_survives_a_rebuild_with_its_nodes() {
        // The receiver rebuilds on every retune. Building the patch again
        // from its description each time would reset each stage, which for a
        // burst detector means losing the noise floor it measured.
        use crate::patch::Source;
        let mut patch = crate::patch::Patch::default();
        let env = patch.add("envelope");
        patch.connect(Source::Span, (env, 0));
        let plan = manual(patch);
        let mut rx = Receiver::build(&plan, Default::default()).expect("a patch");
        rx.rebuild(&plan).expect("a retune");
        assert_eq!(
            rx.topology().nodes.iter().filter(|n| n.tag == Some(env)).count(),
            1,
            "the stage should come through the rebuild once, not twice or not at all"
        );
    }

    fn chan(id: u64, offset: f64, demod: Demod) -> ChannelSpec {
        ChannelSpec {
            id,
            label: format!("CH{id}"),
            offset_hz: offset,
            mode: ChanMode::Audio(demod),
            bandwidth_hz: None,
            volume: 1.0,
            muted: false,
            squelch_db: None,
            agc: true,
            voice: false,
            tx: None,
        }
    }

    fn block(n: usize) -> Vec<C32> {
        (0..n)
            .map(|i| {
                let p = std::f32::consts::TAU * 0.01 * i as f32;
                C32::new(p.cos() * 0.2, p.sin() * 0.2)
            })
            .collect()
    }

    #[test]
    fn everything_the_receiver_does_is_in_one_graph() {
        // The point of the whole arrangement. If any of these is missing it
        // is being driven by hand somewhere, which is how a chain ends up
        // invisible to the view, the parameters and the latency accounting.
        let mut p = plan(2_400_000.0, Hz::mhz(433));
        p.channels = vec![chan(1, 100_000.0, Demod::Nfm)];
        let rx = Receiver::build(&p, Sinks::default()).unwrap();
        let labels: Vec<String> = rx.topology().nodes.iter().map(|n| n.label.clone()).collect();
        for want in ["DC block", "Spectrum", "31 kHz bank", "125 kHz bank", "Mixer"] {
            assert!(labels.iter().any(|l| l == want), "{want} is not in {labels:?}");
        }
    }

    #[test]
    fn the_spectrum_keeps_what_the_plan_says_across_a_retune() {
        // The transform cannot be resized or moved, so the spectrum node is
        // the one stage a retune always replaces. Anything only the node held
        // was therefore lost on every move of the dial: the averaging went
        // back to its default each time, which reads as the waterfall
        // changing character on its own.
        let mut p = plan(2_400_000.0, Hz::mhz(433));
        p.smoothing = 0.6;
        p.refresh_hz = 12.0;
        let mut rx = Receiver::build(&p, Sinks::default()).unwrap();
        assert_eq!(rx.spectrum_mut().map(|s| (s.smoothing(), s.refresh())), Some((0.6, 12.0)));

        p.center = Hz::mhz(434);
        rx.rebuild(&p).unwrap();
        assert_eq!(rx.spectrum_mut().map(|s| (s.smoothing(), s.refresh())), Some((0.6, 12.0)));
    }

    #[test]
    fn a_channel_keeps_its_level_across_a_retune() {
        // A fader is a plan value, so it survives being moved by the strip
        // without a rebuild and a rebuild without the strip. It is not an
        // edit either: the operator moved a level the receiver draws, not the
        // graph the receiver drew.
        let mut p = plan(2_400_000.0, Hz::mhz(433));
        let mut ch = chan(1, 100_000.0, Demod::Nfm);
        ch.volume = 0.25;
        p.channels = vec![ch];
        let mut rx = Receiver::build(&p, Sinks::default()).unwrap();
        let level =
            |rx: &Receiver| rx.strips().iter().find(|s| s.channel == Some(1)).map(|s| s.volume);
        assert_eq!(level(&rx), Some(0.25));

        p.channels[0].volume = 0.75;
        assert!(rx.params_only(&p), "a level is a number on a node that is already there");
        rx.apply_params(&p);
        assert_eq!(level(&rx), Some(0.75));
        assert_eq!(rx.edits(), crate::patch::Edits::default(), "the strip owns the fader");

        p.center = Hz::mhz(434);
        rx.rebuild(&p).unwrap();
        assert_eq!(level(&rx), Some(0.75));
    }

    #[test]
    fn a_bank_shows_the_chain_its_channels_run() {
        let rx = Receiver::build(&plan(2_400_000.0, Hz::mhz(433)), Sinks::default()).unwrap();
        let topo = rx.topology();
        let bank = topo.nodes.iter().find(|n| n.label == "31 kHz bank").expect("the 31 kHz bank");
        let inner = bank.inner.first().expect("what a channel runs");
        // One stage per channel now, where there were two: it measures the
        // burst and then runs whichever front end reads it.
        assert!(inner.nodes.iter().any(|n| n.label.contains("Classify")));
        assert!(bank.inner_count > 1, "a bank of one channel is not a bank");
        // What a bank passes on is the bursts its channels detected, decoded
        // or not, which is what a log or an analyser attaches to.
        assert_eq!(bank.outputs[0].1.kind, pipeline::PortKind::Pulses);
    }

    #[test]
    fn adding_a_channel_leaves_the_others_untouched() {
        // The reason a rebuild reuses nodes. Building afresh would cost the
        // first channel its RDS station and its AGC convergence every time a
        // second one was added.
        let mut p = plan(2_400_000.0, Hz::mhz(95));
        p.channels = vec![chan(1, 100_000.0, Demod::Wfm)];
        let mut rx = Receiver::build(&p, Sinks::default()).unwrap();
        rx.process(&block(4096)).unwrap();

        p.channels.push(chan(2, -250_000.0, Demod::Nfm));
        rx.rebuild(&p).unwrap();
        assert_eq!(rx.channels().len(), 2);
        assert!(rx.channels()[0].kept, "the channel that did not change was rebuilt");
        assert!(!rx.channels()[1].kept, "a new channel cannot have kept anything");
    }

    #[test]
    fn a_decode_channel_is_a_front_end_on_the_strip() {
        // A channel that decodes is on both buses: its packets go to the log
        // with every other front end's, and its speech to the mixer under the
        // fader the strip gives it. Drawn after the packet bus, its packets
        // went nowhere at all.
        let mut p = plan(2_400_000.0, Hz::mhz(433));
        p.fronts.clear();
        let mut spec = chan(1, 100_000.0, Demod::Nfm);
        spec.mode = ChanMode::Decode("m17".into());
        p.channels = vec![spec];
        let patch = derived_patch(&p);

        use pipeline::registry::SettingsExt;
        let front = patch.stages().iter().find(|s| s.kind == "m17").expect("the front end");
        assert_eq!(
            front.settings.f64_or("channel_hz", 0.0),
            433_100_000.0,
            "the front end reads the frequency the channel is tuned to",
        );
        let to = |bus: u64, port: usize| {
            patch.links().iter().any(|l| {
                l.to.0 == bus && matches!(l.from, crate::patch::Source::Stage(f, o) if f == front.id && o == port)
            })
        };
        assert!(to(derived::BUS, 0), "its packets never reach the log");
        assert!(to(derived::AUDIO, 1), "its speech never reaches the mixer");

        // And the receiver builds it: a channel refused at negotiation is a
        // patch that describes something that cannot run.
        let rx = Receiver::build(&p, Sinks::default()).unwrap();
        assert_eq!(rx.channels().len(), 1);
        assert!(rx.refused.is_none(), "{:?}", rx.refused);
    }

    /// Every protocol the auto node can place is a mode a strip channel can
    /// be set to, span-wide ones included: the registry is the one list, so
    /// a decoder that reads a fixed allocation is a channel at that
    /// allocation rather than a special case the strip cannot offer.
    #[test]
    fn every_protocol_is_a_channel_mode() {
        for proto in nodes::protocol::all() {
            let shape = proto.shape();
            let hz = proto.default_hz();
            // A span wide enough to hold the channel with the margin the
            // strip demands, at the rate the decoder is fed.
            let rate = (shape.widths[0] * 4.0).max(shape.feed_rate_hz * 2.0).max(2_400_000.0);
            let mut p = plan(rate, Hz(hz as u64));
            p.fronts.clear();
            let mut spec = chan(1, 0.0, Demod::Nfm);
            spec.mode = ChanMode::Decode(proto.id().to_string());
            p.channels = vec![spec];
            let patch = derived_patch(&p);
            assert!(
                patch.stages().iter().any(|s| s.kind == proto.id()),
                "{}: no front end drawn",
                proto.id()
            );
            let rx = Receiver::build(&p, Sinks::default())
                .unwrap_or_else(|e| panic!("{}: {e}", proto.id()));
            assert_eq!(rx.channels().len(), 1, "{}", proto.id());
            assert!(rx.refused.is_none(), "{}: {:?}", proto.id(), rx.refused);
        }
    }

    #[test]
    fn an_auto_channel_watches_the_band_it_was_given() {
        // The scanner table's front end, put where somebody pointed: the
        // band is the channel's own centre and width, not a block's.
        let mut p = plan(2_400_000.0, Hz::mhz(433));
        p.fronts.clear();
        let mut spec = chan(1, 100_000.0, Demod::Nfm);
        spec.mode = ChanMode::Auto;
        spec.bandwidth_hz = Some(40_000.0);
        p.channels = vec![spec];
        let patch = derived_patch(&p);

        use pipeline::registry::SettingsExt;
        let front = patch
            .stages()
            .iter()
            .find(|s| s.kind == "auto" && s.settings.contains_key("channel"))
            .expect("the auto front end");
        assert_eq!(front.settings.f64_or("band_lo_hz", 0.0), 433_080_000.0);
        assert_eq!(front.settings.f64_or("band_hi_hz", 0.0), 433_120_000.0);
        // The tuner's centre is 100 kHz away, so there is no spur inside this
        // band to tell the node about.
        assert!(!front.settings.contains_key("spur_hz"));

        let to = |bus: u64, port: usize| {
            patch.links().iter().any(|l| {
                l.to.0 == bus && matches!(l.from, crate::patch::Source::Stage(f, o) if f == front.id && o == port)
            })
        };
        assert!(to(derived::BUS, 0), "its packets never reach the log");
        assert!(to(derived::AUDIO, 1), "its speech never reaches the mixer");
        // And whatever it finds that produces a picture, on the port it
        // publishes those on: the video bus is where a camera it opened lands,
        // with nothing here knowing which front end read it.
        assert!(to(derived::VIDEO, 2), "its pictures never reach the video bus");

        let rx = Receiver::build(&p, Sinks::default()).unwrap();
        assert_eq!(rx.channels().len(), 1);
        assert!(rx.refused.is_none(), "{:?}", rx.refused);
    }

    #[test]
    fn a_channel_set_by_hand_is_built_at_that_width() {
        // The width is what every filter in the chain is designed around, so
        // setting it has to reach the IF filter and the marker alike, and a
        // channel that changed width cannot keep the old coefficients.
        use pipeline::registry::SettingsExt;
        let mut p = plan(2_400_000.0, Hz::mhz(145));
        let mut spec = chan(1, 0.0, Demod::Nfm);
        spec.bandwidth_hz = Some(25_000.0);
        p.channels = vec![spec.clone()];
        let patch = derived_patch(&p);
        let ifd = patch
            .stages()
            .iter()
            .find(|s| s.settings.get("label").and_then(|v| v.as_str()) == Some("IF decimator"))
            .expect("the IF decimator");
        assert_eq!(ifd.settings.f64_or("passband_hz", 0.0), 12_500.0);

        let mut narrow = spec.clone();
        narrow.bandwidth_hz = Some(12_500.0);
        assert_ne!(
            chan_stage_id("chan_ifdec", &p.channels[0], p.eff_rate()),
            chan_stage_id("chan_ifdec", &narrow, p.eff_rate()),
            "a channel that changed width kept a filter designed for the old one",
        );

        let rx = Receiver::build(&p, Sinks::default()).unwrap();
        assert!(rx.refused.is_none(), "{:?}", rx.refused);
        // And the running receiver treats the change as one. A width applied
        // as a parameter, like a squelch level, changed nothing.
        let mut changed = plan(2_400_000.0, Hz::mhz(145));
        changed.channels = vec![narrow];
        assert!(!rx.params_only(&changed), "a width change was taken as a parameter tweak");
        let mut same = plan(2_400_000.0, Hz::mhz(145));
        let mut sq = spec.clone();
        sq.squelch_db = Some(-20.0);
        same.channels = vec![sq];
        assert!(rx.params_only(&same), "a squelch change should not rebuild");
    }

    #[test]
    fn changing_what_a_channel_listens_to_rebuilds_it() {
        // A rebuild for another reason still gives a channel that moved a
        // clean start: its station, its gain and its squelch floor belong to
        // the frequency it was on.
        let mut p = plan(2_400_000.0, Hz::mhz(95));
        p.channels = vec![chan(1, 100_000.0, Demod::Wfm)];
        let mut rx = Receiver::build(&p, Sinks::default()).unwrap();
        rx.process(&block(4096)).unwrap();

        p.channels = vec![chan(1, 300_000.0, Demod::Wfm)];
        rx.rebuild(&p).unwrap();
        assert!(!rx.channels()[0].kept);
    }

    /// Moving a channel is a number on its mixer, not a graph built again.
    ///
    /// A rebuild throws away every filter's history, the AGC's gain and the
    /// audio resampler's phase, which is heard as a click. A satellite pass
    /// being followed down retunes every few seconds, and clicking at every
    /// step is not listening to it.
    #[test]
    fn moving_a_channel_is_a_parameter_and_not_a_rebuild() {
        let mut p = plan(2_400_000.0, Hz::mhz(145));
        p.channels = vec![chan(1, 20_000.0, Demod::Nfm)];
        let mut rx = Receiver::build(&p, Sinks::default()).unwrap();
        rx.process(&block(4096)).unwrap();

        let mut moved = plan(2_400_000.0, Hz::mhz(145));
        moved.channels = vec![chan(1, 23_500.0, Demod::Nfm)];
        assert!(rx.params_only(&moved), "a retune was taken as a rebuild");
        rx.apply_params(&moved);
        // The mixer is where the frequency lives, and it has to have heard
        // about it: the offset used to be applied only when the graph was
        // built again.
        let mix = rx
            .node_of_stage(chan_stage_id("chan_mix", &moved.channels[0], moved.eff_rate()))
            .expect("a channel has a mixer");
        let shift = rx
            .graph
            .node(mix)
            .and_then(|n| n.params().into_iter().find(|q| q.name == "shift_hz"))
            .and_then(|q| q.value.as_f64())
            .expect("the mixer's shift");
        assert!((shift + 23_500.0).abs() < 0.5, "{shift}");
        // And a width or a mode still is a rebuild: every filter in the
        // chain was designed around those.
        let mut wider = plan(2_400_000.0, Hz::mhz(145));
        let mut w = chan(1, 23_500.0, Demod::Nfm);
        w.bandwidth_hz = Some(6_250.0);
        wider.channels = vec![w];
        assert!(!rx.params_only(&wider), "a width change was taken as a retune");
    }

    #[test]
    fn the_speech_path_is_on_the_graph_like_everything_else() {
        // The call bus used to be a struct in the radio thread fed by hand,
        // so the drawing of the receiver said nothing about where the audio
        // went. Every front end that carries voice publishes it on a port,
        // and the bus is the node on the end of them.
        let mut p = plan(2_400_000.0, Hz::mhz(433));
        p.fronts = vec![crate::scanners::FrontAt {
            front: Front::protocol("m17", 433_475_000.0),
            band: (0.0, f64::INFINITY),
        }];
        let rx = Receiver::build(&p, Sinks::default()).expect("a receiver");
        let topo = rx.topology();
        let m17 = topo.nodes.iter().find(|n| n.label.contains("M17")).expect("an M17 front end");
        let bus = topo.nodes.iter().find(|n| n.label == "Audio").expect("the bus");
        let voice = m17
            .outputs
            .iter()
            .find(|(_, s)| s.kind == PortKind::Voice)
            .expect("speech leaves on a port of its own");
        assert!(
            bus.inputs.iter().any(|(o, _)| *o == voice.0),
            "the speech has to arrive somewhere"
        );
        // And it comes out as audio, at the rate the speaker wants.
        assert_eq!(bus.outputs[0].1.kind, PortKind::Real);
        assert_eq!(bus.outputs[0].1.frame_rate(), crate::audiobus::OUT_HZ);
        assert!(rx.audio().is_some());
        // Speech, and a spare input for the next thing to be wired in. A
        // wire the bus does not read is worse than no wire: it says the
        // audio depends on something it does not.
        let kinds: Vec<(PortKind, bool)> =
            bus.inputs.iter().map(|(_, s)| (s.kind, s.is_silence())).collect();
        assert_eq!(kinds, vec![(PortKind::Voice, false), (PortKind::Real, true)], "{kinds:?}");
    }

    /// A carrier at `offset` from the centre, at full deviation of nothing:
    /// enough for an AM chain to produce a level and an FM chain to open.
    fn carrier(rate: f64, offset: f64, n: usize) -> Vec<C32> {
        (0..n)
            .map(|i| {
                let p = std::f64::consts::TAU * offset * i as f64 / rate;
                C32::new(p.cos() as f32 * 0.5, p.sin() as f32 * 0.5)
            })
            .collect()
    }

    fn rms(pcm: &[f32]) -> f32 {
        (pcm.iter().map(|v| v * v).sum::<f32>() / pcm.len().max(1) as f32).sqrt()
    }

    #[test]
    fn every_channel_is_heard_through_the_bus() {
        // The mix is a node: a channel's audio reaches the speaker by a wire
        // into the bus, at the level its strip on the bus says, and nothing
        // in the radio thread sums anything.
        let mut p = plan(2_400_000.0, Hz::mhz(433));
        p.fronts.clear();
        p.channels = vec![chan(1, 200_000.0, Demod::Am)];
        p.channels[0].volume = 0.5;
        p.audio.master = 1.0;
        let mut rx = Receiver::build(&p, Sinks::default()).unwrap();
        let ch = &rx.channels()[0];
        assert_eq!(ch.port, Some(0), "the channel is wired into the bus");
        let strips = rx.strips();
        assert_eq!(strips.len(), 1);
        assert_eq!(strips[0].channel, Some(1));
        assert_eq!(strips[0].volume, 0.5);
        assert_eq!(strips[0].label, "CH1");
        for _ in 0..4 {
            rx.process(&carrier(2_400_000.0, 200_000.0, 65_536)).unwrap();
        }
        let (out, rate) = rx.audio_out();
        assert_eq!(rate, crate::audiobus::OUT_HZ);
        assert!(rms(out) > 0.01, "the channel is silent at the speaker: {:e}", rms(out));
        assert!(rx.channel_states()[0].level > 0.0, "the meter on the strip saw nothing");
    }

    #[test]
    fn a_channel_added_in_manual_mode_is_heard() {
        // Manual mode used to freeze the channels as they were when it was
        // switched on: the strip still sent its list, the build looked for
        // stages nobody had drawn, and a channel added afterwards was
        // silent while its old stages kept running for nobody.
        let mut p = plan(2_400_000.0, Hz::mhz(433));
        p.fronts.clear();
        p.audio.master = 1.0;
        let mut rx = Receiver::build(&p, Sinks::default()).unwrap();
        // Taking the graph over changes nothing about what runs.
        p.edits = rx.edits();
        assert!(p.edits.is_empty(), "{:?}", p.edits);
        p.channels = vec![chan(1, 200_000.0, Demod::Am)];
        rx.rebuild(&p).unwrap();
        assert_eq!(rx.channels().len(), 1, "the channel was not built");
        for _ in 0..4 {
            rx.process(&carrier(2_400_000.0, 200_000.0, 65_536)).unwrap();
        }
        assert!(rms(rx.audio_out().0) > 0.01, "the channel is silent");

        // Retuning it in manual mode moves it rather than losing it, and
        // the stages it had are not left behind.
        p.edits = rx.edits();
        p.channels[0].offset_hz = -300_000.0;
        p.channels[0].mode = ChanMode::Audio(Demod::Nfm);
        rx.rebuild(&p).unwrap();
        assert_eq!(rx.channels().len(), 1);
        let chan_stages =
            rx.patch().stages().iter().filter(|s| s.settings.contains_key("channel")).count();
        assert_eq!(chan_stages, 9, "an NFM chain is nine stages, and no more were kept");
        // A fader drag in manual mode is a number on the bus, not a rebuild
        // that would drop every source the auto node had open.
        p.channels[0].volume = 0.3;
        assert!(rx.params_only(&p), "a fader change rebuilt the graph");
        rx.apply_params(&p);
        assert_eq!(rx.strips()[0].volume, 0.3);
    }

    #[test]
    fn a_chain_the_operator_drew_reaches_the_speaker() {
        // The spare input on the bus is what a hand-drawn demodulator is
        // wired into. Before the bus took real audio there was nothing to
        // wire it to, and a chain the strip could not name was silent.
        use crate::patch::Source;
        let mut p = plan(2_400_000.0, Hz::mhz(433));
        p.fronts.clear();
        p.audio.master = 1.0;
        let mut patch = derived_patch(&p);
        let mix = patch.add("mixer");
        patch
            .stage_mut(mix)
            .unwrap()
            .settings
            .insert("shift_hz".into(), pipeline::ParamValue::Float(-200_000.0));
        let env = patch.add("envelope");
        patch.connect(Source::Span, (mix, 0));
        patch.connect(Source::Stage(mix, 0), (env, 0));
        let spare = patch
            .stage(derived::AUDIO)
            .and_then(|s| s.settings.get("inputs"))
            .and_then(|v| v.as_i64())
            .expect("the bus says how many inputs it has") as usize
            - 1;
        patch.connect(Source::Stage(env, 0), (derived::AUDIO, spare));
        p.edits = crate::patch::Edits::diff(&patch, &derived_patch(&p), operator_owns);
        let mut rx = Receiver::build(&p, Sinks::default()).unwrap();
        let strips = rx.strips();
        assert_eq!(strips.len(), 1, "{strips:?}");
        assert_eq!(strips[0].channel, None, "it is nobody's channel");
        assert!(!strips[0].voice);
        assert_eq!(strips[0].label, "Envelope", "named after what feeds it");
        for _ in 0..4 {
            rx.process(&carrier(2_400_000.0, 200_000.0, 65_536)).unwrap();
        }
        assert!(rms(rx.audio_out().0) > 0.01, "the chain is silent at the speaker");
        // And there is a new spare behind it.
        let bus = rx.topology().nodes.into_iter().find(|n| n.label == "Audio").unwrap();
        assert_eq!(bus.inputs.len(), 2);
        assert!(bus.inputs[1].1.is_silence());

        // Its level, set by the chain view's route, survives the rebuild a
        // retune causes: the setting went into the patch as well as the node.
        let id = rx.audio_node_id().unwrap();
        rx.set_node_param(id, "vol0", pipeline::ParamValue::Float(0.25)).unwrap();
        p.edits = rx.edits();
        p.center = Hz::mhz(434);
        rx.rebuild(&p).unwrap();
        assert_eq!(rx.strips()[0].volume, 0.25);
    }

    #[test]
    fn an_edit_rides_the_dial_rather_than_freezing_it() {
        // Manual mode used to keep a whole drawing, front ends and all, and
        // rebuild that on every retune: the scanner table stopped following
        // the dial, and a drawing saved on another day brought its tuning
        // with it. An edit is a difference from the derived graph now, so
        // the graph keeps following the dial and the edit stays on it.
        use crate::patch::Source;
        let mut p = plan(2_400_000.0, Hz::mhz(433));
        let mut rx = Receiver::build(&p, Sinks::default()).unwrap();
        let banks_before = rx.bank_channels();
        assert!(!banks_before.is_empty(), "the scanner table put banks on 433");
        // A decimator put between the head and the spectrum, by hand.
        let mut patch = rx.patch().clone();
        let dec = patch.add("decimate");
        patch
            .stage_mut(dec)
            .unwrap()
            .settings
            .insert("factor".into(), pipeline::ParamValue::Int(4));
        patch.connect(Source::Span, (dec, 0));
        patch.connect(Source::Stage(dec, 0), (derived::SPECTRUM, 0));
        p.edits = crate::patch::Edits::diff(&patch, rx.base(), operator_owns);
        assert_eq!(p.edits.stages.len(), 1);
        assert_eq!(p.edits.links.len(), 2, "{:?}", p.edits.links);
        rx.rebuild(&p).unwrap();
        assert_eq!(rx.spectrum_rate(), p.eff_rate() / 4.0, "the edit took");

        // The dial moves to a band with different front ends. The edit is
        // still there and the front ends are the new band's.
        p.center = Hz::mhz(1090);
        p.fronts = vec![crate::scanners::FrontAt {
            front: Front::named("mode_s").unwrap(),
            band: (0.0, f64::INFINITY),
        }];
        rx.rebuild(&p).unwrap();
        assert_eq!(rx.spectrum_rate(), p.eff_rate() / 4.0, "the edit was lost on retune");
        assert!(rx.bank_channels().is_empty(), "the old band's banks came along");
        assert!(running(&rx, "mode_s"), "the new band's front end was not built");
        // And what the receiver reports as the edits is what was made.
        assert_eq!(rx.edits(), p.edits);
    }

    #[test]
    fn a_setting_changed_by_hand_is_an_edit_the_strip_learns_of() {
        // A squelch threshold set in the chain view lands on the node. The
        // strip has to learn of it, or the next thing the strip sends puts
        // it back; and the bus levels the same.
        let mut p = plan(2_400_000.0, Hz::mhz(433));
        p.fronts.clear();
        p.channels = vec![chan(1, 200_000.0, Demod::Nfm)];
        let mut rx = Receiver::build(&p, Sinks::default()).unwrap();
        let sq = rx
            .topology()
            .nodes
            .into_iter()
            .find(|n| n.kind == "squelch")
            .expect("an NFM channel has a squelch");
        rx.set_node_param(sq.id.0, "threshold_db", pipeline::ParamValue::Float(-12.0)).unwrap();
        let bus = rx.audio_node_id().unwrap();
        rx.set_node_param(bus, "master", pipeline::ParamValue::Float(0.3)).unwrap();
        rx.set_node_param(bus, "vol0", pipeline::ParamValue::Float(0.6)).unwrap();
        let (audio, chans) = rx.levels();
        assert_eq!(audio.master, 0.3);
        assert_eq!(chans[0].squelch_db, Some(-12.0));
        assert_eq!(chans[0].volume, 0.6);
        // Not an override: the strip owns these, so they are not in the
        // edits, where they would fight what the strip says next.
        assert!(rx.edits().is_empty(), "{:?}", rx.edits());
    }

    #[test]
    fn scrubbing_the_dial_keeps_a_channel() {
        // A channel is keyed by what it listens to, not by where the dial
        // is. Moving the dial under it changes its offset and nothing else,
        // and that used to build it afresh: every scrub cost every channel
        // its station and its gain.
        let mut p = plan(2_400_000.0, Hz::mhz(95));
        p.channels = vec![chan(1, 100_000.0, Demod::Wfm)];
        let mut rx = Receiver::build(&p, Sinks::default()).unwrap();
        rx.process(&block(4096)).unwrap();
        p.center = Hz(p.center.0 + 50_000);
        p.channels[0].offset_hz = 50_000.0;
        rx.rebuild(&p).unwrap();
        assert!(rx.channels()[0].kept, "the dial moved and the channel was rebuilt");
    }

    #[test]
    fn a_channel_outside_the_span_is_refused_rather_than_demodulated() {
        // Restoring a session tuned elsewhere leaves channels behind that the
        // radio is no longer sampling. Demodulating one shifts a frequency
        // that was never received down to baseband, and the result is noise
        // that sounds like a dead station.
        let mut p = plan(2_400_000.0, Hz::mhz(1090));
        p.channels = vec![chan(1, -994_200_000.0, Demod::Wfm)];
        let rx = Receiver::build(&p, Sinks::default()).unwrap();
        assert!(rx.channels().is_empty());
        assert!(rx.refused.is_none(), "not a fault: the dial fixes it");
    }

    #[test]
    fn zooming_rebuilds_at_the_narrower_rate() {
        let mut p = plan(2_400_000.0, Hz::mhz(433));
        p.zoom = 8;
        let rx = Receiver::build(&p, Sinks::default()).unwrap();
        let topo = rx.topology();
        let zoom = topo.nodes.iter().find(|n| n.label.starts_with("Zoom")).expect("a zoom stage");
        assert_eq!(zoom.outputs[0].1.rate, 300_000.0);
        // Everything downstream sees the narrowed rate, which is the whole
        // reason the zoom is a node rather than something the caller does to
        // the buffer first.
        let bank = topo.nodes.iter().find(|n| n.label == "31 kHz bank").unwrap();
        assert_eq!(bank.inputs[0].1.rate, 300_000.0);
    }

    #[test]
    fn the_log_is_fed_by_every_front_end_at_once() {
        // One file, in the order things arrived, rather than a log per
        // source: the banks and the 1090 MHz decoder all hear bursts, and
        // which of them heard one is a property of the record, not the file.
        let mut p = plan(2_400_000.0, Hz::mhz(433));
        p.log = true;
        let d = std::env::temp_dir().join(format!("sr-chainlog-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&d);
        let rx = Receiver::build(&p, Sinks { packet_log: Some(d.clone()), ..Default::default() })
            .unwrap();
        let topo = rx.topology();
        let bus = topo.nodes.iter().find(|n| n.label == "Packet log").expect("a packet bus");
        assert_eq!(
            bus.inputs.len(),
            crate::scanners::DEFAULT_WIDTHS.len(),
            "every bank tier feeds it"
        );
        // Every input carries detected bursts rather than decoded frames.
        assert!(bus.inputs.iter().all(|(_, s)| s.kind == pipeline::PortKind::Pulses));
        // And what leaves it is one stream, whatever produced it.
        assert_eq!(bus.outputs[0].1.kind, pipeline::PortKind::Packets);
        let _ = std::fs::remove_dir_all(&d);
    }

    #[test]
    fn a_view_reads_the_bus_rather_than_the_demodulator() {
        // The whole shape of it: sources feed the log, the protocols run once
        // over everything on it, and consumers hang off the far side of
        // those. A view wired straight to a demodulator would have to be
        // rebuilt for every new source and would see nothing when the source
        // it knew about was not running; one wired to the bus ahead of the
        // protocols would have to decode the frame for itself.
        let mut p = plan(2_400_000.0, Hz::mhz(1090));
        p.fronts = vec![anywhere(Front::named("mode_s").unwrap())];
        let rx = Receiver::build(&p, Sinks::default()).unwrap();
        let topo = rx.topology();
        let bus = topo.nodes.iter().find(|n| n.label == "Packet log").expect("a bus");
        let decode = topo.nodes.iter().find(|n| n.label == "Protocols").expect("the protocols");
        let rows = topo.nodes.iter().find(|n| n.label == "Dedupe").expect("the dedupe");
        let tracker = topo.nodes.iter().find(|n| n.label == "Tracks").expect("a tracker");
        assert!(feeds(bus, decode), "the protocols are not fed by the bus");
        assert!(feeds(decode, rows), "the dedupe is not fed by the protocols");
        // And every consumer reads the far side of the dedupe, so the map
        // sees the rows the packet list shows rather than the copies of a
        // burst that its neighbouring channels also read.
        assert!(feeds(rows, tracker), "the flight list is not fed by the decoded bus");
        assert_eq!(tracker.inputs[0].1.kind, pipeline::PortKind::Packets);
    }

    /// AIS is a front end like Mode S: it feeds the bus, and the tracker
    /// reads it from there rather than being wired to the demodulator.
    ///
    /// This is the test that says the bus abstraction actually holds. It had
    /// exactly one producer of tracks until AIS, and an abstraction with one
    /// implementation has not been shown to be one.
    #[test]
    fn ais_reaches_the_tracker_through_the_bus_like_mode_s_does() {
        let mut p = plan(2_400_000.0, Hz(162_000_000));
        p.fronts = vec![anywhere(Front::named("ais").unwrap())];
        let rx = Receiver::build(&p, Sinks::default()).unwrap();
        assert!(running(&rx, "ais"), "the AIS decoder is not running");
        let topo = rx.topology();
        let ais = topo.nodes.iter().find(|n| n.label == "162 AIS").expect("an AIS node");
        let bus = topo.nodes.iter().find(|n| n.label == "Packet log").expect("a bus");
        let tracker = topo.nodes.iter().find(|n| n.label == "Tracks").expect("a tracker");
        assert!(feeds(ais, bus), "AIS does not reach the bus");
        // And the tracker reads the far side of the protocols and the dedupe,
        // which is the bus with what each packet decoded to on it and one row
        // per burst.
        let decode = topo.nodes.iter().find(|n| n.label == "Protocols").expect("the protocols");
        let rows = topo.nodes.iter().find(|n| n.label == "Dedupe").expect("the dedupe");
        assert!(feeds(bus, decode), "the protocols are not fed by the bus");
        assert!(feeds(decode, rows), "the dedupe is not fed by the protocols");
        assert!(feeds(rows, tracker), "the tracker is not fed by the decoded bus");
    }

    /// A span wide enough for two protocols runs both of them, and both
    /// reach the same bus.
    ///
    /// This is what the span rather than the dial deciding actually buys: at
    /// 2.4 MS/s in the middle of VHF the receiver has a packet channel and a
    /// pager channel in front of it at once, and hearing only one of them
    /// because its block was written first was never a decision anybody made.
    #[test]
    fn two_front_ends_on_one_span_both_reach_the_bus() {
        let mut p = plan(2_400_000.0, Hz(144_400_000));
        p.fronts = vec![
            anywhere(Front::protocol("aprs", 144_800_000.0)),
            anywhere(Front::protocol("pocsag", 153_350_000.0)),
        ];
        // The pager channel is nine megahertz away, well outside this span,
        // so it is dropped rather than built into a node that would refuse
        // its own input and take the graph down.
        let rx = Receiver::build(&p, Sinks::default()).unwrap();
        assert!(running(&rx, "aprs"));
        assert!(!running(&rx, "pocsag"), "a channel outside the span must not be built");
        assert!(rx.refused.is_some(), "and the interface has to be told why");

        // Both inside the span now.
        let mut p = plan(2_400_000.0, Hz(144_400_000));
        p.fronts = vec![
            anywhere(Front::protocol("aprs", 144_800_000.0)),
            anywhere(Front::protocol("pocsag", 145_000_000.0)),
        ];
        let rx = Receiver::build(&p, Sinks::default()).unwrap();
        assert!(
            running(&rx, "aprs") && running(&rx, "pocsag"),
            "both front ends should run"
        );
        let topo = rx.topology();
        let bus = topo.nodes.iter().find(|n| n.label == "Packet log").expect("a bus");
        for label in ["144.800 APRS", "145.0000 pager"] {
            let node = topo
                .nodes
                .iter()
                .find(|n| n.label == label)
                .unwrap_or_else(|| panic!("no {label} node"));
            let to_bus = node
                .outputs
                .iter()
                .any(|(slot, _)| bus.inputs.iter().any(|(in_slot, _)| in_slot == slot));
            assert!(to_bus, "{label} does not reach the bus");
        }
    }

    /// The banks understand nothing on 162 MHz, so they must not run there:
    /// it would be a pass over every sample to invent unknown bursts out of
    /// GMSK.
    #[test]
    fn the_ism_banks_do_not_run_on_the_ais_band() {
        let mut p = plan(2_400_000.0, Hz(162_000_000));
        p.fronts = vec![anywhere(Front::named("ais").unwrap())];
        let rx = Receiver::build(&p, Sinks::default()).unwrap();
        let topo = rx.topology();
        assert!(
            !topo.nodes.iter().any(|n| n.label.contains("bank")),
            "a channel bank is running on the AIS band"
        );
    }

    /// A feed is a front end, not a special case. It has to reach the bus,
    /// and the tracker has to be there to read it even on a band where this
    /// receiver demodulates nothing of the sort.
    #[test]
    fn a_feed_is_a_front_end_on_a_band_that_has_none() {
        let mut p = plan(2_400_000.0, Hz::mhz(433));
        // Nothing listens on port 1; the graph must build regardless, because
        // a feed that is down is a status line rather than a broken receiver.
        p.feeds = vec![nodes::FeedSpec::new("127.0.0.1", 1, &nodes::feed_nodes::BEAST)];
        let rx = Receiver::build(&p, Sinks::default()).unwrap();
        let topo = rx.topology();
        let feed = topo
            .nodes
            .iter()
            .find(|n| n.label.contains("127.0.0.1:1"))
            .expect("the feed is in the graph");
        let bus = topo.nodes.iter().find(|n| n.label == "Packet log").expect("a bus");
        let feeds_bus = feed
            .outputs
            .iter()
            .any(|(slot, _)| bus.inputs.iter().any(|(in_slot, _)| in_slot == slot));
        assert!(feeds_bus, "the feed does not reach the bus");
        assert!(rx.tracking(), "a Mode S feed should bring the flight list with it");
        let status = rx.feed_status();
        assert_eq!(status.len(), 1);
        assert!(!status[0].connected);
    }

    /// A feed that is carried across a retune keeps its socket: reconnecting
    /// on every tuning change would drop frames for as long as it takes the
    /// far end to accept, for no reason at all.
    #[test]
    fn a_feed_survives_a_retune() {
        let mut p = plan(2_400_000.0, Hz::mhz(433));
        p.feeds = vec![nodes::FeedSpec::new("127.0.0.1", 1, &nodes::feed_nodes::BEAST)];
        let mut rx = Receiver::build(&p, Sinks::default()).unwrap();
        let before = rx.feed_status()[0].spec.clone();
        p.center = Hz::mhz(868);
        rx.rebuild(&p).expect("retune");
        let after = rx.feed_status();
        assert_eq!(after.len(), 1);
        assert_eq!(after[0].spec, before);
    }

    #[test]
    fn retuning_to_a_different_set_of_front_ends_rewires_the_bus() {
        // 433 MHz has two channel banks, 1090 MHz has one Mode S
        // demodulator. The bus is carried over so it keeps the file it is
        // writing, and a node carried over still claiming two inputs makes a
        // graph that cannot be built.
        let mut p = plan(2_400_000.0, Hz::mhz(433));
        let mut rx = Receiver::build(&p, Sinks::default()).unwrap();
        p.center = Hz::mhz(1090);
        p.fronts.clear();
        p.fronts = vec![anywhere(Front::named("mode_s").unwrap())];
        rx.rebuild(&p).expect("a receiver that can retune onto 1090");
        let topo = rx.topology();
        let bus = topo.nodes.iter().find(|n| n.label == "Packet log").expect("a bus");
        assert_eq!(bus.inputs.len(), 1, "only Mode S produces packets here");
    }

    #[test]
    fn the_bus_runs_without_a_file() {
        // Turning the log off stops writing to disk; it must not disconnect
        // every view from the traffic.
        let mut p = plan(2_400_000.0, Hz::mhz(1090));
        p.fronts = vec![anywhere(Front::named("mode_s").unwrap())];
        let rx = Receiver::build(&p, Sinks::default()).unwrap();
        assert!(rx.topology().nodes.iter().any(|n| n.label == "Packet log"));
        assert!(rx.topology().nodes.iter().any(|n| n.label == "Tracks"));
        assert_eq!(rx.logged(), 0, "nothing was asked to be written");
    }

    /// The size on screen is the folder's, not one sink's running total. A
    /// span with no front end that produces packets has no bus and no sink,
    /// and reported 0 B beside a limit of eight gigabytes and a folder
    /// holding eight.
    #[test]
    fn the_log_folder_is_reported_with_nothing_writing_to_it() {
        let d = std::env::temp_dir().join(format!("sr-logfolder-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&d);
        std::fs::create_dir_all(&d).unwrap();
        std::fs::write(d.join("2026-09-08.000.wspkt"), vec![0u8; 65_536]).unwrap();
        let mut p = plan(2_400_000.0, Hz::mhz(2457));
        p.fronts.clear();
        let mut rx =
            Receiver::build(&p, Sinks { packet_log: Some(d.clone()), ..Default::default() })
                .unwrap();
        assert!(!rx.topology().nodes.iter().any(|n| n.label == "Packet log"), "a bus was drawn");
        rx.refresh_log_folder();
        assert_eq!(rx.log_bytes(), 65_536);
        let _ = std::fs::remove_dir_all(&d);
    }

    /// A bank over a scanner's own band, at the width that band asked for.
    fn ism_at(center_mhz: f64, rate: f64) -> Plan {
        let mut p = plan(rate, Hz((center_mhz * 1e6) as u64));
        p.fronts = vec![crate::scanners::FrontAt {
            front: Front::Banks(vec![OOK_CHANNEL_HZ]),
            band: (433.05e6, 434.79e6),
        }];
        p
    }

    #[test]
    fn a_wide_span_does_not_coarsen_the_channels_in_a_scanner_band() {
        // The complaint this was written for. At 60 MS/s a bank over the whole
        // span hits its 1024 channel ceiling and every channel is 60 kHz, far
        // wider than the 25 to 30 kHz an ISM sensor occupies, so several
        // devices share one channel and the detector sees one long burst
        // instead of packets.
        let p = ism_at(433.92, 60_000_000.0);
        let rx = Receiver::build(&p, Sinks::default()).unwrap();
        let ch = rx.bank_channels();
        assert_eq!(ch.len(), 1, "one bank");
        // The band is under 2 MHz, so whatever the span, the channels are the
        // width the block asked for rather than the span divided by 1024.
        let width = 2_000_000.0 / ch[0] as f64;
        assert!(
            width < OOK_CHANNEL_HZ * 1.5,
            "{} channels over the band is {width:.0} Hz each",
            ch[0]
        );
    }

    #[test]
    fn scrubbing_the_dial_does_not_disturb_a_bank() {
        // A bank anchored to the receiver's centre moves every channel and
        // resets every detector on each retune, which is what a drag on the
        // tuner is a hundred of. Anchored to the band, the retune is a change
        // of mixer shift and nothing else.
        let mut p = ism_at(433.92, 10_000_000.0);
        let mut rx = Receiver::build(&p, Sinks::default()).unwrap();
        let before = rx.bank_channels();
        let labels = |t: &pipeline::graph::Topology| -> Vec<String> {
            t.nodes.iter().map(|n| n.label.clone()).collect()
        };
        let shape = labels(&rx.topology());
        // Well inside the span, so the band stays fully covered.
        p.center = Hz((434.5e6) as u64);
        rx.rebuild(&p).unwrap();
        assert_eq!(rx.bank_channels(), before, "the channel grid changed under a retune");
        assert_eq!(labels(&rx.topology()), shape, "the graph was rebuilt differently");
    }

    #[test]
    fn a_bank_decodes_the_band_asked_for_and_not_the_margin_around_it() {
        // The extraction decimates by powers of two, so the bank is handed up
        // to twice the width the block asked for. Those extra channels are
        // real, and left alone they report sensors from outside the band and
        // spend the CPU doing it.
        let p = ism_at(433.92, 10_000_000.0);
        let rx = Receiver::build(&p, Sinks::default()).unwrap();
        let live = rx.bank_channels()[0];
        let marks = scan_marks_of(&p);
        let ScanMark::Band { lo, hi, spacing, origin, .. } = &marks[0] else { panic!("{marks:?}") };
        // The grid the ticks are drawn on has to be the grid the channels are
        // on: a channel centre is the origin plus a whole number of spacings.
        let k = (433.92e6 - origin) / spacing;
        assert!((k - k.round()).abs() < 0.001 || (433.92e6 - origin).abs() < *spacing);
        let asked = ((hi - lo) / spacing).round() as usize;
        assert!(
            live.abs_diff(asked) <= 2,
            "{live} channels are decoding over a band {asked} channels wide"
        );
        assert!(*lo >= 433.0e6 && *hi <= 434.85e6, "the mark covers {lo} to {hi}");
    }

    /// The marks the interface would draw for a plan, for tests about them.
    fn scan_marks_of(p: &Plan) -> Vec<ScanMark> {
        let mut s =
            crate::scanners::Scanners { list: Vec::new(), version: crate::scanners::VERSION };
        s.list.push(crate::scanners::Scanner {
            name: "ISM 433".into(),
            lo: 433.05e6,
            hi: 434.79e6,
            min_rate: 250_000.0,
            channels: Vec::new(),
            margin_hz: 0.0,
            front: Front::Banks(vec![OOK_CHANNEL_HZ]),
            enabled: true,
        });
        scan_marks(&s, p.center.as_f64(), p.eff_rate())
    }

    #[test]
    fn a_band_the_span_has_moved_off_stops_being_channelized() {
        // Nothing to extract, so nothing to run: a bank over a band the radio
        // is no longer sampling would be channelizing the anti-alias filter.
        let mut p = ism_at(433.92, 2_000_000.0);
        p.center = Hz::mhz(868);
        let rx = Receiver::build(&p, Sinks::default()).unwrap();
        assert!(rx.bank_channels().is_empty());
    }

    #[test]
    fn the_log_survives_a_retune() {
        // It holds an open file, and a rebuild that cannot lift it out of the
        // old graph drops it: logging stops at the first retune and nothing
        // says so. Every sink with state has this failure mode.
        let mut p = plan(2_400_000.0, Hz::mhz(433));
        p.log = true;
        let d = std::env::temp_dir().join(format!("sr-keeplog-{}", std::process::id()));
        let mut rx =
            Receiver::build(&p, Sinks { packet_log: Some(d), ..Default::default() }).unwrap();
        p.center = Hz::mhz(868);
        rx.rebuild(&p).unwrap();
        assert!(
            rx.topology().nodes.iter().any(|n| n.label == "Packet log"),
            "the log was dropped by a retune"
        );
        // Still the same node, so still the same open file: a fresh one
        // would have restarted the count.
        assert_eq!(rx.logged(), 0);
    }

    #[test]
    fn switching_the_log_on_later_puts_it_in_the_graph() {
        // How it actually happens: the interface names a directory after the
        // radio thread is already running, so the log arrives at a receiver
        // that was built without one.
        let mut p = plan(2_400_000.0, Hz::mhz(433));
        let mut rx = Receiver::build(&p, Sinks::default()).unwrap();
        let d = std::env::temp_dir().join(format!("sr-latelog-{}", std::process::id()));
        rx.set_packet_log(Some(d));
        p.log = true;
        rx.rebuild(&p).unwrap();
        assert!(
            rx.topology().nodes.iter().any(|n| n.label == "Packet log"),
            "the log never joined the graph"
        );
    }

    #[test]
    fn the_recorder_holds_the_burst_before_anything_decodes_it() {
        // A recording that starts when a decoder reports has already missed
        // the packet, so the ring must run ahead of the banks.
        let mut p = plan(2_400_000.0, Hz::mhz(433));
        p.record = true;
        let dir = std::env::temp_dir().join(format!("sr-chain-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        let rec = Recorder::new(&dir, p.eff_rate(), p.center).unwrap();
        let rx = Receiver::build(&p, Sinks { recorder: Some(rec), ..Default::default() }).unwrap();
        let order: Vec<String> = rx.topology().nodes.iter().map(|n| n.label.clone()).collect();
        let ring = order.iter().position(|l| l == "Recorder").expect("a recorder");
        let bank = order.iter().position(|l| l == "31 kHz bank").expect("a bank");
        assert!(ring < bank, "the recorder runs after the decoders: {order:?}");
        let _ = std::fs::remove_dir_all(&dir);
    }
}

#[cfg(test)]
mod scan_mark_tests {
    use super::*;

    #[test]
    fn the_ism_band_is_marked_where_the_detector_is_looking() {
        let s = crate::scanners::Scanners::default();
        let marks = scan_marks(&s, 433_800_000.0, 2_048_000.0);
        let band = marks
            .iter()
            .find_map(|m| match m {
                ScanMark::Band { lo, hi, spacing, label, .. } => {
                    Some((*lo, *hi, *spacing, label.clone()))
                }
                _ => None,
            })
            .expect("the ISM block should mark a band");
        assert!(band.0 >= 432.0e6 && band.1 <= 435.5e6, "{band:?}");
        // Auto has no channel grid, and the mark says so.
        assert_eq!(band.2, 0.0, "{band:?}");
        assert_eq!(band.3, "auto");
    }

    #[test]
    fn a_bank_block_still_marks_its_grid() {
        let s = crate::scanners::Scanners::parse(
            "[ISM]\nrange = 433.05 - 434.79 MHz\nspan = 250 kHz\nfront = banks\nwidths = 31.25 kHz\n",
        );
        let marks = scan_marks(&s, 433_800_000.0, 2_048_000.0);
        let spacing = marks
            .iter()
            .find_map(|m| match m {
                ScanMark::Band { spacing, .. } => Some(*spacing),
                _ => None,
            })
            .expect("a band");
        assert!(spacing > 0.0 && spacing < 200_000.0, "{spacing}");
    }

    #[test]
    fn a_pager_channel_is_marked_at_its_frequency() {
        let s = crate::scanners::Scanners::parse(
            "[POCSAG]\nrange = 439.9 - 440.1 MHz\nspan = 100 kHz\nfront = pocsag\n\
             channels = 439.9875 MHz\nmargin = 12.5 kHz\n",
        );
        let marks = scan_marks(&s, 439_987_500.0, 500_000.0);
        assert!(
            marks.iter().any(
                |m| matches!(m, ScanMark::Channel { hz, .. } if (*hz - 439_987_500.0).abs() < 1.0)
            ),
            "{marks:?}"
        );
    }
}

/// The transmitter as a graph on its own, for tests.
///
/// The receiver does not use this: keying adds the same stages to the patch
/// it derives, so they are in the chain view with everything else. This
/// builds the same shape without a receiver around it, which is what a test
/// of the modulators wants.
///
/// The same shape as the receive side and built the same way: what goes on
/// air is a chain of stages, so it can be drawn, tapped and parameterised
/// rather than being something the radio thread does to a buffer. A keyed
/// channel is `tone` into a modulator into `radio_tx`, and the sink is what
/// paces it, since the device's write blocks once the radio has enough.
///
/// Everything runs at the radio's rate, because there is no resampler on this
/// side yet. Generating a 1 kHz tone at 2 MS/s is wasteful and honest; a
/// resampler is the next thing this wants.
pub fn transmit_graph(
    tx: &crate::radio::TxSpec,
    mode: crate::radio::TxMode,
    rate: f64,
    center: Hz,
    stream: Box<dyn common::TxStream>,
    mic: Option<std::sync::Arc<dyn audio::AudioSource>>,
) -> Result<Graph> {
    use crate::radio::{TxMode, TxSource};

    let input = StreamSpec {
        kind: PortKind::Real,
        rate,
        center,
        // What the audio occupies, which is what a modulator checks its
        // deviation against rather than assuming the stream is full of it.
        bandwidth: 6_000.0,
        flow: pipeline::port::Flow::Tx,
        ..Default::default()
    };
    // A carrier is a tone at nothing: the modulator sees silence and leaves
    // the carrier where it is, which is exactly an unmodulated transmission.
    // Speech gets no tone laid over it either.
    let level = match (mode, tx.source) {
        (TxMode::Carrier, _) | (_, TxSource::Mic) => 0.0,
        _ => 0.8,
    };
    // The microphone is a stage in front of the modulator rather than
    // something the radio thread pushes in, so what is being transmitted can
    // be seen and tapped between the two.
    // What the audio is limited to, which is what keeps a transmission inside
    // its channel: deviation is only half of Carson and the other half is the
    // highest note the modulator was given, so speech running to 15 kHz
    // through a 2.5 kHz deviation puts 35 kHz on the air where the band plan
    // allows 12.5. The limiting happens in the microphone stage, at the
    // microphone's own rate, because a filter this sharp is unaffordable at
    // the radio's.
    let band = tx_audio_band(mode);
    let head: Box<dyn pipeline::Node> = match (tx.source, mic) {
        (TxSource::Mic, Some(src)) => Box::new(nodes::MicNode::with_band(src, tx.mic_gain, band)),
        (TxSource::Mic, None) => {
            return Err(common::Error::other("no microphone is open to transmit from"))
        }
        (TxSource::Tone, _) => Box::new(nodes::ToneNode::new(tx.tone_hz.max(1.0), level)),
    };
    let modulator: Box<dyn pipeline::Node> = match mode {
        TxMode::Nfm | TxMode::Carrier => Box::new(nodes::FmModNode::narrowband(0.0)),
        TxMode::Fm => Box::new(nodes::FmModNode::new(0.0, nodes::FM_DEVIATION_HZ, 0.25)),
        TxMode::Wfm => Box::new(nodes::FmModNode::wideband(0.0)),
        TxMode::Am => Box::new(nodes::AmModNode::new(0.0, 0.8, 0.25)),
    };
    pipeline::chain(input, vec![head, modulator, Box::new(nodes::TxSinkNode::new(stream))])
}

#[cfg(test)]
mod extraction_tests {
    use super::tests::{anywhere, plan};
    use super::*;

    fn topo_labels(p: &Plan) -> Vec<String> {
        let rx = Receiver::build(p, Sinks::default()).unwrap();
        rx.topology().nodes.iter().map(|n| n.label.clone()).collect()
    }

    #[test]
    fn a_pager_on_a_wide_span_is_mixed_down_before_it_sees_anything() {
        // It used to be handed the whole span and cut its own channel out
        // inside `process`, with a mixer over every sample and a filter of
        // several thousand taps, none of it visible in the chain.
        let mut p = plan(20_000_000.0, Hz::mhz(440));
        p.fronts = vec![anywhere(Front::protocol("pocsag", 439_987_500.0))];
        let labels = topo_labels(&p);
        assert!(labels.iter().any(|l| l.contains("mixer")), "{labels:?}");
        assert!(labels.iter().any(|l| l.starts_with('/')), "{labels:?}");
    }

    #[test]
    fn mode_s_sees_two_megahertz_rather_than_the_whole_span() {
        // Its detector measures an envelope. Over 20 MHz that envelope is
        // every carrier in the span added together, which lifts the floor its
        // preamble threshold is measured against and invents edges.
        let mut p = plan(20_000_000.0, Hz::mhz(1090));
        p.fronts = vec![anywhere(Front::named("mode_s").unwrap())];
        let rx = Receiver::build(&p, Sinks::default()).unwrap();
        let topo = rx.topology();
        let modes = topo.nodes.iter().find(|n| n.kind == "mode_s").expect("a mode s node");
        let rate = modes.inputs[0].1.rate;
        assert!(rate <= 5_000_000.0, "mode s was handed {rate} S/s");
        assert!(rate >= 2_000_000.0, "mode s needs 2 MS/s and got {rate}");
    }

    #[test]
    fn even_a_narrow_span_is_cut_down_before_the_front_end() {
        // Worth doing at 2.4 MS/s too: the mixer replaces the one the node ran
        // internally, and what follows it is a 12.5 kHz channel filtered at
        // 300 kHz instead of at the radio's rate.
        let mut p = plan(2_400_000.0, Hz(439_987_500));
        p.fronts = vec![anywhere(Front::protocol("pocsag", 439_987_500.0))];
        let rx = Receiver::build(&p, Sinks::default()).unwrap();
        let topo = rx.topology();
        let pager = topo.nodes.iter().find(|n| n.kind == "pocsag").expect("a pager node");
        assert!(pager.inputs[0].1.rate <= 400_000.0, "{} S/s", pager.inputs[0].1.rate);
    }

    /// A span-wide front end the span is too narrow for is left out and the
    /// reason is reported. Built anyway, the node refuses its own input at
    /// negotiation and the whole graph fails, so a receiver somebody pinned a
    /// camera on came up with nothing in it at all.
    #[test]
    fn a_span_wide_front_end_the_span_cannot_feed_is_refused_not_built() {
        let mut p = plan(10_000_000.0, Hz(5_865_000_000));
        p.fronts = vec![anywhere(Front::protocol("video", 5_865_000_000.0))];
        let rx = Receiver::build(&p, Sinks::default()).expect("the graph still builds");
        assert!(!rx.topology().nodes.iter().any(|n| n.kind == "video"));
        assert!(rx.refused.is_some(), "and the interface has to be told why");

        // Wide enough, and it is there.
        let mut p = plan(20_000_000.0, Hz(5_865_000_000));
        p.fronts = vec![anywhere(Front::protocol("video", 5_865_000_000.0))];
        let rx = Receiver::build(&p, Sinks::default()).expect("a graph");
        assert!(rx.topology().nodes.iter().any(|n| n.kind == "video"));
        assert!(rx.refused.is_none(), "{:?}", rx.refused);
    }

    #[test]
    fn a_band_that_is_already_the_span_adds_no_nodes() {
        // A mixer that shifts by nothing and a decimator that divides by one
        // are two passes over every sample to achieve nothing.
        let mut p = plan(2_400_000.0, Hz::mhz(1090));
        p.fronts = vec![anywhere(Front::named("mode_s").unwrap())];
        let labels = topo_labels(&p);
        assert!(!labels.iter().any(|l| l.contains("mixer")), "{labels:?}");
    }

    #[test]
    fn two_front_ends_in_one_band_share_one_extraction() {
        let mut p = plan(20_000_000.0, Hz::mhz(145));
        p.fronts = vec![
            anywhere(Front::protocol("aprs", 144_800_000.0)),
            anywhere(Front::protocol("aprs", 144_800_000.0)),
        ];
        let labels = topo_labels(&p);
        let mixers = labels.iter().filter(|l| l.contains("mixer")).count();
        assert_eq!(mixers, 1, "{labels:?}");
    }
}

#[cfg(test)]
mod tx_tests {
    use super::*;
    use crate::radio::{TxMode, TxSource, TxSpec};
    use common::{Device, SampleFormat, Sps};

    fn sink(rate: f64) -> (sources::FileSink, std::sync::Arc<parking_lot::Mutex<Vec<u8>>>) {
        sources::FileSink::in_memory(Sps(rate as u64), SampleFormat::Cs8)
    }

    fn transmit(tx: TxSpec, mode: TxMode, rate: f64, blocks: usize) -> Vec<C32> {
        let (mut dev, buf) = sink(rate);
        let mut g = transmit_graph(&tx, mode, rate, Hz(145_500_000), dev.start_tx().unwrap(), None)
            .unwrap();
        for _ in 0..blocks {
            let b = g.input_buf();
            b.clear();
            b.real_mut().resize(4_800, 0.0);
            g.run().unwrap();
        }
        let id = g.order().last().map(|(id, _)| id).unwrap();
        if let Some(n) = g.node_mut(id) {
            if let Some(s) = n.as_any_mut().downcast_mut::<nodes::TxSinkNode>() {
                s.finish(std::time::Duration::from_millis(50));
            }
        }
        let mut iq = Vec::new();
        SampleFormat::Cs8.convert(&buf.lock(), &mut iq);
        iq
    }

    #[test]
    fn a_keyed_nfm_channel_is_a_tone_on_a_carrier() {
        let rate = 48_000.0;
        let tx = TxSpec { tone_hz: 1_000.0, ..Default::default() };
        let iq = transmit(tx, TxMode::Nfm, rate, 5);
        assert_eq!(iq.len(), 5 * 4_800);

        let mut demod = dsp::FmDemod::new(rate, nodes::NBFM_DEVIATION_HZ);
        let mut audio = Vec::new();
        demod.process(&iq, &mut audio);
        let seg = &audio[1_000..];
        let crossings = seg.windows(2).filter(|w| w[0] <= 0.0 && w[1] > 0.0).count();
        let hz = crossings as f64 * rate / seg.len() as f64;
        assert!((hz - 1_000.0).abs() < 10.0, "recovered {hz:.0} Hz");
    }

    #[test]
    fn the_carrier_mode_transmits_nothing_but_a_carrier() {
        // What a power measurement wants, and the check that a mode with no
        // audio still keys: a steady envelope and no deviation.
        let rate = 48_000.0;
        let iq = transmit(TxSpec::default(), TxMode::Carrier, rate, 2);
        let mut demod = dsp::FmDemod::new(rate, nodes::NBFM_DEVIATION_HZ);
        let mut audio = Vec::new();
        demod.process(&iq, &mut audio);
        let worst = audio[100..].iter().fold(0.0f32, |m, v| m.max(v.abs()));
        assert!(worst < 0.05, "an unmodulated carrier deviated by {worst}");
        let level = iq[100].norm();
        assert!(iq.iter().skip(100).all(|s| (s.norm() - level).abs() < 0.02));
    }

    #[test]
    fn an_am_channel_modulates_its_envelope() {
        let rate = 48_000.0;
        let iq = transmit(TxSpec { tone_hz: 1_000.0, ..Default::default() }, TxMode::Am, rate, 2);
        let (mut lo, mut hi) = (f32::MAX, 0.0f32);
        for s in iq.iter().skip(100) {
            lo = lo.min(s.norm());
            hi = hi.max(s.norm());
        }
        assert!(hi > lo * 3.0, "envelope barely moved: {lo} to {hi}");
    }

    #[test]
    fn a_channel_set_to_the_microphone_transmits_what_it_hears() {
        // Through the same node the microphone feeds, with a known waveform
        // where the room would be: a 1 kHz tone at 48 kHz, resampled to the
        // radio's rate, modulated, and read back off the capture.
        let rate = 48_000.0;
        let mic_rate = 48_000.0;
        let tone: Vec<f32> = (0..4_800)
            .map(|i| 0.8 * (std::f32::consts::TAU * 1_000.0 * i as f32 / mic_rate as f32).sin())
            .collect();
        let src: std::sync::Arc<dyn audio::AudioSource> =
            std::sync::Arc::new(audio::Canned::new(tone, mic_rate, true));

        // Levelling off, because what is under test is the path rather than
        // the leveller: an AGC winding up over the first tenth of a second
        // changes the amplitude while it does it, which is what it is for.
        let tx = TxSpec { source: TxSource::Mic, ..Default::default() };
        let (mut dev, buf) = sink(rate);
        let mut g = transmit_graph(
            &tx,
            TxMode::Nfm,
            rate,
            Hz(145_500_000),
            dev.start_tx().unwrap(),
            Some(src),
        )
        .unwrap();
        for _ in 0..5 {
            let b = g.input_buf();
            b.clear();
            b.real_mut().resize(4_800, 0.0);
            g.run().unwrap();
        }
        let id = g.order().last().map(|(id, _)| id).unwrap();
        if let Some(n) = g.node_mut(id) {
            if let Some(s) = n.as_any_mut().downcast_mut::<nodes::TxSinkNode>() {
                s.finish(std::time::Duration::from_millis(50));
            }
        }
        let mut iq = Vec::new();
        SampleFormat::Cs8.convert(&buf.lock(), &mut iq);

        let mut demod = dsp::FmDemod::new(rate, nodes::NBFM_DEVIATION_HZ);
        let mut audio_back = Vec::new();
        demod.process(&iq, &mut audio_back);
        let seg = &audio_back[2_000..];
        let crossings = seg.windows(2).filter(|w| w[0] <= 0.0 && w[1] > 0.0).count();
        let hz = crossings as f64 * rate / seg.len() as f64;
        assert!((hz - 1_000.0).abs() < 15.0, "what went on air was {hz:.0} Hz");
    }

    #[test]
    fn a_channel_set_to_the_microphone_will_not_key_without_one() {
        // Better than transmitting silence: an over that nobody hears is
        // indistinguishable from a radio that is not working.
        let (mut dev, _b) = sink(48_000.0);
        let tx = TxSpec { source: TxSource::Mic, ..Default::default() };
        let err = transmit_graph(
            &tx,
            TxMode::Nfm,
            48_000.0,
            Hz(145_500_000),
            dev.start_tx().unwrap(),
            None,
        )
        .unwrap_err()
        .to_string();
        assert!(err.contains("microphone"), "unhelpful: {err}");
    }

    #[test]
    fn a_transmission_fits_the_channel_its_mode_is_for() {
        // The band plan says PMR446 is 12.5 kHz, and a transmission has to
        // sit inside that: 99% of the power inside the channel the mode
        // claims. Music through a microphone is the case that broke it,
        // because a sound card hands over the whole audio band.
        let rate = 2_000_000.0;
        let noisy: Vec<f32> = (0..48_000)
            .map(|i| {
                let t = i as f32 / 48_000.0;
                0.5 * (std::f32::consts::TAU * 800.0 * t).sin()
                    + 0.5 * (std::f32::consts::TAU * 11_000.0 * t).sin()
            })
            .collect();
        let src: std::sync::Arc<dyn audio::AudioSource> =
            std::sync::Arc::new(audio::Canned::new(noisy, 48_000.0, true));
        let tx = TxSpec { source: TxSource::Mic, ..Default::default() };
        let (mut dev, buf) = sink(rate);
        let mut g = transmit_graph(
            &tx,
            TxMode::Nfm,
            rate,
            Hz(446_050_000),
            dev.start_tx().unwrap(),
            Some(src),
        )
        .unwrap();
        for _ in 0..4 {
            let b = g.input_buf();
            b.clear();
            b.real_mut().resize(40_000, 0.0);
            g.run().unwrap();
        }
        let id = g.order().last().map(|(i, _)| i).unwrap();
        if let Some(n) = g.node_mut(id) {
            if let Some(s) = n.as_any_mut().downcast_mut::<nodes::TxSinkNode>() {
                s.finish(std::time::Duration::from_millis(50));
            }
        }
        let mut iq = Vec::new();
        common::SampleFormat::Cs8.convert(&buf.lock(), &mut iq);

        const N: usize = 8192;
        let mut spec = dsp::spectrum::Spectrum::new(N);
        spec.smoothing = 1.0;
        spec.process(&iq[40_000..]);
        let db = spec.power_db();
        let power: Vec<f64> = db.iter().map(|d| 10f64.powf(*d as f64 / 10.0)).collect();
        let total: f64 = power.iter().sum();
        // The 12.5 kHz channel, in bins either side of centre.
        let half = ((6_250.0 / rate) * N as f64).ceil() as usize;
        let mid = N / 2;
        let inside: f64 = power[mid - half..=mid + half].iter().sum();
        let frac = inside / total;
        assert!(frac > 0.99, "only {:.1}% of the power is in the channel", frac * 100.0);
    }

    #[test]
    fn each_mode_deviates_by_what_that_mode_means() {
        // The width of an FM transmission is its deviation, and the deviation
        // is the difference between the modes: 2.5 kHz fits a 12.5 kHz
        // channel, 75 kHz is broadcast. Mapping wideband onto the 5 kHz
        // modulator made every WFM transmission a narrowband one, which on a
        // waterfall is a thin line where a 200 kHz block should be.
        let rate = 2_000_000.0;
        for (mode, want) in [
            (TxMode::Nfm, nodes::NBFM_DEVIATION_HZ),
            (TxMode::Fm, nodes::FM_DEVIATION_HZ),
            (TxMode::Wfm, nodes::WBFM_DEVIATION_HZ),
        ] {
            let iq = transmit(TxSpec { tone_hz: 1_000.0, ..Default::default() }, mode, rate, 1);
            // Peak deviation, averaged over 200 sample windows: the capture
            // is eight bit, so a single phase step is dominated by
            // quantisation and reads several kilohertz whatever was sent.
            let peak = iq[1_000..]
                .chunks(200)
                .map(|c| {
                    let turns: f64 = c.windows(2).map(|w| (w[1] * w[0].conj()).arg() as f64).sum();
                    (turns / (c.len() - 1) as f64 / std::f64::consts::TAU * rate).abs()
                })
                .fold(0.0f64, f64::max);
            // The tone is 0.8 of full scale, so it deviates by 0.8 of what
            // the mode allows: full deviation is what full scale audio does.
            let want = want * 0.8;
            assert!(
                (peak - want).abs() < want * 0.15,
                "{} deviated {peak:.0} Hz, expected {want:.0}",
                mode.label()
            );
        }
    }

    #[test]
    fn the_transmit_chain_is_three_stages_ending_in_the_radio() {
        let (mut dev, _b) = sink(48_000.0);
        let g = transmit_graph(
            &TxSpec::default(),
            TxMode::Nfm,
            48_000.0,
            Hz(145_500_000),
            dev.start_tx().unwrap(),
            None,
        )
        .unwrap();
        let topo = g.topology();
        let names: Vec<&str> = topo.nodes.iter().map(|n| n.label.as_str()).collect();
        assert_eq!(names, ["tone", "fm_mod", "radio_tx"]);
        assert!(g.output_spec().is_tx());
    }
}

#[cfg(test)]
mod tx_in_graph_tests {
    use super::*;
    use crate::radio::{ChanMode, ChannelSpec, Demod, TxMode, TxSource, TxSpec};
    use common::{Device, Sps};

    fn plan_with_tx(source: TxSource) -> Plan {
        let mut p = tests::plan(2_000_000.0, Hz(446_000_000));
        p.channels = vec![ChannelSpec {
            id: 1,
            label: "CH1".into(),
            offset_hz: 49_000.0,
            mode: ChanMode::Audio(Demod::Nfm),
            bandwidth_hz: None,
            volume: 0.8,
            muted: false,
            squelch_db: None,
            agc: true,
            voice: false,
            tx: Some(TxSpec { source, ..Default::default() }),
        }];
        p.tx = Some(TxPlan {
            spec: TxSpec { source, ..Default::default() },
            mode: TxMode::Nfm,
            on_air: Hz(446_049_000),
        });
        p
    }

    #[test]
    fn the_transmit_chain_is_built_and_idle_before_anything_is_keyed() {
        // The reason it is in the graph when the key is up: so it can be
        // looked at, and so keying is one node being handed a radio rather
        // than a rebuild. A tone needs no device, so this must build with
        // nothing handed in at all.
        let plan = plan_with_tx(TxSource::Tone);
        let rx = Receiver::build(&plan, Sinks::default()).unwrap();
        assert!(rx.tx_sink().is_some(), "no transmitter stage in the graph");
        assert!(!rx.keyed());
        let topo = rx.topology();
        let kinds: Vec<&str> = topo.nodes.iter().map(|n| n.kind.as_str()).collect();
        for want in ["tx_clock", "tone", "fm_mod", "radio_tx"] {
            assert!(kinds.contains(&want), "{want} missing from {kinds:?}");
        }
    }

    #[test]
    fn keying_hands_the_idle_transmitter_a_radio_without_a_rebuild() {
        let plan = plan_with_tx(TxSource::Tone);
        let mut rx = Receiver::build(&plan, Sinks::default()).unwrap();
        let (mut dev, captured) =
            sources::FileSink::in_memory(Sps(2_000_000), common::SampleFormat::Cs8);
        assert!(rx.key(dev.start_tx().unwrap()), "the transmitter stage was not found");
        assert!(rx.keyed());

        let block = vec![C32::new(0.0, 0.0); 40_000];
        for _ in 0..3 {
            rx.process(&block).unwrap();
        }
        // What went to the radio is on the transmitter's own port, which is
        // what the monitor at the head of the receive chain reads.
        let sent = rx.node_of_stage(derived::TX_RADIO).and_then(|id| rx.graph.buf(id.o())).and_then(|b| b.as_iq());
        assert_eq!(sent.map(<[C32]>::len), Some(40_000), "the monitor port is empty while keyed");
        rx.unkey();
        assert!(!rx.keyed());
        assert_eq!(captured.lock().len(), 3 * 40_000 * 2, "not every block reached the radio");
        // And nothing leaves that port once the key is up, so the monitor
        // stops drawing a transmission that has ended.
        rx.process(&block).unwrap();
        let sent = rx.node_of_stage(derived::TX_RADIO).and_then(|id| rx.graph.buf(id.o())).and_then(|b| b.as_iq());
        assert_eq!(sent.map(<[C32]>::len), Some(0));
    }

    #[test]
    fn what_is_transmitted_is_drawn_on_the_receiver_span() {
        // The point of the monitor: a half duplex radio hears nothing while
        // it transmits, so the operator is shown their own signal where a
        // receiver across the room would hear it. The plan transmits 49 kHz
        // above the centre, so that is where it has to land.
        let plan = plan_with_tx(TxSource::Tone);
        let mut rx = Receiver::build(&plan, Sinks::default()).unwrap();
        let (mut dev, _c) =
            sources::FileSink::in_memory(Sps(2_000_000), common::SampleFormat::Cs8);
        assert!(rx.key(dev.start_tx().unwrap()));
        let quiet = vec![C32::new(0.0, 0.0); 40_000];

        // Off, which is what a full duplex radio wants: the span is what the
        // radio delivered and nothing else.
        rx.process(&quiet).unwrap();
        let seen = |rx: &Receiver| -> Vec<C32> {
            let id = rx.node_of_stage(derived::TX_MONITOR).expect("a monitor stage");
            rx.graph.buf(id.o()).and_then(|b| b.as_iq()).unwrap_or(&[]).to_vec()
        };
        assert!(seen(&rx).iter().all(|s| s.norm() < 1e-6), "silence was drawn on");

        rx.set_tx_monitor(true);
        rx.process(&quiet).unwrap();
        let span = seen(&rx);
        assert_eq!(span.len(), quiet.len(), "the monitor did not pass the span on");
        // Mean frequency, from the phase advance between samples. A tone
        // through the NFM modulator sits within its deviation of the
        // carrier, so this is where the transmission is.
        let turn: C32 = span.windows(2).map(|w| w[1] * w[0].conj()).sum();
        let hz = turn.arg() as f64 * plan.rate / std::f64::consts::TAU;
        assert!((hz - 49_000.0).abs() < 500.0, "the transmission was drawn at {hz:.0} Hz");
        // At the level the modulator produced, which is a quarter of full
        // scale: this is a monitor and not a measurement, so what is drawn is
        // where the transmission is and not how strong a receiver would hear
        // it.
        let peak = span.iter().fold(0.0f32, |a, s| a.max(s.norm()));
        assert!((peak - 0.25).abs() < 0.01, "the transmission was drawn at {peak:.3}");
    }

    #[test]
    fn a_microphone_chain_builds_once_the_microphone_arrives() {
        // With no microphone the mic stage waits, and the chain after it is
        // unfed: a transmitter with nothing to transmit. Handing the
        // microphone in at the rebuild is what completes it.
        let plan = plan_with_tx(TxSource::Mic);
        let mut rx = Receiver::build(&plan, Sinks::default()).unwrap();
        assert!(rx.tx_mic().is_none(), "a mic stage was built with no microphone");

        let src: std::sync::Arc<dyn audio::AudioSource> =
            std::sync::Arc::new(audio::Canned::new(vec![0.0; 4_800], 48_000.0, true));
        rx.set_transmitter(Some(TxSinks { stream: None, mic: Some(src) }));
        rx.rebuild(&plan).unwrap();
        assert!(rx.tx_mic().is_some(), "the microphone stage did not appear");
        assert!(rx.tx_sink().is_some(), "the transmitter stage is missing with a microphone");
    }
}
