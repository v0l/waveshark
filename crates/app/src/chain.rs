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

use crate::scanners::Front;
use common::{Hz, Result, C32};
use dsp::rds::Station;
use nodes::{
    AgcNode, BankNode, DecimateNode, SpectrumNode, SquelchNode, WfmDemodNode,
};
use pipeline::graph::{NodePart, Topology};
use pipeline::{Graph, GraphBuilder, NodeId, Out, PortKind, StreamSpec};

use crate::radio::{ChanMode, ChannelSpec, DecodeRecord, Demod};
use crate::record::Recorder;
use std::path::PathBuf;

/// Channel width for the OOK bank. Below this the measurements show no further
/// gain, because the sensor's own bandwidth and its carrier offset start to
/// matter more than the noise saved.
pub const OOK_CHANNEL_HZ: f64 = 31_250.0;
/// Channel width for the FSK bank. rtl_433 runs at 250 kHz for the same
/// signals; half that still holds a 50 kHz tone separation comfortably.
pub const FSK_CHANNEL_HZ: f64 = 125_000.0;

/// Audio sample rate every channel branch aims for.
const AUDIO_HZ: f64 = 48_000.0;

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

/// What a node in the graph is for, so a rebuild can find it again.
///
/// Keyed by purpose rather than by position: the whole point is that a node
/// keeps its state when the graph around it changes, and its position is
/// exactly what changed. Every node in the receiver is a patch stage now, so
/// there is one kind of purpose left.
#[derive(Clone, PartialEq, Eq, Hash, Debug)]
enum Role {
    /// A stage the operator drew, keyed by its patch id and what it is. The
    /// kind is in the key because editing a patch can replace a stage with a
    /// different one, and reusing an envelope detector as a mixer would hand
    /// the graph a node of the wrong type entirely.
    Patch(u64, String),
}


/// What a channel branch was built for. A branch is only reused while all of
/// this is unchanged, since every one of these decides a filter's
/// coefficients or a mixer's shift.
#[derive(Clone, Copy, PartialEq, Debug)]
struct ChanKey {
    mode: u64,
    offset_bits: u64,
    rate_bits: u64,
}

impl ChanKey {
    fn new(spec: &ChannelSpec, rate: f64) -> Self {
        Self {
            mode: spec.mode.key(),
            offset_bits: spec.offset_hz.to_bits(),
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
    pub snr_db: f32,
}

pub struct Receiver {
    graph: Graph,
    /// What each node is, indexed by `NodeId`, so a rebuild can hand the same
    /// nodes to the new graph.
    roles: Vec<Role>,
    /// The DC blocker, when the graph has one: it is a stage like any other
    /// and can be taken out.
    dc: Option<NodeId>,
    /// What the parts of the receiver that are not drawn yet read.
    head: Out,
    spectrum: Option<NodeId>,
    /// The graph as a description: what is running, in the operator's terms.
    patch: crate::patch::Patch,
    /// The graph as the receiver drew it before the operator's edits, which
    /// is what an edited copy of `patch` is read against to find them.
    base: crate::patch::Patch,
    record: Option<NodeId>,
    /// The raw span capture, which is always in the graph and almost always
    /// switched off; see [`Receiver::set_capture`].
    capture: Option<NodeId>,
    /// The audio bus, where every channel and every voice front end meets.
    audio: Option<NodeId>,
    modes: Option<NodeId>,
    ais: Option<NodeId>,
    aprs: Option<NodeId>,
    pocsag: Option<NodeId>,
    m17: Option<NodeId>,
    banks: Vec<Bank>,
    /// The source detectors, one per band watched.
    sources: Vec<NodeId>,
    chans: Vec<Chan>,
    /// A recorder waiting for the next rebuild to become a node.
    pending_record: Option<RecordRing>,
    /// Where the packet log is written, if it is. Held as a directory rather
    /// than an open file so that a rebuild has something to reopen when the
    /// bus itself had to be built again.
    log_dir: Option<PathBuf>,
    /// Size the packet log's folder may reach, or `None` for no limit.
    log_cap: Option<u64>,
    bus: Option<NodeId>,
    decode: Option<NodeId>,
    tracks: Option<NodeId>,
    /// Where the receiver is, which resolves a position from a single frame.
    location: Option<(f64, f64)>,
    /// Bursts logged before the last rebuild, since the node holding the
    /// count is replaced by each one.
    logged: u64,
    center: Hz,
    rate: f64,
    /// Rate at the spectrum's own input, which is the span's unless the
    /// operator has put something in front of it.
    spectrum_rate: f64,
    /// Spectrum stages the operator added, by patch id. Each is a display of
    /// its own: a patch can watch a decimated band and the whole span at the
    /// same time, which is most of the reason to draw one.
    patch_spectra: Vec<(u64, NodeId)>,
    /// Channels that could not be built, for the status line.
    pub refused: Option<String>,
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
            roles: Vec::new(),
            dc: None,
            head: pipeline::graph::GRAPH_INPUT,
            spectrum: None,
            patch: crate::patch::Patch::default(),
            base: crate::patch::Patch::default(),
            record: None,
            capture: None,
            audio: None,
            modes: None,
            ais: None,
            aprs: None,
            pocsag: None,
            m17: None,
            banks: Vec::new(),
            sources: Vec::new(),
            chans: Vec::new(),
            pending_record: None,
            log_dir: sinks.packet_log,
            log_cap: Some(crate::packetlog::DEFAULT_MAX_BYTES),
            bus: None,
            decode: None,
            tracks: None,
            location: None,
            logged: 0,
            center: plan.center,
            rate: plan.rate,
            spectrum_rate: plan.eff_rate(),
            patch_spectra: Vec::new(),
            refused: None,
        };
        rx.assemble(plan, HashMap::new(), sinks.recorder.map(RecordRing::new))?;
        Ok(rx)
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
        let roles = std::mem::take(&mut self.roles);
        let mut pool: HashMap<Role, NodePart> =
            roles.into_iter().zip(graph.into_parts()).collect();

        // A channel whose mixer shift or filter design would differ is not
        // the same channel, and it does not have to be caught here any more:
        // everything a filter was designed against is in the id the stage is
        // derived under, so a channel that changed asks for stages that were
        // never in the pool.
        let retuned = plan.center != old_center || plan.rate != old_rate;
        pool.retain(|role, _| match role {
            // A bank rebuilds itself internally on a retune and keeps its
            // chains, which is cheaper than building several hundred graphs.
            Role::Patch(_, kind) if kind == "bank" => true,
            // The spectrum's FFT size can change and the node cannot resize,
            // and one holding an average of another band is worse than one
            // starting empty.
            Role::Patch(_, kind) if kind == "spectrum" => {
                !retuned && self.fft_size() == plan.fft
            }
            _ => true,
        });

        // The recorder is a file being written; it survives every rebuild
        // short of being switched off, and a newly started one is waiting
        // here for its place in the graph.
        let ring = self.pending_record.take().or_else(|| {
            // The ring is a stage like any other, so it comes back out of the
            // pool by the same name it went in under.
            let role = Role::Patch(derived::RING, RING.to_string());
            pool.remove(&role).and_then(|p| RecordRing::from_part(p.node))
        });

        self.record = None;
        self.capture = None;
        self.audio = None;
        self.modes = None;
        self.ais = None;
        self.aprs = None;
        self.pocsag = None;
        self.m17 = None;
        self.banks.clear();
        self.sources.clear();
        self.center = plan.center;
        self.rate = plan.rate;
        self.assemble(plan, pool, ring)?;
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
        self.spectrum
            .and_then(|id| self.graph.node(id))
            .and_then(|n| n.as_any())
            .and_then(|a| a.downcast_ref::<SpectrumNode>())
            .map(|s| s.size())
            .unwrap_or(0)
    }

    fn assemble(
        &mut self,
        plan: &Plan,
        mut pool: HashMap<Role, NodePart>,
        ring: Option<RecordRing>,
    ) -> Result<()> {
        let input = StreamSpec::iq(plan.rate, plan.center);
        let mut b = Graph::builder(input);
        let mut roles: Vec<Role> = Vec::new();

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
        let (patch_packets, patch_ids, reused) = match add_patch(
            &mut b,
            &mut roles,
            &mut pool,
            pipeline::graph::GRAPH_INPUT,
            &patch,
            &mut ring,
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
        let stage_of = |kind: &str| -> Option<NodeId> {
            patch
                .stages()
                .iter()
                .find(|s| s.kind == kind)
                .and_then(|s| patch_ids.get(&s.id))
                .copied()
        };
        let dc = stage_of("dc_block");
        let spectrum = stage_of("spectrum");
        let record = stage_of(RING);
        let capture = stage_of("iq_capture");
        let audio = stage_of("audio_bus");

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
            let (hz, width, what) = match at.front {
                Front::Aprs(hz) => (hz, nodes::aprs_nodes::CHANNEL_WIDTH_HZ, "aprs"),
                Front::Pocsag(hz) => (hz, nodes::pocsag_nodes::CHANNEL_WIDTH_HZ, "pocsag"),
                Front::M17(hz) => (hz, nodes::m17_nodes::CHANNEL_WIDTH_HZ, "m17"),
                _ => continue,
            };
            if (hz - plan.center.as_f64()).abs() > plan.eff_rate() / 2.0 - width {
                refused =
                    Some(format!("{:.4} MHz is too near the span edge for {what}", hz / 1e6));
            }
        }
        let modes = of_kind("mode_s").first().copied();
        let ais = of_kind("ais").first().copied();
        let aprs = of_kind("aprs").first().copied();
        let pocsag = of_kind("pocsag").first().copied();
        let m17 = of_kind("m17").first().copied();
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
            if spec.offset_hz.abs() > plan.eff_rate() / 2.0 {
                refused = Some(format!(
                    "{:.4} MHz is outside the span",
                    (plan.center.as_f64() + spec.offset_hz) / 1e6,
                ));
                continue;
            }
            if plan.eff_rate() < spec.mode.min_rate() {
                refused = Some(format!(
                    "{} needs a span of at least {:.0} kHz; this one is {:.0} kHz",
                    spec.mode.label(),
                    spec.mode.min_rate() / 1e3,
                    plan.eff_rate() / 1e3,
                ));
                continue;
            }
            let of = |what: &str| -> Option<NodeId> {
                patch_ids.get(&chan_stage_id(what, spec, plan.eff_rate())).copied()
            };
            // A played channel ends in the blend; a decoded one ends in its
            // front end, which is heard only if it has speech to give.
            let last = if spec.mode.is_decode() { "chan_front" } else { "chan_blend" };
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
                    ChanMode::Decode(kind) => tail.out(
                        VOICE_TAILS
                            .iter()
                            .find(|(k, _)| k == kind)
                            .map(|(_, port)| *port)
                            .unwrap_or(0),
                    ),
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

        // The bus, the protocols and the tracker are stages too now, so this
        // is a matter of finding them: the parts of the receiver that talk to
        // them need a node id, not a construction.
        let bus = of_kind("packet_bus").first().copied();
        let decode = of_kind("protocols").first().copied();
        let tracks = of_kind("tracks").first().copied();

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
                    .and_then(|n| n.as_any())
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
        if let Some(id) = bus {
            let want = self.log_dir.is_some();
            if let Some(n) = graph
                .node_mut(id)
                .and_then(|n| n.as_any_mut())
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
        if let Some(n) = dc
            .and_then(|id| graph.node_mut(id))
            .and_then(|n| n.as_any_mut())
            .and_then(|a| a.downcast_mut::<nodes::DcBlockNode>())
        {
            n.set_enabled(plan.dc_block);
        }
        if let Some(n) = spectrum
            .and_then(|id| graph.node_mut(id))
            .and_then(|n| n.as_any_mut())
            .and_then(|a| a.downcast_mut::<SpectrumNode>())
        {
            n.set_refresh(plan.refresh_hz);
        }
        for c in chans.iter() {
            // Applied every time rather than only on a fresh node: a channel
            // whose nodes were reused still has to be told what the channel
            // list now says about its squelch and its gain control.
            if let (Some(id), Some(db)) = (c.squelch, c.spec.squelch_db) {
                if let Some(sq) = graph
                    .node_mut(id)
                    .and_then(|n| n.as_any_mut())
                    .and_then(|a| a.downcast_mut::<SquelchNode>())
                {
                    sq.set_threshold_db(db);
                }
            }
            if let Some(a) = c
                .agc
                .and_then(|id| graph.node_mut(id))
                .and_then(|n| n.as_any_mut())
                .and_then(|a| a.downcast_mut::<AgcNode>())
            {
                a.set_enabled(c.spec.agc);
            }
        }
        self.graph = graph;
        self.dc = dc;
        self.head = head;
        self.patch = patch;
        self.base = base;
        self.roles = roles;
        self.spectrum = spectrum;
        self.record = record;
        self.capture = capture;
        self.audio = audio;
        self.bus = bus;
        self.decode = decode;
        self.ais = ais;
        self.aprs = aprs;
        self.pocsag = pocsag;
        self.m17 = m17;
        self.tracks = tracks;
        // A tracker built fresh has to be told where the receiver is, which
        // is what resolves a position from a single frame.
        if let Some((lat, lon)) = self.location {
            self.set_location(lat, lon);
        }
        self.modes = modes;
        self.banks = banks
            .into_iter()
            .map(|id| {
                let channels = self
                    .graph
                    .node(id)
                    .and_then(|n| n.as_any())
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
        self.graph.run()?;
        self.read_back();
        Ok(())
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
    pub fn apply_params(&mut self, plan: &Plan) {
        for (want, have) in plan.channels.iter().zip(self.chans.iter_mut()) {
            have.spec = want.clone();
        }
        /// One channel's settable numbers, lifted out of `self.chans` so the
        /// graph can be borrowed mutably while they are applied.
        struct Update {
            squelch: Option<NodeId>,
            db: Option<f32>,
            agc: Option<NodeId>,
            on: bool,
        }
        let updates: Vec<Update> = self
            .chans
            .iter()
            .map(|c| Update {
                squelch: c.squelch,
                db: c.spec.squelch_db,
                agc: c.agc,
                on: c.spec.agc,
            })
            .collect();
        // The levels live on the bus, one strip per channel.
        let levels: Vec<(usize, f32, bool, String)> = self
            .chans
            .iter()
            .filter_map(|c| {
                c.port.map(|k| (k, c.spec.volume, c.spec.muted, c.spec.label.clone()))
            })
            .collect();
        if let Some(b) = self.audio_mut().map(|n| n.bus_mut()) {
            for (k, volume, muted, label) in levels {
                if let Some(s) = b.strip_mut(k) {
                    s.volume = volume;
                    s.muted = muted;
                    s.label = label;
                }
            }
        }
        // And in the description, so a rebuild draws what is running.
        if let Some(st) = self.patch.stage_mut(derived::AUDIO) {
            for c in &self.chans {
                let Some(k) = c.port else { continue };
                strip_settings(&mut st.settings, k, c.spec.volume, c.spec.muted, &c.spec.label);
            }
        }
        for Update { squelch, db, agc, on } in updates {
            if let (Some(id), Some(db)) = (squelch, db) {
                if let Some(sq) = self
                    .graph
                    .node_mut(id)
                    .and_then(|n| n.as_any_mut())
                    .and_then(|a| a.downcast_mut::<SquelchNode>())
                {
                    sq.set_threshold_db(db);
                }
            }
            if let Some(a) = agc
                .and_then(|id| self.graph.node_mut(id))
                .and_then(|n| n.as_any_mut())
                .and_then(|a| a.downcast_mut::<AgcNode>())
            {
                a.set_enabled(on);
            }
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
        self.spectrum
            .and_then(|id| downcast::<SpectrumNode>(&self.graph, id))
            .map(|s| s.is_fresh())
            .unwrap_or(false)
    }

    pub fn power_db(&mut self) -> &[f32] {
        self.spectrum_mut().map(|s| s.power_db()).unwrap_or(&[])
    }

    pub fn adc(&mut self) -> nodes::AdcHealth {
        self.spectrum_mut().map(|s| s.adc()).unwrap_or_default()
    }

    pub fn modes_on(&self) -> bool {
        self.modes.is_some() || self.auto_wide("mode_s")
    }

    pub fn ais_on(&self) -> bool {
        self.ais.is_some() || self.auto_wide("ais")
    }

    pub fn aprs_on(&self) -> bool {
        self.aprs.is_some()
    }

    pub fn pocsag_on(&self) -> bool {
        self.pocsag.is_some()
    }

    /// The audio bus, for the subscriptions, the levels, the meters and what
    /// it is playing. `None` only when the patch could not be built at all.
    pub fn audio(&self) -> Option<&crate::audiobus::AudioBusNode> {
        downcast::<crate::audiobus::AudioBusNode>(&self.graph, self.audio?)
    }

    pub fn audio_mut(&mut self) -> Option<&mut crate::audiobus::AudioBusNode> {
        let id = self.audio?;
        self.graph
            .node_mut(id)
            .and_then(|n| n.as_any_mut())
            .and_then(|a| a.downcast_mut::<crate::audiobus::AudioBusNode>())
    }

    /// The bus's position in the running graph, for setting its parameters
    /// by the same route the chain view uses.
    pub fn audio_node_id(&self) -> Option<usize> {
        self.audio.map(|id| id.0)
    }

    /// This block's mix as it leaves for the speaker: stereo, interleaved,
    /// and the frame rate it is at.
    pub fn audio_out(&self) -> (&[f32], f64) {
        let pcm = self
            .audio
            .and_then(|id| self.graph.buf(id.o()))
            .and_then(|p| p.as_real())
            .unwrap_or(&[]);
        let rate = self
            .audio
            .and_then(|id| self.graph.spec_of(id.o()))
            .map(|s| s.frame_rate())
            .unwrap_or(crate::audiobus::OUT_HZ);
        (pcm, rate)
    }

    /// Every input of the bus, as the strip draws it.
    pub fn strips(&self) -> Vec<StripState> {
        let Some(bus) = self.audio().map(|n| n.bus()) else { return Vec::new() };
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
    fn voices(&self) -> Vec<common::Voice> {
        self.graph
            .order()
            .flat_map(|(id, _)| {
                let out = id.o();
                let voice = self.graph.spec_of(out).map(|s| s.kind) == Some(PortKind::Voice);
                let outs = self.graph.node(id).map(|n| n.num_outputs()).unwrap_or(1);
                let mut ports: Vec<Out> = voice.then_some(out).into_iter().collect();
                ports.extend(
                    (1..outs)
                        .map(|p| id.out(p))
                        .filter(|o| self.graph.spec_of(*o).map(|s| s.kind) == Some(PortKind::Voice)),
                );
                ports
            })
            .filter_map(|o| self.graph.buf(o).and_then(|p| p.as_voice()))
            .flat_map(|v| v.iter().cloned())
            .collect()
    }

    /// Whether an M17 front end is running anywhere: a stage on a channel,
    /// or one the auto node built for a source it found.
    pub fn m17_on(&self) -> bool {
        self.voices().iter().any(|v| v.system == "M17")
    }

    /// Whether anything is tracking aircraft, from the local demodulator or
    /// from a feed.
    pub fn tracking(&self) -> bool {
        self.tracks.is_some()
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
            let Some(spec) = self.graph.spec_of(id.o()) else { continue };
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
                    snr_db: s.peak_snr_db,
                });
            }
        }
        out
    }

    /// The key status of every TETRA front end in the graph, placed by hand
    /// or by a scanner: what the key manager shows, and how a recovered key
    /// reaches persistence. Deduplicated by cell, since two front ends on
    /// the same carrier report the same cell.
    pub fn tetra_key_status(&self) -> Vec<nodes::tetra_nodes::KeyStatus> {
        let mut out: Vec<nodes::tetra_nodes::KeyStatus> = Vec::new();
        let mut push = |s: nodes::tetra_nodes::KeyStatus| {
            if !out.iter().any(|e| (e.mcc, e.mnc, e.colour) == (s.mcc, s.mnc, s.colour)) {
                out.push(s);
            }
        };
        for (id, name) in self.graph.order() {
            if name == "tetra" {
                if let Some(t) = downcast::<nodes::TetraNode>(&self.graph, id) {
                    if let Some(s) = t.key_status() {
                        push(s);
                    }
                }
            }
        }
        for &id in &self.sources {
            if let Some(a) = downcast::<nodes::AutoNode>(&self.graph, id) {
                for s in a.inner_tetra_status() {
                    push(s);
                }
            }
        }
        out
    }

    /// Install a key for a cell colour on every TETRA front end, so traffic
    /// on that cell decodes. From the key manager, for a manual key.
    #[cfg(feature = "tea")]
    pub fn set_tetra_key(&mut self, colour: u8, key: decode::tea::Key) {
        let ids: Vec<_> = self.graph.order().filter(|(_, n)| *n == "tetra").map(|(id, _)| id).collect();
        for id in ids {
            if let Some(n) = self.graph.node_mut(id) {
                if let Some(t) = n.as_any_mut().and_then(|a| a.downcast_mut::<nodes::TetraNode>()) {
                    t.add_key(colour, key);
                }
            }
        }
        for &id in &self.sources.clone() {
            if let Some(n) = self.graph.node_mut(id) {
                if let Some(a) = n.as_any_mut().and_then(|a| a.downcast_mut::<nodes::AutoNode>()) {
                    a.set_inner_tetra_key(colour, key);
                }
            }
        }
    }

    /// Install a TA61 identity secret for a cell colour on every TETRA front
    /// end, so its encrypted identities show as real subscribers.
    #[cfg(feature = "tea")]
    pub fn set_tetra_id_secret(&mut self, colour: u8, c: [u8; 8]) {
        let ids: Vec<_> =
            self.graph.order().filter(|(_, n)| *n == "tetra").map(|(id, _)| id).collect();
        for id in ids {
            if let Some(n) = self.graph.node_mut(id) {
                if let Some(t) = n.as_any_mut().and_then(|a| a.downcast_mut::<nodes::TetraNode>()) {
                    t.add_id_secret(colour, c);
                }
            }
        }
        for &id in &self.sources.clone() {
            if let Some(n) = self.graph.node_mut(id) {
                if let Some(a) = n.as_any_mut().and_then(|a| a.downcast_mut::<nodes::AutoNode>()) {
                    a.set_inner_tetra_id_secret(colour, c);
                }
            }
        }
    }

    /// The span-wide decoders the auto nodes are running, by stage name.
    fn auto_wide(&self, name: &str) -> bool {
        self.sources
            .iter()
            .filter_map(|&id| downcast::<nodes::AutoNode>(&self.graph, id))
            .any(|n| n.wide().contains(&name))
    }

    /// The raw span capture, for switching on and for reading how far it has
    /// got.
    pub fn capture(&self) -> Option<&nodes::IqCaptureNode> {
        downcast::<nodes::IqCaptureNode>(&self.graph, self.capture?)
    }

    /// Start or stop writing the span to disk.
    ///
    /// A parameter rather than a rebuild: the point of a capture is the
    /// transmission happening right now, and rebuilding the graph to add a
    /// stage would drop every source the auto node has open.
    pub fn set_capture(&mut self, on: bool) {
        let Some(id) = self.capture else { return };
        if let Some(n) = self
            .graph
            .node_mut(id)
            .and_then(|n| n.as_any_mut())
            .and_then(|a| a.downcast_mut::<nodes::IqCaptureNode>())
        {
            n.set_enabled(on);
        }
    }

    pub fn capturing(&self) -> bool {
        self.capture().is_some_and(|n| n.is_enabled() && !n.is_full())
    }

    /// Add the capture folder up again, for the status that reports it
    /// against the limit. Throttled inside the node.
    pub fn refresh_capture_folder(&mut self) {
        let Some(id) = self.capture else { return };
        if let Some(n) = self
            .graph
            .node_mut(id)
            .and_then(|n| n.as_any_mut())
            .and_then(|a| a.downcast_mut::<nodes::IqCaptureNode>())
        {
            n.refresh_folder();
        }
    }

    /// How large the capture folder may get. Raising it lets a capture that
    /// stopped be started again, which is what pressing the button after
    /// reading why it stopped is asking for.
    pub fn set_capture_cap(&mut self, bytes: u64) {
        let Some(id) = self.capture else { return };
        let mb = bytes as f64 / (1u64 << 20) as f64;
        let _ = self.set_node_param(id.0, "budget_mb", pipeline::ParamValue::Float(mb));
    }

    pub fn recorder_mut(&mut self) -> Option<&mut Recorder> {
        let id = self.record?;
        self.graph
            .node_mut(id)
            .and_then(|n| n.as_any_mut())
            .and_then(|a| a.downcast_mut::<nodes::RingNode<RecordRing>>())
            .and_then(|r| r.ring_mut().rec.as_mut())
    }

    /// Shape of everything running, for the chain view.
    pub fn topology(&self) -> Topology {
        self.graph.topology()
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
        if let Some(st) = self
            .graph
            .tag_of(pipeline::graph::NodeId(id))
            .and_then(|tag| self.patch.stage_mut(tag))
        {
            st.settings.insert(name.to_string(), value);
        }
        Ok(affects_rate)
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
        crate::patch::Edits::diff(&self.patch, &self.base)
    }

    /// The levels as the nodes hold them, for the plan to follow.
    ///
    /// A fader or a squelch set through the chain view lands on the node,
    /// and the strip has to learn of it or the next thing the strip sends
    /// puts it back. Returns the bus levels and each running channel's
    /// settings as the graph has them.
    pub fn levels(&self) -> (AudioPlan, Vec<ChannelSpec>) {
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
                let mut spec = c.spec.clone();
                if let Some(s) = c.port.and_then(|k| bus.and_then(|b| b.strips().get(k))) {
                    spec.volume = s.volume;
                    spec.muted = s.muted;
                    if !s.label.is_empty() {
                        spec.label = s.label.clone();
                    }
                }
                if let Some(sq) = c.squelch.and_then(|id| downcast::<SquelchNode>(&self.graph, id)) {
                    spec.squelch_db = Some(sq.threshold_db());
                }
                if let Some(a) = c.agc.and_then(|id| downcast::<AgcNode>(&self.graph, id)) {
                    spec.agc = a.is_enabled();
                }
                spec
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
    pub fn patch_spectra(&mut self) -> Vec<(u64, Vec<f32>, f64, f64)> {
        let ids = self.patch_spectra.clone();
        let mut out = Vec::with_capacity(ids.len());
        for (tag, id) in ids {
            let Some(n) = self
                .graph
                .node_mut(id)
                .and_then(|n| n.as_any_mut())
                .and_then(|a| a.downcast_mut::<SpectrumNode>())
            else {
                continue;
            };
            let (rate, center) = (n.rate(), n.center().as_f64());
            out.push((tag, n.power_db().to_vec(), center, rate));
        }
        out
    }

    pub fn latency_ms(&self, i: usize) -> f64 {
        let Some(c) = self.chans.get(i) else { return 0.0 };
        self.graph.latency_of(c.tail) as f64 / c.audio_rate.max(1.0) * 1e3
    }



    pub fn set_refresh(&mut self, hz: f32) {
        if let Some(s) = self.spectrum_mut() {
            s.set_refresh(hz);
        }
    }

    pub fn set_smoothing(&mut self, v: f32) {
        if let Some(s) = self.spectrum_mut() {
            s.set_smoothing(v);
        }
    }

    fn spectrum_mut(&mut self) -> Option<&mut SpectrumNode> {
        self.spectrum
            .and_then(|id| self.graph.node_mut(id))
            .and_then(|n| n.as_any_mut())
            .and_then(|a| a.downcast_mut::<SpectrumNode>())
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
        self.dc
            .and_then(|id| self.graph.node_mut(id))
            .and_then(|n| n.as_any_mut())
            .and_then(|a| a.downcast_mut::<nodes::DcBlockNode>())
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
        let Some(n) =
            self.decode.and_then(|id| downcast::<nodes::PacketDecodeNode>(&self.graph, id))
        else {
            return Vec::new();
        };
        n.hits().iter().map(|d| record(at, d)).collect()
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
        self.roles
            .iter()
            .enumerate()
            .filter(|(_, r)| matches!(r, Role::Patch(_, kind) if kind == "feed"))
            .filter_map(|(k, _)| downcast::<nodes::FeedNode>(&self.graph, NodeId(k)))
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
    pub fn log_bytes(&self) -> u64 {
        self.bus
            .and_then(|id| downcast::<nodes::PacketBusNode>(&self.graph, id))
            .map(|b| b.sink_bytes())
            .unwrap_or(0)
    }

    pub fn log_full(&self) -> bool {
        self.bus
            .and_then(|id| downcast::<nodes::PacketBusNode>(&self.graph, id))
            .is_some_and(|b| b.sink_full())
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
        self.bus
            .and_then(|id| self.graph.node_mut(id))
            .and_then(|n| n.as_any_mut())
            .and_then(|a| a.downcast_mut::<nodes::PacketBusNode>())
    }

    /// Tracks heard recently, in the order they were first heard.
    pub fn tracks(&self, now: std::time::Instant) -> Vec<crate::tracks::Track> {
        self.tracks
            .and_then(|id| downcast::<crate::tracks::TracksNode>(&self.graph, id))
            .map(|n| n.rows(now))
            .unwrap_or_default()
    }

    /// Tell the tracker roughly where the receiver is.
    pub fn set_location(&mut self, lat: f64, lon: f64) {
        self.location = Some((lat, lon));
        if let Some(n) = self
            .tracks
            .and_then(|id| self.graph.node_mut(id))
            .and_then(|n| n.as_any_mut())
            .and_then(|a| a.downcast_mut::<crate::tracks::TracksNode>())
        {
            n.set_reference(lat, lon);
        }
    }

    /// Packets written to the log since the receiver started.
    pub fn logged(&self) -> u64 {
        self.logged
            + self
                .bus
                .and_then(|id| downcast::<nodes::PacketBusNode>(&self.graph, id))
                .map(|n| n.written())
                .unwrap_or(0)
    }

    /// Take the recorder back out, after a replay that wrote one.
    #[cfg_attr(not(test), allow(dead_code))]
    pub fn take_recorder(&mut self) -> Option<Recorder> {
        let id = self.record.take()?;
        self.graph
            .node_mut(id)
            .and_then(|n| n.as_any_mut())
            .and_then(|a| a.downcast_mut::<nodes::RingNode<RecordRing>>())
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

/// Bandwidth a Mode S transmission occupies, for the log's channel column.
const MODES_BAND_HZ: f64 = 2_000_000.0;

/// The front ends a strip channel can run, with the channel each expects.
///
/// Asked of the registry, not listed here. A decode stage that declares a
/// channel width through `Node::channels` is one that reads a fixed channel,
/// which is exactly what a strip channel points at; one that declares none is
/// placed by band or sweeps, and belongs in the scanner table instead. Built
/// once, because building every decode stage to ask it a question is not work
/// to repeat per frame.
pub fn channel_fronts() -> &'static [(&'static str, f64)] {
    static FRONTS: std::sync::OnceLock<Vec<(&'static str, f64)>> = std::sync::OnceLock::new();
    FRONTS.get_or_init(|| {
        let reg = nodes::registry();
        let mut out: Vec<(&'static str, f64)> = reg
            .by_category("decode")
            .filter_map(|d| {
                let node = reg.build(d.name, &Default::default()).ok()?;
                // The narrowest channel it declares: a mode keyed at several
                // spacings still fits in its widest, and the marker on the
                // spectrum should not claim more of the band than it reads.
                let w = node.channels().iter().cloned().fold(f64::INFINITY, f64::min);
                w.is_finite().then_some((d.name, w))
            })
            .collect();
        out.sort_by_key(|(name, _)| *name);
        out
    })
}

/// The channel one of those front ends expects, or None if it is not one.
pub fn front_width(kind: &str) -> Option<f64> {
    channel_fronts().iter().find(|(k, _)| *k == kind).map(|(_, w)| *w)
}

/// What a front end is called on a strip button.
///
/// The registry name in capitals: every one of these is an acronym, and a
/// second table mapping "m17" to "M17" would be a table to keep in step.
pub fn front_label(kind: &str) -> String {
    kind.to_uppercase()
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
        // Mode S is 2 MHz wide and its detector refuses anything slower.
        Front::ModeS => Some(((1_089_000_000.0, 1_091_000_000.0), 2_400_000.0)),
        Front::Ais => {
            let w = nodes::ais_nodes::CHANNEL_WIDTH_HZ;
            Some((
                (dsp::ais::CHANNEL_HZ[0] - w, dsp::ais::CHANNEL_HZ[1] + w),
                // The detector mixes both channels itself and wants room
                // between them, so this stays well above their separation.
                600_000.0,
            ))
        }
        Front::Aprs(hz) => {
            let w = nodes::aprs_nodes::CHANNEL_WIDTH_HZ;
            Some(((hz - w, hz + w), 192_000.0))
        }
        Front::Pocsag(hz) => {
            let w = nodes::pocsag_nodes::CHANNEL_WIDTH_HZ;
            Some(((hz - w, hz + w), 192_000.0))
        }
        Front::M17(hz) => {
            let w = nodes::m17_nodes::CHANNEL_WIDTH_HZ;
            Some(((hz - w, hz + w), 192_000.0))
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



/// Patch stages whose output the packet bus accepts.
const BUS_TAILS: [&str; 11] = [
    "pulse_detect",
    "ask_detect",
    "fsk_detect",
    "bank",
    "source_decode",
    "auto",
    "mode_s",
    "ais",
    "aprs",
    "pocsag",
    "m17",
];

/// Stages that carry speech, and the output port it leaves on.
///
/// The same arrangement as [`BUS_TAILS`], and for the same reason: the patch
/// is a description, written before any node exists to be asked. What each
/// front end does with the port is its own business; this only says which
/// wire to draw.
const VOICE_TAILS: [(&str, usize); 4] = [("m17", 1), ("tetra", 1), ("dmr", 1), ("auto", 1)];

/// Stages that report something a position can be resolved from, so the
/// tracker is worth attaching to the bus.
const TRACK_SOURCES: [&str; 4] = ["mode_s", "ais", "aprs", "auto"];

/// The recorder's ring, which is a stage in the graph but owns an open file
/// and so cannot be built from a description alone.
const RING: &str = "ring";

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
    pub const AUDIO: u64 = Patch::DERIVED_BASE + 9;

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
        zoom.insert(
            "passband_hz".into(),
            pipeline::ParamValue::Float(plan.eff_rate() * 0.45),
        );
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
    p.add_derived(derived::SPECTRUM, "spectrum", spectrum);
    p.connect(head, (derived::SPECTRUM, 0));

    if plan.record {
        p.add_derived(derived::RING, RING, Settings::new());
        p.connect(head, (derived::RING, 0));
    }

    // The raw capture is always in the graph and switched off, because the
    // transmission worth having is the one already on the screen. Adding the
    // stage when somebody asks for it would rebuild the graph first, and a
    // rebuild loses the source the auto node has open, which is exactly the
    // signal they were trying to capture. Switched on it costs a parameter;
    // switched off it costs a memcpy of nothing.
    {
        let mut s = Settings::new();
        s.insert(
            "dir".into(),
            pipeline::ParamValue::Text(plan.capture_dir.display().to_string()),
        );
        s.insert("enabled".into(), pipeline::ParamValue::Bool(false));
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
        let fits =
            |hz: f64, width: f64| (hz - plan.center.as_f64()).abs() <= plan.eff_rate() / 2.0 - width;
        match front {
            Front::ModeS => {
                let id = p.add_derived(derived::at("mode_s", 0, 0), "mode_s", Settings::new());
                p.connect(src, (id, 0));
            }
            Front::Ais => {
                let id = p.add_derived(derived::at("ais", 0, 0), "ais", Settings::new());
                p.connect(src, (id, 0));
            }
            Front::Aprs(hz) if fits(*hz, nodes::aprs_nodes::CHANNEL_WIDTH_HZ) => {
                let mut s = Settings::new();
                s.insert("channel_hz".into(), pipeline::ParamValue::Float(*hz));
                s.insert(
                    "label".into(),
                    pipeline::ParamValue::Text(format!("{:.3} APRS", hz / 1e6)),
                );
                let id = p.add_derived(derived::at("aprs", *hz as u64, 0), "aprs", s);
                p.connect(src, (id, 0));
            }
            Front::Pocsag(hz) if fits(*hz, nodes::pocsag_nodes::CHANNEL_WIDTH_HZ) => {
                let mut s = Settings::new();
                s.insert("channel_hz".into(), pipeline::ParamValue::Float(*hz));
                s.insert(
                    "label".into(),
                    pipeline::ParamValue::Text(format!("{:.4} pager", hz / 1e6)),
                );
                let id = p.add_derived(derived::at("pocsag", *hz as u64, 0), "pocsag", s);
                p.connect(src, (id, 0));
            }
            Front::M17(hz) if fits(*hz, nodes::m17_nodes::CHANNEL_WIDTH_HZ) => {
                let mut s = Settings::new();
                s.insert("channel_hz".into(), pipeline::ParamValue::Float(*hz));
                s.insert(
                    "label".into(),
                    pipeline::ParamValue::Text(format!("{:.4} M17", hz / 1e6)),
                );
                let id = p.add_derived(derived::at("m17", *hz as u64, 0), "m17", s);
                p.connect(src, (id, 0));
            }
            Front::Aprs(_) | Front::Pocsag(_) | Front::M17(_) => {}
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
                    let channels =
                        nodes::BankNode::channels_for(sub.rate(plan.eff_rate()), width);
                    if built.contains(&channels) {
                        continue;
                    }
                    built.push(channels);
                    let mut s = Settings::new();
                    s.insert("channel_hz".into(), pipeline::ParamValue::Float(width));
                    s.insert("band_lo_hz".into(), pipeline::ParamValue::Float(band.0));
                    s.insert("band_hi_hz".into(), pipeline::ParamValue::Float(band.1));
                    let id =
                        p.add_derived(derived::at("bank", sub.key(), width as u64), "bank", s);
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

    // Everything that produces packets meets at the bus, and everything that
    // consumes them hangs off the far side. One input per source: the bus is
    // the only stage whose shape follows the rest of the graph rather than
    // its own settings.
    let sources: Vec<u64> = p
        .stages()
        .iter()
        .filter(|s| puts_packets_on_bus(&s.kind))
        .map(|s| s.id)
        .collect();
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

        // The tracker is a consumer of the bus like any other, which is what
        // stops every view being wired to the demodulator it happens to care
        // about. Attached whenever anything could produce a frame it can
        // resolve a position from: a feed is usually the reason to run one at
        // all on a band that is neither 1090 nor 162.
        let makes_tracks = p
            .stages()
            .iter()
            .any(|s| TRACK_SOURCES.contains(&s.kind.as_str()) || s.kind == "feed");
        if makes_tracks {
            let t = p.add_derived(derived::TRACKS, "tracks", Settings::new());
            p.connect(Source::Stage(bus, 0), (t, 0));
        }
    }

    p
}

/// Whether a stage of this kind puts packets on the bus.
///
/// The table, plus every front end that declares a channel of its own: those
/// are what a strip channel can be set to, and a decoder nobody wired to the
/// bus decodes into silence.
fn puts_packets_on_bus(kind: &str) -> bool {
    BUS_TAILS.contains(&kind) || kind == "feed" || front_width(kind).is_some()
}

/// The stages of one strip channel, in the order they are built. A decode
/// channel uses the first two and then its front end; an audio one uses the
/// rest.
const CHAN_STAGES: [&str; 9] = [
    "chan_mix",
    "chan_ifdec",
    "chan_front",
    "chan_demod",
    "chan_squelch",
    "chan_audiodec",
    "chan_deemph",
    "chan_agc",
    "chan_blend",
];

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
        if spec.offset_hz.abs() > rate / 2.0 || rate < spec.mode.min_rate() {
            continue;
        }
        let tail = channel_stages(p, head, spec, plan.center.as_f64(), rate);
        want.extend(CHAN_STAGES.iter().map(|w| chan_stage_id(w, spec, rate)));
        // Where the strip listens to it. A played channel ends in audio; a
        // decoded one is heard only if its front end has speech to give, and
        // a pager does not.
        let port = match &spec.mode {
            ChanMode::Audio(_) => Some(0),
            ChanMode::Decode(kind) => {
                VOICE_TAILS.iter().find(|(k, _)| k == kind).map(|(_, port)| *port)
            }
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
        if let Some((_, port)) = VOICE_TAILS.iter().find(|(kind, _)| *kind == st.kind) {
            let from = Source::Stage(st.id, *port);
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
    let mut wired: Vec<(usize, Source)> = p
        .links()
        .iter()
        .filter(|l| l.to.0 == bus)
        .map(|l| (l.to.1, l.from))
        .collect();
    wired.sort_by_key(|(k, _)| *k);
    let per_port: Vec<Settings> = wired
        .iter()
        .map(|(k, _)| {
            let mut own = Settings::new();
            for what in ["vol", "mute", "label"] {
                if let Some(v) = s.get(&format!("{what}{k}")) {
                    own.insert(what.into(), v.clone());
                }
            }
            own
        })
        .collect();
    s.retain(|name, _| !["vol", "mute", "label"].iter().any(|w| {
        name.strip_prefix(w).is_some_and(|k| k.parse::<usize>().is_ok())
    }));
    for (k, _) in &wired {
        p.disconnect((bus, *k));
    }
    for (k, ((_, from), own)) in wired.iter().zip(per_port).enumerate() {
        p.connect(*from, (bus, k));
        for (what, v) in own {
            s.insert(format!("{what}{k}"), v);
        }
        match owned.iter().find(|(o, ..)| o == from) {
            // A channel's level is the strip's to say.
            Some((_, Some(spec), label)) => {
                strip_settings(&mut s, k, spec.volume, spec.muted, label);
            }
            // A voice port's level is the subscriptions' business; the strip
            // itself passes it whole.
            Some((_, None, label)) => {
                s.insert(format!("label{k}"), V::Text(label.clone()));
                s.entry(format!("vol{k}")).or_insert(V::Float(1.0));
            }
            // A chain the operator drew, named after what feeds it.
            None => {
                if let Source::Stage(f, _) = from {
                    if let Some(st) = p.stage(*f) {
                        s.entry(format!("label{k}"))
                            .or_insert(V::Text(stage_label(&st.kind, &st.settings)));
                    }
                }
            }
        }
    }
    s.insert("inputs".into(), V::Int(wired.len() as i64 + 1));
    p.add_derived(bus, "audio_bus", s);
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
    s.insert(format!("vol{k}"), V::Float(volume as f64));
    s.insert(format!("mute{k}"), V::Bool(muted));
    s.insert(format!("label{k}"), V::Text(label.to_string()));
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
    }
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
    let width = spec.mode.bandwidth();
    let at = |p: &mut crate::patch::Patch, what: &str, kind: &str, mut s: Settings| -> u64 {
        s.insert("channel".into(), V::Int(spec.id as i64));
        p.add_derived(chan_stage_id(what, spec, rate), kind, s)
    };

    let mut mix = Settings::new();
    mix.insert("shift_hz".into(), V::Float(-hz));
    let m = at(p, "chan_mix", "mixer", mix);
    p.connect(head, (m, 0));

    // The front end mixes and filters its own channel out of what it is
    // handed, so this only has to bring the rate down far enough that it is
    // not doing that at the radio's. Decimating to the channel itself would
    // leave the node no transition band and no room for the tuning error the
    // dial has, so the target is well above it.
    let target = (width * DECODE_RATE_RATIO).max(DECODE_MIN_RATE_HZ);
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
    // is labelled with where it came from.
    let mut s = Settings::new();
    s.insert("channel_hz".into(), V::Float(center + hz));
    s.insert("label".into(), V::Text(spec.label.clone()));
    let f = at(p, "chan_front", kind, s);
    p.connect(Source::Stage(i, 0), (f, 0));
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

    let if_dec = ((rate / mode.if_rate()).round() as usize).max(1);
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
    mix.insert("shift_hz".into(), V::Float(-(spec.offset_hz - mode.cw_pitch())));
    let m = at(p, "chan_mix", "mixer", mix);
    p.connect(head, (m, 0));

    // Sized from the signal's bandwidth, not from the decimation factor: the
    // stopband has to land where the first alias folds down.
    let mut ifd = Settings::new();
    ifd.insert("factor".into(), V::Int(if_dec as i64));
    ifd.insert("passband_hz".into(), V::Float(mode.bandwidth() / 2.0));
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
        let lsb = mode.sideband() == dsp::ssb::Sideband::Lower;
        d.insert("sideband".into(), V::Text(if lsb { "lsb" } else { "usb" }.into()));
        if mode == Demod::Cw {
            d.insert("pitch_hz".into(), V::Float(mode.cw_pitch()));
            d.insert("width_hz".into(), V::Float(CW_FILTER_HZ));
            d.insert("label".into(), V::Text("CW filter".into()));
        } else {
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
    if let Some(db) = spec.squelch_db.or_else(|| mode.default_squelch_db()) {
        let mut s = Settings::new();
        s.insert(
            "kind".into(),
            V::Text(if mode == Demod::Nfm { "noise" } else { "level" }.into()),
        );
        s.insert("threshold_db".into(), V::Float(db as f64));
        let sq = at(p, "chan_squelch", "squelch", s);
        p.connect(tail, (sq, 0));
        tail = Source::Stage(sq, 0);
    }

    let mut ad = Settings::new();
    ad.insert("factor".into(), V::Int(au_dec as i64));
    ad.insert("passband_hz".into(), V::Float(mode.audio_bw()));
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
        Demod::Cw => Some("cw"),
        Demod::Nfm | Demod::Am | Demod::Usb | Demod::Lsb => Some("voice"),
        Demod::Wfm => None,
    };
    if let Some(preset) = preset {
        let mut a = Settings::new();
        a.insert("preset".into(), V::Text(preset.into()));
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
fn chan_stage_id(what: &str, spec: &ChannelSpec, rate: f64) -> u64 {
    use std::hash::{Hash, Hasher};
    let mut h = std::collections::hash_map::DefaultHasher::new();
    spec.mode.key().hash(&mut h);
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
    mix.insert(
        "shift_hz".into(),
        pipeline::ParamValue::Float(plan.center.as_f64() - sub.center),
    );
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
        "pulse_detect" => "OOK pulses".into(),
        "ask_detect" => "ASK pulses".into(),
        "fsk_detect" => "FSK pulses".into(),
        "protocol_decode" => "Protocols".into(),
        "high_blend" => "High blend".into(),
        "protocols" => "Protocols".into(),
        "tracks" => "Tracks".into(),
        "packet_bus" => "Packet log".into(),
        "audio_bus" => "Audio".into(),
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

/// What building a patch produced: the stages that put packets on the bus,
/// where every stage ended up, and which of them kept the node they had.
type Built = (Vec<NodeId>, HashMap<u64, NodeId>, Vec<u64>);

/// Every stage type this receiver can build.
///
/// The node registry plus the ones that only make sense inside the
/// application: the tracker folds positions together for the map, which is a
/// view rather than a signal path, so it lives here.
pub fn registry() -> pipeline::registry::Registry {
    use pipeline::registry::StageDesc;
    let mut r = nodes::registry();
    r.register(
        StageDesc {
            name: "tracks",
            summary: "Fold reported positions into tracks: aircraft, vessels and marks",
            category: "sink",
        },
        |_s| Ok(Box::new(crate::tracks::TracksNode::new()) as Box<dyn pipeline::node::Node>),
    );
    r.register(
        StageDesc {
            name: "audio_bus",
            summary: "Every channel and every voice front end in one place: \
                      what reaches the speaker is what is wired in here, at \
                      the level its strip says",
            category: "audio",
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
    roles: &mut Vec<Role>,
    pool: &mut HashMap<Role, NodePart>,
    span: Out,
    patch: &crate::patch::Patch,
    ring: &mut Option<RecordRing>,
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
        let role = Role::Patch(st.id, st.kind.clone());
        // Reused where it can be, so editing one wire does not reset the
        // detector's noise floor on every other stage in the graph.
        let mut node = match pool.remove(&role) {
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
            None => {
                let mut node = reg.build(&st.kind, &st.settings)?;
                // A decimator's passband is designed rather than set, so it
                // is not something a number alone can carry: the design needs
                // the rate it is being cut from.
                if let (Some(pb), Some(rate)) = (
                    st.settings.get("passband_hz").and_then(|v| v.as_f64()),
                    st.settings.get("input_rate_hz").and_then(|v| v.as_f64()),
                ) {
                    if let Some(d) = node.as_any_mut().and_then(|a| a.downcast_mut::<DecimateNode>())
                    {
                        d.set_passband_hz(rate, pb);
                    }
                }
                node
            }
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
        // The bus is the one stage whose shape follows the graph rather than
        // its own settings: how many inputs it has is how many things feed
        // it, and that changes with every retune. It is carried across
        // rebuilds because it holds the open log file, so it has to be told.
        if st.kind == "packet_bus" {
            if let Some(n) = node.as_any_mut().and_then(|a| a.downcast_mut::<nodes::PacketBusNode>())
            {
                n.set_inputs(st.settings.i64_or("inputs", 1).max(1) as usize);
            }
        }
        // A bank's band decides which of its channels get a decoder, and that
        // is settled while the graph negotiates: it has to be set before the
        // node goes in, on a reused node as much as a fresh one, because the
        // span has usually moved under it since it was last built.
        if st.kind == "bank" {
            let (lo, hi) =
                (st.settings.f64_or("band_lo_hz", 0.0), st.settings.f64_or("band_hi_hz", 0.0));
            if let Some(n) = node.as_any_mut().and_then(|a| a.downcast_mut::<BankNode>()) {
                n.set_band((hi > lo).then_some((lo, hi)));
            }
        }
        // The same for the source detector and the auto node, and for the
        // same reason.
        if st.kind == "source_detect" || st.kind == "auto" {
            let (lo, hi) =
                (st.settings.f64_or("band_lo_hz", 0.0), st.settings.f64_or("band_hi_hz", 0.0));
            let band = (hi > lo).then_some((lo, hi));
            if let Some(n) =
                node.as_any_mut().and_then(|a| a.downcast_mut::<nodes::SourceDetectNode>())
            {
                n.set_band(band);
            }
            if let Some(n) = node.as_any_mut().and_then(|a| a.downcast_mut::<nodes::AutoNode>()) {
                n.set_band(band);
                let spur = st.settings.f64_or("spur_hz", 0.0);
                n.set_spur((spur > 0.0).then_some(spur));
                let step = st.settings.f64_or("raster_hz", 0.0);
                n.set_raster((step > 0.0).then(|| (st.settings.f64_or("raster_origin_hz", 0.0), step)));
            }
        }
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
        // box on screen is the stage that asked for it.
        b.set_tag(nid, id);
        roles.push(Role::Patch(id, kind.clone()));
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
        if BUS_TAILS.contains(&kind.as_str()) && patch.is_tail(id) {
            packets.push(nid);
        }
    }

    // A stage added out of dependency order is fed by one that has not been
    // given its `NodeId` yet, so the wires are made again once every stage
    // has one. Connecting an input twice replaces the earlier edge, which is
    // exactly what is wanted here.
    for st in patch.stages().iter().filter(|s| live.contains(&s.id)) {
        let Some(&nid) = ids.get(&st.id) else { continue };
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
pub fn scan_marks(
    scanners: &crate::scanners::Scanners,
    center: f64,
    rate: f64,
) -> Vec<ScanMark> {
    let mut out = Vec::new();
    for at in scanners.fronts(center, rate) {
        match &at.front {
            Front::ModeS => out.push(ScanMark::Channel {
                hz: 1_090_000_000.0,
                width: MODES_BAND_HZ,
                label: "Mode S".into(),
            }),
            Front::Ais => {
                for (i, hz) in dsp::ais::CHANNEL_HZ.iter().enumerate() {
                    out.push(ScanMark::Channel {
                        hz: *hz,
                        width: nodes::ais_nodes::CHANNEL_WIDTH_HZ,
                        label: format!("AIS {}", if i == 0 { "A" } else { "B" }),
                    });
                }
            }
            Front::Aprs(hz) => out.push(ScanMark::Channel {
                hz: *hz,
                width: nodes::aprs_nodes::CHANNEL_WIDTH_HZ,
                label: "APRS".into(),
            }),
            Front::Pocsag(hz) => out.push(ScanMark::Channel {
                hz: *hz,
                width: nodes::pocsag_nodes::CHANNEL_WIDTH_HZ,
                label: "POCSAG".into(),
            }),
            Front::M17(hz) => out.push(ScanMark::Channel {
                hz: *hz,
                width: nodes::m17_nodes::CHANNEL_WIDTH_HZ,
                label: "M17".into(),
            }),
            Front::Auto => {
                // No grid to draw: the band is watched whole and whatever
                // is in it is found where it is.
                let Some(band) = at.covered(center, rate) else { continue };
                out.push(ScanMark::Band {
                    lo: band.0,
                    hi: band.1,
                    origin: (band.0 + band.1) / 2.0,
                    spacing: 0.0,
                    label: "auto".into(),
                });
            }
            Front::Banks(widths) => {
                let Some(band) = at.covered(center, rate) else { continue };
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

fn record(at: std::time::Instant, d: &pipeline::event::Decoded) -> DecodeRecord {
    // Mode S occupies the whole band it is transmitted in; there is no
    // channel to speak of, and nothing else is near enough to be confused
    // with it. Anything from a bank was heard through one of its channels,
    // and which bank is what its keying says.
    // The width the packet was actually heard through, when the chain that
    // produced it knows. The match below is the fallback for the chains that
    // do not carry one, and it is a guess: it reads the width off the keying,
    // which stops being a proxy for the bank tier as soon as anything measures
    // the keying properly.
    let channel_hz = match d.bandwidth_hz {
        Some(hz) if hz > 0.0 => hz,
        _ => channel_hz_from_keying(d),
    };
    DecodeRecord {
        at,
        freq: d.center.as_f64(),
        channel_hz,
        model: d.protocol.to_string(),
        modulation: d.modulation.unwrap_or("?"),
        detail: d.detail.clone().or_else(|| d.text.clone()).unwrap_or_default(),
        fields: d.fields.clone(),
        media_type: d.media_type,
        rssi_dbfs: d.rssi_dbfs.unwrap_or(f32::NAN),
        snr_db: d.snr_db.unwrap_or(f32::NAN),
        bytes: d.payload.clone(),
        crc: d.crc_ok,
        iq: d.iq.clone(),
        audio: d.audio.clone(),
    }
}

fn channel_hz_from_keying(d: &pipeline::event::Decoded) -> f64 {
    match d.modulation {
        Some("PPM") => MODES_BAND_HZ,
        // AIS is heard through one 25 kHz marine channel, whichever of the
        // two carried the frame.
        Some("GMSK") => nodes::ais_nodes::CHANNEL_WIDTH_HZ,
        // A pager is keyed FSK like an 868 MHz sensor and heard through a
        // channel a tenth the width, so the keying alone does not say which
        // front end produced it.
        Some("FSK") if d.protocol.starts_with("POCSAG") => {
            nodes::pocsag_nodes::CHANNEL_WIDTH_HZ
        }
        Some("4FSK") => nodes::m17_nodes::CHANNEL_WIDTH_HZ,
        Some("FSK") => FSK_CHANNEL_HZ,
        _ => OOK_CHANNEL_HZ,
    }
}

/// Where raw span captures go when nobody says otherwise: beside the packet
/// log, since both are recordings of what was on the air.
pub fn default_capture_dir() -> PathBuf {
    crate::packetlog::PacketLog::default_dir()
        .map(|d| d.with_file_name("captures"))
        .unwrap_or_else(|| std::env::temp_dir().join("waveshark-captures"))
}

fn downcast<T: 'static>(g: &Graph, id: NodeId) -> Option<&T> {
    g.node(id).and_then(|n| n.as_any()).and_then(|a| a.downcast_ref::<T>())
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
        let any = node.into_any()?;
        any.downcast::<nodes::RingNode<Self>>().ok().map(|n| n.into_ring())
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
            fft: 1024,
            channels: Vec::new(),
            audio: AudioPlan::default(),
            fronts: vec![crate::scanners::FrontAt {
                front: Front::Banks(crate::scanners::DEFAULT_WIDTHS.to_vec()),
                band: (0.0, f64::INFINITY),
            }],
            record: false,
            capture_dir: crate::chain::default_capture_dir(),
            capture_format: common::SampleFormat::Cu8,
            log: false,
            feeds: Vec::new(),
        }
    }

    /// A plan with nothing on the span but what the operator drew: the
    /// drawing is read against the graph the receiver would draw, and the
    /// difference is what runs on top of it.
    fn manual(patch: crate::patch::Patch) -> Plan {
        let mut p = plan(2_400_000.0, Hz::mhz(433));
        p.fronts.clear();
        p.edits = crate::patch::Edits::diff(&patch, &derived_patch(&p));
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
        let spectrum =
            topo.nodes.iter().find(|n| n.tag == Some(view)).expect("a spectrum");
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
        assert_eq!(seen[0].0, view);
        // Its own band, not the span's: a strip drawn from the dial's rate
        // would put every signal in it at eight times the offset.
        assert_eq!(seen[0].3, plan.eff_rate() / 8.0);
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
            volume: 1.0,
            muted: false,
            squelch_db: None,
            agc: true,
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
        let labels: Vec<String> =
            rx.topology().nodes.iter().map(|n| n.label.clone()).collect();
        for want in ["DC block", "Spectrum", "31 kHz bank", "125 kHz bank", "Mixer"] {
            assert!(labels.iter().any(|l| l == want), "{want} is not in {labels:?}");
        }
    }

    #[test]
    fn a_bank_shows_the_chain_its_channels_run() {
        let rx = Receiver::build(&plan(2_400_000.0, Hz::mhz(433)), Sinks::default()).unwrap();
        let topo = rx.topology();
        let bank = topo.nodes.iter().find(|n| n.label == "31 kHz bank").expect("the 31 kHz bank");
        let inner = bank.inner.as_ref().expect("what a channel runs");
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

    #[test]
    fn changing_what_a_channel_listens_to_rebuilds_it() {
        // Its mixer shift and every filter design follow from the offset and
        // the mode, so reusing those nodes would leave a chain built for a
        // frequency it is no longer on.
        let mut p = plan(2_400_000.0, Hz::mhz(95));
        p.channels = vec![chan(1, 100_000.0, Demod::Wfm)];
        let mut rx = Receiver::build(&p, Sinks::default()).unwrap();
        rx.process(&block(4096)).unwrap();

        p.channels = vec![chan(1, 300_000.0, Demod::Wfm)];
        rx.rebuild(&p).unwrap();
        assert!(!rx.channels()[0].kept);
    }

    #[test]
    fn the_speech_path_is_on_the_graph_like_everything_else() {
        // The call bus used to be a struct in the radio thread fed by hand,
        // so the drawing of the receiver said nothing about where the audio
        // went. Every front end that carries voice publishes it on a port,
        // and the bus is the node on the end of them.
        let mut p = plan(2_400_000.0, Hz::mhz(433));
        p.fronts = vec![crate::scanners::FrontAt {
            front: Front::M17(433_475_000.0),
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
        let chan_stages = rx
            .patch()
            .stages()
            .iter()
            .filter(|s| s.settings.contains_key("channel"))
            .count();
        assert_eq!(chan_stages, 8, "an NFM chain is eight stages, and no more were kept");
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
        patch.stage_mut(mix).unwrap().settings.insert(
            "shift_hz".into(),
            pipeline::ParamValue::Float(-200_000.0),
        );
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
        p.edits = crate::patch::Edits::diff(&patch, &derived_patch(&p));
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
        patch.stage_mut(dec).unwrap().settings.insert("factor".into(), pipeline::ParamValue::Int(4));
        patch.connect(Source::Span, (dec, 0));
        patch.connect(Source::Stage(dec, 0), (derived::SPECTRUM, 0));
        p.edits = crate::patch::Edits::diff(&patch, rx.base());
        assert_eq!(p.edits.stages.len(), 1);
        assert_eq!(p.edits.links.len(), 2, "{:?}", p.edits.links);
        rx.rebuild(&p).unwrap();
        assert_eq!(rx.spectrum_rate(), p.eff_rate() / 4.0, "the edit took");

        // The dial moves to a band with different front ends. The edit is
        // still there and the front ends are the new band's.
        p.center = Hz::mhz(1090);
        p.fronts = vec![crate::scanners::FrontAt {
            front: Front::ModeS,
            band: (0.0, f64::INFINITY),
        }];
        rx.rebuild(&p).unwrap();
        assert_eq!(rx.spectrum_rate(), p.eff_rate() / 4.0, "the edit was lost on retune");
        assert!(rx.bank_channels().is_empty(), "the old band's banks came along");
        assert!(rx.modes_on(), "the new band's front end was not built");
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
        assert!(rx.refused.unwrap().contains("outside the span"));
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
        let rx = Receiver::build(
            &p,
            Sinks {
                packet_log: Some(d.clone()),
                ..Default::default()
            },
        )
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
        // The whole shape of it: sources feed the log, consumers hang off the
        // far side. A view wired straight to a demodulator would have to be
        // rebuilt for every new source, and would see nothing when the source
        // it knew about was not running.
        let mut p = plan(2_400_000.0, Hz::mhz(1090));
        p.fronts = vec![anywhere(Front::ModeS)];
        let rx = Receiver::build(&p, Sinks::default()).unwrap();
        let topo = rx.topology();
        let bus = topo.nodes.iter().find(|n| n.label == "Packet log").expect("a bus");
        let tracker = topo.nodes.iter().find(|n| n.label == "Tracks").expect("a tracker");
        let from_bus = bus.outputs.iter().any(|(slot, _)| {
            tracker.inputs.iter().any(|(in_slot, _)| in_slot == slot)
        });
        assert!(from_bus, "the flight list is not fed by the bus");
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
        p.fronts = vec![anywhere(Front::Ais)];
        let rx = Receiver::build(&p, Sinks::default()).unwrap();
        assert!(rx.ais_on(), "the AIS decoder is not running");
        let topo = rx.topology();
        let ais = topo.nodes.iter().find(|n| n.label == "162 AIS").expect("an AIS node");
        let bus = topo.nodes.iter().find(|n| n.label == "Packet log").expect("a bus");
        let tracker = topo.nodes.iter().find(|n| n.label == "Tracks").expect("a tracker");
        let to_bus = ais
            .outputs
            .iter()
            .any(|(slot, _)| bus.inputs.iter().any(|(in_slot, _)| in_slot == slot));
        assert!(to_bus, "AIS does not reach the bus");
        let from_bus = bus
            .outputs
            .iter()
            .any(|(slot, _)| tracker.inputs.iter().any(|(in_slot, _)| in_slot == slot));
        assert!(from_bus, "the tracker is not fed by the bus");
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
        p.fronts = vec![anywhere(Front::Aprs(144_800_000.0)), anywhere(Front::Pocsag(153_350_000.0))];
        // The pager channel is nine megahertz away, well outside this span,
        // so it is dropped rather than built into a node that would refuse
        // its own input and take the graph down.
        let rx = Receiver::build(&p, Sinks::default()).unwrap();
        assert!(rx.aprs_on());
        assert!(!rx.pocsag_on(), "a channel outside the span must not be built");
        assert!(rx.refused.is_some(), "and the interface has to be told why");

        // Both inside the span now.
        let mut p = plan(2_400_000.0, Hz(144_400_000));
        p.fronts = vec![anywhere(Front::Aprs(144_800_000.0)), anywhere(Front::Pocsag(145_000_000.0))];
        let rx = Receiver::build(&p, Sinks::default()).unwrap();
        assert!(rx.aprs_on() && rx.pocsag_on(), "both front ends should run");
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
        p.fronts = vec![anywhere(Front::Ais)];
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
        p.fronts = vec![anywhere(Front::ModeS)];
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
        p.fronts = vec![anywhere(Front::ModeS)];
        let rx = Receiver::build(&p, Sinks::default()).unwrap();
        assert!(rx.topology().nodes.iter().any(|n| n.label == "Packet log"));
        assert!(rx.topology().nodes.iter().any(|n| n.label == "Tracks"));
        assert_eq!(rx.logged(), 0, "nothing was asked to be written");
    }

    /// A bank over a scanner's own band, at the width that band asked for.
    fn ism_at(center_mhz: f64, rate: f64) -> Plan {
        let mut p = plan(rate, Hz(( center_mhz * 1e6) as u64));
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
        let ScanMark::Band { lo, hi, spacing, origin, .. } = &marks[0] else {
            panic!("{marks:?}")
        };
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
        let mut s = crate::scanners::Scanners { list: Vec::new() };
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
        let mut rx = Receiver::build(
            &p,
            Sinks {
                packet_log: Some(d),
                ..Default::default()
            },
        )
        .unwrap();
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
        let rx =
            Receiver::build(&p, Sinks { recorder: Some(rec), ..Default::default() }).unwrap();
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
        assert!(marks.iter().any(|m| matches!(m, ScanMark::Channel { hz, .. } if (*hz - 439_987_500.0).abs() < 1.0)), "{marks:?}");
    }
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
        p.fronts = vec![anywhere(Front::Pocsag(439_987_500.0))];
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
        p.fronts = vec![anywhere(Front::ModeS)];
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
        p.fronts = vec![anywhere(Front::Pocsag(439_987_500.0))];
        let rx = Receiver::build(&p, Sinks::default()).unwrap();
        let topo = rx.topology();
        let pager = topo.nodes.iter().find(|n| n.kind == "pocsag").expect("a pager node");
        assert!(pager.inputs[0].1.rate <= 400_000.0, "{} S/s", pager.inputs[0].1.rate);
    }

    #[test]
    fn a_band_that_is_already_the_span_adds_no_nodes() {
        // A mixer that shifts by nothing and a decimator that divides by one
        // are two passes over every sample to achieve nothing.
        let mut p = plan(2_400_000.0, Hz::mhz(1090));
        p.fronts = vec![anywhere(Front::ModeS)];
        let labels = topo_labels(&p);
        assert!(!labels.iter().any(|l| l.contains("mixer")), "{labels:?}");
    }

    #[test]
    fn two_front_ends_in_one_band_share_one_extraction() {
        let mut p = plan(20_000_000.0, Hz::mhz(145));
        p.fronts = vec![anywhere(Front::Aprs(144_800_000.0)), anywhere(Front::Aprs(144_800_000.0))];
        let labels = topo_labels(&p);
        let mixers = labels.iter().filter(|l| l.contains("mixer")).count();
        assert_eq!(mixers, 1, "{labels:?}");
    }
}

