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

use common::{Hz, Result, C32};
use dsp::rds::Station;
use nodes::{
    AgcNode, BankNode, DeemphasisNode, DecimateNode, EnvelopeNode, FmDemodNode, HighBlendNode,
    MixerNode, ModeSNode, RealDecimateNode, SpectrumNode, SquelchNode, SsbDemodNode, WfmDemodNode,
};
use pipeline::graph::{NodePart, Topology};
use pipeline::{Graph, GraphBuilder, NodeId, Out, StreamSpec};

use crate::radio::{ChannelSpec, DecodeRecord, Demod};
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

/// The narrow CW filter, in Hz.
const CW_FILTER_HZ: f64 = 500.0;

/// What a node in the graph is for, so a rebuild can find it again.
///
/// Keyed by purpose rather than by position: the whole point is that a node
/// keeps its state when the graph around it changes, and its position is
/// exactly what changed.
#[derive(Clone, PartialEq, Eq, Hash, Debug)]
enum Role {
    DcBlock,
    /// The software zoom decimator, keyed by its factor: a different factor is
    /// a different filter.
    Zoom(usize),
    Spectrum,
    Record,
    ModeS,
    Ais,
    /// Banks are distinguished by the channel width they were built for.
    Bank(u32),
    /// A stage of one listening channel.
    Stage(u64, Stage),
    PacketBus,
    PacketDecode,
    /// A packet feed from another receiver, keyed by where it comes from so a
    /// rebuild keeps the connection open rather than reconnecting.
    Feed(String),
    Flights,
}

#[derive(Clone, Copy, PartialEq, Eq, Hash, Debug)]
enum Stage {
    Mixer,
    IfDecimate,
    Demod,
    Squelch,
    AudioDecimate,
    Deemphasis,
    Agc,
    Blend,
}

/// What a channel branch was built for. A branch is only reused while all of
/// this is unchanged, since every one of these decides a filter's
/// coefficients or a mixer's shift.
#[derive(Clone, Copy, PartialEq, Debug)]
struct ChanKey {
    demod: Demod,
    offset_bits: u64,
    rate_bits: u64,
}

impl ChanKey {
    fn new(spec: &ChannelSpec, rate: f64) -> Self {
        Self {
            demod: spec.demod,
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

impl Chan {
    pub fn is_stereo(&self) -> bool {
        self.channels == 2
    }
}

/// One bank sweeping the span.
pub struct Bank {
    pub channels: usize,
}

pub struct Receiver {
    graph: Graph,
    /// What each node is, indexed by `NodeId`, so a rebuild can hand the same
    /// nodes to the new graph.
    roles: Vec<Role>,
    dc: NodeId,
    /// Last node of the head chain, whose output every branch reads.
    head: NodeId,
    spectrum: NodeId,
    record: Option<NodeId>,
    modes: Option<NodeId>,
    ais: Option<NodeId>,
    banks: Vec<Bank>,
    chans: Vec<Chan>,
    /// A recorder waiting for the next rebuild to become a node.
    pending_record: Option<RecordRing>,
    /// Where the packet log is written, if it is. Held as a directory rather
    /// than an open file so that a rebuild has something to reopen when the
    /// bus itself had to be built again.
    log_dir: Option<PathBuf>,
    /// Size at which a day's file stops growing, or `None` for no limit.
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
    /// Whether to sweep the span with the ISM banks.
    pub scan: bool,
    /// Whether to run the Mode S decoder.
    pub modes: bool,
    /// Whether to run the AIS decoder, which needs both 162 MHz channels in
    /// the span.
    pub ais: bool,
    pub record: bool,
    /// Log every burst the front ends detect.
    pub log: bool,
    /// Other receivers feeding the same packet bus.
    pub feeds: Vec<nodes::FeedSpec>,
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
            dc: NodeId(0),
            head: NodeId(0),
            spectrum: NodeId(0),
            record: None,
            modes: None,
            ais: None,
            banks: Vec::new(),
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
            refused: None,
        };
        rx.assemble(plan, HashMap::new(), sinks.recorder.map(RecordRing::new))?;
        Ok(rx)
    }

    /// Change what the receiver is doing, keeping every node that still means
    /// the same thing.
    pub fn rebuild(&mut self, plan: &Plan) -> Result<()> {
        let old_keys: HashMap<u64, ChanKey> =
            self.chans.iter().map(|c| (c.spec.id, c.key)).collect();
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
        // the same channel; drop its nodes rather than reuse coefficients
        // computed for something else.
        let retuned = plan.center != old_center || plan.rate != old_rate;
        pool.retain(|role, _| match role {
            Role::Stage(id, _) => {
                let want = plan.channels.iter().find(|c| c.id == *id);
                match (want, old_keys.get(id)) {
                    (Some(w), Some(k)) => ChanKey::new(w, plan.eff_rate()) == *k,
                    _ => false,
                }
            }
            // A bank rebuilds itself internally on a retune and keeps its
            // chains, which is cheaper than building several hundred graphs.
            Role::Bank(_) => true,
            // The spectrum's FFT size can change, and the node cannot resize.
            Role::Spectrum => !retuned && self.fft_size() == plan.fft,
            _ => true,
        });

        // The recorder is a file being written; it survives every rebuild
        // short of being switched off, and a newly started one is waiting
        // here for its place in the graph.
        let ring = self.pending_record.take().or_else(|| {
            self.record
                .and_then(|_| pool.remove(&Role::Record))
                .and_then(|p| RecordRing::from_part(p.node))
        });

        self.record = None;
        self.modes = None;
        self.ais = None;
        self.banks.clear();
        self.center = plan.center;
        self.rate = plan.rate;
        self.assemble(plan, pool, ring)
    }

    fn fft_size(&self) -> usize {
        self.graph
            .node(self.spectrum)
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

        // The head of the chain: what every branch downstream agrees the
        // samples are. Both stages belong here rather than in the caller,
        // because a branch that saw the spur or the full rate would disagree
        // with the others about what arrived.
        let dc = match pool.remove(&Role::DcBlock) {
            Some(p) => b.add_existing(p),
            None => b.add_labeled("DC block", Box::new(nodes::DcBlockNode::new())),
        };
        b.source(dc.i());
        roles.push(Role::DcBlock);

        let mut head = dc;
        if plan.zoom > 1 {
            let role = Role::Zoom(plan.zoom);
            let id = match pool.remove(&role) {
                Some(p) => b.add_existing(p),
                None => {
                    // Passband just inside the new Nyquist: the whole point is
                    // that what is left is clean, since anything folded in
                    // cannot be told from a signal afterwards.
                    let mut d = DecimateNode::new(plan.zoom);
                    d.set_passband_hz(plan.rate, plan.eff_rate() * 0.45);
                    b.add_labeled(format!("Zoom /{}", plan.zoom), Box::new(d))
                }
            };
            b.connect(head.o(), id.i());
            roles.push(role);
            head = id;
        }

        // Added before the decoders so it runs before them. Nodes are executed
        // in the order they were added once their inputs are ready, and the
        // ring has to hold a burst before whatever decodes it says so: a
        // recording that starts when a decoder finishes has already missed the
        // packet.
        let mut record = None;
        if plan.record {
            if let Some(r) = ring {
                let id = b.add_labeled("Recorder", Box::new(nodes::RingNode::new(r)));
                b.connect(head.o(), id.i());
                roles.push(Role::Record);
                record = Some(id);
            }
        }

        let spectrum = match pool.remove(&Role::Spectrum) {
            Some(p) => b.add_existing(p),
            None => b.add_labeled("Spectrum", Box::new(SpectrumNode::new(plan.fft))),
        };
        b.connect(head.o(), spectrum.i());
        roles.push(Role::Spectrum);

        let mut modes = None;
        if plan.modes {
            let id = match pool.remove(&Role::ModeS) {
                Some(p) => b.add_existing(p),
                None => b.add_labeled("1090 Mode S", Box::new(ModeSNode::default())),
            };
            b.connect(head.o(), id.i());
            roles.push(Role::ModeS);
            modes = Some(id);
        }

        // AIS is the same shape: a wideband decoder on the head of the chain
        // that puts frames on the bus. It costs a pass over every sample, so
        // like Mode S it only runs where its signal is.
        let mut ais = None;
        if plan.ais {
            let id = match pool.remove(&Role::Ais) {
                Some(p) => b.add_existing(p),
                None => b.add_labeled("162 AIS", Box::new(nodes::AisNode::default())),
            };
            b.connect(head.o(), id.i());
            roles.push(Role::Ais);
            ais = Some(id);
        }

        let mut banks = Vec::new();
        if plan.scan {
            for (label, width, make) in [
                ("OOK bank", OOK_CHANNEL_HZ, nodes::ism_ook_graph as fn(_) -> _),
                ("FSK bank", FSK_CHANNEL_HZ, nodes::ism_fsk_graph as fn(_) -> _),
            ] {
                let role = Role::Bank(width as u32);
                let id = match pool.remove(&role) {
                    Some(p) => b.add_existing(p),
                    None => b.add_labeled(label, Box::new(BankNode::new(label, width, make))),
                };
                b.connect(head.o(), id.i());
                roles.push(role);
                banks.push((id, width));
            }
        }

        let mut chans: Vec<Chan> = Vec::new();
        let mut refused = None;
        for spec in &plan.channels {
            // A channel the span no longer covers cannot be demodulated: the
            // mixer would shift a frequency the radio never sampled down to
            // baseband, and the chain would produce noise that sounds like a
            // dead station rather than silence.
            if spec.offset_hz.abs() > plan.eff_rate() / 2.0 {
                refused = Some(format!(
                    "{:.4} MHz is outside the span",
                    (plan.center.as_f64() + spec.offset_hz) / 1e6,
                ));
                continue;
            }
            if plan.eff_rate() < spec.demod.if_rate() {
                refused = Some(format!(
                    "{} needs a span of at least {:.0} kHz; this one is {:.0} kHz",
                    spec.demod.label(),
                    spec.demod.if_rate() / 1e3,
                    plan.eff_rate() / 1e3,
                ));
                continue;
            }
            chans.push(add_channel(&mut b, &mut roles, &mut pool, head, spec, plan.eff_rate()));
        }

        // Everything that produces packets meets here, and everything that
        // consumes them hangs off the far side. One input per source, added
        // after the sources so it runs once they have all had their say.
        // A feed from another receiver is a front end like any other: it
        // produces packets, so it belongs upstream of the bus rather than
        // beside it. Added before the bus so its output exists to connect.
        let mut feeds = Vec::new();
        for spec in &plan.feeds {
            let role = Role::Feed(spec.address());
            let id = match pool.remove(&role) {
                Some(part) => b.add_existing(part),
                None => b.add_labeled(
                    format!("{} {}", spec.kind.name, spec.address()),
                    Box::new(nodes::FeedNode::new(spec.clone())),
                ),
            };
            roles.push(role);
            feeds.push(id);
        }

        let mut bus = None;
        let sources: Vec<Out> = banks
            .iter()
            .map(|(id, _)| id.o())
            .chain(modes.map(|m| m.o()))
            .chain(ais.map(|a| a.o()))
            .chain(feeds.iter().map(|f| f.o()))
            .collect();
        if !sources.is_empty() {
            let id = match pool.remove(&Role::PacketBus) {
                Some(mut part) => {
                    // The set of front ends changes with the tuning, and the
                    // bus is carried over rather than rebuilt so that it
                    // keeps the file it is writing.
                    if let Some(n) = part
                        .node
                        .as_any_mut()
                        .and_then(|a| a.downcast_mut::<nodes::PacketBusNode>())
                    {
                        n.set_inputs(sources.len());
                    }
                    b.add_existing(part)
                }
                None => b.add_labeled(
                    "Packet log",
                    Box::new(nodes::PacketBusNode::new(sources.len())),
                ),
            };
            for (k, src) in sources.iter().enumerate() {
                b.connect(*src, id.input(k));
            }
            roles.push(Role::PacketBus);
            bus = Some(id);
        }

        // The protocols run here, once, over everything on the bus. They used
        // to run inside every channel of every bank, which meant a hundred
        // copies of the same tables, decodes that reached the rest of the
        // program through whatever happened to collect them, and no decoding
        // at all for a packet that arrived by any other route.
        let mut decode = None;
        if let Some(bus) = bus {
            let id = match pool.remove(&Role::PacketDecode) {
                Some(p) => b.add_existing(p),
                None => b.add_labeled("Protocols", Box::new(nodes::PacketDecodeNode::default())),
            };
            b.connect(bus.o(), id.i());
            roles.push(Role::PacketDecode);
            decode = Some(id);
        }

        // The flight tracker is a consumer of the bus like any other, which
        // is what stops every view being wired to the demodulator it happens
        // to care about.
        // Attached whenever anything could produce a frame it can track: the
        // Mode S or AIS demodulators, or a feed from a receiver that has one.
        // A feed is usually the reason to run this at all on a band that is
        // neither 1090 nor 162.
        let mut tracks = None;
        if let (Some(bus), true) = (bus, plan.modes || plan.ais || !plan.feeds.is_empty()) {
            let id = match pool.remove(&Role::Flights) {
                Some(p) => b.add_existing(p),
                None => b.add_labeled("Tracks", Box::new(crate::tracks::TracksNode::new())),
            };
            b.connect(bus.o(), id.i());
            roles.push(Role::Flights);
            tracks = Some(id);
        }

        // Some output has to be nominated and none of them is the output: a
        // receiver has as many as it has channels, and the spectrum and the
        // decoders produce nothing that flows. The last channel is as good as
        // any; readers ask for the port they want by name.
        if let Some(c) = chans.last() {
            b.output(c.tail);
        }

        let mut graph = b.build()?;
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
        if let Some(n) = graph
            .node_mut(dc)
            .and_then(|n| n.as_any_mut())
            .and_then(|a| a.downcast_mut::<nodes::DcBlockNode>())
        {
            n.set_enabled(plan.dc_block);
        }
        if let Some(n) = graph
            .node_mut(spectrum)
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
        self.roles = roles;
        self.spectrum = spectrum;
        self.record = record;
        self.bus = bus;
        self.decode = decode;
        self.ais = ais;
        self.tracks = tracks;
        // A tracker built fresh has to be told where the receiver is, which
        // is what resolves a position from a single frame.
        if let Some((lat, lon)) = self.location {
            self.set_location(lat, lon);
        }
        self.modes = modes;
        self.banks = banks
            .into_iter()
            .map(|(id, _width)| {
                let channels = self
                    .graph
                    .node(id)
                    .and_then(|n| n.as_any())
                    .and_then(|a| a.downcast_ref::<BankNode>())
                    .map(|b| b.channels())
                    .unwrap_or(0);
                Bank { channels }
            })
            .collect();
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

    /// Audio from one channel, as the graph left it.
    pub fn audio(&self, i: usize) -> &[f32] {
        self.chans
            .get(i)
            .and_then(|c| self.graph.buf(c.tail))
            .and_then(|p| p.as_real())
            .unwrap_or(&[])
    }

    /// Whether the spectrum completed a frame this block.
    pub fn spectrum_ready(&self) -> bool {
        downcast::<SpectrumNode>(&self.graph, self.spectrum).map(|s| s.is_fresh()).unwrap_or(false)
    }

    pub fn power_db(&mut self) -> &[f32] {
        self.graph
            .node_mut(self.spectrum)
            .and_then(|n| n.as_any_mut())
            .and_then(|a| a.downcast_mut::<SpectrumNode>())
            .map(|s| s.power_db())
            .unwrap_or(&[])
    }

    pub fn modes_on(&self) -> bool {
        self.modes.is_some()
    }

    pub fn ais_on(&self) -> bool {
        self.ais.is_some()
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

    /// Delay to a channel's audio, in milliseconds.
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
        self.graph
            .node_mut(self.spectrum)
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
        self.graph
            .node_mut(self.dc)
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
        self.roles
            .iter()
            .enumerate()
            .filter(|(_, r)| matches!(r, Role::Feed(_)))
            .filter_map(|(k, _)| downcast::<nodes::FeedNode>(&self.graph, NodeId(k)))
            .map(|n| FeedStatus {
                spec: n.spec().clone(),
                connected: n.connected(),
                frames: n.frames(),
                error: n.error(),
            })
            .collect()
    }

    /// Size at which a day's file stops growing. Changing it takes effect on
    /// the file being written, so raising it restarts a log that stopped.
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
        self.graph.buf(self.head.o()).and_then(|p| p.as_iq()).unwrap_or(&[])
    }

}

/// Bandwidth a Mode S transmission occupies, for the log's channel column.
const MODES_BAND_HZ: f64 = 2_000_000.0;

fn record(at: std::time::Instant, d: &pipeline::event::Decoded) -> DecodeRecord {
    // Mode S occupies the whole band it is transmitted in; there is no
    // channel to speak of, and nothing else is near enough to be confused
    // with it. Anything from a bank was heard through one of its channels,
    // and which bank is what its keying says.
    let channel_hz = match d.modulation {
        Some("PPM") => MODES_BAND_HZ,
        // AIS is heard through one 25 kHz marine channel, whichever of the
        // two carried the frame.
        Some("GMSK") => nodes::ais_nodes::CHANNEL_WIDTH_HZ,
        Some("FSK") => FSK_CHANNEL_HZ,
        _ => OOK_CHANNEL_HZ,
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
    }
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

/// Build one listening channel's branch onto the graph.
///
/// The same construction the receiver has always used, lifted out so the one
/// graph and the single-channel test harness cannot drift apart.
fn add_channel(
    b: &mut GraphBuilder,
    roles: &mut Vec<Role>,
    pool: &mut HashMap<Role, NodePart>,
    head: NodeId,
    spec: &ChannelSpec,
    rate: f64,
) -> Chan {
    let mode = spec.demod;
    let id = spec.id;
    let if_dec = ((rate / mode.if_rate()).round() as usize).max(1);
    let if_rate = rate / if_dec as f64;
    let au_dec = ((if_rate / AUDIO_HZ).round() as usize).max(1);

    // Set false by the first stage that has to be built rather than reused.
    let mut kept = true;
    let mut take = |b: &mut GraphBuilder,
                    roles: &mut Vec<Role>,
                    stage: Stage,
                    label: &str,
                    make: Box<dyn pipeline::node::Node>| {
        let role = Role::Stage(id, stage);
        let nid = match pool.remove(&role) {
            Some(p) => b.add_existing(p),
            None => {
                kept = false;
                b.add_labeled(label, make)
            }
        };
        roles.push(role);
        nid
    };

    // CW is tuned low by the pitch so the dial reads the carrier rather than
    // the note; every other mode is tuned to what it listens to.
    let mix = take(
        b,
        roles,
        Stage::Mixer,
        "Mixer",
        Box::new(MixerNode::new(-(spec.offset_hz - mode.cw_pitch()))),
    );
    // Sized from the signal's bandwidth, not from the decimation factor: the
    // stopband has to land where the first alias folds down.
    let mut dec = DecimateNode::new(if_dec);
    dec.set_passband_hz(rate, mode.bandwidth() / 2.0);
    let ifd = take(b, roles, Stage::IfDecimate, "IF decimator", Box::new(dec));
    b.connect(head.o(), mix.i());
    b.connect(mix.o(), ifd.i());

    let stereo = mode == Demod::Wfm && if_rate >= 130_000.0;
    let mut wfm = None;
    let demod = if stereo {
        let d = take(b, roles, Stage::Demod, "WFM demod", Box::new(WfmDemodNode::new()));
        wfm = Some(d);
        d
    } else if mode == Demod::Am {
        take(b, roles, Stage::Demod, "AM envelope", Box::new(EnvelopeNode))
    } else if mode.is_ssb() {
        let node = if mode == Demod::Cw {
            SsbDemodNode::cw(mode.sideband(), mode.cw_pitch(), CW_FILTER_HZ)
        } else {
            SsbDemodNode::voice(mode.sideband())
        };
        let label = if mode == Demod::Cw { "CW filter" } else { "Sideband filter" };
        take(b, roles, Stage::Demod, label, Box::new(node))
    } else {
        take(
            b,
            roles,
            Stage::Demod,
            "FM discriminator",
            Box::new(FmDemodNode::new(mode.deviation())),
        )
    };
    b.connect(ifd.o(), demod.i());

    // The squelch goes here, on the demodulator's raw output, and not later
    // where the audio is. An FM noise squelch works by measuring the hiss
    // above the speech band, and the audio filter's whole job is to remove
    // that: measured on an empty 2 m channel, a squelch after the filter saw a
    // clean signal and held itself open on pure noise.
    let mut tail = demod;
    let mut squelch = None;
    if let Some(sq) = crate::radio::squelch_for(mode) {
        let s = take(b, roles, Stage::Squelch, "Squelch", Box::new(sq));
        b.connect(tail.o(), s.i());
        squelch = Some(s);
        tail = s;
    }

    let mut ad = RealDecimateNode::new(au_dec);
    ad.set_passband_hz(if_rate, mode.audio_bw());
    let aud = take(b, roles, Stage::AudioDecimate, "Audio decimator", Box::new(ad));
    b.connect(tail.o(), aud.i());
    tail = aud;

    if !(mode == Demod::Am || mode.is_ssb()) {
        // De-emphasis is an FM thing: it undoes the pre-emphasis the
        // transmitter applied. Applying it to AM or SSB would just be a treble
        // cut nobody asked for.
        let de =
            take(b, roles, Stage::Deemphasis, "De-emphasis", Box::new(DeemphasisNode::new(50.0)));
        b.connect(tail.o(), de.i());
        tail = de;
    }

    // The gain control comes after the squelch, so what it sees is either a
    // signal or silence. The other order lets the AGC lift the noise on a dead
    // channel up to the threshold and hold the squelch open.
    let mut agc = None;
    if let Some(node) = crate::radio::agc_for(mode) {
        let a = take(b, roles, Stage::Agc, "AGC", Box::new(node));
        b.connect(tail.o(), a.i());
        agc = Some(a);
        tail = a;
    }

    let hb = take(b, roles, Stage::Blend, "High blend", Box::new(HighBlendNode::new()));
    b.connect(tail.o(), hb.i());

    Chan {
        spec: spec.clone(),
        kept,
        key: ChanKey::new(spec, rate),
        tail: hb.o(),
        agc,
        squelch,
        wfm,
        audio_rate: AUDIO_HZ,
        channels: if stereo { 2 } else { 1 },
        detail: format!(
            "if /{if_dec} to {:.0} kHz, audio /{au_dec}{}",
            if_rate / 1e3,
            if stereo { ", stereo" } else { "" }
        ),
        agc_gain_db: 0.0,
        squelch_open: true,
        squelch_db: 0.0,
        blend: 0.0,
        station: Station::default(),
        rds_stats: (0, 0, false),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn plan(rate: f64, center: Hz) -> Plan {
        Plan {
            center,
            rate,
            zoom: 1,
            dc_block: true,
            refresh_hz: 30.0,
            fft: 1024,
            channels: Vec::new(),
            scan: true,
            modes: false,
            ais: false,
            record: false,
            log: false,
            feeds: Vec::new(),
        }
    }

    fn chan(id: u64, offset: f64, demod: Demod) -> ChannelSpec {
        ChannelSpec {
            id,
            offset_hz: offset,
            demod,
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
        for want in ["DC block", "Spectrum", "OOK bank", "FSK bank", "Mixer"] {
            assert!(labels.iter().any(|l| l == want), "{want} is not in {labels:?}");
        }
    }

    #[test]
    fn a_bank_shows_the_chain_its_channels_run() {
        let rx = Receiver::build(&plan(2_400_000.0, Hz::mhz(433)), Sinks::default()).unwrap();
        let topo = rx.topology();
        let bank = topo.nodes.iter().find(|n| n.label == "OOK bank").expect("the OOK bank");
        let inner = bank.inner.as_ref().expect("what a channel runs");
        assert!(inner.nodes.iter().any(|n| n.label.contains("Envelope")));
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
        let bank = topo.nodes.iter().find(|n| n.label == "OOK bank").unwrap();
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
        assert_eq!(bus.inputs.len(), 2, "both banks feed it");
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
        p.modes = true;
        p.scan = false;
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
        p.ais = true;
        p.scan = false;
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

    /// The banks understand nothing on 162 MHz, so they must not run there:
    /// it would be a pass over every sample to invent unknown bursts out of
    /// GMSK.
    #[test]
    fn the_ism_banks_do_not_run_on_the_ais_band() {
        let mut p = plan(2_400_000.0, Hz(162_000_000));
        p.ais = true;
        p.scan = false;
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
        p.scan = false;
        p.modes = true;
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
        p.modes = true;
        let rx = Receiver::build(&p, Sinks::default()).unwrap();
        assert!(rx.topology().nodes.iter().any(|n| n.label == "Packet log"));
        assert!(rx.topology().nodes.iter().any(|n| n.label == "Tracks"));
        assert_eq!(rx.logged(), 0, "nothing was asked to be written");
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
        let bank = order.iter().position(|l| l == "OOK bank").expect("a bank");
        assert!(ring < bank, "the recorder runs after the decoders: {order:?}");
        let _ = std::fs::remove_dir_all(&dir);
    }
}
