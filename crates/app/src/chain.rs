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
    /// Keyed by the channel, like the pager one and for the same reason: a
    /// scanner table pointed at another frequency has to rebuild the node
    /// rather than keep one tuned where it was.
    Aprs(u64),
    Pocsag(u64),
    /// Banks are distinguished by the band they cover and the channel width
    /// they were built for. Two scanner blocks can ask for the same width in
    /// different bands, and they are not the same bank.
    Bank(u64, u32),
    /// The decimator feeding one band's banks, keyed by the band and the
    /// factor: a different factor is a different filter.
    SubBandDecimate(u64, usize),
    /// The mixer in front of it. Never reused, because its shift follows the
    /// dial and it is a phase accumulator rather than a filter to design, but
    /// it still needs a role: `roles` is zipped with the graph's nodes to find
    /// them again, so a node added without one shifts every role after it onto
    /// the wrong node.
    SubBandMix(u64),
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
    aprs: Option<NodeId>,
    pocsag: Option<NodeId>,
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
    /// The front ends to run, from the scanner table for this span. Empty is
    /// a span nothing is configured for, which costs nothing rather than
    /// sweeping it for sensors that are not there.
    ///
    /// Several, because a span is wide: a couple of megahertz of VHF can hold
    /// a pager channel and a packet channel at once, and both are one
    /// narrowband demodulator each.
    pub fronts: Vec<crate::scanners::FrontAt>,
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
            aprs: None,
            pocsag: None,
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
            Role::Bank(..) => true,
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
        self.aprs = None;
        self.pocsag = None;
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

        // The front ends the scanner table put on this span, rather than a
        // chain of band tests here. Which demodulator belongs on which
        // frequency is configuration, not structure: see `scanners.rs`.
        let (mut modes, mut ais, mut aprs, mut pocsag) = (None, None, None, None);
        let mut banks = Vec::new();
        let mut refused = None;
        // Everything that is not a bank, in the order the table listed it, so
        // the bus can be connected to all of them without asking which kinds
        // happen to be running.
        let mut narrowband: Vec<NodeId> = Vec::new();
        // A single-channel front end whose channel does not clear the span
        // edge by its own bandwidth is dropped rather than built. The node
        // would refuse it at negotiation and take the whole graph down with
        // it, and one badly placed block should cost its own front end, not
        // the receiver.
        let fits = |hz: f64, width: f64| {
            (hz - plan.center.as_f64()).abs() <= plan.eff_rate() / 2.0 - width
        };
        // One extraction per band, shared by whatever listens in it.
        let mut extracts: HashMap<(u64, usize), NodeId> = HashMap::new();
        for at in &plan.fronts {
            let front = &at.front;
            // Everything the scanner table puts on the span is fed a band cut
            // out for it rather than the whole span. What each front end then
            // does inside itself is a small residual shift at a low rate,
            // instead of a mixer and a several-thousand-tap filter running at
            // the radio's own rate where nothing could see them.
            let want = front_band(front, at).and_then(|(band, min_rate)| {
                let band = match front {
                    // A bank takes what the span covers of its band; a
                    // single-channel front end needs all of its channel or it
                    // is refused outright below.
                    Front::Banks(_) => at.covered(plan.center.as_f64(), plan.eff_rate())?,
                    _ => band,
                };
                (band.1 > band.0).then(|| SubBand::plan(band, plan.eff_rate(), min_rate))
            });
            let src = match want {
                Some(sub) => extract(
                    &mut b,
                    &mut roles,
                    &mut pool,
                    &mut extracts,
                    head,
                    plan.center.as_f64(),
                    plan.eff_rate(),
                    sub,
                ),
                None => head,
            };
            match front {
            Front::ModeS => {
                let id = match pool.remove(&Role::ModeS) {
                    Some(p) => b.add_existing(p),
                    None => b.add_labeled("1090 Mode S", Box::new(ModeSNode::default())),
                };
                b.connect(src.o(), id.i());
                roles.push(Role::ModeS);
                narrowband.push(id);
                modes = Some(id);
            }
            Front::Ais => {
                let id = match pool.remove(&Role::Ais) {
                    Some(p) => b.add_existing(p),
                    None => b.add_labeled("162 AIS", Box::new(nodes::AisNode::default())),
                };
                b.connect(src.o(), id.i());
                roles.push(Role::Ais);
                narrowband.push(id);
                ais = Some(id);
            }
            Front::Aprs(hz) if !fits(*hz, nodes::aprs_nodes::CHANNEL_WIDTH_HZ) => {
                refused = Some(format!("{:.4} MHz is too near the span edge for aprs", hz / 1e6));
            }
            Front::Aprs(hz) => {
                let role = Role::Aprs(*hz as u64);
                let id = match pool.remove(&role) {
                    Some(p) => b.add_existing(p),
                    None => b.add_labeled(
                        format!("{:.3} APRS", hz / 1e6),
                        Box::new(nodes::AprsNode::new(*hz)),
                    ),
                };
                b.connect(src.o(), id.i());
                roles.push(role);
                narrowband.push(id);
                aprs = Some(id);
            }
            Front::Pocsag(hz) if !fits(*hz, nodes::pocsag_nodes::CHANNEL_WIDTH_HZ) => {
                refused =
                    Some(format!("{:.4} MHz is too near the span edge for pocsag", hz / 1e6));
            }
            Front::Pocsag(hz) => {
                let role = Role::Pocsag(*hz as u64);
                let id = match pool.remove(&role) {
                    Some(p) => b.add_existing(p),
                    None => b.add_labeled(
                        format!("{:.4} pager", hz / 1e6),
                        Box::new(nodes::PocsagNode::new(*hz)),
                    ),
                };
                b.connect(src.o(), id.i());
                roles.push(role);
                narrowband.push(id);
                pocsag = Some(id);
            }
            Front::Banks(widths) => {
                // The band the block was written about, not the whole span.
                // A bank handed 60 MS/s divides it into 1024 channels at best,
                // which is 60 kHz each: far wider than the 25 kHz an OOK
                // sensor occupies, so several devices share a channel and the
                // detector sees one long burst instead of packets. The
                // extraction above buys that resolution back, and costs less,
                // because the channelizer then runs at the band's rate rather
                // than the radio's.
                let Some(band) = at.covered(plan.center.as_f64(), plan.eff_rate()) else {
                    continue;
                };
                let sub = SubBand::plan(band, plan.eff_rate(), 0.0);
                // Two tiers that come out the same width are one tier. A
                // channelizer has a floor of two channels, so every tier wider
                // than half the band degenerates to that floor and duplicates
                // whichever tier got there first: on a 250 kHz capture the
                // 125 kHz tier and the 500 kHz one are both two channels of
                // 125 kHz, and the burst is then decoded twice, identically,
                // and logged as two receptions of one transmission.
                let mut built: Vec<usize> = Vec::new();
                for &width in widths {
                    let channels = BankNode::channels_for(sub.rate(plan.eff_rate()), width);
                    if built.contains(&channels) {
                        continue;
                    }
                    built.push(channels);
                    // Named by width rather than by keying. Every tier runs
                    // the same graph now that the burst is measured instead of
                    // assumed, so "the OOK bank" would be a claim about what a
                    // channel hears that nothing checks: the wide tier carries
                    // on-off keyed sensors, and always did.
                    let label = bank_label(width);
                    let role = Role::Bank(sub.key(), width as u32);
                    let id = match pool.remove(&role) {
                        // Set before it goes back into the graph, because the
                        // mask decides which channels get a decoder and that
                        // happens while the graph negotiates.
                        Some(mut p) => {
                            if let Some(n) = p
                                .node
                                .as_any_mut()
                                .and_then(|a| a.downcast_mut::<BankNode>())
                            {
                                n.set_band(Some(band));
                            }
                            b.add_existing(p)
                        }
                        None => {
                            let mut n =
                                BankNode::new(label.clone(), width, nodes::ism_decode_graph);
                            n.set_band(Some(band));
                            b.add_labeled(label, Box::new(n))
                        }
                    };
                    b.connect(src.o(), id.i());
                    roles.push(role);
                    banks.push((id, width));
                }
            }
            }
        }

        let mut chans: Vec<Chan> = Vec::new();
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
            .chain(narrowband.iter().map(|n| n.o()))
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
        let makes_tracks = plan
            .fronts
            .iter()
            .any(|f| matches!(f.front, Front::ModeS | Front::Ais | Front::Aprs(_)));
        if let (Some(bus), true) = (bus, makes_tracks || !plan.feeds.is_empty()) {
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
        self.aprs = aprs;
        self.pocsag = pocsag;
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
                    // What is decoding, not what the channelizer produces:
                    // the channels outside the wanted band have no decoder on
                    // them and reporting them overstates what is being heard.
                    .map(|b| b.active_channels())
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

    pub fn aprs_on(&self) -> bool {
        self.aprs.is_some()
    }

    pub fn pocsag_on(&self) -> bool {
        self.pocsag.is_some()
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
        node.set_param(name, value)?;
        Ok(affects_rate)
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
        Front::Banks(widths) => {
            let band = at.band;
            // Two channels is the least a channelizer will build, so the band
            // has to arrive at least that wide.
            let widest = widths.iter().cloned().fold(0.0f64, f64::max);
            Some((band, widest * 2.0))
        }
    }
}


/// Build the mixer and decimator that cut one band out of the span.
///
/// Returns the node whose output the front end should read, which is the head
/// itself when the band is the span and nothing needs doing. Cached by band
/// and factor, so two front ends listening in the same place share one
/// extraction instead of each running its own mixer over every sample.
#[allow(clippy::too_many_arguments)]
fn extract(
    b: &mut GraphBuilder,
    roles: &mut Vec<Role>,
    pool: &mut HashMap<Role, NodePart>,
    cache: &mut HashMap<(u64, usize), NodeId>,
    head: NodeId,
    span_center: f64,
    span_rate: f64,
    sub: SubBand,
) -> NodeId {
    if sub.is_whole_span(span_center) {
        return head;
    }
    if let Some(id) = cache.get(&(sub.key(), sub.factor)) {
        return *id;
    }
    // The mixer is built fresh every time: its shift follows the dial, and it
    // is a phase accumulator rather than a filter worth keeping.
    let mix = b.add_labeled(
        format!("{:.4} MHz mixer", sub.center / 1e6),
        Box::new(MixerNode::new(span_center - sub.center)),
    );
    roles.push(Role::SubBandMix(sub.key()));
    b.connect(head.o(), mix.i());

    let role = Role::SubBandDecimate(sub.key(), sub.factor);
    let dec = match pool.remove(&role) {
        Some(p) => b.add_existing(p),
        None => {
            let mut d = DecimateNode::new(sub.factor);
            d.set_passband_hz(span_rate, sub.need / 2.0);
            b.add_labeled(
                format!("/{} to {}", sub.factor, hz_label(sub.rate(span_rate))),
                Box::new(d),
            )
        }
    };
    roles.push(role);
    b.connect(mix.o(), dec.i());
    cache.insert((sub.key(), sub.factor), dec);
    dec
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
    /// A band a bank channelizes, and the grid it channelizes it on.
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
        Some("FSK") => FSK_CHANNEL_HZ,
        _ => OOK_CHANNEL_HZ,
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

    /// A front end with no band of its own, which is what every front end
    /// except a bank has: only a bank is built over a range.
    pub(super) fn anywhere(front: Front) -> crate::scanners::FrontAt {
        crate::scanners::FrontAt { front, band: (0.0, f64::INFINITY) }
    }

    pub(super) fn plan(rate: f64, center: Hz) -> Plan {
        Plan {
            center,
            rate,
            zoom: 1,
            dc_block: true,
            refresh_hz: 30.0,
            fft: 1024,
            channels: Vec::new(),
            fronts: vec![crate::scanners::FrontAt {
                front: Front::Banks(crate::scanners::DEFAULT_WIDTHS.to_vec()),
                band: (0.0, f64::INFINITY),
            }],
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
    fn the_ism_band_is_marked_where_the_bank_is_looking() {
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
        assert!(band.2 > 0.0 && band.2 < 200_000.0, "{band:?}");
    }

    #[test]
    fn a_pager_channel_is_marked_at_its_frequency() {
        let s = crate::scanners::Scanners::default();
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
