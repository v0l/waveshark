//! One node that finds and decodes whatever is in the span, on its own.
//!
//! In at one end is complex baseband at whatever rate and centre the radio
//! has. Out at the other are packets: bursts as timings for the protocol
//! tables, and frames from the demodulators that produce bytes. Nothing in
//! between is told a frequency, a width or a modulation.
//!
//! Inside are the two things a span holds. Most of what transmits is found
//! by watching the span as a spectrogram, as [`dsp::source`] does: a source
//! opens where something appears, is cut out at a rate that fits its width,
//! and gets decoders of its own for as long as it lasts. The burst front end
//! always, since it measures the burst and picks the demodulator itself; and
//! the narrowband frame decoders whose channel a source of that width could
//! be, a pager or a packet channel, which decide for themselves whether the
//! bits are theirs. A pager channel is a pager channel at 153 MHz and at
//! 440 MHz, and a receiver that has to be told which is not detecting.
//!
//! The rest is what a spectrogram cannot find. A Mode S reply is 120
//! microseconds of pulses two megahertz wide, shorter than a frame; AIS is
//! two channels 50 kHz apart that stations alternate between. Those
//! demodulators watch the whole span themselves, and run when the span
//! covers the frequency they are for. That is the one piece of knowledge
//! about where things are that the node keeps, because it is knowledge about
//! the world rather than about this radio: 1090 MHz is 1090 MHz everywhere.

use common::{Hz, Packet, PacketBody, Result, SourceBlock, SourceId, SourceState, C32};
use dsp::{SourceConfig, SourceDetector, SourceEvent, SourceExtractor};
use pipeline::event::Event;
use pipeline::graph::Topology;
use pipeline::node::{Node, NodeCtx, PortSpec};
use pipeline::param::{Param, ParamValue};
use pipeline::port::{Payload, PortKind, StreamSpec};
use pipeline::registry::Registry;
use pipeline::{Graph, Out};
use rayon::prelude::*;
use std::collections::{BTreeMap, HashMap};
use std::time::Instant;

use crate::{build_chain, NodeSpec};

/// SNR a bin must reach before the auto node opens a source there.
///
/// Above the detector's own default because this node builds a decoder chain
/// for everything it opens: on a band with a strong transmitter on it, the
/// weakest openings are mostly the splash and spurs around one signal, and
/// each of them costs a chain.
pub const AUTO_OPEN_DB: f32 = 15.0;

/// How much wider than its declared channel a source may measure and still
/// have that channel's front end placed on it. A clean channel measures a
/// little over its width (an M17 12.5 kHz channel lands around 25 kHz once
/// extracted); splatter and a nearby spur can measure it far wider, and past
/// this a 12.5 kHz decoder does not belong on the signal. Three times keeps
/// the old ~40 kHz ceiling for a 12.5 kHz channel while scaling with width.
const CHANNEL_WIDTH_TOLERANCE: f64 = 3.0;

/// Widths a meter transmission has: 100 kchip/s keyed 50 kHz either way,
/// with what the extraction adds around it.
const METER_HZ: std::ops::RangeInclusive<f64> = 60_000.0..=450_000.0;

/// How often a transmission that never ends is reported, in seconds.
///
/// A base station carrier is on all day. The burst front end cuts it into
/// pieces of half a second to have something to measure, and a row per
/// piece would be a list of nothing else. One when it is found, then one
/// every so often to say it is still there, is what "which channels are
/// busy" needs.
const REPORT_S: f64 = 5.0;

/// One decoder over one stream: a graph, and where its packets come out.
struct Member {
    name: &'static str,
    graph: Graph,
    pulses: Vec<Out>,
    frames: Vec<Out>,
    /// Front ends that build their own packets, because what they produce is
    /// more than bytes: an M17 voice stream carries speech beside them.
    packets: Vec<Out>,
    /// Front ends that carry speech, on the port it travels out on.
    voice: Vec<Out>,
    /// Pictures, for a front end inside that produces them. Same arrangement
    /// as `voice` and for the same reason: what a source turns out to be
    /// decides what leaves the node, and a video front end placed on a source
    /// must have somewhere to publish to without the auto node knowing which
    /// front end it was.
    video: Vec<Out>,
    /// The burst front end inside, when this is it: its packets are read
    /// from what it measured rather than from its port, so every burst
    /// leaves with its measurement, and a burst no front end reads leaves
    /// as a packet of nothing but the measurement.
    router: Option<pipeline::NodeId>,
    /// The SNR the detector measured for the source this member reads, so a
    /// frame decoder that measures no level of its own still reports the
    /// level of the transmission it came from rather than nothing. NaN
    /// until [`Slot::open`] sets it from the source block.
    source_snr_db: f32,
    /// Peak mean-square power of the extracted stream since the last frame
    /// left, held across the blocks a transmission spans. A frame decoder
    /// reads bits and reports no level, but the samples it read have one,
    /// and the loudest block of a page is the page's RSSI. Reset when a
    /// frame is emitted, so the tail silence after it does not drag the
    /// next transmission's level down.
    peak_pow: f32,
    /// When a transmission still going was last reported, in seconds of
    /// stream, so it is reported every [`REPORT_S`] rather than every piece.
    last_report_s: Option<f64>,
    /// Width of the channel this front end was placed for, in hertz, or
    /// zero for one that measures the burst rather than reading a channel.
    channel_hz: f64,
    /// Silence to feed after the source closes, in seconds: the most any
    /// node in the graph asked for through [`Node::flush_s`]. A pager
    /// transmission has no closing flag and a DMR over that lost its
    /// terminator ends on a second and a half of silence, so dropping the
    /// decoder when the source closes drops the page or the over with it.
    flush_s: f64,
    /// The source's samples since the last packet left, up to
    /// [`RING_MAX_S`], so a packet from a front end that did not cut its
    /// own samples out still leaves with the stream it was read from. A
    /// packet in the log without its samples cannot be decoded again by
    /// anything written later, and a row that says only what one decoder
    /// made of a burst is an event, not a packet.
    ring: Vec<C32>,
    /// Quietest block power seen, rising slowly, so a source whose level the
    /// detector did not measure (a channel kept open for the session) still
    /// reports a signal to noise ratio on its packets.
    noise_pow: f32,
    /// The burst front end inside has named a burst of this source a chirp.
    /// What places a LoRa front end on the source, late, fed from `ring`.
    chirp_seen: bool,
}

/// Longest run of samples kept behind a packet, in seconds.
const RING_MAX_S: f64 = 2.0;

/// How wide a source has to be before a video front end is placed on it.
///
/// The AKK transmitter measured here occupied about 4.6 MHz, and nothing else
/// this receiver reads is anywhere near that: the widest is BLE at 2 MHz.
/// Three is comfortably between them.
const VIDEO_MIN_HZ: f64 = 3e6;

/// And how fast the stream carrying it has to be. PAL luma reaches 5 MHz with
/// the colour subcarrier at 4.43, so a slower stream cannot hold a picture
/// whatever the source measured.
const VIDEO_MIN_RATE_HZ: f64 = 12e6;

impl Member {
    fn build(
        name: &'static str,
        spec: StreamSpec,
        settings: NodeSpec,
        reg: &Registry,
    ) -> Result<Self> {
        let graph = build_chain(spec, &[settings], reg)?;
        let pulses = taps(&graph, PortKind::Pulses);
        let frames = taps(&graph, PortKind::Frames);
        let packets = taps(&graph, PortKind::Packets);
        let voice = taps(&graph, PortKind::Voice);
        let video = taps(&graph, PortKind::Video);
        let router = graph
            .order()
            .find(|(_, n)| *n == "burst_route")
            .map(|(id, _)| id);
        let flush_s = graph
            .order()
            .filter_map(|(id, _)| graph.node(id).map(|n| n.flush_s()))
            .fold(0.25, f64::max);
        Ok(Self {
            name,
            graph,
            pulses,
            frames,
            packets,
            voice,
            video,
            router,
            source_snr_db: f32::NAN,
            peak_pow: 0.0,
            last_report_s: None,
            channel_hz: 0.0,
            flush_s,
            ring: Vec::new(),
            noise_pow: f32::NAN,
            chirp_seen: false,
        })
    }

    /// Run one block through and collect what came out as packets.
    fn run(&mut self, iq: &[C32], at_us: u64, out: &mut Vec<Packet>) -> Vec<Event> {
        let rate = self.graph.input_spec().rate;
        if !iq.is_empty() {
            let pow = iq.iter().map(|c| c.norm_sqr()).sum::<f32>() / iq.len() as f32;
            self.peak_pow = self.peak_pow.max(pow);
            // The floor follows the quietest block and climbs a hundredth
            // a block, so a burst does not become the floor and a real
            // rise in the noise is learned within a second or so.
            self.noise_pow = if self.noise_pow.is_nan() {
                pow
            } else {
                pow.min(self.noise_pow * 1.01)
            };
            self.ring.extend_from_slice(iq);
            // Trimmed once it holds twice what is kept, not every block:
            // trimming a full ring by a block's worth moves the whole of it
            // down, and five members on each of a few sources doing that on
            // every block was gigabytes a second of memmove on a busy band,
            // more than the decoding they were keeping the samples for.
            let cap = (RING_MAX_S * rate) as usize;
            if self.ring.len() >= 2 * cap {
                let drop = self.ring.len() - cap;
                self.ring.drain(..drop);
            }
        }
        let first = out.len();
        let events = self.run_graph(iq, at_us, out);
        // What the front end did not cut out for itself is given the stream
        // since the last packet, and the level it stood at.
        let mut attached = false;
        for p in &mut out[first..] {
            if p.iq.is_none() && !self.ring.is_empty() {
                p.iq = Some(std::sync::Arc::new(common::IqBurst {
                    rate,
                    center_hz: self.graph.input_spec().center.0,
                    samples: self.ring.clone(),
                }));
                attached = true;
            }
            if p.rssi_dbfs.is_nan() {
                p.rssi_dbfs = 10.0 * self.peak_pow.max(1e-20).log10();
            }
            if p.snr_db.is_nan() && self.noise_pow > 0.0 {
                p.snr_db = 10.0 * (self.peak_pow / self.noise_pow).max(1.0).log10();
            }
        }
        if attached {
            self.ring.clear();
        }
        events
    }

    fn run_graph(&mut self, iq: &[C32], at_us: u64, out: &mut Vec<Packet>) -> Vec<Event> {
        let buf = self.graph.input_buf();
        buf.clear();
        buf.iq_mut().extend_from_slice(iq);
        let mut events = match self.graph.run() {
            Ok(ev) => ev.to_vec(),
            Err(e) => vec![Event::Warning {
                stage: self.name.into(),
                message: e.to_string(),
            }],
        };
        if let Some(id) = self.router {
            let spec = self.graph.spec_of(id.o());
            let center_hz = spec.map(|s| s.center.0).unwrap_or(0);
            let bandwidth_hz = spec.map(|s| s.bandwidth as u32).unwrap_or(0);
            let node = self
                .graph
                .node(id)
                .and_then(|n| n.as_any())
                .and_then(|a| a.downcast_ref::<crate::BurstRouteNode>());
            // The samples are at the rate the router was fed, which is the
            // source's extraction rate; the router's own output port is a
            // packet stream and carries no rate.
            let rate = self.graph.input_spec().rate;
            for b in node.map(|n| n.routed()).unwrap_or(&[]) {
                if b.class.modulation == dsp::Modulation::Chirp {
                    self.chirp_seen = true;
                }
                // A diagnostic: with `SR_DUMP_BURSTS` naming a directory,
                // every burst the router cut is written there as
                // interleaved f32 IQ, named with the centre, the rate and
                // the start sample, which is what the classifier's
                // `score_a_dumped_burst` test reads. How
                // a verdict on a real signal came out is otherwise
                // invisible, and that is how the TETRA carriers were found
                // to be read as OFDM.
                if let Some(dir) = std::env::var_os("SR_DUMP_BURSTS") {
                    let path = std::path::Path::new(&dir).join(format!(
                        "burst_{}_{}_{}.c64",
                        center_hz, rate as u64, b.start_sample
                    ));
                    if !path.exists() {
                        let mut bytes = Vec::with_capacity(b.iq.len() * 8);
                        for c in &b.iq {
                            bytes.extend_from_slice(&c.re.to_le_bytes());
                            bytes.extend_from_slice(&c.im.to_le_bytes());
                        }
                        let _ = std::fs::write(path, bytes);
                    }
                }
                let m = crate::decode_nodes::measure_of(b, center_hz as f64);
                let iq = Some(std::sync::Arc::new(common::IqBurst {
                    rate,
                    center_hz,
                    samples: b.iq.clone(),
                }));
                if b.packages.is_empty() {
                    // A burst nothing reads is worth a row when the
                    // classifier named it as something no front end here
                    // reads, and was sure: a chirp, a carrier. One a front
                    // end read and got no pulses from is too short or too
                    // weak to be a packet, and one the classifier could not
                    // name is a gate opening on noise inside a stream; a
                    // list of those is a list of nothing.
                    if b.routed_to != "none"
                        || b.class.confidence < 0.5
                        || !b.class.modulation.is_named()
                    {
                        continue;
                    }
                    // A piece of a transmission that is still going is the
                    // same news as the last piece, most of the time.
                    if b.continuous {
                        let t = b.start_sample as f64 / rate.max(1.0);
                        if self.last_report_s.is_some_and(|l| t - l < REPORT_S) {
                            continue;
                        }
                        self.last_report_s = Some(t);
                    }
                    out.push(Packet {
                        at_us,
                        center_hz,
                        bandwidth_hz,
                        // Filled from the source's own level in `run`, which
                        // is where the samples are; the classifier measures
                        // the burst against the noise it found, and reports
                        // nothing when it never found any.
                        rssi_dbfs: f32::NAN,
                        snr_db: if b.class.features.snr_db > 0.0 {
                            b.class.features.snr_db
                        } else {
                            f32::NAN
                        },
                        modulation: None,
                        body: PacketBody::Pulses(Vec::new()),
                        measure: Some(m),
                        audio: None,
                        iq: iq.clone(),
                    });
                    continue;
                }
                for p in &b.packages {
                    out.push(Packet {
                        at_us,
                        center_hz,
                        bandwidth_hz,
                        rssi_dbfs: p.rssi_dbfs,
                        snr_db: p.snr_db,
                        modulation: p.modulation,
                        body: PacketBody::Pulses(p.pulses.clone()),
                        measure: Some(m.clone()),
                        audio: None,
                        iq: iq.clone(),
                    });
                }
            }
            // The front end's own report of a burst nothing reads is the
            // measurement it just handed over; a second row would say the
            // same thing.
            events.retain(|e| !matches!(e, Event::Decoded(d) if d.protocol == "unidentified"));
            return events;
        }
        for t in &self.pulses {
            let spec = self.graph.spec_of(*t);
            let Some(pkgs) = self.graph.buf(*t).and_then(|p| p.as_pulses()) else {
                continue;
            };
            for p in pkgs {
                out.push(Packet {
                    at_us,
                    center_hz: p.center_hz,
                    bandwidth_hz: spec.map(|s| s.bandwidth as u32).unwrap_or(0),
                    rssi_dbfs: p.rssi_dbfs,
                    snr_db: p.snr_db,
                    modulation: p.modulation,
                    body: PacketBody::Pulses(p.pulses.clone()),
                    measure: None,
                    audio: None,
                    iq: None,
                });
            }
        }
        for t in &self.packets {
            let Some(pk) = self.graph.buf(*t).and_then(|p| p.as_packets()) else {
                continue;
            };
            // Taken as they are, except for a level the front end left
            // unmeasured: a dechirp reports its processing gain, not a
            // channel level, so the LoRa node leaves both NaN and the
            // source's own measurement fills them here. A front end that did
            // measure keeps what it said.
            for p in pk {
                let mut p = p.clone();
                if p.rssi_dbfs.is_nan() {
                    p.rssi_dbfs = 10.0 * self.peak_pow.max(1e-20).log10();
                }
                if p.snr_db.is_nan() {
                    p.snr_db = self.source_snr_db;
                }
                out.push(p);
            }
            if !pk.is_empty() {
                self.peak_pow = 0.0;
            }
        }
        for t in &self.frames {
            let spec = self.graph.spec_of(*t);
            let Some(frames) = self.graph.buf(*t).and_then(|p| p.as_frames()) else {
                continue;
            };
            for f in frames {
                // What the front end measured, where it measured anything: it
                // read the channel this frame came off, and the source's own
                // level is the whole extraction. The fallbacks are here for a
                // front end that has not been taught to measure yet.
                let rssi = if f.rssi_dbfs.is_nan() {
                    10.0 * self.peak_pow.max(1e-20).log10()
                } else {
                    f.rssi_dbfs
                };
                let snr = if f.snr_db.is_nan() {
                    self.source_snr_db
                } else {
                    f.snr_db
                };
                out.push(Packet {
                    at_us,
                    center_hz: f
                        .center_hz
                        .unwrap_or_else(|| spec.map(|s| s.center.0).unwrap_or(0)),
                    bandwidth_hz: spec.map(|s| s.bandwidth as u32).unwrap_or(0),
                    rssi_dbfs: rssi,
                    snr_db: snr,
                    modulation: None,
                    body: PacketBody::Frame(f.bytes.clone()),
                    measure: None,
                    audio: None,
                    iq: f.iq.clone(),
                });
            }
            // The page has left carrying the loudest block it was read
            // from; the next transmission on this source measures its own.
            if !frames.is_empty() {
                self.peak_pow = 0.0;
            }
        }
        events
    }
}

/// Every output of a graph carrying a given kind.
fn taps(g: &Graph, kind: PortKind) -> Vec<Out> {
    // Every port, not only the first. A front end that carries speech
    // alongside its packets puts it on a second output, and a scan that
    // stopped at port zero found the packets and left the audio where it
    // was: decoded, and never heard.
    g.order()
        .flat_map(|(id, _)| {
            let outs = g.node(id).map(|n| n.num_outputs()).unwrap_or(1);
            (0..outs).map(move |p| id.out(p))
        })
        .filter(|o| g.spec_of(*o).map(|s| s.kind) == Some(kind))
        .collect()
}

/// A channel a front end has read something on, kept for as long as the
/// node runs.
///
/// Detection finds what is transmitting; it does not know what will. A
/// channel that has produced a decoded frame is one that will produce
/// another, on the same frequency, in the same width, read by the same
/// front end, and from then on it does not have to be found again: it is
/// cut out continuously and that front end alone reads it. The cases this
/// pays for are the ones detection gets wrong on a channel it has already
/// proved: a burst too strong for the converter, which lights the whole
/// span and measures as two megahertz wide, and a weak one that flickers
/// around the open threshold and comes out as three slivers. Where a
/// transmitter moves, the front end that reads it is the one to follow it;
/// the channel here is where it was heard.
///
/// Not saved. A channel is remembered for a session, and a session that
/// starts fresh finds its channels the same way it did the first time.
struct Sticky {
    id: SourceId,
    name: &'static str,
    center_hz: f64,
    width_hz: f64,
}

/// Source ids counted down from the top, where the detector's never reach.
const STICKY_ID_BASE: u64 = u64::MAX - 1_000_000;

/// One open source and the decoders reading it.
struct Slot {
    id: SourceId,
    center_hz: u64,
    members: Vec<Member>,
    /// A front end has read something from this source. From then on the
    /// burst front end's measurement of it is not news: a row saying what
    /// the carrier looks like beside rows saying what it said.
    heard: bool,
    /// The stream the members were built for, and the width the detector
    /// measured, for a front end placed after the source opened.
    spec: StreamSpec,
    signal_hz: f64,
    /// A LoRa front end has been placed, or ruled out, for this source.
    lora_placed: bool,
}

pub struct AutoNode {
    label: String,
    cfg: SourceConfig,
    rate: f64,
    center: Hz,
    input_bw: f64,
    band: Option<(f64, f64)>,
    spur: Option<f64>,
    /// Around the spur, in absolute hertz, once the resolution is known.
    spur_band: Option<(f64, f64)>,
    /// The channel plan on this band, as an origin and a step in hertz, when
    /// there is one. See [`snap_to_raster`].
    raster: Option<(f64, f64)>,
    detector: Option<SourceDetector>,
    extractor: Option<SourceExtractor>,
    reg: Registry,
    slots: Vec<Slot>,
    /// Decoders that watch the whole span, and the bands they own, in
    /// absolute hertz, where no source is opened.
    wide: Vec<Member>,
    exclude: Vec<(f64, f64)>,
    /// The burst front end at a nominal rate, for the view and the
    /// parameters before any source has opened.
    template: Option<Graph>,
    events: Vec<SourceEvent>,
    blocks: Vec<SourceBlock>,
    hits: Vec<(Hz, Event)>,
    /// Narrowband front ends and the channel widths they declared, asked of
    /// the registry once rather than kept as a table here: the auto node no
    /// longer decides who fits where, it asks each front what it wants. Only
    /// stages that returned a non-empty `Node::channels` are here.
    narrowband: Vec<(&'static str, &'static [f64])>,
    /// Sources decoders were built for, over the node's life.
    built: u64,
    sticky: Vec<Sticky>,
    /// Channels to cut out from the next block on: newly heard ones, and
    /// after a rebuild every one the span still covers.
    pending_sticky: Vec<SourceId>,
    /// What each channel has announced about itself, so a source that
    /// closes and opens again, or decoders rebuilt with the graph, do not
    /// log the same cell's identity a second time.
    announced: HashMap<u64, Vec<Vec<u8>>>,
    /// Where a block's time went: watching the band, cutting sources out,
    /// the front ends as a whole, and each kind of front end's processor
    /// time summed over every source it was running on.
    phases: BTreeMap<String, pipeline::cost::Ring>,
    /// Scratch for the per-kind sums of one block.
    phase_sum: BTreeMap<&'static str, u64>,
}

impl AutoNode {
    pub fn new(label: impl Into<String>, cfg: SourceConfig) -> Self {
        Self {
            label: label.into(),
            cfg,
            rate: 0.0,
            center: Hz(0),
            input_bw: 0.0,
            band: None,
            spur: None,
            spur_band: None,
            raster: None,
            detector: None,
            extractor: None,
            reg: crate::registry(),
            narrowband: Vec::new(),
            slots: Vec::new(),
            wide: Vec::new(),
            exclude: Vec::new(),
            template: None,
            events: Vec::new(),
            blocks: Vec::new(),
            hits: Vec::new(),
            built: 0,
            sticky: Vec::new(),
            pending_sticky: Vec::new(),
            announced: HashMap::new(),
            phases: BTreeMap::new(),
            phase_sum: BTreeMap::new(),
        }
    }

    fn phase(&mut self, name: &str, us: u64, block_s: f64) {
        let ring = match self.phases.get_mut(name) {
            Some(r) => r,
            None => self.phases.entry(name.to_string()).or_default(),
        };
        ring.push(us.min(u32::MAX as u64) as u32, block_s);
    }

    /// Channels front ends have read something on this session, as
    /// (front end, centre, width) in hertz.
    pub fn remembered(&self) -> Vec<(&'static str, f64, f64)> {
        self.sticky
            .iter()
            .map(|s| (s.name, s.center_hz, s.width_hz))
            .collect()
    }

    /// Limit detection to a band inside the input, or `None` for all of it.
    pub fn set_band(&mut self, band: Option<(f64, f64)>) {
        self.band = band;
        self.apply_band();
    }

    /// The tuner's own centre, where a source may open only when nothing
    /// else is transmitting.
    ///
    /// A direct-conversion receiver's DC offset is not steady: a strong
    /// signal anywhere in the span modulates it with its own envelope, and
    /// the DC block passes that as readily as any other keying. Read as a
    /// source it was an unknown 10 kHz wide, exactly as long as the sensor
    /// burst beside it, for every packet that sensor sent. It never happens
    /// alone, so a source at the centre is refused only while another is
    /// open elsewhere, and a device that really sits on the centre still
    /// opens when it transmits by itself.
    pub fn set_spur(&mut self, hz: Option<f64>) {
        self.spur = hz;
        self.apply_band();
    }

    pub fn band(&self) -> Option<(f64, f64)> {
        self.band
    }

    /// The channel plan on this band: a frequency the plan lands on and the
    /// spacing, in hertz. A source found close to a channel is locked to it;
    /// see [`snap_to_raster`].
    pub fn set_raster(&mut self, raster: Option<(f64, f64)>) {
        self.raster = raster.filter(|(_, step)| *step > 0.0);
    }

    pub fn raster(&self) -> Option<(f64, f64)> {
        self.raster
    }

    fn apply_band(&mut self) {
        if let (Some(d), Some((lo, hi))) = (self.detector.as_mut(), self.band) {
            let c = self.center.as_f64();
            d.set_band(lo - c, hi - c);
        }
        // Three bins either side, tested against the source's centre.
        self.spur_band = match (self.detector.as_ref(), self.spur) {
            (Some(d), Some(hz)) => Some((hz - 3.0 * d.bin_hz(), hz + 3.0 * d.bin_hz())),
            _ => None,
        };
        // And the floor cap left off there: the residual DC is a permanent
        // hump the cap would otherwise unhide, and reported it is an unknown
        // at the centre of every span for as long as the receiver runs.
        if let (Some(d), Some((lo, hi))) = (self.detector.as_mut(), self.spur_band) {
            let c = self.center.as_f64();
            d.exempt_from_cap(lo - c, hi - c);
        }
    }

    /// Sources open right now.
    pub fn live(&self) -> Vec<dsp::Source> {
        self.detector
            .as_ref()
            .map(|d| d.live().copied().collect())
            .unwrap_or_default()
    }

    /// What decoded in the last block, and where.
    pub fn hits(&self) -> &[(Hz, Event)] {
        &self.hits
    }

    /// Sources with decoders on them right now.
    pub fn active(&self) -> usize {
        self.slots.len()
    }

    /// Sources decoders were built for since the node was made.
    pub fn built(&self) -> u64 {
        self.built
    }

    /// The span-wide decoders running, by stage name.
    pub fn wide(&self) -> Vec<&'static str> {
        self.wide.iter().map(|m| m.name).collect()
    }

    /// Speech from every front end inside, read off the ports it came out
    /// on, whatever protocol produced it.
    ///
    /// A source found a moment ago has decoders built for it there and then,
    /// and they are as much a part of the receiver as a stage somebody
    /// placed by hand. Taken from the ports rather than from a list of
    /// protocol names kept here, so a voice front end added later is heard
    /// without this file being touched.
    /// The key status of every TETRA front end the scanner placed inside, so
    /// a cell heard through the auto node reaches the key manager the same as
    /// one placed by hand.
    pub fn inner_tetra_status(&self) -> Vec<crate::tetra_nodes::KeyStatus> {
        let mut out = Vec::new();
        for slot in &self.slots {
            for m in &slot.members {
                for (id, name) in m.graph.order() {
                    if name != "tetra" {
                        continue;
                    }
                    if let Some(t) = m
                        .graph
                        .node(id)
                        .and_then(|n| n.as_any())
                        .and_then(|a| a.downcast_ref::<crate::tetra_nodes::TetraNode>())
                    {
                        out.extend(t.key_status());
                    }
                }
            }
        }
        out
    }

    /// Install a key on every inner TETRA front end for a cell colour, so a
    /// manual key entered in the manager reaches a cell heard through the
    /// scanner as well as one placed by hand.
    #[cfg(feature = "tea")]
    pub fn set_inner_tetra_key(&mut self, colour: u8, key: decode::tea::Key) {
        self.each_inner_tetra(|t| t.add_key(colour, key));
    }

    /// Install a TA61 identity secret on every inner TETRA front end.
    #[cfg(feature = "tea")]
    pub fn set_inner_tetra_id_secret(&mut self, colour: u8, c: [u8; 8]) {
        self.each_inner_tetra(|t| t.add_id_secret(colour, c));
    }

    /// Run `f` over every TETRA front end the scanner placed inside.
    #[cfg(feature = "tea")]
    fn each_inner_tetra(&mut self, mut f: impl FnMut(&mut crate::tetra_nodes::TetraNode)) {
        for slot in &mut self.slots {
            for m in &mut slot.members {
                let ids: Vec<_> = m
                    .graph
                    .order()
                    .filter(|(_, n)| *n == "tetra")
                    .map(|(id, _)| id)
                    .collect();
                for id in ids {
                    if let Some(n) = m.graph.node_mut(id) {
                        if let Some(t) = n
                            .as_any_mut()
                            .and_then(|a| a.downcast_mut::<crate::tetra_nodes::TetraNode>())
                        {
                            f(t);
                        }
                    }
                }
            }
        }
    }

    fn inner_voice(&self, out: &mut Vec<common::Voice>) {
        for slot in &self.slots {
            for m in &slot.members {
                for t in &m.voice {
                    let Some(v) = m.graph.buf(*t).and_then(|p| p.as_voice()) else {
                        continue;
                    };
                    out.extend(v.iter().cloned());
                }
            }
        }
    }

    fn inner_video(&self, out: &mut Vec<common::VideoFrame>) {
        for slot in &self.slots {
            for m in &slot.members {
                for t in &m.video {
                    let Some(v) = m.graph.buf(*t).and_then(|p| p.as_video()) else {
                        continue;
                    };
                    out.extend(v.iter().cloned());
                }
            }
        }
    }

    fn rebuild(&mut self) -> Result<()> {
        if self.rate <= 0.0 {
            return Ok(());
        }
        let d = SourceDetector::new(self.rate, self.input_bw, self.cfg);
        let keep = d.latency_samples();
        self.extractor = Some(SourceExtractor::new(
            self.rate,
            self.center.as_f64(),
            keep,
            self.cfg,
        ));
        self.detector = Some(d);
        self.slots.clear();
        self.pending_sticky = self.sticky.iter().map(|s| s.id).collect();
        self.narrowband = self.query_narrowband();

        // The span-wide decoders, where the span reaches what they are for.
        let mut spec = StreamSpec::iq(self.rate, self.center);
        spec.bandwidth = self.input_bw;
        let c = self.center.as_f64();
        let half = self.input_bw / 2.0;
        self.wide.clear();
        self.exclude.clear();
        let covers = |lo: f64, hi: f64| c - half <= lo && hi <= c + half;
        let modes = (1_089_000_000.0, 1_091_000_000.0);
        if self.rate >= 2_000_000.0 && covers(modes.0, modes.1) {
            self.wide.push(Member::build(
                "mode_s",
                spec,
                NodeSpec::new("mode_s"),
                &self.reg,
            )?);
            self.exclude.push(modes);
        }
        let w = crate::ais_nodes::CHANNEL_WIDTH_HZ;
        let ais = (dsp::ais::CHANNEL_HZ[0] - w, dsp::ais::CHANNEL_HZ[1] + w);
        if covers(ais.0, ais.1) {
            self.wide
                .push(Member::build("ais", spec, NodeSpec::new("ais"), &self.reg)?);
            self.exclude.push(ais);
        }
        // Bluetooth advertising, on whichever of the three channels the span
        // holds. Span-wide for the reason AIS is: the channel is where the
        // standard put it rather than where a spectrogram finds it, and an
        // advertisement is 80 us of a hopping device that may never be heard
        // twice, which is not enough for a source to open around.
        let bw = crate::ble_nodes::CHANNEL_WIDTH_HZ / 2.0;
        for (_, hz) in dsp::ble::ADV_CHANNELS {
            if self.rate >= 4_000_000.0 && covers(hz - bw, hz + bw) {
                self.wide
                    .push(Member::build("ble", spec, NodeSpec::new("ble"), &self.reg)?);
                self.exclude.push((hz - bw, hz + bw));
                break;
            }
        }
        self.apply_band();

        let nominal = StreamSpec::iq(self.cfg.min_rate_hz, self.center);
        self.template = Some(crate::ism_decode_graph(nominal)?);
        Ok(())
    }

    /// The decoders a source of this shape gets.
    ///
    /// The burst front end always. The narrowband frame decoders where the
    /// source is the width of such a channel and its stream is wide enough
    /// to hold one; each decides for itself whether the bits are its own,
    /// since a page has its sync word and a packet its flags and checksum.
    /// A frame decoder that will not build is left out rather than fatal:
    /// the source still has the front end, and one decoder's refusal is not
    /// a reason to stop the receiver.
    /// Ask the registry which decode stages are narrowband channels and what
    /// widths they want. Each is built once with nominal settings and asked
    /// through [`Node::channels`]; the ones that answer with a width are the
    /// candidates [`open`](Self::open) may place. This replaces a hand-kept
    /// table of who fits where with a question put to each front end.
    fn query_narrowband(&self) -> Vec<(&'static str, &'static [f64])> {
        let mut out = Vec::new();
        for desc in self.reg.by_category("decode") {
            let Ok(node) = self
                .reg
                .build(desc.name, &NodeSpec::new(desc.name).settings)
            else {
                continue;
            };
            let ch = node.channels();
            if !ch.is_empty() {
                out.push((desc.name, ch));
            }
        }
        out
    }

    fn open(&self, b: &SourceBlock) -> Result<Slot> {
        let mut spec = StreamSpec::iq(b.rate, Hz(b.center_hz));
        spec.bandwidth = b.bandwidth_hz.min(b.rate);
        if let Some(st) = self.sticky.iter().find(|s| s.id == b.id) {
            let mut m = Member::build(
                st.name,
                spec,
                Self::place(st.name, st.center_hz, st.width_hz),
                &self.reg,
            )?;
            m.channel_hz = st.width_hz;
            return Ok(Slot {
                id: b.id,
                center_hz: b.center_hz,
                members: vec![m],
                heard: true,
                spec,
                signal_hz: b.signal_hz,
                lora_placed: true,
            });
        }
        // The front end is told how strong the detector found the source,
        // so a stream that begins inside a transmission is not read as
        // noise from its first sample to its last.
        let route = NodeSpec::new("burst_route").f("source_snr_db", b.snr_db as f64);
        let mut members = vec![Member::build("burst_route", spec, route, &self.reg)?];
        let hz = b.center_hz as f64;
        // Place every narrowband front whose declared channel fits inside the
        // detected source: the stream must carry the channel (rate over its
        // width), and the source's own width must be within reach of the
        // channel (up to CHANNEL_WIDTH_TOLERANCE times it), so a fat or
        // splattered measurement does not put a 12.5 kHz decoder on a
        // 200 kHz signal, but a channel measured a little wide still places.
        for (name, widths) in &self.narrowband {
            let fits = widths
                .iter()
                .any(|&w| b.rate > w && b.bandwidth_hz <= w * CHANNEL_WIDTH_TOLERANCE);
            if fits {
                if let Ok(mut m) =
                    Member::build(name, spec, Self::place(name, hz, widths[0]), &self.reg)
                {
                    m.channel_hz = widths[0];
                    members.push(m);
                }
            }
        }
        // TETRA is placed by band, not width: unlike the amateur channels its
        // carriers live in licensed downlink allocations, and its hunt
        // correlates continuously, not worth paying on every 433 MHz burst.
        if dsp::tetra::is_downlink_band(hz) && b.rate >= crate::tetra_nodes::MIN_RATE_HZ {
            let w = crate::tetra_nodes::CHANNEL_WIDTH_HZ;
            if let Ok(mut m) = Member::build("tetra", spec, Self::place("tetra", hz, w), &self.reg)
            {
                m.channel_hz = w;
                members.push(m);
            }
        }
        if METER_HZ.contains(&b.bandwidth_hz) {
            let w = crate::wmbus_nodes::CHANNEL_WIDTH_HZ;
            if let Ok(mut m) = Member::build("wmbus", spec, Self::place("wmbus", hz, w), &self.reg)
            {
                m.channel_hz = w;
                members.push(m);
            }
        }
        // Analogue video, by width: a PAL carrier occupies megahertz where
        // everything else here occupies kilohertz, so a source this wide is
        // either video or nothing this receiver reads, and the front end
        // costs an FM demodulation and a slicer. The picture only assembles
        // if the sync pulses are really there, so a wide source that is not
        // video produces no fields rather than a wrong picture.
        if b.bandwidth_hz >= VIDEO_MIN_HZ && b.rate >= VIDEO_MIN_RATE_HZ {
            if let Ok(mut m) =
                Member::build("video", spec, Self::place("video", hz, b.bandwidth_hz), &self.reg)
            {
                m.channel_hz = b.bandwidth_hz;
                members.push(m);
            }
        }
        // LoRa is not placed here. It is the dearest front end to run and
        // a chirp is the one thing the burst front end names reliably, so
        // it is placed when that front end has named one; see `place_lora`.
        for m in &mut members {
            m.source_snr_db = b.snr_db;
        }
        Ok(Slot {
            id: b.id,
            center_hz: b.center_hz,
            members,
            heard: false,
            spec,
            signal_hz: b.signal_hz,
            lora_placed: false,
        })
    }

    /// Place LoRa front ends on a source the burst front end has named a
    /// chirp, one per channel width the source could be, and read them the
    /// source's samples so far from the ring the burst front end kept.
    ///
    /// Placed on the verdict and not on the width because LoRa was the
    /// dearest front end on a busy band, dechirping six spreading factors
    /// on every source over 44 kHz, and on a band of hard-keyed sensors
    /// most sources measure that wide from their splatter. The verdict
    /// costs nothing extra: the burst front end classifies every burst
    /// anyway. What it costs is latency, since a burst is named when it
    /// ends or half a second in, and the ring is what pays that back: a
    /// short packet is read whole from it after the fact, and a long one
    /// is caught up and then followed live.
    fn place_lora(
        &mut self,
        k: usize,
        at_us: u64,
        closed: bool,
        ev: &mut Vec<Event>,
        pk: &mut Vec<Packet>,
        heard: &mut Vec<(&'static str, f64)>,
    ) {
        let slot = &mut self.slots[k];
        slot.lora_placed = true;
        let Some(history) = slot
            .members
            .iter()
            .find(|m| m.router.is_some())
            .map(|m| m.ring.clone())
        else {
            return;
        };
        let hz = slot.center_hz as f64;
        let snr = slot.members.first().map_or(f32::NAN, |m| m.source_snr_db);
        for bw in crate::lora_nodes::bandwidths_for(slot.signal_hz) {
            let Ok(mut m) =
                Member::build("lora", slot.spec, Self::place("lora", hz, bw), &self.reg)
            else {
                continue;
            };
            m.channel_hz = bw;
            m.source_snr_db = snr;
            let before = pk.len();
            // The samples the source has produced so far, in the blocks the
            // live path would have handed over, then the flush if it has
            // already closed.
            for chunk in history.chunks(16_384) {
                ev.extend(m.run(chunk, at_us, pk));
            }
            if closed {
                let quiet = vec![C32::new(0.0, 0.0); (m.flush_s * slot.spec.rate) as usize];
                ev.extend(m.run(&quiet, at_us, pk));
            }
            if pk.len() > before {
                slot.heard = true;
                heard.push((m.name, m.channel_hz));
            }
            slot.members.push(m);
        }
    }

    /// The settings a front end is built with for a channel: where it is
    /// and, for the one that needs telling, how wide.
    fn place(name: &'static str, hz: f64, width_hz: f64) -> NodeSpec {
        match name {
            "lora" => NodeSpec::new(name).f("bandwidth_hz", width_hz),
            "wmbus" => NodeSpec::new(name),
            _ => NodeSpec::new(name).f("channel_hz", hz),
        }
    }

    /// Remember a channel a front end has just read, unless it is one
    /// already kept, and have it cut out from the next block on.
    fn remember(&mut self, name: &'static str, center_hz: f64, width_hz: f64) -> Option<Event> {
        if width_hz <= 0.0 {
            return None;
        }
        let same =
            |s: &Sticky| s.name == name && (s.center_hz - center_hz).abs() <= s.width_hz / 2.0;
        if self.sticky.iter().any(same) {
            return None;
        }
        let id = SourceId(STICKY_ID_BASE + self.sticky.len() as u64);
        self.sticky.push(Sticky {
            id,
            name,
            center_hz,
            width_hz,
        });
        self.pending_sticky.push(id);
        Some(Event::Warning {
            stage: self.label.clone(),
            message: format!(
                "{name} read {:.4} MHz; keeping that channel open for the session",
                center_hz / 1e6
            ),
        })
    }
}

/// Lock a source onto the channel plan when it is plainly on it.
///
/// A source is a measurement: the power centroid of the bins that stood over
/// the floor in its first frames, and the run of them with a margin. On a
/// band with a plan that is the wrong answer to a right question. A TETRA
/// carrier found at 391.1812 MHz, 24.6 kHz wide, is the 391.175 MHz channel
/// seen through a tuner a few parts per million out, and every opening
/// would otherwise measure it slightly differently, cut it out at a
/// different width, and log it at a frequency nobody's plan lists.
///
/// So: within 0.4 of a step of a channel, and between 0.4 and 1.6 of a
/// step wide, a source is the channel, and takes its centre and its width.
/// Anything else is left as measured; a plan says where channels are, not
/// that nothing else transmits. The reach is what a tuner tens of parts per
/// million out needs at UHF: a third of a step left one carrier 8.6 kHz off
/// its channel unlocked while its neighbour 6 kHz off locked.
fn snap_to_raster(s: &mut dsp::Source, (origin, step): (f64, f64), stream_center_hz: f64) {
    let hz = stream_center_hz + s.center_hz;
    let on = origin + ((hz - origin) / step).round() * step;
    let near = (hz - on).abs() <= step * 0.4;
    let width = s.bandwidth_hz();
    let fits = width >= step * 0.4 && width <= step * 1.6;
    if near && fits {
        s.center_hz = on - stream_center_hz;
        s.lo_hz = s.center_hz - step / 2.0;
        s.hi_hz = s.center_hz + step / 2.0;
    }
}

fn now_us() -> u64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_micros() as u64)
        .unwrap_or(0)
}

/// What comes out: everything decoded, and everything heard.
const OUT_PACKETS: usize = 0;
const OUT_VOICE: usize = 1;
const OUT_VIDEO: usize = 2;

impl Node for AutoNode {
    fn name(&self) -> &str {
        &self.label
    }

    fn as_any(&self) -> Option<&dyn std::any::Any> {
        Some(self)
    }

    fn as_any_mut(&mut self) -> Option<&mut dyn std::any::Any> {
        Some(self)
    }

    fn num_inputs(&self) -> usize {
        1
    }

    fn num_outputs(&self) -> usize {
        3
    }

    fn subgraph(&self) -> Option<Topology> {
        self.slots
            .first()
            .and_then(|s| s.members.first())
            .map(|m| m.graph.topology())
            .or_else(|| self.template.as_ref().map(|g| g.topology()))
    }

    fn subgraph_count(&self) -> usize {
        self.slots.len().max(1)
    }

    fn phases(&self) -> Vec<(String, pipeline::cost::Cost)> {
        self.phases
            .iter()
            .map(|(n, r)| (n.clone(), r.cost()))
            .collect()
    }

    fn negotiate(&mut self, inputs: &[PortSpec]) -> Result<Vec<StreamSpec>> {
        let i = &inputs[0];
        if i.spec.kind != PortKind::Iq {
            return Err(common::Error::other(format!("{}: needs IQ", self.label)));
        }
        self.rate = i.spec.rate;
        self.center = i.spec.center;
        self.input_bw = if i.spec.bandwidth > 0.0 {
            i.spec.bandwidth.min(i.spec.rate)
        } else {
            i.spec.rate
        };
        self.rebuild()?;
        // Packets are events in time, not a sampled stream, and each one
        // carries its own frequency and width.
        let mut out = i.spec.with_kind(PortKind::Packets);
        out.rate = 0.0;
        out.bandwidth = 0.0;
        // Speech from whatever front end inside is carrying it, at the
        // vocoder's rate rather than the radio's.
        let mut voice = out.with_kind(PortKind::Voice);
        voice.rate = crate::m17_nodes::VOICE_HZ;
        // Pictures from whatever front end inside is producing them. A field
        // is not a sampled stream, so the rate says nothing and the frame
        // carries its own geometry.
        let video = out.with_kind(PortKind::Video);
        Ok(vec![out, voice, video])
    }

    fn process(
        &mut self,
        inputs: &[&Payload],
        outputs: &mut [Payload],
        c: &mut NodeCtx<'_>,
    ) -> Result<()> {
        self.hits.clear();
        let iq = inputs[0].as_iq().unwrap_or(&[]);
        let (Some(d), Some(e)) = (self.detector.as_mut(), self.extractor.as_mut()) else {
            return Ok(());
        };
        let at_us = now_us();
        let rate = c.inputs[0].spec.rate.max(1.0);
        let out = outputs[OUT_PACKETS].packets_mut();

        // The sources: find, cut out, and read. The span-wide decoders run
        // beside them, in the same fanout, once the blocks are known.
        let mut events: Vec<Event> = Vec::new();
        self.events.clear();
        let c0 = self.center.as_f64();
        let exclude = &self.exclude;
        let spur = self.spur_band;
        let block_s = c.block_seconds;
        let t_detect = Instant::now();
        let raw: Vec<SourceEvent> = d.process(iq).to_vec();
        let detect_us = t_detect.elapsed().as_micros() as u64;
        let others = d
            .live()
            .filter(|s| !spur.is_some_and(|(lo, hi)| (lo..=hi).contains(&(c0 + s.center_hz))))
            .count();
        let sticky = &self.sticky;
        let covered = |hz: f64, w: f64| {
            sticky.iter().any(|s| {
                (s.center_hz - hz).abs() <= s.width_hz / 2.0
                    && w <= s.width_hz * CHANNEL_WIDTH_TOLERANCE
            })
        };
        self.events.extend(raw.iter().filter(|ev| {
            let SourceEvent::Opened(s) = ev else {
                return true;
            };
            let hz = c0 + s.center_hz;
            if exclude.iter().any(|(lo, hi)| (*lo..=*hi).contains(&hz)) {
                return false;
            }
            if covered(hz, s.bandwidth_hz()) {
                return false;
            }
            // The tuner's centre while something else transmits: the
            // offset following that something's envelope.
            !(others > 0 && spur.is_some_and(|(lo, hi)| (lo..=hi).contains(&hz)))
        }));
        // A source plainly on a channel of the plan is that channel: what
        // is cut out, and what is reported, is the channel rather than
        // this frame's measurement of it.
        if let Some(raster) = self.raster {
            for ev in self.events.iter_mut() {
                if let SourceEvent::Opened(s) = ev {
                    snap_to_raster(s, raster, c0);
                }
            }
        }
        // Channels kept from earlier: opened once, at their own width, never
        // closed. The extractor takes them from the current position, so a
        // channel kept before a rebuild starts again where the new span
        // begins.
        let half = self.input_bw / 2.0;
        for id in std::mem::take(&mut self.pending_sticky) {
            let Some(st) = self.sticky.iter().find(|s| s.id == id) else {
                continue;
            };
            let off = st.center_hz - c0;
            if off.abs() + st.width_hz / 2.0 > half {
                continue;
            }
            // Not while the source that earned it is still open: two
            // decoders on one channel are every burst twice in the log, and
            // the new one would start mid-transmission without the header
            // the old one read. It takes over once that source closes.
            let busy = self.slots.iter().any(|sl| {
                sl.id.0 < STICKY_ID_BASE
                    && (sl.center_hz as f64 - st.center_hz).abs() <= st.width_hz / 2.0
            });
            if busy {
                self.pending_sticky.push(id);
                continue;
            }
            self.events.push(SourceEvent::Opened(dsp::Source {
                id,
                lo_hz: off - st.width_hz / 2.0,
                hi_hz: off + st.width_hz / 2.0,
                center_hz: off,
                start_sample: d.position(),
                end_sample: None,
                peak_snr_db: f32::NAN,
                frames: 0,
            }));
        }
        self.blocks.clear();
        let t_extract = Instant::now();
        e.process(iq, &self.events, &mut self.blocks);
        let extract_us = t_extract.elapsed().as_micros() as u64;
        let (bank_feed_us, bank_start_us) = e.take_bank_cost();
        for ev in &self.events {
            if let SourceEvent::Opened(s) = ev {
                if s.id.0 >= STICKY_ID_BASE {
                    continue;
                }
                events.push(Event::Detection {
                    center: Hz((self.center.as_f64() + s.center_hz).max(0.0) as u64),
                    bandwidth: s.bandwidth_hz(),
                    snr_db: s.peak_snr_db,
                    at: s.start_sample as f64 / rate,
                });
            }
        }
        for b in &self.blocks {
            if !self.slots.iter().any(|s| s.id == b.id) {
                let slot = self.open(b)?;
                self.slots.push(slot);
                self.built += 1;
            }
        }

        // Every member of every source is a task of its own, not one task
        // per source: the members share nothing but the block they read, and
        // per-source tasks left an m17 member decoding voice alone on one
        // lane while the others sat finished. The span-wide decoders join
        // the same fanout, since a Mode S correlator over the whole span
        // costs more than any narrowband member.
        let blocks = &self.blocks;
        let wide = &mut self.wide;
        let slots = &mut self.slots;
        let t_fronts = Instant::now();
        let (wide_results, results): (
            Vec<(Vec<Event>, Vec<Packet>, &'static str, u64)>,
            Vec<(
                usize,
                Vec<Event>,
                Vec<Packet>,
                bool,
                Vec<(&'static str, f64)>,
                Vec<(&'static str, u64)>,
            )>,
        ) = rayon::join(
            || {
                wide.par_iter_mut()
                    .map(|m| {
                        let mut pk = Vec::new();
                        let t = Instant::now();
                        let ev = m.run(iq, at_us, &mut pk);
                        (ev, pk, m.name, t.elapsed().as_micros() as u64)
                    })
                    .collect()
            },
            || {
                slots
                    .par_iter_mut()
                    .enumerate()
                    .filter_map(|(k, slot)| {
                        let b = blocks.iter().find(|b| b.id == slot.id)?;
                        let closed = b.state == SourceState::Closed;
                        let per: Vec<(
                            Vec<Event>,
                            Vec<Packet>,
                            Option<(&'static str, f64)>,
                            (&'static str, u64),
                        )> = slot
                            .members
                            .par_iter_mut()
                            .map(|m| {
                                let mut pk = Vec::new();
                                let t = Instant::now();
                                let mut ev = m.run(&b.samples, at_us, &mut pk);
                                if closed {
                                    let quiet =
                                        vec![C32::new(0.0, 0.0); (m.flush_s * b.rate) as usize];
                                    ev.extend(m.run(&quiet, at_us, &mut pk));
                                }
                                let us = t.elapsed().as_micros() as u64;
                                let read = m.router.is_none() && !pk.is_empty();
                                (ev, pk, read.then_some((m.name, m.channel_hz)), (m.name, us))
                            })
                            .collect();
                        let mut ev = Vec::new();
                        let mut pk = Vec::new();
                        let mut heard = Vec::new();
                        let mut spent = Vec::new();
                        for (e2, p2, read, cost) in per {
                            ev.extend(e2);
                            pk.extend(p2);
                            spent.push(cost);
                            if let Some(r) = read {
                                slot.heard = true;
                                heard.push(r);
                            }
                        }
                        // A measurement of a source a front end reads is
                        // not news.
                        if slot.heard {
                            pk.retain(|p| {
                                !(p.measure.is_some()
                                    && matches!(&p.body, PacketBody::Pulses(v) if v.is_empty()))
                            });
                        }
                        let done = matches!(b.state, SourceState::Closed | SourceState::Superseded);
                        if b.state == SourceState::Superseded {
                            // A wider stream for the same transmitter takes over
                            // from its start. Whatever this one made of the sliver it
                            // had is half a burst, and half a burst is not evidence.
                            pk.clear();
                            ev.retain(|e| !matches!(e, Event::Decoded(_)));
                        }
                        Some((k, ev, pk, done, heard, spent))
                    })
                    .collect()
            },
        );
        let fronts_us = t_fronts.elapsed().as_micros() as u64;
        self.phase_sum.clear();
        for (ev, pk, name, us) in wide_results {
            events.extend(ev);
            out.extend(pk);
            *self.phase_sum.entry(name).or_default() += us;
        }
        let mut results = results;
        results.sort_by_key(|(k, ..)| *k);
        for (_, _, _, _, _, spent) in &results {
            for (name, us) in spent {
                *self.phase_sum.entry(name).or_default() += us;
            }
        }
        self.phase("detect", detect_us, block_s);
        self.phase("extract", extract_us, block_s);
        self.phase("extract bank", bank_feed_us, block_s);
        self.phase("extract catch-up", bank_start_us, block_s);
        self.phase("fronts", fronts_us, block_s);
        let sums: Vec<(&'static str, u64)> = self.phase_sum.iter().map(|(n, u)| (*n, *u)).collect();
        for (name, us) in sums {
            self.phase(&format!("{name} cpu"), us, block_s);
        }
        let mut closed = Vec::new();
        for (k, mut ev, mut pk, done, mut heard, _) in results {
            let center = Hz(self.slots[k].center_hz);
            if !self.slots[k].lora_placed && self.slots[k].members.iter().any(|m| m.chirp_seen) {
                self.place_lora(k, at_us, done, &mut ev, &mut pk, &mut heard);
            }
            for (name, width) in &heard {
                if let Some(e) = self.remember(name, center.as_f64(), *width) {
                    c.emit(e);
                }
            }
            // Latch. Every front end whose channel the source could be was
            // built for it and asked; the one that read a frame has
            // answered what the source is, and from here it alone reads
            // it. The others were each a decoder's worth of work per block
            // and, for a pager or a packet channel, a second row saying
            // the same burst was nothing.
            if !heard.is_empty() && self.slots[k].members.len() > 1 {
                // Two LoRa channels an octave apart share a sweep rate two
                // spreading factors apart, so a 250 kHz packet also reads,
                // as something, through a 125 kHz demodulator seeing half of
                // it. The narrower one cannot read a chirp of the wider, so
                // when both read the wider one is the channel.
                let widest = heard
                    .iter()
                    .filter(|(n, _)| *n == "lora")
                    .map(|(_, w)| *w)
                    .fold(0.0, f64::max);
                self.slots[k].members.retain(|m| {
                    heard
                        .iter()
                        .any(|(n, w)| *n == m.name && (*n != "lora" || *w >= widest))
                });
            }
            for e in ev {
                if matches!(e, Event::Decoded(_)) {
                    self.hits.push((center, e.clone()));
                }
                events.push(e);
            }
            // A cell's identity, once, per channel, whatever the decoders
            // that read it have been through since.
            let seen = self.announced.entry(center.0).or_default();
            out.extend(pk.into_iter().filter(|p| {
                let PacketBody::Frame(bytes) = &p.body else {
                    return true;
                };
                let Some(key) = decode::tetra::Event::identity_key(bytes) else {
                    return true;
                };
                if seen.contains(&key) {
                    return false;
                }
                seen.push(key);
                true
            }));
            if done {
                closed.push(self.slots[k].id);
            }
        }
        self.slots.retain(|s| !closed.contains(&s.id));
        self.inner_voice(outputs[OUT_VOICE].voice_mut());
        self.inner_video(outputs[OUT_VIDEO].video_mut());

        for e in events {
            match &e {
                // Warnings are per burst and per source; across a whole band
                // they arrive in the thousands.
                Event::Warning { .. } => {}
                Event::Decoded(_) => {
                    if !self.hits.iter().any(|(_, h)| std::ptr::eq(h, &e)) {
                        self.hits.push((self.center, e.clone()));
                    }
                    c.emit(e);
                }
                _ => c.emit(e),
            }
        }
        Ok(())
    }

    fn reset(&mut self) {
        if let Some(d) = &mut self.detector {
            d.reset();
        }
        if let Some(e) = &mut self.extractor {
            e.reset();
        }
        self.slots.clear();
        for m in &mut self.wide {
            m.graph.reset();
        }
        self.hits.clear();
    }

    /// The detector's knobs, then the burst front end's.
    fn params(&self) -> Vec<Param> {
        let mut p = vec![
            Param::float("open_db", self.cfg.open_db as f64, 3.0..=40.0)
                .unit("dB")
                .label("SNR that opens a source"),
            Param::float("close_db", self.cfg.close_db as f64, 1.0..=40.0)
                .unit("dB")
                .label("SNR that closes it again"),
            Param::float("hang_ms", self.cfg.hang_us as f64 / 1e3, 1.0..=2_000.0)
                .unit("ms")
                .label("Silence that closes a source")
                .log(),
            Param::float("bin_hz", self.cfg.bin_hz, 100.0..=100_000.0)
                .unit("Hz")
                .label("Spectral resolution")
                .log(),
        ];
        if let Some(t) = &self.template {
            p.extend(t.topology().nodes.into_iter().flat_map(|n| n.params));
        }
        p
    }

    fn set_param(&mut self, name: &str, v: ParamValue) -> Result<()> {
        let f = v.as_f64().unwrap_or_default();
        match name {
            "open_db" => {
                self.cfg.open_db = f as f32;
                self.cfg.close_db = self.cfg.close_db.min(self.cfg.open_db - 1.0);
            }
            "close_db" => self.cfg.close_db = (f as f32).min(self.cfg.open_db - 1.0),
            "hang_ms" => self.cfg.hang_us = (f * 1e3).max(1.0) as u32,
            "bin_hz" => {
                self.cfg.bin_hz = f.max(1.0);
                return self.rebuild();
            }
            "raster_hz" => {
                let origin = self.raster.map(|(o, _)| o).unwrap_or(0.0);
                self.set_raster((f > 0.0).then_some((origin, f)));
            }
            "raster_origin_hz" => {
                if let Some((_, step)) = self.raster {
                    self.raster = Some((f, step));
                }
            }
            _ => {
                // A front end's own knob: set on the template, so sources
                // that open later start with it, and on every running copy.
                let mut found = false;
                let mut err = None;
                let mut apply = |g: &mut Graph| {
                    let ids: Vec<_> = g.topology().nodes.iter().map(|n| n.id).collect();
                    for id in ids {
                        let Some(node) = g.node_mut(id) else { continue };
                        if !node.params().iter().any(|p| p.name == name) {
                            continue;
                        }
                        found = true;
                        if let Err(e) = node.set_param(name, v.clone()) {
                            err = Some(e);
                        }
                    }
                };
                if let Some(t) = &mut self.template {
                    apply(t);
                }
                for s in &mut self.slots {
                    for m in &mut s.members {
                        apply(&mut m.graph);
                    }
                }
                return match err {
                    Some(e) => Err(e),
                    None if found => Ok(()),
                    None => Err(common::Error::other(format!(
                        "{}: unknown parameter {name:?}",
                        self.label
                    ))),
                };
            }
        }
        // Thresholds and timings: read every frame, so the detector is
        // built again with them and nothing else changes.
        if let Some(d) = &mut self.detector {
            *d = SourceDetector::new(self.rate, self.input_bw, self.cfg);
            self.apply_band();
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use pipeline::node::Node;

    fn spec(rate: f64, center: Hz) -> PortSpec {
        PortSpec {
            spec: StreamSpec::iq(rate, center),
            latency: 0,
        }
    }

    #[test]
    fn the_node_turns_iq_into_packets() {
        let mut n = AutoNode::new("auto", SourceConfig::default());
        let out = Node::negotiate(&mut n, &[spec(2_400_000.0, Hz::mhz(433))]).unwrap();
        assert_eq!(out[0].kind, PortKind::Packets);
        assert!(n.wide().is_empty(), "nothing span-wide belongs at 433 MHz");
        assert!(
            Node::subgraph(&n).is_some(),
            "the burst front end is shown before any source"
        );
    }

    /// Noise with a keyed carrier `offset` hertz up from the centre for the
    /// last stretch of it.
    fn keyed(rate: f64, offset: f64) -> Vec<C32> {
        let mut seed = 0x51u64;
        let mut iq: Vec<C32> = (0..600_000)
            .map(|_| {
                seed ^= seed << 13;
                seed ^= seed >> 7;
                seed ^= seed << 17;
                let a = (seed >> 11) as f32 / (1u64 << 53) as f32 - 0.5;
                seed ^= seed << 13;
                seed ^= seed >> 7;
                seed ^= seed << 17;
                let b = (seed >> 11) as f32 / (1u64 << 53) as f32 - 0.5;
                C32::new(a * 0.05, b * 0.05)
            })
            .collect();
        for i in 0..100_000usize {
            if (i / 500) % 2 == 0 {
                let ph = std::f64::consts::TAU * offset * i as f64 / rate;
                iq[300_000 + i] += C32::new(0.3 * ph.cos() as f32, 0.3 * ph.sin() as f32);
            }
        }
        iq
    }

    /// Where the node said sources opened, as offsets from the centre.
    fn openings(n: &mut AutoNode, rate: f64, center: Hz, iq: &[C32]) -> Vec<f64> {
        let ins = [spec(rate, center)];
        let mut opened = Vec::new();
        for block in iq.chunks(16_384) {
            let input = Payload::Iq(block.to_vec());
            let mut out = [
                Payload::Packets(Vec::new()),
                Payload::Voice(Vec::new()),
                Payload::Video(Vec::new()),
            ];
            let (mut events, mut tags) = (Vec::new(), Vec::new());
            let mut ctx = NodeCtx::new(0, &ins, &[], &mut events, &mut tags);
            Node::process(n, &[&input], &mut out, &mut ctx).unwrap();
            opened.extend(events.iter().filter_map(|e| match e {
                Event::Detection { center: c, .. } => Some(c.as_f64() - center.as_f64()),
                _ => None,
            }));
        }
        opened
    }

    /// A source megahertz wide is either analogue video or nothing this
    /// receiver reads, so the front end is placed by width the way TETRA is
    /// placed by band. The picture only assembles if the sync pulses are
    /// really there, so a wide source that is not video costs a demodulation
    /// and produces nothing.
    #[test]
    fn a_source_wide_enough_to_be_a_picture_gets_a_video_front_end() {
        let placed = |bandwidth_hz: f64, rate: f64| -> bool {
            bandwidth_hz >= VIDEO_MIN_HZ && rate >= VIDEO_MIN_RATE_HZ
        };
        // A 5.8 GHz video carrier at 20 MS/s: the one measured here was
        // 4.6 MHz wide.
        assert!(placed(4.6e6, 20e6));
        // BLE is the widest thing here that is not video, at 2 MHz.
        assert!(!placed(2e6, 20e6));
        // And a wide source on a span too slow to hold a picture is not one:
        // PAL luma reaches 5 MHz with the subcarrier at 4.43.
        assert!(!placed(4.6e6, 8e6));
    }

    #[test]
    fn a_source_near_a_channel_of_the_plan_is_that_channel() {
        // A source is a measurement, and a measurement of a channel that a
        // plan lists is the channel seen through a tuner a few parts per
        // million out. Locked, it is cut out and logged as the channel;
        // left as measured it is a different frequency every time it opens.
        let rate = 1_000_000.0;
        let center = Hz::mhz(434);
        let iq = keyed(rate, 356_000.0);
        let mut plain = AutoNode::new("auto", SourceConfig::default());
        Node::negotiate(&mut plain, &[spec(rate, center)]).unwrap();
        let measured = openings(&mut plain, rate, center, &iq);
        assert!(
            measured.iter().any(|o| (o - 356_000.0).abs() < 5_000.0),
            "{measured:?}"
        );
        assert!(
            !measured.iter().any(|o| (o - 350_000.0).abs() < 1.0),
            "not on the grid yet"
        );

        let mut planned = AutoNode::new("auto", SourceConfig::default());
        planned.set_raster(Some((0.0, 25_000.0)));
        Node::negotiate(&mut planned, &[spec(rate, center)]).unwrap();
        let locked = openings(&mut planned, rate, center, &iq);
        assert!(
            locked.iter().any(|o| (o - 350_000.0).abs() < 1.0),
            "{locked:?}"
        );

        // Half a channel off the grid is not on it, and stays as measured.
        let iq = keyed(rate, 362_500.0);
        let mut planned = AutoNode::new("auto", SourceConfig::default());
        planned.set_raster(Some((0.0, 25_000.0)));
        Node::negotiate(&mut planned, &[spec(rate, center)]).unwrap();
        let between = openings(&mut planned, rate, center, &iq);
        assert!(
            between.iter().any(|o| (o - 362_500.0).abs() < 5_000.0),
            "{between:?}"
        );
        assert!(!between
            .iter()
            .any(|o| (o - 350_000.0).abs() < 1.0 || (o - 375_000.0).abs() < 1.0));
    }

    #[test]
    fn nothing_opens_on_the_tuner_s_own_centre() {
        // A direct-conversion receiver's offset follows a strong signal's
        // envelope, and that is a burst at the centre for as long as the
        // signal lasts. With the centre declared, a burst there while
        // another source is open is not a source.
        let rate = 1_000_000.0;
        let center = Hz::mhz(434);
        let mut n = AutoNode::new("auto", SourceConfig::default());
        n.set_spur(Some(center.as_f64()));
        Node::negotiate(&mut n, &[spec(rate, center)]).unwrap();
        let mut seed = 0x51u64;
        let mut iq: Vec<C32> = (0..600_000)
            .map(|_| {
                seed ^= seed << 13;
                seed ^= seed >> 7;
                seed ^= seed << 17;
                let a = (seed >> 11) as f32 / (1u64 << 53) as f32 - 0.5;
                seed ^= seed << 13;
                seed ^= seed >> 7;
                seed ^= seed << 17;
                let b = (seed >> 11) as f32 / (1u64 << 53) as f32 - 0.5;
                C32::new(a * 0.05, b * 0.05)
            })
            .collect();
        // 100 ms keyed at the centre, and the same 350 kHz up: far enough
        // that the two are not taken for the tones of one transmitter.
        for i in 0..100_000usize {
            let on = (i / 500) % 2 == 0;
            if on {
                iq[300_000 + i] += C32::new(0.3, 0.0);
                let ph = std::f64::consts::TAU * 350_000.0 * i as f64 / rate;
                iq[300_000 + i] += C32::new(0.3 * ph.cos() as f32, 0.3 * ph.sin() as f32);
            }
        }
        let ins = [spec(rate, center)];
        let mut opened = Vec::new();
        for block in iq.chunks(16_384) {
            let input = Payload::Iq(block.to_vec());
            // Packets and speech: the node has a port for each.
            let mut out = [
                Payload::Packets(Vec::new()),
                Payload::Voice(Vec::new()),
                Payload::Video(Vec::new()),
            ];
            let (mut events, mut tags) = (Vec::new(), Vec::new());
            let mut ctx = NodeCtx::new(0, &ins, &[], &mut events, &mut tags);
            Node::process(&mut n, &[&input], &mut out, &mut ctx).unwrap();
            opened.extend(events.iter().filter_map(|e| match e {
                Event::Detection { center: c, .. } => Some(c.as_f64() - center.as_f64()),
                _ => None,
            }));
        }
        assert!(
            opened.iter().any(|o| (o - 350_000.0).abs() < 10_000.0),
            "the real one opened: {opened:?}"
        );
        assert!(
            !opened.iter().any(|o| o.abs() < 10_000.0),
            "the spur opened: {opened:?}"
        );
    }

    #[test]
    fn a_source_at_the_extraction_floor_still_gets_the_channel_decoders() {
        // A clean 12.5 kHz transmission measures a few kilohertz across at
        // the detector's 20 dB extent, so its extraction lands on the
        // 25 kHz floor. That is wide enough to hold the channel, and every
        // narrowband decoder has to be built for it.
        let mut n = AutoNode::new("auto", SourceConfig::default());
        Node::negotiate(&mut n, &[spec(2_400_000.0, Hz::mhz(433))]).unwrap();
        let b = SourceBlock {
            id: SourceId(1),
            state: SourceState::Opened,
            center_hz: 433_475_000,
            // The two-bin minimum the detector can report, which is what a
            // clean 12.5 kHz channel measures at its 20 dB extent.
            bandwidth_hz: 4_000.0,
            signal_hz: 4_000.0,
            rate: n.cfg.min_rate_hz,
            start_sample: 0,
            snr_db: 20.0,
            samples: Vec::new(),
        };
        let slot = n.open(&b).unwrap();
        let names: Vec<&str> = slot.members.iter().map(|m| m.name).collect();
        assert!(names.contains(&"m17"), "{names:?}");
        assert!(names.contains(&"pocsag"), "{names:?}");
    }

    #[test]
    fn the_span_wide_decoders_run_where_the_span_reaches_them() {
        let mut n = AutoNode::new("auto", SourceConfig::default());
        Node::negotiate(&mut n, &[spec(2_400_000.0, Hz::mhz(1090))]).unwrap();
        assert_eq!(n.wide(), ["mode_s"]);
        Node::negotiate(&mut n, &[spec(2_400_000.0, Hz::mhz(162))]).unwrap();
        assert_eq!(n.wide(), ["ais"]);
        Node::negotiate(&mut n, &[spec(250_000.0, Hz::mhz(1090))]).unwrap();
        assert!(n.wide().is_empty(), "Mode S needs 2 MS/s");
        // Bluetooth advertising is one of these and not a scanner block:
        // the three channels are where the standard put them, and this is
        // the only thing that places the front end on them.
        Node::negotiate(&mut n, &[spec(20_000_000.0, Hz::mhz(2426))]).unwrap();
        assert_eq!(n.wide(), ["ble"]);
        Node::negotiate(&mut n, &[spec(20_000_000.0, Hz::mhz(2450))]).unwrap();
        assert!(
            n.wide().is_empty(),
            "no advertising channel inside that span"
        );
        Node::negotiate(&mut n, &[spec(2_400_000.0, Hz::mhz(2426))]).unwrap();
        assert!(n.wide().is_empty(), "BLE needs 4 MS/s");
    }
}
