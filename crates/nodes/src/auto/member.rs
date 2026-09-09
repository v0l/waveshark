//! One decoder over one stream: its graph, the ring of samples behind it,
//! and the level every packet leaves with.

use common::{Packet, Result, C32};
use pipeline::event::Event;
use pipeline::port::{PortKind, StreamSpec};
use pipeline::registry::{Registry, Settings};
use pipeline::{Graph, Out};
use std::collections::VecDeque;

use crate::protocol::{Placed, Protocol};
use crate::{build_chain, NodeSpec};

/// How often a transmission that never ends is reported, in seconds.
///
/// A base station carrier is on all day. The burst front end cuts it into
/// pieces of half a second to have something to measure, and a row per
/// piece would be a list of nothing else. One when it is found, then one
/// every so often to say it is still there, is what "which channels are
/// busy" needs.
const REPORT_S: f64 = 5.0;

/// One decoder over one stream: a graph, and where its packets come out.
pub(super) struct Member {
    pub(super) name: &'static str,
    /// What the graph reads, or None for the burst front end, which is
    /// not a protocol but the thing that names one.
    pub(super) protocol: Option<&'static dyn Protocol>,
    pub(super) graph: Graph,
    pub(super) pulses: Vec<Out>,
    pub(super) frames: Vec<Out>,
    /// Front ends that build their own packets, because what they produce is
    /// more than bytes: an M17 voice stream carries speech beside them.
    pub(super) packets: Vec<Out>,
    /// Front ends that carry speech, on the port it travels out on.
    pub(super) voice: Vec<Out>,
    /// Pictures, for a front end inside that produces them. Same arrangement
    /// as `voice` and for the same reason: what a source turns out to be
    /// decides what leaves the node, and a video front end placed on a source
    /// must have somewhere to publish to without the auto node knowing which
    /// front end it was.
    pub(super) video: Vec<Out>,
    /// The band this front end owns, closed to the detector, when it owns
    /// one. A span-wide front end owns the channel the standard put it on
    /// from the moment the span reaches it; the video front end owns the
    /// whole span, but only while it is reading a picture off it.
    pub(super) band: Option<(f64, f64)>,
    /// The band a span-wide front end was placed on: what it owns once it
    /// says it is reading. A claim is widened to this, because a decoder
    /// knows the stream it was handed and not the span it was cut from, and
    /// after `Video::chain` band-limits 20 MS/s down to 10 the camera would
    /// otherwise claim half of its own carrier's span and leave the skirts
    /// to be opened as sources.
    pub(super) placed_band: Option<(f64, f64)>,
    /// The burst front end inside, when this is it: its packets are read
    /// from what it measured rather than from its port, so every burst
    /// leaves with its measurement, and a burst no front end reads leaves
    /// as a packet of nothing but the measurement.
    pub(super) router: Option<pipeline::NodeId>,
    /// The SNR the detector measured for the source this member reads, so a
    /// frame decoder that measures no level of its own still reports the
    /// level of the transmission it came from rather than nothing. NaN
    /// until [`Slot::open`] sets it from the source block.
    pub(super) source_snr_db: f32,
    /// Peak mean-square power of the extracted stream since the last frame
    /// left, held across the blocks a transmission spans. A frame decoder
    /// reads bits and reports no level, but the samples it read have one,
    /// and the loudest block of a page is the page's RSSI. Reset when a
    /// frame is emitted, so the tail silence after it does not drag the
    /// next transmission's level down.
    pub(super) peak_pow: f32,
    /// When a transmission still going was last reported, in seconds of
    /// stream, so it is reported every [`REPORT_S`] rather than every piece.
    pub(super) last_report_s: Option<f64>,
    /// Width of the channel this front end was placed for, in hertz, or
    /// zero for one that measures the burst rather than reading a channel.
    pub(super) channel_hz: f64,
    /// Silence to feed after the source closes, in seconds: the most any
    /// node in the graph asked for through [`Node::flush_s`]. A pager
    /// transmission has no closing flag and a DMR over that lost its
    /// terminator ends on a second and a half of silence, so dropping the
    /// decoder when the source closes drops the page or the over with it.
    pub(super) flush_s: f64,
    /// The source's samples since the last packet left, up to
    /// [`RING_MAX_S`], so a packet from a front end that did not cut its
    /// own samples out still leaves with the stream it was read from. A
    /// packet in the log without its samples cannot be decoded again by
    /// anything written later, and a row that says only what one decoder
    /// made of a burst is an event, not a packet.
    pub(super) ring: Vec<C32>,
    /// Quietest block power seen, rising slowly, so a source whose level the
    /// detector did not measure (a channel kept open for the session) still
    /// reports a signal to noise ratio on its packets.
    pub(super) noise_pow: f32,
    /// What the burst front end inside has named the bursts of this source,
    /// each once. What places the decoders that wait for a verdict, late,
    /// fed from `ring`.
    /// What the burst front end inside has named the bursts of this source,
    /// each once, with how wide it measured them. The width is what places
    /// a decoder on the right channel of a protocol keyed at several: a
    /// 125 kHz LoRa packet inside a channel remembered at 250 kHz is a
    /// chirp either way, and only its width says which demodulator reads
    /// it.
    pub(super) verdicts: Vec<(dsp::Modulation, f64)>,
    /// Where in its watch cycle a sampled span-wide front end is, in
    /// samples, and how long since it last read anything. See
    /// [`Protocol::watch`].
    pub(super) watched: usize,
    pub(super) since_read: f64,
    /// Whether this front end can produce a packet at all, and so whether
    /// the ring and the levels behind it are worth keeping.
    ///
    /// A picture is not a packet: the video front end publishes fields and
    /// nothing else, so every sample copied into its ring is copied to be
    /// thrown away. At 20 MS/s the ring is [`RING_MAX_S`] seconds of complex
    /// samples, which is 320 MB held and rewritten for a member that has no
    /// packet to hang it on, and the memory traffic was most of what the
    /// video front end appeared to cost.
    keeps_samples: bool,
    /// Samples still to be read before the live ones: what a decoder placed
    /// late has to catch up on, and every block that arrives while it does.
    /// Read a bounded amount a block. Two seconds of history through six
    /// spreading factors of dechirp, in the one block a chirp was named
    /// in, held the radio thread for several block times and the device
    /// dropped samples; spread over a few blocks it costs a few times a
    /// block each and nothing is lost.
    pub(super) backlog: VecDeque<C32>,
}

/// How many blocks' worth of backlog a member reads per block, over the
/// live block itself. Three catches up on two seconds of history in under
/// a second at a cost the fanout absorbs; ten would be the old stall in
/// slower motion.
const CATCHUP_RATIO: usize = 3;

/// The least a draining member reads per call, for the blocks after its
/// source closed, when there is no live block to scale from.
const CATCHUP_MIN: usize = 16_384;

/// Longest run of samples kept behind a packet, in seconds.
const RING_MAX_S: f64 = 2.0;

/// Most verdicts kept for one source. A source producing more distinct
/// modulations and widths than this is a channel with a lot in it, and the
/// decoders the first few placed are what it gets.
const VERDICTS_MAX: usize = 8;

/// How much of that ring a packet leaves with.
///
/// The ring is long because a front end may need to look back; a packet only
/// needs the transmission it was read from. A quarter of a second holds any
/// burst this receiver decodes, including a LoRa packet at the highest
/// spreading factor over the narrowest bandwidth.
const IQ_KEEP_S: f64 = 0.25;

impl Member {
    /// The burst front end for a stream.
    pub(super) fn classifier(spec: StreamSpec, settings: NodeSpec, reg: &Registry) -> Result<Self> {
        Self::build("burst_route", None, spec, vec![settings], reg)
    }

    /// A protocol's decoder for a placed channel.
    /// A protocol's decoder for a placed channel. `extra` is what the
    /// decoder is told beyond its channel: where its stream sits in the
    /// span, and whatever the channel's asker said it needs.
    pub(super) fn place(
        p: &'static dyn Protocol,
        spec: StreamSpec,
        at: Placed,
        extra: &Settings,
        reg: &Registry,
    ) -> Result<Self> {
        Self::place_behind(p, spec, at, &[], extra, reg)
    }

    /// The same, with stages in front of the protocol's own: what the span
    /// is cut down with before a span-wide decoder reads it.
    pub(super) fn place_behind(
        p: &'static dyn Protocol,
        spec: StreamSpec,
        at: Placed,
        pre: &[NodeSpec],
        extra: &Settings,
        reg: &Registry,
    ) -> Result<Self> {
        let mut chain = pre.to_vec();
        chain.extend(p.chain(at));
        if let Some(last) = chain.last_mut() {
            last.settings.extend(extra.iter().map(|(k, v)| (k.clone(), v.clone())));
        }
        let mut m = Self::build(p.id(), Some(p), spec, chain, reg)?;
        m.channel_hz = at.width_hz;
        m.source_snr_db = at.snr_db;
        Ok(m)
    }

    pub(super) fn build(
        name: &'static str,
        protocol: Option<&'static dyn Protocol>,
        spec: StreamSpec,
        chain: Vec<NodeSpec>,
        reg: &Registry,
    ) -> Result<Self> {
        let graph = build_chain(spec, &chain, reg)?;
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
        let (pulses_empty, frames_empty, packets_empty) =
            (pulses.is_empty(), frames.is_empty(), packets.is_empty());
        Ok(Self {
            name,
            protocol,
            graph,
            pulses,
            frames,
            packets,
            voice,
            video,
            band: None,
            placed_band: None,
            router,
            source_snr_db: f32::NAN,
            peak_pow: 0.0,
            last_report_s: None,
            channel_hz: 0.0,
            flush_s,
            ring: Vec::new(),
            noise_pow: f32::NAN,
            verdicts: Vec::new(),
            backlog: VecDeque::new(),
            watched: 0,
            since_read: f64::INFINITY,
            keeps_samples: !pulses_empty || !frames_empty || !packets_empty || router.is_some(),
        })
    }

    /// Whether this front end wants this block, and account for it either
    /// way.
    ///
    /// A protocol that watches everything always does. One that samples
    /// reads a window of every cycle until it decodes something, and then
    /// reads everything until it has been quiet for its hold.
    pub(super) fn wants(&mut self, samples: usize, rate: f64) -> bool {
        let watch = match self.protocol.map(|p| p.watch()) {
            Some(w) => w,
            None => return true,
        };
        let (on_s, every_s, hold_s) = match watch {
            crate::protocol::Watch::Everything => return true,
            crate::protocol::Watch::Sampled { on_s, every_s, hold_s } => (on_s, every_s, hold_s),
        };
        let block_s = samples as f64 / rate.max(1.0);
        self.since_read += block_s;
        if self.since_read <= hold_s {
            return true;
        }
        let period = (every_s * rate) as usize;
        let on = (on_s * rate) as usize;
        let at = self.watched % period.max(1);
        self.watched = self.watched.wrapping_add(samples);
        at < on
    }

    /// Say that this front end read something, which puts it back on the
    /// whole stream for its hold.
    pub(super) fn read_something(&mut self) {
        self.since_read = 0.0;
    }

    /// Give a member placed late the samples it missed. They are read a
    /// bounded amount a block from then on, ahead of whatever arrives.
    pub(super) fn catch_up(&mut self, history: &[C32]) {
        self.backlog.extend(history.iter().copied());
    }

    /// Whether there is still history to read before the live stream.
    pub(super) fn behind(&self) -> bool {
        !self.backlog.is_empty()
    }

    /// Run one block through and collect what came out as packets. With a
    /// backlog, the block joins the queue and a bounded amount of the
    /// queue is read instead.
    pub(super) fn run(&mut self, iq: &[C32], at_us: u64, out: &mut Vec<Packet>) -> Vec<Event> {
        if self.backlog.is_empty() {
            return self.run_now(iq, at_us, out);
        }
        self.backlog.extend(iq.iter().copied());
        let budget = (iq.len().max(CATCHUP_MIN) * CATCHUP_RATIO).min(self.backlog.len());
        let take: Vec<C32> = self.backlog.drain(..budget).collect();
        let mut events = Vec::new();
        for chunk in take.chunks(16_384) {
            events.extend(self.run_now(chunk, at_us, out));
        }
        events
    }

    fn run_now(&mut self, iq: &[C32], at_us: u64, out: &mut Vec<Packet>) -> Vec<Event> {
        let rate = self.graph.input_spec().rate;
        if !iq.is_empty() && self.keeps_samples {
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
                // The end of the ring, not all of it. A packet arrives when
                // its burst ends, so the samples worth carrying are the last
                // ones; the rest is however long the channel was quiet
                // before it. Two seconds of a 2.4 MS/s source is sixteen
                // megabytes a packet in the log, which is how a day's log
                // reached 122 GB.
                let keep = (IQ_KEEP_S * rate) as usize;
                let from = self.ring.len().saturating_sub(keep.max(1));
                p.iq = Some(std::sync::Arc::new(common::IqBurst {
                    rate,
                    center_hz: self.graph.input_spec().center.0,
                    samples: self.ring[from..].to_vec(),
                }));
                attached = true;
            }
            let snr = if self.noise_pow > 0.0 {
                10.0 * (self.peak_pow / self.noise_pow).max(1.0).log10()
            } else {
                f32::NAN
            };
            p.fill_level(10.0 * self.peak_pow.max(1e-20).log10(), snr);
        }
        if attached {
            self.ring.clear();
        }
        events
    }

    pub(super) fn run_graph(
        &mut self,
        iq: &[C32],
        at_us: u64,
        out: &mut Vec<Packet>,
    ) -> Vec<Event> {
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
                // The same modulation at a clearly different width is a
                // second verdict, not a repeat of the first: that is what
                // two LoRa networks sharing a frequency look like.
                let w = b.class.features.bandwidth_hz as f64;
                let same = |(m, v): &(dsp::Modulation, f64)| {
                    *m == b.class.modulation
                        && (*v <= 0.0 || w <= 0.0 || (w - v).abs() <= v.max(w) * 0.4)
                };
                if !self.verdicts.iter().any(same) && self.verdicts.len() < VERDICTS_MAX {
                    self.verdicts.push((b.class.modulation, w));
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
                    if b.routed_to != common::FrontEnd::None
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
                    // The level is filled from the source's own in `run`,
                    // which is where the samples are; the classifier measures
                    // the burst against the noise it found and reports
                    // nothing when it never found any.
                    let mut pkt = Packet::of_pulses(
                        at_us,
                        bandwidth_hz,
                        common::Package {
                            pulses: Vec::new(),
                            snr_db: if b.class.features.snr_db > 0.0 {
                                b.class.features.snr_db
                            } else {
                                f32::NAN
                            },
                            rssi_dbfs: f32::NAN,
                            start_sample: b.start_sample,
                            center_hz,
                            modulation: None,
                        },
                    );
                    pkt.measure = Some(m);
                    pkt.iq = iq.clone();
                    out.push(pkt);
                    continue;
                }
                for p in &b.packages {
                    let mut pkg = p.clone();
                    pkg.center_hz = center_hz;
                    let mut pkt = Packet::of_pulses(at_us, bandwidth_hz, pkg);
                    pkt.measure = Some(m.clone());
                    pkt.iq = iq.clone();
                    out.push(pkt);
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
                out.push(Packet::of_pulses(
                    at_us,
                    spec.map(|s| s.bandwidth as u32).unwrap_or(0),
                    p.clone(),
                ));
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
                p.fill_level(10.0 * self.peak_pow.max(1e-20).log10(), self.source_snr_db);
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
                // What the front end measured, where it measured anything:
                // it read the channel this frame came off, and the source's
                // own level is of the whole extraction. The fills are for a
                // front end that has not been taught to measure yet.
                let mut f = f.clone();
                if f.center_hz == 0 {
                    f.center_hz = spec.map(|s| s.center.0).unwrap_or(0);
                }
                let mut pkt =
                    Packet::of_frame(at_us, spec.map(|s| s.bandwidth as u32).unwrap_or(0), f);
                pkt.fill_level(10.0 * self.peak_pow.max(1e-20).log10(), self.source_snr_db);
                out.push(pkt);
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
pub(super) fn taps(g: &Graph, kind: PortKind) -> Vec<Out> {
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

#[cfg(test)]
mod tests {
    use super::*;
    use common::Hz;

    /// A member handed two seconds of history reads a few blocks' worth of
    /// it per block, not all of it at once: reading it at once is what held
    /// the radio thread and dropped samples.
    #[test]
    fn history_is_read_a_bounded_amount_a_block() {
        let rate = 250_000.0;
        let spec = StreamSpec::iq(rate, Hz::mhz(868));
        let reg = crate::registry();
        let mut m = Member::classifier(spec, NodeSpec::new("burst_route"), &reg).unwrap();
        let history = vec![C32::new(0.0, 0.0); (2.0 * rate) as usize];
        m.catch_up(&history);
        assert!(m.behind());
        let block = vec![C32::new(0.0, 0.0); 13_600];
        let mut out = Vec::new();
        let before = m.backlog.len();
        m.run(&block, 0, &mut out);
        let read = before + block.len() - m.backlog.len();
        assert_eq!(read, CATCHUP_MIN * CATCHUP_RATIO);
        // And it does catch up, block by block, until the live stream is
        // read directly again.
        let mut blocks = 0;
        while m.behind() {
            m.run(&block, 0, &mut out);
            blocks += 1;
        }
        assert!((8..=16).contains(&blocks), "caught up in {blocks} blocks");
    }
}
