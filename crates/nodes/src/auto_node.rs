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
use pipeline::node::{NodeCtx, PortSpec, Simple};
use pipeline::param::{Param, ParamValue};
use pipeline::port::{Payload, PortKind, StreamSpec};
use pipeline::registry::Registry;
use pipeline::{Graph, Out};
use rayon::prelude::*;

use crate::{build_chain, NodeSpec};

/// Widths a source can have and still be a narrowband voice or data
/// channel worth trying the frame decoders on, in hertz.
const NARROW_HZ: std::ops::RangeInclusive<f64> = 6_000.0..=40_000.0;

/// Widths a meter transmission has: 100 kchip/s keyed 50 kHz either way,
/// with what the extraction adds around it.
const METER_HZ: std::ops::RangeInclusive<f64> = 60_000.0..=450_000.0;

/// Silence fed to a source's decoders after its last block, in seconds.
///
/// A pager transmission has no closing flag: it ends when the batch that
/// should follow is not there, and the demodulator only says so once it has
/// heard enough silence to be sure. Dropping the decoder the moment the
/// source closes would drop the page with it.
const FLUSH_S: f64 = 0.25;

/// One decoder over one stream: a graph, and where its packets come out.
struct Member {
    name: &'static str,
    graph: Graph,
    pulses: Vec<Out>,
    frames: Vec<Out>,
    /// The burst front end inside, when this is it: its packets are read
    /// from what it measured rather than from its port, so every burst
    /// leaves with its measurement, and a burst no front end reads leaves
    /// as a packet of nothing but the measurement.
    router: Option<pipeline::NodeId>,
}

impl Member {
    fn build(name: &'static str, spec: StreamSpec, settings: NodeSpec, reg: &Registry) -> Result<Self> {
        let graph = build_chain(spec, &[settings], reg)?;
        let pulses = taps(&graph, PortKind::Pulses);
        let frames = taps(&graph, PortKind::Frames);
        let router = graph.order().find(|(_, n)| *n == "burst_route").map(|(id, _)| id);
        Ok(Self { name, graph, pulses, frames, router })
    }

    /// Run one block through and collect what came out as packets.
    fn run(&mut self, iq: &[C32], at_us: u64, out: &mut Vec<Packet>) -> Vec<Event> {
        let buf = self.graph.input_buf();
        buf.clear();
        buf.iq_mut().extend_from_slice(iq);
        let mut events = match self.graph.run() {
            Ok(ev) => ev.to_vec(),
            Err(e) => vec![Event::Warning { stage: self.name.into(), message: e.to_string() }],
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
                    if b.routed_to != "none" || b.class.confidence < 0.5 {
                        continue;
                    }
                    out.push(Packet {
                        at_us,
                        center_hz,
                        bandwidth_hz,
                        rssi_dbfs: f32::NAN,
                        snr_db: b.class.features.snr_db,
                        modulation: None,
                        body: PacketBody::Pulses(Vec::new()),
                        measure: Some(m),
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
            let Some(pkgs) = self.graph.buf(*t).and_then(|p| p.as_pulses()) else { continue };
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
                    iq: None,
                });
            }
        }
        for t in &self.frames {
            let spec = self.graph.spec_of(*t);
            let Some(frames) = self.graph.buf(*t).and_then(|p| p.as_frames()) else { continue };
            for f in frames {
                out.push(Packet {
                    at_us,
                    center_hz: spec.map(|s| s.center.0).unwrap_or(0),
                    bandwidth_hz: spec.map(|s| s.bandwidth as u32).unwrap_or(0),
                    rssi_dbfs: f32::NAN,
                    snr_db: f32::NAN,
                    modulation: None,
                    body: PacketBody::Frame(f.clone()),
                    measure: None,
                    iq: None,
                });
            }
        }
        events
    }
}

/// Every output of a graph carrying a given kind.
fn taps(g: &Graph, kind: PortKind) -> Vec<Out> {
    g.order()
        .filter_map(|(id, _)| {
            let out = id.o();
            (g.spec_of(out).map(|s| s.kind) == Some(kind)).then_some(out)
        })
        .collect()
}

/// One open source and the decoders reading it.
struct Slot {
    id: SourceId,
    center_hz: u64,
    members: Vec<Member>,
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
    /// Sources decoders were built for, over the node's life.
    built: u64,
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
            detector: None,
            extractor: None,
            reg: crate::registry(),
            slots: Vec::new(),
            wide: Vec::new(),
            exclude: Vec::new(),
            template: None,
            events: Vec::new(),
            blocks: Vec::new(),
            hits: Vec::new(),
            built: 0,
        }
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
    }

    /// Sources open right now.
    pub fn live(&self) -> Vec<dsp::Source> {
        self.detector.as_ref().map(|d| d.live().copied().collect()).unwrap_or_default()
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

    fn rebuild(&mut self) -> Result<()> {
        if self.rate <= 0.0 {
            return Ok(());
        }
        let d = SourceDetector::new(self.rate, self.input_bw, self.cfg);
        let keep = d.latency_samples();
        self.extractor = Some(SourceExtractor::new(self.rate, self.center.as_f64(), keep, self.cfg));
        self.detector = Some(d);
        self.slots.clear();

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
            self.wide.push(Member::build("mode_s", spec, NodeSpec::new("mode_s"), &self.reg)?);
            self.exclude.push(modes);
        }
        let w = crate::ais_nodes::CHANNEL_WIDTH_HZ;
        let ais = (dsp::ais::CHANNEL_HZ[0] - w, dsp::ais::CHANNEL_HZ[1] + w);
        if covers(ais.0, ais.1) {
            self.wide.push(Member::build("ais", spec, NodeSpec::new("ais"), &self.reg)?);
            self.exclude.push(ais);
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
    fn open(&self, b: &SourceBlock) -> Result<Slot> {
        let mut spec = StreamSpec::iq(b.rate, Hz(b.center_hz));
        spec.bandwidth = b.bandwidth_hz.min(b.rate);
        let mut members = vec![Member::build("burst_route", spec, NodeSpec::new("burst_route"), &self.reg)?];
        if NARROW_HZ.contains(&b.bandwidth_hz) {
            let hz = b.center_hz as f64;
            let fits = |width: f64| b.rate / 2.0 - width > 0.0;
            if fits(crate::pocsag_nodes::CHANNEL_WIDTH_HZ) {
                if let Ok(m) = Member::build("pocsag", spec, NodeSpec::new("pocsag").f("channel_hz", hz), &self.reg) {
                    members.push(m);
                }
            }
            if fits(crate::aprs_nodes::CHANNEL_WIDTH_HZ) {
                if let Ok(m) = Member::build("aprs", spec, NodeSpec::new("aprs").f("channel_hz", hz), &self.reg) {
                    members.push(m);
                }
            }
        }
        if METER_HZ.contains(&b.bandwidth_hz) {
            if let Ok(m) = Member::build("wmbus", spec, NodeSpec::new("wmbus"), &self.reg) {
                members.push(m);
            }
        }
        Ok(Slot { id: b.id, center_hz: b.center_hz, members })
    }

}

fn now_us() -> u64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_micros() as u64)
        .unwrap_or(0)
}

impl Simple for AutoNode {
    fn name(&self) -> &str {
        &self.label
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

    fn negotiate(&mut self, i: &PortSpec) -> Result<StreamSpec> {
        if i.spec.kind != PortKind::Iq {
            return Err(common::Error::other(format!("{}: needs IQ", self.label)));
        }
        self.rate = i.spec.rate;
        self.center = i.spec.center;
        self.input_bw = if i.spec.bandwidth > 0.0 { i.spec.bandwidth.min(i.spec.rate) } else { i.spec.rate };
        self.rebuild()?;
        // Packets are events in time, not a sampled stream, and each one
        // carries its own frequency and width.
        let mut out = i.spec.with_kind(PortKind::Packets);
        out.rate = 0.0;
        out.bandwidth = 0.0;
        Ok(out)
    }

    fn process(&mut self, i: &Payload, o: &mut Payload, c: &mut NodeCtx<'_>) -> Result<()> {
        self.hits.clear();
        let iq = i.as_iq().unwrap_or(&[]);
        let (Some(d), Some(e)) = (self.detector.as_mut(), self.extractor.as_mut()) else {
            return Ok(());
        };
        let at_us = now_us();
        let rate = c.inputs[0].spec.rate.max(1.0);
        let out = o.packets_mut();

        // The span-wide decoders see every block.
        let mut events: Vec<Event> = Vec::new();
        for m in &mut self.wide {
            events.extend(m.run(iq, at_us, out));
        }

        // Then the sources: find, cut out, and read.
        self.events.clear();
        let c0 = self.center.as_f64();
        let exclude = &self.exclude;
        let spur = self.spur_band;
        let raw: Vec<SourceEvent> = d.process(iq).to_vec();
        let others = d
            .live()
            .filter(|s| !spur.is_some_and(|(lo, hi)| (lo..=hi).contains(&(c0 + s.center_hz))))
            .count();
        self.events.extend(raw.iter().filter(|ev| {
            let SourceEvent::Opened(s) = ev else { return true };
            let hz = c0 + s.center_hz;
            if exclude.iter().any(|(lo, hi)| (*lo..=*hi).contains(&hz)) {
                return false;
            }
            // The tuner's centre while something else transmits: the
            // offset following that something's envelope.
            !(others > 0 && spur.is_some_and(|(lo, hi)| (lo..=hi).contains(&hz)))
        }));
        self.blocks.clear();
        e.process(iq, &self.events, &mut self.blocks);
        for ev in &self.events {
            if let SourceEvent::Opened(s) = ev {
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

        // One block per source per call, and the sources share nothing, so
        // the pool takes them all at once.
        let blocks = &self.blocks;
        let results: Vec<(usize, Vec<Event>, Vec<Packet>, bool)> = self
            .slots
            .par_iter_mut()
            .enumerate()
            .filter_map(|(k, slot)| {
                let b = blocks.iter().find(|b| b.id == slot.id)?;
                let mut pk = Vec::new();
                let mut ev = Vec::new();
                for m in &mut slot.members {
                    ev.extend(m.run(&b.samples, at_us, &mut pk));
                }
                let done = matches!(b.state, SourceState::Closed | SourceState::Superseded);
                if b.state == SourceState::Closed {
                    let quiet = vec![C32::new(0.0, 0.0); (FLUSH_S * b.rate) as usize];
                    for m in &mut slot.members {
                        ev.extend(m.run(&quiet, at_us, &mut pk));
                    }
                }
                if b.state == SourceState::Superseded {
                    // A wider stream for the same transmitter takes over
                    // from its start. Whatever this one made of the sliver it
                    // had is half a burst, and half a burst is not evidence.
                    pk.clear();
                    ev.retain(|e| !matches!(e, Event::Decoded(_)));
                }
                Some((k, ev, pk, done))
            })
            .collect();
        let mut results = results;
        results.sort_by_key(|(k, ..)| *k);
        let mut closed = Vec::new();
        for (k, ev, pk, done) in results {
            let center = Hz(self.slots[k].center_hz);
            for e in ev {
                if matches!(e, Event::Decoded(_)) {
                    self.hits.push((center, e.clone()));
                }
                events.push(e);
            }
            out.extend(pk);
            if done {
                closed.push(self.slots[k].id);
            }
        }
        self.slots.retain(|s| !closed.contains(&s.id));

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
        PortSpec { spec: StreamSpec::iq(rate, center), latency: 0 }
    }

    #[test]
    fn the_node_turns_iq_into_packets() {
        let mut n = AutoNode::new("auto", SourceConfig::default());
        let out = Node::negotiate(&mut n, &[spec(2_400_000.0, Hz::mhz(433))]).unwrap();
        assert_eq!(out[0].kind, PortKind::Packets);
        assert!(n.wide().is_empty(), "nothing span-wide belongs at 433 MHz");
        assert!(Node::subgraph(&n).is_some(), "the burst front end is shown before any source");
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
            let mut out = [Payload::Packets(Vec::new())];
            let (mut events, mut tags) = (Vec::new(), Vec::new());
            let mut ctx = NodeCtx::new(0, &ins, &[], &mut events, &mut tags);
            Node::process(&mut n, &[&input], &mut out, &mut ctx).unwrap();
            opened.extend(events.iter().filter_map(|e| match e {
                Event::Detection { center: c, .. } => Some(c.as_f64() - center.as_f64()),
                _ => None,
            }));
        }
        assert!(opened.iter().any(|o| (o - 350_000.0).abs() < 10_000.0), "the real one opened: {opened:?}");
        assert!(!opened.iter().any(|o| o.abs() < 10_000.0), "the spur opened: {opened:?}");
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
    }
}
