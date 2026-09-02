//! Sources as graph stages: one node that finds them, one that reads them.
//!
//! [`SourceDetectNode`] watches a wideband stream and turns it into a
//! [`PortKind::Sources`] port: a run of blocks per transmitter, each at a
//! rate that fits the width the transmitter was measured to have. See
//! [`dsp::source`] for how, and for why this replaces a grid of channels.
//!
//! [`SourceDecodeNode`] is the other side of that port. It builds a decode
//! graph when a source opens, feeds it that source's blocks for as long as
//! the source lasts, and drops it when the source closes. Decoders inside it
//! therefore see one continuous stream each, however short, and do not know
//! or care that it was cut out of a span. The graph is the same one the
//! channel bank ran per channel, so what changed is what it is fed, not what
//! it does with it.
//!
//! The two are separate stages rather than one because the port between
//! them is worth having: a recorder, a waterfall or a log can be hung on the
//! sources themselves, and the decoder can be swapped for another without
//! touching detection.

use common::{Hz, Package, Result, SourceId, SourceState};
use dsp::{SourceConfig, SourceDetector, SourceEvent, SourceExtractor};
use pipeline::event::Event;
use pipeline::graph::Topology;
use pipeline::node::{NodeCtx, PortSpec, Simple};
use pipeline::param::{Param, ParamValue};
use pipeline::port::{Payload, PortKind, StreamSpec};
use pipeline::{Graph, Out};
use rayon::prelude::*;

/// Finds transmitters in a wideband stream and hands each one over as its
/// own stream.
pub struct SourceDetectNode {
    cfg: SourceConfig,
    rate: f64,
    center: Hz,
    detector: Option<SourceDetector>,
    extractor: Option<SourceExtractor>,
    events: Vec<SourceEvent>,
    /// Sources that opened in the last block, for a host that wants them
    /// without reading every event.
    opened: Vec<dsp::Source>,
    /// The band wanted, in absolute hertz, when narrower than the input.
    band: Option<(f64, f64)>,
    /// Bandwidth of the input the detector was built for.
    input_bw: f64,
}

impl SourceDetectNode {
    pub fn new(cfg: SourceConfig) -> Self {
        Self {
            cfg,
            rate: 0.0,
            center: Hz(0),
            detector: None,
            extractor: None,
            events: Vec::new(),
            opened: Vec::new(),
            band: None,
            input_bw: 0.0,
        }
    }

    /// Limit detection to a band inside the input, or `None` for all of it.
    pub fn set_band(&mut self, band: Option<(f64, f64)>) {
        self.band = band;
        self.apply_band();
    }

    pub fn band(&self) -> Option<(f64, f64)> {
        self.band
    }

    fn apply_band(&mut self) {
        let (Some(d), Some((lo, hi))) = (self.detector.as_mut(), self.band) else { return };
        let c = self.center.as_f64();
        d.set_band(lo - c, hi - c);
    }

    pub fn config(&self) -> SourceConfig {
        self.cfg
    }

    /// Sources open right now.
    pub fn live(&self) -> Vec<dsp::Source> {
        self.detector.as_ref().map(|d| d.live().copied().collect()).unwrap_or_default()
    }

    /// Sources that opened in the last block.
    pub fn opened(&self) -> &[dsp::Source] {
        &self.opened
    }

    fn rebuild(&mut self, bandwidth: f64) {
        if self.rate <= 0.0 {
            return;
        }
        let d = SourceDetector::new(self.rate, bandwidth, self.cfg);
        let keep = d.latency_samples();
        self.extractor = Some(SourceExtractor::new(self.rate, self.center.as_f64(), keep, self.cfg));
        self.detector = Some(d);
        self.input_bw = bandwidth;
        self.apply_band();
    }
}

impl Default for SourceDetectNode {
    fn default() -> Self {
        Self::new(SourceConfig::default())
    }
}

impl Simple for SourceDetectNode {
    fn name(&self) -> &str {
        "source_detect"
    }

    fn negotiate(&mut self, i: &PortSpec) -> Result<StreamSpec> {
        if i.spec.kind != PortKind::Iq {
            return Err(common::Error::other("source_detect needs IQ"));
        }
        self.rate = i.spec.rate;
        self.center = i.spec.center;
        self.rebuild(i.spec.bandwidth);
        // Sources are streams of their own; the port's rate says nothing
        // about any of them, and the bandwidth is the span they were found
        // in.
        let mut out = i.spec.with_kind(PortKind::Sources);
        out.rate = 0.0;
        Ok(out)
    }

    fn process(&mut self, i: &Payload, o: &mut Payload, c: &mut NodeCtx<'_>) -> Result<()> {
        self.opened.clear();
        let iq = i.as_iq().unwrap_or(&[]);
        let (Some(d), Some(e)) = (self.detector.as_mut(), self.extractor.as_mut()) else {
            return Ok(());
        };
        self.events.clear();
        self.events.extend_from_slice(d.process(iq));
        e.process(iq, &self.events, o.sources_mut());

        let rate = c.inputs[0].spec.rate.max(1.0);
        for ev in &self.events {
            if let SourceEvent::Opened(s) = ev {
                self.opened.push(*s);
                c.emit(Event::Detection {
                    center: Hz((self.center.as_f64() + s.center_hz).max(0.0) as u64),
                    bandwidth: s.bandwidth_hz(),
                    snr_db: s.peak_snr_db,
                    at: s.start_sample as f64 / rate,
                });
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
        self.opened.clear();
    }

    fn params(&self) -> Vec<Param> {
        vec![
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
            Param::float("lead_ms", self.cfg.lead_us as f64 / 1e3, 0.5..=200.0)
                .unit("ms")
                .label("Kept before a source opens"),
            Param::float("tail_ms", self.cfg.tail_us as f64 / 1e3, 1.0..=500.0)
                .unit("ms")
                .label("Kept after it closes"),
            Param::float("min_rate_hz", self.cfg.min_rate_hz, 1_000.0..=1_000_000.0)
                .unit("Hz")
                .label("Lowest rate a source is read at")
                .log(),
        ]
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
            "bin_hz" => self.cfg.bin_hz = f.max(1.0),
            "lead_ms" => self.cfg.lead_us = (f * 1e3).max(0.0) as u32,
            "tail_ms" => self.cfg.tail_us = (f * 1e3).max(0.0) as u32,
            "min_rate_hz" => self.cfg.min_rate_hz = f.max(1.0),
            _ => {
                return Err(common::Error::other(format!(
                    "source_detect: unknown parameter {name:?}"
                )))
            }
        }
        // Thresholds and timings are read every frame; everything else
        // shapes the detector and needs it built again. Rebuilding drops
        // the floors and every open source, which is right for a change of
        // resolution and wrong for a nudge to a threshold.
        if matches!(name, "bin_hz" | "min_rate_hz" | "lead_ms" | "tail_ms") {
            self.rebuild(self.input_bw);
        } else if self.detector.is_some() {
            self.detector = Some(SourceDetector::new(self.rate, self.input_bw, self.cfg));
            self.apply_band();
        }
        Ok(())
    }
}

/// Runs a decode graph per source.
pub struct SourceDecodeNode {
    make: Box<dyn Fn(StreamSpec) -> Result<Graph> + Send>,
    label: String,
    graphs: Vec<(SourceId, Graph, Vec<Out>)>,
    /// A graph built at a nominal rate, so the chain view can show what a
    /// source will run before any has opened.
    template: Option<Graph>,
    /// What decoded in the last block, alongside where it came from.
    hits: Vec<(Hz, Event)>,
    /// Sources a graph was built for, over the node's life.
    built: u64,
}

impl SourceDecodeNode {
    pub fn new(
        label: impl Into<String>,
        make: impl Fn(StreamSpec) -> Result<Graph> + Send + 'static,
    ) -> Self {
        Self {
            make: Box::new(make),
            label: label.into(),
            graphs: Vec::new(),
            template: None,
            hits: Vec::new(),
            built: 0,
        }
    }

    /// What decoded in the last block, and where.
    pub fn hits(&self) -> &[(Hz, Event)] {
        &self.hits
    }

    /// Sources with a decoder running right now.
    pub fn active(&self) -> usize {
        self.graphs.len()
    }

    /// Graphs built since the node was made.
    pub fn built(&self) -> u64 {
        self.built
    }

    fn build(&mut self, b: &common::SourceBlock) -> Result<()> {
        let mut spec = StreamSpec::iq(b.rate, Hz(b.center_hz));
        spec.bandwidth = b.bandwidth_hz.min(b.rate);
        let g = (self.make)(spec)
            .map_err(|e| common::Error::other(format!("{}: source {}: {e}", self.label, b.id.0)))?;
        let taps = pulse_taps(&g);
        self.graphs.push((b.id, g, taps));
        self.built += 1;
        Ok(())
    }
}

impl Simple for SourceDecodeNode {
    fn name(&self) -> &str {
        &self.label
    }

    fn subgraph(&self) -> Option<Topology> {
        self.graphs
            .first()
            .map(|(_, g, _)| g.topology())
            .or_else(|| self.template.as_ref().map(|g| g.topology()))
    }

    fn subgraph_count(&self) -> usize {
        self.graphs.len().max(1)
    }

    fn negotiate(&mut self, i: &PortSpec) -> Result<StreamSpec> {
        if i.spec.kind != PortKind::Sources {
            return Err(common::Error::other(format!("{}: needs sources", self.label)));
        }
        let nominal = StreamSpec::iq(SourceConfig::default().min_rate_hz, i.spec.center);
        self.template = Some((self.make)(nominal)?);
        let mut out = i.spec.with_kind(PortKind::Pulses);
        out.rate = 0.0;
        // Every burst carries its own frequency, and its width was decided
        // per source; the port cannot claim one for all of them.
        out.bandwidth = 0.0;
        Ok(out)
    }

    fn process(&mut self, i: &Payload, o: &mut Payload, c: &mut NodeCtx<'_>) -> Result<()> {
        self.hits.clear();
        let blocks = i.as_sources().unwrap_or(&[]);
        if blocks.is_empty() {
            return Ok(());
        }
        for b in blocks {
            if !self.graphs.iter().any(|(id, _, _)| *id == b.id) {
                // A block whose source was never seen opening is a source
                // that opened before this node was attached, or after its
                // graph failed to build; either way it starts here.
                self.build(b)?;
            }
        }

        // One block per source per call, so each graph runs at most once,
        // and they share nothing: the pool takes them all at once.
        let results: Vec<(usize, Vec<Event>, Vec<Package>, bool)> = self
            .graphs
            .par_iter_mut()
            .enumerate()
            .filter_map(|(k, (id, g, taps))| {
                let b = blocks.iter().find(|b| b.id == *id)?;
                let buf = g.input_buf();
                buf.clear();
                buf.iq_mut().extend_from_slice(&b.samples);
                let evs = match g.run() {
                    Ok(ev) => ev.to_vec(),
                    Err(e) => vec![Event::Warning {
                        stage: format!("source {}", id.0),
                        message: e.to_string(),
                    }],
                };
                let pkgs: Vec<Package> = taps
                    .iter()
                    .filter_map(|t| g.buf(*t))
                    .filter_map(|p| p.as_pulses())
                    .flat_map(|p| p.iter().cloned())
                    .collect();
                Some((k, evs, pkgs, matches!(b.state, SourceState::Closed | SourceState::Superseded)))
            })
            .collect();

        let mut results = results;
        results.sort_by_key(|(k, _, _, _)| *k);
        let mut closed = Vec::new();
        for (k, evs, pkgs, done) in results {
            let center = Hz(blocks.iter().find(|b| b.id == self.graphs[k].0).map(|b| b.center_hz).unwrap_or(0));
            for e in evs {
                if matches!(e, Event::Decoded(_)) {
                    self.hits.push((center, e.clone()));
                    c.emit(e);
                }
            }
            o.pulses_mut().extend(pkgs);
            if done {
                closed.push(self.graphs[k].0);
            }
        }
        self.graphs.retain(|(id, _, _)| !closed.contains(id));
        Ok(())
    }

    fn reset(&mut self) {
        self.graphs.clear();
        self.hits.clear();
    }

    /// The decoder's own knobs, read from the template since every source
    /// runs a copy of it.
    fn params(&self) -> Vec<Param> {
        self.template
            .as_ref()
            .map(|g| g.topology().nodes.into_iter().flat_map(|n| n.params).collect())
            .unwrap_or_default()
    }

    /// Set on every running copy and on the template, so sources that open
    /// later start with it too.
    fn set_param(&mut self, name: &str, v: ParamValue) -> Result<()> {
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
        for (_, g, _) in &mut self.graphs {
            apply(g);
        }
        match err {
            Some(e) => Err(e),
            None if found => Ok(()),
            None => Err(common::Error::other(format!(
                "{}: unknown parameter {name:?}",
                self.label
            ))),
        }
    }
}

/// Every output in a graph that carries packages.
fn pulse_taps(g: &Graph) -> Vec<Out> {
    g.order()
        .filter_map(|(id, _)| {
            let out = id.o();
            matches!(g.spec_of(out).map(|s| s.kind), Some(PortKind::Pulses)).then_some(out)
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use pipeline::node::Node;

    fn spec(rate: f64) -> PortSpec {
        PortSpec { spec: StreamSpec::iq(rate, Hz(433_920_000)), latency: 0 }
    }

    #[test]
    fn the_detector_negotiates_a_sources_port() {
        let mut n = SourceDetectNode::default();
        let out = Node::negotiate(&mut n, &[spec(2_400_000.0)]).unwrap();
        assert_eq!(out[0].kind, PortKind::Sources);
        assert_eq!(out[0].rate, 0.0);
    }

    #[test]
    fn the_decoder_shows_its_graph_before_any_source_opens() {
        let mut n = SourceDecodeNode::new("sources", crate::ism_decode_graph);
        let mut s = spec(2_400_000.0);
        s.spec.kind = PortKind::Sources;
        s.spec.rate = 0.0;
        let out = Node::negotiate(&mut n, &[s]).unwrap();
        assert_eq!(out[0].kind, PortKind::Pulses);
        let inner = Node::subgraph(&n).expect("template graph");
        assert!(inner.nodes.iter().any(|n| n.label.contains("Classify")));
        assert!(!Node::params(&n).is_empty(), "the decoder's knobs are the node's");
    }

    #[test]
    fn the_decoder_refuses_iq() {
        let mut n = SourceDecodeNode::new("sources", crate::ism_decode_graph);
        assert!(Node::negotiate(&mut n, &[spec(2_400_000.0)]).is_err());
    }
}
