//! The channel bank as a single node.
//!
//! A bank runs hundreds of decode chains, and it is still one node, because
//! the whole reason it is fast is that those chains are not independent
//! branches of the outer graph. One polyphase channelizer produces every
//! channel at once, one tiled transpose lays them out, one burst detector
//! decides which are worth running, and then a rayon pool sweeps the chains
//! that are. Expressed as N branches from the source, the scheduler would run
//! them one after another on one thread and the channelizer would be done N
//! times over.
//!
//! What the outer graph gets instead is a node that says what it contains:
//! [`Node::subgraphs`] reports the chain one channel runs and
//! [`Node::subgraph_count`] how many channels run it. So a view of the chain
//! shows the bank's decoder rather than an opaque box, without pretending the
//! channels are separate nodes.

use common::{Hz, Result};
use dsp::DetectorConfig;
use pipeline::Graph;
use pipeline::event::Event;
use pipeline::graph::Topology;
use pipeline::node::{NodeCtx, PortSpec, Simple};
use pipeline::param::{Param, ParamValue};
use pipeline::port::{Payload, PortKind, StreamSpec};

use crate::bank::{ChannelBank, Gating};
use pipeline::registry::{Category, Settings, SettingsExt, StageDesc};

pub struct BankNode {
    bank: ChannelBank,
    /// Width each channel is treated as occupying, for the packet log. This is
    /// the width asked for rather than the channelizer's spacing, since a span
    /// rarely divides into exactly the requested width.
    width_hz: f64,
    make: Box<dyn Fn(StreamSpec) -> Result<Graph> + Send>,
    label: String,
    /// Channel centre of each event this block, alongside the event itself.
    /// Kept so the host can log where a packet came from, which the event
    /// alone cannot say: every channel's decoder believes it is at baseband.
    hits: Vec<(Hz, Event)>,
    rate: f64,
    center: Hz,
    /// Gate settings the channels run with, kept so they can be reported as
    /// parameters and restored onto a bank rebuilt for a new span.
    detect: DetectorConfig,
    /// The band actually wanted, when it is narrower than the input.
    ///
    /// A band is extracted by mixing and decimating, and the decimation is a
    /// power of two, so what arrives here is up to twice the width that was
    /// asked for. Channels outside the wanted band are real channels with real
    /// decoders on them, so without this the receiver reports sensors from
    /// outside the band a scanner block declared, and spends the CPU to do it.
    band: Option<(f64, f64)>,
    /// Dial frequencies where one tuner's span ends and the next begins, as
    /// `sources::Combined::seams` places them.
    ///
    /// A grid channel covering one is made of two slices that each carry
    /// their own DC spike and rolloff and slip against each other by whole
    /// blocks, so whatever it demodulates is a guess. Empty on a receiver
    /// that is one tuner.
    seams: Vec<f64>,
}

impl BankNode {
    /// A bank splitting its input into channels roughly `width_hz` wide, each
    /// running a chain from `make`.
    pub fn new(
        label: impl Into<String>,
        width_hz: f64,
        make: impl Fn(StreamSpec) -> Result<Graph> + Send + 'static,
    ) -> Self {
        Self {
            // Sized properly at negotiation, when the input rate is known.
            // Two channels is the smallest a channelizer will build.
            bank: ChannelBank::new(2, 12, 2.0 * width_hz, Hz(0)),
            width_hz,
            make: Box::new(make),
            label: label.into(),
            hits: Vec::new(),
            rate: 0.0,
            center: Hz(0),
            detect: crate::ism_detector_config(),
            band: None,
            seams: Vec::new(),
        }
    }

    /// Limit the bank to a band inside its input, or `None` for all of it.
    pub fn set_band(&mut self, band: Option<(f64, f64)>) {
        if self.band != band {
            self.band = band;
            self.apply_mask();
        }
    }

    pub fn band(&self) -> Option<(f64, f64)> {
        self.band
    }

    /// The joins of a stitched receiver inside the bank's input.
    pub fn set_seams(&mut self, seams: Vec<f64>) {
        if self.seams != seams {
            self.seams = seams;
            self.apply_mask();
        }
    }

    pub fn seams(&self) -> &[f64] {
        &self.seams
    }

    /// Drop the decoders on channels the wanted band does not reach, and on
    /// channels covering a join between two tuners.
    ///
    /// Their samples are still channelized, because the channelizer produces
    /// every channel at once whether or not anything reads them, but nothing
    /// downstream runs and nothing they hear is reported.
    fn apply_mask(&mut self) {
        if self.band.is_none() && self.seams.is_empty() {
            return;
        }
        let half = self.bank.channel_bandwidth() / 2.0;
        for ch in 0..self.bank.channels() {
            let c = self.bank.channel_center(ch).as_f64();
            let outside = self.band.is_some_and(|(lo, hi)| c + half <= lo || c - half >= hi);
            let on_join = self.seams.iter().any(|h| (c - half..=c + half).contains(h));
            if outside || on_join {
                self.bank.clear_chain(ch);
            }
        }
    }

    /// Channels with a decoder on them, which is what the bank is doing rather
    /// than what it could do.
    pub fn active_channels(&self) -> usize {
        self.bank.active_chains()
    }

    /// Put a decoder back on every channel, then mask again.
    fn rebuild_graphs(&mut self) -> Result<()> {
        self.bank.set_all_graphs(&self.make)?;
        self.apply_mask();
        Ok(())
    }

    /// Channels a span splits into at a given width.
    ///
    /// The channelizer requires an even count, and a single channel would be a
    /// decimator with extra steps.
    pub fn channels_for(rate: f64, width_hz: f64) -> usize {
        let n = (rate / width_hz).round() as usize;
        (n.clamp(2, 1024) + 1) & !1
    }

    pub fn channels(&self) -> usize {
        self.bank.channels()
    }

    /// Width each channel is treated as occupying, never wider than the
    /// channelizer actually delivers.
    pub fn channel_hz(&self) -> f64 {
        self.width_hz.min(self.bank.channel_bandwidth())
    }

    /// What decoded in the last block, and on which channel.
    pub fn hits(&self) -> &[(Hz, Event)] {
        &self.hits
    }

    pub fn set_detector_config(&mut self, cfg: DetectorConfig) {
        self.detect = cfg;
        self.bank.set_detector_config(cfg);
    }

    /// Every channel runs the same graph, so one channel's parameters are the
    /// bank's. The first channel with a decoder on it is the one asked: a
    /// banded bank has no decoder on channel zero.
    fn inner_params(&self) -> Vec<Param> {
        let Some(g) = (0..self.bank.channels()).find_map(|c| self.bank.graph(c)) else {
            return Vec::new();
        };
        g.topology().nodes.into_iter().flat_map(|n| n.params).collect()
    }

    /// Set a parameter on every channel's copy of the decoder.
    ///
    /// Applied to all of them rather than to a template, because the graphs
    /// already exist and hold burst state; rebuilding them to change a
    /// threshold would drop whatever was half received across the band.
    fn set_inner_param(&mut self, name: &str, v: &ParamValue) -> Result<()> {
        let mut found = false;
        let mut err = None;
        for ch in 0..self.bank.channels() {
            let Some(g) = self.bank.graph_mut(ch) else {
                continue;
            };
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
        }
        match err {
            Some(e) => Err(e),
            None if found => Ok(()),
            None => {
                Err(common::Error::other(format!("{}: unknown parameter {name:?}", self.label)))
            }
        }
    }

    pub fn set_gating(&mut self, g: Gating) {
        self.bank.set_gating(g);
    }

    /// Rebuild the bank for a span or a centre frequency.
    ///
    /// A change of centre alone keeps the graphs and clears their state, since
    /// every channel now covers a different frequency and anything half
    /// collected belongs to the old one. A change of rate changes how many
    /// channels there are, so the bank is built again from nothing.
    fn configure(&mut self, rate: f64, center: Hz) -> Result<()> {
        if rate == self.rate {
            let moved = self.bank.center() != center;
            self.bank.set_center(center);
            self.center = center;
            if moved {
                // The channels cover different frequencies now, so which of
                // them are in the wanted band has changed with them. `reset`
                // does not restore the chains this dropped, so a bank that
                // moved has to be re-masked from a full set.
                self.rebuild_graphs()?;
            }
            return Ok(());
        }
        let channels = Self::channels_for(rate, self.width_hz);
        // 12 taps per branch is about 90 dB of channel-to-channel isolation,
        // enough that a strong transmitter does not paint copies of itself
        // across the band and decode several times over.
        let mut bank = ChannelBank::new(channels, 12, rate, center);
        bank.set_gating(Gating::OnDetection);
        bank.set_detector_config(self.detect);
        bank.set_all_graphs(&self.make)?;
        self.bank = bank;
        self.rate = rate;
        self.center = center;
        self.apply_mask();
        Ok(())
    }
}

impl Simple for BankNode {
    fn name(&self) -> &str {
        DESC.name
    }

    fn subgraphs(&self) -> Vec<Topology> {
        // The one chain every channel runs; `subgraph_count` says how many
        // are running it.
        (0..self.bank.channels())
            .find_map(|c| self.bank.graph(c))
            .map(|g| vec![g.topology()])
            .unwrap_or_default()
    }

    fn subgraph_count(&self) -> usize {
        self.bank.channels()
    }

    /// The band the bank is limited to decides which of its channels get a
    /// decoder, and that is settled while the graph negotiates: it has to be
    /// set before the node goes in, on a bank that came through a rebuild as
    /// much as on a fresh one, because the span has usually moved under it
    /// since it was last built.
    fn configure(&mut self, settings: &Settings) {
        self.set_band(crate::band_of(settings));
        self.set_seams(crate::seams_of(settings));
    }

    /// The decoders on the channels that have one, so something asked of
    /// every node in the receiver reaches them rather than stopping at the
    /// bank.
    fn each_inner(&self, f: &mut dyn FnMut(&dyn pipeline::node::Node)) {
        for c in 0..self.bank.channels() {
            let Some(g) = self.bank.graph(c) else { continue };
            for (id, _) in g.order() {
                if let Some(n) = g.node(id) {
                    f(n);
                }
            }
        }
    }

    fn each_inner_mut(&mut self, f: &mut dyn FnMut(&mut dyn pipeline::node::Node)) {
        for c in 0..self.bank.channels() {
            let Some(g) = self.bank.graph_mut(c) else { continue };
            let ids: Vec<_> = g.order().map(|(id, _)| id).collect();
            for id in ids {
                if let Some(n) = g.node_mut(id) {
                    f(n);
                }
            }
        }
    }

    /// The gate in front of the channels, and then whatever the channel graph
    /// itself exposes. Both belong to the bank as far as an operator is
    /// concerned: the decoder inside it is not a node they can reach.
    fn params(&self) -> Vec<Param> {
        let mut p = vec![
            Param::float("open_db", self.detect.open_db as f64, 3.0..=30.0)
                .unit("dB")
                .label("SNR that opens a channel"),
            Param::float("close_db", self.detect.close_db as f64, 1.0..=30.0)
                .unit("dB")
                .label("SNR that closes it again"),
        ];
        p.extend(self.inner_params());
        p
    }

    fn set_param(&mut self, name: &str, v: ParamValue) -> Result<()> {
        let f = v.as_f64().unwrap_or_default();
        match name {
            // Held apart so the close threshold stays under the open one; a
            // gate that closes at the level it opens at chatters.
            "open_db" => {
                self.detect.open_db = f as f32;
                self.detect.close_db = self.detect.close_db.min(self.detect.open_db - 1.0);
            }
            "close_db" => self.detect.close_db = (f as f32).min(self.detect.open_db - 1.0),
            _ => return self.set_inner_param(name, &v),
        }
        self.bank.set_detector_config(self.detect);
        Ok(())
    }

    fn negotiate(&mut self, i: &PortSpec) -> Result<StreamSpec> {
        if i.spec.kind != PortKind::Iq {
            return Err(common::Error::other(format!("{}: needs IQ", self.label)));
        }
        self.configure(i.spec.rate, i.spec.center)?;
        // Every burst the channels detected leaves as a packet, so a log or
        // an analyser can be attached to the bank the same way anything else
        // is attached to anything else. A bank merges its channels onto one
        // port, so the reception is assembled here, where which channel heard
        // it is still known. Packets are events in time rather than a sampled
        // stream, so the rate is zero; the bandwidth is one channel's, since
        // that is what each burst was heard through.
        let mut out = i.spec.with_kind(PortKind::Packets);
        out.rate = 0.0;
        out.bandwidth = self.channel_hz();
        Ok(out)
    }

    fn process(&mut self, i: &Payload, o: &mut Payload, ctx: &mut NodeCtx<'_>) -> Result<()> {
        self.hits.clear();
        let iq = i.as_iq().unwrap_or(&[]);
        if iq.is_empty() {
            return Ok(());
        }
        for ev in self.bank.process(iq)? {
            // Warnings are per burst and per channel, so across a whole band
            // they arrive in the thousands. Only decodes are worth passing up.
            if matches!(ev.event, Event::Decoded(_)) {
                self.hits.push((ev.center, ev.event.clone()));
                ctx.emit(ev.event.clone());
            }
        }
        let bandwidth_hz = self.channel_hz() as u32;
        let at_us = common::packet::now_us();
        o.packets_mut().extend(self.bank.detections().iter().map(|(center, d)| {
            let carrier = common::packet::Carrier::heard(
                at_us,
                center.0,
                bandwidth_hz,
                d.rssi_dbfs,
                d.snr_db,
                common::SourceId(0),
            )
            .lasting(d.duration_us);
            common::packet::Packet::heard(carrier).keyed(d.keying.clone())
        }));
        Ok(())
    }

    fn reset(&mut self) {
        self.bank.reset();
        self.hits.clear();
    }
}

/// The width of one channel in the bank.
const CHANNEL_HZ: &str = "channel_hz";

/// The width a bank channelizes to when nothing has said otherwise: the
/// narrowest ISM sensors are read at.
const DEFAULT_CHANNEL_HZ: f64 = 31_250.0;

pub const DESC: StageDesc = StageDesc {
    name: "bank",
    summary: "Channelize a band and run a burst front end in every \
              channel of it at once",
    category: Category::Decode,
    feeds_bus: true,
};

pub fn build(s: &Settings) -> Result<Box<dyn pipeline::node::Node>> {
    let width = s.f64_or(CHANNEL_HZ, DEFAULT_CHANNEL_HZ).max(1.0);
    // Every tier runs the same graph: what a channel holds is measured and
    // then routed, rather than assumed from the width the tier was built at.
    let label = if width >= 1e6 {
        format!("{:.1} MHz bank", width / 1e6)
    } else {
        format!("{:.0} kHz bank", width / 1e3)
    };
    let mut n = BankNode::new(label, width, crate::ism_decode_graph);
    Simple::configure(&mut n, s);
    Ok(Box::new(n))
}

#[cfg(test)]
mod tests {
    use super::*;
    use pipeline::node::Node;

    fn bank() -> BankNode {
        BankNode::new("31 kHz bank", 31_250.0, crate::ism_decode_graph)
    }

    fn spec(rate: f64) -> PortSpec {
        PortSpec { spec: StreamSpec::iq(rate, Hz(433_920_000)), latency: 0 }
    }

    #[test]
    fn a_span_splits_into_channels_of_about_the_width_asked_for() {
        let mut b = bank();
        Node::negotiate(&mut b, &[spec(2_400_000.0)]).unwrap();
        assert_eq!(b.channels(), 78, "2.4 MHz at 31.25 kHz");
        assert!(b.channel_hz() <= 31_250.0);
    }

    #[test]
    fn a_narrow_span_still_gets_a_usable_bank() {
        // The channelizer will not build fewer than two channels, and a span
        // narrower than one channel must not round down to zero.
        let mut b = bank();
        Node::negotiate(&mut b, &[spec(40_000.0)]).unwrap();
        assert_eq!(b.channels(), 2);
    }

    #[test]
    fn the_bank_says_what_its_channels_run() {
        // The point of the composite: one node to the scheduler, a visible
        // chain to anything drawing the graph.
        let mut b = bank();
        Node::negotiate(&mut b, &[spec(2_400_000.0)]).unwrap();
        let inner = Node::subgraphs(&b).pop().expect("the chain a channel runs");
        let names: Vec<&str> = inner.nodes.iter().map(|n| n.label.as_str()).collect();
        assert!(names.iter().any(|n| n.contains("Classify")), "{names:?}");
        assert_eq!(Node::subgraph_count(&b), b.channels());
    }

    #[test]
    fn retuning_keeps_the_chains_and_drops_their_state() {
        let mut b = bank();
        Node::negotiate(&mut b, &[spec(2_400_000.0)]).unwrap();
        let before = b.channels();
        let mut moved = spec(2_400_000.0);
        moved.spec.center = Hz(868_300_000);
        Node::negotiate(&mut b, &[moved]).unwrap();
        assert_eq!(b.channels(), before, "a retune is not a rebuild");
        assert!(!Node::subgraphs(&b).is_empty(), "the chains survived");
    }

    #[test]
    fn a_join_between_two_tuners_takes_the_decoders_off_the_channels_on_it() {
        let rate = 2_400_000.0;
        let seam = 433_920_000.0 + 400_000.0;
        let mut plain = bank();
        Node::negotiate(&mut plain, &[spec(rate)]).unwrap();
        assert_eq!(plain.active_channels(), 78, "every channel decodes on one tuner");

        let mut stitched = bank();
        let mut s = Settings::new();
        s.insert("seams_hz".into(), ParamValue::Text(format!("{seam}")));
        Simple::configure(&mut stitched, &s);
        assert_eq!(stitched.seams(), [seam]);
        Node::negotiate(&mut stitched, &[spec(rate)]).unwrap();
        assert_eq!(stitched.active_channels(), 77, "the channel on the join still has a decoder");
        let half = stitched.channel_hz() / 2.0;
        let dropped: Vec<f64> = (0..stitched.channels())
            .filter(|c| stitched.bank.graph(*c).is_none())
            .map(|c| stitched.bank.channel_center(c).as_f64())
            .collect();
        assert_eq!(dropped.len(), 1, "{dropped:?}");
        assert!((dropped[0] - seam).abs() <= half, "{dropped:?} is not the channel on {seam}");
    }

    #[test]
    fn a_join_outside_the_wanted_band_costs_no_channel() {
        let rate = 2_400_000.0;
        let mut b = bank();
        b.set_band(Some((433_800_000.0, 434_000_000.0)));
        Node::negotiate(&mut b, &[spec(rate)]).unwrap();
        let banded = b.active_channels();
        assert_eq!(banded, 8, "200 kHz of 31 kHz channels, with the edges");

        let mut inside = bank();
        inside.set_band(Some((433_800_000.0, 434_000_000.0)));
        inside.set_seams(vec![433_900_000.0]);
        Node::negotiate(&mut inside, &[spec(rate)]).unwrap();
        assert_eq!(inside.active_channels(), banded - 1);

        let mut outside = bank();
        outside.set_band(Some((433_800_000.0, 434_000_000.0)));
        outside.set_seams(vec![434_500_000.0]);
        Node::negotiate(&mut outside, &[spec(rate)]).unwrap();
        assert_eq!(outside.active_channels(), banded);
    }

    #[test]
    fn a_wider_span_rebuilds_the_bank() {
        let mut b = bank();
        Node::negotiate(&mut b, &[spec(2_400_000.0)]).unwrap();
        Node::negotiate(&mut b, &[spec(1_024_000.0)]).unwrap();
        assert_eq!(b.channels(), 34, "1.024 MHz at 31.25 kHz");
    }
}
