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
//! [`Node::subgraph`] reports the chain one channel runs and
//! [`Node::subgraph_count`] how many channels run it. So a view of the chain
//! shows the bank's decoder rather than an opaque box, without pretending the
//! channels are separate nodes.

use common::{Hz, Result};
use dsp::DetectorConfig;
use pipeline::event::Event;
use pipeline::graph::Topology;
use pipeline::node::{NodeCtx, PortSpec, Simple};
use pipeline::port::{Payload, PortKind, StreamSpec};
use pipeline::Graph;

use crate::bank::{ChannelBank, Gating};

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
    /// The band actually wanted, when it is narrower than the input.
    ///
    /// A band is extracted by mixing and decimating, and the decimation is a
    /// power of two, so what arrives here is up to twice the width that was
    /// asked for. Channels outside the wanted band are real channels with real
    /// decoders on them, so without this the receiver reports sensors from
    /// outside the band a scanner block declared, and spends the CPU to do it.
    band: Option<(f64, f64)>,
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
            band: None,
        }
    }

    /// Limit the bank to a band inside its input, or `None` for all of it.
    pub fn set_band(&mut self, band: Option<(f64, f64)>) {
        if self.band != band {
            self.band = band;
            self.apply_band();
        }
    }

    pub fn band(&self) -> Option<(f64, f64)> {
        self.band
    }

    /// Drop the decoders on channels the wanted band does not reach.
    ///
    /// Their samples are still channelized, because the channelizer produces
    /// every channel at once whether or not anything reads them, but nothing
    /// downstream runs and nothing they hear is reported.
    fn apply_band(&mut self) {
        let Some((lo, hi)) = self.band else { return };
        let half = self.bank.channel_bandwidth() / 2.0;
        for ch in 0..self.bank.channels() {
            let c = self.bank.channel_center(ch).as_f64();
            if c + half <= lo || c - half >= hi {
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
        self.apply_band();
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
        self.bank.set_detector_config(cfg);
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
        bank.set_detector_config(crate::ism_detector_config());
        bank.set_all_graphs(&self.make)?;
        self.bank = bank;
        self.rate = rate;
        self.center = center;
        self.apply_band();
        Ok(())
    }
}

impl Simple for BankNode {
    fn name(&self) -> &str {
        &self.label
    }

    fn subgraph(&self) -> Option<Topology> {
        (0..self.bank.channels()).find_map(|c| self.bank.graph(c)).map(|g| g.topology())
    }

    fn subgraph_count(&self) -> usize {
        self.bank.channels()
    }

    fn negotiate(&mut self, i: &PortSpec) -> Result<StreamSpec> {
        if i.spec.kind != PortKind::Iq {
            return Err(common::Error::other(format!("{}: needs IQ", self.label)));
        }
        self.configure(i.spec.rate, i.spec.center)?;
        // Every burst the channels detected leaves as a package, so a log or
        // an analyser can be attached to the bank the same way anything else
        // is attached to anything else. Packages are events in time rather
        // than a sampled stream, so the rate is zero; the bandwidth is one
        // channel's, since that is what each burst was heard through.
        let mut out = i.spec.with_kind(PortKind::Pulses);
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
        o.pulses_mut().extend_from_slice(self.bank.packages());
        Ok(())
    }

    fn reset(&mut self) {
        self.bank.reset();
        self.hits.clear();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use pipeline::node::Node;

    fn bank() -> BankNode {
        BankNode::new("OOK bank", 31_250.0, crate::ism_ook_graph)
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
        let inner = Node::subgraph(&b).expect("the chain a channel runs");
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
        assert!(Node::subgraph(&b).is_some(), "the chains survived");
    }

    #[test]
    fn a_wider_span_rebuilds_the_bank() {
        let mut b = bank();
        Node::negotiate(&mut b, &[spec(2_400_000.0)]).unwrap();
        Node::negotiate(&mut b, &[spec(1_024_000.0)]).unwrap();
        assert_eq!(b.channels(), 34, "1.024 MHz at 31.25 kHz");
    }
}
