//! Mode S and ADS-B as a graph node.
//!
//! Everything else in this crate splits the work across two nodes: a front end
//! that turns samples into structured bursts, and a protocol node that reads
//! them. Mode S is one node instead, and the reason is worth recording,
//! because it looks like a shortcut and is not.
//!
//! The demodulator searches for preambles, and a frame it believes blanks the
//! 120 us it occupies, since nothing inside a frame can be the start of
//! another one. A false preamble therefore destroys every real frame
//! overlapping it. The only thing that can tell a false preamble from a real
//! one is the CRC, so the acceptance test has to run *inside* the search
//! rather than downstream of it. Split across two nodes, the front end would
//! blank on candidates the protocol node later rejects: measured on a
//! recorded band, that is 8 frames recovered instead of 27.
//!
//! What stays modular is the parts. The demodulator is `dsp::modes`, the frame
//! format is `decode::adsb`, and neither knows about the other or about
//! pipelines. This node is the wiring, exactly as `PulseDetectNode` is the
//! wiring around `dsp::OokDetector`.

use crate::NodeSpec;
use crate::protocol::{FrameClaim, Mark, Placed, Placement, Protocol, Shape};
use common::Result;
pub use decode::adsb::commb_fields;
pub use decode::adsb::read;
pub use decode::adsb::round1;
use decode::adsb::{self, AddressBook};
use dsp::{ModeSConfig, ModeSDetector, ModeSFrame};
use identify::Signal;
use pipeline::node::{NodeCtx, PortSpec, Simple};
use pipeline::param::{Param, ParamValue};
use pipeline::port::{Payload, PortKind, StreamSpec};
use pipeline::registry::{Category, Settings, StageDesc};

pub struct ModeSNode {
    cfg: ModeSConfig,
    rate: f64,
    det: ModeSDetector,
    meter: crate::FrameMeter,
    book: AddressBook,
    frames: Vec<ModeSFrame>,
    /// Frames accepted since the node was built.
    accepted: u64,
}

impl Default for ModeSNode {
    fn default() -> Self {
        Self::new(ModeSConfig::default())
    }
}

impl ModeSNode {
    pub fn new(cfg: ModeSConfig) -> Self {
        Self {
            cfg,
            // Replaced at negotiation, when the real sample rate is known.
            rate: 2_400_000.0,
            det: ModeSDetector::new(2_400_000.0, cfg),
            // A Mode S frame is 120 us at most, so a millisecond holds one
            // whole with room either side, at any rate a receiver reads
            // 1090 MHz with.
            meter: crate::FrameMeter::new(2_400_000.0, 1_090_000_000, 0.001)
                .keyed_as(common::Modulation::Ppm),
            book: AddressBook::new(),
            frames: Vec::new(),
            accepted: 0,
        }
    }

    /// Aircraft whose address has proved itself.
    pub fn aircraft(&self) -> usize {
        self.book.len()
    }

    pub fn accepted(&self) -> u64 {
        self.accepted
    }
}

impl Simple for ModeSNode {
    fn name(&self) -> &str {
        "mode_s"
    }

    fn negotiate(&mut self, i: &PortSpec) -> Result<StreamSpec> {
        if i.spec.kind != PortKind::Iq {
            return Err(common::Error::other("mode_s reads complex baseband"));
        }
        let rate = i.spec.rate;
        if rate < 2_000_000.0 {
            return Err(common::Error::other(
                "mode_s needs 2 MS/s or more: its bits are 1 us wide",
            ));
        }
        self.det = ModeSDetector::new(rate, self.cfg);
        self.rate = rate;
        self.meter =
            crate::FrameMeter::new(rate, i.spec.center.0, 0.001).keyed_as(common::Modulation::Ppm);
        // Frames rather than bytes: two short replies written into one
        // buffer are indistinguishable from one long frame, and a reply's
        // length is what says which kind of reply it is.
        Ok(i.spec.with_kind(PortKind::Packets))
    }

    fn params(&self) -> Vec<Param> {
        vec![
            Param::float("preamble_ratio", self.cfg.preamble_ratio as f64, 1.0..=8.0)
                .label("Preamble above the quiet slots"),
            Param::float("min_level", self.cfg.min_level as f64, 0.0001..=0.5)
                .label("Preamble amplitude floor")
                .log(),
            Param::bool("crc_framing", self.cfg.crc_framing).label("Also frame by the CRC"),
        ]
    }

    fn set_param(&mut self, name: &str, v: ParamValue) -> Result<()> {
        let f = v.as_f64().unwrap_or_default();
        match name {
            "preamble_ratio" => self.cfg.preamble_ratio = f.max(1.0) as f32,
            "min_level" => self.cfg.min_level = f.max(0.0) as f32,
            "crc_framing" => self.cfg.crc_framing = v.as_bool().unwrap_or(true),
            _ => return Err(common::Error::other(format!("mode_s: unknown parameter {name:?}"))),
        }
        // The detector holds its config by value, and its buffered tail is
        // one frame long, so rebuilding it costs nothing.
        self.det = ModeSDetector::new(self.det.rate(), self.cfg);
        Ok(())
    }

    fn process(&mut self, i: &Payload, o: &mut Payload, c: &mut NodeCtx<'_>) -> Result<()> {
        let Some(iq) = i.as_iq() else { return Ok(()) };
        self.meter.feed(iq);
        let Self { det, book, frames, .. } = self;
        frames.clear();
        let book = std::cell::RefCell::new(book);
        det.process_valid(iq, frames, &|f: &ModeSFrame| {
            book.borrow_mut().accept(&f.bytes, f.preamble_ratio)
        });

        let center = c.inputs[0].spec.center;
        let out = o.packets_mut();
        for f in &self.frames {
            // Correcting a flipped bit is arithmetic on the frame, so it
            // happens in `adsb::accept` rather than in the demodulator, and
            // anything else reading this demodulator's frames makes the same
            // decision there.
            let Some((bytes, frame)) = adsb::accept(&f.bytes) else {
                continue;
            };
            self.accepted += 1;
            // 8 us of preamble and 56 or 112 us of data at 1 Mbit/s, with a
            // little either side.
            let len = ((bytes.len() * 8 + 16) as f64 * 1e-6 * self.rate) as usize;
            let mut pkt = crate::measured(
                center.0,
                c.inputs[0].spec.bandwidth as u32,
                bytes.clone(),
                f.rssi_dbfs,
                self.meter.snr_db(),
            )
            .keyed(common::packet::Keying::configured(common::Modulation::Ppm));
            pkt.carrier.iq = self.meter.iq_at(f.at_sample, len);
            out.push(pkt);
            // Not emitted as a decode here. The frame goes on the bus and
            // the decoder attached to it turns every packet into a row,
            // whichever front end produced it.
            let _ = (&frame, center);
        }
        Ok(())
    }
}

/// Mode S as the auto node and the tables know it: the 1090 MHz allocation,
/// read off the span because a reply is shorter than a detector frame.
///
/// The description is `identify::modes::ModeS`, which anything holding a
/// recording can read without a graph; what this crate adds is the wiring.
pub use identify::modes::ModeS;

impl Protocol for ModeS {
    fn arrives(&self) -> crate::protocol::Arrives {
        crate::protocol::Arrives::InBursts
    }

    fn id(&self) -> &'static str {
        Signal::id(self)
    }
    /// An extended squitter carries the aircraft's own position.
    fn reports_position(&self) -> bool {
        true
    }
    fn label(&self) -> &'static str {
        Signal::label(self)
    }
    fn aliases(&self) -> &'static [&'static str] {
        Signal::aliases(self)
    }
    fn placement(&self) -> Placement {
        Signal::placement(self)
    }
    fn frame_claim(&self) -> FrameClaim {
        FrameClaim::Band { width_hz: 2_000_000 }
    }
    /// A Mode S frame and an AIS frame are both bytes, and nothing tells them
    /// apart except where they were received.
    fn stated(&self, p: &common::packet::Packet) -> Option<Vec<common::packet::Proto>> {
        let bytes = p.bytes();
        if !dsp::modes::is_modes_band(p.center_hz() as f64) {
            return None;
        }
        Some(adsb::parse(bytes).map(|f| vec![read(&f)]).unwrap_or_default())
    }
    fn shape(&self) -> Shape {
        Signal::shape(self)
    }
    fn stage_label(&self, _hz: f64) -> String {
        "1090 Mode S".into()
    }
    fn marks(&self, hz: f64) -> Vec<Mark> {
        vec![Mark { hz, width_hz: Signal::shape(self).widths[0], label: "Mode S".into() }]
    }
    /// Every sample gets a correlation, so what it is handed is what it
    /// costs: 2.4 MS/s is plenty for a 1 Mbit/s pulse train and the 20 MS/s
    /// a receiver may be running is eight times the work for the same
    /// frames.
    fn narrow_span(&self) -> bool {
        true
    }
    fn chain(&self, _at: Placed) -> Vec<NodeSpec> {
        vec![NodeSpec::new("mode_s")]
    }
}

pub const DESC: StageDesc = StageDesc {
    name: "mode_s",
    summary: "1090 MHz ADS-B: preamble search and pulse-position bits",
    category: Category::Decode,
    feeds_bus: true,
};

pub fn build(_s: &Settings) -> Result<Box<dyn pipeline::node::Node>> {
    Ok(Box::new(ModeSNode::default()))
}

#[cfg(test)]
mod tests {
    use super::*;
    use common::Hz;

    fn spec(rate: f64) -> PortSpec {
        PortSpec { spec: StreamSpec::iq(rate, Hz(1_090_000_000)), latency: 0 }
    }

    #[test]
    fn the_node_refuses_a_rate_its_bits_cannot_survive() {
        // Better to refuse the graph than to build one that reports noise.
        let mut n = ModeSNode::default();
        assert!(n.negotiate(&spec(1_024_000.0)).is_err());
        assert!(n.negotiate(&spec(2_400_000.0)).is_ok());
    }

    #[test]
    fn the_node_outputs_bytes() {
        let mut n = ModeSNode::default();
        let out = n.negotiate(&spec(2_400_000.0)).unwrap();
        assert_eq!(out.kind, PortKind::Packets);
    }

    #[test]
    fn a_position_frame_becomes_a_decoded_event_with_fields() {
        let bytes = hex("8d40621d58c382d690c8ac2863a7");
        let frame = adsb::parse(&bytes).unwrap();
        let d = read(&frame);
        assert_eq!((d.id, d.kind), ("adsb", "airborne_position"));
        assert_eq!(d.subject.as_ref().map(|e| e.id.to_string()).as_deref(), Some("40621d"));
        // Half a position, which is all a frame carries, and a height, which
        // is a reading rather than part of a place.
        assert!(d.facts.iter().any(|f| matches!(f, common::packet::Fact::PartialPosition(_))));
        assert!(d.facts.iter().any(|f| matches!(
            f,
            common::packet::Fact::Sensed(r)
                if r.quantity == common::packet::Quantity::Altitude
                    && (r.value - 38_000.0 * 0.3048).abs() < 1.0
        )));
    }

    #[test]
    fn a_short_reply_claims_no_integrity_check() {
        let bytes = hex("02e19838adb7c4");
        let frame = adsb::parse(&bytes).unwrap();
        let d = read(&frame);
        assert_eq!((d.id, d.kind), ("adsb", "reply"));
    }

    fn hex(s: &str) -> Vec<u8> {
        (0..s.len() / 2).map(|i| u8::from_str_radix(&s[i * 2..i * 2 + 2], 16).unwrap()).collect()
    }
}
