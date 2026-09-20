//! Wireless M-Bus as a stage: a source's stream in, meter frames out.
//!
//! One demodulator over one source, as the pager and packet stages are.
//! It reads complex baseband at whatever rate the source was cut out at,
//! provided that is four samples a chip or more, and puts out each frame
//! that passed its CRCs as bytes, from the length field on. The frames
//! reach the packet bus as any other frame does, and the protocols node
//! reads the address out of them; see [`decode::wmbus`].

use crate::NodeSpec;
use crate::protocol::{FrameClaim, Placed, Placement, Protocol, Shape};
use common::Result;
pub use decode::wmbus::read;
use dsp::wmbus::{CHIP_RATE, Demod};
use identify::Signal;
pub use identify::wmbus::CHANNEL_WIDTH_HZ;
pub use identify::wmbus::Wmbus;
use pipeline::node::{NodeCtx, PortSpec, Simple};
use pipeline::port::{Payload, PortKind, StreamSpec};
use pipeline::registry::{Category, Settings, StageDesc};

pub struct WmbusNode {
    demod: Option<Demod>,
    meter: crate::FrameMeter,
    frames: u64,
}

impl Default for WmbusNode {
    fn default() -> Self {
        Self::new()
    }
}

impl WmbusNode {
    pub fn new() -> Self {
        Self {
            demod: None,
            meter: crate::FrameMeter::new(1.0, 0, 0.05).keyed_as(common::Modulation::Fsk2),
            frames: 0,
        }
    }

    /// Frames that passed their CRCs since the node was made.
    pub fn frames(&self) -> u64 {
        self.frames
    }
}

impl Simple for WmbusNode {
    fn name(&self) -> &str {
        "wmbus"
    }

    fn negotiate(&mut self, i: &PortSpec) -> Result<StreamSpec> {
        if i.spec.kind != PortKind::Iq {
            return Err(common::Error::other("wmbus reads complex baseband"));
        }
        let d = Demod::new(i.spec.rate);
        if !d.usable() {
            return Err(common::Error::other(format!(
                "wmbus needs at least {} S/s: its chips are {} us wide",
                4.0 * CHIP_RATE,
                1e6 / CHIP_RATE
            )));
        }
        self.demod = Some(d);
        // A meter frame is a few milliseconds; fifty gives the burst and the
        // quiet either side of it without keeping the band.
        self.meter = crate::FrameMeter::new(i.spec.rate, i.spec.center.0, 0.05)
            .keyed_as(common::Modulation::Fsk2);
        let mut out = i.spec.with_kind(PortKind::Packets);
        out.bandwidth = CHANNEL_WIDTH_HZ.min(i.spec.rate);
        Ok(out)
    }

    fn process(&mut self, i: &Payload, o: &mut Payload, _c: &mut NodeCtx<'_>) -> Result<()> {
        let (Some(iq), Some(d)) = (i.as_iq(), self.demod.as_mut()) else {
            return Ok(());
        };
        self.meter.feed(iq);
        for f in d.process(iq) {
            self.frames += 1;
            o.packets_mut().push(self.meter.packet_now(f.bytes.clone()));
        }
        Ok(())
    }

    fn reset(&mut self) {
        self.meter.reset();
        if let Some(d) = &mut self.demod {
            d.reset();
        }
    }
}

/// Widths a meter transmission has: 100 kchip/s keyed 50 kHz either way,
/// with what the extraction adds around it.
const METER_HZ: std::ops::RangeInclusive<f64> = 60_000.0..=450_000.0;

impl Protocol for Wmbus {
    fn id(&self) -> &'static str {
        Signal::id(self)
    }
    fn label(&self) -> &'static str {
        Signal::label(self)
    }
    fn placement(&self) -> Placement {
        Signal::placement(self)
    }
    fn shape(&self) -> Shape {
        Signal::shape(self)
    }
    fn default_hz(&self) -> f64 {
        Signal::default_hz(self)
    }

    /// Where meters transmit (EN 13757-4). Placed by band rather than
    /// anywhere, because a meter transmission's width is a width many
    /// things have: placed by width alone the decoder was built on every
    /// 200 kHz GSM carrier and every splattering 433 MHz sensor, and ran on
    /// all of them for nothing.

    /// The wider of the two meter bands, the 868.95 MHz uplink.
    fn frame_claim(&self) -> FrameClaim {
        FrameClaim::Band { width_hz: 500_000 }
    }
    fn stated(&self, p: &common::packet::Packet) -> Option<Vec<common::packet::Proto>> {
        let bytes = p.bytes();
        if !dsp::wmbus::is_wmbus_band(p.center_hz() as f64) {
            return None;
        }
        Some(read(bytes).into_iter().collect())
    }

    fn accepts_width(&self, _hz: f64, source_width_hz: f64) -> bool {
        METER_HZ.contains(&source_width_hz)
    }
    /// Mode T and C meters, at 868.95 MHz.

    fn chain(&self, _at: Placed) -> Vec<NodeSpec> {
        vec![NodeSpec::new("wmbus")]
    }
}

pub const DESC: StageDesc = StageDesc {
    name: "wmbus",
    summary: "Wireless M-Bus meter frames, modes T and C at 100 kchip/s",
    category: Category::Decode,
    feeds_bus: true,
};

pub fn build(_s: &Settings) -> Result<Box<dyn pipeline::node::Node>> {
    Ok(Box::new(WmbusNode::new()))
}
