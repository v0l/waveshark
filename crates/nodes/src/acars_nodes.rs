//! ACARS as a graph node.
//!
//! The same two-layer shape as APRS and POCSAG, with an AM channel where those
//! have an FM one: mix the aircraft channel down, filter it to the 15 kHz an
//! airband channel occupies, detect the envelope, and hand the audio to
//! `dsp::msk`. The block assembly and the fields are `decode::acars`, which
//! knows nothing about pipelines, and the demodulator knows nothing about
//! aircraft.
//!
//! What reaches the bus is a block whose every character passed its parity and
//! whose CRC-16 checked, so a row here is a message that was received rather
//! than one that was guessed at.

use crate::NodeSpec;
use crate::protocol::{FrameClaim, Placed, Placement, Protocol, Shape};
use common::Result;
use decode::acars;
pub use decode::acars::read;
use dsp::msk::{MskConfig, MskDemod};
use dsp::{AmDemod, FirDecim, Mixer};
use identify::Signal;
pub use identify::acars::Acars;
pub use identify::acars::CHANNEL_WIDTH_HZ;
pub use identify::acars::DEFAULT_HZ;
pub use identify::acars::{AUDIO_HZ, CARRIER_TRACK_HZ};
use pipeline::node::{NodeCtx, PortSpec, Simple};
use pipeline::port::{Payload, PortKind, StreamSpec};
use pipeline::registry::{Category, Settings, SettingsExt, StageDesc};

pub struct AcarsNode {
    channel_hz: f64,
    mixer: Mixer,
    decim: FirDecim,
    am: AmDemod,
    msk: MskDemod,
    framer: acars::Framer,
    mixed: Vec<common::C32>,
    narrow: Vec<common::C32>,
    audio: Vec<f32>,
    bits: Vec<bool>,
    blocks: Vec<Vec<u8>>,
    meter: crate::FrameMeter,
    accepted: u64,
}

impl Default for AcarsNode {
    fn default() -> Self {
        Self::new(DEFAULT_HZ)
    }
}

impl AcarsNode {
    pub fn new(channel_hz: f64) -> Self {
        Self {
            channel_hz,
            // All replaced at negotiation, when the real rate is known.
            mixer: Mixer::new(0.0, 1.0),
            decim: FirDecim::design_hz(AUDIO_HZ, 1, CHANNEL_WIDTH_HZ / 2.0, 60.0),
            am: AmDemod::new(AUDIO_HZ, CARRIER_TRACK_HZ),
            msk: MskDemod::new(AUDIO_HZ, MskConfig::ACARS),
            framer: acars::Framer::new(),
            mixed: Vec::new(),
            narrow: Vec::new(),
            audio: Vec::new(),
            bits: Vec::new(),
            blocks: Vec::new(),
            meter: crate::FrameMeter::new(AUDIO_HZ, channel_hz as u64, 2.0)
                .keyed_as(common::Modulation::Msk),
            accepted: 0,
        }
    }

    /// Blocks that passed their parity and their CRC since the node was built.
    pub fn accepted(&self) -> u64 {
        self.accepted
    }
}

impl Simple for AcarsNode {
    fn name(&self) -> &str {
        "acars"
    }

    fn negotiate(&mut self, i: &PortSpec) -> Result<StreamSpec> {
        if i.spec.kind != PortKind::Iq {
            return Err(common::Error::other("acars reads complex baseband"));
        }
        let (rate, center) = (i.spec.rate, i.spec.center.as_f64());
        if (self.channel_hz - center).abs() > rate / 2.0 - CHANNEL_WIDTH_HZ / 2.0 {
            return Err(common::Error::other("acars needs its channel inside the span"));
        }
        let factor = (rate / AUDIO_HZ).round().max(1.0) as usize;
        let audio_rate = rate / factor as f64;
        self.mixer = Mixer::new(center - self.channel_hz, rate);
        self.decim = FirDecim::design_hz(rate, factor, CHANNEL_WIDTH_HZ / 2.0, 60.0);
        self.am = AmDemod::new(audio_rate, CARRIER_TRACK_HZ);
        self.msk = MskDemod::new(audio_rate, MskConfig::ACARS);
        self.framer.reset();
        // On the channel, not on the span: a 15 kHz channel inside a couple of
        // megahertz of band is a fraction of a percent of the power.
        self.meter = crate::FrameMeter::new(audio_rate, self.channel_hz as u64, 2.0)
            .keyed_as(common::Modulation::Msk);

        let mut out = i.spec.with_kind(PortKind::Packets);
        out.center = common::Hz(self.channel_hz as u64);
        out.bandwidth = CHANNEL_WIDTH_HZ;
        Ok(out)
    }

    fn process(&mut self, i: &Payload, o: &mut Payload, _c: &mut NodeCtx<'_>) -> Result<()> {
        let Some(iq) = i.as_iq() else { return Ok(()) };
        self.mixed.clear();
        self.mixer.process(iq, &mut self.mixed);
        self.narrow.clear();
        self.decim.process(&self.mixed, &mut self.narrow);
        self.audio.clear();
        self.am.process(&self.narrow, &mut self.audio);

        self.meter.feed(&self.narrow);
        self.bits.clear();
        self.msk.process(&self.audio, &mut self.bits);
        self.blocks.clear();
        let bits = std::mem::take(&mut self.bits);
        self.framer.process(&bits, &mut self.blocks);
        self.bits = bits;

        let out = o.packets_mut();
        for b in &self.blocks {
            self.accepted += 1;
            out.push(self.meter.packet_now(b.clone()));
        }
        Ok(())
    }

    fn reset(&mut self) {
        self.mixer.reset();
        self.decim.reset();
        self.am.reset();
        self.msk.reset();
        self.framer.reset();
    }
}

impl Protocol for Acars {
    fn arrives(&self) -> crate::protocol::Arrives {
        crate::protocol::Arrives::InBursts
    }

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

    /// The data half of the VHF airband. Which channels are in use is
    /// regional, so the band is the claim and the scanner table names the
    /// carriers inside it.

    fn frame_claim(&self) -> FrameClaim {
        FrameClaim::Band { width_hz: 8_000_000 }
    }
    fn stated(&self, p: &common::packet::Packet) -> Option<Vec<common::packet::Proto>> {
        let bytes = p.bytes();
        if !(129e6..137e6).contains(&(p.center_hz() as f64)) {
            return None;
        }
        let m = acars::parse(bytes)?;
        Some(vec![read(&m)])
    }

    fn stage_label(&self, hz: f64) -> String {
        format!("{:.3} ACARS", hz / 1e6)
    }
    fn chain(&self, at: Placed) -> Vec<NodeSpec> {
        vec![NodeSpec::new(DESC.name).f(CHANNEL_HZ, at.center_hz)]
    }
}

/// The carrier this stage is pointed at.
const CHANNEL_HZ: &str = "channel_hz";

pub const DESC: StageDesc = StageDesc {
    name: "acars",
    summary: "One ACARS channel: AM, 2400 baud MSK, ARINC 618 blocks",
    category: Category::Decode,
    feeds_bus: true,
};

pub fn build(s: &Settings) -> Result<Box<dyn pipeline::node::Node>> {
    Ok(Box::new(AcarsNode::new(s.f64_or(CHANNEL_HZ, DEFAULT_HZ))))
}

#[cfg(test)]
mod tests {
    use super::*;
    use common::Hz;

    #[test]
    fn the_channel_has_to_be_inside_the_span() {
        let mut n = AcarsNode::new(131_725_000.0);
        let far = PortSpec { spec: StreamSpec::iq(250_000.0, Hz(140_000_000)), latency: 0 };
        assert!(n.negotiate(&far).is_err());
        let near = PortSpec { spec: StreamSpec::iq(250_000.0, Hz(131_700_000)), latency: 0 };
        let out = n.negotiate(&near).expect("a channel in the span");
        assert_eq!(out.kind, PortKind::Packets);
        assert_eq!(out.center, Hz(131_725_000));
    }

    #[test]
    fn a_block_becomes_a_row_naming_the_aircraft() {
        let block = b"2.EI-DEO\x15Q01\x02S01AEIN123ENGINE OK\x03";
        let m = acars::parse(block).expect("a message");
        let d = read(&m);
        assert_eq!((d.id, d.kind), ("acars", "downlink"));
        // The aircraft, by the registration the block carries and the flight
        // it was operating.
        let who = d.subject.as_ref().expect("the aircraft");
        assert_eq!(who.id.to_string(), "EI-DEO");
        assert_eq!(who.name.as_deref(), Some("EIN123"));
        // The box sent it, not the crew, so nobody wrote anything.
        assert!(d.wrote().is_none());
    }
}
