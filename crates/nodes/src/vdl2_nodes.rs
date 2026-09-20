//! VDL Mode 2 as a graph node.
//!
//! The channel is 25 kHz of differentially encoded 8-PSK at 10500 symbols a
//! second, so the node mixes it down, filters it, resamples to the ten
//! samples a symbol the demodulator wants, and hands complex baseband to
//! `dsp::d8psk`. The burst's blocks, the Reed-Solomon and the AVLC frames are
//! `decode::vdl2`, which knows nothing about pipelines.
//!
//! What reaches the bus is an AVLC frame whose check sequence passed, with
//! the aircraft or ground station that sent it named from its address. This
//! is the link layer: an ACARS message inside one is read, and the X.25 and
//! CLNP that carry the air traffic protocols above it are not.

use crate::NodeSpec;
use crate::protocol::{FrameClaim, Placed, Placement, Protocol, Shape};
use common::Result;
use decode::vdl2;
pub use decode::vdl2::read;
use dsp::d8psk::{Burst, D8pskConfig, D8pskDemod};
use dsp::resample::Rational;
use dsp::{FirDecim, Mixer};
use identify::Signal;
pub use identify::vdl2::BAND_HZ;
pub use identify::vdl2::CHANNEL_WIDTH_HZ;
pub use identify::vdl2::DEFAULT_HZ;
pub use identify::vdl2::Vdl2;
use pipeline::node::{NodeCtx, PortSpec, Simple};
use pipeline::port::{Payload, PortKind, StreamSpec};
use pipeline::registry::{Category, Settings, SettingsExt, StageDesc};

pub struct Vdl2Node {
    channel_hz: f64,
    mixer: Mixer,
    decim: FirDecim,
    resample: Rational,
    demod: D8pskDemod,
    mixed: Vec<common::C32>,
    narrow: Vec<common::C32>,
    at_rate: Vec<common::C32>,
    bursts: Vec<Burst>,
    meter: crate::FrameMeter,
    accepted: u64,
}

impl Default for Vdl2Node {
    fn default() -> Self {
        Self::new(DEFAULT_HZ)
    }
}

impl Vdl2Node {
    pub fn new(channel_hz: f64) -> Self {
        let rate = D8pskConfig::VDL2.rate();
        Self {
            channel_hz,
            mixer: Mixer::new(0.0, 1.0),
            decim: FirDecim::design_hz(rate, 1, CHANNEL_WIDTH_HZ / 2.0, 60.0),
            resample: Rational::with_ratio(1, 1),
            demod: D8pskDemod::new(D8pskConfig::VDL2),
            mixed: Vec::new(),
            narrow: Vec::new(),
            at_rate: Vec::new(),
            bursts: Vec::new(),
            meter: crate::FrameMeter::new(rate, channel_hz as u64, 2.0)
                .keyed_as(common::Modulation::D8psk),
            accepted: 0,
        }
    }

    /// AVLC frames whose check sequence passed since the node was built.
    pub fn accepted(&self) -> u64 {
        self.accepted
    }
}

impl Simple for Vdl2Node {
    fn name(&self) -> &str {
        "vdl2"
    }

    fn negotiate(&mut self, i: &PortSpec) -> Result<StreamSpec> {
        if i.spec.kind != PortKind::Iq {
            return Err(common::Error::other("vdl2 reads complex baseband"));
        }
        let (rate, center) = (i.spec.rate, i.spec.center.as_f64());
        if (self.channel_hz - center).abs() > rate / 2.0 - CHANNEL_WIDTH_HZ / 2.0 {
            return Err(common::Error::other("vdl2 needs its channel inside the span"));
        }
        let want = D8pskConfig::VDL2.rate();
        if rate < want {
            return Err(common::Error::other("vdl2 needs 105 kHz of channel"));
        }
        // Decimate as far as whole samples allow, then resample the rest: the
        // symbol clock is 10500, which divides almost no radio's rate.
        let (factor, resample) = dsp::resample::stage(rate, want, 4096)
            .ok_or_else(|| common::Error::other("vdl2 cannot reach 105 kHz from here"))?;
        self.mixer = Mixer::new(center - self.channel_hz, rate);
        self.decim = FirDecim::design_hz(rate, factor, CHANNEL_WIDTH_HZ / 2.0, 60.0);
        self.resample = resample.unwrap_or_else(|| Rational::with_ratio(1, 1));
        self.demod = D8pskDemod::new(D8pskConfig::VDL2);
        self.meter = crate::FrameMeter::new(want, self.channel_hz as u64, 2.0)
            .keyed_as(common::Modulation::D8psk);

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
        self.at_rate.clear();
        self.resample.process(&self.narrow, &mut self.at_rate);
        self.meter.feed(&self.at_rate);

        self.bursts.clear();
        // The demodulator cannot know how long a burst is: the length is in
        // the header, which is in the burst.
        let mut done = |bits: &[bool]| vdl2::wanted_bits(bits).is_some_and(|n| bits.len() >= n);
        self.demod.process(&self.at_rate, &mut done, &mut self.bursts);

        let out = o.packets_mut();
        for b in &self.bursts {
            for f in vdl2::frame_bytes(&b.bits) {
                self.accepted += 1;
                out.push(self.meter.packet_now(f));
            }
        }
        Ok(())
    }

    fn reset(&mut self) {
        self.mixer.reset();
        self.decim.reset();
        self.demod.reset();
    }
}

impl Protocol for Vdl2 {
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

    /// The VHF datalink sub-band, which is the same everywhere: 136.65 is a
    /// guard channel and the datalink channels run up from it.

    fn frame_claim(&self) -> FrameClaim {
        FrameClaim::Band { width_hz: 400_000 }
    }
    fn stated(&self, p: &common::packet::Packet) -> Option<Vec<common::packet::Proto>> {
        let bytes = p.bytes();
        if !(BAND_HZ.0..BAND_HZ.1).contains(&(p.center_hz() as f64)) {
            return None;
        }
        let f = vdl2::parse_frame(bytes)?;
        Some(vec![read(&f)])
    }

    fn stage_label(&self, hz: f64) -> String {
        format!("{:.3} VDL2", hz / 1e6)
    }
    fn chain(&self, at: Placed) -> Vec<NodeSpec> {
        vec![NodeSpec::new(DESC.name).f(CHANNEL_HZ, at.center_hz)]
    }
}

/// The carrier this stage is pointed at.
const CHANNEL_HZ: &str = "channel_hz";

pub const DESC: StageDesc = StageDesc {
    name: "vdl2",
    summary: "One VDL Mode 2 channel: D8PSK at 10500 baud, AVLC frames",
    category: Category::Decode,
    feeds_bus: true,
};

pub fn build(s: &Settings) -> Result<Box<dyn pipeline::node::Node>> {
    Ok(Box::new(Vdl2Node::new(s.f64_or(CHANNEL_HZ, DEFAULT_HZ))))
}

#[cfg(test)]
mod tests {
    use super::*;
    use common::Hz;

    /// The same rates, against the 105 kHz the demodulator runs at.
    #[test]
    fn an_awkward_radio_rate_is_still_accepted() {
        // 9142857 is the DVB-T rate rounded to hertz, which a HackRF will sit
        // at after a television span. It has no small exact ratio to 105 kHz,
        // so this node refused its input and the whole receiver's graph went
        // down with it: "cannot build the chain: node 5 (136.975 VDL2)
        // rejected its input".
        for rate in [2_048_000.0, 2_400_000.0, 2_880_000.0, 8_000_000.0, 9_142_857.0, 20_000_000.0]
        {
            let mut n = Vdl2Node::new(DEFAULT_HZ);
            let spec = PortSpec { spec: StreamSpec::iq(rate, Hz(136_975_000)), latency: 0 };
            n.negotiate(&spec).unwrap_or_else(|e| panic!("{rate} refused: {e}"));
        }
    }

    #[test]
    fn the_channel_has_to_be_inside_the_span_and_wide_enough() {
        let mut n = Vdl2Node::new(DEFAULT_HZ);
        let far = PortSpec { spec: StreamSpec::iq(250_000.0, Hz(140_000_000)), latency: 0 };
        assert!(n.negotiate(&far).is_err());
        // 50 kHz holds the channel but not the symbol rate the demodulator
        // needs, which is a refusal rather than a quiet half-decode.
        let thin = PortSpec { spec: StreamSpec::iq(50_000.0, Hz(136_975_000)), latency: 0 };
        assert!(n.negotiate(&thin).is_err());
        let ok = PortSpec { spec: StreamSpec::iq(250_000.0, Hz(136_950_000)), latency: 0 };
        let out = n.negotiate(&ok).expect("a channel in the span");
        assert_eq!(out.kind, PortKind::Packets);
        assert_eq!(out.center, Hz(136_975_000));
    }
}
