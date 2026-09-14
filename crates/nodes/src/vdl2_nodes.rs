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
use dsp::d8psk::{Burst, D8pskConfig, D8pskDemod};
use dsp::resample::Rational;
use dsp::{FirDecim, Mixer};
use pipeline::event::{Decoded, media};
use pipeline::node::{NodeCtx, PortSpec, Simple};
use pipeline::port::{Payload, PortKind, StreamSpec};
use pipeline::registry::{Category, Settings, SettingsExt, StageDesc};

/// The busiest of the European VDL2 channels and the one every ground station
/// carries: the common signalling channel.
pub const DEFAULT_HZ: f64 = 136_975_000.0;

/// An airband channel on the 25 kHz grid.
pub const CHANNEL_WIDTH_HZ: f64 = 25_000.0;

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
            meter: crate::FrameMeter::new(rate, channel_hz as u64, 2.0),
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
        self.meter = crate::FrameMeter::new(want, self.channel_hz as u64, 2.0);

        let mut out = i.spec.with_kind(PortKind::Frames);
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

        let out = o.frames_mut();
        for b in &self.bursts {
            for f in vdl2::frame_bytes(&b.bits) {
                self.accepted += 1;
                out.push(self.meter.frame(f));
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

/// The decode an AVLC frame becomes.
pub fn vdl2_decoded(f: &vdl2::Frame, bytes: &[u8], center: common::Hz) -> Decoded {
    let mut fields: Vec<(String, common::Value)> = vec![
        ("from".into(), common::Value::Text(format!("{:06X}", f.src.addr))),
        ("from_kind".into(), common::Value::Text(f.src.kind.label().into())),
        ("to".into(), common::Value::Text(format!("{:06X}", f.dst.addr))),
        ("to_kind".into(), common::Value::Text(f.dst.kind.label().into())),
        ("frame".into(), common::Value::Text(f.control.label().into())),
    ];
    let acars = f.acars();
    if let Some(a) = &acars {
        fields.extend(a.fields());
    }
    let detail = fields.iter().map(|(k, v)| format!("{k}={v}")).collect::<Vec<_>>().join(" ");
    // An aircraft is named by its ICAO address, which is the same number
    // ADS-B carries, so a frame here and a position there are one aeroplane.
    let who = if f.src.kind.is_aircraft() {
        common::Identity::new("icao", format!("{:06X}", f.src.addr))
    } else {
        common::Identity::new("vdl2-gs", format!("{:06X}", f.src.addr))
    };
    let mut d = Decoded::bytes("VDL2", center, 0.0, bytes.to_vec())
        .by(who)
        .with_detail(detail)
        .with_fields(fields)
        .with_modulation(common::Modulation::D8psk)
        // The frame check sequence, over the whole frame.
        .with_crc(Some(true));
    if acars.as_ref().is_some_and(|a| !a.text.is_empty()) {
        d.media_type = media::TEXT;
    }
    d
}

pub struct Vdl2;

impl Protocol for Vdl2 {
    fn id(&self) -> &'static str {
        "vdl2"
    }
    fn label(&self) -> &'static str {
        "vdl2"
    }
    /// The VHF datalink sub-band, which is the same everywhere: 136.65 is a
    /// guard channel and the datalink channels run up from it.
    fn placement(&self) -> Placement {
        Placement::Bands(vec![(136_650_000.0, 137_000_000.0)])
    }
    fn frame_claim(&self) -> FrameClaim {
        FrameClaim::Band { width_hz: 400_000 }
    }
    fn read_frame(&self, p: &common::Packet, bytes: &[u8]) -> Option<Vec<Decoded>> {
        if !(136_650_000.0..137_000_000.0).contains(&(p.center_hz() as f64)) {
            return None;
        }
        let f = vdl2::parse_frame(bytes)?;
        Some(vec![vdl2_decoded(&f, bytes, common::Hz(p.center_hz()))])
    }
    fn shape(&self) -> Shape {
        Shape {
            widths: &[CHANNEL_WIDTH_HZ],
            min_rate_hz: 105_000.0,
            feed_rate_hz: 200_000.0,
            span_wide: false,
            families: &[],
        }
    }
    fn default_hz(&self) -> f64 {
        DEFAULT_HZ
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
        for rate in [2_048_000.0, 2_400_000.0, 2_880_000.0, 8_000_000.0, 20_000_000.0] {
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
        assert_eq!(out.kind, PortKind::Frames);
        assert_eq!(out.center, Hz(136_975_000));
    }
}
