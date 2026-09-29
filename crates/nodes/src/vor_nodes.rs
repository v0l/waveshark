use crate::NodeSpec;
use crate::protocol::{FrameClaim, Mark, Placed, Placement, Protocol, Shape};
use common::Result;
use decode::vor;
use dsp::{FirDecim, Mixer};
use identify::Signal;
pub use identify::vor::{AUDIO_HZ, CHANNEL_WIDTH_HZ, DEFAULT_HZ, Reader, Vor, WINDOW_S};
use pipeline::node::{NodeCtx, PortSpec, Simple};
use pipeline::port::{Payload, PortKind, StreamSpec};
use pipeline::registry::{Category, Settings, SettingsExt, StageDesc};

pub struct VorNode {
    channel_hz: f64,
    mixer: Mixer,
    decim: FirDecim,
    reader: Reader,
    mixed: Vec<common::C32>,
    narrow: Vec<common::C32>,
    frames: Vec<Vec<u8>>,
    meter: crate::FrameMeter,
    bearings: u64,
}

impl Default for VorNode {
    fn default() -> Self {
        Self::new(DEFAULT_HZ)
    }
}

impl VorNode {
    pub fn new(channel_hz: f64) -> Self {
        Self {
            channel_hz,
            mixer: Mixer::new(0.0, 1.0),
            decim: FirDecim::design_hz(AUDIO_HZ, 1, CHANNEL_WIDTH_HZ / 2.0, 60.0),
            reader: Reader::new(AUDIO_HZ),
            mixed: Vec::new(),
            narrow: Vec::new(),
            frames: Vec::new(),
            meter: crate::FrameMeter::new(AUDIO_HZ, channel_hz as u64, WINDOW_S)
                .keyed_as(common::Modulation::Ask),
            bearings: 0,
        }
    }
}

impl Simple for VorNode {
    fn name(&self) -> &str {
        "vor"
    }

    fn negotiate(&mut self, i: &PortSpec) -> Result<StreamSpec> {
        if i.spec.kind != PortKind::Iq {
            return Err(common::Error::other("vor reads complex baseband"));
        }
        let (rate, center) = (i.spec.rate, i.spec.center.as_f64());
        if (self.channel_hz - center).abs() > rate / 2.0 - CHANNEL_WIDTH_HZ / 2.0 {
            return Err(common::Error::other("vor needs its channel inside the span"));
        }
        let factor = (rate / AUDIO_HZ).floor().max(1.0) as usize;
        let audio_rate = rate / factor as f64;
        self.mixer = Mixer::new(center - self.channel_hz, rate);
        self.decim = FirDecim::design_hz(rate, factor, CHANNEL_WIDTH_HZ / 2.0, 60.0);
        self.reader = Reader::new(audio_rate);
        self.meter = crate::FrameMeter::new(audio_rate, self.channel_hz as u64, WINDOW_S)
            .keyed_as(common::Modulation::Ask);
        let mut out = i.spec.with_kind(PortKind::Packets);
        out.center = common::Hz(self.channel_hz as u64);
        out.bandwidth = CHANNEL_WIDTH_HZ;
        out.rate = 0.0;
        Ok(out)
    }

    fn process(&mut self, i: &Payload, o: &mut Payload, _c: &mut NodeCtx<'_>) -> Result<()> {
        let Some(iq) = i.as_iq() else { return Ok(()) };
        self.mixed.clear();
        self.mixer.process(iq, &mut self.mixed);
        self.narrow.clear();
        self.decim.process(&self.mixed, &mut self.narrow);
        self.meter.feed(&self.narrow);
        self.frames.clear();
        self.reader.push(&self.narrow, &mut self.frames);
        self.bearings += self.frames.len() as u64;
        let read = self.meter.packets(self.frames.drain(..), common::packet::now_us());
        o.packets_mut().extend(read);
        Ok(())
    }

    fn reset(&mut self) {
        self.mixer.reset();
        self.decim.reset();
        self.meter.reset();
    }

    fn acquisition(&self) -> Option<pipeline::Acquisition> {
        Some(match self.bearings {
            0 => pipeline::Acquisition::Searching,
            _ => pipeline::Acquisition::Locked,
        })
    }
}

impl Protocol for Vor {
    fn arrives(&self) -> crate::protocol::Arrives {
        crate::protocol::Arrives::Continuously
    }

    fn id(&self) -> &'static str {
        Signal::id(self)
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
    fn shape(&self) -> Shape {
        Signal::shape(self)
    }
    fn default_hz(&self) -> f64 {
        Signal::default_hz(self)
    }
    fn frame_claim(&self) -> FrameClaim {
        FrameClaim::Tagged
    }
    fn keys(&self) -> Option<common::Modulation> {
        Some(common::Modulation::Ask)
    }
    fn stated(&self, p: &common::packet::Packet) -> Option<Vec<common::packet::Proto>> {
        vor::read(p.bytes()).map(|d| vec![d])
    }
    fn stage_label(&self, hz: f64) -> String {
        format!("{:.2} VOR", hz / 1e6)
    }
    fn marks(&self, hz: f64) -> Vec<Mark> {
        vec![Mark { hz, width_hz: CHANNEL_WIDTH_HZ, label: "VOR".into() }]
    }
    fn chain(&self, at: Placed) -> Vec<NodeSpec> {
        vec![NodeSpec::new(DESC.name).f(CHANNEL_HZ, at.center_hz)]
    }
}

const CHANNEL_HZ: &str = "channel_hz";

pub const DESC: StageDesc = StageDesc {
    name: "vor",
    summary: "One VOR beacon: the radial from its two 30 Hz tones, and its Morse identifier",
    category: Category::Decode,
    feeds_bus: true,
};

pub fn build(s: &Settings) -> Result<Box<dyn pipeline::node::Node>> {
    Ok(Box::new(VorNode::new(s.f64_or(CHANNEL_HZ, DEFAULT_HZ))))
}
