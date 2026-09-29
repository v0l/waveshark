use crate::NodeSpec;
use crate::protocol::{FrameClaim, Placed, Placement, Protocol, Shape};
use common::Result;
use decode::ysf;
pub use decode::ysf::read;
use dsp::c4fm::SymbolClock;
use dsp::fir::FirDecimReal;
use dsp::m17::rrc_taps;
use dsp::{FirDecim, FmDemod, Mixer};
use identify::Signal;
pub use identify::ysf::{
    AUDIO_HZ, BAUD, CHANNEL_WIDTH_HZ, DEFAULT_HZ, DEVIATION_HZ, RRC_ALPHA, Ysf,
};
use pipeline::node::{NodeCtx, PortSpec, Simple};
use pipeline::port::{Payload, PortKind, StreamSpec};
use pipeline::registry::{Category, Settings, SettingsExt, StageDesc};

const FILTER_CUTOFF_HZ: f64 = 7_000.0;

const KEEP_S: f64 = (4 * ysf::FRAME_DIBITS) as f64 / BAUD;

pub struct YsfNode {
    channel_hz: f64,
    mixer: Mixer,
    decim: FirDecim,
    fm: FmDemod,
    rrc: FirDecimReal,
    clock: SymbolClock,
    framer: ysf::Framer,
    mixed: Vec<common::C32>,
    narrow: Vec<common::C32>,
    audio: Vec<f32>,
    shaped: Vec<f32>,
    syms: Vec<f32>,
    meter: crate::FrameMeter,
    audio_rate: f64,
    accepted: u64,
}

impl Default for YsfNode {
    fn default() -> Self {
        Self::new(DEFAULT_HZ)
    }
}

impl YsfNode {
    pub fn new(channel_hz: f64) -> Self {
        Self {
            channel_hz,
            mixer: Mixer::new(0.0, 1.0),
            decim: FirDecim::design_hz(AUDIO_HZ, 1, FILTER_CUTOFF_HZ, 60.0),
            fm: FmDemod::new(AUDIO_HZ, DEVIATION_HZ),
            rrc: FirDecimReal::new(rrc_taps(AUDIO_HZ / BAUD, RRC_ALPHA, 8), 1),
            clock: SymbolClock::new(AUDIO_HZ, BAUD),
            framer: ysf::Framer::new(),
            mixed: Vec::new(),
            narrow: Vec::new(),
            audio: Vec::new(),
            shaped: Vec::new(),
            syms: Vec::new(),
            meter: crate::FrameMeter::new(AUDIO_HZ, channel_hz as u64, KEEP_S)
                .keyed_as(common::Modulation::Fsk4),
            audio_rate: AUDIO_HZ,
            accepted: 0,
        }
    }

    pub fn accepted(&self) -> u64 {
        self.accepted
    }

    fn packet(&mut self, frame: &ysf::Frame) -> common::packet::Packet {
        self.accepted += 1;
        let sps = self.audio_rate / BAUD;
        let start = (frame.at as f64 * sps) as u64;
        let len = (ysf::FRAME_DIBITS as f64 * sps) as usize;
        let snr_db = self.meter.snr_db_at(start, len);
        self.meter
            .packet_measured(ysf::encode_frame(frame), start, len, snr_db)
            .checked(common::packet::Integrity::Passed)
    }
}

impl Simple for YsfNode {
    fn name(&self) -> &str {
        "ysf"
    }

    fn negotiate(&mut self, i: &PortSpec) -> Result<StreamSpec> {
        if i.spec.kind != PortKind::Iq {
            return Err(common::Error::other("ysf reads complex baseband"));
        }
        let (rate, center) = (i.spec.rate, i.spec.center.as_f64());
        if (self.channel_hz - center).abs() > rate / 2.0 - CHANNEL_WIDTH_HZ / 2.0 {
            return Err(common::Error::other("ysf needs its channel inside the span"));
        }
        let factor = (rate / AUDIO_HZ).round().max(1.0) as usize;
        let audio_rate = rate / factor as f64;
        self.mixer = Mixer::new(center - self.channel_hz, rate);
        self.decim = FirDecim::design_hz(rate, factor, FILTER_CUTOFF_HZ, 60.0);
        self.fm = FmDemod::new(audio_rate, DEVIATION_HZ);
        self.rrc = FirDecimReal::new(rrc_taps(audio_rate / BAUD, RRC_ALPHA, 8), 1);
        self.clock = SymbolClock::new(audio_rate, BAUD);
        self.framer = ysf::Framer::new();
        self.audio_rate = audio_rate;
        self.meter = crate::FrameMeter::new(audio_rate, self.channel_hz as u64, KEEP_S)
            .keyed_as(common::Modulation::Fsk4);
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
        self.audio.clear();
        self.fm.process(&self.narrow, &mut self.audio);
        self.shaped.clear();
        self.rrc.process(&self.audio, &mut self.shaped);
        self.syms.clear();
        self.clock.push(&self.shaped, &mut self.syms);
        let mut frames = Vec::new();
        self.framer.push(&self.syms, &mut frames);
        for f in &frames {
            let p = self.packet(f);
            o.packets_mut().push(p);
        }
        Ok(())
    }

    fn reset(&mut self) {
        self.mixer.reset();
        self.decim.reset();
        self.fm.reset();
        self.rrc.reset();
        self.clock.reset();
        self.framer.reset();
        self.meter.reset();
    }
}

impl Protocol for Ysf {
    fn arrives(&self) -> crate::protocol::Arrives {
        crate::protocol::Arrives::InBursts
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
        Some(common::Modulation::Fsk4)
    }
    fn stated(&self, p: &common::packet::Packet) -> Option<Vec<common::packet::Proto>> {
        read(p.bytes()).map(|d| vec![d])
    }
    fn chain(&self, at: Placed) -> Vec<NodeSpec> {
        vec![NodeSpec::new(DESC.name).f(CHANNEL_HZ, at.center_hz)]
    }
}

const CHANNEL_HZ: &str = "channel_hz";

pub const DESC: StageDesc = StageDesc {
    name: "ysf",
    summary: "One System Fusion channel: C4FM at 4800 baud, frame information and callsigns",
    category: Category::Decode,
    feeds_bus: true,
};

pub fn build(s: &Settings) -> Result<Box<dyn pipeline::node::Node>> {
    Ok(Box::new(YsfNode::new(s.f64_or(CHANNEL_HZ, DEFAULT_HZ))))
}
