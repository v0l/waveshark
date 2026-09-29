use crate::NodeSpec;
use crate::protocol::{FrameClaim, Mark, Placed, Placement, Protocol, Shape};
use common::Result;
use decode::{navtex, sitor};
use dsp::afsk::Symbol;
use dsp::fsk::TonePair;
use dsp::{FirDecim, Mixer};
use identify::Signal;
pub use identify::navtex::{AUDIO_HZ, BAUD, CHANNEL_WIDTH_HZ, DEFAULT_HZ, Navtex, SHIFT_HZ};
use pipeline::node::{NodeCtx, PortSpec, Simple};
use pipeline::param::{Param, ParamValue};
use pipeline::port::{Payload, PortKind, StreamSpec};
use pipeline::registry::{Category, Settings, SettingsExt, StageDesc};

pub struct NavtexNode {
    channel_hz: f64,
    rate: f64,
    factor: usize,
    mixer: Mixer,
    decim: FirDecim,
    tones: TonePair,
    fec: sitor::Fec,
    bulletins: navtex::Assembler,
    mixed: Vec<common::C32>,
    narrow: Vec<common::C32>,
    symbols: Vec<Symbol>,
    reads: Vec<sitor::Read>,
    meter: crate::FrameMeter,
    bulletins_read: u64,
}

impl Default for NavtexNode {
    fn default() -> Self {
        Self::new(DEFAULT_HZ)
    }
}

impl NavtexNode {
    pub fn new(channel_hz: f64) -> Self {
        Self {
            channel_hz,
            rate: AUDIO_HZ,
            factor: 1,
            mixer: Mixer::new(0.0, 1.0),
            decim: FirDecim::design_hz(AUDIO_HZ, 1, CHANNEL_WIDTH_HZ / 2.0, 60.0),
            tones: TonePair::new(AUDIO_HZ, BAUD, SHIFT_HZ),
            fec: sitor::Fec::new(),
            bulletins: navtex::Assembler::new(),
            mixed: Vec::new(),
            narrow: Vec::new(),
            symbols: Vec::new(),
            reads: Vec::new(),
            meter: crate::FrameMeter::new(AUDIO_HZ, channel_hz as u64, 30.0)
                .keyed_as(common::Modulation::Fsk2),
            bulletins_read: 0,
        }
    }

    pub fn bulletins(&self) -> u64 {
        self.bulletins_read
    }

    fn rebuild(&mut self) {
        let audio_rate = self.rate / self.factor as f64;
        self.decim = FirDecim::design_hz(self.rate, self.factor, CHANNEL_WIDTH_HZ / 2.0, 60.0);
        self.tones = TonePair::new(audio_rate, BAUD, SHIFT_HZ);
        self.fec.reset();
        self.bulletins.reset();
        self.meter = crate::FrameMeter::new(audio_rate, self.channel_hz as u64, 30.0)
            .keyed_as(common::Modulation::Fsk2);
    }
}

impl Simple for NavtexNode {
    fn name(&self) -> &str {
        "navtex"
    }

    fn negotiate(&mut self, i: &PortSpec) -> Result<StreamSpec> {
        if i.spec.kind != PortKind::Iq {
            return Err(common::Error::other("navtex reads complex baseband"));
        }
        let (rate, center) = (i.spec.rate, i.spec.center.as_f64());
        if (self.channel_hz - center).abs() > rate / 2.0 - CHANNEL_WIDTH_HZ / 2.0 {
            return Err(common::Error::other("navtex needs its channel inside the span"));
        }
        self.rate = rate;
        self.factor = (rate / AUDIO_HZ).floor().max(1.0) as usize;
        self.mixer = Mixer::new(center - self.channel_hz, rate);
        self.rebuild();
        if !self.tones.usable() {
            return Err(common::Error::other("navtex needs four samples a bit"));
        }
        let mut out = i.spec.with_kind(PortKind::Packets);
        out.center = common::Hz(self.channel_hz as u64);
        out.bandwidth = CHANNEL_WIDTH_HZ.min(rate);
        Ok(out)
    }

    fn process(&mut self, i: &Payload, o: &mut Payload, _c: &mut NodeCtx<'_>) -> Result<()> {
        let Some(iq) = i.as_iq() else { return Ok(()) };
        self.mixed.clear();
        self.mixer.process(iq, &mut self.mixed);
        self.narrow.clear();
        self.decim.process(&self.mixed, &mut self.narrow);
        self.meter.feed(&self.narrow);
        self.symbols.clear();
        self.tones.process(&self.narrow, &mut self.symbols);
        let mut done = Vec::new();
        for s in &self.symbols {
            self.reads.clear();
            self.fec.push(s.mark, &mut self.reads);
            done.extend(self.reads.iter().filter_map(|r| self.bulletins.push(*r)));
        }
        for codes in done {
            self.bulletins_read += 1;
            o.packets_mut().push(self.meter.packet_now(codes));
        }
        Ok(())
    }

    fn reset(&mut self) {
        self.mixer.reset();
        self.rebuild();
    }

    fn params(&self) -> Vec<Param> {
        vec![Param::float(CHANNEL_HZ, self.channel_hz, 1e5..=1e9).unit("Hz").label("Channel")]
    }

    fn set_param(&mut self, name: &str, v: ParamValue) -> Result<()> {
        match name {
            CHANNEL_HZ => self.channel_hz = v.as_f64().unwrap_or(self.channel_hz),
            _ => return Err(common::Error::other(format!("navtex: unknown parameter {name:?}"))),
        }
        self.rebuild();
        Ok(())
    }
}

impl Protocol for Navtex {
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
        FrameClaim::Band { width_hz: 2 * CHANNEL_WIDTH_HZ as u64 }
    }
    fn stated(&self, p: &common::packet::Packet) -> Option<Vec<common::packet::Proto>> {
        let near = Signal::placement(self).covers(p.center_hz() as f64, 2.0 * CHANNEL_WIDTH_HZ);
        if !near {
            return None;
        }
        navtex::read(p.bytes()).map(|d| vec![d])
    }
    fn stage_label(&self, hz: f64) -> String {
        format!("{:.1} kHz NAVTEX", hz / 1e3)
    }
    fn marks(&self, hz: f64) -> Vec<Mark> {
        vec![Mark { hz, width_hz: CHANNEL_WIDTH_HZ, label: "NAVTEX".into() }]
    }
    fn chain(&self, at: Placed) -> Vec<NodeSpec> {
        vec![NodeSpec::new(DESC.name).f(CHANNEL_HZ, at.center_hz)]
    }
}

const CHANNEL_HZ: &str = "channel_hz";

pub const DESC: StageDesc = StageDesc {
    name: "navtex",
    summary: "One NAVTEX channel: SITOR-B at 100 baud, bulletins between ZCZC and NNNN",
    category: Category::Decode,
    feeds_bus: true,
};

pub fn build(s: &Settings) -> Result<Box<dyn pipeline::node::Node>> {
    Ok(Box::new(NavtexNode::new(s.f64_or(CHANNEL_HZ, DEFAULT_HZ))))
}

#[cfg(test)]
mod tests {
    use super::*;
    use common::{C32, Hz};

    fn spec(rate: f64, center: f64) -> PortSpec {
        PortSpec { spec: StreamSpec::iq(rate, Hz(center as u64)), latency: 0 }
    }

    fn keyed(bits: &[bool], rate: f64) -> Vec<C32> {
        let sps = rate / BAUD;
        let mut ph = 0.0f64;
        let n = (bits.len() as f64 * sps) as usize;
        (0..n)
            .map(|i| {
                let bit = bits[((i as f64) / sps) as usize];
                let hz = if bit { SHIFT_HZ / 2.0 } else { -SHIFT_HZ / 2.0 };
                ph += std::f64::consts::TAU * hz / rate;
                C32::new(0.3 * ph.cos() as f32, 0.3 * ph.sin() as f32)
            })
            .collect()
    }

    #[test]
    fn a_keyed_bulletin_comes_off_the_air_as_one_row() {
        let (rate, center) = (8_000.0, DEFAULT_HZ);
        let sent = "ZCZC EA39\nNASH POINT LIGHT, NORMAL CONDITIONS RESTORED.\nNNNN";
        let iq = keyed(&sitor::fec_bits(&sitor::encode(sent), 40), rate);
        let mut node = NavtexNode::default();
        node.negotiate(&spec(rate, center)).unwrap();
        let ins = [spec(rate, center)];
        let tags = Vec::new();
        let mut rows = Vec::new();
        for chunk in iq.chunks(4096) {
            let mut out = Payload::Packets(Vec::new());
            let (mut events, mut new_tags) = (Vec::new(), Vec::new());
            let mut ctx = NodeCtx::new(0, &ins, &tags, &mut events, &mut new_tags);
            node.process(&Payload::Iq(chunk.to_vec()), &mut out, &mut ctx).unwrap();
            if let Payload::Packets(ps) = out {
                rows.extend(ps.iter().filter_map(|p| navtex::read(p.bytes())));
            }
        }
        assert_eq!(rows.len(), 1);
        assert_eq!(rows[0].kind, "navigational_warning");
        assert_eq!(rows[0].subject.as_ref().map(|e| e.id.to_string()).as_deref(), Some("EA39"));
    }
}
