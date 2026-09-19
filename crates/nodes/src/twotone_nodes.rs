//! Two-tone sequential paging as a graph node: an FM channel in, the pair
//! that was sent out.
//!
//! The channel is ordinary FM, so the node mixes it down, filters it and
//! discriminates it exactly as a listening channel would; the address is
//! then in the audio as two tones held in turn, which [`dsp::tone::ToneRuns`]
//! finds and [`decode::twotone`] reads.
//!
//! Nothing on the air says whose pager a pair belongs to, so the name comes
//! from a list the operator keeps on this stage. A page with no match is
//! still published, with its tones: the tones are the address whether or not
//! anybody here knows the addressee.

use crate::NodeSpec;
use crate::protocol::{FrameClaim, Mark, Placed, Placement, Protocol, Shape};
use common::Result;
pub use decode::twotone::TAG;
pub use decode::twotone::decoded;
pub use decode::twotone::framed;
use decode::twotone::{Pagers, Sequential};
use dsp::tone::{RunConfig, ToneRuns};
use dsp::{FirDecim, FmDemod, Mixer};
use identify::Signal;
pub use identify::twotone::CHANNEL_WIDTH_HZ;
pub use identify::twotone::DEFAULT_HZ;
pub use identify::twotone::TwoTone;
pub use identify::twotone::{AUDIO_HZ, DEVIATION_HZ};
use pipeline::event::Decoded;
use pipeline::node::{NodeCtx, PortSpec, Simple};
use pipeline::param::{Param, ParamValue};
use pipeline::port::{Payload, PortKind, StreamSpec};
use pipeline::registry::{Category, Settings, SettingsExt, StageDesc};

const CHANNEL_HZ: &str = "channel_hz";
const PAGERS: &str = "pagers";

pub struct TwoToneNode {
    channel_hz: f64,
    mixer: Mixer,
    decim: FirDecim,
    fm: FmDemod,
    tones: ToneRuns,
    pages: Sequential,
    who: Pagers,
    list: String,
    mixed: Vec<common::C32>,
    narrow: Vec<common::C32>,
    audio: Vec<f32>,
    runs: Vec<dsp::tone::Run>,
    meter: crate::FrameMeter,
    read: u64,
}

impl Default for TwoToneNode {
    fn default() -> Self {
        Self::new(DEFAULT_HZ)
    }
}

impl TwoToneNode {
    pub fn new(channel_hz: f64) -> Self {
        Self {
            channel_hz,
            // All replaced at negotiation, when the real rate is known.
            mixer: Mixer::new(0.0, 1.0),
            decim: FirDecim::design_hz(AUDIO_HZ, 1, CHANNEL_WIDTH_HZ / 2.0, 60.0),
            fm: FmDemod::new(AUDIO_HZ, DEVIATION_HZ),
            tones: ToneRuns::new(AUDIO_HZ, RunConfig::default()),
            pages: Sequential::default(),
            who: Pagers::default(),
            list: String::new(),
            mixed: Vec::new(),
            narrow: Vec::new(),
            audio: Vec::new(),
            runs: Vec::new(),
            meter: crate::FrameMeter::new(AUDIO_HZ, channel_hz as u64, 2.0),
            read: 0,
        }
    }

    /// Pages published since the node was built.
    pub fn read(&self) -> u64 {
        self.read
    }

    /// The tone being heard now, for a readout: a page is four seconds long
    /// and looks like nothing at all until it ends.
    pub fn hearing(&self) -> Option<f64> {
        self.tones.open().map(|r| r.hz)
    }
}

impl Simple for TwoToneNode {
    fn name(&self) -> &str {
        "twotone"
    }

    fn negotiate(&mut self, i: &PortSpec) -> Result<StreamSpec> {
        if i.spec.kind != PortKind::Iq {
            return Err(common::Error::other("two-tone paging reads complex baseband"));
        }
        let (rate, center) = (i.spec.rate, i.spec.center.as_f64());
        if (self.channel_hz - center).abs() > rate / 2.0 - CHANNEL_WIDTH_HZ / 2.0 {
            return Err(common::Error::other("two-tone paging needs its channel inside the span"));
        }
        let factor = (rate / AUDIO_HZ).round().max(1.0) as usize;
        let audio_rate = rate / factor as f64;
        if audio_rate < 2.0 * RunConfig::default().band_hz.1 {
            return Err(common::Error::other("two-tone paging needs room for a 3 kHz tone"));
        }
        self.mixer = Mixer::new(center - self.channel_hz, rate);
        self.decim = FirDecim::design_hz(rate, factor, CHANNEL_WIDTH_HZ / 2.0, 60.0);
        self.fm = FmDemod::new(audio_rate, DEVIATION_HZ);
        self.tones = ToneRuns::new(audio_rate, RunConfig::default());
        self.pages.reset();
        self.meter = crate::FrameMeter::new(audio_rate, self.channel_hz as u64, 2.0);

        let mut out = i.spec.with_kind(PortKind::Frames);
        out.center = common::Hz(self.channel_hz as u64);
        out.bandwidth = CHANNEL_WIDTH_HZ.min(rate);
        Ok(out)
    }

    fn params(&self) -> Vec<Param> {
        vec![
            Param::float(CHANNEL_HZ, self.channel_hz, 1e5..=1e10).unit("Hz").label("Channel"),
            Param::text(PAGERS, self.list.clone()).label("Pagers, one per line: name = A/B"),
        ]
    }

    fn set_param(&mut self, name: &str, v: ParamValue) -> Result<()> {
        match name {
            CHANNEL_HZ => {
                self.channel_hz = v.as_f64().unwrap_or(self.channel_hz);
                Ok(())
            }
            PAGERS => {
                self.list = v.as_str().unwrap_or_default().to_string();
                self.who = Pagers::parse(&self.list);
                Ok(())
            }
            _ => Err(common::Error::other(format!("twotone: unknown parameter {name:?}"))),
        }
    }

    fn readings(&self) -> Vec<(String, String)> {
        let mut out = vec![("read".into(), self.read.to_string())];
        if let Some(hz) = self.hearing() {
            out.push(("tone".into(), format!("{hz:.1} Hz")));
        }
        if !self.who.is_empty() {
            out.push(("pagers".into(), self.who.len().to_string()));
        }
        out
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

        let mut runs = std::mem::take(&mut self.runs);
        runs.clear();
        let audio = std::mem::take(&mut self.audio);
        self.tones.process(&audio, &mut runs);
        self.audio = audio;
        for run in &runs {
            if let Some(page) = self.pages.run(*run) {
                self.read += 1;
                let name = self.who.who(&page).map(|p| p.name.clone());
                o.frames_mut().push(self.meter.frame(framed(&page, name.as_deref())));
            }
        }
        self.runs = runs;
        Ok(())
    }

    fn reset(&mut self) {
        self.mixer.reset();
        self.decim.reset();
        self.fm.reset();
        self.tones.reset();
        self.pages.reset();
        self.meter.reset();
    }
}

impl Protocol for TwoTone {
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

    /// The front end writes its tag in front of the tones, which is the only
    /// thing that separates a page from any other decoder's text: two tones
    /// carry no check sequence and no address anybody else would recognise.
    fn frame_claim(&self) -> FrameClaim {
        FrameClaim::Tagged
    }
    fn read_frame(&self, p: &common::Packet, bytes: &[u8]) -> Option<Vec<Decoded>> {
        decoded(bytes, common::Hz(p.center_hz())).map(|d| vec![d])
    }
    fn stage_label(&self, hz: f64) -> String {
        format!("{:.4} 2-TONE", hz / 1e6)
    }
    fn marks(&self, hz: f64) -> Vec<Mark> {
        vec![Mark { hz, width_hz: CHANNEL_WIDTH_HZ, label: "2-TONE".into() }]
    }
    fn chain(&self, at: Placed) -> Vec<NodeSpec> {
        vec![NodeSpec::new(DESC.name).f(CHANNEL_HZ, at.center_hz)]
    }
}

pub const DESC: StageDesc = StageDesc {
    name: "twotone",
    summary: "One FM channel: the tone pair that opens a fire or ambulance pager",
    category: Category::Decode,
    feeds_bus: true,
};

pub fn build(s: &Settings) -> Result<Box<dyn pipeline::node::Node>> {
    let mut n = TwoToneNode::new(s.f64_or(CHANNEL_HZ, DEFAULT_HZ));
    let list = s.get(PAGERS).and_then(|v| v.as_str().map(String::from)).unwrap_or_default();
    if !list.is_empty() {
        Simple::set_param(&mut n, PAGERS, ParamValue::Text(list))?;
    }
    Ok(Box::new(n))
}

#[cfg(test)]
mod tests {
    use super::*;
    use common::{C32, Hz};

    const RATE: f64 = 48_000.0;

    fn spec(rate: f64, center: f64) -> PortSpec {
        PortSpec { spec: StreamSpec::iq(rate, Hz(center as u64)), latency: 0 }
    }

    /// A page keyed onto the channel: the tones as audio, FM modulated, with
    /// the carrier up throughout and silence either side.
    fn keyed(parts: &[(f64, f64)], noise: f32) -> Vec<C32> {
        let mut audio: Vec<f32> = Vec::new();
        let mut phase = 0.0f64;
        let mut hold = |hz: f64, seconds: f64, audio: &mut Vec<f32>| {
            for _ in 0..(RATE * seconds) as usize {
                phase += std::f64::consts::TAU * hz / RATE;
                audio.push(0.5 * phase.sin() as f32);
            }
        };
        audio.extend(vec![0.0; (RATE * 0.2) as usize]);
        for &(hz, seconds) in parts {
            hold(hz, seconds, &mut audio);
        }
        audio.extend(vec![0.0; (RATE * 0.3) as usize]);

        let mut seed = 0x51ed_2701_9ab3_c4d5u64;
        let mut rng = || {
            seed ^= seed << 13;
            seed ^= seed >> 7;
            seed ^= seed << 17;
            (seed >> 40) as f32 / 8_388_608.0 - 1.0
        };
        let mut carrier = 0.0f64;
        audio
            .iter()
            .map(|&a| {
                carrier += std::f64::consts::TAU * (f64::from(a) * DEVIATION_HZ) / RATE;
                C32::new(carrier.cos() as f32 + noise * rng(), carrier.sin() as f32 + noise * rng())
            })
            .collect()
    }

    fn node(center: f64, pagers: &str) -> TwoToneNode {
        let mut n = TwoToneNode::new(center);
        n.negotiate(&spec(RATE, center)).unwrap();
        if !pagers.is_empty() {
            Simple::set_param(&mut n, PAGERS, ParamValue::Text(pagers.into())).unwrap();
        }
        n
    }

    fn run(node: &mut TwoToneNode, iq: &[C32], center: f64) -> Vec<Vec<u8>> {
        let ins = [spec(RATE, center)];
        let tags = Vec::new();
        let mut frames = Vec::new();
        for block in iq.chunks(4096) {
            let input = Payload::Iq(block.to_vec());
            let mut out = Payload::Frames(Vec::new());
            let (mut events, mut new_tags) = (Vec::new(), Vec::new());
            let mut ctx = NodeCtx::new(0, &ins, &tags, &mut events, &mut new_tags);
            node.process(&input, &mut out, &mut ctx).unwrap();
            if let Payload::Frames(f) = out {
                frames.extend(f.into_iter().map(|x| x.bytes));
            }
        }
        frames
    }

    #[test]
    fn the_node_refuses_a_span_without_its_channel() {
        let mut n = TwoToneNode::default();
        assert!(n.negotiate(&spec(2_400_000.0, DEFAULT_HZ)).is_ok());
        assert!(n.negotiate(&spec(2_400_000.0, 160_000_000.0)).is_err());
        assert!(n.negotiate(&spec(20_000.0, DEFAULT_HZ)).is_err());
    }

    /// The whole path on synthetic RF: a Quick Call II page keyed onto an FM
    /// channel, into the node, out as the pair that was sent and the pager
    /// the operator's list says it opens.
    #[test]
    fn a_keyed_page_is_read_back_as_its_pair() {
        let mut n = node(DEFAULT_HZ, "Station 3 = 947.3/332.5\n");
        let frames = run(&mut n, &keyed(&[(947.3, 1.0), (332.5, 3.0)], 0.0), DEFAULT_HZ);
        assert_eq!(frames.len(), 1, "{} pages off the air", frames.len());
        assert_eq!(n.read(), 1);

        let d = decoded(&frames[0], Hz(DEFAULT_HZ as u64)).expect("a decode");
        assert_eq!(d.protocol, "Two-tone page");
        // The tone reading is a few hertz off, which is what a 25 ms window
        // measures a tone to; the pager list matches within 1.5%.
        let a = d.field("tone_a_hz").and_then(|v| v.as_f64()).expect("an A tone");
        let b = d.field("tone_b_hz").and_then(|v| v.as_f64()).expect("a B tone");
        assert!((a - 947.3).abs() < 5.0, "A read as {a:.1}");
        assert!((b - 332.5).abs() < 5.0, "B read as {b:.1}");
        let a_s = d.field("tone_a_s").and_then(|v| v.as_f64()).unwrap_or(0.0);
        let b_s = d.field("tone_b_s").and_then(|v| v.as_f64()).unwrap_or(0.0);
        assert!((a_s - 1.0).abs() < 0.1, "A held {a_s:.2} s");
        assert!((b_s - 3.0).abs() < 0.1, "B held {b_s:.2} s");
        assert_eq!(d.field("pager"), Some(&common::Value::Text("Station 3".into())));
        assert_eq!(d.identity.as_ref().map(|i| i.name.clone()), Some(Some("Station 3".into())));
        assert!(!d.written, "a tone sender wrote nothing");
        assert_eq!(d.crc_ok, None, "two tones carry no check");
    }

    /// A pair nobody listed is still a page: the tones are the address
    /// whether or not the operator knows whose they are.
    #[test]
    fn an_unlisted_pair_is_still_published() {
        let mut n = node(DEFAULT_HZ, "");
        let frames = run(&mut n, &keyed(&[(600.9, 1.0), (1153.4, 3.0)], 0.0), DEFAULT_HZ);
        assert_eq!(frames.len(), 1);
        let d = decoded(&frames[0], Hz(DEFAULT_HZ as u64)).expect("a decode");
        assert_eq!(d.field("pager"), None);
        assert_eq!(d.identity, None, "an unlisted pair names nobody");
        let tones = d.field("tones").map(|v| v.to_string()).unwrap_or_default();
        assert!(tones.starts_with("60"), "{tones}");
    }

    /// A long tone is an all-call, and it is published as one rather than as
    /// half a page.
    #[test]
    fn a_long_tone_is_published_as_a_group_call() {
        let mut n = node(DEFAULT_HZ, "Fire brigade = 1122.5/1153.4\n");
        let frames = run(&mut n, &keyed(&[(1153.4, 8.0)], 0.0), DEFAULT_HZ);
        assert_eq!(frames.len(), 1);
        let d = decoded(&frames[0], Hz(DEFAULT_HZ as u64)).expect("a decode");
        assert_eq!(d.field("call"), Some(&common::Value::Text("group".into())));
        assert_eq!(d.field("pager"), Some(&common::Value::Text("Fire brigade".into())));
        let seconds = d.field("tone_s").and_then(|v| v.as_f64()).unwrap_or(0.0);
        assert!((seconds - 8.0).abs() < 0.1, "held {seconds:.2} s");
    }

    /// Somebody talking on the channel is not a page, which is the whole
    /// risk: a dispatch channel carries speech all day and the tone test is
    /// what stops every over becoming a page.
    #[test]
    fn speech_on_the_channel_is_not_a_page() {
        let mut phase = [0.0f64; 11];
        let mut audio: Vec<f32> = Vec::new();
        for i in 0..(RATE * 20.0) as usize {
            let t = i as f64 / RATE;
            let pitch = 190.0 + 40.0 * (std::f64::consts::TAU * 2.0 * t).sin();
            let mut v = 0.0;
            for h in 1..=10 {
                phase[h] += std::f64::consts::TAU * pitch * h as f64 / RATE;
                v += phase[h].sin() / h as f64;
            }
            audio.push((v * 0.4) as f32);
        }
        let mut carrier = 0.0f64;
        let iq: Vec<C32> = audio
            .iter()
            .map(|&a| {
                carrier += std::f64::consts::TAU * (f64::from(a) * DEVIATION_HZ) / RATE;
                C32::new(carrier.cos() as f32, carrier.sin() as f32)
            })
            .collect();
        let mut n = node(DEFAULT_HZ, "");
        let frames = run(&mut n, &iq, DEFAULT_HZ);
        assert_eq!(frames.len(), 0, "twenty seconds of speech made {} pages", frames.len());
    }

    /// Two minutes of noise produces nothing.
    #[test]
    fn noise_produces_no_pages() {
        let mut seed = 0x0bad_f00d_1234_5678u64;
        let mut rng = || {
            seed ^= seed << 13;
            seed ^= seed >> 7;
            seed ^= seed << 17;
            (seed >> 40) as f32 / 8_388_608.0 - 1.0
        };
        let iq: Vec<C32> =
            (0..(RATE * 120.0) as usize).map(|_| C32::new(rng() * 0.3, rng() * 0.3)).collect();
        let mut n = node(DEFAULT_HZ, "Station 3 = 947.3/332.5\n");
        let frames = run(&mut n, &iq, DEFAULT_HZ);
        assert_eq!(frames.len(), 0, "noise made {} pages", frames.len());
        assert_eq!(n.read(), 0);
    }
}
