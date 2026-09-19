//! Morse as a graph node: a channel of the span in, what the operator sent
//! out.
//!
//! The receiver has been able to send Morse for longer than it has been able
//! to hear it, and this is the middle that was missing: the channel is mixed
//! so the keyed carrier lands at an audible pitch, [`dsp::cw::CwDetector`]
//! tracks that pitch and hands back the marks and gaps, and
//! [`decode::morse::decode`] reads them as the letters they are. Nothing here
//! knows the speed, because the decoder measures the dot off the burst.
//!
//! # Which side of the dial
//!
//! A CW signal is a carrier, so the audio is the difference between it and
//! where the operator is tuned, and the sign of that difference is lost when
//! the channel becomes real audio. A station [`PITCH_HZ`] below the dial and
//! one that far above it both arrive at the same pitch and both are read.
//! That is what a receiver without a sideband filter does, and for a mode
//! whose whole bandwidth is a few tens of hertz it is a feature: the operator
//! tunes near enough and the tracker finds the rest.
//!
//! What is published is one message per transmission, with the speed it was
//! sent at. A person tapped it out and addressed it to somebody, so it is
//! `written` and belongs in the message view rather than the packet list.

use crate::NodeSpec;
use crate::protocol::{FrameClaim, Mark, Placed, Placement, Protocol, Shape};
use common::Result;
pub use decode::morse::ENVELOPE;
pub use decode::morse::TAG;
pub use decode::morse::decoded;
pub use decode::morse::{MIN_CHARS, MIN_FIT, MIN_KNOWN};
pub use decode::morse::{config, framed};
use dsp::cw::CwDetector;
use dsp::{FirDecim, Mixer};
use identify::Signal;
pub use identify::morse::AUDIO_HZ;
pub use identify::morse::CHANNEL_WIDTH_HZ;
pub use identify::morse::DEFAULT_HZ;
pub use identify::morse::Morse;
pub use identify::morse::REACH_HZ;
pub use identify::morse::{EDGE_HZ, PITCH_HZ};
use pipeline::event::Decoded;
use pipeline::node::{NodeCtx, PortSpec, Simple};
use pipeline::param::{Param, ParamValue};
use pipeline::port::{Payload, PortKind, StreamSpec};
use pipeline::registry::{Category, Settings, SettingsExt, StageDesc};

pub struct MorseNode {
    channel_hz: f64,
    rate: f64,
    factor: usize,
    mixer: Mixer,
    decim: FirDecim,
    det: CwDetector,
    mixed: Vec<common::C32>,
    narrow: Vec<common::C32>,
    audio: Vec<f32>,
    packages: Vec<common::Package>,
    meter: crate::FrameMeter,
    runs: u64,
}

impl Default for MorseNode {
    fn default() -> Self {
        Self::new(DEFAULT_HZ)
    }
}

impl MorseNode {
    pub fn new(channel_hz: f64) -> Self {
        Self {
            channel_hz,
            // All replaced at negotiation, when the real rate is known.
            rate: AUDIO_HZ,
            factor: 1,
            mixer: Mixer::new(0.0, 1.0),
            decim: FirDecim::design_band(AUDIO_HZ, 1, PITCH_HZ + REACH_HZ, EDGE_HZ, 60.0),
            det: CwDetector::new(AUDIO_HZ, config(PITCH_HZ, REACH_HZ)),
            mixed: Vec::new(),
            narrow: Vec::new(),
            audio: Vec::new(),
            packages: Vec::new(),
            meter: crate::FrameMeter::new(AUDIO_HZ, channel_hz as u64, 2.0),
            runs: 0,
        }
    }

    /// Transmissions published since the node was built.
    pub fn runs(&self) -> u64 {
        self.runs
    }

    /// The pitch the tracker is on, in hertz: how far the station is from
    /// where the operator left the dial.
    pub fn pitch_hz(&self) -> f64 {
        self.det.tone_hz()
    }
}

impl Simple for MorseNode {
    fn name(&self) -> &str {
        "morse"
    }

    fn negotiate(&mut self, i: &PortSpec) -> Result<StreamSpec> {
        if i.spec.kind != PortKind::Iq {
            return Err(common::Error::other("morse reads complex baseband"));
        }
        let (rate, center) = (i.spec.rate, i.spec.center.as_f64());
        if (self.channel_hz - center).abs() > rate / 2.0 - CHANNEL_WIDTH_HZ / 2.0 {
            return Err(common::Error::other("morse needs its channel inside the span"));
        }
        self.rate = rate;
        self.factor = (rate / AUDIO_HZ).round().max(1.0) as usize;
        let audio_rate = rate / self.factor as f64;
        if audio_rate < 2.0 * (PITCH_HZ + REACH_HZ) {
            return Err(common::Error::other("morse needs room for the beat note"));
        }
        // The carrier is put at the beat note rather than at zero: a tone at
        // DC has no pitch to track and no way to say which side of the dial
        // it is on.
        self.mixer = Mixer::new(center - self.channel_hz + PITCH_HZ, rate);
        // Designed from both edges: what the tracker can reach survives and
        // anything 300 Hz beyond it is gone, so a strong station outside the
        // channel cannot key the envelope through the skirt.
        self.decim = FirDecim::design_band(rate, self.factor, PITCH_HZ + REACH_HZ, EDGE_HZ, 60.0);
        self.det = CwDetector::new(audio_rate, config(PITCH_HZ, REACH_HZ));
        self.meter = crate::FrameMeter::new(audio_rate, self.channel_hz as u64, 2.0);

        let mut out = i.spec.with_kind(PortKind::Frames);
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

        self.audio.clear();
        self.audio.extend(self.narrow.iter().map(|c| c.re));
        let mut packages = std::mem::take(&mut self.packages);
        packages.clear();
        self.det.process(&self.audio, &mut packages);
        for pkg in &packages {
            if let Some(bytes) = framed(pkg) {
                self.runs += 1;
                o.frames_mut().push(self.meter.frame(bytes));
            }
        }
        self.packages = packages;
        Ok(())
    }

    fn reset(&mut self) {
        self.mixer.reset();
        self.decim.reset();
        self.det.reset();
        self.meter.reset();
    }

    fn params(&self) -> Vec<Param> {
        vec![Param::float(CHANNEL_HZ, self.channel_hz, 1e5..=1e10).unit("Hz").label("Channel")]
    }

    fn set_param(&mut self, name: &str, v: ParamValue) -> Result<()> {
        match name {
            CHANNEL_HZ => {
                self.channel_hz = v.as_f64().unwrap_or(self.channel_hz);
                Ok(())
            }
            _ => Err(common::Error::other(format!("morse: unknown parameter {name:?}"))),
        }
    }
}

impl Protocol for Morse {
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

    /// Not `cw`, which is already the audio mode that puts a beat note on
    /// the speaker: an operator asking for that wants to listen, and one
    /// asking for this wants it read.

    /// The front end writes its tag and the dot length in front of the text
    /// it read, which is the only thing that separates a line of Morse from
    /// any other decoder's ASCII on the bus.
    fn frame_claim(&self) -> FrameClaim {
        FrameClaim::Tagged
    }
    fn read_frame(&self, p: &common::Packet, bytes: &[u8]) -> Option<Vec<Decoded>> {
        decoded(bytes, common::Hz(p.center_hz())).map(|d| vec![d])
    }
    fn stage_label(&self, hz: f64) -> String {
        format!("{:.4} MORSE", hz / 1e6)
    }
    fn marks(&self, hz: f64) -> Vec<Mark> {
        vec![Mark { hz, width_hz: CHANNEL_WIDTH_HZ, label: "MORSE".into() }]
    }
    fn chain(&self, at: Placed) -> Vec<NodeSpec> {
        vec![NodeSpec::new(DESC.name).f(CHANNEL_HZ, at.center_hz)]
    }
}

/// The carrier this stage is pointed at.
const CHANNEL_HZ: &str = "channel_hz";

pub const DESC: StageDesc = StageDesc {
    name: "morse",
    summary: "One CW channel: a keyed carrier read as the letters it spells",
    category: Category::Decode,
    feeds_bus: true,
};

pub fn build(s: &Settings) -> Result<Box<dyn pipeline::node::Node>> {
    Ok(Box::new(MorseNode::new(s.f64_or(CHANNEL_HZ, DEFAULT_HZ))))
}

#[cfg(test)]
mod tests {
    use super::*;
    use common::{C32, Hz};

    const OVER: &str = "CQ CQ DE MI0ABC K";

    fn spec(rate: f64, center: f64) -> PortSpec {
        PortSpec { spec: StreamSpec::iq(rate, Hz(center as u64)), latency: 0 }
    }

    /// A station keying `text` at `wpm`, `offset_hz` from where the receiver
    /// is tuned, with noise throughout and half a second of it either side.
    ///
    /// `jitter` is a fist rather than a keyer: each element is stretched or
    /// squeezed by up to that fraction, which is what a hand does.
    fn keyed(
        text: &str,
        wpm: f32,
        rate: f64,
        offset_hz: f64,
        amp: f32,
        noise: f32,
        jitter: f32,
    ) -> Vec<C32> {
        let mut seed = 0x51ed_2b3c_9e17_44a1u64;
        let mut rng = move || {
            seed ^= seed << 13;
            seed ^= seed >> 7;
            seed ^= seed << 17;
            (seed >> 40) as f32 / 8_388_608.0 - 1.0
        };
        let pkg = decode::morse::encode(text, wpm);
        let samples = |us: f32| (us as f64 * rate / 1e6).round() as usize;
        let mut out: Vec<C32> = Vec::new();
        let mut phase = 0.0f64;
        let key = |out: &mut Vec<C32>,
                   n: usize,
                   on: bool,
                   phase: &mut f64,
                   rng: &mut dyn FnMut() -> f32| {
            for _ in 0..n {
                *phase += std::f64::consts::TAU * offset_hz / rate;
                let c = match on {
                    true => C32::new(amp * phase.cos() as f32, amp * phase.sin() as f32),
                    false => C32::new(0.0, 0.0),
                };
                out.push(c + C32::new(noise * rng(), noise * rng()));
            }
        };
        key(&mut out, samples(500_000.0), false, &mut phase, &mut rng);
        for (i, p) in pkg.pulses.iter().enumerate() {
            // Alternating, so a hand that runs its elements together on one
            // letter drags them out on the next.
            let skew = 1.0 + if i % 2 == 0 { jitter } else { -jitter };
            key(&mut out, samples(p.mark as f32 * skew), true, &mut phase, &mut rng);
            key(&mut out, samples(p.gap as f32 * skew), false, &mut phase, &mut rng);
        }
        // Long enough for the silence that ends a transmission to pass, so
        // the node publishes without being flushed, as a live receiver does.
        key(&mut out, samples(3_000_000.0), false, &mut phase, &mut rng);
        out
    }

    fn run(node: &mut MorseNode, iq: &[C32], rate: f64, center: f64) -> Vec<Vec<u8>> {
        let ins = [spec(rate, center)];
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

    /// The whole path on synthetic RF: a keyed carrier in the span, into the
    /// node, out as the text that was sent and the speed it was sent at.
    #[test]
    fn a_keyed_over_is_read_back_as_text() {
        let (rate, center) = (48_000.0, DEFAULT_HZ);
        let iq = keyed(OVER, 18.0, rate, 0.0, 0.5, 0.02, 0.0);
        let mut node = MorseNode::default();
        node.negotiate(&spec(rate, center)).unwrap();
        let frames = run(&mut node, &iq, rate, center);

        assert_eq!(frames.len(), 1, "{} transmissions off the air", frames.len());
        let d = decoded(&frames[0], Hz(center as u64)).expect("a decode");
        assert_eq!(d.text.as_deref(), Some(OVER));
        assert_eq!(d.protocol, "Morse");
        assert!(d.written, "an operator sent it");
        // 18 wpm is a 66.7 ms dot, and the detector's thresholds cost a few
        // percent of it at each edge.
        let wpm = d.field("speed").and_then(|v| v.as_f64()).expect("a speed");
        assert!((wpm - 18.0).abs() < 1.5, "{wpm:.1} wpm");
        assert_eq!(d.crc_ok, None, "nothing in Morse checks");
    }

    /// Speed is measured off the burst, not configured: the same node reads
    /// a slow sender and a fast one, and says which was which.
    #[test]
    fn the_speed_is_read_off_the_transmission() {
        let (rate, center) = (48_000.0, DEFAULT_HZ);
        for want in [8.0f32, 18.0, 30.0] {
            let iq = keyed("SOS DE EI2ABC", want, rate, 0.0, 0.5, 0.02, 0.0);
            let mut node = MorseNode::default();
            node.negotiate(&spec(rate, center)).unwrap();
            let frames = run(&mut node, &iq, rate, center);
            assert_eq!(frames.len(), 1, "{want} wpm: {} transmissions", frames.len());
            let d = decoded(&frames[0], Hz(center as u64)).expect("a decode");
            assert_eq!(d.text.as_deref(), Some("SOS DE EI2ABC"), "at {want} wpm");
            let got = d.field("speed").and_then(|v| v.as_f64()).unwrap_or(0.0) as f32;
            assert!((got - want).abs() < want * 0.1, "{want} wpm read as {got:.1}");
        }
    }

    /// A hand is not a keyer. Measured on an 18 wpm over whose elements are
    /// stretched and squeezed by turns: 10% and 15% read whole, 20% and 25%
    /// are refused by the timing test, and 30% publishes the words broken
    /// into their letters, which a person can still read and which is what
    /// copying a bad fist by ear is like.
    #[test]
    fn a_human_fist_still_reads() {
        let (rate, center) = (48_000.0, DEFAULT_HZ);
        let read = |jitter: f32| {
            let iq = keyed(OVER, 18.0, rate, 0.0, 0.5, 0.02, jitter);
            let mut node = MorseNode::default();
            node.negotiate(&spec(rate, center)).unwrap();
            let frames = run(&mut node, &iq, rate, center);
            frames.first().and_then(|b| decoded(b, Hz(center as u64))).and_then(|d| d.text.clone())
        };
        assert_eq!(read(0.10).as_deref(), Some(OVER), "a 10% fist");
        assert_eq!(read(0.15).as_deref(), Some(OVER), "a 15% fist");
        assert_eq!(read(0.20), None, "a 20% fist was published anyway");
        assert_eq!(read(0.30).as_deref(), Some("CQ CQ D E MI0 A B C K"), "a 30% fist");
    }

    /// Nobody is tuned exactly, and the tracker is what makes that bearable.
    /// Measured: a station within 600 Hz of the dial reads whole either side
    /// of it, and in fact reads to +800 and -900 before the channel filter
    /// takes it. What matters more is what happens past that: at 900 Hz and
    /// beyond, what reaches the envelope through the skirt is a string of
    /// blips, and the timing test refuses it rather than publishing the
    /// dozen letter Es it spells.
    #[test]
    fn a_mistuned_station_still_reads() {
        let (rate, center) = (48_000.0, DEFAULT_HZ);
        for offset in [-600.0, -200.0, 0.0, 200.0, 600.0] {
            let iq = keyed("SOS DE EI2ABC", 18.0, rate, offset, 0.5, 0.02, 0.0);
            let mut node = MorseNode::default();
            node.negotiate(&spec(rate, center)).unwrap();
            let frames = run(&mut node, &iq, rate, center);
            assert_eq!(frames.len(), 1, "{offset} Hz off: {} transmissions", frames.len());
            let d = decoded(&frames[0], Hz(center as u64)).expect("a decode");
            assert_eq!(d.text.as_deref(), Some("SOS DE EI2ABC"), "{offset} Hz off");
            let pitch = node.pitch_hz();
            assert!((pitch - (PITCH_HZ + offset)).abs() < 30.0, "pitch {pitch:.0} at {offset} Hz");
        }
        for offset in [900.0, 1_000.0, 1_200.0] {
            let iq = keyed("SOS DE EI2ABC", 18.0, rate, offset, 0.5, 0.02, 0.0);
            let mut node = MorseNode::default();
            node.negotiate(&spec(rate, center)).unwrap();
            let frames = run(&mut node, &iq, rate, center);
            assert_eq!(frames.len(), 0, "{offset} Hz out was read as {} messages", frames.len());
        }
    }

    /// Two minutes of noise produces nothing. Morse carries no check at all,
    /// so what refuses a transmission is the level test, the element count
    /// and whether the elements spell letters.
    #[test]
    fn noise_produces_no_transmissions() {
        let (rate, center) = (48_000.0, DEFAULT_HZ);
        let mut seed = 0xfeed_face_dead_beefu64;
        let mut rng = move || {
            seed ^= seed << 13;
            seed ^= seed >> 7;
            seed ^= seed << 17;
            (seed >> 40) as f32 / 8_388_608.0 - 1.0
        };
        let iq: Vec<C32> =
            (0..(rate as usize * 120)).map(|_| C32::new(0.1 * rng(), 0.1 * rng())).collect();
        let mut node = MorseNode::default();
        node.negotiate(&spec(rate, center)).unwrap();
        let frames = run(&mut node, &iq, rate, center);
        assert_eq!(frames.len(), 0, "{} transmissions out of two minutes of noise", frames.len());
    }

    /// A frame with somebody else's bytes in it is not this protocol's, and
    /// the text of one that is comes back whole.
    #[test]
    fn only_a_tagged_frame_is_claimed() {
        assert!(decoded(b"CQ CQ DE MI0ABC", Hz(0)).is_none());
        assert!(decoded(&TAG, Hz(0)).is_none(), "a tag with no text");
        let mut bytes = TAG.to_vec();
        bytes.extend_from_slice(&66_667u32.to_le_bytes());
        bytes.extend_from_slice(b"SOS");
        let d = decoded(&bytes, Hz(DEFAULT_HZ as u64)).expect("a decode");
        assert_eq!(d.text.as_deref(), Some("SOS"));
        assert_eq!(d.detail.as_deref(), Some("18 wpm"));
    }

    #[test]
    fn the_node_refuses_a_span_without_its_channel() {
        let mut n = MorseNode::default();
        assert!(n.negotiate(&spec(48_000.0, DEFAULT_HZ)).is_ok());
        assert!(n.negotiate(&spec(48_000.0, DEFAULT_HZ + 30_000.0)).is_err());
        assert!(n.negotiate(&spec(2_500.0, DEFAULT_HZ)).is_err(), "no room for the beat note");
    }
}
