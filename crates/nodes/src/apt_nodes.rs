//! NOAA APT as a graph node.
//!
//! Wiring: the downlink at 137 MHz is FM about 34 kHz wide, so the node mixes
//! the channel down, discriminates it and hands the audio to `decode::apt`,
//! which finds the sync runs and paints the lines. What reaches the video bus
//! is the lines themselves, as each is read, so a fifteen minute pass fills
//! in as it is heard rather than arriving at the end of it.
//!
//! The pictures are right way up for a southbound pass and upside down for a
//! northbound one, because the satellite is. Nothing here turns them over:
//! which way a pass went is a fact about the pass, and the operator can see
//! it.

use crate::NodeSpec;
use crate::protocol::{Placed, Placement, Protocol, Shape};
use common::{Cadence, Pixels, Result, Update, VideoFrame};
use decode::apt;
use dsp::resample::Rational;
use dsp::{FirDecim, FmDemod, Mixer};
use identify::Signal;
pub use identify::apt::AUDIO_HZ;
pub use identify::apt::Apt;
pub use identify::apt::CHANNEL_WIDTH_HZ;
pub use identify::apt::DEFAULT_HZ;
pub use identify::apt::DEVIATION_HZ;
pub use identify::apt::SATELLITES;
use pipeline::node::{NodeCtx, PortSpec, Simple};
use pipeline::port::{Payload, PortKind, StreamSpec};
use pipeline::registry::{Category, Settings, SettingsExt, StageDesc};

/// What a transmission is called on the video bus.
const SYSTEM: &str = "APT";

pub struct AptNode {
    channel_hz: f64,
    mixer: Mixer,
    decim: FirDecim,
    fm: FmDemod,
    resample: Option<Rational>,
    rx: apt::Receiver,
    mixed: Vec<common::C32>,
    narrow: Vec<common::C32>,
    audio: Vec<f32>,
    at_rate: Vec<f32>,
    pictures: u64,
}

impl Default for AptNode {
    fn default() -> Self {
        Self::new(DEFAULT_HZ)
    }
}

impl AptNode {
    pub fn new(channel_hz: f64) -> Self {
        Self {
            channel_hz,
            mixer: Mixer::new(0.0, 1.0),
            decim: FirDecim::design_hz(AUDIO_HZ, 1, CHANNEL_WIDTH_HZ / 2.0, 60.0),
            fm: FmDemod::new(AUDIO_HZ, DEVIATION_HZ),
            resample: None,
            rx: apt::Receiver::new(AUDIO_HZ),
            mixed: Vec::new(),
            narrow: Vec::new(),
            audio: Vec::new(),
            at_rate: Vec::new(),
            pictures: 0,
        }
    }

    /// Pictures finished since the node was built.
    pub fn pictures(&self) -> u64 {
        self.pictures
    }

    /// Which satellite this channel is, where it is one of the three.
    fn satellite(&self) -> Option<&'static str> {
        SATELLITES
            .iter()
            .find(|(_, hz)| (hz - self.channel_hz).abs() < CHANNEL_WIDTH_HZ / 2.0)
            .map(|(name, _)| *name)
    }

    fn publish(&mut self, rows: decode::linescan::Rows, out: &mut Vec<VideoFrame>) {
        if rows.complete {
            self.pictures += 1;
        }
        let lines = rows.lines();
        if lines == 0 {
            return;
        }
        out.push(VideoFrame {
            system: SYSTEM,
            channel_hz: self.channel_hz,
            label: self.satellite().map(str::to_string),
            width: rows.width,
            height: rows.height,
            // Square pixels: a word is about four kilometres across and a
            // line about four along the track, so the picture is drawn at the
            // shape it was sampled at.
            aspect: rows.width as f32 / rows.height as f32,
            pixels: Pixels::Luma8,
            samples: std::sync::Arc::new(rows.gray),
            lines_seen: lines,
            sequence: rows.picture,
            update: Update::Rows { first: rows.first },
            cadence: Cadence::Still,
            sent_at_us: None,
        });
    }
}

impl Simple for AptNode {
    fn name(&self) -> &str {
        "apt"
    }

    fn negotiate(&mut self, i: &PortSpec) -> Result<StreamSpec> {
        match i.spec.kind {
            PortKind::Iq => {
                let (rate, center) = (i.spec.rate, i.spec.center.as_f64());
                if (self.channel_hz - center).abs() > rate / 2.0 - CHANNEL_WIDTH_HZ / 2.0 {
                    return Err(common::Error::other("apt needs its channel inside the span"));
                }
                let (factor, resample) = dsp::resample::stage(rate, AUDIO_HZ, 4096)
                    .ok_or_else(|| common::Error::other("apt cannot reach 41.6 kHz from here"))?;
                let mid = rate / factor as f64;
                self.mixer = Mixer::new(center - self.channel_hz, rate);
                self.decim = FirDecim::design_hz(rate, factor, CHANNEL_WIDTH_HZ / 2.0, 60.0);
                self.fm = FmDemod::new(mid, DEVIATION_HZ);
                self.resample = resample;
            }
            // Audio, which is what a receiver already listening to the pass
            // on FM produces.
            PortKind::Real => {
                self.resample = dsp::resample::stage(i.spec.rate.max(AUDIO_HZ), AUDIO_HZ, 4096)
                    .and_then(|(_, r)| r);
            }
            _ => return Err(common::Error::other("apt reads baseband or audio")),
        }
        self.rx = apt::Receiver::new(AUDIO_HZ);

        let mut out = i.spec.with_kind(PortKind::Video);
        out.center = common::Hz(self.channel_hz as u64);
        Ok(out)
    }

    fn process(&mut self, i: &Payload, o: &mut Payload, _c: &mut NodeCtx<'_>) -> Result<()> {
        self.audio.clear();
        match i {
            Payload::Iq(iq) => {
                self.mixed.clear();
                self.mixer.process(iq, &mut self.mixed);
                self.narrow.clear();
                self.decim.process(&self.mixed, &mut self.narrow);
                self.fm.process(&self.narrow, &mut self.audio);
            }
            Payload::Real(a) => self.audio.extend_from_slice(a),
            _ => return Ok(()),
        }
        let audio = match self.resample.as_mut() {
            Some(r) => {
                self.at_rate.clear();
                r.process_real(&self.audio, &mut self.at_rate);
                &self.at_rate
            }
            None => &self.audio,
        };
        if let Some(rows) = self.rx.push(audio) {
            let out = o.video_mut();
            self.publish(rows, out);
        }
        Ok(())
    }

    fn reset(&mut self) {
        self.mixer.reset();
        self.decim.reset();
        self.fm.reset();
        self.rx.reset();
    }
}

impl Protocol for Apt {
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

    fn outputs(&self) -> &'static [PortKind] {
        &[PortKind::Video]
    }
    /// The three channels the birds are on and nowhere else: APT is a
    /// downlink with three transmitters in the sky.

    fn stage_label(&self, hz: f64) -> String {
        match SATELLITES.iter().find(|(_, c)| (c - hz).abs() < CHANNEL_WIDTH_HZ / 2.0) {
            Some((name, _)) => format!("{name} APT"),
            None => format!("{:.3} APT", hz / 1e6),
        }
    }
    fn chain(&self, at: Placed) -> Vec<NodeSpec> {
        vec![NodeSpec::new(DESC.name).f(CHANNEL_HZ, at.center_hz)]
    }
    /// A pass somebody is already listening to on FM, whose audio is the
    /// picture.
    fn audio_stage(&self, hz: f64) -> Option<NodeSpec> {
        Some(NodeSpec::new(DESC.name).f(CHANNEL_HZ, hz))
    }
}

/// The carrier this stage is pointed at.
const CHANNEL_HZ: &str = "channel_hz";

pub const DESC: StageDesc = StageDesc {
    name: "apt",
    summary: "One NOAA APT downlink: the pass as a picture, a line at a time",
    category: Category::Decode,
    feeds_bus: false,
};

pub fn build(s: &Settings) -> Result<Box<dyn pipeline::node::Node>> {
    Ok(Box::new(AptNode::new(s.f64_or(CHANNEL_HZ, DEFAULT_HZ))))
}

#[cfg(test)]
mod tests {
    use super::*;
    use common::Hz;
    use std::f64::consts::TAU;

    fn spec(rate: f64, center: u64) -> PortSpec {
        PortSpec { spec: StreamSpec::iq(rate, Hz(center)), latency: 0 }
    }

    /// A pass as a radio hears it: the picture on a 2400 Hz subcarrier, the
    /// subcarrier on an FM carrier at the satellite's channel.
    ///
    /// The picture is flat grey with the sync runs in front of each half
    /// line, which is all the node has to find; what the shades come out as
    /// is settled where the decoder is tested.
    fn downlink(lines: usize, rate: f64, center: f64) -> Vec<common::C32> {
        let words = lines * 2_080;
        let spw = rate / apt::WORD_RATE;
        let mut phase = 0.0f64;
        let mut carrier = 0.0f64;
        let offset = 137_100_000.0 - center;
        (0..(words as f64 * spw) as usize)
            .map(|i| {
                let word = (i as f64 / spw) as usize % 2_080;
                let half = word % 1_040;
                let cycle = apt::WORD_RATE / 1_040.0;
                let value = match half {
                    h if (h as f64) < 7.0 * cycle => match (h as f64 % cycle) < cycle / 2.0 {
                        true => 11.0,
                        false => 244.0,
                    },
                    h if h < 86 => 0.0,
                    h if h < 995 => 128.0,
                    _ => 0.0,
                } / 255.0;
                phase += TAU * apt::SUBCARRIER_HZ / rate;
                let audio = (0.1 + 0.87 * value) * phase.sin();
                // FM at the deviation the downlink uses.
                carrier += TAU * (offset + DEVIATION_HZ * audio) / rate;
                common::C32::new(carrier.cos() as f32, carrier.sin() as f32)
            })
            .collect()
    }

    /// The whole node against a synthesised downlink: mixer, discriminator,
    /// resampler and decoder, with lines reaching the video bus as they are
    /// read and the satellite named on them.
    #[test]
    fn a_synthesised_pass_reaches_the_video_bus() {
        let (rate, center) = (250_000.0, 137_200_000.0);
        let iq = downlink(8, rate, center);
        let mut node = AptNode::new(DEFAULT_HZ);
        node.negotiate(&spec(rate, center as u64)).expect("a channel in the span");

        let ins = [spec(rate, center as u64)];
        let tags = Vec::new();
        let mut frames: Vec<VideoFrame> = Vec::new();
        for block in iq.chunks(65_536) {
            let input = Payload::Iq(block.to_vec());
            let mut out = Payload::Video(Vec::new());
            let mut events = Vec::new();
            let mut new_tags = Vec::new();
            let mut ctx = NodeCtx::new(0, &ins, &tags, &mut events, &mut new_tags);
            node.process(&input, &mut out, &mut ctx).unwrap();
            if let Payload::Video(v) = out {
                frames.extend(v);
            }
        }

        let lines: usize = frames.iter().map(|f| f.lines_seen).sum();
        assert_eq!(lines, 7, "lines off a pass of eight");
        let f = frames.first().expect("a frame");
        assert_eq!(f.system, "APT");
        assert_eq!(f.width, 2_080);
        assert_eq!(f.pixels, Pixels::Luma8);
        assert_eq!(f.label.as_deref(), Some("NOAA 19"));
        assert_eq!(f.channel_hz, DEFAULT_HZ);
        assert!(matches!(f.update, Update::Rows { first: 0 }), "{:?}", f.update);
        // The picture was sent flat at 128 counts, and the line comes back
        // at that: the sync runs calibrated it.
        let px = f.samples[500] as i32;
        assert!((px - 128).abs() <= 8, "the picture read {px}, sent 128");
    }

    #[test]
    fn the_channel_has_to_be_inside_the_span() {
        let mut n = AptNode::new(DEFAULT_HZ);
        let far = PortSpec { spec: StreamSpec::iq(250_000.0, Hz(137_500_000)), latency: 0 };
        assert!(n.negotiate(&far).is_err());
        let near = PortSpec { spec: StreamSpec::iq(240_000.0, Hz(137_150_000)), latency: 0 };
        let out = n.negotiate(&near).expect("a channel in the span");
        assert_eq!(out.kind, PortKind::Video);
        assert_eq!(out.center, Hz(137_100_000));
    }

    /// The rates a radio on 137 MHz runs at, none of which divides into
    /// 41.6 kHz.
    #[test]
    fn an_awkward_radio_rate_is_still_accepted() {
        for rate in [250_000.0, 1_024_000.0, 2_048_000.0, 2_400_000.0, 8_000_000.0] {
            let mut n = AptNode::new(DEFAULT_HZ);
            let spec = PortSpec { spec: StreamSpec::iq(rate, Hz(137_100_000)), latency: 0 };
            n.negotiate(&spec).unwrap_or_else(|e| panic!("{rate} refused: {e}"));
        }
    }

    /// Each channel is named for the satellite on it, and a channel that is
    /// none of them is named for its frequency.
    #[test]
    fn a_channel_is_named_for_its_satellite() {
        assert_eq!(Apt.stage_label(137_100_000.0), "NOAA 19 APT");
        assert_eq!(Apt.stage_label(137_620_000.0), "NOAA 15 APT");
        assert_eq!(Apt.stage_label(137_912_500.0), "NOAA 18 APT");
        assert_eq!(Apt.stage_label(137_350_000.0), "137.350 APT");
        assert_eq!(AptNode::new(137_912_500.0).satellite(), Some("NOAA 18"));
        assert_eq!(AptNode::new(137_350_000.0).satellite(), None);
    }
}
