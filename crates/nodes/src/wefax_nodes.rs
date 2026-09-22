//! Weather fax as a graph node.
//!
//! Wiring: a chart is sent on shortwave as a sideband signal, so the node
//! mixes the channel down, demodulates upper sideband and hands the audio to
//! `decode::wefax`, which finds the phasing signal, measures the line rate
//! and paints the chart. Where a chain already has audio, from an SSB strip
//! channel or a recording, the node reads that instead and there is no
//! demodulator in this path at all.
//!
//! The dial frequency of a fax broadcast is published 1.9 kHz below the
//! carrier so that the picture lands in the middle of an ordinary sideband
//! passband, which is why the assigned frequency and the frequency to tune
//! are not the same number. What is set here is the frequency to tune.

use crate::NodeSpec;
use crate::protocol::{Placed, Placement, Protocol, Shape};
use common::{Cadence, Pixels, Result, Update, VideoFrame};
use decode::wefax;
use dsp::resample::Rational;
use dsp::ssb::{Sideband, SsbDemod};
use dsp::{FirDecim, Mixer};
use identify::Signal;
pub use identify::wefax::AUDIO_HZ;
pub use identify::wefax::CHANNEL_WIDTH_HZ;
pub use identify::wefax::DEFAULT_HZ;
pub use identify::wefax::Wefax;
pub use identify::wefax::{PASS_HIGH_HZ, PASS_LOW_HZ};
use pipeline::node::{NodeCtx, PortSpec, Simple};
use pipeline::port::{Payload, PortKind, StreamSpec};
use pipeline::registry::{Category, Settings, SettingsExt, StageDesc};

/// What a transmission is called on the video bus.
const SYSTEM: &str = "weather fax";

pub struct WefaxNode {
    channel_hz: f64,
    mixer: Mixer,
    decim: FirDecim,
    ssb: SsbDemod,
    resample: Option<Rational>,
    rx: wefax::Receiver,
    mixed: Vec<common::C32>,
    narrow: Vec<common::C32>,
    audio: Vec<f32>,
    at_rate: Vec<f32>,
    pictures: u64,
}

impl Default for WefaxNode {
    fn default() -> Self {
        Self::new(DEFAULT_HZ)
    }
}

impl WefaxNode {
    pub fn new(channel_hz: f64) -> Self {
        Self {
            channel_hz,
            mixer: Mixer::new(0.0, 1.0),
            decim: FirDecim::design_hz(AUDIO_HZ, 1, CHANNEL_WIDTH_HZ / 2.0, 60.0),
            ssb: SsbDemod::new(AUDIO_HZ, Sideband::Upper, PASS_LOW_HZ, PASS_HIGH_HZ),
            resample: None,
            rx: wefax::Receiver::new(AUDIO_HZ),
            mixed: Vec::new(),
            narrow: Vec::new(),
            audio: Vec::new(),
            at_rate: Vec::new(),
            pictures: 0,
        }
    }

    /// Charts finished since the node was built.
    pub fn pictures(&self) -> u64 {
        self.pictures
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
            label: self.rx.lpm().map(|l| l.label().to_string()),
            width: rows.width,
            height: rows.height,
            // The index of cooperation is the ratio of the drum's
            // circumference to its pitch, so a fax pixel is square by
            // definition and the chart is drawn at the shape it was sampled.
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

impl Simple for WefaxNode {
    fn name(&self) -> &str {
        "wefax"
    }

    fn negotiate(&mut self, i: &PortSpec) -> Result<StreamSpec> {
        match i.spec.kind {
            PortKind::Iq => {
                let (rate, center) = (i.spec.rate, i.spec.center.as_f64());
                if (self.channel_hz - center).abs() > rate / 2.0 - CHANNEL_WIDTH_HZ / 2.0 {
                    return Err(common::Error::other("wefax needs its channel inside the span"));
                }
                let (factor, resample) = dsp::resample::stage(rate, AUDIO_HZ, 4096)
                    .ok_or_else(|| common::Error::other("wefax cannot reach 44.1 kHz here"))?;
                let mid = rate / factor as f64;
                self.mixer = Mixer::new(center - self.channel_hz, rate);
                self.decim = FirDecim::design_hz(rate, factor, CHANNEL_WIDTH_HZ / 2.0, 60.0);
                self.ssb = SsbDemod::new(mid, Sideband::Upper, PASS_LOW_HZ, PASS_HIGH_HZ);
                self.resample = resample;
            }
            // Audio out of a sideband strip channel, which is how an
            // operator tuned by ear will have it.
            PortKind::Real => {
                self.resample = dsp::resample::stage(i.spec.rate.max(AUDIO_HZ), AUDIO_HZ, 4096)
                    .and_then(|(_, r)| r);
            }
            _ => return Err(common::Error::other("wefax reads baseband or audio")),
        }
        self.rx = wefax::Receiver::new(AUDIO_HZ);

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
                self.ssb.process(&self.narrow, &mut self.audio);
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
        self.ssb.reset();
        self.rx.reset();
    }
}

impl Protocol for Wefax {
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
    /// Wherever a schedule puts one: the marine and meteorological
    /// broadcasts are scattered from 2 to 20 MHz and each service has its
    /// own list, so this is a frequency an operator sets.

    fn stage_label(&self, hz: f64) -> String {
        format!("{:.3} FAX", hz / 1e6)
    }
    fn chain(&self, at: Placed) -> Vec<NodeSpec> {
        vec![NodeSpec::new(DESC.name).f(CHANNEL_HZ, at.center_hz)]
    }
    /// A chart on a sideband channel, which is how an operator tuned by ear
    /// will have it.
    fn audio_stage(&self, hz: f64) -> Option<NodeSpec> {
        Some(NodeSpec::new(DESC.name).f(CHANNEL_HZ, hz))
    }
}

/// The carrier this stage is pointed at.
const CHANNEL_HZ: &str = "channel_hz";

pub const DESC: StageDesc = StageDesc {
    name: "wefax",
    summary: "One weather fax channel: the chart as it is drawn",
    category: Category::Decode,
    feeds_bus: false,
};

pub fn build(s: &Settings) -> Result<Box<dyn pipeline::node::Node>> {
    Ok(Box::new(WefaxNode::new(s.f64_or(CHANNEL_HZ, DEFAULT_HZ))))
}

#[cfg(test)]
mod tests {
    use super::*;
    use common::Hz;
    use std::f64::consts::TAU;

    fn spec(rate: f64, center: u64) -> PortSpec {
        PortSpec { spec: StreamSpec::iq(rate, Hz(center)), latency: 0 }
    }

    /// A chart as a radio hears it: the tone swinging between 1500 and 2300
    /// Hz, sent as upper sideband on the dial frequency, which is one
    /// complex exponential at the dial plus the tone.
    ///
    /// Phasing lines first, then a picture of one flat shade, which is all
    /// the node has to lock to; what the shades come out as is settled where
    /// the decoder is tested.
    fn broadcast(phasing: usize, lines: usize, rate: f64, center: f64) -> Vec<common::C32> {
        let line = wefax::Lpm::OneTwenty.line_time() * rate;
        let offset = DEFAULT_HZ - center;
        let mut phase = 0.0f64;
        (0..((phasing + lines) as f64 * line) as usize)
            .map(|i| {
                let y = (i as f64 / line) as usize;
                let across = (i as f64 % line) / line;
                let value = match y < phasing {
                    true => match across < 0.05 {
                        true => 255.0,
                        false => 0.0,
                    },
                    false => 128.0,
                };
                let tone = 1_500.0 + value / 255.0 * 800.0;
                phase += TAU * (offset + tone) / rate;
                common::C32::new(phase.cos() as f32, phase.sin() as f32)
            })
            .collect()
    }

    /// The whole node against a synthesised broadcast: mixer, sideband
    /// demodulator, resampler and decoder, with the rate measured off the
    /// phasing signal and lines reaching the video bus as they are read.
    #[test]
    fn a_synthesised_chart_reaches_the_video_bus() {
        let (rate, center) = (96_000.0, 7_881_000.0);
        let iq = broadcast(6, 20, rate, center);
        let mut node = WefaxNode::new(DEFAULT_HZ);
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
        assert_eq!(lines, 25, "lines off a broadcast of twenty six");
        let f = frames.first().expect("a frame");
        assert_eq!(f.system, "weather fax");
        assert_eq!(f.width, 1_809);
        assert_eq!(f.pixels, Pixels::Luma8);
        assert_eq!(f.label.as_deref(), Some("120 lpm"));
        assert!(matches!(f.update, Update::Rows { first: 0 }), "{:?}", f.update);
        // The picture was sent flat at 128 counts and comes back at it,
        // which is the whole path being in tune: a sideband demodulator a
        // hundred hertz off would read a different shade.
        let last = frames.last().expect("a frame");
        let px = last.samples[last.samples.len() - 900] as i32;
        assert!((px - 128).abs() <= 6, "the picture read {px}, sent 128");
    }

    #[test]
    fn the_channel_has_to_be_inside_the_span() {
        let mut n = WefaxNode::new(DEFAULT_HZ);
        let far = PortSpec { spec: StreamSpec::iq(96_000.0, Hz(8_040_000)), latency: 0 };
        assert!(n.negotiate(&far).is_err());
        let near = PortSpec { spec: StreamSpec::iq(96_000.0, Hz(7_881_000)), latency: 0 };
        let out = n.negotiate(&near).expect("a channel in the span");
        assert_eq!(out.kind, PortKind::Video);
        assert_eq!(out.center, Hz(7_880_000));
    }

    /// A shortwave receiver's rates, and a software one's.
    #[test]
    fn an_awkward_radio_rate_is_still_accepted() {
        for rate in [96_000.0, 250_000.0, 768_000.0, 2_048_000.0, 2_400_000.0] {
            let mut n = WefaxNode::new(DEFAULT_HZ);
            let spec = PortSpec { spec: StreamSpec::iq(rate, Hz(7_880_000)), latency: 0 };
            n.negotiate(&spec).unwrap_or_else(|e| panic!("{rate} refused: {e}"));
        }
    }

    /// A chain that already has audio feeds the same node without a
    /// demodulator in front, which is what a sideband strip channel gives it.
    #[test]
    fn audio_is_read_as_well_as_baseband() {
        let mut n = WefaxNode::new(13_882_500.0);
        let mut spec = StreamSpec::iq(48_000.0, Hz(0));
        spec.kind = PortKind::Real;
        let audio = PortSpec { spec, latency: 0 };
        let out = n.negotiate(&audio).expect("audio");
        assert_eq!(out.kind, PortKind::Video);
    }
}
