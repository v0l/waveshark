//! SSTV as a graph node.
//!
//! Wiring: the channel is ordinary narrowband FM on two metres, so the node
//! mixes it down, discriminates it and hands the audio to `decode::sstv`,
//! which finds the calibration header, reads the mode and builds the picture.
//! On the shortwave calling frequencies the same audio comes out of a
//! sideband demodulator instead, so the node reads either a baseband channel
//! or, where a chain already has one, an audio stream.
//!
//! What reaches the video bus is the lines themselves, as each is read. The
//! bus keeps the canvas they are painted into, so a transmission appears
//! line by line over its two minutes rather than in jumps, and nothing
//! rescans audio it has already read.

use crate::NodeSpec;
use crate::RealFir;
use crate::protocol::{Placed, Placement, Protocol, Shape};
use common::{Cadence, Pixels, Result, Update, VideoFrame};
use decode::sstv;
use dsp::resample::Rational;
use dsp::{FirDecim, FmDemod, Mixer};
use pipeline::node::{NodeCtx, PortSpec, Simple};
use pipeline::port::{Payload, PortKind, StreamSpec};
use pipeline::registry::{Category, Settings, SettingsExt, StageDesc};

/// What a transmission is called on the video bus, beside "analogue video".
const SYSTEM: &str = "SSTV";

/// The two metre calling frequency, which is where SSTV lives across Europe.
/// The shortwave calling frequencies are 14.230 and 7.171, and those want a
/// sideband demodulator in front rather than this node's own.
pub const DEFAULT_HZ: f64 = 144_500_000.0;

/// A 2 m FM channel.
pub const CHANNEL_WIDTH_HZ: f64 = 12_500.0;

/// The rate the picture is read at.
///
/// Not a free choice: the analysis windows are counted in samples, so the
/// same recording decoded at 22.05 kHz differs from its 44.1 kHz decode by a
/// mean of 4.5 counts a channel, and at 11.025 kHz by 6.6.
const AUDIO_HZ: f64 = 44_100.0;

/// Deviation mapped to full scale on the discriminator. Only the tone scale
/// depends on it, and the decoder reads frequencies rather than amplitudes,
/// so this need only be in the right region.
const DEVIATION_HZ: f64 = 3_000.0;

/// The band an SSTV transmission lives in: 1200 Hz for the sync pulses, 1500
/// to 2300 for the picture, and 1900 for the calibration leader.
const TONE_LOW_HZ: f64 = 900.0;
const TONE_HIGH_HZ: f64 = 2_700.0;

/// The filter in front of the decoder.
///
/// A decoder that reads the frequency of a tone cannot tell a tone from the
/// loudest thing in the band, so mains hum under the signal or hiss over it
/// becomes a pixel. This is most of what a receiver can do about that, and it
/// costs a few hundred multiplies a sample at 44.1 kHz.
fn band_taps() -> Vec<f32> {
    dsp::fir::bandpass(255, TONE_LOW_HZ / AUDIO_HZ, TONE_HIGH_HZ / AUDIO_HZ, 60.0)
}

pub struct SstvNode {
    channel_hz: f64,
    mixer: Mixer,
    decim: FirDecim,
    fm: FmDemod,
    resample: Option<Rational>,
    band: RealFir,
    rx: sstv::Receiver,
    mixed: Vec<common::C32>,
    narrow: Vec<common::C32>,
    audio: Vec<f32>,
    at_rate: Vec<f32>,
    pictures: u64,
}

impl Default for SstvNode {
    fn default() -> Self {
        Self::new(DEFAULT_HZ)
    }
}

impl SstvNode {
    pub fn new(channel_hz: f64) -> Self {
        Self {
            channel_hz,
            mixer: Mixer::new(0.0, 1.0),
            decim: FirDecim::design_hz(AUDIO_HZ, 1, CHANNEL_WIDTH_HZ / 2.0, 60.0),
            fm: FmDemod::new(AUDIO_HZ, DEVIATION_HZ),
            resample: None,
            band: RealFir::new(band_taps()),
            rx: sstv::Receiver::new(AUDIO_HZ),
            mixed: Vec::new(),
            narrow: Vec::new(),
            audio: Vec::new(),
            at_rate: Vec::new(),
            pictures: 0,
        }
    }

    /// Pictures completed since the node was built.
    pub fn pictures(&self) -> u64 {
        self.pictures
    }

    fn publish(&mut self, lines: sstv::Lines, out: &mut Vec<VideoFrame>) {
        if lines.complete {
            self.pictures += 1;
        }
        let rows = lines.rgb.len() / (lines.mode.width * 3);
        if rows == 0 {
            return;
        }
        out.push(VideoFrame {
            system: SYSTEM,
            channel_hz: self.channel_hz,
            label: Some(lines.mode.name.to_string()),
            width: lines.mode.width,
            height: lines.mode.height,
            // SSTV pictures are 4:3 whatever their pixel count, which is 320
            // by 256 in the Martin and Scottie modes.
            aspect: 4.0 / 3.0,
            pixels: Pixels::Rgb8,
            samples: std::sync::Arc::new(lines.rgb),
            // Of this batch. What the picture as a whole has is the bus's to
            // count, since the bus is what holds it.
            lines_seen: rows,
            sequence: lines.picture,
            update: Update::Rows { first: lines.first },
            cadence: Cadence::Still,
        });
    }
}

impl Simple for SstvNode {
    fn name(&self) -> &str {
        "sstv"
    }

    fn negotiate(&mut self, i: &PortSpec) -> Result<StreamSpec> {
        match i.spec.kind {
            PortKind::Iq => {
                let (rate, center) = (i.spec.rate, i.spec.center.as_f64());
                if (self.channel_hz - center).abs() > rate / 2.0 - CHANNEL_WIDTH_HZ / 2.0 {
                    return Err(common::Error::other("sstv needs its channel inside the span"));
                }
                let (factor, resample) = dsp::resample::stage(rate, AUDIO_HZ, 4096)
                    .ok_or_else(|| common::Error::other("sstv cannot reach 44.1 kHz from here"))?;
                let mid = rate / factor as f64;
                self.mixer = Mixer::new(center - self.channel_hz, rate);
                self.decim = FirDecim::design_hz(rate, factor, CHANNEL_WIDTH_HZ / 2.0, 60.0);
                self.fm = FmDemod::new(mid, DEVIATION_HZ);
                self.resample = resample;
            }
            // Real audio, which is what a sideband chain on the shortwave
            // calling frequencies produces.
            PortKind::Real => {
                self.resample = dsp::resample::stage(i.spec.rate.max(AUDIO_HZ), AUDIO_HZ, 4096)
                    .and_then(|(_, r)| r);
            }
            _ => return Err(common::Error::other("sstv reads baseband or audio")),
        }
        self.rx = sstv::Receiver::new(AUDIO_HZ);
        self.band = RealFir::new(band_taps());

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
                &mut self.at_rate
            }
            None => &mut self.audio,
        };
        self.band.process(audio);
        if let Some(p) = self.rx.push(audio) {
            let out = o.video_mut();
            self.publish(p, out);
        }
        Ok(())
    }

    fn reset(&mut self) {
        self.mixer.reset();
        self.decim.reset();
        self.fm.reset();
        self.band.reset();
        self.rx.reset();
    }
}

pub struct Sstv;

impl Protocol for Sstv {
    fn id(&self) -> &'static str {
        "sstv"
    }
    fn label(&self) -> &'static str {
        "sstv"
    }
    fn outputs(&self) -> &'static [PortKind] {
        &[PortKind::Video]
    }
    /// Wherever a picture is sent: the two metre calling frequency is the one
    /// with a channel, and the shortwave ones are worked by hand.
    fn placement(&self) -> Placement {
        Placement::Anywhere
    }
    fn shape(&self) -> Shape {
        Shape {
            widths: &[CHANNEL_WIDTH_HZ],
            min_rate_hz: CHANNEL_WIDTH_HZ,
            feed_rate_hz: 100_000.0,
            span_wide: false,
            families: &[],
        }
    }
    fn default_hz(&self) -> f64 {
        DEFAULT_HZ
    }
    fn stage_label(&self, hz: f64) -> String {
        format!("{:.3} SSTV", hz / 1e6)
    }
    fn chain(&self, at: Placed) -> Vec<NodeSpec> {
        vec![NodeSpec::new(DESC.name).f(CHANNEL_HZ, at.center_hz)]
    }
}

/// The carrier this stage is pointed at.
const CHANNEL_HZ: &str = "channel_hz";

pub const DESC: StageDesc = StageDesc {
    name: "sstv",
    summary: "One SSTV channel: Martin, Scottie and Robot pictures",
    category: Category::Decode,
    feeds_bus: false,
};

pub fn build(s: &Settings) -> Result<Box<dyn pipeline::node::Node>> {
    Ok(Box::new(SstvNode::new(s.f64_or(CHANNEL_HZ, DEFAULT_HZ))))
}

#[cfg(test)]
mod tests {
    use super::*;
    use common::Hz;

    #[test]
    fn the_channel_has_to_be_inside_the_span() {
        let mut n = SstvNode::new(DEFAULT_HZ);
        let far = PortSpec { spec: StreamSpec::iq(250_000.0, Hz(145_000_000)), latency: 0 };
        assert!(n.negotiate(&far).is_err());
        let near = PortSpec { spec: StreamSpec::iq(240_000.0, Hz(144_490_000)), latency: 0 };
        let out = n.negotiate(&near).expect("a channel in the span");
        assert_eq!(out.kind, PortKind::Video);
        assert_eq!(out.center, Hz(144_500_000));
    }

    /// The rate a HackRF runs at. It has no whole-number path to 44.1 kHz,
    /// which is what the first version tried for and refused the radio over.
    #[test]
    fn an_awkward_radio_rate_is_still_accepted() {
        for rate in [2_048_000.0, 2_400_000.0, 2_880_000.0, 8_000_000.0, 20_000_000.0] {
            let mut n = SstvNode::new(DEFAULT_HZ);
            let spec = PortSpec { spec: StreamSpec::iq(rate, Hz(144_500_000)), latency: 0 };
            n.negotiate(&spec).unwrap_or_else(|e| panic!("{rate} refused: {e}"));
        }
    }

    /// A chain that already has audio, which is what the shortwave modes need,
    /// feeds the same node without a demodulator in front.
    #[test]
    fn audio_is_read_as_well_as_baseband() {
        let mut n = SstvNode::new(14_230_000.0);
        let mut spec = StreamSpec::iq(48_000.0, Hz(0));
        spec.kind = PortKind::Real;
        let audio = PortSpec { spec, latency: 0 };
        let out = n.negotiate(&audio).expect("audio");
        assert_eq!(out.kind, PortKind::Video);
    }
}
