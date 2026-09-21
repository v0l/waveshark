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
use identify::Signal;
pub use identify::sstv::AUDIO_HZ;
pub use identify::sstv::CHANNEL_WIDTH_HZ;
pub use identify::sstv::DEFAULT_HZ;
pub use identify::sstv::DEVIATION_HZ;
pub use identify::sstv::Sstv;
use pipeline::node::{NodeCtx, PortSpec, Simple};
use pipeline::port::{Payload, PortKind, StreamSpec};
use pipeline::registry::{Category, Settings, SettingsExt, StageDesc};

/// What a transmission is called on the video bus, beside "analogue video".
const SYSTEM: &str = "SSTV";

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

/// A picture, as the tones that send it.
///
/// The mirror of [`SstvNode`]: `decode::sstv::encode` builds the header, the
/// VIS code and the lines, and the FM modulator puts them on a carrier. The
/// picture is built once at [`AUDIO_HZ`], where the timings are whole enough
/// samples to be exact, and read out at the radio's rate: a minute of Martin
/// 1 is ten megabytes at 44.1 kHz and half a gigabyte at 2.4 MS/s, so a
/// transmission generated at the stream's own rate could not be held.
pub struct SstvTxNode {
    path: String,
    mode: &'static sstv::Mode,
    /// The picture, row major RGB, `mode.width` by `mode.height`.
    rgb: Vec<u8>,
    /// The whole transmission at [`AUDIO_HZ`], built when the picture or the
    /// mode changes.
    audio: Vec<f32>,
    /// How far through `audio` the transmission has been sent, in samples of
    /// it.
    at: f64,
    rate: f64,
    /// Samples of silence left before the picture is sent again, and how
    /// many there are between pictures.
    rest: f64,
    pause_s: f64,
    /// Pictures sent whole.
    pictures: u64,
}

impl Default for SstvTxNode {
    fn default() -> Self {
        let mode = &sstv::MODES[0];
        let mut n = Self {
            path: String::new(),
            mode,
            rgb: Vec::new(),
            audio: Vec::new(),
            at: 0.0,
            rate: 0.0,
            rest: 0.0,
            // A picture is a minute or two of air, so five seconds between
            // them is a transmitter sending back to back.
            pause_s: 5.0,
            pictures: 0,
        };
        n.load();
        n
    }
}

impl SstvTxNode {
    pub fn new(path: &str, mode: &'static sstv::Mode) -> Self {
        let mut n = Self { path: path.into(), mode, ..Default::default() };
        // The default built the test card in its own mode, so the picture
        // and the transmission are rebuilt for the one asked for.
        n.load();
        n
    }

    /// Read the picture and build the transmission. A picture that will not
    /// load is the test card, so a transmitter keyed with a bad path sends
    /// something a receiver can show rather than silence nobody can debug.
    fn load(&mut self) {
        self.rgb = match self.path.is_empty() {
            true => test_card(self.mode),
            false => match read_picture(&self.path, self.mode) {
                Some(rgb) => rgb,
                None => {
                    tracing::warn!("sstv_tx: {}: not a picture", self.path);
                    test_card(self.mode)
                }
            },
        };
        self.audio = sstv::encode(&self.rgb, self.mode, AUDIO_HZ).unwrap_or_default();
        self.at = 0.0;
    }

    /// How long one transmission takes, in seconds.
    pub fn seconds(&self) -> f64 {
        self.audio.len() as f64 / AUDIO_HZ
    }

    pub fn sent(&self) -> u64 {
        self.pictures
    }
}

/// Eight colour bars, which is what a receiver needs to show that everything
/// between the two ends is working and that the channels are in step.
fn test_card(mode: &sstv::Mode) -> Vec<u8> {
    const BARS: [[u8; 3]; 8] = [
        [255, 255, 255],
        [255, 255, 0],
        [0, 255, 255],
        [0, 255, 0],
        [255, 0, 255],
        [255, 0, 0],
        [0, 0, 255],
        [0, 0, 0],
    ];
    let mut rgb = vec![0u8; mode.width * mode.height * 3];
    for y in 0..mode.height {
        for x in 0..mode.width {
            rgb[(y * mode.width + x) * 3..][..3].copy_from_slice(&BARS[x * 8 / mode.width]);
        }
    }
    rgb
}

/// A picture from a file, scaled to the mode by nearest neighbour. Nearest
/// rather than filtered because the tone meter reading it back is several
/// pixels wide already, so a smoother scale buys nothing a receiver sees.
fn read_picture(path: &str, mode: &sstv::Mode) -> Option<Vec<u8>> {
    let img = image::ImageReader::open(path).ok()?.with_guessed_format().ok()?.decode().ok()?;
    let img = img.to_rgb8();
    let (w, h) = (img.width() as usize, img.height() as usize);
    if w == 0 || h == 0 {
        return None;
    }
    let mut rgb = vec![0u8; mode.width * mode.height * 3];
    for y in 0..mode.height {
        let sy = (y * h / mode.height).min(h - 1);
        for x in 0..mode.width {
            let sx = (x * w / mode.width).min(w - 1);
            rgb[(y * mode.width + x) * 3..][..3]
                .copy_from_slice(&img.get_pixel(sx as u32, sy as u32).0);
        }
    }
    Some(rgb)
}

impl Simple for SstvTxNode {
    fn name(&self) -> &str {
        SSTV_TX.name
    }

    fn readings(&self) -> Vec<(String, String)> {
        let what = match self.path.is_empty() {
            true => "the test card".to_string(),
            false => std::path::Path::new(&self.path)
                .file_name()
                .map(|n| n.to_string_lossy().into_owned())
                .unwrap_or_else(|| self.path.clone()),
        };
        vec![
            ("sending".into(), format!("{what} in {}", self.mode.name)),
            ("pictures".into(), self.pictures.to_string()),
        ]
    }

    fn negotiate(&mut self, i: &PortSpec) -> Result<StreamSpec> {
        if i.spec.rate <= 0.0 {
            return Err(common::Error::other("sstv_tx needs a clock to send against"));
        }
        self.rate = i.spec.rate;
        let mut out = i.spec.with_kind(PortKind::Real);
        out.flow = pipeline::port::Flow::Tx;
        out.channels = 1;
        out.bandwidth = 2.0 * TONE_HIGH_HZ;
        Ok(out)
    }

    fn process(&mut self, i: &Payload, o: &mut Payload, _c: &mut NodeCtx<'_>) -> Result<()> {
        if self.audio.is_empty() || self.rate <= 0.0 {
            return Ok(());
        }
        let step = AUDIO_HZ / self.rate;
        let out = o.real_mut();
        for _ in 0..i.len() {
            if self.at >= self.audio.len() as f64 {
                // Between pictures the carrier carries silence rather than
                // stopping: a transmission that keyed down per picture would
                // rebuild the receiving end's idea of the channel every two
                // minutes.
                self.rest -= 1.0;
                if self.rest <= 0.0 {
                    self.at = 0.0;
                    self.pictures += 1;
                    self.rest = self.pause_s * self.rate;
                }
                out.push(0.0);
                continue;
            }
            // Linear between the samples of a 44.1 kHz picture. The tones
            // are under 2.5 kHz and the rate above is at least forty times
            // that, so what the interpolation leaves is far outside the
            // channel filter of anything reading it.
            let k = self.at as usize;
            let f = (self.at - k as f64) as f32;
            let a = self.audio[k];
            let b = *self.audio.get(k + 1).unwrap_or(&0.0);
            out.push(a + (b - a) * f);
            self.at += step;
        }
        Ok(())
    }

    fn reset(&mut self) {
        self.at = 0.0;
        self.rest = 0.0;
        self.pictures = 0;
    }

    fn params(&self) -> Vec<pipeline::param::Param> {
        use pipeline::param::Param;
        vec![
            Param::text(PICTURE, self.path.clone()).label("Picture"),
            Param::choice(
                MODE,
                sstv::MODES.iter().position(|m| m.vis == self.mode.vis).unwrap_or(0),
                sstv::MODES.iter().map(|m| m.name.to_string()).collect(),
            )
            .label("Mode"),
            Param::float(PAUSE_S, self.pause_s, 0.0..=600.0).label("Between pictures").unit("s"),
        ]
    }

    fn set_param(&mut self, name: &str, value: pipeline::param::ParamValue) -> Result<()> {
        use pipeline::param::ParamValue;
        match name {
            PICTURE => {
                let want = match value {
                    ParamValue::Text(t) => t,
                    _ => return Err(common::Error::other("sstv_tx: a picture is a path")),
                };
                if want != self.path {
                    self.path = want;
                    self.load();
                }
            }
            MODE => {
                let k = value.as_i64().unwrap_or(0).clamp(0, sstv::MODES.len() as i64 - 1);
                let want = &sstv::MODES[k as usize];
                if want.vis != self.mode.vis {
                    self.mode = want;
                    self.load();
                }
            }
            PAUSE_S => self.pause_s = value.as_f64().unwrap_or(5.0).clamp(0.0, 600.0),
            _ => return Err(common::Error::other(format!("sstv_tx: unknown parameter {name:?}"))),
        }
        Ok(())
    }
}

impl Protocol for Sstv {
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

    fn outputs(&self) -> &'static [PortKind] {
        &[PortKind::Video]
    }
    /// Wherever a picture is sent: the two metre calling frequency is the one
    /// with a channel, and the shortwave ones are worked by hand.

    fn stage_label(&self, hz: f64) -> String {
        format!("{:.3} SSTV", hz / 1e6)
    }
    fn chain(&self, at: Placed) -> Vec<NodeSpec> {
        vec![NodeSpec::new(DESC.name).f(CHANNEL_HZ, at.center_hz)]
    }
    /// A picture on a sideband channel, which is how the calling
    /// frequencies are tuned by ear.
    fn audio_stage(&self, hz: f64) -> Option<NodeSpec> {
        Some(NodeSpec::new(DESC.name).f(CHANNEL_HZ, hz))
    }
    /// The picture into the FM modulator, at the deviation the receiving
    /// discriminator is scaled for.
    fn transmit(&self) -> Option<crate::protocol::TxChain> {
        Some(crate::protocol::TxChain {
            source: NodeSpec::new(SSTV_TX.name),
            modulator: NodeSpec::new(crate::mod_nodes::FM_MOD.name).f("deviation_hz", DEVIATION_HZ),
        })
    }
}

/// The carrier this stage is pointed at.
const CHANNEL_HZ: &str = "channel_hz";

/// What the transmit side is set with.
/// The file the picture comes from, named `path` because that is what the
/// strip draws a file row for.
const PICTURE: &str = "path";
const MODE: &str = "mode";
const PAUSE_S: &str = "pause_s";

pub const DESC: StageDesc = StageDesc {
    name: "sstv",
    summary: "One SSTV channel: Martin, Scottie and Robot pictures",
    category: Category::Decode,
    feeds_bus: false,
};

pub const SSTV_TX: StageDesc = StageDesc {
    name: "sstv_tx",
    summary: "Send a picture as SSTV: Martin, Scottie and Robot modes",
    category: Category::Transmit,
    feeds_bus: false,
};

pub fn build(s: &Settings) -> Result<Box<dyn pipeline::node::Node>> {
    Ok(Box::new(SstvNode::new(s.f64_or(CHANNEL_HZ, DEFAULT_HZ))))
}

pub fn build_tx(s: &Settings) -> Result<Box<dyn pipeline::node::Node>> {
    let k = s.i64_or(MODE, 0).clamp(0, sstv::MODES.len() as i64 - 1) as usize;
    let mut n = SstvTxNode::new(s.str_or(PICTURE, ""), &sstv::MODES[k]);
    n.pause_s = s.f64_or(PAUSE_S, 5.0).clamp(0.0, 600.0);
    Ok(Box::new(n))
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

    /// Sent by the transmit stage and read back by the receive stage. Twelve
    /// seconds of a Martin 2 transmission rather than the whole minute, since
    /// what is being checked is that the header, the VIS code and the line
    /// timings agree, and every line after the first few says the same thing
    /// again at the cost of a second of test time each.
    #[test]
    fn a_picture_this_receiver_sent_is_a_picture_this_receiver_reads() {
        let rate = 44_100.0;
        let martin2 = sstv::MODES.iter().find(|m| m.name == "Martin 2").unwrap();
        let mut tx = SstvTxNode::new("", martin2);
        let mut spec = StreamSpec::iq(rate, Hz(0));
        spec.kind = PortKind::Real;
        let audio_spec = tx.negotiate(&PortSpec { spec, latency: 0 }).unwrap();
        assert_eq!(audio_spec.kind, PortKind::Real);
        // The test card in Martin 2: 0.88 s of header and VIS, then 256
        // lines of 0.2268 s.
        assert!((tx.seconds() - 58.97).abs() < 0.01, "{} s", tx.seconds());

        let mut rx = SstvNode::new(14_230_000.0);
        rx.negotiate(&PortSpec { spec: audio_spec, latency: 0 }).unwrap();
        let ins = [PortSpec { spec, latency: 0 }];
        let (mut ev, mut tg) = (Vec::new(), Vec::new());
        let mut rows = 0usize;
        let mut mode = None;
        for _ in 0..(12.0 * rate / 4096.0) as usize {
            let mut audio = Payload::Real(Vec::new());
            let mut ctx = NodeCtx::new(0, &ins, &[], &mut ev, &mut tg);
            Simple::process(&mut tx, &Payload::Real(vec![0.0; 4096]), &mut audio, &mut ctx)
                .unwrap();
            let mut video = Payload::Video(Vec::new());
            let mut ctx = NodeCtx::new(0, &ins, &[], &mut ev, &mut tg);
            Simple::process(&mut rx, &audio, &mut video, &mut ctx).unwrap();
            if let Payload::Video(frames) = video {
                for f in frames {
                    mode = f.label.clone();
                    rows += f.lines_seen;
                }
            }
        }
        assert_eq!(mode.as_deref(), Some("Martin 2"), "the VIS code named the mode");
        // Twelve seconds less the 0.91 s of header and VIS, at 0.2268 s a
        // line, is 48; the last is still being read when the audio runs
        // out, since a line is published once the one after it has started.
        assert_eq!(rows, 48, "lines off the air");
    }

    /// Every mode the receiver reads is a mode the transmitter sends, and
    /// picking one builds a transmission as long as its table says.
    #[test]
    fn each_mode_is_keyed_at_the_length_its_timings_say() {
        use pipeline::param::ParamValue;
        let mut n = SstvTxNode::default();
        let want = [115.20, 58.97, 110.53, 72.00, 269.79, 36.91, 72.91];
        for (k, mode) in sstv::MODES.iter().enumerate() {
            let at = sstv::MODES.iter().position(|m| m.vis == mode.vis).unwrap();
            n.set_param(MODE, ParamValue::Int(at as i64)).unwrap_or_else(|e| panic!("{e}"));
            assert_eq!(n.mode.name, mode.name);
            assert!(
                (n.seconds() - want[k]).abs() < 0.01,
                "{}: {} s, not {}",
                mode.name,
                n.seconds(),
                want[k]
            );
        }
    }

    /// A Robot 36 transmission keyed by the stage and read back by the
    /// receive node: the alternating colour difference, the half-width scan
    /// and the separators all agree with the decoder or the VIS code is the
    /// only thing that comes back.
    #[test]
    fn a_robot_picture_this_receiver_sent_is_a_picture_this_receiver_reads() {
        let rate = 44_100.0;
        let robot = sstv::MODES.iter().find(|m| m.name == "Robot 36").unwrap();
        let mut tx = SstvTxNode::new("", robot);
        let mut spec = StreamSpec::iq(rate, Hz(0));
        spec.kind = PortKind::Real;
        let audio_spec = tx.negotiate(&PortSpec { spec, latency: 0 }).unwrap();

        let mut rx = SstvNode::new(14_230_000.0);
        rx.negotiate(&PortSpec { spec: audio_spec, latency: 0 }).unwrap();
        let ins = [PortSpec { spec, latency: 0 }];
        let (mut ev, mut tg) = (Vec::new(), Vec::new());
        let (mut rows, mut mode) = (0usize, None);
        for _ in 0..(6.0 * rate / 4096.0) as usize {
            let mut audio = Payload::Real(Vec::new());
            let mut ctx = NodeCtx::new(0, &ins, &[], &mut ev, &mut tg);
            Simple::process(&mut tx, &Payload::Real(vec![0.0; 4096]), &mut audio, &mut ctx)
                .unwrap();
            let mut video = Payload::Video(Vec::new());
            let mut ctx = NodeCtx::new(0, &ins, &[], &mut ev, &mut tg);
            Simple::process(&mut rx, &audio, &mut video, &mut ctx).unwrap();
            if let Payload::Video(frames) = video {
                for f in frames {
                    mode = f.label.clone();
                    rows += f.lines_seen;
                }
            }
        }
        assert_eq!(mode.as_deref(), Some("Robot 36"), "the VIS code named the mode");
        // Six seconds less the 0.91 s of header and VIS, at 0.15 s a line,
        // is 33; the last is still being read when the audio runs out, and
        // Robot 36 publishes a line behind the scan because it borrows the
        // colour difference from the line after.
        assert_eq!(rows, 32, "lines off the air");
    }
}
