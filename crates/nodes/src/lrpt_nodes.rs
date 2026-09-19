//! Meteor-M LRPT as a graph node.
//!
//! Wiring: the downlink at 137 MHz is 72 kilosymbols of QPSK about 120 kHz
//! wide, so the node mixes the channel down, resamples it to four samples a
//! symbol, hands it to `dsp::qpsk`, and pours the soft symbols through
//! `decode::ccsds` into `decode::lrpt`. What reaches the video bus is eight
//! rows of one instrument channel as each strip is read, so a fifteen minute
//! pass fills in as it is heard rather than arriving at the end of it, the
//! same way APT does.
//!
//! One picture per instrument channel, because that is what the satellite
//! sends: three of MSU-MR's six channels, each its own greyscale scan of the
//! same ground. They are named apart on the bus so that the pane can be
//! switched between them; a false colour composite of the three would be a
//! judgement about which channel is which colour, and the receiver has no
//! way to know that.
//!
//! Which way up the picture is depends on which way the satellite was going,
//! and nothing here turns it over.

use crate::NodeSpec;
use crate::protocol::{Placed, Placement, Protocol, Shape};
use common::{Cadence, Pixels, Result, Update, VideoFrame};
use decode::{ccsds, lrpt};
use dsp::qpsk::{QpskConfig, QpskDemod};
use dsp::resample::Rational;
use dsp::{FirDecim, Mixer};
use identify::Signal;
pub use identify::lrpt::CHANNEL_WIDTH_HZ;
pub use identify::lrpt::DEFAULT_HZ;
pub use identify::lrpt::Lrpt;
pub use identify::lrpt::SATELLITES;
use pipeline::node::{NodeCtx, PortSpec, Simple};
use pipeline::port::{Payload, PortKind, StreamSpec};
use pipeline::registry::{Category, Settings, SettingsExt, StageDesc};

/// What a transmission is called on the video bus, one per instrument
/// channel: the bus tells pictures apart by what produced them and where
/// they were received, and a pass carries three channels on one frequency.
const SYSTEMS: [&str; 6] = ["LRPT 1", "LRPT 2", "LRPT 3", "LRPT 4", "LRPT 5", "LRPT 6"];

/// Rows a picture is tall before another starts.
///
/// A strip is eight rows and LRPT carries about 3.9 kB/s of packets, which
/// is a strip every seven seconds or so, so a fifteen minute pass is around
/// 1000 rows: one canvas holds a pass with room over, and a satellite that
/// stays up longer than that starts a second picture rather than painting
/// over the first.
const CANVAS_ROWS: usize = 1_536;

pub struct LrptNode {
    channel_hz: f64,
    cfg: QpskConfig,
    mixer: Mixer,
    decim: FirDecim,
    resample: Option<Rational>,
    demod: QpskDemod,
    deframer: ccsds::Deframer,
    packets: ccsds::Packets,
    rx: lrpt::Receiver,
    mixed: Vec<common::C32>,
    narrow: Vec<common::C32>,
    at_rate: Vec<common::C32>,
    symbols: Vec<common::C32>,
    soft: Vec<f32>,
    frames: Vec<ccsds::Frame>,
    packet_buf: Vec<ccsds::SpacePacket>,
    strips: u64,
}

impl Default for LrptNode {
    fn default() -> Self {
        Self::new(DEFAULT_HZ, QpskConfig::LRPT_OFFSET)
    }
}

impl LrptNode {
    pub fn new(channel_hz: f64, cfg: QpskConfig) -> Self {
        Self {
            channel_hz,
            cfg,
            mixer: Mixer::new(0.0, 1.0),
            decim: FirDecim::design_hz(cfg.rate(), 1, CHANNEL_WIDTH_HZ / 2.0, 60.0),
            resample: None,
            demod: QpskDemod::new(cfg),
            deframer: ccsds::Deframer::new(),
            packets: ccsds::Packets::new(),
            rx: lrpt::Receiver::new(),
            mixed: Vec::new(),
            narrow: Vec::new(),
            at_rate: Vec::new(),
            symbols: Vec::new(),
            soft: Vec::new(),
            frames: Vec::new(),
            packet_buf: Vec::new(),
            strips: 0,
        }
    }

    /// Strips of picture published since the node was built.
    pub fn strips(&self) -> u64 {
        self.strips
    }

    /// Frames the deframer read, and the ones it could not correct.
    pub fn frames(&self) -> (u64, u64) {
        (self.deframer.found(), self.deframer.failed())
    }

    /// Whether the demodulator is tracking a carrier.
    pub fn locked(&self) -> bool {
        self.demod.locked()
    }

    /// Which satellite this channel is, where it is one of them.
    fn satellite(&self) -> Option<&'static str> {
        SATELLITES
            .iter()
            .find(|(_, hz)| (hz - self.channel_hz).abs() < CHANNEL_WIDTH_HZ / 2.0)
            .map(|(name, _)| *name)
    }

    fn publish(&mut self, strip: lrpt::Strip, out: &mut Vec<VideoFrame>) {
        self.strips += 1;
        let picture = strip.first_row / CANVAS_ROWS;
        let first = strip.first_row % CANVAS_ROWS;
        let channel = usize::from(strip.channel).clamp(1, SYSTEMS.len()) - 1;
        out.push(VideoFrame {
            system: SYSTEMS[channel],
            channel_hz: self.channel_hz,
            label: self.satellite().map(str::to_string),
            width: lrpt::WIDTH,
            height: CANVAS_ROWS,
            // Square pixels: the instrument's ground sample is about a
            // kilometre across and a kilometre along the track.
            aspect: lrpt::WIDTH as f32 / CANVAS_ROWS as f32,
            pixels: Pixels::Luma8,
            samples: std::sync::Arc::new(strip.gray),
            lines_seen: lrpt::STRIP_ROWS,
            sequence: picture as u64,
            update: Update::Rows { first },
            cadence: Cadence::Still,
        });
    }

    /// Soft symbols in, pictures out: the frames, the packets and the
    /// strips they paint.
    fn read(&mut self, out: &mut Vec<VideoFrame>) {
        self.soft.clear();
        for s in &self.symbols {
            self.soft.push(s.re);
            self.soft.push(s.im);
        }
        self.frames.clear();
        let mut frames = std::mem::take(&mut self.frames);
        self.deframer.push(&self.soft, &mut frames);
        for frame in frames.drain(..) {
            let Some(vcdu) = ccsds::Vcdu::parse(&frame.vcdu) else { continue };
            self.packet_buf.clear();
            let mut packets = std::mem::take(&mut self.packet_buf);
            self.packets.push(&vcdu, &mut packets);
            for packet in packets.drain(..) {
                if let Some(strip) = self.rx.push(&packet) {
                    self.publish(strip, out);
                }
            }
            self.packet_buf = packets;
        }
        self.frames = frames;
    }
}

impl Simple for LrptNode {
    fn name(&self) -> &str {
        "lrpt"
    }

    fn negotiate(&mut self, i: &PortSpec) -> Result<StreamSpec> {
        let want = self.cfg.rate();
        match i.spec.kind {
            PortKind::Iq => {
                let (rate, center) = (i.spec.rate, i.spec.center.as_f64());
                if (self.channel_hz - center).abs() > rate / 2.0 - CHANNEL_WIDTH_HZ / 2.0 {
                    return Err(common::Error::other("lrpt needs its channel inside the span"));
                }
                let (factor, resample) = dsp::resample::stage(rate, want, 4096)
                    .ok_or_else(|| common::Error::other("lrpt cannot reach 288 kHz from here"))?;
                self.mixer = Mixer::new(center - self.channel_hz, rate);
                self.decim = FirDecim::design_hz(rate, factor, CHANNEL_WIDTH_HZ / 2.0, 60.0);
                self.resample = resample;
            }
            _ => return Err(common::Error::other("lrpt reads baseband")),
        }
        self.demod = QpskDemod::new(self.cfg);
        self.deframer.reset();
        self.packets.reset();
        self.rx.reset();

        let mut out = i.spec.with_kind(PortKind::Video);
        out.center = common::Hz(self.channel_hz as u64);
        Ok(out)
    }

    fn process(&mut self, i: &Payload, o: &mut Payload, _c: &mut NodeCtx<'_>) -> Result<()> {
        let Payload::Iq(iq) = i else { return Ok(()) };
        self.mixed.clear();
        self.mixer.process(iq, &mut self.mixed);
        self.narrow.clear();
        self.decim.process(&self.mixed, &mut self.narrow);
        let at_rate = match self.resample.as_mut() {
            Some(r) => {
                self.at_rate.clear();
                r.process(&self.narrow, &mut self.at_rate);
                &self.at_rate
            }
            None => &self.narrow,
        };
        self.symbols.clear();
        let mut symbols = std::mem::take(&mut self.symbols);
        self.demod.process(at_rate, &mut symbols);
        self.symbols = symbols;
        if self.symbols.is_empty() {
            return Ok(());
        }
        let mut pictures = Vec::new();
        self.read(&mut pictures);
        if !pictures.is_empty() {
            o.video_mut().extend(pictures);
        }
        Ok(())
    }

    fn reset(&mut self) {
        self.mixer.reset();
        self.decim.reset();
        self.demod.reset();
        self.deframer.reset();
        self.packets.reset();
        self.rx.reset();
    }
}

impl Protocol for Lrpt {
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
    /// The two channels the satellites are on and nowhere else.

    fn stage_label(&self, hz: f64) -> String {
        match SATELLITES.iter().find(|(_, c)| (c - hz).abs() < CHANNEL_WIDTH_HZ / 2.0) {
            Some((name, _)) => format!("{name} LRPT"),
            None => format!("{:.3} LRPT", hz / 1e6),
        }
    }
    fn chain(&self, at: Placed) -> Vec<NodeSpec> {
        vec![NodeSpec::new(DESC.name).f(CHANNEL_HZ, at.center_hz)]
    }
}

/// The carrier this stage is pointed at.
const CHANNEL_HZ: &str = "channel_hz";

/// Whether the downlink is keyed offset, which both operating satellites
/// are, or plain.
const OFFSET: &str = "offset";

pub const DESC: StageDesc = StageDesc {
    name: "lrpt",
    summary: "One Meteor-M LRPT downlink: the pass as three pictures, a strip at a time",
    category: Category::Decode,
    feeds_bus: false,
};

pub fn build(s: &Settings) -> Result<Box<dyn pipeline::node::Node>> {
    let cfg = match s.bool_or(OFFSET, true) {
        true => QpskConfig::LRPT_OFFSET,
        false => QpskConfig::LRPT,
    };
    Ok(Box::new(LrptNode::new(s.f64_or(CHANNEL_HZ, DEFAULT_HZ), cfg)))
}

#[cfg(test)]
mod tests {
    use super::*;
    use common::Hz;
    use decode::jpeg;
    use dsp::qpsk::{self, Coding, Keying};
    use std::f64::consts::TAU;

    fn spec(rate: f64, center: u64) -> PortSpec {
        PortSpec { spec: StreamSpec::iq(rate, Hz(center)), latency: 0 }
    }

    /// A run of bits, most significant first, as a transmitter writes them.
    #[derive(Default)]
    struct Writer {
        bytes: Vec<u8>,
        at: usize,
    }

    impl Writer {
        fn push(&mut self, code: u16, length: u8) {
            for k in (0..length).rev() {
                if self.at.is_multiple_of(8) {
                    self.bytes.push(0);
                }
                let last = self.bytes.len() - 1;
                self.bytes[last] |= (((code >> k) & 1) as u8) << (7 - self.at % 8);
                self.at += 1;
            }
        }
    }

    /// One image packet of fourteen flat blocks, as the satellite sends it.
    fn mcu_packet(apid: u16, sequence: u16, mcu: usize, dc: i32) -> Vec<u8> {
        let (dctab, actab) = (jpeg::Table::luma_dc(), jpeg::Table::luma_ac());
        let mut w = Writer::default();
        for k in 0..lrpt::MCUS_PER_PACKET {
            let step = match k {
                0 => dc,
                _ => 0,
            };
            let category = match step {
                0 => 0u8,
                n => (32 - n.unsigned_abs().leading_zeros()) as u8,
            };
            let (length, code) = dctab.code_for(category).expect("a DC code");
            w.push(code, length);
            if category > 0 {
                w.push(step as u16, category);
            }
            let (length, code) = actab.code_for(0x00).expect("an end of block");
            w.push(code, length);
        }
        let mut payload = vec![0u8; 14];
        payload[8] = mcu as u8;
        payload[13] = 60;
        payload.extend(w.bytes);

        let mut packet = Vec::new();
        packet.extend(((1u16 << 11) | apid).to_be_bytes());
        packet.extend((0xc000u16 | sequence).to_be_bytes());
        packet.extend(((payload.len() - 1) as u16).to_be_bytes());
        packet.extend(payload);
        packet
    }

    /// A pass on the air: strips of three channels, in packets, in frames,
    /// coded and keyed the way Meteor-M2-4 keys them.
    fn downlink(strips: usize, rate: f64, center: f64, cfg: QpskConfig) -> Vec<common::C32> {
        // Every packet of the pass, in the order the satellite sends them:
        // one channel's whole scan, then the next.
        let mut stream = Vec::new();
        let mut sequence = 0u16;
        for _ in 0..strips {
            for (channel, apid) in [64u16, 65, 66].iter().enumerate() {
                for mcu in (0..lrpt::MCU_COLUMNS).step_by(lrpt::MCUS_PER_PACKET) {
                    let dc = 20 + 10 * channel as i32;
                    stream.extend(mcu_packet(*apid, sequence, mcu, dc));
                    sequence = sequence.wrapping_add(1);
                }
            }
            // The housekeeping packet that goes with every strip, which is
            // what makes the cadence 43 packets rather than 42.
            let mut telemetry = Vec::new();
            telemetry.extend(((1u16 << 11) | lrpt::TELEMETRY_APID).to_be_bytes());
            telemetry.extend((0xc000u16 | sequence).to_be_bytes());
            telemetry.extend(31u16.to_be_bytes());
            telemetry.extend(vec![0u8; 32]);
            stream.extend(telemetry);
            sequence = sequence.wrapping_add(1);
        }

        // Cut it into the packet zones of as many frames as it takes.
        let zone = ccsds::VCDU_BYTES - 10;
        let mut soft = Vec::new();
        let mut at = 0usize;
        let mut counter = 0u32;
        // Where the next packet header falls in the zone, which is what a
        // frame has to say for the assembler to find it.
        let mut boundary = 0usize;
        while at < stream.len() {
            let take = zone.min(stream.len() - at);
            let mut vcdu = vec![0u8; ccsds::VCDU_BYTES];
            let id = (1u16 << 14) | (57 << 6) | 5;
            vcdu[..2].copy_from_slice(&id.to_be_bytes());
            vcdu[2..5].copy_from_slice(&counter.to_be_bytes()[1..]);
            let pointer = match boundary < take {
                true => boundary as u16,
                // No packet starts in this frame at all.
                false => 2047,
            };
            vcdu[8..10].copy_from_slice(&pointer.to_be_bytes());
            vcdu[10..10 + take].copy_from_slice(&stream[at..at + take]);
            soft.extend(ccsds::frame_soft_bits(&vcdu));
            // Where the next header is, once this zone has been filled: the
            // packets are laid end to end, so it is wherever one straddles
            // the frame boundary.
            boundary = match boundary < take {
                true => {
                    let mut next = boundary;
                    while next < take {
                        let length = usize::from(u16::from_be_bytes([
                            stream[at + next + 4],
                            stream[at + next + 5],
                        ])) + 7;
                        next += length;
                    }
                    next - take
                }
                false => boundary - take,
            };
            at += take;
            counter += 1;
        }

        // And key it: the soft bits are the bits, two to a symbol.
        let bits: Vec<u8> = soft
            .iter()
            .map(|&s: &f32| match s > 0.0 {
                true => 0,
                false => 1,
            })
            .collect();
        let baseband = qpsk::modulate(&bits, cfg);
        // Up to the radio's rate, and off the middle of the span.
        let up = (rate / cfg.rate()).round() as usize;
        let offset = DEFAULT_HZ - center;
        let mut out = Vec::with_capacity(baseband.len() * up);
        let mut phase = 0.0f64;
        let mut r = Rational::with_ratio(up, 1);
        let mut resampled = Vec::new();
        r.process(&baseband, &mut resampled);
        for s in resampled {
            phase += TAU * offset / rate;
            out.push(s * common::C32::new(phase.cos() as f32, phase.sin() as f32));
        }
        out
    }

    /// The whole node against a synthesised pass: mixer, resampler,
    /// demodulator, deframer, Reed-Solomon, packets and blocks, with three
    /// pictures reaching the video bus a strip at a time.
    #[test]
    fn a_synthesised_pass_reaches_the_video_bus() {
        let (rate, center) = (576_000.0, 137_150_000.0);
        let cfg = QpskConfig::LRPT_OFFSET;
        let iq = downlink(8, rate, center, cfg);
        let mut node = LrptNode::new(DEFAULT_HZ, cfg);
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

        assert!(node.locked(), "the demodulator never locked");
        let (found, failed) = node.frames();
        // Ten frames of the pass, none of which needed correcting, and the
        // tail of the stream is short of the two frames the sync search
        // needs in hand.
        assert_eq!((found, failed), (10, 0), "{found} frames read, {failed} failed");
        assert_eq!(node.strips(), 16);
        // Three strips of each of three channels: the fourth is still being
        // painted when the pass ends.
        // Five or six strips of each of three channels: the first strip of
        // the pass is lost to the sync search and the last is still being
        // painted when it ends.
        assert_eq!(frames.len(), 16, "strips on the bus");
        let mut systems: Vec<&str> = frames.iter().map(|f| f.system).collect();
        systems.sort_unstable();
        systems.dedup();
        assert_eq!(systems, vec!["LRPT 1", "LRPT 2", "LRPT 3"]);
        for f in &frames {
            assert_eq!(f.width, lrpt::WIDTH);
            assert_eq!(f.height, CANVAS_ROWS);
            assert_eq!(f.pixels, Pixels::Luma8);
            assert_eq!(f.label.as_deref(), Some("Meteor-M2-4"));
            assert_eq!(f.channel_hz, DEFAULT_HZ);
            assert_eq!(f.lines_seen, lrpt::STRIP_ROWS);
            assert_eq!(f.samples.len(), lrpt::STRIP_ROWS * lrpt::WIDTH);
        }
        // The rows landed where the strips belong, eight apart.
        let rows: Vec<usize> = frames
            .iter()
            .filter(|f| f.system == "LRPT 1")
            .map(|f| match f.update {
                Update::Rows { first } => first,
                Update::Whole => usize::MAX,
            })
            .collect();
        assert_eq!(rows, vec![0, 8, 16, 24, 32]);
        // And each channel came out at the level it was sent at, so nothing
        // was painted into the wrong picture.
        for (system, dc) in [("LRPT 1", 20i32), ("LRPT 2", 30), ("LRPT 3", 40)] {
            let f = frames.iter().find(|f| f.system == system).expect("a strip");
            let step = jpeg::quant_table(60)[0];
            let want = ((dc * step) as f64 / 8.0 + 128.0).round() as u8;
            assert!(f.samples.iter().all(|&p| p.abs_diff(want) <= 1), "{system} is not flat");
            assert_eq!(f.samples[0], want, "{system} came out at the wrong level");
        }
    }

    /// Noise on the channel: no lock, no frames, no pictures.
    #[test]
    fn noise_produces_no_pictures() {
        let (rate, center) = (576_000.0, 137_100_000.0);
        let mut x = 0x1357_9bdfu32;
        let noise: Vec<common::C32> = (0..2_000_000)
            .map(|_| {
                let mut r = || {
                    x ^= x << 13;
                    x ^= x >> 17;
                    x ^= x << 5;
                    (x as i32 as f32) / i32::MAX as f32
                };
                common::C32::new(r(), r())
            })
            .collect();
        let mut node = LrptNode::new(DEFAULT_HZ, QpskConfig::LRPT_OFFSET);
        node.negotiate(&spec(rate, center as u64)).expect("a channel in the span");
        let ins = [spec(rate, center as u64)];
        let tags = Vec::new();
        let mut frames: Vec<VideoFrame> = Vec::new();
        for block in noise.chunks(65_536) {
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
        // Three and a half seconds of noise at 576 kS/s.
        assert_eq!(frames.len(), 0, "{} pictures out of noise", frames.len());
        assert_eq!(node.frames(), (0, 0));
        assert_eq!(node.strips(), 0);
        assert!(!node.locked());
    }

    #[test]
    fn the_channel_has_to_be_inside_the_span() {
        let mut n = LrptNode::new(DEFAULT_HZ, QpskConfig::LRPT_OFFSET);
        let far = spec(576_000.0, 137_600_000);
        assert!(n.negotiate(&far).is_err());
        let near = spec(576_000.0, 137_250_000);
        let out = n.negotiate(&near).expect("a channel in the span");
        assert_eq!(out.kind, PortKind::Video);
        assert_eq!(out.center, Hz(137_100_000));
    }

    /// The rates a radio on 137 MHz runs at, none of which divides into
    /// 288 kHz.
    #[test]
    fn an_awkward_radio_rate_is_still_accepted() {
        for rate in [300_000.0, 1_024_000.0, 2_048_000.0, 2_400_000.0, 8_000_000.0] {
            let mut n = LrptNode::new(DEFAULT_HZ, QpskConfig::LRPT_OFFSET);
            n.negotiate(&spec(rate, 137_100_000)).unwrap_or_else(|e| panic!("{rate} refused: {e}"));
        }
    }

    /// Each channel is named for its satellite, and one that is neither is
    /// named for its frequency.
    #[test]
    fn a_channel_is_named_for_its_satellite() {
        assert_eq!(Lrpt.stage_label(137_100_000.0), "Meteor-M2-4 LRPT");
        assert_eq!(Lrpt.stage_label(137_900_000.0), "Meteor-M2-3 LRPT");
        assert_eq!(Lrpt.stage_label(137_500_000.0), "137.500 LRPT");
        assert_eq!(
            LrptNode::new(137_900_000.0, QpskConfig::LRPT_OFFSET).satellite(),
            Some("Meteor-M2-3")
        );
        assert_eq!(LrptNode::new(137_500_000.0, QpskConfig::LRPT_OFFSET).satellite(), None);
    }

    /// The stage is built either way up the keying can be, since the first
    /// Meteor keyed plain QPSK where the two operating satellites key
    /// offset.
    #[test]
    fn the_keying_is_a_setting() {
        let mut s = Settings::new();
        s.insert(CHANNEL_HZ.into(), pipeline::param::ParamValue::Float(137_900_000.0));
        assert!(build(&s).is_ok());
        s.insert(OFFSET.into(), pipeline::param::ParamValue::Bool(false));
        assert!(build(&s).is_ok());
        assert_eq!(QpskConfig::LRPT.keying, Keying::Coherent);
        assert_eq!(QpskConfig::LRPT.coding, Coding::Direct);
        assert_eq!(QpskConfig::LRPT_OFFSET.keying, Keying::Offset);
        assert_eq!(QpskConfig::LRPT_OFFSET.coding, Coding::Differential);
    }
}
