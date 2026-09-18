//! Iridium's L-band downlink as a graph node: one channel in, ring alerts
//! and broadcasts out, and a satellite on the map.
//!
//! Wiring only. The waveform is [`dsp::dqpsk`], the frames are
//! [`decode::iridium`], and neither knows about the other or about the
//! graph.
//!
//! # A channel wider than a channel
//!
//! A satellite 780 km up moves the carrier by [`iridium::DOPPLER_HZ`] either
//! side, which is most of two channels: a filter cut to the 41.7 kHz channel
//! would throw away the pass that made the frame worth having. So the node
//! filters the channel plus the doppler and lets the demodulator find the
//! carrier, and a frame is reported at the channel this stage was pointed
//! at rather than at the frequency it arrived on. The two disagree by up to
//! a channel, and nothing on the air says which of them is right: the
//! offset the demodulator took out is a field on the row.

use crate::NodeSpec;
use crate::protocol::{FrameClaim, Mark, Placed, Placement, Protocol, Shape, Stickiness};
use common::Result;
use decode::iridium::{self, RING_ALERT_HZ};
use dsp::dqpsk::{DqpskBurst, DqpskConfig, DqpskDemod};
use dsp::{FirDecim, Mixer};
use pipeline::event::{Decoded, media};
use pipeline::node::{NodeCtx, PortSpec, Simple};
use pipeline::port::{Payload, PortKind, StreamSpec};
use pipeline::registry::{Category, Settings, SettingsExt, StageDesc};

/// The rate the demodulator runs at: ten samples a symbol at 25 kbaud, which
/// is enough to put the symbol clock inside a tenth of a symbol without
/// interpolating.
const WORK_HZ: f64 = 250_000.0;

/// The rate to ask the receiver for. Two work rates, so the channel filter
/// has somewhere to roll off.
const FEED_HZ: f64 = 500_000.0;

/// Half the width the channel filter passes: the channel itself and the
/// doppler either side of it.
const HALF_PASS_HZ: f64 = iridium::CHANNEL_WIDTH_HZ / 2.0 + iridium::DOPPLER_HZ;

/// What the front end writes in front of a frame's bits, so the packet bus
/// can tell one from anything else arriving on an L-band centre.
pub const TAG: [u8; 3] = *b"IRD";

/// The carrier this stage is pointed at.
const CHANNEL_HZ: &str = "channel_hz";

/// The longest frame: the access word, the ring alert header and twelve
/// pages, each page a 64 bit group.
const MAX_FRAME_BITS: usize = 24 + 96 + 13 * 64;

pub struct IridiumNode {
    channel_hz: f64,
    rate: f64,
    mixer: Mixer,
    decim: FirDecim,
    demod: DqpskDemod,
    meter: crate::FrameMeter,
    mixed: Vec<common::C32>,
    narrow: Vec<common::C32>,
    bursts: Vec<DqpskBurst>,
    frames: u64,
}

impl Default for IridiumNode {
    fn default() -> Self {
        Self::new(RING_ALERT_HZ)
    }
}

impl IridiumNode {
    pub fn new(channel_hz: f64) -> Self {
        Self {
            channel_hz,
            rate: WORK_HZ,
            // All replaced at negotiation, when the span's rate is known.
            mixer: Mixer::new(0.0, WORK_HZ),
            decim: FirDecim::design_hz(WORK_HZ, 1, HALF_PASS_HZ, 60.0),
            demod: DqpskDemod::new(WORK_HZ, DqpskConfig::IRIDIUM),
            // Half a second at the work rate: a ring alert burst is 20 ms,
            // so a frame's own samples are there when the row is built.
            meter: crate::FrameMeter::new(WORK_HZ, channel_hz as u64, 0.5),
            mixed: Vec::new(),
            narrow: Vec::new(),
            bursts: Vec::new(),
            frames: 0,
        }
    }

    /// Frames whose blocks all checked since the node was built.
    pub fn frames(&self) -> u64 {
        self.frames
    }
}

/// A burst's bits as a frame for the bus: the tag, how many bits there are,
/// and the bits from the access word on.
fn pack(bits: &[bool]) -> Option<Vec<u8>> {
    let at = iridium::find_access(bits, &iridium::DOWNLINK_ACCESS)?;
    let from = at - iridium::DOWNLINK_ACCESS.len();
    let bits = &bits[from..bits.len().min(from + MAX_FRAME_BITS)];
    let mut out = TAG.to_vec();
    out.extend((bits.len() as u16).to_be_bytes());
    out.extend(bits.chunks(8).map(|byte| {
        byte.iter().enumerate().fold(0u8, |acc, (i, b)| acc | u8::from(*b) << (7 - i))
    }));
    Some(out)
}

/// The bits back out of a frame the front end wrote.
pub fn unpack(bytes: &[u8]) -> Option<Vec<bool>> {
    if bytes.len() < 5 || bytes[..3] != TAG {
        return None;
    }
    let count = usize::from(u16::from_be_bytes([bytes[3], bytes[4]]));
    let body = &bytes[5..];
    (count <= body.len() * 8 && count <= MAX_FRAME_BITS)
        .then(|| (0..count).map(|i| body[i / 8] >> (7 - i % 8) & 1 == 1).collect())
}

impl Simple for IridiumNode {
    fn name(&self) -> &str {
        "iridium"
    }

    fn negotiate(&mut self, i: &PortSpec) -> Result<StreamSpec> {
        if i.spec.kind != PortKind::Iq {
            return Err(common::Error::other("iridium reads complex baseband"));
        }
        let (rate, center) = (i.spec.rate, i.spec.center.as_f64());
        if iridium::channel_at(self.channel_hz).is_none() {
            return Err(common::Error::other("iridium reads the 1616 to 1626.5 MHz downlink"));
        }
        if (self.channel_hz - center).abs() > rate / 2.0 - HALF_PASS_HZ {
            return Err(common::Error::other(
                "iridium needs its channel and the doppler in the span",
            ));
        }
        let factor = (rate / WORK_HZ).round().max(1.0) as usize;
        let work = rate / factor as f64;
        if work < 2.0 * HALF_PASS_HZ {
            return Err(common::Error::other("iridium needs at least 150 kS/s"));
        }
        self.rate = work;
        self.mixer = Mixer::new(center - self.channel_hz, rate);
        self.decim = FirDecim::design_hz(rate, factor, HALF_PASS_HZ, 60.0);
        self.demod = DqpskDemod::new(work, DqpskConfig::IRIDIUM);
        self.meter = crate::FrameMeter::new(work, self.channel_hz as u64, 0.5);

        let mut out = i.spec.with_kind(PortKind::Frames);
        out.center = common::Hz(self.channel_hz as u64);
        out.bandwidth = iridium::CHANNEL_WIDTH_HZ;
        Ok(out)
    }

    fn process(&mut self, i: &Payload, o: &mut Payload, _c: &mut NodeCtx<'_>) -> Result<()> {
        let Some(iq) = i.as_iq() else { return Ok(()) };
        self.mixed.clear();
        self.mixer.process(iq, &mut self.mixed);
        self.narrow.clear();
        self.decim.process(&self.mixed, &mut self.narrow);
        self.meter.feed(&self.narrow);

        self.bursts.clear();
        let mut bursts = std::mem::take(&mut self.bursts);
        self.demod.process(&self.narrow, &mut bursts);
        let out = o.frames_mut();
        for b in &bursts {
            // Read here as well as on the bus, so a burst whose blocks do
            // not check never becomes a row: an access word comes up in
            // noise eventually, and its blocks do not.
            let Some(bytes) = pack(&b.bits) else { continue };
            let Some(bits) = unpack(&bytes) else { continue };
            if iridium::parse(&bits).is_none() {
                continue;
            }
            self.frames += 1;
            let mut frame =
                common::Frame::measured(bytes, b.rssi_dbfs, b.snr_db).at(self.channel_hz as u64);
            let length = (b.bits.len() / 2) as f64 * self.demod.sps();
            frame.iq = self.meter.iq_at(b.start_sample, length as usize);
            out.push(frame);
        }
        self.bursts = bursts;
        Ok(())
    }

    fn reset(&mut self) {
        self.mixer.reset();
        self.decim.reset();
        self.demod.reset();
        self.meter.reset();
    }
}

/// The row a frame becomes.
///
/// Nobody wrote any of it: a ring alert is one machine paging another, so
/// the row carries its fields, its position and no claim that it is a
/// message.
pub fn iridium_decoded(bytes: &[u8], center: common::Hz) -> Option<Decoded> {
    let bits = unpack(bytes)?;
    let f = iridium::parse(&bits)?;
    let mut fields = f.fields();
    if let Some(ch) = iridium::channel_at(center.as_f64()) {
        fields.insert(1, ("channel".into(), common::Value::Text(ch.label())));
    }
    let detail = fields.iter().map(|(k, v)| format!("{k}={v}")).collect::<Vec<_>>().join(" ");
    let mut who = common::Identity::new("iridium", format!("SV{:03}", f.sat()));
    who.name = Some(format!("Iridium {}", f.sat()));
    who.vendor = Some("Iridium".into());
    let mut d = Decoded::bytes("Iridium", center, 0.0, bytes.to_vec())
        .by(who)
        .with_detail(detail)
        .with_fields(fields)
        .with_media(media::BYTES)
        .with_modulation(common::Modulation::Dqpsk)
        // Every block carried a BCH(31,21) and a parity bit, and a frame
        // reaching here had all of them agree.
        .with_crc(Some(true));
    if let Some((lat, lon, altitude_km)) = f.position() {
        d = d.at_position(common::Position {
            lat,
            lon,
            altitude_m: Some(altitude_km * 1000.0),
            ..Default::default()
        });
    }
    Some(d)
}

/// Iridium as the auto node and the tables know it.
pub struct Iridium;

impl Protocol for Iridium {
    fn id(&self) -> &'static str {
        "iridium"
    }
    fn label(&self) -> &'static str {
        "iridium"
    }
    fn aliases(&self) -> &'static [&'static str] {
        &["ira", "ring alert", "iridium-ra"]
    }
    /// The downlink band, duplex channels and the simplex ones above them:
    /// ring alerts are on the simplex channels and broadcasts on the duplex,
    /// and a scanner block may name either.
    fn placement(&self) -> Placement {
        Placement::Bands(vec![(iridium::BASE_HZ, iridium::SIMPLEX_BAND_HZ.1)])
    }
    /// The ring alert channel, which is the one frequency worth parking on:
    /// every satellite overhead transmits there.
    fn default_hz(&self) -> f64 {
        RING_ALERT_HZ
    }
    fn shape(&self) -> Shape {
        Shape {
            widths: &[iridium::CHANNEL_WIDTH_HZ],
            // The channel and the doppler either side of it, which is what
            // the demodulator searches over.
            min_rate_hz: 150_000.0,
            feed_rate_hz: FEED_HZ,
            span_wide: false,
            families: &[],
        }
    }
    /// The front end's own tag, which is the only thing that tells these
    /// bits from any other burst read on an L-band centre: Inmarsat's
    /// channels are in the same band and its frames are bytes too.
    fn frame_claim(&self) -> FrameClaim {
        FrameClaim::Tagged
    }
    fn read_frame(&self, p: &common::Packet, bytes: &[u8]) -> Option<Vec<Decoded>> {
        iridium_decoded(bytes, common::Hz(p.center_hz())).map(|d| vec![d])
    }
    /// A ring alert says where the satellite that sent it is, which is a
    /// track the map can draw.
    fn reports_position(&self) -> bool {
        true
    }
    /// A channel that carried a ring alert carries the next one 90 ms later,
    /// and the constellation keeps passing overhead.
    fn stickiness(&self) -> Stickiness {
        Stickiness::SESSION
    }
    fn stage_label(&self, hz: f64) -> String {
        match iridium::channel_at(hz) {
            Some(ch) => format!("{} IRIDIUM", ch.label()),
            None => format!("{:.4} IRIDIUM", hz / 1e6),
        }
    }
    fn marks(&self, hz: f64) -> Vec<Mark> {
        let label = match iridium::channel_at(hz) {
            Some(ch) => format!("IRIDIUM {}", ch.label()),
            None => "IRIDIUM".into(),
        };
        vec![Mark { hz, width_hz: iridium::CHANNEL_WIDTH_HZ, label }]
    }
    fn chain(&self, at: Placed) -> Vec<NodeSpec> {
        vec![NodeSpec::new(DESC.name).f(CHANNEL_HZ, at.center_hz)]
    }
}

pub const DESC: StageDesc = StageDesc {
    name: "iridium",
    summary: "One Iridium downlink channel: 25 kbaud DQPSK bursts, ring alerts and broadcasts",
    category: Category::Decode,
    feeds_bus: true,
};

pub fn build(s: &Settings) -> Result<Box<dyn pipeline::node::Node>> {
    Ok(Box::new(IridiumNode::new(s.f64_or(CHANNEL_HZ, RING_ALERT_HZ))))
}

#[cfg(test)]
mod tests {
    use super::*;
    use common::{C32, Hz};
    use decode::iridium::{Page, RingAlert};

    fn spec(rate: f64, center: f64) -> PortSpec {
        PortSpec { spec: StreamSpec::iq(rate, Hz(center as u64)), latency: 0 }
    }

    fn a_ring_alert() -> RingAlert {
        RingAlert {
            sat: 108,
            beam: 31,
            ecef_km: (3_236.0, -292.0, 6_364.0),
            lat: 0.0,
            lon: 0.0,
            altitude_km: 0.0,
            interval: 3,
            slot: 1,
            broadcast_subband: 14,
            pages: vec![Page { tmsi: 0x0a1b_2c3d, msc_id: 7 }],
        }
    }

    /// One keyed burst on the ring alert channel, read through the node the
    /// receiver builds and offered to the registry the way the packet bus
    /// offers it.
    fn read(rate: f64, center: f64, offset_hz: f64, noise: f32) -> (Vec<common::Frame>, u64) {
        let bits = decode::iridium::encode_ring_alert(&a_ring_alert());
        let burst = dsp::dqpsk::key(&bits, rate, &DqpskConfig::IRIDIUM, offset_hz);
        let mut s = 0x9e37_79b9_7f4a_7c15u64;
        let mut hiss = |n: usize| -> Vec<C32> {
            (0..n)
                .map(|_| {
                    let mut next = || {
                        s = s.wrapping_mul(6364136223846793005).wrapping_add(1);
                        ((s >> 33) as f32 / (1u64 << 30) as f32 - 1.0) * noise
                    };
                    C32::new(next(), next())
                })
                .collect()
        };
        let mut iq = hiss(rate as usize / 10);
        iq.extend(burst);
        iq.extend(hiss(rate as usize / 10));
        // The burst is keyed at the channel and the tuner is parked
        // wherever the caller says, so the whole stream is shifted by the
        // difference and the node mixes it back.
        let shift = std::f64::consts::TAU * (RING_ALERT_HZ - center) / rate;
        let iq2: Vec<C32> = iq
            .iter()
            .enumerate()
            .map(|(i, x)| {
                let p = shift * i as f64;
                *x * C32::new(p.cos() as f32, p.sin() as f32)
            })
            .collect();
        let mut node = IridiumNode::default();
        let port = spec(rate, center);
        node.negotiate(&port).expect("a channel");
        let ins = [port];
        let tags = Vec::new();
        let mut frames = Vec::new();
        for block in iq2.chunks(32_768) {
            let mut out = Payload::Frames(Vec::new());
            let (mut events, mut new_tags) = (Vec::new(), Vec::new());
            let mut ctx = NodeCtx::new(0, &ins, &tags, &mut events, &mut new_tags);
            node.process(&Payload::Iq(block.to_vec()), &mut out, &mut ctx).expect("read");
            if let Payload::Frames(f) = out {
                frames.extend(f);
            }
        }
        (frames, node.frames())
    }

    #[test]
    fn the_channel_has_to_be_in_the_band_and_in_the_span() {
        let mut n = IridiumNode::default();
        assert!(n.negotiate(&spec(500_000.0, RING_ALERT_HZ)).is_ok());
        // The channel at the very edge of the span, where the doppler
        // search would be reading the skirt of the receiver's own filter.
        assert!(n.negotiate(&spec(200_000.0, RING_ALERT_HZ - 95_000.0)).is_err());
        // In the span, but the span is too narrow to hold the doppler.
        assert!(n.negotiate(&spec(100_000.0, RING_ALERT_HZ)).is_err());
        // Nowhere near the band.
        let mut off = IridiumNode::new(868_000_000.0);
        assert!(off.negotiate(&spec(500_000.0, 868_000_000.0)).is_err());
    }

    /// The whole path: a ring alert keyed on the ring alert channel, read
    /// off a span parked 100 kHz away, onto the bus and back as a row with
    /// the satellite, its beam, its page and where it is. The registry is
    /// asked which protocol the frame belongs to, because that walk is
    /// where a frame gets lost.
    #[test]
    fn a_keyed_ring_alert_comes_back_off_the_bus_as_a_row() {
        let (frames, counted) = read(500_000.0, RING_ALERT_HZ - 100_000.0, 0.0, 0.0);
        assert_eq!(frames.len(), 1);
        assert_eq!(counted, 1);
        let frame = &frames[0];
        assert_eq!(frame.center_hz, RING_ALERT_HZ as u64);
        assert!(frame.rssi_dbfs.is_finite() && frame.snr_db.is_finite());
        assert!(frame.iq.is_some(), "a frame carries the samples it was read from");

        let packet = common::Packet::of_frame(0, iridium::CHANNEL_WIDTH_HZ as u32, frame.clone());
        let rows: Vec<Decoded> = crate::protocol::frame_readers()
            .iter()
            .find_map(|p| p.read_frame(&packet, &frame.bytes))
            .expect("a protocol claimed it");
        assert_eq!(rows.len(), 1);
        let d = &rows[0];
        assert_eq!(d.protocol, "Iridium");
        assert_eq!(d.crc_ok, Some(true));
        assert!(!d.written);
        assert_eq!(d.identity.as_ref().map(|i| i.id.clone()), Some("SV108".into()));
        let detail = d.detail.as_deref().unwrap();
        assert!(detail.contains("frame=IRA"), "{detail}");
        assert!(detail.contains("channel=S.07"), "{detail}");
        assert!(detail.contains("beam=31"), "{detail}");
        assert!(detail.contains("tmsi=0a1b2c3d"), "{detail}");
        let p = d.position.clone().expect("a position");
        assert!((p.lat - 63.09).abs() < 0.02, "{}", p.lat);
        assert!((p.lon - -5.16).abs() < 0.02, "{}", p.lon);
        assert!((p.altitude_m.unwrap() - 780_000.0).abs() < 8_000.0);
    }

    /// The same burst with a satellite's doppler on it and noise 5 dB below
    /// it across the span, which is where the sensitivity is.
    ///
    /// Measured by keying this burst into uniform noise at three offsets:
    /// 4.9 dB of span SNR reads every one, 2.7 dB reads none. What gives
    /// out is the envelope gate rather than the demodulator, which reads a
    /// burst it is handed without a bit error below where the gate stops
    /// opening one.
    #[test]
    fn a_ring_alert_reads_through_doppler_and_noise() {
        for offset in [-36_000.0, -15_000.0, 21_000.0, 36_000.0] {
            let (frames, counted) = read(500_000.0, RING_ALERT_HZ, offset, 0.7);
            assert_eq!(frames.len(), 1, "one frame at {offset} Hz");
            assert_eq!(counted, 1);
        }
        // And past it nothing is invented: a burst too far under the noise
        // is a burst the gate never opens.
        let (frames, counted) = read(500_000.0, RING_ALERT_HZ, 0.0, 0.9);
        assert_eq!((frames.len(), counted), (0, 0));
    }

    /// Five minutes of noise on the channel and nothing reaches the bus: an
    /// access word turns up in noise, and the blocks behind it never check.
    #[test]
    fn noise_produces_no_frames() {
        let rate = 500_000.0;
        let mut s = 0x0123_4567_89ab_cdefu64;
        let mut node = IridiumNode::default();
        let port = spec(rate, RING_ALERT_HZ);
        node.negotiate(&port).expect("a channel");
        let ins = [port];
        let tags = Vec::new();
        let mut frames = 0usize;
        for _ in 0..(300 * 500_000 / 65_536) {
            let block: Vec<C32> = (0..65_536)
                .map(|_| {
                    let mut next = || {
                        s = s.wrapping_mul(6364136223846793005).wrapping_add(1);
                        (s >> 33) as f32 / (1u64 << 30) as f32 - 1.0
                    };
                    C32::new(next(), next())
                })
                .collect();
            let mut out = Payload::Frames(Vec::new());
            let (mut events, mut new_tags) = (Vec::new(), Vec::new());
            let mut ctx = NodeCtx::new(0, &ins, &tags, &mut events, &mut new_tags);
            node.process(&Payload::Iq(block), &mut out, &mut ctx).expect("read");
            if let Payload::Frames(f) = out {
                frames += f.len();
            }
        }
        assert_eq!(frames, 0);
        assert_eq!(node.frames(), 0);
    }

    /// Bytes that are not this front end's are not claimed, so the bus
    /// offers them on.
    #[test]
    fn a_frame_without_the_tag_is_not_claimed() {
        assert!(unpack(&[0, 1, 2, 3, 4, 5]).is_none());
        assert!(iridium_decoded(b"IRD\x00\x08\xff", Hz(RING_ALERT_HZ as u64)).is_none());
        assert!(
            Iridium
                .read_frame(
                    &common::Packet::of_frame(
                        0,
                        1_000,
                        common::Frame::measured(vec![1, 2, 3], -30.0, 10.0)
                            .at(RING_ALERT_HZ as u64),
                    ),
                    &[1, 2, 3],
                )
                .is_none()
        );
    }
}
