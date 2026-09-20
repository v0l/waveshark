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
pub use decode::iridium::MAX_FRAME_BITS;
pub use decode::iridium::TAG;
pub use decode::iridium::pack;
pub use decode::iridium::read;
pub use decode::iridium::unpack;
use decode::iridium::{self, RING_ALERT_HZ};
use dsp::dqpsk::{DqpskBurst, DqpskConfig, DqpskDemod};
use dsp::{FirDecim, Mixer};
use identify::Signal;
pub use identify::iridium::FEED_HZ;
pub use identify::iridium::Iridium;
pub use identify::iridium::WORK_HZ;
use pipeline::node::{NodeCtx, PortSpec, Simple};
use pipeline::port::{Payload, PortKind, StreamSpec};
use pipeline::registry::{Category, Settings, SettingsExt, StageDesc};

/// Half the width the channel filter passes: the channel itself and the
/// doppler either side of it.
const HALF_PASS_HZ: f64 = iridium::CHANNEL_WIDTH_HZ / 2.0 + iridium::DOPPLER_HZ;

/// The carrier this stage is pointed at.
const CHANNEL_HZ: &str = "channel_hz";

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

        let mut out = i.spec.with_kind(PortKind::Packets);
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
        let out = o.packets_mut();
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
            let mut pkt = crate::measured(
                self.channel_hz as u64,
                iridium::CHANNEL_WIDTH_HZ as u32,
                bytes,
                b.rssi_dbfs,
                b.snr_db,
            );
            let length = (b.bits.len() / 2) as f64 * self.demod.sps();
            pkt.carrier.iq = self.meter.iq_at(b.start_sample, length as usize);
            out.push(pkt);
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

impl Protocol for Iridium {
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

    /// The downlink band, duplex channels and the simplex ones above them:
    /// ring alerts are on the simplex channels and broadcasts on the duplex,
    /// and a scanner block may name either.

    /// The ring alert channel, which is the one frequency worth parking on:
    /// every satellite overhead transmits there.

    /// The front end's own tag, which is the only thing that tells these
    /// bits from any other burst read on an L-band centre: Inmarsat's
    /// channels are in the same band and its frames are bytes too.
    fn frame_claim(&self) -> FrameClaim {
        FrameClaim::Tagged
    }
    fn stated(&self, p: &common::packet::Packet) -> Option<Vec<common::packet::Proto>> {
        let bytes = p.bytes();
        read(bytes, common::Hz(p.center_hz())).map(|d| vec![d])
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
    fn read(
        rate: f64,
        center: f64,
        offset_hz: f64,
        noise: f32,
    ) -> (Vec<common::packet::Packet>, u64) {
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
            let mut out = Payload::Packets(Vec::new());
            let (mut events, mut new_tags) = (Vec::new(), Vec::new());
            let mut ctx = NodeCtx::new(0, &ins, &tags, &mut events, &mut new_tags);
            node.process(&Payload::Iq(block.to_vec()), &mut out, &mut ctx).expect("read");
            if let Payload::Packets(f) = out {
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
        assert_eq!(frame.carrier.center_hz, RING_ALERT_HZ as u64);
        assert!(frame.carrier.rssi_dbfs.is_finite() && frame.carrier.snr_db.is_finite());
        assert!(frame.carrier.iq.is_some(), "a frame carries the samples it was read from");

        let rows = crate::protocol::frame_readers()
            .iter()
            .find_map(|p| p.stated(frame))
            .expect("a protocol claimed it");
        assert_eq!(rows.len(), 1);
        let d = &rows[0];
        assert_eq!(d.id, "iridium");
        assert!(d.wrote().is_none());
        assert_eq!(d.subject.as_ref().map(|e| e.id.to_string()).as_deref(), Some("SV108"));
        let p = d.placed().expect("a position");
        assert!((p.lat - 63.09).abs() < 0.02, "{}", p.lat);
        assert!((p.lon - -5.16).abs() < 0.02, "{}", p.lon);
        // The height it reported, as a reading rather than part of the place.
        assert!(d.facts.iter().any(|f| matches!(
            f,
            common::packet::Fact::Sensed(r)
                if r.quantity == common::packet::Quantity::Altitude
                    && (r.value - 780_000.0).abs() < 8_000.0
        )));
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
            let mut out = Payload::Packets(Vec::new());
            let (mut events, mut new_tags) = (Vec::new(), Vec::new());
            let mut ctx = NodeCtx::new(0, &ins, &tags, &mut events, &mut new_tags);
            node.process(&Payload::Iq(block), &mut out, &mut ctx).expect("read");
            if let Payload::Packets(f) = out {
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
        assert!(decode::iridium::read(b"IRD\x00\x08\xff", Hz(RING_ALERT_HZ as u64)).is_none());
        let p = crate::measured(RING_ALERT_HZ as u64, 1_000, vec![1, 2, 3], -30.0, 10.0);
        assert!(Iridium.stated(&p).is_none());
    }
}
