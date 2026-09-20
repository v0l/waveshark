//! Vaisala RS41 radiosondes as a stage: a source's stream in, sonde frames
//! out.
//!
//! A sonde is a balloon-borne transmitter keying 4800 baud GFSK on a channel
//! between 400 and 406 MHz, one 320 or 518 byte frame a second for the two
//! hours it takes to reach 35 km and burst. The waveform is
//! [`dsp::fsk::BitSync`] parameterised at that baud, the frame is
//! [`decode::rs41`], and this is the wire between them: find the header in
//! the bit stream, take the scrambler off, let the Reed-Solomon code repair
//! what it can, and put the frame on the packet bus.
//!
//! The bits arrive least significant first, and the scrambler covers the
//! header as well as the body, so there are two header constants and using
//! the wrong one finds nothing: `rs41::HEADER_AIR` is what is correlated in
//! the bit stream and `rs41::HEADER` is what that descrambles to.

use crate::NodeSpec;
use crate::protocol::{FrameClaim, Placed, Placement, Protocol, Shape};
use common::Result;
use decode::rs41;
pub use decode::rs41::read;
use dsp::fsk::BitSync;
use identify::Signal;
use pipeline::event::Request;
use pipeline::node::{NodeCtx, PortSpec, Simple};
use pipeline::port::{Payload, PortKind, StreamSpec};
use pipeline::registry::{Category, Settings, StageDesc};

/// The waveform and the band are `identify::rs41`, which anything holding a
/// recording reads without a graph.
pub use identify::rs41::{BAND, BAUD, CHANNEL_WIDTH_HZ, OCCUPIED_HZ, Rs41};

/// How far off the middle of its channel a sonde may sit before the channel
/// is moved onto it.
///
/// The transmitter drifts: the crystal is at ground temperature when the
/// balloon leaves and near -60 C at the tropopause, and Vaisala only
/// specifies the channel to within a few kilohertz to begin with. The bit
/// clock's own loop takes the offset off the bits, so what a move protects
/// is the filter in front of it: the channel passes `OCCUPIED_HZ / 2`, which
/// is 4.8 kHz either side, and the tones sit at 2.4 kHz, so the upper tone
/// starts being cut at 2.4 kHz of drift. Half of that leaves the loop room
/// to have measured the offset before the signal it measured on is being
/// attenuated.
const DRIFT_LIMIT_HZ: f32 = 1_200.0;

/// Seconds between moves. A reshape closes the stream and cuts a new one, so
/// a frame or two goes with each move, and a sonde sends one a second.
const MOVE_EVERY_S: f64 = 10.0;

pub struct Rs41Node {
    sync: Option<BitSync>,
    meter: crate::FrameMeter,
    framer: rs41::Framer,
    frames: u64,
    /// The factory calibration, a sixteenth of a frame at a time
    cal: rs41::Calibration,
    /// The middle of the stream this node was handed, which is what a
    /// measured offset is measured from.
    center_hz: f64,
    /// When the channel was last asked to move, in seconds of stream.
    moved_s: Option<f64>,
}

impl Default for Rs41Node {
    fn default() -> Self {
        Self::new()
    }
}

impl Rs41Node {
    pub fn new() -> Self {
        Self {
            sync: None,
            meter: crate::FrameMeter::new(1.0, 0, 0.6),
            framer: rs41::Framer::new(),
            frames: 0,
            cal: rs41::Calibration::new(),
            center_hz: 0.0,
            moved_s: None,
        }
    }

    /// Frames whose Reed-Solomon codewords both decoded, since the node was
    /// made.
    pub fn frames(&self) -> u64 {
        self.frames
    }

    /// What a frame says, with the air temperature the calibration allows.
    ///
    /// A sonde sends a sixteenth of its factory calibration per frame and
    /// the sensor block is ratios until the pieces that turn them into
    /// degrees have arrived, so the fold is here, where a flight is being
    /// followed, rather than in the stateless read: the balloon was sent up
    /// for the thermometer, and one frame alone cannot read it.
    fn read_flight(&mut self, bytes: &[u8]) -> Option<common::packet::Proto> {
        use common::packet::{Fact, Quantity};
        let mut p = rs41::read(bytes)?;
        let f = rs41::parse(bytes)?;
        if let Some((n, piece)) = &f.subframe {
            self.cal.feed(*n, piece);
        }
        let meas = f.meas?;
        let t = self.cal.air_temperature_c(&meas)?;
        p = p.saying(Fact::sensed(Quantity::Temperature, f64::from(t), common::Unit::Celsius));
        if let Some(rh) = self.cal.humidity_pct(&meas, t) {
            p = p.saying(Fact::sensed(Quantity::Humidity, f64::from(rh), common::Unit::Percent));
        }
        Some(p)
    }

    /// Ask for the channel to be cut `offset_hz` further along, where the
    /// sonde has drifted far enough to be worth the move.
    ///
    /// Only ever off a frame that decoded. The offset the bit clock reports
    /// is a mean of the discriminator, which on an empty channel is a mean
    /// of noise, and acting on that would walk the channel away from the
    /// band a sonde was about to appear in.
    fn follow_drift(&mut self, offset_hz: f32, c: &mut NodeCtx<'_>) {
        let now_s = c.timestamp();
        if offset_hz.abs() < DRIFT_LIMIT_HZ {
            return;
        }
        if self.moved_s.is_some_and(|at| now_s - at < MOVE_EVERY_S) {
            return;
        }
        let hz = self.center_hz + offset_hz as f64;
        self.moved_s = Some(now_s);
        c.request(Request::Reshape {
            lo_hz: hz - CHANNEL_WIDTH_HZ / 2.0,
            hi_hz: hz + CHANNEL_WIDTH_HZ / 2.0,
        });
    }
}

impl Simple for Rs41Node {
    fn name(&self) -> &str {
        "rs41"
    }

    fn negotiate(&mut self, i: &PortSpec) -> Result<StreamSpec> {
        if i.spec.kind != PortKind::Iq {
            return Err(common::Error::other("rs41 reads complex baseband"));
        }
        let s = BitSync::with_bandwidth(i.spec.rate, BAUD, OCCUPIED_HZ);
        if !s.usable() {
            return Err(common::Error::other(format!(
                "rs41 needs at least {} S/s for its {BAUD} baud",
                4.0 * BAUD
            )));
        }
        self.sync = Some(s);
        // A frame is 534 ms of air, so the ring has to be long enough to
        // give one back once it has decoded.
        self.meter = crate::FrameMeter::new(i.spec.rate, i.spec.center.0, 0.6);
        self.center_hz = i.spec.center.0 as f64;
        let mut out = i.spec.with_kind(PortKind::Packets);
        out.bandwidth = CHANNEL_WIDTH_HZ.min(i.spec.rate);
        Ok(out)
    }

    fn process(&mut self, i: &Payload, o: &mut Payload, c: &mut NodeCtx<'_>) -> Result<()> {
        let (Some(iq), Some(s)) = (i.as_iq(), self.sync.as_mut()) else {
            return Ok(());
        };
        self.meter.feed(iq);
        s.process(iq, self.framer.sink());
        let offset_hz = s.offset_hz();
        let frames = self.framer.take();
        let decoded = !frames.is_empty();
        for frame in frames {
            self.frames += 1;
            // Both Reed-Solomon codewords decoded and every block's CRC
            // passed, or the framer would not have handed this over.
            let keying = common::packet::Keying::configured(common::Modulation::Fsk2)
                .of(common::packet::KeyingParams { baud: BAUD as f32, ..Default::default() });
            let mut p = self
                .meter
                .packet_now(frame)
                .keyed(keying)
                .checked(common::packet::Integrity::Passed);
            if let Some(read) = self.read_flight(p.bytes()) {
                p = p.decoded(read);
            }
            o.packets_mut().push(p);
        }
        if decoded {
            self.follow_drift(offset_hz, c);
        }
        self.framer.trim();
        Ok(())
    }

    fn reset(&mut self) {
        self.meter.reset();
        self.framer.reset();
        self.cal = rs41::Calibration::new();
        self.moved_s = None;
        if let Some(s) = &mut self.sync {
            s.reset();
        }
    }
}

impl Protocol for Rs41 {
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
    /// The middle of the band. Europe launches mostly between 402 and 405
    /// MHz, and a sonde is found by scanning rather than by being known.
    fn default_hz(&self) -> f64 {
        403_000_000.0
    }
    fn shape(&self) -> Shape {
        Signal::shape(self)
    }
    /// The six megahertz of the sonde band, which nothing else here claims.
    fn frame_claim(&self) -> FrameClaim {
        FrameClaim::Band { width_hz: (BAND.1 - BAND.0) as u64 }
    }
    /// Kept while it is being read, and given back half a minute after the
    /// last frame.
    ///
    /// Not for the session, which is what a channel that stays put wants.
    /// The node follows the transmitter's drift by asking for its channel to
    /// be recut ([`Rs41Node::follow_drift`]), and that only works while
    /// something is still decoding: drift that outruns the filter during a
    /// fade leaves a channel nothing decodes on, and a latch held for the
    /// session would keep the detector out of the 30 kHz around it for the
    /// rest of the run, which is three raster steps the sonde could have
    /// moved to. Thirty frames of silence is a sonde that has gone or gone
    /// somewhere else, and either way the band is better off back with the
    /// detector.
    fn stickiness(&self) -> crate::protocol::Stickiness {
        crate::protocol::Stickiness::Latch { hold_s: Some(30.0) }
    }
    /// The band alone is not enough to claim a frame here: a DFM is launched
    /// into the same six megahertz, so the frame has to be the length of an
    /// RS41's and start with its header before this refuses to pass it on.
    fn stated(&self, p: &common::packet::Packet) -> Option<Vec<common::packet::Proto>> {
        let bytes = p.bytes();
        let hz = p.center_hz() as f64;
        if !(BAND.0..BAND.1).contains(&hz) {
            return None;
        }
        if !matches!(bytes.len(), rs41::FRAME_STD | rs41::FRAME_AUX)
            || bytes.get(..8) != Some(&rs41::HEADER[..])
        {
            return None;
        }
        Some(read(bytes).into_iter().collect())
    }
    fn reports_position(&self) -> bool {
        true
    }
    fn chain(&self, _at: Placed) -> Vec<NodeSpec> {
        vec![NodeSpec::new("rs41")]
    }
}

pub const DESC: StageDesc = StageDesc {
    name: "rs41",
    summary: "Vaisala RS41 radiosonde frames, 4800 baud GFSK at 400 to 406 MHz",
    category: Category::Decode,
    feeds_bus: true,
};

pub fn build(_s: &Settings) -> Result<Box<dyn pipeline::node::Node>> {
    Ok(Box::new(Rs41Node::new()))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The height a layer stated, which is a reading rather than a place.
    fn height(d: &common::packet::Proto) -> Option<f64> {
        d.facts.iter().find_map(|f| match f {
            common::packet::Fact::Sensed(r) if r.quantity == common::packet::Quantity::Altitude => {
                Some(r.value)
            }
            _ => None,
        })
    }

    /// A frame made here, keyed at 4800 baud and read back off the samples,
    /// which is the whole chain this file is: bit clock, header search,
    /// descramble, Reed-Solomon, parse.
    pub(super) fn a_frame(serial: &[u8; 8], frame_no: u16) -> Vec<u8> {
        let mut f = vec![0u8; rs41::FRAME_STD];
        f[..8].copy_from_slice(&rs41::HEADER);
        f[56] = 0x0F;
        let mut body = vec![0u8; 0x28];
        body[..2].copy_from_slice(&frame_no.to_le_bytes());
        body[2..10].copy_from_slice(serial);
        body[0x0A] = 27;
        body[0x0D] = 0x01;
        let mut at = 57;
        let put = |id: u8, body: &[u8], f: &mut Vec<u8>, at: &mut usize| {
            f[*at] = id;
            f[*at + 1] = body.len() as u8;
            f[*at + 2..*at + 2 + body.len()].copy_from_slice(body);
            let crc = rs41::block_crc(body).to_le_bytes();
            f[*at + 2 + body.len()..*at + 4 + body.len()].copy_from_slice(&crc);
            *at += 4 + body.len();
        };
        put(0x79, &body, &mut f, &mut at);
        // A position block placing it over the Irish Sea at 4712 m above
        // the ellipsoid, which is where these earth-centred metres are.
        let mut pos = vec![0u8; 0x15];
        let (x, y, z) = (3_800_000.0f64, -340_000.0f64, 5_100_000.0f64);
        pos[0..4].copy_from_slice(&((x * 100.0) as i32).to_le_bytes());
        pos[4..8].copy_from_slice(&((y * 100.0) as i32).to_le_bytes());
        pos[8..12].copy_from_slice(&((z * 100.0) as i32).to_le_bytes());
        pos[0x12] = 11;
        put(0x7B, &pos, &mut f, &mut at);
        let pad = vec![0u8; rs41::FRAME_STD - at - 4];
        put(0x76, &pad, &mut f, &mut at);
        assert_eq!(at, rs41::FRAME_STD);
        rs41::protect(&mut f);
        f
    }

    /// Frame bytes back onto the air: scrambled from the parity on, then
    /// keyed least significant bit first.
    pub(super) fn key(frame: &[u8], rate: f64) -> Vec<common::C32> {
        let mut wire = frame.to_vec();
        rs41::descramble(&mut wire);
        assert_eq!(wire[..8], rs41::HEADER_AIR);
        let bits: Vec<bool> =
            wire.iter().flat_map(|b| (0..8).map(move |k| b >> k & 1 != 0)).collect();
        // A run of alternating symbols first, so the timing loop has
        // something to lock to before the header arrives, as a sonde's own
        // preamble gives it.
        let mut wire: Vec<bool> = (0..200).map(|i| i % 2 == 0).collect();
        wire.extend(bits);
        // And a tail, because the last bit is only decided once the symbol
        // after it has gone through the filter.
        wire.extend((0..40).map(|i| i % 2 == 0));
        dsp::fsk::modulate(&wire, rate, BAUD, 2_400.0, 0.5)
    }

    #[test]
    fn a_keyed_frame_is_read_off_the_samples() {
        let rate = 48_000.0;
        let frame = a_frame(b"W1234567", 900);
        let iq = key(&frame, rate);
        let mut n = Rs41Node::new();
        n.sync = Some(BitSync::with_bandwidth(rate, BAUD, OCCUPIED_HZ));
        let mut got: Vec<Vec<u8>> = Vec::new();
        for block in iq.chunks(2048) {
            n.sync.as_mut().unwrap().process(block, n.framer.sink());
            got.extend(n.framer.take());
        }
        assert_eq!(got.len(), 1, "{} frames off one transmission", got.len());
        assert_eq!(got[0], frame, "the bytes are not the ones that were keyed");
        let d = read(&got[0]).expect("a decode");
        assert_eq!(d.subject.as_ref().map(|e| e.id.to_string()).as_deref(), Some("W1234567"));
        let p = d.placed().expect("a position");
        assert!((p.lat - 53.385_054).abs() < 1e-5, "{}", p.lat);
        assert!((p.lon + 5.112_850).abs() < 1e-5, "{}", p.lon);
        assert!(height(&d).is_some_and(|m| (m - 4_712.22).abs() < 0.01), "{d:?}");
    }

    /// A sonde keyed 1.6 kHz off the middle of its channel still decodes,
    /// and the node asks for the channel to be cut where the sonde actually
    /// is. One request, not one a block: the move is held off until either
    /// the channel has been recut or ten seconds have passed.
    #[test]
    fn a_drifted_sonde_moves_its_channel() {
        let (rate, center, drift) = (48_000.0, 403_000_000.0, 1_600.0);
        let frame = a_frame(b"W7654321", 71);
        let keyed = key(&frame, rate);
        let iq: Vec<common::C32> = keyed
            .iter()
            .enumerate()
            .map(|(i, s)| {
                let ph = std::f64::consts::TAU * drift * i as f64 / rate;
                s * common::C32::new(ph.cos() as f32, ph.sin() as f32)
            })
            .collect();

        let port = PortSpec { spec: StreamSpec::iq(rate, common::Hz(center as u64)), latency: 0 };
        let mut n = Rs41Node::new();
        n.negotiate(&port).unwrap();

        let ins = [port];
        let tags = Vec::new();
        let mut frames = 0;
        let mut moves: Vec<f64> = Vec::new();
        for block in iq.chunks(2048) {
            let input = Payload::Iq(block.to_vec());
            let mut out = Payload::Packets(Vec::new());
            let (mut events, mut new_tags) = (Vec::new(), Vec::new());
            let mut ctx = NodeCtx::new(0, &ins, &tags, &mut events, &mut new_tags);
            n.process(&input, &mut out, &mut ctx).unwrap();
            if let Payload::Packets(f) = out {
                frames += f.len();
            }
            for e in events {
                if let pipeline::event::Event::Request(Request::Reshape { lo_hz, hi_hz }) = e {
                    assert!(
                        (hi_hz - lo_hz - CHANNEL_WIDTH_HZ).abs() < 1.0,
                        "{} Hz wide",
                        hi_hz - lo_hz
                    );
                    moves.push((lo_hz + hi_hz) / 2.0);
                }
            }
        }

        assert_eq!(frames, 1, "{frames} frames off one drifted transmission");
        assert_eq!(moves.len(), 1, "{} channel moves off one frame", moves.len());
        let want = center + drift;
        // The offset is a one-pole mean over 64 symbols, so it lands within
        // a couple of hundred hertz rather than exactly.
        assert!((moves[0] - want).abs() < 300.0, "moved to {:.0} Hz, wanted {want:.0}", moves[0]);
    }

    /// Twenty seconds of noise moves nothing. The offset the bit clock
    /// reports on an empty channel is a mean of noise, and a channel that
    /// followed it would walk off the band.
    #[test]
    fn noise_does_not_move_the_channel() {
        let (rate, center) = (48_000.0, 403_000_000.0);
        let mut seed = 0x0f1e_2d3c_4b5a_6978u64;
        let mut rng = move || {
            seed ^= seed << 13;
            seed ^= seed >> 7;
            seed ^= seed << 17;
            (seed >> 40) as f32 / 8_388_608.0 - 1.0
        };
        let port = PortSpec { spec: StreamSpec::iq(rate, common::Hz(center as u64)), latency: 0 };
        let mut n = Rs41Node::new();
        n.negotiate(&port).unwrap();
        let ins = [port];
        let tags = Vec::new();
        let mut asked = 0;
        for _ in 0..(rate as usize * 20 / 4096) {
            let block: Vec<common::C32> =
                (0..4096).map(|_| common::C32::new(rng(), rng())).collect();
            let input = Payload::Iq(block);
            let mut out = Payload::Packets(Vec::new());
            let (mut events, mut new_tags) = (Vec::new(), Vec::new());
            let mut ctx = NodeCtx::new(0, &ins, &tags, &mut events, &mut new_tags);
            n.process(&input, &mut out, &mut ctx).unwrap();
            asked +=
                events.iter().filter(|e| matches!(e, pipeline::event::Event::Request(_))).count();
        }
        assert_eq!(asked, 0, "{asked} requests out of twenty seconds of noise");
    }

    /// Noise on the channel produces no frames at all. A header search with
    /// four bits of slack finds candidates in noise regularly, and the only
    /// thing that stops one being reported is the code refusing it.
    #[test]
    fn noise_produces_no_frames() {
        let rate = 48_000.0;
        let mut seed = 0x1234_5678_9abc_def0u64;
        let mut rng = move || {
            seed ^= seed << 13;
            seed ^= seed >> 7;
            seed ^= seed << 17;
            (seed >> 40) as f32 / 8_388_608.0 - 1.0
        };
        let iq: Vec<common::C32> =
            (0..rate as usize * 20).map(|_| common::C32::new(rng(), rng())).collect();
        let mut n = Rs41Node::new();
        n.sync = Some(BitSync::with_bandwidth(rate, BAUD, OCCUPIED_HZ));
        let mut frames = 0;
        for block in iq.chunks(4096) {
            n.sync.as_mut().unwrap().process(block, n.framer.sink());
            frames += n.framer.take().len();
            n.framer.trim();
        }
        assert_eq!(frames, 0, "{frames} frames out of twenty seconds of noise");
    }
}
