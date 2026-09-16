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
use dsp::fsk::BitSync;
use pipeline::event::{Decoded, Request};
use pipeline::node::{NodeCtx, PortSpec, Simple};
use pipeline::port::{Payload, PortKind, StreamSpec};
use pipeline::registry::{Category, Settings, StageDesc};

/// Symbols a second.
pub const BAUD: f64 = 4_800.0;

/// The channel a sonde is tuned to, which is the 10 kHz raster Vaisala
/// steps through the band on. The signal itself is 9.6 kHz, so the channel
/// is barely wider than what is in it and the two neighbours are clear.
pub const CHANNEL_WIDTH_HZ: f64 = 10_000.0;

/// What the signal itself occupies: 4800 baud keyed 2.4 kHz either way is
/// 9.6 kHz by Carson's rule, which is also what Vaisala quotes.
pub const OCCUPIED_HZ: f64 = 9_600.0;

/// The band sondes are launched into (ITU meteorological aids, region 1 and
/// beyond). Vaisala tunes an RS41 anywhere in it in 10 kHz steps.
pub const BAND: (f64, f64) = (400_000_000.0, 406_000_000.0);

/// Bits held while looking for a header. Two long frames and the gap
/// between them: enough that a frame straddling two blocks is never lost,
/// and bounded so a channel with nothing on it cannot grow.
const MAX_BITS: usize = rs41::FRAME_AUX * 8 * 3;

/// Header bit errors tolerated. The header is 64 bits and a sonde at the
/// edge of reception loses a few; more than this and it is not a header.
/// Four leaves a false alarm rate of about one in a million bit positions,
/// which at 4800 baud is one spurious search every three minutes and costs
/// nothing, because the Reed-Solomon code then refuses it.
const HEADER_SLACK: u32 = 4;

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
    bits: Vec<bool>,
    /// Bits already searched and known not to start a header. Only appended
    /// to, so what was rejected stays rejected.
    scanned: usize,
    frames: u64,
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
            bits: Vec::new(),
            scanned: 0,
            frames: 0,
            center_hz: 0.0,
            moved_s: None,
        }
    }

    /// Frames whose Reed-Solomon codewords both decoded, since the node was
    /// made.
    pub fn frames(&self) -> u64 {
        self.frames
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

    /// Look for headers in the bits held, returning every frame behind one.
    fn search(&mut self) -> Vec<Vec<u8>> {
        let mut out = Vec::new();
        let header = header_bits();
        let mut at = self.scanned;
        while at + header.len() <= self.bits.len() {
            let mut wrong = 0u32;
            for (k, &want) in header.iter().enumerate() {
                if self.bits[at + k] != want {
                    wrong += 1;
                    if wrong > HEADER_SLACK {
                        break;
                    }
                }
            }
            if wrong > HEADER_SLACK {
                at += 1;
                continue;
            }
            match self.read_frame(at) {
                // A header with a frame behind it: take both out of the
                // buffer so the search does not walk back into them.
                Some(Some(frame)) => {
                    let used = frame.len() * 8;
                    out.push(frame);
                    self.bits.drain(..at + used);
                    at = 0;
                    self.scanned = 0;
                }
                // A header whose frame has not all arrived: wait here.
                Some(None) => {
                    self.scanned = at;
                    return out;
                }
                None => at += 1,
            }
        }
        self.scanned = at;
        out
    }

    /// Read the frame starting at bit `at`. `None` where those bits are not
    /// a frame, `Some(None)` where not enough of them have arrived.
    fn read_frame(&self, at: usize) -> Option<Option<Vec<u8>>> {
        // Not enough bits yet is not the same answer as not a frame: the
        // search resumes where it stopped, so treating a short buffer as a
        // rejection walks the cursor past the header and loses the frame
        // that was about to arrive.
        let Some(mut probe) = pack(&self.bits, at, rs41::FRAME_STD) else { return Some(None) };
        rs41::descramble(&mut probe);
        // The length marker sits in the data, so it has to be read before
        // the code has passed on it; a wrong bit here costs one frame.
        let len = rs41::frame_len(probe[rs41::DATA_AT])?;
        let mut frame = if len == rs41::FRAME_STD {
            probe
        } else {
            let Some(mut long) = pack(&self.bits, at, len) else { return Some(None) };
            rs41::descramble(&mut long);
            long
        };
        rs41::correct(&mut frame)?;
        Some(Some(frame))
    }
}

/// The header as it arrives: eight bytes, least significant bit first.
fn header_bits() -> Vec<bool> {
    rs41::HEADER_AIR.iter().flat_map(|b| (0..8).map(move |k| b >> k & 1 != 0)).collect()
}

/// `len` bytes from bit `at`, least significant bit first, or `None` where
/// the bits are not all there.
fn pack(bits: &[bool], at: usize, len: usize) -> Option<Vec<u8>> {
    if at + len * 8 > bits.len() {
        return None;
    }
    Some(
        bits[at..at + len * 8]
            .chunks(8)
            .map(|c| c.iter().enumerate().fold(0u8, |b, (k, &s)| b | (s as u8) << k))
            .collect(),
    )
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
        let mut out = i.spec.with_kind(PortKind::Frames);
        out.bandwidth = CHANNEL_WIDTH_HZ.min(i.spec.rate);
        Ok(out)
    }

    fn process(&mut self, i: &Payload, o: &mut Payload, c: &mut NodeCtx<'_>) -> Result<()> {
        let (Some(iq), Some(s)) = (i.as_iq(), self.sync.as_mut()) else {
            return Ok(());
        };
        self.meter.feed(iq);
        s.process(iq, &mut self.bits);
        let offset_hz = s.offset_hz();
        let frames = self.search();
        let decoded = !frames.is_empty();
        for frame in frames {
            self.frames += 1;
            o.frames_mut().push(self.meter.frame(frame));
        }
        if decoded {
            self.follow_drift(offset_hz, c);
        }
        if self.bits.len() > MAX_BITS {
            let drop = self.bits.len() - MAX_BITS;
            self.bits.drain(..drop);
            self.scanned = self.scanned.saturating_sub(drop);
        }
        Ok(())
    }

    fn reset(&mut self) {
        self.meter.reset();
        self.bits.clear();
        self.scanned = 0;
        self.moved_s = None;
        if let Some(s) = &mut self.sync {
            s.reset();
        }
    }
}

/// What the protocols node makes of a sonde frame: which balloon it is,
/// where, and how it is flying.
pub fn rs41_decoded(bytes: &[u8], center: common::Hz) -> Option<Decoded> {
    let f = rs41::parse(bytes)?;
    let mut fields: Vec<(String, common::Value)> = vec![
        ("serial".into(), common::Value::Text(f.serial.clone())),
        ("frame".into(), common::Value::Int(f.frame_no as i64)),
        ("battery_v".into(), common::Value::Float(f.battery_v as f64)),
        ("state".into(), common::Value::Text(f.flight.label().into())),
    ];
    if f.has_position() {
        fields.push(("altitude_m".into(), common::Value::Float(f.altitude_m)));
        fields.push(("climb_ms".into(), common::Value::Float(f.climb_ms)));
        fields.push(("speed_kt".into(), common::Value::Float(f.speed_kt)));
        fields.push(("course_deg".into(), common::Value::Float(f.course_deg)));
        fields.push(("satellites".into(), common::Value::Int(f.satellites as i64)));
    }
    if let (Some(w), Some(t)) = (f.gps_week, f.gps_tow_ms) {
        fields.push(("gps_week".into(), common::Value::Int(w as i64)));
        fields.push(("gps_tow_ms".into(), common::Value::Int(t as i64)));
    }
    fields.push(("pcb_temp_c".into(), common::Value::Int(f.pcb_temp_c as i64)));
    if f.bad_blocks > 0 {
        fields.push(("bad_blocks".into(), common::Value::Int(f.bad_blocks as i64)));
    }

    let mut d = Decoded::bytes("rs41", center, 0.0, bytes.to_vec())
        .with_modulation(common::Modulation::Fsk2)
        .with_crc(Some(f.bad_blocks == 0))
        .with_text(f.summary())
        .with_detail(format!("frame {}, {:.1} V, {}", f.frame_no, f.battery_v, f.flight.label()))
        .with_fields(fields)
        .by(common::Identity::new("vaisala", f.serial.clone()).made_by("Vaisala"));
    if f.has_position() {
        d = d
            .reporting(common::ReportDetail::Sonde {
                altitude_m: f.altitude_m,
                climb_ms: f.climb_ms,
                battery_v: f.battery_v,
                satellites: f.satellites,
                descending: f.flight == rs41::Flight::Descent,
                sensors: f.meas.map(|meas| common::SondeSensors { meas, calibration: f.subframe }),
            })
            .at_position(common::Position {
                lat: f.lat_deg,
                lon: f.lon_deg,
                altitude_m: Some(f.altitude_m),
                speed_kt: Some(f.speed_kt),
                course_deg: Some(f.course_deg),
            });
    }
    Some(d)
}

pub struct Rs41;

impl Protocol for Rs41 {
    fn id(&self) -> &'static str {
        "rs41"
    }
    fn label(&self) -> &'static str {
        "rs41"
    }
    fn aliases(&self) -> &'static [&'static str] {
        &["radiosonde", "sonde", "vaisala"]
    }
    /// The meteorological aids allocation. Placed by band rather than
    /// anywhere: 4800 baud FSK in a 15 kHz channel is a shape a great many
    /// things have, and outside this band none of them is a sonde.
    fn placement(&self) -> Placement {
        Placement::Bands(vec![BAND])
    }
    /// The middle of the band. Europe launches mostly between 402 and 405
    /// MHz, and a sonde is found by scanning rather than by being known.
    fn default_hz(&self) -> f64 {
        403_000_000.0
    }
    fn shape(&self) -> Shape {
        Shape {
            widths: &[CHANNEL_WIDTH_HZ],
            // Four samples a symbol is the demodulator's floor.
            min_rate_hz: 4.0 * BAUD,
            // Ten, which leaves the timing loop room to interpolate.
            feed_rate_hz: 48_000.0,
            span_wide: false,
            families: &[],
        }
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
    fn read_frame(&self, p: &common::Packet, bytes: &[u8]) -> Option<Vec<Decoded>> {
        let hz = p.center_hz() as f64;
        if !(BAND.0..BAND.1).contains(&hz) {
            return None;
        }
        Some(rs41_decoded(bytes, common::Hz(p.center_hz())).into_iter().collect())
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
        let mut got = Vec::new();
        for block in iq.chunks(2048) {
            let mut bits = Vec::new();
            n.sync.as_mut().unwrap().process(block, &mut bits);
            n.bits.extend(bits);
            got.extend(n.search());
        }
        assert_eq!(got.len(), 1, "{} frames off one transmission", got.len());
        assert_eq!(got[0], frame, "the bytes are not the ones that were keyed");
        let d = rs41_decoded(&got[0], common::Hz(403_000_000)).expect("a decode");
        assert_eq!(d.field("serial").map(|v| v.to_string()).as_deref(), Some("W1234567"));
        assert_eq!(d.crc_ok, Some(true));
        let p = d.position.expect("a position");
        assert!((p.lat - 53.385_054).abs() < 1e-5, "{}", p.lat);
        assert!((p.lon + 5.112_850).abs() < 1e-5, "{}", p.lon);
        assert!((p.altitude_m.unwrap() - 4_712.22).abs() < 0.01, "{:?}", p.altitude_m);
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
            let mut out = Payload::Frames(Vec::new());
            let (mut events, mut new_tags) = (Vec::new(), Vec::new());
            let mut ctx = NodeCtx::new(0, &ins, &tags, &mut events, &mut new_tags);
            n.process(&input, &mut out, &mut ctx).unwrap();
            if let Payload::Frames(f) = out {
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
            let mut out = Payload::Frames(Vec::new());
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
            let mut bits = Vec::new();
            n.sync.as_mut().unwrap().process(block, &mut bits);
            n.bits.extend(bits);
            frames += n.search().len();
            if n.bits.len() > MAX_BITS {
                let drop = n.bits.len() - MAX_BITS;
                n.bits.drain(..drop);
                n.scanned = n.scanned.saturating_sub(drop);
            }
        }
        assert_eq!(frames, 0, "{frames} frames out of twenty seconds of noise");
    }
}
