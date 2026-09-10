//! DJI DroneID as a graph node.
//!
//! Wiring only, like `ble_nodes` and `wifi_nodes`: the OFDM receiver is
//! `dsp::droneid`, the frame is `decode::droneid`, and neither knows about
//! the other or about pipelines.
//!
//! # Where it can be
//!
//! On the handful of centres DJI transmits DroneID on, which are known by
//! observation rather than from any specification. The frame is defined at
//! 15.36 MS/s and occupies about 10 MHz, so this is a channel a HackRF can
//! hold whole and an RTL-SDR cannot reach at all.
//!
//! Those centres are fixed, so this reads the span and cuts them out of it
//! the way `wifi_nodes` reads its channels: one energy gate off one
//! transform a block, and a correlator per centre that runs only while that
//! centre is lit. It was placed per source instead, on anything the detector
//! opened between five and fifteen megahertz wide, and on a busy 2.4 GHz
//! band that is Wi-Fi splatter: eleven sources over the same spectrum, each
//! extracted at 15.36 MS/s and each correlating for the same five possible
//! bursts. A drone sits inside a Wi-Fi channel rather than taking turns with
//! it, so both front ends read the span at once, always.
//!
//! # What a row is evidence of
//!
//! A frame that reaches the bus passed a CRC-16 over 89 bytes this project
//! did not construct, inside a code block whose own CRC-24 also checks. What
//! the fields say is still the aircraft's claim, and an aircraft with no fix
//! sends zeros, which `decode::droneid` reports as absent rather than as a
//! position off Africa.

use crate::protocol::{FrameClaim, Mark, Placed, Placement, Protocol, Shape, Stickiness};
use crate::NodeSpec;
use common::Result;
use pipeline::event::Decoded;
use pipeline::node::{NodeCtx, PortSpec, Simple};
use pipeline::port::{Payload, PortKind, StreamSpec};
use pipeline::registry::{Category, Settings, StageDesc};

/// The occupied bandwidth: 600 carriers 15 kHz apart, plus guards.
pub const WIDTH_HZ: f64 = dsp::droneid::WIDTH_HZ;

/// The rate the frame is defined at, which is also the rate this asks for.
pub const RATE_HZ: f64 = dsp::droneid::RATE;

/// The centre a receiver picks when it has to pick one: the middle of the
/// 2.4 GHz set, where the bursts in the bench captures were.
pub const DEFAULT_HZ: f64 = 2_444_500_000.0;

/// How strongly the Zadoff-Chu symbol has to correlate. Off air, a burst
/// scores 0.88 to 0.99 and nothing else in a busy 2.4 GHz band comes near
/// half of that, so this is not a knife edge.
const THRESHOLD: f32 = 0.5;

/// Every centre DroneID has been seen on, 2.4 GHz then 5.8.
pub fn channels() -> Vec<f64> {
    dsp::droneid::CENTERS_2G4_HZ
        .iter()
        .chain(dsp::droneid::CENTERS_5G8_HZ.iter())
        .copied()
        .collect()
}

/// The tag in front of a frame on the bus, so a row can be told from any
/// other 91 bytes arriving on the same centre.
const TAG: [u8; 4] = *b"DJID";

pub fn wrap(frame: &[u8]) -> Vec<u8> {
    let mut v = TAG.to_vec();
    v.extend_from_slice(frame);
    v
}

pub struct DroneIdNode {
    span: Option<dsp::droneid::DroneIdSpan>,
    bursts: Vec<dsp::droneid::SpanBurst>,
    accepted: u64,
}

impl Default for DroneIdNode {
    fn default() -> Self {
        Self::new()
    }
}

impl DroneIdNode {
    pub fn new() -> Self {
        Self { span: None, bursts: Vec::new(), accepted: 0 }
    }

    /// Frames that passed their CRC since the node was built.
    pub fn accepted(&self) -> u64 {
        self.accepted
    }

    /// The centres being read: every one the span holds whole.
    pub fn channels(&self) -> Vec<f64> {
        self.span.as_ref().map(|s| s.channels()).unwrap_or_default()
    }
}

impl Simple for DroneIdNode {
    fn name(&self) -> &'static str {
        "droneid"
    }

    fn negotiate(&mut self, input: &PortSpec) -> Result<StreamSpec> {
        if input.spec.kind != PortKind::Iq {
            return Err(common::Error::other("droneid reads complex baseband"));
        }
        let (rate, center) = (input.spec.rate, input.spec.center.as_f64());
        if rate + 1.0 < RATE_HZ {
            return Err(common::Error::other(
                "droneid needs 15.36 MS/s: the frame is 600 carriers 15 kHz apart",
            ));
        }
        let Some(span) = dsp::droneid::DroneIdSpan::new(rate, center, &channels(), THRESHOLD)
        else {
            return Err(common::Error::other("droneid needs a whole 10 MHz centre in the span"));
        };
        // Where the port says the frames came from: the one centre when the
        // span holds one, and the span itself when it holds several, because
        // a frame cannot then be placed by the port alone. The rule
        // `wifi_nodes` and `ble_nodes` follow, and each frame carries its own
        // centre.
        let heard = span.channels();
        let hz = match heard.as_slice() {
            [one] => *one,
            _ => center,
        };
        self.span = Some(span);
        let mut out = input.spec.with_kind(PortKind::Frames);
        out.center = common::Hz(hz as u64);
        out.bandwidth = WIDTH_HZ;
        Ok(out)
    }

    fn process(&mut self, input: &Payload, output: &mut Payload, ctx: &mut NodeCtx) -> Result<()> {
        let (Payload::Iq(iq), Some(span)) = (input, self.span.as_mut()) else {
            return Ok(());
        };
        let _ = ctx;
        self.bursts.clear();
        span.process(iq, &mut self.bursts);
        let out = output.frames_mut();
        for b in &self.bursts {
            // The bits are a frame when the CRC-16 the aircraft computed says
            // so; a correlation peak that was not a burst does not get that
            // far.
            if decode::droneid::parse(&b.frame).is_none() {
                continue;
            }
            self.accepted += 1;
            let mut frame = common::Frame::measured(wrap(&b.frame), b.rssi_dbfs, b.snr_db)
                .at(b.center_hz as u64);
            // The whole burst is the frame's own samples, at the rate the
            // centre was read at rather than the span's.
            frame.iq = Some(std::sync::Arc::new(common::IqBurst {
                rate: b.rate,
                center_hz: b.center_hz as u64,
                samples: b.samples.clone(),
            }));
            out.push(frame);
        }
        Ok(())
    }

    fn reset(&mut self) {
        if let Some(s) = self.span.as_mut() {
            s.reset();
        }
        self.bursts.clear();
    }
}

/// The row a frame becomes.
///
/// `None` when the bytes are not a DroneID frame, which is how the packet bus
/// tells one from anything else arriving on the same centre.
pub fn droneid_decoded(bytes: &[u8], center: common::Hz) -> Option<Decoded> {
    use common::Value;
    if bytes.len() < TAG.len() + decode::droneid::FRAME_LEN || bytes[..4] != TAG {
        return None;
    }
    let f = decode::droneid::parse(&bytes[4..])?;
    let mut fields = decode::droneid::fields(&f);
    fields.push(("protocol".into(), Value::Text("DJI DroneID".into())));
    let detail = fields.iter().map(|(k, v)| format!("{k}={v}")).collect::<Vec<_>>().join(" ");

    // The row is filed under the airframe's serial, which is the identity
    // this protocol exists to broadcast and is printed on the aircraft.
    let mut who = common::Identity::new("dji", f.serial.clone());
    who.name = decode::droneid::device_name(f.device_type).map(str::to_string);
    who.vendor = Some("DJI".into());
    let mut d = Decoded::bytes("DJI-DroneID", center, 0.0, bytes[4..].to_vec())
        .by(who)
        .with_detail(detail)
        .with_fields(fields)
        .with_modulation(common::Modulation::Ofdm)
        // A CRC-16 over the frame and a CRC-24 over the block it rode in.
        .with_crc(Some(true));
    if let (Some(lat), Some(lon)) = (f.latitude, f.longitude) {
        d = d.at_position(common::Position {
            lat,
            lon,
            altitude_m: Some(f.altitude_m),
            ..Default::default()
        });
    }
    Some(d)
}

/// DroneID as the auto node and the tables know it.
pub struct DroneId;

impl Protocol for DroneId {
    fn id(&self) -> &'static str {
        "droneid"
    }
    fn label(&self) -> &'static str {
        "droneid"
    }
    fn placement(&self) -> Placement {
        Placement::Channels(channels())
    }
    /// The front end puts its own tag in front of the frame, which is the
    /// most specific claim there is and the only one that tells a DroneID
    /// frame from the 91 bytes anything else might read on a 2.4 GHz centre.
    fn frame_claim(&self) -> FrameClaim {
        FrameClaim::Tagged
    }
    fn read_frame(&self, p: &common::Packet, bytes: &[u8]) -> Option<Vec<Decoded>> {
        droneid_decoded(bytes, common::Hz(p.center_hz())).map(|d| vec![d])
    }
    fn shape(&self) -> Shape {
        Shape {
            widths: &[WIDTH_HZ],
            min_rate_hz: RATE_HZ,
            feed_rate_hz: RATE_HZ,
            span_wide: true,
            families: &[],
        }
    }
    fn default_hz(&self) -> f64 {
        DEFAULT_HZ
    }
    /// A burst is 720 us roughly twice a second, so a decoder that owns its
    /// channel between bursts owns 10 MHz of a shared band for nothing. The
    /// band is shared: 2.4 GHz holds Wi-Fi, Bluetooth and every ISM device
    /// there is, and an aircraft transmits inside a Wi-Fi channel rather
    /// than instead of one.
    fn stickiness(&self) -> Stickiness {
        Stickiness::Forget
    }
    /// Nothing while the detector has found nothing at all, and everything
    /// once it has.
    ///
    /// A burst is 720 us, which is longer than a detector frame at any rate
    /// this front end runs at, so anything worth reading has been on the air
    /// long enough for a source to be open somewhere in the span; the front
    /// end is handed the lead-in it missed out of the span's ring. The gate
    /// is any source and not one 10 MHz wide, for the reason in
    /// [`crate::protocol::Wake`]: the detector's extent is every bin within
    /// 20 dB of a run's peak, which is far narrower than the transmission.
    /// The hold covers the gap between one aircraft's bursts, which is half
    /// a second.
    fn wakes_on(&self) -> crate::protocol::Wake {
        crate::protocol::Wake::Detected { hold_s: 1.0 }
    }
    fn stage_label(&self, hz: f64) -> String {
        format!("{:.1} DRONEID", hz / 1e6)
    }
    fn marks(&self, hz: f64) -> Vec<Mark> {
        vec![Mark { hz, width_hz: WIDTH_HZ, label: "DRONEID".into() }]
    }
    fn chain(&self, _at: Placed) -> Vec<NodeSpec> {
        vec![NodeSpec::new("droneid")]
    }
    /// An aircraft repeats the same frame twice a second and most of it does
    /// not change on the ground. One row per serial per sequence number, so a
    /// stationary drone is one row and a moving one is a row a burst.
    fn dedupe_key(&self, p: &common::Packet) -> Option<Vec<u8>> {
        let common::PacketBody::Frame(fr) = &p.body else {
            return None;
        };
        let f = decode::droneid::parse(fr.bytes.get(4..)?)?;
        let mut key = b"dji".to_vec();
        key.extend(f.serial.as_bytes());
        key.extend(f.sequence.to_le_bytes());
        Some(key)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A frame off air from a Mini 4K on the bench, with the CRC-16 the
    /// aircraft computed.
    fn off_air() -> Vec<u8> {
        let hex = "581002b801060f4638504a433235344a3030314a52345200000000000000000000\
                   00000000000000df08000000000000000000000000000000000000000000000000\
                   6b133139333537343334333039343231343635363000000000000000000000";
        (0..hex.len() / 2)
            .map(|i| u8::from_str_radix(&hex[i * 2..i * 2 + 2], 16).unwrap())
            .collect()
    }

    #[test]
    fn a_frame_becomes_a_row_named_for_the_airframe() {
        let mut frame = off_air();
        frame.resize(decode::droneid::FRAME_LEN - 2, 0);
        let crc = decode::droneid::crc16(&frame);
        frame.extend(crc.to_le_bytes());
        let d = droneid_decoded(&wrap(&frame), common::Hz(2_444_500_000)).expect("a row");
        assert_eq!(d.protocol, "DJI-DroneID");
        let detail = d.detail.as_deref().unwrap();
        assert!(detail.contains("serial=F8PJC254J001JR4R"), "{detail}");
        assert_eq!(d.crc_ok, Some(true));
    }

    #[test]
    fn bytes_that_are_not_a_frame_are_not_a_row() {
        assert!(droneid_decoded(&[0u8; 40], common::Hz(2_444_500_000)).is_none());
        // A frame with no tag in front of it did not come from here.
        assert!(droneid_decoded(&off_air(), common::Hz(2_444_500_000)).is_none());
        // And one whose CRC does not check is not reported at all.
        let mut bad = off_air();
        bad.resize(decode::droneid::FRAME_LEN, 0);
        assert!(droneid_decoded(&wrap(&bad), common::Hz(2_444_500_000)).is_none());
    }

    #[test]
    fn the_node_refuses_a_rate_too_low_for_the_frame() {
        let mut n = DroneIdNode::new();
        let spec = |rate: f64| PortSpec {
            spec: StreamSpec::iq(rate, common::Hz(2_444_500_000)),
            latency: 0,
        };
        assert!(n.negotiate(&spec(2_400_000.0)).is_err());
        assert!(n.negotiate(&spec(dsp::droneid::RATE)).is_ok());
        assert!(n.negotiate(&spec(20_000_000.0)).is_ok());
    }
}

pub const DESC: StageDesc = StageDesc {
    name: "droneid",
    summary: "DJI DroneID: the 15.36 MS/s OFDM burst an aircraft broadcasts about itself",
    category: Category::Decode,
    feeds_bus: true,
};

pub fn build(_s: &Settings) -> Result<Box<dyn pipeline::node::Node>> {
    Ok(Box::new(DroneIdNode::new()))
}
