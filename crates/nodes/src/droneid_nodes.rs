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
//! # What a row is evidence of
//!
//! A frame that reaches the bus passed a CRC-16 over 89 bytes this project
//! did not construct, inside a code block whose own CRC-24 also checks. What
//! the fields say is still the aircraft's claim, and an aircraft with no fix
//! sends zeros, which `decode::droneid` reports as absent rather than as a
//! position off Africa.

use crate::frame_meter::FrameMeter;
use crate::protocol::{Mark, Placed, Placement, Protocol, Shape, Stickiness};
use crate::NodeSpec;
use common::Result;
use pipeline::event::Decoded;
use pipeline::node::{NodeCtx, PortSpec, Simple};
use pipeline::port::{Payload, PortKind, StreamSpec};
use pipeline::registry::{Category, Settings, StageDesc};

/// The occupied bandwidth: 600 carriers 15 kHz apart, plus guards.
pub const WIDTH_HZ: f64 = 10_000_000.0;

/// The rate the frame is defined at, which is also the rate this asks for.
pub const RATE_HZ: f64 = dsp::droneid::RATE;

/// The centre a receiver picks when it has to pick one: the middle of the
/// 2.4 GHz set, where the bursts in the bench captures were.
pub const DEFAULT_HZ: f64 = 2_444_500_000.0;

/// How strongly the Zadoff-Chu symbol has to correlate. Off air, a burst
/// scores 0.88 to 0.99 and nothing else in a busy 2.4 GHz band comes near
/// half of that, so this is not a knife edge.
const THRESHOLD: f32 = 0.5;

/// Samples kept behind the receiver, so a frame carries the samples it was
/// read from.
///
/// A burst is nine symbols, about 720 us, but the ring has to outlast a whole
/// block as well: measured with 4 ms of ring and 64k sample blocks, every
/// frame fell out of the ring before it was measured and reported the block's
/// level instead of its own. Twenty milliseconds is four blocks at this rate.
const KEEP_S: f64 = 0.02;

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
    meter: Option<FrameMeter>,
    carry: Vec<common::C32>,
    at: usize,
    rate: f64,
    /// Where the last burst reported began, so the one straddling a block
    /// boundary is not reported twice: the tail of every block is handed to
    /// the next one, and a burst inside it correlates in both.
    last: Option<usize>,
}

impl Default for DroneIdNode {
    fn default() -> Self {
        Self::new()
    }
}

impl DroneIdNode {
    pub fn new() -> Self {
        Self {
            meter: None,
            carry: Vec::new(),
            at: 0,
            rate: RATE_HZ,
            last: None,
        }
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
        let rate = input.spec.rate;
        if rate + 1.0 < RATE_HZ {
            return Err(common::Error::other(
                "droneid needs 15.36 MS/s: the frame is 600 carriers 15 kHz apart",
            ));
        }
        self.rate = rate;
        self.meter = Some(FrameMeter::new(rate, input.spec.center.0, KEEP_S));
        let mut out = input.spec.with_kind(PortKind::Frames);
        out.bandwidth = WIDTH_HZ;
        Ok(out)
    }

    fn process(&mut self, input: &Payload, output: &mut Payload, ctx: &mut NodeCtx) -> Result<()> {
        let Payload::Iq(iq) = input else {
            return Ok(());
        };
        let out = output.frames_mut();
        let Some(meter) = self.meter.as_mut() else {
            return Ok(());
        };
        meter.feed(iq);
        let _ = ctx;

        // A burst can straddle a block, so the tail of the previous one is
        // carried: nine symbols is 9384 samples at this rate.
        let carried = self.carry.len();
        let mut window = std::mem::take(&mut self.carry);
        window.extend_from_slice(iq);
        for found in dsp::droneid::find_bursts(&window, self.rate, THRESHOLD) {
            let Some(start) = found
                .zc4_at
                .checked_sub(dsp::droneid::symbol_offset(self.rate, 4))
            else {
                continue;
            };
            let absolute_start = self.at + start;
            if self
                .last
                .is_some_and(|l| absolute_start.saturating_sub(l) < dsp::droneid::burst_len(self.rate))
            {
                continue;
            }
            let Some(frame) = dsp::droneid::frame_bits(&window, start, self.rate) else {
                continue;
            };
            if decode::droneid::parse(&frame).is_none() {
                continue;
            }
            // The whole burst is the frame's own samples, and its level is
            // measured over exactly those: a 10 MHz channel that is quiet
            // between bursts would otherwise report the noise it sat in.
            self.last = Some(absolute_start);
            let absolute = absolute_start as u64;
            let len = dsp::droneid::burst_len(self.rate);
            // The floor next to the burst rather than the meter's own,
            // which on a channel a link keeps busy is the link. A burst
            // length of quiet a burst length before it is what a detector
            // would have measured against.
            let signal = meter.power_dbfs_at(absolute, len);
            let noise = absolute
                .checked_sub(2 * len as u64)
                .and_then(|at| meter.power_dbfs_at(at, len));
            let snr = match (signal, noise) {
                (Some(s), Some(n)) => s - n,
                // Nothing to measure against is a level of nothing, not a
                // level of zero: the correlation is all that is left.
                _ => 20.0 * found.score.max(1e-6).log10(),
            };
            out.push(meter.frame_measured(wrap(&frame), absolute, len, snr));
        }
        // What the next call will see as window[0]: everything but the tail
        // kept for a burst that straddles the boundary. Advancing by the
        // block rather than by the window loses the carried prefix, and the
        // frames then name samples the meter no longer holds, so every one
        // of them reports the block's level instead of its own.
        let keep = window.len().min(dsp::droneid::burst_len(self.rate) * 2);
        self.at += window.len() - keep;
        self.carry = window[window.len() - keep..].to_vec();
        let _ = carried;
        Ok(())
    }

    fn reset(&mut self) {
        if let Some(m) = self.meter.as_mut() {
            m.reset();
        }
        self.carry.clear();
        self.at = 0;
        self.last = None;
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
    let detail = fields
        .iter()
        .map(|(k, v)| format!("{k}={v}"))
        .collect::<Vec<_>>()
        .join(" ");

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
    fn shape(&self) -> Shape {
        Shape {
            widths: &[WIDTH_HZ],
            min_rate_hz: RATE_HZ,
            feed_rate_hz: RATE_HZ,
            span_wide: false,
            families: &[],
        }
    }
    fn default_hz(&self) -> f64 {
        DEFAULT_HZ
    }
    /// A burst is 720 us roughly twice a second, so a decoder that owns its
    /// channel between bursts owns 10 MHz of a shared band for nothing.
    fn stickiness(&self) -> Stickiness {
        Stickiness::Forget
    }
    fn stage_label(&self, hz: f64) -> String {
        format!("{:.1} DRONEID", hz / 1e6)
    }
    fn marks(&self, hz: f64) -> Vec<Mark> {
        vec![Mark {
            hz,
            width_hz: WIDTH_HZ,
            label: "DRONEID".into(),
        }]
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
