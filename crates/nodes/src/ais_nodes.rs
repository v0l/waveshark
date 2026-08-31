//! AIS as a graph node.
//!
//! The wiring only, exactly like `modes_nodes`: the demodulator and its link
//! layer are `dsp::ais`, the message tables are `decode::ais`, and neither
//! knows about pipelines or about the other.
//!
//! Unlike Mode S this is one node because there is nothing to split, not
//! because splitting would lose frames. The frame check sequence has already
//! decided what is a frame by the time anything leaves the demodulator, so
//! what reaches the bus is bytes that proved themselves, and the parsing that
//! happens downstream is reading rather than acceptance.

use common::Result;
use decode::ais::{self, Message};
use dsp::ais::{AisConfig, AisDetector, AisFrame, BAND_CENTER_HZ, CHANNEL_HZ};
use pipeline::event::Decoded;
use pipeline::node::{NodeCtx, PortSpec, Simple};
use pipeline::port::{Payload, PortKind, StreamSpec};

/// The width one AIS channel occupies, which is what a frame was heard
/// through whichever of the two carried it.
pub const CHANNEL_WIDTH_HZ: f64 = 25_000.0;

pub struct AisNode {
    cfg: AisConfig,
    det: AisDetector,
    frames: Vec<AisFrame>,
    accepted: u64,
}

impl Default for AisNode {
    fn default() -> Self {
        Self::new(AisConfig::default())
    }
}

impl AisNode {
    pub fn new(cfg: AisConfig) -> Self {
        Self {
            cfg,
            // Replaced at negotiation, when the real rate and centre are known.
            det: AisDetector::new(2_400_000.0, BAND_CENTER_HZ, cfg),
            frames: Vec::new(),
            accepted: 0,
        }
    }

    /// Frames that passed their check sequence since the node was built.
    pub fn accepted(&self) -> u64 {
        self.accepted
    }
}

impl Simple for AisNode {
    fn name(&self) -> &str {
        "ais"
    }

    fn negotiate(&mut self, i: &PortSpec) -> Result<StreamSpec> {
        if i.spec.kind != PortKind::Iq {
            return Err(common::Error::other("ais reads complex baseband"));
        }
        let (rate, center) = (i.spec.rate, i.spec.center.as_f64());
        // Both channels have to be inside the span with their bandwidth to
        // spare. A channel sitting on the edge is a channel being demodulated
        // through the anti-alias filter's skirt, which reads as silence.
        let edge = rate / 2.0 - CHANNEL_WIDTH_HZ;
        if CHANNEL_HZ.iter().any(|c| (c - center).abs() > edge) {
            return Err(common::Error::other(
                "ais needs both 161.975 and 162.025 MHz inside the span",
            ));
        }
        self.det = AisDetector::new(rate, center, self.cfg);
        // Frames rather than bytes, for the same reason Mode S says so: two
        // messages written into one buffer cannot be told apart afterwards.
        //
        // The centre reported is the band rather than the channel a frame
        // arrived on. Which of the two carried it is the demodulator's own
        // knowledge, like the level it measured, and the bus carries evidence
        // a log can hold per frame rather than what the front end happened to
        // know while producing it.
        let mut out = i.spec.with_kind(PortKind::Frames);
        out.center = common::Hz(BAND_CENTER_HZ as u64);
        out.bandwidth = CHANNEL_WIDTH_HZ;
        Ok(out)
    }

    fn process(&mut self, i: &Payload, o: &mut Payload, _c: &mut NodeCtx<'_>) -> Result<()> {
        let Some(iq) = i.as_iq() else { return Ok(()) };
        self.frames.clear();
        self.det.process(iq, &mut self.frames);
        let out = o.frames_mut();
        for f in &self.frames {
            self.accepted += 1;
            out.push(f.payload.clone());
        }
        Ok(())
    }

    fn reset(&mut self) {
        self.det.reset();
    }
}

/// The decode an AIS payload becomes.
///
/// Takes the bytes rather than a parsed message for the same reason the Mode S
/// one does: what travels on the bus is the payload, and a consumer draws its
/// own conclusions from it.
pub fn ais_decoded(frame: &ais::Frame, bytes: &[u8], center: common::Hz) -> Decoded {
    use common::Value;
    let mut fields: Vec<(String, Value)> = Vec::new();
    // The identity every message carries, and the field that turns a stream of
    // them into tracks.
    fields.push(("mmsi".into(), Value::Int(i64::from(frame.mmsi))));

    let protocol = match &frame.kind {
        Message::Position(p) => {
            if let Some((lat, lon)) = p.position {
                fields.push(("lat".into(), Value::Float(round(lat, 5))));
                fields.push(("lon".into(), Value::Float(round(lon, 5))));
            }
            if let Some(v) = p.sog_kt {
                fields.push(("ground_speed_kt".into(), Value::Float(v)));
            }
            if let Some(v) = p.cog_deg {
                fields.push(("track_deg".into(), Value::Float(v)));
            }
            if let Some(v) = p.heading_deg {
                fields.push(("heading_deg".into(), Value::Float(v)));
            }
            if let Some(v) = p.nav_status {
                fields.push(("nav_status".into(), Value::Text(ais::nav_status_name(v).into())));
            }
            if p.class_b {
                "AIS-PositionB"
            } else {
                "AIS-Position"
            }
        }
        Message::Static(s) => {
            if let Some(n) = &s.name {
                fields.push(("name".into(), Value::Text(n.clone())));
            }
            if let Some(c) = &s.callsign {
                fields.push(("callsign".into(), Value::Text(c.clone())));
            }
            if let Some(t) = s.ship_type {
                fields.push(("ship_type".into(), Value::Text(ais::ship_type_name(t).into())));
            }
            if let Some(d) = &s.destination {
                fields.push(("destination".into(), Value::Text(d.clone())));
            }
            if let Some(d) = s.draught_m {
                fields.push(("draught_m".into(), Value::Float(d)));
            }
            "AIS-Static"
        }
        Message::BaseStation { position, .. } => {
            if let Some((lat, lon)) = position {
                fields.push(("lat".into(), Value::Float(round(*lat, 5))));
                fields.push(("lon".into(), Value::Float(round(*lon, 5))));
            }
            "AIS-BaseStation"
        }
        Message::AidToNavigation { name, position, .. } => {
            if let Some(n) = name {
                fields.push(("name".into(), Value::Text(n.clone())));
            }
            if let Some((lat, lon)) = position {
                fields.push(("lat".into(), Value::Float(round(*lat, 5))));
                fields.push(("lon".into(), Value::Float(round(*lon, 5))));
            }
            "AIS-AidToNav"
        }
        Message::Unsupported { msg_type } => {
            fields.push(("msg_type".into(), Value::Int(i64::from(*msg_type))));
            "AIS-Other"
        }
    };

    let detail = fields.iter().map(|(k, v)| format!("{k}={v}")).collect::<Vec<_>>().join(" ");
    Decoded::bytes(protocol, center, 0.0, bytes.to_vec())
        .with_detail(detail)
        .with_fields(fields)
        .with_modulation("GMSK")
        // Every frame that reaches here passed the X.25 frame check sequence
        // in the demodulator, which is a real integrity check and not a
        // plausibility argument.
        .with_crc(Some(true))
}

fn round(v: f64, places: i32) -> f64 {
    let f = 10f64.powi(places);
    (v * f).round() / f
}

#[cfg(test)]
mod tests {
    use super::*;
    use common::Hz;

    fn spec(rate: f64, center: f64) -> PortSpec {
        PortSpec { spec: StreamSpec::iq(rate, Hz(center as u64)), latency: 0 }
    }

    #[test]
    fn the_node_refuses_a_span_that_does_not_hold_both_channels() {
        let mut n = AisNode::default();
        // Tuned to the band with room to spare.
        assert!(n.negotiate(&spec(2_400_000.0, 162_000_000.0)).is_ok());
        // Tuned elsewhere in marine VHF: the channels are not in the span.
        assert!(n.negotiate(&spec(2_400_000.0, 157_000_000.0)).is_err());
        // On the band but too narrow to hold both channels.
        assert!(n.negotiate(&spec(48_000.0, 162_000_000.0)).is_err());
    }

    #[test]
    fn the_node_outputs_frames_tagged_with_the_band() {
        let mut n = AisNode::default();
        let out = n.negotiate(&spec(2_400_000.0, 162_000_000.0)).unwrap();
        assert_eq!(out.kind, PortKind::Frames);
        assert_eq!(out.center, Hz(BAND_CENTER_HZ as u64));
        assert_eq!(out.bandwidth, CHANNEL_WIDTH_HZ);
    }

    /// A position report becomes a row with the fields a map reads.
    #[test]
    fn a_position_becomes_a_decode_with_a_position_in_it() {
        use common::Value;
        // The Le Havre report, the payload both other crates are tested on.
        let bytes = vec![
            0x04, 0x36, 0x1f, 0x64, 0xa0, 0x20, 0x00, 0x00, 0x00, 0x99, 0xf6, 0x1c, 0x4f, 0x66,
            0x21, 0x6f, 0xff, 0x9c, 0x00, 0x56, 0x78,
        ];
        let frame = ais::parse(&bytes).unwrap();
        let d = ais_decoded(&frame, &bytes, Hz(BAND_CENTER_HZ as u64));
        assert_eq!(d.protocol, "AIS-Position");
        assert_eq!(d.crc_ok, Some(true), "it passed the check sequence to get here");
        let get = |k: &str| d.fields.iter().find(|(n, _)| n == k).map(|(_, v)| v.clone());
        assert_eq!(get("mmsi"), Some(Value::Int(227_006_760)));
        assert_eq!(get("lat"), Some(Value::Float(49.47558)));
        assert_eq!(get("lon"), Some(Value::Float(0.13138)));
    }

    /// Modulate on-air symbols as FSK at the AIS rate and deviation.
    ///
    /// Plain FSK rather than GMSK: the discriminator does not care about the
    /// pulse shaping, and a test that had to implement a Gaussian filter to
    /// check the wiring would be testing the wrong thing.
    fn modulate(levels: &[bool], rate: f64, offset_hz: f64) -> Vec<common::C32> {
        let sps = (rate / dsp::ais::BAUD) as usize;
        let mut out = Vec::with_capacity(levels.len() * sps);
        let mut phase = 0.0f64;
        for &l in levels {
            let f = offset_hz + if l { 2_400.0 } else { -2_400.0 };
            for _ in 0..sps {
                phase += std::f64::consts::TAU * f / rate;
                out.push(common::C32::new(phase.cos() as f32, phase.sin() as f32));
            }
        }
        out
    }

    /// The whole path on synthetic RF: modulated frame into the node, vessel
    /// out of the message tables.
    ///
    /// The point is that the three layers agree about bit order. Each is
    /// tested alone and each could be self-consistently wrong; only running
    /// them together shows that the payload the demodulator packs is the one
    /// the message tables read. This is the test that would fail if somebody
    /// "fixed" the bit order in either place.
    #[test]
    fn a_modulated_frame_becomes_a_vessel_at_the_right_place() {
        let payload: Vec<u8> = vec![
            0x04, 0x36, 0x1f, 0x64, 0xa0, 0x20, 0x00, 0x00, 0x00, 0x99, 0xf6, 0x1c, 0x4f, 0x66,
            0x21, 0x6f, 0xff, 0x9c, 0x00, 0x56, 0x78,
        ];
        let (rate, center) = (2_400_000.0, 162_000_000.0);
        let iq = modulate(&dsp::ais::encode_slot(&payload, 168), rate, CHANNEL_HZ[0] - center);

        let mut node = AisNode::default();
        node.negotiate(&spec(rate, center)).unwrap();

        let ins = [spec(rate, center)];
        let tags = Vec::new();
        let mut frames: Vec<Vec<u8>> = Vec::new();
        let quiet = vec![common::C32::new(0.0, 0.0); 4096];
        // Silence after the burst flushes the decimator, without which the
        // closing flag is still inside the filter when the block ends.
        for block in [&quiet[..], &iq[..], &quiet[..]] {
            let input = Payload::Iq(block.to_vec());
            let mut out = Payload::Frames(Vec::new());
            let mut events = Vec::new();
            let mut new_tags = Vec::new();
            let mut ctx = NodeCtx::new(0, &ins, &tags, &mut events, &mut new_tags);
            node.process(&input, &mut out, &mut ctx).unwrap();
            if let Payload::Frames(f) = out {
                frames.extend(f);
            }
        }

        assert_eq!(frames.len(), 1, "expected one frame off the air");
        let parsed = ais::parse(&frames[0]).expect("a message");
        assert_eq!(parsed.mmsi, 227_006_760);
        let Message::Position(p) = parsed.kind.clone() else { panic!("{parsed:?}") };
        let (lat, lon) = p.position.expect("a fix");
        assert!((lat - 49.475_576).abs() < 1e-5, "latitude {lat}");
        assert!((lon - 0.131_38).abs() < 1e-5, "longitude {lon}");

        // And the row the packet list would show for it.
        let d = ais_decoded(&parsed, &frames[0], Hz(BAND_CENTER_HZ as u64));
        assert_eq!(d.protocol, "AIS-Position");
        assert_eq!(d.crc_ok, Some(true));
    }
}
