//! Mode S and ADS-B as a graph node.
//!
//! Everything else in this crate splits the work across two nodes: a front end
//! that turns samples into structured bursts, and a protocol node that reads
//! them. Mode S is one node instead, and the reason is worth recording,
//! because it looks like a shortcut and is not.
//!
//! The demodulator searches for preambles, and a frame it believes blanks the
//! 120 us it occupies, since nothing inside a frame can be the start of
//! another one. A false preamble therefore destroys every real frame
//! overlapping it. The only thing that can tell a false preamble from a real
//! one is the CRC, so the acceptance test has to run *inside* the search
//! rather than downstream of it. Split across two nodes, the front end would
//! blank on candidates the protocol node later rejects: measured on a
//! recorded band, that is 8 frames recovered instead of 27.
//!
//! What stays modular is the parts. The demodulator is `dsp::modes`, the frame
//! format is `decode::adsb`, and neither knows about the other or about
//! pipelines. This node is the wiring, exactly as `PulseDetectNode` is the
//! wiring around `dsp::OokDetector`.

use common::Result;
use decode::adsb::{self, AddressBook, Message};
use dsp::{ModeSConfig, ModeSDetector, ModeSFrame};
use pipeline::event::{Decoded, Event};
use pipeline::node::{NodeCtx, PortSpec, Simple};
use pipeline::param::{Param, ParamValue};
use pipeline::port::{Payload, PortKind, StreamSpec};

pub struct ModeSNode {
    cfg: ModeSConfig,
    det: ModeSDetector,
    book: AddressBook,
    frames: Vec<ModeSFrame>,
    /// What decoded in the last block. Events also go to the graph; this is
    /// the typed view, for a host that wants this node's decodes rather than
    /// everything the graph produced.
    hits: Vec<Event>,
    /// Frames accepted since the node was built.
    accepted: u64,
}

impl Default for ModeSNode {
    fn default() -> Self {
        Self::new(ModeSConfig::default())
    }
}

impl ModeSNode {
    /// What decoded in the last block.
    pub fn hits(&self) -> &[Event] {
        &self.hits
    }

    pub fn new(cfg: ModeSConfig) -> Self {
        Self {
            cfg,
            // Replaced at negotiation, when the real sample rate is known.
            det: ModeSDetector::new(2_400_000.0, cfg),
            hits: Vec::new(),
            book: AddressBook::new(),
            frames: Vec::new(),
            accepted: 0,
        }
    }

    /// Aircraft whose address has proved itself.
    pub fn aircraft(&self) -> usize {
        self.book.len()
    }

    pub fn accepted(&self) -> u64 {
        self.accepted
    }
}

impl Simple for ModeSNode {
    fn name(&self) -> &str {
        "mode_s"
    }

    fn negotiate(&mut self, i: &PortSpec) -> Result<StreamSpec> {
        if i.spec.kind != PortKind::Iq {
            return Err(common::Error::other("mode_s reads complex baseband"));
        }
        let rate = i.spec.rate;
        if rate < 2_000_000.0 {
            return Err(common::Error::other(
                "mode_s needs 2 MS/s or more: its bits are 1 us wide",
            ));
        }
        self.det = ModeSDetector::new(rate, self.cfg);
        Ok(i.spec.with_kind(PortKind::Bytes))
    }

    fn params(&self) -> Vec<Param> {
        vec![
            Param::float("preamble_ratio", self.cfg.preamble_ratio as f64, 1.0..=8.0)
                .label("Preamble above the quiet slots"),
            Param::float("min_level", self.cfg.min_level as f64, 0.0001..=0.5)
                .label("Preamble amplitude floor")
                .log(),
        ]
    }

    fn set_param(&mut self, name: &str, v: ParamValue) -> Result<()> {
        let f = v.as_f64().unwrap_or_default();
        match name {
            "preamble_ratio" => self.cfg.preamble_ratio = f.max(1.0) as f32,
            "min_level" => self.cfg.min_level = f.max(0.0) as f32,
            _ => {
                return Err(common::Error::other(format!("mode_s: unknown parameter {name:?}")))
            }
        }
        // The detector holds its config by value, and its buffered tail is
        // one frame long, so rebuilding it costs nothing.
        self.det = ModeSDetector::new(self.det.rate(), self.cfg);
        Ok(())
    }

    fn process(&mut self, i: &Payload, o: &mut Payload, c: &mut NodeCtx<'_>) -> Result<()> {
        let Some(iq) = i.as_iq() else { return Ok(()) };
        self.frames.clear();
        self.hits.clear();
        let book = std::cell::RefCell::new(std::mem::take(&mut self.book));
        self.det.process_valid(iq, &mut self.frames, &|f: &ModeSFrame| {
            book.borrow_mut().accept(&f.bytes, f.weak_bits == 0)
        });
        self.book = book.into_inner();

        let center = c.inputs[0].spec.center;
        let out = o.bytes_mut();
        for f in &self.frames {
            // Correcting a flipped bit is arithmetic on the frame, so it
            // happens here rather than in the demodulator.
            let bytes = match f.bytes[0] >> 3 {
                17 | 18 => adsb::fix_single_bit(&f.bytes).unwrap_or_else(|| f.bytes.clone()),
                _ => f.bytes.clone(),
            };
            let Ok(frame) = adsb::parse(&bytes) else { continue };
            self.accepted += 1;
            out.extend_from_slice(&bytes);
            let d = Event::Decoded(decoded(&frame, f, center, &bytes));
            self.hits.push(d.clone());
            c.emit(d);
        }
        Ok(())
    }
}

fn decoded(
    frame: &adsb::Frame,
    raw: &ModeSFrame,
    center: common::Hz,
    bytes: &[u8],
) -> Decoded {
    use common::Value;
    let mut fields: Vec<(String, Value)> = Vec::new();
    if let Some(icao) = frame.icao {
        fields.push(("icao".into(), Value::Text(format!("{icao:06x}"))));
    }
    let protocol = match &frame.kind {
        Message::Identification { callsign, category } => {
            fields.push(("callsign".into(), Value::Text(callsign.clone())));
            fields.push(("category".into(), Value::Int(*category as i64)));
            "ADSB-Identification"
        }
        Message::AirbornePosition { altitude_ft, odd, lat_cpr, lon_cpr } => {
            if let Some(alt) = altitude_ft {
                fields.push(("altitude_ft".into(), Value::Int(*alt as i64)));
            }
            // The encoded halves are reported as they arrive. Turning a pair
            // of them into a latitude needs state across frames, which is a
            // tracker's job rather than a decoder's.
            fields.push(("cpr_odd".into(), Value::Bool(*odd)));
            fields.push(("lat_cpr".into(), Value::Int(*lat_cpr as i64)));
            fields.push(("lon_cpr".into(), Value::Int(*lon_cpr as i64)));
            "ADSB-Position"
        }
        Message::SurfacePosition { odd, lat_cpr, lon_cpr } => {
            fields.push(("cpr_odd".into(), Value::Bool(*odd)));
            fields.push(("lat_cpr".into(), Value::Int(*lat_cpr as i64)));
            fields.push(("lon_cpr".into(), Value::Int(*lon_cpr as i64)));
            "ADSB-Surface"
        }
        Message::Velocity { ground_speed_kt, track_deg, vertical_rate_fpm } => {
            fields.push(("ground_speed_kt".into(), Value::Float(round1(*ground_speed_kt))));
            fields.push(("track_deg".into(), Value::Float(round1(*track_deg))));
            fields.push(("vertical_rate_fpm".into(), Value::Int(*vertical_rate_fpm as i64)));
            "ADSB-Velocity"
        }
        Message::Unsupported { type_code } => {
            fields.push(("type_code".into(), Value::Int(*type_code as i64)));
            "ADSB-Other"
        }
        // A reply to an interrogation, which is most of what a busy sky
        // sounds like. Worth reporting: it says an aircraft is up there, and
        // its address is the only identity it gives.
        Message::ShortReply => "ModeS-Reply",
    };
    let detail =
        fields.iter().map(|(k, v)| format!("{k}={v}")).collect::<Vec<_>>().join(" ");
    Decoded::bytes(protocol, center, raw.at_sample as f64, bytes.to_vec())
        .with_detail(detail)
        .with_fields(fields)
        .with_modulation("PPM")
        // Mode S has no per-frame noise estimate the way a gated burst does.
        .with_level(raw.rssi_dbfs, f32::NAN)
        // Only the extended squitters carry a CRC of their own. A short reply
        // is believed because its address is one an ADS-B frame proved, which
        // is corroboration rather than an integrity check.
        .with_crc(matches!(frame.df, 17 | 18).then_some(true))
}

fn round1(v: f64) -> f64 {
    (v * 10.0).round() / 10.0
}

#[cfg(test)]
mod tests {
    use super::*;
    use common::Hz;

    fn spec(rate: f64) -> PortSpec {
        PortSpec { spec: StreamSpec::iq(rate, Hz(1_090_000_000)), latency: 0 }
    }

    #[test]
    fn the_node_refuses_a_rate_its_bits_cannot_survive() {
        // Better to refuse the graph than to build one that reports noise.
        let mut n = ModeSNode::default();
        assert!(n.negotiate(&spec(1_024_000.0)).is_err());
        assert!(n.negotiate(&spec(2_400_000.0)).is_ok());
    }

    #[test]
    fn the_node_outputs_bytes() {
        let mut n = ModeSNode::default();
        let out = n.negotiate(&spec(2_400_000.0)).unwrap();
        assert_eq!(out.kind, PortKind::Bytes);
    }

    #[test]
    fn a_position_frame_becomes_a_decoded_event_with_fields() {
        use common::Value;
        let bytes = hex("8d40621d58c382d690c8ac2863a7");
        let frame = adsb::parse(&bytes).unwrap();
        let raw = ModeSFrame { bytes: bytes.clone(), at_sample: 7, rssi_dbfs: -12.0, weak_bits: 0 };
        let d = decoded(&frame, &raw, Hz(1_090_000_000), &bytes);
        assert_eq!(d.protocol, "ADSB-Position");
        assert_eq!(d.crc_ok, Some(true));
        let get = |k: &str| d.fields.iter().find(|(n, _)| n == k).map(|(_, v)| v.clone());
        assert_eq!(get("icao"), Some(Value::Text("40621d".into())));
        assert_eq!(get("altitude_ft"), Some(Value::Int(38_000)));
    }

    #[test]
    fn a_short_reply_claims_no_integrity_check() {
        let bytes = hex("02e19838adb7c4");
        let frame = adsb::parse(&bytes).unwrap();
        let raw = ModeSFrame { bytes: bytes.clone(), at_sample: 0, rssi_dbfs: -20.0, weak_bits: 0 };
        let d = decoded(&frame, &raw, Hz(1_090_000_000), &bytes);
        assert_eq!(d.protocol, "ModeS-Reply");
        assert_eq!(d.crc_ok, None, "a reply's parity is an address, not a check");
    }

    fn hex(s: &str) -> Vec<u8> {
        (0..s.len() / 2).map(|i| u8::from_str_radix(&s[i * 2..i * 2 + 2], 16).unwrap()).collect()
    }
}
