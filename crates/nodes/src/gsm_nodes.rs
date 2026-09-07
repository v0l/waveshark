//! A GSM carrier as a graph node.
//!
//! The wiring only. The tone search, the burst demodulator and the channel
//! coding are `dsp::gsm`, and the row a decode becomes is at the bottom of
//! this file; neither knows about pipelines.
//!
//! One node watches one carrier, given as an offset from the span's centre,
//! because that is what a GSM beacon is: cells are 200 kHz apart and a
//! receiver reads one of them at a time. Watching a whole band means placing
//! one of these per carrier, which is the scanner table's job and not a loop
//! hidden in here.

use common::Result;
use dsp::gsm::{self, sch, GsmConfig, SchDetector, SchHit};
use pipeline::event::Decoded;
use pipeline::node::{NodeCtx, PortSpec, Simple};
use pipeline::param::{Param, ParamValue};
use pipeline::port::{Payload, PortKind, StreamSpec};

/// What one carrier occupies, and the width a burst was heard through.
pub const CHANNEL_WIDTH_HZ: f64 = gsm::CHANNEL_SPACING_HZ;

pub struct GsmNode {
    cfg: GsmConfig,
    /// Which carrier to watch, as an offset from the centre of the span.
    offset_hz: f64,
    channel_hz: f64,
    det: SchDetector,
    meter: crate::FrameMeter,
    hits: Vec<SchHit>,
    accepted: u64,
}

impl Default for GsmNode {
    fn default() -> Self {
        Self::new(0.0, GsmConfig::default())
    }
}

impl GsmNode {
    pub fn new(offset_hz: f64, cfg: GsmConfig) -> Self {
        // Replaced at negotiation, when the real rate and centre are known.
        let rate = 2_400_000.0;
        Self {
            cfg,
            offset_hz,
            channel_hz: 0.0,
            det: SchDetector::new(rate, 0.0, 0.0, cfg),
            meter: crate::FrameMeter::new(rate, 0, 0.25),
            hits: Vec::new(),
            accepted: 0,
        }
    }

    /// Synchronisation bursts whose parity held since the node was built.
    pub fn accepted(&self) -> u64 {
        self.accepted
    }
}

impl Simple for GsmNode {
    fn name(&self) -> &str {
        "gsm"
    }

    fn negotiate(&mut self, i: &PortSpec) -> Result<StreamSpec> {
        if i.spec.kind != PortKind::Iq {
            return Err(common::Error::other("gsm reads complex baseband"));
        }
        let (rate, center) = (i.spec.rate, i.spec.center.as_f64());
        if !SchDetector::rate_is_enough(rate) {
            return Err(common::Error::other(
                "gsm needs at least three samples a symbol, so 813 kS/s",
            ));
        }
        // The carrier and its skirts, not just its centre: a channel sitting
        // on the edge of the span is one being read through the anti-alias
        // filter, which is a channel that decodes nothing.
        if self.offset_hz.abs() > rate / 2.0 - CHANNEL_WIDTH_HZ / 2.0 {
            return Err(common::Error::other("gsm: the carrier is outside the span"));
        }
        self.channel_hz = center + self.offset_hz;
        self.det = SchDetector::new(rate, center, self.channel_hz, self.cfg);
        self.meter =
            crate::FrameMeter::new(self.det.channel_rate(), self.channel_hz as u64, 0.25);

        // Frames rather than bytes: two bursts written into one buffer cannot
        // be told apart afterwards.
        let mut out = i.spec.with_kind(PortKind::Frames);
        out.center = common::Hz(self.channel_hz as u64);
        out.bandwidth = CHANNEL_WIDTH_HZ;
        Ok(out)
    }

    fn process(&mut self, i: &Payload, o: &mut Payload, _c: &mut NodeCtx<'_>) -> Result<()> {
        let Some(iq) = i.as_iq() else { return Ok(()) };
        self.hits.clear();
        self.det.process(iq, &mut self.hits);
        // The channel this cut out, not the span it came from: what a burst
        // was heard at is the level of one carrier.
        self.meter.feed(self.det.channel());
        let out = o.frames_mut();
        for hit in &self.hits {
            let Some(bytes) = sch::pack(&hit.sch) else { continue };
            self.accepted += 1;
            out.push(self.meter.frame(bytes.to_vec()).at(self.channel_hz as u64));
        }
        Ok(())
    }

    fn reset(&mut self) {
        self.det.reset();
        self.meter.reset();
    }

    fn params(&self) -> Vec<Param> {
        vec![Param::float("offset_hz", self.offset_hz, -30e6..=30e6)
            .unit("Hz")
            .label("Carrier offset from the span centre")]
    }

    fn set_param(&mut self, name: &str, v: ParamValue) -> Result<()> {
        match name {
            "offset_hz" => self.offset_hz = v.as_f64().unwrap_or(0.0),
            _ => return Err(common::Error::other(format!("gsm: unknown parameter {name:?}"))),
        }
        Ok(())
    }
}

/// The row a synchronisation burst becomes.
///
/// Takes the bytes off the bus rather than a parsed burst, for the same
/// reason the AIS one does: what travelled is the information field, and a
/// consumer reads it for itself.
pub fn gsm_decoded(bytes: &[u8], center: common::Hz) -> Option<Decoded> {
    use common::Value;
    let sch = sch::unpack(bytes)?;
    let mut fields: Vec<(String, Value)> = vec![
        ("bsic".into(), Value::Int(i64::from(sch.bsic()))),
        ("ncc".into(), Value::Int(i64::from(sch.ncc))),
        ("bcc".into(), Value::Int(i64::from(sch.bcc))),
        ("frame".into(), Value::Int(i64::from(sch.frame_number))),
    ];
    let arfcn = gsm::arfcn(center.as_f64());
    if let Some(n) = arfcn {
        fields.push(("arfcn".into(), Value::Int(i64::from(n))));
    }

    // The cell, as it is written down: the colour code as two octal digits,
    // which is how a base station is configured and how a survey names it.
    let cell = match arfcn {
        Some(n) => format!("ARFCN {n} BSIC {}{}", sch.ncc, sch.bcc),
        None => format!("BSIC {}{}", sch.ncc, sch.bcc),
    };
    let detail = format!("{cell} frame {}", sch.frame_number);
    Some(
        Decoded::bytes("GSM-SCH", center, 0.0, bytes.to_vec())
            .with_link(pipeline::event::Link::beacon(pipeline::event::Party::unit(cell)))
            .with_detail(detail)
            .with_fields(fields)
            .with_modulation("GMSK")
            // The ten parity bits held in the demodulator, which is a real
            // check and the only reason this burst exists rather than a
            // Viterbi decoder's best guess at noise.
            .with_crc(Some(true)),
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use common::{Hz, C32};
    use dsp::gsm::Sch;

    fn spec(rate: f64, center: f64) -> PortSpec {
        PortSpec { spec: StreamSpec::iq(rate, Hz(center as u64)), latency: 0 }
    }

    #[test]
    fn the_node_refuses_a_span_it_cannot_read() {
        let mut n = GsmNode::default();
        assert!(n.negotiate(&spec(2_400_000.0, 947_400_000.0)).is_ok());
        // Under three samples a symbol there is nothing to interpolate.
        assert!(n.negotiate(&spec(400_000.0, 947_400_000.0)).is_err());
        // A carrier outside the span is not a carrier.
        n.set_param("offset_hz", ParamValue::Float(1_400_000.0)).unwrap();
        assert!(n.negotiate(&spec(2_400_000.0, 947_400_000.0)).is_err());
    }

    #[test]
    fn the_node_outputs_frames_tagged_with_the_carrier() {
        let mut n = GsmNode::new(600_000.0, GsmConfig::default());
        let out = n.negotiate(&spec(2_400_000.0, 947_400_000.0)).unwrap();
        assert_eq!(out.kind, PortKind::Frames);
        assert_eq!(out.center, Hz(948_000_000));
        assert_eq!(out.bandwidth, CHANNEL_WIDTH_HZ);
    }

    /// A cell becomes a row saying which cell it is.
    #[test]
    fn a_burst_becomes_a_row_naming_the_cell() {
        use common::Value;
        let sch = Sch { ncc: 5, bcc: 3, frame_number: 51 * 26 * 42 + 21 };
        let bytes = sch::pack(&sch).unwrap();
        let d = gsm_decoded(&bytes, Hz(947_400_000)).expect("a row");
        assert_eq!(d.protocol, "GSM-SCH");
        assert_eq!(d.crc_ok, Some(true));
        let get = |k: &str| d.fields.iter().find(|(n, _)| n == k).map(|(_, v)| v.clone());
        assert_eq!(get("bsic"), Some(Value::Int(0x2B)));
        assert_eq!(get("frame"), Some(Value::Int(i64::from(sch.frame_number))));
        assert_eq!(get("arfcn"), Some(Value::Int(62)), "947.4 MHz is channel 62");
        assert!(d.detail.as_deref().unwrap().contains("BSIC 53"));
    }

    /// The whole path on synthetic RF: a beacon into the node, a cell out of
    /// the row it produces.
    ///
    /// The point is that the three layers agree. The demodulator's soft bits,
    /// the channel coding's field order and the row's reading of the packed
    /// bytes are each tested alone and each could be self-consistently wrong;
    /// only running them together shows that what the node puts on the bus is
    /// what a consumer gets back.
    #[test]
    fn a_modulated_beacon_becomes_a_cell_on_the_bus() {
        let want = Sch { ncc: 2, bcc: 6, frame_number: 51 * 26 * 9 + 31 };
        let (rate, center) = (2_400_000.0, 947_400_000.0);
        let iq = beacon(&want, rate);

        let mut node = GsmNode::default();
        node.negotiate(&spec(rate, center)).unwrap();
        let ins = [spec(rate, center)];
        let tags = Vec::new();
        let mut frames: Vec<common::Frame> = Vec::new();
        for block in iq.chunks(8192) {
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

        assert_eq!(frames.len(), 1, "expected one burst off the air");
        let f = &frames[0];
        // Every packet carries what it was heard at, measured on the channel
        // rather than on the span.
        assert!(f.rssi_dbfs.is_finite(), "level {}", f.rssi_dbfs);
        assert!(f.snr_db.is_finite() && f.snr_db > 0.0, "snr {}", f.snr_db);
        assert!(f.iq.is_some(), "a burst with no samples behind it");

        let d = gsm_decoded(&f.bytes, Hz(f.center_hz)).expect("a row");
        assert_eq!(d.detail.as_deref(), Some("ARFCN 62 BSIC 26 frame 11965"));
    }

    /// A frequency correction burst and, one TDMA frame later, the
    /// synchronisation burst for `sch`, in a span of noise.
    fn beacon(sch: &Sch, rate: f64) -> Vec<C32> {
        let sps = 8;
        let work = gsm::SYMBOL_RATE * sps as f64;
        let lead = 200.0;
        let fcch = gsm::modulate(&[0u8; gsm::BURST_BITS], sps);
        let sync = gsm::modulate(&gsm::sch_burst_bits(sch).unwrap(), sps);
        let total =
            ((lead * 2.0 + gsm::FRAME_SYMBOLS + gsm::BURST_SYMBOLS) * sps as f64) as usize;
        let mut base = vec![C32::new(0.0, 0.0); total];
        base[(lead * sps as f64) as usize..][..fcch.len()].copy_from_slice(&fcch);
        let at = ((lead + gsm::FRAME_SYMBOLS) * sps as f64) as usize;
        base[at..][..sync.len()].copy_from_slice(&sync);

        let ratio = work / rate;
        let n = (base.len() as f64 / ratio) as usize - 1;
        let mut seed = 0x9E37_79B9u32;
        let mut rand = move || {
            seed ^= seed << 13;
            seed ^= seed >> 17;
            seed ^= seed << 5;
            (seed as f32 / u32::MAX as f32) - 0.5
        };
        // A little noise, so the floor the level is measured against is a
        // floor rather than a divide by zero.
        (0..n).map(|i| base[(i as f64 * ratio) as usize] + C32::new(rand(), rand()) * 0.05).collect()
    }
}
