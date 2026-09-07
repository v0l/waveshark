//! BLE advertising as a graph node.
//!
//! Wiring only, like `ais_nodes`: the demodulator, the whitening and the CRC
//! are `dsp::ble`, the PDU tables are `decode::ble`, and neither knows about
//! pipelines or about the other.
//!
//! # One channel at a time, and why that is not a limitation
//!
//! The three primary advertising channels are 2402, 2426 and 2480 MHz, so the
//! nearest pair is 24 MHz apart. No tuner this receiver supports samples that
//! wide: a HackRF stops at 20 MS/s. A span therefore covers exactly one
//! advertising channel in practice, and the node reports frames at that
//! channel's own frequency rather than at some band centre they were never
//! transmitted on. Where a span somehow covers more than one, every covered
//! channel is read and the port carries the band instead, because a frame
//! cannot then be placed by the port alone.

use common::Result;
use decode::ble as pdu;
use dsp::ble::{BleConfig, BleDetector, BleFrame, ADV_CHANNELS};
use pipeline::event::Decoded;
use pipeline::node::{NodeCtx, PortSpec, Simple};
use pipeline::port::{Payload, PortKind, StreamSpec};

/// The width one advertising channel occupies: 1 MHz of modulation with the
/// guard that puts the neighbours 2 MHz away.
pub const CHANNEL_WIDTH_HZ: f64 = 2_000_000.0;

/// Where a receiver tunes when it cannot say which channel a frame came from.
/// The middle of the 2.4 GHz ISM band, which is not an advertising channel.
pub const BAND_CENTER_HZ: f64 = 2_441_000_000.0;

/// Whether a packet's reported centre says it came off an advertising
/// channel. The same trick `dsp::ais::is_ais_band` uses: where a frame was
/// received is evidence it already carries.
pub fn is_advertising_channel(center_hz: f64) -> bool {
    channel_of(center_hz).is_some()
}

/// The advertising channel index a centre names, if it names one.
pub fn channel_of(center_hz: f64) -> Option<u8> {
    ADV_CHANNELS
        .iter()
        .find(|(_, hz)| (hz - center_hz).abs() < 500_000.0)
        .map(|&(ch, _)| ch)
}

pub struct BleNode {
    rate: f64,
    cfg: BleConfig,
    det: BleDetector,
    meter: crate::FrameMeter,
    frames: Vec<BleFrame>,
    accepted: u64,
}

impl Default for BleNode {
    fn default() -> Self {
        Self::new(BleConfig::default())
    }
}

impl BleNode {
    pub fn new(cfg: BleConfig) -> Self {
        Self {
            rate: 16_000_000.0,
            cfg,
            // Replaced at negotiation, when the real rate and centre are known.
            det: BleDetector::new(16_000_000.0, ADV_CHANNELS[1].1, cfg),
            // Two milliseconds at 16 MS/s: an advertisement is 80 to 400 us,
            // so a frame's own samples are in there without keeping a ring
            // the size of the span.
            meter: crate::FrameMeter::new(16_000_000.0, ADV_CHANNELS[1].1 as u64, 0.002),
            frames: Vec::new(),
            accepted: 0,
        }
    }

    fn meter_rate(&self) -> f64 {
        self.rate
    }

    /// Packets that passed CRC since the node was built.
    pub fn accepted(&self) -> u64 {
        self.accepted
    }
}

impl Simple for BleNode {
    fn name(&self) -> &str {
        "ble"
    }

    fn negotiate(&mut self, i: &PortSpec) -> Result<StreamSpec> {
        if i.spec.kind != PortKind::Iq {
            return Err(common::Error::other("ble reads complex baseband"));
        }
        let (rate, center) = (i.spec.rate, i.spec.center.as_f64());
        // Two megasamples is the modulation's own width, and a channel read
        // through the anti-alias skirt is a channel read as silence.
        if rate < 4_000_000.0 {
            return Err(common::Error::other("ble needs at least 4 MS/s"));
        }
        let det = BleDetector::new(rate, center, self.cfg);
        let covered = det.channels();
        let hz = match covered.as_slice() {
            [] => {
                return Err(common::Error::other(
                    "ble needs an advertising channel (2402, 2426 or 2480 MHz) inside the span",
                ))
            }
            [one] => ADV_CHANNELS
                .iter()
                .find(|(c, _)| c == one)
                .map(|(_, hz)| *hz)
                .unwrap(),
            _ => BAND_CENTER_HZ,
        };
        self.det = det;
        self.meter = crate::FrameMeter::new(rate, hz as u64, 0.002);
        self.rate = rate;
        let mut out = i.spec.with_kind(PortKind::Frames);
        out.center = common::Hz(hz as u64);
        out.bandwidth = CHANNEL_WIDTH_HZ;
        Ok(out)
    }

    fn process(&mut self, i: &Payload, o: &mut Payload, _c: &mut NodeCtx<'_>) -> Result<()> {
        let Some(iq) = i.as_iq() else { return Ok(()) };
        self.meter.feed(iq);
        self.frames.clear();
        self.det.process(iq, &mut self.frames);
        let out = o.frames_mut();
        for f in &self.frames {
            self.accepted += 1;
            // The detector measured this burst against the floor either side
            // of it, which is a better number than a block mean; what it
            // does not keep is the samples, and the channel a span holding
            // more than one advertising channel cannot get from the port.
            let hz = ADV_CHANNELS
                .iter()
                .find(|(c, _)| *c == f.channel)
                .map(|(_, hz)| *hz as u64)
                .unwrap_or(BAND_CENTER_HZ as u64);
            let mut out_frame = common::Frame::measured(f.pdu.clone(), f.rssi_dbfs, f.snr_db).at(hz);
            // A frame is 8 preamble bits plus the PDU at one bit a
            // microsecond, with room either side for the ramp.
            let len = ((f.pdu.len() + 12) * 8) as f64 * 1e-6 * self.meter_rate();
            out_frame.iq = self.meter.iq_at(f.start_sample, len as usize);
            out.push(out_frame);
        }
        Ok(())
    }

    fn reset(&mut self) {
        self.meter.reset();
        self.det.reset();
    }
}

/// The decode an advertising PDU becomes.
///
/// `None` when the bytes are not a PDU this reads, which is how the packet bus
/// tells a BLE frame from anything else that arrived on the same centre.
pub fn ble_decoded(bytes: &[u8], center: common::Hz) -> Option<Decoded> {
    use common::Value;
    let adv = pdu::parse(bytes)?;
    let mut fields = adv.fields();
    if let Some(ch) = channel_of(center.as_f64()) {
        fields.insert(0, ("channel".into(), Value::Int(i64::from(ch))));
    }
    let detail = fields
        .iter()
        .map(|(k, v)| format!("{k}={v}"))
        .collect::<Vec<_>>()
        .join(" ");
    let link = pipeline::event::Link {
        from: Some(pipeline::event::Party::unit(adv.address.to_string())),
        to: Some(match adv.target {
            Some(t) => pipeline::event::Party::unit(t.to_string()),
            None => pipeline::event::Party::broadcast(),
        }),
    };
    Some(
        Decoded::bytes("BLE-Adv", center, 0.0, bytes.to_vec())
            .with_link(link)
            .with_detail(detail)
            .with_fields(fields)
            .with_modulation("GFSK")
            // Everything that reaches here passed the link layer's CRC-24 in
            // the demodulator, which is a real check and not an argument
            // from plausibility.
            .with_crc(Some(true)),
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use common::Hz;

    fn spec(rate: f64, center: f64) -> PortSpec {
        PortSpec {
            spec: StreamSpec::iq(rate, Hz(center as u64)),
            latency: 0,
        }
    }

    #[test]
    fn the_node_refuses_a_span_with_no_advertising_channel_in_it() {
        let mut n = BleNode::default();
        assert!(n.negotiate(&spec(16_000_000.0, 2_430_000_000.0)).is_ok());
        // Inside the ISM band but between the advertising channels.
        assert!(n.negotiate(&spec(8_000_000.0, 2_450_000_000.0)).is_err());
        // Wide enough, wrong band.
        assert!(n.negotiate(&spec(16_000_000.0, 868_000_000.0)).is_err());
        // On channel 38, too narrow to hold the modulation.
        assert!(n.negotiate(&spec(2_000_000.0, 2_426_000_000.0)).is_err());
    }

    /// Frames are tagged with the channel they arrived on, not with the
    /// tuner's centre: 2430 is where this receiver was parked and no BLE
    /// packet was ever sent there.
    #[test]
    fn frames_are_tagged_with_the_advertising_channel() {
        let mut n = BleNode::default();
        let out = n.negotiate(&spec(16_000_000.0, 2_430_000_000.0)).unwrap();
        assert_eq!(out.kind, PortKind::Frames);
        assert_eq!(out.center, Hz(2_426_000_000));
        assert_eq!(channel_of(out.center.as_f64()), Some(38));
    }

    #[test]
    fn a_pdu_becomes_a_row_naming_the_device() {
        let pdu = [
            0x04, 0x1b, 0x4d, 0x72, 0xef, 0xcb, 0x70, 0x6c, 0x14, 0x08, 0x34, 0x39, 0x22, 0x20,
            0x4f, 0x64, 0x79, 0x73, 0x73, 0x65, 0x79, 0x20, 0x4f, 0x4c, 0x45, 0x44, 0x20, 0x47,
            0x39,
        ];
        let d = ble_decoded(&pdu, Hz(2_426_000_000)).expect("a decode");
        assert_eq!(d.protocol, "BLE-Adv");
        assert_eq!(d.crc_ok, Some(true));
        assert!(d.detail.as_deref().unwrap().contains("6C:70:CB:EF:72:4D"));
        assert!(d.detail.as_deref().unwrap().contains("channel=38"));
    }
}
