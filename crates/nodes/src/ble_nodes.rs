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
//!
//! # Why the data channels are not locked
//!
//! A connected device hops across the 37 data channels two megahertz apart,
//! which is the same shape as an ExpressLRS handset's hop set, and a
//! [`pipeline::Lock`] over that raster was considered for the same reason:
//! every hop the detector opens is a source with a classifier and a set of
//! decoders on it. It is the wrong mechanism here twice over. A lock says
//! that the front end holding it will read the bursts it claims, and this one
//! will not: `BleConfig::data_channels` is off by default because a mixer and
//! a decimator per channel is dear, and even switched on it reads the data
//! channels off the span rather than out of a source handed to it, so a
//! claimed hop would be routed into silence. The lock layer would then score
//! the claims, find that none of them decoded, and drop the lock, which is
//! the right answer arrived at expensively. And a connection's packets are
//! whitened under an access address a listener never saw, so there is nothing
//! to decode there in the first place.
//!
//! Measured on the 61.44 MS/s capture of a busy 2.4 GHz band, this costs
//! nothing: none of the sources the detector opens there sits on the data
//! channel raster, and the 2 MHz hops of a connection are not among what it
//! finds. A capture that does hold them is what would be needed to say more.

use crate::protocol::{FrameClaim, Mark, Placed, Placement, Protocol, Shape};
use crate::NodeSpec;
use common::Result;
use decode::ble as pdu;
use dsp::ble::{BleConfig, BleDetector, BleFrame, ADV_CHANNELS};
use pipeline::event::Decoded;
use pipeline::node::{NodeCtx, PortSpec, Simple};
use pipeline::port::{Payload, PortKind, StreamSpec};
use pipeline::registry::{Category, Settings, StageDesc};

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
    ADV_CHANNELS.iter().find(|(_, hz)| (hz - center_hz).abs() < 500_000.0).map(|&(ch, _)| ch)
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
            [one] => ADV_CHANNELS.iter().find(|(c, _)| c == one).map(|(_, hz)| *hz).unwrap(),
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
            let mut out_frame =
                common::Frame::measured(f.pdu.clone(), f.rssi_dbfs, f.snr_db).at(hz);
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
    // An aircraft's broadcast is not a row about a Bluetooth device that
    // happens to carry some bytes, so it is named for what it is and its own
    // fields go in front of the link layer's.
    let odid: Vec<decode::odid::Parsed> = adv
        .data
        .iter()
        .filter(|s| s.kind == 0x16)
        .filter_map(|s| decode::odid::from_service_data(&s.value))
        .flatten()
        .collect();
    let protocol = if odid.is_empty() { "BLE-Adv" } else { "OpenDroneID" };
    if !odid.is_empty() {
        let mut f = decode::odid::fields(&odid);
        f.append(&mut fields);
        fields = f;
    }
    if let Some(ch) = channel_of(center.as_f64()) {
        fields.insert(0, ("channel".into(), Value::Int(i64::from(ch))));
    }
    let detail = fields.iter().map(|(k, v)| format!("{k}={v}")).collect::<Vec<_>>().join(" ");
    let link = pipeline::event::Link {
        from: Some(pipeline::event::Party::unit(adv.address.to_string())),
        to: Some(match adv.target {
            Some(t) => pipeline::event::Party::unit(t.to_string()),
            None => pipeline::event::Party::broadcast(),
        }),
    };
    let mut who = common::Identity::new("ble", adv.address.to_string());
    who.name = adv.name.clone();
    who.vendor = adv.company.and_then(pdu::company_name).map(str::to_string);
    Some(
        Decoded::bytes(protocol, center, 0.0, bytes.to_vec())
            .with_link(link)
            .by(who)
            .with_detail(detail)
            .with_fields(fields)
            .with_modulation(common::Modulation::Gfsk)
            // Everything that reaches here passed the link layer's CRC-24 in
            // the demodulator, which is a real check and not an argument
            // from plausibility.
            .with_crc(Some(true)),
    )
}

/// BLE advertising as the auto node and the tables know it: whichever of
/// the three channels the span holds, read off the span because an
/// advertisement is 80 us of a hopping device that may never be heard
/// twice, which is not enough for a source to open around.
pub struct Ble;

impl Protocol for Ble {
    fn id(&self) -> &'static str {
        "ble"
    }
    fn label(&self) -> &'static str {
        "ble"
    }
    fn aliases(&self) -> &'static [&'static str] {
        &["bluetooth"]
    }
    fn placement(&self) -> Placement {
        Placement::Channels(ADV_CHANNELS.iter().map(|(_, hz)| *hz).collect())
    }
    fn frame_claim(&self) -> FrameClaim {
        FrameClaim::Band { width_hz: 1_000_000 }
    }
    /// An advertising channel is a frequency nothing else here transmits a
    /// frame from.
    fn read_frame(&self, p: &common::Packet, bytes: &[u8]) -> Option<Vec<Decoded>> {
        if !is_advertising_channel(p.center_hz() as f64) {
            return None;
        }
        Some(ble_decoded(bytes, common::Hz(p.center_hz())).into_iter().collect())
    }
    /// It cuts its three advertising channels out of the span itself, rather
    /// than taking them from the bank the extractor channelizes the span
    /// with.
    ///
    /// Measured against what the bank offers: its channels are
    /// [`dsp::source::BANK_CHANNEL_HZ`] apart at twice that rate, so one is
    /// 1 MHz wide at 2 MS/s. An advertising channel is 2 MHz wide and this
    /// decoder refuses under 4 MS/s, since at 1 Mbit/s the bit centres have
    /// to be found in the samples themselves and four a symbol is where the
    /// packet count stops moving. So a bank channel is half the width and
    /// half the rate it needs, and a pair of them summed is one channel's
    /// worth of flat response, not two. Widening the bank to suit would
    /// widen it for every source cut from it.
    fn shape(&self) -> Shape {
        Shape {
            widths: &[CHANNEL_WIDTH_HZ],
            min_rate_hz: 4_000_000.0,
            feed_rate_hz: 8_000_000.0,
            span_wide: true,
            families: &[],
        }
    }
    /// Advertising channel 38, which sits in the gap between the Wi-Fi
    /// channels and is the one of the three least often buried.
    fn default_hz(&self) -> f64 {
        2_426_000_000.0
    }
    fn stage_label(&self, hz: f64) -> String {
        format!("{:.0} BLE", hz / 1e6)
    }
    fn marks(&self, hz: f64) -> Vec<Mark> {
        let label = match channel_of(hz) {
            Some(ch) => format!("BLE {ch}"),
            None => "BLE".into(),
        };
        vec![Mark { hz, width_hz: CHANNEL_WIDTH_HZ, label }]
    }
    fn chain(&self, _at: Placed) -> Vec<NodeSpec> {
        vec![NodeSpec::new("ble")]
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use common::Hz;

    fn spec(rate: f64, center: f64) -> PortSpec {
        PortSpec { spec: StreamSpec::iq(rate, Hz(center as u64)), latency: 0 }
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

    /// An aircraft's broadcast is the same link layer carrying service data,
    /// and what makes it another row is what is inside. The serial leads and
    /// the Bluetooth address stays behind it, because a drone's address
    /// rotates and the serial is the airframe.
    #[test]
    fn an_advertisement_carrying_open_drone_id_becomes_an_aircraft_row() {
        let mut msg = vec![0x02, (1 << 4) | 2];
        let mut id = b"1596F3AAAAAAAAAAAAAA".to_vec();
        id.resize(20, 0);
        msg.extend_from_slice(&id);
        msg.resize(25, 0);

        let mut sd = vec![0xfa, 0xff, 0x0d, 3];
        sd.extend_from_slice(&msg);

        let mut pdu = vec![0x02, 0];
        pdu.extend_from_slice(&[0x11, 0x22, 0x33, 0x44, 0x55, 0x66]);
        pdu.push((sd.len() + 1) as u8);
        pdu.push(0x16);
        pdu.extend_from_slice(&sd);
        pdu[1] = (pdu.len() - 2) as u8;

        let d = ble_decoded(&pdu, Hz(2_402_000_000)).expect("a decode");
        assert_eq!(d.protocol, "OpenDroneID");
        let detail = d.detail.as_deref().unwrap();
        assert!(detail.contains("uas_id=1596F3AAAAAAAAAAAAAA"), "{detail}");
        assert!(detail.contains("ua_type=multirotor"), "{detail}");
        assert!(detail.contains("66:55:44:33:22:11"), "{detail}");
    }
}

pub const DESC: StageDesc = StageDesc {
    name: "ble",
    summary: "One BLE advertising channel: GFSK at 1 Mbit/s, dewhitening and CRC-24",
    category: Category::Decode,
    feeds_bus: true,
};

pub fn build(_s: &Settings) -> Result<Box<dyn pipeline::node::Node>> {
    Ok(Box::new(BleNode::default()))
}
