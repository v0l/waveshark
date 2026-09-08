//! 802.11a/g as a graph node.
//!
//! Wiring only, like `ble_nodes`: the OFDM receiver is `dsp::wifi` and the MAC
//! frame is `decode::wifi`, and neither knows about the other or about
//! pipelines.
//!
//! # The span is the channel
//!
//! Everything else here cuts a narrow channel out of the span and reads that.
//! An 802.11 channel is 20 MHz wide, which is as wide as a HackRF can sample
//! and most of what a LimeSDR can, so there is nothing to cut: the node reads
//! the span itself and the span is one channel. That is why it is `span_wide`
//! and why it refuses anything under 20 MS/s rather than doing its best.
//!
//! # What a frame reports it was heard on
//!
//! The channel the tuner is parked on, not the one a beacon claims in its DS
//! parameter set. Those disagree more often than one would think: an access
//! point on channel 6 is heard on channel 1 through the skirt of a 20 MHz
//! filter, and its beacon still says 6. The claim is carried as a field so
//! the two can be compared, and the port centre stays the truth about where
//! the receiver was listening.

use crate::protocol::{Mark, Placed, Placement, Protocol, Shape, Stickiness};
use crate::NodeSpec;
use common::Result;
use decode::wifi as mac;
use dsp::wifi::{ofdm, WifiConfig, WifiDetector, WifiFrame};
use pipeline::event::Decoded;
use pipeline::node::{NodeCtx, PortSpec, Simple};
use pipeline::port::{Payload, PortKind, StreamSpec};

/// One channel's width, which is the whole span this reads.
pub const CHANNEL_WIDTH_HZ: f64 = ofdm::CHANNEL_WIDTH_HZ;

/// Channel 6, which is where a 2.4 GHz receiver that has to pick one sits.
pub const DEFAULT_HZ: f64 = 2_437_000_000.0;

/// Samples kept behind the receiver so a frame can carry its own. Six
/// milliseconds is longer than any legal frame at 6 Mbit/s, so nothing is
/// truncated; at 20 MS/s it is a megabyte of ring, which is the price of
/// reading a band this wide.
const KEEP_S: f64 = 0.006;

/// A frame's samples are capped at this many, about 1.6 ms.
///
/// A 4095 byte frame at 6 Mbit/s is 110,000 samples, and a busy access point
/// sends hundreds of frames a second: carrying every sample of every one of
/// them puts a gigabyte a second on the packet bus. What the capture is for
/// is looking at a decode that went wrong, and the preamble and the first
/// symbols are where that is visible.
const MAX_FRAME_IQ: usize = 32_768;

/// Every 20 MHz channel this can be placed on: the 2.4 GHz band and the
/// 5 GHz one.
pub fn channels() -> Vec<f64> {
    let mut v: Vec<f64> = (1..=13).filter_map(dsp::wifi::channel_2ghz).collect();
    v.push(2_484_000_000.0);
    v.extend(dsp::wifi::channels_5ghz());
    v
}

/// The channel number a centre names, if it names one.
///
/// Half a megahertz, which is tight because the bands are crowded: BLE's
/// advertising channel 38 is at 2426 MHz and Wi-Fi channel 4 is at 2427, so
/// a wider window would file every Bluetooth advertisement under a Wi-Fi
/// channel and try to read it as a MAC frame.
pub fn channel_of(center_hz: f64) -> Option<u16> {
    for n in 1..=14u8 {
        if let Some(hz) = dsp::wifi::channel_2ghz(n) {
            if (hz - center_hz).abs() <= 0.5e6 {
                return Some(u16::from(n));
            }
        }
    }
    (1..=196u16).find(|&n| (dsp::wifi::channel_5ghz(n) - center_hz).abs() <= 0.5e6 && n >= 32)
}

/// Whether a packet's reported centre says it came off a Wi-Fi channel.
pub fn is_wifi_channel(center_hz: f64) -> bool {
    channel_of(center_hz).is_some()
}

pub struct WifiNode {
    cfg: WifiConfig,
    det: Option<WifiDetector>,
    meter: crate::FrameMeter,
    frames: Vec<WifiFrame>,
    accepted: u64,
}

impl Default for WifiNode {
    fn default() -> Self {
        Self::new(WifiConfig::default())
    }
}

impl WifiNode {
    pub fn new(cfg: WifiConfig) -> Self {
        Self {
            cfg,
            det: None,
            meter: crate::FrameMeter::new(ofdm::RATE_HZ, DEFAULT_HZ as u64, KEEP_S),
            frames: Vec::new(),
            accepted: 0,
        }
    }

    /// Frames that passed their FCS since the node was built.
    pub fn accepted(&self) -> u64 {
        self.accepted
    }
}

impl Simple for WifiNode {
    fn name(&self) -> &str {
        "wifi"
    }

    fn negotiate(&mut self, i: &PortSpec) -> Result<StreamSpec> {
        if i.spec.kind != PortKind::Iq {
            return Err(common::Error::other("wifi reads complex baseband"));
        }
        let (rate, center) = (i.spec.rate, i.spec.center.as_f64());
        let Some(det) = WifiDetector::new(rate, self.cfg) else {
            return Err(common::Error::other(
                "802.11 needs a whole 20 MHz channel: a rate that is a multiple of 20 MS/s",
            ));
        };
        self.det = Some(det);
        self.meter = crate::FrameMeter::new(rate, center as u64, KEEP_S);
        let mut out = i.spec.with_kind(PortKind::Frames);
        out.bandwidth = CHANNEL_WIDTH_HZ;
        Ok(out)
    }

    fn process(&mut self, i: &Payload, o: &mut Payload, _c: &mut NodeCtx<'_>) -> Result<()> {
        let (Some(iq), Some(det)) = (i.as_iq(), self.det.as_mut()) else {
            return Ok(());
        };
        self.meter.feed(iq);
        self.frames.clear();
        det.process(iq, &mut self.frames);
        let out = o.frames_mut();
        for f in &self.frames {
            if !f.fcs_ok {
                continue;
            }
            self.accepted += 1;
            let mut frame = common::Frame::measured(f.psdu.clone(), f.rssi_dbfs, f.snr_db);
            // Preamble, SIGNAL and as much of the payload as the cap allows.
            let len = (320 + 80 + f.psdu.len() * 8 * 20 / f.rate.mbps as usize).min(MAX_FRAME_IQ);
            frame.iq = self.meter.iq_at(f.start_sample, len);
            out.push(frame);
        }
        Ok(())
    }

    fn reset(&mut self) {
        self.meter.reset();
        if let Some(d) = self.det.as_mut() {
            d.reset();
        }
    }
}

/// The row a MAC frame becomes.
///
/// `None` when the bytes are not a MAC frame, which is how the packet bus
/// tells one from anything else arriving on the same centre.
pub fn wifi_decoded(bytes: &[u8], center: common::Hz) -> Option<Decoded> {
    use common::Value;
    if !dsp::wifi::fcs_ok(bytes) {
        return None;
    }
    let f = mac::parse(bytes)?;
    let mut fields: Vec<(String, Value)> = vec![("type".into(), Value::Text(f.kind.name().into()))];
    if let Some(ch) = channel_of(center.as_f64()) {
        fields.push(("channel".into(), Value::Int(i64::from(ch))));
    }
    if let Some(n) = &f.network {
        if let Some(ssid) = &n.ssid {
            fields.push(("ssid".into(), Value::Text(ssid.clone())));
        } else if matches!(f.kind, mac::Kind::Management(8)) {
            fields.push(("ssid".into(), Value::Text("<hidden>".into())));
        }
        if let Some(ch) = n.channel {
            fields.push(("claims_channel".into(), Value::Int(i64::from(ch))));
        }
        if n.beacon_interval > 0 {
            fields.push((
                "beacon_ms".into(),
                Value::Int(i64::from(n.beacon_interval) * 1024 / 1000),
            ));
        }
        let security = match (n.rsn, n.privacy) {
            (true, _) => "wpa2",
            (false, true) => "wep",
            (false, false) => "open",
        };
        fields.push(("security".into(), Value::Text(security.into())));
    }
    if f.protected {
        fields.push(("protected".into(), Value::Int(1)));
    }
    if let Some(a) = f.source() {
        if a.is_local() {
            fields.push(("randomised".into(), Value::Int(1)));
        }
    }
    if let Some(s) = f.seq {
        fields.push(("seq".into(), Value::Int(i64::from(s))));
    }
    let detail = fields
        .iter()
        .map(|(k, v)| format!("{k}={v}"))
        .collect::<Vec<_>>()
        .join(" ");
    let link = pipeline::event::Link {
        from: f
            .source()
            .map(|a| pipeline::event::Party::unit(a.to_string())),
        to: Some(if f.addr1.is_broadcast() {
            pipeline::event::Party::broadcast()
        } else {
            pipeline::event::Party::unit(f.addr1.to_string())
        }),
    };
    // The row is filed under whoever transmitted it. A control frame names
    // nobody, so it is filed under the station it is addressed to, which is
    // the only party it has.
    let who_addr = f.source().unwrap_or(f.addr1);
    let mut who = common::Identity::new("wifi", who_addr.to_string());
    who.name = f.network.as_ref().and_then(|n| n.ssid.clone());
    Some(
        Decoded::bytes("802.11", center, 0.0, bytes.to_vec())
            .with_link(link)
            .by(who)
            .with_detail(detail)
            .with_fields(fields)
            .with_modulation("OFDM")
            // Nothing reaches here without the frame check sequence, which
            // is a real CRC-32 over the whole frame.
            .with_crc(Some(true)),
    )
}

/// 802.11a/g as the auto node and the tables know it: one 20 MHz channel,
/// read off the span because there is no narrower stream an OFDM frame can
/// be cut into.
pub struct Wifi;

impl Protocol for Wifi {
    fn id(&self) -> &'static str {
        "wifi"
    }
    fn label(&self) -> &'static str {
        "wifi"
    }
    fn placement(&self) -> Placement {
        Placement::Channels(channels())
    }
    fn shape(&self) -> Shape {
        Shape {
            widths: &[CHANNEL_WIDTH_HZ],
            min_rate_hz: ofdm::RATE_HZ,
            feed_rate_hz: ofdm::RATE_HZ,
            span_wide: true,
            families: &[],
        }
    }
    fn default_hz(&self) -> f64 {
        DEFAULT_HZ
    }
    /// Nothing is locked. A latch on a span-wide decoder hands it its whole
    /// band from the moment the span reaches it, and this band is 20 MHz of
    /// shared spectrum: 2.4 GHz holds Bluetooth and every ISM device there
    /// is, and 5.8 GHz holds the FPV video channels. Owning it would keep
    /// the detector out of all of them for a beacon.
    fn stickiness(&self) -> Stickiness {
        Stickiness::Forget
    }
    fn stage_label(&self, hz: f64) -> String {
        match channel_of(hz) {
            Some(ch) => format!("WIFI {ch}"),
            None => format!("{:.0} WIFI", hz / 1e6),
        }
    }
    fn marks(&self, hz: f64) -> Vec<Mark> {
        let label = match channel_of(hz) {
            Some(ch) => format!("WIFI {ch}"),
            None => "WIFI".into(),
        };
        vec![Mark {
            hz,
            width_hz: CHANNEL_WIDTH_HZ,
            label,
        }]
    }
    fn chain(&self, _at: Placed) -> Vec<NodeSpec> {
        vec![NodeSpec::new("wifi")]
    }
    /// A beacon is the same news a hundred times a second. One row per
    /// network per channel is what a person wants to see; a beacon whose
    /// SSID or security changed is a different key and reports again.
    fn dedupe_key(&self, p: &common::Packet) -> Option<Vec<u8>> {
        let common::PacketBody::Frame(fr) = &p.body else {
            return None;
        };
        let f = mac::parse(&fr.bytes)?;
        if !matches!(f.kind, mac::Kind::Management(8)) {
            return None;
        }
        let mut key = b"beacon".to_vec();
        key.extend(f.bssid()?.0);
        if let Some(n) = &f.network {
            key.extend(n.ssid.clone().unwrap_or_default().as_bytes());
            key.push(u8::from(n.privacy) | u8::from(n.rsn) << 1);
        }
        Some(key)
    }
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

    fn beacon() -> Vec<u8> {
        let mut v = vec![0x80, 0x00, 0x00, 0x00];
        v.extend([0xff; 6]);
        v.extend([0x00, 0x1a, 0x2b, 0x3c, 0x4d, 0x5e]);
        v.extend([0x00, 0x1a, 0x2b, 0x3c, 0x4d, 0x5e]);
        v.extend([0x10, 0x00]);
        v.extend([0u8; 8]);
        v.extend([0x64, 0x00]);
        v.extend([0x11, 0x04]);
        v.extend([0x00, 0x09]);
        v.extend(b"waveshark");
        v.extend([0x03, 0x01, 0x06]);
        v.extend([0x30, 0x02, 0x01, 0x00]);
        let crc = dsp::wifi::crc32(&v);
        v.extend(crc.to_le_bytes());
        v
    }

    #[test]
    fn the_node_refuses_a_span_too_narrow_for_a_channel() {
        let mut n = WifiNode::default();
        assert!(n.negotiate(&spec(20_000_000.0, 2_437_000_000.0)).is_ok());
        assert!(n.negotiate(&spec(40_000_000.0, 5_180_000_000.0)).is_ok());
        // An RTL-SDR's whole range, and a rate that is not a multiple.
        assert!(n.negotiate(&spec(2_400_000.0, 2_437_000_000.0)).is_err());
        assert!(n.negotiate(&spec(30_000_000.0, 2_437_000_000.0)).is_err());
    }

    /// The receiver reads a frame out of samples the transmitter in `dsp`
    /// made, through the node, and the row names the network.
    #[test]
    fn a_beacon_off_the_span_becomes_a_row_naming_the_network() {
        let want = beacon();
        let mut n = WifiNode::default();
        let out = n.negotiate(&spec(20_000_000.0, 2_437_000_000.0)).unwrap();
        assert_eq!(out.kind, PortKind::Frames);

        let mut samples = vec![common::C32::default(); 2000];
        samples.extend(dsp::wifi::tx::frame(&want, 6, 0x5d));
        samples.extend(vec![common::C32::default(); 2000]);
        let input = Payload::Iq(samples);
        let mut output = Payload::Frames(Vec::new());
        let ins = [spec(20_000_000.0, 2_437_000_000.0)];
        let (tags, mut events, mut new_tags) = (Vec::new(), Vec::new(), Vec::new());
        let mut ctx = NodeCtx::new(0, &ins, &tags, &mut events, &mut new_tags);
        n.process(&input, &mut output, &mut ctx).unwrap();

        let frames = output.as_frames().expect("frames");
        assert_eq!(frames.len(), 1);
        assert_eq!(frames[0].bytes, want);
        assert!(frames[0].rssi_dbfs.is_finite() && frames[0].snr_db.is_finite());
        assert!(
            frames[0].iq.is_some(),
            "a frame carries what it was read from"
        );

        let d = wifi_decoded(&frames[0].bytes, Hz(2_437_000_000)).expect("a decode");
        assert_eq!(d.protocol, "802.11");
        let detail = d.detail.as_deref().unwrap();
        assert!(detail.contains("ssid=waveshark"), "{detail}");
        assert!(detail.contains("type=beacon"), "{detail}");
        assert!(detail.contains("channel=6"), "{detail}");
        assert!(detail.contains("security=wpa2"), "{detail}");
    }

    #[test]
    fn bytes_that_are_not_a_frame_are_not_a_row() {
        assert!(wifi_decoded(&[0u8; 20], Hz(2_437_000_000)).is_none());
        let mut bad = beacon();
        bad[8] ^= 0xff;
        assert!(wifi_decoded(&bad, Hz(2_437_000_000)).is_none());
    }

    #[test]
    fn the_channel_a_centre_names_is_the_one_on_the_box() {
        assert_eq!(channel_of(2_437_000_000.0), Some(6));
        assert_eq!(channel_of(2_412_000_000.0), Some(1));
        assert_eq!(channel_of(5_180_000_000.0), Some(36));
        assert_eq!(channel_of(2_426_000_000.0), None);
        assert!(!is_wifi_channel(868_000_000.0));
    }
}

#[cfg(test)]
mod placement_tests {
    use crate::auto::AutoNode;
    use common::Hz;
    use dsp::source::SourceConfig;
    use pipeline::node::{Node, PortSpec};
    use pipeline::port::StreamSpec;

    fn spec(rate: f64, hz: Hz) -> PortSpec {
        PortSpec {
            spec: StreamSpec::iq(rate, hz),
            latency: 0,
        }
    }

    /// The auto node places this for itself when the span is a channel, and
    /// does not when the span is too narrow to hold one.
    #[test]
    fn a_twenty_megahertz_span_on_a_channel_gets_a_wifi_front_end() {
        let mut n = AutoNode::new("auto", SourceConfig::default());
        Node::negotiate(&mut n, &[spec(20e6, Hz::mhz(2437))]).unwrap();
        assert!(n.wide().contains(&"wifi"), "{:?}", n.wide());
        Node::negotiate(&mut n, &[spec(2.4e6, Hz::mhz(2437))]).unwrap();
        assert!(!n.wide().contains(&"wifi"), "{:?}", n.wide());
    }
}
