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

use crate::NodeSpec;
use crate::protocol::{FrameClaim, Mark, Placed, Placement, Protocol, Shape, Stickiness};
use common::Result;
use decode::wifi as mac;
pub use decode::wifi::CHANNEL_WIDTH_HZ;
pub use decode::wifi::channel_of;
pub use decode::wifi::read;
use dsp::wifi::{WifiConfig, WifiFrame, WifiSpan, ofdm};
use identify::Signal;
pub use identify::wifi::DEFAULT_HZ;
pub use identify::wifi::Wifi;
pub use identify::wifi::channels;
use pipeline::node::{NodeCtx, PortSpec, Simple};
use pipeline::port::{Payload, PortKind, StreamSpec};
use pipeline::registry::{Category, Settings, StageDesc};

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

/// The channels a span starts on.
///
/// At 2.4 GHz the channels overlap on a 5 MHz grid, so reading all thirteen
/// is reading each transmission three times over for three times the work.
/// Access points sit on 1, 6, 11 and, where it is allowed, 13, and a network
/// anywhere else announces where it is in every beacon it sends, which is
/// heard on the neighbouring channels anyway. So: start on the four that
/// cover the band, and open the others when something says to. The 5 GHz
/// channels do not overlap and are all here.
pub fn starting_channels() -> Vec<f64> {
    let mut v: Vec<f64> =
        [1u8, 6, 11, 13, 14].iter().filter_map(|&n| dsp::wifi::channel_2ghz(n)).collect();
    v.extend(dsp::wifi::channels_5ghz());
    v
}

/// Whether a packet's reported centre says it came off a Wi-Fi channel.
pub fn is_wifi_channel(center_hz: f64) -> bool {
    channel_of(center_hz).is_some()
}

pub struct WifiNode {
    cfg: WifiConfig,
    span: Option<WifiSpan>,
    meter: crate::FrameMeter,
    frames: Vec<WifiFrame>,
    accepted: u64,
    /// Channels opened because a beacon said its network was there.
    opened: u64,
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
            span: None,
            meter: crate::FrameMeter::new(ofdm::RATE_HZ, DEFAULT_HZ as u64, KEEP_S),
            frames: Vec::new(),
            accepted: 0,
            opened: 0,
        }
    }

    /// Frames that passed their FCS since the node was built.
    pub fn accepted(&self) -> u64 {
        self.accepted
    }

    /// The channels being read, which grows as beacons name their own.
    pub fn channels(&self) -> Vec<f64> {
        self.span.as_ref().map(|s| s.channels()).unwrap_or_default()
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
        let Some(span) = WifiSpan::new(rate, center, &starting_channels(), self.cfg) else {
            return Err(common::Error::other(
                "802.11 needs a whole 20 MHz channel inside the span",
            ));
        };
        // Where the port says the frames came from: the one channel when the
        // span holds one, and the span itself when it holds several, because
        // a frame cannot then be placed by the port alone. The same rule
        // `ble_nodes` follows, and each frame carries its own channel.
        let heard = span.channels();
        let hz = match heard.as_slice() {
            [one] => *one,
            _ => center,
        };
        self.span = Some(span);
        self.meter = crate::FrameMeter::new(rate, center as u64, KEEP_S);
        let mut out = i.spec.with_kind(PortKind::Packets);
        out.center = common::Hz(hz as u64);
        out.bandwidth = CHANNEL_WIDTH_HZ;
        Ok(out)
    }

    fn process(&mut self, i: &Payload, o: &mut Payload, _c: &mut NodeCtx<'_>) -> Result<()> {
        let (Some(iq), Some(span)) = (i.as_iq(), self.span.as_mut()) else {
            return Ok(());
        };
        self.meter.feed(iq);
        self.frames.clear();
        span.process(iq, &mut self.frames);
        // A beacon says which channel its network is on. When that is a
        // channel the span holds and nothing is reading, start reading it:
        // a network on channel 3 is heard on 1 and 6 well enough to be
        // announced, and not well enough to be read.
        let claimed: Vec<f64> = self
            .frames
            .iter()
            .filter(|f| f.fcs_ok)
            .filter_map(|f| mac::parse(&f.psdu))
            .filter_map(|m| m.network.and_then(|n| n.channel))
            .filter_map(dsp::wifi::channel_2ghz)
            .collect();
        for hz in claimed {
            if let Some(s) = self.span.as_mut()
                && s.open(hz, self.cfg)
            {
                self.opened += 1;
            }
        }

        let out = o.packets_mut();
        for f in &self.frames {
            if !f.fcs_ok {
                continue;
            }
            self.accepted += 1;
            let bytes = mac::wrap(&f.psdu, f.rate.mcs, f.rate.mbps, f.rate.short_gi, f.aggregated);
            let mut pkt = crate::measured(
                f.center_hz as u64,
                CHANNEL_WIDTH_HZ as u32,
                bytes,
                f.rssi_dbfs,
                f.snr_db,
            );
            // Preamble, headers and as much of the payload as the cap allows.
            let len =
                (400 + (f.psdu.len() as f32 * 8.0 * 20.0 / f.rate.mbps) as usize).min(MAX_FRAME_IQ);
            pkt.carrier.iq = self.meter.iq_at(f.start_sample, len);
            // The FCS over the whole frame, which is what `fcs_ok` above is.
            out.push(pkt.checked(common::packet::Integrity::Passed));
        }
        Ok(())
    }

    fn reset(&mut self) {
        self.meter.reset();
        if let Some(s) = self.span.as_mut() {
            s.reset();
        }
    }
}

impl Protocol for Wifi {
    fn id(&self) -> &'static str {
        Signal::id(self)
    }
    fn label(&self) -> &'static str {
        Signal::label(self)
    }
    fn placement(&self) -> Placement {
        Signal::placement(self)
    }
    fn shape(&self) -> Shape {
        Signal::shape(self)
    }
    fn default_hz(&self) -> f64 {
        Signal::default_hz(self)
    }

    fn frame_claim(&self) -> FrameClaim {
        FrameClaim::Band { width_hz: 1_000_000 }
    }
    /// A MAC frame arrives tagged with the 20 MHz channel it was read on, and
    /// carries a CRC-32 over the whole of itself that the front end already
    /// checked.
    fn stated(&self, p: &common::packet::Packet) -> Option<Vec<common::packet::Proto>> {
        let bytes = p.bytes();
        if !is_wifi_channel(p.center_hz() as f64) {
            return None;
        }
        read(bytes, common::Hz(p.center_hz())).map(|d| vec![d])
    }

    /// Nothing is locked. A latch on a span-wide decoder hands it its whole
    /// band from the moment the span reaches it, and this band is 20 MHz of
    /// shared spectrum: 2.4 GHz holds Bluetooth and every ISM device there
    /// is, and 5.8 GHz holds the FPV video channels. Owning it would keep
    /// the detector out of all of them for a beacon.
    ///
    /// Nor a claim after a frame has decoded, which is the camera's
    /// arrangement and would say something true: the megahertz-wide runs the
    /// detector opens inside a Wi-Fi carrier are pieces of what this front
    /// end is already reading, and each of them costs an extraction and a
    /// DroneID correlator at 15.36 MS/s, since a source that wide could be a
    /// DroneID burst. Measured on the 61.44 MS/s capture of a busy 2.4 GHz
    /// band, it is still the wrong trade. Frames decode on channels 1 and 6
    /// there, so a claim would close 2402 to 2422 and 2427 to 2447 MHz: two
    /// of the eleven sources wide enough for DroneID, and twenty of the
    /// twenty-nine visits of the ExpressLRS handset hopping through the same
    /// band. The nine that cost the most sit at 2459 MHz, which is the lower
    /// two thirds of channel 11 with the span's edge through it, so this
    /// front end cannot read that channel and would never claim it.
    fn stickiness(&self) -> Stickiness {
        Stickiness::Forget
    }
    /// A fifth of the air while nothing is being read, and all of it for ten
    /// seconds after anything is.
    ///
    /// Wi-Fi is the one span-wide front end whose traffic repeats hard
    /// enough to sample: every network beacons about ten times a second, so
    /// a 200 ms window names every access point in range within a second of
    /// arriving on the channel. Reading the span costs 2.4 times real time
    /// on its own, measured, and on a band with no Wi-Fi on it that is the
    /// whole of what the receiver spends its afternoon doing. Once a frame
    /// does decode the sampling stops, because from then on what is being
    /// missed is somebody's traffic rather than the next copy of a beacon.
    fn watch(&self) -> crate::protocol::Watch {
        crate::protocol::Watch::Sampled { on_s: 0.2, every_s: 1.0, hold_s: 10.0 }
    }
    /// And nothing at all while the detector has found nothing. A frame is
    /// over 200 us at 1 Mbit/s and its access point beacons ten times a
    /// second, so anything worth reading has been on the air long enough for
    /// the detector to have a source open. The hold covers the gap between
    /// one station's frames, which is longer than the detector's own hang.
    fn wakes_on(&self) -> crate::protocol::Wake {
        crate::protocol::Wake::Detected { hold_s: 1.0 }
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
        vec![Mark { hz, width_hz: CHANNEL_WIDTH_HZ, label }]
    }
    fn chain(&self, _at: Placed) -> Vec<NodeSpec> {
        vec![NodeSpec::new("wifi")]
    }
    /// A beacon is the same news a hundred times a second. One row per
    /// network per channel is what a person wants to see; a beacon whose
    /// SSID or security changed is a different key and reports again.
    fn dedupe_key(&self, p: &common::packet::Packet) -> Option<Vec<u8>> {
        let Some(fr) = p.frame.as_ref() else {
            return None;
        };
        let f = mac::parse(&mac::Received::parse(&fr.bytes)?.mpdu)?;
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

    /// The channel a layer states it was working.
    fn channel(d: &common::packet::Proto) -> Option<common::packet::Channel> {
        d.facts.iter().find_map(|f| match f {
            common::packet::Fact::Channel(c) => Some(c.clone()),
            _ => None,
        })
    }

    use common::Hz;

    fn spec(rate: f64, center: f64) -> PortSpec {
        PortSpec { spec: StreamSpec::iq(rate, Hz(center as u64)), latency: 0 }
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
        // An RTL-SDR's whole range, and a span with no channel inside it.
        assert!(n.negotiate(&spec(2_400_000.0, 2_437_000_000.0)).is_err());
        assert!(n.negotiate(&spec(20_000_000.0, 868_000_000.0)).is_err());
        // A LimeSDR's widest, which holds a dozen channels and needs a rate
        // no decimator reaches from it.
        let out = n.negotiate(&spec(61_440_000.0, 2_457_000_000.0)).unwrap();
        assert_eq!(out.center, Hz(2_457_000_000));
    }

    /// The receiver reads a frame out of samples the transmitter in `dsp`
    /// made, through the node, and the row names the network.
    #[test]
    fn a_beacon_off_the_span_becomes_a_row_naming_the_network() {
        let want = beacon();
        let mut n = WifiNode::default();
        let out = n.negotiate(&spec(20_000_000.0, 2_437_000_000.0)).unwrap();
        assert_eq!(out.kind, PortKind::Packets);

        let mut samples = vec![common::C32::default(); 2000];
        samples.extend(dsp::wifi::tx::frame(&want, 6, 0x5d));
        samples.extend(vec![common::C32::default(); 2000]);
        let input = Payload::Iq(samples);
        let mut output = Payload::Packets(Vec::new());
        let ins = [spec(20_000_000.0, 2_437_000_000.0)];
        let (tags, mut events, mut new_tags) = (Vec::new(), Vec::new(), Vec::new());
        let mut ctx = NodeCtx::new(0, &ins, &tags, &mut events, &mut new_tags);
        n.process(&input, &mut output, &mut ctx).unwrap();
        // A frame is held for one block before it is handed on, so that a
        // copy of the same transmission heard a block later on an
        // overlapping channel is recognised rather than reported twice.
        let quiet = Payload::Iq(vec![common::C32::default(); 4096]);
        n.process(&quiet, &mut output, &mut ctx).unwrap();

        let frames = output.as_packets().expect("packets");
        assert_eq!(frames.len(), 1);
        let r = mac::Received::parse(&frames[0].bytes()).expect("an envelope");
        assert_eq!(r.mpdu, want);
        assert_eq!(r.mbps, 6);
        assert!(frames[0].carrier.rssi_dbfs.is_finite() && frames[0].carrier.snr_db.is_finite());
        assert!(frames[0].carrier.iq.is_some(), "a frame carries what it was read from");

        let d = read(frames[0].bytes(), Hz(2_437_000_000)).expect("a decode");
        assert_eq!((d.id, d.kind), ("wifi", "beacon"));
        // The network names itself, and says what protects it and which
        // channel it is really working.
        assert!(d.facts.iter().any(|f| matches!(
            f,
            common::packet::Fact::Named(n) if n.label == "waveshark" && n.fixed
        )));
        let ch = channel(&d).expect("a channel");
        assert_eq!((ch.heard, ch.claims), (6, Some(6)));
        assert_eq!(ch.secrecy, common::Secrecy::Encrypted(Some("wpa2".into())));
    }

    /// A basic id message naming a serial, which is the one field a row
    /// about an aircraft has to carry.
    fn basic_id() -> Vec<u8> {
        let mut m = vec![0x02, (1 << 4) | 2];
        m.extend_from_slice(b"1596F3AAAAAAAAAAAAAA");
        m.resize(decode::odid::MESSAGE_LEN, 0);
        m
    }

    /// An aircraft broadcasting Remote ID over Wi-Fi sends an ordinary
    /// beacon with one extra element, so the row has to be named for the
    /// aircraft rather than for the network it looks like.
    #[test]
    fn a_beacon_carrying_remote_id_is_a_row_about_the_aircraft() {
        let mut v = beacon();
        v.truncate(v.len() - 4);
        let mut ie = decode::odid::WIFI_OUI.to_vec();
        ie.push(decode::odid::WIFI_OUI_TYPE);
        ie.push(0x07); // the transmitter's message counter
        ie.extend_from_slice(&basic_id());
        v.push(221);
        v.push(ie.len() as u8);
        v.extend_from_slice(&ie);
        v.extend(dsp::wifi::crc32(&v).to_le_bytes());

        let d = read(&mac::wrap(&v, None, 6.0, false, false), Hz(2_437_000_000)).expect("a decode");
        // Named for what it is: an aircraft's broadcast, not a row about a
        // network that happens to carry some bytes. The serial is in the
        // frame, under Remote ID's own layout.
        assert_eq!(d.id, "opendroneid");
        assert!(channel(&d).is_some(), "the network it rode on is still stated");
    }

    /// A NAN service discovery frame is an action frame, which carries no
    /// elements at all, so the pack has to be found in its payload.
    #[test]
    fn a_nan_action_frame_is_a_row_about_the_aircraft() {
        let mut v = vec![0xd0, 0x00, 0x00, 0x00];
        v.extend([0x51, 0x6f, 0x9a, 0x01, 0x00, 0x00]);
        v.extend([0x02, 0x1a, 0x2b, 0x3c, 0x4d, 0x5e]);
        v.extend([0x50, 0x6f, 0x9a, 0x01, 0x00, 0xff]);
        v.extend([0x10, 0x00]);
        v.extend([0x04, 0x09]); // public action, vendor specific
        v.extend([0x50, 0x6f, 0x9a, 0x13]); // Wi-Fi Alliance, NAN

        let mut pack = vec![(0x0f << 4) | 2, decode::odid::MESSAGE_LEN as u8, 1];
        pack.extend_from_slice(&basic_id());

        let mut sda = decode::odid::NAN_SERVICE_ID.to_vec();
        sda.extend([0x01, 0x00, 0x10]);
        sda.push((1 + pack.len()) as u8);
        sda.push(0x07);
        sda.extend_from_slice(&pack);
        v.push(0x03);
        v.extend((sda.len() as u16).to_le_bytes());
        v.extend_from_slice(&sda);
        v.extend(dsp::wifi::crc32(&v).to_le_bytes());

        let d = read(&mac::wrap(&v, None, 6.0, false, false), Hz(2_437_000_000)).expect("a decode");
        assert_eq!((d.id, d.kind), ("opendroneid", "action"));
    }

    #[test]
    fn bytes_that_are_not_a_frame_are_not_a_row() {
        assert!(read(&[0u8; 20], Hz(2_437_000_000)).is_none());
        // A MAC frame with no envelope in front of it did not come from here.
        assert!(read(&beacon(), Hz(2_437_000_000)).is_none());
        let mut bad = beacon();
        bad[8] ^= 0xff;
        assert!(read(&mac::wrap(&bad, None, 6.0, false, false), Hz(2_437_000_000)).is_none());
    }

    /// The row says how the frame arrived, which is the only place that can
    /// be said: a MAC frame carries no rate inside itself.
    #[test]
    fn a_row_names_the_rate_the_frame_arrived_at() {
        let b = mac::wrap(&beacon(), Some(7), 65.0, true, true);
        let d = read(&b, Hz(2_437_000_000)).expect("a decode");
        // How the frame arrived is the keying, and the envelope the front
        // end wrote carries it; the row names the network.
        let r = mac::Received::parse(&b).expect("an envelope");
        assert_eq!(r.mcs, Some(7));
        assert!(r.aggregated);
        assert_eq!(d.id, "wifi");
    }

    /// A span starts on the channels that cover the band, and opens another
    /// when a beacon says its network is there.
    #[test]
    fn a_beacon_opens_the_channel_it_says_it_is_on() {
        let mut n = WifiNode::default();
        n.negotiate(&spec(61_440_000.0, 2_457_000_000.0)).unwrap();
        let before = n.channels();
        assert!(before.contains(&2_462_000_000.0), "{before:?}");
        assert!(!before.contains(&2_452_000_000.0), "{before:?}");

        // A beacon on channel 11 whose DS parameter set says channel 9.
        let mut want = beacon();
        // The DS parameter set's channel byte: behind the four byte check and
        // the four byte security element.
        let at = want.len() - 9;
        assert_eq!(want[at], 6, "the beacon's channel is not where it was");
        want[at] = 9;
        let crc = dsp::wifi::crc32(&want[..want.len() - 4]);
        want.truncate(want.len() - 4);
        want.extend(crc.to_le_bytes());

        let ratio = 61_440_000.0 / dsp::wifi::ofdm::RATE_HZ;
        let frame = dsp::wifi::tx::frame(&want, 6, 0x5d);
        let mut samples = vec![common::C32::default(); 2000];
        let shift = 5_000_000.0f64 / 61_440_000.0;
        samples.extend((0..(frame.len() as f64 * ratio) as usize).map(|i| {
            let x = i as f64 / ratio;
            let (a, f) = (x.floor() as usize, (x - x.floor()) as f32);
            let v =
                frame[a.min(frame.len() - 1)] * (1.0 - f) + frame[(a + 1).min(frame.len() - 1)] * f;
            let ph = std::f32::consts::TAU * shift as f32 * (i + 2000) as f32;
            v * common::C32::new(ph.cos(), ph.sin())
        }));
        samples.extend(vec![common::C32::default(); 40_000]);

        let ins = [spec(61_440_000.0, 2_457_000_000.0)];
        let (tags, mut events, mut new_tags) = (Vec::new(), Vec::new(), Vec::new());
        let mut output = Payload::Packets(Vec::new());
        for block in samples.chunks(16_384) {
            let mut ctx = NodeCtx::new(0, &ins, &tags, &mut events, &mut new_tags);
            n.process(&Payload::Iq(block.to_vec()), &mut output, &mut ctx).unwrap();
        }
        assert!(
            output.as_packets().map(|f| !f.is_empty()).unwrap_or(false),
            "the beacon did not decode at all"
        );
        assert!(
            n.channels().contains(&2_452_000_000.0),
            "channel 9 was announced and not opened: {:?}",
            n.channels()
        );
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
        PortSpec { spec: StreamSpec::iq(rate, hz), latency: 0 }
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

pub const DESC: StageDesc = StageDesc {
    name: "wifi",
    summary: "One 20 MHz 802.11a/g channel: OFDM, the legacy rates, and the MAC frame",
    category: Category::Decode,
    feeds_bus: true,
};

pub fn build(_s: &Settings) -> Result<Box<dyn pipeline::node::Node>> {
    Ok(Box::new(WifiNode::default()))
}
