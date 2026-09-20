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

use crate::NodeSpec;
use crate::protocol::{FrameClaim, Mark, Placed, Placement, Protocol, Shape};
use common::Result;
use decode::ble as pdu;
pub use decode::ble::CHANNEL_WIDTH_HZ;
pub use decode::ble::channel_of;
pub use decode::ble::read;
use dsp::ble::{ADV_CHANNELS, BleConfig, BleDetector, BleFrame};
use identify::Signal;
pub use identify::ble::Ble;
use pipeline::node::{NodeCtx, PortSpec, Simple};
use pipeline::port::{Payload, PortKind, StreamSpec};
use pipeline::registry::{Category, Settings, StageDesc};

/// Where a receiver tunes when it cannot say which channel a frame came from.
/// The middle of the 2.4 GHz ISM band, which is not an advertising channel.
pub const BAND_CENTER_HZ: f64 = 2_441_000_000.0;

/// Whether a packet's reported centre says it came off an advertising
/// channel. The same trick `dsp::ais::is_ais_band` uses: where a frame was
/// received is evidence it already carries.
pub fn is_advertising_channel(center_hz: f64) -> bool {
    channel_of(center_hz).is_some()
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
                ));
            }
            [one] => ADV_CHANNELS.iter().find(|(c, _)| c == one).map(|(_, hz)| *hz).unwrap(),
            _ => BAND_CENTER_HZ,
        };
        self.det = det;
        self.meter = crate::FrameMeter::new(rate, hz as u64, 0.002);
        self.rate = rate;
        let mut out = i.spec.with_kind(PortKind::Packets);
        out.center = common::Hz(hz as u64);
        out.bandwidth = CHANNEL_WIDTH_HZ;
        Ok(out)
    }

    fn process(&mut self, i: &Payload, o: &mut Payload, _c: &mut NodeCtx<'_>) -> Result<()> {
        let Some(iq) = i.as_iq() else { return Ok(()) };
        self.meter.feed(iq);
        self.frames.clear();
        self.det.process(iq, &mut self.frames);
        let out = o.packets_mut();
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
            // A frame is 8 preamble bits plus the PDU at one bit a
            // microsecond, with room either side for the ramp.
            let len = ((f.pdu.len() + 12) * 8) as f64 * 1e-6 * self.meter_rate();
            let held = (len / self.meter_rate() * 1e6) as u32;
            let carrier = common::packet::Carrier::heard(
                common::packet::now_us(),
                hz,
                CHANNEL_WIDTH_HZ as u32,
                f.rssi_dbfs,
                f.snr_db,
                common::SourceId(0),
            )
            .lasting(held);
            let carrier = match self.meter.iq_at(f.start_sample, len as usize) {
                Some(iq) => carrier.with_iq(iq),
                None => carrier,
            };
            // The advertising CRC, checked against the channel's init word
            // before the PDU got here: a packet that failed it was dropped.
            out.push(
                common::packet::Packet::heard(carrier)
                    .framed(common::packet::Frame::of(f.pdu.clone()))
                    .checked(common::packet::Integrity::Passed),
            );
        }
        Ok(())
    }

    fn reset(&mut self) {
        self.meter.reset();
        self.det.reset();
    }
}

/// An advertisement, keyed as pulse timings.
///
/// The mirror of [`BleNode`]: `decode::ble::encode_adv_ind` builds the PDU,
/// `dsp::ble::encode_packet` adds the preamble, the access address, the CRC
/// and the channel's whitening, and `dsp::pulse::nrz` turns the bits into
/// timings. A bit is one microsecond exactly at 1 Mbit/s, so nothing is lost
/// to the port's integer microseconds.
///
/// The modulator behind is plain CPFSK where a real advertiser sends GFSK.
/// The shaping narrows the spectrum and the discriminator reading it does not
/// care, so a receiver decodes this; a transmitter meant for a crowded band
/// would want the Gaussian filter for the neighbours' sake.
pub struct BleTxNode {
    address: pdu::Address,
    name: String,
    channel: u8,
    keyer: crate::tx_nodes::Keyer,
    rate: f64,
}

impl Default for BleTxNode {
    fn default() -> Self {
        // A locally administered random address, which is what a device
        // advertising a name without a registered assignment uses.
        let address = pdu::Address { bytes: [0x01, 0x02, 0x03, 0x04, 0x05, 0xC0], random: true };
        Self::new(address, "waveshark", 38)
    }
}

impl BleTxNode {
    pub fn new(address: pdu::Address, name: &str, channel: u8) -> Self {
        // An advertiser rests between bursts, and at a megabit a second the
        // rest is most of the air: 20 ms, the shortest interval the
        // specification allows, is 20 000 bit times.
        let mut n = Self {
            address,
            name: name.into(),
            channel,
            keyer: crate::tx_nodes::Keyer::new(dsp::ble::BAUD, 20_000.0).resting_silent(),
            rate: 0.0,
        };
        n.reload();
        n
    }

    /// Encode what the settings now say, from the top.
    fn reload(&mut self) {
        self.keyer.load(dsp::ble::encode_packet(self.channel, &self.pdu()));
    }

    pub fn pdu(&self) -> Vec<u8> {
        pdu::encode_adv_ind(self.address, &self.name)
    }

    /// Advertisements keyed whole since the stage was built.
    pub fn sent(&self) -> u64 {
        self.keyer.passes()
    }
}

impl Simple for BleTxNode {
    fn name(&self) -> &str {
        BLE_TX.name
    }

    fn readings(&self) -> Vec<(String, String)> {
        vec![
            ("advertising".into(), format!("{} on {}", self.name, self.channel)),
            ("sent".into(), self.sent().to_string()),
        ]
    }

    fn negotiate(&mut self, i: &PortSpec) -> Result<StreamSpec> {
        if i.spec.rate <= 0.0 {
            return Err(common::Error::other("ble_tx needs a clock to key against"));
        }
        self.rate = i.spec.rate;
        let mut out = i.spec.with_kind(PortKind::Timings);
        out.flow = pipeline::port::Flow::Tx;
        out.bandwidth = CHANNEL_WIDTH_HZ;
        Ok(out)
    }

    fn process(&mut self, i: &Payload, o: &mut Payload, _c: &mut NodeCtx<'_>) -> Result<()> {
        if i.is_empty() {
            return Ok(());
        }
        o.timings_mut().extend(self.keyer.take(i.len(), self.rate));
        Ok(())
    }

    fn params(&self) -> Vec<pipeline::param::Param> {
        use pipeline::param::Param;
        vec![
            Param::text(NAME, self.name.clone()).label("Name"),
            Param::text(ADDRESS, self.address.to_string()).label("Address"),
            Param::choice(
                CHANNEL,
                ADV_CHANNELS.iter().position(|(ch, _)| *ch == self.channel).unwrap_or(1),
                ADV_CHANNELS.iter().map(|(ch, _)| ch.to_string()).collect(),
            )
            .label("Channel"),
        ]
    }

    fn set_param(&mut self, name: &str, value: pipeline::param::ParamValue) -> Result<()> {
        use pipeline::param::ParamValue;
        match name {
            NAME => {
                self.name = match value {
                    ParamValue::Text(t) => t,
                    _ => return Err(common::Error::other("ble_tx: a name is text")),
                }
            }
            ADDRESS => {
                let text = match value {
                    ParamValue::Text(t) => t,
                    _ => return Err(common::Error::other("ble_tx: an address is text")),
                };
                self.address = parse_address(&text)
                    .ok_or_else(|| common::Error::other("ble_tx: six hex bytes, most first"))?;
            }
            // A picker sends the index into the three, not the index of the
            // channel, and 0, 1 and 2 are not advertising channels.
            CHANNEL => {
                let v = value.as_i64().unwrap_or(1);
                self.channel = match ADV_CHANNELS.iter().find(|(ch, _)| i64::from(*ch) == v) {
                    Some((ch, _)) => *ch,
                    None => ADV_CHANNELS[v.clamp(0, 2) as usize].0,
                };
            }
            _ => return Err(common::Error::other(format!("ble_tx: unknown parameter {name:?}"))),
        }
        self.reload();
        Ok(())
    }
}

/// `AA:BB:CC:DD:EE:FF`, most significant byte first as every tool prints one,
/// which is the reverse of the order it goes on the air in.
fn parse_address(text: &str) -> Option<pdu::Address> {
    let mut bytes = [0u8; 6];
    let parts: Vec<&str> = text.split([':', '-']).collect();
    if parts.len() != 6 {
        return None;
    }
    for (i, p) in parts.iter().rev().enumerate() {
        bytes[i] = u8::from_str_radix(p.trim(), 16).ok()?;
    }
    // The top two bits of the most significant byte say what kind of address
    // it is; a static random one has both set.
    Some(pdu::Address { bytes, random: bytes[5] & 0xC0 != 0 })
}

impl Protocol for Ble {
    fn id(&self) -> &'static str {
        Signal::id(self)
    }
    fn label(&self) -> &'static str {
        Signal::label(self)
    }
    fn aliases(&self) -> &'static [&'static str] {
        Signal::aliases(self)
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
    /// An advertising channel is a frequency nothing else here transmits a
    /// frame from.
    fn stated(&self, p: &common::packet::Packet) -> Option<Vec<common::packet::Proto>> {
        let bytes = p.bytes();
        if !is_advertising_channel(p.center_hz() as f64) {
            return None;
        }
        Some(read(bytes, common::Hz(p.center_hz())).into_iter().collect())
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

    /// Advertising channel 38, which sits in the gap between the Wi-Fi
    /// channels and is the one of the three least often buried.

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
    /// The advertiser into the two-tone modulator. The shift is the full
    /// 500 kHz between the tones, which is the +-250 kHz deviation a
    /// 1 Mbit/s advertisement is specified at.
    fn transmit(&self) -> Option<crate::protocol::TxChain> {
        Some(crate::protocol::TxChain {
            source: NodeSpec::new(BLE_TX.name),
            modulator: NodeSpec::new(crate::mod_nodes::FSK_MOD.name)
                .f("shift_hz", 500_000.0)
                .s("rest", "silence"),
        })
    }
}

/// What the transmit side is set with.
const NAME: &str = "name";
const ADDRESS: &str = "address";
const CHANNEL: &str = "channel";

pub const DESC: StageDesc = StageDesc {
    name: "ble",
    summary: "One BLE advertising channel: GFSK at 1 Mbit/s, dewhitening and CRC-24",
    category: Category::Decode,
    feeds_bus: true,
};

pub const BLE_TX: StageDesc = StageDesc {
    name: "ble_tx",
    summary: "Advertise a name and an address on a BLE advertising channel",
    category: Category::Transmit,
    feeds_bus: false,
};

pub fn build(_s: &Settings) -> Result<Box<dyn pipeline::node::Node>> {
    Ok(Box::new(BleNode::default()))
}

pub fn build_tx(s: &Settings) -> Result<Box<dyn pipeline::node::Node>> {
    use pipeline::registry::SettingsExt;
    let mut n = BleTxNode::default();
    if let Some(a) = parse_address(s.str_or(ADDRESS, "")) {
        n.address = a;
    }
    n.name = s.str_or(NAME, "waveshark").to_string();
    n.reload();
    Ok(Box::new(n))
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

    /// Advertised by the transmit stage, modulated, and read back off the
    /// span: the PDU builder, the whitening, the CRC and the demodulator all
    /// agree or the name does not come back.
    #[test]
    fn an_advertisement_this_receiver_sent_is_one_this_receiver_reads() {
        let (rate, center) = (8_000_000.0, 2_426_000_000.0);
        // Two advertising intervals of clock, which at the 20 ms this stage
        // rests for is two packets on the air.
        let air = crate::tx_nodes::transmit_for(
            &Ble,
            rate,
            Hz(center as u64),
            0.05,
            &[(NAME, pipeline::param::ParamValue::Text("waveshark".into()))],
        );
        // The stage rests on no carrier, so most of this is silence: three
        // packets of 240 us and the rest of the clock keyed as no carrier,
        // which is every sample of it, 49 blocks of 8192.
        assert_eq!(air.len(), 401_408, "50 ms of samples at {rate}");
        let quiet = air.iter().filter(|s| s.norm() < 1e-6).count();
        assert_eq!(quiet, 395_636, "the rests are silence, not a held tone");

        // The detector needs a floor to measure a burst against, so the
        // advertisements arrive between two stretches of quiet band.
        let mut seed = 0x2545_f491_4f6c_dd1du64;
        let mut noise = |n: usize| -> Vec<common::C32> {
            (0..n)
                .map(|_| {
                    let mut r = || {
                        seed ^= seed << 13;
                        seed ^= seed >> 7;
                        seed ^= seed << 17;
                        ((seed >> 40) as f32 / 8_388_608.0 - 1.0) * 0.01
                    };
                    common::C32::new(r(), r())
                })
                .collect()
        };
        let mut rx = BleNode::default();
        rx.negotiate(&spec(rate, center)).unwrap();
        let ins = [spec(rate, center)];
        let (mut ev, mut tg) = (Vec::new(), Vec::new());
        let mut frames: Vec<Vec<u8>> = Vec::new();
        for block in [noise(20_000), air, noise(20_000)] {
            for chunk in block.chunks(8_192) {
                let mut out = Payload::Packets(Vec::new());
                let mut ctx = NodeCtx::new(0, &ins, &[], &mut ev, &mut tg);
                Simple::process(&mut rx, &Payload::Iq(chunk.to_vec()), &mut out, &mut ctx).unwrap();
                if let Payload::Packets(f) = out {
                    frames.extend(f.into_iter().map(|x| x.bytes().to_vec()));
                }
            }
        }
        // 240 us of packet and then 20 ms of rest, so 50 ms of clock carries
        // three: one at the start and one after each rest.
        assert_eq!(frames.len(), 3, "{} advertisements off the air", frames.len());
        assert_eq!(rx.accepted(), 3);
        assert_eq!(BleTxNode::default().pdu().len(), 22, "the PDU the stage builds");
        let d = read(&frames[0], Hz(center as u64)).expect("a decode");
        assert_eq!(d.id, "ble");
        let who = d.subject.as_ref().expect("the device that advertised");
        assert_eq!(who.id.to_string(), "C0:05:04:03:02:01");
        assert_eq!(who.name.as_deref(), Some("waveshark"));
    }

    /// An address is written most significant byte first and goes on the air
    /// the other way round, which is the one thing to get wrong here.
    #[test]
    fn an_address_is_read_the_way_it_is_printed() {
        let a = parse_address("C0:05:04:03:02:01").expect("six bytes");
        assert_eq!(a.bytes, [0x01, 0x02, 0x03, 0x04, 0x05, 0xC0]);
        assert_eq!(a.to_string(), "C0:05:04:03:02:01");
        assert!(a.random, "both top bits set is a static random address");
        assert!(parse_address("C0:05:04:03:02").is_none());
        assert!(parse_address("C0:05:04:03:02:ZZ").is_none());
    }

    /// Frames are tagged with the channel they arrived on, not with the
    /// tuner's centre: 2430 is where this receiver was parked and no BLE
    /// packet was ever sent there.
    #[test]
    fn frames_are_tagged_with_the_advertising_channel() {
        let mut n = BleNode::default();
        let out = n.negotiate(&spec(16_000_000.0, 2_430_000_000.0)).unwrap();
        assert_eq!(out.kind, PortKind::Packets);
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
        let d = read(&pdu, Hz(2_426_000_000)).expect("a decode");
        assert_eq!(d.id, "ble");
        assert_eq!(
            d.subject.as_ref().map(|e| e.id.to_string()).as_deref(),
            Some("6C:70:CB:EF:72:4D")
        );
        let ch = d.facts.iter().find_map(|f| match f {
            common::packet::Fact::Channel(c) => Some(c.clone()),
            _ => None,
        });
        assert_eq!(ch.map(|c| c.heard), Some(38));
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

        let d = read(&pdu, Hz(2_402_000_000)).expect("a decode");
        // Named for what it is rather than for the link layer it rode on.
        assert_eq!(d.id, "opendroneid");
        assert_eq!(
            d.subject.as_ref().map(|e| e.id.to_string()).as_deref(),
            Some("66:55:44:33:22:11")
        );
    }
}
