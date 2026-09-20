//! ExpressLRS on the graph: the chirps of an SX1280 read as the packets of a
//! control link.
//!
//! What this node adds to the LoRa front end it shares a reader with is
//! everything below the symbols. ExpressLRS sends in implicit header mode,
//! so nothing on the air says how long a packet is or how it is coded; the
//! rate says, and the rate is measured from the sweep the classifier read
//! and the packet's own symbol count. The payload is under the SX1280's long
//! interleaved coding (`decode::lora_li`), and the packet's own CRC is seeded
//! from the binding UID, which a listener does not have.
//!
//! # Without a binding phrase
//!
//! Two ways to learn the link. A sync packet carries bytes 4 and 5 of the
//! UID in the clear and its CRC is seeded from those and nothing else, so
//! it checks itself: read the two bytes off it, seed the CRC with them, and
//! if the CRC agrees the packet is a sync packet of that link, one time in
//! 16384 aside. But the handset sends sync packets on one channel of the
//! eighty, and a span that does not reach it never sees one; the capture
//! this was written against was 20 MHz at 2415 and held two hundred and
//! forty RC packets and no sync packet at all. So the packets themselves
//! are asked: consecutive packets on one channel have CRC seeds that share
//! a high byte and count in the low one, and `decode::elrs::recover_link`
//! reads `uid[4]` back out of that through the CRC's linearity. That is as
//! much as a passing CRC on an RC packet ever showed anyway, since the
//! counter search in `decode::elrs::validate` cancels `uid[5]`.
//!
//! Either way the node holds two bytes and not six, which is why it cannot
//! follow the hop sequence and reads whatever channel it was placed on; a
//! phrase or a full UID given as a setting does both.
//!
//! # What it can predict without the sequence
//!
//! Two bytes are enough to say *where* the handset will be, if not when. The
//! eighty 2.4 GHz channels are a megahertz apart from 2400.4 MHz whatever the
//! UID is, and every one of them is keyed 812.5 kHz wide. So once a link has
//! decoded here, the node publishes a [`pipeline::Lock`] over that raster,
//! and the auto node hands it every hop the detector opens without
//! classifying any of them; see [`crate::auto`]. Which hop is next needs
//! `decode::elrs::hop_sequence`, and that needs four UID bytes, so a lock
//! carries no schedule.
//!
//! # Only one rate decodes
//!
//! The long interleaved coding is measured at SF7 and 4/8, which is the
//! 100 Hz Full and 150 Hz rates, and `decode::lora_li` refuses the rest
//! rather than guess; the packet is still reported as a chirp it could not
//! read. `decode::lora_li` says how another rate was measured.

use crate::NodeSpec;
use crate::protocol::{FrameClaim, Placed, Placement, Protocol, Shape};
use common::{C32, Result};
use decode::elrs;
pub use decode::elrs::ENVELOPE;
pub use decode::elrs::SPREADING_FACTORS;
pub use decode::elrs::TAG;
pub use decode::elrs::hex;
pub use decode::elrs::read;
pub use decode::elrs::{CODING_RATE, RECOVER_FROM};
use dsp::lora::{ChirpReader, Found};
use identify::Signal;
pub use identify::elrs::CHANNEL_WIDTH_HZ;
pub use identify::elrs::Elrs;
use pipeline::event::Request;
use pipeline::lock::{Lock, Raster};
use pipeline::node::{NodeCtx, PortSpec, Simple};
use pipeline::param::{Param, ParamValue};
use pipeline::port::{Payload, PortKind, StreamSpec};
use pipeline::registry::{Category, Settings, SettingsExt, StageDesc};

/// How much of a channel a source may measure and still be one of this
/// link's visits: a chirp fills its channel, a source measures it a little
/// over, and a strong one up to twice. What [`Elrs::accepts_width`] tests
/// and what a lock claims on, which are the same question.
const WIDTH_SHARE: (f64, f64) = (0.7, 2.0);

/// How far off a channel of the hop set a source may be measured and still
/// be that channel.
///
/// A source is the power centroid of the bins that stood over the floor, so
/// it lands near a channel rather than on it, and always a little above:
/// measured on the 61.44 MS/s capture of one handset, the twenty-nine
/// visits sat 9 to 195 kHz above their channel. A quarter of the megahertz
/// spacing covers that with margin and leaves no source that two channels
/// could both claim.
const RASTER_TOLERANCE_HZ: f64 = 250_000.0;

pub struct ElrsNode {
    reader: ChirpReader,
    /// The binding UID, when one was given: the whole of it from a phrase
    /// or twelve hex digits, or bytes 4 and 5 learned from a sync packet.
    uid: Option<[u8; 6]>,
    /// Whether `uid` is the whole UID, and so can generate the hop sequence.
    uid_whole: bool,
    ota_version: u8,
    decoded: u64,
    /// Packets whose symbols were read but did not decode as this link's.
    refused: u64,
    /// Packets read before the link was known, in the order they came,
    /// for the link to be recovered from. Consecutive on one channel,
    /// since this node reads one source.
    /// Packets held back until the link is known, with what each was heard
    /// at: a packet published later was still received when it was received,
    /// and a row with no level is a row nobody can act on.
    unplaced: Vec<(Vec<u8>, f32, f32)>,
    /// Whether the lock over the hop set has been published. Once, on the
    /// first packet that decodes: before that there is no link to lock on,
    /// and after it the statement does not change.
    published: bool,
}

impl Default for ElrsNode {
    fn default() -> Self {
        Self::new(None)
    }
}

impl ElrsNode {
    pub fn new(uid: Option<[u8; 6]>) -> Self {
        Self {
            reader: ChirpReader::new(),
            uid_whole: uid.is_some(),
            uid,
            ota_version: elrs::OTA_VERSION,
            decoded: 0,
            refused: 0,
            unplaced: Vec::new(),
            published: false,
        }
    }

    /// What this node can predict about the link it has read: which eighty
    /// channels it will be on and how wide each visit is.
    ///
    /// Published once a packet of the link has decoded, which is the whole
    /// of the evidence there is that a link is on the air here. Two UID
    /// bytes are enough for this and not for the hop sequence, so the lock
    /// says where and not when.
    fn lock(&self, uid: [u8; 6]) -> Lock {
        let link = format!("{:02x}{:02x}", uid[4], uid[5]);
        let mut settings = Settings::new();
        settings.insert(LINK.into(), ParamValue::Text(link.clone()));
        Lock {
            transmitter: link,
            raster: Raster {
                start_hz: elrs::FREQ_START_HZ as f64,
                step_hz: (elrs::FREQ_STOP_HZ - elrs::FREQ_START_HZ) as f64
                    / (elrs::CHANNEL_COUNT - 1) as f64,
                count: elrs::CHANNEL_COUNT,
            },
            tolerance_hz: RASTER_TOLERANCE_HZ,
            width_hz: CHANNEL_WIDTH_HZ,
            width_share: WIDTH_SHARE,
            confidence: 1.0,
            settings,
        }
    }

    pub fn read(&self) -> u64 {
        self.decoded
    }

    pub fn refused(&self) -> u64 {
        self.refused
    }

    /// The UID being checked against, and whether all of it is known.
    pub fn uid(&self) -> Option<([u8; 6], bool)> {
        self.uid.map(|u| (u, self.uid_whole))
    }

    /// Learn the link from a sync packet that checks itself, or from
    /// enough consecutive packets of it, when no UID was given.
    fn learn(&mut self, packet: &[u8], rssi_dbfs: f32, snr_db: f32) {
        if self.uid.is_some() {
            return;
        }
        if let Some(uid) = elrs::uid_from_sync(packet, self.ota_version) {
            self.uid = Some(uid);
            self.uid_whole = false;
            self.unplaced.clear();
            return;
        }
        self.unplaced.push((packet.to_vec(), rssi_dbfs, snr_db));
        if self.unplaced.len() < RECOVER_FROM {
            return;
        }
        let last: Vec<&[u8]> = self.unplaced[self.unplaced.len() - RECOVER_FROM..]
            .iter()
            .map(|(p, _, _)| &p[..])
            .collect();
        if let Some(r) = elrs::recover_link(&last, self.ota_version) {
            self.uid = Some([0, 0, 0, 0, r.uid4, r.uid5.unwrap_or(0)]);
            self.uid_whole = false;
        } else {
            // A window that slides, so a stray packet of another link on
            // the channel costs one packet's wait rather than three.
            self.unplaced.remove(0);
        }
    }
}

impl Simple for ElrsNode {
    fn name(&self) -> &str {
        "elrs"
    }

    fn negotiate(&mut self, i: &PortSpec) -> Result<StreamSpec> {
        if i.spec.kind != PortKind::Iq {
            return Err(common::Error::other("elrs reads complex baseband"));
        }
        let center_hz = i.spec.center.as_f64();
        if !(2_400e6..=2_500e6).contains(&center_hz) {
            return Err(common::Error::other(
                "elrs reads the 2.4 GHz link only: the 900 MHz coding has not been measured",
            ));
        }
        self.reader.design(i.spec.rate, center_hz, CHANNEL_WIDTH_HZ, SPREADING_FACTORS, true)?;
        let mut out = i.spec.with_kind(PortKind::Packets);
        out.bandwidth = CHANNEL_WIDTH_HZ;
        out.rate = 0.0;
        Ok(out)
    }

    fn process(&mut self, i: &Payload, o: &mut Payload, c: &mut NodeCtx<'_>) -> Result<()> {
        let Some(iq) = i.as_iq() else { return Ok(()) };
        self.reader.feed(iq);
        while let Some(Found { packet, samples, rssi_dbfs, snr_db }) = self.reader.next() {
            let Some(bytes) = elrs::payload(&packet.symbols, packet.sf) else {
                self.refused += 1;
                c.warn(format!(
                    "SF{}: {} symbols that are not a packet at a rate this reads",
                    packet.sf,
                    packet.symbols.len()
                ));
                continue;
            };
            let known = self.uid.is_some();
            self.learn(&bytes, rssi_dbfs, snr_db);
            let Some(uid) = self.uid else {
                self.refused += 1;
                continue;
            };
            // The link was just learned from the packets held back, and
            // they were this link's: they leave now, in order, the last
            // of them being this one.
            let mut ready: Vec<(Vec<u8>, Option<Vec<C32>>, f32, f32)> = Vec::new();
            if !known {
                let held = std::mem::take(&mut self.unplaced);
                let n = held.len();
                for (k, (p, rssi, snr)) in held.into_iter().enumerate() {
                    if k + 1 < n && p != bytes {
                        ready.push((p, None, rssi, snr));
                    }
                }
            }
            ready.push((bytes, Some(samples), rssi_dbfs, snr_db));
            for (bytes, samples, rssi_dbfs, snr_db) in ready {
                let Some(d) = elrs::decode(&bytes, &uid, self.ota_version) else {
                    self.refused += 1;
                    continue;
                };
                self.decoded += 1;
                if !self.published {
                    self.published = true;
                    c.request(Request::Lock(self.lock(uid)));
                }
                let bus = elrs::to_bytes(
                    packet.sf,
                    CHANNEL_WIDTH_HZ,
                    self.ota_version,
                    &uid,
                    d.nonce,
                    &bytes,
                );
                let mut p = crate::measured(
                    self.reader.center_hz() as u64,
                    CHANNEL_WIDTH_HZ as u32,
                    bus,
                    rssi_dbfs,
                    snr_db,
                );
                if let Some(samples) = samples {
                    p.carrier.iq = Some(std::sync::Arc::new(common::IqBurst {
                        rate: self.reader.sample_rate(),
                        center_hz: self.reader.center_hz() as u64,
                        samples,
                    }));
                }
                o.packets_mut().push(p);
            }
        }
        Ok(())
    }

    fn reset(&mut self) {
        self.reader.reset();
    }

    fn params(&self) -> Vec<Param> {
        let uid = self
            .uid
            .map(|u| u.iter().map(|b| format!("{b:02x}")).collect::<String>())
            .unwrap_or_default();
        vec![
            Param::text(UID, uid).label("Binding UID, twelve hex digits, or blank to learn it"),
            Param::float("ota_version", f64::from(self.ota_version), 3.0..=4.0)
                .label("Firmware generation"),
        ]
    }

    fn set_param(&mut self, name: &str, v: ParamValue) -> Result<()> {
        match name {
            UID => {
                let t = v.as_str().unwrap_or("").trim().to_string();
                self.uid = parse_uid(&t);
                self.uid_whole = self.uid.is_some();
            }
            PHRASE => {
                let t = v.as_str().unwrap_or("").trim().to_string();
                self.uid = (!t.is_empty()).then(|| elrs::uid_from_phrase(&t));
                self.uid_whole = self.uid.is_some();
            }
            LINK => {
                self.uid = parse_link(v.as_str().unwrap_or(""));
                self.uid_whole = false;
            }
            "ota_version" => self.ota_version = v.as_f64().unwrap_or(4.0) as u8,
            _ => return Err(common::Error::other(format!("elrs: unknown parameter {name:?}"))),
        }
        Ok(())
    }
}

/// The two UID bytes a link is known by here, as four hex digits: what a
/// sync packet names and what the CRC seed recovers, which is as much of a
/// UID as this node ever learns off the air.
///
/// Kept apart from [`parse_uid`] because they mean different things: a whole
/// UID generates the hop sequence and a link id does not, and a node that
/// took two bytes for six would say it could follow a handset it cannot.
pub fn parse_link(text: &str) -> Option<[u8; 6]> {
    let hex: String = text.chars().filter(|c| c.is_ascii_hexdigit()).collect();
    if hex.len() != 4 {
        return None;
    }
    let hi = u8::from_str_radix(&hex[..2], 16).ok()?;
    let lo = u8::from_str_radix(&hex[2..], 16).ok()?;
    Some([0, 0, 0, 0, hi, lo])
}

/// Twelve hex digits, with or without separators, as a UID.
pub fn parse_uid(text: &str) -> Option<[u8; 6]> {
    let hex: String = text.chars().filter(|c| c.is_ascii_hexdigit()).collect();
    if hex.len() != 12 {
        return None;
    }
    let mut uid = [0u8; 6];
    for (i, b) in uid.iter_mut().enumerate() {
        *b = u8::from_str_radix(&hex[2 * i..2 * i + 2], 16).ok()?;
    }
    Some(uid)
}

impl Protocol for Elrs {
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

    /// A chirp like LoRa's, tagged by the front end with the link it checked
    /// the packet against; the check is made again in `read_frame`.
    fn frame_claim(&self) -> FrameClaim {
        FrameClaim::Tagged
    }
    fn stated(&self, p: &common::packet::Packet) -> Option<Vec<common::packet::Proto>> {
        let bytes = p.bytes();
        read(bytes).map(|d| vec![d])
    }

    fn outputs(&self) -> &'static [PortKind] {
        &[PortKind::Packets]
    }
    /// A hopping link is on a channel for a few packets and gone; the
    /// next transmission is somewhere else.
    fn stickiness(&self) -> crate::protocol::Stickiness {
        crate::protocol::Stickiness::Forget
    }
    fn accepts_width(&self, _hz: f64, source_width_hz: f64) -> bool {
        let (least, most) = WIDTH_SHARE;
        (CHANNEL_WIDTH_HZ * least..=CHANNEL_WIDTH_HZ * most).contains(&source_width_hz)
    }
    fn chain(&self, _at: Placed) -> Vec<NodeSpec> {
        vec![NodeSpec::new("elrs")]
    }
}

/// The setting names this stage reads.
const UID: &str = "uid";
const PHRASE: &str = "phrase";
const LINK: &str = "link";

pub const DESC: StageDesc = StageDesc {
    name: "elrs",
    summary: "ExpressLRS 2.4 GHz: an SX1280's chirps read as the packets of a \
              control link, the link learned from its sync packet or given \
              as a binding phrase",
    category: Category::Decode,
    feeds_bus: true,
};

pub fn build(s: &Settings) -> Result<Box<dyn pipeline::node::Node>> {
    let mut n = ElrsNode::new(parse_uid(s.str_or(UID, "")));
    let phrase = s.str_or(PHRASE, "");
    if !phrase.is_empty() {
        Simple::set_param(&mut n, PHRASE, ParamValue::Text(phrase.into()))?;
    }
    // The link a lock carries, so a decoder placed on a claimed hop starts
    // knowing it rather than recovering it again from the first packets of
    // every visit. Two bytes, and the node still knows it cannot follow the
    // sequence with them.
    let link = s.str_or(LINK, "");
    if !link.is_empty() && n.uid.is_none() {
        Simple::set_param(&mut n, LINK, ParamValue::Text(link.into()))?;
    }
    Ok(Box::new(n))
}

#[cfg(test)]
mod tests {
    use super::*;
    use common::{C32, Hz};

    const UID: [u8; 6] = [0x11, 0x22, 0x33, 0x44, 0x55, 0x66];

    /// A packet the way the handset sends it: body, then the CRC placed
    /// where the firmware puts it, then the long interleaved coding, then
    /// the chirps the other way up.
    fn packet_symbols(body: &[u8], nonce: u8) -> (Vec<u8>, Vec<u16>) {
        let bytes = elrs::build(body, &UID, elrs::OTA_VERSION, nonce);
        let symbols = decode::lora_li::encode(&bytes, 7, CODING_RATE).expect("coded");
        (bytes.to_vec(), symbols)
    }

    fn air(symbols: &[u16]) -> Vec<C32> {
        dsp::lora::modulate(7, 8, 0x12, symbols, true)
    }

    fn run(node: &mut ElrsNode, iq: &[C32], rate: f64) -> Vec<common::packet::Packet> {
        let mut s = StreamSpec::iq(rate, Hz(2_440_400_000));
        s.bandwidth = CHANNEL_WIDTH_HZ;
        let ins = [PortSpec { spec: s, latency: 0 }];
        node.negotiate(&ins[0]).unwrap();
        let tags = Vec::new();
        let mut out = Vec::new();
        // Silence after the last packet, as a source has after a burst: a
        // packet is complete once the chirps have stopped, and the reader
        // waits for that rather than read the tail of a block as a packet.
        let mut iq = iq.to_vec();
        iq.extend(std::iter::repeat_n(C32::default(), 16 * 256));
        for block in iq.chunks(4096) {
            let input = Payload::Iq(block.to_vec());
            let mut o = Payload::Packets(Vec::new());
            let (mut ev, mut nt) = (Vec::new(), Vec::new());
            let mut ctx = NodeCtx::new(0, &ins, &tags, &mut ev, &mut nt);
            node.process(&input, &mut o, &mut ctx).unwrap();
            if let Payload::Packets(p) = o {
                out.extend(p.into_iter().filter(|p| !p.bytes().is_empty()));
            }
        }
        out
    }

    /// The node learns the link from a sync packet and then reads the RC
    /// packets after it, with nothing told to it.
    #[test]
    fn a_link_is_learned_from_its_sync_packet_and_then_read() {
        let rate = CHANNEL_WIDTH_HZ * 2.0;
        let (_, sync) = packet_symbols(&[0b10, 37, 200, 6, 0x03, UID[4], UID[5]], 0);
        let (_, rc) = packet_symbols(&[0b00, 0x12, 0x34, 0x56, 0x78, 0x9a, 0x80], 201);
        let mut iq = air(&sync);
        iq.extend(air(&rc));
        iq.extend(air(&rc));
        let mut n = ElrsNode::new(None);
        let frames = run(&mut n, &iq, rate);
        assert_eq!(frames.len(), 3, "sync and two rc packets, got {}", frames.len());
        let (uid, whole) = n.uid().expect("a link");
        assert_eq!(uid[4..], UID[4..]);
        assert!(!whole, "two bytes off a sync packet are not the whole UID");
        let rows: Vec<common::packet::Proto> =
            frames.iter().filter_map(|f| read(f.bytes())).collect();
        assert_eq!(rows.len(), 3);
        // A sync packet is the handset saying which link this is; the rc
        // packets after it carry the sticks.
        assert_eq!(rows[0].kind, "sync");
        assert!(rows[1..].iter().all(|r| r.kind == "rc"));
        assert_eq!(rows[1].subject.as_ref().map(|e| e.id.to_string()).as_deref(), Some("5566"));
        // An rc packet is where the sticks are.
        assert!(rows[1].facts.iter().any(|f| matches!(f, common::packet::Fact::Control(_))));
        assert!(frames.iter().all(|f| f.carrier.rssi_dbfs.is_finite() && f.carrier.iq.is_some()));
    }

    /// An RC packet before any sync packet is a chirp of an unknown link:
    /// read, and refused rather than reported on a CRC nothing seeded.
    #[test]
    fn a_packet_of_an_unknown_link_is_not_a_row() {
        let rate = CHANNEL_WIDTH_HZ * 2.0;
        let (_, rc) = packet_symbols(&[0b00, 1, 2, 3, 4, 5, 0x80], 9);
        let mut n = ElrsNode::new(None);
        assert!(run(&mut n, &air(&rc), rate).is_empty());
        assert_eq!(n.refused(), 1);
        // Told the UID, the same packet reads.
        let mut n = ElrsNode::new(Some(UID));
        assert_eq!(run(&mut n, &air(&rc), rate).len(), 1);
    }

    /// Three RC packets in a row on one channel are enough to read the
    /// link off, sync packet or none: the capture this was written against
    /// never reached the sync channel. The packets held back come out
    /// once the link is known.
    #[test]
    fn a_link_is_recovered_from_consecutive_packets_without_a_sync() {
        let rate = CHANNEL_WIDTH_HZ * 2.0;
        let mut iq = Vec::new();
        for k in 0..4u8 {
            let (_, rc) = packet_symbols(&[0b00, k, 2, 3, 4, 5, 0x80], 120 + k);
            iq.extend(air(&rc));
        }
        let mut n = ElrsNode::new(None);
        let frames = run(&mut n, &iq, rate);
        let (uid, whole) = n.uid().expect("the link");
        assert_eq!(uid[4] & 0x3f, UID[4] & 0x3f);
        assert!(!whole);
        assert_eq!(frames.len(), 4, "the held packets come out too, got {}", frames.len());
        assert!(frames[3].carrier.iq.is_some() && frames[0].carrier.iq.is_none());
    }

    #[test]
    fn a_uid_is_read_from_hex_however_it_is_written() {
        assert_eq!(parse_uid("112233445566"), Some(UID));
        assert_eq!(parse_uid("11,22,33,44,55,66"), Some(UID));
        assert_eq!(parse_uid("1122"), None);
    }

    #[test]
    fn the_node_reads_the_band_it_is_for_only() {
        let mut n = ElrsNode::default();
        let s = StreamSpec::iq(2_000_000.0, Hz(868_000_000));
        assert!(n.negotiate(&PortSpec { spec: s, latency: 0 }).is_err());
    }
}
