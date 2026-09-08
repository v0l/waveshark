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
//! # Only one rate decodes
//!
//! The long interleaved coding is measured at SF7 and 4/8, which is the
//! 100 Hz Full and 150 Hz rates, and `decode::lora_li` refuses the rest
//! rather than guess; the packet is still reported as a chirp it could not
//! read. `docs/protocols.md` says how to measure another.

use crate::lora_nodes::{ChirpReader, Found};
use crate::protocol::{Placed, Placement, Protocol, Shape};
use crate::NodeSpec;
use common::{Result, C32};
use decode::elrs;
use pipeline::event::{Decoded, Event};
use pipeline::node::{NodeCtx, PortSpec, Simple};
use pipeline::param::{Param, ParamValue};
use pipeline::port::{Payload, PortKind, StreamSpec};

/// The one bandwidth every ExpressLRS LoRa rate on 2.4 GHz uses.
pub const CHANNEL_WIDTH_HZ: f64 = 812_500.0;

/// The spreading factors the 2.4 GHz LoRa rates use.
const SPREADING_FACTORS: std::ops::RangeInclusive<u8> = 5..=8;

/// Coding rate denominator of the rates that decode: `CR_LI 4/8`.
const CODING_RATE: u8 = 8;

/// Tag on the bus, so a packet is recognised by its shape.
pub const TAG: [u8; 4] = *b"ELRS";

/// Bytes before the packet on the bus: the tag, the spreading factor, the
/// bandwidth in kilohertz, the firmware generation, the two UID bytes the
/// CRC was checked with, and the counter it took, 0xff for none.
const ENVELOPE: usize = 11;

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
    unplaced: Vec<Vec<u8>>,
}

/// Packets held back for the link to be recovered from. Two settle the
/// high byte; a third is asked for so a stray packet of another link on
/// the same channel does not pass for agreement.
const RECOVER_FROM: usize = 3;

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
        }
    }

    pub fn decoded(&self) -> u64 {
        self.decoded
    }

    pub fn refused(&self) -> u64 {
        self.refused
    }

    /// The UID being checked against, and whether all of it is known.
    pub fn uid(&self) -> Option<([u8; 6], bool)> {
        self.uid.map(|u| (u, self.uid_whole))
    }

    /// Read a packet's bytes out of its symbols, by the length the symbol
    /// count says: a Full rate is thirteen bytes, the rest eight.
    fn payload(symbols: &[u16], sf: u8) -> Option<Vec<u8>> {
        let n = symbols.len();
        let len = if n >= decode::lora_li::symbol_count(sf, elrs::PACKET_LEN_FULL) {
            elrs::PACKET_LEN_FULL
        } else if n >= decode::lora_li::symbol_count(sf, elrs::PACKET_LEN) {
            elrs::PACKET_LEN
        } else {
            return None;
        };
        decode::lora_li::decode(symbols, sf, CODING_RATE, len).map(|d| d.bytes)
    }

    /// Learn the link from a sync packet that checks itself, or from
    /// enough consecutive packets of it, when no UID was given.
    fn learn(&mut self, packet: &[u8]) {
        if self.uid.is_some() {
            return;
        }
        if let Some(uid) = uid_from_sync(packet, self.ota_version) {
            self.uid = Some(uid);
            self.uid_whole = false;
            self.unplaced.clear();
            return;
        }
        self.unplaced.push(packet.to_vec());
        if self.unplaced.len() < RECOVER_FROM {
            return;
        }
        let last: Vec<&[u8]> =
            self.unplaced[self.unplaced.len() - RECOVER_FROM..].iter().map(|p| &p[..]).collect();
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

/// The UID bytes a sync packet names, when its CRC agrees that it is one.
fn uid_from_sync(packet: &[u8], ota_version: u8) -> Option<[u8; 6]> {
    let sync = match packet.len() {
        n if n >= elrs::PACKET_LEN_FULL => elrs::parse_full(packet),
        n if n >= elrs::PACKET_LEN => elrs::parse(packet),
        _ => None,
    }?;
    let elrs::Packet::Sync(s) = sync else { return None };
    let uid = [0, 0, 0, 0, s.uid45[0], s.uid45[1]];
    let checks = if packet.len() >= elrs::PACKET_LEN_FULL {
        elrs::validate_full(packet, &uid, ota_version).is_some()
    } else {
        elrs::validate(packet, &uid, ota_version).is_some()
    };
    checks.then_some(uid)
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
        self.reader
            .design(i.spec.rate, center_hz, CHANNEL_WIDTH_HZ, SPREADING_FACTORS, true)?;
        let mut out = i.spec.with_kind(PortKind::Packets);
        out.bandwidth = CHANNEL_WIDTH_HZ;
        out.rate = 0.0;
        Ok(out)
    }

    fn process(&mut self, i: &Payload, o: &mut Payload, c: &mut NodeCtx<'_>) -> Result<()> {
        let Some(iq) = i.as_iq() else { return Ok(()) };
        self.reader.feed(iq);
        while let Some(Found { packet, samples, rssi_dbfs, snr_db }) = self.reader.next() {
            let Some(bytes) = Self::payload(&packet.symbols, packet.sf) else {
                self.refused += 1;
                c.emit(Event::Warning {
                    stage: "elrs".into(),
                    message: format!(
                        "SF{}: {} symbols that are not a packet at a rate this reads",
                        packet.sf,
                        packet.symbols.len()
                    ),
                });
                continue;
            };
            let known = self.uid.is_some();
            self.learn(&bytes);
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
                for (k, p) in held.into_iter().enumerate() {
                    if k + 1 < n && p != bytes {
                        ready.push((p, None, f32::NAN, f32::NAN));
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
                let bus =
                    to_bytes(packet.sf, CHANNEL_WIDTH_HZ, self.ota_version, &uid, d.nonce, &bytes);
                let mut f = common::Frame::measured(bus, rssi_dbfs, snr_db)
                    .at(self.reader.center_hz() as u64);
                if let Some(samples) = samples {
                    f = f.with_iq(std::sync::Arc::new(common::IqBurst {
                        rate: self.reader.sample_rate(),
                        center_hz: self.reader.center_hz() as u64,
                        samples,
                    }));
                }
                o.packets_mut().push(common::Packet::of_frame(now_us(), CHANNEL_WIDTH_HZ as u32, f));
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
            Param::text("uid", uid).label("Binding UID, twelve hex digits, or blank to learn it"),
            Param::float("ota_version", f64::from(self.ota_version), 3.0..=4.0)
                .label("Firmware generation"),
        ]
    }

    fn set_param(&mut self, name: &str, v: ParamValue) -> Result<()> {
        match name {
            "uid" => {
                let t = v.as_str().unwrap_or("").trim().to_string();
                self.uid = parse_uid(&t);
                self.uid_whole = self.uid.is_some();
            }
            "phrase" => {
                let t = v.as_str().unwrap_or("").trim().to_string();
                self.uid = (!t.is_empty()).then(|| elrs::uid_from_phrase(&t));
                self.uid_whole = self.uid.is_some();
            }
            "ota_version" => self.ota_version = v.as_f64().unwrap_or(4.0) as u8,
            _ => return Err(common::Error::other(format!("elrs: unknown parameter {name:?}"))),
        }
        Ok(())
    }
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

/// The bytes a packet travels as on the bus.
fn to_bytes(sf: u8, bw: f64, ota: u8, uid: &[u8; 6], nonce: Option<u8>, packet: &[u8]) -> Vec<u8> {
    let mut out = Vec::with_capacity(ENVELOPE + packet.len());
    out.extend_from_slice(&TAG);
    out.push(sf);
    out.extend_from_slice(&((bw / 1e3).round() as u16).to_le_bytes());
    out.push(ota);
    out.extend_from_slice(&uid[4..6]);
    out.push(nonce.unwrap_or(0xff));
    out.extend_from_slice(packet);
    out
}

/// One packet off the bus as a row: which link, what kind of packet, and
/// what it carried. Checked again against the UID bytes it travelled with,
/// so a row is never taken on the front end's word alone.
pub fn elrs_decoded(bytes: &[u8], center: common::Hz) -> Option<Decoded> {
    use common::Value;
    if bytes.len() < ENVELOPE + elrs::PACKET_LEN || bytes[..4] != TAG {
        return None;
    }
    let sf = bytes[4];
    let khz = u16::from_le_bytes([bytes[5], bytes[6]]);
    let ota = bytes[7];
    let uid = [0, 0, 0, 0, bytes[8], bytes[9]];
    let packet = &bytes[ENVELOPE..];
    if !SPREADING_FACTORS.contains(&sf) || khz == 0 {
        return None;
    }
    let d = elrs::decode(packet, &uid, ota)?;
    let kind = match &d.packet {
        elrs::Packet::Rc { .. } | elrs::Packet::RcFull { .. } => "rc",
        elrs::Packet::Sync(_) => "sync",
        elrs::Packet::Data { .. } => "data",
        elrs::Packet::Unknown(_) => "unknown",
    };
    let link_id = format!("{:02x}{:02x}", uid[4], uid[5]);
    let mut fields: Vec<(String, Value)> = vec![
        ("spreading_factor".into(), Value::Int(i64::from(sf))),
        ("bandwidth_hz".into(), Value::Float(f64::from(khz) * 1e3)),
        ("link".into(), Value::Text(link_id.clone())),
        ("full".into(), Value::Bool(packet.len() >= elrs::PACKET_LEN_FULL)),
    ];
    fields.extend(elrs::fields(&d));
    let detail = match &d.packet {
        elrs::Packet::Rc { channels, armed, .. } => format!(
            "rc {}{}",
            channels.iter().map(|c| c.to_string()).collect::<Vec<_>>().join(" "),
            if *armed { " armed" } else { "" }
        ),
        elrs::Packet::RcFull { channels, armed, .. } => format!(
            "rc {}{}",
            channels.iter().map(|c| c.to_string()).collect::<Vec<_>>().join(" "),
            if *armed { " armed" } else { "" }
        ),
        elrs::Packet::Sync(s) => format!(
            "sync {} hop {} counter {}",
            elrs::RATES_2G4.get(usize::from(s.rate_index)).map_or("unknown rate", |r| r.name),
            s.fhss_index,
            s.nonce
        ),
        elrs::Packet::Data { package_index, payload } => {
            format!("data {package_index}: {}", hex(payload))
        }
        elrs::Packet::Unknown(k) => format!("packet type {k}"),
    };
    let mut out = Decoded::bytes("ExpressLRS", center, 0.0, packet.to_vec())
        .with_modulation("CSS")
        .with_bandwidth(f64::from(khz) * 1e3)
        .with_crc(Some(true))
        .with_detail(format!("SF{sf} {kind}: {detail} link {link_id}"))
        .with_fields(fields);
    // The handset is the transmitting end and the link's UID bytes are the
    // nearest thing to its name; the model it flies is the other end.
    out.link = Some(common::Link {
        from: Some(common::Party::unit(format!("elrs {link_id}"))),
        to: Some(common::Party::unit(format!("elrs {link_id} rx"))),
    });
    out.identity = Some(common::Identity::new("elrs", link_id));
    Some(out)
}

fn hex(b: &[u8]) -> String {
    b.iter().map(|x| format!("{x:02x}")).collect()
}

fn now_us() -> u64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_micros() as u64)
        .unwrap_or(0)
}

/// ExpressLRS as the auto node knows it: on the 2.4 GHz band, a chirp
/// 812.5 kHz wide, placed once the classifier has named one.
pub struct Elrs;

impl Protocol for Elrs {
    fn id(&self) -> &'static str {
        "elrs"
    }
    fn label(&self) -> &'static str {
        "elrs"
    }
    fn placement(&self) -> Placement {
        Placement::Bands(vec![(2_400_000_000.0, 2_483_500_000.0)])
    }
    fn default_hz(&self) -> f64 {
        // The middle of the hop set.
        2_440_400_000.0
    }
    fn shape(&self) -> Shape {
        Shape {
            widths: &[CHANNEL_WIDTH_HZ],
            min_rate_hz: 0.0,
            feed_rate_hz: 0.0,
            span_wide: false,
            families: &[dsp::Modulation::Chirp],
        }
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
        // A chirp fills its channel; a source measures it a little over
        // and a strong one up to twice.
        (CHANNEL_WIDTH_HZ * 0.7..=CHANNEL_WIDTH_HZ * 2.0).contains(&source_width_hz)
    }
    fn chain(&self, _at: Placed) -> Vec<NodeSpec> {
        vec![NodeSpec::new("elrs")]
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use common::{Hz, C32};

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

    fn run(node: &mut ElrsNode, iq: &[C32], rate: f64) -> Vec<common::Frame> {
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
                out.extend(p.into_iter().filter_map(|p| match p.body {
                    common::PacketBody::Frame(f) => Some(f),
                    _ => None,
                }));
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
        let rows: Vec<Decoded> = frames
            .iter()
            .filter_map(|f| elrs_decoded(&f.bytes, Hz(f.center_hz)))
            .collect();
        assert_eq!(rows.len(), 3);
        assert!(rows[0].detail.as_deref().unwrap().contains("sync LoRa 250 Hz hop 37"), "{:?}", rows[0].detail);
        assert!(rows[1].detail.as_deref().unwrap().starts_with("SF7 rc:"), "{:?}", rows[1].detail);
        assert_eq!(rows[1].identity.as_ref().unwrap().id, "5566");
        assert!(frames.iter().all(|f| f.rssi_dbfs.is_finite() && f.iq.is_some()));
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
        assert!(frames[3].iq.is_some() && frames[0].iq.is_none());
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
