//! MeshCore: a LoRa mesh whose routing is in the clear and whose
//! advertisements are the whole node.
//!
//! Where Meshtastic puts a sixteen byte header in front of an encrypted body,
//! MeshCore puts a one byte header, an optional pair of transport codes and
//! the path the packet has walked, and only the payload past that is
//! enciphered. The routing is therefore always readable: what kind of packet
//! it is, whether it is flooding or following a known route, and the chain of
//! node hashes it has been through.
//!
//! The advertisement (`PAYLOAD_TYPE_ADVERT`) is not enciphered at all. It
//! carries the node's Ed25519 public key, a timestamp, a signature over both,
//! and application data holding what the node is, optionally where it is, and
//! its name. Every node broadcasts one periodically, so a receiver that reads
//! nothing else still learns the name, role and position of everything in
//! earshot.
//!
//! Layout from the project's own documentation (`docs/packet_format.md` and
//! `docs/payloads.md` at `meshcore-dev/MeshCore`, firmware v1.12.0).
//!
//! # Telling a MeshCore packet from anything else
//!
//! There is no sync word to lean on. Meshtastic marks itself with 0x2b, but
//! MeshCore uses 0x12, which is the generic private-network sync word every
//! plain LoRa device ships with, so the sync word narrows the field and
//! settles nothing. Identification is therefore structural: a header whose
//! payload version is not v1 or whose type is one of the reserved three is
//! refused, a path length that does not fit the packet is refused, and a
//! payload shorter than the fixed fields its type begins with is refused.
//!
//! That is enough to throw out most things and not enough to be sure. An
//! enciphered payload is bytes with no structure left to check, so a packet
//! from another network sharing this sync word can satisfy every rule above.
//! [`Packet::corroborated`] is the honest line: it is true when something
//! beyond the header agrees, which today means an advertisement that parses
//! in full, and a caller should present anything else as a likely rather than
//! a fact. The measured false-positive rate on random payloads is in the
//! tests.
//!
//! What would settle it outright is verifying the Ed25519 signature an advert
//! carries, which needs SHA-512 and curve arithmetic that is not in the tree.
//! Until that is here, an advert is believed on shape alone, and
//! [`Advert::signature`] is carried but unchecked.

/// The LoRa sync word MeshCore uses. Shared with every other private-network
/// LoRa device, so it is a precondition and not evidence.
pub const SYNC: u8 = 0x12;

/// Bytes of an advert before its application data: key, timestamp, signature.
const ADVERT_FIXED: usize = 32 + 4 + 64;

/// The window a timestamp has to fall in to be believed: 2020 to 2100. An
/// advert is stamped when it is sent, so anything outside this is noise that
/// happened to parse.
const TIME_LOW: u32 = 1_577_836_800;
const TIME_HIGH: u32 = 4_102_444_800;

/// How the packet is being moved through the mesh.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RouteType {
    /// Flooded, and carrying transport codes.
    TransportFlood,
    /// Flooded to everyone.
    Flood,
    /// Following the path in the header.
    Direct,
    /// Following a path, and carrying transport codes.
    TransportDirect,
}

impl RouteType {
    fn from_bits(bits: u8) -> Self {
        match bits & 0x03 {
            0 => RouteType::TransportFlood,
            1 => RouteType::Flood,
            2 => RouteType::Direct,
            _ => RouteType::TransportDirect,
        }
    }

    /// Whether this route carries the four transport code bytes.
    fn has_transport_codes(self) -> bool {
        matches!(self, RouteType::TransportFlood | RouteType::TransportDirect)
    }

    pub fn name(self) -> &'static str {
        match self {
            RouteType::TransportFlood => "transport flood",
            RouteType::Flood => "flood",
            RouteType::Direct => "direct",
            RouteType::TransportDirect => "transport direct",
        }
    }
}

/// What the payload is. The three reserved values are refused rather than
/// carried, since a reserved type is one of the better signs that this was
/// never a MeshCore packet.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PayloadType {
    Req,
    Response,
    TxtMsg,
    Ack,
    Advert,
    GrpTxt,
    GrpData,
    AnonReq,
    Path,
    Trace,
    Multipart,
    Control,
    RawCustom,
}

impl PayloadType {
    fn from_bits(bits: u8) -> Option<Self> {
        Some(match bits {
            0x00 => PayloadType::Req,
            0x01 => PayloadType::Response,
            0x02 => PayloadType::TxtMsg,
            0x03 => PayloadType::Ack,
            0x04 => PayloadType::Advert,
            0x05 => PayloadType::GrpTxt,
            0x06 => PayloadType::GrpData,
            0x07 => PayloadType::AnonReq,
            0x08 => PayloadType::Path,
            0x09 => PayloadType::Trace,
            0x0a => PayloadType::Multipart,
            0x0b => PayloadType::Control,
            0x0f => PayloadType::RawCustom,
            // 0x0c to 0x0e are reserved and nothing sends them.
            _ => return None,
        })
    }

    pub fn name(self) -> &'static str {
        match self {
            PayloadType::Req => "request",
            PayloadType::Response => "response",
            PayloadType::TxtMsg => "text",
            PayloadType::Ack => "ack",
            PayloadType::Advert => "advert",
            PayloadType::GrpTxt => "group text",
            PayloadType::GrpData => "group data",
            PayloadType::AnonReq => "anonymous request",
            PayloadType::Path => "path",
            PayloadType::Trace => "trace",
            PayloadType::Multipart => "multipart",
            PayloadType::Control => "control",
            PayloadType::RawCustom => "custom",
        }
    }

    /// The least a payload of this type can be, from the fixed fields its
    /// format begins with: the hashes, the cipher MAC, the key. A payload
    /// shorter than this was never one of these, which is most of what stands
    /// in for the sync word this protocol does not have.
    fn min_payload(self) -> usize {
        match self {
            // destination hash, source hash, cipher MAC.
            PayloadType::Req
            | PayloadType::Response
            | PayloadType::TxtMsg
            | PayloadType::Path => 4,
            // A CRC of the message it acknowledges, and nothing else.
            PayloadType::Ack => 4,
            PayloadType::Advert => ADVERT_FIXED,
            // channel hash, cipher MAC.
            PayloadType::GrpTxt | PayloadType::GrpData => 3,
            // destination hash, the sender's public key, cipher MAC.
            PayloadType::AnonReq => 35,
            PayloadType::Trace | PayloadType::Multipart | PayloadType::Control => 1,
            PayloadType::RawCustom => 0,
        }
    }

    /// Whether the payload behind this type is enciphered. The routing header
    /// in front of it is readable either way.
    pub fn is_encrypted(self) -> bool {
        matches!(
            self,
            PayloadType::Req
                | PayloadType::Response
                | PayloadType::TxtMsg
                | PayloadType::GrpTxt
                | PayloadType::GrpData
                | PayloadType::AnonReq
                | PayloadType::Path
        )
    }
}

/// One MeshCore packet, read as far as its routing header.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Packet<'a> {
    /// Payload version, 0 for the v1 format this reads.
    pub version: u8,
    pub route: RouteType,
    pub payload_type: PayloadType,
    /// The two transport codes, on the route types that carry them.
    pub transport_codes: Option<(u16, u16)>,
    /// Node hashes the packet has passed through, `hash_size` bytes each.
    pub path: &'a [u8],
    /// Bytes per path hash: 1, 2 or 3.
    pub hash_size: u8,
    pub payload: &'a [u8],
}

impl<'a> Packet<'a> {
    pub fn parse(bytes: &'a [u8]) -> Option<Self> {
        let header = *bytes.first()?;
        let route = RouteType::from_bits(header);
        let payload_type = PayloadType::from_bits((header >> 2) & 0x0f)?;
        // Only v1 exists. A packet claiming a later one is either firmware
        // from the future or, far more likely, not MeshCore at all.
        let version = (header >> 6) & 0x03;
        if version != 0 {
            return None;
        }

        let mut at = 1usize;
        let transport_codes = if route.has_transport_codes() {
            let c = bytes.get(at..at + 4)?;
            at += 4;
            Some((
                u16::from_le_bytes([c[0], c[1]]),
                u16::from_le_bytes([c[2], c[3]]),
            ))
        } else {
            None
        };

        // The path length byte is not a byte count: the low six bits are how
        // many hashes there are and the top two are one less than the size of
        // each, so a five hop path can be five, ten or fifteen bytes.
        let path_len = *bytes.get(at)?;
        at += 1;
        let hops = usize::from(path_len & 0x3f);
        let hash_size = (path_len >> 6) + 1;
        if hash_size > 3 {
            return None; // 0b11 is reserved and unsupported
        }
        let path = bytes.get(at..at + hops * usize::from(hash_size))?;
        at += path.len();

        let payload = bytes.get(at..)?;
        if payload.len() < payload_type.min_payload() {
            return None;
        }
        // An acknowledgement is a checksum and nothing else, so its length is
        // known exactly rather than only at the low end.
        if payload_type == PayloadType::Ack && payload.len() != 4 {
            return None;
        }

        Some(Packet {
            version,
            route,
            payload_type,
            transport_codes,
            path,
            hash_size,
            payload,
        })
    }

    /// How many hops the packet has taken.
    pub fn hops(&self) -> usize {
        if self.hash_size == 0 {
            return 0;
        }
        self.path.len() / usize::from(self.hash_size)
    }

    /// The advertisement this packet carries, if it is one.
    pub fn advert(&self) -> Option<Advert> {
        (self.payload_type == PayloadType::Advert).then(|| Advert::parse(self.payload)).flatten()
    }

    /// Whether something beyond the header agrees this is MeshCore.
    ///
    /// The header and the length rules are weak on their own: this protocol
    /// has no sync word of its own and an encrypted payload is bytes with no
    /// structure to check, so a stray packet from another 0x12 network can
    /// look like a plausible one of these. An advert that parses in full is
    /// the corroboration, and where there is none a reader should treat the
    /// reading as a likely rather than a fact.
    pub fn corroborated(&self) -> bool {
        self.advert().is_some()
    }
}

/// What a node says it is.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum NodeType {
    Chat,
    Repeater,
    RoomServer,
    Sensor,
    /// A role this build does not know, carried by number.
    Other(u8),
}

impl NodeType {
    fn from_flags(flags: u8) -> Self {
        // The role is a small number in the low nibble, not a set of bits:
        // 3 is a room server rather than a chat node that is also a repeater.
        match flags & 0x0f {
            1 => NodeType::Chat,
            2 => NodeType::Repeater,
            3 => NodeType::RoomServer,
            4 => NodeType::Sensor,
            other => NodeType::Other(other),
        }
    }

    pub fn name(self) -> &'static str {
        match self {
            NodeType::Chat => "chat",
            NodeType::Repeater => "repeater",
            NodeType::RoomServer => "room server",
            NodeType::Sensor => "sensor",
            NodeType::Other(_) => "node",
        }
    }
}

/// A node's advertisement of itself: who it is, what it does, and where.
///
/// Nothing here is enciphered, so this is readable on any MeshCore network
/// whether or not its channels are.
#[derive(Clone, Debug, PartialEq)]
pub struct Advert {
    /// The node's Ed25519 public key, which is also its identity.
    pub public_key: [u8; 32],
    /// When the node says it sent this, in seconds since the epoch.
    pub timestamp: u32,
    /// Ed25519 over the key, timestamp and appdata. Carried but **not
    /// verified**: there is no curve arithmetic in the tree yet.
    pub signature: [u8; 64],
    pub node_type: NodeType,
    pub latitude: Option<f64>,
    pub longitude: Option<f64>,
    pub name: Option<String>,
}

impl Advert {
    /// The node hash other packets address it by: the first byte of its key.
    pub fn hash(&self) -> u8 {
        self.public_key[0]
    }

    fn parse(payload: &[u8]) -> Option<Self> {
        if payload.len() < ADVERT_FIXED {
            return None;
        }
        let public_key: [u8; 32] = payload[..32].try_into().ok()?;
        let timestamp = u32::from_le_bytes(payload[32..36].try_into().ok()?);
        if !(TIME_LOW..TIME_HIGH).contains(&timestamp) {
            return None;
        }
        let signature: [u8; 64] = payload[36..ADVERT_FIXED].try_into().ok()?;

        let appdata = &payload[ADVERT_FIXED..];
        let mut a = Advert {
            public_key,
            timestamp,
            signature,
            node_type: NodeType::Other(0),
            latitude: None,
            longitude: None,
            name: None,
        };
        // Appdata is optional in full: a bare advert is key, time, signature.
        let Some((&flags, mut rest)) = appdata.split_first() else {
            return Some(a);
        };
        a.node_type = NodeType::from_flags(flags);

        let mut take = |n: usize| -> Option<&[u8]> {
            let (head, tail) = rest.split_at_checked(n)?;
            rest = tail;
            Some(head)
        };
        if flags & 0x10 != 0 {
            // Degrees times a million, signed: the documentation says only
            // "integer", but a southern or western node needs the sign.
            let lat = i32::from_le_bytes(take(4)?.try_into().ok()?);
            let lon = i32::from_le_bytes(take(4)?.try_into().ok()?);
            a.latitude = Some(f64::from(lat) * 1e-6);
            a.longitude = Some(f64::from(lon) * 1e-6);
            // A location outside the globe is not one.
            if !(-90.0..=90.0).contains(&a.latitude?) || !(-180.0..=180.0).contains(&a.longitude?) {
                return None;
            }
        }
        if flags & 0x20 != 0 {
            take(2)?;
        }
        if flags & 0x40 != 0 {
            take(2)?;
        }
        if flags & 0x80 != 0 {
            // The name runs to the end, and has to be text for this to have
            // been an advert at all.
            a.name = Some(std::str::from_utf8(rest).ok()?.to_owned());
        } else if !rest.is_empty() {
            // Bytes nobody claimed: the flags did not describe this packet.
            return None;
        }
        Some(a)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Build an advert packet the way a node sends one.
    fn advert_packet(flags: u8, extra: &[u8], route: u8) -> Vec<u8> {
        let mut v = vec![(0x04 << 2) | route];
        if route == 0x00 || route == 0x03 {
            v.extend_from_slice(&[1, 0, 2, 0]);
        }
        v.push(0x00); // no path
        v.extend_from_slice(&[0xab; 32]); // public key
        v.extend_from_slice(&1_700_000_000u32.to_le_bytes());
        v.extend_from_slice(&[0xcd; 64]); // signature
        v.push(flags);
        v.extend_from_slice(extra);
        v
    }

    #[test]
    fn a_header_says_how_the_packet_is_routed() {
        // Type 0x02 (text), route 0x01 (flood), version 0. The payload is a
        // destination hash, a source hash and a two byte cipher MAC, which is
        // the least a text message can be.
        let bytes = [(0x02 << 2) | 0x01, 0x00, 0xde, 0x5c, 0xaa, 0xbb];
        let p = Packet::parse(&bytes).expect("a packet");
        assert_eq!(p.payload_type, PayloadType::TxtMsg);
        assert_eq!(p.route, RouteType::Flood);
        assert_eq!(p.hops(), 0);
        assert_eq!(p.payload, &[0xde, 0x5c, 0xaa, 0xbb]);
        assert!(p.payload_type.is_encrypted());
        // Nothing past the header agrees this is MeshCore, and it says so.
        assert!(!p.corroborated());
    }

    /// The path length byte packs a hop count and a hash width, so the same
    /// count of hops is a different number of bytes at each width.
    #[test]
    fn the_path_length_encodes_both_count_and_width() {
        // An ack over three hops of two byte hashes: 0b01_000011.
        let mut bytes = vec![(0x03 << 2) | 0x02, 0x43];
        bytes.extend_from_slice(&[1, 2, 3, 4, 5, 6]);
        bytes.extend_from_slice(&[0x99, 0x88, 0x77, 0x66]);
        let p = Packet::parse(&bytes).expect("a packet");
        assert_eq!(p.hash_size, 2);
        assert_eq!(p.hops(), 3);
        assert_eq!(p.path, &[1, 2, 3, 4, 5, 6]);
        assert_eq!(p.payload, &[0x99, 0x88, 0x77, 0x66]);
    }

    #[test]
    fn transport_codes_are_read_only_where_the_route_carries_them() {
        let with = advert_packet(0x80, b"n", 0x00);
        assert_eq!(Packet::parse(&with).unwrap().transport_codes, Some((1, 2)));
        let without = advert_packet(0x80, b"n", 0x01);
        assert_eq!(Packet::parse(&without).unwrap().transport_codes, None);
    }

    #[test]
    fn an_advert_carries_the_node_its_name_and_where_it_is() {
        let mut extra = Vec::new();
        extra.extend_from_slice(&53_608_448i32.to_le_bytes());
        extra.extend_from_slice(&(-6_684_672i32).to_le_bytes());
        extra.extend_from_slice(b"Balbriggan Repeater");
        // Repeater, has location, has name.
        let bytes = advert_packet(0x02 | 0x10 | 0x80, &extra, 0x01);
        let a = Packet::parse(&bytes).unwrap().advert().expect("an advert");
        assert_eq!(a.node_type, NodeType::Repeater);
        assert_eq!(a.name.as_deref(), Some("Balbriggan Repeater"));
        assert!((a.latitude.unwrap() - 53.608448).abs() < 1e-9);
        assert!((a.longitude.unwrap() + 6.684672).abs() < 1e-9);
        assert_eq!(a.hash(), 0xab);
    }

    /// An advert with no application data is still an advert: the key, the
    /// time and the signature are the whole of what is required.
    #[test]
    fn an_advert_without_appdata_still_reads() {
        let mut bytes = advert_packet(0x00, &[], 0x01);
        bytes.truncate(bytes.len() - 1); // drop the flags byte too
        let a = Packet::parse(&bytes).unwrap().advert().expect("an advert");
        assert_eq!(a.name, None);
        assert_eq!(a.latitude, None);
    }

    #[test]
    fn a_reserved_payload_type_is_refused() {
        for t in [0x0c, 0x0d, 0x0e] {
            let bytes = [(t << 2) | 0x01, 0x00];
            assert!(Packet::parse(&bytes).is_none(), "type {t:#x} was accepted");
        }
    }

    #[test]
    fn a_later_payload_version_is_refused() {
        let bytes = [0x40 | (0x04 << 2) | 0x01, 0x00];
        assert!(Packet::parse(&bytes).is_none());
    }

    #[test]
    fn a_path_running_past_the_packet_is_refused() {
        // Twenty hops claimed, no bytes behind them.
        let bytes = [(0x02 << 2) | 0x02, 20];
        assert!(Packet::parse(&bytes).is_none());
    }

    /// The strictness that stands in for a sync word: appdata whose flags do
    /// not account for every byte is not an advert.
    #[test]
    fn an_advert_with_unclaimed_appdata_is_refused() {
        // Flags say nothing is present, yet bytes follow.
        let bytes = advert_packet(0x02, b"trailing", 0x01);
        assert!(Packet::parse(&bytes).unwrap().advert().is_none());
    }

    #[test]
    fn an_advert_with_a_name_that_is_not_text_is_refused() {
        let bytes = advert_packet(0x80, &[0xff, 0xfe], 0x01);
        assert!(Packet::parse(&bytes).unwrap().advert().is_none());
    }

    #[test]
    fn an_advert_stamped_outside_living_memory_is_refused() {
        let mut bytes = advert_packet(0x80, b"n", 0x01);
        // The timestamp sits after the header byte, path byte and key.
        let at = 2 + 32;
        bytes[at..at + 4].copy_from_slice(&5u32.to_le_bytes());
        assert!(Packet::parse(&bytes).unwrap().advert().is_none());
    }

    /// How often arbitrary bytes read as a MeshCore packet, and how often as
    /// a full advert. The first number is why [`Packet::corroborated`] exists;
    /// the second is why an advert is worth believing. Both are asserted so a
    /// change that loosens the rules shows up as a number rather than as a
    /// quietly worse packet log.
    #[test]
    fn random_payloads_rarely_read_as_meshcore_and_never_as_adverts() {
        let mut x = 0x243f_6a88_85a3_08d3u64;
        let (mut parsed, mut adverts) = (0, 0);
        const TRIES: usize = 20_000;
        for _ in 0..TRIES {
            let mut buf = [0u8; 64];
            for b in buf.iter_mut() {
                x = x.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
                *b = (x >> 56) as u8;
            }
            if let Some(p) = Packet::parse(&buf) {
                parsed += 1;
                if p.corroborated() {
                    adverts += 1;
                }
            }
        }
        // Measured at 6.9% over 200k buffers: the header and the length rules
        // together throw out most noise and cannot throw out all of it, which
        // is the whole reason `corroborated` exists. Adverts never happen by
        // chance, which is why one is worth believing.
        assert!(parsed * 10 < TRIES, "{parsed} of {TRIES} random buffers parsed");
        assert_eq!(adverts, 0, "a random buffer read as a full advert");
    }

    #[test]
    fn an_acknowledgement_is_refused_unless_it_is_exactly_a_checksum() {
        let ack = |n: usize| {
            let mut v = vec![(0x03 << 2) | 0x01, 0x00];
            v.extend(std::iter::repeat_n(0xaa, n));
            v
        };
        assert!(Packet::parse(&ack(4)).is_some(), "four bytes is the checksum");
        assert!(Packet::parse(&ack(3)).is_none());
        assert!(Packet::parse(&ack(5)).is_none());
    }

    #[test]
    fn a_payload_too_short_for_its_type_is_refused() {
        // An anonymous request begins with a 32 byte public key.
        let mut v = vec![(0x07 << 2) | 0x01, 0x00];
        v.extend(std::iter::repeat_n(0xaa, 10));
        assert!(Packet::parse(&v).is_none());
    }

    #[test]
    fn a_location_off_the_globe_is_refused() {
        let mut extra = Vec::new();
        extra.extend_from_slice(&999_000_000i32.to_le_bytes());
        extra.extend_from_slice(&0i32.to_le_bytes());
        let bytes = advert_packet(0x02 | 0x10, &extra, 0x01);
        assert!(Packet::parse(&bytes).unwrap().advert().is_none());
    }
}
