//! IEEE 802.15.4 MAC frames: what a frame says once it has passed its check.
//!
//! The split is `dsp::ble`'s. Everything deciding whether a frame happened at
//! all is in `dsp::oqpsk`: the spreading code, the synchronisation header and
//! the frame check. What arrives here is a MAC frame that passed, so this is
//! a header walk and nothing else, and can be read against frames somebody
//! else captured.
//!
//! # What a frame is evidence of
//!
//! Almost nothing above the MAC is readable. Zigbee, Thread and Matter all
//! encrypt their network and application layers with a key a listener does
//! not have, and the MAC's own security flag says when the payload is
//! protected as well. What is in clear is the header: who addressed whom, in
//! which personal area network, how often, and whether the sender is a
//! coordinator announcing a network or a device asking for its data. That is
//! the same evidence the Bluetooth decoder gives.
//!
//! An address is a short one, sixteen bits assigned by the coordinator, or an
//! extended one, which is the device's EUI-64 and carries a manufacturer's
//! OUI in its top three bytes. A short address only means anything inside its
//! PAN, so it is reported with the PAN it was used in.

use common::Decoded;
use common::Value;

/// What a frame is for.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FrameType {
    Beacon,
    Data,
    Ack,
    Command,
    /// A multipurpose or fragment frame, or one from a version of the
    /// standard this does not read.
    Other(u8),
}

impl FrameType {
    pub fn from_bits(v: u16) -> Self {
        match v & 0x07 {
            0 => Self::Beacon,
            1 => Self::Data,
            2 => Self::Ack,
            3 => Self::Command,
            other => Self::Other(other as u8),
        }
    }

    pub fn name(&self) -> &'static str {
        match self {
            Self::Beacon => "beacon",
            Self::Data => "data",
            Self::Ack => "ack",
            Self::Command => "command",
            Self::Other(_) => "other",
        }
    }
}

/// An address field, which is absent, short or extended depending on the two
/// bits of the frame control field that introduce it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Address {
    /// No address field at all, which for a destination means the frame is
    /// for whoever the coordinator is.
    Absent,
    /// Sixteen bits, meaningful only inside its PAN. 0xffff is everybody.
    Short(u16),
    /// The device's EUI-64, whose top three bytes are an OUI.
    Extended(u64),
}

impl Address {
    /// Whether this names one device rather than every device listening.
    pub fn is_broadcast(&self) -> bool {
        matches!(self, Address::Short(0xffff))
    }

    /// The manufacturer's OUI of an extended address, as the six hex digits
    /// a lookup wants.
    pub fn oui(&self) -> Option<String> {
        match self {
            Address::Extended(v) => Some(format!("{:06X}", (v >> 40) & 0xff_ffff)),
            _ => None,
        }
    }
}

impl std::fmt::Display for Address {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Address::Absent => write!(f, "-"),
            Address::Short(0xffff) => write!(f, "broadcast"),
            Address::Short(v) => write!(f, "0x{v:04x}"),
            Address::Extended(v) => {
                let b = v.to_be_bytes();
                let s: Vec<String> = b.iter().map(|x| format!("{x:02X}")).collect();
                write!(f, "{}", s.join(":"))
            }
        }
    }
}

/// The MAC commands a listener sees, which are the ones that build and keep a
/// network together.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Command {
    AssociationRequest,
    AssociationResponse,
    DisassociationNotification,
    DataRequest,
    PanIdConflict,
    OrphanNotification,
    BeaconRequest,
    CoordinatorRealignment,
    GtsRequest,
    Other(u8),
}

impl Command {
    pub fn from_id(v: u8) -> Self {
        match v {
            0x01 => Self::AssociationRequest,
            0x02 => Self::AssociationResponse,
            0x03 => Self::DisassociationNotification,
            0x04 => Self::DataRequest,
            0x05 => Self::PanIdConflict,
            0x06 => Self::OrphanNotification,
            0x07 => Self::BeaconRequest,
            0x08 => Self::CoordinatorRealignment,
            0x09 => Self::GtsRequest,
            other => Self::Other(other),
        }
    }

    pub fn name(&self) -> &'static str {
        match self {
            Self::AssociationRequest => "association request",
            Self::AssociationResponse => "association response",
            Self::DisassociationNotification => "disassociation",
            Self::DataRequest => "data request",
            Self::PanIdConflict => "PAN id conflict",
            Self::OrphanNotification => "orphan notification",
            Self::BeaconRequest => "beacon request",
            Self::CoordinatorRealignment => "coordinator realignment",
            Self::GtsRequest => "GTS request",
            Self::Other(_) => "unknown command",
        }
    }
}

/// What a beacon says about the network it announces.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Superframe {
    /// How often the coordinator beacons, 15 meaning only when asked.
    pub beacon_order: u8,
    pub superframe_order: u8,
    /// Whether the sender is the PAN coordinator rather than a router
    /// beaconing on its behalf.
    pub pan_coordinator: bool,
    /// Whether the network is taking new devices, which is the closest thing
    /// 802.15.4 has to an open network.
    pub association_permit: bool,
}

/// A parsed MAC frame, without its check bytes.
#[derive(Clone, Debug, PartialEq)]
pub struct Frame {
    pub frame_type: FrameType,
    /// The MAC payload is encrypted and an auxiliary security header is
    /// present.
    pub secured: bool,
    /// The sender has more for the recipient, which is how a sleeping device
    /// is told to stay awake.
    pub pending: bool,
    pub ack_request: bool,
    pub version: u8,
    pub seq: Option<u8>,
    pub dst_pan: Option<u16>,
    pub dst: Address,
    pub src_pan: Option<u16>,
    pub src: Address,
    pub command: Option<Command>,
    pub superframe: Option<Superframe>,
    /// Whatever followed the header, which above the MAC is usually
    /// encrypted.
    pub payload: Vec<u8>,
}

fn u16le(b: &[u8], at: usize) -> Option<u16> {
    Some(u16::from_le_bytes([*b.get(at)?, *b.get(at + 1)?]))
}

/// Parse a MAC frame as `dsp::oqpsk` hands it over: the frame control field
/// onwards, without the two check bytes.
pub fn parse(mpdu: &[u8]) -> Option<Frame> {
    let fcf = u16le(mpdu, 0)?;
    let frame_type = FrameType::from_bits(fcf);
    let secured = fcf >> 3 & 1 == 1;
    let pending = fcf >> 4 & 1 == 1;
    let ack_request = fcf >> 5 & 1 == 1;
    let pan_compressed = fcf >> 6 & 1 == 1;
    let dst_mode = (fcf >> 10 & 0x03) as u8;
    let version = (fcf >> 12 & 0x03) as u8;
    let src_mode = (fcf >> 14 & 0x03) as u8;
    // A reserved addressing mode means this is not a frame laid out the way
    // the header walk below assumes.
    if dst_mode == 1 || src_mode == 1 {
        return None;
    }
    let seq = mpdu.get(2).copied();
    let mut at = 3usize;

    let address = |mode: u8, at: &mut usize| -> Option<Address> {
        Some(match mode {
            0 => Address::Absent,
            2 => {
                let v = u16le(mpdu, *at)?;
                *at += 2;
                Address::Short(v)
            }
            _ => {
                let b: [u8; 8] = mpdu.get(*at..*at + 8)?.try_into().ok()?;
                *at += 8;
                // Least significant octet first on the air, and every tool
                // prints one the other way round.
                Address::Extended(u64::from_le_bytes(b))
            }
        })
    };

    let mut dst_pan = None;
    let mut src_pan = None;
    let mut dst = Address::Absent;
    let mut src = Address::Absent;
    if frame_type != FrameType::Ack {
        if dst_mode != 0 {
            dst_pan = u16le(mpdu, at);
            at += 2;
            dst = address(dst_mode, &mut at)?;
        }
        if src_mode != 0 {
            // With the compression bit set and both addresses present, the
            // source is in the destination's PAN and sends no identifier of
            // its own.
            if !(pan_compressed && dst_mode != 0) {
                src_pan = u16le(mpdu, at);
                at += 2;
            } else {
                src_pan = dst_pan;
            }
            src = address(src_mode, &mut at)?;
        }
    }
    if at > mpdu.len() {
        return None;
    }
    // The auxiliary security header is not walked: what follows it is
    // encrypted anyway, so the header fields are the evidence either way and
    // the payload is reported as it stands.
    let body = &mpdu[at..];

    let mut command = None;
    let mut superframe = None;
    match frame_type {
        FrameType::Command if !secured => command = body.first().map(|&v| Command::from_id(v)),
        FrameType::Beacon if !secured => {
            if let Some(s) = u16le(body, 0) {
                superframe = Some(Superframe {
                    beacon_order: (s & 0x0f) as u8,
                    superframe_order: (s >> 4 & 0x0f) as u8,
                    pan_coordinator: s >> 14 & 1 == 1,
                    association_permit: s >> 15 & 1 == 1,
                });
            }
        }
        _ => {}
    }

    Some(Frame {
        frame_type,
        secured,
        pending,
        ack_request,
        version,
        seq,
        dst_pan,
        dst,
        src_pan,
        src,
        command,
        superframe,
        payload: body.to_vec(),
    })
}

impl Frame {
    /// How the sender is named where one transmitter is being followed across
    /// frames: its extended address where it sent one, otherwise the short
    /// address with the PAN it means something in.
    pub fn source_id(&self) -> Option<String> {
        match self.src {
            Address::Absent => None,
            Address::Extended(_) => Some(self.src.to_string()),
            Address::Short(v) => {
                Some(format!("{}/0x{v:04x}", self.src_pan.map(|p| format!("0x{p:04x}"))?))
            }
        }
    }

    /// The fields a log or a bus carries, in the order they are worth
    /// reading.
    pub fn fields(&self) -> Vec<(String, Value)> {
        let mut f: Vec<(String, Value)> = Vec::new();
        f.push(("type".into(), Value::Text(self.frame_type.name().into())));
        if let Some(c) = self.command {
            f.push(("command".into(), Value::Text(c.name().into())));
        }
        if let Some(s) = self.seq {
            f.push(("seq".into(), Value::Int(i64::from(s))));
        }
        if let Some(p) = self.dst_pan.or(self.src_pan) {
            f.push(("pan".into(), Value::Text(format!("0x{p:04x}"))));
        }
        if self.src != Address::Absent {
            f.push(("src".into(), Value::Text(self.src.to_string())));
        }
        if self.dst != Address::Absent {
            f.push(("dst".into(), Value::Text(self.dst.to_string())));
        }
        if let Some(oui) = self.src.oui() {
            f.push(("src_oui".into(), Value::Text(oui)));
        }
        if let Some(s) = self.superframe {
            f.push(("pan_coordinator".into(), Value::Bool(s.pan_coordinator)));
            f.push(("association_permit".into(), Value::Bool(s.association_permit)));
            f.push(("beacon_order".into(), Value::Int(i64::from(s.beacon_order))));
        }
        f.push(("secured".into(), Value::Bool(self.secured)));
        if self.ack_request {
            f.push(("ack_request".into(), Value::Bool(true)));
        }
        if self.pending {
            f.push(("pending".into(), Value::Bool(true)));
        }
        if !self.payload.is_empty() {
            f.push(("payload_bytes".into(), Value::Int(self.payload.len() as i64)));
        }
        f
    }
}

/// The decode a MAC frame becomes.
///
/// `None` when the bytes are not a frame this reads, which is how the packet
/// bus tells one from anything else that arrived on the same centre.
pub fn decoded(bytes: &[u8], center: common::Hz) -> Option<Decoded> {
    use common::Value;
    let f = parse(bytes)?;
    let mut fields = f.fields();
    let channel = channel_of(center.as_f64());
    if let Some(ch) = channel {
        fields.insert(0, ("channel".into(), Value::Int(i64::from(ch))));
    }
    let detail = fields.iter().map(|(k, v)| format!("{k}={v}")).collect::<Vec<_>>().join(" ");
    let link = common::Link {
        from: f.source_id().map(common::Party::unit),
        to: Some(if f.dst.is_broadcast() || f.dst == Address::Absent {
            common::Party::broadcast()
        } else {
            common::Party::unit(f.dst.to_string())
        }),
    };
    let mut d = Decoded::bytes("802.15.4", center, 0.0, bytes.to_vec());
    if let Some(id) = f.source_id() {
        let mut who = common::Identity::new("ieee802154", id);
        // The one durable name a listener gets: a short address is handed
        // out afresh at every association, and the OUI is in the EUI-64.
        who.vendor = f.src.oui();
        d = d.by(who);
    }
    if let Some(ch) = channel {
        // What protects the traffic is the MAC's own statement. A frame
        // without the security bit is a clear MAC header, which is not a
        // promise about the Zigbee or Thread payload above it, so an
        // unsecured frame says nothing rather than saying the network is
        // open.
        let secrecy = if f.secured {
            common::Secrecy::Encrypted(Some("802.15.4 MAC".into()))
        } else {
            common::Secrecy::Unsaid
        };
        d = d.on_channel(
            common::ChannelUse::new(
                common::ChannelPlan::Ieee802154,
                u16::from(ch),
                CHANNEL_WIDTH_HZ as u32,
            )
            .protected_by(secrecy),
        );
    }
    Some(
        d.with_link(link)
            .with_detail(detail)
            .with_fields(fields)
            .with_modulation(common::Modulation::Oqpsk)
            // Everything reaching here passed the MAC's CRC-16 in the
            // demodulator, which is a real check and not an argument from
            // plausibility.
            .with_crc(Some(true)),
    )
}

/// The channel a centre names, if it names one.
pub fn channel_of(center_hz: f64) -> Option<u8> {
    dsp::oqpsk::channels_2450()
        .into_iter()
        .find(|(_, hz)| (hz - center_hz).abs() < 500_000.0)
        .map(|(c, _)| c)
}

/// The width one channel occupies. The modulation is two megahertz wide
/// between its first nulls and the neighbouring channel is five away.
pub const CHANNEL_WIDTH_HZ: f64 = 2_000_000.0;

#[cfg(test)]
mod tests {
    use super::*;

    /// A data frame with both addresses short and the PAN compressed, which
    /// is what nearly all Zigbee traffic looks like. Bytes as Wireshark's
    /// 802.15.4 dissector reads them, without the check.
    const DATA: [u8; 11] = [0x61, 0x88, 0x2b, 0x34, 0x12, 0x01, 0x00, 0x00, 0x00, 0xaa, 0xbb];

    #[test]
    fn a_data_frame_names_both_ends_and_one_pan() {
        let f = parse(&DATA).expect("a frame");
        assert_eq!(f.frame_type, FrameType::Data);
        assert_eq!(f.seq, Some(0x2b));
        assert_eq!(f.dst_pan, Some(0x1234));
        assert_eq!(f.dst, Address::Short(0x0001));
        // Compressed: the source is in the destination's PAN and sent no
        // identifier of its own.
        assert_eq!(f.src_pan, Some(0x1234));
        assert_eq!(f.src, Address::Short(0x0000));
        assert!(f.ack_request, "the header asks for an acknowledgement");
        assert!(!f.secured);
        assert_eq!(f.payload, vec![0xaa, 0xbb]);
        assert_eq!(f.source_id().as_deref(), Some("0x1234/0x0000"));
    }

    /// A beacon request is the one frame a device sends before it belongs to
    /// anything: no source PAN, a broadcast destination, and a command byte.
    #[test]
    fn a_beacon_request_is_a_broadcast_command() {
        let mpdu = [0x03, 0x08, 0x4f, 0xff, 0xff, 0xff, 0xff, 0x07];
        let f = parse(&mpdu).expect("a frame");
        assert_eq!(f.frame_type, FrameType::Command);
        assert_eq!(f.command, Some(Command::BeaconRequest));
        assert_eq!(f.command.unwrap().name(), "beacon request");
        assert_eq!(f.dst_pan, Some(0xffff));
        assert!(f.dst.is_broadcast());
        assert_eq!(f.src, Address::Absent);
        assert_eq!(f.source_id(), None);
    }

    /// A beacon says whether the network is taking new devices, which is
    /// what a device sending the request above is listening for.
    #[test]
    fn a_beacon_says_whether_the_network_is_open() {
        // Beacon, short source, superframe 0xcfff: beacon order 15,
        // PAN coordinator, association permitted.
        let mpdu = [0x00, 0x80, 0x11, 0x34, 0x12, 0x00, 0x00, 0xff, 0xcf, 0x00, 0x00];
        let f = parse(&mpdu).expect("a frame");
        assert_eq!(f.frame_type, FrameType::Beacon);
        assert_eq!(f.src_pan, Some(0x1234));
        assert_eq!(f.src, Address::Short(0x0000));
        let s = f.superframe.expect("a superframe specification");
        assert!(s.pan_coordinator);
        assert!(s.association_permit);
        assert_eq!(s.beacon_order, 15);
    }

    /// An extended address is the device's EUI-64 and carries a
    /// manufacturer's OUI, which is the one durable identity a listener gets:
    /// the short address is handed out afresh at every association.
    #[test]
    fn an_extended_address_carries_an_oui() {
        // Data, PAN compressed, short destination and extended source.
        let mut mpdu = vec![0x41, 0xc8, 0x07, 0x34, 0x12];
        mpdu.extend_from_slice(&[0x01, 0x00]);
        // 00:12:4B:00:11:22:33:44, least significant octet first.
        mpdu.extend_from_slice(&[0x44, 0x33, 0x22, 0x11, 0x00, 0x4b, 0x12, 0x00]);
        let f = parse(&mpdu).expect("a frame");
        assert_eq!(f.src, Address::Extended(0x0012_4b00_1122_3344));
        assert_eq!(f.src.to_string(), "00:12:4B:00:11:22:33:44");
        assert_eq!(f.src.oui().as_deref(), Some("00124B"));
        assert_eq!(f.source_id().as_deref(), Some("00:12:4B:00:11:22:33:44"));
    }

    /// An acknowledgement is three bytes and names nobody, which is why the
    /// sequence number is the only thing tying it to what it answers.
    #[test]
    fn an_acknowledgement_is_a_sequence_number_and_nothing_else() {
        let f = parse(&[0x02, 0x00, 0x6a]).expect("a frame");
        assert_eq!(f.frame_type, FrameType::Ack);
        assert_eq!(f.seq, Some(0x6a));
        assert_eq!(f.src, Address::Absent);
        assert_eq!(f.dst, Address::Absent);
        assert_eq!(f.fields().len(), 3, "type, seq and whether it is secured");
    }

    /// A secured frame says so, and its payload is not read: the key belongs
    /// to the network.
    #[test]
    fn a_secured_frame_reports_that_it_is_secured_and_nothing_from_inside() {
        let mut mpdu = DATA.to_vec();
        mpdu[0] |= 0x08;
        let f = parse(&mpdu).expect("a frame");
        assert!(f.secured);
        assert_eq!(f.command, None);
        assert!(f.fields().iter().any(|(k, v)| k == "secured" && *v == Value::Bool(true)));
    }

    /// A header that runs off the end of the frame is a truncated reception,
    /// not a frame about something.
    #[test]
    fn a_header_that_overruns_is_not_a_frame() {
        assert!(parse(&[0x61, 0x88, 0x2b, 0x34]).is_none());
        assert!(parse(&[0x61]).is_none());
        // Addressing mode 1 is reserved and nothing lays a header out for it.
        assert!(parse(&[0x41, 0x84, 0x01, 0x34, 0x12, 0x01, 0x00]).is_none());
    }
}
