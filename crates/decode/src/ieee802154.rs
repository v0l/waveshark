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

use common::Value;
use common::packet::{Channel, Entity, Fact, Id, Link, Party, Proto};

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

/// Which edition of the standard laid the header out.
///
/// The addressing fields are walked differently for a frame the 2015 edition
/// calls its own: the PAN identifier fields are in Table 7-2 rather than in
/// the 2006 rule, the sequence number may be suppressed, information
/// elements may follow the addressing fields, and an acknowledgement may
/// carry addresses.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Version {
    Ieee2003,
    Ieee2006,
    Ieee2015,
    /// The fourth value, which no edition has assigned and which is walked
    /// the 2015 way.
    Reserved,
}

impl Version {
    pub fn from_bits(fcf: u16) -> Self {
        match fcf >> 12 & 0x03 {
            0 => Self::Ieee2003,
            1 => Self::Ieee2006,
            2 => Self::Ieee2015,
            _ => Self::Reserved,
        }
    }

    /// Whether the header is laid out the 2015 way.
    pub fn enhanced(&self) -> bool {
        matches!(self, Self::Ieee2015 | Self::Reserved)
    }

    pub fn name(&self) -> &'static str {
        match self {
            Self::Ieee2003 => "2003",
            Self::Ieee2006 => "2006",
            Self::Ieee2015 => "2015",
            Self::Reserved => "reserved",
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
    pub version: Version,
    pub seq: Option<u8>,
    /// The security level of the auxiliary header, where the frame carried
    /// one. Four and above encipher the payload; one to three authenticate
    /// it and leave it readable to anyone above this decoder.
    pub security_level: Option<u8>,
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

/// Which PAN identifier fields a header carries: Table 7-2 of
/// 802.15.4-2015 for a frame of that version, the 2006 rule for anything
/// older.
fn pan_ids_present(version: Version, compressed: bool, dst_mode: u8, src_mode: u8) -> (bool, bool) {
    let plain = (dst_mode != 0, src_mode != 0 && !compressed);
    if !version.enhanced() {
        return plain;
    }
    match (compressed, dst_mode, src_mode) {
        (true, 0, 0) => (true, false),
        (true, 0, _) | (true, _, 0) => (false, false),
        (true, 3, 3) => (false, false),
        (false, 3, 3) => (true, false),
        _ => plain,
    }
}

/// How many bytes the auxiliary security header occupies, and the security
/// level it declares.
fn aux_security(mpdu: &[u8], at: usize) -> Option<(usize, u8)> {
    let sc = *mpdu.get(at)?;
    let mut len = 1usize;
    if sc >> 5 & 1 == 0 {
        len += 4;
    }
    len += match sc >> 3 & 0x03 {
        0 => 0,
        1 => 1,
        2 => 5,
        _ => 9,
    };
    (at + len <= mpdu.len()).then_some((len, sc & 0x07))
}

/// Walk the header information elements a 2015 frame may put between its
/// addressing fields and its payload, returning where the payload starts.
fn skip_header_ies(mpdu: &[u8], mut at: usize) -> Option<usize> {
    while at < mpdu.len() {
        let ie = u16le(mpdu, at)?;
        let len = (ie & 0x7f) as usize;
        let id = (ie >> 7 & 0xff) as u8;
        at = at.checked_add(2 + len).filter(|a| *a <= mpdu.len())?;
        if id == 0x7e || id == 0x7f {
            break;
        }
    }
    Some(at)
}

/// How many bytes of message integrity code a security level puts at the end
/// of the frame, which are not payload.
fn mic_len(level: u8) -> usize {
    match level & 0x03 {
        0 => 0,
        1 => 4,
        2 => 8,
        _ => 16,
    }
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
    let version = Version::from_bits(fcf);
    let src_mode = (fcf >> 14 & 0x03) as u8;
    // A reserved addressing mode means this is not a frame laid out the way
    // the header walk below assumes.
    if dst_mode == 1 || src_mode == 1 {
        return None;
    }
    let suppressed = version.enhanced() && fcf >> 8 & 1 == 1;
    let ies = version.enhanced() && fcf >> 9 & 1 == 1;
    let (seq, mut at) = match suppressed {
        true => (None, 2usize),
        false => (Some(*mpdu.get(2)?), 3usize),
    };

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

    let (wants_dst_pan, wants_src_pan) =
        pan_ids_present(version, pan_compressed, dst_mode, src_mode);
    let mut dst_pan = None;
    let mut src_pan = None;
    if wants_dst_pan {
        dst_pan = Some(u16le(mpdu, at)?);
        at += 2;
    }
    let dst = address(dst_mode, &mut at)?;
    if wants_src_pan {
        src_pan = Some(u16le(mpdu, at)?);
        at += 2;
    }
    let src = address(src_mode, &mut at)?;
    if src != Address::Absent && src_pan.is_none() {
        src_pan = dst_pan;
    }
    if at > mpdu.len() {
        return None;
    }
    let mut security_level = None;
    if secured {
        let (len, level) = aux_security(mpdu, at)?;
        at += len;
        security_level = Some(level);
    }
    if ies {
        at = skip_header_ies(mpdu, at)?;
    }
    let end = mpdu.len().checked_sub(security_level.map_or(0, mic_len)).filter(|e| *e >= at)?;
    let body = &mpdu[at..end];

    let mut command = None;
    let mut superframe = None;
    match frame_type {
        FrameType::Command if !secured => command = body.first().map(|&v| Command::from_id(v)),
        FrameType::Beacon if !secured && !version.enhanced() => {
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
        security_level,
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
        if let Some(level) = self.security_level {
            f.push(("security_level".into(), Value::Int(i64::from(level))));
        }
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

/// What a MAC frame says.
///
/// `None` when the bytes are not a frame this reads, which is how the packet
/// bus tells one from anything else that arrived on the same centre.
pub fn read(bytes: &[u8], center: common::Hz) -> Option<Proto> {
    let f = parse(bytes)?;
    let mut p = Proto::new("ieee802154", f.frame_type.name()).between(Link {
        from: f.source_id().map(Party::unit),
        to: Some(match f.dst.is_broadcast() || f.dst == Address::Absent {
            true => Party::broadcast(),
            false => Party::unit(f.dst.to_string()),
        }),
    });
    if let Some(id) = f.source_id() {
        let mut who = Entity::new("ieee802154", Id::Text(id));
        // The one durable name a listener gets: a short address is handed
        // out afresh at every association, and the OUI is in the EUI-64.
        who.vendor = f.src.oui();
        // A short address is handed out at association and taken back, so
        // two sightings of one are not evidence of one device.
        if matches!(f.src, Address::Short(_)) {
            who = who.lasting(common::packet::Stability::Session);
        }
        p = p.by(who);
    }
    if let Some(ch) = channel_of(center.as_f64()) {
        // What protects the traffic is the MAC's own statement. A frame
        // without the security bit is a clear MAC header, which is not a
        // promise about the Zigbee or Thread payload above it, so an
        // unsecured frame says nothing rather than saying the network is
        // open.
        // Levels one to three authenticate the payload and leave it
        // readable, so only four and above are a statement that it is not.
        let secrecy = match f.security_level {
            Some(level) if level >= 4 => common::Secrecy::Encrypted(Some("802.15.4 MAC".into())),
            _ => common::Secrecy::Unsaid,
        };
        p = p.saying(Fact::Channel(
            Channel::new(common::ChannelPlan::Ieee802154, u16::from(ch), CHANNEL_WIDTH_HZ as u32)
                .protected_by(secrecy),
        ));
    }
    Some(p)
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
    /// to the network. The auxiliary header and the integrity code are
    /// walked off either end of it, so what is reported as payload is the
    /// enciphered bytes and nothing else.
    #[test]
    fn a_secured_frame_reports_its_level_and_the_length_of_what_it_hides() {
        // Security control 0x0d: level 5, key identifier mode 1, so a four
        // byte frame counter and a one byte key index follow, and level 5
        // puts a four byte integrity code at the end.
        let mut mpdu = vec![0x69, 0x88, 0x2b, 0x34, 0x12, 0x01, 0x00, 0x00, 0x00];
        mpdu.extend_from_slice(&[0x0d, 0x01, 0x00, 0x00, 0x00, 0x01]);
        mpdu.extend_from_slice(&[0xaa, 0xbb, 0xcc]);
        mpdu.extend_from_slice(&[0xde, 0xad, 0xbe, 0xef]);
        let f = parse(&mpdu).expect("a frame");
        assert!(f.secured);
        assert_eq!(f.security_level, Some(5));
        assert_eq!(f.command, None);
        assert_eq!(f.payload, vec![0xaa, 0xbb, 0xcc]);
        assert_eq!(f.dst, Address::Short(0x0001));
        assert!(f.fields().iter().any(|(k, v)| k == "secured" && *v == Value::Bool(true)));
        assert!(f.fields().iter().any(|(k, v)| k == "security_level" && *v == Value::Int(5)));
    }

    /// A frame counter that is suppressed and a key named by an extended
    /// address are nine bytes of auxiliary header rather than six, and a
    /// level of two puts eight bytes of integrity code on the end.
    #[test]
    fn an_auxiliary_header_is_as_long_as_its_control_byte_says() {
        // 0x3a: level 2, key identifier mode 3 (eight byte source and an
        // index), frame counter suppressed.
        let mut mpdu = vec![0x69, 0x88, 0x2b, 0x34, 0x12, 0x01, 0x00, 0x00, 0x00, 0x3a];
        mpdu.extend_from_slice(&[1, 2, 3, 4, 5, 6, 7, 8, 0x09]);
        mpdu.extend_from_slice(&[0xaa, 0xbb]);
        mpdu.extend_from_slice(&[0; 8]);
        let f = parse(&mpdu).expect("a frame");
        assert_eq!(f.security_level, Some(2));
        assert_eq!(f.payload, vec![0xaa, 0xbb]);
        let cut = &mpdu[..mpdu.len() - 3];
        assert!(parse(cut).is_none(), "a frame too short for its own integrity code");
    }

    /// Table 7-2 of 802.15.4-2015, a row at a time, against the 2006 rule it
    /// replaced. Walked the same way as tcpdump's `print-802_15_4.c` and
    /// Wireshark's `packet-ieee802154.c`, which is what the byte offsets in
    /// the tests below were checked against.
    #[test]
    fn the_pan_identifier_fields_are_table_7_2_for_a_version_two_frame() {
        let v2 = Version::Ieee2015;
        const NONE: u8 = 0;
        const SHORT: u8 = 2;
        const EXT: u8 = 3;
        // The rows the 2015 table changed.
        assert_eq!(pan_ids_present(v2, true, NONE, NONE), (true, false));
        assert_eq!(pan_ids_present(v2, true, SHORT, NONE), (false, false));
        assert_eq!(pan_ids_present(v2, true, NONE, SHORT), (false, false));
        assert_eq!(pan_ids_present(v2, true, EXT, EXT), (false, false));
        assert_eq!(pan_ids_present(v2, false, EXT, EXT), (true, false));
        // The rows it kept.
        assert_eq!(pan_ids_present(v2, false, NONE, NONE), (false, false));
        assert_eq!(pan_ids_present(v2, false, SHORT, NONE), (true, false));
        assert_eq!(pan_ids_present(v2, false, NONE, SHORT), (false, true));
        assert_eq!(pan_ids_present(v2, false, SHORT, SHORT), (true, true));
        assert_eq!(pan_ids_present(v2, false, SHORT, EXT), (true, true));
        assert_eq!(pan_ids_present(v2, false, EXT, SHORT), (true, true));
        assert_eq!(pan_ids_present(v2, true, SHORT, SHORT), (true, false));
        assert_eq!(pan_ids_present(v2, true, SHORT, EXT), (true, false));
        assert_eq!(pan_ids_present(v2, true, EXT, SHORT), (true, false));
        // The five combinations the two editions disagree about, read the
        // 2006 way for a 2006 frame.
        let v1 = Version::Ieee2006;
        assert_eq!(pan_ids_present(v1, true, NONE, NONE), (false, false));
        assert_eq!(pan_ids_present(v1, true, SHORT, NONE), (true, false));
        assert_eq!(pan_ids_present(v1, true, NONE, SHORT), (false, false));
        assert_eq!(pan_ids_present(v1, true, EXT, EXT), (true, false));
        assert_eq!(pan_ids_present(v1, false, EXT, EXT), (true, true));
        assert_eq!(pan_ids_present(Version::Ieee2003, false, SHORT, SHORT), (true, true));
        assert_eq!(pan_ids_present(Version::Reserved, false, EXT, EXT), (true, false));
    }

    /// Two extended addresses and a clear compression bit carry one PAN, the
    /// destination's, where the 2006 rule reads a second. Read the old way
    /// the source comes out as 0x4b00_1122_3344_5566, two bytes into its own
    /// address, which is a plausible EUI-64 and therefore silent.
    #[test]
    fn a_version_two_frame_with_two_extended_addresses_has_one_pan() {
        let mut mpdu = vec![0x01, 0xec, 0x11, 0x34, 0x12];
        mpdu.extend_from_slice(&[0x88, 0x77, 0x66, 0x55, 0x44, 0x33, 0x22, 0x11]);
        mpdu.extend_from_slice(&[0x44, 0x33, 0x22, 0x11, 0x00, 0x4b, 0x12, 0x00]);
        mpdu.extend_from_slice(&[0xaa, 0xbb]);
        let f = parse(&mpdu).expect("a frame");
        assert_eq!(f.version, Version::Ieee2015);
        assert_eq!(f.dst_pan, Some(0x1234));
        assert_eq!(f.dst, Address::Extended(0x1122_3344_5566_7788));
        assert_eq!(f.src, Address::Extended(0x0012_4b00_1122_3344));
        assert_eq!(f.src_pan, Some(0x1234));
        assert_eq!(f.payload, vec![0xaa, 0xbb]);
        assert_eq!(f.source_id().as_deref(), Some("00:12:4B:00:11:22:33:44"));
    }

    /// The same two addresses with the compression bit set carry no PAN at
    /// all, where the 2006 rule reads the first two bytes of the destination
    /// as one.
    #[test]
    fn a_version_two_frame_with_two_extended_addresses_compressed_has_no_pan() {
        let mut mpdu = vec![0x41, 0xec, 0x11];
        mpdu.extend_from_slice(&[0x88, 0x77, 0x66, 0x55, 0x44, 0x33, 0x22, 0x11]);
        mpdu.extend_from_slice(&[0x44, 0x33, 0x22, 0x11, 0x00, 0x4b, 0x12, 0x00]);
        mpdu.extend_from_slice(&[0xaa]);
        let f = parse(&mpdu).expect("a frame");
        assert_eq!(f.dst_pan, None);
        assert_eq!(f.src_pan, None);
        assert_eq!(f.dst, Address::Extended(0x1122_3344_5566_7788));
        assert_eq!(f.src, Address::Extended(0x0012_4b00_1122_3344));
        assert_eq!(f.payload, vec![0xaa]);
        assert_eq!(f.fields().iter().filter(|(k, _)| k == "pan").count(), 0);
    }

    /// One address and the compression bit set carry no PAN either, and two
    /// absent addresses with it set carry a destination PAN and nothing to
    /// go with it.
    #[test]
    fn a_version_two_frame_with_one_address_compressed_has_no_pan() {
        let one = [0x41, 0x28, 0x11, 0x01, 0x00, 0xaa, 0xbb];
        let f = parse(&one).expect("a frame");
        assert_eq!(f.dst, Address::Short(0x0001));
        assert_eq!(f.dst_pan, None);
        assert_eq!(f.src, Address::Absent);
        assert_eq!(f.payload, vec![0xaa, 0xbb]);

        let neither = [0x41, 0x20, 0x11, 0x34, 0x12, 0xcc];
        let f = parse(&neither).expect("a frame");
        assert_eq!(f.dst_pan, Some(0x1234));
        assert_eq!(f.dst, Address::Absent);
        assert_eq!(f.src, Address::Absent);
        assert_eq!(f.payload, vec![0xcc]);
    }

    /// A version two frame may leave its sequence number out, which shortens
    /// the header by the byte every older frame has there.
    #[test]
    fn a_suppressed_sequence_number_is_not_a_byte_of_the_header() {
        let mpdu = [0x61, 0xa9, 0x34, 0x12, 0x01, 0x00, 0x00, 0x00, 0xaa, 0xbb];
        assert_eq!(mpdu.len(), 10);
        let f = parse(&mpdu).expect("a frame");
        assert_eq!(f.seq, None);
        assert_eq!(f.dst_pan, Some(0x1234));
        assert_eq!(f.dst, Address::Short(0x0001));
        assert_eq!(f.src, Address::Short(0x0000));
        assert_eq!(f.payload, vec![0xaa, 0xbb]);
        assert!(f.fields().iter().all(|(k, _)| k != "seq"));
        // The same bit in a 2006 frame is reserved and says nothing.
        let older = [0x61, 0x99, 0x2b, 0x34, 0x12, 0x01, 0x00, 0x00, 0x00, 0xaa];
        let f = parse(&older).expect("a frame");
        assert_eq!(f.version, Version::Ieee2006);
        assert_eq!(f.seq, Some(0x2b));
        assert_eq!(f.payload, vec![0xaa]);
    }

    /// Header information elements sit between the addressing fields and the
    /// payload, so a frame carrying them reports six bytes of payload where
    /// the walk that does not know about them reports twelve.
    #[test]
    fn header_information_elements_are_walked_off_the_front_of_the_payload() {
        let mut mpdu = vec![0x61, 0xab, 0x34, 0x12, 0x01, 0x00, 0x00, 0x00];
        // Element 0x1a, two bytes of content.
        mpdu.extend_from_slice(&[0x02, 0x0d, 0x33, 0x44]);
        // Header termination 2: the payload follows directly.
        mpdu.extend_from_slice(&[0x80, 0x3f]);
        mpdu.extend_from_slice(&[1, 2, 3, 4, 5, 6]);
        let f = parse(&mpdu).expect("a frame");
        assert_eq!(f.seq, None);
        assert_eq!(f.dst, Address::Short(0x0001));
        assert_eq!(f.payload, vec![1, 2, 3, 4, 5, 6]);
        assert_eq!(
            f.fields().iter().find(|(k, _)| k == "payload_bytes").map(|(_, v)| v.clone()),
            Some(Value::Int(6))
        );
        // An element claiming more bytes than the frame holds is a
        // truncated reception.
        let mut short = mpdu.clone();
        short[8] = 0x40;
        assert!(parse(&short).is_none());
    }

    /// An enhanced acknowledgement carries addresses, where the three byte
    /// acknowledgement of every older version names nobody.
    #[test]
    fn an_enhanced_acknowledgement_names_who_it_is_for() {
        let mpdu = [0x02, 0x29, 0x34, 0x12, 0x01, 0x00, 0x99];
        let f = parse(&mpdu).expect("a frame");
        assert_eq!(f.frame_type, FrameType::Ack);
        assert_eq!(f.version, Version::Ieee2015);
        assert_eq!(f.seq, None);
        assert_eq!(f.dst_pan, Some(0x1234));
        assert_eq!(f.dst, Address::Short(0x0001));
        assert_eq!(f.src, Address::Absent);
        assert_eq!(f.payload, vec![0x99]);
    }

    /// An enhanced beacon carries its contents in information elements, so
    /// the two bytes where an older beacon puts its superframe specification
    /// are not one.
    #[test]
    fn an_enhanced_beacon_has_no_superframe_specification() {
        let mpdu = [0x00, 0xa0, 0x11, 0x34, 0x12, 0x00, 0x00, 0xff, 0xcf];
        let f = parse(&mpdu).expect("a frame");
        assert_eq!(f.frame_type, FrameType::Beacon);
        assert_eq!(f.version, Version::Ieee2015);
        assert_eq!(f.src_pan, Some(0x1234));
        assert_eq!(f.superframe, None);
        assert_eq!(f.payload, vec![0xff, 0xcf]);
        let older = [0x00, 0x80, 0x11, 0x34, 0x12, 0x00, 0x00, 0xff, 0xcf, 0x00, 0x00];
        assert!(parse(&older).expect("a frame").superframe.is_some());
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
