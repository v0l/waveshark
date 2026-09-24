use crate::ieee802154;
use common::packet::{Entity, Fact, Id, Link, Party, Proto, Stability};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FrameType {
    Data,
    Command,
    InterPan,
    Reserved,
}

impl FrameType {
    fn from_bits(fcf: u16) -> Self {
        match fcf & 0x03 {
            0 => Self::Data,
            1 => Self::Command,
            2 => Self::Reserved,
            _ => Self::InterPan,
        }
    }

    pub fn name(&self) -> &'static str {
        match self {
            Self::Data => "data",
            Self::Command => "command",
            Self::InterPan => "inter-PAN",
            Self::Reserved => "reserved",
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum KeyId {
    Data,
    Network,
    KeyTransport,
    KeyLoad,
}

impl KeyId {
    fn from_control(control: u8) -> Self {
        match control >> 3 & 0x03 {
            0 => Self::Data,
            1 => Self::Network,
            2 => Self::KeyTransport,
            _ => Self::KeyLoad,
        }
    }

    pub fn name(&self) -> &'static str {
        match self {
            Self::Data => "link key",
            Self::Network => "network key",
            Self::KeyTransport => "key-transport key",
            Self::KeyLoad => "key-load key",
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Security {
    pub key: KeyId,
    pub frame_counter: u32,
    pub source: Option<u64>,
    pub key_seq: Option<u8>,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Nwk {
    pub frame_type: FrameType,
    pub multicast: bool,
    pub end_device_initiator: bool,
    pub dst: u16,
    pub src: u16,
    pub radius: u8,
    pub seq: u8,
    pub dst_ieee: Option<u64>,
    pub src_ieee: Option<u64>,
    pub relays: Vec<u16>,
    pub security: Option<Security>,
}

impl Nwk {
    pub fn is_broadcast(&self) -> bool {
        self.dst >= 0xfff8
    }

    pub fn hop_sender(&self) -> Option<u64> {
        self.security.and_then(|s| s.source)
    }
}

const PRO: u16 = 2;

pub fn parse(payload: &[u8]) -> Option<Nwk> {
    let fcf = u16le(payload, 0)?;
    if fcf >> 2 & 0x0f != PRO {
        return None;
    }
    let frame_type = FrameType::from_bits(fcf);
    if frame_type == FrameType::InterPan || frame_type == FrameType::Reserved {
        return None;
    }
    let multicast = fcf >> 8 & 1 == 1;
    let secured = fcf >> 9 & 1 == 1;
    let source_route = fcf >> 10 & 1 == 1;
    let has_dst_ieee = fcf >> 11 & 1 == 1;
    let has_src_ieee = fcf >> 12 & 1 == 1;
    let end_device_initiator = fcf >> 13 & 1 == 1;
    let dst = u16le(payload, 2)?;
    let src = u16le(payload, 4)?;
    let radius = *payload.get(6)?;
    let seq = *payload.get(7)?;
    let mut at = 8;
    let dst_ieee = has_dst_ieee.then(|| u64le(payload, &mut at)).flatten();
    if has_dst_ieee && dst_ieee.is_none() {
        return None;
    }
    let src_ieee = has_src_ieee.then(|| u64le(payload, &mut at)).flatten();
    if has_src_ieee && src_ieee.is_none() {
        return None;
    }
    if multicast {
        at += 1;
    }
    let mut relays = Vec::new();
    if source_route {
        let count = usize::from(*payload.get(at)?);
        at += 2;
        for _ in 0..count {
            relays.push(u16le(payload, at)?);
            at += 2;
        }
    }
    let security = match secured {
        true => Some(security(payload, &mut at)?),
        false => None,
    };
    (at <= payload.len()).then_some(Nwk {
        frame_type,
        multicast,
        end_device_initiator,
        dst,
        src,
        radius,
        seq,
        dst_ieee,
        src_ieee,
        relays,
        security,
    })
}

fn security(b: &[u8], at: &mut usize) -> Option<Security> {
    let control = *b.get(*at)?;
    *at += 1;
    let frame_counter = u32::from_le_bytes(b.get(*at..*at + 4)?.try_into().ok()?);
    *at += 4;
    let source = match control >> 5 & 1 {
        1 => Some(u64le(b, at)?),
        _ => None,
    };
    let key = KeyId::from_control(control);
    let key_seq = match key {
        KeyId::Network => {
            let v = *b.get(*at)?;
            *at += 1;
            Some(v)
        }
        _ => None,
    };
    Some(Security { key, frame_counter, source, key_seq })
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Beacon {
    pub stack_profile: u8,
    pub router_capacity: bool,
    pub depth: u8,
    pub end_device_capacity: bool,
    pub extended_pan: u64,
    pub update_id: u8,
}

pub fn parse_beacon(payload: &[u8]) -> Option<Beacon> {
    if *payload.first()? != 0 {
        return None;
    }
    let profile = *payload.get(1)?;
    if u16::from(profile >> 4) != PRO {
        return None;
    }
    let caps = *payload.get(2)?;
    let mut at = 3;
    let extended_pan = u64le(payload, &mut at)?;
    Some(Beacon {
        stack_profile: profile & 0x0f,
        router_capacity: caps >> 2 & 1 == 1,
        depth: caps >> 3 & 0x0f,
        end_device_capacity: caps >> 7 & 1 == 1,
        extended_pan,
        update_id: payload.get(14).copied().unwrap_or(0),
    })
}

pub fn read(mac: &ieee802154::Frame) -> Option<Proto> {
    if mac.secured {
        return None;
    }
    match mac.frame_type {
        ieee802154::FrameType::Data => parse(&mac.payload).map(|n| nwk_layer(&n)),
        ieee802154::FrameType::Beacon => {
            mac.beacon_payload().and_then(parse_beacon).map(|b| beacon_layer(mac, &b))
        }
        _ => None,
    }
}

fn eui(v: u64) -> Entity {
    let mut who = Entity::new("zigbee", Id::Text(ieee802154::Address::Extended(v).to_string()));
    who.vendor = ieee802154::Address::Extended(v).oui();
    who
}

fn short(v: u16) -> Party {
    Party::unit(format!("0x{v:04x}"))
}

fn nwk_layer(n: &Nwk) -> Proto {
    let to = match n.is_broadcast() {
        true => Party::broadcast(),
        false => short(n.dst),
    };
    let mut p = Proto::new("zigbee", n.frame_type.name())
        .between(Link { from: Some(short(n.src)), to: Some(to) });
    if let Some(v) = n.hop_sender() {
        p = p.by(eui(v));
    } else {
        p = p
            .by(Entity::new("zigbee", Id::Text(format!("0x{:04x}", n.src)))
                .lasting(Stability::Session));
    }
    if let Some(s) = n.security {
        p = p.saying(Fact::Protected(common::Secrecy::Encrypted(Some(format!(
            "Zigbee {}",
            s.key.name()
        )))));
    }
    p
}

fn beacon_layer(mac: &ieee802154::Frame, b: &Beacon) -> Proto {
    let from = match mac.src {
        ieee802154::Address::Short(v) => Some(short(v)),
        _ => None,
    };
    Proto::new("zigbee", "beacon").between(Link { from, to: Some(Party::broadcast()) }).saying(
        Fact::Named(common::packet::Named {
            label: format!("Zigbee PAN {}", ieee802154::Address::Extended(b.extended_pan)),
            thing: common::packet::ThingKind::Station,
            state: None,
            role: Some(match mac.superframe.is_some_and(|s| s.pan_coordinator) {
                true => "coordinator",
                false => "router",
            }),
            fixed: true,
        }),
    )
}

fn u16le(b: &[u8], at: usize) -> Option<u16> {
    Some(u16::from_le_bytes([*b.get(at)?, *b.get(at + 1)?]))
}

fn u64le(b: &[u8], at: &mut usize) -> Option<u64> {
    let v = u64::from_le_bytes(b.get(*at..*at + 8)?.try_into().ok()?);
    *at += 8;
    Some(v)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn mpdu(hex: &str) -> Vec<u8> {
        (0..hex.len()).step_by(2).map(|i| u8::from_str_radix(&hex[i..i + 2], 16).unwrap()).collect()
    }

    fn nwk(hex: &str) -> Nwk {
        let mac = ieee802154::parse(&mpdu(hex)).expect("a MAC frame");
        parse(&mac.payload).expect("a Zigbee network header")
    }

    const LINK_STATUS: &str = "4188c5cb7affff97080912fcff97080161fffffdd40138c1a42817fd8200fffffdd40138c1a4004aae10b35196791aec016d5341";
    const RELAYED: &str = "6188c7cb7a00009708091a00000d0a1d87194186feffed2cc0ffffb9f50138c1a42819fd8200fffffdd40138c1a400fea457ea5fd86d0572";
    const END_DEVICE: &str = "6188accb7a0000641a48220000641a1e2728ad5c080048dd86feff81a094002632556f2aa8d0bff692eb619e3d8df1f7";

    #[test]
    fn a_link_status_off_air_reads_as_wireshark_4_4_18_read_it() {
        let n = nwk(LINK_STATUS);
        assert_eq!(n.frame_type, FrameType::Command);
        assert_eq!((n.src, n.dst, n.radius, n.seq), (0x0897, 0xfffc, 1, 97));
        assert!(n.is_broadcast());
        assert_eq!(n.src_ieee, Some(0xa4c1_3801_d4fd_ffff));
        assert_eq!(n.dst_ieee, None);
        let s = n.security.expect("a security header");
        assert_eq!(s.key, KeyId::Network);
        assert_eq!(s.frame_counter, 8_584_471);
        assert_eq!(s.source, Some(0xa4c1_3801_d4fd_ffff));
        assert_eq!(s.key_seq, Some(0));
    }

    #[test]
    fn a_relayed_command_names_its_origin_and_the_hop_as_wireshark_4_4_18_did() {
        let n = nwk(RELAYED);
        assert_eq!(n.frame_type, FrameType::Command);
        assert_eq!((n.src, n.dst, n.radius, n.seq), (0x0a0d, 0x0000, 29, 135));
        assert_eq!(n.src_ieee, Some(0xa4c1_3801_f5b9_ffff));
        assert_eq!(n.dst_ieee, Some(0xc02c_edff_fe86_4119));
        let s = n.security.expect("a security header");
        assert_eq!(s.frame_counter, 8_584_473);
        assert_eq!(n.hop_sender(), Some(0xa4c1_3801_d4fd_ffff), "the router that relayed it");
    }

    #[test]
    fn an_end_device_data_frame_reads_as_wireshark_4_4_18_read_it() {
        let n = nwk(END_DEVICE);
        assert_eq!(n.frame_type, FrameType::Data);
        assert_eq!((n.src, n.dst, n.radius, n.seq), (0x1a64, 0x0000, 30, 39));
        assert!(n.end_device_initiator);
        assert_eq!((n.src_ieee, n.dst_ieee), (None, None));
        let s = n.security.expect("a security header");
        assert_eq!(s.frame_counter, 548_013);
        assert_eq!(s.source, Some(0x94a0_81ff_fe86_dd48));
    }

    #[test]
    fn the_hop_sender_is_the_layers_subject_with_its_oui() {
        let mac = ieee802154::parse(&mpdu(RELAYED)).unwrap();
        let p = read(&mac).expect("a Zigbee layer");
        assert_eq!(p.kind, "command");
        let who = p.subject.clone().expect("a subject");
        assert_eq!(who.id.to_string(), "A4:C1:38:01:D4:FD:FF:FF");
        assert_eq!(who.vendor.as_deref(), Some("A4C138"));
        assert_eq!(who.stability, Stability::Durable);
        assert_eq!(p.parties(), (Some("0x0a0d"), Some("0x0000")));
        assert!(
            p.facts.iter().any(
                |f| matches!(f, Fact::Protected(s) if s.cipher() == Some("Zigbee network key"))
            )
        );
    }

    #[test]
    fn a_mac_command_is_not_a_zigbee_frame() {
        let mac = ieee802154::parse(&[0x63, 0x88, 0xa9, 0xcb, 0x7a, 0x00, 0x00, 0x64, 0x1a, 0x04])
            .unwrap();
        assert_eq!(read(&mac), None);
    }

    #[test]
    fn a_thread_frame_is_not_a_zigbee_frame() {
        assert_eq!(parse(&[0x7a, 0x33, 0x3a, 0x80, 0x00, 0x00, 0x00, 0x00]), None);
    }

    #[test]
    fn a_synthesised_zigbee_beacon_reads_as_wireshark_4_4_18_read_it() {
        let mac = ieee802154::parse(&mpdu(BEACON)).expect("a MAC frame");
        let b =
            parse_beacon(mac.beacon_payload().expect("a beacon payload")).expect("a Zigbee beacon");
        assert_eq!(b.stack_profile, 2);
        assert!(b.router_capacity);
        assert_eq!(b.depth, 0);
        assert!(b.end_device_capacity);
        assert_eq!(b.extended_pan, 0xdd8a_1c2e_4b35_9f01);
        assert_eq!(b.update_id, 0);
        let p = read(&mac).expect("a Zigbee layer");
        assert_eq!(p.kind, "beacon");
        assert_eq!(p.parties(), (Some("0x0000"), Some("broadcast")));
    }

    const BEACON: &str = "008001cb7a0000ffcf0000002284019f354b2e1c8addffffff00";
}
