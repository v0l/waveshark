//! Bluetooth advertising PDUs: what a packet says once it has proved itself.
//!
//! The split is `dsp::ais`'s. Everything that decides whether a packet
//! happened at all is in `dsp::ble`: the access address, the whitening the
//! channel index seeds, and the CRC-24 that accepts or rejects the frame.
//! What arrives here is a PDU that passed, so this file is tables of offsets
//! and nothing else, and can be checked against packets somebody else
//! captured.
//!
//! # What an advertisement is evidence of
//!
//! An address is not an identity. Since 4.0 a device may advertise a resolvable
//! private address that rotates every fifteen minutes or so, which is the
//! whole point of the feature, and the address type bit in the header says
//! which kind it is. So the address is reported together with whether it is
//! public, and a public one is an OUI that can be looked up while a random one
//! is not a device that will still be there in an hour.
//!
//! The payload is a list of advertising data structures, each a length, a type
//! and a value, from the Bluetooth SIG's assigned numbers. Manufacturer data,
//! type 0xff, opens with a company identifier and the rest is the vendor's
//! own, which is where most of what a device is actually saying lives.

use common::Value;

/// PDU types on the primary advertising channels.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PduType {
    AdvInd,
    AdvDirectInd,
    AdvNonconnInd,
    ScanReq,
    ScanRsp,
    ConnectInd,
    AdvScanInd,
    AdvExtInd,
    Reserved(u8),
}

impl PduType {
    pub fn from_bits(v: u8) -> Self {
        match v & 0x0f {
            0 => Self::AdvInd,
            1 => Self::AdvDirectInd,
            2 => Self::AdvNonconnInd,
            3 => Self::ScanReq,
            4 => Self::ScanRsp,
            5 => Self::ConnectInd,
            6 => Self::AdvScanInd,
            7 => Self::AdvExtInd,
            other => Self::Reserved(other),
        }
    }

    pub fn name(&self) -> &'static str {
        match self {
            Self::AdvInd => "ADV_IND",
            Self::AdvDirectInd => "ADV_DIRECT_IND",
            Self::AdvNonconnInd => "ADV_NONCONN_IND",
            Self::ScanReq => "SCAN_REQ",
            Self::ScanRsp => "SCAN_RSP",
            Self::ConnectInd => "CONNECT_IND",
            Self::AdvScanInd => "ADV_SCAN_IND",
            Self::AdvExtInd => "ADV_EXT_IND",
            Self::Reserved(_) => "reserved",
        }
    }
}

/// A 48 bit device address and whether it is a real one.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Address {
    pub bytes: [u8; 6],
    /// A random address rotates, so it identifies a transmission rather than a
    /// device.
    pub random: bool,
}

impl std::fmt::Display for Address {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        // Most significant byte first, which is how every tool prints one.
        let s: Vec<String> = self
            .bytes
            .iter()
            .rev()
            .map(|b| format!("{b:02X}"))
            .collect();
        write!(f, "{}", s.join(":"))
    }
}

/// One advertising data structure from the payload.
#[derive(Clone, Debug, PartialEq)]
pub struct AdStructure {
    pub kind: u8,
    pub value: Vec<u8>,
}

/// A parsed advertising PDU.
#[derive(Clone, Debug, PartialEq)]
pub struct Advertisement {
    pub pdu_type: PduType,
    pub address: Address,
    /// The address a directed advertisement or a scan request is aimed at.
    pub target: Option<Address>,
    pub data: Vec<AdStructure>,
    /// The local name, complete or shortened, when one was advertised.
    pub name: Option<String>,
    /// Company identifier from the manufacturer data, when there is any.
    pub company: Option<u16>,
    /// Advertised transmit power in dBm, which with the received level is the
    /// only distance evidence a packet carries.
    pub tx_power: Option<i8>,
}

/// Parse a PDU as `dsp::ble` hands it over: two header bytes, then payload.
pub fn parse(pdu: &[u8]) -> Option<Advertisement> {
    if pdu.len() < 8 {
        return None;
    }
    let pdu_type = PduType::from_bits(pdu[0]);
    let address = Address {
        bytes: pdu[2..8].try_into().ok()?,
        random: pdu[0] & 0x40 != 0,
    };
    // The types that carry a second address carry it directly after the
    // first, and the flag for it is a different bit of the same header byte.
    let directed = matches!(
        pdu_type,
        PduType::AdvDirectInd | PduType::ScanReq | PduType::ConnectInd
    );
    let (target, rest) = if directed && pdu.len() >= 14 {
        let t = Address {
            bytes: pdu[8..14].try_into().ok()?,
            random: pdu[0] & 0x80 != 0,
        };
        (Some(t), &pdu[14..])
    } else {
        (None, &pdu[8..])
    };

    let mut data = Vec::new();
    let mut name = None;
    let mut company = None;
    let mut tx_power = None;
    let mut i = 0usize;
    while i < rest.len() {
        let len = rest[i] as usize;
        // A zero length is the early-terminator the specification allows, and
        // a length past the end is a payload that disagrees with its own
        // header: stop either way rather than inventing a structure.
        if len == 0 || i + 1 + len > rest.len() {
            break;
        }
        let kind = rest[i + 1];
        let value = rest[i + 2..i + 1 + len].to_vec();
        match kind {
            // Shortened and complete local name. A name is UTF-8 by
            // specification and is a vendor's field in practice, so it is
            // taken lossily rather than dropped when it is not.
            0x08 | 0x09 => name = Some(String::from_utf8_lossy(&value).into_owned()),
            0x0a if value.len() == 1 => tx_power = Some(value[0] as i8),
            0xff if value.len() >= 2 => company = Some(u16::from_le_bytes([value[0], value[1]])),
            _ => {}
        }
        data.push(AdStructure { kind, value });
        i += 1 + len;
    }

    Some(Advertisement {
        pdu_type,
        address,
        target,
        data,
        name,
        company,
        tx_power,
    })
}

impl Advertisement {
    /// The fields a log or a bus carries, in the order they are worth reading.
    pub fn fields(&self) -> Vec<(String, Value)> {
        let mut f: Vec<(String, Value)> = Vec::new();
        f.push(("type".into(), Value::Text(self.pdu_type.name().into())));
        f.push(("address".into(), Value::Text(self.address.to_string())));
        // Who sent it and who for, under the names every protocol here uses
        // for its parties, so a link between two of them can be followed
        // without knowing which protocol they were speaking.
        f.push(("from".into(), Value::Text(self.address.to_string())));
        f.push((
            "to".into(),
            Value::Text(self.target.map(|t| t.to_string()).unwrap_or_else(|| "broadcast".into())),
        ));
        f.push((
            "address_kind".into(),
            Value::Text(
                if self.address.random {
                    "random"
                } else {
                    "public"
                }
                .into(),
            ),
        ));
        if let Some(t) = self.target {
            f.push(("target".into(), Value::Text(t.to_string())));
        }
        if let Some(n) = &self.name {
            f.push(("name".into(), Value::Text(n.clone())));
        }
        if let Some(c) = self.company {
            f.push(("company".into(), Value::Text(format!("0x{c:04x}"))));
            if let Some(v) = company_name(c) {
                f.push(("vendor".into(), Value::Text(v.into())));
            }
        }
        if let Some(p) = self.tx_power {
            f.push(("tx_power_dbm".into(), Value::Int(i64::from(p))));
        }
        f
    }
}

/// The company identifiers seen often enough to be worth naming.
///
/// Deliberately short. The SIG's list is four figures long and changes every
/// few months, and a stale copy of it compiled in is a decoder that confidently
/// mislabels a device; the identifier itself is always reported, so a name
/// missing here costs a lookup rather than the evidence.
pub fn company_name(id: u16) -> Option<&'static str> {
    Some(match id {
        0x0006 => "Microsoft",
        0x004c => "Apple",
        0x0059 => "Nordic Semiconductor",
        0x0075 => "Samsung",
        0x00e0 => "Google",
        0x0171 => "Amazon",
        0x02e1 => "Victron Energy",
        0x0499 => "Ruuvi",
        _ => return None,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A Samsung monitor's ADV_IND, off air on channel 38, dewhitened and
    /// CRC checked by `dsp::ble` before it reached this.
    const ADV_IND: [u8; 39] = [
        0x00, 0x25, 0x4d, 0x72, 0xef, 0xcb, 0x70, 0x6c, 0x02, 0x01, 0x18, 0x1b, 0xff, 0x75, 0x00,
        0x42, 0x04, 0x01, 0x01, 0x67, 0x6c, 0x70, 0xcb, 0xef, 0x72, 0x4d, 0x6e, 0x70, 0xcb, 0xef,
        0x72, 0x4c, 0x24, 0xe2, 0xea, 0x70, 0x05, 0x05, 0xb8,
    ];

    /// The same device's SCAN_RSP, which is where the name is.
    const SCAN_RSP: [u8; 29] = [
        0x04, 0x1b, 0x4d, 0x72, 0xef, 0xcb, 0x70, 0x6c, 0x14, 0x08, 0x34, 0x39, 0x22, 0x20, 0x4f,
        0x64, 0x79, 0x73, 0x73, 0x65, 0x79, 0x20, 0x4f, 0x4c, 0x45, 0x44, 0x20, 0x47, 0x39,
    ];

    #[test]
    fn an_advertisement_reports_its_address_and_vendor() {
        let a = parse(&ADV_IND).expect("a PDU");
        assert_eq!(a.pdu_type, PduType::AdvInd);
        assert_eq!(a.address.to_string(), "6C:70:CB:EF:72:4D");
        assert!(!a.address.random, "the header says this address is public");
        assert_eq!(a.company, Some(0x0075));
        assert_eq!(company_name(0x0075), Some("Samsung"));
    }

    /// The name is in the scan response rather than the advertisement, which
    /// is the usual arrangement and the reason a passive listener sees a
    /// nameless device until something scans it.
    #[test]
    fn a_scan_response_carries_the_name() {
        let a = parse(&SCAN_RSP).expect("a PDU");
        assert_eq!(a.pdu_type, PduType::ScanRsp);
        assert_eq!(a.name.as_deref(), Some("49\" Odyssey OLED G9"));
    }

    /// A scan request is two addresses and no payload, and the second one is
    /// the device being asked rather than another advertiser.
    #[test]
    fn a_scan_request_reports_who_it_is_aimed_at() {
        let mut pdu = vec![0x43, 0x0c];
        pdu.extend_from_slice(&[0x57, 0xf4, 0xa1, 0xba, 0x57, 0x1a]);
        pdu.extend_from_slice(&[0x4d, 0x72, 0xef, 0xcb, 0x70, 0x6c]);
        let a = parse(&pdu).expect("a PDU");
        assert_eq!(a.pdu_type, PduType::ScanReq);
        assert!(a.address.random, "the scanner used a random address");
        assert_eq!(
            a.target.map(|t| t.to_string()).as_deref(),
            Some("6C:70:CB:EF:72:4D")
        );
    }

    /// A payload whose structure lengths run past the end is truncated
    /// reception, not a device saying something interesting. What was read
    /// before the break is kept and the rest is dropped.
    #[test]
    fn a_payload_that_overruns_stops_rather_than_inventing_a_structure() {
        let pdu = [
            0x00, 0x0b, 1, 2, 3, 4, 5, 6, 0x02, 0x01, 0x06, 0x40, 0x09, b'x',
        ];
        let a = parse(&pdu).expect("a PDU");
        assert_eq!(a.data.len(), 1, "only the flags structure is complete");
        assert_eq!(a.name, None);
    }

    #[test]
    fn a_pdu_too_short_to_hold_an_address_is_not_one() {
        assert!(parse(&[0x00, 0x05, 1, 2, 3]).is_none());
    }
}
