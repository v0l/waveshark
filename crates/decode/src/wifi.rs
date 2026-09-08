//! The 802.11 MAC frame: who sent it, to whom, and what a beacon says about
//! the network it belongs to.
//!
//! Header parsing and the information elements a management frame carries.
//! Nothing here demodulates; `dsp::wifi` does that and hands over a PSDU whose
//! FCS it has already checked.

use std::fmt;

/// A MAC address.
#[derive(Clone, Copy, PartialEq, Eq, Hash)]
pub struct Mac(pub [u8; 6]);

impl Mac {
    /// The all-ones address every station listens to.
    pub fn is_broadcast(&self) -> bool {
        self.0 == [0xff; 6]
    }

    /// Whether the address is one the device made up rather than one it was
    /// given. Every phone since about 2014 randomises for probe requests, so
    /// an address without this bit is a device that can be followed and one
    /// with it is a device that cannot.
    pub fn is_local(&self) -> bool {
        self.0[0] & 0x02 != 0
    }

    /// The manufacturer's three byte prefix, which is only meaningful when
    /// the address is not locally administered.
    pub fn oui(&self) -> Option<[u8; 3]> {
        (!self.is_local()).then_some([self.0[0], self.0[1], self.0[2]])
    }
}

impl fmt::Display for Mac {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let s: Vec<String> = self.0.iter().map(|b| format!("{b:02X}")).collect();
        write!(f, "{}", s.join(":"))
    }
}

impl fmt::Debug for Mac {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{self}")
    }
}

/// What a frame is for. The names are the standard's, shortened to what a
/// person reading a packet list needs.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Kind {
    Management(u8),
    Control(u8),
    Data(u8),
}

impl Kind {
    pub fn name(&self) -> &'static str {
        match self {
            Kind::Management(s) => match s {
                0 => "assoc-req",
                1 => "assoc-resp",
                2 => "reassoc-req",
                3 => "reassoc-resp",
                4 => "probe-req",
                5 => "probe-resp",
                8 => "beacon",
                9 => "atim",
                10 => "disassoc",
                11 => "auth",
                12 => "deauth",
                13 => "action",
                _ => "management",
            },
            Kind::Control(s) => match s {
                8 => "block-ack-req",
                9 => "block-ack",
                10 => "ps-poll",
                11 => "rts",
                12 => "cts",
                13 => "ack",
                14 => "cf-end",
                _ => "control",
            },
            Kind::Data(s) => match s {
                0 => "data",
                4 => "null",
                8 => "qos-data",
                12 => "qos-null",
                _ => "data",
            },
        }
    }

    /// Whether a frame of this kind carries the three addresses and the
    /// sequence control field. A control frame does not: an ACK is ten bytes
    /// and a CTS is the same, and reading a source address out of one is
    /// reading the FCS.
    pub fn has_full_header(&self) -> bool {
        !matches!(self, Kind::Control(_))
    }
}

/// What a beacon or a probe response says about its network.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Network {
    /// The SSID, or `None` for a hidden one, which is a zero length element
    /// rather than a missing one.
    pub ssid: Option<String>,
    /// The channel it claims to be on, from the DS parameter set. Worth
    /// carrying because it need not be the channel it was heard on.
    pub channel: Option<u8>,
    /// Whether the capability information says the network is protected.
    pub privacy: bool,
    /// The beacon interval in time units of 1024 us.
    pub beacon_interval: u16,
    /// Whether an RSN element is present, meaning WPA2 or later rather than
    /// the WEP that the privacy bit alone would mean.
    pub rsn: bool,
}

/// A parsed MAC frame.
#[derive(Clone, Debug, PartialEq)]
pub struct Frame {
    pub kind: Kind,
    /// The receiver, or the broadcast address.
    pub addr1: Mac,
    /// The transmitter, where the frame has one.
    pub addr2: Option<Mac>,
    /// The BSS, or the third address of a data frame.
    pub addr3: Option<Mac>,
    /// Whether the payload is encrypted.
    pub protected: bool,
    /// Which way the frame is going, as the two distribution system bits say.
    pub to_ds: bool,
    pub from_ds: bool,
    pub seq: Option<u16>,
    pub network: Option<Network>,
}

impl Frame {
    /// The address a row about this frame is filed under: whoever sent it,
    /// falling back to the BSS for a control frame that names nobody.
    pub fn source(&self) -> Option<Mac> {
        self.addr2
    }

    /// The network's own address, which for a beacon is the access point.
    pub fn bssid(&self) -> Option<Mac> {
        match (self.to_ds, self.from_ds) {
            (false, false) => self.addr3,
            (false, true) => self.addr2,
            (true, false) => Some(self.addr1),
            (true, true) => None,
        }
    }
}

fn mac(b: &[u8]) -> Mac {
    let mut m = [0u8; 6];
    m.copy_from_slice(&b[..6]);
    Mac(m)
}

/// Parse a MAC frame. `psdu` includes the FCS, which is not read here.
///
/// `None` when the bytes are too short to be the frame they claim to be,
/// which is the only structural check available: 802.11 has no length field
/// in the header, so what protects this is the FCS the demodulator checked.
pub fn parse(psdu: &[u8]) -> Option<Frame> {
    let body = psdu.get(..psdu.len().checked_sub(4)?)?;
    if body.len() < 10 {
        return None;
    }
    let fc = u16::from_le_bytes([body[0], body[1]]);
    let subtype = (fc >> 4 & 0xf) as u8;
    let kind = match fc >> 2 & 3 {
        0 => Kind::Management(subtype),
        1 => Kind::Control(subtype),
        2 => Kind::Data(subtype),
        _ => return None,
    };
    let to_ds = fc & 0x100 != 0;
    let from_ds = fc & 0x200 != 0;
    let protected = fc & 0x4000 != 0;

    let addr1 = mac(&body[4..]);
    let (mut addr2, mut addr3, mut seq) = (None, None, None);
    if kind.has_full_header() {
        if body.len() < 24 {
            return None;
        }
        addr2 = Some(mac(&body[10..]));
        addr3 = Some(mac(&body[16..]));
        seq = Some(u16::from_le_bytes([body[22], body[23]]) >> 4);
    }

    let network = match kind {
        // A beacon and a probe response carry a timestamp, a beacon interval
        // and the capability information before their elements.
        Kind::Management(8) | Kind::Management(5) if body.len() >= 36 => Some(network(&body[24..])),
        // A probe request has no fixed fields, and the SSID it asks for is
        // the first element.
        Kind::Management(4) => Some(Network {
            ssid: elements(&body[24..])
                .find(|(id, _)| *id == 0)
                .map(|(_, v)| ssid(v)),
            ..Default::default()
        }),
        _ => None,
    };

    Some(Frame {
        kind,
        addr1,
        addr2,
        addr3,
        protected,
        to_ds,
        from_ds,
        seq,
        network,
    })
}

/// The information elements of a management frame body, as id and value.
fn elements(mut b: &[u8]) -> impl Iterator<Item = (u8, &[u8])> {
    std::iter::from_fn(move || {
        let (&id, rest) = b.split_first()?;
        let (&len, rest) = rest.split_first()?;
        let (val, rest) = rest.split_at_checked(len as usize)?;
        b = rest;
        Some((id, val))
    })
}

/// An SSID as a string. Non-UTF-8 is escaped rather than dropped: an access
/// point is free to put any bytes here and some do it on purpose.
fn ssid(v: &[u8]) -> String {
    String::from_utf8(v.to_vec())
        .unwrap_or_else(|_| v.iter().map(|b| format!("\\x{b:02x}")).collect())
}

fn network(body: &[u8]) -> Network {
    let mut n = Network {
        beacon_interval: u16::from_le_bytes([body[8], body[9]]),
        privacy: u16::from_le_bytes([body[10], body[11]]) & 0x10 != 0,
        ..Default::default()
    };
    for (id, v) in elements(&body[12..]) {
        match id {
            0 if !v.is_empty() => n.ssid = Some(ssid(v)),
            3 if !v.is_empty() => n.channel = Some(v[0]),
            48 => n.rsn = true,
            _ => {}
        }
    }
    n
}

#[cfg(test)]
mod tests {
    use super::*;

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
        v.extend([0u8; 4]);
        v
    }

    #[test]
    fn a_beacon_names_its_network_and_its_access_point() {
        let f = parse(&beacon()).expect("a frame");
        assert_eq!(f.kind, Kind::Management(8));
        assert_eq!(f.kind.name(), "beacon");
        assert!(f.addr1.is_broadcast());
        assert_eq!(f.source().unwrap().to_string(), "00:1A:2B:3C:4D:5E");
        assert_eq!(f.bssid(), f.source());
        assert_eq!(f.seq, Some(1));
        let n = f.network.unwrap();
        assert_eq!(n.ssid.as_deref(), Some("waveshark"));
        assert_eq!(n.channel, Some(6));
        assert_eq!(n.beacon_interval, 100);
        assert!(n.privacy && n.rsn);
    }

    /// A hidden network sends a beacon with an SSID element of length zero,
    /// which is a different thing from a beacon with no SSID element and has
    /// to stay distinguishable from one.
    #[test]
    fn a_hidden_network_has_no_ssid_but_is_still_a_beacon() {
        let mut v = beacon();
        // Replace the nine byte SSID with an empty one.
        let at = 36;
        v.splice(at..at + 11, [0x00, 0x00]);
        let f = parse(&v).expect("a frame");
        let n = f.network.unwrap();
        assert_eq!(n.ssid, None);
        assert_eq!(n.channel, Some(6));
    }

    /// An acknowledgement is fourteen bytes and names only its receiver.
    /// Reading three addresses out of one reads past the end of the frame.
    #[test]
    fn a_control_frame_has_one_address_and_no_sequence() {
        let mut v = vec![0xd4, 0x00, 0x00, 0x00];
        v.extend([0x00, 0x11, 0x22, 0x33, 0x44, 0x55]);
        v.extend([0u8; 4]);
        let f = parse(&v).expect("a frame");
        assert_eq!(f.kind.name(), "ack");
        assert_eq!(f.addr2, None);
        assert_eq!(f.seq, None);
        assert_eq!(f.addr1.to_string(), "00:11:22:33:44:55");
    }

    #[test]
    fn a_randomised_address_says_so() {
        assert!(Mac([0x02, 0, 0, 0, 0, 1]).is_local());
        assert_eq!(Mac([0x02, 0, 0, 0, 0, 1]).oui(), None);
        assert_eq!(
            Mac([0x00, 0x1a, 0x2b, 0, 0, 1]).oui(),
            Some([0x00, 0x1a, 0x2b])
        );
    }

    #[test]
    fn a_truncated_frame_is_refused_rather_than_read_past() {
        assert_eq!(parse(&[]), None);
        assert_eq!(parse(&[0x80, 0x00, 0x00, 0x00, 0xff]), None);
        // A management frame that stops inside its addresses.
        let mut v = vec![0x80, 0x00, 0x00, 0x00];
        v.extend([0xff; 12]);
        assert_eq!(parse(&v), None);
    }
}
