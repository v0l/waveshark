//! What a GSM cell says about itself on its broadcast channel.
//!
//! The 23 byte blocks `dsp::gsm::bcch` recovers are layer 3 messages with a
//! two byte layer 2 header in front of them, and on the broadcast channel
//! they are almost all system information: the cell's identity, the location
//! area it belongs to, which neighbours to measure, and how to get on the
//! random access channel. None of it is ciphered, because a phone that has
//! not registered yet has to be able to read it.
//!
//! This reads the identity and leaves the rest named but unparsed. Cell
//! identity, location area and the operator behind it are what turn a decode
//! into a row somebody can act on: two receivers in different places can
//! compare them, a survey can plot them, and a change in one is a network
//! that reconfigured. The frequency lists and the access parameters are
//! large, are only useful to something that intends to transmit, and can be
//! added a field at a time when there is a reason to.
//!
//! Message numbers are from 3GPP TS 44.018 table 10.4.1, and the identity
//! layout from TS 24.008 section 10.5.1.3.

/// The operator and area a cell belongs to.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Lai {
    /// Mobile country code, three digits.
    pub mcc: u16,
    /// Mobile network code, two digits or three.
    pub mnc: u16,
    /// How many digits the network code was sent with, which is part of its
    /// identity: 01 and 001 are different networks in the same country.
    pub mnc_digits: u8,
    /// Location area, which is the unit a phone is paged across.
    pub lac: u16,
}

impl std::fmt::Display for Lai {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self.mnc_digits {
            3 => write!(f, "{:03}-{:03}", self.mcc, self.mnc),
            _ => write!(f, "{:03}-{:02}", self.mcc, self.mnc),
        }
    }
}

/// The channel numbers a frequency list holds.
///
/// One format of several, and the only one seen on a live network so far:
/// GSM 05.08's bit map 0, which is 124 bits, one per channel number, and
/// covers the whole 900 band. The others encode ranges of channel numbers
/// for the 1800 band, where 124 bits would not reach, and are refused rather
/// than guessed at: a wrong frequency list reads as a neighbour that is not
/// there, and nothing downstream can tell that from a cell that has gone off
/// the air.
///
/// The layout is 3GPP TS 44.018 figure 10.5.2.1b.2.1: channel 124 is bit 4
/// of the second octet, and channel 1 is bit 1 of the seventeenth.
fn channels(ie: &[u8]) -> Option<Vec<u16>> {
    if ie.len() < 16 {
        return None;
    }
    // Bits 8 and 7 of the first octet say which format this is; bit map 0
    // is zero. Bits 6 and 5 are spare in a cell allocation and carry the
    // extension and allocation sequence indicators in a neighbour list, so
    // neither is looked at here.
    if ie[0] & 0xC0 != 0 {
        return None;
    }
    let mut out = Vec::new();
    for n in 1..=124u16 {
        // Counting down from channel 124, which is the top bit of the first
        // octet's low nibble; the four that fit there come first and the
        // rest fill the fifteen octets after it from the top bit down.
        let from_top = usize::from(124 - n);
        let (byte, place) = match from_top {
            0..=3 => (ie[0], 3 - from_top),
            t => (ie[1 + (t - 4) / 8], 7 - (t - 4) % 8),
        };
        if byte >> place & 1 == 1 {
            out.push(n);
        }
    }
    Some(out)
}

/// Who a paging request is calling.
///
/// A network pages by temporary identity almost always, which is the point
/// of the temporary identity: it is reallocated, so a run of them says how
/// busy a cell is without saying whose phones they are. A network that pages
/// by permanent identity has given that up, and reading it is how anyone
/// knows that is happening.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Identity {
    /// Temporary subscriber identity, four octets, reallocated by the
    /// network whenever it chooses.
    Tmsi(u32),
    /// The permanent subscriber identity, as digits.
    Imsi(String),
    /// The equipment's identity, with or without its software version.
    Imei(String),
    /// A type this does not read, kept so a row still says a page happened.
    Other(u8),
}

impl std::fmt::Display for Identity {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Identity::Tmsi(v) => write!(f, "TMSI {v:08X}"),
            Identity::Imsi(s) => write!(f, "IMSI {s}"),
            Identity::Imei(s) => write!(f, "IMEI {s}"),
            Identity::Other(t) => write!(f, "identity type {t}"),
        }
    }
}

/// Read a mobile identity, 3GPP TS 24.008 section 10.5.1.4.
///
/// The low three bits of the first octet are the type and the fourth says
/// whether the digit count is odd; the digits follow as nibbles, low nibble
/// of each octet first. A temporary identity is not digits at all: it is
/// four octets after a first octet of 0xF4.
fn identity(b: &[u8]) -> Option<Identity> {
    let first = *b.first()?;
    let odd = first & 0x08 != 0;
    Some(match first & 0x07 {
        // Type zero is "no identity", which a network sends to keep the
        // channel occupied when it has nobody to call. Counting those as
        // pages made a quiet cell look busy: a third of the pages in a
        // recording were this.
        0 => return None,
        4 => Identity::Tmsi(u32::from_be_bytes(b.get(1..5)?.try_into().ok()?)),
        t @ (1 | 2 | 3) => {
            let mut digits = String::new();
            digits.push(char::from(b'0' + (first >> 4)));
            for &octet in &b[1..] {
                digits.push(char::from(b'0' + (octet & 0x0F)));
                digits.push(char::from(b'0' + (octet >> 4)));
            }
            // An even count of digits leaves a filler nibble at the end.
            if !odd {
                digits.pop();
            }
            let plausible = match t {
                1 => (6..=15).contains(&digits.len()),
                _ => (14..=16).contains(&digits.len()),
            };
            if !plausible || digits.bytes().any(|c| !c.is_ascii_digit()) {
                return None;
            }
            if t == 1 {
                Identity::Imsi(digits)
            } else {
                Identity::Imei(digits)
            }
        }
        t => Identity::Other(t),
    })
}

/// The identities a paging request carries.
///
/// Three shapes, from 3GPP TS 44.018 sections 9.1.22 to 9.1.24. The first
/// carries one or two identities of any kind as length-prefixed fields; the
/// second carries two temporary identities as bare octets and may add a
/// third of any kind; the third carries four temporary identities.
fn pages(type_id: u8, body: &[u8]) -> Vec<Identity> {
    let mut out = Vec::new();
    // The first octet is the page mode and which channel the phones should
    // answer on, neither of which says who is being called.
    let Some(rest) = body.get(1..) else { return out };
    match type_id {
        0x21 => {
            let mut at = 0usize;
            // Two identities at most: the first is always there as a length
            // and a value, the second arrives behind the tag 0x17.
            for _ in 0..2 {
                let Some(&len) = rest.get(at) else { break };
                let len = usize::from(len);
                if len == 0 || at + 1 + len > rest.len() {
                    break;
                }
                if let Some(id) = identity(&rest[at + 1..at + 1 + len]) {
                    out.push(id);
                }
                at += 1 + len;
                if rest.get(at) != Some(&0x17) {
                    break;
                }
                at += 1;
            }
        }
        0x22 | 0x24 => {
            // Bare temporary identities, four octets each: two in a type 2
            // and four in a type 3.
            let count = if type_id == 0x22 { 2 } else { 4 };
            for n in 0..count {
                let Some(v) = rest.get(n * 4..n * 4 + 4) else { break };
                let tmsi = u32::from_be_bytes(v.try_into().unwrap());
                // A network with nothing to page fills the field with ones.
                if tmsi != u32::MAX {
                    out.push(Identity::Tmsi(tmsi));
                }
            }
            // A type 2 may add one identity of any kind behind the tag.
            if type_id == 0x22 {
                if let Some(at) = rest.get(8).and_then(|&t| (t == 0x17).then_some(9)) {
                    let len = usize::from(*rest.get(at).unwrap_or(&0));
                    if len > 0 && at + 1 + len <= rest.len() {
                        if let Some(id) = identity(&rest[at + 1..at + 1 + len]) {
                            out.push(id);
                        }
                    }
                }
            }
        }
        _ => {}
    }
    out
}

/// The channel a network has just granted a phone.
///
/// An immediate assignment is the reply to a phone that asked for a channel,
/// so it says where a transaction is about to happen: which timeslot on
/// which carrier, or which hopping sequence, and how far away the phone is.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Grant {
    /// What kind of channel, as the standard names them.
    pub kind: &'static str,
    /// Subchannel within it, where the kind has several.
    pub subchannel: u8,
    /// Timeslot in the TDMA frame, zero to seven.
    pub timeslot: u8,
    /// Which training sequence the channel uses.
    pub tsc: u8,
    /// The channel number, where the channel does not hop.
    pub arfcn: Option<u16>,
    /// The hopping sequence and this channel's offset in it, where it does.
    pub hopping: Option<(u8, u8)>,
    /// Timing advance the network told the phone to use, in bit periods.
    ///
    /// A measurement rather than a setting: it is how long the phone's burst
    /// took to arrive, so it is the distance to it. One unit is a bit period,
    /// which light covers in about 554 metres there and back.
    pub timing_advance: u8,
}

impl Grant {
    /// How far away the phone is, in metres, as the timing advance measures
    /// it. Coarse by construction: the whole scale is 64 steps of 554 m.
    pub fn distance_m(&self) -> u32 {
        u32::from(self.timing_advance) * 554
    }
}

/// Read the channel description and what follows it, 3GPP TS 44.018 sections
/// 10.5.2.5 and 9.1.18.
fn grant(body: &[u8]) -> Option<Grant> {
    let d = body.get(1..4)?;
    // The first octet is the same layout the A-bis interface uses for a
    // channel number: a variable length type field with the timeslot in the
    // low three bits.
    let (kind, subchannel) = match d[0] >> 3 {
        0b00001 => ("TCH/F", 0),
        v if v >> 1 == 0b0001 => ("TCH/H", v & 1),
        v if v >> 2 == 0b001 => ("SDCCH/4", v & 3),
        v if v >> 3 == 0b01 => ("SDCCH/8", v & 7),
        0b10000 => ("BCCH", 0),
        0b10010 => ("CCCH", 0),
        _ => return None,
    };
    let hopping = d[1] & 0x10 != 0;
    Some(Grant {
        kind,
        subchannel,
        timeslot: d[0] & 0x07,
        tsc: d[1] >> 5,
        arfcn: (!hopping).then(|| u16::from(d[1] & 0x03) << 8 | u16::from(d[2])),
        // Four bits of the offset in the second octet and two in the third,
        // then the hopping sequence number.
        hopping: hopping.then(|| ((d[1] & 0x0F) << 2 | d[2] >> 6, d[2] & 0x3F)),
        // Page mode octet, channel description, request reference, then the
        // timing advance.
        timing_advance: body.get(7).copied().unwrap_or(0),
    })
}

/// A message off the broadcast channel.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Message {
    /// What the message is called, for a row that has to say something even
    /// when nothing inside it is parsed.
    pub name: &'static str,
    /// The layer 3 message type, kept because the name is this file's
    /// opinion and the number is the network's.
    pub type_id: u8,
    /// The cell, where the message carries it.
    pub cell_id: Option<u16>,
    pub lai: Option<Lai>,
    /// The channel numbers the message lists. Empty where it carries none,
    /// or carries them in a format this does not read.
    pub channels: Vec<u16>,
    /// What that list is. A type 1 carries the cell's own allocation, which
    /// is the set of frequencies it hops over; a type 2 carries the
    /// neighbours a phone is told to measure, which is where the rest of the
    /// network is. Confusing the two would put a scanner on the serving
    /// cell's own hopping set and call it the neighbourhood.
    pub channels_are_neighbours: bool,
    /// Who a paging request is calling, where the message is one.
    pub pages: Vec<Identity>,
    /// The channel an immediate assignment grants.
    pub grant: Option<Grant>,
    /// The service the frame belonged to on a dedicated channel: zero is
    /// signalling and three is short messages. Zero on the broadcast
    /// channel, which addresses nobody.
    pub sapi: u8,
    /// The phone a dedicated message names, where it names one. This is the
    /// exchange that happens before ciphering starts: a phone arriving on a
    /// signalling channel says who it is, and a network that does not
    /// recognise the temporary identity asks for the permanent one.
    pub identity: Option<Identity>,
}

impl Message {
    fn named(name: &'static str, type_id: u8) -> Self {
        Self {
            name,
            type_id,
            cell_id: None,
            lai: None,
            channels: Vec::new(),
            channels_are_neighbours: false,
            pages: Vec::new(),
            grant: None,
            sapi: 0,
            identity: None,
        }
    }
}

/// A layer 3 message on a dedicated channel, 3GPP TS 24.008 and 44.018.
///
/// The first octet's low nibble says which protocol the message belongs to
/// and the second is its type. Only the messages that say something about
/// who is on the channel are read past their name: everything else is worth
/// a row saying it happened and nothing more.
fn parse_l3(b: &[u8]) -> Option<Message> {
    let (pd, type_id) = (b.first()? & 0x0F, *b.get(1)?);
    let body = &b[2..];
    // A length-prefixed identity at `at`.
    let ident_at = |at: usize| -> Option<Identity> {
        let len = usize::from(*body.get(at)?);
        identity(body.get(at + 1..at + 1 + len)?)
    };
    let mut m = match (pd, type_id) {
        // Mobility management: the phone arriving, being asked who it is,
        // and being given a new temporary identity.
        (0x05, 0x08) => {
            let mut m = Message::named("LocationUpdatingRequest", type_id);
            m.lai = body.get(1..6).and_then(lai);
            // Location updating type, the area it was last in, one octet of
            // classmark, then the phone.
            m.identity = ident_at(7);
            m
        }
        (0x05, 0x01) => Message::named("ImsiDetach", type_id),
        (0x05, 0x02) => {
            let mut m = Message::named("LocationUpdatingAccept", type_id);
            m.lai = body.get(..5).and_then(lai);
            m
        }
        (0x05, 0x04) => Message::named("LocationUpdatingReject", type_id),
        (0x05, 0x12) => Message::named("AuthenticationRequest", type_id),
        (0x05, 0x14) => Message::named("AuthenticationResponse", type_id),
        (0x05, 0x18) => Message::named("IdentityRequest", type_id),
        (0x05, 0x19) => {
            let mut m = Message::named("IdentityResponse", type_id);
            m.identity = ident_at(0);
            m
        }
        (0x05, 0x1A) => {
            let mut m = Message::named("TmsiReallocationCommand", type_id);
            m.lai = body.get(..5).and_then(lai);
            m.identity = ident_at(5);
            m
        }
        (0x05, 0x24) => {
            let mut m = Message::named("CmServiceRequest", type_id);
            // Service type and key sequence, then the classmark as a length
            // and a value, then the phone.
            let after = 1 + usize::from(*body.first().unwrap_or(&0)) + 1;
            m.identity = ident_at(after.min(body.len()));
            m
        }
        (0x05, 0x21) => Message::named("CmServiceAccept", type_id),
        (0x05, 0x23) => Message::named("CmServiceReject", type_id),
        // Radio resource, as it appears on a dedicated channel rather than
        // on the broadcast one.
        (0x06, 0x27) => {
            let mut m = Message::named("PagingResponse", type_id);
            let after = 1 + usize::from(*body.get(1).unwrap_or(&0)) + 1;
            m.identity = ident_at(after.min(body.len()));
            m
        }
        (0x06, 0x35) => Message::named("CipheringModeCommand", type_id),
        (0x06, 0x32) => Message::named("CipheringModeComplete", type_id),
        (0x06, 0x0D) => Message::named("ChannelRelease", type_id),
        (0x06, 0x2E) => Message::named("AssignmentCommand", type_id),
        (0x06, 0x29) => Message::named("AssignmentComplete", type_id),
        (0x06, 0x2B) => Message::named("HandoverCommand", type_id),
        (0x06, 0x15) => Message::named("ClassmarkChange", type_id),
        (0x06, 0x16) => Message::named("ClassmarkEnquiry", type_id),
        (0x06, 0x06) => Message::named("SI5ter", type_id),
        // Call control and short messages, named only: what they carry is
        // the call itself, and by the time one appears the channel is
        // ciphered.
        (0x03, _) => Message::named("CallControl", type_id),
        (0x09, _) => Message::named("ShortMessage", type_id),
        _ => return None,
    };
    m.type_id = type_id;
    Some(m)
}

/// Read a block off a dedicated channel.

/// Read a signalling channel block a cell has assigned to a phone.
///
/// The blocks are the same 23 bytes and the same coding, but a dedicated
/// channel puts a link layer in front of the message: an address saying which
/// service the frame belongs to, a control field carrying the sequence
/// numbers, and a length. The broadcast channel skips all three, because
/// nothing there is acknowledged and nobody is addressed.
///
/// `None` where the frame carries no message: an unacknowledged fill frame,
/// a link layer acknowledgement with nothing behind it, or padding.
pub fn parse_dedicated(block: &[u8]) -> Option<Message> {
    let (&address, &control, &length) = (block.first()?, block.get(1)?, block.get(2)?);
    // Bits 4 and 3 of the address are the service access point: zero is
    // signalling, three is short messages. The rest is the direction bit and
    // two extension bits that are always set on this link.
    let sapi = address >> 2 & 0x07;
    // A control field with its low bit clear is an information frame, and
    // one ending in 0b11 is unnumbered. Supervisory frames, ending in 0b01,
    // acknowledge and carry nothing.
    if control & 0x03 == 0x01 {
        return None;
    }
    // The length's top six bits are the count of message octets.
    let len = usize::from(length >> 2);
    if len == 0 || 3 + len > block.len() {
        return None;
    }
    let mut msg = parse_l3(&block[3..3 + len])?;
    msg.sapi = sapi;
    Some(msg)
}

/// Read a block off the broadcast or common control channel.
///
/// `None` means the block is not a radio resource message this understands,
/// which on a real cell mostly means a filler frame: a base station with
/// nothing to send transmits `2B` padding, and that is not a decode.
pub fn parse(block: &[u8]) -> Option<Message> {
    if block.len() < 3 {
        return None;
    }
    // The layer 2 pseudo length says how much of the block the message
    // occupies; the rest is padding. Its two low bits are not part of it.
    let len = usize::from(block[0] >> 2);
    // A protocol discriminator of 6 is radio resource management. The high
    // nibble is the skip indicator and is ignored on the broadcast channel.
    if block[1] & 0x0F != 0x06 {
        return None;
    }
    let type_id = block[2];
    let body = &block[3..(3 + len.saturating_sub(1)).min(block.len())];

    // Where a message opens with a 16 octet frequency list: the cell's own
    // allocation in a type 1, and the neighbours to measure in the types
    // that describe a BCCH allocation.
    let (name, ident, has_list) = match type_id {
        0x19 => ("SI1", Ident::None, true),
        0x1A => ("SI2", Ident::None, true),
        0x02 => ("SI2bis", Ident::None, true),
        0x03 => ("SI2ter", Ident::None, true),
        0x07 => ("SI2quater", Ident::None, false),
        // The two that carry the cell's own identity, and the reason this
        // file exists: type 3 on the broadcast channel, type 6 on the slow
        // associated channel of a call in progress.
        0x1B => ("SI3", Ident::CellAndArea, false),
        0x1E => ("SI6", Ident::CellAndArea, false),
        0x1C => ("SI4", Ident::AreaOnly, false),
        0x1D => ("SI5", Ident::None, true),
        0x05 => ("SI5bis", Ident::None, true),
        0x06 => ("SI5ter", Ident::None, true),
        0x00 => ("SI13", Ident::None, false),
        0x21 => ("Paging1", Ident::None, false),
        0x22 => ("Paging2", Ident::None, false),
        0x24 => ("Paging3", Ident::None, false),
        0x3F => ("ImmediateAssign", Ident::None, false),
        0x39 => ("ImmediateAssignExt", Ident::None, false),
        0x3A => ("ImmediateAssignReject", Ident::None, false),
        _ => return None,
    };

    let (cell_id, lai) = match ident {
        Ident::None => (None, None),
        Ident::CellAndArea => {
            let cell = body.get(..2).map(|b| u16::from(b[0]) << 8 | u16::from(b[1]));
            (cell, body.get(2..7).and_then(lai))
        }
        Ident::AreaOnly => (None, body.get(..5).and_then(lai)),
    };
    let list = has_list.then(|| channels(body)).flatten().unwrap_or_default();
    Some(Message {
        name,
        type_id,
        cell_id,
        lai,
        channels: list,
        channels_are_neighbours: type_id != 0x19,
        // The whole block rather than what the pseudo length covers: a
        // request that understates its own length would otherwise drop the
        // identity it was sent to carry, and the padding after one cannot be
        // mistaken for another, since a second has to arrive behind its tag.
        pages: pages(type_id, &block[3..]),
        grant: (type_id == 0x3F).then(|| grant(body)).flatten(),
        sapi: 0,
        identity: None,
    })
}

enum Ident {
    None,
    /// Cell identity then location area, as system information 3 and 6 carry
    /// them.
    CellAndArea,
    /// Location area alone, as system information 4 carries it.
    AreaOnly,
}

/// Five octets: the country and network codes as swapped nibbles, then the
/// location area.
///
/// The nibble order is the trap here and it is not an accident of this file:
/// a location area is written MCC2 MCC1, MNC3 MCC3, MNC2 MNC1, so 62 F2 10
/// is country 262 network 01. An `F` in the network's third digit means the
/// network code has two digits rather than three.
fn lai(b: &[u8]) -> Option<Lai> {
    let digit = |v: u8| (v <= 9).then_some(u16::from(v));
    let mcc = digit(b[0] & 0x0F)? * 100 + digit(b[0] >> 4)? * 10 + digit(b[1] & 0x0F)?;
    let third = b[1] >> 4;
    let (mnc, mnc_digits) = if third == 0x0F {
        (digit(b[2] & 0x0F)? * 10 + digit(b[2] >> 4)?, 2)
    } else {
        (digit(b[2] & 0x0F)? * 100 + digit(b[2] >> 4)? * 10 + digit(third)?, 3)
    };
    Some(Lai { mcc, mnc, mnc_digits, lac: u16::from(b[3]) << 8 | u16::from(b[4]) })
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A system information type 3 built to the layout in 44.018: pseudo
    /// length, radio resource discriminator, message type, then the cell
    /// identity and the location area.
    fn si3(cell: u16, lai: [u8; 5]) -> Vec<u8> {
        let mut b = vec![0x49, 0x06, 0x1B];
        b.extend_from_slice(&cell.to_be_bytes());
        b.extend_from_slice(&lai);
        // Control channel description, cell options, selection parameters
        // and the access parameters, none of which is read yet.
        b.extend_from_slice(&[0x00; 9]);
        b.resize(23, 0x2B);
        b
    }

    /// The nibble order, against a value that is public knowledge: 62 F2 10
    /// is the German operator 262-01, which is the example every SIM tool
    /// gives. Getting this wrong reports a plausible network in the wrong
    /// country, which no CRC anywhere will catch.
    #[test]
    fn a_location_area_reads_country_network_and_area() {
        let m = parse(&si3(0x1234, [0x62, 0xF2, 0x10, 0x11, 0x22])).expect("an SI3");
        assert_eq!(m.name, "SI3");
        assert_eq!(m.cell_id, Some(0x1234));
        let lai = m.lai.expect("a location area");
        assert_eq!((lai.mcc, lai.mnc, lai.mnc_digits), (262, 1, 2));
        assert_eq!(lai.lac, 0x1122);
        assert_eq!(lai.to_string(), "262-01");
    }

    /// A three digit network code, which North America uses and which the
    /// two digit reading would report as a different network.
    #[test]
    fn a_three_digit_network_code_is_not_truncated() {
        // 310-260: country 310, network 260.
        let m = parse(&si3(1, [0x13, 0x00, 0x62, 0x00, 0x01])).unwrap();
        let lai = m.lai.unwrap();
        assert_eq!((lai.mcc, lai.mnc, lai.mnc_digits), (310, 260, 3));
        assert_eq!(lai.to_string(), "310-260");
    }

    /// Type 4 carries the area but not the cell, and reading it at the
    /// offset type 3 uses would report the first two bytes of the area as a
    /// cell identity.
    #[test]
    fn type_four_carries_an_area_and_no_cell() {
        let mut b = vec![0x25, 0x06, 0x1C, 0x62, 0xF2, 0x10, 0x00, 0x64];
        b.resize(23, 0x2B);
        let m = parse(&b).unwrap();
        assert_eq!(m.name, "SI4");
        assert_eq!(m.cell_id, None);
        assert_eq!(m.lai.unwrap().lac, 100);
    }

    /// The frequency list, against the figure in 44.018: channel 124 is bit
    /// 4 of the first octet of the element and channel 1 is bit 1 of the
    /// last, so a list is read from the top down.
    #[test]
    fn a_frequency_list_reads_from_the_top_channel_down() {
        let mut ie = [0u8; 16];
        ie[0] = 0b0000_1001; // channels 124 and 121
        ie[1] = 0b1000_0000; // channel 120
        ie[15] = 0b0000_0101; // channels 3 and 1
        assert_eq!(channels(&ie), Some(vec![1, 3, 120, 121, 124]));

        // A format this does not read is refused rather than guessed at.
        ie[0] |= 0b1000_0000;
        assert_eq!(channels(&ie), None);
    }

    /// A type 2 carries the neighbours a phone is told to measure, which is
    /// the list that says where the rest of the network is.
    #[test]
    fn a_type_two_lists_the_neighbours() {
        let mut b = vec![0x59, 0x06, 0x1A];
        // Bit map 0, no extension, allocation sequence 1, then channels 65,
        // 63, 58 and 57 as a live cell listed them.
        b.extend_from_slice(&[
            0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x43, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00,
        ]);
        b.resize(23, 0x2B);
        let m = parse(&b).unwrap();
        assert_eq!(m.name, "SI2");
        assert_eq!(m.channels, vec![57, 58, 63, 65]);
    }

    /// A type 3 has no frequency list, and reading one out of its cell
    /// identity would invent a network's worth of neighbours.
    #[test]
    fn a_type_three_lists_no_channels() {
        let m = parse(&si3(0x1234, [0x62, 0xF2, 0x10, 0x11, 0x22])).unwrap();
        assert!(m.channels.is_empty());
    }

    /// A paging request by temporary identity, which is nearly all of them.
    #[test]
    fn a_paging_request_says_who_is_called() {
        // Page mode and channels needed, then a length, then the temporary
        // identity's own first octet and its four.
        let mut b = vec![0x15, 0x06, 0x21, 0x00, 0x05, 0xF4, 0x12, 0x34, 0xAB, 0xCD];
        b.resize(23, 0x2B);
        let m = parse(&b).unwrap();
        assert_eq!(m.name, "Paging1");
        assert_eq!(m.pages, vec![Identity::Tmsi(0x1234_ABCD)]);
        assert_eq!(m.pages[0].to_string(), "TMSI 1234ABCD");
    }

    /// Two identities in one request, the second behind its tag, and the
    /// second a permanent identity: a network that pages this way has given
    /// up the point of the temporary one, and the row should say so.
    #[test]
    fn a_request_can_carry_two_and_can_name_a_subscriber() {
        let mut b = vec![0x2D, 0x06, 0x21, 0x00];
        b.extend_from_slice(&[0x05, 0xF4, 0x00, 0x00, 0x00, 0x01]);
        // Tag, length, then an odd count of digits: 272013456789012.
        b.extend_from_slice(&[
            0x17, 0x08, 0x29, 0x27, 0x10, 0x43, 0x65, 0x87, 0x09, 0x21,
        ]);
        b.resize(23, 0x2B);
        let m = parse(&b).unwrap();
        assert_eq!(m.pages.len(), 2);
        assert_eq!(m.pages[0], Identity::Tmsi(1));
        assert_eq!(m.pages[1], Identity::Imsi("272013456789012".into()));
    }

    /// A type 3 carries four temporary identities, and a network with fewer
    /// to page fills the rest with ones rather than leaving them out.
    #[test]
    fn a_type_three_carries_four_and_drops_the_filler() {
        let mut b = vec![0x21, 0x06, 0x24, 0x00];
        b.extend_from_slice(&0xAAAA_AAAAu32.to_be_bytes());
        b.extend_from_slice(&0xBBBB_BBBBu32.to_be_bytes());
        b.extend_from_slice(&u32::MAX.to_be_bytes());
        b.extend_from_slice(&u32::MAX.to_be_bytes());
        b.resize(23, 0x2B);
        let m = parse(&b).unwrap();
        assert_eq!(m.pages, vec![Identity::Tmsi(0xAAAA_AAAA), Identity::Tmsi(0xBBBB_BBBB)]);
    }

    /// An immediate assignment says which channel a phone was sent to and
    /// how far away it is.
    #[test]
    fn an_assignment_names_a_channel_and_a_distance() {
        // Page mode, then a channel description: SDCCH/8 subchannel 3 on
        // timeslot 1, training sequence 7, no hopping, channel 56. Then a
        // request reference and a timing advance of 3.
        let mut b = vec![0x2D, 0x06, 0x3F, 0x00];
        b.extend_from_slice(&[0b0101_1001, 0b1110_0000, 56]);
        b.extend_from_slice(&[0x00, 0x00, 0x00, 0x03]);
        b.resize(23, 0x2B);
        let g = parse(&b).unwrap().grant.expect("a grant");
        assert_eq!(g.kind, "SDCCH/8");
        assert_eq!((g.subchannel, g.timeslot, g.tsc), (3, 1, 7));
        assert_eq!(g.arfcn, Some(56));
        assert_eq!(g.hopping, None);
        assert_eq!(g.timing_advance, 3);
        assert_eq!(g.distance_m(), 1662);
    }

    /// A hopping channel names a sequence and an offset instead of a
    /// frequency, and reading the frequency field anyway would report a
    /// carrier the transaction never touches.
    #[test]
    fn a_hopping_grant_names_its_sequence() {
        let mut b = vec![0x2D, 0x06, 0x3F, 0x00];
        // TCH/F on timeslot 2, training sequence 5, hopping, offset 9,
        // sequence 42.
        b.extend_from_slice(&[0b0000_1010, 0b1011_0010, 0b0110_1010]);
        b.extend_from_slice(&[0x00, 0x00, 0x00, 0x00]);
        b.resize(23, 0x2B);
        let g = parse(&b).unwrap().grant.unwrap();
        assert_eq!((g.kind, g.timeslot, g.tsc), ("TCH/F", 2, 5));
        assert_eq!(g.arfcn, None);
        assert_eq!(g.hopping, Some((9, 42)));
    }

    /// A phone arriving on a signalling channel says where it was and who
    /// it is, before anything is ciphered.
    #[test]
    fn a_location_update_names_the_phone_and_where_it_came_from() {
        // Link layer: signalling service, an unnumbered frame, and a length
        // of 15 octets. Then mobility management, location updating request,
        // the update type, the area the phone was last in, a classmark, and
        // the phone as a length and a temporary identity.
        let mut b = vec![0x01, 0x03, 15 << 2, 0x05, 0x08, 0x70];
        b.extend_from_slice(&[0x62, 0xF2, 0x10, 0x0C, 0x81]);
        b.extend_from_slice(&[0x33]);
        b.extend_from_slice(&[0x05, 0xF4, 0xAA, 0xBB, 0xCC, 0xDD]);
        b.resize(23, 0x2B);
        let m = parse_dedicated(&b).expect("a message");
        assert_eq!(m.name, "LocationUpdatingRequest");
        assert_eq!(m.lai.map(|l| (l.to_string(), l.lac)), Some(("262-01".into(), 0x0C81)));
        assert_eq!(m.identity, Some(Identity::Tmsi(0xAABB_CCDD)));
        assert_eq!(m.sapi, 0);
    }

    /// The exchange that gives a permanent identity away: the network does
    /// not recognise the temporary one and asks.
    #[test]
    fn an_identity_response_carries_what_was_asked_for() {
        let mut b = vec![0x01, 0x03, 11 << 2, 0x05, 0x19, 0x08];
        b.extend_from_slice(&[0x29, 0x27, 0x10, 0x43, 0x65, 0x87, 0x09, 0x21]);
        b.resize(23, 0x2B);
        let m = parse_dedicated(&b).unwrap();
        assert_eq!(m.name, "IdentityResponse");
        assert_eq!(m.identity, Some(Identity::Imsi("272013456789012".into())));
    }

    /// A link layer frame with nothing behind it is not a message. A
    /// dedicated channel is full of these: an acknowledgement, or a fill
    /// frame keeping the link alive.
    #[test]
    fn an_empty_link_frame_is_not_a_message() {
        // A supervisory frame, which acknowledges and carries nothing.
        assert!(parse_dedicated(&[0x01, 0x01, 0x00, 0x2B, 0x2B]).is_none());
        // An unnumbered frame with a length of zero.
        assert!(parse_dedicated(&[0x03, 0x03, 0x01, 0x2B, 0x2B]).is_none());
        assert!(parse_dedicated(&[0x2B; 23]).is_none());
    }

    #[test]
    fn padding_and_other_protocols_are_not_messages() {
        assert_eq!(parse(&[0x2B; 23]), None, "a filler frame is not a decode");
        // Mobility management rather than radio resource: not on this
        // channel, and not this file's to read.
        let mut mm = vec![0x49, 0x05, 0x1B];
        mm.resize(23, 0x2B);
        assert_eq!(parse(&mm), None);
        assert_eq!(parse(&[0x49, 0x06]), None, "a block too short to hold a type");
    }

    /// An unknown message type is refused rather than reported as whatever
    /// the table's last entry was.
    #[test]
    fn an_unknown_message_type_is_refused() {
        let mut b = vec![0x49, 0x06, 0x7E];
        b.resize(23, 0x2B);
        assert_eq!(parse(&b), None);
    }
}
