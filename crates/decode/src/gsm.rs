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
