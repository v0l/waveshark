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

    let (name, ident) = match type_id {
        0x19 => ("SI1", Ident::None),
        0x1A => ("SI2", Ident::None),
        0x02 => ("SI2bis", Ident::None),
        0x03 => ("SI2ter", Ident::None),
        0x07 => ("SI2quater", Ident::None),
        // The two that carry the cell's own identity, and the reason this
        // file exists: type 3 on the broadcast channel, type 6 on the slow
        // associated channel of a call in progress.
        0x1B => ("SI3", Ident::CellAndArea),
        0x1E => ("SI6", Ident::CellAndArea),
        0x1C => ("SI4", Ident::AreaOnly),
        0x1D => ("SI5", Ident::None),
        0x05 => ("SI5bis", Ident::None),
        0x06 => ("SI5ter", Ident::None),
        0x00 => ("SI13", Ident::None),
        0x21 => ("Paging1", Ident::None),
        0x22 => ("Paging2", Ident::None),
        0x24 => ("Paging3", Ident::None),
        0x3F => ("ImmediateAssign", Ident::None),
        0x39 => ("ImmediateAssignExt", Ident::None),
        0x3A => ("ImmediateAssignReject", Ident::None),
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
    Some(Message { name, type_id, cell_id, lai })
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
