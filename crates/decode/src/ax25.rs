//! AX.25 frames, the link layer APRS rides on.
//!
//! The frame layer only: bytes in, addresses and an information field out.
//! What the information field *means* is [`crate::aprs`]'s problem, and the
//! demodulator that produced the bytes is `dsp::afsk`'s. AX.25 carries plenty
//! that is not APRS, so the split is not academic.
//!
//! A frame is addresses, a control byte, a protocol identifier, then the
//! information field:
//!
//! ```text
//! DEST7 SRC7 [DIGI7]*0..8  CTRL1 PID1  INFO*
//! ```
//!
//! # Callsigns are shifted
//!
//! Every address byte is shifted left by one bit, because the low bit is
//! needed as the flag marking the last address in the list. So a callsign
//! character is `byte >> 1`, and a decoder that forgets this gets plausible
//! looking garbage rather than an obvious failure: 'A' becomes 0x82, which is
//! still a byte.

/// One address: a callsign and its substation identifier.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Address {
    pub call: String,
    /// 0 to 15. Distinguishes a station's several transmitters, so `EI2ABC-9`
    /// is conventionally the one in a vehicle.
    pub ssid: u8,
    /// For a digipeater, whether it has already repeated the frame.
    pub repeated: bool,
}

impl std::fmt::Display for Address {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        if self.ssid == 0 {
            write!(f, "{}", self.call)
        } else {
            write!(f, "{}-{}", self.call, self.ssid)
        }
    }
}

impl std::str::FromStr for Address {
    type Err = ();

    /// `EI2ABC-9`, or `EI2ABC` for SSID zero, and a trailing `*` on a path
    /// address for one that has already repeated the frame, which is how
    /// every APRS log prints one.
    fn from_str(s: &str) -> Result<Self, ()> {
        let s = s.trim();
        let (s, repeated) = match s.strip_suffix('*') {
            Some(rest) => (rest, true),
            None => (s, false),
        };
        let (call, ssid) = match s.split_once('-') {
            Some((c, n)) => (c, n.parse::<u8>().map_err(|_| ())?),
            None => (s, 0),
        };
        let call = call.trim().to_ascii_uppercase();
        if call.is_empty() || call.len() > 6 || !call.bytes().all(|b| b.is_ascii_alphanumeric()) {
            return Err(());
        }
        match ssid > 15 {
            true => Err(()),
            false => Ok(Address { call, ssid, repeated }),
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct Frame {
    pub destination: Address,
    pub source: Address,
    /// The path the frame took, in order. Empty for a direct transmission.
    pub path: Vec<Address>,
    pub control: u8,
    pub pid: Option<u8>,
    pub info: Vec<u8>,
}

impl Frame {
    /// Whether this is an unnumbered information frame, which is what APRS
    /// uses and the only kind carrying a payload worth reading here.
    pub fn is_ui(&self) -> bool {
        // The control field's low bits identify the frame type; UI is 0x03
        // with the poll/final bit possibly set.
        self.control & 0xEF == 0x03
    }

    /// The information field as text, when it is text.
    pub fn info_text(&self) -> String {
        self.info.iter().map(|&b| if (32..127).contains(&b) { b as char } else { '.' }).collect()
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ParseError {
    /// Not enough bytes for two addresses, a control byte and a PID.
    TooShort,
    /// The address list never terminated, or ran past its limit of eight
    /// digipeaters.
    BadAddresses,
}

/// Addresses are seven bytes: six characters and an SSID byte.
const ADDR_LEN: usize = 7;
/// Two addresses, a control byte and a PID is the shortest useful frame.
const MIN_LEN: usize = ADDR_LEN * 2 + 2;
/// The standard allows at most eight digipeaters in the path.
const MAX_PATH: usize = 8;

fn address(b: &[u8]) -> Address {
    // Six characters, each shifted left by one bit, space padded.
    let call: String =
        b[..6].iter().map(|c| (c >> 1) as char).collect::<String>().trim_end().to_string();
    Address {
        call,
        ssid: (b[6] >> 1) & 0x0F,
        // The top bit is the "has been repeated" flag on a digipeater
        // address, and the command/response bit on the first two.
        repeated: b[6] & 0x80 != 0,
    }
}

/// Whether this address byte terminates the address list.
fn last(b: &[u8]) -> bool {
    b[6] & 1 != 0
}

pub fn parse(bytes: &[u8]) -> Result<Frame, ParseError> {
    if bytes.len() < MIN_LEN {
        return Err(ParseError::TooShort);
    }
    let destination = address(&bytes[0..ADDR_LEN]);
    let source = address(&bytes[ADDR_LEN..ADDR_LEN * 2]);

    let mut at = ADDR_LEN * 2;
    let mut path = Vec::new();
    if !last(&bytes[ADDR_LEN..]) {
        loop {
            if at + ADDR_LEN > bytes.len() || path.len() > MAX_PATH {
                return Err(ParseError::BadAddresses);
            }
            let a = &bytes[at..at + ADDR_LEN];
            path.push(address(a));
            at += ADDR_LEN;
            if last(a) {
                break;
            }
        }
    }

    if at >= bytes.len() {
        return Err(ParseError::TooShort);
    }
    let control = bytes[at];
    at += 1;
    // Only frames that carry one have a protocol identifier; for the
    // unnumbered information frames APRS uses, it is always present.
    let pid = bytes.get(at).copied();
    if pid.is_some() {
        at += 1;
    }
    Ok(Frame {
        destination,
        source,
        path,
        control,
        pid,
        info: bytes.get(at..).unwrap_or(&[]).to_vec(),
    })
}

/// An unnumbered information frame carrying `info`, ready for the check
/// sequence and the flags `dsp::hdlc::encode_frame` puts around it.
///
/// The inverse of [`parse`] for the one frame type APRS sends. The address
/// bytes are shifted left the way the parser unshifts them, the low bit of
/// the last one terminates the list, and the two bits above the SSID are the
/// reserved ones every station sets.
pub fn encode(destination: &Address, source: &Address, path: &[Address], info: &[u8]) -> Vec<u8> {
    let mut out = Vec::with_capacity(ADDR_LEN * (2 + path.len()) + 2 + info.len());
    let all: Vec<&Address> = [destination, source].into_iter().chain(path).collect();
    let n = all.len();
    for (i, a) in all.into_iter().enumerate() {
        for c in format!("{:<6}", a.call).bytes().take(6) {
            out.push(c << 1);
        }
        out.push(
            0x60 | (a.ssid & 0x0F) << 1 | u8::from(i + 1 == n) | if a.repeated { 0x80 } else { 0 },
        );
    }
    // UI, and no layer 3 above, which is what APRS declares.
    out.push(0x03);
    out.push(0xF0);
    out.extend_from_slice(info);
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    fn addr(call: &str, ssid: u8) -> Address {
        Address { call: call.into(), ssid, repeated: false }
    }

    fn build(dest: (&str, u8), src: (&str, u8), path: &[(&str, u8)], info: &[u8]) -> Vec<u8> {
        let path: Vec<Address> = path.iter().map(|(c, s)| addr(c, *s)).collect();
        encode(&addr(dest.0, dest.1), &addr(src.0, src.1), &path, info)
    }

    /// The wire format, spelled out, so the encoder these tests build their
    /// frames with is checked against the bytes rather than against the
    /// parser that shares its assumptions.
    #[test]
    fn an_encoded_frame_is_the_bytes_that_go_on_the_air() {
        let raw = build(("APRS", 0), ("EI2ABC", 9), &[], b"hi");
        assert_eq!(&raw[..6], &[b'A' << 1, b'P' << 1, b'R' << 1, b'S' << 1, b' ' << 1, b' ' << 1]);
        assert_eq!(raw[6], 0x60, "the destination is not the last address");
        assert_eq!(raw[13], 0x60 | 9 << 1 | 1, "the source ends the list");
        assert_eq!(&raw[14..], &[0x03, 0xF0, b'h', b'i']);
    }

    #[test]
    fn a_callsign_parses_once_with_its_ssid_and_its_repeated_mark() {
        use std::str::FromStr;
        assert_eq!(Address::from_str("EI2ABC-9").unwrap(), addr("EI2ABC", 9));
        assert_eq!(Address::from_str("aprs").unwrap(), addr("APRS", 0));
        assert_eq!(
            Address::from_str("WIDE1-1*").unwrap(),
            Address { call: "WIDE1".into(), ssid: 1, repeated: true }
        );
        assert!(Address::from_str("TOOLONGCALL").is_err());
        assert!(Address::from_str("EI2ABC-16").is_err());
        assert!(Address::from_str("").is_err());
    }

    #[test]
    fn a_direct_frame_gives_both_callsigns_and_the_payload() {
        let raw = build(("APRS", 0), ("EI2ABC", 9), &[], b"hello");
        let f = parse(&raw).unwrap();
        assert_eq!(f.destination.call, "APRS");
        assert_eq!(f.source.call, "EI2ABC");
        assert_eq!(f.source.ssid, 9);
        assert_eq!(f.source.to_string(), "EI2ABC-9");
        assert!(f.path.is_empty(), "a direct frame has no path");
        assert!(f.is_ui());
        assert_eq!(f.pid, Some(0xF0));
        assert_eq!(f.info, b"hello");
    }

    /// The shift is the trap. Without it every callsign is garbage that still
    /// looks like bytes, so this checks the characters and not just the
    /// length.
    #[test]
    fn callsign_characters_are_unshifted() {
        let raw = build(("APRS", 0), ("EI2ABC", 0), &[], b"x");
        // Every address byte really is shifted on the wire.
        assert_eq!(raw[0], b'A' << 1);
        assert_eq!(parse(&raw).unwrap().destination.call, "APRS");
    }

    #[test]
    fn a_digipeated_frame_keeps_its_path_in_order() {
        let raw = build(("APRS", 0), ("EI2ABC", 9), &[("WIDE1", 1), ("WIDE2", 2)], b"x");
        let f = parse(&raw).unwrap();
        let path: Vec<String> = f.path.iter().map(|a| a.to_string()).collect();
        assert_eq!(path, vec!["WIDE1-1", "WIDE2-2"]);
    }

    /// A truncated frame must not panic or read past the end. The air
    /// produces these constantly.
    #[test]
    fn a_short_frame_is_refused_rather_than_read_past() {
        assert_eq!(parse(&[]), Err(ParseError::TooShort));
        assert_eq!(parse(&[0u8; 10]), Err(ParseError::TooShort));
        // Address list that never terminates.
        let mut raw = vec![0u8; 16];
        for b in raw.iter_mut().skip(6).step_by(7) {
            *b = 0x60;
        }
        assert!(parse(&raw).is_err());
    }

    #[test]
    fn a_path_longer_than_the_standard_allows_is_refused() {
        // Ten digipeaters, none of them terminating early.
        let mut raw = Vec::new();
        for _ in 0..12 {
            raw.extend_from_slice(&[b'A' << 1; 6]);
            raw.push(0x60);
        }
        assert_eq!(parse(&raw), Err(ParseError::BadAddresses));
    }
}
