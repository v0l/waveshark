//! Wireless M-Bus link and transport layers: what a meter's frame says
//! about itself before the part that is usually encrypted.
//!
//! A frame is the length, a control byte, the manufacturer as three letters
//! packed into two bytes, the meter's number in binary-coded decimal, its
//! version and what kind of meter it is, then a control information byte
//! that says what follows: a short transport header with the access
//! number, status and configuration word, a long one with a second address
//! in front of that, or an extended link layer wrapped around either. The
//! configuration word says whether the payload is encrypted, and it nearly
//! always is: a utility's readings are under AES with a key the utility
//! holds. So what can be reported without the key is who transmitted, what
//! it is, and the bytes as they arrived, which is what rtl_433 reports too.

use crate::protocol::Report;

/// Meter types of EN 13757-3, in the words rtl_433 uses for them.
pub fn device_type(t: u8) -> &'static str {
    match t {
        0x00 => "Other",
        0x01 => "Oil",
        0x02 => "Electricity",
        0x03 => "Gas",
        0x04 => "Heat",
        0x05 => "Steam",
        0x06 => "Warm Water",
        0x07 => "Water",
        0x08 => "Heat Cost Allocator",
        0x09 => "Compressed Air",
        0x0A | 0x0B => "Cooling load meter",
        0x0C => "Heat",
        0x0D => "Heat / Cooling load meter",
        0x0E => "Bus/System component",
        0x0F => "Unknown",
        0x15 => "Hot water",
        0x16 => "Cold Water",
        0x17 => "Dual register (hot/cold) Water",
        0x18 => "Pressure",
        0x19 => "A/D Converter",
        0x1A => "Smoke detector",
        0x1B => "Room sensor",
        0x1C => "Gas detector",
        0x20 => "Breaker (electricity)",
        0x21 => "Valve (gas or water)",
        0x25 => "Customer unit",
        0x28 => "Waste water",
        0x29 => "Garbage",
        0x30 => "Service tool",
        0x31 => "Gateway",
        0x32 => "Unidirectional repeater",
        0x33 => "Bidirectional repeater",
        0x36 => "Radio converter (system side)",
        0x37 => "Radio converter (meter side)",
        _ => "",
    }
}

/// The manufacturer field as its three letters: five bits each, A being 1.
pub fn manufacturer(m: u16) -> String {
    let letter = |v: u16| char::from(((v & 0x1f) as u8).wrapping_add(64));
    [letter(m >> 10), letter(m >> 5), letter(m)].into_iter().collect()
}

/// Binary-coded decimal, least significant byte first, as the number
/// printed on the meter.
pub fn bcd_id(a: &[u8]) -> u64 {
    a.iter().rev().fold(0u64, |acc, b| acc * 100 + ((b >> 4) as u64) * 10 + (b & 0xf) as u64)
}

/// Parse a frame's bytes from the length field onward, CRCs removed, as the
/// demodulator hands them over. `mode` is the mode's letter where the
/// receiver knows it.
pub fn parse(bytes: &[u8], mode: Option<&str>) -> Option<Report> {
    if bytes.len() < 10 {
        return None;
    }
    let l = bytes[0] as usize;
    let c = bytes[1];
    let m = u16::from_le_bytes([bytes[2], bytes[3]]);
    let id = bcd_id(&bytes[4..8]);
    let version = bytes[8];
    let kind = bytes[9];
    let mut r = Report::new("Wireless-MBus");
    r.crc_valid = Some(true);
    r.raw = bytes.to_vec();
    if let Some(mode) = mode {
        r = r.text("mode", mode);
    }
    r = r
        .text("M", manufacturer(m))
        .int("id", id as i64)
        .int("version", version as i64)
        .int("type", kind as i64)
        .text("type_string", device_type(kind))
        .int("C", c as i64)
        .int("L", l as i64)
        .text("data", hex(bytes));

    // Past the address: the control information byte and its header.
    let mut at = 10usize;
    let Some(&ci) = bytes.get(at) else { return Some(r) };
    at += 1;
    // An extended link layer wraps the rest: CC and ACC, and for 0x8D a
    // session number and a payload CRC, then the real CI.
    let mut ci = ci;
    if ci == 0x8C || ci == 0x8D {
        r = r.int("ell_ci", ci as i64);
        if let (Some(&cc), Some(&acc)) = (bytes.get(at), bytes.get(at + 1)) {
            r = r.int("ell_cc", cc as i64).int("ell_acc", acc as i64);
        }
        at += 2;
        if ci == 0x8D {
            if let Some(sn) = bytes.get(at..at + 4) {
                r = r.int("ell_sn", u32::from_le_bytes([sn[0], sn[1], sn[2], sn[3]]) as i64);
            }
            at += 6;
        }
        match bytes.get(at) {
            Some(&inner) => {
                ci = inner;
                at += 1;
            }
            None => return Some(r),
        }
    }
    r = r.int("CI", ci as i64);
    let header = match ci {
        // Short transport header: access number, status, configuration.
        0x7A | 0x7B | 0x7D | 0x7F | 0x8A => Some(0),
        // Long transport header: a second address first.
        0x72 | 0x73 | 0x75 | 0x7C | 0x7E | 0x8B => Some(8),
        _ => None,
    };
    if let Some(skip) = header {
        let h = at + skip;
        if let Some(hdr) = bytes.get(h..h + 4) {
            let cw = u16::from_le_bytes([hdr[2], hdr[3]]);
            r = r.int("AC", hdr[0] as i64).int("ST", hdr[1] as i64).int("CW", cw as i64);
            // Bits 8 to 12 of the configuration word are the encryption
            // mode; zero is none, and five is the AES-128 CBC a utility's
            // meters use.
            let enc_mode = (cw >> 8) & 0x1f;
            if enc_mode != 0 {
                r = r.int("payload_encrypted", 1).int("encryption_mode", enc_mode as i64);
            }
        }
    }
    Some(r)
}

fn hex(b: &[u8]) -> String {
    b.iter().map(|x| format!("{x:02x}")).collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protocol::Value;

    fn unhex(s: &str) -> Vec<u8> {
        (0..s.len()).step_by(2).map(|i| u8::from_str_radix(&s[i..i + 2], 16).unwrap()).collect()
    }

    #[test]
    fn a_diehl_water_meter_reads_as_rtl_433_reads_it() {
        // rtl_433's mode T recording of a Diehl Hydrus, CRCs removed.
        let f = unhex("5344a5112901858476078c00ae900f002c25f00c2f005d8c2c1dac2ca7c07a3a80310710a7f26ca73e8a384744684fe6a79dd0844ebe8c89debb0615906f9f9581b60dbf73e59f525cbc0182172ac76923f254d4");
        let r = parse(&f, Some("T")).unwrap();
        assert_eq!(r.get("M"), Some(&Value::Text("DME".into())));
        assert_eq!(r.get("id"), Some(&Value::Int(84850129)));
        assert_eq!(r.get("version"), Some(&Value::Int(118)));
        assert_eq!(r.get("type"), Some(&Value::Int(7)));
        assert_eq!(r.get("type_string"), Some(&Value::Text("Water".into())));
        assert_eq!(r.get("C"), Some(&Value::Int(68)));
        // The extended link layer here (ell_ci 0x8c) is recognised; the
        // inner CI past it needs offsets this decoder does not yet place,
        // which is why the C-mode ELL captures are not decoded further.
        assert_eq!(r.get("ell_ci"), Some(&Value::Int(140)));
    }

    #[test]
    fn a_meter_behind_a_repeater_has_the_long_header() {
        let f = unhex("4644b42571550210050e7287545505b42501079a003025403e848957876e48759da51bd3f945751967d301a2254d6a2851fd29931b624681f21e8106633cc25a6e3e8a06812405");
        let r = parse(&f, Some("T")).unwrap();
        assert_eq!(r.get("M"), Some(&Value::Text("IMT".into())));
        assert_eq!(r.get("id"), Some(&Value::Int(10025571)));
        assert_eq!(r.get("type_string"), Some(&Value::Text("Bus/System component".into())));
        assert_eq!(r.get("CI"), Some(&Value::Int(114)));
        assert_eq!(r.get("AC"), Some(&Value::Int(154)));
        assert_eq!(r.get("ST"), Some(&Value::Int(0)));
        assert_eq!(r.get("CW"), Some(&Value::Int(9520)));
        assert_eq!(r.get("payload_encrypted"), Some(&Value::Int(1)));
    }

    #[test]
    fn a_kamstrup_heat_meter_in_mode_c() {
        // Format B, so the CRCs the recording carried inside the length are
        // gone here and the length field is left as it was sent.
        let f = unhex("41442d2c32839760190c8d20bb901f3522d30883bdbfd4eac25b78dcb20a964d8fa3a27b9efe2a38d6a160cc2bdfb310f64faaa672b37d7ad91c9aa244111a78");
        let mut without = f[..10].to_vec();
        without.extend_from_slice(&f[12..f.len() - 2]);
        let r = parse(&without, Some("C")).unwrap();
        assert_eq!(r.get("M"), Some(&Value::Text("KAM".into())));
        assert_eq!(r.get("id"), Some(&Value::Int(60978332)));
        assert_eq!(r.get("type_string"), Some(&Value::Text("Heat".into())));
    }

    #[test]
    fn the_manufacturer_letters_come_from_five_bit_fields() {
        assert_eq!(manufacturer(0x11a5), "DME");
        assert_eq!(manufacturer(0x2c2d), "KAM");
        assert_eq!(bcd_id(&[0x29, 0x01, 0x85, 0x84]), 84850129);
    }
}
