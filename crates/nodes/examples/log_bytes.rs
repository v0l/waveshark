//! Print the analysed bytes of every pulse packet in a .wspkt log:
//!     log_bytes <file.wspkt> [hex prefix]
fn main() {
    let a: Vec<String> = std::env::args().collect();
    let buf = std::fs::read(&a[1]).unwrap();
    let want: Vec<u8> = a.get(2).map(|h| (0..h.len() / 2).map(|i| u8::from_str_radix(&h[i * 2..i * 2 + 2], 16).unwrap()).collect()).unwrap_or_default();
    // Minimal reader of the log format, mirroring app::packetlog.
    let mut at = 8;
    while at + 4 <= buf.len() {
        let len = u32::from_le_bytes(buf[at..at + 4].try_into().unwrap()) as usize;
        at += 4;
        if at + len > buf.len() { break; }
        let r = &buf[at..at + len];
        at += len;
        let kind = r[0];
        let keying = r[1];
        let count = u16::from_le_bytes([r[2], r[3]]) as usize;
        let at_us = u64::from_le_bytes(r[4..12].try_into().unwrap());
        let hz = u64::from_le_bytes(r[12..20].try_into().unwrap());
        let snr = f32::from_le_bytes(r[28..32].try_into().unwrap());
        let mut body = &r[32..];
        if kind == 3 {
            // skip measure: 3 strings, f32, u32, 5 f32
            let mut o = 0;
            for _ in 0..3 { let n = u16::from_le_bytes([body[o], body[o + 1]]) as usize; o += 2 + n; }
            o += 4 + 4 + 20;
            body = &body[o..];
        }
        if kind != 1 && kind != 3 { continue; }
        let pulses: Vec<common::Pulse> = (0..count.min(body.len() / 8)).map(|k| common::Pulse {
            mark: u32::from_le_bytes(body[k * 8..k * 8 + 4].try_into().unwrap()),
            gap: u32::from_le_bytes(body[k * 8 + 4..k * 8 + 8].try_into().unwrap()),
        }).collect();
        let pkg = common::Package { center_hz: hz, pulses, rssi_dbfs: 0.0, snr_db: snr, modulation: None, ..Default::default() };
        let Some(an) = decode::analyze(&pkg) else { continue };
        let bytes = an.frame_bytes();
        if !want.is_empty() && !bytes.starts_with(&want) { continue; }
        let secs = at_us / 1_000_000 % 86_400;
        let ascii: String = bytes.iter().map(|b| if b.is_ascii_graphic() { *b as char } else { '.' }).collect();
        println!("{:02}:{:02}:{:02} {:10.4} MHz key{} snr {:5.1} {:3} B {}  {}  | {}",
            secs / 3600, secs / 60 % 60, secs % 60, hz as f64 / 1e6, keying, snr, bytes.len(),
            bytes.iter().map(|b| format!("{b:02x}")).collect::<String>(), ascii, an.summary());
    }
}
