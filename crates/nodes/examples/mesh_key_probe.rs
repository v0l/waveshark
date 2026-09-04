//! Try a Meshtastic channel key on every logged frame in a file of hex lines.
//!     mesh_key_probe <frames.hex> <channel name> <key hex|base64>
fn main() {
    let a: Vec<String> = std::env::args().collect();
    let text = std::fs::read_to_string(&a[1]).unwrap();
    let psk = decode::channel_keys::parse_key(&a[3]).expect("a key");
    let chan = decode::meshtastic::Channel { name: a[2].clone(), psk: psk.clone() };
    eprintln!("channel {} hash {:?} key {:?}", a[2], chan.hash(), chan.key());
    let (mut n, mut on_hash, mut opened, mut default) = (0, 0, 0, 0);
    for line in text.lines() {
        let bytes: Vec<u8> = (0..line.len() / 2).filter_map(|i| u8::from_str_radix(&line[i * 2..i * 2 + 2], 16).ok()).collect();
        let Some(r) = decode::lora::Received::parse(&bytes) else { continue };
        let Some(m) = r.meshtastic() else { continue };
        n += 1;
        if r.meshtastic_message().is_some() {
            default += 1;
            continue;
        }
        let force = std::env::var_os("ALL").is_some();
        if force || Some(m.channel_hash) == chan.hash() {
            on_hash += 1;
            let ct = &r.payload[decode::lora::Meshtastic::HEADER..];
            if let Some(d) = decode::meshtastic::Decoded::under(ct, m.source, m.packet_id, chan.key().unwrap()) {
                opened += 1;
                eprintln!("  {:08x} -> {:?}", m.source, d.message);
            } else {
                eprintln!("  {:08x} id {:08x} hash {:02x}: {} bytes, did not open", m.source, m.packet_id, m.channel_hash, ct.len());
            }
        }
    }
    eprintln!("{n} meshtastic frames, {default} on the default key, {on_hash} on this channel's hash, {opened} opened");
}
