//! Reading a Meshtastic node's channels over its HTTP API.
//!
//! A node on the network answers `PUT /api/v1/toradio` with a `ToRadio`
//! protobuf and hands its configuration back a message at a time from
//! `GET /api/v1/fromradio`, ending with the id the request carried. Among
//! those messages are its channels, name and PSK included, which is the
//! copy of a key that cannot be mistyped: the one in the radio.
//!
//! The two messages needed are decoded by hand. A channel is three fields
//! deep and the alternative is a protobuf dependency and a generated file
//! for the sake of one name and one byte string.

use std::time::Duration;

/// A channel the node holds, as the node holds it.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Channel {
    pub index: u32,
    pub name: String,
    /// Empty for no encryption, one byte for an index into the default
    /// key, and the key itself otherwise.
    pub psk: Vec<u8>,
}

impl Channel {
    /// Whether the PSK is a key rather than a reference to the built-in
    /// one, which is already tried on everything.
    pub fn has_own_key(&self) -> bool {
        self.psk.len() > 1
    }
}

/// The node's channels, over the network.
///
/// `host` is what was typed: an address, with or without a scheme or a
/// port. The read is bounded by the request timeout and by how many
/// messages a node can send before its configuration is complete.
pub async fn channels(host: String) -> Result<Vec<Channel>, String> {
    let host = host.as_str();
    let base = base_url(host);
    let http = reqwest::Client::builder()
        .timeout(Duration::from_secs(8))
        .build()
        .map_err(|e| e.to_string())?;
    // ToRadio.want_config_id, field 3, varint. Any nonce does; the node
    // echoes it in FromRadio.config_complete_id, field 7.
    const NONCE: u64 = 0x2a;
    let mut want = vec![0x18];
    put_varint(&mut want, NONCE);
    http.put(format!("{base}/api/v1/toradio"))
        .header("Content-Type", "application/x-protobuf")
        .body(want)
        .send()
        .await
        .map_err(|e| format!("{host}: {e}"))?
        .error_for_status()
        .map_err(|e| format!("{host}: {e}"))?;

    let mut out = Vec::new();
    // A node sends its node database as well as its configuration, and a
    // busy mesh has hundreds of entries; the cap is a guard against a node
    // that never says it is done, not a working limit.
    for _ in 0..2000 {
        let body = http
            .get(format!("{base}/api/v1/fromradio?all=false"))
            .send()
            .await
            .map_err(|e| format!("{host}: {e}"))?
            .bytes()
            .await
            .map_err(|e| format!("{host}: {e}"))?;
        if body.is_empty() {
            tokio::time::sleep(Duration::from_millis(100)).await;
            continue;
        }
        match from_radio(&body) {
            FromRadio::Channel(c) => out.push(c),
            FromRadio::ConfigComplete(id) if id == NONCE => return Ok(out),
            _ => {}
        }
    }
    Err(format!("{host}: the node never finished sending its configuration"))
}

fn base_url(host: &str) -> String {
    let h = host.trim().trim_end_matches('/');
    if h.starts_with("http://") || h.starts_with("https://") {
        h.to_string()
    } else {
        format!("http://{h}")
    }
}

enum FromRadio {
    Channel(Channel),
    ConfigComplete(u64),
    Other,
}

/// One `FromRadio` message: field 10 is a `Channel`, field 7 the
/// completion id.
fn from_radio(b: &[u8]) -> FromRadio {
    for (field, wire) in fields(b) {
        match (field, wire) {
            (7, Wire::Varint(v)) => return FromRadio::ConfigComplete(v),
            (10, Wire::Bytes(c)) => return FromRadio::Channel(channel(c)),
            _ => {}
        }
    }
    FromRadio::Other
}

/// A `Channel`: field 1 the index, field 2 the `ChannelSettings`, whose
/// field 2 is the PSK and field 3 the name.
fn channel(b: &[u8]) -> Channel {
    let mut c = Channel { index: 0, name: String::new(), psk: Vec::new() };
    for (field, wire) in fields(b) {
        match (field, wire) {
            (1, Wire::Varint(v)) => c.index = v as u32,
            (2, Wire::Bytes(s)) => {
                for (f, w) in fields(s) {
                    match (f, w) {
                        (2, Wire::Bytes(p)) => c.psk = p.to_vec(),
                        (3, Wire::Bytes(n)) => c.name = String::from_utf8_lossy(n).into_owned(),
                        _ => {}
                    }
                }
            }
            _ => {}
        }
    }
    c
}

enum Wire<'a> {
    Varint(u64),
    Bytes(&'a [u8]),
    Fixed,
}

/// The fields of one message, in order. Stops at the first thing that
/// does not parse rather than guessing past it.
fn fields(b: &[u8]) -> Vec<(u32, Wire<'_>)> {
    let mut out = Vec::new();
    let mut at = 0;
    while at < b.len() {
        let Some((key, next)) = varint(b, at) else { break };
        at = next;
        let field = (key >> 3) as u32;
        match key & 7 {
            0 => {
                let Some((v, next)) = varint(b, at) else { break };
                at = next;
                out.push((field, Wire::Varint(v)));
            }
            1 => {
                at += 8;
                out.push((field, Wire::Fixed));
            }
            2 => {
                let Some((n, next)) = varint(b, at) else { break };
                let Some(s) = b.get(next..next + n as usize) else { break };
                at = next + n as usize;
                out.push((field, Wire::Bytes(s)));
            }
            5 => {
                at += 4;
                out.push((field, Wire::Fixed));
            }
            _ => break,
        }
    }
    out
}

fn varint(b: &[u8], mut at: usize) -> Option<(u64, usize)> {
    let mut v = 0u64;
    for shift in (0..64).step_by(7) {
        let x = *b.get(at)?;
        at += 1;
        v |= u64::from(x & 0x7f) << shift;
        if x < 0x80 {
            return Some((v, at));
        }
    }
    None
}

fn put_varint(out: &mut Vec<u8>, mut v: u64) {
    while v >= 0x80 {
        out.push((v as u8 & 0x7f) | 0x80);
        v >>= 7;
    }
    out.push(v as u8);
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A `FromRadio` as a node on the bench sent it: channel 1, named
    /// "waveshark", with a 24-byte PSK. The node's primary channel came as
    /// an index-only PSK, and the rest as bare indexes with no settings.
    #[test]
    fn a_channel_reads_out_of_the_node_message() {
        let psk = b"\xef\x50\x36\xdb\x9d\xf7\xeb\xae\x38\x00\x5d\xb8\xf0\x3e\xf9\xdf\x7f\x42\xd8\x5e\x35\xe8\x0d\x7a";
        let mut settings = vec![0x12, psk.len() as u8];
        settings.extend_from_slice(psk);
        settings.extend_from_slice(&[0x1a, 9]);
        settings.extend_from_slice(b"waveshark");
        let mut ch = vec![0x08, 1, 0x12, settings.len() as u8];
        ch.extend_from_slice(&settings);
        ch.extend_from_slice(&[0x18, 2]);
        let mut msg = vec![0x52, ch.len() as u8];
        msg.extend_from_slice(&ch);
        let FromRadio::Channel(c) = from_radio(&msg) else { panic!("not a channel") };
        assert_eq!(c, Channel { index: 1, name: "waveshark".into(), psk: psk.to_vec() });
        assert!(c.has_own_key());

        let primary = [0x52, 6, 0x12, 4, 0x12, 1, 0x01, 0x18, 1];
        let FromRadio::Channel(c) = from_radio(&primary) else { panic!("not a channel") };
        assert_eq!(c.psk, vec![1]);
        assert!(!c.has_own_key());

        assert!(matches!(from_radio(&[0x38, 0x2a]), FromRadio::ConfigComplete(0x2a)));
        assert!(matches!(from_radio(&[0x0a, 2, 0x08, 1]), FromRadio::Other));
    }

    #[test]
    fn a_host_gets_a_scheme_and_keeps_one() {
        assert_eq!(base_url("10.100.2.252"), "http://10.100.2.252");
        assert_eq!(base_url(" 10.100.2.252/ "), "http://10.100.2.252");
        assert_eq!(base_url("https://node.local:8443/"), "https://node.local:8443");
    }
}

#[cfg(test)]
mod live {
    /// Against a node on the bench; `MESH_NODE=10.0.0.5 cargo test -- --ignored live`.
    #[test]
    #[ignore]
    fn a_node_hands_over_its_channels() {
        let host = std::env::var("MESH_NODE").expect("MESH_NODE");
        let rt = tokio::runtime::Runtime::new().unwrap();
        let chans = rt.block_on(super::channels(host)).unwrap();
        for c in &chans {
            eprintln!("{} {:?} {}", c.index, c.name, decode::channel_keys::hex(&c.psk));
        }
        assert!(chans.iter().any(|c| c.has_own_key()));
    }
}
