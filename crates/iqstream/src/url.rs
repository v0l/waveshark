pub const SUBPROTOCOL: &str = "iqstream";

pub fn is_url(s: &str) -> bool {
    let s = s.trim_start().to_ascii_lowercase();
    s.starts_with("ws://")
        || s.starts_with("wss://")
        || s.starts_with("https://")
        || s.starts_with(WEBRTC)
}

pub const WEBRTC: &str = "webrtc://";

pub fn webrtc_key(s: &str) -> Option<&str> {
    let key = s.trim().strip_prefix(WEBRTC)?.split(['/', '#', '?']).next()?;
    (key.len() == 64 && key.bytes().all(|b| b.is_ascii_hexdigit())).then_some(key)
}

pub fn is_webtransport(s: &str) -> bool {
    s.trim_start().to_ascii_lowercase().starts_with("https://")
}

pub fn webtransport_url(host_port: &str, hashes: &[String]) -> String {
    format!("https://{host_port}/?cert={}", hashes.join(","))
}

pub fn certificates(url: &str) -> Vec<[u8; 32]> {
    let query = url.split_once('?').map_or("", |(_, q)| q);
    let query = query.split('#').next().unwrap_or_default();
    query
        .split('&')
        .filter_map(|pair| pair.strip_prefix("cert="))
        .flat_map(|v| v.split(','))
        .filter_map(unhex)
        .collect()
}

fn unhex(s: &str) -> Option<[u8; 32]> {
    let s = s.trim();
    let mut out = [0u8; 32];
    if s.len() != 64 {
        return None;
    }
    for (i, b) in out.iter_mut().enumerate() {
        *b = u8::from_str_radix(s.get(i * 2..i * 2 + 2)?, 16).ok()?;
    }
    Some(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_webtransport_address_carries_the_certificates_it_will_accept() {
        let a = "ab".repeat(32);
        let b = "0f".repeat(32);
        let url = webtransport_url("radarpi.example:1235", &[a.clone(), b.clone()]);
        assert_eq!(url, format!("https://radarpi.example:1235/?cert={a},{b}"));
        assert!(is_url(&url) && is_webtransport(&url));
        assert_eq!(certificates(&url), vec![[0xab; 32], [0x0f; 32]]);
        assert_eq!(certificates(&format!("{url}#2")), vec![[0xab; 32], [0x0f; 32]]);
        assert_eq!(certificates("https://h:1/?cert=zz"), Vec::<[u8; 32]>::new());
        assert!(!is_webtransport("wss://h:1/"));
    }

    #[test]
    fn a_webrtc_address_is_the_listing_key_it_signals_to() {
        let key = "ab".repeat(32);
        assert!(is_url(&format!("webrtc://{key}")));
        assert_eq!(webrtc_key(&format!("webrtc://{key}#2")), Some(key.as_str()));
        assert_eq!(webrtc_key("webrtc://nope"), None);
        assert_eq!(webrtc_key(&format!("https://{key}")), None);
    }
}
