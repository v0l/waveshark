pub fn from_query(search: &str) -> Vec<String> {
    let pairs: Vec<(String, String)> = search
        .trim_start_matches('?')
        .split('&')
        .filter(|p| !p.is_empty())
        .map(|p| {
            let (k, v) = p.split_once('=').unwrap_or((p, ""));
            (decode(k), decode(v))
        })
        .filter(|(k, _)| !k.is_empty())
        .collect();
    let mut args = vec!["waveshark".to_string()];
    for (k, v) in &pairs {
        args.push(format!("--{}", k.trim_start_matches('-')));
        if !v.is_empty() {
            args.push(v.clone());
        }
    }
    let named = |key: &str| pairs.iter().any(|(k, _)| k.trim_start_matches('-') == key);
    if !named("device")
        && let Some((_, stream)) = pairs.iter().find(|(k, _)| k.trim_start_matches('-') == "stream")
    {
        args.extend(["--device".to_string(), stream.clone()]);
    }
    args
}

fn decode(s: &str) -> String {
    let bytes = s.as_bytes();
    let mut out = Vec::with_capacity(bytes.len());
    let mut i = 0;
    while i < bytes.len() {
        let escaped = (bytes[i] == b'%')
            .then(|| bytes.get(i + 1..i + 3))
            .flatten()
            .and_then(|h| std::str::from_utf8(h).ok())
            .and_then(|h| u8::from_str_radix(h, 16).ok());
        match (escaped, bytes[i]) {
            (Some(b), _) => {
                out.push(b);
                i += 3;
            }
            (None, b'+') => {
                out.push(b' ');
                i += 1;
            }
            (None, b) => {
                out.push(b);
                i += 1;
            }
        }
    }
    String::from_utf8_lossy(&out).into_owned()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_stream_in_the_address_is_offered_and_opened() {
        let key = "ab".repeat(32);
        assert_eq!(
            from_query(&format!("?stream=webrtc%3A%2F%2F{key}&tune=1090")),
            [
                "waveshark",
                "--stream",
                &format!("webrtc://{key}"),
                "--tune",
                "1090",
                "--device",
                &format!("webrtc://{key}")
            ]
        );
    }

    #[test]
    fn a_device_named_in_the_address_wins_over_the_stream() {
        assert_eq!(
            from_query("?stream=mast.local%3A5555&device=hackrf"),
            ["waveshark", "--stream", "mast.local:5555", "--device", "hackrf"]
        );
    }

    #[test]
    fn a_webtransport_address_keeps_its_certificate_hashes() {
        assert_eq!(
            from_query("?stream=https%3A%2F%2Fa.example%3A5556%2F%3Fcert%3Daa%2Cbb")[2],
            "https://a.example:5556/?cert=aa,bb"
        );
    }

    #[test]
    fn a_bare_key_is_a_switch_and_nothing_is_no_arguments() {
        assert_eq!(from_query("?video&tune=474"), ["waveshark", "--video", "--tune", "474"]);
        assert_eq!(from_query(""), ["waveshark"]);
        assert_eq!(from_query("?"), ["waveshark"]);
    }

    #[test]
    fn a_broken_escape_is_kept_as_written() {
        assert_eq!(from_query("?tune=%zz")[2], "%zz");
        assert_eq!(from_query("?tune=1%2")[2], "1%2");
    }
}
