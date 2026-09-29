pub fn is_url(s: &str) -> bool {
    let s = s.trim_start().to_ascii_lowercase();
    s.starts_with("ws://") || s.starts_with("wss://")
}
