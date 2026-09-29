#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Scheme {
    Ws,
    Wss,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Url {
    pub scheme: Scheme,
    pub host: String,
    pub port: u16,
}

impl Url {
    pub fn parse(url: &str) -> Result<Url, String> {
        let (scheme, rest) = match (url.strip_prefix("wss://"), url.strip_prefix("ws://")) {
            (Some(rest), _) => (Scheme::Wss, rest),
            (None, Some(rest)) => (Scheme::Ws, rest),
            (None, None) => return Err(format!("{url}: not a ws:// or wss:// address")),
        };
        let authority = rest.split(['/', '?', '#']).next().unwrap_or_default();
        let default = match scheme {
            Scheme::Ws => 80,
            Scheme::Wss => 443,
        };
        let (host, port) = match authority.strip_prefix('[') {
            Some(v6) => match v6.split_once(']') {
                Some((host, "")) => (host, None),
                Some((host, port)) => (host, port.strip_prefix(':')),
                None => return Err(format!("{url}: unclosed [")),
            },
            None => match authority.rsplit_once(':') {
                Some((host, port)) => (host, Some(port)),
                None => (authority, None),
            },
        };
        let port = match port {
            Some(p) => p.parse().map_err(|_| format!("{url}: port {p:?} is not a number"))?,
            None => default,
        };
        match host.is_empty() {
            true => Err(format!("{url}: no host")),
            false => Ok(Url { scheme, host: host.to_string(), port }),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_relay_address_names_its_host_and_port() {
        let at = |u: &str| Url::parse(u).map(|u| (u.scheme, u.host, u.port));
        assert_eq!(at("wss://relay.damus.io"), Ok((Scheme::Wss, "relay.damus.io".into(), 443)));
        assert_eq!(at("ws://127.0.0.1:7777/"), Ok((Scheme::Ws, "127.0.0.1".into(), 7777)));
        assert_eq!(at("ws://[::1]:7777/x"), Ok((Scheme::Ws, "::1".into(), 7777)));
        assert_eq!(at("wss://[::1]"), Ok((Scheme::Wss, "::1".into(), 443)));
        assert!(at("https://relay.damus.io").is_err());
        assert!(at("wss://:443").is_err());
        assert!(at("wss://relay:x").is_err());
    }
}
