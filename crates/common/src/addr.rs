use std::net::{IpAddr, Ipv4Addr, Ipv6Addr, SocketAddr};

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum AddrError {
    Empty,
    Space,
    NoHost,
    NoPort,
    Port(String),
    Ipv6(String),
    NotAnAddress(String),
}

impl std::fmt::Display for AddrError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Empty => f.write_str("nothing typed"),
            Self::Space => f.write_str("an address has no spaces in it"),
            Self::NoHost => f.write_str("no host before the port"),
            Self::NoPort => f.write_str("no port"),
            Self::Port(p) => write!(f, "{p:?} is not a port, which is 1 to 65535"),
            Self::Ipv6(h) => write!(f, "{h:?} is not an IPv6 address"),
            Self::NotAnAddress(h) => {
                write!(f, "{h:?} is not an IP address, and only localhost is looked up here")
            }
        }
    }
}

impl std::error::Error for AddrError {}

#[derive(Clone, Debug, PartialEq, Eq, Hash)]
pub struct HostPort {
    pub host: String,
    pub port: u16,
}

impl HostPort {
    pub fn parse(s: &str, default_port: u16) -> Result<Self, AddrError> {
        let (host, port) = split(s)?;
        let port = match port {
            Some(p) => port_of(p)?,
            None => default_port,
        };
        Ok(Self { host: host.to_string(), port })
    }
}

impl std::fmt::Display for HostPort {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&join(&self.host, self.port))
    }
}

pub fn join(host: &str, port: u16) -> String {
    match host.contains(':') {
        true => format!("[{host}]:{port}"),
        false => format!("{host}:{port}"),
    }
}

pub fn port(s: &str) -> Result<u16, AddrError> {
    match s.trim() {
        "" => Err(AddrError::Empty),
        t if t.contains(char::is_whitespace) => Err(AddrError::Space),
        t => port_of(t),
    }
}

pub fn listen(s: &str, bare_port: IpAddr) -> Result<SocketAddr, AddrError> {
    let t = s.trim();
    if !t.is_empty() && t.bytes().all(|b| b.is_ascii_digit()) {
        return Ok(SocketAddr::new(bare_port, listen_port(t)?));
    }
    let (host, port) = split(t)?;
    let ip = match host {
        h if h.eq_ignore_ascii_case("localhost") => IpAddr::V4(Ipv4Addr::LOCALHOST),
        h => h.parse().map_err(|_| AddrError::NotAnAddress(h.to_string()))?,
    };
    let port = listen_port(port.ok_or(AddrError::NoPort)?)?;
    Ok(SocketAddr::new(ip, port))
}

fn split(s: &str) -> Result<(&str, Option<&str>), AddrError> {
    let s = s.trim();
    if s.is_empty() {
        return Err(AddrError::Empty);
    }
    if s.contains(char::is_whitespace) {
        return Err(AddrError::Space);
    }
    if let Some(inner) = s.strip_prefix('[') {
        let (host, rest) = inner.split_once(']').ok_or_else(|| AddrError::Ipv6(s.into()))?;
        host.parse::<Ipv6Addr>().map_err(|_| AddrError::Ipv6(host.into()))?;
        return match rest {
            "" => Ok((host, None)),
            r => match r.strip_prefix(':') {
                Some(p) => Ok((host, Some(p))),
                None => Err(AddrError::Ipv6(s.into())),
            },
        };
    }
    if s.matches(':').count() > 1 {
        s.parse::<Ipv6Addr>().map_err(|_| AddrError::Ipv6(s.into()))?;
        return Ok((s, None));
    }
    match s.split_once(':') {
        Some(("", _)) => Err(AddrError::NoHost),
        Some((host, p)) => Ok((host, Some(p))),
        None => Ok((s, None)),
    }
}

fn port_of(p: &str) -> Result<u16, AddrError> {
    listen_port(p).and_then(|n| match n {
        0 => Err(AddrError::Port(p.into())),
        n => Ok(n),
    })
}

fn listen_port(p: &str) -> Result<u16, AddrError> {
    match p.bytes().all(|b| b.is_ascii_digit()) {
        true => p.parse().map_err(|_| AddrError::Port(p.into())),
        false => Err(AddrError::Port(p.into())),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const LOOPBACK: IpAddr = IpAddr::V4(Ipv4Addr::LOCALHOST);

    fn host(s: &str) -> Result<String, AddrError> {
        HostPort::parse(s, 1234).map(|h| h.to_string())
    }

    fn at(s: &str) -> Result<SocketAddr, AddrError> {
        listen(s, LOOPBACK)
    }

    fn sock(s: &str) -> Result<SocketAddr, AddrError> {
        Ok(s.parse().unwrap())
    }

    #[test]
    fn a_host_takes_the_default_port_and_keeps_its_own() {
        assert_eq!(host("localhost"), Ok("localhost:1234".into()));
        assert_eq!(host(" radarpi:9000 "), Ok("radarpi:9000".into()));
        assert_eq!(host("10.0.0.5:30005"), Ok("10.0.0.5:30005".into()));
        assert_eq!(HostPort::parse("pi.local", 30002).unwrap().host, "pi.local");
    }

    #[test]
    fn an_ipv6_host_is_not_split_on_its_last_group() {
        assert_eq!(host("::1"), Ok("[::1]:1234".into()));
        assert_eq!(HostPort::parse("::1", 1234).unwrap().host, "::1");
        assert_eq!(host("fd00::1"), Ok("[fd00::1]:1234".into()));
        assert_eq!(host("[::1]:8001"), Ok("[::1]:8001".into()));
        assert_eq!(host("[::1]"), Ok("[::1]:1234".into()));
        assert_eq!(host("[::1]8001"), Err(AddrError::Ipv6("[::1]8001".into())));
        assert_eq!(host("[::1"), Err(AddrError::Ipv6("[::1".into())));
        assert_eq!(host("a:b:c"), Err(AddrError::Ipv6("a:b:c".into())));
    }

    #[test]
    fn a_host_with_a_bad_port_or_a_space_is_refused() {
        assert_eq!(host("host:99999"), Err(AddrError::Port("99999".into())));
        assert_eq!(host("host:0"), Err(AddrError::Port("0".into())));
        assert_eq!(host("host:"), Err(AddrError::Port("".into())));
        assert_eq!(host("host:+80"), Err(AddrError::Port("+80".into())));
        assert_eq!(host(":80"), Err(AddrError::NoHost));
        assert_eq!(host("two words"), Err(AddrError::Space));
        assert_eq!(host("host: 80"), Err(AddrError::Space));
        assert_eq!(host("  "), Err(AddrError::Empty));
    }

    #[test]
    fn a_port_is_a_number_from_one_to_65535() {
        assert_eq!(port(" 1883 "), Ok(1883));
        assert_eq!(port("65535"), Ok(65535));
        assert_eq!(port("65536"), Err(AddrError::Port("65536".into())));
        assert_eq!(port("0"), Err(AddrError::Port("0".into())));
        assert_eq!(port("188 3"), Err(AddrError::Space));
        assert_eq!(port("mqtt"), Err(AddrError::Port("mqtt".into())));
        assert_eq!(port(""), Err(AddrError::Empty));
    }

    #[test]
    fn a_listening_address_is_a_port_or_an_ip_and_port() {
        assert_eq!(at("8001"), sock("127.0.0.1:8001"));
        assert_eq!(listen("1234", IpAddr::V4(Ipv4Addr::UNSPECIFIED)), sock("0.0.0.0:1234"));
        assert_eq!(at("0.0.0.0:8010"), sock("0.0.0.0:8010"));
        assert_eq!(at("[::1]:8001"), sock("[::1]:8001"));
        assert_eq!(at("localhost:8001"), sock("127.0.0.1:8001"));
        assert_eq!(at("LOCALHOST:8001"), sock("127.0.0.1:8001"));
        assert_eq!(at("127.0.0.1:0"), sock("127.0.0.1:0"), "a port the kernel picks");
    }

    #[test]
    fn a_listening_address_says_what_is_wrong_with_it() {
        assert_eq!(at("::1"), Err(AddrError::NoPort));
        assert_eq!(at("0.0.0.0"), Err(AddrError::NoPort));
        assert_eq!(at("localhost"), Err(AddrError::NoPort));
        assert_eq!(at("host:99999"), Err(AddrError::NotAnAddress("host".into())));
        assert_eq!(at("0.0.0.0:99999"), Err(AddrError::Port("99999".into())));
        assert_eq!(at("99999"), Err(AddrError::Port("99999".into())));
        assert_eq!(at("pi.local:8001"), Err(AddrError::NotAnAddress("pi.local".into())));
        assert_eq!(at("banana"), Err(AddrError::NotAnAddress("banana".into())));
        assert_eq!(at("0.0.0.0: 8001"), Err(AddrError::Space));
        assert_eq!(at(""), Err(AddrError::Empty));
    }
}
