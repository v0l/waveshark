mod door;

pub use door::{Doors, MOST_SESSIONS, Speaks};
pub use iqstream::{Offered, Server, ServerConfig, Stream, StreamConfig};

use common::Result;
use std::net::SocketAddr;
use std::sync::Arc;

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum WebTransport {
    #[default]
    Beside,
    On(u16),
    Off,
}

impl WebTransport {
    pub fn port(self, tcp: u16) -> Option<u16> {
        match self {
            WebTransport::Beside if tcp == 0 => Some(0),
            WebTransport::Beside => tcp.checked_add(1),
            WebTransport::On(port) => Some(port),
            WebTransport::Off => None,
        }
    }
}

impl std::str::FromStr for WebTransport {
    type Err = String;

    fn from_str(s: &str) -> std::result::Result<Self, Self::Err> {
        match s.trim().to_ascii_lowercase().as_str() {
            "off" | "no" | "none" => Ok(WebTransport::Off),
            "" | "on" | "auto" => Ok(WebTransport::Beside),
            port => port.parse().map(WebTransport::On).map_err(|_| format!("{s:?} is not a port")),
        }
    }
}

#[derive(Clone, Debug)]
pub struct Options {
    pub name: String,
    pub streams: Vec<StreamConfig>,
    pub webtransport: WebTransport,
    pub webrtc: bool,
    pub rtl_tcp_and_spyserver: bool,
}

impl Default for Options {
    fn default() -> Self {
        Options {
            name: "waveshark".into(),
            streams: Vec::new(),
            webtransport: WebTransport::Beside,
            webrtc: true,
            rtl_tcp_and_spyserver: true,
        }
    }
}

pub fn start(addr: SocketAddr, options: Options) -> Result<Arc<Server>> {
    let Options { name, streams, webtransport, webrtc, rtl_tcp_and_spyserver } = options;
    let config = ServerConfig {
        name,
        streams,
        door: rtl_tcp_and_spyserver.then(Doors::shared),
        webtransport: webtransport.port(addr.port()),
        webrtc,
    };
    Server::start(addr, config)
}

pub fn answer_webrtc<S: AsRef<str>>(
    keys: nostr_directory::Keys,
    relays: &[S],
    serving: impl Fn() -> Option<Arc<Server>> + Send + Sync + 'static,
) -> nostr_directory::signal::Answerer {
    let answer: nostr_directory::signal::Answer = Arc::new(move |offer: &str| {
        let server = serving().ok_or("nothing is being served")?;
        server.answer(offer).map_err(|e| e.to_string())
    });
    nostr_directory::signal::Answerer::start(keys, relays, answer)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn webtransport_sits_on_the_next_port_unless_told_otherwise() {
        assert_eq!(WebTransport::Beside.port(5555), Some(5556));
        assert_eq!(WebTransport::Beside.port(0), Some(0), "a free port asks for another free one");
        assert_eq!(WebTransport::Beside.port(u16::MAX), None);
        assert_eq!(WebTransport::On(4433).port(5555), Some(4433));
        assert_eq!(WebTransport::Off.port(5555), None);
        assert_eq!("off".parse(), Ok(WebTransport::Off));
        assert_eq!("4433".parse(), Ok(WebTransport::On(4433)));
        assert_eq!("".parse(), Ok(WebTransport::Beside));
        assert!("udp".parse::<WebTransport>().is_err());
    }

    #[test]
    fn every_protocol_is_on_by_default() {
        let server = start("127.0.0.1:0".parse().unwrap(), Options::default()).unwrap();
        assert!(server.webrtc());
        assert!(server.webtransport().is_some());
        let bare = Options {
            webtransport: WebTransport::Off,
            webrtc: false,
            rtl_tcp_and_spyserver: false,
            ..Options::default()
        };
        let server = start("127.0.0.1:0".parse().unwrap(), bare).unwrap();
        assert!(!server.webrtc());
        assert!(server.webtransport().is_none());
    }
}
