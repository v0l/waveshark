use common::time::{Duration, Instant};
use rustls::pki_types::ServerName;
use rustls::{ClientConfig, ClientConnection, RootCertStore, StreamOwned};
use serde_json::Value;
use std::io::{self, Read, Write};
use std::net::{TcpStream, ToSocketAddrs};
use std::sync::{Arc, OnceLock};
use tungstenite::client::IntoClientRequest;
use tungstenite::http::HeaderValue;
use tungstenite::http::header::USER_AGENT;
use tungstenite::{Message, WebSocket};

pub enum Stream {
    Plain(TcpStream),
    Tls(Box<StreamOwned<ClientConnection, TcpStream>>),
}

impl Stream {
    fn tcp(&self) -> &TcpStream {
        match self {
            Stream::Plain(s) => s,
            Stream::Tls(s) => s.get_ref(),
        }
    }
}

impl Read for Stream {
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        match self {
            Stream::Plain(s) => s.read(buf),
            Stream::Tls(s) => s.read(buf),
        }
    }
}

impl Write for Stream {
    fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        match self {
            Stream::Plain(s) => s.write(buf),
            Stream::Tls(s) => s.write(buf),
        }
    }

    fn flush(&mut self) -> io::Result<()> {
        match self {
            Stream::Plain(s) => s.flush(),
            Stream::Tls(s) => s.flush(),
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Scheme {
    Ws,
    Wss,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Url {
    scheme: Scheme,
    host: String,
    port: u16,
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

pub struct Socket {
    ws: WebSocket<Stream>,
}

impl Socket {
    pub fn open(url: &str, wait: Duration) -> Result<Socket, String> {
        let at = Url::parse(url)?;
        let tcp = dial(&at.host, at.port, wait)?;
        tcp.set_read_timeout(Some(wait)).map_err(|e| e.to_string())?;
        tcp.set_write_timeout(Some(wait)).map_err(|e| e.to_string())?;
        let _ = tcp.set_nodelay(true);
        let stream = match at.scheme {
            Scheme::Ws => Stream::Plain(tcp),
            Scheme::Wss => {
                let name = ServerName::try_from(at.host.clone()).map_err(|e| e.to_string())?;
                let tls = ClientConnection::new(tls(), name).map_err(|e| e.to_string())?;
                Stream::Tls(Box::new(StreamOwned::new(tls, tcp)))
            }
        };
        let mut req = url.into_client_request().map_err(|e| e.to_string())?;
        req.headers_mut().insert(USER_AGENT, HeaderValue::from_static(httpc::USER_AGENT));
        let (ws, _) = tungstenite::client(req, stream).map_err(|e| match e {
            tungstenite::HandshakeError::Interrupted(_) => "timed out in the handshake".to_string(),
            tungstenite::HandshakeError::Failure(e) => e.to_string(),
        })?;
        Ok(Socket { ws })
    }

    pub fn send(&mut self, v: &Value) -> Result<(), String> {
        self.ws.send(Message::text(v.to_string())).map_err(|e| e.to_string())
    }

    pub fn recv(&mut self, until: Instant) -> Result<Option<Value>, String> {
        loop {
            let left = until.saturating_duration_since(Instant::now());
            if left.is_zero() {
                return Ok(None);
            }
            self.ws.get_ref().tcp().set_read_timeout(Some(left)).map_err(|e| e.to_string())?;
            match self.ws.read() {
                Ok(Message::Text(t)) => {
                    if let Ok(v) = serde_json::from_str(&t) {
                        return Ok(Some(v));
                    }
                }
                Ok(Message::Close(_)) => return Err("the relay closed the connection".into()),
                Ok(_) => {}
                Err(tungstenite::Error::Io(e))
                    if matches!(e.kind(), io::ErrorKind::WouldBlock | io::ErrorKind::TimedOut) =>
                {
                    return Ok(None);
                }
                Err(e) => return Err(e.to_string()),
            }
        }
    }

    pub fn close(mut self) {
        let _ = self.ws.close(None);
        let _ = self.ws.flush();
    }
}

fn dial(host: &str, port: u16, wait: Duration) -> Result<TcpStream, String> {
    let addrs = (host, port).to_socket_addrs().map_err(|e| format!("{host}: {e}"))?;
    let mut last = format!("{host} has no address");
    for addr in addrs {
        match TcpStream::connect_timeout(&addr, wait) {
            Ok(s) => return Ok(s),
            Err(e) => last = format!("{addr}: {e}"),
        }
    }
    Err(last)
}

fn tls() -> Arc<ClientConfig> {
    static CONFIG: OnceLock<Arc<ClientConfig>> = OnceLock::new();
    CONFIG
        .get_or_init(|| {
            let roots = RootCertStore { roots: webpki_roots::TLS_SERVER_ROOTS.to_vec() };
            let provider = Arc::new(rustls::crypto::ring::default_provider());
            Arc::new(
                ClientConfig::builder_with_provider(provider)
                    .with_safe_default_protocol_versions()
                    .expect("ring offers the default protocol versions")
                    .with_root_certificates(roots)
                    .with_no_client_auth(),
            )
        })
        .clone()
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
