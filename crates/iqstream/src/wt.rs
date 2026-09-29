use crate::config::Offered;
use common::time::Duration;
use common::{Error, Result};
use std::net::SocketAddr;
use std::sync::{Arc, Mutex};
use wtransport::{Endpoint, Identity, ServerConfig};

pub const VALID_DAYS: u32 = 13;
pub const ROTATE: Duration = Duration::from_secs(6 * 24 * 3600);

fn certificate() -> Result<(Identity, [u8; 32])> {
    let identity = Identity::self_signed_builder()
        .subject_alt_names(["localhost", "waveshark"])
        .from_now_utc()
        .validity_days(VALID_DAYS)
        .build()
        .map_err(|e| Error::other(format!("webtransport certificate: {e}")))?;
    let hash = *identity.certificate_chain().as_slice()[0].hash().as_ref();
    Ok((identity, hash))
}

fn config(socket: Option<std::net::UdpSocket>, identity: Identity) -> ServerConfig {
    let bound = match socket {
        Some(s) => ServerConfig::builder().with_bind_socket(s),
        None => ServerConfig::builder().with_bind_default(0),
    };
    bound.with_identity(identity).keep_alive_interval(Some(Duration::from_secs(5))).build()
}

pub struct Listener {
    endpoint: Endpoint<wtransport::endpoint::endpoint_side::Server>,
    offered: Arc<Mutex<Option<Offered>>>,
    next: Mutex<(Identity, [u8; 32])>,
    port: u16,
}

impl Listener {
    pub fn bind(at: SocketAddr, offered: Arc<Mutex<Option<Offered>>>) -> Result<Self> {
        let socket =
            std::net::UdpSocket::bind(at).map_err(|e| Error::other(format!("{at}: {e}")))?;
        let port = socket.local_addr().map_err(|e| Error::other(e.to_string()))?.port();
        let (identity, hash) = certificate()?;
        let (next, coming) = certificate()?;
        let endpoint = Endpoint::server(config(Some(socket), identity))
            .map_err(|e| Error::other(format!("webtransport on {at}: {e}")))?;
        if let Ok(mut held) = offered.lock() {
            *held = Some(Offered { port, hashes: vec![hash, coming] });
        }
        Ok(Listener { endpoint, offered, next: Mutex::new((next, coming)), port })
    }

    pub fn port(&self) -> u16 {
        self.port
    }

    pub fn rotate(&self) -> Result<()> {
        let fresh = certificate()?;
        let mut next = self.next.lock().map_err(|_| Error::other("webtransport identity"))?;
        let (serving, hash) = std::mem::replace(&mut *next, fresh);
        self.endpoint
            .reload_config(config(None, serving), false)
            .map_err(|e| Error::other(format!("webtransport certificate: {e}")))?;
        if let Ok(mut held) = self.offered.lock() {
            *held = Some(Offered { port: self.port, hashes: vec![hash, next.1] });
        }
        Ok(())
    }

    pub async fn accept(
        &self,
    ) -> Result<(wtransport::SendStream, wtransport::RecvStream, SocketAddr)> {
        let session =
            self.endpoint.accept().await.await.map_err(|e| Error::other(e.to_string()))?;
        let peer = session.remote_address();
        let connection = session.accept().await.map_err(|e| Error::other(e.to_string()))?;
        let (send, recv) = connection.accept_bi().await.map_err(|e| Error::other(e.to_string()))?;
        tokio::spawn(async move {
            connection.closed().await;
        });
        Ok((send, recv, peer))
    }
}

pub async fn connect(url: &str) -> Result<(crate::ws::Read, crate::ws::Write)> {
    let hashes = crate::url::certificates(url).into_iter().map(Into::into);
    let config = wtransport::ClientConfig::builder()
        .with_bind_default()
        .with_server_certificate_hashes(hashes)
        .keep_alive_interval(Some(Duration::from_secs(5)))
        .build();
    let endpoint = Endpoint::client(config).map_err(|e| Error::other(format!("{url}: {e}")))?;
    let connection =
        endpoint.connect(url).await.map_err(|e| Error::other(format!("{url}: {e}")))?;
    let (send, recv) = connection
        .open_bi()
        .await
        .map_err(|e| Error::other(format!("{url}: {e}")))?
        .await
        .map_err(|e| Error::other(format!("{url}: {e}")))?;
    Ok((Box::new(Held { recv, _endpoint: endpoint, _connection: connection }), Box::new(send)))
}

struct Held {
    recv: wtransport::RecvStream,
    _endpoint: Endpoint<wtransport::endpoint::endpoint_side::Client>,
    _connection: wtransport::Connection,
}

impl tokio::io::AsyncRead for Held {
    fn poll_read(
        mut self: std::pin::Pin<&mut Self>,
        cx: &mut std::task::Context<'_>,
        buf: &mut tokio::io::ReadBuf<'_>,
    ) -> std::task::Poll<std::io::Result<()>> {
        std::pin::Pin::new(&mut self.recv).poll_read(cx, buf)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn the_next_certificate_is_offered_before_it_is_served() {
        let offered = Arc::new(Mutex::new(None));
        let listener =
            Arc::new(Listener::bind("127.0.0.1:0".parse().unwrap(), offered.clone()).unwrap());
        let before = offered.lock().unwrap().clone().unwrap();
        assert_eq!(before.hashes.len(), 2);
        listener.rotate().unwrap();
        let after = offered.lock().unwrap().clone().unwrap();
        assert_eq!(after.port, before.port);
        assert_eq!(after.hashes[0], before.hashes[1], "what was offered next is served now");
        assert!(!before.hashes.contains(&after.hashes[1]));

        let accepting = listener.clone();
        tokio::spawn(async move { while accepting.accept().await.is_ok() {} });
        let at = format!("127.0.0.1:{}", after.port);
        let told_before = crate::url::webtransport_url(&at, &before.hex());
        assert!(
            connect(&told_before).await.is_ok(),
            "a listing read before the change still works"
        );
        let gone = crate::url::webtransport_url(&at, &before.hex()[..1]);
        assert!(connect(&gone).await.is_err(), "the certificate served before is retired");
    }
}
