use common::{Error, Result};
use futures_util::stream::{SplitSink, SplitStream};
use futures_util::{Sink, Stream, StreamExt};
use std::pin::Pin;
use std::task::{Context, Poll, ready};
use tokio::io::{AsyncRead, AsyncWrite, ReadBuf};
use tokio_tungstenite::WebSocketStream;
use tokio_tungstenite::tungstenite::Message;
use tokio_tungstenite::tungstenite::handshake::server::{ErrorResponse, Request, Response};
use tokio_tungstenite::tungstenite::http::HeaderValue;

pub const SUBPROTOCOL: &str = "iqstream";

pub type Read = Box<dyn AsyncRead + Send + Unpin>;
pub type Write = Box<dyn AsyncWrite + Send + Unpin>;

pub fn is_url(s: &str) -> bool {
    let s = s.trim_start().to_ascii_lowercase();
    s.starts_with("ws://") || s.starts_with("wss://")
}

pub struct Reader<S> {
    stream: SplitStream<WebSocketStream<S>>,
    held: Vec<u8>,
    at: usize,
}

pub struct Writer<S> {
    sink: SplitSink<WebSocketStream<S>, Message>,
    sent: bool,
}

pub fn split<S>(ws: WebSocketStream<S>) -> (Reader<S>, Writer<S>)
where
    S: AsyncRead + AsyncWrite + Unpin,
{
    let (sink, stream) = ws.split();
    (Reader { stream, held: Vec::new(), at: 0 }, Writer { sink, sent: false })
}

fn io(e: impl std::fmt::Display) -> std::io::Error {
    std::io::Error::other(e.to_string())
}

impl<S: AsyncRead + AsyncWrite + Unpin> AsyncRead for Reader<S> {
    fn poll_read(
        mut self: Pin<&mut Self>,
        cx: &mut Context<'_>,
        buf: &mut ReadBuf<'_>,
    ) -> Poll<std::io::Result<()>> {
        while self.at >= self.held.len() {
            match ready!(Pin::new(&mut self.stream).poll_next(cx)) {
                None | Some(Ok(Message::Close(_))) => return Poll::Ready(Ok(())),
                Some(Ok(Message::Binary(b))) => {
                    self.held = b.to_vec();
                    self.at = 0;
                }
                Some(Ok(_)) => {}
                Some(Err(e)) => return Poll::Ready(Err(io(e))),
            }
        }
        let n = buf.remaining().min(self.held.len() - self.at);
        let at = self.at;
        buf.put_slice(&self.held[at..at + n]);
        self.at += n;
        Poll::Ready(Ok(()))
    }
}

impl<S: AsyncRead + AsyncWrite + Unpin> AsyncWrite for Writer<S> {
    fn poll_write(
        mut self: Pin<&mut Self>,
        cx: &mut Context<'_>,
        buf: &[u8],
    ) -> Poll<std::io::Result<usize>> {
        if !self.sent {
            ready!(Pin::new(&mut self.sink).poll_ready(cx)).map_err(io)?;
            Pin::new(&mut self.sink).start_send(Message::binary(buf.to_vec())).map_err(io)?;
            self.sent = true;
        }
        ready!(Pin::new(&mut self.sink).poll_flush(cx)).map_err(io)?;
        self.sent = false;
        Poll::Ready(Ok(buf.len()))
    }

    fn poll_flush(mut self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<std::io::Result<()>> {
        Pin::new(&mut self.sink).poll_flush(cx).map_err(io)
    }

    fn poll_shutdown(mut self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<std::io::Result<()>> {
        Pin::new(&mut self.sink).poll_close(cx).map_err(io)
    }
}

#[allow(clippy::result_large_err)]
fn offer_subprotocol(
    req: &Request,
    mut resp: Response,
) -> std::result::Result<Response, ErrorResponse> {
    let asked = req
        .headers()
        .get_all("sec-websocket-protocol")
        .iter()
        .filter_map(|v| v.to_str().ok())
        .flat_map(|v| v.split(','))
        .any(|p| p.trim() == SUBPROTOCOL);
    if asked {
        resp.headers_mut().insert("sec-websocket-protocol", HeaderValue::from_static(SUBPROTOCOL));
    }
    Ok(resp)
}

pub async fn accept<S>(sock: S) -> Result<(Read, Write)>
where
    S: AsyncRead + AsyncWrite + Unpin + Send + 'static,
{
    let ws = tokio_tungstenite::accept_hdr_async(sock, offer_subprotocol)
        .await
        .map_err(|e| Error::other(format!("websocket: {e}")))?;
    let (r, w) = split(ws);
    Ok((Box::new(r), Box::new(w)))
}

fn tls() -> std::result::Result<std::sync::Arc<rustls::ClientConfig>, rustls::Error> {
    let roots = rustls::RootCertStore { roots: webpki_roots::TLS_SERVER_ROOTS.to_vec() };
    let config = rustls::ClientConfig::builder_with_provider(std::sync::Arc::new(
        rustls::crypto::ring::default_provider(),
    ))
    .with_safe_default_protocol_versions()?
    .with_root_certificates(roots)
    .with_no_client_auth();
    Ok(std::sync::Arc::new(config))
}

pub async fn connect(url: &str) -> Result<(Read, Write)> {
    use tokio_tungstenite::tungstenite::client::IntoClientRequest;
    let mut req = url.into_client_request().map_err(|e| Error::other(format!("{url}: {e}")))?;
    req.headers_mut().insert("sec-websocket-protocol", HeaderValue::from_static(SUBPROTOCOL));
    let tls = tls().map_err(|e| Error::other(format!("tls: {e}")))?;
    let (ws, _) = tokio_tungstenite::connect_async_tls_with_config(
        req,
        None,
        false,
        Some(tokio_tungstenite::Connector::Rustls(tls)),
    )
    .await
    .map_err(|e| Error::other(format!("{url}: {e}")))?;
    let (r, w) = split(ws);
    Ok((Box::new(r), Box::new(w)))
}
