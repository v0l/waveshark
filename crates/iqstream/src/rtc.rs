mod crypto;

use common::{Error, Result};
use std::collections::VecDeque;
use std::net::{IpAddr, SocketAddr};
use std::pin::Pin;
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use std::sync::{Arc, Mutex};
use std::task::{Context, Poll, Waker};
use std::time::{Duration, Instant};
use str0m::change::SdpOffer;
use str0m::channel::ChannelId;
use str0m::net::{Protocol, Receive};
use str0m::{Candidate, Event, IceConnectionState, Input, Output, Rtc};
use tokio::io::{AsyncRead, AsyncWrite, ReadBuf};
use tokio::net::UdpSocket;
use tokio::sync::{Notify, mpsc, oneshot};

pub const LABEL: &str = "iqstream";
const PIECE: usize = 16 * 1024;
const OUTBOX: usize = 512 * 1024;
const IDLE: Duration = Duration::from_millis(100);
const UNOPENED: Duration = Duration::from_secs(30);

pub fn is_rtc(datagram: &[u8]) -> bool {
    matches!(datagram.first(), Some(0..=3 | 20..=63))
}

pub type Serve = Arc<dyn Fn(PipeRead, PipeWrite, SocketAddr) + Send + Sync>;

struct Asked {
    offer: String,
    answer: oneshot::Sender<Result<String>>,
}

#[derive(Clone)]
pub struct Hub {
    asked: mpsc::UnboundedSender<Asked>,
    heard: mpsc::Sender<(Vec<u8>, SocketAddr)>,
}

impl Hub {
    pub fn start(socket: Arc<UdpSocket>, candidates: Candidates, serve: Serve) -> Self {
        static PROVIDER: std::sync::Once = std::sync::Once::new();
        PROVIDER.call_once(|| crypto::provider().install_process_default());
        let (asked, asks) = mpsc::unbounded_channel();
        let (heard, hears) = mpsc::channel(256);
        tokio::spawn(run(socket, candidates, serve, asks, hears));
        Hub { asked, heard }
    }

    pub fn heard(&self, datagram: &[u8], from: SocketAddr) {
        let _ = self.heard.try_send((datagram.to_vec(), from));
    }

    pub async fn answer(&self, offer: String) -> Result<String> {
        let (answer, answered) = oneshot::channel();
        self.asked.send(Asked { offer, answer }).map_err(|_| Error::other("webrtc has stopped"))?;
        answered.await.map_err(|_| Error::other("webrtc has stopped"))?
    }
}

#[derive(Clone, Default)]
pub struct Candidates {
    pub local: Option<SocketAddr>,
    pub public: Arc<Mutex<Option<SocketAddr>>>,
}

impl Candidates {
    pub fn of(bound: SocketAddr) -> Self {
        let ip = match bound.ip().is_unspecified() {
            false => Some(bound.ip()),
            true => toward_the_world(),
        };
        Candidates {
            local: ip.map(|ip| SocketAddr::new(ip, bound.port())),
            public: Default::default(),
        }
    }

    fn all(&self) -> Vec<SocketAddr> {
        let public = self.public.lock().ok().and_then(|p| *p);
        self.local.into_iter().chain(public.filter(|p| Some(*p) != self.local)).collect()
    }
}

fn toward_the_world() -> Option<IpAddr> {
    let probe = std::net::UdpSocket::bind("0.0.0.0:0").ok()?;
    probe.connect("192.0.2.1:9").ok()?;
    Some(probe.local_addr().ok()?.ip())
}

struct Outbox {
    queue: Mutex<VecDeque<Vec<u8>>>,
    bytes: AtomicUsize,
    waiting: Mutex<Option<Waker>>,
    closed: AtomicBool,
    hub: Arc<Notify>,
}

impl Outbox {
    fn wake(&self) {
        if let Some(w) = self.waiting.lock().ok().and_then(|mut w| w.take()) {
            w.wake();
        }
    }
}

pub struct PipeRead {
    from: mpsc::UnboundedReceiver<Vec<u8>>,
    held: Vec<u8>,
    at: usize,
}

impl AsyncRead for PipeRead {
    fn poll_read(
        mut self: Pin<&mut Self>,
        cx: &mut Context<'_>,
        buf: &mut ReadBuf<'_>,
    ) -> Poll<std::io::Result<()>> {
        while self.at >= self.held.len() {
            match self.from.poll_recv(cx) {
                Poll::Ready(Some(b)) => {
                    self.held = b;
                    self.at = 0;
                }
                Poll::Ready(None) => return Poll::Ready(Ok(())),
                Poll::Pending => return Poll::Pending,
            }
        }
        let n = buf.remaining().min(self.held.len() - self.at);
        let at = self.at;
        buf.put_slice(&self.held[at..at + n]);
        self.at += n;
        Poll::Ready(Ok(()))
    }
}

pub struct PipeWrite {
    outbox: Arc<Outbox>,
}

impl AsyncWrite for PipeWrite {
    fn poll_write(
        self: Pin<&mut Self>,
        cx: &mut Context<'_>,
        buf: &[u8],
    ) -> Poll<std::io::Result<usize>> {
        let o = &self.outbox;
        if o.closed.load(Ordering::Acquire) {
            return Poll::Ready(Err(std::io::ErrorKind::BrokenPipe.into()));
        }
        if o.bytes.load(Ordering::Acquire) >= OUTBOX {
            if let Ok(mut w) = o.waiting.lock() {
                *w = Some(cx.waker().clone());
            }
            if o.bytes.load(Ordering::Acquire) >= OUTBOX {
                return Poll::Pending;
            }
        }
        if let Ok(mut q) = o.queue.lock() {
            q.push_back(buf.to_vec());
        }
        o.bytes.fetch_add(buf.len(), Ordering::AcqRel);
        o.hub.notify_one();
        Poll::Ready(Ok(buf.len()))
    }

    fn poll_flush(self: Pin<&mut Self>, _: &mut Context<'_>) -> Poll<std::io::Result<()>> {
        Poll::Ready(Ok(()))
    }

    fn poll_shutdown(self: Pin<&mut Self>, _: &mut Context<'_>) -> Poll<std::io::Result<()>> {
        self.outbox.closed.store(true, Ordering::Release);
        self.outbox.hub.notify_one();
        Poll::Ready(Ok(()))
    }
}

impl Drop for PipeWrite {
    fn drop(&mut self) {
        self.outbox.closed.store(true, Ordering::Release);
        self.outbox.hub.notify_one();
    }
}

struct Peer {
    rtc: Rtc,
    born: Instant,
    from: Option<SocketAddr>,
    channel: Option<ChannelId>,
    inbound: Option<mpsc::UnboundedSender<Vec<u8>>>,
    outbox: Arc<Outbox>,
}

impl Peer {
    fn drive(&mut self, socket: &UdpSocket, serve: &Serve) -> Instant {
        loop {
            if !self.rtc.is_alive() {
                return Instant::now();
            }
            match self.rtc.poll_output() {
                Ok(Output::Timeout(t)) => return t,
                Ok(Output::Transmit(t)) => {
                    self.from = Some(t.destination);
                    let _ = socket.try_send_to(&t.contents, t.destination);
                }
                Ok(Output::Event(e)) => self.event(e, serve),
                Err(e) => {
                    tracing::debug!("iqstream: webrtc: {e}");
                    self.rtc.disconnect();
                }
            }
        }
    }

    fn event(&mut self, e: Event, serve: &Serve) {
        match e {
            Event::ChannelOpen(id, label) if label == LABEL && self.channel.is_none() => {
                self.channel = Some(id);
                let (inbound, from) = mpsc::unbounded_channel();
                self.inbound = Some(inbound);
                let peer = self.from.unwrap_or(SocketAddr::from(([0, 0, 0, 0], 0)));
                let write = PipeWrite { outbox: self.outbox.clone() };
                serve(PipeRead { from, held: Vec::new(), at: 0 }, write, peer);
            }
            Event::ChannelData(d) if Some(d.id) == self.channel => {
                if let Some(i) = &self.inbound {
                    let _ = i.send(d.data);
                }
            }
            Event::ChannelClose(id) if Some(id) == self.channel => self.rtc.disconnect(),
            Event::IceConnectionStateChange(IceConnectionState::Disconnected) => {
                self.rtc.disconnect()
            }
            _ => {}
        }
    }

    fn flush(&mut self) {
        let Some(id) = self.channel else { return };
        let Some(mut channel) = self.rtc.channel(id) else { return };
        let Ok(mut queue) = self.outbox.queue.lock() else { return };
        while let Some(front) = queue.front_mut() {
            let n = front.len().min(PIECE);
            match channel.write(true, &front[..n]) {
                Ok(true) => {
                    front.drain(..n);
                    if front.is_empty() {
                        queue.pop_front();
                    }
                    self.outbox.bytes.fetch_sub(n, Ordering::AcqRel);
                }
                Ok(false) => break,
                Err(e) => {
                    tracing::debug!("iqstream: webrtc: {e}");
                    break;
                }
            }
        }
        drop(queue);
        if self.outbox.bytes.load(Ordering::Acquire) < OUTBOX {
            self.outbox.wake();
        }
        if self.outbox.closed.load(Ordering::Acquire)
            && self.outbox.bytes.load(Ordering::Acquire) == 0
        {
            self.rtc.disconnect();
        }
    }
}

impl Drop for Peer {
    fn drop(&mut self) {
        self.outbox.closed.store(true, Ordering::Release);
        self.outbox.wake();
    }
}

fn accept(offer: &str, candidates: &Candidates, hub: &Arc<Notify>) -> Result<(Peer, String)> {
    let offer =
        SdpOffer::from_sdp_string(offer).map_err(|e| Error::other(format!("offer: {e}")))?;
    let mut rtc = Rtc::builder().set_ice_lite(true).build(Instant::now());
    let all = candidates.all();
    if all.is_empty() {
        return Err(Error::other("webrtc: no address to offer"));
    }
    for at in all {
        let c = Candidate::host(at, "udp").map_err(|e| Error::other(format!("candidate: {e}")))?;
        rtc.add_local_candidate(c);
    }
    let answer =
        rtc.sdp_api().accept_offer(offer).map_err(|e| Error::other(format!("offer: {e}")))?;
    let outbox = Arc::new(Outbox {
        queue: Mutex::new(VecDeque::new()),
        bytes: AtomicUsize::new(0),
        waiting: Mutex::new(None),
        closed: AtomicBool::new(false),
        hub: hub.clone(),
    });
    let peer = Peer { rtc, born: Instant::now(), from: None, channel: None, inbound: None, outbox };
    Ok((peer, answer.to_sdp_string()))
}

async fn run(
    socket: Arc<UdpSocket>,
    candidates: Candidates,
    serve: Serve,
    mut asks: mpsc::UnboundedReceiver<Asked>,
    mut hears: mpsc::Receiver<(Vec<u8>, SocketAddr)>,
) {
    let wake = Arc::new(Notify::new());
    let mut peers: Vec<Peer> = Vec::new();
    loop {
        let mut next = Instant::now() + IDLE;
        for p in peers.iter_mut() {
            p.flush();
            next = next.min(p.drive(&socket, &serve));
        }
        peers.retain(|p| p.rtc.is_alive() && (p.channel.is_some() || p.born.elapsed() < UNOPENED));
        let until = tokio::time::Instant::from_std(next.max(Instant::now()));
        tokio::select! {
            asked = asks.recv() => {
                let Some(Asked { offer, answer }) = asked else { return };
                match accept(&offer, &candidates, &wake) {
                    Ok((peer, sdp)) => {
                        peers.push(peer);
                        let _ = answer.send(Ok(sdp));
                    }
                    Err(e) => {
                        let _ = answer.send(Err(e));
                    }
                }
            }
            heard = hears.recv() => {
                let Some((datagram, from)) = heard else { return };
                let Ok(contents) = datagram.as_slice().try_into() else { continue };
                let destination = candidates.local.unwrap_or(from);
                let input = Input::Receive(
                    Instant::now(),
                    Receive { proto: Protocol::Udp, source: from, destination, contents },
                );
                if let Some(p) = peers.iter_mut().find(|p| p.rtc.accepts(&input))
                    && let Err(e) = p.rtc.handle_input(input)
                {
                    tracing::debug!("iqstream: webrtc: {e}");
                }
            }
            _ = wake.notified() => {}
            _ = tokio::time::sleep_until(until) => {}
        }
        let now = Instant::now();
        for p in peers.iter_mut() {
            let _ = p.rtc.handle_input(Input::Timeout(now));
        }
    }
}
