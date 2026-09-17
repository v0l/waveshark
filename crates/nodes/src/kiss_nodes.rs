//! A KISS TNC on a socket, so packet software can use the radio.
//!
//! [`KissTncNode`] hangs off the packet bus and writes every AX.25 frame the
//! receiver heard to whoever is connected. What those clients send back goes
//! out through the packet channel's own transmit source,
//! [`crate::aprs_nodes::AprsTxNode`], which is the one thing a channel keys
//! with. Neither knows anything about KISS itself, which is `decode::kiss`,
//! or about the tones, which are `dsp::afsk`.
//!
//! The server outlives a rebuild. The graph is redrawn on every retune, and a
//! listener that went with it would drop every client each time the dial
//! moved, so servers are held in a table by the address they were asked for
//! and looked up rather than created. The transmit chain runs on its own
//! thread and is built from its own settings, which is the other reason: the
//! two halves find the same server by naming the same address.
//!
//! Nothing is radiated by a client on its own. The transmit node exists only
//! while an operator has a channel in a mode that transmits, and it is fed
//! samples only while that channel is keyed, so a frame arriving from a
//! client waits in the queue exactly as a beacon would.

use common::{PacketBody, Result};
use decode::{ax25, kiss};
use pipeline::node::{Node, NodeCtx, PortSpec, Simple};
use pipeline::port::{Payload, PortKind, StreamSpec};
use pipeline::registry::{Category, Settings, SettingsExt, StageDesc};
use std::collections::VecDeque;
use std::io::{Read, Write};
use std::net::{SocketAddr, TcpListener, TcpStream};
use std::sync::atomic::{AtomicBool, AtomicU64, AtomicUsize, Ordering};
use std::sync::{Arc, Mutex};

/// Where a TNC listens when nothing has said otherwise. 8001 is what Dire
/// Wolf serves KISS over TCP on, so a client configured for one finds this.
pub const DEFAULT_PORT: u16 = 8001;

/// Frames held for transmission before the oldest is dropped.
///
/// A 100 byte frame at 1200 baud is two thirds of a second on the air, so
/// this is about twenty seconds of queue: enough that a client emptying its
/// buffer into an unkeyed radio loses nothing, and short enough that what
/// finally goes out is not minutes stale.
const QUEUE_DEPTH: usize = 32;

/// How long a write to a client may block the graph for.
///
/// A blocked socket must not stall the packet bus: a client that cannot keep
/// up is disconnected instead.
const WRITE_TIMEOUT: std::time::Duration = std::time::Duration::from_millis(100);

/// What both nodes share: the listener, the clients, and the queue between
/// them.
pub struct Tnc {
    /// The address asked for, which is the key this is found under.
    asked: SocketAddr,
    /// The address it actually listens on, which differs when a port of zero
    /// was asked for.
    bound: Mutex<Option<SocketAddr>>,
    clients: Mutex<Vec<Client>>,
    /// AX.25 frames a client has sent, oldest first.
    pending: Mutex<VecDeque<Vec<u8>>>,
    params: Mutex<kiss::Params>,
    connected: AtomicUsize,
    to_clients: AtomicU64,
    from_clients: AtomicU64,
    /// Frames dropped because the queue was full, which is a client talking
    /// to a radio that is not keyed.
    dropped: AtomicU64,
    error: Mutex<Option<String>>,
    stop: AtomicBool,
    next_id: AtomicU64,
}

struct Client {
    id: u64,
    sock: TcpStream,
}

impl Tnc {
    /// The address it listens on, once the listener has bound.
    pub fn bound(&self) -> Option<SocketAddr> {
        *self.bound.lock().unwrap()
    }

    pub fn address(&self) -> SocketAddr {
        self.asked
    }

    pub fn connected(&self) -> usize {
        self.connected.load(Ordering::Relaxed)
    }

    pub fn sent(&self) -> u64 {
        self.to_clients.load(Ordering::Relaxed)
    }

    pub fn received(&self) -> u64 {
        self.from_clients.load(Ordering::Relaxed)
    }

    pub fn dropped(&self) -> u64 {
        self.dropped.load(Ordering::Relaxed)
    }

    /// Why it is not listening, when it is not.
    pub fn error(&self) -> Option<String> {
        self.error.lock().unwrap().clone()
    }

    pub fn params(&self) -> kiss::Params {
        *self.params.lock().unwrap()
    }

    /// How many frames are waiting to go out.
    pub fn queued(&self) -> usize {
        self.pending.lock().unwrap().len()
    }

    /// The next frame to transmit, if a client has sent one.
    pub fn next_frame(&self) -> Option<Vec<u8>> {
        self.pending.lock().unwrap().pop_front()
    }

    /// Write an AX.25 frame to every client, dropping those that will not
    /// take it.
    pub fn broadcast(&self, frame: &[u8]) {
        let wire = kiss::data(0, frame);
        let mut clients = self.clients.lock().unwrap();
        clients.retain_mut(|c| c.sock.write_all(&wire).and_then(|()| c.sock.flush()).is_ok());
        self.connected.store(clients.len(), Ordering::Relaxed);
        if !clients.is_empty() {
            self.to_clients.fetch_add(1, Ordering::Relaxed);
        }
    }

    fn queue(&self, frame: Vec<u8>) {
        let mut q = self.pending.lock().unwrap();
        // The oldest goes rather than the newest: a client that has been
        // talking to an unkeyed radio for a minute wants what it said last.
        if q.len() >= QUEUE_DEPTH {
            q.pop_front();
            self.dropped.fetch_add(1, Ordering::Relaxed);
        }
        q.push_back(frame);
        self.from_clients.fetch_add(1, Ordering::Relaxed);
    }

    fn remove(&self, id: u64) {
        let mut clients = self.clients.lock().unwrap();
        clients.retain(|c| c.id != id);
        self.connected.store(clients.len(), Ordering::Relaxed);
    }
}

/// Every server started in this process, by the address it was asked for.
///
/// Held for the life of the program rather than dropped with the graph: see
/// the module note. A second node naming the same address gets the running
/// server, not a second listener on a port already in use.
static SERVERS: Mutex<Vec<(SocketAddr, Arc<Tnc>)>> = Mutex::new(Vec::new());

/// The server already running on this address, if there is one.
///
/// What the transmit half uses: a chain keyed on a packet channel must not
/// open a listening socket nobody asked for, so it takes the server the
/// receive half started or transmits silence.
pub fn running(addr: SocketAddr) -> Option<Arc<Tnc>> {
    SERVERS.lock().unwrap().iter().find(|(a, _)| *a == addr).map(|(_, t)| t.clone())
}

/// The server on this address, started if it is not running yet.
///
/// A port of zero is never shared: it asks the kernel for a free port, so two
/// of them are two servers however the request was written.
pub fn server(addr: SocketAddr) -> Arc<Tnc> {
    let mut servers = SERVERS.lock().unwrap();
    if addr.port() != 0
        && let Some((_, t)) = servers.iter().find(|(a, _)| *a == addr)
    {
        return t.clone();
    }
    let tnc = Arc::new(Tnc {
        asked: addr,
        bound: Mutex::new(None),
        clients: Mutex::new(Vec::new()),
        pending: Mutex::new(VecDeque::new()),
        params: Mutex::new(kiss::Params::default()),
        connected: AtomicUsize::new(0),
        to_clients: AtomicU64::new(0),
        from_clients: AtomicU64::new(0),
        dropped: AtomicU64::new(0),
        error: Mutex::new(None),
        stop: AtomicBool::new(false),
        next_id: AtomicU64::new(0),
    });
    let listener = TcpListener::bind(addr);
    match listener {
        Ok(l) => {
            *tnc.bound.lock().unwrap() = l.local_addr().ok();
            let t = tnc.clone();
            let _ = std::thread::Builder::new()
                .name(format!("kiss-{addr}"))
                .spawn(move || accept_loop(l, t));
        }
        // Said once and carried on from, the way a port already in use is
        // everywhere else here: it is not a reason to refuse to be a
        // receiver.
        Err(e) => *tnc.error.lock().unwrap() = Some(e.to_string()),
    }
    servers.push((addr, tnc.clone()));
    tnc
}

fn accept_loop(listener: TcpListener, tnc: Arc<Tnc>) {
    for sock in listener.incoming() {
        if tnc.stop.load(Ordering::Relaxed) {
            return;
        }
        let Ok(sock) = sock else { continue };
        let _ = sock.set_write_timeout(Some(WRITE_TIMEOUT));
        // Nagle would hold a frame back waiting for the next one, which on a
        // packet channel is minutes away.
        let _ = sock.set_nodelay(true);
        let id = tnc.next_id.fetch_add(1, Ordering::Relaxed);
        let Ok(reader) = sock.try_clone() else { continue };
        {
            let mut clients = tnc.clients.lock().unwrap();
            clients.push(Client { id, sock });
            tnc.connected.store(clients.len(), Ordering::Relaxed);
        }
        let t = tnc.clone();
        let _ = std::thread::Builder::new()
            .name(format!("kiss-client-{id}"))
            .spawn(move || read_loop(reader, id, t));
    }
}

fn read_loop(mut sock: TcpStream, id: u64, tnc: Arc<Tnc>) {
    let mut decoder = kiss::Decoder::new();
    let mut frames = Vec::new();
    let mut chunk = [0u8; 4096];
    loop {
        let n = match sock.read(&mut chunk) {
            Ok(0) | Err(_) => break,
            Ok(n) => n,
        };
        frames.clear();
        decoder.push(&chunk[..n], &mut frames);
        for f in &frames {
            if let Some(bytes) = f.data() {
                // An address pair, a control byte and a check sequence the
                // client did not send: shorter than that is not a frame, and
                // queueing it would key the radio for nothing.
                if ax25::parse(bytes).is_ok() {
                    tnc.queue(bytes.to_vec());
                }
                continue;
            }
            tnc.params.lock().unwrap().apply(f);
        }
    }
    tnc.remove(id);
}

/// The receive half: AX.25 off the packet bus, out to every client.
pub struct KissTncNode {
    tnc: Arc<Tnc>,
}

impl Default for KissTncNode {
    fn default() -> Self {
        Self::new(default_address())
    }
}

impl KissTncNode {
    pub fn new(addr: SocketAddr) -> Self {
        Self::attach(server(addr))
    }

    pub fn attach(tnc: Arc<Tnc>) -> Self {
        Self { tnc }
    }

    pub fn tnc(&self) -> &Arc<Tnc> {
        &self.tnc
    }
}

impl Simple for KissTncNode {
    fn name(&self) -> &str {
        TNC.name
    }

    fn is_sink(&self) -> bool {
        true
    }

    fn readings(&self) -> Vec<(String, String)> {
        let where_ = match (self.tnc.bound(), self.tnc.error()) {
            (_, Some(e)) => format!("not listening: {e}"),
            (Some(a), None) => a.to_string(),
            (None, None) => "not listening".into(),
        };
        vec![
            ("serving".into(), where_),
            ("clients".into(), self.tnc.connected().to_string()),
            ("to clients".into(), self.tnc.sent().to_string()),
            ("from clients".into(), self.tnc.received().to_string()),
        ]
    }

    fn negotiate(&mut self, i: &PortSpec) -> Result<StreamSpec> {
        if i.spec.kind != PortKind::Packets {
            return Err(common::Error::other("kiss_tnc reads the packet bus"));
        }
        Ok(i.spec)
    }

    fn process(&mut self, i: &Payload, _o: &mut Payload, _c: &mut NodeCtx<'_>) -> Result<()> {
        for p in i.as_packets().unwrap_or(&[]) {
            let PacketBody::Frame(f) = &p.body else { continue };
            // What a TNC is for is AX.25, and the bus carries every protocol
            // the receiver reads. The band is where the receiver already
            // decides an unlabelled frame is AX.25, and the parse is the
            // decoder's own answer rather than a guess at the bytes.
            if !dsp::afsk::is_packet_band(f.center_hz as f64) {
                continue;
            }
            if ax25::parse(&f.bytes).is_ok() {
                self.tnc.broadcast(&f.bytes);
            }
        }
        Ok(())
    }
}

/// Flags before a frame have to survive being rounded to whole bytes at any
/// baud rate, and a client asking for none would key up into a receiver whose
/// clock has not settled. Two is what a TNC sends at its shortest setting.
pub const MIN_LEAD_FLAGS: usize = 2;

/// The setting both nodes read: where the TNC listens.
pub const ADDRESS: &str = "address";

/// Where a stage that was not told an address listens.
///
/// One address for the process, set from the command line: the receive half
/// is drawn from the plan and the transmit half from the protocol registry on
/// the transmitter's own thread, and the second has no plan to read. A stage
/// an operator placed by hand still carries its own [`ADDRESS`].
static DEFAULT_ADDRESS: Mutex<Option<SocketAddr>> = Mutex::new(None);

pub fn set_default_address(addr: SocketAddr) {
    *DEFAULT_ADDRESS.lock().unwrap() = Some(addr);
}

pub fn default_address() -> SocketAddr {
    DEFAULT_ADDRESS
        .lock()
        .unwrap()
        .unwrap_or_else(|| SocketAddr::from(([127, 0, 0, 1], DEFAULT_PORT)))
}

pub const TNC: StageDesc = StageDesc {
    name: "kiss_tnc",
    summary: "Serve AX.25 frames to packet software over KISS, and take back \
              what it wants transmitted",
    category: Category::Sink,
    feeds_bus: false,
};

fn address(s: &Settings) -> SocketAddr {
    s.str_or(ADDRESS, "").parse().unwrap_or_else(|_| default_address())
}

pub fn build(s: &Settings) -> Result<Box<dyn Node>> {
    Ok(Box::new(KissTncNode::new(address(s))))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A port of the kind a node is being handed, for negotiation.
    fn port(kind: PortKind, rate: f64) -> PortSpec {
        let mut spec = pipeline::StreamSpec::iq(rate, common::Hz(144_800_000));
        spec.kind = kind;
        PortSpec { spec, latency: 0 }
    }

    /// Run one block through a node, the way the graph runs it.
    fn run(node: &mut impl Simple, input: Payload, kind: PortKind) -> Payload {
        let ins = [port(kind, 48_000.0)];
        let (tags, mut events, mut new_tags) = (Vec::new(), Vec::new(), Vec::new());
        let mut out = Payload::empty_of(kind);
        let mut ctx = NodeCtx::new(0, &ins, &tags, &mut events, &mut new_tags);
        node.process(&input, &mut out, &mut ctx).expect("the node ran");
        out
    }

    /// A UI frame from one station to another, which is what a client sends
    /// and what the receiver hears. Laid out by hand: callsigns are shifted
    /// left one bit, leaving the low bit as the address-extension flag.
    fn frame(info: &str) -> Vec<u8> {
        let mut f = Vec::new();
        for (call, ssid, last) in [("APRS  ", 0u8, false), ("EI2ABC", 7, true)] {
            f.extend(call.bytes().map(|c| c << 1));
            f.push(0x60 | (ssid << 1) | u8::from(last));
        }
        f.push(0x03); // UI
        f.push(0xF0); // no layer 3
        f.extend_from_slice(info.as_bytes());
        f
    }

    /// A server on a port the kernel picks, so tests do not collide.
    fn serving() -> (Arc<Tnc>, SocketAddr) {
        let tnc = server(SocketAddr::from(([127, 0, 0, 1], 0)));
        let bound = tnc.bound().expect("the listener bound");
        (tnc, bound)
    }

    /// Wait for something the client threads do, rather than sleeping a
    /// guessed interval: a connection and a read are on other threads.
    fn until(what: &str, mut done: impl FnMut() -> bool) {
        let deadline = std::time::Instant::now() + std::time::Duration::from_secs(5);
        while std::time::Instant::now() < deadline {
            if done() {
                return;
            }
            std::thread::sleep(std::time::Duration::from_millis(5));
        }
        panic!("timed out waiting for {what}");
    }

    fn packet_of(bytes: Vec<u8>, center_hz: u64) -> common::Packet {
        common::Packet::of_frame(
            0,
            16_000,
            common::Frame::measured(bytes, -60.0, 20.0).at(center_hz),
        )
    }

    /// The whole receive direction: frames off the bus, KISS on the socket.
    /// A Mode S frame on the same bus is not AX.25 and must not reach a
    /// client as one.
    #[test]
    fn frames_off_the_bus_reach_a_client_as_kiss() {
        let (tnc, addr) = serving();
        let mut client = TcpStream::connect(addr).expect("connected");
        client.set_read_timeout(Some(std::time::Duration::from_secs(5))).unwrap();
        until("the server to see the client", || tnc.connected() == 1);

        let mut node = KissTncNode::attach(tnc.clone());
        let packets = vec![
            packet_of(frame("!5338.00N/00615.00W-one"), 144_800_000),
            // Seventeen bytes off 1090 MHz: a frame, not an AX.25 one.
            packet_of(vec![0x8D; 14], 1_090_000_000),
            packet_of(frame("!5338.00N/00615.00W-two"), 144_800_000),
        ];
        Simple::negotiate(&mut node, &port(PortKind::Packets, 0.0)).unwrap();
        run(&mut node, Payload::Packets(packets), PortKind::Packets);

        let mut buf = vec![0u8; 512];
        let n = client.read(&mut buf).expect("the client was written to");
        let mut decoder = kiss::Decoder::new();
        let mut got = Vec::new();
        decoder.push(&buf[..n], &mut got);
        // The second frame's opening delimiter closes the first, so a single
        // read can hold both.
        until("both frames", || {
            if got.len() >= 2 {
                return true;
            }
            match client.read(&mut buf) {
                Ok(n) if n > 0 => decoder.push(&buf[..n], &mut got),
                _ => {}
            }
            got.len() >= 2
        });
        assert_eq!(got.len(), 2, "expected two frames, got {}", got.len());
        assert_eq!(tnc.sent(), 2);
        let read: Vec<ax25::Frame> =
            got.iter().filter_map(|f| ax25::parse(f.data().unwrap()).ok()).collect();
        let infos: Vec<String> = read.iter().map(|f| f.info_text()).collect();
        assert_eq!(infos, ["!5338.00N/00615.00W-one", "!5338.00N/00615.00W-two"]);
        assert_eq!(read[0].source.to_string(), "EI2ABC-7");
    }

    /// The whole transmit direction, read back by the demodulator that reads
    /// the air: three frames in over the socket, three frames out of the
    /// audio, byte for byte.
    #[test]
    fn what_a_client_sends_comes_back_out_of_the_demodulator() {
        let (tnc, addr) = serving();
        let mut client = TcpStream::connect(addr).expect("connected");
        // 100 ms of flags, which is ten of them at 1200 baud after the
        // rounding: enough for the correlators and short enough that the
        // test is not seconds of audio.
        client.write_all(&kiss::encode(0, kiss::Command::TxDelay, &[10])).unwrap();
        let sent: Vec<Vec<u8>> = ["first", "second", "third"].iter().map(|s| frame(s)).collect();
        for f in &sent {
            client.write_all(&kiss::data(0, f)).unwrap();
        }
        client.flush().unwrap();
        until("three frames queued", || tnc.queued() == 3);
        assert_eq!(tnc.params().txdelay_ms, 100);
        assert_eq!(tnc.params().lead_flags(dsp::afsk::BELL202.baud), 15);

        let rate = 48_000.0;
        let mut node = crate::aprs_nodes::AprsTxNode::attach(tnc.clone());
        Simple::negotiate(&mut node, &port(PortKind::Real, rate)).unwrap();
        let block = 2_048;
        let mut audio = Vec::new();
        // Six seconds of clock, which is well past the three frames.
        for _ in 0..(6.0 * rate / block as f64) as usize {
            let out = run(&mut node, Payload::Real(vec![0.0; block]), PortKind::Real);
            audio.extend_from_slice(out.as_real().unwrap());
        }
        assert_eq!(node.sent(), 3);
        assert_eq!(tnc.queued(), 0);

        let mut demod = dsp::afsk::AfskDemod::new(rate, dsp::afsk::AfskConfig::default());
        let mut got = Vec::new();
        demod.process(&audio, &mut got);
        assert_eq!(got.len(), 3, "expected three frames, got {}", got.len());
        assert_eq!(got, sent, "a frame came back changed");
    }

    /// A client that is not speaking KISS, or is sending something too short
    /// to be a frame, must not key the radio.
    #[test]
    fn junk_from_a_client_queues_nothing() {
        let (tnc, addr) = serving();
        let mut client = TcpStream::connect(addr).expect("connected");
        client.write_all(b"GET / HTTP/1.1\r\n\r\n").unwrap();
        client.write_all(&kiss::data(0, b"short")).unwrap();
        client.write_all(&kiss::data(0, &[])).unwrap();
        // A frame that does parse, behind the junk, so there is something to
        // wait for rather than an interval to guess at.
        client.write_all(&kiss::data(0, &frame("real"))).unwrap();
        client.flush().unwrap();
        until("the one real frame", || tnc.queued() == 1);
        assert_eq!(tnc.received(), 1, "junk was queued for transmission");

        // And the audio is that one frame and nothing else.
        let rate = 48_000.0;
        let mut node = crate::aprs_nodes::AprsTxNode::attach(tnc.clone());
        Simple::negotiate(&mut node, &port(PortKind::Real, rate)).unwrap();
        run(&mut node, Payload::Real(vec![0.0; 48_000 * 3]), PortKind::Real);
        assert_eq!(node.sent(), 1);
    }

    /// A client's frame goes out ahead of the operator's beacon: a channel
    /// has one transmit source, and somebody at a keyboard is waiting on the
    /// frame where a beacon repeats anyway.
    #[test]
    fn a_client_frame_is_keyed_before_the_beacon() {
        let (tnc, addr) = serving();
        let rate = 48_000.0;
        let mut node = crate::aprs_nodes::AprsTxNode::attach(tnc.clone());
        for (name, value) in [("source", "MI0ABC-9"), ("info", "!5338.00N/00615.00W-beacon")] {
            Simple::set_param(&mut node, name, pipeline::ParamValue::Text(value.into())).unwrap();
        }
        Simple::negotiate(&mut node, &port(PortKind::Real, rate)).unwrap();

        let mut client = TcpStream::connect(addr).expect("connected");
        client.write_all(&kiss::data(0, &frame("from a client"))).unwrap();
        client.flush().unwrap();
        until("the client's frame queued", || tnc.queued() == 1);

        // Two seconds, which holds the client's frame and the beacon behind
        // it but not a second beacon: the gap between two is two seconds.
        let block = 2_048;
        let mut audio = Vec::new();
        for _ in 0..(2.0 * rate / block as f64) as usize {
            let out = run(&mut node, Payload::Real(vec![0.0; block]), PortKind::Real);
            audio.extend_from_slice(out.as_real().unwrap());
        }
        let mut demod = dsp::afsk::AfskDemod::new(rate, dsp::afsk::AfskConfig::default());
        let mut got = Vec::new();
        demod.process(&audio, &mut got);
        assert_eq!(got.len(), 2, "expected the client's frame then the beacon");
        assert_eq!(ax25::parse(&got[0]).unwrap().info, b"from a client");
        assert_eq!(ax25::parse(&got[1]).unwrap().source.to_string(), "MI0ABC-9");
        assert_eq!(node.sent(), 2);
        assert_eq!(tnc.queued(), 0);
    }

    /// An idle TNC radiates nothing: with no client and nothing queued the
    /// transmit node is silent, sample for sample.
    #[test]
    fn an_idle_tnc_is_silence() {
        let (tnc, _) = serving();
        let mut node = crate::aprs_nodes::AprsTxNode::attach(tnc.clone());
        Simple::negotiate(&mut node, &port(PortKind::Real, 48_000.0)).unwrap();
        let mut audio = Vec::new();
        for _ in 0..100 {
            let out = run(&mut node, Payload::Real(vec![0.0; 2_048]), PortKind::Real);
            audio.extend_from_slice(out.as_real().unwrap());
        }
        assert_eq!(audio.len(), 204_800);
        assert_eq!(audio.iter().filter(|s| **s != 0.0).count(), 0, "an idle TNC made a noise");
        assert_eq!(node.sent(), 0);
    }

    /// Two nodes naming one address are one TNC: the transmit chain is built
    /// separately from the bus consumer and on another thread.
    #[test]
    fn a_transmit_chain_opens_no_socket_of_its_own() {
        // A named port rather than an ephemeral one, because finding the
        // running server by address is what is being tested. Out of the
        // range nothing is registered in.
        let addr =
            SocketAddr::from(([127, 0, 0, 1], 49_152 + (std::process::id() % 16_000) as u16));
        let mut alone = crate::aprs_nodes::AprsTxNode::keying(addr);
        assert!(alone.attached().is_none(), "the transmit half started a server");
        assert_eq!(running(addr).map(|t| t.address()), None);

        // Once the receive half is serving there, the same one is found.
        let rx = KissTncNode::new(addr);
        assert!(Arc::ptr_eq(alone.attached().expect("the server was found"), rx.tnc()));
        assert_eq!(SERVERS.lock().unwrap().iter().filter(|(a, _)| *a == addr).count(), 1);
    }
}
