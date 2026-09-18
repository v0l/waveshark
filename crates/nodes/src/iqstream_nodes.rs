//! Handing the span out over the network, as an iqstream server.
//!
//! The receiver reads other people's tuners through `remote::iqstream`; this
//! is the other direction. A stage on the head publishes what the radio is
//! hearing to any number of subscribers, so a decoder on another machine can
//! read the same samples without a second dongle and without taking this one
//! away.
//!
//! # A remote tune is a request like any other
//!
//! Nothing here moves the radio. A subscriber's `TUNE` arrives on the server's
//! own thread and is parked; [`process`](IqStreamServerNode::process) picks it
//! up on the next block and emits a [`Request::Retune`], which is the same
//! thing the band walk in `scan_nodes` emits and is answered by whoever holds
//! the device. Where the tuner actually landed comes back as the next
//! negotiated centre, and that is what is sent to the subscribers.
//!
//! The consequence is deliberate and worth stating: a granted remote tune
//! moves the dial on the local screen, because there is one tuner and one
//! frequency. That is why the stage refuses to advertise itself as tunable
//! unless an operator turned it on.
//!
//! # The server outlives the graph
//!
//! A listening socket cannot be opened and closed on every rebuild: a rebuild
//! happens on every retune, and a port in TIME_WAIT would refuse the next one.
//! So servers are kept in a table by the address they were asked for and the
//! stage borrows one, exactly as `remote` keeps its connections.

use common::{Hz, Result, SampleFormat};
use pipeline::event::{Event, Request};
use pipeline::node::{NodeCtx, PortSpec, Simple};
use pipeline::port::{Payload, PortKind, StreamSpec};
use pipeline::registry::{Category, Settings, StageDesc};
use pipeline::SettingsExt;
use std::collections::HashMap;
use std::net::SocketAddr;
use std::sync::{Arc, Mutex, OnceLock};

/// Where a server listens when nothing has said otherwise.
///
/// 1234 is what `iqstreamd` and `rtl_tcp` both serve on, so a client pointed
/// at a bare host finds this without being told a port.
pub const DEFAULT_PORT: u16 = 1234;

/// Every server started in this process, by the address it was asked for.
///
/// Held for the life of the program rather than dropped with the graph: see
/// the module note. A second stage naming the same address gets the running
/// server, not a second listener on a port already in use.
fn servers() -> &'static Mutex<HashMap<SocketAddr, Arc<iqstream::Server>>> {
    static SERVERS: OnceLock<Mutex<HashMap<SocketAddr, Arc<iqstream::Server>>>> = OnceLock::new();
    SERVERS.get_or_init(Default::default)
}

/// The server on this address, started if it is not running yet.
///
/// A port of zero is never shared: it asks the kernel for a free port, so two
/// of them are two servers however the request was written.
fn server(addr: SocketAddr, cfg: iqstream::ServerConfig) -> Result<Arc<iqstream::Server>> {
    if addr.port() != 0
        && let Ok(map) = servers().lock()
        && let Some(s) = map.get(&addr)
    {
        return Ok(s.clone());
    }
    let s = iqstream::Server::start(addr, cfg)?;
    if addr.port() != 0
        && let Ok(mut map) = servers().lock()
    {
        map.insert(addr, s.clone());
    }
    Ok(s)
}

pub struct IqStreamServerNode {
    address: String,
    tunable: bool,
    server: Option<Arc<iqstream::Server>>,
    center: Hz,
    rate: f64,
    /// Where the tuner reaches, so a subscriber can be offered a dial with
    /// ends on it. Taken from the negotiated span when nothing better is
    /// known: a stage cannot see the device, and a range of the span alone is
    /// at least true.
    span: (u64, u64),
    /// Interleaved UC8, rebuilt per block and kept to save the allocation.
    uc8: Vec<u8>,
    /// What was last asked for, so the same request is not emitted every block
    /// while the device takes its time.
    asked: Option<u64>,
}

impl Default for IqStreamServerNode {
    fn default() -> Self {
        Self::new(&format!("0.0.0.0:{DEFAULT_PORT}"), false)
    }
}

impl IqStreamServerNode {
    pub fn new(address: &str, tunable: bool) -> Self {
        Self {
            address: address.into(),
            tunable,
            server: None,
            center: Hz(0),
            rate: 0.0,
            span: (0, 0),
            uc8: Vec::new(),
            asked: None,
        }
    }

    /// Start the server, or take the one already listening on that address.
    ///
    /// A port that will not bind is logged and left: a receiver that stopped
    /// because somebody else had 1234 would be a worse fault than one that
    /// serves nothing.
    fn attach(&mut self) -> Option<&Arc<iqstream::Server>> {
        if self.server.is_none() {
            let addr: SocketAddr = self.address.parse().ok()?;
            let cfg = iqstream::ServerConfig {
                name: "waveshark".into(),
                center_hz: self.center.0,
                sample_rate: self.rate as u32,
                gain_db: None,
                tunable: self.tunable,
                tune_range_hz: Some(self.span),
            };
            match server(addr, cfg) {
                Ok(s) => self.server = Some(s),
                Err(e) => {
                    tracing::warn!("iqstream_server: {}: {e}", self.address);
                    return None;
                }
            }
        }
        self.server.as_ref()
    }
}

impl Simple for IqStreamServerNode {
    fn name(&self) -> &str {
        DESC.name
    }

    fn is_sink(&self) -> bool {
        true
    }

    fn readings(&self) -> Vec<(String, String)> {
        let (serving, readers, blocks) = match &self.server {
            Some(s) => (s.addr().to_string(), s.subscribers().to_string(), s.blocks_sent()),
            None => ("not listening".into(), "0".into(), 0),
        };
        vec![
            ("serving".into(), serving),
            ("readers".into(), readers),
            ("blocks".into(), blocks.to_string()),
            ("dial".into(), if self.tunable { "offered" } else { "held here" }.into()),
        ]
    }

    fn negotiate(&mut self, i: &PortSpec) -> Result<StreamSpec> {
        if i.spec.kind != PortKind::Iq {
            return Err(common::Error::other("iqstream_server needs IQ"));
        }
        self.center = i.spec.center;
        self.rate = i.spec.rate;
        // The span the radio is on, widened enough that a subscriber's dial
        // has somewhere to go. Not the tuner's real range, which a stage
        // cannot see, so a request outside it is refused by the device rather
        // than by the server.
        let half = (i.spec.rate / 2.0) as u64;
        self.span = (self.center.0.saturating_sub(half.max(1)), self.center.0 + half.max(1));
        // A rate or a centre change is a different stream. The subscribers
        // are told the new centre; a rate change they cannot be told about,
        // so the server is let go and the next block starts a fresh one.
        if let Some(s) = &self.server {
            if s.sample_rate() != self.rate as u32 {
                self.server = None;
            } else {
                s.retuned(self.center.0);
            }
        }
        Ok(i.spec)
    }

    fn process(&mut self, i: &Payload, _o: &mut Payload, c: &mut NodeCtx<'_>) -> Result<()> {
        let iq = i.as_iq().unwrap_or(&[]);
        let Some(server) = self.attach() else {
            return Ok(());
        };
        let server = server.clone();

        // What a subscriber asked for, forwarded once. The device answers by
        // retuning, which arrives back here as a negotiate with a new centre.
        if let Some(t) = server.wanted()
            && self.asked != Some(t.center_hz)
        {
            self.asked = Some(t.center_hz);
            c.emit(Event::Request(Request::Retune { center_hz: t.center_hz as f64 }));
        }
        if self.asked == Some(self.center.0) {
            self.asked = None;
        }

        if !iq.is_empty() {
            // UC8 because that is what the protocol carries, and because
            // eight bits is the resolution most of what feeds this has
            // anyway. A subscriber wanting fewer asks for fewer and the
            // server packs them down.
            SampleFormat::Cu8.encode(iq, &mut self.uc8);
            server.push(&self.uc8);
            self.uc8.clear();
        }
        Ok(())
    }
}

/// Where the server listens, as `host:port`.
pub const ADDRESS: &str = "address";
/// Whether a subscriber may move this receiver's dial.
pub const TUNABLE: &str = "tunable";

pub const DESC: StageDesc = StageDesc {
    name: "iqstream_server",
    summary: "Serve the span to network subscribers over iqstream, so another \
              machine can read the same samples without a second radio",
    category: Category::Sink,
    feeds_bus: false,
};

pub fn build(s: &Settings) -> Result<Box<dyn pipeline::node::Node>> {
    let default = format!("0.0.0.0:{DEFAULT_PORT}");
    let addr = s.str_or(ADDRESS, &default);
    Ok(Box::new(IqStreamServerNode::new(&addr, s.bool_or(TUNABLE, false))))
}

#[cfg(test)]
mod tests {
    use super::*;
    use pipeline::node::Node;

    fn spec(rate: f64, center: Hz) -> PortSpec {
        PortSpec { spec: StreamSpec::iq(rate, center), latency: 0 }
    }

    /// The stage takes IQ and nothing else: audio or a bin frame reaching it
    /// would be published as samples of something that is not spectrum.
    #[test]
    fn the_stage_serves_iq_and_refuses_anything_else() {
        let mut n = IqStreamServerNode::new("127.0.0.1:0", false);
        assert!(Simple::negotiate(&mut n, &spec(2_400_000.0, Hz::mhz(1090))).is_ok());
        let mut audio = StreamSpec::iq(48_000.0, Hz(0));
        audio.kind = PortKind::Real;
        assert!(Simple::negotiate(&mut n, &PortSpec { spec: audio, latency: 0 }).is_err());
    }

    /// The dial is held here unless somebody said otherwise, because granting
    /// it moves the frequency on the local screen.
    #[test]
    fn the_dial_is_not_offered_unless_it_was_asked_for() {
        let held = build(&Settings::new()).unwrap();
        assert_eq!(held.readings().iter().find(|(k, _)| k == "dial").map(|(_, v)| v.as_str()),
                   Some("held here"));
        let mut s = Settings::new();
        s.insert(TUNABLE.into(), pipeline::ParamValue::Bool(true));
        s.insert(ADDRESS.into(), pipeline::ParamValue::Text("127.0.0.1:0".into()));
        let offered = build(&s).unwrap();
        assert_eq!(offered.readings().iter().find(|(k, _)| k == "dial").map(|(_, v)| v.as_str()),
                   Some("offered"));
    }

    /// One block through the stage, and whatever retunes it asked for.
    fn block(n: &mut IqStreamServerNode, samples: usize) -> Vec<f64> {
        let ins = [spec(2_400_000.0, n.center)];
        let (tags, mut events, mut new_tags) = (Vec::new(), Vec::new(), Vec::new());
        let mut out = Payload::Iq(Vec::new());
        let mut ctx = NodeCtx::new(0, &ins, &tags, &mut events, &mut new_tags);
        let iq = Payload::Iq(vec![common::C32::new(0.25, -0.25); samples]);
        Simple::process(n, &iq, &mut out, &mut ctx).unwrap();
        events
            .iter()
            .filter_map(|e| match e {
                Event::Request(Request::Retune { center_hz }) => Some(*center_hz),
                _ => None,
            })
            .collect()
    }

    /// A tune from a subscriber leaves as a retune request and no more than
    /// one: the device takes several blocks to answer, and a request repeated
    /// every block would be a retune storm.
    #[test]
    fn a_subscribers_tune_becomes_one_retune_request() {
        let mut n = IqStreamServerNode::new("127.0.0.1:0", true);
        Node::negotiate(&mut n, &[spec(2_400_000.0, Hz::mhz(1090))]).unwrap();
        assert_eq!(block(&mut n, 64), Vec::<f64>::new(), "nobody asked");
        let server = n.server.clone().expect("a server on a free port");
        assert!(server.tunable());

        // Park a request the way a subscriber's TUNE does, then run blocks.
        // The first carries it out and the next four say nothing, because the
        // device has not answered yet.
        server.ask(433_920_000);
        assert_eq!(block(&mut n, 64), vec![433_920_000.0]);
        for _ in 0..4 {
            assert_eq!(block(&mut n, 64), Vec::<f64>::new(), "asked once");
        }

        // The device answered: the graph is rebuilt on the new centre and the
        // subscribers are told where it landed.
        Node::negotiate(&mut n, &[spec(2_400_000.0, Hz(433_920_000))]).unwrap();
        assert_eq!(server.center_hz(), 433_920_000);
        assert_eq!(block(&mut n, 64), Vec::<f64>::new());

        // And a second, different request is carried out rather than swallowed
        // by the first having been seen.
        server.ask(868_300_000);
        assert_eq!(block(&mut n, 64), vec![868_300_000.0]);
    }

    /// A stage that was not offered the dial does not forward a request, even
    /// if one somehow reaches its server.
    #[test]
    fn a_dial_held_here_forwards_nothing() {
        let mut n = IqStreamServerNode::new("127.0.0.1:0", false);
        Node::negotiate(&mut n, &[spec(2_400_000.0, Hz::mhz(1090))]).unwrap();
        assert_eq!(block(&mut n, 64), Vec::<f64>::new());
        let server = n.server.clone().expect("a server on a free port");
        assert!(!server.tunable());
        server.ask(433_920_000);
        assert_eq!(block(&mut n, 64), Vec::<f64>::new(), "a held dial does not move");
    }
}
