//! The transmitter, on a thread of its own.
//!
//! A transmission is still a graph of stages, drawn and parameterised like
//! everything else the receiver does; what changed is who runs it. The
//! receive chain is clocked by samples arriving from the radio and the
//! transmit chain by the radio taking them away, and those are two clocks:
//! on one thread they run in series, so the receiver waits for the device to
//! swallow a block and the transmission waits for four channel banks to
//! decode a noise floor the radio cannot hear anyway.
//!
//! What crosses between them is narrow on purpose. A built graph goes one
//! way, samples that went out come back through the monitor's queue, and
//! everything a person reads off the transmitter is an atomic.

use common::TxStream;
use parking_lot::Mutex;
use pipeline::Graph;
use pipeline::graph::Topology;
use pipeline::param::ParamValue;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicU32, AtomicU64, Ordering};
use std::time::{Duration, Instant};

/// How much of a transmission may be built ahead of real time.
///
/// The device's queue is the pace once it is full; until then nothing stops
/// the chain, so it would build the file as fast as the machine can and the
/// radio would be a minute behind what the operator sees. A fifth of a
/// second is enough to ride out a block that took longer than its own
/// length.
const AHEAD: Duration = Duration::from_millis(200);

/// The block the clock hands over, in seconds. Short enough that unkeying is
/// prompt and long enough that a modulator is not called for a handful of
/// symbols at a time.
const BLOCK_S: f64 = 0.02;

/// What the interface reads off a transmitter it cannot reach into.
#[derive(Default)]
pub struct Readings {
    pub keyed: AtomicBool,
    /// Samples handed to the radio since the key went down.
    pub written: AtomicU64,
    /// Transfers the radio sent as silence for want of anything queued.
    pub underruns: AtomicU64,
    /// The microphone's peak, as `f32` bits: the meter an operator sets a
    /// level against.
    pub mic_peak: AtomicU32,
    pub mic_clipped: AtomicBool,
    /// Whether the chain has a vox in it at all, since the rest of these
    /// mean nothing without one.
    pub vox: AtomicBool,
    /// Whether the vox says the key should be down. Read by the radio loop,
    /// which is the only thing that can key: this thread holds the chain and
    /// not the radio's dial.
    pub vox_open: AtomicBool,
    /// The level it is deciding on, as `f32` bits, for the control the
    /// threshold is set on, and whether the receiver playing is what is
    /// holding the key up.
    pub vox_level: AtomicU32,
    pub vox_held: AtomicBool,
    /// Whether the courtesy tone that ends the over is still going out, so
    /// the key is held down until it has. Set by whoever let the key up,
    /// cleared by the thread when the stage has sent it.
    pub roger: AtomicBool,
    /// Blocks the radio refused, which is how a device unplugged in the
    /// middle of an over shows up: the stream reports every write failing
    /// and there is nothing else to notice it by.
    pub refused: AtomicU64,
    /// Set when the over ended because the radio went away rather than
    /// because anybody let the key up.
    pub lost: AtomicBool,
    /// The chain as it is running, republished as it costs.
    pub topo: Mutex<Option<Topology>>,
    /// What any scope the operator hung off the transmit chain last drew.
    /// A scope watching the modulator runs on this thread, so its frames
    /// cross to the interface the same way the readings do.
    pub scopes: Mutex<Vec<(usize, nodes::ScopeFrame)>>,
}

impl Readings {
    pub fn mic_peak(&self) -> f32 {
        f32::from_bits(self.mic_peak.load(Ordering::Relaxed))
    }
}

enum Job {
    /// A chain to run from now on, and whether it runs before it is keyed.
    Chain(Box<Graph>, bool),
    Key(Box<dyn TxStream>),
    /// The over has ended, told to the chain so the stage that ends one can
    /// send its tone before the radio goes back.
    EndOver,
    Unkey,
    /// Nothing left to transmit: the chain goes, and with it anything that
    /// was waiting to be put on it.
    Drop,
    Param(usize, String, ParamValue),
    /// Answered once everything sent before it has been done, for a caller
    /// that has to know the chain is running what it just asked for.
    Settled(crossbeam_channel::Sender<()>),
    Stop,
}

/// A handle on the thread that transmits.
pub struct Transmitter {
    to: crossbeam_channel::Sender<Job>,
    thread: Option<std::thread::JoinHandle<()>>,
    readings: Arc<Readings>,
    /// The chain it was last given, so a rebuild that changes nothing about
    /// the transmitter leaves it alone: rebuilding would restart whatever
    /// the source has open, which for a film is a decoder half a second in.
    built: Option<crate::patch::Patch>,
    /// Whether the chain it is running was built with a microphone to hand.
    mic: bool,
    /// Whether a radio has been handed over and not taken back. Kept here
    /// rather than read off the thread because whoever keyed has to know
    /// before the thread has run again, and `keyed` is only the intent.
    armed: bool,
}

impl Default for Transmitter {
    fn default() -> Self {
        Self::new()
    }
}

impl Transmitter {
    pub fn new() -> Self {
        let (to, work) = crossbeam_channel::unbounded::<Job>();
        let readings = Arc::new(Readings::default());
        let mine = readings.clone();
        let thread =
            std::thread::Builder::new().name("transmit".into()).spawn(move || run(work, mine)).ok();
        Self { to, thread, readings, built: None, mic: false, armed: false }
    }

    pub fn readings(&self) -> &Arc<Readings> {
        &self.readings
    }

    pub fn keyed(&self) -> bool {
        self.readings.keyed.load(Ordering::Relaxed)
    }

    /// Whether the radio is on a chain that can carry what it makes.
    ///
    /// [`Self::keyed`] is the key being down, true from the moment it is
    /// pressed; this is there being somewhere to put the result. A key lit
    /// over a transmitter with no chain is the one state an operator must
    /// never be shown, because nothing on the screen would say it is not on
    /// air.
    pub fn on_air(&self) -> bool {
        self.armed && self.built.is_some()
    }

    /// Whether a chain of this shape is already running, so a key-up can go
    /// straight on it without waiting for a rebuild.
    pub fn ready(&self, mic: bool) -> bool {
        self.built.is_some() && self.mic == mic
    }

    /// Whether the last over ended because the radio went away. Cleared by
    /// reading it, since it is news once.
    pub fn lost(&self) -> bool {
        self.readings.lost.swap(false, Ordering::Relaxed)
    }

    /// What the scopes on the transmit chain last drew.
    pub fn scope_frames(&self) -> Vec<(usize, nodes::ScopeFrame)> {
        self.readings.scopes.lock().clone()
    }

    /// The chain it is running, for the chain view.
    pub fn topology(&self) -> Option<Topology> {
        self.readings.topo.lock().clone()
    }

    /// Whether this is the chain it is already running. The microphone
    /// counts: it is handed in rather than described, so a chain built
    /// before one arrived is a chain with a stage missing.
    pub fn is_running(&self, patch: &crate::patch::Patch, mic: bool) -> bool {
        self.built.as_ref().is_some_and(|b| b == patch) && self.mic == mic
    }

    /// Give it a chain to run. `idle` says whether it runs before the key
    /// goes down, which a microphone's does so its meter moves.
    pub fn set_chain(&mut self, patch: crate::patch::Patch, graph: Graph, idle: bool, mic: bool) {
        self.built = Some(patch);
        self.mic = mic;
        // Published here rather than when the thread first runs it: the
        // chain view draws a transmitter that has never been keyed, and a
        // chain nobody can see until it transmits is no use for setting up.
        *self.readings.topo.lock() = Some(graph.topology());
        let _ = self.to.send(Job::Chain(Box::new(graph), idle));
    }

    /// Nothing to transmit: whatever it was running is dropped.
    ///
    /// The chain goes on the thread as well as here. Left there it would go
    /// on running whenever it runs idle, republishing the topology this just
    /// blanked, and a later key would attach the radio to it and transmit
    /// the chain the receiver believes it no longer has.
    pub fn clear(&mut self) {
        if self.built.take().is_some() || self.armed {
            self.armed = false;
            *self.readings.topo.lock() = None;
            let _ = self.to.send(Job::Drop);
        }
    }

    /// Hand it the radio.
    ///
    /// Held until there is a chain to put it on, so a key that arrives while
    /// the graph is being rebuilt for it is not lost, and carried across a
    /// rebuild that happens mid-over.
    pub fn key(&mut self, stream: Box<dyn TxStream>) -> bool {
        self.armed = true;
        self.readings.keyed.store(true, Ordering::Relaxed);
        self.to.send(Job::Key(stream)).is_ok()
    }

    /// Tell the chain the over has ended.
    ///
    /// True when a courtesy tone is on its way out, which is the answer to
    /// whether the key may come up yet: it is the same answer whether a
    /// voice, a hand or the agent ended the over, because all three arrive
    /// here. Reported from the chain that was built rather than from the
    /// thread, since whoever asks has to know before the thread runs again.
    pub fn end_over(&mut self) -> bool {
        let ms = self
            .built
            .as_ref()
            .and_then(|p| p.stage(crate::chain::derived::ROGER))
            .map(|s| s.settings.get("roger_ms").and_then(|v| v.as_f64()).unwrap_or(0.0))
            .unwrap_or(0.0);
        if ms <= 0.0 || !self.armed {
            return false;
        }
        self.readings.roger.store(true, Ordering::Relaxed);
        self.to.send(Job::EndOver).is_ok()
    }

    /// Whether the courtesy tone is still going out.
    pub fn sending_roger(&self) -> bool {
        self.readings.roger.load(Ordering::Relaxed)
    }

    /// Let the queue out and give the radio back, reporting the idle
    /// transfers the over cost.
    pub fn unkey(&mut self) -> u64 {
        self.armed = false;
        self.readings.roger.store(false, Ordering::Relaxed);
        // Marked up here rather than on the thread: the interface asks
        // whether it is still transmitting in the same breath as telling it
        // to stop, and an answer that lags the key by a block reads as a key
        // that did not take.
        self.readings.keyed.store(false, Ordering::Relaxed);
        let _ = self.to.send(Job::Unkey);
        // The thread takes the stream down; what it read off it last is what
        // the over cost.
        self.readings.underruns.load(Ordering::Relaxed)
    }

    /// Record a setting sent to the running chain, so the next rebuild sees
    /// the chain it is already running rather than a different one and
    /// rebuilds it: a level moved mid-transmission would otherwise restart
    /// whatever the source has open.
    pub fn note_param(&mut self, stage: u64, name: &str, value: ParamValue) {
        if let Some(st) = self.built.as_mut().and_then(|p| p.stage_mut(stage)) {
            st.settings.insert(name.to_string(), value);
        }
    }

    pub fn set_param(&self, node: usize, name: &str, value: ParamValue) {
        let _ = self.to.send(Job::Param(node, name.to_string(), value));
    }

    /// Wait until the chain is running everything asked of it. For a test,
    /// and for anything that has to transmit what it just set rather than
    /// what it was set to a block ago.
    pub fn settled(&self, patience: Duration) -> bool {
        let (reply, done) = crossbeam_channel::bounded(1);
        self.to.send(Job::Settled(reply)).is_ok() && done.recv_timeout(patience).is_ok()
    }
}

impl Drop for Transmitter {
    fn drop(&mut self) {
        let _ = self.to.send(Job::Stop);
        if let Some(t) = self.thread.take() {
            let _ = t.join();
        }
    }
}

/// The thread: a chain, a clock, and the radio taking what it makes.
fn run(work: crossbeam_channel::Receiver<Job>, readings: Arc<Readings>) {
    let mut graph: Option<Graph> = None;
    let mut idle = false;
    let mut clock = Clock::default();
    // A radio waiting for a chain. Keying is what asks for the chain to be
    // built, so the two arrive in that order about half the time.
    let mut waiting: Option<Box<dyn TxStream>> = None;
    loop {
        let running = graph.is_some() && (idle || readings.keyed.load(Ordering::Relaxed));
        // Parked when there is nothing to do, so an idle receiver does not
        // carry a spinning thread.
        let job = match running {
            true => work.try_recv().ok(),
            false => match work.recv() {
                Ok(job) => Some(job),
                Err(_) => return,
            },
        };
        match job {
            Some(Job::Chain(g, runs_idle)) => {
                let mut g = *g;
                if let Some(old) = graph.take() {
                    hand_back(old, &mut g);
                }
                arm(&mut g, &mut waiting);
                graph = Some(g);
                idle = runs_idle;
                clock = Clock::default();
            }
            Some(Job::Key(stream)) => {
                readings.keyed.store(true, Ordering::Relaxed);
                readings.lost.store(false, Ordering::Relaxed);
                readings.written.store(0, Ordering::Relaxed);
                clock = Clock::default();
                waiting = Some(stream);
                if let Some(g) = graph.as_mut() {
                    arm(g, &mut waiting);
                }
            }
            Some(Job::EndOver) => {
                let sending =
                    graph.as_mut().and_then(roger_of).map(|r| r.end_over()).unwrap_or(false);
                readings.roger.store(sending, Ordering::Relaxed);
            }
            Some(Job::Unkey) => {
                waiting = None;
                readings.roger.store(false, Ordering::Relaxed);
                if let Some(g) = graph.as_mut()
                    && let Some(sink) = sink_of(g)
                {
                    readings.underruns.store(sink.underruns(), Ordering::Relaxed);
                    sink.finish(Duration::from_secs(1));
                }
                readings.keyed.store(false, Ordering::Relaxed);
            }
            Some(Job::Drop) => {
                waiting = None;
                readings.roger.store(false, Ordering::Relaxed);
                if let Some(mut g) = graph.take()
                    && let Some(sink) = sink_of(&mut g)
                {
                    readings.underruns.store(sink.underruns(), Ordering::Relaxed);
                    sink.finish(Duration::from_secs(1));
                }
                readings.keyed.store(false, Ordering::Relaxed);
                *readings.topo.lock() = None;
                *readings.scopes.lock() = Vec::new();
            }
            Some(Job::Param(node, name, value)) => {
                if let Some(g) = graph.as_mut() {
                    if let Some(n) = g.node_mut(pipeline::graph::NodeId(node))
                        && let Err(e) = pipeline::node::Node::set_param(n, &name, value)
                    {
                        tracing::warn!("the transmitter refused {name}: {e}");
                    }
                    follow_the_multiplex(g);
                    // Republished here as well as after a block: with
                    // nothing keyed the chain is not running, and a setting
                    // that does not come back is a control that springs
                    // back to where it was.
                    *readings.topo.lock() = Some(g.topology());
                }
            }
            Some(Job::Settled(reply)) => {
                let _ = reply.send(());
            }
            Some(Job::Stop) => return,
            None => {}
        }
        let Some(g) = graph.as_mut() else { continue };
        if !(idle || readings.keyed.load(Ordering::Relaxed)) {
            continue;
        }
        if let Some(wait) = clock.wait() {
            std::thread::sleep(wait);
            continue;
        }
        let n = (g.topology().input.rate * BLOCK_S).max(1.0) as usize;
        let block = vec![0.0f32; n];
        // Blocks here while the radio is keyed, which is the point: the
        // device taking samples away is the transmitter's clock.
        if let Err(e) = g.feed_real(&block) {
            tracing::warn!("the transmit chain stopped: {e}");
            graph = None;
            continue;
        }
        clock.ran(BLOCK_S);
        if read_off(g, &readings) {
            // The radio went away mid-over. Ending it here is what tells the
            // interface: a key that stays lit over a transmitter that is not
            // transmitting is worse than one that drops.
            tracing::warn!("the radio stopped taking samples: the over is over");
            if let Some(sink) = sink_of(g) {
                sink.finish(Duration::from_millis(100));
            }
            readings.lost.store(true, Ordering::Relaxed);
            readings.keyed.store(false, Ordering::Relaxed);
        }
    }
}

/// What a person reads off the transmitter, published where they can.
fn read_off(g: &mut Graph, readings: &Readings) -> bool {
    let mut lost = false;
    if let Some(sink) = sink_of(g) {
        let (written, underruns) = (sink.written(), sink.underruns());
        readings.written.store(written, Ordering::Relaxed);
        readings.underruns.store(underruns, Ordering::Relaxed);
        // A radio that has refused this many blocks in a row is gone, not
        // busy: every write after the first failure fails the same way.
        let refused = sink.failed_blocks();
        readings.refused.store(refused, Ordering::Relaxed);
        lost = refused >= REFUSALS;
    }
    let mic = g
        .by_tag(crate::chain::derived::TX_SOURCE)
        .and_then(|id| g.node_mut(id))
        .and_then(|n| n.as_any_mut().downcast_mut::<nodes::MicNode>())
        .map(|m| (m.peak(), m.input_clipped()));
    if let Some((peak, clipped)) = mic {
        readings.mic_peak.store(peak.to_bits(), Ordering::Relaxed);
        readings.mic_clipped.store(clipped, Ordering::Relaxed);
    }
    if let Some(r) = roger_of(g) {
        let sending = r.sending();
        // Only ever cleared here: the key went down again while the tone was
        // going out, and the stage was reset with the rest of the chain.
        if !sending {
            readings.roger.store(false, Ordering::Relaxed);
        }
    }
    let vox = g
        .by_tag(crate::chain::derived::VOX)
        .and_then(|id| g.node_mut(id))
        .and_then(|n| n.as_any_mut().downcast_mut::<nodes::VoxNode>())
        .map(|v| (v.is_open(), v.level(), v.held_off()));
    readings.vox.store(vox.is_some(), Ordering::Relaxed);
    if let Some((open, level, held)) = vox {
        readings.vox_open.store(open, Ordering::Relaxed);
        readings.vox_level.store(level.to_bits(), Ordering::Relaxed);
        readings.vox_held.store(held, Ordering::Relaxed);
    }
    *readings.topo.lock() = Some(g.topology());
    *readings.scopes.lock() = scope_frames(g);
    lost
}

/// Keep what the source sends in step with what the modulation carries.
///
/// The two are the same multiplex described twice, and changing the
/// constellation or the code rate changes how much it holds: 64-QAM 2/3 is
/// 24.1 Mbit/s and QPSK 1/2 is 6.0. A source still feeding the old rate
/// backs the queue up until packets are dropped, and a packet dropped out of
/// the middle of a multiplex takes the tables with it, so the picture goes
/// and the service list empties while the signal still looks right.
fn follow_the_multiplex(g: &mut Graph) {
    let Some(want) = g
        .by_tag(crate::chain::derived::TX_MOD)
        .and_then(|id| g.node_mut(id))
        .and_then(|n| n.as_any_mut().downcast_ref::<nodes::dvbt_nodes::DvbtModNode>())
        .map(|m| m.bitrate())
    else {
        return;
    };
    if let Some(id) = g.by_tag(crate::chain::derived::TX_SOURCE)
        && let Some(n) = g.node_mut(id)
        && n.as_any_mut().downcast_ref::<nodes::dvbt_nodes::TsSourceNode>().is_some()
    {
        let _ = pipeline::node::Node::set_param(n, "bitrate", pipeline::ParamValue::Float(want));
    }
}

/// The frames of every scope in the chain, by node id.
fn scope_frames(g: &mut Graph) -> Vec<(usize, nodes::ScopeFrame)> {
    let ids: Vec<_> = g.order().map(|(id, _)| id).collect();
    let mut out = Vec::new();
    for id in ids {
        let Some(scope) = g
            .node_mut(id)
            .map(|n| n.as_any_mut())
            .and_then(|a| a.downcast_mut::<nodes::ScopeNode>())
        else {
            continue;
        };
        let (frame, _) = scope.frame();
        if !frame.spectrum.is_empty() || frame.peak > 0.0 {
            out.push((id.0, frame.clone()));
        }
    }
    out
}

/// Blocks the radio may refuse before the over is given up as lost. More
/// than one, because a single failure could be a stall; a device that has
/// been unplugged refuses every one.
const REFUSALS: u64 = 3;

/// Put the radio on the chain, if both are to hand.
fn arm(g: &mut Graph, waiting: &mut Option<Box<dyn TxStream>>) {
    let Some(stream) = waiting.take() else { return };
    match sink_of(g) {
        Some(sink) => sink.attach(stream),
        // A chain with no transmit stage is not one a radio can go on. Held
        // for the next chain rather than dropped, since the key is still
        // down.
        None => *waiting = Some(stream),
    }
}

/// Move the radio from the chain being replaced to the one replacing it, so
/// a rebuild during an over does not end the over.
fn hand_back(mut old: Graph, new: &mut Graph) {
    let stream = sink_of(&mut old).and_then(|s| s.detach());
    if let Some(stream) = stream
        && let Some(sink) = sink_of(new)
    {
        sink.attach(stream);
    }
}

fn sink_of(g: &mut Graph) -> Option<&mut nodes::TxSinkNode> {
    let id = g.by_tag(crate::chain::derived::TX_RADIO)?;
    g.node_mut(id)?.as_any_mut().downcast_mut::<nodes::TxSinkNode>()
}

fn roger_of(g: &mut Graph) -> Option<&mut nodes::RogerNode> {
    let id = g.by_tag(crate::chain::derived::ROGER)?;
    g.node_mut(id)?.as_any_mut().downcast_mut::<nodes::RogerNode>()
}

/// Keeps the chain from running away from real time before the radio's own
/// queue is full enough to pace it.
struct Clock {
    start: Instant,
    made_s: f64,
}

impl Default for Clock {
    fn default() -> Self {
        Self { start: Instant::now(), made_s: 0.0 }
    }
}

impl Clock {
    fn ran(&mut self, seconds: f64) {
        self.made_s += seconds;
    }

    /// How long to wait, or `None` to carry straight on.
    fn wait(&self) -> Option<Duration> {
        let ahead = Duration::from_secs_f64(self.made_s).checked_sub(self.start.elapsed())?;
        (ahead > AHEAD).then(|| (ahead - AHEAD).min(Duration::from_millis(20)))
    }
}

/// Where a transmit node's id starts when the two chains are published as
/// one, so an interface holding an id knows which of the two it names.
///
/// Larger than any graph the receiver builds: a chain of a million stages is
/// not a thing a person can draw, and the spare bits cost nothing.
pub const TX_ID_BASE: usize = 1 << 20;

/// The two chains as one topology, the transmit side's ids and slots moved
/// out of the way of the receiver's.
pub fn merged(rx: &Topology, tx: Option<&Topology>) -> Topology {
    let Some(tx) = tx else {
        return rx.clone();
    };
    let mut out = rx.clone();
    let slot_base = out.rates.len();
    out.rates.extend(tx.rates.iter().copied());
    for node in &tx.nodes {
        let mut node = node.clone();
        node.id = pipeline::graph::NodeId(node.id.0 + TX_ID_BASE);
        for (slot, _) in node.inputs.iter_mut().chain(node.outputs.iter_mut()) {
            *slot += slot_base;
        }
        out.nodes.push(node);
    }
    out
}
