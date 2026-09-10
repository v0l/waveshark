//! The `Node` trait: one block in the signal flow graph.

use crate::event::{Event, Request};
use crate::param::{Param, ParamValue};
use crate::port::{Payload, StreamSpec, Tag};
use common::Result;
use std::any::Any;

/// Seconds of silence a node needs after a transmission to finish with it,
/// unless it says otherwise. See [`Node::flush_s`].
pub const FLUSH_S: f64 = 0.25;

/// Downcasting, given to every node rather than opted into.
///
/// A host reads state a node exposes beyond its ports by asking for the
/// concrete type back: an RDS decoder's station, a PLL's lock, a transmit
/// sink's stream. This was a `Node` method defaulting to `None` that every
/// node had to remember to override, and the one node that forgot was
/// invisible to all of those readers with nothing to say it should not be.
pub trait AsAny: Any {
    fn as_any(&self) -> &dyn Any;
    fn as_any_mut(&mut self) -> &mut dyn Any;
    /// Downcast an owned node back to its concrete type, for when a graph is
    /// taken apart and something inside a node has to come out with it: a
    /// recorder's open file, which must survive the graph being rebuilt
    /// around it.
    fn into_any(self: Box<Self>) -> Box<dyn Any>;
}

impl<T: Any> AsAny for T {
    fn as_any(&self) -> &dyn Any {
        self
    }
    fn as_any_mut(&mut self) -> &mut dyn Any {
        self
    }
    fn into_any(self: Box<Self>) -> Box<dyn Any> {
        self
    }
}

/// The optional half of [`Node`], written once so [`Simple`] carries the
/// same set rather than a hand-copied subset of it.
///
/// The two traits drifted apart while the list was maintained by hand:
/// `optional_inputs` existed on one and not the other, and the flush default
/// was spelled twice.
macro_rules! node_options {
    () => {
        /// The graphs this node runs inside itself, if it is a composite.
        ///
        /// A node that owns an inner graph is still one node to the
        /// scheduler, which is how a bank keeps hundreds of channels behind
        /// a single channelizer and one batched transform. Without this the
        /// work it does would be invisible: a view of the chain would show a
        /// box labelled "bank" and no sign of the decoder every channel is
        /// running.
        ///
        /// Several rather than one, because a composite is not always one
        /// chain repeated: the auto node holds a graph per front end on
        /// every source it has open, and one of them drawn is the rest of
        /// its work missing from the drawing. A bank returns the one chain
        /// all its channels run and says how many run it through
        /// [`Node::subgraph_count`].
        fn subgraphs(&self) -> Vec<crate::graph::Topology> {
            Vec::new()
        }

        /// How many times the graphs above run per block, for a view that
        /// draws one branch and says how many there are.
        fn subgraph_count(&self) -> usize {
            1
        }

        /// The nodes this one runs inside itself, for a caller that has to
        /// reach every node in the receiver rather than only the ones the
        /// graph holds directly.
        ///
        /// [`Node::subgraphs`] says what the inner graphs look like; this
        /// lends the nodes themselves out, so a capability asked of every
        /// node is asked of a composite's too. Without it the answer stops
        /// at the composite: a key installed on every front end reached the
        /// one placed by hand and not the one an auto node built for a
        /// source it found.
        fn each_inner(&self, _f: &mut dyn FnMut(&dyn Node)) {}

        /// The mutable counterpart of [`Node::each_inner`].
        fn each_inner_mut(&mut self, _f: &mut dyn FnMut(&mut dyn Node)) {}

        /// Where the time inside one call goes, for a node that does several
        /// things per block and can say which cost what. The graph measures
        /// the call as a whole; this is the breakdown only the node can
        /// give.
        fn phases(&self) -> Vec<(String, crate::cost::Cost)> {
            Vec::new()
        }

        /// Whether this node ends the stream rather than passing one on.
        ///
        /// A spectrum display, a recorder and a channel bank all consume
        /// samples and write no buffer. The graph still gives them an output
        /// slot, because every node has one, so without saying so they look
        /// like stages with an output nobody happens to have connected.
        fn is_sink(&self) -> bool {
            false
        }

        /// Whether an input port nothing feeds is acceptable, and read as
        /// silence, rather than a build error.
        ///
        /// A mixer has a spare input by nature: the next thing to be heard
        /// is wired into it, and until then it carries nothing. Every other
        /// node's unfed input is a mistake, and the build refuses it so that
        /// a chain with a wire missing cannot run and produce something that
        /// looks like a result.
        fn optional_inputs(&self) -> bool {
            false
        }

        /// Whether this node is meant to be fed by both directions at once.
        ///
        /// False everywhere but the transmit monitor, which puts what went
        /// to the antenna back onto what the receiver hears. Everywhere else
        /// a receive stream and a transmit stream meeting is a wiring
        /// mistake that only shows up as nonsense on air, so the graph
        /// refuses it; a node that joins them on purpose has to say so.
        fn joins_flows(&self) -> bool {
            false
        }

        /// Seconds of silence this node needs after a transmission to finish
        /// with it: what a decoder that ends an over on a timeout has to
        /// hear before it says the over ended. A source's decoders are
        /// dropped once they have had it.
        fn flush_s(&self) -> f64 {
            crate::node::FLUSH_S
        }

        /// Drop all history: called on retune, stream restart, or channel
        /// reuse.
        fn reset(&mut self) {}

        /// Take what a stage's description says that [`Node::set_param`]
        /// cannot carry one value at a time, as the graph is built.
        ///
        /// The band a bank or a detector is limited to, how many things feed
        /// the packet bus, the passband a decimator designs from the rate it
        /// is cutting: each needs more than one setting at once and each has
        /// to be settled before the node goes in, so the build hands every
        /// node the whole description and each takes what it understands. A
        /// host that reached in by name instead had to know which kinds of
        /// node cared.
        fn configure(&mut self, _settings: &crate::registry::Settings) {}

        /// Whether this node can be reused for its stage in a rebuilt graph,
        /// given whether the span moved under it and what the stage now asks
        /// for.
        ///
        /// True unless a node says otherwise, because carrying what a node
        /// has learned across a rebuild is the point: a bank rebuilds itself
        /// internally on a retune and keeps its several hundred channels
        /// rather than building them again.
        fn survives_rebuild(&self, _retuned: bool, _settings: &crate::registry::Settings) -> bool {
            true
        }

        fn params(&self) -> Vec<Param> {
            Vec::new()
        }

        fn set_param(&mut self, name: &str, _value: ParamValue) -> Result<()> {
            Err(common::Error::other(format!(
                "{}: unknown parameter {name:?}",
                self.name()
            )))
        }
    };
}

/// The same set, handed from a [`Simple`] node up to its [`Node`] impl.
macro_rules! forward_node_options {
    () => {
        fn subgraphs(&self) -> Vec<crate::graph::Topology> {
            Simple::subgraphs(self)
        }
        fn subgraph_count(&self) -> usize {
            Simple::subgraph_count(self)
        }
        fn each_inner(&self, f: &mut dyn FnMut(&dyn Node)) {
            Simple::each_inner(self, f)
        }
        fn each_inner_mut(&mut self, f: &mut dyn FnMut(&mut dyn Node)) {
            Simple::each_inner_mut(self, f)
        }
        fn phases(&self) -> Vec<(String, crate::cost::Cost)> {
            Simple::phases(self)
        }
        fn is_sink(&self) -> bool {
            Simple::is_sink(self)
        }
        fn optional_inputs(&self) -> bool {
            Simple::optional_inputs(self)
        }
        fn joins_flows(&self) -> bool {
            Simple::joins_flows(self)
        }
        fn flush_s(&self) -> f64 {
            Simple::flush_s(self)
        }
        fn reset(&mut self) {
            Simple::reset(self)
        }
        fn configure(&mut self, settings: &crate::registry::Settings) {
            Simple::configure(self, settings)
        }
        fn survives_rebuild(&self, retuned: bool, settings: &crate::registry::Settings) -> bool {
            Simple::survives_rebuild(self, retuned, settings)
        }
        fn params(&self) -> Vec<Param> {
            Simple::params(self)
        }
        fn set_param(&mut self, name: &str, value: ParamValue) -> Result<()> {
            Simple::set_param(self, name, value)
        }
    };
}

/// What a node is told about one of its input ports during negotiation.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PortSpec {
    pub spec: StreamSpec,
    /// Cumulative group delay from the graph source to this port, expressed in
    /// samples at *this port's* rate.
    ///
    /// Fan-in is the whole reason this exists. Two paths into a merge node
    /// almost never have equal delay, because each filter along the way adds
    /// its own. Without this a combiner silently adds misaligned signals, which
    /// looks like a mysterious loss of SNR rather than an obvious bug.
    pub latency: u64,
}

/// Per-call context.
pub struct NodeCtx<'a> {
    /// Index of the first input sample in this call, from stream start.
    pub sample_index: u64,
    /// How much time the block being processed covers, in seconds, measured
    /// at the graph's own input.
    ///
    /// The run's clock, for a node whose output is paced by time rather than
    /// by its input: a bus mixing speech has to produce a block's worth of
    /// audio whether or not anybody spoke during it. Taking the span as an
    /// input to count its samples worked and drew a wire that carried
    /// nothing, which is a worse lie than no wire at all.
    pub block_seconds: f64,
    /// Specs of each input port.
    pub inputs: &'a [PortSpec],
    /// Tags landing within this call's input window, one slice per input
    /// port, each sorted by index.
    ///
    /// Per port rather than one list, because a node with more than one
    /// input cannot otherwise tell which stream a tag arrived on: with a
    /// single list it could only ever be the first port's, so the audio bus
    /// and the packet bus lost every tag but one strip's.
    in_tags: &'a [&'a [Tag]],
    events: &'a mut Vec<Event>,
    out_tags: &'a mut Vec<Tag>,
}

impl<'a> NodeCtx<'a> {
    pub fn new(
        sample_index: u64,
        inputs: &'a [PortSpec],
        in_tags: &'a [&'a [Tag]],
        events: &'a mut Vec<Event>,
        out_tags: &'a mut Vec<Tag>,
    ) -> Self {
        Self { sample_index, block_seconds: 0.0, inputs, in_tags, events, out_tags }
    }

    /// Tags that arrived on one input port this call.
    pub fn in_tags(&self, port: usize) -> &[Tag] {
        self.in_tags.get(port).copied().unwrap_or(&[])
    }

    /// The same with the run's clock, which only the graph knows.
    pub fn with_block_seconds(mut self, secs: f64) -> Self {
        self.block_seconds = secs;
        self
    }

    /// Report something that is not a sample: a detection, a decoded frame.
    pub fn emit(&mut self, e: Event) {
        self.events.push(e);
    }

    /// Ask something of whatever placed this node; see [`Request`].
    pub fn request(&mut self, r: Request) {
        self.events.push(Event::Request(r));
    }

    /// Report something that went wrong without stopping the chain. Which
    /// node said it is the graph's to say; see [`Event::Warning`].
    pub fn warn(&mut self, message: impl Into<String>) {
        self.events.push(Event::Warning { message: message.into() });
    }

    /// Attach metadata to an absolute output sample index, on every output
    /// port. Tags propagate downstream automatically, rate-scaled at each
    /// node.
    pub fn tag(&mut self, t: Tag) {
        self.out_tags.push(t);
    }

    pub fn timestamp(&self) -> f64 {
        let rate = self.inputs.first().map(|p| p.spec.rate).unwrap_or(1.0);
        self.sample_index as f64 / rate
    }
}

/// A block in the graph.
///
/// Unlike GNU Radio there is no `forecast`/`consume`/`produce` protocol. A node
/// is handed whatever arrived and writes whatever it can; if it needs to
/// accumulate (a framer waiting for a full packet) it buffers internally and
/// emits nothing that call. This removes the single largest class of bugs in
/// GNU Radio block authoring at the cost of each node owning a little state.
pub trait Node: AsAny + Send + 'static {
    fn name(&self) -> &str;

    node_options!();

    fn num_inputs(&self) -> usize {
        1
    }

    fn num_outputs(&self) -> usize {
        1
    }

    /// Validate inputs and declare one spec per output port.
    ///
    /// Also where rate-dependent state is built: a filter designs its taps
    /// here, since only now does it know its input rate. Must be idempotent,
    /// as it is re-run whenever an upstream rate changes.
    fn negotiate(&mut self, inputs: &[PortSpec]) -> Result<Vec<StreamSpec>>;

    /// Group delay this node adds on `port`, in output samples. A symmetric
    /// FIR reports half its tap count.
    fn latency(&self, _port: usize) -> u64 {
        0
    }

    /// Transform inputs into outputs. Output buffers arrive cleared and of the
    /// negotiated variant.
    fn process(
        &mut self,
        inputs: &[&Payload],
        outputs: &mut [Payload],
        ctx: &mut NodeCtx<'_>,
    ) -> Result<()>;
}

/// Convenience for the overwhelmingly common single-in single-out node.
///
/// Implement this and get a `Node` impl for free, without hand-writing slice
/// indexing in every filter. Everything optional is the same set `Node`
/// carries, expanded from one macro so the two cannot drift.
pub trait Simple: Send {
    fn name(&self) -> &str;

    node_options!();

    fn negotiate(&mut self, input: &PortSpec) -> Result<StreamSpec>;

    fn latency(&self) -> u64 {
        0
    }

    fn process(
        &mut self,
        input: &Payload,
        output: &mut Payload,
        ctx: &mut NodeCtx<'_>,
    ) -> Result<()>;
}

impl<T: Simple + 'static> Node for T {
    fn name(&self) -> &str {
        Simple::name(self)
    }

    forward_node_options!();

    fn negotiate(&mut self, inputs: &[PortSpec]) -> Result<Vec<StreamSpec>> {
        let input = inputs.first().ok_or_else(|| {
            common::Error::other(format!("{}: needs one input", Simple::name(self)))
        })?;
        Ok(vec![Simple::negotiate(self, input)?])
    }

    fn latency(&self, _port: usize) -> u64 {
        Simple::latency(self)
    }

    fn process(
        &mut self,
        inputs: &[&Payload],
        outputs: &mut [Payload],
        ctx: &mut NodeCtx<'_>,
    ) -> Result<()> {
        Simple::process(self, inputs[0], &mut outputs[0], ctx)
    }
}

/// Call `f` on `n` and on every node it runs inside itself, however deep.
///
/// What a caller wants when it is asking something of the whole receiver:
/// the graph holds the composites, and this reaches what they hold.
pub fn walk(n: &dyn Node, f: &mut dyn FnMut(&dyn Node)) {
    f(n);
    n.each_inner(&mut |i| walk(i, &mut *f));
}

/// The mutable counterpart of [`walk`].
pub fn walk_mut(n: &mut dyn Node, f: &mut dyn FnMut(&mut dyn Node)) {
    f(n);
    n.each_inner_mut(&mut |i| walk_mut(i, &mut *f));
}
