//! The `Node` trait: one block in the signal flow graph.

use crate::event::Event;
use crate::param::{Param, ParamValue};
use crate::port::{Payload, StreamSpec, Tag};
use common::Result;

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
    /// Tags landing within this call's input window, sorted by index.
    pub in_tags: &'a [Tag],
    events: &'a mut Vec<Event>,
    out_tags: &'a mut Vec<Tag>,
}

impl<'a> NodeCtx<'a> {
    pub fn new(
        sample_index: u64,
        inputs: &'a [PortSpec],
        in_tags: &'a [Tag],
        events: &'a mut Vec<Event>,
        out_tags: &'a mut Vec<Tag>,
    ) -> Self {
        Self { sample_index, block_seconds: 0.0, inputs, in_tags, events, out_tags }
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

    /// Attach metadata to an absolute output sample index. Tags propagate
    /// downstream automatically, rate-scaled at each node.
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
pub trait Node: Send + 'static {
    fn name(&self) -> &str;

    /// Downcast hook, so a host can read state a node exposes beyond its
    /// ports: an RDS decoder's station, a PLL's lock. Returns `None` unless a
    /// node opts in, which keeps the trait usable for nodes that have nothing
    /// extra to say.
    fn as_any(&self) -> Option<&dyn std::any::Any> {
        None
    }

    /// Downcast an owned node back to its concrete type.
    ///
    /// Needed when a graph is taken apart and something inside a node has to
    /// come out with it: a recorder's open file, for instance, which must
    /// survive the graph being rebuilt around it. `as_any` cannot do this
    /// because it only ever borrows.
    fn into_any(self: Box<Self>) -> Option<Box<dyn std::any::Any>> {
        None
    }

    /// The mutable counterpart, for changing a running node's own settings.
    ///
    /// Parameters can also be set by name, and that is the right route for
    /// anything generic like a chain editor. This is for the cases where the
    /// caller already knows the type and wants its own API, such as a squelch
    /// control that has to ask what the threshold is measured against before
    /// it can label itself.
    fn as_any_mut(&mut self) -> Option<&mut dyn std::any::Any> {
        None
    }

    /// The graph this node runs inside itself, if it is a composite.
    ///
    /// A node that owns an inner graph is still one node to the scheduler,
    /// which is how a bank keeps hundreds of channels behind a single
    /// channelizer and one batched transform. Without this the work it does
    /// would be invisible: a view of the chain would show a box labelled
    /// "bank" and no sign of the decoder every channel is running.
    fn subgraph(&self) -> Option<crate::graph::Topology> {
        None
    }

    /// How many times [`Node::subgraph`] runs per block, for a view that draws
    /// one branch and says how many there are.
    fn subgraph_count(&self) -> usize {
        1
    }

    /// Whether this node ends the stream rather than passing one on.
    ///
    /// A spectrum display, a recorder and a channel bank all consume samples
    /// and write no buffer. The graph still gives them an output slot, because
    /// every node has one, so without saying so they look like stages with an
    /// output nobody happens to have connected.
    fn is_sink(&self) -> bool {
        false
    }

    fn num_inputs(&self) -> usize {
        1
    }

    /// Whether an input port nothing feeds is acceptable, and read as
    /// silence, rather than a build error.
    ///
    /// A mixer has a spare input by nature: the next thing to be heard is
    /// wired into it, and until then it carries nothing. Every other node's
    /// unfed input is a mistake, and the build refuses it so that a chain
    /// with a wire missing cannot run and produce something that looks like
    /// a result.
    fn optional_inputs(&self) -> bool {
        false
    }

    fn num_outputs(&self) -> usize {
        1
    }

    /// The channel width(s), in hertz, this node expects to receive when it is
    /// a narrowband front end tuned to one channel. Empty (the default) means
    /// the node is not an auto-placeable channel: a filter, a sink, a wideband
    /// decoder, or a front end whose placement is decided by band rather than
    /// width. A non-empty list lets the auto node ask what a front end wants
    /// rather than keep its own table of who fits where, and place it on a
    /// detected source whose width matches one of them. More than one width is
    /// for a mode that is keyed at several channel spacings.
    fn channels(&self) -> &'static [f64] {
        &[]
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

    /// Drop all history: called on retune, stream restart, or channel reuse.
    fn reset(&mut self) {}

    fn params(&self) -> Vec<Param> {
        Vec::new()
    }

    fn set_param(&mut self, name: &str, _value: ParamValue) -> Result<()> {
        Err(common::Error::other(format!("{}: unknown parameter {name:?}", self.name())))
    }
}

/// Convenience for the overwhelmingly common single-in single-out node.
///
/// Implement this and get a `Node` impl for free, without hand-writing slice
/// indexing in every filter.
pub trait Simple: Send {
    fn name(&self) -> &str;
    fn negotiate(&mut self, input: &PortSpec) -> Result<StreamSpec>;
    /// See [`Node::subgraph`]. A composite with one input and one output, such
    /// as a channel bank, is still the common case.
    fn subgraph(&self) -> Option<crate::graph::Topology> {
        None
    }
    fn subgraph_count(&self) -> usize {
        1
    }
    /// See [`Node::is_sink`].
    fn is_sink(&self) -> bool {
        false
    }
    /// See [`Node::channels`].
    fn channels(&self) -> &'static [f64] {
        &[]
    }
    fn latency(&self) -> u64 {
        0
    }
    fn process(
        &mut self,
        input: &Payload,
        output: &mut Payload,
        ctx: &mut NodeCtx<'_>,
    ) -> Result<()>;
    fn reset(&mut self) {}
    fn params(&self) -> Vec<Param> {
        Vec::new()
    }
    fn set_param(&mut self, name: &str, _value: ParamValue) -> Result<()> {
        Err(common::Error::other(format!("{}: unknown parameter {name:?}", self.name())))
    }
}

impl<T: Simple + 'static> Node for T {
    fn name(&self) -> &str {
        Simple::name(self)
    }
    fn negotiate(&mut self, inputs: &[PortSpec]) -> Result<Vec<StreamSpec>> {
        let input = inputs
            .first()
            .ok_or_else(|| common::Error::other(format!("{}: needs one input", Simple::name(self))))?;
        Ok(vec![Simple::negotiate(self, input)?])
    }
    fn latency(&self, _port: usize) -> u64 {
        Simple::latency(self)
    }
    fn channels(&self) -> &'static [f64] {
        Simple::channels(self)
    }
    fn process(
        &mut self,
        inputs: &[&Payload],
        outputs: &mut [Payload],
        ctx: &mut NodeCtx<'_>,
    ) -> Result<()> {
        Simple::process(self, inputs[0], &mut outputs[0], ctx)
    }
    fn reset(&mut self) {
        Simple::reset(self)
    }
    fn params(&self) -> Vec<Param> {
        Simple::params(self)
    }
    fn set_param(&mut self, name: &str, value: ParamValue) -> Result<()> {
        Simple::set_param(self, name, value)
    }
    /// Forwarded so a caller can read a running node's own state.
    ///
    /// Without this, every node written against `Simple` was invisible: the
    /// blanket implementation left the default returning `None`, so a
    /// downcast to the concrete type always failed and the state could only
    /// be reached through tags. That is fine for something that changes
    /// occasionally and wrong for something a display wants on every frame,
    /// like the gain an AGC is currently applying.
    fn as_any(&self) -> Option<&dyn std::any::Any> {
        Some(self)
    }
    fn as_any_mut(&mut self) -> Option<&mut dyn std::any::Any> {
        Some(self)
    }
    fn into_any(self: Box<Self>) -> Option<Box<dyn std::any::Any>> {
        Some(self)
    }
    fn subgraph(&self) -> Option<crate::graph::Topology> {
        Simple::subgraph(self)
    }
    fn subgraph_count(&self) -> usize {
        Simple::subgraph_count(self)
    }
    fn is_sink(&self) -> bool {
        Simple::is_sink(self)
    }
}
