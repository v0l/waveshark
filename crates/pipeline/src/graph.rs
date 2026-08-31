//! The flow graph: a DAG of nodes with typed, rate-negotiated edges.
//!
//! # Execution model
//!
//! Nodes are topologically sorted once at build time and then run strictly in
//! order, on one thread, for each call to [`Graph::run`]. There is no
//! per-node thread and no scheduler.
//!
//! That is the deliberate opposite of GNU Radio, and it is the central design
//! choice in super-radio. GNU Radio gives every block its own thread, which is
//! reasonable when a flowgraph has twenty blocks and unworkable when it has a
//! thousand: at 512 channels times five nodes, thread-per-block means 2560
//! threads fighting over 48 cores, and the context switching would cost more
//! than the DSP. Here, parallelism lives one level up: each channel owns an
//! independent `Graph`, and the channel set is spread across a rayon pool.
//! Hundreds of cheap serial graphs running concurrently beat one graph
//! parallelised internally, because the channels are already embarrassingly
//! parallel and share nothing.
//!
//! # No cycles
//!
//! The graph is acyclic by construction. Feedback loops (PLLs, timing
//! recovery, AGC) belong *inside* a node, where the loop is a few lines of
//! arithmetic, rather than across nodes, where they would demand explicit
//! delay elements and an iterative scheduler for no benefit.

use crate::event::Event;
use crate::node::{Node, NodeCtx, PortSpec};
use crate::port::{Payload, StreamSpec, Tag};
use common::{Error, Result};
use std::collections::HashMap;

#[derive(Clone, Copy, PartialEq, Eq, Hash, Debug, PartialOrd, Ord)]
pub struct NodeId(pub usize);

/// One output of one node: the producer end of an edge.
#[derive(Clone, Copy, PartialEq, Eq, Hash, Debug)]
pub struct Out {
    pub node: NodeId,
    pub port: usize,
}

/// One input of one node: the consumer end of an edge.
#[derive(Clone, Copy, PartialEq, Eq, Hash, Debug)]
pub struct In {
    pub node: NodeId,
    pub port: usize,
}

impl NodeId {
    pub fn out(self, port: usize) -> Out {
        Out { node: self, port }
    }
    pub fn input(self, port: usize) -> In {
        In { node: self, port }
    }
    /// Shorthand for the first output, which is what most nodes have.
    pub fn o(self) -> Out {
        self.out(0)
    }
    /// Shorthand for the first input.
    pub fn i(self) -> In {
        self.input(0)
    }
}

/// A buffer slot: one per node output port, plus one for the graph input.
type Slot = usize;

const INPUT_SLOT: Slot = 0;

struct Entry {
    node: Box<dyn Node>,
    label: String,
    /// Slot feeding each input port.
    in_slots: Vec<Slot>,
    /// Slot each output port writes to.
    out_slots: Vec<Slot>,
}

pub struct GraphBuilder {
    input: StreamSpec,
    nodes: Vec<Box<dyn Node>>,
    labels: Vec<String>,
    /// Consumer end -> producer end. One producer per input port; fan-out is
    /// many inputs referencing the same `Out`.
    edges: HashMap<In, Out>,
    /// External graph input, referenced as a pseudo-producer.
    input_consumers: Vec<In>,
    output: Option<Out>,
}

/// Marker for connecting a node's input to the graph's external input.
pub const GRAPH_INPUT: Out = Out { node: NodeId(usize::MAX), port: 0 };

impl GraphBuilder {
    pub fn new(input: StreamSpec) -> Self {
        Self {
            input,
            nodes: Vec::new(),
            labels: Vec::new(),
            edges: HashMap::new(),
            input_consumers: Vec::new(),
            output: None,
        }
    }

    pub fn add(&mut self, node: Box<dyn Node>) -> NodeId {
        let label = node.name().to_string();
        self.nodes.push(node);
        self.labels.push(label);
        NodeId(self.nodes.len() - 1)
    }

    /// Add a node taken out of a previous graph, keeping its label.
    ///
    /// This is how a graph changes shape without losing what its nodes had
    /// learned. The alternative, building the new graph from fresh nodes,
    /// silently resets every one of them: an RDS decoder forgets the station
    /// it had spent a minute acquiring, AGCs re-converge audibly, and burst
    /// detectors lose their noise floor and spend the next second either deaf
    /// or hallucinating. Since only the wiring is changing, the nodes should
    /// not notice at all.
    pub fn add_existing(&mut self, part: NodePart) -> NodeId {
        let id = self.add(part.node);
        self.labels[id.0] = part.label;
        id
    }

    pub fn add_labeled(&mut self, label: impl Into<String>, node: Box<dyn Node>) -> NodeId {
        let id = self.add(node);
        self.labels[id.0] = label.into();
        id
    }

    /// Connect a producer to a consumer. Connecting an input twice replaces the
    /// previous edge; fan-out is expressed by connecting one `Out` to several
    /// `In`s.
    pub fn connect(&mut self, from: Out, to: In) -> &mut Self {
        // An input port has exactly one producer, so connecting it must clear
        // whichever of the two tables held the previous edge. Missing the
        // second direction here silently keeps a stale connection alive, which
        // shows up much later as a graph that cannot possibly work but builds
        // without complaint.
        self.edges.remove(&to);
        self.input_consumers.retain(|c| *c != to);
        if from == GRAPH_INPUT {
            self.input_consumers.push(to);
        } else {
            self.edges.insert(to, from);
        }
        self
    }

    /// Connect the graph's external input to a node's first input port.
    pub fn source(&mut self, to: In) -> &mut Self {
        self.connect(GRAPH_INPUT, to)
    }

    /// Chain helper: connect `a.o() -> b.i()`.
    pub fn link(&mut self, a: NodeId, b: NodeId) -> &mut Self {
        self.connect(a.o(), b.i())
    }

    /// Designate which output the graph as a whole exposes.
    pub fn output(&mut self, from: Out) -> &mut Self {
        self.output = Some(from);
        self
    }

    pub fn build(self) -> Result<Graph> {
        Graph::assemble(self)
    }
}

/// A node lifted out of a graph, ready to be built into another one.
pub struct NodePart {
    pub label: String,
    pub node: Box<dyn Node>,
}

/// A node as it exists in a built graph.
#[derive(Clone, Debug)]
pub struct TopoNode {
    pub id: NodeId,
    pub label: String,
    /// The node type's own name, which is what a registry would call it.
    pub kind: String,
    pub latency: u64,
    pub inputs: Vec<(usize, StreamSpec)>,
    pub outputs: Vec<(usize, StreamSpec)>,
    /// The graph this node runs inside itself, and how many times per block.
    /// A bank reports the chain one channel runs and the number of channels.
    pub inner: Option<Box<Topology>>,
    pub inner_count: usize,
    /// Whether the node ends the stream rather than passing one on.
    pub sink: bool,
}

/// The built graph's shape, in execution order.
#[derive(Clone, Debug)]
pub struct Topology {
    pub input: StreamSpec,
    pub nodes: Vec<TopoNode>,
    pub output_slot: usize,
}

impl Topology {
    /// Which node writes a slot, for drawing edges.
    pub fn producer(&self, slot: usize) -> Option<&TopoNode> {
        self.nodes.iter().find(|n| n.outputs.iter().any(|(s, _)| *s == slot))
    }
}

pub struct Graph {
    entries: Vec<Entry>,
    /// Execution order, a topological sort of `entries`.
    order: Vec<usize>,
    bufs: Vec<Payload>,
    specs: Vec<StreamSpec>,
    latency: Vec<u64>,
    tags: Vec<Vec<Tag>>,
    /// Cumulative samples emitted on each slot.
    produced: Vec<u64>,
    output_slot: Slot,
    events: Vec<Event>,
    /// Reused per-call scratch so steady state never allocates.
    scratch_out: Vec<Payload>,
    scratch_specs: Vec<PortSpec>,
    scratch_tags: Vec<Tag>,
    scratch_new_tags: Vec<Tag>,
}

impl std::fmt::Debug for Graph {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Graph")
            .field("nodes", &self.entries.iter().map(|e| &e.label).collect::<Vec<_>>())
            .field("order", &self.order)
            .field("output_rate", &self.output_spec().rate)
            .field("output_latency", &self.output_latency())
            .finish()
    }
}

impl Graph {
    pub fn builder(input: StreamSpec) -> GraphBuilder {
        GraphBuilder::new(input)
    }

    fn assemble(b: GraphBuilder) -> Result<Graph> {
        let n = b.nodes.len();

        // Slot 0 is the external input; node k's output port p follows.
        let mut out_slot_base = vec![0usize; n];
        let mut next = 1usize;
        for (k, node) in b.nodes.iter().enumerate() {
            out_slot_base[k] = next;
            next += node.num_outputs().max(1);
        }
        let n_slots = next;

        let mut in_slots: Vec<Vec<Slot>> = Vec::with_capacity(n);
        for (k, node) in b.nodes.iter().enumerate() {
            let mut slots = Vec::with_capacity(node.num_inputs());
            for p in 0..node.num_inputs() {
                let want = In { node: NodeId(k), port: p };
                if b.input_consumers.contains(&want) {
                    slots.push(INPUT_SLOT);
                } else if let Some(src) = b.edges.get(&want) {
                    if src.node.0 >= n {
                        return Err(Error::other(format!(
                            "node {} ({}) input {p} is fed by a non-existent node",
                            k, b.labels[k]
                        )));
                    }
                    slots.push(out_slot_base[src.node.0] + src.port);
                } else {
                    return Err(Error::other(format!(
                        "node {} ({}) input port {p} is not connected",
                        k, b.labels[k]
                    )));
                }
            }
            in_slots.push(slots);
        }

        // Topological sort by Kahn's algorithm. A cycle is a build-time error
        // rather than a runtime hang.
        let mut producer_of: HashMap<Slot, usize> = HashMap::new();
        for (k, base) in out_slot_base.iter().enumerate() {
            for p in 0..b.nodes[k].num_outputs().max(1) {
                producer_of.insert(base + p, k);
            }
        }
        let mut indeg = vec![0usize; n];
        let mut succ: Vec<Vec<usize>> = vec![Vec::new(); n];
        for (k, slots) in in_slots.iter().enumerate() {
            for s in slots {
                if let Some(&p) = producer_of.get(s) {
                    succ[p].push(k);
                    indeg[k] += 1;
                }
            }
        }
        let mut queue: Vec<usize> = (0..n).filter(|&k| indeg[k] == 0).collect();
        queue.sort_unstable();
        let mut order = Vec::with_capacity(n);
        let mut qi = 0;
        while qi < queue.len() {
            let k = queue[qi];
            qi += 1;
            order.push(k);
            for &s in &succ[k] {
                indeg[s] -= 1;
                if indeg[s] == 0 {
                    queue.push(s);
                }
            }
        }
        if order.len() != n {
            let stuck: Vec<&str> = (0..n)
                .filter(|k| indeg[*k] > 0)
                .map(|k| b.labels[k].as_str())
                .collect();
            return Err(Error::other(format!(
                "graph has a cycle involving: {}. Feedback loops belong inside a node.",
                stuck.join(", ")
            )));
        }

        let mut g = Graph {
            entries: Vec::with_capacity(n),
            order,
            bufs: Vec::new(),
            specs: vec![b.input; n_slots],
            latency: vec![0; n_slots],
            tags: vec![Vec::new(); n_slots],
            produced: vec![0; n_slots],
            output_slot: INPUT_SLOT,
            events: Vec::new(),
            scratch_out: Vec::new(),
            scratch_specs: Vec::new(),
            scratch_tags: Vec::new(),
            scratch_new_tags: Vec::new(),
        };

        let mut nodes = b.nodes;
        for (k, node) in nodes.drain(..).enumerate() {
            let outs = node.num_outputs().max(1);
            g.entries.push(Entry {
                node,
                label: b.labels[k].clone(),
                in_slots: in_slots[k].clone(),
                out_slots: (0..outs).map(|p| out_slot_base[k] + p).collect(),
            });
        }

        g.output_slot = match b.output {
            Some(o) => out_slot_base[o.node.0] + o.port,
            // Default to the last node in execution order, which is what a
            // linear chain wants and an explicit `output()` overrides.
            None => g
                .order
                .last()
                .map(|&k| g.entries[k].out_slots[0])
                .unwrap_or(INPUT_SLOT),
        };

        g.specs[INPUT_SLOT] = b.input;
        g.negotiate()?;
        g.bufs = g.specs.iter().map(|s| Payload::empty_of(s.kind)).collect();
        Ok(g)
    }

    /// Propagate specs and latency forward in topological order.
    pub fn negotiate(&mut self) -> Result<()> {
        for oi in 0..self.order.len() {
            let k = self.order[oi];
            let ins: Vec<PortSpec> = self.entries[k]
                .in_slots
                .iter()
                .map(|&s| PortSpec { spec: self.specs[s], latency: self.latency[s] })
                .collect();

            let outs = self.entries[k].node.negotiate(&ins).map_err(|e| {
                Error::other(format!("node {k} ({}) rejected its input: {e}", self.entries[k].label))
            })?;

            let expect = self.entries[k].out_slots.len();
            if outs.len() != expect {
                return Err(Error::other(format!(
                    "node {k} ({}) declared {} output specs but has {expect} output ports",
                    self.entries[k].label,
                    outs.len()
                )));
            }

            // Worst-case delay across inputs, converted to seconds so it can be
            // re-expressed at the output rate.
            let in_delay_s = ins
                .iter()
                .map(|p| if p.spec.rate > 0.0 { p.latency as f64 / p.spec.rate } else { 0.0 })
                .fold(0.0f64, f64::max);

            for (p, spec) in outs.into_iter().enumerate() {
                let slot = self.entries[k].out_slots[p];
                self.specs[slot] = spec;
                let own = self.entries[k].node.latency(p);
                self.latency[slot] = (in_delay_s * spec.rate).round() as u64 + own;
                if self.bufs.len() > slot && self.bufs[slot].kind() != spec.kind {
                    self.bufs[slot] = Payload::empty_of(spec.kind);
                }
            }
        }
        Ok(())
    }

    pub fn input_spec(&self) -> StreamSpec {
        self.specs[INPUT_SLOT]
    }

    pub fn output_spec(&self) -> StreamSpec {
        self.specs[self.output_slot]
    }

    /// End-to-end group delay in output samples. Report this to the user: it is
    /// the difference between "the packet arrived at 12:00:00.000" and the
    /// truth.
    pub fn output_latency(&self) -> u64 {
        self.latency[self.output_slot]
    }

    pub fn len(&self) -> usize {
        self.entries.len()
    }

    pub fn is_empty(&self) -> bool {
        self.entries.is_empty()
    }

    pub fn node(&self, id: NodeId) -> Option<&dyn Node> {
        self.entries.get(id.0).map(|e| &*e.node)
    }

    pub fn node_mut(&mut self, id: NodeId) -> Option<&mut (dyn Node + 'static)> {
        self.entries.get_mut(id.0).map(|e| &mut *e.node)
    }

    pub fn label(&self, id: NodeId) -> Option<&str> {
        self.entries.get(id.0).map(|e| e.label.as_str())
    }

    /// Execution order, for display and debugging.
    pub fn order(&self) -> impl Iterator<Item = (NodeId, &str)> {
        self.order.iter().map(|&k| (NodeId(k), self.entries[k].label.as_str()))
    }

    /// Structure and negotiated rates, for drawing the graph.
    ///
    /// Reported from the built graph rather than from whatever assembled it, so
    /// a view of the chain shows the chain that is running.
    pub fn topology(&self) -> Topology {
        let mut nodes = Vec::new();
        for &k in &self.order {
            let e = &self.entries[k];
            nodes.push(TopoNode {
                id: NodeId(k),
                label: e.label.clone(),
                kind: e.node.name().to_string(),
                latency: self.latency[e.out_slots[0]],
                inputs: e
                    .in_slots
                    .iter()
                    .map(|&s| (s, self.specs[s]))
                    .collect(),
                outputs: e.out_slots.iter().map(|&s| (s, self.specs[s])).collect(),
                inner: e.node.subgraph().map(Box::new),
                inner_count: e.node.subgraph_count(),
                sink: e.node.is_sink(),
            });
        }
        Topology { input: self.specs[INPUT_SLOT], nodes, output_slot: self.output_slot }
    }

    /// Take the nodes back out, to be rebuilt into a different shape.
    ///
    /// Returned in `NodeId` order, so a caller that remembers which id was
    /// which can pick out the ones it still wants and let the rest drop.
    pub fn into_parts(self) -> Vec<NodePart> {
        self.entries
            .into_iter()
            .map(|e| NodePart { label: e.label, node: e.node })
            .collect()
    }

    pub fn reset(&mut self) {
        for e in &mut self.entries {
            e.node.reset();
        }
        for b in &mut self.bufs {
            b.clear();
        }
        for t in &mut self.tags {
            t.clear();
        }
        self.produced.iter_mut().for_each(|p| *p = 0);
    }

    /// Buffer the caller fills with graph input. Clear before filling.
    pub fn input_buf(&mut self) -> &mut Payload {
        &mut self.bufs[INPUT_SLOT]
    }

    /// Attach a tag to the graph input at an absolute input-sample index.
    pub fn tag_input(&mut self, t: Tag) {
        self.tags[INPUT_SLOT].push(t);
    }

    pub fn output(&self) -> &Payload {
        &self.bufs[self.output_slot]
    }

    /// What a particular node port produced this block.
    ///
    /// A graph carrying the whole receiver has more than one place output
    /// leaves it: every listening channel ends in audio of its own, and
    /// nominating one of them as *the* output would be arbitrary. Decoders
    /// need none of this, since what they produce arrives as events.
    pub fn buf(&self, from: Out) -> Option<&Payload> {
        let e = self.entries.get(from.node.0)?;
        e.out_slots.get(from.port).map(|&s| &self.bufs[s])
    }

    /// Negotiated spec at a particular node port.
    pub fn spec_of(&self, from: Out) -> Option<StreamSpec> {
        let e = self.entries.get(from.node.0)?;
        e.out_slots.get(from.port).map(|&s| self.specs[s])
    }

    /// Group delay at a particular node port, in samples at that port's rate.
    pub fn latency_of(&self, from: Out) -> u64 {
        self.entries
            .get(from.node.0)
            .and_then(|e| e.out_slots.get(from.port))
            .map(|&s| self.latency[s])
            .unwrap_or(0)
    }

    /// Run every node once over the current input buffer.
    pub fn run(&mut self) -> Result<&[Event]> {
        self.events.clear();
        let n_in = self.bufs[INPUT_SLOT].len() as u64;

        for oi in 0..self.order.len() {
            let k = self.order[oi];

            // Destructure so the node, its output buffers, and its input
            // buffers can be borrowed simultaneously without cloning.
            let Graph {
                entries,
                bufs,
                specs,
                latency,
                tags,
                produced,
                events,
                scratch_out,
                scratch_specs,
                scratch_tags,
                scratch_new_tags,
                ..
            } = self;
            let e = &mut entries[k];

            scratch_specs.clear();
            scratch_specs.extend(
                e.in_slots.iter().map(|&s| PortSpec { spec: specs[s], latency: latency[s] }),
            );

            // Tags arriving on the primary input port for this call's window.
            scratch_tags.clear();
            if let Some(&s0) = e.in_slots.first() {
                scratch_tags.extend_from_slice(&tags[s0]);
            }
            let base_index = e.in_slots.first().map(|&s| produced[s]).unwrap_or(0);
            let in_rate = scratch_specs.first().map(|p| p.spec.rate).unwrap_or(1.0);

            // Move this node's output buffers out of the arena. Outputs belong
            // to this node and inputs belong to others, so after this the
            // remaining arena can be borrowed immutably without conflict.
            scratch_out.clear();
            for &s in &e.out_slots {
                let mut b = std::mem::replace(&mut bufs[s], Payload::Bytes(Vec::new()));
                b.clear();
                scratch_out.push(b);
            }

            scratch_new_tags.clear();
            let res = {
                let ins: Vec<&Payload> = e.in_slots.iter().map(|&s| &bufs[s]).collect();
                let mut ctx = NodeCtx::new(
                    base_index,
                    scratch_specs,
                    scratch_tags,
                    events,
                    scratch_new_tags,
                );
                e.node.process(&ins, scratch_out, &mut ctx)
            };

            // Put the buffers back before propagating any error, so a failing
            // node does not leave the arena holding placeholder payloads.
            for (p, &s) in e.out_slots.iter().enumerate() {
                bufs[s] = std::mem::replace(&mut scratch_out[p], Payload::Bytes(Vec::new()));
            }

            res.map_err(|err| Error::other(format!("node {k} ({}): {err}", e.label)))?;

            for (p, &s) in e.out_slots.iter().enumerate() {
                debug_assert_eq!(
                    bufs[s].kind(),
                    specs[s].kind,
                    "node {} wrote the wrong payload kind on port {p}",
                    e.label
                );
                // Rate-scale inbound tags onto this output, then append the
                // node's own.
                let out_rate = specs[s].rate;
                tags[s].clear();
                for t in scratch_tags.iter() {
                    tags[s].push(t.rescale(in_rate, out_rate));
                }
                tags[s].extend(scratch_new_tags.iter().cloned());
                tags[s].sort_by_key(|t| t.index);
                produced[s] += bufs[s].len() as u64;
            }
        }

        self.tags[INPUT_SLOT].clear();
        self.produced[INPUT_SLOT] += n_in;
        Ok(&self.events)
    }

    /// Fill the input buffer from IQ and run.
    pub fn feed_iq(&mut self, samples: &[common::C32]) -> Result<&[Event]> {
        let b = self.bufs[INPUT_SLOT].iq_mut();
        b.clear();
        b.extend_from_slice(samples);
        self.run()
    }

    /// Render the graph as Graphviz DOT, for debugging a chain that will not
    /// negotiate.
    pub fn to_dot(&self) -> String {
        let mut s = String::from("digraph flow {\n  rankdir=LR;\n  node [shape=box];\n");
        s.push_str("  input [shape=ellipse];\n");
        for (k, e) in self.entries.iter().enumerate() {
            let spec = self.specs[e.out_slots[0]];
            s.push_str(&format!(
                "  n{k} [label=\"{}\\n{:.0} S/s {:?}\"];\n",
                e.label, spec.rate, spec.kind
            ));
        }
        for (k, e) in self.entries.iter().enumerate() {
            for &slot in &e.in_slots {
                if slot == INPUT_SLOT {
                    s.push_str(&format!("  input -> n{k};\n"));
                } else if let Some(src) = self.entries.iter().position(|x| x.out_slots.contains(&slot))
                {
                    s.push_str(&format!("  n{src} -> n{k};\n"));
                }
            }
        }
        s.push_str("}\n");
        s
    }
}

/// Build a linear graph from an ordered list of nodes: the common per-channel
/// case, without the connection boilerplate.
pub fn chain(input: StreamSpec, nodes: Vec<Box<dyn Node>>) -> Result<Graph> {
    let mut b = GraphBuilder::new(input);
    let mut prev: Option<NodeId> = None;
    for n in nodes {
        let id = b.add(n);
        match prev {
            None => {
                b.source(id.i());
            }
            Some(p) => {
                b.link(p, id);
            }
        }
        prev = Some(id);
    }
    if let Some(p) = prev {
        b.output(p.o());
    }
    b.build()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::node::Simple;
    use crate::port::{PortKind, TagValue};
    use common::{Hz, C32};

    /// Decimate by 2 and add a declared group delay, so latency bookkeeping
    /// has something to accumulate.
    struct Halve {
        delay: u64,
    }
    impl Simple for Halve {
        fn name(&self) -> &str {
            "halve"
        }
        fn negotiate(&mut self, i: &PortSpec) -> Result<StreamSpec> {
            if i.spec.kind != PortKind::Iq {
                return Err(Error::other("needs IQ"));
            }
            Ok(i.spec.with_rate(i.spec.rate / 2.0))
        }
        fn latency(&self) -> u64 {
            self.delay
        }
        fn process(&mut self, i: &Payload, o: &mut Payload, _c: &mut NodeCtx<'_>) -> Result<()> {
            let d = o.iq_mut();
            d.extend(i.as_iq().unwrap().iter().step_by(2));
            Ok(())
        }
    }

    struct Gain(f32);
    impl Simple for Gain {
        fn name(&self) -> &str {
            "gain"
        }
        fn negotiate(&mut self, i: &PortSpec) -> Result<StreamSpec> {
            Ok(i.spec)
        }
        fn process(&mut self, i: &Payload, o: &mut Payload, _c: &mut NodeCtx<'_>) -> Result<()> {
            let g = self.0;
            o.iq_mut().extend(i.as_iq().unwrap().iter().map(|c| c * g));
            Ok(())
        }
    }

    struct Mag;
    impl Simple for Mag {
        fn name(&self) -> &str {
            "mag"
        }
        fn negotiate(&mut self, i: &PortSpec) -> Result<StreamSpec> {
            if i.spec.kind != PortKind::Iq {
                return Err(Error::other("needs IQ"));
            }
            Ok(i.spec.with_kind(PortKind::Real))
        }
        fn process(&mut self, i: &Payload, o: &mut Payload, _c: &mut NodeCtx<'_>) -> Result<()> {
            o.real_mut().extend(i.as_iq().unwrap().iter().map(|c| c.norm()));
            Ok(())
        }
    }

    /// Two-input sum. Publishes the latency skew it was told about at build
    /// time, which is the thing fan-in has to get right.
    #[derive(Default)]
    struct Sum {
        skew: std::sync::Arc<std::sync::atomic::AtomicI64>,
    }
    impl Node for Sum {
        fn name(&self) -> &str {
            "sum"
        }
        fn num_inputs(&self) -> usize {
            2
        }
        fn negotiate(&mut self, ins: &[PortSpec]) -> Result<Vec<StreamSpec>> {
            if ins[0].spec.rate != ins[1].spec.rate {
                return Err(Error::other("sum inputs must share a rate"));
            }
            self.skew.store(
                ins[0].latency as i64 - ins[1].latency as i64,
                std::sync::atomic::Ordering::Relaxed,
            );
            Ok(vec![ins[0].spec])
        }
        fn process(
            &mut self,
            ins: &[&Payload],
            outs: &mut [Payload],
            _c: &mut NodeCtx<'_>,
        ) -> Result<()> {
            let a = ins[0].as_iq().unwrap();
            let b = ins[1].as_iq().unwrap();
            let o = outs[0].iq_mut();
            o.extend(a.iter().zip(b).map(|(x, y)| x + y));
            Ok(())
        }
    }

    /// Emits a tag on its first output sample of every call.
    struct Tagger;
    impl Simple for Tagger {
        fn name(&self) -> &str {
            "tagger"
        }
        fn negotiate(&mut self, i: &PortSpec) -> Result<StreamSpec> {
            Ok(i.spec)
        }
        fn process(&mut self, i: &Payload, o: &mut Payload, c: &mut NodeCtx<'_>) -> Result<()> {
            c.tag(Tag::new(c.sample_index, "burst", TagValue::Int(7)));
            o.iq_mut().extend_from_slice(i.as_iq().unwrap());
            Ok(())
        }
    }

    /// Records the tags it saw, to prove propagation and rate scaling.
    struct TagSpy {
        seen: std::sync::Arc<std::sync::Mutex<Vec<Tag>>>,
    }
    impl Simple for TagSpy {
        fn name(&self) -> &str {
            "tagspy"
        }
        fn negotiate(&mut self, i: &PortSpec) -> Result<StreamSpec> {
            Ok(i.spec)
        }
        fn process(&mut self, i: &Payload, o: &mut Payload, c: &mut NodeCtx<'_>) -> Result<()> {
            self.seen.lock().unwrap().extend(c.in_tags.iter().cloned());
            o.iq_mut().extend_from_slice(i.as_iq().unwrap());
            Ok(())
        }
    }

    fn spec() -> StreamSpec {
        StreamSpec::iq(2_048_000.0, Hz::mhz(100))
    }

    fn ramp(n: usize) -> Vec<C32> {
        (0..n).map(|i| C32::new(i as f32, 0.0)).collect()
    }

    #[test]
    fn linear_chain_propagates_rate_and_kind() {
        let g = chain(
            spec(),
            vec![Box::new(Halve { delay: 0 }), Box::new(Halve { delay: 0 }), Box::new(Mag)],
        )
        .unwrap();
        assert_eq!(g.output_spec().rate, 512_000.0);
        assert_eq!(g.output_spec().kind, PortKind::Real);
    }

    #[test]
    fn misordered_chain_fails_at_build() {
        let err = chain(spec(), vec![Box::new(Mag), Box::new(Halve { delay: 0 })])
            .unwrap_err()
            .to_string();
        assert!(err.contains("halve"), "unhelpful: {err}");
    }

    #[test]
    fn unconnected_input_is_a_build_error() {
        let mut b = Graph::builder(spec());
        let s = b.add(Box::<Sum>::default());
        b.source(s.input(0));
        // input port 1 deliberately left dangling
        let err = b.build().unwrap_err().to_string();
        assert!(err.contains("input port 1 is not connected"), "got: {err}");
    }

    #[test]
    fn cycles_are_rejected_with_a_useful_message() {
        let mut b = Graph::builder(spec());
        let a = b.add_labeled("alpha", Box::new(Gain(1.0)));
        let c = b.add_labeled("beta", Box::new(Gain(1.0)));
        b.source(a.i());
        b.connect(a.o(), c.i());
        b.connect(c.o(), a.i()); // replaces the source edge, closing the loop
        let err = b.build().unwrap_err().to_string();
        assert!(err.contains("cycle"), "got: {err}");
        assert!(err.contains("alpha") && err.contains("beta"), "got: {err}");
    }

    #[test]
    fn fanout_feeds_two_branches_from_one_producer() {
        let mut b = Graph::builder(spec());
        let src = b.add(Box::new(Gain(2.0)));
        let left = b.add_labeled("left", Box::new(Gain(10.0)));
        let right = b.add_labeled("right", Box::new(Mag));
        b.source(src.i());
        b.connect(src.o(), left.i());
        b.connect(src.o(), right.i());
        b.output(left.o());
        let mut g = b.build().unwrap();

        g.feed_iq(&ramp(4)).unwrap();
        // Both branches ran; the designated output is the left one.
        assert_eq!(
            g.output().as_iq().unwrap(),
            &[C32::new(0.0, 0.0), C32::new(20.0, 0.0), C32::new(40.0, 0.0), C32::new(60.0, 0.0)]
        );
    }

    #[test]
    fn fanin_is_told_the_latency_skew() {
        // Left path: two halvers with 100 samples of delay each.
        // Right path: two halvers with none. Same rate, different delay.
        let mut b = Graph::builder(spec());
        let l1 = b.add(Box::new(Halve { delay: 100 }));
        let l2 = b.add(Box::new(Halve { delay: 100 }));
        let r1 = b.add(Box::new(Halve { delay: 0 }));
        let r2 = b.add(Box::new(Halve { delay: 0 }));
        let skew = std::sync::Arc::new(std::sync::atomic::AtomicI64::new(0));
        let sum = b.add(Box::new(Sum { skew: skew.clone() }));
        b.source(l1.i());
        b.source(r1.i());
        b.link(l1, l2);
        b.link(r1, r2);
        b.connect(l2.o(), sum.input(0));
        b.connect(r2.o(), sum.input(1));
        b.output(sum.o());
        let g = b.build().unwrap();

        // l1 adds 100 at rate/2. Rescaled to rate/4 that is 50, plus l2's own
        // 100, so the left port arrives 150 samples late; the right, 0.
        assert_eq!(skew.load(std::sync::atomic::Ordering::Relaxed), 150);
        assert_eq!(g.output_latency(), 150);
        let _ = sum;
    }

    #[test]
    fn tags_propagate_and_rescale_across_a_rate_change() {
        let seen = std::sync::Arc::new(std::sync::Mutex::new(Vec::new()));
        let g = chain(
            spec(),
            vec![
                Box::new(Tagger),
                Box::new(Halve { delay: 0 }),
                Box::new(TagSpy { seen: seen.clone() }),
            ],
        );
        let mut g = g.unwrap();
        g.feed_iq(&ramp(8)).unwrap();
        g.feed_iq(&ramp(8)).unwrap();

        let s = seen.lock().unwrap();
        assert_eq!(s.len(), 2, "tags: {s:?}");
        // First call tagged input sample 0; second tagged input sample 8,
        // which at half the rate is output sample 4.
        assert_eq!(s[0].index, 0);
        assert_eq!(s[1].index, 4);
        assert_eq!(s[1].value, TagValue::Int(7));
    }

    #[test]
    fn output_latency_accumulates_through_rate_changes() {
        let g = chain(
            spec(),
            vec![Box::new(Halve { delay: 64 }), Box::new(Halve { delay: 32 })],
        )
        .unwrap();
        // 64 at rate/2 becomes 32 at rate/4, plus the second node's 32.
        assert_eq!(g.output_latency(), 64);
    }

    #[test]
    fn dot_output_names_every_node() {
        let g = chain(spec(), vec![Box::new(Gain(1.0)), Box::new(Mag)]).unwrap();
        let dot = g.to_dot();
        assert!(dot.contains("gain") && dot.contains("mag"), "{dot}");
        assert!(dot.contains("input ->"));
    }

    /// A node with history, standing in for anything that has to learn:
    /// an RDS decoder holding a station, an AGC that has converged.
    struct Counting(u64);
    impl Simple for Counting {
        fn name(&self) -> &str {
            "counting"
        }
        fn negotiate(&mut self, i: &PortSpec) -> Result<StreamSpec> {
            Ok(i.spec)
        }
        fn process(&mut self, i: &Payload, o: &mut Payload, _c: &mut NodeCtx<'_>) -> Result<()> {
            self.0 += i.len() as u64;
            o.iq_mut().extend_from_slice(i.as_iq().unwrap());
            Ok(())
        }
    }

    #[test]
    fn rebuilding_a_graph_out_of_its_own_nodes_keeps_what_they_had_learned() {
        let mut g = chain(spec(), vec![Box::new(Counting(0)), Box::new(Gain(2.0))]).unwrap();
        g.feed_iq(&ramp(8)).unwrap();

        // The shape changes, the nodes do not.
        let mut parts = g.into_parts();
        let gain = parts.pop().expect("gain");
        let counter = parts.pop().expect("counter");
        let mut b = GraphBuilder::new(spec());
        let a = b.add_existing(gain);
        let c = b.add_existing(counter);
        b.source(a.i());
        b.link(a, c);
        b.output(c.o());
        let mut g = b.build().unwrap();

        assert_eq!(g.label(c), Some("counting"), "a rebuilt node keeps its label");
        g.feed_iq(&ramp(4)).unwrap();
        let n = g
            .node(c)
            .and_then(|n| n.as_any())
            .and_then(|a| a.downcast_ref::<Counting>())
            .map(|c| c.0);
        assert_eq!(n, Some(12), "the node kept counting rather than starting over");
    }

    #[test]
    fn empty_graph_passes_input_through() {
        let mut g = chain(spec(), vec![]).unwrap();
        g.feed_iq(&ramp(4)).unwrap();
        assert_eq!(g.output().as_iq().unwrap().len(), 4);
    }
}
