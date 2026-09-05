//! Transmit stages: text to timings, timings to IQ.
//!
//! The same shape as the receive side read backwards. A keyer turns bytes
//! into a [`PortKind::Pulses`] burst the way a slicer turns a burst into
//! bytes, and a modulator turns that burst into [`PortKind::Iq`] the way a
//! detector turns IQ into a burst. Both are nodes, so a transmission is
//! visible in the chain view, tappable, and parameterised like everything
//! else the receiver does.
//!
//! A chain does not usually wire the two by hand. [`MorseTxNode`] is one node
//! taking bytes and producing IQ, holding both inside and showing them
//! through [`pipeline::node::Node::subgraph`], which is the transmit mirror
//! of the front ends that take IQ and produce packets. The modulator is
//! shared: it keys whatever timings arrive, so every protocol with a timing
//! table adds an encoder and reuses this carrier.

use crate::mod_nodes::OokModNode;
use common::Result;
use pipeline::graph::Topology;
use pipeline::node::{Node, NodeCtx, PortSpec, Simple};
use pipeline::param::{Param, ParamValue};
use pipeline::port::{Domain, Flow, Payload, PortKind, StreamSpec};
use pipeline::Graph;

/// Bytes of text in, Morse timings out.
pub struct MorseKeyNode {
    wpm: f32,
}

impl Default for MorseKeyNode {
    fn default() -> Self {
        Self { wpm: 20.0 }
    }
}

impl MorseKeyNode {
    pub fn new(wpm: f32) -> Self {
        Self { wpm: wpm.clamp(1.0, 60.0) }
    }
}

impl Simple for MorseKeyNode {
    fn name(&self) -> &str {
        "morse_key"
    }

    fn negotiate(&mut self, input: &PortSpec) -> Result<StreamSpec> {
        if input.spec.kind != PortKind::Bytes {
            return Err(common::Error::other("morse_key takes text as bytes"));
        }
        Ok(StreamSpec {
            kind: PortKind::Pulses,
            // Timings are microseconds, so the port has no sample rate of its
            // own; the rate it carries is the one the modulator will key at,
            // passed through so a chain stays rate-consistent.
            rate: input.spec.rate,
            center: input.spec.center,
            bandwidth: input.spec.bandwidth,
            channels: 1,
            flow: Flow::Tx,
            domain: Domain::Baseband,
        })
    }

    fn process(
        &mut self,
        input: &Payload,
        output: &mut Payload,
        _ctx: &mut NodeCtx<'_>,
    ) -> Result<()> {
        let Some(bytes) = input.as_bytes() else { return Ok(()) };
        if bytes.is_empty() {
            return Ok(());
        }
        let text = String::from_utf8_lossy(bytes);
        let pkg = decode::morse::encode(&text, self.wpm);
        if !pkg.pulses.is_empty() {
            output.pulses_mut().push(pkg);
        }
        Ok(())
    }

    fn params(&self) -> Vec<Param> {
        vec![Param::float("wpm", self.wpm as f64, 1.0..=60.0)
            .label("Speed")
            .unit("wpm")]
    }

    fn set_param(&mut self, name: &str, value: ParamValue) -> Result<()> {
        match name {
            "wpm" => {
                self.wpm = value.as_f64().unwrap_or(20.0).clamp(1.0, 60.0) as f32;
                Ok(())
            }
            _ => Err(common::Error::other(format!("morse_key: unknown parameter {name:?}"))),
        }
    }
}

/// Morse as one stage: text in, keyed carrier out.
///
/// The transmit mirror of a front end like [`crate::M17Node`], which takes IQ
/// and produces packets with its stages inside it. What it holds is an
/// ordinary graph of the two stages above, so the timings between them are a
/// real edge that the chain view draws and a tap can read, and the modulator
/// is the same one every other keyed protocol will use.
pub struct MorseTxNode {
    inner: Graph,
}

impl Default for MorseTxNode {
    fn default() -> Self {
        Self::new(20.0, 0.0)
    }
}

impl MorseTxNode {
    pub fn new(wpm: f32, offset_hz: f64) -> Self {
        // A placeholder rate, because a graph negotiates as it is built and
        // the real rate is not known until the graph around this node says
        // what the radio is running at. `negotiate` replaces it.
        let input = StreamSpec {
            kind: PortKind::Bytes,
            rate: 1.0,
            flow: Flow::Tx,
            ..StreamSpec::default()
        };
        let mut b = Graph::builder(input);
        let key = b.add_labeled("morse_key", Box::new(MorseKeyNode::new(wpm)));
        let modu = b.add_labeled(
            "ook_mod",
            Box::new(OokModNode::new(offset_hz, 0.25, 500.0)),
        );
        b.source(key.i());
        b.link(key, modu);
        b.output(modu.o());
        Self { inner: b.build().expect("morse_tx graph is fixed and acyclic") }
    }
}

impl Node for MorseTxNode {
    fn name(&self) -> &str {
        "morse_tx"
    }

    fn subgraph(&self) -> Option<Topology> {
        Some(self.inner.topology())
    }

    fn negotiate(&mut self, inputs: &[PortSpec]) -> Result<Vec<StreamSpec>> {
        let i = &inputs[0];
        if i.spec.kind != PortKind::Bytes {
            return Err(common::Error::other("morse_tx takes text as bytes"));
        }
        if i.spec.rate <= 0.0 {
            return Err(common::Error::other("morse_tx needs the rate it should key at"));
        }
        *self.inner.input_buf() = Payload::Bytes(Vec::new());
        self.inner.set_input_spec(StreamSpec {
            kind: PortKind::Bytes,
            rate: i.spec.rate,
            center: i.spec.center,
            bandwidth: i.spec.bandwidth,
            channels: 1,
            flow: Flow::Tx,
            domain: Domain::Baseband,
        })?;
        Ok(vec![self.inner.output_spec()])
    }

    fn process(
        &mut self,
        inputs: &[&Payload],
        outputs: &mut [Payload],
        ctx: &mut NodeCtx<'_>,
    ) -> Result<()> {
        let Some(bytes) = inputs[0].as_bytes() else { return Ok(()) };
        if bytes.is_empty() {
            return Ok(());
        }
        {
            let buf = self.inner.input_buf();
            buf.clear();
            buf.bytes_mut().extend_from_slice(bytes);
        }
        self.inner.run()?;
        if let Some(iq) = self.inner.output().as_iq() {
            outputs[0].iq_mut().extend_from_slice(iq);
        }
        for t in self.inner.output_tags() {
            ctx.tag(t.clone());
        }
        Ok(())
    }

    fn reset(&mut self) {
        self.inner.reset();
    }

    fn params(&self) -> Vec<Param> {
        let mut out = Vec::new();
        for (id, _) in self.inner.order() {
            if let Some(n) = self.inner.node(id) {
                out.extend(n.params());
            }
        }
        out
    }

    fn set_param(&mut self, name: &str, value: ParamValue) -> Result<()> {
        let ids: Vec<_> = self.inner.order().map(|(id, _)| id).collect();
        for id in ids {
            let Some(n) = self.inner.node_mut(id) else { continue };
            if n.params().iter().any(|p| p.name == name) {
                return n.set_param(name, value);
            }
        }
        Err(common::Error::other(format!("morse_tx: unknown parameter {name:?}")))
    }
}

