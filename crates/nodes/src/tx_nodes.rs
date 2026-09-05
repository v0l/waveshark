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

use common::{Package, Result, C32};
use pipeline::graph::Topology;
use pipeline::node::{Node, NodeCtx, PortSpec, Simple};
use pipeline::param::{Param, ParamValue};
use pipeline::port::{Domain, Flow, Payload, PortKind, StreamSpec, TAG_TX_END, TAG_TX_START};
use pipeline::{Graph, Tag};

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

/// Timings in, keyed carrier out.
///
/// The carrier sits at `offset` from the stream's centre rather than on it,
/// because a transmitter keying its own local oscillator puts the signal
/// under the DC spur of every direct conversion receiver listening, including
/// the one that sent it.
///
/// Edges are raised cosine over `ramp_us`. Switching a carrier on in one
/// sample is a step, and the spectrum of a step is the whole band: measured
/// on a keyed dot at 250 kS/s, a hard edge leaves 74 dB more energy 25 kHz
/// off channel than a 1 ms ramp does.
pub struct OokModNode {
    offset_hz: f64,
    amplitude: f32,
    ramp_us: f32,
    rate: f64,
    phase: f64,
    /// Output samples produced since the last reset, so burst tags land on
    /// absolute indices rather than block-relative ones.
    produced: u64,
}

impl Default for OokModNode {
    fn default() -> Self {
        Self {
            offset_hz: 0.0,
            // A quarter of full scale. The DAC clips at one, and a clipped
            // carrier is spread across the band rather than confined to it.
            amplitude: 0.25,
            ramp_us: 500.0,
            rate: 0.0,
            phase: 0.0,
            produced: 0,
        }
    }
}

impl OokModNode {
    pub fn new(offset_hz: f64, amplitude: f32, ramp_us: f32) -> Self {
        Self {
            offset_hz,
            amplitude: amplitude.clamp(0.0, 1.0),
            ramp_us: ramp_us.max(0.0),
            ..Self::default()
        }
    }

    /// Samples this package will produce at the negotiated rate.
    pub fn sample_count(&self, pkg: &Package) -> usize {
        let per_us = self.rate / 1e6;
        pkg.pulses
            .iter()
            .map(|p| ((p.mark as f64 + p.gap as f64) * per_us).round() as usize)
            .sum()
    }

    /// Key one package into `out`, carrying the carrier phase across calls so
    /// consecutive blocks join without a discontinuity.
    fn key(&mut self, pkg: &Package, out: &mut Vec<C32>) {
        let per_us = self.rate / 1e6;
        let ramp = ((self.ramp_us as f64 * per_us).round() as usize).max(1);
        let step = std::f64::consts::TAU * self.offset_hz / self.rate;

        for p in &pkg.pulses {
            let mark = ((p.mark as f64 * per_us).round() as usize).max(1);
            let gap = (p.gap as f64 * per_us).round() as usize;
            // A ramp longer than half the symbol would never reach full
            // amplitude, so a fast dot shapes over what it has.
            let r = ramp.min(mark / 2);
            for i in 0..mark {
                let env = if r == 0 {
                    1.0
                } else if i < r {
                    raised_cosine(i as f32 / r as f32)
                } else if i >= mark - r {
                    raised_cosine((mark - 1 - i) as f32 / r as f32)
                } else {
                    1.0
                };
                let (s, c) = self.phase.sin_cos();
                out.push(C32::new(c as f32, s as f32) * (env * self.amplitude));
                self.advance(step);
            }
            for _ in 0..gap {
                out.push(C32::new(0.0, 0.0));
                self.advance(step);
            }
        }
    }

    fn advance(&mut self, step: f64) {
        self.phase += step;
        if self.phase > std::f64::consts::TAU {
            self.phase -= std::f64::consts::TAU;
        } else if self.phase < -std::f64::consts::TAU {
            self.phase += std::f64::consts::TAU;
        }
    }
}

/// Rise from 0 to 1 over `x` in [0, 1] with zero slope at both ends.
fn raised_cosine(x: f32) -> f32 {
    0.5 - 0.5 * (std::f32::consts::PI * x.clamp(0.0, 1.0)).cos()
}

impl Simple for OokModNode {
    fn name(&self) -> &str {
        "ook_mod"
    }

    fn negotiate(&mut self, input: &PortSpec) -> Result<StreamSpec> {
        if input.spec.kind != PortKind::Pulses {
            return Err(common::Error::other("ook_mod takes pulse timings"));
        }
        if input.spec.rate <= 0.0 {
            return Err(common::Error::other("ook_mod needs the rate it should key at"));
        }
        self.rate = input.spec.rate;
        Ok(StreamSpec {
            kind: PortKind::Iq,
            rate: self.rate,
            center: input.spec.center,
            // What the keying occupies, not what the stream can carry. A
            // ramped edge is roughly two over the ramp time wide.
            bandwidth: (2e6 / self.ramp_us.max(1.0) as f64).min(self.rate),
            channels: 1,
            flow: Flow::Tx,
            domain: Domain::Baseband,
        })
    }

    fn process(
        &mut self,
        input: &Payload,
        output: &mut Payload,
        ctx: &mut NodeCtx<'_>,
    ) -> Result<()> {
        let Some(pkgs) = input.as_pulses() else { return Ok(()) };
        let out = output.iq_mut();
        for pkg in pkgs {
            let pkg = pkg.clone();
            let start = self.produced + out.len() as u64;
            self.key(&pkg, out);
            let end = self.produced + out.len() as u64;
            // What the radio keys on. Without these the stage that hands
            // samples over has to infer a burst from the samples going quiet,
            // which cannot tell a gap inside a transmission from its end.
            ctx.tag(Tag::marker(start, TAG_TX_START));
            ctx.tag(Tag::marker(end.saturating_sub(1), TAG_TX_END));
        }
        self.produced += out.len() as u64;
        Ok(())
    }

    fn reset(&mut self) {
        self.phase = 0.0;
        self.produced = 0;
    }

    fn params(&self) -> Vec<Param> {
        vec![
            Param::float("offset_hz", self.offset_hz, -1e6..=1e6)
                .label("Carrier offset")
                .unit("Hz"),
            Param::float("amplitude", self.amplitude as f64, 0.0..=1.0)
                .label("Amplitude")
                .unit("FS"),
            Param::float("ramp_us", self.ramp_us as f64, 0.0..=5000.0)
                .label("Edge ramp")
                .unit("us"),
        ]
    }

    fn set_param(&mut self, name: &str, value: ParamValue) -> Result<()> {
        let v = value.as_f64().unwrap_or(0.0);
        match name {
            "offset_hz" => self.offset_hz = v,
            "amplitude" => self.amplitude = v.clamp(0.0, 1.0) as f32,
            "ramp_us" => self.ramp_us = v.max(0.0) as f32,
            _ => return Err(common::Error::other(format!("ook_mod: unknown parameter {name:?}"))),
        }
        Ok(())
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

#[cfg(test)]
mod tests {
    use super::*;
    use common::Pulse;

    fn modulate(pkg: &Package, rate: f64, offset: f64, ramp_us: f32) -> Vec<C32> {
        let mut n = OokModNode::new(offset, 0.5, ramp_us);
        n.rate = rate;
        let mut out = Vec::new();
        n.key(pkg, &mut out);
        out
    }

    #[test]
    fn a_keyed_dot_is_as_long_as_it_was_asked_to_be() {
        let rate = 250_000.0;
        let pkg = Package {
            pulses: vec![Pulse { mark: 60_000, gap: 60_000 }],
            ..Default::default()
        };
        let iq = modulate(&pkg, rate, 10_000.0, 0.0);
        assert_eq!(iq.len(), 30_000, "120 ms at 250 kS/s is 30000 samples");
        let on = iq.iter().filter(|c| c.norm() > 0.25).count();
        // Half the burst is carrier, to within the edge samples.
        assert!((on as i64 - 15_000).abs() < 50, "{on} samples of carrier");
    }

    #[test]
    fn the_carrier_lands_at_the_offset_it_was_given() {
        let rate = 250_000.0;
        let offset = 12_500.0;
        let pkg = Package {
            pulses: vec![Pulse { mark: 40_000, gap: 0 }],
            ..Default::default()
        };
        let iq = modulate(&pkg, rate, offset, 100.0);
        // Average phase advance per sample over the steady part.
        let mid = &iq[2000..8000];
        let mut turns = 0.0f64;
        for w in mid.windows(2) {
            turns += (w[1] * w[0].conj()).arg() as f64;
        }
        let hz = turns / (mid.len() - 1) as f64 / std::f64::consts::TAU * rate;
        assert!((hz - offset).abs() < 5.0, "carrier at {hz:.1} Hz, wanted {offset}");
    }

    #[test]
    fn a_ramped_edge_is_far_quieter_off_channel_than_a_hard_one() {
        // The reason the ramp exists. Both signals key the same dot; the
        // hard-switched one splatters, and this is the measurement that says
        // by how much rather than an assertion that it does.
        let rate = 250_000.0;
        let pkg = Package {
            pulses: vec![Pulse { mark: 20_000, gap: 20_000 }],
            ..Default::default()
        };
        let hard = modulate(&pkg, rate, 0.0, 0.0);
        let soft = modulate(&pkg, rate, 0.0, 1000.0);

        // Power 25 kHz off channel, averaged over the whole burst. The
        // window's own sidelobes are 92 dB down, well under what is measured.
        let far = |iq: &[C32]| -> f32 {
            const N: usize = 4096;
            let mut spec = dsp::spectrum::Spectrum::new(N);
            spec.smoothing = 1.0;
            spec.process(iq);
            let bin = N / 2 + (25_000.0 / rate * N as f64).round() as usize;
            spec.power_db()[bin]
        };
        let (h, s) = (far(&hard), far(&soft));
        println!("hard {h:.1} dB, ramped {s:.1} dB, {:.1} dB bought", h - s);
        assert!(s < h - 20.0, "ramping bought only {:.1} dB off channel", h - s);
    }

    #[test]
    fn phase_is_continuous_across_packages() {
        let rate = 250_000.0;
        let pkg = Package {
            pulses: vec![Pulse { mark: 4_000, gap: 0 }],
            ..Default::default()
        };
        let mut n = OokModNode::new(10_000.0, 0.5, 0.0);
        n.rate = rate;
        let mut a = Vec::new();
        n.key(&pkg, &mut a);
        let mut b = Vec::new();
        n.key(&pkg, &mut b);
        // The step across the join must match the step inside a package: a
        // modulator that restarts its phase clicks at every package edge.
        let inside = (a[1] * a[0].conj()).arg();
        let across = (b[0] * a[a.len() - 1].conj()).arg();
        assert!((inside - across).abs() < 1e-3, "phase jumped {across} against {inside}");
    }
}
