//! Sinks: nodes that consume a stream and produce no stream.
//!
//! A spectrum display and a recorder are not stages in a chain, they are
//! places a stream ends. That makes them look like a poor fit for a graph,
//! and they are not: what they consume is IQ from the same source as
//! everything else, they need the same negotiated rate and centre frequency
//! to interpret it, and if they sit outside the graph then the graph is no
//! longer a description of what the receiver is doing.
//!
//! What they produce is read back by downcasting rather than through events.
//! An event per block carrying a spectrum frame would clone a few thousand
//! floats for a display that only ever draws the most recent one, and the
//! recorder's ring is far larger than that. Events are for things that
//! happened; these are things that are.

use common::{Error, Result, C32};
use dsp::Spectrum;
use pipeline::node::{NodeCtx, PortSpec, Simple};
use pipeline::param::{Param, ParamValue};
use pipeline::port::{Payload, PortKind, StreamSpec};

/// The FFT behind the spectrum and the waterfall.
pub struct SpectrumNode {
    spec: Spectrum,
    rate: f64,
    center: common::Hz,
    /// Whether the last block completed a frame, so the host knows when there
    /// is something new to draw rather than redrawing the same bins.
    fresh: bool,
    /// Frames per second worth producing.
    refresh_hz: f32,
    /// Samples still to be discarded before starting the next frame.
    ///
    /// Without this the FFT runs on every sample the radio delivers, which at
    /// 2.4 MS/s and a 4096-point transform is around 1200 frames a second for
    /// a display that shows thirty. The samples in between are not signal
    /// being missed: a spectrum frame is a snapshot, and the ones nobody sees
    /// cost a core to compute.
    debt: f64,
    /// True while a frame is part-collected, which must be finished before
    /// the gate applies again.
    collecting: bool,
}

impl SpectrumNode {
    pub fn new(size: usize) -> Self {
        Self {
            spec: Spectrum::new(size),
            rate: 0.0,
            center: common::Hz(0),
            fresh: false,
            refresh_hz: 30.0,
            debt: 0.0,
            collecting: true,
        }
    }

    /// How often a frame is worth producing.
    pub fn set_refresh(&mut self, hz: f32) {
        self.refresh_hz = hz.clamp(1.0, 240.0);
    }

    pub fn refresh(&self) -> f32 {
        self.refresh_hz
    }

    pub fn size(&self) -> usize {
        self.spec.size()
    }

    /// Whether a new frame completed in the last block.
    pub fn is_fresh(&self) -> bool {
        self.fresh
    }

    /// The averaged power spectrum, in dBFS.
    pub fn power_db(&mut self) -> &[f32] {
        self.spec.power_db()
    }

    pub fn rate(&self) -> f64 {
        self.rate
    }

    pub fn center(&self) -> common::Hz {
        self.center
    }

    pub fn set_smoothing(&mut self, v: f32) {
        self.spec.smoothing = v.clamp(0.0, 1.0);
    }
}

impl Simple for SpectrumNode {
    fn name(&self) -> &str {
        "spectrum"
    }

    fn is_sink(&self) -> bool {
        true
    }

    fn negotiate(&mut self, i: &PortSpec) -> Result<StreamSpec> {
        if i.spec.kind != PortKind::Iq {
            return Err(Error::other("spectrum needs IQ"));
        }
        self.rate = i.spec.rate;
        self.center = i.spec.center;
        // A sink still declares an output spec, because the graph gives every
        // node a slot. Nothing is written to it.
        Ok(i.spec)
    }

    fn process(&mut self, i: &Payload, _o: &mut Payload, _c: &mut NodeCtx<'_>) -> Result<()> {
        let iq = i.as_iq().unwrap_or(&[]);
        self.fresh = false;
        if !self.collecting {
            self.debt -= iq.len() as f64;
            if self.debt > 0.0 {
                return Ok(());
            }
            self.collecting = true;
        }
        if self.spec.process(iq) {
            self.fresh = true;
            self.collecting = false;
            self.debt = self.rate / self.refresh_hz.max(1.0) as f64;
        }
        Ok(())
    }

    fn reset(&mut self) {
        self.spec.reset();
        self.fresh = false;
        self.collecting = true;
        self.debt = 0.0;
    }

    fn params(&self) -> Vec<Param> {
        vec![
            Param::float("smoothing", self.spec.smoothing as f64, 0.0..=0.99)
                .label("Frame to frame averaging"),
            Param::float("refresh", self.refresh_hz as f64, 1.0..=120.0)
                .unit("Hz")
                .label("Frames a second"),
        ]
    }

    fn set_param(&mut self, name: &str, v: ParamValue) -> Result<()> {
        match name {
            "smoothing" => {
                self.set_smoothing(v.as_f64().unwrap_or_default() as f32);
                Ok(())
            }
            "refresh" => {
                self.set_refresh(v.as_f64().unwrap_or(30.0) as f32);
                Ok(())
            }
            _ => Err(Error::other(format!("spectrum: unknown parameter {name:?}"))),
        }
    }
}

/// A tap that keeps the recent past, so a transmission can be saved after the
/// fact.
///
/// The node owns the ring and nothing else does. What to keep is a decision
/// made from decoded events, which arrive after this node has already run, so
/// the host makes that call between blocks through [`RingNode::ring_mut`]
/// rather than the node trying to read the future.
pub struct RingNode<T: Ring> {
    ring: T,
    rate: f64,
}

/// Whatever the host wants filled with raw samples.
///
/// A trait rather than a concrete recorder so `nodes` does not have to know
/// how a file is written, which is an application's business.
pub trait Ring: Send + 'static {
    fn push(&mut self, iq: &[C32]);
    fn reset(&mut self) {}
}

impl<T: Ring> RingNode<T> {
    pub fn new(ring: T) -> Self {
        Self { ring, rate: 0.0 }
    }

    pub fn ring(&self) -> &T {
        &self.ring
    }

    pub fn ring_mut(&mut self) -> &mut T {
        &mut self.ring
    }

    pub fn into_ring(self) -> T {
        self.ring
    }

    pub fn rate(&self) -> f64 {
        self.rate
    }
}

impl<T: Ring> Simple for RingNode<T> {
    fn name(&self) -> &str {
        "record"
    }

    fn is_sink(&self) -> bool {
        true
    }

    fn negotiate(&mut self, i: &PortSpec) -> Result<StreamSpec> {
        if i.spec.kind != PortKind::Iq {
            return Err(Error::other("record needs IQ"));
        }
        self.rate = i.spec.rate;
        Ok(i.spec)
    }

    fn process(&mut self, i: &Payload, _o: &mut Payload, _c: &mut NodeCtx<'_>) -> Result<()> {
        self.ring.push(i.as_iq().unwrap_or(&[]));
        Ok(())
    }

    fn reset(&mut self) {
        self.ring.reset();
    }
}

/// The DC notch at the head of the chain.
///
/// A direct-conversion front end puts local oscillator leakage and the ADC's
/// offset at exactly the tuned frequency, which reads as a very strong carrier
/// that is not there. It is removed here, before anything else sees it, so the
/// spectrum, the detectors and the audio all agree about what arrived.
pub struct DcBlockNode {
    dc: Option<dsp::DcBlock>,
    rate: f64,
    enabled: bool,
}

impl Default for DcBlockNode {
    fn default() -> Self {
        Self::new()
    }
}

impl DcBlockNode {
    pub fn new() -> Self {
        Self { dc: None, rate: 0.0, enabled: true }
    }

    pub fn set_enabled(&mut self, on: bool) {
        if on != self.enabled {
            self.enabled = on;
            // The estimate belongs to a particular tuning and gain setting;
            // switching back on later should measure again rather than
            // subtract whatever was true minutes ago.
            self.dc = None;
        }
    }

    pub fn is_enabled(&self) -> bool {
        self.enabled
    }

    /// Forget the measured offset. Every retune and gain change moves it, and
    /// a stale estimate shows up as a spur that was not there a moment ago.
    pub fn remeasure(&mut self) {
        self.dc = None;
    }

    pub fn offset(&self) -> C32 {
        self.dc.as_ref().map(|d| d.offset()).unwrap_or_default()
    }
}

impl Simple for DcBlockNode {
    fn name(&self) -> &str {
        "dc_block"
    }

    fn negotiate(&mut self, i: &PortSpec) -> Result<StreamSpec> {
        if i.spec.kind != PortKind::Iq {
            return Err(Error::other("dc block needs IQ"));
        }
        if self.rate != i.spec.rate {
            self.rate = i.spec.rate;
            // The notch width is set from the rate.
            self.dc = None;
        }
        Ok(i.spec)
    }

    fn process(&mut self, i: &Payload, o: &mut Payload, _c: &mut NodeCtx<'_>) -> Result<()> {
        let iq = i.as_iq().unwrap_or(&[]);
        let out = o.iq_mut();
        out.extend_from_slice(iq);
        if !self.enabled {
            return Ok(());
        }
        let rate = self.rate;
        let dc = self.dc.get_or_insert_with(|| {
            let mut d = dsp::DcBlock::new(rate);
            // Primed from the first block it sees, so the notch does not have
            // to settle while the display is already showing the result.
            d.prime(iq);
            d
        });
        dc.process(out);
        Ok(())
    }

    fn reset(&mut self) {
        self.dc = None;
    }

    fn params(&self) -> Vec<Param> {
        vec![Param::bool("enabled", self.enabled).label("Remove the centre spur")]
    }

    fn set_param(&mut self, name: &str, v: ParamValue) -> Result<()> {
        match name {
            "enabled" => {
                self.set_enabled(v.as_bool().unwrap_or(true));
                Ok(())
            }
            _ => Err(Error::other(format!("dc_block: unknown parameter {name:?}"))),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use common::Hz;

    fn spec(rate: f64) -> PortSpec {
        PortSpec { spec: StreamSpec::iq(rate, Hz(433_920_000)), latency: 0 }
    }

    fn tone(n: usize) -> Vec<C32> {
        (0..n)
            .map(|k| {
                let p = std::f32::consts::TAU * 0.1 * k as f32;
                C32::new(p.cos(), p.sin())
            })
            .collect()
    }

    #[test]
    fn the_spectrum_reports_a_frame_only_when_one_completed() {
        let mut s = SpectrumNode::new(256);
        Simple::negotiate(&mut s, &spec(2_400_000.0)).unwrap();
        let mut out = Payload::Iq(Vec::new());
        let mut events = Vec::new();
        let mut tags = Vec::new();
        let ins = [spec(2_400_000.0)];

        let mut run = |n: usize, s: &mut SpectrumNode| {
            let mut new_tags = Vec::new();
            let mut ctx = NodeCtx::new(0, &ins, &tags, &mut events, &mut new_tags);
            Simple::process(s, &Payload::Iq(tone(n)), &mut out, &mut ctx).unwrap();
            s.is_fresh()
        };

        assert!(!run(64, &mut s), "a quarter of a frame is not a frame");
        assert!(run(256, &mut s), "past a full frame there is something to draw");
        // At 2.4 MS/s and 30 frames a second, the next 80k samples are not
        // worth transforming.
        assert!(!run(4096, &mut s), "the gate holds off the next frame");
        tags.clear();
    }

    #[test]
    fn the_spectrum_finds_the_tone_where_it_is() {
        let mut s = SpectrumNode::new(256);
        Simple::negotiate(&mut s, &spec(2_400_000.0)).unwrap();
        let mut out = Payload::Iq(Vec::new());
        let mut events = Vec::new();
        let tags = Vec::new();
        let ins = [spec(2_400_000.0)];
        let mut new_tags = Vec::new();
        let mut ctx = NodeCtx::new(0, &ins, &tags, &mut events, &mut new_tags);
        Simple::process(&mut s, &Payload::Iq(tone(4096)), &mut out, &mut ctx).unwrap();

        // 0.1 cycles per sample, so bin 0.1 * 256 from the centre.
        let db = s.power_db().to_vec();
        let peak = db
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.partial_cmp(b.1).unwrap())
            .map(|(i, _)| i)
            .unwrap();
        assert_eq!(peak, 128 + 26, "peak at bin {peak} of {}", db.len());
    }

    #[test]
    fn a_sink_writes_no_stream() {
        // The graph gives every node an output slot; a sink leaves it empty
        // rather than copying its input into a buffer nobody reads.
        let mut s = SpectrumNode::new(256);
        Simple::negotiate(&mut s, &spec(2_400_000.0)).unwrap();
        let mut out = Payload::Iq(Vec::new());
        let mut events = Vec::new();
        let tags = Vec::new();
        let ins = [spec(2_400_000.0)];
        let mut new_tags = Vec::new();
        let mut ctx = NodeCtx::new(0, &ins, &tags, &mut events, &mut new_tags);
        Simple::process(&mut s, &Payload::Iq(tone(512)), &mut out, &mut ctx).unwrap();
        assert_eq!(out.len(), 0);
    }

    struct Counting(usize);
    impl Ring for Counting {
        fn push(&mut self, iq: &[C32]) {
            self.0 += iq.len();
        }
    }

    #[test]
    fn the_ring_sees_every_sample_the_graph_was_fed() {
        let mut r = RingNode::new(Counting(0));
        Simple::negotiate(&mut r, &spec(2_400_000.0)).unwrap();
        let mut out = Payload::Iq(Vec::new());
        let mut events = Vec::new();
        let tags = Vec::new();
        let ins = [spec(2_400_000.0)];
        for _ in 0..3 {
            let mut new_tags = Vec::new();
            let mut ctx = NodeCtx::new(0, &ins, &tags, &mut events, &mut new_tags);
            Simple::process(&mut r, &Payload::Iq(tone(100)), &mut out, &mut ctx).unwrap();
        }
        assert_eq!(r.ring().0, 300);
    }
}
