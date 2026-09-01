//! Pass and block filters as graph stages.
//!
//! Two of them, because the choice between them is a real one rather than an
//! implementation detail: an FIR is linear phase and costs a tap per sample
//! per tap, an IIR costs five multiplies and smears phase around its cutoff.
//! A chain being drawn by hand wants both, and wants to be told which is
//! which rather than being handed "filter".
//!
//! Both take whatever they are given, IQ or real. On an IQ stream the
//! response is symmetric about the middle of the span, which is what a filter
//! described by one frequency can mean there; a filter meant to keep one side
//! of a carrier is a mixer and one of these, which is how the channel chains
//! do it.

use common::{Result, C32};
use dsp::filter::{design, Biquad, Response};
use dsp::fir::Fir;
use pipeline::node::{NodeCtx, PortSpec, Simple};
use pipeline::param::{Param, ParamValue};
use pipeline::port::{Payload, PortKind, StreamSpec};

const ATTEN_DB: f64 = 70.0;

/// The response names, in the order a control offers them.
const RESPONSES: [Response; 4] =
    [Response::Lowpass, Response::Highpass, Response::Bandpass, Response::Bandstop];

fn response_index(r: Response) -> usize {
    RESPONSES.iter().position(|x| *x == r).unwrap_or(0)
}

fn response_names() -> Vec<String> {
    RESPONSES.iter().map(|r| r.name().to_string()).collect()
}

/// A windowed-sinc filter, designed at the rate it is handed.
pub struct FirFilterNode {
    response: Response,
    freq_hz: f64,
    width_hz: f64,
    taps: usize,
    rate: f64,
    /// One filter per stream: an IQ stream runs the complex one, a real
    /// stream a real one per channel, since running one filter over
    /// interleaved samples feeds each channel the other's history.
    iq: Fir,
    real: Vec<RealFir>,
    scratch: Vec<C32>,
}

impl FirFilterNode {
    pub fn new(response: Response, freq_hz: f64, width_hz: f64, taps: usize) -> Self {
        Self {
            response,
            freq_hz,
            width_hz,
            taps: taps.max(3),
            rate: 0.0,
            iq: Fir::new(vec![1.0]),
            real: Vec::new(),
            scratch: Vec::new(),
        }
    }

    fn taps_now(&self) -> Vec<f32> {
        design(self.response, self.taps, self.rate.max(1.0), self.freq_hz, self.width_hz, ATTEN_DB)
    }

    fn redesign(&mut self, channels: usize, iq: bool) {
        let h = self.taps_now();
        if iq {
            self.iq = Fir::new(h);
            self.real.clear();
        } else {
            self.real = (0..channels.max(1)).map(|_| RealFir::new(h.clone())).collect();
        }
    }
}

impl Simple for FirFilterNode {
    fn name(&self) -> &str {
        "fir_filter"
    }

    fn negotiate(&mut self, i: &PortSpec) -> Result<StreamSpec> {
        if !matches!(i.spec.kind, PortKind::Iq | PortKind::Real) {
            return Err(common::Error::other("a filter takes IQ or audio"));
        }
        self.rate = i.spec.frame_rate();
        self.redesign(i.spec.channels.max(1), i.spec.kind == PortKind::Iq);
        Ok(i.spec)
    }

    fn latency(&self) -> u64 {
        // Linear phase, so the delay is half the taps and the same at every
        // frequency. Reported rather than hidden: it is added to whatever
        // else the chain owes, and a chain that under-reports its delay
        // mistimes everything downstream of it.
        (self.taps.max(3) / 2) as u64
    }

    fn process(&mut self, i: &Payload, o: &mut Payload, _c: &mut NodeCtx<'_>) -> Result<()> {
        if let Some(iq) = i.as_iq() {
            let out = o.iq_mut();
            self.iq.process(iq, out);
            return Ok(());
        }
        let real = i.as_real().unwrap_or(&[]);
        let out = o.real_mut();
        out.extend_from_slice(real);
        let ch = self.real.len().max(1);
        if self.real.is_empty() {
            return Ok(());
        }
        for (c, f) in self.real.iter_mut().enumerate() {
            f.process_strided(out, c, ch);
        }
        Ok(())
    }

    fn reset(&mut self) {
        self.iq.reset();
        self.real.iter_mut().for_each(|f| f.reset());
        self.scratch.clear();
    }

    fn params(&self) -> Vec<Param> {
        vec![
            Param::choice("response", response_index(self.response), response_names())
                .label("What it keeps"),
            Param::float("freq_hz", self.freq_hz, 10.0..=30e6)
                .unit("Hz")
                .log()
                .label(if self.response.is_band() { "Band centre" } else { "Cutoff" }),
            Param::float("width_hz", self.width_hz, 10.0..=30e6)
                .unit("Hz")
                .log()
                .label("Band width"),
            Param::int("taps", self.taps as i64, 3..=2047).label("Taps"),
        ]
    }

    fn set_param(&mut self, name: &str, v: ParamValue) -> Result<()> {
        match name {
            "response" => {
                let i = v.as_i64().unwrap_or(0).clamp(0, RESPONSES.len() as i64 - 1) as usize;
                self.response = RESPONSES[i];
            }
            "freq_hz" => self.freq_hz = v.as_f64().unwrap_or(self.freq_hz),
            "width_hz" => self.width_hz = v.as_f64().unwrap_or(self.width_hz),
            "taps" => self.taps = v.as_i64().unwrap_or(self.taps as i64).clamp(3, 4095) as usize,
            _ => return Err(common::Error::other(format!("fir_filter: unknown parameter {name:?}"))),
        }
        // Designed again rather than at the next negotiation: a filter that
        // took a new cutoff and kept filtering at the old one would be a
        // control that does nothing until something else happens to change.
        let iq = self.real.is_empty();
        let ch = self.real.len();
        self.redesign(ch, iq);
        Ok(())
    }
}

/// A real-valued FIR with its own history, kept here because the DSP crate's
/// one is a decimator and a filter that keeps every sample is not that.
struct RealFir {
    taps: Vec<f32>,
    hist: Vec<f32>,
}

impl RealFir {
    fn new(taps: Vec<f32>) -> Self {
        let n = taps.len();
        Self { taps, hist: vec![0.0; n] }
    }

    fn reset(&mut self) {
        self.hist.iter_mut().for_each(|v| *v = 0.0);
    }

    /// Filter every `stride`th sample starting at `offset`, in place: one
    /// channel of an interleaved buffer.
    fn process_strided(&mut self, buf: &mut [f32], offset: usize, stride: usize) {
        let n = self.taps.len();
        let mut i = offset;
        while i < buf.len() {
            self.hist.copy_within(1..n, 0);
            self.hist[n - 1] = buf[i];
            let mut acc = 0.0;
            for (k, c) in self.taps.iter().enumerate() {
                acc += c * self.hist[n - 1 - k];
            }
            buf[i] = acc;
            i += stride;
        }
    }
}

/// A biquad, for when a few coefficients beat a few hundred taps.
pub struct IirFilterNode {
    response: Response,
    freq_hz: f64,
    q: f64,
    rate: f64,
    /// Two sections per complex stream, one per real channel.
    sections: Vec<Biquad>,
    iq: bool,
}

impl IirFilterNode {
    pub fn new(response: Response, freq_hz: f64, q: f64) -> Self {
        Self {
            response,
            freq_hz,
            q: q.max(0.05),
            rate: 0.0,
            sections: Vec::new(),
            iq: false,
        }
    }

    fn redesign(&mut self, n: usize) {
        let f = Biquad::design(self.response, self.rate.max(1.0), self.freq_hz, self.q);
        self.sections = vec![f; n.max(1)];
    }
}

impl Simple for IirFilterNode {
    fn name(&self) -> &str {
        "iir_filter"
    }

    fn negotiate(&mut self, i: &PortSpec) -> Result<StreamSpec> {
        if !matches!(i.spec.kind, PortKind::Iq | PortKind::Real) {
            return Err(common::Error::other("a filter takes IQ or audio"));
        }
        self.rate = i.spec.frame_rate();
        self.iq = i.spec.kind == PortKind::Iq;
        // A complex stream needs one section per component; a real one needs
        // one per channel.
        self.redesign(if self.iq { 2 } else { i.spec.channels.max(1) });
        Ok(i.spec)
    }

    fn process(&mut self, i: &Payload, o: &mut Payload, _c: &mut NodeCtx<'_>) -> Result<()> {
        if let Some(iq) = i.as_iq() {
            let out = o.iq_mut();
            out.reserve(iq.len());
            let (re, im) = self.sections.split_at_mut(1);
            for s in iq {
                out.push(C32::new(re[0].process(s.re), im[0].process(s.im)));
            }
            return Ok(());
        }
        let out = o.real_mut();
        out.extend_from_slice(i.as_real().unwrap_or(&[]));
        let ch = self.sections.len().max(1);
        for (c, f) in self.sections.iter_mut().enumerate() {
            let mut k = c;
            while k < out.len() {
                out[k] = f.process(out[k]);
                k += ch;
            }
        }
        Ok(())
    }

    fn reset(&mut self) {
        self.sections.iter_mut().for_each(|f| f.reset());
    }

    fn params(&self) -> Vec<Param> {
        vec![
            Param::choice("response", response_index(self.response), response_names())
                .label("What it keeps"),
            Param::float("freq_hz", self.freq_hz, 10.0..=30e6)
                .unit("Hz")
                .log()
                .label(if self.response.is_band() { "Band centre" } else { "Cutoff" }),
            Param::float("q", self.q, 0.1..=50.0)
                .log()
                .label("Resonance: higher is narrower"),
        ]
    }

    fn set_param(&mut self, name: &str, v: ParamValue) -> Result<()> {
        match name {
            "response" => {
                let i = v.as_i64().unwrap_or(0).clamp(0, RESPONSES.len() as i64 - 1) as usize;
                self.response = RESPONSES[i];
            }
            "freq_hz" => self.freq_hz = v.as_f64().unwrap_or(self.freq_hz),
            "q" => self.q = v.as_f64().unwrap_or(self.q).clamp(0.05, 200.0),
            _ => return Err(common::Error::other(format!("iir_filter: unknown parameter {name:?}"))),
        }
        let n = self.sections.len();
        self.redesign(n);
        Ok(())
    }
}
