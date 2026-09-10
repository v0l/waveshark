//! A scope: wire it in anywhere and look at what is flowing there.
//!
//! The chain view draws what the receiver is doing; this is for seeing what
//! a wire carries while it does it. Inserted between two stages it passes
//! the stream through untouched and keeps three readings of it: a spectrum,
//! a spectrogram of the last few seconds of spectra, and a level meter. The
//! interface reads them back by downcast, the way it reads the receiver's
//! own spectrum, and draws them in the inspector for the stage.
//!
//! It takes complex baseband or real audio. A real stream's spectrum is one
//! sided, from DC to half the rate, and its level is what a VU meter would
//! show; a complex one is drawn from minus half the rate to plus, centred on
//! the stream's frequency.

use common::{Result, C32};
use dsp::{FirDecim, Spectrum};
use pipeline::node::{NodeCtx, PortSpec, Simple};
use pipeline::param::{Param, ParamValue};
use pipeline::port::{Payload, PortKind, StreamSpec};
use pipeline::registry::{Category, Settings, SettingsExt, StageDesc};

/// Lowest reading kept, in dBFS.
pub const FLOOR_DB: f32 = -140.0;

/// Rows of spectrogram kept: at the default refresh that is a little over
/// four seconds, which is a transmission and its surroundings.
pub const HISTORY_ROWS: usize = 128;

/// What the scope has seen lately, for the interface to draw.
#[derive(Clone, Debug, Default)]
pub struct ScopeFrame {
    pub rate: f64,
    pub center_hz: f64,
    /// Whether the stream is real, and so the spectrum one sided.
    pub real: bool,
    /// Latest spectrum in dBFS, lowest frequency first.
    pub spectrum: Vec<f32>,
    /// Spectrogram rows, oldest first, each `spectrum.len()` long, in dBFS.
    pub history: Vec<Vec<f32>>,
    /// Peak and rms of the last block, linear.
    pub peak: f32,
    pub rms: f32,
    /// Peak held with a slow fall, for the meter's peak mark.
    pub peak_hold: f32,
}

/// Widest a real audio wire is shown by default. A microphone or a
/// demodulator's output runs at whatever rate the graph has there, which on
/// a transmit chain is the radio's, and a spectrum of speech across a
/// megahertz is three pixels of speech.
const AUDIO_SPAN_HZ: f64 = 10_000.0;

pub struct ScopeNode {
    size: usize,
    refresh_hz: f32,
    /// Width shown, in hertz, or zero for the whole stream. On a real stream
    /// this is from DC up; on IQ it is centred.
    span_hz: f64,
    /// Brings the stream down to about twice the span before the transform,
    /// so the bins land on what is being looked at.
    decim: Option<FirDecim>,
    /// Rate the transform sees.
    seen_rate: f64,
    spec: Spectrum,
    rate: f64,
    center_hz: f64,
    real: bool,
    /// Samples still to skip before the next frame is collected, so the
    /// transform runs at the refresh rate and not at the stream's.
    debt: f64,
    collecting: bool,
    frame: ScopeFrame,
    fresh: bool,
    scratch: Vec<C32>,
    scratch_dec: Vec<C32>,
}

impl Default for ScopeNode {
    fn default() -> Self {
        Self::new(1024)
    }
}

impl ScopeNode {
    pub fn new(size: usize) -> Self {
        let size = size.next_power_of_two().clamp(64, 16_384);
        Self {
            size,
            refresh_hz: 30.0,
            span_hz: 0.0,
            decim: None,
            seen_rate: 0.0,
            spec: Spectrum::new(size),
            rate: 0.0,
            center_hz: 0.0,
            real: false,
            debt: 0.0,
            collecting: true,
            frame: ScopeFrame::default(),
            fresh: false,
            scratch: Vec::new(),
            scratch_dec: Vec::new(),
        }
    }

    /// The latest readings, and whether anything changed since last asked.
    pub fn frame(&mut self) -> (&ScopeFrame, bool) {
        let fresh = std::mem::take(&mut self.fresh);
        (&self.frame, fresh)
    }

    fn level(&mut self, peak: f32, rms: f32) {
        self.frame.peak = peak;
        self.frame.rms = rms;
        // Falls about 20 dB a second at the refresh rate.
        let fall = 10f32.powf(-1.0 / self.refresh_hz.max(1.0));
        self.frame.peak_hold = (self.frame.peak_hold * fall).max(peak);
    }

    /// The decimator for the span set, or none when the whole stream is
    /// shown. Complex through and through: a real stream is already promoted
    /// by the time it gets here, and a lowpass on a real signal made complex
    /// keeps the positive half, which is the half that is drawn.
    fn design(&mut self) {
        let full = if self.real { self.rate / 2.0 } else { self.rate };
        let want = if self.span_hz > 0.0 { self.span_hz.min(full) } else { full };
        // Twice the span of complex rate for IQ, since the span is the whole
        // width shown; a real stream shows only the top half of its
        // transform, so it needs twice that again.
        let out_rate = if self.real { want * 2.2 } else { want * 1.1 };
        let factor = (self.rate / out_rate).floor().max(1.0) as usize;
        if factor <= 1 {
            self.decim = None;
            self.seen_rate = self.rate;
        } else {
            let pb = if self.real { want } else { want / 2.0 };
            self.decim = Some(FirDecim::design_hz(self.rate, factor, pb, 60.0));
            self.seen_rate = self.rate / factor as f64;
        }
        self.frame.rate = self.seen_rate;
        self.spec.reset();
        self.frame.history.clear();
    }

    fn transform(&mut self, iq: &[C32]) {
        if self.decim.is_some() {
            let mut dec = std::mem::take(&mut self.scratch_dec);
            dec.clear();
            self.decim.as_mut().unwrap().process(iq, &mut dec);
            self.transform_seen(&dec);
            self.scratch_dec = dec;
        } else {
            self.transform_seen(iq);
        }
    }

    fn transform_seen(&mut self, iq: &[C32]) {
        // Collect one frame's worth per refresh period, then skip the rest.
        let period = self.seen_rate / self.refresh_hz.max(1.0) as f64;
        let mut at = 0usize;
        let mut any = false;
        while at < iq.len() {
            if !self.collecting {
                let skip = (self.debt.ceil() as usize).min(iq.len() - at);
                self.debt -= skip as f64;
                at += skip;
                if self.debt <= 0.0 {
                    self.collecting = true;
                    self.debt = 0.0;
                }
                continue;
            }
            let take = (iq.len() - at).min(self.size);
            if self.spec.process(&iq[at..at + take]) {
                any = true;
                self.collecting = false;
                self.debt = (period - self.size as f64).max(0.0);
            }
            at += take;
        }
        if any {
            let db = self.spec.power_db();
            // Floored where a 24 bit converter's noise would be. An empty
            // wire transforms to -200 dB with a DC bin of numerical dust
            // above it, and drawn against real signal that dust was a
            // bright band across the picture.
            let clamp = |v: f32| v.max(FLOOR_DB);
            let row: Vec<f32> = if self.real {
                // One sided: the upper half of the shifted spectrum is DC up.
                db[db.len() / 2..].iter().map(|&v| clamp(v)).collect()
            } else {
                db.iter().map(|&v| clamp(v)).collect()
            };
            self.frame.spectrum = row.clone();
            self.frame.history.push(row);
            if self.frame.history.len() > HISTORY_ROWS {
                self.frame.history.remove(0);
            }
            self.fresh = true;
        }
    }
}

impl Simple for ScopeNode {
    fn name(&self) -> &str {
        "scope"
    }

    fn negotiate(&mut self, input: &PortSpec) -> Result<StreamSpec> {
        match input.spec.kind {
            PortKind::Iq => self.real = false,
            PortKind::Real => self.real = true,
            _ => return Err(common::Error::other("scope looks at samples: IQ or real audio")),
        }
        self.rate = input.spec.rate;
        self.center_hz = input.spec.center.as_f64();
        self.spec = Spectrum::new(self.size);
        if self.real && self.span_hz <= 0.0 {
            self.span_hz = AUDIO_SPAN_HZ.min(self.rate / 2.0);
        }
        self.frame = ScopeFrame {
            rate: self.rate,
            center_hz: self.center_hz,
            real: self.real,
            ..Default::default()
        };
        self.design();
        Ok(input.spec)
    }

    fn process(
        &mut self,
        input: &Payload,
        output: &mut Payload,
        _c: &mut NodeCtx<'_>,
    ) -> Result<()> {
        match input {
            Payload::Iq(iq) => {
                output.iq_mut().extend_from_slice(iq);
                if iq.is_empty() {
                    return Ok(());
                }
                let (mut peak, mut sum) = (0.0f32, 0.0f32);
                for c in iq {
                    let p = c.norm_sqr();
                    sum += p;
                    peak = peak.max(p);
                }
                self.level(peak.sqrt(), (sum / iq.len() as f32).sqrt());
                self.transform(iq);
            }
            Payload::Real(v) => {
                output.real_mut().extend_from_slice(v);
                if v.is_empty() {
                    return Ok(());
                }
                let (mut peak, mut sum) = (0.0f32, 0.0f32);
                for &x in v {
                    sum += x * x;
                    peak = peak.max(x.abs());
                }
                self.level(peak, (sum / v.len() as f32).sqrt());
                self.scratch.clear();
                self.scratch.extend(v.iter().map(|&x| C32::new(x, 0.0)));
                let s = std::mem::take(&mut self.scratch);
                self.transform(&s);
                self.scratch = s;
            }
            _ => {}
        }
        Ok(())
    }

    fn reset(&mut self) {
        self.spec.reset();
        self.frame.history.clear();
        self.frame.peak_hold = 0.0;
        self.debt = 0.0;
        self.collecting = true;
    }

    fn params(&self) -> Vec<Param> {
        vec![
            Param::int(FFT_SIZE, self.size as i64, 64..=16_384).label("FFT size"),
            Param::float(REFRESH_HZ, self.refresh_hz as f64, 1.0..=120.0)
                .label("Refresh")
                .unit("Hz"),
            Param::float("span_hz", self.span_hz, 0.0..=50_000_000.0)
                .label("Span (0 = all)")
                .unit("Hz"),
        ]
    }

    fn set_param(&mut self, name: &str, value: ParamValue) -> Result<()> {
        match name {
            FFT_SIZE => {
                let n = (value.as_i64().unwrap_or(1024).max(64) as usize)
                    .next_power_of_two()
                    .min(16_384);
                if n != self.size {
                    self.size = n;
                    self.spec = Spectrum::new(n);
                    if self.rate > 0.0 {
                        self.design();
                    }
                }
                Ok(())
            }
            REFRESH_HZ => {
                self.refresh_hz = value.as_f64().unwrap_or(30.0).clamp(1.0, 120.0) as f32;
                Ok(())
            }
            "span_hz" => {
                self.span_hz = value.as_f64().unwrap_or(0.0).max(0.0);
                if self.rate > 0.0 {
                    self.design();
                }
                Ok(())
            }
            _ => Err(common::Error::other(format!("scope: unknown parameter {name:?}"))),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use pipeline::port::Flow;

    fn run(node: &mut ScopeNode, spec: StreamSpec, input: Payload) -> Payload {
        Simple::negotiate(node, &PortSpec { spec, latency: 0 }).unwrap();
        let mut out = match input {
            Payload::Iq(_) => Payload::Iq(Vec::new()),
            _ => Payload::Real(Vec::new()),
        };
        let (mut ev, mut tg) = (Vec::new(), Vec::new());
        let ins = [PortSpec { spec, latency: 0 }];
        let mut ctx = NodeCtx::new(0, &ins, &[], &mut ev, &mut tg);
        Simple::process(node, &input, &mut out, &mut ctx).unwrap();
        out
    }

    #[test]
    fn a_tone_passes_through_and_lands_in_the_right_bin() {
        let rate = 48_000.0;
        let iq: Vec<C32> = (0..8192)
            .map(|i| {
                let p = std::f32::consts::TAU * 6_000.0 * i as f32 / rate as f32;
                C32::new(0.5 * p.cos(), 0.5 * p.sin())
            })
            .collect();
        let mut n = ScopeNode::new(1024);
        let spec = StreamSpec::iq(rate, common::Hz(1_000_000));
        let out = run(&mut n, spec, Payload::Iq(iq.clone()));
        assert_eq!(out.as_iq().unwrap(), &iq[..], "the scope changed the stream");
        let (f, fresh) = n.frame();
        assert!(fresh);
        assert!(!f.real);
        assert_eq!(f.spectrum.len(), 1024);
        let peak = f.spectrum.iter().enumerate().max_by(|a, b| a.1.total_cmp(b.1)).unwrap().0;
        // +6 kHz of a 48 kHz span: an eighth of the way up from the centre.
        assert!((peak as i64 - 640).abs() <= 1, "peak in bin {peak}");
        assert!((f.peak - 0.5).abs() < 0.01 && (f.rms - 0.5).abs() < 0.01);
    }

    #[test]
    fn real_audio_gives_a_one_sided_spectrum_and_a_vu_reading() {
        let rate = 48_000.0;
        let v: Vec<f32> = (0..8192)
            .map(|i| 0.25 * (std::f32::consts::TAU * 1_000.0 * i as f32 / rate as f32).sin())
            .collect();
        let mut n = ScopeNode::new(1024);
        let spec = StreamSpec {
            kind: PortKind::Real,
            rate,
            center: common::Hz(0),
            bandwidth: 0.0,
            flow: Flow::Rx,
            ..Default::default()
        };
        run(&mut n, spec, Payload::Real(v));
        let (f, _) = n.frame();
        assert!(f.real);
        assert_eq!(f.spectrum.len(), 512);
        // The default span on audio is 10 kHz, so the stream is brought
        // down to 24 kS/s and the half spectrum runs to 12 kHz.
        assert!((f.rate - 24_000.0).abs() < 1.0, "seen at {}", f.rate);
        let peak = f.spectrum.iter().enumerate().max_by(|a, b| a.1.total_cmp(b.1)).unwrap().0;
        assert!((peak as i64 - 43).abs() <= 1, "peak in bin {peak}");
        assert!((f.peak - 0.25).abs() < 0.01);
        assert!((f.rms - 0.25 / 2f32.sqrt()).abs() < 0.01);
    }
}

#[cfg(test)]
mod span_tests {
    use super::*;
    use pipeline::port::Flow;

    /// Speech at the radio's rate: the span brings it down so the audio band
    /// fills the picture rather than the first few bins of it.
    #[test]
    fn a_real_wire_at_a_radio_rate_is_shown_across_the_audio_band() {
        let rate = 2_048_000.0;
        let v: Vec<f32> = (0..262_144)
            .map(|i| 0.25 * (std::f32::consts::TAU * 2_000.0 * i as f32 / rate as f32).sin())
            .collect();
        let mut n = ScopeNode::new(1024);
        let spec = StreamSpec {
            kind: PortKind::Real,
            rate,
            center: common::Hz(0),
            bandwidth: 0.0,
            flow: Flow::Rx,
            ..Default::default()
        };
        Simple::negotiate(&mut n, &PortSpec { spec, latency: 0 }).unwrap();
        let mut out = Payload::Real(Vec::new());
        let (mut ev, mut tg) = (Vec::new(), Vec::new());
        let ins = [PortSpec { spec, latency: 0 }];
        let mut ctx = NodeCtx::new(0, &ins, &[], &mut ev, &mut tg);
        Simple::process(&mut n, &Payload::Real(v), &mut out, &mut ctx).unwrap();
        let (f, _) = n.frame();
        assert!(f.rate < 30_000.0, "still looking at {} S/s", f.rate);
        let half = f.rate / 2.0;
        let peak = f.spectrum.iter().enumerate().max_by(|a, b| a.1.total_cmp(b.1)).unwrap().0;
        let hz = peak as f64 / f.spectrum.len() as f64 * half;
        assert!(
            (hz - 2_000.0).abs() < half / f.spectrum.len() as f64 * 2.0,
            "tone read at {hz:.0} Hz"
        );
    }
}

/// The setting names this stage reads.
const FFT_SIZE: &str = "fft_size";
const REFRESH_HZ: &str = "refresh_hz";

/// How many bins the transform runs at, and how often it is drawn, when
/// nothing has said otherwise.
const DEFAULT_FFT_SIZE: i64 = 1_024;
const DEFAULT_REFRESH_HZ: f64 = 30.0;

pub const DESC: StageDesc = StageDesc {
    name: "scope",
    summary: "Look at a wire: a spectrum, a spectrogram and a level meter of \
              whatever passes through, which it passes on untouched",
    category: Category::Sink,
    feeds_bus: false,
};

pub fn build(s: &Settings) -> Result<Box<dyn pipeline::node::Node>> {
    let mut n = ScopeNode::new(s.i64_or(FFT_SIZE, DEFAULT_FFT_SIZE).max(64) as usize);
    Simple::set_param(
        &mut n,
        REFRESH_HZ,
        ParamValue::Float(s.f64_or(REFRESH_HZ, DEFAULT_REFRESH_HZ)),
    )?;
    Ok(Box::new(n))
}
