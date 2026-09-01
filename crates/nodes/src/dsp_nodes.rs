//! DSP primitives wrapped as graph nodes.
//!
//! Each is a thin adapter: the arithmetic stays in `dsp`, and these add
//! rate negotiation and parameter introspection. Keeping them separate means
//! `dsp` is usable without the graph, and the graph never constrains how the
//! DSP is written.

use common::{Result, C32};
use dsp::agc::Agc;
use dsp::squelch::{NoiseMeter, Squelch};
use dsp::ssb::{Sideband, SsbDemod};
use dsp::{Deemphasis, FirDecim, FmDemod, HighBlend, Mixer};
use pipeline::param::{Param, ParamValue};
use pipeline::node::{NodeCtx, PortSpec, Simple};
use pipeline::port::{Payload, PortKind, StreamSpec, Tag, TagValue};

/// Shift a signal in frequency.
///
/// Used to bring an off-centre signal to baseband. Tuning a receiver so the
/// signal of interest sits at 0 Hz puts it directly on the RTL2832U's DC spur,
/// so the usual arrangement is to tune deliberately off and correct here.
pub struct MixerNode {
    shift_hz: f64,
    mixer: Mixer,
}

impl MixerNode {
    pub fn new(shift_hz: f64) -> Self {
        Self { shift_hz, mixer: Mixer::new(shift_hz, 1.0) }
    }
}

impl Simple for MixerNode {
    fn name(&self) -> &str {
        "mixer"
    }

    fn negotiate(&mut self, i: &PortSpec) -> Result<StreamSpec> {
        if i.spec.kind != PortKind::Iq {
            return Err(common::Error::other("mixer needs an IQ input"));
        }
        self.mixer.set_shift(self.shift_hz, i.spec.rate);
        // The centre frequency moves with the shift, so anything downstream
        // reporting "where did this come from" stays correct.
        let mut out = i.spec;
        out.center = common::Hz(
            (i.spec.center.get() as i64).saturating_sub(self.shift_hz as i64).max(0) as u64,
        );
        Ok(out)
    }

    fn process(&mut self, i: &Payload, o: &mut Payload, _c: &mut NodeCtx<'_>) -> Result<()> {
        self.mixer.process(i.as_iq().unwrap(), o.iq_mut());
        Ok(())
    }

    fn reset(&mut self) {
        self.mixer.reset();
    }

    fn params(&self) -> Vec<Param> {
        vec![Param::float("shift_hz", self.shift_hz, -30e6..=30e6)
            .unit("Hz")
            .label("Frequency shift")]
    }

    fn set_param(&mut self, name: &str, v: ParamValue) -> Result<()> {
        match name {
            "shift_hz" => {
                self.shift_hz = v.as_f64().ok_or_else(|| common::Error::other("expected a number"))?;
                Ok(())
            }
            _ => Err(common::Error::other(format!("mixer: unknown parameter {name:?}"))),
        }
    }
}

/// Lowpass and decimate.
pub struct DecimateNode {
    factor: usize,
    passband: f64,
    atten_db: f64,
    dec: FirDecim,
}

impl DecimateNode {
    /// Place the passband edge at a real frequency rather than a fraction of
    /// Nyquist. What matters is the signal's bandwidth: a filter sized from the
    /// decimation factor alone puts the transition band wherever it lands,
    /// which is either wasteful or lets an alias through.
    pub fn set_passband_hz(&mut self, input_rate: f64, hz: f64) {
        let out = input_rate / self.factor as f64;
        self.passband = (hz / (out / 2.0)).clamp(0.1, 0.99);
    }

    pub fn new(factor: usize) -> Self {
        Self {
            factor: factor.max(1),
            passband: 0.9,
            atten_db: 80.0,
            dec: FirDecim::design(factor.max(1), 0.9, 80.0),
        }
    }
}

impl Simple for DecimateNode {
    fn name(&self) -> &str {
        "decimate"
    }

    fn negotiate(&mut self, i: &PortSpec) -> Result<StreamSpec> {
        if i.spec.kind != PortKind::Iq {
            return Err(common::Error::other("decimate needs an IQ input"));
        }
        // Rebuild here, not in the constructor: the tap count depends on the
        // transition width, which is only knowable once the rate is.
        self.dec = FirDecim::design(self.factor, self.passband, self.atten_db);
        Ok(i.spec.with_rate(i.spec.rate / self.factor as f64))
    }

    fn latency(&self) -> u64 {
        // A symmetric FIR delays by half its length, measured at the output
        // rate. Reporting this is what lets a fan-in node align its branches.
        self.dec.latency() as u64
    }

    fn process(&mut self, i: &Payload, o: &mut Payload, _c: &mut NodeCtx<'_>) -> Result<()> {
        self.dec.process(i.as_iq().unwrap(), o.iq_mut());
        Ok(())
    }

    fn params(&self) -> Vec<Param> {
        vec![
            Param::int("factor", self.factor as i64, 1..=1024)
                .label("Decimation")
                .affects_rate(),
            Param::float("passband", self.passband, 0.5..=0.99).label("Passband fraction"),
            Param::float("atten_db", self.atten_db, 30.0..=120.0)
                .unit("dB")
                .label("Stopband attenuation"),
        ]
    }

    fn set_param(&mut self, name: &str, v: ParamValue) -> Result<()> {
        match name {
            "factor" => {
                self.factor = v.as_i64().unwrap_or(1).max(1) as usize;
                Ok(())
            }
            "passband" => {
                self.passband = v.as_f64().unwrap_or(0.9).clamp(0.1, 0.99);
                Ok(())
            }
            "atten_db" => {
                self.atten_db = v.as_f64().unwrap_or(80.0).clamp(20.0, 150.0);
                Ok(())
            }
            _ => Err(common::Error::other(format!("decimate: unknown parameter {name:?}"))),
        }
    }
}

/// Complex magnitude: the envelope an OOK detector needs.
pub struct EnvelopeNode;

impl Simple for EnvelopeNode {
    fn name(&self) -> &str {
        "envelope"
    }

    fn negotiate(&mut self, i: &PortSpec) -> Result<StreamSpec> {
        if i.spec.kind != PortKind::Iq {
            return Err(common::Error::other("envelope needs an IQ input"));
        }
        Ok(i.spec.with_kind(PortKind::Real))
    }

    fn process(&mut self, i: &Payload, o: &mut Payload, _c: &mut NodeCtx<'_>) -> Result<()> {
        o.real_mut().extend(i.as_iq().unwrap().iter().map(|c| c.norm()));
        Ok(())
    }
}

/// Frequency demodulator.
pub struct FmDemodNode {
    deviation_hz: f64,
    demod: FmDemod,
}

impl FmDemodNode {
    pub fn new(deviation_hz: f64) -> Self {
        Self { deviation_hz, demod: FmDemod::new(1.0, deviation_hz) }
    }

    /// Broadcast WFM: 75 kHz peak deviation.
    pub fn wide() -> Self {
        Self::new(75_000.0)
    }

    /// Narrowband voice and most FSK telemetry.
    pub fn narrow() -> Self {
        Self::new(5_000.0)
    }
}

impl Simple for FmDemodNode {
    fn name(&self) -> &str {
        "fm_demod"
    }

    fn negotiate(&mut self, i: &PortSpec) -> Result<StreamSpec> {
        if i.spec.kind != PortKind::Iq {
            return Err(common::Error::other("fm_demod needs an IQ input"));
        }
        self.demod = FmDemod::new(i.spec.rate, self.deviation_hz);
        Ok(i.spec.with_kind(PortKind::Real))
    }

    fn process(&mut self, i: &Payload, o: &mut Payload, _c: &mut NodeCtx<'_>) -> Result<()> {
        self.demod.process(i.as_iq().unwrap(), o.real_mut());
        Ok(())
    }

    fn reset(&mut self) {
        self.demod.reset();
    }

    fn params(&self) -> Vec<Param> {
        vec![Param::float("deviation_hz", self.deviation_hz, 500.0..=200_000.0)
            .unit("Hz")
            .label("Peak deviation")
            .log()]
    }

    fn set_param(&mut self, name: &str, v: ParamValue) -> Result<()> {
        match name {
            "deviation_hz" => {
                self.deviation_hz = v.as_f64().unwrap_or(75_000.0).max(1.0);
                Ok(())
            }
            _ => Err(common::Error::other(format!("fm_demod: unknown parameter {name:?}"))),
        }
    }
}

/// FM de-emphasis, undoing the transmitter's treble boost.
pub struct DeemphasisNode {
    tau_us: f64,
    /// One per channel. A single filter run over interleaved samples feeds
    /// each channel the other's history, which is both crosstalk and a cutoff
    /// at half the intended frequency.
    filt: Vec<Deemphasis>,
}

impl DeemphasisNode {
    pub fn new(tau_us: f64) -> Self {
        Self { tau_us, filt: vec![Deemphasis::new(1.0, tau_us)] }
    }
}

impl Simple for DeemphasisNode {
    fn name(&self) -> &str {
        "deemphasis"
    }

    fn negotiate(&mut self, i: &PortSpec) -> Result<StreamSpec> {
        if i.spec.kind != PortKind::Real {
            return Err(common::Error::other("deemphasis needs a real input"));
        }
        let ch = i.spec.channels.max(1);
        self.filt = (0..ch).map(|_| Deemphasis::new(i.spec.frame_rate(), self.tau_us)).collect();
        Ok(i.spec)
    }

    fn process(&mut self, i: &Payload, o: &mut Payload, _c: &mut NodeCtx<'_>) -> Result<()> {
        let out = o.real_mut();
        out.extend_from_slice(i.as_real().unwrap());
        let ch = self.filt.len().max(1);
        if ch == 1 {
            self.filt[0].process(out);
            return Ok(());
        }
        for (c, f) in self.filt.iter_mut().enumerate() {
            f.process_strided(out, c, ch);
        }
        Ok(())
    }

    fn reset(&mut self) {
        self.filt.iter_mut().for_each(|f| f.reset());
    }

    fn params(&self) -> Vec<Param> {
        vec![Param::float("tau_us", self.tau_us, 25.0..=100.0)
            .unit("us")
            .label("Time constant (50 EU, 75 Americas)")]
    }

    fn set_param(&mut self, name: &str, v: ParamValue) -> Result<()> {
        match name {
            "tau_us" => {
                self.tau_us = v.as_f64().unwrap_or(50.0).clamp(1.0, 1000.0);
                Ok(())
            }
            _ => Err(common::Error::other(format!("deemphasis: unknown parameter {name:?}"))),
        }
    }
}

/// Decimate a real-valued stream, for audio after a demodulator.
pub struct RealDecimateNode {
    factor: usize,
    passband: f64,
    dec: Vec<FirDecim>,
    scratch: Vec<C32>,
    out: Vec<C32>,
}

impl RealDecimateNode {
    pub fn new(factor: usize) -> Self {
        Self {
            factor: factor.max(1),
            passband: 0.9,
            dec: vec![FirDecim::design(factor.max(1), 0.9, 80.0)],
            scratch: Vec::new(),
            out: Vec::new(),
        }
    }

    /// Put the passband edge at an audio frequency rather than a fraction of
    /// Nyquist, so the filter is sized by what has to survive it.
    pub fn set_passband_hz(&mut self, input_frame_rate: f64, hz: f64) {
        let out = input_frame_rate / self.factor as f64;
        self.passband = (hz / (out / 2.0)).clamp(0.1, 0.99);
    }
}

impl Simple for RealDecimateNode {
    fn name(&self) -> &str {
        "real_decimate"
    }

    fn negotiate(&mut self, i: &PortSpec) -> Result<StreamSpec> {
        if i.spec.kind != PortKind::Real {
            return Err(common::Error::other("real_decimate needs a real input"));
        }
        let ch = i.spec.channels.max(1);
        self.dec =
            (0..ch).map(|_| FirDecim::design(self.factor, self.passband, 80.0)).collect();
        Ok(i.spec.with_rate(i.spec.frame_rate() / self.factor as f64))
    }

    fn process(&mut self, i: &Payload, o: &mut Payload, _c: &mut NodeCtx<'_>) -> Result<()> {
        // Reuses the complex decimator with a zero imaginary part. Wasteful by
        // half, but it keeps one well-tested filter implementation instead of
        // two that can drift apart.
        let src = i.as_real().unwrap();
        let ch = self.dec.len().max(1);
        let frames = src.len() / ch;
        let out = o.real_mut();
        let base = out.len();
        for c in 0..ch {
            self.scratch.clear();
            self.scratch
                .extend((0..frames).map(|k| C32::new(src[k * ch + c], 0.0)));
            self.out.clear();
            self.dec[c].process(&self.scratch, &mut self.out);
            if c == 0 {
                out.resize(base + self.out.len() * ch, 0.0);
            }
            for (k, v) in self.out.iter().enumerate() {
                let idx = base + k * ch + c;
                if idx < out.len() {
                    out[idx] = v.re;
                }
            }
        }
        Ok(())
    }

    fn params(&self) -> Vec<Param> {
        vec![Param::int("factor", self.factor as i64, 1..=1024)
            .label("Decimation")
            .affects_rate()]
    }

    fn set_param(&mut self, name: &str, v: ParamValue) -> Result<()> {
        match name {
            "factor" => {
                self.factor = v.as_i64().unwrap_or(1).max(1) as usize;
                Ok(())
            }
            _ => Err(common::Error::other(format!("real_decimate: unknown parameter {name:?}"))),
        }
    }
}

/// Roll the treble off as the signal gets noisy, reading the noise estimate a
/// demodulator upstream tagged onto the stream.
///
/// The measurement has to happen before decimation, since it looks at
/// discriminator output above the audio band, so this node cannot make it
/// itself and takes it from a tag instead.
pub struct HighBlendNode {
    /// Empty until negotiation. There is no placeholder rate worth inventing:
    /// the lowpass clamps its cutoff against the sample rate, so constructing
    /// one at a made-up rate panics rather than being merely wrong.
    blend: Vec<HighBlend>,
    noise: f32,
}

impl Default for HighBlendNode {
    fn default() -> Self {
        Self::new()
    }
}

impl HighBlendNode {
    pub fn new() -> Self {
        Self { blend: Vec::new(), noise: 0.0 }
    }

    /// Current cutoff, for display.
    pub fn cutoff(&self) -> f64 {
        self.blend.first().map(|b| b.cutoff()).unwrap_or(0.0)
    }
}

impl Simple for HighBlendNode {
    fn name(&self) -> &str {
        "high_blend"
    }

    fn negotiate(&mut self, i: &PortSpec) -> Result<StreamSpec> {
        if i.spec.kind != PortKind::Real {
            return Err(common::Error::other("high_blend needs a real input"));
        }
        let ch = i.spec.channels.max(1);
        self.blend = (0..ch).map(|_| HighBlend::new(i.spec.frame_rate())).collect();
        Ok(i.spec)
    }

    fn process(&mut self, i: &Payload, o: &mut Payload, c: &mut NodeCtx<'_>) -> Result<()> {
        // Last tag in the window rather than the first: it is the most recent
        // estimate, and a block covers many of them at audio rate.
        for t in c.in_tags {
            if t.key == "noise" {
                if let TagValue::Float(v) = t.value {
                    self.noise = v as f32;
                }
            }
        }
        let out = o.real_mut();
        out.extend_from_slice(i.as_real().unwrap());
        let ch = self.blend.len();
        if ch == 0 {
            return Err(common::Error::other("high_blend ran before negotiation"));
        }
        if ch == 1 {
            self.blend[0].process(self.noise, out);
            return Ok(());
        }
        // Deinterleave, filter, put back. Each channel must keep its own
        // history or the filter mixes the two together.
        let frames = out.len() / ch;
        let mut lane = vec![0.0f32; frames];
        for (k, b) in self.blend.iter_mut().enumerate() {
            for f in 0..frames {
                lane[f] = out[f * ch + k];
            }
            b.process(self.noise, &mut lane);
            for f in 0..frames {
                out[f * ch + k] = lane[f];
            }
        }
        Ok(())
    }

    fn reset(&mut self) {
        self.blend.iter_mut().for_each(|b| b.reset());
        self.noise = 0.0;
    }
}

/// Single sideband and CW demodulator.
pub struct SsbDemodNode {
    sideband: Sideband,
    low_hz: f64,
    high_hz: f64,
    demod: SsbDemod,
}

impl SsbDemodNode {
    pub fn new(sideband: Sideband, low_hz: f64, high_hz: f64) -> Self {
        Self {
            sideband,
            low_hz,
            high_hz,
            demod: SsbDemod::new(48_000.0, sideband, low_hz, high_hz),
        }
    }

    pub fn voice(sideband: Sideband) -> Self {
        Self::new(sideband, 300.0, 2_700.0)
    }

    /// A CW filter of `width_hz` centred on the pitch the operator hears.
    pub fn cw(sideband: Sideband, pitch_hz: f64, width_hz: f64) -> Self {
        let half = width_hz.max(50.0) / 2.0;
        Self::new(sideband, (pitch_hz - half).max(50.0), pitch_hz + half)
    }
}

impl Simple for SsbDemodNode {
    fn name(&self) -> &str {
        "ssb_demod"
    }

    fn negotiate(&mut self, i: &PortSpec) -> Result<StreamSpec> {
        if i.spec.kind != PortKind::Iq {
            return Err(common::Error::other("ssb_demod needs an IQ input"));
        }
        self.demod = SsbDemod::new(i.spec.rate, self.sideband, self.low_hz, self.high_hz);
        Ok(i.spec.with_kind(PortKind::Real))
    }

    fn process(&mut self, i: &Payload, o: &mut Payload, _c: &mut NodeCtx<'_>) -> Result<()> {
        self.demod.process(i.as_iq().unwrap(), o.real_mut());
        Ok(())
    }

    fn reset(&mut self) {
        self.demod.reset();
    }

    fn params(&self) -> Vec<Param> {
        vec![
            Param::float("low_hz", self.low_hz, 50.0..=3_000.0).unit("Hz").label("Filter low edge"),
            Param::float("high_hz", self.high_hz, 100.0..=6_000.0)
                .unit("Hz")
                .label("Filter high edge"),
        ]
    }

    fn set_param(&mut self, name: &str, v: ParamValue) -> Result<()> {
        match name {
            "low_hz" => self.low_hz = v.as_f64().unwrap_or(300.0),
            "high_hz" => self.high_hz = v.as_f64().unwrap_or(2_700.0),
            _ => return Err(common::Error::other(format!("ssb_demod: unknown parameter {name:?}"))),
        }
        self.demod = SsbDemod::new(self.demod_rate(), self.sideband, self.low_hz, self.high_hz);
        Ok(())
    }
}

impl SsbDemodNode {
    fn demod_rate(&self) -> f64 {
        48_000.0
    }
}

/// Automatic gain control on an audio stream.
pub struct AgcNode {
    attack_ms: f64,
    release_ms: f64,
    hang_ms: f64,
    max_gain_db: f32,
    enabled: bool,
    agc: Agc,
}

impl AgcNode {
    pub fn new(attack_ms: f64, release_ms: f64, hang_ms: f64) -> Self {
        Self {
            attack_ms,
            release_ms,
            hang_ms,
            max_gain_db: 60.0,
            enabled: true,
            agc: Agc::new(48_000.0, attack_ms, release_ms, hang_ms),
        }
    }

    pub fn voice() -> Self {
        Self::new(5.0, 500.0, 300.0)
    }

    pub fn cw() -> Self {
        Self::new(2.0, 1_000.0, 500.0)
    }
}

impl AgcNode {
    /// Gain currently applied, or 0 dB when switched off.
    pub fn gain_db(&self) -> f32 {
        if self.enabled { self.agc.gain_db() } else { 0.0 }
    }

    pub fn set_enabled(&mut self, on: bool) {
        // Reset on the way back in, so switching it on does not apply a gain
        // worked out from a signal that was there a minute ago.
        if on && !self.enabled {
            self.agc.reset();
        }
        self.enabled = on;
    }

    pub fn is_enabled(&self) -> bool {
        self.enabled
    }
}

impl Simple for AgcNode {
    fn name(&self) -> &str {
        "agc"
    }

    fn negotiate(&mut self, i: &PortSpec) -> Result<StreamSpec> {
        if i.spec.kind != PortKind::Real {
            return Err(common::Error::other("agc needs a real input"));
        }
        self.agc = Agc::new(i.spec.rate, self.attack_ms, self.release_ms, self.hang_ms);
        self.agc.set_max_gain_db(self.max_gain_db);
        Ok(i.spec)
    }

    fn process(&mut self, i: &Payload, o: &mut Payload, c: &mut NodeCtx<'_>) -> Result<()> {
        let out = o.real_mut();
        out.extend_from_slice(i.as_real().unwrap());
        if !self.enabled {
            return Ok(());
        }
        self.agc.process(out);
        // Reported rather than hidden: on a weak signal the gain is the
        // difference between "the band is dead" and "the receiver is deaf",
        // and only one of those is worth acting on.
        c.tag(Tag::new(c.sample_index, "agc_gain_db", TagValue::Float(self.agc.gain_db() as f64)));
        Ok(())
    }

    fn reset(&mut self) {
        self.agc.reset();
    }

    fn params(&self) -> Vec<Param> {
        vec![
            Param::float("attack_ms", self.attack_ms, 0.5..=50.0).unit("ms").label("Attack"),
            Param::float("release_ms", self.release_ms, 50.0..=5_000.0)
                .unit("ms")
                .label("Release")
                .log(),
            Param::float("hang_ms", self.hang_ms, 0.0..=2_000.0).unit("ms").label("Hang"),
            Param::float("max_gain_db", self.max_gain_db as f64, 0.0..=90.0)
                .unit("dB")
                .label("Maximum gain"),
            Param::bool("enabled", self.enabled).label("Enabled"),
        ]
    }

    fn set_param(&mut self, name: &str, v: ParamValue) -> Result<()> {
        match name {
            "attack_ms" => self.attack_ms = v.as_f64().unwrap_or(5.0),
            "release_ms" => self.release_ms = v.as_f64().unwrap_or(500.0),
            "hang_ms" => self.hang_ms = v.as_f64().unwrap_or(300.0),
            "max_gain_db" => {
                self.max_gain_db = v.as_f64().unwrap_or(60.0) as f32;
                self.agc.set_max_gain_db(self.max_gain_db);
                return Ok(());
            }
            "enabled" => {
                self.set_enabled(v.as_bool().unwrap_or(true));
                return Ok(());
            }
            _ => return Err(common::Error::other(format!("agc: unknown parameter {name:?}"))),
        }
        let rate = self.agc.rate();
        self.agc = Agc::new(rate, self.attack_ms, self.release_ms, self.hang_ms);
        self.agc.set_max_gain_db(self.max_gain_db);
        Ok(())
    }
}

/// How a squelch decides whether a channel is busy.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SquelchKind {
    /// Noise above the speech band against everything below it. For FM.
    Noise,
    /// Plain audio level. For everything else.
    Level,
}

/// Mute a channel with nothing on it.
pub struct SquelchNode {
    kind: SquelchKind,
    threshold_db: f32,
    hysteresis_db: f32,
    squelch: Squelch,
    meter: NoiseMeter,
    open: bool,
    measured: f32,
}

/// How long the mute takes to open or close, in milliseconds.
///
/// Five was a click on every transmission when the threshold sat near the
/// signal's own level; twenty is short enough not to swallow the first
/// syllable and long enough that the edge is a fade rather than a step.
const RAMP_MS: f64 = 20.0;

impl SquelchNode {
    pub fn new(kind: SquelchKind, threshold_db: f32) -> Self {
        Self {
            kind,
            threshold_db,
            hysteresis_db: 3.0,
            squelch: Squelch::new(48_000.0, threshold_db, threshold_db - 3.0, RAMP_MS),
            meter: NoiseMeter::new(48_000.0, 4_000.0),
            open: false,
            measured: -120.0,
        }
    }

    /// Narrowband FM, at the level where a signal becomes intelligible.
    pub fn fm() -> Self {
        Self::new(SquelchKind::Noise, 9.0)
    }

    pub fn is_open(&self) -> bool {
        self.open
    }

    /// What the squelch measured on the last block, in dB.
    pub fn measured_db(&self) -> f32 {
        self.measured
    }

    /// Where the squelch opens, in dB on whatever it is measuring.
    pub fn threshold_db(&self) -> f32 {
        self.threshold_db
    }

    pub fn set_threshold_db(&mut self, db: f32) {
        self.threshold_db = db;
        self.squelch.set_thresholds(db, db - self.hysteresis_db);
    }

    /// What the threshold means for this squelch, for a control to label.
    pub fn kind(&self) -> SquelchKind {
        self.kind
    }
}

impl Simple for SquelchNode {
    fn name(&self) -> &str {
        "squelch"
    }

    fn negotiate(&mut self, i: &PortSpec) -> Result<StreamSpec> {
        if i.spec.kind != PortKind::Real {
            return Err(common::Error::other("squelch needs a real input"));
        }
        self.squelch = Squelch::new(
            i.spec.rate,
            self.threshold_db,
            self.threshold_db - self.hysteresis_db,
            RAMP_MS,
        );
        self.meter = NoiseMeter::new(i.spec.rate, 4_000.0);
        Ok(i.spec)
    }

    fn process(&mut self, i: &Payload, o: &mut Payload, c: &mut NodeCtx<'_>) -> Result<()> {
        let input = i.as_real().unwrap();
        let measured = match self.kind {
            SquelchKind::Noise => self.meter.measure(input),
            SquelchKind::Level => dsp::squelch::level_db(input),
        };
        self.open = self.squelch.update(measured, input.len());
        // The smoothed figure, not the raw one. The meter exists to set the
        // threshold against, and a bar that jumps either side of a line the
        // audio is not crossing makes the control look broken.
        self.measured = self.squelch.level_db();
        let out = o.real_mut();
        out.extend_from_slice(input);
        self.squelch.apply(out);
        let at = c.sample_index;
        c.tag(Tag::new(at, "squelch_open", TagValue::Int(self.open as i64)));
        c.tag(Tag::new(at, "squelch_db", TagValue::Float(measured as f64)));
        Ok(())
    }

    fn reset(&mut self) {
        self.squelch.reset();
        self.meter.reset();
        self.open = false;
    }

    fn params(&self) -> Vec<Param> {
        vec![
            Param::float("threshold_db", self.threshold_db as f64, -100.0..=40.0)
                .unit("dB")
                .label("Threshold"),
            Param::float("hysteresis_db", self.hysteresis_db as f64, 0.0..=20.0)
                .unit("dB")
                .label("Hysteresis"),
        ]
    }

    fn set_param(&mut self, name: &str, v: ParamValue) -> Result<()> {
        match name {
            "threshold_db" => self.threshold_db = v.as_f64().unwrap_or(9.0) as f32,
            "hysteresis_db" => self.hysteresis_db = v.as_f64().unwrap_or(3.0) as f32,
            _ => return Err(common::Error::other(format!("squelch: unknown parameter {name:?}"))),
        }
        self.squelch
            .set_thresholds(self.threshold_db, self.threshold_db - self.hysteresis_db);
        Ok(())
    }
}
