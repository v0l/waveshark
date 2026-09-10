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
use pipeline::node::{Node, NodeCtx, PortSpec, Simple};
use pipeline::param::{Param, ParamValue};
use pipeline::port::{Payload, PortKind, StreamSpec, Tag, TagValue};
use pipeline::registry::{Category, Settings, SettingsExt, StageDesc};

/// The setting names these stages read, spelled once for the builder, the
/// parameter list and the setter that share each of them.
const SHIFT_HZ: &str = "shift_hz";
const FACTOR: &str = "factor";
const PASSBAND: &str = "passband";
const ATTEN_DB: &str = "atten_db";
const DEVIATION_HZ: &str = "deviation_hz";
const TAU_US: &str = "tau_us";
const SIDEBAND: &str = "sideband";
const LOW_HZ: &str = "low_hz";
const HIGH_HZ: &str = "high_hz";
const PITCH_HZ: &str = "pitch_hz";
const WIDTH_HZ: &str = "width_hz";
const PRESET: &str = "preset";
const ATTACK_MS: &str = "attack_ms";
const RELEASE_MS: &str = "release_ms";
const HANG_MS: &str = "hang_ms";
const MAX_GAIN_DB: &str = "max_gain_db";
const ENABLED: &str = "enabled";
const KIND: &str = "kind";
const THRESHOLD_DB: &str = "threshold_db";
const HYSTERESIS_DB: &str = "hysteresis_db";

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
        let input = i.as_iq().unwrap();
        // A shift of nothing is the head of every unzoomed chain, and a
        // rotating phasor multiplied through 16 MS/s of it cost a tenth of
        // real time to change no sample.
        if self.shift_hz == 0.0 {
            o.iq_mut().extend_from_slice(input);
        } else {
            self.mixer.process(input, o.iq_mut());
        }
        Ok(())
    }

    fn reset(&mut self) {
        self.mixer.reset();
    }

    fn params(&self) -> Vec<Param> {
        vec![Param::float(SHIFT_HZ, self.shift_hz, -30e6..=30e6)
            .unit("Hz")
            .label("Frequency shift")]
    }

    fn set_param(&mut self, name: &str, v: ParamValue) -> Result<()> {
        match name {
            SHIFT_HZ => {
                self.shift_hz =
                    v.as_f64().ok_or_else(|| common::Error::other("expected a number"))?;
                Ok(())
            }
            _ => Err(common::Error::other(format!("mixer: unknown parameter {name:?}"))),
        }
    }
}

/// Passband edge as a fraction of the output Nyquist, when nothing has said
/// where it belongs in hertz.
const PASSBAND_FRACTION: f64 = 0.9;

/// Stopband attenuation of the decimating filter, in dB.
const STOPBAND_DB: f64 = 80.0;

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
            passband: PASSBAND_FRACTION,
            atten_db: STOPBAND_DB,
            dec: FirDecim::design(factor.max(1), PASSBAND_FRACTION, STOPBAND_DB),
        }
    }
}

impl Simple for DecimateNode {
    fn name(&self) -> &str {
        "decimate"
    }

    /// A passband is designed rather than set, so it is not something one
    /// number can carry: the design needs the rate it is being cut from.
    fn configure(&mut self, settings: &Settings) {
        let pb = settings.f64_or("passband_hz", 0.0);
        let rate = settings.f64_or("input_rate_hz", 0.0);
        if pb > 0.0 && rate > 0.0 {
            self.set_passband_hz(rate, pb);
        }
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
        if self.factor == 1 {
            return 0;
        }
        self.dec.latency() as u64
    }

    fn process(&mut self, i: &Payload, o: &mut Payload, _c: &mut NodeCtx<'_>) -> Result<()> {
        let input = i.as_iq().unwrap();
        // By one is the head of every unzoomed chain. Designed as a filter
        // it is a hundred-odd taps at the full rate that keep every sample
        // as it was: 45% of real time at 16 MS/s, measured, for nothing.
        if self.factor == 1 {
            o.iq_mut().extend_from_slice(input);
        } else {
            self.dec.process(input, o.iq_mut());
        }
        Ok(())
    }

    fn params(&self) -> Vec<Param> {
        vec![
            Param::int(FACTOR, self.factor as i64, 1..=1024).label("Decimation").affects_rate(),
            Param::float(PASSBAND, self.passband, 0.5..=0.99).label("Passband fraction"),
            Param::float(ATTEN_DB, self.atten_db, 30.0..=120.0)
                .unit("dB")
                .label("Stopband attenuation"),
        ]
    }

    fn set_param(&mut self, name: &str, v: ParamValue) -> Result<()> {
        match name {
            FACTOR => {
                self.factor = v.as_i64().unwrap_or(1).max(1) as usize;
                Ok(())
            }
            PASSBAND => {
                self.passband = v.as_f64().unwrap_or(PASSBAND_FRACTION).clamp(0.1, 0.99);
                Ok(())
            }
            ATTEN_DB => {
                self.atten_db = v.as_f64().unwrap_or(STOPBAND_DB).clamp(20.0, 150.0);
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

/// Peak deviation of broadcast FM, which is what a discriminator with
/// nothing else to go on is scaled for.
pub const WIDE_DEVIATION_HZ: f64 = 75_000.0;

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
        Self::new(WIDE_DEVIATION_HZ)
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
        vec![Param::float(DEVIATION_HZ, self.deviation_hz, 500.0..=200_000.0)
            .unit("Hz")
            .label("Peak deviation")
            .log()]
    }

    fn set_param(&mut self, name: &str, v: ParamValue) -> Result<()> {
        match name {
            DEVIATION_HZ => {
                self.deviation_hz = v.as_f64().unwrap_or(WIDE_DEVIATION_HZ).max(1.0);
                Ok(())
            }
            _ => Err(common::Error::other(format!("fm_demod: unknown parameter {name:?}"))),
        }
    }
}

/// The pre-emphasis time constant used in Europe, in microseconds. The
/// Americas use 75.
pub const EUROPE_TAU_US: f64 = 50.0;

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
        vec![Param::float(TAU_US, self.tau_us, 25.0..=100.0)
            .unit("us")
            .label("Time constant (50 EU, 75 Americas)")]
    }

    fn set_param(&mut self, name: &str, v: ParamValue) -> Result<()> {
        match name {
            TAU_US => {
                self.tau_us = v.as_f64().unwrap_or(EUROPE_TAU_US).clamp(1.0, 1000.0);
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
            passband: PASSBAND_FRACTION,
            dec: vec![FirDecim::design(factor.max(1), PASSBAND_FRACTION, STOPBAND_DB)],
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
            (0..ch).map(|_| FirDecim::design(self.factor, self.passband, STOPBAND_DB)).collect();
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
            self.scratch.extend((0..frames).map(|k| C32::new(src[k * ch + c], 0.0)));
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
        vec![Param::int(FACTOR, self.factor as i64, 1..=1024).label("Decimation").affects_rate()]
    }

    fn set_param(&mut self, name: &str, v: ParamValue) -> Result<()> {
        match name {
            FACTOR => {
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
        for t in c.in_tags(0) {
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

/// The speech passband a sideband receiver filters to, in hertz.
pub const VOICE_LOW_HZ: f64 = 300.0;
pub const VOICE_HIGH_HZ: f64 = 2_700.0;

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
        Self::new(sideband, VOICE_LOW_HZ, VOICE_HIGH_HZ)
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
            Param::float(LOW_HZ, self.low_hz, 50.0..=3_000.0).unit("Hz").label("Filter low edge"),
            Param::float(HIGH_HZ, self.high_hz, 100.0..=6_000.0)
                .unit("Hz")
                .label("Filter high edge"),
        ]
    }

    fn set_param(&mut self, name: &str, v: ParamValue) -> Result<()> {
        match name {
            LOW_HZ => self.low_hz = v.as_f64().unwrap_or(VOICE_LOW_HZ),
            HIGH_HZ => self.high_hz = v.as_f64().unwrap_or(VOICE_HIGH_HZ),
            _ => {
                return Err(common::Error::other(format!("ssb_demod: unknown parameter {name:?}")))
            }
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

/// How much gain the control will apply before it stops, in dB.
const DEFAULT_MAX_GAIN_DB: f64 = 60.0;

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
            max_gain_db: DEFAULT_MAX_GAIN_DB as f32,
            enabled: true,
            agc: Agc::new(48_000.0, attack_ms, release_ms, hang_ms),
        }
    }

    /// The times a mode asks for by name.
    pub fn preset(p: AgcPreset) -> Self {
        let (attack, release, hang) = p.times();
        Self::new(attack, release, hang)
    }
}

/// The gain behaviour a mode asks for by name, so a derived chain does not
/// have to restate three time constants to say "the one for speech".
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AgcPreset {
    /// Speech: fast enough to catch a syllable, slow enough not to pump.
    Voice,
    /// Morse: faster still, and a long hang so the gain does not ride up
    /// between characters.
    Cw,
}

impl AgcPreset {
    /// Attack, release and hang, in milliseconds.
    pub fn times(self) -> (f64, f64, f64) {
        match self {
            Self::Voice => (5.0, 500.0, 300.0),
            Self::Cw => (2.0, 1_000.0, 500.0),
        }
    }

    /// What a setting or a menu calls it, and what [`FromStr`] reads back.
    pub fn label(self) -> &'static str {
        match self {
            Self::Voice => "voice",
            Self::Cw => "cw",
        }
    }
}

impl std::str::FromStr for AgcPreset {
    type Err = common::Error;

    fn from_str(s: &str) -> Result<Self> {
        match s.trim().to_ascii_lowercase().as_str() {
            "voice" => Ok(Self::Voice),
            "cw" => Ok(Self::Cw),
            other => Err(common::Error::other(format!("no gain preset called {other:?}"))),
        }
    }
}

impl std::fmt::Display for AgcPreset {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(self.label())
    }
}

impl AgcNode {
    /// Gain currently applied, or 0 dB when switched off.
    pub fn gain_db(&self) -> f32 {
        if self.enabled {
            self.agc.gain_db()
        } else {
            0.0
        }
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
            Param::float(ATTACK_MS, self.attack_ms, 0.5..=50.0).unit("ms").label("Attack"),
            Param::float(RELEASE_MS, self.release_ms, 50.0..=5_000.0)
                .unit("ms")
                .label("Release")
                .log(),
            Param::float(HANG_MS, self.hang_ms, 0.0..=2_000.0).unit("ms").label("Hang"),
            Param::float(MAX_GAIN_DB, self.max_gain_db as f64, 0.0..=90.0)
                .unit("dB")
                .label("Maximum gain"),
            Param::bool(ENABLED, self.enabled).label("Enabled"),
        ]
    }

    fn set_param(&mut self, name: &str, v: ParamValue) -> Result<()> {
        let (attack, release, hang) = AgcPreset::Voice.times();
        match name {
            ATTACK_MS => self.attack_ms = v.as_f64().unwrap_or(attack),
            RELEASE_MS => self.release_ms = v.as_f64().unwrap_or(release),
            HANG_MS => self.hang_ms = v.as_f64().unwrap_or(hang),
            MAX_GAIN_DB => {
                self.max_gain_db = v.as_f64().unwrap_or(DEFAULT_MAX_GAIN_DB) as f32;
                self.agc.set_max_gain_db(self.max_gain_db);
                return Ok(());
            }
            ENABLED => {
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

impl SquelchKind {
    /// What a setting or a menu calls it, and what [`FromStr`] reads back.
    pub fn label(self) -> &'static str {
        match self {
            Self::Noise => "noise",
            Self::Level => "level",
        }
    }
}

impl std::str::FromStr for SquelchKind {
    type Err = common::Error;

    fn from_str(s: &str) -> Result<Self> {
        match s.trim().to_ascii_lowercase().as_str() {
            "noise" => Ok(Self::Noise),
            "level" => Ok(Self::Level),
            other => Err(common::Error::other(format!("no squelch measurement called {other:?}"))),
        }
    }
}

impl std::fmt::Display for SquelchKind {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(self.label())
    }
}

/// Where a squelch opens by default, in dB on whatever it measures: the
/// level at which narrowband FM speech becomes intelligible.
pub const DEFAULT_SQUELCH_DB: f32 = 9.0;

/// How far the level has to fall below the threshold before the mute closes
/// again, in dB.
const DEFAULT_HYSTERESIS_DB: f64 = 3.0;

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
            hysteresis_db: DEFAULT_HYSTERESIS_DB as f32,
            squelch: Squelch::new(
                48_000.0,
                threshold_db,
                threshold_db - DEFAULT_HYSTERESIS_DB as f32,
                RAMP_MS,
            ),
            meter: NoiseMeter::new(48_000.0, 4_000.0),
            open: false,
            measured: -120.0,
        }
    }

    /// Narrowband FM, at the level where a signal becomes intelligible.
    pub fn fm() -> Self {
        Self::new(SquelchKind::Noise, DEFAULT_SQUELCH_DB)
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
            Param::float(THRESHOLD_DB, self.threshold_db as f64, -100.0..=40.0)
                .unit("dB")
                .label("Threshold"),
            Param::float(HYSTERESIS_DB, self.hysteresis_db as f64, 0.0..=20.0)
                .unit("dB")
                .label("Hysteresis"),
        ]
    }

    fn set_param(&mut self, name: &str, v: ParamValue) -> Result<()> {
        match name {
            THRESHOLD_DB => {
                self.threshold_db = v.as_f64().unwrap_or(DEFAULT_SQUELCH_DB as f64) as f32
            }
            HYSTERESIS_DB => {
                self.hysteresis_db = v.as_f64().unwrap_or(DEFAULT_HYSTERESIS_DB) as f32
            }
            _ => return Err(common::Error::other(format!("squelch: unknown parameter {name:?}"))),
        }
        self.squelch.set_thresholds(self.threshold_db, self.threshold_db - self.hysteresis_db);
        Ok(())
    }
}

/// What the registry knows about these stages, and how it builds one.
///
/// The description sits beside the node it describes so that a default is
/// spelled once: the registry used to restate the discriminator's deviation
/// and the de-emphasis time constant, and either could be changed here
/// without the other following.
pub const MIXER: StageDesc = StageDesc {
    name: "mixer",
    summary: "Shift the signal in frequency, to bring an off-centre \
              carrier to baseband and away from the DC spur",
    category: Category::Filter,
    feeds_bus: false,
};

pub fn build_mixer(s: &Settings) -> Result<Box<dyn Node>> {
    Ok(Box::new(MixerNode::new(s.f64_or(SHIFT_HZ, 0.0))))
}

pub const DECIMATE: StageDesc = StageDesc {
    name: "decimate",
    summary: "Lowpass and reduce the sample rate of an IQ stream",
    category: Category::Filter,
    feeds_bus: false,
};

pub fn build_decimate(s: &Settings) -> Result<Box<dyn Node>> {
    Ok(Box::new(DecimateNode::new(s.i64_or(FACTOR, 1).max(1) as usize)))
}

pub const REAL_DECIMATE: StageDesc = StageDesc {
    name: "real_decimate",
    summary: "Reduce the sample rate of a real stream, for audio",
    category: Category::Filter,
    feeds_bus: false,
};

pub fn build_real_decimate(s: &Settings) -> Result<Box<dyn Node>> {
    Ok(Box::new(RealDecimateNode::new(s.i64_or(FACTOR, 1).max(1) as usize)))
}

pub const ENVELOPE: StageDesc = StageDesc {
    name: "envelope",
    summary: "Complex magnitude; the input an OOK pulse detector needs",
    category: Category::Demod,
    feeds_bus: false,
};

pub fn build_envelope(_s: &Settings) -> Result<Box<dyn Node>> {
    Ok(Box::new(EnvelopeNode))
}

pub const FM_DEMOD: StageDesc = StageDesc {
    name: "fm_demod",
    summary: "Quadrature frequency discriminator, for FM and FSK",
    category: Category::Demod,
    feeds_bus: false,
};

pub fn build_fm_demod(s: &Settings) -> Result<Box<dyn Node>> {
    Ok(Box::new(FmDemodNode::new(s.f64_or(DEVIATION_HZ, WIDE_DEVIATION_HZ))))
}

pub const DEEMPHASIS: StageDesc = StageDesc {
    name: "deemphasis",
    summary: "Undo broadcast FM pre-emphasis (50 us in Europe, 75 in the Americas)",
    category: Category::Filter,
    feeds_bus: false,
};

pub fn build_deemphasis(s: &Settings) -> Result<Box<dyn Node>> {
    Ok(Box::new(DeemphasisNode::new(s.f64_or(TAU_US, EUROPE_TAU_US))))
}

pub const SSB_DEMOD: StageDesc = StageDesc {
    name: "ssb_demod",
    summary: "Demodulate one sideband, or a narrow slice of it for CW",
    category: Category::Demod,
    feeds_bus: false,
};

pub fn build_ssb_demod(s: &Settings) -> Result<Box<dyn Node>> {
    let sideband = s.str_or(SIDEBAND, Sideband::Upper.label()).parse().unwrap_or(Sideband::Upper);
    // A CW filter is the same stage with its passband put around the pitch
    // the operator hears rather than around speech.
    if s.get(PITCH_HZ).is_some() {
        return Ok(Box::new(SsbDemodNode::cw(
            sideband,
            s.f64_or(PITCH_HZ, 700.0),
            s.f64_or(WIDTH_HZ, 500.0),
        )));
    }
    Ok(Box::new(SsbDemodNode::new(
        sideband,
        s.f64_or(LOW_HZ, VOICE_LOW_HZ),
        s.f64_or(HIGH_HZ, VOICE_HIGH_HZ),
    )))
}

pub const HIGH_BLEND: StageDesc = StageDesc {
    name: "high_blend",
    summary: "Roll the top off audio in proportion to the noise on it, \
              so a weak channel hisses less",
    category: Category::Audio,
    feeds_bus: false,
};

pub fn build_high_blend(_s: &Settings) -> Result<Box<dyn Node>> {
    Ok(Box::new(HighBlendNode::new()))
}

pub const AGC: StageDesc = StageDesc {
    name: "agc",
    summary: "Hold audio at a usable level without riding the volume control",
    category: Category::Audio,
    feeds_bus: false,
};

pub fn build_agc(s: &Settings) -> Result<Box<dyn Node>> {
    let (attack, release, hang) = AgcPreset::Voice.times();
    let mut n = match s.str_or(PRESET, "").parse::<AgcPreset>() {
        Ok(p) => AgcNode::preset(p),
        Err(_) => AgcNode::new(
            s.f64_or(ATTACK_MS, attack),
            s.f64_or(RELEASE_MS, release),
            s.f64_or(HANG_MS, hang),
        ),
    };
    if let Some(v) = s.get(MAX_GAIN_DB) {
        Node::set_param(&mut n, MAX_GAIN_DB, v.clone())?;
    }
    Ok(Box::new(n))
}

pub const SQUELCH: StageDesc = StageDesc {
    name: "squelch",
    summary: "Mute a channel with nothing on it, by noise for FM or by level",
    category: Category::Audio,
    feeds_bus: false,
};

pub fn build_squelch(s: &Settings) -> Result<Box<dyn Node>> {
    let kind = s.str_or(KIND, SquelchKind::Noise.label()).parse().unwrap_or(SquelchKind::Noise);
    Ok(Box::new(SquelchNode::new(kind, s.f64_or(THRESHOLD_DB, DEFAULT_SQUELCH_DB as f64) as f32)))
}
