//! Modulators: baseband in, transmit IQ out.
//!
//! One node per way of putting something on a carrier, and none of them knows
//! what it is carrying. A keyed protocol hands [`OokModNode`] or
//! [`FskModNode`] the mark and gap timings its table produced; speech or tone
//! hands [`AmModNode`] or [`FmModNode`] real samples. That is the transmit
//! side of the same split the receiver has, where a demodulator produces
//! audio and knows nothing about what is being said on it.
//!
//! Two rules every one of them follows.
//!
//! Phase is continuous across blocks and across bursts. A modulator that
//! restarts its oscillator puts a step in the signal at every block boundary,
//! and the spectrum of a step is the whole band.
//!
//! Amplitude defaults to a quarter of full scale. The converter clips at one,
//! and a clipped carrier splatters across the band instead of staying in the
//! channel it was tuned to.

use common::{Package, Result, C32};
use pipeline::node::{NodeCtx, PortSpec, Simple};
use pipeline::param::{Param, ParamValue};
use pipeline::port::{Domain, Flow, Payload, PortKind, StreamSpec, TAG_TX_END, TAG_TX_START};
use pipeline::Tag;

/// Where a modulator's own carrier sits, and how loud, since every one of
/// them has these two.
fn tx_spec(input: &PortSpec, rate: f64, bandwidth: f64) -> StreamSpec {
    StreamSpec {
        kind: PortKind::Iq,
        rate,
        center: input.spec.center,
        bandwidth: bandwidth.min(rate),
        channels: 1,
        flow: Flow::Tx,
        domain: Domain::Baseband,
    }
}

/// Rise from 0 to 1 over `x` in [0, 1] with zero slope at both ends.
fn raised_cosine(x: f32) -> f32 {
    0.5 - 0.5 * (std::f32::consts::PI * x.clamp(0.0, 1.0)).cos()
}

/// A carrier that keeps its phase across calls.
///
/// Every modulator here holds one. Frequency is set per sample, so the same
/// oscillator serves a fixed offset, an FSK tone pair and an FM deviation.
#[derive(Clone, Copy, Debug, Default)]
pub struct Carrier {
    phase: f64,
}

impl Carrier {
    /// One sample at `hz` from the stream centre, at unit amplitude.
    pub fn step(&mut self, hz: f64, rate: f64) -> C32 {
        let (s, c) = self.phase.sin_cos();
        self.phase += std::f64::consts::TAU * hz / rate;
        if self.phase > std::f64::consts::TAU {
            self.phase -= std::f64::consts::TAU;
        } else if self.phase < -std::f64::consts::TAU {
            self.phase += std::f64::consts::TAU;
        }
        C32::new(c as f32, s as f32)
    }

    pub fn reset(&mut self) {
        self.phase = 0.0;
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
    carrier: Carrier,
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
            carrier: Carrier::default(),
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
                let c = self.carrier.step(self.offset_hz, self.rate);
                out.push(c * (env * self.amplitude));
            }
            for _ in 0..gap {
                // The oscillator keeps running through the gap so the next
                // mark starts where an unbroken carrier would have been.
                self.carrier.step(self.offset_hz, self.rate);
                out.push(C32::new(0.0, 0.0));
            }
        }
    }

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
        self.carrier.reset();
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


/// Timings in, two tones out.
///
/// The transmit side of `dsp::fsk`, and it inherits that side's convention: a
/// mark is the upper tone and a gap the lower one, which is all
/// [`PortKind::Pulses`] can say. Anything with more than two levels needs a
/// port that carries symbols rather than durations.
///
/// Phase is continuous across the tone change, which is what makes this CPFSK
/// rather than two oscillators switched between. Switching puts a step at
/// every symbol boundary, and a receiver sees that as a click in every slot.
pub struct FskModNode {
    offset_hz: f64,
    shift_hz: f64,
    amplitude: f32,
    rate: f64,
    carrier: Carrier,
    produced: u64,
}

impl Default for FskModNode {
    fn default() -> Self {
        Self {
            offset_hz: 0.0,
            // The separation most ISM sensors use, and wide enough that a
            // discriminator reads it without a narrow filter.
            shift_hz: 50_000.0,
            amplitude: 0.25,
            rate: 0.0,
            carrier: Carrier::default(),
            produced: 0,
        }
    }
}

impl FskModNode {
    pub fn new(offset_hz: f64, shift_hz: f64, amplitude: f32) -> Self {
        Self {
            offset_hz,
            shift_hz: shift_hz.abs(),
            amplitude: amplitude.clamp(0.0, 1.0),
            ..Self::default()
        }
    }

    fn key(&mut self, pkg: &Package, out: &mut Vec<C32>) {
        let per_us = self.rate / 1e6;
        let (hi, lo) = (
            self.offset_hz + self.shift_hz / 2.0,
            self.offset_hz - self.shift_hz / 2.0,
        );
        for p in &pkg.pulses {
            let mark = ((p.mark as f64 * per_us).round() as usize).max(1);
            let gap = (p.gap as f64 * per_us).round() as usize;
            for _ in 0..mark {
                let c = self.carrier.step(hi, self.rate);
                out.push(c * self.amplitude);
            }
            for _ in 0..gap {
                let c = self.carrier.step(lo, self.rate);
                out.push(c * self.amplitude);
            }
        }
    }
}

impl Simple for FskModNode {
    fn name(&self) -> &str {
        "fsk_mod"
    }

    fn negotiate(&mut self, input: &PortSpec) -> Result<StreamSpec> {
        if input.spec.kind != PortKind::Pulses {
            return Err(common::Error::other("fsk_mod takes pulse timings"));
        }
        if input.spec.rate <= 0.0 {
            return Err(common::Error::other("fsk_mod needs the rate it should key at"));
        }
        if self.shift_hz >= input.spec.rate {
            return Err(common::Error::other(format!(
                "fsk_mod: a {} Hz shift does not fit in a {} Hz stream",
                self.shift_hz, input.spec.rate
            )));
        }
        self.rate = input.spec.rate;
        // Carson's rule with the symbol rate unknown: the tones plus a symbol
        // either side. The tone separation is the part that is known.
        Ok(tx_spec(input, self.rate, self.shift_hz * 2.0))
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
            ctx.tag(Tag::marker(start, TAG_TX_START));
            ctx.tag(Tag::marker(end.saturating_sub(1), TAG_TX_END));
        }
        self.produced += out.len() as u64;
        Ok(())
    }

    fn reset(&mut self) {
        self.carrier.reset();
        self.produced = 0;
    }

    fn params(&self) -> Vec<Param> {
        vec![
            Param::float("offset_hz", self.offset_hz, -1e6..=1e6)
                .label("Carrier offset")
                .unit("Hz"),
            Param::float("shift_hz", self.shift_hz, 100.0..=500_000.0)
                .label("Tone separation")
                .unit("Hz"),
            Param::float("amplitude", self.amplitude as f64, 0.0..=1.0)
                .label("Amplitude")
                .unit("FS"),
        ]
    }

    fn set_param(&mut self, name: &str, value: ParamValue) -> Result<()> {
        let v = value.as_f64().unwrap_or(0.0);
        match name {
            "offset_hz" => self.offset_hz = v,
            "shift_hz" => self.shift_hz = v.abs(),
            "amplitude" => self.amplitude = v.clamp(0.0, 1.0) as f32,
            _ => return Err(common::Error::other(format!("fsk_mod: unknown parameter {name:?}"))),
        }
        Ok(())
    }
}

/// Symbol amplitudes in, a carrier that follows them out.
///
/// The general case [`OokModNode`] is the two-level special case of: each
/// input sample is one symbol's amplitude between zero and one, held for
/// `sps` output samples, with a raised cosine transition between neighbours.
/// Multi-level ASK is what a protocol with more than on and off needs, and
/// what the timings port cannot express.
///
/// Transitions are shaped for the same reason the keyed edges are, and the
/// shape spans the boundary between symbols rather than sitting inside one,
/// so a run of equal symbols is a steady carrier.
pub struct AskModNode {
    offset_hz: f64,
    amplitude: f32,
    sps: usize,
    /// Fraction of a symbol the transition takes.
    transition: f32,
    rate: f64,
    carrier: Carrier,
    /// The level the last symbol ended on, so a block joins the one before.
    last: f32,
}

impl Default for AskModNode {
    fn default() -> Self {
        Self {
            offset_hz: 0.0,
            amplitude: 0.25,
            sps: 10,
            transition: 0.25,
            rate: 0.0,
            carrier: Carrier::default(),
            last: 0.0,
        }
    }
}

impl AskModNode {
    pub fn new(offset_hz: f64, amplitude: f32, sps: usize) -> Self {
        Self {
            offset_hz,
            amplitude: amplitude.clamp(0.0, 1.0),
            sps: sps.max(1),
            ..Self::default()
        }
    }
}

impl Simple for AskModNode {
    fn name(&self) -> &str {
        "ask_mod"
    }

    fn negotiate(&mut self, input: &PortSpec) -> Result<StreamSpec> {
        if !matches!(input.spec.kind, PortKind::Soft | PortKind::Real) {
            return Err(common::Error::other("ask_mod takes one amplitude per symbol"));
        }
        if input.spec.rate <= 0.0 {
            return Err(common::Error::other("ask_mod needs a symbol rate"));
        }
        self.rate = input.spec.rate * self.sps as f64;
        // Two symbol rates wide, which is what a shaped ASK signal occupies.
        Ok(tx_spec(input, self.rate, input.spec.rate * 2.0))
    }

    fn process(
        &mut self,
        input: &Payload,
        output: &mut Payload,
        _ctx: &mut NodeCtx<'_>,
    ) -> Result<()> {
        let Some(sym) = input.as_real() else { return Ok(()) };
        let out = output.iq_mut();
        let tr = ((self.sps as f32 * self.transition).round() as usize).clamp(0, self.sps);
        for &s in sym {
            let level = s.clamp(0.0, 1.0);
            for i in 0..self.sps {
                let env = if i < tr && tr > 0 {
                    let x = raised_cosine((i as f32 + 0.5) / tr as f32);
                    self.last + (level - self.last) * x
                } else {
                    level
                };
                let c = self.carrier.step(self.offset_hz, self.rate);
                out.push(c * (env * self.amplitude));
            }
            self.last = level;
        }
        Ok(())
    }

    fn reset(&mut self) {
        self.carrier.reset();
        self.last = 0.0;
    }

    fn params(&self) -> Vec<Param> {
        vec![
            Param::float("offset_hz", self.offset_hz, -1e6..=1e6)
                .label("Carrier offset")
                .unit("Hz"),
            Param::float("amplitude", self.amplitude as f64, 0.0..=1.0)
                .label("Amplitude")
                .unit("FS"),
            Param::int("sps", self.sps as i64, 1..=1024)
                .label("Samples per symbol"),
            Param::float("transition", self.transition as f64, 0.0..=1.0)
                .label("Transition")
                .unit("symbol"),
        ]
    }

    fn set_param(&mut self, name: &str, value: ParamValue) -> Result<()> {
        match name {
            "offset_hz" => self.offset_hz = value.as_f64().unwrap_or(0.0),
            "amplitude" => {
                self.amplitude = value.as_f64().unwrap_or(0.25).clamp(0.0, 1.0) as f32
            }
            "sps" => self.sps = value.as_i64().unwrap_or(10).max(1) as usize,
            "transition" => {
                self.transition = value.as_f64().unwrap_or(0.25).clamp(0.0, 1.0) as f32
            }
            _ => return Err(common::Error::other(format!("ask_mod: unknown parameter {name:?}"))),
        }
        Ok(())
    }
}

/// Audio in, amplitude modulated carrier out.
///
/// `depth` is the modulation index: at 1.0 the envelope reaches zero on the
/// negative peaks of full-scale audio, which is 100% modulation. Past that
/// the envelope would go negative, which in a real transmitter is
/// overmodulation and splatter, so the envelope is clamped at zero and the
/// input should be limited before it gets here.
///
/// The carrier is left in, because this is the AM that a broadcast or airband
/// receiver expects: an envelope detector needs it. Suppressing it is a
/// different modulation with a different demodulator.
pub struct AmModNode {
    offset_hz: f64,
    depth: f32,
    amplitude: f32,
    rate: f64,
    carrier: Carrier,
}

impl Default for AmModNode {
    fn default() -> Self {
        Self {
            offset_hz: 0.0,
            depth: 0.8,
            amplitude: 0.25,
            rate: 0.0,
            carrier: Carrier::default(),
        }
    }
}

impl AmModNode {
    pub fn new(offset_hz: f64, depth: f32, amplitude: f32) -> Self {
        Self {
            offset_hz,
            depth: depth.clamp(0.0, 1.0),
            amplitude: amplitude.clamp(0.0, 1.0),
            ..Self::default()
        }
    }
}

impl Simple for AmModNode {
    fn name(&self) -> &str {
        "am_mod"
    }

    fn negotiate(&mut self, input: &PortSpec) -> Result<StreamSpec> {
        if input.spec.kind != PortKind::Real {
            return Err(common::Error::other("am_mod takes real audio"));
        }
        if input.spec.channels != 1 {
            return Err(common::Error::other("am_mod takes one channel"));
        }
        if input.spec.rate <= 0.0 {
            return Err(common::Error::other("am_mod needs an audio rate"));
        }
        self.rate = input.spec.rate;
        // Both sidebands, so twice the highest audio frequency, which is at
        // most half the audio rate.
        Ok(tx_spec(input, self.rate, input.spec.rate))
    }

    fn process(
        &mut self,
        input: &Payload,
        output: &mut Payload,
        _ctx: &mut NodeCtx<'_>,
    ) -> Result<()> {
        let Some(audio) = input.as_real() else { return Ok(()) };
        let out = output.iq_mut();
        for &a in audio {
            let env = (1.0 + self.depth * a.clamp(-1.0, 1.0)).max(0.0);
            let c = self.carrier.step(self.offset_hz, self.rate);
            // Divided by what the peak of a fully modulated envelope reaches,
            // so `amplitude` means the same full-scale fraction here as it
            // does on every other modulator.
            out.push(c * (env * self.amplitude / (1.0 + self.depth)));
        }
        Ok(())
    }

    fn reset(&mut self) {
        self.carrier.reset();
    }

    fn params(&self) -> Vec<Param> {
        vec![
            Param::float("offset_hz", self.offset_hz, -1e6..=1e6)
                .label("Carrier offset")
                .unit("Hz"),
            Param::float("depth", self.depth as f64, 0.0..=1.0).label("Modulation depth"),
            Param::float("amplitude", self.amplitude as f64, 0.0..=1.0)
                .label("Amplitude")
                .unit("FS"),
        ]
    }

    fn set_param(&mut self, name: &str, value: ParamValue) -> Result<()> {
        let v = value.as_f64().unwrap_or(0.0);
        match name {
            "offset_hz" => self.offset_hz = v,
            "depth" => self.depth = v.clamp(0.0, 1.0) as f32,
            "amplitude" => self.amplitude = v.clamp(0.0, 1.0) as f32,
            _ => return Err(common::Error::other(format!("am_mod: unknown parameter {name:?}"))),
        }
        Ok(())
    }
}

/// Narrowband FM: 2.5 kHz deviation, the European 12.5 kHz channel standard.
pub const NBFM_DEVIATION_HZ: f64 = 2_500.0;

/// The 25 kHz channel version, and what most amateur and business radio uses.
pub const FM_DEVIATION_HZ: f64 = 5_000.0;

/// Broadcast FM, 75 kHz deviation on a 200 kHz channel.
pub const WBFM_DEVIATION_HZ: f64 = 75_000.0;

/// Audio in, frequency modulated carrier out.
///
/// Constant envelope, so `amplitude` is the whole signal level and there is
/// nothing to clip: this is the modulation a class C amplifier wants.
///
/// The deviation is what separates narrowband from wide, and there is no
/// other difference in the modulator: `nbfm` is 2.5 kHz, `fm` 5 kHz and
/// `wbfm` 75 kHz. What a real broadcast transmitter adds on top is
/// pre-emphasis and a 15 kHz audio limit, and neither belongs here: they are
/// stages on the audio before it arrives, so they can be seen in the chain
/// and switched for the region.
pub struct FmModNode {
    offset_hz: f64,
    deviation_hz: f64,
    amplitude: f32,
    rate: f64,
    carrier: Carrier,
}

impl Default for FmModNode {
    fn default() -> Self {
        Self {
            offset_hz: 0.0,
            deviation_hz: FM_DEVIATION_HZ,
            amplitude: 0.25,
            rate: 0.0,
            carrier: Carrier::default(),
        }
    }
}

impl FmModNode {
    pub fn new(offset_hz: f64, deviation_hz: f64, amplitude: f32) -> Self {
        Self {
            offset_hz,
            deviation_hz: deviation_hz.abs(),
            amplitude: amplitude.clamp(0.0, 1.0),
            ..Self::default()
        }
    }

    pub fn narrowband(offset_hz: f64) -> Self {
        Self::new(offset_hz, NBFM_DEVIATION_HZ, 0.25)
    }

    pub fn wideband(offset_hz: f64) -> Self {
        Self::new(offset_hz, WBFM_DEVIATION_HZ, 0.25)
    }
}

impl Simple for FmModNode {
    fn name(&self) -> &str {
        "fm_mod"
    }

    fn negotiate(&mut self, input: &PortSpec) -> Result<StreamSpec> {
        if input.spec.kind != PortKind::Real {
            return Err(common::Error::other("fm_mod takes real audio"));
        }
        if input.spec.channels != 1 {
            return Err(common::Error::other(
                "fm_mod takes one channel; stereo needs the multiplex built first",
            ));
        }
        if input.spec.rate <= 0.0 {
            return Err(common::Error::other("fm_mod needs an audio rate"));
        }
        // Carson's rule: twice the deviation plus twice the highest audio
        // frequency. Deviating further than the stream can carry folds the
        // signal back on itself.
        //
        // The audio's own bandwidth is only counted when the stream says what
        // it is. Assuming it fills Nyquist would refuse every ordinary case,
        // since speech at a 96 kHz sample rate occupies 3 kHz of it and not
        // 48, and a modulator is not the place to guess.
        let audio_max = match input.spec.bandwidth > 0.0 && input.spec.bandwidth < input.spec.rate {
            true => input.spec.bandwidth / 2.0,
            false => 0.0,
        };
        let occupied = 2.0 * (self.deviation_hz + audio_max);
        if occupied > input.spec.rate {
            return Err(common::Error::other(format!(
                "fm_mod: {} Hz deviation needs {:.0} Hz of stream, not {:.0}",
                self.deviation_hz, occupied, input.spec.rate
            )));
        }
        self.rate = input.spec.rate;
        Ok(tx_spec(input, self.rate, occupied))
    }

    fn process(
        &mut self,
        input: &Payload,
        output: &mut Payload,
        _ctx: &mut NodeCtx<'_>,
    ) -> Result<()> {
        let Some(audio) = input.as_real() else { return Ok(()) };
        let out = output.iq_mut();
        for &a in audio {
            let hz = self.offset_hz + self.deviation_hz * a.clamp(-1.0, 1.0) as f64;
            let c = self.carrier.step(hz, self.rate);
            out.push(c * self.amplitude);
        }
        Ok(())
    }

    fn reset(&mut self) {
        self.carrier.reset();
    }

    fn params(&self) -> Vec<Param> {
        vec![
            Param::float("offset_hz", self.offset_hz, -1e6..=1e6)
                .label("Carrier offset")
                .unit("Hz"),
            Param::float("deviation_hz", self.deviation_hz, 100.0..=200_000.0)
                .label("Deviation")
                .unit("Hz"),
            Param::float("amplitude", self.amplitude as f64, 0.0..=1.0)
                .label("Amplitude")
                .unit("FS"),
        ]
    }

    fn set_param(&mut self, name: &str, value: ParamValue) -> Result<()> {
        let v = value.as_f64().unwrap_or(0.0);
        match name {
            "offset_hz" => self.offset_hz = v,
            "deviation_hz" => self.deviation_hz = v.abs(),
            "amplitude" => self.amplitude = v.clamp(0.0, 1.0) as f32,
            _ => return Err(common::Error::other(format!("fm_mod: unknown parameter {name:?}"))),
        }
        Ok(())
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

    fn run<N: Simple>(node: &mut N, input: Payload, spec: StreamSpec) -> Vec<C32> {
        let ins = [PortSpec { spec, latency: 0 }];
        let mut out = Payload::Iq(Vec::new());
        let (mut ev, mut tg) = (Vec::new(), Vec::new());
        let mut ctx = NodeCtx::new(0, &ins, &[], &mut ev, &mut tg);
        node.process(&input, &mut out, &mut ctx).unwrap();
        match out {
            Payload::Iq(v) => v,
            _ => unreachable!(),
        }
    }

    fn audio_spec(rate: f64) -> StreamSpec {
        StreamSpec {
            kind: PortKind::Real,
            rate,
            center: common::Hz(433_920_000),
            bandwidth: rate,
            ..Default::default()
        }
    }

    /// Mean frequency of a stretch of IQ, in Hz.
    fn mean_hz(iq: &[C32], rate: f64) -> f64 {
        let mut turns = 0.0f64;
        for w in iq.windows(2) {
            turns += (w[1] * w[0].conj()).arg() as f64;
        }
        turns / (iq.len() - 1) as f64 / std::f64::consts::TAU * rate
    }

    #[test]
    fn fsk_puts_a_mark_on_one_tone_and_a_gap_on_the_other() {
        let rate = 250_000.0;
        let mut n = FskModNode::new(0.0, 50_000.0, 0.5);
        let spec = StreamSpec {
            kind: PortKind::Pulses,
            rate,
            center: common::Hz(433_920_000),
            bandwidth: rate,
            flow: Flow::Tx,
            ..Default::default()
        };
        n.negotiate(&PortSpec { spec, latency: 0 }).unwrap();
        let pkg = Package {
            pulses: vec![Pulse { mark: 4_000, gap: 4_000 }],
            ..Default::default()
        };
        let iq = run(&mut n, Payload::Pulses(vec![pkg]), spec);
        assert_eq!(iq.len(), 2_000);
        assert!((mean_hz(&iq[100..900], rate) - 25_000.0).abs() < 200.0);
        assert!((mean_hz(&iq[1_100..1_900], rate) + 25_000.0).abs() < 200.0);
        // Constant envelope: an FSK transmitter has nothing to clip.
        for s in &iq {
            assert!((s.norm() - 0.5).abs() < 1e-3, "envelope moved to {}", s.norm());
        }
    }

    #[test]
    fn fsk_keeps_its_phase_across_the_tone_change() {
        // Continuous phase is the difference between CPFSK and two
        // oscillators switched between, and a switch is a click in the
        // channel at every symbol boundary.
        let rate = 250_000.0;
        let mut n = FskModNode::new(0.0, 50_000.0, 0.5);
        let spec = StreamSpec {
            kind: PortKind::Pulses,
            rate,
            center: common::Hz(0),
            bandwidth: rate,
            flow: Flow::Tx,
            ..Default::default()
        };
        n.negotiate(&PortSpec { spec, latency: 0 }).unwrap();
        let pkg = Package { pulses: vec![Pulse { mark: 4_000, gap: 4_000 }], ..Default::default() };
        let iq = run(&mut n, Payload::Pulses(vec![pkg]), spec);
        // Step across the boundary, against the step either side of it.
        let at = 1_000;
        let jump = (iq[at] * iq[at - 1].conj()).arg().abs();
        assert!(jump < 1.0, "phase jumped {jump} rad at the tone change");
    }

    #[test]
    fn a_shift_wider_than_the_stream_is_refused() {
        let mut n = FskModNode::new(0.0, 300_000.0, 0.5);
        let spec = StreamSpec {
            kind: PortKind::Pulses,
            rate: 250_000.0,
            center: common::Hz(0),
            bandwidth: 250_000.0,
            flow: Flow::Tx,
            ..Default::default()
        };
        assert!(n.negotiate(&PortSpec { spec, latency: 0 }).is_err());
    }

    #[test]
    fn ask_holds_each_symbol_at_the_level_it_was_given() {
        let mut n = AskModNode::new(0.0, 1.0, 20);
        let spec = StreamSpec {
            kind: PortKind::Soft,
            rate: 10_000.0,
            center: common::Hz(0),
            bandwidth: 10_000.0,
            flow: Flow::Tx,
            ..Default::default()
        };
        let out = n.negotiate(&PortSpec { spec, latency: 0 }).unwrap();
        assert_eq!(out.rate, 200_000.0, "20 samples a symbol at 10 kBd");

        let levels = vec![0.0f32, 1.0, 0.5, 0.25, 1.0];
        let iq = run(&mut n, Payload::Soft(levels.clone()), spec);
        assert_eq!(iq.len(), levels.len() * 20);
        for (k, want) in levels.iter().enumerate() {
            // The end of each symbol, past the transition into it.
            let got = iq[k * 20 + 19].norm();
            assert!((got - want).abs() < 0.01, "symbol {k} came out at {got}, wanted {want}");
        }
    }

    #[test]
    fn am_envelope_follows_the_audio_and_never_goes_negative() {
        let rate = 96_000.0;
        let mut n = AmModNode::new(0.0, 1.0, 1.0);
        let spec = audio_spec(rate);
        n.negotiate(&PortSpec { spec, latency: 0 }).unwrap();
        // A full-scale tone at 1 kHz, which at 100% modulation should take
        // the envelope from zero to twice the carrier.
        let audio: Vec<f32> = (0..960)
            .map(|i| (std::f32::consts::TAU * 1_000.0 * i as f32 / rate as f32).sin())
            .collect();
        let iq = run(&mut n, Payload::Real(audio.clone()), spec);

        let (mut lo, mut hi) = (f32::MAX, 0.0f32);
        for s in &iq {
            lo = lo.min(s.norm());
            hi = hi.max(s.norm());
        }
        assert!(lo < 0.02, "envelope bottomed out at {lo}, not near zero");
        assert!((hi - 1.0).abs() < 0.02, "envelope peaked at {hi}, not full scale");

        // And an envelope detector reads the tone back.
        let mut det = dsp::demod::AmDemod::new(rate, 50.0);
        let mut back = Vec::new();
        det.process(&iq, &mut back);
        let tail = &back[back.len() / 2..];
        let corr: f32 = tail
            .iter()
            .zip(&audio[audio.len() - tail.len()..])
            .map(|(a, b)| a * b)
            .sum::<f32>()
            / tail.len() as f32;
        assert!(corr > 0.1, "the demodulated audio does not follow what was sent ({corr})");
    }

    #[test]
    fn fm_deviates_by_exactly_what_it_was_asked_for() {
        let rate = 96_000.0;
        for dev in [NBFM_DEVIATION_HZ, FM_DEVIATION_HZ] {
            let mut n = FmModNode::new(0.0, dev, 0.5);
            let spec = audio_spec(rate);
            n.negotiate(&PortSpec { spec, latency: 0 }).unwrap();
            // Held at full scale, so the carrier sits at the deviation.
            let iq = run(&mut n, Payload::Real(vec![1.0; 4_000]), spec);
            let got = mean_hz(&iq, rate);
            assert!((got - dev).abs() < 5.0, "asked {dev} Hz, deviated {got:.0} Hz");
            for s in &iq {
                assert!((s.norm() - 0.5).abs() < 1e-3, "FM is constant envelope");
            }
        }
    }

    #[test]
    fn what_fm_sends_the_receiver_discriminator_reads_back() {
        let rate = 96_000.0;
        let mut n = FmModNode::narrowband(0.0);
        let spec = audio_spec(rate);
        n.negotiate(&PortSpec { spec, latency: 0 }).unwrap();
        let audio: Vec<f32> = (0..4_800)
            .map(|i| 0.8 * (std::f32::consts::TAU * 1_200.0 * i as f32 / rate as f32).sin())
            .collect();
        let iq = run(&mut n, Payload::Real(audio.clone()), spec);

        let mut demod = dsp::FmDemod::new(rate, NBFM_DEVIATION_HZ);
        let mut back = Vec::new();
        demod.process(&iq, &mut back);
        // One sample of delay, and it is the discriminator's: the phase step
        // between samples i and i+1 is the frequency the modulator was given
        // at i, so what comes out at i+1 is the audio at i. Aligned for that,
        // the recovery is exact rather than approximate, which is the point
        // worth asserting: the two agree on the deviation scale as well as
        // on the shape.
        let n = back.len() - 1;
        let rms = (back[1..]
            .iter()
            .zip(&audio[..n])
            .map(|(a, b)| ((a - b) * (a - b)) as f64)
            .sum::<f64>()
            / n as f64)
            .sqrt();
        assert!(rms < 1e-3, "recovered audio is {rms:.4} rms away from what was modulated");
    }

    #[test]
    fn fm_refuses_a_deviation_the_stream_cannot_carry() {
        // Broadcast deviation into a 96 kHz audio stream needs 246 kHz by
        // Carson, so it would fold back on itself rather than transmit.
        let mut n = FmModNode::wideband(0.0);
        let spec = audio_spec(96_000.0);
        let err = n.negotiate(&PortSpec { spec, latency: 0 }).unwrap_err().to_string();
        assert!(err.contains("deviation"), "unhelpful: {err}");
    }
}
