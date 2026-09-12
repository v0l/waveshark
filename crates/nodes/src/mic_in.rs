//! The microphone as a receive-side source.
//!
//! The counterpart of [`crate::MicNode`], which reads the microphone for the
//! transmitter. This puts the same audio into the graph so a decoder can read
//! it: hold a phone playing SSTV to the laptop and the picture builds, with no
//! radio and no licence in the way. Anything else that decodes audio can be
//! wired to it too.
//!
//! It has an input it does not read. The graph is clocked by the radio, so a
//! stage with no input never runs; this takes the head's stream purely to be
//! told how much time has passed, and hands over that much microphone audio.
//!
//! A microphone is its own clock and runs a few parts per million away from
//! the radio's, so over a two minute transmission the two drift by a few
//! milliseconds. That shows up here as a sample repeated or dropped now and
//! then rather than as a growing delay, which is what a decoder that
//! resynchronises on every line can absorb and a bit-timed one cannot.

use common::Result;
use pipeline::node::{NodeCtx, PortSpec, Simple};
use pipeline::port::{Payload, PortKind, StreamSpec};
use pipeline::registry::{Category, Settings, StageDesc};

pub struct MicInNode {
    src: Option<std::sync::Arc<dyn audio::AudioSource>>,
    /// The rate the stage claims, which is the microphone's where there is
    /// one and a plausible one where there is not: a stage waiting for a
    /// device still has to negotiate, or everything wired to it is dropped.
    rate: f64,
    /// Samples per input sample, from negotiation.
    step: f64,
    /// Fraction of a sample carried between blocks, so a ratio that is not a
    /// whole number does not lose a sample a block.
    owed: f64,
    /// Output samples with no microphone audio behind them, which is the
    /// device not keeping up or not being there at all.
    silent: u64,
    peak: f32,
}

/// What the stage reports when no device is open. 48 kHz is what the receiver
/// asks a microphone for.
const IDLE_RATE: f64 = 48_000.0;

impl Default for MicInNode {
    fn default() -> Self {
        Self::idle()
    }
}

impl MicInNode {
    pub fn new(src: std::sync::Arc<dyn audio::AudioSource>) -> Self {
        let rate = src.rate().max(1.0);
        Self { src: Some(src), rate, step: 0.0, owed: 0.0, silent: 0, peak: 0.0 }
    }

    /// A stage with no microphone yet. It produces silence rather than
    /// refusing to build, so the chain behind it survives until a device
    /// arrives.
    pub fn idle() -> Self {
        Self { src: None, rate: IDLE_RATE, step: 0.0, owed: 0.0, silent: 0, peak: 0.0 }
    }

    pub fn is_open(&self) -> bool {
        self.src.is_some()
    }

    /// Peak of the last block, for a meter beside whatever is listening.
    pub fn peak(&self) -> f32 {
        self.peak
    }

    /// Samples handed on that the microphone had nothing behind.
    pub fn silent(&self) -> u64 {
        self.silent
    }
}

impl Simple for MicInNode {
    fn name(&self) -> &str {
        "mic_in"
    }

    fn negotiate(&mut self, input: &PortSpec) -> Result<StreamSpec> {
        if input.spec.rate <= 0.0 {
            return Err(common::Error::other("mic_in needs a stream to pace it"));
        }
        self.step = self.rate / input.spec.rate;
        self.owed = 0.0;
        let mut out = input.spec.with_kind(PortKind::Real);
        out.rate = self.rate;
        out.bandwidth = self.rate / 2.0;
        Ok(out)
    }

    fn process(&mut self, i: &Payload, o: &mut Payload, _c: &mut NodeCtx<'_>) -> Result<()> {
        let n = match i {
            Payload::Iq(v) => v.len(),
            Payload::Real(v) => v.len(),
            _ => return Ok(()),
        };
        if n == 0 {
            return Ok(());
        }
        let exact = n as f64 * self.step + self.owed;
        let want = exact.floor() as usize;
        self.owed = exact - want as f64;

        let out = o.real_mut();
        let before = out.len();
        if let Some(src) = &self.src {
            src.take(out, want);
        }
        let got = out.len() - before;
        // Short is normal at the start and after a hiccup: the ring holds a
        // tenth of a second, and what is missing is silence rather than a
        // gap in time, since the decoder downstream counts samples.
        if got < want {
            self.silent += (want - got) as u64;
            out.resize(before + want, 0.0);
        }
        self.peak = out[before..].iter().fold(0.0f32, |m, v| m.max(v.abs()));
        Ok(())
    }

    fn reset(&mut self) {
        self.owed = 0.0;
    }
}

pub const DESC: StageDesc = StageDesc {
    name: "mic_in",
    summary: "The microphone, as audio for a decoder to read",
    category: Category::Audio,
    feeds_bus: false,
};

/// Built without a device: the receiver hands the microphone in when it has
/// one, the way it does for the transmit side.
pub fn build(_s: &Settings) -> Result<Box<dyn pipeline::node::Node>> {
    Ok(Box::new(MicInNode::idle()))
}

#[cfg(test)]
mod tests {
    use super::*;
    use common::Hz;

    /// A known waveform through the same node the microphone feeds.
    fn canned(rate: f64, n: usize) -> std::sync::Arc<dyn audio::AudioSource> {
        let tone: Vec<f32> = (0..n)
            .map(|i| (std::f32::consts::TAU * 1000.0 * i as f32 / rate as f32).sin())
            .collect();
        std::sync::Arc::new(audio::Canned::new(tone, rate, true))
    }

    fn run(node: &mut MicInNode, blocks: usize, block: usize, rate: f64) -> Vec<f32> {
        let spec = PortSpec { spec: StreamSpec::iq(rate, Hz(0)), latency: 0 };
        node.negotiate(&spec).expect("a stream to pace it");
        let ins = [spec];
        let (tags, mut events, mut new_tags) = (Vec::new(), Vec::new(), Vec::new());
        let mut ctx = NodeCtx::new(0, &ins, &tags, &mut events, &mut new_tags);
        let mut out = Vec::new();
        for _ in 0..blocks {
            let mut o = Payload::Real(Vec::new());
            let iq = Payload::Iq(vec![common::C32::default(); block]);
            node.process(&iq, &mut o, &mut ctx).expect("process");
            out.extend(o.as_real().unwrap_or(&[]).iter().copied());
        }
        out
    }

    /// A block of radio samples buys its own length in time of audio, and the
    /// fraction left over is carried: 16384 samples at 2.048 MS/s is 384 at
    /// 48 kHz exactly, and at 2.4 MS/s it is 327.68, which has to average out
    /// rather than truncate every block.
    #[test]
    fn the_audio_keeps_up_with_the_clock_that_paces_it() {
        let mut n = MicInNode::new(canned(48_000.0, 48_000));
        let got = run(&mut n, 100, 16_384, 2_400_000.0);
        let want = (100.0 * 16_384.0 * 48_000.0 / 2_400_000.0) as usize;
        assert!(got.len().abs_diff(want) <= 1, "{} samples against {want}", got.len());
    }

    /// With no device the stage still builds, negotiates and produces the
    /// silence its consumers expect, rather than taking the chain down.
    #[test]
    fn a_stage_with_no_microphone_produces_silence() {
        let mut n = MicInNode::idle();
        assert!(!n.is_open());
        let got = run(&mut n, 4, 16_384, 2_048_000.0);
        assert_eq!(got.len(), 4 * 384);
        assert!(got.iter().all(|v| *v == 0.0), "silence");
        assert_eq!(n.silent(), 4 * 384);
    }
}
