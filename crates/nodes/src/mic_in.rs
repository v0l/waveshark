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
//! The microphone is the clock, not the radio. What comes out each block is
//! whatever the device has produced since the last one, so a device running
//! a few parts per million away from the radio simply delivers slightly more
//! or fewer samples: the stream stays continuous and a decoder downstream
//! reads it at its own pace.
//!
//! Producing a fixed count per block instead, and padding with silence when
//! the device was behind, looked reasonable and was not. The padding is not
//! time that passed, it is samples the transmission never had, so an SSTV
//! picture drifted further out of step the longer it went on: the top of the
//! picture was right and the bottom was shredded.

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
    /// Most samples to take in one block, from negotiation: a burst longer
    /// than this is a device that stalled, and catching up on all of it at
    /// once would put a lump of old audio into the stream.
    most: usize,
    /// Samples the device produced that nobody read in time, which is this
    /// node not being run often enough.
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
        Self { src: Some(src), rate, most: 0, silent: 0, peak: 0.0 }
    }

    /// A stage with no microphone yet. It produces silence rather than
    /// refusing to build, so the chain behind it survives until a device
    /// arrives.
    pub fn idle() -> Self {
        Self { src: None, rate: IDLE_RATE, most: 0, silent: 0, peak: 0.0 }
    }

    pub fn is_open(&self) -> bool {
        self.src.is_some()
    }

    /// Peak of the last block, for a meter beside whatever is listening.
    pub fn peak(&self) -> f32 {
        self.peak
    }

    /// Samples the microphone produced that were never read.
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
        // Half a second: long enough to ride a scheduling hiccup, short
        // enough that what arrives is still what was heard.
        self.most = (self.rate * 0.5) as usize;
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
        let out = o.real_mut();
        let before = out.len();
        if let Some(src) = &self.src {
            // Everything the device has, rather than a share of the block:
            // the microphone's own clock decides how much that is.
            src.take(out, self.most);
            self.silent = src.overruns();
        }
        self.peak = out[before..].iter().fold(0.0f32, |m, v| m.max(v.abs()));
        Ok(())
    }

    fn reset(&mut self) {}
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

    /// What the device produced, with nothing added and nothing dropped.
    ///
    /// The stream has to be the microphone's own samples in order: padding a
    /// short block with silence puts time into the stream that never
    /// happened, and an SSTV picture drifts out of step by the bottom.
    #[test]
    fn what_comes_out_is_what_the_microphone_put_in() {
        let n = 4_800;
        let tone: Vec<f32> =
            (0..n).map(|i| (std::f32::consts::TAU * 1000.0 * i as f32 / 48_000.0).sin()).collect();
        let src = std::sync::Arc::new(audio::Canned::new(tone.clone(), 48_000.0, false));
        let mut node = MicInNode::new(src);
        let got = run(&mut node, 40, 16_384, 2_048_000.0);
        assert_eq!(got.len(), n, "every sample once");
        assert_eq!(got, tone, "in the order the device produced them");
    }

    /// With no device the stage still builds and negotiates, and produces
    /// nothing rather than taking the chain down.
    #[test]
    fn a_stage_with_no_microphone_produces_nothing() {
        let mut n = MicInNode::idle();
        assert!(!n.is_open());
        let got = run(&mut n, 4, 16_384, 2_048_000.0);
        assert!(got.is_empty(), "no device, no audio");
    }
}
