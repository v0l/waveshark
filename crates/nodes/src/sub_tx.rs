//! Flipper SubGhz transmit: a `.sub` file's pulses keyed as they stand.
//!
//! The mirror of [`MorseTxNode`] for a saved signal: the pulses are the
//! file's own timings rather than something encoded from text, so the source
//! is a replay rather than a keyer. The modulator is still
//! [`OokModNode`], because a `.sub` file under the async OOK presets is
//! keyed carrier whatever protocol its capture was of, and a custom preset
//! that turns out to be FSK says so on its face rather than in the
//! register dump this build does not read.

use common::Result;
use common::pulse::Package;
use pipeline::node::{Node, NodeCtx, PortSpec, Simple};
use pipeline::param::{Param, ParamValue};
use pipeline::port::{Domain, Flow, Payload, PortKind, StreamSpec};
use pipeline::registry::{Category, Settings, StageDesc};
use std::time::Duration;

/// How many times the file plays per key-down. A remote sends its frame
/// several times because a receiver misses bursts; a replay needs the same
/// margin, and one pass of a short capture is over before the radio has
/// keyed.
const REPEATS: &str = "repeats";
const PAUSE_MS: &str = "pause_ms";

const FILE_PARAM: &str = "path";

/// Pulses in, pulses out, on the file's own clock.
///
/// A `.sub` capture's timings are microseconds of on-air time and the port
/// carrying them has no rate of its own, so this takes the graph's clock the
/// way [`MorseKeyNode`] does: whatever block arrives, the file is so much
/// time of it. The pulses were parsed once at load; this only paces them.
///
/// Holding the parsed file rather than a path keeps the transmitter honest:
/// a file read block by block would catch an edit half written, and a file
/// parsed at key-up would add a stall between the key going down and
/// anything going out.
pub struct SubTxNode {
    bursts: Vec<Package>,
    path: String,
    repeats: usize,
    pause: Duration,
}

impl Default for SubTxNode {
    fn default() -> Self {
        Self {
            bursts: Vec::new(),
            path: String::new(),
            repeats: 1,
            pause: Duration::from_millis(500),
        }
    }
}

impl SubTxNode {
    pub fn new(file: Option<decode::subghz::SubGhz>) -> Self {
        let mut n = Self::default();
        n.set_file(file);
        n
    }

    /// Swap in a parsed file. `None` leaves the stage holding nothing, which
    /// is what it starts as and what a cleared setting means.
    pub fn set_file(&mut self, file: Option<decode::subghz::SubGhz>) {
        match file {
            Some(f) => {
                self.bursts = f.bursts;
                self.repeats = 1;
            }
            None => self.bursts.clear(),
        }
    }

    pub fn is_loaded(&self) -> bool {
        !self.bursts.is_empty()
    }
}

impl Simple for SubTxNode {
    fn name(&self) -> &str {
        "sub_tx"
    }

    /// A pooled node holds the file it was built with, which may no longer
    /// be the one the plan carries. The stage's settings cannot say which
    /// file is loaded, so the presence of a handed-in file is the test: a
    /// rebuild whose sinks carry one is a rebuild for a newly chosen file,
    /// and the node must be built again to read it.
    fn survives_rebuild(&self, _retuned: bool, settings: &Settings) -> bool {
        !settings.contains_key("file_loaded")
    }

    fn negotiate(&mut self, input: &PortSpec) -> Result<StreamSpec> {
        if input.spec.rate <= 0.0 {
            return Err(common::Error::other("sub_tx needs the rate it should play at"));
        }
        Ok(StreamSpec {
            kind: PortKind::Pulses,
            // The timings are microseconds, so the port has no rate of its
            // own; the rate is the modulator's, passed through so a chain
            // stays rate-consistent. See `MorseKeyNode`.
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
        // A block of clock and nothing loaded is silence, which is the
        // stage's idle state rather than a refusal.
        if input.is_empty() || self.bursts.is_empty() {
            return Ok(());
        }
        let out = output.pulses_mut();
        // One pass per block: the whole file goes out the first time a block
        // arrives after a key-down, and the modulator takes it from there.
        // The pause between passes is a real gap so a burst detector at the
        // other end splits the frames the way it split the original.
        for burst in &self.bursts {
            out.push(burst.clone());
        }
        if let Some(last) = out.last_mut()
            && let Some(p) = last.pulses.last_mut()
        {
            p.gap = p.gap.max(self.pause.as_micros() as u32);
        }
        Ok(())
    }

    fn params(&self) -> Vec<Param> {
        vec![
            Param::text(FILE_PARAM, self.path.clone()).label("File"),
            Param::int(REPEATS, self.repeats as i64, 1..=100).label("Passes"),
            Param::float(PAUSE_MS, self.pause.as_millis() as f64, 0.0..=5_000.0)
                .label("Pause")
                .unit("ms"),
        ]
    }

    fn configure(&mut self, _settings: &Settings) {}

    fn set_param(&mut self, name: &str, value: ParamValue) -> Result<()> {
        match name {
            REPEATS => {
                self.repeats = value.as_i64().unwrap_or(1).clamp(1, 100) as usize;
                Ok(())
            }
            PAUSE_MS => {
                self.pause = Duration::from_millis(value.as_f64().unwrap_or(500.0).max(0.0) as u64);
                Ok(())
            }
            // A path set on the running node is a fault here rather than a
            // load: the file is parsed on the interface and handed in whole,
            // because parsing on the radio thread would stall the stream.
            FILE_PARAM => Err(common::Error::other("sub_tx takes a parsed file, not a path")),
            _ => Err(common::Error::other(format!("sub_tx: unknown parameter {name:?}"))),
        }
    }
}

pub const SUB_TX: StageDesc = StageDesc {
    name: "sub_tx",
    summary: "Play a Flipper .sub file's pulses, ready for a transmitter",
    category: Category::Transmit,
    feeds_bus: false,
};

pub fn build_sub_tx(_s: &Settings) -> Result<Box<dyn Node>> {
    Ok(Box::new(SubTxNode::default()))
}

#[cfg(test)]
mod tests {
    use super::*;
    use common::pulse::Pulse;

    fn spec(rate: f64) -> PortSpec {
        PortSpec { spec: StreamSpec { rate, ..Default::default() }, latency: 0 }
    }

    fn file() -> decode::subghz::SubGhz {
        decode::subghz::SubGhz {
            frequency: 433_920_000,
            preset: decode::subghz::Preset::Ook,
            protocol: "RAW".into(),
            bursts: vec![Package {
                pulses: vec![Pulse { mark: 350, gap: 350 }, Pulse { mark: 350, gap: 350 }],
                ..Default::default()
            }],
        }
    }

    #[test]
    fn a_loaded_file_plays_once_per_block() {
        let mut n = SubTxNode::new(Some(file()));
        let input = Payload::Real(vec![0.0; 960]);
        let mut out = Payload::empty_of(PortKind::Pulses);
        let mut ev = Vec::new();
        let mut tg = Vec::new();
        let ins = [spec(48_000.0)];
        let ctx = &mut NodeCtx::new(0, &ins, &[], &mut ev, &mut tg);
        Simple::process(&mut n, &input, &mut out, ctx).unwrap();
        let pkgs = out.as_pulses().unwrap();
        assert_eq!(pkgs.len(), 1);
        assert_eq!(pkgs[0].pulses.len(), 2, "the file's pulses, as they are");
        assert_eq!(pkgs[0].pulses[0].mark, 350);
    }

    #[test]
    fn nothing_loaded_is_silence_not_a_fault() {
        let mut n = SubTxNode::default();
        let input = Payload::Real(vec![0.0; 960]);
        let mut out = Payload::empty_of(PortKind::Pulses);
        let mut ev = Vec::new();
        let mut tg = Vec::new();
        let ins = [spec(48_000.0)];
        let ctx = &mut NodeCtx::new(0, &ins, &[], &mut ev, &mut tg);
        Simple::process(&mut n, &input, &mut out, ctx).unwrap();
        assert!(out.as_pulses().unwrap().is_empty());
    }

    #[test]
    fn a_path_is_refused_with_why() {
        let mut n = SubTxNode::default();
        assert!(
            Simple::set_param(&mut n, FILE_PARAM, ParamValue::Text("/tmp/x.sub".into())).is_err()
        );
    }
}
