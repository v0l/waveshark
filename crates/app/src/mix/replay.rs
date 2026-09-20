//! A decoded transmission played back once, from the packet list.
//!
//! A stage with no input: what it plays was handed to it, and it paces
//! itself off the run's clock. Through the same bus as everything else,
//! because it is the same audio path and the master should mean the same
//! thing for a replay as for a channel.

use common::{Result, Speech};
use pipeline::node::{Node, NodeCtx, PortSpec};
use pipeline::port::{Payload, PortKind, StreamSpec};
use std::sync::Arc;

pub const KIND: &str = "replay";

/// What a replay is called on the bus, where everything is labelled.
pub const SYSTEM: &str = "Replay";

pub struct ReplayNode {
    out_rate: f64,
    /// What is left to play, already at `out_rate`.
    queue: std::collections::VecDeque<f32>,
    /// The fraction of an output frame the last block was worth and did not
    /// produce. See [`super::bus::BusNode`] for why it is carried.
    owed: f64,
}

impl ReplayNode {
    pub fn new(out_rate: f64) -> Self {
        Self { out_rate, queue: std::collections::VecDeque::new(), owed: 0.0 }
    }

    /// Queue a transmission to be played once, replacing whatever was
    /// already playing: two at a time is noise, not a review.
    pub fn play(&mut self, speech: &Arc<Speech>) {
        let mut rs = audio::Resampler::new(speech.rate, self.out_rate, 4);
        let mut out = Vec::with_capacity(speech.pcm.len() * 8);
        rs.process(&speech.pcm, &mut out);
        self.queue.clear();
        self.queue.extend(out);
    }

    pub fn stop(&mut self) {
        self.queue.clear();
    }

    #[cfg_attr(not(test), allow(dead_code))]
    pub fn replaying(&self) -> bool {
        !self.queue.is_empty()
    }

    /// Seconds left to play.
    pub fn left(&self) -> f64 {
        self.queue.len() as f64 / self.out_rate.max(1.0)
    }

    /// A block's worth, so a replay runs at real time rather than arriving
    /// all at once. With no clock at all it paces itself at fifty a second.
    pub fn take(&mut self, block_s: f64) -> Vec<f32> {
        let want = if block_s > 0.0 {
            let w = block_s * self.out_rate + self.owed;
            let frames = w.max(0.0).floor();
            self.owed = w - frames;
            frames as usize
        } else {
            (self.out_rate / 50.0) as usize
        };
        let take = want.min(self.queue.len());
        self.queue.drain(..take).collect()
    }
}

impl Node for ReplayNode {
    fn name(&self) -> &str {
        KIND
    }

    fn num_inputs(&self) -> usize {
        0
    }

    fn negotiate(&mut self, _inputs: &[PortSpec]) -> Result<Vec<StreamSpec>> {
        Ok(vec![StreamSpec {
            kind: PortKind::Voice,
            rate: self.out_rate,
            center: common::Hz(0),
            bandwidth: 0.0,
            channels: 1,
            ..Default::default()
        }])
    }

    fn process(
        &mut self,
        _inputs: &[&Payload],
        outputs: &mut [Payload],
        ctx: &mut NodeCtx<'_>,
    ) -> Result<()> {
        if self.queue.is_empty() {
            return Ok(());
        }
        let block = self.take(ctx.block_seconds);
        if let Some(o) = outputs.first_mut() {
            o.voice_mut().push(common::Voice {
                system: SYSTEM,
                channel_hz: 0.0,
                to: None,
                from: None,
                code: None,
                over: None,
                rate: self.out_rate,
                channels: 1,
                pcm: block,
            });
        }
        Ok(())
    }

    fn reset(&mut self) {
        self.stop();
        self.owed = 0.0;
    }

    fn readings(&self) -> Vec<(String, String)> {
        if self.queue.is_empty() {
            Vec::new()
        } else {
            vec![("left".into(), format!("{:.1} s", self.left()))]
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_replay_is_paced_by_the_block_rather_than_dumped() {
        // Handing the speaker a whole transmission at once plays it at
        // whatever rate the device drains, which is not the rate it was
        // spoken at.
        let mut r = ReplayNode::new(48_000.0);
        let speech = Arc::new(Speech { pcm: vec![0.5; 8_000], rate: 8_000.0 });
        r.play(&speech);
        assert!(r.replaying());
        assert!((r.left() - 1.0).abs() < 0.05, "{} s queued", r.left());
        let n = r.take(0.025).len();
        assert_eq!(n, 1_200, "a block's worth at a time");
        assert!(r.replaying(), "the rest is still queued");
        r.stop();
        assert!(!r.replaying());
    }
}
