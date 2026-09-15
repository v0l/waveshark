//! The speaker: the last stage, holding the sound card.
//!
//! The master level and the mute belong here and nowhere else, because
//! this is where they act. Applied to the mix instead, a mute took a queue
//! length to be heard and a recording made off a tap carried the listener's
//! volume setting; kept as the bus's parameters and applied by the radio
//! loop, the bus lied about what it did.

use common::{Error, Result};
use pipeline::node::{NodeCtx, PortSpec, Simple};
use pipeline::param::{Param, ParamValue};
use pipeline::port::{Payload, PortKind, StreamSpec};

pub const KIND: &str = "speaker";

pub struct SpeakerNode {
    /// The device, when there is one. Handed in rather than opened here,
    /// because the stream that owns it cannot cross a thread and lives in
    /// the radio loop; the graph gets the half that can be written to.
    sink: Option<audio::AudioSink>,
    master: f32,
    muted: bool,
    /// Silent while transmitting, whatever the fader says: a radio playing
    /// its own over back is talking over itself, and with desktop audio as
    /// the microphone what comes out goes back in.
    keyed: bool,
    rate: f64,
    /// Peak of the last block as it left, after the master. Zero while
    /// nothing is heard.
    peak: f32,
    backlog: i64,
}

impl SpeakerNode {
    pub fn new(sink: Option<audio::AudioSink>) -> Self {
        Self {
            sink,
            master: 0.5,
            muted: false,
            keyed: false,
            rate: super::OUT_HZ,
            peak: 0.0,
            backlog: 0,
        }
    }

    /// Swap the device. `None` leaves the receiver running silent, which is
    /// what a machine with no sound card does.
    pub fn set_sink(&mut self, sink: Option<audio::AudioSink>) {
        self.sink = sink;
        self.drive();
    }

    pub fn set_keyed(&mut self, keyed: bool) {
        if keyed != self.keyed {
            self.keyed = keyed;
            self.drive();
        }
    }

    pub fn master(&self) -> (f32, bool) {
        (self.master, self.muted)
    }

    /// Whether anything can be heard: the mute, or the key.
    pub fn silent(&self) -> bool {
        self.muted || self.keyed
    }

    /// The level of the last block as it left.
    pub fn peak(&self) -> f32 {
        self.peak
    }

    /// Frames queued at the device, for the backlog reading.
    pub fn backlog(&self) -> i64 {
        self.backlog
    }

    /// Put the level and the mute on the device, which applies them in its
    /// own callback: heard at once rather than a queue length later.
    fn drive(&mut self) {
        let silent = self.silent();
        if let Some(s) = &mut self.sink {
            s.set_output(self.master, silent);
        }
    }
}

impl Simple for SpeakerNode {
    fn name(&self) -> &str {
        KIND
    }

    fn is_sink(&self) -> bool {
        true
    }

    fn negotiate(&mut self, input: &PortSpec) -> Result<StreamSpec> {
        if input.spec.kind != PortKind::Real {
            return Err(Error::other("the speaker takes audio"));
        }
        self.rate = input.spec.frame_rate();
        Ok(input.spec)
    }

    fn process(
        &mut self,
        input: &Payload,
        _output: &mut Payload,
        _ctx: &mut NodeCtx<'_>,
    ) -> Result<()> {
        let Some(pcm) = input.as_real() else {
            return Ok(());
        };
        if pcm.is_empty() {
            self.peak = 0.0;
            return Ok(());
        }
        self.peak = if self.silent() {
            0.0
        } else {
            pcm.iter().fold(0.0f32, |a, v| a.max(v.abs())) * self.master
        };
        let Some(s) = &mut self.sink else {
            return Ok(());
        };
        // Written whether or not it is heard, so the drift loop stays
        // converged and unmuting does not open with a burst of resampling.
        s.write_adaptive_stereo(pcm, self.rate);
        self.backlog = s.backlog().max(0);
        Ok(())
    }

    fn readings(&self) -> Vec<(String, String)> {
        let mut out = Vec::new();
        if self.sink.is_none() {
            out.push(("device".into(), "none".into()));
        }
        if self.keyed {
            out.push(("held".into(), "transmitting".into()));
        }
        out
    }

    fn params(&self) -> Vec<Param> {
        vec![
            Param::float("master", self.master as f64, 0.0..=1.0).label("Master"),
            Param::bool("muted", self.muted).label("Muted"),
        ]
    }

    fn set_param(&mut self, name: &str, v: ParamValue) -> Result<()> {
        match name {
            "master" => {
                let f = v.as_f64().ok_or_else(|| Error::other("expected a number"))?;
                self.master = (f as f32).clamp(0.0, 1.0);
            }
            "muted" => {
                self.muted = v.as_bool().ok_or_else(|| Error::other("expected a switch"))?;
            }
            _ => return Err(Error::other(format!("speaker: unknown parameter {name:?}"))),
        }
        self.drive();
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use pipeline::node::Node;

    fn run(n: &mut SpeakerNode, pcm: &[f32]) {
        let spec = StreamSpec {
            kind: PortKind::Real,
            rate: super::super::OUT_HZ * 2.0,
            channels: 2,
            ..Default::default()
        };
        let ins = [PortSpec { spec, latency: 0 }];
        Node::negotiate(n, &ins).unwrap();
        let input = Payload::Real(pcm.to_vec());
        let mut out = [Payload::Real(Vec::new())];
        let (tags, mut events, mut new_tags) = (Vec::new(), Vec::new(), Vec::new());
        let mut ctx = NodeCtx::new(0, &ins, &tags, &mut events, &mut new_tags);
        Node::process(n, &[&input], &mut out, &mut ctx).unwrap();
    }

    #[test]
    fn the_key_silences_the_speaker_whatever_the_fader_says() {
        let mut n = SpeakerNode::new(None);
        Node::set_param(&mut n, "master", ParamValue::Float(1.0)).unwrap();
        run(&mut n, &[0.5; 96]);
        assert_eq!(n.peak(), 0.5);
        n.set_keyed(true);
        run(&mut n, &[0.5; 96]);
        assert_eq!(n.peak(), 0.0, "an over is not played back to the operator");
        n.set_keyed(false);
        Node::set_param(&mut n, "muted", ParamValue::Bool(true)).unwrap();
        run(&mut n, &[0.5; 96]);
        assert_eq!(n.peak(), 0.0);
    }

    #[test]
    fn the_master_is_what_the_meter_reads() {
        let mut n = SpeakerNode::new(None);
        Node::set_param(&mut n, "master", ParamValue::Float(0.25)).unwrap();
        run(&mut n, &[1.0; 96]);
        assert_eq!(n.peak(), 0.25);
    }
}
