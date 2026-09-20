//! One input's level, mute and name, on a stage of its own.
//!
//! A fader used to be a numbered input on the bus, `vol3` and `mute3`, and
//! the number moved every time a channel came or went: every rebuild had to
//! carry the settings across the renumbering by hand, and a channel found
//! its fader by scanning the wires for its port. A fader is a stage now,
//! derived under an id that is the channel's, so its level is a setting on
//! it like any other and an edit carries it across a rebuild.
//!
//! What leaves is speech with its labels, whatever arrived. Audio from a
//! demodulator is named here, as the channel it came off, so that
//! everything downstream of a fader can say what it is carrying: the bus
//! knows what it mixed and the speaker what it is playing. Two outputs: the
//! first after the fader, for the mix; the second before it, for the
//! transcriber and the call list, which want what the receiver heard whether
//! or not the operator chose to listen. A muted strip is still written down.

use common::{Error, Result};
use pipeline::node::{Node, NodeCtx, PortSpec};
use pipeline::param::{Param, ParamValue};
use pipeline::port::{Payload, PortKind, StreamSpec};

pub const KIND: &str = "fader";

/// What an analogue channel is called on the tap. Not a modulation: the
/// fader is handed audio and does not know how it was demodulated, and what
/// matters downstream is that nothing named the speaker or the talkgroup.
pub const ANALOGUE: &str = "Audio";

/// What the fader is passing, as the graph negotiated it.
#[derive(Clone, Copy, Debug, PartialEq)]
enum Feed {
    Silent,
    /// Real audio, mono or stereo, at some rate.
    Audio {
        channels: usize,
        rate: f64,
        center_hz: f64,
    },
    /// Speech with its labels. Levelled the same, tapped as it is.
    Voice,
}

pub struct FaderNode {
    volume: f32,
    muted: bool,
    label: String,
    /// Whether the operator says what passes here is people talking. On the
    /// tap it is then named as a conversation, with the label standing in
    /// for the party called, so the transcriber can follow it. Speech that
    /// arrives labelled needs no such switch.
    speech: bool,
    feed: Feed,
    /// Peak of the last block after the fader, for the meter beside it.
    peak: f32,
}

impl FaderNode {
    pub fn new() -> Self {
        Self {
            volume: 0.8,
            muted: false,
            label: String::new(),
            speech: false,
            feed: Feed::Silent,
            peak: 0.0,
        }
    }

    pub fn volume(&self) -> f32 {
        self.volume
    }

    pub fn muted(&self) -> bool {
        self.muted
    }

    pub fn label(&self) -> &str {
        &self.label
    }

    #[cfg_attr(not(test), allow(dead_code))]
    pub fn speech(&self) -> bool {
        self.speech
    }

    pub fn peak(&self) -> f32 {
        self.peak
    }

    /// Whether this passes speech rather than audio.
    pub fn is_voice(&self) -> bool {
        self.feed == Feed::Voice
    }

    fn gain(&self) -> f32 {
        if self.muted { 0.0 } else { self.volume.clamp(0.0, 1.0) }
    }
}

impl Default for FaderNode {
    fn default() -> Self {
        Self::new()
    }
}

impl Node for FaderNode {
    fn name(&self) -> &str {
        KIND
    }

    fn num_outputs(&self) -> usize {
        2
    }

    fn negotiate(&mut self, inputs: &[PortSpec]) -> Result<Vec<StreamSpec>> {
        let input = inputs.first().ok_or_else(|| Error::other("a fader needs an input"))?.spec;
        let voice = |channels: usize| StreamSpec {
            kind: PortKind::Voice,
            rate: input.frame_rate() * channels as f64,
            center: input.center,
            bandwidth: 0.0,
            channels,
            ..Default::default()
        };
        self.feed = match input.kind {
            PortKind::Voice => Feed::Voice,
            PortKind::Real if input.is_silence() => Feed::Silent,
            PortKind::Real => Feed::Audio {
                channels: input.channels.max(1),
                rate: input.frame_rate(),
                center_hz: input.center.as_f64(),
            },
            other => {
                return Err(Error::other(format!("a fader takes audio or speech, not {other:?}")));
            }
        };
        Ok(vec![voice(input.channels.max(1)), voice(1)])
    }

    fn process(
        &mut self,
        inputs: &[&Payload],
        outputs: &mut [Payload],
        _ctx: &mut NodeCtx<'_>,
    ) -> Result<()> {
        let gain = self.gain();
        let (out, tap) = match outputs {
            [out, tap, ..] => (out, tap),
            _ => return Ok(()),
        };
        let mut peak = 0.0f32;
        let mut pass = |v: common::Voice| {
            // Mono on the tap, because a transcriber wants one voice and
            // not a stereo image, and at the input's own rate: whoever
            // reads it resamples.
            tap.voice_mut().push(common::Voice { channels: 1, pcm: v.mono(), ..v.clone() });
            if v.pcm.is_empty() {
                return;
            }
            let mut faded = v;
            for s in &mut faded.pcm {
                *s *= gain;
            }
            peak = faded.pcm.iter().fold(peak, |a, s| a.max(s.abs()));
            out.voice_mut().push(faded);
        };
        match (inputs.first(), self.feed) {
            (Some(Payload::Real(pcm)), Feed::Audio { channels, rate, center_hz }) => {
                if !pcm.is_empty() {
                    pass(common::Voice {
                        system: ANALOGUE,
                        channel_hz: center_hz,
                        to: self.speech.then(|| self.label.clone()),
                        from: None,
                        // A fader in front of a channel with no identity
                        // stage has nothing to say about the group.
                        code: None,
                        over: None,
                        rate,
                        channels,
                        pcm: pcm.clone(),
                    });
                }
            }
            (Some(Payload::Voice(voices)), Feed::Voice) => {
                for v in voices {
                    pass(v.clone());
                }
            }
            _ => {}
        }
        self.peak = peak;
        Ok(())
    }

    fn reset(&mut self) {
        self.peak = 0.0;
    }

    fn params(&self) -> Vec<Param> {
        vec![
            Param::float("vol", self.volume as f64, 0.0..=1.0).label("Level"),
            Param::bool("mute", self.muted).label("Mute"),
            Param::text("label", self.label.clone()).label("Name"),
            Param::bool("speech", self.speech).label("Speech"),
        ]
    }

    fn set_param(&mut self, name: &str, v: ParamValue) -> Result<()> {
        let num = |v: &ParamValue| {
            v.as_f64().map(|f| f as f32).ok_or_else(|| Error::other("expected a number"))
        };
        let flag = |v: &ParamValue| v.as_bool().ok_or_else(|| Error::other("expected a switch"));
        match name {
            "vol" => self.volume = num(&v)?.clamp(0.0, 1.0),
            "mute" => self.muted = flag(&v)?,
            "label" => self.label = v.as_str().unwrap_or_default().to_string(),
            "speech" => self.speech = flag(&v)?,
            _ => return Err(Error::other(format!("fader: unknown parameter {name:?}"))),
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn run(n: &mut FaderNode, spec: StreamSpec, input: Payload) -> (Payload, Payload) {
        let ins = [PortSpec { spec, latency: 0 }];
        let specs = n.negotiate(&ins).unwrap();
        let mut out: Vec<Payload> = specs.iter().map(|s| Payload::empty_of(s.kind)).collect();
        let (tags, mut events, mut new_tags) = (Vec::new(), Vec::new(), Vec::new());
        let mut ctx = NodeCtx::new(0, &ins, &tags, &mut events, &mut new_tags);
        n.process(&[&input], &mut out, &mut ctx).unwrap();
        let tap = out.pop().unwrap();
        (out.pop().unwrap(), tap)
    }

    fn stereo() -> StreamSpec {
        StreamSpec {
            kind: PortKind::Real,
            rate: 96_000.0,
            channels: 2,
            center: common::Hz(145_500_000),
            ..Default::default()
        }
    }

    #[test]
    fn the_level_is_applied_after_the_tap() {
        let mut n = FaderNode::new();
        n.set_param("vol", ParamValue::Float(0.5)).unwrap();
        n.set_param("label", ParamValue::Text("CH1".into())).unwrap();
        n.set_param("speech", ParamValue::Bool(true)).unwrap();
        let (out, tap) = run(&mut n, stereo(), Payload::Real(vec![1.0, 0.0, -1.0, 0.0]));
        let out = out.as_voice().unwrap();
        assert_eq!(out.len(), 1);
        assert_eq!(out[0].pcm, vec![0.5, 0.0, -0.5, 0.0]);
        assert_eq!(out[0].channels, 2, "a stereo station keeps its sides");
        assert_eq!(out[0].to.as_deref(), Some("CH1"), "named, so the bus can say what it plays");
        assert_eq!(n.peak(), 0.5);
        let tap = tap.as_voice().unwrap();
        assert_eq!(tap.len(), 1);
        assert_eq!(tap[0].pcm, vec![0.5, -0.5], "mono, before the fader");
        assert_eq!(tap[0].rate, 48_000.0);
        assert_eq!(tap[0].channel_hz, 145_500_000.0);
        assert_eq!(tap[0].to.as_deref(), Some("CH1"), "named as a conversation");
        assert_eq!(tap[0].system, ANALOGUE);
    }

    #[test]
    fn a_muted_strip_is_still_written_down() {
        let mut n = FaderNode::new();
        n.set_param("mute", ParamValue::Bool(true)).unwrap();
        let (out, tap) = run(&mut n, stereo(), Payload::Real(vec![1.0, 1.0]));
        assert_eq!(out.as_voice().unwrap()[0].pcm, vec![0.0, 0.0]);
        assert_eq!(tap.as_voice().unwrap()[0].pcm, vec![1.0]);
        assert_eq!(tap.as_voice().unwrap()[0].to, None, "not speech unless somebody says");
    }

    #[test]
    fn speech_is_levelled_the_same_and_tapped_as_it_came() {
        let mut n = FaderNode::new();
        n.set_param("vol", ParamValue::Float(0.25)).unwrap();
        let spec = StreamSpec { kind: PortKind::Voice, rate: 8_000.0, ..Default::default() };
        let v = common::Voice {
            system: "M17",
            channel_hz: 433_475_000.0,
            to: Some("ALL".into()),
            from: Some("M0ABC".into()),
            code: None,
            over: None,
            rate: 8_000.0,
            channels: 1,
            pcm: vec![0.8, -0.8],
        };
        let (out, tap) = run(&mut n, spec, Payload::Voice(vec![v.clone()]));
        assert_eq!(out.as_voice().unwrap()[0].pcm, vec![0.2, -0.2]);
        assert_eq!(out.as_voice().unwrap()[0].to.as_deref(), Some("ALL"));
        assert_eq!(tap.as_voice().unwrap(), &[v]);
        assert!(n.is_voice());
    }
}
