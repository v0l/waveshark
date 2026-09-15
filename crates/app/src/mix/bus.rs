//! The audio bus: where every path to the speaker meets.
//!
//! A sum, and the record of what it summed. Each input carries speech with
//! its labels, whatever it was demodulated from: a fader names the channel
//! it came off, the calls stage passes each conversation as it arrived. The
//! bus brings each block to the speaker's rate, adds it into a stereo mix
//! for the speaker, and puts the same blocks out labelled on a second port,
//! so that what is being heard now, by whom, on what frequency and to which
//! group, can be read off the graph rather than guessed from the mix.
//!
//! Levels are not applied here; every input has a [`super::fader`] or is a
//! stage with a level of its own, so the bus has no numbered settings that
//! have to be carried across a rebuild when the inputs are renumbered.
//!
//! The last input is always spare and fed by nothing. That is what a chain
//! drawn by hand is wired into, and the receiver draws a new spare once it
//! is taken.

use common::{Error, Result};
use pipeline::node::{Node, NodeCtx, PortSpec};
use pipeline::param::ParamValue;
use pipeline::port::{Payload, PortKind, StreamSpec};
use std::collections::HashMap;

pub const KIND: &str = "audio_bus";

/// One thing the bus is playing, as the interface shows it.
#[derive(Clone, Debug, PartialEq)]
pub struct Playing {
    pub key: common::ConversationKey,
    /// Peak of the last block that had anything in it, after the faders.
    pub peak: f32,
    /// How long since it last had anything in it, so a row can be drawn
    /// through a pause without saying it is loud.
    pub quiet_s: f64,
}

/// Below this peak a block is nothing: a squelched channel and a replay
/// between words both deliver blocks of silence, and listing them is
/// listing everything the graph has.
const FLOOR: f32 = 0.002;

/// How long something stays in the list after its last block of audio.
///
/// Speech pauses between words and between sentences, and a row that came
/// and went with every pause moved everything under it on the strip. Long
/// enough to cover a breath, short enough that the list is still what is
/// playing now.
const HOLD_S: f64 = 1.5;

/// What one input carries, as negotiated.
#[derive(Clone, Copy, Debug, PartialEq)]
enum Feed {
    Silent,
    Voice,
    /// Bare audio wired in by hand, with no fader to name it: heard under
    /// the analogue name at the frequency the stream says, at unity.
    Audio {
        channels: usize,
        rate: f64,
        center_hz: f64,
    },
}

pub struct BusNode {
    out_rate: f64,
    feeds: Vec<Feed>,
    /// One resampler per stream, because each carries filter state. Keyed
    /// by input and carrier: a fader's input is one stream, and the calls
    /// stage delivers everything at the speaker's rate already.
    rs: HashMap<(usize, String), audio::Resampler>,
    /// This block's mix, stereo interleaved at `out_rate`.
    mix: Vec<f32>,
    scratch: Vec<f32>,
    lane: Vec<f32>,
    lane_out: Vec<f32>,
    /// What went into the mix this block, labelled and at `out_rate`.
    played: Vec<common::Voice>,
    /// What was last in the mix, for the interface.
    playing: Vec<Playing>,
    /// The last named party heard, for the interface to show.
    last: Option<String>,
    /// The fraction of an output frame this block was worth and the last one
    /// did not produce. See [`BusNode::process`].
    owed: f64,
}

impl BusNode {
    pub fn new(out_rate: f64) -> Self {
        Self {
            out_rate,
            feeds: vec![Feed::Silent],
            rs: HashMap::new(),
            mix: Vec::new(),
            scratch: Vec::new(),
            lane: Vec::new(),
            lane_out: Vec::new(),
            played: Vec::new(),
            playing: Vec::new(),
            last: None,
            owed: 0.0,
        }
    }

    #[cfg_attr(not(test), allow(dead_code))]
    pub fn out_rate(&self) -> f64 {
        self.out_rate
    }

    /// What was in the mix last block, with its level.
    pub fn playing(&self) -> &[Playing] {
        &self.playing
    }

    /// The last conversation mixed, as "who to whom".
    #[cfg_attr(not(test), allow(dead_code))]
    pub fn last_heard(&self) -> Option<&str> {
        self.last.as_deref()
    }

    /// Put one block into the mix.
    ///
    /// Interleaved with as many channels as the block says; a mono block is
    /// heard on both sides, which is what a receiver does with a mono
    /// station anyway, and it means a broadcast in stereo can share the
    /// output with a narrowband channel that has no such thing.
    fn feed(&mut self, k: usize, v: &common::Voice) {
        if v.pcm.is_empty() {
            return;
        }
        let ch = v.channels.max(1);
        // Brought to the speaker's rate first. Two channels at slightly
        // different rates summed sample for sample play one of them at the
        // wrong pitch, which is what happened when this was a loop in the
        // radio thread that took the last channel's rate for all of them.
        let frames = if (v.rate - self.out_rate).abs() < 0.5 {
            self.scratch.clear();
            self.scratch.extend_from_slice(&v.pcm);
            v.pcm.len() / ch
        } else {
            let mut frames = 0;
            self.scratch.clear();
            for side in 0..ch {
                let key = (k, format!("{}:{:.0}:{side}", v.system, v.channel_hz));
                let (rate, out_rate) = (v.rate, self.out_rate);
                let r =
                    self.rs.entry(key).or_insert_with(|| audio::Resampler::new(rate, out_rate, 8));
                self.lane.clear();
                self.lane.extend(v.pcm.iter().skip(side).step_by(ch).copied());
                self.lane_out.clear();
                r.process(&self.lane, &mut self.lane_out);
                if side == 0 {
                    frames = self.lane_out.len();
                    self.scratch.resize(frames * ch, 0.0);
                }
                for (f, s) in self.lane_out.iter().take(frames).enumerate() {
                    self.scratch[f * ch + side] = *s;
                }
            }
            frames
        };
        if self.mix.len() < frames * 2 {
            self.mix.resize(frames * 2, 0.0);
        }
        for f in 0..frames {
            let (l, r) = if ch >= 2 {
                (self.scratch[f * ch], self.scratch[f * ch + 1])
            } else {
                (self.scratch[f], self.scratch[f])
            };
            self.mix[f * 2] += l;
            self.mix[f * 2 + 1] += r;
        }
        self.played.push(common::Voice {
            rate: self.out_rate,
            pcm: std::mem::take(&mut self.scratch),
            ..v.clone()
        });
        if let Some(to) = &v.to {
            self.last = Some(match &v.from {
                Some(f) => format!("{f} to {to}"),
                None => to.clone(),
            });
        }
    }

    /// This block's audio, held inside full scale, as stereo at the output
    /// rate, at least `frames` long.
    ///
    /// The block's worth of silence when nothing is playing, rather than
    /// nothing at all: the speaker is driven by what comes out of here, and
    /// a bus with nothing to mix handed the sound card no samples and the
    /// speaker starved, which is the same symptom as a receiver that is not
    /// running.
    ///
    /// Clipped rather than scaled to fit: several channels at once can sum
    /// past full scale, and quietly turning everything down would make the
    /// level of the channel being listened to depend on how busy its
    /// neighbours are.
    fn render(&mut self, frames: usize) -> &[f32] {
        if self.mix.len() < frames * 2 {
            self.mix.resize(frames * 2, 0.0);
        }
        for v in self.mix.iter_mut() {
            *v = v.clamp(-1.0, 1.0);
        }
        &self.mix
    }
}

impl Node for BusNode {
    fn name(&self) -> &str {
        KIND
    }

    fn num_inputs(&self) -> usize {
        self.feeds.len().max(1)
    }

    /// The speaker, and what is playing on it.
    fn num_outputs(&self) -> usize {
        2
    }

    /// A mixer has a spare input by nature.
    fn optional_inputs(&self) -> bool {
        true
    }

    fn negotiate(&mut self, inputs: &[PortSpec]) -> Result<Vec<StreamSpec>> {
        for (k, i) in inputs.iter().enumerate() {
            let feed = match i.spec.kind {
                PortKind::Voice => Feed::Voice,
                PortKind::Real if i.spec.is_silence() => Feed::Silent,
                PortKind::Real => Feed::Audio {
                    channels: i.spec.channels.max(1),
                    rate: i.spec.frame_rate(),
                    center_hz: i.spec.center.as_f64(),
                },
                other => {
                    return Err(Error::other(format!(
                        "the audio bus takes audio, and input {k} carries {other:?}"
                    )));
                }
            };
            if let Some(f) = self.feeds.get_mut(k) {
                *f = feed;
            }
        }
        let out_rate = self.out_rate;
        Ok(vec![
            StreamSpec {
                kind: PortKind::Real,
                rate: out_rate * 2.0,
                center: common::Hz(0),
                bandwidth: 0.0,
                channels: 2,
                ..Default::default()
            },
            StreamSpec {
                kind: PortKind::Voice,
                rate: out_rate,
                center: common::Hz(0),
                bandwidth: 0.0,
                channels: 1,
                ..Default::default()
            },
        ])
    }

    fn process(
        &mut self,
        inputs: &[&Payload],
        outputs: &mut [Payload],
        ctx: &mut NodeCtx<'_>,
    ) -> Result<()> {
        for (k, p) in inputs.iter().enumerate() {
            match (p, self.feeds.get(k).copied()) {
                (Payload::Voice(voices), _) => {
                    for v in voices {
                        self.feed(k, v);
                    }
                }
                (Payload::Real(pcm), Some(Feed::Audio { channels, rate, center_hz })) => {
                    let v = common::Voice {
                        system: super::fader::ANALOGUE,
                        channel_hz: center_hz,
                        to: None,
                        from: None,
                        code: None,
                        rate,
                        channels,
                        pcm: pcm.clone(),
                    };
                    self.feed(k, &v);
                }
                _ => {}
            }
        }
        // What this block is worth in audio, from the run's own clock, with
        // the fraction of a frame carried to the next block.
        //
        // Rounding each block on its own is a rate error, not a rounding
        // error, because the same block length arrives every time: 131072
        // samples at 20 MS/s is 6.5536 ms, which is 314.57 frames at 48 kHz,
        // and 315 of them every block is 0.14% too much audio forever. The
        // sink's drift loop trims by at most 0.1%, so it cannot absorb that:
        // the queue climbed from its 1024 sample target to the 12000 where
        // the callback throws blocks away, clicked, and climbed again.
        let want = ctx.block_seconds * self.out_rate + self.owed;
        let frames = want.max(0.0).floor();
        self.owed = want - frames;
        let frames = frames as usize;
        let (out, played) = match outputs {
            [out, played, ..] => (out, played),
            _ => return Ok(()),
        };
        out.real_mut().extend_from_slice(self.render(frames));
        self.mix.clear();
        // Held rather than rebuilt each block: what is playing is a list of
        // conversations, not of blocks, and the pauses in speech are not
        // gaps in it. A row keeps its place in the list, so nothing moves
        // under the pointer while somebody is talking.
        for p in self.playing.iter_mut() {
            p.quiet_s += ctx.block_seconds.max(0.0);
        }
        for v in &self.played {
            let peak = v.pcm.iter().fold(0.0f32, |a, s| a.max(s.abs()));
            if peak <= FLOOR {
                continue;
            }
            let key = common::ConversationKey::of(v);
            match self.playing.iter_mut().find(|p| p.key == key) {
                Some(p) => {
                    p.peak = peak;
                    p.quiet_s = 0.0;
                }
                None => self.playing.push(Playing { key, peak, quiet_s: 0.0 }),
            }
        }
        self.playing.retain(|p| p.quiet_s < HOLD_S);
        played.voice_mut().append(&mut self.played);
        Ok(())
    }

    fn reset(&mut self) {
        self.mix.clear();
        self.played.clear();
        self.playing.clear();
        self.rs.clear();
        self.owed = 0.0;
    }

    fn readings(&self) -> Vec<(String, String)> {
        match &self.last {
            Some(l) => vec![("heard".into(), l.clone())],
            None => Vec::new(),
        }
    }

    fn set_param(&mut self, name: &str, v: ParamValue) -> Result<()> {
        match name {
            "inputs" => {
                let n = v.as_i64().ok_or_else(|| Error::other("expected a count"))?;
                self.feeds.resize(n.max(1) as usize, Feed::Silent);
            }
            _ => return Err(Error::other(format!("audio_bus: unknown parameter {name:?}"))),
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn voice(rate: f64, channels: usize, pcm: &[f32]) -> common::Voice {
        common::Voice {
            system: super::super::fader::ANALOGUE,
            channel_hz: 145_500_000.0,
            to: Some("CH1".into()),
            from: None,
            code: None,
            rate,
            channels,
            pcm: pcm.to_vec(),
        }
    }

    /// A bus with `n` inputs.
    fn bus(n: usize) -> (BusNode, Vec<PortSpec>) {
        let mut node = BusNode::new(48_000.0);
        node.set_param("inputs", ParamValue::Int(n as i64)).unwrap();
        let spec = StreamSpec { kind: PortKind::Voice, rate: 48_000.0, ..Default::default() };
        let ins: Vec<PortSpec> = (0..n).map(|_| PortSpec { spec, latency: 0 }).collect();
        node.negotiate(&ins).unwrap();
        (node, ins)
    }

    fn run(
        node: &mut BusNode,
        ins: &[PortSpec],
        inputs: &[Payload],
        block_s: f64,
    ) -> (Vec<f32>, Vec<common::Voice>) {
        let refs: Vec<&Payload> = inputs.iter().collect();
        let mut out = [Payload::Real(Vec::new()), Payload::Voice(Vec::new())];
        let (mut events, mut tags) = (Vec::new(), Vec::new());
        let mut ctx = NodeCtx::new(0, ins, &[], &mut events, &mut tags);
        ctx.block_seconds = block_s;
        node.process(&refs, &mut out, &mut ctx).unwrap();
        (out[0].as_real().unwrap().to_vec(), out[1].as_voice().unwrap().to_vec())
    }

    #[test]
    fn a_mono_input_is_heard_on_both_sides() {
        let (mut n, ins) = bus(1);
        let (out, _) =
            run(&mut n, &ins, &[Payload::Voice(vec![voice(48e3, 1, &[0.5, -0.5])])], 0.0);
        assert_eq!(out, &[0.5, 0.5, -0.5, -0.5]);
    }

    #[test]
    fn a_stereo_input_keeps_its_sides() {
        let (mut n, ins) = bus(1);
        let v = voice(48e3, 2, &[0.5, -0.5, 0.25, -0.25]);
        let (out, _) = run(&mut n, &ins, &[Payload::Voice(vec![v])], 0.0);
        assert_eq!(out, &[0.5, -0.5, 0.25, -0.25]);
    }

    #[test]
    fn two_inputs_sum_and_the_sum_is_clipped() {
        let (mut n, ins) = bus(2);
        let a = Payload::Voice(vec![voice(48e3, 1, &[0.75, 0.25])]);
        let b = Payload::Voice(vec![voice(48e3, 1, &[0.75, 0.25])]);
        let (out, _) = run(&mut n, &ins, &[a, b], 0.0);
        assert_eq!(out, &[1.0, 1.0, 0.5, 0.5]);
    }

    #[test]
    fn an_input_at_another_rate_is_brought_to_the_speakers() {
        let (mut n, ins) = bus(1);
        let (out, played) =
            run(&mut n, &ins, &[Payload::Voice(vec![voice(8e3, 1, &[0.5; 160])])], 0.0);
        let frames = out.len() / 2;
        assert!(frames > 900 && frames < 1000, "{frames} frames from 160 at 8 kHz");
        assert_eq!(played[0].rate, 48_000.0, "what played is reported at the speaker's rate");
    }

    #[test]
    fn the_bus_says_what_it_mixed() {
        // The labels survive the mix: what is being heard, by whom, on what
        // frequency and to which group is read off the bus rather than
        // guessed from the audio.
        let (mut n, ins) = bus(2);
        let call = common::Voice {
            system: "M17",
            channel_hz: 433_475_000.0,
            to: Some("ALL".into()),
            from: Some("M0ABC".into()),
            code: None,
            rate: 48_000.0,
            channels: 1,
            pcm: vec![0.25; 48],
        };
        let a = Payload::Voice(vec![voice(48e3, 1, &[0.5; 48])]);
        let b = Payload::Voice(vec![call]);
        let (_, played) = run(&mut n, &ins, &[a, b], 0.001);
        assert_eq!(played.len(), 2);
        assert_eq!(played[1].from.as_deref(), Some("M0ABC"));
        let keys: Vec<String> = n.playing().iter().map(|p| p.key.to_string()).collect();
        assert_eq!(keys, vec!["Audio:145500000:CH1:", "M17:433475000:ALL:M0ABC"]);
        assert_eq!(n.playing()[1].peak, 0.25);
        assert_eq!(n.last_heard(), Some("M0ABC to ALL"));

        // A block with nothing in it played nothing, but the two are still
        // what is playing: speech pauses, and a list that emptied at every
        // pause moved everything under it on the strip.
        let silence = [Payload::Voice(vec![]), Payload::Voice(vec![])];
        let (out, played) = run(&mut n, &ins, &silence, 0.001);
        assert_eq!(out.len(), 96, "a block's worth of silence");
        assert!(played.is_empty());
        assert_eq!(n.playing().len(), 2, "held through a pause");
        assert!(n.playing()[0].quiet_s > 0.0);

        // The pause runs on, and they go.
        for _ in 0..16 {
            run(&mut n, &ins, &silence, 0.1);
        }
        assert!(n.playing().is_empty(), "nothing has played for over a second and a half");

        // A block of silence is not something playing either: every
        // squelched channel on the strip delivers one.
        let quiet = [Payload::Voice(vec![voice(48e3, 1, &[0.0005; 48])]), Payload::Voice(vec![])];
        run(&mut n, &ins, &quiet, 0.001);
        assert!(n.playing().is_empty(), "a block under the floor is not playing");
    }

    #[test]
    fn bare_audio_wired_in_by_hand_is_heard_at_unity() {
        // A chain with no fader in front of it still reaches the speaker,
        // named for what it is: audio, at the frequency its stream says.
        let mut node = BusNode::new(48_000.0);
        let spec = StreamSpec {
            kind: PortKind::Real,
            rate: 48_000.0,
            center: common::Hz(145_500_000),
            ..Default::default()
        };
        let ins = [PortSpec { spec, latency: 0 }];
        node.negotiate(&ins).unwrap();
        let (out, played) = run(&mut node, &ins, &[Payload::Real(vec![0.5, -0.5])], 0.0);
        assert_eq!(out, &[0.5, 0.5, -0.5, -0.5]);
        assert_eq!(played[0].system, super::super::fader::ANALOGUE);
        assert_eq!(played[0].channel_hz, 145_500_000.0);
        assert_eq!(played[0].to, None);
    }

    /// The bus produces exactly real time's worth of audio, block after
    /// block, whatever the block length rounds to.
    #[test]
    fn a_block_is_worth_its_time_in_audio_and_not_a_frame_more() {
        let rate = 20e6;
        let block = 131_072usize;
        let block_s = block as f64 / rate;
        let (mut n, ins) = bus(1);
        let blocks = (1.0 / block_s).round() as usize;
        let mut frames = 0usize;
        for _ in 0..blocks {
            frames += run(&mut n, &ins, &[Payload::Voice(Vec::new())], block_s).0.len() / 2;
        }
        let want = (blocks as f64 * block_s * 48_000.0).round() as usize;
        assert!(
            frames.abs_diff(want) <= 1,
            "{frames} frames for {want} of air: {:.3}% out",
            100.0 * (frames as f64 - want as f64) / want as f64
        );
    }
}
