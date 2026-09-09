//! An analogue channel as a voice front end.
//!
//! A DMR or M17 channel puts its speech on the bus, so it appears in the call
//! list, is recorded with the call, and can be transcribed. An FM channel on
//! the strip did none of that: its audio went to the speaker and nowhere
//! else, so the one kind of transmission a scanner exists for was the one
//! kind the receiver could not tell you about afterwards.
//!
//! This is what a channel marked as voice adds to its own chain. It ends the
//! over the way an operator would hear it end, on the squelch closing, and
//! then puts the whole transmission on the bus as a packet carrying its
//! speech, measured on the channel it was heard on.
//!
//! Two inputs, because the two questions need different samples: what was
//! said is in the audio, and how strong it was is in the IF. A level taken
//! off demodulated audio is a level of the demodulator's gain structure and
//! says nothing about the transmitter.
//!
//! The audio is input 0 and the IF is input 1, and that order is not
//! arbitrary: the graph hands a node the tags of its first input only, and
//! the squelch's own open-or-shut decision is a tag on the audio chain.
//! Wired the other way round this node never saw it and judged the audio's
//! level for itself instead, which is a different decision from the one the
//! operator hears.

use common::{Decoded, Frame, Packet, Result, Speech, Value};
use pipeline::event::media;
use pipeline::node::{Node, NodeCtx, PortSpec};
use pipeline::param::{Param, ParamValue};
use pipeline::port::{Payload, PortKind, StreamSpec, TagValue};
use std::sync::Arc;

use crate::frame_meter::FrameMeter;

/// The system name a channel's calls are listed under.
pub const SYSTEM: &str = "Voice";

/// Longest transmission kept in one piece, in seconds. A repeater left keyed
/// open would otherwise grow this without bound; the call is split and the
/// halves show as two overs.
const MAX_OVER_S: f64 = 300.0;

pub struct VoiceChannelNode {
    channel_hz: f64,
    label: String,
    /// Seconds of silence that end an over. Long enough to survive the
    /// squelch chattering on a mobile at the edge of the repeater, short
    /// enough that a reply is a new row.
    hang_s: f64,
    /// Shortest over worth a row. A squelch crash is not a transmission.
    min_over_s: f64,
    meter: FrameMeter,
    audio_rate: f64,
    bandwidth_hz: f64,
    /// The over being collected, and how long it has been quiet.
    over: Vec<f32>,
    /// Samples of it the squelch was open for. The hang time is collected
    /// too, so the length of `over` is not how long anybody was talking, and
    /// judging a squelch crash by that let every one of them through.
    voiced: usize,
    quiet_s: f64,
    /// Whether an over is being collected. Not the same as the squelch being
    /// open: the hang time is part of the over, so this outlives the gate.
    in_over: bool,
    /// Whether the squelch upstream has ever tagged the stream. Without one
    /// the gate falls back to the audio's own level, which is what a channel
    /// with the squelch switched off gives us.
    tagged: bool,
    at_us: u64,
    started_us: u64,
}

impl VoiceChannelNode {
    pub fn new(channel_hz: f64, label: impl Into<String>) -> Self {
        Self {
            channel_hz,
            label: label.into(),
            hang_s: 0.7,
            min_over_s: 0.3,
            meter: FrameMeter::new(1.0, channel_hz as u64, 0.0),
            audio_rate: 48_000.0,
            bandwidth_hz: 0.0,
            over: Vec::new(),
            voiced: 0,
            quiet_s: 0.0,
            in_over: false,
            tagged: false,
            at_us: 0,
            started_us: 0,
        }
    }

    /// How long somebody was talking, which is not how long the buffer is.
    fn seconds(&self) -> f64 {
        self.voiced as f64 / self.audio_rate.max(1.0)
    }

    /// The packet that ends an over: the whole transmission, at what the
    /// channel measured while it ran.
    fn close(&mut self, packets: &mut Vec<Packet>) {
        let seconds = self.seconds();
        let pcm = std::mem::take(&mut self.over);
        self.voiced = 0;
        if seconds < self.min_over_s {
            return;
        }
        let frame = self.meter.frame(Vec::new());
        let audio = Arc::new(Speech { pcm, rate: self.audio_rate });
        let mut p = Packet::of_frame(self.started_us, self.channel_hz as u32, frame.clone());
        p.audio = Some(audio.clone());
        p.decodes.push(self.decode(seconds, false).with_audio(Some(audio)).with_level(
            frame.rssi_dbfs,
            frame.snr_db,
        ));
        packets.push(p);
    }

    fn decode(&self, seconds: f64, live: bool) -> Decoded {
        let mut d = Decoded::bytes(
            "voice",
            common::Hz(self.channel_hz as u64),
            self.at_us as f64 / 1e6,
            Vec::new(),
        );
        d.media_type = media::TEXT;
        d.modulation = Some(common::Modulation::Fm);
        d.bandwidth_hz = Some(self.meter_bandwidth());
        d.fields = vec![
            ("voice".into(), Value::Bool(true)),
            ("to".into(), Value::Text(self.label.clone())),
            ("seconds".into(), Value::Float(seconds)),
            ("live".into(), Value::Bool(live)),
        ];
        d
    }

    /// What the channel was cut to, which the IF port said at negotiation.
    fn meter_bandwidth(&self) -> f64 {
        self.bandwidth_hz
    }
}

impl Node for VoiceChannelNode {
    fn name(&self) -> &str {
        "voice"
    }

    fn num_inputs(&self) -> usize {
        2
    }

    fn num_outputs(&self) -> usize {
        2
    }

    fn negotiate(&mut self, i: &[PortSpec]) -> Result<Vec<StreamSpec>> {
        let (audio, iq) = match (i.first(), i.get(1)) {
            (Some(a), Some(b)) => (a, b),
            _ => return Err(common::Error::other("voice needs the channel audio and its IF")),
        };
        if audio.spec.kind != PortKind::Real {
            return Err(common::Error::other("voice input 0 is the channel's audio"));
        }
        if iq.spec.kind != PortKind::Iq {
            return Err(common::Error::other("voice input 1 is the channel IF, before demodulation"));
        }
        self.audio_rate = audio.spec.rate;
        self.meter = FrameMeter::new(iq.spec.rate, self.channel_hz as u64, 0.0);
        self.bandwidth_hz = if iq.spec.bandwidth > 0.0 { iq.spec.bandwidth } else { iq.spec.rate };

        let mut packets = audio.spec.with_kind(PortKind::Packets);
        packets.center = common::Hz(self.channel_hz as u64);
        packets.rate = 0.0;
        let mut voice = packets.with_kind(PortKind::Voice);
        voice.rate = self.audio_rate;
        Ok(vec![packets, voice])
    }

    fn process(
        &mut self,
        inputs: &[&Payload],
        outputs: &mut [Payload],
        c: &mut NodeCtx<'_>,
    ) -> Result<()> {
        if let Some(iq) = inputs.get(1).and_then(|p| p.as_iq()) {
            self.meter.feed(iq);
        }
        let audio = inputs[0].as_real().unwrap_or(&[]);
        let block_s = audio.len() as f64 / self.audio_rate.max(1.0);
        self.at_us += (block_s * 1e6) as u64;

        // The squelch's own decision when there is one upstream. Its tag is
        // the same decision the operator hears, including the hysteresis it
        // applies, which no measurement taken here could reproduce.
        let mut open = None;
        for t in c.in_tags.iter().filter(|t| t.key == "squelch_open") {
            if let TagValue::Int(v) = t.value {
                open = Some(v != 0);
                self.tagged = true;
            }
        }
        let open = match open {
            Some(v) => v,
            None if self.tagged => self.in_over && self.quiet_s == 0.0,
            // No squelch in the chain: the audio's own level decides, which
            // is the same question asked a cruder way.
            None => rms(audio) > 1e-4,
        };

        let packets = outputs[0].packets_mut();
        if open {
            if !self.in_over {
                self.started_us = self.at_us;
                let mut p = Packet::of_frame(self.at_us, self.channel_hz as u32, Frame {
                    bytes: Vec::new(),
                    center_hz: self.channel_hz as u64,
                    rssi_dbfs: self.meter.rssi_dbfs(),
                    snr_db: self.meter.snr_db(),
                    iq: None,
                });
                p.decodes.push(
                    self.decode(0.0, true)
                        .with_level(self.meter.rssi_dbfs(), self.meter.snr_db()),
                );
                packets.push(p);
            }
            self.in_over = true;
            self.quiet_s = 0.0;
            self.over.extend_from_slice(audio);
            self.voiced += audio.len();
            if self.seconds() > MAX_OVER_S {
                self.close(packets);
            }
        } else if self.in_over {
            self.quiet_s += block_s;
            // The hang time is collected too: cutting on the first quiet
            // block clips the last syllable of every over.
            self.over.extend_from_slice(audio);
            if self.quiet_s >= self.hang_s {
                self.close(packets);
                self.in_over = false;
            }
        }

        // The speaker's copy, this block's worth, with the call it belongs
        // to. The strip listens here rather than to the audio directly, so a
        // channel marked as voice can be subscribed to by group like any
        // other system's traffic.
        let voice = outputs[1].voice_mut();
        voice.push(common::Voice {
            system: SYSTEM,
            channel_hz: self.channel_hz,
            to: Some(self.label.clone()),
            from: None,
            rate: self.audio_rate,
            pcm: audio.to_vec(),
        });
        Ok(())
    }

    fn reset(&mut self) {
        self.over.clear();
        self.voiced = 0;
        self.in_over = false;
        self.quiet_s = 0.0;
        self.meter.reset();
    }

    fn params(&self) -> Vec<Param> {
        vec![
            Param::float("hang_s", self.hang_s, 0.0..=5.0).unit("s").label("Silence that ends an over"),
            Param::float("min_over_s", self.min_over_s, 0.0..=10.0)
                .unit("s")
                .label("Shortest over worth a row"),
        ]
    }

    fn set_param(&mut self, name: &str, v: ParamValue) -> Result<()> {
        match name {
            "hang_s" => self.hang_s = v.as_f64().unwrap_or(self.hang_s),
            "min_over_s" => self.min_over_s = v.as_f64().unwrap_or(self.min_over_s),
            "channel_hz" => self.channel_hz = v.as_f64().unwrap_or(self.channel_hz),
            "label" => self.label = v.as_str().unwrap_or(&self.label).to_string(),
            _ => return Err(common::Error::other(format!("voice: unknown parameter {name:?}"))),
        }
        Ok(())
    }
}

fn rms(x: &[f32]) -> f32 {
    if x.is_empty() {
        return 0.0;
    }
    (x.iter().map(|v| v * v).sum::<f32>() / x.len() as f32).sqrt()
}

#[cfg(test)]
mod tests {
    use super::*;
    use common::{Hz, C32};
    use pipeline::port::Tag;

    const IF_RATE: f64 = 96_000.0;
    const AUDIO_RATE: f64 = 48_000.0;

    fn specs() -> Vec<PortSpec> {
        let mut iq = StreamSpec::iq(IF_RATE, Hz(145_000_000));
        iq.bandwidth = 12_500.0;
        let audio = iq.with_kind(PortKind::Real).with_rate(AUDIO_RATE);
        // Audio first, as the graph wires it: the squelch's tag rides the
        // audio chain and a node is handed the tags of its first input.
        vec![PortSpec { spec: audio, latency: 0 }, PortSpec { spec: iq, latency: 0 }]
    }

    /// One block through the node, with the squelch saying whether the
    /// channel is open.
    fn block(n: &mut VoiceChannelNode, open: bool, level: f32, samples: usize) -> Vec<Packet> {
        // The IF carries the channel's noise whether or not the squelch is
        // open, which is what the floor is measured from.
        let iq = Payload::Iq(vec![C32::new(level.max(0.002), 0.0); samples * 2]);
        let audio = Payload::Real(vec![level; samples]);
        let ins: Vec<&Payload> = vec![&audio, &iq];
        let mut outs = vec![Payload::Packets(Vec::new()), Payload::Voice(Vec::new())];
        let specs = specs();
        let tags = vec![Tag::new(0, "squelch_open", TagValue::Int(open as i64))];
        let mut events = Vec::new();
        let mut new_tags = Vec::new();
        let mut c = NodeCtx::new(0, &specs, &tags, &mut events, &mut new_tags);
        n.process(&ins, &mut outs, &mut c).unwrap();
        match outs.remove(0) {
            Payload::Packets(p) => p,
            _ => unreachable!(),
        }
    }

    fn node() -> VoiceChannelNode {
        let mut n = VoiceChannelNode::new(145_500_000.0, "CH1");
        n.negotiate(&specs()).unwrap();
        n
    }

    /// The over ends when the squelch closes, and what comes out is the whole
    /// transmission with what it was heard at.
    #[test]
    fn an_over_becomes_one_packet_carrying_its_speech() {
        let mut n = node();
        let mut opened = block(&mut n, true, 0.5, 48_000);
        assert_eq!(opened.len(), 1, "the call is announced as it starts");
        assert!(opened.remove(0).decodes[0].field("live").is_some());

        // A second of speech, then silence past the hang time.
        block(&mut n, true, 0.5, 48_000);
        let mut done = Vec::new();
        for _ in 0..4 {
            done.extend(block(&mut n, false, 0.0, 24_000));
        }
        assert_eq!(done.len(), 1, "one over, one packet");
        let p = &done[0];
        let speech = p.audio.as_ref().expect("the speech is the payload");
        assert!(speech.seconds() >= 2.0, "{} s of speech", speech.seconds());
        let d = &p.decodes[0];
        assert_eq!(d.field("voice").map(|v| v.to_string()).as_deref(), Some("true"));
        assert!(d.rssi_dbfs.is_some_and(|v| v.is_finite()), "a call without a level is half a row");
        assert!(d.snr_db.is_some_and(|v| v.is_finite()));
    }

    /// A squelch crash is not a transmission.
    #[test]
    fn a_burst_shorter_than_the_minimum_leaves_no_call() {
        let mut n = node();
        block(&mut n, true, 0.5, 2_400);
        let mut out = Vec::new();
        for _ in 0..4 {
            out.extend(block(&mut n, false, 0.0, 24_000));
        }
        assert!(out.iter().all(|p| p.audio.is_none()), "nothing was said");
    }
}
