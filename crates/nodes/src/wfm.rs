//! Broadcast FM as a single node: stereo audio plus RDS.
//!
//! The multiplex carries audio, a 19 kHz pilot, the difference signal on 38 kHz
//! and RDS on 57 kHz, and all of them come off one PLL. Splitting this across
//! separate nodes would mean either running three PLLs on the same pilot or
//! inventing a port type to pass a phase array between them, so it stays one
//! node with the discriminator, stereo decoder and RDS chain inside.
//!
//! The audio port is always two interleaved channels. Mono is the blend
//! reaching zero, not a different output format: changing the channel count
//! mid-stream would mean reopening the audio device every time reception
//! wobbled.

use common::Result;
use dsp::rds::{BlockSync, GroupDecoder, RdsDemod};
use dsp::NoiseMeter;
use dsp::{FmDemod, StereoDecoder};
use pipeline::event::{media, Decoded, Event};
use pipeline::node::{Node, NodeCtx, PortSpec};
use pipeline::param::{Param, ParamValue};
use pipeline::port::{Payload, PortKind, StreamSpec, Tag, TagValue};

/// Peak deviation of broadcast FM.
const DEVIATION_HZ: f64 = 75_000.0;

pub struct WfmDemodNode {
    demod: FmDemod,
    stereo: StereoDecoder,
    noise: NoiseMeter,
    rds: Option<RdsDemod>,
    sync: BlockSync,
    groups: GroupDecoder,
    stereo_enabled: bool,
    rds_enabled: bool,
    mpx: Vec<f32>,
    left: Vec<f32>,
    right: Vec<f32>,
    bits: Vec<u8>,
    /// Last reported values, so a metric is only emitted when it moves.
    last_blend: f32,
    was_locked: bool,
    last_text: Option<String>,
    samples: u64,
}

impl Default for WfmDemodNode {
    fn default() -> Self {
        Self::new()
    }
}

impl WfmDemodNode {
    pub fn new() -> Self {
        Self {
            demod: FmDemod::new(1.0, DEVIATION_HZ),
            stereo: StereoDecoder::new(1.0),
            noise: NoiseMeter::new(1.0),
            rds: None,
            sync: BlockSync::new(),
            groups: GroupDecoder::new(),
            stereo_enabled: true,
            rds_enabled: true,
            mpx: Vec::new(),
            left: Vec::new(),
            right: Vec::new(),
            bits: Vec::new(),
            last_blend: -1.0,
            was_locked: false,
            last_text: None,
            samples: 0,
        }
    }

    pub fn mono(mut self) -> Self {
        self.stereo_enabled = false;
        self
    }

    pub fn without_rds(mut self) -> Self {
        self.rds_enabled = false;
        self
    }

    /// Station information accumulated so far.
    /// Groups decoded, blocks rejected, and whether framing is held.
    pub fn rds_stats(&self) -> (u64, u64, bool) {
        (self.sync.groups, self.sync.errors, self.sync.is_synced())
    }

    pub fn station(&self) -> &dsp::rds::Station {
        self.groups.station()
    }

    /// How much stereo separation is currently applied, 0 mono to 1 full.
    pub fn blend(&self) -> f32 {
        self.stereo.blend()
    }

    fn emit_rds(&mut self, c: &mut NodeCtx<'_>) {
        let center = c.inputs[0].spec.center;
        let at = c.timestamp();
        let before = self.groups.station().clone();
        for b in std::mem::take(&mut self.bits) {
            if let Some(g) = self.sync.push(b) {
                self.groups.push(&g);
            }
        }
        let now = self.groups.station();
        if now.name != before.name || now.radiotext != before.radiotext {
            let text = render(now);
            // Only report when the rendering actually changed, or a station
            // repeating its name every 80 ms would flood the event log.
            if self.last_text.as_deref() != Some(text.as_str()) {
                self.last_text = Some(text.clone());
                c.emit(Event::Decoded(
                    Decoded::bytes("rds", center, at, text.clone().into_bytes())
                        .with_media(media::TEXT)
                        .with_text(text)
                        .with_crc(Some(true)),
                ));
            }
        }
    }
}

fn render(s: &dsp::rds::Station) -> String {
    let mut out = String::new();
    if let Some(pi) = s.pi {
        out.push_str(&format!("PI={pi:04X}"));
    }
    if let Some(n) = &s.name {
        out.push_str(&format!(" \"{n}\""));
    }
    if let Some(p) = s.pty_name() {
        out.push_str(&format!(" [{p}]"));
    }
    if let Some(rt) = &s.radiotext {
        out.push_str(&format!(" {rt}"));
    }
    out
}

impl Node for WfmDemodNode {
    fn name(&self) -> &str {
        "wfm_demod"
    }

    fn as_any(&self) -> Option<&dyn std::any::Any> {
        Some(self)
    }

    fn num_inputs(&self) -> usize {
        1
    }

    fn num_outputs(&self) -> usize {
        1
    }

    fn negotiate(&mut self, inputs: &[PortSpec]) -> Result<Vec<StreamSpec>> {
        let i = inputs[0];
        if i.spec.kind != PortKind::Iq {
            return Err(common::Error::other("wfm_demod needs an IQ input"));
        }
        let rate = i.spec.rate;
        if rate < 130_000.0 {
            // The pilot is at 19 kHz and RDS at 57 kHz, so anything that does
            // not reach past 57 kHz cannot carry them at all.
            return Err(common::Error::other(format!(
                "wfm_demod needs at least 130 kHz to reach the 57 kHz subcarrier, got {rate:.0}"
            )));
        }
        self.demod = FmDemod::new(rate, DEVIATION_HZ);
        self.stereo = StereoDecoder::new(rate);
        self.noise = NoiseMeter::new(rate);
        self.rds = self.rds_enabled.then(|| RdsDemod::new(rate));
        self.sync = BlockSync::new();
        self.groups.reset();
        // Two interleaved channels. The port's sample rate is twice the frame
        // rate, which the channel count now says outright rather than leaving
        // downstream filters to infer it from a rate that looks too high.
        Ok(vec![i
            .spec
            .with_kind(PortKind::Real)
            .with_rate(rate)
            .with_channels(2)])
    }

    fn process(
        &mut self,
        inputs: &[&Payload],
        outputs: &mut [Payload],
        c: &mut NodeCtx<'_>,
    ) -> Result<()> {
        let iq = inputs[0].as_iq().unwrap();
        self.mpx.clear();
        self.demod.process(iq, &mut self.mpx);

        // Measured here because it needs the discriminator output above the
        // audio band, which no longer exists after decimation. Tagged rather
        // than emitted so it rate-scales down the chain to whatever is
        // listening for it.
        let noise = self.noise.process(&self.mpx);
        c.tag(Tag::new(
            self.samples * 2,
            "noise",
            TagValue::Float(noise as f64),
        ));

        if self.stereo_enabled {
            self.stereo
                .process(&self.mpx, &mut self.left, &mut self.right);
        } else {
            self.stereo.process_mono(&self.mpx, &mut self.left);
            self.right.clear();
            self.right.extend_from_slice(&self.left);
        }

        let out = outputs[0].real_mut();
        out.reserve(self.left.len() * 2);
        for (l, r) in self.left.iter().zip(&self.right) {
            out.push(*l);
            out.push(*r);
        }

        if self.rds_enabled {
            if let Some(rds) = &mut self.rds {
                self.bits.clear();
                rds.process(&self.mpx, self.stereo.phases(), &mut self.bits);
                self.emit_rds(c);
            }
        }

        let locked = self.stereo.is_locked();
        if locked != self.was_locked {
            // Tag the exact sample, so anything downstream knows where the
            // transition landed rather than only that it happened.
            c.tag(Tag::new(
                self.samples * 2,
                "stereo_lock",
                TagValue::Int(locked as i64),
            ));
            self.was_locked = locked;
        }
        let blend = self.stereo.blend();
        if (blend - self.last_blend).abs() > 0.02 {
            self.last_blend = blend;
            c.emit(Event::Metric {
                name: "stereo_blend",
                value: blend as f64,
            });
        }
        self.samples += self.left.len() as u64;
        Ok(())
    }

    fn reset(&mut self) {
        self.demod.reset();
        self.stereo.reset();
        if let Some(r) = &mut self.rds {
            r.reset();
        }
        self.sync = BlockSync::new();
        self.groups.reset();
        self.last_blend = -1.0;
        self.was_locked = false;
        self.last_text = None;
        self.samples = 0;
    }

    fn params(&self) -> Vec<Param> {
        vec![
            Param::bool("stereo", self.stereo_enabled).label("Stereo"),
            Param::bool("rds", self.rds_enabled).label("RDS"),
        ]
    }

    fn set_param(&mut self, name: &str, v: ParamValue) -> Result<()> {
        match name {
            "stereo" => {
                self.stereo_enabled = v.as_bool().unwrap_or(true);
                Ok(())
            }
            "rds" => {
                self.rds_enabled = v.as_bool().unwrap_or(true);
                Ok(())
            }
            _ => Err(common::Error::other(format!(
                "wfm_demod: unknown parameter {name:?}"
            ))),
        }
    }
}
