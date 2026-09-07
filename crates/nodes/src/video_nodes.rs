//! Analogue video as a graph node.
//!
//! Wiring only, like `ble_nodes`: the sync separation, the field assembly and
//! the colour demodulation are `dsp::video`, and it knows nothing about
//! pipelines. The one thing here that is specific to a band rather than to
//! video is the 5.8 GHz channel plan in `decode::video_channels`, which names
//! a frequency where that band has a naming convention and leaves the label
//! empty where it does not.
//!
//! What the node adds is what the receiver needs and a demodulator does not
//! have: the FM demodulator in front, the standard measured from the line
//! period rather than configured, and a port that carries whole fields with
//! the channel they came from.

use common::{Pixels, Result, VideoFrame};
use dsp::video::{find_lines, Lock, Standard, SyncSeparator};
use pipeline::node::{NodeCtx, PortSpec, Simple};
use pipeline::param::{Param, ParamValue};
use pipeline::port::{Payload, PortKind, StreamSpec};

/// What the picture is resampled to. A PAL line holds about 720 samples at
/// broadcast rates and a small camera rather fewer, so this is a choice rather
/// than a measurement.
const WIDTH: usize = 640;

/// Peak deviation mapped to full scale. Only the contrast depends on it, and
/// the separator normalises again from the sync tip, so it need not be exact:
/// the 5.8 GHz transmitter measured here was about 1 MHz rms.
const DEVIATION_HZ: f64 = 6e6;

pub struct VideoNode {
    demod: dsp::FmDemod,
    sep: Option<SyncSeparator>,
    /// Set by hand, or `None` to measure it from the line period.
    forced: Option<Standard>,
    colour: bool,
    rate: f64,
    center_hz: f64,
    base: Vec<f32>,
    /// Samples kept while the standard is still being measured.
    priming: Vec<f32>,
    /// Samples to skip before looking again, after a look that found no line
    /// rate.
    ///
    /// A source can be megahertz wide and not be video, and the receiver
    /// places this front end on width alone, so most of what reaches here on
    /// a busy band is not a camera. Measuring 40 ms and then waiting a second
    /// costs a twenty-fifth of the demodulation while still finding a picture
    /// within a second of it starting.
    backoff: usize,
    /// What the last successful look found, for a caller that wants to know
    /// why there is a picture or why there is not.
    lock: Option<Lock>,
    sequence: u64,
    fields: u64,
}

impl Default for VideoNode {
    fn default() -> Self {
        Self::new(None, true)
    }
}

impl VideoNode {
    pub fn new(forced: Option<Standard>, colour: bool) -> Self {
        Self {
            demod: dsp::FmDemod::new(20e6, DEVIATION_HZ),
            sep: None,
            forced,
            colour,
            rate: 20e6,
            center_hz: 0.0,
            base: Vec::new(),
            priming: Vec::new(),
            backoff: 0,
            lock: None,
            sequence: 0,
            fields: 0,
        }
    }

    /// Fields assembled since the node was built.
    pub fn fields(&self) -> u64 {
        self.fields
    }

    /// The standard in use, once it is known.
    pub fn standard(&self) -> Option<Standard> {
        self.sep.as_ref().map(|s| s.standard())
    }

    /// What the line-rate test found, if anything: how many sync pulses and
    /// how well they agreed. Empty on a source that is not video, which is
    /// most of the wide ones.
    pub fn lock(&self) -> Option<Lock> {
        self.lock
    }

}

impl VideoNode {
    /// Whether it is reading a picture right now.
    ///
    /// Asked by the auto node: while a camera is locked, the span is that
    /// camera, and every source the detector finds inside its carrier is a
    /// piece of it. Opening those costs an extraction and a set of front ends
    /// each, and produces rows for sensors that are not there.
    pub fn locked(&self) -> bool {
        self.sep.is_some() && self.lock.is_some()
    }

}

impl Simple for VideoNode {
    fn name(&self) -> &str {
        "video"
    }

    fn negotiate(&mut self, i: &PortSpec) -> Result<StreamSpec> {
        if i.spec.kind != PortKind::Iq {
            return Err(common::Error::other("video reads complex baseband"));
        }
        // A PAL luma signal reaches 5 MHz and the colour subcarrier sits at
        // 4.43, so a span narrower than this cannot hold a picture.
        if i.spec.rate < 12e6 {
            return Err(common::Error::other(
                "analogue video needs at least 12 MS/s to hold its baseband",
            ));
        }
        self.rate = i.spec.rate;
        self.center_hz = i.spec.center.as_f64();
        self.demod = dsp::FmDemod::new(self.rate, DEVIATION_HZ);
        self.sep = self.forced.map(|std| {
            let s = SyncSeparator::new(self.rate, std, WIDTH);
            if self.colour {
                s.with_colour()
            } else {
                s
            }
        });
        self.priming.clear();
        let mut out = i.spec.with_kind(PortKind::Video);
        // A field is not a sampled stream, so the rate the graph negotiated
        // means nothing downstream; the frame carries its own geometry.
        out.rate = 0.0;
        Ok(out)
    }

    fn process(&mut self, i: &Payload, o: &mut Payload, _c: &mut NodeCtx<'_>) -> Result<()> {
        let Some(iq) = i.as_iq() else { return Ok(()) };
        // Before the demodulation and not after it: this runs on the whole
        // span, so a span with no camera in it would otherwise FM demodulate
        // twenty megasamples a second to decide that again every block.
        if self.sep.is_none() && self.backoff > 0 {
            self.backoff = self.backoff.saturating_sub(iq.len());
            return Ok(());
        }
        self.base.clear();
        self.demod.process(iq, &mut self.base);

        if self.sep.is_none() {
            // Two fields' worth before deciding, so the test has hundreds of
            // lines to judge rather than a handful.
            self.priming.extend_from_slice(&self.base);
            if self.priming.len() as f64 <= 0.04 * self.rate {
                return Ok(());
            }
            let priming = std::mem::take(&mut self.priming);
            let Some(lock) = find_lines(&priming, self.rate) else {
                // Not a camera. Wait before looking again rather than
                // demodulating every block of a wide source that will never
                // be one.
                self.lock = None;
                self.backoff = self.rate as usize;
                return Ok(());
            };
            let s = SyncSeparator::new(self.rate, lock.standard, WIDTH);
            self.sep = Some(if self.colour { s.with_colour() } else { s });
            self.lock = Some(lock);
            // The samples that decided it are still video, so they are read
            // rather than thrown away.
            self.base.splice(0..0, priming);
        }
        let Some(sep) = self.sep.as_mut() else {
            return Ok(());
        };

        let mut fields = Vec::new();
        sep.process(&self.base, &mut fields);
        let label = decode::video_channels::name_at(self.center_hz as u64, 3_000_000);
        let out = o.video_mut();
        for f in fields {
            self.fields += 1;
            self.sequence += 1;
            let (pixels, samples) = match f.rgb {
                Some(rgb) => (Pixels::Rgb8, rgb),
                None => (Pixels::Luma8, f.luma),
            };
            out.push(VideoFrame {
                system: "analogue video",
                channel_hz: self.center_hz,
                label: label.clone(),
                width: f.width,
                height: f.height,
                aspect: f.aspect,
                pixels,
                samples: std::sync::Arc::new(samples),
                lines_seen: f.lines_seen,
                sequence: self.sequence,
            });
        }
        Ok(())
    }

    fn reset(&mut self) {
        self.demod.reset();
        self.priming.clear();
        self.backoff = 0;
        self.lock = None;
        if let Some(s) = self.sep.as_mut() {
            s.reset();
        }
    }

    fn params(&self) -> Vec<Param> {
        vec![
            Param::bool("colour", self.colour).label("Colour"),
            Param::choice(
                "standard",
                match self.forced {
                    None => 0,
                    Some(Standard::Pal) => 1,
                    Some(Standard::Ntsc) => 2,
                },
                ["auto", "pal", "ntsc"].map(String::from).to_vec(),
            )
            .label("Standard"),
        ]
    }

    fn set_param(&mut self, name: &str, v: ParamValue) -> Result<()> {
        match name {
            "colour" => {
                self.colour = v.as_bool().unwrap_or(true);
                // The separator is built with colour on or off, so it has to
                // be rebuilt; the next block primes it again.
                self.sep = None;
            }
            "standard" => {
                self.forced = match &v {
                    ParamValue::Choice(1) => Some(Standard::Pal),
                    ParamValue::Choice(2) => Some(Standard::Ntsc),
                    ParamValue::Text(t) if t == "pal" => Some(Standard::Pal),
                    ParamValue::Text(t) if t == "ntsc" => Some(Standard::Ntsc),
                    _ => None,
                };
                self.sep = None;
            }
            other => return Err(common::Error::other(format!("video has no {other}"))),
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use common::{Hz, C32};

    fn spec(rate: f64, center: f64) -> PortSpec {
        PortSpec {
            spec: StreamSpec::iq(rate, Hz(center as u64)),
            latency: 0,
        }
    }

    /// Modulate composite video onto a carrier the way a transmitter does,
    /// so the node is fed what a radio hands it rather than a baseband.
    fn modulate(base: &[f32], rate: f64) -> Vec<C32> {
        let mut phase = 0.0f64;
        base.iter()
            .map(|&v| {
                phase += std::f64::consts::TAU * f64::from(v) * DEVIATION_HZ / rate;
                C32::new(phase.cos() as f32, phase.sin() as f32)
            })
            .collect()
    }

    /// Composite PAL with a ramp on every line, built the way a camera builds
    /// it.
    fn pal(rate: f64, fields: usize) -> Vec<f32> {
        let std = Standard::Pal;
        let n = |s: f64| (s * rate).round() as usize;
        let mut out = Vec::new();
        for _ in 0..fields {
            for _ in 0..std.active_lines() {
                out.extend(std::iter::repeat_n(-0.3f32, n(std.sync_s())));
                out.extend(std::iter::repeat_n(0.0f32, n(std.back_porch_s())));
                let active = n(std.active_s());
                for i in 0..active {
                    out.push(i as f32 / active as f32 * 0.7);
                }
                let used = n(std.sync_s()) + n(std.back_porch_s()) + active;
                out.extend(std::iter::repeat_n(0.0f32, n(std.line_s()).saturating_sub(used)));
            }
            out.extend(std::iter::repeat_n(-0.3f32, n(27.3e-6)));
            out.extend(std::iter::repeat_n(0.0f32, n(std.line_s())));
        }
        out
    }

    #[test]
    fn the_node_refuses_a_span_too_narrow_for_a_picture() {
        let mut n = VideoNode::default();
        assert!(n.negotiate(&spec(20e6, 5_865e6)).is_ok());
        assert!(n.negotiate(&spec(2e6, 5_865e6)).is_err());
    }

    /// The whole path: a transmitter's carrier in, fields out, on a port that
    /// carries the channel they came from.
    #[test]
    fn a_modulated_camera_comes_back_as_fields() {
        let rate = 16e6;
        let iq = modulate(&pal(rate, 4), rate);
        let mut n = VideoNode::new(None, false);
        let out_spec = n.negotiate(&spec(rate, 5_865e6)).expect("a span");
        assert_eq!(out_spec.kind, PortKind::Video);

        let ins = [spec(rate, 5_865e6)];
        let tags = Vec::new();
        let mut out = Payload::empty_of(PortKind::Video);
        let mut fields: Vec<VideoFrame> = Vec::new();
        for chunk in iq.chunks(1 << 16) {
            let mut events = Vec::new();
            let mut new_tags = Vec::new();
            let mut ctx = NodeCtx::new(0, &ins, &tags, &mut events, &mut new_tags);
            out.clear();
            n.process(&Payload::Iq(chunk.to_vec()), &mut out, &mut ctx)
                .expect("process");
            fields.extend(out.as_video().unwrap_or(&[]).iter().cloned());
        }
        assert!(!fields.is_empty(), "no field reached the port");
        assert_eq!(n.standard(), Some(Standard::Pal), "the standard was measured");

        let f = fields.iter().max_by_key(|f| f.lines_seen).expect("a field");
        assert_eq!(f.system, "analogue video");
        assert_eq!(f.pixels, Pixels::Luma8);
        assert_eq!(f.samples.len(), f.width * f.height);
        assert!(f.completeness() > 0.8, "{} of {} lines", f.lines_seen, f.height);
        // 5865 MHz is two channels of the plan pilots share, and the frame
        // says both, since nothing in the signal chooses between them.
        assert_eq!(f.label.as_deref(), Some("A1 or B8"));
        // Sequence numbers so a viewer can tell a still picture from a
        // repeated one.
        assert!(fields.windows(2).all(|w| w[1].sequence > w[0].sequence));
    }
}
