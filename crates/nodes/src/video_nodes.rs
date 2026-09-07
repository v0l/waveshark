//! Analogue video as a graph node.
//!
//! Wiring only, like `ble_nodes`: the sync separation, the field assembly and
//! the colour demodulation are `dsp::video`, the channel plan is
//! `decode::fpv`, and neither knows about pipelines.
//!
//! What the node adds is what the receiver needs and a demodulator does not
//! have: the FM demodulator in front, the standard measured from the line
//! period rather than configured, and a port that carries whole fields with
//! the channel they came from.

use common::{Pixels, Result, VideoFrame};
use dsp::video::{Standard, SyncSeparator};
use pipeline::node::{NodeCtx, PortSpec, Simple};
use pipeline::param::{Param, ParamValue};
use pipeline::port::{Payload, PortKind, StreamSpec};

/// What the picture is resampled to. A PAL line holds about 720 samples at
/// broadcast rates and an FPV camera rather fewer, so this is a choice rather
/// than a measurement.
const WIDTH: usize = 640;

/// Peak deviation mapped to full scale. Only the contrast depends on it, and
/// the separator normalises again from the sync tip, so it need not be exact:
/// the AKK transmitter measured about 1 MHz rms.
const DEVIATION_HZ: f64 = 6e6;

pub struct FpvNode {
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
    sequence: u64,
    fields: u64,
}

impl Default for FpvNode {
    fn default() -> Self {
        Self::new(None, true)
    }
}

impl FpvNode {
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

    /// Measure the line period and name the standard.
    ///
    /// PAL and NTSC are 0.7% apart, which no transmitter's timebase error
    /// reaches, so this settles it. The sync pulses have to be found on a
    /// filtered copy for the same reason the separator filters: a 20 MHz
    /// baseband's noise breaks every run otherwise.
    fn measure(&self, base: &[f32]) -> Option<Standard> {
        let mut sorted: Vec<f32> = base.to_vec();
        sorted.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        let thresh = (sorted[sorted.len() / 50] + sorted[sorted.len() * 13 / 100]) / 2.0;
        let taps = ((0.25e-6 * self.rate) as usize).max(1);
        let (mut edges, mut low) = (Vec::new(), 0usize);
        for (i, w) in base.windows(taps).enumerate() {
            let mean = w.iter().sum::<f32>() / taps as f32;
            if mean < thresh {
                low += 1;
            } else {
                if ((2e-6 * self.rate) as usize..=(8e-6 * self.rate) as usize).contains(&low) {
                    edges.push(i);
                }
                low = 0;
            }
        }
        if edges.len() < 100 {
            return None;
        }
        let mut gaps: Vec<f64> = edges
            .windows(2)
            .map(|w| (w[1] - w[0]) as f64 / self.rate)
            .collect();
        gaps.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        Standard::from_line_period(gaps[gaps.len() / 2])
    }
}

impl Simple for FpvNode {
    fn name(&self) -> &str {
        "fpv"
    }

    fn negotiate(&mut self, i: &PortSpec) -> Result<StreamSpec> {
        if i.spec.kind != PortKind::Iq {
            return Err(common::Error::other("fpv reads complex baseband"));
        }
        // A PAL luma signal reaches 5 MHz and the colour subcarrier sits at
        // 4.43, so a span narrower than this cannot hold a picture.
        if i.spec.rate < 12e6 {
            return Err(common::Error::other(
                "fpv needs at least 12 MS/s to hold a video baseband",
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
        self.base.clear();
        self.demod.process(iq, &mut self.base);

        if self.sep.is_none() {
            // Two fields' worth before deciding, so the measurement has
            // hundreds of lines behind it rather than a handful.
            self.priming.extend_from_slice(&self.base);
            if self.priming.len() as f64 <= 0.04 * self.rate {
                return Ok(());
            }
            let priming = std::mem::take(&mut self.priming);
            let Some(std) = self.measure(&priming) else {
                // Not video, or not yet. Drop what was kept rather than
                // growing without bound on a channel that never locks.
                return Ok(());
            };
            let s = SyncSeparator::new(self.rate, std, WIDTH);
            self.sep = Some(if self.colour { s.with_colour() } else { s });
            // The samples that decided it are still video, so they are read
            // rather than thrown away.
            self.base.splice(0..0, priming);
        }
        let Some(sep) = self.sep.as_mut() else {
            return Ok(());
        };

        let mut fields = Vec::new();
        sep.process(&self.base, &mut fields);
        let label = decode::fpv::name_at(self.center_hz as u64, 3_000_000);
        let out = o.video_mut();
        for f in fields {
            self.fields += 1;
            self.sequence += 1;
            let (pixels, samples) = match f.rgb {
                Some(rgb) => (Pixels::Rgb8, rgb),
                None => (Pixels::Luma8, f.luma),
            };
            out.push(VideoFrame {
                system: "FPV",
                channel_hz: self.center_hz,
                label: label.clone(),
                width: f.width,
                height: f.height,
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
            other => return Err(common::Error::other(format!("fpv has no {other}"))),
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
        let mut n = FpvNode::default();
        assert!(n.negotiate(&spec(20e6, 5_865e6)).is_ok());
        assert!(n.negotiate(&spec(2e6, 5_865e6)).is_err());
    }

    /// The whole path: a transmitter's carrier in, fields out, on a port that
    /// carries the channel they came from.
    #[test]
    fn a_modulated_camera_comes_back_as_fields() {
        let rate = 16e6;
        let iq = modulate(&pal(rate, 4), rate);
        let mut n = FpvNode::new(None, false);
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
        assert_eq!(f.system, "FPV");
        assert_eq!(f.pixels, Pixels::Luma8);
        assert_eq!(f.samples.len(), f.width * f.height);
        assert!(f.completeness() > 0.8, "{} of {} lines", f.lines_seen, f.height);
        // 5865 MHz is two channels of the plan and the frame says both.
        assert_eq!(f.label.as_deref(), Some("A1 or B8"));
        // Sequence numbers so a viewer can tell a still picture from a
        // repeated one.
        assert!(fields.windows(2).all(|w| w[1].sequence > w[0].sequence));
    }
}
