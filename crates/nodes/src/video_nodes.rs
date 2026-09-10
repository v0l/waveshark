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

use crate::protocol::{Placed, Placement, Protocol, Shape, Stickiness};
use crate::NodeSpec;
use common::{Pixels, Result, VideoFrame};
use dsp::video::{find_lines, Lock, Standard, SyncSeparator};
use pipeline::event::Request;
use pipeline::node::{NodeCtx, PortSpec};
use pipeline::param::{Param, ParamValue};
use pipeline::port::{Payload, PortKind, StreamSpec};
use pipeline::registry::{Category, Settings, SettingsExt, StageDesc};

/// What a camera's transmission is called, on a frame and on the sound that
/// came with it.
const SYSTEM: &str = "analogue video";

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
    /// The picture's own filter, when the span is wider than the picture
    /// needs.
    ///
    /// The two halves of a camera's transmission want different bandwidths
    /// and a chain cannot fork, so the narrowing happens here rather than as
    /// a stage in front. The picture is read from 10 MS/s, because a
    /// discriminator handed the whole 20 carries all of that bandwidth's
    /// noise into it and a weak link then produces no sync pulse at all. The
    /// sound is on a 6.5 MHz subcarrier, which only exists in the wide
    /// baseband: filtered to 10 MS/s it is gone.
    narrow: Option<dsp::fir::FirDecim>,
    narrowed: Vec<common::C32>,
    /// The sound subcarrier, and the discriminator that reaches it.
    sound: Option<dsp::video::Sound>,
    wide_demod: dsp::FmDemod,
    wide: Vec<f32>,
    pcm: Vec<f32>,
    sep: Option<SyncSeparator>,
    /// Set by hand, or `None` to measure it from the line period.
    forced: Option<Standard>,
    colour: bool,
    /// The rate the picture is read at, after narrowing.
    rate: f64,
    /// The rate the node is fed at, which is what it claims and what the
    /// sound is read from.
    span_hz: f64,
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
    /// Samples since the separator last produced a field.
    quiet: usize,
    sequence: u64,
    fields: u64,
}

/// How long a locked separator may produce nothing before the lock is
/// dropped and the standard measured again.
///
/// The lock is not evidence that lasts: it is one measurement of one 40 ms
/// window, and a window of noise that happens to score can hold the span for
/// the rest of the session, since the claim keeps the detector and every
/// other span-wide decoder out of it. Three seconds is far longer than any
/// fade a picture rides through at fifty fields a second, and short enough
/// that a wrong lock costs a few seconds of the band rather than all of it.
const RELOCK_S: f64 = 3.0;

impl Default for VideoNode {
    fn default() -> Self {
        Self::new(None, true)
    }
}

impl VideoNode {
    pub fn new(forced: Option<Standard>, colour: bool) -> Self {
        Self {
            demod: dsp::FmDemod::new(20e6, DEVIATION_HZ),
            narrow: None,
            narrowed: Vec::new(),
            sound: None,
            wide_demod: dsp::FmDemod::new(20e6, DEVIATION_HZ),
            wide: Vec::new(),
            pcm: Vec::new(),
            sep: None,
            forced,
            colour,
            rate: 20e6,
            span_hz: 20e6,
            center_hz: 0.0,
            base: Vec::new(),
            priming: Vec::new(),
            backoff: 0,
            lock: None,
            quiet: 0,
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

impl pipeline::node::Node for VideoNode {
    fn name(&self) -> &str {
        "video"
    }

    fn num_inputs(&self) -> usize {
        1
    }

    /// The picture, and the sound that came with it.
    fn num_outputs(&self) -> usize {
        2
    }

    fn negotiate(&mut self, inputs: &[PortSpec]) -> Result<Vec<StreamSpec>> {
        let i = &inputs[0];
        if i.spec.kind != PortKind::Iq {
            return Err(common::Error::other("video reads complex baseband"));
        }
        // A PAL luma signal reaches 5 MHz and the colour subcarrier sits at
        // 4.43, so a stream that cannot carry 4.43 MHz of baseband cannot
        // hold a picture. This is the rate the node reads at, which is not
        // the span: `Video::chain` band-limits a wide span down to
        // `WORK_RATE_HZ` first, and `Shape::min_rate_hz` is what decides
        // whether a span is worth putting the front end on at all.
        if i.spec.rate < 2.0 * Standard::Pal.subcarrier_hz() {
            return Err(common::Error::other(
                "analogue video needs at least 8.9 MS/s to hold its baseband",
            ));
        }
        let span = i.spec.rate;
        self.span_hz = span;
        self.center_hz = i.spec.center.as_f64();
        // The sound first, because it is the one that needs the whole span.
        self.sound = dsp::video::Sound::new(span);
        self.wide_demod = dsp::FmDemod::new(span, DEVIATION_HZ);
        // Then the picture's own rate, and the filter that reaches it.
        let factor = decimation(span);
        self.rate = span / factor as f64;
        self.narrow = (factor > 1)
            .then(|| dsp::fir::FirDecim::design_hz(span, factor, self.rate * 0.4, 60.0));
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
        let mut voice = out.with_kind(PortKind::Voice);
        // The subcarrier's own rate after two decimations, not the graph's.
        voice.rate = self.sound.as_ref().map_or(crate::m17_nodes::VOICE_HZ, |s| s.rate());
        voice.channels = 1;
        Ok(vec![out, voice])
    }

    fn process(
        &mut self,
        inputs: &[&Payload],
        outputs: &mut [Payload],
        c: &mut NodeCtx<'_>,
    ) -> Result<()> {
        let Some(iq) = inputs[0].as_iq() else { return Ok(()) };
        let (o, sound_out) = outputs.split_at_mut(1);
        let o = &mut o[0];
        // Before the demodulation and not after it: this runs on the whole
        // span, so a span with no camera in it would otherwise FM demodulate
        // twenty megasamples a second to decide that again every block.
        if self.sep.is_none() && self.backoff > 0 {
            self.backoff = self.backoff.saturating_sub(iq.len());
            return Ok(());
        }
        self.base.clear();
        match self.narrow.as_mut() {
            Some(f) => {
                self.narrowed.clear();
                f.process(iq, &mut self.narrowed);
                self.demod.process(&self.narrowed, &mut self.base);
            }
            None => self.demod.process(iq, &mut self.base),
        }

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
            // The whole of what it is reading, which for composite video is
            // the whole span: an FM camera carrier at 5.8 GHz occupies the
            // best part of twenty megahertz, and every run a detector finds
            // inside it is a piece of the picture rather than a signal of
            // its own.
            c.request(Request::Claim {
                lo_hz: self.center_hz - self.span_hz / 2.0,
                hi_hz: self.center_hz + self.span_hz / 2.0,
            });
            // The samples that decided it are still video, so they are read
            // rather than thrown away.
            self.base.splice(0..0, priming);
        }
        let Some(sep) = self.sep.as_mut() else {
            return Ok(());
        };

        let mut fields = Vec::new();
        sep.process(&self.base, &mut fields);
        // A lock that has stopped producing is either a transmitter that has
        // gone or a lock that was never real. Both are answered the same
        // way: drop it, give the span back, and measure again.
        self.quiet = if fields.is_empty() { self.quiet + self.base.len() } else { 0 };
        if self.quiet as f64 > RELOCK_S * self.rate {
            self.quiet = 0;
            self.lock = None;
            self.priming.clear();
            self.backoff = self.rate as usize;
            if self.forced.is_none() {
                self.sep = None;
            } else if let Some(s) = self.sep.as_mut() {
                s.reset();
            }
            c.request(Request::Release);
            return Ok(());
        }
        let label = decode::video_channels::name_at(self.center_hz as u64, 3_000_000);

        // The sound, from the whole span rather than the picture's slice of
        // it: the subcarrier is at 6.5 MHz and the picture is read from a
        // baseband that reaches 5. Only once there is a picture, because a
        // second discriminator over 20 MS/s is a seventh of a core and a
        // camera nobody can see is not one whose sound anybody wants.
        if let Some(sound) = self.sound.as_mut() {
            self.wide.clear();
            self.wide_demod.process(iq, &mut self.wide);
            self.pcm.clear();
            sound.process(&self.wide, &mut self.pcm);
            if !self.pcm.is_empty() {
                if let Some(v) = sound_out.first_mut() {
                    v.voice_mut().push(common::Voice {
                        system: SYSTEM,
                        channel_hz: self.center_hz,
                        // No party, because there is none: this is the sound
                        // half of a transmission, not a call somebody placed
                        // to somebody. It is heard because the receiver is
                        // receiving it, and the picture is what says which
                        // channel it came from.
                        to: None,
                        from: None,
                        rate: sound.rate(),
                        pcm: std::mem::take(&mut self.pcm),
                    });
                }
            }
        }

        let out = o.video_mut();
        for f in fields {
            self.fields += 1;
            self.sequence += 1;
            let (pixels, samples) = match f.rgb {
                Some(rgb) => (Pixels::Rgb8, rgb),
                None => (Pixels::Luma8, f.luma),
            };
            out.push(VideoFrame {
                system: SYSTEM,
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
        self.wide_demod.reset();
        if let Some(s) = self.sound.as_mut() {
            s.reset();
        }
        self.priming.clear();
        self.backoff = 0;
        self.lock = None;
        self.quiet = 0;
        if let Some(s) = self.sep.as_mut() {
            s.reset();
        }
    }

    fn params(&self) -> Vec<Param> {
        vec![
            Param::bool(COLOUR, self.colour).label("Colour"),
            Param::choice(
                STANDARD,
                match self.forced {
                    None => 0,
                    Some(Standard::Pal) => 1,
                    Some(Standard::Ntsc) => 2,
                },
                [AUTO, Standard::Pal.label(), Standard::Ntsc.label()].map(String::from).to_vec(),
            )
            .label("Standard"),
        ]
    }

    fn set_param(&mut self, name: &str, v: ParamValue) -> Result<()> {
        match name {
            COLOUR => {
                self.colour = v.as_bool().unwrap_or(true);
                // The separator is built with colour on or off, so it has to
                // be rebuilt; the next block primes it again.
                self.sep = None;
            }
            STANDARD => {
                self.forced = match &v {
                    ParamValue::Choice(1) => Some(Standard::Pal),
                    ParamValue::Choice(2) => Some(Standard::Ntsc),
                    ParamValue::Text(t) => t.parse().ok(),
                    _ => None,
                };
                self.sep = None;
            }
            other => return Err(common::Error::other(format!("video has no {other}"))),
        }
        Ok(())
    }
}

/// Analogue video as the auto node knows it: on the span, where the
/// channel plan reaches, and owning the band only once it has a picture.
///
/// A camera's carrier is not a channel a detector can cut out: FM video at
/// 5.8 GHz occupies the best part of twenty megahertz, and what a detector
/// measures is the few megahertz around the carrier that stand above the
/// floor. Cut to that, the picture is gone. And claiming the span before
/// there is a picture would turn the band off for everything else on the
/// chance a camera turns up.
pub struct Video;

/// Half of what a channel of the plan occupies.
const CHANNEL_HALF_HZ: f64 = 9e6;

/// What the front end would rather read, in samples per second.
///
/// Enough for the whole FM signal (4.6 MHz measured on the AKK capture) and
/// for the 4.43 MHz colour subcarrier in the baseband that comes out of it,
/// and no more: the noise a discriminator sees is the bandwidth it is
/// handed. A line is then 640 samples, which is exactly the width a field is
/// resampled to.
const WORK_RATE_HZ: f64 = 10e6;

/// How much to divide a span by to reach [`WORK_RATE_HZ`] without going
/// under it. A 20 MS/s span stays whole, since halving it would leave 10.
fn decimation(rate: f64) -> usize {
    let mut f = 1usize;
    while rate / (f * 2) as f64 >= WORK_RATE_HZ {
        f *= 2;
    }
    f
}

impl Protocol for Video {
    fn id(&self) -> &'static str {
        "video"
    }
    fn label(&self) -> &'static str {
        "video"
    }
    fn placement(&self) -> Placement {
        Placement::Channels(
            decode::video_channels::channels().iter().map(|ch| ch.hz as f64).collect(),
        )
    }
    fn shape(&self) -> Shape {
        Shape {
            widths: &[2.0 * CHANNEL_HALF_HZ],
            // PAL luma reaches 5 MHz with the colour subcarrier at 4.43, so
            // a slower stream cannot be carrying a picture.
            min_rate_hz: 12e6,
            feed_rate_hz: WORK_RATE_HZ,
            span_wide: true,
            families: &[],
        }
    }
    fn stickiness(&self) -> Stickiness {
        Stickiness::Claim
    }
    /// Nothing at all until something transmits in the span. A camera's
    /// carrier is on for seconds at a time, so an empty band is an empty
    /// band, and demodulating 20 MS/s of it costs most of a core for a
    /// picture nobody is sending. Once the front end has locked it claims
    /// the span and the detector stops looking, which is why a claim keeps
    /// it awake by itself.
    fn wakes_on(&self) -> crate::protocol::Wake {
        crate::protocol::Wake::Detected { hold_s: 1.0 }
    }
    /// Not by the receiver: the front end needs the whole span for the
    /// sound and narrows the picture itself. See [`Video::chain`].
    fn narrow_span(&self) -> bool {
        false
    }
    fn outputs(&self) -> &'static [PortKind] {
        &[PortKind::Video, PortKind::Voice]
    }
    /// The whole span, and the node narrows what it needs to.
    ///
    /// The two halves of a camera's transmission want different bandwidths.
    /// The picture wants about 10 MS/s: it occupies under 5 MHz, the AKK
    /// capture measures 4.6, and a discriminator handed the whole 20 carries
    /// all of that bandwidth's noise into it. Off air 40 dB down the band
    /// that was the difference between no sync pulse at all and the line
    /// rate to within 0.1 us. The sound wants all of it: the subcarrier sits
    /// at 6.5 MHz of baseband, which a 10 MS/s stream cannot hold. A chain
    /// cannot fork, so the front end takes the span and does both.
    fn chain(&self, _at: Placed) -> Vec<NodeSpec> {
        vec![NodeSpec::new("video")]
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use common::{Hz, C32};
    use pipeline::node::Node;

    /// One block through the node, as the graph would run it: the fields it
    /// produced, the sound that came with them, and what it asked for.
    fn run(
        n: &mut VideoNode,
        iq: &[C32],
        rate: f64,
    ) -> (Vec<VideoFrame>, Vec<common::Voice>, Vec<Request>) {
        let ins = [spec(rate, 5_865e6)];
        let tags = Vec::new();
        let (mut fields, mut heard, mut asked) = (Vec::new(), Vec::new(), Vec::new());
        for chunk in iq.chunks(1 << 16) {
            let mut events = Vec::new();
            let mut new_tags = Vec::new();
            let mut ctx = NodeCtx::new(0, &ins, &tags, &mut events, &mut new_tags);
            let mut out = [Payload::empty_of(PortKind::Video), Payload::empty_of(PortKind::Voice)];
            let input = Payload::Iq(chunk.to_vec());
            Node::process(n, &[&input], &mut out, &mut ctx).expect("process");
            fields.extend(out[0].as_video().unwrap_or(&[]).iter().cloned());
            heard.extend(out[1].as_voice().unwrap_or(&[]).iter().cloned());
            asked.extend(events.into_iter().filter_map(|e| match e {
                pipeline::event::Event::Request(request) => Some(request),
                _ => None,
            }));
        }
        (fields, heard, asked)
    }

    fn spec(rate: f64, center: f64) -> PortSpec {
        PortSpec { spec: StreamSpec::iq(rate, Hz(center as u64)), latency: 0 }
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

    /// A lock is one measurement of one 40 ms window, and the claim it takes
    /// shuts the detector and every other span-wide decoder out of the band.
    /// So a lock that stops producing fields is given up: a camera that has
    /// left the air puts the span back, and a window of noise that scored
    /// costs a few seconds rather than the session.
    #[test]
    fn a_lock_that_stops_producing_gives_the_span_back() {
        let rate = 16e6;
        let mut n = VideoNode::new(None, false);
        Node::negotiate(&mut n, &[spec(rate, 5_865e6)]).expect("a span");
        let (fields, _, asked) = run(&mut n, &modulate(&pal(rate, 4), rate), rate);
        assert!(!fields.is_empty(), "no picture to lose");
        assert!(n.locked(), "not locked");
        assert!(
            asked.iter().any(|r| matches!(r, Request::Claim { .. })),
            "a picture claims the span"
        );

        // The transmitter goes. Silence at the same rate, longer than the
        // relock timeout, and the claim comes back.
        let quiet = vec![C32::new(0.0, 0.0); ((RELOCK_S + 1.0) * rate) as usize];
        let (_, _, asked) = run(&mut n, &quiet, rate);
        assert!(
            asked.iter().any(|r| matches!(r, Request::Release)),
            "the span was never given back"
        );
        assert!(!n.locked(), "still locked on nothing");
        assert_eq!(n.standard(), None, "the standard is measured again");
    }

    #[test]
    fn the_node_refuses_a_span_too_narrow_for_a_picture() {
        let mut n = VideoNode::default();
        assert!(Node::negotiate(&mut n, &[spec(20e6, 5_865e6)]).is_ok());
        assert!(Node::negotiate(&mut n, &[spec(2e6, 5_865e6)]).is_err());
    }

    /// The whole path: a transmitter's carrier in, fields out, on a port that
    /// carries the channel they came from.
    #[test]
    fn a_modulated_camera_comes_back_as_fields() {
        let rate = 16e6;
        let iq = modulate(&pal(rate, 4), rate);
        let mut n = VideoNode::new(None, false);
        let out = Node::negotiate(&mut n, &[spec(rate, 5_865e6)]).expect("a span");
        assert_eq!(out[0].kind, PortKind::Video);
        // The sound the camera sends beside the picture, on a port of its
        // own, so a channel a receiver is watching can be listened to.
        assert_eq!(out[1].kind, PortKind::Voice);

        let (fields, _, _) = run(&mut n, &iq, rate);
        assert!(!fields.is_empty(), "no field reached the port");
        assert_eq!(n.standard(), Some(Standard::Pal), "the standard was measured");

        let f = fields.iter().max_by_key(|f| f.lines_seen).expect("a field");
        assert_eq!(f.system, SYSTEM);
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

/// The setting names this stage reads.
const STANDARD: &str = "standard";
const COLOUR: &str = "colour";

/// What the standard setting is called when the node is to measure it
/// rather than be told.
const AUTO: &str = "auto";

pub const DESC: StageDesc = StageDesc {
    name: "video",
    summary: "Analogue video: FM to composite, sync separation, PAL or NTSC fields, colour",
    category: Category::Decode,
    feeds_bus: false,
};

pub fn build(s: &Settings) -> Result<Box<dyn pipeline::node::Node>> {
    let forced = s.str_or(STANDARD, AUTO).parse().ok();
    Ok(Box::new(VideoNode::new(forced, s.bool_or(COLOUR, true))))
}
