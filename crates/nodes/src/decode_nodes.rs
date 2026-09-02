//! Pulse extraction and protocol decoding as graph nodes.
//!
//! This is where the architecture pays off. `PulseDetectNode` is the boundary:
//! everything above it is per-sample DSP, everything below is integer parsing.
//! `ProtocolDecodeNode` sits below and is cheap enough to run every known
//! protocol against every burst.

use common::Result;
use decode::protocol::{DecodeError, Protocols};
use dsp::{AskConfig, AskDetector, FskConfig, FskDetector, OokDetector, PulseConfig};
use pipeline::event::{Decoded, Event};
use pipeline::node::{NodeCtx, PortSpec, Simple};
use pipeline::param::{Param, ParamValue};
use pipeline::port::{Payload, PortKind, StreamSpec, Tag, TagValue};

/// Envelope to pulse packages.
pub struct PulseDetectNode {
    cfg: PulseConfig,
    det: OokDetector,
}

impl PulseDetectNode {
    pub fn new(cfg: PulseConfig) -> Self {
        Self { cfg, det: OokDetector::new(1.0, cfg) }
    }

    pub fn default_ook() -> Self {
        Self::new(PulseConfig::default())
    }

    /// Shortest burst worth reporting, in pulses.
    pub fn set_min_pulses(&mut self, n: usize) -> &mut Self {
        self.cfg.min_pulses = n.max(1);
        let rate = self.det.rate();
        self.det = OokDetector::new(rate, self.cfg);
        self
    }
}

impl Simple for PulseDetectNode {
    fn name(&self) -> &str {
        "pulse_detect"
    }

    fn negotiate(&mut self, i: &PortSpec) -> Result<StreamSpec> {
        if i.spec.kind != PortKind::Real {
            return Err(common::Error::other(
                "pulse_detect needs a real envelope; put an `envelope` node before it",
            ));
        }
        self.det = OokDetector::new(i.spec.rate, self.cfg);
        // Packages are events in time, not a sampled stream, so a "rate" here
        // would be a fiction. Zero says so explicitly rather than inviting
        // something downstream to divide by it.
        let mut out = i.spec.with_kind(PortKind::Pulses);
        out.rate = 0.0;
        Ok(out)
    }

    fn process(&mut self, i: &Payload, o: &mut Payload, c: &mut NodeCtx<'_>) -> Result<()> {
        let pkgs = o.pulses_mut();
        self.det.process(i.as_real().unwrap(), pkgs);
        // Where the burst was received. The detector reads a stream and knows
        // nothing about frequency; the port it arrived on does, and in a
        // channel bank that is the channel's centre rather than the tuner's.
        let center = c.inputs[0].spec.center.0;
        for p in pkgs.iter_mut() {
            p.center_hz = center;
        }
        for p in pkgs.iter() {
            // Tag the burst so anything downstream, or a waterfall, can point
            // at exactly where in the stream it happened.
            c.tag(Tag::new(p.start_sample, "burst", TagValue::Float(p.snr_db as f64)));
        }

        // Report what was thrown away. Without this a mistuned chain produces
        // total silence, which looks identical to a disconnected antenna and
        // gives no hint which parameter is wrong.
        let s = self.det.take_stats();
        if s.rejected_total() > 0 {
            let mut why = Vec::new();
            if s.rejected_too_few_pulses > 0 {
                why.push(format!(
                    "{} with fewer than {} pulses (raise reset_us, or lower min_pulses)",
                    s.rejected_too_few_pulses, self.cfg.min_pulses
                ));
            }
            if s.rejected_low_snr > 0 {
                why.push(format!(
                    "{} below {:.0} dB SNR (lower min_snr_db, or increase gain)",
                    s.rejected_low_snr, self.cfg.min_snr_db
                ));
            }
            c.emit(Event::Warning {
                stage: "pulse_detect".into(),
                message: format!("discarded {}", why.join("; ")),
            });
        }
        Ok(())
    }

    fn reset(&mut self) {
        self.det.reset();
    }

    fn params(&self) -> Vec<Param> {
        vec![
            Param::float("reset_us", self.cfg.reset_us as f64, 500.0..=100_000.0)
                .unit("us")
                .label("Gap that ends a packet")
                .log(),
            Param::float("min_mark_us", self.cfg.min_mark_us as f64, 10.0..=2000.0)
                .unit("us")
                .label("Shortest credible mark"),
            Param::int("min_pulses", self.cfg.min_pulses as i64, 2..=512)
                .label("Minimum pulses per packet"),
            Param::float("min_snr_db", self.cfg.min_snr_db as f64, 3.0..=40.0)
                .unit("dB")
                .label("Minimum SNR"),
            Param::float("hysteresis", self.cfg.hysteresis as f64, 0.0..=0.5)
                .label("Threshold hysteresis"),
        ]
    }

    fn set_param(&mut self, name: &str, v: ParamValue) -> Result<()> {
        let f = v.as_f64().unwrap_or_default();
        match name {
            "reset_us" => self.cfg.reset_us = f.max(1.0) as u32,
            "min_mark_us" => self.cfg.min_mark_us = f.max(0.0) as u32,
            "min_pulses" => self.cfg.min_pulses = f.max(1.0) as usize,
            "min_snr_db" => self.cfg.min_snr_db = f as f32,
            "hysteresis" => self.cfg.hysteresis = f.clamp(0.0, 0.9) as f32,
            _ => {
                return Err(common::Error::other(format!(
                    "pulse_detect: unknown parameter {name:?}"
                )))
            }
        }
        // The detector caches derived values, so rebuild at the current rate.
        let rate = self.det.rate();
        self.det = OokDetector::new(rate, self.cfg);
        Ok(())
    }
}

/// Shallow ASK to pulse packages.
///
/// The fallback for when `pulse_detect` reports one enormous mark: below about
/// 11 dB of modulation depth its adaptive threshold latches high, because the
/// low symbol never goes under it. This one buffers the burst and thresholds
/// between the two levels it measures, at the cost of a burst of latency.
/// Takes the same envelope input, so it is a drop-in swap.
pub struct AskDetectNode {
    cfg: AskConfig,
    det: AskDetector,
}

impl AskDetectNode {
    pub fn new(cfg: AskConfig) -> Self {
        Self { cfg, det: AskDetector::new(1.0, cfg) }
    }

    pub fn default_ask() -> Self {
        Self::new(AskConfig::default())
    }
}

impl Simple for AskDetectNode {
    fn name(&self) -> &str {
        "ask_detect"
    }

    fn negotiate(&mut self, i: &PortSpec) -> Result<StreamSpec> {
        if i.spec.kind != PortKind::Real {
            return Err(common::Error::other(
                "ask_detect needs a real envelope; put an `envelope` node before it",
            ));
        }
        self.det = AskDetector::new(i.spec.rate, self.cfg);
        let mut out = i.spec.with_kind(PortKind::Pulses);
        out.rate = 0.0;
        Ok(out)
    }

    fn process(&mut self, i: &Payload, o: &mut Payload, c: &mut NodeCtx<'_>) -> Result<()> {
        let pkgs = o.pulses_mut();
        self.det.process(i.as_real().unwrap(), pkgs);
        // Where the burst was received; see `pulse_detect`.
        let center = c.inputs[0].spec.center.0;
        for p in pkgs.iter_mut() {
            p.center_hz = center;
        }
        let depth = self.det.depth_db() as f64;
        for p in pkgs.iter() {
            c.tag(Tag::new(p.start_sample, "burst", TagValue::Float(p.snr_db as f64)));
            c.tag(Tag::new(p.start_sample, "ask_depth_db", TagValue::Float(depth)));
        }

        let s = self.det.take_stats();
        if s.rejected_total() > 0 {
            let mut why = Vec::new();
            if s.rejected_no_separation > 0 {
                why.push(format!(
                    "{} shallower than {:.0} dB, so not keyed (lower min_depth_db)",
                    s.rejected_no_separation, self.cfg.min_depth_db
                ));
            }
            if s.rejected_too_few_pulses > 0 {
                why.push(format!(
                    "{} with fewer than {} pulses (raise reset_us, or lower min_pulses)",
                    s.rejected_too_few_pulses, self.cfg.min_pulses
                ));
            }
            if s.rejected_low_snr > 0 {
                why.push(format!(
                    "{} below {:.0} dB SNR (lower min_snr_db, or increase gain)",
                    s.rejected_low_snr, self.cfg.min_snr_db
                ));
            }
            c.emit(Event::Warning {
                stage: "ask_detect".into(),
                message: format!("discarded {}", why.join("; ")),
            });
        }
        Ok(())
    }

    fn reset(&mut self) {
        self.det.reset();
    }

    fn params(&self) -> Vec<Param> {
        vec![
            Param::float("reset_us", self.cfg.reset_us as f64, 500.0..=100_000.0)
                .unit("us")
                .label("Gap that ends a packet")
                .log(),
            Param::float("min_run_us", self.cfg.min_run_us as f64, 10.0..=2000.0)
                .unit("us")
                .label("Shortest credible symbol"),
            Param::int("min_pulses", self.cfg.min_pulses as i64, 2..=512)
                .label("Minimum pulses per packet"),
            Param::float("min_depth_db", self.cfg.min_depth_db as f64, 1.0..=40.0)
                .unit("dB")
                .label("Minimum modulation depth"),
            Param::float("min_snr_db", self.cfg.min_snr_db as f64, 3.0..=40.0)
                .unit("dB")
                .label("Minimum SNR"),
            Param::float("hysteresis", self.cfg.hysteresis as f64, 0.0..=0.5)
                .label("Threshold hysteresis"),
        ]
    }

    fn set_param(&mut self, name: &str, v: ParamValue) -> Result<()> {
        let f = v.as_f64().unwrap_or_default();
        match name {
            "reset_us" => self.cfg.reset_us = f.max(1.0) as u32,
            "min_run_us" => self.cfg.min_run_us = f.max(1.0) as u32,
            "min_pulses" => self.cfg.min_pulses = f.max(1.0) as usize,
            "min_depth_db" => self.cfg.min_depth_db = f as f32,
            "min_snr_db" => self.cfg.min_snr_db = f as f32,
            "hysteresis" => self.cfg.hysteresis = f.clamp(0.0, 0.9) as f32,
            _ => {
                return Err(common::Error::other(format!(
                    "ask_detect: unknown parameter {name:?}"
                )))
            }
        }
        let rate = self.det.rate();
        self.det = AskDetector::new(rate, self.cfg);
        Ok(())
    }
}

/// Two-level FSK to pulse packages.
///
/// Takes IQ rather than a real stream, unlike [`PulseDetectNode`], because it
/// needs the amplitude to know when a burst is happening and the phase to know
/// which tone is being sent. Putting an `envelope` or `fm_demod` node in front
/// would throw away exactly the half it still needs.
pub struct FskDetectNode {
    cfg: FskConfig,
    det: FskDetector,
}

impl FskDetectNode {
    pub fn new(cfg: FskConfig) -> Self {
        Self { cfg, det: FskDetector::new(1.0, cfg) }
    }

    pub fn default_fsk() -> Self {
        Self::new(FskConfig::default())
    }
}

impl Simple for FskDetectNode {
    fn name(&self) -> &str {
        "fsk_detect"
    }

    fn negotiate(&mut self, i: &PortSpec) -> Result<StreamSpec> {
        if i.spec.kind != PortKind::Iq {
            return Err(common::Error::other(
                "fsk_detect needs IQ; it does its own discrimination, so remove any \
                 `envelope` or `fm_demod` node before it",
            ));
        }
        self.det = FskDetector::new(i.spec.rate, self.cfg);
        let mut out = i.spec.with_kind(PortKind::Pulses);
        out.rate = 0.0;
        Ok(out)
    }

    fn process(&mut self, i: &Payload, o: &mut Payload, c: &mut NodeCtx<'_>) -> Result<()> {
        let pkgs = o.pulses_mut();
        self.det.process(i.as_iq().unwrap(), pkgs);
        // Where the burst was received; see `pulse_detect`.
        let center = c.inputs[0].spec.center.0;
        for p in pkgs.iter_mut() {
            p.center_hz = center;
        }
        let sep = self.det.separation_hz() as f64;
        for p in pkgs.iter() {
            c.tag(Tag::new(p.start_sample, "burst", TagValue::Float(p.snr_db as f64)));
            // The measured deviation names a device family before anything has
            // decoded, so it is worth carrying even when no protocol matches.
            c.tag(Tag::new(p.start_sample, "fsk_separation_hz", TagValue::Float(sep)));
        }

        let s = self.det.take_stats();
        if s.rejected_total() > 0 {
            let mut why = Vec::new();
            if s.rejected_no_separation > 0 {
                why.push(format!(
                    "{} with tones closer than {:.0} Hz, so not FSK (lower \
                     min_separation_hz, or widen the channel)",
                    s.rejected_no_separation, self.cfg.min_separation_hz
                ));
            }
            if s.rejected_too_few_pulses > 0 {
                why.push(format!(
                    "{} with fewer than {} pulses (raise reset_us, or lower min_pulses)",
                    s.rejected_too_few_pulses, self.cfg.min_pulses
                ));
            }
            if s.rejected_low_snr > 0 {
                why.push(format!(
                    "{} below {:.0} dB SNR (lower min_snr_db, or increase gain)",
                    s.rejected_low_snr, self.cfg.min_snr_db
                ));
            }
            c.emit(Event::Warning {
                stage: "fsk_detect".into(),
                message: format!("discarded {}", why.join("; ")),
            });
        }
        Ok(())
    }

    fn reset(&mut self) {
        self.det.reset();
    }

    fn params(&self) -> Vec<Param> {
        vec![
            Param::float("reset_us", self.cfg.reset_us as f64, 100.0..=100_000.0)
                .unit("us")
                .label("Silence that ends a burst")
                .log(),
            Param::float("min_run_us", self.cfg.min_run_us as f64, 2.0..=2000.0)
                .unit("us")
                .label("Shortest credible symbol"),
            Param::int("min_pulses", self.cfg.min_pulses as i64, 2..=512)
                .label("Minimum pulses per packet"),
            Param::float(
                "min_separation_hz",
                self.cfg.min_separation_hz as f64,
                200.0..=200_000.0,
            )
            .unit("Hz")
            .label("Minimum tone separation")
            .log(),
            Param::float("min_snr_db", self.cfg.min_snr_db as f64, 3.0..=40.0)
                .unit("dB")
                .label("Minimum SNR"),
            Param::float("hysteresis", self.cfg.hysteresis as f64, 0.0..=0.5)
                .label("Threshold hysteresis"),
        ]
    }

    fn set_param(&mut self, name: &str, v: ParamValue) -> Result<()> {
        let f = v.as_f64().unwrap_or_default();
        match name {
            "reset_us" => self.cfg.reset_us = f.max(1.0) as u32,
            "min_run_us" => self.cfg.min_run_us = f.max(1.0) as u32,
            "min_pulses" => self.cfg.min_pulses = f.max(1.0) as usize,
            "min_separation_hz" => self.cfg.min_separation_hz = f.max(0.0) as f32,
            "min_snr_db" => self.cfg.min_snr_db = f as f32,
            "hysteresis" => self.cfg.hysteresis = f.clamp(0.0, 0.9) as f32,
            _ => {
                return Err(common::Error::other(format!(
                    "fsk_detect: unknown parameter {name:?}"
                )))
            }
        }
        let rate = self.det.rate();
        self.det = FskDetector::new(rate, self.cfg);
        Ok(())
    }
}

/// Turn one report into the event a consumer sees.
///
/// Shared with the packet bus decoder, which runs the same protocols over the
/// same packages at a different point in the graph. Two copies of this drifted
/// within a day of existing.
pub fn decoded_event(
    report: &decode::Report,
    pkg: &common::Package,
    center: common::Hz,
    modulation: &'static str,
) -> Decoded {
    Decoded::bytes(report.model, center, pkg.start_sample as f64, report.raw.clone())
        .with_text(report.to_string())
        .with_detail(report.fields_line())
        .with_fields(report.fields.iter().map(|(k, v)| (k.clone(), v.clone())).collect())
        .with_modulation(modulation)
        .with_level(pkg.rssi_dbfs, pkg.snr_db)
        .with_crc(report.crc_valid)
}

/// The event for a burst no protocol claimed, read under a guessed coding.
///
/// Worth emitting, and it is the whole reason a scanner is worth running
/// across a band: an unknown device is exactly what should be surfaced.
/// Silence would make the receiver useless for the case it should be best at,
/// and the inferred bits are where reverse engineering starts.
pub fn unmatched_event(
    pkg: &common::Package,
    center: common::Hz,
    modulation: &'static str,
) -> Decoded {
    let at = pkg.start_sample as f64;
    let ev = match decode::analyze(pkg) {
        Some(a) => Decoded::bytes("unknown", center, at, a.bits.as_bytes().to_vec())
            .with_text(format!("unknown: {}", a.summary()))
            .with_detail(a.summary()),
        // Too short or too irregular to read. Still worth a line: it says
        // something was there, which is the difference between a quiet band
        // and a misconfigured chain.
        None => Decoded::bytes("unknown", center, at, Vec::new())
            .with_text("unknown: unreadable burst")
            .with_detail(format!(
                "{} pulses, {:.1} ms, no coding inferred",
                pkg.pulses.len(),
                pkg.duration_us() as f64 / 1000.0,
            )),
    };
    ev.with_modulation(modulation).with_level(pkg.rssi_dbfs, pkg.snr_db)
}

/// Run protocols against pulse packages and emit decodes as events.
pub struct ProtocolDecodeNode {
    protocols: Protocols,
    /// Report every protocol that claims a package, rather than only the first.
    report_all: bool,
    /// Emit a warning event when a package matched a protocol's timings but
    /// failed its CRC.
    report_crc_failures: bool,
    /// Report bursts no protocol claimed, with the coding inferred from their
    /// timings.
    ///
    /// On by default, and it is the whole reason this is worth running across
    /// a band: an unknown device is exactly what a scanner should surface.
    /// Silence would make the receiver useless for the case it should be best
    /// at, and the inferred bits are where reverse engineering starts.
    report_unknown: bool,
    /// How the pulses reaching this node were keyed, for the report.
    modulation: &'static str,
}

impl ProtocolDecodeNode {
    pub fn new(protocols: Protocols) -> Self {
        Self {
            protocols,
            report_all: true,
            report_crc_failures: true,
            report_unknown: true,
            modulation: "OOK",
        }
    }

    /// Name the modulation feeding this decoder: "OOK", "FSK", "ASK".
    pub fn with_modulation(mut self, m: &'static str) -> Self {
        self.modulation = m;
        self
    }

    /// Emit a burst no protocol claimed, read under a guessed coding.
    fn report_unmatched(&self, pkg: &common::Package, c: &mut NodeCtx<'_>) {
        if !self.report_unknown {
            return;
        }
        let center = c.inputs[0].spec.center;
        c.emit(Event::Decoded(unmatched_event(pkg, center, self.modulation)));
    }

    pub fn all() -> Self {
        Self::new(Protocols::all())
    }

    pub fn protocols(&self) -> &Protocols {
        &self.protocols
    }
}

impl Simple for ProtocolDecodeNode {
    fn name(&self) -> &str {
        "protocol_decode"
    }

    fn negotiate(&mut self, i: &PortSpec) -> Result<StreamSpec> {
        if i.spec.kind != PortKind::Pulses {
            return Err(common::Error::other(
                "protocol_decode needs pulses; put a `pulse_detect` node before it",
            ));
        }
        Ok(i.spec.with_kind(PortKind::Bytes))
    }

    fn process(&mut self, i: &Payload, o: &mut Payload, c: &mut NodeCtx<'_>) -> Result<()> {
        let out = o.bytes_mut();
        for pkg in i.as_pulses().unwrap() {
            let mut matched = false;
            for (name, res) in self.protocols.diagnose(pkg) {
                match res {
                    Ok(report) => {
                        matched = true;
                        out.extend_from_slice(&report.raw);
                        let center = c.inputs[0].spec.center;
                        c.emit(Event::Decoded(decoded_event(
                            &report,
                            pkg,
                            center,
                            self.modulation,
                        )));
                        if !self.report_all {
                            break;
                        }
                    }
                    Err(DecodeError::CrcFailed) if self.report_crc_failures => {
                        // Distinguishing "wrong protocol" from "right protocol,
                        // bad reception" is the difference between a silent
                        // tool and one that tells you to move the antenna.
                        c.emit(Event::Warning {
                            stage: name.to_string(),
                            message: format!(
                                "timings matched but CRC failed ({} pulses, {:.1} dB SNR)",
                                pkg.pulses.len(),
                                pkg.snr_db
                            ),
                        });
                    }
                    Err(_) => {}
                }
            }
            if !matched {
                self.report_unmatched(pkg, c);
            }
        }
        Ok(())
    }

    fn params(&self) -> Vec<Param> {
        vec![
            Param::bool("report_all", self.report_all).label("Report every matching protocol"),
            Param::bool("report_crc_failures", self.report_crc_failures)
                .label("Warn on CRC failures"),
            Param::bool("report_unknown", self.report_unknown)
                .label("Report unrecognised bursts"),
        ]
    }

    fn set_param(&mut self, name: &str, v: ParamValue) -> Result<()> {
        match name {
            "report_all" => self.report_all = v.as_bool().unwrap_or(true),
            "report_crc_failures" => self.report_crc_failures = v.as_bool().unwrap_or(true),
            "report_unknown" => self.report_unknown = v.as_bool().unwrap_or(true),
            _ => {
                return Err(common::Error::other(format!(
                    "protocol_decode: unknown parameter {name:?}"
                )))
            }
        }
        Ok(())
    }
}

/// Classify each burst, then run the one front end that can read it.
///
/// Replaces the pair of unconditional front ends the ISM graph used to run
/// over every channel. See [`dsp::route`] for why the order inverts and what a
/// refusal costs.
///
/// The routing is inside one node rather than spread across a branch per front
/// end because the decision is made from the burst, and a graph edge cannot
/// carry "this burst, to that node": the pipeline's ports are streams. What
/// the chain view loses in visible structure it gains in a stage that reports
/// what it decided, which is the `modulation` tag on every burst.
pub struct BurstRouteNode {
    /// Least confidence before a burst nothing demodulated is worth a log
    /// entry.
    ///
    /// The router acts on a much lower bar, and should: sending a doubtful
    /// burst to both front ends costs a little work and never loses a decode.
    /// Reporting is the opposite trade. An entry is a claim somebody reads,
    /// and a wrong one is worse than a missing one, so the bar for saying
    /// something out loud is higher than the bar for trying to demodulate.
    ///
    /// Half, measured against the off-air captures: of the bursts the
    /// classifier names there, the ones it gets right sit at a median
    /// confidence of 0.88 and the ones it gets wrong at 0.24. Half keeps 23 of
    /// the 25 correct and drops 43 of the 50 wrong, which is precision 0.33 to
    /// 0.77 for four percent of the recall.
    report_min_confidence: f32,
    cfg: dsp::RouterConfig,
    router: dsp::BurstRouter,
    bursts: Vec<dsp::RoutedBurst>,
}

impl BurstRouteNode {
    pub fn new(cfg: dsp::RouterConfig) -> Self {
        Self {
            report_min_confidence: 0.5,
            cfg,
            router: dsp::BurstRouter::new(1.0, cfg),
            bursts: Vec::new(),
        }
    }

    /// Least confidence before an undemodulated burst is reported.
    ///
    /// Confidence is a margin in 0 to 1, so anything above one silences the
    /// reporting entirely. That is deliberate and worth having: a wideband
    /// tier over a noisy band produces these constantly, and "log none of
    /// them" should be expressible without deleting the feature.
    pub fn set_report_confidence(&mut self, c: f32) -> &mut Self {
        self.report_min_confidence = c.max(0.0);
        self
    }

    pub fn default_ism() -> Self {
        Self::new(dsp::RouterConfig::default())
    }
}

impl Simple for BurstRouteNode {
    fn name(&self) -> &str {
        "burst_route"
    }

    fn negotiate(&mut self, i: &PortSpec) -> Result<StreamSpec> {
        if i.spec.kind != PortKind::Iq {
            return Err(common::Error::other(
                "burst_route needs IQ: it classifies the burst before deciding whether \
                 the envelope or the discriminator reads it, so it needs both",
            ));
        }
        self.cfg.classify.channel_hz = i.spec.rate as f32;
        self.router = dsp::BurstRouter::new(i.spec.rate, self.cfg);
        let mut out = i.spec.with_kind(PortKind::Pulses);
        out.rate = 0.0;
        Ok(out)
    }

    fn process(&mut self, i: &Payload, o: &mut Payload, c: &mut NodeCtx<'_>) -> Result<()> {
        self.bursts.clear();
        self.router.process(i.as_iq().unwrap(), &mut self.bursts);

        let center = c.inputs[0].spec.center.0;
        let pkgs = o.pulses_mut();
        for b in &self.bursts {
            // What it was measured to be, whether or not anything read it.
            // A burst nothing decodes is still evidence, and this is most of
            // what makes it useful.
            c.tag(Tag::new(
                b.start_sample,
                "modulation",
                TagValue::Text(b.class.modulation.label().into()),
            ));
            c.tag(Tag::new(
                b.start_sample,
                "modulation_confidence",
                TagValue::Float(b.class.confidence as f64),
            ));
            if b.class.features.bandwidth_hz > 0.0 {
                c.tag(Tag::new(
                    b.start_sample,
                    "bandwidth_hz",
                    TagValue::Float(b.class.features.bandwidth_hz as f64),
                ));
            }
            if b.class.features.baud > 0.0 {
                c.tag(Tag::new(b.start_sample, "baud", TagValue::Float(b.class.features.baud as f64)));
            }
            for p in &b.packages {
                c.tag(Tag::new(p.start_sample, "burst", TagValue::Float(p.snr_db as f64)));
                let mut p = p.clone();
                p.center_hz = center;
                pkgs.push(p);
            }

            // A burst nothing here can demodulate is still a burst that
            // happened, and until now it left only a tag on a sample index
            // and a count in a warning: nothing a packet list could show. A
            // chirp swept at 30 MHz per second is a more useful log line than
            // silence, and it is the line somebody starts from when they go
            // looking for a decoder to write.
            if b.routed_to == "none" && b.class.confidence >= self.report_min_confidence {
                let f = &b.class.features;
                let mut fields: Vec<(String, common::Value)> = Vec::new();
                if f.baud > 0.0 {
                    fields.push(("baud".into(), common::Value::Float(f.baud as f64)));
                }
                if f.separation_hz > 0.0 {
                    fields.push((
                        "separation_hz".into(),
                        common::Value::Float(f.separation_hz as f64),
                    ));
                }
                if f.chirp_rate.abs() > 0.0 {
                    fields.push((
                        "sweep_hz_per_s".into(),
                        common::Value::Float(f.chirp_rate as f64),
                    ));
                }
                if f.cyclic_period_s > 0.0 {
                    fields.push((
                        "symbol_period_us".into(),
                        common::Value::Float(f.cyclic_period_s as f64 * 1e6),
                    ));
                }
                fields.push((
                    "confidence".into(),
                    common::Value::Float(b.class.confidence as f64),
                ));

                // Name the mode where the parameters place one. This is the
                // only caller: the router needs a family to pick a front end
                // and nothing more, but a log wants "LoRa SF11 BW250".
                let mode = dsp::classify::mode::identify(
                    b.class.modulation,
                    &b.class.features,
                    center as f64,
                );
                let at = b.start_sample as f64 / c.inputs[0].spec.rate.max(1.0);
                let mut d = Decoded::bytes("unidentified", common::Hz(center), at, Vec::new())
                    .with_modulation(b.class.modulation.label())
                    .with_fields(fields);
                if f.bandwidth_hz > 0.0 {
                    d = d.with_bandwidth(f.bandwidth_hz as f64);
                }
                d = match mode {
                    Some(m) => d.with_detail(format!("{} ({})", m.name, m.note)),
                    None => d.with_detail(format!(
                        "no front end reads {}",
                        b.class.modulation.label()
                    )),
                };
                c.emit(Event::Decoded(d));
            }
        }

        let s = self.router.take_stats();
        if s.no_front_end > 0 {
            c.emit(Event::Warning {
                stage: "burst_route".into(),
                message: format!(
                    "{} burst(s) named as something no front end here reads",
                    s.no_front_end
                ),
            });
        }
        Ok(())
    }

    fn reset(&mut self) {
        self.router.reset();
    }

    fn params(&self) -> Vec<Param> {
        vec![
            Param::float("reset_us", self.cfg.reset_us as f64, 500.0..=100_000.0)
                .unit("us")
                .label("Silence that ends a burst")
                .log(),
            Param::float("margin_us", self.cfg.margin_us as f64, 100.0..=20_000.0)
                .unit("us")
                .label("Samples kept either side"),
            Param::float("min_snr_db", self.cfg.min_snr_db as f64, 3.0..=40.0)
                .unit("dB")
                .label("Minimum SNR"),
            Param::float("min_score", self.cfg.classify.min_score as f64, 0.1..=0.9)
                .label("Score below which the burst is unnamed"),
            Param::float("min_margin", self.cfg.classify.min_margin as f64, 0.0..=0.5)
                .label("Margin over the runner-up required"),
            // Past one on purpose: the top of the range means never, and a
            // busy wideband tier wants that available without a rebuild.
            Param::float("report_confidence", self.report_min_confidence as f64, 0.0..=1.01)
                .label("Confidence before an undecodable burst is logged"),
        ]
    }

    fn set_param(&mut self, name: &str, v: ParamValue) -> Result<()> {
        let f = v.as_f64().unwrap_or_default();
        match name {
            "reset_us" => self.cfg.reset_us = f.max(1.0) as u32,
            "margin_us" => self.cfg.margin_us = f.max(1.0) as u32,
            "min_snr_db" => self.cfg.min_snr_db = f as f32,
            "min_score" => self.cfg.classify.min_score = f as f32,
            "min_margin" => self.cfg.classify.min_margin = f as f32,
            // Reporting only: it does not touch the router, so it must not
            // rebuild it below either.
            "report_confidence" => {
                self.report_min_confidence = f.max(0.0) as f32;
                return Ok(());
            }
            _ => {
                return Err(common::Error::other(format!(
                    "burst_route: unknown parameter {name:?}"
                )))
            }
        }
        let rate = self.cfg.classify.channel_hz as f64;
        self.router = dsp::BurstRouter::new(rate.max(1.0), self.cfg);
        Ok(())
    }
}
