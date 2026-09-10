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
use pipeline::node::{Node, NodeCtx, PortSpec, Simple};
use pipeline::param::{Param, ParamValue};
use pipeline::port::{Payload, PortKind, StreamSpec, Tag, TagValue};
use pipeline::registry::{Category, Settings, SettingsExt, StageDesc};

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
            c.warn(format!("discarded {}", why.join("; ")));
        }
        Ok(())
    }

    fn reset(&mut self) {
        self.det.reset();
    }

    fn params(&self) -> Vec<Param> {
        vec![
            Param::float(RESET_US, self.cfg.reset_us as f64, 500.0..=100_000.0)
                .unit("us")
                .label("Gap that ends a packet")
                .log(),
            Param::float(MIN_MARK_US, self.cfg.min_mark_us as f64, 10.0..=2000.0)
                .unit("us")
                .label("Shortest credible mark"),
            Param::int(MIN_PULSES, self.cfg.min_pulses as i64, 2..=512)
                .label("Minimum pulses per packet"),
            Param::float(MIN_SNR_DB, self.cfg.min_snr_db as f64, 3.0..=40.0)
                .unit("dB")
                .label("Minimum SNR"),
            Param::float(HYSTERESIS, self.cfg.hysteresis as f64, 0.0..=0.5)
                .label("Threshold hysteresis"),
        ]
    }

    fn set_param(&mut self, name: &str, v: ParamValue) -> Result<()> {
        let f = v.as_f64().unwrap_or_default();
        match name {
            RESET_US => self.cfg.reset_us = f.max(1.0) as u32,
            MIN_MARK_US => self.cfg.min_mark_us = f.max(0.0) as u32,
            MIN_PULSES => self.cfg.min_pulses = f.max(1.0) as usize,
            MIN_SNR_DB => self.cfg.min_snr_db = f as f32,
            HYSTERESIS => self.cfg.hysteresis = f.clamp(0.0, 0.9) as f32,
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
            c.warn(format!("discarded {}", why.join("; ")));
        }
        Ok(())
    }

    fn reset(&mut self) {
        self.det.reset();
    }

    fn params(&self) -> Vec<Param> {
        vec![
            Param::float(RESET_US, self.cfg.reset_us as f64, 500.0..=100_000.0)
                .unit("us")
                .label("Gap that ends a packet")
                .log(),
            Param::float(MIN_RUN_US, self.cfg.min_run_us as f64, 10.0..=2000.0)
                .unit("us")
                .label("Shortest credible symbol"),
            Param::int(MIN_PULSES, self.cfg.min_pulses as i64, 2..=512)
                .label("Minimum pulses per packet"),
            Param::float(MIN_DEPTH_DB, self.cfg.min_depth_db as f64, 1.0..=40.0)
                .unit("dB")
                .label("Minimum modulation depth"),
            Param::float(MIN_SNR_DB, self.cfg.min_snr_db as f64, 3.0..=40.0)
                .unit("dB")
                .label("Minimum SNR"),
            Param::float(HYSTERESIS, self.cfg.hysteresis as f64, 0.0..=0.5)
                .label("Threshold hysteresis"),
        ]
    }

    fn set_param(&mut self, name: &str, v: ParamValue) -> Result<()> {
        let f = v.as_f64().unwrap_or_default();
        match name {
            RESET_US => self.cfg.reset_us = f.max(1.0) as u32,
            MIN_RUN_US => self.cfg.min_run_us = f.max(1.0) as u32,
            MIN_PULSES => self.cfg.min_pulses = f.max(1.0) as usize,
            MIN_DEPTH_DB => self.cfg.min_depth_db = f as f32,
            MIN_SNR_DB => self.cfg.min_snr_db = f as f32,
            HYSTERESIS => self.cfg.hysteresis = f.clamp(0.0, 0.9) as f32,
            _ => {
                return Err(common::Error::other(format!("ask_detect: unknown parameter {name:?}")))
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
            c.warn(format!("discarded {}", why.join("; ")));
        }
        Ok(())
    }

    fn reset(&mut self) {
        self.det.reset();
    }

    fn params(&self) -> Vec<Param> {
        vec![
            Param::float(RESET_US, self.cfg.reset_us as f64, 100.0..=100_000.0)
                .unit("us")
                .label("Silence that ends a burst")
                .log(),
            Param::float(MIN_RUN_US, self.cfg.min_run_us as f64, 2.0..=2000.0)
                .unit("us")
                .label("Shortest credible symbol"),
            Param::int(MIN_PULSES, self.cfg.min_pulses as i64, 2..=512)
                .label("Minimum pulses per packet"),
            Param::float(MIN_SEPARATION_HZ, self.cfg.min_separation_hz as f64, 200.0..=200_000.0)
                .unit("Hz")
                .label("Minimum tone separation")
                .log(),
            Param::float(MIN_SNR_DB, self.cfg.min_snr_db as f64, 3.0..=40.0)
                .unit("dB")
                .label("Minimum SNR"),
            Param::float(HYSTERESIS, self.cfg.hysteresis as f64, 0.0..=0.5)
                .label("Threshold hysteresis"),
        ]
    }

    fn set_param(&mut self, name: &str, v: ParamValue) -> Result<()> {
        let f = v.as_f64().unwrap_or_default();
        match name {
            RESET_US => self.cfg.reset_us = f.max(1.0) as u32,
            MIN_RUN_US => self.cfg.min_run_us = f.max(1.0) as u32,
            MIN_PULSES => self.cfg.min_pulses = f.max(1.0) as usize,
            MIN_SEPARATION_HZ => self.cfg.min_separation_hz = f.max(0.0) as f32,
            MIN_SNR_DB => self.cfg.min_snr_db = f as f32,
            HYSTERESIS => self.cfg.hysteresis = f.clamp(0.0, 0.9) as f32,
            _ => {
                return Err(common::Error::other(format!("fsk_detect: unknown parameter {name:?}")))
            }
        }
        let rate = self.det.rate();
        self.det = FskDetector::new(rate, self.cfg);
        Ok(())
    }
}

/// Turn one report into the event a consumer sees.
///
/// The conclusion only. How strongly the burst was heard and what it was read
/// from stay on the package and the packet the decode is attached to.
///
/// Shared with the packet bus decoder, which runs the same protocols over the
/// same packages at a different point in the graph. Two copies of this drifted
/// within a day of existing.
pub fn decoded_event(
    report: &decode::Report,
    pkg: &common::Package,
    center: common::Hz,
    modulation: common::Modulation,
) -> Decoded {
    let mut d = Decoded::bytes(report.model, center, pkg.start_sample as f64, report.raw.clone())
        .with_text(report.to_string())
        .with_detail(report.fields_line())
        .with_fields(report.fields.iter().map(|(k, v)| (k.clone(), v.clone())).collect())
        .with_modulation(modulation)
        .with_crc(report.crc_valid);
    if let Some(id) = &report.device {
        // The model is part of the space, not decoration. A sensor's id is a
        // handful of bits chosen at random, so two stations of different
        // makes sharing one is ordinary, and merging them would report a
        // single device reading two temperatures.
        d = d.by(common::Identity::new(format!("ism:{}", report.model), id.clone()));
    }
    d
}

/// What a burst no protocol claimed is named, everywhere it is asked about.
/// An open identifier rather than a set, since every other value is a
/// protocol id from the registry.
pub const UNKNOWN: &str = "unknown";

/// The event for a burst no protocol claimed, read under a guessed coding.
///
/// Worth emitting, and it is the whole reason a scanner is worth running
/// across a band: an unknown device is exactly what should be surfaced.
/// Silence would make the receiver useless for the case it should be best at,
/// and the inferred bits are where reverse engineering starts.
pub fn unmatched_event(
    pkg: &common::Package,
    center: common::Hz,
    modulation: common::Modulation,
    measure: Option<&common::Measure>,
) -> Decoded {
    let at = pkg.start_sample as f64;
    // What the burst was measured to be comes first, since it is what
    // there is to say about a burst nothing decoded: the coding guessed
    // from the timings follows, where there were timings.
    let measured = measure.map(|m| m.summary());
    let join = |a: Option<String>, b: String| match a {
        Some(a) => format!("{a}; {b}"),
        None => b,
    };
    let mut framing_fields: Vec<(String, common::Value)> = Vec::new();
    let ev = match decode::analyze(pkg) {
        // The bytes are the frame-aligned ones where the burst carried a
        // preamble to align to. Handing over the slicer's own phase instead is
        // what makes one device look like a different one on every reception.
        Some(a) => {
            if let Some(f) = &a.framing {
                framing_fields
                    .push(("preamble_bits".into(), common::Value::Int(f.preamble_bits as i64)));
                framing_fields.push(("sync".into(), common::Value::Text(f.sync_hex())));
                framing_fields
                    .push(("frame_bytes".into(), common::Value::Int(f.content_bytes() as i64)));
                if !f.repeats.is_empty() {
                    framing_fields
                        .push(("copies".into(), common::Value::Int(f.repeats.len() as i64 + 1)));
                }
            }
            if let Some(f) = &a.framed {
                framing_fields
                    .push(("frame_len".into(), common::Value::Int(f.payload.len() as i64)));
                framing_fields.push((
                    "whitening".into(),
                    common::Value::Text(if f.whitened { "PN9".into() } else { "none".into() }),
                ));
            }
            Decoded::bytes(UNKNOWN, center, at, a.frame_bytes().to_vec())
                .with_text(format!("unknown: {}", a.summary()))
                .with_detail(join(measured, a.summary()))
        }
        // Too short or too irregular to read. Still worth a line: it says
        // something was there, which is the difference between a quiet band
        // and a misconfigured chain.
        None if pkg.pulses.is_empty() && measure.is_some() => {
            Decoded::bytes(UNKNOWN, center, at, Vec::new())
                .with_text(format!("unknown: {}", measured.clone().unwrap_or_default()))
                .with_detail(measured.unwrap_or_default())
        }
        None => Decoded::bytes(UNKNOWN, center, at, Vec::new())
            .with_text("unknown: unreadable burst")
            .with_detail(join(
                measured,
                format!(
                    "{} pulses, {:.1} ms, no coding inferred",
                    pkg.pulses.len(),
                    pkg.duration_us() as f64 / 1000.0,
                ),
            )),
    };
    let mut ev = ev.with_modulation(modulation);
    let mut fields: Vec<(String, common::Value)> = Vec::new();
    if let Some(m) = measure {
        fields.push(("confidence".into(), common::Value::Float(m.confidence as f64)));
        if m.baud > 0.0 {
            fields.push(("baud".into(), common::Value::Float(m.baud as f64)));
        }
        if m.separation_hz > 0.0 {
            fields.push(("separation_hz".into(), common::Value::Float(m.separation_hz as f64)));
        }
        if m.sweep_hz_s.abs() > 0.0 {
            fields.push(("sweep_hz_per_s".into(), common::Value::Float(m.sweep_hz_s as f64)));
        }
        if m.symbol_period_us > 0.0 {
            fields
                .push(("symbol_period_us".into(), common::Value::Float(m.symbol_period_us as f64)));
        }
        if let Some(mode) = &m.mode {
            fields.push(("mode".into(), common::Value::Text(mode.clone())));
        }
    }
    fields.append(&mut framing_fields);
    if !fields.is_empty() {
        ev = ev.with_fields(fields);
    }
    ev
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
    modulation: common::Modulation,
}

impl ProtocolDecodeNode {
    pub fn new(protocols: Protocols) -> Self {
        Self {
            protocols,
            report_all: true,
            report_crc_failures: true,
            report_unknown: true,
            modulation: common::Modulation::Ook,
        }
    }

    /// Name the modulation feeding this decoder.
    pub fn with_modulation(mut self, m: common::Modulation) -> Self {
        self.modulation = m;
        self
    }

    /// Emit a burst no protocol claimed, read under a guessed coding.
    fn report_unmatched(&self, pkg: &common::Package, c: &mut NodeCtx<'_>) {
        if !self.report_unknown {
            return;
        }
        let center = c.inputs[0].spec.center;
        c.emit(Event::Decoded(unmatched_event(pkg, center, self.modulation, None)));
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
                        c.warn(format!(
                            "{name}: timings matched but CRC failed \
                             ({} pulses, {:.1} dB SNR)",
                            pkg.pulses.len(),
                            pkg.snr_db
                        ));
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
            Param::bool(REPORT_ALL, self.report_all).label("Report every matching protocol"),
            Param::bool(REPORT_CRC_FAILURES, self.report_crc_failures)
                .label("Warn on CRC failures"),
            Param::bool(REPORT_UNKNOWN, self.report_unknown).label("Report unrecognised bursts"),
        ]
    }

    fn set_param(&mut self, name: &str, v: ParamValue) -> Result<()> {
        match name {
            REPORT_ALL => self.report_all = v.as_bool().unwrap_or(true),
            REPORT_CRC_FAILURES => self.report_crc_failures = v.as_bool().unwrap_or(true),
            REPORT_UNKNOWN => self.report_unknown = v.as_bool().unwrap_or(true),
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
    /// When a transmission that never ends was last reported, in seconds of
    /// stream. See [`REPORT_S`].
    last_report_s: Option<f64>,
    /// Where every burst is written as it is cut, when `SR_DUMP_BURSTS` names
    /// a directory. Read once, at build: it is a diagnostic switch, not a
    /// setting, and looking it up per burst is a lookup per burst.
    dump_dir: Option<std::path::PathBuf>,
}

/// How often a transmission that never ends is reported, in seconds.
///
/// A base station carrier is on all day. The router cuts it into pieces of
/// half a second to have something to measure, and a packet per piece would
/// be a list of nothing else. One when it is found, then one every so often
/// to say it is still there, is what "which channels are busy" needs.
const REPORT_S: f64 = 5.0;

fn now_us() -> u64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_micros() as u64)
        .unwrap_or(0)
}

impl BurstRouteNode {
    pub fn new(cfg: dsp::RouterConfig) -> Self {
        Self {
            report_min_confidence: 0.5,
            cfg,
            router: dsp::BurstRouter::new(1.0, cfg),
            bursts: Vec::new(),
            last_report_s: None,
            dump_dir: std::env::var_os("SR_DUMP_BURSTS").map(std::path::PathBuf::from),
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

    /// Every burst the last block finished, with what it was measured to be
    /// and what the front end it went to made of it.
    pub fn routed(&self) -> &[dsp::RoutedBurst] {
        &self.bursts
    }
}

/// What a routed burst was measured to be, as evidence a packet carries.
pub fn measure_of(b: &dsp::RoutedBurst, centre_hz: f64) -> common::Measure {
    let f = &b.class.features;
    let mode = dsp::classify::mode::identify(b.class.modulation, f, centre_hz).map(|m| m.label());
    common::Measure {
        modulation: b.class.modulation,
        confidence: b.class.confidence,
        front_end: b.routed_to,
        mode,
        duration_us: f.duration_us as u32,
        bandwidth_hz: f.bandwidth_hz,
        baud: f.baud,
        separation_hz: f.separation_hz,
        sweep_hz_s: f.chirp_rate,
        symbol_period_us: if f.cyclic_period_s > 0.0 { f.cyclic_period_s * 1e6 } else { 0.0 },
    }
}

impl Node for BurstRouteNode {
    fn name(&self) -> &str {
        "burst_route"
    }

    fn num_outputs(&self) -> usize {
        2
    }

    fn negotiate(&mut self, inputs: &[PortSpec]) -> Result<Vec<StreamSpec>> {
        let i = &inputs[0];
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
        // What each burst was, as evidence: the packages read from it where
        // something read them, the measurement either way, and the samples it
        // was cut from. The pulses port is for the chain that reads them on;
        // this port is the burst itself, for whatever logs it.
        let mut packets = out.with_kind(PortKind::Packets);
        packets.rate = 0.0;
        Ok(vec![out, packets])
    }

    fn process(
        &mut self,
        inputs: &[&Payload],
        outputs: &mut [Payload],
        c: &mut NodeCtx<'_>,
    ) -> Result<()> {
        self.bursts.clear();
        self.router.process(inputs[0].as_iq().unwrap(), &mut self.bursts);

        let center = c.inputs[0].spec.center.0;
        let rate = c.inputs[0].spec.rate.max(1.0);
        let bandwidth_hz = c.inputs[0].spec.bandwidth as u32;
        let at_us = now_us();
        let (pulses, packets) = outputs.split_at_mut(1);
        let pkgs = pulses[0].pulses_mut();
        let out = packets[0].packets_mut();
        for b in &self.bursts {
            // A diagnostic: with `SR_DUMP_BURSTS` naming a directory, every
            // burst cut here is written there as interleaved f32 IQ, named
            // with the centre, the rate and the start sample, which is what
            // the classifier's `score_a_dumped_burst` test reads. How a
            // verdict on a real signal came out is otherwise invisible, and
            // that is how the TETRA carriers were found to be read as OFDM.
            // A file per burst, written where the burst was cut, which on a
            // busy band is inside whatever fanout is running this node.
            if let Some(dir) = &self.dump_dir {
                let path =
                    dir.join(format!("burst_{}_{}_{}.c64", center, rate as u64, b.start_sample));
                if !path.exists() {
                    let mut bytes = Vec::with_capacity(b.iq.len() * 8);
                    for s in &b.iq {
                        bytes.extend_from_slice(&s.re.to_le_bytes());
                        bytes.extend_from_slice(&s.im.to_le_bytes());
                    }
                    let _ = std::fs::write(path, bytes);
                }
            }
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
                c.tag(Tag::new(
                    b.start_sample,
                    "baud",
                    TagValue::Float(b.class.features.baud as f64),
                ));
            }
            // The measurement and the samples, built once for the burst and
            // only where a packet leaves carrying them.
            let evidence = || {
                (
                    measure_of(b, center as f64),
                    Some(std::sync::Arc::new(common::IqBurst {
                        rate,
                        center_hz: center,
                        samples: b.iq.clone(),
                    })),
                )
            };
            if !b.packages.is_empty() {
                let (m, iq) = evidence();
                for p in &b.packages {
                    c.tag(Tag::new(p.start_sample, "burst", TagValue::Float(p.snr_db as f64)));
                    let mut p = p.clone();
                    p.center_hz = center;
                    let mut pkt = common::Packet::of_pulses(at_us, bandwidth_hz, p.clone());
                    pkt.measure = Some(m.clone());
                    pkt.iq = iq.clone();
                    out.push(pkt);
                    pkgs.push(p);
                }
            }

            // A burst nothing here can demodulate is still a burst that
            // happened, and until now it left only a tag on a sample index
            // and a count in a warning: nothing a packet list could show. A
            // chirp swept at 30 MHz per second is a more useful log line than
            // silence, and it is the line somebody starts from when they go
            // looking for a decoder to write.
            if b.routed_to == common::FrontEnd::None
                && b.class.confidence >= self.report_min_confidence
                && b.class.modulation.is_named()
            {
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
                    fields
                        .push(("sweep_hz_per_s".into(), common::Value::Float(f.chirp_rate as f64)));
                }
                if f.cyclic_period_s > 0.0 {
                    fields.push((
                        "symbol_period_us".into(),
                        common::Value::Float(f.cyclic_period_s as f64 * 1e6),
                    ));
                }
                fields.push(("confidence".into(), common::Value::Float(b.class.confidence as f64)));

                // Name the mode where the parameters place one. This is the
                // only caller: the router needs a family to pick a front end
                // and nothing more, but a log wants "LoRa SF11 BW250".
                let mode = dsp::classify::mode::identify(
                    b.class.modulation,
                    &b.class.features,
                    center as f64,
                );
                let at = b.start_sample as f64 / c.inputs[0].spec.rate.max(1.0);
                let d = Decoded::bytes("unidentified", common::Hz(center), at, Vec::new())
                    .with_modulation(b.class.modulation)
                    .with_fields(fields);
                let d = match mode {
                    Some(m) => d.with_detail(m.label()),
                    None => {
                        d.with_detail(format!("no front end reads {}", b.class.modulation.label()))
                    }
                };
                c.emit(Event::Decoded(d));

                // And as a packet, so what is left of a burst nothing read is
                // a row with its measurement and its samples on it rather
                // than a line of text. A piece of a transmission that is
                // still going is the same news as the last piece, so those
                // are reported every [`REPORT_S`].
                let due = !b.continuous || self.last_report_s.is_none_or(|l| at - l >= REPORT_S);
                if b.packages.is_empty() && due {
                    if b.continuous {
                        self.last_report_s = Some(at);
                    }
                    let (m, iq) = evidence();
                    // The level is filled in by whatever holds the samples
                    // this was cut from; the classifier measures the burst
                    // against the noise it found and reports nothing when it
                    // never found any.
                    let mut pkt = common::Packet::of_pulses(
                        at_us,
                        bandwidth_hz,
                        common::Package {
                            pulses: Vec::new(),
                            snr_db: if b.class.features.snr_db > 0.0 {
                                b.class.features.snr_db
                            } else {
                                f32::NAN
                            },
                            rssi_dbfs: f32::NAN,
                            start_sample: b.start_sample,
                            center_hz: center,
                            modulation: None,
                        },
                    );
                    pkt.measure = Some(m);
                    pkt.iq = iq;
                    out.push(pkt);
                }
            }
        }

        let s = self.router.take_stats();
        if s.no_front_end > 0 {
            c.warn(format!(
                "{} burst(s) named as something no front end here reads",
                s.no_front_end
            ));
        }
        Ok(())
    }

    fn reset(&mut self) {
        self.router.reset();
    }

    fn params(&self) -> Vec<Param> {
        vec![
            Param::float(RESET_US, self.cfg.reset_us as f64, 500.0..=100_000.0)
                .unit("us")
                .label("Silence that ends a burst")
                .log(),
            Param::float(MARGIN_US, self.cfg.margin_us as f64, 100.0..=20_000.0)
                .unit("us")
                .label("Samples kept either side"),
            Param::float(MIN_SNR_DB, self.cfg.min_snr_db as f64, 3.0..=40.0)
                .unit("dB")
                .label("Minimum SNR"),
            Param::float(MIN_SCORE, self.cfg.classify.min_score as f64, 0.1..=0.9)
                .label("Score below which the burst is unnamed"),
            Param::float(MIN_MARGIN, self.cfg.classify.min_margin as f64, 0.0..=0.5)
                .label("Margin over the runner-up required"),
            // Past one on purpose: the top of the range means never, and a
            // busy wideband tier wants that available without a rebuild.
            Param::float(REPORT_CONFIDENCE, self.report_min_confidence as f64, 0.0..=1.01)
                .label("Confidence before an undecodable burst is logged"),
        ]
    }

    fn set_param(&mut self, name: &str, v: ParamValue) -> Result<()> {
        let f = v.as_f64().unwrap_or_default();
        match name {
            RESET_US => self.cfg.reset_us = f.max(1.0) as u32,
            MARGIN_US => self.cfg.margin_us = f.max(1.0) as u32,
            MIN_SNR_DB => self.cfg.min_snr_db = f as f32,
            MIN_SCORE => self.cfg.classify.min_score = f as f32,
            MIN_MARGIN => self.cfg.classify.min_margin = f as f32,
            // Reporting only: it does not touch the router, so it must not
            // rebuild it below either.
            REPORT_CONFIDENCE => {
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

/// The setting names these stages read.
///
/// The three burst detectors take most of the same ones, so a threshold is
/// spelled where the builder, the parameter list and the setter all see it.
const RESET_US: &str = "reset_us";
const MIN_PULSES: &str = "min_pulses";
const MIN_SNR_DB: &str = "min_snr_db";
const HYSTERESIS: &str = "hysteresis";
const NOISE_THRESHOLD_RATIO: &str = "noise_threshold_ratio";
const TAU_US: &str = "tau_us";
const MIN_MARK_US: &str = "min_mark_us";
const MIN_RUN_US: &str = "min_run_us";
const MERGE_DROPOUTS: &str = "merge_dropouts";
const MEASURED_NOISE_FLOOR: &str = "measured_noise_floor";
const NOISE_FLOOR_MARGIN: &str = "noise_floor_margin";
const MIN_DEPTH_DB: &str = "min_depth_db";
const MAX_BURST_US: &str = "max_burst_us";
const MIN_SEPARATION_HZ: &str = "min_separation_hz";
const MARGIN_US: &str = "margin_us";
const SOURCE_SNR_DB: &str = "source_snr_db";
const MIN_SCORE: &str = "min_score";
const MIN_MARGIN: &str = "min_margin";
const REPORT_CONFIDENCE: &str = "report_confidence";
const MODULATION: &str = "modulation";
const REPORT_ALL: &str = "report_all";
const REPORT_CRC_FAILURES: &str = "report_crc_failures";
const REPORT_UNKNOWN: &str = "report_unknown";

/// How sure the classifier has to be before a burst nothing read is logged.
const DEFAULT_REPORT_CONFIDENCE: f64 = 0.5;

pub const PULSE_DETECT: StageDesc = StageDesc {
    name: "pulse_detect",
    summary: "Envelope to mark/gap timings; the boundary between DSP \
              and protocol parsing",
    category: Category::Decode,
    feeds_bus: true,
};

pub fn build_pulse_detect(s: &Settings) -> Result<Box<dyn Node>> {
    let d = PulseConfig::default();
    Ok(Box::new(PulseDetectNode::new(PulseConfig {
        reset_us: s.f64_or(RESET_US, d.reset_us as f64) as u32,
        min_mark_us: s.f64_or(MIN_MARK_US, d.min_mark_us as f64) as u32,
        min_pulses: s.i64_or(MIN_PULSES, d.min_pulses as i64).max(1) as usize,
        min_snr_db: s.f64_or(MIN_SNR_DB, d.min_snr_db as f64) as f32,
        hysteresis: s.f64_or(HYSTERESIS, d.hysteresis as f64) as f32,
        noise_threshold_ratio: s.f64_or(NOISE_THRESHOLD_RATIO, d.noise_threshold_ratio as f64)
            as f32,
        tau_us: s.f64_or(TAU_US, d.tau_us as f64) as f32,
        merge_dropouts: s.bool_or(MERGE_DROPOUTS, d.merge_dropouts),
        measured_noise_floor: s.bool_or(MEASURED_NOISE_FLOOR, d.measured_noise_floor),
        noise_floor_margin: s.f64_or(NOISE_FLOOR_MARGIN, d.noise_floor_margin as f64) as f32,
    })))
}

pub const ASK_DETECT: StageDesc = StageDesc {
    name: "ask_detect",
    summary: "Amplitude keying with a low level that is not silence, \
              which `pulse_detect` latches through",
    category: Category::Decode,
    feeds_bus: true,
};

pub fn build_ask_detect(s: &Settings) -> Result<Box<dyn Node>> {
    let d = AskConfig::default();
    Ok(Box::new(AskDetectNode::new(AskConfig {
        reset_us: s.f64_or(RESET_US, d.reset_us as f64) as u32,
        min_run_us: s.f64_or(MIN_RUN_US, d.min_run_us as f64) as u32,
        min_pulses: s.i64_or(MIN_PULSES, d.min_pulses as i64).max(1) as usize,
        hysteresis: s.f64_or(HYSTERESIS, d.hysteresis as f64) as f32,
        tau_us: s.f64_or(TAU_US, d.tau_us as f64) as f32,
        min_snr_db: s.f64_or(MIN_SNR_DB, d.min_snr_db as f64) as f32,
        noise_threshold_ratio: s.f64_or(NOISE_THRESHOLD_RATIO, d.noise_threshold_ratio as f64)
            as f32,
        min_depth_db: s.f64_or(MIN_DEPTH_DB, d.min_depth_db as f64) as f32,
        max_burst_us: s.f64_or(MAX_BURST_US, d.max_burst_us as f64) as u32,
    })))
}

pub const FSK_DETECT: StageDesc = StageDesc {
    name: "fsk_detect",
    summary: "Two-level FSK to mark/gap timings, straight from IQ; the \
              constant-envelope signals an OOK detector cannot see",
    category: Category::Decode,
    feeds_bus: true,
};

pub fn build_fsk_detect(s: &Settings) -> Result<Box<dyn Node>> {
    let d = FskConfig::default();
    Ok(Box::new(FskDetectNode::new(FskConfig {
        reset_us: s.f64_or(RESET_US, d.reset_us as f64) as u32,
        min_run_us: s.f64_or(MIN_RUN_US, d.min_run_us as f64) as u32,
        min_pulses: s.i64_or(MIN_PULSES, d.min_pulses as i64).max(1) as usize,
        hysteresis: s.f64_or(HYSTERESIS, d.hysteresis as f64) as f32,
        tau_us: s.f64_or(TAU_US, d.tau_us as f64) as f32,
        min_snr_db: s.f64_or(MIN_SNR_DB, d.min_snr_db as f64) as f32,
        noise_threshold_ratio: s.f64_or(NOISE_THRESHOLD_RATIO, d.noise_threshold_ratio as f64)
            as f32,
        min_separation_hz: s.f64_or(MIN_SEPARATION_HZ, d.min_separation_hz as f64) as f32,
        max_burst_us: s.f64_or(MAX_BURST_US, d.max_burst_us as f64) as u32,
    })))
}

pub const BURST_ROUTE: StageDesc = StageDesc {
    name: "burst_route",
    summary: "Measure each burst, then run the one front end that reads it: \
              on-off, shallow ASK, two-level FSK or four-level",
    category: Category::Decode,
    feeds_bus: false,
};

pub fn build_burst_route(s: &Settings) -> Result<Box<dyn Node>> {
    let d = dsp::RouterConfig::default();
    let cfg = dsp::RouterConfig {
        reset_us: s.f64_or(RESET_US, d.reset_us as f64) as u32,
        margin_us: s.f64_or(MARGIN_US, d.margin_us as f64) as u32,
        min_snr_db: s.f64_or(MIN_SNR_DB, d.min_snr_db as f64) as f32,
        source_snr_db: s.f64_or(SOURCE_SNR_DB, 0.0) as f32,
        classify: dsp::ClassifyConfig {
            min_score: s.f64_or(MIN_SCORE, d.classify.min_score as f64) as f32,
            min_margin: s.f64_or(MIN_MARGIN, d.classify.min_margin as f64) as f32,
            ..d.classify
        },
        ..d
    };
    let mut n = BurstRouteNode::new(cfg);
    n.set_report_confidence(s.f64_or(REPORT_CONFIDENCE, DEFAULT_REPORT_CONFIDENCE) as f32);
    Ok(Box::new(n))
}

pub const PROTOCOL_DECODE: StageDesc = StageDesc {
    name: "protocol_decode",
    summary: "Try every known protocol against each burst",
    category: Category::Decode,
    feeds_bus: false,
};

pub fn build_protocol_decode(s: &Settings) -> Result<Box<dyn Node>> {
    let default = common::Modulation::Ook;
    let m = common::Modulation::parse(s.str_or(MODULATION, default.label())).unwrap_or(default);
    let mut n = ProtocolDecodeNode::all().with_modulation(m);
    for k in [REPORT_ALL, REPORT_CRC_FAILURES, REPORT_UNKNOWN] {
        if let Some(v) = s.get(k) {
            Node::set_param(&mut n, k, v.clone())?;
        }
    }
    Ok(Box::new(n))
}
