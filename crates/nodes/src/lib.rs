//! Graph nodes and the registry that builds them by name.
//!
//! The registry is what lets a chain be described as data. That matters for
//! ambiguous signals: when a burst does not decode, the answer is usually to
//! change the chain (a different decimation, a longer reset gap, FM
//! discrimination instead of an envelope), and that should be a
//! reconfiguration rather than a recompile.

pub mod ais_nodes;
pub mod aprs_nodes;
pub mod bank;
pub mod decode_nodes;
pub mod dsp_nodes;
pub mod modes_nodes;
pub mod feed_nodes;
pub mod packet_nodes;
pub mod pocsag_nodes;
pub mod bank_node;
mod filter_nodes;
pub mod sink_nodes;
pub mod wfm;

pub use bank::{ChannelBank, ChannelEvent, Gating};
pub use wfm::WfmDemodNode;
pub use decode_nodes::{
    AskDetectNode, BurstRouteNode, FskDetectNode, ProtocolDecodeNode, PulseDetectNode,
};
pub use ais_nodes::AisNode;
pub use aprs_nodes::AprsNode;
pub use pocsag_nodes::PocsagNode;
pub use modes_nodes::ModeSNode;
pub use feed_nodes::{feed_kind, FeedKind, FeedNode, FeedSpec, FEED_KINDS};
pub use packet_nodes::PacketDecodeNode;
pub use bank_node::BankNode;
pub use filter_nodes::{FirFilterNode, IirFilterNode};
pub use sink_nodes::{DcBlockNode, PacketBusNode, PacketSink, Ring, RingNode, SpectrumNode};
pub use dsp_nodes::{
    AgcNode, DecimateNode, DeemphasisNode, EnvelopeNode, FmDemodNode, HighBlendNode, MixerNode,
    RealDecimateNode, SquelchKind, SquelchNode, SsbDemodNode,
};

use common::Result;
use dsp::{AskConfig, FskConfig, PulseConfig};
use pipeline::node::Node;
use pipeline::registry::{Registry, Settings, SettingsExt, StageDesc};
use pipeline::{Graph, StreamSpec};

/// Every node type compiled into this build.
pub fn registry() -> Registry {
    let mut r = Registry::new();

    r.register(
        StageDesc {
            name: "mixer",
            summary: "Shift the signal in frequency, to bring an off-centre \
                      carrier to baseband and away from the DC spur",
            category: "filter",
        },
        |s: &Settings| Ok(Box::new(MixerNode::new(s.f64_or("shift_hz", 0.0))) as Box<dyn Node>),
    );

    r.register(
        StageDesc {
            name: "decimate",
            summary: "Lowpass and reduce the sample rate of an IQ stream",
            category: "filter",
        },
        |s: &Settings| {
            Ok(Box::new(DecimateNode::new(s.i64_or("factor", 1).max(1) as usize))
                as Box<dyn Node>)
        },
    );

    r.register(
        StageDesc {
            name: "real_decimate",
            summary: "Reduce the sample rate of a real stream, for audio",
            category: "filter",
        },
        |s: &Settings| {
            Ok(Box::new(RealDecimateNode::new(s.i64_or("factor", 1).max(1) as usize))
                as Box<dyn Node>)
        },
    );

    // The front ends the scanner table puts on a span. Registered like any
    // other stage so that the graph the receiver derives for itself is a
    // description rather than a special case, and so an operator can put one
    // anywhere rather than only where the table would have.
    r.register(
        StageDesc {
            name: "mode_s",
            summary: "1090 MHz ADS-B: preamble search and pulse-position bits",
            category: "decode",
        },
        |_s: &Settings| Ok(Box::new(ModeSNode::default()) as Box<dyn Node>),
    );

    r.register(
        StageDesc {
            name: "ais",
            summary: "Both marine AIS channels: GMSK demodulation and HDLC framing",
            category: "decode",
        },
        |_s: &Settings| Ok(Box::new(AisNode::default()) as Box<dyn Node>),
    );

    r.register(
        StageDesc {
            name: "aprs",
            summary: "One APRS channel: narrowband FM, Bell 202 AFSK, AX.25",
            category: "decode",
        },
        |s: &Settings| {
            Ok(Box::new(AprsNode::new(s.f64_or("channel_hz", 144_800_000.0))) as Box<dyn Node>)
        },
    );

    r.register(
        StageDesc {
            name: "pocsag",
            summary: "One pager channel: narrowband FM and POCSAG at 512 to 2400 baud",
            category: "decode",
        },
        |s: &Settings| {
            Ok(Box::new(PocsagNode::new(s.f64_or("channel_hz", 153_350_000.0))) as Box<dyn Node>)
        },
    );

    r.register(
        StageDesc {
            name: "bank",
            summary: "Channelize a band and run a burst front end in every \
                      channel of it at once",
            category: "decode",
        },
        |s: &Settings| {
            let width = s.f64_or("channel_hz", 31_250.0).max(1.0);
            // Every tier runs the same graph: what a channel holds is
            // measured and then routed, rather than assumed from the width
            // the tier was built at.
            let label = if width >= 1e6 {
                format!("{:.1} MHz bank", width / 1e6)
            } else {
                format!("{:.0} kHz bank", width / 1e3)
            };
            let mut n = BankNode::new(label, width, ism_decode_graph);
            let (lo, hi) = (s.f64_or("band_lo_hz", 0.0), s.f64_or("band_hi_hz", 0.0));
            if hi > lo {
                n.set_band(Some((lo, hi)));
            }
            Ok(Box::new(n) as Box<dyn Node>)
        },
    );

    // Where everything that produces packets meets, and what hangs off the
    // far side of it.
    r.register(
        StageDesc {
            name: "packet_bus",
            summary: "Gather bursts, frames and packets from every front end \
                      into one stream, and write them to the log",
            category: "sink",
        },
        |s: &Settings| {
            Ok(Box::new(PacketBusNode::new(s.i64_or("inputs", 1).max(1) as usize))
                as Box<dyn Node>)
        },
    );

    r.register(
        StageDesc {
            name: "protocols",
            summary: "Run every known protocol over everything on the bus, once",
            category: "decode",
        },
        |_s: &Settings| Ok(Box::new(PacketDecodeNode::default()) as Box<dyn Node>),
    );

    r.register(
        StageDesc {
            name: "feed",
            summary: "Packets from another receiver, over the network",
            category: "decode",
        },
        |s: &Settings| {
            let kind = feed_kind(s.str_or("format", FEED_KINDS[0].name))
                .ok_or_else(|| common::Error::other("no feed format of that name"))?;
            let spec = FeedSpec::new(
                s.str_or("host", "127.0.0.1"),
                s.i64_or("port", kind.default_port as i64) as u16,
                kind,
            );
            Ok(Box::new(FeedNode::new(spec)) as Box<dyn Node>)
        },
    );

    r.register(
        StageDesc {
            name: "dc_block",
            summary: "Remove the centre spur a direct-conversion receiver \
                      produces, by measuring it rather than notching it out",
            category: "filter",
        },
        |_s: &Settings| Ok(Box::new(DcBlockNode::new()) as Box<dyn Node>),
    );

    r.register(
        StageDesc {
            name: "spectrum",
            summary: "An FFT display of whatever is wired into it, drawn \
                      alongside the receiver's own",
            category: "sink",
        },
        |s: &Settings| {
            let size = s.i64_or("size", 1024).clamp(64, 32_768) as usize;
            // A power of two, because that is what the transform takes.
            let size = 1usize << (usize::BITS - 1 - size.leading_zeros()) as usize;
            Ok(Box::new(SpectrumNode::new(size)) as Box<dyn Node>)
        },
    );

    r.register(
        StageDesc {
            name: "fir_filter",
            summary: "Windowed-sinc pass or block filter: linear phase, and \
                      as sharp as the taps you pay for",
            category: "filter",
        },
        |s: &Settings| {
            let r = dsp::filter::Response::from_name(s.str_or("response", "lowpass"))
                .unwrap_or(dsp::filter::Response::Lowpass);
            Ok(Box::new(FirFilterNode::new(
                r,
                s.f64_or("freq_hz", 5_000.0),
                s.f64_or("width_hz", 2_000.0),
                s.i64_or("taps", 127).clamp(3, 4095) as usize,
            )) as Box<dyn Node>)
        },
    );

    r.register(
        StageDesc {
            name: "iir_filter",
            summary: "Biquad pass or block filter: a handful of coefficients \
                      where an FIR would need hundreds, at the cost of phase",
            category: "filter",
        },
        |s: &Settings| {
            let r = dsp::filter::Response::from_name(s.str_or("response", "lowpass"))
                .unwrap_or(dsp::filter::Response::Lowpass);
            let freq = s.f64_or("freq_hz", 5_000.0);
            // A band is described by its width everywhere except in the
            // arithmetic, which wants a resonance.
            let q = match s.get("width_hz") {
                Some(w) => dsp::filter::Biquad::band_q(freq, w.as_f64().unwrap_or(1.0)),
                None => s.f64_or("q", 0.707),
            };
            Ok(Box::new(IirFilterNode::new(r, freq, q)) as Box<dyn Node>)
        },
    );

    r.register(
        StageDesc {
            name: "envelope",
            summary: "Complex magnitude; the input an OOK pulse detector needs",
            category: "demod",
        },
        |_s: &Settings| Ok(Box::new(EnvelopeNode) as Box<dyn Node>),
    );

    r.register(
        StageDesc {
            name: "fm_demod",
            summary: "Quadrature frequency discriminator, for FM and FSK",
            category: "demod",
        },
        |s: &Settings| {
            Ok(Box::new(FmDemodNode::new(s.f64_or("deviation_hz", 75_000.0)))
                as Box<dyn Node>)
        },
    );

    r.register(
        StageDesc {
            name: "deemphasis",
            summary: "Undo broadcast FM pre-emphasis (50 us in Europe, 75 in the Americas)",
            category: "filter",
        },
        |s: &Settings| {
            Ok(Box::new(DeemphasisNode::new(s.f64_or("tau_us", 50.0))) as Box<dyn Node>)
        },
    );

    r.register(
        StageDesc {
            name: "pulse_detect",
            summary: "Envelope to mark/gap timings; the boundary between DSP \
                      and protocol parsing",
            category: "decode",
        },
        |s: &Settings| {
            let d = PulseConfig::default();
            let cfg = PulseConfig {
                reset_us: s.f64_or("reset_us", d.reset_us as f64) as u32,
                min_mark_us: s.f64_or("min_mark_us", d.min_mark_us as f64) as u32,
                min_pulses: s.i64_or("min_pulses", d.min_pulses as i64).max(1) as usize,
                min_snr_db: s.f64_or("min_snr_db", d.min_snr_db as f64) as f32,
                hysteresis: s.f64_or("hysteresis", d.hysteresis as f64) as f32,
                noise_threshold_ratio: s.f64_or("noise_threshold_ratio", d.noise_threshold_ratio as f64) as f32,
                tau_us: s.f64_or("tau_us", d.tau_us as f64) as f32,
                merge_dropouts: s.bool_or("merge_dropouts", d.merge_dropouts),
                measured_noise_floor: s
                    .bool_or("measured_noise_floor", d.measured_noise_floor),
                noise_floor_margin: s.f64_or("noise_floor_margin", d.noise_floor_margin as f64)
                    as f32,
            };
            Ok(Box::new(PulseDetectNode::new(cfg)) as Box<dyn Node>)
        },
    );

    r.register(
        StageDesc {
            name: "ask_detect",
            summary: "Amplitude keying with a low level that is not silence, \
                      which `pulse_detect` latches through",
            category: "decode",
        },
        |s: &Settings| {
            let d = AskConfig::default();
            let cfg = AskConfig {
                reset_us: s.f64_or("reset_us", d.reset_us as f64) as u32,
                min_run_us: s.f64_or("min_run_us", d.min_run_us as f64) as u32,
                min_pulses: s.i64_or("min_pulses", d.min_pulses as i64).max(1) as usize,
                hysteresis: s.f64_or("hysteresis", d.hysteresis as f64) as f32,
                tau_us: s.f64_or("tau_us", d.tau_us as f64) as f32,
                min_snr_db: s.f64_or("min_snr_db", d.min_snr_db as f64) as f32,
                noise_threshold_ratio: s
                    .f64_or("noise_threshold_ratio", d.noise_threshold_ratio as f64)
                    as f32,
                min_depth_db: s.f64_or("min_depth_db", d.min_depth_db as f64) as f32,
                max_burst_us: s.f64_or("max_burst_us", d.max_burst_us as f64) as u32,
            };
            Ok(Box::new(AskDetectNode::new(cfg)) as Box<dyn Node>)
        },
    );

    r.register(
        StageDesc {
            name: "fsk_detect",
            summary: "Two-level FSK to mark/gap timings, straight from IQ; the \
                      constant-envelope signals an OOK detector cannot see",
            category: "decode",
        },
        |s: &Settings| {
            let d = FskConfig::default();
            let cfg = FskConfig {
                reset_us: s.f64_or("reset_us", d.reset_us as f64) as u32,
                min_run_us: s.f64_or("min_run_us", d.min_run_us as f64) as u32,
                min_pulses: s.i64_or("min_pulses", d.min_pulses as i64).max(1) as usize,
                hysteresis: s.f64_or("hysteresis", d.hysteresis as f64) as f32,
                tau_us: s.f64_or("tau_us", d.tau_us as f64) as f32,
                min_snr_db: s.f64_or("min_snr_db", d.min_snr_db as f64) as f32,
                noise_threshold_ratio: s
                    .f64_or("noise_threshold_ratio", d.noise_threshold_ratio as f64)
                    as f32,
                min_separation_hz: s.f64_or("min_separation_hz", d.min_separation_hz as f64)
                    as f32,
                max_burst_us: s.f64_or("max_burst_us", d.max_burst_us as f64) as u32,
            };
            Ok(Box::new(FskDetectNode::new(cfg)) as Box<dyn Node>)
        },
    );

    r.register(
        StageDesc {
            name: "burst_route",
            summary: "Measure each burst, then run the one front end that reads it: \
                      on-off, shallow ASK, two-level FSK or four-level",
            category: "decode",
        },
        |s: &Settings| {
            let d = dsp::RouterConfig::default();
            let cfg = dsp::RouterConfig {
                reset_us: s.f64_or("reset_us", d.reset_us as f64) as u32,
                margin_us: s.f64_or("margin_us", d.margin_us as f64) as u32,
                min_snr_db: s.f64_or("min_snr_db", d.min_snr_db as f64) as f32,
                classify: dsp::ClassifyConfig {
                    min_score: s.f64_or("min_score", d.classify.min_score as f64) as f32,
                    min_margin: s.f64_or("min_margin", d.classify.min_margin as f64) as f32,
                    ..d.classify
                },
                ..d
            };
            Ok(Box::new(BurstRouteNode::new(cfg)) as Box<dyn Node>)
        },
    );

    r.register(
        StageDesc {
            name: "ssb_demod",
            summary: "Demodulate one sideband, or a narrow slice of it for CW",
            category: "demod",
        },
        |s: &Settings| {
            let sideband = match s.str_or("sideband", "usb") {
                "lsb" | "LSB" => dsp::ssb::Sideband::Lower,
                _ => dsp::ssb::Sideband::Upper,
            };
            // A CW filter is the same stage with its passband put around the
            // pitch the operator hears rather than around speech.
            if s.get("pitch_hz").is_some() {
                return Ok(Box::new(SsbDemodNode::cw(
                    sideband,
                    s.f64_or("pitch_hz", 700.0),
                    s.f64_or("width_hz", 500.0),
                )) as Box<dyn Node>);
            }
            Ok(Box::new(SsbDemodNode::new(
                sideband,
                s.f64_or("low_hz", 300.0),
                s.f64_or("high_hz", 2_700.0),
            )) as Box<dyn Node>)
        },
    );

    r.register(
        StageDesc {
            name: "wfm_demod",
            summary: "Broadcast FM with stereo and RDS, from a wide IF",
            category: "demod",
        },
        |_s: &Settings| Ok(Box::new(WfmDemodNode::new()) as Box<dyn Node>),
    );

    r.register(
        StageDesc {
            name: "high_blend",
            summary: "Roll the top off audio in proportion to the noise on it, \
                      so a weak channel hisses less",
            category: "audio",
        },
        |_s: &Settings| Ok(Box::new(HighBlendNode::new()) as Box<dyn Node>),
    );

    r.register(
        StageDesc {
            name: "agc",
            summary: "Hold audio at a usable level without riding the volume control",
            category: "audio",
        },
        |s: &Settings| {
            // The presets a mode asks for, so a derived chain does not have
            // to restate three time constants to say "the one for speech".
            let mut n = match s.str_or("preset", "") {
                "voice" => AgcNode::voice(),
                "cw" => AgcNode::cw(),
                _ => AgcNode::new(
                    s.f64_or("attack_ms", 5.0),
                    s.f64_or("release_ms", 500.0),
                    s.f64_or("hang_ms", 300.0),
                ),
            };
            if let Some(v) = s.get("max_gain_db") {
                pipeline::node::Node::set_param(&mut n, "max_gain_db", v.clone())?;
            }
            Ok(Box::new(n) as Box<dyn Node>)
        },
    );

    r.register(
        StageDesc {
            name: "squelch",
            summary: "Mute a channel with nothing on it, by noise for FM or by level",
            category: "audio",
        },
        |s: &Settings| {
            let kind = match s.str_or("kind", "noise") {
                "level" => SquelchKind::Level,
                _ => SquelchKind::Noise,
            };
            Ok(Box::new(SquelchNode::new(kind, s.f64_or("threshold_db", 9.0) as f32))
                as Box<dyn Node>)
        },
    );

    r.register(
        StageDesc {
            name: "protocol_decode",
            summary: "Try every known protocol against each burst",
            category: "decode",
        },
        |s: &Settings| {
            // Static because the modulation rides on every packet this node
            // emits, and a list column should not own a string per row.
            let m = match s.str_or("modulation", "OOK") {
                "FSK" | "fsk" => "FSK",
                "ASK" | "ask" => "ASK",
                _ => "OOK",
            };
            let mut n = ProtocolDecodeNode::all().with_modulation(m);
            for k in ["report_all", "report_crc_failures", "report_unknown"] {
                if let Some(v) = s.get(k) {
                    pipeline::node::Node::set_param(&mut n, k, v.clone())?;
                }
            }
            Ok(Box::new(n) as Box<dyn Node>)
        },
    );

    r
}

/// One node in a chain description.
#[derive(Clone, Debug, Default)]
pub struct NodeSpec {
    pub kind: String,
    pub settings: Settings,
}

impl NodeSpec {
    pub fn new(kind: &str) -> Self {
        Self { kind: kind.into(), settings: Settings::new() }
    }

    pub fn set(mut self, k: &str, v: pipeline::ParamValue) -> Self {
        self.settings.insert(k.into(), v);
        self
    }

    pub fn f(self, k: &str, v: f64) -> Self {
        self.set(k, pipeline::ParamValue::Float(v))
    }

    pub fn i(self, k: &str, v: i64) -> Self {
        self.set(k, pipeline::ParamValue::Int(v))
    }

    pub fn b(self, k: &str, v: bool) -> Self {
        self.set(k, pipeline::ParamValue::Bool(v))
    }
}

/// Build a linear graph from a chain description.
///
/// Errors name the offending node and its index, because a chain assembled
/// from a config file is exactly the situation where "type mismatch" without a
/// position is useless.
pub fn build_chain(input: StreamSpec, specs: &[NodeSpec], reg: &Registry) -> Result<Graph> {
    let mut nodes: Vec<Box<dyn Node>> = Vec::with_capacity(specs.len());
    for (i, s) in specs.iter().enumerate() {
        if !reg.contains(&s.kind) {
            let known: Vec<&str> = reg.list().map(|d| d.name).collect();
            return Err(common::Error::other(format!(
                "chain node {i}: no node type named {:?}. Known types: {}",
                s.kind,
                known.join(", ")
            )));
        }
        nodes.push(
            reg.build(&s.kind, &s.settings)
                .map_err(|e| common::Error::other(format!("chain node {i} ({}): {e}", s.kind)))?,
        );
    }
    pipeline::chain(input, nodes)
}

/// A ready-made OOK chain for ISM decoding: shift, decimate, envelope, detect,
/// decode.
pub fn ook_chain(shift_hz: f64, decimate: usize, reset_us: u32) -> Vec<NodeSpec> {
    vec![
        NodeSpec::new("mixer").f("shift_hz", shift_hz),
        NodeSpec::new("decimate").i("factor", decimate as i64),
        NodeSpec::new("envelope"),
        NodeSpec::new("pulse_detect").f("reset_us", reset_us as f64).i("min_pulses", 20),
        NodeSpec::new("protocol_decode"),
    ]
}

/// Burst detection settings for gating ISM decode chains.
///
/// The stock 10 dB open threshold is right for finding transmissions to look
/// at and wrong for gating a decoder, because it is stricter than the decoder
/// it is gating. Measured on the Fine Offset capture spread across four
/// channels, the channel-power detector sees 12 dB while the pulse detector
/// reads 19 dB on the same burst and decodes it perfectly: at 10 dB the gate
/// throws away packets that would otherwise have been decoded, which is the
/// one thing a gate must never do. At 6 dB all four decode.
///
/// The floor is what a gate is worth: idle channels cost the detector only,
/// and that is most of the band most of the time.
pub fn ism_detector_config() -> dsp::DetectorConfig {
    dsp::DetectorConfig { open_db: 6.0, close_db: 3.0, ..Default::default() }
}

/// Everything an ISM channel needs, in one graph.
///
/// ```text
///   IQ ---> burst_route ---> packages
/// ```
///
/// One gate and one classifier, and then whichever of the on-off, shallow
/// ASK, two-level FSK or four-level front ends the burst was measured to
/// need. See [`dsp::route`] for why, and for what happens to a burst the
/// classifier will not name: it goes to the on-off and two-level front ends
/// both, which is what this graph used to do with every burst unconditionally.
///
/// The same graph runs in every bank tier. It used to come in an OOK flavour
/// and an FSK one, chosen by the channel width, because the width was the only
/// evidence available about what a channel would hear. It is not evidence: a
/// 125 kHz channel carries on-off keyed sensors all day. What the width really
/// decides is how much noise comes with the signal, and the classifier reads
/// that from the channel it is given.
pub fn ism_decode_graph(input: StreamSpec) -> Result<Graph> {
    let mut b = Graph::builder(input);
    let node = b.add_labeled("Classify and route", Box::new(BurstRouteNode::default_ism()));
    b.source(node.i());
    b.output(node.o());
    b.build()
}

/// The 1090 MHz chain: one node over the wideband stream.
///
/// A graph of one looks odd next to the ISM chains, and it is still worth
/// being a graph: it is how the chain view, the parameter surface and the
/// latency accounting reach a decoder, and none of those should need to know
/// which decoder they are looking at.
pub fn adsb_graph(input: StreamSpec) -> Result<Graph> {
    let mut b = Graph::builder(input);
    let n = b.add_labeled("1090 Mode S", Box::new(ModeSNode::default()));
    b.source(n.i());
    b.output(n.o());
    b.build()
}

/// A ready-made FSK chain: shift, decimate, detect, decode.
///
/// Shorter than the OOK chain by one node, because the detector takes IQ and
/// does its own discrimination. `deviation_hz` is the protocol's published
/// deviation; the separation between the tones is twice that, and the check is
/// set at half of it so a mistuned or drifting transmitter still passes.
pub fn fsk_chain(shift_hz: f64, decimate: usize, deviation_hz: f64, reset_us: u32) -> Vec<NodeSpec> {
    vec![
        NodeSpec::new("mixer").f("shift_hz", shift_hz),
        NodeSpec::new("decimate").i("factor", decimate as i64),
        NodeSpec::new("fsk_detect")
            .f("reset_us", reset_us as f64)
            .f("min_separation_hz", deviation_hz),
        NodeSpec::new("protocol_decode"),
    ]
}
