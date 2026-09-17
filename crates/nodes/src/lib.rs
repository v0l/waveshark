//! Graph nodes and the registry that builds them by name.
//!
//! The registry is what lets a chain be described as data. That matters for
//! ambiguous signals: when a burst does not decode, the answer is usually to
//! change the chain (a different decimation, a longer reset gap, FM
//! discrimination instead of an envelope), and that should be a
//! reconfiguration rather than a recompile.

pub mod acars_nodes;
pub mod ais_nodes;
pub mod aprs_nodes;
pub mod apt_nodes;
pub mod auto;
pub mod bank;
pub mod bank_node;
pub mod beacondb_nodes;
pub mod ble_nodes;
pub mod capture_nodes;
pub mod channel_nodes;
pub mod decode_nodes;
pub mod dfm_nodes;
pub mod dmr_nodes;
pub mod droneid_nodes;
pub mod dsp_nodes;
pub mod dvbt_nodes;
pub mod eas_nodes;
pub mod elrs_nodes;
pub mod epirb_nodes;
pub mod feed_nodes;
pub mod filter_nodes;
pub mod flex_nodes;
pub mod frame_meter;
pub mod gsm_nodes;
pub mod homeassistant_nodes;
pub mod ident_nodes;
pub mod imet_nodes;
pub mod iq_tx;
pub mod keyed;
pub mod lms6_nodes;
pub mod lora_nodes;
pub mod m10_nodes;
pub mod m17_nodes;
pub mod mdc_nodes;
pub mod meisei_nodes;
pub mod mic_in;
pub mod mod_nodes;
pub mod modes_nodes;
pub mod morse_nodes;
pub mod mrz_nodes;
pub mod nrf24_nodes;
pub mod p25_nodes;
pub mod packet_nodes;
pub mod pocsag_nodes;
pub mod protocol;
pub mod rs41_nodes;
pub mod rtty_nodes;
pub mod scan_nodes;
pub mod scope_nodes;
pub mod sink_nodes;
pub mod source_nodes;
pub mod sstv_nodes;
pub mod sub_tx;
pub mod survey_nodes;
pub mod tetra_nodes;
pub mod twotone_nodes;
pub mod tx_nodes;
pub mod vdl2_nodes;
pub mod video_nodes;
pub mod wefax_nodes;
pub mod wfm;
pub mod wifi_nodes;
pub mod wigle_nodes;
pub mod wmbus_nodes;

pub use acars_nodes::AcarsNode;
pub use ais_nodes::AisNode;
pub use aprs_nodes::AprsNode;
pub use auto::{AUTO_OPEN_DB, AutoNode};
pub use bank::{ChannelBank, ChannelEvent, Gating};
pub use bank_node::BankNode;
pub use beacondb_nodes::{BeaconDbNode, BeaconDbStatus};
pub use ble_nodes::BleNode;
pub use capture_nodes::IqCaptureNode;
pub use channel_nodes::{ChannelLoad, ChannelMapNode, ChannelStatus, Station};
pub use decode_nodes::{
    AskDetectNode, BurstRouteNode, FskDetectNode, ProtocolDecodeNode, PulseDetectNode, UNKNOWN,
};
pub use dfm_nodes::DfmNode;
pub use dmr_nodes::DmrNode;
pub use dsp_nodes::{
    AgcNode, AgcPreset, DecimateNode, DeemphasisNode, EnvelopeNode, FmDemodNode, HighBlendNode,
    MixerNode, RealDecimateNode, SquelchKind, SquelchNode, SsbDemodNode,
};
pub use elrs_nodes::ElrsNode;
pub use feed_nodes::{FEED_KINDS, FeedKind, FeedNode, FeedSpec, feed_kind};
pub use filter_nodes::{FirFilterNode, IirFilterNode, RealFir};
pub use flex_nodes::FlexNode;
pub use frame_meter::FrameMeter;
pub use homeassistant_nodes::{
    Broker, HomeAssistantNode, HomeAssistantStatus, Publish, Publisher as HomeAssistantPublisher,
    mqtt_packet,
};
pub use imet_nodes::ImetNode;
pub use keyed::{Keyed, keyed, keyed_mut};
pub use lms6_nodes::Lms6Node;
pub use lora_nodes::LoraNode;
pub use m10_nodes::M10Node;
pub use m17_nodes::M17Node;
pub use mdc_nodes::MdcNode;
pub use meisei_nodes::MeiseiNode;
pub use mic_in::MicInNode;
pub use mod_nodes::{
    AmModNode, AskModNode, Carrier, FM_DEVIATION_HZ, FmModNode, FskModNode, NBFM_DEVIATION_HZ,
    OokModNode, WBFM_DEVIATION_HZ,
};
pub use modes_nodes::ModeSNode;
pub use morse_nodes::MorseNode;
pub use mrz_nodes::MrzNode;
pub use nrf24_nodes::Nrf24Node;
pub use packet_nodes::{DedupeNode, PacketDecodeNode};
pub use pocsag_nodes::PocsagNode;
pub use protocol::{Placed, Placement, Protocol, Shape, Stickiness};
pub use rs41_nodes::Rs41Node;
pub use rtty_nodes::RttyNode;
pub use scan_nodes::{BandScanNode, Found, Key, Linger, Lock, OnHit, ScanStatus};
pub use scope_nodes::{ScopeFrame, ScopeNode};
pub use sink_nodes::{
    AdcHealth, DcBlockNode, PacketBusNode, PacketSink, Ring, RingNode, SpectrumNode,
};
pub use source_nodes::{SourceDecodeNode, SourceDetectNode};
pub use sstv_nodes::SstvNode;
pub use sub_tx::SubTxNode;
pub use survey_nodes::SurveyNode;
pub use tetra_nodes::TetraNode;
pub use tx_nodes::{
    DEFAULT_VOX_TAIL_MS, DEFAULT_VOX_THRESHOLD, Heard, MIC_GAIN_MAX, MicNode, MorseKeyNode,
    MorseTxNode, ToneNode, TxClockNode, TxMonitorNode, TxSinkNode, VoxNode,
};
pub use vdl2_nodes::Vdl2Node;
pub use video_nodes::VideoNode;
pub use wfm::WfmDemodNode;
pub use wifi_nodes::WifiNode;
pub use wigle_nodes::{Account, WigleNode, WigleStatus};
pub use wmbus_nodes::WmbusNode;

use common::Result;
use pipeline::node::Node;
use pipeline::registry::{Registry, Settings, SettingsExt, StageDesc};
use pipeline::{Graph, StreamSpec};

/// The band a stage is limited to inside its input, as its description
/// carries it, or `None` for all of it.
///
/// Read here rather than by whatever builds the graph, so the two halves of
/// the setting are spelled once and the node that obeys them is the node
/// that reads them.
pub fn band_of(settings: &Settings) -> Option<(f64, f64)> {
    let (lo, hi) = (settings.f64_or("band_lo_hz", 0.0), settings.f64_or("band_hi_hz", 0.0));
    (hi > lo).then_some((lo, hi))
}

/// Where a sink spools what it has to send, as its description carries it,
/// or the folder that sink keeps its own spool in.
///
/// Read here for the same reason as [`band_of`]: two sinks upload to two
/// services and both answer the setting the same way.
pub fn spool_dir(settings: &Settings, fallback: fn() -> std::path::PathBuf) -> std::path::PathBuf {
    match settings.str_or("spool", "") {
        "" => fallback(),
        p => std::path::PathBuf::from(p),
    }
}

/// Every node type compiled into this build.
///
/// A description and a builder each, both belonging to the module that owns
/// the node, so a default is spelled once where the node reads it rather than
/// restated here and left to drift.
const STAGES: &[(StageDesc, fn(&Settings) -> Result<Box<dyn Node>>)] = &[
    (tx_nodes::TX_CLOCK, tx_nodes::build_tx_clock),
    (tx_nodes::TX_MONITOR, tx_nodes::build_tx_monitor),
    (tx_nodes::TONE, tx_nodes::build_tone),
    (tx_nodes::VOX, tx_nodes::build_vox),
    (tx_nodes::MORSE_TX, tx_nodes::build_morse_tx),
    (tx_nodes::MORSE_KEY, tx_nodes::build_morse_key),
    (sub_tx::SUB_TX, sub_tx::build_sub_tx),
    (pocsag_nodes::POCSAG_TX, pocsag_nodes::build_tx),
    (rtty_nodes::RTTY_TX, rtty_nodes::build_tx),
    (aprs_nodes::APRS_TX, aprs_nodes::build_tx),
    (iq_tx::IQ_TX, iq_tx::build_iq_tx),
    (mod_nodes::OOK_MOD, mod_nodes::build_ook_mod),
    (mod_nodes::FSK_MOD, mod_nodes::build_fsk_mod),
    (mod_nodes::ASK_MOD, mod_nodes::build_ask_mod),
    (mod_nodes::AM_MOD, mod_nodes::build_am_mod),
    (mod_nodes::FM_MOD, mod_nodes::build_fm_mod),
    (dsp_nodes::MIXER, dsp_nodes::build_mixer),
    (dsp_nodes::DECIMATE, dsp_nodes::build_decimate),
    (dsp_nodes::REAL_DECIMATE, dsp_nodes::build_real_decimate),
    (dsp_nodes::ENVELOPE, dsp_nodes::build_envelope),
    (dsp_nodes::FM_DEMOD, dsp_nodes::build_fm_demod),
    (dsp_nodes::DEEMPHASIS, dsp_nodes::build_deemphasis),
    (dsp_nodes::SSB_DEMOD, dsp_nodes::build_ssb_demod),
    (dsp_nodes::HIGH_BLEND, dsp_nodes::build_high_blend),
    (dsp_nodes::AGC, dsp_nodes::build_agc),
    (dsp_nodes::SQUELCH, dsp_nodes::build_squelch),
    (wfm::DESC, wfm::build),
    (wfm::RDS_TX, wfm::build_rds_tx),
    (filter_nodes::FIR_FILTER, filter_nodes::build_fir),
    (filter_nodes::IIR_FILTER, filter_nodes::build_iir),
    // The front ends the scanner table puts on a span. Registered like any
    // other stage so that the graph the receiver derives for itself is a
    // description rather than a special case, and so an operator can put one
    // anywhere rather than only where the table would have.
    (modes_nodes::DESC, modes_nodes::build),
    (ais_nodes::DESC, ais_nodes::build),
    (gsm_nodes::DESC, gsm_nodes::build),
    (mic_in::DESC, mic_in::build),
    (sstv_nodes::DESC, sstv_nodes::build),
    (sstv_nodes::SSTV_TX, sstv_nodes::build_tx),
    (apt_nodes::DESC, apt_nodes::build),
    (wefax_nodes::DESC, wefax_nodes::build),
    (vdl2_nodes::DESC, vdl2_nodes::build),
    (dvbt_nodes::DESC, dvbt_nodes::build),
    (ident_nodes::DESC, ident_nodes::build),
    (dvbt_nodes::TS_SOURCE, dvbt_nodes::build_ts_source),
    (dvbt_nodes::DVBT_MOD, dvbt_nodes::build_dvbt_mod),
    (video_nodes::DESC, video_nodes::build),
    (ble_nodes::DESC, ble_nodes::build),
    (ble_nodes::BLE_TX, ble_nodes::build_tx),
    (wifi_nodes::DESC, wifi_nodes::build),
    (droneid_nodes::DESC, droneid_nodes::build),
    (acars_nodes::DESC, acars_nodes::build),
    (aprs_nodes::DESC, aprs_nodes::build),
    (m17_nodes::DESC, m17_nodes::build),
    (tetra_nodes::DESC, tetra_nodes::build),
    (dmr_nodes::DESC, dmr_nodes::build),
    (p25_nodes::DESC, p25_nodes::build),
    (pocsag_nodes::DESC, pocsag_nodes::build),
    (flex_nodes::DESC, flex_nodes::build),
    (rtty_nodes::DESC, rtty_nodes::build),
    (morse_nodes::DESC, morse_nodes::build),
    (eas_nodes::DESC, eas_nodes::build),
    (mdc_nodes::DESC, mdc_nodes::build),
    (twotone_nodes::DESC, twotone_nodes::build),
    (nrf24_nodes::DESC, nrf24_nodes::build),
    (lora_nodes::DESC, lora_nodes::build),
    (elrs_nodes::DESC, elrs_nodes::build),
    (wmbus_nodes::DESC, wmbus_nodes::build),
    (dfm_nodes::DESC, dfm_nodes::build),
    (epirb_nodes::DESC, epirb_nodes::build),
    (imet_nodes::DESC, imet_nodes::build),
    (lms6_nodes::DESC, lms6_nodes::build),
    (m10_nodes::DESC, m10_nodes::build),
    (meisei_nodes::DESC, meisei_nodes::build),
    (mrz_nodes::DESC, mrz_nodes::build),
    (rs41_nodes::DESC, rs41_nodes::build),
    (bank_node::DESC, bank_node::build),
    (auto::DESC, auto::build),
    (source_nodes::SOURCE_DETECT, source_nodes::build_source_detect),
    (source_nodes::SOURCE_DECODE, source_nodes::build_source_decode),
    (decode_nodes::PULSE_DETECT, decode_nodes::build_pulse_detect),
    (decode_nodes::ASK_DETECT, decode_nodes::build_ask_detect),
    (decode_nodes::FSK_DETECT, decode_nodes::build_fsk_detect),
    (decode_nodes::BURST_ROUTE, decode_nodes::build_burst_route),
    (decode_nodes::PROTOCOL_DECODE, decode_nodes::build_protocol_decode),
    // Where everything that produces packets meets, and what hangs off the
    // far side of it.
    (sink_nodes::PACKET_BUS, sink_nodes::build_packet_bus),
    (sink_nodes::DC_BLOCK, sink_nodes::build_dc_block),
    (sink_nodes::SPECTRUM, sink_nodes::build_spectrum),
    (packet_nodes::PROTOCOLS, packet_nodes::build_protocols),
    (packet_nodes::DEDUPE, packet_nodes::build_dedupe),
    (feed_nodes::DESC, feed_nodes::build),
    (scope_nodes::DESC, scope_nodes::build),
    (capture_nodes::DESC, capture_nodes::build),
    (survey_nodes::DESC, survey_nodes::build),
    (scan_nodes::DESC, scan_nodes::build),
    (channel_nodes::DESC, channel_nodes::build),
    (wigle_nodes::DESC, wigle_nodes::build),
    (beacondb_nodes::DESC, beacondb_nodes::build),
    (homeassistant_nodes::DESC, homeassistant_nodes::build),
];

/// Every node type compiled into this build, ready to make one by name.
pub fn registry() -> Registry {
    let mut r = Registry::new();
    for (desc, build) in STAGES {
        r.register(desc.clone(), *build);
    }
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
pub fn fsk_chain(
    shift_hz: f64,
    decimate: usize,
    deviation_hz: f64,
    reset_us: u32,
) -> Vec<NodeSpec> {
    vec![
        NodeSpec::new("mixer").f("shift_hz", shift_hz),
        NodeSpec::new("decimate").i("factor", decimate as i64),
        NodeSpec::new("fsk_detect")
            .f("reset_us", reset_us as f64)
            .f("min_separation_hz", deviation_hz),
        NodeSpec::new("protocol_decode"),
    ]
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A node's `name` is its registry id, which is what every consumer of a
    /// topology matches on. A node that returns a display label instead is a
    /// box the chain view, the packet bus and the transmit pane cannot find.
    #[test]
    fn every_stage_answers_to_its_registered_name() {
        for (desc, build) in STAGES {
            let node = build(&Settings::new())
                .unwrap_or_else(|e| panic!("{} could not be built: {e}", desc.name));
            assert_eq!(
                node.name(),
                desc.name,
                "{} is registered under a name it does not answer to",
                desc.name
            );
        }
    }
}
