//! The 1090 MHz path, as a graph like every other decoder.
//!
//! This module is the tuning decision and nothing else: whether 1090 MHz is
//! what the receiver is pointed at, and if so, running the graph that decodes
//! it and turning what falls out into packet log rows. The demodulator lives
//! in `dsp::modes`, the frame format in `decode::adsb`, and the node wiring
//! them together in `nodes::modes_nodes`, so the chain view, the parameter
//! surface and the latency accounting all reach it the same way they reach an
//! ISM chain.

use crate::radio::DecodeRecord;
use common::C32;
use pipeline::graph::Graph;
use pipeline::port::StreamSpec;

/// The only frequency Mode S is ever on.
const CENTER_HZ: f64 = 1_090_000_000.0;

/// How far off 1090 the dial may sit and still count as tuned to it.
///
/// The demodulator works on the envelope, which does not care about a small
/// frequency offset, so a receiver nudged off centre to dodge the DC spur
/// still decodes. A larger offset is not worth allowing: it would only mean
/// paying for a pass over every sample while the signal sits in the corner of
/// the band with the rest of the spectrum's noise on top of it.
const TOLERANCE_HZ: f64 = 100_000.0;

/// Occupied bandwidth of a Mode S transmission, for the log's channel column.
const BAND_HZ: f64 = 2_000_000.0;

/// Slowest sample rate the 1 us bits survive. The node refuses anything below
/// this too; checking here as well keeps a doomed graph from being built.
const MIN_RATE: f64 = 2_000_000.0;

pub struct ModeS {
    graph: Graph,
    rate: f64,
}

impl ModeS {
    /// A decoder for this tuning, or `None` unless the dial is on 1090 MHz.
    pub fn for_tuning(center: f64, rate: f64) -> Option<Self> {
        if rate < MIN_RATE || (center - CENTER_HZ).abs() > TOLERANCE_HZ {
            return None;
        }
        let spec = StreamSpec::iq(rate, common::Hz(center as u64));
        // A graph that will not build is a bug in the chain, not a runtime
        // condition, but it must not take the receiver down either.
        match nodes::adsb_graph(spec) {
            Ok(graph) => Some(Self { graph, rate }),
            Err(e) => {
                tracing::warn!("no 1090 MHz chain: {e}");
                None
            }
        }
    }

    /// Shape of the running chain, for the chain view.
    pub fn topology(&self) -> pipeline::graph::Topology {
        self.graph.topology()
    }

    /// Delay through the chain, in milliseconds.
    pub fn latency_ms(&self) -> f64 {
        self.graph.output_latency() as f64 / self.rate.max(1.0) * 1e3
    }

    pub fn process(&mut self, iq: &[C32], out: &mut Vec<DecodeRecord>) {
        // Stamped at the start of the block, not at the moment the frame fell
        // out of it: the aircraft transmitted somewhere inside the block, and
        // the whole block has already been read by the time this runs.
        let block = std::time::Duration::from_secs_f64(iq.len() as f64 / self.rate.max(1.0));
        let now = std::time::Instant::now() - block;
        let buf = self.graph.input_buf();
        buf.clear();
        buf.iq_mut().extend_from_slice(iq);
        let Ok(events) = self.graph.run() else { return };
        for ev in events {
            let pipeline::event::Event::Decoded(d) = ev else { continue };
            out.push(DecodeRecord {
                at: now,
                freq: CENTER_HZ,
                // Mode S occupies the whole band it is transmitted in; there
                // is no channel to speak of, and nothing else is near enough
                // to be confused with it.
                channel_hz: BAND_HZ,
                model: d.protocol.to_string(),
                modulation: d.modulation.unwrap_or("PPM"),
                detail: d.detail.clone().or_else(|| d.text.clone()).unwrap_or_default(),
                fields: d.fields.clone(),
                media_type: d.media_type,
                rssi_dbfs: d.rssi_dbfs.unwrap_or(f32::NAN),
                snr_db: d.snr_db.unwrap_or(f32::NAN),
                bytes: d.payload.clone(),
                crc: d.crc_ok,
            });
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_wideband_path_only_runs_where_mode_s_is() {
        // It costs a pass over every sample, so it must not run while the
        // receiver is listening to anything else.
        assert!(ModeS::for_tuning(1_090_000_000.0, 2_400_000.0).is_some());
        // Nudged off centre to dodge the DC spur, still on 1090.
        assert!(ModeS::for_tuning(1_089_950_000.0, 2_400_000.0).is_some());
        // Far enough off that the signal is in the corner of the band.
        assert!(ModeS::for_tuning(1_089_000_000.0, 2_400_000.0).is_none());
        assert!(ModeS::for_tuning(433_920_000.0, 2_400_000.0).is_none());
        assert!(ModeS::for_tuning(95_800_000.0, 2_400_000.0).is_none());
    }

    #[test]
    fn a_span_too_narrow_for_one_microsecond_bits_is_refused() {
        // Under two samples a bit the two halves cannot be told apart, so a
        // decoder here would report noise rather than nothing.
        assert!(ModeS::for_tuning(1_090_000_000.0, 1_024_000.0).is_none());
        assert!(ModeS::for_tuning(1_090_000_000.0, 2_048_000.0).is_some());
    }

    #[test]
    fn the_chain_is_a_graph_like_any_other() {
        // Which is the point of this module being thin: the chain view asks a
        // graph what shape it is, and does not care what it decodes.
        let m = ModeS::for_tuning(1_090_000_000.0, 2_400_000.0).expect("a chain");
        let topo = m.topology();
        assert!(!topo.nodes.is_empty(), "a chain with no nodes is not a chain");
    }
}
