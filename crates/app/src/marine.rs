//! Where AIS is.
//!
//! The tuning question only, the shape `modes.rs` has for Mode S. The
//! demodulator lives in `dsp::ais`, the message tables in `decode::ais`, and
//! the node wiring them together in `nodes::ais_nodes`.

use dsp::ais::CHANNEL_HZ;

/// Width one AIS channel occupies, which both of them must clear the edge of
/// the span by. A channel demodulated through the anti-alias filter's skirt
/// reads as silence, which looks exactly like an empty band.
const CHANNEL_HZ_WIDTH: f64 = 25_000.0;

/// The narrowest span that can hold both channels and their bandwidth. They
/// are 50 kHz apart, so this is not a matter of taste.
const MIN_RATE: f64 = 150_000.0;

/// Whether this tuning is one where AIS can be decoded.
///
/// Both channels have to be inside the span, which is the real constraint: a
/// receiver on 162.025 alone would hear half the traffic, since stations
/// alternate between the two.
pub fn tuned_to_ais(center: f64, rate: f64) -> bool {
    if rate < MIN_RATE {
        return false;
    }
    let edge = rate / 2.0 - CHANNEL_HZ_WIDTH;
    CHANNEL_HZ.iter().all(|c| (c - center).abs() <= edge)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_ais_path_runs_where_both_channels_are_in_the_span() {
        assert!(tuned_to_ais(162_000_000.0, 2_400_000.0));
        // Nudged off centre to dodge the DC spur, both channels still inside.
        assert!(tuned_to_ais(162_050_000.0, 2_400_000.0));
        // Elsewhere in marine VHF: the channels are not in the span at all.
        assert!(!tuned_to_ais(157_000_000.0, 2_400_000.0));
        assert!(!tuned_to_ais(1_090_000_000.0, 2_400_000.0));
    }

    #[test]
    fn a_span_that_holds_only_one_channel_is_refused() {
        // 50 kHz apart, so a span that covers one and clips the other would
        // hear half the traffic and look like a quiet band rather than a
        // misconfigured one.
        assert!(!tuned_to_ais(161_975_000.0, 60_000.0));
        assert!(tuned_to_ais(162_000_000.0, 200_000.0));
    }
}
