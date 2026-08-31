//! Where Mode S is.
//!
//! All that is left here is the tuning question. The demodulator lives in
//! `dsp::modes`, the frame format in `decode::adsb`, and the node wiring them
//! together in `nodes::modes_nodes`, which the receiver's graph attaches like
//! any other decoder.

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

/// Slowest sample rate the 1 us bits survive. The node refuses anything below
/// this too; asking here as well keeps a doomed decoder out of the graph.
const MIN_RATE: f64 = 2_000_000.0;

/// Whether this tuning is one where Mode S can be decoded.
pub fn tuned_to_mode_s(center: f64, rate: f64) -> bool {
    rate >= MIN_RATE && (center - CENTER_HZ).abs() <= TOLERANCE_HZ
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_wideband_path_only_runs_where_mode_s_is() {
        // It costs a pass over every sample, so it must not run while the
        // receiver is listening to anything else.
        assert!(tuned_to_mode_s(1_090_000_000.0, 2_400_000.0));
        // Nudged off centre to dodge the DC spur, still on 1090.
        assert!(tuned_to_mode_s(1_089_950_000.0, 2_400_000.0));
        // Far enough off that the signal is in the corner of the band.
        assert!(!tuned_to_mode_s(1_089_000_000.0, 2_400_000.0));
        assert!(!tuned_to_mode_s(433_920_000.0, 2_400_000.0));
        assert!(!tuned_to_mode_s(95_800_000.0, 2_400_000.0));
    }

    #[test]
    fn a_span_too_narrow_for_one_microsecond_bits_is_refused() {
        // Under two samples a bit the two halves cannot be told apart, so a
        // decoder here would report noise rather than nothing.
        assert!(!tuned_to_mode_s(1_090_000_000.0, 1_024_000.0));
        assert!(tuned_to_mode_s(1_090_000_000.0, 2_048_000.0));
    }
}
