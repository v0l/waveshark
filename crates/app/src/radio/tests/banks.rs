use super::*;

#[test]
fn each_bank_splits_the_span_to_the_width_its_front_end_wants() {
    for rate in [250_000.0, 1_024_000.0, 2_400_000.0, 20_000_000.0] {
        for (want, lo, hi) in [
            (12_500.0, 6_000.0, 30_000.0),
            (OOK_CHANNEL_HZ, 15_000.0, 70_000.0),
            // The wide tier of the scanner table, in `scanners::DEFAULT_WIDTHS`.
            (125_000.0, 60_000.0, 260_000.0),
            (500_000.0, 240_000.0, 1_100_000.0),
        ] {
            let n = nodes::BankNode::channels_for(rate, want);
            assert_eq!(n % 2, 0, "{rate} at {want} Hz gave an odd count {n}");
            let width = rate / n as f64;
            assert!(
                (lo..hi).contains(&width) || n == 2,
                "{rate} at {want} Hz gave {n} channels of {width} Hz"
            );
        }
    }
}

#[test]
fn an_ism_band_is_watched_for_sources_and_not_channelized() {
    // What the shipped table asks for on an ISM band: one detector that
    // finds transmitters where they are, rather than a set of channel
    // grids at guessed widths.
    let rx = replay_receiver(&empty_buf(2_400_000.0, Hz::mhz(868)), None).unwrap();
    let labels: Vec<String> = rx.topology().nodes.iter().map(|n| n.label.clone()).collect();
    assert!(rx.has_sources(), "no source detector on the 868 MHz band: {labels:?}");
    assert!(
        rx.bank_channels().is_empty(),
        "a bank tier is still running: {:?}",
        rx.bank_channels()
    );
    assert!(rx.live_sources().is_empty(), "an empty band has no sources");
}

#[test]
fn a_narrow_span_still_gets_a_usable_bank() {
    // Two channels is the floor: the channelizer needs an even count and
    // one channel would just be a decimator.
    assert_eq!(nodes::BankNode::channels_for(1_000.0, OOK_CHANNEL_HZ), 2);
    assert!(
        nodes::BankNode::channels_for(1e9, OOK_CHANNEL_HZ) <= 1024,
        "the count has to stay bounded"
    );
}
