use super::*;

#[test]
fn a_decode_channel_set_to_lte_names_both_cells_on_the_band_28_carrier() {
    let p = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../testdata/offair/ofdm_lte_762M_12000k.cs16");
    if !p.exists() {
        eprintln!("skipping: ofdm_lte_762M_12000k.cs16 absent, run testdata/fetch.sh");
        return;
    }
    let buf = sources::FileSource::open(&p).unwrap().read_all().unwrap();
    let mut plan = replay_plan(&buf, false);
    plan.fronts.clear();
    plan.channels = vec![ChannelSpec {
        id: 1,
        label: "LTE".into(),
        offset_hz: 762.95e6 - buf.center.as_f64(),
        mode: ChanMode::Decode("lte".into()),
        bandwidth_hz: None,
        audio_low_hz: None,
        squelch_db: None,
        voice: false,
        reads: None,
        agc: true,
        blanker: None,
        denoise: false,
        denoise_db: dsp::denoise::DEFAULT_DEPTH_DB,
        agc_tune: nodes::AgcTune::default(),
        notch: false,
        tx: None,
        tone: None,
    }];
    let mut rx = crate::chain::Receiver::build(&plan, Default::default()).expect("a channel");
    let out = replay_blocks(&mut rx, &buf);

    let mib = read_as(&out, "lte", "mib");
    let si = read_as(&out, "lte", "system_information");
    assert_eq!(
        (mib.len(), si.len()),
        (2, 3),
        "a MIB and a SIB1 off each sector, and one SIB2 and SIB3 message: {:?}",
        out.iter().map(|r| r.detail()).collect::<Vec<_>>()
    );
    assert_eq!(read_as(&out, "lte", "network").len(), 4, "SIB5 and SIB7 off each sector");
    let mut cells: Vec<String> = si.iter().filter_map(|r| who(r)).collect();
    cells.sort();
    assert_eq!(cells, ["272-03-40111-11350600", "272-03-40111-11350601"]);
    for r in mib.iter().chain(&si) {
        assert!(checked(r));
        assert_eq!(r.freq(), 763e6, "on the raster the channel was snapped to");
    }
    every_row_carries_its_measurements(&mib);
    every_row_carries_its_measurements(&si);
    assert_eq!(rx.decoding()[0].1.acquisition, Some(pipeline::Acquisition::Locked));
}

/// The whole receiver on a 30.72 MS/s span over Madrid's band 20, with
/// nothing but the shipped scanner table to go on: the `LTE 800` block puts
/// `auto` over the span, `auto` places the LTE front end across it, and the
/// front end finds all three operators' carriers by itself.
#[test]
fn the_scanner_table_reads_all_three_madrid_band_20_operators_without_being_told_a_carrier() {
    let p = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../testdata/lte_b20_madrid_806M_30720k.cs8");
    if !p.exists() {
        eprintln!("skipping: lte_b20_madrid_806M_30720k.cs8 absent, run testdata/fetch.sh");
        return;
    }
    let buf = sources::FileSource::open(&p).unwrap().read_all().unwrap();
    let mut plan = replay_plan(&buf, false);
    plan.fronts = crate::scanners::Scanners::default().fronts_in(
        crate::bands::Plan::Europe,
        crate::scanners::Span::new(buf.center.as_f64(), buf.rate.as_f64()),
    );
    let mut rx = crate::chain::Receiver::build(&plan, Default::default()).expect("a chain");
    let out = replay_blocks(&mut rx, &buf);
    let mut mibs: Vec<(f64, String)> =
        read_as(&out, "lte", "mib").iter().map(|r| (r.freq(), r.detail())).collect();
    mibs.sort_by(|a, b| a.0.total_cmp(&b.0));
    assert_eq!(
        mibs,
        [
            (796e6, "site code 144 10 MHz wide".to_string()),
            (806e6, "site code 380 10 MHz wide".to_string()),
            (816e6, "site code 324 10 MHz wide".to_string()),
        ]
    );
    let mut cells: Vec<String> =
        read_as(&out, "lte", "system_information").iter().filter_map(|r| who(r)).collect();
    cells.sort();
    assert_eq!(cells, ["214-01-278-73430023", "214-03-1371-73430136", "214-07-28673-73816853"]);
    let mut held: Vec<(f64, f64)> = rx
        .live_sources()
        .iter()
        .filter(|s| s.locked_to == Some("lte"))
        .map(|s| (s.center_hz, s.bandwidth_hz))
        .collect();
    held.sort_by(|a, b| a.0.total_cmp(&b.0));
    assert_eq!(
        held,
        [(796e6, 9e6), (806e6, 9e6), (816e6, 9e6)],
        "each carrier's occupied 9 MHz locked to LTE, so nothing else is opened on it"
    );
}
