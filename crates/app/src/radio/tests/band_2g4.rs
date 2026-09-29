use super::*;

/// The BLE capture: 2 s of advertising channel 38, tuned onto the channel
/// so the packets are read across the tuner's own DC spike.
fn ble_fixture() -> Option<common::IqBuf> {
    let p = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../testdata/offair/gfsk_ble_2426M_20000k.cs8");
    if !p.exists() {
        return None;
    }
    sources::FileSource::open(&p).ok()?.read_all().ok()
}

/// Bluetooth advertising, through the whole receiver: the 2.4 GHz block
/// puts `auto` on the span, `auto` runs the BLE front end across it
/// because channel 38 is inside it, and what comes back is what the
/// devices in the room were saying.
///
/// Every packet counted here passed the link layer's CRC-24, so a run
/// that produces the wrong number is a demodulator that got worse rather
/// than a threshold that moved.
#[test]
fn bluetooth_advertising_is_found_and_read() {
    let Some(buf) = ble_fixture() else {
        eprintln!("skipping: testdata/offair/gfsk_ble_2426M_20000k.cs8 is local only");
        return;
    };
    // The shipped table rather than whatever is in this machine's config:
    // a block the operator deleted should not fail the corpus.
    let fronts = crate::scanners::Scanners::default()
        .fronts(crate::scanners::Span::new(buf.center.as_f64(), buf.rate.as_f64()));
    assert!(
        fronts.iter().any(|f| f.front == crate::scanners::Front::Auto),
        "the table put nothing on a span covering channel 38: {fronts:?}"
    );
    let mut plan = replay_plan(&buf, false);
    plan.fronts = fronts;
    let mut rx = crate::chain::Receiver::build(&plan, crate::chain::Sinks::default()).unwrap();
    let out = replay_blocks(&mut rx, &buf);
    let ble = read_by(&out, "ble");
    // Seven of the eight in the capture. It was six until the channel
    // filter was split into a coarse and a sharp stage, which is a
    // cleaner passband as well as a third of the multiplies.
    assert_eq!(ble.len(), 7, "read {} of the 8 advertisements in the capture", ble.len());
    for r in &ble {
        assert!(checked(r), "a packet without its CRC got through: {r:?}");
        assert!(
            (r.freq() - 2_426_000_000.0).abs() < 1e6,
            "reported at {} Hz rather than on channel 38",
            r.freq()
        );
        assert_eq!(channel(r), Some(38), "read as {}", r.detail());
        assert!(who(r).is_some(), "no advertiser named on {}", r.detail());
    }
    every_row_carries_its_measurements(&ble);
}

fn wifi_fixture() -> Option<common::IqBuf> {
    let p = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../testdata/offair/ofdm_wifi_frames_2462M_20000k.cs8");
    if !p.exists() {
        return None;
    }
    sources::FileSource::open(&p).ok()?.read_all().ok()
}

/// The channel view over the same capture: who is on which channel, read
/// off the channel every decode carries rather than off its fields.
///
/// The capture is one 20 MHz span parked on channel 11, and the two
/// devices talking on it are 70:03:9F:0D:A9:8D and A8:29:48:F4:91:C0,
/// the same pair `testdata/decode.toml` names. Neither beacons
/// here, so neither claims a channel and both are filed where they were
/// heard, which is all a receiver can honestly say about a capture with
/// no beacon in it.
#[test]
fn the_channel_view_lists_who_is_on_the_channel() {
    let Some(buf) = wifi_fixture() else {
        eprintln!("skipping: ofdm_wifi_frames_2462M_20000k.cs8 absent, run testdata/fetch.sh");
        return;
    };
    let mut plan = replay_plan(&buf, false);
    plan.fronts = crate::scanners::Scanners::default()
        .fronts(crate::scanners::Span::new(buf.center.as_f64(), buf.rate.as_f64()));
    let mut rx = crate::chain::Receiver::build(&plan, crate::chain::Sinks::default()).unwrap();
    let out = replay_blocks(&mut rx, &buf);
    let read = read_by(&out, "wifi").len();
    let st = rx.channel_status().expect("the channel map reports itself");
    assert_eq!(st.loads.len(), 1, "one span, one channel: {:?}", st.loads);
    let ch = &st.loads[0];
    assert_eq!((ch.plan, ch.number), (common::ChannelPlan::Wifi, 11));
    assert_eq!(ch.overlapping, 0, "nothing else was heard to reach channel 11");
    assert_eq!(ch.stations, st.stations.len() as u32);
    assert_eq!(st.stations.len(), 2, "expected the two devices: {:?}", st.stations);
    let mut ids: Vec<&str> = st.stations.iter().map(|s| s.id.as_str()).collect();
    ids.sort();
    assert_eq!(ids, vec!["70:03:9F:0D:A9:8D", "A8:29:48:F4:91:C0"]);
    for s in &st.stations {
        assert_eq!(s.channel, 11);
        assert_eq!(s.heard_on, 11);
        assert_eq!(s.width_hz, 20_000_000);
        // No beacon, so nothing says what protects the network.
        assert_eq!(s.secrecy, common::Secrecy::Unsaid);
        assert!(s.rssi_dbfs.is_finite() && s.best_rssi_dbfs >= s.rssi_dbfs);
    }
    // Every frame the receiver read is filed under one of the two: the
    // channel view and the packet list cannot disagree about how much
    // was heard. Ninety-two of the capture's ninety-four frames reach
    // the bus here, the same floor `testdata/decode.toml`
    // pins.
    let filed: usize = st.stations.iter().map(|s| s.packets as usize).sum();
    assert!(read >= 88, "the receiver read {read} frames, expected 92");
    assert_eq!(filed, read, "{filed} frames filed against {read} read");
}

/// DJI DroneID through the whole receiver: the 2.4 GHz block puts `auto`
/// on the span, `auto` runs the DroneID front end across it because one
/// of the five centres is inside it, and what comes back is the aircraft
/// naming itself.
///
/// The count and the sequence numbers are the same nine bursts
/// `nodes/tests/droneid_capture.rs` reads through the node alone, so a
/// difference between the two is the receiver around the front end and
/// not the front end. Nothing decoded here at all until the front end
/// read the span rather than a source cut out of it, because the source
/// the detector opened for a 720 us burst is not the 10 MHz channel the
/// frame occupies.
#[test]
fn a_drone_naming_itself_is_read_off_the_span() {
    let p = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../testdata/droneid_mini4k_2444.5M_15360k.cs8");
    if !p.exists() {
        eprintln!("skipping: droneid_mini4k_2444.5M_15360k.cs8 absent, run testdata/fetch.sh");
        return;
    }
    let buf = sources::FileSource::open(&p).unwrap().read_all().unwrap();
    let mut rx = replay_receiver(&buf, None).unwrap();
    let out = replay_blocks(&mut rx, &buf);
    let dji = read_by(&out, "droneid");
    // The aircraft's own transmission counter, read back out of the
    // frame: which bursts were caught is evidence about the receiver,
    // and a sequence number is not a fact about the world.
    let seq: Vec<u16> = dji
        .iter()
        .filter_map(|r| decode::droneid::parse(&r.bytes()[4..]).map(|f| f.sequence))
        .collect();
    assert_eq!(seq, [437, 438, 439, 440, 440, 441, 442, 444, 444]);
}

/// A survey built by replaying the BLE capture through the whole
/// receiver, with the GPS saying the receiver was in one place.
///
/// The point of the test is the seam between the three parts: the front
/// end finds packets, the survey node turns a decode into an identity,
/// and the database holds one row per transmitter with the position the
/// receiver was at. A device list built from a capture whose contents are
/// known is the only way to see all three working at once.
#[test]
fn a_survey_records_the_devices_heard_and_where_from() {
    let Some(buf) = ble_fixture() else {
        eprintln!("skipping: testdata/offair/gfsk_ble_2426M_20000k.cs8 is local only");
        return;
    };
    let fronts = crate::scanners::Scanners::default()
        .fronts(crate::scanners::Span::new(buf.center.as_f64(), buf.rate.as_f64()));
    let mut plan = replay_plan(&buf, false);
    plan.fronts = fronts;
    let mut rx = crate::chain::Receiver::build(&plan, crate::chain::Sinks::default()).unwrap();

    let dir =
        common::platform::scratch_dir().join(format!("waveshark-survey-{}", std::process::id()));
    let path = dir.join("survey.sqlite");
    let _ = std::fs::remove_file(&path);
    plan.settings.survey_path = Some(path.clone());
    rx.apply_settings(&plan);
    rx.set_fix(Some(gps::Fix {
        lat: 53.5137,
        lon: -6.2431,
        hdop: Some(0.9),
        ..Default::default()
    }));

    let _ = replay_blocks(&mut rx, &buf);
    let rows = rx.survey_devices(survey::Query::default());
    assert!(!rows.is_empty(), "the capture decodes and nothing was recorded");
    assert!(rows.iter().all(|d| d.protocol == "ble"), "{rows:?}");
    // The advertiser that dominates this capture.
    let d = rows
        .iter()
        .find(|d| d.ident == "6C:70:CB:EF:72:4D")
        .unwrap_or_else(|| panic!("the Samsung advertiser is missing: {rows:?}"));
    assert!(d.packets >= 4, "only {} receptions attributed to it", d.packets);
    // A sighting says where the receiver was, not where the device is.
    let s = rx.survey_sightings(d.id);
    assert!(!s.is_empty());
    assert_eq!((s[0].lat, s[0].lon), (Some(53.5137), Some(-6.2431)));
    assert_eq!(d.best_lat, Some(53.5137), "the strongest sighting keeps its position");
    let _ = std::fs::remove_dir_all(&dir);
}
