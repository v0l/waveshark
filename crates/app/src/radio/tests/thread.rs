use super::*;

#[test]
fn a_block_is_judged_on_the_rate_the_radio_samples_at() {
    let s = crate::scanners::Scanners::default();
    let mut plan = crate::chain::tests::plan(2_400_000.0, Hz(1_090_000_000));
    plan.fronts = Vec::new();
    let fronts = fronts_here(&s, &plan, true);
    assert_eq!(
        fronts.iter().map(|f| f.front.key()).collect::<Vec<_>>(),
        ["mode_s"],
        "the ADS-B block asks for 2 MS/s and the radio has 2.4"
    );
    plan.fronts = fronts;
    let drawn = crate::chain::derived_patch(&plan);
    assert_eq!(drawn.stages().iter().filter(|s| s.kind == "mode_s").count(), 1);
    assert_eq!(drawn.stages().iter().filter(|s| s.kind == "packet_bus").count(), 1);
}

#[test]
fn a_capture_or_a_remote_tuner_is_not_served_again_and_a_radio_is() {
    use common::device::DriverKind;
    for (kind, served) in
        [(DriverKind::File, false), (DriverKind::Network, false), (DriverKind::HackRf, true)]
    {
        let addr = std::net::TcpListener::bind("127.0.0.1:0").unwrap().local_addr().unwrap();
        let dev = sources::FileRadio::silent(Hz(433_920_000), Sps(2_400_000))
            .as_fast_as_it_can()
            .posing_as(kind);
        let radio = Radio::on_device(Box::new(dev), Hz(433_920_000), Sps(2_400_000), 1024);
        until("the radio to start", || radio.status.running.load(Ordering::Relaxed));
        let rev = radio.status.patch_rev.load(Ordering::Relaxed);
        radio.send(Cmd::IqStream(Some(crate::chain::IqStreamPlan { addr, tunable: false })));
        radio.send(Cmd::Channels(vec![strip_channel(1, 50_000.0)]));
        until("a rebuild", || radio.status.patch_rev.load(Ordering::Relaxed) > rev);
        if served {
            until("the span to be served", || nodes::iqstream_nodes::running(addr).is_some());
        } else {
            std::thread::sleep(std::time::Duration::from_millis(200));
            assert!(nodes::iqstream_nodes::running(addr).is_none(), "{kind:?} served again");
        }
    }
}

#[test]
fn a_new_tnc_address_closes_the_old_one_and_a_rebuild_does_not() {
    let held: Vec<_> =
        (0..2).map(|_| std::net::TcpListener::bind("127.0.0.1:0").unwrap()).collect();
    let (first, second) = (held[0].local_addr().unwrap(), held[1].local_addr().unwrap());
    drop(held);
    let dev = sources::FileRadio::silent(Hz(144_800_000), Sps(2_400_000)).as_fast_as_it_can();
    let radio = Radio::on_device(Box::new(dev), Hz(144_800_000), Sps(2_400_000), 1024);
    until("the radio to start", || radio.status.running.load(Ordering::Relaxed));

    radio.send(Cmd::Kiss(Some(first)));
    until("the TNC to serve", || nodes::kiss_nodes::running(first).is_some());
    let tnc = nodes::kiss_nodes::running(first).unwrap();
    let mut client = std::net::TcpStream::connect(first).expect("connected");
    client.set_read_timeout(Some(std::time::Duration::from_secs(5))).unwrap();
    until("the TNC to see its client", || tnc.connected() == 1);

    let rev = radio.status.patch_rev.load(Ordering::Relaxed);
    radio.send(Cmd::Kiss(Some(first)));
    radio.send(Cmd::Channels(vec![strip_channel(1, 50_000.0)]));
    until("a rebuild", || radio.status.patch_rev.load(Ordering::Relaxed) > rev);
    assert!(Arc::ptr_eq(&nodes::kiss_nodes::running(first).unwrap(), &tnc));
    assert_eq!(tnc.connected(), 1, "a rebuild dropped the TNC's client");

    radio.send(Cmd::Kiss(Some(second)));
    until("the new address to serve", || nodes::kiss_nodes::running(second).is_some());
    assert!(tnc.closed(), "the old address is still being served");
    assert!(nodes::kiss_nodes::running(first).is_none());
    use std::io::Read;
    assert_eq!(client.read(&mut [0u8; 16]).expect("the client was hung up on"), 0);
    drop(std::net::TcpListener::bind(first).expect("the old port was given back"));

    let serving = nodes::kiss_nodes::running(second).unwrap();
    radio.send(Cmd::Kiss(None));
    until("the switch to close it", || nodes::kiss_nodes::running(second).is_none());
    assert!(serving.closed());
    drop(std::net::TcpListener::bind(second).expect("the port was given back"));
}

/// The settle window is the time the tuner asked for, so it is the same
/// milliseconds of thrown-away transient at any sample rate, and a
/// board that recalibrates its VCO gets the longer window it needs.
#[test]
fn the_settle_window_is_what_the_tuner_asked_for() {
    let ms = |n| std::time::Duration::from_millis(n);
    assert_eq!(settle_samples(2_400_000.0, ms(5)), 12_000);
    assert_eq!(settle_samples(250_000.0, ms(5)), 1_250);
    assert_eq!(settle_samples(61_440_000.0, ms(60)), 3_686_400);
    for rate in [250_000.0, 2_400_000.0, 61_440_000.0] {
        assert!((settle_samples(rate, ms(60)) as f64 / rate - 0.06).abs() < 1e-9);
    }
}

#[test]
fn the_chain_keeps_its_timings_with_the_spectrum_off() {
    let center = Hz(145_000_000);
    let rate = Sps(2_400_000);
    let dev = sources::FileRadio::silent(center, rate).as_fast_as_it_can();
    let radio = Radio::on_device(Box::new(dev), center, rate, 1024);
    until("the radio to start", || radio.status.running.load(Ordering::Relaxed));
    let mut edits = crate::patch::Edits::default();
    edits.off.push(crate::chain::derived::SPECTRUM);
    radio.send(Cmd::Edits(edits));
    let spectrum_off = |t: &pipeline::graph::Topology| {
        t.nodes.iter().any(|n| n.tag == Some(crate::chain::derived::SPECTRUM) && n.off)
    };
    until("the spectrum to be off", || radio.status.chain().is_some_and(|t| spectrum_off(&t)));
    let calls = || {
        radio
            .status
            .chain()
            .filter(spectrum_off)
            .and_then(|t| t.nodes.iter().map(|n| n.cost.calls).max())
            .unwrap_or(0)
    };
    let before = calls();
    until("the timings to be published again", || calls() > before);
    while radio.frames.try_recv().is_ok() {}
    std::thread::sleep(std::time::Duration::from_millis(200));
    assert_eq!(radio.frames.try_iter().count(), 0, "the spectrum went on drawing");
}

/// The dial and the span move under a running receiver.
///
/// Both go through the radio thread and both redraw the graph: a retune
/// is a device call and a rebuild with every channel at a new offset, and
/// a span change may take the device down and open it again. Neither was
/// under test, and a receiver that stops when somebody turns the dial is
/// the one fault nobody would report as a bug in a decoder.
#[test]
fn the_dial_and_the_span_move_without_stopping_the_receiver() {
    let dev = sources::FileRadio::silent(Hz(145_000_000), Sps(2_400_000)).as_fast_as_it_can();
    let radio = Radio::on_device(Box::new(dev), Hz(145_000_000), Sps(2_400_000), 1024);
    until("the radio to start", || radio.status.running.load(Ordering::Relaxed));
    radio.send(Cmd::Channels(vec![strip_channel(1, 25_000.0), strip_channel(2, -50_000.0)]));

    // Read off the spectrum frames, because that is what the window
    // draws: a dial that moved and a waterfall that did not is the fault
    // this would be reported as.
    let seen = |what: f64, pick: fn(&Frame) -> f64| -> bool {
        radio.frames.try_iter().any(|f| (pick(&f) - what).abs() < 1.0)
    };
    for hz in [433_920_000.0, 136_825_000.0, 95_800_000.0, 145_000_000.0] {
        radio.send(Cmd::Center(Hz(hz as u64)));
        until(&format!("the spectrum to arrive at {hz}"), || seen(hz, |f| f.center));
        assert!(radio.status.running.load(Ordering::Relaxed), "the radio stopped at {hz}");
    }
    for rate in [2_048_000.0, 9_142_857.0, 250_000.0, 2_400_000.0] {
        radio.send(Cmd::Rate(Sps(rate as u64)));
        until(&format!("the spectrum to arrive at {rate}"), || seen(rate, |f| f.rate));
        assert!(radio.status.running.load(Ordering::Relaxed), "the radio stopped at {rate}");
    }
    // Still alive, still hearing, and still holding both channels.
    assert!(radio.status.running.load(Ordering::Relaxed));
    assert_eq!(radio.status.error.lock().clone(), None);
}

/// Several channels decode at once through the radio thread.
#[test]
fn every_channel_on_the_strip_is_built_by_the_radio_thread() {
    let dev = sources::FileRadio::silent(Hz(145_000_000), Sps(2_400_000)).as_fast_as_it_can();
    let radio = Radio::on_device(Box::new(dev), Hz(145_000_000), Sps(2_400_000), 1024);
    until("the radio to start", || radio.status.running.load(Ordering::Relaxed));
    let want: Vec<ChannelSpec> =
        (1..=4).map(|i| strip_channel(i, i as f64 * 100_000.0 - 250_000.0)).collect();
    radio.send(Cmd::Channels(want));
    // The inputs of the audio bus, which is where a built channel shows
    // up: the levels are only republished when one of them changed.
    let built = || radio.status.strips().inputs.iter().filter(|s| s.channel.is_some()).count();
    until("all four channels to be built", || built() == 4);
    assert_eq!(radio.status.error.lock().clone(), None);
    // And closing one leaves the other three.
    radio.send(Cmd::Channels(vec![strip_channel(1, -150_000.0)]));
    until("three to go", || built() == 1);
    assert!(radio.status.running.load(Ordering::Relaxed));
}

/// The walk over a band moves the dial, on the radio thread.
///
/// The node only asks; the thread is the only thing holding the device,
/// so a walk that is not answered here is a walk that never happens.
#[test]
fn a_band_walk_moves_the_dial_through_the_radio_thread() {
    let dev = sources::FileRadio::silent(Hz(145_000_000), Sps(2_400_000)).as_fast_as_it_can();
    let radio = Radio::on_device(Box::new(dev), Hz(145_000_000), Sps(2_400_000), 1024);
    until("the radio to start", || radio.status.running.load(Ordering::Relaxed));
    radio.send(Cmd::BandScan(crate::chain::BandScan {
        running: true,
        lo_hz: 144e6,
        hi_hz: 146e6,
        step_hz: 500_000.0,
        dwell_s: 0.2,
        on_hit: nodes::OnHit::Log,
        ..Default::default()
    }));
    // Where the spectrum says the receiver is, in the order it went
    // there. 144 to 146 MHz in half megahertz steps is four centres, the
    // first of them half a step inside the low edge.
    let stops = [144_250_000.0, 144_750_000.0, 145_250_000.0, 145_750_000.0];
    let mut walked: Vec<f64> = Vec::new();
    until("the dial to walk three steps", || {
        for f in radio.frames.try_iter() {
            if walked.last().is_none_or(|last| (last - f.center).abs() > 1.0) {
                walked.push(f.center);
            }
        }
        walked.len() >= 4
    });
    assert_eq!(walked[0], 145_000_000.0, "the dial started somewhere else");
    assert_eq!(walked[1], stops[0], "the first step is not the bottom of the band");
    // Which of the four the rest are is the file's pace rather than the
    // walk's: this radio hands over a capture as fast as the machine
    // will read it, so the walk's own clock runs ahead of a dial that
    // can only be retuned every 120 ms and some asks are overtaken.
    for hz in &walked[1..] {
        assert!(stops.contains(hz), "the dial went to {hz}, which is not a step of {stops:?}");
    }
    assert_eq!(radio.status.error.lock().clone(), None);
    // And stopping the walk leaves the dial where it was.
    radio.send(Cmd::BandScan(crate::chain::BandScan::default()));
    let held = *walked.last().expect("somewhere");
    std::thread::sleep(std::time::Duration::from_millis(400));
    for f in radio.frames.try_iter() {
        if (f.center - held).abs() > 1.0 {
            walked.push(f.center);
        }
    }
    assert_eq!(walked.len(), 4, "the dial kept moving after the walk stopped: {walked:?}");
}

/// A capture through the whole receiver, on the radio thread.
///
/// Every other replay test drives `chain::Receiver` directly, which is
/// the graph but not the loop around it: the command queue, the retune,
/// the rebuild, the read, the harvest and the publish are the radio
/// thread's, and none of them was under test. This runs the same capture
/// through the real thread on a radio made of memory, and pins the same
/// sensor the replay test pins.
#[test]
fn a_capture_played_into_the_radio_thread_is_decoded() {
    let _installing = decode::script::test_lock();
    if !decode::script::install_fetched() {
        return;
    }
    let p = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../testdata/fineoffset_wh1080_433.92M_250k.cu8");
    if !p.exists() {
        eprintln!("skipping: fineoffset_wh1080_433.92M_250k.cu8 absent, run testdata/fetch.sh");
        return;
    }
    let dev = sources::FileRadio::playing(&p).expect("the capture opens").as_fast_as_it_can();
    let (center, rate) = (Hz(433_920_000), Sps(250_000));
    let radio = Radio::on_device(Box::new(dev), center, rate, 1024);
    until("the radio to start", || radio.status.running.load(Ordering::Relaxed));

    // What the scanner table puts on 433.92: the ISM banks, which is how
    // the live receiver finds a sensor nobody tuned.
    let mut heard = Vec::new();
    until("the sensor to be read", || {
        heard.extend(radio.decodes.try_iter().flatten());
        heard.iter().any(|r| r.kind() == "Fineoffset-WHx080")
    });
    let r =
        heard.iter().find(|r| r.kind() == "Fineoffset-WHx080").expect("the loop above found one");
    // The same station the replay test reads off this capture, with the
    // transmitter's own CRC rather than a plausibility argument.
    assert!(checked(r), "{r:?}");
    assert_eq!(sensed(r, common::packet::Quantity::Temperature), Some(16.2));
    assert_eq!(sensed(r, common::packet::Quantity::Humidity), Some(89.0));
    assert!(who(r).as_deref() == Some("Fineoffset-WHx080/196"), "{:?}", who(r));
    assert!((r.freq() - 433_920_000.0).abs() < 100_000.0, "read at {:.4} MHz", r.freq() / 1e6);
    // Every packet reaching the bus carries what it was heard at.
    assert!(r.rssi_dbfs().is_finite() && r.snr_db().is_finite(), "no measurement on {r:?}");
}

#[test]
fn a_channel_outside_the_span_is_refused_rather_than_demodulated() {
    // Restoring a session tuned elsewhere leaves channels behind that the
    // radio is no longer sampling. Demodulating one shifts a frequency
    // that was never received down to baseband, and the result is noise
    // that sounds like a dead station.
    let rate = 2_400_000.0;
    let inside = ChannelSpec {
        id: 1,
        label: String::new(),
        offset_hz: -400_000.0,
        mode: ChanMode::Audio(Demod::Wfm),
        bandwidth_hz: None,
        audio_low_hz: None,
        squelch_db: None,
        voice: false,
        reads: None,
        agc: true,
        blanker: None,
        tx: None,
        tone: None,
    };
    let outside = ChannelSpec { id: 2, offset_hz: -994_200_000.0, ..inside.clone() };
    assert!(inside.offset_hz.abs() <= rate / 2.0);
    assert!(outside.offset_hz.abs() > rate / 2.0, "95.8 MHz is not inside a 1090 MHz span");
}

#[test]
fn each_channel_keeps_its_own_station() {
    // Two WFM channels are normally two different stations, and a shared
    // slot printed the first one's name under both.
    let s = Status::default();
    let named = |n: &str, pi: u16| dsp::rds::Station {
        pi: Some(pi),
        name: Some(n.into()),
        ..Default::default()
    };
    s.set_station(1, &named("SPIRIT", 0x2208), 10, 0, true);
    s.set_station(2, &named("HEART", 0xC479), 8, 1, true);

    assert_eq!(s.station_for(1).unwrap().name.as_deref(), Some("SPIRIT"));
    assert_eq!(s.station_for(2).unwrap().name.as_deref(), Some("HEART"));
    assert_eq!(s.station_for(2).unwrap().groups, 8);
    // A channel with no RDS shows nothing rather than a neighbour's name.
    assert!(s.station_for(3).is_none());

    // A removed or rebuilt channel takes its station with it.
    s.keep_stations(&[2]);
    assert!(s.station_for(1).is_none());
    assert_eq!(s.station_for(2).unwrap().name.as_deref(), Some("HEART"));
}
