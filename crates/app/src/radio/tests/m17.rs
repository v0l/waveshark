use super::*;

/// The M17 capture: three seconds of a busy 433 MHz band with an
/// OpenRTX handheld on the calling channel, recorded 550 kHz off centre.
fn m17_fixture() -> Option<common::IqBuf> {
    let p = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../testdata/m17_openrtx_434.02M_2400k.cu8");
    if !p.exists() {
        return None;
    }
    sources::FileSource::open(&p).ok()?.read_all().ok()
}

#[test]
fn a_decode_channel_reads_its_frequency_with_the_scanner_switched_off() {
    // The point of a decode channel: one front end at a fixed centre and
    // width, and nothing else running. Before this the only way to read
    // one frequency was a scanner block, which searched the span it
    // covered whether or not anything else in it was wanted.
    let Some(buf) = m17_fixture() else {
        eprintln!("skipping: fixture absent, run testdata/fetch.sh");
        return;
    };
    let mut plan = replay_plan(&buf, false);
    plan.fronts.clear();
    plan.channels = vec![ChannelSpec {
        id: 1,
        label: "M17".into(),
        offset_hz: 433_475_000.0 - buf.center.as_f64(),
        mode: ChanMode::Decode("m17".into()),
        bandwidth_hz: None,
        audio_low_hz: None,
        squelch_db: None,
        voice: false,
        reads: None,
        agc: true,
        tx: None,
        tone: None,
    }];
    let mut rx = crate::chain::Receiver::build(&plan, Default::default()).expect("a channel");
    let out = replay_blocks(&mut rx, &buf);

    let m17 = read_by(&out, "m17");
    assert!(!m17.is_empty(), "nothing read as M17 from {} rows", out.len());
    assert!(
        m17.iter().any(|r| {
            r.packet
                .innermost()
                .and_then(|l| l.link.from.as_ref())
                .is_some_and(|p| p.label() == "OPNRTX")
        }),
        "no callsign: {:?}",
        m17.iter().map(|r| r.detail()).take(4).collect::<Vec<_>>()
    );
    // The frequency the channel was set to, not one anything searched
    // for: a decode channel is told where to listen.
    let hz = m17[0].freq();
    assert!((hz - 433_475_000.0).abs() < 1.0, "read at {hz} Hz");
    let decoding = rx.decoding();
    assert_eq!(decoding.len(), 1, "one decode channel, one reading of it");
    let (id, strip) = &decoding[0];
    assert_eq!(*id, 1);
    assert_eq!(strip.acquisition, None, "M17 arrives in bursts and has no lock to report");
    assert_eq!(
        (strip.heard, m17.len()),
        (53, 53),
        "frames the strip says the channel heard, and M17 rows the receiver read"
    );
    // Placed by the strip, so nothing above it measures anything: the
    // auto node's fill is not in this path at all, and a row still has
    // its level, its ratio to the floor and the samples behind it.
    every_row_carries_its_measurements(&m17);
}

#[test]
fn an_auto_channel_finds_and_reads_what_is_in_its_own_bandwidth() {
    // An auto channel is the scanner table's front end pointed by hand:
    // no block covers this capture's frequency, nothing was told what the
    // signal is, and the width searched is the one set on the strip.
    let Some(buf) = m17_fixture() else {
        eprintln!("skipping: fixture absent, run testdata/fetch.sh");
        return;
    };
    let mut plan = replay_plan(&buf, false);
    plan.fronts.clear();
    plan.channels = vec![ChannelSpec {
        id: 1,
        label: "Watch".into(),
        offset_hz: 433_475_000.0 - buf.center.as_f64(),
        mode: ChanMode::Auto,
        bandwidth_hz: Some(100_000.0),
        audio_low_hz: None,
        squelch_db: None,
        voice: false,
        reads: None,
        agc: true,
        tx: None,
        tone: None,
    }];
    let mut rx = crate::chain::Receiver::build(&plan, Default::default()).expect("a channel");
    let out = replay_blocks(&mut rx, &buf);

    let m17 = read_by(&out, "m17");
    assert!(!m17.is_empty(), "nothing read as M17 from {} rows", out.len());
    assert!(
        m17.iter().any(|r| {
            r.packet
                .innermost()
                .and_then(|l| l.link.from.as_ref())
                .is_some_and(|p| p.label() == "OPNRTX")
        }),
        "no callsign: {:?}",
        m17.iter().map(|r| r.detail()).take(4).collect::<Vec<_>>()
    );
    // The detector's own answer, in absolute frequency: a source found
    // inside the channel is reported where it is on the dial.
    let hz = m17[0].freq();
    assert!((hz - 433_475_000.0).abs() < 25_000.0, "read at {hz} Hz");
}

#[test]
fn a_transmission_the_receiver_found_itself_is_audible() {
    // Decoding a call and being able to hear it are two different
    // things, and for a while this receiver did the first without the
    // second: live speech was read from the one M17 stage the scanner
    // table places, so every transmission the auto node found for itself
    // played back as silence. Speech is a port now and the call bus is a
    // node on the end of it, so this is the whole path from the air to
    // the mixer.
    let Some(buf) = m17_fixture() else {
        eprintln!("skipping: fixture absent, run testdata/fetch.sh");
        return;
    };
    let mut rx = replay_receiver(&buf, None).unwrap();
    let calls = rx.calls_mut().expect("the calls are always there");
    calls.set_subscriptions(vec![crate::mix::calls::Subscription::new(
        crate::mix::calls::Rule::Everything,
    )]);

    let mut pcm: Vec<f32> = Vec::new();
    let mut heard = None;
    for block in buf.samples.chunks(16_384) {
        if rx.process(block).is_err() {
            break;
        }
        // One side of the stereo mix, which carries speech on both.
        pcm.extend(rx.audio_out().0.iter().step_by(2));
        heard = heard.or_else(|| rx.audio().and_then(|b| b.last_heard()).map(str::to_string));
    }
    assert_eq!(heard.as_deref(), Some("OPNRTX to BROADCAST"), "nobody was heard");
    // The bus resamples to its output rate, and the over is seconds
    // long. Half a second of it is enough to say the vocoder ran on live
    // frames and the mix reached the far end.
    let seconds = pcm.len() as f64 / rx.audio().unwrap().out_rate();
    assert!(seconds > 0.5, "only {seconds:.2} s of speech");
    // Speech, not a run of zeros: a decoder that returns silence for
    // every frame would pass every assertion above.
    let rms = (pcm.iter().map(|v| v * v).sum::<f32>() / pcm.len() as f32).sqrt();
    assert!(rms > 1e-3, "the mix is silent at {rms:e} rms");
}
