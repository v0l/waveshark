use super::*;

/// The widest graph the receiver builds, as a replay at 20 MS/s decides it.
fn widest_chain() -> crate::chain::Receiver {
    let rate = 20_000_000.0;
    let mut plan = plan_at(rate, Hz::mhz(433));
    plan.fronts
        .extend(crate::scanners::Scanners::load().fronts(crate::scanners::Span::new(433e6, rate)));
    plan.channels = vec![ChannelSpec {
        id: 1,
        label: String::new(),
        offset_hz: 0.0,
        mode: ChanMode::Audio(Demod::Nfm),
        bandwidth_hz: None,
        audio_low_hz: None,
        squelch_db: Some(-200.0),
        agc: false,
        blanker: None,
        denoise: false,
        denoise_db: dsp::denoise::DEFAULT_DEPTH_DB,
        agc_tune: nodes::AgcTune::default(),
        notch: false,
        voice: false,
        reads: None,
        tx: None,
        tone: None,
    }];
    crate::chain::Receiver::build(&plan, Default::default()).expect("the widest chain")
}

#[test]
#[cfg_attr(debug_assertions, ignore = "timing test, run with --release")]
fn a_block_of_no_samples_costs_half_a_percent_of_a_block_that_has_some() {
    // A source with nothing to hand over returns a block of no samples
    // and the radio thread runs the whole graph on it, because the same
    // turn of the loop is what takes commands and retunes. This says what
    // that run costs, so that skipping it can be judged rather than
    // assumed.
    //
    // Measured on the widest graph the receiver builds, 44 nodes at
    // 20 MS/s: 28 to 31 us for an empty block against 6541 us for 262144
    // samples, which is under half a percent of it. Delivered at the 50 a
    // second a stalled source produces, the empty runs come to 0.15% of
    // one core, so the graph run stays where it is and the turn keeps its
    // one shape.
    let mut rx = widest_chain();
    let sig = block(262_144);
    rx.process(&sig).expect("a block of samples");

    let turns = 3_000;
    let t = std::time::Instant::now();
    for _ in 0..turns {
        rx.process(&[]).expect("a block of no samples");
        let _ = rx.spectrum_ready();
        let _ = rx.rows(std::time::Instant::now());
    }
    let empty = t.elapsed().as_secs_f64() / turns as f64;

    let runs = 20;
    let t = std::time::Instant::now();
    for _ in 0..runs {
        rx.process(&sig).expect("a block of samples");
    }
    let full = t.elapsed().as_secs_f64() / runs as f64;

    let share = empty / full.max(1e-9);
    eprintln!(
        "an empty turn is {:.1} us against {:.0} us for 262144 samples, {:.3}% of it, \
             and {:.3}% of a core at 50 a second",
        empty * 1e6,
        full * 1e6,
        share * 100.0,
        empty * 50.0 * 100.0
    );
    // A ratio rather than a time: a shared runner is slower on both
    // sides. The bar is twenty times the measured 0.47%, which no
    // contention reaches and a graph run that started doing real work on
    // nothing would blow through.
    assert!(
        share < 0.1,
        "an empty block costs {:.1}% of a block with samples in it",
        share * 100.0
    );
}

#[test]
fn a_minute_of_blocks_with_no_samples_decodes_nothing_and_keeps_the_graph() {
    // Three thousand empty blocks is a minute of a stalled source at the
    // 50 a second one produces.
    let mut rx = widest_chain();
    let mut rows = 0;
    for _ in 0..3_000 {
        rx.process(&[]).expect("a block of no samples");
        rows += rx.rows(std::time::Instant::now()).len();
    }
    assert_eq!(rows, 0, "{rows} rows came out of blocks with no samples in them");
    // And the chain still reads its own span afterwards rather than
    // having been walked into a state it cannot come back from.
    rx.process(&block(262_144)).expect("a block of samples after the empty ones");
}

#[test]
fn blocks_with_no_samples_between_the_real_ones_change_nothing_that_is_read() {
    let _installing = decode::script::test_lock();
    if !decode::script::install_fetched() {
        return;
    }
    let Some(buf) = fixture() else {
        eprintln!("skipping: fixture absent, run testdata/fetch.sh");
        return;
    };
    let straight = replay_blocks(&mut replay_receiver(&buf, None).unwrap(), &buf);
    let mut rx = replay_receiver(&buf, None).unwrap();
    let mut out = Vec::new();
    let rate = buf.rate.as_f64();
    for blk in buf.samples.chunks(16_384) {
        // Fifty of them, a second of stall between every block of the
        // capture.
        for _ in 0..50 {
            rx.process(&[]).expect("a block of no samples");
            out.extend(harvest(&mut rx, std::time::Instant::now()));
        }
        rx.process(blk).expect("a block of samples");
        let at = block_start(std::time::Instant::now(), blk.len(), rate);
        out.extend(harvest(&mut rx, at));
    }
    // The same one packet the capture gives up when it is replayed
    // straight through, with the same reading on it.
    assert_eq!(straight.len(), 1, "the capture stopped giving up its packet: {straight:#?}");
    assert_eq!(out.len(), 1, "the stalls changed what was read: {out:#?}");
    let (a, b) = (&straight[0], &out[0]);
    assert_eq!(b.kind(), a.kind());
    assert_eq!(b.modulation(), common::Modulation::Ook);
    assert_eq!(
        sensed(b, common::packet::Quantity::Temperature),
        sensed(a, common::packet::Quantity::Temperature)
    );
}

#[test]
fn a_decode_is_stamped_when_the_block_started_not_when_it_finished() {
    // Arithmetic rather than a race against the clock. This used to
    // compare the stamp against an instant taken before the whole replay,
    // which quietly asserted that decoding beat real time: every block
    // processed before the first decode had to fit inside one block's
    // worth of signal. On a loaded machine, or in a debug build, it failed
    // for a reason that had nothing to do with stamping, which is the
    // other test's job.
    let finished = std::time::Instant::now();
    let at = block_start(finished, 16_384, 250_000.0);
    // 16384 samples at 250 kS/s is 65.536 ms of signal.
    let back = finished.duration_since(at).as_secs_f64();
    assert!((back - 0.065_536).abs() < 1e-9, "stamped {back}s before the block ended");
    assert!(at < finished, "the stamp must precede the block it came from");
    // A rate of zero must not divide by it.
    assert!(block_start(finished, 16_384, 0.0) < finished);
}

/// And the same thing where it is actually used, which no arithmetic test
/// can check: a replayed decode must not be stamped in the future.
#[test]
fn a_replayed_decode_is_not_stamped_in_the_future() {
    let _installing = decode::script::test_lock();
    if !decode::script::install_fetched() {
        return;
    }
    let Some(buf) = fixture() else {
        eprintln!("skipping: fixture absent, run testdata/fetch.sh");
        return;
    };
    let mut rx = replay_receiver(&buf, None).unwrap();
    let out = replay_blocks(&mut rx, &buf);
    let done = std::time::Instant::now();
    let rec = out.first().expect("a decode");
    assert!(rec.at < done, "a decode is stamped after the replay that produced it");
}

#[test]
fn retuning_clears_state_rather_than_carrying_it_across() {
    // A burst half-collected at one frequency must not finish at another.
    let mut rx = replay_receiver(&empty_buf(250_000.0, Hz::mhz(433)), None).unwrap();
    rx.process(&block(8192)).unwrap();
    let mut plan = plan_at(250_000.0, Hz::mhz(868));
    plan.fronts = vec![crate::scanners::FrontAt {
        front: crate::scanners::Front::Banks(crate::scanners::DEFAULT_WIDTHS.to_vec()),
        band: (0.0, f64::INFINITY),
    }];
    rx.rebuild(&plan).unwrap();
    rx.process(&block(8192)).unwrap();
    let out = rx.rows(std::time::Instant::now());
    assert!(out.is_empty(), "a steady tone decoded as {out:?}");
}

#[test]
fn the_scanner_keeps_up_with_the_stream() {
    // Decoding the whole span is only worth having if it runs in real
    // time; if it does not, it is stealing from the thread that has to
    // drain USB and the radio drops samples instead.
    if cfg!(debug_assertions) {
        eprintln!("skipping: an unoptimised build says nothing about throughput");
        return;
    }
    // SCAN_RATE=16000000 asks the same question of a wideband span.
    let rate = std::env::var("SCAN_RATE").ok().and_then(|v| v.parse().ok()).unwrap_or(2_400_000.0);
    let mut rx = replay_receiver(&empty_buf(rate, Hz::mhz(868)), None).unwrap();
    let b = block(262_144);
    // One pass to warm the filters and the pool.
    rx.process(&b).unwrap();

    let t = std::time::Instant::now();
    let blocks = 20;
    for _ in 0..blocks {
        rx.process(&b).unwrap();
    }
    let secs = t.elapsed().as_secs_f64();
    let audio_secs = blocks as f64 * b.len() as f64 / rate;
    let x = audio_secs / secs;
    eprintln!("scanner: {x:.1}x real time watching for sources");
    assert!(x > 1.0, "the scanner ran at only {x:.2}x real time");
}

#[test]
fn scratch_buffers_do_not_grow_across_blocks() {
    // Every stage appends to its output. If one is not cleared it grows
    // without bound and each block re-filters the whole history, which
    // looks like the radio slowly seizing up rather than an obvious fault.
    // Measured by time rather than by reaching into buffers, which the
    // chain no longer exposes now it is a graph. A buffer that is never
    // cleared refilters its whole history, so the cost per block climbs;
    // that is the symptom either way.
    let mut a = Audio::new(120_000.0, 2_304_000.0, Demod::Wfm, 48_000.0);
    let b = block(8192);
    // The quickest of three passes, because a shared machine's jitter only
    // ever adds time: a loaded CI runner read 3.3x off single passes of a
    // chain that was not growing at all.
    let cost = |a: &mut Audio| {
        (0..3)
            .map(|_| {
                let t = std::time::Instant::now();
                for _ in 0..10 {
                    a.process(&b, 0.5);
                }
                t.elapsed().as_secs_f64()
            })
            .fold(f64::MAX, f64::min)
    };
    let first = cost(&mut a);
    for _ in 0..5 {
        cost(&mut a);
    }
    let later = cost(&mut a);
    assert!(later < first * 3.0, "cost per block climbed from {first:.4}s to {later:.4}s");
    assert!(a.pcm.len() <= b.len(), "audio output grew");
}

#[test]
fn output_length_is_steady_block_to_block() {
    let mut a = Audio::new(0.0, 2_304_000.0, Demod::Nfm, 48_000.0);
    let b = block(4800);
    let first = a.process(&b, 0.5).len();
    for _ in 0..10 {
        let n = a.process(&b, 0.5).len();
        assert!((n as i64 - first as i64).abs() <= 1, "block produced {n} samples after {first}");
    }
}

#[test]
fn every_mode_runs_faster_than_real_time() {
    // The audio chain shares the radio thread with USB draining, so
    // anything near 1x drops samples.
    //
    // The bound is deliberately far below what any developer machine
    // manages, because it has to hold on the slowest shared CI runner too:
    // this one reads 6.4x here and 3.0x on a two core VM. It is a guard
    // against a chain that has gone accidentally quadratic, not a
    // performance target. Real numbers come from --bench-audio.
    // Only meaningful in a release build. The workspace optimises
    // dependencies in dev but not the crate under test, so this code runs
    // unoptimised here and reads a fraction of real time no matter how
    // healthy the chain is. CI runs the suite with --release, which is
    // where the guard bites.
    if cfg!(debug_assertions) {
        eprintln!("skipping: throughput is only measurable in a release build");
        return;
    }
    let rate = 2_304_000.0;
    let b = block(131_072);
    for mode in [Demod::Wfm, Demod::Nfm, Demod::Am, Demod::Usb, Demod::Cw] {
        let mut a = Audio::new(120_000.0, rate, mode, 48_000.0);
        a.process(&b, 0.5);
        let t = std::time::Instant::now();
        for _ in 0..4 {
            a.process(&b, 0.5);
        }
        let x = (4.0 * b.len() as f64 / rate) / t.elapsed().as_secs_f64();
        assert!(x > 1.5, "{} only ran at {x:.1}x real time", mode.label());
    }
}

#[test]
fn the_chain_does_not_delay_the_audio_audibly() {
    // Driving the IF rate down to the channel bandwidth leaves no
    // transition band and asks for thousands of taps, which shows up as
    // delay. The graph adds up each filter's group delay, so this catches
    // it wherever in the chain it happens rather than at one filter that
    // was remembered to be checked.
    for mode in [Demod::Wfm, Demod::Nfm, Demod::Am, Demod::Usb, Demod::Cw] {
        let a = Audio::new(0.0, 2_304_000.0, mode, 48_000.0);
        let ms = a.latency_ms();
        assert!(ms < 40.0, "{} delays audio by {ms:.1} ms", mode.label());
    }
}

#[test]
fn the_chain_is_a_graph_with_every_stage_named() {
    // The point of building on the graph: the stages can be listed, which
    // is what the chain view draws.
    let a = Audio::new(0.0, 2_304_000.0, Demod::Wfm, 48_000.0);
    let topo = a.topology();
    let names: Vec<&str> = topo.nodes.iter().map(|n| n.label.as_str()).collect();
    assert!(names.contains(&"Mixer"), "{names:?}");
    assert!(names.contains(&"WFM demod"), "{names:?}");
    assert!(names.contains(&"High blend"), "{names:?}");
    assert!(names.iter().all(|n| !n.is_empty()));
}

#[test]
fn am_skips_de_emphasis_and_uses_an_envelope_detector() {
    let a = Audio::new(0.0, 2_304_000.0, Demod::Am, 48_000.0);
    let topo = a.topology();
    let names: Vec<&str> = topo.nodes.iter().map(|n| n.label.as_str()).collect();
    assert!(names.contains(&"AM envelope"), "{names:?}");
    assert!(!names.contains(&"De-emphasis"), "{names:?}");
}

#[test]
fn the_audio_rate_is_close_to_what_was_asked_for() {
    for mode in [Demod::Wfm, Demod::Nfm, Demod::Am, Demod::Usb, Demod::Cw] {
        let a = Audio::new(0.0, 2_304_000.0, mode, 48_000.0);
        let r = a.audio_rate();
        assert!((r - 48_000.0).abs() < 12_000.0, "{} gave {r} Hz", mode.label());
    }
}
