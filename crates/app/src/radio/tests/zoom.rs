use super::*;

/// A tone that is a fraction of a channel wide on the full span.
fn tone(rate: f64, hz: f64, n: usize) -> Vec<C32> {
    (0..n)
        .map(|i| {
            let p = std::f64::consts::TAU * hz * i as f64 / rate;
            C32::new(0.5 * p.cos() as f32, 0.5 * p.sin() as f32)
        })
        .collect()
}

#[test]
fn narrowing_the_span_puts_more_bins_across_a_channel() {
    // The reason this exists: on a HackRF's narrowest span a 12.5 kHz
    // channel is a fraction of a pixel, and no cursor can be placed on it.
    let native = 2_000_000.0;
    let fft = 2048;
    for zoom in [1usize, 32] {
        let rate = native / zoom as f64;
        let bins_per_channel = 12_500.0 / (rate / fft as f64);
        if zoom == 1 {
            assert!(bins_per_channel < 15.0, "{bins_per_channel:.1} bins already");
        } else {
            assert!(
                bins_per_channel > 400.0,
                "only {bins_per_channel:.0} bins across a channel at /{zoom}"
            );
        }
    }
}

/// The receiver zoomed in, with nothing else attached, so what is being
/// measured is the narrowing and not the rest of the chain.
fn zoomed(native: f64, zoom: usize) -> crate::chain::Receiver {
    let mut plan = plan_at(native, Hz::mhz(433));
    plan.zoom = zoom;
    plan.fronts.clear();
    crate::chain::Receiver::build(&plan, Default::default()).expect("a zoom chain")
}

/// How long one chain takes to eat a second of signal.
fn seconds_to_process(rx: &mut crate::chain::Receiver, sig: &[C32]) -> f64 {
    rx.process(&sig[..1024]).unwrap();
    let t = common::time::Instant::now();
    rx.process(sig).unwrap();
    t.elapsed().as_secs_f64()
}

// Timing, so it needs optimisation to mean anything.
#[test]
#[cfg_attr(debug_assertions, ignore = "timing test, run with --release")]
fn narrowing_costs_little_enough_to_run_alongside_everything_else() {
    // It runs at the head of the graph, ahead of the spectrum, the banks
    // and the audio, so anything near real time here stalls all three.
    //
    // Measured against the same span with no narrowing in it rather than
    // against the clock. An absolute figure says as much about the machine
    // as about the code: this desktop runs it at ten times real time and a
    // shared CI runner managed two, while running the other two hundred
    // tests on the same cores, so a threshold in real time has to be set
    // so low it would miss a regression. A ratio survives that, because
    // whatever slows one side slows the other.
    //
    // Narrowing is not free and is not meant to be: the decimator costs
    // its tap count per output sample, where the chain it is compared
    // against is one FFT. That comes out at 6.6 to 6.8 across every zoom
    // factor here, so the bar is twelve.
    let native = 2_400_000.0;
    let sig = tone(native, 30_000.0, 2_400_000);
    let flat = seconds_to_process(&mut zoomed(native, 1), &sig);
    for zoom in [2usize, 8, 32] {
        let took = seconds_to_process(&mut zoomed(native, zoom), &sig);
        let ratio = took / flat.max(1e-9);
        eprintln!(
            "zoom /{zoom}: {:.0}x real time, {ratio:.2} times the unzoomed chain",
            1.0 / took
        );
        assert!(ratio < 12.0, "narrowing by {zoom} costs {ratio:.1} times the chain without it");
        // And a floor against the catastrophic case, loose enough that no
        // runner can trip it on contention alone.
        assert!(took < 1.0, "narrowing by {zoom} took {took:.2} s for one second of signal");
    }
}

#[test]
fn what_survives_the_narrowing_is_what_was_inside_it() {
    // Decimating without filtering folds the rest of the span on top of
    // what is left, and a folded signal cannot be told from a real one.
    let native = 2_000_000.0;
    let zoom = 8;
    let keep = native / zoom as f64 / 2.0;
    let mut plan = plan_at(native, Hz::mhz(433));
    plan.zoom = zoom;
    plan.fronts.clear();
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
    let mut rx = crate::chain::Receiver::build(&plan, Default::default()).expect("a zoom chain");
    // A signal well outside the narrowed span, which must not appear.
    rx.process(&tone(native, keep * 4.0, 262_144)).unwrap();
    let out = rx.zoomed_samples();
    let tail = &out[out.len() / 2..];
    let leaked = tail.iter().map(|c| c.norm()).fold(0.0f32, f32::max);
    let db = 20.0 * leaked.max(1e-12).log10();
    assert!(db < -60.0, "a signal outside the span folded in at {db:.1} dBFS");
}

/// Every capture in the corpus, through the whole receiver, at twice the
/// speed it was recorded at, on four threads.
///
/// The dashboard's speed trace is this number live, and a block that
/// takes longer than the samples in it is a block the radio drops. So the
/// rule is per block and not a mean: one slow block in a hundred is a lag
/// spike, and a mean of 20x hides it. The first blocks of a capture
/// allocate, fault pages in and open whatever the detector finds, so they
/// are run and not judged, and a short capture is repeated until enough
/// blocks have been timed to say anything.
///
/// Twice, because the machine this is measured on is not the machine it
/// runs on: a laptop's core is about half as fast, and 1x here is a
/// receiver that drops samples there. Four threads for the same reason,
/// and because measured on 48 the pool made nothing faster: the work in
/// a block is serial, so what a laptop lacks in cores it does not miss.
///
/// Blocks are the size a HackRF delivers, which is the worst case: a
/// bigger block is more work between two reads of the clock.
#[test]
#[cfg_attr(debug_assertions, ignore = "timing test, run with --release")]
fn every_capture_runs_faster_than_real_time() {
    let pool = rayon::ThreadPoolBuilder::new().num_threads(4).build().expect("a pool");
    pool.install(every_capture_runs_at_twice_real_time);
}

fn every_capture_runs_at_twice_real_time() {
    const BLOCK: usize = 131_072;
    const WARM: usize = 4;
    const TIMED: usize = 32;
    /// Blocks slower than this, in multiples of real time, fail.
    const FLOOR_X: f64 = 2.0;
    let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../testdata");
    let mut files: Vec<std::path::PathBuf> = ["", "offair", "rtl433"]
        .iter()
        .filter_map(|d| std::fs::read_dir(root.join(d)).ok())
        .flatten()
        .flatten()
        .map(|e| e.path())
        .filter(|p| {
            matches!(p.extension().and_then(|e| e.to_str()), Some("cu8" | "cs8" | "cs16" | "cf32"))
        })
        .collect();
    files.sort();
    if files.is_empty() {
        eprintln!("skipping: no captures in testdata, run testdata/fetch.sh");
        return;
    }
    // Captures known not to keep up, each with the reason, so the test
    // stays a gate for everything else while the reason is worked on.
    // One that starts keeping up fails the test until it is taken off
    // the list, so the list cannot outlive its reasons.
    //
    // Every reason here was measured with `--bench-iq`, which prints the
    // auto node's phases; none of them is the span-wide burst router any
    // more, which is what they all used to say.
    //
    // What they mostly say now is the classifier. It costs about 1.3 ms
    // of one core per burst, measured on a 30000 sample burst at
    // 2.5 MS/s, and that is spread over a level histogram, four
    // Welch-averaged transforms and two autocorrelations, none of which
    // is more than a quarter of it; halving the window it measures takes
    // the corpus from 46 of 52 captures named right to 42, so the cost is
    // the measurement rather than an overhead around it. A band where
    // several megahertz-wide sources burst at once therefore asks for
    // more classification than a block has time for, and that is a
    // throughput problem rather than a spike.
    const KNOWN_SLOW: &[(&str, &str)] = &[
        (
            "ism24_busy_2431M_61440k.cs16",
            "a whole 2.4 GHz band at 61.44 MS/s, which is five things at once, measured as \
                 processor time in a 2.13 ms block: the Wi-Fi front end on the channels it is \
                 on, 2.0 ms; the ZigBee front end on twelve, 1.1; the detector over 61 MS/s, \
                 1.1, of which half is eight 32768 point transforms and a third the floor \
                 pass; the BLE front end on two advertising channels, 0.7; the DroneID \
                 correlator on the three centres the span holds, 0.6; and cutting the \
                 fifty-three sources the band opens out of the span, 0.9. Seven milliseconds \
                 of processor time for a two millisecond block, of which the front ends now \
                 read one block behind so they overlap the detector, which took the median \
                 from 0.5x to 0.8x. What is left is each front end mixing and filtering the \
                 whole span for itself, two dozen times over, where one channel bank would \
                 pay the input rate once. No spikes: the worst block is 27 ms where it was \
                 127",
        ),
        (
            "droneid_mini4k_2444.5M_15360k.cs8",
            "the DroneID correlator over the 10 MHz centre the aircraft is on, 1.8 ms of an \
                 8.5 ms block, and the classifier behind the source the burst opens, 1.9. The \
                 correlator runs whenever anything is transmitting in that channel, which on \
                 this capture is the aircraft's own link in nearly every block; what it buys is \
                 the seven bursts, which the receiver did not read at all while the front end \
                 was placed on a source instead of on the span",
        ),
        (
            "odid_bt5lr_holybro_2474M_20000k.cs8",
            "classifying the coded-PHY bursts of several megahertz-wide sources at once, \
                 5 ms of a 6.5 ms block on average and 14 in the worst. They are auxiliary \
                 advertising on data channels, which the BLE front end does not read, so no \
                 front end has already decoded what is being measured",
        ),
        (
            "odid_holybro_2431M_20000k.cs8",
            "the same, with the chirp decoders behind it: the classifier names an 855 kHz \
                 Bluetooth burst a chirp at full confidence, and LoRa and ExpressLRS are each \
                 placed on that for 2.9 ms a block",
        ),
        (
            "offair/elrs_100hz_2415M_20000k.cs8",
            "classifying a handset's channel visit, 44 ms and 1.4 MHz wide, which is one \
                 burst that costs three blocks, beside the chirp decoder it places",
        ),
        (
            "offair/ofdm_wifi_2462M_20000k.cs8",
            "the Wi-Fi front end reading the channel it is on, 4 ms of a 6.5 ms block, \
                 beside 1.8 ms cutting the sources the same signal opens out of the span",
        ),
        (
            "offair/ofdm_wifi_frames_2462M_20000k.cs8",
            "the same, and DroneID correlating over the 2459.5 MHz centre the span holds, \
                 which the Wi-Fi traffic lights in every block",
        ),
        (
            "offair/gfsk_ble_2426M_20000k.cs8",
            "the closest of them: about 2.7x on four threads with the BLE front end and \
                 the detector at 1.0 and 0.9 ms each of 6.5, and one block in ten at half that \
                 speed. The span holds a DroneID centre too, and the advertising channel sits \
                 inside it, so every advertisement lights that correlator as well",
        ),
        (
            "dvbs2_two_carriers_1083M_61440k.cs8",
            "no DVB-S2 front end is placed here; the span holds 1090 MHz and the 1065 MHz \
                 screen clock. By --bench-iq, of a 2.13 ms block: Mode S 0.41 ms at 3.84 MS/s, \
                 about 50 ns a sample as on any ADS-B capture, behind its mixer and /16 at 0.30; \
                 the spectrum 0.44; and the screen decoder's mixer and /4 at 0.43, for a decoder \
                 that itself costs 0.02 on its own thread",
        ),
        (
            "dvbs2_bbc_hd_1097M_40000k.cs8",
            "the same two front ends at 40 MS/s, 3.28 ms a block: Mode S 0.42 ms behind its \
                 mixer and /16 at 0.60, the spectrum 0.45, and the screen decoder's mixer and /2 \
                 to 1080 MHz at 0.47. About 2.5x at the median, with one block in three under 2x \
                 on four threads",
        ),
        (
            "zigbee_join_ch11_2405M_8000k.cs8",
            "four front ends over one 8 MS/s channel, 8 ms of a 16.4 ms block at the \
                 median. By perf, BLE on advertising channel 37 and 802.15.4 take 7% of the \
                 samples each, the burst router 3% and nRF24 2%; the bench's own per front \
                 end figures are wall time inside rayon tasks and count work stolen while a \
                 task waits, which once made the router look like the worst of them",
        ),
        (
            "pal_camera_5865M_20000k.cs8",
            "the video front end itself: 3.8 ms of every 6.5 ms block, which is two FM \
                 discriminators, one at 10 MS/s for the picture and one at 20 for the sound \
                 subcarrier, and the field assembly between them",
        ),
    ];
    let mut slow: Vec<String> = Vec::new();
    let mut recovered: Vec<String> = Vec::new();
    for path in &files {
        let name = path.strip_prefix(&root).unwrap_or(path).display().to_string();
        let known = KNOWN_SLOW.iter().find(|(n, _)| *n == name).map(|(_, why)| *why);
        let buf = match sources::FileSource::open(path).and_then(|s| s.read_all()) {
            Ok(b) if b.samples.len() >= BLOCK => b,
            Ok(_) => {
                eprintln!("{name}: shorter than one block, not timed");
                continue;
            }
            Err(e) => panic!("{name}: {e}"),
        };
        let rate = buf.rate.as_f64().max(1.0);
        let block_secs = BLOCK as f64 / rate;
        let mut rx = replay_receiver(&buf, None).expect(&name);
        let mut us: Vec<f64> = Vec::new();
        'passes: for _ in 0..64 {
            for chunk in buf.samples.chunks(BLOCK) {
                if chunk.len() < BLOCK {
                    break;
                }
                let t = common::time::Instant::now();
                rx.process(chunk).expect(&name);
                us.push(t.elapsed().as_secs_f64() * 1e6);
                let at = block_start(common::time::Instant::now(), chunk.len(), rate);
                let _ = harvest(&mut rx, at);
                if us.len() >= WARM + TIMED {
                    break 'passes;
                }
            }
        }
        let timed = &us[WARM.min(us.len().saturating_sub(1))..];
        let worst = timed.iter().copied().fold(0.0f64, f64::max);
        let mut sorted = timed.to_vec();
        sorted.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let median = sorted[sorted.len() / 2];
        let x = |us: f64| block_secs * 1e6 / us.max(1e-9);
        let over = timed.iter().filter(|&&b| b * FLOOR_X > block_secs * 1e6).count();
        eprintln!(
            "{name}: {} blocks, median {:.1}x, worst {:.2}x{}",
            timed.len(),
            x(median),
            x(worst),
            if over > 0 { format!(", {over} under {FLOOR_X}x") } else { String::new() },
        );
        // A listed capture comes off the list once it clears the floor
        // with room to spare, not the first time it lands over it: one
        // that sits at the floor would otherwise fail one run in two.
        let clear = x(worst) >= FLOOR_X * 1.5;
        match (over > 0, known) {
            (true, None) => {
                slow.push(format!("{name}: worst block {:.2}x, floor {FLOOR_X}x", x(worst)))
            }
            (true, Some(why)) => eprintln!("{name}: known slow, {why}"),
            (false, Some(_)) if clear => recovered.push(name.clone()),
            (false, _) => {}
        }
    }
    assert!(slow.is_empty(), "captures the receiver cannot keep up with:\n{}", slow.join("\n"));
    assert!(
        recovered.is_empty(),
        "captures that keep up now and should come off KNOWN_SLOW:\n{}",
        recovered.join("\n")
    );
}

struct ScreenRun {
    speeds: Vec<f64>,
    readings: Vec<(String, String)>,
}

fn screen_at_744(mode: Option<&str>) -> Option<ScreenRun> {
    const NAME: &str = "screen_744M_20000k.cs8";
    const BLOCK: usize = 131_072;
    const PACE_X: f64 = 2.0;
    let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../testdata").join(NAME);
    if !path.exists() {
        eprintln!("skipping: {NAME} absent, run testdata/fetch.sh");
        return None;
    }
    let buf = sources::FileSource::open(&path).and_then(|s| s.read_all()).expect(NAME);
    let rate = buf.rate.as_f64();
    let block_secs = BLOCK as f64 / rate;
    let mut plan = replay_plan(&buf, false);
    plan.fronts = crate::scanners::Scanners::default()
        .fronts(crate::scanners::Span::new(buf.center.as_f64(), rate));
    let mut rx = crate::chain::Receiver::build(&plan, Default::default()).expect(NAME);
    let screen = rx
        .topology()
        .nodes
        .iter()
        .find(|n| n.kind == "tempest")
        .map(|n| n.id.0)
        .expect("the scanner table puts a screen decoder on the 742.5 MHz cable clock");
    if let Some(m) = mode {
        rx.set_node_param(screen, "mode", pipeline::ParamValue::Text(m.into()))
            .expect("a mode the table knows");
    }
    let mut speeds = Vec::new();
    let started = common::time::Instant::now();
    for (i, chunk) in buf.samples.chunks_exact(BLOCK).enumerate() {
        let due = started + common::time::Duration::from_secs_f64(i as f64 * block_secs / PACE_X);
        std::thread::sleep(due.saturating_duration_since(common::time::Instant::now()));
        let t = common::time::Instant::now();
        rx.process(chunk).expect(NAME);
        speeds.push(block_secs / t.elapsed().as_secs_f64().max(1e-9));
        let _ = harvest(&mut rx, block_start(common::time::Instant::now(), chunk.len(), rate));
        if started.elapsed().as_secs_f64() > 20.0 {
            break;
        }
    }
    let readings = rx
        .topology()
        .nodes
        .into_iter()
        .find(|n| n.id.0 == screen)
        .map(|n| n.readings)
        .unwrap_or_default();
    Some(ScreenRun { speeds, readings })
}

fn lows(run: &ScreenRun, floor_x: f64) -> Vec<(usize, f64)> {
    eprintln!(
        "{} blocks, slowest {:.2}x real time",
        run.speeds.len(),
        run.speeds.iter().copied().fold(f64::INFINITY, f64::min)
    );
    run.speeds.iter().copied().enumerate().filter(|(_, x)| *x < floor_x).collect()
}

#[test]
#[cfg_attr(debug_assertions, ignore = "timing test, run with --release")]
fn the_screen_search_at_744_mhz_keeps_every_block_over_twice_real_time() {
    const FLOOR_X: f64 = 2.0;
    let Some(run) = screen_at_744(None) else { return };
    assert_eq!(run.speeds.len(), 183, "blocks of 131072 in 1.2 s at 20 MS/s");
    assert_eq!(lows(&run, FLOOR_X), [], "(block, speed) under {FLOOR_X}x");
    assert_eq!(run.readings, [], "auto locked on the band");
}

#[test]
#[cfg_attr(debug_assertions, ignore = "timing test, run with --release")]
fn a_screen_named_1080p60_at_744_mhz_keeps_every_block_over_twice_real_time() {
    const FLOOR_X: f64 = 2.0;
    let Some(run) = screen_at_744(Some("1920x1080 60 Hz")) else { return };
    assert_eq!(run.speeds.len(), 183, "blocks of 131072 in 1.2 s at 20 MS/s");
    assert_eq!(lows(&run, FLOOR_X), [], "(block, speed) under {FLOOR_X}x");
    let held = run.readings.iter().find(|(k, _)| k == "held").map(|(_, v)| v.as_str());
    assert_eq!(
        held,
        Some("47 of 63 frames"),
        "frames on the fifth harmonic, read {} s behind",
        nodes::tempest_nodes::LAG_S
    );
}
