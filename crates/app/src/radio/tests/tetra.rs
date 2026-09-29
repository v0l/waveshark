use super::*;

fn tetra_fixture() -> Option<common::IqBuf> {
    let p = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../testdata/tetra_downlink_391.5M_2400k.cu8");
    if !p.exists() {
        return None;
    }
    sources::FileSource::open(&p).ok()?.read_all().ok()
}

/// Two TETRA base station downlinks, on for every one of the capture's
/// ten seconds, that the receiver never reports.
#[test]
fn a_permanent_tetra_downlink_is_found() {
    let Some(buf) = tetra_fixture() else {
        eprintln!("skipping: fixture absent, run testdata/fetch.sh");
        return;
    };
    let mut rx = replay_receiver(&buf, None).unwrap();
    let mut seen: Vec<(f64, Option<f32>)> = Vec::new();
    for block in buf.samples.chunks(16_384) {
        if rx.process(block).is_err() {
            break;
        }
        for s in rx.live_sources() {
            if !seen.iter().any(|(hz, _)| (hz - s.center_hz).abs() < 12_500.0) {
                seen.push((s.center_hz, s.snr_db));
            }
        }
    }
    let near = |hz: f64| seen.iter().any(|(c, _)| (c - hz).abs() < 12_500.0);
    assert!(near(391_181_000.0), "391.181 MHz was never opened: {seen:?}");
    assert!(near(391_704_500.0), "391.7045 MHz was never opened: {seen:?}");
}

/// The key manager reaches a front end the receiver built for itself.
///
/// Nothing here places a TETRA stage: the scanner table watches the band,
/// the auto node builds a front end on each carrier it finds, and the key
/// status is read by asking every node in the receiver whether it takes
/// keys. Read off a stage held by name, an encrypted network the receiver
/// found for itself could never be given one.
#[test]
fn the_key_manager_reaches_a_front_end_the_receiver_found_for_itself() {
    let Some(buf) = tetra_fixture() else {
        eprintln!("skipping: fixture absent, run testdata/fetch.sh");
        return;
    };
    let mut rx = replay_receiver(&buf, None).unwrap();
    replay_blocks(&mut rx, &buf);
    let keys = rx.tetra_key_status();
    // One row per cell heard, both on the same network, as the cells
    // themselves broadcast it: the control carrier at 391.175 MHz says
    // its traffic is enciphered (air interface encryption 3) and the
    // other carrier is in the clear.
    assert_eq!(keys.len(), 2, "{keys:?}");
    assert!(keys.iter().all(|k| (k.mcc, k.mnc) == (272, 6838)), "{keys:?}");
    let mut colours: Vec<u8> = keys.iter().map(|k| k.colour).collect();
    colours.sort();
    assert_eq!(colours, [3, 5], "{keys:?}");
    assert!(keys.iter().any(|k| k.aie == 3), "the encrypting cell was not seen: {keys:?}");
}

/// Finding the carriers is half of it. The scanner block promises that
/// each is measured and logged, so the list says which channels are
/// busy; a source that never closes never used to reach the list at all,
/// because a burst front end reports a burst when it ends.
#[test]
fn a_permanent_tetra_downlink_is_logged_with_its_measurement() {
    let Some(buf) = tetra_fixture() else {
        eprintln!("skipping: fixture absent, run testdata/fetch.sh");
        return;
    };
    // Replayed in two halves with the graph rebuilt between them, which
    // is what a retune or a setting does live: every source's decoders
    // are built again.
    let plan = replay_plan(&buf, false);
    let mut rx = crate::chain::Receiver::build(&plan, Default::default()).unwrap();
    let half = buf.samples.len() / 2;
    let first = common::IqBuf::new(buf.samples[..half].to_vec(), buf.center, buf.rate, 0);
    let second =
        common::IqBuf::new(buf.samples[half..].to_vec(), buf.center, buf.rate, half as u64);
    let mut out = replay_blocks(&mut rx, &first);
    rx.rebuild(&plan).unwrap();
    out.extend(replay_blocks(&mut rx, &second));
    let rows: Vec<String> = out
        .iter()
        .map(|r| {
            format!("{:.4} MHz {} {} {}", r.freq() / 1e6, r.protocol(), r.modulation(), r.detail())
        })
        .collect();
    for hz in [391_181_000.0, 391_704_500.0] {
        let mine: Vec<&Reception> =
            out.iter().filter(|r| (r.freq() - hz).abs() < 12_500.0).collect();
        assert!(!mine.is_empty(), "{:.4} MHz was never logged: {rows:?}", hz / 1e6);
        // Every row says what it was heard at: the front end measures the
        // slot each block came out of rather than handing over a frame
        // with nothing on it.
        every_row_carries_its_measurements(&mine);
        // Logged as the channel the plan lists, not as this tuner's
        // measurement of it: the band is on a 25 kHz raster and the
        // carrier was found a few kilohertz off it.
        let channel = (hz / 25_000.0).round() * 25_000.0;
        assert!(
            mine.iter().all(|r| (r.freq() - channel).abs() < 1.0),
            "{:.4} MHz was logged at {:?}",
            hz / 1e6,
            mine.iter().map(|r| r.freq()).collect::<Vec<_>>()
        );
        // The cell's identity once, and once only, though its decoders
        // were built twice.
        let sync = mine.iter().filter(|r| r.kind() == "sync").count();
        let sysinfo = mine.iter().filter(|r| r.kind() == "sysinfo").count();
        assert_eq!((sync, sysinfo), (1, 1), "{:.4} MHz: {rows:?}", hz / 1e6);
        assert!(
            mine.iter().any(|r| {
                r.packet.facts().any(|(_, f)| match f {
                    common::packet::Fact::Infrastructure(c) => c.mcc == Some(272),
                    _ => false,
                })
            }),
            "{:.4} MHz: the network was not named",
            hz / 1e6
        );
        // The cell's own map of its neighbours, read off the network
        // broadcast the control carrier sends: every list it cycles
        // through once, each cell on this network's band. The other
        // carrier is an idle traffic carrier and broadcasts nothing but
        // its identity.
        let network: Vec<&&Reception> = mine.iter().filter(|r| r.kind() == "network").collect();
        if hz == 391_181_000.0 {
            assert!(!network.is_empty(), "{:.4} MHz: no network broadcast: {rows:?}", hz / 1e6);
        }
        assert!(network.len() <= 8, "{:.4} MHz: {} network rows", hz / 1e6, network.len());
        // Each neighbour the cell named, with the carrier to go and
        // look for it on: inside this network's band, or the map is of
        // somewhere else.
        for r in &network {
            let cells: Vec<u64> = r
                .packet
                .facts()
                .filter_map(|(_, f)| match f {
                    common::packet::Fact::Infrastructure(c) => c.carrier_hz,
                    _ => None,
                })
                .collect();
            assert!(!cells.is_empty(), "no neighbour on {}", r.detail());
            for hz in cells {
                assert!((390_000_000..400_000_000).contains(&hz), "{hz} Hz");
            }
        }
        // A measurement of what the carrier looks like is not news once
        // a front end is reading it: at most the one piece cut before
        // the front end found its first sync burst.
        let measured: Vec<&&Reception> = mine.iter().filter(|r| !r.is_known()).collect();
        assert!(
            measured.len() <= 1,
            "{:.4} MHz measured {} times while being read: {rows:?}",
            hz / 1e6,
            measured.len()
        );
        assert!(
            measured.iter().all(
                |r| r.modulation() == common::Modulation::Dqpsk && r.detail().contains("TETRA")
            ),
            "{:.4} MHz was measured as {:?}",
            hz / 1e6,
            measured
                .iter()
                .map(|r| format!("{} {}", r.modulation(), r.detail()))
                .collect::<Vec<_>>()
        );
    }
}

#[test]
#[ignore]
fn a_call_four_tetra_sites_send_is_heard_once_and_listed_once() {
    let p = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../testdata/offair/tetra_band_392.8013M_6000k.cu8");
    if !p.exists() {
        eprintln!("skipping: testdata/offair/tetra_band_392.8013M_6000k.cu8 is local only");
        return;
    }
    let buf = sources::FileSource::open(&p).unwrap().read_all().unwrap();
    let mut rx = replay_receiver(&buf, None).unwrap();
    rx.calls_mut().expect("the calls").set_subscriptions(vec![
        crate::mix::calls::Subscription::new(crate::mix::calls::Rule::Everything),
    ]);
    let mut list = crate::calls::Calls::new();
    let mut spoken: std::collections::BTreeMap<String, (f64, std::collections::BTreeSet<u64>)> =
        Default::default();
    let mut sent: std::collections::BTreeMap<(String, u64), f64> = Default::default();
    let mut overlapping = 0;
    let mut per_block: std::collections::BTreeMap<String, Vec<usize>> = Default::default();
    let (mut blocks, mut out_frames) = (0, 0);
    for block in buf.samples.chunks(65_536) {
        rx.process(block).unwrap();
        out_frames += rx.audio_out().0.len() / 2;
        let mut once = std::collections::HashSet::new();
        for v in rx.voices().into_iter().filter(|v| v.system == "TETRA" && v.rate == 8_000.0) {
            let pcm: Vec<u32> = v.pcm.iter().map(|s| s.to_bits()).collect();
            let at = (v.to.clone().unwrap_or_default(), v.channel_hz as u64);
            if once.insert((at.clone(), pcm)) {
                *sent.entry(at).or_default() += v.seconds();
            }
        }
        let mut now: std::collections::BTreeMap<String, std::collections::BTreeSet<u64>> =
            Default::default();
        let mut mixed = std::collections::HashSet::new();
        for v in rx.voices().into_iter().filter(|v| v.system == "TETRA" && v.rate == 48_000.0) {
            let to = v.to.clone().unwrap_or_default();
            let pcm: Vec<u32> = v.pcm.iter().map(|s| s.to_bits()).collect();
            if !mixed.insert((to.clone(), v.channel_hz as u64, pcm)) {
                continue;
            }
            let e = spoken.entry(to.clone()).or_default();
            e.0 += v.seconds();
            e.1.insert(v.channel_hz as u64);
            if v.network.is_some() {
                let frames = per_block.entry(to.clone()).or_default();
                frames.resize(blocks + 1, 0);
                frames[blocks] += v.pcm.len();
                now.entry(to).or_default().insert(v.channel_hz as u64);
            }
        }
        blocks += 1;
        overlapping += now.values().filter(|c| c.len() > 1).count();
        let mut calls = rx.heard_mut().unwrap().take_calls();
        for c in &mut calls {
            c.sites = rx.network().unwrap().sites(&c.key());
            list.hear(c);
        }
    }
    assert_eq!(overlapping, 0, "a call played from two sites in one block");
    assert_eq!(rx.network().unwrap().dropped(), 306, "copies from the other sites");
    let block_frames = (65_536.0 / buf.rate.as_f64() * crate::mix::OUT_HZ) as usize;
    for (to, frames) in &per_block {
        let first = frames.iter().position(|n| *n > 0).unwrap();
        let last = frames.iter().rposition(|n| *n > 0).unwrap();
        let short = frames[first..last].iter().filter(|n| **n < block_frames).count();
        assert_eq!(short, 0, "{to} broke up between blocks {first} and {last}");
    }
    let (air, heard) =
        (buf.samples.len() as f64 / buf.rate.as_f64(), out_frames as f64 / crate::mix::OUT_HZ);
    assert!((air - heard).abs() < 0.001, "{air:.3} s of air made {heard:.3} s of sound");
    let sites = [390.85e6, 391.925e6, 392.55e6, 393.3e6];
    let later = common::time::Instant::now() + common::time::Duration::from_secs(60);
    let rows = list.active(later);
    let networked: Vec<(&str, Option<&str>, usize)> = rows
        .iter()
        .filter(|c| c.network.as_deref() == Some("424-10"))
        .map(|c| (c.to.as_str(), c.from.as_deref(), c.sites.len()))
        .collect();
    assert_eq!(
        networked,
        [("7661987", Some("7661062"), 4), ("7309858", Some("7306696"), 4), ("7306782", None, 1)],
        "one row per call, where there were four"
    );
    assert!(rows.iter().filter(|c| c.sites.len() == 4).all(|c| c.sites == sites));
    assert_eq!(
        rows.iter().filter(|c| c.network.is_none()).count(),
        4,
        "usage markers stay per carrier"
    );
    let round = |s: f64| (s * 100.0).round() / 100.0;
    for to in ["7309858", "7661987"] {
        let best = sent.iter().filter(|((t, _), _)| t == to).map(|(_, s)| *s).fold(0.0, f64::max);
        let played = spoken[to].0;
        assert!(
            played <= best + 1e-9 && played >= best - 0.06 - 1e-9,
            "{to} played {played:.2} s where its most complete site sent {best:.2} s: \
                 no more, and at most one frame less"
        );
    }
    assert_eq!(
        (round(spoken["7309858"].0), round(spoken["7661987"].0)),
        (3.84, 2.88),
        "seconds played"
    );
    assert_eq!(round(sent[&("7309858".to_string(), 392_550_000)]), 2.76, "the site that broke up");
}

#[test]
#[ignore]
fn a_clear_tetra_call_reaches_the_voice_port_through_the_receiver() {
    let p = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../testdata/offair/tetra_voice_390.9M_250k.cu8");
    if !p.exists() {
        eprintln!("skipping: testdata/offair/tetra_voice_390.9M_250k.cu8 is local only");
        return;
    }
    let buf = sources::FileSource::open(&p).unwrap().read_all().unwrap();
    let mut rx = replay_receiver(&buf, None).unwrap();
    let mut rows = Vec::new();
    let mut heard: Vec<common::Voice> = Vec::new();
    for block in buf.samples.chunks(16_384) {
        rx.process(block).unwrap();
        let mut once = std::collections::HashSet::new();
        heard.extend(rx.voices().into_iter().filter(|v| {
            let pcm: Vec<u32> = v.pcm.iter().map(|s| s.to_bits()).collect();
            v.system == "TETRA" && once.insert((v.to.clone(), v.from.clone(), pcm))
        }));
        let at = block_start(common::time::Instant::now(), block.len(), buf.rate.as_f64());
        rows.extend(harvest(&mut rx, at));
    }
    let voice: Vec<&Reception> =
        rows.iter().filter(|r| r.protocol() == "tetra" && r.kind() == "voice").collect();
    assert_eq!(voice.len(), 115, "two bursts go by before the front end is built");
    assert!(voice.iter().all(|r| r.freq() == 390_850_000.0));
    every_row_carries_its_measurements(&voice);

    let spoken: f64 = heard.iter().map(|v| v.seconds()).sum();
    assert!(
        (spoken - (115.0 + 6.0) * 0.06).abs() < 1e-6,
        "{spoken} s spoken, where six slots stolen mid-call are concealed"
    );
    let parties: std::collections::BTreeSet<_> =
        heard.iter().map(|v| (v.to.as_deref(), v.from.as_deref())).collect();
    assert_eq!(parties.len(), 4, "{parties:?}");
    assert!(
        heard.iter().all(|v| v.over.as_ref().map(|o| &o.secrecy) == Some(&common::Secrecy::Clear)),
        "the call is in the clear"
    );
    let peak = heard.iter().flat_map(|v| v.pcm.iter()).fold(0.0f32, |a, s| a.max(s.abs()));
    assert_eq!(peak, 1.0, "ETSI sdecoder reaches full scale on this call");
}

/// The network in the capture enciphers its air interface, so no call
/// control PDU is readable; the MAC headers are, and they say which
/// groups are being addressed. That is worth logging, with what protects
/// it, so a key that undoes it later has a row to change. It is not
/// worth a call row: an enciphered SDU addressed to a radio is as likely
/// to be a registration or a data session as speech, and this twelve
/// seconds contains no traffic channel grant to say otherwise.
#[test]
fn an_encrypting_tetra_network_names_its_busy_groups_but_not_as_calls() {
    let Some(buf) = tetra_fixture() else {
        eprintln!("skipping: fixture absent, run testdata/fetch.sh");
        return;
    };
    let mut rx = replay_receiver(&buf, None).unwrap();
    let out = replay_blocks(&mut rx, &buf);
    let calls = read_as(&out, "tetra", "call");
    assert!(!calls.is_empty(), "no call rows from {} rows", out.len());
    let to = |r: &Reception| {
        r.packet.innermost().and_then(|l| l.link.to.as_ref()).map(|p| p.label().to_string())
    };
    let cipher = |r: &Reception| {
        r.packet.facts().find_map(|(_, f)| match f {
            common::packet::Fact::Protected(s) => s.cipher().map(str::to_string),
            _ => None,
        })
    };
    let groups: Vec<String> = calls.iter().filter_map(|r| to(r)).collect();
    assert!(groups.iter().any(|g| g == "10223295" || g == "15835885"), "addressed {groups:?}");
    // The network says what protects its air interface in the clear, on
    // every header, whether or not anything here can read the traffic.
    assert!(
        calls.iter().all(|r| cipher(r).as_deref() == Some("AIE-3")),
        "{:?}",
        calls.iter().map(|r| (to(r), cipher(r), r.detail())).collect::<Vec<_>>()
    );
    // Not a row per slot: an address that keeps being addressed is one
    // row every couple of seconds.
    assert!(calls.len() <= 12, "{} rows in twelve seconds", calls.len());

    // A MAC header says an address is being talked to, not that anybody
    // is talking: behind an enciphered SDU it is as likely to be a radio
    // registering or a data session. Those rows belong in the log and
    // not in a list of voice calls.
    // The call list is fed from the audio bus, and nothing on this
    // capture ever reached it: twelve seconds of enciphered headers is
    // not twelve seconds of anybody talking.
    assert!(
        rx.calls().is_none_or(|c| !c.listening()),
        "nothing here proved a voice call: {:?}",
        calls.iter().map(|r| (to(r), r.detail())).collect::<Vec<_>>()
    );
}
