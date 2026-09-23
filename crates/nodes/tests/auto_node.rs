//! One node over a span, told nothing, against recordings of three kinds of
//! thing: keyed sensors, Mode S replies and a pager transmission.

use common::packet::Symbols;
use common::{C32, Hz};
use dsp::Mixer;
use nodes::{NodeSpec, build_chain, registry};
use pipeline::StreamSpec;
use sources::FileSource;

fn fixture(name: &str) -> Option<common::IqBuf> {
    let p = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../testdata").join(name);
    if !p.exists() {
        eprintln!("skipping: {name} absent, run testdata/fetch.sh");
        return None;
    }
    FileSource::open(&p).ok()?.read_all().ok()
}

/// Run a stream through one stage, in radio-sized blocks, and collect the
/// packets it puts out, letting the last source drain.
fn packets(stage: NodeSpec, rate: f64, center: Hz, iq: &[C32]) -> Vec<common::packet::Packet> {
    let mut g = build_chain(StreamSpec::iq(rate, center), &[stage], &registry()).expect("build");
    let mut out = Vec::new();
    let silence = vec![C32::new(0.0, 0.0); 16_384];
    for block in iq.chunks(16_384).chain(std::iter::repeat_n(&silence[..], 4)) {
        g.feed_iq(block).expect("run");
        if let pipeline::Payload::Packets(p) = g.output() {
            out.extend(p.iter().map(|p| {
                let mut p = p.clone();
                if p.carrier.center_hz == 0 {
                    p.carrier.center_hz = center.0;
                }
                p
            }));
        }
    }
    out
}

/// The same, with what the auto node had learned by the end: the
/// transmitters a front end inside claimed the bursts of.
fn run_auto(rate: f64, center: Hz, iq: &[C32]) -> Ran {
    let mut g = build_chain(StreamSpec::iq(rate, center), &[NodeSpec::new("auto")], &registry())
        .expect("build");
    let mut packets = Vec::new();
    let silence = vec![C32::new(0.0, 0.0); 16_384];
    for block in iq.chunks(16_384).chain(std::iter::repeat_n(&silence[..], 4)) {
        g.feed_iq(block).expect("run");
        if let pipeline::Payload::Packets(p) = g.output() {
            packets.extend_from_slice(p);
        }
    }
    let auto = g
        .order()
        .find_map(|(id, _)| g.node(id)?.as_any().downcast_ref::<nodes::AutoNode>())
        .expect("the auto node");
    Ran {
        locked: auto
            .locked_transmitters()
            .into_iter()
            .map(|(p, t, c)| (p.to_string(), t.to_string(), c))
            .collect(),
        sources: auto.built(),
        packets,
    }
}

/// What one run of the auto node over a capture left behind.
struct Ran {
    packets: Vec<common::packet::Packet>,
    /// The transmitters a front end inside had learned by the end, as
    /// (front end, what it calls the transmitter, how sure it still is).
    locked: Vec<(String, String, f32)>,
    /// Sources the node built decoders for over the run.
    sources: u64,
}

fn decodes(pk: &[common::packet::Packet], model: &str) -> Vec<(u64, String)> {
    let protocols = decode::Protocols::published();
    let mut out = Vec::new();
    for p in pk {
        let Some(Symbols::Pulses(pulses)) = p.keying.as_ref().map(|k| &k.symbols) else {
            continue;
        };
        for r in protocols.decode_all(pulses) {
            if r.model.contains(model) && r.proof.passed() {
                out.push((p.carrier.center_hz, r.to_string()));
            }
        }
    }
    out
}

#[test]
fn four_sensors_placed_anywhere_all_decode() {
    let Some(buf) = fixture("fineoffset_wh1080_433.92M_250k.cu8") else {
        return;
    };
    let rate = 250_000.0;
    let offsets = [-93_000.0, -37_000.0, 21_500.0, 78_000.0];
    let stagger = 1_250usize;
    let mut wide = vec![C32::new(0.0, 0.0); buf.samples.len() + stagger * offsets.len()];
    for (k, &off) in offsets.iter().enumerate() {
        let mut m = Mixer::new(off, rate);
        let mut s = Vec::new();
        m.process(&buf.samples, &mut s);
        for (o, x) in wide[k * stagger..].iter_mut().zip(&s) {
            *o += *x;
        }
    }
    let pk = packets(NodeSpec::new("auto"), rate, buf.center, &wide);
    // Every burst carries its own samples at the rate it was read at, and
    // the full 184 ms transmission is among them. The router also emits the
    // short repeats and fragments a transmission breaks into, so not every
    // packet is the whole thing, but the whole thing is there.
    let longest = pk
        .iter()
        .filter_map(|p| p.carrier.iq.as_ref())
        .map(|iq| {
            assert!(!iq.samples.is_empty() && iq.rate > 0.0, "a burst without samples or rate");
            iq.samples.len() as f64 / iq.rate
        })
        .fold(0.0f64, f64::max);
    assert!(longest > 0.15, "the full transmission's samples are missing; longest {longest:.3}s");
    let mut got = decodes(&pk, "WHx080");
    got.sort();
    got.dedup_by(|a, b| a.0.abs_diff(b.0) < 4_000);
    assert_eq!(got.len(), 4, "{got:#?}");
    for (hz, text) in &got {
        assert!(text.contains("station_id=196"), "{text}");
        let off = *hz as f64 - buf.center.as_f64();
        // 4.6 kHz is where this recording's carrier sits relative to nominal.
        assert!(
            offsets.iter().any(|o| (off - o - 4_600.0).abs() < 4_000.0),
            "decoded at {off:+.0} Hz"
        );
    }
}

#[test]
fn mode_s_replies_are_heard_without_being_asked_for() {
    let Some(buf) = fixture("adsb_1090M_2400k.cu8") else {
        return;
    };
    let rate = buf.rate.as_f64();
    let alone = packets(NodeSpec::new("mode_s"), rate, buf.center, &buf.samples);
    let auto = packets(NodeSpec::new("auto"), rate, buf.center, &buf.samples);
    let frames =
        |pk: &[common::packet::Packet]| pk.iter().filter(|p| !p.bytes().is_empty()).count();
    assert!(frames(&alone) > 10, "the Mode S stage alone heard {} frames", frames(&alone));
    assert_eq!(frames(&auto), frames(&alone), "the auto node hears what the Mode S stage does");
}

#[test]
fn a_pager_transmission_somewhere_in_the_span_becomes_a_page() {
    // A 1200 baud POCSAG page, keyed at 4.5 kHz deviation, 310 kHz above
    // the centre of a 2.4 MS/s span, in noise. Nothing names the frequency.
    let rate = 2_400_000.0;
    let center = Hz::hz(439_800_000);
    let offset = 310_000.0;
    let contents = decode::pocsag::encode(
        1_234_568,
        3,
        &decode::pocsag::Body::Alpha("MOVE TO CHANNEL 2".into()),
    );
    let bits = dsp::pocsag::encode_bits(&contents);
    let sps = (rate / 1200.0) as usize;
    let mut seed = 0x2545F4914F6CDD1Du64;
    let mut noise = move || {
        seed ^= seed << 13;
        seed ^= seed >> 7;
        seed ^= seed << 17;
        let u1 = ((seed >> 11) as f64 / (1u64 << 53) as f64).max(1e-12);
        seed ^= seed << 13;
        seed ^= seed >> 7;
        seed ^= seed << 17;
        let u2 = (seed >> 11) as f64 / (1u64 << 53) as f64;
        let r = (-2.0 * u1.ln()).sqrt();
        C32::new(
            (r * (std::f64::consts::TAU * u2).cos()) as f32 * 0.02,
            (r * (std::f64::consts::TAU * u2).sin()) as f32 * 0.02,
        )
    };
    let lead = 600_000usize;
    let mut iq: Vec<C32> = (0..lead).map(|_| noise()).collect();
    let mut phase = 0.0f64;
    for &b in &bits {
        let f = offset + if b { -4_500.0 } else { 4_500.0 };
        for _ in 0..sps {
            phase += std::f64::consts::TAU * f / rate;
            iq.push(C32::new(0.3 * phase.cos() as f32, 0.3 * phase.sin() as f32) + noise());
        }
    }
    iq.extend((0..lead).map(|_| noise()));

    let pk = packets(NodeSpec::new("auto"), rate, center, &iq);
    let frames: Vec<&common::packet::Packet> =
        pk.iter().filter(|p| !p.bytes().is_empty()).collect();
    assert!(!frames.is_empty(), "no frame came out; packets: {}", pk.len());
    let f = frames[0];
    assert!(
        (f.carrier.center_hz as f64 - (center.as_f64() + offset)).abs() < 5_000.0,
        "page at {}",
        f.carrier.center_hz
    );
    let pages = decode::pocsag::read(f.bytes());
    assert_eq!(pages.len(), 1, "{pages:?}");
    assert_eq!(pages[0].wrote(), Some("MOVE TO CHANNEL 2"));
}

#[test]
fn a_lora_burst_somewhere_in_the_span_is_named_a_chirp() {
    // Chirp spread spectrum, 125 kHz wide at spreading factor 9: a symbol
    // sweeps the whole width in 4.1 ms, so in the frames it takes a source to
    // open the burst is a tone a few kilohertz wide. Eight upchirps of
    // preamble and thirty of payload, 150 kHz below the centre of a 2.4 MS/s
    // span, in noise.
    let rate = 2_400_000.0;
    let center = Hz::hz(869_500_000);
    let offset = -150_000.0;
    let bw = 125_000.0;
    let symbol = (rate * 512.0 / bw) as usize;
    let mut seed = 0x9E3779B97F4A7C15u64;
    let mut noise = move || {
        seed ^= seed << 13;
        seed ^= seed >> 7;
        seed ^= seed << 17;
        let u1 = ((seed >> 11) as f64 / (1u64 << 53) as f64).max(1e-12);
        seed ^= seed << 13;
        seed ^= seed >> 7;
        seed ^= seed << 17;
        let u2 = (seed >> 11) as f64 / (1u64 << 53) as f64;
        let r = (-2.0 * u1.ln()).sqrt();
        C32::new(
            (r * (std::f64::consts::TAU * u2).cos()) as f32 * 0.02,
            (r * (std::f64::consts::TAU * u2).sin()) as f32 * 0.02,
        )
    };
    let lead = 800_000usize;
    let mut iq: Vec<C32> = (0..lead).map(|_| noise()).collect();
    let mut ph = 0.0f64;
    for k in 0..38usize {
        // Payload symbols start their sweep partway through, as a modulated
        // chirp does; the preamble sweeps from the bottom.
        let shift = if k < 8 { 0.0 } else { ((k * 97) % 512) as f64 / 512.0 };
        for i in 0..symbol {
            let t = ((i as f64 / symbol as f64) + shift) % 1.0;
            let f = offset - bw / 2.0 + bw * t;
            ph += std::f64::consts::TAU * f / rate;
            iq.push(C32::new(0.3 * ph.cos() as f32, 0.3 * ph.sin() as f32) + noise());
        }
    }
    iq.extend((0..lead).map(|_| noise()));

    // It leaves the node as a packet carrying its measurement, with no
    // timings, which is what a log or a list gets to show for it.
    let pk = packets(NodeSpec::new("auto"), rate, center, &iq);
    let measured: Vec<(u64, common::packet::Keying)> =
        pk.iter().filter_map(|p| p.keying.clone().map(|k| (p.carrier.center_hz, k))).collect();
    let (hz, chirp) = measured
        .iter()
        .find(|(_, k)| k.modulation == common::Modulation::Chirp)
        .unwrap_or_else(|| {
            panic!(
                "no chirp measurement among {:?}",
                measured.iter().map(|(_, k)| k.modulation).collect::<Vec<_>>()
            )
        });
    assert!(chirp.params.sweep_hz_s.abs() > 1e6, "{chirp:?}");
    assert!(chirp.params.bandwidth_hz > 60_000.0, "{chirp:?}");
    let at = *hz as f64 - center.as_f64();
    assert!((at - offset).abs() < 20_000.0, "measured at {at:+.0} Hz, sent at {offset:+.0}");
}

/// M17 has no home frequency: it runs wherever an amateur puts it, so being
/// told a channel is not detection. The front end is one demodulator on one
/// channel, and what makes it general is that the source detector finds the
/// transmission first and hands it a stream centred on it.
#[test]
fn an_m17_transmission_anywhere_in_the_span_is_found_and_read() {
    use decode::m17::Address;
    use dsp::m17::{BAUD, DEVIATION_HZ, Kind, fec, frame_symbols, preamble_symbols};

    let rate = 2_400_000.0;
    let center = Hz::hz(433_000_000);
    // Nowhere near the calling channel, and not on any grid.
    let offset = 417_300.0;

    let mut lsf = [0u8; 30];
    lsf[..6].copy_from_slice(&Address::encode("ALL").to_be_bytes()[2..]);
    lsf[6..12].copy_from_slice(&Address::encode("M0ABC").to_be_bytes()[2..]);
    lsf[12..14].copy_from_slice(&(1u16 | 2 << 1).to_be_bytes());
    let crc = fec::crc16(&lsf[..28]);
    lsf[28..].copy_from_slice(&crc.to_be_bytes());

    let mut symbols = preamble_symbols();
    symbols.extend(frame_symbols(Kind::Lsf, &lsf, 0, &[0; 6]));
    for n in 0..25u16 {
        let cnt = (n % 6) as usize;
        let mut lich = [0u8; 6];
        lich[..5].copy_from_slice(&lsf[cnt * 5..cnt * 5 + 5]);
        lich[5] = (cnt as u8) << 5;
        symbols.extend(frame_symbols(Kind::Stream, &[0x5au8; 16], n, &lich));
    }

    // With noise, as the pager test does: the detector estimates its floor
    // from the quietest recent frames, and a span of exact zeros has no
    // floor to estimate.
    let mut seed = 0x2545_F491_4F6C_DD1Du64;
    let mut noise = move || {
        seed ^= seed << 13;
        seed ^= seed >> 7;
        seed ^= seed << 17;
        let u1 = ((seed >> 11) as f64 / (1u64 << 53) as f64).max(1e-12);
        seed ^= seed << 13;
        seed ^= seed >> 7;
        seed ^= seed << 17;
        let u2 = (seed >> 11) as f64 / (1u64 << 53) as f64;
        let r = (-2.0 * u1.ln()).sqrt();
        C32::new(
            (r * (std::f64::consts::TAU * u2).cos()) as f32 * 0.02,
            (r * (std::f64::consts::TAU * u2).sin()) as f32 * 0.02,
        )
    };
    let sps = (rate / BAUD) as usize;
    let mut iq: Vec<C32> = (0..600_000).map(|_| noise()).collect();
    let mut phase = 0.0f64;
    for &s in &symbols {
        let f = offset + f64::from(s) / 3.0 * DEVIATION_HZ;
        for _ in 0..sps {
            phase += std::f64::consts::TAU * f / rate;
            iq.push(C32::new(0.3 * phase.cos() as f32, 0.3 * phase.sin() as f32) + noise());
        }
    }
    iq.extend((0..600_000).map(|_| noise()));

    let pk = packets(NodeSpec::new("auto"), rate, center, &iq);
    let rows: Vec<common::packet::Proto> =
        pk.iter().filter_map(|p| decode::m17::read(p.bytes())).collect();
    let setup = rows.iter().find(|d| d.kind == "link_setup");
    assert!(setup.is_some(), "nothing read as M17; {} packets", pk.len());
    assert_eq!(setup.unwrap().parties().0, Some("M0ABC"));
}

/// A real DMR capture through the auto node: no frequency told, only a span.
/// Proves the auto path detects the carrier, places the dmr front end on it,
/// decodes a voice over and labels it, so it reaches the call list as DMR.
///
/// Ignored because it needs a *clean* capture: the auto node places a front
/// end only when the detected source measures within
/// [`CHANNEL_WIDTH_TOLERANCE`] of the channel width, and the corpus capture,
/// which has a strong nearby spur from an overloaded front end, measures
/// far wider than 12.5 kHz and is rightly refused. A capture from an SDR with
/// a clean front end (no close-in spur, radio not overloading it) opens a
/// ~14 kHz source and places DMR. See the direct-path test in dmr_nodes for
/// the decode proven without the detector in the way.
#[cfg(feature = "ambe")]
#[test]
#[ignore]
fn auto_finds_dmr_in_a_real_capture() {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../testdata/dmr_tg9_433.45M_2048k.cu8");
    if !std::path::Path::new(path).exists() {
        eprintln!("skipping: dmr_tg9_433.45M_2048k.cu8 absent, run testdata/fetch.sh");
        return;
    }
    let raw = std::fs::read(path).unwrap();
    let rate = 2_048_000.0;
    let center = Hz(433_450_000);
    let iq: Vec<C32> = raw
        .chunks_exact(2)
        .map(|c| C32::new((c[0] as f32 - 127.5) / 127.5, (c[1] as f32 - 127.5) / 127.5))
        .collect();

    let pk = packets(NodeSpec::new("auto"), rate, center, &iq);
    let dmr: Vec<_> = pk.iter().filter_map(|p| decode::dmr::read(p.bytes())).collect();
    assert!(!dmr.is_empty(), "auto placed no DMR that decoded; {} packets", pk.len());
    let d = &dmr[0];
    assert_eq!(d.id, "dmr");
    // The speech is not on the packets: an over is stated on the voice port,
    // which is where a call-list row plays it back from.
    eprintln!("auto decoded {} DMR row(s)", dmr.len());
}

/// A real LoRa capture through the auto node: no frequency told, only a span.
/// Proves the auto path detects the chirp source, places the lora front end,
/// and that a decoded frame reaches the log as LoRa, not just a chirp
/// description. Skips when the fixture is absent.
#[test]
#[ignore]
fn auto_finds_lora_in_a_real_capture() {
    let p = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../testdata/offair/lora_sf11_meshtastic_a_869.525M_2000k.cs16");
    if !p.exists() {
        eprintln!("skipping: {p:?} absent");
        return;
    }
    let buf = sources::FileSource::open(&p).unwrap().read_all().unwrap();
    let iq: Vec<C32> = buf.samples.clone();
    let rate = 2_000_000.0;
    let center = Hz(869_525_000);
    let pk = packets(NodeSpec::new("auto"), rate, center, &iq);
    let lora: Vec<_> = pk.iter().filter_map(|p| decode::lora::read(p.bytes())).collect();
    let chirps = pk
        .iter()
        .filter(|p| p.keying.as_ref().map(|k| k.modulation) == Some(common::Modulation::Chirp))
        .count();
    // The one Meshtastic packet in the capture, out of the three rows the
    // span produces. Pinned, because the way the verdict path breaks is a
    // second row for the same packet or a decode replaced by a chirp
    // measurement, and neither empties the list.
    assert_eq!(
        lora.len(),
        1,
        "{} LoRa decoded of {} packets, {chirps} chirps",
        lora.len(),
        pk.len()
    );
    assert_eq!(pk.len(), 3, "{} packets in all, {chirps} chirps", pk.len());
}

/// An ExpressLRS handset heard through the whole auto path: the source
/// opened a megahertz wide, the burst of four packets named a chirp, the
/// front end placed on the verdict, the link recovered from the packets'
/// own CRC seeds with no sync packet in the span, the sticks read, and the
/// hop set locked so the visits after the first two cost one extraction
/// each and nothing else.
///
/// Three separate faults kept this at zero before there was a capture:
/// the detector refused any source wider than 600 kHz, the classifier
/// measured the four packets as one keyed burst, and nothing placed an
/// ExpressLRS decoder at all. None of them showed on synthesised packets.
#[test]
fn auto_reads_an_expresslrs_handset_in_a_real_capture() {
    let p = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../testdata/offair/elrs_100hz_2415M_20000k.cs8");
    if !p.exists() {
        eprintln!("skipping: elrs_100hz_2415M_20000k.cs8 absent, run testdata/fetch.sh");
        return;
    }
    let buf = sources::FileSource::open(&p).unwrap().read_all().unwrap();
    let Ran { packets: pk, locked, .. } = run_auto(buf.rate.as_f64(), buf.center, &buf.samples);
    let rows: Vec<_> = pk.iter().filter_map(|p| decode::elrs::read(p.bytes())).collect();
    let measured = pk
        .iter()
        .filter(|p| {
            p.keying
                .as_ref()
                .is_some_and(|k| matches!(k.how, common::packet::Knowledge::Measured { .. }))
        })
        .count();
    // Sixteen packets on four channel visits, four to a visit, and eighteen
    // rows in all. Pinned exactly: the way this breaks is a packet lost to a
    // floor or a splice, and "twelve or more" would not say so.
    //
    // It was fifteen of nineteen before the hop set could be locked, and
    // both numbers moved for the same reason. The link decodes on the first
    // visit and a lock over the eighty channels goes up, so the third and
    // fourth visits are claimed: each is read by an ExpressLRS front end
    // that already knows the link, with no classifier beside it. Knowing the
    // link is the extra packet, since a visit whose link has to be recovered
    // holds its first packets back and drops one that repeats; and the two
    // rows fewer are the classifier's own chirp measurements of those two
    // visits, which are not news about a source a front end read. The first
    // two visits overlap in time, so the second opens before the first has
    // finished decoding and neither is claimed; their chirp rows stay.
    assert_eq!(
        rows.len(),
        16,
        "{} ExpressLRS rows of {} packets, {measured} measurements",
        rows.len(),
        pk.len()
    );
    assert_eq!(pk.len(), 18, "{} packets in all", pk.len());
    assert_eq!(measured, 2, "the classifier's measurement of the visits nothing claimed");
    // And the lock the front end published, still believed at the end.
    assert_eq!(locked.len(), 1, "{locked:?}");
    assert_eq!(locked[0].0, "elrs");
    assert_eq!(locked[0].1, "6f37");
    assert_eq!(locked[0].2, 1.0, "the lock lost confidence: {locked:?}");
    for r in &rows {
        assert_eq!(r.subject.as_ref().map(|e| e.id.to_string()).as_deref(), Some("6f37"), "{r:?}");
        // The sticks the views draw, as microseconds rather than as counts.
        // This is a Full rate, so eight channels arrive: the four sticks and
        // AUX2-5, with nothing said about AUX6-9. The throttle is at 988 us,
        // the bottom of the CRSF span.
        let Some(common::packet::Fact::Control(sticks)) =
            r.facts.iter().find(|f| matches!(f, common::packet::Fact::Control(_)))
        else {
            panic!("an rc packet with no sticks: {r:?}")
        };
        let common::packet::Sticks { channels, armed, uplink_power_mw } = sticks;
        assert_eq!(armed, &Some(false), "{r:?}");
        assert!(channels[..8].iter().all(Option::is_some), "{channels:?}");
        assert!(channels[8..].iter().all(Option::is_none), "{channels:?}");
        assert_eq!(channels[2], Some(988), "throttle in us: {channels:?}");
        // Two sticks centred to within a count, which is 1.25 us.
        assert!(
            channels[..2].iter().flatten().all(|us| (1495..=1520).contains(us)),
            "{channels:?}"
        );
        // Power index 0, which the firmware's own enum order makes 10 mW.
        // The bench handset's setting was not recorded, so this pins what the
        // packet says rather than what the transmitter was told to do.
        assert_eq!(uplink_power_mw, &Some(10), "{r:?}");
    }
    let channels: std::collections::BTreeSet<u64> = pk
        .iter()
        .filter(|p| decode::elrs::read(p.bytes()).is_some())
        .map(|p| p.carrier.center_hz / 100_000)
        .collect();
    // The four channel visits in the capture, in hundreds of kilohertz.
    assert_eq!(channels, [24084, 24114, 24125, 24224].into_iter().collect(), "{channels:?}");
}

/// The same handset on a whole 2.4 GHz band, hopping through it: one
/// transmitter, thirty channel visits in two seconds, and a lock that turns
/// every visit after the first into one extraction and one decoder.
///
/// This is the capture the lock layer exists for. Without it the detector
/// opens fifty-three sources in two seconds and nothing joins them up: each
/// hop gets a burst classifier, a LoRa decoder and an ExpressLRS decoder
/// built, run and torn down, and the receiver runs at 0.38 times real time on
/// four threads. What is pinned here is what it reads, which is what an
/// optimisation may not change.
#[test]
fn a_hopping_link_is_one_transmitter_on_a_busy_band() {
    let Some(buf) = fixture("ism24_busy_2431M_61440k.cs16") else {
        return;
    };
    let Ran { packets: pk, locked, sources } =
        run_auto(buf.rate.as_f64(), buf.center, &buf.samples);
    let rows: Vec<_> = pk.iter().filter_map(|p| decode::elrs::read(p.bytes())).collect();
    let visits: std::collections::BTreeSet<u64> = pk
        .iter()
        .filter(|p| decode::elrs::read(p.bytes()).is_some())
        .map(|p| p.carrier.center_hz)
        .collect();
    assert_eq!(rows.len(), 107, "{} ExpressLRS rows of {} packets", rows.len(), pk.len());
    assert_eq!(visits.len(), 30, "{visits:?}");
    // The fifty-three sources the detector opens in two seconds, which is the
    // number the lock layer was built for: a lock does not stop a source
    // opening, it decides what is built on one.
    assert_eq!(sources, 53, "sources opened");
    // One handset, and it is the same one throughout: a second link id here
    // would be a CRC seeded from the wrong two bytes agreeing by chance.
    assert!(
        rows.iter()
            .all(|r| r.subject.as_ref().map(|e| e.id.to_string()).as_deref() == Some("6f37")),
        "more than one link"
    );
    // The lock the handset's front end published, still believed after every
    // claim it made: the eighty channels are a megahertz apart from
    // 2400.4 MHz and the visits sit within 195 kHz of one of them.
    assert_eq!(locked.len(), 1, "{locked:?}");
    assert_eq!((locked[0].0.as_str(), locked[0].1.as_str()), ("elrs", "6f37"));
    assert_eq!(locked[0].2, 1.0, "the lock lost confidence: {locked:?}");
    // Every visit is on a channel of the hop set, bar the first: it opens
    // while the detector's floor is still settling and its centroid lands
    // 463 kHz below the channel, which is what the lock is published too
    // late to claim anyway.
    let off_raster = visits
        .iter()
        .filter(|hz| {
            let k = ((**hz as f64 - 2_400_400_000.0) / 1e6).round();
            (**hz as f64 - (2_400_400_000.0 + k * 1e6)).abs() > 250_000.0
        })
        .count();
    assert_eq!(off_raster, 1, "{visits:?}");
}

/// A channel a front end has read is kept for the session. The Meshtastic
/// capture decodes once through detection; afterwards the node must report
/// the LoRa channel as remembered, so the next packet on it is read without
/// having to be found, however the converter measures it that time.
#[test]
fn a_channel_that_decoded_is_remembered() {
    let p = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../testdata/offair/lora_sf11_meshtastic_c_869.0M_2400k.cu8");
    if !p.exists() {
        eprintln!("skipping: {p:?} absent, run testdata/fetch.sh");
        return;
    }
    let buf = FileSource::open(&p).unwrap().read_all().unwrap();
    let rate = 2_400_000.0;
    let center = Hz(869_000_000);
    let mut g =
        build_chain(StreamSpec::iq(rate, center), &[NodeSpec::new("auto")], &registry()).unwrap();
    let mut decoded = 0;
    let mut rows = 0;
    let mut measures = 0;
    for block in buf.samples.chunks(16_384) {
        g.feed_iq(block).unwrap();
        if let pipeline::Payload::Packets(p) = g.output() {
            rows += p.len();
            measures += p
                .iter()
                .filter(|p| {
                    p.keying.as_ref().is_some_and(|k| {
                        matches!(k.how, common::packet::Knowledge::Measured { .. })
                    })
                })
                .count();
            decoded += p.iter().filter(|p| decode::lora::read(p.bytes()).is_some()).count();
        }
    }
    // The one Meshtastic packet in the capture, the five rows the receiver
    // puts out for it, and the four of those that carry the burst front
    // end's measurement of what it saw. Pinned rather than left at "one or
    // more" because the classifier now rides along on the remembered
    // channel, and the way that goes wrong is a duplicate row or a row that
    // lost its measurement, neither of which changes whether something
    // decoded.
    assert_eq!(decoded, 1, "{decoded} LoRa packets of {rows} rows");
    assert_eq!(rows, 5, "rows: {rows}");
    assert_eq!(measures, 4, "rows carrying a measurement: {measures}");
    let auto = g
        .order()
        .find_map(|(id, _)| g.node(id)?.as_any().downcast_ref::<nodes::AutoNode>())
        .expect("the auto node");
    let kept = auto.remembered();
    let lora = kept.iter().find(|(name, _, _)| *name == "lora");
    let Some((_, hz, width)) = lora else { panic!("no LoRa channel remembered: {kept:?}") };
    assert!((hz - 869_525_000.0).abs() < 50_000.0, "remembered at {hz}");
    assert_eq!(*width, 250_000.0);
}

fn gaussian(seed: u64, sigma: f32) -> impl FnMut() -> C32 {
    let mut seed = seed;
    let mut next = move || {
        seed ^= seed << 13;
        seed ^= seed >> 7;
        seed ^= seed << 17;
        (seed >> 11) as f64 / (1u64 << 53) as f64
    };
    move || {
        let u1 = next().max(1e-12);
        let u2 = next();
        let r = (-2.0 * u1.ln()).sqrt();
        C32::new(
            (r * (std::f64::consts::TAU * u2).cos()) as f32 * sigma,
            (r * (std::f64::consts::TAU * u2).sin()) as f32 * sigma,
        )
    }
}

fn z_wave_burst(
    iq: &mut Vec<C32>,
    noise: &mut impl FnMut() -> C32,
    center: Hz,
    hz: f64,
    baud: f64,
    value: u8,
) {
    let rate = 2_000_000.0;
    let (deviation, manchester, fcs, preamble) = match baud {
        19_200.0 => (20_000.0, true, decode::zwave::Fcs::Xor, 20),
        40_000.0 => (20_000.0, false, decode::zwave::Fcs::Xor, 20),
        _ => (29_000.0, false, decode::zwave::Fcs::Crc16, 25),
    };
    let frame = decode::zwave::encode(
        fcs,
        0xd6b2_6208,
        1,
        7,
        decode::zwave::singlecast_control(3, true),
        &[0x25, 0x01, value],
    );
    let bits = decode::zwave::keyed(&frame, preamble);
    let symbols: Vec<bool> =
        if manchester { bits.iter().flat_map(|b| [*b, !*b]).collect() } else { bits };
    let keyed = dsp::fsk::modulate(&symbols, rate, baud, deviation, 0.3);
    let mut mix = Mixer::new(hz - center.as_f64(), rate);
    let mut moved = Vec::new();
    mix.process(&keyed, &mut moved);
    iq.extend(moved.into_iter().map(|s| s + noise()));
    iq.extend((0..300_000).map(|_| noise()));
}

fn z_wave_read(pk: &[common::packet::Packet]) -> Vec<(u64, u8)> {
    pk.iter()
        .filter(|p| decode::zwave::read(p.bytes()).is_some())
        .map(|p| (p.carrier.center_hz, decode::zwave::parse(p.bytes()).unwrap().payload[2]))
        .collect()
}

#[test]
fn z_wave_at_all_three_rates_is_read_and_868_49_is_not() {
    let center = Hz::hz(868_200_000);
    let mut noise = gaussian(0x5DEE_CE66_D1CE_4E5B, 0.02);
    let mut iq: Vec<C32> = (0..300_000).map(|_| noise()).collect();
    z_wave_burst(&mut iq, &mut noise, center, 868_490_000.0, 40_000.0, 0x44);
    z_wave_burst(&mut iq, &mut noise, center, 868_420_000.0, 19_200.0, 0x11);
    z_wave_burst(&mut iq, &mut noise, center, 868_420_000.0, 40_000.0, 0x22);
    z_wave_burst(&mut iq, &mut noise, center, 868_400_000.0, 100_000.0, 0x33);
    let pk = packets(NodeSpec::new("auto"), 2_000_000.0, center, &iq);
    let read = z_wave_read(&pk);
    let values: Vec<u8> = read.iter().map(|(_, v)| *v).collect();
    assert_eq!(values, [0x11, 0x22, 0x33], "{read:x?} of {} packets", pk.len());
    assert!(read.iter().all(|(hz, _)| hz.abs_diff(868_410_000) < 15_000), "{read:?}");
}

#[test]
fn z_wave_up_to_24_khz_off_is_built_on_open_since_the_classifier_misses_40_kbit_s() {
    let center = Hz::hz(868_200_000);
    let (mut frames, mut rows) = (Vec::new(), Vec::new());
    for (hz, baud) in
        [(868_420_000.0, 19_200.0), (868_420_000.0, 40_000.0), (868_400_000.0, 100_000.0)]
    {
        let (mut f, mut r) = (0, 0);
        for sigma in [0.03f32, 0.1] {
            for trial in 0..12u64 {
                let mut noise = gaussian(0x1234_5678 ^ (trial * 7919 + baud as u64), sigma);
                let lead = 300_000 + trial as usize * 7_919;
                let mut iq: Vec<C32> = (0..lead).map(|_| noise()).collect();
                let off = (trial as f64 - 6.0) * 4_000.0;
                z_wave_burst(&mut iq, &mut noise, center, hz + off, baud, trial as u8);
                let pk = packets(NodeSpec::new("auto"), 2_000_000.0, center, &iq);
                let read = z_wave_read(&pk);
                assert!(read.iter().all(|(_, v)| *v == trial as u8), "{read:?}");
                f += usize::from(!read.is_empty());
                r += read.len();
            }
        }
        frames.push(f);
        rows.push(r);
    }
    assert_eq!(
        frames,
        [24, 24, 20],
        "bursts read of 24 at 9.6, 40 and 100 kbit/s; master before #195 read [0, 1, 20], \
         and waiting for an Fsk2 or Msk verdict reads [24, 0, 6]"
    );
    assert_eq!(rows, [24, 28, 20], "four 40 kbit/s bursts at 0.1 open two sources each");
}
