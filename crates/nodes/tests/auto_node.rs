//! One node over a span, told nothing, against recordings of three kinds of
//! thing: keyed sensors, Mode S replies and a pager transmission.

use common::{Hz, PacketBody, C32};
use dsp::Mixer;
use nodes::{build_chain, registry, NodeSpec};
use pipeline::StreamSpec;
use sources::FileSource;

fn fixture(name: &str) -> Option<common::IqBuf> {
    let p = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../testdata").join(name);
    if !p.exists() {
        eprintln!("skipping: {name} absent, run testdata/fetch.sh");
        return None;
    }
    Some(FileSource::open(&p).ok()?.read_all().ok()?)
}

/// Run a stream through one stage, in radio-sized blocks, and collect the
/// packets it puts out, letting the last source drain.
fn packets(stage: NodeSpec, rate: f64, center: Hz, iq: &[C32]) -> Vec<common::Packet> {
    let mut g = build_chain(StreamSpec::iq(rate, center), &[stage], &registry()).expect("build");
    let mut out = Vec::new();
    let silence = vec![C32::new(0.0, 0.0); 16_384];
    for block in iq.chunks(16_384).chain(std::iter::repeat(&silence[..]).take(4)) {
        g.feed_iq(block).expect("run");
        match g.output() {
            pipeline::Payload::Packets(p) => out.extend_from_slice(p),
            pipeline::Payload::Frames(f) => {
                out.extend(f.iter().map(|f| common::Packet {
                    at_us: 0,
                    center_hz: center.0,
                    bandwidth_hz: 0,
                    rssi_dbfs: f32::NAN,
                    snr_db: f32::NAN,
                    modulation: None,
                    body: PacketBody::Frame(f.clone()),
                    measure: None,
                    audio: None,
                    iq: None,
                }))
            }
            _ => {}
        }
    }
    out
}

fn decodes(pk: &[common::Packet], model: &str) -> Vec<(u64, String)> {
    let protocols = decode::Protocols::all();
    let mut out = Vec::new();
    for p in pk {
        let Some(pkg) = p.package() else { continue };
        for r in protocols.decode_all(&pkg) {
            if r.model.contains(model) && r.crc_valid == Some(true) {
                out.push((p.center_hz, r.to_string()));
            }
        }
    }
    out
}

#[test]
fn four_sensors_placed_anywhere_all_decode() {
    let Some(buf) = fixture("fineoffset_wh1080_433.92M_250k.cu8") else { return };
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
        .filter_map(|p| p.iq.as_ref())
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
        assert!(offsets.iter().any(|o| (off - o - 4_600.0).abs() < 4_000.0), "decoded at {off:+.0} Hz");
    }
}

#[test]
fn mode_s_replies_are_heard_without_being_asked_for() {
    let Some(buf) = fixture("adsb_1090M_2400k.cu8") else { return };
    let rate = buf.rate.as_f64();
    let alone = packets(NodeSpec::new("mode_s"), rate, buf.center, &buf.samples);
    let auto = packets(NodeSpec::new("auto"), rate, buf.center, &buf.samples);
    let frames = |pk: &[common::Packet]| {
        pk.iter().filter(|p| matches!(p.body, PacketBody::Frame(_))).count()
    };
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
    let contents = decode::pocsag::encode(1_234_568, 3, &decode::pocsag::Body::Alpha("MOVE TO CHANNEL 2".into()));
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
        C32::new((r * (std::f64::consts::TAU * u2).cos()) as f32 * 0.02, (r * (std::f64::consts::TAU * u2).sin()) as f32 * 0.02)
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
    let frames: Vec<&common::Packet> =
        pk.iter().filter(|p| matches!(p.body, PacketBody::Frame(_))).collect();
    assert!(!frames.is_empty(), "no frame came out; packets: {}", pk.len());
    let f = frames[0];
    assert!((f.center_hz as f64 - (center.as_f64() + offset)).abs() < 5_000.0, "page at {}", f.center_hz);
    let PacketBody::Frame(bytes) = &f.body else { unreachable!() };
    let pages = nodes::pocsag_nodes::pocsag_decoded(bytes, Hz(f.center_hz));
    assert_eq!(pages.len(), 1, "{pages:?}");
    assert_eq!(pages[0].text.as_deref(), Some("MOVE TO CHANNEL 2"));
}

/// Run a stream through one stage and collect the events it emits.
fn events(stage: NodeSpec, rate: f64, center: Hz, iq: &[C32]) -> Vec<pipeline::event::Event> {
    let mut g = build_chain(StreamSpec::iq(rate, center), &[stage], &registry()).expect("build");
    let mut out = Vec::new();
    let silence = vec![C32::new(0.0, 0.0); 16_384];
    for block in iq.chunks(16_384).chain(std::iter::repeat(&silence[..]).take(8)) {
        out.extend_from_slice(g.feed_iq(block).expect("run"));
    }
    out
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
        C32::new((r * (std::f64::consts::TAU * u2).cos()) as f32 * 0.02, (r * (std::f64::consts::TAU * u2).sin()) as f32 * 0.02)
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
    let measured: Vec<(u64, &common::Measure)> =
        pk.iter().filter_map(|p| p.measure.as_ref().map(|m| (p.center_hz, m))).collect();
    let (hz, chirp) = measured.iter().find(|(_, m)| m.modulation == "chirp").unwrap_or_else(|| {
        panic!(
            "no chirp measurement among {:?}",
            measured.iter().map(|(_, m)| m.summary()).collect::<Vec<_>>()
        )
    });
    assert!(chirp.sweep_hz_s.abs() > 1e6, "{}", chirp.summary());
    assert!(chirp.bandwidth_hz > 60_000.0, "{}", chirp.summary());
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
    use dsp::m17::{fec, frame_symbols, preamble_symbols, Kind, BAUD, DEVIATION_HZ};

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
    let rows: Vec<pipeline::event::Decoded> = pk
        .iter()
        .filter_map(|p| match &p.body {
            PacketBody::Frame(b) => nodes::m17_nodes::m17_decoded(b, Hz(p.center_hz)),
            _ => None,
        })
        .collect();
    let setup = rows.iter().find(|d| d.protocol == "M17-Setup");
    assert!(setup.is_some(), "nothing read as M17; {} packets", pk.len());
    let from = setup
        .unwrap()
        .fields
        .iter()
        .find(|(k, _)| k == "from")
        .map(|(_, v)| v.to_string());
    assert_eq!(from.as_deref(), Some("M0ABC"));
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
    let dmr: Vec<_> = pk
        .iter()
        .filter_map(|p| match &p.body {
            PacketBody::Frame(b) => nodes::dmr_nodes::dmr_decoded(b, Hz(p.center_hz)),
            _ => None,
        })
        .collect();
    assert!(!dmr.is_empty(), "auto placed no DMR that decoded; {} packets", pk.len());
    let d = &dmr[0];
    assert_eq!(d.protocol, "DMR-Voice");
    // The over carried audio, so a call-list row would be playable.
    let carried = pk.iter().any(|p| p.audio.is_some());
    assert!(carried, "the DMR over reached the packet with no audio");
    eprintln!("auto decoded {} DMR row(s), audio present={carried}", dmr.len());
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
    if !p.exists() { eprintln!("skipping: {p:?} absent"); return; }
    let buf = sources::FileSource::open(&p).unwrap().read_all().unwrap();
    let iq: Vec<C32> = buf.samples.clone();
    let rate = 2_000_000.0;
    let center = Hz(869_525_000);
    let pk = packets(NodeSpec::new("auto"), rate, center, &iq);
    let lora: Vec<_> = pk.iter().filter_map(|p| match &p.body {
        PacketBody::Frame(b) => nodes::lora_nodes::lora_decoded(b, Hz(p.center_hz)),
        _ => None,
    }).collect();
    let chirps = pk.iter().filter(|p| p.modulation == Some("chirp")).count();
    eprintln!("auto: {} LoRa decoded, {} chirp rows, {} packets total", lora.len(), chirps, pk.len());
    assert!(!lora.is_empty(), "auto placed no LoRa that decoded; {} packets, {chirps} chirps", pk.len());
}






