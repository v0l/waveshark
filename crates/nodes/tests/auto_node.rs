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
