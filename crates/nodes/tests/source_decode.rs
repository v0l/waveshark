//! Decode several simultaneous transmitters from one wideband stream, with
//! nothing told in advance about where they are or how wide.
//!
//! The mirror of `wideband_bank.rs`: the same recorded Fine Offset
//! transmission is frequency shifted to several offsets and summed, so the
//! stream genuinely holds four transmitters at once. The bank test placed
//! them on its channel grid. This one places them wherever, because there is
//! no grid: the detector has to find each one, measure it, cut it out at a
//! width that fits and hand it to a decoder that has never heard of the span.

use common::{Hz, C32};
use dsp::Mixer;
use nodes::{build_chain, registry, NodeSpec};
use pipeline::event::Event;
use sources::FileSource;

const FIXTURE: &str = "fineoffset_wh1080_433.92M_250k.cu8";
const RATE: f64 = 250_000.0;
const CENTER: Hz = Hz(433_920_000);
/// Offsets to put a transmitter at, in hertz from the stream centre.
///
/// Chosen to sit nowhere in particular: not on any grid, unevenly spaced,
/// and with the recording's own -10.5 kHz carrier offset riding along.
const OFFSETS: [f64; 4] = [-93_000.0, -37_000.0, 21_500.0, 78_000.0];
/// Where the recording's carrier sits relative to nominal: the peak of its
/// averaged spectrum, measured with numpy over the whole file. The bank test
/// quotes rtl_433's figure of -10.5 kHz, which does not match this file.
const CARRIER_OFFSET: f64 = 4_600.0;

fn fixture() -> Option<common::IqBuf> {
    let p = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../testdata")
        .join(FIXTURE);
    if !p.exists() {
        return None;
    }
    Some(FileSource::open(&p).ok()?.read_all().ok()?)
}

macro_rules! need_fixture {
    ($e:expr) => {
        match $e {
            Some(v) => v,
            None => {
                eprintln!("skipping: {FIXTURE} absent, run testdata/fetch.sh");
                return;
            }
        }
    };
}

/// Each copy starts this much later than the one before, in samples.
///
/// Four sensors keying up in the same quarter of a millisecond is not a
/// thing that happens, and the detector treats runs that open in the same
/// frame near each other as the two tones of one transmitter, which is a
/// thing that does. Independent devices have independent timing, and five
/// milliseconds is a small dose of it.
const STAGGER: usize = 1_250;

fn wideband(base: &[C32]) -> Vec<C32> {
    let mut out = vec![C32::new(0.0, 0.0); base.len() + STAGGER * OFFSETS.len()];
    for (k, &offset) in OFFSETS.iter().enumerate() {
        let mut m = Mixer::new(offset, RATE);
        let mut shifted = Vec::with_capacity(base.len());
        m.process(base, &mut shifted);
        for (o, s) in out[k * STAGGER..].iter_mut().zip(&shifted) {
            *o += *s;
        }
    }
    out
}

fn chain() -> Vec<NodeSpec> {
    vec![NodeSpec::new("source_detect"), NodeSpec::new("source_decode")]
}

/// Run the stream through in blocks the size a radio delivers, so sources
/// open, run and close across block boundaries.
fn run(wide: &[C32]) -> (Vec<Event>, Vec<common::Package>) {
    let spec = pipeline::StreamSpec::iq(RATE, CENTER);
    let mut g = build_chain(spec, &chain(), &registry()).expect("build chain");
    let mut events = Vec::new();
    let mut packages = Vec::new();
    for block in wide.chunks(16_384) {
        events.extend_from_slice(g.feed_iq(block).expect("run"));
        packages.extend_from_slice(g.output().as_pulses().unwrap_or(&[]));
    }
    // The last source's tail may still be draining: a stretch of silence
    // lets it close.
    let silence = vec![C32::new(0.0, 0.0); 16_384];
    for _ in 0..4 {
        events.extend_from_slice(g.feed_iq(&silence).expect("run"));
        packages.extend_from_slice(g.output().as_pulses().unwrap_or(&[]));
    }
    (events, packages)
}

#[test]
fn every_transmitter_is_found_where_it_is() {
    let buf = need_fixture!(fixture());
    let wide = wideband(&buf.samples);
    let (events, _) = run(&wide);

    let mut found: Vec<(f64, f64)> = events
        .iter()
        .filter_map(|e| match e {
            Event::Detection { center, bandwidth, .. } => {
                Some((center.as_f64() - CENTER.as_f64(), *bandwidth))
            }
            _ => None,
        })
        .collect();
    found.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());
    assert_eq!(found.len(), OFFSETS.len(), "one source per transmitter, got {found:?}");
    for (want, (got, bw)) in OFFSETS.iter().zip(&found) {
        let want = want + CARRIER_OFFSET;
        assert!((got - want).abs() < 4_000.0, "wanted {want}, found {got} ({bw} Hz wide); all {found:?}");
        assert!(*bw < 40_000.0, "a 4 kHz sensor measured {bw} Hz wide at {got}");
    }
}

#[test]
fn every_transmitter_decodes_through_its_own_stream() {
    let buf = need_fixture!(fixture());
    let wide = wideband(&buf.samples);
    let (events, packages) = run(&wide);
    for e in &events {
        if let Event::Detection { center, bandwidth, snr_db, at } = e {
            eprintln!("opened {:+.0} Hz {bandwidth:.0} wide {snr_db:.1} dB at {at:.3} s", center.as_f64() - CENTER.as_f64());
        }
    }
    assert!(!packages.is_empty(), "no bursts reached the pulse port");

    let protocols = decode::Protocols::all();
    let mut decoded: Vec<(f64, String)> = Vec::new();
    for p in &packages {
        eprintln!(
            "package at {:+.0} Hz: {} pulses, {:.1} dB, {:?}",
            p.center_hz as f64 - CENTER.as_f64(),
            p.pulses.len(),
            p.snr_db,
            p.modulation
        );
        let t: Vec<String> = p.pulses.iter().map(|q| format!("{}/{}", q.mark, q.gap)).collect();
        eprintln!("  start {} pulses {}", p.start_sample, t.join(" "));
        for r in protocols.decode_all(p) {
            if r.model.contains("WHx080") && r.crc_valid == Some(true) {
                decoded.push((p.center_hz as f64 - CENTER.as_f64(), r.to_string()));
            }
        }
    }
    decoded.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());
    decoded.dedup_by(|a, b| (a.0 - b.0).abs() < 4_000.0);
    assert_eq!(
        decoded.len(),
        OFFSETS.len(),
        "expected a decode from each transmitter, got {decoded:#?}"
    );
    for (want, (got, text)) in OFFSETS.iter().zip(&decoded) {
        let want = want + CARRIER_OFFSET;
        assert!((got - want).abs() < 4_000.0, "wanted {want}, decoded at {got}: {text}");
        assert!(text.contains("station_id=196"), "{text}");
        assert!(text.contains("temperature_c=16.2"), "{text}");
    }
}

#[test]
fn noise_alone_opens_nothing() {
    // Deterministic Gaussian noise at the level of the recording's own floor.
    let mut s = 0x9e3779b97f4a7c15u64;
    let mut next = move || {
        s ^= s << 13;
        s ^= s >> 7;
        s ^= s << 17;
        (s >> 11) as f64 / (1u64 << 53) as f64
    };
    let noise: Vec<C32> = (0..500_000)
        .map(|_| {
            let r = (-2.0 * next().max(1e-12).ln()).sqrt();
            let th = std::f64::consts::TAU * next();
            C32::new((r * th.cos()) as f32 * 0.01, (r * th.sin()) as f32 * 0.01)
        })
        .collect();
    let (events, packages) = run(&noise);
    let opened = events.iter().filter(|e| matches!(e, Event::Detection { .. })).count();
    assert_eq!(opened, 0, "noise opened {opened} sources");
    assert!(packages.is_empty(), "noise produced {} bursts", packages.len());
}
