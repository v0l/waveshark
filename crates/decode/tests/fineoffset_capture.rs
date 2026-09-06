//! Decode a real recorded transmission, end to end.
//!
//! This is the test that actually matters. Every other test in the workspace
//! checks our code against signals we also generated, which shares assumptions
//! between the test and the thing under test. Here the input is real RF
//! recorded off the air, and the expected values come from rtl_433 25.02, an
//! entirely independent implementation. Agreement is therefore evidence rather
//! than a tautology.
//!
//! The fixture is fetched by `testdata/fetch.sh`. When it is absent the test
//! skips rather than fails, so a fresh clone with no network still passes.

use decode::{Protocol, Protocols};
use decode::protocol::Value;
use decode::protocols::FineOffsetWh1080;
use dsp::{FirDecim, OokDetector, PulseConfig};
use sources::FileSource;

const FIXTURE: &str = "fineoffset_wh1080_433.92M_250k.cu8";

fn fixture_path() -> Option<std::path::PathBuf> {
    let p = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../testdata")
        .join(FIXTURE);
    p.exists().then_some(p)
}

fn packages() -> Option<Vec<dsp::Package>> {
    let path = fixture_path()?;
    let src = FileSource::open(&path).expect("open fixture");
    let buf = src.read_all().expect("read fixture");

    // Filter to roughly the signal's bandwidth before taking the envelope.
    // The transmission occupies about 4 kHz, so in the full 250 kHz span 98%
    // of the power in the envelope is noise and the signal sits only 12.5 dB
    // above it. Decimating by 8 first cuts the noise by 9 dB and takes the
    // detector from marginal to comfortable. Detecting on an unfiltered
    // wideband envelope is always the wrong thing to do.
    const DECIM: usize = 8;
    let mut dec = FirDecim::design(DECIM, 0.9, 80.0);
    let mut narrow = Vec::new();
    dec.process(&buf.samples, &mut narrow);
    let rate = buf.rate.as_f64() / DECIM as f64;
    let env: Vec<f32> = narrow.iter().map(|c| c.norm()).collect();

    // Fine Offset's inter-symbol gaps run near 1 ms, so the reset must be well
    // clear of that or one transmission is split into many packages.
    let cfg = PulseConfig { reset_us: 10_000, min_pulses: 20, ..Default::default() };
    let mut d = OokDetector::new(rate, cfg);
    let mut pkgs = Vec::new();
    d.process(&env, &mut pkgs);
    Some(pkgs)
}

macro_rules! skip_without_fixture {
    ($e:expr) => {
        match $e {
            Some(v) => v,
            None => {
                eprintln!(
                    "skipping: {FIXTURE} not present, run testdata/fetch.sh to enable"
                );
                return;
            }
        }
    };
}

#[test]
fn detects_exactly_one_transmission() {
    let pkgs = skip_without_fixture!(packages());
    assert_eq!(pkgs.len(), 1, "expected one package, got {}", pkgs.len());
    // 11 bytes plus a possible partial leading pulse.
    assert!(
        (88..=89).contains(&pkgs[0].pulses.len()),
        "expected 88 pulses for an 11-byte frame, got {}",
        pkgs[0].pulses.len()
    );
}

#[test]
fn measured_timings_match_the_published_protocol() {
    let pkgs = skip_without_fixture!(packages());
    let marks = pkgs[0].mark_histogram(150);
    let clusters: Vec<u32> = marks.iter().filter(|(_, n)| *n > 5).map(|(c, _)| *c).collect();
    assert_eq!(clusters.len(), 2, "expected two PWM symbol widths, got {marks:?}");

    // rtl_433 publishes 544 and 1524 us. Every envelope detector measures
    // short, because it thresholds partway up the pulse edge rather than at
    // its true start. Around 60 us of bias is normal and harmless, since the
    // slicer classifies against the midpoint. A much larger error would mean
    // the sample rate is wrong.
    assert!((450..=560).contains(&clusters[0]), "short symbol was {} us", clusters[0]);
    assert!((1420..=1540).contains(&clusters[1]), "long symbol was {} us", clusters[1]);

    let ratio = clusters[1] as f64 / clusters[0] as f64;
    assert!((2.6..3.2).contains(&ratio), "symbol ratio {ratio:.2}, expected about 2.8");
}

#[test]
fn decodes_and_agrees_with_rtl_433() {
    let pkgs = skip_without_fixture!(packages());
    let report = FineOffsetWh1080
        .decode_package(&pkgs[0])
        .expect("decode the real capture");

    // Ground truth, from: rtl_433 -r fineoffset_wh1080_433.92M_250k.cu8
    //   model: Fineoffset-WHx080  Station ID: 196  Battery: 1
    //   Temperature: 16.2 C  Humidity: 89 %  Wind Direction: 180
    //   Wind avg speed: 0.00  Wind gust: 0.00  Total rainfall: 84.3
    //   Integrity: CRC
    assert_eq!(report.model, "Fineoffset-WHx080");
    assert_eq!(report.crc_valid, Some(true), "CRC must verify on a real frame");
    assert_eq!(report.get("station_id"), Some(&Value::Int(196)));
    assert_eq!(report.get("temperature_c"), Some(&Value::Float(16.2)));
    assert_eq!(report.get("humidity_pct"), Some(&Value::Int(89)));
    assert_eq!(report.get("wind_direction_deg"), Some(&Value::Int(180)));
    assert_eq!(report.get("wind_avg_ms"), Some(&Value::Float(0.0)));
    assert_eq!(report.get("wind_gust_ms"), Some(&Value::Float(0.0)));
    assert_eq!(report.get("rain_total_mm"), Some(&Value::Float(84.3)));
    assert_eq!(report.get("battery_ok"), Some(&Value::Bool(true)));
}

#[test]
fn the_registry_finds_it_without_being_told_which_protocol() {
    // The actual use case: a burst arrives and every protocol is tried.
    let pkgs = skip_without_fixture!(packages());
    let reports = Protocols::all().decode_all(&pkgs[0]);
    assert_eq!(reports.len(), 1, "expected exactly one protocol to claim it");
    assert_eq!(reports[0].model, "Fineoffset-WHx080");
}
