//! Demodulate and decode real 1090 MHz RF, and agree with dump1090 about it.
//!
//! The synthetic tests in `dsp::modes` and `decode::adsb` check the two halves
//! against signals and frames this project generated itself, which shares
//! every assumption between the test and the code. Here the input is four
//! seconds of recorded band and the expected output came out of dump1090-rb
//! 1.0.15 run over the same file, so agreement means something.
//!
//! Two properties matter, and they pull in opposite directions. Missing frames
//! is a sensitivity problem and costs coverage. Inventing frames is worse: an
//! aircraft that does not exist, at an altitude nobody is at, is the kind of
//! output that makes a receiver untrustworthy rather than merely deaf. So the
//! yield assertion has slack in it and the false positive assertion has none.
//!
//! The fixture is fetched by `testdata/fetch.sh`. When it is absent the test
//! skips rather than fails, so a fresh clone with no network still passes.

use decode::adsb::{self, AddressBook};
use dsp::{ModeSConfig, ModeSDetector, ModeSFrame};
use std::collections::HashSet;
use std::sync::LazyLock;

const FIXTURE: &str = "adsb_1090M_2400k.cu8";
const REFERENCE: &str = "adsb_1090M_2400k.dump1090.hex";
const RATE: f64 = 2_400_000.0;

fn testdata(name: &str) -> std::path::PathBuf {
    std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../testdata").join(name)
}

/// Samples handed to the detector at a time, the way a radio delivers them,
/// so the buffer boundary is part of what is being tested.
const BLOCK: usize = 65_536;

/// The capture, demodulated once for the whole file.
///
/// Each test asks a different question about the same four seconds, and cargo
/// runs them in parallel, so demodulating inside each one ran the search three
/// times over and held three copies of the capture at once. The work is
/// identical, so it happens once and the tests read the frames.
static FRAMES: LazyLock<Option<Vec<String>>> = LazyLock::new(|| decode(ModeSConfig::default()));

/// The same capture with the CRC framing pass switched off, so what that pass
/// is worth can be stated as a difference rather than asserted on trust.
static PREAMBLE_ONLY: LazyLock<Option<Vec<String>>> =
    LazyLock::new(|| decode(ModeSConfig { crc_framing: false, ..ModeSConfig::default() }));

/// What dump1090 made of the same file.
fn reference() -> HashSet<String> {
    std::fs::read_to_string(testdata(REFERENCE))
        .expect("reference decode is committed, unlike the capture")
        .lines()
        .map(|l| l.trim().to_string())
        .filter(|l| !l.is_empty())
        .collect()
}

/// Every frame the receiver believes, as hex.
fn decode(cfg: ModeSConfig) -> Option<Vec<String>> {
    let raw = std::fs::read(testdata(FIXTURE)).ok()?;
    let mut d = ModeSDetector::new(RATE, cfg);
    let book = std::cell::RefCell::new(AddressBook::new());
    let mut frames = Vec::new();
    // Converted a block at a time rather than all at once: the detector reads
    // the capture in blocks anyway, so building the whole thing as complex
    // floats first only costs memory.
    let mut block: Vec<common::C32> = Vec::with_capacity(BLOCK);
    for bytes in raw.chunks(BLOCK * 2) {
        block.clear();
        block.extend(bytes.chunks_exact(2).map(|c| {
            common::C32::new((c[0] as f32 - 127.5) / 127.5, (c[1] as f32 - 127.5) / 127.5)
        }));
        d.process_valid(&block, &mut frames, &|f: &ModeSFrame| {
            book.borrow_mut().accept(&f.bytes, f.weak_bits == 0)
        });
    }
    Some(
        frames
            .iter()
            .map(|f| {
                let fixed = match f.bytes[0] >> 3 {
                    17 | 18 => adsb::fix_single_bit(&f.bytes).unwrap_or_else(|| f.bytes.clone()),
                    _ => f.bytes.clone(),
                };
                fixed.iter().map(|b| format!("{b:02x}")).collect()
            })
            .collect(),
    )
}

macro_rules! skip_without_fixture {
    ($e:expr_2021) => {
        match $e {
            Some(v) => v,
            None => {
                eprintln!("skipping: {FIXTURE} not present, run testdata/fetch.sh to enable");
                return;
            }
        }
    };
}

#[test]
fn every_frame_we_report_is_one_dump1090_also_saw() {
    let ours: HashSet<String> = skip_without_fixture!(FRAMES.as_ref()).iter().cloned().collect();
    let theirs = reference();
    let invented: Vec<&String> = ours.difference(&theirs).collect();
    assert!(
        invented.is_empty(),
        "{} frames nobody else saw: {:?}",
        invented.len(),
        &invented[..invented.len().min(5)]
    );
}

#[test]
fn most_of_what_dump1090_found_is_found_here_too() {
    let ours: HashSet<String> = skip_without_fixture!(FRAMES.as_ref()).iter().cloned().collect();
    let theirs = reference();
    let matched = ours.intersection(&theirs).count();
    // dump1090 recovers a few more through two-bit error correction and
    // interrogator-id guessing, neither of which is implemented here, so the
    // bar is most rather than all. It was 27 of 40 when this was written, and
    // 29 once frames were also framed by their CRC.
    assert!(matched >= 29, "matched only {matched} of {} reference frames", theirs.len());
}

#[test]
fn framing_by_the_crc_reads_frames_the_preamble_search_never_sees() {
    // The whole claim of the second pass: a window whose parity comes to zero
    // is a frame wherever it sits, so a transmission whose preamble another
    // aircraft sat on is still readable. Both counts are of distinct frames
    // dump1090 also saw, over the same four seconds.
    let theirs = reference();
    let with: HashSet<String> = skip_without_fixture!(FRAMES.as_ref()).iter().cloned().collect();
    let without: HashSet<String> =
        skip_without_fixture!(PREAMBLE_ONLY.as_ref()).iter().cloned().collect();
    assert_eq!(without.intersection(&theirs).count(), 27, "preamble search alone");
    assert_eq!(with.intersection(&theirs).count(), 29, "with CRC framing");
    // Two of them: a DF17 velocity report and a DF11 all-call reply.
    let extra: Vec<&String> = with.difference(&without).collect();
    assert_eq!(extra.len(), 2, "extra frames: {extra:?}");
    assert!(extra.iter().all(|f| theirs.contains(*f)), "invented {extra:?}");
    assert!(without.difference(&with).count() == 0, "the second pass lost a frame");
}

#[test]
fn the_second_pass_costs_a_fraction_of_the_time_the_capture_covers() {
    // It sits in the hot path of a wide span, so what it costs matters as much
    // as what it finds. Measured on this file: the second pass adds 0.14 s to
    // the four seconds of 2.4 MS/s the capture holds, about 3.5% of real time.
    // The ceiling is a fifth of real time, loose enough for a slow or loaded
    // machine and tight enough to catch the pass being made an order of
    // magnitude dearer.
    let seconds = || -> Option<f64> {
        let t = std::time::Instant::now();
        decode(ModeSConfig::default())?;
        let both = t.elapsed().as_secs_f64();
        let t = std::time::Instant::now();
        decode(ModeSConfig { crc_framing: false, ..ModeSConfig::default() })?;
        Some(both - t.elapsed().as_secs_f64())
    };
    let added = skip_without_fixture!(seconds());
    // Four seconds of signal in the file.
    assert!(added < 0.8, "the CRC pass added {added:.3} s to four seconds of capture");
}

#[test]
fn the_aircraft_in_the_capture_decodes_to_a_position_and_a_callsign() {
    // The end of the pipeline, on real RF: bytes to something a map could use.
    let frames = skip_without_fixture!(FRAMES.as_ref());
    let mut altitude = None;
    let mut position = None;
    let mut callsign = None;
    let mut icaos = HashSet::new();
    for hex in frames {
        let bytes: Vec<u8> = (0..hex.len() / 2)
            .map(|i| u8::from_str_radix(&hex[i * 2..i * 2 + 2], 16).unwrap())
            .collect();
        let Ok(f) = adsb::parse(&bytes) else { continue };
        if let Some(icao) = f.icao {
            icaos.insert(icao);
        }
        match f.kind {
            adsb::Message::AirbornePosition { altitude_ft, odd, lat_cpr, lon_cpr } => {
                altitude = altitude_ft.or(altitude);
                position = Some((lat_cpr, lon_cpr, odd));
            }
            adsb::Message::Identification { callsign: c, .. } => callsign = Some(c),
            _ => {}
        }
    }

    assert!(icaos.contains(&0x4b1880), "expected the aircraft dump1090 saw, got {icaos:x?}");
    let alt = altitude.expect("an altitude");
    assert!((30_000..=40_000).contains(&alt), "altitude {alt} ft is not a cruise level");

    assert_eq!(callsign.as_deref(), Some("SWR14V"), "the flight dump1090 also identified");

    // Only odd-parity position frames survived in this window, in dump1090's
    // decode as well as ours, so there is no pair to resolve globally. That is
    // the normal case rather than a defect: a receiver that has any idea where
    // it is decodes a single frame against its own position, which is what
    // this does. The reference is the recording site.
    const HERE: (f64, f64) = (53.64, -6.65);
    let (lat_cpr, lon_cpr, odd) = position.expect("a position frame");
    let (lat, lon) = adsb::cpr_local(HERE, (lat_cpr, lon_cpr), odd);
    let (dlat, dlon) = (lat - HERE.0, lon - HERE.1);
    // Within a couple of hundred kilometres, which is as far as an aircraft at
    // 36000 feet can be and still be heard on a telescopic antenna indoors.
    let km = ((dlat * 111.0).powi(2) + (dlon * 111.0 * HERE.0.to_radians().cos()).powi(2)).sqrt();
    assert!(km < 250.0, "aircraft resolved to {lat:.4},{lon:.4}, {km:.0} km away");
}
