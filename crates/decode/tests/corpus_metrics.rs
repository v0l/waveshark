//! How much of each capture we get, not merely whether we get it.
//!
//! `rtl433_corpus` asks whether every decode rtl_433 reported has a match
//! here. That question is answered "yes" for the whole corpus, which makes it
//! useless for judging a change to the detector or the slicers: a method that
//! doubled the number of frames recovered and one that halved it both pass.
//!
//! These two are the metrics with room in them.
//!
//! * *Yield* counts distinct decodes per capture against rtl_433's own count.
//!   A sensor transmits its message several times per burst, and recovering
//!   three repeats out of eight is three chances to hear a device rather than
//!   eight. rtl_433's reference JSON has one line per decode, so its count is
//!   directly comparable.
//! * *Noise floor* adds white noise to a capture until nothing decodes. Where
//!   that point sits is what "detection accuracy" means for a receiver left
//!   running on a band, and it is the number any change to the burst detector,
//!   the timing estimator or the coding guess should be judged against.
//!
//! * *False decodes* are the other half of accuracy and the half that a change
//!   aimed at recall will quietly spend. Two of them: what the decoders claim
//!   on a capture rtl_433 read as something else, and what they claim on pure
//!   noise, which is the band a receiver spends most of its life listening to.
//!   Both are split by whether the protocol carries an integrity check,
//!   because a fixed-code remote with no checksum will answer to almost
//!   anything by design and a CRC-backed claim is one a user is entitled to
//!   believe.
//!
//! All of them are printed rather than asserted, because a threshold invented
//! today would be a number nobody measured. Run them, write the numbers down,
//! change something, run them again.

mod corpus;

use common::C32;
use corpus::{fixtures, packages, Fixture};
use decode::protocol::Report;
use decode::Protocols;

/// Decodes recovered from a capture, deduplicated the way the corpus harness
/// does it.
fn decodes(f: &Fixture) -> Vec<Report> {
    f.decode()
}

#[test]
#[ignore]
fn yield_against_rtl_433() {
    let fixtures = fixtures();
    if fixtures.is_empty() {
        eprintln!("skipping: no rtl_433 fixtures, run testdata/fetch.sh to enable");
        return;
    }

    let (mut ours, mut theirs) = (0usize, 0usize);
    let mut rows: Vec<(String, usize, usize)> = Vec::new();
    for f in &fixtures {
        // Only the models we have a decoder for: a capture rtl_433 read as
        // three devices we do not implement is not a yield failure.
        let want = f.expected.len();
        let got = decodes(f).len();
        ours += got;
        theirs += want;
        rows.push((f.name.clone(), got, want));
    }
    rows.sort_by_key(|(_, got, want)| *got as i64 - *want as i64);
    println!("\n{:<44} {:>6} {:>8}", "capture", "here", "rtl_433");
    for (name, got, want) in &rows {
        println!("{name:<44} {got:>6} {want:>8}");
    }
    println!("\ntotal: {ours} decodes here against {theirs} in the reference");
}

/// Add white noise at a given signal-to-noise ratio, in dB.
///
/// The reference level is the capture's own mean power, so the number means
/// the same thing across captures recorded at different gains.
fn noisy(samples: &[C32], snr_db: f64, seed: u64) -> Vec<C32> {
    let p: f64 =
        samples.iter().map(|s| s.norm_sqr() as f64).sum::<f64>() / samples.len().max(1) as f64;
    let sigma = (p / 10f64.powf(snr_db / 10.0)).sqrt() as f32;
    // Box-Muller off a xorshift, so the figures reproduce exactly.
    let mut x = seed | 1;
    let mut next = || {
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        (x >> 11) as f32 / (1u64 << 53) as f32
    };
    samples
        .iter()
        .map(|s| {
            let (u1, u2): (f32, f32) = (next().max(1e-9), next());
            let r = sigma * (-2.0 * u1.ln()).sqrt();
            let (a, b) = (std::f32::consts::TAU * u2).sin_cos();
            C32::new(s.re + r * a, s.im + r * b)
        })
        .collect()
}

#[test]
#[ignore]
fn how_far_into_the_noise_each_capture_survives() {
    let fixtures = fixtures();
    if fixtures.is_empty() {
        eprintln!("skipping: no rtl_433 fixtures, run testdata/fetch.sh to enable");
        return;
    }
    let protocols = Protocols::all();
    // Three noise realisations per level, and every level tried rather than
    // stopping at the first failure. One draw decides a whole row otherwise,
    // and the row then moves by six decibels between runs of the same code,
    // which is more than any change being measured.
    const SEEDS: [u64; 3] = [0x9E3779B9, 0x517CC1B7, 0x2545F491];
    const LEVELS: [i32; 9] = [30, 24, 20, 16, 12, 9, 6, 3, 0];

    println!("\n{:<44} {:>10}  decoded of 3 at each level", "capture", "floor dB");
    let mut floors = Vec::new();
    for f in &fixtures {
        let src = sources::FileSource::open(&f.path).expect("open");
        let buf = src.read_all().expect("read");
        let dir = std::env::temp_dir().join(format!("sr-noise-{}", std::process::id()));
        std::fs::create_dir_all(&dir).expect("temp dir");
        let path = dir.join(format!(
            "noise_{:.6}M_{:.0}k.cu8",
            buf.center.as_f64() / 1e6,
            buf.rate.as_f64() / 1e3
        ));

        let mut row = String::new();
        let mut floor = None;
        for snr in LEVELS {
            let mut hits = 0;
            for seed in SEEDS {
                let dirty = noisy(&buf.samples, snr as f64, seed ^ snr as u64);
                write_cu8(&path, &dirty);
                let ok = packages(&path)
                    .iter()
                    .any(|pkg| protocols.decode_all(pkg).iter().any(|r| f.rtl_433_saw(r.model)));
                hits += ok as usize;
            }
            row += &format!("{hits}");
            // The floor is the lowest level that still decodes a majority of
            // the draws, so one lucky realisation cannot claim it.
            if hits >= 2 {
                floor = Some(snr);
            }
        }
        let _ = std::fs::remove_file(&path);
        match floor {
            Some(snr) => {
                println!("{:<44} {snr:>10}  {row}", f.name);
                floors.push(snr);
            }
            None => println!("{:<44} {:>10}  {row}", f.name, "none"),
        }
    }
    floors.sort_unstable();
    if !floors.is_empty() {
        println!(
            "\n{} of {} captures decode under noise, median floor {} dB, {} at 0 dB",
            floors.len(),
            fixtures.len(),
            floors[floors.len() / 2],
            floors.iter().filter(|s| **s == 0).count()
        );
    }
}

/// White noise at a given standard deviation per component, full scale 1.0.
///
/// 0.05 is what a receiver with its gain up sees on an empty band: well above
/// the converter's own floor and well below a real transmission.
fn white(n: usize, sigma: f32, seed: u64) -> Vec<C32> {
    let mut x = seed | 1;
    let mut next = || {
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        (x >> 11) as f32 / (1u64 << 53) as f32
    };
    (0..n)
        .map(|_| {
            let (u1, u2): (f32, f32) = (next().max(1e-9), next());
            let r = sigma * (-2.0 * u1.ln()).sqrt();
            let (a, b) = (std::f32::consts::TAU * u2).sin_cos();
            C32::new(r * a, r * b)
        })
        .collect()
}

/// The capture back out as a file, since the harness reads paths.
fn write_cu8(path: &std::path::Path, samples: &[C32]) {
    let mut bytes = Vec::with_capacity(samples.len() * 2);
    for s in samples {
        for v in [s.re, s.im] {
            bytes.push(((v * 127.5 + 127.5).clamp(0.0, 255.0)) as u8);
        }
    }
    std::fs::write(path, bytes).expect("write");
}


/// What the decoders claim on captures rtl_433 read as something else.
#[test]
#[ignore]
fn unverified_claims_in_the_corpus() {
    let fixtures = fixtures();
    if fixtures.is_empty() {
        eprintln!("skipping: no rtl_433 fixtures, run testdata/fetch.sh to enable");
        return;
    }
    let (mut checked, mut unchecked) = (0usize, 0usize);
    for f in &fixtures {
        for r in decodes(f) {
            if f.rtl_433_saw(r.model) {
                continue;
            }
            match r.crc_valid {
                Some(true) => {
                    checked += 1;
                    println!("{}: {} claims a passing check", f.name, r.model);
                }
                _ => {
                    unchecked += 1;
                    println!("{}: {} claims no check", f.name, r.model);
                }
            }
        }
    }
    println!(
        "\n{checked} claims with a passing integrity check, {unchecked} without, \
         across {} captures",
        fixtures.len()
    );
}

/// What the decoders claim when there is nothing there at all.
///
/// The number that matters for a receiver left running: a band with no traffic
/// on it should produce nothing, and whatever it does produce is the floor of
/// false reports the packet log will fill with.
#[test]
#[ignore]
fn false_decodes_on_pure_noise() {
    let protocols = Protocols::all();
    let rate = 250_000.0;
    let seconds = 60;
    let dir = std::env::temp_dir().join(format!("sr-fp-{}", std::process::id()));
    std::fs::create_dir_all(&dir).expect("temp dir");

    let mut by_model: std::collections::BTreeMap<String, (usize, usize)> = Default::default();
    for chunk in 0..seconds {
        let n = rate as usize;
        // Noise at the level a receiver with its gain up sees on a quiet band.
        // Absolute level, not a ratio: noise measured against a signal that
        // is not there is zero noise, which is silence and decodes nothing for
        // the wrong reason.
        let samples = white(n, 0.05, 0xA5A5 ^ chunk as u64);
        let path = dir.join(format!("noise_433.920000M_{:.0}k.cu8", rate / 1e3));
        write_cu8(&path, &samples);
        for pkg in packages(&path) {
            for r in protocols.decode_all(&pkg) {
                let e = by_model.entry(r.model.to_string()).or_default();
                if r.crc_valid == Some(true) {
                    e.0 += 1;
                } else {
                    e.1 += 1;
                }
            }
        }
    }
    let _ = std::fs::remove_dir_all(&dir);

    println!("\n{seconds} s of noise at 250 kS/s");
    println!("{:<28} {:>8} {:>10}", "model", "checked", "unchecked");
    let (mut c, mut u) = (0usize, 0usize);
    for (model, (checked, unchecked)) in &by_model {
        println!("{model:<28} {checked:>8} {unchecked:>10}");
        c += checked;
        u += unchecked;
    }
    println!(
        "\ntotal {c} checked and {u} unchecked false decodes, \
         {:.1} and {:.1} per minute",
        c as f64 * 60.0 / seconds as f64,
        u as f64 * 60.0 / seconds as f64
    );
}

/// Sanity: the noise harness has to produce bursts at all, or a zero false
/// decode count is measuring nothing.
#[test]
#[ignore]
fn the_noise_harness_actually_produces_bursts() {
    let dir = std::env::temp_dir().join(format!("sr-fpsan-{}", std::process::id()));
    std::fs::create_dir_all(&dir).expect("temp dir");
    for sigma in [0.02f32, 0.05, 0.15, 0.4] {
        let samples = white(250_000, sigma, 7);
        let path = dir.join("noise_433.920000M_250k.cu8");
        write_cu8(&path, &samples);
        let pkgs = packages(&path);
        let pulses: usize = pkgs.iter().map(|p| p.pulses.len()).sum();
        println!("sigma {sigma}: {} packages, {pulses} pulses", pkgs.len());
    }
    let _ = std::fs::remove_dir_all(&dir);
}


/// Why a capture stops decoding, rather than merely when.
///
/// Prints, per noise level, which rung of the harness ladder still detects the
/// burst and which still decodes it, so a change can be aimed at the stage
/// that is actually failing instead of the one that seems likely.
#[test]
#[ignore]
fn where_a_weak_capture_fails() {
    use dsp::{FirDecim, Mixer, OokDetector, PulseConfig};
    let want = std::env::var("SR_CAPTURE").unwrap_or_else(|_| "gtwt02_a".into());
    let Some(f) = fixtures().into_iter().find(|f| f.name.contains(&want)) else {
        eprintln!("skipping: no fixture matching {want}");
        return;
    };
    let src = sources::FileSource::open(&f.path).expect("open");
    let buf = src.read_all().expect("read");
    let protocols = Protocols::all();
    let rate = buf.rate.as_f64();
    let offset = corpus::carrier_offset(&buf.samples, rate);
    println!("\n{} (carrier {:+.0} Hz off centre)", f.name, offset);

    for snr in [30, 20, 16, 12, 9, 6, 3] {
        let dirty = noisy(&buf.samples, snr as f64, 0x9E3779B9 ^ snr as u64);
        let mut line = format!("{snr:>3} dB:");
        for centred in [false, true] {
            let iq = if centred {
                let mut v = Vec::with_capacity(dirty.len());
                Mixer::new(-offset, rate).process(&dirty, &mut v);
                v
            } else {
                dirty.clone()
            };
            for decim in [1usize, 2, 8] {
                let mut narrow = Vec::new();
                FirDecim::design(decim, 0.9, 80.0).process(&iq, &mut narrow);
                let r = rate / decim as f64;
                let env: Vec<f32> = narrow.iter().map(|c| c.norm()).collect();
                let mut det =
                    OokDetector::new(r, PulseConfig { min_pulses: 8, ..Default::default() });
                let mut pkgs = Vec::new();
                det.process(&env, &mut pkgs);
                let ok = pkgs
                    .iter()
                    .any(|pkg| protocols.decode_all(pkg).iter().any(|r| f.rtl_433_saw(r.model)));
                let tag = if ok { "decode" } else if !pkgs.is_empty() { "burst" } else { "-" };
                line += &format!("  {}{}:{:<6}", if centred { "mix/" } else { "raw/" }, decim, tag);
            }
        }
        println!("{line}");
    }
}

/// What the pulses themselves look like as a capture is buried.
///
/// Detection and decoding fail for different reasons, and the fix for one is
/// no use against the other. This shows which: if the pulse count holds and
/// the widths spread, the bits are being mistimed; if the count collapses, the
/// burst is coming apart.
#[test]
#[ignore]
fn how_the_pulses_degrade() {
    use dsp::{OokDetector, PulseConfig};
    let want = std::env::var("SR_CAPTURE").unwrap_or_else(|_| "gtwt02_a".into());
    let Some(f) = fixtures().into_iter().find(|f| f.name.contains(&want)) else {
        eprintln!("skipping: no fixture matching {want}");
        return;
    };
    let src = sources::FileSource::open(&f.path).expect("open");
    let buf = src.read_all().expect("read");
    println!("\n{}", f.name);
    for snr in [40, 30, 24, 20, 16, 12] {
        let dirty = noisy(&buf.samples, snr as f64, 0x9E3779B9 ^ snr as u64);
        // Filtered even at a factor of one: it trims the band edges, where the
        // dongle's worst noise lives, and on several captures that alone is
        // the difference between detecting and not.
        let mut narrow = Vec::new();
        dsp::FirDecim::design(1, 0.9, 80.0).process(&dirty, &mut narrow);
        let env: Vec<f32> = narrow.iter().map(|c| c.norm()).collect();
        let mut det = OokDetector::new(
            buf.rate.as_f64(),
            PulseConfig { min_pulses: 8, ..Default::default() },
        );
        let mut pkgs = Vec::new();
        det.process(&env, &mut pkgs);
        let Some(p) = pkgs.iter().max_by_key(|p| p.pulses.len()) else {
            println!("{snr:>3} dB: no burst");
            continue;
        };
        let marks: Vec<u32> = p.pulses.iter().map(|x| x.mark).collect();
        let gaps: Vec<u32> = p.pulses.iter().map(|x| x.gap).take(p.pulses.len() - 1).collect();
        let spread = |v: &[u32]| -> String {
            let mut s = v.to_vec();
            s.sort_unstable();
            if s.is_empty() {
                return "-".into();
            }
            format!("{}/{}/{}", s[0], s[s.len() / 2], s[s.len() - 1])
        };
        println!(
            "{snr:>3} dB: {:>3} pulses  marks min/med/max {:<16} gaps {:<16} rejoined {}",
            p.pulses.len(),
            spread(&marks),
            spread(&gaps),
            det.stats().rejoined_marks
        );
    }
}
