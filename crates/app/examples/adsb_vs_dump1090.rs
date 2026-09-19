//! Read the same tuner two ways and compare what each decoder got.
//!
//! dump1090 owns the dongle and publishes AVR lines on port 30002; an
//! iqstream server on the same machine fans the samples out, so this reads
//! the identical radio at the identical moment and the only difference is the
//! decoder. Anything else would compare two antennas.
//!
//! ```text
//! cargo run --release -p app --example adsb_vs_dump1090 -- 10.100.2.249
//! cargo run --release -p app --example adsb_vs_dump1090 -- 10.100.2.249 --seconds 300 --iq 1234
//! ```
//!
//! The verdict is the matched fraction each way. A frame counts as matched
//! when the other side reported the same bytes within [`WINDOW`], since
//! neither side timestamps at the antenna and dump1090's AVR output has no
//! timestamp at all.

use common::{C32, Device as _, Hz};
use decode::adsb::{self, AddressBook};
use dsp::{ModeSConfig, ModeSDetector, ModeSFrame};
use std::collections::HashMap;
use std::io::{BufRead, BufReader};
use std::net::TcpStream;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

/// How far apart two reports of the same bytes may be and still be the same
/// transmission
///
/// Generous on purpose: dump1090 writes a line when it has it and the two
/// pipelines buffer differently. An aircraft repeating a frame inside this is
/// rare enough not to matter, and both sides see the repeat anyway.
const WINDOW: Duration = Duration::from_secs(4);

fn main() {
    let mut args = std::env::args().skip(1);
    let host = args.next().unwrap_or_else(|| {
        eprintln!("usage: adsb_vs_dump1090 <host> [--seconds N] [--avr PORT] [--iq PORT]");
        std::process::exit(2);
    });
    let (mut seconds, mut avr_port, mut iq_port) = (60.0f64, 30002u16, 1234u16);
    while let Some(flag) = args.next() {
        let v = args.next().unwrap_or_default();
        match flag.as_str() {
            "--seconds" => seconds = v.parse().expect("--seconds N"),
            "--avr" => avr_port = v.parse().expect("--avr PORT"),
            "--iq" => iq_port = v.parse().expect("--iq PORT"),
            other => panic!("unknown flag {other}"),
        }
    }

    let stop = Arc::new(AtomicBool::new(false));
    let dump = Arc::new(Mutex::new(Vec::<Seen>::new()));
    let avr = {
        let (addr, stop, out) = (format!("{host}:{avr_port}"), stop.clone(), dump.clone());
        std::thread::spawn(move || read_avr(&addr, stop, out))
    };

    let ours = read_iqstream(&format!("{host}:{iq_port}"), seconds);
    stop.store(true, Ordering::Relaxed);
    let _ = avr.join();
    let theirs = std::mem::take(&mut *dump.lock().unwrap());

    report(&theirs, &ours);
}

/// One frame, as bytes and when it was seen here
struct Seen {
    hex: String,
    df: u8,
    icao: Option<u32>,
    /// Whether that address was checked rather than assumed
    ///
    /// DF17 and DF18 carry the address in the frame and the parity over it,
    /// so a corrupt one does not decode. DF0, 4, 5, 16, 20 and 21 carry the
    /// address only as parity overlay: reading one back is a guess, both
    /// decoders make it, and counting those as aircraft invents traffic.
    known: bool,
    /// The address a reply that cannot check itself claims
    ///
    /// The parity of DF0, 4, 5, 16, 20 and 21 is the address XORed over it, so
    /// any 56 or 112 bits yield one. It is evidence only against the
    /// addresses something else proved.
    overlaid: Option<u32>,
    /// Whether the frame proves itself, whoever reported it
    ///
    /// The parity comes to zero on DF17 and DF18, and on a DF11 answering an
    /// all-call. Everything else is a frame somebody decided to believe, so
    /// counting those as decodes compares two policies rather than two
    /// demodulators.
    provable: bool,
    at: Instant,
}

impl Seen {
    fn new(bytes: &[u8], at: Instant) -> Self {
        let hex = bytes.iter().map(|b| format!("{b:02x}")).collect();
        let df = bytes[0] >> 3;
        Self {
            hex,
            df,
            icao: adsb::parse(bytes).ok().and_then(|f| f.icao),
            known: matches!(df, 17 | 18),
            overlaid: (!matches!(df, 11 | 17 | 18))
                .then(|| adsb::overlaid_address(bytes))
                .flatten(),
            provable: matches!(df, 11 | 17 | 18) && decode::adsb::crc24(bytes) == 0,
            at,
        }
    }
}

/// dump1090's AVR output: one `*8d4840d6...;` a line, nothing else in it.
fn read_avr(addr: &str, stop: Arc<AtomicBool>, out: Arc<Mutex<Vec<Seen>>>) {
    let sock = match TcpStream::connect(addr) {
        Ok(s) => s,
        Err(e) => {
            eprintln!("avr {addr}: {e}");
            return;
        }
    };
    // So the loop notices the stop flag between frames on a quiet band.
    let _ = sock.set_read_timeout(Some(Duration::from_millis(500)));
    let mut lines = BufReader::new(sock).lines();
    while !stop.load(Ordering::Relaxed) {
        let Some(line) = lines.next() else { break };
        let Ok(line) = line else { continue };
        let t = line.trim();
        let Some(body) = t.strip_prefix('*').and_then(|s| s.strip_suffix(';')) else { continue };
        // An MLAT line carries a timestamp prefix and is not a frame.
        let Some(bytes) = unhex(body) else { continue };
        if !matches!(bytes.len(), 7 | 14) {
            continue;
        }
        out.lock().unwrap().push(Seen::new(&bytes, Instant::now()));
    }
}

fn unhex(s: &str) -> Option<Vec<u8>> {
    (s.len() % 2 == 0)
        .then(|| {
            (0..s.len() / 2).map(|i| u8::from_str_radix(&s[i * 2..i * 2 + 2], 16).ok()).collect()
        })
        .flatten()
}

/// Our own decoder over the same tuner, wired exactly as `ModesNode` wires it.
fn read_iqstream(addr: &str, seconds: f64) -> Vec<Seen> {
    let mut dev = match remote::iqstream::Device::open(addr) {
        Ok(d) => d,
        Err(e) => {
            eprintln!("iqstream {addr}: {e}");
            std::process::exit(1);
        }
    };
    let rate = dev.info().rates[0].0 as f64;
    println!(
        "iqstream {addr}: {} at {:.4} MHz, {:.3} MS/s",
        dev.info().label,
        dev.center().0 as f64 / 1e6,
        rate / 1e6
    );
    if (dev.center().0 as f64 - 1_090_000_000.0).abs() > 1e6 {
        // A pinned server ignores this, which is the case worth saying out
        // loud: the comparison is only fair on the band dump1090 is reading.
        let _ = dev.set_center(Hz(1_090_000_000));
    }
    let mut stream = dev.start_rx().expect("the server starts a subscription");

    // The detector's thresholds, so a run can ask what a looser search would
    // have found without a rebuild: MODES_RATIO, MODES_LEVEL, MODES_CRC.
    let mut cfg = ModeSConfig::default();
    if let Some(v) = std::env::var("MODES_RATIO").ok().and_then(|v| v.parse().ok()) {
        cfg.preamble_ratio = v;
    }
    if let Some(v) = std::env::var("MODES_LEVEL").ok().and_then(|v| v.parse().ok()) {
        cfg.min_level = v;
    }
    if let Ok(v) = std::env::var("MODES_CRC") {
        cfg.crc_framing = v != "0";
    }
    println!(
        "detector: ratio {:.2}, level {:.4}, crc framing {}",
        cfg.preamble_ratio, cfg.min_level, cfg.crc_framing
    );
    let mut det = ModeSDetector::new(rate, cfg);
    let mut book = AddressBook::new();
    let mut frames = Vec::new();
    let mut out = Vec::new();
    let mut samples = 0u64;

    let began = Instant::now();
    while began.elapsed().as_secs_f64() < seconds {
        let Ok(buf) = stream.read() else { break };
        samples += buf.samples.len() as u64;
        let iq: &[C32] = &buf.samples;
        frames.clear();
        let held = std::cell::RefCell::new(std::mem::take(&mut book));
        det.process_valid(iq, &mut frames, &|f: &ModeSFrame| {
            held.borrow_mut().accept(&f.bytes)
        });
        book = held.into_inner();
        let at = Instant::now();
        for f in &frames {
            let bytes = match f.bytes[0] >> 3 {
                17 | 18 => adsb::fix_single_bit(&f.bytes).unwrap_or_else(|| f.bytes.clone()),
                _ => f.bytes.clone(),
            };
            // The node emits a frame only where it parses, so neither does
            // this: a frame the receiver would throw away is not a decode.
            if adsb::parse(&bytes).is_ok() {
                out.push(Seen::new(&bytes, at));
            }
        }
    }
    let el = began.elapsed().as_secs_f64();
    println!(
        "read {:.1} s of samples in {el:.1} s ({:.3} MS/s, {} dropped)\n",
        samples as f64 / rate,
        samples as f64 / el / 1e6,
        stream.dropped()
    );
    out
}

/// Match each side's frames against the other and print the difference.
fn report(theirs: &[Seen], ours: &[Seen]) {
    let matched_ours = matched(ours, theirs);
    let matched_theirs = matched(theirs, ours);
    let pct = |a: usize, b: usize| if b == 0 { 0.0 } else { 100.0 * a as f64 / b as f64 };

    println!("{:<12} {:>8} {:>8} {:>8} {:>10}", "", "frames", "matched", "only", "aircraft");
    for (name, all, m) in
        [("dump1090", theirs, &matched_theirs), ("waveshark", ours, &matched_ours)]
    {
        println!(
            "{name:<12} {:>8} {:>7.1}% {:>8} {:>10}",
            all.len(),
            pct(m.iter().filter(|x| **x).count(), all.len()),
            m.iter().filter(|x| !**x).count(),
            aircraft(all)
        );
    }

    // What each side can prove, which is the only comparison of demodulators
    // rather than of how much unverifiable traffic each is willing to
    // publish. dump1090 emits DF0, 4, 5, 16, 20 and 21 whose parity is an
    // address it cannot check, and DF11 whose parity comes to neither zero
    // nor an interrogator id.
    let (tp, op): (Vec<&Seen>, Vec<&Seen>) = (
        theirs.iter().filter(|s| s.provable).collect(),
        ours.iter().filter(|s| s.provable).collect(),
    );
    println!(
        "\nparity comes to zero: dump1090 {} of {} ({:.1}%), waveshark {} of {} ({:.1}%)",
        tp.len(),
        theirs.len(),
        pct(tp.len(), theirs.len()),
        op.len(),
        ours.len(),
        pct(op.len(), ours.len())
    );

    println!("\nby downlink format, and of those the ones whose parity comes to zero");
    println!(
        "{:<6} {:>9} {:>9} {:>9} {:>9} {:>9} {:>9}",
        "df", "dump1090", "ours", "we missed", "we only", "their ok", "our ok"
    );
    let mut dfs: Vec<u8> = theirs.iter().chain(ours).map(|s| s.df).collect();
    dfs.sort_unstable();
    dfs.dedup();
    for df in dfs {
        let t = theirs.iter().filter(|s| s.df == df).count();
        let o = ours.iter().filter(|s| s.df == df).count();
        let missed = zip_count(theirs, &matched_theirs, df);
        let extra = zip_count(ours, &matched_ours, df);
        let tok = theirs.iter().filter(|s| s.df == df && s.provable).count();
        let ook = ours.iter().filter(|s| s.df == df && s.provable).count();
        println!("{df:<6} {t:>9} {o:>9} {missed:>9} {extra:>9} {tok:>9} {ook:>9}");
    }

    // What confidence is available for the replies neither decoder can
    // check. Corroboration against the addresses ADS-B proved is the obvious
    // test and it answers nothing: measured here, near enough none of either
    // side's overlaid replies name an aircraft that also broadcast a position
    // in the same window, because an aircraft answering a radar is usually
    // not the one broadcasting. What is left is repetition. A frame read out
    // of noise yields a uniformly random address that never comes back, so
    // the share of frames belonging to an address seen many times is the
    // share that is real, and the addresses seen once are the invented ones.
    let proved: std::collections::HashSet<u32> =
        icaos(theirs).keys().chain(icaos(ours).keys()).copied().collect();
    println!(
        "\noverlaid replies, by how often the address they claim comes back \
         ({} addresses were proved by a parity check)",
        proved.len()
    );
    println!(
        "{:<12} {:>8} {:>9} {:>10} {:>10}",
        "", "replies", "addresses", "seen once", "seen 10+"
    );
    for (name, all) in [("dump1090", theirs), ("waveshark", ours)] {
        let mut claims: HashMap<u32, usize> = HashMap::new();
        for a in all.iter().filter_map(|s| s.overlaid) {
            *claims.entry(a).or_default() += 1;
        }
        let replies: usize = claims.values().sum();
        let once = claims.values().filter(|n| **n == 1).count();
        let solid: usize = claims.values().filter(|n| **n >= 10).sum();
        println!(
            "{name:<12} {replies:>8} {:>9} {once:>10} {:>9.1}%",
            claims.len(),
            pct(solid, replies)
        );
    }

    // Who each side saw is the answer that matters to somebody watching the
    // map: a decoder reporting fewer frames but the same aircraft has lost
    // repeats, and one missing aircraft has lost coverage. Off the checkable
    // addresses alone, so a bit error in a surveillance reply cannot add an
    // aeroplane to either column.
    let (ta, oa) = (icaos(theirs), icaos(ours));
    let missing: Vec<String> =
        ta.keys().filter(|k| !oa.contains_key(*k)).map(|i| format!("{i:06x}")).collect();
    let extra: Vec<String> =
        oa.keys().filter(|k| !ta.contains_key(*k)).map(|i| format!("{i:06x}")).collect();
    println!("\naircraft dump1090 saw and we did not: {}", list(&missing));
    println!("aircraft we saw and dump1090 did not: {}", list(&extra));
}

/// For each frame on one side, whether the other side reported the same bytes
/// inside [`WINDOW`].
///
/// Each report on the far side is consumed by at most one match, so a run of
/// repeats compares as a run rather than every one of them matching the same
/// single report on the other side.
fn matched(these: &[Seen], those: &[Seen]) -> Vec<bool> {
    let mut by_hex: HashMap<&str, Vec<(Instant, bool)>> = HashMap::new();
    for s in those {
        by_hex.entry(&s.hex).or_default().push((s.at, false));
    }
    these
        .iter()
        .map(|s| {
            let Some(list) = by_hex.get_mut(s.hex.as_str()) else { return false };
            let near = list.iter_mut().find(|(at, taken)| {
                !*taken && (if *at > s.at { *at - s.at } else { s.at - *at }) < WINDOW
            });
            match near {
                Some((_, taken)) => {
                    *taken = true;
                    true
                }
                None => false,
            }
        })
        .collect()
}

fn zip_count(all: &[Seen], m: &[bool], df: u8) -> usize {
    all.iter().zip(m).filter(|(s, ok)| s.df == df && !**ok).count()
}

fn icaos(all: &[Seen]) -> HashMap<u32, usize> {
    let mut out: HashMap<u32, usize> = HashMap::new();
    for s in all.iter().filter(|s| s.known).filter_map(|s| s.icao) {
        *out.entry(s).or_default() += 1;
    }
    out
}

fn aircraft(all: &[Seen]) -> usize {
    icaos(all).len()
}

fn list(v: &[String]) -> String {
    match v.is_empty() {
        true => "none".into(),
        false => v.join(" "),
    }
}
