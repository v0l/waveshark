//! WaveShark.
//!
//! On Windows a release build is a windows-subsystem binary, so double
//! clicking it opens the receiver and not a console behind it. A debug build
//! keeps the console, which is where its logging goes, and the command line
//! options still work either way: a console application started from a
//! terminal writes to it, and this one attaches to the parent console when
//! there is one, so `--help` and the offline tools still print.
#![cfg_attr(all(target_os = "windows", not(debug_assertions)), windows_subsystem = "windows")]

use clap::{Parser, ValueEnum};
use std::path::PathBuf;

/// The mark the window manager, the dock and the task bar draw.
///
/// Compiled in rather than read from a path beside the binary: the release
/// archives hold an executable and a text file, so anything looked up at run
/// time is missing on every machine but a checkout.
fn window_icon() -> Option<egui::IconData> {
    let png = include_bytes!("../../../assets/logo/waveshark-icon.png");
    let img = image::load_from_memory(png).ok()?.into_rgba8();
    let (width, height) = img.dimensions();
    Some(egui::IconData { rgba: img.into_raw(), width, height })
}

#[cfg(feature = "mcp")]
mod agent;
mod audiobus;
mod bands;
mod beacondb;
mod calls;
mod chain;
mod chainview;
mod control;
mod data;
mod devices;
mod dial;
mod i18n;
mod icons;
mod keystore;
mod links;
mod locale;
mod map;
mod memory;
mod meshnode;
mod messages;
mod packetlog;
mod patch;
mod prof;
mod radio;
mod record;
mod sats;
mod scanners;
mod session;
mod shutdown;
mod station;
mod theme;
mod tracks;
mod transcripts;
mod ui;
mod update;
mod videobus;
mod waterfall;
mod wheel;

/// `--probe <mhz>` runs the radio thread without a window and reports what the
/// waterfall would be drawing, so the signal path can be checked over ssh.
/// Report what the squelch is measuring on a frequency, for setting one.
///
/// The threshold has to sit above the reading on an empty channel and below
/// the reading on a signal, and neither number can be guessed from a bench
/// test: the IF filter limits how much noise there is to measure, so the
/// figures depend on the mode's own bandwidth.
fn squelch_probe(mhz: f64, mode: radio::Demod) {
    use common::{Hz, Sps};
    let Some(entry) = devices::list().into_iter().next() else {
        println!("no radio found");
        return;
    };
    let r = radio::Radio::start(entry, Hz((mhz * 1e6) as u64), Sps(2_304_000), 2048, || {});
    r.send(radio::Cmd::Channels(vec![radio::ChannelSpec {
        id: 1,
        label: String::new(),
        offset_hz: 0.0,
        mode: radio::ChanMode::Audio(mode),
        bandwidth_hz: None,
        volume: 1.0,
        // Measuring, not listening: the numbers are the same either way and
        // this can be run over ssh.
        muted: true,
        squelch_db: None,
        voice: false,
        agc: true,
        tx: None,
    }]));
    std::thread::sleep(std::time::Duration::from_secs(2));

    let mut readings = Vec::new();
    let start = std::time::Instant::now();
    while start.elapsed().as_secs_f32() < 6.0 {
        std::thread::sleep(std::time::Duration::from_millis(50));
        let Some(st) = r.status.channel_state(1) else {
            continue;
        };
        readings.push((st.squelch_db, st.agc_gain_db, st.squelch_open));
    }
    r.send(radio::Cmd::Stop);
    if readings.is_empty() {
        println!("no audio ran");
        return;
    }
    let mut m: Vec<f32> = readings.iter().map(|(v, _, _)| *v).collect();
    m.sort_by(f32::total_cmp);
    let pct = |p: f32| m[((m.len() - 1) as f32 * p) as usize];
    let open = readings.iter().filter(|(_, _, o)| *o).count();
    println!(
        "{mhz} MHz {}: squelch reads {:.1} / {:.1} / {:.1} dB (min/median/max),          open {}% of the time, agc {:+.0} dB",
        mode.label(),
        pct(0.0),
        pct(0.5),
        pct(1.0),
        open * 100 / readings.len(),
        readings.last().unwrap().1,
    );
}

fn probe(mhz: f64, listen: bool, want: Option<String>, dc_on: bool) {
    use common::{Hz, Sps};
    let rate = 2_304_000.0;
    // Picked by name so a HackRF can be probed while an RTL-SDR is plugged in.
    let all = devices::list();
    let Some(entry) = want
        .and_then(|w| {
            all.iter().find(|d| d.label.to_lowercase().contains(&w.to_lowercase())).cloned()
        })
        .or_else(|| all.into_iter().next())
    else {
        println!("no radio found");
        return;
    };
    println!("using {}", entry.label);
    let r = radio::Radio::start(entry, Hz((mhz * 1e6) as u64), Sps(rate as u64), 2048, || {});
    r.send(radio::Cmd::DcBlock(dc_on));
    println!("dc block: {}", if dc_on { "on" } else { "off" });
    if listen {
        // Decode a channel off-centre, the case that was dropping samples.
        r.send(radio::Cmd::Channels(vec![radio::ChannelSpec {
            id: 1,
            label: String::new(),
            offset_hz: 0.0,
            mode: radio::ChanMode::Audio(radio::Demod::Wfm),
            bandwidth_hz: None,
            volume: 1.0,
            muted: true,
            squelch_db: None,
            voice: false,
            agc: true,
            tx: None,
        }]));
        println!("decoding a WFM channel while measuring");
    }
    let start = std::time::Instant::now();
    let mut n = 0;
    // RDS needs longer than a spectrum check: a station name is four groups
    // and radiotext is sixteen, repeated every couple of seconds.
    let secs = if listen { 25 } else { 8 };
    while start.elapsed().as_secs() < secs {
        let Ok(f) = r.frames.recv_timeout(std::time::Duration::from_secs(3)) else {
            break;
        };
        n += 1;
        if n % 20 != 0 {
            continue;
        }
        let (mut peak, mut idx) = (f32::MIN, 0);
        for (i, &v) in f.db.iter().enumerate() {
            if v > peak {
                peak = v;
                idx = i;
            }
        }
        let mut s: Vec<f32> = f.db.iter().copied().filter(|x| x.is_finite()).collect();
        s.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let median = s[s.len() / 2];
        // The centre bin is where a direct-conversion receiver puts its own
        // leakage, so report it against the floor: that number is the spur.
        let mid = f.db[f.db.len() / 2];
        let hz = f.center - f.rate / 2.0 + idx as f64 * f.rate / f.db.len() as f64;
        println!(
            "frame {n:3}  peak {peak:6.1} dBFS at {:9.4} MHz  floor {median:6.1}  \
             peak-floor {:5.1} dB  centre-floor {:5.1} dB",
            hz / 1e6,
            peak - median,
            mid - median
        );
    }
    if listen {
        let st = r.status.station();
        println!(
            "\nstereo blend {:.2}   PI {}   name {:?}   pty {:?}",
            r.status.blend(),
            st.pi.map(|p| format!("{p:04X}")).unwrap_or_else(|| "-".into()),
            st.name,
            st.pty
        );
        println!(
            "rds groups {}   block errors {}   synced {}",
            st.groups, st.block_errors, st.synced
        );
        if let Some(rt) = st.radiotext {
            println!("radiotext: {rt}");
        }
    }
    let dropped = r.status.dropped.load(std::sync::atomic::Ordering::Relaxed);
    println!("\n{n} frames in {:.1}s   dropped {dropped}", start.elapsed().as_secs_f64());
    let err = r.status.error.lock().clone();
    if let Some(e) = err {
        println!("error: {e}");
    }
}

/// Report what is actually in the multiplex at a given station.
///
/// Guessing at why RDS will not decode is expensive: the pilot, the difference
/// subcarrier and RDS are all at known frequencies, so measuring their levels
/// says immediately whether the problem is the receiver or the transmitter.
fn mpx_report(mhz: f64) {
    use common::{Hz, Sps};
    use dsp::rds::RdsDemod;
    use dsp::{FirDecim, FmDemod, StereoDecoder};
    use std::f64::consts::TAU;

    let rate = 2_304_000.0;
    let Some(entry) = devices::list().into_iter().next() else {
        println!("no radio found");
        return;
    };
    let mut dev = match devices::open(&entry) {
        Ok(d) => d,
        Err(e) => {
            println!("open failed: {e}");
            return;
        }
    };
    dev.set_rate(Sps(rate as u64)).ok();
    dev.set_center(Hz((mhz * 1e6) as u64)).ok();
    dev.set_gain("tuner", common::device::GainMode::Auto).ok();
    let mut stream = match dev.start_rx() {
        Ok(s) => s,
        Err(e) => {
            println!("start failed: {e}");
            return;
        }
    };

    let dec = 7usize;
    let if_rate = rate / dec as f64;
    let mut iff = FirDecim::design_hz(rate, dec, 132_000.0, 70.0);
    let mut fm = FmDemod::new(if_rate, 75_000.0);
    let mut st = StereoDecoder::new(if_rate);
    let mut rds = RdsDemod::new(if_rate);
    let (mut iq, mut disc) = (Vec::new(), Vec::new());
    let (mut l, mut r, mut bits) = (Vec::new(), Vec::new(), Vec::new());

    let g = |x: &[f32], f: f64| {
        let k = TAU * f / if_rate;
        let c = 2.0 * k.cos();
        let (mut a, mut b) = (0.0f64, 0.0f64);
        for &v in x {
            let t = v as f64 + c * a - b;
            b = a;
            a = t;
        }
        (a * a + b * b - c * a * b).sqrt() / x.len() as f64
    };

    let start = std::time::Instant::now();
    let mut n = 0;
    while start.elapsed().as_secs() < 12 {
        let Ok(buf) = stream.read() else { break };
        iq.clear();
        iff.process(&buf.samples, &mut iq);
        disc.clear();
        fm.process(&iq, &mut disc);
        st.process(&disc, &mut l, &mut r);
        bits.clear();
        rds.process(&disc, st.phases(), &mut bits);
        n += 1;
        if n % 12 != 0 {
            continue;
        }
        let db = |v: f64, r: f64| 20.0 * (v / r.max(1e-15)).log10();
        let a1 = g(&disc, 1_000.0);
        println!(
            "pilot19 {:6.1} dB   diff38 {:6.1} dB   rds57 {:6.1} dB   (ref audio 1 kHz)                lock {:.2} blend {:.2} | rds level {:.5} arm {} margin {:.2} locked {}",
            db(g(&disc, 19_000.0), a1),
            db(g(&disc, 38_000.0), a1),
            db(g(&disc, 57_000.0), a1),
            st.lock(),
            st.blend(),
            rds.level(),
            rds.timing().0,
            rds.timing().1,
            rds.timing_locked(),
        );
    }
}

/// Time the per-channel audio chain against real time, which is the only
/// number that decides whether the radio thread can keep draining USB.
fn bench_audio() {
    use common::C32;
    let rate = 2_304_000.0;
    let block = 262_144usize;
    let sig: Vec<C32> = (0..block)
        .map(|i| {
            let p = std::f64::consts::TAU * 0.1 * i as f64;
            C32::new(p.cos() as f32 * 0.5, p.sin() as f32 * 0.5)
        })
        .collect();
    println!("{:5} {:>44} {:>10} {:>10}", "mode", "filters", "x real", "us/block");
    for mode in [radio::Demod::Wfm, radio::Demod::Nfm, radio::Demod::Am] {
        let mut a = radio::Audio::new(120_000.0, rate, mode, 48_000.0);
        a.process(&sig, 0.5);
        let reps = 24;
        let t = std::time::Instant::now();
        for _ in 0..reps {
            a.process(&sig, 0.5);
        }
        let el = t.elapsed().as_secs_f64();
        let audio_secs = reps as f64 * block as f64 / rate;
        println!(
            "{:5} {:>44} {:>9.1}x {:>9.0}",
            mode.label(),
            a.cost(),
            audio_secs / el,
            el / reps as f64 * 1e6
        );
    }
}

/// Replay a capture through the whole receiver, block by block, and report
/// where throughput drops.
///
/// A lag spike is one block that took longer than the samples in it, so a mean
/// says nothing: the worst blocks, how often they come and which stage was in
/// them are the whole of the answer. The graph is the one the scanner table
/// puts on the capture's frequency, the same as `--replay`, so what is timed
/// is what the live receiver runs.
fn bench_iq(path: &str, block: usize) -> anyhow::Result<()> {
    let src = sources::FileSource::open(std::path::Path::new(path))?;
    let buf = src.read_all()?;
    if buf.samples.is_empty() {
        anyhow::bail!("{path} holds no samples");
    }
    let rate = buf.rate.as_f64().max(1.0);
    let block = block.clamp(1, buf.samples.len());
    let block_secs = block as f64 / rate;
    let per_pass = buf.samples.len() / block;
    // Enough blocks that a percentile means something, and a bound so a long
    // capture is not run twice for nothing.
    let passes = (200usize.div_ceil(per_pass.max(1))).clamp(1, 20);

    let mut rx = radio::replay_receiver(&buf, None)?;
    let labels: Vec<String> = rx.node_costs().into_iter().map(|(l, _)| l).collect();
    println!(
        "{path}\n{:.4} MHz at {:.3} MS/s, {:.2} s, {} nodes",
        buf.center.as_f64() / 1e6,
        rate / 1e6,
        buf.samples.len() as f64 / rate,
        labels.len(),
    );
    println!(
        "block {block} samples ({:.2} ms), {} blocks a pass, {passes} pass(es){}\n",
        block_secs * 1e3,
        per_pass,
        if passes > 1 { ", so the file repeats" } else { "" },
    );

    /// One block's measurement: how long it took and where it went.
    struct Blk {
        us: f64,
        top: [(usize, u64); 3],
    }
    let mut blocks: Vec<Blk> = Vec::with_capacity(per_pass * passes);
    let mut prev: Vec<u64> = rx.node_costs().into_iter().map(|(_, us)| us).collect();
    let mut delta = vec![0u64; prev.len()];
    let wall = std::time::Instant::now();
    for _ in 0..passes {
        for chunk in buf.samples.chunks(block) {
            if chunk.len() < block {
                break;
            }
            let t = std::time::Instant::now();
            if rx.process(chunk).is_err() {
                anyhow::bail!("the graph refused a block");
            }
            let us = t.elapsed().as_secs_f64() * 1e6;
            // Read outside the timed region: the strings it allocates would
            // otherwise be counted as the graph's own cost.
            let now = rx.node_costs();
            for (i, d) in delta.iter_mut().enumerate() {
                *d = now.get(i).map(|(_, us)| *us).unwrap_or(0).saturating_sub(prev[i]);
            }
            let mut top = [(0usize, 0u64); 3];
            for (i, &d) in delta.iter().enumerate() {
                if d > top[2].1 {
                    top[2] = (i, d);
                    top.sort_by(|a, b| b.1.cmp(&a.1));
                }
            }
            for (i, (_, us)) in now.iter().enumerate() {
                prev[i] = *us;
            }
            let _ = rx.decodes(std::time::Instant::now());
            blocks.push(Blk { us, top });
        }
    }
    let wall = wall.elapsed().as_secs_f64();
    if blocks.len() < 8 {
        anyhow::bail!(
            "only {} blocks: use a longer capture or a smaller --bench-block",
            blocks.len()
        );
    }

    // The first blocks allocate, fault in pages and open the sources the
    // detector finds, so they say nothing about a steady state.
    const WARM: usize = 4;
    let timed = &blocks[WARM..];
    let mut sorted: Vec<f64> = timed.iter().map(|b| b.us).collect();
    sorted.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let at = |q: f64| sorted[(((sorted.len() - 1) as f64) * q) as usize];
    let x = |us: f64| block_secs * 1e6 / us.max(1e-9);
    let median = at(0.5);
    println!(
        "{} blocks timed, {:.2} s of signal in {wall:.2} s wall, {:.1}x mean",
        timed.len(),
        timed.len() as f64 * block_secs,
        timed.len() as f64 * block_secs / wall.max(1e-9),
    );
    println!(
        "\n{:>8} {:>10} {:>10}\n{:>8} {:>9.2}x {:>9.2}\n{:>8} {:>9.2}x {:>9.2}\n{:>8} {:>9.2}x {:>9.2}\n{:>8} {:>9.2}x {:>9.2}\n{:>8} {:>9.2}x {:>9.2}",
        "", "x real", "ms",
        "median", x(median), median / 1e3,
        "p90", x(at(0.90)), at(0.90) / 1e3,
        "p99", x(at(0.99)), at(0.99) / 1e3,
        "worst", x(sorted[sorted.len() - 1]), sorted[sorted.len() - 1] / 1e3,
        "best", x(sorted[0]), sorted[0] / 1e3,
    );
    let over = timed.iter().filter(|b| b.us > block_secs * 1e6).count();
    println!(
        "{over} block(s) slower than real time, {} over three times the median",
        timed.iter().filter(|b| b.us > 3.0 * median).count(),
    );

    // A trace of the whole run, with the slowest block in each column, since
    // it is the lows that are being looked for and a mean hides them.
    const COLS: usize = 96;
    let bars = [
        ' ', '\u{2581}', '\u{2582}', '\u{2583}', '\u{2584}', '\u{2585}', '\u{2586}', '\u{2587}',
        '\u{2588}',
    ];
    let per_col = timed.len().div_ceil(COLS).max(1);
    let worst_x = sorted[sorted.len() - 1];
    let trace: String = timed
        .chunks(per_col)
        .map(|c| {
            let slow = c.iter().map(|b| b.us).fold(0.0f64, f64::max);
            let f = (slow / worst_x).clamp(0.0, 1.0);
            bars[(f * (bars.len() - 1) as f64).round() as usize]
        })
        .collect();
    println!("\ntime in a block, tall is slow, full height is {:.1} ms\n{trace}", worst_x / 1e3);

    // How regularly the slow blocks arrive. A spike that comes every N blocks
    // is something running on a period, and the period says which thing.
    let slow: Vec<usize> =
        timed.iter().enumerate().filter(|(_, b)| b.us > 2.0 * median).map(|(i, _)| i).collect();
    if slow.len() > 2 {
        let mut gaps: Vec<usize> = slow.windows(2).map(|w| w[1] - w[0]).collect();
        gaps.sort_unstable();
        let g = gaps[gaps.len() / 2];
        println!(
            "\n{} slow blocks, typically {g} blocks apart ({:.0} ms), spread {}..{}",
            slow.len(),
            g as f64 * block_secs * 1e3,
            gaps[0],
            gaps[gaps.len() - 1],
        );
    }

    let name = |i: usize| labels.get(i).map(String::as_str).unwrap_or("?");
    println!("\n{:>7} {:>9} {:>8}  {}", "block", "ms", "x real", "where the time went");
    let mut order: Vec<usize> = (0..timed.len()).collect();
    order.sort_by(|&a, &b| timed[b].us.partial_cmp(&timed[a].us).unwrap());
    for &i in order.iter().take(12) {
        let b = &timed[i];
        let where_ = b
            .top
            .iter()
            .filter(|(_, us)| *us > 0)
            .map(|(n, us)| format!("{} {:.2} ms", name(*n), *us as f64 / 1e3))
            .collect::<Vec<_>>()
            .join(", ");
        println!("{:>7} {:>9.2} {:>7.2}x  {where_}", i + WARM, b.us / 1e3, x(b.us));
    }

    // Past a hundred per cent between them: independent nodes run beside
    // each other, so the shares are of one core's time and not of the wall.
    println!("\n{:>12} {:>10} {:>9}  {}", "total ms", "% of wall", "us/block", "node");
    let mut totals: Vec<(String, u64)> = rx.node_costs();
    totals.sort_by(|a, b| b.1.cmp(&a.1));
    for (label, us) in totals.into_iter().take(15).filter(|(_, us)| *us > 0) {
        println!(
            "{:>12.1} {:>9.1}% {:>9.0}  {label}",
            us as f64 / 1e3,
            us as f64 / 1e6 / wall * 100.0,
            us as f64 / blocks.len() as f64,
        );
    }

    // Inside the composites, where a node reports its own parts. The p95 is
    // what the chain view shows and is the number a spike lives in.
    for n in rx.topology().nodes.iter().filter(|n| !n.phases.is_empty()) {
        println!("\n{:>10} {:>10} {:>9}  {} phases", "p95 us", "mean us", "calls", n.label);
        let mut ph = n.phases.clone();
        ph.sort_by(|a, b| b.1.p95_us.cmp(&a.1.p95_us));
        for (name, c) in ph.iter().filter(|(_, c)| c.calls > 0) {
            println!("{:>10} {:>10.0} {:>9}  {name}", c.p95_us, c.mean_us, c.calls);
        }
    }
    Ok(())
}

/// How long a retune actually costs, which decides whether a drag can send one
/// per frame.
fn bench_tune() {
    use common::Hz;
    let Some(entry) = devices::list().into_iter().next() else {
        println!("no radio found");
        return;
    };
    println!("using {}", entry.label);
    let mut dev = match devices::open(&entry) {
        Ok(d) => d,
        Err(e) => {
            println!("open failed: {e}");
            return;
        }
    };
    let mut best = f64::MAX;
    let mut worst: f64 = 0.0;
    let mut total = 0.0;
    const N: usize = 60;
    for i in 0..N {
        let f = 95_000_000 + (i as u64 % 20) * 25_000;
        let t = std::time::Instant::now();
        let _ = dev.set_center(Hz(f));
        let ms = t.elapsed().as_secs_f64() * 1e3;
        best = best.min(ms);
        worst = worst.max(ms);
        total += ms;
    }
    println!(
        "set_center x{N}:  min {best:.2} ms   mean {:.2} ms   max {worst:.2} ms",
        total / N as f64
    );
    println!("a 60 fps drag spends {:.0}% of each frame retuning", total / N as f64 / 16.7 * 100.0);
}

/// Frames still delivered while the centre is being dragged.
///
/// A drag issues one retune per displayed frame, and a retune blocks the thread
/// that reads samples, so without coalescing the spectrum stops updating for as
/// long as the drag lasts.
fn bench_pan() {
    use common::{Hz, Sps};
    let Some(entry) = devices::list().into_iter().next() else {
        println!("no radio found");
        return;
    };
    println!("using {}", entry.label);
    let r = radio::Radio::start(entry, Hz(95_800_000), Sps(2_304_000), 2048, || {});

    let count = |label: &str, drag: bool| {
        // Settle, then count for three seconds.
        std::thread::sleep(std::time::Duration::from_millis(600));
        while r.frames.try_recv().is_ok() {}
        let t = std::time::Instant::now();
        let mut n = 0;
        let mut step = 0u64;
        while t.elapsed() < std::time::Duration::from_secs(3) {
            if drag {
                // One per frame at 60 fps, which is what a drag produces.
                step = (step + 1) % 200;
                r.send(radio::Cmd::Center(Hz(95_800_000 + step * 500)));
            }
            while r.frames.try_recv().is_ok() {
                n += 1;
            }
            std::thread::sleep(std::time::Duration::from_millis(16));
        }
        println!("{label:22}  {:.1} frames/s", n as f64 / 3.0);
    };
    count("idle", false);
    count(
        if std::env::var("SR_TUNE_GAP_MS").as_deref() == Ok("0") {
            "dragging, uncoalesced"
        } else {
            "dragging the centre"
        },
        true,
    );
    r.send(radio::Cmd::Stop);
    std::thread::sleep(std::time::Duration::from_millis(300));
}

/// Decode a capture, or a directory of them, and print what came out.
///
/// This is the short loop: record once, then run this after every change to
/// a slicer or a protocol and see immediately whether the same burst now
/// decodes. No radio, no waiting for a device to transmit, and the same
/// answer every time.
/// Run the decoders over a packet log.
///
/// The log stores what the demodulator produced rather than what a decoder
/// made of it, so this is not a printout of old conclusions: the protocols
/// run again, now, over the bursts as they were heard. A decoder written
/// after the log was written gets its chance at every burst in it, and a
/// decoder that has since been fixed shows what it used to get wrong.
fn replay_log(path: &std::path::Path) -> anyhow::Result<()> {
    let bursts = packetlog::read(path)?;
    if bursts.is_empty() {
        anyhow::bail!("{} holds no bursts", path.display());
    }
    let protocols = decode::Protocols::all();
    let (mut decoded, mut silent) = (0, 0);
    for p in &bursts {
        let secs = p.at_us / 1_000_000 % 86_400;
        let when = format!("{:02}:{:02}:{:02}", secs / 3600, secs / 60 % 60, secs % 60);
        let mhz = p.center_hz() as f64 / 1e6;
        let Some(pkg) = p.package() else {
            let bytes = p.frame().unwrap_or_default();
            println!(
                "{when}  {mhz:10.4} MHz  {:>4} B  {}",
                bytes.len(),
                bytes.iter().map(|x| format!("{x:02x}")).collect::<String>()
            );
            decoded += 1;
            continue;
        };
        let reports = protocols.decode_all(&pkg);
        if reports.is_empty() {
            silent += 1;
            println!(
                "{when}  {mhz:10.4} MHz  {:>4} pulses  {:>5.1} dB  unclaimed",
                pkg.pulses.len(),
                p.snr_db(),
            );
        }
        for r in reports {
            decoded += 1;
            println!(
                "{when}  {mhz:10.4} MHz  {:>4} pulses  {:>5.1} dB  {:<22} {}",
                pkg.pulses.len(),
                p.snr_db(),
                r.model,
                r.fields_line(),
            );
        }
    }
    println!("\n{} burst(s): {decoded} decoded, {silent} unclaimed", bursts.len());
    Ok(())
}

/// Scan a band headless: the receiver as the window runs it, with every
/// packet printed as it arrives instead of listed.
///
/// The same radio thread, the same scanner table and the same decode
/// switch as the interactive receiver, so what this prints is what the
/// window would list, one line per packet in the list's columns. For a
/// capture on disk, `--replay` does the same from the file.
fn scan(
    mhz: f64,
    span_khz: f64,
    want: Option<String>,
    packet_log: Option<PathBuf>,
    survey: Option<PathBuf>,
    gps: Option<gps::Transport>,
    location: Option<(f64, f64)>,
    ha: Option<nodes::Publish>,
    dc_on: bool,
    print: bool,
) {
    use common::{Hz, Sps};
    eprintln!("listing radios");
    let all = devices::list();
    for d in &all {
        eprintln!("  {}", d.label);
    }
    let Some(entry) = want
        .as_ref()
        .and_then(|w| {
            all.iter().find(|d| d.label.to_lowercase().contains(&w.to_lowercase())).cloned()
        })
        .or_else(|| all.into_iter().next())
    else {
        println!("no radio found");
        return;
    };
    let rate = span_khz * 1e3;
    let r =
        radio::Radio::start(entry.clone(), Hz((mhz * 1e6) as u64), Sps(rate as u64), 1024, || {});
    r.send(radio::Cmd::DcBlock(dc_on));
    r.send(radio::Cmd::Decode(true));
    if let Some((lat, lon)) = location {
        r.send(radio::Cmd::Location(lat, lon));
    }
    r.send(radio::Cmd::PacketLog(packet_log.clone()));
    r.send(radio::Cmd::Survey(survey.clone()));
    if let Some(p) = ha.clone() {
        eprintln!("publishing devices to {}:{}", p.broker.host, p.broker.port);
        r.send(radio::Cmd::HomeAssistant(Some(p)));
    }
    if let Some(t) = gps.clone() {
        r.send(radio::Cmd::Gps(Some(t)));
    }
    eprintln!(
        "scanning {:.4} MHz at {:.3} MS/s on {}; packet log {}; ctrl-c stops",
        mhz,
        rate / 1e6,
        entry.label,
        packet_log.as_ref().map(|d| d.display().to_string()).unwrap_or_else(|| "off".into())
    );
    if print {
        println!("{}", radio::DecodeRecord::line_header());
    }
    let start = std::time::Instant::now();
    let mut n = 0u64;
    loop {
        // The spectrum frames are for a window; drained so the radio thread
        // never waits on a display that is not there.
        while r.frames.try_recv().is_ok() {}
        match r.decodes.recv_timeout(std::time::Duration::from_millis(500)) {
            Ok(recs) => {
                for rec in recs {
                    n += 1;
                    if print {
                        println!("{}", rec.line(start));
                    }
                }
            }
            Err(crossbeam_channel::RecvTimeoutError::Timeout) => {
                if let Some(e) = r.status.error.lock().clone() {
                    eprintln!("radio: {e}");
                    break;
                }
                if !r.status.running.load(std::sync::atomic::Ordering::Relaxed) {
                    break;
                }
            }
            Err(crossbeam_channel::RecvTimeoutError::Disconnected) => break,
        }
    }
    eprintln!("{n} packets");
}

fn replay(path: &str) -> anyhow::Result<()> {
    let path = std::path::Path::new(path);
    if path.extension().and_then(|s| s.to_str()) == Some("wspkt") {
        return replay_log(path);
    }
    let mut files: Vec<std::path::PathBuf> = if path.is_dir() {
        std::fs::read_dir(path)?
            .filter_map(|e| e.ok().map(|e| e.path()))
            .filter(|p| {
                matches!(
                    p.extension().and_then(|s| s.to_str()),
                    Some("cu8" | "cs8" | "cs16" | "cf32" | "data")
                )
            })
            .collect()
    } else {
        vec![path.to_path_buf()]
    };
    files.sort();
    if files.is_empty() {
        anyhow::bail!("no captures in {}", path.display());
    }

    let (mut decoded, mut unknown) = (0, 0);
    for f in &files {
        let name = f.file_name().and_then(|s| s.to_str()).unwrap_or("");
        match radio::replay(f) {
            Ok(recs) if recs.is_empty() => println!("{name}: nothing decoded"),
            Ok(recs) => {
                for r in &recs {
                    if !r.is_known() {
                        unknown += 1;
                    } else {
                        decoded += 1;
                    }
                    println!(
                        "{name}: {:.4} MHz {} {:>6.1} dBFS {:>5.1} dB  {:<22} {:>3} B  {}",
                        r.freq / 1e6,
                        r.modulation,
                        r.rssi_dbfs,
                        r.snr_db,
                        r.protocol(),
                        r.bytes.len(),
                        r.detail,
                    );
                }
            }
            Err(e) => println!("{name}: {e}"),
        }
    }
    println!("\n{} capture(s): {decoded} decoded, {unknown} unknown", files.len());
    Ok(())
}

/// Command line surface.
///
/// The interactive receiver is what running this with no arguments gives you.
/// Everything else is either a diagnostic that prints numbers and exits, or a
/// switch that sets the receiver up so a session can be reproduced without a
/// dozen clicks first.
/// `53.64,-6.65` as a pair of degrees.
/// `LAT,LON` in decimal degrees, from the command line or the station field.
/// A GPS source as the operator writes one on the command line.
fn parse_gps(s: &str) -> Result<gps::Transport, String> {
    gps::Transport::parse(s).ok_or_else(|| format!("{s:?} is not a serial port or a gpsd address"))
}

/// `homeassistant.local`, `host:1883`, or `mqtt://user:pass@host:1883`.
///
/// One argument rather than five, because what a person has in front of them
/// is the line their broker is described by somewhere else.
fn parse_broker(s: &str) -> Result<nodes::Publish, String> {
    let rest = s.trim().trim_start_matches("mqtt://");
    if rest.is_empty() {
        return Err("expected [user:password@]host[:port]".into());
    }
    let (creds, hostport) = match rest.rsplit_once('@') {
        Some((c, h)) => (Some(c), h),
        None => (None, rest),
    };
    let (host, port) = match hostport.rsplit_once(':') {
        Some((h, p)) => (h, p.parse().map_err(|_| format!("{p:?} is not a port"))?),
        None => (hostport, 1883u16),
    };
    if host.is_empty() {
        return Err("no host in the broker address".into());
    }
    let (username, password) = match creds {
        Some(c) => match c.split_once(':') {
            Some((u, p)) => (u.to_string(), p.to_string()),
            None => (c.to_string(), String::new()),
        },
        None => (String::new(), String::new()),
    };
    let broker = nodes::Broker { port, username, password, ..nodes::Broker::new(host) };
    Ok(nodes::Publish { broker, spaces: String::new() })
}

/// `8931` or `127.0.0.1:8931`, for the MCP server's address.
///
/// A bare port means loopback: an agent socket that carries the whole
/// receiver should not be offered to a network by leaving a host out.
#[cfg(feature = "mcp")]
fn parse_listen(s: &str) -> Result<std::net::SocketAddr, String> {
    if let Ok(port) = s.parse::<u16>() {
        return Ok(std::net::SocketAddr::from(([127, 0, 0, 1], port)));
    }
    s.parse().map_err(|_| format!("{s:?} is not a port or a host:port"))
}

pub fn parse_location(s: &str) -> Result<(f64, f64), String> {
    let (a, o) = s.split_once(',').ok_or("expected LAT,LON")?;
    let lat: f64 = a.trim().parse().map_err(|_| "latitude is not a number")?;
    let lon: f64 = o.trim().parse().map_err(|_| "longitude is not a number")?;
    if !(-90.0..=90.0).contains(&lat) || !(-180.0..=180.0).contains(&lon) {
        return Err("outside the range of a coordinate".into());
    }
    Ok((lat, lon))
}

#[derive(Parser, Debug)]
#[command(name = "waveshark", about = "Wideband SDR receiver and RF protocol analyser", version)]
struct Args {
    /// Start tuned to this frequency, in MHz, and listening to it. Repeat it
    /// to open several channels at once, which are mixed together
    #[arg(long, value_name = "MHZ")]
    tune: Vec<f64>,

    /// Demodulator to start in
    #[arg(long, value_enum, default_value_t = Mode::Wfm)]
    mode: Mode,

    /// Start at the nearest span to this, in kHz, narrowing in software when
    /// the radio cannot sample that slowly
    #[arg(long, value_name = "KHZ")]
    span: Option<f64>,

    /// Pick a radio by name, for when several are plugged in
    #[arg(long, value_name = "NAME")]
    device: Option<String>,

    /// Total tuner gain in dB, distributed across the radio's stages
    #[arg(long, value_name = "DB")]
    rf_gain: Option<f32>,

    /// Offer an iqstream server as a radio, as host or host:port. Repeatable,
    /// and added to whatever the session already holds
    #[arg(long, value_name = "HOST")]
    stream: Vec<String>,

    /// Open a recorded capture as the receiver and replay it at the rate it
    /// was recorded at, which is what the receiver list's file dialog does
    #[arg(long, value_name = "FILE")]
    capture: Vec<PathBuf>,

    /// Write a PNG of the interface and exit
    #[arg(long, value_name = "PATH", num_args = 0..=1, default_missing_value = "/tmp/shot.png")]
    shot: Option<String>,

    /// Seconds to wait before the screenshot, for views that need traffic
    #[arg(long, value_name = "SECS", default_value_t = 6.0)]
    shot_after: f32,

    /// Open on the signal chain view
    #[arg(long)]
    chain: bool,

    /// Open on the flight tracker
    #[arg(long)]
    flights: bool,

    /// Open on the call list
    #[arg(long)]
    calls: bool,

    /// Open on the messages received
    #[arg(long)]
    messages: bool,

    /// Open on the transcript: what the local model read off the audio bus
    #[arg(long)]
    transcript: bool,

    /// Open on the data links: who is talking to whom, and what passed
    #[arg(long)]
    links: bool,

    /// Open on the control links: where the sticks are on every handset heard
    #[arg(long)]
    control: bool,

    /// Start the radio as soon as the window opens, without a click on play
    #[arg(long)]
    run: bool,

    /// Publish every device heard to this MQTT broker, so Home Assistant
    /// builds them: [user:password@]host[:port]
    #[arg(long, value_name = "BROKER", value_parser = parse_broker)]
    ha_broker: Option<nodes::Publish>,

    /// Publish only these identity spaces, comma separated: `ism,wmbus` is a
    /// house's own sensors and meters without the street's handsets
    #[arg(long, value_name = "SPACES")]
    ha_spaces: Option<String>,
    /// Serve MCP on this address, so an agent can drive this receiver:
    /// a port, or host:port. Loopback unless a host is given
    #[cfg(feature = "mcp")]
    #[arg(long, value_name = "ADDR", value_parser = parse_listen)]
    mcp_listen: Option<std::net::SocketAddr>,

    /// Open on the picture, for analogue video
    #[arg(long)]
    video: bool,

    /// Tune here, in MHz, without opening a channel on it
    #[arg(long, value_name = "MHZ")]
    center: Option<f64>,

    /// Open the radio's own controls
    #[arg(long)]
    gain: bool,

    /// Open the scanner table, which decides what runs on which frequency
    #[arg(long)]
    scanners: bool,

    /// Open setup, which is where the cached datasets are listed
    #[arg(long)]
    setup: bool,

    /// Write every burst that decodes into this directory
    #[arg(long, value_name = "DIR", num_args = 0..=1, default_missing_value = "captures")]
    record: Option<PathBuf>,

    /// Start writing the raw span to a file as soon as the radio is running
    #[arg(long)]
    capture_iq: bool,

    /// Write a binary packet log, one file a day. Off unless asked for, and
    /// in the interface the switch is remembered. Defaults to
    /// $XDG_DATA_HOME/waveshark/packets
    #[arg(long, value_name = "DIR", num_args = 0..=1, default_missing_value = "")]
    packet_log: Option<PathBuf>,

    /// Do not write the packet log, whatever the saved setting says
    #[arg(long)]
    no_packet_log: bool,

    /// Record a database of the devices heard and where they were heard. Off
    /// unless asked for. Defaults to
    /// $XDG_DATA_HOME/waveshark/survey.sqlite
    #[arg(long, value_name = "FILE", num_args = 0..=1, default_missing_value = "")]
    survey: Option<PathBuf>,

    /// Do not record a device database, whatever the saved setting says
    #[arg(long)]
    no_survey: bool,

    /// Read the receiver's own position from a GPS other than the local gpsd,
    /// which is looked for anyway: a serial port (/dev/ttyACM0, or
    /// /dev/ttyUSB0@4800) or a gpsd address (gpsd:host:port)
    #[arg(long, value_name = "SOURCE", value_parser = parse_gps)]
    gps: Option<gps::Transport>,

    /// Receiver position as LAT,LON in degrees, which lets one ADS-B frame
    /// fix an aircraft instead of needing a matching pair
    #[arg(long, value_name = "LAT,LON", value_parser = parse_location)]
    location: Option<(f64, f64)>,

    /// How much may be written before recording stops
    #[arg(long, value_name = "MB")]
    record_mb: Option<u64>,

    /// Decode a capture, or a directory of them, and print what came out
    #[arg(long, value_name = "PATH", num_args = 0..=1, default_missing_value = "captures")]
    replay: Option<String>,

    /// Run for this many seconds, then report CPU and span timings
    #[arg(long, value_name = "SECS", num_args = 0..=1, default_missing_value = "12")]
    soak: Option<f32>,

    /// Check the signal path with no display
    #[arg(long, value_name = "MHZ", num_args = 0..=1, default_missing_value = "95.8")]
    probe: Option<f64>,

    /// Decode a channel while probing, the case that used to drop samples
    #[arg(long)]
    listen: bool,

    /// Run the receiver without a window, on the radio at `--tune` and
    /// `--span`, scanning and logging as the window would.
    #[arg(long)]
    headless: bool,

    /// Print every packet to standard output as it arrives, in the packet
    /// list's columns, with or without a window.
    #[arg(long)]
    print_log: bool,

    /// Report what the squelch reads on a frequency
    #[arg(long, value_name = "MHZ", num_args = 0..=1, default_missing_value = "145.5")]
    squelch_probe: Option<f64>,

    /// Report FM multiplex levels
    #[arg(long, value_name = "MHZ", num_args = 0..=1, default_missing_value = "95.8")]
    mpx: Option<f64>,

    /// Leave the centre spur in, for measuring what removing it does
    #[arg(long)]
    no_dc: bool,

    /// Read the M17 streams out of a packet log and report what their
    /// payloads decode to, for tracing a call that arrived silent
    #[arg(long, value_name = "LOG")]
    m17_dump: Option<PathBuf>,

    /// Time a retune
    #[arg(long)]
    bench_tune: bool,

    /// Frames delivered while the centre is dragged
    #[arg(long)]
    bench_pan: bool,

    /// Audio chain throughput
    #[arg(long)]
    bench_audio: bool,

    /// Replay a capture through the receiver and report where throughput
    /// drops, which is where a lag spike comes from
    #[arg(long, value_name = "PATH")]
    bench_iq: Option<String>,

    /// Samples a block for `--bench-iq`. The default is a HackRF transfer,
    /// which is the parcel the live radio hands the graph, so anything that
    /// runs once a block keeps its period
    #[arg(long, value_name = "SAMPLES", default_value_t = 131_072)]
    bench_block: usize,

    /// Download or revalidate the cached datasets and exit, for warming the
    /// cache before going somewhere without a connection
    #[arg(long)]
    fetch_data: bool,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq, ValueEnum)]
enum Mode {
    Wfm,
    Nfm,
    Am,
    Usb,
    Lsb,
    Cw,
}

impl From<Mode> for radio::Demod {
    fn from(m: Mode) -> Self {
        match m {
            Mode::Wfm => radio::Demod::Wfm,
            Mode::Nfm => radio::Demod::Nfm,
            Mode::Am => radio::Demod::Am,
            Mode::Usb => radio::Demod::Usb,
            Mode::Lsb => radio::Demod::Lsb,
            Mode::Cw => radio::Demod::Cw,
        }
    }
}

/// What a logged M17 transmission holds, and what the vocoder makes of it.
///
/// The log keeps every stream frame's payload, so a call that sounded wrong
/// can be taken apart afterwards: whether the frames are there, whether their
/// payloads are anything but zero, and what level the speech comes out at.
fn m17_dump(path: &std::path::Path) {
    use decode::m17::Event;
    let packets = match packetlog::read(path) {
        Ok(p) => p,
        Err(e) => {
            eprintln!("cannot read {}: {e}", path.display());
            return;
        }
    };
    let mut runs: Vec<Vec<(u16, [u8; 16])>> = Vec::new();
    let mut setups = 0usize;
    for p in &packets {
        let common::PacketBody::Frame(fr) = &p.body else {
            continue;
        };
        let b = &fr.bytes;
        match Event::parse(b) {
            Some(Event::LinkSetup { lsf, .. }) => {
                setups += 1;
                println!("setup: {} to {}", lsf.source(), lsf.destination());
                runs.push(Vec::new());
            }
            Some(Event::StreamFrame { number, payload, .. }) => {
                if runs.is_empty() {
                    runs.push(Vec::new());
                }
                runs.last_mut().unwrap().push((number, payload));
            }
            _ => {}
        }
    }
    println!("{} packets, {setups} link setups, {} streams", packets.len(), runs.len());
    for (i, run) in runs.iter().enumerate().filter(|(_, r)| !r.is_empty()) {
        let zeros = run.iter().filter(|(_, p)| p.iter().all(|b| *b == 0)).count();
        let bytes: Vec<u8> = run.iter().flat_map(|(_, p)| p.iter().copied()).collect();
        let pcm = nodes::m17_nodes::decode_stream_voice(&bytes);
        let peak = pcm.iter().fold(0.0f32, |a, b| a.max(b.abs()));
        let rms = (pcm.iter().map(|v| v * v).sum::<f32>() / pcm.len().max(1) as f32).sqrt();
        let db = |v: f32| 20.0 * v.max(1e-9).log10();
        println!(
            "stream {i}: {} frames, numbers {:?}..{:?}, {zeros} payloads all zero",
            run.len(),
            run.first().map(|(n, _)| *n),
            run.last().map(|(n, _)| *n),
        );
        for (n, p) in run.iter().take(3) {
            println!(
                "  frame {n:>3}: {}",
                p.iter().map(|b| format!("{b:02x}")).collect::<String>()
            );
        }
        println!("  {} samples, peak {:.1} dBFS, rms {:.1} dBFS", pcm.len(), db(peak), db(rms));
        // The payloads as they were on the air, for taking apart with
        // anything else that speaks Codec 2.
        let out = path.with_extension(format!("stream{i}.c2"));
        match std::fs::write(&out, &bytes) {
            Ok(()) => println!("  payloads written to {}", out.display()),
            Err(e) => eprintln!("  cannot write {}: {e}", out.display()),
        }
    }
}

/// Write to the console that started this, if one did.
///
/// A windows-subsystem binary has no console of its own, which is the point:
/// double clicking it must not put a black window behind the receiver. Run
/// from a terminal it should still answer `--help` and print what the
/// offline tools say, and that needs the parent's console attached by hand.
#[cfg(windows)]
fn attach_console() {
    use windows_sys::Win32::System::Console::{AttachConsole, ATTACH_PARENT_PROCESS};
    // SAFETY: no arguments, no handles, and a failure (there is no parent
    // console) is reported by the return value rather than by anything
    // happening.
    unsafe {
        AttachConsole(ATTACH_PARENT_PROCESS);
    }
}

#[cfg(not(windows))]
fn attach_console() {}

fn main() -> eframe::Result<()> {
    attach_console();
    let args = Args::parse();

    // Registered before anything enumerates: a radio on the network is
    // configuration, and nothing on the bus will reveal it.
    for s in &args.stream {
        if devices::add_stream(s, "").is_none() {
            eprintln!("--stream {s}: expected host or host:port");
            std::process::exit(1);
        }
    }
    for c in &args.capture {
        if devices::add_capture(c.clone()).is_none() {
            eprintln!(
                "--capture {}: name it like <what>_<centre>_<rate>.<format>, \
                 e.g. bench_433.92M_250k.cu8",
                c.display()
            );
            std::process::exit(1);
        }
    }

    if args.fetch_data {
        data::fetch_all();
        return Ok(());
    }
    if let Some(log) = &args.m17_dump {
        m17_dump(log);
        return Ok(());
    }
    if args.bench_pan {
        bench_pan();
        return Ok(());
    }
    if args.bench_tune {
        bench_tune();
        return Ok(());
    }
    if args.bench_audio {
        bench_audio();
        return Ok(());
    }
    if let Some(path) = &args.bench_iq {
        if let Err(e) = bench_iq(path, args.bench_block) {
            eprintln!("bench failed: {e}");
            std::process::exit(1);
        }
        return Ok(());
    }
    if let Some(mhz) = args.mpx {
        mpx_report(mhz);
        return Ok(());
    }
    if let Some(mhz) = args.squelch_probe {
        squelch_probe(mhz, args.mode.into());
        return Ok(());
    }
    if let Some(mhz) = args.probe {
        probe(mhz, args.listen, args.device.clone(), !args.no_dc);
        return Ok(());
    }
    if args.headless {
        // The agent server hands out the interface's own state, so there is
        // nothing to serve without one. Said rather than ignored: an agent
        // waiting on a port that will never open is a worse failure.
        #[cfg(feature = "mcp")]
        if args.mcp_listen.is_some() {
            eprintln!("--mcp-listen needs the window: it serves the receiver the interface holds");
            std::process::exit(1);
        }
        // Nothing is written down unless it was asked for. Headless has no
        // session to remember a choice in, so the choice is the command line.
        let log = args
            .packet_log
            .clone()
            .filter(|_| !args.no_packet_log)
            .map(|d| if d.as_os_str().is_empty() { None } else { Some(d) })
            .map(|d| d.or_else(packetlog::PacketLog::default_dir))
            .unwrap_or(None);
        let survey = args
            .survey
            .clone()
            .filter(|_| !args.no_survey)
            .map(|f| if f.as_os_str().is_empty() { None } else { Some(f) })
            .map(|f| f.or_else(packetlog::PacketLog::default_survey_path))
            .unwrap_or(None);
        scan(
            args.tune.first().copied().unwrap_or(433.92),
            args.span.unwrap_or(2_400.0),
            args.device.clone(),
            log,
            survey,
            args.gps.clone(),
            args.location,
            args.ha_broker.clone().map(|mut p| {
                p.spaces = args.ha_spaces.clone().unwrap_or_default();
                p
            }),
            !args.no_dc,
            args.print_log,
        );
        return Ok(());
    }
    if let Some(path) = &args.replay {
        if let Err(e) = replay(path) {
            eprintln!("replay failed: {e}");
            std::process::exit(1);
        }
        return Ok(());
    }

    if args.soak.is_some() {
        use tracing_subscriber::prelude::*;
        prof::enable();
        tracing_subscriber::registry().with(prof::Timing).init();
    } else if std::env::var_os("RUST_LOG").is_some() {
        // The window build logs nothing by default, which is right: the
        // interface is where things are said. With RUST_LOG set it should
        // still be possible to see what the radio thread is doing, and
        // without this every tracing call in the receiver went nowhere.
        let _ = tracing_subscriber::fmt()
            .with_env_filter(tracing_subscriber::EnvFilter::from_default_env())
            .try_init();
    }

    // Started before the window: on a warm cache the airports are parsed
    // before the first frame that could draw them.
    data::start();
    // One request, so it is asked for before the window and answered while
    // the first frames draw.
    update::check();

    let mut viewport = egui::ViewportBuilder::default()
        .with_inner_size(if args.shot.is_some() { [1400.0, 860.0] } else { [1280.0, 800.0] })
        .with_min_inner_size([800.0, 500.0])
        .with_title("waveshark");
    if let Some(icon) = window_icon() {
        viewport = viewport.with_icon(icon);
    }
    let opts = eframe::NativeOptions { viewport, ..Default::default() };
    eframe::run_native(
        "waveshark",
        opts,
        Box::new(move |cc| {
            let mut app = ui::App::new(cc);
            if let Some(name) = &args.device {
                app.set_device(name);
            }
            if let Some(db) = args.rf_gain {
                app.set_rf_gain(db);
            }
            if let Some(khz) = args.span {
                app.set_span(khz * 1e3);
            }
            for mhz in &args.tune {
                app.tune_to(*mhz, args.mode.into());
            }
            // The command line overrides the saved switch in either
            // direction, and says nothing when it is not given: the receiver
            // then does what it was last told in the interface.
            let named = |p: &Option<PathBuf>| p.clone().filter(|d| !d.as_os_str().is_empty());
            if args.no_packet_log || args.packet_log.is_some() {
                app.set_packet_log(args.no_packet_log, named(&args.packet_log));
            }
            if args.no_survey || args.survey.is_some() {
                app.set_survey(args.no_survey, named(&args.survey));
            }
            if let Some(t) = args.gps.clone() {
                app.set_gps(Some(t));
            }
            if let Some((lat, lon)) = args.location {
                app.set_location(lat, lon);
            }
            if let Some(mut p) = args.ha_broker.clone() {
                p.spaces = args.ha_spaces.clone().unwrap_or_default();
                app.publish_to(p);
            }
            #[cfg(feature = "mcp")]
            if let Some(addr) = args.mcp_listen {
                if let Err(e) = app.serve_mcp(addr, &cc.egui_ctx) {
                    eprintln!("--mcp-listen {addr}: {e}");
                    std::process::exit(1);
                }
            }
            app.shot = args.shot.clone();
            app.shot_after = args.shot_after;
            app.set_print_log(args.print_log);
            if let Some(dir) = args.record.clone() {
                app.record_to(dir, args.record_mb);
            }
            if args.capture_iq {
                app.set_capture(true);
            }
            if args.gain {
                app.show_radio_settings();
            }
            if args.scanners {
                app.show_scanner_settings();
            }
            if args.setup {
                app.show_setup();
            }
            if let Some(mhz) = args.center {
                app.set_center(mhz);
            }
            if args.chain {
                app.show_chain();
            }
            if args.flights {
                app.show_map();
            }
            if args.calls {
                app.show_calls();
            }
            if args.messages {
                app.show_messages();
            }
            if args.transcript {
                app.show_transcript(None);
            }
            if args.video {
                app.show_video();
            }
            if args.links {
                app.show_links();
            }
            if args.control {
                app.show_control();
            }
            if args.run {
                app.start_on_open();
            }
            app.soak = args.soak;
            Ok(Box::new(app))
        }),
    )
}
