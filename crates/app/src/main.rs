use clap::{Parser, ValueEnum};
use std::path::PathBuf;

mod prof;
mod wheel;
mod chain;
mod bands;
mod devices;
mod dial;
mod tracks;
mod map;
mod airports;
mod marine;
mod modes;
mod packetlog;
mod chainview;
mod theme;
mod radio;
mod record;
mod session;
mod ui;
mod waterfall;

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
        offset_hz: 0.0,
        demod: mode,
        volume: 1.0,
        // Measuring, not listening: the numbers are the same either way and
        // this can be run over ssh.
        muted: true,
        squelch_db: None,
        agc: true,
    }]));
    std::thread::sleep(std::time::Duration::from_secs(2));

    let mut readings = Vec::new();
    let start = std::time::Instant::now();
    while start.elapsed().as_secs_f32() < 6.0 {
        std::thread::sleep(std::time::Duration::from_millis(50));
        let Some(st) = r.status.channel_state(1) else { continue };
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
            all.iter()
                .find(|d| d.label.to_lowercase().contains(&w.to_lowercase()))
                .cloned()
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
            offset_hz: 0.0,
            demod: radio::Demod::Wfm,
            volume: 1.0,
            muted: true,
            squelch_db: None,
            agc: true,
        }]));
        println!("decoding a WFM channel while measuring");
    }
    let start = std::time::Instant::now();
    let mut n = 0;
    // RDS needs longer than a spectrum check: a station name is four groups
    // and radiotext is sixteen, repeated every couple of seconds.
    let secs = if listen { 25 } else { 8 };
    while start.elapsed().as_secs() < secs {
        let Ok(f) = r.frames.recv_timeout(std::time::Duration::from_secs(3)) else { break };
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
    println!(
        "\n{n} frames in {:.1}s   dropped {dropped}",
        start.elapsed().as_secs_f64()
    );
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
        let mhz = p.center_hz as f64 / 1e6;
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
                p.snr_db,
            );
        }
        for r in reports {
            decoded += 1;
            println!(
                "{when}  {mhz:10.4} MHz  {:>4} pulses  {:>5.1} dB  {:<22} {}",
                pkg.pulses.len(),
                p.snr_db,
                r.model,
                r.fields_line(),
            );
        }
    }
    println!("\n{} burst(s): {decoded} decoded, {silent} unclaimed", bursts.len());
    Ok(())
}

fn replay(path: &str) -> anyhow::Result<()> {
    let path = std::path::Path::new(path);
    if path.extension().and_then(|s| s.to_str()) == Some("srpkt") {
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
                    if r.model == "unknown" {
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
                        r.model,
                        r.bytes.len(),
                        r.detail,
                    );
                }
            }
            Err(e) => println!("{name}: {e}"),
        }
    }
    println!(
        "\n{} capture(s): {decoded} decoded, {unknown} unknown",
        files.len()
    );
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
#[command(name = "super-radio", about = "Software defined radio receiver", version)]
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

    /// Tune here, in MHz, without opening a channel on it
    #[arg(long, value_name = "MHZ")]
    center: Option<f64>,

    /// Open the radio's own controls
    #[arg(long)]
    gain: bool,

    /// Write every burst that decodes into this directory
    #[arg(long, value_name = "DIR", num_args = 0..=1, default_missing_value = "captures")]
    record: Option<PathBuf>,

    /// Where decoded packets are appended as JSON lines. Defaults to
    /// $XDG_DATA_HOME/super-radio/packets
    #[arg(long, value_name = "DIR")]
    packet_log: Option<PathBuf>,

    /// Do not write the packet log
    #[arg(long)]
    no_packet_log: bool,

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

    /// Report what the squelch reads on a frequency
    #[arg(long, value_name = "MHZ", num_args = 0..=1, default_missing_value = "145.5")]
    squelch_probe: Option<f64>,

    /// Report FM multiplex levels
    #[arg(long, value_name = "MHZ", num_args = 0..=1, default_missing_value = "95.8")]
    mpx: Option<f64>,

    /// Leave the centre spur in, for measuring what removing it does
    #[arg(long)]
    no_dc: bool,

    /// Time a retune
    #[arg(long)]
    bench_tune: bool,

    /// Frames delivered while the centre is dragged
    #[arg(long)]
    bench_pan: bool,

    /// Audio chain throughput
    #[arg(long)]
    bench_audio: bool,
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

fn main() -> eframe::Result<()> {
    let args = Args::parse();

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
    }

    let opts = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_inner_size(if args.shot.is_some() { [1400.0, 860.0] } else { [1280.0, 800.0] })
            .with_min_inner_size([800.0, 500.0])
            .with_title("super-radio"),
        ..Default::default()
    };
    eframe::run_native(
        "super-radio",
        opts,
        Box::new(move |cc| {
            let mut app = ui::App::new(cc);
            if let Some(khz) = args.span {
                app.set_span(khz * 1e3);
            }
            for mhz in &args.tune {
                app.tune_to(*mhz, args.mode.into());
            }
            app.set_packet_log(args.no_packet_log, args.packet_log.clone());
            if let Some((lat, lon)) = args.location {
                app.set_location(lat, lon);
            }
            app.shot = args.shot.clone();
            app.shot_after = args.shot_after;
            if let Some(dir) = args.record.clone() {
                app.record_to(dir, args.record_mb);
            }
            if args.gain {
                app.show_radio_settings();
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
            app.soak = args.soak;
            Ok(Box::new(app))
        }),
    )
}
