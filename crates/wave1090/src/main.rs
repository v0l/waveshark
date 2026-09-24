//! A 1090 MHz receiver that speaks dump1090's network protocols.
//!
//! Same ports, same formats, so anything already fed by dump1090 (piaware,
//! readsb, fr24feed, VRS, a Beast client) can be pointed here without
//! changing. What is different is the demodulator: `dsp::ModeSDetector` frames
//! on the preamble and then on the parity, sliced at quarter-sample offsets,
//! which off the same samples reads more DF17 than dump1090 does.
//!
//! The samples can come from a dongle on this machine or, unlike dump1090,
//! from an iqstream server somewhere else, which is how one aerial feeds
//! several receivers.

mod clock;
mod json;
mod net;
mod sbs;
mod stats;
mod track;

use anyhow::{Context, Result, bail};
use clap::Parser;
use common::device::{Device, GainMode};
use common::{Hz, SampleFormat, Sps};
use decode::adsb::{self, AddressBook};
use dsp::{ModeSConfig, ModeSDetector, ModeSFrame};

/// The frequency this is about. Present as a flag because dump1090 has one,
/// and because a downconverter puts the band somewhere else.
const MODE_S_HZ: u64 = 1_090_000_000;

/// How finely the parity search slices, as a word rather than a number
#[derive(Clone, Copy, PartialEq, clap::ValueEnum)]
enum Search {
    Off,
    Coarse,
    Fine,
}

impl Search {
    fn config(self, preamble_ratio: f32) -> ModeSConfig {
        let base = ModeSConfig { preamble_ratio, ..ModeSConfig::default() };
        match self {
            Search::Off => ModeSConfig { crc_framing: false, ..base },
            Search::Coarse => ModeSConfig { phase_step: 0.5, ..base },
            Search::Fine => base,
        }
    }
}

#[derive(Parser)]
#[command(name = "wave1090", version, about = "Mode S and ADS-B, speaking dump1090's protocols")]
struct Args {
    /// RTL-SDR to use, by index or serial
    #[arg(long, value_name = "INDEX", default_value = "0")]
    device_index: String,

    /// Read samples from an iqstream server instead of a local dongle, as
    /// host:port. One aerial can feed several receivers this way
    #[arg(long, value_name = "HOST:PORT")]
    iqstream: Option<String>,

    /// Read a recorded capture instead of a radio, at the rate in its name
    #[arg(long, value_name = "FILE")]
    file: Option<std::path::PathBuf>,

    /// Tuner gain in dB, or -10 for the dongle's own control
    #[arg(long, value_name = "DB", default_value_t = 49.6, allow_negative_numbers = true)]
    gain: f32,

    /// Frequency to listen on, in Hz
    #[arg(long, value_name = "HZ", default_value_t = MODE_S_HZ)]
    freq: u64,

    /// Sample rate in Hz. 2.4 MS/s is what the demodulator is tuned for
    #[arg(long, value_name = "HZ", default_value_t = 2_400_000)]
    sample_rate: u32,

    /// Correct the dongle's reference oscillator, in parts per million
    #[arg(long, value_name = "PPM", default_value_t = 0.0, allow_negative_numbers = true)]
    ppm: f64,

    /// Enable the dongle's own gain control rather than a fixed gain
    #[arg(long)]
    enable_agc: bool,

    /// Print every frame as AVR hex on standard output
    #[arg(long)]
    raw: bool,

    /// Address the network ports are served on
    #[arg(long, value_name = "ADDR", default_value = "0.0.0.0")]
    net_bind_address: String,

    /// AVR hex output, dump1090's 30002. Zero to serve none
    #[arg(long, value_name = "PORT", default_value_t = 30002)]
    net_ro_port: u16,

    /// BaseStation output, dump1090's 30003. Zero to serve none
    #[arg(long, value_name = "PORT", default_value_t = 30003)]
    net_sbs_port: u16,

    /// Beast binary output, dump1090's 30005. Zero to serve none
    #[arg(long, value_name = "PORT", default_value_t = 30005)]
    net_bo_port: u16,

    /// Take frames from somebody else on these ports, comma separated, and
    /// republish them. An mlat client hands its results back this way
    #[arg(long, value_name = "PORTS", value_delimiter = ',')]
    net_bi_port: Vec<u16>,

    /// Where this receiver is, in degrees. A position frame then resolves
    /// against it rather than waiting for the other half of its pair
    #[arg(long, value_name = "DEG", allow_negative_numbers = true)]
    lat: Option<f64>,

    /// Where this receiver is, in degrees
    #[arg(long, value_name = "DEG", allow_negative_numbers = true)]
    lon: Option<f64>,

    /// Serve the samples on to other receivers over iqstream, as addr:port
    /// or a bare port. One aerial then feeds this and whatever else wants it
    #[arg(long, value_name = "ADDR")]
    iqstream_listen: Option<String>,

    /// List the iqstream server in the public directory on nostr, so another
    /// receiver can find it. Published at start and every 24 hours, and
    /// withdrawn on SIGINT or SIGTERM
    #[arg(long, requires = "iqstream_listen")]
    iqstream_list: bool,

    /// What the station is called in the directory
    #[arg(long, value_name = "NAME", default_value = "wave1090")]
    iqstream_name: String,

    /// A line about the station, shown in the directory
    #[arg(long, value_name = "TEXT", default_value = "")]
    iqstream_description: String,

    /// The antenna, shown in the directory
    #[arg(long, value_name = "TEXT", default_value = "")]
    iqstream_antenna: String,

    /// The host other receivers reach this one at. Without it the router is
    /// asked to open the port, over UPnP, PCP or NAT-PMP, and the address it
    /// gives is listed
    #[arg(long, value_name = "HOST")]
    iqstream_public_host: Option<String>,

    /// List --lat and --lon as well, to about five kilometres
    #[arg(long, requires = "lat", requires = "lon")]
    iqstream_locate: bool,

    /// Where the key the listing is signed with is kept, made on first use.
    /// ~/.config/wave1090/directory.nsec when not given
    #[arg(long, value_name = "FILE")]
    iqstream_key: Option<std::path::PathBuf>,

    /// Nostr relays to list on, comma separated. Four public ones by default
    #[arg(long, value_name = "URL", value_delimiter = ',')]
    iqstream_relay: Vec<String>,

    /// How hard the parity search looks, which is what this costs.
    ///
    /// `fine` reads more than dump1090 and wants a third of a fast core;
    /// `coarse` draws level with it for half that; `off` leaves the parity
    /// search out and reads what a preamble search alone finds
    #[arg(long, value_name = "HOW", default_value = "fine")]
    parity_search: Search,

    /// Write tar1090's JSON into this directory: the aircraft, receiver,
    /// history and statistics files a web map reads. Somewhere in RAM,
    /// as dump1090's own packages use /run
    #[arg(long, value_name = "DIR")]
    write_json: Option<std::path::PathBuf>,

    /// How often aircraft.json is rewritten, in seconds
    #[arg(long, value_name = "SECS", default_value_t = 1.0)]
    write_json_every: f64,

    /// How far a preamble must stand above the quiet slots between its
    /// pulses. Lower reads more of the replies that overlay their address on
    /// the parity, and costs processor: 1.75 reads 5% more frames for 16% more
    /// of a fast core and 23% of a Raspberry Pi 4's
    #[arg(long, value_name = "RATIO", default_value_t = ModeSConfig::default().preamble_ratio)]
    preamble_ratio: f32,

    /// Log only faults. The log goes to standard error, every five seconds
    /// with what was read and who is connected; `RUST_LOG` sets it finer
    #[arg(long)]
    quiet: bool,
}

fn main() -> Result<()> {
    let args = Args::parse();
    let level = if args.quiet { "warn" } else { "info" };
    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| tracing_subscriber::EnvFilter::new(level)),
        )
        .with_writer(std::io::stderr)
        .with_ansi(std::io::IsTerminal::is_terminal(&std::io::stderr()))
        .with_target(false)
        .init();

    let ports = Ports {
        avr: serve(&args, "AVR", args.net_ro_port)?,
        sbs: serve(&args, "SBS", args.net_sbs_port)?,
        beast: serve(&args, "Beast", args.net_bo_port)?,
    };
    for (name, port) in [("AVR", &ports.avr), ("SBS", &ports.sbs), ("Beast", &ports.beast)] {
        if let Some(p) = port {
            tracing::info!("{name} on {}", p.addr());
        }
    }

    let incoming: Vec<std::sync::mpsc::Receiver<Vec<u8>>> = args
        .net_bi_port
        .iter()
        .filter(|p| **p > 0)
        .map(|port| {
            let rx = net::accept_frames(&args.net_bind_address, *port)
                .with_context(|| format!("cannot serve port {port}"))?;
            tracing::info!("input on {}:{port}", args.net_bind_address);
            Ok(rx)
        })
        .collect::<Result<_>>()?;

    match (&args.file, &args.iqstream) {
        (Some(path), _) => from_file(&args, path, ports),
        (None, Some(addr)) => {
            let dev =
                remote::iqstream::Device::open(addr).with_context(|| format!("iqstream {addr}"))?;
            from_radio(&args, Box::new(dev), true, ports, incoming)
        }
        (None, None) => {
            let dev = rtlsdr::RtlSdr::open_by_id(&args.device_index)
                .with_context(|| format!("no RTL-SDR {}", args.device_index))?;
            from_radio(&args, Box::new(dev), false, ports, incoming)
        }
    }
}

/// Where the aerial is, for the cheap half of CPR.
fn station(args: &Args) -> Option<(f64, f64)> {
    args.lat.zip(args.lon)
}

/// The web map's files, where an operator asked for them.
fn writer(args: &Args) -> Result<Option<json::Writer>> {
    let Some(dir) = &args.write_json else { return Ok(None) };
    if !args.write_json_every.is_finite() || args.write_json_every <= 0.0 {
        bail!("--write-json-every wants a positive number of seconds");
    }
    let every = std::time::Duration::from_secs_f64(args.write_json_every);
    let out = json::Writer::new(dir, every, station(args))
        .with_context(|| format!("cannot write JSON into {}", dir.display()))?;
    if !args.quiet {
        println!("JSON   in {} every {:.3} s", dir.display(), every.as_secs_f64());
    }
    Ok(Some(out))
}

/// Where a decoded frame goes
struct Ports {
    avr: Option<net::Fanout>,
    sbs: Option<net::Fanout>,
    beast: Option<net::Fanout>,
}

fn serve(args: &Args, name: &'static str, port: u16) -> Result<Option<net::Fanout>> {
    if port == 0 {
        return Ok(None);
    }
    Ok(Some(
        net::Fanout::serve(name, &args.net_bind_address, port)
            .with_context(|| format!("cannot serve port {port}"))?,
    ))
}

/// The iqstream server this receiver fans its own samples out on, if asked.
///
/// A subscriber here is reading the same tuner, which is how one aerial feeds
/// this and a second receiver at once. Not tunable: the dial belongs to
/// whoever this is decoding for.
fn listen(args: &Args, center_hz: u64, rate: f64, hardware: &str) -> Result<Option<Fanned>> {
    let Some(spec) = &args.iqstream_listen else { return Ok(None) };
    let addr: std::net::SocketAddr = match spec.parse() {
        Ok(a) => a,
        Err(_) => match spec.parse::<u16>() {
            Ok(port) => ([0, 0, 0, 0], port).into(),
            Err(_) => bail!("--iqstream-listen wants addr:port or a port, not {spec:?}"),
        },
    };
    let cfg = iqstream::ServerConfig::single(
        "wave1090",
        iqstream::StreamConfig {
            name: "span".into(),
            center_hz,
            sample_rate: rate as u32,
            gain_db: Some(args.gain),
            tunable: false,
            tune_range_hz: None,
            settings: Vec::new(),
        },
    );
    let server = iqstream::Server::start(addr, cfg).context("cannot serve iqstream")?;
    tracing::info!("iqstream on {}", server.addr());
    let tuner = server.default_stream().context("the server kept no tuner")?;
    tuner.set_hardware(hardware);
    if args.iqstream_list {
        let listing = listing(args)?;
        let shared = server.clone();
        let lister = iqdirectory::lister::Lister::start(move || Some(shared.clone()), listing)
            .context("cannot start the directory listing")?;
        withdraw_on_signal(lister)?;
    }
    Ok(Some(Fanned { server, tuner }))
}

fn key_path(args: &Args) -> Result<std::path::PathBuf> {
    if let Some(p) = &args.iqstream_key {
        return Ok(p.clone());
    }
    let base = std::env::var_os("XDG_CONFIG_HOME")
        .map(std::path::PathBuf::from)
        .or_else(|| std::env::var_os("HOME").map(|h| std::path::PathBuf::from(h).join(".config")))
        .or_else(|| std::env::var_os("APPDATA").map(std::path::PathBuf::from))
        .context("no home to keep the directory key in, so name it with --iqstream-key")?;
    Ok(base.join("wave1090").join("directory.nsec"))
}

fn listing(args: &Args) -> Result<iqdirectory::lister::Listing> {
    let path = key_path(args)?;
    let keys = iqdirectory::identity_file(&path)
        .with_context(|| format!("directory key {}", path.display()))?;
    tracing::info!("iqstream directory key {}", iqdirectory::npub(&keys));
    let nsec = iqdirectory::nsec(&keys).context("the directory key would not encode")?;
    let relays = match args.iqstream_relay.is_empty() {
        true => iqdirectory::RELAYS.iter().map(|r| r.to_string()).collect(),
        false => args.iqstream_relay.clone(),
    };
    Ok(iqdirectory::lister::Listing {
        name: args.iqstream_name.clone(),
        description: args.iqstream_description.clone(),
        antenna: args.iqstream_antenna.clone(),
        location: station(args).filter(|_| args.iqstream_locate),
        public_host: args.iqstream_public_host.clone(),
        nsec,
        relays,
    })
}

fn withdraw_on_signal(lister: iqdirectory::lister::Lister) -> Result<()> {
    std::thread::Builder::new()
        .name("signals".into())
        .spawn(move || {
            let Ok(rt) = tokio::runtime::Builder::new_current_thread().enable_all().build() else {
                return;
            };
            rt.block_on(stopped());
            tracing::info!("withdrawing the directory listing");
            lister.withdraw(std::time::Duration::from_secs(3));
            std::process::exit(0);
        })
        .context("cannot watch for signals")?;
    Ok(())
}

#[cfg(unix)]
async fn stopped() {
    use tokio::signal::unix::{SignalKind, signal};
    let Ok(mut term) = signal(SignalKind::terminate()) else {
        let _ = tokio::signal::ctrl_c().await;
        return;
    };
    tokio::select! {
        _ = tokio::signal::ctrl_c() => {}
        _ = term.recv() => {}
    }
}

#[cfg(not(unix))]
async fn stopped() {
    let _ = tokio::signal::ctrl_c().await;
}

/// The one tuner this receiver serves, and the server holding the port open
struct Fanned {
    // held only to keep the port open: dropping the server ends its thread
    #[allow(dead_code)]
    server: std::sync::Arc<iqstream::Server>,
    tuner: std::sync::Arc<iqstream::Stream>,
}

/// How long a radio may deliver nothing before it is treated as stopped.
///
/// At 2.4 MS/s a working tuner hands over a block every 27 ms, so seconds of
/// nothing is not a slow moment. Long enough that a remote source reconnecting
/// is not mistaken for a dead one.
const SILENCE: std::time::Duration = std::time::Duration::from_secs(5);

/// When a source last delivered a sample
struct Silence(std::time::Instant);

impl Default for Silence {
    fn default() -> Self {
        Self(std::time::Instant::now())
    }
}

impl Silence {
    /// How long the radio has been quiet, once that is long enough to call it
    /// stopped.
    fn stalled(&mut self, samples: usize) -> Option<std::time::Duration> {
        if samples > 0 {
            self.0 = std::time::Instant::now();
            return None;
        }
        let quiet = self.0.elapsed();
        (quiet >= SILENCE).then_some(quiet)
    }
}

/// The state a run carries between blocks.
struct Reader {
    det: ModeSDetector,
    book: AddressBook,
    track: track::Tracker,
    stats: stats::Stats,
    json: Option<json::Writer>,
    frames: Vec<ModeSFrame>,
    clock: clock::Clock,
    raw: bool,
    read: u64,
    kept: u64,
    /// Samples the source produced that never reached the demodulator
    dropped: u64,
    /// Where the samples are fanned out, and the buffer they are packed into
    server: Option<Fanned>,
    uc8: Vec<u8>,
}

impl Reader {
    fn new(rate: f64, raw: bool, cfg: ModeSConfig) -> Self {
        Self {
            det: ModeSDetector::new(rate, cfg),
            book: AddressBook::new(),
            track: track::Tracker::default(),
            stats: stats::Stats::new(unix(chrono::Utc::now())),
            json: None,
            frames: Vec::new(),
            clock: clock::Clock::new(rate),
            raw,
            read: 0,
            kept: 0,
            dropped: 0,
            server: None,
            uc8: Vec::new(),
        }
    }

    /// One block of samples in, whatever it held out on every port.
    ///
    /// `seq` is where the block sits in the stream the source produced,
    /// dropped samples included, which is what makes the Beast clock keep
    /// time rather than count what happened to arrive.
    fn block(&mut self, seq: u64, iq: &[common::C32], ports: &Ports) {
        self.read += iq.len() as u64;
        let mut lost = 0;
        if let clock::Step::Broke(n) = self.clock.block(seq, iq.len()) {
            // What the demodulator is carrying ended before the break, and a
            // splice between two bursts frames as a preamble nobody sent.
            self.det.reset();
            self.dropped += n;
            lost = n;
        }
        self.stats.samples(iq.len() as u64, lost);
        // Before the decoding, so a subscriber's copy is not delayed by it,
        // and only where somebody is connected: packing costs a pass over
        // every sample.
        if let Some(fanned) = &self.server
            && fanned.tuner.subscribers() > 0
        {
            SampleFormat::Cu8.encode(iq, &mut self.uc8);
            fanned.tuner.push(&self.uc8);
            self.uc8.clear();
        }
        let Self { det, book, frames, .. } = self;
        frames.clear();
        let book = std::cell::RefCell::new(book);
        det.process_valid(iq, frames, &|f: &ModeSFrame| {
            book.borrow_mut().accept(&f.bytes, f.preamble_ratio)
        });

        let now = chrono::Utc::now();
        // Taken out of the way, because publishing borrows the rest of the
        // reader: the buffer goes back afterwards so a block costs no
        // allocation.
        let frames = std::mem::take(&mut self.frames);
        for f in &frames {
            // Correcting a flipped bit is arithmetic on the frame rather than
            // signal processing, so it happens here, as it does in the
            // receiver's own Mode S node.
            let fixed = match f.bytes[0] >> 3 {
                17 | 18 => adsb::fix_single_bit(&f.bytes),
                _ => None,
            };
            let corrected = fixed.as_ref().is_some_and(|b| *b != f.bytes) as usize;
            let bytes = fixed.unwrap_or_else(|| f.bytes.clone());
            let at = self.clock.at(f.at_sample, f.at_frac);
            let heard =
                Heard { clock: at, rssi_dbfs: f.rssi_dbfs, from: stats::Source::Air, corrected };
            self.publish(&bytes, heard, ports, now);
        }
        self.frames = frames;
        let at = std::time::Instant::now();
        let single = self.track.expire(at);
        self.stats.expired(single);
        self.write_json(now, false);
    }

    /// The web map's files, where one is due or the run is ending.
    fn write_json(&mut self, now: chrono::DateTime<chrono::Utc>, ending: bool) {
        let unix = unix(now);
        let at = std::time::Instant::now();
        let rolled = self.stats.tick(unix, at);
        let Some(w) = &mut self.json else { return };
        if rolled {
            w.minute();
        }
        let wrote = match ending {
            true => w.flush(&self.track, &self.stats, unix, at),
            false => w.tick(&self.track, &self.stats, unix, at),
        };
        if let Err(e) = wrote {
            eprintln!("cannot write the JSON: {e}");
        }
    }

    /// One frame out on every port, whether it was heard here or handed back
    /// by somebody else.
    fn publish(
        &mut self,
        bytes: &[u8],
        heard: Heard,
        ports: &Ports,
        now: chrono::DateTime<chrono::Utc>,
    ) {
        let Ok(frame) = adsb::parse(bytes) else { return };
        self.kept += 1;
        let Heard { clock, rssi_dbfs, from, corrected } = heard;
        self.stats.frame(frame.df, rssi_dbfs, from, corrected);
        if let Some(seen) = self.track.accept(&frame, rssi_dbfs) {
            self.stats.seen(&seen);
        }
        if self.raw || ports.avr.is_some() {
            let line = net::avr(bytes);
            if self.raw {
                print!("{line}");
            }
            if let Some(p) = &ports.avr {
                p.send(line.as_bytes());
            }
        }
        // A frame with no time is left off Beast rather than sent with the
        // sentinel: mlat-client reads that value as a timestamp 23456248
        // seconds in and counts the frame as an outlier against its fit,
        // where a frame it never saw costs it nothing. AVR and SBS carry no
        // clock, so the frame is still fed (#154).
        if let Some(p) = &ports.beast
            && clock != clock::UNTIMED
        {
            p.send(&net::beast(bytes, clock, rssi_dbfs));
        }
        if let Some(p) = &ports.sbs {
            for line in sbs::lines(&self.track, &frame, now) {
                p.send(format!("{line}\r\n").as_bytes());
            }
        }
    }
}

/// What a frame arrived as, beside the bytes
struct Heard {
    clock: u64,
    rssi_dbfs: f32,
    from: stats::Source,
    corrected: usize,
}

/// The time a file's readers count in: seconds since the epoch.
fn unix(now: chrono::DateTime<chrono::Utc>) -> f64 {
    now.timestamp_millis() as f64 / 1000.0
}

fn from_radio(
    args: &Args,
    mut dev: Box<dyn Device>,
    remote: bool,
    ports: Ports,
    incoming: Vec<std::sync::mpsc::Receiver<Vec<u8>>>,
) -> Result<()> {
    // A remote server owns its own tuner: the rate and the dial are whatever
    // it is already streaming, and asking is how a shared dongle gets shared.
    if !remote {
        dev.set_rate(Sps(args.sample_rate as u64)).context("the radio refused that rate")?;
        dev.correct(args.ppm);
        dev.set_dial(Hz(args.freq)).context("the radio refused that frequency")?;
        let gain = match args.enable_agc {
            true => GainMode::Auto,
            false => GainMode::Manual(args.gain),
        };
        let _ = dev.set_gain("tuner", gain);
    }
    let rate = dev.rate().0 as f64;
    let center = dev.center().0;
    tracing::info!(
        "{} at {:.4} MHz, {:.3} MS/s, preamble gate {}",
        dev.info().label,
        center as f64 / 1e6,
        rate / 1e6,
        args.preamble_ratio
    );
    if (center as i64 - args.freq as i64).abs() > 1_000_000 {
        bail!("the radio is on {:.4} MHz, not 1090", center as f64 / 1e6);
    }
    if rate < 2_000_000.0 {
        bail!("{:.3} MS/s is too slow to read a microsecond wide chip", rate / 1e6);
    }

    let mut stream = dev.start_rx().context("the radio would not start")?;
    let mut reader = Reader::new(rate, args.raw, args.parity_search.config(args.preamble_ratio));
    reader.server = listen(args, center, rate, dev.info().kind.as_str())?;
    reader.track.here = station(args);
    reader.json = writer(args)?;
    let mut stats = Stats::default();
    let mut heard = Silence::default();
    loop {
        let buf = match stream.read() {
            Ok(b) => b,
            Err(e) => bail!("the radio stopped: {e}"),
        };
        // A dongle that falls off the USB bus does not fail a read: it hands
        // back nothing, for ever, and a loop that only watches for an error
        // spins on a core feeding nobody while systemd sees a healthy service.
        // Measured on radarpi: 18 minutes at 82% of a core after the tuner
        // stopped answering, with every feeder reconnecting every 120 s.
        if let Some(quiet) = heard.stalled(buf.samples.len()) {
            bail!("the radio delivered no samples for {:.0} s", quiet.as_secs_f64());
        }
        if buf.samples.is_empty() {
            // Nothing to decode, and nothing to gain from asking again at the
            // speed of the processor.
            std::thread::sleep(std::time::Duration::from_millis(20));
            continue;
        }
        reader.block(buf.seq, &buf.samples, &ports);
        // Whatever came back from an mlat client since the last block, put
        // out as though it had been heard here, which is what
        // `--forward-mlat` means to everything downstream. Not with a
        // timestamp of this receiver's, though: the frame was computed
        // somewhere else and was never at a sample index here, so it goes out
        // untimed and a client leaves it out of its clock fit.
        for rx in &incoming {
            while let Ok(bytes) = rx.try_recv() {
                let heard = Heard {
                    clock: clock::UNTIMED,
                    rssi_dbfs: f32::NEG_INFINITY,
                    from: stats::Source::Network,
                    corrected: 0,
                };
                reader.publish(&bytes, heard, &ports, chrono::Utc::now());
            }
        }
        stats.tick(&reader, stream.dropped(), &ports);
    }
}

/// What the receiver says about itself every five seconds.
struct Stats {
    said: std::time::Instant,
    kept: u64,
    dropped: u64,
    gaps: u64,
}

impl Default for Stats {
    fn default() -> Self {
        Self { said: std::time::Instant::now(), kept: 0, dropped: 0, gaps: 0 }
    }
}

impl Stats {
    const EVERY: std::time::Duration = std::time::Duration::from_secs(5);

    /// A line of what happened since the last one, once the interval is up.
    fn tick(&mut self, reader: &Reader, dropped: u64, ports: &Ports) {
        let since = self.said.elapsed();
        if since < Self::EVERY {
            return;
        }
        let frames = reader.kept - self.kept;
        let lost = dropped - self.dropped;
        let gaps = reader.dropped - self.gaps;
        let count = |p: &Option<net::Fanout>| p.as_ref().map_or(0, |p| p.clients());
        let iq = reader.server.as_ref().map_or(0, |f| f.tuner.subscribers());
        tracing::info!(
            "{frames} frames, {:.0}/s, {} aircraft, {lost} samples dropped, {gaps} gaps; \
             avr {} sbs {} beast {} iqstream {iq}",
            frames as f64 / since.as_secs_f64(),
            reader.book.len(),
            count(&ports.avr),
            count(&ports.sbs),
            count(&ports.beast),
        );
        self.said = std::time::Instant::now();
        self.kept = reader.kept;
        self.dropped = dropped;
        self.gaps = reader.dropped;
    }
}

fn from_file(args: &Args, path: &std::path::Path, ports: Ports) -> Result<()> {
    let src = sources::FileSource::open(path).with_context(|| format!("{}", path.display()))?;
    let buf = src.read_all().context("reading the capture")?;
    let rate = buf.rate.as_f64();
    tracing::info!(
        "{} at {:.4} MHz, {:.3} MS/s, {:.1} s, preamble gate {}",
        path.display(),
        buf.center.as_f64() / 1e6,
        rate / 1e6,
        buf.samples.len() as f64 / rate,
        args.preamble_ratio
    );
    let mut reader = Reader::new(rate, args.raw, args.parity_search.config(args.preamble_ratio));
    reader.server = listen(args, buf.center.0, rate, "file")?;
    reader.track.here = station(args);
    reader.json = writer(args)?;
    for (n, block) in buf.samples.chunks(65_536).enumerate() {
        reader.block((n * 65_536) as u64, block, &ports);
    }
    reader.write_json(chrono::Utc::now(), true);
    tracing::info!("{} frames, {} aircraft", reader.kept, reader.book.len());
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;
    use std::io::Read;

    fn parsed(extra: &[&str]) -> std::result::Result<Args, clap::Error> {
        Args::try_parse_from(std::iter::once("wave1090").chain(extra.iter().copied()))
    }

    #[test]
    fn a_listing_needs_a_server_and_a_location_needs_a_position() {
        let err = |a: &[&str]| parsed(a).err().map(|e| e.kind());
        assert_eq!(
            err(&["--iqstream-list"]),
            Some(clap::error::ErrorKind::MissingRequiredArgument)
        );
        assert_eq!(
            err(&["--iqstream-listen", "1234", "--iqstream-list", "--iqstream-locate"]),
            Some(clap::error::ErrorKind::MissingRequiredArgument)
        );
        assert_eq!(err(&["--iqstream-listen", "1234", "--iqstream-list"]), None);
    }

    #[test]
    fn the_listing_takes_its_key_from_a_file_made_once_and_the_position_only_when_asked() {
        let dir = std::env::temp_dir().join(format!("wave1090-key-{}", std::process::id()));
        let key = dir.join("directory.nsec");
        let key = key.to_str().unwrap();
        let base = ["--iqstream-listen", "1234", "--iqstream-list", "--iqstream-key", key];
        let at = ["--lat", "51.45", "--lon", "-0.97"];
        let args = parsed(&[&base[..], &at[..]].concat()).unwrap();
        let first = listing(&args).unwrap();
        assert_eq!(first.name, "wave1090");
        assert_eq!(first.location, None, "a position given for CPR is not listed unasked");
        assert_eq!(first.relays.len(), iqdirectory::RELAYS.len());
        assert_eq!(first.public_host, None);

        let named = [
            "--iqstream-locate",
            "--iqstream-name",
            "radarpi",
            "--iqstream-public-host",
            "sdr.example.net",
            "--iqstream-relay",
            "wss://a.example,wss://b.example",
        ];
        let args = parsed(&[&base[..], &at[..], &named[..]].concat()).unwrap();
        let second = listing(&args).unwrap();
        assert_eq!(second.nsec, first.nsec, "the same key on the second start");
        assert_eq!(second.location, Some((51.45, -0.97)));
        assert_eq!(second.name, "radarpi");
        assert_eq!(second.public_host.as_deref(), Some("sdr.example.net"));
        assert_eq!(second.relays, ["wss://a.example", "wss://b.example"]);
        std::fs::remove_dir_all(&dir).unwrap();
    }

    const CAPTURE: &str = "../../testdata/adsb_1090M_2400k.cu8";
    /// Ten seconds off radarpi, with dump1090's own Beast timestamps beside it
    const BUSY: &str = "../../testdata/adsb_radarpi_1090M_2400k.cu8";
    const BUSY_REFERENCE: &str = "../../testdata/adsb_radarpi_1090M_2400k.dump1090.ts";
    /// A block of the capture with no frame in it, so dropping it can be
    /// compared frame for frame against not dropping it.
    const HOLE: usize = 73;

    fn capture() -> Option<common::IqBuf> {
        let path = std::path::Path::new(CAPTURE);
        if !path.exists() {
            eprintln!("skipping: no {CAPTURE}, run testdata/fetch.sh");
            return None;
        }
        Some(sources::FileSource::open(path).unwrap().read_all().unwrap())
    }

    /// The capture in the 65536 sample blocks a dongle hands over, each with
    /// where it sits in the stream.
    fn blocks(buf: &common::IqBuf) -> Vec<(u64, Vec<common::C32>)> {
        buf.samples
            .chunks(65_536)
            .enumerate()
            .map(|(n, b)| ((n * 65_536) as u64, b.to_vec()))
            .collect()
    }

    /// Every Beast frame the receiver put on the wire, as timestamp and body.
    ///
    /// Through the socket rather than around it, because a timestamp is only
    /// right if it survives the escaping as well as the arithmetic.
    fn beast_over_the_wire(blocks: &[(u64, Vec<common::C32>)], rate: f64) -> Vec<(u64, Vec<u8>)> {
        beast_at(blocks, rate, ModeSConfig::default())
    }

    /// The same, with the detector set some other way.
    fn beast_at(
        blocks: &[(u64, Vec<common::C32>)],
        rate: f64,
        cfg: ModeSConfig,
    ) -> Vec<(u64, Vec<u8>)> {
        let fanout = net::Fanout::serve("test", "127.0.0.1", 0).expect("a port");
        let mut client = std::net::TcpStream::connect(fanout.addr()).expect("connect");
        client.set_read_timeout(Some(std::time::Duration::from_millis(500))).unwrap();
        // The listener accepts on its own thread, so wait for it to hold the
        // socket before anything is sent to nobody.
        std::thread::sleep(std::time::Duration::from_millis(100));
        let reader = std::thread::spawn(move || {
            let (mut buf, mut chunk) = (Vec::new(), [0u8; 65_536]);
            while let Ok(n) = client.read(&mut chunk) {
                if n == 0 {
                    break;
                }
                buf.extend_from_slice(&chunk[..n]);
            }
            buf
        });

        let ports = Ports { avr: None, sbs: None, beast: Some(fanout) };
        let mut rx = Reader::new(rate, false, cfg);
        for (seq, iq) in blocks {
            rx.block(*seq, iq, &ports);
        }
        drop(ports);
        let bytes = reader.join().expect("the client thread");

        let mut out = Vec::new();
        let mut at = 0usize;
        while let Some((frame, used)) = net::next_frame(&bytes[at..]) {
            if let Some(f) = frame {
                out.push((timestamp(&bytes[at + 2..]), f));
            }
            at += used;
        }
        out
    }

    /// The six bytes of clock at the head of a Beast body, unescaped.
    fn timestamp(body: &[u8]) -> u64 {
        let (mut ts, mut i) = (0u64, 0usize);
        for _ in 0..6 {
            i += (body[i] == 0x1a) as usize;
            ts = (ts << 8) | body[i] as u64;
            i += 1;
        }
        ts
    }

    /// The whole point of the Beast timestamp: it is a clock.
    ///
    /// Four seconds of the 1090 MHz capture, and every frame's timestamp is
    /// the sample it was heard at times five, in order and never repeated.
    #[test]
    fn a_beast_timestamp_is_the_sample_index_at_twelve_megahertz() {
        let Some(buf) = capture() else { return };
        let frames = beast_over_the_wire(&blocks(&buf), buf.rate.as_f64());
        assert_eq!(frames.len(), 76, "frames on 30005");
        // Not merely that they are timed: a frame the clock cannot place is
        // kept off this port entirely, so the sentinel never reaches a client
        // that would fit a line through it.
        assert!(frames.iter().all(|(ts, _)| *ts != clock::UNTIMED), "a frame lost its time");
        assert!(
            frames.windows(2).all(|w| w[0].0 < w[1].0),
            "timestamps out of order: {:?}",
            frames.iter().map(|(t, _)| *t).collect::<Vec<_>>()
        );
        // 2.4 MS/s is five ticks a sample and four seconds is 9.6 M samples,
        // so no frame can be past 48 M ticks.
        let last = frames[frames.len() - 1].0;
        assert!(last < 48_000_000, "the last frame is at {last}");
        // And the clock is finer than the samples it is made from: a whole
        // sample index can only land on a multiple of five, which is 416 ns of
        // quantising on a clock that counts in 83 ns. 51 of the 70 sit off the
        // sample grid, the rest being frames whose peak needed no moving.
        let between =
            frames.iter().filter(|(ts, _)| (ts - clock::BEAST_REPORTS_AT) % 5 != 0).count();
        assert_eq!(between, 54, "timestamps quantised to whole samples");
    }

    /// A frame with no time feeds the networks but not the clock.
    ///
    /// mlat-client reads the sentinel as a timestamp rather than skipping it,
    /// and one frame in 35705 off radarpi cannot be timed, so the rule is that
    /// Beast carries only frames with a clock while AVR carries the lot.
    #[test]
    fn a_frame_the_clock_cannot_place_goes_out_on_avr_and_not_on_beast() {
        let frame =
            [0x8d, 0x40, 0x7c, 0xaf, 0x99, 0x88, 0x68, 0x37, 0xb8, 0x08, 0x2d, 0xd0, 0xde, 0x39];
        let read = |port: &net::Fanout| {
            let mut c = std::net::TcpStream::connect(port.addr()).expect("connect");
            c.set_read_timeout(Some(std::time::Duration::from_millis(250))).unwrap();
            c
        };
        let (avr, beast) = (
            net::Fanout::serve("test", "127.0.0.1", 0).unwrap(),
            net::Fanout::serve("test", "127.0.0.1", 0).unwrap(),
        );
        let (mut avr_client, mut beast_client) = (read(&avr), read(&beast));
        std::thread::sleep(std::time::Duration::from_millis(100));

        let ports = Ports { avr: Some(avr), sbs: None, beast: Some(beast) };
        let mut rx =
            Reader::new(2.4e6, false, Search::Fine.config(ModeSConfig::default().preamble_ratio));
        let heard =
            |clock| Heard { clock, rssi_dbfs: -20.0, from: stats::Source::Air, corrected: 0 };
        rx.publish(&frame, heard(clock::UNTIMED), &ports, chrono::Utc::now());
        rx.publish(&frame, heard(5_000), &ports, chrono::Utc::now());
        drop(ports);

        let mut got = Vec::new();
        let _ = avr_client.read_to_end(&mut got);
        assert_eq!(got.len(), net::avr(&frame).len() * 2, "AVR kept both");
        let mut got = Vec::new();
        let _ = beast_client.read_to_end(&mut got);
        let (one, used) = net::next_frame(&got).expect("a frame");
        assert_eq!(one.as_deref(), Some(&frame[..]), "the timed frame");
        assert_eq!(used, got.len(), "the untimed frame went out as well");
    }

    /// A radio that stops delivering is a radio that stopped.
    ///
    /// The fault this is for: a dongle that falls off the USB bus keeps
    /// answering reads, with nothing in them, so nothing errors and the
    /// process holds the port open and feeds silence to four networks.
    /// Exiting lets systemd restart it, which is the only thing that can
    /// re-open the device.
    #[test]
    fn a_radio_that_hands_back_nothing_for_long_enough_has_stopped() {
        let mut heard = Silence::default();
        assert_eq!(heard.stalled(65_536), None, "samples arrived");
        assert_eq!(heard.stalled(0), None, "one empty read is not a dead radio");

        // Wound back past the limit rather than waited out, so the test costs
        // nothing: the rule is the elapsed time, whoever measures it.
        heard.0 = std::time::Instant::now() - SILENCE - std::time::Duration::from_millis(1);
        let quiet = heard.stalled(0).expect("a radio quiet for longer than the limit");
        assert!(quiet >= SILENCE, "reported {quiet:?}");

        // And a single block puts it right: a source that reconnects carries
        // on rather than ending the process it just fed.
        assert_eq!(heard.stalled(1), None);
        assert_eq!(heard.stalled(0), None);
    }

    /// The timestamps agree with dump1090's to a fraction of a sample.
    ///
    /// The one test in the corpus that can say so: every other reference is a
    /// list of frames, where this one is dump1090-rb 1.0.15's Beast output
    /// over the same file, ticks and all. Frames are matched by payload and
    /// the constant offset between two receivers removed, since an mlat server
    /// solves that away and only the spread is a fault.
    ///
    /// Measured: 0.91 ticks RMS, 76 ns, over 2158 matched frames. A decoder
    /// timing to a whole sample cannot beat 5 ticks at this rate.
    #[test]
    fn the_clock_agrees_with_dump1090_to_a_fraction_of_a_sample() {
        let Some(buf) = busy() else { return };
        let (theirs, ours) = (reference(), beast_over_the_wire(&blocks(&buf), buf.rate.as_f64()));
        assert!(ours.len() >= 3_700, "read only {} frames", ours.len());

        let mut by_payload: std::collections::HashMap<Vec<u8>, Vec<u64>> = Default::default();
        for (ts, f) in &theirs {
            by_payload.entry(f.clone()).or_default().push(*ts);
        }
        // A payload sent twice seconds apart would match the wrong copy, so
        // the nearest in time is taken and anything past a frame's length
        // apart is not treated as the same transmission at all.
        let mut gaps: Vec<i64> = Vec::new();
        for (ts, f) in &ours {
            let Some(cands) = by_payload.get(f) else { continue };
            let Some(near) = cands.iter().min_by_key(|t| t.abs_diff(*ts)) else { continue };
            gaps.push(*ts as i64 - *near as i64);
        }
        assert!(gaps.len() >= 2_000, "only {} frames matched the reference", gaps.len());
        gaps.sort_unstable();
        let offset = gaps[gaps.len() / 2];
        let close: Vec<f64> =
            gaps.iter().map(|g| (g - offset) as f64).filter(|r| r.abs() < 600.0).collect();
        assert!(close.len() >= 2_000, "only {} frames timed against the reference", close.len());
        let rms = (close.iter().map(|r| r * r).sum::<f64>() / close.len() as f64).sqrt();
        assert!(rms <= 1.5, "{rms:.2} ticks RMS against dump1090, over {} frames", close.len());
    }

    /// What each receiver read of the same ten seconds.
    ///
    /// The counts are the ones in #159, and the point of pinning them is that
    /// the shortfall is in the replies that overlay their address on the
    /// parity, which no CRC can frame. dump1090-rb 1.0.15 read 4091 frames,
    /// 1181 of them all-call replies, 825 of which answer a ground station.
    ///
    #[test]
    fn all_call_replies_keep_up_with_dump1090_where_the_comm_b_replies_do_not() {
        let Some(buf) = busy() else { return };
        let (theirs, ours) = (reference(), beast_over_the_wire(&blocks(&buf), buf.rate.as_f64()));
        let count = |frames: &[(u64, Vec<u8>)], df: u8| {
            frames.iter().filter(|(_, f)| f[0] >> 3 == df).count()
        };
        assert_eq!(theirs.len(), 4_091, "the reference decode");
        assert_eq!((count(&theirs, 11), count(&theirs, 17)), (1_181, 1_013));

        // Ahead on the squitters the parity search can frame.
        assert_eq!(count(&ours, 17), 1_109, "DF17, to their 1013");
        // Level on all-call replies, which needs the ones answering a ground
        // station: reading only the ones answering nobody gave 411.
        assert_eq!(count(&ours, 11), 1_144, "DF11, to their 1181");
        // And behind on Comm-B, which is what #159 is about: no CRC can frame
        // a reply that overlays its address on the parity, so the preamble is
        // all it has, and the gate decides how many are read. What a looser
        // one buys is pinned in the test below.
        assert_eq!((count(&ours, 20), count(&ours, 21)), (341, 341), "Comm-B, to their 514/472");
        assert_eq!(ours.len(), 3_722, "frames on the wire, to their 4091");

        // No aircraft of our own invention: every frame names one the
        // reference also saw.
        assert!(strangers(&theirs, &ours).is_empty(), "aircraft nobody else saw");
    }

    /// What `--preamble-ratio` buys, and that it invents nothing.
    ///
    /// The gate is 2.0 by default because of what 1.75 costs a Raspberry Pi
    /// 4, which is the machine running this: 8.3 s against 6.7 s for these
    /// ten seconds, on a core that has about 17% to spare. Anything faster
    /// can afford it, so what it is worth is pinned rather than claimed:
    /// 3923 frames against 3722, the gain almost all replies that overlay
    /// their address, DF20 341 to 378 and DF21 341 to 364. It loses 4D2258,
    /// two frames at the edge of the noise that the looser search blanks
    /// under a reply it hears instead.
    ///
    /// The one thing a looser gate could do is invent an aeroplane. With the
    /// second gate taken out, 1.75 reads a DF18 naming abcf1e and 1.5 reads
    /// four aircraft no other receiver saw, 000000, 67694E, A6FC75 and
    /// ABCF1E. They stay off the wire because a frame naming an aircraft
    /// nothing has proved is held to 2.0 by `adsb::AddressBook` whatever the
    /// search is set to, which is what makes the gate safe to move at all.
    #[test]
    fn a_looser_preamble_gate_reads_more_replies_and_never_another_aircraft() {
        let Some(buf) = busy() else { return };
        let theirs = reference();
        let at = |ratio: f32| {
            let cfg = ModeSConfig { preamble_ratio: ratio, ..ModeSConfig::default() };
            beast_at(&blocks(&buf), buf.rate.as_f64(), cfg)
        };
        let count = |frames: &[(u64, Vec<u8>)], df: u8| {
            frames.iter().filter(|(_, f)| f[0] >> 3 == df).count()
        };

        let loose = at(1.75);
        assert_eq!(loose.len(), 3_923, "frames on the wire at 1.75, to their 4091");
        assert_eq!((count(&loose, 20), count(&loose, 21)), (378, 364), "Comm-B, to their 514/472");
        assert_eq!((count(&loose, 11), count(&loose, 17)), (1_196, 1_159));
        assert!(strangers(&theirs, &loose).is_empty(), "aircraft nobody else saw");

        let looser = at(1.5);
        assert_eq!(looser.len(), 3_986, "frames on the wire at 1.5");
        assert!(strangers(&theirs, &looser).is_empty(), "aircraft nobody else saw");
    }

    /// Frames naming an aircraft the reference decode never saw.
    fn strangers(theirs: &[(u64, Vec<u8>)], ours: &[(u64, Vec<u8>)]) -> Vec<String> {
        let known: std::collections::HashSet<u32> =
            theirs.iter().filter_map(|(_, f)| names(f)).collect();
        ours.iter()
            .filter(|(_, f)| !names(f).is_some_and(|a| known.contains(&a)))
            .map(|(_, f)| f.iter().map(|b| format!("{b:02x}")).collect())
            .collect()
    }

    /// A directory of this test's own, emptied first.
    fn scratch(name: &str) -> std::path::PathBuf {
        let dir = std::env::temp_dir().join(format!("wave1090-{name}-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).expect("a scratch directory");
        dir
    }

    fn read_json(dir: &std::path::Path, name: &str) -> serde_json::Value {
        let bytes = std::fs::read(dir.join(name)).unwrap_or_else(|e| panic!("{name}: {e}"));
        serde_json::from_slice(&bytes).unwrap_or_else(|e| panic!("{name} is not JSON: {e}"))
    }

    /// What tar1090 would draw off ten seconds of Dublin approach.
    #[test]
    fn aircraft_json_holds_the_aircraft_the_capture_named() {
        let Some(buf) = busy() else { return };
        let dir = scratch("aircraft");
        let ports = Ports { avr: None, sbs: None, beast: None };
        let mut rx = Reader::new(
            buf.rate.as_f64(),
            false,
            Search::Fine.config(ModeSConfig::default().preamble_ratio),
        );
        for (seq, iq) in blocks(&buf) {
            rx.block(seq, &iq, &ports);
        }
        rx.json = Some(json::Writer::new(&dir, std::time::Duration::ZERO, None).unwrap());
        rx.write_json(chrono::Utc::now(), true);

        let doc = read_json(&dir, "aircraft.json");
        let list = doc["aircraft"].as_array().expect("an array");
        assert_eq!(list.len(), 53, "aircraft on the list");
        assert_eq!(doc["messages"], 3_722, "messages since the receiver started");
        let with = |key: &str| list.iter().filter(|a| a.get(key).is_some()).count();
        assert_eq!((with("flight"), with("lat")), (36, 47), "callsigns and positions");
        assert_eq!((with("alt_baro"), with("gs"), with("squawk")), (51, 51, 45), "fields");

        // A named aircraft, whole: the same Ryanair flight dump1090-rb read
        // off this file, at the same place and height.
        let ryr = list.iter().find(|a| a["hex"] == "4ca242").expect("4ca242");
        assert_eq!(ryr["flight"], "RYR1AA");
        assert_eq!(ryr["alt_baro"], 27_000);
        assert_eq!(ryr["squawk"], "1254");
        assert_eq!((ryr["lat"].as_f64(), ryr["lon"].as_f64()), (Some(53.55202), Some(-5.6763)));
        assert_eq!((ryr["gs"].as_f64(), ryr["track"].as_f64()), (Some(467.5), Some(98.0)));
        assert_eq!(ryr["messages"], 65, "frames from 4ca242");

        // Nothing on the list that a map cannot draw: every entry is an
        // address, a count and a level, and no key carries a null where a
        // number could not be made.
        for a in list {
            let hex = a["hex"].as_str().expect("a hex address");
            assert_eq!(hex.len(), 6, "{hex}");
            assert!(hex.chars().all(|c| c.is_ascii_hexdigit() && !c.is_ascii_uppercase()), "{hex}");
            assert!(a["messages"].as_u64().is_some_and(|n| n >= 1), "{hex} has no messages");
            assert!(a["seen"].as_f64().is_some_and(|s| s < 60.0), "{hex} is stale");
            // The detector measures the peak of a complex magnitude, which
            // a strong frame carries past full scale where dump1090's
            // squared and clamped level cannot go.
            let rssi = a["rssi"].as_f64().expect("a level");
            assert!((-60.0..3.1).contains(&rssi), "{hex} at {rssi} dBFS");
            assert!(a.as_object().unwrap().values().all(|v| !v.is_null()), "{a} has a null");
        }

        // receiver.json is what tells the map how often to come back, and
        // how many history files it may ask for.
        let recv = read_json(&dir, "receiver.json");
        assert_eq!(recv["refresh"], 0, "the interval this test wrote at");
        assert_eq!(recv["history"], 1, "one copy taken");
        assert_eq!(recv["version"], env!("CARGO_PKG_VERSION"));
        assert_eq!(read_json(&dir, "history_0.json"), doc, "the first copy is the file");

        // And graphs1090 reads these, by these names.
        let stats = read_json(&dir, "stats.json");
        let total = &stats["total"];
        assert_eq!(total["messages"], 3_722);
        assert_eq!(total["cpr"]["global_ok"], 302);
        assert_eq!(total["cpr"]["local_ok"], 0, "no station was given, so no cheap fix");
        assert_eq!(total["tracks"]["all"], 53);
        assert_eq!(total["local"]["samples_processed"], 24_000_000);
        assert_eq!(total["local"]["samples_dropped"], 0);
        let by_df = total["messages_by_df"].as_array().expect("an array of 32");
        assert_eq!(by_df.len(), 32);
        assert_eq!(
            (&by_df[11], &by_df[17], &by_df[20]),
            (&json!(1_144), &json!(1_109), &json!(341))
        );
        let signal = total["local"]["signal"].as_f64().expect("a mean level");
        assert!((-8.0..0.0).contains(&signal), "{signal} dBFS mean");
        assert!(stats["last1min"]["local"].is_object(), "the minute the plugin reads");
        assert!(total["cpu"].as_object().is_some_and(|c| c.is_empty()), "nothing timed the CPU");
    }

    /// The aircraft a frame names, from its address field or over its parity
    fn names(f: &[u8]) -> Option<u32> {
        match f.first()? >> 3 {
            11 | 17 | 18 => Some(((f[1] as u32) << 16) | ((f[2] as u32) << 8) | f[3] as u32),
            _ => adsb::overlaid_address(f),
        }
    }

    fn busy() -> Option<common::IqBuf> {
        let path = std::path::Path::new(BUSY);
        if !path.exists() {
            eprintln!("skipping: no {BUSY}, run testdata/fetch.sh");
            return None;
        }
        Some(sources::FileSource::open(path).unwrap().read_all().unwrap())
    }

    /// dump1090's Beast output over the same file, as ticks and payload
    fn reference() -> Vec<(u64, Vec<u8>)> {
        std::fs::read_to_string(BUSY_REFERENCE)
            .expect("the reference decode is committed, unlike the capture")
            .lines()
            .filter_map(|l| {
                let (ts, hex) = l.split_once(' ')?;
                let bytes = (0..hex.len() / 2)
                    .map(|i| u8::from_str_radix(&hex[i * 2..i * 2 + 2], 16).ok())
                    .collect::<Option<Vec<u8>>>()?;
                Some((ts.parse().ok()?, bytes))
            })
            .collect()
    }

    /// Samples the radio dropped are time that passed.
    ///
    /// The same capture with one block never delivered: every frame after it
    /// keeps the timestamp it had, because the clock is made from the
    /// sequence number the source counts and not from what arrived.
    #[test]
    fn a_dropped_block_does_not_move_the_frames_after_it() {
        let Some(buf) = capture() else { return };
        let rate = buf.rate.as_f64();
        let all = blocks(&buf);
        let whole = beast_over_the_wire(&all, rate);
        let holed: Vec<(u64, Vec<common::C32>)> =
            all.iter().enumerate().filter(|(n, _)| *n != HOLE).map(|(_, b)| b.clone()).collect();
        let gapped = beast_over_the_wire(&holed, rate);

        assert_eq!(whole.len(), 76);
        assert_eq!(gapped.len(), 76, "a frame went with the dropped block");
        let past = (HOLE as u64 + 1) * 65_536 * 5;
        let mut checked = 0;
        for ((a, one), (b, other)) in whole.iter().zip(&gapped) {
            assert_eq!(one, other, "a different frame came out");
            assert_eq!(a, b, "a frame moved by {} ticks", *b as i64 - *a as i64);
            checked += (*a > past) as usize;
        }
        assert_eq!(checked, 44, "frames past the gap");
    }
}
