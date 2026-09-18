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

mod net;
mod sbs;

use anyhow::{Context, Result, bail};
use clap::Parser;
use common::device::{Device, GainMode};
use common::{Hz, SampleFormat, Sps};
use decode::adsb::{self, AddressBook};
use dsp::{ModeSConfig, ModeSDetector, ModeSFrame};

/// The frequency this is about. Present as a flag because dump1090 has one,
/// and because a downconverter puts the band somewhere else.
const MODE_S_HZ: u64 = 1_090_000_000;

/// Mode S counts time in twelfths of a microsecond, and every Beast client
/// reads the timestamp that way.
const BEAST_CLOCK_HZ: f64 = 12_000_000.0;

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

    /// Say nothing on standard output but what was asked for
    #[arg(long)]
    quiet: bool,
}

fn main() -> Result<()> {
    let args = Args::parse();

    let ports = Ports {
        avr: serve(&args, args.net_ro_port)?,
        sbs: serve(&args, args.net_sbs_port)?,
        beast: serve(&args, args.net_bo_port)?,
    };
    if !args.quiet {
        for (name, port) in
            [("AVR", args.net_ro_port), ("SBS", args.net_sbs_port), ("Beast", args.net_bo_port)]
        {
            if port > 0 {
                println!("{name:<6} on {}:{port}", args.net_bind_address);
            }
        }
    }

    let incoming: Vec<std::sync::mpsc::Receiver<Vec<u8>>> = args
        .net_bi_port
        .iter()
        .filter(|p| **p > 0)
        .map(|port| {
            let rx = net::accept_frames(&args.net_bind_address, *port)
                .with_context(|| format!("cannot serve port {port}"))?;
            if !args.quiet {
                println!("input  on {}:{port}", args.net_bind_address);
            }
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

/// Where a decoded frame goes
struct Ports {
    avr: Option<net::Fanout>,
    sbs: Option<net::Fanout>,
    beast: Option<net::Fanout>,
}

fn serve(args: &Args, port: u16) -> Result<Option<net::Fanout>> {
    if port == 0 {
        return Ok(None);
    }
    Ok(Some(
        net::Fanout::serve(&args.net_bind_address, port)
            .with_context(|| format!("cannot serve port {port}"))?,
    ))
}

/// The iqstream server this receiver fans its own samples out on, if asked.
///
/// A subscriber here is reading the same tuner, which is how one aerial feeds
/// this and a second receiver at once. Not tunable: the dial belongs to
/// whoever this is decoding for.
fn listen(
    args: &Args,
    center_hz: u64,
    rate: f64,
) -> Result<Option<std::sync::Arc<iqstream::Server>>> {
    let Some(spec) = &args.iqstream_listen else { return Ok(None) };
    let addr: std::net::SocketAddr = match spec.parse() {
        Ok(a) => a,
        Err(_) => match spec.parse::<u16>() {
            Ok(port) => ([0, 0, 0, 0], port).into(),
            Err(_) => bail!("--iqstream-listen wants addr:port or a port, not {spec:?}"),
        },
    };
    let cfg = iqstream::ServerConfig {
        name: "wave1090".into(),
        center_hz,
        sample_rate: rate as u32,
        gain_db: Some(args.gain),
        tunable: false,
        tune_range_hz: None,
    };
    let server = iqstream::Server::start(addr, cfg).context("cannot serve iqstream")?;
    if !args.quiet {
        println!("iqstream on {}", server.addr());
    }
    Ok(Some(server))
}

/// The state a run carries between blocks.
struct Reader {
    det: ModeSDetector,
    book: AddressBook,
    sbs: sbs::Sbs,
    frames: Vec<ModeSFrame>,
    rate: f64,
    raw: bool,
    read: u64,
    kept: u64,
    /// Where the samples are fanned out, and the buffer they are packed into
    server: Option<std::sync::Arc<iqstream::Server>>,
    uc8: Vec<u8>,
}

impl Reader {
    fn new(rate: f64, raw: bool) -> Self {
        Self {
            det: ModeSDetector::new(rate, ModeSConfig::default()),
            book: AddressBook::new(),
            sbs: sbs::Sbs::default(),
            frames: Vec::new(),
            rate,
            raw,
            read: 0,
            kept: 0,
            server: None,
            uc8: Vec::new(),
        }
    }

    /// One block of samples in, whatever it held out on every port.
    fn block(&mut self, iq: &[common::C32], ports: &Ports) {
        self.read += iq.len() as u64;
        // Before the decoding, so a subscriber's copy is not delayed by it,
        // and only where somebody is connected: packing costs a pass over
        // every sample.
        if let Some(server) = &self.server
            && server.subscribers() > 0
        {
            SampleFormat::Cu8.encode(iq, &mut self.uc8);
            server.push(&self.uc8);
            self.uc8.clear();
        }
        self.frames.clear();
        let book = std::cell::RefCell::new(std::mem::take(&mut self.book));
        self.det.process_valid(iq, &mut self.frames, &|f: &ModeSFrame| {
            book.borrow_mut().accept(&f.bytes, f.weak_bits == 0)
        });
        self.book = book.into_inner();

        let now = chrono::Utc::now();
        // Taken out of the way, because publishing borrows the rest of the
        // reader: the buffer goes back afterwards so a block costs no
        // allocation.
        let frames = std::mem::take(&mut self.frames);
        for f in &frames {
            // Correcting a flipped bit is arithmetic on the frame rather than
            // signal processing, so it happens here, as it does in the
            // receiver's own Mode S node.
            let bytes = match f.bytes[0] >> 3 {
                17 | 18 => adsb::fix_single_bit(&f.bytes).unwrap_or_else(|| f.bytes.clone()),
                _ => f.bytes.clone(),
            };
            let clock =
                (f.at_sample as f64 * BEAST_CLOCK_HZ / self.rate) as u64 & 0x0000_ffff_ffff_ffff;
            self.publish(&bytes, clock, f.rssi_dbfs, ports, now);
        }
        self.frames = frames;
        self.sbs.expire();
    }

    /// One frame out on every port, whether it was heard here or handed back
    /// by somebody else.
    fn publish(
        &mut self,
        bytes: &[u8],
        clock: u64,
        rssi_dbfs: f32,
        ports: &Ports,
        now: chrono::DateTime<chrono::Utc>,
    ) {
        let Ok(frame) = adsb::parse(bytes) else { return };
        self.kept += 1;
        if self.raw || ports.avr.is_some() {
            let line = net::avr(bytes);
            if self.raw {
                print!("{line}");
            }
            if let Some(p) = &ports.avr {
                p.send(line.as_bytes());
            }
        }
        if let Some(p) = &ports.beast {
            p.send(&net::beast(bytes, clock, rssi_dbfs));
        }
        if let Some(p) = &ports.sbs {
            for line in self.sbs.lines(&frame, now) {
                p.send(format!("{line}\r\n").as_bytes());
            }
        }
    }
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
    if !args.quiet {
        println!("{} at {:.4} MHz, {:.3} MS/s", dev.info().label, center as f64 / 1e6, rate / 1e6);
    }
    if (center as i64 - args.freq as i64).abs() > 1_000_000 {
        bail!("the radio is on {:.4} MHz, not 1090", center as f64 / 1e6);
    }
    if rate < 2_000_000.0 {
        bail!("{:.3} MS/s is too slow to read a microsecond wide chip", rate / 1e6);
    }

    let mut stream = dev.start_rx().context("the radio would not start")?;
    let mut reader = Reader::new(rate, args.raw);
    reader.server = listen(args, center, rate)?;
    reader.sbs.here = station(args);
    let began = std::time::Instant::now();
    let mut said = began;
    loop {
        let buf = match stream.read() {
            Ok(b) => b,
            Err(e) => bail!("the radio stopped: {e}"),
        };
        reader.block(&buf.samples, &ports);
        // Whatever came back from an mlat client since the last block, put
        // out as though it had been heard here, which is what
        // `--forward-mlat` means to everything downstream.
        for rx in &incoming {
            while let Ok(bytes) = rx.try_recv() {
                reader.publish(&bytes, 0, f32::NEG_INFINITY, &ports, chrono::Utc::now());
            }
        }
        if !args.quiet && said.elapsed().as_secs() >= 10 {
            said = std::time::Instant::now();
            eprintln!(
                "{} frames in {:.0} s, {:.1} a second, {} dropped",
                reader.kept,
                began.elapsed().as_secs_f64(),
                reader.kept as f64 / began.elapsed().as_secs_f64(),
                stream.dropped()
            );
        }
    }
}

fn from_file(args: &Args, path: &std::path::Path, ports: Ports) -> Result<()> {
    let src = sources::FileSource::open(path).with_context(|| format!("{}", path.display()))?;
    let buf = src.read_all().context("reading the capture")?;
    let rate = buf.rate.as_f64();
    if !args.quiet {
        println!(
            "{} at {:.4} MHz, {:.3} MS/s, {:.1} s",
            path.display(),
            buf.center.as_f64() / 1e6,
            rate / 1e6,
            buf.samples.len() as f64 / rate
        );
    }
    let mut reader = Reader::new(rate, args.raw);
    reader.server = listen(args, buf.center.0, rate)?;
    reader.sbs.here = station(args);
    for block in buf.samples.chunks(65_536) {
        reader.block(block, &ports);
    }
    if !args.quiet {
        println!("{} frames", reader.kept);
    }
    Ok(())
}
