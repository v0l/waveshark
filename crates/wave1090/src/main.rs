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

/// How finely the parity search slices, as a word rather than a number
#[derive(Clone, Copy, PartialEq, clap::ValueEnum)]
enum Search {
    Off,
    Coarse,
    Fine,
}

impl Search {
    fn config(self) -> ModeSConfig {
        let base = ModeSConfig::default();
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

    /// How hard the parity search looks, which is what this costs.
    ///
    /// `fine` reads more than dump1090 and wants a third of a fast core;
    /// `coarse` draws level with it for half that; `off` leaves the parity
    /// search out and reads what a preamble search alone finds
    #[arg(long, value_name = "HOW", default_value = "fine")]
    parity_search: Search,

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
        for (name, port) in [("AVR", &ports.avr), ("SBS", &ports.sbs), ("Beast", &ports.beast)] {
            if let Some(p) = port {
                println!("{name:<6} on {}", p.addr());
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
fn listen(args: &Args, center_hz: u64, rate: f64) -> Result<Option<Fanned>> {
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
    if !args.quiet {
        println!("iqstream on {}", server.addr());
    }
    let tuner = server.default_stream().context("the server kept no tuner")?;
    Ok(Some(Fanned { server, tuner }))
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
    sbs: sbs::Sbs,
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
    fn new(rate: f64, raw: bool, search: Search) -> Self {
        Self {
            det: ModeSDetector::new(rate, search.config()),
            book: AddressBook::new(),
            sbs: sbs::Sbs::default(),
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
        if let clock::Step::Broke(n) = self.clock.block(seq, iq.len()) {
            // What the demodulator is carrying ended before the break, and a
            // splice between two bursts frames as a preamble nobody sent.
            self.det.reset();
            self.dropped += n;
        }
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
        self.frames.clear();
        let book = std::cell::RefCell::new(std::mem::take(&mut self.book));
        self.det.process_valid(iq, &mut self.frames, &|f: &ModeSFrame| {
            book.borrow_mut().accept(&f.bytes)
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
            let at = self.clock.at(f.at_sample, f.at_frac);
            self.publish(&bytes, at, f.rssi_dbfs, ports, now);
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
    let mut reader = Reader::new(rate, args.raw, args.parity_search);
    reader.server = listen(args, center, rate)?;
    reader.sbs.here = station(args);
    let began = std::time::Instant::now();
    let mut said = began;
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
                reader.publish(
                    &bytes,
                    clock::UNTIMED,
                    f32::NEG_INFINITY,
                    &ports,
                    chrono::Utc::now(),
                );
            }
        }
        if !args.quiet && said.elapsed().as_secs() >= 10 {
            said = std::time::Instant::now();
            eprintln!(
                "{} frames in {:.0} s, {:.1} a second, {} dropped, {} gaps",
                reader.kept,
                began.elapsed().as_secs_f64(),
                reader.kept as f64 / began.elapsed().as_secs_f64(),
                stream.dropped(),
                reader.dropped
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
    let mut reader = Reader::new(rate, args.raw, args.parity_search);
    reader.server = listen(args, buf.center.0, rate)?;
    reader.sbs.here = station(args);
    for (n, block) in buf.samples.chunks(65_536).enumerate() {
        reader.block((n * 65_536) as u64, block, &ports);
    }
    if !args.quiet {
        println!("{} frames", reader.kept);
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Read;

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
        let fanout = net::Fanout::serve("127.0.0.1", 0).expect("a port");
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
        let mut rx = Reader::new(rate, false, Search::Fine);
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
            net::Fanout::serve("127.0.0.1", 0).unwrap(),
            net::Fanout::serve("127.0.0.1", 0).unwrap(),
        );
        let (mut avr_client, mut beast_client) = (read(&avr), read(&beast));
        std::thread::sleep(std::time::Duration::from_millis(100));

        let ports = Ports { avr: Some(avr), sbs: None, beast: Some(beast) };
        let mut rx = Reader::new(2.4e6, false, Search::Fine);
        rx.publish(&frame, clock::UNTIMED, -20.0, &ports, chrono::Utc::now());
        rx.publish(&frame, 5_000, -20.0, &ports, chrono::Utc::now());
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
        assert!(count(&ours, 17) >= 1_050, "DF17: {} to their 1013", count(&ours, 17));
        // Level on all-call replies, which needs the ones answering a ground
        // station: reading only the ones answering nobody gave 411.
        assert!(count(&ours, 11) >= 1_100, "DF11: {} to their 1181", count(&ours, 11));
        // And behind on Comm-B, which is #159. A floor, so closing the gap
        // does not fail the test, and a ceiling nowhere near theirs so that
        // closing it is visible as a failure worth updating.
        assert!((300..500).contains(&count(&ours, 20)), "DF20: {}", count(&ours, 20));

        // No aircraft of our own invention: every frame names one the
        // reference also saw.
        let known: std::collections::HashSet<u32> =
            theirs.iter().filter_map(|(_, f)| names(f)).collect();
        let strangers: Vec<String> = ours
            .iter()
            .filter(|(_, f)| !names(f).is_some_and(|a| known.contains(&a)))
            .map(|(_, f)| f.iter().map(|b| format!("{b:02x}")).collect())
            .collect();
        assert!(strangers.is_empty(), "aircraft nobody else saw: {strangers:?}");
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
