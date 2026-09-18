//! A dongle on another machine, spoken to with `rtl_tcp`.
//!
//! One socket carries both directions: the server greets a new connection
//! with twelve bytes saying which tuner it has, then sends interleaved 8-bit
//! IQ for as long as the connection lasts, while the client sends five-byte
//! commands the other way. There is no reply to a command and no way to read
//! anything back, so what the tuner is on is what we last told it.
//!
//! The dongle is ours while we are connected: frequency, rate and gain are
//! settings rather than readings, and the server turns away a second client
//! for as long as this one holds the socket.

use crate::{CONNECT_TIMEOUT, Probe, Proto, QUEUE_DEPTH};
use common::device::{
    Device as DeviceTrait, DeviceInfo, DriverKind, GainMode, GainStage, RxStream,
};
use common::rtl::{self, Tuner};
use common::{Error, Hz, IqBuf, Result, SampleFormat, Sps};
use crossbeam_channel::{Receiver, Sender, TrySendError, bounded};
use std::io::{Read, Write};
use std::net::{TcpStream, ToSocketAddrs};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};

/// The greeting: "RTL0", the tuner type, and how many gain steps it has.
const MAGIC: [u8; 4] = *b"RTL0";
const GREETING: usize = 12;

/// Commands, as `rtl_tcp.c` numbers them. One byte of command and four of
/// value, big endian, with no reply.
#[derive(Clone, Copy)]
#[repr(u8)]
enum Cmd {
    Center = 0x01,
    Rate = 0x02,
    /// 1 puts the tuner's gain under our control, 0 leaves it to the tuner.
    GainMode = 0x03,
    /// Tenths of a dB. The far end snaps it to a step the tuner has.
    Gain = 0x04,
    Ppm = 0x05,
    /// The RTL2832U's own digital AGC, which is not the tuner's.
    RtlAgc = 0x08,
}

/// How long a read may block before the stream calls the server gone. Long
/// enough that a quiet moment is not a disconnection: at the slowest rate
/// offered, a block is a fifth of a second.
const READ_TIMEOUT: std::time::Duration = std::time::Duration::from_secs(5);

fn connect(addr: &str) -> Result<(TcpStream, Tuner, u32)> {
    let resolved = addr
        .to_socket_addrs()
        .map_err(|e| Error::other(format!("{addr}: {e}")))?
        .next()
        .ok_or_else(|| Error::other(format!("{addr} resolves to nothing")))?;
    let mut sock = TcpStream::connect_timeout(&resolved, CONNECT_TIMEOUT)
        .map_err(|e| Error::other(format!("{addr}: {e}")))?;
    sock.set_read_timeout(Some(CONNECT_TIMEOUT))
        .map_err(|e| Error::other(format!("{addr}: {e}")))?;
    // Commands are five bytes and matter immediately, so Nagle would hold a
    // retune back waiting for more to send.
    let _ = sock.set_nodelay(true);

    let mut hello = [0u8; GREETING];
    sock.read_exact(&mut hello)
        .map_err(|e| Error::other(format!("{addr} sent no rtl_tcp greeting: {e}")))?;
    if hello[..4] != MAGIC {
        return Err(Error::other(format!("{addr} is not rtl_tcp")));
    }
    let tuner = Tuner::from_code(u32::from_be_bytes([hello[4], hello[5], hello[6], hello[7]]));
    let gains = u32::from_be_bytes([hello[8], hello[9], hello[10], hello[11]]);
    Ok((sock, tuner, gains))
}

fn send(sock: &mut TcpStream, cmd: Cmd, value: u32) -> Result<()> {
    let mut b = [0u8; 5];
    b[0] = cmd as u8;
    b[1..].copy_from_slice(&value.to_be_bytes());
    sock.write_all(&b).map_err(|e| Error::other(format!("rtl_tcp: {e}")))
}

/// Ask what is at an address, then disconnect.
///
/// The greeting arrives unprompted, so one read settles whether this is an
/// rtl_tcp server. It says nothing about frequency or rate, because those are
/// ours to choose once connected.
pub fn probe(addr: &str) -> Result<Probe> {
    let addr = Proto::RtlTcp.parse_addr(addr).ok_or(Error::NoDevice)?;
    let (sock, tuner, gains) = connect(&addr)?;
    drop(sock);
    tracing::debug!("rtl_tcp {addr}: {} with {gains} gain steps", tuner.name());
    Ok(Probe {
        proto: Proto::RtlTcp,
        addr,
        center: None,
        rate: None,
        gain_db: None,
        // A dongle on rtl_tcp is the whole server, so there is nothing to
        // tell it apart from.
        name: String::new(),
        // rtl_tcp says nothing about the dongle's gain, and the one control
        // it takes is spoken as its own command rather than as a setting.
        settings: Vec::new(),
        tunable: true,
        // The dongle's own range, which the greeting names by naming its
        // tuner chip.
        tune_range: tuner.ranges().first().map(|r| r.range.clone()),
        tuner: tuner.name().to_string(),
    })
}

pub struct Device {
    addr: String,
    info: DeviceInfo,
    /// The converter on the cable and the reference correction, which the
    /// `Device` trait does the arithmetic with.
    tuning: common::Tuning,
    /// The control connection, which is also the one the samples come back on
    /// once `start_rx` takes a clone of it.
    sock: TcpStream,
    center: Hz,
    rate: Sps,
    streaming: Arc<AtomicBool>,
}

impl Device {
    /// Connect, learn which tuner is at the far end, and set it going at a
    /// default the receiver will immediately overwrite.
    pub fn open(addr: &str) -> Result<Self> {
        let addr = Proto::RtlTcp.parse_addr(addr).ok_or(Error::NoDevice)?;
        let (sock, tuner, steps) = connect(&addr)?;
        let info = DeviceInfo {
            kind: DriverKind::Network,
            id: format!("rtl_tcp:{addr}"),
            label: format!("rtl_tcp {addr} ({})", tuner.name()),
            tuner: tuner.name().to_string(),
            ranges: tuner.ranges(),
            rates: rtl::RATES.to_vec(),
            rate_range: rtl::RATE_RANGE,
            gain_stages: vec![GainStage {
                name: "tuner".to_string(),
                label: "Tuner RF".to_string(),
                range: 0.0..=tuner.max_gain_db(),
                // The greeting counts the tuner's steps without saying what
                // they are, so the control is continuous here and the far end
                // snaps what it is sent to the nearest step it has.
                values: Vec::new(),
                step: 0.0,
                auto: true,
            }],
            native_format: SampleFormat::Cu8,
            usable_bandwidth_ratio: rtl::USABLE_BANDWIDTH_RATIO,
            tunable: true,
            tx: None,
        };
        tracing::debug!("rtl_tcp {addr}: {} with {steps} gain steps", tuner.name());

        let mut me = Self {
            addr,
            info,
            tuning: Default::default(),
            sock,
            center: Hz::mhz(100),
            rate: Sps(2_048_000),
            streaming: Arc::new(AtomicBool::new(false)),
        };
        // The same defaults the USB driver opens with: manual tuner gain,
        // because an AGC hunting moves the noise floor under wideband
        // detection, and the RTL's digital AGC off.
        me.set_rate(Sps(2_048_000))?;
        me.set_center(Hz::mhz(100))?;
        me.set_gain("tuner", GainMode::Auto)?;
        send(&mut me.sock, Cmd::RtlAgc, 0)?;
        Ok(me)
    }

    pub fn address(&self) -> &str {
        &self.addr
    }
}

impl DeviceTrait for Device {
    fn tuning(&self) -> &common::Tuning {
        &self.tuning
    }

    fn tuning_mut(&mut self) -> &mut common::Tuning {
        &mut self.tuning
    }

    fn info(&self) -> &DeviceInfo {
        &self.info
    }

    fn set_center(&mut self, f: Hz) -> Result<()> {
        send(&mut self.sock, Cmd::Center, f.0 as u32)?;
        self.center = f;
        Ok(())
    }

    fn center(&self) -> Hz {
        self.center
    }

    fn set_rate(&mut self, r: Sps) -> Result<()> {
        if !rtl::RATE_RANGE.contains(&r) {
            return Err(Error::RateUnsupported { req: r });
        }
        send(&mut self.sock, Cmd::Rate, r.0 as u32)?;
        self.rate = r;
        Ok(())
    }

    fn rate(&self) -> Sps {
        self.rate
    }

    fn set_gain(&mut self, stage: &str, mode: GainMode) -> Result<()> {
        if stage != "tuner" {
            return Err(Error::other(format!("rtl_tcp has no {stage} gain")));
        }
        match mode {
            GainMode::Auto => send(&mut self.sock, Cmd::GainMode, 0),
            GainMode::Manual(db) => {
                send(&mut self.sock, Cmd::GainMode, 1)?;
                // Tenths of a dB, and negative gains exist on the E4000, so
                // the cast has to go through i32 rather than clamp at zero.
                send(&mut self.sock, Cmd::Gain, ((db * 10.0).round() as i32) as u32)
            }
        }
    }

    fn set_ppm(&mut self, ppm: f64) -> Result<()> {
        send(&mut self.sock, Cmd::Ppm, (ppm.round() as i32) as u32)
    }

    fn start_rx(&mut self) -> Result<Box<dyn RxStream>> {
        if self.streaming.swap(true, Ordering::SeqCst) {
            return Err(Error::Busy);
        }
        let sock = self
            .sock
            .try_clone()
            .map_err(|e| Error::other(format!("rtl_tcp {}: {e}", self.addr)))?;
        sock.set_read_timeout(Some(READ_TIMEOUT))
            .map_err(|e| Error::other(format!("rtl_tcp {}: {e}", self.addr)))?;

        let (tx, rx) = bounded::<IqBuf>(QUEUE_DEPTH);
        let dropped = Arc::new(AtomicU64::new(0));
        let stop = Arc::new(AtomicBool::new(false));

        let (center, rate) = (self.center, self.rate);
        let streaming = self.streaming.clone();
        let counted = dropped.clone();
        let halt = stop.clone();
        let addr = self.addr.clone();
        let join = std::thread::Builder::new()
            .name("rtl-tcp-rx".into())
            .spawn(move || {
                if let Err(e) = pump(sock, center, rate, tx, counted, halt) {
                    tracing::warn!("rtl_tcp {addr}: {e}");
                }
                streaming.store(false, Ordering::SeqCst);
            })
            .map_err(|e| Error::other(format!("spawn rx thread: {e}")))?;

        Ok(Box::new(NetStream {
            rx,
            dropped,
            stop,
            sock: self.sock.try_clone().ok(),
            join: Some(join),
        }))
    }
}

/// Read blocks off the socket and hand them over until told to stop.
fn pump(
    mut sock: TcpStream,
    center: Hz,
    rate: Sps,
    tx: Sender<IqBuf>,
    dropped: Arc<AtomicU64>,
    stop: Arc<AtomicBool>,
) -> Result<()> {
    // About 20 ms of samples: short enough that the spectrum moves, long
    // enough that 2.4 MS/s is not thousands of reads a second.
    let pairs = ((rate.as_f64() / 50.0) as usize).clamp(2048, 1 << 18);
    let mut raw = vec![0u8; pairs * 2];
    let mut samples = Vec::with_capacity(pairs);
    let mut seq = 0u64;
    while !stop.load(Ordering::Relaxed) {
        match sock.read_exact(&mut raw) {
            Ok(()) => {}
            Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => continue,
            Err(e) if e.kind() == std::io::ErrorKind::TimedOut => continue,
            // The far end closed, which is how rtl_tcp ends a session: it has
            // no goodbye.
            Err(e) if e.kind() == std::io::ErrorKind::UnexpectedEof => break,
            Err(e) => return Err(Error::other(format!("{e}"))),
        }
        samples.clear();
        SampleFormat::Cu8.convert(&raw, &mut samples);
        let n = samples.len() as u64;
        let buf = IqBuf::new(std::mem::take(&mut samples), center, rate, seq);
        samples = Vec::with_capacity(pairs);
        seq += n;
        match tx.try_send(buf) {
            Ok(()) => {}
            // A consumer that cannot keep up loses the oldest samples rather
            // than stalling the socket, which would make the server's own
            // buffer overrun and drop them anyway.
            Err(TrySendError::Full(buf)) => {
                dropped.fetch_add(buf.len() as u64, Ordering::Relaxed);
            }
            Err(TrySendError::Disconnected(_)) => break,
        }
    }
    Ok(())
}

struct NetStream {
    rx: Receiver<IqBuf>,
    dropped: Arc<AtomicU64>,
    stop: Arc<AtomicBool>,
    /// The control socket, shut down so a reader blocked mid-block wakes.
    sock: Option<TcpStream>,
    join: Option<std::thread::JoinHandle<()>>,
}

impl RxStream for NetStream {
    fn read(&mut self) -> Result<IqBuf> {
        self.rx.recv().map_err(|_| Error::Disconnected)
    }

    fn dropped(&self) -> u64 {
        self.dropped.load(Ordering::Relaxed)
    }

    fn stop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        // A read that has half a block waits for the rest, which on a quiet
        // 240 kS/s stream is most of a second; shutting the socket ends it now.
        if let Some(s) = &self.sock {
            let _ = s.shutdown(std::net::Shutdown::Read);
        }
    }
}

impl Drop for NetStream {
    fn drop(&mut self) {
        self.stop();
        while self.rx.try_recv().is_ok() {}
        if let Some(j) = self.join.take() {
            let _ = j.join();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::net::TcpListener;

    /// A server that greets, sends a ramp, and keeps every command it was
    /// sent.
    fn fake(tuner: u32, gains: u32) -> (String, std::sync::mpsc::Receiver<[u8; 5]>) {
        let l = TcpListener::bind("127.0.0.1:0").unwrap();
        let addr = l.local_addr().unwrap().to_string();
        let (tx, rx) = std::sync::mpsc::channel();
        std::thread::spawn(move || {
            let (mut sock, _) = l.accept().unwrap();
            let mut hello = [0u8; GREETING];
            hello[..4].copy_from_slice(&MAGIC);
            hello[4..8].copy_from_slice(&tuner.to_be_bytes());
            hello[8..12].copy_from_slice(&gains.to_be_bytes());
            sock.write_all(&hello).unwrap();
            let mut writer = sock.try_clone().unwrap();
            std::thread::spawn(move || {
                let block: Vec<u8> = (0..=255u8).cycle().take(65536).collect();
                while writer.write_all(&block).is_ok() {
                    std::thread::sleep(std::time::Duration::from_millis(1));
                }
            });
            let mut cmd = [0u8; 5];
            while sock.read_exact(&mut cmd).is_ok() {
                if tx.send(cmd).is_err() {
                    break;
                }
            }
        });
        (addr, rx)
    }

    #[test]
    fn a_probe_reads_the_tuner_out_of_the_greeting() {
        let (addr, _cmds) = fake(5, 29);
        let p = probe(&addr).unwrap();
        assert_eq!(p.proto, Proto::RtlTcp);
        assert_eq!(p.tuner, "R820T");
        assert!(p.tunable);
        // The greeting says nothing about where it is tuned, because that is
        // ours to set once connected.
        assert_eq!(p.center, None);
        assert_eq!(p.rate, None);
    }

    #[test]
    fn a_server_that_does_not_greet_is_not_rtl_tcp() {
        let l = TcpListener::bind("127.0.0.1:0").unwrap();
        let addr = l.local_addr().unwrap().to_string();
        std::thread::spawn(move || {
            let (mut sock, _) = l.accept().unwrap();
            let _ = sock.write_all(b"IQST\0\0\0\0\0\0\0\0");
            std::thread::sleep(std::time::Duration::from_millis(200));
        });
        let e = probe(&addr).unwrap_err().to_string();
        assert!(e.contains("not rtl_tcp"), "{e}");
    }

    #[test]
    fn opening_sets_the_far_end_going_and_a_retune_is_five_bytes() {
        let (addr, cmds) = fake(5, 29);
        let mut d = Device::open(&addr).unwrap();
        assert_eq!(d.info().tuner, "R820T");
        assert!(d.info().tunable);
        assert_eq!(d.info().rates.len(), 8);
        d.set_center(Hz::mhz(1090)).unwrap();
        d.set_gain("tuner", GainMode::Manual(49.6)).unwrap();
        assert_eq!(d.center(), Hz::mhz(1090));

        let mut seen = Vec::new();
        while seen.len() < 7 {
            seen.push(cmds.recv_timeout(std::time::Duration::from_secs(2)).unwrap());
        }
        let value = |b: &[u8; 5]| u32::from_be_bytes([b[1], b[2], b[3], b[4]]);
        // Rate, centre, automatic gain, RTL AGC off on open; then the retune
        // and the manual gain, which is mode then tenths of a dB.
        assert_eq!((seen[0][0], value(&seen[0])), (0x02, 2_048_000));
        assert_eq!((seen[1][0], value(&seen[1])), (0x01, 100_000_000));
        assert_eq!((seen[2][0], value(&seen[2])), (0x03, 0));
        assert_eq!((seen[3][0], value(&seen[3])), (0x08, 0));
        assert_eq!((seen[4][0], value(&seen[4])), (0x01, 1_090_000_000));
        assert_eq!((seen[5][0], value(&seen[5])), (0x03, 1));
        assert_eq!((seen[6][0], value(&seen[6])), (0x04, 496));
    }

    #[test]
    fn a_stream_delivers_what_the_socket_sends() {
        let (addr, _cmds) = fake(5, 29);
        let mut d = Device::open(&addr).unwrap();
        d.set_rate(Sps(240_000)).unwrap();
        let mut s = d.start_rx().unwrap();
        // 240 kS/s at a fiftieth of a second is 4800 pairs a block.
        let b = s.read().unwrap();
        assert_eq!(b.len(), 4800);
        assert_eq!(b.rate, Sps(240_000));
        let second = s.read().unwrap();
        assert_eq!(second.seq, 4800);
        // Cu8 is centred on 127.5, so a ramp through 0 and 255 spans the
        // converter and nothing falls outside it.
        assert!(b.samples.iter().all(|c| c.re.abs() <= 1.01 && c.im.abs() <= 1.01));
        assert_eq!(s.dropped(), 0);
        s.stop();
    }

    #[test]
    fn a_second_reader_is_refused_while_one_is_running() {
        let (addr, _cmds) = fake(5, 29);
        let mut d = Device::open(&addr).unwrap();
        let _s = d.start_rx().unwrap();
        assert!(matches!(d.start_rx(), Err(Error::Busy)));
    }
}
