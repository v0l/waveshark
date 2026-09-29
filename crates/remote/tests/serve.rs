use common::device::Device as _;
use common::{C32, Hz, Sps};
use dsp::spectrum::{Detector, Spectrum};
use iqstream::{Server, ServerConfig, StreamConfig};
use remote::door::Doors;
use std::io::{Read, Write};
use std::net::TcpStream;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

const CENTER: u64 = 433_000_000;
const RATE: u32 = 2_400_000;
const TONE: f64 = 300_000.0;
const BLOCK: usize = 24_000;

struct Air {
    server: Arc<Server>,
    stop: Arc<AtomicBool>,
    join: Option<std::thread::JoinHandle<()>>,
}

impl Drop for Air {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::SeqCst);
        if let Some(j) = self.join.take() {
            let _ = j.join();
        }
    }
}

fn air() -> Air {
    let server = Server::start(
        "127.0.0.1:0".parse().unwrap(),
        ServerConfig {
            name: "test".into(),
            streams: vec![StreamConfig {
                name: "span".into(),
                center_hz: CENTER,
                sample_rate: RATE,
                ..Default::default()
            }],
            door: Some(Doors::shared()),
        },
    )
    .unwrap();
    let stream = server.default_stream().unwrap();
    stream.set_hardware("rtlsdr");
    let tone: Vec<C32> = (0..BLOCK)
        .map(|k| {
            let p = std::f64::consts::TAU * TONE * k as f64 / RATE as f64;
            C32::new(0.5 * p.cos() as f32, 0.5 * p.sin() as f32)
        })
        .collect();
    let mut block = Vec::new();
    common::SampleFormat::Cu8.encode(&tone, &mut block);
    let stop = Arc::new(AtomicBool::new(false));
    let halt = stop.clone();
    let join = std::thread::spawn(move || {
        while !halt.load(Ordering::SeqCst) {
            stream.push(&block);
            std::thread::sleep(Duration::from_millis(10));
        }
    });
    Air { server, stop, join: Some(join) }
}

fn loudest_hz(iq: &[C32], rate: f64) -> f64 {
    let mut s = Spectrum::new(1024);
    s.process(iq);
    let f = s.take(&[Detector::Average]);
    let bins = Detector::Average.of(&f);
    let (at, _) =
        bins.iter().enumerate().fold((0, f32::MIN), |b, (i, &v)| if v > b.1 { (i, v) } else { b });
    (at as f64 - 512.0) * rate / 1024.0
}

fn read_for(rx: &mut Box<dyn common::device::RxStream>, secs: f64) -> (usize, Vec<C32>, Sps) {
    let until = Instant::now() + Duration::from_secs_f64(secs);
    let (mut n, mut last, mut rate) = (0, Vec::new(), Sps(0));
    while Instant::now() < until {
        let b = rx.read().unwrap();
        n += b.samples.len();
        rate = b.rate;
        last.extend_from_slice(&b.samples);
        let keep = last.len().saturating_sub(8192);
        last.drain(..keep);
    }
    (n, last, rate)
}

#[test]
fn an_rtl_tcp_client_cuts_240_khz_out_of_the_span_where_it_tuned() {
    let air = air();
    let addr = air.server.addr().to_string();
    let mut dev = remote::rtl_tcp::Device::open(&addr).unwrap();
    dev.set_rate(Sps(240_000)).unwrap();
    dev.set_center(Hz(CENTER + 280_000)).unwrap();
    let mut rx = dev.start_rx().unwrap();
    read_for(&mut rx, 0.3);
    let (n, last, _) = read_for(&mut rx, 1.0);
    assert!((200_000..=280_000).contains(&n), "{n} samples in a second at 240 kS/s");
    assert_eq!(
        loudest_hz(&last, 240_000.0),
        19_921.875,
        "the bin nearest 20 kHz: the 300 kHz tone above 280"
    );
    assert_eq!(air.server.default_stream().unwrap().subscribers(), 1);
}

#[test]
fn an_rtl_tcp_client_at_the_span_rate_gets_the_span_untouched() {
    let air = air();
    let addr = air.server.addr().to_string();
    let dev = remote::rtl_tcp::Device::open(&addr).unwrap();
    let mut dev = dev;
    dev.set_rate(Sps(RATE as u64)).unwrap();
    let mut rx = dev.start_rx().unwrap();
    let (n, last, _) = read_for(&mut rx, 1.0);
    assert!((2_000_000..=2_800_000).contains(&n), "{n} samples in a second at 2.4 MS/s");
    assert_eq!(loudest_hz(&last, RATE as f64), 300_000.0);
}

#[test]
fn a_spyserver_client_is_told_the_span_and_reads_a_decimated_cut_of_it() {
    let air = air();
    let addr = air.server.addr().to_string();
    let probe = remote::spyserver::probe(&addr).unwrap();
    assert_eq!(probe.tuner, "RTL-SDR");
    assert_eq!(probe.name, "RTL-SDR (shared)");
    assert!(!probe.tunable);
    assert_eq!(probe.center, Some(Hz(CENTER)));
    assert_eq!(probe.rates.first(), Some(&Sps(9_375)));
    assert_eq!(probe.rates.last(), Some(&Sps(RATE as u64)));
    assert_eq!(probe.rates.len(), 9);

    let mut dev = remote::spyserver::Device::open(&addr).unwrap();
    dev.set_rate(Sps(300_000)).unwrap();
    dev.set_center(Hz(CENTER + 280_000)).unwrap();
    let mut rx = dev.start_rx().unwrap();
    read_for(&mut rx, 0.3);
    let (n, last, rate) = read_for(&mut rx, 1.0);
    assert_eq!(rate, Sps(300_000));
    assert!((250_000..=350_000).contains(&n), "{n} samples in a second at 300 kS/s");
    assert_eq!(
        loudest_hz(&last, 300_000.0),
        19_921.875,
        "the bin nearest 20 kHz: the tone above 280"
    );
}

fn command(sock: &mut TcpStream, cmd: u32, body: &[u8]) {
    let mut out = Vec::new();
    out.extend_from_slice(&cmd.to_le_bytes());
    out.extend_from_slice(&(body.len() as u32).to_le_bytes());
    out.extend_from_slice(body);
    sock.write_all(&out).unwrap();
}

fn setting(sock: &mut TcpStream, which: u32, value: u32) {
    let mut b = which.to_le_bytes().to_vec();
    b.extend_from_slice(&value.to_le_bytes());
    command(sock, 2, &b);
}

fn message(sock: &mut TcpStream) -> (u32, Vec<u8>) {
    let mut head = [0u8; 20];
    sock.read_exact(&mut head).unwrap();
    let word = |i: usize| u32::from_le_bytes(head[i..i + 4].try_into().unwrap());
    let mut body = vec![0u8; word(16) as usize];
    sock.read_exact(&mut body).unwrap();
    (word(4) & 0xffff, body)
}

#[test]
fn a_spyserver_client_asking_for_the_display_gets_15_frames_a_second_with_the_tone_on_them() {
    let air = air();
    let mut sock = TcpStream::connect(air.server.addr()).unwrap();
    sock.set_read_timeout(Some(Duration::from_secs(5))).unwrap();
    let mut hello = ((2u32 << 24) | 1700).to_le_bytes().to_vec();
    hello.extend_from_slice(b"test");
    command(&mut sock, 0, &hello);
    assert_eq!(message(&mut sock).0, 0);
    assert_eq!(message(&mut sock).0, 1);
    setting(&mut sock, 205, 600);
    setting(&mut sock, 204, 100);
    setting(&mut sock, 0, 4);
    setting(&mut sock, 1, 1);
    let until = Instant::now() + Duration::from_secs(2);
    let mut frames = Vec::new();
    while Instant::now() < until {
        let (kind, body) = message(&mut sock);
        if kind == 301 {
            frames.push(body);
        }
    }
    assert!((26..=34).contains(&frames.len()), "{} frames in two seconds", frames.len());
    let last = frames.last().unwrap();
    assert_eq!(last.len(), 600);
    let loudest = last.iter().enumerate().max_by_key(|(_, v)| **v).unwrap().0;
    assert_eq!(loudest, 375, "300 kHz up a 2.4 MHz display 600 pixels wide");
    assert!(last[loudest] > 200, "a tone 6 dB under full scale near the top: {}", last[loudest]);
    assert!(last[100] < 80, "the floor near the bottom: {}", last[100]);
}

#[test]
fn an_iqstream_client_still_finds_the_server_behind_the_door() {
    let air = air();
    let addr = air.server.addr().to_string();
    let found = remote::iqstream::probe_all(&addr).unwrap();
    assert_eq!(found.len(), 1);
    assert_eq!(found[0].center, Some(Hz(CENTER)));
    assert_eq!(remote::identify(&addr).unwrap().proto, remote::Proto::IqStream);
}

#[test]
fn a_ninth_listener_is_turned_away_and_the_directory_is_told_eight() {
    let air = air();
    let addr = air.server.addr().to_string();
    let held: Vec<_> = (0..remote::door::MOST_SESSIONS)
        .map(|_| remote::rtl_tcp::Device::open(&addr).unwrap())
        .collect();
    assert!(remote::rtl_tcp::Device::open(&addr).is_err());
    assert_eq!(remote::door::MOST_SESSIONS as u32, sdr_directory::airspy::MAX_CLIENTS);
    drop(held);
    std::thread::sleep(Duration::from_millis(600));
    assert!(remote::rtl_tcp::Device::open(&addr).is_ok());
}

#[test]
fn an_iqstream_client_reads_the_span_through_a_websocket_on_the_same_port() {
    let air = air();
    let url = format!("ws://{}/", air.server.addr());
    let found = remote::iqstream::probe_all(&url).unwrap();
    assert_eq!(found.len(), 1);
    assert_eq!(found[0].addr, format!("{url}#0"));
    assert_eq!(found[0].rate, Some(Sps(RATE as u64)));
    let mut dev = remote::iqstream::Device::open(&found[0].addr).unwrap();
    let mut rx = dev.start_rx().unwrap();
    read_for(&mut rx, 0.3);
    let (n, last, _) = read_for(&mut rx, 1.0);
    assert!((2_000_000..=2_800_000).contains(&n), "{n} samples in a second at 2.4 MS/s");
    assert_eq!(loudest_hz(&last, RATE as f64), 300_000.0);
}
