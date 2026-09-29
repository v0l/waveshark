use common::time::Duration;
use iqstream::proto::Codec;
use iqstream::{ClientConfig, IqStream, Server, ServerConfig, StreamConfig};
use std::sync::Arc;
use std::time::Instant;
use str0m::change::SdpAnswer;
use str0m::net::{Protocol, Receive};
use str0m::{Candidate, Event, Input, Output, Rtc};
use tokio::io::{AsyncReadExt, AsyncWriteExt};
use tokio::net::UdpSocket;
use tokio::sync::oneshot;

fn server() -> Arc<Server> {
    Server::start(
        "127.0.0.1:0".parse().unwrap(),
        ServerConfig {
            webrtc: true,
            ..ServerConfig::single(
                "test",
                StreamConfig {
                    name: "span".into(),
                    center_hz: 1_090_000_000,
                    sample_rate: 2_400_000,
                    ..Default::default()
                },
            )
        },
    )
    .unwrap()
}

async fn dial(srv: Arc<Server>) -> (iqstream::ws::Read, iqstream::ws::Write) {
    let socket = UdpSocket::bind("127.0.0.1:0").await.unwrap();
    let local = socket.local_addr().unwrap();
    let mut rtc = Rtc::builder().build(Instant::now());
    rtc.add_local_candidate(Candidate::host(local, "udp").unwrap());
    let mut change = rtc.sdp_api();
    let id = change.add_channel(iqstream::rtc::LABEL.into());
    let (offer, pending) = change.apply().unwrap();
    let offer = offer.to_sdp_string();
    let answer = tokio::task::spawn_blocking(move || srv.answer(&offer)).await.unwrap().unwrap();
    rtc.sdp_api().accept_answer(pending, SdpAnswer::from_sdp_string(&answer).unwrap()).unwrap();

    let (app, driver) = tokio::io::duplex(1 << 20);
    let (mut from_app, mut to_app) = tokio::io::split(driver);
    let (opened, open) = oneshot::channel();
    tokio::spawn(async move {
        let mut opened = Some(opened);
        let mut buf = vec![0u8; 2048];
        let mut out = vec![0u8; 16 * 1024];
        let mut held: Vec<u8> = Vec::new();
        loop {
            let wait = loop {
                match rtc.poll_output().unwrap() {
                    Output::Timeout(t) => break t,
                    Output::Transmit(t) => {
                        let _ = socket.send_to(&t.contents, t.destination).await;
                    }
                    Output::Event(Event::ChannelOpen(..)) => {
                        if let Some(o) = opened.take() {
                            let _ = o.send(());
                        }
                    }
                    Output::Event(Event::ChannelData(d)) => {
                        to_app.write_all(&d.data).await.unwrap()
                    }
                    Output::Event(_) => {}
                }
            };
            if !held.is_empty()
                && let Some(mut ch) = rtc.channel(id)
                && ch.write(true, &held).unwrap()
            {
                held.clear();
            }
            let until = tokio::time::Instant::from_std(wait.max(Instant::now()));
            tokio::select! {
                got = socket.recv_from(&mut buf) => {
                    let (n, from) = got.unwrap();
                    let input = Input::Receive(Instant::now(), Receive {
                        proto: Protocol::Udp,
                        source: from,
                        destination: local,
                        contents: buf[..n].try_into().unwrap(),
                    });
                    rtc.handle_input(input).unwrap();
                }
                read = from_app.read(&mut out), if held.is_empty() => match read {
                    Ok(0) | Err(_) => return,
                    Ok(n) => held = out[..n].to_vec(),
                },
                _ = tokio::time::sleep_until(until) => {}
            }
            rtc.handle_input(Input::Timeout(Instant::now())).unwrap();
        }
    });
    tokio::time::timeout(Duration::from_secs(10), open).await.unwrap().unwrap();
    let (read, write) = tokio::io::split(app);
    (Box::new(read), Box::new(write))
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn a_data_channel_carries_the_samples_byte_for_byte() {
    let srv = server();
    assert!(srv.webrtc());
    let tuner = srv.default_stream().unwrap();
    let (read, write) = dial(srv.clone()).await;
    let config =
        ClientConfig { name: "test".into(), bits: 8, codec: Codec::Zstd, ..Default::default() };
    let mut stream = IqStream::connect_over(read, write, config).await.unwrap();
    assert_eq!(stream.transport(), iqstream::Transport::Tcp);
    assert_eq!(stream.info().center_hz, 1_090_000_000);
    let block: Vec<u8> = (0..4096 * 2).map(|i| (i % 251) as u8).collect();
    let mut got = Vec::new();
    let deadline = tokio::time::Instant::now() + Duration::from_secs(10);
    while got.len() < 3 && tokio::time::Instant::now() < deadline {
        tuner.push(&block);
        if let Ok(Ok(Some(b))) =
            tokio::time::timeout(Duration::from_millis(100), stream.next_block()).await
        {
            got.push(b);
        }
    }
    assert_eq!(got.len(), 3);
    for b in &got {
        assert_eq!(b.samples, block, "a byte changed in flight");
    }
    assert_eq!(got[2].sample_index - got[0].sample_index, 8192);
}

#[test]
fn an_offer_to_a_server_without_webrtc_is_refused() {
    let srv = Server::start("127.0.0.1:0".parse().unwrap(), ServerConfig::default()).unwrap();
    assert!(!srv.webrtc());
    assert!(srv.answer("v=0").is_err());
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn a_data_channel_keeps_up_with_a_dongle_at_full_rate() {
    if cfg!(debug_assertions) {
        eprintln!("skipped: times the channel, run with --release");
        return;
    }
    let srv = server();
    let tuner = srv.default_stream().unwrap();
    let (read, write) = dial(srv.clone()).await;
    let config =
        ClientConfig { name: "test".into(), bits: 8, codec: Codec::Zstd, ..Default::default() };
    let mut stream = IqStream::connect_over(read, write, config).await.unwrap();
    let mut noise = 0x2545_f491_4f6c_dd1du64;
    let block: Vec<u8> = (0..20_480 * 2)
        .map(|_| {
            noise ^= noise << 13;
            noise ^= noise >> 7;
            noise ^= noise << 17;
            noise as u8
        })
        .collect();
    let pushing = tuner.clone();
    let pushed = tokio::spawn(async move {
        let mut tick = tokio::time::interval(Duration::from_millis(20));
        for _ in 0..150 {
            tick.tick().await;
            pushing.push(&block);
        }
    });
    let mut samples = 0u64;
    let deadline = tokio::time::Instant::now() + Duration::from_secs(5);
    while tokio::time::Instant::now() < deadline {
        match tokio::time::timeout(Duration::from_millis(200), stream.next_block()).await {
            Ok(Ok(Some(b))) => samples += b.samples.len() as u64 / 2 - b.padded_before,
            _ if pushed.is_finished() => break,
            _ => {}
        }
    }
    let sent = 150 * 20_480u64;
    assert!(
        samples * 100 >= sent * 95,
        "floor: {samples} of {sent} samples of incompressible 2.4 MS/s came through"
    );
}
