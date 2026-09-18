//! Both ends of the protocol against each other: a server on a free port, a
//! client subscribing to it, and the samples arriving as they were pushed.

use iqstream::proto::Codec;
use iqstream::{ClientConfig, IqStream, Server, ServerConfig};
use std::sync::Arc;
use std::time::Duration;

fn server(tunable: bool) -> Arc<Server> {
    Server::start(
        "127.0.0.1:0".parse().unwrap(),
        ServerConfig {
            name: "test".into(),
            center_hz: 1_090_000_000,
            sample_rate: 2_400_000,
            gain_db: Some(49.6),
            tunable,
            tune_range_hz: Some((24_000_000, 1_766_000_000)),
        },
    )
    .expect("a free port")
}

/// A ramp is the strongest thing to send: every byte differs from the last, so
/// a fragment arriving out of order or a block assembled from the wrong pieces
/// shows up as a value rather than as a length.
fn ramp(samples: usize) -> Vec<u8> {
    (0..samples * 2).map(|i| (i % 251) as u8).collect()
}

/// Pump `block` at the server until the client has taken `want` blocks, or the
/// deadline passes. The push has to repeat because a subscriber only exists
/// after its handshake, and the handshake finishes on the server's own thread.
async fn collect(
    stream: &mut IqStream,
    srv: &Arc<Server>,
    block: &[u8],
    want: usize,
) -> Vec<iqstream::Block> {
    let mut got = Vec::new();
    let deadline = tokio::time::Instant::now() + Duration::from_secs(10);
    while got.len() < want && tokio::time::Instant::now() < deadline {
        srv.push(block);
        match tokio::time::timeout(Duration::from_millis(100), stream.next_block()).await {
            Ok(Ok(Some(b))) => got.push(b),
            Ok(Ok(None)) => break,
            Ok(Err(e)) => panic!("{e}"),
            Err(_) => continue,
        }
    }
    got
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn eight_bit_samples_arrive_byte_for_byte() {
    let srv = server(false);
    let mut stream = IqStream::connect(
        srv.addr().to_string().as_str(),
        ClientConfig { name: "test".into(), bits: 8, codec: Codec::None, ..Default::default() },
    )
    .await
    .unwrap();

    assert_eq!(stream.info().center_hz, 1_090_000_000);
    assert_eq!(stream.info().sample_rate, 2_400_000);
    assert_eq!(stream.info().gain_db, Some(49.6));
    assert!(!stream.info().tunable);

    // 4096 complex samples is 8192 bytes, which is six fragments at the 1400
    // byte payload limit: enough that reassembly is exercised rather than
    // skipped by a block that happens to fit one datagram.
    let block = ramp(4096);
    let got = collect(&mut stream, &srv, &block, 3).await;
    assert_eq!(got.len(), 3, "three blocks through the fan-out");
    for b in &got {
        assert_eq!(b.samples.len(), 8192);
        // Eight bits is the dongle's own resolution and passes through
        // untouched, so this is equality and not a tolerance.
        assert_eq!(b.samples, block, "a byte changed in flight");
        assert_eq!(b.padded_before, 0, "nothing was lost");
        assert_eq!(b.center_hz, 1_090_000_000);
    }
    assert_eq!(got[0].sample_index, 0);
    assert_eq!(got[1].sample_index, 4096);
    assert_eq!(got[2].sample_index, 8192);
    assert_eq!(srv.subscribers(), 1);
}

/// Six bits is lossy by two, and the reconstruction sits at the middle of each
/// step rather than its floor, so every value is within two of what went in
/// and the top six bits are exact.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn six_bit_zstd_keeps_the_top_six_bits() {
    let srv = server(false);
    let mut stream = IqStream::connect(
        srv.addr().to_string().as_str(),
        ClientConfig { name: "test".into(), bits: 6, codec: Codec::Zstd, ..Default::default() },
    )
    .await
    .unwrap();
    assert_eq!(stream.info().bit_depth, 6);
    assert_eq!(stream.info().codec, Codec::Zstd);

    let block = ramp(2048);
    let got = collect(&mut stream, &srv, &block, 1).await;
    assert_eq!(got.len(), 1);
    assert_eq!(got[0].samples.len(), 4096);
    for (a, b) in block.iter().zip(&got[0].samples) {
        assert_eq!(a >> 2, b >> 2, "top six bits");
        assert!(a.abs_diff(*b) <= 2, "{a} came back as {b}");
    }
}

/// A subscriber asks, the owner of the radio answers, and the answer reaches
/// every subscriber rather than only the one that asked.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn a_tune_is_a_request_the_owner_answers() {
    let srv = server(true);
    let mut asking = IqStream::connect(
        srv.addr().to_string().as_str(),
        ClientConfig { name: "asking".into(), bits: 8, codec: Codec::None, ..Default::default() },
    )
    .await
    .unwrap();
    let mut watching = IqStream::connect(
        srv.addr().to_string().as_str(),
        ClientConfig { name: "watching".into(), bits: 8, codec: Codec::None, ..Default::default() },
    )
    .await
    .unwrap();
    assert!(asking.info().tunable);
    assert_eq!(asking.info().center_hz, 1_090_000_000);
    assert_eq!(watching.info().center_hz, 1_090_000_000);

    asking.tune(433_920_000).await.unwrap();

    // Nothing has moved yet: the server parks the request for whoever owns
    // the tuner, which is the whole point of the split.
    let deadline = tokio::time::Instant::now() + Duration::from_secs(5);
    let mut asked = None;
    while asked.is_none() && tokio::time::Instant::now() < deadline {
        asked = srv.wanted();
        tokio::time::sleep(Duration::from_millis(10)).await;
    }
    assert_eq!(asked.map(|t| t.center_hz), Some(433_920_000));
    assert_eq!(asking.info().center_hz, 1_090_000_000, "not until the tuner moved");

    // The dongle stepped to somewhere near what was asked for, and says so.
    srv.retuned(433_921_500);
    let a = settle(&mut asking, &srv, 433_921_500).await;
    let w = settle(&mut watching, &srv, 433_921_500).await;
    assert_eq!(asking.info().center_hz, 433_921_500, "where it landed, not what was asked");
    assert_eq!(watching.info().center_hz, 433_921_500, "the one that did not ask is told too");
    assert_eq!(a, 433_921_500);
    assert_eq!(w, 433_921_500);
}

/// Read blocks until the stream says it is on `want`, and return the centre
/// the last block carried.
///
/// Samples travel over UDP and a retune over TCP, so the two arrive on
/// separate paths and a block sent just after the tuner moved can be taken
/// before the TUNED that describes it. What is pinned here is that the label
/// catches up, not that it was never briefly behind.
async fn settle(stream: &mut IqStream, srv: &Arc<Server>, want: u64) -> u64 {
    let block = ramp(256);
    let deadline = tokio::time::Instant::now() + Duration::from_secs(10);
    let mut last = stream.info().center_hz;
    while last != want && tokio::time::Instant::now() < deadline {
        for b in collect(stream, srv, &block, 1).await {
            last = b.center_hz;
        }
    }
    last
}

/// A server that was not offered says so in its welcome, the client refuses
/// before anything goes on the wire, and a request made anyway is refused
/// without ending the subscription.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn a_server_that_was_not_offered_refuses_a_tune() {
    let srv = server(false);
    let mut stream = IqStream::connect(
        srv.addr().to_string().as_str(),
        ClientConfig { name: "test".into(), bits: 8, codec: Codec::None, ..Default::default() },
    )
    .await
    .unwrap();
    assert!(!stream.info().tunable);
    assert!(stream.tune(433_920_000).await.is_err());
    assert_eq!(srv.wanted(), None);

    // And the samples keep coming, because a refused tune is not a fault.
    let got = collect(&mut stream, &srv, &ramp(256), 1).await;
    assert_eq!(got.len(), 1);
    assert_eq!(stream.info().center_hz, 1_090_000_000);
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn a_port_already_served_is_refused_rather_than_shared() {
    let first = server(false);
    let again = Server::start(first.addr(), ServerConfig::default());
    assert!(matches!(again, Err(common::Error::Busy)), "two servers on one port");
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn nobody_listening_costs_nothing() {
    let srv = server(false);
    assert_eq!(srv.subscribers(), 0);
    for _ in 0..1000 {
        srv.push(&ramp(4096));
    }
    assert_eq!(srv.blocks_sent(), 0, "a block with no subscriber is not packed");
}
