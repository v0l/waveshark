//! Samples on the control connection, for a client no datagram reaches.

use iqstream::proto::Codec;
use iqstream::{
    ClientConfig, IqStream, Prefer, Server, ServerConfig, Stream, StreamConfig, Transport,
};
use std::sync::Arc;
use std::time::Duration;
use tokio::net::TcpListener;

fn server() -> (Arc<Server>, Arc<Stream>) {
    let srv = Server::start(
        "127.0.0.1:0".parse().unwrap(),
        ServerConfig::single(
            "test",
            StreamConfig {
                name: "span".into(),
                center_hz: 1_090_000_000,
                sample_rate: 2_400_000,
                gain_db: Some(49.6),
                tunable: false,
                tune_range_hz: None,
                settings: Vec::new(),
            },
        ),
    )
    .expect("a free port");
    let tuner = srv.default_stream().expect("a tuner");
    (srv, tuner)
}

fn ramp(samples: usize) -> Vec<u8> {
    (0..samples * 2).map(|i| (i % 251) as u8).collect()
}

async fn collect(
    stream: &mut IqStream,
    tuner: &Arc<Stream>,
    block: &[u8],
    want: usize,
) -> Vec<iqstream::Block> {
    let mut got = Vec::new();
    let deadline = tokio::time::Instant::now() + Duration::from_secs(10);
    while got.len() < want && tokio::time::Instant::now() < deadline {
        tuner.push(block);
        match tokio::time::timeout(Duration::from_millis(100), stream.next_block()).await {
            Ok(Ok(Some(b))) => got.push(b),
            Ok(Ok(None)) => break,
            Ok(Err(e)) => panic!("{e}"),
            Err(_) => continue,
        }
    }
    got
}

/// Asked for on the connection from the start, the samples arrive whole: the
/// stream underneath is already in order, so nothing is cut into fragments
/// and nothing is lost.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn samples_asked_for_on_the_control_connection_arrive_whole() {
    let (srv, tuner) = server();
    let mut stream = IqStream::connect(
        srv.addr().to_string().as_str(),
        ClientConfig {
            name: "test".into(),
            bits: 8,
            codec: Codec::None,
            transport: Prefer::Tcp,
            ..Default::default()
        },
    )
    .await
    .unwrap();
    assert_eq!(stream.transport(), Transport::Tcp);
    assert_eq!(stream.local_port(), 0, "no datagram socket was bound at all");

    // 4096 complex samples is 8192 bytes, which over UDP is six or seven
    // datagrams and here is one record.
    let block = ramp(4096);
    let got = collect(&mut stream, &tuner, &block, 5).await;
    assert_eq!(got.len(), 5);
    for b in &got {
        assert_eq!(b.samples.len(), 8192);
        assert_eq!(b.samples, block, "a byte changed in flight");
        assert_eq!(b.padded_before, 0, "nothing was lost on a stream that cannot lose it");
        assert_eq!(b.center_hz, 1_090_000_000);
    }
    assert_eq!(got[0].sample_index, 0);
    assert_eq!(got[4].sample_index, 4 * 4096);
    assert_eq!(stream.stats().blocks, 5);
    assert_eq!(stream.stats().incomplete_blocks, 0);
    assert_eq!(tuner.blocks_dropped(), 0, "a reader keeping up loses nothing");
    assert_eq!(tuner.subscribers(), 1);
}

/// A connection through something that carries TCP and not UDP, which is
/// every symmetric NAT and every firewall with the datagrams turned off: the
/// punch goes nowhere, nothing arrives, and the client asks for the samples
/// on the connection it already has.
#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn a_client_no_datagram_reaches_reads_the_samples_off_the_connection() {
    let (srv, tuner) = server();

    // The relay is at an address of its own, so the client punches at 127.0.0.2
    // where the server is at 127.0.0.1 and nothing answers: the server never
    // learns where to send, exactly as through a symmetric NAT.
    let relay = TcpListener::bind("127.0.0.2:0").await.unwrap();
    let through = relay.local_addr().unwrap();
    let to = srv.addr();
    tokio::spawn(async move {
        while let Ok((mut client, _)) = relay.accept().await {
            tokio::spawn(async move {
                let Ok(mut server) = tokio::net::TcpStream::connect(to).await else {
                    return;
                };
                let _ = tokio::io::copy_bidirectional(&mut client, &mut server).await;
            });
        }
    });

    let mut stream = IqStream::connect(
        through.to_string().as_str(),
        ClientConfig {
            name: "test".into(),
            bits: 8,
            codec: Codec::None,
            udp_timeout: Duration::from_millis(300),
            ..Default::default()
        },
    )
    .await
    .unwrap();
    assert_eq!(stream.transport(), Transport::Udp, "UDP is what it asks for first");

    let block = ramp(4096);
    let got = collect(&mut stream, &tuner, &block, 4).await;
    assert_eq!(got.len(), 4, "the samples never arrived by either route");
    assert_eq!(stream.transport(), Transport::Tcp, "it gave up on the datagrams");
    for b in &got {
        assert_eq!(b.samples, block);
    }
    assert_eq!(got[0].sample_index, 0, "the second subscription counts from the start");
    assert_eq!(got[3].sample_index, 3 * 4096);
    assert_eq!(tuner.subscribers(), 1, "the subscription it gave up on was replaced");
}

/// A reader that stops taking its samples has them thrown away rather than
/// held: they share a socket with the keepalives, and a stream that could
/// hold those up would take the subscription down with it.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn a_reader_that_stops_taking_blocks_has_them_dropped_rather_than_queued() {
    let (srv, tuner) = server();
    let mut stream = IqStream::connect(
        srv.addr().to_string().as_str(),
        ClientConfig {
            name: "test".into(),
            bits: 8,
            codec: Codec::None,
            transport: Prefer::Tcp,
            ..Default::default()
        },
    )
    .await
    .unwrap();
    let block = ramp(4096);
    assert_eq!(collect(&mut stream, &tuner, &block, 1).await.len(), 1);

    // Nothing is read for the length of this, so the queue to the socket
    // fills and the pump has to choose between dropping and waiting.
    let accounted = || tuner.blocks_sent() + tuner.blocks_dropped();
    let before = accounted();
    for _ in 0..2000 {
        tuner.push(&block);
        tokio::task::yield_now().await;
    }
    let deadline = tokio::time::Instant::now() + Duration::from_secs(10);
    while accounted() < before + 2000 && tokio::time::Instant::now() < deadline {
        tokio::time::sleep(Duration::from_millis(10)).await;
    }
    let dropped = tuner.blocks_dropped();
    assert!(
        dropped >= 1,
        "2000 blocks pushed at a reader taking none dropped {dropped}, which is below the floor of 1"
    );
    assert!(dropped <= 2000, "{dropped} dropped is more than were ever pushed");
    assert_eq!(tuner.subscribers(), 1, "and the subscription survived it");

    // The connection is still a connection: the unsubscribe goes up, the
    // answer comes back down past the blocks that were kept.
    stream.unsubscribe().await.unwrap();
    let deadline = tokio::time::Instant::now() + Duration::from_secs(10);
    let mut ended = false;
    while !ended && tokio::time::Instant::now() < deadline {
        match tokio::time::timeout(Duration::from_millis(100), stream.next_block()).await {
            Ok(Ok(None)) => ended = true,
            Ok(Err(e)) => panic!("{e}"),
            _ => continue,
        }
    }
    assert!(ended, "the unsubscribe was stuck behind the samples");
}
