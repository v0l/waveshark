//! Reaching a client the server cannot address, and sizing what it sends.
//!
//! The client here is written out by hand rather than taken from
//! [`iqstream::IqStream`], because what is being pinned is what the server
//! does with a client that lies about its port, one that never punches at
//! all, and one that answers for some datagram sizes and not others. A real
//! client cannot be made to do any of those.

use iqstream::proto::{
    CONTROL_MAGIC, DATA_HEADER_LEN, DataHeader, Frame, PREAMBLE_LEN, Tlvs, Transport, decode_probe,
    encode_punch, msg, tag,
};
use iqstream::{Server, ServerConfig, Stream, StreamConfig};
use std::sync::Arc;
use std::time::Duration;
use tokio::io::{AsyncReadExt, AsyncWriteExt};
use tokio::net::{TcpStream, UdpSocket};

/// The server's own data port is the one a punch opens a hole to, and the one
/// every datagram comes back from.
fn served() -> (Arc<Server>, Arc<Stream>) {
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

struct Bare {
    control: TcpStream,
    data_port: u16,
}

/// Connect, greet, and take the port the server says punches go to.
async fn greet(srv: &Arc<Server>) -> Bare {
    let mut control = TcpStream::connect(srv.addr()).await.unwrap();
    let mut preamble = [0u8; PREAMBLE_LEN];
    preamble[0..4].copy_from_slice(&CONTROL_MAGIC);
    preamble[4..6].copy_from_slice(&1u16.to_le_bytes());
    preamble[6..8].copy_from_slice(&4u16.to_le_bytes());
    control.write_all(&preamble).await.unwrap();
    control.read_exact(&mut [0u8; PREAMBLE_LEN]).await.unwrap();

    let mut hello = Tlvs::new();
    hello.str(tag::CLIENT_NAME, "bare");
    control.write_all(&Frame::new(msg::HELLO, &hello).encode()).await.unwrap();
    let welcome = read_frame(&mut control).await;
    assert_eq!(welcome.msg_type, msg::WELCOME);
    let data_port = welcome.tlvs().unwrap().u16(tag::DATA_PORT).expect("a 1.4 server punches");
    assert_eq!(data_port, srv.addr().port(), "punches go to the port the control came in on");
    Bare { control, data_port }
}

async fn read_frame(sock: &mut TcpStream) -> Frame {
    let mut head = [0u8; 4];
    sock.read_exact(&mut head).await.unwrap();
    let len = u16::from_le_bytes([head[2], head[3]]) as usize;
    let mut payload = vec![0u8; len];
    sock.read_exact(&mut payload).await.unwrap();
    Frame { version: head[0], msg_type: head[1], payload }
}

async fn subscribe(bare: &mut Bare, token: Option<u64>, udp_port: Option<u16>) {
    let mut s = Tlvs::new();
    s.u16(tag::STREAM_ID, 0)
        .u8(tag::BIT_DEPTH, 8)
        .u8(tag::CODEC, 0)
        .u16(tag::DECIMATION, 1)
        .u8(tag::TRANSPORT, Transport::Udp.code());
    if let Some(token) = token {
        s.u64(tag::PUNCH_TOKEN, token);
    }
    if let Some(port) = udp_port {
        s.u16(tag::UDP_PORT, port);
    }
    bare.control.write_all(&Frame::new(msg::SUBSCRIBE, &s).encode()).await.unwrap();
    let reply = read_frame(&mut bare.control).await;
    assert_eq!(reply.msg_type, msg::SUBSCRIBED);
    assert_eq!(reply.tlvs().unwrap().u8(tag::TRANSPORT), Some(0), "over UDP");
}

/// Push at the tuner until `want` datagrams have arrived on `sock` or the
/// deadline passes, answering probes of `answer` bytes and no others.
///
/// Returns every datagram taken, with its length: what the path was told it
/// could carry shows up as the size of the fragments.
async fn gather(
    tuner: &Arc<Stream>,
    sock: &UdpSocket,
    control: &mut TcpStream,
    want: usize,
    answer: Option<u16>,
) -> Vec<(usize, DataHeader)> {
    let block = ramp(4096);
    let mut got = Vec::new();
    let mut buf = [0u8; 65536];
    let deadline = tokio::time::Instant::now() + Duration::from_secs(10);
    while got.len() < want && tokio::time::Instant::now() < deadline {
        tuner.push(&block);
        let Ok(Ok(n)) = tokio::time::timeout(Duration::from_millis(50), sock.recv(&mut buf)).await
        else {
            continue;
        };
        if let Some((_, size)) = decode_probe(&buf[..n]) {
            if answer == Some(size) {
                let mut t = Tlvs::new();
                t.u16(tag::STREAM_ID, 0).u16(tag::PROBE_SIZE, size);
                control.write_all(&Frame::new(msg::PROBED, &t).encode()).await.unwrap();
            }
            continue;
        }
        let (header, _) = DataHeader::decode(&buf[..n]).expect("a data datagram");
        got.push((n, header));
    }
    got
}

/// The port a client names is its own idea of where it is, which behind a NAT
/// is not where anything reaches it. The server sends to the address the
/// punch came from instead, and the named port gets nothing.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn samples_go_to_the_address_that_punched_and_not_the_port_that_was_named() {
    let (srv, tuner) = served();
    let mut bare = greet(&srv).await;

    let punched = UdpSocket::bind("127.0.0.1:0").await.unwrap();
    let claimed = UdpSocket::bind("127.0.0.1:0").await.unwrap();
    let wrong = claimed.local_addr().unwrap().port();
    assert_ne!(wrong, punched.local_addr().unwrap().port());

    let token = 0x5ea5_1de5_0f11_11a1;
    punched.send_to(&encode_punch(token), ("127.0.0.1", bare.data_port)).await.unwrap();
    subscribe(&mut bare, Some(token), Some(wrong)).await;

    let got = gather(&tuner, &punched, &mut bare.control, 8, None).await;
    assert_eq!(got.len(), 8, "the punched socket was served");
    assert_eq!(got[0].1.stream_id, 0);
    assert_eq!(got[0].1.block_samples, 4096);
    assert_eq!(got[0].1.bit_depth, 8);

    let mut nothing = [0u8; 2048];
    let at_the_wrong_port =
        tokio::time::timeout(Duration::from_millis(200), claimed.recv(&mut nothing)).await;
    assert!(at_the_wrong_port.is_err(), "the port the client named was served anyway");
    assert_eq!(tuner.subscribers(), 1);
}

/// A client from before there were punches names its port and is served
/// exactly as it always was, at the full 1400 byte payload.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn a_client_that_names_a_port_and_never_punches_is_served_there() {
    let (srv, tuner) = served();
    let mut bare = greet(&srv).await;
    let sock = UdpSocket::bind("127.0.0.1:0").await.unwrap();
    subscribe(&mut bare, None, Some(sock.local_addr().unwrap().port())).await;

    let got = gather(&tuner, &sock, &mut bare.control, 12, None).await;
    assert_eq!(got.len(), 12);
    let largest = got.iter().map(|(n, _)| *n).max().unwrap();
    assert_eq!(largest, DATA_HEADER_LEN + 1400, "unprobed clients keep the 1.3 datagram");
    assert_eq!(got[0].1.frag_count, 6, "8192 bytes in 1400 byte pieces");
}

/// The path is measured rather than assumed: the server starts at the size
/// every path takes, offers larger ones, and uses the largest that was
/// answered for.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn datagrams_grow_to_the_largest_size_the_client_answered_for() {
    let (srv, tuner) = served();
    let mut bare = greet(&srv).await;
    let sock = UdpSocket::bind("127.0.0.1:0").await.unwrap();
    let token = 0x0102_0304_0506_0708;
    sock.send_to(&encode_punch(token), ("127.0.0.1", bare.data_port)).await.unwrap();
    subscribe(&mut bare, Some(token), None).await;

    // Answering for 1228 and not for 1400 is a path of 1312 bytes, which is
    // an ordinary PPPoE WAN and the case the whole measurement is for.
    let got = gather(&tuner, &sock, &mut bare.control, 200, Some(1228)).await;
    assert_eq!(got.len(), 200);
    let largest = got.iter().map(|(n, _)| *n).max().unwrap();
    assert_eq!(largest, DATA_HEADER_LEN + 1228, "the size that was answered for");
    assert!(
        got.iter().any(|(n, _)| *n == DATA_HEADER_LEN + 1196),
        "the first blocks leave at the size any path takes"
    );
    assert_eq!(got.last().unwrap().1.frag_count, 7, "8192 bytes in 1228 byte pieces");
    assert!(
        got[150..].iter().all(|(n, _)| *n != DATA_HEADER_LEN + 1196),
        "a measured path went back to the floor"
    );
}

/// A path that answers for nothing keeps the size no path can refuse, rather
/// than the server guessing at a larger one.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn a_path_that_answers_for_nothing_stays_at_the_floor() {
    let (srv, tuner) = served();
    let mut bare = greet(&srv).await;
    let sock = UdpSocket::bind("127.0.0.1:0").await.unwrap();
    let token = 0x1111_2222_3333_4444;
    sock.send_to(&encode_punch(token), ("127.0.0.1", bare.data_port)).await.unwrap();
    subscribe(&mut bare, Some(token), None).await;

    let got = gather(&tuner, &sock, &mut bare.control, 20, None).await;
    assert_eq!(got.len(), 20);
    let largest = got.iter().map(|(n, _)| *n).max().unwrap();
    assert_eq!(largest, DATA_HEADER_LEN + 1196, "1280 bytes on the wire, the IPv6 minimum");
    assert_eq!(got[0].1.frag_count, 7);
}

/// Nothing is sent anywhere until the client has been heard from: a
/// subscription that names no port and never punches is an address the server
/// does not have.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn a_subscriber_that_never_punches_is_sent_nothing() {
    let (srv, tuner) = served();
    let mut bare = greet(&srv).await;
    let sock = UdpSocket::bind("127.0.0.1:0").await.unwrap();
    subscribe(&mut bare, Some(0xdead_beef_dead_beef), None).await;

    let block = ramp(4096);
    for _ in 0..50 {
        tuner.push(&block);
    }
    let mut buf = [0u8; 65536];
    let quiet = tokio::time::timeout(Duration::from_millis(300), sock.recv(&mut buf)).await;
    assert!(quiet.is_err(), "samples went somewhere they were never asked for");
    assert_eq!(tuner.blocks_sent(), 0, "nothing was pumped at an unknown address");
    assert_eq!(tuner.subscribers(), 1, "and the subscription is still open for the punch");
}

/// Naming neither a port nor a token is a subscribe with nowhere to go, and
/// is refused rather than accepted into silence.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn a_subscribe_with_no_address_at_all_is_refused() {
    let (srv, tuner) = served();
    let mut bare = greet(&srv).await;
    let mut s = Tlvs::new();
    s.u16(tag::STREAM_ID, 0).u8(tag::BIT_DEPTH, 8).u8(tag::TRANSPORT, Transport::Udp.code());
    bare.control.write_all(&Frame::new(msg::SUBSCRIBE, &s).encode()).await.unwrap();
    let reply = read_frame(&mut bare.control).await;
    assert_eq!(reply.msg_type, msg::ERROR);
    assert_eq!(reply.tlvs().unwrap().u16(tag::ERROR_CODE), Some(1), "a bad request");
    assert_eq!(tuner.subscribers(), 0);
}

/// A transport this server has never heard of is refused as such, so a later
/// version asking for one can tell that from a server that is simply broken.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn a_transport_the_server_does_not_know_is_refused() {
    let (srv, tuner) = served();
    let mut bare = greet(&srv).await;
    let mut s = Tlvs::new();
    s.u16(tag::STREAM_ID, 0).u8(tag::BIT_DEPTH, 8).u8(tag::TRANSPORT, 9);
    bare.control.write_all(&Frame::new(msg::SUBSCRIBE, &s).encode()).await.unwrap();
    let reply = read_frame(&mut bare.control).await;
    assert_eq!(reply.msg_type, msg::ERROR);
    assert_eq!(reply.tlvs().unwrap().u16(tag::ERROR_CODE), Some(10), "unsupported transport");
    assert_eq!(tuner.subscribers(), 0);
}
