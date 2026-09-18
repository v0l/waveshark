//! Both ends of the protocol against each other: a server on a free port, a
//! client subscribing to it, and the samples arriving as they were pushed.

use iqstream::proto::Codec;
use iqstream::{ClientConfig, IqStream, Server, ServerConfig, Stream, StreamConfig};
use std::sync::Arc;
use std::time::Duration;

fn tuner(name: &str, center_hz: u64, tunable: bool) -> StreamConfig {
    StreamConfig {
        name: name.into(),
        center_hz,
        sample_rate: 2_400_000,
        gain_db: Some(49.6),
        tunable,
        tune_range_hz: Some((24_000_000, 1_766_000_000)),
        settings: Vec::new(),
    }
}

/// A server with one tuner on it, which is what a 1.1 server was.
fn server(tunable: bool) -> Arc<Server> {
    Server::start(
        "127.0.0.1:0".parse().unwrap(),
        ServerConfig::single("test", tuner("span", 1_090_000_000, tunable)),
    )
    .expect("a free port")
}

fn only(srv: &Arc<Server>) -> Arc<Stream> {
    srv.default_stream().expect("a tuner")
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
    let got = collect(&mut stream, &only(&srv), &block, 3).await;
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
    assert_eq!(only(&srv).subscribers(), 1);
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
    let got = collect(&mut stream, &only(&srv), &block, 1).await;
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
        asked = only(&srv).wanted();
        tokio::time::sleep(Duration::from_millis(10)).await;
    }
    assert_eq!(asked.map(|t| t.center_hz), Some(433_920_000));
    assert_eq!(asking.info().center_hz, 1_090_000_000, "not until the tuner moved");

    // The dongle stepped to somewhere near what was asked for, and says so.
    only(&srv).retuned(433_921_500);
    let a = settle(&mut asking, &only(&srv), 433_921_500).await;
    let w = settle(&mut watching, &only(&srv), 433_921_500).await;
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
async fn settle(stream: &mut IqStream, tuner: &Arc<Stream>, want: u64) -> u64 {
    let block = ramp(256);
    let deadline = tokio::time::Instant::now() + Duration::from_secs(10);
    let mut last = stream.info().center_hz;
    while last != want && tokio::time::Instant::now() < deadline {
        for b in collect(stream, tuner, &block, 1).await {
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
    assert_eq!(only(&srv).wanted(), None);

    // And the samples keep coming, because a refused tune is not a fault.
    let got = collect(&mut stream, &only(&srv), &ramp(256), 1).await;
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
    assert_eq!(only(&srv).subscribers(), 0);
    for _ in 0..1000 {
        only(&srv).push(&ramp(4096));
    }
    assert_eq!(only(&srv).blocks_sent(), 0, "a block with no subscriber is not packed");
}

/// Two dongles on one port: each reader gets the tuner it named and none of
/// the other's samples, and each is told where its own dial is.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn two_tuners_on_one_port_are_read_apart() {
    let srv = Server::start(
        "127.0.0.1:0".parse().unwrap(),
        ServerConfig {
            name: "test".into(),
            streams: vec![tuner("mast", 1_090_000_000, false), tuner("loft", 433_920_000, true)],
        },
    )
    .expect("a free port");
    let (mast, loft) = (srv.stream(0).unwrap(), srv.stream(1).unwrap());
    assert_eq!(mast.name(), "mast");
    assert_eq!(loft.name(), "loft");

    let addr = srv.addr().to_string();
    let reader = |stream| {
        let addr = addr.clone();
        async move {
            IqStream::connect(
                addr.as_str(),
                ClientConfig {
                    name: "test".into(),
                    bits: 8,
                    codec: Codec::None,
                    stream,
                    ..Default::default()
                },
            )
            .await
            .unwrap()
        }
    };
    let mut on_mast = reader(Some(0)).await;
    let mut on_loft = reader(Some(1)).await;

    // Both are offered, whichever was subscribed to.
    assert_eq!(on_mast.available().len(), 2);
    assert_eq!(on_loft.available().len(), 2);
    assert_eq!(on_mast.info().name, "mast");
    assert_eq!(on_mast.info().center_hz, 1_090_000_000);
    assert!(!on_mast.info().tunable, "the mast dial is held at the mast");
    assert_eq!(on_loft.info().name, "loft");
    assert_eq!(on_loft.info().center_hz, 433_920_000);
    assert!(on_loft.info().tunable);

    // A ramp each way, distinguishable by its first byte, pushed at both
    // tuners at once. Neither reader may see the other's.
    let (a, b) = (ramp(512), ramp(512).iter().map(|v| v ^ 0xff).collect::<Vec<u8>>());
    let deadline = tokio::time::Instant::now() + Duration::from_secs(10);
    let (mut got_a, mut got_b) = (Vec::new(), Vec::new());
    while (got_a.len() < 2 || got_b.len() < 2) && tokio::time::Instant::now() < deadline {
        mast.push(&a);
        loft.push(&b);
        if let Ok(Ok(Some(block))) =
            tokio::time::timeout(Duration::from_millis(50), on_mast.next_block()).await
        {
            got_a.push(block);
        }
        if let Ok(Ok(Some(block))) =
            tokio::time::timeout(Duration::from_millis(50), on_loft.next_block()).await
        {
            got_b.push(block);
        }
    }
    assert!(got_a.len() >= 2, "{} blocks off the mast", got_a.len());
    assert!(got_b.len() >= 2, "{} blocks off the loft", got_b.len());
    for block in &got_a {
        assert_eq!(block.stream_id, 0);
        assert_eq!(block.samples, a, "the loft's samples reached the mast's reader");
        assert_eq!(block.center_hz, 1_090_000_000);
    }
    for block in &got_b {
        assert_eq!(block.stream_id, 1);
        assert_eq!(block.samples, b);
        assert_eq!(block.center_hz, 433_920_000);
    }
    assert_eq!(mast.subscribers(), 1);
    assert_eq!(loft.subscribers(), 1);
}

/// A tune names a dial. The other tuner does not move, and the reader on it
/// is not told anything about the one that did.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn a_tune_moves_the_dial_it_named_and_no_other() {
    let srv = Server::start(
        "127.0.0.1:0".parse().unwrap(),
        ServerConfig {
            name: "test".into(),
            streams: vec![tuner("mast", 1_090_000_000, true), tuner("loft", 433_920_000, true)],
        },
    )
    .expect("a free port");
    let (mast, loft) = (srv.stream(0).unwrap(), srv.stream(1).unwrap());

    let mut on_loft = IqStream::connect(
        srv.addr().to_string().as_str(),
        ClientConfig {
            name: "test".into(),
            bits: 8,
            codec: Codec::None,
            stream: Some(1),
            ..Default::default()
        },
    )
    .await
    .unwrap();
    on_loft.tune(868_300_000).await.unwrap();

    let deadline = tokio::time::Instant::now() + Duration::from_secs(5);
    let mut asked = None;
    while asked.is_none() && tokio::time::Instant::now() < deadline {
        asked = loft.wanted();
        assert_eq!(mast.wanted(), None, "the mast was asked for nothing");
        tokio::time::sleep(Duration::from_millis(10)).await;
    }
    assert_eq!(asked.map(|t| t.center_hz), Some(868_300_000));

    loft.retuned(868_301_200);
    assert_eq!(settle(&mut on_loft, &loft, 868_301_200).await, 868_301_200);
    assert_eq!(mast.center_hz(), 1_090_000_000, "the other dial stayed put");
}

/// A tuner added while a client is connected is announced, and one taken away
/// ends its readers rather than leaving them on a stream that is gone.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn a_tuner_plugged_in_or_pulled_out_reaches_the_readers() {
    let srv = server(false);
    let mut watching = IqStream::connect(
        srv.addr().to_string().as_str(),
        ClientConfig { name: "test".into(), bits: 8, codec: Codec::None, ..Default::default() },
    )
    .await
    .unwrap();
    assert_eq!(watching.available().len(), 1);

    let added = srv.add_stream(tuner("loft", 433_920_000, true));
    assert_eq!(added.id(), 1);

    // The announcement arrives on the control connection, which is read as
    // blocks are taken.
    let deadline = tokio::time::Instant::now() + Duration::from_secs(10);
    while watching.available().len() < 2 && tokio::time::Instant::now() < deadline {
        collect(&mut watching, &only(&srv), &ramp(256), 1).await;
    }
    assert_eq!(watching.available().len(), 2, "the new tuner was announced");
    assert_eq!(watching.available()[1].name, "loft");

    srv.remove_stream(1);
    let deadline = tokio::time::Instant::now() + Duration::from_secs(10);
    while watching.available().len() > 1 && tokio::time::Instant::now() < deadline {
        collect(&mut watching, &only(&srv), &ramp(256), 1).await;
    }
    assert_eq!(watching.available().len(), 1, "and so was the one taken away");
    assert_eq!(srv.streams().len(), 1);
}

/// Naming a tuner that is not there is refused, and refused as such rather
/// than by handing over whatever stream happened to be first.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn a_stream_that_is_not_there_is_refused() {
    let srv = server(false);
    let refused = IqStream::connect(
        srv.addr().to_string().as_str(),
        ClientConfig {
            name: "test".into(),
            bits: 8,
            codec: Codec::None,
            stream: Some(7),
            ..Default::default()
        },
    )
    .await;
    assert!(refused.is_err(), "there is no tuner 7");
    assert_eq!(only(&srv).subscribers(), 0);
}

fn gain(db: f32) -> iqstream::Setting {
    iqstream::Setting {
        name: "tuner".into(),
        label: "RF gain".into(),
        kind: iqstream::SettingKind::Gain,
        value: iqstream::SettingValue::Gain(db),
        options: Vec::new(),
        range_db: Some((0.0, 49.6)),
    }
}

fn bias(on: bool) -> iqstream::Setting {
    iqstream::Setting {
        name: "bias_t".into(),
        label: "Bias tee".into(),
        kind: iqstream::SettingKind::Switch,
        value: iqstream::SettingValue::Switch(on),
        options: Vec::new(),
        range_db: None,
    }
}

/// A gain turned down or a switch thrown reaches the reader, because what it
/// hears was heard at the new setting and a level reported against the old
/// one is a level nothing was taken at.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn a_setting_that_moves_reaches_the_readers() {
    let srv = server(false);
    let tuner = only(&srv);
    tuner.set_settings(vec![gain(32.8), bias(false)]);

    let mut reading = IqStream::connect(
        srv.addr().to_string().as_str(),
        ClientConfig { name: "test".into(), bits: 8, codec: Codec::None, ..Default::default() },
    )
    .await
    .unwrap();
    // The welcome already carries them, before anything has changed.
    assert_eq!(reading.info().settings, vec![gain(32.8), bias(false)]);
    assert_eq!(reading.info().gain_db, Some(49.6), "what the server started at");

    tuner.set_settings(vec![gain(14.4), bias(true)]);
    tuner.set_gain_db(Some(14.4));
    let settled = |s: &IqStream| {
        s.info().settings == vec![gain(14.4), bias(true)] && s.info().gain_db == Some(14.4)
    };
    let deadline = tokio::time::Instant::now() + Duration::from_secs(10);
    while !settled(&reading) && tokio::time::Instant::now() < deadline {
        collect(&mut reading, &tuner, &ramp(256), 1).await;
    }
    assert_eq!(reading.info().settings, vec![gain(14.4), bias(true)]);
    assert_eq!(reading.info().gain_db, Some(14.4), "the whole front end's level, too");
    // And the same tuner in the list of what the server offers, so a pane
    // showing the others is showing what they are set to now.
    assert_eq!(reading.available()[0].settings, vec![gain(14.4), bias(true)]);
}

/// A setting on a tuner nobody here subscribed to still reaches this
/// connection: a reader choosing between them has to see what each is set to.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn a_setting_on_another_tuner_reaches_a_reader_of_the_first() {
    let srv = Server::start(
        "127.0.0.1:0".parse().unwrap(),
        ServerConfig {
            name: "test".into(),
            streams: vec![tuner("mast", 1_090_000_000, false), tuner("loft", 433_920_000, true)],
        },
    )
    .expect("a free port");
    let (mast, loft) = (srv.stream(0).unwrap(), srv.stream(1).unwrap());

    let mut on_mast = IqStream::connect(
        srv.addr().to_string().as_str(),
        ClientConfig {
            name: "test".into(),
            bits: 8,
            codec: Codec::None,
            stream: Some(0),
            ..Default::default()
        },
    )
    .await
    .unwrap();
    loft.set_settings(vec![bias(true)]);

    let deadline = tokio::time::Instant::now() + Duration::from_secs(10);
    while on_mast.available()[1].settings.is_empty() && tokio::time::Instant::now() < deadline {
        collect(&mut on_mast, &mast, &ramp(256), 1).await;
    }
    assert_eq!(on_mast.available()[1].settings, vec![bias(true)]);
    assert!(on_mast.info().settings.is_empty(), "the one being read did not move");
}

/// A set that has not changed is not announced: the stage reads the driver
/// twice a second and a message each time would be a message a second per
/// reader for nothing.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn a_setting_written_again_unchanged_says_nothing() {
    let srv = server(false);
    let tuner = only(&srv);
    tuner.set_settings(vec![gain(32.8)]);
    let mut reading = IqStream::connect(
        srv.addr().to_string().as_str(),
        ClientConfig { name: "test".into(), bits: 8, codec: Codec::None, ..Default::default() },
    )
    .await
    .unwrap();
    assert_eq!(reading.info().settings, vec![gain(32.8)]);

    for _ in 0..20 {
        tuner.set_settings(vec![gain(32.8)]);
        tuner.set_sample_rate(2_400_000);
    }
    // Nothing to deliver, so the connection carries samples and keepalives
    // alone and the reading is what it was.
    collect(&mut reading, &tuner, &ramp(256), 2).await;
    assert_eq!(reading.info().settings, vec![gain(32.8)]);
    assert_eq!(reading.info().sample_rate, 2_400_000);
}

fn antenna(selected: &str) -> iqstream::Setting {
    iqstream::Setting {
        name: "antenna".into(),
        label: "Antenna".into(),
        kind: iqstream::SettingKind::Choice,
        value: iqstream::SettingValue::Choice(selected.into()),
        options: vec!["LNAH".into(), "LNAL".into(), "LNAW".into()],
        range_db: None,
    }
}

/// A subscriber asks for a gain, a switch and a port; the owner of the radio
/// carries them out and says what they became, which is not always what was
/// asked for.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn a_setting_can_be_asked_of_a_tuner_that_was_offered() {
    let srv = server(true);
    let tuner = only(&srv);
    tuner.set_settings(vec![gain(32.8), bias(false), antenna("LNAH")]);

    let mut asking = IqStream::connect(
        srv.addr().to_string().as_str(),
        ClientConfig { name: "test".into(), bits: 8, codec: Codec::None, ..Default::default() },
    )
    .await
    .unwrap();
    asking.set_setting("tuner", iqstream::SettingValue::Gain(14.0)).await.unwrap();
    asking.set_setting("bias_t", iqstream::SettingValue::Switch(true)).await.unwrap();
    asking.set_setting("antenna", iqstream::SettingValue::Choice("LNAW".into())).await.unwrap();

    // Nothing has moved yet: the requests are parked for whoever owns the
    // radio, exactly as a tune is.
    let deadline = tokio::time::Instant::now() + Duration::from_secs(5);
    let mut asked = Vec::new();
    while asked.len() < 3 && tokio::time::Instant::now() < deadline {
        asked.extend(tuner.asked());
        tokio::time::sleep(Duration::from_millis(10)).await;
    }
    assert_eq!(asked.len(), 3, "three requests, in the order they were made: {asked:?}");
    assert_eq!(asked[0].name, "tuner");
    assert_eq!(asked[0].value, iqstream::SettingValue::Gain(14.0));
    assert_eq!(asked[1].value, iqstream::SettingValue::Switch(true));
    assert_eq!(asked[2].value, iqstream::SettingValue::Choice("LNAW".into()));
    assert_eq!(asking.info().settings[0], gain(32.8), "not until the radio moved");

    // The dongle snapped the gain to its nearest step, and says so.
    tuner.set_settings(vec![gain(14.4), bias(true), antenna("LNAW")]);
    let deadline = tokio::time::Instant::now() + Duration::from_secs(10);
    while asking.info().settings[0] == gain(32.8) && tokio::time::Instant::now() < deadline {
        collect(&mut asking, &tuner, &ramp(256), 1).await;
    }
    assert_eq!(asking.info().settings, vec![gain(14.4), bias(true), antenna("LNAW")]);
}

/// A dial that was not offered is not a set of controls either, and a name
/// the tuner never had is refused rather than parked for nobody.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn a_setting_is_refused_where_the_radio_was_not_offered() {
    let held = server(false);
    only(&held).set_settings(vec![gain(32.8)]);
    let mut reading = IqStream::connect(
        held.addr().to_string().as_str(),
        ClientConfig { name: "test".into(), bits: 8, codec: Codec::None, ..Default::default() },
    )
    .await
    .unwrap();
    assert!(reading.set_setting("tuner", iqstream::SettingValue::Gain(14.0)).await.is_err());
    assert!(only(&held).asked().is_empty());

    let offered = server(true);
    only(&offered).set_settings(vec![gain(32.8)]);
    let mut asking = IqStream::connect(
        offered.addr().to_string().as_str(),
        ClientConfig { name: "test".into(), bits: 8, codec: Codec::None, ..Default::default() },
    )
    .await
    .unwrap();
    assert!(
        asking.set_setting("bias_t", iqstream::SettingValue::Switch(true)).await.is_err(),
        "this tuner has no bias tee"
    );
    // And the subscription is not ended by either refusal: the samples keep
    // coming from a radio that would not be set.
    assert_eq!(collect(&mut asking, &only(&offered), &ramp(256), 1).await.len(), 1);
    assert!(only(&offered).asked().is_empty());
}

/// A gain dragged across its range is one request by the time the owner of
/// the radio looks, because only the last of them means anything.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn a_setting_asked_for_repeatedly_is_taken_once() {
    let srv = server(true);
    let tuner = only(&srv);
    tuner.set_settings(vec![gain(32.8), bias(false)]);
    for db in [10.0, 20.0, 30.0, 40.0] {
        tuner.ask_setting("tuner", iqstream::SettingValue::Gain(db));
    }
    tuner.ask_setting("bias_t", iqstream::SettingValue::Switch(true));
    let asked = tuner.asked();
    assert_eq!(asked.len(), 2, "one per control: {asked:?}");
    assert_eq!(asked[0].value, iqstream::SettingValue::Gain(40.0), "the last one asked for");
    assert!(tuner.asked().is_empty(), "taken means taken");
}

/// A tuner taken off the server ends its readers. The control connection
/// stays up, so without being told a reader would wait for samples that are
/// never coming.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn a_reader_of_a_tuner_that_is_taken_away_is_ended() {
    let srv = Server::start(
        "127.0.0.1:0".parse().unwrap(),
        ServerConfig {
            name: "test".into(),
            streams: vec![tuner("mast", 1_090_000_000, false), tuner("loft", 433_920_000, false)],
        },
    )
    .expect("a free port");
    let loft = srv.stream(1).unwrap();
    let mut on_loft = IqStream::connect(
        srv.addr().to_string().as_str(),
        ClientConfig {
            name: "test".into(),
            bits: 8,
            codec: Codec::None,
            stream: Some(1),
            ..Default::default()
        },
    )
    .await
    .unwrap();
    assert_eq!(collect(&mut on_loft, &loft, &ramp(256), 1).await.len(), 1);

    srv.remove_stream(1);
    let deadline = tokio::time::Instant::now() + Duration::from_secs(10);
    let mut ending = None;
    while ending.is_none() && tokio::time::Instant::now() < deadline {
        match tokio::time::timeout(Duration::from_millis(100), on_loft.next_block()).await {
            Ok(Ok(None)) => ending = Some(()),
            Ok(Err(e)) => panic!("{e}"),
            _ => continue,
        }
    }
    assert!(ending.is_some(), "the reader was left waiting on a tuner that is gone");
    assert_eq!(loft.subscribers(), 0);
    assert_eq!(srv.streams().len(), 1, "and the other tuner is still there");
}
