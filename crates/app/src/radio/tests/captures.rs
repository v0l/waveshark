use super::*;

/// An ISM sensor through the whole receiver and out to a broker.
///
/// The publisher, the node and the packet bus each have tests of their
/// own; what none of them said is that a sensor heard by the live
/// receiver reaches a broker, which is the only thing an operator can
/// see. A socket that speaks enough MQTT to accept the connection stands
/// in for the broker, and what arrives on it is what Home Assistant
/// would read.
#[test]
fn a_sensor_heard_by_the_receiver_reaches_the_house() {
    let _installing = decode::script::test_lock();
    if !decode::script::install_fetched() {
        return;
    }
    use std::io::{Read, Write};
    let p = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../testdata/fineoffset_wh1080_433.92M_250k.cu8");
    if !p.exists() {
        eprintln!("skipping: fineoffset_wh1080_433.92M_250k.cu8 absent, run testdata/fetch.sh");
        return;
    }
    let buf = sources::FileSource::open(&p).unwrap().read_all().unwrap();

    let listener = std::net::TcpListener::bind("127.0.0.1:0").expect("a port");
    let port = listener.local_addr().unwrap().port();
    let (tx, seen) = std::sync::mpsc::channel::<(String, String)>();
    std::thread::spawn(move || {
        let (mut sock, _) = listener.accept().expect("a connection");
        let mut held = Vec::new();
        let mut chunk = [0u8; 4096];
        loop {
            let n = match sock.read(&mut chunk) {
                Ok(0) | Err(_) => return,
                Ok(n) => n,
            };
            held.extend_from_slice(&chunk[..n]);
            while let Some((kind, flags, body, used)) = nodes::mqtt_packet(&held) {
                held.drain(..used);
                match kind {
                    1 => {
                        if sock.write_all(&[0x20, 0x02, 0x00, 0x00]).is_err() {
                            return;
                        }
                    }
                    3 => {
                        let tl = u16::from_be_bytes([body[0], body[1]]) as usize;
                        let topic = String::from_utf8_lossy(&body[2..2 + tl]).to_string();
                        let at = 2 + tl + if (flags >> 1) & 3 > 0 { 2 } else { 0 };
                        let payload = String::from_utf8_lossy(&body[at..]).to_string();
                        if tx.send((topic, payload)).is_err() {
                            return;
                        }
                    }
                    _ => {}
                }
            }
        }
    });

    let mut rx = replay_receiver(&buf, None).expect("a receiver");
    let mut plan = replay_plan(&buf, false);
    plan.settings.homeassistant = Some(nodes::Publish {
        broker: nodes::Broker { port, ..nodes::Broker::new("127.0.0.1") },
        spaces: "ism".into(),
        buses: true,
    });
    rx.apply_settings(&plan);
    let up = std::time::Instant::now();
    while !rx.homeassistant_status().is_some_and(|s| s.connected)
        && up.elapsed() < std::time::Duration::from_secs(5)
    {
        std::thread::sleep(std::time::Duration::from_millis(10));
    }
    let st = rx.homeassistant_status();
    assert!(st.as_ref().is_some_and(|s| s.connected), "never connected: {st:?}");

    let rows = replay_blocks(&mut rx, &buf);
    assert!(rows.iter().any(|r| r.kind() == "Fineoffset-WHx080"), "{rows:?}");
    let st = rx.homeassistant_status().unwrap();
    assert_eq!(st.devices, 1, "{st:?}");
    assert_eq!(st.dropped, 0, "{st:?}");

    let mut got: Vec<(String, String)> = Vec::new();
    while let Ok(m) = seen.recv_timeout(std::time::Duration::from_secs(2)) {
        got.push(m);
        if got.iter().any(|(t, _)| t.ends_with("/state")) {
            break;
        }
    }
    let topics: Vec<&str> = got.iter().map(|(t, _)| t.as_str()).collect();
    let state = got
        .iter()
        .find(|(t, _)| t.starts_with("waveshark/ism/") && t.ends_with("/state"))
        .unwrap_or_else(|| panic!("no reading reached the broker: {topics:?}"));
    // The reading as the decoder stated it: the quantity is the name and
    // the unit travels with the discovery block.
    assert!(state.1.contains("\"temperature\""), "{}", state.1);
    assert!(
        topics
            .iter()
            .any(|t| t.starts_with("homeassistant/sensor/waveshark_ism_fineoffset_whx080_")
                && t.ends_with("/temperature/config")),
        "{topics:?}"
    );
}

/// The 5.8 GHz camera: an AKK RaceRunner with a PAL camera on it.
fn camera_fixture() -> Option<common::IqBuf> {
    let p = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../testdata/pal_camera_5865M_20000k.cs8");
    if !p.exists() {
        return None;
    }
    sources::FileSource::open(&p).ok()?.read_all().ok()
}

/// A camera through the whole receiver, which is where it was broken.
///
/// `decode::video` read this capture from the first commit, because that
/// test hands it the samples. The receiver never saw it: the source
/// detector refuses a run wider than the widest narrowband signal, so a
/// carrier megahertz wide never opened as one source, the runs inside it
/// opened instead, and a camera arrived as a packet list of sensors that
/// were not there.
/// A still that grew is published again. Keying on the picture number
/// alone left the pane holding the first line of an SSTV transmission
/// while the bus filled the rest in.
#[test]
fn a_picture_filling_in_is_published_each_time() {
    let status = Status::default();
    let frame = |lines: usize| common::VideoFrame {
        system: "SSTV",
        channel_hz: 144_500_000.0,
        label: Some("Martin 1".into()),
        width: 2,
        height: 4,
        aspect: 4.0 / 3.0,
        pixels: common::Pixels::Rgb8,
        samples: std::sync::Arc::new(vec![0u8; 2 * 4 * 3]),
        lines_seen: lines,
        sequence: 1,
        update: common::Update::Whole,
        cadence: common::Cadence::Still,
        sent_at_us: None,
        decoder: None,
    };
    status.set_video(Some(frame(1)));
    assert_eq!(status.video().map(|f| f.lines_seen), Some(1));
    status.set_video(Some(frame(3)));
    assert_eq!(status.video().map(|f| f.lines_seen), Some(3), "the picture grew");
    status.set_video(None);
    assert!(status.video().is_none(), "and it can be cleared");
}

#[test]
fn a_camera_reaches_the_video_bus_through_the_receiver() {
    let Some(buf) = camera_fixture() else {
        eprintln!("skipping: pal_camera_5865M_20000k.cs8 absent, run testdata/fetch.sh");
        return;
    };
    let mut rx = replay_receiver(&buf, None).expect("a receiver");
    let _ = replay_blocks(&mut rx, &buf);
    let bus = rx.video().expect("a video bus");
    let fed = bus.bus().channels().iter().filter(|c| c.is_fed()).count();
    let picture = bus.bus().thumbnails().next().is_some();
    assert!(picture, "no picture on the video bus; {fed} channels fed");
    // And once it has a picture the camera owns the span: it asked for
    // the band, and the auto node closed the detector out of it.
    let owned = rx
        .live_sources()
        .into_iter()
        .find(|s| s.locked_to == Some("video"))
        .expect("the camera never claimed its band");
    // The channel of the plan, which is what a camera occupies and what
    // the front end was placed on. Not the sampled span: the front end
    // reads a band-limited 10 MS/s of it and cannot claim what it was
    // never handed, so the auto node widens the claim to the band it
    // placed the decoder on.
    assert!(owned.bandwidth_hz >= 18e6, "{owned:?}");

    // And the sound that came with it. A camera puts audio on a
    // subcarrier of the same transmission, 6.5 MHz up the baseband on
    // this one, and it arrives on a voice port like any other speech: a
    // picture with no sound is half a receiver.
    let heard = rx.voices();
    let sound = heard.iter().find(|v| v.system == "analogue video").expect("no sound");
    assert!(sound.rate > 30e3 && sound.rate < 60e3, "{} Hz", sound.rate);
    // It names no party, because it has none: the sound half of a
    // transmission is not a call, and a row for it would say only that a
    // transmitter is on the air, which the picture says already.
    assert_eq!(sound.to, None);
    let peak = sound.pcm.iter().fold(0.0f32, |a: f32, s: &f32| a.max(s.abs()));
    // The capture is a quiet room, so this is small; what it may not be
    // is zero, which is what a subcarrier nobody demodulated sounds
    // like.
    assert!(peak > 1e-3, "the sound port carried silence, peak {peak}");
}

fn zigbee_fixture() -> Option<common::IqBuf> {
    let p = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../testdata/zigbee_join_ch11_2405M_8000k.cs8");
    if !p.exists() {
        return None;
    }
    sources::FileSource::open(&p).ok()?.read_all().ok()
}

#[test]
fn a_zigbee_join_reads_as_wireshark_4_4_18_read_it() {
    use decode::ieee802154::{Command, parse};
    let Some(buf) = zigbee_fixture() else {
        eprintln!("skipping: zigbee_join_ch11_2405M_8000k.cs8 absent, run testdata/fetch.sh");
        return;
    };
    let mut rx = replay_receiver(&buf, None).expect("a receiver");
    let rows: Vec<_> = replay_blocks(&mut rx, &buf)
        .into_iter()
        .filter(|r| r.packet.stack.first().is_some_and(|l| l.id == "ieee802154"))
        .collect();
    assert_eq!(rows.len(), 30, "Wireshark 4.4.18 dissected all 30, every check good");
    assert!(rows.iter().all(|r| r.packet.carrier.center_hz == 2_405_000_000), "channel 11");
    assert!(rows.iter().all(|r| r.packet.carrier.rssi_dbfs.is_finite()));
    assert!(rows.iter().all(|r| r.packet.carrier.snr_db.is_finite()));

    let count = |id: &str, kind: &str| {
        rows.iter().filter(|r| r.packet.stack.iter().any(|l| l.id == id && l.kind == kind)).count()
    };
    assert_eq!(
        [count("ieee802154", "beacon"), count("ieee802154", "data"), count("ieee802154", "ack")],
        [5, 9, 7]
    );
    assert_eq!(
        [count("zigbee", "beacon"), count("zigbee", "data"), count("zigbee", "command")],
        [5, 5, 4]
    );

    let commands: Vec<Command> =
        rows.iter().filter_map(|r| parse(r.packet.bytes())?.command).collect();
    let of = |c: Command| commands.iter().filter(|&&x| x == c).count();
    assert_eq!(
        [
            of(Command::BeaconRequest),
            of(Command::AssociationRequest),
            of(Command::AssociationResponse),
            of(Command::DataRequest),
        ],
        [2, 1, 1, 5]
    );
    let joining = rows
        .iter()
        .filter_map(|r| parse(r.packet.bytes()))
        .find(|f| f.command == Some(Command::AssociationRequest))
        .unwrap();
    assert_eq!(joining.src.to_string(), "94:A0:81:FF:FE:86:DD:48");

    let hops: Vec<String> = rows
        .iter()
        .flat_map(|r| &r.packet.stack)
        .filter(|l| l.id == "zigbee" && l.kind != "beacon")
        .filter_map(|l| l.subject.as_ref().map(|s| s.id.to_string()))
        .collect();
    assert_eq!(hops.len(), 9);
    assert!(hops.iter().all(|h| h == "A4:C1:38:01:D4:FD:FF:FF"), "{hops:?}");

    let mut networks: Vec<String> = rows
        .iter()
        .flat_map(|r| &r.packet.stack)
        .filter(|l| l.id == "zigbee" && l.kind == "beacon")
        .flat_map(|l| &l.facts)
        .filter_map(|f| match f {
            common::packet::Fact::Named(n) => Some(n.label.clone()),
            _ => None,
        })
        .collect();
    networks.sort();
    assert_eq!(
        networks,
        [
            "Zigbee PAN 41:0E:3A:D6:A1:CF:ED:BD",
            "Zigbee PAN 41:0E:3A:D6:A1:CF:ED:BD",
            "Zigbee PAN 41:0E:3A:D6:A1:CF:ED:BD",
            "Zigbee PAN 41:0E:3A:D6:A1:CF:ED:BD",
            "Zigbee PAN F1:4F:EF:31:2C:69:DE:F7",
        ]
    );
}

/// The radiosonde capture: 40 s of a Vaisala RS41 recorded near London
/// and published by SDRangel, at its own centre with the sonde 9.76 kHz
/// off it.
fn rs41_fixture() -> Option<common::IqBuf> {
    let p = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../testdata/rs41_herstmonceux_405.80024M_31.25k.cs16");
    if !p.exists() {
        return None;
    }
    sources::FileSource::open(&p).ok()?.read_all().ok()
}

/// A weather balloon read the whole way through the receiver: detector,
/// raster, remembered channel, decoder, map.
///
/// Everything about this capture is hostile to a decoder placed on a
/// source and nothing else. The transmission is 4 dB over the floor and
/// lasts 534 ms a second, so the detector opens it late and closes it
/// again, measuring a slightly different centre each time; the sonde sits
/// two thirds of the way to the edge of a 31 kHz span. What makes it work
/// is the band's 10 kHz raster turning forty measurements into one
/// channel, and the latch keeping that channel once a frame has decoded
/// on it. Take either away and this reads one frame in forty seconds.
#[test]
fn a_radiosonde_is_found_and_tracked_through_the_receiver() {
    let Some(buf) = rs41_fixture() else {
        eprintln!(
            "skipping: rs41_herstmonceux_405.80024M_31.25k.cs16 absent, run testdata/fetch.sh"
        );
        return;
    };
    let mut rx = replay_receiver(&buf, None).expect("a receiver");
    let rows = replay_blocks(&mut rx, &buf);
    let sonde = read_by(&rows, "rs41");
    assert!(
        sonde.iter().all(|r| r.bytes().len() == decode::rs41::FRAME_STD),
        "a frame was not a standard 320 byte one"
    );

    // Consecutive frame numbers, which is the sonde's own clock: one a
    // second, none missed once it is being tracked.
    // The frame counter is the sonde's bookkeeping rather than anything
    // about the world, so it stays in the bytes and is read back out of
    // them with the decoder's own parser.
    let nums: Vec<i64> = sonde
        .iter()
        .filter_map(|r| decode::rs41::parse(r.bytes()).map(|f| i64::from(f.frame_no)))
        .collect();
    assert_eq!(nums.len(), 28);
    assert_eq!(nums[0], 3409, "{nums:?}");
    assert_eq!(*nums.last().unwrap(), 3441, "{nums:?}");
    assert!(nums.windows(2).all(|w| w[1] > w[0]), "out of order: {nums:?}");
    // Once it has the channel it holds it: twenty in a row without a
    // gap, through twenty transmitter silences of 466 ms each.
    let run = nums
        .windows(2)
        .fold((1, 1), |(best, run), w| {
            let run = if w[1] == w[0] + 1 { run + 1 } else { 1 };
            (best.max(run), run)
        })
        .0;
    assert_eq!(run, 20, "longest unbroken run in {nums:?}");

    // And where it was: climbing through 10.3 km over Sussex, drifting
    // east, which is a 12 UTC Herstmonceux sounding an hour after launch.
    let last = sonde.last().unwrap();
    use common::packet::Quantity;
    let q = |x: Quantity| sensed(last, x).unwrap_or(f64::NAN);
    assert!((q(Quantity::Altitude) - 10_500.5).abs() < 1.0, "{}", q(Quantity::Altitude));
    assert!((q(Quantity::Battery) - 2.7).abs() < 0.05, "{}", q(Quantity::Battery));
    let climb = last
        .packet
        .facts()
        .find_map(|(_, f)| match f {
            common::packet::Fact::Motion(m) => m.climb_ms,
            _ => None,
        })
        .unwrap_or(f64::NAN);
    assert!((climb - 4.30).abs() < 0.05, "{climb}");
    // Which way it is going is the kind of frame it is, not a field.
    assert_eq!(last.kind(), "ascent");

    // The map reads the tracker and the tracker reads `position`, so
    // this is the test that a balloon is drawn: one track, labelled with
    // the serial, with a trail behind it. It drifted 900 m east across
    // the capture, which is the whole of the trail.
    let tracks = rx.tracks(std::time::Instant::now());
    assert_eq!(tracks.len(), 1, "{tracks:?}");
    let t = &tracks[0];
    assert_eq!(t.id, crate::tracks::TrackId::Sonde("S1720982".into()));
    assert_eq!(t.id.system(), "Radiosonde");
    let (lat, lon) = t.position.expect("no fix on the map");
    assert!((lat - 50.7898).abs() < 1e-3, "{lat}");
    assert!((lon - 0.9226).abs() < 1e-3, "{lon}");
    // One point per frame read, and a balloon at 10 km in a westerly
    // drifting east across the whole of them.
    assert_eq!(t.trail.len(), 28);
    let east = t.trail.last().unwrap().1 - t.trail[0].1;
    assert!((east - 0.0230).abs() < 1e-3, "drifted {east} degrees east");

    // And its thermometer, which is what the balloon was sent up for.
    //
    // A sonde sends a sixteenth of its factory calibration a second, so
    // there is no temperature on the first frame and there is one by the
    // end: the counts in a frame are ratios, and the front end following
    // the flight is where the pieces that turn them into degrees are
    // joined.
    let crate::tracks::Detail::Sonde {
        temperature_c, humidity_pct, altitude_m, descending, ..
    } = t.detail
    else {
        panic!("the track is not a sonde: {:?}", t.detail);
    };
    assert!(!descending);
    assert!((altitude_m - 10_500.5).abs() < 1.0, "{altitude_m}");
    // The Met Office published this ascent: Herstmonceux (03882), 00 UTC
    // on 27 December 2021, which is the hour the frames' own GPS time
    // gives. Its profile reads -59.4 C at 10475 m and -59.3 C at 10586 m,
    // against the -59.1 C read here at 10500 m. Nothing about that number
    // comes from this repository: it is a second reduction, of the same
    // balloon, from the station that launched it.
    let t_c = temperature_c.expect("no temperature");
    assert!((t_c + 59.1).abs() < 0.2, "{t_c} C against the published -59.4 at 10475 m");
    // Humidity is the empirical fit rather than Vaisala's own reduction,
    // and the published profile says 55% at 10475 m.
    let rh = humidity_pct.expect("no humidity");
    assert!((rh - 55.7).abs() < 0.5, "{rh}% against the published 55%");
}

fn ais_fixture() -> Option<common::IqBuf> {
    let p = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../testdata/ais_nijmegen_162M_768k.cu8");
    if !p.exists() {
        return None;
    }
    sources::FileSource::open(&p).ok()?.read_all().ok()
}

#[test]
fn each_barge_on_the_waal_links_to_vesselfinder() {
    let Some(buf) = ais_fixture() else {
        eprintln!("skipping: ais_nijmegen_162M_768k.cu8 absent, run testdata/fetch.sh");
        return;
    };
    let mut plan = replay_plan(&buf, false);
    plan.fronts = crate::scanners::Scanners::default()
        .fronts(crate::scanners::Span::new(buf.center.as_f64(), buf.rate.as_f64()));
    let mut rx = crate::chain::Receiver::build(&plan, crate::chain::Sinks::default()).unwrap();
    let _ = replay_blocks(&mut rx, &buf);
    let tracks = rx.tracks(std::time::Instant::now());
    let mut vessels: Vec<(u32, Option<String>)> = tracks
        .iter()
        .filter_map(|t| match t.id {
            crate::tracks::TrackId::Mmsi(m) => Some((m, t.vesselfinder())),
            _ => None,
        })
        .collect();
    vessels.sort();
    assert_eq!(tracks.len(), 9, "{tracks:?}");
    assert_eq!(
        vessels,
        [
            205581490, 211664370, 244013030, 244038327, 244650495, 244650878, 244670443, 244690403,
            244700331,
        ]
        .map(|m| (m, Some(format!("https://www.vesselfinder.com/vessels/details/{m}"))))
        .to_vec()
    );
}

#[test]
fn barges_on_the_waal_are_read_through_the_dc_block() {
    let Some(buf) = ais_fixture() else {
        eprintln!("skipping: ais_nijmegen_162M_768k.cu8 absent, run testdata/fetch.sh");
        return;
    };
    let mut plan = replay_plan(&buf, false);
    plan.dc_block = true;
    plan.fronts = crate::scanners::Scanners::default()
        .fronts(crate::scanners::Span::new(buf.center.as_f64(), buf.rate.as_f64()));
    let mut rx = crate::chain::Receiver::build(&plan, crate::chain::Sinks::default()).unwrap();
    let out = replay_blocks(&mut rx, &buf);
    assert_eq!(read_by(&out, "ais").len(), 24, "the same 24 as without the DC block");
}

/// A MeshCore advert on the European preset, which is the one LoRa
/// channel the receiver did not have: 62.5 kHz at SF8, from a node in
/// the same building, strong enough that the detector measures it at
/// up to twice its width and the front end lifts the whole span while
/// it lasts. Found, placed, read, and the node's name and position
/// come out of the signed advert.
#[test]
fn a_meshcore_advert_is_found_and_read() {
    let p = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../testdata/offair/meshcore_advert_868.9M_2048k.cu8");
    if !p.exists() {
        eprintln!("skipping: testdata/offair/meshcore_advert_868.9M_2048k.cu8 is local only");
        return;
    }
    let buf = sources::FileSource::open(&p).unwrap().read_all().unwrap();
    let mut rx = replay_receiver(&buf, None).unwrap();
    let out = replay_blocks(&mut rx, &buf);
    let rows: Vec<String> = out
        .iter()
        .map(|r| format!("{:.4} MHz {} {}", r.freq() / 1e6, r.protocol(), r.detail()))
        .collect();
    let r = out
        .iter()
        .find(|r| r.protocol() == "meshcore")
        .unwrap_or_else(|| panic!("nothing read it: {rows:?}"));
    assert!(checked(r), "{r:?}");
    let k = r.packet.keying.as_ref().expect("no keying on a LoRa packet");
    assert_eq!(k.params.spreading, Some(8), "read at {:?}", k.params);
    assert!((k.params.bandwidth_hz - 62_500.0).abs() < 1_000.0, "{:?}", k.params);
    assert!(r.detail().contains("Kieran"), "read as {}", r.detail());
    assert!((r.freq() - 869_618_000.0).abs() < 62_500.0, "read at {} Hz", r.freq());
    // The packet carries what it was: its samples and its level.
    assert!(
        r.packet.carrier.iq.as_ref().is_some_and(|q| !q.samples.is_empty()),
        "no samples on the row"
    );
    assert!(r.snr_db().is_finite() && r.rssi_dbfs().is_finite(), "no level on the row");
}
