//! Naming a recording with nothing but the samples and the dial reading.
//!
//! Both captures are the receiver's own corpus, so what is asserted here can
//! be compared with what the whole graph reads off the same files: the point
//! of this crate is that a program without a graph measures them the same
//! way the field does.

use common::C32;
use identify::Signal;
use sources::FileSource;

fn fixture(name: &str) -> Option<common::IqBuf> {
    let p = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../testdata").join(name);
    if !p.exists() {
        eprintln!("skipping: {name} absent, run testdata/fetch.sh");
        return None;
    }
    FileSource::open(&p).ok()?.read_all().ok()
}

/// Four seconds of 1090 MHz is named Mode S, with the aircraft dump1090 also
/// saw in it.
///
/// 76 frames, of which one aircraft names itself. dump1090-rb 1.0.15 read 40
/// *distinct* frames off this file and the decode crate's capture test
/// requires 25 of those; a repeated squitter counts once there and once per
/// transmission here, which is why the two numbers differ.
#[test]
fn a_1090_recording_is_named_mode_s() {
    let Some(buf) = fixture("adsb_1090M_2400k.cu8") else { return };
    let got = identify::identify(&buf.samples, buf.rate.as_f64(), buf.center.as_f64())
        .expect("something is in four seconds of 1090 MHz");
    assert_eq!(got.protocol, "mode_s");
    assert_eq!(got.label, "mode s");
    assert_eq!(got.frames, 76);
    assert_eq!(got.center_hz, 1_090_000_000.0);
    // The aircraft the fixture's manifest names, and the only one that gave
    // an address here.
    assert_eq!(got.identities, vec!["4b1880".to_string()]);
}

/// A sonde 9.76 kHz off the middle of a 31.25 kHz recording is found on the
/// raster and named by its serial.
///
/// 35 frames of the 40 seconds, where the whole receiver reads 28 off the
/// same file: the difference is the transmissions the detector spends finding
/// the channel, which a program handed the whole file does not pay for.
#[test]
fn a_405_mhz_recording_is_named_by_the_sonde_in_it() {
    let Some(buf) = fixture("rs41_herstmonceux_405.80024M_31.25k.cs16") else { return };
    let got = identify::identify(&buf.samples, buf.rate.as_f64(), buf.center.as_f64())
        .expect("a sonde is in this recording");
    assert_eq!(got.protocol, "rs41");
    assert_eq!(got.frames, 35);
    // Vaisala's raster, not the recorder's centre of 405.80024 MHz.
    assert_eq!(got.center_hz, 405_810_000.0);
    assert_eq!(got.identities, vec!["S1720982".to_string()]);
}

/// The band is what decides which decoders a recording is offered to.
#[test]
fn the_dial_reading_decides_what_is_tried() {
    let Some(buf) = fixture("adsb_1090M_2400k.cu8") else { return };
    let at =
        |hz| identify::candidates(buf.rate.as_f64(), hz).iter().map(|s| s.id()).collect::<Vec<_>>();
    // 1090 MHz is Mode S alone; nothing else is placed there.
    assert_eq!(at(1_090_000_000.0), ["mode_s"]);
    // The 70 cm amateur band is a busy one, and Mode S is not in it.
    let uhf = at(434_020_000.0);
    assert!(!uhf.contains(&"mode_s"), "{uhf:?}");
    assert!(uhf.contains(&"m17") && uhf.contains(&"lora"), "{uhf:?}");
}

/// Minutes of noise on each protocol's own band read as nothing.
///
/// A minute of Mode S at 2.4 MS/s and two of the sonde band at 31.25 kS/s,
/// which is 144 million and 3.75 million samples of thermal noise. Mode S
/// accepts one frame of that, a 24-bit parity passing by chance over that
/// many preamble candidates, which is what `identify::MIN_FRAMES` is for: the
/// frame arrives 48 seconds in, so a shorter run would pin nothing.
///
/// One reading of each band, not one for the threshold and another for the
/// count: `read_all` is what `identify` filters, so the frames it reports are
/// the frames `identify` was offered.
#[test]
fn minutes_of_noise_name_nothing() {
    let mut seed = 0x2545F4914F6CDD1Du64;
    let mut noise = move || {
        seed ^= seed << 13;
        seed ^= seed >> 7;
        seed ^= seed << 17;
        (seed >> 40) as f32 / 8_388_608.0 - 0.125
    };
    let bands: Vec<(f64, f64, usize, Vec<C32>)> =
        [(2_400_000.0f64, 1_090_000_000.0f64, 60.0, 1usize), (31_250.0, 405_800_240.0, 120.0, 0)]
            .into_iter()
            .map(|(rate, center, seconds, frames)| {
                let n = (rate * seconds) as usize;
                (rate, center, frames, (0..n).map(|_| C32::new(noise(), noise())).collect())
            })
            .collect();
    std::thread::scope(|scope| {
        for (rate, center, frames, iq) in &bands {
            let (rate, center, frames, iq) = (*rate, *center, *frames, iq.as_slice());
            scope.spawn(move || {
                let read = identify::read_all(iq, rate, center);
                let raw: usize = read.iter().map(|i| i.frames).sum();
                assert_eq!(raw, frames, "{center} Hz read {read:?} out of noise");
                assert!(raw < identify::MIN_FRAMES, "{raw} frames out of noise at {center} Hz");
            });
        }
    });
    // And the threshold end to end, on the band cheap enough to read twice.
    let (rate, center, _, iq) = &bands[1];
    assert_eq!(identify::identify(iq, *rate, *center), None, "{center} Hz named something");
}

/// Every signal refuses a stream it cannot read rather than guessing at it.
#[test]
fn a_stream_below_the_floor_is_refused() {
    let iq = vec![C32::new(0.0, 0.0); 4_800];
    for s in identify::all() {
        // A protocol that names no floor takes whatever it is given.
        if s.shape().min_rate_hz <= 0.0 {
            continue;
        }
        let slow = s.shape().min_rate_hz / 2.0;
        let named = identify::candidates(slow, s.default_hz());
        assert!(
            !named.iter().any(|c| c.id() == s.id()),
            "{} is offered {slow} S/s, under its own floor",
            s.id()
        );
    }
    // And Mode S refuses it directly, not only through the band table.
    assert_eq!(identify::modes::ModeS.read(&iq, 1_000_000.0, 1_090_000_000.0).count(), 0);
}

/// The busy 2.4 GHz capture is named by what is actually in it.
///
/// 61.44 MS/s at 2431 MHz covers the lower advertising channel, the 802.15.4
/// channels of that half of the band and three Wi-Fi channels, so the same
/// samples are offered to all three and each answers for itself.
#[test]
fn a_busy_24_ghz_recording_names_what_is_in_it() {
    let Some(buf) = fixture("ism24_busy_2431M_61440k.cs16") else { return };
    let named = identify::candidates(buf.rate.as_f64(), buf.center.as_f64())
        .iter()
        .map(|s| s.id())
        .collect::<Vec<_>>();
    assert_eq!(named, ["ble", "ieee802154", "wifi", "droneid", "nrf24", "lora", "elrs"]);
}

/// Every protocol reads a recording of somebody else's band as nothing.
///
/// The 1090 MHz capture offered to each protocol in turn, at its own default
/// frequency so the band table lets it through, and none of them may find
/// anything: a decoder that reads Mode S pulses as its own frames is a
/// decoder that will name every recording after itself.
#[test]
fn nobody_reads_another_protocol_s_capture() {
    let Some(buf) = fixture("adsb_1090M_2400k.cu8") else { return };
    let rate = buf.rate.as_f64();
    std::thread::scope(|scope| {
        for s in identify::all() {
            if s.id() == "mode_s" {
                continue;
            }
            let iq = buf.samples.as_slice();
            scope.spawn(move || {
                let read = s.read(iq, rate, s.default_hz());
                assert!(
                    read.count() < identify::MIN_FRAMES,
                    "{} read {} out of four seconds of 1090 MHz",
                    s.id(),
                    read.count()
                );
            });
        }
    });
}

const WELLE_IO_2_4: [(&str, &str); 29] = [
    ("10C0", "RSN Racing&Sport"),
    ("10C1", "RSN Carnival 1"),
    ("10C2", "TAB Live"),
    ("10C3", "RSN Carnival 2"),
    ("1106", "1116 SEN"),
    ("1107", "SEN2"),
    ("1108", "SEN Track"),
    ("1109", "easy music 3MP"),
    ("110A", "Rythmos"),
    ("110B", "NICHE RADIO"),
    ("110C", "SEN SYDNEY"),
    ("111A", "Nova 100"),
    ("111B", "smoothfm 91.5"),
    ("111C", "Coles Radio"),
    ("111D", "Smooth Relax"),
    ("111E", "Radio Maria"),
    ("111F", "Nova Noughties"),
    ("1137", "MMM SOFT ROCK"),
    ("1138", "MMM CLASSIC ROCK"),
    ("1139", "MMM COUNTRY"),
    ("113A", "Little Fox"),
    ("113B", "MMM HARD N HEAVY"),
    ("114C", "3RRR Digital"),
    ("114D", "LightDigital"),
    ("114E", "3MBS Fine Music"),
    ("114F", "3ZZZ Ethnic"),
    ("1150", "IRIS Melbourne"),
    ("1151", "Light89.9"),
    ("1152", "LightChristmas"),
];

#[test]
fn a_band_iii_recording_is_the_melbourne_ensemble_welle_io_2_4_read() {
    let Some(buf) = fixture("dab_melbourne_9a_202.928M_2500k.cs16") else { return };
    let (rate, center) = (buf.rate.as_f64(), buf.center.as_f64());
    let got = identify::identify(&buf.samples, rate, center).expect("an ensemble");
    assert_eq!(got.protocol, "dab");
    assert_eq!(got.frames, 30, "the ensemble and its 29 services");
    assert_eq!(got.center_hz, 202_928_000.0);

    let rows = identify::dab::DabProtocol.read(&buf.samples, rate, center).rows;
    let named = |row: &common::packet::Proto| {
        row.facts.iter().find_map(|f| match f {
            common::packet::Fact::Named(n) => Some(n.label.clone()),
            _ => None,
        })
    };
    assert_eq!(rows[0].kind, "ensemble");
    assert_eq!(named(&rows[0]).as_deref(), Some("DAB Melbourne 1"));
    let mut services: Vec<(String, String)> = rows[1..]
        .iter()
        .map(|r| {
            assert_eq!(r.kind, "audio_service");
            (r.subject.as_ref().expect("a service id").id.to_string(), named(r).expect("a label"))
        })
        .collect();
    services.sort();
    let theirs: Vec<(String, String)> =
        WELLE_IO_2_4.iter().map(|(a, b)| (a.to_string(), b.to_string())).collect();
    assert_eq!(services, theirs, "welle.io 2.4 on the same three seconds");
}

#[test]
fn a_band_iii_recording_is_the_bbc_national_multiplex() {
    let Some(buf) = fixture("dab_bbc_12b_225.648M_2048k.cs16") else { return };
    let (rate, center) = (buf.rate.as_f64(), buf.center.as_f64());
    let got = identify::identify(&buf.samples, rate, center).expect("an ensemble");
    assert_eq!(got.protocol, "dab");
    assert_eq!(got.frames, 12, "the ensemble and its 11 services");
    assert_eq!(got.center_hz, 225_648_000.0);

    let rows = identify::dab::DabProtocol.read(&buf.samples, rate, center).rows;
    let named = |row: &common::packet::Proto| {
        row.facts.iter().find_map(|f| match f {
            common::packet::Fact::Named(n) => Some(n.label.clone()),
            _ => None,
        })
    };
    let ensemble: Vec<_> = rows.iter().filter(|r| r.kind == "ensemble").collect();
    assert_eq!(ensemble.len(), 1);
    assert_eq!(named(ensemble[0]).as_deref(), Some("BBC National DAB"));
    let mut services: Vec<(String, String)> = rows
        .iter()
        .filter(|r| r.kind == "audio_service")
        .map(|r| {
            (r.subject.as_ref().expect("a service id").id.to_string(), named(r).expect("a label"))
        })
        .collect();
    services.sort();
    assert_eq!(
        services,
        [
            ("C221", "BBC Radio 1"),
            ("C222", "BBC Radio 2"),
            ("C223", "BBC Radio 3"),
            ("C224", "BBC Radio 4"),
            ("C225", "BBC Radio 5 Live"),
            ("C228", "BBC R5LiveSportX"),
            ("C22A", "BBC Radio 1Xtra"),
            ("C22B", "BBC Radio 6Music"),
            ("C22C", "BBC Radio 4Extra"),
            ("C236", "BBC AsianNetwork"),
            ("C238", "BBC WorldService"),
        ]
        .map(|(a, b)| (a.to_string(), b.to_string()))
    );
}

#[test]
fn a_pass_of_the_psat_digipeater_names_four_stations() {
    let Some(buf) = fixture("ax25_no84_145.825M_24k.cs16") else { return };
    let got = identify::identify(&buf.samples, buf.rate.as_f64(), buf.center.as_f64())
        .expect("packet in this recording");
    assert_eq!(got.protocol, "aprs");
    assert_eq!(got.frames, 4);
    assert_eq!(got.center_hz, 145_825_000.0);
    assert_eq!(got.identities, ["IU3MEY", "PSAT", "DL6AP-7", "2E0SUD"]);
}

fn nxdn_calls(name: &str) -> Option<(usize, Vec<(String, String, u16)>)> {
    let buf = fixture(name)?;
    let rows = identify::nxdn::Nxdn.read(&buf.samples, buf.rate.as_f64(), buf.center.as_f64()).rows;
    let calls = rows
        .iter()
        .filter(|r| r.kind == "vcall")
        .map(|r| {
            let (from, to) = r.parties();
            let ran = r.facts.iter().find_map(|f| match f {
                common::packet::Fact::Infrastructure(c) => c.site_code,
                _ => None,
            });
            (from.unwrap().to_string(), to.unwrap().to_string(), ran.expect("a RAN"))
        })
        .collect();
    Some((rows.len(), calls))
}

#[test]
fn an_nxdn48_call_is_the_one_dsd_fme_read() {
    let Some((rows, calls)) = nxdn_calls("nxdn48_453M_48k.cs16") else { return };
    assert_eq!(rows, 123);
    assert_eq!(calls.len(), 15, "dsd-fme at 719d970 printed 15 VCALL for this file");
    assert!(calls.iter().all(|c| *c == ("901".into(), "0".into(), 1)), "{calls:?}");
}

#[test]
fn an_nxdn96_call_is_the_one_dsd_fme_read() {
    let Some((rows, calls)) = nxdn_calls("nxdn96_453M_48k.cs16") else { return };
    assert_eq!(rows, 300);
    assert_eq!(calls.len(), 146);
    assert!(
        calls.iter().all(|c| *c == ("2".into(), "0".into(), 0)),
        "dsd-fme at 719d970 read source 2 to group 0 on RAN 0: {calls:?}"
    );
}

#[test]
fn an_inmarsat_c_tdm_reads_as_stdcdec_read_it() {
    use decode::inmarsat::stdc;
    let Some(buf) = fixture("stdc_egc_1541.45M_48k.cs16") else { return };
    let (rate, center) = (buf.rate.as_f64(), buf.center.as_f64());
    let mut chan = identify::Channel::new(rate, center, center, 6_000.0, 9_600.0).unwrap();
    let mut demod = dsp::bpsk::BpskDemod::new(chan.rate_hz, dsp::bpsk::BpskConfig::INMARSAT_C);
    let mut framer = stdc::Framer::new();
    let (mut narrow, mut soft, mut frames) = (Vec::new(), Vec::new(), Vec::new());
    for b in buf.samples.chunks(8192) {
        chan.process(b, &mut narrow);
        soft.clear();
        demod.process(&narrow, &mut soft);
        framer.process(&soft, &mut frames);
    }
    let numbers: Vec<u16> = frames.iter().map(|f| f.number).collect();
    assert_eq!(numbers, [5987, 5988, 5989, 5990], "stdcdec read 5988 to 5990, locking late");

    let mut kinds = std::collections::BTreeMap::new();
    for f in frames.iter().filter(|f| f.number >= 5988) {
        for p in stdc::packets(&f.bytes) {
            assert!(p.check_ok, "frame {} packet {:02x}", f.number, p.bytes[0]);
            *kinds.entry(p.bytes[0]).or_insert(0) += 1;
        }
    }
    let stdcdec: std::collections::BTreeMap<u8, i32> = [
        (0x6c, 3),
        (0x7d, 3),
        (0x81, 20),
        (0x92, 2),
        (0x93, 3),
        (0xa8, 12),
        (0xac, 1),
        (0xb1, 1),
        (0xb2, 1),
    ]
    .into_iter()
    .collect();
    assert_eq!(kinds, stdcdec, "stdcdec 4b4ef4a with inmarsatc cda1242, frames 5988 to 5990");

    let rows = identify::stdc::Stdc.read(&buf.samples, rate, center).rows;
    assert_eq!(rows.len(), 55, "every packet of the four frames");
}

#[test]
fn a_drm_mode_b_recording_is_the_service_dream_read() {
    let Some(buf) = fixture("drm_b_3.965M_48k.cs16") else { return };
    let (rate, center) = (buf.rate.as_f64(), buf.center.as_f64());
    let (factor, mut resample) = dsp::resample::stage(rate, identify::drm::RATE_HZ, 4096).unwrap();
    let mut chan = identify::Channel::new(
        rate,
        center,
        center,
        identify::drm::CHANNEL_WIDTH_HZ,
        rate / factor as f64,
    )
    .unwrap();
    let mut rx = decode::drm::DrmReceiver::new(dsp::drm::Mode::B);
    let (mut narrow, mut at) = (Vec::new(), Vec::new());
    for b in buf.samples.chunks(8192) {
        chan.process(b, &mut narrow);
        match resample.as_mut() {
            Some(r) => {
                at.clear();
                r.process(&narrow, &mut at);
                rx.push(&at);
            }
            None => {
                rx.push(&narrow);
            }
        }
    }
    assert_eq!((rx.stats.frames, rx.stats.fac_ok), (71, 71));
    assert_eq!(rx.stats.sdc_ok, 23, "every super frame's 16-QAM description channel");
    let m = rx.multiplex();
    assert_eq!(m.services.len(), 1);
    let s = m.services[0];
    assert_eq!(
        (s.id, s.language, s.programme, s.audio),
        (0x3EE, decode::drm::Language::German, decode::dab::ProgrammeType::Science, true),
        "Dream at e3e104c read service 3EE, German, science"
    );
    assert_eq!(m.label(0), Some("Spark"), "Dream at e3e104c read the label Spark");
}

#[test]
fn a_zigbee_join_reads_as_wireshark_4_4_18_read_it() {
    use decode::ieee802154::{Command, layers, parse};
    use dsp::oqpsk::{OQPSK_2450, OqpskConfig, OqpskDetector, channels_2450};
    let Some(buf) = fixture("zigbee_join_ch11_2405M_8000k.cs8") else { return };
    let (rate, center) = (buf.rate.as_f64(), buf.center.as_f64());
    let mut det =
        OqpskDetector::new(rate, center, OQPSK_2450, &channels_2450(), OqpskConfig::default());
    let mut frames = Vec::new();
    for b in buf.samples.chunks(65_536) {
        det.process(b, &mut frames);
    }
    assert_eq!(frames.len(), 31, "Wireshark 4.4.18 dissected all 31, every check good");
    assert!(frames.iter().all(|f| f.channel == 11));
    let stacks: Vec<Vec<common::packet::Proto>> =
        frames.iter().map(|f| layers(&f.psdu, common::Hz(2_405_000_000))).collect();
    let count = |id: &str, kind: &str| {
        stacks.iter().filter(|s| s.iter().any(|l| l.id == id && l.kind == kind)).count()
    };
    assert_eq!(
        [count("ieee802154", "beacon"), count("ieee802154", "data"), count("ieee802154", "ack")],
        [6, 9, 7]
    );
    assert_eq!(
        [count("zigbee", "beacon"), count("zigbee", "data"), count("zigbee", "command")],
        [6, 5, 4]
    );
    let commands: Vec<Command> = frames.iter().filter_map(|f| parse(&f.psdu)?.command).collect();
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
    let joining = frames
        .iter()
        .filter_map(|f| parse(&f.psdu))
        .find(|f| f.command == Some(Command::AssociationRequest))
        .unwrap();
    assert_eq!(joining.src.to_string(), "94:A0:81:FF:FE:86:DD:48");
    let hops: Vec<String> = stacks
        .iter()
        .flatten()
        .filter(|l| l.id == "zigbee" && l.kind != "beacon")
        .filter_map(|l| l.subject.as_ref().map(|s| s.id.to_string()))
        .collect();
    assert_eq!(hops.len(), 9);
    assert!(hops.iter().all(|h| h == "A4:C1:38:01:D4:FD:FF:FF"), "{hops:?}");
    let mut networks: Vec<String> = stacks
        .iter()
        .flatten()
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
            "Zigbee PAN F1:4F:EF:31:2C:69:DE:F7",
        ]
    );
}

#[test]
fn an_aero_10500_channel_reads_as_jaero_read_it() {
    use decode::inmarsat::{OQPSK, aero};
    let Some(buf) = fixture("aero_oqpsk_1546M_48k.cs16") else { return };
    let (rate, center) = (buf.rate.as_f64(), buf.center.as_f64());
    let (factor, mut resample) = dsp::resample::stage(rate, OQPSK.rate(), 4096).unwrap();
    let mut chan = identify::Channel::new(
        rate,
        center,
        center,
        identify::aero::WIDE_CHANNEL_HZ,
        rate / factor as f64,
    )
    .unwrap();
    let mut demod = dsp::qpsk::QpskDemod::new(OQPSK);
    let mut framer = aero::Framer::new(aero::Rate::P10500);
    let mut assembler = aero::Assembler::new();
    let (mut narrow, mut at_rate, mut symbols, mut soft) =
        (Vec::new(), Vec::new(), Vec::new(), Vec::new());
    let (mut frames, mut messages) = (Vec::new(), Vec::new());
    for b in buf.samples.chunks(8192) {
        chan.process(b, &mut narrow);
        at_rate.clear();
        resample.as_mut().unwrap().process(&narrow, &mut at_rate);
        symbols.clear();
        demod.process(&at_rate, &mut symbols);
        soft.clear();
        soft.extend(symbols.iter().flat_map(|s| [s.re, s.im]));
        let at = frames.len();
        framer.process_soft(&soft, &mut frames);
        for f in &frames[at..] {
            for su in f.sus.iter().filter(|s| s.crc_ok) {
                messages.extend(assembler.update(su.data()));
            }
        }
    }
    let units: usize = frames.iter().map(|f| f.sus.len()).sum();
    let checked: usize = frames.iter().map(|f| f.sus.iter().filter(|s| s.crc_ok).count()).sum();
    assert_eq!(
        (frames.len(), units, checked),
        (86, 2236, 2236),
        "frames, units, units that checked"
    );

    let read: Vec<(String, char, usize)> = messages
        .iter()
        .map(|m| {
            let block = aero::acars_block(&m.bytes).expect("an ACARS block whose check agrees");
            let acars = decode::acars::parse(&block).expect("an ACARS message");
            (acars.registration, acars.block_id, m.bytes.len())
        })
        .collect();
    let sent = |reg: &str, id: char, len: usize| (reg.to_string(), id, len);
    assert_eq!(
        read,
        [
            sent("N531QS", 'R', 19),
            sent("N642UA", 'R', 240),
            sent("N642UA", 'S', 240),
            sent("N531QS", 'S', 19),
            sent("N642UA", 'T', 235),
            sent("N531QS", 'T', 19),
            sent("N531QS", 'U', 19),
            sent("N701WH", 'F', 19),
            sent("N531QS", 'V', 19),
        ],
        "inmarsat-sniffer c5e767e read the same bytes for all but N642UA's blocks S and T"
    );
    assert_eq!(
        messages[0].bytes,
        [
            0xFF, 0xFF, 0x01, 0x32, 0xAE, 0xCE, 0xB5, 0xB3, 0x31, 0x51, 0xD3, 0xB9, 0xDF, 0x7F,
            0x52, 0x83, 0x56, 0x31, 0x7F
        ],
        "as inmarsat-sniffer c5e767e read it through JAERO's OqpskDemodulator and AeroL"
    );

    let got = identify::identify(&buf.samples, rate, center).expect("an Aero channel");
    assert_eq!(got.protocol, "aero");
    assert_eq!(got.frames, 2245, "every unit that checked and the nine messages");
    assert_eq!(got.center_hz, 1_546_000_000.0);
    assert_eq!(got.identities, ["a6b593", "N531QS", "a86e6c", "N642UA", "a95a41", "N701WH"]);
}

#[test]
fn a_band_28_recording_is_named_lte_with_both_cells_and_the_carriers_their_sib5_and_sib7_name() {
    let Some(buf) = fixture("offair/ofdm_lte_762M_12000k.cs16") else { return };
    let got = identify::identify(&buf.samples, buf.rate.as_f64(), buf.center.as_f64())
        .expect("an LTE carrier is in this recording");
    assert_eq!(got.protocol, "lte");
    assert_eq!(got.center_hz, 763_000_000.0);
    assert_eq!(got.identities, ["272-03-40111-11350600", "272-03-40111-11350601"]);
    let kinds: Vec<&str> = got.rows.iter().map(|r| r.kind).collect();
    assert_eq!(
        kinds,
        [
            "mib",
            "mib",
            "system_information",
            "system_information",
            "system_information",
            "network",
            "network",
            "network",
            "network"
        ]
    );
    let carriers = |r: &common::packet::Proto| -> Vec<u64> {
        r.facts
            .iter()
            .filter_map(|f| match f {
                common::packet::Fact::Infrastructure(c) => c.carrier_hz,
                _ => None,
            })
            .collect()
    };
    let lte = [2_160_000_000, 1_872_500_000, 796_000_000];
    assert_eq!((carriers(&got.rows[5]), carriers(&got.rows[6])), (lte.to_vec(), lte.to_vec()));
    let gsm: Vec<usize> = got.rows[7..].iter().map(|r| carriers(r).len()).collect();
    assert_eq!(
        gsm,
        [23, 21],
        "the E-GSM 900 carriers each sector's SIB7 names, as asn1tools reads it"
    );
    assert!(
        got.rows[7..].iter().flat_map(carriers).all(|hz| (925_000_000..=935_000_000).contains(&hz))
    );
}

fn lte_cell(buf: &common::IqBuf, channel_hz: f64) -> Vec<dsp::lte::Heard> {
    let mut rx = dsp::lte::Receiver::new(buf.rate.as_f64(), buf.center.as_f64(), channel_hz)
        .expect("an LTE rate");
    let mut heard = Vec::new();
    for b in buf.samples.chunks(131_072) {
        let from = heard.len();
        rx.push(b, &mut heard);
        for h in &heard[from..] {
            identify::lte::follow(&mut rx, h);
        }
    }
    heard
}

fn system_information(heard: &[dsp::lte::Heard]) -> Vec<String> {
    heard
        .iter()
        .filter_map(|h| match h {
            dsp::lte::Heard::SystemInformation { bytes, .. } => {
                Some(bytes.iter().map(|b| format!("{b:02x}")).collect())
            }
            _ => None,
        })
        .collect()
}

const ESTEVEZ_PCI_380: [&str; 4] = [
    "4848500301164607407819311081044c23cb52000000",
    "00830ac3230dfc545826e0f2dc5a0000c0361dc62d540880c029b9858389180078006002a106ff31628df2780000000000",
    "000d2402ee8c8a5ac8080d0480cb219211590101a090032232295aa0203412006a86462a34040682407460c8e6468080d0000000000000000000000000",
    "0011052fd8a022ca334ec940459466",
];

#[test]
fn the_vodafone_cell_in_estevezs_b20_recording_gives_the_system_information_his_decoder_read() {
    let Some(buf) = fixture("lte_b20_madrid_806M_30720k.cs8") else { return };
    let heard = lte_cell(&buf, 806e6);
    let pci: Vec<u16> = heard
        .iter()
        .filter_map(|h| match h {
            dsp::lte::Heard::Mib { pci, .. } => Some(*pci),
            _ => None,
        })
        .collect();
    assert_eq!(pci, [380]);
    assert_eq!(
        system_information(&heard),
        ESTEVEZ_PCI_380,
        "SIB1, SIB2, SIB5 and SIB6 as lte-downlink.pcap in daniestevez/jupyter_notebooks has them"
    );
}

#[test]
fn the_three_b20_operators_in_estevezs_recording_each_name_their_cell_and_neighbours() {
    let Some(buf) = fixture("lte_b20_madrid_806M_30720k.cs8") else { return };
    let mut named = Vec::new();
    for hz in [796e6, 806e6, 816e6] {
        let rows: Vec<common::packet::Proto> =
            lte_cell(&buf, hz).iter().flat_map(identify::lte::rows).collect();
        let cell =
            rows.iter().find_map(|r| r.subject.as_ref().map(|e| e.id.to_string())).expect("a SIB1");
        let network: usize = rows.iter().filter(|r| r.kind == "network").count();
        named.push((hz as u64, cell, network));
    }
    assert_eq!(
        named,
        [
            (796_000_000, "214-03-1371-73430136".to_string(), 2),
            (806_000_000, "214-01-278-73430023".to_string(), 2),
            (816_000_000, "214-07-28673-73816853".to_string(), 3),
        ]
    );
}

#[test]
fn a_dwd_broadcast_is_read_as_the_baltic_forecast() {
    let Some(buf) = fixture("rtty_dwd_11.039M_2k.cs16") else { return };
    let got = identify::identify(&buf.samples, buf.rate.as_f64(), buf.center.as_f64())
        .expect("a teleprinter");
    assert_eq!(got.protocol, "rtty");
    assert_eq!(got.frames, 6);
    let text: String = got
        .rows
        .iter()
        .flat_map(|r| &r.facts)
        .filter_map(|f| match f {
            common::packet::Fact::Message(m) => Some(m.text.clone()),
            _ => None,
        })
        .collect::<Vec<_>>()
        .join("\n");
    for line in [
        "CQ CQ CQ DE DDH47 DDH9 DDH8",
        "FREQUENCIES   147.3 KHZ   11039 KHZ   14467.3 KHZ",
        "ZCZC 964",
        "FEBQ52 EDZW 280600",
        "MITTELFRIST - SEEWETTERBERICHT FUER DIE OSTSEE",
        "HERAUSGEGEBEN VOM SEEWETTERDIENST HAMBURG",
    ] {
        assert!(text.contains(line), "{line:?} missing from {text}");
    }
}

#[test]
fn two_minutes_of_a_noaa_pass_are_one_picture() {
    use identify::Signal;
    let Some(buf) = fixture("apt_noaa18_137.912M_62.5k.cs16") else { return };
    let rate = buf.rate.as_f64();
    assert_eq!(identify::apt::Apt.read(&buf.samples, rate, 137_912_500.0).pictures, 1);
    let read = identify::read_all(&buf.samples, rate, buf.center.as_f64());
    let named: Vec<(&str, usize)> =
        read.iter().filter(|r| r.frames > 0).map(|r| (r.protocol, r.frames)).collect();
    assert_eq!(named, [("apt", 1)]);
}

#[test]
fn niton_navtex_is_read_as_its_ten_bulletins() {
    let Some(buf) = fixture("navtex_niton_0.518M_1.953k.cs16") else { return };
    let got =
        identify::identify(&buf.samples, buf.rate.as_f64(), buf.center.as_f64()).expect("NAVTEX");
    assert_eq!(got.protocol, "navtex");
    assert_eq!(got.frames, 10);
    assert_eq!(
        got.identities,
        ["EA39", "EL09", "EA34", "EA33", "EA31", "EA30", "EA25", "EA22", "EA14", "EA09"]
    );
    let text: Vec<String> = got
        .rows
        .iter()
        .flat_map(|r| &r.facts)
        .filter_map(|f| match f {
            common::packet::Fact::Message(m) => Some(m.text.clone()),
            _ => None,
        })
        .collect();
    assert!(text[0].contains("NASH POINT LIGHT, NORMAL CONDITIONS RESTORED."), "{}", text[0]);
    assert!(
        text[1].starts_with("FOST SUBFACTS AND GUNFACTS WARNING (ALL TIMES UTC)."),
        "{}",
        text[1]
    );
}

#[test]
fn a_fusion_over_through_gb3xp_names_g1rce() {
    let Some(buf) = fixture("ysf_145.6875M_74.999k.cs16") else { return };
    let got = identify::identify(&buf.samples, buf.rate.as_f64(), buf.center.as_f64())
        .expect("System Fusion");
    assert_eq!(got.protocol, "ysf");
    assert_eq!(got.frames, 70);
    assert_eq!(got.identities, ["G1RCE"]);
}

#[test]
fn ockham_and_biggin_put_the_receiver_on_crossing_radials() {
    let Some(buf) = fixture("vor_ockham_biggin_115.2M_384k.cs16") else { return };
    let read = |hz: f64| {
        let rows = identify::vor::read_at(&buf.samples, buf.rate.as_f64(), buf.center.as_f64(), hz);
        let radials: Vec<f64> = rows
            .iter()
            .flat_map(|r| &r.facts)
            .filter_map(|f| match f {
                common::packet::Fact::Sensed(x) => Some(x.value),
                _ => None,
            })
            .collect();
        let named: Vec<String> =
            rows.iter().filter_map(|r| r.subject.as_ref().map(|e| e.id.to_string())).collect();
        (radials, named)
    };
    let ((ockham, ock), (biggin, big)) = (read(115_300_000.0), read(115_100_000.0));
    assert_eq!((ockham.len(), biggin.len()), (2, 2));
    assert!(ockham.iter().all(|r| (88.0..=94.0).contains(r)), "Ockham {ockham:?}");
    assert!(biggin.iter().all(|r| (248.0..=264.0).contains(r)), "Biggin {biggin:?}");
    assert_eq!(ock, ["OCK"]);
    assert_eq!(big, Vec::<String>::new());
}

#[test]
fn msf_dcf77_and_tdf_agree_on_the_minute() {
    let Some(buf) = fixture("clocks_msf_dcf77_tdf_0.11M_192k.cs16") else { return };
    let read = identify::read_all(&buf.samples, buf.rate.as_f64(), buf.center.as_f64());
    let clock = |id: &str| -> Vec<f64> {
        read.iter()
            .filter(|r| r.protocol == id)
            .flat_map(|r| &r.rows)
            .flat_map(|r| &r.facts)
            .filter_map(|f| match f {
                common::packet::Fact::Sensed(x) => Some(x.value),
                _ => None,
            })
            .collect()
    };
    assert_eq!(clock("msf"), [1_638_129_540.0]);
    assert_eq!(clock("dcf77"), [1_638_129_540.0]);
    assert_eq!(clock("tdf"), [1_638_129_540.0]);
    assert_eq!(common::packet::clock_label(1_638_129_540), "2021-11-28 19:59 UTC");
}
