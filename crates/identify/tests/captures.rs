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
