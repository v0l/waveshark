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
