//! Each protocol from pulse timings, not from a clean bit buffer.
//!
//! The unit tests in each module check the payload parser. This checks the
//! layer above it: that the published timings, run through this project's
//! slicers, produce the bits the parser expects, and that the registry hands
//! a burst to one protocol rather than several.
//!
//! Timings here are exact. A real detector measures every mark short by tens
//! of microseconds, which the slicer's midpoint classification absorbs; that
//! behaviour is covered against real RF in `fineoffset_capture.rs`.

use decode::bits::{checksum8, crc8, lfsr_digest8_reflect};
use decode::protocol::Value;
use decode::protocols::{
    Acurite609Txc, AcuriteTower, Bresser3Ch, Ev1527, FineOffsetWh51, GtWt02, GtWt03, LacrosseIt,
    LacrosseTx141thBv2, NexusTh, OregonV3, Rubicson, X10Rf,
};
use decode::{Protocol, Protocols};
use dsp::pulse::{Package, Pulse};

fn package(pulses: Vec<(u32, u32)>) -> Package {
    Package {
        pulses: pulses.into_iter().map(|(mark, gap)| Pulse { mark, gap }).collect(),
        snr_db: 22.0,
        rssi_dbfs: -20.0,
        start_sample: 0,
        center_hz: 0,
    }
}

fn bits_of(bytes: &[u8], n: usize) -> Vec<bool> {
    (0..n).map(|i| bytes[i / 8] & (0x80 >> (i % 8)) != 0).collect()
}

/// PPM: every mark the same, a short gap for 0 and a long one for 1.
fn ppm(bits: &[bool], mark: u32, short: u32, long: u32, reset: u32) -> Package {
    let mut p: Vec<(u32, u32)> =
        bits.iter().map(|b| (mark, if *b { long } else { short })).collect();
    p.push((mark, reset));
    package(p)
}

/// PWM: a short mark for 1 and a long one for 0, the gap being the complement.
fn pwm(bits: &[bool], short: u32, long: u32) -> Vec<(u32, u32)> {
    bits.iter().map(|b| if *b { (short, long) } else { (long, short) }).collect()
}

#[test]
fn an_acurite_609txc_burst_decodes_from_its_timings() {
    let mut f = [0x8f, 0x21, 0x2d, 0x38, 0x00];
    f[4] = checksum8(&f[..4]);
    // rtl_433: OOK_PULSE_PPM, short 1000, long 2000, reset 10000.
    let pkg = ppm(&bits_of(&f, 40), 500, 1000, 2000, 10_000);

    let r = Acurite609Txc.decode_package(&pkg).expect("decode");
    assert_eq!(r.get("id"), Some(&Value::Int(0x8f)));
    assert_eq!(r.get("temperature_c"), Some(&Value::Float(30.1)));
    assert_eq!(r.get("humidity_pct"), Some(&Value::Int(56)));
    assert_eq!(Protocols::all().decode_all(&pkg).len(), 1, "claimed by more than one protocol");
}

#[test]
fn an_acurite_tower_burst_decodes_from_its_timings() {
    // 0x1234 on channel A, 18.4 C, 55%, battery good, with parity applied.
    let mut f = [0xd2, 0x34, 0x44, 0xb7, 0x09, 0xa0, 0x00];
    f[6] = checksum8(&f[..6]);
    // The frame travels inverted, and every burst opens with a sync mark.
    let inverted: Vec<u8> = f.iter().map(|b| !b).collect();
    let mut pulses = vec![(620, 596)];
    pulses.extend(pwm(&bits_of(&inverted, 56), 220, 408));
    let pkg = package(pulses);

    let r = AcuriteTower.decode_package(&pkg).expect("decode");
    assert_eq!(r.get("id"), Some(&Value::Int(0x1234)));
    assert_eq!(r.get("channel"), Some(&Value::Text("A".into())));
    assert_eq!(r.get("temperature_c"), Some(&Value::Float(18.4)));
    assert_eq!(r.get("humidity_pct"), Some(&Value::Int(55)));
    assert_eq!(Protocols::all().decode_all(&pkg).len(), 1);
}

#[test]
fn a_lacrosse_tx141th_burst_decodes_through_its_sync_marks() {
    let mut f = [0x9c, 0x12, 0xe0, 0x2c, 0x00];
    f[4] = lfsr_digest8_reflect(&f[..4], 0x31, 0xf4);
    let inverted: Vec<u8> = f.iter().map(|b| !b).collect();
    // Four 833 us sync marks, then the 40 data bits, twice over: a real burst
    // repeats the frame and the detector rarely catches the first one whole.
    let mut pulses = Vec::new();
    for _ in 0..2 {
        pulses.extend(std::iter::repeat_n((833u32, 833u32), 4));
        pulses.extend(pwm(&bits_of(&inverted, 40), 208, 417));
    }
    let pkg = package(pulses);

    let r = LacrosseTx141thBv2.decode_package(&pkg).expect("decode");
    assert_eq!(r.get("id"), Some(&Value::Int(0x9c)));
    assert_eq!(r.get("temperature_c"), Some(&Value::Float(23.6)));
    assert_eq!(r.get("humidity_pct"), Some(&Value::Int(44)));
    assert_eq!(Protocols::all().decode_all(&pkg).len(), 1);
}

#[test]
fn a_lacrosse_it_burst_decodes_from_fsk_runs() {
    // Preamble, sync word 0x2dd4, then the payload.
    let mut f = [0x99, 0x46, 0x13, 0x39, 0x00];
    f[4] = crc8(&f[..4], 0x31, 0x00);
    let mut bits: Vec<bool> = (0..16).map(|i| i % 2 == 0).collect();
    bits.extend(bits_of(&[0x2d, 0xd4], 16));
    bits.extend(bits_of(&f, 40));

    // NRZ: runs of like bits become one mark and one gap at 55 us a bit.
    const BIT_US: u32 = 55;
    let mut pulses: Vec<(u32, u32)> = Vec::new();
    let mut i = 0;
    while i < bits.len() {
        let run = |from: usize, want: bool| {
            bits[from..].iter().take_while(|b| **b == want).count()
        };
        if !bits[i] {
            // A run of zeros before any mark cannot be expressed as a gap, so
            // fold it onto the previous pulse.
            let n = run(i, false) as u32;
            match pulses.last_mut() {
                Some(p) => p.1 += n * BIT_US,
                None => pulses.push((0, n * BIT_US)),
            }
            i += n as usize;
            continue;
        }
        let ones = run(i, true) as u32;
        i += ones as usize;
        let zeros = run(i, false) as u32;
        i += zeros as usize;
        pulses.push((ones * BIT_US, zeros * BIT_US));
    }
    // A trailing mark, so the frame's last zero bits are a gap the slicer
    // reads rather than the terminating silence it discards.
    pulses.push((BIT_US, 4000));
    let pkg = package(pulses);

    let r = LacrosseIt::tx29().decode_package(&pkg).expect("decode");
    assert_eq!(r.get("id"), Some(&Value::Int(0x25)));
    assert_eq!(r.get("temperature_c"), Some(&Value::Float(21.3)));
    assert_eq!(r.get("humidity_pct"), Some(&Value::Int(57)));
}

#[test]
fn a_nexus_burst_decodes_from_its_timings() {
    // id 0x5c, channel 2, 19.4 C, 62%, battery good.
    let f = [0x5c, 0x90, 0xc2, 0xf3, 0xe0];
    let pkg = ppm(&bits_of(&f, 36), 500, 1000, 2000, 5000);

    let r = NexusTh.decode_package(&pkg).expect("decode");
    assert_eq!(r.get("id"), Some(&Value::Int(0x5c)));
    assert_eq!(r.get("channel"), Some(&Value::Int(2)));
    assert_eq!(r.get("temperature_c"), Some(&Value::Float(19.4)));
    assert_eq!(r.get("humidity_pct"), Some(&Value::Int(62)));
    assert_eq!(r.crc_valid, None, "a constant nibble is not an integrity check");
}

#[test]
fn an_ev1527_remote_press_decodes_from_its_timings() {
    // 24 data bits plus the sync mark, inverted on the air.
    let id: u16 = 0xa13f;
    let cmd: u8 = 0x08;
    let air = [!(id >> 8) as u8, !(id as u8), !cmd];
    let mut pulses = pwm(&bits_of(&air, 24), 464, 1404);
    pulses.push((464, 10_000));
    let pkg = package(pulses);

    let r = Ev1527.decode_package(&pkg).expect("decode");
    assert_eq!(r.get("id"), Some(&Value::Int(0xa13f)));
    assert_eq!(r.get("cmd"), Some(&Value::Int(8)));
    assert_eq!(r.crc_valid, None);
}

#[test]
fn a_rubicson_burst_decodes_from_its_timings() {
    // id 0x74, channel 1, 14.9 C, battery ok, with the CRC that makes the
    // whole frame check to zero.
    let f = [0x74, 0x80, 0x95, 0xf4, 0x90];
    let pkg = ppm(&bits_of(&f, 36), 500, 1000, 2000, 4800);

    let r = Rubicson.decode_package(&pkg).expect("decode");
    assert_eq!(r.get("id"), Some(&Value::Int(0x74)));
    assert_eq!(r.get("temperature_c"), Some(&Value::Float(14.9)));
    // Nexus shares this layout and must hand the frame over rather than
    // report it as its own with a humidity read off the CRC.
    let claimed = Protocols::all().decode_all(&pkg);
    assert_eq!(claimed.len(), 1, "claimed by {claimed:?}");
    assert_eq!(claimed[0].model, "Rubicson-Temperature");
}

#[test]
fn a_bresser_3ch_burst_decodes_from_its_timings() {
    // 68.0 F is 20.0 C. Frame travels inverted, behind 750 us sync marks.
    let mut f: [u8; 5] = [0x3d, 0x26, 0x2c, 0x33, 0x00];
    f[4] = f[0].wrapping_add(f[1]).wrapping_add(f[2]).wrapping_add(f[3]);
    let inverted: Vec<u8> = f.iter().map(|b| !b).collect();
    let mut pulses = vec![(750, 750); 4];
    pulses.extend(pwm(&bits_of(&inverted, 40), 250, 500));
    let pkg = package(pulses);

    let r = Bresser3Ch.decode_package(&pkg).expect("decode");
    assert_eq!(r.get("id"), Some(&Value::Int(0x3d)));
    assert_eq!(r.get("temperature_c"), Some(&Value::Float(20.0)));
    assert_eq!(r.get("humidity_pct"), Some(&Value::Int(51)));
    assert_eq!(Protocols::all().decode_all(&pkg).len(), 1);
}

#[test]
fn a_gt_wt_02_burst_decodes_from_its_millisecond_symbols() {
    // id 0x34, channel 1, 23.7 C, 35%, with the nibble-sum checksum.
    let f = [0x34, 0x00, 0xed, 0x47, 0x60];
    let pkg = ppm(&bits_of(&f, 37), 600, 2500, 5000, 12_000);

    let r = GtWt02.decode_package(&pkg).expect("decode");
    assert_eq!(r.get("id"), Some(&Value::Int(0x34)));
    assert_eq!(r.get("temperature_c"), Some(&Value::Float(23.7)));
    assert_eq!(r.get("humidity_pct"), Some(&Value::Int(35)));
    assert_eq!(Protocols::all().decode_all(&pkg).len(), 1);
}

#[test]
fn a_gt_wt_03_burst_decodes_from_its_timings() {
    // id 0x17, channel 1, 26.1 C, 48%, then the stop bit.
    let f: [u8; 6] = [0x17, 0x30, 0x01, 0x05, 0xcb, 0x80];
    let inverted: Vec<u8> = f.iter().map(|b| !b).collect();
    let mut pulses = vec![(855, 855)];
    pulses.extend(pwm(&bits_of(&inverted, 41), 256, 625));
    let pkg = package(pulses);

    let r = GtWt03.decode_package(&pkg).expect("decode");
    assert_eq!(r.get("id"), Some(&Value::Int(0x17)));
    assert_eq!(r.get("temperature_c"), Some(&Value::Float(26.1)));
    assert_eq!(r.get("humidity_pct"), Some(&Value::Int(48)));
    assert_eq!(Protocols::all().decode_all(&pkg).len(), 1);
}

/// NRZ: runs of like bits become one mark and one gap at `bit_us` a bit.
fn nrz(bits: &[bool], bit_us: u32) -> Package {
    let mut pulses: Vec<(u32, u32)> = Vec::new();
    let mut i = 0;
    while i < bits.len() {
        let run = |from: usize, want: bool| bits[from..].iter().take_while(|b| **b == want).count();
        if !bits[i] {
            let n = run(i, false) as u32;
            match pulses.last_mut() {
                Some(p) => p.1 += n * bit_us,
                None => pulses.push((0, n * bit_us)),
            }
            i += n as usize;
            continue;
        }
        let ones = run(i, true) as u32;
        i += ones as usize;
        let zeros = run(i, false) as u32;
        i += zeros as usize;
        pulses.push((ones * bit_us, zeros * bit_us));
    }
    // A trailing mark, so the frame's last zero bits land in a gap the slicer
    // reads rather than in the terminating silence it discards.
    pulses.push((bit_us, 5000));
    package(pulses)
}

#[test]
fn a_wh51_soil_probe_decodes_from_fsk_runs() {
    let mut f = [0u8; 14];
    f[0] = 0x51;
    f[1..4].copy_from_slice(&[0x00, 0x6b, 0x58]);
    f[4] = 0x6e; // boost 3, battery 1.4 V
    f[5] = 0x7f;
    f[6] = 36; // moisture percent
    f[7] = 0xf8;
    f[8] = 0xd2;
    f[9..12].copy_from_slice(&[0xff, 0xff, 0xff]);
    f[12] = decode::bits::crc8(&f[..12], 0x31, 0x00);
    f[13] = decode::bits::checksum8(&f[..13]);

    let mut bits: Vec<bool> = (0..24).map(|i| i % 2 == 0).collect();
    bits.extend(bits_of(&[0xaa, 0x2d, 0xd4], 24));
    bits.extend(bits_of(&f, 14 * 8));
    let pkg = nrz(&bits, 58);

    let r = FineOffsetWh51.decode_package(&pkg).expect("decode");
    assert_eq!(r.get("id"), Some(&Value::Text("006b58".into())));
    assert_eq!(r.get("moisture_pct"), Some(&Value::Int(36)));
    assert_eq!(r.get("ad_raw"), Some(&Value::Int(210)));
    assert_eq!(r.get("battery_mv"), Some(&Value::Int(1400)));
}

#[test]
fn an_oregon_v3_burst_decodes_from_manchester_timings() {
    // Preamble, sync, then nibble-reversed payload, at 488 us a half symbol.
    let mut msg: [u8; 9] = [0xf8, 0x24, 0x1a, 0x30, 0x71, 0x20, 0x84, 0x00, 0x00];
    let sum: u16 = msg[..7].iter().map(|b| (b >> 4) as u16 + (b & 0x0f) as u16).sum();
    let sum = ((sum + (msg[7] >> 4) as u16) & 0xff) as u8;
    msg[7] = sum & 0x0f;
    msg[8] = sum & 0xf0;

    let mut bits: Vec<bool> = vec![true; 24];
    bits.extend([true, false, true, false]);
    for b in msg {
        let wire = b.rotate_left(4);
        for i in 0..8 {
            bits.push(wire & (0x80 >> i) != 0);
        }
    }
    // Manchester on the wire: each bit is a pair of half symbols.
    const HALF: u32 = 488;
    let mut levels: Vec<bool> = Vec::new();
    for b in &bits {
        levels.push(*b);
        levels.push(!*b);
    }
    let mut pulses: Vec<(u32, u32)> = Vec::new();
    let mut i = 0;
    while i < levels.len() {
        let ones = levels[i..].iter().take_while(|v| **v).count() as u32;
        if ones == 0 {
            i += 1;
            continue;
        }
        i += ones as usize;
        let zeros = levels[i..].iter().take_while(|v| !**v).count() as u32;
        i += zeros as usize;
        pulses.push((ones * HALF, zeros.max(1) * HALF));
    }
    let pkg = package(pulses);

    let r = OregonV3.decode_package(&pkg).expect("decode");
    assert_eq!(r.model, "Oregon-THGR810");
    assert_eq!(r.get("temperature_c"), Some(&Value::Float(21.7)));
    assert_eq!(r.get("humidity_pct"), Some(&Value::Int(48)));
}

#[test]
fn an_x10_press_decodes_from_its_timings() {
    let f = [0x60u8, !0x60u8, 0x00, 0xff];
    let pkg = ppm(&bits_of(&f, 32), 562, 562, 1687, 6000);

    let r = X10Rf.decode_package(&pkg).expect("decode");
    assert_eq!(r.get("channel"), Some(&Value::Text("A".into())));
    assert_eq!(r.get("unit"), Some(&Value::Int(1)));
    assert_eq!(r.get("state"), Some(&Value::Text("ON".into())));
    assert_eq!(Protocols::all().decode_all(&pkg).len(), 1);
}

#[test]
fn noise_is_claimed_by_nothing() {
    // Pulses at widths no protocol uses. An empty result is the right answer,
    // and a decoder that invents one from this is worse than no decoder.
    let pulses: Vec<(u32, u32)> = (0..48)
        .map(|i| (700 + (i % 5) * 37, 900 + (i % 7) * 53))
        .collect();
    let pkg = package(pulses);
    let claimed = Protocols::all().decode_all(&pkg);
    assert!(claimed.is_empty(), "noise decoded as {:?}", claimed);
}

#[test]
fn a_long_noisy_burst_does_not_manufacture_a_sensor() {
    // An 8 bit checksum passes on one window in 256, and a burst this long
    // offers hundreds of windows, so a decoder that searches without asking
    // for corroboration will report a device that is not there. This was seen
    // on air: a LaCrosse sensor claiming 43.6 C and 5% humidity, from noise.
    //
    // Deterministic pseudo-noise rather than a fixed vector, so the test
    // covers many windows rather than the one that happened to be caught.
    let mut state = 0x1234_5678u32;
    let mut rand = move || {
        state ^= state << 13;
        state ^= state >> 17;
        state ^= state << 5;
        state
    };
    for _ in 0..200 {
        let pulses: Vec<(u32, u32)> = (0..150)
            .map(|_| {
                let short = rand() & 1 == 0;
                if short { (208, 417) } else { (417, 208) }
            })
            .collect();
        let claimed = Protocols::all().decode_all(&package(pulses));
        assert!(claimed.is_empty(), "noise decoded as {claimed:?}");
    }
}
