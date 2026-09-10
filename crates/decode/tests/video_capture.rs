//! A camera, off air, through the whole analogue chain.
//!
//! The unit tests in `dsp::video` build composite video themselves, which
//! cannot catch an assumption the builder and the reader share. This is an
//! AKK RaceRunner on 5865 MHz with a PAL camera on it, read the way the
//! receiver reads one: FM demodulate, slice sync, assemble fields, and
//! demodulate colour against the burst.
//!
//! What it asserts is what a picture is, since there is no CRC anywhere in
//! analogue video and nothing else to check against: the line period the
//! transmission itself carries, fields that locked with nearly all their
//! lines, a colour burst found on enough of them to report colour at all, and
//! a picture with structure in it rather than a uniform grey.

use common::C32;
use dsp::video::{Standard, SyncSeparator};

const FIXTURE: &str = "../../testdata/pal_camera_5865M_20000k.cs8";
const RATE: f64 = 20e6;

fn baseband() -> Option<Vec<f32>> {
    let p = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join(FIXTURE);
    if !p.exists() {
        eprintln!("skipping: {FIXTURE} absent, run testdata/fetch.sh to enable");
        return None;
    }
    let bytes = std::fs::read(&p).ok()?;
    let iq: Vec<C32> = bytes
        .chunks_exact(2)
        .map(|c| C32::new(c[0] as i8 as f32 / 128.0, c[1] as i8 as f32 / 128.0))
        .collect();
    let mut demod = dsp::FmDemod::new(RATE, 6e6);
    let mut out = Vec::new();
    demod.process(&iq, &mut out);
    Some(out)
}

/// The line rate says this is a camera, and which standard it is. Both are
/// measured rather than configured: PAL and NTSC are 0.7% apart, which no
/// transmitter's timebase error reaches, and what separates a camera from a
/// wide burst that is not one is how well its sync pulses agree with each
/// other.
#[test]
fn the_transmission_is_recognisable_as_video() {
    let Some(base) = baseband() else { return };
    let lock = dsp::video::find_lines(&base, RATE).expect("a camera");
    assert_eq!(lock.standard, dsp::video::Standard::Pal);
    // Two fields hold 625 lines and off air 410 of them clear the slicer,
    // which is what a real picture with a noisy baseband looks like: the
    // separator recovers most of the rest by then knowing where to look.
    assert!(lock.pulses > 300, "{} sync pulses", lock.pulses);
    // 0.67 off air against 0.99 on a synthesised camera: the difference is
    // noise breaking runs and the odd line lost to the vertical interval.
    // What matters is the gap to everything that is not a camera, and the
    // WiFi, BLE and impulsive-noise captures in `testdata/offair` do not
    // reach this test at all.
    assert!(
        lock.agreement > 0.5,
        "only {:.2} of the gaps agree with the line period",
        lock.agreement
    );
}

#[test]
fn the_fields_lock_and_carry_a_picture() {
    let Some(base) = baseband() else { return };
    let mut sep = SyncSeparator::new(RATE, Standard::Pal, 640).with_colour();
    let mut fields = Vec::new();
    sep.process(&base, &mut fields);
    assert!(fields.len() >= 4, "only {} fields locked", fields.len());

    let full = fields.iter().filter(|f| f.lines_seen >= 250).count();
    assert!(full >= 2, "{full} fields of {} carried most of their lines", fields.len());

    // A picture, not a flat grey: a room with a window has both ends of the
    // range in it, and a separator that lost lock produces neither.
    let f = fields.iter().max_by_key(|f| f.lines_seen).expect("a field");
    let dark = f.luma.iter().filter(|&&v| v < 40).count();
    let bright = f.luma.iter().filter(|&&v| v > 200).count();
    assert!(
        dark > f.luma.len() / 50 && bright > f.luma.len() / 50,
        "the picture is flat: {dark} dark and {bright} bright of {}",
        f.luma.len()
    );

    // And colour: the burst was found on enough lines to report it, and the
    // result is not grey, which is what a wrong subcarrier phase reference
    // would leave.
    let rgb = f.rgb.as_ref().expect("the burst was found");
    let saturated = rgb
        .chunks_exact(3)
        .filter(|p| {
            let (max, min) = (p.iter().max().unwrap(), p.iter().min().unwrap());
            max - min > 40
        })
        .count();
    assert!(saturated > rgb.len() / 300, "only {saturated} pixels carry colour");
}

/// The channel plan names the frequency the capture was taken on, and names
/// both channels that share it rather than picking one.
#[test]
fn the_channel_plan_names_where_this_came_from() {
    assert_eq!(
        decode::video_channels::name_at(5_865_000_000, 1_000_000).as_deref(),
        Some("A1 or B8")
    );
}
