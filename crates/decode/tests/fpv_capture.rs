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

const FIXTURE: &str = "../../testdata/fpv_pal_akk_5865M_20000k.cs8";
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

/// The line period says which standard the camera is, and it is measured
/// rather than configured: PAL and NTSC are 0.7% apart, which no
/// transmitter's timebase error reaches.
#[test]
fn the_transmission_says_which_standard_it_is() {
    let Some(base) = baseband() else { return };
    let mut sorted: Vec<f32> = base.iter().take(1 << 20).copied().collect();
    sorted.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    let thresh = (sorted[sorted.len() / 50] + sorted[sorted.len() * 13 / 100]) / 2.0;
    let mut edges = Vec::new();
    let mut low = 0usize;
    // The same quarter-microsecond mean the separator uses: without it the
    // noise on a 20 MHz baseband breaks every run.
    let taps = (0.25e-6 * RATE) as usize;
    for (i, w) in base.windows(taps).enumerate() {
        let window = w.iter().sum::<f32>() / taps as f32;
        if window < thresh {
            low += 1;
        } else {
            if ((2e-6 * RATE) as usize..=(8e-6 * RATE) as usize).contains(&low) {
                edges.push(i);
            }
            low = 0;
        }
    }
    assert!(edges.len() > 500, "only {} sync edges found", edges.len());
    let mut gaps: Vec<f64> = edges
        .windows(2)
        .map(|w| (w[1] - w[0]) as f64 / RATE)
        .collect();
    gaps.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let median = gaps[gaps.len() / 2];
    assert_eq!(
        Standard::from_line_period(median),
        Some(Standard::Pal),
        "median line period {:.3} us",
        median * 1e6
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
    assert!(
        full >= 2,
        "{full} fields of {} carried most of their lines",
        fields.len()
    );

    // A picture, not a flat grey: a room with a window has both ends of the
    // range in it, and a separator that lost lock produces neither.
    let f = fields
        .iter()
        .max_by_key(|f| f.lines_seen)
        .expect("a field");
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
    assert!(
        saturated > rgb.len() / 300,
        "only {saturated} pixels carry colour"
    );
}

/// The channel plan names the frequency the capture was taken on, and names
/// both channels that share it rather than picking one.
#[test]
fn the_channel_plan_names_where_this_came_from() {
    assert_eq!(
        decode::fpv::name_at(5_865_000_000, 1_000_000).as_deref(),
        Some("A1 or B8")
    );
}
