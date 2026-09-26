//! A monitor's cable, off air, and the same band with the screen switched
//! off.
//!
//! The synthetic screens in `dsp::raster` and `decode::videoleak` are built
//! by the same arithmetic that reads them, which cannot catch what a real
//! screen withholds. This pair was recorded three minutes apart at 595 MHz,
//! the fourth harmonic of a 1920x1080 screen's 148.5 MHz clock, off a
//! LimeSDR-USB at the same dial, span and gain: one with the screen's HDMI
//! output on and one with it off. The screen is the only thing that changed.

use common::{C32, SampleFormat};
use decode::display;
use decode::videoleak::Reader;

const ON: &str = "../../testdata/screen_1080p60_595M_20000k.cs16";
const OFF: &str = "../../testdata/screen_off_595M_20000k.cs16";
const RATE: f64 = 20e6;

fn capture(which: &str) -> Option<Vec<C32>> {
    let p = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join(which);
    if !p.exists() {
        eprintln!("skipping: {which} absent, run testdata/fetch.sh to enable");
        return None;
    }
    let bytes = std::fs::read(&p).ok()?;
    let mut iq = Vec::new();
    SampleFormat::Cs16.convert(&bytes, &mut iq);
    Some(iq)
}

const DIAL: f64 = 595e6;

fn read(iq: &[C32], mode: Option<&'static display::Mode>) -> Reader {
    let mut reader = Reader::new(RATE);
    reader.set_dial(DIAL);
    reader.force(mode);
    for block in iq.chunks(131_072) {
        reader.push(block);
    }
    reader
}

/// The line period is the half of the measurement a real screen gives up,
/// and this one gives it up exactly: 67.501 kHz against the mode's own
/// 67.500, which is 15 parts per million.
#[test]
fn the_line_rate_is_the_mode_s_own() {
    let Some(iq) = capture(ON) else { return };
    let mode = display::by_label("1920x1080 60 Hz").expect("the mode");
    let reader = read(&iq, Some(mode));
    let lock = reader.locked().expect("a screen");
    assert_eq!(lock.periods.lines, 1125);
    assert!(
        (lock.line_hz - mode.line_hz()).abs() < 5.0,
        "{:.1} Hz a line, the mode runs {:.1}",
        lock.line_hz,
        mode.line_hz()
    );
    // Which is the same statement as the refresh, since the operator named
    // the line count: 60.0007 Hz, seven parts in a million from 60.
    assert!((lock.frame_hz - 60.0).abs() < 0.002, "{} Hz", lock.frame_hz);
    let (held, judged) = reader.held();
    assert_eq!((held, judged), (31, 38), "frames found where the period says they are");
    // The fourth harmonic of the clock sits 993 kHz below the dial an
    // operator tuned, and finding it there is what lets the frames be
    // averaged as complex numbers rather than as magnitudes.
    assert!(reader.coherent(), "the harmonic was not found");
}

/// And the frame is the half it withholds. This screen carries its picture
/// weakly at the fourth harmonic, so the strongest line count in the whole
/// refresh range stands about as far above the other 601 as the largest of
/// 601 draws of noise does, and the receiver says so rather than painting a
/// picture that rolls.
#[test]
fn the_frame_is_not_there_to_be_found_and_is_refused() {
    let Some(iq) = capture(ON) else { return };
    let reader = read(&iq, None);
    assert!(reader.locked().is_none(), "auto locked on {:?}", reader.locked().map(|l| l.label()));
    assert_eq!(reader.frames(), 0);
}

/// The same band with the screen's output switched off holds no raster at
/// all, named mode or not. This is what says the other capture is the
/// screen rather than the receiver, the band, or the decoder's own
/// arithmetic.
#[test]
fn the_band_with_the_screen_off_holds_no_raster() {
    let Some(iq) = capture(OFF) else { return };
    let mode = display::by_label("1920x1080 60 Hz").expect("the mode");
    for named in [None, Some(mode)] {
        let reader = read(&iq, named);
        assert!(
            reader.locked().is_none(),
            "read a screen off a band with the screen switched off: {:?}",
            reader.locked().map(|l| l.label())
        );
        assert_eq!(reader.frames(), 0);
    }
}

#[test]
fn the_screen_gathers_at_its_line_rate_and_the_band_without_it_does_not() {
    let (Some(on), Some(off)) = (capture(ON), capture(OFF)) else { return };
    let line_hz = 67_501.0;
    let window = 1 << 19;
    for skip in [0, 3_000_000, 8_000_000] {
        let lit = dsp::raster::comb_contrast(&on[skip..skip + window], RATE, line_hz, 3, 3e-4);
        let dark = dsp::raster::comb_contrast(&off[skip..skip + window], RATE, line_hz, 3, 3e-4);
        let (lit, dark) = (lit.expect("a spectrum"), dark.expect("a spectrum"));
        assert!(lit > 35.0, "the screen gathered {lit:.1} dB at {skip}, 37.8 to 39.0 measured");
        assert!(dark < 4.0, "the band without it {dark:.1} dB at {skip}, 2.0 to 2.4 measured");
    }
}

#[test]
fn a_locked_screen_is_read_faster_than_it_arrives() {
    if cfg!(debug_assertions) {
        return;
    }
    let Some(iq) = capture(ON) else { return };
    let mode = display::by_label("1920x1080 60 Hz").expect("the mode");
    let t = std::time::Instant::now();
    let reader = read(&iq, Some(mode));
    let speed = iq.len() as f64 / RATE / t.elapsed().as_secs_f64();
    assert!(reader.locked().is_some(), "the screen was not locked");
    assert!(speed > 1.5, "{speed:.2} times real time, 2.6 measured");
}

fn tuned(iq: &[C32], rate: f64, offset_hz: f64, down: usize) -> Vec<C32> {
    let step = -std::f64::consts::TAU * offset_hz / rate;
    let by = C32::new(step.cos() as f32, step.sin() as f32);
    let mut turn = C32::new(1.0, 0.0);
    let mixed: Vec<C32> = iq
        .iter()
        .enumerate()
        .map(|(i, s)| {
            let out = *s * turn;
            turn *= by;
            if i % 4096 == 0 {
                turn /= turn.norm();
            }
            out
        })
        .collect();
    let mut out = Vec::new();
    dsp::resample::Rational::with_ratio(1, down).process(&mixed, &mut out);
    out
}

#[test]
fn the_busy_24_ghz_band_holds_no_screen_on_the_channels_the_table_watches() {
    let name = "ism24_busy_2431M_61440k.cs16";
    let p = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../testdata").join(name);
    let Ok(bytes) = std::fs::read(&p) else {
        eprintln!("skipping: {name} absent, run testdata/fetch.sh to enable");
        return;
    };
    let mut iq = Vec::new();
    SampleFormat::Cs16.convert(&bytes, &mut iq);
    drop(bytes);
    let (rate, centre) = (61.44e6, 2431e6);
    let mut read = Vec::new();
    for dial in [2410e6, 2415e6, 2418e6, 2420e6] {
        for down in [3, 6] {
            let span = tuned(&iq, rate, dial - centre, down);
            let mut reader = Reader::new(rate / down as f64);
            reader.set_dial(dial);
            let (mut locks, mut pictures) = (0, 0);
            for block in span.chunks(131_072) {
                let r = reader.push(block);
                locks += r.locked as usize;
                pictures += r.picture.is_some() as usize;
            }
            read.push(((dial / 1e6) as u32, down, locks, pictures));
        }
    }
    let locks: usize = read.iter().map(|r| r.2).sum();
    let pictures: usize = read.iter().map(|r| r.3).sum();
    assert_eq!(
        (read.len(), locks, pictures),
        (8, 0, 0),
        "spans, locks and pictures, where the envelope alone locked four times and painted \
         37: {read:?}"
    );
}
