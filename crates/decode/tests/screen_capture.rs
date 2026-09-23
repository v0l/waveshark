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

fn read(iq: &[C32], mode: Option<&'static display::Mode>) -> Reader {
    let mut reader = Reader::new(RATE);
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
    assert_eq!((held, judged), (30, 31), "frames found where the period says they are");
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
