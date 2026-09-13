//! Every SSTV mode, against transmissions whose source picture is known.
//!
//! One pySSTV recording per mode, all of the same picture: the colour bars
//! across the top half, a red-to-green ramp at constant blue across the
//! bottom. What should come out is therefore not a judgement, and what is
//! asserted is which colour each bar reads as, which line count came back,
//! and that the ramp rises.
//!
//! Bars rather than a photograph, because what differs between the modes is
//! the timing table and nothing else, and a wrong figure there gives a
//! picture that still looks like a picture: sheared by a fraction of a line,
//! or with the colour a channel out of step. On a photograph that reads as a
//! poor signal. On bars, cyan turns green.
//!
//! Two real bugs are pinned here. Robot 36's two colour differences were
//! swapped, and its even lines borrowed the missing half from the line before
//! rather than the line after; both survived an off-air photograph looking
//! merely dull.
//!
//! Robot 72 is not covered: pySSTV has no encoder for it, so the only thing
//! behind that mode is the published timing table.

use decode::sstv;

/// The bars, in the order they are sent.
const BARS: [(&str, (u8, u8, u8)); 7] = [
    ("white", (255, 255, 255)),
    ("yellow", (255, 255, 0)),
    ("cyan", (0, 255, 255)),
    ("green", (0, 255, 0)),
    ("magenta", (255, 0, 255)),
    ("red", (255, 0, 0)),
    ("blue", (0, 0, 255)),
];

/// The fixture, the mode it should be read as, and how many lines of it come
/// back. The counts are pinned rather than checked for "most of it": a mode
/// whose last line needs a neighbour it never gets is one short, and that is
/// a fact about the mode rather than a fault.
const MODES: [(&str, &str, usize); 5] = [
    ("sstv_martin2_bars_44100.wav", "Martin 2", 256),
    ("sstv_scottie1_bars_44100.wav", "Scottie 1", 255),
    ("sstv_scottie2_bars_44100.wav", "Scottie 2", 255),
    ("sstv_scottiedx_bars_44100.wav", "Scottie DX", 256),
    ("sstv_robot36_bars_44100.wav", "Robot 36", 239),
];

fn picture(name: &str) -> Option<sstv::Picture> {
    let p = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../testdata").join(name);
    if !p.exists() {
        eprintln!("skipping: {name} absent, run testdata/fetch.sh to enable");
        return None;
    }
    let raw = std::fs::read(&p).ok()?;
    let rate = u32::from_le_bytes([raw[24], raw[25], raw[26], raw[27]]) as f64;
    let audio: Vec<f32> = raw[44..]
        .chunks_exact(2)
        .map(|c| i16::from_le_bytes([c[0], c[1]]) as f32 / 32768.0)
        .collect();
    sstv::decode(&audio, rate)
}

fn pixel(p: &sstv::Picture, x: usize, y: usize) -> (u8, u8, u8) {
    let at = (y * p.width + x) * 3;
    (p.rgb[at], p.rgb[at + 1], p.rgb[at + 2])
}

/// Which bar a pixel is, rather than how many counts it is off by. A colour
/// difference is sent at half the picture's resolution and read through a
/// window several pixels wide, so a saturated bar lands tens of counts from
/// where it started; what must not happen is it landing on another bar.
fn nearest_bar(got: (u8, u8, u8)) -> usize {
    BARS.iter()
        .enumerate()
        .min_by_key(|(_, (_, c))| {
            let d = |a: u8, b: u8| (a as i64 - b as i64).pow(2);
            d(got.0, c.0) + d(got.1, c.1) + d(got.2, c.2)
        })
        .map(|(k, _)| k)
        .unwrap_or(usize::MAX)
}

#[test]
fn every_mode_reads_the_bars_in_the_order_they_were_sent() {
    for (file, mode, lines) in MODES {
        let Some(p) = picture(file) else { continue };
        assert_eq!(p.mode.name, mode, "{file}: the VIS code named the wrong mode");
        assert_eq!(p.lines, lines, "{mode}: lines decoded");

        // An even line and an odd one, because Robot 36 carries a different
        // half of the colour on each and a bug in one is invisible in the
        // other.
        for y in [30usize, 31] {
            for (i, (name, _)) in BARS.iter().enumerate() {
                let x = i * p.width / 7 + p.width / 14;
                let got = pixel(&p, x, y);
                assert_eq!(nearest_bar(got), i, "{mode} line {y}: the {name} bar read as {got:?}");
            }
        }
    }
}

/// The ramp says the tone scale is right and the lines are in step: a picture
/// built from the wrong neighbour shows it as a stair rather than a slope,
/// and one read at the wrong rate shows it sheared.
#[test]
fn every_mode_reads_the_ramp_as_a_ramp() {
    for (file, mode, _) in MODES {
        let Some(p) = picture(file) else { continue };
        let y = p.height * 3 / 4;
        let left = pixel(&p, p.width / 16, y);
        let mid = pixel(&p, p.width / 2, y);
        let right = pixel(&p, p.width * 15 / 16, y);
        assert!(left.0 < mid.0 && mid.0 < right.0, "{mode}: red rises {left:?} {mid:?} {right:?}");
        assert!(
            left.1 > mid.1 && mid.1 > right.1,
            "{mode}: green falls {left:?} {mid:?} {right:?}"
        );
        for (at, px) in [("left", left), ("middle", mid), ("right", right)] {
            assert!((px.2 as i32 - 128).abs() < 60, "{mode}: blue is constant, {at} read {}", px.2);
        }
    }
}
