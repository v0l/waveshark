//! SSTV against the picture an independent decoder gets out of the same
//! recording.
//!
//! `testdata/sstv_martin1_44100.wav` is the Martin 1 example shipped with
//! colaclanth's Python `sstv`, and that project ships the picture its decoder
//! produces beside it. The expectation below is a grid of block means taken
//! off that PNG, not off this code: if this decoder drifts a line, samples
//! the wrong window or gets the channel order wrong, the blocks move.
//!
//! A picture is not a packet, so an exact match is the wrong test. Two
//! decoders sampling the same tone with slightly different windows differ by
//! a count or two per pixel, and a half-pixel timing difference shows at an
//! edge. Block means over an eighth of the picture average that away while
//! still catching anything structural: a shifted line, a swapped channel, a
//! mode read at the wrong speed.
//!
//! What the two do not share is the contrast. The reference places its peak
//! between the bins by a parabola through the log magnitudes, which under a
//! Hann window reads back about 0.72 of the distance from the bin centre, so
//! its picture is squeezed towards mid grey; `dsp::tone` inverts the Hann
//! kernel exactly and reads the tone that was sent. The blocks are therefore
//! compared through the straight line between the two decoders rather than
//! count for count, which still moves if a line drifts or a channel swaps.
//!
//! What else differs is the edge columns: the sampling window here is clipped
//! to the scan it belongs to, where the reference lets it run past the end,
//! which is what puts a green stripe down the right of a Robot picture.

use decode::sstv;

const FIXTURE: &str = "../../testdata/sstv_martin1_44100.wav";

/// Mean red, green and blue over each eighth of the reference picture,
/// `examples/m1.png` from the Python decoder at the commit the manifest pins.
#[rustfmt::skip]
const REFERENCE: [[(f32, f32, f32); 8]; 8] = [
    [(170.4,145.0,111.3), (125.1,160.4,128.3), (99.3,165.9,131.4), (137.1,141.2,137.0), (170.7,108.8,136.4), (136.9,104.6,135.4), (103.7,103.3,131.4), (128.5,128.6,114.3)],
    [(133.8,113.7,113.9), (128.9,129.0,129.1), (141.0,141.0,141.1), (137.5,147.8,144.9), (141.8,149.1,147.8), (138.5,138.6,138.5), (129.2,129.2,129.1), (126.3,126.4,113.4)],
    [(157.8,103.8,103.7), (145.4,145.4,145.6), (148.1,160.0,165.1), (136.7,131.8,126.1), (146.9,177.0,187.9), (134.7,137.5,138.0), (128.5,128.4,128.5), (137.6,138.0,138.0)],
    [(118.2,111.0,111.1), (127.7,127.7,127.6), (121.3,130.1,137.5), (131.6,98.4,66.6), (117.4,106.8,90.4), (123.4,120.4,110.2), (123.7,123.5,123.7), (115.5,115.5,115.6)],
    [(110.5,109.2,141.0), (116.7,116.9,116.8), (121.0,92.7,88.1), (152.6,108.1,58.1), (97.2,110.4,60.4), (120.8,132.4,113.9), (130.1,130.0,130.0), (137.1,137.2,137.3)],
    [(110.2,106.0,144.4), (117.8,117.9,117.9), (124.1,121.5,121.7), (137.9,117.0,84.5), (160.3,141.3,80.0), (133.8,133.1,129.3), (125.9,125.7,125.8), (119.0,108.2,119.1)],
    [(114.2,109.6,144.1), (128.0,128.0,128.0), (129.5,129.6,129.5), (140.1,140.4,140.2), (141.5,141.7,141.5), (129.1,129.2,129.1), (127.9,127.8,128.1), (134.2,131.2,107.2)],
    [(55.0,77.1,54.9), (51.4,85.7,51.1), (51.2,84.2,51.0), (58.2,80.7,58.1), (56.6,78.7,56.5), (51.1,86.2,50.9), (51.6,84.1,51.3), (55.5,77.5,55.2)],
];

fn audio() -> Option<(Vec<f32>, f64)> {
    let p = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join(FIXTURE);
    if !p.exists() {
        eprintln!("skipping: {FIXTURE} absent, run testdata/fetch.sh to enable");
        return None;
    }
    let raw = std::fs::read(&p).ok()?;
    let rate = u32::from_le_bytes([raw[24], raw[25], raw[26], raw[27]]) as f64;
    let mut at = 12;
    let body = loop {
        let len = u32::from_le_bytes([raw[at + 4], raw[at + 5], raw[at + 6], raw[at + 7]]) as usize;
        if &raw[at..at + 4] == b"data" {
            break &raw[at + 8..at + 8 + len];
        }
        at += 8 + len;
    };
    let samples =
        body.chunks_exact(2).map(|c| i16::from_le_bytes([c[0], c[1]]) as f32 / 32768.0).collect();
    Some((samples, rate))
}

/// Mean of each channel over each eighth of the picture.
fn blocks(p: &sstv::Picture) -> [[(f32, f32, f32); 8]; 8] {
    let mut out = [[(0.0, 0.0, 0.0); 8]; 8];
    for (by, row) in out.iter_mut().enumerate() {
        for (bx, cell) in row.iter_mut().enumerate() {
            let (mut r, mut g, mut b, mut n) = (0.0f64, 0.0f64, 0.0f64, 0u32);
            for y in by * p.height / 8..(by + 1) * p.height / 8 {
                for x in bx * p.width / 8..(bx + 1) * p.width / 8 {
                    let at = (y * p.width + x) * 3;
                    r += p.rgb[at] as f64;
                    g += p.rgb[at + 1] as f64;
                    b += p.rgb[at + 2] as f64;
                    n += 1;
                }
            }
            let n = n as f64;
            *cell = ((r / n) as f32, (g / n) as f32, (b / n) as f32);
        }
    }
    out
}

#[test]
fn the_recording_decodes_as_the_martin_1_picture_the_reference_gets() {
    let Some((audio, rate)) = audio() else { return };
    let p = sstv::decode(&audio, rate).expect("a calibration header and a VIS code");

    assert_eq!(p.mode.name, "Martin 1", "the VIS code names the mode");
    assert_eq!((p.width, p.height), (320, 256));
    // Every line, not most of them: the recording holds a whole picture, so a
    // decoder that runs out early has lost its clock rather than its audio.
    assert_eq!(p.lines, 256, "lines decoded");

    let got = blocks(&p);
    let mut pairs = Vec::new();
    for (a, b) in got.iter().zip(REFERENCE.iter()) {
        for (g, r) in a.iter().zip(b.iter()) {
            pairs.push((r.0, g.0));
            pairs.push((r.1, g.1));
            pairs.push((r.2, g.2));
        }
    }
    assert_eq!(pairs.len(), 192, "sixty-four blocks of three channels");

    let n = pairs.len() as f32;
    let mean_r = pairs.iter().map(|p| p.0).sum::<f32>() / n;
    let mean_g = pairs.iter().map(|p| p.1).sum::<f32>() / n;
    let cov = pairs.iter().map(|p| (p.0 - mean_r) * (p.1 - mean_g)).sum::<f32>();
    let var = pairs.iter().map(|p| (p.0 - mean_r).powi(2)).sum::<f32>();
    let gain = cov / var;
    let lift = mean_g - gain * mean_r;
    // Measured over the 192 block means: a gain of 1.390 and a lift of
    // -46.7 counts, which is the reference's own 0.72 compression undone. A
    // gain of 1.0 would mean this decoder had taken the compression back on.
    assert!((1.34..=1.44).contains(&gain), "contrast against the reference is {gain:.3}");

    let mut worst = 0.0f32;
    for (y, (a, b)) in got.iter().zip(REFERENCE.iter()).enumerate() {
        for (x, (g, r)) in a.iter().zip(b.iter()).enumerate() {
            for (c, (gv, rv)) in [(g.0, r.0), (g.1, r.1), (g.2, r.2)].iter().enumerate() {
                let want = gain * rv + lift;
                let d = (gv - want).abs();
                worst = worst.max(d);
                assert!(d < 3.0, "block ({x},{y}) channel {c}: {gv:.1} against {want:.1}");
            }
        }
    }
    // 2.2 counts measured; the blocks the reference and this decoder read
    // through different end-of-scan windows are the ones that reach it.
    eprintln!("worst block difference from the reference picture: {worst:.1} counts");
}

/// A transmission that stops part way ends there.
///
/// The decoder runs on its own clock once it has the header, so noise after
/// the transmitter stops reads as lines and it will fill the rest of the
/// picture with them. Half of this recording, then silence: the picture has
/// to end where the signal did.
#[test]
fn a_transmission_that_stopped_does_not_get_written_to_the_end() {
    let Some((audio, rate)) = audio() else { return };
    let half = audio.len() / 2;

    let mut rx = sstv::Receiver::new(rate);
    for block in audio[..half].chunks(4096) {
        rx.push(block);
    }
    let sent = rx.picture().map(|p| p.lines).unwrap_or(0);
    // Half the recording is a bit over half the picture, since the header and
    // the tail either side are not picture.
    assert!((120..=160).contains(&sent), "half the recording read {sent} lines");

    // Then a minute of the quiet that follows a transmission.
    for _ in 0..30 {
        rx.push(&vec![0.0; rate as usize]);
    }
    let p = rx.picture().expect("the picture");
    // Not exactly what was read before the silence: the line that straddles
    // the cut is still readable, and the detector needs a line or two of
    // nothing before it calls the transmission over. What matters is that it
    // stops there rather than running on to 256.
    assert!(
        (sent..=sent + 3).contains(&p.lines),
        "{} lines after the signal stopped at {sent}",
        p.lines
    );
    assert!(!rx.receiving(), "the decoder is listening for the next one");
    assert_eq!(p.height, 256, "the picture is still a whole Martin 1 frame");
}
