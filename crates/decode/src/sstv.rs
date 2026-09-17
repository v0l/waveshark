//! Slow-scan television: a picture sent as a tone that changes with
//! brightness.
//!
//! There is nothing digital in it below the VIS code. A line is a sync pulse
//! at 1200 Hz, then each pixel's brightness as a frequency between 1500 Hz
//! for black and 2300 Hz for white, one channel at a time. So the decoder is
//! a clock and a frequency meter: find the calibration header, read the seven
//! bits that name the mode, then walk the line timings sampling
//! `dsp::tone::ToneMeter` once per pixel, realigning on every sync pulse
//! because transmitter and receiver clocks differ by parts per million and a
//! picture is two minutes long.
//!
//! The modes and their timings are the published ones, checked against
//! colaclanth's Python `sstv` decoder, which is also what the capture test
//! compares pictures with.

use crate::linescan::luma;
use dsp::tone::ToneMeter;

/// Which order a mode sends its three channels in, and what they mean.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Colour {
    /// Green, blue, red: what the Martin and Scottie modes send.
    Gbr,
    /// Luminance and two colour differences.
    Yuv,
}

/// One mode's timings, in seconds unless said otherwise.
#[derive(Clone, Copy, Debug)]
pub struct Mode {
    pub name: &'static str,
    pub vis: u8,
    pub colour: Colour,
    pub width: usize,
    pub height: usize,
    pub scan_time: f64,
    /// Robot modes send the two colour difference channels at half width in
    /// time, so their second and third channels run at a different rate.
    pub half_scan_time: f64,
    pub sync_pulse: f64,
    pub channels: usize,
    /// Which channel the line's sync pulse belongs to. Scottie puts it inside
    /// the line rather than in front of it.
    pub sync_channel: usize,
    /// Where each channel starts, from the start of the line.
    pub offsets: [f64; 3],
    pub line_time: f64,
    /// How many pixel times wide the sampling window is. Wider is smoother
    /// and blurrier; these are the reference decoder's values.
    pub window_factor: f64,
    /// Scottie begins with one sync pulse before the picture.
    pub start_sync: bool,
    pub half_scan: bool,
    /// Robot 36 alternates which colour difference each line carries.
    pub alt_scan: bool,
}

impl Mode {
    pub fn pixel_time(&self) -> f64 {
        self.scan_time / self.width as f64
    }

    pub fn half_pixel_time(&self) -> f64 {
        self.half_scan_time / self.width as f64
    }
}

const fn martin(name: &'static str, vis: u8, scan: f64, window: f64) -> Mode {
    let sync_porch = 0.000572;
    let sep = 0.000572;
    let chan = sep + scan;
    let first = 0.004862 + sync_porch;
    Mode {
        name,
        vis,
        colour: Colour::Gbr,
        width: 320,
        height: 256,
        scan_time: scan,
        half_scan_time: 0.0,
        sync_pulse: 0.004862,
        channels: 3,
        sync_channel: 0,
        offsets: [first, first + chan, first + 2.0 * chan],
        line_time: 0.004862 + sync_porch + 3.0 * chan,
        window_factor: window,
        start_sync: false,
        half_scan: false,
        alt_scan: false,
    }
}

const fn scottie(name: &'static str, vis: u8, scan: f64, window: f64) -> Mode {
    let sync = 0.009;
    let sync_porch = 0.0015;
    let sep = 0.0015;
    let chan = sep + scan;
    let third = sync + sync_porch;
    Mode {
        name,
        vis,
        colour: Colour::Gbr,
        width: 320,
        height: 256,
        scan_time: scan,
        half_scan_time: 0.0,
        sync_pulse: sync,
        channels: 3,
        sync_channel: 2,
        offsets: [third + chan, third + 2.0 * chan, third],
        line_time: sync + 3.0 * chan,
        window_factor: window,
        start_sync: true,
        half_scan: false,
        alt_scan: false,
    }
}

/// Martin 1 and 2, Scottie 1, 2 and DX, Robot 36 and 72: the seven modes the
/// VIS codes in common use name.
pub const MODES: [Mode; 7] = [
    martin("Martin 1", 44, 0.146432, 2.34),
    martin("Martin 2", 40, 0.073216, 4.68),
    scottie("Scottie 1", 60, 0.138240, 2.48),
    scottie("Scottie 2", 56, 0.088064, 3.82),
    scottie("Scottie DX", 76, 0.345600, 0.98),
    // Robot 36: luminance every line, one colour difference alternating.
    Mode {
        name: "Robot 36",
        vis: 8,
        colour: Colour::Yuv,
        width: 320,
        height: 240,
        scan_time: 0.088,
        half_scan_time: 0.044,
        sync_pulse: 0.009,
        channels: 2,
        sync_channel: 0,
        offsets: [0.012, 0.012 + 0.0925 + 0.0015, 0.0],
        line_time: 0.012 + 0.0925 + 0.0015 + 0.044,
        window_factor: 7.70,
        start_sync: false,
        half_scan: true,
        alt_scan: true,
    },
    // Robot 72: luminance and both colour differences, the latter at half
    // width in time.
    Mode {
        name: "Robot 72",
        vis: 12,
        colour: Colour::Yuv,
        width: 320,
        height: 240,
        scan_time: 0.138,
        half_scan_time: 0.069,
        sync_pulse: 0.009,
        channels: 3,
        sync_channel: 0,
        offsets: [0.012, 0.012 + 0.1425 + 0.0015, 0.012 + 0.1425 + 0.0015 + 0.0735 + 0.0015],
        line_time: 0.012 + 0.1425 + 0.0015 + 0.0735 + 0.0015 + 0.069,
        window_factor: 4.88,
        start_sync: false,
        half_scan: true,
        alt_scan: false,
    },
];

pub fn mode_of(vis: u8) -> Option<&'static Mode> {
    MODES.iter().find(|m| m.vis == vis)
}

/// The calibration header: 1900 Hz, a 1200 Hz break, 1900 Hz again, then the
/// 1200 Hz start bit of the VIS code.
const BREAK_OFFSET: f64 = 0.300;
const LEADER_OFFSET: f64 = 0.010 + BREAK_OFFSET;
const VIS_START_OFFSET: f64 = 0.300 + LEADER_OFFSET;
const HDR_SIZE: f64 = 0.030 + VIS_START_OFFSET;
const HDR_WINDOW: f64 = 0.010;
/// The break and the VIS start bit are short, so they are tested over a
/// window narrower than they are, centred inside them: the search steps in
/// millisecond jumps, and a window as wide as the tone it is testing lets a
/// millisecond of error drag the neighbouring leader into it. Hunting a live
/// stream showed this up, since where a block boundary falls decides which
/// offsets get tried and the header was found or missed accordingly.
const SHORT_WINDOW: f64 = 0.006;
const SHORT_INSET: f64 = 0.002;
const VIS_BIT: f64 = 0.030;

/// A decoded picture, in RGB, row major.
#[derive(Clone, Debug)]
pub struct Picture {
    pub mode: &'static Mode,
    pub width: usize,
    pub height: usize,
    pub rgb: Vec<u8>,
    /// How many lines were read before the audio ran out. A transmission cut
    /// short leaves the rest black, and this says where that started.
    pub lines: usize,
}

/// The tones a picture is sent in, with the margin a reading off a noisy
/// signal needs: black is 1500 Hz and white 2300, and a real transmission
/// overshoots both by a little. Below this band is a sync pulse, above it is
/// nothing any mode sends. Tightening it any further starts throwing away
/// black.
const PICTURE_LOW_HZ: f64 = 1330.0;
const PICTURE_HIGH_HZ: f64 = 2470.0;

/// Fill the runs of pixels whose tone was not a picture tone, straight across
/// from the last good pixel to the next one. Returns how many were filled.
///
/// Only holes with a good pixel on both sides are filled. A run at either end
/// of the line is the sampling window overlapping what comes before or after
/// the scan rather than a dropout, and there is nothing on the far side to
/// interpolate towards; a line with no good pixel at all is left as it is,
/// and the caller treats it as a line that did not arrive.
fn fill_gaps(row: &mut [u8], gaps: &[bool]) -> usize {
    let holes = gaps.iter().filter(|g| **g).count();
    if holes == 0 || holes == row.len() {
        return holes;
    }
    let mut at = 0;
    while at < row.len() {
        if !gaps[at] {
            at += 1;
            continue;
        }
        let start = at;
        while at < row.len() && gaps[at] {
            at += 1;
        }
        let (Some(before), Some(after)) =
            (start.checked_sub(1).map(|i| row[i] as f32), row.get(at).map(|v| *v as f32))
        else {
            continue;
        };
        let span = (at - start) as f32;
        for (k, i) in (start..at).enumerate() {
            row[i] = (before + (after - before) * (k as f32 + 1.0) / (span + 1.0)) as u8;
        }
    }
    holes
}

/// Where the calibration header ends, searching from the start of `audio`.
fn find_header(audio: &[f32], meter: &mut ToneMeter) -> Option<usize> {
    let rate = meter.rate();
    let header = (HDR_SIZE * rate).round() as usize;
    let window = (HDR_WINDOW * rate).round() as usize;
    let jump = (0.001 * rate).round() as usize;
    if audio.len() < header {
        return None;
    }
    let short = (SHORT_WINDOW * rate).round() as usize;
    let inset = (SHORT_INSET * rate).round() as usize;
    let brk = (BREAK_OFFSET * rate).round() as usize + inset;
    let leader2 = (LEADER_OFFSET * rate).round() as usize;
    let vis = (VIS_START_OFFSET * rate).round() as usize + inset;

    let mut at = 0;
    while at + header < audio.len() {
        let area = &audio[at..at + header];
        let near = |off: usize, len: usize, hz: f64, meter: &mut ToneMeter| {
            (meter.peak_hz(&area[off..off + len]) - hz).abs() < 50.0
        };
        if near(0, window, 1900.0, meter)
            && near(brk, short, 1200.0, meter)
            && near(leader2, window, 1900.0, meter)
            && near(vis, short, 1200.0, meter)
        {
            return Some(at + header);
        }
        at += jump;
    }
    None
}

/// The seven bits after the header, which name the mode. 1100 Hz is a one,
/// 1300 Hz a zero, and the eighth bit makes the count even.
fn read_vis(audio: &[f32], at: usize, meter: &mut ToneMeter) -> Option<u8> {
    let bit = (VIS_BIT * meter.rate()).round() as usize;
    let mut bits = [0u8; 8];
    for (i, b) in bits.iter_mut().enumerate() {
        let start = at + i * bit;
        if start + bit > audio.len() {
            return None;
        }
        *b = (meter.peak_hz(&audio[start..start + bit]) <= 1200.0) as u8;
    }
    if bits.iter().map(|b| *b as u32).sum::<u32>() % 2 != 0 {
        return None;
    }
    let mut value = 0u8;
    for b in bits[..7].iter().rev() {
        value = (value << 1) | b;
    }
    Some(value)
}

/// Where the next sync pulse starts, hunting forward from `from`.
///
/// Realigning on every line is what keeps a two minute picture straight: a
/// clock 20 parts per million out drifts a whole pixel every four lines.
fn align_sync(
    audio: &[f32],
    from: usize,
    mode: &Mode,
    meter: &mut ToneMeter,
    want_start: bool,
) -> Option<(usize, usize, bool)> {
    let rate = meter.rate();
    let window = (mode.sync_pulse * 1.4 * rate).round() as usize;
    if from + window >= audio.len() {
        return None;
    }
    // What is looked for is where the sync ends, since everything after it is
    // picture and picture is above this threshold while the pulse is below.
    let threshold = 1350.0;
    // Back a pulse and a half, because a line that arrived early is still
    // this line and a search that only looks forward cannot see it. Looking
    // forward only was a picture that slid sideways a little more on every
    // line: a transmitter running fast is always a touch ahead of the clock,
    // the pulse had already gone by, and the hunt caught the next thing above
    // the threshold instead, which is picture.
    let begin = from.saturating_sub((mode.sync_pulse * 1.5 * rate).round() as usize);
    // A pulse comes once a line, so a search wider than one is looking at
    // somebody else's line, and in silence an unbounded one walked to the end
    // of the buffer and left the decoder stuck there.
    let stop = (from + (mode.line_time * rate).round() as usize).min(audio.len() - window);
    // A pulse lasts; noise crossing the threshold does not. Confirming a
    // crossing before believing it costs nothing on a clean signal, where the
    // first crossing is the pulse, and is what stops a weak one being read as
    // a line starting wherever the noise happened to peak.
    let hold = (mode.sync_pulse * 0.5 * rate).round().max(2.0) as usize;
    let probe = ((rate * 0.0005).round() as usize).max(1);

    let mut at = begin;
    let mut found = false;
    // The pulse itself has to be seen before its end means anything.
    let mut in_pulse = false;
    while at < stop {
        if meter.peak_hz(&audio[at..at + window]) <= threshold {
            in_pulse = true;
            at += 1;
            continue;
        }
        if !in_pulse {
            at += 1;
            continue;
        }
        let until = (at + hold).min(stop);
        let mut k = at + probe;
        let mut steady = true;
        while k < until {
            if meter.peak_hz(&audio[k..k + window]) <= threshold {
                steady = false;
                break;
            }
            k += probe;
        }
        if steady {
            found = true;
            break;
        }
        // Past the tone that was not the end of a pulse, rather than one
        // sample on: every sample of it would otherwise be tested again.
        at = k + probe;
        in_pulse = false;
    }
    let at = if found { at } else { from };
    let end = at + window / 2;
    let start = match want_start {
        true => end.saturating_sub((mode.sync_pulse * rate).round() as usize),
        false => end,
    };
    // How far from the prediction it was, and whether a pulse was seen at
    // all: in noise something crosses the threshold eventually, so the
    // distance is what tells a line that arrived from one that was invented.
    Some((start, at.abs_diff(from), found))
}

/// The audio that sends `rgb` in `mode`: the calibration header, the VIS
/// code, then a line at a time.
///
/// The inverse of [`decode`] for the modes that send three full-width
/// channels, which is Martin and Scottie. A Robot mode sends luminance and
/// two half-width colour differences and is refused rather than sent wrong.
///
/// `rgb` is row major, three bytes a pixel, `mode.width` by `mode.height`; a
/// picture short of that is sent as far as it goes and the rest black.
pub fn encode(rgb: &[u8], mode: &Mode, rate: f64) -> Option<Vec<f32>> {
    if mode.colour != Colour::Gbr || mode.channels != 3 {
        return None;
    }
    let mut out =
        Vec::with_capacity(((HDR_SIZE + mode.height as f64 * mode.line_time) * rate) as usize);
    let mut phase = 0.0f64;
    // Where the tone that has been written should have ended, in seconds.
    // A pixel at 11 kHz is two and a half samples long, so a tone rounded on
    // its own runs a fifth over and a picture ends a quarter of a minute
    // late; each tone is written up to its place on the timeline instead.
    let mut elapsed = 0.0f64;
    let mut tone = |hz: f64, seconds: f64, out: &mut Vec<f32>| {
        elapsed += seconds;
        let until = (elapsed * rate).round() as usize;
        // Phase continuous across every tone, because a picture is read by a
        // frequency meter and a step is a frequency of its own.
        while out.len() < until {
            phase += std::f64::consts::TAU * hz / rate;
            out.push(phase.sin() as f32);
        }
    };

    tone(1900.0, BREAK_OFFSET, &mut out);
    tone(1200.0, LEADER_OFFSET - BREAK_OFFSET, &mut out);
    tone(1900.0, VIS_START_OFFSET - LEADER_OFFSET, &mut out);
    tone(1200.0, HDR_SIZE - VIS_START_OFFSET, &mut out);
    // Seven bits least significant first, then a parity bit making the count
    // of ones even. A one is 1100 Hz.
    let mut ones = 0;
    for b in 0..7 {
        let one = mode.vis >> b & 1 == 1;
        ones += u32::from(one);
        tone(if one { 1100.0 } else { 1300.0 }, VIS_BIT, &mut out);
    }
    tone(if ones % 2 == 1 { 1100.0 } else { 1300.0 }, VIS_BIT, &mut out);

    // The channel order is green, blue, red, and the offsets say when each
    // goes out; anything between them is the separator tone.
    let plane = [1usize, 2, 0];
    let pixel_time = mode.pixel_time();
    for y in 0..mode.height {
        tone(1200.0, mode.sync_pulse, &mut out);
        let mut at = mode.sync_pulse;
        let mut order: Vec<usize> = (0..3).collect();
        order.sort_by(|a, b| mode.offsets[*a].total_cmp(&mode.offsets[*b]));
        for chan in order {
            tone(1500.0, mode.offsets[chan] - at, &mut out);
            for x in 0..mode.width {
                let i = (y * mode.width + x) * 3 + plane[chan];
                let v = f64::from(rgb.get(i).copied().unwrap_or(0));
                tone(1500.0 + 800.0 * v / 255.0, pixel_time, &mut out);
            }
            at = mode.offsets[chan] + mode.scan_time;
        }
        tone(1500.0, mode.line_time - at, &mut out);
    }
    Some(out)
}

/// Decode the first picture in `audio`, or `None` where there is no header.
///
/// The whole-buffer form of [`Receiver`], and the same code: a test that
/// decodes a recording and a node reading a live stream must not be two
/// decoders that agree only by luck.
pub fn decode(audio: &[f32], rate: f64) -> Option<Picture> {
    let mut rx = Receiver::new(rate);
    rx.push(audio);
    rx.finish();
    rx.picture().cloned()
}

/// How a line came out.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) enum Read {
    /// The audio ran out part way. On a live stream the rest is still
    /// coming; at the end of one it never will.
    Hungry,
    /// Read, with the sync where it was expected and the picture at a level
    /// the rest of the transmission was at.
    Line { synced: bool, level: f32 },
}

/// One line of the picture, read into the channel planes.
fn read_line(
    audio: &[f32],
    base: u64,
    meter: &mut ToneMeter,
    mode: &'static Mode,
    line: usize,
    seq: &mut u64,
    planes: &mut [Vec<Vec<u8>>],
) -> Read {
    let rate = meter.rate();
    let here = |abs: u64| -> Option<usize> { abs.checked_sub(base).map(|v| v as usize) };
    // A sync found more than this far from where the clock said it would be
    // is not this line's sync. A quarter of a line is far more drift than any
    // transmitter has and far less than a line of noise needs.
    let slack = (mode.line_time * 0.25 * rate).round() as usize;
    let mut synced = true;

    if mode.sync_channel > 0 && line == 0 {
        // Scottie's sync sits inside the line, so the first line starts
        // before the pulse that was just found.
        let back = ((mode.offsets[mode.sync_channel] + mode.scan_time) * rate).round() as u64;
        *seq = seq.saturating_sub(back);
    }
    let mut row = vec![vec![0u8; mode.width]; mode.channels];
    // Pixels whose tone was not a picture tone, per channel. Left as they
    // were read they are black or white speckle; what they are is a gap, and
    // a gap between two known pixels is better guessed than declared.
    let mut gaps = vec![vec![false; mode.width]; mode.channels];
    for chan in 0..mode.channels {
        if chan == mode.sync_channel {
            if line > 0 || chan > 0 {
                *seq += (mode.line_time * rate).round() as u64;
            }
            let Some(from) = here(*seq) else { return Read::Hungry };
            let Some((at, walked, found)) = align_sync(audio, from, mode, meter, true) else {
                return Read::Hungry;
            };
            // Follow the sync wherever it is inside the line, because that is
            // what keeps a picture straight: transmitter and receiver clocks
            // differ, and a line that started late is still this line. What
            // the distance decides is only whether to believe there was a
            // transmission here at all, which a run of unsynced lines answers.
            if found {
                *seq = base + at as u64;
            }
            synced &= found && walked <= slack;
        }
        let half = mode.half_scan && chan > 0;
        let pixel_time = if half { mode.half_pixel_time() } else { mode.pixel_time() };
        let scan_time = if half { mode.half_scan_time } else { mode.scan_time };
        let half_window = pixel_time * mode.window_factor / 2.0;
        // The window is several pixels wide, so at the ends of a scan it
        // reaches past it. Where what follows is a separator pulse that is
        // where the smearing stops, but a Robot mode's colour difference
        // ends the line, so the last pixels read the next line's sync pulse:
        // at 1200 Hz both differences come out at zero, which is a bright
        // green stripe down the right of the picture.
        let first = mode.offsets[chan];
        let last = first + scan_time;
        for px in 0..mode.width {
            let centre = first + px as f64 * pixel_time;
            let from = (centre - half_window).max(first);
            let to = (centre + half_window).min(last);
            let at = (*seq as f64 - base as f64 + from * rate).round() as isize;
            let len = (((to - from) * rate).round() as usize).max(4);
            if at < 0 || at as usize + len >= audio.len() {
                return Read::Hungry;
            }
            let hz = meter.peak_hz(&audio[at as usize..at as usize + len]);
            row[chan][px] = luma(hz);
            gaps[chan][px] = !(PICTURE_LOW_HZ..=PICTURE_HIGH_HZ).contains(&hz);
        }
        // Interference, a dropout or a sync pulse read as picture: whatever
        // it was, the tone was not one this mode sends, and a run of them
        // between two pixels that are is a hole to fill rather than evidence.
        let holes = fill_gaps(&mut row[chan], &gaps[chan]);
        // A line that is mostly holes is not a line that was received.
        if holes * 2 > mode.width {
            synced = false;
        }
    }
    // The level of the line, for telling a transmission that stopped from
    // one that faded: the picture tones are a constant amplitude, so a line
    // of quiet is a line of nothing.
    let from = here(*seq).unwrap_or(0);
    let span = ((mode.line_time * rate).round() as usize).min(audio.len().saturating_sub(from));
    let level = match span {
        0 => 0.0,
        n => (audio[from..from + n].iter().map(|v| v * v).sum::<f32>() / n as f32).sqrt(),
    };
    planes[line] = row;
    Read::Line { synced, level }
}

/// One line of the picture as RGB, which is where a mode's colour order and
/// Robot 36's alternating colour difference are undone.
fn convert_row(mode: &'static Mode, planes: &[Vec<Vec<u8>>], y: usize, out: &mut [u8]) {
    for x in 0..mode.width {
        let px = match (mode.channels, mode.colour) {
            (3, Colour::Gbr) => (planes[y][2][x], planes[y][0][x], planes[y][1][x]),
            // Robot 72 sends luminance, then R-Y, then B-Y.
            (3, Colour::Yuv) => yuv(planes[y][0][x], planes[y][1][x], planes[y][2][x]),
            (2, Colour::Yuv) => {
                // Robot 36 sends one colour difference a line: R-Y on the
                // even ones, B-Y on the odd. So a line has half of what it
                // needs and borrows the other half from the line after it,
                // which is why conversion runs a line behind the scan.
                let (cr, cb) = match y % 2 == 0 {
                    true => (planes[y][1][x], planes[(y + 1).min(mode.height - 1)][1][x]),
                    false => (planes[y - 1][1][x], planes[y][1][x]),
                };
                yuv(planes[y][0][x], cr, cb)
            }
            _ => (planes[y][0][x], planes[y][0][x], planes[y][0][x]),
        };
        out[x * 3] = px.0;
        out[x * 3 + 1] = px.1;
        out[x * 3 + 2] = px.2;
    }
}

/// The conversion every SSTV decoder uses, which is JPEG's YCbCr.
fn yuv(y: u8, cr: u8, cb: u8) -> (u8, u8, u8) {
    let (y, cb, cr) = (y as f32, cb as f32 - 128.0, cr as f32 - 128.0);
    let r = y + 1.402 * cr;
    let g = y - 0.344136 * cb - 0.714136 * cr;
    let b = y + 1.772 * cb;
    (r.clamp(0.0, 255.0) as u8, g.clamp(0.0, 255.0) as u8, b.clamp(0.0, 255.0) as u8)
}

/// Rows that have just been read, for a consumer that paints them into a
/// picture rather than redrawing one.
#[derive(Clone, Debug)]
pub struct Lines {
    pub mode: &'static Mode,
    /// Which picture these belong to, counted from the receiver's first.
    pub picture: u64,
    /// The row the batch starts at.
    pub first: usize,
    /// The rows themselves, RGB, row major.
    pub rgb: Vec<u8>,
    /// Whether the transmission is over: every line read, or the decoder
    /// giving up on one that stopped part way.
    pub complete: bool,
}

/// A picture being received, fed audio as it arrives.
///
/// A transmission is two minutes long, so a node cannot wait for the end of
/// the stream and cannot keep the stream either. This keeps what it needs: a
/// few seconds while it is hunting for a header, and from then on only the
/// audio of the line it is reading.
///
/// Lines are read once, as the audio for each arrives, and handed over as
/// they are. The first version rescanned the whole transmission every
/// sixteen lines to hand over a picture, which is the same decode done
/// sixteen times and a picture that flashes rather than one that fills in.
pub struct Receiver {
    rate: f64,
    meter: ToneMeter,
    audio: Vec<f32>,
    /// Absolute position of `audio[0]`, since the buffer is drained as lines
    /// are read and every timing here is absolute.
    base: u64,
    state: State,
    /// Pictures started, which names the one being received.
    pictures: u64,
    canvas: Option<Picture>,
}

enum State {
    Hunting,
    Reading {
        mode: &'static Mode,
        /// Where the current line starts.
        seq: u64,
        line: usize,
        planes: Vec<Vec<Vec<u8>>>,
        /// Lines read but not painted: they had no sync where one was due, or
        /// arrived far below the level the picture has been at, so they wait
        /// until a good line proves them part of a fade rather than the end
        /// of the transmission.
        held: usize,
        /// The loudest the transmission has been, for telling a fade from
        /// nothing at all.
        loud: f32,
    },
}

/// How much audio to keep while hunting: enough for the header search window
/// plus the jump it steps by.
const HUNT_KEEP: f64 = HDR_SIZE + 0.5;

/// Lines with no sync, or with no signal, before the transmission is taken to
/// be over. Three is longer than any burst of interference lasts and shorter
/// than anybody would want painted into their picture.
const LOST_LINES: usize = 3;

/// How far below the loudest the picture has been a line has to be for it to
/// count as nothing arriving. 26 dB down: a fade that deep has no picture in
/// it either.
const QUIET: f32 = 0.05;

impl Receiver {
    pub fn new(rate: f64) -> Self {
        Self {
            rate,
            meter: ToneMeter::new(rate),
            audio: Vec::new(),
            base: 0,
            state: State::Hunting,
            pictures: 0,
            canvas: None,
        }
    }

    pub fn reset(&mut self) {
        let rate = self.rate;
        *self = Self::new(rate);
    }

    /// Whether a transmission is being received right now.
    pub fn receiving(&self) -> bool {
        matches!(self.state, State::Reading { .. })
    }

    /// The picture as it stands, complete or not.
    pub fn picture(&self) -> Option<&Picture> {
        self.canvas.as_ref()
    }

    /// Feed audio. Returns whatever lines that completed.
    pub fn push(&mut self, audio: &[f32]) -> Option<Lines> {
        self.audio.extend_from_slice(audio);
        if matches!(self.state, State::Hunting) {
            self.hunt();
        }
        self.advance(false)
    }

    /// No more audio is coming: give up on the picture in progress and hand
    /// over what was read. What a file ends in, and what a transmission that
    /// faded out becomes.
    pub fn finish(&mut self) -> Option<Lines> {
        self.advance(true)
    }

    fn hunt(&mut self) {
        let need = (HDR_SIZE * self.rate).round() as usize;
        if self.audio.len() < need {
            return;
        }
        if let Some(end) = find_header(&self.audio, &mut self.meter) {
            // The VIS code is 240 ms behind the header, which on a live
            // stream has usually not arrived yet. Waiting for it is not
            // optional: reading it early fails, and treating that as a bad
            // header threw the header away and the picture with it.
            if self.audio.len() < end + (VIS_BIT * 8.0 * self.rate).round() as usize {
                return;
            }
            if let Some(mode) = read_vis(&self.audio, end, &mut self.meter).and_then(mode_of) {
                let mut seq =
                    self.base + (end + (VIS_BIT * 9.0 * self.rate).round() as usize) as u64;
                if mode.start_sync {
                    // Scottie opens with a sync pulse before the picture.
                    let from = (seq - self.base) as usize;
                    match align_sync(&self.audio, from, mode, &mut self.meter, false) {
                        Some((at, _, _)) => seq = self.base + at as u64,
                        None => return,
                    }
                }
                self.pictures += 1;
                self.canvas = Some(Picture {
                    mode,
                    width: mode.width,
                    height: mode.height,
                    rgb: vec![0; mode.width * mode.height * 3],
                    lines: 0,
                });
                self.state = State::Reading {
                    mode,
                    seq,
                    line: 0,
                    planes: vec![vec![vec![0u8; mode.width]; mode.channels]; mode.height],
                    held: 0,
                    loud: 0.0,
                };
                return;
            }
            // A header with a VIS this receiver cannot read is still a
            // header: skip past it rather than finding it again every block.
            self.drain_to(self.base + end as u64);
            return;
        }
        let keep = (HUNT_KEEP * self.rate).round() as usize;
        if self.audio.len() > keep {
            self.drain_to(self.base + (self.audio.len() - keep) as u64);
        }
    }

    fn drain_to(&mut self, abs: u64) {
        let Some(cut) = abs.checked_sub(self.base) else { return };
        let cut = (cut as usize).min(self.audio.len());
        self.audio.drain(..cut);
        self.base += cut as u64;
    }

    /// Read as many lines as the audio in hand allows.
    fn advance(&mut self, ending: bool) -> Option<Lines> {
        // Taken out of `self` for the duration: reading a line needs the
        // buffer and the meter, and the borrow checker will not have both
        // halves of the receiver at once.
        let taken = std::mem::replace(&mut self.state, State::Hunting);
        let State::Reading { mode, mut seq, mut line, mut planes, mut held, mut loud } = taken
        else {
            return None;
        };
        let rate = self.rate;
        // A line cannot be read until the audio behind the last pixel of its
        // last channel has arrived, plus the window the sync search walks.
        let reach = mode.line_time
            + mode.offsets.iter().take(mode.channels).fold(0.0f64, |m, o| m.max(*o))
            + mode.scan_time
            + 2.0 * mode.sync_pulse;
        let reach = (reach * rate).round() as u64;

        let mut first = line;
        let mut rows: Vec<u8> = Vec::new();
        let mut complete = false;
        loop {
            if line >= mode.height {
                complete = true;
                break;
            }
            let have = self.base + self.audio.len() as u64;
            if !ending && have < seq + reach {
                break;
            }
            match read_line(
                &self.audio,
                self.base,
                &mut self.meter,
                mode,
                line,
                &mut seq,
                &mut planes,
            ) {
                // Out of audio part way: on a live stream the rest is still
                // coming, at the end of one it never will.
                Read::Hungry => {
                    complete = ending;
                    break;
                }
                // A transmission that stopped leaves the decoder reading its
                // own clock across noise, and it will fill the rest of the
                // picture with it. A line with no sync where one was due, or
                // one far below the level the picture has been at, is not a
                // line, and a few in a row is the end of the transmission.
                Read::Line { synced, level } => {
                    loud = loud.max(level);
                    let quiet = loud > 0.0 && level < loud * QUIET;
                    held = match !synced || quiet {
                        true => held + 1,
                        false => 0,
                    };
                }
            }
            line += 1;
            if held >= LOST_LINES {
                // What is held is noise, so it is never painted: the picture
                // ends at the last line that arrived.
                complete = true;
                break;
            }
            let ready = match mode.alt_scan {
                // Robot 36 needs the next line's colour difference to convert
                // this one, so conversion runs one line behind it.
                true => line.saturating_sub(held + 1),
                false => line.saturating_sub(held),
            };
            let canvas = self.canvas.as_mut().expect("a canvas while reading");
            while canvas.lines < ready {
                let y = canvas.lines;
                let mut row = vec![0u8; mode.width * 3];
                convert_row(mode, &planes, y, &mut row);
                canvas.rgb[y * mode.width * 3..(y + 1) * mode.width * 3].copy_from_slice(&row);
                canvas.lines += 1;
                if rows.is_empty() {
                    first = y;
                }
                rows.extend_from_slice(&row);
            }
            // The audio behind a line that is read is not needed again.
            let keep = seq.saturating_sub((mode.line_time * rate).round() as u64);
            self.drain_to(keep);
        }

        if complete {
            // The last line of an alternating-scan mode has no neighbour to
            // borrow from, so it converts against itself.
            if let (Some(canvas), true) = (self.canvas.as_mut(), mode.alt_scan) {
                while canvas.lines < line {
                    let y = canvas.lines;
                    let mut row = vec![0u8; mode.width * 3];
                    convert_row(mode, &planes, y, &mut row);
                    canvas.rgb[y * mode.width * 3..(y + 1) * mode.width * 3].copy_from_slice(&row);
                    canvas.lines += 1;
                    if rows.is_empty() {
                        first = y;
                    }
                    rows.extend_from_slice(&row);
                }
            }
        } else {
            self.state = State::Reading { mode, seq, line, planes, held, loud };
        }
        if rows.is_empty() && !complete {
            return None;
        }
        Some(Lines { mode, picture: self.pictures, first, rgb: rows, complete })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A picture sent and read back. The mode is Martin 2 because it is the
    /// shortest of the full-width modes at 58 seconds, and the rate is what
    /// an SSTV program uses.
    #[test]
    fn a_picture_this_encoder_sent_comes_back_off_the_tones() {
        let mode = MODES.iter().find(|m| m.name == "Martin 2").unwrap();
        // Eight vertical bars of a flat colour each, which is a picture
        // whose every pixel has a known value and whose edges show a line
        // read at the wrong offset.
        let bars: [[u8; 3]; 8] = [
            [255, 255, 255],
            [255, 255, 0],
            [0, 255, 255],
            [0, 255, 0],
            [255, 0, 255],
            [255, 0, 0],
            [0, 0, 255],
            [0, 0, 0],
        ];
        let mut rgb = vec![0u8; mode.width * mode.height * 3];
        for y in 0..mode.height {
            for x in 0..mode.width {
                let bar = bars[x * 8 / mode.width];
                rgb[(y * mode.width + x) * 3..][..3].copy_from_slice(&bar);
            }
        }

        let rate = 44_100.0;
        let audio = encode(&rgb, mode, rate).expect("a full width mode encodes");
        // Two leaders, a break, the VIS code, then 256 lines.
        let seconds = audio.len() as f64 / rate;
        assert!(
            (seconds - (HDR_SIZE + 8.0 * VIS_BIT + 256.0 * mode.line_time)).abs() < 0.01,
            "{seconds} s"
        );

        let got = decode(&audio, rate).expect("a picture");
        assert_eq!(got.mode.name, "Martin 2", "the VIS code named the mode");
        // Every line but the last: the sampling window is several pixels
        // wide, so reading the final line needs audio from after the
        // transmission that a file ending at the picture does not have.
        assert_eq!(got.lines, mode.height - 1);

        // What comes back is the right colour at the right place, but
        // compressed toward mid grey: black reads 35 and white 215 rather
        // than 0 and 255. That is the tone meter's peak estimator, which
        // interpolates over three bins of a window only about a kilohertz
        // wide, and it is why the off-air mode tests match a bar by which
        // colour is nearest rather than by its value. Pinned here because it
        // is measured from a picture whose every pixel is known.
        let centre = |b: usize, y: usize| -> [u8; 3] {
            let x = b * mode.width / 8 + mode.width / 16;
            got.rgb[(y * mode.width + x) * 3..][..3].try_into().unwrap()
        };
        let mut checked = 0;
        for y in [8, 128, 247] {
            for (b, want) in bars.iter().enumerate() {
                let px = centre(b, y);
                for c in 0..3 {
                    let sent = f64::from(want[c]);
                    let expect = 35.0 + sent * (215.0 - 35.0) / 255.0;
                    assert!(
                        (f64::from(px[c]) - expect).abs() <= 6.0,
                        "bar {b} at line {y} channel {c}: {} not near {expect:.0}",
                        px[c]
                    );
                }
                checked += 1;
            }
        }
        assert_eq!(checked, 24, "eight bars on three lines");
        // The bars are in the order they were sent, which is what a channel
        // out of step would break however the levels came out.
        assert!(centre(0, 128)[0] > centre(7, 128)[0], "white brighter than black");
        assert!(centre(3, 128)[1] > centre(3, 128)[2], "the green bar has no blue");
    }

    /// A Robot mode sends luminance and two half-width colour differences,
    /// which this encoder does not build, so it refuses rather than sending
    /// a picture no receiver would show.
    #[test]
    fn a_mode_the_encoder_cannot_send_is_refused() {
        let robot = MODES.iter().find(|m| m.name == "Robot 36").unwrap();
        assert!(encode(&[0; 320 * 240 * 3], robot, 11_025.0).is_none());
        let robot72 = MODES.iter().find(|m| m.name == "Robot 72").unwrap();
        assert!(encode(&[0; 320 * 240 * 3], robot72, 11_025.0).is_none());
    }

    /// The VIS codes are what a receiver keys off, so a wrong one here is a
    /// picture decoded at the wrong speed and nothing else to say so.
    #[test]
    fn the_vis_codes_name_the_modes_they_are_published_as() {
        assert_eq!(mode_of(44).unwrap().name, "Martin 1");
        assert_eq!(mode_of(40).unwrap().name, "Martin 2");
        assert_eq!(mode_of(60).unwrap().name, "Scottie 1");
        assert_eq!(mode_of(56).unwrap().name, "Scottie 2");
        assert_eq!(mode_of(76).unwrap().name, "Scottie DX");
        assert_eq!(mode_of(8).unwrap().name, "Robot 36");
        assert_eq!(mode_of(12).unwrap().name, "Robot 72");
        assert!(mode_of(0).is_none());
    }

    /// A Martin 1 line is 446.446 ms and its picture 114 seconds, which is
    /// the number every published table gives.
    #[test]
    fn a_martin_1_picture_takes_the_time_the_tables_say() {
        let m = mode_of(44).unwrap();
        assert!((m.line_time - 0.446446).abs() < 1e-6, "line is {}", m.line_time);
        let total = m.line_time * m.height as f64;
        assert!((total - 114.3).abs() < 0.1, "picture is {total} seconds");
    }

    /// A hole between two pixels that arrived is filled straight across, and
    /// one at the end of a line is left alone: there is nothing on the far
    /// side of it to guess from.
    #[test]
    fn a_hole_in_a_line_is_filled_from_both_sides() {
        let mut row = [10u8, 0, 0, 0, 50, 200, 0];
        let gaps = [false, true, true, true, false, false, true];
        assert_eq!(fill_gaps(&mut row, &gaps), 4, "holes found");
        assert_eq!(&row[..5], &[10, 20, 30, 40, 50], "filled straight across");
        assert_eq!(row[6], 0, "the run at the end is left as it was read");
    }

    /// A line with nothing in it is not a line to guess at.
    #[test]
    fn a_line_of_holes_is_left_alone() {
        let mut row = [7u8; 4];
        let gaps = [true; 4];
        assert_eq!(fill_gaps(&mut row, &gaps), 4);
        assert_eq!(row, [7u8; 4]);
    }
}
