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

/// A pixel's brightness from its tone: 1500 Hz is black, 2300 Hz white.
fn luma(hz: f64) -> u8 {
    let v = ((hz - 1500.0) / 3.1372549).round();
    v.clamp(0.0, 255.0) as u8
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
) -> Option<usize> {
    let rate = meter.rate();
    let window = (mode.sync_pulse * 1.4 * rate).round() as usize;
    if from + window >= audio.len() {
        return None;
    }
    let stop = audio.len() - window;
    let mut at = from;
    while at < stop {
        if meter.peak_hz(&audio[at..at + window]) > 1350.0 {
            break;
        }
        at += 1;
    }
    let end = at + window / 2;
    if want_start {
        Some(end.saturating_sub((mode.sync_pulse * rate).round() as usize))
    } else {
        Some(end)
    }
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

/// One line of the picture, read into the channel planes.
///
/// Returns where the next line starts, or `None` when the audio runs out
/// part way: a line half read is not written, since a viewer cannot tell a
/// half-read line from a received one.
fn read_line(
    audio: &[f32],
    base: u64,
    meter: &mut ToneMeter,
    mode: &'static Mode,
    line: usize,
    seq: &mut u64,
    planes: &mut [Vec<Vec<u8>>],
) -> bool {
    let rate = meter.rate();
    let here = |abs: u64| -> Option<usize> { abs.checked_sub(base).map(|v| v as usize) };

    if mode.sync_channel > 0 && line == 0 {
        // Scottie's sync sits inside the line, so the first line starts
        // before the pulse that was just found.
        let back = ((mode.offsets[mode.sync_channel] + mode.scan_time) * rate).round() as u64;
        *seq = seq.saturating_sub(back);
    }
    let mut row = vec![vec![0u8; mode.width]; mode.channels];
    for chan in 0..mode.channels {
        if chan == mode.sync_channel {
            if line > 0 || chan > 0 {
                *seq += (mode.line_time * rate).round() as u64;
            }
            let Some(from) = here(*seq) else { return false };
            let Some(found) = align_sync(audio, from, mode, meter, true) else { return false };
            *seq = base + found as u64;
        }
        let half = mode.half_scan && chan > 0;
        let pixel_time = if half { mode.half_pixel_time() } else { mode.pixel_time() };
        let scan_time = if half { mode.half_scan_time } else { mode.scan_time };
        let half_window = pixel_time * mode.window_factor / 2.0;
        let window = (half_window * 2.0 * rate).round() as usize;
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
                return false;
            }
            row[chan][px] = luma(meter.peak_hz(&audio[at as usize..at as usize + len]));
        }
    }
    planes[line] = row;
    true
}

/// One line of the picture as RGB, which is where a mode's colour order and
/// Robot 36's alternating colour difference are undone.
fn convert_row(mode: &'static Mode, planes: &[Vec<Vec<u8>>], y: usize, out: &mut [u8]) {
    for x in 0..mode.width {
        let px = match (mode.channels, mode.colour) {
            (3, Colour::Gbr) => (planes[y][2][x], planes[y][0][x], planes[y][1][x]),
            (3, Colour::Yuv) => yuv(planes[y][0][x], planes[y][2][x], planes[y][1][x]),
            (2, Colour::Yuv) => {
                // Robot 36 sends R-Y on even lines and B-Y on odd ones, so
                // each line borrows the other from its neighbour.
                let odd = y % 2;
                let a = planes[y.saturating_sub(odd.wrapping_sub(1) & 1)][1][x];
                let b = planes[y.saturating_sub(odd)][1][x];
                yuv(planes[y][0][x], a, b)
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
    },
}

/// How much audio to keep while hunting: enough for the header search window
/// plus the jump it steps by.
const HUNT_KEEP: f64 = HDR_SIZE + 0.5;

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
                        Some(at) => seq = self.base + at as u64,
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
        let State::Reading { mode, mut seq, mut line, mut planes } = taken else {
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
            if !read_line(
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
                complete = ending;
                break;
            }
            line += 1;
            // Robot 36 needs the next line's colour difference to convert
            // this one, so conversion runs one line behind it.
            let ready = match mode.alt_scan {
                true => line.saturating_sub(1),
                false => line,
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
            self.state = State::Reading { mode, seq, line, planes };
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

    #[test]
    fn black_is_1500_hz_and_white_is_2300() {
        assert_eq!(luma(1500.0), 0);
        assert_eq!(luma(2300.0), 255);
        assert_eq!(luma(1900.0), 128);
        // Anything outside the band is clamped rather than wrapped, so a sync
        // pulse read as a pixel is black and not white.
        assert_eq!(luma(1200.0), 0);
        assert_eq!(luma(2500.0), 255);
    }
}
