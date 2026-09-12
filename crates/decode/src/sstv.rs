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
pub fn decode(audio: &[f32], rate: f64) -> Option<Picture> {
    let mut meter = ToneMeter::new(rate);
    let header_end = find_header(audio, &mut meter)?;
    let vis = read_vis(audio, header_end, &mut meter)?;
    let mode = mode_of(vis)?;
    let start = header_end + (VIS_BIT * 9.0 * rate).round() as usize;
    Some(scan(audio, &mut meter, mode, start))
}

/// Walk the line timings, sampling one window per pixel.
fn scan(audio: &[f32], meter: &mut ToneMeter, mode: &'static Mode, start: usize) -> Picture {
    let rate = meter.rate();
    let (w, h) = (mode.width, mode.height);
    // Channel planes, as sent: the colour conversion comes after.
    let mut planes = vec![vec![vec![0u8; w]; mode.channels]; h];
    let mut done = 0usize;

    let mut seq = start;
    if mode.start_sync {
        match align_sync(audio, seq, mode, meter, false) {
            Some(s) => seq = s,
            None => return picture(mode, &planes, 0),
        }
    }

    'lines: for line in 0..h {
        if mode.sync_channel > 0 && line == 0 {
            // Scottie's sync sits inside the line, so the first line starts
            // before the pulse that was just found.
            let back = ((mode.offsets[mode.sync_channel] + mode.scan_time) * rate).round() as usize;
            seq = seq.saturating_sub(back);
        }
        for chan in 0..mode.channels {
            if chan == mode.sync_channel {
                if line > 0 || chan > 0 {
                    seq += (mode.line_time * rate).round() as usize;
                }
                match align_sync(audio, seq, mode, meter, true) {
                    Some(s) => seq = s,
                    None => break 'lines,
                }
            }
            let pixel_time =
                if mode.half_scan && chan > 0 { mode.half_pixel_time() } else { mode.pixel_time() };
            let half_window = pixel_time * mode.window_factor / 2.0;
            let window = (half_window * 2.0 * rate).round() as usize;
            for px in 0..w {
                let centre = mode.offsets[chan] + px as f64 * pixel_time - half_window;
                let at = (seq as f64 + centre * rate).round() as isize;
                if at < 0 || at as usize + window >= audio.len() {
                    break 'lines;
                }
                let at = at as usize;
                planes[line][chan][px] = luma(meter.peak_hz(&audio[at..at + window]));
            }
        }
        done = line + 1;
    }
    picture(mode, &planes, done)
}

/// Turn the channel planes into RGB, which is where a mode's colour order
/// and Robot 36's alternating colour difference are undone.
fn picture(mode: &'static Mode, planes: &[Vec<Vec<u8>>], lines: usize) -> Picture {
    let (w, h) = (mode.width, mode.height);
    let mut rgb = vec![0u8; w * h * 3];
    for y in 0..h {
        for x in 0..w {
            let px = match (mode.channels, mode.colour) {
                (3, Colour::Gbr) => (planes[y][2][x], planes[y][0][x], planes[y][1][x]),
                (3, Colour::Yuv) => yuv(planes[y][0][x], planes[y][2][x], planes[y][1][x]),
                (2, Colour::Yuv) => {
                    // Robot 36 sends R-Y on even lines and B-Y on odd ones,
                    // so each line borrows the other from its neighbour.
                    let odd = y % 2;
                    let a = planes[y.saturating_sub(odd.wrapping_sub(1) & 1)][1][x];
                    let b = planes[y.saturating_sub(odd)][1][x];
                    yuv(planes[y][0][x], a, b)
                }
                _ => (planes[y][0][x], planes[y][0][x], planes[y][0][x]),
            };
            let at = (y * w + x) * 3;
            rgb[at] = px.0;
            rgb[at + 1] = px.1;
            rgb[at + 2] = px.2;
        }
    }
    Picture { mode, width: w, height: h, rgb, lines }
}

/// The conversion every SSTV decoder uses, which is JPEG's YCbCr.
fn yuv(y: u8, cr: u8, cb: u8) -> (u8, u8, u8) {
    let (y, cb, cr) = (y as f32, cb as f32 - 128.0, cr as f32 - 128.0);
    let r = y + 1.402 * cr;
    let g = y - 0.344136 * cb - 0.714136 * cr;
    let b = y + 1.772 * cb;
    (r.clamp(0.0, 255.0) as u8, g.clamp(0.0, 255.0) as u8, b.clamp(0.0, 255.0) as u8)
}

/// A picture being received, fed audio as it arrives.
///
/// A transmission is two minutes long, so a node cannot wait for the end of
/// the stream and cannot keep the stream either. This keeps what it needs: a
/// few seconds while it is hunting for a header, and the picture's own audio
/// once it has found one.
pub struct Receiver {
    rate: f64,
    meter: ToneMeter,
    audio: Vec<f32>,
    /// Where in `audio` the picture starts, once the header has been read.
    start: Option<(usize, &'static Mode)>,
    /// Samples dropped off the front, so a caller can count in absolute time.
    dropped: u64,
    /// How many lines were published for the picture in progress, so a
    /// partial picture is only redrawn when it has grown.
    published: usize,
}

/// How much audio to keep while hunting: enough for the header search window
/// plus the jump it steps by.
const HUNT_KEEP: f64 = HDR_SIZE + 0.5;

/// How often a picture in progress is handed out, in lines. Every line would
/// be a full rescan of the picture's audio for one new row.
const PUBLISH_EVERY: usize = 16;

impl Receiver {
    pub fn new(rate: f64) -> Self {
        Self {
            rate,
            meter: ToneMeter::new(rate),
            audio: Vec::new(),
            start: None,
            dropped: 0,
            published: 0,
        }
    }

    pub fn reset(&mut self) {
        let rate = self.rate;
        *self = Self::new(rate);
    }

    /// Feed audio. Returns a picture whenever there is more of one to show:
    /// partly filled as it is received, and once more when it is complete.
    pub fn push(&mut self, audio: &[f32]) -> Option<Picture> {
        self.audio.extend_from_slice(audio);
        match self.start {
            None => self.hunt(),
            Some((at, mode)) => self.fill(at, mode),
        }
    }

    /// Whether a transmission is being received right now.
    pub fn receiving(&self) -> bool {
        self.start.is_some()
    }

    fn hunt(&mut self) -> Option<Picture> {
        let need = (HDR_SIZE * self.rate).round() as usize;
        if self.audio.len() < need {
            return None;
        }
        if let Some(end) = find_header(&self.audio, &mut self.meter) {
            // The VIS code is 240 ms behind the header, which on a live
            // stream has usually not arrived yet. Waiting for it is not
            // optional: reading it early fails, and treating that as a bad
            // header threw the header away and the picture with it.
            if self.audio.len() < end + (VIS_BIT * 8.0 * self.rate).round() as usize {
                return None;
            }
            if let Some(vis) = read_vis(&self.audio, end, &mut self.meter) {
                if let Some(mode) = mode_of(vis) {
                    let start = end + (VIS_BIT * 9.0 * self.rate).round() as usize;
                    self.start = Some((start, mode));
                    self.published = 0;
                    return None;
                }
            }
            // A header with a VIS this receiver cannot read is still a
            // header: skip past it rather than finding it again every block.
            self.audio.drain(..end);
            self.dropped += end as u64;
            return None;
        }
        let keep = (HUNT_KEEP * self.rate).round() as usize;
        if self.audio.len() > keep {
            let cut = self.audio.len() - keep;
            self.audio.drain(..cut);
            self.dropped += cut as u64;
        }
        None
    }

    fn fill(&mut self, at: usize, mode: &'static Mode) -> Option<Picture> {
        let have = self.audio.len().saturating_sub(at) as f64 / self.rate;
        let whole = mode.line_time * mode.height as f64;
        let lines = ((have / mode.line_time) as usize).min(mode.height);
        let complete = have >= whole;
        if !complete && lines < self.published + PUBLISH_EVERY {
            return None;
        }
        let picture = scan(&self.audio, &mut self.meter, mode, at);
        self.published = picture.lines;
        if complete {
            // Keep whatever came after the picture: two transmissions back to
            // back are two pictures, not one and a half.
            let used = at + (whole * self.rate).round() as usize;
            let used = used.min(self.audio.len());
            self.audio.drain(..used);
            self.dropped += used as u64;
            self.start = None;
            self.published = 0;
        }
        Some(picture)
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
