//! Weather fax: the charts the meteorological and marine services send on
//! shortwave, and the pictures a few of them still send on VHF.
//!
//! A sideband receiver hears a tone that swings 400 Hz either side of 1900:
//! 1500 Hz is black and 2300 Hz white, the same scale SSTV uses. A chart is
//! sent as a start tone, then half a minute of phasing, then the picture, a
//! line at a time, and finally a stop tone. There is no sync in the picture
//! at all: the phasing signal is the only thing that says where a line
//! starts, and everything after it is the receiver's own clock.
//!
//! So the decoder is the phasing search and a ruler. A chart is ten to
//! twenty minutes long, and an operator watches it fill.
//!
//! Rates and the index of cooperation are from WMO-No. 386, the Manual on
//! the Global Telecommunication System, volume I part II attachment II-4.

use crate::linescan::{Assembler, Hit, Marker, Picture, Rows, luma};
use dsp::subcarrier::Frequency;

/// The tone the picture rides on and the shift either side of it.
pub const CENTRE_HZ: f64 = 1_900.0;
pub const SHIFT_HZ: f64 = 400.0;
/// What the discriminator is allowed to see. Wider than the shift, so a
/// transmitter tuned a little off still reads rather than clipping.
const BAND_HZ: f64 = 1_600.0;

/// How many lines a minute a transmission sends.
///
/// The four rates the manual allows. Which one is in use is not announced in
/// anything the decoder can read before the picture, so it is measured: the
/// phasing pulses come once a line, and the gap between two of them names
/// the rate.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Lpm {
    /// Slow, for HF charts sent where the path is poor.
    Sixty,
    Ninety,
    /// What almost every marine and meteorological chart uses.
    OneTwenty,
    /// Fast, for satellite pictures relayed over VHF.
    OneEighty,
}

impl Lpm {
    pub const ALL: [Lpm; 4] = [Lpm::Sixty, Lpm::Ninety, Lpm::OneTwenty, Lpm::OneEighty];

    pub fn per_minute(self) -> f64 {
        match self {
            Lpm::Sixty => 60.0,
            Lpm::Ninety => 90.0,
            Lpm::OneTwenty => 120.0,
            Lpm::OneEighty => 180.0,
        }
    }

    pub fn line_time(self) -> f64 {
        60.0 / self.per_minute()
    }

    pub fn label(self) -> &'static str {
        match self {
            Lpm::Sixty => "60 lpm",
            Lpm::Ninety => "90 lpm",
            Lpm::OneTwenty => "120 lpm",
            Lpm::OneEighty => "180 lpm",
        }
    }

    /// The rate whose line is closest to `seconds`, where one is within a
    /// twentieth of it. A gap that is not one of the four rates is two
    /// phasing pulses that were not a line apart.
    pub fn nearest(seconds: f64) -> Option<Lpm> {
        Lpm::ALL.into_iter().find(|l| (l.line_time() - seconds).abs() / l.line_time() < 0.05)
    }
}

/// The index of cooperation, which with the line rate fixes how wide a line
/// is: the drum's circumference over its pitch, so the width in pixels is
/// pi times this.
///
/// 576 is what the marine and meteorological broadcasts use. 288 exists, is
/// announced by a start tone of 675 Hz rather than 300, and is not read here
/// because nothing on the air sends it.
pub const IOC: f64 = 576.0;

/// Pixels across a line, which is pi times the index of cooperation.
pub const WIDTH: usize = 1_809;

/// How tall a picture is before another is started. A chart is ten to twenty
/// minutes; at 120 lines a minute this is a shade over sixteen.
pub const PICTURE_LINES: usize = 2_000;

/// How much of a line the phasing pulse is white for. The manual puts it at
/// 5%, at the start of the line, with the rest black.
const PHASE_PULSE: f64 = 0.05;

/// How well the phasing pulse has to match before it is believed, as a
/// normalised correlation.
///
/// Measured on a synthesised chart at 120 lines a minute: a clean phasing
/// line scores 1.00, and with noise swinging the tone 200 Hz, a quarter of
/// the shift, it still scores 1.00, since the pulse is half the line's
/// energy. The best the same search reached against a picture with no
/// phasing in front of it was 0.50.
const LOCK: f32 = 0.75;

/// Lines whose tone is not a picture tone before the chart is taken to have
/// ended. Fifteen at 120 lines a minute is seven seconds, which is longer
/// than a fade on shortwave and shorter than the stop tone.
const LOST_LINES: usize = 15;

/// Tones a picture is sent in, with the room a signal tuned a little off
/// needs. Outside this the transmitter has gone rather than sent a shade.
const PICTURE_LOW_HZ: f64 = 1_350.0;
const PICTURE_HIGH_HZ: f64 = 2_450.0;

/// The phasing pulse as a template: white for a twentieth of the line, black
/// for the rest of it.
///
/// Weighted by how much of the line each part is, rather than plus and minus
/// one: the pulse is a twentieth of the line, so a template that swings
/// equally either side of nothing scores a perfect phasing line 0.44 and
/// leaves no room above the 0.62 a picture with no phasing in it reaches.
/// Against the mean-removed pulse itself the same line scores 1.0.
fn phase_marker(line_samples: f64) -> Marker {
    let len = line_samples.round() as usize;
    let pulse = (line_samples * PHASE_PULSE).round() as usize;
    let duty = pulse as f32 / len as f32;
    Marker::new((0..len).map(|i| if i < pulse { 1.0 - duty } else { -duty }).collect())
}

/// What the phasing search is run at before it is refined.
///
/// The template is a whole line long and the search walks a whole line, so at
/// 44.1 kHz an unaided search of a 120 lpm line is 22 thousand positions of a
/// 22 thousand sample correlation: 480 million multiplies to find one pulse,
/// and four rates to try. Averaged down to 400 Hz first it is 55 thousand,
/// and the answer is then refined at the full rate. The pulse is a twentieth
/// of a line, which is seven coarse samples at the fastest rate, so it
/// survives the averaging.
const COARSE_HZ: f64 = 400.0;

/// `signal` averaged down by `by`, which is how the phasing search affords to
/// walk a whole line.
fn coarsen(signal: &[f32], by: usize) -> Vec<f32> {
    signal.chunks(by).map(|c| c.iter().sum::<f32>() / c.len() as f32).collect()
}

/// A chart being received, fed audio as it arrives.
pub struct Receiver {
    rate: f64,
    freq: Frequency,
    /// The tone of every sample, in hertz, and the brightness that is.
    hz: Vec<f64>,
    shade: Vec<f32>,
    /// Absolute position of `shade[0]`.
    base: u64,
    state: State,
    asm: Assembler,
    lpm: Option<Lpm>,
}

enum State {
    /// Looking for a phasing pulse, and then for the one a line after it.
    Hunting,
    Reading {
        seq: f64,
        lost: usize,
    },
}

/// How much to keep while hunting: two of the longest line, which is what
/// measuring a line rate from two pulses needs.
const HUNT_KEEP: f64 = 2.5;

impl Receiver {
    pub fn new(rate: f64) -> Self {
        Self {
            rate,
            freq: Frequency::new(rate, CENTRE_HZ, BAND_HZ),
            hz: Vec::new(),
            shade: Vec::new(),
            base: 0,
            state: State::Hunting,
            asm: Assembler::new(WIDTH, PICTURE_LINES),
            lpm: None,
        }
    }

    pub fn reset(&mut self) {
        let rate = self.rate;
        *self = Self::new(rate);
    }

    pub fn receiving(&self) -> bool {
        matches!(self.state, State::Reading { .. })
    }

    /// The rate the transmission was found to be sending at, once it has.
    pub fn lpm(&self) -> Option<Lpm> {
        self.lpm
    }

    pub fn picture(&self) -> Picture {
        self.asm.picture()
    }

    pub fn push(&mut self, audio: &[f32]) -> Option<Rows> {
        self.hz.clear();
        self.freq.process(audio, &mut self.hz);
        self.shade.extend(self.hz.iter().map(|hz| luma(*hz) as f32));
        if matches!(self.state, State::Hunting) {
            self.hunt();
        }
        self.advance(false)
    }

    pub fn finish(&mut self) -> Option<Rows> {
        self.advance(true)
    }

    fn drain_to(&mut self, abs: u64) {
        let Some(cut) = abs.checked_sub(self.base) else { return };
        let cut = (cut as usize).min(self.shade.len());
        self.shade.drain(..cut);
        self.base += cut as u64;
    }

    /// Find the phasing signal, which is the only thing in a transmission
    /// that says where a line starts. A pulse on its own is not evidence:
    /// what is looked for is one, then another a whole line later at one of
    /// the four rates, which names the rate as well as the phase.
    fn hunt(&mut self) {
        let longest = (Lpm::Sixty.line_time() * self.rate) as usize;
        if self.shade.len() < 2 * longest + longest / 10 {
            return;
        }
        let by = (self.rate / COARSE_HZ).round().max(1.0) as usize;
        let coarse = coarsen(&self.shade, by);
        let mut lock: Option<(Lpm, Hit)> = None;
        for lpm in Lpm::ALL {
            let line = lpm.line_time() * self.rate / by as f64;
            let marker = phase_marker(line);
            let first = marker.best(&coarse, 0, line.round() as usize);
            if first.score < LOCK {
                continue;
            }
            let next = first.at + line.round() as usize;
            // A hundredth of a line either side: two pulses that are not a
            // line apart to within that are two different rates.
            let slack = (line * 0.01).round().max(1.0) as usize;
            let then = marker.best(&coarse, next.saturating_sub(slack), next + slack);
            if then.score >= LOCK && lock.is_none_or(|(_, b)| then.score > b.score) {
                // Where the pulse is to the sample, which the coarse search
                // only knows to within its own step.
                let fine = phase_marker(lpm.line_time() * self.rate);
                let at = first.at * by;
                let hit = fine.best(&self.shade, at.saturating_sub(by), at + by);
                lock = Some((lpm, hit));
            }
        }
        if let Some((lpm, first)) = lock {
            self.lpm = Some(lpm);
            self.asm.start();
            self.state = State::Reading { seq: (self.base + first.at as u64) as f64, lost: 0 };
            return;
        }
        let keep = (HUNT_KEEP * self.rate) as usize;
        if self.shade.len() > keep {
            self.drain_to(self.base + (self.shade.len() - keep) as u64);
        }
    }

    /// The pixels of the line starting at sample `at`, each the mean shade
    /// across its span. Returns how many read a tone no picture sends, which
    /// is how a transmission that has stopped is told from one still going.
    fn read_line(&self, at: f64, per_pixel: f64, row: &mut [u8]) -> usize {
        let mut adrift = 0;
        for (x, px) in row.iter_mut().enumerate() {
            let from = (at + x as f64 * per_pixel).round() as usize;
            let to = (at + (x + 1) as f64 * per_pixel).round() as usize;
            let span = &self.hz_window(from, to);
            let mean = match span.1 {
                0 => {
                    adrift += 1;
                    continue;
                }
                n => span.0 / n as f64,
            };
            if !(PICTURE_LOW_HZ..=PICTURE_HIGH_HZ).contains(&mean) {
                adrift += 1;
            }
            *px = luma(mean);
        }
        adrift
    }

    /// The mean tone across a span of samples, as a sum and a count, taken
    /// off the shade rather than the frequency because the shade is what is
    /// kept: the scale is linear in hertz, so the two are the same average.
    fn hz_window(&self, from: usize, to: usize) -> (f64, usize) {
        let span = &self.shade[from.min(self.shade.len())..to.min(self.shade.len())];
        let sum: f64 = span.iter().map(|s| 1500.0 + *s as f64 * 3.1372549).sum();
        (sum, span.len())
    }

    fn advance(&mut self, ending: bool) -> Option<Rows> {
        let State::Reading { mut seq, mut lost } = self.state else { return None };
        let lpm = self.lpm?;
        let line = lpm.line_time() * self.rate;
        let per_pixel = line / WIDTH as f64;
        let mut complete = false;
        let mut row = vec![0u8; WIDTH];

        loop {
            let here = seq - self.base as f64;
            if here < 0.0 {
                break;
            }
            let here = here as usize;
            if self.shade.len() < here + line.round() as usize {
                complete = ending;
                break;
            }
            let adrift = self.read_line(here as f64, per_pixel, &mut row);
            // Half a line of tones no picture sends is not a line: what is
            // there is noise, or the transmitter has stopped.
            match adrift * 2 > WIDTH {
                true => lost += 1,
                false => {
                    lost = 0;
                    if self.asm.row(&row) {
                        complete = true;
                    }
                }
            }
            seq += line;
            self.drain_to(seq as u64);
            if complete || lost >= LOST_LINES {
                complete = true;
                break;
            }
        }

        match complete {
            true => self.state = State::Hunting,
            false => self.state = State::Reading { seq, lost },
        }
        self.asm.take(complete)
    }
}

/// Decode everything in `audio`, a whole recording at a time.
pub fn decode(audio: &[f32], rate: f64) -> Picture {
    let mut rx = Receiver::new(rate);
    rx.push(audio);
    rx.finish();
    rx.picture()
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::TAU;

    /// What a sideband receiver hands over: 44.1 kHz, as the SSTV chain
    /// already produces.
    const RATE: f64 = 44_100.0;

    /// A chart, as audio: phasing lines, then a picture whose left half is a
    /// ramp and whose right half is a grey step wedge.
    fn chart(lpm: Lpm, phasing: usize, lines: usize, noise: f64, seed: u64) -> Vec<f32> {
        let line = lpm.line_time() * RATE;
        let per_pixel = line / WIDTH as f64;
        let total = ((phasing + lines) as f64 * line) as usize;
        let mut rng = seed.max(1);
        let mut next = || {
            rng ^= rng << 13;
            rng ^= rng >> 7;
            rng ^= rng << 17;
            (rng >> 11) as f64 / (1u64 << 53) as f64 * 2.0 - 1.0
        };
        let mut phase = 0.0f64;
        (0..total)
            .map(|i| {
                let y = (i as f64 / line) as usize;
                let x = ((i as f64 % line) / per_pixel) as usize;
                let value = match y < phasing {
                    // Phasing: white for a twentieth of the line, black for
                    // the rest.
                    true => match (x as f64) < WIDTH as f64 * PHASE_PULSE {
                        true => 255.0,
                        false => 0.0,
                    },
                    false => scene(y - phasing, x) as f64,
                };
                let hz = 1500.0 + value / 255.0 * 800.0 + noise * next() * 200.0;
                phase += TAU * hz / RATE;
                phase.sin() as f32
            })
            .collect()
    }

    /// The picture: a ramp across the left half, a six step wedge across the
    /// right, so a line read at the wrong width or the wrong offset shows.
    fn scene(line: usize, x: usize) -> u8 {
        match x < WIDTH / 2 {
            true => (x * 255 / (WIDTH / 2)) as u8,
            false => {
                let step = (x - WIDTH / 2) * 6 / (WIDTH - WIDTH / 2);
                let shade = (step * 255 / 5) as u8;
                match line % 64 < 4 {
                    // A black rule every 64 lines, which says whether the
                    // clock has slipped a line over the picture.
                    true => 0,
                    false => shade,
                }
            }
        }
    }

    /// The four rates the manual allows, and the widths they give.
    #[test]
    fn the_rates_are_the_four_the_manual_allows() {
        assert_eq!(Lpm::OneTwenty.line_time(), 0.5);
        assert_eq!(Lpm::Sixty.line_time(), 1.0);
        assert!((Lpm::Ninety.line_time() - 2.0 / 3.0).abs() < 1e-12);
        assert_eq!(Lpm::nearest(0.5), Some(Lpm::OneTwenty));
        assert_eq!(Lpm::nearest(0.34), Some(Lpm::OneEighty));
        assert_eq!(Lpm::nearest(0.42), None, "a gap that is no rate at all");
        // The index of cooperation times pi, to the pixel.
        assert_eq!((IOC * std::f64::consts::PI).round() as usize, WIDTH + 1);
    }

    /// A chart at 120 lines a minute: the rate is measured off the phasing
    /// signal, the picture starts where the phasing ends and the ramp and
    /// the wedge come back where they were sent.
    #[test]
    fn a_chart_is_read_at_the_rate_its_phasing_gives() {
        let audio = chart(Lpm::OneTwenty, 6, 40, 0.0, 1);
        let mut rx = Receiver::new(RATE);
        rx.push(&audio);
        rx.finish();
        assert_eq!(rx.lpm(), Some(Lpm::OneTwenty));
        let pic = rx.picture();
        assert_eq!(pic.width, WIDTH);
        // The six phasing lines are painted too, since they are part of the
        // transmission; the last line has no room behind it.
        assert_eq!(pic.lines, 45, "lines read out of 46 sent");

        let row = |y: usize| &pic.gray[y * WIDTH..(y + 1) * WIDTH];
        for y in [10, 25, 40] {
            let r = row(y);
            for (x, want) in [(200usize, 56u8), (800, 225)] {
                let got = r[x];
                assert!(got.abs_diff(want) <= 4, "line {y} pixel {x} read {got}, sent {want}");
            }
            // The step wedge: six levels and nothing between them.
            let step = r[WIDTH / 2 + 700] as i32;
            assert!((step - 204).abs() <= 6, "the fifth step read {step}, sent 204");
        }
    }

    /// At 60 lines a minute the same chart reads, and the decoder says so:
    /// the rate is measured and not assumed.
    #[test]
    fn a_slower_chart_names_its_own_rate() {
        let audio = chart(Lpm::Sixty, 4, 12, 0.0, 2);
        let mut rx = Receiver::new(RATE);
        rx.push(&audio);
        rx.finish();
        assert_eq!(rx.lpm(), Some(Lpm::Sixty));
        let pic = rx.picture();
        assert_eq!(pic.lines, 15);
        let r = &pic.gray[10 * WIDTH..11 * WIDTH];
        assert!(r[200].abs_diff(56) <= 4, "the ramp read {}", r[200]);
    }

    /// Noise that swings the tone 200 Hz is a fifth of the shift, and the
    /// chart still reads: every line, and the ramp within twenty counts of
    /// what was sent.
    #[test]
    fn a_noisy_chart_still_reads_every_line() {
        let audio = chart(Lpm::OneTwenty, 6, 40, 1.0, 3);
        let mut rx = Receiver::new(RATE);
        rx.push(&audio);
        rx.finish();
        assert_eq!(rx.lpm(), Some(Lpm::OneTwenty));
        let pic = rx.picture();
        assert_eq!(pic.lines, 45);
        let r = &pic.gray[25 * WIDTH..26 * WIDTH];
        let got = r[800] as i32;
        assert!((got - 225).abs() < 20, "the ramp read {got} under noise, sent 225");
    }

    /// Noise is not a chart: five minutes of it produce no lines, because a
    /// phasing pulse has to be found twice a line apart before anything is
    /// painted.
    #[test]
    fn minutes_of_noise_produce_no_lines() {
        let n = (300.0 * RATE) as usize;
        let mut rng = 0xfaceu64;
        let noise: Vec<f32> = (0..n)
            .map(|_| {
                rng ^= rng << 13;
                rng ^= rng >> 7;
                rng ^= rng << 17;
                ((rng >> 11) as f64 / (1u64 << 53) as f64 * 2.0 - 1.0) as f32
            })
            .collect();
        let pic = decode(&noise, RATE);
        assert_eq!(pic.lines, 0, "noise painted lines");
    }

    /// A picture with no phasing signal in front of it is not read: there is
    /// nothing in a fax line that says where it starts, so a decoder that
    /// locked onto the picture would be inventing the phase.
    #[test]
    fn a_picture_without_phasing_is_not_read() {
        let audio = chart(Lpm::OneTwenty, 0, 40, 0.0, 4);
        let pic = decode(&audio, RATE);
        assert_eq!(pic.lines, 0);
    }
}
