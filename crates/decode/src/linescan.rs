//! A picture that arrives one line at a time.
//!
//! What APT and weather fax have in common once the modulation is off: a
//! width in pixels, a line rate, and rows that have to be painted into a
//! canvas as they are read and handed to a viewer in batches. Neither
//! transmission has an end anybody can wait for, since a satellite pass is
//! fifteen minutes and a fax chart ten, so the canvas is a fixed height and a
//! picture that fills it starts another.
//!
//! One byte a pixel, because both are grey. Colour belongs to the decoder
//! that has any, which is why SSTV keeps its own assembly.

/// A pixel's brightness from its tone, which is the scale every picture sent
/// as an audio tone uses: 1500 Hz is black and 2300 Hz white.
///
/// SSTV, weather fax and the tone-keyed modes all send this scale, so a
/// decoder reading frequencies maps them here rather than keeping its own
/// copy of the two numbers.
pub fn luma(hz: f64) -> u8 {
    let v = ((hz - 1500.0) / 3.1372549).round();
    v.clamp(0.0, 255.0) as u8
}

/// Where a line starts, and how sure of it the decoder is.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Hit {
    /// Index into the signal the marker was searched in.
    pub at: usize,
    /// A normalised correlation: 1.0 is the marker itself, 0.0 anything
    /// uncorrelated.
    pub score: f32,
}

/// The thing at the head of a line that says where the line starts: an APT
/// sync run, a weather fax phasing pulse.
///
/// Both are a fixed shape once a line at a known rate, so both are the same
/// search: a normalised correlation with the mean taken out, so that how
/// bright the line is does not decide whether its marker was found. Samples
/// the template leaves at zero say nothing about where the line is and are
/// left out of the sum.
pub struct Marker {
    template: Vec<f32>,
    energy: f32,
}

impl Marker {
    pub fn new(template: Vec<f32>) -> Self {
        let energy = template.iter().map(|t| t * t).sum::<f32>().sqrt();
        Self { template, energy }
    }

    pub fn len(&self) -> usize {
        self.template.len()
    }

    pub fn is_empty(&self) -> bool {
        self.template.is_empty()
    }

    /// The template itself, for a caller that wants to know which samples it
    /// sent high and which low.
    pub fn template(&self) -> &[f32] {
        &self.template
    }

    /// How well the signal at `at` matches.
    pub fn score(&self, signal: &[f32], at: usize) -> f32 {
        let Some(window) = signal.get(at..at + self.template.len()) else { return 0.0 };
        let used = self.template.iter().filter(|t| **t != 0.0).count().max(1) as f32;
        let mean = window
            .iter()
            .zip(&self.template)
            .filter(|(_, t)| **t != 0.0)
            .map(|(x, _)| *x)
            .sum::<f32>()
            / used;
        let mut dot = 0.0;
        let mut power = 0.0;
        for (x, t) in window.iter().zip(&self.template) {
            if *t == 0.0 {
                continue;
            }
            let v = x - mean;
            dot += v * t;
            power += v * v;
        }
        match power > 0.0 {
            true => dot / (power.sqrt() * self.energy),
            false => 0.0,
        }
    }

    /// The best match anywhere in `from..to`.
    pub fn best(&self, signal: &[f32], from: usize, to: usize) -> Hit {
        let mut best = Hit { at: from, score: 0.0 };
        for at in from..to {
            let score = self.score(signal, at);
            if score > best.score {
                best = Hit { at, score };
            }
        }
        best
    }
}

/// The rows of one picture that a batch of audio produced.
#[derive(Clone, Debug)]
pub struct Rows {
    /// Which picture these belong to, counted from the assembler's first.
    pub picture: u64,
    pub width: usize,
    pub height: usize,
    /// The row the batch starts at.
    pub first: usize,
    /// The rows themselves, one byte a pixel, row major.
    pub gray: Vec<u8>,
    /// Whether the picture is finished: the canvas full, or the decoder
    /// giving up on a transmission that stopped.
    pub complete: bool,
}

impl Rows {
    pub fn lines(&self) -> usize {
        match self.width {
            0 => 0,
            w => self.gray.len() / w,
        }
    }
}

/// A picture as it stands, taken out of an assembler.
#[derive(Clone, Debug)]
pub struct Picture {
    pub width: usize,
    pub height: usize,
    /// One byte a pixel, row major. Rows past `lines` are black.
    pub gray: Vec<u8>,
    /// How many rows were received, out of `height`.
    pub lines: usize,
}

/// A canvas being filled a row at a time.
pub struct Assembler {
    width: usize,
    height: usize,
    gray: Vec<u8>,
    lines: usize,
    pictures: u64,
    batch: Vec<u8>,
    batch_first: usize,
}

impl Assembler {
    pub fn new(width: usize, height: usize) -> Self {
        Self {
            width,
            height,
            gray: vec![0; width * height],
            lines: 0,
            pictures: 0,
            batch: Vec::new(),
            batch_first: 0,
        }
    }

    pub fn width(&self) -> usize {
        self.width
    }

    pub fn height(&self) -> usize {
        self.height
    }

    /// Rows painted into the picture being received.
    pub fn lines(&self) -> usize {
        self.lines
    }

    /// Pictures started, which names the one being painted.
    pub fn sequence(&self) -> u64 {
        self.pictures
    }

    /// The canvas as it stands, whatever of it has arrived.
    pub fn canvas(&self) -> &[u8] {
        &self.gray
    }

    /// A copy of the picture so far, for a caller that wants the whole thing
    /// rather than the rows as they arrive.
    pub fn picture(&self) -> Picture {
        Picture {
            width: self.width,
            height: self.height,
            gray: self.gray.clone(),
            lines: self.lines,
        }
    }

    /// Begin a picture. Everything held for the last one is dropped, so a
    /// caller wanting the rows takes them first.
    pub fn start(&mut self) {
        self.gray.iter_mut().for_each(|p| *p = 0);
        self.lines = 0;
        self.pictures += 1;
        self.batch.clear();
        self.batch_first = 0;
    }

    /// Paint a row. Returns whether the canvas is now full, which is a
    /// picture to hand over and another to start.
    pub fn row(&mut self, row: &[u8]) -> bool {
        if self.lines >= self.height || row.len() != self.width {
            return self.lines >= self.height;
        }
        let at = self.lines * self.width;
        self.gray[at..at + self.width].copy_from_slice(row);
        if self.batch.is_empty() {
            self.batch_first = self.lines;
        }
        self.batch.extend_from_slice(row);
        self.lines += 1;
        self.lines >= self.height
    }

    /// The rows painted since the last take, or nothing where none were and
    /// the picture is not over.
    pub fn take(&mut self, complete: bool) -> Option<Rows> {
        if self.batch.is_empty() && !complete {
            return None;
        }
        Some(Rows {
            picture: self.pictures,
            width: self.width,
            height: self.height,
            first: self.batch_first,
            gray: std::mem::take(&mut self.batch),
            complete,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn black_is_1500_hz_and_white_is_2300() {
        assert_eq!(luma(1500.0), 0);
        assert_eq!(luma(2300.0), 255);
        assert_eq!(luma(1900.0), 128);
        // Outside the band is clamped rather than wrapped, so a sync pulse
        // read as a pixel is black and not white.
        assert_eq!(luma(1200.0), 0);
        assert_eq!(luma(2500.0), 255);
    }

    /// A marker is found where it was put, and a signal with none in it
    /// scores far below one that has.
    #[test]
    fn a_marker_is_found_where_it_was_put() {
        let m = Marker::new(vec![-1.0, -1.0, 1.0, 1.0, 0.0, 0.0]);
        let mut signal = vec![0.3f32; 40];
        for (i, v) in [0.1, 0.1, 0.9, 0.9, 0.3, 0.3].iter().enumerate() {
            signal[17 + i] = *v;
        }
        let hit = m.best(&signal, 0, 30);
        assert_eq!(hit.at, 17);
        assert!(hit.score > 0.99, "scored {}", hit.score);
        // The brightness of the line does not decide it: the same shape a
        // tenth as deep scores the same.
        for (i, v) in [0.28, 0.28, 0.32, 0.32, 0.3, 0.3].iter().enumerate() {
            signal[17 + i] = *v;
        }
        assert!(m.best(&signal, 0, 30).score > 0.99);
        assert!(m.best(&[0.3f32; 40], 0, 30).score < 0.01, "flat signal matched");
    }

    /// Rows are painted where they belong and handed over once each: a
    /// viewer that is given the same row twice paints a picture that jumps.
    #[test]
    fn rows_are_handed_over_once_and_in_order() {
        let mut a = Assembler::new(4, 3);
        a.start();
        assert!(!a.row(&[1, 1, 1, 1]));
        let first = a.take(false).expect("a row was painted");
        assert_eq!(first.first, 0);
        assert_eq!(first.lines(), 1);
        assert_eq!(first.picture, 1);
        assert!(a.take(false).is_none(), "nothing new to hand over");
        assert!(!a.row(&[2, 2, 2, 2]));
        assert!(a.row(&[3, 3, 3, 3]), "the canvas is full at three rows");
        let rest = a.take(true).expect("two more rows");
        assert_eq!(rest.first, 1);
        assert_eq!(rest.lines(), 2);
        assert!(rest.complete);
        assert_eq!(a.canvas(), &[1, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3]);
    }

    /// A full canvas takes no more rows, and the next picture starts from an
    /// empty one with a number of its own.
    #[test]
    fn a_full_canvas_starts_another_picture() {
        let mut a = Assembler::new(2, 1);
        a.start();
        assert!(a.row(&[9, 9]));
        assert!(a.row(&[8, 8]), "the canvas is full");
        assert_eq!(a.lines(), 1);
        assert_eq!(a.canvas(), &[9, 9]);
        a.start();
        assert_eq!(a.sequence(), 2);
        assert_eq!(a.lines(), 0);
        assert_eq!(a.canvas(), &[0, 0]);
    }
}
