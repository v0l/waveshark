//! Two-tone sequential paging: an A tone, a B tone, and a pager opens.
//!
//! Runs of steady tone in ([`dsp::tone::Run`]), a page out. Nothing here
//! measures audio; what arrives has already been decided to be a tone held
//! for a while, which is all a two-tone scheme is.
//!
//! Motorola Quick Call II sends the A tone for a second and the B tone for
//! three, and the European schemes differ in the timings rather than in the
//! idea. A single long tone is a group call: everybody on that tone opens.
//!
//! The tones themselves are the address. Which pager they belong to is
//! operator knowledge, since the pairs are assigned locally and nothing on
//! the air says whose they are, so a name comes from a list the operator
//! supplies ([`Pagers`]) and a page with no match still reports its pair.

use common::Decoded;
use dsp::tone::Run;

/// What was sent: a pair addressed to one pager, or a long tone addressed to
/// everybody on it.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Page {
    /// The A tone then the B tone, with how long each was held.
    Pair { a_hz: f64, a_s: f64, b_hz: f64, b_s: f64, at_s: f64 },
    /// One tone held long enough to be a group call rather than an A tone
    /// whose B never came.
    Group { hz: f64, seconds: f64, at_s: f64 },
}

impl Page {
    /// The tones, as a person writes a pair on a programming sheet.
    pub fn tones(&self) -> String {
        match self {
            Page::Pair { a_hz, b_hz, .. } => format!("{a_hz:.1}/{b_hz:.1}"),
            Page::Group { hz, .. } => format!("{hz:.1}"),
        }
    }

    pub fn at_s(&self) -> f64 {
        match self {
            Page::Pair { at_s, .. } | Page::Group { at_s, .. } => *at_s,
        }
    }
}

/// The timings a page has to fit. Defaults are Quick Call II with the slack
/// the European variants need: a 1 s A tone and a 3 s B tone sit well inside
/// them, and so does a 0.8/2.8 s pair.
#[derive(Clone, Copy, Debug)]
pub struct Limits {
    pub a_s: (f64, f64),
    pub b_s: (f64, f64),
    /// A tone held at least this long with nothing after it is a group call.
    pub group_s: f64,
    /// The most silence allowed between the two tones. A transmitter sends
    /// them back to back; this is the gap the detector's own windows and a
    /// fade can open.
    pub gap_s: f64,
    /// How far apart the two tones of a pair have to be, as a fraction. The
    /// closest pair in a Motorola group is about 6% apart, and anything
    /// nearer than this is one tone that wandered rather than two.
    pub separation: f64,
}

impl Default for Limits {
    fn default() -> Self {
        Self { a_s: (0.4, 2.0), b_s: (0.8, 8.0), group_s: 4.0, gap_s: 0.35, separation: 0.03 }
    }
}

/// Runs in, pages out.
pub struct Sequential {
    limits: Limits,
    /// The run that could still be an A tone.
    held: Option<Run>,
}

impl Default for Sequential {
    fn default() -> Self {
        Self::new(Limits::default())
    }
}

impl Sequential {
    pub fn new(limits: Limits) -> Self {
        Self { limits, held: None }
    }

    pub fn reset(&mut self) {
        self.held = None;
    }

    /// One run of steady tone. `Some` where it completed a page.
    pub fn run(&mut self, run: Run) -> Option<Page> {
        let l = self.limits;
        let held = self.held.take();

        // A long tone on its own is a group call, and is not held back to
        // see whether a B tone follows: nothing follows a group call.
        if run.seconds >= l.group_s {
            return Some(Page::Group { hz: run.hz, seconds: run.seconds, at_s: run.start_s });
        }

        if let Some(a) = held {
            let gap = run.start_s - (a.start_s + a.seconds);
            let apart = (run.hz - a.hz).abs() / a.hz.min(run.hz);
            let fits = gap <= l.gap_s
                && (l.b_s.0..=l.b_s.1).contains(&run.seconds)
                && apart >= l.separation;
            if fits {
                return Some(Page::Pair {
                    a_hz: a.hz,
                    a_s: a.seconds,
                    b_hz: run.hz,
                    b_s: run.seconds,
                    at_s: a.start_s,
                });
            }
        }

        // Whatever it was, it can still be the A tone of the next page.
        if (l.a_s.0..=l.a_s.1).contains(&run.seconds) {
            self.held = Some(run);
        }
        None
    }
}

/// One pager the operator knows about: what to call it and the pair that
/// opens it.
#[derive(Clone, Debug, PartialEq)]
pub struct Pager {
    pub name: String,
    pub a_hz: f64,
    pub b_hz: f64,
}

/// The operator's list of pagers, because nothing on the air says whose
/// tones these are.
///
/// One per line, `name = A/B` in hertz, `#` to the end of a line ignored:
///
/// ```text
/// Station 3 pagers = 947.3/332.5
/// Fire brigade     = 1122.5/1153.4   # all call is 1153.4 alone
/// ```
#[derive(Clone, Debug, Default)]
pub struct Pagers {
    list: Vec<Pager>,
    /// How far a heard tone may be from a listed one and still match it, as
    /// a fraction. A transmitter is within a tenth of a percent; the reading
    /// is within a few hertz on a 25 ms window, which at 300 Hz is 1%.
    tolerance: f64,
}

impl Pagers {
    pub fn parse(text: &str) -> Self {
        let mut list = Vec::new();
        for line in text.lines() {
            let line = line.split('#').next().unwrap_or("").trim();
            let Some((name, tones)) = line.split_once('=') else { continue };
            let Some((a, b)) = tones.split_once('/') else { continue };
            let (Ok(a_hz), Ok(b_hz)) = (a.trim().parse::<f64>(), b.trim().parse::<f64>()) else {
                continue;
            };
            if a_hz <= 0.0 || b_hz <= 0.0 {
                continue;
            }
            list.push(Pager { name: name.trim().to_string(), a_hz, b_hz });
        }
        Self { list, tolerance: 0.015 }
    }

    pub fn len(&self) -> usize {
        self.list.len()
    }

    pub fn is_empty(&self) -> bool {
        self.list.is_empty()
    }

    /// Whom a page was addressed to, where the operator said.
    pub fn who(&self, page: &Page) -> Option<&Pager> {
        let near = |heard: f64, listed: f64| (heard - listed).abs() <= listed * self.tolerance;
        self.list.iter().find(|p| match page {
            Page::Pair { a_hz, b_hz, .. } => near(*a_hz, p.a_hz) && near(*b_hz, p.b_hz),
            // A group call is the B tone of a pair on its own, which is how
            // an all-call to a fleet is sent.
            Page::Group { hz, .. } => near(*hz, p.b_hz),
        })
    }
}

/// One row: which tones, how long each was held, and whose pager that is
/// where the operator said.
pub fn decoded(bytes: &[u8], center: common::Hz) -> Option<Decoded> {
    if bytes.len() <= TAG.len() || bytes[..TAG.len()] != TAG {
        return None;
    }
    let body = String::from_utf8_lossy(&bytes[TAG.len()..]).to_string();
    let (kind, rest) = body.split_once(' ')?;
    // The name is whatever is left after the numbers, spaces and all.
    let mut words = match kind {
        "pair" => rest.splitn(5, ' '),
        _ => rest.splitn(3, ' '),
    };
    let num = |w: Option<&str>| w.and_then(|w| w.parse::<f64>().ok());
    let mut fields: Vec<(String, common::Value)> = Vec::new();
    let (tones, seconds, name) = match kind {
        "pair" => {
            let (a_hz, a_s, b_hz, b_s) =
                (num(words.next())?, num(words.next())?, num(words.next())?, num(words.next())?);
            fields.push(("tone_a_hz".into(), common::Value::Float(a_hz)));
            fields.push(("tone_b_hz".into(), common::Value::Float(b_hz)));
            fields.push(("tone_a_s".into(), common::Value::Float(a_s)));
            fields.push(("tone_b_s".into(), common::Value::Float(b_s)));
            (format!("{a_hz:.1}/{b_hz:.1}"), a_s + b_s, words.next())
        }
        "group" => {
            let (hz, seconds) = (num(words.next())?, num(words.next())?);
            fields.push(("tone_hz".into(), common::Value::Float(hz)));
            fields.push(("tone_s".into(), common::Value::Float(seconds)));
            // A long tone opens every pager on it, so it is addressed to a
            // fleet rather than to one radio.
            fields.push(("call".into(), common::Value::Text("group".into())));
            (format!("{hz:.1}"), seconds, words.next())
        }
        _ => return None,
    };
    fields.insert(0, ("tones".into(), common::Value::Text(tones.clone())));
    let detail = match name {
        Some(name) => {
            fields.push(("pager".into(), common::Value::Text(name.to_string())));
            format!("{name} on {tones}")
        }
        None => format!("{tones} for {seconds:.1} s"),
    };
    let mut d = Decoded::bytes("Two-tone page", center, 0.0, bytes.to_vec())
        .with_detail(detail)
        .with_fields(fields)
        .with_modulation(common::Modulation::Fm);
    // The tones are the address, so a named pager is a device heard rather
    // than a field: the operator's list is what turns a pair into a who.
    if let Some(name) = name {
        d = d.by(common::Identity::new("pager-tones", tones).named(name.to_string()));
    }
    // Nothing checks, because two tones carry nothing to check with.
    d.crc_ok = None;
    Some(d)
}

/// Bytes before the page on the bus: the tag this front end writes, which is
/// what separates a page from any other decoder's text. A page has no check
/// sequence of its own, because two tones carry none.
pub const TAG: [u8; 4] = *b"2TON";

/// What a page puts on the bus: the tag, then the tones and their lengths as
/// text, and the name where the operator's list had one.
pub fn framed(page: &Page, name: Option<&str>) -> Vec<u8> {
    let body = match page {
        Page::Pair { a_hz, a_s, b_hz, b_s, .. } => {
            format!("pair {a_hz:.1} {a_s:.2} {b_hz:.1} {b_s:.2}")
        }
        Page::Group { hz, seconds, .. } => format!("group {hz:.1} {seconds:.2}"),
    };
    let mut out = Vec::with_capacity(TAG.len() + body.len() + 16);
    out.extend_from_slice(&TAG);
    out.extend_from_slice(body.as_bytes());
    if let Some(name) = name {
        out.push(b' ');
        out.extend_from_slice(name.as_bytes());
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    fn run(hz: f64, start_s: f64, seconds: f64) -> Run {
        Run { hz, start_s, seconds, level: 1.0 }
    }

    /// A Quick Call II page: a second of A, three of B, and the pair is what
    /// was sent.
    #[test]
    fn an_a_tone_then_a_b_tone_is_one_page() {
        let mut s = Sequential::default();
        assert_eq!(s.run(run(947.3, 10.0, 1.0)), None, "the A tone alone is not a page");
        let page = s.run(run(332.5, 11.0, 3.0)).expect("a page");
        assert_eq!(page, Page::Pair { a_hz: 947.3, a_s: 1.0, b_hz: 332.5, b_s: 3.0, at_s: 10.0 });
        assert_eq!(page.tones(), "947.3/332.5");
        assert_eq!(page.at_s(), 10.0);
    }

    /// A long tone on its own is an all-call, not half a page.
    #[test]
    fn a_long_tone_is_a_group_call() {
        let mut s = Sequential::default();
        let page = s.run(run(1153.4, 4.0, 8.0)).expect("a group call");
        assert_eq!(page, Page::Group { hz: 1153.4, seconds: 8.0, at_s: 4.0 });
    }

    /// Two tones that are not a page: too far apart in time, too close in
    /// frequency, and a B tone too short to be one.
    #[test]
    fn tones_that_are_not_a_page_are_not_one() {
        let mut s = Sequential::default();
        s.run(run(947.3, 0.0, 1.0));
        assert_eq!(s.run(run(332.5, 2.0, 3.0)), None, "two seconds of silence between them");

        let mut s = Sequential::default();
        s.run(run(947.3, 0.0, 1.0));
        assert_eq!(s.run(run(952.0, 1.0, 3.0)), None, "half a percent apart is one tone");

        let mut s = Sequential::default();
        s.run(run(947.3, 0.0, 1.0));
        assert_eq!(s.run(run(332.5, 1.0, 0.3)), None, "a B tone of 300 ms");
    }

    /// Two pages in a row, which is how a dispatcher raises two stations.
    #[test]
    fn two_pages_in_a_row_are_two_pages() {
        let mut s = Sequential::default();
        let mut pages = Vec::new();
        pages.extend(s.run(run(947.3, 0.0, 1.0)));
        pages.extend(s.run(run(332.5, 1.0, 3.0)));
        pages.extend(s.run(run(600.9, 5.0, 1.0)));
        pages.extend(s.run(run(1153.4, 6.0, 3.0)));
        assert_eq!(pages.len(), 2);
        assert_eq!(pages[0].tones(), "947.3/332.5");
        assert_eq!(pages[1].tones(), "600.9/1153.4");
    }

    /// The operator's list names the pager, and a pair nobody listed is
    /// still reported as tones.
    #[test]
    fn the_list_says_whose_tones_they_are() {
        let list = Pagers::parse(
            "Station 3 = 947.3/332.5\n\
             Fire brigade = 1122.5 / 1153.4  # all call is the B tone alone\n\
             rubbish\n\
             bad = 947.3/zero\n",
        );
        assert_eq!(list.len(), 2);
        let page = Page::Pair { a_hz: 950.7, a_s: 1.0, b_hz: 331.9, b_s: 3.0, at_s: 0.0 };
        assert_eq!(list.who(&page).map(|p| p.name.as_str()), Some("Station 3"));
        let all = Page::Group { hz: 1153.0, seconds: 8.0, at_s: 0.0 };
        assert_eq!(list.who(&all).map(|p| p.name.as_str()), Some("Fire brigade"));
        let other = Page::Pair { a_hz: 600.9, a_s: 1.0, b_hz: 1153.4, b_s: 3.0, at_s: 0.0 };
        assert_eq!(list.who(&other), None);
    }
}
