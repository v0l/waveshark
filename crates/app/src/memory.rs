//! The memory bank: channels worth coming back to, in groups.
//!
//! A file rather than part of the session, for the reason the scanner table
//! is: the session is rewritten whole every few seconds and a list somebody
//! curates by hand cannot live in a file the program keeps overwriting. This
//! one is read at start, and written when the bank changes.
//!
//! What is kept is what rebuilds the channel: where it is, what it does, how
//! wide, and what it is called. Levels and squelch are the strip's and are
//! set against the signal on the day.

pub mod formats;

use crate::radio::{ChanMode, TxSource, TxSpec};
use crate::scanners::{hz, num};
pub use dsp::squelch::Coded;
use std::path::PathBuf;

#[derive(Clone, PartialEq, Debug)]
pub struct Saved {
    pub group: String,
    pub label: String,
    pub freq: f64,
    pub mode: ChanMode,
    /// `None` for the mode's own width.
    pub bandwidth_hz: Option<f64>,
    /// What it puts on the air, or `None` for the mode's own default.
    ///
    /// A repeater channel is one channel that listens on the output and
    /// transmits on the input, so its shift is as much a part of it as its
    /// frequency: recalled without one it is a channel that works simplex
    /// on a repeater's output, which nobody hears.
    pub tx: Option<TxSpec>,
    /// The coded squelch it opens on, or `None` to hear whoever is there.
    ///
    /// The other half of what a programmed channel is: two groups share the
    /// frequency and the tone is what says which of them this channel is
    /// for, so a repeater saved without it comes back opening on the next
    /// town's traffic.
    pub tone: Option<Coded>,
}

#[derive(Clone, PartialEq, Debug, Default)]
pub struct Memory {
    pub list: Vec<Saved>,
}

/// The group a channel goes into when none was named.
pub const UNGROUPED: &str = "Channels";

impl Memory {
    /// `$XDG_CONFIG_HOME/waveshark/channels`, beside the scanner table.
    pub fn path() -> Option<PathBuf> {
        let base = std::env::var_os("XDG_CONFIG_HOME")
            .map(PathBuf::from)
            .or_else(|| std::env::var_os("HOME").map(|h| PathBuf::from(h).join(".config")))?;
        Some(base.join("waveshark").join("channels"))
    }

    pub fn load() -> Self {
        Self::path()
            .and_then(|p| std::fs::read_to_string(p).ok())
            .map(|t| Self::parse(&t))
            .unwrap_or_default()
    }

    pub fn save(&self) -> std::io::Result<()> {
        let Some(path) = Self::path() else {
            return Err(std::io::Error::other("no config directory"));
        };
        if let Some(dir) = path.parent() {
            std::fs::create_dir_all(dir)?;
        }
        std::fs::write(path, self.render())
    }

    /// The groups, in the order they first appear.
    pub fn groups(&self) -> Vec<&str> {
        let mut out: Vec<&str> = Vec::new();
        for s in &self.list {
            if !out.contains(&s.group.as_str()) {
                out.push(&s.group);
            }
        }
        out
    }

    pub fn in_group<'a>(&'a self, group: &'a str) -> impl Iterator<Item = (usize, &'a Saved)> + 'a {
        self.list.iter().enumerate().filter(move |(_, s)| s.group == group)
    }

    /// Add a channel, replacing one already saved at the same frequency and
    /// mode in the same group: saving twice is a correction, not a duplicate.
    pub fn add(&mut self, mut s: Saved) {
        if s.group.trim().is_empty() {
            s.group = UNGROUPED.into();
        }
        s.group = s.group.trim().to_string();
        let same =
            |e: &Saved| e.group == s.group && (e.freq - s.freq).abs() < 1.0 && e.mode == s.mode;
        if let Some(i) = self.list.iter().position(same) {
            self.list[i] = s;
        } else {
            // Beside the rest of its group, so the file reads as groups.
            let at = self
                .list
                .iter()
                .rposition(|e| e.group == s.group)
                .map(|i| i + 1)
                .unwrap_or(self.list.len());
            self.list.insert(at, s);
        }
    }

    /// Add a list read from somewhere else, correcting whatever is already
    /// saved at the same frequency in the same group. Returns how many
    /// entries the bank grew by, which is fewer than were read when a list
    /// is imported twice.
    pub fn merge(&mut self, list: Vec<Saved>) -> usize {
        let before = self.list.len();
        for s in list {
            self.add(s);
        }
        self.list.len() - before
    }

    pub fn remove(&mut self, i: usize) {
        if i < self.list.len() {
            self.list.remove(i);
        }
    }

    /// `[group]` headings over one channel per line: frequency, mode, an
    /// optional width, and the rest of the line as the label.
    pub fn parse(text: &str) -> Self {
        let mut list = Vec::new();
        let mut group = UNGROUPED.to_string();
        for line in text.lines() {
            let line = line.split('#').next().unwrap_or("").trim();
            if line.is_empty() {
                continue;
            }
            if let Some(g) = line.strip_prefix('[').and_then(|l| l.strip_suffix(']')) {
                group = g.trim().to_string();
                if group.is_empty() {
                    group = UNGROUPED.into();
                }
                continue;
            }
            let mut t = line.split_whitespace();
            let (Some(f), Some(u)) = (t.next(), t.next()) else {
                continue;
            };
            let Some(freq) = hz(&format!("{f} {u}")) else {
                continue;
            };
            let Some(mode) = t.next().and_then(mode_from) else {
                continue;
            };
            let rest: Vec<&str> = t.collect();
            // A width is a number followed by a unit; a label that starts
            // with a number would have to be written after one.
            let (bandwidth_hz, label_from) = match rest.as_slice() {
                [n, u, ..] if n.parse::<f64>().is_ok() && is_unit(u) => {
                    (hz(&format!("{n} {u}")), 2)
                }
                _ => (None, 0),
            };
            let (tx, tone, label_from) = tokens(&rest[label_from..], label_from);
            let label = rest[label_from..].join(" ");
            list.push(Saved { group: group.clone(), label, freq, mode, bandwidth_hz, tx, tone });
        }
        Self { list }
    }

    pub fn render(&self) -> String {
        let mut s = String::from(HEADER);
        for g in self.groups() {
            s.push_str(&format!("\n[{g}]\n"));
            for (_, c) in self.in_group(g) {
                s.push_str(&format!(
                    "{:<14}{:<8}",
                    format!("{} MHz", num(c.freq / 1e6)),
                    c.mode.label()
                ));
                match c.bandwidth_hz {
                    Some(bw) => s.push_str(&format!("{:<12}", format!("{} kHz", num(bw / 1e3)))),
                    None => s.push_str(&format!("{:<12}", "")),
                }
                for token in written_tokens(c) {
                    s.push_str(&format!("{token:<16}"));
                }
                s.push_str(c.label.trim());
                s.push('\n');
            }
        }
        s
    }
}

/// What a channel carries beyond its frequency, read off the `key:value`
/// tokens in front of the label, and how many tokens that took.
///
/// Only what differs from the mode's own default is written, so a plain
/// simplex channel reads and writes exactly as it did before any of this.
fn tokens(rest: &[&str], from: usize) -> (Option<TxSpec>, Option<Coded>, usize) {
    let mut tx: Option<TxSpec> = None;
    let mut tone: Option<Coded> = None;
    let mut n = 0;
    for token in rest {
        let Some((key, value)) = token.split_once(':') else { break };
        match key.to_ascii_lowercase().as_str() {
            "shift" => match hz(value) {
                Some(v) => tx.get_or_insert_with(TxSpec::default).shift_hz = v,
                None => break,
            },
            "src" => match value.to_ascii_lowercase().as_str() {
                "mic" => tx.get_or_insert_with(TxSpec::default).source = TxSource::Mic,
                "tone" => tx.get_or_insert_with(TxSpec::default).source = TxSource::Tone,
                _ => break,
            },
            "trim" => match value.trim_end_matches(|c: char| c.is_ascii_alphabetic()).parse() {
                Ok(v) => tx.get_or_insert_with(TxSpec::default).trim_db = v,
                Err(_) => break,
            },
            "tone" => match value.parse() {
                Ok(c) => tone = Some(c),
                Err(()) => break,
            },
            // A label may hold a colon. Anything not named here ends the
            // tokens and starts it.
            _ => break,
        }
        n += 1;
    }
    match n {
        0 => (None, None, from),
        n => (tx, tone, from + n),
    }
}

fn written_tokens(c: &Saved) -> Vec<String> {
    let mut out = Vec::new();
    if let Some(tx) = c.tx {
        let default = TxSpec::default();
        if tx.shift_hz != default.shift_hz {
            out.push(format!("shift:{}kHz", num(tx.shift_hz / 1e3)));
        }
        if tx.source != default.source {
            out.push(format!("src:{}", tx.source.label().to_ascii_lowercase()));
        }
        if tx.trim_db != default.trim_db {
            out.push(format!("trim:{}dB", num(tx.trim_db as f64)));
        }
    }
    if let Some(tone) = c.tone {
        out.push(format!("tone:{tone}"));
    }
    out
}

fn is_unit(s: &str) -> bool {
    matches!(s.to_ascii_lowercase().as_str(), "hz" | "khz" | "mhz" | "ghz")
}

/// A mode as written in the file: a demodulator, `auto`, or a front end the
/// registry knows.
pub fn mode_from(s: &str) -> Option<ChanMode> {
    use crate::radio::Demod;
    let l = s.to_ascii_lowercase();
    Some(match l.as_str() {
        "wfm" => ChanMode::Audio(Demod::Wfm),
        "nfm" | "fm" => ChanMode::Audio(Demod::Nfm),
        "am" => ChanMode::Audio(Demod::Am),
        "usb" => ChanMode::Audio(Demod::Usb),
        "lsb" => ChanMode::Audio(Demod::Lsb),
        "cw" => ChanMode::Audio(Demod::Cw),
        "auto" => ChanMode::Auto,
        // A protocol, by the name the file writes (its label) or the one
        // the registry knows it by.
        _ => ChanMode::Decode(crate::chain::front_kind(&l)?.to_string()),
    })
}

const HEADER: &str = "\
# waveshark channels: the memory bank.
#
# One channel per line under a [group] heading: frequency, mode, an optional
# width, and the rest of the line is the label. The interface rewrites this
# file from its own list, so comments below this header are not kept.
#
#   modes   WFM NFM AM USB LSB CW AUTO, or a front end such as M17 or POCSAG
#   width   e.g. 25 kHz; leave it out for the mode's own
#   shift:  what it transmits away from its own frequency, e.g. shift:-600kHz
#   src:    what it transmits, mic or tone; tone unless it says otherwise
#   trim:   this channel's own offset from the transmit gain, e.g. trim:-6dB
#   tone:   the coded squelch it opens on, e.g. tone:88.5 or tone:D023
";

#[cfg(test)]
mod tests {
    use super::*;
    use crate::radio::Demod;

    #[test]
    fn a_hand_written_bank_reads_and_round_trips() {
        let m = Memory::parse(
            "[Airband]\n118.1 MHz AM Dublin tower\n\
             [Repeaters]\n145.6 MHz NFM 25 kHz GB3XX  # wide\n\
             439.9875 MHz POCSAG\n433.475 MHz auto 40 kHz calling\n",
        );
        assert_eq!(m.list.len(), 4);
        assert_eq!(m.list[0].label, "Dublin tower");
        assert_eq!(m.list[0].mode, ChanMode::Audio(Demod::Am));
        assert_eq!(m.list[0].bandwidth_hz, None);
        assert_eq!(m.list[1].bandwidth_hz, Some(25_000.0));
        assert_eq!(m.list[1].label, "GB3XX");
        assert_eq!(m.list[2].mode, ChanMode::Decode("pocsag".into()));
        assert_eq!(m.list[2].label, "");
        assert_eq!(m.list[3].mode, ChanMode::Auto);
        assert_eq!(m.groups(), ["Airband", "Repeaters"]);
        assert_eq!(m.list.iter().filter(|c| c.tx.is_some()).count(), 0);
        assert_eq!(m.list.iter().filter(|c| c.tone.is_some()).count(), 0);
        assert_eq!(Memory::parse(&m.render()), m);
    }

    /// A channel keeps the coded squelch it was programmed with.
    ///
    /// Two groups share a repeater's output and the tone is what says which
    /// of them this channel is for. Saved without it, the channel comes back
    /// opening on the other one.
    #[test]
    fn a_channel_keeps_the_coded_squelch_it_opens_on() {
        let m = Memory::parse(
            "[Repeaters]\n\
             145.7375 MHz NFM 12.5 kHz shift:-600kHz tone:88.5 GB3XX\n\
             430.875 MHz NFM tone:D023 GB7YY\n\
             446.05 MHz NFM PMR5\n\
             433.5 MHz NFM tone:89.2 made up\n",
        );
        assert_eq!(m.list.len(), 4);
        assert_eq!(m.list[0].tone, Some(Coded::Tone(8)));
        assert_eq!(m.list[0].tx.expect("the shift").shift_hz, -600_000.0);
        assert_eq!(m.list[0].label, "GB3XX");
        assert_eq!(m.list[1].tone, Some(Coded::Dcs(23)));
        assert_eq!(m.list[1].tx, None, "a tone is not a transmit side");
        assert_eq!(m.list[1].label, "GB7YY");
        assert_eq!(m.list[2].tone, None, "nothing said, so whoever is there");
        // A tone no radio offers is not a tone, and the line is still a
        // channel: the token stays in the label rather than programming the
        // channel to something it cannot hear.
        assert_eq!(m.list[3].tone, None);
        assert_eq!(m.list[3].label, "tone:89.2 made up");

        let written = m.render();
        assert!(written.contains("tone:88.5"), "{written}");
        assert!(written.contains("tone:D023"), "{written}");
        assert_eq!(Memory::parse(&written).list[..3], m.list[..3]);
    }

    /// A repeater channel keeps its shift.
    ///
    /// It is one channel that listens on the output and transmits on the
    /// input, so a bank that saves only the frequency saves half of it: the
    /// channel comes back simplex on a repeater's output, where nobody is
    /// listening. The source and the trim go with it for the same reason,
    /// since they belong to the channel and not to the day.
    #[test]
    fn a_repeater_channel_keeps_what_it_transmits() {
        let m = Memory::parse(
            "[Repeaters]\n\
             145.7375 MHz NFM 12.5 kHz shift:-600kHz src:mic GB3XX\n\
             430.875 MHz NFM shift:-7.6MHz trim:-6dB GB7YY\n\
             446.05 MHz NFM PMR5\n\
             144.8 MHz NFM 3:1 odds\n",
        );
        assert_eq!(m.list.len(), 4);

        let gb3xx = m.list[0].tx.expect("the repeater transmits");
        assert_eq!(gb3xx.shift_hz, -600_000.0);
        assert_eq!(gb3xx.source, TxSource::Mic);
        assert_eq!(m.list[0].label, "GB3XX");

        let gb7yy = m.list[1].tx.expect("the repeater transmits");
        assert_eq!(gb7yy.shift_hz, -7_600_000.0);
        assert_eq!(gb7yy.trim_db, -6.0);
        assert_eq!(gb7yy.source, TxSource::Tone, "nothing was said, so the default");
        assert_eq!(m.list[1].label, "GB7YY");

        // Nothing said about transmitting is not a transmit side, so the
        // channel comes back on whatever the mode's default is.
        assert_eq!(m.list[2].tx, None);
        assert_eq!(m.list[2].label, "PMR5");
        // And a label may hold a colon: only the names above are tokens.
        assert_eq!(m.list[3].tx, None);
        assert_eq!(m.list[3].label, "3:1 odds");

        assert_eq!(Memory::parse(&m.render()), m);
        let written = m.render();
        assert!(written.contains("shift:-600kHz"), "{written}");
        assert!(written.contains("src:mic"), "{written}");
        assert!(written.contains("trim:-6dB"), "{written}");
    }

    #[test]
    fn saving_the_same_channel_again_corrects_it() {
        let mut m = Memory::default();
        let s = |label: &str, bw: Option<f64>| Saved {
            group: "A".into(),
            label: label.into(),
            freq: 145_600_000.0,
            mode: ChanMode::Audio(Demod::Nfm),
            bandwidth_hz: bw,
            tx: None,
            tone: None,
        };
        m.add(s("first", None));
        m.add(s("second", Some(25_000.0)));
        assert_eq!(m.list.len(), 1);
        assert_eq!(m.list[0].label, "second");
        // A different group is a different entry, filed beside its group.
        m.add(Saved { group: "B".into(), ..s("b", None) });
        m.add(Saved { group: "A".into(), freq: 145_500_000.0, ..s("a2", None) });
        assert_eq!(m.groups(), ["A", "B"]);
        assert_eq!(m.list[1].label, "a2");
    }

    #[test]
    fn a_broken_line_is_skipped_not_fatal() {
        let m = Memory::parse("[x]\nnonsense\n145.6 MHz FOO\n145.6 MHz NFM ok\n");
        assert_eq!(m.list.len(), 1);
        assert_eq!(m.list[0].group, "x");
    }
}
