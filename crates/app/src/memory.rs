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

use crate::radio::ChanMode;
use crate::scanners::{hz, num};
use std::path::PathBuf;

#[derive(Clone, PartialEq, Debug)]
pub struct Saved {
    pub group: String,
    pub label: String,
    pub freq: f64,
    pub mode: ChanMode,
    /// `None` for the mode's own width.
    pub bandwidth_hz: Option<f64>,
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
        let same = |e: &Saved| {
            e.group == s.group && (e.freq - s.freq).abs() < 1.0 && e.mode == s.mode
        };
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
            let (Some(f), Some(u)) = (t.next(), t.next()) else { continue };
            let Some(freq) = hz(&format!("{f} {u}")) else { continue };
            let Some(mode) = t.next().and_then(mode_from) else { continue };
            let rest: Vec<&str> = t.collect();
            // A width is a number followed by a unit; a label that starts
            // with a number would have to be written after one.
            let (bandwidth_hz, label_from) = match rest.as_slice() {
                [n, u, ..] if n.parse::<f64>().is_ok() && is_unit(u) => {
                    (hz(&format!("{n} {u}")), 2)
                }
                _ => (None, 0),
            };
            let label = rest[label_from..].join(" ");
            list.push(Saved { group: group.clone(), label, freq, mode, bandwidth_hz });
        }
        Self { list }
    }

    pub fn render(&self) -> String {
        let mut s = String::from(HEADER);
        for g in self.groups() {
            s.push_str(&format!("\n[{g}]\n"));
            for (_, c) in self.in_group(g) {
                s.push_str(&format!("{:<14}{:<8}", format!("{} MHz", num(c.freq / 1e6)), c.mode.label()));
                match c.bandwidth_hz {
                    Some(bw) => s.push_str(&format!("{:<12}", format!("{} kHz", num(bw / 1e3)))),
                    None => s.push_str(&format!("{:<12}", "")),
                }
                s.push_str(c.label.trim());
                s.push('\n');
            }
        }
        s
    }
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
        assert_eq!(Memory::parse(&m.render()), m);
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
