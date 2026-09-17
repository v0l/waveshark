//! Frequency lists somebody else wrote, and one this can leave as.
//!
//! The format is decided once from the contents and everything after that is
//! a [`Saved`]. Read generously, because a published band plan is a
//! spreadsheet somebody exported: a column may be missing, a frequency may be
//! in MHz or in Hz, and a width may be in kHz or in Hz.
//!
//! What the bank cannot hold is counted rather than guessed at. A range entry
//! is a band, not a channel, and a CTCSS tone has nowhere to go because the
//! receiver detects tones and is not squelched by one.

use super::{Memory, Saved, UNGROUPED, mode_from};
use crate::radio::{ChanMode, Demod, TxSpec};

/// The shape of a frequency list, read off its first lines.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Format {
    /// Our own `[group]` file.
    Waveshark,
    /// A handheld programmer's export: `Location,Name,Frequency,Duplex,...`.
    Chirp,
    /// Any other comma separated list, read by its header where it has one.
    Csv,
    /// PortaPack Mayhem `FREQMAN/*.TXT`: `f=`, `a=`/`b=`, `r=`/`t=` lines.
    Freqman,
    /// An SDR# `frequencies.xml` memory list.
    SdrSharp,
}

impl Format {
    pub fn label(self) -> &'static str {
        match self {
            Format::Waveshark => "waveshark",
            Format::Chirp => "Chirp",
            Format::Csv => "CSV",
            Format::Freqman => "Freqman",
            Format::SdrSharp => "SDR#",
        }
    }

    /// Decide the format from the contents, never from the file name: a
    /// Freqman list and a Chirp export are both `.TXT` or `.csv` to whoever
    /// sent them.
    pub fn of(text: &str) -> Format {
        let head: String = text.chars().take(4096).collect();
        let lower = head.to_ascii_lowercase();
        if lower.contains("<memoryentry") || lower.contains("arrayofmemoryentry") {
            return Format::SdrSharp;
        }
        let first = head
            .lines()
            .map(str::trim)
            .find(|l| !l.is_empty() && !l.starts_with('#'))
            .unwrap_or("");
        let flower = first.to_ascii_lowercase();
        if flower.starts_with("f=")
            || flower.starts_with("a=")
            || flower.starts_with("r=")
            || flower.starts_with("l=")
        {
            return Format::Freqman;
        }
        if flower.contains("duplex")
            || (flower.contains("location") && flower.contains("frequency"))
        {
            return Format::Chirp;
        }
        if first.contains(',') {
            return Format::Csv;
        }
        Format::Waveshark
    }
}

/// What a list gave up, and what it held that the bank cannot.
#[derive(Default, Debug)]
pub struct Read {
    pub list: Vec<Saved>,
    /// Entries naming a band rather than a channel.
    pub ranges: usize,
    /// Entries carrying a CTCSS tone, which is dropped.
    pub tones: usize,
    /// Lines that said nothing a channel could be built from.
    pub skipped: usize,
}

impl Read {
    /// A sentence for the operator, saying what arrived and what did not.
    pub fn note(&self, format: Format) -> String {
        let mut s = format!("{} channels from a {} list", self.list.len(), format.label());
        if self.ranges > 0 {
            s.push_str(&format!(", {} ranges dropped", self.ranges));
        }
        if self.tones > 0 {
            s.push_str(&format!(", {} CTCSS tones dropped", self.tones));
        }
        if self.skipped > 0 {
            s.push_str(&format!(", {} lines unread", self.skipped));
        }
        s
    }
}

/// Read a list, putting anything that names no group of its own into
/// `group`, which the interface takes from the file name.
pub fn read(text: &str, group: &str) -> (Format, Read) {
    let format = Format::of(text);
    let group = match group.trim() {
        "" => UNGROUPED,
        g => g,
    };
    let out = match format {
        Format::Waveshark => Read { list: Memory::parse(text).list, ..Read::default() },
        Format::Chirp | Format::Csv => csv_list(text, group),
        Format::Freqman => freqman(text, group),
        Format::SdrSharp => sdrsharp(text, group),
    };
    (format, out)
}

/// The bank as a Chirp CSV, which is the one export a handheld's programming
/// software will take.
///
/// Chirp knows nothing about a group or a decoder, and its columns are fixed,
/// so both go in the comment behind a `waveshark:` marker: a protocol channel
/// leaves as the FM channel a handheld can hold and comes back here as itself.
/// A file from anywhere else carries no marker and is read as it is written.
pub fn write_csv(m: &Memory) -> String {
    let mut s = String::from(CHIRP_HEADER);
    for (n, c) in m.list.iter().enumerate() {
        let (duplex, offset) = match c.tx.map(|t| t.shift_hz).unwrap_or(0.0) {
            v if v < 0.0 => ("-", -v),
            v if v > 0.0 => ("+", v),
            _ => ("", 0.0),
        };
        let comment =
            format!("{} waveshark:{}", c.group, c.mode.label().to_ascii_lowercase()).trim().into();
        let comment: String = comment;
        s.push_str(&format!(
            "{n},{},{:.6},{duplex},{:.6},,88.5,88.5,023,NN,{},5.00,,{}\n",
            quoted(&c.label),
            c.freq / 1e6,
            offset / 1e6,
            chirp_mode(c),
            quoted(&comment),
        ));
    }
    s
}

/// Chirp's own mode names. A decoded channel is FM on the air, wide or
/// narrow by the width it was saved with.
fn chirp_mode(c: &Saved) -> &'static str {
    match c.mode {
        ChanMode::Audio(Demod::Wfm) => "WFM",
        ChanMode::Audio(Demod::Am) => "AM",
        ChanMode::Audio(Demod::Usb) => "USB",
        ChanMode::Audio(Demod::Lsb) => "LSB",
        ChanMode::Audio(Demod::Cw) => "CW",
        // Chirp's FM is a 25 kHz channel and its NFM a 12.5 kHz one, so the
        // width decides which of the two a handheld is told to use.
        _ => match c.bandwidth_hz {
            Some(bw) if bw <= 15_000.0 => "NFM",
            _ => "FM",
        },
    }
}

fn quoted(s: &str) -> String {
    match s.contains(',') || s.contains('"') {
        true => format!("\"{}\"", s.replace('"', "\"\"")),
        false => s.to_string(),
    }
}

const CHIRP_HEADER: &str = "Location,Name,Frequency,Duplex,Offset,Tone,rToneFreq,cToneFreq,\
DtcsCode,DtcsPolarity,Mode,TStep,Skip,Comment\n";

/// A comma separated list, Chirp's or anybody's, read by its header.
fn csv_list(text: &str, group: &str) -> Read {
    let mut out = Read::default();
    let mut rdr = csv::ReaderBuilder::new()
        .flexible(true)
        .has_headers(false)
        .trim(csv::Trim::All)
        .from_reader(text.as_bytes());
    let mut cols: Option<Columns> = None;
    for row in rdr.records().flatten() {
        let fields: Vec<&str> = row.iter().collect();
        if fields.iter().all(|f| f.is_empty()) {
            continue;
        }
        if cols.is_none() {
            match Columns::of(&fields) {
                // A header names its columns and is not itself a channel.
                Some(c) => {
                    cols = Some(c);
                    continue;
                }
                // No header at all: the first field is the frequency and the
                // second the name, which is what a bare export looks like.
                None => cols = Some(Columns::bare()),
            }
        }
        let c = cols.as_ref().expect("set above");
        match c.row(&fields, group) {
            Some((saved, tone)) => {
                out.tones += usize::from(tone);
                out.list.push(saved);
            }
            None => out.skipped += 1,
        }
    }
    out
}

/// Which column holds what, by the names the exporters use.
#[derive(Default, Debug)]
struct Columns {
    freq: Option<usize>,
    name: Option<usize>,
    mode: Option<usize>,
    width: Option<usize>,
    group: Option<usize>,
    duplex: Option<usize>,
    offset: Option<usize>,
    tone: Option<usize>,
    comment: Option<usize>,
}

impl Columns {
    fn bare() -> Self {
        Columns { freq: Some(0), name: Some(1), mode: Some(2), width: Some(3), ..Self::default() }
    }

    /// A header row, or `None` where the row is already a channel.
    fn of(fields: &[&str]) -> Option<Self> {
        let mut c = Self::default();
        for (i, f) in fields.iter().enumerate() {
            let k: String = f.to_ascii_lowercase().replace([' ', '_'], "");
            let slot = match k.as_str() {
                "frequency" | "freq" | "rxfrequency" | "receivefrequency" | "outputfrequency" => {
                    &mut c.freq
                }
                "name" | "label" | "channelname" | "description" => &mut c.name,
                "mode" | "modulation" | "detectortype" | "demod" => &mut c.mode,
                "bandwidth" | "filterbandwidth" | "width" | "bw" => &mut c.width,
                "group" | "groupname" | "category" | "bank" => &mut c.group,
                "duplex" => &mut c.duplex,
                "offset" | "shift" => &mut c.offset,
                "tone" | "rtonefreq" | "ctcss" | "ctonefreq" => &mut c.tone,
                "comment" | "notes" => &mut c.comment,
                _ => continue,
            };
            slot.get_or_insert(i);
        }
        c.freq?;
        Some(c)
    }

    /// The channel a row holds, and whether it carried a tone that had to be
    /// dropped.
    fn row(&self, fields: &[&str], group: &str) -> Option<(Saved, bool)> {
        let at = |i: Option<usize>| i.and_then(|i| fields.get(i)).map(|s| s.trim()).unwrap_or("");
        let freq = frequency(at(self.freq))?;
        if freq <= 0.0 {
            return None;
        }
        let comment = at(self.comment);
        // Our own export puts the group and the front end in the comment,
        // because Chirp's columns hold neither.
        let ours = comment.split_once("waveshark:");
        let (mode, mut bandwidth_hz) = match mode_of(at(self.mode)) {
            Some(m) => m,
            // An empty mode column is not a broken row: an FM list that never
            // says so is the commonest export there is.
            None if at(self.mode).is_empty() => (ChanMode::Audio(Demod::Nfm), None),
            None => return None,
        };
        let mode = ours
            .and_then(|(_, k)| mode_from(k.split_whitespace().next().unwrap_or("")))
            .unwrap_or(mode);
        if let Some(bw) = width(at(self.width)) {
            bandwidth_hz = Some(bw);
        }
        let shift = match at(self.duplex) {
            "-" => -frequency(at(self.offset)).unwrap_or(0.0),
            "+" => frequency(at(self.offset)).unwrap_or(0.0),
            // A split entry names the transmit frequency outright.
            "split" => frequency(at(self.offset)).unwrap_or(0.0) - freq,
            _ => match self.duplex {
                Some(_) => 0.0,
                None => frequency(at(self.offset)).unwrap_or(0.0),
            },
        };
        let label = match (at(self.name), ours) {
            ("", None) => comment.to_string(),
            (n, _) => n.to_string(),
        };
        let tone = !matches!(at(self.tone), "" | "0" | "0.0");
        let group = match (at(self.group), ours.map(|(g, _)| g.trim())) {
            ("", None | Some("")) => group.to_string(),
            ("", Some(g)) => g.to_string(),
            (g, _) => g.to_string(),
        };
        Some((
            Saved {
                group,
                label,
                freq,
                mode,
                bandwidth_hz,
                tx: (shift != 0.0).then(|| TxSpec { shift_hz: shift, ..TxSpec::default() }),
            },
            tone,
        ))
    }
}

/// PortaPack Mayhem's `FREQMAN` list: `key=value` pairs, one entry a line.
///
/// `f=` is a channel, `r=`/`t=` and `l=`/`t=` a repeater pair whose transmit
/// side is the shift, and `a=`/`b=` a range, which the bank cannot hold.
fn freqman(text: &str, group: &str) -> Read {
    let mut out = Read::default();
    for line in text.lines() {
        let line = line.trim();
        if line.is_empty() || line.starts_with('#') {
            continue;
        }
        let mut rx = None;
        let mut tx = None;
        let mut range = false;
        let mut label = String::new();
        let mut mode = None;
        let mut bandwidth_hz = None;
        let mut tone = false;
        for token in line.split(',') {
            let Some((k, v)) = token.split_once('=') else { continue };
            match k.trim().to_ascii_lowercase().as_str() {
                "f" | "r" | "l" => rx = v.trim().parse::<f64>().ok(),
                "a" => {
                    rx = v.trim().parse::<f64>().ok();
                    range = true;
                }
                "b" => range = true,
                "t" => tx = v.trim().parse::<f64>().ok(),
                "d" => label = v.trim().to_string(),
                "m" => mode = mode_of(v.trim()).map(|(m, _)| m),
                "bw" => bandwidth_hz = width(v.trim()),
                "c" => tone = !v.trim().is_empty(),
                _ => {}
            }
        }
        out.tones += usize::from(tone);
        if range {
            out.ranges += 1;
            continue;
        }
        let Some(freq) = rx.filter(|f| *f > 0.0) else {
            out.skipped += 1;
            continue;
        };
        let shift = tx.map(|t| t - freq).unwrap_or(0.0);
        out.list.push(Saved {
            group: group.to_string(),
            label,
            freq,
            mode: mode.unwrap_or(ChanMode::Audio(Demod::Nfm)),
            bandwidth_hz,
            tx: (shift != 0.0).then(|| TxSpec { shift_hz: shift, ..TxSpec::default() }),
        });
    }
    out
}

/// SDR#'s `frequencies.xml`, which is one `<MemoryEntry>` per channel with
/// its group, its detector and its filter width beside it.
fn sdrsharp(text: &str, group: &str) -> Read {
    let mut out = Read::default();
    for entry in text.split("<MemoryEntry").skip(1) {
        let body = entry.split("</MemoryEntry>").next().unwrap_or(entry);
        let Some(freq) = tag(body, "Frequency").and_then(|f| frequency(&f)).filter(|f| *f > 0.0)
        else {
            out.skipped += 1;
            continue;
        };
        let (mode, default_bw) = tag(body, "DetectorType")
            .and_then(|d| mode_of(&d))
            .unwrap_or((ChanMode::Audio(Demod::Nfm), None));
        let shift = tag(body, "Shift").and_then(|s| s.parse::<f64>().ok()).unwrap_or(0.0);
        out.list.push(Saved {
            group: tag(body, "GroupName").filter(|g| !g.is_empty()).unwrap_or(group.to_string()),
            label: tag(body, "Name").unwrap_or_default(),
            freq,
            mode,
            bandwidth_hz: tag(body, "FilterBandwidth").and_then(|b| width(&b)).or(default_bw),
            tx: (shift != 0.0).then(|| TxSpec { shift_hz: shift, ..TxSpec::default() }),
        });
    }
    out
}

fn tag(body: &str, name: &str) -> Option<String> {
    let open = format!("<{name}>");
    let after = body.split_once(&open)?.1;
    let inner = after.split_once(&format!("</{name}>"))?.0;
    Some(
        inner
            .trim()
            .replace("&amp;", "&")
            .replace("&lt;", "<")
            .replace("&gt;", ">")
            .replace("&quot;", "\"")
            .replace("&apos;", "'"),
    )
}

/// A frequency as a list writes it: with a unit, in Hz, or in MHz.
///
/// No unit is the common case and the ambiguous one. A number at or above a
/// million is Hz, because 1 MHz is below every band a list is written about
/// and 1,000,000 MHz is above every band there is.
fn frequency(s: &str) -> Option<f64> {
    let s = s.trim();
    if s.is_empty() {
        return None;
    }
    let lower = s.to_ascii_lowercase();
    let unit = if lower.ends_with("ghz") {
        Some(1e9)
    } else if lower.ends_with("mhz") {
        Some(1e6)
    } else if lower.ends_with("khz") {
        Some(1e3)
    } else if lower.ends_with("hz") {
        Some(1.0)
    } else {
        None
    };
    let v: f64 = lower.trim_end_matches(|c: char| c.is_ascii_alphabetic()).trim().parse().ok()?;
    Some(v * unit.unwrap_or(if v.abs() >= 1e6 { 1.0 } else { 1e6 }))
}

/// A width in Hz, from `12500`, `12.5 kHz`, or Mayhem's `12k5`.
fn width(s: &str) -> Option<f64> {
    let s = s.trim();
    if s.is_empty() {
        return None;
    }
    // "DSB 9k" and "USB+3k" name a filter as well as a width, and "25 kHz"
    // is the number and its unit with a space between them: take the last
    // run of digits, points and k, which is the width in all three.
    let full = s.to_ascii_lowercase();
    let stripped = full.strip_suffix("hz").unwrap_or(&full);
    let lower = stripped
        .split(|c: char| !(c.is_ascii_digit() || c == '.' || c == 'k'))
        .rfind(|t| t.chars().any(|c| c.is_ascii_digit()))?
        .to_string();
    if let Some((a, b)) = lower.split_once('k')
        && !b.is_empty()
        && b.chars().all(|c| c.is_ascii_digit())
    {
        // 12k5 is 12.5 kHz: the k stands where the point would be.
        return format!("{a}.{b}").parse::<f64>().ok().map(|v| v * 1e3);
    }
    let v: f64 = lower
        .trim_end_matches(|c: char| c.is_ascii_alphabetic())
        .trim()
        .parse()
        .ok()
        .filter(|v: &f64| *v > 0.0)?;
    // A bare number is Hz above a thousand and kHz below it: no list means a
    // 12 Hz channel, and none means a 25,000 kHz one either.
    Some(match full.contains('k') || v < 1000.0 {
        true => v * 1e3,
        false => v,
    })
}

/// A mode as another program names it, and the width that name implies.
fn mode_of(s: &str) -> Option<(ChanMode, Option<f64>)> {
    let l = s.trim().to_ascii_lowercase();
    Some(match l.as_str() {
        "" => return None,
        // Chirp's FM is a 25 kHz channel and its NFM a 12.5 kHz one; nothing
        // else in these formats says a width by its mode name.
        "fm" => (ChanMode::Audio(Demod::Nfm), Some(25_000.0)),
        "nfm" | "fmn" => (ChanMode::Audio(Demod::Nfm), Some(12_500.0)),
        // SDR#'s RAW and Mayhem's SPEC are a span rather than a demodulator,
        // which here is the auto front end over that width.
        "raw" | "spec" | "iq" => (ChanMode::Auto, None),
        "dsb" => (ChanMode::Audio(Demod::Am), None),
        _ => (mode_from(&l)?, None),
    })
}

/// A file name for an export, with the day in it so two are not the same.
pub fn export_name() -> String {
    let now = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_micros() as u64)
        .unwrap_or(0);
    format!("waveshark-channels-{}.csv", crate::segments::when(now).format("%Y%m%d"))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::radio::TxSource;

    /// A handheld programmer's export, which is where most lists arrive from.
    ///
    /// Chirp writes the frequency and the offset in MHz, the shift as a
    /// direction in `Duplex`, and the tone in three columns of which only
    /// `Tone` says whether it is used.
    #[test]
    fn a_chirp_export_reads_with_its_shifts() {
        let text = "Location,Name,Frequency,Duplex,Offset,Tone,rToneFreq,cToneFreq,DtcsCode,\
             DtcsPolarity,Mode,TStep,Skip,Comment\n\
             0,GB3DB,145.725000,-,0.600000,Tone,103.5,103.5,023,NN,FM,5.00,,Dunstable\n\
             1,GB7IC,430.875000,-,7.600000,,88.5,88.5,023,NN,NFM,12.50,,\n\
             2,Airband,118.100000,,0.000000,,88.5,88.5,023,NN,AM,25.00,,Tower\n\
             3,Broadcast,98.100000,,0.000000,,88.5,88.5,023,NN,WFM,100.00,,\n";
        let (format, out) = read(text, "Import");
        assert_eq!(format, Format::Chirp);
        assert_eq!(out.list.len(), 4);
        assert_eq!(out.skipped, 0);
        assert_eq!(out.tones, 1, "only the first entry says its tone is used");

        assert_eq!(out.list[0].label, "GB3DB");
        assert_eq!(out.list[0].freq, 145_725_000.0);
        assert_eq!(out.list[0].bandwidth_hz, Some(25_000.0), "Chirp FM is a 25 kHz channel");
        assert_eq!(out.list[0].tx.expect("a repeater shift").shift_hz, -600_000.0);
        assert_eq!(out.list[0].group, "Import", "no group column, so the file's name");

        assert_eq!(out.list[1].tx.expect("a repeater shift").shift_hz, -7_600_000.0);
        assert_eq!(out.list[1].bandwidth_hz, Some(12_500.0));
        assert_eq!(out.list[1].label, "GB7IC");

        assert_eq!(out.list[2].mode, ChanMode::Audio(Demod::Am));
        assert_eq!(out.list[2].tx, None);
        assert_eq!(out.list[3].mode, ChanMode::Audio(Demod::Wfm));
        assert_eq!(out.list[3].freq, 98_100_000.0);
    }

    /// Mayhem's list, where a range is a band and a repeater is a pair.
    #[test]
    fn a_freqman_list_keeps_the_pairs_and_counts_the_ranges() {
        let text = "# PMR and repeaters\n\
             f=446006250,m=NFM,bw=12k5,d=PMR446 1\n\
             f=468000000,m=AM,d=Single Freq AM\n\
             a=87000000,b=110000000,m=WFM,bw=200k,s=50kHz,d=WFM radio search\n\
             r=430150000,t=430550000,m=NFM,bw=12k5,c=88.5,d=HAM with CTCSS\n\
             d=nothing here\n";
        let (format, out) = read(text, "Portapack");
        assert_eq!(format, Format::Freqman);
        assert_eq!(out.list.len(), 3);
        assert_eq!(out.ranges, 1, "a band is not a channel");
        assert_eq!(out.tones, 1);
        assert_eq!(out.skipped, 1, "the line with no frequency");

        assert_eq!(out.list[0].freq, 446_006_250.0);
        assert_eq!(out.list[0].bandwidth_hz, Some(12_500.0), "12k5 is 12.5 kHz");
        assert_eq!(out.list[0].label, "PMR446 1");
        assert_eq!(out.list[0].group, "Portapack");
        assert_eq!(out.list[1].mode, ChanMode::Audio(Demod::Am));

        let pair = out.list[2].tx.expect("the transmit side of the pair");
        assert_eq!(out.list[2].freq, 430_150_000.0);
        assert_eq!(pair.shift_hz, 400_000.0);
        assert_eq!(pair.source, TxSource::Tone, "nothing was said, so the default");
    }

    #[test]
    fn an_sdrsharp_list_reads_its_groups_and_widths() {
        let text = "<?xml version=\"1.0\"?>\n<ArrayOfMemoryEntry>\n\
             <MemoryEntry><Name>Tower</Name><GroupName>Airband</GroupName>\
             <Frequency>118100000</Frequency><DetectorType>AM</DetectorType>\
             <FilterBandwidth>8000</FilterBandwidth><Shift>0</Shift></MemoryEntry>\n\
             <MemoryEntry><Name>Beacon &amp; marker</Name><GroupName>Beacons</GroupName>\
             <Frequency>144412500</Frequency><DetectorType>NFM</DetectorType>\
             <FilterBandwidth>12500</FilterBandwidth></MemoryEntry>\n\
             <MemoryEntry><Name>broken</Name></MemoryEntry>\n\
             </ArrayOfMemoryEntry>\n";
        let (format, out) = read(text, "Import");
        assert_eq!(format, Format::SdrSharp);
        assert_eq!(out.list.len(), 2);
        assert_eq!(out.skipped, 1);
        assert_eq!(out.list[0].group, "Airband");
        assert_eq!(out.list[0].mode, ChanMode::Audio(Demod::Am));
        assert_eq!(out.list[0].bandwidth_hz, Some(8_000.0));
        assert_eq!(out.list[1].label, "Beacon & marker", "the entities are unescaped");
        assert_eq!(out.list[1].freq, 144_412_500.0);
        assert_eq!(out.list[1].group, "Beacons");
    }

    /// A spreadsheet somebody exported: a header naming its own columns, a
    /// frequency in MHz in one row and in Hz in the next.
    #[test]
    fn a_plain_csv_reads_by_its_header() {
        let text = "Group Name,Name,Frequency,Modulation,Bandwidth\n\
             Marine,Ch 16,156.800,NFM,25000\n\
             Marine,Ch 6,156300000,NFM,25\n\
             Pagers,Capcodes,153.350,POCSAG,\n\
             ,,,,\n\
             Bad,No frequency,,NFM,\n";
        let (format, out) = read(text, "Import");
        assert_eq!(format, Format::Csv);
        assert_eq!(out.list.len(), 3);
        assert_eq!(out.skipped, 1, "the row with no frequency; the empty row is not a row");
        assert_eq!(out.list[0].freq, 156_800_000.0);
        assert_eq!(out.list[1].freq, 156_300_000.0, "a number that big is already Hz");
        assert_eq!(out.list[1].bandwidth_hz, Some(25_000.0), "a small number is kHz");
        assert_eq!(out.list[2].mode, ChanMode::Decode("pocsag".into()));
        assert_eq!(out.list[2].group, "Pagers");
    }

    /// A list with no header at all, which is the other half of what people
    /// send: frequency first, then the name.
    #[test]
    fn a_headerless_csv_takes_the_frequency_first() {
        let (format, out) = read("145.500,Calling,NFM,12.5\n433.500,Simplex\n", "Import");
        assert_eq!(format, Format::Csv);
        assert_eq!(out.list.len(), 2);
        assert_eq!(out.list[0].freq, 145_500_000.0);
        assert_eq!(out.list[0].label, "Calling");
        assert_eq!(out.list[0].bandwidth_hz, Some(12_500.0));
        assert_eq!(out.list[1].freq, 433_500_000.0);
        assert_eq!(out.list[1].mode, ChanMode::Audio(Demod::Nfm), "nothing said, so FM");
    }

    /// What leaves has to come back: a bank exported as Chirp CSV and read
    /// again is the same bank, protocol channels and shifts included.
    #[test]
    fn the_export_reads_back_as_the_same_bank() {
        let bank = Memory::parse(
            "[Airband]\n118.1 MHz AM Dublin tower\n\
             [Repeaters]\n145.7375 MHz NFM 12.5 kHz shift:-600kHz GB3XX\n\
             [Pagers]\n439.9875 MHz POCSAG capcodes\n\
             [Watch]\n433.475 MHz auto 40 kHz calling\n",
        );
        assert_eq!(bank.list.len(), 4);
        let csv = write_csv(&bank);
        let (format, back) = read(&csv, UNGROUPED);
        assert_eq!(format, Format::Chirp);
        assert_eq!(back.list.len(), 4);

        let modes: Vec<ChanMode> = back.list.iter().map(|c| c.mode.clone()).collect();
        assert_eq!(
            modes,
            vec![
                ChanMode::Audio(Demod::Am),
                ChanMode::Audio(Demod::Nfm),
                ChanMode::Decode("pocsag".into()),
                ChanMode::Auto,
            ]
        );
        let groups: Vec<&str> = back.list.iter().map(|c| c.group.as_str()).collect();
        assert_eq!(groups, ["Airband", "Repeaters", "Pagers", "Watch"]);
        let labels: Vec<&str> = back.list.iter().map(|c| c.label.as_str()).collect();
        assert_eq!(labels, ["Dublin tower", "GB3XX", "capcodes", "calling"]);
        assert_eq!(back.list[1].tx.expect("the shift").shift_hz, -600_000.0);
        assert_eq!(back.list[1].freq, 145_737_500.0);
        assert_eq!(back.list[3].freq, 433_475_000.0);
        // Chirp keeps five decimal places of a megahertz at most, so a
        // frequency goes out in MHz and comes back to the nearest 10 Hz.
        assert!(csv.contains("145.737500"), "{csv}");
    }

    /// Our own file is still our own file, whichever door it comes in by.
    #[test]
    fn our_own_bank_is_recognised_and_not_read_as_a_csv() {
        let (format, out) = read("[Airband]\n118.1 MHz AM Dublin tower\n", "Import");
        assert_eq!(format, Format::Waveshark);
        assert_eq!(out.list.len(), 1);
        assert_eq!(out.list[0].group, "Airband");
    }

    /// Reading the same list twice is a correction, not a second copy of
    /// everything in it.
    #[test]
    fn importing_the_same_list_twice_adds_nothing() {
        let text = "Group Name,Name,Frequency,Modulation\n\
             Marine,Ch 16,156.800,NFM\n\
             Marine,Ch 6,156.300,NFM\n";
        let mut bank = Memory::default();
        assert_eq!(bank.merge(read(text, "Import").1.list), 2);
        assert_eq!(bank.merge(read(text, "Import").1.list), 0);
        assert_eq!(bank.list.len(), 2);
        assert_eq!(bank.groups(), ["Marine"]);
        // And the file it writes is still the file it reads.
        assert_eq!(Memory::parse(&bank.render()).list, bank.list);
    }

    #[test]
    fn nothing_useful_is_nothing_saved() {
        let (_, out) = read("", "Import");
        assert_eq!(out.list.len(), 0);
        let (_, out) = read("the quick brown fox\njumped over\n", "Import");
        assert_eq!(out.list.len(), 0);
        assert_eq!(out.skipped, 0, "a file that is not a list at all is not a list of bad lines");
    }

    #[test]
    fn a_width_is_read_in_whatever_unit_it_was_written() {
        assert_eq!(width("12500"), Some(12_500.0));
        assert_eq!(width("12.5"), Some(12_500.0));
        assert_eq!(width("12k5"), Some(12_500.0));
        assert_eq!(width("8k5"), Some(8_500.0));
        assert_eq!(width("200k"), Some(200_000.0));
        assert_eq!(width("DSB 9k"), Some(9_000.0));
        assert_eq!(width("USB+3k"), Some(3_000.0));
        assert_eq!(width("25 kHz"), Some(25_000.0));
        assert_eq!(width(""), None);
        assert_eq!(width("wide"), None);
    }

    #[test]
    fn a_frequency_is_read_in_whatever_unit_it_was_written() {
        assert_eq!(frequency("145.500"), Some(145_500_000.0));
        assert_eq!(frequency("145500000"), Some(145_500_000.0));
        assert_eq!(frequency("145.5 MHz"), Some(145_500_000.0));
        assert_eq!(frequency("1090000 kHz"), Some(1_090_000_000.0));
        assert_eq!(frequency("2.4 GHz"), Some(2_400_000_000.0));
        assert_eq!(frequency("nonsense"), None);
    }
}
