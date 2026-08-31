//! Every packet, written to disk as it arrives.
//!
//! On by default and with no switch in the interface, because the value of a
//! packet log is entirely in having it already: the interesting transmission
//! is always the one that happened before anyone thought to press record. A
//! band left running overnight is a test corpus, and a corpus of real frames
//! is the only honest way to tell whether a change to a decoder helped.
//!
//! One JSON object per line, appended, one file per day. That format is
//! deliberate:
//!
//! - Appending a line is atomic enough that a receiver killed mid-write costs
//!   at most the last packet, not the file.
//! - A day per file bounds any single read and makes deleting old data a
//!   matter of deleting files.
//! - Line-delimited JSON is what every analysis tool already reads, and it
//!   survives this project's own record layout changing, which a binary format
//!   would not.
//!
//! Written by hand rather than through a serialisation crate: the shape is
//! eight scalars and a field map, and it is not worth a dependency in the
//! crate that has none.

use crate::radio::DecodeRecord;
use common::Value;
use std::io::Write;
use std::path::PathBuf;

/// Stop appending when a day's file reaches this. A receiver on a busy band
/// writes a few megabytes an hour; this is a runaway guard, not a budget.
const MAX_BYTES: u64 = 512 << 20;

pub struct PacketLog {
    dir: PathBuf,
    /// The day currently open, as `YYYY-MM-DD`, and its writer.
    open: Option<(String, std::io::BufWriter<std::fs::File>)>,
    written: u64,
    full: bool,
    /// Packets appended since the receiver started.
    logged: u64,
}

impl PacketLog {
    /// `$XDG_DATA_HOME/super-radio/packets`, or `~/.local/share` when unset.
    pub fn default_dir() -> Option<PathBuf> {
        let base = std::env::var_os("XDG_DATA_HOME")
            .map(PathBuf::from)
            .or_else(|| std::env::var_os("HOME").map(|h| PathBuf::from(h).join(".local/share")))?;
        Some(base.join("super-radio").join("packets"))
    }

    pub fn new(dir: PathBuf) -> Self {
        Self { dir, open: None, written: 0, full: false, logged: 0 }
    }

    pub fn logged(&self) -> u64 {
        self.logged
    }

    pub fn dir(&self) -> &std::path::Path {
        &self.dir
    }

    /// Append a batch, opening or rolling the day's file as needed.
    ///
    /// Errors are swallowed on purpose. A full disk or a read-only home must
    /// not take the receiver down or spam the fault line: the log is a
    /// convenience, and losing it is not worth losing the packets on screen.
    pub fn append(&mut self, records: &[DecodeRecord], now: std::time::SystemTime) {
        if self.full || records.is_empty() {
            return;
        }
        let day = day_of(now);
        if self.open.as_ref().is_none_or(|(d, _)| *d != day) {
            if std::fs::create_dir_all(&self.dir).is_err() {
                self.full = true;
                return;
            }
            let path = self.dir.join(format!("{day}.jsonl"));
            let Ok(f) = std::fs::OpenOptions::new().create(true).append(true).open(&path) else {
                self.full = true;
                return;
            };
            self.written = f.metadata().map(|m| m.len()).unwrap_or(0);
            self.open = Some((day, std::io::BufWriter::new(f)));
        }
        let Some((_, w)) = self.open.as_mut() else { return };
        let wall = now.duration_since(std::time::UNIX_EPOCH).unwrap_or_default().as_secs_f64();
        for r in records {
            let line = to_json(r, wall);
            if w.write_all(line.as_bytes()).is_err() {
                self.full = true;
                return;
            }
            self.written += line.len() as u64;
            self.logged += 1;
        }
        // Flushed every batch rather than left to the buffer: a receiver is
        // usually killed rather than closed, and an unflushed buffer is the
        // packets nobody has.
        let _ = w.flush();
        if self.written >= MAX_BYTES {
            self.full = true;
        }
    }
}

/// UTC date as `YYYY-MM-DD`, by civil-from-days rather than a calendar crate.
fn day_of(t: std::time::SystemTime) -> String {
    let secs = t.duration_since(std::time::UNIX_EPOCH).unwrap_or_default().as_secs() as i64;
    let days = secs.div_euclid(86_400);
    // Howard Hinnant's civil_from_days, which is exact and fits in a function.
    let z = days + 719_468;
    let era = z.div_euclid(146_097);
    let doe = z.rem_euclid(146_097);
    let yoe = (doe - doe / 1460 + doe / 36_524 - doe / 146_096) / 365;
    let y = yoe + era * 400;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let d = doy - (153 * mp + 2) / 5 + 1;
    let m = if mp < 10 { mp + 3 } else { mp - 9 };
    let y = if m <= 2 { y + 1 } else { y };
    format!("{y:04}-{m:02}-{d:02}")
}

fn to_json(r: &DecodeRecord, wall: f64) -> String {
    let mut s = String::with_capacity(256);
    s.push('{');
    s.push_str(&format!("\"at\":{wall:.3}"));
    s.push_str(&format!(",\"freq\":{:.0}", r.freq));
    s.push_str(&format!(",\"model\":{}", quote(&r.model)));
    s.push_str(&format!(",\"modulation\":{}", quote(r.modulation)));
    if r.rssi_dbfs.is_finite() {
        s.push_str(&format!(",\"rssi_dbfs\":{:.1}", r.rssi_dbfs));
    }
    if r.snr_db.is_finite() {
        s.push_str(&format!(",\"snr_db\":{:.1}", r.snr_db));
    }
    match r.crc {
        Some(v) => s.push_str(&format!(",\"crc_ok\":{v}")),
        // Absent rather than null: a protocol with no integrity check is not
        // the same as one whose check was not run, and a reader filtering on
        // the key gets that distinction for free.
        None => {}
    }
    let hex: String = r.bytes.iter().map(|b| format!("{b:02x}")).collect();
    s.push_str(&format!(",\"bytes\":{}", quote(&hex)));
    if !r.fields.is_empty() {
        s.push_str(",\"fields\":{");
        for (i, (k, v)) in r.fields.iter().enumerate() {
            if i > 0 {
                s.push(',');
            }
            s.push_str(&format!("{}:{}", quote(k), value(v)));
        }
        s.push('}');
    }
    s.push_str("}\n");
    s
}

fn value(v: &Value) -> String {
    match v {
        Value::Int(i) => i.to_string(),
        // Non-finite floats are not JSON. They should not occur, but a log
        // that emits `NaN` is a log no parser will read.
        Value::Float(f) if f.is_finite() => format!("{f}"),
        Value::Float(_) => "null".into(),
        Value::Bool(b) => b.to_string(),
        Value::Text(t) => quote(t),
    }
}

fn quote(s: &str) -> String {
    let mut out = String::with_capacity(s.len() + 2);
    out.push('"');
    for c in s.chars() {
        match c {
            '"' => out.push_str("\\\""),
            '\\' => out.push_str("\\\\"),
            '\n' => out.push_str("\\n"),
            '\r' => out.push_str("\\r"),
            '\t' => out.push_str("\\t"),
            c if (c as u32) < 0x20 => out.push_str(&format!("\\u{:04x}", c as u32)),
            c => out.push(c),
        }
    }
    out.push('"');
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    fn record() -> DecodeRecord {
        let mut r = DecodeRecord::for_test(433_920_000.0, "Fineoffset-WHx080");
        r.fields = vec![
            ("temperature_c".into(), Value::Float(16.2)),
            ("humidity_pct".into(), Value::Int(89)),
            ("battery_ok".into(), Value::Bool(true)),
            ("note".into(), Value::Text("say \"hi\"\n".into())),
        ];
        r.bytes = vec![0xff, 0xa4, 0x01];
        r.crc = Some(true);
        r.rssi_dbfs = -18.0;
        r.snr_db = 21.5;
        r
    }

    #[test]
    fn a_record_becomes_one_json_line() {
        let line = to_json(&record(), 1_700_000_000.5);
        assert!(line.ends_with('\n'));
        assert_eq!(line.matches('\n').count(), 1, "a packet must not span lines");
        assert!(line.contains("\"model\":\"Fineoffset-WHx080\""));
        assert!(line.contains("\"bytes\":\"ffa401\""));
        assert!(line.contains("\"crc_ok\":true"));
        assert!(line.contains("\"temperature_c\":16.2"));
        assert!(line.contains("\"at\":1700000000.500"));
    }

    #[test]
    fn text_is_escaped_so_the_line_stays_parseable() {
        // A protocol's own text is arbitrary bytes from the air. A quote or a
        // newline in it must not be able to corrupt the file.
        let line = to_json(&record(), 0.0);
        assert!(line.contains(r#""note":"say \"hi\"\n""#), "{line}");
    }

    #[test]
    fn an_unchecked_protocol_omits_the_key_rather_than_lying() {
        let mut r = record();
        r.crc = None;
        let line = to_json(&r, 0.0);
        assert!(!line.contains("crc_ok"), "no check is not the same as a failed one");
    }

    #[test]
    fn a_non_finite_measurement_does_not_break_the_format() {
        let mut r = record();
        r.snr_db = f32::NAN;
        r.fields = vec![("x".into(), Value::Float(f64::INFINITY))];
        let line = to_json(&r, 0.0);
        assert!(!line.contains("NaN") && !line.contains("inf"), "{line}");
    }

    #[test]
    fn days_are_utc_and_roll_at_midnight() {
        let at = |s: u64| day_of(std::time::UNIX_EPOCH + std::time::Duration::from_secs(s));
        assert_eq!(at(0), "1970-01-01");
        assert_eq!(at(86_399), "1970-01-01");
        assert_eq!(at(86_400), "1970-01-02");
        // 2024-02-29, because a leap year is where a hand-rolled date breaks.
        assert_eq!(at(1_709_164_800), "2024-02-29");
        assert_eq!(at(1_709_251_200), "2024-03-01");
    }

    #[test]
    fn packets_are_appended_to_a_file_per_day() {
        let dir = std::env::temp_dir().join(format!("sr-packetlog-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        let mut log = PacketLog::new(dir.clone());
        let day = std::time::UNIX_EPOCH + std::time::Duration::from_secs(1_709_164_800);
        log.append(&[record(), record()], day);
        // The next day opens a new file rather than growing the old one.
        log.append(&[record()], day + std::time::Duration::from_secs(86_400));

        let first = std::fs::read_to_string(dir.join("2024-02-29.jsonl")).unwrap();
        let second = std::fs::read_to_string(dir.join("2024-03-01.jsonl")).unwrap();
        assert_eq!(first.lines().count(), 2);
        assert_eq!(second.lines().count(), 1);
        assert_eq!(log.logged(), 3);

        // Reopening appends rather than truncating: a restarted receiver must
        // not delete this morning's packets.
        let mut again = PacketLog::new(dir.clone());
        again.append(&[record()], day);
        let first = std::fs::read_to_string(dir.join("2024-02-29.jsonl")).unwrap();
        assert_eq!(first.lines().count(), 3);
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn a_directory_that_cannot_be_written_is_not_fatal() {
        // The log is a convenience. A receiver that refuses to run because it
        // cannot write one is worse than a receiver with no log.
        let mut log = PacketLog::new(PathBuf::from("/proc/nonexistent/packets"));
        log.append(&[record()], std::time::SystemTime::now());
        assert_eq!(log.logged(), 0);
    }
}
