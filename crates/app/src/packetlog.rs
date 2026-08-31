//! Every burst the receiver hears, written to disk as it arrives.
//!
//! On by default and with no switch in the interface, because the value of a
//! log like this is entirely in having it already: the interesting
//! transmission is always the one that happened before anyone thought to
//! press record. A band left running overnight is a test corpus, and a corpus
//! of real bursts is the only honest way to tell whether a change to a
//! decoder helped.
//!
//! # What is stored
//!
//! What the demodulator produced, and nothing else: the mark and gap timings
//! of each burst, or the frame bytes where the demodulator makes bytes rather
//! than timings. The parsed frame is deliberately absent. A parse is a
//! conclusion, and a conclusion stored without the evidence cannot be checked
//! later, corrected by a better decoder, or used to prove that a decoder was
//! wrong. Timings can be decoded again next year; a field map cannot be
//! un-decoded.
//!
//! Undecoded bursts are written too, and they are the ones that matter most.
//! A burst that no protocol claimed leaves no other trace at all, and it is
//! the raw material for adding the protocol that would have claimed it.
//!
//! # The format
//!
//! A little-endian binary stream, one file per day, appended:
//!
//! ```text
//! file   := "SRPKT\0" u16 version
//! record := u32 body_len, u8 kind, u8 flags, u16 pulses_or_bytes,
//!           u64 at_us, u64 center_hz, u32 bandwidth_hz,
//!           f32 rssi_dbfs, f32 snr_db, body
//! body   := kind 1: [u32 mark_us, u32 gap_us] * n
//!           kind 2: [u8] * n
//! ```
//!
//! Binary rather than the line-delimited JSON this replaces, because the
//! content changed: a burst is a few hundred timings, and a hundred bytes of
//! JSON per pulse turns an overnight capture into gigabytes of quoting. The
//! length prefix means an unknown `kind` can be skipped rather than
//! misparsed, and a receiver killed mid-write costs the last record rather
//! than the file, since a short tail cannot be mistaken for a complete
//! record.
//!
//! Written by hand rather than through a serialisation crate: the record is
//! nine scalars and an array, and it is not worth a dependency in the crate
//! that has none.

use std::io::{Read, Write};
use std::path::PathBuf;

use common::{Package, Pulse};

/// Stop appending when a day's file reaches this. A busy band writes a few
/// megabytes an hour; this is a runaway guard, not a budget.
const MAX_BYTES: u64 = 512 << 20;

const MAGIC: &[u8; 6] = b"SRPKT\0";
const VERSION: u16 = 1;

/// Timings from a front end that detects bursts.
pub const KIND_PULSES: u8 = 1;
/// Bytes from a demodulator that produces frames directly, such as Mode S.
pub const KIND_BYTES: u8 = 2;

/// Bytes before the body of a record: kind, flags, count, time, frequency,
/// bandwidth, level and noise.
const HEAD_LEN: usize = 1 + 1 + 2 + 8 + 8 + 4 + 4 + 4;

pub struct PacketLog {
    dir: PathBuf,
    /// The day currently open, as `YYYY-MM-DD`, and its writer.
    open: Option<(String, std::io::BufWriter<std::fs::File>)>,
    written: u64,
    full: bool,
    /// Bursts appended since the receiver started.
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

    /// Open or roll the day's file, returning false once logging has stopped.
    ///
    /// Errors are swallowed on purpose. A full disk or a read-only home must
    /// not take the receiver down or spam the fault line: the log is a
    /// convenience, and losing it is not worth losing the packets on screen.
    fn writer(&mut self, at_us: u64) -> Option<&mut std::io::BufWriter<std::fs::File>> {
        if self.full {
            return None;
        }
        let day = day_of(at_us);
        if self.open.as_ref().is_none_or(|(d, _)| *d != day) {
            if std::fs::create_dir_all(&self.dir).is_err() {
                self.full = true;
                return None;
            }
            let path = self.dir.join(format!("{day}.srpkt"));
            let fresh = !path.exists();
            let Ok(f) = std::fs::OpenOptions::new().create(true).append(true).open(&path) else {
                self.full = true;
                return None;
            };
            let mut w = std::io::BufWriter::new(f);
            self.written = w.get_ref().metadata().map(|m| m.len()).unwrap_or(0);
            if fresh || self.written == 0 {
                if w.write_all(MAGIC).is_err() || w.write_all(&VERSION.to_le_bytes()).is_err() {
                    self.full = true;
                    return None;
                }
                self.written += MAGIC.len() as u64 + 2;
            }
            self.open = Some((day, w));
        }
        self.open.as_mut().map(|(_, w)| w)
    }

    fn append(&mut self, at_us: u64, rec: &[u8]) {
        let Some(w) = self.writer(at_us) else { return };
        if w.write_all(rec).is_err() {
            self.full = true;
            return;
        }
        // Flushed every record rather than left to the buffer: a receiver is
        // usually killed rather than closed, and an unflushed buffer is the
        // bursts nobody has.
        let _ = w.flush();
        self.written += rec.len() as u64;
        self.logged += 1;
        if self.written >= MAX_BYTES {
            self.full = true;
        }
    }
}

impl nodes::PackageSink for PacketLog {
    fn pulses(&mut self, at_us: u64, bandwidth_hz: u32, p: &Package) {
        // A burst longer than this is not a packet; the count is capped
        // rather than the record refused, so whatever it was is still on
        // record with its level and frequency.
        let n = p.pulses.len().min(u16::MAX as usize);
        let mut rec = Vec::with_capacity(4 + HEAD_LEN + n * 8);
        put_head(
            &mut rec,
            KIND_PULSES,
            n as u16,
            n * 8,
            at_us,
            p.center_hz,
            bandwidth_hz,
            p.rssi_dbfs,
            p.snr_db,
        );
        for pulse in &p.pulses[..n] {
            rec.extend_from_slice(&pulse.mark.to_le_bytes());
            rec.extend_from_slice(&pulse.gap.to_le_bytes());
        }
        self.append(at_us, &rec);
    }

    fn bytes(&mut self, at_us: u64, center_hz: u64, bytes: &[u8]) {
        let n = bytes.len().min(u16::MAX as usize);
        let mut rec = Vec::with_capacity(4 + HEAD_LEN + n);
        put_head(&mut rec, KIND_BYTES, n as u16, n, at_us, center_hz, 0, f32::NAN, f32::NAN);
        rec.extend_from_slice(&bytes[..n]);
        self.append(at_us, &rec);
    }
}

#[allow(clippy::too_many_arguments)]
fn put_head(
    out: &mut Vec<u8>,
    kind: u8,
    count: u16,
    body_len: usize,
    at_us: u64,
    center_hz: u64,
    bandwidth_hz: u32,
    rssi_dbfs: f32,
    snr_db: f32,
) {
    out.extend_from_slice(&((HEAD_LEN + body_len) as u32).to_le_bytes());
    out.push(kind);
    out.push(0);
    out.extend_from_slice(&count.to_le_bytes());
    out.extend_from_slice(&at_us.to_le_bytes());
    out.extend_from_slice(&center_hz.to_le_bytes());
    out.extend_from_slice(&bandwidth_hz.to_le_bytes());
    out.extend_from_slice(&rssi_dbfs.to_le_bytes());
    out.extend_from_slice(&snr_db.to_le_bytes());
}

/// One record, as read back.
#[derive(Clone, Debug, PartialEq)]
pub struct LoggedBurst {
    pub kind: u8,
    pub at_us: u64,
    pub center_hz: u64,
    pub bandwidth_hz: u32,
    pub rssi_dbfs: f32,
    pub snr_db: f32,
    pub pulses: Vec<Pulse>,
    pub bytes: Vec<u8>,
}

impl LoggedBurst {
    /// The burst as a package again, ready to be handed to a decoder.
    ///
    /// This is the point of logging timings rather than conclusions: what
    /// comes back off disk is what the front end produced, so a decoder can
    /// be run over it exactly as it would have run at the time, including one
    /// written years later.
    pub fn package(&self) -> Package {
        Package {
            pulses: self.pulses.clone(),
            snr_db: self.snr_db,
            rssi_dbfs: self.rssi_dbfs,
            start_sample: 0,
            center_hz: self.center_hz,
        }
    }
}

/// Read a log back.
///
/// A log nothing can read is a log nobody keeps, so the reader ships with the
/// writer and is tested against it. A truncated final record, which is what a
/// receiver killed mid-write leaves, ends the iteration rather than failing:
/// every complete record before it is still good.
pub fn read(path: impl AsRef<std::path::Path>) -> std::io::Result<Vec<LoggedBurst>> {
    let mut f = std::fs::File::open(path)?;
    let mut buf = Vec::new();
    f.read_to_end(&mut buf)?;
    Ok(parse(&buf))
}

pub fn parse(buf: &[u8]) -> Vec<LoggedBurst> {
    let mut out = Vec::new();
    if buf.len() < MAGIC.len() + 2 || &buf[..MAGIC.len()] != MAGIC {
        return out;
    }
    let mut at = MAGIC.len() + 2;
    while at + 4 <= buf.len() {
        let len = u32::from_le_bytes(buf[at..at + 4].try_into().unwrap()) as usize;
        at += 4;
        if len < HEAD_LEN || at + len > buf.len() {
            break;
        }
        let r = &buf[at..at + len];
        at += len;
        let kind = r[0];
        let count = u16::from_le_bytes(r[2..4].try_into().unwrap()) as usize;
        let get64 = |o: usize| u64::from_le_bytes(r[o..o + 8].try_into().unwrap());
        let get32 = |o: usize| u32::from_le_bytes(r[o..o + 4].try_into().unwrap());
        let getf = |o: usize| f32::from_le_bytes(r[o..o + 4].try_into().unwrap());
        let body = &r[HEAD_LEN..];
        let mut b = LoggedBurst {
            kind,
            at_us: get64(4),
            center_hz: get64(12),
            bandwidth_hz: get32(20),
            rssi_dbfs: getf(24),
            snr_db: getf(28),
            pulses: Vec::new(),
            bytes: Vec::new(),
        };
        match kind {
            KIND_PULSES => {
                for k in 0..count.min(body.len() / 8) {
                    let o = k * 8;
                    b.pulses.push(Pulse {
                        mark: u32::from_le_bytes(body[o..o + 4].try_into().unwrap()),
                        gap: u32::from_le_bytes(body[o + 4..o + 8].try_into().unwrap()),
                    });
                }
            }
            KIND_BYTES => b.bytes.extend_from_slice(&body[..count.min(body.len())]),
            // An unknown kind is skipped by its length rather than guessed
            // at, which is the whole reason the length comes first.
            _ => {}
        }
        out.push(b);
    }
    out
}

/// UTC date as `YYYY-MM-DD`, by civil-from-days rather than a calendar crate.
fn day_of(at_us: u64) -> String {
    let days = (at_us / 1_000_000) as i64 / 86_400;
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

#[cfg(test)]
mod tests {
    use super::*;
    use nodes::PackageSink;

    fn dir(name: &str) -> PathBuf {
        let d = std::env::temp_dir().join(format!("sr-pktlog-{name}-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&d);
        d
    }

    fn burst(center: u64) -> Package {
        Package {
            pulses: vec![
                Pulse { mark: 500, gap: 1500 },
                Pulse { mark: 1500, gap: 500 },
                Pulse { mark: 500, gap: 9000 },
            ],
            snr_db: 18.5,
            rssi_dbfs: -21.25,
            start_sample: 4096,
            center_hz: center,
        }
    }

    /// 2026-08-31T12:00:00Z, in microseconds.
    const AT: u64 = 1_788_177_600_000_000;

    #[test]
    fn a_burst_comes_back_exactly_as_it_was_detected() {
        // The whole point of storing the demodulator's output: what comes
        // back has to be good enough to decode again.
        let d = dir("roundtrip");
        let mut log = PacketLog::new(d.clone());
        let p = burst(433_920_000);
        log.pulses(AT, 31_250, &p);

        let got = read(d.join("2026-08-31.srpkt")).unwrap();
        assert_eq!(got.len(), 1);
        assert_eq!(got[0].kind, KIND_PULSES);
        assert_eq!(got[0].pulses, p.pulses);
        assert_eq!(got[0].center_hz, 433_920_000);
        assert_eq!(got[0].bandwidth_hz, 31_250);
        assert_eq!(got[0].at_us, AT);
        assert_eq!(got[0].rssi_dbfs, -21.25);
        assert_eq!(got[0].snr_db, 18.5);
        let _ = std::fs::remove_dir_all(&d);
    }

    #[test]
    fn a_byte_frame_is_stored_as_bytes() {
        // Mode S demodulates to frames rather than timings, and storing those
        // is storing the demodulator's output just the same.
        let d = dir("bytes");
        let mut log = PacketLog::new(d.clone());
        let frame = [0x8d, 0x48, 0x40, 0xd6, 0x20, 0x2c, 0xc3];
        log.bytes(AT, 1_090_000_000, &frame);

        let got = read(d.join("2026-08-31.srpkt")).unwrap();
        assert_eq!(got.len(), 1);
        assert_eq!(got[0].kind, KIND_BYTES);
        assert_eq!(got[0].bytes, frame);
        assert_eq!(got[0].center_hz, 1_090_000_000);
        let _ = std::fs::remove_dir_all(&d);
    }

    #[test]
    fn a_receiver_killed_mid_write_costs_one_record() {
        // How a log actually ends: not closed, but killed. Every complete
        // record before the tear has to survive.
        let d = dir("torn");
        let mut log = PacketLog::new(d.clone());
        for _ in 0..3 {
            log.pulses(AT, 31_250, &burst(868_300_000));
        }
        let path = d.join("2026-08-31.srpkt");
        let mut raw = std::fs::read(&path).unwrap();
        raw.truncate(raw.len() - 9);
        assert_eq!(parse(&raw).len(), 2, "a torn tail took a good record with it");
        let _ = std::fs::remove_dir_all(&d);
    }

    #[test]
    fn the_day_rolls_over_into_a_second_file() {
        let d = dir("roll");
        let mut log = PacketLog::new(d.clone());
        log.pulses(AT, 31_250, &burst(433_920_000));
        log.pulses(AT + 86_400_000_000, 31_250, &burst(433_920_000));
        assert!(d.join("2026-08-31.srpkt").exists());
        assert!(d.join("2026-09-01.srpkt").exists());
        let _ = std::fs::remove_dir_all(&d);
    }

    #[test]
    fn a_file_that_is_not_a_log_reads_as_empty() {
        assert!(parse(b"this is not a packet log at all").is_empty());
        assert!(parse(b"").is_empty());
    }

    #[test]
    fn dates_are_utc_civil_days() {
        assert_eq!(day_of(AT), "2026-08-31");
        assert_eq!(day_of(0), "1970-01-01");
        // A leap day, which is where a hand-rolled calendar goes wrong.
        assert_eq!(day_of(1_709_208_000_000_000), "2024-02-29");
    }
}
