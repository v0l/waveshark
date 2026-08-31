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

use common::{Packet, PacketBody, Pulse};

/// Stop appending when a day's file reaches this. A busy band writes a few
/// megabytes an hour and 1090 MHz writes rather more; this is a runaway
/// guard, not a budget, and it can be raised or lifted in the settings.
pub const DEFAULT_MAX_BYTES: u64 = 512 << 20;

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
    /// Bytes in the day's file, against the runaway guard.
    bytes: u64,
    full: bool,
    /// Packets appended since the receiver started.
    written: u64,
    /// Size at which a day's file stops growing, or `None` for no limit.
    cap: Option<u64>,
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
        Self { dir, open: None, bytes: 0, full: false, written: 0, cap: Some(DEFAULT_MAX_BYTES) }
    }

    /// Change the runaway guard. `None` lifts it, which is what a receiver
    /// left running on 1090 MHz for a week wants.
    pub fn with_cap(mut self, cap: Option<u64>) -> Self {
        self.cap = cap;
        // Raising the cap on a log that stopped should start it again, or the
        // setting would only take effect on the next restart.
        if self.cap.is_none_or(|c| self.bytes < c) {
            self.full = false;
        }
        self
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
            self.bytes = w.get_ref().metadata().map(|m| m.len()).unwrap_or(0);
            if fresh || self.bytes == 0 {
                if w.write_all(MAGIC).is_err() || w.write_all(&VERSION.to_le_bytes()).is_err() {
                    self.full = true;
                    return None;
                }
                self.bytes += MAGIC.len() as u64 + 2;
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
        self.bytes += rec.len() as u64;
        self.written += 1;
        if self.cap.is_some_and(|c| self.bytes >= c) {
            self.full = true;
        }
    }
}

impl nodes::PacketSink for PacketLog {
    fn bytes(&self) -> u64 {
        self.bytes
    }

    fn full(&self) -> bool {
        self.full
    }

    fn write(&mut self, p: &Packet) {
        let rec = match &p.body {
            PacketBody::Pulses(pulses) => {
                // A burst longer than this is not a packet; the count is
                // capped rather than the record refused, so whatever it was
                // is still on record with its level and frequency.
                let n = pulses.len().min(u16::MAX as usize);
                let mut rec = Vec::with_capacity(4 + HEAD_LEN + n * 8);
                put_head(&mut rec, KIND_PULSES, n as u16, n * 8, p);
                for pulse in &pulses[..n] {
                    rec.extend_from_slice(&pulse.mark.to_le_bytes());
                    rec.extend_from_slice(&pulse.gap.to_le_bytes());
                }
                rec
            }
            PacketBody::Frame(bytes) => {
                let n = bytes.len().min(u16::MAX as usize);
                let mut rec = Vec::with_capacity(4 + HEAD_LEN + n);
                put_head(&mut rec, KIND_BYTES, n as u16, n, p);
                rec.extend_from_slice(&bytes[..n]);
                rec
            }
        };
        self.append(p.at_us, &rec);
    }

    fn written(&self) -> u64 {
        self.written
    }
}

fn put_head(out: &mut Vec<u8>, kind: u8, count: u16, body_len: usize, p: &Packet) {
    out.extend_from_slice(&((HEAD_LEN + body_len) as u32).to_le_bytes());
    out.push(kind);
    out.push(0);
    out.extend_from_slice(&count.to_le_bytes());
    out.extend_from_slice(&p.at_us.to_le_bytes());
    out.extend_from_slice(&p.center_hz.to_le_bytes());
    out.extend_from_slice(&p.bandwidth_hz.to_le_bytes());
    out.extend_from_slice(&p.rssi_dbfs.to_le_bytes());
    out.extend_from_slice(&p.snr_db.to_le_bytes());
}

/// Read a log back.
///
/// A log nothing can read is a log nobody keeps, so the reader ships with the
/// writer and is tested against it. A truncated final record, which is what a
/// receiver killed mid-write leaves, ends the iteration rather than failing:
/// every complete record before it is still good.
pub fn read(path: impl AsRef<std::path::Path>) -> std::io::Result<Vec<Packet>> {
    let mut f = std::fs::File::open(path)?;
    let mut buf = Vec::new();
    f.read_to_end(&mut buf)?;
    Ok(parse(&buf))
}

pub fn parse(buf: &[u8]) -> Vec<Packet> {
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
        let packet_body = match kind {
            KIND_PULSES => {
                let mut pulses = Vec::new();
                for k in 0..count.min(body.len() / 8) {
                    let o = k * 8;
                    pulses.push(Pulse {
                        mark: u32::from_le_bytes(body[o..o + 4].try_into().unwrap()),
                        gap: u32::from_le_bytes(body[o + 4..o + 8].try_into().unwrap()),
                    });
                }
                PacketBody::Pulses(pulses)
            }
            KIND_BYTES => PacketBody::Frame(body[..count.min(body.len())].to_vec()),
            // An unknown kind is skipped by its length rather than guessed
            // at, which is the whole reason the length comes first.
            _ => continue,
        };
        out.push(Packet {
            at_us: get64(4),
            center_hz: get64(12),
            bandwidth_hz: get32(20),
            rssi_dbfs: getf(24),
            snr_db: getf(28),
            body: packet_body,
        });
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
    use nodes::PacketSink;

    fn dir(name: &str) -> PathBuf {
        let d = std::env::temp_dir().join(format!("sr-pktlog-{name}-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&d);
        d
    }

    fn burst(center: u64) -> Packet {
        Packet {
            at_us: AT,
            center_hz: center,
            bandwidth_hz: 31_250,
            rssi_dbfs: -21.25,
            snr_db: 18.5,
            body: PacketBody::Pulses(vec![
                Pulse { mark: 500, gap: 1500 },
                Pulse { mark: 1500, gap: 500 },
                Pulse { mark: 500, gap: 9000 },
            ]),
        }
    }

    fn frame(at_us: u64, bytes: &[u8]) -> Packet {
        Packet {
            at_us,
            center_hz: 1_090_000_000,
            bandwidth_hz: 2_000_000,
            rssi_dbfs: f32::NAN,
            snr_db: f32::NAN,
            body: PacketBody::Frame(bytes.to_vec()),
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
        log.write(&p);

        let got = read(d.join("2026-08-31.srpkt")).unwrap();
        assert_eq!(got.len(), 1);
        assert_eq!(got[0], p, "what came back is not what was heard");
        // And it is a package again, ready for a decoder that did not exist
        // when it was written.
        assert_eq!(got[0].package().map(|p| p.pulses.len()), Some(3));
        let _ = std::fs::remove_dir_all(&d);
    }

    #[test]
    fn a_byte_frame_is_stored_as_bytes() {
        // Mode S demodulates to frames rather than timings, and storing those
        // is storing the demodulator's output just the same.
        let d = dir("bytes");
        let mut log = PacketLog::new(d.clone());
        let bytes = [0x8d, 0x48, 0x40, 0xd6, 0x20, 0x2c, 0xc3];
        log.write(&frame(AT, &bytes));

        let got = read(d.join("2026-08-31.srpkt")).unwrap();
        assert_eq!(got.len(), 1);
        assert_eq!(got[0].frame(), Some(&bytes[..]));
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
            log.write(&burst(868_300_000));
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
        log.write(&burst(433_920_000));
        let mut tomorrow = burst(433_920_000);
        tomorrow.at_us += 86_400_000_000;
        log.write(&tomorrow);
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
