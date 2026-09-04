//! Speech files, one per over, assembled from the packets that carried it.
//!
//! A voice front end logs one packet per burst, each with its slice of
//! speech, the way the packet log wants it. A listener wants the over in one
//! file. So the bursts of one call are appended here while they arrive and
//! written out when the transmission says it ended, or when nothing more
//! has come for as long as the call list keeps a call live.

use crate::radio::DecodeRecord;
use common::{Speech, Value};
use std::path::{Path, PathBuf};
use std::time::Instant;

struct Open {
    system: String,
    from: String,
    to: String,
    freq: f64,
    channel_hz: f64,
    started_secs: u64,
    model: String,
    pcm: Vec<f32>,
    rate: f64,
    last: Instant,
}

#[derive(Default)]
pub struct CallRecorder {
    open: Vec<Open>,
}

/// A finished over: where to put it and what it holds.
pub struct Finished {
    pub path: PathBuf,
    pub speech: Speech,
}

fn field(r: &DecodeRecord, k: &str) -> Option<String> {
    r.fields.iter().find(|(n, _)| n == k).map(|(_, v)| match v {
        Value::Text(t) => t.clone(),
        other => other.to_string(),
    })
}

fn clean(s: &str) -> String {
    s.chars().map(|c| if c.is_ascii_alphanumeric() || c == '-' { c } else { '_' }).collect()
}

impl CallRecorder {
    /// Take in one decode. A record with speech joins the call it belongs
    /// to, opening one if there is none; a record that says the transmission
    /// ended closes it. Returns whatever is now ready to write.
    pub fn feed(&mut self, r: &DecodeRecord, at: Instant, dir: &Path) -> Vec<Finished> {
        let voice = r.fields.iter().any(|(k, v)| matches!((k.as_str(), v), ("voice", Value::Bool(true))));
        if !voice && r.audio.is_none() {
            return Vec::new();
        }
        let system = r.model.split('-').next().unwrap_or(&r.model).to_string();
        let from = field(r, "from").unwrap_or_else(|| "unknown".into());
        let to = field(r, "to").unwrap_or_else(|| "unknown".into());
        let live = r.fields.iter().any(|(k, v)| matches!((k.as_str(), v), ("live", Value::Bool(true))));
        let same = |o: &Open| {
            o.system == system && o.from == from && o.to == to && (o.freq - r.freq).abs() < o.channel_hz.max(1.0)
        };
        let i = match self.open.iter().position(same) {
            Some(i) => i,
            None => {
                self.open.push(Open {
                    system,
                    from,
                    to,
                    freq: r.freq,
                    channel_hz: r.channel_hz,
                    started_secs: std::time::SystemTime::now()
                        .duration_since(std::time::UNIX_EPOCH)
                        .map(|d| d.as_secs())
                        .unwrap_or(0),
                    model: r.model.clone(),
                    pcm: Vec::new(),
                    rate: 0.0,
                    last: at,
                });
                self.open.len() - 1
            }
        };
        let o = &mut self.open[i];
        o.last = at;
        if let Some(a) = &r.audio {
            o.rate = a.rate;
            o.pcm.extend_from_slice(&a.pcm);
        }
        if live {
            return Vec::new();
        }
        let o = self.open.remove(i);
        Self::close(o, dir).into_iter().collect()
    }

    /// Close every call nothing has been heard from for a while.
    pub fn tick(&mut self, at: Instant, dir: &Path) -> Vec<Finished> {
        let mut out = Vec::new();
        let mut k = 0;
        while k < self.open.len() {
            if at.duration_since(self.open[k].last) >= crate::calls::LIVE {
                let o = self.open.remove(k);
                out.extend(Self::close(o, dir));
            } else {
                k += 1;
            }
        }
        out
    }

    fn close(o: Open, dir: &Path) -> Option<Finished> {
        if o.pcm.is_empty() || o.rate <= 0.0 {
            return None;
        }
        let name = format!("{}_{}_{}_{}.wav", o.started_secs, o.model, clean(&o.from), clean(&o.to));
        Some(Finished { path: dir.join(name), speech: Speech { pcm: o.pcm, rate: o.rate } })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn rec(model: &str, live: bool, pcm: Option<Vec<f32>>) -> DecodeRecord {
        let mut fields = vec![
            ("voice".to_string(), Value::Bool(true)),
            ("from".to_string(), Value::Text("1234567".into())),
            ("to".to_string(), Value::Text("9".into())),
        ];
        if live {
            fields.push(("live".to_string(), Value::Bool(true)));
        }
        DecodeRecord {
            at: Instant::now(),
            freq: 433_450_000.0,
            channel_hz: 12_500.0,
            model: model.into(),
            modulation: "4FSK",
            detail: String::new(),
            fields,
            media_type: "",
            rssi_dbfs: f32::NAN,
            snr_db: f32::NAN,
            bytes: Vec::new(),
            crc: None,
            iq: None,
            audio: pcm.map(|p| std::sync::Arc::new(Speech { pcm: p, rate: 8000.0 })),
        }
    }

    #[test]
    fn bursts_become_one_file_when_the_over_ends() {
        let mut c = CallRecorder::default();
        let now = Instant::now();
        let dir = Path::new("/tmp");
        assert!(c.feed(&rec("DMR-Header", true, None), now, dir).is_empty());
        for _ in 0..3 {
            assert!(c.feed(&rec("DMR-Voice", true, Some(vec![0.1; 480])), now, dir).is_empty());
        }
        let done = c.feed(&rec("DMR-Terminator", false, None), now, dir);
        assert_eq!(done.len(), 1);
        assert_eq!(done[0].speech.pcm.len(), 3 * 480);
        assert!(done[0].path.to_string_lossy().ends_with("_DMR-Header_1234567_9.wav"));
        assert!(c.open.is_empty());
    }

    #[test]
    fn an_over_with_no_end_closes_when_it_goes_quiet() {
        let mut c = CallRecorder::default();
        let now = Instant::now();
        let dir = Path::new("/tmp");
        c.feed(&rec("DMR-Voice", true, Some(vec![0.1; 480])), now, dir);
        assert!(c.tick(now + crate::calls::LIVE / 2, dir).is_empty());
        let done = c.tick(now + crate::calls::LIVE, dir);
        assert_eq!(done.len(), 1);
    }

    #[test]
    fn a_whole_over_in_one_record_is_one_file_at_once() {
        let mut c = CallRecorder::default();
        let done = c.feed(&rec("M17-Voice", false, Some(vec![0.0; 800])), Instant::now(), Path::new("/tmp"));
        assert_eq!(done.len(), 1);
    }
}
