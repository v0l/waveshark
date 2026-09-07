//! Transcription, as a consumer of the packet bus.
//!
//! Sits where `PacketDecodeNode` sits and for the same reason: there is one
//! of it, it sees every front end's output whether the call came from a
//! channel the operator placed or from a source the auto node found, and a
//! transmission that arrived by some other route (a log being replayed) is
//! transcribed too.
//!
//! Whisper is far too slow to run in the block loop, so the model lives on a
//! worker thread and the node holds a call's packet until the text comes
//! back. That reordering is the cost: a voice packet reaches the log a
//! fraction of a second after packets that were heard later. Everything
//! downstream keys on `at_us`, and a packet released without its transcript
//! is worse than one that arrives late, so the queue waits.

use common::{Decoded, Packet, Result, Value};
use crossbeam_channel::{bounded, Receiver, Sender, TryRecvError};
use pipeline::event::Event;
use pipeline::node::{NodeCtx, PortSpec, Simple};
use pipeline::param::{Param, ParamValue};
use pipeline::port::{Payload, PortKind, StreamSpec};
use std::path::PathBuf;
use std::sync::Arc;
use std::time::{Duration, Instant};

/// A call handed to the worker, and what came back.
struct Job {
    id: u64,
    speech: Arc<common::Speech>,
}

struct Done {
    id: u64,
    result: Result<stt::Transcript>,
}

struct Pending {
    id: u64,
    packet: Packet,
    give_up: Instant,
}

pub struct TranscribeNode {
    dir: PathBuf,
    language: Option<String>,
    enabled: bool,
    /// Shorter than this and there is nothing for a model to read: a 200 ms
    /// squelch tail transcribes as "Thank you." with high confidence.
    min_speech_s: f64,
    /// How long a packet waits for its text before going on without it.
    max_wait_s: f64,
    worker: Option<Worker>,
    pending: Vec<Pending>,
    next_id: u64,
    /// A failure to load the model is reported once, not once per call.
    reported: bool,
}

struct Worker {
    jobs: Sender<Job>,
    done: Receiver<Done>,
}

impl TranscribeNode {
    /// `dir` holds `config.json`, `tokenizer.json` and the weights. See
    /// `stt::Files`.
    pub fn new(dir: impl Into<PathBuf>) -> Self {
        Self {
            dir: dir.into(),
            language: Some("en".into()),
            enabled: true,
            min_speech_s: 0.4,
            max_wait_s: 20.0,
            worker: None,
            pending: Vec::new(),
            next_id: 0,
            reported: false,
        }
    }

    pub fn language(mut self, l: Option<&str>) -> Self {
        self.language = l.map(|s| s.to_string());
        self
    }

    fn worker(&mut self) -> Option<&Worker> {
        if self.worker.is_none() {
            let (jobs_tx, jobs_rx) = bounded::<Job>(64);
            let (done_tx, done_rx) = bounded::<Done>(64);
            let dir = self.dir.clone();
            let language = self.language.clone();
            std::thread::Builder::new()
                .name("whisper".into())
                .spawn(move || run_worker(dir, language, jobs_rx, done_tx))
                .ok()?;
            self.worker = Some(Worker {
                jobs: jobs_tx,
                done: done_rx,
            });
        }
        self.worker.as_ref()
    }

    /// The speech a packet carries, wherever it is: the front end puts it on
    /// the packet, a decoder that produced it puts it on its decode, and both
    /// happen.
    fn speech_of(p: &Packet) -> Option<Arc<common::Speech>> {
        p.audio
            .clone()
            .or_else(|| p.decodes.iter().find_map(|d| d.audio.clone()))
    }
}

impl Simple for TranscribeNode {
    fn name(&self) -> &str {
        "transcribe"
    }

    fn negotiate(&mut self, i: &PortSpec) -> Result<StreamSpec> {
        if i.spec.kind != PortKind::Packets {
            return Err(common::Error::other("transcribe reads the packet bus"));
        }
        Ok(i.spec.clone())
    }

    fn process(&mut self, i: &Payload, o: &mut Payload, c: &mut NodeCtx<'_>) -> Result<()> {
        let out = o.packets_mut();
        let incoming = i.as_packets().unwrap_or(&[]);

        for p in incoming {
            let speech = if self.enabled {
                Self::speech_of(p)
            } else {
                None
            };
            let long_enough = speech
                .as_ref()
                .is_some_and(|s| s.seconds() >= self.min_speech_s);
            if !long_enough {
                out.push(p.clone());
                continue;
            }
            let speech = speech.unwrap();
            let id = self.next_id;
            self.next_id += 1;
            let wait = Duration::from_secs_f64(self.max_wait_s);
            let queued = match self.worker() {
                Some(w) => w.jobs.try_send(Job { id, speech }).is_ok(),
                None => false,
            };
            if queued {
                self.pending.push(Pending {
                    id,
                    packet: p.clone(),
                    give_up: Instant::now() + wait,
                });
            } else {
                // A full queue means the model cannot keep up with the
                // traffic. Passing the call on untranscribed keeps the log
                // honest; silently dropping it would not.
                out.push(p.clone());
            }
        }

        loop {
            let done = match self.worker.as_ref().map(|w| w.done.try_recv()) {
                Some(Ok(d)) => d,
                Some(Err(TryRecvError::Empty)) | None => break,
                Some(Err(TryRecvError::Disconnected)) => {
                    self.worker = None;
                    break;
                }
            };
            let Some(idx) = self.pending.iter().position(|p| p.id == done.id) else {
                continue;
            };
            let mut held = self.pending.remove(idx);
            match done.result {
                Ok(t) => attach(&mut held.packet, &t),
                Err(e) => {
                    if !self.reported {
                        self.reported = true;
                        c.emit(Event::Warning {
                            stage: "transcribe".into(),
                            message: format!("{e}"),
                        });
                    }
                }
            }
            out.push(held.packet);
        }

        let now = Instant::now();
        let mut i = 0;
        while i < self.pending.len() {
            if self.pending[i].give_up <= now {
                out.push(self.pending.remove(i).packet);
            } else {
                i += 1;
            }
        }

        out.sort_by_key(|p| p.at_us);
        Ok(())
    }

    fn reset(&mut self) {
        self.pending.clear();
    }

    fn params(&self) -> Vec<Param> {
        vec![
            Param::bool("enabled", self.enabled).label("Transcribe speech"),
            Param::float("min_speech_s", self.min_speech_s, 0.1..=5.0)
                .unit("s")
                .label("Shortest call worth reading"),
            Param::float("max_wait_s", self.max_wait_s, 1.0..=120.0)
                .unit("s")
                .label("Longest wait for text"),
        ]
    }

    fn set_param(&mut self, name: &str, v: ParamValue) -> Result<()> {
        match name {
            "enabled" => self.enabled = v.as_bool().unwrap_or(self.enabled),
            "min_speech_s" => self.min_speech_s = v.as_f64().unwrap_or(self.min_speech_s),
            "max_wait_s" => self.max_wait_s = v.as_f64().unwrap_or(self.max_wait_s),
            _ => {
                return Err(common::Error::other(format!(
                    "transcribe: unknown parameter {name:?}"
                )))
            }
        }
        Ok(())
    }
}

/// Put the text where a view will find it: on the decode that carried the
/// speech, or on a decode of its own when the call produced none.
fn attach(p: &mut Packet, t: &stt::Transcript) {
    if t.is_empty() {
        return;
    }
    let fields = vec![
        ("transcript".to_string(), Value::Text(t.text.clone())),
        (
            "transcript_logprob".to_string(),
            Value::Float(t.avg_logprob()),
        ),
    ];
    if let Some(d) = p.decodes.iter_mut().find(|d| d.audio.is_some()) {
        d.fields.extend(fields);
        return;
    }
    if let Some(d) = p.decodes.first_mut() {
        d.fields.extend(fields);
        return;
    }
    let mut d = Decoded::bytes("speech", common::Hz(0), p.at_us as f64 / 1e6, Vec::new());
    d.media_type = common::decode::media::TEXT;
    d.text = Some(t.text.clone());
    d.audio = p.audio.clone();
    d.fields = fields;
    p.decodes.push(d);
}

fn run_worker(dir: PathBuf, language: Option<String>, jobs: Receiver<Job>, done: Sender<Done>) {
    let loaded = stt::Files::in_dir(&dir)
        .and_then(|f| stt::Whisper::load(&f, candle_core::Device::Cpu, language.as_deref()));
    let mut model = match loaded {
        Ok(m) => m,
        Err(e) => {
            // Answer every job with the load failure rather than dying, so
            // the node reports why it is transcribing nothing.
            let msg = format!("whisper in {}: {e}", dir.display());
            while let Ok(job) = jobs.recv() {
                let _ = done.send(Done {
                    id: job.id,
                    result: Err(common::Error::other(&msg)),
                });
            }
            return;
        }
    };
    while let Ok(job) = jobs.recv() {
        let r = model.transcribe(&job.speech.pcm, job.speech.rate);
        if done
            .send(Done {
                id: job.id,
                result: r,
            })
            .is_err()
        {
            break;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use pipeline::port::StreamSpec;

    fn packet_with_speech(at_us: u64, seconds: f64) -> Packet {
        let mut p = Packet::of_frame(
            at_us,
            12_500,
            common::Frame {
                bytes: vec![1, 2, 3],
                center_hz: 434_000_000,
                rssi_dbfs: -40.0,
                snr_db: 20.0,
                iq: None,
            },
        );
        p.audio = Some(Arc::new(common::Speech {
            pcm: vec![0.0; (8000.0 * seconds) as usize],
            rate: 8000.0,
        }));
        p
    }

    fn run(node: &mut TranscribeNode, input: Vec<Packet>) -> Vec<Packet> {
        let ins = [PortSpec {
            spec: StreamSpec::iq(0.0, common::Hz(0)).with_kind(PortKind::Packets),
            latency: 0,
        }];
        let mut events = Vec::new();
        let mut tags = Vec::new();
        let mut ctx = NodeCtx::new(0, &ins, &[], &mut events, &mut tags);
        let mut out = Payload::Packets(Vec::new());
        node.process(&Payload::Packets(input), &mut out, &mut ctx)
            .unwrap();
        match out {
            Payload::Packets(p) => p,
            _ => unreachable!(),
        }
    }

    /// A call whose transcription fails still reaches the log. A missing
    /// model must cost the text, not the packet.
    #[test]
    fn a_packet_survives_a_model_that_will_not_load() {
        let mut n = TranscribeNode::new("/nonexistent/whisper");
        let held = run(&mut n, vec![packet_with_speech(1_000, 2.0)]);
        assert!(
            held.is_empty(),
            "the call is held while the worker is asked"
        );

        let deadline = Instant::now() + Duration::from_secs(5);
        let mut seen = Vec::new();
        while seen.is_empty() && Instant::now() < deadline {
            std::thread::sleep(Duration::from_millis(20));
            seen = run(&mut n, Vec::new());
        }
        assert_eq!(seen.len(), 1, "the call came back untranscribed");
        assert_eq!(seen[0].at_us, 1_000);
    }

    /// Too short to read, so it is not queued and not delayed.
    #[test]
    fn a_squelch_tail_passes_straight_through() {
        let mut n = TranscribeNode::new("/nonexistent/whisper");
        let out = run(&mut n, vec![packet_with_speech(7, 0.1)]);
        assert_eq!(out.len(), 1);
        assert!(n.pending.is_empty());
    }
}
