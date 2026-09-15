//! The audio path after the demodulators: what is heard, and how loudly.
//!
//! Every stage of it is a node, so it can be seen, tapped and set from the
//! chain view like the rest of the receiver, and every level lives on the
//! one node that applies it:
//!
//! ```text
//! channel chain ──► fader ─0─► audio_bus ──► speaker
//!                       └─1─► heard ──► transcribe
//! voice front end ─────────► calls ──► audio_bus
//!               └──────────► heard
//! replay ──────────────────► audio_bus
//! ```
//!
//! A [`fader`] is one input's level, mute and name. [`calls`] is what the
//! subscriptions admit, at the calls level. [`heard`] is everything before
//! anybody decided, for the transcriber and the call list. [`bus`] sums.
//! [`speaker`] holds the sound card and the master. [`replay`] plays a
//! decoded transmission back once.

pub mod bus;
pub mod calls;
pub mod fader;
pub mod heard;
pub mod replay;
pub mod speaker;

use common::Speech;
use std::path::Path;

/// The rate the speaker is fed at, and everything is brought to.
pub const OUT_HZ: f64 = 48_000.0;

/// Register every stage in this module.
pub fn register(r: &mut pipeline::registry::Registry) {
    use pipeline::node::Node;
    use pipeline::registry::{Category, Settings, StageDesc};

    fn settle<N: Node>(mut n: N, s: &Settings) -> common::Result<Box<dyn Node>> {
        for (name, value) in s {
            let _ = n.set_param(name, value.clone());
        }
        Ok(Box::new(n))
    }

    r.register(
        StageDesc {
            name: fader::KIND,
            summary: "One input's level, mute and name, with a tap of what \
                      arrived for the transcriber",
            category: Category::Audio,
            feeds_bus: false,
        },
        |s: &Settings| settle(fader::FaderNode::new(), s),
    );
    r.register(
        StageDesc {
            name: calls::KIND,
            summary: "Every voice front end in one place: what the \
                      subscriptions admit, levelled, at the calls level",
            category: Category::Audio,
            feeds_bus: false,
        },
        |s: &Settings| settle(calls::CallsNode::new(OUT_HZ), s),
    );
    r.register(
        StageDesc {
            name: heard::KIND,
            summary: "Everything the receiver hears, labelled, before \
                      anybody decides whether to listen: what the \
                      transcriber and the call list read",
            category: Category::Audio,
            feeds_bus: false,
        },
        |s: &Settings| settle(heard::HeardNode::new(), s),
    );
    r.register(
        StageDesc {
            name: replay::KIND,
            summary: "A decoded transmission played back once",
            category: Category::Audio,
            feeds_bus: false,
        },
        |s: &Settings| settle(replay::ReplayNode::new(OUT_HZ), s),
    );
    r.register(
        StageDesc {
            name: bus::KIND,
            summary: "Where every path to the speaker meets: each input is \
                      brought to one rate and summed",
            category: Category::Audio,
            feeds_bus: false,
        },
        |s: &Settings| settle(bus::BusNode::new(OUT_HZ), s),
    );
    r.register(
        StageDesc {
            name: speaker::KIND,
            summary: "The sound card: the master level and the mute act here",
            category: Category::Sink,
            feeds_bus: false,
        },
        |s: &Settings| settle(speaker::SpeakerNode::new(None), s),
    );
}

/// Write speech to a 16 bit WAV, which is what everything else can open.
///
/// Voice is kept out of the packet log deliberately: an hour of a busy
/// repeater is gigabytes and the log is a record of what was on the air, not
/// of what it sounded like. A transmission worth keeping is worth a file of
/// its own, and a file is what a spectrogram, a player or another decoder can
/// be pointed at. Only the speech-to-text path writes one out.
#[cfg_attr(not(any(feature = "stt", test)), allow(dead_code))]
pub fn write_wav(path: &Path, speech: &Speech) -> std::io::Result<()> {
    use std::io::Write;
    if let Some(dir) = path.parent() {
        std::fs::create_dir_all(dir)?;
    }
    let mut f = std::io::BufWriter::new(std::fs::File::create(path)?);
    f.write_all(&wav_bytes(speech))?;
    f.flush()
}

/// The same WAV, in memory: what a transcription server is posted.
#[cfg_attr(not(any(feature = "stt", test)), allow(dead_code))]
pub fn wav_bytes(speech: &Speech) -> Vec<u8> {
    let rate = speech.rate.max(1.0) as u32;
    let data_len = speech.pcm.len() as u32 * 2;
    let mut v = Vec::with_capacity(44 + data_len as usize);
    v.extend(b"RIFF");
    v.extend((36 + data_len).to_le_bytes());
    v.extend(b"WAVEfmt ");
    v.extend(16u32.to_le_bytes());
    v.extend(1u16.to_le_bytes()); // PCM
    v.extend(1u16.to_le_bytes()); // mono
    v.extend(rate.to_le_bytes());
    v.extend((rate * 2).to_le_bytes()); // bytes per second
    v.extend(2u16.to_le_bytes()); // block align
    v.extend(16u16.to_le_bytes());
    v.extend(b"data");
    v.extend(data_len.to_le_bytes());
    for s in &speech.pcm {
        v.extend(((s.clamp(-1.0, 1.0) * 32767.0) as i16).to_le_bytes());
    }
    v
}

/// Peak and RMS of a transmission, in dBFS.
///
/// Shown beside the replay button, because "I can hear nothing" has two
/// causes that look identical from the speaker: nothing was decoded, or it
/// was decoded quietly and something later in the path lost it.
pub fn levels_db(speech: &Speech) -> (f32, f32) {
    let peak = speech.pcm.iter().fold(0.0f32, |a, v| a.max(v.abs()));
    let rms =
        (speech.pcm.iter().map(|v| v * v).sum::<f32>() / speech.pcm.len().max(1) as f32).sqrt();
    let db = |v: f32| if v > 0.0 { 20.0 * v.log10() } else { -120.0 };
    (db(peak), db(rms))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_transmission_writes_a_wav_that_says_what_it_holds() {
        let dir = std::env::temp_dir().join(format!("waveshark-call-{}", std::process::id()));
        let path = dir.join("call.wav");
        let speech = Speech { pcm: vec![0.5, -0.5, 0.25, 0.0], rate: 8_000.0 };
        write_wav(&path, &speech).expect("wav");
        let bytes = std::fs::read(&path).expect("read back");
        assert_eq!(&bytes[..4], b"RIFF");
        assert_eq!(&bytes[8..12], b"WAVE");
        // 44 byte header, then one sixteen bit sample per value.
        assert_eq!(bytes.len(), 44 + speech.pcm.len() * 2);
        assert_eq!(u32::from_le_bytes(bytes[24..28].try_into().unwrap()), 8_000);
        let (peak, rms) = levels_db(&speech);
        assert!((peak + 6.02).abs() < 0.1, "peak {peak}");
        assert!(rms < peak, "rms {rms} is not below the peak");
        let _ = std::fs::remove_dir_all(&dir);
    }
}
