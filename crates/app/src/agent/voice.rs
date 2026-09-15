//! Turning what the agent says into something a radio can transmit.
//!
//! Two ways, chosen in the settings. A model on this machine, through
//! `crates/tts`, which is how the receiver reads speech as well and needs
//! nothing running anywhere; or an OpenAI-compatible `/v1/audio/speech`,
//! which is the same shape of server the chat already talks to and is what a
//! machine with no card should use.
//!
//! Either way it becomes mono `f32` at a known rate, and the transmit chain
//! resamples it to whatever the radio is doing.
//!
//! Nothing here keys anything. It produces samples.

use super::config::{Config, Speech};
use serde_json::json;

#[cfg(feature = "tts")]
mod local;

/// What OpenAI's `pcm` format is, and the rate a WAV without a header would
/// be assumed to be.
const PCM_RATE: f64 = 24_000.0;

/// How long a sentence may take to come back. Generous: a local speech
/// server on a CPU is slower than the radio it is talking to.
const PATIENCE: std::time::Duration = std::time::Duration::from_secs(120);

/// Speech, and the rate it was produced at.
pub struct Said {
    pub samples: Vec<f32>,
    pub rate: f64,
}

/// Say `text`, however this receiver is set up to.
///
/// The local model runs on a thread of its own rather than on the runtime:
/// generation is seconds of arithmetic, and a runtime worker blocked in it is
/// a runtime worker not answering the chat.
pub async fn speak(config: &Config, text: &str) -> Result<Said, String> {
    match config.speech {
        Speech::Local => local_speak(config.clone(), text.to_string()).await,
        Speech::Server => server_speak(config, text).await,
    }
}

#[cfg(feature = "tts")]
async fn local_speak(config: Config, text: String) -> Result<Said, String> {
    let (tx, rx) = tokio::sync::oneshot::channel();
    std::thread::Builder::new()
        .name("tts".into())
        .spawn(move || {
            let _ = tx.send(local::speak(&config, &text));
        })
        .map_err(|e| e.to_string())?;
    rx.await.map_err(|_| "the speech model went away".to_string())?
}

#[cfg(not(feature = "tts"))]
async fn local_speak(_config: Config, _text: String) -> Result<Said, String> {
    Err("this build has no speech model: use a speech server".into())
}

/// Ask the server to say `text`.
async fn server_speak(config: &Config, text: &str) -> Result<Said, String> {
    if config.voice_url.trim().is_empty() || config.voice_model.trim().is_empty() {
        return Err("no speech server: the Agent settings take one".into());
    }
    let client = httpc::client(PATIENCE).map_err(|e| e.to_string())?;
    let url = format!("{}/audio/speech", config.voice_url.trim_end_matches('/'));
    let body = json!({
        "model": config.voice_model,
        "voice": config.voice,
        "input": text,
        // WAV rather than the default mp3: no decoder in this program, and
        // a header that says what rate it came back at.
        "response_format": "wav",
    });
    let mut req = client.post(url).json(&body);
    let key = match config.voice_key.trim() {
        "" => config.key.trim(),
        k => k,
    };
    if !key.is_empty() {
        req = req.bearer_auth(key);
    }
    let resp = req.send().await.map_err(|e| e.to_string())?;
    let status = resp.status();
    let bytes = resp.bytes().await.map_err(|e| e.to_string())?;
    if !status.is_success() {
        let said = String::from_utf8_lossy(&bytes);
        return Err(format!("{status}: {}", said.trim()));
    }
    decode(&bytes)
}

/// A WAV, or headerless 16-bit PCM, as mono `f32`.
///
/// Written here rather than taken from a crate because it is one header: the
/// program has no other use for an audio file reader, and a server that sends
/// raw PCM anyway has to be handled in either case.
pub fn decode(bytes: &[u8]) -> Result<Said, String> {
    if bytes.len() < 4 || &bytes[..4] != b"RIFF" {
        return Ok(Said { samples: pcm16(bytes, 1), rate: PCM_RATE });
    }
    if bytes.len() < 12 || &bytes[8..12] != b"WAVE" {
        return Err("not a WAV".into());
    }
    let mut at = 12;
    let mut rate = PCM_RATE;
    let mut channels = 1usize;
    let mut bits = 16u16;
    let mut float = false;
    while at + 8 <= bytes.len() {
        let id = &bytes[at..at + 4];
        let len = u32::from_le_bytes([bytes[at + 4], bytes[at + 5], bytes[at + 6], bytes[at + 7]])
            as usize;
        let body = at + 8;
        let end = body.saturating_add(len).min(bytes.len());
        match id {
            b"fmt " if len >= 16 => {
                let word = |i: usize| u16::from_le_bytes([bytes[body + i], bytes[body + i + 1]]);
                float = word(0) == 3;
                channels = word(2).max(1) as usize;
                rate = f64::from(u32::from_le_bytes([
                    bytes[body + 4],
                    bytes[body + 5],
                    bytes[body + 6],
                    bytes[body + 7],
                ]));
                bits = word(14);
            }
            b"data" => {
                let data = &bytes[body..end];
                let samples = match (float, bits) {
                    (true, 32) => pcm32f(data, channels),
                    (false, 16) => pcm16(data, channels),
                    _ => return Err(format!("{bits}-bit audio is not read here")),
                };
                return Ok(Said { samples, rate: rate.max(1.0) });
            }
            _ => {}
        }
        // Chunks are padded to an even length, and a reader that ignores that
        // reads the next header one byte late.
        at = body + len + (len & 1);
    }
    Err("a WAV with no data".into())
}

/// Interleaved 16-bit samples, folded to mono.
fn pcm16(data: &[u8], channels: usize) -> Vec<f32> {
    let channels = channels.max(1);
    data.chunks_exact(2)
        .map(|b| f32::from(i16::from_le_bytes([b[0], b[1]])) / 32_768.0)
        .collect::<Vec<f32>>()
        .chunks(channels)
        .map(|f| f.iter().sum::<f32>() / channels as f32)
        .collect()
}

fn pcm32f(data: &[u8], channels: usize) -> Vec<f32> {
    let channels = channels.max(1);
    data.chunks_exact(4)
        .map(|b| f32::from_le_bytes([b[0], b[1], b[2], b[3]]))
        .collect::<Vec<f32>>()
        .chunks(channels)
        .map(|f| f.iter().sum::<f32>() / channels as f32)
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A WAV as a server would send one: header, format, data.
    fn wav(rate: u32, channels: u16, samples: &[i16]) -> Vec<u8> {
        let data: Vec<u8> = samples.iter().flat_map(|s| s.to_le_bytes()).collect();
        let mut v = Vec::new();
        v.extend(b"RIFF");
        v.extend(((36 + data.len()) as u32).to_le_bytes());
        v.extend(b"WAVEfmt ");
        v.extend(16u32.to_le_bytes());
        v.extend(1u16.to_le_bytes());
        v.extend(channels.to_le_bytes());
        v.extend(rate.to_le_bytes());
        v.extend((rate * u32::from(channels) * 2).to_le_bytes());
        v.extend((channels * 2).to_le_bytes());
        v.extend(16u16.to_le_bytes());
        v.extend(b"data");
        v.extend((data.len() as u32).to_le_bytes());
        v.extend(data);
        v
    }

    #[test]
    fn a_wav_comes_back_as_samples_at_its_own_rate() {
        let said = decode(&wav(16_000, 1, &[0, 16_384, -16_384, 32_767])).expect("a wav");
        assert_eq!(said.rate, 16_000.0);
        assert_eq!(said.samples.len(), 4);
        assert!((said.samples[1] - 0.5).abs() < 1e-4);
        assert!((said.samples[2] + 0.5).abs() < 1e-4);
        assert!(said.samples[3] > 0.99);
    }

    /// Stereo is folded, because a transmitter has one channel and taking
    /// the left would lose half of a voice panned anywhere.
    #[test]
    fn stereo_is_folded_to_one_channel() {
        let said = decode(&wav(24_000, 2, &[16_384, 16_384, -32_768, 32_767])).expect("a wav");
        assert_eq!(said.samples.len(), 2);
        assert!((said.samples[0] - 0.5).abs() < 1e-4);
        assert!(said.samples[1].abs() < 1e-3, "opposite samples cancel");
    }

    /// A chunk of odd length is padded, and a reader that forgets reads the
    /// next header one byte late and finds nothing.
    #[test]
    fn an_odd_length_chunk_before_the_data_is_skipped_whole() {
        let mut v = wav(8_000, 1, &[1_000, 2_000]);
        // A LIST chunk of three bytes, between the format and the data.
        let at = v.iter().position(|_| false).unwrap_or(36);
        let mut extra = Vec::new();
        extra.extend(b"LIST");
        extra.extend(3u32.to_le_bytes());
        extra.extend([b'a', b'b', b'c', 0]);
        v.splice(at..at, extra);
        let said = decode(&v).expect("a wav with an extra chunk");
        assert_eq!(said.samples.len(), 2);
        assert_eq!(said.rate, 8_000.0);
    }

    /// A server that sends raw PCM is understood at the rate the API says it
    /// uses, rather than refused.
    #[test]
    fn headerless_pcm_is_read_at_the_documented_rate() {
        let data: Vec<u8> = [0i16, -32_768, 32_767].iter().flat_map(|s| s.to_le_bytes()).collect();
        let said = decode(&data).expect("raw pcm");
        assert_eq!(said.rate, 24_000.0);
        assert_eq!(said.samples.len(), 3);
        assert!((said.samples[1] + 1.0).abs() < 1e-6);
    }

    #[test]
    fn a_truncated_file_is_refused_rather_than_read_as_noise() {
        assert!(decode(b"RIFF\0\0\0\0WAVE").is_err());
        assert!(decode(b"RIFF\0\0\0\0NOPE????").is_err());
    }
}
