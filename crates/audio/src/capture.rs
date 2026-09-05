//! Microphone capture, for the transmit side.
//!
//! The mirror of [`crate::AudioPlayer`], and deliberately smaller. There is no
//! drift loop: what reads from this is a modulator being paced by the radio,
//! so a microphone running a few parts per million fast or slow shows up as
//! the ring filling or emptying, and the ring is what absorbs it.
//!
//! The device is opened when a transmission starts and closed when it ends.
//! A radio that holds the microphone open all the time is a radio listening
//! to the room all the time, which is not a thing to do quietly.

use crate::AudioError;
use cpal::traits::{DeviceTrait, HostTrait, StreamTrait};
use cpal::{Device, SampleFormat, StreamConfig};
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex};

/// Seconds of audio the ring holds.
///
/// A tenth of a second. Long enough to cover a scheduling hiccup on either
/// side, short enough that what goes on air is what was said rather than what
/// was said a moment ago: latency here is heard by whoever is listening, not
/// by the operator, so it cannot be tuned out by ear.
const RING_SECONDS: f64 = 0.1;

/// What the transmit chain reads from, without knowing where it came from.
///
/// A trait so a test can transmit a known waveform through the same path the
/// microphone uses, and so a recorded file or another receiver's audio can be
/// put on air later without the node changing.
pub trait AudioSource: Send + Sync {
    /// Samples per second the source produces.
    fn rate(&self) -> f64;

    /// Take up to `out.capacity()` samples, appending what is available.
    /// Returns how many were taken; fewer than asked for means the source has
    /// nothing more yet, which the caller fills with silence.
    fn take(&self, out: &mut Vec<f32>, want: usize) -> usize;

    /// Samples that were dropped because nobody read fast enough.
    fn overruns(&self) -> u64 {
        0
    }
}

struct Ring {
    buf: std::collections::VecDeque<f32>,
    cap: usize,
}

/// An open microphone.
///
/// Dropping it closes the device, which is how a transmission ends.
pub struct AudioCapture {
    /// The cpal stream, alive for as long as this is.
    _stream: cpal::Stream,
    shared: Arc<Shared>,
    device_name: String,
}

struct Shared {
    ring: Mutex<Ring>,
    rate: f64,
    overruns: AtomicU64,
}

impl AudioCapture {
    /// Open the default input at the rate asked for, or at whatever the
    /// device does if it will not do that one.
    pub fn open(want_rate: u32) -> Result<Self, AudioError> {
        let host = cpal::default_host();
        let device = host.default_input_device().ok_or(AudioError::NoDevice)?;
        Self::open_on(device, want_rate)
    }

    /// Microphones this machine has, without ALSA's plugin aliases.
    pub fn devices() -> Vec<String> {
        let host = cpal::default_host();
        let mut seen = std::collections::BTreeSet::new();
        host.input_devices()
            .map(|it| {
                it.map(|d| d.to_string())
                    .filter(|n| seen.insert(n.clone()) && is_real_device(n))
                    .collect()
            })
            .unwrap_or_default()
    }

    pub fn open_named(needle: &str, want_rate: u32) -> Result<Self, AudioError> {
        let host = cpal::default_host();
        let needle = needle.to_lowercase();
        let device = host
            .input_devices()
            .map_err(|e| AudioError::Cpal(e.to_string()))?
            .find(|d| d.to_string().to_lowercase().contains(&needle))
            .ok_or(AudioError::NoDevice)?;
        Self::open_on(device, want_rate)
    }

    fn open_on(device: Device, want_rate: u32) -> Result<Self, AudioError> {
        let device_name = device.to_string();
        let default = device
            .default_input_config()
            .map_err(|e| AudioError::Cpal(e.to_string()))?;

        let supports_want = device
            .supported_input_configs()
            .map(|it| {
                it.filter(|c| c.sample_format() == SampleFormat::F32).any(|c| {
                    c.min_sample_rate() <= want_rate && want_rate <= c.max_sample_rate()
                })
            })
            .unwrap_or(false);
        if default.sample_format() != SampleFormat::F32 && !supports_want {
            return Err(AudioError::NoFormat);
        }
        let (rate, channels) = match supports_want {
            true => (want_rate, default.channels().min(2)),
            false => (default.sample_rate(), default.channels().min(2)),
        };

        let shared = Arc::new(Shared {
            ring: Mutex::new(Ring {
                buf: std::collections::VecDeque::with_capacity(
                    (rate as f64 * RING_SECONDS) as usize,
                ),
                cap: (rate as f64 * RING_SECONDS) as usize,
            }),
            rate: rate as f64,
            overruns: AtomicU64::new(0),
        });

        let cb = shared.clone();
        let ch = channels as usize;
        let stream = device
            .build_input_stream(
                StreamConfig {
                    channels,
                    sample_rate: rate,
                    buffer_size: cpal::BufferSize::Default,
                },
                move |input: &[f32], _| {
                    let Ok(mut ring) = cb.ring.lock() else { return };
                    for frame in input.chunks(ch) {
                        // Down to mono by averaging: a stereo headset with a
                        // dead right channel would otherwise transmit at half
                        // level, and nothing on this path is stereo.
                        let v = frame.iter().sum::<f32>() / ch as f32;
                        if ring.buf.len() == ring.cap {
                            ring.buf.pop_front();
                            cb.overruns.fetch_add(1, Ordering::Relaxed);
                        }
                        ring.buf.push_back(v);
                    }
                },
                |e| tracing::warn!("microphone stream error: {e}"),
                None,
            )
            .map_err(|e| AudioError::Cpal(e.to_string()))?;
        stream.play().map_err(|e| AudioError::Cpal(e.to_string()))?;

        tracing::info!("microphone open: {device_name} at {rate} Hz, {channels} channels");
        Ok(Self { _stream: stream, shared, device_name })
    }

    pub fn device_name(&self) -> &str {
        &self.device_name
    }

    /// A handle the graph can hold, since the stream itself is not `Send`.
    pub fn source(&self) -> Arc<dyn AudioSource> {
        self.shared.clone()
    }

    /// Peak level since the last call, for a meter beside the key.
    pub fn peak(&self) -> f32 {
        let Ok(ring) = self.shared.ring.lock() else { return 0.0 };
        ring.buf.iter().fold(0.0f32, |m, v| m.max(v.abs()))
    }
}

impl AudioSource for Shared {
    fn rate(&self) -> f64 {
        self.rate
    }

    fn take(&self, out: &mut Vec<f32>, want: usize) -> usize {
        let Ok(mut ring) = self.ring.lock() else { return 0 };
        let n = want.min(ring.buf.len());
        out.extend(ring.buf.drain(..n));
        n
    }

    fn overruns(&self) -> u64 {
        self.overruns.load(Ordering::Relaxed)
    }
}

/// A source of a fixed waveform, for tests and for putting a known signal on
/// air without a microphone in the room.
pub struct Canned {
    samples: Mutex<std::collections::VecDeque<f32>>,
    rate: f64,
    repeat: bool,
    original: Vec<f32>,
}

impl Canned {
    pub fn new(samples: Vec<f32>, rate: f64, repeat: bool) -> Self {
        Self {
            samples: Mutex::new(samples.iter().copied().collect()),
            rate,
            repeat,
            original: samples,
        }
    }
}

impl AudioSource for Canned {
    fn rate(&self) -> f64 {
        self.rate
    }

    fn take(&self, out: &mut Vec<f32>, want: usize) -> usize {
        let Ok(mut q) = self.samples.lock() else { return 0 };
        while self.repeat && q.len() < want && !self.original.is_empty() {
            q.extend(self.original.iter().copied());
        }
        let n = want.min(q.len());
        out.extend(q.drain(..n));
        n
    }
}

/// Whether a name cpal reported is a sound card rather than one of ALSA's
/// plugins.
///
/// ALSA exposes its whole plugin chain as devices: rate converters, channel
/// up and downmixers, a Speex processor, the null sink, and a route to every
/// other sound system on the machine. Two dozen entries for a laptop with one
/// headset is not a picker anybody can use, and none of the plugins is a
/// thing an operator means to talk into.
///
/// Matched on the names ALSA gives them, which are stable because they come
/// from the description field of the plugin definitions rather than from
/// hardware. A card whose name happens to contain one of these words is lost,
/// which is why the list is words that only appear in plugin descriptions.
pub(crate) fn is_real_device(name: &str) -> bool {
    const PLUGINS: &[&str] = &[
        "Discard all samples",
        "Rate Converter",
        "Plugin using",
        "Plugin for channel",
        "Open Sound System",
        "JACK Audio Connection Kit",
        "Direct sample",
        "Direct hardware device without any conversions",
        "Hardware device with all software conversions",
    ];
    !PLUGINS.iter().any(|p| name.contains(p))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_picker_shows_sound_cards_and_not_alsa_plugins() {
        // The real list off a workstation, which is what made the point: one
        // interface and one motherboard codec, behind twenty-two entries.
        let reported = [
            "Discard all samples (playback) or generate zero samples (capture)",
            "Rate Converter Plugin Using Libav/FFmpeg Library",
            "JACK Audio Connection Kit",
            "Open Sound System",
            "PipeWire Sound Server",
            "PulseAudio Sound Server",
            "Plugin using Speex DSP (resample, agc, denoise, echo, dereverb)",
            "Plugin for channel upmix (4,6,8)",
            "Default ALSA Output (currently PipeWire Media Server)",
            "HD-Audio Generic, ALC1220 Analog",
            "Scarlett 2i2 USB, USB Audio",
            "Scarlett 2i2 USB",
        ];
        let kept: Vec<&str> = reported.into_iter().filter(|n| is_real_device(n)).collect();
        assert_eq!(
            kept,
            [
                "PipeWire Sound Server",
                "PulseAudio Sound Server",
                "Default ALSA Output (currently PipeWire Media Server)",
                "HD-Audio Generic, ALC1220 Analog",
                "Scarlett 2i2 USB, USB Audio",
                "Scarlett 2i2 USB",
            ]
        );
    }

    #[test]
    fn a_canned_source_hands_over_what_it_was_given() {
        let src = Canned::new(vec![0.1, 0.2, 0.3], 48_000.0, false);
        let mut out = Vec::new();
        assert_eq!(src.take(&mut out, 2), 2);
        assert_eq!(out, vec![0.1, 0.2]);
        // Short reads are how a starved source says so, rather than blocking
        // a modulator that has a radio waiting on it.
        assert_eq!(src.take(&mut out, 5), 1);
        assert_eq!(src.take(&mut out, 5), 0);
    }

    #[test]
    fn a_repeating_source_never_runs_out() {
        let src = Canned::new(vec![1.0, -1.0], 8_000.0, true);
        let mut out = Vec::new();
        assert_eq!(src.take(&mut out, 10), 10);
        assert_eq!(out.len(), 10);
    }
}
