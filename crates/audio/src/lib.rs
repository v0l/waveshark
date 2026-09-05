//! Audio playback.
//!
//! The callback never allocates, locks or blocks: it pops a pre-filled buffer
//! and returns the drained allocation for reuse.

mod resample;

pub use resample::Resampler;

use cpal::traits::{DeviceTrait, HostTrait, StreamTrait};
use cpal::{Device, SampleFormat, StreamConfig};
use crossbeam_channel::{bounded, Receiver, Sender, TryRecvError};
use std::sync::atomic::{AtomicBool, AtomicI64, AtomicU32, AtomicU64, Ordering};
use std::sync::Arc;

/// Queue depth before the producer drops.
const QUEUE_DEPTH: usize = 32;
/// Target backlog held by the drift loop, in samples per channel.
///
/// The producer adds a whole block at a time and the callback drains it in
/// device-period chunks, so the backlog is a sawtooth whose mean is about half
/// a block. Targeting the mean rather than the peak keeps the loop near
/// equilibrium instead of chasing an unreachable level.
const TARGET_BACKLOG: f64 = 1024.0;

#[derive(Debug, thiserror::Error)]
pub enum AudioError {
    #[error("no output device available")]
    NoDevice,
    #[error("device does not support any usable output format")]
    NoFormat,
    #[error("cpal: {0}")]
    Cpal(String),
}

/// Both counters rising means the rates disagree.
#[derive(Debug, Default)]
pub struct AudioStats {
    /// Samples per channel currently queued.
    pub backlog: AtomicI64,
    /// Buffers dropped because the queue was full: the producer is too fast.
    pub dropped: AtomicU64,
    /// Callbacks that ran out of samples and emitted silence: too slow.
    pub underruns: AtomicU64,
    pub samples_played: AtomicU64,
    /// Set by the producer, read by the device callback: while it is on the
    /// device is fed silence and whatever is queued is thrown away.
    pub muted: AtomicBool,
    /// Master level the callback applies, as `f32` bits.
    pub volume: AtomicU32,
}

/// Producer half. `Send`, unlike the stream.
pub struct AudioSink {
    tx: Sender<Vec<f32>>,
    recycle: Receiver<Vec<f32>>,
    stats: Arc<AudioStats>,
    rate: u32,
    channels: u16,
    /// Partially filled buffer being assembled.
    staging: Vec<f32>,
    block: usize,
    /// Drift tracking state for [`AudioSink::write_adaptive`].
    resampler: Option<Resampler>,
    /// Second resampler for the right channel. Both are driven with the same
    /// ratio, so the channels cannot drift apart and smear the stereo image.
    resampler_r: Option<Resampler>,
    trim: f64,
    resampled: Vec<f32>,
    resampled_r: Vec<f32>,
    muted: bool,
    volume: f32,
    deint_l: Vec<f32>,
    deint_r: Vec<f32>,
}

impl AudioSink {
    /// Rate the sink expects; anything else plays at the wrong pitch.
    pub fn rate(&self) -> u32 {
        self.rate
    }

    pub fn channels(&self) -> u16 {
        self.channels
    }

    pub fn stats(&self) -> &AudioStats {
        &self.stats
    }

    /// Samples per channel currently queued.
    pub fn backlog(&self) -> i64 {
        self.stats.backlog.load(Ordering::Relaxed)
    }

    /// The master level and mute, applied to the device rather than to what
    /// is queued for it.
    ///
    /// Applied upstream of the queue, a change is heard a queue length late,
    /// which is a fifth of a second of whatever was playing, and it reaches
    /// only the audio that came through the mix: a stage wired downstream of
    /// the bus went on playing through a mute. Blocks keep being written
    /// while it is muted, so the drift loop stays converged and unmuting is
    /// immediate.
    pub fn set_output(&mut self, volume: f32, muted: bool) {
        let volume = volume.clamp(0.0, 1.0);
        if muted != self.muted {
            self.muted = muted;
            self.stats.muted.store(muted, Ordering::Relaxed);
        }
        if volume != self.volume {
            self.volume = volume;
            self.stats.volume.store(volume.to_bits(), Ordering::Relaxed);
        }
    }

    pub fn muted(&self) -> bool {
        self.muted
    }

    pub fn volume(&self) -> f32 {
        self.volume
    }

    /// Queue mono samples, resampling to absorb clock drift.
    ///
    /// The radio and the sound card run off independent crystals, so their
    /// rates differ by tens to hundreds of ppm no matter how exact the decimation
    /// chain is. Left alone that slowly drains or overfills the queue, which is
    /// audible as intermittent clicks long before either counter moves. The
    /// resample ratio is steered by queue depth to hold it near half full.
    pub fn write_adaptive(&mut self, samples: &[f32], in_rate: f64) {
        let out_rate = self.rate as f64;
        if self.resampler.is_none() {
            self.resampler = Some(Resampler::new(in_rate, out_rate, 8));
        }

        // Integral-only, and deliberately very slow. This corrects crystal
        // drift, which is tens of ppm and constant; it must not chase the
        // block-to-block jitter of the queue, which is far larger. The gain
        // gives a time constant of minutes, and the clamp is wider than any
        // real crystal error so hitting it means something else is wrong.
        let correction = self.drift_correction();

        let r = self.resampler.as_mut().unwrap();
        r.set_ratio(in_rate / out_rate * (1.0 + correction));
        self.resampled.clear();
        let mut out = std::mem::take(&mut self.resampled);
        r.process(samples, &mut out);
        self.write(&out);
        self.resampled = out;
    }

    /// Steer the resample ratio from queue depth.
    ///
    /// Integral-dominant and deliberately very slow: this corrects crystal
    /// drift, which is tens of ppm and constant, and must not chase the
    /// block-to-block jitter of the queue, which is far larger. The clamp is
    /// wider than any real crystal error, so hitting it means something else
    /// is wrong. The proportional term supplies damping the integral lacks
    /// without contributing to windup.
    fn drift_correction(&mut self) -> f64 {
        let backlog = self.stats.backlog.load(Ordering::Relaxed) as f64;
        let err = (backlog - TARGET_BACKLOG) / TARGET_BACKLOG;
        self.trim = (self.trim + 2e-7 * err).clamp(-1e-3, 1e-3);
        (self.trim + 5e-6 * err).clamp(-2e-3, 2e-3)
    }

    /// Queue interleaved stereo, resampling both channels by the same ratio.
    ///
    /// The frames are split, resampled separately and interleaved again.
    /// Resampling the interleaved stream directly would filter across the
    /// channel boundary and mix left into right.
    pub fn write_adaptive_stereo(&mut self, frames: &[f32], in_rate: f64) {
        if self.channels < 2 {
            // A mono device cannot carry the image; fold down rather than
            // playing only the left channel.
            self.deint_l.clear();
            self.deint_l.extend(frames.chunks_exact(2).map(|f| 0.5 * (f[0] + f[1])));
            let mono = std::mem::take(&mut self.deint_l);
            self.write_adaptive(&mono, in_rate);
            self.deint_l = mono;
            return;
        }

        let out_rate = self.rate as f64;
        if self.resampler.is_none() {
            self.resampler = Some(Resampler::new(in_rate, out_rate, 8));
        }
        if self.resampler_r.is_none() {
            self.resampler_r = Some(Resampler::new(in_rate, out_rate, 8));
        }

        let ratio = in_rate / out_rate * (1.0 + self.drift_correction());

        self.deint_l.clear();
        self.deint_r.clear();
        for f in frames.chunks_exact(2) {
            self.deint_l.push(f[0]);
            self.deint_r.push(f[1]);
        }

        let mut l = std::mem::take(&mut self.resampled);
        let mut r = std::mem::take(&mut self.resampled_r);
        l.clear();
        r.clear();
        let rl = self.resampler.as_mut().unwrap();
        rl.set_ratio(ratio);
        rl.process(&self.deint_l, &mut l);
        let rr = self.resampler_r.as_mut().unwrap();
        rr.set_ratio(ratio);
        rr.process(&self.deint_r, &mut r);

        let n = l.len().min(r.len());
        for i in 0..n {
            self.staging.push(l[i]);
            self.staging.push(r[i]);
            if self.staging.len() >= self.block {
                self.flush();
            }
        }
        self.resampled = l;
        self.resampled_r = r;
    }

    /// Current drift correction in ppm, for diagnostics.
    pub fn drift_ppm(&self) -> f64 {
        self.trim * 1e6
    }

    /// Queue mono samples. Duplicated across channels if the device is stereo.
    pub fn write(&mut self, samples: &[f32]) {
        for &s in samples {
            for _ in 0..self.channels {
                self.staging.push(s);
            }
            if self.staging.len() >= self.block {
                self.flush();
            }
        }
    }

    /// Push whatever is staged, even if the buffer is short.
    pub fn flush(&mut self) {
        if self.staging.is_empty() {
            return;
        }
        // Reuse a drained buffer if one has come back, else grow the pool.
        let mut buf = match self.recycle.try_recv() {
            Ok(mut b) => {
                b.clear();
                b
            }
            Err(_) => Vec::with_capacity(self.block),
        };
        std::mem::swap(&mut buf, &mut self.staging);
        self.staging.clear();

        let n = (buf.len() / self.channels.max(1) as usize) as i64;
        self.stats.backlog.fetch_add(n, Ordering::Relaxed);
        if let Err(e) = self.tx.try_send(buf) {
            self.stats.backlog.fetch_sub(n, Ordering::Relaxed);
            let _ = e;
            // Drop rather than block: stalling the DSP thread behind the sound
            // card would lose radio samples.
            self.stats.dropped.fetch_add(1, Ordering::Relaxed);
        }
    }
}

/// Owns the cpal stream. Keep it alive for as long as you want sound.
pub struct AudioPlayer {
    _stream: cpal::Stream,
    stats: Arc<AudioStats>,
    rate: u32,
    channels: u16,
    device_name: String,
}

impl AudioPlayer {
    /// Open the default device at `want_rate` if supported, else the device
    /// default. Check [`Self::rate`] afterwards.
    pub fn open(want_rate: u32) -> Result<(Self, AudioSink), AudioError> {
        let host = cpal::default_host();
        let device = host.default_output_device().ok_or(AudioError::NoDevice)?;
        Self::open_on(device, want_rate)
    }

    /// List output devices, deduplicated. ALSA reports the same card under
    /// dozens of plugin aliases, which is noise in a device picker.
    pub fn devices() -> Vec<String> {
        let host = cpal::default_host();
        let mut seen = std::collections::BTreeSet::new();
        host.output_devices()
            .map(|it| {
                it.map(|d| d.to_string())
                    .filter(|n| seen.insert(n.clone()))
                    .collect()
            })
            .unwrap_or_default()
    }

    /// Open a device whose name contains `needle`, case-insensitively.
    pub fn open_named(needle: &str, want_rate: u32) -> Result<(Self, AudioSink), AudioError> {
        let host = cpal::default_host();
        let needle = needle.to_lowercase();
        let device = host
            .output_devices()
            .map_err(|e| AudioError::Cpal(e.to_string()))?
            .find(|d| d.to_string().to_lowercase().contains(&needle))
            .ok_or(AudioError::NoDevice)?;
        Self::open_on(device, want_rate)
    }

    fn open_on(device: Device, want_rate: u32) -> Result<(Self, AudioSink), AudioError> {
        let device_name = device.to_string();
        let default = device
            .default_output_config()
            .map_err(|e| AudioError::Cpal(e.to_string()))?;

        // f32 avoids a conversion in the callback.
        let supports_want = device
            .supported_output_configs()
            .map(|it| {
                it.filter(|c| c.sample_format() == SampleFormat::F32).any(|c| {
                    c.min_sample_rate() <= want_rate && want_rate <= c.max_sample_rate()
                })
            })
            .unwrap_or(false);

        let (rate, channels) = if supports_want {
            (want_rate, default.channels().min(2))
        } else {
            (default.sample_rate(), default.channels().min(2))
        };

        if default.sample_format() != SampleFormat::F32 && !supports_want {
            return Err(AudioError::NoFormat);
        }

        let config = StreamConfig {
            channels,
            sample_rate: rate,
            buffer_size: cpal::BufferSize::Default,
        };

        let (tx, rx) = bounded::<Vec<f32>>(QUEUE_DEPTH);
        let (recycle_tx, recycle_rx) = bounded::<Vec<f32>>(QUEUE_DEPTH * 2);
        let stats = Arc::new(AudioStats::default());
        stats.volume.store(1.0f32.to_bits(), Ordering::Relaxed);

        let cb_stats = stats.clone();
        let ch = channels;
        // Leftovers from a buffer the callback only partly consumed.
        let mut current: Vec<f32> = Vec::new();
        let mut pos = 0usize;
        // Underruns before the source has produced anything are just startup
        // (a radio takes a second to lock), and counting them buries the ones
        // that indicate a real problem.
        let mut had_real_data = false;

        let stream = device
            .build_output_stream(
                config.clone(),
                move |out: &mut [f32], _| {
                    if cb_stats.muted.load(Ordering::Relaxed) {
                        out.fill(0.0);
                        // Drain rather than hold: on unmute the queue would
                        // otherwise play the seconds that went by while it
                        // was muted before catching up to now.
                        pos = current.len();
                        while let Ok(b) = rx.try_recv() {
                            let n = (b.len() / ch as usize) as i64;
                            cb_stats.backlog.fetch_sub(n, Ordering::Relaxed);
                            let _ = recycle_tx.try_send(b);
                        }
                        cb_stats.samples_played.fetch_add(out.len() as u64, Ordering::Relaxed);
                        return;
                    }
                    let volume = f32::from_bits(cb_stats.volume.load(Ordering::Relaxed));
                    let mut written = 0;
                    while written < out.len() {
                        if pos >= current.len() {
                            match rx.try_recv() {
                                Ok(next) => {
                                    had_real_data = true;
                                    let done = std::mem::replace(&mut current, next);
                                    if !done.is_empty() {
                                        let _ = recycle_tx.try_send(done);
                                    }
                                    pos = 0;
                                }
                                Err(TryRecvError::Empty) | Err(TryRecvError::Disconnected) => {
                                    // Silence beats repeating stale audio.
                                    out[written..].fill(0.0);
                                    if volume != 1.0 {
                                        for s in out[..written].iter_mut() {
                                            *s *= volume;
                                        }
                                    }
                                    if had_real_data {
                                        cb_stats.underruns.fetch_add(1, Ordering::Relaxed);
                                    }
                                    return;
                                }
                            }
                        }
                        let n = (out.len() - written).min(current.len() - pos);
                        out[written..written + n].copy_from_slice(&current[pos..pos + n]);
                        written += n;
                        pos += n;
                    }
                    if volume != 1.0 {
                        for s in out.iter_mut() {
                            *s *= volume;
                        }
                    }
                    cb_stats
                        .samples_played
                        .fetch_add(out.len() as u64, Ordering::Relaxed);
                    cb_stats
                        .backlog
                        .fetch_sub((out.len() / ch as usize) as i64, Ordering::Relaxed);
                },
                move |e| tracing::error!("audio stream error: {e}"),
                None,
            )
            .map_err(|e| AudioError::Cpal(e.to_string()))?;

        // Prime the queue before starting, or the first callbacks run dry and
        // the stream opens with a burst of underruns.
        let block = 2048 * channels as usize;
        stream.play().map_err(|e| AudioError::Cpal(e.to_string()))?;

        let sink = AudioSink {
            tx,
            recycle: recycle_rx,
            stats: stats.clone(),
            rate,
            channels,
            staging: Vec::with_capacity(4096),
            block,
            resampler: None,
            resampler_r: None,
            resampled_r: Vec::new(),
            deint_l: Vec::new(),
            deint_r: Vec::new(),
            trim: 0.0,
            resampled: Vec::new(),
            muted: false,
            volume: 1.0,
        };

        Ok((
            Self { _stream: stream, stats, rate, channels, device_name },
            sink,
        ))
    }

    /// Actual output rate. May differ from what was requested.
    pub fn rate(&self) -> u32 {
        self.rate
    }

    pub fn channels(&self) -> u16 {
        self.channels
    }

    pub fn device_name(&self) -> &str {
        &self.device_name
    }

    pub fn stats(&self) -> &AudioStats {
        &self.stats
    }
}
