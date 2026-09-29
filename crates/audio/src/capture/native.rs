use super::{Shared, is_real_device};
use crate::AudioError;
use cpal::traits::{DeviceTrait, HostTrait, StreamTrait};
use cpal::{Device, SampleFormat, StreamConfig};
use std::sync::Arc;

pub struct Input {
    _stream: cpal::Stream,
}

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

pub fn open(
    needle: Option<String>,
    want_rate: u32,
) -> Result<(Input, Arc<Shared>, String), AudioError> {
    let host = cpal::default_host();
    let device = match needle {
        None => host.default_input_device().ok_or(AudioError::NoDevice)?,
        Some(needle) => host
            .input_devices()
            .map_err(|e| AudioError::Cpal(e.to_string()))?
            .find(|d| d.to_string().to_lowercase().contains(&needle))
            .ok_or(AudioError::NoDevice)?,
    };
    open_on(device, want_rate)
}

fn open_on(device: Device, want_rate: u32) -> Result<(Input, Arc<Shared>, String), AudioError> {
    let device_name = device.to_string();
    let default = device.default_input_config().map_err(|e| AudioError::Cpal(e.to_string()))?;

    let supports_want = device
        .supported_input_configs()
        .map(|it| {
            it.filter(|c| c.sample_format() == SampleFormat::F32)
                .any(|c| c.min_sample_rate() <= want_rate && want_rate <= c.max_sample_rate())
        })
        .unwrap_or(false);
    if default.sample_format() != SampleFormat::F32 && !supports_want {
        return Err(AudioError::NoFormat);
    }
    let (rate, channels) = match supports_want {
        true => (want_rate, default.channels().min(2)),
        false => (default.sample_rate(), default.channels().min(2)),
    };

    let shared = Shared::new(rate as f64);
    let cb = shared.clone();
    let ch = channels as usize;
    let stream = device
        .build_input_stream(
            StreamConfig { channels, sample_rate: rate, buffer_size: cpal::BufferSize::Default },
            move |input: &[f32], _| cb.push(input, ch),
            |e| tracing::warn!("microphone stream error: {e}"),
            None,
        )
        .map_err(|e| AudioError::Cpal(e.to_string()))?;
    stream.play().map_err(|e| AudioError::Cpal(e.to_string()))?;

    tracing::info!("microphone open: {device_name} at {rate} Hz, {channels} channels");
    Ok((Input { _stream: stream }, shared, device_name))
}
