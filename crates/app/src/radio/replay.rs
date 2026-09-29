use super::*;

/// Scan a buffer while recording, as the radio thread does. Test support.
/// A receiver set up to sweep a capture, the way the live one sweeps the air.
pub(crate) fn replay_receiver(
    buf: &common::IqBuf,
    rec: Option<crate::record::Recorder>,
) -> anyhow::Result<crate::chain::Receiver> {
    // A 1090 MHz capture goes through the wideband path instead of the
    // channel banks, the same way the live receiver decides: 1090 carries
    // nothing the ISM banks understand, so running them there only spends CPU
    // inventing unknown bursts out of Mode S.
    // A capture goes through whatever front end the scanner table puts on
    // its frequency, the same way the live receiver decides.
    let plan = replay_plan(buf, rec.is_some());
    Ok(crate::chain::Receiver::build(
        &plan,
        crate::chain::Sinks { recorder: rec, ..Default::default() },
    )?)
}

/// The plan a capture is replayed under, as the live receiver would decide
/// it from the scanner table.
pub(crate) fn replay_plan(buf: &common::IqBuf, record: bool) -> Plan {
    let rate = buf.rate.as_f64();
    let scanners = crate::scanners::Scanners::load();
    let fronts = scanners.fronts(crate::scanners::Span::new(buf.center.as_f64(), rate));
    Plan {
        center: buf.center,
        rate,
        zoom: 1,
        // A file has already been through whatever the receiver did to it.
        dc_block: false,
        centre_spur: true,
        refresh_hz: 30.0,
        smoothing: crate::chain::DEFAULT_SMOOTHING,
        trace: dsp::spectrum::Detector::Average,
        wf_detector: dsp::spectrum::Detector::Peak,
        fft: 1024,
        channels: Vec::new(),
        fronts,
        feeds: Vec::new(),
        iqstream: None,
        iqstream_tuners: Vec::new(),
        kiss: None,
        seams: Vec::new(),
        tx: None,
        tx_capture: None,
        rds: None,
        scan: Default::default(),
        heat: Default::default(),
        edits: Default::default(),
        record,
        capture: false,
        capture_dir: crate::chain::default_capture_dir(),
        capture_format: common::SampleFormat::Cu8,
        capture_arm: Default::default(),
        log: false,
        calls: None,
        transcribe: false,
        transcribe_model: String::new(),
        transcribe_device: String::new(),
        settings: Default::default(),
    }
}

/// When a block's signal arrived, given the moment its processing finished.
///
/// A decode is stamped with the start of the block that carried it rather than
/// the moment the decoder finished with it. The burst is somewhere inside
/// those samples, and a stamp taken afterwards drifts by however long decoding
/// took, which on a loaded machine is longer than the block itself.
pub(super) fn block_start(
    finished: common::time::Instant,
    samples: usize,
    rate: f64,
) -> common::time::Instant {
    finished - common::time::Duration::from_secs_f64(samples as f64 / rate.max(1.0))
}

/// What one block decoded to, and what the recorder should keep of it.
///
/// One place, used by the live loop and by a replay, because a replay that
/// harvested differently would be evidence about a different receiver. The
/// copies of a burst other channels read are already gone: the dedupe is a
/// node in the graph, so every consumer of the bus sees the rows this
/// returns.
pub(crate) fn harvest(
    rx: &mut crate::chain::Receiver,
    at: common::time::Instant,
) -> Vec<crate::row::Reception> {
    let found = rx.rows(at);
    if let Some(r) = rx.recorder_mut() {
        for d in &found {
            r.capture(d);
        }
    }
    found
}

/// Sweep a capture as the radio thread does, block by block.
///
/// Blocks are the size the radio delivers, because deduplication depends on
/// how a burst falls across block boundaries and a whole-file call would not
/// exercise it.
pub(crate) fn replay_blocks(
    rx: &mut crate::chain::Receiver,
    buf: &common::IqBuf,
) -> Vec<crate::row::Reception> {
    let mut out = Vec::new();
    let rate = buf.rate.as_f64().max(1.0);
    for block in buf.samples.chunks(16_384) {
        if rx.process(block).is_err() {
            break;
        }
        let at = block_start(common::time::Instant::now(), block.len(), rate);
        out.extend(harvest(rx, at));
    }
    // One block of nothing after the capture, because the auto node reads a
    // block one call behind finding it, and a radio never stops delivering.
    let quiet = vec![C32::default(); 16_384];
    if rx.process(&quiet).is_ok() {
        let at = block_start(common::time::Instant::now(), quiet.len(), rate);
        out.extend(harvest(rx, at));
    }
    out
}

/// Scan a buffer while recording, as the radio thread does. Test support.
#[cfg(test)]
pub fn scan_with_recorder(
    buf: &common::IqBuf,
    rec: crate::record::Recorder,
) -> (Vec<crate::row::Reception>, Option<crate::record::Recorder>) {
    let mut rx = match replay_receiver(buf, Some(rec)) {
        Ok(rx) => rx,
        Err(_) => return (Vec::new(), None),
    };
    let out = replay_blocks(&mut rx, buf);
    (out, rx.take_recorder())
}

/// Run a capture through the same chain the live receiver uses.
///
/// The point of recording bursts is to be able to try again without waiting
/// for a device to transmit, so replay has to go through the same code the
/// receiver does, not a simplified copy of it.
pub fn replay(path: impl AsRef<std::path::Path>) -> anyhow::Result<Vec<crate::row::Reception>> {
    replay_as(path, sources::FileMeta::default())
}

/// The same, for a recording whose name does not say what it holds.
pub fn replay_as(
    path: impl AsRef<std::path::Path>,
    given: sources::FileMeta,
) -> anyhow::Result<Vec<crate::row::Reception>> {
    let src = sources::FileSource::open_as(path.as_ref(), given)?;
    let buf = src.read_all()?;
    let mut rx = replay_receiver(&buf, None)?;
    Ok(replay_blocks(&mut rx, &buf))
}
