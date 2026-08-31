//! Wideband channel bank: one channelizer feeding many independent decode
//! graphs, run in parallel.
//!
//! This is the point of the whole project. A polyphase channelizer splits a
//! wide span into `N` channels for roughly the cost of one filter plus an FFT,
//! and each channel then owns its own graph. The graphs share nothing, so they
//! are embarrassingly parallel and go straight onto a rayon pool.
//!
//! # Why the parallelism is here and not inside the graph
//!
//! GNU Radio gives every block its own thread. That is reasonable for a
//! twenty-block flowgraph and catastrophic here: 512 channels of five nodes
//! would be 2560 threads on 48 cores, and the context switching would cost
//! more than the DSP. Inverting it, so each graph is strictly serial and the
//! *channels* are parallel, means the thread count is bounded by the pool
//! rather than by the flowgraph, and each worker keeps one channel's filter
//! state hot in cache for the whole block.
//!
//! # The transpose
//!
//! The channelizer emits one frame at a time: `N` samples, one per channel.
//! Graphs need the opposite layout, a contiguous run of samples per channel.
//! Converting between the two is a transpose, and it is not a footnote:
//! measured at 50 MS/s with 512 channels, a separate transpose pass cost
//! 1.15 s against the channelizer's own 0.88 s. It was the single largest
//! cost in the bank.
//!
//! The fix is a *blocked* transpose, not the absence of one. Reading frames
//! row-wise while writing a tile of channels keeps both sides cache-resident:
//! measured at 50 MS/s with 512 channels, a tiled parallel transpose costs
//! 0.05 s against 1.15 s for the naive single-threaded version that first
//! suggested transposing was expensive at all.
//!
//! Letting each channel gather its own column instead looks like it saves a
//! pass, and does not. A column walk has a `channels * 8` byte stride, so every
//! 8-byte sample drags in a full 64-byte cache line and eight ninths of the
//! memory bandwidth is wasted. Doing it once, in tiles, then handing each graph
//! a contiguous run, is far cheaper than doing it lazily per channel.

use common::{Error, Hz, Package, Result, C32};
use dsp::{Channelizer, Detector, DetectorConfig};
use pipeline::event::Event;
use pipeline::registry::Registry;
use pipeline::{Graph, Out, StreamSpec};
use rayon::prelude::*;

use crate::{build_chain, NodeSpec};

/// An event, tagged with where it came from.
#[derive(Clone, Debug)]
pub struct ChannelEvent {
    pub channel: usize,
    /// RF centre frequency of that channel.
    pub center: Hz,
    pub event: Event,
}

/// How idle channels are handled.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Gating {
    /// Run every channel that has a chain, every block.
    ///
    /// Costs more but keeps each graph's filter state continuous, which
    /// stateful nodes require.
    Always,
    /// Run a channel's graph only around bursts detected there.
    ///
    /// Much cheaper on a mostly empty band, and the right choice for ISM
    /// monitoring. The caveat is real though: a gated graph sees a
    /// discontinuous stream, so any node whose output depends on history
    /// across the gap will be wrong. Pulse detection is unaffected because a
    /// gap between bursts is exactly what it treats as a packet boundary.
    ///
    /// "Around", not "during", in two respects, and both are needed before
    /// anything decodes at all. A block in which a burst opened *and closed*
    /// still has to run, and so does the block after it: a pulse detector only
    /// emits a package once it has seen the silence that ends it, so cutting
    /// the channel off the instant the carrier drops leaves the last packet
    /// sitting in the detector until the next transmission dislodges it.
    OnDetection,
}

pub struct ChannelBank {
    ch: Channelizer,
    input_rate: f64,
    center: Hz,
    channels: usize,

    /// Staged frames, `channels` samples per frame, row-major.
    frames: Vec<C32>,
    n_frames: usize,
    /// Per-channel contiguous samples, filled by a tiled transpose.
    lanes: Vec<Vec<C32>>,

    graphs: Vec<Option<Graph>>,
    detector: Detector,
    gating: Gating,
    /// Blocks since each channel last had a burst, for the gate's hangover.
    idle_blocks: Vec<u32>,
    /// Scratch for collected results, reused between blocks.
    out: Vec<ChannelEvent>,
    /// Every burst the channels detected this block, decoded or not.
    ///
    /// The undecoded ones are the point: a burst nothing claimed is exactly
    /// what a log is for, and it is gone the moment the block is overwritten.
    packages: Vec<Package>,
    /// Which output of a channel's graph carries packages, discovered from
    /// the built chain rather than assumed, since a chain is free to be
    /// shaped however it likes as long as something in it detects bursts.
    pulse_tap: Option<Out>,
}

impl ChannelBank {
    /// `channels` must be even. `taps_per_branch` of 12 gives about 90 dB of
    /// channel-to-channel isolation, enough that a strong transmitter does not
    /// paint false detections across the band.
    pub fn new(channels: usize, taps_per_branch: usize, input_rate: f64, center: Hz) -> Self {
        let ch = Channelizer::new(channels, taps_per_branch, 90.0);
        Self {
            ch,
            input_rate,
            center,
            channels,
            frames: Vec::new(),
            n_frames: 0,
            lanes: (0..channels).map(|_| Vec::new()).collect(),
            graphs: (0..channels).map(|_| None).collect(),
            detector: Detector::new(channels, DetectorConfig::default()),
            gating: Gating::Always,
            idle_blocks: vec![u32::MAX; channels],
            out: Vec::new(),
            packages: Vec::new(),
            pulse_tap: None,
        }
    }

    pub fn channels(&self) -> usize {
        self.channels
    }

    /// Output sample rate of each channel.
    pub fn channel_rate(&self) -> f64 {
        self.ch.channel_rate(self.input_rate)
    }

    /// Spectral width each channel occupies.
    pub fn channel_bandwidth(&self) -> f64 {
        self.ch.channel_bandwidth(self.input_rate)
    }

    /// RF centre frequency of a channel.
    pub fn channel_center(&self, ch: usize) -> Hz {
        let off = self.ch.channel_offset_hz(ch, self.input_rate);
        Hz((self.center.as_f64() + off).max(0.0) as u64)
    }

    /// The channel nearest a given RF frequency.
    pub fn channel_for(&self, f: Hz) -> usize {
        self.ch.channel_for_offset(f.as_f64() - self.center.as_f64(), self.input_rate)
    }

    /// Retune the bank, keeping its graphs.
    ///
    /// Every channel now covers a different frequency, so all the state that
    /// described the old ones is wrong: the noise floors, the open bursts and
    /// any half-collected packet. Clearing them is not a precaution, it is the
    /// difference between a retune and a fabricated decode stitched from two
    /// frequencies.
    pub fn set_center(&mut self, center: Hz) -> &mut Self {
        if center != self.center {
            self.center = center;
            self.reset();
        }
        self
    }

    pub fn center(&self) -> Hz {
        self.center
    }

    pub fn set_gating(&mut self, g: Gating) -> &mut Self {
        self.gating = g;
        self
    }

    pub fn set_detector_config(&mut self, cfg: DetectorConfig) -> &mut Self {
        self.detector = Detector::new(self.channels, cfg);
        self
    }

    /// Spec seen by a channel's graph.
    pub fn channel_spec(&self, ch: usize) -> StreamSpec {
        let mut s = StreamSpec::iq(self.channel_rate(), self.channel_center(ch));
        // Only the middle `1/2` of a 2x oversampled channel's Nyquist span is
        // actually occupied; telling downstream nodes this stops a detector
        // treating filter roll-off as signal.
        s.bandwidth = self.channel_bandwidth();
        s
    }

    /// Give one channel a decode chain.
    pub fn set_chain(&mut self, ch: usize, specs: &[NodeSpec], reg: &Registry) -> Result<()> {
        if ch >= self.channels {
            return Err(Error::other(format!(
                "channel {ch} out of range, bank has {}",
                self.channels
            )));
        }
        let g = build_chain(self.channel_spec(ch), specs, reg)
            .map_err(|e| Error::other(format!("channel {ch}: {e}")))?;
        self.pulse_tap = pulse_tap(&g);
        self.graphs[ch] = Some(g);
        Ok(())
    }

    /// Give every channel the same chain.
    ///
    /// Each channel gets its *own* graph instance, so their filter states stay
    /// independent. Sharing one graph across channels would silently
    /// cross-contaminate them.
    pub fn set_all_chains(&mut self, specs: &[NodeSpec], reg: &Registry) -> Result<()> {
        for ch in 0..self.channels {
            self.set_chain(ch, specs, reg)?;
        }
        Ok(())
    }

    /// Give every channel a graph built by `make`, for chains that branch and
    /// so cannot be described as a list of nodes.
    ///
    /// `make` is called once per channel with that channel's spec, and must
    /// return a fresh graph each time for the same reason
    /// [`Self::set_all_chains`] does.
    pub fn set_all_graphs(
        &mut self,
        make: impl Fn(StreamSpec) -> Result<Graph>,
    ) -> Result<()> {
        for ch in 0..self.channels {
            let g = make(self.channel_spec(ch))
                .map_err(|e| Error::other(format!("channel {ch}: {e}")))?;
            self.pulse_tap = pulse_tap(&g);
            self.graphs[ch] = Some(g);
        }
        Ok(())
    }

    pub fn clear_chain(&mut self, ch: usize) {
        if ch < self.channels {
            self.graphs[ch] = None;
        }
    }

    pub fn active_chains(&self) -> usize {
        self.graphs.iter().filter(|g| g.is_some()).count()
    }

    pub fn graph(&self, ch: usize) -> Option<&Graph> {
        self.graphs.get(ch).and_then(|g| g.as_ref())
    }

    pub fn graph_mut(&mut self, ch: usize) -> Option<&mut Graph> {
        self.graphs.get_mut(ch).and_then(|g| g.as_mut())
    }

    pub fn reset(&mut self) {
        self.ch.reset();
        self.detector.reset();
        self.idle_blocks.fill(u32::MAX);
        for g in self.graphs.iter_mut().flatten() {
            g.reset();
        }
        for l in &mut self.lanes {
            l.clear();
        }
        self.frames.clear();
        self.n_frames = 0;
    }

    /// Channelize a block and run every channel's graph.
    pub fn process(&mut self, input: &[C32]) -> Result<&[ChannelEvent]> {
        let n = self.channels;

        // 1. Channelize across the pool, staging frames row-major.
        //
        //    The serial channelizer runs at about 50 MS/s on one core no
        //    matter how many are idle, which caps the entire receiver: at
        //    50 MS/s it alone consumed 0.98 of every second of real time,
        //    leaving nothing for the decoders. Overlap-save makes every frame
        //    independent, and the same work then takes 0.17 s.
        let count = self.ch.process_parallel(input, &mut self.frames);
        self.n_frames = count;

        // 2. Tiled transpose to channel-major. Rows are read contiguously and
        //    a tile of channels is written at a time, so both sides stay in
        //    cache. TILE is chosen so a tile's worth of write cursors fits
        //    comfortably in L1.
        const TILE: usize = 32;
        const FBLOCK: usize = 64;
        let frames = &self.frames;
        self.lanes.par_chunks_mut(TILE).enumerate().for_each(|(gi, group)| {
            let c0 = gi * TILE;
            for lane in group.iter_mut() {
                lane.clear();
                lane.reserve(count);
            }
            let mut f0 = 0;
            while f0 < count {
                let fe = (f0 + FBLOCK).min(count);
                for f in f0..fe {
                    let row = &frames[f * n..(f + 1) * n];
                    for (j, lane) in group.iter_mut().enumerate() {
                        lane.push(row[c0 + j]);
                    }
                }
                f0 = fe;
            }
        });

        // 3. Update the burst detector, in parallel over the now channel-major
        //    lanes. Doing this frame by frame instead is single-threaded and
        //    was, by a wide margin, the most expensive thing in the bank.
        self.detector.process_lanes(&self.lanes);

        // 4. Run the graphs in parallel. Each reads a contiguous lane, owns
        //    its own state, and mutates nothing shared, so there is no locking
        //    anywhere in here.
        for (c, idle) in self.idle_blocks.iter_mut().enumerate() {
            if self.detector.was_open(c) {
                *idle = 0;
            } else {
                *idle = idle.saturating_add(1);
            }
        }

        let gating = self.gating;
        let idle_blocks = &self.idle_blocks;
        let lanes = &self.lanes;
        let tap = self.pulse_tap;
        let results: Vec<(usize, Vec<Event>, Vec<Package>)> = self
            .graphs
            .par_iter_mut()
            .enumerate()
            .filter_map(|(c, slot)| {
                let g = slot.as_mut()?;
                // One block of hangover: enough for the trailing silence to
                // reach the pulse detector and close the package.
                if gating == Gating::OnDetection && idle_blocks[c] > 1 {
                    return None;
                }
                let buf = g.input_buf();
                buf.clear();
                buf.iq_mut().extend_from_slice(&lanes[c]);
                let evs = match g.run() {
                    Ok(ev) => ev.to_vec(),
                    Err(e) => vec![Event::Warning {
                        stage: format!("channel {c}"),
                        message: e.to_string(),
                    }],
                };
                // Read back what the front end detected, before the protocols
                // had their say. A burst nothing recognised leaves no event at
                // all, and it is the one worth keeping.
                let pkgs: Vec<Package> = tap
                    .and_then(|t| g.buf(t))
                    .and_then(|p| p.as_pulses())
                    .map(|p| p.to_vec())
                    .unwrap_or_default();
                if evs.is_empty() && pkgs.is_empty() {
                    return None;
                }
                Some((c, evs, pkgs))
            })
            .collect();

        // 5. Flatten, in channel order so output is deterministic regardless
        //    of how rayon happened to schedule the work.
        self.out.clear();
        self.packages.clear();
        let mut results = results;
        results.sort_by_key(|(c, _, _)| *c);
        for (c, evs, pkgs) in results {
            let center = self.channel_center(c);
            for e in evs {
                self.out.push(ChannelEvent { channel: c, center, event: e });
            }
            self.packages.extend(pkgs);
        }
        Ok(&self.out)
    }

    /// Every burst the last block detected, decoded or not.
    pub fn packages(&self) -> &[Package] {
        &self.packages
    }

    /// Channels with a burst in progress.
    pub fn active_channels(&self) -> Vec<usize> {
        self.detector.active().collect()
    }

    /// Peak power per channel in dB since the last call, for a waterfall.
    pub fn drain_peak_hold_db(&mut self, out: &mut Vec<f32>) {
        self.detector.drain_peak_hold_db(out);
    }
}

/// The first output in a chain that carries packages.
fn pulse_tap(g: &Graph) -> Option<Out> {
    g.order().find_map(|(id, _)| {
        let out = id.o();
        matches!(g.spec_of(out).map(|s| s.kind), Some(pipeline::PortKind::Pulses)).then_some(out)
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::registry;

    #[test]
    fn channel_centres_and_lookup_are_inverses() {
        let b = ChannelBank::new(64, 8, 2_400_000.0, Hz::mhz(433));
        for c in 0..64 {
            assert_eq!(b.channel_for(b.channel_center(c)), c);
        }
    }

    #[test]
    fn channel_rate_is_twice_the_spacing() {
        let b = ChannelBank::new(16, 8, 250_000.0, Hz::mhz(433));
        assert_eq!(b.channel_bandwidth(), 250_000.0 / 16.0);
        assert_eq!(b.channel_rate(), 2.0 * 250_000.0 / 16.0);
    }

    #[test]
    fn a_bad_chain_names_the_channel() {
        let mut b = ChannelBank::new(8, 8, 250_000.0, Hz::mhz(433));
        let bad = vec![NodeSpec::new("pulse_detect")];
        let err = b.set_chain(3, &bad, &registry()).unwrap_err().to_string();
        assert!(err.contains("channel 3"), "{err}");
    }

    #[test]
    fn channels_out_of_range_are_rejected() {
        let mut b = ChannelBank::new(8, 8, 250_000.0, Hz::mhz(433));
        assert!(b.set_chain(99, &[], &registry()).is_err());
    }
}
