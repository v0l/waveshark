//! Source detection and extraction: find what is transmitting in a wideband
//! stream, and hand each transmitter over as its own stream.
//!
//! The channel bank decided a signal's width before it had seen the signal.
//! A grid of fixed channels was split over a span, a power gate watched every
//! channel, and whatever landed in one was read through that channel's
//! filter, however wide the signal really was. That is why there were four
//! grids of different widths over the same band, and why one burst arrived
//! in several of them at once.
//!
//! This inverts the order. The span is watched as a spectrogram, one FFT
//! frame at a time. Each bin tracks its own noise floor, so a bin above it is
//! a bin with something in it, and a run of such bins that persists from one
//! frame to the next is a *source*: something transmitting at a centre with
//! a width, from one instant until another. Width and centre come out of the
//! measurement rather than going into it.
//!
//! Each source then gets its own extraction, mixed to baseband and decimated
//! to a rate that fits the width just measured, from a short ring of the
//! wideband stream so the lead-in the detector needed to make up its mind is
//! not lost. What comes out is a run of [`SourceBlock`]s per source, in
//! order, with no gaps: a stream, however brief. A packet four milliseconds
//! long is a stream that runs four milliseconds; a broadcast carrier is one
//! that never closes. Whatever reads them is built when the source opens and
//! dropped when it closes, and does not have to know which it was given.
//!
//! # Where the floor comes from
//!
//! Minimum statistics per bin, as in [`crate::detect`]: the minimum of the
//! smoothed power over a window longer than any transmission, corrected for
//! the bias a minimum carries. The correction here is derived from the
//! smoothing rather than fixed, because the smoothed power of noise is not
//! exponential any more and the square-root rule of thumb over-corrects it by
//! several dB, which is sensitivity thrown away.
//!
//! # What this cannot do yet
//!
//! Two transmitters overlapping in frequency at the same time are one source,
//! and will read as nothing sensible. A frequency hopper is a new source per
//! hop. A source's extraction is designed at the width it had when it opened;
//! one whose extent keeps growing afterwards, a sweep, is reopened at the
//! full width from its start, but one that widens once and holds is read
//! through the filter it opened with.

//! The three parts are here as three files: [`detect`] watches the
//! spectrogram and says what opened and closed, [`extract`] cuts each of
//! those out as a stream of its own, and [`bank`] is the one channelizer they
//! are cut from where the span is wide enough for it to pay. What they share,
//! and what a caller sees, is here.

mod bank;
mod detect;
mod extract;

pub use bank::{BANK_CHANNEL_HZ, BANK_MIN_CHANNELS};
use common::SourceId;
pub use detect::SourceDetector;
pub use extract::SourceExtractor;

/// Everything the pair is built with.
///
/// One description for both, because they are one instrument: the width the
/// detector will open is the width the extraction has to hold, and a rate
/// asked of one is a rate the other has to serve. Which part reads what is
/// said on each field; roughly, everything down to `pair_hz` is detection,
/// `lead_us` to `history_s` is extraction, and the last few are read by both.
#[derive(Clone, Copy, Debug)]
pub struct SourceConfig {
    /// FFT frame length, or zero to pick one giving bins near `bin_hz`.
    pub fft_size: usize,
    /// Bin width to aim for when `fft_size` is zero.
    ///
    /// Narrower bins hear a weak narrow signal better, since less noise is
    /// integrated beside it, and cost time resolution: a bin of `b` hertz is
    /// a frame of `1/b` seconds. Two kilohertz puts a 1.5 kbit/s sensor in a
    /// couple of bins and resolves half a millisecond, which is the pulse
    /// width the fastest ISM devices key at.
    pub bin_hz: f64,
    /// SNR a bin must reach for a new source to open there, in dB.
    pub open_db: f32,
    /// SNR below which a bin no longer counts towards a source, in dB. Held
    /// under `open_db` so a signal fading around one threshold does not open
    /// and close a source every frame.
    pub close_db: f32,
    /// Frames of exponential power smoothing before thresholding. The power
    /// of one frame of noise in one bin is exponentially distributed and
    /// cannot be thresholded on its own; four frames bring its spread to a
    /// few dB.
    pub integrate_frames: usize,
    /// How far back the floor looks, in seconds. On its own this must exceed
    /// the longest transmission expected, or the transmission is learned as
    /// floor; `floor_cap_db` is what removes that requirement.
    pub floor_memory_s: f64,
    /// How far a bin's floor may sit above the floor of the bins around it,
    /// in dB, before it is taken to be a signal rather than noise.
    ///
    /// A floor from minimum statistics alone cannot see a carrier that never
    /// stops: the minimum in its bin is the carrier, so it reads as its own
    /// noise and its SNR is zero. TETRA base station downlinks are the case
    /// that showed it, on air continuously and already transmitting before
    /// the receiver tuned to them, so they were never reported at all. The
    /// band answers what time cannot: a carrier is a few bins out of
    /// hundreds, and the bins beside it say what the noise is.
    ///
    /// The cap is the headroom left for the floor's own shape across a
    /// chunk, so it is set by tilt, not by any signal. Filter roll-off,
    /// tuner gain slope and a distant transmitter's shoulder are a few dB
    /// over a few hundred kilohertz, and a signal worth reporting is tens.
    /// Set it high enough that no chunk trips it and the detector goes back
    /// to minimum statistics alone.
    pub floor_cap_db: f32,
    /// Bins the cap's floor is measured over, or zero for an eighth of the
    /// frame, and never fewer than sixty-four.
    ///
    /// The measure is a median, so it holds while the signals in a chunk are
    /// under half of it: a 25 kHz channel in a chunk of a quarter megahertz
    /// is a fifteenth. Wider chunks tolerate wider signals and follow the
    /// floor's shape less closely. A signal that fills its chunk is
    /// indistinguishable from a noise floor by this test and is left to the
    /// minimum statistics, which is the right answer for a band nobody can
    /// see past.
    pub floor_chunk_bins: usize,
    /// Consecutive frames a run of bins must persist before it is a source.
    pub min_frames: usize,
    /// Silence before a source closes, in microseconds.
    ///
    /// Also what holds a source open across the gaps inside it. Fine Offset
    /// stations repeat their frame three times with 8 ms between repeats,
    /// and those three belong to one source and one package. A DMR radio
    /// on one slot of two keys 30 ms bursts with 30 ms between them, and a
    /// hang shorter than that gap closed the source at every burst: each
    /// slot opened as a transmission of its own, with a fresh decoder that
    /// had a burst to read and no superframe to read it in.
    pub hang_us: u32,
    /// Bins of gap to bridge inside one source. A keyed signal has nulls in
    /// its spectrum, and a null is not the edge of the signal.
    pub guard_bins: usize,
    /// How far apart two runs can be and still be one transmitter, when
    /// they appear in the same frame, in hertz.
    ///
    /// Frequency-shift keying is two tones with nothing between them, and
    /// the gap is often far wider than either tone: a LaCrosse sensor keys
    /// tones 120 kHz apart that are 10 kHz wide. They are one source, and
    /// the extraction has to hold both or the discriminator has nothing to
    /// discriminate. Two transmitters keying up in the same half
    /// millisecond within this distance are merged too, which costs a wider
    /// extraction and nothing else.
    pub pair_hz: f64,
    /// Samples handed over from before the source opened, in microseconds.
    /// A demodulator's gate needs noise to measure the signal against, and
    /// the detector took a few frames to decide, so the burst's own start is
    /// already behind by the time it opens.
    pub lead_us: u32,
    /// Samples handed over after the source closed, in microseconds. What a
    /// pulse front end needs to see the silence that ends a package.
    pub tail_us: u32,
    /// Output rate as a multiple of the extracted width.
    pub oversample: f64,
    /// Lowest rate a source is extracted at. A pulse read at a few kS/s has
    /// no timing left to measure.
    pub min_rate_hz: f64,
    /// Width kept around a source, as a multiple of the width measured. The
    /// bins above threshold are the loud middle of a signal, not its edges.
    pub width_margin: f64,
    /// How far under a source's peak a bin can be and still count towards
    /// its extent, in dB. Bounds the extent of a strong signal, whose keying
    /// sidebands and switching transients sit over the floor far beyond
    /// anything a receiver would call its width.
    pub extent_db: f32,
    /// Stopband of the extraction filter, in dB.
    pub atten_db: f64,
    /// Movement in a candidate's peak power, in dB, before it is taken to be
    /// a transmission rather than a fixture of the receiver.
    ///
    /// Now that a carrier which never stops is no longer absorbed into the
    /// floor, the receiver's own spurs are not either: the tuner's leakage at
    /// the centre, a switching supply's harmonic, a bare oscillator. What
    /// separates those from a transmission is not width or strength but that
    /// nothing is being sent. A modulated carrier's strongest bin moves by
    /// several dB from frame to frame however constant its envelope, and an
    /// unmodulated one does not move at all.
    ///
    /// A candidate that has not moved this far is held as a candidate rather
    /// than discarded, so it opens on the frame it first does. The cost is
    /// that a genuinely unmodulated carrier, a beacon sending nothing, is
    /// never reported; it is indistinguishable from a spur by any measure
    /// this detector has.
    ///
    /// Asked only of a candidate that appeared when the floor cap first
    /// did, within `fixture_s` of it. Minimum statistics hide a fixture
    /// until the cap unhides it, so that is the frame every fixture is born
    /// in; a candidate born later came from nothing, and that is movement
    /// enough. Asked of everything, it cost a short on-off keyed burst:
    /// smoothed over the integration, its peak did not move three decibels
    /// in the whole of its life.
    pub steady_db: f32,
    /// How long after the floor cap is first measured a new candidate is
    /// still taken to be possibly a fixture, in seconds, and so has to move
    /// `steady_db` before it opens. A few frames of integration is all a
    /// fixture needs to appear once it is unhidden.
    pub fixture_s: f64,
    /// Fewest bins a run must occupy to open a source.
    ///
    /// Two, because nothing keyed is one bin wide, and what is one bin wide
    /// is a spur: the tuner's own leakage, a switching supply's harmonic, a
    /// bare oscillator. Each of those would otherwise be a carrier reported
    /// every half second for as long as the receiver ran.
    pub min_bins: usize,
    /// Growth of a source's extent past the width it opened at that has it
    /// reopened at the new width, as a ratio.
    ///
    /// A slow chirp sweeps a few kilohertz in the frames it takes to open
    /// and hundreds over its symbol, so the width it opens at is a sliver of
    /// what it is. An extraction designed at open would keep the sliver and
    /// lose the sweep. When the measured extent outgrows the extraction, the
    /// stream is closed as superseded and a new one opened at the full width
    /// from the transmitter's start, which is what the history below is for.
    pub regrow: f64,
    /// Wideband samples kept behind the newest, in seconds, so a reopened
    /// source can start again from where it began.
    pub history_s: f64,
    /// Widest a source may be, in hertz. Nothing wider opens, and an open
    /// source stops growing rather than pass it.
    ///
    /// A receiver driven into saturation lights its whole span: the floor
    /// comes up, intermodulation lines stand every few hundred kilohertz,
    /// and the detector reads one thing as wide as the input. Cut out at
    /// the full rate and handed to every front end that will take it, that
    /// cost more than the rest of the band together and decoded nothing,
    /// since nothing this receiver reads is wider than a 500 kHz LoRa
    /// channel. Set it above the widest signal a front end reads.
    pub max_width_hz: f64,
    /// Most sources open at once.
    ///
    /// Every open source costs an extraction and a front end on every block
    /// for as long as it lasts, so a band that never goes quiet costs
    /// without bound: 2.4 GHz opened 26 at once on Wi-Fi and the burst
    /// router alone ran at ten times real time, which is a receiver that
    /// stops answering rather than one that reads more. At the cap the
    /// quietest open source is closed to make room for a louder candidate,
    /// so what is dropped is what a listener would have dropped.
    pub max_open: usize,
    /// Channel width the shared extraction bank aims for, in hertz, and the
    /// fewest channels worth running it with. See [`Bank`]. Zero channels
    /// disables it.
    pub bank_channel_hz: f64,
    pub bank_min_channels: usize,
}

impl Default for SourceConfig {
    fn default() -> Self {
        Self {
            fft_size: 0,
            bin_hz: 2_000.0,
            open_db: 10.0,
            close_db: 5.0,
            integrate_frames: 4,
            floor_memory_s: 2.0,
            floor_cap_db: 8.0,
            floor_chunk_bins: 0,
            min_frames: 2,
            hang_us: 40_000,
            guard_bins: 2,
            pair_hz: 150_000.0,
            lead_us: 5_000,
            tail_us: 30_000,
            oversample: 2.5,
            min_rate_hz: 25_000.0,
            width_margin: 1.5,
            extent_db: 20.0,
            atten_db: 60.0,
            steady_db: 3.0,
            fixture_s: 0.05,
            min_bins: 2,
            regrow: 1.5,
            history_s: 0.3,
            max_width_hz: 600_000.0,
            max_open: 12,
            bank_channel_hz: BANK_CHANNEL_HZ,
            bank_min_channels: BANK_MIN_CHANNELS,
        }
    }
}

impl SourceConfig {
    /// The frame length this configuration uses at a given rate.
    pub fn fft_size_at(&self, rate: f64) -> usize {
        if self.fft_size >= 16 {
            return self.fft_size.next_power_of_two();
        }
        let n = (rate / self.bin_hz.max(1.0)).round().max(16.0) as usize;
        n.next_power_of_two().clamp(16, 1 << 15)
    }
}

/// One transmitter, as the detector sees it.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Source {
    pub id: SourceId,
    /// Extent, as offsets from the stream centre in hertz. Grows over the
    /// source's life and never shrinks: a signal that was wide for a moment
    /// was that wide.
    pub lo_hz: f64,
    pub hi_hz: f64,
    /// Power-weighted centre of the first frames, as an offset in hertz.
    pub center_hz: f64,
    /// Wideband sample index of the frame the source was first seen in.
    pub start_sample: u64,
    /// Wideband sample index the source was last seen up to, once closed.
    pub end_sample: Option<u64>,
    pub peak_snr_db: f32,
    /// Frames the source was seen in.
    pub frames: u64,
}

impl Source {
    /// Width of the extent measured so far, in hertz.
    pub fn bandwidth_hz(&self) -> f64 {
        self.hi_hz - self.lo_hz
    }
}

/// A channel a front end is already reading, closed to the detector.
///
/// A channel that has produced a decode is that front end's, and whatever the
/// detector measures inside it is the same transmitter it is already reading:
/// opening a source for that spends a stream and an extraction to log the
/// same burst twice. What `max_width_hz` allows for is the other case, a
/// second transmitter sharing the frequency at a width the front end there
/// cannot read, which is what two LoRa networks at different bandwidths are.
/// A band a front end owns outright, rather than a channel it reads, sets it
/// infinite.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Owned {
    /// Offsets from the stream centre, in hertz.
    pub lo_hz: f64,
    pub hi_hz: f64,
    pub max_width_hz: f64,
}

impl Owned {
    fn holds(&self, center_hz: f64) -> bool {
        (self.lo_hz..=self.hi_hz).contains(&center_hz)
    }
}

/// A source opening or closing. Everything in between is a source that is
/// simply still there, which [`SourceDetector::live`] lists.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum SourceEvent {
    Opened(Source),
    Closed(Source),
    /// Closed because it outgrew its width; an `Opened` for the same
    /// transmitter, under a new id and at the new width, follows in the same
    /// batch.
    Superseded(Source),
}

impl SourceEvent {
    pub fn source(&self) -> &Source {
        match self {
            SourceEvent::Opened(s) | SourceEvent::Closed(s) | SourceEvent::Superseded(s) => s,
        }
    }
}

fn frame_start(frame: u64, hop: u64) -> u64 {
    frame * hop
}

/// Which bin an offset from the stream centre falls in, unrounded, in a frame
/// of `n` bins `bin_hz` wide. Bin zero is the bottom of the span, so the
/// centre is at `n / 2`.
fn bin_of_hz(hz: f64, n: usize, bin_hz: f64) -> f64 {
    hz / bin_hz + (n / 2) as f64
}

/// The other way about: the offset from the stream centre that bin covers,
/// measured at its middle.
fn hz_of_bin(bin: f64, n: usize, bin_hz: f64) -> f64 {
    (bin + 0.5 - (n / 2) as f64) * bin_hz
}

#[cfg(test)]
mod tests {
    use super::detect::floor_bias;
    use super::*;
    use common::{SourceBlock, SourceState, C32};

    fn noise(n: usize, amp: f32, seed: u64) -> Vec<C32> {
        // xorshift, Box-Muller: deterministic Gaussian noise.
        let mut s = seed.max(1);
        let mut next = move || {
            s ^= s << 13;
            s ^= s >> 7;
            s ^= s << 17;
            (s >> 11) as f64 / (1u64 << 53) as f64
        };
        (0..n)
            .map(|_| {
                let u1 = next().max(1e-12);
                let u2 = next();
                let r = (-2.0 * u1.ln()).sqrt();
                let th = std::f64::consts::TAU * u2;
                C32::new((r * th.cos()) as f32 * amp, (r * th.sin()) as f32 * amp)
            })
            .collect()
    }

    fn tone(n: usize, hz: f64, rate: f64, amp: f32) -> Vec<C32> {
        (0..n)
            .map(|i| {
                let ph = std::f64::consts::TAU * hz * i as f64 / rate;
                C32::new(amp * ph.cos() as f32, amp * ph.sin() as f32)
            })
            .collect()
    }

    /// A phase-keyed carrier at `hz`, `baud` symbols a second: a TETRA
    /// downlink's shape, which is what the bare tone that `tone` gives is
    /// not. The envelope is constant and the spectrum moves, and both
    /// matter. A detector tells a transmission from the receiver's own
    /// leakage by the spectrum moving, and a carrier whose envelope fades
    /// would close and reopen on its own fades rather than on the air.
    fn modulated(n: usize, hz: f64, baud: f64, rate: f64, amp: f32, seed: u64) -> Vec<C32> {
        let hold = (rate / baud).round().max(1.0) as usize;
        let mut state = seed | 1;
        let mut next = move || {
            state ^= state << 13;
            state ^= state >> 7;
            state ^= state << 17;
            (state >> 11) as f64 / (1u64 << 53) as f64
        };
        let mut out = Vec::with_capacity(n);
        let mut phase = 0.0f64;
        let mut step = 0.0f64;
        for i in 0..n {
            if i % hold == 0 {
                // Quarter turns, as pi/4-DQPSK keys, and turned into over
                // the symbol rather than jumped. An unshaped jump every few
                // dozen samples is a spectrum tens of times wider than the
                // keying, and the detector rightly reopens it at that width.
                step = std::f64::consts::FRAC_PI_4 * (1.0 + 2.0 * (next() * 4.0).floor())
                    / hold as f64;
            }
            phase += step;
            let ph = std::f64::consts::TAU * hz * i as f64 / rate + phase;
            out.push(C32::new(amp * ph.cos() as f32, amp * ph.sin() as f32));
        }
        out
    }

    const RATE: f64 = 1_000_000.0;

    fn cfg() -> SourceConfig {
        SourceConfig { floor_memory_s: 0.5, ..Default::default() }
    }

    #[test]
    fn the_floor_lands_on_the_noise() {
        let mut d = SourceDetector::new(RATE, RATE, cfg());
        let n = noise(1_000_000, 0.1, 7);
        d.process(&n);
        let snr = d.snr_db();
        let mean = snr.iter().sum::<f32>() / snr.len() as f32;
        assert!(mean.abs() < 1.5, "mean bin SNR on pure noise is {mean} dB, floor is off");
        assert!(d.live().count() == 0, "noise opened a source");
    }

    #[test]
    fn nothing_opens_on_noise() {
        let mut d = SourceDetector::new(RATE, RATE, cfg());
        let mut opened = 0;
        for chunk in noise(3_000_000, 0.1, 3).chunks(16384) {
            opened +=
                d.process(chunk).iter().filter(|e| matches!(e, SourceEvent::Opened(_))).count();
        }
        assert_eq!(opened, 0, "noise alone opened {opened} sources");
    }

    #[test]
    fn a_burst_is_one_source_with_its_frequency_and_extent() {
        let mut d = SourceDetector::new(RATE, RATE, cfg());
        let mut x = noise(1_000_000, 0.05, 11);
        // 50 ms of tone at +123 kHz, starting at 400 ms.
        let start = 400_000;
        let t = tone(50_000, 123_000.0, RATE, 1.0);
        for (i, s) in t.iter().enumerate() {
            x[start + i] += *s;
        }
        let mut events = Vec::new();
        for chunk in x.chunks(8192) {
            events.extend_from_slice(d.process(chunk));
        }
        let opened: Vec<_> = events
            .iter()
            .filter_map(|e| match e {
                SourceEvent::Opened(s) => Some(*s),
                _ => None,
            })
            .collect();
        assert_eq!(opened.len(), 1, "{events:?}");
        let s = opened[0];
        assert!((s.center_hz - 123_000.0).abs() < 2.0 * d.bin_hz(), "centre {}", s.center_hz);
        assert!(s.lo_hz < 123_000.0 && s.hi_hz > 123_000.0, "{s:?}");
        assert!(
            (s.start_sample as i64 - start as i64).abs() < 2 * d.fft_size() as i64,
            "start {}",
            s.start_sample
        );
        let closed: Vec<_> = events
            .iter()
            .filter_map(|e| match e {
                SourceEvent::Closed(s) => Some(*s),
                _ => None,
            })
            .collect();
        assert_eq!(closed.len(), 1, "{events:?}");
        let end = closed[0].end_sample.unwrap();
        assert!(
            (end as i64 - (start + 50_000) as i64).abs() < 2 * d.fft_size() as i64,
            "end {end}"
        );
    }

    #[test]
    fn two_transmitters_apart_in_frequency_are_two_sources() {
        let mut d = SourceDetector::new(RATE, RATE, cfg());
        let mut x = noise(1_000_000, 0.05, 5);
        for (i, (a, b)) in tone(100_000, -200_000.0, RATE, 0.5)
            .iter()
            .zip(tone(100_000, 310_000.0, RATE, 0.5))
            .enumerate()
        {
            x[500_000 + i] += a + b;
        }
        let mut opened = Vec::new();
        for chunk in x.chunks(8192) {
            opened.extend(d.process(chunk).iter().filter_map(|e| match e {
                SourceEvent::Opened(s) => Some(s.center_hz),
                _ => None,
            }));
        }
        opened.sort_by(|a, b| a.partial_cmp(b).unwrap());
        assert_eq!(opened.len(), 2, "{opened:?}");
        assert!((opened[0] + 200_000.0).abs() < 2.0 * d.bin_hz(), "{opened:?}");
        assert!((opened[1] - 310_000.0).abs() < 2.0 * d.bin_hz(), "{opened:?}");
    }

    #[test]
    fn a_narrow_source_is_given_the_whole_stream_it_was_cut_at() {
        // A clean narrowband channel measures a couple of bins across, its
        // rate comes from the floor rather than from that measurement, and
        // the extraction must not then filter the stream down to the two
        // bins that were measured. The source is handed to the extractor
        // rather than detected, so the width under test is exactly the one
        // written here: a signal 4 kHz off the centre, which is where M17's
        // outer symbols and a pager's deviation both sit, has to survive.
        let c = cfg();
        let at = 120_000.0;
        let mut x = noise(400_000, 0.05, 17);
        for (i, s) in tone(300_000, at, RATE, 0.4).iter().enumerate() {
            x[50_000 + i] += *s;
        }
        for (i, s) in tone(300_000, at + 4_000.0, RATE, 0.4).iter().enumerate() {
            x[50_000 + i] += *s;
        }
        let src = Source {
            id: SourceId(1),
            lo_hz: at - c.bin_hz,
            hi_hz: at + c.bin_hz,
            center_hz: at,
            start_sample: 50_000,
            end_sample: None,
            peak_snr_db: 30.0,
            frames: 4,
        };
        assert!(src.bandwidth_hz() * c.width_margin * c.oversample < c.min_rate_hz);

        // Opened once the ring holds the samples the source began in, as
        // the detector's own latency arranges on a live stream.
        let mut e = SourceExtractor::new(RATE, 100e6, 40_000, c);
        let mut blocks = Vec::new();
        let ev = [SourceEvent::Opened(src)];
        for (k, chunk) in x.chunks(8192).enumerate() {
            e.process(chunk, if k == 8 { &ev } else { &[] }, &mut blocks);
        }
        let rate = blocks.first().expect("nothing extracted").rate;
        // The floor, not the width, decided the rate: the integer split
        // between the two decimation stages can leave it a little above.
        assert!(rate >= c.min_rate_hz, "rate {rate} under the floor");
        let all: Vec<C32> = blocks.iter().flat_map(|b| b.samples.iter().copied()).collect();
        let body = &all[all.len() / 3..all.len() * 2 / 3];
        // Each tone against its own frequency in the extracted stream. Both
        // were sent at the same level, so filtering to the measured width
        // shows up as one of them arriving quieter than the other.
        let level = |hz: f64| {
            let mut acc = C32::new(0.0, 0.0);
            for (i, s) in body.iter().enumerate() {
                let ph = -std::f64::consts::TAU * hz * i as f64 / rate;
                acc += s * C32::new(ph.cos() as f32, ph.sin() as f32);
            }
            acc.norm() / body.len() as f32
        };
        let ratio = 20.0 * (level(4_000.0) / level(0.0)).log10();
        assert!(ratio > -6.0, "the signal 4 kHz out came through {ratio:.0} dB down");
    }

    #[test]
    fn the_extracted_stream_holds_the_signal_at_baseband() {
        let c = cfg();
        let mut d = SourceDetector::new(RATE, RATE, c);
        let mut e = SourceExtractor::new(RATE, 100e6, d.latency_samples(), c);
        let mut x = noise(1_000_000, 0.05, 9);
        let start = 300_000;
        // A tone keyed on and off at 1 kHz: an amplitude-keyed signal a few
        // kHz wide, which is what the extraction has to keep intact. About
        // 40 dB per bin, which is a strong sensor and not a laboratory one.
        for (i, s) in tone(200_000, -87_500.0, RATE, 0.3).iter().enumerate() {
            let on = (i / 500) % 2 == 0;
            if on {
                x[start + i] += *s;
            }
        }
        let mut blocks = Vec::new();
        for chunk in x.chunks(8192) {
            let ev = d.process(chunk).to_vec();
            e.process(chunk, &ev, &mut blocks);
        }
        assert!(!blocks.is_empty(), "nothing extracted");
        let ids: std::collections::BTreeSet<_> = blocks.iter().map(|b| b.id).collect();
        assert_eq!(ids.len(), 1, "one source, got {ids:?}");
        let states: Vec<_> =
            blocks.iter().map(|b| (b.state, b.start_sample, b.samples.len())).collect();
        assert_eq!(blocks.first().unwrap().state, SourceState::Opened, "{states:?}");
        assert_eq!(blocks.last().unwrap().state, SourceState::Closed, "{states:?}");
        // Contiguous: every block starts where the previous one stopped.
        let rate = blocks[0].rate;
        assert!(rate >= c.min_rate_hz, "rate {rate}");
        let factor = (RATE / rate).round() as u64;
        let mut expect = blocks[0].start_sample;
        let mut all = Vec::new();
        for b in &blocks {
            // Within a decimation step: the decimator keeps its own phase,
            // so a block's count is a floor or a ceiling of its share.
            let off = b.start_sample as i64 - expect as i64;
            assert!(off.unsigned_abs() < factor, "gap before block at {}", b.start_sample);
            expect = b.start_sample + b.samples.len() as u64 * factor;
            all.extend_from_slice(&b.samples);
        }
        // The lead-in is noise and the keyed tone sits at DC after that:
        // its residual frequency is near zero and its envelope alternates.
        let lead = (c.lead_us as f64 * 1e-6 * rate) as usize;
        let body_len = (100_000.0 * rate / RATE) as usize;
        let body = &all[lead + 1000..lead + body_len];
        let mut acc = C32::new(0.0, 0.0);
        for w in body.windows(2) {
            if w[0].norm() > 0.15 && w[1].norm() > 0.15 {
                acc += w[1] * w[0].conj();
            }
        }
        let residual_hz = acc.arg() as f64 / std::f64::consts::TAU * rate;
        let b0 = &blocks[0];
        // The centre is a centroid over bins, so it is good to a fraction of
        // a bin and no better, and a fraction of a bin is what the width
        // margin exists to cover.
        assert!(
            residual_hz.abs() < d.bin_hz(),
            "signal is {residual_hz} Hz off baseband; centre {} width {} rate {}",
            b0.center_hz as f64 - 100e6,
            b0.bandwidth_hz,
            b0.rate
        );
        let on = body.iter().filter(|s| s.norm() > 0.15).count();
        let ratio = on as f64 / body.len() as f64;
        assert!((0.35..0.65).contains(&ratio), "keying lost, on ratio {ratio}");
        assert!(RATE / rate >= 2.0, "a 4 kHz signal came out at {rate}, no decimation");
        assert_eq!(e.active(), 0, "the source was dropped once closed");
    }

    /// Run the keyed-tone stream through detector and extractor with the
    /// given config, returning the extracted stream and the first block.
    fn keyed_tone_through(c: SourceConfig, hz: f64) -> (Vec<C32>, SourceBlock, usize) {
        let mut d = SourceDetector::new(RATE, RATE, c);
        let mut e = SourceExtractor::new(RATE, 100e6, d.latency_samples(), c);
        let mut x = noise(1_000_000, 0.05, 9);
        let start = 300_000;
        for (i, s) in tone(200_000, hz, RATE, 0.3).iter().enumerate() {
            if (i / 500) % 2 == 0 {
                x[start + i] += *s;
            }
        }
        let mut blocks = Vec::new();
        let mut banked = 0;
        for chunk in x.chunks(8192) {
            let ev = d.process(chunk).to_vec();
            e.process(chunk, &ev, &mut blocks);
            banked = banked.max(e.banked());
        }
        assert!(!blocks.is_empty(), "nothing extracted");
        let first = blocks[0].clone();
        let mut all = Vec::new();
        let mut expect = first.start_sample;
        let factor = (RATE / first.rate).round() as u64;
        for b in &blocks {
            let off = b.start_sample as i64 - expect as i64;
            assert!(off.unsigned_abs() < factor, "gap before block at {}", b.start_sample);
            expect = b.start_sample + b.samples.len() as u64 * factor;
            all.extend_from_slice(&b.samples);
        }
        (all, first, banked)
    }

    /// Residual frequency and on-ratio of the keyed tone in an extracted
    /// stream.
    fn keyed_tone_quality(all: &[C32], rate: f64, c: &SourceConfig) -> (f64, f64, f64) {
        let lead = (c.lead_us as f64 * 1e-6 * rate) as usize;
        let body_len = (100_000.0 * rate / RATE) as usize;
        let body = &all[lead + 1000..lead + body_len];
        let mut acc = C32::new(0.0, 0.0);
        let mut level = 0.0f64;
        let mut on = 0usize;
        for w in body.windows(2) {
            if w[0].norm() > 0.15 && w[1].norm() > 0.15 {
                acc += w[1] * w[0].conj();
                level += w[0].norm() as f64;
                on += 1;
            }
        }
        let residual_hz = acc.arg() as f64 / std::f64::consts::TAU * rate;
        (residual_hz, on as f64 / body.len() as f64, level / on.max(1) as f64)
    }

    /// A source read from the shared bank is the same stream as one read from
    /// the ring: same residual, same keying, same level. Tried at an offset
    /// well inside a channel and at one on the edge between two, which is
    /// read from the pair.
    #[test]
    fn the_bank_gives_the_stream_the_ring_gives() {
        let mut direct = cfg();
        direct.bank_min_channels = 0;
        let mut banked = cfg();
        // Channels 62.5 kHz wide at this rate: the keyed tone measures about
        // 14 kHz and is cut 20 kHz wide, which sits inside one channel near
        // its centre and across two anywhere else.
        banked.bank_channel_hz = 62_500.0;
        banked.bank_min_channels = 4;
        for hz in [-87_500.0, -123_000.0] {
            let (a, fa, ba) = keyed_tone_through(direct, hz);
            let (b, fb, bb) = keyed_tone_through(banked, hz);
            assert_eq!(ba, 0, "the direct config ran the bank");
            assert!(bb > 0, "the banked config did not use the bank at {hz} Hz");
            assert_eq!(fa.rate, fb.rate, "rates differ at {hz} Hz");
            let (ra, oa, la) = keyed_tone_quality(&a, fa.rate, &direct);
            let (rb, ob, lb) = keyed_tone_quality(&b, fb.rate, &banked);
            assert!((ra - rb).abs() < 2.0, "residual {ra} vs {rb} Hz at {hz} Hz");
            assert!((oa - ob).abs() < 0.03, "keying {oa} vs {ob} at {hz} Hz");
            let db = 20.0 * (lb / la).log10();
            assert!(db.abs() < 0.3, "level differs by {db:.2} dB at {hz} Hz");
        }
    }

    #[test]
    fn a_continuous_carrier_stays_open() {
        let c = SourceConfig { floor_memory_s: 5.0, ..cfg() };
        let mut d = SourceDetector::new(RATE, RATE, c);
        let mut x = noise(2_000_000, 0.05, 13);
        for (i, s) in tone(1_800_000, 50_000.0, RATE, 0.3).iter().enumerate() {
            x[200_000 + i] += *s;
        }
        let mut closed = 0;
        for chunk in x.chunks(8192) {
            closed +=
                d.process(chunk).iter().filter(|e| matches!(e, SourceEvent::Closed(_))).count();
        }
        assert_eq!(closed, 0);
        assert_eq!(d.live().count(), 1);
    }

    #[test]
    fn a_carrier_already_on_when_the_stream_starts_is_found() {
        // A TETRA base station downlink is on before the receiver tunes to
        // it and stays on. Nothing in the stream is that bin without it, so
        // minimum statistics have nothing to measure and the bins beside it
        // are the only thing that says what the noise is.
        let mut d = SourceDetector::new(RATE, RATE, cfg());
        let mut x = noise(4_000_000, 0.05, 31);
        for (i, s) in modulated(4_000_000, 50_000.0, 18_000.0, RATE, 0.3, 32).iter().enumerate() {
            x[i] += *s;
        }
        let mut opened = 0;
        for chunk in x.chunks(8192) {
            opened +=
                d.process(chunk).iter().filter(|e| matches!(e, SourceEvent::Opened(_))).count();
        }
        assert!(opened > 0, "a permanent carrier never opened a source");
    }

    #[test]
    fn a_carrier_outlasting_the_floor_memory_stays_open() {
        // Same detector, carrier starting after the floor has been learned
        // and running far longer than the memory.
        let mut d = SourceDetector::new(RATE, RATE, cfg());
        let mut x = noise(5_000_000, 0.05, 33);
        for (i, s) in modulated(4_000_000, 50_000.0, 18_000.0, RATE, 0.3, 34).iter().enumerate() {
            x[1_000_000 + i] += *s;
        }
        let mut opened = 0;
        let mut closed = 0;
        for chunk in x.chunks(8192) {
            for e in d.process(chunk) {
                match e {
                    SourceEvent::Opened(_) => opened += 1,
                    SourceEvent::Closed(_) => closed += 1,
                    _ => {}
                }
            }
        }
        assert_eq!(opened, 1, "opened {opened}");
        assert_eq!(closed, 0, "the carrier was learned as floor and closed under it");
    }

    #[test]
    fn leading_silence_does_not_become_the_floor() {
        // rtl_433's captures open with a run of zeros while the tuner
        // settles. A zero in the minimum is a floor of nothing.
        let mut d = SourceDetector::new(RATE, RATE, cfg());
        let mut x = vec![C32::new(0.0, 0.0); 50_000];
        x.extend(noise(950_000, 0.05, 21));
        for (i, s) in tone(50_000, 80_000.0, RATE, 0.3).iter().enumerate() {
            x[500_000 + i] += *s;
        }
        let mut opened = Vec::new();
        for chunk in x.chunks(8192) {
            opened.extend(d.process(chunk).iter().filter_map(|e| match e {
                SourceEvent::Opened(s) => Some((s.center_hz, s.peak_snr_db)),
                _ => None,
            }));
        }
        assert_eq!(opened.len(), 1, "{opened:?}");
        assert!((opened[0].0 - 80_000.0).abs() < 2.0 * d.bin_hz(), "{opened:?}");
        assert!(opened[0].1 < 60.0, "an SNR of {} dB means the floor is nothing", opened[0].1);
    }

    #[test]
    fn a_settling_tuner_s_constant_is_silence_too() {
        // Byte value zero in a cu8 is minus one on both rails: a full-scale
        // constant, which is a carrier at DC and an empty floor everywhere
        // else.
        let mut d = SourceDetector::new(RATE, RATE, cfg());
        let mut x = vec![C32::new(-1.0, -1.0); 250_000];
        x.extend(noise(750_000, 0.05, 23));
        for (i, s) in tone(50_000, 80_000.0, RATE, 0.3).iter().enumerate() {
            x[600_000 + i] += *s;
        }
        let mut opened = Vec::new();
        for chunk in x.chunks(8192) {
            opened.extend(d.process(chunk).iter().filter_map(|e| match e {
                SourceEvent::Opened(s) => Some((s.center_hz, s.peak_snr_db)),
                _ => None,
            }));
        }
        assert_eq!(opened.len(), 1, "{opened:?}");
        assert!((opened[0].0 - 80_000.0).abs() < 2.0 * d.bin_hz(), "{opened:?}");
        assert!(opened[0].1 < 60.0, "an SNR of {} dB means the floor is nothing", opened[0].1);
    }

    #[test]
    fn the_two_tones_of_an_fsk_burst_are_one_source() {
        let mut d = SourceDetector::new(RATE, RATE, cfg());
        let mut x = noise(1_000_000, 0.05, 31);
        // 17 kbit/s two-tone keying, tones 120 kHz apart, as a LaCrosse
        // sensor does it.
        let (f0, f1) = (-72_000.0, 48_000.0);
        let mut ph = 0.0f64;
        for i in 0..20_000usize {
            let bit = (i / 58) % 3 == 0;
            let f = if bit { f1 } else { f0 };
            ph += std::f64::consts::TAU * f / RATE;
            x[500_000 + i] += C32::new(0.3 * ph.cos() as f32, 0.3 * ph.sin() as f32);
        }
        let mut opened = Vec::new();
        for chunk in x.chunks(8192) {
            opened.extend(d.process(chunk).iter().filter_map(|e| match e {
                SourceEvent::Opened(s) => Some(*s),
                _ => None,
            }));
        }
        assert_eq!(opened.len(), 1, "{opened:?}");
        let s = opened[0];
        assert!(s.lo_hz < f0 && s.hi_hz > f1, "extent {}..{} misses a tone", s.lo_hz, s.hi_hz);
        let mid = (f0 + f1) / 2.0;
        assert!(
            (s.center_hz - mid).abs() < 4.0 * d.bin_hz(),
            "centre {} for tones at {f0} and {f1}",
            s.center_hz
        );
    }

    #[test]
    fn a_blanketed_band_does_not_sprout_narrow_sources() {
        // Sixteen megasamples of 2.4 GHz with Wi-Fi on it: the whole span
        // above the threshold, and a spectrum full of spikes 15 to 20 dB
        // over the floor. Every one of those spikes used to be born as a
        // candidate and most opened, so the band filled with narrow channels
        // nothing was transmitting on. Only a run well clear of the blanket
        // is a transmitter now.
        const WIDE_RATE: f64 = 16_000_000.0;
        let mut d = SourceDetector::new(WIDE_RATE, WIDE_RATE, cfg());
        let mut x = noise(4_000_000, 0.02, 71);
        // A 16 MHz blanket with a ragged top, as an OFDM burst's spectrum
        // has, plus one narrow transmitter well above it.
        let mut seed = 99u64;
        let mut rnd = move || {
            seed ^= seed << 13;
            seed ^= seed >> 7;
            seed ^= seed << 17;
            (seed >> 11) as f32 / (1u64 << 53) as f32
        };
        for i in 500_000..3_500_000usize {
            let n = C32::new(rnd() - 0.5, rnd() - 0.5);
            x[i] += n * 0.35;
        }
        let mut ph = 0.0f64;
        for i in 0..3_000_000usize {
            ph += std::f64::consts::TAU * 3_000_000.0 / WIDE_RATE;
            x[500_000 + i] += C32::new(0.9 * ph.cos() as f32, 0.9 * ph.sin() as f32);
        }
        let opened: Vec<Source> = x
            .chunks(65_536)
            .flat_map(|c| {
                d.process(c)
                    .iter()
                    .filter_map(|e| match e {
                        SourceEvent::Opened(s) => Some(*s),
                        _ => None,
                    })
                    .collect::<Vec<_>>()
            })
            .collect();
        assert!(opened.len() <= 3, "{} sources under a blanket: {opened:?}", opened.len());
        assert!(
            opened.iter().any(|s| (s.center_hz - 3_000_000.0).abs() < 200_000.0),
            "the one transmitter above it was missed: {opened:?}"
        );
    }

    #[test]
    fn only_so_many_sources_open_at_once() {
        // Four transmitters at once, far enough apart not to be read as one,
        // and room for two. A band that never goes quiet costs an extraction
        // and a front end per source on every block, which is what made
        // 2.4 GHz unusable: the cap is what bounds that.
        let mut c = cfg();
        c.max_open = 2;
        let mut d = SourceDetector::new(RATE, RATE, c);
        let mut x = noise(1_000_000, 0.02, 51);
        for (k, hz) in [-400_000.0f64, -150_000.0, 100_000.0, 350_000.0].iter().enumerate() {
            let amp = 0.05 + 0.02 * k as f32;
            let mut ph = 0.0f64;
            for i in 0..400_000usize {
                ph += std::f64::consts::TAU * hz / RATE;
                x[300_000 + i] += C32::new(amp * ph.cos() as f32, amp * ph.sin() as f32);
            }
        }
        // Counted between blocks, not between events: a candidate that takes
        // the place of the quietest source opens before that one is closed.
        let mut open: Vec<SourceId> = Vec::new();
        let mut most = 0usize;
        for chunk in x.chunks(8192) {
            for e in d.process(chunk) {
                match e {
                    SourceEvent::Opened(s) => open.push(s.id),
                    SourceEvent::Closed(s) | SourceEvent::Superseded(s) => {
                        open.retain(|id| *id != s.id)
                    }
                }
            }
            most = most.max(open.len());
        }
        assert!(most <= 2, "{most} sources open at once, cap is 2");
        assert!(d.capped() > 0, "nothing was refused, so the cap was never reached");
    }

    #[test]
    fn a_long_keyed_burst_is_not_reopened_around_its_splatter() {
        // A wM-Bus meter: 100 kchip/s keyed 50 kHz either way, held for 16
        // ms, loud. Its skirts light up most of the span, and the centre of
        // everything lit wanders as they come and go, which read as a
        // transmitter that had outgrown its extraction: the source was
        // reopened three times as wide, centred off the signal, and every
        // front end placed on the narrow one went with it. rtl_433's mode C
        // captures decoded only once this stopped.
        let mut d = SourceDetector::new(RATE, RATE, cfg());
        let mut x = noise(1_000_000, 0.02, 37);
        let mut ph = 0.0f64;
        for i in 0..16_000usize {
            let f = if (i / 10) % 2 == 0 { 50_000.0 } else { -50_000.0 };
            ph += std::f64::consts::TAU * f / RATE;
            x[300_000 + i] += C32::new(0.9 * ph.cos() as f32, 0.9 * ph.sin() as f32);
        }
        let mut events = Vec::new();
        for chunk in x.chunks(8192) {
            events.extend(d.process(chunk).iter().copied());
        }
        let opened: Vec<Source> = events
            .iter()
            .filter_map(|e| match e {
                SourceEvent::Opened(s) => Some(*s),
                _ => None,
            })
            .collect();
        // The sidebands of keying this hard open sources of their own,
        // which is the detector reporting what is there; what must not
        // happen is the carrier being reopened around them.
        assert!(
            !events.iter().any(|e| matches!(e, SourceEvent::Superseded(_))),
            "superseded: {events:?}"
        );
        let carrier = opened
            .iter()
            .find(|s| s.lo_hz <= -50_000.0 && s.hi_hz >= 50_000.0)
            .unwrap_or_else(|| panic!("no source holds both tones: {opened:?}"));
        assert!(carrier.bandwidth_hz() < 300_000.0, "width {}", carrier.bandwidth_hz());
    }

    #[test]
    fn a_slow_chirp_is_reopened_at_its_full_width() {
        // 200 kHz swept in 40 ms, over and over: a symbol of chirp spread
        // spectrum at a high spreading factor. In the frames it takes to
        // open it is a tone a few kilohertz wide.
        let c = cfg();
        let mut d = SourceDetector::new(RATE, RATE, c);
        let mut e = SourceExtractor::new(RATE, 100e6, d.latency_samples(), c);
        let mut x = noise(1_000_000, 0.05, 41);
        let start = 300_000usize;
        let mut ph = 0.0f64;
        for i in 0..200_000usize {
            let t = (i % 40_000) as f64 / 40_000.0;
            let f = -100_000.0 + 200_000.0 * t;
            ph += std::f64::consts::TAU * f / RATE;
            x[start + i] += C32::new(0.3 * ph.cos() as f32, 0.3 * ph.sin() as f32);
        }
        let mut events = Vec::new();
        let mut blocks = Vec::new();
        for chunk in x.chunks(8192) {
            let ev = d.process(chunk).to_vec();
            events.extend_from_slice(&ev);
            e.process(chunk, &ev, &mut blocks);
        }
        let opened: Vec<Source> = events
            .iter()
            .filter_map(|e| match e {
                SourceEvent::Opened(s) => Some(*s),
                _ => None,
            })
            .collect();
        assert!(opened.len() >= 2, "never reopened: {events:?}");
        let last = opened.last().unwrap();
        assert!(last.bandwidth_hz() > 150_000.0, "final width {}", last.bandwidth_hz());
        assert!(events.iter().any(|e| matches!(e, SourceEvent::Superseded(_))));
        // The wide stream starts where the transmitter did, not where it was
        // noticed to be wide.
        let wide: Vec<&SourceBlock> = blocks.iter().filter(|b| b.id == last.id).collect();
        assert!(!wide.is_empty());
        assert!(
            wide[0].start_sample < start as u64 + 20_000,
            "restarted at {}",
            wide[0].start_sample
        );
        assert!(wide[0].rate >= 375_000.0, "rate {}", wide[0].rate);
        let old: Vec<&SourceBlock> = blocks.iter().filter(|b| b.id == opened[0].id).collect();
        assert_eq!(old.last().unwrap().state, SourceState::Superseded);
        // And the reopened stream is one stream: every block starts where
        // the one before it stopped.
        let factor = (RATE / wide[0].rate).round() as u64;
        let mut expect = wide[0].start_sample;
        for b in &wide {
            let off = b.start_sample as i64 - expect as i64;
            assert!(
                off.unsigned_abs() < factor,
                "reopened stream jumps at {} (expected {expect})",
                b.start_sample
            );
            expect = b.start_sample + b.samples.len() as u64 * factor;
        }
    }

    #[test]
    fn a_spur_one_bin_wide_opens_nothing() {
        // A bare tone: the tuner's leakage or a bare oscillator. Nothing
        // keyed is this narrow, and a carrier reported every half second
        // for as long as the receiver runs is a list of nothing.
        let mut d = SourceDetector::new(RATE, RATE, cfg());
        let mut x = noise(1_000_000, 0.05, 43);
        // Exactly on a bin centre, so it stays one bin wide.
        let hz = 64.0 * d.bin_hz();
        for (i, s) in tone(1_000_000, hz, RATE, 0.3).iter().enumerate() {
            x[i] += *s;
        }
        let mut opened = 0;
        for chunk in x.chunks(8192) {
            opened +=
                d.process(chunk).iter().filter(|e| matches!(e, SourceEvent::Opened(_))).count();
        }
        assert_eq!(opened, 0, "a one-bin spur opened a source");
    }

    #[test]
    fn floor_bias_is_modest_for_smoothed_power() {
        // Sanity on the derivation: a few frames of smoothing over a few
        // hundred frames of window is a correction of a few dB, not ten.
        let b = floor_bias(0.25, 1024);
        let db = 10.0 * b.log10();
        assert!((1.0..8.0).contains(&db), "{db} dB");
    }
}
