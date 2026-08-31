//! AIS on 162 MHz: two narrow channels, GMSK, and the HDLC link layer.
//!
//! AIS is the opposite problem to Mode S in every respect that matters here.
//! Mode S is one wide channel of 1 us pulses that has to be searched sample by
//! sample; AIS is two 25 kHz channels of 9600 baud GMSK, which is slow enough
//! that the work is in the framing rather than in the demodulation. So this
//! runs the standard narrowband chain per channel, mix down, filter, decimate,
//! discriminate, and spends its care on the clock and on HDLC.
//!
//! Both channels are always run. They carry the same kind of traffic and
//! stations alternate between them, so listening to one halves what is heard
//! for no saving worth having: at the decimated rate the whole chain is a few
//! percent of a core.
//!
//! # Why the link layer lives here and not in `decode`
//!
//! Everything from the flag search down is what decides whether a frame
//! happened at all: the flags delimit it, the destuffing recovers it, and the
//! frame check sequence is the only thing that separates a frame from a run of
//! noise that looked like one. That is an acceptance test, and it belongs with
//! the demodulator for the same reason Mode S puts its CRC inside the search.
//! What `decode::ais` gets is a payload that has already proved itself.
//!
//! # Bit order, which is where this went wrong
//!
//! HDLC puts each byte on the air least significant bit first, and that is
//! all there is to it: the payload is packed low bit first, exactly like the
//! AX.25 frames sharing this link layer. Reading the *fields* inside those
//! bytes most significant first is a separate question and `decode::ais`'s.
//!
//! This file said the opposite for a while, and every test agreed with it,
//! because `encode_slot` built its bits the same wrong way round. Twenty
//! synthetic tests cannot catch a convention shared by the encoder and the
//! decoder. What caught it was fourteen seconds of real off-air audio, in
//! which every frame passed the check sequence and every message decoded as
//! a binary broadcast from an impossible country. The check sequence runs
//! over the bit stream and never sees the packing, so it cannot object.

use crate::demod::FmDemod;
use crate::fir::FirDecim;
use crate::hdlc::{self, Hdlc};
use crate::mixer::Mixer;
use common::C32;

/// The two AIS channels, 87B and 88B, as absolute frequencies.
pub const CHANNEL_HZ: [f64; 2] = [161_975_000.0, 162_025_000.0];

/// Midway between the two channels: where a receiver tunes for AIS, and the
/// centre a frame from either channel is reported at.
pub const BAND_CENTER_HZ: f64 = 162_000_000.0;

/// Whether a packet's reported centre says it came off the AIS band.
///
/// This is how a consumer of the bus tells an AIS payload from a Mode S frame
/// without either of them carrying a label: they are both bytes, and where
/// they were received is evidence the packet already holds.
pub fn is_ais_band(center_hz: f64) -> bool {
    (center_hz - BAND_CENTER_HZ).abs() < 100_000.0
}

/// Symbol rate. Fixed by the standard, not a choice.
pub const BAUD: f64 = 9600.0;

/// Half the bandwidth the signal occupies. GMSK at 9600 baud with 2.4 kHz
/// deviation is about 14 kHz wide by Carson, so this passes the signal and
/// nothing of the neighbouring channel 25 kHz away.
const PASSBAND_HZ: f64 = 7_000.0;

/// Samples per symbol aimed for after decimation. Five is comfortably above
/// the two the clock recovery needs and keeps the decimated rate near 48 kHz.
const TARGET_SPS: f64 = 5.0;

/// Frames outside this range are not AIS. The short bound is the header a
/// message must have to identify anything; the long one is well past the
/// longest defined message and stops a stretch of noise being assembled into
/// something enormous before the check rejects it.
const MIN_FRAME_BITS: usize = 48;
const MAX_FRAME_BITS: usize = 1200;

#[derive(Clone, Copy, Debug)]
pub struct AisConfig {
    /// Discriminator output below this is treated as no signal, so a silent
    /// channel does not clock noise through the framer.
    pub min_level: f32,
    /// How hard a symbol transition pulls the bit clock. Higher locks faster
    /// on the preamble and jitters more on noise.
    pub clock_gain: f32,
}

impl Default for AisConfig {
    fn default() -> Self {
        Self { min_level: 0.02, clock_gain: 0.35 }
    }
}

/// A frame that passed its check sequence.
#[derive(Clone, Debug, PartialEq)]
pub struct AisFrame {
    /// The message, packed most significant bit first, as `decode::ais` reads
    /// it and as every published sentence is written.
    pub payload: Vec<u8>,
    /// 0 for channel A at 161.975 MHz, 1 for B at 162.025.
    pub channel: u8,
}

/// Everything below the discriminator: clock recovery, NRZI and HDLC.
///
/// Separated from the radio front end because the boundary is real. What a
/// receiver hands over at this point is the same whether it came from a
/// wideband capture mixed down here, from a narrowband FM receiver's
/// discriminator pin, or from a sound card. Splitting it is what lets a real
/// off-air recording test the parts where the risk actually is, without a
/// baseband capture of the whole band.
///
/// Polarity does not matter here, which is worth knowing before trying to
/// correct for it: NRZI encodes a bit as the presence or absence of a
/// transition, so inverting the discriminator inverts every level and leaves
/// every transition, and the decoded bits are identical.
pub struct AisSymbols {
    /// Running mean of the discriminator, which is the frequency error of the
    /// tuner and the transmitter together. Removed rather than trusted: it is
    /// tens of hertz to a few kHz and it decides the sign of every symbol.
    dc: f32,
    dc_alpha: f32,
    /// Samples per symbol at the rate being fed in.
    sps: f32,
    /// Samples since the last symbol was taken.
    since: f32,
    last_sign: bool,
    /// NRZI reference: the level the previous symbol sat at.
    prev_level: bool,
    hdlc: Hdlc,
}

impl AisSymbols {
    pub fn new(rate: f64) -> Self {
        Self {
            dc: 0.0,
            // A few hundred symbols of memory: long enough not to track the
            // data, short enough to follow a drifting tuner.
            dc_alpha: (1.0 / (rate as f32 / BAUD as f32 * 200.0)).min(0.05),
            sps: (rate / BAUD) as f32,
            since: 0.0,
            last_sign: false,
            prev_level: false,
            hdlc: Hdlc::new(MIN_FRAME_BITS, MAX_FRAME_BITS),
        }
    }

    pub fn reset(&mut self) {
        self.hdlc.reset();
        self.dc = 0.0;
        self.since = 0.0;
    }

    /// One discriminator sample: track the clock, and take a symbol when it
    /// comes round. Returns a payload when a frame closed and its check
    /// sequence held.
    pub fn push(&mut self, f: f32, cfg: &AisConfig) -> Option<Vec<u8>> {
        self.dc += self.dc_alpha * (f - self.dc);
        let v = f - self.dc;
        let sign = v > 0.0;

        // A transition marks a symbol boundary, so the next symbol centre is
        // half a symbol away. The clock is nudged towards that rather than
        // set to it, or every noise crossing would drag it.
        if sign != self.last_sign {
            let want = self.sps * 0.5;
            self.since += cfg.clock_gain * (want - self.since);
            self.last_sign = sign;
        }

        self.since += 1.0;
        if self.since < self.sps {
            return None;
        }
        self.since -= self.sps;

        // Below the floor there is no signal, and clocking noise into the
        // framer only gives the check sequence more chances to be fooled.
        if v.abs() < cfg.min_level {
            self.hdlc.reset();
            self.prev_level = sign;
            return None;
        }

        // NRZI: a zero is a transition, a one is no transition.
        let bit = sign == self.prev_level;
        self.prev_level = sign;
        // Least significant bit first, the same as AX.25: an AIS message is
        // a byte string and HDLC puts each byte on the air low bit first.
        // The fields inside those bytes are then read most significant first,
        // which is a different question and `decode::ais`'s.
        self.hdlc.push(bit).map(|f| hdlc::pack_lsb(&f))
    }
}

/// One AIS channel demodulated from a receiver's discriminator output.
///
/// The audio counterpart of [`AisDetector`], for a narrowband FM receiver
/// already tuned to 161.975 or 162.025, or a recording of one.
pub struct AisAudioDemod {
    syms: AisSymbols,
    cfg: AisConfig,
}

impl AisAudioDemod {
    pub fn new(rate: f64, cfg: AisConfig) -> Self {
        Self { syms: AisSymbols::new(rate), cfg }
    }

    pub fn reset(&mut self) {
        self.syms.reset();
    }

    /// Demodulate a block of discriminator audio, appending the payloads that
    /// passed their check sequence.
    pub fn process(&mut self, audio: &[f32], out: &mut Vec<Vec<u8>>) {
        for &x in audio {
            if let Some(f) = self.syms.push(x, &self.cfg) {
                out.push(f);
            }
        }
    }
}

/// One channel's receiver: the narrowband chain, then the framer.
struct ChannelRx {
    channel: u8,
    mixer: Mixer,
    decim: FirDecim,
    demod: FmDemod,
    /// Shifted baseband, decimated channel and discriminator output, all
    /// reused between blocks so a steady stream allocates nothing.
    mixed: Vec<C32>,
    narrow: Vec<C32>,
    freq: Vec<f32>,
    syms: AisSymbols,
}

impl ChannelRx {
    fn new(channel: u8, rate: f64, center_hz: f64) -> Self {
        let shift = center_hz - CHANNEL_HZ[channel as usize];
        // Land as near 48 kHz as the input rate allows; the clock recovery
        // takes the leftover as a fractional samples-per-symbol rather than
        // requiring the rate to divide evenly.
        let factor = (rate / (BAUD * TARGET_SPS)).round().max(1.0) as usize;
        let work = rate / factor as f64;
        Self {
            channel,
            mixer: Mixer::new(shift, rate),
            decim: FirDecim::design_hz(rate, factor, PASSBAND_HZ, 60.0),
            // Scaled so a symbol at full deviation reads near +/-1.
            demod: FmDemod::new(work, 2_400.0),
            mixed: Vec::new(),
            narrow: Vec::new(),
            freq: Vec::new(),
            syms: AisSymbols::new(work),
        }
    }

    fn process(&mut self, iq: &[C32], cfg: &AisConfig, out: &mut Vec<AisFrame>) {
        self.mixed.clear();
        self.mixer.process(iq, &mut self.mixed);
        self.narrow.clear();
        self.decim.process(&self.mixed, &mut self.narrow);
        self.freq.clear();
        self.demod.process(&self.narrow, &mut self.freq);

        // Moved out so the per-sample loop can take `&mut self`; the buffer
        // goes back afterwards, so nothing is reallocated.
        for &f in &self.freq {
            if let Some(payload) = self.syms.push(f, cfg) {
                out.push(AisFrame { payload, channel: self.channel });
            }
        }
    }

    fn reset(&mut self) {
        self.mixer.reset();
        self.decim.reset();
        self.demod.reset();
        self.syms.reset();
    }
}

/// Build the on-air symbol levels for a payload: training sequence, start
/// flag, bit-stuffed data and check sequence, end flag, NRZI encoded.
///
/// The inverse of what the detector does, and public for the same reason a
/// decoder's tests want an encoder: without a recorded capture the only
/// honest way to test a demodulator is to transmit something known and see
/// whether it comes back. `bits` is the message length, 168 for the common
/// messages and more for the longer ones.
pub fn encode_slot(payload: &[u8], bits: usize) -> Vec<bool> {
    // Least significant bit first within each byte, which is how HDLC puts a
    // byte on the air.
    let msg: Vec<bool> = (0..bits).map(|i| payload[i / 8] >> (i % 8) & 1 == 1).collect();
    // Twenty four bits of alternating training before the flag, which is what
    // gives the receiver's clock something to lock to.
    let training: Vec<bool> = (0..24).map(|i| i % 2 == 0).collect();
    hdlc::encode_frame(&msg, &training)
}

/// Both AIS channels, demodulated from one wideband stream.
pub struct AisDetector {
    rate: f64,
    cfg: AisConfig,
    chans: Vec<ChannelRx>,
}

impl AisDetector {
    pub fn new(rate: f64, center_hz: f64, cfg: AisConfig) -> Self {
        let chans =
            (0..CHANNEL_HZ.len() as u8).map(|c| ChannelRx::new(c, rate, center_hz)).collect();
        Self { rate, cfg, chans }
    }

    pub fn rate(&self) -> f64 {
        self.rate
    }

    /// Demodulate a block, appending whatever frames closed inside it.
    pub fn process(&mut self, iq: &[C32], out: &mut Vec<AisFrame>) {
        for c in &mut self.chans {
            c.process(iq, &self.cfg, out);
        }
    }

    pub fn reset(&mut self) {
        for c in &mut self.chans {
            c.reset();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Modulate a level sequence as FSK at the AIS rate and deviation.
    ///
    /// Plain FSK rather than GMSK: the discriminator does not care about the
    /// pulse shaping, and a test that had to implement a Gaussian filter to
    /// check the framer would be testing the wrong thing.
    pub(super) fn modulate(levels: &[bool], rate: f64, offset_hz: f64) -> Vec<C32> {
        let sps = rate / BAUD;
        let mut out = Vec::with_capacity((levels.len() as f64 * sps) as usize);
        let mut phase = 0.0f64;
        for &l in levels {
            let f = offset_hz + if l { 2_400.0 } else { -2_400.0 };
            for _ in 0..sps as usize {
                phase += std::f64::consts::TAU * f / rate;
                out.push(C32::new(phase.cos() as f32, phase.sin() as f32));
            }
        }
        out
    }

    /// Run a burst through a detector with silence either side.
    ///
    /// The trailing silence is not decoration: the decimator holds half its
    /// tap count, so a block that ends at the closing flag leaves that flag
    /// inside the filter and the frame never closes. Without this a test that
    /// expects no frames passes for the wrong reason.
    pub(super) fn run(iq: &[C32], rate: f64, center: f64) -> Vec<AisFrame> {
        let mut det = AisDetector::new(rate, center, AisConfig::default());
        let mut out = Vec::new();
        let quiet = vec![C32::new(0.0, 0.0); 4096];
        det.process(&quiet, &mut out);
        det.process(iq, &mut out);
        det.process(&quiet, &mut out);
        out
    }

    /// The whole link layer, end to end: a real message modulated onto a real
    /// channel and recovered by the detector.
    ///
    /// This is the test that pins the bit order down. Every layer here can be
    /// individually plausible and still produce nothing, because HDLC's byte
    /// order and the message's bit order disagree by design.
    #[test]
    fn a_modulated_frame_comes_back_out_of_the_detector() {
        // The Le Havre position report, the same bytes `decode::ais` is
        // tested against.
        let payload: Vec<u8> = vec![
            0x04, 0x36, 0x1f, 0x64, 0xa0, 0x20, 0x00, 0x00, 0x00, 0x99, 0xf6, 0x1c, 0x4f, 0x66,
            0x21, 0x6f, 0xff, 0x9c, 0x00, 0x56, 0x78,
        ];
        let rate = 2_400_000.0;
        let center = 162_000_000.0;
        let air = encode_slot(&payload, 168);
        // Put it on channel A, where the detector has to find it after mixing
        // 25 kHz down from the tuned centre.
        let offset = CHANNEL_HZ[0] - center;
        let iq = modulate(&air, rate, offset);

        let out = run(&iq, rate, center);
        assert_eq!(out.len(), 1, "expected exactly one frame, got {}", out.len());
        assert_eq!(out[0].channel, 0, "it was transmitted on channel A");
        assert_eq!(out[0].payload.len(), 21, "a 168 bit message is 21 bytes");
        assert_eq!(out[0].payload, payload, "the payload came back changed");
    }

    /// Noise must produce nothing. The check sequence is the only thing
    /// standing between a busy band and a map full of invented vessels, so
    /// this is the test that says it works.
    #[test]
    fn noise_produces_no_frames() {
        let rate = 2_400_000.0;
        let mut seed = 0x9e3779b97f4a7c15u64;
        let mut rng = || {
            seed ^= seed << 13;
            seed ^= seed >> 7;
            seed ^= seed << 17;
            (seed >> 40) as f32 / 8_388_608.0 - 1.0
        };
        let iq: Vec<C32> = (0..rate as usize / 4).map(|_| C32::new(rng(), rng())).collect();
        let mut det = AisDetector::new(rate, 162_000_000.0, AisConfig::default());
        let mut out = Vec::new();
        det.process(&iq, &mut out);
        assert!(out.is_empty(), "noise produced {} frames", out.len());
    }

    /// A frame whose check sequence fails is dropped rather than passed up
    /// with a flag saying so. Unlike a sensor reading, half a position is not
    /// worth showing: it puts a vessel somewhere it is not.
    #[test]
    fn a_corrupted_frame_is_dropped() {
        let payload = vec![0x04, 0x36, 0x1f, 0x64, 0xa0, 0x20, 0x00, 0x00, 0x00, 0x99, 0xf6,
            0x1c, 0x4f, 0x66, 0x21, 0x6f, 0xff, 0x9c, 0x00, 0x56, 0x78];
        let mut air = encode_slot(&payload, 168);
        // Flip a symbol well inside the frame, past the training and flag.
        air[60] = !air[60];
        let iq = modulate(&air, 2_400_000.0, CHANNEL_HZ[0] - 162_000_000.0);
        let out = run(&iq, 2_400_000.0, 162_000_000.0);
        assert!(out.is_empty(), "a corrupted frame was accepted");
    }

    /// Both channels are received, and a frame is attributed to the one it
    /// actually arrived on.
    #[test]
    fn a_frame_on_channel_b_is_reported_as_channel_b() {
        let payload = vec![0x04, 0x36, 0x1f, 0x64, 0xa0, 0x20, 0x00, 0x00, 0x00, 0x99, 0xf6,
            0x1c, 0x4f, 0x66, 0x21, 0x6f, 0xff, 0x9c, 0x00, 0x56, 0x78];
        let iq = modulate(&encode_slot(&payload, 168), 2_400_000.0, CHANNEL_HZ[1] - 162_000_000.0);
        let out = run(&iq, 2_400_000.0, 162_000_000.0);
        assert_eq!(out.len(), 1, "expected one frame on channel B");
        assert_eq!(out[0].channel, 1);
    }
}

#[cfg(test)]
mod tolerance {
    use super::tests::*;
    use super::*;

    fn payload() -> Vec<u8> {
        vec![
            0x04, 0x36, 0x1f, 0x64, 0xa0, 0x20, 0x00, 0x00, 0x00, 0x99, 0xf6, 0x1c, 0x4f, 0x66,
            0x21, 0x6f, 0xff, 0x9c, 0x00, 0x56, 0x78,
        ]
    }

    const RATE: f64 = 2_048_000.0;
    const CENTER: f64 = 162_000_000.0;

    /// A real receiver is never exactly on frequency, and the other tests all
    /// transmit with none of that error, so they cannot show this.
    ///
    /// At 162 MHz one part per million is 162 Hz against a deviation of 2400,
    /// so a few ppm of crystal error is a large fraction of the thing being
    /// measured. 2 kHz is about twelve ppm, past any HackRF and most of the
    /// cheap sticks, and the transmitter's own error on top.
    #[test]
    fn a_real_receivers_frequency_error_is_survivable() {
        for err in [0.0, 500.0, 1200.0, 2000.0] {
            let iq = modulate(&encode_slot(&payload(), 168), RATE, CHANNEL_HZ[0] - CENTER + err);
            let got = run(&iq, RATE, CENTER);
            assert_eq!(got.len(), 1, "lost the frame at {err} Hz of offset");
        }
    }

    /// Noise the frame survives, quoted at the antenna rather than in the
    /// channel.
    ///
    /// The distinction matters or the number flatters us: the noise here is
    /// spread across the whole 2 MS/s span and the channel filter keeps about
    /// 14 kHz of it, which is roughly 22 dB of processing gain. So surviving
    /// -10 dB measured wideband is about +12 dB in the channel, which is an
    /// ordinary demodulator rather than a miraculous one.
    #[test]
    fn a_frame_survives_more_noise_than_signal_across_the_span() {
        let clean = modulate(&encode_slot(&payload(), 168), RATE, CHANNEL_HZ[0] - CENTER);
        let mut seed = 0x243f_6a88_85a3_08d3u64;
        let mut rng = || {
            seed ^= seed << 13;
            seed ^= seed >> 7;
            seed ^= seed << 17;
            (seed >> 40) as f32 / 8_388_608.0 - 1.0
        };
        for snr_db in [20.0f32, 6.0, 0.0] {
            let n = 10f32.powf(-snr_db / 20.0);
            let noisy: Vec<C32> =
                clean.iter().map(|s| s + C32::new(rng() * n, rng() * n)).collect();
            assert_eq!(
                run(&noisy, RATE, CENTER).len(),
                1,
                "lost the frame at {snr_db} dB across the span"
            );
        }
    }
}
