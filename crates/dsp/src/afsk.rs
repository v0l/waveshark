//! Two audio tones and a bit clock, which is what APRS rides on.
//!
//! Bell 202 is one keying of it and the default here; MDC-1200 is another,
//! 1200 Hz against 1800 Hz at the same 1200 baud, and an iMet sonde a third.
//! The tones and the baud are a parameter ([`Tones`]) because nothing in the
//! correlator pair cares which two frequencies it is told to watch.
//!
//! Two layers of modulation, and keeping them straight is most of the work.
//! The radio channel is ordinary narrowband FM, so the first step is the same
//! discriminator a voice channel uses. What comes out is *audio*, and the data
//! is in that audio as two tones: 1200 Hz for a mark and 2200 Hz for a space.
//! So this demodulates audio, not RF, and the RF side is a plain NFM receiver
//! that happens to be feeding it.
//!
//! # Telling 1200 from 2200
//!
//! The tones are close together, barely under an octave, and a symbol is only
//! a bit over one cycle of the lower one. That rules out anything that needs
//! to watch a tone for a while: at 1200 baud a Goertzel bin wide enough to
//! resolve the two is wider than the symbol.
//!
//! What works at this ratio is a correlator pair. Multiply the audio by a
//! reference at each tone, integrate over one symbol, and compare the two
//! magnitudes. Both quadratures are needed at each tone because nothing
//! synchronises the receiver's reference to the transmitter's phase, and a
//! single quadrature nulls whenever the two are ninety degrees apart, which
//! reads as the other tone.
//!
//! # What sits above
//!
//! The bit stream is NRZI and then HDLC, both shared with AIS in
//! [`crate::hdlc`]. AX.25 is HDLC, so once the tones are decided the rest is
//! the same link layer on a different band.

use crate::hdlc::{self, Hdlc};

/// Symbol rate, fixed by Bell 202.
pub const BAUD: f64 = 1200.0;

/// The two tones. Mark is the lower one, which is the convention every AX.25
/// document uses.
pub const MARK_HZ: f64 = 1200.0;
pub const SPACE_HZ: f64 = 2200.0;

/// One keying: the two tones and the rate they are sent at.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Tones {
    /// The tone a one is sent as, the lower of the two by convention.
    pub mark_hz: f64,
    pub space_hz: f64,
    pub baud: f64,
}

/// Bell 202: 1200 Hz and 2200 Hz at 1200 baud, which is AX.25 on 2 m.
pub const BELL202: Tones = Tones { mark_hz: MARK_HZ, space_hz: SPACE_HZ, baud: BAUD };

/// SAME: the header bursts an EAS or NOAA Weather Radio alert opens with.
/// The only keying here whose mark is the *higher* tone, which is what the
/// standard says and what the bit decisions above it assume.
pub const SAME: Tones = Tones { mark_hz: 2083.3, space_hz: 1562.5, baud: 520.83 };

/// CCIR fast FSK: 1200 Hz and 1800 Hz at 1200 baud, phase continuous, which
/// is what MDC-1200 and MPT1327 key inside an FM voice channel.
pub const FFSK1200: Tones = Tones { mark_hz: 1200.0, space_hz: 1800.0, baud: BAUD };

/// Whether a packet's reported centre says it came off the 2 m packet
/// segment.
///
/// The counterpart of `ais::is_ais_band`, and used the same way: a consumer of
/// the bus tells an AX.25 frame from a Mode S one by where it was received,
/// since both are bytes and that is evidence the packet already carries.
/// A range rather than a frequency because the scanner decides where APRS
/// listens, and 144.800, 144.390 and 144.640 are all in use.
pub fn is_packet_band(center_hz: f64) -> bool {
    (144_000_000.0..146_000_000.0).contains(&center_hz)
}

/// An AX.25 frame is at least an address pair, a control byte and a PID, and
/// at most that plus 256 bytes of information. In bits, with the check
/// sequence, that brackets what the framer should assemble.
const MIN_FRAME_BITS: usize = 136;
const MAX_FRAME_BITS: usize = 2400;

#[derive(Clone, Copy, Debug)]
pub struct AfskConfig {
    /// Correlator output below this is treated as no signal, so a silent
    /// channel does not clock noise into the framer.
    pub min_level: f32,
    /// How hard a symbol transition pulls the bit clock.
    pub clock_gain: f32,
}

impl Default for AfskConfig {
    fn default() -> Self {
        Self { min_level: 1e-4, clock_gain: 0.35 }
    }
}

/// One tone correlator: a sliding integration of the audio against a complex
/// reference, whose magnitude says how much of that tone is present.
struct Tone {
    /// Reference phase step per sample.
    step: f32,
    phase: f32,
    /// One symbol of history for each quadrature, as a circular buffer, so the
    /// integration is a running sum rather than a fresh pass per sample.
    i_hist: Vec<f32>,
    q_hist: Vec<f32>,
    pos: usize,
    i_sum: f32,
    q_sum: f32,
}

impl Tone {
    fn new(freq: f64, rate: f64, window: usize) -> Self {
        Self {
            step: (std::f64::consts::TAU * freq / rate) as f32,
            phase: 0.0,
            i_hist: vec![0.0; window],
            q_hist: vec![0.0; window],
            pos: 0,
            i_sum: 0.0,
            q_sum: 0.0,
        }
    }

    /// Feed one audio sample, returning the tone's current power.
    fn push(&mut self, x: f32) -> f32 {
        let (s, c) = self.phase.sin_cos();
        self.phase += self.step;
        if self.phase > std::f32::consts::TAU {
            self.phase -= std::f32::consts::TAU;
        }
        let (i, q) = (x * c, x * s);
        // Running sum: add the new sample, drop the one leaving the window.
        self.i_sum += i - self.i_hist[self.pos];
        self.q_sum += q - self.q_hist[self.pos];
        self.i_hist[self.pos] = i;
        self.q_hist[self.pos] = q;
        self.pos = (self.pos + 1) % self.i_hist.len();
        self.i_sum * self.i_sum + self.q_sum * self.q_sum
    }

    fn reset(&mut self) {
        self.i_hist.fill(0.0);
        self.q_hist.fill(0.0);
        self.i_sum = 0.0;
        self.q_sum = 0.0;
        self.pos = 0;
        self.phase = 0.0;
    }
}

/// Bell 202 audio to HDLC frames.
///
/// Takes the discriminator output of an NFM receiver, not RF: what modulation
/// the channel used is the caller's business, and a soundcard fed from a
/// handheld would present the same samples.
/// One symbol the tones decided, and whether there was a signal to decide
/// it from.
///
/// The quiet flag is not a detail of the framer above: a channel with
/// nothing on it still produces symbols, and every framer has to know the
/// difference between a mark and silence. HDLC restarts on it; an
/// asynchronous line reads it as the idle between characters.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct Symbol {
    /// True for the mark tone, the lower of the two.
    pub mark: bool,
    pub quiet: bool,
}

/// The tone pair and the bit clock: audio in, symbols out.
///
/// Everything above this differs by protocol. APRS reads the symbols as NRZI
/// and then HDLC ([`AfskDemod`]); an iMet radiosonde reads them as an
/// ordinary asynchronous line with a start bit and a stop bit. Neither is a
/// property of Bell 202, which is why the split is here.
pub struct AfskBits {
    cfg: AfskConfig,
    mark: Tone,
    space: Tone,
    /// Samples per symbol at the audio rate.
    sps: f32,
    since: f32,
    last_sign: bool,
}

impl AfskBits {
    pub fn new(rate: f64, cfg: AfskConfig) -> Self {
        Self::with_tones(rate, BELL202, cfg)
    }

    pub fn with_tones(rate: f64, tones: Tones, cfg: AfskConfig) -> Self {
        // Integrate over exactly one symbol. Shorter and the two tones are not
        // resolved; longer and the correlator straddles a transition and reads
        // both tones at once.
        let window = ((rate / tones.baud).round() as usize).max(2);
        Self {
            cfg,
            mark: Tone::new(tones.mark_hz, rate, window),
            space: Tone::new(tones.space_hz, rate, window),
            sps: (rate / tones.baud) as f32,
            since: 0.0,
            last_sign: false,
        }
    }

    pub fn reset(&mut self) {
        self.mark.reset();
        self.space.reset();
        self.since = 0.0;
    }

    /// Decide a symbol at every bit instant in this block of audio.
    pub fn process(&mut self, audio: &[f32], out: &mut Vec<Symbol>) {
        for &x in audio {
            let m = self.mark.push(x);
            let s = self.space.push(x);
            // The decision is which tone is stronger, so the difference is the
            // signal and its magnitude is the confidence.
            let sign = m - s > 0.0;

            // A transition marks a symbol boundary, so the next symbol centre
            // is half a symbol away. Nudged towards it rather than set to it,
            // or every noise crossing would drag the clock.
            if sign != self.last_sign {
                let want = self.sps * 0.5;
                self.since += self.cfg.clock_gain * (want - self.since);
                self.last_sign = sign;
            }

            self.since += 1.0;
            if self.since < self.sps {
                continue;
            }
            self.since -= self.sps;
            out.push(Symbol { mark: sign, quiet: (m + s) < self.cfg.min_level });
        }
    }
}

pub struct AfskDemod {
    bits: AfskBits,
    symbols: Vec<Symbol>,
    /// NRZI reference: the level the previous symbol sat at.
    prev_level: bool,
    hdlc: Hdlc,
}

impl AfskDemod {
    pub fn new(rate: f64, cfg: AfskConfig) -> Self {
        Self {
            bits: AfskBits::new(rate, cfg),
            symbols: Vec::new(),
            prev_level: false,
            hdlc: Hdlc::new(MIN_FRAME_BITS, MAX_FRAME_BITS),
        }
    }

    pub fn reset(&mut self) {
        self.bits.reset();
        self.hdlc.reset();
    }

    /// Demodulate a block of audio, appending the AX.25 frames that closed
    /// inside it. Frames are packed least significant bit first, which is how
    /// HDLC puts bytes on the air and therefore how AX.25 fields read.
    pub fn process(&mut self, audio: &[f32], out: &mut Vec<Vec<u8>>) {
        let mut symbols = std::mem::take(&mut self.symbols);
        symbols.clear();
        self.bits.process(audio, &mut symbols);
        for sym in &symbols {
            if sym.quiet {
                self.hdlc.reset();
                self.prev_level = sym.mark;
                continue;
            }
            // NRZI: a zero is a transition, a one is no transition.
            let bit = sym.mark == self.prev_level;
            self.prev_level = sym.mark;
            if let Some(frame) = self.hdlc.push(bit) {
                out.push(hdlc::pack_lsb(&frame));
            }
        }
        self.symbols = symbols;
    }
}

/// Modulate bytes as Bell 202 audio, for tests and for anything that wants to
/// transmit. `lead` is the flag count before the frame, which a real
/// transmitter uses to let the receiver's clock settle.
pub fn encode(frame: &[u8], rate: f64, lead_flags: usize) -> Vec<f32> {
    // AX.25 bytes go on the air least significant bit first.
    let data: Vec<bool> = (0..frame.len() * 8).map(|i| frame[i / 8] >> (i % 8) & 1 == 1).collect();
    let lead: Vec<bool> = std::iter::repeat_n(hdlc::flag_bits(), lead_flags).flatten().collect();
    modulate(&hdlc::encode_frame(&data, &lead), rate, BELL202)
}

/// Symbols to audio, phase continuous across every boundary: true is the
/// mark tone. A discontinuity at a transition costs the next several bits in
/// any receiver that integrates phase, which is what reads fast FSK.
pub fn modulate(levels: &[bool], rate: f64, tones: Tones) -> Vec<f32> {
    let sps = rate / tones.baud;
    let mut out = Vec::with_capacity((levels.len() as f64 * sps) as usize);
    let mut phase = 0.0f64;
    for &level in levels {
        let f = if level { tones.mark_hz } else { tones.space_hz };
        for _ in 0..sps as usize {
            phase += std::f64::consts::TAU * f / rate;
            out.push(phase.sin() as f32);
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A minimal AX.25 UI frame: destination, source, control, PID, info.
    fn ax25_frame() -> Vec<u8> {
        let mut f = Vec::new();
        // Callsigns are shifted left one bit in AX.25, leaving the low bit as
        // the address-extension flag.
        for (call, ssid, last) in [("APRS  ", 0u8, false), ("EI2ABC", 7, true)] {
            for c in call.bytes() {
                f.push(c << 1);
            }
            f.push(0x60 | (ssid << 1) | u8::from(last));
        }
        f.push(0x03); // UI
        f.push(0xF0); // no layer 3
        f.extend_from_slice(b"!5338.00N/00615.00W-test");
        f
    }

    /// The whole audio path: modulated Bell 202 in, the same bytes out.
    #[test]
    fn a_modulated_frame_comes_back_out_of_the_demodulator() {
        let rate = 48_000.0;
        let frame = ax25_frame();
        let audio = encode(&frame, rate, 8);

        let mut d = AfskDemod::new(rate, AfskConfig::default());
        let mut out = Vec::new();
        d.process(&vec![0.0; 2048], &mut out);
        d.process(&audio, &mut out);
        d.process(&vec![0.0; 2048], &mut out);

        assert_eq!(out.len(), 1, "expected one frame, got {}", out.len());
        assert_eq!(out[0], frame, "the frame came back changed");
    }

    /// The tones are barely under an octave apart and a symbol is only a bit
    /// over one cycle of the lower one, so this is the property the correlator
    /// window is sized for.
    #[test]
    fn the_two_tones_are_told_apart_within_one_symbol() {
        let rate = 48_000.0;
        let window = (rate / BAUD) as usize;
        for (freq, expect_mark) in [(MARK_HZ, true), (SPACE_HZ, false)] {
            let mut mark = Tone::new(MARK_HZ, rate, window);
            let mut space = Tone::new(SPACE_HZ, rate, window);
            let (mut m, mut s) = (0.0, 0.0);
            for n in 0..window * 2 {
                let x = (std::f64::consts::TAU * freq * n as f64 / rate).sin() as f32;
                m = mark.push(x);
                s = space.push(x);
            }
            assert_eq!(m > s, expect_mark, "{freq} Hz was read as the wrong tone");
        }
    }

    /// Noise must produce nothing. The check sequence is all that stands
    /// between a busy band and invented stations.
    #[test]
    fn noise_produces_no_frames() {
        let mut seed = 0x9e37_79b9_7f4a_7c15u64;
        let mut rng = || {
            seed ^= seed << 13;
            seed ^= seed >> 7;
            seed ^= seed << 17;
            (seed >> 40) as f32 / 8_388_608.0 - 1.0
        };
        let audio: Vec<f32> = (0..48_000 * 4).map(|_| rng()).collect();
        let mut d = AfskDemod::new(48_000.0, AfskConfig::default());
        let mut out = Vec::new();
        d.process(&audio, &mut out);
        assert!(out.is_empty(), "noise produced {} frames", out.len());
    }

    /// A corrupted frame is dropped rather than passed up with a flag.
    #[test]
    fn a_corrupted_frame_is_dropped() {
        let rate = 48_000.0;
        let mut audio = encode(&ax25_frame(), rate, 8);
        // Wipe a few symbols well inside the frame.
        let at = audio.len() / 2;
        for x in &mut audio[at..at + 200] {
            *x = 0.0;
        }
        let mut d = AfskDemod::new(rate, AfskConfig::default());
        let mut out = Vec::new();
        d.process(&audio, &mut out);
        d.process(&vec![0.0; 2048], &mut out);
        assert!(out.is_empty(), "a corrupted frame was accepted");
    }

    /// Real audio arrives at whatever rate the receiver's chain produced.
    #[test]
    fn it_works_at_the_rates_a_receiver_actually_produces() {
        for rate in [22_050.0, 44_100.0, 48_000.0] {
            let frame = ax25_frame();
            let audio = encode(&frame, rate, 8);
            let mut d = AfskDemod::new(rate, AfskConfig::default());
            let mut out = Vec::new();
            d.process(&audio, &mut out);
            d.process(&vec![0.0; 2048], &mut out);
            assert_eq!(out.len(), 1, "no frame at {rate} Hz");
            assert_eq!(out[0], frame);
        }
    }
}
