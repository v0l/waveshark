//! M17: 4-FSK at 4800 baud, framed 25 times a second, with an open codec.
//!
//! The link layer only. What leaves here is a frame whose forward error
//! correction was run and whose contents explain the bits that arrived; what
//! the frame *means*, the callsigns, the stream type, the packet text, is
//! `decode::m17`, for the same reason the POCSAG and AIS splits are where
//! they are.
//!
//! # Why this does not use the four-level front end
//!
//! [`crate::c4fm`] recovers symbols from a burst it gates on envelope, which
//! is the right shape for FLEX or a wireless M-Bus meter: a short packet with
//! silence either side. An M17 transmission is a continuous carrier that can
//! last minutes, and its clock has to stay locked across the whole thing. But
//! M17 puts a 16 bit synchronisation burst in front of *every* 40 ms frame,
//! so a receiver never has to hold a clock for longer than one frame: it
//! correlates for the next sync, and the 184 symbols behind it are the
//! payload. Drift over one frame at any believable crystal error is a
//! hundredth of a symbol. That makes the framing simpler than a burst
//! detector, not harder, and it works on a signal that never stops.
//!
//! # The polarity trap
//!
//! The four sync words are two complementary pairs: inverting every symbol
//! turns the link setup sync into the stream sync, and the BERT sync into the
//! packet sync. Whether the discriminator's output is the right way up
//! depends on the receiver, not on the transmitter, so correlation alone
//! cannot tell "a link setup frame received normally" from "a stream frame
//! received inverted". Both readings are tried, and only one of them will
//! have a link setup CRC that checks, a link information channel whose Golay
//! codewords decode, or contents that re-encode to the bits that arrived.
//! Once one does, the polarity is known and held for the rest of the
//! transmission.

pub mod fec;

use crate::fir::FirDecimReal;
use crate::fourlevel;
use fec::PAYLOAD_BITS;

/// Symbol rate. 9600 bits per second, two bits to a symbol.
pub const BAUD: f64 = 4_800.0;

/// Deviation of the outer symbols. The inner pair sit at a third of it.
pub const DEVIATION_HZ: f64 = 2_400.0;

/// Occupied bandwidth. The specification is written for a 9 kHz transmission
/// in a 12.5 kHz channel.
pub const CHANNEL_WIDTH_HZ: f64 = 9_000.0;

/// Symbols in a frame: 8 of sync and 184 of payload, 40 ms in all.
pub const SYMBOLS_PER_FRAME: usize = 192;
pub const SYNC_SYMBOLS: usize = 8;
pub const PAYLOAD_SYMBOLS: usize = 184;

/// The four sync words, as they go on the air.
pub const SYNC_LSF: u16 = 0x55F7;
pub const SYNC_STREAM: u16 = 0xFF5D;
pub const SYNC_PACKET: u16 = 0x75FF;
pub const SYNC_BERT: u16 = 0xDF55;

/// Which kind of frame a sync word introduced.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Kind {
    Lsf,
    Stream,
    Packet,
    Bert,
}

/// A frame's contents, after its error correction has been run.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Body {
    /// The 30 byte link setup frame: destination, source, type, metadata and
    /// its CRC. Only emitted when that CRC checks.
    Lsf([u8; 30]),
    /// One 40 ms slice of a stream: a sixth of the link setup frame in the
    /// link information channel, the frame number, and 128 bits of payload,
    /// which for a voice stream is two Codec 2 frames.
    Stream {
        lich: [u8; 6],
        /// Bits the Golay decoder had to change across the four chunks.
        lich_errors: u32,
        number: u16,
        /// The frame number's top bit, which the transmitter sets on the last
        /// frame of a stream.
        last: bool,
        payload: [u8; 16],
    },
    /// A 25 byte slice of a packet, with the counter that says where it goes.
    Packet {
        data: [u8; 25],
        /// Set on the last frame, where `counter` is the number of valid
        /// bytes in this frame rather than the frame's position.
        eof: bool,
        counter: u8,
    },
    /// A bit error rate test frame. Its contents are a PRBS, so there is
    /// nothing to report but the fact of it and how cleanly it read.
    Bert,
}

/// One frame off the air.
#[derive(Clone, Debug, PartialEq)]
pub struct Frame {
    pub body: Body,
    /// Fraction of the received bits that disagree with the decoded frame
    /// re-encoded. Zero on a clean signal, and the measure a marginal
    /// reception should be judged by.
    pub ber: f32,
    /// How well the sync burst matched, -1 to 1. The sign says which of a
    /// complementary pair of sync words matched, not which way up the signal
    /// was: those two questions have the same answer only once the polarity
    /// is known.
    pub correlation: f32,
    /// RMS distance of the payload symbols from the four levels fitted to
    /// them, in steps.
    pub evm: f32,
    /// Sample index the sync burst started at, for lining a frame up with a
    /// waterfall.
    pub start_sample: u64,
}

impl Frame {
    pub fn kind(&self) -> Kind {
        match self.body {
            Body::Lsf(_) => Kind::Lsf,
            Body::Stream { .. } => Kind::Stream,
            Body::Packet { .. } => Kind::Packet,
            Body::Bert => Kind::Bert,
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct M17Config {
    /// How well the eight sync symbols must match before a frame is read out
    /// behind them. Correlation is scale and offset free, so this is a
    /// judgement about shape alone.
    pub min_correlation: f32,
    /// Largest disagreement between the decoded frame re-encoded and the bits
    /// that arrived, for a frame that carries a check of its own.
    pub max_ber: f32,
    /// The same, for a frame that carries none.
    ///
    /// Tighter, and the reason is worth stating: the disagreement rate a
    /// convolutional decoder reaches on *noise* is not a half. A rate 1/2
    /// K=5 code has enough freedom to explain about seven eighths of a
    /// random bit stream, so a false sync followed by 368 bits of noise
    /// re-encodes at around 0.12, measured over pure noise in this module's
    /// tests. Real frames stay under 0.05 well past the point where they
    /// still decode correctly, so the gap is there, but it is a gap of a few
    /// hundredths rather than the half it looks like it should be.
    pub max_unchecked_ber: f32,
    /// Bits the Golay decoder may repair across a frame's four link
    /// information chunks before the frame is refused.
    pub max_lich_errors: u32,
    /// Frames of silence before the receiver forgets which way up the signal
    /// was. One transmission's worth of hangover, so a pause between overs
    /// does not cost the polarity.
    pub hold_frames: u32,
}

impl Default for M17Config {
    fn default() -> Self {
        Self {
            min_correlation: 0.85,
            max_ber: 0.10,
            max_unchecked_ber: 0.05,
            max_lich_errors: 8,
            hold_frames: 50,
        }
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct M17Stats {
    pub frames: u64,
    /// Sync bursts that correlated but whose contents were refused, which is
    /// what a false sync in noise looks like.
    pub rejected: u64,
}

/// The receiver: discriminator output in, frames out.
///
/// Takes real samples rather than complex baseband because the demodulation
/// M17 needs is an ordinary FM discriminator, which every channel in this
/// receiver already has. What this adds is the matched filter, the sync
/// search and the framing.
pub struct M17Demod {
    cfg: M17Config,
    sps: f64,
    rrc: FirDecimReal,
    /// Matched-filtered samples not yet consumed by the sync search.
    buf: Vec<f32>,
    /// Absolute sample index of `buf[0]`.
    origin: u64,
    /// Where in `buf` the next sync search starts.
    hunt: usize,
    /// Whether the symbols arrive upside down, once something has proved it.
    invert: Option<bool>,
    /// Frames since anything decoded, for expiring `invert`.
    idle: u32,
    syms: Vec<f32>,
    scratch: Vec<f32>,
    soft: Vec<f32>,
    typed: Vec<f32>,
    stats: M17Stats,
}

impl M17Demod {
    pub fn new(rate: f64, cfg: M17Config) -> Self {
        let sps = rate / BAUD;
        Self {
            cfg,
            sps,
            rrc: FirDecimReal::new(rrc_taps(sps, 0.5, 8), 1),
            buf: Vec::new(),
            origin: 0,
            hunt: 0,
            invert: None,
            idle: 0,
            syms: Vec::new(),
            scratch: Vec::new(),
            soft: vec![0.0; PAYLOAD_BITS],
            typed: vec![0.0; PAYLOAD_BITS],
            stats: M17Stats::default(),
        }
    }

    pub fn stats(&self) -> M17Stats {
        self.stats
    }

    pub fn take_stats(&mut self) -> M17Stats {
        std::mem::take(&mut self.stats)
    }

    pub fn reset(&mut self) {
        self.rrc.reset();
        self.buf.clear();
        self.hunt = 0;
        self.invert = None;
        self.idle = 0;
    }

    /// Feed a block of discriminator output, appending any completed frames.
    ///
    /// The input should be scaled so that the outer symbols sit near ±1,
    /// which an [`crate::FmDemod`] built for [`DEVIATION_HZ`] gives, but
    /// nothing here depends on it: both the sync search and the symbol
    /// slicing measure their own scale from the signal.
    pub fn process(&mut self, audio: &[f32], out: &mut Vec<Frame>) {
        self.rrc.process(audio, &mut self.buf);

        // A frame plus a symbol of slack either side, which is what the sync
        // search needs in hand before it can commit to a position.
        let span = (SYMBOLS_PER_FRAME as f64 * self.sps).ceil() as usize + 2;
        while self.hunt + span < self.buf.len() {
            match self.scan() {
                Some(frame) => {
                    let step = (SYMBOLS_PER_FRAME as f64 * self.sps).round() as usize;
                    // Start hunting a symbol early: the next sync is one frame
                    // on, and meeting it slightly before it arrives is what
                    // lets the search find its peak rather than its edge.
                    self.hunt += step.saturating_sub(self.sps as usize);
                    self.idle = 0;
                    self.stats.frames += 1;
                    out.push(frame);
                }
                None => self.hunt += 1,
            }
        }

        // Everything before the hunt position has been decided on.
        if self.hunt > 1 << 14 {
            self.buf.drain(..self.hunt);
            self.origin += self.hunt as u64;
            self.hunt = 0;
        }
    }

    /// Try to read a frame whose sync burst starts at the hunt position.
    ///
    /// Returns `None` when nothing correlates there, or when what did
    /// correlate could not be decoded, which is the same answer as far as the
    /// caller is concerned: move on by a sample and try again.
    fn scan(&mut self) -> Option<Frame> {
        let (mut best, mut at, mut template) = (0.0f32, self.hunt, SYNC_LSF);
        for sync in [SYNC_LSF, SYNC_BERT] {
            let r = self.correlate(self.hunt, sync);
            if r.abs() > best.abs() {
                best = r;
                template = sync;
            }
        }
        if best.abs() < self.cfg.min_correlation {
            self.idle = self.idle.saturating_add(1);
            if self.idle > self.cfg.hold_frames * self.sps as u32 * SYMBOLS_PER_FRAME as u32 {
                self.invert = None;
            }
            return None;
        }

        // Walk to the top of the peak. A sync burst correlates over about a
        // symbol either side of where it really is, and reading the payload
        // from the wrong edge of that costs half a symbol of timing.
        let window = self.sps.ceil() as usize;
        for n in self.hunt + 1..=self.hunt + window {
            let r = self.correlate(n, template);
            if r.abs() > best.abs() {
                best = r;
                at = n;
            }
        }

        // Which frame type this is depends on a polarity the signal cannot
        // report; see the module note. A positive correlation against the
        // link setup template is a link setup frame the right way up or a
        // stream frame upside down, and only the decode can say which.
        let pair = match template {
            SYNC_LSF => (Kind::Lsf, Kind::Stream),
            _ => (Kind::Bert, Kind::Packet),
        };
        let (upright, inverted) =
            if best > 0.0 { (pair.0, pair.1) } else { (pair.1, pair.0) };

        self.slice(at);
        let mut got = None;
        for (kind, invert) in [(upright, false), (inverted, true)] {
            if self.invert.is_some_and(|v| v != invert) {
                continue;
            }
            if let Some((body, ber)) = self.decode(kind, invert) {
                got = Some((body, ber, invert));
                break;
            }
        }
        let Some((body, ber, invert)) = got else {
            self.stats.rejected += 1;
            return None;
        };
        self.invert = Some(invert);

        let fit = fourlevel::levels(&mut self.scratch, &self.syms);
        Some(Frame {
            body,
            ber,
            correlation: best,
            evm: fit.map(|f| f.evm(&self.syms)).unwrap_or(f32::INFINITY),
            start_sample: self.origin + at as u64,
        })
    }

    /// Pearson correlation between the eight symbols at `n` and a sync word.
    ///
    /// Correlation rather than a distance because it is free of both the
    /// tuning offset the discriminator rides on and the deviation the
    /// transmitter chose, neither of which is known when the search runs.
    fn correlate(&self, n: usize, sync: u16) -> f32 {
        let mut x = [0.0f32; SYNC_SYMBOLS];
        for (k, v) in x.iter_mut().enumerate() {
            *v = interp(&self.buf, n as f64 + k as f64 * self.sps);
        }
        let t = sync_symbols(sync);
        let mx = x.iter().sum::<f32>() / SYNC_SYMBOLS as f32;
        let mt = t.iter().sum::<f32>() / SYNC_SYMBOLS as f32;
        let mut num = 0.0;
        let mut dx = 0.0;
        let mut dt = 0.0;
        for k in 0..SYNC_SYMBOLS {
            let (a, b) = (x[k] - mx, t[k] - mt);
            num += a * b;
            dx += a * a;
            dt += b * b;
        }
        let den = (dx * dt).sqrt();
        if den > 1e-12 {
            num / den
        } else {
            0.0
        }
    }

    /// Collect the 184 payload symbols behind a sync burst at `n`.
    fn slice(&mut self, n: usize) {
        self.syms.clear();
        for i in 0..PAYLOAD_SYMBOLS {
            let pos = n as f64 + (SYNC_SYMBOLS + i) as f64 * self.sps;
            self.syms.push(interp(&self.buf, pos));
        }
    }

    /// Read the sliced symbols as a frame of the given kind.
    ///
    /// The level fit is per frame rather than tracked, because a frame is all
    /// the context needed: 184 symbols of randomised payload use all four
    /// levels evenly, so the centre and the step follow from the frame
    /// itself and no automatic gain control has to be right.
    fn decode(&mut self, kind: Kind, invert: bool) -> Option<(Body, f32)> {
        let fit = fourlevel::levels(&mut self.scratch, &self.syms)?;
        let sign = if invert { -1.0 } else { 1.0 };
        for (i, &s) in self.syms.iter().enumerate() {
            let v = sign * (s - fit.center) / fit.step;
            // The dibit's first bit says which side of zero the symbol is on
            // and its second says whether it is an outer level, so both are
            // straight lines through the constellation rather than a table.
            self.soft[2 * i] = (-v * 0.5).clamp(-1.0, 1.0);
            self.soft[2 * i + 1] = (v.abs() - 2.0).clamp(-1.0, 1.0);
        }
        fec::derandomize(&mut self.soft);
        let (soft, mut typed) = (&self.soft, std::mem::take(&mut self.typed));
        fec::deinterleave(soft, &mut typed);
        self.typed = typed;

        let body = match kind {
            Kind::Lsf => {
                let (bits, ber) = fec::viterbi(&self.typed, &fec::P1, 240);
                if ber > self.cfg.max_ber {
                    return None;
                }
                let bytes = fec::pack(&bits);
                // The only real integrity check in the protocol, and the one
                // thing that makes a link setup frame worth reporting: a
                // callsign read out of a frame that failed here is a
                // callsign nobody transmitted.
                if fec::crc16(&bytes) != 0 {
                    return None;
                }
                let mut lsf = [0u8; 30];
                lsf.copy_from_slice(&bytes);
                (Body::Lsf(lsf), ber)
            }
            Kind::Stream => {
                let mut lich_bits = Vec::with_capacity(48);
                let mut lich_errors = 0;
                for chunk in 0..4 {
                    let mut word = 0u32;
                    for i in 0..24 {
                        word = word << 1 | u32::from(self.typed[chunk * 24 + i] > 0.0);
                    }
                    let (data, fixed) = fec::golay_decode(word)?;
                    lich_errors += fixed;
                    for i in (0..12).rev() {
                        lich_bits.push((data >> i & 1) as u8);
                    }
                }
                if lich_errors > self.cfg.max_lich_errors {
                    return None;
                }
                // The counter says which sixth of the link setup frame this
                // chunk is, so only six of its eight values exist. Cheap,
                // and it throws out a quarter of everything a false sync
                // manages to push through the Golay decoder.
                let lich_bytes = fec::pack(&lich_bits);
                if lich_bytes[5] >> 5 > 5 {
                    return None;
                }
                let (bits, ber) = fec::viterbi(&self.typed[96..], &fec::P2, 144);
                if ber > self.cfg.max_ber {
                    return None;
                }
                let contents = fec::pack(&bits);
                let number = u16::from_be_bytes([contents[0], contents[1]]);
                let mut lich = [0u8; 6];
                lich.copy_from_slice(&lich_bytes);
                let mut payload = [0u8; 16];
                payload.copy_from_slice(&contents[2..18]);
                (
                    Body::Stream {
                        lich,
                        lich_errors,
                        number: number & 0x7FFF,
                        last: number & 0x8000 != 0,
                        payload,
                    },
                    ber,
                )
            }
            Kind::Packet => {
                let (bits, ber) = fec::viterbi(&self.typed, &fec::P3, 206);
                if ber > self.cfg.max_unchecked_ber {
                    return None;
                }
                let bytes = fec::pack(&bits[..200]);
                let mut data = [0u8; 25];
                data.copy_from_slice(&bytes);
                let counter = bits[201..206].iter().fold(0u8, |a, &b| a << 1 | b);
                (Body::Packet { data, eof: bits[200] == 1, counter }, ber)
            }
            Kind::Bert => {
                let (_, ber) = fec::viterbi(&self.typed, &fec::P2, 197);
                if ber > self.cfg.max_unchecked_ber {
                    return None;
                }
                (Body::Bert, ber)
            }
        };
        Some(body)
    }
}

/// A sync word as the eight symbols it is transmitted as.
pub fn sync_symbols(sync: u16) -> [f32; SYNC_SYMBOLS] {
    let mut out = [0.0; SYNC_SYMBOLS];
    for (k, v) in out.iter_mut().enumerate() {
        *v = symbol((sync >> (14 - 2 * k) & 3) as u8);
    }
    out
}

/// The dibit to symbol map. Bit 1 of the dibit is the sign and bit 0 says
/// whether the level is an outer one, which is the property the soft slicer
/// in [`M17Demod::decode`] relies on.
pub fn symbol(dibit: u8) -> f32 {
    match dibit & 3 {
        0b01 => 3.0,
        0b00 => 1.0,
        0b10 => -1.0,
        _ => -3.0,
    }
}

fn interp(x: &[f32], pos: f64) -> f32 {
    if pos <= 0.0 || x.is_empty() {
        return x.first().copied().unwrap_or(0.0);
    }
    let i = pos.floor() as usize;
    if i + 1 >= x.len() {
        return x[x.len() - 1];
    }
    let f = (pos - i as f64) as f32;
    x[i] * (1.0 - f) + x[i + 1] * f
}

/// Root raised cosine taps, the matched filter for the shaping M17 transmits
/// with: alpha 0.5, spanning `span` symbols at `sps` samples per symbol.
pub fn rrc_taps(sps: f64, alpha: f64, span: usize) -> Vec<f32> {
    let n = ((span as f64 * sps).round() as usize) | 1;
    let mid = (n - 1) as f64 / 2.0;
    let mut h = Vec::with_capacity(n);
    for i in 0..n {
        let t = (i as f64 - mid) / sps;
        let v = if t.abs() < 1e-9 {
            1.0 + alpha * (4.0 / std::f64::consts::PI - 1.0)
        } else if (t.abs() - 1.0 / (4.0 * alpha)).abs() < 1e-9 {
            // The removable singularity at t = T/4a, where the denominator
            // below is zero and the limit is finite.
            let q = std::f64::consts::PI / (4.0 * alpha);
            alpha / 2.0f64.sqrt()
                * ((1.0 + 2.0 / std::f64::consts::PI) * q.sin()
                    + (1.0 - 2.0 / std::f64::consts::PI) * q.cos())
        } else {
            let pt = std::f64::consts::PI * t;
            let num = (pt * (1.0 - alpha)).sin() + 4.0 * alpha * t * (pt * (1.0 + alpha)).cos();
            let den = pt * (1.0 - (4.0 * alpha * t).powi(2));
            num / den
        };
        h.push(v as f32);
    }
    let dc: f32 = h.iter().sum();
    if dc.abs() > 1e-20 {
        for v in &mut h {
            *v /= dc;
        }
    }
    h
}

/// Build a frame's 192 symbols: sync word, then the payload with its error
/// correction, interleaving and randomising applied.
///
/// The transmit side of everything above, and for now the only way to
/// exercise it: a decoder tested against frames it built itself proves the
/// two directions agree, which is not the same as proving either matches
/// another implementation. That is what a recording off a real radio is for.
pub fn frame_symbols(kind: Kind, contents: &[u8], number: u16, lich: &[u8; 6]) -> Vec<f32> {
    let mut bits = Vec::new();
    let mut type3 = Vec::new();
    let mut lich_bits: Vec<u8> = Vec::new();
    let sync = match kind {
        Kind::Lsf => {
            fec::unpack(contents, &mut bits);
            fec::conv_encode(&bits, &fec::P1, &mut type3);
            SYNC_LSF
        }
        Kind::Stream => {
            let mut all = number.to_be_bytes().to_vec();
            all.extend_from_slice(contents);
            fec::unpack(&all, &mut bits);
            fec::conv_encode(&bits, &fec::P2, &mut type3);
            let mut chunk = Vec::new();
            fec::unpack(lich, &mut chunk);
            for c in chunk.chunks(12) {
                let data = c.iter().fold(0u16, |a, &b| a << 1 | u16::from(b));
                let word = fec::golay_encode(data);
                for i in (0..24).rev() {
                    lich_bits.push((word >> i & 1) as u8);
                }
            }
            SYNC_STREAM
        }
        Kind::Packet => {
            fec::unpack(contents, &mut bits);
            bits.truncate(206);
            fec::conv_encode(&bits, &fec::P3, &mut type3);
            SYNC_PACKET
        }
        Kind::Bert => {
            fec::unpack(contents, &mut bits);
            bits.truncate(197);
            fec::conv_encode(&bits, &fec::P2, &mut type3);
            SYNC_BERT
        }
    };

    let mut payload: Vec<u8> = lich_bits;
    payload.extend_from_slice(&type3);
    assert_eq!(payload.len(), PAYLOAD_BITS, "a frame is 368 payload bits");

    // Interleave and randomise, which on the receiving side are undone as
    // soft values; here the bits are still hard.
    let mut type4 = vec![0u8; PAYLOAD_BITS];
    for (i, &j) in fec::INTERLEAVE.iter().enumerate() {
        type4[i] = payload[j as usize];
    }
    for (i, b) in type4.iter_mut().enumerate() {
        *b ^= fec::RANDOMIZER[(i / 8) % fec::RANDOMIZER.len()] >> (7 - i % 8) & 1;
    }

    let mut out = sync_symbols(sync).to_vec();
    for d in type4.chunks_exact(2) {
        out.push(symbol(d[0] << 1 | d[1]));
    }
    out
}

/// The preamble every transmission opens with: alternating outer symbols.
pub fn preamble_symbols() -> Vec<f32> {
    (0..SYMBOLS_PER_FRAME).map(|i| if i % 2 == 0 { 3.0 } else { -3.0 }).collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    const RATE: f64 = 48_000.0;

    fn lsf(dst: u64, src: u64, kind: u16) -> [u8; 30] {
        let mut f = [0u8; 30];
        f[..6].copy_from_slice(&dst.to_be_bytes()[2..]);
        f[6..12].copy_from_slice(&src.to_be_bytes()[2..]);
        f[12..14].copy_from_slice(&kind.to_be_bytes());
        let crc = fec::crc16(&f[..28]);
        f[28..].copy_from_slice(&crc.to_be_bytes());
        f
    }

    /// Symbols to a discriminator-shaped waveform: upsample, shape with the
    /// same root raised cosine the receiver matches against, and scale so the
    /// outer symbols land where an `FmDemod` built for 2.4 kHz would put
    /// them.
    fn modulate(symbols: &[f32], sps: f64, gain: f32, offset: f32, noise: f32) -> Vec<f32> {
        let taps = rrc_taps(sps, 0.5, 8);
        let n = ((symbols.len() as f64 + 8.0) * sps) as usize;
        let mut imp = vec![0.0f32; n];
        for (i, &s) in symbols.iter().enumerate() {
            let at = (i as f64 * sps).round() as usize;
            if at < n {
                imp[at] += s * sps as f32;
            }
        }
        let mut seed = 4242u64;
        let mut out = Vec::with_capacity(n);
        for i in 0..n {
            let mut acc = 0.0;
            for (k, &t) in taps.iter().enumerate() {
                if i >= k {
                    acc += t * imp[i - k];
                }
            }
            seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1);
            let r = ((seed >> 33) as f32 / (1u64 << 30) as f32 - 1.0) * noise;
            out.push(acc * gain / 3.0 + offset + r);
        }
        out
    }

    fn run(audio: &[f32]) -> Vec<Frame> {
        let mut d = M17Demod::new(RATE, M17Config::default());
        let mut out = Vec::new();
        for chunk in audio.chunks(1000) {
            d.process(chunk, &mut out);
        }
        out
    }

    /// A whole transmission the way a radio sends one: preamble, link setup,
    /// then stream frames.
    fn transmission(frames: usize) -> Vec<f32> {
        let setup = lsf(0xFFFF_FFFF_FFFF, 0x0000_9FDD_51, 0x0005);
        let mut symbols = preamble_symbols();
        symbols.extend(frame_symbols(Kind::Lsf, &setup, 0, &[0; 6]));
        for n in 0..frames {
            let cnt = (n % 6) as u8;
            let mut lich = [0u8; 6];
            lich[..5].copy_from_slice(&setup[cnt as usize * 5..cnt as usize * 5 + 5]);
            lich[5] = cnt << 5;
            let payload: [u8; 16] = std::array::from_fn(|i| (n * 16 + i) as u8);
            symbols.extend(frame_symbols(Kind::Stream, &payload, n as u16, &lich));
        }
        symbols
    }

    #[test]
    fn a_transmission_decodes_to_its_link_setup_and_stream() {
        let audio = modulate(&transmission(12), RATE / BAUD, 1.0, 0.0, 0.0);
        let frames = run(&audio);
        assert_eq!(frames.len(), 13, "expected the link setup and twelve stream frames");
        let Body::Lsf(got) = frames[0].body else { panic!("first frame is not a link setup") };
        assert_eq!(got, lsf(0xFFFF_FFFF_FFFF, 0x0000_9FDD_51, 0x0005));
        for (n, f) in frames[1..].iter().enumerate() {
            let Body::Stream { number, payload, lich_errors, .. } = f.body else {
                panic!("frame {n} is not a stream frame")
            };
            assert_eq!(number, n as u16);
            assert_eq!(payload[0], (n * 16) as u8);
            assert_eq!(lich_errors, 0);
            assert_eq!(f.ber, 0.0, "a clean frame should re-encode exactly");
        }
    }

    /// Which way up the discriminator's output arrives is the receiver's
    /// business, not the transmitter's, and the sync words cannot tell the
    /// two apart: inverted, a link setup frame correlates as a stream frame.
    /// Only the contents resolve it.
    #[test]
    fn an_inverted_signal_reads_the_same(
    ) {
        let symbols: Vec<f32> = transmission(6).iter().map(|s| -s).collect();
        let audio = modulate(&symbols, RATE / BAUD, 1.0, 0.0, 0.0);
        let frames = run(&audio);
        assert_eq!(frames.len(), 7);
        assert!(matches!(frames[0].body, Body::Lsf(_)), "polarity was not resolved");
        // The stream frames' sync words correlate positively against the link
        // setup template here, which is precisely the ambiguity the contents
        // had to settle.
        assert!(frames[1..].iter().all(|f| f.correlation > 0.0));
        assert!(frames[1..].iter().all(|f| matches!(f.body, Body::Stream { .. })));
    }

    #[test]
    fn a_tuning_offset_and_a_wrong_gain_do_not_matter() {
        // Half the expected deviation and a kilohertz of tuning error, which
        // between them move every symbol and the centre they are measured
        // from.
        let audio = modulate(&transmission(6), RATE / BAUD, 0.5, 0.42, 0.0);
        assert_eq!(run(&audio).len(), 7);
    }

    /// Noise as loud as the signal, which is past where the eye is open at
    /// all: every symbol decision is unreliable and the convolutional code is
    /// what is holding the frame together.
    #[test]
    fn frames_survive_noise() {
        let audio = modulate(&transmission(24), RATE / BAUD, 1.0, 0.0, 1.0);
        let frames = run(&audio);
        assert!(frames.len() >= 20, "only {} of 25 frames survived", frames.len());
        assert!(frames.iter().any(|f| matches!(f.body, Body::Lsf(_))));
        assert!(frames.iter().all(|f| f.ber < 0.06), "a surviving frame read worse than noise does");
    }

    #[test]
    fn noise_alone_produces_nothing() {
        let audio = modulate(&[], RATE / BAUD, 0.0, 0.0, 1.0);
        let noise: Vec<f32> = (0..RATE as usize * 2)
            .scan(7u64, |s, _| {
                *s = s.wrapping_mul(6364136223846793005).wrapping_add(1);
                Some((*s >> 33) as f32 / (1u64 << 30) as f32 - 1.0)
            })
            .collect();
        assert!(run(&audio).is_empty());
        let frames = run(&noise);
        assert!(frames.is_empty(), "noise decoded to {} frames", frames.len());
    }

    #[test]
    fn a_packet_frame_carries_its_counter() {
        let mut contents = [0u8; 26];
        for (i, b) in contents[..25].iter_mut().enumerate() {
            *b = b'A' + (i as u8 % 26);
        }
        // End of frame, and seventeen valid bytes in it.
        contents[25] = 0x80 | 17 << 2;
        let mut symbols = preamble_symbols();
        symbols.extend(frame_symbols(Kind::Packet, &contents, 0, &[0; 6]));
        let frames = run(&modulate(&symbols, RATE / BAUD, 1.0, 0.0, 0.0));
        assert_eq!(frames.len(), 1);
        let Body::Packet { data, eof, counter } = frames[0].body else {
            panic!("not a packet frame: {:?}", frames[0].body)
        };
        assert_eq!(&data[..3], b"ABC");
        assert!(eof);
        assert_eq!(counter, 17);
    }

    /// The frames libm17 builds from the same input, symbol for symbol.
    ///
    /// This is the test that matters, because everything the encoder and the
    /// decoder here share could be self-consistently wrong: a puncturing
    /// pattern read backwards, a Golay row order, an interleaver applied in
    /// the wrong direction. Each of those round-trips perfectly and puts a
    /// different signal on the air. The vectors below were generated by the
    /// M17 project's own C library, calling `gen_frame` for a link setup
    /// frame, a stream frame with LICH counter 2 and frame number 0x1234,
    /// and a packet frame, with symbols written as A=+3, B=+1, C=-1, D=-3.
    #[test]
    fn the_frames_match_the_reference_implementation() {
        const REF_LSF: &str = "AAAADDADDABCCDDADBCDBBBDBBCDDABBBCCBABDBCAADBBDBACBBDBBABBCCCACADBAAAACDCCBCACBAAADACADBABBBDCCAABCAACDDABAABBCBBDBBDACBAADCBABBDCBBAABBDCAACBBACBBCDBBDCCBCADDCCACADADADDBDBABCDBADCACDDCDCABBB";
        const REF_STREAM: &str = "DDDDAADAAACDCCAADCDBABACABAADDDACCBDDCDDBDCDCDADBBBDBBCBDABCCADDADACDDADABCACDDBDABBBBACCBABBADDBAAACAACCCCBCABBBACDACBCCBDACAAABBBAABBBDADDDDBBCDBABDDBDDBAADBDADDCADABABBBDBBCACBABCDADCCADBCA";
        const REF_PACKET: &str = "ADAADDDDCBADCCBBBADDADBDBADDCCBBCBDBCCDDAADBADCBBDDBACABCCABBDABBCABACDBDBDAADCCADABBABCADCCDDCBADCAABAADDCBBBBBDBAADDABCBDBCBABACBBDCBCCBABBDDBCDDCBDACCBDABBABCDBADAACCAAACDCCDDBCCAACDBDCDDDA";

        fn spell(symbols: &[f32]) -> String {
            symbols
                .iter()
                .map(|&s| match s as i32 {
                    3 => 'A',
                    1 => 'B',
                    -1 => 'C',
                    _ => 'D',
                })
                .collect()
        }

        // DST "M17-M17 C", SRC "AB1CD", a voice stream type, and a metadata
        // field counting up so that every LICH chunk differs.
        let mut setup = [0u8; 30];
        setup[..6].copy_from_slice(&[0x12, 0x02, 0xBC, 0xCE, 0xCA, 0xED]);
        setup[6..12].copy_from_slice(&[0x00, 0x00, 0x00, 0x9F, 0xDD, 0x51]);
        setup[12..14].copy_from_slice(&[0x00, 0x05]);
        for (i, b) in setup[14..28].iter_mut().enumerate() {
            *b = (i * 7) as u8;
        }
        let crc = fec::crc16(&setup[..28]);
        setup[28..].copy_from_slice(&crc.to_be_bytes());
        assert_eq!(crc, 0x8DCD, "the reference frame's CRC came out differently");

        assert_eq!(spell(&frame_symbols(Kind::Lsf, &setup, 0, &[0; 6])), REF_LSF);

        let payload: [u8; 16] = std::array::from_fn(|i| (i * 3 + 1) as u8);
        let lich = [0xDD, 0x51, 0x00, 0x05, 0x00, 0x40];
        assert_eq!(spell(&frame_symbols(Kind::Stream, &payload, 0x1234, &lich)), REF_STREAM);

        let mut packet = [0u8; 26];
        for (i, b) in packet[..25].iter_mut().enumerate() {
            *b = b'A' + (i as u8 % 26);
        }
        packet[25] = 0x80 | 17 << 2;
        assert_eq!(spell(&frame_symbols(Kind::Packet, &packet, 0, &[0; 6])), REF_PACKET);
    }

    #[test]
    fn the_sync_words_are_the_symbols_the_specification_prints() {
        assert_eq!(sync_symbols(SYNC_LSF), [3.0, 3.0, 3.0, 3.0, -3.0, -3.0, 3.0, -3.0]);
        assert_eq!(sync_symbols(SYNC_STREAM), [-3.0, -3.0, -3.0, -3.0, 3.0, 3.0, -3.0, 3.0]);
        assert_eq!(sync_symbols(SYNC_PACKET), [3.0, -3.0, 3.0, 3.0, -3.0, -3.0, -3.0, -3.0]);
        assert_eq!(sync_symbols(SYNC_BERT), [-3.0, 3.0, -3.0, -3.0, 3.0, 3.0, 3.0, 3.0]);
    }
}
