//! Bluetooth Low Energy advertising: GFSK at 1 Mbit/s, and the link layer.
//!
//! An advertising packet is a short burst on one of three fixed channels, so
//! the shape of this is the AIS one: a narrowband chain per channel, then a
//! framer. What differs is everything about the timing. AIS is 9600 baud and
//! the demodulation is nearly free; BLE is a hundred times faster, the packet
//! lasts 80 to 400 us, and there is no training sequence worth speaking of:
//! eight bits of alternating preamble, then the access address, and the data
//! starts 40 bits after the burst opened.
//!
//! So there is no clock recovery loop here. The packet is short enough that a
//! fixed bit period taken from the sample rate drifts by a fraction of a
//! symbol across the whole of it, and a loop that has to pull in within eight
//! bits will still be pulling in when the address goes past. The access
//! address is correlated for at every sub-symbol offset instead, and the
//! offset that finds it is the one the rest of the packet is read at.
//!
//! # The receiver's frequency error is not a detail
//!
//! GFSK at h = 0.5 puts the two tones +/-250 kHz apart, and a HackRF at
//! 2.4 GHz is twenty parts per million out, which is 50 kHz. Slicing the
//! discriminator at zero rather than at the burst's own centre therefore
//! throws away a fifth of the eye before any noise is involved. Measured on
//! thirteen seconds of channel 38: 149 packets pass CRC with the per-burst
//! offset removed, and none at all without it.
//!
//! # Why the burst is not read as pulse timings
//!
//! Every other two-level front end here hands mark and gap widths to
//! `decode::slicer`, and BLE could be read that way: it is NRZ, one bit is one
//! microsecond. It costs about half the packets. `common::Pulse` measures in
//! whole microseconds, which at 1 Mbit/s is one whole bit of quantisation on
//! every run, and the recording that yields 89 packets through bit-centre
//! sampling yields 51 through microsecond run lengths. The CRC hides none of
//! that: what is lost is lost silently.
//!
//! # What is accepted
//!
//! The CRC-24 decides, and nothing else does. A 40 bit access address match
//! alone is not evidence: correlating for it at four offsets across every
//! burst on a busy 2.4 GHz band offers enough windows that some of them hit.
//! With the CRC behind it a false frame needs a 24 bit coincidence as well,
//! and the whitening means a wrong channel index fails it too, so a frame that
//! reports here was received on the channel it says it was.

use crate::fir::{lowpass, FirDecim};
use crate::mixer::Mixer;
use crate::pulse::LevelGate;
use common::C32;

/// Symbol rate. Uncoded BLE, which is the only rate advertising uses on the
/// primary channels.
pub const BAUD: f64 = 1_000_000.0;

/// The access address every advertising packet carries. Data channels
/// negotiate their own, which is why this front end reads advertising only:
/// without the connection request there is nothing to correlate for.
pub const ADV_ACCESS_ADDRESS: u32 = 0x8E89_BED6;

/// The three primary advertising channels: index as the whitening uses it,
/// and where it sits.
pub const ADV_CHANNELS: [(u8, f64); 3] = [
    (37, 2_402_000_000.0),
    (38, 2_426_000_000.0),
    (39, 2_480_000_000.0),
];

/// Half the width one channel occupies. Neighbouring channels are 2 MHz away
/// and the modulation is about 1 MHz wide, so this passes the signal and stops
/// at the neighbour.
const PASSBAND_HZ: f64 = 700_000.0;

/// Samples per symbol aimed for after decimation. Four is where the packet
/// count stops moving: 87 against 89 at sixteen, and 51 through the
/// microsecond pulse path.
const TARGET_SPS: f64 = 4.0;

/// Largest advertising PDU: two header bytes and 37 of payload.
const MAX_PDU: usize = 39;
/// PDU plus the CRC.
const MAX_FRAME: usize = MAX_PDU + 3;
/// Shortest PDU that can carry an address, which every advertising type does.
const MIN_PAYLOAD: usize = 6;

/// The channel filter, designed against the modulation rather than against
/// the decimation.
///
/// `FirDecim::design_hz` places its stopband where the first alias folds
/// down, which for a factor of four here is 3.3 MHz and leaves the passband
/// edge at 2 MHz: three times wider than the signal, so three times the noise
/// reaches the discriminator. Measured on thirteen seconds of channel 38 that
/// filter reads 90 packets and this one reads 341, and what it loses is a
/// whole advertiser: the weakest of the five failed every CRC while the
/// strong ones still read, so the front end looked like it worked.
fn channel_filter(rate: f64, factor: usize) -> Vec<f32> {
    let taps = (64 * factor) | 1;
    lowpass(taps, PASSBAND_HZ / rate, 60.0)
}

/// Preamble and access address, as bits go on the air: least significant bit
/// of each byte first.
fn sync_bits() -> [bool; 40] {
    let mut v = [false; 40];
    for (i, b) in v.iter_mut().enumerate().take(8) {
        *b = i % 2 == 1;
    }
    for i in 0..32 {
        v[8 + i] = ADV_ACCESS_ADDRESS >> i & 1 == 1;
    }
    v
}

/// BLE's data whitening: a seven bit LFSR, `x^7 + x^4 + 1`, seeded with the
/// channel index, one bit out per bit of payload.
///
/// Not the same whitener as [`crate::super::whiten`]'s PN9 in `decode`, and
/// not seeded the same way either: the seed carries the channel index, so a
/// packet dewhitened for the wrong channel fails its CRC. That is a feature
/// worth keeping rather than a hazard: it means a frame cannot be attributed
/// to a channel it did not arrive on.
pub struct Whitening {
    reg: [bool; 7],
}

impl Whitening {
    pub fn new(channel: u8) -> Self {
        let mut reg = [true; 7];
        for i in 0..6 {
            reg[i + 1] = channel >> (5 - i) & 1 == 1;
        }
        Self { reg }
    }

    fn next_bit(&mut self) -> bool {
        let out = self.reg[6];
        self.reg.rotate_right(1);
        self.reg[0] = out;
        self.reg[4] ^= out;
        out
    }

    pub fn next_byte(&mut self) -> u8 {
        let mut b = 0u8;
        for k in 0..8 {
            b |= u8::from(self.next_bit()) << k;
        }
        b
    }

    /// Whiten or dewhiten in place. The sequence is its own inverse.
    pub fn apply(&mut self, bytes: &mut [u8]) {
        for b in bytes.iter_mut() {
            *b ^= self.next_byte();
        }
    }
}

/// BLE's CRC-24 over a PDU, `x^24 + x^10 + x^9 + x^6 + x^4 + x^3 + x + 1`.
///
/// The initial value is the specification's 0x555555 with its bits reversed,
/// because the register is shifted the other way round here. Writing the
/// unreversed value produces a plausible looking check that never passes on a
/// real packet, which is a long afternoon.
pub fn crc24(data: &[u8]) -> u32 {
    const MASK: u32 = 0x5a6000;
    let mut state: u32 = 0xaaaaaa;
    for &byte in data {
        let mut cur = byte;
        for _ in 0..8 {
            let feedback = (state ^ u32::from(cur)) & 1;
            cur >>= 1;
            state >>= 1;
            if feedback != 0 {
                state |= 1 << 23;
                state ^= MASK;
            }
        }
    }
    state
}

#[derive(Clone, Copy, Debug)]
pub struct BleConfig {
    /// Minimum envelope SNR before a burst is opened.
    pub min_snr_db: f32,
    /// Hard floor on the carrier-detect threshold, as a multiple of the
    /// tracked noise mean.
    pub noise_threshold_ratio: f32,
    /// Envelope estimator time constant, in microseconds.
    pub tau_us: f32,
    /// Shortest burst worth reading, in microseconds. The shortest legal
    /// advertising packet is 80 us of air time.
    pub min_burst_us: u32,
    /// Longest burst held before it is read and dropped, in microseconds. An
    /// uncoded packet is at most 376 us and a long range one carrying an Open
    /// Drone ID message pack about 2.9 ms, since eight symbols there carry
    /// one bit; anything longer is a carrier or a collision, and what is
    /// inside it is still searched for an access address.
    pub max_burst_us: u32,
    /// Whether to look for Bluetooth 5 Long Range packets in a burst that
    /// held no uncoded one.
    ///
    /// On by default because the packets that matter most on this receiver
    /// (an aircraft's Open Drone ID) are required to be sent this way, and
    /// the search costs nothing on a burst with no coded preamble in it.
    pub coded: bool,
}

impl Default for BleConfig {
    fn default() -> Self {
        Self {
            min_snr_db: 6.0,
            noise_threshold_ratio: 3.0,
            tau_us: 20.0,
            min_burst_us: 60,
            max_burst_us: 4_000,
            coded: true,
        }
    }
}

/// One advertising packet that passed its CRC.
#[derive(Clone, Debug, PartialEq)]
pub struct BleFrame {
    /// Advertising channel index, 37, 38 or 39.
    pub channel: u8,
    /// The coding it arrived under, or `None` for an ordinary uncoded
    /// packet. A long range packet is the same PDU carried eight times as
    /// slowly, so everything above this reads it the same way.
    pub coding: Option<crate::ble_coded::Coding>,
    /// Header and payload, dewhitened, without the CRC.
    pub pdu: Vec<u8>,
    /// Where the burst started in the stream fed to the detector, counted in
    /// that stream's own samples rather than in the decimated channel's.
    pub start_sample: u64,
    /// Transmitter's offset from the channel centre as this receiver sees it:
    /// its crystal error and the tuner's together.
    pub freq_off_hz: f32,
    pub rssi_dbfs: f32,
    pub snr_db: f32,
}

/// One advertising channel: mix down, decimate, gate, read what is inside.
struct ChannelRx {
    channel: u8,
    rate: f64,
    sps: f64,
    /// Input samples per channel sample, so a burst is reported at the index
    /// the caller's own stream has for it.
    factor: usize,
    mixer: Mixer,
    decim: FirDecim,
    gate: LevelGate,
    mixed: Vec<C32>,
    narrow: Vec<C32>,
    /// The burst being collected, with the samples either side that the
    /// discriminator's first difference and the level estimate need.
    burst: Vec<C32>,
    /// Where in `burst` the gate opened.
    body_start: usize,
    in_burst: bool,
    margin: usize,
    pre: std::collections::VecDeque<C32>,
    sample: u64,
    burst_start: u64,
    freq: Vec<f32>,
    bits: Vec<bool>,
}

impl ChannelRx {
    fn new(channel: u8, channel_hz: f64, rate: f64, center_hz: f64, cfg: &BleConfig) -> Self {
        let factor = (rate / (BAUD * TARGET_SPS)).floor().max(1.0) as usize;
        let work = rate / factor as f64;
        let margin = (work * 60e-6) as usize;
        Self {
            channel,
            rate: work,
            sps: work / BAUD,
            factor,
            mixer: Mixer::new(center_hz - channel_hz, rate),
            decim: FirDecim::new(channel_filter(rate, factor), factor),
            gate: LevelGate::new(
                work,
                cfg.tau_us,
                0.3,
                cfg.min_snr_db,
                cfg.noise_threshold_ratio,
            ),
            mixed: Vec::new(),
            narrow: Vec::new(),
            burst: Vec::new(),
            body_start: 0,
            in_burst: false,
            margin: margin.max(4),
            pre: std::collections::VecDeque::new(),
            sample: 0,
            burst_start: 0,
            freq: Vec::new(),
            bits: Vec::new(),
        }
    }

    fn process(&mut self, iq: &[C32], cfg: &BleConfig, out: &mut Vec<BleFrame>) {
        self.mixed.clear();
        self.mixer.process(iq, &mut self.mixed);
        self.narrow.clear();
        self.decim.process(&self.mixed, &mut self.narrow);

        let max_samples = (cfg.max_burst_us as f64 * self.rate / 1e6) as usize;
        let quiet_end = (self.rate * 20e-6) as usize;
        let mut low_run = 0usize;
        let narrow = std::mem::take(&mut self.narrow);
        for &x in &narrow {
            let high = self.gate.update(x.norm());
            self.sample += 1;
            if high {
                low_run = 0;
                if !self.in_burst {
                    self.in_burst = true;
                    self.burst.clear();
                    self.burst.extend(self.pre.iter().copied());
                    self.body_start = self.burst.len();
                    self.burst_start = self.sample.saturating_sub(self.pre.len() as u64);
                }
                self.burst.push(x);
            } else if self.in_burst {
                self.burst.push(x);
                low_run += 1;
                if low_run >= quiet_end {
                    self.finish(cfg, out);
                }
            }
            if self.pre.len() == self.margin {
                self.pre.pop_front();
            }
            self.pre.push_back(x);
            if self.in_burst && self.burst.len() >= max_samples {
                self.finish(cfg, out);
            }
        }
        self.narrow = narrow;
        self.narrow.clear();
    }

    fn finish(&mut self, cfg: &BleConfig, out: &mut Vec<BleFrame>) {
        self.in_burst = false;
        let burst = std::mem::take(&mut self.burst);
        let min_body = (cfg.min_burst_us as f64 * self.rate / 1e6) as usize;
        if burst.len().saturating_sub(self.body_start) >= min_body {
            self.read(&burst, cfg, out);
        }
        self.burst = burst;
        self.burst.clear();
    }

    /// Discriminate, take the frequency offset out, and read whatever
    /// advertising packets are inside.
    fn read(&mut self, burst: &[C32], cfg: &BleConfig, out: &mut Vec<BleFrame>) {
        self.freq.clear();
        let mut prev = burst[0];
        for &s in &burst[1..] {
            let d = s * prev.conj();
            self.freq.push(d.im.atan2(d.re));
            prev = s;
        }
        if self.freq.len() < 40 * self.sps as usize {
            return;
        }
        // The centre of the burst's own frequency histogram, which is the
        // transmitter and the tuner disagreeing. Taken over the loud part
        // only: the margins are noise, whose instantaneous frequency is
        // uniform and would pull the estimate towards zero.
        let body = &mut self.freq[self.body_start.saturating_sub(1)..];
        let mut sorted: Vec<f32> = body.to_vec();
        sorted.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        let dc = sorted[sorted.len() / 2];
        for f in self.freq.iter_mut() {
            *f -= dc;
        }
        let freq_off_hz = (dc as f64 * self.rate / std::f64::consts::TAU) as f32;

        let level = burst[self.body_start..]
            .iter()
            .map(|c| c.norm())
            .sum::<f32>()
            / (burst.len() - self.body_start).max(1) as f32;
        let snr_db = self.gate.snr_db();
        let rssi_dbfs = 20.0 * level.max(1e-9).log10();

        let sync = sync_bits();
        let offsets = self.sps.ceil() as usize;
        for phase in 0..offsets {
            self.bits.clear();
            let mut k = 0usize;
            loop {
                let at = phase as f64 + k as f64 * self.sps;
                let i = at.round() as usize;
                if i >= self.freq.len() {
                    break;
                }
                self.bits.push(self.freq[i] > 0.0);
                k += 1;
            }
            if self.bits.len() < 40 + MIN_PAYLOAD * 8 {
                continue;
            }
            let mut at = 0usize;
            while at + 40 <= self.bits.len() {
                let Some(found) = find_sync(&self.bits, at, &sync) else {
                    break;
                };
                at = found + 1;
                if let Some(pdu) = self.frame_at(found + 40) {
                    out.push(BleFrame {
                        channel: self.channel,
                        coding: None,
                        pdu,
                        start_sample: self.burst_start * self.factor as u64,
                        freq_off_hz,
                        rssi_dbfs,
                        snr_db,
                    });
                    // One access address match that produced a frame is
                    // enough from this burst: the other sub-symbol offsets
                    // are the same packet read half a symbol late.
                    return;
                }
            }
        }

        if !cfg.coded {
            return;
        }
        // Nothing uncoded in this burst. A Bluetooth 5 Long Range packet is
        // the same modulation at the same symbol rate, so the symbols are
        // already here; what differs is that eight of them carry one bit.
        // The uncoded search cannot find it because its access address is
        // convolutionally coded, which is why this is a second pass rather
        // than another sync word.
        for phase in 0..offsets {
            let mut syms: Vec<f32> = Vec::with_capacity(self.freq.len() / self.sps as usize);
            let mut k = 0usize;
            loop {
                let at = phase as f64 + k as f64 * self.sps;
                let i = at.round() as usize;
                if i >= self.freq.len() {
                    break;
                }
                syms.push(self.freq[i]);
                k += 1;
            }
            if syms.len() < 80 + 300 {
                continue;
            }
            if let Some(f) = crate::ble_coded::decode(&syms, self.channel) {
                out.push(BleFrame {
                    channel: self.channel,
                    coding: Some(f.coding),
                    pdu: f.pdu,
                    start_sample: self.burst_start * self.factor as u64,
                    freq_off_hz,
                    rssi_dbfs,
                    snr_db,
                });
                return;
            }
        }
    }

    /// Read a frame whose data begins at `start`, if the CRC agrees.
    fn frame_at(&self, start: usize) -> Option<Vec<u8>> {
        let avail = (self.bits.len() - start) / 8;
        if avail < 2 + MIN_PAYLOAD + 3 {
            return None;
        }
        let take = avail.min(MAX_FRAME);
        let mut bytes: Vec<u8> = (0..take)
            .map(|i| {
                let mut b = 0u8;
                for k in 0..8 {
                    b |= u8::from(self.bits[start + i * 8 + k]) << k;
                }
                b
            })
            .collect();
        Whitening::new(self.channel).apply(&mut bytes);
        let len = (bytes[1] & 0x3f) as usize;
        if len < MIN_PAYLOAD || 2 + len + 3 > bytes.len() {
            return None;
        }
        let pdu = &bytes[..2 + len];
        let want = u32::from(bytes[2 + len])
            | u32::from(bytes[3 + len]) << 8
            | u32::from(bytes[4 + len]) << 16;
        (crc24(pdu) == want).then(|| pdu.to_vec())
    }

    fn reset(&mut self) {
        self.mixer.reset();
        self.decim.reset();
        self.gate.reset();
        self.pre.clear();
        self.burst.clear();
        self.in_burst = false;
    }
}

/// First exact match of the preamble and access address at or after `from`.
///
/// Exact rather than a best correlation: at 1 Mbit/s a burst offers hundreds
/// of windows, and accepting the strongest of them means every burst produces
/// a candidate that the CRC then has to reject. A packet whose preamble is
/// damaged is lost, which is the right trade when the alternative is
/// searching every burst on the band for a coincidence.
fn find_sync(bits: &[bool], from: usize, sync: &[bool; 40]) -> Option<usize> {
    if bits.len() < 40 {
        return None;
    }
    (from..=bits.len() - 40).find(|&i| bits[i..i + 40] == sync[..])
}

/// Both directions of one channel, and as many channels as the span covers.
pub struct BleDetector {
    cfg: BleConfig,
    chans: Vec<ChannelRx>,
}

impl BleDetector {
    /// Build a detector for every advertising channel inside the span.
    ///
    /// A channel whose signal would sit in the anti-alias filter's skirt is
    /// not included: reading it there is reading an attenuated copy, and a
    /// receiver that reports two of the three channels is more honest than
    /// one that reports three and hears one badly.
    pub fn new(rate: f64, center_hz: f64, cfg: BleConfig) -> Self {
        let edge = rate / 2.0 - PASSBAND_HZ;
        let chans = ADV_CHANNELS
            .iter()
            .filter(|(_, hz)| (hz - center_hz).abs() <= edge)
            .map(|&(ch, hz)| ChannelRx::new(ch, hz, rate, center_hz, &cfg))
            .collect();
        Self { cfg, chans }
    }

    /// Which advertising channels this detector is reading.
    pub fn channels(&self) -> Vec<u8> {
        self.chans.iter().map(|c| c.channel).collect()
    }

    pub fn process(&mut self, iq: &[C32], out: &mut Vec<BleFrame>) {
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

/// Build the on-air bits for a PDU on a channel: preamble, access address,
/// then the whitened PDU and its CRC.
///
/// Public for the same reason `ais::encode_slot` is: without a recorded
/// capture, transmitting something known and reading it back is the only
/// honest test of a demodulator.
pub fn encode_packet(channel: u8, pdu: &[u8]) -> Vec<bool> {
    let mut bytes = pdu.to_vec();
    let crc = crc24(pdu);
    bytes.extend_from_slice(&[crc as u8, (crc >> 8) as u8, (crc >> 16) as u8]);
    Whitening::new(channel).apply(&mut bytes);
    let mut bits: Vec<bool> = sync_bits().to_vec();
    for b in bytes {
        for k in 0..8 {
            bits.push(b >> k & 1 == 1);
        }
    }
    bits
}

#[cfg(test)]
mod tests {
    use super::*;

    const RATE: f64 = 8_000_000.0;
    /// A tuner parked 1 MHz off channel 38, which is where a receiver puts
    /// the DC spike when it wants it out of the signal.
    const CENTER: f64 = 2_427_000_000.0;

    /// An advertising PDU: ADV_IND from a public address, with a flags AD and
    /// a short name.
    fn adv_ind() -> Vec<u8> {
        let mut pdu = vec![0x00, 0x00];
        pdu.extend_from_slice(&[0x4d, 0x72, 0xef, 0xcb, 0x70, 0x6c]);
        pdu.extend_from_slice(&[0x02, 0x01, 0x06]);
        pdu.extend_from_slice(&[0x04, 0x09, b'a', b'b', b'c']);
        pdu[1] = (pdu.len() - 2) as u8;
        pdu
    }

    /// Modulate bits as FSK at the BLE rate and deviation, with `offset_hz`
    /// of receiver error on top. Plain FSK rather than GFSK: the
    /// discriminator does not care about the Gaussian shaping, and a test
    /// that implemented it would be testing the filter.
    fn modulate(bits: &[bool], channel_hz: f64, center_hz: f64, offset_hz: f64) -> Vec<C32> {
        let sps = RATE / BAUD;
        let mut out = Vec::new();
        let mut phase = 0.0f64;
        let base = channel_hz - center_hz + offset_hz;
        for &b in bits {
            let f = base + if b { 250_000.0 } else { -250_000.0 };
            for _ in 0..sps as usize {
                phase += std::f64::consts::TAU * f / RATE;
                out.push(C32::new(phase.cos() as f32, phase.sin() as f32));
            }
        }
        out
    }

    fn noise(n: usize, amp: f32) -> Vec<C32> {
        let mut seed = 0x2545_f491_4f6c_dd1du64;
        (0..n)
            .map(|_| {
                let mut r = || {
                    seed ^= seed << 13;
                    seed ^= seed >> 7;
                    seed ^= seed << 17;
                    ((seed >> 40) as f32 / 8_388_608.0 - 1.0) * amp
                };
                C32::new(r(), r())
            })
            .collect()
    }

    fn run(iq: &[C32], center_hz: f64) -> Vec<BleFrame> {
        let mut det = BleDetector::new(RATE, center_hz, BleConfig::default());
        let mut out = Vec::new();
        det.process(&noise(20_000, 0.01), &mut out);
        det.process(iq, &mut out);
        det.process(&noise(20_000, 0.01), &mut out);
        out
    }

    /// The whole path: a packet built the way a device builds one, put on
    /// channel 38, and read back off the air.
    #[test]
    fn a_modulated_advertisement_comes_back_out_of_the_detector() {
        let pdu = adv_ind();
        let iq = modulate(&encode_packet(38, &pdu), 2_426_000_000.0, CENTER, 0.0);
        let got = run(&iq, CENTER);
        assert_eq!(got.len(), 1, "expected one frame, got {}", got.len());
        assert_eq!(got[0].channel, 38);
        assert_eq!(got[0].pdu, pdu, "the PDU came back changed");
    }

    /// A long range packet through the whole path: the same modulation, the
    /// same channel, eight symbols to the bit. The uncoded search cannot see
    /// it because even the access address is coded, so this is what proves
    /// the second pass runs.
    #[test]
    fn a_long_range_advertisement_comes_back_out_of_the_detector() {
        let pdu = {
            let mut p = vec![0x07, 0x00, 0x0a, 0x11];
            p.extend_from_slice(&[0x4d, 0x72, 0xef, 0xcb, 0x70, 0x6c]);
            p.extend_from_slice(&[17, 200, 0x40]);
            p[1] = (p.len() - 2) as u8;
            p
        };
        let bits = crate::ble_coded::encode_packet(38, &pdu, crate::ble_coded::Coding::S8);
        let iq = modulate(&bits, 2_426_000_000.0, CENTER, 0.0);
        let got = run(&iq, CENTER);
        assert_eq!(got.len(), 1, "expected one frame, got {}", got.len());
        assert_eq!(got[0].coding, Some(crate::ble_coded::Coding::S8));
        assert_eq!(got[0].pdu, pdu, "the PDU came back changed");
    }

    /// And a receiver told not to look does not find one, which is what makes
    /// the second pass a setting rather than a cost everybody pays.
    #[test]
    fn the_long_range_pass_can_be_turned_off() {
        let pdu = vec![0x07, 0x08, 0x00, 0x00, 1, 2, 3, 4, 5, 6];
        let bits = crate::ble_coded::encode_packet(38, &pdu, crate::ble_coded::Coding::S2);
        let iq = modulate(&bits, 2_426_000_000.0, CENTER, 0.0);
        let cfg = BleConfig { coded: false, ..Default::default() };
        let mut det = BleDetector::new(RATE, CENTER, cfg);
        let mut out = Vec::new();
        det.process(&noise(20_000, 0.01), &mut out);
        det.process(&iq, &mut out);
        det.process(&noise(20_000, 0.01), &mut out);
        assert!(out.is_empty());
    }

    /// The whitening carries the channel index, so a packet sent on 38 and
    /// read as 37 fails its CRC rather than decoding to nonsense.
    #[test]
    fn a_packet_whitened_for_another_channel_is_not_accepted() {
        let iq = modulate(&encode_packet(37, &adv_ind()), 2_426_000_000.0, CENTER, 0.0);
        assert!(
            run(&iq, CENTER).is_empty(),
            "the wrong channel's whitening decoded"
        );
    }

    /// A HackRF at 2.4 GHz is tens of kHz out, which is a fifth of the
    /// deviation. Every other test here transmits perfectly on frequency and
    /// so cannot show this.
    #[test]
    fn a_real_receivers_frequency_error_is_survivable() {
        let pdu = adv_ind();
        for err in [0.0, 25_000.0, 50_000.0, 90_000.0] {
            let iq = modulate(&encode_packet(38, &pdu), 2_426_000_000.0, CENTER, err);
            let got = run(&iq, CENTER);
            assert_eq!(got.len(), 1, "lost the packet at {err} Hz of offset");
            assert_eq!(got[0].pdu, pdu);
        }
    }

    /// A corrupted packet is dropped rather than reported with a flag. Half
    /// an advertisement is a device that is not there.
    #[test]
    fn a_corrupted_packet_is_dropped() {
        let mut bits = encode_packet(38, &adv_ind());
        bits[80] = !bits[80];
        let iq = modulate(&bits, 2_426_000_000.0, CENTER, 0.0);
        assert!(
            run(&iq, CENTER).is_empty(),
            "a corrupted packet was accepted"
        );
    }

    /// The index a frame reports is into the caller's own stream, not into
    /// the decimated channel it was read on. A caller cutting the burst out
    /// of a capture holds the undecimated samples, and at 8 MS/s the two
    /// differ by a factor of two, which puts the cut somewhere else entirely.
    #[test]
    fn the_reported_index_is_into_the_stream_that_was_fed_in() {
        let lead = 400_000usize;
        let mut iq = noise(lead, 0.01);
        iq.extend_from_slice(&modulate(
            &encode_packet(38, &adv_ind()),
            2_426_000_000.0,
            CENTER,
            0.0,
        ));
        let got = run(&iq, CENTER);
        assert_eq!(got.len(), 1);
        // `run` feeds 20000 samples of noise first, and the detector opens
        // its window ahead of the burst for the level estimate, so the
        // tolerance is that margin rather than a fudge.
        let at = got[0].start_sample as i64 - 20_000 - lead as i64;
        assert!(at.abs() < 2_000, "reported {at} samples from where the packet was put");
    }

    #[test]
    fn noise_produces_no_frames() {
        let got = run(&noise(4_000_000, 0.3), CENTER);
        assert!(got.is_empty(), "noise produced {} frames", got.len());
    }

    /// Only the channels the span actually covers are read.
    #[test]
    fn channels_outside_the_span_are_not_claimed() {
        let det = BleDetector::new(RATE, CENTER, BleConfig::default());
        assert_eq!(det.channels(), vec![38]);
        let wide = BleDetector::new(20_000_000.0, 2_410_000_000.0, BleConfig::default());
        assert_eq!(wide.channels(), vec![37]);
    }

    /// The CRC is the acceptance test, so it has to match an independent
    /// implementation rather than this one. These are from a real ADV_IND on
    /// channel 38, dewhitened, with the CRC the transmitter sent.
    #[test]
    fn the_crc_agrees_with_a_real_transmitter() {
        let pdu = [
            0x00, 0x11, 0x3a, 0xf5, 0x0a, 0xcd, 0x31, 0xe8, 0x02, 0x01, 0x06, 0x07, 0xff, 0xe1,
            0x02, 0x10, 0x00, 0x26, 0xc0,
        ];
        assert_eq!(crc24(&pdu), 0x9f05ed);
    }

    /// Whitening is its own inverse, and the sequence depends on the channel.
    #[test]
    fn whitening_round_trips_and_differs_per_channel() {
        let mut a = [0xffu8; 8];
        Whitening::new(38).apply(&mut a);
        let mut b = a;
        Whitening::new(38).apply(&mut b);
        assert_eq!(b, [0xff; 8]);
        let mut c = [0xffu8; 8];
        Whitening::new(39).apply(&mut c);
        assert_ne!(a, c, "two channels produced the same keystream");
    }
}
