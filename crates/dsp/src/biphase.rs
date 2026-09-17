//! Biphase-L data off a phase-modulated carrier.
//!
//! A transmitter that shifts its carrier a fixed angle either side of the
//! unmodulated phase, rather than keying it all the way round, leaves a
//! residual carrier standing in the middle of the two states: at the
//! 1.1 radian peak a 406 MHz distress beacon keys, cos(1.1) is 0.45 of the
//! amplitude, and that residual is the phase reference the demodulator
//! needs. So there is no squaring loop and no Costas loop here. The average
//! of the samples *is* the carrier, because biphase-L has no mean.
//!
//! Biphase-L (Manchester) keys two chips a bit with a transition in the
//! middle of every one, so the chip clock is in the signal whatever the data
//! does, and a zero-crossing loop over the chips holds it against the one
//! percent the standards allow the baud. What comes out is chips rather than
//! bits: which half of a pair starts a bit, and which way up the modulation
//! was, are questions a sync word answers, and the caller has the sync word.
//!
//! Parameterised by rate and baud, so any biphase-L carrier can be read
//! through it.

use common::C32;
use std::f64::consts::TAU;

/// Chips a bit. Biphase-L is one transition per bit whatever the data, which
/// is what makes the clock recoverable from a run of identical bits.
pub const CHIPS_PER_BIT: usize = 2;

/// Time constant of the carrier-offset estimate, in seconds.
///
/// Long, because it is averaging the modulation away as well as the noise: a
/// chip boundary throws a large step into the sample-to-sample rotation and
/// only the symmetry of the steps cancels it. Fifty milliseconds is a beacon
/// burst's unmodulated carrier and some, so the offset is settled before the
/// preamble arrives.
const OFFSET_TAU_S: f64 = 0.05;

/// Time constant of the phase reference, in seconds.
///
/// Short enough to follow what the offset estimate left behind, long enough
/// that the modulation averages out of it. Three milliseconds is two and a
/// half chips at 400 baud; at one millisecond the reference follows the data
/// instead of the carrier and the chips lose half their amplitude.
const REFERENCE_TAU_S: f64 = 0.003;

/// How hard a zero crossing pulls the chip clock, as a fraction of the
/// distance it was out.
///
/// A first order loop leaves a standing error of the baud error divided by
/// this, so 0.1 holds a one percent baud error a tenth of a chip off centre,
/// which is nothing against half a chip of margin. Measured on a synthesised
/// burst: 0.02 loses the last bits of a 144-bit message when the baud is a
/// percent fast, and 0.4 jitters on noise.
const CLOCK_GAIN: f64 = 0.1;

/// A phase-modulated carrier in, biphase-L chips out.
pub struct BiphaseDemod {
    rate: f64,
    /// Chip clock as a fraction of a chip per sample.
    step: f64,
    offset_alpha: f32,
    reference_alpha: f32,
    prev: C32,
    /// Average sample-to-sample rotation, which is the frequency offset.
    rotation: C32,
    /// Phase accumulated to undo that offset.
    derotate: f64,
    /// Average of the derotated samples, which is the carrier.
    reference: C32,
    /// Where in the chip the next sample falls.
    phase: f64,
    sum: f32,
    n: u32,
    prev_phase_error: f32,
}

impl BiphaseDemod {
    pub fn new(rate_hz: f64, baud: f64) -> Self {
        let rate = rate_hz.max(1.0);
        let chip_rate = baud * CHIPS_PER_BIT as f64;
        Self {
            rate,
            step: chip_rate / rate,
            offset_alpha: (1.0 / (rate * OFFSET_TAU_S)) as f32,
            reference_alpha: (1.0 / (rate * REFERENCE_TAU_S)) as f32,
            prev: C32::new(0.0, 0.0),
            rotation: C32::new(0.0, 0.0),
            derotate: 0.0,
            reference: C32::new(0.0, 0.0),
            phase: 0.0,
            sum: 0.0,
            n: 0,
            prev_phase_error: 0.0,
        }
    }

    /// Chips a second, which is what the caller times a message in.
    pub fn chip_rate(&self) -> f64 {
        self.step * self.rate
    }

    /// Read a block. Each chip comes out as the mean phase over it, in
    /// radians off the carrier, so its sign is the level and its size is how
    /// sure the demodulator is.
    pub fn process(&mut self, iq: &[C32], out: &mut Vec<f32>) {
        for &x in iq {
            let turn = x * self.prev.conj();
            self.prev = x;
            self.rotation += (turn - self.rotation) * self.offset_alpha;
            self.derotate -= f64::from(self.rotation.arg());
            if self.derotate.abs() > TAU {
                self.derotate = self.derotate.rem_euclid(TAU);
            }
            let y = x * C32::from_polar(1.0, self.derotate as f32);
            self.reference += (y - self.reference) * self.reference_alpha;

            let error = (y * self.reference.conj()).arg();
            self.sum += error;
            self.n += 1;

            // A crossing belongs on a chip boundary, which is where the
            // phase wraps, so how far it is from one is the clock error.
            if (error < 0.0) != (self.prev_phase_error < 0.0) {
                let off = match self.phase > 0.5 {
                    true => self.phase - 1.0,
                    false => self.phase,
                };
                self.phase -= CLOCK_GAIN * off;
            }
            self.prev_phase_error = error;

            self.phase += self.step;
            if self.phase >= 1.0 {
                self.phase -= 1.0;
                out.push(match self.n {
                    0 => 0.0,
                    n => self.sum / n as f32,
                });
                self.sum = 0.0;
                self.n = 0;
            }
        }
    }

    pub fn reset(&mut self) {
        self.prev = C32::new(0.0, 0.0);
        self.rotation = C32::new(0.0, 0.0);
        self.derotate = 0.0;
        self.reference = C32::new(0.0, 0.0);
        self.phase = 0.0;
        self.sum = 0.0;
        self.n = 0;
        self.prev_phase_error = 0.0;
    }
}

/// Key bits as biphase-L on a carrier, which is what a beacon transmits and
/// what a test needs to make one.
///
/// `deviation_rad` is the peak phase shift either side of the unmodulated
/// carrier, `offset_hz` how far the carrier sits from the middle of the
/// stream. The transitions are smoothed over `rise_s`, as a transmitter's
/// are: a beacon's are 150 microseconds and the spectrum is rather wider
/// without them.
pub fn key_biphase(
    bits: &[bool],
    rate_hz: f64,
    baud: f64,
    deviation_rad: f32,
    offset_hz: f64,
    rise_s: f64,
) -> Vec<C32> {
    let chip_samples = rate_hz / (baud * CHIPS_PER_BIT as f64);
    let n = (bits.len() as f64 * chip_samples * CHIPS_PER_BIT as f64).round() as usize;
    let smooth = 1.0 / (rise_s * rate_hz).max(1.0) as f32;
    let mut out = Vec::with_capacity(n);
    let (mut carrier, mut held) = (0.0f64, 0.0f32);
    for i in 0..n {
        let chip = (i as f64 / chip_samples) as usize;
        // Biphase-L: a one is the high chip first, a zero the low one.
        let high = match bits.get(chip / CHIPS_PER_BIT) {
            Some(bit) => chip.is_multiple_of(CHIPS_PER_BIT) == *bit,
            None => false,
        };
        let want = match high {
            true => deviation_rad,
            false => -deviation_rad,
        };
        // One pole towards the wanted phase, which is the rise time.
        held += (want - held) * smooth;
        carrier = (carrier + TAU * offset_hz / rate_hz).rem_euclid(TAU);
        out.push(C32::from_polar(1.0, carrier as f32 + held));
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Levels a bit pattern keys as biphase-L: the high chip first for a
    /// one, the low one first for a zero.
    fn chips_of(bits: &[bool]) -> Vec<bool> {
        bits.iter().flat_map(|b| [*b, !*b]).collect()
    }

    fn noisy(iq: &[C32], amplitude: f32, seed: u64) -> Vec<C32> {
        let mut s = seed;
        let mut next = move || {
            s ^= s << 13;
            s ^= s >> 7;
            s ^= s << 17;
            (s >> 40) as f32 / 8_388_608.0 - 1.0
        };
        iq.iter().map(|x| x + C32::new(next() * amplitude, next() * amplitude)).collect()
    }

    /// A keyed message comes back chip for chip, carrier offset and all: the
    /// offset estimate removes 900 Hz of tuning error and the residual
    /// carrier at 1.1 radians is the only phase reference used.
    #[test]
    fn a_keyed_message_comes_back_through_a_tuning_error() {
        let rate = 9_600.0;
        let baud = 400.0;
        let mut bits = vec![true; 15];
        bits.extend([false, false, false, true, false, true, true, true, true]);
        let mut seed = 0x5eed_1234u64;
        bits.extend((0..120).map(|_| {
            seed = seed.wrapping_mul(6_364_136_223_846_793_005).wrapping_add(1);
            seed >> 62 & 1 != 0
        }));

        let iq = key_biphase(&bits, rate, baud, 1.1, 900.0, 150e-6);
        // Half a second of carrier in front, as a beacon sends.
        let mut stream: Vec<C32> = (0..(rate * 0.16) as usize)
            .map(|i| C32::from_polar(1.0, (TAU * 900.0 * i as f64 / rate) as f32))
            .collect();
        stream.extend_from_slice(&iq);
        stream.extend(std::iter::repeat_n(C32::new(1.0, 0.0), (rate * 0.05) as usize));

        let mut d = BiphaseDemod::new(rate, baud);
        let mut chips = Vec::new();
        for block in stream.chunks(512) {
            d.process(block, &mut chips);
        }
        assert_eq!(d.chip_rate(), 800.0);

        let want = chips_of(&bits);
        let got: Vec<bool> = chips.iter().map(|c| *c > 0.0).collect();
        // The keyed chips are somewhere in the stream: find them, and every
        // one after must agree.
        let at = (0..=got.len() - want.len())
            .find(|i| got[*i..*i + 16] == want[..16])
            .expect("the preamble is not in the chips");
        let wrong = got[at..at + want.len()].iter().zip(&want).filter(|(a, b)| a != b).count();
        assert_eq!(wrong, 0, "{wrong} chips of {} came back wrong", want.len());
    }

    /// What the carrier reference and the clock survive, measured over the
    /// 288 chips of a long message in a 9.6 kHz stream: every chip right
    /// down to 7.8 dB of signal to noise, one wrong at 4.9 dB, and the whole
    /// message lost by 1.8 dB. The code behind it puts back three wrong bits,
    /// so a beacon is still read at the middle figure.
    #[test]
    fn every_chip_survives_eight_decibels_of_noise() {
        let (rate, baud) = (9_600.0, 400.0);
        let mut bits = vec![true; 15];
        bits.extend([false, false, false, true, false, true, true, true, true]);
        bits.extend((0..120).map(|i| i % 5 == 0 || i % 7 == 3));
        let iq = key_biphase(&bits, rate, baud, 1.1, 0.0, 150e-6);
        let mut stream: Vec<C32> = vec![C32::new(1.0, 0.0); (rate * 0.16) as usize];
        stream.extend_from_slice(&iq);
        stream.extend(std::iter::repeat_n(C32::new(1.0, 0.0), (rate * 0.05) as usize));
        let stream = noisy(&stream, 0.5, 0xfeed_beef);

        let mut d = BiphaseDemod::new(rate, baud);
        let mut chips = Vec::new();
        d.process(&stream, &mut chips);
        let want = chips_of(&bits);
        let got: Vec<bool> = chips.iter().map(|c| *c > 0.0).collect();
        let at = (0..=got.len() - want.len())
            .find(|i| got[*i..*i + 16] == want[..16])
            .expect("the preamble is not in the chips");
        let wrong = got[at..at + want.len()].iter().zip(&want).filter(|(a, b)| a != b).count();
        assert_eq!(wrong, 0, "{wrong} chips of {} came back wrong", want.len());
    }
}
