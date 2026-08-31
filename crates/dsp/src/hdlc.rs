//! HDLC framing: flags, bit stuffing and the frame check sequence.
//!
//! Shared, because two unrelated things on two unrelated bands turn out to
//! use exactly this. AIS on 162 MHz and AX.25 on 144 MHz both wrap their
//! payload in `0x7E` flags, stuff a zero after five ones so no flag can occur
//! inside a frame, and finish with the X.25 check sequence. They differ in
//! everything above this layer and in nothing at it.
//!
//! # What this decides, and what it does not
//!
//! It decides whether a frame happened. The flags delimit it, the destuffing
//! recovers it, and the check sequence is the only thing separating a frame
//! from a run of noise that looked like one, which is why the acceptance test
//! lives here rather than in whatever parses the result.
//!
//! It does **not** decide what the bits mean, and deliberately hands back
//! bits rather than bytes. HDLC puts each byte on the air least significant
//! bit first, but the layer above does not have to agree with that: AX.25
//! frames are ordinary bytes and pack least significant first, while an AIS
//! message is defined as a bit string read most significant first. Packing
//! here would force one of them to unpack and reverse again, which is exactly
//! how a decoder ends up recovering a message type and nothing else. So the
//! caller packs, with [`pack_lsb`] or [`pack_msb`].

/// The flag, and the only byte that cannot occur inside a frame.
pub const FLAG: u8 = 0x7E;

/// What the check sequence leaves behind when the frame is intact.
///
/// A property of the CRC rather than a magic number: running the check over
/// the data and its own complemented remainder always lands here.
pub const FCS_RESIDUE: u16 = 0xF0B8;

/// Bits of check sequence at the end of every frame.
const FCS_BITS: usize = 16;

/// Flags, destuffing and the check sequence, one bit at a time.
pub struct Hdlc {
    /// Last eight bits, for spotting a flag.
    history: u8,
    /// Collecting between flags.
    active: bool,
    bits: Vec<bool>,
    ones: u8,
    min_bits: usize,
    max_bits: usize,
}

impl Hdlc {
    /// `min_bits` and `max_bits` bound the data and its check sequence
    /// together. They are not the same for every protocol: an AIS message is
    /// 168 bits and an AX.25 frame carrying a full information field is well
    /// over a thousand, and a bound that fits both would let a stretch of
    /// noise be assembled into something enormous before the check rejects
    /// it.
    pub fn new(min_bits: usize, max_bits: usize) -> Self {
        Self {
            history: 0,
            active: false,
            bits: Vec::with_capacity(max_bits.min(4096)),
            ones: 0,
            min_bits,
            max_bits,
        }
    }

    pub fn reset(&mut self) {
        self.active = false;
        self.bits.clear();
        self.ones = 0;
        self.history = 0;
    }

    /// Feed one decoded bit.
    ///
    /// Returns the data bits of a frame that closed and whose check sequence
    /// held, with the check sequence itself removed. Nothing is returned for
    /// a frame that failed it: unlike a sensor reading, half a position is
    /// not worth passing on, because it puts a thing somewhere it is not.
    pub fn push(&mut self, bit: bool) -> Option<Vec<bool>> {
        self.history = (self.history << 1) | u8::from(bit);
        if self.history == FLAG {
            // A flag both ends the frame in progress and opens the next,
            // which is what makes back-to-back frames work. Seven of the
            // flag's eight bits were collected before it was recognised,
            // since nothing shorter identifies it.
            let done = self.active && self.bits.len() > 7;
            let frame = done.then(|| {
                let keep = self.bits.len() - 7;
                self.bits.truncate(keep);
                self.finish()
            });
            self.active = true;
            self.bits.clear();
            self.ones = 0;
            return frame.flatten();
        }

        if !self.active {
            return None;
        }

        // Five ones then a zero is a stuffed bit and never data.
        if self.ones == 5 && !bit {
            self.ones = 0;
            return None;
        }
        self.ones = if bit { self.ones + 1 } else { 0 };
        // Seven ones is the abort sequence, and more than that is noise.
        if self.ones > 6 || self.bits.len() > self.max_bits {
            self.reset();
            return None;
        }
        self.bits.push(bit);
        None
    }

    fn finish(&self) -> Option<Vec<bool>> {
        if self.bits.len() < self.min_bits {
            return None;
        }
        // The check runs over the data and the sequence together, in the
        // order they went on the air.
        if crc_x25(&self.bits) != FCS_RESIDUE {
            return None;
        }
        Some(self.bits[..self.bits.len() - FCS_BITS].to_vec())
    }
}

/// The X.25 frame check sequence, fed one bit at a time in transmission
/// order, which is least significant bit first within each byte.
pub fn crc_x25(bits: &[bool]) -> u16 {
    let mut crc: u16 = 0xFFFF;
    for &b in bits {
        let x = (crc & 1) != 0;
        crc >>= 1;
        if x != b {
            crc ^= 0x8408;
        }
    }
    crc
}

/// Pack bits least significant first, which is the order HDLC puts bytes on
/// the air and therefore how AX.25 fields are reassembled.
pub fn pack_lsb(bits: &[bool]) -> Vec<u8> {
    let mut out = vec![0u8; bits.len() / 8];
    for (i, &b) in bits.iter().enumerate() {
        if b && i / 8 < out.len() {
            out[i / 8] |= 1 << (i % 8);
        }
    }
    out
}

/// Pack bits most significant first, which is how an AIS message is defined
/// and how every published AIS payload is written.
pub fn pack_msb(bits: &[bool]) -> Vec<u8> {
    let mut out = vec![0u8; bits.len().div_ceil(8)];
    for (i, &b) in bits.iter().enumerate() {
        if b {
            out[i / 8] |= 0x80 >> (i % 8);
        }
    }
    out
}

/// Append the check sequence a transmitter would: the complement of the
/// remainder, least significant bit first.
pub fn with_fcs(data: &[bool]) -> Vec<bool> {
    let crc = !crc_x25(data);
    let mut out = data.to_vec();
    for i in 0..FCS_BITS {
        out.push((crc >> i) & 1 == 1);
    }
    out
}

/// Insert a zero after every five ones, as a transmitter must so that no flag
/// can occur inside a frame.
pub fn stuff(bits: &[bool]) -> Vec<bool> {
    let mut out = Vec::with_capacity(bits.len() + bits.len() / 5);
    let mut ones = 0;
    for &b in bits {
        out.push(b);
        ones = if b { ones + 1 } else { 0 };
        if ones == 5 {
            out.push(false);
            ones = 0;
        }
    }
    out
}

/// The flag as bits, most significant first.
pub fn flag_bits() -> Vec<bool> {
    (0..8).map(|i| FLAG >> (7 - i) & 1 == 1).collect()
}

/// NRZI encode: a zero is a transition, a one holds the level.
pub fn nrzi(bits: &[bool]) -> Vec<bool> {
    let mut level = false;
    bits.iter()
        .map(|&b| {
            if !b {
                level = !level;
            }
            level
        })
        .collect()
}

/// Wrap already-ordered payload bits in a complete frame: `lead` flags, the
/// stuffed data and check sequence, then a closing flag, NRZI encoded.
///
/// The inverse of what the framer does, and public for the same reason a
/// decoder's tests want an encoder: without a recorded capture the only
/// honest way to test a demodulator is to transmit something known and see
/// whether it comes back.
pub fn encode_frame(data: &[bool], lead: &[bool]) -> Vec<bool> {
    let flag = flag_bits();
    let mut air: Vec<bool> = lead.to_vec();
    air.extend_from_slice(&flag);
    air.extend(stuff(&with_fcs(data)));
    air.extend_from_slice(&flag);
    nrzi(&air)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn bits_msb(bytes: &[u8]) -> Vec<bool> {
        (0..bytes.len() * 8).map(|i| bytes[i / 8] >> (7 - i % 8) & 1 == 1).collect()
    }

    /// The property the acceptance test rests on.
    #[test]
    fn a_frame_and_its_own_check_sequence_leave_the_residue() {
        let data = bits_msb(&[0x01, 0x23, 0x45, 0x67, 0x89, 0xab]);
        assert_eq!(crc_x25(&with_fcs(&data)), FCS_RESIDUE);
        let mut bad = with_fcs(&data);
        bad[5] = !bad[5];
        assert_ne!(crc_x25(&bad), FCS_RESIDUE);
    }

    /// Framing round trip, with no radio in the way: what a transmitter would
    /// send comes back out of the framer as what went in.
    #[test]
    fn a_stuffed_frame_survives_the_framer() {
        let data = bits_msb(&[0xff, 0xff, 0x7e, 0x00, 0x3f, 0xf8]);
        let air = encode_frame(&data, &[]);
        // Undo the NRZI the encoder applied, which is the demodulator's job.
        let mut out = None;
        let mut h = Hdlc::new(16, 2048);
        let mut prev = false;
        for level in air {
            let bit = level == prev;
            prev = level;
            if let Some(f) = h.push(bit) {
                out = Some(f);
            }
        }
        assert_eq!(out.as_deref(), Some(&data[..]), "the payload came back changed");
    }

    /// Data that contains five ones must not be able to fake a flag, which is
    /// the entire reason stuffing exists.
    #[test]
    fn stuffing_keeps_a_flag_from_occurring_inside_a_frame() {
        let data = bits_msb(&[0xfe, 0x7e, 0xff]);
        let stuffed = stuff(&data);
        let mut run = 0;
        for b in stuffed {
            run = if b { run + 1 } else { 0 };
            assert!(run < 6, "six ones in a row would read as a flag");
        }
    }

    /// The two orders are genuinely different, which is why the framer refuses
    /// to choose one.
    #[test]
    fn the_two_packings_disagree_and_both_are_needed() {
        let bits = bits_msb(&[0b1010_0000]);
        assert_eq!(pack_msb(&bits), vec![0b1010_0000]);
        assert_eq!(pack_lsb(&bits), vec![0b0000_0101]);
    }

    #[test]
    fn a_frame_that_fails_its_check_is_dropped() {
        let data = bits_msb(&[0x12, 0x34, 0x56, 0x78]);
        let mut air = encode_frame(&data, &[]);
        // Flip a symbol inside the data, past the opening flag.
        air[20] = !air[20];
        let mut h = Hdlc::new(16, 2048);
        let mut prev = false;
        for level in air {
            let bit = level == prev;
            prev = level;
            assert!(h.push(bit).is_none(), "a corrupted frame was accepted");
        }
    }
}
