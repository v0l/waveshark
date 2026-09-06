//! RDS block framing: syndromes, offset words, and group synchronisation.
//!
//! Constants are from IEC 62106 / EN 50067. A block is 26 bits: 16 of data
//! followed by a 10-bit checkword, and the checkword has one of five offset
//! words added so the syndrome identifies which position in the group the
//! block occupies. That is the only sync marker RDS has, so recovering the
//! group boundary means finding a bit offset where the syndromes line up.

/// g(x) = x^10 + x^8 + x^7 + x^5 + x^4 + x^3 + 1
pub const POLY: u32 = 0x5B9;

/// Parity check matrix rows, most significant bit of the block first.
const H: [u16; 26] = [
    0b1000000000,
    0b0100000000,
    0b0010000000,
    0b0001000000,
    0b0000100000,
    0b0000010000,
    0b0000001000,
    0b0000000100,
    0b0000000010,
    0b0000000001,
    0b1011011100,
    0b0101101110,
    0b0010110111,
    0b1010000111,
    0b1110011111,
    0b1100010011,
    0b1101010101,
    0b1101110110,
    0b0110111011,
    0b1000000001,
    0b1111011100,
    0b0111101110,
    0b0011110111,
    0b1010100111,
    0b1110001111,
    0b1100011011,
];

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Offset {
    A,
    B,
    C,
    /// C', used by version B groups.
    CPrime,
    D,
}

impl Offset {
    /// The word added to the checkword when transmitting.
    pub const fn word(self) -> u16 {
        match self {
            Offset::A => 0b0011111100,
            Offset::B => 0b0110011000,
            Offset::C => 0b0101101000,
            Offset::CPrime => 0b1101010000,
            Offset::D => 0b0110110100,
        }
    }

    /// The syndrome an error-free block carrying this offset produces.
    pub const fn syndrome(self) -> u16 {
        match self {
            Offset::A => 0b1111011000,
            Offset::B => 0b1111010100,
            Offset::C => 0b1001011100,
            Offset::CPrime => 0b1111001100,
            Offset::D => 0b1001011000,
        }
    }

    pub fn from_syndrome(s: u16) -> Option<Self> {
        [Offset::A, Offset::B, Offset::C, Offset::CPrime, Offset::D]
            .into_iter()
            .find(|o| o.syndrome() == s)
    }

    /// Position within a group, with C and C' both being the third block.
    pub fn index(self) -> usize {
        match self {
            Offset::A => 0,
            Offset::B => 1,
            Offset::C | Offset::CPrime => 2,
            Offset::D => 3,
        }
    }
}

/// Syndrome of a 26-bit block.
pub fn syndrome(block: u32) -> u16 {
    let mut r = 0u16;
    for k in 0..26 {
        if (block >> k) & 1 != 0 {
            r ^= H[25 - k];
        }
    }
    r
}

/// Checkword for 16 bits of data, before the offset is added.
pub fn checkword(data: u16) -> u16 {
    let mut r = (data as u32) << 10;
    for k in (10..26).rev() {
        if r & (1 << k) != 0 {
            r ^= POLY << (k - 10);
        }
    }
    (r & 0x3FF) as u16
}

/// Build a transmittable 26-bit block.
pub fn encode(data: u16, offset: Offset) -> u32 {
    ((data as u32) << 10) | ((checkword(data) ^ offset.word()) as u32)
}

pub fn data_of(block: u32) -> u16 {
    ((block >> 10) & 0xFFFF) as u16
}

/// One complete group: four data words plus which C variant was seen.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Group {
    pub words: [u16; 4],
    /// Which blocks passed their syndrome check. A block that did not is
    /// whatever noise happened to be there, so a consumer must not read it.
    pub valid: [bool; 4],
    pub c_prime: bool,
}

impl Group {
    pub fn all_valid(&self) -> bool {
        self.valid.iter().all(|v| *v)
    }
}

/// Recovers group framing from a bit stream.
///
/// RDS has no preamble, so sync is established by finding a bit position where
/// blocks decode to the expected offsets in sequence, and is dropped again
/// after enough consecutive failures.
pub struct BlockSync {
    reg: u32,
    bits: u32,
    /// Position within the group once synced, 0 to 3.
    slot: usize,
    synced: bool,
    words: [u16; 4],
    valid: [bool; 4],
    c_prime: bool,
    good: u32,
    bad: u32,
    pub groups: u64,
    pub errors: u64,
    /// Groups discarded because their identifying blocks did not check out.
    pub rejected: u64,
}

/// Consecutive bad blocks tolerated before sync is dropped.
const MAX_BAD: u32 = 6;

impl Default for BlockSync {
    fn default() -> Self {
        Self::new()
    }
}

impl BlockSync {
    pub fn new() -> Self {
        Self {
            reg: 0,
            bits: 0,
            slot: 0,
            synced: false,
            words: [0; 4],
            valid: [false; 4],
            c_prime: false,
            good: 0,
            bad: 0,
            groups: 0,
            errors: 0,
            rejected: 0,
        }
    }

    pub fn is_synced(&self) -> bool {
        self.synced
    }

    /// Feed one bit, returning a group once four blocks have lined up.
    pub fn push(&mut self, bit: u8) -> Option<Group> {
        self.reg = ((self.reg << 1) | bit as u32) & 0x3FF_FFFF;
        self.bits += 1;
        if self.bits < 26 {
            return None;
        }

        let found = Offset::from_syndrome(syndrome(self.reg));

        if !self.synced {
            // Only block A can start a group, so waiting for it costs at most
            // one group and removes any ambiguity about the slot.
            if found == Some(Offset::A) {
                self.synced = true;
                self.slot = 0;
                self.words[0] = data_of(self.reg);
                self.valid = [false; 4];
                self.valid[0] = true;
                self.c_prime = false;
                self.slot = 1;
                self.bits = 0;
                self.good = 1;
                self.bad = 0;
            }
            return None;
        }

        // Synced: a block is due every 26 bits regardless of whether its
        // syndrome checks out, or one bad block would shift the framing.
        if self.bits < 26 {
            return None;
        }
        self.bits = 0;

        let expected_ok = matches!(
            (self.slot, found),
            (0, Some(Offset::A))
                | (1, Some(Offset::B))
                | (2, Some(Offset::C | Offset::CPrime))
                | (3, Some(Offset::D))
        );
        if expected_ok {
            self.good += 1;
            self.bad = 0;
        } else {
            self.bad += 1;
            self.errors += 1;
            if self.bad >= MAX_BAD {
                self.synced = false;
                self.good = 0;
                self.bad = 0;
                return None;
            }
        }
        if self.slot == 2 {
            self.c_prime = found == Some(Offset::CPrime);
        }
        self.words[self.slot] = data_of(self.reg);
        self.valid[self.slot] = expected_ok;

        self.slot += 1;
        if self.slot == 4 {
            self.slot = 0;
            // Blocks A and B carry the identifier and the group type, so
            // without them there is nothing to interpret the rest against.
            // Emitting a group assembled from blocks that failed their check
            // is worse than emitting nothing: measured against stations with
            // no RDS at all it produced about eight groups per sixty-eight
            // block times, each with a different identifier, which reads as a
            // working decoder finding a station that is not there.
            if !(self.valid[0] && self.valid[1]) {
                self.rejected += 1;
                return None;
            }
            self.groups += 1;
            return Some(Group {
                words: self.words,
                valid: self.valid,
                c_prime: self.c_prime,
            });
        }
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const ALL: [Offset; 5] =
        [Offset::A, Offset::B, Offset::C, Offset::CPrime, Offset::D];

    #[test]
    fn encoded_blocks_produce_the_standard_syndromes() {
        for o in ALL {
            let b = encode(0xF212, o);
            assert_eq!(syndrome(b), o.syndrome(), "{o:?} syndrome mismatch");
            assert_eq!(Offset::from_syndrome(syndrome(b)), Some(o));
        }
    }

    #[test]
    fn the_offset_words_themselves_carry_their_syndrome() {
        // A block of all-zero data with only the offset added must still
        // identify, which is what makes the syndrome a position marker.
        for o in ALL {
            assert_eq!(syndrome(o.word() as u32), o.syndrome());
        }
    }

    #[test]
    fn data_survives_a_round_trip() {
        for d in [0x0000u16, 0xFFFF, 0xF212, 0x1234, 0xABCD] {
            for o in ALL {
                assert_eq!(data_of(encode(d, o)), d);
            }
        }
    }

    #[test]
    fn a_single_bit_error_is_detected() {
        for o in ALL {
            let good = encode(0xF212, o);
            for bit in 0..26 {
                let bad = good ^ (1 << bit);
                assert_ne!(syndrome(bad), o.syndrome(), "flip of bit {bit} went unnoticed");
            }
        }
    }

    /// Bits of a group, most significant first, as they go on air.
    fn group_bits(words: [u16; 4]) -> Vec<u8> {
        let offs = [Offset::A, Offset::B, Offset::C, Offset::D];
        let mut v = Vec::new();
        for (w, o) in words.iter().zip(offs) {
            let b = encode(*w, o);
            for k in (0..26).rev() {
                v.push(((b >> k) & 1) as u8);
            }
        }
        v
    }

    #[test]
    fn a_clean_stream_synchronises_and_decodes() {
        let words = [0xF212, 0x0408, 0x2037, 0x4D41];
        let mut s = BlockSync::new();
        let mut got = Vec::new();
        for _ in 0..4 {
            for b in group_bits(words) {
                if let Some(g) = s.push(b) {
                    got.push(g);
                }
            }
        }
        assert!(s.is_synced());
        assert!(!got.is_empty(), "no groups recovered from a clean stream");
        assert_eq!(got[0].words, words);
    }

    #[test]
    fn sync_is_found_despite_leading_junk() {
        let words = [0xF212, 0x0408, 0x2037, 0x4D41];
        let mut s = BlockSync::new();
        let mut got = Vec::new();
        // Arbitrary bits before the first real block, as on a fresh tune.
        let mut stream: Vec<u8> = (0..37).map(|i| ((i * 7 + 3) % 2) as u8).collect();
        for _ in 0..5 {
            stream.extend(group_bits(words));
        }
        for b in stream {
            if let Some(g) = s.push(b) {
                got.push(g);
            }
        }
        assert!(!got.is_empty(), "never synchronised");
        assert_eq!(got[0].words, words);
    }

    #[test]
    fn sync_is_dropped_when_the_signal_becomes_noise() {
        let words = [0xF212, 0x0408, 0x2037, 0x4D41];
        let mut s = BlockSync::new();
        for _ in 0..3 {
            for b in group_bits(words) {
                s.push(b);
            }
        }
        assert!(s.is_synced());
        let mut n = 0u32;
        for i in 0..26 * 40 {
            n = n.wrapping_mul(1103515245).wrapping_add(12345 + i);
            s.push(((n >> 16) & 1) as u8);
        }
        assert!(!s.is_synced(), "held sync on pure noise");
    }

    #[test]
    fn noise_does_not_produce_groups() {
        // Against a station with no RDS this used to emit roughly one group
        // per eight block times, each with a different identifier, which is
        // indistinguishable from a working decoder until the identifier is
        // noticed to change on every run.
        let mut s = BlockSync::new();
        let mut n = 0x1234_5678u32;
        let mut got = 0;
        for _ in 0..26 * 4000 {
            n = n.wrapping_mul(1103515245).wrapping_add(12345);
            if s.push(((n >> 16) & 1) as u8).is_some() {
                got += 1;
            }
        }
        assert!(got <= 1, "{got} groups out of pure noise");
    }

    #[test]
    fn a_group_reports_which_blocks_checked_out() {
        let words = [0xF212, 0x0408, 0x2037, 0x4D41];
        let mut s = BlockSync::new();
        for b in group_bits(words) {
            s.push(b);
        }
        // Corrupt block D only.
        let mut bits = group_bits(words);
        let last = bits.len() - 10;
        bits[last] ^= 1;
        let mut got = None;
        for b in bits {
            if let Some(g) = s.push(b) {
                got = Some(g);
            }
        }
        let g = got.expect("group should still be delivered");
        assert!(g.valid[0] && g.valid[1] && g.valid[2]);
        assert!(!g.valid[3], "corrupt block reported as good");
        assert!(!g.all_valid());
    }

    #[test]
    fn a_corrupt_block_does_not_shift_the_framing() {
        // One bad block must not desynchronise, or a single burst of noise
        // costs far more than the group it landed in.
        let words = [0xF212, 0x0408, 0x2037, 0x4D41];
        let mut s = BlockSync::new();
        for b in group_bits(words) {
            s.push(b);
        }
        let mut bits = group_bits(words);
        bits[30] ^= 1;
        for b in bits {
            s.push(b);
        }
        let mut got = None;
        for b in group_bits(words) {
            if let Some(g) = s.push(b) {
                got = Some(g);
            }
        }
        assert!(s.is_synced(), "one bad block dropped sync");
        assert_eq!(got.map(|g| g.words), Some(words), "framing shifted after an error");
    }
}
