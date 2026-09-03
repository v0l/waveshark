//! Port of jmbe `jmbe.binary.BinaryFrame`: a fixed-size bit set with the
//! field access and rotation helpers the frame parsers rely on.

/// Fixed-size bit frame. Bit index 0 is the first bit, matching Java's
/// BitSet usage in jmbe.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct BitFrame {
    bits: Vec<bool>,
    size: usize,
    pointer: usize,
}

impl BitFrame {
    pub fn new(size: usize) -> Self {
        Self {
            bits: vec![false; size],
            size,
            pointer: 0,
        }
    }

    /// Builds a frame from `bits` with the pointer left at `size - 1`,
    /// matching the `BinaryFrame(BitSet, int)` constructor used for
    /// sub-message copies.
    pub fn from_bits(bits: &[bool]) -> Self {
        let size = bits.len();
        Self {
            bits: bits.to_vec(),
            size,
            pointer: size.saturating_sub(1),
        }
    }

    /// Port of `fromBytes`. `big_endian` true matches Java's
    /// `BitSet.valueOf` path: bits are taken LSB-first within each byte.
    /// Little endian matches `setByte` per byte: MSB of each byte first.
    pub fn from_bytes(data: &[u8], big_endian: bool) -> Self {
        let mut frame = Self::new(data.len() * 8);
        if big_endian {
            for (byte_index, byte) in data.iter().enumerate() {
                for bit in 0..8 {
                    frame.set_value(byte_index * 8 + bit, (byte >> bit) & 1 == 1);
                }
            }
        } else {
            for (byte_index, byte) in data.iter().enumerate() {
                frame.set_byte(byte_index * 8, *byte);
            }
        }
        frame
    }

    pub fn filled(size: usize, value: u64) -> Self {
        let mut frame = Self::new(size);
        frame.load(0, size as u32, value);
        frame
    }

    pub fn size(&self) -> usize {
        self.size
    }

    pub fn get(&self, index: usize) -> bool {
        self.bits.get(index).copied().unwrap_or(false)
    }

    pub fn set(&mut self, index: usize) {
        if index < self.bits.len() {
            self.bits[index] = true;
        }
    }

    pub fn clear(&mut self, index: usize) {
        if index < self.bits.len() {
            self.bits[index] = false;
        }
    }

    pub fn set_value(&mut self, index: usize, value: bool) {
        if index < self.bits.len() {
            self.bits[index] = value;
        }
    }

    pub fn flip(&mut self, index: usize) {
        if index < self.bits.len() {
            self.bits[index] = !self.bits[index];
        }
    }

    pub fn cardinality(&self) -> usize {
        self.bits.iter().filter(|b| **b).count()
    }

    /// Index of the first set bit at or after `from`, if any.
    pub fn next_set_bit(&self, from: usize) -> Option<usize> {
        (from..self.size).find(|i| self.bits[*i])
    }

    /// Copies bits `[start, end)` into a new frame.
    pub fn sub(&self, start: usize, end: usize) -> Self {
        Self::from_bits(&self.bits[start.min(self.size)..end.min(self.size)])
    }

    /// Writes a byte MSB-first starting at bit `index`.
    pub fn set_byte(&mut self, index: usize, value: u8) {
        let mut mask = 0x80u8;
        for x in 0..8 {
            self.set_value(index + x, (mask & value) == mask);
            mask >>= 1;
        }
    }

    /// Reads the byte starting at bit `offset`, MSB first. Ported
    /// faithfully, including the final rotate jmbe applies after the last
    /// bit, which makes this a left-rotated version of the packed byte.
    /// The codec frame parsers never call this, so the quirk is inert.
    pub fn get_byte(&self, offset: usize) -> u8 {
        let mut value: u32 = 0;
        for x in offset..offset + 8 {
            if self.get(x) {
                value += 1;
            }
            value = value.rotate_left(1);
        }
        (value & 0xFF) as u8
    }

    pub fn pointer(&self) -> usize {
        self.pointer
    }

    pub fn set_pointer(&mut self, index: usize) {
        self.pointer = index;
    }

    pub fn adjust_pointer(&mut self, adjustment: i32) {
        self.pointer = (self.pointer as i64 + adjustment as i64).max(0) as usize;
    }

    pub fn is_full(&self) -> bool {
        self.pointer >= self.size
    }

    /// Appends a bit at the pointer and advances it.
    pub fn add(&mut self, value: bool) {
        if self.pointer < self.bits.len() {
            self.bits[self.pointer] = value;
        }
        self.pointer += 1;
    }

    /// Bits from `start` through `end`, both inclusive. Returns `None` for
    /// the out-of-range cases where Java returns null.
    pub fn get_bits(&self, start: usize, end: usize) -> Option<Vec<bool>> {
        if start < end && end < self.size {
            Some(self.bits[start..=end].to_vec())
        } else {
            None
        }
    }

    /// Bits from `start` through the end of the frame.
    pub fn get_bits_from(&self, start: usize) -> Option<Vec<bool>> {
        if start < self.size {
            Some(self.bits[start..self.size].to_vec())
        } else {
            None
        }
    }

    /// The right-most `bit_count` bits plus one, matching
    /// `toReverseIntegerArray`'s sibling `right(int)` in jmbe.
    pub fn right(&self, bit_count: usize) -> Option<Vec<bool>> {
        if bit_count + 1 <= self.size {
            self.get_bits(self.size - bit_count - 1, self.size - 1)
        } else {
            None
        }
    }

    /// Integer value of the listed bit positions, index 0 treated as MSB.
    pub fn get_int(&self, bits: &[usize]) -> u32 {
        let mut value: u32 = 0;
        for index in bits {
            value = value.rotate_left(1);
            if self.get(*index) {
                value += 1;
            }
        }
        value
    }

    /// Integer value of a bit range. Ported faithfully: `start < end`
    /// reads forward; `start == end` reads the single bit; `start > end`
    /// hits jmbe's backward branch whose loop condition can never hold,
    /// so jmbe returns zero there and so does this port.
    pub fn get_int_range(&self, start: i32, end: i32) -> u32 {
        let mut value: u32 = 0;
        if start < end {
            for x in start..=end {
                value = value.rotate_left(1);
                if self.get(x.max(0) as usize) {
                    value += 1;
                }
            }
        } else if start == end {
            value = value.rotate_left(1);
            if self.get(start.max(0) as usize) {
                value += 1;
            }
        }
        value
    }

    /// Writes `value` as `width` bits starting at `offset`, MSB first.
    pub fn load(&mut self, offset: usize, width: u32, value: u64) {
        for x in 0..width {
            let mask = 1u64 << (width - x - 1);
            self.set_value(offset as usize + x as usize, (mask & value) == mask);
        }
    }

    /// Rotates bits `[start, end]` left by `places`, wrapping the left-most
    /// bit around to the end.
    pub fn rotate_left(&mut self, places: usize, start: usize, end: usize) {
        for _ in 0..places {
            self.rotate_left_once(start, end);
        }
    }

    pub fn rotate_left_once(&mut self, start: usize, end: usize) {
        if start >= self.bits.len() || end >= self.bits.len() || start >= end {
            return;
        }
        let wrap = self.bits[start];
        for x in start..end {
            self.bits[x] = self.bits[x + 1];
        }
        self.bits[end] = wrap;
    }

    /// Rotates bits `[start, end]` right by `places`, wrapping the right-most
    /// bit around to the start.
    pub fn rotate_right(&mut self, places: usize, start: usize, end: usize) {
        for _ in 0..places {
            self.rotate_right_once(start, end);
        }
    }

    pub fn rotate_right_once(&mut self, start: usize, end: usize) {
        if start >= self.bits.len() || end >= self.bits.len() || start >= end {
            return;
        }
        let wrap = self.bits[end];
        for x in (start..end).rev() {
            self.bits[x + 1] = self.bits[x];
        }
        self.bits[start] = wrap;
    }

    /// XORs the low `width` bits of `value` (MSB first) into the frame at
    /// `offset`.
    pub fn xor(&mut self, offset: usize, width: u32, value: u32) {
        for x in 0..width {
            let bit = (value >> (width - x - 1)) & 1 == 1;
            let index = offset as usize + x as usize;
            if index < self.bits.len() {
                self.bits[index] ^= bit;
            }
        }
    }

    pub fn as_bools(&self) -> &[bool] {
        &self.bits
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn byte_round_trip_carries_jmbe_rotate_quirk() {
        let mut frame = BitFrame::new(16);
        frame.set_byte(0, 0xA5);
        // 0xA5 packed MSB-first is 1010 0101; jmbe's getByte rotates the
        // packed byte left once more than a plain read, giving 0100 1010.
        assert_eq!(frame.get_byte(0), 0x4A);
    }

    #[test]
    fn from_bytes_little_endian_sets_msb_first() {
        let frame = BitFrame::from_bytes(&[0b1010_0001], false);
        assert!(frame.get(0));
        assert!(!frame.get(1));
        // get_byte carries jmbe's final rotate quirk: packed 0xA1 shifted
        // left once.
        assert_eq!(frame.get_byte(0), 0x42);
    }

    #[test]
    fn from_bytes_big_endian_is_lsb_first() {
        let frame = BitFrame::from_bytes(&[0b1010_0001], true);
        assert!(frame.get(0));
        assert!(frame.get(7));
        assert!(!frame.get(1));
    }

    #[test]
    fn int_range_forward_reads_msb_first() {
        let mut frame = BitFrame::new(24);
        frame.load(0, 12, 0b1010_0001_1111);
        assert_eq!(frame.get_int_range(0, 11), 0b1010_0001_1111);
        // jmbe's backward branch never iterates, so reversed ranges are 0.
        assert_eq!(frame.get_int_range(11, 0), 0);
        assert_eq!(frame.get_int_range(0, 0), 0b1);
    }

    #[test]
    fn rotations_wrap() {
        let mut frame = BitFrame::new(8);
        frame.load(0, 8, 0b1000_0001);
        frame.rotate_left_once(0, 7);
        assert_eq!(frame.get_int_range(0, 7), 0b0000_0011);
        frame.rotate_right_once(0, 7);
        frame.rotate_right_once(0, 7);
        assert_eq!(frame.get_int_range(0, 7), 0b1100_0000);
    }
}
