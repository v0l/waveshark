//! Reading an MPEG-2 video stream a bit at a time, and finding the start
//! codes that break it into pieces.
//!
//! Start codes are the one thing in the format that can be found without
//! decoding: four bytes of 00 00 01 and an identifier, byte aligned, and the
//! encoder guarantees the pattern appears nowhere else by never letting more
//! than two zero bytes run inside coded data.

/// A bit reader over one picture's worth of bytes, most significant bit
/// first, which is the order every field in this format is written in.
pub struct Bits<'a> {
    data: &'a [u8],
    /// Bits consumed from the front.
    at: usize,
}

impl<'a> Bits<'a> {
    pub fn new(data: &'a [u8]) -> Self {
        Self { data, at: 0 }
    }

    /// Where the reader is, in bits.
    pub fn position(&self) -> usize {
        self.at
    }

    pub fn seek(&mut self, bit: usize) {
        self.at = bit;
    }

    pub fn exhausted(&self) -> bool {
        self.at >= self.data.len() * 8
    }

    /// The next `n` bits as a number, zero past the end. A decoder that runs
    /// off the end of a damaged picture reads zeros rather than panicking,
    /// and the picture is refused by whatever notices it made no sense.
    pub fn take(&mut self, n: usize) -> u32 {
        let v = self.peek(n);
        self.at += n;
        v
    }

    /// The next `n` bits without consuming them.
    pub fn peek(&self, n: usize) -> u32 {
        debug_assert!(n <= 32);
        let mut out = 0u32;
        for i in 0..n {
            let bit = self.at + i;
            let byte = self.data.get(bit / 8).copied().unwrap_or(0);
            out = (out << 1) | ((byte >> (7 - bit % 8)) & 1) as u32;
        }
        out
    }

    pub fn bit(&mut self) -> u32 {
        self.take(1)
    }

    /// A signed field of `n` bits, two's complement, as the format writes
    /// motion vector residuals and DC differentials.
    pub fn signed(&mut self, n: usize) -> i32 {
        if n == 0 {
            return 0;
        }
        let v = self.take(n) as i32;
        // The top bit is the sign, and a value with it clear is negative,
        // which is the opposite of two's complement: this is the format's own
        // convention for a differential.
        match v >> (n - 1) {
            0 => v - (1 << n) + 1,
            _ => v,
        }
    }

    /// Step over the padding to the next byte boundary.
    pub fn align(&mut self) {
        self.at = self.at.div_ceil(8) * 8;
    }

    /// Whether what follows is the padding before a start code, which is
    /// what ends a slice. Coded data can never produce 23 zero bits, and the
    /// reader is not byte aligned here, so the zeros are what is looked for
    /// rather than the code itself.
    pub fn at_start_code(&self) -> bool {
        self.peek(23) == 0 || self.exhausted()
    }
}

/// One piece of an elementary stream: a start code and everything up to the
/// next one.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Unit<'a> {
    pub code: u8,
    /// The bytes after the four byte start code.
    pub body: &'a [u8],
}

/// Where the last start code in `data` begins, which is where a feed has to
/// be cut: everything after it is a unit whose end has not arrived.
pub fn last_start(data: &[u8]) -> Option<usize> {
    if data.len() < 4 {
        return None;
    }
    (0..=data.len() - 4).rev().find(|&i| data[i] == 0 && data[i + 1] == 0 && data[i + 2] == 1)
}

/// Split a stream into its start code units. Anything before the first start
/// code is dropped, which is what happens to the tail of a picture a receiver
/// joined half way through.
pub fn units(data: &[u8]) -> Vec<Unit<'_>> {
    let mut out: Vec<Unit<'_>> = Vec::new();
    let mut starts: Vec<usize> = Vec::new();
    let mut i = 0usize;
    while i + 3 < data.len() {
        if data[i] == 0 && data[i + 1] == 0 && data[i + 2] == 1 {
            starts.push(i);
            i += 3;
        } else {
            i += 1;
        }
    }
    for (n, &s) in starts.iter().enumerate() {
        // One start code can follow another immediately, which leaves a unit
        // with no body rather than a range that runs backwards.
        let end = starts.get(n + 1).copied().unwrap_or(data.len()).max(s + 4);
        out.push(Unit { code: data[s + 3], body: &data[s + 4..end.min(data.len())] });
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn bits_come_out_most_significant_first() {
        let mut b = Bits::new(&[0b1011_0010, 0b0100_0001]);
        assert_eq!(b.take(1), 1);
        assert_eq!(b.take(3), 0b011);
        assert_eq!(b.take(4), 0b0010);
        assert_eq!(b.take(8), 0b0100_0001);
        assert!(b.exhausted());
        // Past the end is zeros rather than a panic.
        assert_eq!(b.take(8), 0);
    }

    /// The DC differential's sign convention: with the top bit set the value
    /// is the code, and with it clear the value is negative. Three bits carry
    /// 4 to 7 and -7 to -4, and never zero, because a differential of zero is
    /// coded as a size of zero and no bits at all.
    #[test]
    fn a_differential_is_signed_the_formats_way() {
        let mut b = Bits::new(&[0b1000_0000]);
        assert_eq!(b.signed(3), 4, "100 is +4");
        let mut b = Bits::new(&[0b1110_0000]);
        assert_eq!(b.signed(3), 7, "111 is +7");
        let mut b = Bits::new(&[0b0110_0000]);
        assert_eq!(b.signed(3), -4, "011 is -4");
        let mut b = Bits::new(&[0b0000_0000]);
        assert_eq!(b.signed(3), -7, "000 is -7");
        assert_eq!(Bits::new(&[0xFF]).signed(0), 0, "no bits is no change");
    }

    /// A stream splits at its start codes, and what comes before the first
    /// one is not a unit.
    #[test]
    fn a_stream_splits_at_its_start_codes() {
        let data = [0xAA, 0xBB, 0, 0, 1, 0xB3, 1, 2, 3, 0, 0, 1, 0x00, 9];
        let got = units(&data);
        assert_eq!(got.len(), 2);
        assert_eq!(got[0], Unit { code: 0xB3, body: &[1, 2, 3] });
        assert_eq!(got[1], Unit { code: 0x00, body: &[9] });
    }

    /// A slice ends where the next start code begins, and nothing inside
    /// coded data can look like one.
    #[test]
    fn a_start_code_is_seen_through_its_padding() {
        let b = Bits::new(&[0x00, 0x00, 0x01, 0x01]);
        assert!(b.at_start_code());
        // Part way through a byte, with the padding still to come.
        let mut b = Bits::new(&[0b0000_0111, 0x00, 0x00, 0x01, 0x01]);
        b.take(5);
        assert!(!b.at_start_code(), "there is coded data left");
        b.take(3);
        assert!(b.at_start_code());
        let b = Bits::new(&[0x00, 0x00, 0x02]);
        assert!(!b.at_start_code());
    }
}
