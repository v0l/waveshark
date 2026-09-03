//! Port of jmbe `jmbe.edac`: Golay(23,12), Golay(24,12) with parity, and
//! Hamming(15,11) check-and-correct, operating on [`BitFrame`].

use crate::bits::BitFrame;

pub const GOLAY_MAX_CORRECTABLE: usize = 3;

/// Golay(23,12) generator polynomial checksums for the 12 data bits.
pub const GOLAY_CHECKSUMS: [u32; 23] = [
    0x63A, 0x31D, 0x7B4, 0x3DA, 0x1ED, 0x6CC, 0x366, 0x1B3, 0x6E3, 0x54B, 0x49F, 0x475, 0x400,
    0x200, 0x100, 0x080, 0x040, 0x020, 0x010, 0x008, 0x004, 0x002, 0x001,
];

fn golay_checksum(frame: &BitFrame, start: usize) -> u32 {
    let mut calculated: u32 = 0;
    let mut i = match frame.next_set_bit(start) {
        Some(i) if i < start + 12 => i,
        _ => return calculated,
    };
    while let Some(index) = frame.next_set_bit(i) {
        if index >= start + 12 {
            break;
        }
        calculated ^= GOLAY_CHECKSUMS[index - start];
        i = index + 1;
    }
    calculated
}

fn golay_syndrome(frame: &BitFrame, start: usize) -> u32 {
    let calculated = golay_checksum(frame, start);
    let checksum = frame.get_int_range((start + 12) as i32, (start + 22) as i32);
    checksum ^ calculated
}

/// Golay(23,12) check and correct of the 23 bits at `start`. Returns the
/// number of corrected bit errors, or 4 when correction failed.
pub fn golay23_check_and_correct(frame: &mut BitFrame, start: usize) -> u32 {
    let syndrome = golay_syndrome(frame, start);
    if syndrome == 0 {
        return 0;
    }

    let mut copy = frame.sub(start, start + 23);

    let mut index: i32 = -1;
    let mut syndrome_weight: u32 = GOLAY_MAX_CORRECTABLE as u32;

    while index < 23 {
        if index != -1 {
            if index > 0 {
                copy.flip((index - 1) as usize);
            }
            copy.flip(index as usize);
            syndrome_weight = GOLAY_MAX_CORRECTABLE as u32 - 1;
        }

        let mut syndrome = golay_syndrome(&copy, 0);

        if syndrome > 0 {
            for i in 0..23 {
                let errors = syndrome.count_ones();

                if errors <= syndrome_weight {
                    copy.xor(12, 11, syndrome);
                    copy.rotate_right(i as usize, 0, 22);

                    // Java increments `errors` here for trial flips but
                    // returns errorCount below, so the increment is dead.

                    let corrected = copy.get_int_range(0, 22);
                    let original = frame.get_int_range(start as i32, (start + 22) as i32);
                    let error_count = (original ^ corrected).count_ones();

                    if error_count <= 3 {
                        frame.load(start, 23, corrected as u64);
                    }

                    return error_count;
                } else {
                    copy.rotate_left_once(0, 22);
                    syndrome = golay_syndrome(&copy, 0);
                }
            }

            index += 1;
        }
        // A zero syndrome repeats the same index: the next loop-top toggle
        // moves the trial bit one position left, which is how single
        // data-bit errors get picked up.
    }

    4
}

/// Golay(24,12) with overall parity, over the 24 bits starting at `start`.
/// Returns the number of corrected bit errors, or 4 when correction failed.
pub fn golay24_check_and_correct(frame: &mut BitFrame, start: usize) -> u32 {
    let parity_error = frame.cardinality() % 2 != 0;

    let syndrome = golay_syndrome(frame, start);

    if syndrome == 0 {
        if parity_error {
            frame.flip(start + 23);
            return 1;
        }
        return 0;
    }

    let original = frame.get_int_range(0, 22);

    let mut index: i32 = -1;
    let mut syndrome_weight: u32 = 3;

    while index < 23 {
        if index != -1 {
            if index > 0 {
                frame.flip((index - 1) as usize);
            }
            frame.flip(index as usize);
            syndrome_weight = 2;
        }

        let mut syndrome = golay_syndrome(frame, start);

        if syndrome > 0 {
            for i in 0..23 {
                let errors = syndrome.count_ones();

                if errors <= syndrome_weight {
                    frame.xor(start + 12, 11, syndrome);
                    frame.rotate_right(i as usize, start, start + 22);

                    let mut errors = errors;
                    if index >= 0 {
                        errors += 1;
                    }

                    let corrected = frame.get_int_range(0, 22);

                    if (original ^ corrected).count_ones() > 3 {
                        return 4;
                    }

                    return errors;
                } else {
                    frame.rotate_left_once(start, start + 22);
                    syndrome = golay_syndrome(frame, start);
                }
            }

            index += 1;
        }
        // Same repeat-on-zero-syndrome behaviour as golay23.
    }

    2
}

const HAMMING_CHECKSUMS: [u32; 11] = [0xF, 0xE, 0xD, 0xC, 0xB, 0xA, 0x9, 0x7, 0x6, 0x5, 0x3];

/// Hamming(15,11) check and correct of the 15 bits at `start`. Returns the
/// number of corrected bit errors (0, 1, or 2).
pub fn hamming15_check_and_correct(frame: &mut BitFrame, start: usize) -> u32 {
    let syndrome = hamming_syndrome(frame, start);

    let target: Option<usize> = match syndrome {
        0 => return 0,
        1 => Some(start + 14),
        2 => Some(start + 13),
        3 => Some(start + 10),
        4 => Some(start + 12),
        5 => Some(start + 9),
        6 => Some(start + 8),
        7 => Some(start + 7),
        8 => Some(start + 11),
        9 => Some(start + 6),
        10 => Some(start + 5),
        11 => Some(start + 4),
        12 => Some(start + 3),
        13 => Some(start + 2),
        14 => Some(start + 1),
        15 => Some(start),
        _ => None,
    };

    match target {
        Some(index) => {
            frame.flip(index);
            1
        }
        None => 2,
    }
}

fn hamming_checksum(frame: &BitFrame, start: usize) -> u32 {
    let mut calculated: u32 = 0;
    let mut i = match frame.next_set_bit(start) {
        Some(i) if i < start + 11 => i,
        _ => return calculated,
    };
    while let Some(index) = frame.next_set_bit(i) {
        if index >= start + 11 {
            break;
        }
        calculated ^= HAMMING_CHECKSUMS[index - start];
        i = index + 1;
    }
    calculated
}

fn hamming_syndrome(frame: &BitFrame, start: usize) -> u32 {
    let calculated = hamming_checksum(frame, start);
    let checksum = frame.get_int_range((start + 11) as i32, (start + 14) as i32);
    checksum ^ calculated
}

#[cfg(test)]
mod tests {
    use super::*;
    use rand_compat::xorshift;

    /// Deterministic stand-in so the exhaustive test can flip pseudo-random
    /// error patterns without pulling a random-number crate.
    mod rand_compat {
        pub fn xorshift(state: &mut u64) -> u64 {
            *state ^= *state << 13;
            *state ^= *state >> 7;
            *state ^= *state << 17;
            *state
        }
    }

    /// Builds a valid Golay(23,12) codeword: 12 data bits at 0..11 with the
    /// 11-bit checksum at 12..22.
    fn golay23_encode(data: u32) -> BitFrame {
        let mut frame = BitFrame::new(23);
        frame.load(0, 12, data as u64);
        let checksum = golay_checksum(&frame, 0);
        frame.load(12, 11, checksum as u64);
        frame
    }

    /// The jmbe Golay23 test harness, over valid codewords: any 23-bit
    /// codeword must survive up to 3 flipped bits.
    #[test]
    fn golay23_corrects_all_three_error_patterns() {
        let mut state = 0x1234_5678u64;
        for data in 0..4096u32 {
            let codeword = golay23_encode(data).get_int_range(0, 22);
            let mut frame = BitFrame::new(23);
            frame.load(0, 23, codeword as u64);

            // Flip exactly three pseudo-random distinct bits.
            let mut flipped = [usize::MAX; 3];
            for f in 0..3 {
                loop {
                    let bit = (xorshift(&mut state) % 23) as usize;
                    if !flipped.contains(&bit) {
                        flipped[f] = bit;
                        frame.flip(bit);
                        break;
                    }
                }
            }

            let errors = golay23_check_and_correct(&mut frame, 0);
            assert!(errors <= 3, "data {data:#x} reported {errors} errors");
            assert_eq!(frame.get_int_range(0, 22), codeword, "data {data:#x} not restored");
        }
    }

    #[test]
    fn golay23_clean_frame_reports_zero() {
        let mut frame = golay23_encode(0x1AB);
        assert_eq!(golay23_check_and_correct(&mut frame, 0), 0);
        assert_eq!(frame.get_int_range(0, 22), golay23_encode(0x1AB).get_int_range(0, 22));
    }

    fn hamming15_encode(data: u32) -> BitFrame {
        let mut frame = BitFrame::new(15);
        frame.load(0, 11, data as u64);
        let checksum = hamming_checksum(&frame, 0);
        frame.load(11, 4, checksum as u64);
        frame
    }

    #[test]
    fn hamming15_corrects_single_bit_errors() {
        let codeword = hamming15_encode(0b1010_1011_001).get_int_range(0, 14);
        for bit in 0..15 {
            let mut frame = BitFrame::new(15);
            frame.load(0, 15, codeword as u64);
            frame.flip(bit);
            assert_eq!(hamming15_check_and_correct(&mut frame, 0), 1);
            assert_eq!(frame.get_int_range(0, 14), codeword);
        }
    }

    #[test]
    fn golay24_corrects_with_parity() {
        let mut frame = golay23_encode(0x5A7);
        if frame.cardinality() % 2 == 1 {
            frame.set(23); // overall parity for even cardinality
        }
        assert_eq!(frame.cardinality() % 2, 0);
        frame.flip(7);
        assert_eq!(golay24_check_and_correct(&mut frame, 0), 1);
        assert_eq!(frame.get_int_range(0, 22), golay23_encode(0x5A7).get_int_range(0, 22));
    }
}
