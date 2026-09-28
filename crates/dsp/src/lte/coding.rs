use crate::conv::{Ends, LTE_1_3, Viterbi};

const PERMUTE: [usize; 32] = [
    1, 17, 9, 25, 5, 21, 13, 29, 3, 19, 11, 27, 7, 23, 15, 31, 0, 16, 8, 24, 4, 20, 12, 28, 2, 18,
    10, 26, 6, 22, 14, 30,
];

pub fn sub_block(d: usize) -> Vec<usize> {
    let rows = d.div_ceil(32);
    let pad = rows * 32 - d;
    PERMUTE
        .iter()
        .flat_map(|&col| (0..rows).map(move |r| r * 32 + col))
        .filter_map(|i| i.checked_sub(pad))
        .collect()
}

pub fn conv_dematch(e: &[f32], d: usize) -> Vec<f32> {
    let order = sub_block(d);
    let mut soft = vec![0f32; 3 * d];
    let period = 3 * order.len();
    for (j, &v) in e.iter().enumerate() {
        let at = j % period;
        let (stream, i) = (at / order.len(), order[at % order.len()]);
        soft[3 * i + stream] += v;
    }
    soft
}

pub fn conv_decode(e: &[f32], d: usize) -> Vec<u8> {
    Viterbi::decode_block(LTE_1_3, &conv_dematch(e, d), &[1, 1, 1], d, Ends::TailBiting)
}

pub fn crc16(bits: &[u8]) -> u16 {
    bits.iter().fold(0u16, |reg, &b| {
        let feedback = (reg >> 15) as u8 ^ b;
        let reg = reg << 1;
        if feedback == 1 { reg ^ 0x1021 } else { reg }
    })
}

pub fn crc16_mask(bits: &[u8]) -> Option<u16> {
    let a = bits.len().checked_sub(16)?;
    let sent = bits[a..].iter().fold(0u16, |v, &b| v << 1 | u16::from(b));
    Some(crc16(&bits[..a]) ^ sent)
}

pub fn to_bytes(bits: &[u8]) -> Vec<u8> {
    bits.chunks(8).map(|c| c.iter().fold(0u8, |v, &b| v << 1 | b) << (8 - c.len())).collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_sub_block_interleaver_keeps_40_bits_each_once() {
        let mut order = sub_block(40);
        assert_eq!(order[..6], [9, 25, 17, 1, 33, 13]);
        order.sort_unstable();
        assert_eq!(order, (0..40).collect::<Vec<_>>());
    }
}
