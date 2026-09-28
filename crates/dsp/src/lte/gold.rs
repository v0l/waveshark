pub fn sequence(len: usize, c_init: u32) -> Vec<u8> {
    const NC: usize = 1600;
    let n = NC + len + 31;
    let mut x1 = vec![0u8; n];
    let mut x2 = vec![0u8; n];
    x1[0] = 1;
    for (i, v) in x2.iter_mut().enumerate().take(31) {
        *v = (c_init >> i & 1) as u8;
    }
    for i in 0..n - 31 {
        x1[i + 31] = x1[i + 3] ^ x1[i];
        x2[i + 31] = x2[i + 3] ^ x2[i + 2] ^ x2[i + 1] ^ x2[i];
    }
    (0..len).map(|i| x1[i + NC] ^ x2[i + NC]).collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_pbch_scrambler_of_cell_481_starts_as_the_capture_reads_it() {
        assert_eq!(
            sequence(24, 481),
            [1, 0, 1, 0, 1, 1, 1, 1, 1, 0, 1, 0, 1, 1, 1, 0, 0, 0, 1, 0, 1, 0, 1, 1]
        );
    }
}
