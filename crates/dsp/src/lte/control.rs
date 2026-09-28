use super::alloc::{self, Allocation, Gap};
use super::coding::{conv_decode, crc16_mask, sub_block};
use super::grid::{Channel, Grid, combine, descramble, is_crs, soft_bits};
use super::{Mib, SI_RNTI};

const CFI_CODES: [[u8; 3]; 3] = [[0, 1, 1], [1, 0, 1], [1, 1, 0]];
const CCE_REGS: usize = 9;
const CCE_BITS: usize = 72;

fn reg_size(l: usize, ports: usize) -> usize {
    if l == 0 || (l == 1 && ports == 4) { 6 } else { 4 }
}

fn reg_elements(l: usize, k: usize, pci: u16, ports: usize) -> Vec<(usize, usize)> {
    (k..k + reg_size(l, ports))
        .filter(|&k| !is_crs(l, k, pci, ports.max(2)))
        .map(|k| (l, k))
        .collect()
}

fn pcfich_regs(nrb: usize, pci: u16) -> [usize; 4] {
    let nsc = nrb * 12;
    let first = 6 * (usize::from(pci) % (2 * nrb));
    std::array::from_fn(|i| (first + (i * nrb / 2) * 6) % nsc)
}

pub fn cfi(grid: &Grid, ch: &Channel, subframe: usize, pci: u16, ports: usize) -> Option<usize> {
    let res: Vec<(usize, usize)> =
        pcfich_regs(grid.nrb, pci).iter().flat_map(|&k| reg_elements(0, k, pci, ports)).collect();
    let mut soft = soft_bits(&combine(grid, ch, &res, ports));
    let pci32 = u32::from(pci);
    descramble(&mut soft, (subframe as u32 + 1) * (2 * pci32 + 1) * 512 + pci32);
    let score = |code: &[u8; 3]| -> f32 {
        soft.iter().enumerate().map(|(i, v)| if code[i % 3] == 0 { *v } else { -v }).sum()
    };
    (1..=3).max_by(|a, b| score(&CFI_CODES[a - 1]).total_cmp(&score(&CFI_CODES[b - 1])))
}

fn reg_starts(nrb: usize, l: usize, ports: usize) -> impl Iterator<Item = usize> {
    (0..nrb * 12).step_by(reg_size(l, ports))
}

fn phich_regs(nrb: usize, pci: u16, mib: &Mib, pcfich: &[usize; 4]) -> Vec<(usize, usize)> {
    let symbols = if mib.phich_extended { 3 } else { 1 };
    let free: Vec<Vec<usize>> = (0..symbols)
        .map(|l| reg_starts(nrb, l, mib.ports).filter(|k| l > 0 || !pcfich.contains(k)).collect())
        .collect();
    let n0 = free[0].len();
    let mut out = Vec::new();
    for m in 0..mib.phich.groups(nrb) {
        for i in 0..3 {
            let l = if mib.phich_extended { i } else { 0 };
            let n = free[l].len();
            let at = (usize::from(pci) * n / n0 + m + i * n / 3) % n;
            out.push((l, free[l][at]));
        }
    }
    out
}

pub fn control_symbols(cfi: usize, nrb: usize) -> usize {
    if nrb <= 10 { cfi + 1 } else { cfi }
}

pub fn pdcch_soft(
    grid: &Grid,
    ch: &Channel,
    subframe: usize,
    pci: u16,
    mib: &Mib,
    symbols: usize,
) -> Option<Vec<f32>> {
    let nrb = grid.nrb;
    let pcfich = pcfich_regs(nrb, pci);
    let phich = phich_regs(nrb, pci, mib, &pcfich);
    let mut regs: Vec<Vec<f32>> = Vec::new();
    for k in 0..nrb * 12 {
        for l in 0..symbols {
            if k % reg_size(l, mib.ports) != 0
                || (l == 0 && pcfich.contains(&k))
                || phich.contains(&(l, k))
            {
                continue;
            }
            let res = reg_elements(l, k, pci, mib.ports);
            regs.push(soft_bits(&combine(grid, ch, &res, mib.ports)));
        }
    }
    let m = regs.len();
    let shift = usize::from(pci) % m.max(1);
    let mut quads = vec![Vec::new(); m];
    for (t, j) in sub_block(m).into_iter().enumerate() {
        quads[j] = std::mem::take(&mut regs[(t + m - shift) % m]);
    }
    let cces = m / CCE_REGS;
    let mut soft: Vec<f32> = quads.into_iter().take(cces * CCE_REGS).flatten().collect();
    descramble(&mut soft, subframe as u32 * 512 + u32::from(pci));
    Some(soft)
}

pub fn dci_1a_bits(nrb: usize) -> usize {
    let riv = ((nrb * (nrb + 1) / 2) as f64).log2().ceil() as usize;
    let size = 15 + riv;
    if [12, 14, 16, 20, 24, 26, 32, 40, 44, 56].contains(&size) { size + 1 } else { size }
}

fn riv_bits(values: usize) -> usize {
    (values as f64).log2().ceil() as usize
}

pub fn dci_1c_bits(nrb: usize) -> usize {
    let n = alloc::n_vrb(nrb, Gap::First) / alloc::step_1c(nrb);
    usize::from(nrb >= 50) + riv_bits(n * (n + 1) / 2) + 5
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SiGrant {
    pub allocation: Allocation,
    pub tbs: usize,
    pub rv: u8,
}

fn field(bits: &[u8], from: usize, len: usize) -> usize {
    bits[from..from + len].iter().fold(0, |v, &b| v << 1 | usize::from(b))
}

pub fn rv_of(k: u8) -> u8 {
    (3 * (k % 4)).div_ceil(2) % 4
}

pub fn si_rv(sfn: u16) -> u8 {
    rv_of(((sfn / 2) % 4) as u8)
}

pub fn parse_1a(bits: &[u8], nrb: usize) -> Option<SiGrant> {
    let width = riv_bits(nrb * (nrb + 1) / 2);
    if bits.len() < 15 + width || bits[0] != 1 {
        return None;
    }
    let distributed = bits[1] == 1;
    let (first, count) = alloc::riv(field(bits, 2, width), nrb);
    let at = 2 + width;
    let (mcs, ndi, rv, tpc) =
        (field(bits, at, 5), bits[at + 8], field(bits, at + 9, 2), field(bits, at + 11, 2));
    let allocation = if distributed {
        let gap = if nrb >= 50 && ndi == 1 { Gap::Second } else { Gap::First };
        if first + count > alloc::n_vrb(nrb, gap) {
            return None;
        }
        Allocation::Distributed { first, count, gap }
    } else {
        if first + count > nrb {
            return None;
        }
        Allocation::Localized { first, count }
    };
    let tbs = alloc::tbs_1a(mcs, tpc & 1 == 1)?;
    Some(SiGrant { allocation, tbs, rv: rv as u8 })
}

pub fn parse_1c(bits: &[u8], nrb: usize, rv: u8) -> Option<SiGrant> {
    let wide = usize::from(nrb >= 50);
    let gap = if wide == 1 && bits.first() == Some(&1) { Gap::Second } else { Gap::First };
    let step = alloc::step_1c(nrb);
    let width = riv_bits({
        let n = alloc::n_vrb(nrb, Gap::First) / step;
        n * (n + 1) / 2
    });
    if bits.len() < wide + width + 5 {
        return None;
    }
    let units = alloc::n_vrb(nrb, gap) / step;
    let (first, count) = alloc::riv(field(bits, wide, width), units);
    if first + count > units {
        return None;
    }
    let allocation = Allocation::Distributed { first: first * step, count: count * step, gap };
    let tbs = alloc::tbs_1c(field(bits, wide + width, 5))?;
    Some(SiGrant { allocation, tbs, rv })
}

pub fn si_grant(soft: &[f32], nrb: usize, rv_1c: u8) -> Option<SiGrant> {
    let cces = soft.len() / CCE_BITS;
    let sizes = [dci_1a_bits(nrb), dci_1c_bits(nrb)];
    for level in [4, 8] {
        for start in (0..16.min(cces)).step_by(level) {
            if start + level > cces {
                continue;
            }
            let e = &soft[start * CCE_BITS..(start + level) * CCE_BITS];
            for (format, size) in sizes.iter().enumerate() {
                let bits = conv_decode(e, size + 16);
                if crc16_mask(&bits) != Some(SI_RNTI) {
                    continue;
                }
                let grant = match format {
                    0 => parse_1a(&bits[..*size], nrb),
                    _ => parse_1c(&bits[..*size], nrb, rv_1c),
                };
                if grant.is_some() {
                    return grant;
                }
            }
        }
    }
    None
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn format_1a_is_21_22_25_27_27_28_bits_across_the_six_bandwidths() {
        let sizes: Vec<usize> = [6, 15, 25, 50, 75, 100].iter().map(|&n| dci_1a_bits(n)).collect();
        assert_eq!(sizes, [21, 22, 25, 27, 27, 28]);
    }

    #[test]
    fn the_capture_cells_1a_grant_reads_seven_blocks_from_the_bottom() {
        let bits: Vec<u8> = "100010010110000010000000010".bytes().map(|b| b - b'0').collect();
        let g = parse_1a(&bits, 50).expect("a grant");
        assert_eq!(
            g,
            SiGrant { allocation: Allocation::Localized { first: 0, count: 7 }, tbs: 144, rv: 0 }
        );
    }

    #[test]
    fn format_1c_is_8_10_12_13_14_15_bits_across_the_six_bandwidths() {
        let sizes: Vec<usize> = [6, 15, 25, 50, 75, 100].iter().map(|&n| dci_1c_bits(n)).collect();
        assert_eq!(sizes, [8, 10, 12, 13, 14, 15]);
    }

    #[test]
    fn the_sib1_redundancy_versions_run_0_2_3_1_over_eighty_milliseconds() {
        assert_eq!([0, 2, 4, 6, 8].map(si_rv), [0, 2, 3, 1, 0]);
    }

    #[test]
    fn the_pcfich_of_cell_481_sits_every_quarter_of_a_10_mhz_carrier() {
        assert_eq!(pcfich_regs(50, 481), [486, 36, 186, 336]);
    }
}
