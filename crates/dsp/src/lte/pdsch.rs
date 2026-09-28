use super::control::SiGrant;
use super::grid::{Channel, Grid, combine, descramble, is_crs, soft_bits};
use crate::lte_turbo;

const TURBO_ITERATIONS: usize = 8;

pub fn elements(
    nrb: usize,
    grant: &SiGrant,
    subframe: usize,
    first_symbol: usize,
    pci: u16,
    ports: usize,
) -> Vec<(usize, usize)> {
    let nsc = nrb * 12;
    let centre = nsc / 2 - 36..nsc / 2 + 36;
    let slots = [grant.allocation.prbs(nrb, false), grant.allocation.prbs(nrb, true)];
    (first_symbol..14)
        .flat_map(|l| (0..nsc).map(move |k| (l, k)))
        .filter(|&(l, k)| {
            let sync = matches!(subframe, 0 | 5) && matches!(l, 5 | 6);
            let pbch = subframe == 0 && (7..11).contains(&l);
            slots[l / 7].contains(&(k / 12))
                && !is_crs(l, k, pci, ports)
                && !((sync || pbch) && centre.contains(&k))
        })
        .collect()
}

pub fn read_si(
    grid: &Grid,
    ch: &Channel,
    grant: &SiGrant,
    subframe: usize,
    first_symbol: usize,
    pci: u16,
    ports: usize,
) -> Option<Vec<u8>> {
    let tbs = grant.tbs;
    let k = tbs + 24;
    let res = elements(grid.nrb, grant, subframe, first_symbol, pci, ports);
    let mut soft = soft_bits(&combine(grid, ch, &res, ports));
    let c_init = u32::from(super::SI_RNTI) << 14 | (subframe as u32) << 9 | u32::from(pci);
    descramble(&mut soft, c_init);
    let d = lte_turbo::rate_dematch_rv(&soft, k + 4, usize::from(grant.rv));
    let bits = lte_turbo::decode(&d, k, TURBO_ITERATIONS)?;
    let bytes = super::coding::to_bytes(&bits);
    (lte_turbo::crc24a(&bytes) == 0).then(|| bytes[..tbs / 8].to_vec())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_seven_block_grant_at_the_bottom_of_10_mhz_holds_840_elements() {
        let g = SiGrant {
            allocation: super::super::alloc::Allocation::Localized { first: 0, count: 7 },
            tbs: 144,
            rv: 0,
        };
        assert_eq!(elements(50, &g, 5, 3, 473, 2).len(), 840);
    }
}
