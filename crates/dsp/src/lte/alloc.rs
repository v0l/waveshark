#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Gap {
    First,
    Second,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Allocation {
    Localized { first: usize, count: usize },
    Distributed { first: usize, count: usize, gap: Gap },
}

pub fn rbg_size(nrb: usize) -> usize {
    match nrb {
        0..=10 => 1,
        11..=26 => 2,
        27..=63 => 3,
        _ => 4,
    }
}

pub fn n_gap(nrb: usize, gap: Gap) -> usize {
    match (nrb, gap) {
        (0..=10, _) => nrb.div_ceil(2),
        (11, _) => 4,
        (12..=19, _) => 8,
        (20..=26, _) => 12,
        (27..=44, _) => 18,
        (45..=49, _) => 27,
        (50..=63, Gap::First) => 27,
        (50..=63, Gap::Second) => 9,
        (64..=79, Gap::First) => 32,
        (64..=79, Gap::Second) => 16,
        (_, Gap::First) => 48,
        (_, Gap::Second) => 16,
    }
}

pub fn n_vrb(nrb: usize, gap: Gap) -> usize {
    let g = n_gap(nrb, gap);
    match gap {
        Gap::First => 2 * g.min(nrb - g),
        Gap::Second => nrb / (2 * g) * 2 * g,
    }
}

pub fn step_1c(nrb: usize) -> usize {
    if nrb < 50 { 2 } else { 4 }
}

pub fn riv(value: usize, n: usize) -> (usize, usize) {
    let (len, start) = (value / n + 1, value % n);
    if len + start <= n { (start, len) } else { (n - 1 - start, n + 2 - len) }
}

fn distributed_prb(nrb: usize, gap: Gap, vrb: usize, odd_slot: bool) -> usize {
    let g = n_gap(nrb, gap);
    let unit = match gap {
        Gap::First => n_vrb(nrb, gap),
        Gap::Second => 2 * g,
    };
    let p = rbg_size(nrb);
    let rows = unit.div_ceil(4 * p) * p;
    let nulls = 4 * rows - unit;
    let base = unit * (vrb / unit);
    let v = vrb % unit;
    let first = 2 * rows * (v % 2) + v / 2 + base;
    let second = rows * (v % 4) + v / 4 + base;
    let mut prb = if nulls != 0 && v >= unit - nulls {
        if v % 2 == 1 { first - rows } else { first - rows + nulls / 2 }
    } else if nulls != 0 && v % 4 >= 2 {
        second - nulls / 2
    } else {
        second
    };
    if odd_slot {
        prb = (prb + unit / 2) % unit + base;
    }
    if prb >= unit / 2 { prb + g - unit / 2 } else { prb }
}

impl Allocation {
    pub fn prbs(&self, nrb: usize, odd_slot: bool) -> Vec<usize> {
        match *self {
            Self::Localized { first, count } => (first..first + count).collect(),
            Self::Distributed { first, count, gap } => {
                (first..first + count).map(|v| distributed_prb(nrb, gap, v, odd_slot)).collect()
            }
        }
    }
}

const TBS_1A: [(usize, usize); 27] = [
    (32, 56),
    (56, 88),
    (72, 144),
    (104, 176),
    (120, 208),
    (144, 224),
    (176, 256),
    (224, 328),
    (256, 392),
    (296, 456),
    (328, 504),
    (376, 584),
    (440, 680),
    (488, 744),
    (552, 840),
    (600, 904),
    (632, 968),
    (696, 1064),
    (776, 1160),
    (840, 1288),
    (904, 1384),
    (1000, 1480),
    (1064, 1608),
    (1128, 1736),
    (1192, 1800),
    (1256, 1864),
    (1480, 2216),
];

const TBS_1C: [usize; 32] = [
    40, 56, 72, 120, 136, 144, 176, 208, 224, 256, 280, 296, 328, 336, 392, 488, 552, 600, 632,
    696, 776, 808, 936, 1032, 1096, 1256, 1384, 1544, 1736, 2024, 2280, 2536,
];

pub fn tbs_1a(mcs: usize, three_prbs: bool) -> Option<usize> {
    TBS_1A.get(mcs).map(|&(two, three)| if three_prbs { three } else { two })
}

pub fn tbs_1c(itbs: usize) -> Option<usize> {
    TBS_1C.get(itbs).copied()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_distributed_10_mhz_allocation_spreads_over_46_blocks_and_hops_between_slots() {
        assert_eq!(n_vrb(50, Gap::First), 46);
        assert_eq!(n_vrb(50, Gap::Second), 36);
        let a = Allocation::Distributed { first: 0, count: 4, gap: Gap::First };
        let (even, odd) = (a.prbs(50, false), a.prbs(50, true));
        assert_eq!(even.len(), 4);
        assert_ne!(even, odd, "a distributed block moves in the second slot");
        let mut all: Vec<usize> =
            (0..46).map(|v| distributed_prb(50, Gap::First, v, false)).collect();
        all.sort_unstable();
        all.dedup();
        assert_eq!(all.len(), 46, "every virtual block lands on its own physical one");
    }

    #[test]
    fn the_second_gap_keeps_each_unit_of_18_on_its_own_18_blocks_and_hops_9() {
        for unit in 0..2 {
            let a = Allocation::Distributed { first: 18 * unit, count: 18, gap: Gap::Second };
            let (mut even, odd) = (a.prbs(50, false), a.prbs(50, true));
            let hopped: Vec<usize> =
                even.iter().map(|p| (p - 18 * unit + 9) % 18 + 18 * unit).collect();
            assert_eq!(odd, hopped);
            even.sort_unstable();
            assert_eq!(even, (18 * unit..18 * unit + 18).collect::<Vec<_>>());
        }
    }
}
