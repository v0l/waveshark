use super::coding::{conv_decode, crc16_mask, to_bytes};
use super::grid::{Demodulator, combine, estimate, soft_bits};
use super::{Mib, gold};
use common::C32;

const CODED: usize = 1920;
const PER_FRAME: usize = CODED / 4;
const PAYLOAD: usize = 24;

fn mask(ports: usize) -> u16 {
    match ports {
        1 => 0x0000,
        2 => 0xFFFF,
        _ => 0x5555,
    }
}

pub fn elements(pci: u16) -> Vec<(usize, usize)> {
    let reserved = usize::from(pci) % 3;
    (7..11)
        .flat_map(|l| (0..72).map(move |k| (l, k)))
        .filter(|&(l, k)| l > 8 || k % 3 != reserved)
        .collect()
}

pub struct Read {
    pub mib: Mib,
    pub snr_db: f32,
    pub rsrp_dbfs: f32,
    pub rsrq_db: f32,
}

pub fn read(demod: &mut Demodulator, y: &[C32], frame_start: usize, pci: u16) -> Option<Read> {
    let grid = demod.subframe(y, frame_start, 6)?;
    let res = elements(pci);
    let scrambler = gold::sequence(CODED, u32::from(pci));
    for ports in [1, 2, 4] {
        let ch = estimate(&grid, 0, pci, ports);
        let soft = soft_bits(&combine(&grid, &ch, &res, ports));
        for frame in 0..4 {
            let mut e = vec![0f32; CODED];
            let slot = &mut e[frame * PER_FRAME..(frame + 1) * PER_FRAME];
            for ((v, s), c) in slot.iter_mut().zip(&soft).zip(&scrambler[frame * PER_FRAME..]) {
                *v = if *c == 1 { -s } else { *s };
            }
            let bits = conv_decode(&e, PAYLOAD + 16);
            if crc16_mask(&bits) != Some(mask(ports)) {
                continue;
            }
            let mib = Mib::unpack(&to_bytes(&bits[..PAYLOAD]), frame as u16, ports)?;
            return Some(Read {
                mib,
                snr_db: ch.snr_db,
                rsrp_dbfs: ch.rsrp_dbfs(),
                rsrq_db: ch.rsrq_db(),
            });
        }
    }
    None
}
