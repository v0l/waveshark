//! TETRA downlink: pi/4-DQPSK at 18k symbols a second in a 25 kHz channel.
//!
//! The physical and lower MAC layers only. What leaves here is a logical
//! channel block whose scrambling, interleaving, puncturing, Viterbi and CRC
//! have all been run; what the block *means*, the network identity in a SYNC
//! PDU, the SYSINFO broadcast, is `decode::tetra`, the same split as M17.
//!
//! # Why differential detection
//!
//! pi/4-DQPSK carries each dibit as a phase *step* of an odd multiple of
//! pi/4, so a receiver never needs the absolute carrier phase: the product
//! `z[k] * conj(z[k-1])` reads the step directly, and a frequency error
//! shows up as a constant added to every step rather than a rotation that
//! has to be chased. A base station downlink transmits continuously, so
//! after one synchronization training sequence has been found the receiver
//! walks slot to slot, re-measuring its timing and that constant on every
//! burst's own training sequence.
//!
//! # The layout of a downlink slot
//!
//! 255 symbols, 510 bits, 85/6 ms. Two shapes matter here (EN 300 392-2
//! clause 9.4.4.2): the *sync* burst, whose 120 bit first block is the BSCH
//! readable before anything about the cell is known, and the *normal* burst,
//! two 216 bit blocks whose scrambling needs the identity the BSCH carries.
//! Which training sequence a normal burst uses says whether its two halves
//! are one full-slot channel or two half-slot channels.

pub mod coding;
pub mod speech;

use crate::fir::Fir;
use crate::m17::rrc_taps;
use common::C32;

/// Symbols a second. Two bits to a symbol: 36 kbit/s gross.
pub const BAUD: f64 = 18_000.0;

/// The channel raster the carriers sit on.
pub const CHANNEL_WIDTH_HZ: f64 = 25_000.0;

/// Occupied bandwidth: root raised cosine with alpha 0.35.
pub const OCCUPIED_HZ: f64 = BAUD * 1.35;
const RRC_ALPHA: f64 = 0.35;

/// One timeslot. Four to a frame, 18 frames to a multiframe.
pub const SLOT_SYMBOLS: usize = 255;
pub const SLOT_BITS: usize = 510;

/// 9.4.4.3.2/4: the training sequences that name a downlink burst, as bits.
pub const TRAIN_NORMAL_1: [u8; 22] =
    [1, 1, 0, 1, 0, 0, 0, 0, 1, 1, 1, 0, 1, 0, 0, 1, 1, 1, 0, 1, 0, 0];
pub const TRAIN_NORMAL_2: [u8; 22] =
    [0, 1, 1, 1, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1, 1, 0, 1, 1, 1, 1, 0];
pub const TRAIN_SYNC: [u8; 38] = [
    1, 1, 0, 0, 0, 0, 0, 1, 1, 0, 0, 1, 1, 1, 0, 0, 1, 1, 1, 0, 1, 0, 0, 1, 1, 1, 0, 0, 0, 0,
    0, 1, 1, 0, 0, 1, 1, 1,
];

/// Bit offsets inside a sync burst: block 1, sync training, broadcast, block 2.
pub const SB_BLK1: usize = 94;
pub const SB_TRAIN: usize = 214;
pub const SB_BB: usize = 252;
pub const SB_BLK2: usize = 282;

/// And inside a normal burst.
pub const NDB_BLK1: usize = 14;
pub const NDB_BB1: usize = 230;
pub const NDB_TRAIN: usize = 244;
pub const NDB_BB2: usize = 266;
pub const NDB_BLK2: usize = 282;

/// The European downlink halves of the TETRA allocations: base stations at
/// 390 to 400 MHz for the emergency networks and 420 to 430 for the
/// commercial ones, handsets 10 MHz lower in each. Knowledge about the
/// world rather than about this receiver, the same kind AIS keeps.
pub fn is_downlink_band(hz: f64) -> bool {
    (390.0e6..400.0e6).contains(&hz) || (420.0e6..430.0e6).contains(&hz)
}

/// Which training sequence a burst carried.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BurstKind {
    /// Sync burst: BSCH in block 1, half-slot channel in block 2.
    Sync,
    /// Normal burst, training sequence 1: both blocks are one full slot.
    Normal1,
    /// Normal burst, training sequence 2: two independent half slots.
    Normal2,
}

/// One slot off the air, sliced to bits.
#[derive(Clone, Debug)]
pub struct Burst {
    pub kind: BurstKind,
    pub bits: [u8; SLOT_BITS],
    /// Training sequence correlation, 0 to 1.
    pub quality: f32,
    /// Carrier offset the training sequence measured, in hertz.
    pub freq_offset_hz: f32,
    /// Slot counter, so a listener can tell adjacent slots from a gap the
    /// demodulator lost.
    pub slot: u64,
    pub start_sample: u64,
}

/// A dibit as the phase step it transmits, in units of pi/4.
fn dibit_step(b0: u8, b1: u8) -> i8 {
    match (b0, b1) {
        (0, 0) => 1,
        (0, 1) => -1,
        (1, 0) => 3,
        _ => -3,
    }
}

/// Training bits as the complex steps a clean burst would show.
fn train_steps(bits: &[u8]) -> Vec<C32> {
    bits.chunks_exact(2)
        .map(|d| {
            let a = std::f32::consts::FRAC_PI_4 * dibit_step(d[0], d[1]) as f32;
            C32::new(a.cos(), a.sin())
        })
        .collect()
}

#[derive(Clone, Copy, Debug)]
pub struct TetraConfig {
    /// Training correlation below which a hunt match is not believed.
    pub min_acquire: f32,
    /// The lower bar a slot has to meet to keep an existing lock.
    pub min_track: f32,
    /// Slots that may fail that bar before the lock is dropped.
    pub max_misses: u32,
}

impl Default for TetraConfig {
    fn default() -> Self {
        Self { min_acquire: 0.72, min_track: 0.55, max_misses: 8 }
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct TetraStats {
    pub bursts: u64,
    pub acquisitions: u64,
    pub lost: u64,
}

struct Lock {
    /// Absolute (origin-relative) sample position of the next slot's first
    /// symbol instant.
    next: f64,
    /// Residual carrier, radians per symbol, measured on training sequences.
    drift: f32,
    misses: u32,
    slot: u64,
}

/// Which way the constellation turns.
///
/// Whether a positive phase step arrives as a positive angle depends on the
/// I/Q convention of everything in front of this demodulator, not on the
/// transmitter, the same ambiguity M17 resolves by decoding both ways. Here
/// the synchronization training sequence answers it: correlated against the
/// ideal steps and against their mirror image, only the true sense matches,
/// so the hunt tries both and the winner is kept for the lock's lifetime.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Sense {
    Direct,
    Mirrored,
}

/// The demodulator: complex baseband in, bursts out.
pub struct TetraDemod {
    cfg: TetraConfig,
    sps: f64,
    rrc: Fir,
    buf: Vec<C32>,
    /// Absolute sample index of `buf[0]`.
    origin: u64,
    /// Where the sync hunt has searched to, as a buffer index.
    hunt: usize,
    lock: Option<Lock>,
    sense: Sense,
    stats: TetraStats,
    y_steps: Vec<C32>,
    n_steps: Vec<C32>,
    p_steps: Vec<C32>,
}

impl TetraDemod {
    pub fn new(rate: f64, cfg: TetraConfig) -> Self {
        let sps = rate / BAUD;
        Self {
            cfg,
            sps,
            rrc: Fir::new(rrc_taps(sps.max(1.0), RRC_ALPHA, 8)),
            buf: Vec::new(),
            origin: 0,
            hunt: 0,
            lock: None,
            sense: Sense::Direct,
            stats: TetraStats::default(),
            y_steps: train_steps(&TRAIN_SYNC),
            n_steps: train_steps(&TRAIN_NORMAL_1),
            p_steps: train_steps(&TRAIN_NORMAL_2),
        }
    }

    pub fn stats(&self) -> TetraStats {
        self.stats
    }

    pub fn reset(&mut self) {
        self.rrc.reset();
        self.buf.clear();
        self.origin = 0;
        self.hunt = 0;
        self.lock = None;
        self.sense = Sense::Direct;
    }

    /// A matched-filtered sample at a fractional position, linearly
    /// interpolated. At the extractor's floor of 25 kS/s that is 1.4 samples
    /// a symbol, which interpolation reads well enough for a training
    /// correlation to say where the symbols are.
    fn at(&self, pos: f64) -> C32 {
        if pos <= 0.0 {
            return self.buf.first().copied().unwrap_or_default();
        }
        let i = pos.floor() as usize;
        if i + 1 >= self.buf.len() {
            return self.buf.last().copied().unwrap_or_default();
        }
        let f = (pos - i as f64) as f32;
        self.buf[i] * (1.0 - f) + self.buf[i + 1] * f
    }

    /// The phase step onto the symbol at `pos`, as a unit-ish phasor, read
    /// in whichever sense the training sequence proved.
    fn step(&self, pos: f64) -> C32 {
        let d = self.at(pos) * self.at(pos - self.sps).conj();
        match self.sense {
            Sense::Direct => d,
            Sense::Mirrored => d.conj(),
        }
    }

    /// Correlate the steps at `pos` against a training sequence.
    ///
    /// Returns (quality, mean rotation): the rotation every step shares is a
    /// carrier offset, and taking the magnitude first makes the quality
    /// blind to it, which is what lets acquisition run before any frequency
    /// estimate exists.
    fn correlate(&self, pos: f64, steps: &[C32]) -> (f32, f32) {
        let mut acc = C32::default();
        let mut power = 0.0f32;
        for (k, s) in steps.iter().enumerate() {
            let d = self.step(pos + k as f64 * self.sps);
            acc += d * s.conj();
            power += d.norm();
        }
        if power < 1e-12 {
            return (0.0, 0.0);
        }
        (acc.norm() / power, acc.arg())
    }

    /// The best training match for a slot whose first symbol instant is
    /// near `start`: refined position, kind, quality, rotation.
    fn best_at(&self, start: f64, span: f64) -> Option<(f64, BurstKind, f32, f32)> {
        let mut best: Option<(f64, BurstKind, f32, f32)> = None;
        let step = (self.sps / 8.0).max(0.125);
        let mut off = -span;
        while off <= span {
            let t = start + off;
            for (kind, steps, at_sym) in [
                (BurstKind::Sync, &self.y_steps, SB_TRAIN / 2),
                (BurstKind::Normal1, &self.n_steps, NDB_TRAIN / 2),
                (BurstKind::Normal2, &self.p_steps, NDB_TRAIN / 2),
            ] {
                let (q, rot) = self.correlate(t + at_sym as f64 * self.sps, steps);
                if best.map_or(true, |b| q > b.2) {
                    best = Some((t, kind, q, rot));
                }
            }
            off += step;
        }
        best
    }

    /// Slice the slot at `start` to bits, rotating each step back by the
    /// carrier drift the training sequence measured.
    fn slice(&self, start: f64, drift: f32) -> [u8; SLOT_BITS] {
        let undo = C32::new(drift.cos(), -drift.sin());
        let mut bits = [0u8; SLOT_BITS];
        for k in 0..SLOT_SYMBOLS {
            let d = self.step(start + k as f64 * self.sps) * undo;
            let phi = d.arg();
            bits[2 * k] = u8::from(phi.abs() > std::f32::consts::FRAC_PI_2);
            bits[2 * k + 1] = u8::from(phi < 0.0);
        }
        bits
    }

    /// Feed a block of complex baseband, appending completed bursts.
    pub fn process(&mut self, iq: &[C32], out: &mut Vec<Burst>) {
        self.rrc.process(iq, &mut self.buf);
        let slot_span = SLOT_SYMBOLS as f64 * self.sps;
        // A whole slot, plus the margin the refinement searches over.
        let need = slot_span + 4.0 * self.sps;

        loop {
            if let Some(lock) = self.lock.take() {
                if lock.next + need >= self.buf.len() as f64 {
                    self.lock = Some(lock);
                    break;
                }
                self.lock = self.read_slot(lock, out);
                continue;
            }
            if !self.acquire() {
                break;
            }
        }

        // Nothing behind the hunt position or the lock will be read again.
        let keep = match &self.lock {
            // A slot's training sits mid-burst, so hold one slot of history.
            Some(l) => (l.next - 2.0 * self.sps).min(self.hunt as f64) as usize,
            None => self.hunt,
        };
        if keep > 1 << 14 {
            self.buf.drain(..keep);
            self.origin += keep as u64;
            self.hunt -= keep.min(self.hunt);
            if let Some(l) = &mut self.lock {
                l.next -= keep as f64;
            }
        }
    }

    /// Hunt for a synchronization training sequence. True when a lock was
    /// made and the loop should continue; false when the buffer is spent.
    fn acquire(&mut self) -> bool {
        let step = (self.sps / 2.0).max(0.5);
        // The training sequence sits 107 symbols into its burst, and the
        // whole burst has to be in hand once it is found.
        let lead = (SB_TRAIN / 2 + 1) as f64 * self.sps;
        let tail = (SLOT_SYMBOLS - SB_TRAIN / 2) as f64 * self.sps + 4.0 * self.sps;
        let mut pos = (self.hunt as f64).max(lead);
        loop {
            if pos + tail + self.y_steps.len() as f64 * self.sps >= self.buf.len() as f64 {
                self.hunt = pos as usize;
                return false;
            }
            let mut q = 0.0;
            for sense in [Sense::Direct, Sense::Mirrored] {
                self.sense = sense;
                q = self.correlate(pos, &self.y_steps).0;
                if q > self.cfg.min_acquire {
                    break;
                }
            }
            if q > self.cfg.min_acquire {
                // Walk to the top of the peak before trusting the position.
                if let Some((t, BurstKind::Sync, q, rot)) =
                    self.best_at(pos - lead, self.sps)
                {
                    if q > self.cfg.min_acquire {
                        self.stats.acquisitions += 1;
                        self.hunt = (t + self.sps) as usize;
                        self.lock = Some(Lock {
                            next: t,
                            drift: rot,
                            misses: 0,
                            slot: 0,
                        });
                        return true;
                    }
                }
            }
            pos += step;
            self.hunt = pos as usize;
        }
    }

    /// Read the slot a lock says is next, returning the advanced lock.
    fn read_slot(&mut self, mut lock: Lock, out: &mut Vec<Burst>) -> Option<Lock> {
        let slot_span = SLOT_SYMBOLS as f64 * self.sps;
        let got = self.best_at(lock.next, self.sps / 2.0);
        let Some((t, kind, q, rot)) = got else {
            return self.miss(lock);
        };
        if q < self.cfg.min_track {
            return self.miss(lock);
        }
        // The training measures timing and carrier fresh on every slot, so
        // neither has to be held for longer than 14 ms.
        lock.drift = 0.75 * lock.drift + 0.25 * rot;
        let bits = self.slice(t, lock.drift);
        self.stats.bursts += 1;
        out.push(Burst {
            kind,
            bits,
            quality: q,
            freq_offset_hz: lock.drift * BAUD as f32 / std::f32::consts::TAU,
            slot: lock.slot,
            start_sample: self.origin + t.max(0.0) as u64,
        });
        lock.misses = 0;
        lock.slot += 1;
        lock.next = t + slot_span;
        self.hunt = lock.next as usize;
        Some(lock)
    }

    fn miss(&mut self, mut lock: Lock) -> Option<Lock> {
        lock.misses += 1;
        lock.slot += 1;
        lock.next += SLOT_SYMBOLS as f64 * self.sps;
        if lock.misses > self.cfg.max_misses {
            self.stats.lost += 1;
            self.hunt = lock.next as usize;
            return None;
        }
        Some(lock)
    }
}

/// The logical channel a decoded block belongs to.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Lchan {
    /// Broadcast synchronization: the SYNC PDU, 60 bits.
    Bsch,
    /// Half-slot signalling, 124 bits, which on frame 18 carries SYSINFO.
    SchHd,
    /// Full-slot signalling, 268 bits.
    SchF,
    /// The access assign field every downlink burst carries in its
    /// broadcast block, 14 bits: what this slot is being used for.
    Aach,
}

/// TDMA time as the SYNC PDU counts it: slots 1-4, frames 1-18, multiframes
/// 1-60.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct TdmaTime {
    pub tn: u8,
    pub frame: u8,
    pub multiframe: u8,
}

impl TdmaTime {
    pub fn advance(&mut self, slots: u64) {
        for _ in 0..slots {
            self.tn += 1;
            if self.tn > 4 {
                self.tn = 1;
                self.frame += 1;
                if self.frame > 18 {
                    self.frame = 1;
                    self.multiframe += 1;
                    if self.multiframe > 60 {
                        self.multiframe = 1;
                    }
                }
            }
        }
    }
}

/// One logical channel block whose FEC ran and whose CRC checked.
#[derive(Clone, Debug)]
pub struct Block {
    pub lchan: Lchan,
    pub bits: Vec<u8>,
    /// Where the cell believes itself to be in its multiframe, when a SYNC
    /// PDU has said.
    pub time: Option<TdmaTime>,
    pub slot: u64,
}

/// The identity a BSCH announces, and the scrambler it implies.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Cell {
    pub mcc: u16,
    pub mnc: u16,
    pub colour: u8,
    pub scramb: u32,
}

fn bits_to_u32(bits: &[u8]) -> u32 {
    bits.iter().fold(0, |acc, &b| acc << 1 | u32::from(b))
}

/// Lower MAC state: bursts in, believed blocks out.
///
/// The one piece of state that matters is the cell identity, because every
/// channel except the BSCH is scrambled with it: until one SYNC PDU has
/// decoded, normal bursts are noise by design.
#[derive(Default)]
pub struct TetraRx {
    pub cell: Option<Cell>,
    time: Option<(TdmaTime, u64)>,
    pub blocks: u64,
    pub failed: u64,
}

impl TetraRx {
    pub fn new() -> Self {
        Self::default()
    }

    /// Seed the cell identity and clock without decoding a SYNC burst, for
    /// tests and replay that start mid-stream.
    pub fn seed(&mut self, cell: Cell, time: TdmaTime, slot: u64) {
        self.cell = Some(cell);
        self.time = Some((time, slot));
    }

    /// The cell time at a given slot counter, advanced from the last SYNC.
    pub fn time_at(&self, slot: u64) -> Option<TdmaTime> {
        let (t0, s0) = self.time?;
        let mut t = t0;
        t.advance(slot.saturating_sub(s0));
        Some(t)
    }

    fn take(&mut self, ok: Option<Vec<u8>>, lchan: Lchan, slot: u64, out: &mut Vec<Block>) {
        match ok {
            Some(bits) => {
                self.blocks += 1;
                out.push(Block { lchan, bits, time: self.time_at(slot), slot });
            }
            None => self.failed += 1,
        }
    }

    /// The broadcast block: 30 bits of every downlink burst, scrambled with
    /// the cell's own sequence and block coded rather than convolutionally,
    /// since it has to be read from a single burst.
    fn aach(&mut self, burst: &Burst, out: &mut Vec<Block>) {
        let Some(cell) = self.cell else { return };
        let mut bb = [0u8; 30];
        match burst.kind {
            BurstKind::Sync => bb.copy_from_slice(&burst.bits[SB_BB..SB_BLK2]),
            BurstKind::Normal1 | BurstKind::Normal2 => {
                bb[..14].copy_from_slice(&burst.bits[NDB_BB1..NDB_TRAIN]);
                bb[14..].copy_from_slice(&burst.bits[NDB_BB2..NDB_BLK2]);
            }
        }
        coding::scramble(cell.scramb, &mut bb);
        let word = bits_to_u32(&bb);
        let (info, distance) = coding::rm3014_decode(word);
        // The code corrects three errors, but a burst sliced from noise
        // lands within three of some codeword one time in fourteen, and a
        // wrong field says a traffic channel opened. A burst worth reading
        // decodes clean or nearly so.
        if distance > 1 {
            self.failed += 1;
            return;
        }
        let bits: Vec<u8> = (0..14).map(|i| ((info >> (13 - i)) & 1) as u8).collect();
        self.take(Some(bits), Lchan::Aach, burst.slot, out);
    }

    pub fn push(&mut self, burst: &Burst, out: &mut Vec<Block>) {
        self.aach(burst, out);
        match burst.kind {
            BurstKind::Sync => {
                let sb1 = coding::decode_block(
                    &coding::BLK_BSCH,
                    coding::SCRAMB_INIT,
                    &burst.bits[SB_BLK1..SB_TRAIN],
                );
                if let Some(bits) = &sb1 {
                    // The SYNC PDU carries the scrambler for everything else
                    // and the cell's own clock; both are state here, whatever
                    // the upper layer makes of the rest.
                    let colour = bits_to_u32(&bits[4..10]) as u8;
                    let tn = bits_to_u32(&bits[10..12]) as u8 + 1;
                    let frame = bits_to_u32(&bits[12..17]) as u8;
                    let multiframe = bits_to_u32(&bits[17..23]) as u8;
                    let mcc = bits_to_u32(&bits[31..41]) as u16;
                    let mnc = bits_to_u32(&bits[41..55]) as u16;
                    self.cell = Some(Cell {
                        mcc,
                        mnc,
                        colour,
                        scramb: coding::scramb_init(mcc, mnc, colour),
                    });
                    self.time = Some((TdmaTime { tn, frame, multiframe }, burst.slot));
                }
                self.take(sb1, Lchan::Bsch, burst.slot, out);
                if let Some(cell) = self.cell {
                    let sb2 = coding::decode_block(
                        &coding::BLK_HALF,
                        cell.scramb,
                        &burst.bits[SB_BLK2..SB_BLK2 + 216],
                    );
                    self.take(sb2, Lchan::SchHd, burst.slot, out);
                }
            }
            BurstKind::Normal1 => {
                let Some(cell) = self.cell else { return };
                let mut whole = Vec::with_capacity(432);
                whole.extend_from_slice(&burst.bits[NDB_BLK1..NDB_BB1]);
                whole.extend_from_slice(&burst.bits[NDB_BLK2..NDB_BLK2 + 216]);
                let blk = coding::decode_block(&coding::BLK_FULL, cell.scramb, &whole);
                self.take(blk, Lchan::SchF, burst.slot, out);
            }
            BurstKind::Normal2 => {
                let Some(cell) = self.cell else { return };
                for range in [NDB_BLK1..NDB_BB1, NDB_BLK2..NDB_BLK2 + 216] {
                    let blk = coding::decode_block(
                        &coding::BLK_HALF,
                        cell.scramb,
                        &burst.bits[range],
                    );
                    self.take(blk, Lchan::SchHd, burst.slot, out);
                }
            }
        }
    }
}

/// Test and fixture support: build the on-air bits of downlink bursts.
pub mod synth {
    use super::*;

    /// 9.4.4.3.1: the frequency correction field.
    fn f_bits() -> [u8; 80] {
        let mut f = [0u8; 80];
        for i in 0..8 {
            f[i] = 1;
            f[72 + i] = 1;
        }
        f
    }

    /// Normal training sequence 3, both ends of a continuous burst.
    const TRAIN_Q: [u8; 22] =
        [1, 0, 1, 1, 0, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 0, 1, 0, 1, 1, 0, 1];

    /// 9.4.4.2.6: a synchronization continuous downlink burst.
    ///
    /// The phase adjustment pairs are transmitted as zeros here; nothing in
    /// the receiver reads them.
    pub fn sync_burst(sb1: &[u8], bb: &[u8], bkn2: &[u8]) -> [u8; SLOT_BITS] {
        assert_eq!((sb1.len(), bb.len(), bkn2.len()), (120, 30, 216));
        let mut b = [0u8; SLOT_BITS];
        b[..12].copy_from_slice(&TRAIN_Q[10..]);
        b[14..94].copy_from_slice(&f_bits());
        b[SB_BLK1..SB_TRAIN].copy_from_slice(sb1);
        b[SB_TRAIN..SB_BB].copy_from_slice(&TRAIN_SYNC);
        b[SB_BB..SB_BLK2].copy_from_slice(bb);
        b[SB_BLK2..498].copy_from_slice(bkn2);
        b[500..].copy_from_slice(&TRAIN_Q[..10]);
        b
    }

    /// 9.4.4.2.5: a normal continuous downlink burst.
    pub fn normal_burst(bkn1: &[u8], bb: &[u8], bkn2: &[u8], two_half: bool) -> [u8; SLOT_BITS] {
        assert_eq!((bkn1.len(), bb.len(), bkn2.len()), (216, 30, 216));
        let mut b = [0u8; SLOT_BITS];
        b[..12].copy_from_slice(&TRAIN_Q[10..]);
        b[NDB_BLK1..NDB_BB1].copy_from_slice(bkn1);
        b[NDB_BB1..NDB_TRAIN].copy_from_slice(&bb[..14]);
        b[NDB_TRAIN..NDB_BB2]
            .copy_from_slice(if two_half { &TRAIN_NORMAL_2 } else { &TRAIN_NORMAL_1 });
        b[NDB_BB2..NDB_BLK2].copy_from_slice(&bb[14..]);
        b[NDB_BLK2..498].copy_from_slice(bkn2);
        b[500..].copy_from_slice(&TRAIN_Q[..10]);
        b
    }

    /// Key burst bits onto a carrier: pi/4-DQPSK, one phase step a symbol,
    /// rectangular pulses at `rate` with `offset_hz` of carrier error. Real
    /// shaping is root raised cosine; rectangular keying is wider but reads
    /// identically through a differential detector, and what the tests need
    /// is bits on the air, not a spectrum mask.
    pub fn modulate(bits: &[u8], rate: f64, offset_hz: f64) -> Vec<C32> {
        let sps = rate / BAUD;
        let mut phase = 0.0f64;
        let mut out = Vec::new();
        let mut emitted = 0usize;
        for (k, d) in bits.chunks_exact(2).enumerate() {
            phase += std::f64::consts::FRAC_PI_4 * f64::from(dibit_step(d[0], d[1]));
            let until = ((k + 1) as f64 * sps).round() as usize;
            while emitted < until {
                let t = emitted as f64 / rate;
                let a = phase + std::f64::consts::TAU * offset_hz * t;
                out.push(C32::new(a.cos() as f32, a.sin() as f32));
                emitted += 1;
            }
        }
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A believable SYNC PDU: Ireland's MCC, a made-up MNC and colour.
    fn sync_pdu(mcc: u16, mnc: u16, colour: u8, tn: u8, frame: u8, mn: u8) -> Vec<u8> {
        let mut bits = vec![0u8; 60];
        let put = |bits: &mut Vec<u8>, at: usize, n: usize, v: u32| {
            for i in 0..n {
                bits[at + i] = ((v >> (n - 1 - i)) & 1) as u8;
            }
        };
        put(&mut bits, 4, 6, colour.into());
        put(&mut bits, 10, 2, (tn - 1).into());
        put(&mut bits, 12, 5, frame.into());
        put(&mut bits, 17, 6, mn.into());
        put(&mut bits, 31, 10, mcc.into());
        put(&mut bits, 41, 14, mnc.into());
        bits
    }

    /// A downlink of sync bursts with the fields above, at `rate`.
    fn downlink(rate: f64, offset_hz: f64, slots: usize) -> Vec<C32> {
        let pdu = sync_pdu(272, 91, 3, 1, 18, 7);
        let sb1 = coding::encode_block(&coding::BLK_BSCH, coding::SCRAMB_INIT, &pdu);
        let scramb = coding::scramb_init(272, 91, 3);
        let half: Vec<u8> = (0..124).map(|i| ((i * 3) % 5 < 2) as u8).collect();
        let bkn2 = coding::encode_block(&coding::BLK_HALF, scramb, &half);
        let burst = synth::sync_burst(&sb1, &[0; 30], &bkn2);
        let mut bits = Vec::new();
        for _ in 0..slots {
            bits.extend_from_slice(&burst);
        }
        synth::modulate(&bits, rate, offset_hz)
    }

    fn run(rate: f64, offset_hz: f64) -> (Vec<Burst>, Vec<Block>, TetraRx) {
        let iq = downlink(rate, offset_hz, 12);
        let mut demod = TetraDemod::new(rate, TetraConfig::default());
        let mut rx = TetraRx::new();
        let (mut bursts, mut blocks) = (Vec::new(), Vec::new());
        for chunk in iq.chunks(4096) {
            let mut got = Vec::new();
            demod.process(chunk, &mut got);
            for b in &got {
                rx.push(b, &mut blocks);
            }
            bursts.extend(got);
        }
        (bursts, blocks, rx)
    }

    #[test]
    fn a_clean_downlink_is_read_slot_by_slot() {
        let (bursts, blocks, rx) = run(72_000.0, 0.0);
        assert!(bursts.len() >= 10, "{} bursts of 12 slots", bursts.len());
        assert!(bursts.iter().all(|b| b.kind == BurstKind::Sync));
        let cell = rx.cell.expect("no SYNC PDU decoded");
        assert_eq!((cell.mcc, cell.mnc, cell.colour), (272, 91, 3));
        // Both halves of the burst decode: the BSCH under the fixed
        // scrambler, and block 2 under the one the BSCH announced.
        assert!(blocks.iter().any(|b| b.lchan == Lchan::Bsch));
        assert!(blocks.iter().any(|b| b.lchan == Lchan::SchHd));
        let t = blocks.iter().find_map(|b| b.time).expect("no cell time");
        assert_eq!((t.tn, t.frame, t.multiframe), (1, 18, 7));
    }

    #[test]
    fn a_carrier_off_by_a_kilohertz_still_reads() {
        // A base station is within a hertz; the receiver's own tuner is not.
        let (bursts, _, rx) = run(72_000.0, 1_000.0);
        assert!(rx.cell.is_some(), "no decode at 1 kHz offset");
        let f = bursts.last().unwrap().freq_offset_hz;
        assert!((f - 1_000.0).abs() < 150.0, "measured {f} Hz of 1000");
    }

    #[test]
    fn the_extractor_floor_rate_is_enough() {
        // 25 kS/s is 1.39 samples a symbol, the least a 25 kHz source
        // extraction can deliver.
        let (bursts, _, rx) = run(25_000.0, 200.0);
        assert!(rx.cell.is_some(), "no decode at 25 kS/s, {} bursts", bursts.len());
    }

    #[test]
    fn normal_bursts_follow_once_the_cell_is_known() {
        let rate = 72_000.0;
        let pdu = sync_pdu(272, 91, 3, 1, 1, 1);
        let sb1 = coding::encode_block(&coding::BLK_BSCH, coding::SCRAMB_INIT, &pdu);
        let scramb = coding::scramb_init(272, 91, 3);
        let half: Vec<u8> = (0..124).map(|i| (i % 2) as u8).collect();
        let full: Vec<u8> = (0..268).map(|i| ((i * 7) % 3 == 0) as u8).collect();
        let bkn2 = coding::encode_block(&coding::BLK_HALF, scramb, &half);
        let sync = synth::sync_burst(&sb1, &[0; 30], &bkn2);
        let h1 = coding::encode_block(&coding::BLK_HALF, scramb, &half);
        let f1 = coding::encode_block(&coding::BLK_FULL, scramb, &full);
        let normal2 = synth::normal_burst(&h1, &[0; 30], &h1, true);
        let normal1 = synth::normal_burst(&f1[..216], &[0; 30], &f1[216..], false);

        let mut bits = Vec::new();
        for _ in 0..3 {
            bits.extend_from_slice(&sync);
            bits.extend_from_slice(&normal2);
            bits.extend_from_slice(&normal1);
            bits.extend_from_slice(&normal2);
        }
        let iq = synth::modulate(&bits, rate, 300.0);

        let mut demod = TetraDemod::new(rate, TetraConfig::default());
        let mut rx = TetraRx::new();
        let mut blocks = Vec::new();
        for chunk in iq.chunks(4096) {
            let mut got = Vec::new();
            demod.process(chunk, &mut got);
            for b in &got {
                rx.push(b, &mut blocks);
            }
        }
        let sch_f: Vec<_> = blocks.iter().filter(|b| b.lchan == Lchan::SchF).collect();
        assert!(!sch_f.is_empty(), "no full-slot block decoded");
        assert!(sch_f.iter().all(|b| b.bits == full));
        let halves = blocks.iter().filter(|b| b.lchan == Lchan::SchHd).count();
        assert!(halves >= 4, "{halves} half-slot blocks");
        // The slot counter names each burst's place, so the cell time labels
        // slots the SYNC PDU never saw.
        let t = blocks
            .iter()
            .filter(|b| b.lchan == Lchan::SchF)
            .find_map(|b| b.time)
            .expect("no time on a SCH/F block");
        assert_eq!(t.tn, 3, "SCH/F sits two slots after the sync burst");
    }
}
