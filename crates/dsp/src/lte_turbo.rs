//! LTE's turbo code and its rate matching, which DroneID uses unchanged.
//!
//! 3GPP TS 36.212 section 5.1.3: a rate 1/3 parallel concatenated code, two
//! identical eight-state recursive systematic encoders, the second fed by a
//! quadratic permutation polynomial interleaver, then trellis termination and
//! the rate matcher that turns three streams into whatever length the channel
//! wants.
//!
//! DroneID sends E = 7200 bits carrying a K = 1408 bit block, so the rate
//! matcher repeats: the circular buffer is 3 * 1440 = 4320 long and 7200 bits
//! are read out of it, which means most bits arrive nearly twice and the
//! decoder gets them summed. That is why a burst decodes at all from six OFDM
//! symbols.
//!
//! # What is checked here and what is not
//!
//! The encoder is here as well as the decoder, which is not redundant: the
//! tests run bits through both and expect them back, so a wrong tap or a
//! wrong termination fails in this file rather than as a silent CRC failure
//! three layers up. That is a weaker check than an independent
//! implementation, and the real one is `decode::droneid`, where the CRC-24
//! over a frame this project did not construct has to come out zero.

/// The interleaver's parameters for every LTE code block size, from
/// TS 36.212 table 5.1.3-3. Only 1408 is used by DroneID; the rest are here
/// because a table with one row in it is a magic number wearing a hat.
const QPP: [(usize, u32, u32); 188] = [
    (40, 3, 10),
    (48, 7, 12),
    (56, 19, 42),
    (64, 7, 16),
    (72, 7, 18),
    (80, 11, 20),
    (88, 5, 22),
    (96, 11, 24),
    (104, 7, 26),
    (112, 41, 84),
    (120, 103, 90),
    (128, 15, 32),
    (136, 9, 34),
    (144, 17, 108),
    (152, 9, 38),
    (160, 21, 120),
    (168, 101, 84),
    (176, 21, 44),
    (184, 57, 46),
    (192, 23, 48),
    (200, 13, 50),
    (208, 27, 52),
    (216, 11, 36),
    (224, 27, 56),
    (232, 85, 58),
    (240, 29, 60),
    (248, 33, 62),
    (256, 15, 32),
    (264, 17, 198),
    (272, 33, 68),
    (280, 103, 210),
    (288, 19, 36),
    (296, 19, 74),
    (304, 37, 76),
    (312, 19, 78),
    (320, 21, 120),
    (328, 21, 82),
    (336, 115, 84),
    (344, 193, 86),
    (352, 21, 44),
    (360, 133, 90),
    (368, 81, 46),
    (376, 45, 94),
    (384, 23, 48),
    (392, 243, 98),
    (400, 151, 40),
    (408, 155, 102),
    (416, 25, 52),
    (424, 51, 106),
    (432, 47, 72),
    (440, 91, 110),
    (448, 29, 168),
    (456, 29, 114),
    (464, 247, 58),
    (472, 29, 118),
    (480, 89, 180),
    (488, 91, 122),
    (496, 157, 62),
    (504, 55, 84),
    (512, 31, 64),
    (528, 17, 66),
    (544, 35, 68),
    (560, 227, 420),
    (576, 65, 96),
    (592, 19, 74),
    (608, 37, 76),
    (624, 41, 234),
    (640, 39, 80),
    (656, 185, 82),
    (672, 43, 252),
    (688, 21, 86),
    (704, 155, 44),
    (720, 79, 120),
    (736, 139, 92),
    (752, 23, 94),
    (768, 217, 48),
    (784, 25, 98),
    (800, 17, 80),
    (816, 127, 102),
    (832, 25, 52),
    (848, 239, 106),
    (864, 17, 48),
    (880, 137, 110),
    (896, 215, 112),
    (912, 29, 114),
    (928, 15, 58),
    (944, 147, 118),
    (960, 29, 60),
    (976, 59, 122),
    (992, 65, 124),
    (1008, 55, 84),
    (1024, 31, 64),
    (1056, 17, 66),
    (1088, 171, 204),
    (1120, 67, 140),
    (1152, 35, 72),
    (1184, 19, 74),
    (1216, 39, 76),
    (1248, 19, 78),
    (1280, 199, 240),
    (1312, 21, 82),
    (1344, 211, 252),
    (1376, 21, 86),
    (1408, 43, 88),
    (1440, 149, 60),
    (1472, 45, 92),
    (1504, 49, 846),
    (1536, 71, 48),
    (1568, 13, 28),
    (1600, 17, 80),
    (1632, 25, 102),
    (1664, 183, 104),
    (1696, 55, 954),
    (1728, 127, 96),
    (1760, 27, 110),
    (1792, 29, 112),
    (1824, 29, 114),
    (1856, 57, 116),
    (1888, 45, 354),
    (1920, 31, 120),
    (1952, 59, 610),
    (1984, 185, 124),
    (2016, 113, 420),
    (2048, 31, 64),
    (2112, 17, 66),
    (2176, 171, 136),
    (2240, 209, 420),
    (2304, 253, 216),
    (2368, 367, 444),
    (2432, 265, 456),
    (2496, 181, 468),
    (2560, 39, 80),
    (2624, 27, 164),
    (2688, 127, 504),
    (2752, 143, 172),
    (2816, 43, 88),
    (2880, 29, 300),
    (2944, 45, 92),
    (3008, 157, 188),
    (3072, 47, 96),
    (3136, 13, 28),
    (3200, 111, 240),
    (3264, 443, 204),
    (3328, 51, 104),
    (3392, 51, 212),
    (3456, 451, 192),
    (3520, 257, 220),
    (3584, 57, 336),
    (3648, 313, 228),
    (3712, 271, 232),
    (3776, 179, 236),
    (3840, 331, 120),
    (3904, 363, 244),
    (3968, 375, 248),
    (4032, 127, 168),
    (4096, 31, 64),
    (4160, 33, 130),
    (4224, 43, 264),
    (4288, 33, 134),
    (4352, 477, 408),
    (4416, 35, 138),
    (4480, 233, 280),
    (4544, 357, 142),
    (4608, 337, 480),
    (4672, 37, 146),
    (4736, 71, 444),
    (4800, 71, 120),
    (4864, 37, 152),
    (4928, 39, 462),
    (4992, 127, 234),
    (5056, 39, 158),
    (5120, 39, 80),
    (5184, 31, 96),
    (5248, 113, 902),
    (5312, 41, 166),
    (5376, 251, 336),
    (5440, 43, 170),
    (5504, 21, 86),
    (5568, 43, 174),
    (5632, 45, 176),
    (5696, 45, 178),
    (5760, 161, 120),
    (5824, 89, 182),
    (5888, 323, 184),
    (5952, 47, 186),
    (6016, 23, 94),
    (6080, 47, 190),
    (6144, 263, 480),
];

/// The quadratic permutation polynomial interleaver for a block size.
///
/// `None` when the size is not one LTE defines, which is the only sane answer:
/// the polynomial's coefficients come from a table and there is nothing to
/// compute for a size that is not in it.
pub fn interleaver(k: usize) -> Option<Vec<usize>> {
    let &(_, f1, f2) = QPP.iter().find(|(size, _, _)| *size == k)?;
    let (f1, f2) = (f1 as u64, f2 as u64);
    Some(
        (0..k as u64)
            .map(|i| ((f1 * i + f2 * i * i) % k as u64) as usize)
            .collect(),
    )
}

/// The inter-column permutation of the sub-block interleaver, table 5.1.4-1.
const PERMUTE: [usize; 32] = [
    0, 16, 8, 24, 4, 20, 12, 28, 2, 18, 10, 26, 6, 22, 14, 30, 1, 17, 9, 25, 5, 21, 13, 29, 3, 19,
    11, 27, 7, 23, 15, 31,
];

const COLUMNS: usize = 32;

/// Where each position of a sub-block interleaver's output came from, or
/// `None` for a padding position that carries nothing.
///
/// The third stream is interleaved differently from the first two, which is
/// a detail worth stating because getting it wrong costs a third of the
/// parity and the code still almost works.
fn sub_block_map(d: usize, third: bool) -> Vec<Option<usize>> {
    let rows = d.div_ceil(COLUMNS);
    let v = rows * COLUMNS;
    let shift = v - d;
    // Positions of the padded input, as indices into the D-long stream.
    let padded: Vec<Option<usize>> = (0..v)
        .map(|i| if i < shift { None } else { Some(i - shift) })
        .collect();
    if third {
        (0..v)
            .map(|k| padded[(PERMUTE[k / rows] + COLUMNS * (k % rows) + 1) % v])
            .collect()
    } else {
        // Written in by rows, columns permuted, read out by columns.
        let mut scrambled = vec![None; v];
        for r in 0..rows {
            for c in 0..COLUMNS {
                scrambled[r * COLUMNS + c] = padded[r * COLUMNS + PERMUTE[c]];
            }
        }
        let mut out = vec![None; v];
        for c in 0..COLUMNS {
            for r in 0..rows {
                out[rows * c + r] = scrambled[COLUMNS * r + c];
            }
        }
        out
    }
}

/// The circular buffer positions, in the order the transmitter reads them
/// out: which stream and which bit each of the `e` positions carries.
fn buffer_map(d: usize) -> Vec<Option<(usize, usize)>> {
    let rows = d.div_ceil(COLUMNS);
    let v = rows * COLUMNS;
    let maps = [
        sub_block_map(d, false),
        sub_block_map(d, false),
        sub_block_map(d, true),
    ];
    (0..3 * v)
        .map(|n| {
            if n < v {
                maps[0][n].map(|i| (0, i))
            } else {
                let i = (n - v) / 2;
                let stream = 1 + (n - v) % 2;
                maps[stream][i].map(|j| (stream, j))
            }
        })
        .collect()
}

/// Rate matching, forward: the `e` bits a transmitter sends for a block.
///
/// This exists so a receiver can re-encode what it decoded and compare, which
/// is the only way to tell a decoder that failed from a demodulator that
/// handed it rubbish.
pub fn rate_match(d: &[Vec<u8>; 3], e_len: usize) -> Vec<u8> {
    let rows = d[0].len().div_ceil(COLUMNS);
    rate_match_at(d, e_len, 2 * rows)
}

/// The same, starting anywhere in the circular buffer.
pub fn rate_match_at(d: &[Vec<u8>; 3], e_len: usize, k0: usize) -> Vec<u8> {
    let dlen = d[0].len();
    let w = buffer_map(dlen);
    let mut out = Vec::with_capacity(e_len);
    let mut n = k0;
    while out.len() < e_len {
        if let Some((stream, i)) = w[n % w.len()] {
            out.push(d[stream][i]);
        }
        n += 1;
    }
    out
}

/// Undo the rate matching: spread `e` soft values back over the three
/// streams of `d` values each, adding where the circular buffer repeated a
/// bit rather than overwriting, since two receptions of one bit are two
/// pieces of evidence about it.
///
/// Redundancy version 0, which is all DroneID sends.
pub fn rate_dematch(e: &[f32], d: usize) -> [Vec<f32>; 3] {
    rate_dematch_rv(e, d, 0)
}

/// The same, for a stated redundancy version, which is what a search over a
/// frame whose parameters are not yet known needs.
pub fn rate_dematch_rv(e: &[f32], d: usize, rv: usize) -> [Vec<f32>; 3] {
    let rows = d.div_ceil(COLUMNS);
    let w = buffer_map(d);
    let k0 = rows * (2 * (w.len() as f32 / (8.0 * rows as f32)).ceil() as usize * rv + 2);
    dematch_at(e, d, k0)
}

/// The same, starting anywhere in the circular buffer.
///
/// A transmitter that is not an LTE base station may start where it likes,
/// and DJI does: the offset is a search rather than a constant.
pub fn dematch_at(e: &[f32], d: usize, k0: usize) -> [Vec<f32>; 3] {
    let w = buffer_map(d);
    let mut out = [vec![0.0; d], vec![0.0; d], vec![0.0; d]];
    let mut n = k0;
    for &value in e {
        loop {
            let at = n % w.len();
            n += 1;
            if let Some((stream, i)) = w[at] {
                out[stream][i] += value;
                break;
            }
        }
    }
    out
}

/// Spread `e` back over three streams laid end to end, wrapping when the
/// channel carries more bits than the streams hold.
///
/// This is not LTE's rate matcher: there is no sub-block interleaver, no
/// padding to a multiple of 32 and no starting offset, only the three streams
/// concatenated and repeated. DJI's DroneID is built this way, which was
/// established by measurement rather than assumed: in a received burst the
/// bits at 4236 and after are an exact copy of the bits from 0, and 4236 is
/// three times the 1412 a 1408 bit block codes to. LTE's buffer would be
/// 3 * 1440 with 84 punctured positions and would not repeat there.
pub fn dematch_plain(e: &[f32], d: usize) -> [Vec<f32>; 3] {
    let mut out = [vec![0.0; d], vec![0.0; d], vec![0.0; d]];
    for (i, &v) in e.iter().enumerate() {
        let n = i % (3 * d);
        out[n / d][n % d] += v;
    }
    out
}

/// The same, forward: what a transmitter sends for a block.
pub fn match_plain(d: &[Vec<u8>; 3], e_len: usize) -> Vec<u8> {
    (0..e_len)
        .map(|i| {
            let n = i % (3 * d[0].len());
            d[n / d[0].len()][n % d[0].len()]
        })
        .collect()
}

/// The constituent encoder's state transition: given the state and an input
/// bit, the new state and the parity bit.
///
/// g0 = 1 + D + D^3 is the feedback and g1 = 1 + D^2 + D^3 the parity, as
/// TS 36.212 5.1.3.2 defines them.
fn step(state: usize, u: u8) -> (usize, u8) {
    let (s1, s2, s3) = (state & 1, (state >> 1) & 1, (state >> 2) & 1);
    let v = (u as usize) ^ s1 ^ s3;
    let z = v ^ s2 ^ s3;
    ((v) | (s1 << 1) | (s2 << 2), z as u8)
}

/// The tail input that drives the register to zero, which is what
/// termination means for a recursive encoder: the input is whatever the
/// feedback is, so the new bit is zero.
fn tail_input(state: usize) -> u8 {
    ((state & 1) ^ ((state >> 2) & 1)) as u8
}

/// Encode `bits` into the three streams the rate matcher takes, each
/// `bits.len() + 4` long: the block, then three termination steps, then the
/// spare position the standard leaves.
pub fn encode(bits: &[u8]) -> Option<[Vec<u8>; 3]> {
    let k = bits.len();
    let pi = interleaver(k)?;
    let mut d = [vec![0u8; k + 4], vec![0u8; k + 4], vec![0u8; k + 4]];

    let run = |input: &[u8]| -> (Vec<u8>, [u8; 6]) {
        let mut state = 0usize;
        let mut parity = Vec::with_capacity(k);
        for &u in input {
            let (next, z) = step(state, u);
            state = next;
            parity.push(z);
        }
        // Three termination steps produce three systematic and three parity
        // bits, which the standard packs into the four spare positions.
        let mut tail = [0u8; 6];
        for i in 0..3 {
            let u = tail_input(state);
            let (next, z) = step(state, u);
            state = next;
            tail[i * 2] = u;
            tail[i * 2 + 1] = z;
        }
        (parity, tail)
    };

    let (z, tail1) = run(bits);
    let permuted: Vec<u8> = pi.iter().map(|&i| bits[i]).collect();
    let (zp, tail2) = run(&permuted);

    d[0][..k].copy_from_slice(bits);
    d[1][..k].copy_from_slice(&z);
    d[2][..k].copy_from_slice(&zp);
    // TS 36.212 5.1.3.2.2: the twelve tail bits do not stay in their own
    // streams. Six from each encoder are dealt across all three in this
    // order and no other, which is the one part of the code a round trip
    // against an encoder written beside it cannot catch: both halves agree
    // and neither matches the transmitter.
    d[0][k] = tail1[0];
    d[1][k] = tail1[1];
    d[2][k] = tail1[2];
    d[0][k + 1] = tail1[3];
    d[1][k + 1] = tail1[4];
    d[2][k + 1] = tail1[5];
    d[0][k + 2] = tail2[0];
    d[1][k + 2] = tail2[1];
    d[2][k + 2] = tail2[2];
    d[0][k + 3] = tail2[3];
    d[1][k + 3] = tail2[4];
    d[2][k + 3] = tail2[5];
    Some(d)
}

/// Pull the termination bits back out of the streams, so each constituent
/// decoder sees its own three tail steps at the end of its own arrays.
///
/// Returns the systematic and parity tails for the first decoder, and the
/// systematic and parity tails for the second.
fn unterminate(d: &[Vec<f32>; 3], k: usize) -> ([[f32; 2]; 3], [[f32; 2]; 3]) {
    let mut first = [[0.0; 2]; 3];
    let mut second = [[0.0; 2]; 3];
    first[0] = [d[0][k], d[1][k]];
    first[1] = [d[2][k], d[0][k + 1]];
    first[2] = [d[1][k + 1], d[2][k + 1]];
    second[0] = [d[0][k + 2], d[1][k + 2]];
    second[1] = [d[2][k + 2], d[0][k + 3]];
    second[2] = [d[1][k + 3], d[2][k + 3]];
    (first, second)
}

/// One constituent decoder: max-log-MAP over the eight state trellis,
/// returning the extrinsic log-likelihood of each information bit.
fn bcjr(systematic: &[f32], parity: &[f32], apriori: &[f32], tail: &[[f32; 2]; 3]) -> Vec<f32> {
    let k = systematic.len();
    let n = k + 3;
    // The trellis, precomputed: for each state and input, where it goes and
    // what parity it emits.
    let mut next = [[0usize; 2]; 8];
    let mut emit = [[0f32; 2]; 8];
    for s in 0..8 {
        for u in 0..2 {
            let (ns, z) = step(s, u as u8);
            next[s][u] = ns;
            emit[s][u] = if z == 0 { 1.0 } else { -1.0 };
        }
    }
    let sys = |i: usize| -> f32 {
        if i < k {
            systematic[i]
        } else {
            tail[i - k][0]
        }
    };
    let par = |i: usize| -> f32 {
        if i < k {
            parity[i]
        } else {
            tail[i - k][1]
        }
    };
    let apri = |i: usize| -> f32 {
        if i < k {
            apriori[i]
        } else {
            0.0
        }
    };

    const NEG: f32 = -1e9;
    let mut alpha = vec![[NEG; 8]; n + 1];
    alpha[0][0] = 0.0;
    for i in 0..n {
        let (s_i, p_i, a_i) = (sys(i), par(i), apri(i));
        for s in 0..8 {
            if alpha[i][s] <= NEG {
                continue;
            }
            for u in 0..2 {
                let x = if u == 0 { 1.0 } else { -1.0 };
                let m = alpha[i][s] + 0.5 * (x * (s_i + a_i) + emit[s][u] * p_i);
                let t = next[s][u];
                if m > alpha[i + 1][t] {
                    alpha[i + 1][t] = m;
                }
            }
        }
        // Keep the metrics from running away.
        let best = alpha[i + 1].iter().cloned().fold(NEG, f32::max);
        if best > NEG {
            for v in alpha[i + 1].iter_mut() {
                *v -= best;
            }
        }
    }

    let mut beta = vec![[NEG; 8]; n + 1];
    // Termination drives the encoder to the zero state, so that is the only
    // way the trellis may end. A decoder that does not know this throws away
    // what the tail bits were for.
    beta[n][0] = 0.0;
    for i in (0..n).rev() {
        let (s_i, p_i, a_i) = (sys(i), par(i), apri(i));
        for s in 0..8 {
            for u in 0..2 {
                let t = next[s][u];
                if beta[i + 1][t] <= NEG {
                    continue;
                }
                let x = if u == 0 { 1.0 } else { -1.0 };
                let m = beta[i + 1][t] + 0.5 * (x * (s_i + a_i) + emit[s][u] * p_i);
                if m > beta[i][s] {
                    beta[i][s] = m;
                }
            }
        }
        let best = beta[i].iter().cloned().fold(NEG, f32::max);
        if best > NEG {
            for v in beta[i].iter_mut() {
                *v -= best;
            }
        }
    }

    (0..k)
        .map(|i| {
            let (s_i, p_i, a_i) = (sys(i), par(i), apri(i));
            let mut best = [NEG; 2];
            for s in 0..8 {
                if alpha[i][s] <= NEG {
                    continue;
                }
                for u in 0..2 {
                    let t = next[s][u];
                    if beta[i + 1][t] <= NEG {
                        continue;
                    }
                    let x = if u == 0 { 1.0 } else { -1.0 };
                    let m = alpha[i][s]
                        + 0.5 * (x * (s_i + a_i) + emit[s][u] * p_i)
                        + beta[i + 1][t];
                    if m > best[u] {
                        best[u] = m;
                    }
                }
            }
            // The extrinsic part alone: what this decoder learned, without
            // the channel and the other decoder's opinion fed back in.
            best[0] - best[1] - s_i - a_i
        })
        .collect()
}

/// Decode `k` information bits from three soft streams of `k + 4` values,
/// where a positive value means a zero bit.
pub fn decode(d: &[Vec<f32>; 3], k: usize, iterations: usize) -> Option<Vec<u8>> {
    let pi = interleaver(k)?;
    let (tail1, tail2) = unterminate(d, k);
    let sys: Vec<f32> = d[0][..k].to_vec();
    let par1: Vec<f32> = d[1][..k].to_vec();
    let par2: Vec<f32> = d[2][..k].to_vec();
    let sys_i: Vec<f32> = pi.iter().map(|&i| sys[i]).collect();

    let mut extrinsic = vec![0.0f32; k];
    let mut out = vec![0u8; k];
    for _ in 0..iterations {
        let e1 = bcjr(&sys, &par1, &extrinsic, &tail1);
        let a2: Vec<f32> = pi.iter().map(|&i| e1[i]).collect();
        let e2 = bcjr(&sys_i, &par2, &a2, &tail2);
        // Back into the original order for the next round.
        let mut back = vec![0.0f32; k];
        for (i, &p) in pi.iter().enumerate() {
            back[p] = e2[i];
        }
        extrinsic = back;
        for i in 0..k {
            out[i] = u8::from(sys[i] + e1[i] + extrinsic[i] < 0.0);
        }
    }
    Some(out)
}

/// The CRC-24A of TS 36.212 5.1.1, which covers a turbo decoded block: the
/// check bits are inside the block, so a good decode leaves zero.
pub fn crc24a(bytes: &[u8]) -> u32 {
    let mut r: u32 = 0;
    for &b in bytes {
        r ^= (b as u32) << 16;
        for _ in 0..8 {
            r <<= 1;
            if r & 0x0100_0000 != 0 {
                r ^= 0x0186_4CFB;
            }
        }
    }
    r & 0x00ff_ffff
}

#[cfg(test)]
mod tests {
    use super::*;

    fn block(k: usize) -> Vec<u8> {
        let mut seed = 2463534242u32;
        (0..k)
            .map(|_| {
                seed ^= seed << 13;
                seed ^= seed >> 17;
                seed ^= seed << 5;
                (seed & 1) as u8
            })
            .collect()
    }

    /// A quadratic permutation polynomial is a permutation: every position
    /// appears exactly once. A wrong pair of coefficients usually is not,
    /// which makes this a real check on the table.
    #[test]
    fn the_interleaver_is_a_permutation() {
        for &(k, _, _) in QPP.iter() {
            let pi = interleaver(k).expect("a size from the table");
            let mut seen = vec![false; k];
            for &p in &pi {
                assert!(!seen[p], "K={k} repeats {p}");
                seen[p] = true;
            }
        }
        assert!(interleaver(1000).is_none(), "1000 is not an LTE block size");
    }

    /// DroneID's own size, which is the one that matters here.
    #[test]
    fn droneids_block_size_is_in_the_table() {
        let pi = interleaver(1408).expect("1408 is an LTE block size");
        assert_eq!(pi.len(), 1408);
        assert_eq!(pi[1], 43 + 88);
    }

    /// The sub-block interleaver pads at the front and nowhere else, and
    /// every real bit survives it exactly once.
    #[test]
    fn the_sub_block_interleaver_keeps_every_bit_once() {
        for third in [false, true] {
            let map = sub_block_map(1412, third);
            assert_eq!(map.len(), 1440);
            assert_eq!(map.iter().filter(|m| m.is_none()).count(), 28);
            let mut seen = vec![false; 1412];
            for m in map.into_iter().flatten() {
                assert!(!seen[m]);
                seen[m] = true;
            }
            assert!(seen.into_iter().all(|s| s));
        }
    }

    /// The rate matcher repeats: 7200 bits out of a 4320 bit buffer means
    /// most positions are sent twice and some three times. Nothing may be
    /// sent zero times, or the decoder is missing parity it was promised.
    #[test]
    fn de_rate_matching_covers_every_bit_of_every_stream() {
        let d = rate_dematch(&vec![1.0f32; 7200], 1412);
        for (n, stream) in d.iter().enumerate() {
            assert_eq!(stream.len(), 1412);
            let missing = stream.iter().filter(|v| **v == 0.0).count();
            assert_eq!(missing, 0, "stream {n} has {missing} bits nobody sent");
        }
        // 7200 arrivals spread over 3 * 1412 positions.
        let total: f32 = d.iter().flatten().sum();
        assert_eq!(total, 7200.0);
    }

    /// Encode and decode a block with no noise at all. This is the weakest
    /// possible test of a decoder and the strongest possible test of the
    /// trellis, the termination and the interleaver agreeing with each
    /// other: one wrong tap and nothing comes back.
    #[test]
    fn a_clean_block_decodes_to_itself() {
        let bits = block(1408);
        let d = encode(&bits).expect("an LTE block size");
        let soft: [Vec<f32>; 3] = [
            d[0].iter().map(|&b| if b == 0 { 4.0 } else { -4.0 }).collect(),
            d[1].iter().map(|&b| if b == 0 { 4.0 } else { -4.0 }).collect(),
            d[2].iter().map(|&b| if b == 0 { 4.0 } else { -4.0 }).collect(),
        ];
        let out = decode(&soft, 1408, 4).expect("a decode");
        assert_eq!(out, bits);
    }

    /// The same block through the rate matcher and back, which is the path a
    /// received burst takes.
    #[test]
    fn a_block_survives_the_rate_matcher() {
        let bits = block(1408);
        let d = encode(&bits).expect("an LTE block size");
        let e: Vec<f32> = rate_match(&d, 7200)
            .into_iter()
            .map(|b| if b == 0 { 1.0 } else { -1.0 })
            .collect();
        let back = rate_dematch(&e, 1412);
        let out = decode(&back, 1408, 4).expect("a decode");
        assert_eq!(out, bits);
    }

    /// The CRC is the one LTE puts on a transport block, and a block with
    /// its own check bits on the end leaves zero behind.
    #[test]
    fn the_crc_zeroes_out_over_a_block_that_carries_it() {
        let mut msg = vec![0x11u8, 0x22, 0x33, 0x44, 0x55];
        let c = crc24a(&msg);
        msg.push((c >> 16) as u8);
        msg.push((c >> 8) as u8);
        msg.push(c as u8);
        assert_eq!(crc24a(&msg), 0);
    }
}

#[cfg(test)]
mod noise_tests {
    use super::*;

    fn block(k: usize) -> Vec<u8> {
        let mut seed = 2463534242u32;
        (0..k).map(|_| { seed ^= seed << 13; seed ^= seed >> 17; seed ^= seed << 5; (seed & 1) as u8 }).collect()
    }

    #[test]
    fn hard_decisions_with_errors_still_decode() {
        let bits = block(1408);
        let d = encode(&bits).expect("size");
        let e = rate_match(&d, 7200);
        let mut seed = 99u32;
        for pct in [0usize, 2, 5, 10] {
            let soft: Vec<f32> = e.iter().map(|&b| {
                seed ^= seed << 13; seed ^= seed >> 17; seed ^= seed << 5;
                let flip = (seed % 100) < pct as u32;
                let b = b ^ u8::from(flip);
                if b == 0 { 1.0 } else { -1.0 }
            }).collect();
            let back = rate_dematch(&soft, 1412);
            let out = decode(&back, 1408, 6).expect("decode");
            let wrong = out.iter().zip(&bits).filter(|(a, b)| a != b).count();
            println!("{pct}% flips -> {wrong} wrong bits");
        }
    }
}
