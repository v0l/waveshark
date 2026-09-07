//! Reading a burst through the channel it arrived over.
//!
//! Taking the sign of each symbol works on a signal that arrived by one path
//! and nothing else, which is a transmitter in a laboratory. Two things break
//! it on the air. GMSK spreads every symbol over about three symbol periods
//! by construction, so even a perfect channel delivers each symbol on top of
//! its neighbours; and a real path arrives several times over, microseconds
//! apart, from whatever the signal bounced off.
//!
//! Both are the same problem: the received sample is a sum over the last few
//! symbols rather than one symbol. So the burst's training sequence, which is
//! known, is used to measure what that sum is, and then the whole burst is
//! read through a trellis that knows it.
//!
//! Measured on an off-air capture of a live cell at about 10 dB in the
//! channel, the sign-of-the-symbol reading recovered one synchronisation
//! burst in 119 and this recovered most of them.
//!
//! # What comes out
//!
//! Soft bits, in the convention the rest of this tree uses: positive for a
//! one and larger the more the detector believes it. They come from a
//! max-log-MAP pass rather than from the surviving path, because the
//! convolutional code below wants to know how sure each bit is, and a hard
//! decision throws away about two decibels of what the trellis knows.

use common::C32;

/// Symbols the channel is assumed to spread each symbol over.
///
/// Five covers GMSK's own three and leaves two symbol periods, about seven
/// microseconds, of delay spread for the path. Longer costs states
/// exponentially and buys little: a delay beyond that is past the guard
/// period a GSM burst allows anyway.
pub const TAPS: usize = 5;

/// States in the trellis: the four symbols before the one being decided.
const STATES: usize = 1 << (TAPS - 1);

/// The channel as the training sequence measures it.
///
/// `y` is the burst at one complex sample a symbol, derotated so that a
/// symbol is real; `known` is the training sequence as modulating values,
/// `+1` for a zero bit and `-1` for a one, and `at` is where it sits in the
/// burst.
///
/// Least squares over the part of the sequence where every tap sees a known
/// symbol. The result absorbs the phase the burst arrived at, so nothing
/// downstream has to correct for it separately.
pub fn estimate(y: &[C32], known: &[f32], at: usize) -> Option<[C32; TAPS]> {
    if known.len() <= TAPS || at + known.len() > y.len() {
        return None;
    }
    // Normal equations. The symbols are real and the samples complex, so the
    // matrix is real and only the right hand side carries the phase.
    let mut r = [[0.0f64; TAPS]; TAPS];
    let mut p = [(0.0f64, 0.0f64); TAPS];
    for k in TAPS - 1..known.len() {
        for i in 0..TAPS {
            let vi = f64::from(known[k - i]);
            for j in 0..TAPS {
                r[i][j] += vi * f64::from(known[k - j]);
            }
            let s = y[at + k];
            p[i].0 += vi * f64::from(s.re);
            p[i].1 += vi * f64::from(s.im);
        }
    }
    let re = solve(r, [p[0].0, p[1].0, p[2].0, p[3].0, p[4].0])?;
    let im = solve(r, [p[0].1, p[1].1, p[2].1, p[3].1, p[4].1])?;
    let mut h = [C32::new(0.0, 0.0); TAPS];
    for i in 0..TAPS {
        h[i] = C32::new(re[i] as f32, im[i] as f32);
    }
    Some(h)
}

/// Gaussian elimination with partial pivoting on a small real system.
fn solve(mut a: [[f64; TAPS]; TAPS], mut b: [f64; TAPS]) -> Option<[f64; TAPS]> {
    for col in 0..TAPS {
        let pivot = (col..TAPS).max_by(|&x, &y| {
            a[x][col].abs().partial_cmp(&a[y][col].abs()).unwrap_or(std::cmp::Ordering::Equal)
        })?;
        if a[pivot][col].abs() < 1e-9 {
            return None;
        }
        a.swap(col, pivot);
        b.swap(col, pivot);
        for row in col + 1..TAPS {
            let f = a[row][col] / a[col][col];
            for c in col..TAPS {
                a[row][c] -= f * a[col][c];
            }
            b[row] -= f * b[col];
        }
    }
    let mut x = [0.0f64; TAPS];
    for row in (0..TAPS).rev() {
        let mut v = b[row];
        for c in row + 1..TAPS {
            v -= a[row][c] * x[c];
        }
        x[row] = v / a[row][row];
    }
    Some(x)
}

/// What is left of the frequency error, in radians a symbol, measured on the
/// training sequence alone.
///
/// The tone one frame earlier places the carrier to a couple of kilohertz,
/// which is close enough to hear the burst and not close enough to read it:
/// the channel estimate absorbs a constant phase, so the training sequence
/// in the middle of the burst fits well while the bits at each end have
/// rotated away. Two kilohertz turns the phase a full turn across a burst.
///
/// So the sequence is halved and a channel estimated from each. Both describe
/// the same path, so what differs between them is the phase the carrier
/// drifted through in between, and that divided by the gap is the error.
/// Unambiguous to about ten kilohertz, which is well beyond what survives the
/// tone.
pub fn residual(y: &[C32], known: &[f32], at: usize) -> Option<f32> {
    let half = known.len() / 2;
    if half <= TAPS {
        return None;
    }
    let first = estimate(y, &known[..half], at)?;
    let second = estimate(y, &known[half..], at + half)?;
    // The taps are compared as one vector rather than one at a time, so a
    // weak tap contributes little and a dominant one decides.
    let mut sum = C32::new(0.0, 0.0);
    for (a, b) in second.iter().zip(first.iter()) {
        sum += a * b.conj();
    }
    Some(sum.arg() / half as f32)
}

/// Turn the burst by a fixed amount per symbol, undoing what [`residual`]
/// measured.
pub fn derotate(y: &mut [C32], per_symbol: f32) {
    for (k, s) in y.iter_mut().enumerate() {
        let ph = -per_symbol * k as f32;
        *s *= C32::new(ph.cos(), ph.sin());
    }
}

/// How much of the burst the channel estimate explains, from zero to one.
///
/// One means the training sequence arrived exactly as the estimate predicts,
/// which no real burst does; what this is for is telling a burst that was
/// received from a stretch of noise that happened to correlate.
pub fn fit(y: &[C32], known: &[f32], at: usize, h: &[C32; TAPS]) -> f32 {
    let mut err = 0.0f64;
    let mut power = 0.0f64;
    for k in TAPS - 1..known.len() {
        let want = predict(h, known[k], &known[k + 1 - TAPS..k]);
        let got = y[at + k];
        err += f64::from((got - want).norm_sqr());
        power += f64::from(got.norm_sqr());
    }
    if power <= 0.0 {
        return 0.0;
    }
    (1.0 - err / power).max(0.0) as f32
}

/// The sample a symbol and the four before it produce through `h`.
///
/// `past` is those four, oldest first, as modulating values.
fn predict(h: &[C32; TAPS], now: f32, past: &[f32]) -> C32 {
    let mut v = h[0] * now;
    for (i, &p) in past.iter().rev().enumerate() {
        v += h[i + 1] * p;
    }
    v
}

/// Read the whole burst through the channel, returning a soft bit per symbol.
///
/// The burst begins and ends with three tail bits that are always zero, so
/// the trellis starts and finishes in a known state rather than at whichever
/// state happens to look best, exactly as the convolutional decoder below it
/// does.
pub fn soft_bits(y: &[C32], h: &[C32; TAPS], out: &mut [f32]) {
    let n = out.len().min(y.len());
    // Modulating values for the trellis: bit index 0 of a state is the most
    // recent symbol, and a set bit is a one, which modulates as -1.
    let value = |bit: usize| if bit == 0 { 1.0f32 } else { -1.0f32 };
    let state_values = |s: usize| {
        let mut v = [0.0f32; TAPS - 1];
        for (i, slot) in v.iter_mut().enumerate() {
            *slot = value(s >> i & 1);
        }
        v
    };

    // Branch costs, computed once: for every state and every input, what the
    // channel would have produced.
    let mut gamma = vec![[0.0f32; 2 * STATES]; n];
    for (k, g) in gamma.iter_mut().enumerate() {
        for s in 0..STATES {
            let past = state_values(s);
            for a in 0..2 {
                // `past` runs most recent first; predict wants oldest first.
                let mut oldest_first = [0.0f32; TAPS - 1];
                for i in 0..TAPS - 1 {
                    oldest_first[i] = past[TAPS - 2 - i];
                }
                let want = predict(h, value(a), &oldest_first);
                g[s * 2 + a] = (y[k] - want).norm_sqr();
            }
        }
    }

    // Neither end of the burst is a known state. The three tail bits at the
    // start are zeros, but the four symbols before them belong to whatever
    // the transmitter did in the guard period, and the three at the end are
    // followed by the same. So the trellis begins and ends open: forcing it
    // to start and finish in the all-zero state asserted four symbols that
    // were never sent, and the bits near each end came out of that assertion
    // rather than out of the signal.
    const BIG: f32 = 1e12;
    let mut alpha = vec![[BIG; STATES]; n + 1];
    alpha[0] = [0.0; STATES];
    for k in 0..n {
        for s in 0..STATES {
            if alpha[k][s] >= BIG {
                continue;
            }
            for a in 0..2 {
                let next = (s << 1 | a) & (STATES - 1);
                let m = alpha[k][s] + gamma[k][s * 2 + a];
                if m < alpha[k + 1][next] {
                    alpha[k + 1][next] = m;
                }
            }
        }
    }

    let mut beta = vec![[BIG; STATES]; n + 1];
    beta[n] = [0.0; STATES];
    for k in (0..n).rev() {
        for s in 0..STATES {
            for a in 0..2 {
                let next = (s << 1 | a) & (STATES - 1);
                if beta[k + 1][next] >= BIG {
                    continue;
                }
                let m = beta[k + 1][next] + gamma[k][s * 2 + a];
                if m < beta[k][s] {
                    beta[k][s] = m;
                }
            }
        }
    }

    // The difference between the best explanation with this bit a zero and
    // the best with it a one, which is the evidence for the bit and nothing
    // else.
    for k in 0..n {
        let mut best = [BIG; 2];
        for s in 0..STATES {
            if alpha[k][s] >= BIG {
                continue;
            }
            for a in 0..2 {
                let next = (s << 1 | a) & (STATES - 1);
                let m = alpha[k][s] + gamma[k][s * 2 + a] + beta[k + 1][next];
                if m < best[a] {
                    best[a] = m;
                }
            }
        }
        // Positive for a one, which is the convention every decoder here
        // reads. Scaled by the burst's own noise level so the number means
        // the same thing from one burst to the next.
        out[k] = best[0] - best[1];
    }

    // Normalise on the middling bit rather than the average one, and clamp.
    // A max-log-MAP difference has a long tail: a handful of bits that no
    // path disputes come out orders of magnitude above the rest, and scaling
    // by the average of those leaves every honest bit reading as a doubt.
    let mut mags: Vec<f32> = out[..n].iter().map(|v| v.abs()).collect();
    mags.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    let median = mags[n / 2];
    if median > 0.0 {
        for v in out[..n].iter_mut() {
            *v = (*v / median).clamp(-4.0, 4.0);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A channel with an echo: the main path, and a second arriving a symbol
    /// later at a third of the amplitude and a quarter turn out.
    fn echo() -> [C32; TAPS] {
        [
            C32::new(0.05, -0.02),
            C32::new(0.80, 0.30),
            C32::new(0.25, 0.20),
            C32::new(0.05, 0.0),
            C32::new(0.0, 0.0),
        ]
    }

    fn values(bits: &[u8]) -> Vec<f32> {
        bits.iter().map(|&b| if b == 0 { 1.0 } else { -1.0 }).collect()
    }

    /// Push symbols through a known channel.
    fn through(v: &[f32], h: &[C32; TAPS]) -> Vec<C32> {
        (0..v.len())
            .map(|k| {
                let mut s = C32::new(0.0, 0.0);
                for i in 0..TAPS {
                    if k >= i {
                        s += h[i] * v[k - i];
                    }
                }
                s
            })
            .collect()
    }

    #[test]
    fn the_training_sequence_measures_the_channel() {
        let h = echo();
        let bits: Vec<u8> = (0..40).map(|i| u8::from(i % 3 == 0 || i % 7 == 2)).collect();
        let v = values(&bits);
        let y = through(&v, &h);
        let got = estimate(&y, &v, 0).expect("a channel");
        for (a, b) in got.iter().zip(h.iter()) {
            assert!((a - b).norm() < 1e-3, "{got:?} against {h:?}");
        }
        assert!(fit(&y, &v, 0, &got) > 0.99, "a noiseless burst fits exactly");
    }

    /// The whole point: a burst that arrived through an echo reads back as
    /// what was sent. Slicing the symbols would get about a third of them
    /// wrong here, since the echo is a quarter of the main path.
    #[test]
    fn a_burst_through_an_echo_reads_back() {
        let h = echo();
        let mut bits = vec![0u8; 100];
        let mut state = 0x1234_5678u32;
        for b in bits.iter_mut().skip(4).take(92) {
            state ^= state << 13;
            state ^= state >> 17;
            state ^= state << 5;
            *b = (state & 1) as u8;
        }
        let v = values(&bits);
        let y = through(&v, &h);
        let est = estimate(&y, &v[20..60], 20).expect("a channel");
        let mut soft = vec![0.0f32; bits.len()];
        soft_bits(&y, &est, &mut soft);
        let wrong = bits
            .iter()
            .zip(&soft)
            .filter(|(&b, &s)| (s > 0.0) != (b == 1))
            .count();
        assert_eq!(wrong, 0, "{wrong} bits wrong through the echo");
    }

    /// Noise costs confidence before it costs bits, which is what makes the
    /// soft output worth carrying: the decoder below can use a bit the
    /// detector is unsure of.
    #[test]
    fn noise_lowers_the_confidence_before_it_flips_bits() {
        let h = echo();
        let bits: Vec<u8> = (0..80).map(|i| u8::from(i % 5 < 2)).collect();
        let v = values(&bits);
        let clean = through(&v, &h);
        let mut state = 0x9E37_79B9u32;
        let mut rand = move || {
            state ^= state << 13;
            state ^= state >> 17;
            state ^= state << 5;
            (state as f32 / u32::MAX as f32) - 0.5
        };
        let noisy: Vec<C32> =
            clean.iter().map(|&c| c + C32::new(rand(), rand()) * 0.7).collect();
        let est = estimate(&noisy, &v[20..60], 20).unwrap();
        let mut a = vec![0.0f32; bits.len()];
        let mut b = vec![0.0f32; bits.len()];
        soft_bits(&clean, &est, &mut a);
        soft_bits(&noisy, &est, &mut b);
        // The output is scaled so the average bit reads one either way, so
        // what noise changes is the spread: how many bits the detector is
        // markedly less sure of than the rest.
        let unsure = |s: &[f32]| s.iter().filter(|v| v.abs() < 0.5).count();
        assert!(
            unsure(&b) > unsure(&a),
            "noise did not show up as doubt: {} against {}",
            unsure(&b),
            unsure(&a)
        );
        let wrong = bits.iter().zip(&b).filter(|(&x, &s)| (s > 0.0) != (x == 1)).count();
        assert!(wrong <= 2, "{wrong} bits wrong under noise");
    }
}
