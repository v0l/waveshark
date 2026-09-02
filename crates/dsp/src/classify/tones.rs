//! Counting tones in the averaged spectrum, when the frequency track cannot.
//!
//! The histogram of instantaneous frequency is the better measurement wherever
//! it works: it sees the levels a discriminator would, in the order they were
//! sent, and four levels are as easy as two. It needs samples per symbol to do
//! it, though, and how many a burst has is not the receiver's choice.
//!
//! Bluetooth is the case that forced this. One megabit per second sampled at
//! four megasamples is four samples a symbol, and the per-sample frequency
//! noise at that spacing is larger than the deviation being measured: the
//! histogram reports the two tones 72 kHz apart where they are 500 kHz apart,
//! and every ratio derived from that is wrong.
//!
//! The spectrum has no such problem, because a transform integrates a symbol's
//! worth of samples into each bin, which is exactly the averaging the
//! per-sample track lacks. What it cannot do is tell two levels from four when
//! the shaping merges them, which is why this is a fallback and not the
//! primary.

/// Tone count and outermost separation in hertz, from an averaged spectrum.
///
/// `spec` is power per bin with DC at the centre. Returns 0 tones when the
/// spectrum has no separable structure, which is the honest answer for a
/// single shaped hump.
pub fn from_spectrum(spec: &[f32], rate: f32, floor_mult: f32) -> (u8, f32) {
    let n = spec.len();
    if n < 32 {
        return (0, 0.0);
    }
    let mut sorted: Vec<f32> = spec.to_vec();
    sorted.sort_by(f32::total_cmp);
    // Against the quietest quarter rather than the median: a signal occupying
    // more than half the span *is* the median, and the threshold then sits
    // inside it.
    let q = (n / 4).max(1);
    let floor = sorted[..q].iter().sum::<f32>() / q as f32;
    let th = floor.max(1e-20) * floor_mult;

    // Contiguous occupied regions, bridging single-bin dropouts.
    let mut regions: Vec<(f64, f64, usize)> = Vec::new();
    let mut open: Option<(f64, f64, usize)> = None;
    let mut gap = 0;
    for (i, &p) in spec.iter().enumerate() {
        if p > th {
            gap = 0;
            let e = open.get_or_insert((0.0, 0.0, 0));
            e.0 += p as f64;
            e.1 += p as f64 * i as f64;
            e.2 += 1;
        } else if let Some(e) = open {
            gap += 1;
            if gap > 1 {
                regions.push(e);
                open = None;
            }
        }
    }
    if let Some(e) = open {
        regions.push(e);
    }
    if regions.len() < 2 {
        return (0, 0.0);
    }

    // Strongest first, and a peer has to be a peer: a shoulder or a spur at a
    // twentieth of the power is not a second tone.
    regions.sort_by(|a, b| b.0.total_cmp(&a.0));
    let strongest = regions[0].0;
    regions.retain(|r| r.0 >= strongest * 0.05);
    let count = match regions.len() {
        2 => 2u8,
        3..=5 => 4,
        _ => return (0, 0.0),
    };
    let mut centres: Vec<f64> = regions.iter().map(|r| r.1 / r.0.max(1e-20)).collect();
    centres.sort_by(f64::total_cmp);
    let bin_hz = rate as f64 / n as f64;
    let sep = (centres[centres.len() - 1] - centres[0]) * bin_hz;
    (count, sep as f32)
}
