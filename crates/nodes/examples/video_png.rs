//! Turn a capture of an analogue video carrier into pictures.
//!     video_png <file.cs8|cu8|cs16> <rate> <center_hz> <out-prefix> [fields]
//!
//! FM demodulate, separate sync, write each field as a PGM or, when the
//! colour burst was found, a PPM. The line period is measured rather than
//! assumed, so a capture of an NTSC camera says so, and where the frequency
//! is one the 5.8 GHz plan names, it is named.

use common::C32;
use dsp::video::{Standard, SyncSeparator};

fn main() {
    let a: Vec<String> = std::env::args().collect();
    let path = &a[1];
    let rate: f64 = a[2].parse().unwrap();
    let center: f64 = a[3].parse().unwrap();
    let prefix = a.get(4).cloned().unwrap_or_else(|| "field".into());
    let want: usize = a.get(5).and_then(|s| s.parse().ok()).unwrap_or(4);

    let bytes = std::fs::read(path).unwrap();
    let (width, unsigned) = match path.rsplit('.').next() {
        Some("cu8") => (2usize, true),
        Some("cs16") => (4usize, false),
        _ => (2usize, false),
    };
    let iq: Vec<C32> = bytes
        .chunks_exact(width)
        .map(|c| match (width, unsigned) {
            (4, _) => C32::new(
                i16::from_le_bytes([c[0], c[1]]) as f32 / 32768.0,
                i16::from_le_bytes([c[2], c[3]]) as f32 / 32768.0,
            ),
            (_, true) => C32::new((c[0] as f32 - 127.5) / 127.5, (c[1] as f32 - 127.5) / 127.5),
            _ => C32::new(c[0] as i8 as f32 / 128.0, c[1] as i8 as f32 / 128.0),
        })
        .collect();
    eprintln!(
        "{:.2} s at {rate} S/s, centre {:.1} MHz, channel {}",
        iq.len() as f64 / rate,
        center / 1e6,
        decode::fpv::name_at(center as u64, 3_000_000).unwrap_or_else(|| "not in the plan".into())
    );

    // A transmitter deviates about 6 MHz peak, so that is what maps to full
    // scale; the exact figure only scales the picture's contrast, which the
    // separator normalises again from the sync tip.
    let mut demod = dsp::FmDemod::new(rate, 6e6);
    let mut base = Vec::new();
    demod.process(&iq, &mut base);

    // The line period, measured off the sync pulses rather than assumed: it
    // is what says PAL from NTSC.
    let standard = measure_standard(&base, rate).unwrap_or(Standard::Pal);
    eprintln!("line period says {standard:?}");

    let mut sep = SyncSeparator::new(rate, standard, 640).with_colour();
    let mut fields = Vec::new();
    sep.process(&base, &mut fields);
    let st = sep.stats();
    eprintln!(
        "{} fields; {} line syncs, {} broad pulses, {} lines kept, {} fields too short; sync {:.3} black {:.3}",
        fields.len(),
        st.line_syncs,
        st.broad_pulses,
        st.lines_kept,
        st.fields_short,
        st.sync_level,
        st.black_level
    );
    for (i, f) in fields.iter().take(want).enumerate() {
        let (name, header, body) = match &f.rgb {
            Some(rgb) => (format!("{prefix}{i}.ppm"), "P6", &rgb[..]),
            None => (format!("{prefix}{i}.pgm"), "P5", &f.luma[..]),
        };
        let mut out = format!("{header}\n{} {}\n255\n", f.width, f.height).into_bytes();
        out.extend_from_slice(body);
        std::fs::write(&name, out).unwrap();
        eprintln!(
            "wrote {name}: {} of {} lines, {}",
            f.lines_seen,
            f.height,
            if f.rgb.is_some() { "colour" } else { "luma only" }
        );
    }
}

/// Median interval between sync edges, turned into a standard.
fn measure_standard(base: &[f32], rate: f64) -> Option<Standard> {
    let mut v: Vec<f32> = base.iter().take(1 << 20).copied().collect();
    v.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    let (tip, black) = (v[v.len() / 50], v[v.len() * 13 / 100]);
    let thresh = (tip + black) / 2.0;
    let mut edges = Vec::new();
    let mut low = 0usize;
    for (i, &x) in base.iter().enumerate() {
        if x < thresh {
            low += 1;
        } else {
            // Long enough to be a sync pulse, short enough not to be a
            // vertical one.
            if (2e-6 * rate) as usize <= low && low <= (8e-6 * rate) as usize {
                edges.push(i);
            }
            low = 0;
        }
    }
    if edges.len() < 100 {
        return None;
    }
    let mut gaps: Vec<f64> = edges.windows(2).map(|w| (w[1] - w[0]) as f64 / rate).collect();
    gaps.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let median = gaps[gaps.len() / 2];
    eprintln!("median line period {:.3} us", median * 1e6);
    Standard::from_line_period(median)
}
