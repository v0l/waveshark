//! What is transmitting across a wide capture, and where: one coarse
//! spectrogram row per millisecond, and the runs of energy in each megahertz
//! bin reported as bursts with their width and duration.
//!
//! For a link that hops, which is what a drone control link does, the useful
//! picture is not one burst but the pattern: how wide each hop is, how long
//! it lasts, and how far apart in frequency the next one lands.
//!     band_bursts <file> [threshold_db]
use common::device::Device;
use std::io::Read;

const FFT: usize = 256;

fn main() {
    let path = std::env::args().nth(1).expect("a capture");
    let thresh_db: f32 = std::env::args()
        .nth(2)
        .and_then(|s| s.parse().ok())
        .unwrap_or(8.0);
    let src = sources::FileSource::open(&path).expect("open");
    let rate = src.rate().as_f64();
    let center = src.center().as_f64();
    let bin_hz = rate / FFT as f64;

    // Read the file rather than hold it: 30 s at 61.44 MS/s is 15 GB as C32.
    let mut f = std::fs::File::open(&path).expect("open");
    let mut raw = vec![0u8; FFT * 4];
    let mut plan = rustfft::FftPlanner::<f32>::new();
    let fft = plan.plan_fft_forward(FFT);

    let mut rows: Vec<Vec<f32>> = Vec::new();
    // Every sample is read, not one window a millisecond: a link that
    // transmits for 300 us and then hops is invisible to a snapshot, and a
    // snapshot is what makes a hopping transmitter look like a carrier.
    let per_row = (rate / 1000.0) as usize / FFT;
    let mut acc = vec![0.0f32; FFT];
    let mut n = 0usize;
    let mut peak = vec![f32::MIN; FFT];
    let _ = &mut acc;
    loop {
        if f.read_exact(&mut raw).is_err() {
            break;
        }
        let mut buf: Vec<common::C32> = raw
            .chunks_exact(4)
            .map(|b| {
                let re = i16::from_le_bytes([b[0], b[1]]) as f32 / 32768.0;
                let im = i16::from_le_bytes([b[2], b[3]]) as f32 / 32768.0;
                common::C32::new(re, im)
            })
            .collect();
        fft.process(&mut buf);
        for k in 0..FFT {
            peak[k] = peak[k].max(buf[k].norm_sqr());
        }
        n += 1;
        if n == per_row {
            // The loudest window in the millisecond, not the average: a
            // burst a fiftieth of the row long still has to show.
            rows.push(peak.iter().map(|p| p.max(1e-20).log10() * 10.0).collect());
            peak.iter_mut().for_each(|p| *p = f32::MIN);
            n = 0;
        }
    }
    println!(
        "{} rows, {:.1} s at {:.2} MS/s, centre {:.1} MHz, {:.3} MHz a bin",
        rows.len(),
        rows.len() as f64 / 1000.0,
        rate / 1e6,
        center / 1e6,
        bin_hz / 1e6
    );

    // The floor per bin is its own median over the whole file, so a bin that
    // holds a constant carrier does not set the floor for its neighbours.
    let mut floor = vec![0.0f32; FFT];
    for k in 0..FFT {
        let mut col: Vec<f32> = rows.iter().map(|r| r[k]).collect();
        col.sort_by(|a, b| a.partial_cmp(b).unwrap());
        floor[k] = col[col.len() / 2];
    }

    // A run of consecutive rows where a bin is above its floor is a burst.
    let mut open: Vec<Option<(usize, f32)>> = vec![None; FFT];
    let mut bursts: Vec<(f64, usize, f32)> = Vec::new(); // MHz, ms, peak dB over floor
    for (t, row) in rows.iter().enumerate() {
        for k in 0..FFT {
            let over = row[k] - floor[k];
            match (&mut open[k], over > thresh_db) {
                (slot @ None, true) => *slot = Some((t, over)),
                (Some((_, peak)), true) => *peak = peak.max(over),
                (slot @ Some(_), false) => {
                    let (start, peak) = slot.take().unwrap();
                    let hz = if k < FFT / 2 { k } else { k } as f64;
                    let off = if k < FFT / 2 {
                        hz * bin_hz
                    } else {
                        (hz - FFT as f64) * bin_hz
                    };
                    bursts.push(((center + off) / 1e6, t - start, peak));
                }
                (None, false) => {}
            }
        }
    }
    println!("{} bin-bursts over {thresh_db} dB", bursts.len());

    // Group by megahertz so a signal wider than a bin is one row.
    let mut per_mhz: std::collections::BTreeMap<i64, (usize, usize, f32)> = Default::default();
    for (mhz, len, peak) in &bursts {
        let e = per_mhz.entry(mhz.round() as i64).or_insert((0, 0, 0.0));
        e.0 += 1;
        e.1 += len;
        e.2 = e.2.max(*peak);
    }
    // A text spectrogram of a slice, because a hopping link is a pattern in
    // time and frequency and a table of totals cannot show one.
    if let Ok(v) = std::env::var("SPECTROGRAM") {
        let from: usize = v.parse().unwrap_or(0);
        println!(
            "{:.0} to {:.0} MHz, one row a millisecond",
            (center - rate / 2.0) / 1e6,
            (center + rate / 2.0) / 1e6
        );
        for t in from..(from + 120).min(rows.len()) {
            let line: String = (0..FFT)
                .map(|i| {
                    // Rotate so the row reads low frequency to high.
                    let k = (i + FFT / 2) % FFT;
                    let over = rows[t][k] - floor[k];
                    match over {
                        o if o > 20.0 => '#',
                        o if o > 12.0 => '+',
                        o if o > 6.0 => '.',
                        _ => ' ',
                    }
                })
                .collect();
            println!("{t:>6} {line}");
        }
    }
    println!("   MHz  bursts  total ms  peak dB over floor");
    for (mhz, (n, ms, peak)) in per_mhz {
        if n >= 5 {
            println!("  {mhz:>5}  {n:>6}  {ms:>8}  {peak:>6.1}");
        }
    }
}
