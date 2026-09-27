use common::C32;

fn main() {
    let path = std::env::args().nth(1).expect("a .cs8 recording at 20 MS/s");
    let center: f64 = std::env::args().nth(2).map_or(5_805e6, |s| s.parse().expect("hertz"));
    let bytes = std::fs::read(&path).expect("read");
    let iq: Vec<C32> = bytes
        .chunks_exact(2)
        .map(|c| C32::new(c[0] as i8 as f32 / 128.0, c[1] as i8 as f32 / 128.0))
        .collect();
    let mut span = dsp::artosyn::Span::new(20e6, center, &[center]).expect("the channel fits");
    let mut out = Vec::new();
    let t = std::time::Instant::now();
    for b in iq.chunks(200_000) {
        span.process(b, &mut out);
    }
    span.flush(&mut out);
    let took = t.elapsed().as_secs_f64();
    for r in &out {
        println!(
            "{:.1} MHz frames {} in {:.3} s  snr {:.1} dB  rssi {:.1} dBFS  offset {:.0} Hz  {:?}  MER {:?}  balance {:?}  counter {:?}  uplinks {} at {:?}",
            r.center_hz / 1e6,
            r.frames,
            r.seconds,
            r.snr_db,
            r.rssi_dbfs,
            r.offset_hz,
            r.constellation,
            r.mer_db,
            r.balance_db,
            r.counter,
            r.uplinks,
            r.uplink_offset_hz
        );
    }
    println!("{:.2} s of air in {took:.2} s", iq.len() as f64 / 20e6);
}
