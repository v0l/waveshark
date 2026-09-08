//! Read 802.11 frames out of a recorded 20 MHz capture.
fn main() {
    use common::device::Device;
    let path = std::env::args().nth(1).expect("a capture");
    let src = sources::FileSource::open(&path).expect("open");
    let buf = src.read_all().expect("read");
    let rate = src.rate().as_f64();
    println!("{} samples at {rate}", buf.samples.len());

    // How many places look like a short training field, before any decoding.
    let s = &buf.samples;
    let (win, lag) = (48usize, 16usize);
    let mut best = 0.0f32;
    let mut plateaus = 0usize;
    let mut run = 0usize;
    for n in 0..s.len().saturating_sub(win + lag) {
        let mut c = common::C32::default();
        let mut p = 0.0f32;
        for k in 0..win {
            c += s[n + k] * s[n + k + lag].conj();
            p += s[n + k + lag].norm_sqr();
        }
        let m = if p > 0.0 { c.norm_sqr() / (p * p) } else { 0.0 };
        best = best.max(m);
        if m > 0.4 {
            run += 1;
            if run == 24 {
                plateaus += 1;
            }
        } else {
            run = 0;
        }
    }
    println!("plateaus={plateaus} best_metric={best:.3}");

    for conj in [false, true] {
        let samples: Vec<common::C32> = if conj {
            buf.samples.iter().map(|x| x.conj()).collect()
        } else {
            buf.samples.clone()
        };
        let mut cfg: dsp::wifi::WifiConfig = Default::default();
        if let Ok(v) = std::env::var("MINSNR") {
            cfg.min_level_db = v.parse().unwrap();
        }
        if let Ok(v) = std::env::var("DETECT") {
            cfg.detect = v.parse().unwrap();
        }
        let mut det = dsp::wifi::WifiSpan::new(
            rate,
            src.center().as_f64(),
            &nodes::wifi_nodes::channels(),
            cfg,
        )
        .expect("a channel in the span");
        println!("channels: {:?}", det.channels().iter().map(|c| c / 1e6).collect::<Vec<_>>());
        let mut frames = Vec::new();
        for block in samples.chunks(16_384) {
            det.process(block, &mut frames);
        }
        let ok = frames.iter().filter(|f| f.fcs_ok).count();
        println!("conj={conj}: {} frames, {ok} with a good FCS", frames.len());
        for f in frames.iter().take(20) {
            let mac = decode::wifi::parse(&f.psdu);
            println!(
                "  {:>8.0} MHz {:>10} {:>5} B fcs={} err={:.3} snr={:.1} rssi={:.1} off={:.0} Hz  {}",
                f.center_hz / 1e6,
                f.rate.label(),
                f.psdu.len(),
                f.fcs_ok,
                f.bit_err,
                f.snr_db,
                f.rssi_dbfs,
                f.freq_off_hz,
                match &mac {
                    Some(m) => format!(
                        "{} {} -> {} {:?}",
                        m.kind.name(),
                        m.source().map(|a| a.to_string()).unwrap_or_default(),
                        m.addr1,
                        m.network.as_ref().and_then(|n| n.ssid.clone())
                    ),
                    None => "unparsed".into(),
                }
            );
        }
    }
}
