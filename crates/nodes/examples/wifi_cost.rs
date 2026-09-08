//! What the Wi-Fi front end costs against real time.
fn main() {
    use common::device::Device;
    let path = std::env::args().nth(1).expect("a capture");
    let src = sources::FileSource::open(&path).expect("open");
    let buf = src.read_all().expect("read");
    let rate = src.rate().as_f64();
    let secs = buf.samples.len() as f64 / rate;
    let mut det = dsp::wifi::WifiSpan::new(
        rate,
        src.center().as_f64(),
        &match std::env::var("WIFICH") {
            Ok(v) => v.split(',').filter_map(|c| c.parse().ok()).collect::<Vec<f64>>(),
            Err(_) => nodes::wifi_nodes::channels(),
        },
        Default::default(),
    )
    .expect("a channel in the span");
    println!("{} channels", det.channels().len());
    let mut frames = Vec::new();
    let t = std::time::Instant::now();
    for block in buf.samples.chunks(16_384) {
        det.process(block, &mut frames);
    }
    det.flush(&mut frames);
    let el = t.elapsed().as_secs_f64();
    println!(
        "{} frames, {:.3}s of air in {:.3}s = {:.2}x real time",
        frames.len(),
        secs,
        el,
        secs / el
    );
}
