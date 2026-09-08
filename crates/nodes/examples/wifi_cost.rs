//! What the Wi-Fi front end costs against real time.
fn main() {
    use common::device::Device;
    let path = std::env::args().nth(1).expect("a capture");
    let src = sources::FileSource::open(&path).expect("open");
    let buf = src.read_all().expect("read");
    let rate = src.rate().as_f64();
    let secs = buf.samples.len() as f64 / rate;
    let mut det = dsp::wifi::WifiDetector::new(rate, Default::default()).expect("rate");
    let mut frames = Vec::new();
    let t = std::time::Instant::now();
    for block in buf.samples.chunks(16_384) {
        det.process(block, &mut frames);
    }
    let el = t.elapsed().as_secs_f64();
    println!(
        "{} frames, {:.3}s of air in {:.3}s = {:.2}x real time",
        frames.len(),
        secs,
        el,
        secs / el
    );
}
