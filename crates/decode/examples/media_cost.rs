#[cfg(not(target_os = "linux"))]
fn main() {
    eprintln!("media_cost reads the process clock through getrusage, which it only does on Linux");
}

#[cfg(target_os = "linux")]
fn cpu_seconds() -> f64 {
    let mut u: libc::rusage = unsafe { std::mem::zeroed() };
    unsafe { libc::getrusage(libc::RUSAGE_SELF, &mut u) };
    let s = |t: libc::timeval| t.tv_sec as f64 + t.tv_usec as f64 * 1e-6;
    s(u.ru_utime) + s(u.ru_stime)
}

#[cfg(target_os = "linux")]
fn main() {
    use decode::media::{Decoding, Media, Out};
    let mut args = std::env::args().skip(1);
    let path = args.next().expect("a transport stream");
    let seconds: f64 = args.next().and_then(|s| s.parse().ok()).expect("its length in seconds");
    let decoding = match args.next().as_deref() {
        Some("software") => Decoding::Software,
        _ => Decoding::Hardware,
    };
    let ts = std::fs::read(&path).expect("readable");
    let block = 188 * 64;
    let blocks = ts.len().div_ceil(block);
    let per_block = common::time::Duration::from_secs_f64(seconds / blocks as f64);

    let mut media = Media::decoding(decoding);
    let mut out = Vec::new();
    let mut pictures = 0usize;
    let cpu = cpu_seconds();
    let start = common::time::Instant::now();
    for (i, b) in ts.chunks(block).enumerate() {
        media.push(b);
        media.take(&mut out);
        pictures += out.iter().filter(|o| matches!(o, Out::Picture(_))).count();
        out.clear();
        let due = per_block * (i as u32 + 1);
        if let Some(wait) = due.checked_sub(start.elapsed()) {
            std::thread::sleep(wait);
        }
    }
    media.finish(&mut out);
    pictures += out.iter().filter(|o| matches!(o, Out::Picture(_))).count();
    let used = cpu_seconds() - cpu;
    println!(
        "{path}: {:?} on {:?}, {pictures} pictures, {:.2} cores while playing, fault {:?}",
        decoding,
        media.decoder(),
        used / seconds,
        media.fault()
    );
}
