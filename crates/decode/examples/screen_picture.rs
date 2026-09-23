//! Read a capture as a screen and write what it paints.

use common::{C32, SampleFormat};
use decode::display;
use decode::videoleak::Reader;
use std::io::Read;

fn main() {
    let path = std::env::args().nth(1).expect("a capture");
    let rate: f64 = std::env::args().nth(2).map_or(20e6, |s| s.parse().unwrap());
    let mode = std::env::args().nth(3);
    let out = std::env::args().nth(4).unwrap_or_else(|| "/tmp/screen.pgm".into());

    let mut f = std::fs::File::open(&path).expect("open");
    let mut raw = Vec::new();
    f.read_to_end(&mut raw).expect("read");
    let mut iq: Vec<C32> = Vec::new();
    SampleFormat::Cs16.convert(&raw, &mut iq);
    println!("{} samples at {rate}", iq.len());

    let mut reader = Reader::new(rate);
    if let Some(label) = mode.as_deref().filter(|m| *m != "auto") {
        let m = display::by_label(label).expect("a mode in the table");
        reader.force(Some(m));
        println!("forced {}", m.label());
    }
    let mut last = None;
    for block in iq.chunks(131_072) {
        if let Some(p) = reader.push(block).picture {
            last = Some(p);
        }
    }
    match (reader.locked(), last) {
        (Some(l), Some(p)) => {
            println!(
                "locked: {} at {:.4} Hz, {} lines, {:.3} kHz a line, held {:?}",
                l.label(),
                l.frame_hz,
                l.periods.lines,
                l.line_hz / 1e3,
                reader.held()
            );
            let mut pgm = format!("P5 {} {} 255\n", p.width, p.height).into_bytes();
            pgm.extend_from_slice(&p.gray);
            std::fs::write(&out, pgm).expect("write");
            println!("wrote {out} ({}x{})", p.width, p.height);
        }
        _ => println!("no picture"),
    }
}
