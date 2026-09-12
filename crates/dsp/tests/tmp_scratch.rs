use dsp::msk::{MskConfig, MskDemod};

fn wav(path: &str) -> (usize, f64, Vec<f32>) {
    let d = std::fs::read(path).expect("read");
    let channels = u16::from_le_bytes([d[22], d[23]]) as usize;
    let rate = u32::from_le_bytes([d[24], d[25], d[26], d[27]]) as f64;
    let mut at = 12;
    loop {
        let id = &d[at..at + 4];
        let len = u32::from_le_bytes([d[at + 4], d[at + 5], d[at + 6], d[at + 7]]) as usize;
        if id == b"data" {
            let body = &d[at + 8..at + 8 + len];
            let s = body
                .chunks_exact(2)
                .map(|c| i16::from_le_bytes([c[0], c[1]]) as f32 / 32768.0)
                .collect();
            return (channels, rate, s);
        }
        at += 8 + len;
    }
}

/// Assemble bytes least significant bit first from a bit offset.
fn bytes_at(bits: &[bool], start: usize, n: usize, flip: bool) -> Vec<u8> {
    let mut out = Vec::new();
    for i in 0..n {
        let mut b = 0u8;
        for k in 0..8 {
            let v = bits.get(start + i * 8 + k).copied().unwrap_or(false) != flip;
            b |= (v as u8) << k;
        }
        out.push(b);
    }
    out
}

#[test]
fn scratch() {
    let (channels, rate, samples) = wav("/tmp/acars_test.wav");
    eprintln!("{channels} channels at {rate} Hz");
    for c in 0..channels {
        let audio: Vec<f32> = samples.iter().skip(c).step_by(channels).copied().collect();
        let mut bits = Vec::new();
        MskDemod::new(rate, MskConfig::ACARS).process(&audio, &mut bits);
        let mut hits = 0;
        for flip in [false, true] {
            for at in 0..bits.len().saturating_sub(24) {
                let b = bytes_at(&bits, at, 3, flip);
                if b == [0x16, 0x16, 0x01] {
                    hits += 1;
                    let text = bytes_at(&bits, at + 24, 40, flip);
                    let printable: String = text
                        .iter()
                        .map(|x| (x & 0x7f) as char)
                        .map(|c| if c.is_ascii_graphic() || c == ' ' { c } else { '.' })
                        .collect();
                    eprintln!("ch{} at {at} flip {flip}: {printable}", c + 1);
                }
            }
        }
        eprintln!("ch{}: {} bits, {hits} syncs", c + 1, bits.len());
    }
}
