use common::C32;
use nodes::dvbt_nodes::DvbtReceiver;

fn main() {
    let raw = std::fs::read("testdata/dvbt_hd_429M_9142857.cs8").expect("fixture");
    let samples: Vec<C32> = raw
        .chunks_exact(2)
        .map(|c| C32::new(c[0] as i8 as f32 / 127.0, c[1] as i8 as f32 / 127.0))
        .collect();
    let air = samples.len() as f64 / (64e6 / 7.0);
    let mut rx = DvbtReceiver::new();
    let mut out = Vec::new();
    let t = std::time::Instant::now();
    for block in samples.chunks(65_536) {
        rx.push(block, &mut out);
    }
    let took = t.elapsed().as_secs_f64();
    println!("whole chain {:.2}x real time, {} packets", air / took, out.len());

    // What each layer costs on its own, over the same samples.
    let mut front = dsp::dvbt::Dvbt::new();
    let mut symbols = Vec::new();
    let t = std::time::Instant::now();
    for block in samples.chunks(65_536) {
        front.push(block, &mut symbols);
        symbols.clear();
    }
    println!("front end alone {:.2}x real time", air / t.elapsed().as_secs_f64());

    // The symbols once, then the inner layers over them.
    let mut front = dsp::dvbt::Dvbt::new();
    let mut symbols = Vec::new();
    for block in samples.chunks(65_536) {
        front.push(block, &mut symbols);
    }
    let params = front.params().expect("a lock");
    let mut inner = dsp::dvbt::Inner::new(params.mode, params.constellation);
    let mut soft = Vec::new();
    let t = std::time::Instant::now();
    for s in &symbols {
        soft.clear();
        inner.demodulate(&s.cells, &s.csi, s.index.unwrap_or(0), &mut soft);
    }
    println!("demap and deinterleave {:.2}x real time", air / t.elapsed().as_secs_f64());

    let mut v = dsp::conv::Viterbi::new(dsp::conv::K7_X_FIRST);
    let mut bits = Vec::new();
    let t = std::time::Instant::now();
    for s in &symbols {
        soft.clear();
        inner.demodulate(&s.cells, &s.csi, s.index.unwrap_or(0), &mut soft);
        bits.clear();
        v.push(&soft, params.code_rate_hp.mask(), &mut bits);
    }
    println!("with the viterbi {:.2}x real time", air / t.elapsed().as_secs_f64());

    // The soft values once, then the trellis over them on its own, which is
    // the one stage that cannot be made wider by adding carriers.
    let mut all = Vec::new();
    for s in &symbols {
        soft.clear();
        inner.demodulate(&s.cells, &s.csi, s.index.unwrap_or(0), &mut soft);
        all.extend_from_slice(&soft);
    }
    let mut v = dsp::conv::Viterbi::new(dsp::conv::K7_X_FIRST);
    let mut bits = Vec::new();
    let t = std::time::Instant::now();
    v.push(&all, params.code_rate_hp.mask(), &mut bits);
    let took = t.elapsed().as_secs_f64();
    println!(
        "the trellis alone {:.2}x real time, {:.1} Msteps/s over {} soft values",
        air / took,
        bits.len() as f64 / took / 1e6,
        all.len()
    );

    // And the outer code over the bits it puts out.
    let mut bytes = Vec::with_capacity(bits.len() / 8);
    let mut acc = 0u8;
    for (n, b) in bits.iter().enumerate() {
        acc = (acc << 1) | b;
        if n % 8 == 7 {
            bytes.push(acc);
        }
    }
    let mut outer = decode::dvbt::Outer::new();
    let mut packets = Vec::new();
    let t = std::time::Instant::now();
    outer.push(&bytes, &mut packets);
    println!(
        "the outer code alone {:.2}x real time, {} packets",
        air / t.elapsed().as_secs_f64(),
        packets.len()
    );
}
