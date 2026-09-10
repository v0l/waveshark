//! Key a channel the way the receiver does, with no window in the way.
//!
//! Usage: `key_test [freq_hz] [seconds] [gain_db]`
//!
//! The point is the loop: a receive block is read, the transmitter is pumped
//! with a block of the same length, and both go on at once, which is what the
//! radio thread does and what a bench test of the modulator alone does not.
//! If a transmission dies half a second in, it dies here too, with numbers.

use common::{Device, GainMode, Hz, Sps};

fn main() -> common::Result<()> {
    tracing_subscriber::fmt()
        .with_env_filter(tracing_subscriber::EnvFilter::from_default_env())
        .init();
    let mut args = std::env::args().skip(1);
    let freq: u64 = args.next().and_then(|s| s.parse().ok()).unwrap_or(433_920_000);
    let secs: f64 = args.next().and_then(|s| s.parse().ok()).unwrap_or(5.0);
    let gain: f32 = args.next().and_then(|s| s.parse().ok()).unwrap_or(0.0);

    let mut dev: Box<dyn Device> = Box::new(hackrf::HackRfDevice::open_first()?);
    dev.set_rate(Sps(2_000_000))?;
    dev.set_center(Hz(freq))?;
    dev.set_gain("tuner", GainMode::Manual(32.0))?;
    let mut rx = dev.start_rx()?;

    for name in ["amp", "txvga"] {
        let db = if name == "amp" { 0.0 } else { gain };
        dev.set_tx_gain(name, GainMode::Manual(db))?;
    }
    // A tone rather than a microphone, so what is measured is the path and
    // not the room. The same three stages the receiver keys with.
    let rate = dev.rate().as_f64();
    let input = pipeline::port::StreamSpec {
        kind: pipeline::PortKind::Real,
        rate,
        center: Hz(freq),
        bandwidth: 6_000.0,
        flow: pipeline::port::Flow::Tx,
        ..Default::default()
    };
    // Keying a half duplex radio moves its one synthesiser, which is what
    // the receiver has to put back afterwards.
    println!("tuned at {} Hz before keying", dev.center().0);
    let mut g = pipeline::chain(
        input,
        vec![
            Box::new(nodes::ToneNode::new(1_000.0, 0.8)),
            Box::new(nodes::FmModNode::narrowband(0.0)),
            Box::new(nodes::TxSinkNode::new(dev.start_tx()?)),
        ],
    )?;

    println!("--- transmitting ---");
    let start = std::time::Instant::now();
    let (mut rx_samples, mut blocks) = (0u64, 0u64);
    while start.elapsed().as_secs_f64() < secs {
        let buf = match rx.read() {
            Ok(b) => b,
            Err(e) => {
                println!("receive stopped: {e}");
                break;
            }
        };
        rx_samples += buf.samples.len() as u64;
        let t = std::time::Instant::now();
        {
            let b = g.input_buf();
            b.clear();
            b.real_mut().resize(buf.samples.len(), 0.0);
        }
        g.run()?;
        let pump = t.elapsed().as_secs_f64();
        blocks += 1;
        if blocks % 20 == 0 {
            let id = g.order().last().map(|(i, _)| i).unwrap();
            let sink = g
                .node(id)
                .map(|n| n.as_any())
                .and_then(|a| a.downcast_ref::<nodes::TxSinkNode>())
                .unwrap();
            println!(
                "{:5.2}s rx {:9} tx {:9} unfilled {:3} refused {:3} silent {} pump {:.1} ms",
                start.elapsed().as_secs_f64(),
                rx_samples,
                sink.written(),
                sink.underruns(),
                sink.failed_blocks(),
                rx.silent(),
                pump * 1e3,
            );
        }
    }

    // Unkey, and see whether the receiver comes back: the driver has to put
    // the radio into receive again and the stream has to start delivering
    // real samples rather than the noise floor it stands in with.
    println!("--- unkeying ---");
    let id = g.order().last().map(|(i, _)| i).unwrap();
    if let Some(n) = g.node_mut(id) {
        if let Some(s) = n.as_any_mut().downcast_mut::<nodes::TxSinkNode>() {
            s.finish(std::time::Duration::from_secs(1));
        }
    }
    drop(g);

    let after = std::time::Instant::now();
    let (mut n, mut real) = (0u64, 0u64);
    while after.elapsed().as_secs_f64() < 3.0 {
        match rx.read() {
            Ok(b) => {
                n += b.samples.len() as u64;
                if !rx.silent() {
                    real += b.samples.len() as u64;
                }
                let rms = (b.samples.iter().map(|c| c.norm_sqr() as f64).sum::<f64>()
                    / b.len().max(1) as f64)
                    .sqrt();
                if n % (2_000_000 / 2) < 100_000 {
                    println!(
                        "{:5.2}s after: {n} samples, {real} real, silent {}, rms {:.5}",
                        after.elapsed().as_secs_f64(),
                        rx.silent(),
                        rms
                    );
                }
            }
            Err(e) => {
                println!("receive did not come back: {e}");
                break;
            }
        }
    }
    println!("{real} real samples in {:.1}s after unkeying", after.elapsed().as_secs_f64());
    println!("tuned at {} Hz after unkeying", dev.center().0);
    Ok(())
}
