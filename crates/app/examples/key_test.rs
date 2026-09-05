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
    let mut g = pipeline::chain(
        input,
        vec![
            Box::new(nodes::ToneNode::new(1_000.0, 0.8)),
            Box::new(nodes::FmModNode::narrowband(0.0)),
            Box::new(nodes::TxSinkNode::new(dev.start_tx()?)),
        ],
    )?;

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
                .and_then(|n| n.as_any())
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
    Ok(())
}
