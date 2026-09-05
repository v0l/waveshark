//! Transmit narrowband FM from the graph, into a real radio.
//!
//! Usage: `nfm_tx [freq_hz] [seconds] [txvga_db] [tone_hz]`
//!
//! The same chain the round trip test runs, with a HackRF where the file
//! sink is: tone, `fm_mod` at 2.5 kHz deviation, `radio_tx`. Nothing here
//! paces the loop, because nothing has to: the sink blocks once the radio
//! has enough queued, so the graph runs at the sample rate.
//!
//! This radiates. Into a dummy load, at the bottom of the gain range, on a
//! frequency you are allowed to use.

use common::{Device, GainMode, Hz, Sps};
use nodes::{FmModNode, ToneNode, TxSinkNode};
use pipeline::port::{Flow, PortKind, StreamSpec};
use pipeline::chain;

/// The radio runs well above the audio rate, so the modulator is fed at the
/// transmit rate and the tone is generated there: no resampler exists on this
/// side of the graph yet, and inventing one here would hide that.
const RATE: f64 = 2_000_000.0;
const BLOCK: usize = 32_768;

fn main() -> common::Result<()> {
    let mut args = std::env::args().skip(1);
    let freq: u64 = args.next().and_then(|s| s.parse().ok()).unwrap_or(433_920_000);
    let secs: f64 = args.next().and_then(|s| s.parse().ok()).unwrap_or(2.0);
    let txvga: f32 = args.next().and_then(|s| s.parse().ok()).unwrap_or(0.0);
    let tone: f64 = args.next().and_then(|s| s.parse().ok()).unwrap_or(1_000.0);

    let mut dev = hackrf::HackRfDevice::open_first()?;
    assert!(dev.info().covers_tx(Hz(freq)), "{freq} Hz is outside the transmit range");
    dev.set_rate(Sps(RATE as u64))?;
    dev.set_center(Hz(freq))?;
    dev.set_tx_gain("amp", GainMode::Manual(0.0))?;
    dev.set_tx_gain("txvga", GainMode::Manual(txvga))?;

    let input = StreamSpec {
        kind: PortKind::Real,
        rate: RATE,
        center: Hz(freq),
        bandwidth: 6_000.0,
        flow: Flow::Tx,
        ..Default::default()
    };
    let mut g = chain(
        input,
        vec![
            Box::new(ToneNode::new(tone, 0.8)),
            Box::new(FmModNode::narrowband(0.0)),
            Box::new(TxSinkNode::new(dev.start_tx()?)),
        ],
    )?;

    println!(
        "transmitting {secs:.1} s of NFM at {:.4} MHz, {tone} Hz tone, TXVGA {txvga} dB",
        freq as f64 / 1e6
    );
    let blocks = (secs * RATE / BLOCK as f64).ceil() as usize;
    let start = std::time::Instant::now();
    for _ in 0..blocks {
        let buf = g.input_buf();
        buf.clear();
        buf.real_mut().resize(BLOCK, 0.0);
        g.run()?;
    }

    let id = g.order().last().map(|(id, _)| id).unwrap();
    let mut sent = 0;
    let mut idle = 0;
    if let Some(n) = g.node_mut(id) {
        if let Some(tx) = n.as_any_mut().and_then(|a| a.downcast_mut::<TxSinkNode>()) {
            tx.finish(std::time::Duration::from_secs(2));
            sent = tx.written();
            idle = tx.underruns();
        }
    }
    println!(
        "sent {sent} samples in {:.2} s, {idle} unfilled transfers",
        start.elapsed().as_secs_f64()
    );
    Ok(())
}
