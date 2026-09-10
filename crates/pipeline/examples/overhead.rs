//! What the runner costs per node call, apart from the node.
//!
//! A chain of pass-through nodes on small blocks, run many times, against
//! the same nodes called directly. The difference is the scheduler.
//!
//!     cargo run --release -p pipeline --example overhead -- 8 256

use common::{Hz, Result, C32};
use pipeline::{chain, NodeCtx, Payload, PortSpec, Simple, StreamSpec};
use std::time::Instant;

struct Pass;
impl Simple for Pass {
    fn name(&self) -> &str {
        "pass"
    }
    fn negotiate(&mut self, i: &PortSpec) -> Result<StreamSpec> {
        Ok(i.spec)
    }
    fn process(&mut self, i: &Payload, o: &mut Payload, _c: &mut NodeCtx<'_>) -> Result<()> {
        o.iq_mut().extend_from_slice(i.as_iq().unwrap());
        Ok(())
    }
}

fn main() {
    let depth: usize = std::env::args().nth(1).and_then(|s| s.parse().ok()).unwrap_or(8);
    let block: usize = std::env::args().nth(2).and_then(|s| s.parse().ok()).unwrap_or(256);
    let iters = 200_000usize;
    let spec = StreamSpec::iq(48_000.0, Hz::mhz(100));
    let input: Vec<C32> = (0..block).map(|i| C32::new(i as f32, 0.0)).collect();

    let nodes: Vec<Box<dyn pipeline::Node>> = (0..depth).map(|_| Box::new(Pass) as _).collect();
    let mut g = chain(spec, nodes).unwrap();
    for _ in 0..1000 {
        g.feed_iq(&input).unwrap();
    }
    let t = Instant::now();
    for _ in 0..iters {
        g.feed_iq(&input).unwrap();
    }
    let graph_ns = t.elapsed().as_nanos() as f64 / (iters * depth) as f64;

    let mut direct: Vec<Pass> = (0..depth).map(|_| Pass).collect();
    let mut a = Payload::Iq(Vec::new());
    let mut b = Payload::Iq(Vec::new());
    let specs = [PortSpec { spec, latency: 0 }];
    let mut ev = Vec::new();
    let mut tg = Vec::new();
    let t = Instant::now();
    for _ in 0..iters {
        a.clear();
        a.iq_mut().extend_from_slice(&input);
        for n in &mut direct {
            b.clear();
            let mut ctx = NodeCtx::new(0, &specs, &[], &mut ev, &mut tg);
            Simple::process(n, &a, &mut b, &mut ctx).unwrap();
            std::mem::swap(&mut a, &mut b);
        }
    }
    let direct_ns = t.elapsed().as_nanos() as f64 / (iters * depth) as f64;

    println!("depth {depth}, block {block}");
    println!("graph:  {graph_ns:8.1} ns per node call");
    println!("direct: {direct_ns:8.1} ns per node call");
    println!("runner: {:8.1} ns per node call", graph_ns - direct_ns);
}
