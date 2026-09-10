//! Run a user-specified chain over a capture file.
//!
//! The chain is given on the command line, so an ambiguous signal can be
//! attacked by trying different chains without recompiling:
//!
//!   chain <file> 'envelope | pulse_detect:reset_us=10000,min_pulses=20 | protocol_decode'
//!   chain <file> 'fm_demod:deviation_hz=5000 | pulse_detect | protocol_decode'
//!
//! With no chain argument it lists the available nodes and their parameters.

use nodes::{build_chain, registry, NodeSpec};
use pipeline::event::Event;
use pipeline::{ParamValue, StreamSpec};
use sources::FileSource;

fn parse_chain(s: &str) -> Result<Vec<NodeSpec>, String> {
    s.split('|')
        .map(str::trim)
        .filter(|p| !p.is_empty())
        .map(|part| {
            let (kind, args) = match part.split_once(':') {
                Some((k, a)) => (k.trim(), a),
                None => (part, ""),
            };
            let mut spec = NodeSpec::new(kind);
            for kv in args.split(',').map(str::trim).filter(|s| !s.is_empty()) {
                let (k, v) = kv
                    .split_once('=')
                    .ok_or_else(|| format!("expected key=value, got {kv:?}"))?;
                // Type is inferred from the literal: `true`/`false` are bools,
                // anything with a dot is a float, otherwise an integer.
                let val = if v == "true" || v == "false" {
                    ParamValue::Bool(v == "true")
                } else if v.contains('.') {
                    ParamValue::Float(v.parse().map_err(|_| format!("bad number {v:?}"))?)
                } else {
                    ParamValue::Int(v.parse().map_err(|_| format!("bad number {v:?}"))?)
                };
                spec = spec.set(k.trim(), val);
            }
            Ok(spec)
        })
        .collect()
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let reg = registry();
    let args: Vec<String> = std::env::args().collect();

    if args.len() < 3 {
        println!("usage: chain <capture file> '<node> | <node>:k=v,k=v | ...'\n");
        println!("available nodes:");
        let mut cat = None;
        for d in reg.list() {
            if cat != Some(d.category) {
                cat = Some(d.category);
                println!("\n  [{}]", d.category);
            }
            println!("    {:<18} {}", d.name, d.summary);
        }
        println!("\nexample:");
        println!("  chain testdata/fineoffset_wh1080_433.92M_250k.cu8 \\");
        println!("    'envelope | pulse_detect:reset_us=10000,min_pulses=20 | protocol_decode'");
        return Ok(());
    }

    let src = FileSource::open(&args[1])?;
    let buf = src.read_all()?;
    let specs = parse_chain(&args[2]).map_err(|e| format!("bad chain: {e}"))?;

    let spec = StreamSpec::iq(buf.rate.as_f64(), buf.center);
    let mut g = build_chain(spec, &specs, &reg)?;

    println!(
        "input:  {} samples @ {} centred {}",
        buf.len(),
        buf.rate,
        buf.center
    );
    print!("chain: ");
    for (i, (id, name)) in g.order().enumerate() {
        if i > 0 {
            print!(" -> ");
        }
        print!("{name}");
        let _ = id;
    }
    println!(
        "\noutput: {:?} @ {:.0} S/s, latency {} samples\n",
        g.output_spec().kind,
        g.output_spec().rate,
        g.output_latency()
    );

    // Show each node's live parameters, which is what a UI would render.
    for (id, name) in g.order().collect::<Vec<_>>() {
        let params = g.node(id).unwrap().params();
        if !params.is_empty() {
            let list: Vec<String> = params
                .iter()
                .map(|p| format!("{}={}", p.name, fmt_value(&p.value)))
                .collect();
            println!("  {name}: {}", list.join(" "));
        }
    }

    let events = g.feed_iq(&buf.samples)?.to_vec();
    println!("\n--- {} event(s) ---", events.len());
    for e in &events {
        let stage = g.label(e.node).unwrap_or("?");
        match &e.event {
            Event::Decoded(d) => println!("DECODE  {}", d.text.as_deref().unwrap_or(d.protocol)),
            Event::Warning { message } => println!("warn    [{stage}] {message}"),
            Event::Detection { center, snr_db, .. } => {
                println!("detect  {center} {snr_db:.1} dB")
            }
            other => println!("event   {other:?}"),
        }
    }
    Ok(())
}

fn fmt_value(v: &ParamValue) -> String {
    match v {
        ParamValue::Float(f) => format!("{f}"),
        ParamValue::Int(i) => format!("{i}"),
        ParamValue::Bool(b) => format!("{b}"),
        ParamValue::Text(t) => t.clone(),
        ParamValue::Choice(c) => format!("{c}"),
    }
}
