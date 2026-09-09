//! Look for Remote ID in a recorded 20 MHz Wi-Fi capture: every vendor
//! specific element a beacon carries, and every action frame, so a capture
//! that holds none says so by naming what it does hold.
fn main() {
    use common::device::Device;
    let path = std::env::args().nth(1).expect("a capture");
    let src = sources::FileSource::open(&path).expect("open");
    let buf = src.read_all().expect("read");
    let rate = src.rate().as_f64();

    println!(
        "{} samples, {:.1} s at {} MS/s, centre {} MHz",
        buf.samples.len(),
        buf.samples.len() as f64 / rate,
        rate / 1e6,
        src.center().as_f64() / 1e6
    );
    let mut det = dsp::wifi::WifiSpan::new(
        rate,
        src.center().as_f64(),
        &nodes::wifi_nodes::starting_channels(),
        Default::default(),
    )
    .expect("a channel in the span");
    let mut frames = Vec::new();
    for block in buf.samples.chunks(16_384) {
        det.process(block, &mut frames);
    }
    det.flush(&mut frames);

    let mut ouis: std::collections::BTreeMap<String, usize> = Default::default();
    let mut nets: std::collections::BTreeMap<String, usize> = Default::default();
    let (mut actions, mut odid) = (0usize, 0usize);
    for f in frames.iter().filter(|f| f.fcs_ok) {
        let Some(m) = decode::wifi::parse(&f.psdu) else {
            continue;
        };
        for v in &m.vendor {
            let key = format!(
                "{:02X}:{:02X}:{:02X} type {:02X}",
                v.oui[0], v.oui[1], v.oui[2], v.kind
            );
            *ouis.entry(key).or_default() += 1;
            if let Some(msgs) = decode::odid::from_vendor_element(v.oui, v.kind, &v.data) {
                odid += 1;
                println!("beacon Remote ID: {:?}", decode::odid::fields(&msgs));
            }
        }
        if let Some(n) = &m.network {
            let who = m.source().map(|a| a.to_string()).unwrap_or_default();
            let ssid = n.ssid.clone().unwrap_or_else(|| "<hidden>".into());
            *nets.entry(format!("{ssid}  {who}")).or_default() += 1;
        }
        if let Some(a) = &m.action {
            actions += 1;
            if let Some(msgs) = decode::odid::from_nan_action(a.category, a.code, &a.body) {
                odid += 1;
                println!("NAN Remote ID: {:?}", decode::odid::fields(&msgs));
            }
        }
    }
    let ok = frames.iter().filter(|f| f.fcs_ok).count();
    println!(
        "{} bursts read, {ok} with a good FCS, {actions} action frames, {odid} carrying Remote ID",
        frames.len()
    );
    for (oui, n) in ouis {
        println!("  {oui}: {n}");
    }
    // The networks are worth printing even when no aircraft is in the file:
    // a capture that names nothing is a receiver that heard nothing, which
    // is a different problem from a transmitter that said nothing.
    for (net, n) in nets {
        println!("  {n:>4}  {net}");
    }
}
