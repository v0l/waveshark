//! Run the BLE front end over a capture and print what Open Drone ID is in it.
//!     odid_probe <file.cs8|cu8> <rate> <center_hz> [out.cs8]
//!
//! With an output path, the samples every Open Drone ID packet was read from
//! are written out in the input's own format, each burst with a little
//! silence either side so a detector still has a floor to measure against.
//! What comes back through the same front end is what went in: the point of
//! the cut is a fixture small enough to commit to a manifest, not a different
//! recording.
use common::C32;
use dsp::{BleConfig, BleDetector};

fn main() {
    let a: Vec<String> = std::env::args().collect();
    let path = &a[1];
    let rate: f64 = a[2].parse().unwrap();
    let center: f64 = a[3].parse().unwrap();
    let signed = !path.ends_with(".cu8");
    let bytes = std::fs::read(path).unwrap();
    let iq: Vec<C32> = bytes
        .chunks_exact(2)
        .map(|c| {
            if signed {
                C32::new(c[0] as i8 as f32 / 128.0, c[1] as i8 as f32 / 128.0)
            } else {
                C32::new((c[0] as f32 - 127.5) / 127.5, (c[1] as f32 - 127.5) / 127.5)
            }
        })
        .collect();
    eprintln!("{:.2} s at {rate} S/s, centre {center}", iq.len() as f64 / rate);

    // The data channels are where an extended advertisement puts its
    // payload, so they are read when asked for: ODID_DATA=1.
    let cfg = BleConfig {
        data_channels: std::env::var("ODID_DATA").is_ok(),
        max_burst_us: std::env::var("ODID_MAXBURST")
            .ok()
            .and_then(|v| v.parse().ok())
            .unwrap_or(BleConfig::default().max_burst_us),
        ..BleConfig::default()
    };
    let mut det = BleDetector::new(rate, center, cfg);
    eprintln!("channels in span: {:?}", det.channels());
    let mut frames = Vec::new();
    for chunk in iq.chunks(1 << 20) {
        det.process(chunk, &mut frames);
    }
    let coded = frames.iter().filter(|f| f.coding.is_some()).count();
    eprintln!("{} packets passed CRC, {coded} of them Bluetooth 5 Long Range", frames.len());

    let mut drones = 0usize;
    let mut keep: Vec<(u64, u64)> = Vec::new();
    for f in &frames {
        let Some(adv) = decode::ble::parse(&f.pdu) else {
            continue;
        };
        let msgs: Vec<_> = adv
            .data
            .iter()
            .filter(|s| s.kind == 0x16)
            .filter_map(|s| decode::odid::from_service_data(&s.value))
            .flatten()
            .collect();
        if msgs.is_empty() {
            if f.coding.is_some() {
                println!(
                    "  LR pdu {}",
                    f.pdu.iter().map(|b| format!("{b:02x}")).collect::<String>()
                );
            }
            continue;
        }
        drones += 1;
        // A fixture for the long range path wants the long range packets and
        // not the two hundred legacy ones alongside them: ODID_ONLY_LR=1.
        if std::env::var("ODID_ONLY_LR").is_ok() && f.coding.is_none() {
            continue;
        }
        // A legacy advertisement is at most 47 bytes at a microsecond a bit,
        // so 400 us covers the longest one; 2 ms of margin either side is the
        // floor the detector takes its noise estimate from.
        let margin = (0.002 * rate) as u64;
        // A long range packet carries the same PDU eight or two times more
        // slowly, so its samples run that much longer and a window cut for an
        // uncoded advertisement would take the front of it and nothing else.
        let spread = match f.coding {
            Some(dsp::ble_coded::Coding::S8) => 8.0,
            Some(dsp::ble_coded::Coding::S2) => 2.0,
            None => 1.0,
        };
        let len = ((f.pdu.len() + 16) as f64 * 8.0 * spread * 1e-6 * rate) as u64;
        keep.push((f.start_sample.saturating_sub(margin), f.start_sample + len + margin));
        let fields = decode::odid::fields(&msgs)
            .iter()
            .map(|(k, v)| format!("{k}={v}"))
            .collect::<Vec<_>>()
            .join(" ");
        let phy = match f.coding {
            Some(dsp::ble_coded::Coding::S8) => " LR/S8",
            Some(dsp::ble_coded::Coding::S2) => " LR/S2",
            None => "",
        };
        println!("ch{}{phy} {:.0} dBFS {} {}", f.channel, f.rssi_dbfs, adv.address, fields);
    }
    eprintln!("{drones} of {} packets are Open Drone ID", frames.len());

    let Some(out) = a.get(4) else { return };
    // Overlapping windows become one, so two packets 3 ms apart stay one
    // burst with the gap they actually had rather than being cut and butted
    // back together at a discontinuity.
    keep.sort();
    let mut merged: Vec<(u64, u64)> = Vec::new();
    for (s, e) in keep {
        match merged.last_mut() {
            Some(last) if s <= last.1 => last.1 = last.1.max(e),
            _ => merged.push((s, e)),
        }
    }
    let mut bytes_out: Vec<u8> = Vec::new();
    let mut total = 0u64;
    for (s, e) in &merged {
        let (s, e) = (*s as usize * 2, (*e as usize * 2).min(bytes.len()));
        if s >= e {
            continue;
        }
        total += (e - s) as u64 / 2;
        bytes_out.extend_from_slice(&bytes[s..e]);
    }
    std::fs::write(out, &bytes_out).unwrap();
    eprintln!(
        "wrote {out}: {} bursts, {:.3} s, {:.1} MB, {:.0}x smaller",
        merged.len(),
        total as f64 / rate,
        bytes_out.len() as f64 / 1e6,
        bytes.len() as f64 / bytes_out.len() as f64
    );
}
