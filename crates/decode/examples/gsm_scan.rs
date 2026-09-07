//! Find the GSM carriers in a capture and read what they say.
//!
//! Two passes over the same file. The first averages the spectrum and lists
//! the channels on the 200 kHz raster that stand above the band's floor, so a
//! wide capture does not have to be demodulated 175 times. The second runs
//! the detector on the strongest of them.
//!
//! Usage: `gsm_scan <file.cs8> [channels]`, where the centre and rate come
//! out of the filename the way `sources` reads them.

use dsp::gsm::{GsmConfig, Hit, SchDetector};


fn main() {
    let path = std::env::args().nth(1).expect("a capture");
    let top: usize = std::env::args().nth(2).and_then(|s| s.parse().ok()).unwrap_or(8);
    let buf = sources::FileSource::open(std::path::Path::new(&path))
        .expect("open")
        .read_all()
        .expect("read");
    let (rate, center) = (buf.rate.as_f64(), buf.center.as_f64());
    eprintln!(
        "{} samples, {:.3} MS/s at {:.4} MHz, {:.2} s",
        buf.samples.len(),
        rate / 1e6,
        center / 1e6,
        buf.samples.len() as f64 / rate
    );

    // Average power per FFT bin over the whole file.
    let n = 4096;
    let mut fft = dsp::Spectrum::new(n);
    let mut acc = vec![0f64; n];
    let mut frames = 0usize;
    for chunk in buf.samples.chunks_exact(n).step_by(8) {
        if !fft.process(chunk) {
            continue;
        }
        for (a, db) in acc.iter_mut().zip(fft.power_db()) {
            *a += 10f64.powf(f64::from(*db) / 10.0);
        }
        frames += 1;
    }
    for a in acc.iter_mut() {
        *a /= frames as f64;
    }

    // The power in each 200 kHz channel whose centre is in the span.
    let bin_hz = rate / n as f64;
    let mut chans: Vec<(u16, f64, f64)> = Vec::new();
    for arfcn in 0..1024u16 {
        let Some(hz) = arfcn_hz(arfcn) else { continue };
        if (hz - center).abs() > rate / 2.0 - 150_000.0 {
            continue;
        }
        let mid = ((hz - center) / bin_hz).round() as i64 + (n / 2) as i64;
        let half = (100_000.0 / bin_hz) as i64;
        let mut p = 0.0;
        for b in mid - half..=mid + half {
            if let Some(v) = acc.get(b.rem_euclid(n as i64) as usize) {
                p += v;
            }
        }
        chans.push((arfcn, hz, p));
    }
    let floor = {
        let mut v: Vec<f64> = chans.iter().map(|c| c.2).collect();
        v.sort_by(|a, b| a.partial_cmp(b).unwrap());
        v[v.len() / 2]
    };
    chans.sort_by(|a, b| b.2.partial_cmp(&a.2).unwrap());
    eprintln!("\n{} channels in the span, floor {:.1} dB", chans.len(), 10.0 * floor.log10());
    for (arfcn, hz, p) in chans.iter().take(top) {
        eprintln!("  ARFCN {arfcn:4} {:9.4} MHz  {:+6.1} dB", hz / 1e6, 10.0 * (p / floor).log10());
    }

    for (arfcn, hz, _) in chans.iter().take(top) {
        survey(*arfcn, *hz, &buf);
        let mut det = SchDetector::new(rate, center, *hz, GsmConfig::default());
        let mut out: Vec<Hit> = Vec::new();
        let mut seen_hits = 0usize;
        for block in buf.samples.chunks(65536) {
            det.process(block, &mut out);
            // Follow what the cell assigns, as the receiver does: an
            // immediate assignment names a timeslot, and reading it is the
            // only way to know one is in use.
            for h in &out[seen_hits..] {
                if let Hit::Block(b) = h {
                    let msg = decode::gsm::parse(&b.bytes)
                        .or_else(|| decode::gsm::parse_dedicated(&b.bytes));
                    if let Some(g) = msg.and_then(|m| m.grant) {
                        if g.kind.starts_with("SDCCH")
                            && g.arfcn == Some(*arfcn)
                            && g.timeslot != 0
                        {
                            det.follow(g.timeslot);
                        }
                    }
                }
            }
            seen_hits = out.len();
        }
        // Follow whatever the cell assigns, as the receiver does.
        let syncs = out.iter().filter(|h| matches!(h, Hit::Sync(_))).count();
        let blocks: Vec<_> = out
            .iter()
            .filter_map(|h| match h {
                Hit::Block(b) => Some(b),
                _ => None,
            })
            .collect();
        if syncs == 0 && blocks.is_empty() {
            eprintln!("\nARFCN {arfcn}: nothing");
            continue;
        }
        eprintln!("\nARFCN {arfcn} at {:.4} MHz: {syncs} sync, {} blocks", hz / 1e6, blocks.len());
        if let Some(Hit::Sync(s)) = out.iter().find(|h| matches!(h, Hit::Sync(_))) {
            eprintln!(
                "  BSIC {}{} frame {} offset {:.0} Hz quality {:.2}",
                s.sch.ncc, s.sch.bcc, s.sch.frame_number, s.freq_offset_hz, s.quality
            );
        }
        // Paging is not a fact about the cell but about who is being called,
        // so it is counted rather than listed one line at a time.
        let mut paged = 0usize;
        let mut permanent = 0usize;
        for b in &blocks {
            if let Some(m) = decode::gsm::parse(&b.bytes) {
                paged += m.pages.len();
                permanent += m
                    .pages
                    .iter()
                    .filter(|p| !matches!(p, decode::gsm::Identity::Tmsi(_)))
                    .count();
            }
        }
        if paged > 0 {
            eprintln!("  paged {paged} times, {permanent} by permanent identity");
        }
        // Channel grants, summarised the same way: what matters is what the
        // cell hands out and how far away the phones are, not each one.
        let mut grants: Vec<decode::gsm::Grant> = Vec::new();
        for b in &blocks {
            if let Some(g) = decode::gsm::parse(&b.bytes).and_then(|m| m.grant) {
                grants.push(g);
            }
        }
        if std::env::var("GSM_GRANTS").is_ok() {
            let mut seen: Vec<String> = Vec::new();
            for g in &grants {
                let k = format!("{} TS {} ARFCN {:?} hop {:?}", g.kind, g.timeslot, g.arfcn, g.hopping);
                if !seen.contains(&k) { eprintln!("    {k}"); seen.push(k); }
            }
        }
        if !grants.is_empty() {
            let far = grants.iter().map(|g| g.distance_m()).max().unwrap_or(0);
            let hopping = grants.iter().filter(|g| g.hopping.is_some()).count();
            let mut kinds: Vec<&str> = grants.iter().map(|g| g.kind).collect();
            kinds.sort_unstable();
            kinds.dedup();
            eprintln!(
                "  granted {} channels ({}), {hopping} hopping, furthest phone {far} m",
                grants.len(),
                kinds.join(" ")
            );
        }
        let mut seen: Vec<String> = Vec::new();
        for b in &blocks {
            let name = match decode_name(&b.bytes) {
                Some(n) => n,
                None => continue,
            };
            if !seen.contains(&name) && !name.starts_with("Paging") && !name.starts_with("Imm") {
                if std::env::var("GSM_RAW").is_ok() {
                    eprintln!("  {name}  {:02x?}", b.bytes);
                } else {
                    eprintln!("  {name}");
                }
                seen.push(name);
            }
        }
    }
}

/// Whether a channel carries GSM at all, before any decoding is attempted.
///
/// Two measurements, and between them they say what a spectrum cannot. The
/// first is how coherent the phase advance gets over a burst's length near
/// the frequency correction tone, which is what the detector triggers on.
/// The second is whether the channel's power repeats every 4.615 ms, which is
/// the TDMA frame and which nothing but GSM does: a traffic carrier shows it
/// strongly, a beacon less so because it transmits on every timeslot, and an
/// LTE or UMTS carrier not at all.
fn survey(arfcn: u16, hz: f64, buf: &common::IqBuf) {
    let rate = buf.rate.as_f64();
    let center = buf.center.as_f64();
    let factor = (rate / (dsp::gsm::SYMBOL_RATE * 4.0)).floor().max(1.0) as usize;
    let work = rate / factor as f64;
    let sps = work / dsp::gsm::SYMBOL_RATE;
    let mut mixer = dsp::Mixer::new(center - hz, rate);
    let mut decim = dsp::FirDecim::design_hz(rate, factor, 110_000.0, 60.0);
    let mut mixed = Vec::new();
    let mut chan: Vec<common::C32> = Vec::new();
    mixer.process(&buf.samples, &mut mixed);
    decim.process(&mixed, &mut chan);

    // Coherence of the phase advance over one burst, at the best place in
    // the capture, restricted to advances near the tone.
    let len = (148.0 * sps) as usize;
    let want = std::f64::consts::TAU * dsp::gsm::FCCH_TONE_HZ / work;
    let tol = std::f64::consts::TAU * 30_000.0 / work;
    let (mut sr, mut si, mut power) = (0.0f64, 0.0f64, 0.0f64);
    let mut best = 0.0f64;
    for i in 1..chan.len() {
        let p = chan[i] * chan[i - 1].conj();
        sr += f64::from(p.re);
        si += f64::from(p.im);
        power += f64::from(p.norm());
        if i > len {
            let d = chan[i - len] * chan[i - len - 1].conj();
            sr -= f64::from(d.re);
            si -= f64::from(d.im);
            power -= f64::from(d.norm());
        } else {
            continue;
        }
        if (si.atan2(sr) - want).abs() < tol && power > 0.0 {
            best = best.max(sr.hypot(si) / power);
        }
    }

    // How much the power envelope repeats a TDMA frame later, against a lag
    // that means nothing.
    let env: Vec<f64> = chan.iter().map(|c| f64::from(c.norm_sqr())).collect();
    let mean = env.iter().sum::<f64>() / env.len() as f64;
    let dev: Vec<f64> = env.iter().map(|v| v - mean).collect();
    let norm: f64 = dev.iter().map(|v| v * v).sum();
    let at = |lag: f64| {
        let l = lag as usize;
        dev[..dev.len() - l].iter().zip(&dev[l..]).map(|(a, b)| a * b).sum::<f64>() / norm
    };
    let frame = 4.615e-3 * work;
    eprintln!(
        "ARFCN {arfcn:4}: tone coherence {best:.2}, frame repeat {:+.2} against {:+.2} at a lag that means nothing",
        at(frame),
        at(frame * 0.37)
    );
}

/// The message a block holds/// The message a block holds, rendered the way the packet list would.
fn decode_name(bytes: &[u8]) -> Option<String> {
    let m = decode::gsm::parse(bytes).or_else(|| decode::gsm::parse_dedicated(bytes))?;
    let mut s = m.name.to_string();
    if let Some(lai) = m.lai {
        s.push_str(&format!(" {lai} LAC {}", lai.lac));
    }
    if let Some(id) = m.cell_id {
        s.push_str(&format!(" CI {id}"));
    }
    if let Some(g) = m.grant {
        s.push_str(&format!(" {} TS {} TA {}", g.kind, g.timeslot, g.timing_advance));
        if let Some(n) = g.arfcn {
            s.push_str(&format!(" ARFCN {n}"));
        }
        if let Some((maio, hsn)) = g.hopping {
            s.push_str(&format!(" MAIO {maio} HSN {hsn}"));
        }
    }
    for p in &m.pages {
        s.push_str(&format!(" {p}"));
    }
    if let Some(id) = &m.identity {
        s.push_str(&format!(" {id}"));
    }
    if let Some((_, ta)) = m.sacch {
        s.push_str(&format!(" phone {} m away", u32::from(ta) * 554));
    }
    if !m.channels.is_empty() {
        s.push_str(&format!(
            " {} {}",
            if m.channels_are_neighbours { "neighbours" } else { "allocation" },
            m.channels.iter().map(|n| n.to_string()).collect::<Vec<_>>().join(",")
        ));
    }
    Some(s)
}

/// Downlink frequency for a channel number, for the bands a capture is
/// likely to be in.
fn arfcn_hz(arfcn: u16) -> Option<f64> {
    let hz = match arfcn {
        1..=124 => 935.2e6 + f64::from(arfcn - 1) * 200e3,
        975..=1023 => 925.2e6 + f64::from(arfcn - 975) * 200e3,
        0 => 934.8e6,
        128..=251 => 869.2e6 + f64::from(arfcn - 128) * 200e3,
        512..=885 => 1805.2e6 + f64::from(arfcn - 512) * 200e3,
        _ => return None,
    };
    Some(hz)
}
