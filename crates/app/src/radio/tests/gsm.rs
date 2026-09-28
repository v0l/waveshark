use super::*;

/// The rule every row in the list obeys, whichever front end made it: a
/// level, a signal to noise ratio, and the samples it was read from.
///
/// Without these a row cannot be sorted by strength, a fade cannot be
/// told from a decoder that broke, and there is nothing to look at when
/// the bytes are wrong. Frames used to lose all three at the port
/// boundary, which carried bytes and nothing else, so every front end
/// that produces frames rather than pulses reported NaN.
/// A GSM beacon through the whole receiver: the scanner table puts the
/// GSM front end on the carrier, the front end finds the tone, reads the
/// burst a frame later, and the row that reaches the list names the cell.
///
/// Synthetic, and that is the weakness worth writing down: the modulator
/// here and the demodulator under test share every assumption either of
/// them makes about GSM. What it does prove is the wiring, which is where
/// a front end usually breaks: that the table's channel reaches the node,
/// that the extraction leaves the carrier inside the span it hands over,
/// and that what the node puts on the bus comes back out of the packet
/// list as a cell rather than as four unexplained bytes.
#[test]
fn a_gsm_beacon_is_read_through_the_receiver() {
    let center = Hz(947_400_000);
    let rate = 2_400_000.0;
    let want = dsp::gsm::Sch { ncc: 5, bcc: 3, frame_number: 51 * 26 * 42 + 21 };
    let buf = common::IqBuf::new(gsm_beacon(&want), center, common::Sps(rate as u64), 0);

    let mut plan = replay_plan(&buf, false);
    plan.fronts = vec![crate::scanners::FrontAt {
        front: crate::scanners::Front::protocol("gsm", center.as_f64()),
        band: (center.as_f64() - 200_000.0, center.as_f64() + 200_000.0),
    }];
    let mut rx = crate::chain::Receiver::build(&plan, crate::chain::Sinks::default()).unwrap();
    let out = replay_blocks(&mut rx, &buf);

    let cells = read_as(&out, "gsm", "sync");
    assert_eq!(cells.len(), 2, "expected both bursts, got {out:?}");
    let r = cells[0];
    assert!(checked(r), "the parity is what makes a burst a burst");
    // The base station's own name for itself, and the code that tells
    // two neighbouring sites on one frequency apart.
    assert_eq!(
        r.packet.innermost().and_then(|l| l.link.from.as_ref()).map(|p| p.label()),
        Some("ARFCN 62 BSIC 53")
    );
    let site = r.packet.facts().find_map(|(_, f)| match f {
        common::packet::Fact::Infrastructure(c) => c.site_code,
        _ => None,
    });
    // The BSIC is a network colour code and a base station code in one
    // six-bit number: 5 and 3 written as one is 43.
    assert_eq!(site, Some(43));
    every_row_carries_its_measurements(&cells);

    // And the block the broadcast channel carried in the four frames
    // after it, which is the row that says whose cell this is.
    let si = read_as(&out, "gsm", "system_information");
    assert_eq!(si.len(), 1, "expected one system information block, got {out:?}");
    // The cell as it names itself: the network, then the cell inside it.
    assert_eq!(who(si[0]).as_deref(), Some("262-01-100-4660"), "{}", si[0].detail());
    assert!(si[0].detail().contains("area 100"), "{}", si[0].detail());
    every_row_carries_its_measurements(&si);
}

/// A frequency correction burst, the synchronisation burst one TDMA
/// frame after it, and the four bursts of broadcast channel the
/// multiframe puts after that, at the receiver's rate and with a little
/// noise so the floor a level is measured against is a floor.
fn gsm_beacon(sch: &dsp::gsm::Sch) -> Vec<common::C32> {
    use dsp::gsm;
    let sps = 8;
    let work = gsm::SYMBOL_RATE * sps as f64;
    let lead = 200.0;
    let total = ((lead * 2.0 + 13.0 * gsm::FRAME_SYMBOLS) * sps as f64) as usize;
    let mut base = vec![common::C32::new(0.0, 0.0); total];
    let mut place = |at: f64, wave: &[common::C32]| {
        let at = (at * sps as f64) as usize;
        base[at..at + wave.len()].copy_from_slice(wave);
    };
    // Two beacons ten frames apart, which is what the control multiframe
    // holds and what the receiver needs: a synchronisation burst is
    // reported only once a second one agrees with it about the time.
    for n in 0..2u32 {
        let at = lead + 10.0 * f64::from(n) * gsm::FRAME_SYMBOLS;
        let this = gsm::Sch { frame_number: sch.frame_number + 10 * n, ..*sch };
        place(at, &gsm::modulate(&[0u8; gsm::BURST_BITS], sps));
        place(at + gsm::FRAME_SYMBOLS, &gsm::modulate(&gsm::sch_burst_bits(&this).unwrap(), sps));
    }
    // A system information type 3 on the broadcast channel, in the four
    // frames after the first synchronisation burst: the cell identity
    // and the location area, which is what a receiver is here for.
    let mut block = [0x2Bu8; 23];
    block[..10].copy_from_slice(&[0x49, 0x06, 0x1B, 0x12, 0x34, 0x62, 0xF2, 0x10, 0x00, 0x64]);
    for (n, data) in gsm::bcch::encode(&block).unwrap().iter().enumerate() {
        let bits = gsm::normal_burst_bits(data, usize::from(sch.bcc));
        place(lead + (2.0 + n as f64) * gsm::FRAME_SYMBOLS, &gsm::modulate(&bits, sps));
    }

    let ratio = work / 2_400_000.0;
    let n = (base.len() as f64 / ratio) as usize - 1;
    let mut seed = 0x1357_9BDFu32;
    let mut rand = move || {
        seed ^= seed << 13;
        seed ^= seed >> 17;
        seed ^= seed << 5;
        (seed as f32 / u32::MAX as f32) - 0.5
    };
    // A little noise, so the floor the level is measured against is a
    // floor rather than a divide by zero.
    (0..n)
        .map(|i| base[(i as f64 * ratio) as usize] + common::C32::new(rand(), rand()) * 0.05)
        .collect()
}
