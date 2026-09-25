use common::C32;
use decode::ft8::{Mode, PAYLOAD_BITS, read_slot, unpack, unpack_bits};

const JT9_2_7_0: [(f64, &str); 22] = [
    (2571.0, "W1FC F5BZB -08"),
    (2157.0, "WM3PEN EA6VQ -09"),
    (1197.0, "CQ F5RXL IN94"),
    (641.0, "N1JFU EA6EE R-07"),
    (723.0, "A92EE F5PSR -14"),
    (2695.0, "K1BZM EA3GP -09"),
    (400.0, "W0RSJ EA3BMU RR73"),
    (590.0, "K1JT HA0DU KN07"),
    (2733.0, "W1DIG SV9CVY -14"),
    (1648.0, "K1JT EA3AGB -15"),
    (2852.0, "XE2X HA2NP RR73"),
    (2522.0, "K1BZM EA3CJ JN01"),
    (2546.0, "WA2FZW DL5AXX RR73"),
    (2238.0, "N1API HA6FQ -23"),
    (466.0, "N1PJT HB9CQK -10"),
    (1513.0, "N1API F2VX 73"),
    (2606.0, "CQ DX DL8YHR JO41"),
    (2039.0, "K1JT HA5WA 73"),
    (472.0, "KD2UGC F6GCP R-23"),
    (2280.0, "CQ EA2BFM IN83"),
    (244.0, "K1BZM DK8NE -10"),
    (3390.0, "TU; 7N9RST EI8TRF 589 5732"),
];

fn audio() -> Option<(f64, Vec<f32>)> {
    let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../testdata/ft8_wsjtx_210703_133430_12000.wav");
    let Ok(raw) = std::fs::read(&path) else {
        eprintln!("skipping: ft8_wsjtx_210703_133430_12000.wav absent, run testdata/fetch.sh");
        return None;
    };
    let rate = u32::from_le_bytes([raw[24], raw[25], raw[26], raw[27]]) as f64;
    let mut at = 12;
    let body = loop {
        let len = u32::from_le_bytes([raw[at + 4], raw[at + 5], raw[at + 6], raw[at + 7]]) as usize;
        if &raw[at..at + 4] == b"data" {
            break &raw[at + 8..at + 8 + len];
        }
        at += 8 + len;
    };
    let samples =
        body.chunks_exact(2).map(|c| i16::from_le_bytes([c[0], c[1]]) as f32 / 32768.0).collect();
    Some((rate, samples))
}

#[test]
fn the_wsjtx_sample_reads_11_of_the_22_stations_wsjtx_2_7_0_read() {
    let Some((rate, audio)) = audio() else { return };
    let iq: Vec<C32> = audio.iter().map(|&s| C32::new(s, 0.0)).collect();
    let mut slot = dsp::mfsk::Slot::new(rate, Mode::Ft8.waveform());
    let heard: Vec<(f64, String)> = read_slot(&mut slot, &iq, Mode::Ft8)
        .iter()
        .map(|t| {
            let m = unpack(&unpack_bits(&t.bytes[1..], PAYLOAD_BITS)).expect("a message");
            (t.freq_hz, m.text)
        })
        .collect();
    for (hz, text) in &heard {
        let theirs = JT9_2_7_0.iter().find(|(_, t)| t == text);
        let (at, _) =
            theirs.unwrap_or_else(|| panic!("{text:?} at {hz:.0} Hz, which jt9 did not read"));
        assert!((hz - at).abs() < 3.0, "{text:?} at {hz:.1} Hz where jt9 put it at {at}");
    }
    let mut texts: Vec<&str> = heard.iter().map(|(_, t)| t.as_str()).collect();
    texts.sort();
    assert_eq!(
        texts,
        [
            "A92EE F5PSR -14",
            "CQ EA2BFM IN83",
            "K1JT EA3AGB -15",
            "K1JT HA0DU KN07",
            "K1JT HA5WA 73",
            "N1API HA6FQ -23",
            "N1JFU EA6EE R-07",
            "W0RSJ EA3BMU RR73",
            "W1FC F5BZB -08",
            "WM3PEN EA6VQ -09",
            "XE2X HA2NP RR73",
        ],
        "a floor, not the ceiling: jt9 from WSJT-X 2.7.0 read all 22"
    );
}
