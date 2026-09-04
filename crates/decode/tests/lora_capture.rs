//! Decode the two off-air Meshtastic transmissions in `testdata/offair`.
//!
//! These are the reason the LoRa path exists in the shape it does. Both were
//! recorded on a LimeSDR at 2 MS/s with the node's spreading factor and
//! bandwidth set by hand, so the parameters are known rather than inferred,
//! and both sat far enough under the noise that energy detection found
//! nothing at all: the whole signal is 33 dB of processing gain that does not
//! exist until the dechirp has happened.
//!
//! What is asserted is what the transmitter said, not what this code
//! produces. The header carries its own five bit checksum over the length and
//! the coding rate, and the payload carries a CRC the transmitter computed;
//! neither is anything this receiver could talk itself into. The Meshtastic
//! header on top of that is a third check from a fourth party: a broadcast
//! destination is four bytes of 0xff, and reading those out of a payload that
//! has been Gray decoded, deinterleaved, Hamming decoded and dewhitened means
//! all four transforms are right.
//!
//! Skips when the fixtures are absent, so a fresh clone passes without
//! network access. Run `testdata/fetch.sh` to get them.

use common::{SampleFormat, C32};
use decode::lora;
use dsp::fir::FirDecim;
use std::path::{Path, PathBuf};

const RATE: f64 = 2_000_000.0;
const BW: f64 = 250_000.0;
const SF: u8 = 11;

struct Expected {
    file: &'static str,
    sync_word: u8,
    length: usize,
    coding_rate: u8,
    /// The Meshtastic header's source node, little endian in the first bytes
    /// after a broadcast destination.
    source: u32,
}

/// Both nodes were transmitting on the EU868 plan through the same receiver,
/// 128 seconds apart. The lengths differ and the symbol counts do not, which
/// is what a 4/5 block boundary does and a useful thing for the symbol count
/// to be checked against.
const EXPECTED: &[Expected] = &[
    Expected {
        file: "lora_sf11_meshtastic_a_869.525M_2000k.cs16",
        sync_word: 0x2b,
        length: 55,
        coding_rate: 1,
        source: 0x426d_9eac,
    },
    Expected {
        file: "lora_sf11_meshtastic_b_869.525M_2000k.cs16",
        sync_word: 0x2b,
        length: 51,
        coding_rate: 1,
        source: 0x050d_3664,
    },
];

fn dir() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../../testdata/offair")
}

/// Bring the 2 MHz capture down to two samples per chip. The demodulator
/// wants the channel and not the band around it: at 2 MS/s a symbol is
/// sixteen thousand samples of which seven eighths are somebody else's.
fn channel(raw: &[u8]) -> Vec<C32> {
    let mut iq: Vec<C32> = Vec::new();
    SampleFormat::Cs16.convert(raw, &mut iq);
    let factor = (RATE / (BW * dsp::lora::OVERSAMPLE as f64)) as usize;
    let mut decim = FirDecim::design_hz(RATE, factor, BW / 2.0, 60.0);
    let mut out = Vec::with_capacity(iq.len() / factor + 1);
    decim.process(&iq, &mut out);
    out
}

#[test]
fn the_meshtastic_captures_decode_to_valid_frames() {
    let mut seen = 0;
    for want in EXPECTED {
        let Ok(raw) = std::fs::read(dir().join(want.file)) else {
            eprintln!("{} absent, run testdata/fetch.sh", want.file);
            continue;
        };
        seen += 1;
        let iq = channel(&raw);

        let mut demod = dsp::lora::Demod::new(dsp::lora::Config {
            sf: SF,
            ..Default::default()
        });
        let packet = demod
            .detect(&iq, 0)
            .unwrap_or_else(|| panic!("{}: no packet", want.file));

        assert_eq!(packet.sync_word, want.sync_word, "{}: sync word", want.file);
        assert!(
            packet.preamble_syms >= 8,
            "{}: {} preamble symbols, expected the standard 16",
            want.file,
            packet.preamble_syms
        );

        let ldro = dsp::lora::ldro_default(SF, BW);
        let frame = lora::decode(&packet.symbols, SF, ldro)
            .unwrap_or_else(|e| panic!("{}: {e:?}", want.file));

        assert_eq!(
            frame.header.length, want.length,
            "{}: payload length",
            want.file
        );
        assert_eq!(
            frame.header.coding_rate, want.coding_rate,
            "{}: coding rate",
            want.file
        );
        assert!(frame.header.has_crc, "{}: CRC flag", want.file);
        assert_eq!(
            frame.crc_ok,
            Some(true),
            "{}: payload CRC, {} bytes {}",
            want.file,
            frame.payload.len(),
            hex(&frame.payload)
        );

        // The transmission is exactly as long as its own header says it
        // should be, which a length read out of noise would not be.
        assert_eq!(
            packet.symbols.len(),
            lora::symbol_count(
                frame.header.length,
                SF,
                frame.header.coding_rate,
                true,
                ldro
            ),
            "{}: symbols on the air against the header's shape",
            want.file
        );

        assert_eq!(
            &frame.payload[..4],
            &[0xff; 4],
            "{}: broadcast destination",
            want.file
        );
        let source = u32::from_le_bytes(frame.payload[4..8].try_into().unwrap());
        assert_eq!(source, want.source, "{}: source node", want.file);

        eprintln!(
            "{}: sync 0x{:02x} cfo {:+.0} Hz {} symbols, bin offset {}, {} byte payload from {:08x}, CRC ok",
            want.file,
            packet.sync_word,
            packet.cfo_bins * BW as f32 / (1 << SF) as f32,
            packet.symbols.len(),
            frame.bin_offset,
            frame.payload.len(),
            source,
        );
    }
    if seen == 0 {
        eprintln!("no LoRa fixtures present, skipping");
    }
}

fn hex(b: &[u8]) -> String {
    b.iter().map(|x| format!("{x:02x}")).collect()
}
