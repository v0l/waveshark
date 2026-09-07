# mbe

Rust port of the codec module of [jmbe](https://github.com/DSheirer/jmbe)
(Dennis Sheirer, GPL-3.0): decoders for the DVSI Multi-Band Excitation
vocoder family.

- `ImbeSynthesizer`: IMBE 7200x4400, 18-byte / 144-bit frames (P25 Phase 1).
- `AmbeSynthesizer`: AMBE 3600x2450, 9-byte / 72-bit frames (DMR, NXDN,
  P25 Phase 2, D-STAR), including tone frames.

Both produce 160 samples of 8 kHz mono f32 audio per 20 ms frame. Decode
only; there is no encoder.

An AMBE frame arrives three ways: `decode` for the 9 bytes DMR carries,
`decode_bits` for a transport that recovered the 72 bits through its own outer
FEC, and `decode_keyed` for a frame under a 49-bit privacy keystream.

Only DMR speech uses this crate, through
`crates/nodes/src/dmr_nodes/dmr_ambe.rs`. `ImbeSynthesizer` has no caller: it
is here for a P25 front end nobody has written.

## Patent notice

Verbatim from the jmbe and mbelib READMEs this code descends from:

> This source code is provided for educational purposes only. It is a
> written description of how certain voice encoding/decoding algorithms
> could be implemented. Executable objects compiled or derived from this
> package may be covered by one or more patents. Readers are strongly
> advised to check for any patent restrictions or licensing requirements
> before compiling or using this source code.

For that reason this crate is a workspace member but not a default member, and
the one crate that uses it depends on it optionally: `cargo build` and
`cargo test` at the workspace root never build it. `nodes` links it only under
`--features ambe`, which `app` forwards as its own `ambe` feature; without it
the DMR node still reads signalling and shows who is talking, and decodes no
speech. Build the crate on its own with `cargo test -p mbe`.

## Porting notes

The port is faithful to jmbe, including its quirks, which are marked with
comments at the point they are replicated: `BitFrame::get_byte`'s extra
rotate, `get_int_range`'s dead backward branch, the Golay trial-flip loop
that repeats an index on a zero syndrome, and the first-frame NaN from the
zero-seeded noise generator (mapped to silence at the output, which is what
Java's float-to-short cast does downstream). Java float arithmetic stays
f32; `Math` calls are computed in f64 and narrowed exactly where Java
narrows. The codebook tables were extracted mechanically from the Java
sources. The upstream `ambeplus` package is not ported: nothing in jmbe
references it.
