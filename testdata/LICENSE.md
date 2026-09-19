# Licence of the test recordings

The program is GPL-3.0-or-later (see [`LICENSE`](../LICENSE)). The recordings
are data rather than program, they are not in the repository, and a licence for
code says nothing useful about a file of samples, so they are licensed
separately here.

## Recordings made for this project

**CC BY 4.0** ([Creative Commons Attribution 4.0
International](https://creativecommons.org/licenses/by/4.0/)). Use them for
anything, commercial or not, redistribute them, cut them up and publish what
you make, as long as you credit where they came from:

> IQ recording by the WaveShark project, https://github.com/v0l/waveshark, CC BY 4.0

Every entry in [`fixtures.toml`](fixtures.toml) and
[`offair.toml`](offair.toml) carries a `license` field, and `CC-BY-4.0` there
means this: recorded, or generated, for this project by its authors.

Two things these recordings contain that the licence does not change. They are
recordings of real bands at real places and times, so identities are in them:
aircraft registrations, a Meshtastic node named "Kieran" and its position, the
hardware addresses of whatever was advertising in the room, the serial of a
radiosonde over Sussex. Each manifest entry says which, because that is what
makes the capture evidence. Nothing about the licence makes the law of the
place you are in stop applying to what is in the recording, and nothing in it
is a permission from the people or the transmitters heard.

## Recordings that came from somewhere else

Five of the fixtures are not ours. Each carries its own `license` and a
`license_source` naming where it came from, and the terms are the publisher's,
not this project's:

| file | from | terms |
|---|---|---|
| `acars_acarsdec_12500.wav` | TLeconte/acarsdec, `test.wav` | LGPL-2.0-only, the licence that repository states |
| `vdl2_model_136.975M_1050k.wav` | szpajder/dumpvdl2, `test/vdl2_model_16b_1050kHz.wav` | GPL-3.0, the licence that repository states |
| `sstv_martin1_44100.wav` | colaclanth/sstv, `examples/m1.ogg`, converted here | GPL-3.0, the licence that repository states |
| `dvbt_hd_429M_9142857.cs8` | Ron Economos, `w6rz.net/adv16.cfile`, cut here | no terms stated by the publisher |
| `rs41_herstmonceux_405.80024M_31.25k.cs16` | SDRangel, `sdrangel.org/iq-files`, cut here | no terms stated by the publisher |

The first two are fetched from their own repositories at a pinned commit and
are never re-hosted. The last three are: a conversion, a cut and a requantised
cut are on nostr.download because the originals are lossy, a gigabyte, or a
WAV that needs converting first. Ask the publisher rather than this project if
you need terms for those two `unstated` files.

Neither the rtl_433 corpus ([`rtl433.toml`](rtl433.toml)) nor the LoRa survey
dataset ([`survey.toml`](survey.toml)) is redistributed here at all: both are
fetched from their publisher. The rtl_433 corpus is contributed recordings
under no stated licence; the survey dataset is CC BY 4.0 from the Universidade
de Vigo, doi 10.5281/zenodo.13835721.
