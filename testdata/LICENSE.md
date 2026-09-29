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

Every entry in [`decode.toml`](decode.toml) and
[`fixture.toml`](fixture.toml) carries a `license` field, and `CC-BY-4.0` there
with no `license_source` means this: recorded, or generated, for this project
by its authors. The captures in [`local.toml`](local.toml) are not published
and carry no licence.

Two things these recordings contain that the licence does not change. They are
recordings of real bands at real places and times, so identities are in them:
aircraft registrations, a Meshtastic node's address, the hardware addresses
of the Wi-Fi devices in the room, the serial of a radiosonde over Sussex. Each manifest entry says which, because that is what
makes the capture evidence. Nothing about the licence makes the law of the
place you are in stop applying to what is in the recording, and nothing in it
is a permission from the people or the transmitters heard.

## Recordings that came from somewhere else

Twenty-eight of the fixtures are not ours. Each carries its own `license` and a
`license_source` naming where it came from, and the terms are the publisher's,
not this project's:

| file | from | terms |
|---|---|---|
| `acars_acarsdec_12500.wav` | TLeconte/acarsdec, `test.wav` | LGPL-2.0-only, the licence that repository states |
| `vdl2_model_136.975M_1050k.wav` | szpajder/dumpvdl2, `test/vdl2_model_16b_1050kHz.wav` | GPL-3.0, the licence that repository states |
| `ft8_wsjtx_210703_133430_12000.wav` | WSJTX/wsjtx, `samples/FT8/210703_133430.wav` | GPL-3.0, the licence that repository states |
| `sstv_martin1_44100.wav` | colaclanth/sstv, `examples/m1.ogg`, converted here | GPL-3.0, the licence that repository states |
| `dvbt_hd_429M_9142857.cs8` | Ron Economos, `w6rz.net/adv16.cfile`, cut here | no terms stated by the publisher |
| `rs41_herstmonceux_405.80024M_31.25k.cs16` | SDRangel, `sdrangel.org/iq-files`, cut here | no terms stated by the publisher |
| `adsb_london_1090M_2400k.cu8` | SDRangel, `sdrangel.org/iq-files` adsb.zip, cut and converted here | no terms stated by the publisher |
| `ais_london_162M_250k.cs16` | SDRangel, `sdrangel.org/iq-files` ais.zip, cut and converted here | no terms stated by the publisher |
| `airband_london_119.5M_3000k.cs16` | SDRangel, `sdrangel.org/iq-files` airband.zip, cut and converted here | no terms stated by the publisher |
| `apt_noaa18_137.912M_62.5k.cs16` | SDRangel, `sdrangel.org/iq-files` apt.zip, cut and converted here | no terms stated by the publisher |
| `ax25_no84_145.825M_24k.cs16` | SDRangel, `sdrangel.org/iq-files` no84.zip, converted here | no terms stated by the publisher |
| `clocks_msf_dcf77_tdf_0.11M_192k.cs16` | SDRangel, `sdrangel.org/iq-files` clock.zip, cut and converted here | no terms stated by the publisher |
| `dab_bbc_12b_225.648M_2048k.cs16` | SDRangel, `sdrangel.org/iq-files` dab.zip, cut and converted here | no terms stated by the publisher |
| `graves_iss_143.05M_6k.cs16` | SDRangel, `sdrangel.org/iq-files` radar.zip, converted here | no terms stated by the publisher |
| `navtex_niton_0.518M_1.953k.cs16` | SDRangel, `sdrangel.org/iq-files` navtex.zip, converted here | no terms stated by the publisher |
| `rtty_dwd_11.039M_2k.cs16` | SDRangel, `sdrangel.org/iq-files` rtty.zip, converted here | no terms stated by the publisher |
| `sstv_pd120_iss_145.79544M_250k.cs16` | SDRangel, `sdrangel.org/iq-files` sstv.zip, converted here | no terms stated by the publisher |
| `vor_ockham_biggin_115.2M_384k.cs16` | SDRangel, `sdrangel.org/iq-files` vor.zip, cut and converted here | no terms stated by the publisher |
| `wfm_london_99M_4000k.cs16` | SDRangel, `sdrangel.org/iq-files` bfm.zip, cut and converted here | no terms stated by the publisher |
| `ysf_145.6875M_74.999k.cs16` | SDRangel, `sdrangel.org/iq-files` dsd.zip, converted here | no terms stated by the publisher |
| `dab_melbourne_9a_202.928M_2500k.cs16` | Signal Identification Wiki, `DAB+9A.zip` by Griffonboi, cut here | no terms stated by the publisher |
| `nxdn48_453M_48k.cs16` | Signal Identification Wiki, `NXDN_IQ.zip` by Cartoonman, converted here | no terms stated by the publisher |
| `nxdn96_453M_48k.cs16` | Signal Identification Wiki, `NXDN_IQ.zip` by Cartoonman, converted here | no terms stated by the publisher |
| `eas_tor_kilx_22050.wav` | Signal Identification Wiki, `EAS_Alert_Tornado_Warning.mp3` by Cartoonman, converted here | no terms stated by the publisher |
| `stdc_egc_1541.45M_48k.cs16` | Signal Identification Wiki, `Inmarsat-C_TDM_EGC_IQ.zip` by Cartoonman, converted here | no terms stated by the publisher |
| `drm_b_3.965M_48k.cs16` | Signal Identification Wiki, `DRM_B.zip` by Voxo, converted and tuned here | no terms stated by the publisher |
| `aero_oqpsk_1546M_48k.cs16` | Signal Identification Wiki, `Inmarst_Aero_10500_Bd_OQPSK_IQ.zip` by Cartoonman, converted here | no terms stated by the publisher |
| `lte_b20_madrid_806M_30720k.cs8` | Daniel Estévez, `nas.destevez.net/~daniel/LTE/`, requantised here | CC BY 4.0, credit Daniel Estévez |

The first three are fetched from their own repositories at a pinned commit and
are never re-hosted. The last twenty-five are on nostr.download, converted or cut,
because the originals are lossy, a gigabyte, or a WAV that needs converting
first. Ask the publisher rather than this project if you need terms for the
twenty-three `unstated` files.

Neither the rtl_433 corpus ([`rtl433.toml`](rtl433.toml)) nor the LoRa survey
dataset (`survey/lora_salvora.csv` in [`fixture.toml`](fixture.toml)) is
redistributed here at all: both are
fetched from their publisher. The rtl_433 corpus is contributed recordings
under no stated licence; the survey dataset is CC BY 4.0 from the Universidade
de Vigo, doi 10.5281/zenodo.13835721.
