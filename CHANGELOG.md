# Changelog

All notable changes to WaveShark are recorded here. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and the project
uses [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

Entries say what changed for somebody running the receiver. What changed in
the code is in the commit log; what a decoder can and cannot do is in
`docs/protocols.md`.

## [Unreleased]

### Added

- Bluetooth LE advertising, including Bluetooth 5 Long Range, and Open
  Drone ID out of it or off a beacon.
- GSM: a cell's identity off its synchronisation burst, the blocks it
  broadcasts, who a page is calling, and the signalling channel a phone is
  sent to, on the beacon carrier or on another carrier timed from it.
- ExpressLRS at 2.4 GHz, with the SX1280's long interleaved coding
  measured off the air and a payload read through it, FrSky ACCST, FlySky
  AFHDS-2A and XN297 remotes.
- LoRa at 2.4 GHz as the SX128x sends it, and inverted LoRa as a setting.
- Analogue video: a camera's picture off the span, PAL or NTSC, with
  colour, on a video bus and pane of its own.
- Transcription of decoded speech with a local Whisper model, off the
  audio bus, for every call and for any strip channel marked as voice.
- A survey: every transmitter that identifies itself, recorded once with
  the places it was heard from, exported as WiGLE CSV, and an estimate of
  where each one probably is from the levels along a drive, with how far
  out that might be.
- A links directory with a follow view, and messages as the decoder names
  them.
- The GPS drives the station position, with the satellite count and
  dilution from gpsd.
- Every protocol is a mode a strip channel can be set to, span-wide ones
  included, and a frequency in any scanner block.
- A newer release is reported in setup and shown in the header.

### Changed

- The auto front end asks one protocol registry where each decoder belongs,
  what it reads, how sticky its channel is and how to build it; the
  scanner table, the strip and the spectrum markers read the same
  registry.
- A decoder can ask the receiver for a band, a channel beside its own, a
  reshape of its stream, to be dropped, or a retune; a TETRA control
  channel asks for the traffic carrier a call is sent to, a camera claims
  the span once it has a picture.
- A decoder placed late on a source catches up on the samples it missed
  over several blocks rather than in one, which stalled the radio thread
  on a LoRa chirp and dropped samples.
- wM-Bus is placed by its band rather than on every source of a meter's
  width.
- Every packet carries its level, noise and the samples it was read from,
  bounded and compressed in the log.
- The header is laid out as fixed cells with the setup control pinned
  right.
- TETRA decryption (`tea`) and the AMBE vocoder (`ambe`) build from source
  and are not in a published binary.

### Fixed

- The dial no longer caps every radio at 3 GHz.
- A loud keyed burst is not reopened around its own splatter, and no
  source opens on the spikes of a blanketed band.
- A remembered channel is read by the front end that earned it alone.

## [0.1.0] - 2026-09-06

First tagged release: the auto front end over the ISM bands, Mode S, AIS,
APRS, POCSAG, M17, DMR, TETRA, LoRa, wM-Bus and the rtl_433 sensor
tables, with the packet log, the map, the chain view and transmit.

[Unreleased]: https://github.com/v0l/waveshark/compare/v0.1.0...HEAD
[0.1.0]: https://github.com/v0l/waveshark/releases/tag/v0.1.0
