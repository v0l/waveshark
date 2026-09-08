# Changelog

All notable changes to WaveShark are recorded here. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and the project
uses [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

Entries say what changed for somebody running the receiver. What changed in
the code is in the commit log; what a decoder can and cannot do is in
`docs/protocols.md`.

## [Unreleased]

### Added

- A Satellites view: every pass over the station in the next day, ordered by
  rise time, with the peak elevation, which way to point, live az/el and
  Doppler for whatever is up now, and how stale the elements it was worked
  out from are.
- Listening to a pass: one button on the card puts a channel on the
  satellite's downlink, keeps it there as the pass moves, and closes it when
  the satellite sets, so it does not
  drift out of the channel before the satellite is overhead. The strip names
  it after the satellite and the transmitter, says it is following, and its
  dial is locked while it is; the chain is the demodulator for the mode, or
  the auto front end where this receiver has none.
- A sky plot on the selected pass: the track across the sky with north at
  the top, where it rises and where it sets, and where it is now.
- Free-space loss, one-way delay and footprint radius on a pass in progress.
- A SATS layer on the map, drawing the ground track of whatever is above the
  horizon, brighter ahead of the satellite than behind, with the selected
  one drawn whether or not it is up and its footprint circle with it.
  Clicking a satellite on the map selects it, and clicking it again lets it
  go.
- Orbital elements from CelesTrak, as datasets of their own: amateur,
  weather, CubeSats, space stations and GNSS, each a row that refreshes and
  fails on its own.
- What each satellite transmits on, from the SatNOGS database, so a pass
  quotes the satellite's own downlink and tunes to it.
- A cell you have decoded that the OpenCelliD export has no row for can be
  placed from beaconDB, drawn as a cross with the accuracy it came with.
  Off until switched on: asking says which cells this receiver has heard.
- Feed beacondb.net: Bluetooth devices and cells heard with a position are
  spooled and submitted, from the button beside WiGLE in the devices pane.
  No account, off until it is switched on, and a drive with no coverage
  sends when it gets home.
- Which mobile network an MCC and MNC belong to, and cell positions from
  OpenCelliD for the country set in Setup, which needs a download token of
  your own entered in the datasets window.
- A CELLS layer on the map, drawing each cell in the export at the position
  the crowd averaged for it, with the radius that position is good to and
  the network it belongs to on hover.
- Bluetooth LE advertising, including Bluetooth 5 Long Range, and Open
  Drone ID out of it or off a beacon.
- GSM: a cell's identity off its synchronisation burst, the blocks it
  broadcasts, who a page is calling, and the signalling channel a phone is
  sent to, on the beacon carrier or on another carrier timed from it.
- ExpressLRS at 2.4 GHz as a front end the auto node places and a strip
  mode: the SX1280's long interleaved coding measured off the air, the
  link learned from a sync packet or from the packets' own CRC seeds, and
  the sticks read. FrSky ACCST, FlySky AFHDS-2A and XN297 remotes as
  decoders.
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
- Wardriving: Bluetooth devices and GSM cells heard with a position can be
  uploaded to wigle.net while you drive. Rows are spooled to disc and sent
  when there is a network, so a drive with no coverage uploads when it gets
  home. The API name and token go in the WiGLE dialog on the devices pane,
  which also shows what is waiting, what has been sent and why an upload
  failed.

### Changed

- A second transmitter sharing a channel with one already being read is
  found and given a decoder of its own. Two LoRa networks on one frequency
  at different bandwidths, such as Meshtastic on 250 kHz and MeshCore on
  125 kHz, both decode; before, whichever was heard first kept the channel
  and the other was silent for the rest of the session.
- The map credits what it is drawing, in one plate in the corner: the tiles
  and every layer's data source, each name a link to the publisher, so
  OpenCelliD's masts are credited while they are on screen.
- The cached datasets have a window of their own, opened from the icon
  beside Setup, and each digital voice network is a row of its own that
  refreshes and fails on its own.
- The WiGLE CSV export writes a Bluetooth row and a cell row the way the
  format defines them, and leaves out what the format has no type for:
  aircraft, vessels, pagers and sensors were being exported as Bluetooth
  devices.
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

- The Windows build works again, and a GPS on a COM port is opened there the
  way it is on Linux: the serial settings are applied to the port instead of
  being left at whatever the driver came up with.
- "Folder holds" in the packet log settings reports what is on the disk. It
  showed 0 B until the log had written its first burst, and stayed at 0 B on
  a span where nothing produces packets, beside gigabytes of captures.
- A dataset a publisher refused is not asked for again until somebody presses
  refresh. CelesTrak asks for this in writing and firewalls clients that keep
  retrying.
- The divider between the map and the tracks table can be dragged, and
  double-clicked to put it back. Switching the TRACKS layer off hides the
  table too and gives the map the whole pane.
- The map's status line says why the cells layer is drawing nothing: zoomed
  out past where it draws, or no export downloaded.
- A refused cell tower download no longer throws away the export already
  held. OpenCelliD answers a used-up daily allowance with a success and a
  short complaint, which was being stored as the dataset.
- The cell tower download no longer asks for a country that is already set.
  It was waiting on the mobile networks list, which it now fetches itself,
  and `--fetch-data` reads the country and token out of the saved settings
  instead of skipping the export.
- A burst of several packets with silence between them, which is what a
  hopping link leaves on one channel, is classified from one of the
  packets rather than as a keyed envelope; sources open as wide as the
  widest channel any protocol reads rather than 600 kHz.

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
