# Changelog

All notable changes to WaveShark are recorded here. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and the project
uses [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

Entries say what changed for somebody running the receiver. What changed in
the code is in the commit log; what a decoder can and cannot do is in
`docs/protocols.md`.

## [Unreleased]

### Added

- A camera's sound. Analogue video links carry audio on a subcarrier of the
  same transmission, 6.5 MHz on the one recorded here, and it is now heard
  alongside the picture. A picture with no sound was half a receiver. It is
  not listed as a call: nobody keyed up to start it and it names nobody.
- The model card shows a download as it happens: which file, how much of it
  and how many files are left, with a bar. It used to say "downloading" and
  nothing else for as long as a multi-gigabyte model takes.
- A dashboard, and the receiver opens on it: what the application can do, as
  cards that take you there, and while a radio is running what it is doing.
  What is tuned, whether the host is keeping up, how much has been decoded,
  what is transmitting right now, and what has been heard so far. Turn it off
  in Settings, App, or from the corner of the view itself, and the receiver
  opens on the spectrum again.

- DJI DroneID: a DJI aircraft's own broadcast is read, so a row names the
  airframe by the serial printed on it, with its position, height, home point
  and the operator's position where the aircraft has a fix. Needs 15.36 MS/s,
  so a HackRF or a LimeSDR and not an RTL-SDR.
- The picture pane has a channel chooser: pick a transmission to watch from
  the dropdown, or leave it on the most complete picture the receiver has.
  It is there whether or not anything is on the air, and says how many
  channels are being received.
- `iq_clipper`, which cuts a recording to the bursts in it and can tune it
  onto one channel first, replacing the `cut` example.
- `--bench-iq FILE`, which replays a capture through the receiver a block at
  a time and reports where the throughput drops: the slowest blocks, how
  regularly they come, and which stage the time went into. For finding what
  causes a lag spike without a radio and with the same answer every run.

- The WaveShark mark is the window's icon, so the dock, the task bar and the
  window switcher show it instead of a blank default. On Windows it is in the
  executable as well, and the release archives carry the icon file for a
  desktop entry to point at.
- The transcript view's model card offers a list of models to pick from,
  every size of Whisper plus Qwen3-ASR, and a list of devices to run on,
  the CPU and each GPU by name. A model picked is fetched on first use into
  its own directory, so switching back costs nothing.

- Qwen3-ASR (0.6B and 1.7B) as a transcriber beside Whisper. It reads
  noisy and accented speech better and says what language it heard.

- A Transcript view: what the local speech model read off
  everything the receiver heard, newest at the bottom, with a line still
  being spoken marked as it grows and a low-confidence reading flagged. It
  can be filtered by words or opened on one conversation, and the calls list
  has a "read" button on each row the model heard speech on that opens it
  there.

- The transcript view says what the speech-to-text is doing: which model, the
  directory its files are in, how much is on disc and whether they are the
  English-only or multilingual weights, whether it is loading, ready or
  failed and why, whether it is running on CPU, CUDA or Metal, and how much
  faster than real time it read the last window. A button loads or downloads
  the model there and then instead of waiting for the first transmission.

- The views are a strip of tabs in the top bar instead of a dropdown: one
  click to switch, the keyboard shortcuts Ctrl+1 to Ctrl+0,
  Ctrl+` to go back to the last view, and a dot on any tab whose view has
  taken something in since you last looked at it.

- An OPEN button beside the raw capture folder, with the path next to it, so a
  recording can be replayed or trimmed without typing the path out.

- Drone Remote ID over Wi-Fi: an aircraft that broadcasts its identity in a
  beacon or a NAN discovery frame is read on the 802.11 receiver, and its
  serial, position and operator are the front of the row instead of a
  network's name.

- A Satellites view: every pass over the station in the next day, ordered by
  rise time, with the peak elevation, which way to point, live az/el and
  Doppler for whatever is up now, and how stale the elements it was worked
  out from are.
- Listening to a pass: the listen button on a transmitter's row puts a
  channel on that downlink, keeps it there as the pass moves, and closes it
  when the satellite sets, so it does not
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
- A selected pass lists every transmitter the satellite is still using as a
  table of downlink, where it is arriving now, the Doppler on it, mode,
  baud, uplink and what the channel is, beside the sky plot for the pass.
  Listening is a button on the row, so it is asked for on the downlink it is
  about. The ISS has forty-one live transmitters and only one was ever
  quoted.
- Listening to a satellite builds the chain its mode calls for: LoRa gets the
  LoRa front end at the bandwidth SatNOGS quotes, packet AFSK gets the AX.25
  one, DMR gets DMR, and voice and Morse get the demodulator. Only a mode
  with no decoder here falls back to the auto front end.
- A pass card says what the satellite is for in the words an operator uses,
  such as VHF voice, SSTV, APRS and DVB, instead of leaving forty rows of
  free text to be read.
- Every catalogued object still in orbit, from Space-Track, for a satellite
  no group has classified yet. It needs a free account of your own, entered
  on the row in Datasets.
- The satellites the TinyGS network tracks, as a group of their own: the
  LoRa and FSK cubesats around 400 and 900 MHz, with the elements TinyGS
  points its own stations by.
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
- Wi-Fi: 802.11a/g and single-stream 802.11n frames off a 20 MHz channel,
  with the network name, the addresses talking, the rate each frame
  arrived at and whether an address is randomised. Aggregated 802.11n
  transmissions are listed as the frames inside them. A span wider than
  one channel is read a channel at a time, so a LimeSDR at 61.44 MS/s
  watches the whole 2.4 GHz band at once. 802.11b at 1 and 2 Mbit/s as
  well, which is what every access point sends its beacon at, so networks
  are listed by name with their access point and security. Needs a HackRF
  or a LimeSDR, since one channel is 20 MHz wide. The channels are read in
  parallel, a channel with nothing in it is skipped, and a channel is
  opened when a beacon says its network is there, so a LimeSDR watching
  the whole 2.4 GHz band keeps up with the radio.
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

- Nothing is written down until it is asked for. The packet log, the device
  survey and the transcriber are all off on a new install and on an upgrade,
  and each is remembered from then on, so it is a decision made once rather
  than a switch to find at every start. `--packet-log` and `--survey` turn
  them on from the command line and `--no-packet-log` and `--no-survey` turn
  them off whatever was saved.
- On Windows the receiver no longer opens a console window behind itself. Run
  from a terminal it still prints, so `--help` and the offline tools work as
  before.
- The transcript card no longer prints what the last window came back as
  verbatim.
- The transcript is a table: time, frequency, speaker, group or channel,
  and the words, which wrap.

- CUDA is on by default in a build from source, and the release publishes
  a `-cuda` build for Linux and Windows beside the plain one. Building
  needs the CUDA toolkit; `--no-default-features --features limesdr,stt`
  builds without it, and the `-cuda` binary needs the CUDA 12 runtime.

- The model card names the model rather than printing the path to its
  files; the path is on hover over the "on disc" line.

- The toolbar and view tabs use the Phosphor icon set, so the icons share one
  weight and one grid at every size instead of drifting between the top bar
  and the channel strip.
- The keying column spells every modulation one way, so a pager and a meter
  keyed the same both read `2-FSK` where one used to say `FSK` and the other
  `2-FSK`.
- The frequency correction is saved against the radio it was measured on
  rather than shared by all of them, so plugging in a second receiver no
  longer applies the first one's crystal error to it. A correction saved by
  an earlier version moves to the radio that was in use when it was written.

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

- With the device on Auto, a card that has no room for the model, or that
  opens and then cannot run it, hands the model to the CPU and the card
  says why. It used to be a failed model.

- The model and device picked on the transcript card, and any setting
  changed in the chain inspector, are remembered across a restart. They
  reached the receiver but were never written to the edits file.

- The transcript view and the call list fill in as speech is read. Both
  were only read under the soak option, so a model that read every word
  showed "0 lines" and the call list stayed empty.

- A channel marked as voice is heard through its own fader like any other
  channel. It used to be wrapped in an empty packet so the call list would
  see it, which put a "voice" row saying nothing into the packet list and the
  log for every transmission, and left the channel silent until something
  subscribed to it. Analogue speech is audio, not a packet, and no longer
  touches the packet log at all. It is not listed as a call either: a mode
  and a frequency do not say whether what is coming out is a conversation.
  It is still transcribed.

- The speaker is fed a block of silence rather than nothing at all when
  nothing on the bus is playing, which used to starve the sound card.

- Speech is transcribed as it is spoken instead of piling up: nothing is
  held longer than the thirty seconds the model reads in one pass, and a
  transmission that runs past that is cut at a pause and written down as far
  as it got. A repeater left keyed used to collect a minute and a half of
  audio and show nothing.

- A reading the model was unsure of is shown, marked "unsure", rather than
  thrown away. A weak or fading handheld produced an empty transcript before,
  which looked like a receiver that was not listening.

- A channel carrying noise rather than speech, an open squelch or a hiss, is
  left alone after two windows the model finds no speech in, instead of being
  read over and over for as long as it hisses.

- Weak analogue video decodes. The picture was read off the whole sampled
  span, so a 4.6 MHz camera arrived with 20 MHz of noise on it; it is band
  limited first, which turned a 5.8 GHz link that produced nothing into one
  that produces a picture, and halved what the front end costs.
- Only one video front end is placed on a span. The 5.8 GHz plan names 5865
  and 5866 as different channels, so a receiver on either read the span
  twice and published every field twice.
- Mode S on a wide span reads the 2.4 MS/s it asks for instead of everything
  the radio is sampling. On a 20 MS/s span it was 127% of a processor core
  looking at an empty band, and is now 37%.
- Wi-Fi samples the air, a fifth of a second in every second, until it hears
  something. Every network beacons ten times a second so they are all still
  found within a second, and a band with no Wi-Fi on it no longer costs more
  than everything else the receiver is doing put together.
- A scanner block added in the interface now demodulates the channel typed
  into it. It kept the protocol's own default frequency instead, so a camera
  asked for on 5800 MHz was read on 5865 and nothing appeared.
- While a camera owns the span, the Wi-Fi and other span-wide decoders stop
  reading it. An OFDM search through an FM picture was eighteen times real
  time, which is what kept the picture breaking up.
- A picture that goes away gives the band back, so a receiver that locked onto
  a moment of noise recovers after a few seconds instead of staying deaf for
  the session.
- A front end that reads the whole span is left out, with a reason, when the
  span is too narrow for it. It used to be built anyway and take the whole
  receiver down with it.

- Wi-Fi on a busy 2.4 GHz band costs a sixth of what it did. A hopping
  transmitter held the preamble detector open and cost a training-symbol
  search every 64 samples for as long as it lasted, which on one capture was
  132,000 searches a second and read nothing.
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
