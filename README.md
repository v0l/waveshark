<img src="assets/logo/waveshark-horizontal.svg" alt="WaveShark" width="420">

Wireshark for the radio spectrum. An OSINT tool for RF: leave a cheap SDR on a
band and it tells you what is transmitting around you, instead of what is on
the one frequency somebody already told you about.

Which sensors are in this building, which vehicles keep passing, which remotes
and door contacts are in use on this street, who is paging whom, what is
overhead. That comes from covering a band continuously and keeping everything,
decoded or not.

![Decoding a weather station on 433.92 MHz](assets/screenshot.png)

## What it identifies

| | where | what you get |
|---|---|---|
| ISM devices | 433, 868, 915 MHz | 45 decoders, most from rtl_433's family: weather stations, thermometers, TPMS, door contacts, gate remotes, shelf labels, mostly with a stable device ID |
| Unknown bursts | anywhere | coding inferred and bits sliced out, enough to recognise the same device again and reverse engineer it |
| Aircraft | 1090 MHz | ADS-B and Mode S onto a map with a track table: callsign, altitude, speed, track, position |
| Aircraft datalinks | 131 and 136 MHz | ACARS and VDL Mode 2, the messages crews and airlines send each other |
| Shipping | marine VHF | AIS positions and vessel identity on the same map |
| Radiosondes | 400-406 MHz | Vaisala RS41 weather balloons on the map: serial, height, climb rate, and the air it was sent up to measure |
| APRS | 144.800, 144.390 US, 144.640 JP | packet stations and vehicle trackers, Mic-E included |
| Pagers | wherever you point it | POCSAG at 512, 1200 and 2400 bit/s, message text in clear |
| DMR | 136-174, 400-470 MHz | who called whom on which talkgroup, and speech through the `ambe` feature |
| TETRA | 390-400 MHz | the network, its cells and who is called, with decryption and key recovery under the `tea` feature |
| M17 | amateur VHF and UHF | who called whom, for how long, packet messages in full, and Codec 2 speech |
| LoRa mesh | 433, 868, 915 MHz | LoRaWAN join requests and addresses, Meshtastic text under the public keys, MeshCore adverts |
| Utility meters | 868.95 MHz | wireless M-Bus mode T: manufacturer, meter number, version and type |
| Voice | any band | WFM with stereo and RDS, NFM, AM, USB, LSB, CW, several channels at once |
| Pictures | any band | SSTV in Martin, Scottie and Robot modes, and television off a DVB-T multiplex |

It transmits too, on a radio that can: a microphone or a tone into NFM, WFM or
AM, drawn as the TX side of the same flow graph. A channel's transmit source
can also be an agent, which answers when it hears its name.

Turn on Record and every over is kept as Opus, playable back from the call
list. Messages are written down as they arrive, a file a day, and a local
speech model transcribes what is said.

## Hardware

Any RTL2832U dongle, a HackRF One, or a LimeSDR USB or Mini, and a tuner on
another machine over IQStream, rtl_tcp, SpyServer or KiwiSDR with `--stream
rtl_tcp://<host>`. Public IQStream and SpyServer tuners are listed in the add
dialog's FIND. A €30 RTL-SDR does all of
the receiving above; a HackRF buys you wider spans and a transmitter, and a
LimeSDR both of those plus full duplex.

## Install

Grab a build from [releases](https://github.com/v0l/waveshark/releases): a
`.deb` or `.rpm` for Linux x86_64 and arm64, an `.msi` for Windows x86_64, a
`.dmg` for macOS on Apple silicon. The arm64 packages run on a Raspberry Pi 4
or 5 under a 64-bit system from bookworm onwards. Setup checks for a newer
release and can fetch and open the one for the machine it is running on. The
bare binary is published beside each installer for anyone who would rather not
install anything; the Windows one is a zip holding the `.exe`. `wave1090` is published
for every platform as a binary of its own. Every asset carries the version in
its name.

One build per platform, card or no card. The speech models run on an NVIDIA
GPU when the CUDA 12 runtime is on the machine and on the CPU when it is not,
because the CUDA libraries are loaded when they are first wanted rather than
named in the binary. macOS uses Metal. The TETRA key search runs on any GPU
through wgpu, AMD and Intel included.

The RTL-SDR is driven by the receiver's own USB driver, so no librtlsdr is
installed or loaded. The dongle still needs the udev rules to be openable
without root: install `rtl-sdr` or `librtlsdr0` yourself to get them, or
write the rule by hand. On Windows bind WinUSB to the RTL2832U with
[Zadig](https://zadig.akeo.ie/) first or nothing can open the device. The
Windows build has no LimeSDR: LimeSuite is not packaged for it, so that binary
is built without the driver. Elsewhere LimeSuite is opened when a LimeSDR is
looked for rather than linked, so the binary starts without it and reads a
LimeSDR once `liblimesuite` or `LimeSuite` is installed, whatever version the
distribution carries. Only the macOS build shows pictures off a
multiplex, because it is the only one whose ffmpeg is new enough.

The macOS app is signed ad-hoc and not notarised, so the first open needs
Privacy & Security in System Settings, where macOS offers Open Anyway after
the app has been refused once (on Sonoma and older, a right click and Open
does it), or:

```sh
xattr -dr com.apple.quarantine /Applications/WaveShark.app
```

The app carries its own copies of ffmpeg and LimeSuite. The bare
macOS binary does not: it reads them from Homebrew, so `brew install ffmpeg
limesuite` before running that one.

From source:

```sh
sudo apt install liblimesuite-dev pkg-config libclang-dev \
  libasound2-dev libx11-dev libxrandr-dev libxi-dev libxcursor-dev \
  libxkbcommon-dev libwayland-dev libgl1-mesa-dev
cargo run --release -p app
```

That build has everything, including two decoders the published binaries do
not carry:

```sh
cargo run --release -p app --no-default-features --features limesdr,stt,cuda,mcp
```

is what the release workflow runs, and it leaves out `tea` and `ambe`. `tea`
links the TETRA ciphers and a wgpu key search; without it the keys view is
still there, listing enciphered channels and saying nothing can read them.
`ambe` builds `crates/mbe`, a port of the AMBE and IMBE vocoders, whose
algorithms are patent encumbered; without it DMR still says who is talking and
decodes no speech. Compiling them for yourself is not the same act as a
project distributing them, which is why the source turns them on and the
downloads do not.

## Using it

Plug in a radio and press play. It opens on 433.92 MHz, where the devices it
decodes are.

Decoding does not follow the dial: every scanner inside the sampled span runs
all the time, so a sensor that transmits once a minute is caught whether or not
you were pointing at it. The span is what you collect; the dial is where you
look. Click the spectrum to place a channel and listen, drag to pan, scroll to
scrub, shift to snap to the band plan.

The list along the bottom is every burst heard, with frequency, modulation,
RSSI, SNR and what was made of it. Click a row for its envelope, its
instantaneous frequency and a hex dump. SETTINGS beside the list is the packet
log, including the switch that hides unclaimed bursts; SCANNERS is the table
deciding what decodes where.

The view selector swaps the spectrum for the signal chain, the map and its
track table, the call list, the messages, or the keys. In the header, the
sliders icon is the radio itself (gain, switches, antenna, channel, crystal
correction) and the setup icon is language, country, band plan, your position
and the cached datasets.

The signal chain is not a diagram of the receiver, it is the receiver. Unlock
it and you can add stages, delete them, drag them and draw wires; what is kept
is the difference you made, so the graph goes on following the dial and the
scanner table with your edits still on it.

## The packet log

Every burst goes to `$XDG_DATA_HOME/waveshark/packets`, one file a day, on by
default, because the interesting transmission is always the one that happened
before you thought to record. What is stored is the mark and gap timings, the
frame bytes and the burst's own samples rather than the parsed fields, so a
better decoder can be run over it later, and a different demodulator can be run
over it as well:

```sh
waveshark --replay 2025-08-31.wspkt
```

```
15:55:47   433.9200 MHz    88 pulses   22.5 dB  Fineoffset-WHx080  temperature_c=18 humidity_pct=61
15:55:47   433.9200 MHz   305 pulses   16.4 dB  unclaimed
```

`--packet-log <dir>` moves it and `--no-packet-log` turns it off. The folder is
held to 2 GB by deleting the oldest days, so it rolls rather than stopping.

Collecting has consequences: pager traffic carries medical and personal detail
in clear, and device IDs are a record of who was where. Interception and
retention rules differ by country and that is on you.

## Recording IQ

`--record <dir>` writes each burst as an rtl_433 style capture, so both this
and rtl_433 can read it back:

```sh
waveshark --tune 868.3 --record captures
waveshark --replay captures
```

Capture a band once, then replay after every change: no radio, same answer
every time. A capture that decodes is a test fixture.

A recording made by another program is named for what it holds rather than in
the rtl_433 convention, so `--capture`, `--replay` and `--bench-iq` take the
rate, the sample format and the centre frequency beside the path:

```sh
waveshark --capture "session.iq,rate=2.4M,format=cs16,centre=433.92M"
```

## Letting an agent drive

Every run serves the receiver over the Model Context Protocol at
`http://127.0.0.1:8931/mcp`. It is the receiver on the screen, not a second
one: what the agent tunes, opens or switches on appears in the window, and it
can take a picture of that window to see what it did. It can read the
spectrum, the packets, the calls, the transcript and the tracker, change
anything in the signal chain, and draw the chain itself: add stages, wire
them and take them out again. It reaches the settings too, so it can add a
scanner, store a memory, pick a voice or turn on a feed. On a radio that can
transmit it will key up and speak.

`--mcp-listen off` stops it listening, and a port or a `host:port` puts it
somewhere else. The default reaches no further than this machine.

## Command line

`--help` has the rest.

```
--tune <mhz>           start tuned and listening; repeat for several channels
--mode <mode>          wfm, nfm, am, usb, lsb or cw
--span <khz>           nearest span, narrowed in software if the radio cannot
--device <name>        pick a radio when several are plugged in
--stream <host>        offer a network tuner as a radio, iqstream or
                       rtl_tcp://, spyserver:// or kiwisdr://<host>
--location <lat,lon>   your position, for aircraft positions from a single frame
--record [dir]         write every burst that decodes to a directory of captures
--capture-iq           write the raw span from the moment the radio starts
--capture <file>       open a recording as the receiver, with rate=, format=
                       and centre= where its name does not say
--replay [path]        decode a capture, a directory, or a packet log
--headless             run with no window, scanning and logging as it would
--ha-broker <broker>   publish every device heard to Home Assistant over MQTT
--ha-spaces <kinds>    which kinds are worth publishing, e.g. ism,wmbus
--mcp-listen <addr>    where to serve MCP, or `off`; 8931 on loopback by default
--print-log            print every packet as it arrives, window or not
--fetch-data           warm the dataset cache before going somewhere offline
--squelch-probe [mhz]  report what the squelch reads on a frequency
--probe [mhz]          check the signal path with no display
```

`--chain`, `--flights`, `--calls`, `--messages`, `--transcript`, `--links`,
`--control`, `--video`, `--scanners`, `--gain` and `--setup` open on a view,
and `--settings <name>` on a settings dialog.

## Status

Verified against other people's decoders, not just its own: 81 recordings from
rtl_433's corpus are replayed field for field against what rtl_433 25.02 made
of them, plus ADS-B against dump1090, ACARS against acarsdec, VDL Mode 2
against dumpvdl2, SSTV against colaclanth's decoder, and a radiosonde against
SDRangel. Off-air captures of M17, DMR, TETRA and Meshtastic are asserted
against what the transmission itself says. Those tests need
`testdata/fetch.sh` to have pulled the recordings, and skip cleanly when it
has not, which is also what happens in CI. Coverage is the thin part,
forty-five ISM decoders where the goal is hundreds, and the browser build is
still a plan.

## Where to read next

The code. `crates/nodes/src/protocol.rs` is the registry every decoder is
reached through, `crates/app/src/chain.rs` draws the graph the receiver runs,
and each `crates/decode/src/protocols/*.rs` carries the frame layout it
decodes. [`docs/references.md`](docs/references.md) is the one page kept
outside the code, because the terms a publisher asks for are an obligation
rather than an explanation.

## Licence

GPL-3.0-or-later, text in [`LICENSE`](LICENSE).

The test recordings are data and are licensed apart from the program:
[`testdata/LICENSE.md`](testdata/LICENSE.md) holds the terms, CC BY 4.0 for
the ones recorded here and the publisher's own for the five that came from
somewhere else.
