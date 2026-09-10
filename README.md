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
| ISM devices | 433, 868, 915 MHz | 39 decoders, most from rtl_433's family: weather stations, thermometers, TPMS, door contacts, gate remotes, shelf labels, mostly with a stable device ID |
| Unknown bursts | anywhere | coding inferred and bits sliced out, enough to recognise the same device again and reverse engineer it |
| Aircraft | 1090 MHz | ADS-B and Mode S onto a map with a track table: callsign, altitude, speed, track, position |
| Shipping | marine VHF | AIS positions and vessel identity on the same map |
| APRS | 144.800, 144.390 US, 144.640 JP | packet stations and vehicle trackers, Mic-E included |
| Pagers | wherever you point it | POCSAG at 512, 1200 and 2400 bit/s, message text in clear |
| DMR | 136-174, 400-470 MHz | who called whom on which talkgroup, and speech through the `ambe` feature |
| TETRA | 390-400 MHz | the network, its cells and who is called, with decryption and key recovery under the `tea` feature |
| M17 | amateur VHF and UHF | who called whom, for how long, packet messages in full, and Codec 2 speech |
| LoRa mesh | 433, 868, 915 MHz | LoRaWAN join requests and addresses, Meshtastic text under the public keys, MeshCore adverts |
| Utility meters | 868.95 MHz | wireless M-Bus mode T: manufacturer, meter number, version and type |
| Voice | any band | WFM with stereo and RDS, NFM, AM, USB, LSB, CW, several channels at once |

It transmits too, on a radio that can: a microphone or a tone into NFM, WFM or
AM, drawn as the TX side of the same flow graph.
[`docs/protocols.md`](docs/protocols.md) is the roadmap.

## Hardware

Any RTL2832U dongle, a HackRF One, or a LimeSDR USB or Mini, and a tuner on
another machine over iqstream with `--stream <host>`. A €30 RTL-SDR does all of
the receiving above; a HackRF buys you wider spans and a transmitter, and a
LimeSDR both of those plus full duplex.

## Install

Grab a build from [releases](https://github.com/v0l/waveshark/releases), Linux
x86_64 or Windows x86_64.

The Linux binary links librtlsdr rather than bundling it, so install
`librtlsdr0` or `rtl-sdr` for the udev rules that let you open a dongle without
root. Windows ships the DLLs, but bind WinUSB to the RTL2832U with
[Zadig](https://zadig.akeo.ie/) first or nothing can open the device. The
Windows build has no LimeSDR: LimeSuite is not packaged for it, so that binary
is built without the driver.

From source:

```sh
sudo apt install librtlsdr-dev liblimesuite-dev pkg-config libclang-dev \
  libasound2-dev libx11-dev libxrandr-dev libxi-dev libxcursor-dev \
  libxkbcommon-dev libwayland-dev libgl1-mesa-dev
cargo run --release -p app
```

That build has everything, including two decoders the published binaries do
not carry:

```sh
cargo run --release -p app --no-default-features --features limesdr,stt,mcp
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

## Letting an agent drive

```sh
waveshark --mcp-listen 8931
```

serves the receiver over the Model Context Protocol at
`http://127.0.0.1:8931/mcp`. It is the receiver on the screen, not a second
one: what the agent tunes, opens or switches on appears in the window, and it
can take a picture of that window to see what it did. It can read the
spectrum, the packets, the calls, the transcript and the tracker, and it can
change anything in the signal chain. It cannot transmit.
[`docs/mcp.md`](docs/mcp.md) has the tools and the reasoning.

## Command line

`--help` has the rest.

```
--tune <mhz>           start tuned and listening; repeat for several channels
--mode <mode>          wfm, nfm, am, usb, lsb or cw
--span <khz>           nearest span, narrowed in software if the radio cannot
--device <name>        pick a radio when several are plugged in
--stream <host>        offer an iqstream server as a radio
--location <lat,lon>   your position, for aircraft positions from a single frame
--record [dir]         write every burst that decodes to a directory of captures
--capture-iq           write the raw span from the moment the radio starts
--replay [path]        decode a capture, a directory, or a packet log
--headless             run with no window, scanning and logging as it would
--mcp-listen <addr>    serve this receiver to an agent over MCP, on loopback
--print-log            print every packet as it arrives, window or not
--fetch-data           warm the dataset cache before going somewhere offline
--squelch-probe [mhz]  report what the squelch reads on a frequency
--probe [mhz]          check the signal path with no display
```

`--chain`, `--flights`, `--calls`, `--messages`, `--scanners`, `--gain` and
`--setup` open on a view.

## Status

Verified against other people's decoders, not just its own: 52 recordings from
rtl_433's corpus are replayed field for field against what rtl_433 25.02 made
of them, plus ADS-B against dump1090, and off-air captures of M17, DMR, TETRA
and Meshtastic are asserted against what the transmission itself says. Those
tests need `testdata/fetch.sh` to have pulled the recordings, and skip cleanly
when it has not, which is also what happens in CI. Coverage is the thin part,
thirty-nine ISM decoders where the goal is hundreds, and the browser build
([`docs/web.md`](docs/web.md)) is still a plan.

## Documentation

[`docs/design.md`](docs/design.md) is how it works inside,
[`docs/protocols.md`](docs/protocols.md) the protocol roadmap,
[`docs/views.md`](docs/views.md) how a view attaches to the packet bus,
[`docs/mcp.md`](docs/mcp.md) how an agent drives it, and
[`docs/references.md`](docs/references.md) everything this leans on that
somebody else wrote.

## Licence

GPL-3.0-or-later, text in [`LICENSE`](LICENSE), reasoning at the end of
[`docs/design.md`](docs/design.md).
