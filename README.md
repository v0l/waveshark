# super-radio

A wideband SDR receiver in Rust: channelize a wide span once, then detect and
decode every signal in it in parallel.

## Status

Working receiver, narrow coverage. The signal path runs end to end against real
off-air RF, there is an egui front end, and FM broadcast decodes to stereo audio
with RDS, and ADS-B decodes aircraft off the air. What is missing is protocols:
twenty-five ISM device decoders are implemented where the goal is hundreds, and
only Fine Offset and ADS-B have been checked against real recordings.

Proof it works: `crates/decode/tests/fineoffset_capture.rs` decodes a real
recorded 433.92 MHz weather-station transmission and asserts the result matches
`rtl_433` 25.02 field for field, CRC included. The expected values come from a
separate implementation, so agreement is evidence rather than a restatement of
our own assumptions.

## Layout

| crate | what it does |
|---|---|
| `common` | sample buffers, `Device`/`RxStream` traits, `Hz`/`Sps` units |
| `dsp` | polyphase channelizer, FIR design, mixer, FM/AM demod, FM stereo, RDS, DC blocker, burst detector, OOK/ASK/FSK pulse extraction |
| `pipeline` | the flow graph: typed DAG, rate negotiation, stream tags, events |
| `decode` | bit buffers, pulse slicers, unknown-burst analyser, protocol registry, device decoders |
| `nodes` | DSP and decoders as graph nodes, the registry, and the wideband channel bank |
| `sources` | file replay with rtl_433-style filename metadata |
| `audio` | cpal playback with a drift-tracking resampler |
| `app` | egui front end: spectrum, waterfall, tuner, channels, chain view |
| `rtlsdr-sys` | bindgen FFI to librtlsdr |
| `hackrf` | HackRF One, adapting `rs-hackrf` to the `Device` trait |
| `rtlsdr` | safe driver with an async streaming thread |

Named `common` rather than `core` because a workspace crate called `core`
shadows the Rust sysroot crate.

[`docs/views.md`](docs/views.md) describes the packet stream as a bus: the
list is a view over it, and a map, an image pane or a chart attach the same way, by reading a packet's structured fields and its
media type rather than the demodulator that produced it.

[`docs/protocols.md`](docs/protocols.md) is the protocol roadmap: everything
rtl_433, a Flipper, a PortaPack and SDRangel can do, with what each one costs
here, which direction is realistic for it, and what the transmit path needs
before any of it can transmit.

## Design decisions worth knowing

**Serial graphs, parallel channels.** A flow graph runs on one thread; rayon
spreads independent per-channel graphs across the pool. This is the opposite of
GNU Radio's thread-per-block, and it is deliberate: at 512 channels of five
nodes, thread-per-block means 2560 threads on 48 cores and the scheduler costs
more than the DSP.

**Chains are data, not code.** Nodes are registered by name with
introspectable parameters, so an ambiguous signal is attacked by
reconfiguring the chain rather than recompiling. A mistuned chain reports what
it discarded and which parameter to change, because silence is the worst
possible output for a tool meant to identify unknown signals.

**A shared pulse front end, following rtl_433.** Almost every ISM device is OOK
or two-level FSK, and both reduce to mark/gap timings: `pulse_detect` thresholds
an envelope, `fsk_detect` thresholds a discriminator, `ask_detect` covers keying
too shallow for the first to see, and the slicers and protocols below them
cannot tell which one they were fed. The DSP runs once per
channel; each protocol is then a timing table and a payload parser working on
integers. That is what makes supporting hundreds of protocols affordable.

**One graph, and everything hangs off it.** The receiver is a single
`pipeline::Graph`, from the DC notch and the zoom decimator at the head to the
spectrum, the recorder's ring, the ISM banks, the 1090 MHz decoder and a branch
per channel being listened to. Nothing acts on a sample outside it. Anything
driven beside the graph is invisible to the chain view, absent from the
parameter surface and missing from the latency accounting, and it will drift
from the code that does run.

The shape changes often, and a graph is fixed once built, so a change rebuilds
it out of the *same nodes* (`Graph::into_parts`). Building fresh ones instead
would reset every branch that was left alone: adding a second channel would
cost the first its RDS station, its AGC convergence and its detector's noise
floor. A node is only reused where that is meaningful, so a channel whose
offset, mode or rate changed is built again rather than run with coefficients
designed for something else.

A bank is one node with several hundred graphs inside it, because that is what
makes it fast: one polyphase channelizer, one tiled transpose, one burst
detector, then a rayon sweep of the channels worth running. It reports what its
channels run, so the view shows the decoder rather than an opaque box.

The chain view draws that graph's topology rather than a diagram kept alongside
it, which is the only way it can be trusted: documentation that has drifted is
worse than none, because it is believed.

## Measured throughput

512 channels, each running a full envelope / pulse-detect / protocol-decode
chain, on a 48-thread machine. `x real` above 1.0 keeps up with a live radio.

| input rate | 8 ch | 64 ch | 256 ch | 512 ch |
|---|---|---|---|---|
| 2.4 MS/s (RTL-SDR) | 22.9x | 58.9x | 64.9x | 67.3x |
| 20 MS/s (HackRF) | 3.17x | 7.54x | 9.30x | 10.2x |
| 50 MS/s | 1.34x | 2.69x | 3.63x | 4.19x |
| 100 MS/s | 0.68x | 1.01x | 1.91x | 2.14x |
| 200 MS/s | 0.33x | 0.55x | 0.82x | 1.03x |

Reproduce with `cargo run --release -p nodes --example bench -- 50000000`, in Hz.

Real-time ceiling is a little over 200 MS/s at 512 channels. More channels is
cheaper, not dearer: 8 to 512 channels is three times *faster* at every input
rate, because splitting a fixed input rate across more channels drops the rate
each one runs at, and the filter behind each channel costs per sample. Channel
count is not the thing to economise on. Input rate is.

Getting there needed three things, each found by measuring rather than
guessing, and each initially wrong:

- The **burst detector** ran frame by frame on one thread: 200 million channel
  updates per second of input at 50 MS/s. It now processes channel-major lanes
  in parallel. This was by far the largest cost, and it was invisible until a
  benchmark with *no decode chain at all* still ran at 0.72x.
- The **channelizer** ran on one thread at about 50 MS/s. Overlap-save makes
  every frame independent of every other, so it parallelises; 6x faster.
- The **transpose** wants to be blocked, not eliminated. Letting each channel
  gather its own column has a `channels * 8` byte stride and wastes seven
  eighths of the memory bandwidth.

## Building

```sh
sudo apt install librtlsdr-dev     # or rtl-sdr-devel / rtl-sdr
cargo test --workspace
```

Run the tests in release. Several of them assert the audio chain keeps ahead of
real time, and a debug build misses by enough that the numbers mean nothing.

### Prebuilt binaries

`.github/workflows/build.yml` builds Linux, macOS (Intel and Apple Silicon) and
Windows on every push, and attaches them to a release on a `v*` tag. The
binaries link librtlsdr rather than bundling it, so the copy matching your udev
rules is the one already installed; the Windows zip does ship the DLLs, since
there is no system package to point a Windows user at. Windows also needs
WinUSB bound to the RTL2832U with Zadig before the device can be opened.

## Test fixtures

Recorded IQ lives on nostr.download, not in git; it is near-incompressible and
would bloat history permanently.

```sh
./testdata/fetch.sh
```

Tests that need a fixture skip cleanly when it is missing, so a fresh clone
builds and passes without network access.

### Making your own

`--record <dir>` writes every burst the scanner reports, whether it decoded or
not, as an ordinary rtl_433 style capture:

```sh
super-radio --tune 868.3 --record captures
super-radio --replay captures
```

The recorder keeps the last three quarters of a second of signal in memory and
writes it out when a decode arrives, because a packet is reported long after it
was transmitted: the transmission itself takes tens of milliseconds, the pulse
detector waits for the silence after it, and the filters add their own latency.
Measured on the Fine Offset capture, a quarter of a second of history loses the
packet entirely and three tenths catches it.

Each burst is mixed down to its own frequency, decimated to 250 kS/s and
written as `g0001_<protocol>_<mod>_<freq>M_<rate>k.cu8`, about 380 kB, so the
filename alone tells `sources::FileSource` and rtl_433 everything they need. An
`index.jsonl` alongside records what was made of each one. Recording stops
after 256 MB rather than filling the disk overnight.

This is the short loop for protocol work: capture the band once, then run
`--replay` after every change and see immediately whether the same burst now
decodes, with no radio and the same answer every time. A capture that decodes
is also a test fixture.

## Try it

```sh
# Sweep a band and rank channels by power
cargo run --release -p rtlsdr --example scan -- 868.3 49.6

# Dump pulse trains and timing histograms from live RF
cargo run --release -p rtlsdr --example ism -- 433.92 30

# Same, from a recorded capture
cargo run --release -p sources --example pulses -- testdata/fineoffset_wh1080_433.92M_250k.cu8

# Build a decode chain at runtime; no argument lists the available nodes
cargo run --release -p nodes --example chain -- \
  testdata/fineoffset_wh1080_433.92M_250k.cu8 \
  'envelope | pulse_detect:reset_us=10000,min_pulses=20 | protocol_decode'

# WFM receiver; verifies itself by finding the 19 kHz stereo pilot
cargo run --release -p rtlsdr --example wfm

# Listen live. 2.304 MS/s / 8 / 6 is exactly 48 kHz, so this example needs no
# resampling; the drift loop only absorbs crystal mismatch between the radio
# and the sound card. The app picks its own rates and does resample, because
# WFM needs an IF above 330 kHz to reach the 57 kHz RDS subcarrier and 2.304
# does not divide into that and 48 kHz both.
cargo run --release -p rtlsdr --example listen -- 95.8
cargo run --release -p rtlsdr --example listen -- 446.0 nfm
```

## The app

```
cargo run --release -p app
```

The top bar carries only what you change while tuning: frequency, band,
bandwidth, view, gain, the device with its start and stop, and the fault lamps.
Everything else lives behind a cog in the corner of the pane it affects, because
spectrum and waterfall settings are unrelated and a single settings screen makes
both harder to find.

Stopping releases the USB claim without quitting, which is the only way to hand
the radio to another program: a held claim is what makes the next process fail
to open it at all.

Where it was pointing last time is where it starts: device, centre frequency,
span and zoom, every gain stage, the radio's switches, the crystal correction,
and whether decoding was on. They are written to
`$XDG_CONFIG_HOME/super-radio/session` as plain `key = value` lines, a couple of
seconds after the last change and again on exit, so the file is editable by hand
and a corrupt one costs the settings rather than the startup. With nothing saved
it opens on 433.92 MHz, where the devices this thing decodes actually are.

### Spectrum

Click the pane to place a channel and start listening, click an existing marker
to switch to it, drag a marker to move that channel, drag anywhere else to pan.
Which one a drag means is decided when it starts, so the gesture cannot change
meaning halfway through as the pointer leaves the line it grabbed.

Hold shift while placing or dragging to snap to the band's channel plan: 100 kHz
in the FM broadcast band, 25 kHz in the airband and marine VHF, 12.5 kHz on
2 m and 70 cm. Bands with no even raster declare none and do not snap, because
moving a channel off the signal it was aimed at is worse than not helping.

Every channel is demodulated at once and mixed together. Each has its own
level and mute, running into a master volume, so a busy amateur calling
frequency and a local repeater can be listened to side by side without
switching between them. A channel that is switched off costs nothing: its
chain is torn down rather than muted.

Measured at 2.304 MS/s, one WFM channel takes about 17% of the radio thread
and two take 31%, so the thread runs out somewhere around six broadcast
channels and rather more narrowband ones. Chains are kept across edits and
rebuilt only when what they demodulate changes, because rebuilding on every
volume nudge would restart each AGC and throw away the RDS decoder's state
along with the station name it had recovered.

The mix clips rather than being scaled to fit. Several channels at once can
sum past full scale, and quietly turning everything down would make the level
of the channel you are listening to depend on how busy its neighbours are.

Mode defaults from the band plan (WFM in the broadcast band, AM in the airband,
otherwise NFM) and can be overridden per channel: WFM, NFM and AM for
broadcast and utility listening, USB, LSB and CW for the amateur bands.

The three amateur modes share a sideband filter with complex taps, which is
the direct way to keep one sideband and reject the other: an ordinary lowpass
modulated up to the sideband's centre. A 2.4 kHz voice filter at 48 kHz takes
363 taps, puts its stated edges on -6 dB, and rejects the wrong sideband by
more than 50 dB. CW is the same filter at 500 Hz around the pitch, and the
receiver is tuned low by that pitch so the dial reads the transmitted carrier
rather than the note in your ears.

Both are unusable without gain control, because an SSB signal arrives with no
level control of any kind at the far end. The AGC is the conventional shape,
fast attack and slow release with a hang in between, and it freezes while the
squelch is shut: releasing through a muted channel winds the gain to maximum
and then blasts the first syllable after the squelch opens.

FM squelches on noise rather than level. An FM discriminator with no signal on
it produces mostly high frequency hiss, and any signal at all pushes that down
because FM captures, so measuring the energy above the speech band finds a
station at the point where it becomes intelligible instead of at some level
picked in advance. That measurement has to happen on the discriminator's raw
output: with the squelch after the audio filter, the filter had already removed
the noise the meter looks for and the squelch sat open on an empty channel.
The threshold is measured rather than picked. Through this chain an empty
channel reads about 6.4 dB and an FM signal reads 24 dB and barely moves with
signal strength, because FM captures, so the default sits at 14 dB in the
middle of that gap. It was 9 dB, which is inside the noise's own variation:
live, an empty 2 m channel wanders between 5.6 and 8.4 dB, so any excursion
opened the squelch and the hysteresis then held it open on hiss indefinitely.
`--squelch-probe` reports those numbers for a frequency, which is how the
threshold was set and how to set your own.

AM, USB, LSB and CW have no capture effect to measure, so they squelch on
level, and they start with it off. There is no sensible fixed setting: on an
empty 2 m channel the audio sits at -26 dBFS in AM, -36 in USB and -59 in CW,
all of which move with the RF gain, so a number chosen here would do nothing
in one mode and mute a station in another. The meter under each channel shows
what it is reading now, which is what makes setting one by hand possible.

The bandwidth list goes below what the radio can sample. A HackRF will not
sample slower than 2 MS/s, and on a 2 MHz span a 12.5 kHz PMR446 channel is
half a pixel wide, which is not something a cursor can be placed on. Spans
below the hardware's minimum are the lowest rate it does have, decimated in
software: the list marks them `/8`, `/32` and so on. At 125 kHz that same
channel is 120 pixels across.

Everything downstream sees the narrowed stream at its new rate, so the
spectrum, the channel decoder and the audio all agree about what a frequency
is. The decimator filters before it drops samples, at 70 dB, because anything
folded in from outside the span cannot afterwards be told from a signal that
was really there. Measured at 2.4 MS/s it runs at 25 to 30 times real time
from /2 to /32, on a thread that also has the spectrum, the scanner and the
audio to get through.

Spans narrower than 48 kHz are not offered: that is the rate the narrowband
audio chain runs at, and a span narrower than the demodulator's own IF cannot
be listened to. Asking for one anyway reports which mode needs what.

GAIN opens the radio's own controls, separately from anything the software
does to the samples afterwards. Each gain stage the device reports gets a
slider that snaps to the steps the hardware actually has: an R820T takes 29
discrete values and nothing between them, and a HackRF's LNA moves in 8 dB
steps while its VGA moves in 2. The values shown are read back from the driver
rather than remembered, because asking for 30 dB on an R820T gets you 29.7.

A HackRF's three stages appear separately rather than as one number. They do
different jobs: the LNA sets the noise figure, the VGA drives the converter,
and the front end amp costs noise figure and overloads on a crowded band, so
which one you raise matters as much as the total.

The switches a device offers appear there too, each with what it costs: the
RTL2832U's digital AGC, which rescues a weak signal and ruins any wideband
measurement because the noise floor moves under you; the bias tee, which puts
4.5 V on the antenna socket; direct sampling for HF on a v3 dongle. Reference
oscillator correction and the DC spur removal are in the same place, since
both are about making the receiver honest rather than about the display.

The display auto-scales against the 10th and 99.9th percentiles rather than the
extremes, so one strong carrier does not flatten everything else.

Frequencies are per-digit dials, one for the receiver and one in each channel:
hover a digit and scroll to step that decade, the way you pick a tuning step on
a hardware receiver, and right-click a digit to clear it and everything under
it. Mouse wheel over the spectrum itself scrubs the centre by a twentieth of
the span per notch.

The two accent colours carry meaning rather than decoration: amber is what you
set, cyan is what the radio hears, so an RDS station name is cyan. The waterfall
ramps between them.

Panning slides the waterfall history sideways rather than discarding it, and the
trace is positioned by the frequency its data was taken at, so it slides under a
drag instead of being stretched across the pane while the radio catches up.
Retuning costs about 25 ms of blocked USB, so they are spaced out rather than
issued once per frame.

Each channel is drawn at the width its demodulator actually accepts, so an NFM
and a WFM channel on the same frequency look as different as they are.

The spectrum and the waterfall are split by a handle under the band ribbon:
drag it to give either one more room, double click it to go back to the
default. The packet log resizes the same way, by its top edge. Which pane
matters depends on what is being looked for, so none of the three is fixed.

### Packet log

Every decoded packet is appended to `$XDG_DATA_HOME/super-radio/packets`, one
JSON object per line, one file per day. It is on by default and has no switch
in the interface, because the value of a packet log is in already having it:
the interesting transmission is always the one that happened before anyone
thought to press record. A receiver left on a band overnight is a test corpus,
and real frames are the only honest way to tell whether a change to a decoder
helped.

```
{"at":1788185123.791,"freq":433920000,"model":"Fineoffset-WHx080",
 "modulation":"OOK","rssi_dbfs":-27.4,"snr_db":31.9,"crc_ok":true,
 "bytes":"ffac42473d0306211906fc",
 "fields":{"battery_ok":true,"humidity_pct":61,"temperature_c":15.9}}
```

`crc_ok` is absent rather than false for a protocol that has no integrity
check, so filtering on the key separates "verified" from "unverifiable" without
a special case. `--packet-log <dir>` moves it, `--no-packet-log` turns it off,
and it stops appending at 512 MB, which is a runaway guard rather than a
budget.

### ADS-B

Tune to 1090 MHz with a span of 2 MS/s or more and aircraft appear in the same
packet list as everything else. Mode S does not go through the channel banks:
its bits are 1 us wide, a thousand times faster than anything on 433 MHz, so it
runs on the wideband stream as a branch of the same graph, and the banks are
switched off while it does, since 1090 MHz carries nothing they understand.

It is a single node rather than the usual front end plus protocol pair, and the
reason is in `crates/nodes/src/modes_nodes.rs`: a
believed frame blanks the 120 us it occupies, so a false preamble destroys
every real frame overlapping it, and only the CRC can tell the two apart. The
acceptance test therefore has to run inside the search rather than downstream
of it. Split across two nodes it recovers 8 frames from a recorded band where
one node recovers 27.

Frames arrive as `ADSB-Position`, `ADSB-Velocity`, `ADSB-Identification` and
`ModeS-Reply` rows with structured fields: ICAO address, callsign, altitude,
ground speed, track, and the encoded halves of a position. Turning a pair of
those into a latitude belongs to whatever tracks aircraft over time, not to a
row in a packet log, so `decode::adsb` offers `cpr_global` and `cpr_local` and
leaves the choice to the caller.

Short replies are the awkward part. DF0, 4, 5, 20 and 21 overlay the aircraft
address on their parity field, so they cannot verify themselves and 56 bits of
noise decode to a plausible-looking one every time. They are believed only once
their address has been seen three times in frames the demodulator read without
a single marginal bit. Dropping that rule produced 86 phantom aircraft on a four
second capture; keeping it produced none.

### Flights

The `Flights` view in the top bar is a table of aircraft rather than of
packets: callsign, ICAO address, altitude, ground speed, track, climb rate,
position and how long since the last frame. It is a view over the same packet
stream the log shows, reading the structured fields of a decode and knowing
nothing about how those bytes arrived, which is what `docs/views.md` means by a
bus. Point it at live packets or a day of the packet log and it behaves the
same.

The work it does is that no single ADS-B frame says where an aircraft is. One
carries a callsign, another an altitude, another half a position, and they
arrive interleaved with everyone else's. A position needs either both halves
within ten seconds of each other, or a reference within 180 nautical miles.
`--location 53.64,-6.65` supplies the latter, which is worth doing: with it,
the first frame from an aircraft puts it on the map instead of the first
matching pair. The value is remembered in the session file.

### Decoding the whole span

Data decoding is not something you tune to. The receiver splits whatever span
it is on into channels and runs a decoder on every one of them at once, all
the time, so a sensor that transmits once a minute is caught whether or not you
were looking at its frequency. Both an OOK and an FSK front end run over the
whole span, because which modulation a device uses is not visible in a
waterfall.

It splits the span twice, because the two front ends want opposite things from
a channel. Measured by adding noise to the Fine Offset capture until decoding
stops, a 1.5 kbit/s OOK sensor survives down to 12.3 dB peak-to-noise in a
31 kHz channel and needs 22.9 dB in a 125 kHz one: a wide channel integrates
noise across its whole width while the signal occupies a sliver of it, so it
costs 10.6 dB for nothing. FSK needs the opposite, because its two tones are
tens of kHz apart and a narrow channel cuts one of them off: the same
synthetic packet reads as 46 bits at 110 us a symbol in a 125 kHz channel and
as eight bits of nonsense in a 31 kHz one. So there are two channelizers, a
31.25 kHz bank feeding the OOK path and a 125 kHz bank feeding the FSK path.

Neither front end needs the signal centred, which is what makes any of it
work: the OOK path is an envelope detector and does not care where in the
channel the carrier sits, and the FSK path measures both tones from the burst
itself, so a SAW transmitter tens of kHz off nominal reads the same as one on
frequency. Channel width therefore costs sensitivity and nothing else.

Channels overlap by design, so one burst is seen by several of them and by
both banks at once, each reading a different mangled copy. The strongest
report of a burst wins and a real decode beats a louder guess, so a
transmission appears once. A device that repeats its packet still gets a row
per repeat, because those are separate bursts on the same channel through the
same front end.

Bursts that match no protocol are reported too, and that is the point of
scanning a band rather than a frequency: an unknown device is exactly what
should be surfaced. The coding is inferred from the burst's own histograms
(two mark widths and one gap width is PWM, one mark and two gaps is PPM,
widths at T and 2T in both is Manchester), the symbol timings are measured
from the burst, and the bits are sliced out under that reading. It is a guess,
labelled as one, and it is enough to recognise the same device ID across
several receptions, which is where reverse engineering starts.

The packet list is the bottom pane:

```
 no     time   frequency   mod   rssi   snr  protocol           len  info
 27    4.267  433.9200MHz  OOK  -18.0  21.5  Fineoffset-WHx080   11  station_id=196 temperature_c=16.2
 28    4.284  868.3500MHz  FSK  -26.0  13.0  unknown              6  PWM 488/1464 us, 46 pulses, 46 bits
```

RSSI and SNR are both there because either alone misleads: a strong packet in
a noisy channel and a weak one in a quiet channel share an SNR, and only the
level says the front end is clipping. It is referenced to full scale at the
detector, not to the antenna, so it compares packets on one receiver and is
not a field strength. Clicking a packet opens an offset/hex/ASCII dump of its
bytes. `UNKNOWN` hides unclaimed bursts when a noisy band buries the decodes.

Idle channels cost only the burst detector, which is why the whole band can be
covered continuously even with two banks running: measured at 2.4 MS/s over 78
narrow and 20 wide channels, the scanner runs at about 17x real time.
`DECODE ALL` in the top bar turns it off, `LOG` hides the list.

### Signal chain

This view draws the graph, with the type and rate on every link and the delay
through the chain. It is read from the built graph, so it shows what is running
rather than what was intended.

It shows all of it, because there is only one: the head of the chain, then the
spectrum, the recorder, both ISM banks, the 1090 MHz decoder and every channel
you are listening to, each as a branch of its own. Branches are drawn abreast
rather than stacked, since a column would say one feeds the next. A bank draws
the chain its channels run beneath it, with the channel count. A sink is drawn
without an output rate because it produces no stream, only decodes or a
display.

### FM broadcast

WFM decodes stereo and RDS from one pilot PLL. The station name, programme type
and radiotext appear in the channel card. A block that fails its syndrome check
is discarded rather than published, so a station with no RDS shows nothing at
all instead of plausible noise.

### Direct conversion spur

A zero-IF receiver puts oscillator leakage at exactly the tuned frequency, which
reads as a very strong carrier and is not a signal. It is removed by default;
the toggle is under the spectrum cog. Measured on a HackRF at 95.8 MHz the
centre bin sits 47 dB above the noise floor with this off and 1 dB below it
with this on.

### Command line

`--help` lists everything with its defaults; the ones worth knowing:

```
--tune <mhz>        start tuned and listening; repeat it for several channels
--mode <mode>       wfm, nfm, am, usb, lsb or cw
--span <khz>        start at the nearest span, narrowing in software if needed
--gain              open the radio's own controls on start
--record [dir]      write every burst that decodes to a directory of captures
--replay [path]     decode a capture, or a directory of them, and print it
--shot [path]       write a PNG of the interface and exit
--squelch-probe [mhz]  report what the squelch reads on a frequency
--soak [secs]       run for N seconds, then report CPU and span timings
--probe [mhz]       check the signal path with no display
```

Span times under `--soak` are wall clock, so `rf_read` sitting near 100% means
the radio thread is idle waiting on USB, which is exactly what it should be
doing.

## Running in the browser

Not implemented. [`docs/web.md`](docs/web.md) has the plan: what crosses
unchanged, what has to be rewritten, and why the wideband bank does not make
the trip.

## Reference implementations

Decoders worth reading before writing our own, all MIT and all pure Rust:

| crate | covers |
|---|---|
| `fmradio` | FM with RDS and adaptive resampling |
| `dabradio` | DAB/DAB+, OFDM and AAC |
| `voracious` | VOR / ILS / DME |
| `jet1090` | ADS-B |
| `ship162` | AIS |
| `datalink` | VDL2 and ARINC 629 |

They share the Desperado workspace with `rs-hackrf`, which this repo already
depends on for HackRF USB support.
