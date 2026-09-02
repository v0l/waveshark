# How WaveShark works

Notes on the internals, the measurements behind them, and the mistakes that
produced the current shape. For what the program does and how to run it, see
the [README](../README.md).

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
| `limesdr-sys` | bindgen FFI to LimeSuite's LMS API |
| `limesdr` | LimeSDR-USB and Mini, 100 kHz to 3.8 GHz |

Named `common` rather than `core` because a workspace crate called `core`
shadows the Rust sysroot crate.

## Design decisions worth knowing

**Serial graphs, parallel channels.** A flow graph runs on one thread; rayon
spreads independent per-channel graphs across the pool. This is the opposite of
GNU Radio's thread-per-block, and it is deliberate: at 512 channels of five
nodes, thread-per-block means 2560 threads on 48 cores and the scheduler costs
more than the DSP.

**Chains are data, not code.** Nodes are registered by name with
introspectable parameters, so an ambiguous signal is attacked by reconfiguring
the chain rather than recompiling. A mistuned chain reports what it discarded
and which parameter to change, because silence is the worst possible output for
a tool meant to identify unknown signals.

**A shared pulse front end, following rtl_433.** Almost every ISM device is OOK
or two-level FSK, and both reduce to mark/gap timings: `pulse_detect` thresholds
an envelope, `fsk_detect` thresholds a discriminator, `ask_detect` covers keying
too shallow for the first to see, and the slicers and protocols below them
cannot tell which one they were fed. The DSP runs once per channel; each
protocol is then a timing table and a payload parser working on integers. That
is what makes supporting hundreds of protocols affordable.

Each channel measures a burst before demodulating it rather than running every
front end over every sample. The measurement names on-off keying, shallow ASK,
two-level and four-level FSK, minimum-shift keying, binary and quadrature phase
keying, chirp, a bare carrier and noise-like, and it refuses when nothing fits,
in which case the burst goes to both pulse front ends. Scored against rtl_433's
recordings it puts 46 of 52 in the right family, and the six it misses are
listed by name in the test.

Frames can also arrive from another receiver. A feed over TCP is a front end
like any other: it produces packets, so it attaches upstream of the bus and
everything downstream treats its frames the same as the ones demodulated here.
A wire format is a row in a table with a parser beside it, Beast and AVR to
begin with.

The protocols themselves run once, on the packet bus, rather than inside every
channel. A decoder per channel meant a hundred copies of the same tables, and a
burst that arrived by any other route (a log being replayed, a front end added
later) got no decoding at all. Measured on the throughput test it is also
faster: 12.0x real time against 10.5x, because the tables are consulted once
per burst rather than once per burst per channel.

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

The graph is built from a description rather than by hand. The receiver draws
one for itself out of what it is doing (`chain::derived_patch`): the DC block
and the zoom decimator, the spectrum, the recorder's ring, the band extractions
and front ends the scanner table asks for, eight stages per listening channel,
the packet bus and the protocols and the tracker. Every one of them is a
registry stage with settings and wires, named by an id computed from what it is
for, which is what lets a node keep its state across the rebuild that changed
the shape around it.

The chain view draws that description, and in manual mode it edits it. Stages
can be added from the block list, deleted, dragged, and wired by pulling a wire
from either end onto a port or a box; a stage whose inputs are not all fed yet
is left out of the built graph rather than refusing the whole receiver, and an
edit that will not build is refused and the last graph that did build goes back,
so a wrong wire cannot stop the radio. The graph and where its boxes sit are
saved in `~/.config/waveshark/patch`. While manual mode is on, the scanner table
and the decode switch no longer rebuild the graph: it is yours.

Either way what is drawn is what runs, which is the only way it can be trusted:
documentation that has drifted is worse than none, because it is believed. It is
also where a stage is set by hand. Every node describes its own knobs as data,
which is what lets a chain be saved and reloaded without each stage writing
serialisation glue; the same description renders the controls, so a decoder the
interface has never heard of gets a working panel.

## Verification

`crates/decode/tests/rtl433_corpus.rs` replays 52 recordings from rtl_433's own
test corpus and asserts every decode matches the reference JSON rtl_433 25.02
produced for that file, field for field. Twenty-six device models are covered,
plus ADS-B against dump1090 over a shared recording. The expected values come
from separate implementations, so agreement is evidence rather than a
restatement of our own assumptions.

That corpus earned its place immediately. Four decoders that passed every
synthetic test in this repository decoded nothing at all off real RF, and one
reported the wrong sensor at the wrong temperature with its checksum passing.
The causes were in the shared layers rather than in any one protocol: the
slicers were throwing away the gap between repeats, which is the only evidence
of where a frame begins.

The Manchester slicer went the same way later. It expanded each mark and gap
into half symbols and paired them, which loses the whole frame as soon as one
width rounds to the wrong number of halves, and on rtl_433's Oregon Scientific
recordings that was half of them: fourteen captures out of twenty-eight
decoded, with a correct decoder in front of them. Tracking the time since the
last data edge instead, the way rtl_433 does, costs one bit per mistimed edge
rather than the rest of the transmission, and all twenty-eight decode.

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

Those numbers are for a bank over the whole input. A bank is not given the
whole input any more: each one is mixed and decimated down to the band its
scanner block declares, so a 1.7 MHz ISM band costs the same whether the radio
is at 2.4 MS/s or 61.44. That is the difference between the channelizer being
the expensive thing on a wide span and it being free.

It is also the difference between the channels being the width that was asked
for and not. A bank tops out at 1024 channels, so at 61.44 MS/s a span-wide
bank gives 60 kHz channels where an OOK sensor needs about 25, and several
devices land in one channel where the detector sees a single long burst rather
than packets. Anchoring the bank to the band also anchors its channel grid: it
no longer slides under the signals every time the dial moves, and a drag across
the spectrum costs one mixer shift instead of a rebuilt channelizer, a reset
detector and a lost half-packet per frame.

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

## Two channelizers, not one

Within a band that channelizes, the receiver splits the span into channels and
runs a decoder on every one at once, all the time, so a sensor that transmits
once a minute is caught whether or not you were looking at its frequency.

It splits the span twice, because the two front ends want opposite things from
a channel. Measured by adding noise to the Fine Offset capture until decoding
stops, a 1.5 kbit/s OOK sensor survives down to 12.3 dB peak-to-noise in a
31 kHz channel and needs 22.9 dB in a 125 kHz one: a wide channel integrates
noise across its whole width while the signal occupies a sliver of it, so it
costs 10.6 dB for nothing. FSK needs the opposite, because its two tones are
tens of kHz apart and a narrow channel cuts one of them off: the same synthetic
packet reads as 46 bits at 110 us a symbol in a 125 kHz channel and as eight
bits of nonsense in a 31 kHz one. So there are two channelizers, a 31.25 kHz
bank feeding the OOK path and a 125 kHz bank feeding the FSK path.

Neither front end needs the signal centred, which is what makes any of it work:
the OOK path is an envelope detector and does not care where in the channel the
carrier sits, and the FSK path measures both tones from the burst itself, so a
SAW transmitter tens of kHz off nominal reads the same as one on frequency.
Channel width therefore costs sensitivity and nothing else.

Channels overlap by design, so one burst is seen by several of them and by both
banks at once, each reading a different mangled copy. The strongest report of a
burst wins and a real decode beats a louder guess, so a transmission appears
once. A device that repeats its packet still gets a row per repeat, because
those are separate bursts on the same channel through the same front end.

Bursts that match no protocol are reported too, and that is the point of
scanning a band rather than a frequency. The coding is inferred from the
burst's own histograms (two mark widths and one gap width is PWM, one mark and
two gaps is PPM, widths at T and 2T in both is Manchester), the symbol timings
are measured from the burst, and the bits are sliced out under that reading. It
is a guess, labelled as one, and it is enough to recognise the same device ID
across several receptions, which is where reverse engineering starts.

Idle channels cost only the burst detector, which is why the whole band can be
covered continuously even with two banks running: measured at 2.4 MS/s over 78
narrow and 20 wide channels, the scanner runs at about 17x real time.

## The packet log format

What is stored is what the demodulator produced: the mark and gap timings of
each burst, or the frame bytes where the demodulator makes bytes rather than
timings. The parsed frame is deliberately absent. A parse is a conclusion, and
a conclusion stored without the evidence cannot be checked later, corrected by
a better decoder, or shown to have been wrong. Timings can be decoded again
next year; a field map cannot be un-decoded.

Undecoded bursts are written too, and they matter most: a burst no protocol
claimed leaves no other trace at all, and it is the raw material for the
protocol that would have claimed it.

The log is the packet bus, and it is a node in the graph. Everything that
produces packets feeds it; everything that consumes them hangs off the far
side, starting with the flight list. A view is a consumer of what the
demodulator produced, not something the interface assembles out of the packets
that happened to reach it, so the tracker keeps running whether or not anyone
is looking at the list. Wiring each view to the demodulator it happens to care
about would rebuild the same fan-out for every one of them, and leave each one
blind whenever its source was not running.

The file is optional and the bus is not: turning the log off stops writing to
disk rather than disconnecting every view from the traffic.

The format is little-endian binary: a six-byte magic and a version, then a
length-prefixed record per burst carrying the time, frequency, channel width,
level, noise and the timings themselves. Binary rather than line-delimited
JSON, because a burst is a few hundred timings and a hundred bytes of quoted
JSON per pulse turns an overnight capture into gigabytes. The length prefix
means an unknown record kind is skipped rather than misparsed, and a receiver
killed mid-write costs the last record rather than the file.

## Recording captures

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

## ADS-B

Mode S does not go through the channel banks: its bits are 1 us wide, a
thousand times faster than anything on 433 MHz, so it runs on the wideband
stream as a branch of the same graph, and the banks are switched off while it
does, since 1090 MHz carries nothing they understand.

It is a single node rather than the usual front end plus protocol pair, and the
reason is in `crates/nodes/src/modes_nodes.rs`: a believed frame blanks the
120 us it occupies, so a false preamble destroys every real frame overlapping
it, and only the CRC can tell the two apart. The acceptance test therefore has
to run inside the search rather than downstream of it. Split across two nodes
it recovers 8 frames from a recorded band where one node recovers 27.

Turning a pair of position frames into a latitude belongs to whatever tracks
aircraft over time, not to a row in a packet log, so `decode::adsb` offers
`cpr_global` and `cpr_local` and leaves the choice to the caller. A position
needs either both halves within ten seconds of each other, or a reference
within 180 nautical miles, which is what `--location` supplies.

Short replies are the awkward part. DF0, 4, 5, 20 and 21 overlay the aircraft
address on their parity field, so they cannot verify themselves and 56 bits of
noise decode to a plausible-looking one every time. They are believed only once
their address has been seen three times in frames the demodulator read without
a single marginal bit. Dropping that rule produced 86 phantom aircraft on a
four second capture; keeping it produced none.

## APRS

Two layers of modulation: the channel is ordinary narrowband FM, and the data
is in the audio as Bell 202 tones at 1200 and 2200 Hz. Above the tones it is
AX.25, which is HDLC, which is the same link layer AIS uses, so the flags, the
bit destuffing and the check sequence are shared code.

Positions arrive in three encodings that share almost nothing. Uncompressed is
readable off the packet, compressed packs the same thing into thirteen
characters of base 91, and Mic-E splits it between the payload and the
destination callsign, abusing an address field to carry latitude digits because
a frame has to have a destination anyway. Most vehicle trackers send Mic-E, so
a decoder without it misses most of what moves.

The airports and their frequencies drawn under the aircraft are a bundled slice
of the public-domain OurAirports dataset (`crates/app/data/`), not something
fetched at runtime.

## POCSAG

Plain NRZ two-level FSK, which the discriminator already produces, so the work
is in the framing: a sync word, batches of sixteen codewords, and BCH(31,21)
over every one of them, correcting up to two bit errors. Nothing in the signal
says whether it is being sent at 512, 1200 or 2400 bits per second, so all
three bit clocks run at once and the sync word decides which was right.
Polarity is settled the same way, by searching for the sync word and for its
complement, because a pager transmission is routinely received upside down and
a receiver that only worked one way up would appear perfect on one radio and
deaf on another.

A page is addressed by a 21-bit number, and only 18 of those bits are in the
codeword: the other three are the position of the codeword in the batch,
because a pager only listens during its own frame. Message characters are sent
least significant bit first inside codewords that are sent most significant bit
first, and skipping that reversal still produces printable text, which is why
`crates/decode/src/pocsag.rs` is checked against a published off-air capture
decoded by somebody else's program rather than only against itself.

## Squelch, AGC and the sideband filter

The three amateur modes share a sideband filter with complex taps, which is the
direct way to keep one sideband and reject the other: an ordinary lowpass
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
Through this chain an empty channel reads about 6.4 dB and an FM signal reads
24 dB and barely moves with signal strength, so the default sits at 14 dB in
the middle of that gap. It was 9 dB, which is inside the noise's own variation:
live, an empty 2 m channel wanders between 5.6 and 8.4 dB, so any excursion
opened the squelch and the hysteresis then held it open on hiss indefinitely.
`--squelch-probe` reports those numbers for a frequency.

AM, USB, LSB and CW have no capture effect to measure, so they squelch on level
and start with it off. There is no sensible fixed setting: on an empty 2 m
channel the audio sits at -26 dBFS in AM, -36 in USB and -59 in CW, all of
which move with the RF gain.

## Radio controls

Each gain stage the device reports gets a slider that snaps to the steps the
hardware actually has: an R820T takes 29 discrete values and nothing between
them, and a HackRF's LNA moves in 8 dB steps while its VGA moves in 2. The
values shown are read back from the driver rather than remembered, because
asking for 30 dB on an R820T gets you 29.7.

A HackRF's three stages appear separately rather than as one number. They do
different jobs: the LNA sets the noise figure, the VGA drives the converter,
and the front end amp costs noise figure and overloads on a crowded band.

A LimeSDR is the exception and shows one number for 0 to 73 dB. Its LNA, TIA
and PGA are split by a table inside LimeSuite, and reimplementing that split
through register writes to draw three sliders would change the noise figure in
ways nothing here could account for. It also gets two dropdowns, because a
switch cannot say which of six sockets the cable is in: the board has an H, L
and W port on each of two receivers, separately matched, and Auto picks L below
1.5 GHz and H above. A cable in RX2_H hears almost nothing at 100 MHz, which
looks exactly like a dead radio. `cargo run --release --example limediag`
answers that question directly: it plays the chip's internal test tone to prove
the converter and the path back to the host, then sweeps the gain, since a live
analogue front end has a noise floor that climbs with RF gain and a dead one
does not.

## Spans narrower than the hardware

A HackRF will not sample slower than 2 MS/s, and on a 2 MHz span a 12.5 kHz
PMR446 channel is half a pixel wide. Spans below the hardware's minimum are the
lowest rate it does have, decimated in software; the list marks them `/8`,
`/32` and so on. At 125 kHz that same channel is 120 pixels across.

Everything downstream sees the narrowed stream at its new rate, so the
spectrum, the channel decoder and the audio all agree about what a frequency
is. The decimator filters before it drops samples, at 70 dB, because anything
folded in from outside the span cannot afterwards be told from a signal that
was really there. Measured at 2.4 MS/s it runs at 25 to 30 times real time from
/2 to /32. Spans narrower than 48 kHz are not offered: that is the rate the
narrowband audio chain runs at.

## Interface rules

The top bar carries only what you change while tuning. Everything else lives
behind a cog in the corner of the pane it affects, because spectrum and
waterfall settings are unrelated and a single settings screen makes both harder
to find. SETUP is the exception: it holds the settings that are true of the
installation rather than the session.

The two accent colours carry meaning rather than decoration: amber is what you
set, cyan is what the radio hears, so an RDS station name is cyan. The
waterfall ramps between them.

The display auto-scales against the 10th and 99.9th percentiles rather than the
extremes, so one strong carrier does not flatten everything else.

Panning slides the waterfall history sideways rather than discarding it, and
the trace is positioned by the frequency its data was taken at, so it slides
under a drag instead of being stretched across the pane while the radio catches
up. Retuning costs about 25 ms of blocked USB, so retunes are spaced out rather
than issued once per frame.

Which gesture a drag means is decided when it starts, so it cannot change
meaning halfway through as the pointer leaves the line it grabbed.

Every channel is demodulated at once and mixed together, each with its own
level and mute into a master volume. The mix clips rather than being scaled to
fit: several channels at once can sum past full scale, and quietly turning
everything down would make the level of the channel you are listening to depend
on how busy its neighbours are. A channel that is switched off costs nothing,
because its chain is torn down rather than muted. Measured at 2.304 MS/s, one
WFM channel takes about 17% of the radio thread and two take 31%, so the thread
runs out somewhere around six broadcast channels and rather more narrowband
ones.

Band plans differ by ITU region and by regulator inside one, so a table that is
right in Dublin is wrong in Denver: 915 MHz is the licence-free band an
American sees key fobs and weather sensors in, and the GSM uplink a European
sees phones in. Three tables ship, and the plan decides both the band names
under the spectrum and the channel spacing a frequency snaps to. The American
FM raster is the odd tenths from 88.1, so snapping to 100 kHz there would put
every station on a guard channel. With nothing saved the country comes from
`LC_ALL`, `LC_MESSAGES` or `LANG`; a bare `en` is ignored rather than guessed
at.

Band and protocol names stay as they are on the air in every language: a
translated "POCSAG" is worse than an untranslated one, because it cannot be
searched for.

Stopping the radio releases the USB claim without quitting, which is the only
way to hand the device to another program.

## Direct conversion spur

A zero-IF receiver puts oscillator leakage at exactly the tuned frequency,
which reads as a very strong carrier and is not a signal. It is removed by
default. Measured on a HackRF at 95.8 MHz the centre bin sits 47 dB above the
noise floor with this off and 1 dB below it with this on.

## Test fixtures

Recorded IQ lives on nostr.download, not in git; it is near-incompressible and
would bloat history permanently. `./testdata/fetch.sh` pulls it. Tests that
need a fixture skip cleanly when it is missing, so a fresh clone builds and
passes without network access.

The same script fetches the rtl_433 corpus samples listed in
`testdata/rtl433.toml`: a capture and the reference decode beside it, straight
from `merbanan/rtl_433_tests` at a pinned commit. Those recordings were
contributed by their owners under no stated licence, so this repository points
at them rather than copying them.

To see what the corpus covers and what each capture decodes to:

```sh
cargo test --release -p decode --test rtl433_corpus -- --ignored --nocapture
```

Adding a protocol is worth pairing with a capture from that corpus. Pick one
whose reference JSON names the device, add it to `testdata/rtl433.toml` with
the frequency and sample rate in the local filename, and the existing test
picks it up: it compares every field, and separately requires that nothing
reporting a passing integrity check claims a burst rtl_433 read as something
else.

Run the tests in release. Several of them assert the audio chain keeps ahead of
real time, and a debug build misses by enough that the numbers mean nothing.

## Examples worth knowing

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

# Listen live without the GUI
cargo run --release -p rtlsdr --example listen -- 95.8
cargo run --release -p rtlsdr --example listen -- 446.0 nfm
```

Span times under `--soak` are wall clock, so `rf_read` sitting near 100% means
the radio thread is idle waiting on USB, which is exactly what it should be
doing.

## Reference implementations

Decoders worth reading before writing our own, all MIT and all pure Rust, so
anything borrowed from them can be relicensed into this one:

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

## Licensing rationale

GPL-3.0-or-later. This is where the rest of the band already sits: rtl_433 is
GPLv2 or later, GNU Radio and SDRangel are GPLv3, and the Flipper firmware is
GPLv3. A receiver whose whole value is knowing how a hundred protocols are
framed is built out of other people's reverse engineering, and returning it on
the same terms is the arrangement that produced the knowledge in the first
place.

It is also the correct licence rather than merely the sociable one. The
fixed-code gate remotes in `decode::protocols::keyfob` are ports of
Momentum-Firmware's `lib/subghz/protocols`, which is GPLv3, so the code was
already carrying that obligation while the manifest claimed MIT or Apache.

Everything this links against is compatible: the Rust dependencies are all MIT
or Apache-2.0, and librtlsdr is GPLv2 *or later*, which GPLv3 satisfies. Test
data fetched from rtl_433 or the Flipper firmware is neither modified nor
redistributed here.
