# Working on WaveShark

Read [`docs/design.md`](docs/design.md) first. It has the layout, the
measurements behind the current shape, and the mistakes that produced it. Then
[`docs/protocols.md`](docs/protocols.md) for what is decoded and what is not,
[`docs/views.md`](docs/views.md) before adding a pane, and
[`docs/web.md`](docs/web.md) only knowing it is a plan and not a status report.
What follows is the two rules that are easiest to break without noticing, and
the procedure for adding a capture to the test corpus.

Two cargo features change what builds: `tea` is TETRA decryption and key
recovery, and `ambe` is DMR speech through `crates/mbe`. Both are on by
default, so a plain `cargo test` at the root builds them and `crates/mbe` is
compiled through the `ambe` feature, though it is still not a default
workspace member and `cargo test -p mbe` has to be asked for by name. What the
release workflow publishes is built with `--no-default-features --features
limesdr,stt`, because a cipher and a patent-encumbered vocoder are things a
person compiles for themselves rather than things this project ships.

## Everything the receiver does is in the graph

The flow graph is not an implementation detail of the signal path. It is the
description of what the receiver is doing, and the chain view is that
description drawn. Anything the receiver does that is not a node is invisible
there: it cannot be seen, tapped, parameterised, saved in a patch, or moved by
an operator, and the drawing on the screen is then a lie by omission.

So: **if it processes, routes, records, mixes or decides, it is a node.** The
spectrum is a node. The recorder's ring is a node. The DC blocker, the packet
bus, the packet log, the raw IQ capture, every front end the scanner table
places, and every decoder the auto node builds for a source it found are all
nodes. A helper that runs once per block from the radio thread is not a design
choice, it is a node somebody has not written yet.

What this rules out in practice:

- No processing in the radio loop. `crates/app/src/radio.rs` moves blocks and
  commands. If you find yourself filtering, mixing or deciding in it, that
  belongs in a node the graph holds.
- The same applies to transmitting. `derived_patch` draws four stages,
  `tx_clock`, the source, the modulator and `radio_tx`, whether or not a key is
  down, so the chain can be looked at before it is used.
- No state that only one hard-coded stage can produce. Reaching into a named
  stage by field (`self.m17`, `self.record`) works until the same front end
  exists somewhere else. Live speech was read from the one M17 stage the
  scanner table places, so every M17 transmission the auto node found for
  itself decoded, logged, and played back as silence. The fix was a port kind
  every front end can publish on: `Receiver::voices` reads `PortKind::Voice`
  off the whole graph rather than off a named stage.
- No behaviour keyed on a protocol name where a capability will do. Ask nodes
  what they can do; do not keep a list here of which ones can.
- A composite node owns an inner graph and must say so through
  `Node::subgraph`, or the work it does disappears from the view. The auto node
  is a node holding a graph per open source; a bank is a node holding hundreds
  of channels.

Reading state back by downcasting is fine and is how the spectrum, the
recorder and the capture are read. What is not fine is the work itself
happening outside.

The speaker is on the graph too. `PortKind::Voice` carries decoded speech
with the call it belongs to, front ends publish it on a port of their own,
every listening channel's chain ends in real audio, and `AudioBusNode`
(`crates/app/src/audiobus.rs`) is the node all of them are wired into: one
input per strip, a level and a mute on each, the subscriptions that decide
which calls are heard, the master, the clip, and one stereo output the radio
thread hands to the device. The mix used to be a loop in the radio thread and
the faders were fields on a channel list, so a demodulator drawn by hand had
nothing to be wired to. If you find yourself summing audio anywhere but on
the bus, or keeping a level anywhere but as one of its parameters, you are
adding that back.

## A protocol is one `impl Protocol`, and nothing else

What the receiver knows about a protocol lives in one place:
`crates/nodes/src/protocol.rs` declares the trait, and each `*_nodes.rs` has
the one implementation for its protocol, registered in `protocol::all()`.
Where it can be, what stream it reads, how sticky a channel it has read on
is, the chain of stages that reads it, and which packets repeat. The auto
node, the scanner table, the strip's mode menu, the spectrum markers and the
chain view labels all ask that registry and keep no list of their own.

So: **adding a protocol is a node, an `impl Protocol`, and a line in
`protocol::all()`.** If you find yourself matching on a protocol's name in
`crates/nodes/src/auto/`, in `chain.rs` or in `scanners.rs`, the question you
are answering belongs on the trait. The test beside the registry builds every
protocol's chain and checks its declared outputs against what the chain
negotiates, so a wrong declaration fails there rather than as a wire drawn
to the wrong port.

Two things the trait deliberately does not do. A decoder that is dear to run
may wait for the burst classifier's verdict (`Shape::families`) and be built
late from the ring; only LoRa does, because the classifier is sure of a
chirp and was measured to name an off-air M17 handheld `Unknown` for its
whole transmission. And a decoder that needs more than the stream it was
given asks for it (`pipeline::Request`: a claim, a channel beside it, a
reshape, a release, a retune) rather than reaching for the detector; the
auto node answers what it can and the receiver logs the rest.

## Every HTTP request goes out under the same name

`crates/httpc` holds the user agent and builds every client, asynchronous or
blocking. **No `reqwest::Client::builder()` anywhere else.** The string is how
the far end sees this program: a tile server that blocks an unnamed client
blocks the map, and an operator whose upload was refused needs the log at the
other end to say what sent it. There were four copies of the same constant and
one client, in `meshnode`, that set no agent at all, which is the state this
rule exists to prevent returning to. `crates/datasets` still fetches over
`ureq` and takes the same string from `httpc::USER_AGENT`.

## The changelog is for the person running it

`CHANGELOG.md` follows [Keep a Changelog](https://keepachangelog.com). A
change somebody running the receiver would notice, a protocol it now reads,
a pane, a setting, a fix for something that was wrong on screen, gets a
line under `[Unreleased]` in the same commit, under `Added`, `Changed`,
`Fixed` or `Removed`. A refactor, a test, a doc edit does not. Write the
line for the person, not the code: "Bluetooth LE advertising", not
"add BleNode".

A release is `tools/changelog.sh release X.Y.Z` (which dates the section
and fixes the compare links), the version in `Cargo.toml`, one commit, and
the tag `vX.Y.Z` pushed. The release workflow takes its notes from that
section and refuses a tag that has none; CI checks the file has an
`[Unreleased]` section and that a tagged version has its own.

## Every packet carries what it was heard at

A row in the packet list is evidence, and evidence that does not say how
strong it was is half a row. **Every packet reaching the bus has a finite
`rssi_dbfs`, a finite `snr_db`, and the samples it was read from in `iq`.**
Not most of them, not the ones whose front end happens to measure: all of
them. Without those three a row cannot be sorted by strength, a fade cannot
be told from a decoder that broke, a weak decode cannot be judged, and there
is nothing to look at when the bytes are wrong.

The front end measures, because the front end is the only thing holding the
samples the frame came from. A level taken later is a level of something
else: a 16 kHz packet channel inside 2.4 MS/s of band is 0.7% of the power,
so a reading from the span says what the band was doing, not what the
transmitter was. Where a demodulator already measures, as Mode S does off its
preamble and BLE does off the floor either side of the burst, that number is
the one to carry; where it does not, `nodes::FrameMeter` measures the
channel and keeps a short ring so the frame gets its own samples back.

This has been rediscovered more than once because it is easy to break at a
port boundary rather than in a decoder. `Payload::Frames` carried
`Vec<Vec<u8>>` for a year, which quietly threw away the measurements of every
front end that produced frames instead of pulses; the fix was to make the
port carry `common::Frame`. If you add a payload kind, a bus, a feed or a
sink, ask what happens to the level and the samples as they cross it.
A `f32::NAN` in a level is a bug, not a value.

## The graph is the same graph in both modes

Manual mode is a lock on editing and nothing else. The receiver draws its
graph from the plan on every rebuild (`derived_patch`: the head with its DC
blocker and zoom, the spectrum, the recorder's ring, the raw capture, the front
ends the scanner table puts on the span, the listening channels, the transmit
chain, the feeds, the protocol decoder and the tracker, and the buses), and
what the operator changed is kept apart from it as
`patch::Edits`: stages added, derived stages removed, wires drawn or moved,
settings overridden. Every rebuild is derived graph, then edits on top, then
`sync_audio` to put the strip's stages in step with the result. The edits
apply whether or not the graph is unlocked, and the derived part follows the
dial, the zoom, the scanner table and the strip whether or not it is.

The interface never sends a whole graph. It holds the running patch and the
base the receiver drew underneath it (`Status::patch` publishes both), edits
the running one, and sends `Edits::diff(edited, base)`; that is also what is
saved to `~/.config/waveshark/edits`. A saved whole drawing was the bug this
replaces: taking the graph over swapped in a graph derived for another
tuning, another zoom and another day's front ends, so manual mode behaved
like a different receiver and every pan rebuilt around a stale head.

When you add something the receiver does regardless of mode, put it in
`derived_patch` or `sync_audio`; it will be there in both modes. Two
consequences worth knowing. The build reapplies a derived stage's settings on
every rebuild, so a value set on one by hand survives only as an edit:
`Receiver::set_node_param` writes it into the running patch and the radio
thread reads the edits back off it. A setting the strip owns (a channel's
squelch or gain control, a bus level) is not an edit but a plan value, pulled
back into the plan and published as `Status::levels` so the strip follows;
`Edits::own_settings` is where that line is drawn. And a node's identity
across rebuilds is its derived id, so a channel's stages are keyed by mode and
rate and not by offset; the mixer's shift is a setting, and keying on it meant
every channel was rebuilt, and forgot its station, whenever the dial moved
under it.

## Adding a capture to the test corpus

Recorded IQ lives on nostr.download and is fetched by `testdata/fetch.sh`,
never committed: it is near-incompressible and would bloat history permanently
for data that never changes. `testdata/*.cu8` and friends are gitignored. What
is committed is the manifest entry, and it carries the hash, so a capture that
silently changed cannot invalidate an expectation quietly.

A capture earns its place by failing something. Synthesised signals share every
assumption the code makes and pass; the M17 fixture is here because three
separate faults threw a real transmission away and every one of them was
invisible on synthesised M17.

1. **Record and trim.** Capture with the "Capture the raw span" switch in the
   radio settings, or `--capture-iq`, into
   `~/.local/share/waveshark/captures`. Cut it to the shortest span that
   contains the evidence, keeping enough noise around it for the detector's
   floor. A radio delivers zeros for the first seconds while its stream starts:
   cut those, or the whole band appears to switch on at once and every source
   opens in the same frame.

   `cargo run --release -p sources --example cut` does both cuts. With
   `--bursts` it keeps the samples the transmissions were in and drops the
   silence, which on a packet capture is most of the file: the BLE fixture is
   a seventeenth of what was recorded and every packet still decodes. With
   `--seconds N` (and `--skip N`) it keeps a window, which is all a continuous
   signal allows. It copies bytes straight out of the input, so the output is
   the same recording at the same scale.

   Two things to check before uploading a cut. The margin either side of a
   burst is not decoration: a detector takes its noise floor from a percentile
   of the file, so cutting too close moves the floor and the capture stops
   being evidence of what it was evidence of. The BLE capture reads five of
   eleven packets as MSK with 4 ms of margin, as it did uncut, and six of
   eleven with 2 ms. And every join is a discontinuity: cutting the 1090 MHz
   capture close manufactured a Mode S frame out of a splice, which is why
   that one is uploaded whole. Run the tests that use the capture before and
   after and expect the same numbers, not merely a pass.

   Cut for what is in the file as well as for its size. A recording is of a
   real band at a real place and time, and whatever the test does not look at
   is published anyway. Where the identity is the evidence, say so in the
   description rather than pretending otherwise.

2. **Name it so the file carries its own metadata.**
   `<what>_<centre>M_<rate>k.<format>`, for example
   `m17_openrtx_434.02M_2400k.cu8`. `sources::parse_filename` reads the centre
   and rate out of it; guessing a sample rate wrong rescales every pulse width
   and breaks every decoder downstream. The centre is the tuner's, not the
   signal's. Avoid bare digit tokens: a sequence number was once read as a
   sample rate of 1 Hz.

3. **Compress and upload.**

   ```sh
   cd testdata
   xz -9 -T0 -k m17_openrtx_434.02M_2400k.cu8
   AUTH=$(nak event --kind 24242 -t t=upload \
       -t expiration=$(($(date +%s) + 600)) --sec $(cat ~/.nostr/route96-admin.nsec))
   curl -X PUT -H "Authorization: Nostr $(echo "$AUTH" | base64 -w0)" \
       -H "Content-Type: application/x-xz" \
       --data-binary @m17_openrtx_434.02M_2400k.cu8.xz \
       https://nostr.download/upload
   ```

   The response carries the sha256, and the URL is that hash with the
   compression suffix. The manifest hash is of the compressed upload.

4. **Add the manifest entry** to `testdata/fixtures.toml`, or
   `testdata/offair.toml` for a capture labelled by what it demonstrates rather
   than by an independent decode. Both take `name`, `sha256`, `url`,
   `compression`, `center_hz`, `rate_sps`, `format` and a description saying
   what the capture is evidence of and how that was established. A
   `fixtures.toml` entry then takes a `[capture.expect]` block with the values
   a test asserts; an `offair.toml` entry takes `family`, which is only ever
   what the capture demonstrates, and `receiver`, and asserts nothing beyond
   the classifier's verdict. Write the description for somebody who has to
   decide, two years from now, whether a failing assertion means the code broke
   or the expectation was wrong.

5. **Verify the round trip.** Delete the local file, run `./testdata/fetch.sh`,
   and check the hash of what comes back matches what you uploaded. A manifest
   entry nobody has fetched is a test that fails on every machine but yours.

6. **Write the test against the receiver, not against a node**, when the
   capture is evidence about the receiver: `crates/app/src/radio.rs` has
   `replay_receiver` and `replay_blocks`, which run the same path as the live
   radio including the scanner table. Skip cleanly when the fixture is absent,
   printing the name and `run testdata/fetch.sh`, so a fresh clone still
   passes without network access. Assert what an independent implementation or
   the transmission itself says, such as a callsign or a CRC, rather than what
   this code currently produces.

Run the tests that time the audio chain in release. They check
`cfg!(debug_assertions)` and return with a printed note, so a debug run passes
by skipping them rather than by measuring anything. The rest do not need it: the workspace
already builds the signal path and the test binaries at `opt-level = 3` in the
dev profile.
