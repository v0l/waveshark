# Working on WaveShark

The code is the description of the receiver. There is no second one: start at
`crates/nodes/src/protocol.rs` for the decoder registry and
`crates/app/src/chain.rs` for the graph the receiver builds. What a protocol
reads is in its own module and in the tests beside it. The one page outside
the code is [`docs/references.md`](docs/references.md), which holds the terms
each data publisher requires; read it before adding a data source. A
measurement or a trap that explains a shape goes in a comment next to that
code.

## A protocol is layers, and the layers are shared

A new protocol is mostly other protocols' parts. Put each piece at the layer
it belongs to, named for the waveform or the code rather than for the device,
and a later protocol gets it for nothing.

- **`crates/dsp`, one module per waveform.** `afsk`, `ask`, `c4fm`, `fsk`,
  `fourlevel`, `msk`, `gmsk` (in `m17/`), `lora`, `wifi`. A module here is
  named after the modulation and parameterised by rate, baud and deviation. It
  knows nothing about the device: `dsp::msk` reads any MSK, and ACARS is one
  configuration of it.
- **Framing and slicing are their own modules**, above the waveform and below
  the protocol: `dsp::hdlc`, `dsp::slice`, `decode::slicer`, `decode::framing`,
  `decode::whiten`.
- **Codes and checks live in `decode::bits`**: CRCs, LFSR digests, parity,
  reflection, Golay, BCH, Reed-Solomon. A new polynomial goes there with a
  test, not into the decoder that needed it first.
- **`crates/decode`, one module per protocol payload.** Bytes in, fields out,
  no DSP and no graph.
- **`crates/nodes`, one `*_nodes.rs` per protocol**, which wires a channel to
  a waveform to a payload and is the only layer that knows about the graph.

The test for whether a piece is at the right layer: could a second protocol
using the same modulation, the same framing or the same polynomial call it
without touching it? If not, it is in the wrong file.

## Build

`tea` is TETRA decryption, `ambe` is DMR speech through `crates/mbe`. Both
default on, so `cargo test` at the root builds them; `crates/mbe` is not a
default workspace member and `cargo test -p mbe` must be asked for by name.
Releases build `--no-default-features --features limesdr,stt,mcp`. `mcp` is
the agent server behind `--mcp-listen`, which listens only when given an
address.

## Everything the receiver does is a node

If it processes, routes, records, mixes or decides, it is a node in the flow
graph; anything else cannot be seen, tapped, parameterised, saved in a patch
or moved by an operator, and the chain view is then wrong.

- No processing in the radio loop. `crates/app/src/radio.rs` moves blocks and
  commands.
- Transmit is drawn too: `derived_patch` builds `tx_clock`, the source, the
  modulator and `radio_tx` whether or not a key is down.
- No state only one hard-coded stage can produce. Publish on a port kind every
  front end can use and read it off the whole graph, as `Receiver::voices`
  reads `PortKind::Voice`, rather than reaching into `self.m17`.
- No behaviour keyed on a protocol name where a capability will do.
- A composite node reports its inner graphs through `Node::subgraphs`.
- The audio path is `crates/app/src/mix/`, one node per job and every level
  on the node that applies it: a `fader` per input, `calls` for the
  subscriptions, `heard` for the tap, `audio_bus` for the sum, `speaker` for
  the master. Audio crosses the bus as labelled `Voice`, never bare `Real`,
  so the bus can say what it is playing. Nothing about audio lives in the
  plan, the radio loop or a numbered bus setting.

Reading state back by downcasting is fine, and is how the spectrum, the
recorder and the capture are read.

## A protocol is one `impl Protocol`

`crates/nodes/src/protocol.rs` declares the trait; each `*_nodes.rs` has the
one implementation, registered in `protocol::all()`. It says where the
protocol can be, what stream it reads, how sticky a channel is, the chain of
stages that reads it, and which packets repeat. The auto node, the scanner
table, the mode menu, the spectrum markers and the chain labels all ask the
registry.

Adding a protocol is a node, an `impl Protocol`, and a line in
`protocol::all()`. A `match` on a protocol name in `crates/nodes/src/auto/`,
`chain.rs` or `scanners.rs` is a question that belongs on the trait. The test
beside the registry builds every chain and checks the declared outputs against
what it negotiates.

A decoder too dear to run always may wait for the classifier
(`Shape::families`) and be built late from the ring; only LoRa does. A decoder
needing more than the stream it was given asks with `pipeline::Request`
(claim, channel, reshape, release, retune) rather than reaching for the
detector.

## A known set of values is an enum

Parse a string once, at the edge, into an enum, and match on that afterwards.

- The type lives beside its one `parse` or `FromStr`.
- Keep the original string beside the enum for display; decide from the enum.
- An unknown value is a variant (`Other`, `Unknown`), not a fallback string.
- Avoid `_ =>` where the set is closed, so a new variant fails the build.
- Booleans that cannot all be true are one enum.

The exception is a genuinely open identifier: a registry protocol id, a call
sign.

## Words on the screen go through `theme::Line`

Every caption, reading, sentence and label in a pane is a `theme::Line`
(`crates/app/src/theme.rs`), drawn with `show`, or `wrapped` for prose and text
off the air. No `egui::Label`, no `RichText`, no `ui.label` in a pane: `Line`
paints a row as one galley on a fixed baseline, which a `Label` cannot do.

The face is a meaning: `legend` a silkscreened caption, `value` a reading,
`set` an operator's choice, `heard` what the radio heard, `words` text off the
air, `note` a sentence for a person.

Helpers built on `Line` live in `crates/app/src/ui/widgets.rs` (`hint`, `cell`,
`row`, `card`); a new one belongs there. Painted text is the exception: a table
cell goes through `widgets::cell`, and the spectrum's axis labels are painter
calls because they are part of a plot. `burst.rs` and `chain_pane.rs` still
call `ui.label`; convert what you touch and add no more.

## The design language is a chassis

The screen is a receiver's front panel, and every colour and face means
one thing (`crates/app/src/theme.rs`): `CHASSIS` is the case, `PANEL` a card
standing proud of it, `WELL` a recess (a readout window, a text field),
`ETCH` an edge or a rule, `LEGEND` a silkscreened caption, `VALUE` a
reading, `READOUT` amber what the operator set, `TRACE` cyan what the radio
heard, `OK` and `FAULT` a lamp. Do not introduce a colour or a face; pick the
meaning.

A modal or a settings pane is a column of cards, and nothing else:

- `widgets::section(ui, legend, note, body)`: the legend and a one-line
  purpose in the header, rows in the body. A list of things (scanners,
  datasets, memories, feeds) is one `widgets::card` per thing with its name
  in the header, its actions on the right of the header (TUNE, REMOVE,
  REFRESH), and its state on the rail: amber for what the operator set, cyan
  for what is running or heard, red for a fault, nothing for a plain card.
- A row is `row_help(ui, legend, help, control)`: a legend of at most ten
  characters in the left column, the control filling the right, the
  explanation behind the `?`. Never a paragraph of prose between controls;
  `hint` is for one line under a picker at most.
- Text goes in `field`, `secret` or `prose`; a field with anything after it
  on the row is `field_then(reserve, after)`, because a field fills the row
  and a button added afterwards pushes the card wider on every frame. Never a
  bare `egui::TextEdit`, `ui.checkbox` or `ui.separator` in a modal: a switch
  is `switch`, a closed list is `choice`, a card edge is the separator.
- A number the operator reads is `reading`. Whether the card will work as it
  is set is a `lamp` at the foot of the card, green with what it resolved to
  or red with why not, computed live from the fields above it.
- The modal ends in `footer`, CLOSE outermost right and the action beside
  it. Buttons are legends: uppercase verbs.
- Widths are 520, or 560 for a list. A scroll area inside a modal takes
  `set_max_width` of what was available outside it.

Check a modal by looking at it: `waveshark --settings agent --shot
/tmp/agent.png --shot-after 5` writes the window, and the name can be any
dialog (`radio`, `spectrum`, `waterfall`, `log`, `scanners`, `memory`,
`data`, `app`).

## Every HTTP request goes out under the same name

`crates/httpc` holds the user agent and builds every client, async or
blocking. No `reqwest::Client::builder()` anywhere else. `crates/datasets`
fetches over `ureq` and takes the same string from `httpc::USER_AGENT`.

## Every packet carries what it was heard at

Every packet reaching the bus has a finite `rssi_dbfs`, a finite `snr_db`, and
the samples it was read from in `iq`. A `f32::NAN` in a level is a bug.

The front end measures, because it holds the samples the frame came from: a
level taken from the span is a level of the band, not of the transmitter. Where
a demodulator already measures, as Mode S does off its preamble and BLE off the
floor either side of a burst, carry that; otherwise `nodes::FrameMeter`
measures the channel and keeps a short ring so the frame gets its samples back.
This breaks at port boundaries rather than in decoders, so when adding a
payload kind, bus, feed or sink, ask what happens to the level and the samples
crossing it.

## The graph is the same graph in both modes

Manual mode is a lock on editing. `derived_patch` draws the graph from the plan
on every rebuild: the head with its DC blocker and zoom, the spectrum, the
recorder's ring, the raw capture, the scanner table's front ends, the listening
channels, the transmit chain, the feeds, the protocol decoder, the tracker and
the buses. What the operator changed is kept apart as `patch::Edits`, applied
on top, then `sync_audio` puts the strip's stages in step. Both halves apply
whether or not the graph is unlocked.

The interface never sends a whole graph. It holds the running patch and the
base beneath it (`Status::patch` publishes both) and sends
`Edits::diff(edited, base)`, which is also what `~/.config/waveshark/edits`
holds.

Anything the receiver does regardless of mode goes in `derived_patch` or
`sync_audio`. Three consequences:

- A derived stage's settings are reapplied every rebuild, so a hand-set value
  survives only as an edit; `Receiver::set_node_param` writes it into the
  running patch.
- A setting the strip owns (squelch, gain control) is a plan value, not an
  edit, pulled back into the plan and published as `Status::levels`. A level
  or a mute is never a plan value: it is a setting on a `fader`, `calls` or
  `speaker` stage, set with `Cmd::StageParam` by its derived id and kept as
  an edit. `chain::operator_owns` draws that line.
- A node's identity across rebuilds is its derived id, keyed by mode and rate
  and never by offset; the mixer's shift is a setting.

## A test says how much was read

Assert the number. `assert!(decoded >= 1)` and `assert!(!rows.is_empty())` pass
when fourteen of fifteen packets were thrown away. Pin how many packets
decoded, how many rows came out, how many carried a measurement, which
callsigns and ids, and which channels they were read on. Where another
implementation read the same file, assert what it said and put the provenance
in the test's doc comment.

An optimisation may not change any of those numbers. Run the affected tests
before and after and expect the same numbers, not merely a pass. Where the
signal will not allow exactness, pin a floor and a ceiling with a comment
saying which. A skip for an absent fixture prints the name and
`run testdata/fetch.sh`.

Tests that time the audio chain check `cfg!(debug_assertions)` and skip, so run
those in release. The rest need no release build: the dev profile compiles the
signal path at `opt-level = 3`.

## The changelog is for the person running it

`CHANGELOG.md` follows [Keep a Changelog](https://keepachangelog.com). A change
somebody running the receiver would notice gets a line under `[Unreleased]` in
the same commit, under `Added`, `Changed`, `Fixed` or `Removed`. A refactor or
a test does not. Write it for the person: "Bluetooth LE advertising", not "add
BleNode".

**One line, under about fifteen words, and no how.** The reader is scanning a
list to find out whether to upgrade, not reading an account of the work. Name
the protocol, the pane or the fault and stop. Never list the parts of a fix,
never explain the mechanism, never write a second sentence justifying the
first. The reasoning belongs in a code comment where it is findable, and the
measurements in the test that pins them.

Good: `SSTV pictures: Martin, Scottie and Robot modes, on the video pane.`
Good: `A green stripe down the right of a Robot picture.`
Bad: anything with a colon followed by three clauses, or the word "which".

A release is `tools/changelog.sh release X.Y.Z`, the version in `Cargo.toml`,
one commit, and the tag `vX.Y.Z` pushed. The release workflow takes its notes
from that section and refuses a tag with none; CI checks that `[Unreleased]`
exists and that a tagged version has its own section.

## Commits

Count what is going out with `git log --oneline @{u}..HEAD`. More than ten
commits is a branch to squash into the few a person would want to bisect, each
building and passing its tests alone. Do not rewrite a commit somebody else has
pulled, or a merge recording two lines of work meeting.

## Adding a capture to the test corpus

Recorded IQ lives on nostr.download, is fetched by `testdata/fetch.sh` and is
never committed; `testdata/*.cu8` and friends are gitignored. The manifest
entry carries the hash. A capture earns its place by failing something that
synthesised signals do not.

1. **Record and trim.** Capture with the "Capture the raw span" switch or
   `--capture-iq`, into `~/.local/share/waveshark/captures`. Cut to the
   shortest span holding the evidence, keeping noise around it for the
   detector's floor, and cut the zeros a radio delivers while its stream
   starts. `cargo run --release -p sources --example iq_clipper` cuts:
   `--bursts` keeps the transmissions and drops the silence, `--seconds N`
   with `--skip N` keeps a window, and `--center-hz` with `--rate` mixes and
   resamples, which a wideband recording needs before it can be cut on power
   at all. A tuned output is one channel rather than the band, so name it for
   what it holds and say so in the manifest.

   Margin matters: the BLE capture reads five of eleven packets as MSK with
   4 ms either side and six of eleven with 2 ms, because the floor is a
   percentile of the file. Every join is a discontinuity, and cutting the
   1090 MHz capture close manufactured a Mode S frame out of a splice, so that
   one is uploaded whole. Run the tests that use the capture before and after
   and expect the same numbers. Whatever the test does not look at is published
   anyway, so where an identity is in the recording, say so in the description.

2. **Name it** `<what>_<centre>M_<rate>k.<format>`, for example
   `m17_openrtx_434.02M_2400k.cu8`. `sources::parse_filename` reads the centre
   and rate from it, and the centre is the tuner's, not the signal's. Avoid
   bare digit tokens: a sequence number was once read as a rate of 1 Hz.

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

   The response carries the sha256; the URL is that hash with the compression
   suffix, and the manifest hash is of the compressed upload.

4. **Add the manifest entry** to `testdata/fixtures.toml`, or
   `testdata/offair.toml` for a capture labelled by what it demonstrates rather
   than by an independent decode. Both take `name`, `sha256`, `url`,
   `compression`, `center_hz`, `rate_sps`, `format` and a description of what
   the capture is evidence of and how that was established. A `fixtures.toml`
   entry adds a `[capture.expect]` block of asserted values; an `offair.toml`
   entry adds `family` and `receiver` and asserts nothing beyond the
   classifier's verdict.

5. **Verify the round trip.** Delete the local file, run `./testdata/fetch.sh`,
   and check the hash of what comes back.

6. **Write the test against the receiver** where the capture is evidence about
   the receiver: `crates/app/src/radio.rs` has `replay_receiver` and
   `replay_blocks`, which run the live radio's path including the scanner
   table. Skip cleanly when the fixture is absent.
