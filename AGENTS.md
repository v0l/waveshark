# Working on WaveShark

Read [`docs/design.md`](docs/design.md) first. It has the layout, the
measurements behind the current shape, and the mistakes that produced it. What
follows is the two rules that are easiest to break without noticing.

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
- No state that only one hard-coded stage can produce. Reaching into a named
  stage by field (`self.m17`, `self.record`) works until the same front end
  exists somewhere else. Live speech was read from the one M17 stage the
  scanner table places, so every M17 transmission the auto node found for
  itself decoded, logged, and played back as silence. The fix was a method on
  `Node` that any node can answer, asked of all of them.
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

## The graph is the same graph in both modes

Manual mode is a lock on editing and nothing else. The receiver draws its
graph from the plan on every rebuild (`derived_patch`: the head, the spectrum,
the front ends the scanner table puts on the span, the listening channels,
the buses), and what the operator changed is kept apart from it as
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
separate thresholds threw a real transmission away and none of them was
visible on a generated one.

1. **Record and trim.** Capture with the raw IQ button, or `--capture-iq`, into
   `~/.local/share/waveshark/captures`. Cut it to the shortest span that
   contains the evidence, keeping enough noise around it for the detector's
   floor. A radio delivers zeros for the first seconds while its stream starts:
   cut those, or the whole band appears to switch on at once and every source
   opens in the same frame.

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
   than by an independent decode. Fill in `name`, `sha256`, `url`,
   `compression`, `center_hz`, `rate_sps`, `format`, a description saying what
   the capture is evidence of and how that was established, and a
   `[capture.expect]` block with the values a test asserts. Write the
   description for somebody who has to decide, two years from now, whether a
   failing assertion means the code broke or the expectation was wrong.

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

Run the corpus tests in release. Several assert the audio chain keeps ahead of
real time, and a debug build misses by enough that the numbers mean nothing.
