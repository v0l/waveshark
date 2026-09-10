# Driving the receiver from an agent

`waveshark --mcp-listen 8931` serves the [Model Context
Protocol](https://modelcontextprotocol.io) over streamable HTTP at
`http://127.0.0.1:8931/mcp`. A bare number means loopback; write
`0.0.0.0:8931` if you mean to offer the whole receiver to a network, and
understand that anything reaching that port can retune the radio and start
writing files.

Nothing listens unless the flag is given. The feature is `mcp`, on by default
and in the published binaries.

## It is the same receiver

The tools drive the receiver a person is looking at rather than a second one
behind the same protocol. An agent that tunes moves the dial on screen; an
agent that opens a channel adds a strip; a screenshot shows what it just did.
Two receivers would mean two graphs, two USB claims, and an interface that
cannot be asked what the agent has been up to.

How that works is a queue rather than a lock. `crates/app/src/agent/` speaks
MCP on a tokio task and puts an `Action` on a channel; the interface drains it
at the top of the next frame in `crates/app/src/ui/agent.rs`, applies it to
its own state exactly as a click would, and answers with what it then holds.
So a tool call takes effect in one frame, and the commands it produced leave
for the radio thread with the rest at the end of that frame. A call that
arrives while the window is idle wakes it, since egui does not know about
sockets.

The one exception is `screenshot`: egui hands the image back on a later frame,
so the reply is held until it arrives.

## What it can reach

Everything the interface can, except transmitting. There is no tool that keys
a channel, sets transmit gain or arms a transmit chain. That is deliberate:
keying is legally loaded, the interlock is a person holding a button, and an
agent with a bug should not be able to put a carrier on the air. If you want
that, it needs its own flag and its own thinking.

Observation: `status`, `list_devices`, `spectrum`, `list_channels`, `packets`,
`calls`, `transcript`, `messages`, `links`, `control_links`, `tracks`,
`satellites`, `chain`, `patch`, `list_stage_kinds`, `scanners`, `memory`,
`list_protocols`, `screenshot`.

Control: `start_receiver`, `stop_receiver`, `select_device`, `tune`,
`set_span`, `set_gain`, `set_toggle`, `set_choice`, `set_ppm`, `set_location`,
`add_channel`, `set_channel`, `remove_channel`, `listen`, `set_volume`,
`set_decode`, `set_dc_block`, `set_view`, `set_record`, `set_capture_iq`,
`set_packet_log`, `set_node_param`.

Graph: `add_stage`, `remove_stage`, `connect`, `disconnect`, `undo_edit`,
`redo_edit`, `reset_graph`, `set_manual`.

`set_node_param` reaches any parameter of any node in the running graph, by
the id `chain` reports, which is the same route the chain view uses.

## Drawing the graph

The graph is the receiver, so an agent that can only change settings can only
change half of it. `patch` is the graph as something to edit: every stage with
the id an edit names it by, whether the receiver derived it or somebody added
it, the wires, and what has been changed so far. `list_stage_kinds` is the
node registry, which is what can be placed.

```
add_stage   {kind: "mixer"}                    -> stage 1
add_stage   {kind: "fm_demod"}                 -> stage 2
connect     {source: {from: "span"}, to_stage: 1, to_port: 0}
connect     {source: {from: "stage", id: 1, port: 0}, to_stage: 2, to_port: 0}
```

Three things to know about ids. A patch stage id is not the node id `chain`
reports: nodes are positions in the built graph and are renumbered on every
rebuild, so `chain` carries the patch id beside each node as `stage`, and
`patch` carries the node id beside each stage as `node`. Ids the operator's
own stages take count up from one; the enormous ones are stages the receiver
derived, and those can be deleted and rewired like any other. And
`patch.head` and `patch.span` name the receiver's own markers, for pointing
the spectrum or the recorder at a stage instead of at the head of the chain.

What is kept is the difference from the graph the receiver draws for itself,
not the drawing, so an edit survives a retune, a zoom and a new front end.
`reset_graph` throws the difference away; `undo_edit` and `redo_edit` are the
same history the chain view's buttons walk.

Every edit is answered by the rebuild that took it, not by the frame that
asked for it. The receiver refuses a graph it cannot build and hands the
previous one back, so a tool that answered immediately would report success
for a graph that never ran. A refused edit comes back as an error carrying the
receiver's own words, for example `the patch was refused: graph has a cycle
involving: Mixer, FM discriminator`. If nothing has rebuilt within two and a
half seconds the answer says so rather than waiting longer, and with no radio
running the edit is kept and applied when one starts.

`set_manual` is not needed for any of this. Edits apply whether or not the
graph is unlocked; the lock only decides whether a hand can drag and wire in
the window.

## Units, and what an answer means

Frequencies go in as MHz because that is how a person writes one, and come
back as hertz because that is what arithmetic wants. Widths go in as kHz.
Levels are dBFS and mean nothing on their own: what says whether a signal is
there is `above_floor_db` in `spectrum`, or `snr_db` on a packet.

Every packet row carries the level and the signal to noise it was heard at,
because a row that does not is half a row. A reading of `null` there is a bug
rather than a quiet channel.

Two refusals are worth knowing, since both look like a decoder that heard
nothing if they are guessed at instead:

- A channel outside the span is refused with the span in the message. Tune
  there first, or widen the span.
- A mode nothing answers to is refused with the whole list of demodulators and
  front ends. `list_protocols` is where that list comes from.

## Adding a tool

A variant on `Action` in `crates/app/src/agent/mod.rs`, its parameters as a
`schemars` struct in the `args` module beside it, an arm in `agent_apply` in
`crates/app/src/ui/agent.rs`, and a method in
`crates/app/src/agent/tools.rs` carrying the sentence an agent reads before
choosing it. Four places, all named the same thing.

Two rules the rest of the tree already has and this is not exempt from. A
closed set of values is an enum parsed once at the edge, not a string matched
on later: the view name is an enum, and the mode string is parsed by
`parse_mode` against the protocol registry rather than compared anywhere else.
And a readout comes from the same field the pane draws, so a number an agent
reads cannot disagree with the screen.
