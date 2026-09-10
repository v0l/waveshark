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
`satellites`, `chain`, `scanners`, `memory`, `list_protocols`, `screenshot`.

Control: `start_receiver`, `stop_receiver`, `select_device`, `tune`,
`set_span`, `set_gain`, `set_toggle`, `set_choice`, `set_ppm`, `set_location`,
`add_channel`, `set_channel`, `remove_channel`, `listen`, `set_volume`,
`set_decode`, `set_dc_block`, `set_view`, `set_record`, `set_capture_iq`,
`set_packet_log`, `set_node_param`.

`set_node_param` is the wide one: it reaches any parameter of any node in the
running graph, by the id `chain` reports, which is the same route the chain
view uses. What it cannot do is draw a graph, since an edit is a patch rather
than a setting.

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
