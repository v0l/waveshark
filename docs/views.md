# Views on the packet stream

The packet list is not a display. It is the place every decoded frame arrives,
which makes it the natural bus for everything that wants to show one: a map for
ADS-B and AIS, an image pane for weather satellites and SSTV, a chart for a
temperature sensor, a text pane for pager traffic. Each of those is a view over
the same stream, filtered differently, and none of them should need a private
path back to the demodulator that produced it.

This file describes the contract a view uses, what exists today, and what each
planned view needs from it.

## What a packet carries

`radio::DecodeRecord` is the unit on the bus. The parts a view cares about:

| Field | What it is | Who uses it |
|---|---|---|
| `at` | when the burst arrived, stamped at the start of the block that held it | every view, for ageing and ordering |
| `freq` | centre of the channel it arrived on | list, map (as a filter) |
| `model` | protocol name, or "unknown" | routing |
| `media_type` | what `bytes` holds | routing |
| `fields` | the decoder's own fields, structured | map, chart, text |
| `bytes` | the raw frame | hex dump, image pane |
| `rssi_dbfs`, `snr_db` | how it was received | list, and a map colouring tracks by signal |
| `crc` | integrity check result, `None` when the protocol has none | every view: an unverified position is not a position |

`fields` is the part that makes this work. A decoder produces
`decode::protocol::Report`, whose fields are `common::Value` (int, float, bool
or text), and those survive all the way to the record. A map reads
`field("lat")`; it does not parse `lat=51.5` out of a display string, and it
does not need to know which protocol produced the packet.

`media_type` is the routing key for payloads that are not fields:
`pipeline::event::media` already defines `BYTES`, `JSON`, `TEXT`, `JPEG` and
`PNG`, and `Decoded::matches_media` handles `image/*` style patterns. A view
claims what it can render.

## Today

- **Packet list**: every record, newest at the bottom, with a detail pane
  showing the selected packet's burst as the front end saw it, its
  envelope and instantaneous frequency against time, then its fields and a
  hex dump. The burst view is what an unknown device is worked out from, the
  way Universal Radio Hacker shows a burst beside its bits; the samples are
  kept for the newest rows only.
- **Messages**: every record carrying a `text`, `message` or `sms` field,
  newest first, each drawn as a header line and the words underneath at full
  width rather than clipped to a column. The recipient is read from
  `addressee`, `to`, `dst`, `destination`, `talkgroup` or `address`, so a
  pager's capcode and a TETRA talkgroup land in the same place. Identical
  words from the same sender inside two minutes are one message with a count,
  because a pager sends every page twice and TETRA retransmits until it is
  acknowledged. `crates/app/src/messages.rs`.
Both are views by this definition. Neither knows anything about a protocol.

## Planned views

### Map, for ADS-B, AIS and radiosondes

Needs `lat` and `lon` as floats, and takes `altitude_m`, `ground_speed_kt`,
`track_deg`, `callsign` and an identity field (`icao`, `mmsi`, `serial`) when
the protocol has them. The identity is what turns a stream of positions into
tracks, so it is the one field a protocol must supply to appear on a map at
all.

Retention differs from the list: a list keeps the last N packets, a map keeps
the last position per identity plus a trail, and drops an aircraft that has
been silent for a minute or two. That is the view's business, not the bus's.

ADS-B, AIS and APRS all exist now, and the map was generalised with the second
of them rather than before it, which was the right order. Written against
ADS-B alone the abstraction would have had one implementation, and the parts
that look general would have been indistinguishable from the parts that are
pure ADS-B. What the second protocol showed is where the seam actually is, and
the third fitted without moving it.

### Image pane, for APT, LRPT, SSTV and HRIT

These are not packets in the same sense: a line of an APT image is one strip of
a picture that takes fifteen minutes to arrive. Two options, and the second is
the right one:

1. Emit one packet per line and let the view assemble the image. Puts thousands
   of rows in the list for one picture.
2. Emit a packet per *image*, with `media_type` of `image/png` and the encoded
   picture in `bytes`, and have the decoder hold the partial image. The list
   then shows one row per picture and the pane renders it. Progress while a
   pass is in flight is a `Metric` event, not a packet.

Analogue television is deliberately out of scope: it is not a digital mode and
has no frame to log.

### Chart, for sensors

Any protocol with a numeric field and a stable identity: temperature, humidity,
tyre pressure, battery voltage. Reads `fields` and needs nothing else, which
makes it the cheapest of the three and the best first proof that the bus works.
The Fine Offset decoder already produces everything it needs.

### More into the message view

The message view exists and takes any decode with a text field, so FLEX, ACARS
and AIS safety messages join it by naming their fields the way POCSAG, M17,
TETRA and APRS already do. What it does not yet take is a payload that is text
without being a field: `media_type` of `text/plain` with the words in `bytes`
should be read as the message body.

## What the bus needs before it grows

The current implementation is a `Vec<Logged>` in the app, capped at 500
records, filtered by one flag. That is enough for two views and will not carry
five. Three things have to change, in this order, and only when a second real
view exists to force them:

1. **Retention per view.** A map wants the last position of every aircraft seen
   in the last ten minutes; the list wants the last 500 packets whatever they
   are. One capped vector cannot serve both. The store should keep packets by
   age and let each view take what it wants.

2. **Indexing by identity.** Every view except the list groups by device:
   aircraft, meter, sensor. Scanning 500 records per frame to rebuild those
   groups is affordable now and will not be with a busy band and a map open.

3. **Recording and replay.** Half done: `--record` writes the IQ of every
   burst and an `index.jsonl` describing it, and `--replay` runs captures back
   through the scanner. What is missing is replaying into the *bus* rather than
   into a printed list, which is what would let a view be tested without a
   radio.

### Map

Aircraft from ADS-B on 1090 MHz, vessels and navigation marks from AIS on
162 MHz, and vehicles and stations from APRS on 144.800, all on OpenStreetMap
tiles.

Three protocols, one tracker, and the differences between them are where the
design is. Identity is shared but is not a number: an ICAO address, an MMSI
and a callsign are three identity spaces, so a track is identified by the pair
of protocol and value and nothing can collide. Position reassembly is not
shared at all; only ADS-B has compact position reporting, so the CPR machinery
hangs off that path and neither of the others touches it. Ageing and
plausibility are shared but not constant: an aircraft silent for a minute is
gone, a Class B vessel reports every thirty seconds, and an APRS station
beacons every few minutes.

A kind decides how a thing is drawn and how long it is remembered. AIS says
which it is by message type; APRS says it with a symbol, so an APRS station
reporting itself as a balloon is drawn as an aircraft rather than as a car. The tile layer is ours
rather than a map crate's: slippy tiles are a URL template and a Mercator
projection, and every map widget for egui brings a HTTP stack, an async
runtime and an image pipeline to do what `crates/app/src/map.rs` does in two
hundred lines against the PNG decoder the app already had. Tiles are cached
under `$XDG_CACHE_HOME/waveshark/tiles`, fetched on one thread, and a
failure is said out loud on the map rather than left looking like empty sky.

A position drawn hollow came from a single frame read against the station
position, which is right for anything in ordinary range and a whole latitude
zone out beyond about 180 nm. It is shown because it is usually right and it
appears immediately, but it never joins a trail and is never used to resolve
the next frame. A solid mark has been confirmed by a pair of frames, which
needs no reference and so cannot inherit anyone's mistake.

Zoom is continuous and anchored to the pointer; the tile level is only where
the pictures come from. Range rings are drawn around the station, not around
the middle of the window, because a ring says how far a thing is from the
antenna and that does not change when the map is dragged. The station is set
by right-clicking the map or typing coordinates above it, and is remembered in
the session file.

Once the map is zoomed in past about level nine, airports appear as amber
markers under the aircraft, their ident codes labelled as the view narrows
(large fields first, then medium, then small) and dropping where they would
cover one another. Hovering a marker shows a card with the airport's name,
code and elevation and its air traffic frequencies, primary ones first. The
airports and frequencies come from `crates/app/data/airports.tsv` and
`frequencies.tsv`, a slice of the public-domain OurAirports dataset covering
Ireland, Britain, northern France, the Low Countries, Germany, Scandinavia and
Iceland, read once at startup and held.

The packet log settings, on the `SETTINGS` button in the packet list, control
whether packets are written at all, where they go, and the size at which a
day's file stops growing. The limit is a runaway guard rather than a budget:
1090 MHz with a feed attached writes a few hundred megabytes an hour, a quiet
ISM band a few. A log that has stopped because it hit the limit says so, and
raising the limit starts it again without a restart.

### Feeds

Another receiver's frames, over TCP, added in the packet log settings. A feed
is a source node with no input of its own, so it sits beside the radio at the
head of the graph and its packets join the bus with everything the local front
ends produced: they appear in the packet list, go into the log, and reach the
flight list without any view knowing where they came from.

A format is a row in `FEED_KINDS` in `crates/nodes/src/feed_nodes.rs`: a name,
a port, a band and a function that takes frames off the front of a buffer.
Beast binary and AVR hex are the first two. Nothing outside that file knows one
from the other, so adding a format is adding a row and a parser rather than
touching the settings, the session file and the node. What belongs there is
anything carrying frames; BaseStation CSV on port 30003 does not, since it
sends fields somebody else decoded.

Because the flight list is a consumer of the bus, a feed brings aircraft with
it on a band where this receiver demodulates nothing of the sort. Tuned to
433.92 MHz for weather sensors, with a Beast feed attached, the flight list
fills from the rooftop receiver while the ISM banks run locally.

The bus itself now exists as a node: every front end feeds `packet_bus`, it
writes the packet log on the way through, and consumers attach to its output.
The flight list reads it, and so does `packet_decode`, which runs the protocol
tables once over everything and produces the rows the packet list shows. A map,
a chart or an alert would attach the same way, with one input and nothing to
say about which demodulator was involved.

None of that is worth building before the second view exists. The point of
writing it down is that the shape of the first view should not make any of it
harder.
