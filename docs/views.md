# Views on the packet stream

The packet list is not a display. It is the place every decoded frame arrives,
which makes it the natural bus for everything that wants to show one: a map for
ADS-B and AIS, an image pane for weather satellites and SSTV, a chart for a
temperature sensor, a text pane for pager traffic. Each of those is a view over
the same stream, filtered differently, and none of them should need a private
path back to the demodulator that produced it.

This file describes the contract a view uses, the views built on it, and what
the ones not built yet would need.

## What a packet carries

`radio::DecodeRecord` is the unit on the bus. The parts a view cares about:

| Field | What it is | Who uses it |
|---|---|---|
| `at` | when the burst arrived, stamped at the start of the block that held it | every view, for ageing and ordering |
| `freq` | centre of the channel it arrived on | list, map (as a filter) |
| `channel_hz` | how wide that channel was, and so how far apart two reports must be to be different bursts | deduplication |
| `model` | protocol name, or "unknown" | routing |
| `media_type` | what `bytes` holds | routing |
| `fields` | the decoder's own fields, structured | map, chart, text |
| `bytes` | the raw frame | hex dump, image pane |
| `iq` | the burst's own samples, where the front end kept them | the packet list's burst detail |
| `audio` | decoded speech, with the call it belongs to | call list, audio bus |
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

## Getting to one

The views are tabs in the top bar, in one row of eleven with a gap in the
middle: what the receiver is doing and what it heard on the left of it, who is
out there on the right, in the order of their shortcuts. A
dropdown was there first and cost two clicks and a read of a ten-item menu
each way, which is most of a second to glance at the map and another to come
back; the strip is one click, and it shows which view is open without being
opened itself. `Ctrl` and a digit in reading order selects one, and
``Ctrl+` `` swaps with the last view, which is the movement an operator makes
most. The transcript is `Ctrl+T` rather than a digit: there are eleven views
and ten digits, and renumbering the strip to fit it in would have moved every
shortcut an operator had already learned.

A tab carries a dot when its view has taken something in since it was last
looked at: a track, a call, a message, a device, a picture. Not when it holds
something, which was the first rule and is useless on a real band, where every
list is non-empty a minute after the radio starts and every dot is then lit for
the rest of the session. `App::view_mark` gives each view one number that moves
when it fills up, `read_views` records that number for the open view on every
frame, and the dot is the difference. Both have to stay cheap, since they are
asked for every view on every frame; the spectrum and the chain answer nothing,
because traffic does not collect there.

## The views

- **Packet list**: every record, newest at the bottom, with a detail pane
  showing the selected packet's burst as the front end saw it, its
  envelope and instantaneous frequency against time, then its fields and a
  hex dump. The burst view is what an unknown device is worked out from, the
  way Universal Radio Hacker shows a burst beside its bits; the samples are
  kept for the newest rows only. `crates/app/src/ui/packets.rs`.

- **Map**: everything on the bus that reports a position, over OpenStreetMap
  tiles, with a table of tracks beside it. Described in full below.
  `crates/app/src/ui/map_pane/`.

- **Video**: whatever the video bus is publishing, drawn. A view over that
  bus the way the call list is a view over the audio one: it reads a field off
  the status and knows nothing about which front end produced it or on what
  band. Two things it does that a plain image viewer would not. It prints
  `lines_seen` over the picture and colours the caption when the field is
  incomplete, because analogue video has no integrity check of any kind and a
  fade looks like a picture until the count is read. And it clears itself half
  a second after the last field rather than holding the last one, since a
  still picture of a transmitter that has gone away is the worst thing this
  pane could do. `crates/app/src/ui/video_pane.rs`.

- **Calls**: who is talking, from anything that produces speech. It is fed
  from two buses and keyed so both land on one row. The audio bus, which
  every demodulator's audio passes through first, says who is talking now
  and for how long (`AudioBus::track`, published as `Status::heard`), and
  that is where every analogue call and the airtime of every digital one
  comes from. The packet bus says what only a decoder knows, `call_type`,
  the cipher, the codec, off a decode marked `voice`. A DMR call, a TETRA
  call, an M17 call and an FM channel marked as voice are the same row with
  different fields filled in. An analogue over is never a packet: there is
  no packeting in it, and it goes nowhere near the packet log.
  `crates/app/src/calls.rs`.

- **Transcript**: what was said, as the model on the audio bus tap read it,
  newest at the bottom. A conversation is the transcriber's key,
  `{proto}:{freq}:{chan}:{speaker}`, so the pane can be opened on one and
  nothing else, which is what the calls list's "read" button does; that
  button is drawn only on rows the log has lines for, since a way into an
  empty pane says something was heard when nothing was. A line still being
  spoken is dim and marked, because the next partial replaces it, and a
  reading Whisper is unsure of says so rather than being trusted silently.
  `crates/app/src/ui/transcript_pane.rs`.

  The pane also draws the model as a piece of equipment: which one, where its
  files are, whether they are there at all and what they are, its state, the
  device it runs on, and how fast it read the last window. Without that, a
  model that was never downloaded, one that failed to load, one too slow to
  keep up and a band where nobody is talking all look the same, which is to
  say they all look like an empty pane. What is on disc is what is named,
  because the repository setting is only where files would be fetched from
  and a directory filled by an earlier run holds a different model. The
  status comes off the node through `Status::transcriber`. The transcript
  itself is one for the whole program, `transcripts::log()`: the node writes
  into it and the view copies it when its sequence number moves. It is not
  kept in the node, because the node is a stage in a graph that is rebuilt
  on every retune, and a log that lived there was emptied every time the
  dial moved.

- **Messages**: every record carrying a `text`, `message` or `sms` field,
  newest first, each drawn as a header line and the words underneath at full
  width rather than clipped to a column. The recipient is read from
  `addressee`, `to`, `dst`, `destination`, `talkgroup`, `channel` or
  `address`, so a pager's capcode, a TETRA talkgroup and a mesh channel land
  in the same place; the sender from `from`, `src`, `source` or `radio_id`.
  Identical words from the same sender to the same recipient on the same
  system inside two minutes are one message with a count, because a pager
  sends every page twice and TETRA retransmits until it is acknowledged.
  `crates/app/src/messages.rs`.

  What reaches it is what a decoder says is text, `media::TEXT`, and not
  whatever has a field called `message`. GSM names its own blocks in one
  (`SI3`, `Paging1`) and Open Drone ID names its message types the same way,
  so reading the field alone filled this view with a network talking to
  handsets. A message is somebody writing to somebody.

  The reading itself is `DecodeRecord::to_message`, so anything holding a
  record can ask it for a message rather than repeating the field names:
  the packet log as it appends, a feed, or a view added later. It borrows and
  does not consume, because the record carries on to the log and the message
  is a second reading of it. Keeping it on the record rather than on each
  decoder is what stops a protocol needing a private path to this view.

  The trap this replaces is worth knowing, because it is silent. A decoder
  that names its sender field something outside the list above still shows its
  messages, with nobody's name on them: MeshCore called it `sender` and its
  traffic arrived under a blank header. Nothing errors, and the view looks
  like it works.

- **Devices**: one row per transmitter that identified itself, rather than one
  per transmission, with what it called itself, who made it, the strongest
  level it was ever heard at and when it was last heard. This is the survey
  view: the packet list is unreadable from a moving car, where a hundred rows
  a second arrive and most of them are the same beacon. Selecting a row draws
  that device's sightings on the map. `crates/app/src/ui/devices_pane.rs`,
  over `crates/survey`.

- **Links**: who is talking to whom, on every protocol at once. Wireshark's
  conversation list and follow-stream for radio: a table of links, most
  recently heard first, and picking one opens the packets that link carried
  underneath it, in order, with the payload where the protocol gives one in
  the clear. A link is a pair of ends on one protocol, filled in by the
  decoder that recovered the frame, because it is the only thing that knows:
  DMR reads it off the link control, BLE off the advertiser and any directed
  target, Meshtastic off the mesh header. A transmission addressed to everybody is not a
  link: a meter, a beacon and an advertiser talk to the air, which is a
  reception the packet list already has and a transmitter the device list
  already has. Both ends have to be named, and a group counts as one. Radio is mostly not a byte stream, so
  following a link is the packets and their fields rather than a
  concatenation, and those packets are the packet log filtered by the pair
  rather than a second copy kept beside it. The directory is built from records, so the same code fills
  it from the bus or from the packet log, and the pane can read the log back
  to show what was heard before the receiver was started.
  `crates/app/src/links.rs`, `crates/app/src/ui/links_pane.rs`.

  Reading the ends out of the display fields was the first version and it was
  wrong quietly: those strings are for a person, so a talkgroup, a callsign
  and a MAC were all text, and `9` on DMR merged with `9` anywhere else. A
  party is a kind and an identifier, and `broadcast` is a kind rather than a
  word a device could be called.

- **Calls**, continued: the text beside a call comes from the transcriber on
  the audio bus tap, matched by the key
  `{proto}:{freq}:{chan}:{speaker}` rather than by a wire between the two.
  Neither knows the other exists, which is what lets a view added later ask
  the same log for the whole of a conversation instead of the last line of
  it, which is what the transcript view does. See
  `crates/app/src/transcripts.rs`.

- **Video**: the picture, from anything that produces one. The bus keeps a
  channel per transmission, keyed by what it is and where it was received
  rather than by which wire it arrived on: everything the auto node finds
  comes in on its one video port, so keying on the port collapsed several
  cameras into one picture that flickered between them. A chooser lists the
  channels that have sent a field lately and picking one subscribes to it;
  "best" shows whichever picture is most complete. The image is drawn at the
  shape the transmission says, 4:3 for both analogue standards, and not at
  the shape of the sample grid, which is a fact about the receiver's clock.
  The caption says the grid, the lines received out of the lines a field
  has, and the channel: analogue video has no integrity check, so how much of
  the picture arrived is the only quality there is.
  `crates/app/src/videobus.rs`, `crates/app/src/ui/video_pane.rs`.

- **Satellites**: the one view that shows something nobody has heard. Every
  pass over the station in the next day, from elements CelesTrak published
  and the SGP4 model they were fitted for, with the peak elevation, the
  azimuths to point at, and for whatever is above the horizon now the live
  look angles and the Doppler on a two-metre downlink. Because it is a
  prediction and not a reception, the age of the elements is on the screen:
  a set a fortnight past its epoch puts a low satellite kilometres from
  where it is, and a table that does not say so is fiction drawn as fact.
  Selecting a row draws that satellite's ground track and footprint on the
  map, and clicking one on the map selects it here.
  `crates/app/src/ui/sats_pane.rs`, `crates/app/src/sats.rs`, over
  `crates/orbit` and `datasets::tle`.

  Two approximations in `crates/orbit` are deliberate, and both are far
  inside the error of the elements themselves: the rotation from the model's
  frame to the Earth's leaves out polar motion, and the elevation is
  geometric, with no refraction lifting a satellite on the horizon by about
  half a degree. What Gpredict has that this does not is in
  `docs/references.md`.

  Two costs kept apart, because they differ by three orders of magnitude:
  where one satellite is now is a single propagation and happens while
  drawing, and when a hundred satellites next rise is a search over a day
  that runs on a thread and is published when it finishes.

- **Keys**: a row per enciphered channel a front end reports, and what is known
  about the key for it. The view is always there as an encryption monitor; the
  key store, key entry and the TETRA decryption behind it need the `tea`
  feature. `crates/app/src/ui/keys_pane.rs`.

All of them are views by this definition. None knows anything about a protocol.

A row in the messages, devices or links view does not tune the receiver.
They are records of what has been heard, and a click that retuned took the
receiver off the band it was surveying or the channel somebody was
listening to, which is the opposite of what any of them is for. The calls
view is the exception and keeps it: picking a call is asking to hear it.

## Where retention and identity live

An early draft of this file said a capped vector in the app would not carry
more than two views, and listed retention per view and indexing by identity as
work to be done. Both happened, and not in the store: they moved into the
graph. A view that needs either holds a node on the bus and publishes the table
its pane copies each frame. `TracksNode` (`crates/app/src/tracks.rs`,
registered as `tracks` in `chain.rs`) is the worked example: it keeps the last position per identity
with a trail, ages each kind on its own clock, and the map pane draws whatever
it published on the last frame. The message store does the same thing with its
own cap.

What is left in the app is the packet list's own last 500 records, which is a
list of what arrived rather than a store anything else reads, and is capped
because it carries burst samples for the newest of them.

The bus is a node too: every front end feeds `packet_bus`, which writes the
packet log on the way through, and consumers attach to its output. The flight
tracker reads it, and so does `packet_decode`, which runs the protocol tables
once over everything and produces the rows the packet list shows. A chart or an
alert would attach the same way, with one input and nothing to say about which
demodulator was involved.

Replay goes through the same bus. `--record` writes the IQ of every burst and
an `index.jsonl` describing it, and `--replay` runs a capture back through the
whole receiver rather than a simplified copy of it, bus included, and prints
what came out. A view can be driven from a file by reading the bus instead of
the printout.

## The map

Aircraft from ADS-B on 1090 MHz, vessels and navigation marks from AIS on
162 MHz, vehicles and stations from APRS on 144.800, and mesh nodes from
Meshtastic and MeshCore, all on OpenStreetMap tiles.

Five sources, one tracker, and the differences between them are where the
design is. The tracker parses none of them: the protocols run once on the bus
and it reads what they concluded, an identity saying which track and a report
saying what sort of thing it is. Identity is shared but is not a number: an
ICAO address, an MMSI, a callsign and a node hash are different identity
spaces, so a track is identified by the pair of space and value and nothing
can collide. Position reassembly is not shared at all; only Mode S has compact
position reporting, so the decode carries the halves it received and the CPR
machinery here pairs them, which it can do because it knows where that
aircraft was a second ago and one frame does not.
Ageing and plausibility are shared but not constant: an aircraft silent for a
minute is gone, a vessel lasts ten, a vehicle half an hour and a station an
hour.

A kind decides how a thing is drawn and how long it is remembered. AIS says
which it is by message type; APRS says it with a symbol, so an APRS station
reporting itself as a balloon is drawn as an aircraft rather than as a car.

The tile layer is ours rather than a map crate's: slippy tiles are a URL
template and a Mercator projection, and `crates/app/src/map.rs` does that in
under four hundred lines over the HTTP client, runtime and PNG decoder the app
already had, rather than taking on a widget's own camera, cache and layer
model. Tiles are cached under `$XDG_CACHE_HOME/waveshark/tiles` and fetched on
a two worker background runtime, two requests in flight because that is what
the tile usage policy asks for. A failure is said out loud on the map rather
than left looking like empty sky.

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
airports and their frequencies are the OurAirports files, whole world, fetched
and cached by `crates/datasets` and revalidated a day apart; only heliports,
seaplane bases, balloonports and closed fields are dropped. An earlier build
shipped a filtered slice of northern Europe in the binary, which made a release
the only way to fix a wrong frequency and left a receiver anywhere else looking
at an empty map. `--fetch-data` warms that cache before going somewhere without
a connection.

## The packet log settings

What is written is evidence: timings, bytes, samples or a measurement. A
transmission that has none of those is not written, which is how speech stays
out: an over from a voice channel is on the bus so it can be heard, listed as
a call and read by the transcriber, and its body is an empty frame with the
audio hanging off it. Logging that put a row with nothing in it into the file
for every transmission, and replaying one produced a packet no decoder could
say anything about.

On the SETTINGS button in the packet list: whether packets are written at all,
where they go, whether unrecognised bursts are shown in the list, and how much
the whole folder may take. The limit is a runaway guard rather than a budget:
1090 MHz with a feed attached writes a few hundred megabytes an hour, a quiet
ISM band a few. The oldest days are deleted to keep the folder under it, so the
log rolls rather than stopping. It stops only when a single day's file is over
the limit on its own; it says so, and raising the limit starts it again without
a restart.

## Feeds

Another receiver's frames, over TCP, added in the packet log settings. A feed
is a source node with no input of its own, so it sits beside the radio at the
head of the graph and its packets join the bus with everything the local front
ends produced: they appear in the packet list, go into the log, and reach the
tracker without any view knowing where they came from.

A format is a row in `FEED_KINDS` in `crates/nodes/src/feed_nodes.rs`: a name,
a port, a band and a function that takes frames off the front of a buffer.
Beast binary and AVR hex are the first two. Nothing outside that file knows one
from the other, so adding a format is adding a row and a parser rather than
touching the settings, the session file and the node. What belongs there is
anything carrying frames; BaseStation CSV on port 30003 does not, since it
sends fields somebody else decoded.

Because the tracker is a consumer of the bus, a feed brings aircraft with it on
a band where this receiver demodulates nothing of the sort. Tuned to 433.92 MHz
for weather sensors, with a Beast feed attached, the map fills from the rooftop
receiver while the ISM decoders run locally.

## The device database

A second thing to do with the same bus: record the transmitters rather than
the transmissions. `SurveyNode` (`crates/nodes/src/survey_nodes.rs`) is a sink
on the bus like the tracker, and `crates/survey` is the store behind it, a
SQLite file with a table of devices and a table of sightings.

Three rules shape it.

**Identity is a pair.** `(space, ident)`, the way `tracks` learned to key
identity when AIS arrived beside ADS-B: an ICAO address and an MMSI are both
integers and are not comparable. The identifier is kept as text a person would
recognise. The decoder says it, as `common::Identity` on the decode, and the
node reads nothing else: a table here of which field each protocol keeps its
identifier in was the last thing in the tree keyed on a protocol name, and it
is gone. An ISM sensor's id is eight bits picked when the batteries go in, so
for those the model is part of the space: an Acurite and a Nexus sharing id
163 are two devices. That is `decode::Report::device`, filled from the `id`
field because in that family of protocols `id` is what the sensor calls
itself.

**A sighting is where the receiver was, not where the device is.** It carries
the time, the position from the GPS, the level and the frequency. A survey
driving past a beacon is a line of positions along a road with a level at
each; the strongest of them is the closest approach, and that is what the
store claims. The map draws the trail, and beside it what the trail can
say: `survey::locate` fits the log-distance model, `rssi = P0 - 10 n
log10(d)`, for the transmitter's position with its own strength solved out
and the exponent fixed at 2.5, by a grid search over the drive's reach.
What it draws is the point and the region the levels fit about as well
(a chi-square test on the rise in squared residual, at the 95th
percentile), because levels carry no bearing: a drive along one road
cannot tell which side of it the transmitter is on, the region reaches
across the road to say so, and only a drive that turns a corner closes
it. It wants at least four positioned sightings spread over a hundred
metres, and says nothing from fewer. On a synthesised drive round a block
with 3 dB of fading it lands within 40 m; on a straight one it lands
abeam and reports a radius that reaches the mirror image.

**Sightings are thinned, devices are not.** A beacon advertising ten times a
second for an hour is thirty-six thousand rows that all say the same thing, so
a new sighting is written when the receiver has moved 25 metres, when the
level has changed by 6 dB, or after a minute. What is never thinned is the
device row: first heard, last heard, every reception counted, the best level
and where it was heard from.

The position a sighting carries is the station position, which is the one
position the whole receiver works from: typed into settings, taken from the
country the first time, or moved by a fix. There is no second position for the
survey, which is what a survey used to have, so a receiver whose position was
entered by hand wrote every row blind while the map drew it on its aerial.

Fixes come from `crates/gps`: NMEA from a serial port, or gpsd on TCP, one
parser behind both. The reader is `crate::station`, one per process rather
than one per radio: it used to belong to the radio thread, so there were no
fixes until a device was chosen and none while one was being swapped. It runs
for as long as the program does and looks for a gpsd on the
loopback, so a machine that has one needs nothing set; a refused connection is
retried every five seconds, which is also how a daemon started later is picked
up. A fix goes stale after ten seconds, and the station then stays where the
last fix left it rather than following a receiver that has lost the sky.

`--survey FILE` points the file somewhere, `--no-survey` turns it off, and
`--gps /dev/ttyACM0` or `--gps gpsd:host` names a GPS that is not the local
daemon. Both are in settings under the station position, which is where they
belong: a GPS is the other way of answering the question that box asks, and
while one is producing fixes the station is wherever the last one put it, so
the range rings, the map and anything resolving a position against the
receiver follow the car. The pane
exports WiGLE CSV, which is what wardriving tools read; the survey file itself
is the record, and the CSV is a copy shaped for other people's tools.

### Feeding wigle.net

The same rows can go straight to wigle.net while the drive is happening.
`WigleNode` (`crates/nodes/src/wigle_nodes.rs`) is a second sink on the bus
beside the survey, reading the same decodes and thinning them by the same
rule, and `crates/survey/src/wigle` is the format and the one request that
sends it.

Three things decided its shape.

**Only what the format has a type for.** A BLE or Bluetooth address, and a
GSM cell that named itself, keyed the way WiGLE keys a cell:
`MCCMNC_LAC_CID`, with the carrier as an ARFCN rather than as hertz.
Aircraft, vessels, pagers and tyre sensors have no type here and are not
filed under one that nearly fits: a row in the wrong bucket is wrong in
somebody else's database forever, and nothing downstream can tell it from a
real one. The export the pane writes follows the same rule.

**A spool, not a request per sighting.** WiGLE takes files, and the roads
worth driving have no coverage: that is why the survey exists. Rows go to a
file under `$XDG_DATA_HOME/waveshark/wigle`, closed at two thousand rows or
five minutes, and a thread sends closed files oldest first whenever there is a
network, deleting each only once WiGLE has answered that it took it. A laptop
that comes home to a wireless network sends the whole drive without being
asked. Nothing is held in memory that a power cut would cost: the node writes
what it has collected when it is dropped, which is what a retune does to it.

**One uploader for the process.** A graph is rebuilt on every retune, so a
node lives for seconds while an upload takes minutes. The uploader is a
process-wide thread the node points at rather than something the node owns, so
a rebuild mid-upload neither interrupts it nor lets a second thread send the
same file again.

The account is set in the WiGLE dialog off the devices pane: the API name and
token from wigle.net/account, whether WiGLE may licence the rows on
commercially (off unless it is asked for; they are the operator's
observations to give away), and the switch. It is saved in the session file,
token included, and the dialog says so. The dialog is also where the feed
reports: how many rows are waiting, how many have gone, the last transaction
id, and what the last refusal said. A refusal arrives as a 200 with
`success: false`, so a file is deleted on what WiGLE said and not on the
status code.

## Views not built yet

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
makes it the cheapest of the three. The Fine Offset decoder already produces
everything it needs.

### More into the message view

The message view takes any decode with a text field, so FLEX, ACARS and AIS
safety messages join it by naming their fields the way POCSAG, M17, TETRA and
APRS already do. What it does not yet take is a payload that is text without
being a field: `media_type` of `text/plain` with the words in `bytes` should be
read as the message body.

The shape has held so far. Every view added since the first attaches to the bus
with one input and nothing to say about which demodulator was involved.
