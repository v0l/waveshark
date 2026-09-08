# References

Everything this receiver leans on that somebody else wrote: the data it
downloads, the specifications it decodes against, the implementations it was
checked against or ported from, and the tools whose ground it is trying to
cover. Collected here because these were scattered across a dozen module
headers, and a reference nobody can find is a reference nobody checks.

The rule for what belongs here: if a claim in this program is only true
because of a document or a body of somebody else's work, that document is
listed. A link in a comment stays where it is, next to the line it explains,
and is repeated here only when it is a source rather than a footnote.

## Data downloaded at runtime

Every one of these is a row in the datasets window, cached under
`$XDG_CACHE_HOME/waveshark/data` and revalidated with the publisher's own
headers. `crates/datasets` holds the fetch and the parse; `crates/app/src/data.rs`
holds the rows, the terms and the credit each is shown with.

| what | publisher | terms | endpoint |
|---|---|---|---|
| Airports and their frequencies | OurAirports, via davidmegginson | public domain | `davidmegginson.github.io/ourairports-data/airports.csv`, `airport-frequencies.csv` |
| DMR IDs, NXDN IDs, DMR repeaters | radioid.net | for amateur use, no licence stated | `radioid.net/static/users.json`, `nxdn.csv`, `rptrs.json` |
| M17 reflectors | M17 Project | M17 Project host files | `m17-project.github.io/hostfiles/M17Hosts.json` |
| DMR, DPlus, DExtra, DCS hosts | Pi-Star | Pi-Star host files | `pistar.uk/downloads/` |
| Mobile network names by MCC/MNC | pbakondy/mcc-mnc-list | MIT | `raw.githubusercontent.com/pbakondy/mcc-mnc-list/master/mcc-mnc-list.json` |
| Cell tower positions | OpenCelliD (Unwired Labs) | CC BY-SA 4.0, visible credit and link required | `opencellid.org/ocid/downloads`, one country per MCC, needs the operator's own token, two downloads a file a day |
| Identified signals | Artemis-DB, from the Signal Identification Wiki | Artemis-DB | `github.com/AresValley/Artemis-DB` releases |
| Unidentified signals | sigidwiki.com contributors | wiki contributors | `sigidwiki.com/api.php?action=ask` |
| Orbital elements | CelesTrak (Dr. T.S. Kelso) | [usage policy](https://celestrak.org/usage-policy.php): documented queries only, one download per update, stop on any non-200 | `celestrak.org/NORAD/elements/gp.php?GROUP=…&FORMAT=csv` |
| Satellite transmitters | SatNOGS DB | CC BY-SA 4.0 | `db.satnogs.org/api/transmitters/?format=json` |
| Map tiles | OpenStreetMap contributors | ODbL, tile usage policy | `tile.openstreetmap.org` |

Two things go out rather than come in. `api.wigle.net/api/v2/file/upload`
takes the wardriving export, under the operator's own account, and
`api.beacondb.net/v2/geosubmit` takes observations for beaconDB, which needs
no account and publishes what it collects. beaconDB's `/v1/geolocate` answers
where a receiver seeing a cell probably is; it has no bulk export yet, which
is why OpenCelliD is still the file the map draws from.

The identifying string every one of these sees is `httpc::USER_AGENT`, and
there is exactly one of it: see the rule in `AGENTS.md`.

## Specifications

What a decoder is written against, rather than what it was tested with. Where
a document is behind a paywall or an ETSI portal login it is still named:
knowing which number to look up is most of the work.

| protocol | document |
|---|---|
| ADS-B / Mode S | ICAO Annex 10 Vol IV, RTCA DO-260B for the extended squitter formats |
| AIS | ITU-R M.1371, with ITU-R M.584 for the identity numbering |
| GSM | 3GPP TS 44.018 (radio resource, system information, paging), TS 24.008 (mobility management), TS 45.002 for the burst and hopping structure |
| TETRA | ETSI EN 300 392-2 (air interface), EN 300 392-7 (security), EN 300 395-2 (the full-rate speech codec) |
| DMR | ETSI TS 102 361-1 (air interface), -2 and -3 for voice and data |
| Wireless M-Bus | EN 13757-4, modes T and C |
| LoRaWAN | LoRa Alliance LoRaWAN 1.0.x specification; the PHY itself is not published and is reverse engineered (see below) |
| Bluetooth LE | Bluetooth Core Specification, the link layer and advertising channels |
| Open Drone ID | ASTM F3411 and EN 4709-002, the same message set in both |
| DARC (iBus carrier) | ETSI EN 300 751; the iBus payload above it is not published |
| RDS | IEC 62106 |
| M17 | the M17 protocol specification, `spec.m17project.org` |
| APRS | the APRS 1.0.1 protocol reference, plus AX.25 2.2 underneath |
| POCSAG | ITU-R M.584-2 |
| HMAC test vectors | RFC 4231 |

## Implementations this was checked against

A decoder is not finished because it produces output. These are the
independent implementations whose answers are treated as the truth in tests,
which is a different and stronger claim than "it runs".

- **rtl_433** (`github.com/merbanan/rtl_433`) is the reference for the entire
  sub-GHz device family. `crates/decode/tests/rtl433_corpus.rs` replays
  captures from its own test corpus and compares field for field against the
  JSON rtl_433 25.02 emitted. The corpus is also what the burst classifier
  and the source detector are scored on.
- **dump1090** (the `dump1090-rb` fork) for Mode S: verified over a shared
  recording, 27 of its 40 frames and no frame it did not also see.
- **osmo-tetra** and Midnight Blue's TETRA research for the TETRA stack; see
  the ports below.
- **Flipper Zero firmware** (Momentum fork) for the fixed-code gate remotes.
- The transmitters themselves, where an independent decoder does not exist:
  a LoRa frame carrying its own CRC, an M17 transmission carrying a callsign,
  and a Holybro RemoteID module shipped with a known serial are each evidence
  no second implementation is needed for. See the corpus rules in `AGENTS.md`.

## Code ported or vendored

Kept honest here because a licence obligation that lives only in a module
header is one a packager will miss.

| where | from | licence |
|---|---|---|
| `crates/mbe` | `DSheirer/jmbe`, the AMBE 3600x2450 and IMBE 7200x4400 codecs | see `crates/mbe/README.md`; patent encumbered, off by default |
| `crates/hackrf-usb` | `rs-hackrf` 0.4.2 by Xavier Olive | MIT |
| `crates/dsp/src/stereo.rs` | the approach in the `fmradio` crate | MIT |
| `crates/decode/src/tea.rs` | Midnight Blue's `TETRA_crypto` reference | reverse-engineered reference |
| `crates/decode/src/ta61.rs` | osmo-tetra `taa1.c` and the same research | reverse-engineered reference |
| `crates/decode/src/vocoder/` | a GPL reimplementation of the EN 300 395-2 reference decoder, not a copy: the ETSI source is under a copyright that cannot be vendored | GPL, written here |
| `crates/decode/src/protocols/keyfob/` | Flipper Zero Momentum firmware subghz protocols | GPL |
| `crates/orbit` | the `sgp4` crate, which implements the model in Vallado's *Revisiting Spacetrack Report #3* | MIT/Apache |

## Publisher rules this program is written around

Terms that change how the code works, not only what it prints.

- **CelesTrak** ([usage policy](https://celestrak.org/usage-policy.php),
  [GP data formats](https://celestrak.org/NORAD/documentation/gp-data-formats.php)).
  Ask only the documented `gp.php` query; download only when the data is
  going to be used and once per update, GP being rebuilt every two hours;
  and stop immediately on any answer that is not a 200, reporting it to a
  human rather than retrying. All three are in the code: the URL is built in
  one place and asserted by a test, `MAX_AGE` is six hours and nothing polls,
  and `Cache::refresh` records a refusal and refuses further automatic
  attempts until somebody presses refresh. The two-line format is also
  finished: the catalogue passed five digits in July 2026 and new objects
  have no TLE at all, which is why `datasets::tle` reads CSV.
- **OpenCelliD.** Two downloads of a file a day, and a visible "OpenCelliD"
  credit with a link wherever the data is drawn. The allowance is answered
  with a 200 and a JSON complaint, so `Source::check` refuses that body
  before it can replace a good export, and the map's credit plate names them
  while their masts are on screen and only then.
- **OpenStreetMap tile policy.** A real user agent, which is
  `httpc::USER_AGENT`, and the attribution on the map.
- **beaconDB.** No credential, but they ask clients to identify themselves by
  user agent, and what is submitted is somebody's movements, so both the feed
  and the lookup are off until an operator turns them on.

## Prior art

What this is measured against, in the sense of "does it do what these do".
The target stated in `docs/protocols.md` is the union of rtl_433, a Flipper
Zero, a PortaPack running Mayhem and SDRangel, in one receiver.

- **rtl_433** for sub-GHz devices, **SDRangel** and **GNU Radio** for the
  signal path, **Universal Radio Hacker** for the unknown-burst analysis,
  **gqrx** and **SDR++** for what a general receiver's controls should feel
  like.
- **Gpredict** for satellite work: SGP4/SDP4, any number of satellites and
  ground stations, list, map and polar views, pass prediction with tunable
  conditions, Doppler tuning and rotator tracking through hamlib, and
  transponder data imported from SatNOGS. What it has that this does not is
  listed below.
- **Kismet** and **WiGLE** for the survey and its export format.
- **overpass turbo** (`overpass-turbo.eu`) over the **Overpass API**
  (`overpass-api.de/api/interpreter`), for asking OpenStreetMap what is
  actually there: masts and towers (`man_made=mast`, `communication_tower`),
  aerialways, airfields, level crossings. Nothing here queries it yet. It is
  listed because it is how a question about the map gets answered while
  writing a layer, and because it is the obvious source if the receiver ever
  wants to draw the transmitter sites near a station rather than only the
  ones it has heard. ODbL, like the tiles, and rate limited: a query is a
  computation on somebody's server, not a file.
- **Wireshark** for the packet list, the link directory and follow-stream,
  which is where those views' shape comes from.

## What Gpredict has that this does not

Sorted by what it would buy a receiver, not by what it would cost. Already
taken from it: Doppler tracking that follows a pass, the sky plot, the
footprint circle, and free-space loss with one-way delay.

**Worth having, in roughly this order.**

1. **Sunlit or eclipsed.** A solar-powered beacon that goes quiet in the
   Earth's shadow is not a broken decoder, and saying which it is costs a sun
   vector and a cylinder test.
2. **A time control.** Running the clock forward to see where things will be.
   The map already draws from a timestamp, so this is a slider on the view
   rather than new machinery, and it is how a session is planned rather than
   watched.
3. **AOS and LOS notice.** Something that says a pass is starting while
   another view is open.

**Worth having later.**

4. **Rotator control.** `rotctld` over TCP, az/el from the look angles the
   view already computes. By the rule in `AGENTS.md` it belongs on the graph
   as a node with the pass as its parameter, not as a thread poking a socket
   from the interface.
5. **Uplink and transponder pairing.** SatNOGS carries the uplink and whether
   a transponder inverts, and this receiver can transmit. Working a linear
   transponder means correcting both ends in opposite directions, which is a
   real feature and a real way to transmit somewhere you should not.
6. **Several ground stations.** Here there is one, because everything else in
   the receiver is about where this antenna is. Worth it only for somebody
   planning a schedule rather than listening.

**Deliberately not.**

- **Modules with saved layouts.** Gpredict's modules exist because tracking
  is its whole surface. Here the satellites are one view of a receiver and
  the panes are already the layout.
- **SDP4 as a separate path.** The `sgp4` crate handles the deep-space terms
  inside `Constants`, so there is nothing to choose between.
- **Its own element update scheduler.** The datasets window already fetches
  and revalidates on an age, which is the same job with a UI that exists.

## Test fixtures

Recorded IQ is not committed. `testdata/fixtures.toml` and
`testdata/offair.toml` carry the manifest: a name, a sha256 of the compressed
upload, the URL on nostr.download, the centre and rate, and a description
saying what the capture is evidence of and how that was established. The
procedure for adding one, including why a capture earns its place only by
failing something, is in `AGENTS.md`.
