# Protocols

The target is the union of what rtl_433, a Flipper Zero, a PortaPack running
Mayhem and SDRangel can do, in one receiver, and transmit for the same set. This file lists those protocols, what each one costs to add, and which
direction is realistic for it.

The point of the list is to make the cost visible before starting, because
"add a protocol" ranges from a twenty line table to a new receiver.

## What decides the cost

Everything downstream of a burst is cheap. The expensive part is the front end
that turns radio into symbols, and there are only a few of those.

| Front end | Produces | Receive | Transmit |
|---|---|---|---|
| envelope, `pulse_detect` | mark/gap timings from OOK | yes | no |
| buffered envelope, `ask_detect` | mark/gap timings from shallow ASK | yes | no |
| discriminator, `fsk_detect` | mark/gap timings from two-level FSK | yes | no |
| pilot PLL, `wfm` | stereo audio, RDS | yes | no |
| discriminator, `c4fm_detect` | 4-FSK level symbols | yes | no |
| discriminator plus sync correlation, `m17` | M17 frames | yes | no |
| coherent PSK/GMSK with timing recovery | soft symbols | no | no |
| chirp correlator (dechirp then FFT), `lora` | LoRa symbols | yes | no |
| OFDM (FFT, pilots, equaliser) | subcarrier symbols | no | no |
| DSSS despreader | chip-synchronised symbols | no | no |

The four-level burst front end differs from the others in needing to be told
the symbol rate, because a four-level eye cannot be opened without knowing
where the symbol boundaries should be, and it emits numbered levels rather
than mark/gap timings, because two like symbols in a row are one run and four
levels give no rule for splitting it again. FLEX, ERMES and wireless M-Bus
mode N still read **demod** below for want of a level to bit mapping, a sync
word and a framer on top of it, rather than for want of a demodulator.

M17 does not go through it, and the reason generalises to the rest of the
digital voice modes. A burst detector gates on envelope, which suits a packet
with silence either side; a voice transmission is a continuous carrier that
can last minutes and whose clock has to hold for all of it. M17 puts a 16 bit
sync burst in front of every 40 ms frame, so `dsp::m17` correlates for the
next sync and reads the 184 symbols behind it, and never holds a clock for
longer than one frame. DMR, P25 and NXDN are framed the same way and would be
read the same way; what stops them is the vocoder, not the demodulator.

Which front end runs is measured rather than configured. Each channel gates a
burst once and `dsp::classify` measures it: envelope levels and how long each
is held, occupied bandwidth, the histogram of instantaneous frequency, the
symbol rate from the transition line, the power-law lines that give phase
keying away, and the frequency slope that gives a sweep away. The burst then
goes to the one front end that can read it, and to both pulse front ends when
the measurement will not name it, which is what every channel used to do with
every burst. Scored against rtl_433's recordings, whose devices and therefore
modulations are known, it puts 46 of 52 in the right family;
`crates/decode/tests/classify_corpus.rs` prints the confusion matrix and lists
the six by name. The classes it can name but not yet read (MSK, BPSK, QPSK,
chirp, noise-like, bare carrier) are labels on the burst rather than routes to
anything.

A protocol whose symbols reach the mark/gap layer costs a timing table and a
payload parser, and nothing else: the slicers (PWM, PPM, Manchester, NRZ), the
CRC helpers, the unknown-burst analyser and the packet list already exist.
Everything else costs a demodulator first.

Transmit inverts the same layers and none of it is written yet. See
[Transmit](#transmit) below.

The second constraint is width, and it is no longer a constraint on the
scanner. The `auto` node watches its band as a spectrogram, and a run of bins
over the floor that persists from one frame to the next is a source, with its
centre and width measured rather than assumed. Each source is cut out at a
rate that fits its width, so a 1.5 kbit/s OOK sensor is read through a
channel a few kilohertz wide and a LaCrosse sensor keying tones 120 kHz apart
is read through one that holds both, and a pager channel is found wherever it
is rather than where a block said. The classifier still reports the occupied
bandwidth next to the width it was given, since a source that fills its
extraction is one whose extent was measured wrong. Anything a spectrogram
cannot find, Mode S and AIS, the node runs its own demodulator for when the
span covers the frequency.

Sources cost what is transmitting: an empty band is one FFT, and each source
that opens is a mixer and two decimators for as long as it lasts, plus the
frame decoders where the width warrants them. At 2.4 MS/s on an empty band
that measures about 9x real time on a 48 core machine against the 6.1x the
four bank tiers took, in `radio::tests::the_scanner_keeps_up_with_the_stream`.
Scored on rtl_433's corpus by `crates/nodes/tests/source_corpus.rs`, the node
recovers 50 of the 57 reference decodes against the tiers' 48, and loses
ground on no capture. The tiers remain a front end a scanner block can ask
for as `banks`, for that comparison.

The third is hardware.

| Radio | Range | Rate | Direction |
|---|---|---|---|
| RTL-SDR | 24-1766 MHz | 2.4 MS/s | receive only |
| HackRF One | 1 MHz-6 GHz | 20 MS/s | half duplex, transmit and receive |
| PortaPack | a HackRF with a screen | as HackRF | as HackRF |

Out of scope whatever the ambition: a Flipper's 125 kHz RFID, its 13.56 MHz
NFC, its infrared and its iButton are near-field or optical, not radio an SDR
can reach.

## Status codes

Receive:

- **done**: decoding now, verified against a recording
- **synthetic**: decoding now, but only checked against frames this project
  built itself from rtl_433's published layout. The parser is exercised; the
  timings and the front end in front of it are not
- **table**: fits an existing front end. A timing table plus a payload parser
- **framing**: fits an existing front end, but needs sync words, bit
  destuffing or forward error correction that is not written yet
- **demod**: needs a demodulator this project does not have
- **chain**: needs a receive chain of its own, not a channel in a bank

Transmit:

- **table**: the same timing table, run backwards, once the transmit path
  exists
- **mod**: needs a modulator beyond OOK/FSK keying
- **chain**: needs its own transmit chain

The transmit column is an engineering estimate and nothing else. What is legal
to radiate depends on the band, the power, the antenna and the country, and
that is the operator's call, not this file's. The one place it becomes a code
concern is duty cycle: parts of 868 MHz are capped at 1%, and a scheduler that
enforces the cap is easier to trust than an operator who has to remember it.

## How a status is earned

A **done** here means a recording of the real device decodes to the same
fields another implementation got from the same bytes.
`crates/decode/tests/rtl433_corpus.rs` replays captures from rtl_433's own test
corpus and compares against the JSON rtl_433 25.02 emitted for each one, field
by field. `testdata/rtl433.toml` lists them, and `testdata/fetch.sh` pulls them
from the upstream repository at a pinned commit.

Two things are checked, and the second matters as much as the first. Every
decode rtl_433 found must be found here with the same values, and nothing
reporting a passing integrity check may claim a burst rtl_433 read as something
else. A receiver meant to identify unknown signals is not helped by a decoder
that finds the right sensor and three imaginary ones.

Known gaps, listed in the test so that closing one fails until the note is
removed:

- Two sensors transmitting inside one burst yield one decode, because a
  protocol returns the first frame it finds in a package. rtl_433 reads its
  rows separately and reports both.
- The THR228N is reported as a THN132N. They share a sensor id and a frame
  layout, and rtl_433 tells them apart by message length, which is not
  measurable here: a burst runs one copy straight into the preamble of the
  next, and that preamble unpacks as valid Manchester pairs, so the frame never
  ends where the transmitter stopped. Both sensors report the same fields.

## ISM sensors, remotes and telemetry

The rtl_433 and Flipper sub-GHz domain: roughly 250 device decoders in
rtl_433, almost all OOK or two-level FSK, almost all reachable from the
existing pulse front end. Lowest marginal cost, highest coverage gain.

| Protocol | Where | Modulation | Width | RX | TX | Notes |
|---|---|---|---|---|---|---|
| Fine Offset WH1080 family | 433.92 MHz | OOK PWM 544/1524 us | 31 kHz | done | table | CRC8, matches rtl_433 25.02 field for field, including the DCF77 clock message the station sends around minute 59 |
| Fine Offset WH51 soil moisture | 433.92/868/915 MHz | FSK 58 us | 125 kHz | done | table | CRC8 and a checksum, moisture as a raw AD count and a percentage |
| PT2262 / EV1527 / HS1527 fixed code | 315/433.92 MHz | OOK PWM | 31 kHz | synthetic | table | Garage doors, doorbells, cheap sensors. The most common thing on 433. No integrity check at all, so a burst is only claimed when it is exactly one frame long |
| Princeton, Holtek, CAME 12/24, Ansonic, Bett, Nice Flo, Linear, Holtek HT12x, Linear Delta3 | 315/433.92 MHz | OOK PWM | 31 kHz | synthetic | table | Flipper's fixed-code gate remotes, ported from Momentum-Firmware. No checksum, so a frame is only claimed when it repeats or the package is plainly one frame, and degenerate all-0/all-1 frames are refused. On rtl_433's recordings several of them still claim bursts belonging to weather sensors, reporting no integrity check as they do so. Not verified: the corpus has no capture of one of these remotes that rtl_433 itself reads as more than an unknown code |
| KeeLoq (HCS200/HCS301) | 433.92 MHz | OOK PWM, 3 × 400 us per bit | 8 kHz | off air | a remote on 433.889 MHz pressed every few seconds, its burst in the decoder's test | Microchip's rolling-code encoder inside most gate, garage and car remotes that are not fixed-code: twelve preamble pulses, a 4 ms header, then 66 bits least significant bit first, a 32-bit hopping code that is ciphertext and changes every press, a 28-bit serial, four button bits, a low-battery flag and a repeat flag. Nothing can be checked, so the frame's shape is the evidence: exactly 66 bits on a row of their own behind a row of ones, which noise and other protocols do not fall into. The hopping code is reported as it arrived; decrypting it needs the manufacturer's key |
| Wireless M-Bus (EN 13757-4) | 868.95 MHz | 2-FSK 100 kchip/s, modes T and C | 250 kHz | off air | four meters from rtl_433's corpus, mode T | Utility meters: a Diehl and a Techem water meter, a BMeters water meter and an Itron component behind a repeater. Mode T spreads bytes over the 3-of-6 code, mode C sends them raw; both frame in blocks of at most sixteen bytes under a CRC-16, so a frame that passes is a frame. What reports without the key is who sent it and what it is, the manufacturer, meter number, version and type, since a utility's readings are AES-encrypted with a key it holds. Mode C's frame layout is decoded and its CRC checked, but the C recordings do not yet demodulate here and mode S is unhandled: both wait on a slicer this demodulator lacks |
| Chamberlain / Security+ 1.0 and 2.0 | 310/315/390 MHz | OOK PWM | 31 kHz | table | table | Rolling code: readable, not cloneable |
| Somfy RTS | 433.42 MHz | OOK Manchester 604 us | 31 kHz | done | table | Rolling code: readable, not cloneable. The sync word lives in the half-symbol stream and its odd length breaks naive pairing, so the decoder searches the raw halves for the sync and only then pairs, the way rtl_433 does. 56 bits, descrambled by XOR with the previous byte, guarded by a nibble-XOR checksum |
| KeeLoq, FAAC SLH, Star Line | 433.42/433.92 MHz | OOK PWM/Manchester | 31 kHz | table | table | Frames read fine; the payload is encrypted, so a replay is all a transmitter can do with one. No captures yet, so these wait on real RF before being ported rather than shipping an unverifiable decoder |
| Acurite 609TXC, 592TXR tower | 433.92 MHz | OOK PPM/PWM | 31 kHz | done | table | Checksum, and per-byte parity on the tower family. The 609's sum is eight bits over four bytes, weak enough that the sanity rules around it matter as much: it claimed an X10 remote's burst as a sensor reading 14.3 C until a zero id was refused |
| LaCrosse TX141TH-Bv2 | 433.92 MHz | OOK PWM | 31 kHz | done | table | LFSR digest, not a CRC |
| LaCrosse TX29-IT, TX35DTH-IT | 868.24 MHz | FSK NRZ 55/105 us | 125 kHz | done | table | Sync word 0x2dd4, CRC8, BCD temperature. A frame ending in zero bits ends with the carrier already off, so the tail is padded with the zeros silence stands for and the CRC checked across the padding |
| Nexus, FreeTec, Solight, TFA 30.3209 | 433.92 MHz | OOK PPM | 31 kHz | done | table | No checksum: one constant nibble and rtl_433's sanity rules |
| Rubicson, TFA 30.3197, inFactory PT-310 | 433.92 MHz | OOK PPM | 31 kHz | done | table | CRC8 over a nibble-padded frame. Shares its layout with Nexus, which defers to it |
| Bresser Thermo-/Hygro 3CH, Renkforce DM-7511 | 433.92 MHz | OOK PWM | 31 kHz | done | table | Additive checksum. Measures in Fahrenheit, reported in Celsius. The DM-7511 sends a 1012 us preamble where Bresser publishes 750, which is why an over-long mark is read as a row start rather than matched against a published width |
| Globaltronics GT-WT-02 (Aldi) | 433.92 MHz | OOK PPM, ms symbols | 31 kHz | done | table | Nibble-sum checksum, LL/HH humidity sentinels |
| Globaltronics GT-WT-03 (Aldi, Lidl) | 433.92 MHz | OOK PWM | 31 kHz | done | table | Rolling-key checksum, neither a CRC nor a sum |
| Oregon Scientific v2.1: THGR122N, THN132N, THN129, RTGN318, RTHN129 | 433.92 MHz | OOK Manchester 488 us | 31 kHz | done | table | Every bit is sent twice, inverted the second time, on top of the Manchester coding, so the sliced stream is complementary pairs. Nibbles arrive bit-reversed and values are BCD. Eight bit nibble-sum checksum, starting at a nibble that differs per model |
| Oregon Scientific v3: THGR810, THN802, WGR800 | 433.92 MHz | OOK Manchester 488 us | 31 kHz | done | table | Same payload layout as v2.1 without the doubling. The WGR800 reports wind rather than temperature |
| Acurite 606TX, Technoline TX960 | 433.92 MHz | OOK PPM 2/4 ms | 31 kHz | done | table | An LFSR digest rather than the sum the rest of the family uses. Its symbols are within a quarter of the GT-WT-02's, so the two slice the same way and only the checksums tell them apart |
| Acurite 986 fridge and freezer probe | 433.92 MHz | OOK PPM 520/880 us | 31 kHz | synthetic | table | Sends least significant bit first, with an LSB-first CRC8. Not verified off air: the marks are 220 us and the envelope detector's estimator runs on a 500 us time constant, so it merges them. A protocol needing a faster tracker is a chain parameter rather than a new decoder, but until that is wired up this one is checked against built frames only |
| Acurite Iris 5-in-1, Notos 3-in-1 | 433.92 MHz | OOK PWM | 31 kHz | done | table | The tower sensor's frame one byte longer, with the message type saying which readings it carries. The 5n1 alternates wind, direction and rain with wind, temperature and humidity, so a full picture takes two transmissions. Each repeat is numbered, which is why no two copies in a burst are identical |
| Ambient Weather, other Oregon Scientific, other Acurite (Atlas, 6045M lightning, 899 rain, 515 fridge) | 433.92/915 MHz | OOK PWM/Manchester | 31 kHz | table | table | Several families each, all timing tables, and the Acurite ones share the frame the tower and the 5n1 already use |
| Schrader MRXGG4 tyre sensor | 315/433.92 MHz | OOK Manchester 120 us | 31 kHz | done | table | CRC8 over eight bytes, plus a constant preamble nibble. 28 bit id, pressure and temperature. The id is fixed for the life of the sensor and four of them travel together, which is what makes a wheel worth logging |
| Toyota / Pacific PMV-C210 tyre sensor | 315/433.92 MHz | FSK differential Manchester 52 us | 125 kHz | done | table | CRC8, and the pressure sent twice with the second copy inverted. Also fitted by TRW to other makes |
| Other TPMS (Renault, Citroen, Ford, Jansite, Steelmate) | 315/433.92 MHz | OOK/FSK Manchester | 31-125 kHz | table | table | Bursty, short, CRC8. Sensors report on a timer, so a receiver waits minutes per wheel |
| Honeywell / Ademco door and window sensors, 2Gig DW10 and DW11, RE208, 2GIG-GB1 | 345 MHz | OOK Manchester 136 us | 31 kHz | done | table | CRC16, with the polynomial chosen by the channel field. Reports the serial engraved on the sensor, whether the contact is open, whether the case has been opened and whether the battery is low, all unencrypted |
| Interlogix / GE / UTC security sensors | 319.5 MHz | OOK PPM | 31 kHz | table | table | Two parity bits and a device-type enum are the whole integrity check, so it needs the corroboration rules the checksum-free remotes use |
| EnOcean | 868.3 MHz | ASK | 31 kHz | table | table | Self-powered switches |
| Itron / ERT smart meters | 902-928 MHz | OOK/FSK Manchester | 125 kHz | table | table | The rtlamr target |
| X10 RF | 310/433.92 MHz | OOK | 31 kHz | done | table | House code, unit and state, guarded by parity |
| Homematic | 868.3 MHz | GFSK 10 kbps | 125 kHz | framing | mod | Sync word plus whitening |
| Radiosondes (RS41, DFM, M10) | 400-406 MHz | GFSK 4800 bps | 125 kHz | framing | mod | Reed-Solomon, and a GPS position worth having |
| nRF24 ShockBurst | 2.4 GHz | GFSK 1-2 Mbps | 2 MHz | demod | mod | HackRF only. Flipper does this with a separate module |

## Utility metering and home automation

| Protocol | Where | Modulation | Width | RX | TX | Notes |
|---|---|---|---|---|---|---|
| Wireless M-Bus mode T | 868.95 MHz | 2-FSK 100 kbps, 3-of-6 | 125 kHz | framing | table | Very common on 868. Block CRCs, payloads often encrypted |
| Wireless M-Bus mode S | 868.3 MHz | 2-FSK 32.768 kbps, Manchester | 125 kHz | framing | table | |
| Wireless M-Bus mode C | 868.95 MHz | 2-FSK 100 kbps NRZ | 125 kHz | framing | table | |
| Wireless M-Bus mode N | 169 MHz | 4-GFSK 2.4/4.8 kbps | 31 kHz | demod | mod | Four levels, so the two-level slicer does not apply |
| Z-Wave R1 | 868.42/908.42 MHz | FSK 9.6 kbps, Manchester | 125 kHz | framing | table | Preamble, sync byte, checksum |
| Z-Wave R2/R3 | 868.42/908.42 MHz | FSK 40/100 kbps | 125 kHz | framing | table | |
| Zigbee / 802.15.4 sub-GHz | 868/915 MHz | BPSK DSSS | 125 kHz | demod | mod | Needs a despreader |
| Zigbee / 802.15.4 | 2.4 GHz | O-QPSK DSSS 250 kbps | 2 MHz | demod | mod | HackRF only |
| Bluetooth LE advertising | 2.4 GHz | GFSK 1 Mbps | 2 MHz | demod | mod | Hopping and whitening. HackRF only |

## LPWAN

| Protocol | Where | Modulation | Width | RX | TX | Notes |
|---|---|---|---|---|---|---|
| LoRa | 433/868/915 MHz | CSS chirp SF7-12 | 125-500 kHz | done | mod | `dsp::lora` dechirps and `decode::lora` reads the frame: Gray, diagonal deinterleave, Hamming, dewhitening, header checksum and payload CRC. `LoraNode` places it on any source the width of a LoRa channel and finds the spreading factor by trying, since dechirping at the wrong one gives no peak. Verified against two off-air Meshtastic transmissions at SF11 over 250 kHz, from different nodes 128 seconds apart, both giving a valid header checksum and the transmitter's own payload CRC. That is a different kind of evidence from the rtl_433 corpus and not a weaker one: the check comes from the transmitter rather than from a second decoder |
| LoRaWAN | as LoRa | as LoRa | 125-500 kHz | framing | mod | The PHY is read; what is missing is the MAC layout on top of it. Payloads are AES encrypted, the metadata is still worth logging |
| Meshtastic | 433/868/915 MHz | LoRa | 250 kHz | done | mod | The 0x2B sync word names it and the sixteen byte packet header is read: who transmitted, who for, the packet id, and how many hops it has left of how many it started with. The payload behind that is AES encrypted with the channel key, so it is reported as bytes |
| Sigfox uplink | 868.13 MHz | DBPSK 100 bps (600 US) | 100 Hz | demod | mod | Ultra narrowband, coherent detection, very narrow channel |
| Sigfox downlink | 869.525 MHz | GFSK 600 bps | 31 kHz | framing | mod | |

## Aviation

| Protocol | Where | Modulation | Width | RX | TX | Notes |
|---|---|---|---|---|---|---|
| ADS-B 1090ES (Mode S) | 1090 MHz | PPM 1 Mbit/s | 2 MHz | done | mod | Own demodulator in `dsp::modes`, frames in `decode::adsb`. Verified against dump1090-rb over a shared recording: 27 of its 40 frames, no frame it did not also see |
| Mode A/C | 1090 MHz | pulse pairs | 2 MHz | chain | mod | Same chain as Mode S once it exists |
| UAT | 978 MHz | CPFSK 1.041667 Mbps | 2 MHz | chain | mod | US general aviation, Reed-Solomon |
| ACARS | 129-137 MHz | AM, MSK 2400 bps | 25 kHz | framing | mod | Rides on an AM channel: envelope path plus MSK bit recovery |
| VDL Mode 2 | 136 MHz | D8PSK 31.5 kbps | 25 kHz | demod | mod | Differential 8-PSK, so a coherent chain |
| VOR / ILS | 108-118 MHz | AM with 30 Hz subcarriers | 25 kHz | framing | mod | SDRangel decodes bearing from these; the maths is small |
| HFDL | 2-22 MHz | PSK | 3 kHz | demod | mod | Needs HF hardware too |

## Maritime

| Protocol | Where | Modulation | Width | RX | TX | Notes |
|---|---|---|---|---|---|---|
| AIS | 161.975/162.025 MHz | GMSK 9600 bps | 25 kHz | framing | mod | NRZI, HDLC bit stuffing, CRC16. The discriminator output is usable directly, so this is the cheapest of the "real" protocols |
| DSC | 156.525 MHz, HF | FSK 1200 baud | 25 kHz | framing | table | Distress calls, so anything transmitted here reaches a coastguard watch room |
| NAVTEX | 518 kHz | FSK 100 baud SITOR-B | 1 kHz | chain | table | Needs HF hardware |

## Paging

| Protocol | Where | Modulation | Width | RX | TX | Notes |
|---|---|---|---|---|---|---|
| POCSAG | 137-174, 450-470, 929 MHz | 2-FSK 512/1200/2400 bps | 25 kHz | synthetic | table | `dsp::pocsag` and `decode::pocsag`. All three bit rates are demodulated at once, since nothing in the signal says which is in use, and both polarities are searched for. BCH(31,21) corrects up to two errors per codeword. The message layer is checked against a published off-air capture decoded by POC32; the demodulator in front of it has not met real RF. Amateur DAPNET networks run the same protocol |
| FLEX | 929-932 MHz | 2/4-FSK 1600-6400 bps | 25 kHz | demod | mod | The four-level front end reads the symbols; what is missing is the level to bit mapping, the sync words and the framing |
| ERMES | 169 MHz | 4-FSK 6250 bps | 25 kHz | demod | mod | As FLEX |

Pager traffic is unencrypted and often carries medical and personal detail.
Worth knowing before pointing a decoder at it and logging the output: the
packet log keeps what the demodulator produced, and for POCSAG that is
codewords the message text can be read back out of.

## Land mobile and digital voice

| Protocol | Where | Modulation | Width | RX | TX | Notes |
|---|---|---|---|---|---|---|
| DMR | 136-174, 400-470 MHz | 4-FSK 4800 baud | 12.5 kHz | demod | mod | Four-level slicer, then AMBE, which is patent encumbered |
| P25 phase 1 | 700-900 MHz | C4FM | 12.5 kHz | demod | mod | As DMR, plus IMBE |
| NXDN, dPMR | 400-470 MHz | 4-FSK | 6.25/12.5 kHz | demod | mod | |
| M17 | amateur bands | 4-FSK 4800 baud | 12.5 kHz | synthetic | mod | Link setup, stream and packet frames, in `dsp::m17` and `decode::m17`. Reports who called whom, the channel access number, whether the stream is encrypted or signed, and the position, text or repeater callsigns the metadata carries. Packet mode is reassembled and CRC checked, so an SMS packet reports its message. A receiver that missed the link setup rebuilds it from six stream frames through the link information channel, which is what that channel is for. Frames are verified against the M17 project's own C library symbol for symbol, in `the_frames_match_the_reference_implementation`, which is a stronger check than **synthetic** usually means: an encoder and a decoder written together agree with each other whatever they both misread, and this one agrees with somebody else's. The demodulator in front of them has met synthetic RF and not yet a radio, which is why the status is not **done**. Voice payloads are carried but not decoded: Codec 2 at 3200 bits per second is the last piece missing, and unlike AMBE or IMBE it is free to implement |
| TETRA | 380-400, 410-430 MHz | pi/4-DQPSK 36 kbps | 25 kHz | partial | mod | Control channels read off the air: `dsp::tetra` demodulates by differential detection, resynchronising timing and carrier on every burst's training sequence, then runs the downlink coding stack (scrambling, interleaving, RCPC Viterbi, CRC, and the (30,14) block code of the access assign field), and `decode::tetra` reads the PDUs. A carrier is logged as who it is (SYNC and SYSINFO: MCC, MNC, colour code, location area, main carrier) and what it knows (D-NWRK-BROADCAST: the neighbouring cells by carrier and location area). Signalling to a party is read from the MAC header even when enciphered: the address, the encryption mode, any usage marker and channel allocation. In clear, the CMCE call control PDUs (D-SETUP, D-CONNECT, D-TX GRANTED, D-RELEASE and the rest) give the parties, the call identifier and group or private, and D-SDS-DATA gives text. The access assign field of every slot is followed for traffic, so a call becomes a start row and an end row with its airtime, by usage marker and by the party the marker was given to. Verified against a recorded Irish downlink for everything but the clear-mode PDUs, which that network encrypts; those are tested on synthetic bits. Not read: traffic itself, and voice, which is ACELP and patent encumbered the way AMBE is |
| FM with CTCSS/DCS | any | FM plus subaudible tone | 12.5 kHz | table | mod | Trivial next to the rest: a Goertzel on the discriminator output |

## Broadcast

| Protocol | Where | Modulation | Width | RX | TX | Notes |
|---|---|---|---|---|---|---|
| FM stereo | 87.5-108 MHz | FM, 38 kHz subcarrier | 200 kHz | done | mod | |
| RDS | 87.5-108 MHz | 57 kHz BPSK 1187.5 bps | 200 kHz | done | mod | PortaPack transmits RDS; the encoder is small once the modulator exists |
| AM broadcast | 530-1700 kHz | AM | 10 kHz | done | mod | Envelope detector; the band itself needs HF hardware |
| DAB / DAB+ | 174-240 MHz | OFDM DQPSK | 1.536 MHz | chain | chain | Viterbi plus Reed-Solomon after the OFDM |
| DVB-T | 470-790 MHz | OFDM | 8 MHz | chain | chain | HackRF only, and a large amount of machinery |
| DRM | HF | OFDM | 10 kHz | chain | chain | |
| HD Radio (IBOC) | 88-108 MHz | OFDM sidebands | 400 kHz | chain | chain | |

## Satellite

| Protocol | Where | Modulation | Width | RX | TX | Notes |
|---|---|---|---|---|---|---|
| NOAA APT | 137 MHz | FM, 2.4 kHz AM subcarrier | 40 kHz | framing | mod | An image rather than packets: the demodulation is easy, the presentation is the work |
| Meteor-M LRPT | 137.9 MHz | QPSK 72 kbps | 120 kHz | demod | mod | Viterbi plus Reed-Solomon |
| Iridium | 1616-1626 MHz | QPSK 25 kbaud bursts | 500 kHz | demod | mod | Bursty, needs good timing |
| Inmarsat STD-C | 1537 MHz | BPSK 1200 bps | 10 kHz | demod | mod | Needs an L-band antenna and an LNA |
| GOES HRIT | 1694 MHz | BPSK 927 kbps | 2 MHz | chain | mod | |
| GPS L1 | 1575.42 MHz | BPSK DSSS | 2 MHz | demod | mod | Receiving needs a despreader. PortaPack simulates GPS, which is the usual reason to transmit it |

## Amateur

| Protocol | Where | Modulation | Width | RX | TX | Notes |
|---|---|---|---|---|---|---|
| APRS / AX.25 1200 | 144.39/144.8 MHz | AFSK over FM | 12.5 kHz | framing | mod | Discriminator, Bell 202 tones, HDLC, CRC16. A good first framing target |
| Packet 9600 (G3RUH) | 144-440 MHz | direct FSK 9600 | 25 kHz | framing | mod | Scrambled NRZI |
| Morse (CW) | any | OOK | 500 Hz | table | table | The envelope path already produces the timings, and keying a carrier is the simplest transmit case there is |
| RTTY | HF, VHF | FSK 45.45 baud | 1 kHz | framing | mod | Baudot, and the same two-tone shape as everything else here |
| PSK31 | HF | BPSK 31.25 baud | 100 Hz | demod | mod | Varicode, coherent |
| SSTV | HF, 144 MHz | FM subcarrier | 3 kHz | framing | mod | Image, like APT |
| FT8, WSPR, JS8 | HF, 6 m | narrow MFSK | 50 Hz-3 kHz | demod | mod | Long coherent integration and LDPC. A different kind of receiver |
| DTMF and tone remote | any | audio tones over FM | 12.5 kHz | table | mod | Goertzel pair |

## Cellular

| Protocol | Where | Modulation | Width | RX | TX | Notes |
|---|---|---|---|---|---|---|
| GSM downlink control | 900/1800 MHz | GMSK 270.833 kbps | 200 kHz | demod | mod | Broadcast channels carry cell identity in the clear; traffic uses A5 ciphers, so only the control plane is decodable without attacking them |
| LTE / 5G | various | OFDM | 1.4-100 MHz | chain | chain | Cell search and MIB decode is possible in principle; past that it is a stack, not a decoder |

## Time and beacons

| Protocol | Where | Modulation | Width | RX | TX | Notes |
|---|---|---|---|---|---|---|
| DCF77 | 77.5 kHz | AM plus phase modulation | 100 Hz | chain | mod | Needs VLF hardware |
| MSF, WWVB | 60 kHz | AM | 100 Hz | chain | mod | As DCF77 |
| NDB beacons | 190-535 kHz | keyed carrier | 1 kHz | chain | mod | |

## Transmit

Nothing transmits yet, and the gap is structural rather than protocol by
protocol. Four things are missing:

1. **A transmitting device.** `common::device::Device` has `start_rx` and no
   counterpart. It needs `start_tx` returning a sink, and the `hackrf` crate
   needs the transmit half of the USB protocol and its gain controls. The
   RTL-SDR cannot transmit at all, so the trait has to make that a capability
   rather than an assumption.

2. **Encoders, which are the slicers backwards.** `decode::slicer::slice`
   turns a pulse train into bits under a timing table; the same table turns
   bits back into a pulse train. Every protocol that has a table gets an
   encoder nearly for free, which is why the transmit column above mostly
   mirrors the receive one.

3. **Modulator nodes.** A `Package` of mark/gap timings becomes an envelope,
   and an envelope becomes IQ. Two nodes cover most of this list: OOK keying
   and two-level FSK. Both need edge shaping rather than hard switching, or
   the transmission splatters across the band: a raised-cosine ramp of a few
   microseconds is the difference between a legal signal and interference.

4. **Scheduling and limits.** Transmission is time critical in a way reception
   is not, so the graph needs to produce samples ahead of a deadline rather
   than in response to input. ISM bands also carry duty cycle limits (1% in
   parts of 868 MHz), and the transmitter should enforce them rather than
   leave it to the operator to remember.

A sensible first target is Morse keying into a dummy load: it exercises the
device, the modulator and the scheduler with no framing at all, and a dummy
load keeps the first attempt off the air while it is still wrong.

## Suggested order

Cheapest first, by value per unit of work:

1. **PT2262/EV1527 fixed-code remotes.** One timing table each, the most
   common thing on 433.92 MHz, and the best test of the unknown-burst
   analyser, which should already be reading them as PWM.
2. **The rtl_433 weather station families.** Same shape as the Fine Offset
   decoder that exists: a parser and a CRC each.
3. **TPMS.** Short frames, plenty of them near any road, and the OOK and FSK
   variants exercise both banks.
4. **AIS.** The first protocol needing real framing (NRZI, bit stuffing,
   CRC16) but no new demodulator, and the results are immediately legible.
5. ~~**POCSAG.**~~ Done. The BCH(31,21) correction it added is reusable for
   FLEX, ERMES and the radiosondes.
6. **Wireless M-Bus T and C.** Common on 868 in Europe, and the sync word plus
   block CRC work carries over to Z-Wave and Homematic.
7. **The transmit path, ending in Morse.** Device, encoder, modulator,
   scheduler, proven end to end on the simplest possible protocol.
8. **LoRaWAN.** The MAC layer on top of the LoRa PHY that is now read:
   join requests, device addresses and frame counters, all of which are in
   the clear even though the payload is not.
9. **ADS-B.** Needs its own wideband chain rather than a bank channel, so it
   is a structural change: a scanner tier at 2 MS/s.

Everything below that (OFDM broadcast, trunked voice, cellular) is a project
each rather than a decoder each, and should be judged on its own.
