# Protocols

The target is the union of what rtl_433, a Flipper Zero, a PortaPack running
Mayhem and SDRangel can do, in one receiver, and transmit for the same set,
plus the drone family under [Drones](#drones), which none of them reads and
which announces itself in the clear.
This file lists those protocols, what each one costs to add, and which
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
| discriminator, `dsp::c4fm` | 4-FSK level symbols | yes | no |
| discriminator plus sync correlation, `dsp::m17` | M17 frames | yes | no |
| discriminator plus three bit clocks, `dsp::pocsag` | NRZ FSK bits at 512, 1200 or 2400 | yes | no |
| discriminator plus Bell 202, `dsp::afsk` | 1200 baud bits over FM | yes | no |
| GMSK with timing recovery, `dsp::ais` | 9600 baud bits | yes | no |
| equalised GMSK on a training sequence, `dsp::gsm` | GSM burst bits at 270.833 kbaud | yes | no |
| 100 kchip/s FSK, 3-of-6 and NRZ, `dsp::wmbus` | meter frame bits | yes | no |
| pulse-position at 1 Mbit/s, `dsp::modes` | Mode S frames | yes | no |
| differential PSK on a training sequence, `dsp::tetra` | soft symbols | yes | no |
| coherent PSK with carrier recovery | soft symbols | no | no |
| chirp correlator (dechirp then FFT), `dsp::lora` | LoRa symbols | yes | no |
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
longer than one frame. DMR is read that way now, in `nodes::dmr_nodes`. P25 and
NXDN are framed the same and would be read the same; the vocoder is no longer
what stops them, since `crates/mbe` decodes both IMBE and AMBE behind the
`ambe` feature.

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
the six by name. MSK routes to the FSK front end, and a chirp verdict is what
places LoRa on a source. The rest of what it can name (BPSK, QPSK, DQPSK, OFDM,
DSSS, noise-like, bare carrier) are labels on the burst rather than routes:
DQPSK has a demodulator in `dsp::tetra`, which the router does not dispatch to,
and the others have none.

A protocol whose symbols reach the mark/gap layer costs a timing table and a
payload parser, and nothing else: the slicers (PWM, PPM, Manchester, NRZ), the
CRC helpers, the unknown-burst analyser and the packet list already exist.
Everything else costs a demodulator first.

Transmit inverts the same layers, and the bottom of that stack is built: see
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
that measures about 40x real time on a 48 core machine, in
`radio::tests::the_scanner_keeps_up_with_the_stream`. The four bank tiers
measured 6.1x when they were the default, a number that now lives only as the
comment on `scanners::DEFAULT_WIDTHS`.
Scored on rtl_433's corpus by `crates/nodes/tests/source_corpus.rs`, the node
recovers 53 of the 57 reference decodes against the tiers' 48, and loses
ground on no capture. The tiers remain a front end a scanner block can ask
for as `banks`, for that comparison.

The third is hardware.

| Radio | Range | Rate | Direction |
|---|---|---|---|
| RTL-SDR | 24-1766 MHz | 2.4 MS/s | receive only |
| HackRF One | 1 MHz-6 GHz | 20 MS/s | half duplex, transmit and receive |
| PortaPack | a HackRF with a screen | as HackRF | as HackRF |
| LimeSDR USB / Mini | 100 kHz-3.8 GHz | 61.44 MS/s on USB3 | full duplex, two receive and two transmit channels on a USB board |
| iqstream server | whatever feeds it | whatever feeds it | receive only, and its tuning is a reading rather than a setting |

Out of scope whatever the ambition: a Flipper's 125 kHz RFID, its 13.56 MHz
NFC, its infrared and its iButton are near-field or optical, not radio an SDR
can reach. That rules out most of the shelf labels a Flipper can talk to:
TagTinker and PriceIR drive Pricer and SES tags over the infrared blaster, and
Momentum's sub-GHz protocol list has no shelf label in it at all.

## Status codes

Receive:

- **done**: decoding now, verified against a recording another implementation
  also decoded
- **off air**: decoding now, verified against a recording this project made,
  where the evidence is what the transmission itself says (a callsign, a CRC,
  a signature) rather than a second decoder's opinion
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

Known gaps, listed in `KNOWN_GAPS` in that test so that closing one fails until
the note is removed:

- The Acurite 5n1 numbers its three repeats and rtl_433 prints each of them. A
  protocol here returns the first frame it finds in a package, so only the
  first sequence number is reported. The reading is the same in all three.
- One Honeywell 5816 capture was recorded at close range with the gain control
  never settling, so the burst arrives saturated and the envelope reads nearly
  all of it as one mark. The same family decodes from the 2Gig and RE208
  recordings.

Not in that list but worth knowing: the THR228N is reported as a THN132N. They
share a sensor id and a frame layout, and rtl_433 tells them apart by message
length, which is not measurable here: a burst runs one copy straight into the
preamble of the next, and that preamble unpacks as valid Manchester pairs, so
the frame never ends where the transmitter stopped. Both sensors report the
same fields.

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
| KeeLoq (HCS200/HCS301) | 433.92 MHz | OOK PWM, 3 × 400 us per bit | 8 kHz | off air | table | Verified against a remote on 433.889 MHz pressed every few seconds, its burst kept in the decoder's test. Microchip's rolling-code encoder inside most gate, garage and car remotes that are not fixed-code: twelve preamble pulses, a 4 ms header, then 66 bits least significant bit first, a 32-bit hopping code that is ciphertext and changes every press, a 28-bit serial, four button bits, a low-battery flag and a repeat flag. Nothing can be checked, so the frame's shape is the evidence: exactly 66 bits on a row of their own behind a row of ones, which noise and other protocols do not fall into. The hopping code is reported as it arrived; decrypting it needs the manufacturer's key |
| Wireless M-Bus (EN 13757-4) | 868.95 MHz | 2-FSK 100 kchip/s, modes T and C | 250 kHz | done | table | Verified against seven meters from rtl_433's corpus: four in mode T, a Diehl and a Techem water meter, a BMeters water meter and an Itron component behind a repeater, and three in mode C from two Kamstrup water meters. Mode T spreads bytes over the 3-of-6 code, mode C sends them raw; both frame in blocks of at most sixteen bytes under a CRC-16, so a frame that passes is a frame. What reports without the key is who sent it and what it is, the manufacturer, meter number, version and type, since a utility's readings are AES-encrypted with a key it holds. Mode S is unhandled: nothing has recorded it |
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
| Unidentified 868 MHz alarm link | 868.1/868.5 MHz | 2-FSK 19.6 kbaud NRZ | 125 kHz | off air | no | Sync `47 4F`, a 16-bit id, then 17 to 19 bytes of block-encrypted body with no integrity check outside the cipher. Heard continuously in Ireland, a hub and a repeater relaying one another. The framing is read; nothing inside it is. Nobody has published the sync word, so the name says what was measured rather than whose it is |
| Homematic | 868.3 MHz | GFSK 10 kbps | 125 kHz | framing | mod | Sync word plus whitening |
| Radiosondes (RS41, DFM, M10) | 400-406 MHz | GFSK 4800 bps | 125 kHz | framing | mod | Reed-Solomon, and a GPS position worth having |
| nRF24 ShockBurst | 2.4 GHz | GFSK 1-2 Mbps | 2 MHz | demod | mod | HackRF only. Flipper does this with a separate module |

## Displays: shelf labels, price signs and passenger information

Unrelated systems that happen to share a purpose. Retailers' own tags are the
point here, so the link layer and the payload are listed separately: the
framing is published silicon and reads today, while what a stock tag says
inside it is the vendor's and mostly is not.

| Protocol | Where | Modulation | Width | RX | TX | Notes |
|---|---|---|---|---|---|---|
| Shelf labels, sub-GHz link layer | 863.999-869.034, 903-923 MHz | GFSK 38.38 kbps (20.6 kHz deviation) or 249.94 kbps (165 kHz deviation) | 125 or 650 kHz | synthetic | table | Chroma and the Solum tags behind them run a CC1110 or CC1310, so the link layer is TI's packet engine: preamble, sync, length byte, CRC-16, PN9 whitening, all of which `decode::whiten` already had. `decode::protocols::esl` reads that and reports the frame whoever built it, because the framing is a data sheet and a shop's own tags obey it. The sync word is not matched against a constant, since the stock configuration and the replacement firmware use different ones and a tag on a shelf can be running either: the frame is taken from the end of the preamble and the CRC-16 decides. The 250 kbps variant is the widest thing here an RTL-SDR can still reach, at 9.6 samples per symbol on 2.4 MS/s |
| Stock Solum payload | as above | as above | as above | bytes only | no | What a retailer's own tags actually say inside that frame. Nobody has published it: Dmitry Grinberg's teardown found self-contained firmware with no 802.15.4 stack and a layout of its own. Frames report as `oepl=false` with their length and bytes, which is the starting point for reversing it, not a decode |
| OpenEPaperLink payload | as above | as above | as above | synthetic | table | The replacement firmware's own protocol, parsed when PAN id 0x4447 and a packet type from `oepl-proto.h` say so: tag MAC, battery voltage, temperature, firmware and channel on a check-in, and the size of the image waiting plus the next check-in time on the access point's reply. Checked against frames built from that header only. Useful for a bench tag, not for a shop |
| Hanshow Stellar heartbeat | 2.401-2.480 GHz, 500 kHz channels | GFSK, 500 kbps up, 100 kbps down | 2 MHz | synthetic | mod | A stock shop protocol, and the best documented one. `decode::protocols::hanshow` reads the heartbeat a tag sends about every three minutes: sync `52 56 78 53`, a control byte, then the ESL id, the wakeup, group and data channels, the netmask, battery level, encryption flag, panel temperature and display id. Layout and field names from `dustybee/HanshowESL`, recovered off air at 2.401 GHz. The CRC's polynomial and span were not pinned down there, so nothing is checked and the corroboration is the sync, the control byte and the exact length it implies. The stream is inverted against this slicer's convention and is searched for both ways round. HackRF only |
| Solum ZBS243, Newton M2/M3 | 2.4 GHz, channels 11, 15, 20, 25, 26, 27 | O-QPSK DSSS 250 kbps | 2 MHz | demod | mod | An 802.15.4 PHY carrying a proprietary payload. Needs the despreader. HackRF only |
| SES-imagotag Vusion | 2.4 GHz, 11 channels | proprietary, CC2510 | 2 MHz | demod | mod | Another TI packet engine, so the framing above applies once there is a 2.4 GHz chain to run it on. The chip has hardware AES-128 and the brochures claim encryption, so expect the payload to be ciphertext |
| Gicisky, PICKSMART, ATC shelf labels | 2.4 GHz | BLE advertising, GFSK 1 Mbps | 2 MHz | framing | mod | The BLE front end reads them now; what is missing is the layout inside the manufacturer data, which is each vendor's own. An ATC tag broadcasting temperature and battery is a service data structure this reports as bytes |
| Pricer, SES-imagotag infrared, SES 38 kHz LF loop | optical, or a radiating cable | infrared, or LF induction | | out of scope | out of scope | Not radio an SDR can reach. This is what the Flipper apps talk to |
| Petrol forecourt price signs | 433.92 MHz typically | OOK or 2-FSK, one way, address plus BCD digits | 31 kHz | table | table | A controller at the till repeats the price to each sign continuously. The shape fits the existing pulse front end and costs a timing table, but no layout is published for any of them, so it is capture-driven work: a recording during a price change, or one of the receivers on a bench |
| Axentia iBus bus stop displays | FM broadcast band | DARC, 76 kHz subcarrier, LMSK 16 kbps | 200 kHz | demod | mod | Not sub-GHz at all. The displays are fed over a data channel on an ordinary FM station, above the RDS subcarrier this project already demodulates in `dsp::rds`, so the front end is a second subcarrier on a chain that exists rather than a new one. Decoded off air by windytan in Helsinki and Apollo-NG in Munich. The DARC layers are ETSI EN 300 751; the iBus payload above them is not published |
| STP403 passenger information | 164 and 468.4875 MHz | FFSK, 8 kHz channel | 25 kHz | framing | mod | The French pattern instead: an NFM channel and a bit slicer. Layout unpublished |

## Utility metering and home automation

| Protocol | Where | Modulation | Width | RX | TX | Notes |
|---|---|---|---|---|---|---|
| Wireless M-Bus mode T | 868.95 MHz | 2-FSK 100 kbps, 3-of-6 | 125 kHz | done | table | Very common on 868. `dsp::wmbus` demodulates it, `decode::wmbus` reads the blocks and their CRCs, and the auto node places the front end by bandwidth. Verified field for field against rtl_433's four meter recordings: manufacturer, meter number, version and type. The readings inside are AES encrypted with the utility's key |
| Wireless M-Bus mode S | 868.3 MHz | 2-FSK 32.768 kbps, Manchester | 125 kHz | demod | table | Nothing has recorded it here |
| Wireless M-Bus mode C | 868.95 MHz | 2-FSK 100 kbps NRZ | 125 kHz | done | table | Formats A and B both, verified against rtl_433's three Kamstrup recordings. The demodulator always could read them; what stopped it was the source detector, which reopened a long loud burst around its own splatter and dropped the front ends placed on it |
| Wireless M-Bus mode N | 169 MHz | 4-GFSK 2.4/4.8 kbps | 31 kHz | demod | mod | Four levels, so the two-level slicer does not apply |
| Z-Wave R1 | 868.42/908.42 MHz | FSK 9.6 kbps, Manchester | 125 kHz | framing | table | Preamble, sync byte, checksum |
| Z-Wave R2/R3 | 868.42/908.42 MHz | FSK 40/100 kbps | 125 kHz | framing | table | |
| Zigbee / 802.15.4 sub-GHz | 868/915 MHz | BPSK DSSS | 125 kHz | demod | mod | Needs a despreader |
| Zigbee / 802.15.4 | 2.4 GHz | O-QPSK DSSS 250 kbps | 2 MHz | demod | mod | HackRF only |
| Bluetooth LE advertising | 2402, 2426, 2480 MHz | GFSK 1 Mbps | 2 MHz | done | mod | `dsp::ble` reads one primary advertising channel and `decode::ble` parses the PDU: type, address and whether it rotates, the advertising data structures, the local name and the company identifier. The front end takes the tuner's frequency error out of each burst before slicing, which at 20 ppm is 50 kHz against a 250 kHz deviation and is the difference between every packet and none, correlates the access address at every sub-symbol offset, dewhitens with the channel index and accepts on the CRC-24. Read as microsecond pulse timings instead it loses about two packets in five, because one bit is one microsecond at this rate, so it does not go through `decode::slicer`. Verified against an off-air capture of channel 38 tuned onto the channel, so every packet is read across the tuner's own DC spike, checked against the addresses and company identifiers the devices transmitted. Data channels are not read: they hop, and their access address is negotiated in a connection request this never sees. Placed by the auto node across the whole span wherever an advertising channel is inside it, the way Mode S and AIS are, rather than by a scanner block: the channel is where the standard put it, and an 80 us advertisement from a device that may never repeat is not something a spectrogram opens a source around. Needs 4 MS/s, so HackRF only |

## LPWAN

| Protocol | Where | Modulation | Width | RX | TX | Notes |
|---|---|---|---|---|---|---|
| LoRa | 433/868/915 MHz, and 2.4 GHz | CSS chirp SF5-12 | 62.5-812.5 kHz | done | mod | `dsp::lora` dechirps and `decode::lora` reads the frame: Gray, diagonal deinterleave, Hamming, dewhitening, header checksum and payload CRC. A source in the 2.4 GHz band is read the other way up and at SF5 to SF8, because LoRa there is an SX128x, which swaps I and Q against the SX127x convention and uses only those factors; a demodulator built for 868 MHz finds nothing at all on 2.4. `LoraNode` is placed on a source once the burst front end has named a burst of it a chirp, fed the source's samples so far from that front end's ring, and finds the spreading factor by trying, since dechirping at the wrong one gives no peak. Verified against three off-air Meshtastic packets, two at SF11 over 250 kHz from the same node 128 seconds apart and a third tuned 525 kHz off channel at 2.4 MS/s, and against a MeshCore advert at SF8 over 62.5 kHz, each giving a valid header checksum and the transmitter's own payload CRC. That is a different kind of evidence from the rtl_433 corpus and not a weaker one: the check comes from the transmitter rather than from a second decoder |
| LoRaWAN | as LoRa | as LoRa | 125-500 kHz | synthetic | mod | `decode::lorawan` reads what is in the clear: a join request whole (JoinEUI, DevEUI, nonce), and a data frame's DevAddr, frame counter, port, ACK and ADR flags. A join accept is ciphertext, and so is `FRMPayload`, under a key per device |
| Meshtastic | 433/868/915 MHz | LoRa | 250 kHz | off air | mod | The 0x2B sync word names it and the sixteen byte packet header is read: who transmitted, who for, the packet id, and how many hops it has left of how many it started with. The payload is AES encrypted with the channel key. The default and public keys are built in and tried on every packet, and an operator can add more in the keys pane, so an ordinary LongFast message reads as its text; anything under a private key reports as bytes |
| MeshCore | 433/868/915 MHz | LoRa | 62.5-250 kHz | off air | mod | `decode::meshcore` reads the routing in the clear: the one byte header, whether the packet is flooding or routed, and the path of node hashes it has taken. An advert is not enciphered at all and carries the node's Ed25519 public key, signature, role, name and position, so a receiver learns the mesh from one packet. Verified off air against a node advert at SF8 over 62.5 kHz, signature checked |
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

## Drones

None of rtl_433, a Flipper, a PortaPack or SDRangel reads this family, so it is
the one place in this file where the target is somebody else's work rather
than a fourth copy of theirs: the Open Drone ID library, the RUB-SysSec
DroneID receiver, ExpressLRS's own source, and the reverse engineering of the
hobby control links that Deviation and MultiModule already carry.

What makes it worth the trouble is that a drone announces itself. Remote ID is
a legal requirement in the US and the EU and is transmitted in the clear, DJI
broadcasts the same information plus the operator's own position whether or
not Remote ID is on, and a control link that hops is still a fingerprint. The
cost divides on two lines. Anything carried on Bluetooth advertising is nearly
free, because `dsp::ble` is the front end and the payload is a published
structure. Anything on OFDM (DJI's own link, Wi-Fi Remote ID, every digital
video system) has no front end here at all, and 2.4 and 5.8 GHz mean HackRF or
LimeSDR throughout: an RTL-SDR reaches none of this except the 433 and 868/915
MHz control links.

Hopping is the second structural problem and it is not solved by a wider span.
ELRS at 500 Hz moves every 2 ms across most of a band, so a channel placed on
one frequency sees one packet in fifty. Reading a hopping link properly means
following the sequence, which is derived from the binding UID, so a receiver
that has not seen the bind either brute forces the sequence or reads the band
wide enough to catch every hop. Detection does not need any of that: a burst
pattern at a known rate on a known channel plan is a classification, and
saying "an ELRS transmitter at 500 Hz is up" is most of the operational value.

| Protocol | Where | Modulation | Width | RX | TX | Notes |
|---|---|---|---|---|---|---|
| Open Drone ID over Bluetooth legacy | 2402/2426/2480 MHz | GFSK 1 Mbps, BLE advertising | 2 MHz | off air | mod | ASTM F3411 and EN 4709-002, the same message set in both. A legacy advertisement carries service UUID 0xFFFA, AD type 0x16, application code 0x0D and one 25 byte message: basic id (serial or session id and UA type), location (position, altitude, speed, track, timestamp), self id, system (the operator's position and the area a swarm covers) and operator id. `dsp::ble` reads the advertisement and `decode::ble` hands over the AD structures, so this cost a payload parser and nothing else: `decode::odid` reads a single message and a message pack, and `nodes::ble_nodes` names the row for the aircraft rather than for a Bluetooth device when one is present. Verified off air against a Holybro RemoteID module in `testdata/fixtures.toml`, whose 28 messages in six seconds decode to the identity it was shipped with and to a location message that marks its position absent because it has no fix. Unauthenticated by design: anything received is what the transmitter chose to say, and a field the specification marks absent (a position of exactly 0, an altitude of -1000 m) is reported as absent rather than as a position in the Atlantic |
| Open Drone ID over Bluetooth 5 Long Range | 2402/2426/2480 MHz and the 37 data channels | GFSK 125 or 500 kbps, LE Coded PHY S=8 and S=2 | 2 MHz | off air | mod | The same messages as a message pack (type 0xF) on extended advertising, which regulators require alongside the legacy broadcast, so a receiver that reads only legacy still sees everything the aircraft says. `dsp::ble_coded` reads the PHY: the uncoded 80 symbol preamble, the rate 1/2 constraint length 4 convolutional code with a Viterbi over its eight states, the pattern mapper that makes eight symbols carry one bit at S=8, and the coding indicator that says which rate the body used. It runs as a second pass over a burst that held no uncoded packet, on the same symbols at the same rate, since even the access address is coded and the uncoded search cannot see it. Verified off air against the Holybro module in the same capture as the legacy row: six ADV_EXT_IND packets, CRC-24 checked. What those carry is a pointer rather than a payload, which is the specification working as intended: the message pack rides in an AUX_ADV_IND on a data channel chosen afresh each time, and `decode::ble` reports the pointer so a row says where the rest went instead of showing an empty advertisement. The packs themselves are read too, by parking on a stretch of data channels, which `BleConfig::data_channels` turns on: `odid_bt5lr_holybro_2474M_20000k.cs8` holds eleven of them at S=8 on channels 31 to 36, each carrying basic id, location, self id and operator id in one reception where the legacy transport spreads the same four over seconds |
| Open Drone ID over Wi-Fi Beacon and NAN | 2.4 and 5.8 GHz, 20 MHz channels | 802.11 OFDM | 20 MHz | chain | chain | Vendor specific element under OUI 6A:5C:35 in a beacon, or a NAN service discovery frame. The payload parser is shared with the Bluetooth rows; what is missing is an 802.11 receiver |
| DJI DroneID | 2.4 and 5.8 GHz | OFDM, LTE-like numerology, about 10 MHz occupied | 15.36 MS/s | demod | mod | Broadcast roughly twice a second by DJI aircraft independently of Remote ID, and it carries more: serial number, position, velocity, height, home position, device type and the operator's own position. Sent in the clear, which DJI described as encrypted until the NDSS 2023 paper showed it is not. The frame published there is nine OFDM symbols, two of them Zadoff-Chu sequences used for time and frequency correction, QPSK subcarriers, turbo coded and scrambled under a CRC. A working receiver exists to check against (`RUB-SysSec/DroneSecurity`), which is what makes this the most attackable of the OFDM entries despite being the most work |
| DJI OcuSync and Lightbridge | 2.4 and 5.8 GHz | OFDM, 10/20/40 MHz | 20 MHz | chain | chain | The control and video link itself, AES encrypted both ways. Nothing inside is readable, so the realistic product is detection and classification: occupied bandwidth, hop behaviour and the DroneID frames riding alongside |
| Digital video links: DJI O3/O4, Walksnail Avatar, HDZero | 5.65-5.95 GHz mostly | proprietary OFDM, 20 MHz and wider | 20 MHz | chain | chain | No layout is published for any of them. What a spectrum shows is a wide flat carrier keyed to the frame rate, which is enough to say a link is up and which system it is by width and duty cycle, and nothing beyond that without a large reversing effort. There is a Walksnail Avatar here, so its width, duty cycle and how the downlink and the uplink sit against each other can be measured rather than repeated from a forum post |
| Analogue video links | 5.65-5.95 GHz, also 1.2 and 2.4 GHz | FM, composite video (PAL or NTSC) | 20 MHz | off air | mod | Not a protocol: a camera's composite video frequency modulated onto a carrier, with no framing, addressing or integrity check anywhere. Nothing in the decoding is specific to a model aircraft: `dsp::video` separates sync, assembles fields and demodulates PAL colour off the burst, `nodes::VideoNode` is that on the graph (measuring the standard from the line period rather than being told it) publishing whole fields on `PortKind::Video`, `app::videobus` is where they all end up, holding the last field so a view polling at the screen's rate finds one (a field arrives on one block in fifty, and publishing it for that block alone left the pane empty while fields were arriving) and dropping it after half a second so a still of a departed transmitter is never mistaken for a live picture, wired there the way voice ports are wired to the audio bus: the auto node publishes whatever front end it placed on a video port of its own, so a camera in the span reaches the screen without anything being wired by hand. The front end runs across the span rather than on a detected source, and that correction is worth recording: a detector measures the few megahertz of a camera's carrier that stand above the floor, and placing the reader on that filtered a twenty megahertz FM transmission down to four, leaving no line rate to lock to. It is placed where the channel plan reaches and the span is fast enough to hold a picture, the way AIS and Mode S are placed by their bands. The other half of the same fault was in the detector: `max_width_hz` refuses a run wider than the widest narrowband signal, so a camera never opened as one source at all and the runs inside it opened separately, which is what made a 5.8 GHz camera arrive as a packet list of sensors that were not there. Width alone does not make something a picture, so the front end tests before it decodes: `dsp::video::find_lines` looks for a pulse train at a line rate and asks how well the gaps agree with each other, which the off-air capture scores 0.67 and a synthesised camera 0.99, while the WiFi, BLE and impulsive-noise captures in `testdata/offair` never reach the question. A span that fails is left alone for a second before being looked at again, and the check happens before the demodulation rather than after it, so a span with no camera in it costs a twenty-fifth of one rather than all of it. While a picture is locked the camera owns the span, and it says so through the same door every front end uses: `Node::claimed_hz` returns the band a front end has locked onto, the auto node asks every one of them and keeps what it is told, the detector is closed out of it and the spectrum draws it. Nothing there is keyed on video, or on any other name. Its band is the whole span, because that is what an FM composite carrier occupies; it is claimed when the lock happens rather than at build time, or the band would be turned off for everything else on the chance a camera turns up, and once taken it is kept for the session, because a picture fades and comes back and a claim that followed the signal would hand the band back between fields. Before that every run inside the carrier opened as a source of its own and each got a set of front ends, which cost detection, extraction and the front ends nearly four times real time while reporting sensors that were not transmitting. Colour costs what it used to as well: the subcarrier reference is a rotation advanced per sample from one seed a line, where it was a sine and a cosine per sample per tap, which by itself was seventy million trigonometric calls a second and most of what the separator cost. and `decode::video_channels` is the 40 channel plan, reporting both names where two bands share a frequency because nothing in the signal says which the transmitter was set to. Verified off air against an AKK RaceRunner on A1: 181 fields from 3.6 seconds, most with all 288 lines, and a recognisable colour picture. Two things a reader should know. The line period is measured rather than configured, which is what tells PAL from NTSC, and the subcarrier phase has to free-run rather than restart each line: at 20 MS/s one sample of sync jitter is 80 degrees of subcarrier. The measured deviation was about 1 MHz rms and 4.6 MHz occupied, well inside a 20 MS/s span, so the pessimism about clipping was wrong. A field carries the shape of the picture rather than of its sample grid: both standards are 4:3, and 640 samples across 288 lines drawn from its own numbers is 10:9, a picture with the sides pushed in. Audio sits on a 6.5 MHz subcarrier in the same baseband and is not read |
| ExpressLRS 900 MHz | 433/868/915 MHz | LoRa, SF6-SF9 over 500 kHz | 500 kHz | framing | mod | `dsp::lora` demodulates it already. What is missing is that ELRS uses implicit header mode with a fixed 8 byte payload and no LoRa CRC, guarding the packet with its own 14 bit CRC seeded from the binding UID, and hops on every packet. The payload is CRSF: packed RC channels, or telemetry and link statistics on the return slot |
| ExpressLRS 2.4 GHz | 2400.4-2479.4 MHz, 80 channels 1 MHz apart | LoRa SF5-SF8 over 812.5 kHz, or FLRC at 1 Mbps | 2 MHz | framing, demod for FLRC | mod | An SX1280. `decode::elrs` reads the link layer: the CRC-14 seeded from the binding UID, the four packet types, a sync packet's hop index, counter, rate and the two UID bytes it sends in the clear, the four ten bit channels an RC packet carries, and the hop sequence a UID generates, all from the firmware's own `src/lib/OTA` and `src/lib/FHSS`. Confirmed off air against a TX16S: SF7 over 812.5 kHz, sync word 0x12, on channel after channel of the hop set. Which of the ten rates is running is measured rather than configured, the way every other front end here decides things: the sweep the classifier reads off a burst gives the spreading factor (5.16e9 Hz/s over 812.5 kHz is SF7 and nothing else), and where that leaves two rates, the spacing between packets that stayed on one channel separates them, since the link keys on a fixed interval and hops every four packets. On the bench capture that measures 10.00 ms, which is 100 Hz Full, and the handset was set to 100 Hz Full. The SX1280 transmits with I and Q swapped against the SX127x convention, so the preamble is downchirps to this receiver and the samples have to be conjugated before `dsp::lora` sees anything; five captures read as an empty band before that was found. What is still missing above the dechirper is the SX1280's long interleaved coding rates, which are not the SX127x interleaver `decode::lora` implements, so the payload bytes it currently produces are not to be trusted, and hop following. [Getting the SX1280's coding out of an SX1280](#getting-the-sx1280s-coding-out-of-an-sx1280) is the procedure for closing that. FLRC is a coherent GFSK burst mode with its own coding and is a front end of its own |
| TBS Crossfire | 868/915 MHz | LoRa, roughly 50 channel FHSS | 250 kHz | framing | mod | An SX1272 running LoRa with a proprietary framing and hop sequence on top, reversed publicly by g3gg0. Same shape of work as ELRS and the same CRSF payload underneath |
| FrSky ACCST D16 and ACCESS | 2400-2480 MHz, 47 channels 1.5 MHz apart | GFSK 70 kbit/s, 57 kHz deviation, 9 ms frame | 500 kHz | synthetic | mod | `decode::frsky` reads the packet: the handset id, where it is in the hop sequence, how far the sequence steps, the receiver number and eight channels of stick positions as microseconds, all in the clear under a CRC-16 whose polynomial is checked against a packet dumped from a real handset. It also generates the hop sequence a given id produces, both the v1 and v2 tables and both regulatory variants. Stronger evidence than ExpressLRS gives: sixteen bits of CRC with no seed, so a burst that is not FrSky passes about one time in 65536, and the id is in the packet rather than in the CRC. The two id bytes in front of the CRC are outside it, since the CC2500 filters on them in hardware, so a reported id is corroborated by the packets around it rather than by the check itself. No front end yet, and nothing has met real RF |
| FlySky AFHDS-2A | 2400-2480 MHz, 16 channels drawn from 164 | GFSK, A7105, 3.85 ms frame | 1 MHz | synthetic | mod | `decode::flysky` reads sticks, failsafe, settings, telemetry and bind packets: both ends' four byte ids in every packet, sixteen channels in microseconds, the receiver's battery voltage, RSSI and error rate, and on a bind packet the whole hop table, which is the link. The A7105 checks its CRC in hardware and does not put it in the buffer, so a listener demodulating the air has no check to make: `Packet::plausible` is structural instead (a defined type byte, channel values a servo pulse can take, hop entries in range) and one packet is a maybe where a run from the same ids is a fact. Nothing has met real RF |
| Spektrum DSM2 and DSMX | 2400-2480 MHz | DSSS GFSK 1 Mbps, CYRF6936 | 2 MHz | demod | mod | Needs the despreader the 802.15.4 rows need |
| Toy drone links: Bayang, Syma, Hubsan, E010 | 2400-2480 MHz, a megahertz a channel | GFSK 250 kbit/s or 1 Mbit/s, nRF24 or XN297 or A7105 | 2 MHz | synthetic | mod | `decode::nrf24` reads the XN297 frame, which is the one a listener can read at all: the chip sends a fixed 28 bit preamble (0xC710F55) before the address, so a packet announces itself, and the address, payload and CRC are scrambled with a published table rather than kept secret. Neither the address length nor the payload length is transmitted, so both are searched and the CRC-16 with its length dependent xorout decides. Two honest limits are in the code. The CRC covers address and payload together and its xorout is indexed by their sum, so where one ends and the other begins is not in the signal: five bytes is assumed because that is what toys use, `Packet::raw` is what is actually determined, and `split_is_a_guess` says so. And searching 192 combinations weakens a sixteen bit CRC to about one false accept in 341, measured in the tests. A plain nRF24 without the XN297 preamble stays unreadable without knowing the address first, which is the same problem an ExpressLRS UID poses. No front end yet |
| MAVLink over a SiK radio | 433/868/915 MHz | GFSK 64-250 kbps, FHSS, Golay | 250 kHz | framing | mod | 3DR and RFD900 telemetry, in the clear unless the operator set a key: position, attitude, battery, flight mode and the parameter set. The FSK front end reaches the symbols; the framing is the SiK link layer under the MAVLink v1/v2 parser |

### Getting the SX1280's coding out of an SX1280

The payload of an ExpressLRS 2.4 GHz packet is coded at one of the SX1280's
long interleaved rates, and nobody has published what those are. Semtech names
them in the data sheet and describes nothing; ExpressLRS writes
`SX1280_LORA_CR_LI_4_8` into a register and the modem does the rest, so
neither the firmware nor any of the open LoRa decoders (gr-lora_sdr, gr-lora,
LoRa-SDR, all SX127x) contains the layout. Every ExpressLRS receiver on the
market, the RadioMaster RP series included, is a Semtech SX1280 or SX1281
doing it in hardware.

Guessing is not hopeless, because ExpressLRS supplies an oracle. Its CRC-16 is
seeded with the binding UID and the packet counter, a CRC is linear in its
seed, so any candidate decode can be solved for the seed that would make it
valid. The counter is the low byte of that seed and the UID the high byte, and
the UID does not change between packets: a wrong hypothesis scatters the
solved high byte over all 256 values, and the right one repeats it.
`crates/nodes/examples/elrs_crack.rs` is that test, and no binding phrase is
needed to run it. Against thirty packets off the bench, every arrangement of
the SX127x interleaver scored 0.13 or below where chance is 0.004 and a
correct answer would be 1.0, which is the evidence that the coding really is
something else.

The cheap way to settle it is to make an SX1280 encode payloads we choose:

1. Any SX1280 or SX1281 on an SPI bus. An Ebyte E28-2G4M12S on a spare ESP32
   header, or a spare ExpressLRS receiver reflashed, since an RP1 is an
   ESP8285 wired to an SX1281 and its pin map is in the ExpressLRS target
   definitions. RadioLib drives the family and takes the long interleave flag
   on `setCodingRate`, and `../sub-ghz-modem` already links RadioLib, so this
   is a board variant rather than new protocol code.
2. Configure it as the link does: SF7, 812.5 kHz, CR_LI 4/8, 12 symbol
   preamble, implicit header, 13 byte payload.
3. Transmit an all-zero payload first. Whatever comes back out of the dechirp
   is the whitening sequence by definition, which is one unknown removed.
4. Then transmit 104 payloads with exactly one bit set, walking the position.
   Where each bit lands in the symbols is the interleaver and the Hamming
   layout, read off rather than searched for.
5. Record with the HackRF at 2.4 GHz and dechirp with
   `crates/nodes/examples/elrs_crack.rs`, which already collects symbols per
   packet. Check the answer against the oracle above on real link traffic
   before believing it.

### What we can verify here

A bench with a Remote ID beacon, an ExpressLRS link, a 5.8 GHz video link and a
DJI Mini 4K covers four of the rows above with real RF, which decides the
order more than the cost estimates do. Open Drone ID over Bluetooth legacy is
first: the front end exists, the beacon transmits it once a second, and the
messages say a serial number and a position that can be checked against where
the aircraft actually is.

The beacon here is a Holybro RemoteID module on an S500, which is an ESP32
running ArduRemoteID, and that is better than a black box for two reasons. It
is configurable, so each transport can be switched on alone and a capture can
be attributed with certainty rather than inferred: BT4 legacy by itself is the
first fixture, and turning BT5 Long Range and Wi-Fi on afterwards says exactly
which of them a decoder is missing. And the values it broadcasts are set by
us, over MAVLink from the flight controller or in its own parameters, so a
fixture can carry a serial number and a position chosen in advance. An
expectation in `fixtures.toml` written against a number we configured is a
real check, unlike one written against whatever the decoder happened to print. The Mini 4K then gives DroneID on the same bench,
with a serial number printed on the airframe to check a decode against. It is
on EU firmware, so it also broadcasts EN 4709-002 Direct Remote ID to keep its
class marking, and the first measurement to make is which transport it uses
for that: DJI has shipped both Bluetooth and Wi-Fi beacon across models and
firmware versions, and nothing here should assume one until a capture says
so. If it is Bluetooth, the same parser reads the drone and the Holybro module
and a real aircraft reaches **off air** with no new front end; if it is Wi-Fi,
DroneID is the only thing the Mini 4K can be read by until there is an 802.11
receiver. ELRS gives a hopping
link whose UID is known because we bound it, which is the difference between
testing a decoder and guessing at one. Every capture that earns an assertion
goes in `testdata/fixtures.toml`; a capture that only shows what a system
looks like on air, an OcuSync link or a digital video carrier, goes in
`testdata/offair.toml` as evidence for the classifier and nothing more.

The Bluetooth 5 half needs a different capture from the legacy half, and this
is the thing to get right. A long range advertiser sends almost nothing on the
primary channels: an ADV_EXT_IND pointing at a data channel, on which the
message pack follows a couple of milliseconds later. The pointer names the
channel, and the ones seen here move around the band, so a span covering
2402 to 2480 MHz would catch every one and no radio here reaches that. Two
ways round it: capture the primary channel first, read the pointers, and take
a second capture parked on the channel they name, which works because the
module repeats; or park on a stretch of data channels and read the auxiliary
packets alone, since `dsp::ble_coded` needs only the channel index for the
whitening and an AUX_ADV_IND carries the whole message pack on its own.

Two warnings about capturing this on a bench. Everything at 2.4 and 5.8 GHz
here is transmitting metres away, so the front end will be saturated unless
the gain is wound down and the antenna kept off, and a saturated capture is
worthless: the Honeywell note under [How a status is earned](#how-a-status-is-earned)
is exactly that failure. And a control link is a live aircraft's control link.
Capture receive only; nothing in this section is a thing to transmit near
something flying.

## Maritime

| Protocol | Where | Modulation | Width | RX | TX | Notes |
|---|---|---|---|---|---|---|
| AIS | 161.975/162.025 MHz | GMSK 9600 bps | 25 kHz | synthetic | mod | `dsp::ais` demodulates both channels, `dsp::hdlc` does NRZI, the flags, the bit destuffing and the CRC, and `decode::ais` reads the message tables. The auto node runs it span-wide wherever the span covers 162 MHz, since two channels stations alternate between are not something a spectrogram finds reliably. Checked on synthetic RF only: no recording of real traffic yet |
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
| DMR | 136-174, 400-470 MHz | 4-FSK 4800 baud | 12.5 kHz | off air | mod | `nodes::dmr_nodes` recovers the clock with a Gardner loop, correlates the 48-bit syncs and holds a burst clock through a superframe; `decode::dmr` undoes the Golay(20,8) slot type, the BPTC(196,96) full link control, the QR(16,7,6) EMB and the BPTC(128,72) embedded LC, so who called whom on which talkgroup is read from the header, the terminator and the embedded LC. Verified against an off-air hotspot capture whose own link control names talkgroup 9 and radio ID 1234567 four ways that agree. `decode::dmr_bp` undoes Motorola Basic Privacy. Speech is AMBE, in `crates/mbe` behind the `ambe` feature, which a build from source has and a published binary does not, because the codec is patent encumbered. Slot 2 is not yet separated from slot 1 |
| P25 phase 1 | 700-900 MHz | C4FM | 12.5 kHz | demod | mod | As DMR, plus IMBE |
| NXDN, dPMR | 400-470 MHz | 4-FSK | 6.25/12.5 kHz | demod | mod | |
| M17 | amateur bands | 4-FSK 4800 baud | 12.5 kHz | off air | mod | Link setup, stream and packet frames, in `dsp::m17` and `decode::m17`. Reports who called whom, the channel access number, whether the stream is encrypted or signed, and the position, text or repeater callsigns the metadata carries. Packet mode is reassembled and CRC checked, so an SMS packet reports its message. A receiver that missed the link setup rebuilds it from six stream frames through the link information channel, which is what that channel is for. Frames are verified against the M17 project's own C library symbol for symbol, in `the_frames_match_the_reference_implementation`, which is a stronger check than **synthetic** usually means: an encoder and a decoder written together agree with each other whatever they both misread, and this one agrees with somebody else's. Verified off air against an OpenRTX handheld: the capture in `testdata/fixtures.toml` decodes to the callsign the transmission carries, asserted through the same path the live radio runs. Voice decodes too, Codec 2 at 3200 bits per second through the `codec2` crate, onto the voice bus |
| TETRA | 380-400, 410-430 MHz | pi/4-DQPSK 36 kbps | 25 kHz | partial | mod | Control channels read off the air: `dsp::tetra` demodulates by differential detection, resynchronising timing and carrier on every burst's training sequence, then runs the downlink coding stack (scrambling, interleaving, RCPC Viterbi, CRC, and the (30,14) block code of the access assign field), and `decode::tetra` reads the PDUs. A carrier is logged as who it is (SYNC and SYSINFO: MCC, MNC, colour code, location area, main carrier) and what it knows (D-NWRK-BROADCAST: the neighbouring cells by carrier and location area). Signalling to a party is read from the MAC header even when enciphered: the address, the encryption mode, any usage marker and channel allocation. In clear, the CMCE call control PDUs (D-SETUP, D-CONNECT, D-TX GRANTED, D-RELEASE and the rest) give the parties, the call identifier and group or private, and D-SDS-DATA gives text. The access assign field of every slot is followed for traffic, so a call becomes a start row and an end row with its airtime, by usage marker and by the party the marker was given to. Verified against a recorded Irish downlink for everything but the clear-mode PDUs, which that network encrypts; those are tested on synthetic bits. Traffic is read as well: `dsp::tetra::speech` recovers the two STEC frames a slot carries, `decode::voice` deciphers them, and `decode::vocoder` is a reimplementation of the ETSI EN 300 395-2 fixed-point speech decoder, in progress. Under the `tea` feature the slot keystream is `decode::tea` (TEA1 and TEA2), so an enciphered network's traffic decrypts with a key entered in the keys pane; without one, TEA1's 32-bit fold is brute forced by `decode::recover` on the CPU or `decode::gpu` on a GPU, and TA61 identities are recovered by `decode::ta61`, so the parties can still be named. A binary from the releases page has none of that; a build from source does. Not read: anything on a network whose key is unknown and unrecoverable |
| FM with CTCSS/DCS | any | FM plus subaudible tone | 12.5 kHz | table | mod | Trivial next to the rest: a Goertzel on the discriminator output |

An analogue channel says nothing about itself, so the strip has a `voice`
switch per channel and that is what turns one into a front end. Switched on,
`nodes::VoiceChannelNode` ends an over where the squelch does, puts the whole
transmission on the packet bus with its audio, and the call appears in the
call list beside the digital ones. It is on by default for the modes people
talk on, NFM, AM and SSB, and off for broadcast FM, which would otherwise
transcribe a music station for as long as the receiver runs; the switch is on
the strip, for a channel that turns out to be data. It takes two wires, the channel's IF and
its audio, because what was said is in the audio and how strong it was is only
in the IF: a level read off a demodulator's output is a level of the
demodulator.

What was said is read by `crates/stt`, a local Whisper model through candle,
as `app::transcripts::LiveTranscribeNode` on the audio bus rather than on the
packet bus. The bus has a second output, a tap carrying every strip's audio
and every decoded voice port before the faders and the subscriptions, so what
is written down is what the receiver heard and not what the operator chose to
listen to. Whisper reads a window rather than a stream, so streaming here is a
window re-read as it grows: while somebody is talking the audio so far goes to
the model every two seconds and the running line gets longer, and when the
speech stops the whole utterance goes once more and that reading is the one
kept. A conversation is keyed `{proto}:{freq}:{chan}:{speaker}`, with the
parts the receiver does not know left empty, so an FM channel is
`Audio:145500000::` and a DMR call is `DMR:435000000:9:1234567`; the calls
view finds a call's text by building that key rather than by being wired to
the transcriber. The log is in memory and bounded, 512 conversations of 64
utterances. It is on by default and fetches its
own weights: the worker thread downloads `openai/whisper-base.en`, 74 MB, into
`~/.local/share/waveshark/models/whisper` the first time a call is long enough
to be worth reading, so a receiver that hears no speech never reaches the
network and one that does waits once. Another model is a `model` setting on
the stage, or a directory placed there by hand. `--no-default-features` leaves
candle out of the build entirely. The model runs on whatever candle was built
for: CUDA under `--features cuda`, Metal on a Mac, the CPU otherwise, and a
GPU that opens but cannot launch a kernel falls back rather than failing every
call. Any front end that carries speech is transcribed, not
just analogue channels, so an M17 or DMR call gets the same treatment. The
text arrives as a `transcript` field on the decode, with the model's own mean
log probability beside it, and a call whose text the model does not believe
keeps the audio and shows no words.

What turns an identifier into a name is `crates/datasets`, which fetches and
caches the DMR and NXDN registries, the repeater and reflector lists, the
airports with their air traffic frequencies, and the Artemis signal database.
A DMR radio ID is a number until one of those says whose it is.

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
| APRS / AX.25 1200 | 144.39/144.8/144.64 MHz | AFSK over FM | 16 kHz | synthetic | mod | `dsp::afsk` reads Bell 202 off the discriminator, `dsp::hdlc` does NRZI, destuffing and the CRC, and `decode::ax25` and `decode::aprs` read the frame and the position in all three encodings, uncompressed, compressed and Mic-E. Placed by the auto node on any source whose channel it fits. Checked on synthetic RF only |
| Packet 9600 (G3RUH) | 144-440 MHz | direct FSK 9600 | 25 kHz | framing | mod | Scrambled NRZI |
| Morse (CW) | any | OOK | 500 Hz | synthetic | done | `decode::morse` reads the same table in both directions, and `morse_tx` is the one protocol that transmits end to end: text into timings into a keyed carrier, checked by a round trip through a file sink in `crates/nodes/tests/morse_round_trip.rs` |
| RTTY | HF, VHF | FSK 45.45 baud | 1 kHz | framing | mod | Baudot, and the same two-tone shape as everything else here |
| PSK31 | HF | BPSK 31.25 baud | 100 Hz | demod | mod | Varicode, coherent |
| SSTV | HF, 144 MHz | FM subcarrier | 3 kHz | framing | mod | Image, like APT |
| FT8, WSPR, JS8 | HF, 6 m | narrow MFSK | 50 Hz-3 kHz | demod | mod | Long coherent integration and LDPC. A different kind of receiver |
| DTMF and tone remote | any | audio tones over FM | 12.5 kHz | table | mod | Goertzel pair |

## Cellular

| Protocol | Where | Modulation | Width | RX | TX | Notes |
|---|---|---|---|---|---|---|
| GSM synchronisation channel | 850/900/1800/1900 MHz | GMSK 270.833 kbps | 200 kHz | synthetic | mod | `dsp::gsm` finds the frequency correction burst by the variance of its phase advance, which is a tone a quarter of the symbol rate up, and reads the synchronisation burst one TDMA frame later: the cell's identity code and the frame number, behind ten bits of parity that decide whether a burst happened at all. The tone also measures the tuner's error, which the burst after it needs to a few hundred hertz. The tone is found by the coherence of the phase advance across a burst rather than by its variance, which is what makes it measurable at the signal to noise ratio a real cell arrives at, and a burst is reported only when a second one agrees with it about the frame number: ten bits of parity let one burst in a thousand through, and a scan tries thousands. The scanner table ships a `[GSM]` block for it, switched off and pointed at nothing in particular, because which carriers a network uses is licensed per operator and per country. Checked against a live network off air, which is where both of its bit orders came from, but no capture of one is committed: a cell identity plus a channel number places a receiver within a few hundred metres, and that would be in the history for good. What is committed instead is `a_multiframe_of_beacons_keeps_time_with_itself`, which is that recording's shape and impairments with the identity replaced by the test network 001-01, and two vectors that pin the bit orders from outside this crate. `crates/decode/examples/gsm_scan.rs` says whether a capture holds a cell and what it decodes; keep the capture local |
| GSM broadcast and common control | as above | as above | 200 kHz | synthetic | mod | The four frames after each synchronisation burst, read as normal bursts and deinterleaved into one 23 byte block behind a 40 bit Fire code. Nothing marks a burst as broadcast: the receiver knows where the block is because the synchronisation burst said which frame it was in, and knows which of the eight training sequences to correlate because the standard makes it the cell's own colour code. `decode::gsm` then reads the cell identity and the location area out of system information 3, 4 and 6, the cell's own hopping allocation out of type 1 and the neighbours a phone is told to measure out of type 2, and names the rest. Paging requests are read too, all three types: a network calls a phone by a temporary identity it reallocates, so a run of them says how busy a cell is without saying whose phones they are, and the row separates out the ones paged by permanent subscriber identity because a network doing that has given up the point of the temporary one. On a live cell about 2% of pages were by permanent identity, some of them roamers. An immediate assignment says which channel a phone was just granted, hopping or not, and carries the timing advance the network told it to use, which is how long its burst took to arrive and therefore the range to it: 554 metres a step, and the furthest phone in a recording was 2.8 km away. A cell that names itself fully becomes a device in the survey, keyed by operator, location area and cell number rather than by the carrier it was heard on, so moving a cell to another channel does not make it a new one. The decode says who it is, as `common::Identity`, the way every other protocol does; a synchronisation burst carries only a colour code, which is reused a few streets away, so it claims no identity at all. The neighbour list is what makes one cell worth reading: it is the channel numbers of the others, so a receiver that has read one carrier has been handed the rest of the network. All five frequency list formats are read. Bit map 0 is one bit per channel and covers the 900 band; the 1800 band's numbers run past a thousand, so its lists are sent as a tree of differences instead, and that recursion is transcribed from Wireshark's `f_k` rather than reproduced from the prose, because nobody gets it right from the prose and a list decoded wrongly reads as neighbours that are not there. Every burst is read through the channel the training sequence measures, in `dsp::gsm::equalise`: five taps estimated by least squares and a max-log-MAP pass over the trellis they define. Slicing the symbols instead works on a synthesised signal and falls apart on a real one, because GMSK spreads every symbol over three symbol periods before any reflection does |
| GSM dedicated signalling | as above | as above | 200 kHz | synthetic | mod | An immediate assignment names a timeslot, and `SchDetector::follow` reads it: the eight signalling channels a timeslot carries are four frames each in its own 51 frame multiframe, so which one a block belongs to follows from its frame number. Above that is LAPDm, and above that the exchange a phone has before ciphering starts. On a live cell that is a location updating request naming the phone, an authentication request, and then a ciphering mode command, after which the channel goes dark. The slow channel that rides alongside carries two octets of layer 1 in front of its link layer, so it is read separately: the power the network ordered and the timing advance it set, which is a range to the phone remeasured twice a second for as long as the channel is up. It also repeats the cell's own identity and neighbour list, which is an independent check on what the broadcast channel said. Only signalling channels are followed. A traffic channel's slow channel is laid out on a 26 frame multiframe rather than a 51 frame one, so its blocks are elsewhere, and by the time a call reaches one the ciphering mode command has been sent and they are ciphered regardless: following it would be work spent on frames that cannot decode. A phone whose temporary identity the network does not recognise sends its permanent one in the clear, and roamers do this on arrival; what is committed here reads it, so be deliberate about what a capture of it is kept for |
| GSM traffic | as above | as above | 200 kHz | chain | chain | A5/1 and A5/3 stand in the way, and past the cipher it is a stack rather than a decoder |
| LTE / 5G | various | OFDM | 1.4-100 MHz | chain | chain | Cell search and MIB decode is possible in principle; past that it is a stack, not a decoder |

## Time and beacons

| Protocol | Where | Modulation | Width | RX | TX | Notes |
|---|---|---|---|---|---|---|
| DCF77 | 77.5 kHz | AM plus phase modulation | 100 Hz | chain | mod | Needs VLF hardware |
| MSF, WWVB | 60 kHz | AM | 100 Hz | chain | mod | As DCF77 |
| NDB beacons | 190-535 kHz | keyed carrier | 1 kHz | chain | mod | |

## Transmit

Two things go on air: a keyed carrier and narrowband FM from a microphone or a
tone. The rest of the gap is structural rather than protocol by protocol. The
device layer is in place; three things above it are missing.

The device layer, done: `Device::start_tx` returns a `TxStream`, which takes
blocks and reports the transfers the radio sent as zeros because nothing was
queued in time. A radio that transmits says so through `DeviceInfo::tx`,
which carries the transmit tuning ranges and gain stages separately from the
receive ones because they are different hardware; the default is `None`, so
an RTL-SDR refuses rather than failing at the first block. `hackrf-usb` holds
the transmit half of the USB protocol (bulk OUT, `TRANSCEIVER_MODE_TRANSMIT`,
TXVGA gain) and keeps transfers queued ahead of the radio. `sources::FileSink`
is the same trait writing a capture instead, which is what a modulator should
be developed against: what it produces replays through the receiver, so the
test asserts a decode rather than a waveform, and nothing is radiated while
it is still wrong.

The first protocol is keyed: `morse_tx` takes text as bytes and produces IQ,
holding `morse_key` (the table in `decode::morse`, read the same way in both
directions) and `ook_mod` inside it as an inner graph. The modulator knows
nothing about Morse, so every protocol with a timing table adds an encoder and
reuses the carrier. Streams now carry their direction: `StreamSpec::flow` is
`Rx` or `Tx`, and a node fed by both at once fails to build, which GNU Radio
cannot check because a port there is complex samples and nothing else. Bursts
carry `tx_start`, `tx_end` and `tx_at` tags, named after `tx_sob`, `tx_eob`
and `tx_time` for the same reasons.

Half duplex is the driver's problem, not the receiver's. Keying does not tear
the receive stream down: the HackRF driver takes the reader away, feeds the
stream a floor three bits of the eight bit converter wide, about 33 dB down,
at the same rate and centre for the length of the over, and puts the radio back
when the transmit stream is dropped. That level is what this radio's own floor
measures with the front end running, and it is deliberately not lower: at 90 dB
down every sample lands on the same value and the ADC health check reports a
starved converter for the length of every over. The receive graph runs throughout, so the spectrum's averaging,
every channel's squelch and every part-built frame survive an over, and the
waterfall shows the gap instead of stopping. `RxStream::silent` says which it
is. A LimeSDR is 2x2 and full duplex, so it needs none of that: `crates/limesdr`
transmits on its own chain with its own synthesiser, which `Device::set_tx_center`
tunes, so a repeater pair is one radio listening on the output while it
transmits on the input rather than a retune around every over. `TxInfo::channels`
reports what the board has, asked of the chip: two on a LimeSDR-USB, one on a
Mini.

NFM runs end to end from the graph: `tone` into `fm_mod` into `radio_tx`,
which holds the device's `TxStream` and paces the whole chain by blocking
when the radio has enough queued. `crates/nodes/tests/nfm_tx_path.rs` runs
that graph into a file sink and demodulates what the "radio" received with
`dsp::FmDemod`; `crates/app/examples/nfm_tx.rs` is the same graph with a
HackRF in place of the file. A keyed channel transmits either a test tone or the microphone: `mic`
(`crates/nodes/src/tx_nodes.rs`) holds an `audio::AudioSource`, resamples it
from the microphone's rate to the radio's with Catmull-Rom, and is a stage in
front of the modulator rather than something the radio thread pushes in. The
microphone is opened on key-up and closed on release, so it is live for
exactly as long as the carrier. The interpolation is the weak point: at 48 kHz
into 2 MS/s the images of a 3 kHz tone land near 45 kHz, about 70 dB down,
where a polyphase filter would do better at the cost of a multiply-accumulate
per output sample at the radio's rate.

Still missing:

1. **The rest of the encoders.** `decode::slicer::slice` turns a pulse train
   into bits under a timing table; the same table turns bits back into a pulse
   train. Every protocol that has a table gets an encoder nearly for free,
   which is why the transmit column above mostly mirrors the receive one.

2. **Modulators past the basic set.** `ook_mod`, `fsk_mod`, `ask_mod`,
   `am_mod` and `fm_mod` cover keyed carriers, two tones, multi-level
   amplitude, and voice narrow or wide. `fsk_mod` is CPFSK and describes
   two-level keying only, by the same convention `dsp::fsk` reads: a mark is
   the upper tone. Anything with more than two levels needs a port that
   carries symbols rather than durations, and PSK, GMSK, 4-FSK and OFDM are
   each their own node.

3. **Scheduling and limits.** Transmission is time critical in a way reception
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
4. ~~**AIS.**~~ Written and wired, on synthetic RF only. What it still needs is
   a recording of real traffic.
5. ~~**POCSAG.**~~ Done. The BCH(31,21) correction it added is reusable for
   FLEX, ERMES and the radiosondes.
6. ~~**Wireless M-Bus T and C.**~~ Done, both modes, against seven of
   rtl_433's meter recordings. The sync word plus block CRC work carries over
   to Z-Wave and Homematic. Mode S still waits on a recording.
7. **The transmit path, ending in Morse.** Device, encoder, modulator,
   scheduler, proven end to end on the simplest possible protocol.
8. ~~**LoRaWAN.**~~ Done: join requests, device addresses and frame counters,
   all of which are in the clear even though the payload is not.
9. ~~**ADS-B.**~~ Done, on a wideband branch of its own rather than a bank
   channel.
10. **P25 and NXDN.** Framed like DMR, which is read now, and the vocoder they
   need is already ported. What is missing is the framing layer for each.
11. **A recording of real AIS and APRS traffic**, which is the only thing
   standing between those two and a **done**.
12. ~~**Open Drone ID over Bluetooth, both transports.**~~ Done, off air
   against the Holybro module: the legacy advertisement, and the Bluetooth 5
   Long Range message pack behind the coded PHY and an auxiliary pointer.
   Worth a third capture with a serial, an operator ID and self-ID text
   configured, since both of the current ones carry only what the module
   ships with.
13. **DJI DroneID.** Expensive, an OFDM front end and a turbo decoder, but it
   is the highest value thing in this file that is transmitted in the clear,
   there is a working receiver to check against, and there is an aircraft here
   that sends it.

Everything below that (OFDM broadcast, trunked voice, cellular) is a project
each rather than a decoder each, and should be judged on its own.
