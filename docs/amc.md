# Signal identification

A burst that matches no protocol is the case this receiver should be best at,
and `decode::analyze` currently answers it with two pulse-width histograms.
That reasoning is sound for on/off keyed ISM devices and blind everywhere else:
it never sees the carrier, so FSK, PSK and a chirp all look like whatever their
envelope happened to do. This document plans a learned replacement, published
as an open model together with the data generator and the training code that
produced it.

The goal is every digital mode on the air, not the dozen this repo already
decodes. That goal decides the architecture, because it is not a modulation
classification problem.

## Why one classifier cannot do it

DMR, NXDN and P25 phase 2 are all 4-FSK near 4800 baud. FT8, JT65 and WSPR are
all MFSK. Zigbee and BLE are both constant-envelope quadrature keying. The
difference inside each of those groups is burst timing, frame layout and FEC,
and none of it is in the modulation. A softmax over modulation classes cannot
separate them however many classes it has, because the evidence is not in its
input.

The published automatic modulation classification work is therefore only half
of this. Its benchmark, RadioML 2018.01A, has 24 modulation classes and the
2025 surveys agree on what happens with them: family-level accuracy saturates
above 90 percent at high SNR for nearly every architecture since 2016, the
residual error is almost all within-family order confusion, and it degrades
monotonically as SNR falls. `AMR-Benchmark` reimplements the well known models
on four datasets under one protocol and finds the spread across datasets larger
than the spread across architectures. Architecture is not the lever.

The lever is what the model is asked. Split it in two.

## Two stages

**Stage one estimates the physical layer.** Modulation family, symbol rate,
occupied bandwidth, burst duration and inter-burst repetition, plus a
confidence on each. Family is a small closed set that genuinely is a
classification problem: `ook_ask`, `fsk`, `mfsk`, `msk_gmsk`, `psk`, `qam`,
`am`, `fm`, `chirp`, `ofdm`, `noise`. Symbol rate and bandwidth are
regressions, and they carry more identifying information than the family label
does.

**Stage two identifies the mode, conditioned on those estimates.** Symbol rate
alone collapses most of the candidate set before the model sees anything: 4-FSK
at 4800 baud is a short list, 4-FSK at 3125 baud is a different short list. The
stage-two model is a metric embedding, not a classifier. Each known mode is a
prototype in that space, identification is nearest prototype within a
calibrated radius, and anything outside every radius is returned as unknown
with its stage-one parameters attached.

That structure is the one the 2025 open-set literature converges on, from
OpenMax over deep metric embeddings through prototype methods and few-shot
open-set variants. Here it is not a refinement, it is the only shape that fits
the problem, for two reasons. The mode list is open-ended permanently, so
adding a mode has to cost a handful of labelled bursts and a new prototype
rather than a retrain. And a confidently mislabelled unknown burst is worse
than an honest unknown, because the label is what a person acts on when they
start reverse engineering a device.

Stage one degrades usefully on its own. "4-FSK, 4800 baud, 12.5 kHz, 60 ms
bursts every 30 ms, unidentified" is a result somebody can work with, and it is
strictly more than the histogram analyser can say today.

## Input representation

Decided by ablation rather than by assertion, over the grid used in the
multimodal ablation work: IQ alone, spectrogram alone, constellation alone,
each pair, all three. Two further arms matter here and are not in that grid.
Instantaneous amplitude and frequency as channels, which `dsp::fsk` and
`dsp::ask` already compute. And a cyclostationary or spectral-correlation
summary, which the surveys repeatedly find is the input that survives low SNR
best, and which is also where symbol rate is most directly visible, so stage
one likely wants it whatever the ablation says about family accuracy.

Bursts are stored as complex baseband and every representation is derived on
the fly. Nothing about the representation is baked into the stored dataset.

One parameter is not free, and finding that out cost a class. A chirp sweeps
its whole channel in one symbol, so its bandwidth is `symbol_rate * 2^SF` and
its samples per symbol cannot be drawn independently of its spreading factor.
The first generator drew them independently and produced sweeps up to ten times
wider than the sample rate, which alias into something that fills the spectrum
like a chirp and dechirps like noise: peak-to-mean 3.8 where a real chirp gives
over 400. Occupied bandwidth would not have caught it, because an aliased sweep
still fills the span. What caught it was holding the generator to the same
detector that found the Meshtastic packet off the air.

## Data

Synthetic, generated in-repo from the `dsp` crate so the modulator and the
receiver share one definition of every waveform, with a real held-out test set.

The generator sweeps, per class: symbol rate against sample rate as a
fractional ratio rather than an integer one, SNR from -20 to +30 dB, carrier
frequency offset, phase offset, sample timing offset, IQ imbalance, DC offset,
pulse shaping roll-off, Rayleigh and Rician multipath taps, and burst lengths
including bursts shorter than the model's window. Those are the impairments
this receiver actually has, since its own DC blocker and AGC sit in the path,
and they are the augmentations the transfer-learning survey identifies as
partially closing the sim-to-real gap.

Sim-to-real is the known unsolved problem in this field, so the off-air
evaluation runs from the first training run rather than at the end. The set is
the rtl_433 corpus under `testdata/rtl433`, where the protocol is known so the
modulation and symbol rate are known, plus recordings from `app::record`. It is
never trained on. A model at 99 percent synthetic and 70 percent off-air has
told us the generator is wrong, which is a result worth having in week one.

The label is declared per device, not inferred from the burst. Two inference
rules were tried first and both failed. Labelling by which front end found the
burst is wrong because the FSK detector fires on amplitude-keyed bursts too:
the noise in a gap has a wide instantaneous frequency and that reads as
deviation, which put Oregon and GT-WT03 in the FSK class. Labelling by how much
of a transmission is spent with the carrier off works on most bursts and fails
on the ones where a detector triggered late or merged two frames, where the
measurement describes the window rather than the signal: one LaCrosse TX29IT
frame came out at 0.96 against 0.05 for its siblings. A rule that is right nine
times in ten is not a label.

So the family comes from a table, in `modes.rs` for the devices this repository
decodes and in `data/rtl433-modulation.json` for the 434 rtl_433 knows,
extracted from the `.modulation` field of each of its decoders. Our own decode
identifies the individual burst and wins where it exists; the capture's
reference JSON identifies only the file and is the fallback, recorded as such
in `label_source`, and it is what reaches the devices this repository has no
decoder for. That is most of the corpus and nearly all of its frequency-keyed
traffic: labelling by our decoders alone found 21 FSK bursts, and the table
finds 433.

The envelope measurement is then the cross-check rather than the label. Where
it contradicts the declared family the burst is dropped, which costs 630 of
8574. Its threshold cannot be validated on the surviving bursts, because
dropping the disputes is precisely dropping everything near the threshold and
any separation measured afterwards is guaranteed. It is validated per device
instead, over every burst including the dropped ones, which nothing enforces.

The set is still lopsided at 5286 amplitude keyed against 433 frequency keyed,
and it holds no PSK, QAM, chirp or OFDM at all, because ISM band recordings do
not contain them. Anything the model claims about those families will rest on
synthetic evidence alone until there are captures to say otherwise, and the
evaluation has to say so rather than reporting one number.

Class balance is enforced at generation time. The off-air set is not balanced
and is reported per class, never as one accuracy number. Stage two is evaluated
with modes held out by class, not by sample, because enrolling an unseen mode
is its actual job.

## Framework

Deferred, deliberately. Inference in the receiver is a weight-loading problem,
not a design problem: candle writes safetensors natively and `burn-store` has
`safetensors` and `pytorch` in its default features, so a checkpoint moves into
either once the shapes and tensor names are known. Nothing about the training
work has to wait for that decision, so it does not.

One constraint survives the deferral and binds every model choice from here on.
The network may only use operations candle and burn both implement: 1D and 2D
convolution, batch and layer normalisation, standard activations, linear layers
and plain attention. Complex-valued convolutions, custom cyclostationary
kernels and exotic normalisations are all present in the AMC literature and
none of them survive an export, so any of those has to be a preprocessing step
in `amc-data` rather than a layer in the model. That is a cheap rule to follow
now and an expensive one to retrofit.

The boundary that makes this work is already in place: `amc-data` emits plain
`Vec<f32>` and a shape and names no tensor type, so training can happen in
whatever is fastest to iterate in while the dataset stays framework-neutral.

One risk to remember when the Rust side does get written. `burn` 0.21 declares
`rust-version = 1.92` against this workspace's 1.82, so the ML crates raise the
effective MSRV for anyone who builds them and have to stay out of the default
build. GPU support on this machine is unverified for both candle through cudarc
and burn through cubecl, but that check now belongs to whenever inference is
built rather than to the start of the work.

## Where this ended up

The `amc-data` crate is gone. It existed to train a network and then to hold a
second classifier while the two were compared, and once the comparison was
made most of it was duplication: its classifier against `dsp::classify`, its
corpus labelling against `classify_corpus`, its dataset export against a
network that lost. What survived is the part that was never duplicated, which
is the evidence rather than the code: ten off-air captures, their manifest, and
`crates/decode/tests/classify_offair.rs`, which scores the receiver's own
classifier against them beside the test that scores it against rtl_433's.

The generator, the impairment sweep, the training tooling and the dechirp
detector are all in the history rather than the tree. That is deliberate. The
trainer read a dataset written by the exporter that left with the crate, so a
fresh clone could never have run it, and tooling that cannot run is worse than
tooling that is absent because it looks maintained. The dechirp detector could
still run, and went anyway: it was a scaffold for finding one signal, its job is
done, and `dsp::classify` measures the same sweep as a matter of course now.

Nothing below depends on any of it being present. What survived is the evidence
and the argument: ten captures, a test that scores the receiver against them,
and the record of what the network cost.

## Folded into `dsp::classify`

The receiver grew its own classifier and router while this was being written,
and the two were developed against different evidence: `dsp::classify` against
the rtl_433 corpus, which is amplitude and frequency keyed ISM devices, and
this against captures of LoRa, BLE, Wi-Fi, LTE and an empty band. Run over each
other's material, each is strong where it was developed and weak where it was
not: on the corpus `dsp` is 1.000 of 123 claimed against this crate's 0.425,
and on the off-air captures it is 0.333 of 42 against 0.716.

Neither replaces the other, so the parts that were genuinely missing went in:

- One file per hypothesis, behind a `Hypothesis` trait with the shared terms
  lifted into `Evidence`. Adding a modulation is a file and a line.
- `ofdm` and `dsss`, splitting the noise-like class by what repeats: a cyclic
  prefix at one lag, or a chip sequence in the envelope where the data's sign
  flips cancel the complex correlation. LTE goes from noise-like to OFDM on
  nine bursts of thirteen.
- `cyclo`, the measurement both need, with the localization ratio that stops
  every narrowband burst passing for OFDM.
- `zoom`, off by default so a channelized bank is untouched, for captures
  where the signal is a fraction of its span.
- `mode`, naming LoRa SF11/BW250, BLE advertising, 802.11b beacons and LTE
  from the parameters.

What did not get fixed, with the cause measured rather than guessed: BLE still
reads as OOK in `dsp`. The envelope-mode counter reports four levels on a
constant-envelope packet, and the reason is the noise margin around the burst,
not the histogram. Handed the packet with its margins stripped the count drops
to two and MSK starts winning. So the lead is `Classifier::extent`, which is
not trimming a short constant-envelope burst out of the silence it arrived in,
and everything downstream then measures the silence. A level-separation floor
of two decibels was added along the way and is worth keeping on its own terms,
but it is not the fix: at three decibels it takes the corpus from 52 captures
to 48, because shallow ASK keys its levels only a few decibels apart.

## In the receiver

`nodes::classify_nodes::ClassifyNode`, registered as `classify`, so a chain can
name it like any other stage. It sits on complex baseband beside the pulse
detectors rather than after them, and that placement is the point: a pulse
detector throws away the carrier to get an envelope, and the modulation lives
in the carrier. `PulseDetectNode` can say a burst is keyed and cannot say
whether it is keyed in amplitude, in frequency, or swept.

It gates bursts itself with the same two millisecond hangover the off-air
evaluation needed, tracks its own noise floor from the quiet stretches, refuses
anything under 8 dB, and emits `Event::Classified` with the family, the mode
where the parameters name one, and the evidence. It passes samples through
untouched, so inserting it costs a chain nothing but the measurement.

`Event::Classified` is deliberately not `Event::Decoded`: nothing was decoded,
no checksum passed, no payload came out. What it carries is what the signal is
and why we think so.

## Layout

Three new crates, none in the default build.

`crates/amc-data` generates and stores labelled bursts and derives the input
representations. Depends on `dsp` and `common`, and on no ML framework.

`crates/amc` holds both stages, the training loop, the evaluation harness and
the inference trait. The only crate that names candle or burn.

`crates/amc-modes` holds the mode registry: for each known mode its expected
modulation, symbol rate, bandwidth and burst structure, plus its enrolled
prototype. Data, not code, so contributing a mode does not mean touching the
model.

The receiver hook is in `decode::analyze`. Stage one runs first, stage two runs
when stage one is confident enough to condition on, and the histogram path runs
when stage two rejects or when the model is absent. `Analysis` gains the
estimated parameters, the mode and the confidences, and `summary()` says which
path produced the line.

## Second result: the network is not needed for stage one

`amc-data/src/classify.rs` classifies from measured statistics with no learned
weights, and beats the network nearly everywhere the two can be compared:

| capture | family | feature classifier | NN (range over retrains) |
|---|---|---|---|
| Meshtastic LoRa, both packets | chirp | 1.000 | 0.88 to 0.97 |
| BLE advertising | msk_gmsk | 1.000 | 0.13 to 1.00 |
| LTE downlink | ofdm | 1.000 | 0.01 to 0.99 |
| 802.11 frames | ofdm | 0.833 | 0.51 to 0.98 |
| 868 sensor | fsk | 0.5 (n=2) | 0.01 to 0.73 |
| noise, both captures | noise | 1.000 | 0.02 to 0.95 |
| FM broadcast | fm | 0.000 | 0.01 |

The right column is the real argument: across five retrains of the same
architecture on regenerated data, per-capture scores moved by up to 0.9. The
tests do not move, they explain themselves, and they emit the parameters stage
one exists to estimate (sweep rate, symbol lag, tone separation, modulation
index, occupied bandwidth) as by-products. Every label in the evaluation set
was established by one of these tests in the first place, so the network was
only ever being asked to imitate them, with a sim-to-real gap added.

FM fails because that capture is at 7 dB SNR and the tests say `noise`, which
is arguably the correct answer for a signal below the sensitivity of every
statistic tried. It needs a better capture more than a better classifier.

What a network is still for, if anything: stage two's open-ended mode
fingerprinting, where a matched test cannot be written because the class list
is not known in advance. That was the design's plan for it all along.

## First result (superseded): the network

Trained on synthetic windows alone, scored on recordings of transmitters this
project does not control, restricted to the three families that have real
captures with known parameters:

| capture | family | windows | correct |
|---|---|---|---|
| Meshtastic LoRa SF11, packet a | chirp | 1500 | 0.947 |
| Meshtastic LoRa SF11, packet b | chirp | 1500 | 0.954 |
| BLE advertising, channel 38 | msk_gmsk | 370 | 1.000 |
| LTE band 28 downlink | ofdm | 1500 | 0.955 |
| 802.11 frames, 2462 MHz | ofdm | 44 | 1.000 |

Overall 0.956, from a model that has never seen a recording.

The same model asked the full eleven-way question scores 0.482 off air, and the
first explanation offered for that was wrong. Forty percent of training windows
sit below 0 dB SNR and a fifth below -10 dB, where a 1024 sample window carries
no evidence of the family its label claims, so the obvious suspect was label
noise. Dropping those windows moved the off-air score from 0.482 to 0.485, and
dropping fewer of them made it worse.

The actual answer came from scoring the model against a synthetic holdout drawn
from an unseen seed, which separates memorising the generator from failing to
transfer. It scores 0.575 there, so it does not solve its own synthetic problem
either, and that number decomposes entirely along SNR:

| SNR | holdout accuracy |
|---|---|
| -20 to -15 dB | 0.112 |
| -15 to -10 dB | 0.175 |
| -10 to -5 dB | 0.265 |
| -5 to 0 dB | 0.402 |
| 0 to +5 dB | 0.578 |
| +5 to +10 dB | 0.787 |
| +10 to +20 dB | 0.814 to 0.856 |
| +20 to +30 dB | 0.878 to 0.888 |

Chance is 0.091. The bottom of that sweep is not a model failure, it is a sweep
where the answer is not present in the data, and any single number averaged
over it mostly reports how much of the sweep is unclassifiable. Reporting
accuracy against SNR rather than as one figure is standard in this literature
for exactly this reason, and it is the only honest way to state an eleven-class
result here: about 0.88 in distribution at high SNR.

One thing that table does expose is real. Training accuracy reaches 0.972
across every SNR including -20 dB, where the holdout manages 0.112. Nothing can
classify those windows, so the only way to fit them is to memorise them, and
the capacity spent doing that is capacity not spent on the signals that can be
told apart. The fix is not a smaller SNR range, which was tried, but not
asking the network to fit a label that carries no information: a reject target
below a measured SNR floor, which is the same reject head the open-set design
already calls for.

Three things were needed to get there, and each was found by being wrong first.

The generator had to produce OFDM with a preamble. Real frames spend a third of
a short transmission on repeated short symbols and long symbols, and without
them the model read 802.11 as frequency keying at 2 percent correct while
scoring 95 percent on a continuous LTE downlink.

Bursts had to start and stop inside the window. Two in five generated bursts
are now keyed for only part of their length, with the channel noise present
throughout, because a model that has only seen signal filling every window has
never seen an edge and an edge is most of what a short frame is.

The spectrogram had to be computed at three resolutions. A LoRa SF11 chirp at 2
MS/s sweeps about half a bin of a 64 point transform across a 1024 sample
window, so at that resolution it is a stationary tone and it was being confused
with frequency keying; a 256 point transform resolves the ramp but smears a 51
us 802.11 frame. Both, stacked as channels, took chirp from 0.90 to 0.95 and
802.11 from 0.80 to 1.00.

## Order of work

1. `amc-data`: the synthetic generator and the impairment sweep, with a test
   that every generated class round-trips through the existing demodulators in
   `dsp` at high SNR. A generator whose FSK the FSK detector cannot read is
   generating something else. **Done**, for the eleven families, with the
   detector round-trip, an occupied-bandwidth check and a dechirp check.
2. The off-air evaluation set, extracted and labelled from the rtl_433 corpus,
   before any model exists. **Done**: 3629 captures give 5719 labelled bursts,
   5286 amplitude keyed and 433 frequency keyed, plus 2225 bursts nothing
   could name, which are the open-set material. 630 more were dropped where
   the measurement contradicted the declared family.
4. Stage one, family head only, small ResNet on IQ, scored on both sets. The
   number every later change has to beat. **Done** for three families, in
   a multi-resolution spectrogram rather than raw IQ, in a trainer that is now
   in the history rather than the tree. See the verdict above: it lost to the
   matched tests it was imitating.
5. Stage one parameter regression. Symbol rate error in percent is the metric,
   not accuracy, and it is the number stage two depends on.
6. The representation ablation, on the fixed backbone, against both heads.
7. Stage two: the metric embedding, prototype enrollment, and the reject radius
   calibrated on modes held out by class.
8. The `decode::analyze` integration, the mode registry, and the published
   artifact.
