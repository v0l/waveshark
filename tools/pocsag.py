#!/usr/bin/env python3
"""Key a POCSAG page out of a Flipper Zero, and read it back with the receiver.

    tools/pocsag.py send --address 1234567 --message "CALL CONTROL"
    tools/pocsag.py send --capture            transmit, record, and decode

The Flipper has no POCSAG encoder, so the page is built here and played as a
RAW sub-GHz file: run lengths of a two-level FSK waveform, which is all POCSAG
is. The preset is a custom CC1101 one because none of the stock presets is
near the 4.5 kHz deviation a pager transmitter uses, and the receiver's channel
filter is 12.5 kHz wide: at the stock 47.6 kHz the signal is wider than the
channel and nothing survives the discriminator.

This encoder is deliberately a second implementation rather than a call into
`decode::pocsag::encode`. A transmitter and receiver sharing one encoder agree
with themselves and prove nothing, so this one is written from ITU-R M.584-2
and checked, at import, against a page captured off air and decoded by POC32
(the same page `decode::pocsag`'s test uses).

The Flipper transmits on 433.92 MHz, not on a paging channel: 153 MHz is
outside what its CC1101 will tune, and keying a real pager band is not
something to do by accident. What is under test is the waveform and the
framing, and neither depends on the carrier.
"""

import argparse
import os
import subprocess
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from flipper import Flipper, REMOTE_DIR, ROOT, HRFREC, WAVESHARK, record, replay

# ITU-R M.584-2. The sync word every batch opens with, and the codeword sent
# in a frame with nothing to say.
SYNC = 0x7CD215D8
IDLE = 0x7A89C197
BATCH_WORDS = 16
# BCH(31,21) generator: x^10 + x^9 + x^8 + x^6 + x^5 + x^3 + 1.
BCH_POLY = 0x769

NUMERIC = "0123456789*U -)("

# CC1101 registers, and what a Flipper .sub file calls the preset that loads
# them. Everything but the deviation is the firmware's own 2-FSK async preset;
# DEVIATN 0x13 is (8+3) * 2^1 * 26e6/2^17 = 4364 Hz, the closest the part gets
# to the 4.5 kHz a pager transmitter uses.
PRESET = (
    "02 0D 0B 06 08 32 07 04 14 00 13 02 12 04 11 83 10 67 15 13 "
    "18 18 19 16 1D 91 1C 00 1B 07 20 FB 22 10 21 56 00 00 "
    "C0 00 00 00 00 00 00 00"
)


def codeword(content):
    """The 32-bit codeword for 21 bits of content: BCH parity, then even parity."""
    rem = (content & 0x1FFFFF) << 10
    for i in range(30, 9, -1):
        if rem >> i & 1:
            rem ^= BCH_POLY << (i - 10)
    word = (((content & 0x1FFFFF) << 10) | (rem & 0x3FF)) << 1
    return word | (bin(word).count("1") & 1)


def contents(address, function, text, alpha=True):
    """The codeword contents of one page, padded to whole batches.

    The address is 21 bits and only 18 of them are in the codeword: the low
    three decide which frame of the batch it goes in, because a pager only
    listens during its own frame. Idle codewords fill the frames before it.
    """
    idle = IDLE >> 11 & 0x1FFFFF
    out = [idle] * ((address & 7) * 2)
    out.append(((address >> 3) & 0x3FFFF) << 2 | (function & 3))

    bits = []
    if alpha:
        for ch in text:
            code = ord(ch) & 0x7F
            bits += [code >> i & 1 == 1 for i in range(7)]
    else:
        for ch in text:
            code = NUMERIC.find(ch)
            code = 12 if code < 0 else code
            bits += [code >> i & 1 == 1 for i in range(4)]
        while len(bits) % 20:
            bits += [12 >> i & 1 == 1 for i in range(4)]

    for i in range(0, len(bits), 20):
        chunk = bits[i : i + 20]
        w = 1 << 20
        for j, b in enumerate(chunk):
            if b:
                w |= 1 << (19 - j)
        out.append(w)

    while len(out) % BATCH_WORDS:
        out.append(idle)
    return out


def bitstream(page):
    """Preamble, then batches of a sync word and sixteen codewords."""
    bits = [i % 2 == 0 for i in range(576)]
    words = [codeword(c) for c in page]
    for at in range(0, len(words), BATCH_WORDS):
        batch = words[at : at + BATCH_WORDS]
        for w in [SYNC] + batch:
            bits += [w >> i & 1 == 1 for i in range(31, -1, -1)]
    return bits


def raw_durations(bits, baud):
    """Run lengths in microseconds, signed by level, as a RAW file wants them."""
    period = 1_000_000.0 / baud
    out, run, level = [], 0, bits[0]
    for b in bits + [not bits[-1]]:
        if b == level:
            run += 1
            continue
        d = int(round(run * period))
        out.append(d if level else -d)
        level, run = b, 1
    return out


def sub_file(bits, freq, baud):
    lines = [
        "Filetype: Flipper SubGhz RAW File",
        "Version: 1",
        f"Frequency: {freq}",
        "Preset: FuriHalSubGhzPresetCustom",
        "Custom_preset_module: CC1101",
        f"Custom_preset_data: {PRESET}",
        "Protocol: RAW",
    ]
    d = raw_durations(bits, baud)
    # The parser reads a bounded number of values from each line.
    for at in range(0, len(d), 512):
        lines.append("RAW_Data: " + " ".join(str(v) for v in d[at : at + 512]))
    return "\n".join(lines) + "\n"


def self_check():
    """Reproduce a page captured off air and decoded by POC32.

    Address 1238681, function 0, numeric "1724", in frame 1. If this encoder
    has the BCH, the parity, the frame numbering or the reversed characters
    wrong, it cannot produce these two codewords by accident.
    """
    page = contents(1_238_681, 0, "1724", alpha=False)
    words = [codeword(c) for c in page]
    want = (0b01001011100110100110011000000011, 0b11000111001000010001111000000010)
    got = (words[2], words[3])
    if got != want:
        raise SystemExit(
            f"encoder self-check failed: {got[0]:#034b} {got[1]:#034b}\n"
            f"                  wanted {want[0]:#034b} {want[1]:#034b}"
        )


def cmd_send(args):
    self_check()
    bits = bitstream(contents(args.address, args.function, args.message))
    text = sub_file(bits, args.freq, args.baud)
    secs = len(bits) / args.baud
    print(
        f"{len(bits)} bits, {secs:.2f} s a pass at {args.baud} baud, "
        f"{args.freq / 1e6:.3f} MHz"
    )

    flip = Flipper(args.port)
    remote = f"{REMOTE_DIR}/pocsag.sub"
    print(f"upload {remote}: {flip.upload(remote, text)}")

    proc = None
    if args.capture:
        for tool in (HRFREC, WAVESHARK):
            if not os.path.exists(tool):
                sys.exit(
                    f"missing {tool}: cargo build --release -p app "
                    "--example hrfrec --bin waveshark"
                )
        os.makedirs(args.out, exist_ok=True)
        # Off centre for the same reason the rest of this harness is: on
        # centre the signal sits under the HackRF's DC spur.
        centre = args.freq + 280_000
        proc = record("flipper_pocsag", centre, secs * args.repeat + 3.0, args.gain, args.out)
        time.sleep(1.5)

    print(flip.tx_file(remote, args.repeat, timeout=secs * args.repeat + 30))

    if proc is None:
        return
    proc.wait()
    path = os.path.join(args.out, f"flipper_pocsag_{centre / 1e6}M_2000k.cf32")
    decodes, raw = replay(path)
    if not decodes:
        print(f"nothing decoded from {path}")
        if args.verbose:
            print(raw)
        return
    for name, fields in decodes:
        print(f"{name}: {fields}")


def main():
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    sub = p.add_subparsers(dest="cmd", required=True)
    s = sub.add_parser("send", help="build a page, upload it, key it out")
    s.add_argument("--address", type=int, default=1_234_567)
    s.add_argument("--function", type=int, default=3, help="3 is alphanumeric")
    s.add_argument("--message", default="WAVESHARK TEST PAGE")
    s.add_argument("--baud", type=int, default=1200, choices=(512, 1200, 2400))
    s.add_argument("--freq", type=int, default=433_920_000)
    s.add_argument("--repeat", type=int, default=1)
    s.add_argument("--port")
    s.add_argument("--capture", action="store_true", help="record it and decode it")
    s.add_argument("--out", default=os.path.join(ROOT, "testdata", "offair"))
    s.add_argument("--gain", type=float, default=40.0)
    s.add_argument("--verbose", action="store_true")
    args = p.parse_args()
    {"send": cmd_send}[args.cmd](args)


if __name__ == "__main__":
    main()
