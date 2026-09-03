#!/usr/bin/env python3
"""Drive a Flipper Zero as a transmitter and record what it sends.

The Flipper's sub-GHz encoders are somebody else's implementation of the same
fixed-code and rolling-code remotes this project decodes, so a frame it keys
out and this project reads back is a cross-check rather than a round trip
through one set of assumptions. That is what separates a **synthetic** status
from a verified one in docs/protocols.md.

    tools/flipper.py list                  the protocols this tool knows how to key
    tools/flipper.py trim captures/*.cf32  cut a recording down to its bursts
    tools/flipper.py capture princeton     one protocol
    tools/flipper.py capture --all         everything in the table

Needs a Flipper on USB, a HackRF with an antenna, and a release build of
`hrfrec` and `waveshark`. Captures land in --out, which defaults to testdata/offair: the same place
the off-air corpus lives, and gitignored for the same reason.

The recorder deliberately tunes 280 kHz above the transmission: centred on the
signal, an OOK burst sits under the HackRF's DC spur and nothing is detected at
all.
"""

import argparse
import glob
import os
import re
import subprocess
import sys
import time

import serial

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
HRFREC = os.path.join(ROOT, "target", "release", "examples", "hrfrec")
WAVESHARK = os.path.join(ROOT, "target", "release", "waveshark")
REMOTE_DIR = "/ext/subghz/waveshark"

# Far enough that the burst clears the DC spur, close enough to stay well
# inside the 1.5 MHz the analogue filter passes at 2 MS/s.
OFFSET_HZ = 280_000
PROMPT = ">: "

OOK650 = "FuriHalSubGhzPresetOok650Async"
OOK270 = "FuriHalSubGhzPresetOok270Async"

# Protocol name as the firmware spells it, the key file's fields, and what this
# project should report. `te` is omitted where the encoder has a fixed one.
TABLE = {
    "princeton": dict(
        proto="Princeton", bits=24, key="00 00 00 00 00 AA BB CC", te=400,
        freq=433_920_000, preset=OOK650, expect="Princeton",
    ),
    "came": dict(
        proto="CAME", bits=12, key="00 00 00 00 00 00 0D BC",
        freq=433_920_000, preset=OOK650, expect="CAME-12bit",
    ),
    "nice_flo": dict(
        proto="Nice FLO", bits=12, key="00 00 00 00 00 00 0A B5",
        freq=433_920_000, preset=OOK650, expect="Nice-Flo",
    ),
    # The 40 bits start with a 0x5 header nibble; a key without it is a frame
    # no real remote sends, and the decoder is right to refuse it.
    "holtek": dict(
        proto="Holtek", bits=40, key="00 00 00 51 23 45 67 89", te=433,
        freq=433_920_000, preset=OOK650, expect="Holtek",
    ),
    "holtek_ht12x": dict(
        proto="Holtek_HT12X", bits=12, key="00 00 00 00 00 00 08 5F", te=320,
        freq=433_920_000, preset=OOK650, expect="Holtek-HT12x",
    ),
    # Both are 288/310 MHz protocols in the field, keyed here on 433.92
    # because a stock region blocks 310 MHz: what is under test is the timing
    # and the frame, and neither depends on the carrier.
    "linear": dict(
        proto="Linear", bits=10, key="00 00 00 00 00 00 02 D5",
        freq=433_920_000, preset=OOK650, expect="Linear",
    ),
    # Reboots the firmware whatever the bit count, under both spellings of the
    # name, so this one stays untested until the right key file is known.
    "linear_delta3": dict(
        proto="Linear Delta 3", bits=8, key="00 00 00 00 00 00 00 5A",
        freq=433_920_000, preset=OOK650, expect="Linear-Delta3",
    ),
    "ansonic": dict(
        proto="Ansonic", bits=12, key="00 00 00 00 00 00 05 5A", te=555,
        freq=433_075_000, preset=OOK650, expect="Ansonic",
    ),
    "bett": dict(
        proto="BETT", bits=18, key="00 00 00 00 00 02 AB CD", te=340,
        freq=433_920_000, preset=OOK650, expect="Bett",
    ),
    "keeloq": dict(
        proto="KeeLoq", bits=64, key="E7 06 63 6B 9C 52 38 05",
        manufacture="Dea_Mio",
        freq=433_920_000, preset=OOK650, expect="KeeLoq",
    ),
    "somfy": dict(
        proto="Somfy Telis", bits=56, key="00 A7 00 12 34 56 78 9A",
        freq=433_420_000, preset=OOK650, expect="Somfy-RTS",
    ),
}

# A key file whose bit count does not match what the named encoder expects
# reboots the firmware mid-command, so every entry above is keyed as its own
# protocol rather than probed with a generic frame.


class Flipper:
    def __init__(self, port=None):
        port = port or self.find_port()
        self.port = port
        self.s = self.open(port)
        self.cmd("")

    @staticmethod
    def open(port):
        # A write with no timeout blocks forever when the firmware reboots and
        # stops draining the CDC endpoint, which is the one way this tool can
        # hang with nothing on screen.
        s = serial.Serial(port, 230400, timeout=1, write_timeout=10)
        time.sleep(0.3)
        s.reset_input_buffer()
        return s

    @staticmethod
    def find_port():
        # The node number moves when the device re-enumerates, the by-id link
        # does not.
        links = glob.glob("/dev/serial/by-id/*Flipper*")
        if not links:
            sys.exit("no Flipper on USB (looked for /dev/serial/by-id/*Flipper*)")
        return links[0]

    def reconnect(self, timeout=30.0):
        # A malformed key file reboots the firmware, which drops the CDC port
        # and brings it back a few seconds later under the same by-id link.
        try:
            self.s.close()
        except Exception:
            pass
        deadline = time.time() + timeout
        while time.time() < deadline:
            if glob.glob("/dev/serial/by-id/*Flipper*"):
                try:
                    time.sleep(1.0)
                    self.s = self.open(self.find_port())
                    return True
                except Exception:
                    pass
            time.sleep(0.5)
        return False

    def read_to_prompt(self, timeout=10.0):
        out = ""
        deadline = time.time() + timeout
        while time.time() < deadline:
            try:
                chunk = self.s.read(4096).decode(errors="replace")
            except serial.SerialException:
                return out + "\n<device rebooted>"
            if chunk:
                out += chunk
                if out.endswith(PROMPT):
                    break
            else:
                time.sleep(0.02)
        return out

    def write(self, data):
        try:
            self.s.write(data)
            return True
        except serial.SerialException:
            # Includes SerialTimeoutException: the firmware stopped reading.
            return self.reconnect() and bool(self.s.write(data))

    def cmd(self, line, timeout=10.0):
        try:
            self.s.reset_input_buffer()
        except serial.SerialException:
            self.reconnect()
        if not self.write((line + "\r").encode()):
            return "<device gone>"
        out = self.read_to_prompt(timeout)
        # Strip the echoed command and the trailing prompt.
        return out.replace(line, "", 1).replace(PROMPT, "").strip()

    def upload(self, remote, text):
        self.cmd(f"storage mkdir {REMOTE_DIR}")
        self.cmd(f"storage remove {remote}")
        self.s.reset_input_buffer()
        if not self.write(f"storage write {remote}\r".encode()):
            return "<device gone>"
        time.sleep(0.4)
        self.s.read(4096)
        # `storage write` echoes what it is given and writes it a line at a
        # time. A RAW sub-GHz file is kilobytes rather than the few hundred
        # bytes a key file is, and handing it over in one write times out with
        # the file half written; this is slower and finishes.
        data = text.encode()
        for at in range(0, len(data), 256):
            if not self.write(data[at : at + 256]):
                return "<device gone>"
            self.s.read(4096)
            time.sleep(0.03)
        time.sleep(0.3)
        self.write(b"\x03")
        self.read_to_prompt(5.0)
        size = self.cmd(f"storage stat {remote}")
        return size

    def tx_file(self, remote, repeat, timeout=30.0):
        return self.cmd(f"subghz tx_from_file {remote} {repeat} 0", timeout)


def sub_file(spec):
    lines = [
        "Filetype: Flipper SubGhz Key File",
        "Version: 1",
        f"Frequency: {spec['freq']}",
        f"Preset: {spec['preset']}",
        f"Protocol: {spec['proto']}",
        f"Bit: {spec['bits']}",
        f"Key: {spec['key']}",
    ]
    if "te" in spec:
        lines.append(f"TE: {spec['te']}")
    if "manufacture" in spec:
        lines.append(f"Manufacture: {spec['manufacture']}")
    return "\n".join(lines) + "\n"


def record(name, centre_hz, secs, gain, out_dir):
    mhz = centre_hz / 1e6
    proc = subprocess.Popen(
        [HRFREC, name, f"{mhz}", "2", str(secs), str(gain)],
        cwd=out_dir, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
    )
    return proc


def trim(path, pad=0.1, keep=1.0, floor_margin=8.0):
    """Cut a recording down to the bursts in it, as cs8.

    Five seconds at 2 MS/s is 80 MB of float for a few tens of milliseconds of
    signal. The eight bits are what the HackRF's converter actually produced,
    so nothing is lost by storing what it heard rather than what the recorder
    widened it to.
    """
    import numpy as np

    x = np.fromfile(path, dtype=np.complex64)
    rate = 2_000_000
    m = re.search(r"_([\d.]+)M_(\d+)k\.cf32$", path)
    if m:
        rate = int(float(m.group(2)) * 1000)
    # Notch the DC spur before measuring, or it swamps every burst.
    k = np.ones(201) / 201
    y = x - np.convolve(x, k, mode="same")
    n = max(1, rate // 10_000)
    blocks = len(y) // n
    p = (np.abs(y[: blocks * n].reshape(-1, n)) ** 2).mean(1)
    db = 10 * np.log10(p + 1e-20)
    # A weak capture sits closer to the floor than a strong one, so drop the
    # threshold rather than declaring an empty recording.
    for margin in (floor_margin, 6.0, 4.0, 3.0):
        hot = np.where(db > np.median(db) + margin)[0]
        if len(hot):
            break
    if len(hot) == 0:
        return None, 0
    lo = max(0, int(hot[0] * n - pad * rate))
    hi = min(len(x), int(hot[-1] * n + pad * rate))
    hi = min(hi, lo + int(keep * rate))
    cut = x[lo:hi]
    out = path.replace(".cf32", ".cs8")
    iq = np.empty(cut.size * 2, dtype=np.int8)
    iq[0::2] = np.clip(np.round(cut.real * 128), -128, 127)
    iq[1::2] = np.clip(np.round(cut.imag * 128), -128, 127)
    iq.tofile(out)
    return out, len(hot)


def replay(path):
    r = subprocess.run([WAVESHARK, "--replay", path], capture_output=True, text=True)
    decodes = []
    for line in r.stdout.splitlines():
        # A frame decoder that produces bytes reports no signal level of its
        # own, so the dBFS and dB columns are NaN on exactly the rows that
        # decoded. Matching only a number here dropped every real decode.
        m = re.search(
            r"MHz\s+(\S+)\s+(?:-?[\d.]+|NaN) dBFS\s+(?:[\d.]+|NaN) dB\s+(\S+)\s+(.*)",
            line,
        )
        if m:
            decodes.append((m.group(2), m.group(3).strip()))
    return decodes, r.stdout


def capture_one(flip, key, spec, args):
    remote = f"{REMOTE_DIR}/{key}.sub"
    flip.upload(remote, sub_file(spec))
    centre = spec["freq"] + OFFSET_HZ
    name = f"flipper_{key}"
    proc = record(name, centre, args.secs, args.gain, args.out)
    time.sleep(1.5)
    tx = flip.tx_file(remote, args.repeat)
    proc.wait()
    rec = proc.stdout.read().strip().splitlines()[0] if proc.stdout else ""
    path = os.path.join(args.out, f"{name}_{centre/1e6}M_2000k.cf32")
    decodes, raw = replay(path)
    return dict(tx=tx, rec=rec, path=path, decodes=decodes, raw=raw)


def cmd_list(args):
    for key, spec in TABLE.items():
        te = f"TE {spec['te']}" if "te" in spec else "fixed TE"
        print(f"{key:<14} {spec['proto']:<14} {spec['bits']:>3} bit  "
              f"{spec['freq']/1e6:>7.3f} MHz  {te:<9} expect {spec['expect']}")


def cmd_trim(args):
    for path in args.files:
        out, hot = trim(path, keep=args.keep)
        if out is None:
            print(f"{os.path.basename(path)}: no burst found, left alone")
            continue
        before = os.path.getsize(path)
        after = os.path.getsize(out)
        print(f"{os.path.basename(out)}: {hot} hot blocks, "
              f"{before/1e6:.0f} MB -> {after/1e6:.1f} MB")
        if args.remove:
            os.remove(path)


def cmd_capture(args):
    keys = list(TABLE) if args.all else args.protocol
    if not keys:
        sys.exit("name a protocol or pass --all")
    unknown = [k for k in keys if k not in TABLE]
    if unknown:
        sys.exit(f"unknown protocol(s): {', '.join(unknown)}")
    for tool in (HRFREC, WAVESHARK):
        if not os.path.exists(tool):
            sys.exit(f"missing {tool}: cargo build --release -p app --example hrfrec --bin waveshark")
    os.makedirs(args.out, exist_ok=True)

    flip = Flipper(args.port)
    results = {}
    for key in keys:
        spec = TABLE[key]
        print(f"== {key} ({spec['proto']}, {spec['freq']/1e6:.3f} MHz)")
        r = capture_one(flip, key, spec, args)
        results[key] = r
        if "rebooted" in r["tx"]:
            print("   the firmware rebooted: the key file is not what this encoder wants")
            flip.reconnect()
        print(f"   {r['rec']}")
        if r["decodes"]:
            names = {}
            for name, fields in r["decodes"]:
                names.setdefault(name, fields)
            for name, fields in names.items():
                mark = "OK  " if name == spec["expect"] else "also"
                print(f"   {mark} {name}: {fields}")
        else:
            print("   nothing decoded")

    print("\n== summary")
    for key, r in results.items():
        want = TABLE[key]["expect"]
        got = {n for n, _ in r["decodes"]}
        status = "pass" if want in got else ("other" if got else "no decode")
        print(f"{key:<14} {status:<10} {' '.join(sorted(got)) if got else ''}")


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", help="serial port (default: the by-id link)")
    sub = p.add_subparsers(dest="cmd", required=True)

    sub.add_parser("list", help="show the protocol table")

    t = sub.add_parser("trim", help="cut recordings down to their bursts, as cs8")
    t.add_argument("files", nargs="+")
    t.add_argument("--keep", type=float, default=1.0, help="seconds at most")
    t.add_argument("--remove", action="store_true", help="delete the float original")

    c = sub.add_parser("capture", help="transmit and record")
    c.add_argument("protocol", nargs="*")
    c.add_argument("--all", action="store_true")
    c.add_argument("--out", default=os.path.join(ROOT, "testdata", "offair"))
    c.add_argument("--secs", type=float, default=5.0)
    c.add_argument("--gain", type=float, default=40.0)
    c.add_argument("--repeat", type=int, default=20)

    args = p.parse_args()
    {"list": cmd_list, "trim": cmd_trim, "capture": cmd_capture}[args.cmd](args)


if __name__ == "__main__":
    main()
