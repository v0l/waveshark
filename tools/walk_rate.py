#!/usr/bin/env python3
"""Walk one LoRa configuration end to end: configure the bench transmitter,
record, walk the payload a bit at a time, and solve for the encoder.

    tools/walk_rate.py --sf 7 --cr 8 --li 1 --bytes 13

Writes testdata/sx1280_cr_li_<cr>_sf<sf>_<n>byte_map.json when the columns come
out full rank, and prints a line saying what was found either way. The symbol
widths are measured rather than assumed: each symbol is tried at every width
from SF-2 to SF, and the one under which single payload bits move that symbol
by exactly one bit is the width it carries.
"""

import argparse
import collections
import json
import os
import re
import subprocess
import sys
import time

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MODEM = "/home/kieran/git/sub-ghz-modem/tools/modem.py"
PORT = "/dev/ttyUSB1"
CENTER_MHZ = 2440.4
TUNER_MHZ = 2439.9
RATE = 2_000_000


def modem(*args):
    r = subprocess.run([sys.executable, MODEM, "--port", PORT] + list(args),
                       capture_output=True, text=True, timeout=60)
    return r.stdout.strip()


def symbols_from(path, sf, seconds):
    out = subprocess.run(
        [f"{REPO}/target/release/examples/lora_symbols", path, str(RATE),
         str(int(TUNER_MHZ * 1e6)), str(int(CENTER_MHZ * 1e6)), str(sf),
         "812500", "1", str(seconds)],
        capture_output=True, text=True, timeout=3600).stdout.splitlines()
    pk = []
    for i, line in enumerate(out):
        m = re.match(r"packet \d+ at ([\d.]+) s", line)
        if m:
            pk.append((float(m.group(1)), [int(x) for x in out[i + 1].split()]))
    return pk


def widths_for(votes, sf):
    """Per symbol, the number of bits it carries, decided by which width makes
    a single payload bit move that symbol by a single bit."""
    mod = 1 << sf
    best = []
    for j in range(len(votes[0])):
        scores = {}
        for w in range(sf - 2, sf + 1):
            b = (((votes[0][j] - 1) % mod) >> (sf - w))
            b ^= b >> 1
            single = 0
            for v in votes[1:]:
                c = (((v[j] - 1) % mod) >> (sf - w))
                c ^= c >> 1
                d = b ^ c
                if d and bin(d).count("1") == 1:
                    single += 1
            scores[w] = single
        best.append(max(scores, key=scores.get))
    return best


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sf", type=int, required=True)
    ap.add_argument("--cr", type=int, required=True, help="5, 6 or 8 with long interleaving")
    ap.add_argument("--bytes", type=int, default=13)
    ap.add_argument("--repeat", type=int, default=8)
    ap.add_argument("--gap", type=float, default=0.1)
    ap.add_argument("--group-gap", type=float, default=0.5)
    ap.add_argument("--keep", action="store_true", help="keep the capture")
    a = ap.parse_args()

    tag = f"cr_li_4_{a.cr}_sf{a.sf}_{a.bytes}byte"
    print(f"=== {tag}", flush=True)
    for k, v in (("modem", "lora"), ("bw", "812.5"), ("sf", a.sf), ("cr", a.cr),
                 ("li", "1"), ("sync", "0x12"), ("preamble", "12"), ("crc", "0"),
                 ("implicit", a.bytes), ("power", "13"), ("freq", CENTER_MHZ)):
        modem("set", f"{k}={v}")

    bits = a.bytes * 8
    secs = int((bits + 1) * (a.repeat * (a.gap + 0.14) + a.group_gap) + 12)
    name = f"rate_{tag}"
    for stale in os.listdir(REPO):
        if stale.startswith(name) and stale.endswith(".cf32"):
            os.remove(os.path.join(REPO, stale))
    rec = subprocess.Popen(
        [f"{REPO}/target/release/examples/hrfrec", name, str(TUNER_MHZ), "2", str(secs), "40"],
        cwd=REPO, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(4)

    sched = f"/tmp/{name}.json"
    subprocess.run([sys.executable, f"{REPO}/tools/bitwalk.py", "--port", PORT,
                    "--bytes", str(a.bytes), "--repeat", str(a.repeat),
                    "--gap", str(a.gap), "--group-gap", str(a.group_gap),
                    "--out", sched], check=True, capture_output=True, timeout=1200)
    rec.wait(timeout=secs + 60)
    cap = os.path.join(REPO, f"{name}_{TUNER_MHZ}M_2000k.cf32")
    if not os.path.exists(cap):
        print(f"  no capture at {cap}")
        return 1

    pk = symbols_from(cap, a.sf, secs)
    want = json.load(open(sched))["groups"]
    # Assign by the schedule's own clock rather than by gaps: the walk knows
    # when it transmitted each group, and only the offset between the recorder
    # starting and the walk starting is unknown.
    if not pk:
        print("  no packets")
        return 1
    offset = pk[0][0] - want[0]["start_s"]
    groups = [[] for _ in want]
    for t, syms in pk:
        rel = t - offset
        for gi, g in enumerate(want):
            if g["start_s"] - 0.15 <= rel <= g["end_s"] + 0.25:
                groups[gi].append((t, syms))
                break
    empty = [i for i, g in enumerate(groups) if not g]
    print(f"  {len(pk)} packets over {len(want)} groups, {len(empty)} empty")
    if empty:
        print(f"  groups with nothing in them: {empty[:8]}")
        return 1

    # A stray detection on noise has whatever length it has. The packet length
    # is fixed by the configuration, so take the modal length across the whole
    # capture and ignore anything else before voting within a group.
    nsym = collections.Counter(len(s) for _, s in pk).most_common(1)[0][0]
    votes = []
    for gi, g in enumerate(groups):
        c = collections.Counter(tuple(s) for _, s in g if len(s) == nsym)
        if not c:
            print(f"  group {gi} has no packet of the modal length {nsym}: not solved")
            return 1
        votes.append(c.most_common(1)[0][0])
    ppm = widths_for(votes, a.sf)
    mod = 1 << a.sf

    def coded(sym):
        out = []
        for j, v in enumerate(sym):
            c = ((v - 1) % mod) >> (a.sf - ppm[j])
            c ^= c >> 1
            out += [(c >> k) & 1 for k in range(ppm[j])]
        return out

    base = coded(votes[0])
    cols = [[x ^ y for x, y in zip(base, coded(v))] for v in votes[1:]]
    w = [sum(c) for c in cols]
    mat = [sum(1 << p for p, b in enumerate(c) if b) for c in cols]
    piv, rank = {}, 0
    for v0 in mat:
        v = v0
        while v:
            p = v.bit_length() - 1
            if p in piv:
                v ^= piv[p]
            else:
                piv[p] = v
                rank += 1
                break
    print(f"  {nsym} symbols, widths {collections.Counter(ppm)}, coded bits {len(base)}")
    print(f"  column weight min {min(w)} max {max(w)} mean {sum(w)/len(w):.2f}")
    print(f"  rank {rank} of {len(cols)}")
    if not a.keep:
        os.remove(cap)
    if rank != len(cols):
        print("  NOT full rank: not solved")
        return 1
    doc = {
        "what": f"SX1280 LoRa CR_LI 4/{a.cr}, SF{a.sf}, 812.5 kHz, implicit header,"
                f" no LoRa CRC, {a.bytes} byte payload",
        "measured_with": "RadioMaster RP2 (SX1281) driven by sub-ghz-modem, HackRF at 2 MS/s",
        "symbol_bits": ppm,
        "codeword_from_bin": "gray_encode((bin - 1) mod 2^SF) >> (SF - symbol_bits)",
        "offset_c": base,
        "columns": {f"bit{i}": [j for j, b in enumerate(c) if b] for i, c in enumerate(cols)},
    }
    out = f"{REPO}/testdata/sx1280_{tag}_map.json"
    open(out, "w").write(json.dumps(doc, indent=1))
    print(f"  solved -> {os.path.relpath(out, REPO)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
