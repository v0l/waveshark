#!/usr/bin/env python3
"""Turn a bit walk capture into the encoder's matrix.

Reads the schedule bitwalk.py wrote and the symbols lora_symbols printed, pairs
them by time, and reports for each payload bit how the symbols moved against
the all-zero baseline. If the chain is linear over GF(2) the difference is that
bit's column of M; the script checks that claim rather than assuming it, by
reporting whether XOR or a modulo subtraction gives the smaller, more
repeatable difference.

    tools/bitwalk_read.py /tmp/walk.json /tmp/walk_symbols.txt --sf 7
"""

import argparse
import collections
import json
import re
import sys


def load_packets(path):
    out = []
    lines = open(path).read().splitlines()
    for i, line in enumerate(lines):
        m = re.match(r"packet (\d+) at ([\d.]+) s\s+sync 0x(\w+)\s+preamble (\d+)\s+(\d+) symbols", line)
        if not m:
            continue
        syms = [int(x) for x in lines[i + 1].split()]
        out.append((float(m.group(2)), syms))
    return out


def gray_decode(v, bits):
    """LoRa transmits the Gray code of the codeword, so a one bit change in the
    codeword is several bits of change in the symbol value. Undoing it is what
    makes the walk read as one position per touched symbol."""
    out = v
    shift = 1
    while shift < bits:
        out ^= v >> shift
        shift += 1
    return out & ((1 << bits) - 1)


def cluster(packets, gap):
    groups, cur = [], [packets[0]]
    for p in packets[1:]:
        if p[0] - cur[-1][0] > gap:
            groups.append(cur)
            cur = []
        cur.append(p)
    groups.append(cur)
    return groups


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("schedule")
    ap.add_argument("symbols")
    ap.add_argument("--sf", type=int, default=7)
    ap.add_argument("--gap", type=float, default=0.4, help="group separation in seconds")
    ap.add_argument("--gray", action="store_true", help="undo the Gray map first")
    a = ap.parse_args()

    sched = json.load(open(a.schedule))
    packets = load_packets(a.symbols)
    groups = cluster(packets, a.gap)
    print(f"{len(packets)} packets in {len(groups)} groups, schedule has {len(sched['groups'])}")
    if len(groups) != len(sched["groups"]):
        print("group count does not match the schedule; widen or narrow --gap")
        sys.exit(1)

    mod = 1 << a.sf
    # One symbol vector per group, by majority vote over its repeats.
    voted = []
    for g in groups:
        c = collections.Counter(tuple(s) for _, s in g)
        best, n = c.most_common(1)[0]
        if a.gray:
            best = tuple(gray_decode(v, a.sf) for v in best)
        voted.append((best, n, len(g)))

    base = voted[0][0]
    print(f"\nbaseline (all-zero payload), {voted[0][1]}/{voted[0][2]} agreeing:")
    print("  " + " ".join(f"{v:4d}" for v in base))

    print("\nper bit, symbols that moved (index: xor / mod-difference):")
    xor_weights, sub_weights = [], []
    for i, (syms, n, tot) in enumerate(voted[1:]):
        name = sched["groups"][i + 1]["name"]
        diff = [(j, b ^ v, (v - b) % mod) for j, (b, v) in enumerate(zip(base, syms)) if b != v]
        xor_weights.append(sum(bin(d[1]).count("1") for d in diff))
        sub_weights.append(len(diff))
        moved = " ".join(f"{j}:{x:02x}/{s:02x}" for j, x, s in diff)
        flag = "" if n == tot else f"  ({n}/{tot} agree)"
        print(f"  {name:6} {len(diff):2d} symbols  {moved}{flag}")

    print(f"\nsymbols touched per bit: min {min(sub_weights)}, max {max(sub_weights)}, "
          f"mean {sum(sub_weights)/len(sub_weights):.1f}")
    print(f"total xor weight per bit: min {min(xor_weights)}, max {max(xor_weights)}, "
          f"mean {sum(xor_weights)/len(xor_weights):.1f}")


if __name__ == "__main__":
    main()
