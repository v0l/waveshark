#!/usr/bin/env python3
"""Read a WaveShark packet log (.wspkt) and analyse the unknown FSK bursts.

The format is documented in crates/app/src/packetlog.rs. This is the offline
counterpart to the burst pane: slice the pulse timings, align to the preamble,
and diff the aligned frames against each other so the constant fields (address,
type) separate from the varying ones (counter, payload).
"""
import struct
import sys
from collections import Counter, defaultdict

HEAD_LEN = 1 + 1 + 2 + 8 + 8 + 4 + 4 + 4
KEYING = {0: None, 1: "OOK", 2: "ASK", 3: "FSK", 4: "4-FSK", 5: "MSK"}


def parse(buf):
    if buf[:6] != b"WSPKT\0":
        raise SystemExit("not a wspkt file")
    at = 8
    out = []
    while at + 4 <= len(buf):
        (ln,) = struct.unpack_from("<I", buf, at)
        at += 4
        if ln < HEAD_LEN or at + ln > len(buf):
            break
        r = buf[at : at + ln]
        at += ln
        kind, keying, count = r[0], r[1], struct.unpack_from("<H", r, 2)[0]
        at_us, center, bw = struct.unpack_from("<QQI", r, 4)
        rssi, snr = struct.unpack_from("<ff", r, 24)
        body = r[HEAD_LEN:]
        rec = dict(kind=kind, keying=KEYING.get(keying), at_us=at_us,
                   center_hz=center, bandwidth_hz=bw, rssi=rssi, snr=snr,
                   measure=None, pulses=[], frame=None)
        if kind == 3:
            m, body = take_measure(body)
            if m is None:
                continue
            rec["measure"] = m
        if kind in (1, 3):
            n = min(count, len(body) // 8)
            rec["pulses"] = [struct.unpack_from("<II", body, k * 8) for k in range(n)]
        elif kind == 2:
            rec["frame"] = body[: min(count, len(body))]
        else:
            continue
        out.append(rec)
    return out


def take_measure(body):
    at = 0
    strs = []
    for _ in range(3):
        if at + 2 > len(body):
            return None, body
        (n,) = struct.unpack_from("<H", body, at)
        at += 2
        strs.append(body[at : at + n].decode("utf8", "replace"))
        at += n
    if at + 28 > len(body):
        return None, body
    conf, dur = struct.unpack_from("<fI", body, at)
    nums = struct.unpack_from("<5f", body, at + 8)
    m = dict(modulation=strs[0], front=strs[1], mode=strs[2], confidence=conf,
             duration_us=dur, bandwidth_hz=nums[0], baud=nums[1],
             separation_hz=nums[2], sweep_hz_s=nums[3], symbol_period_us=nums[4])
    return m, body[at + 28 :]


def nrz_bits(pulses, sym_us, reset_us):
    """slice_nrz from crates/decode/src/slicer.rs."""
    bits = []
    max_zeros = max(1, int(reset_us / sym_us))
    for mark, gap in pulses:
        bits += [1] * int(round(mark / sym_us))
        bits += [0] * min(int(round(gap / sym_us)), max_zeros)
    return bits


def align(bits, min_pre=12):
    """framing::frame_from_preamble: cut at the end of the longest alternating run."""
    best = (0, 0)
    start = 0
    for i in range(1, len(bits)):
        if bits[i] == bits[i - 1]:
            if i - start > best[1]:
                best = (start, i - start)
            start = i
    if len(bits) - start > best[1]:
        best = (start, len(bits) - start)
    if best[1] < min_pre or best[0] + best[1] >= len(bits):
        return None
    return best[0] + best[1], best[1]


def pack(bits):
    out = bytearray()
    for i in range(0, len(bits) // 8 * 8, 8):
        v = 0
        for b in bits[i : i + 8]:
            v = (v << 1) | b
        out.append(v)
    return bytes(out)


def hexs(b, n=None):
    return " ".join(f"{x:02x}" for x in (b if n is None else b[:n]))


def main():
    path = sys.argv[1]
    lo, hi = (float(sys.argv[2]), float(sys.argv[3])) if len(sys.argv) > 3 else (0, 2e9)
    recs = parse(open(path, "rb").read())
    band = [r for r in recs if lo <= r["center_hz"] <= hi and r["pulses"]]
    print(f"{len(recs)} records, {len(band)} in band with timings")

    freqs = Counter(round(r["center_hz"] / 1e4) * 10 for r in band)
    print("centres (kHz):", freqs.most_common(12))

    frames = []
    for r in band:
        m = r["measure"] or {}
        sym = m.get("symbol_period_us") or 0
        if not sym:
            widths = sorted(w for p in r["pulses"][:-1] for w in p if w)
            if len(widths) < 8:
                continue
            sym = widths[len(widths) // 5]
        reset = r["pulses"][-1][1] or sym * 20
        bits = nrz_bits(r["pulses"], sym, reset)
        a = align(bits)
        if not a:
            continue
        cut, pre = a
        frames.append(dict(rec=r, bytes=pack(bits[cut:]), preamble=pre, sym=sym))

    print(f"{len(frames)} aligned")
    groups = defaultdict(list)
    for f in frames:
        groups[f["bytes"][:2]].append(f)
    for sync, fs in sorted(groups.items(), key=lambda kv: -len(kv[1]))[:6]:
        print(f"\n=== sync {hexs(sync)}  {len(fs)} frames ===")
        for f in fs[:12]:
            r = f["rec"]
            print(f"  {r['at_us']/1e6:9.3f}s {r['center_hz']/1e6:10.4f}M "
                  f"rssi {r['rssi']:6.1f} sym {f['sym']:5.1f} pre {f['preamble']:3d}  "
                  f"{hexs(f['bytes'], 20)}")
        if len(fs) > 1:
            n = min(len(x["bytes"]) for x in fs)
            const = [i for i in range(n) if len({x["bytes"][i] for x in fs}) == 1]
            print(f"  constant byte offsets over {len(fs)} frames "
                  f"(common length {n}): {const}")


if __name__ == "__main__":
    main()
