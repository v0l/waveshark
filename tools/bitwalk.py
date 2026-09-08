#!/usr/bin/env python3
"""Walk one payload bit at a time through a bench transmitter, for reading a
LoRa encoder off the air instead of guessing at it.

Everything between payload bits and symbol bits in LoRa is linear over GF(2):
the coding, the interleaver, the Gray map, and whitening, which is an XOR with
a fixed sequence. So the encoder is s = M.p XOR c. An all-zero payload gives c
directly, and a payload with exactly one bit set gives that bit's column of M
once c is XORed back out. 8N+1 transmissions read the whole encoder, whatever
the interleaver's geometry turns out to be, and that includes the SX1280 long
interleaved coding rates nobody has published.

Groups are separated by a long gap and repeated within a short one, so a
dropped packet costs a repeat rather than shifting every later assignment.

    tools/bitwalk.py --port /dev/ttyUSB1 --bytes 8 --repeat 6 --out walk.json

Run it while a receiver records, then read the symbols out of the capture with
`cargo run --release -p nodes --example lora_symbols`.
"""

import argparse
import json
import random
import sys
import time

sys.path.insert(0, "/home/kieran/git/sub-ghz-modem/tools")
import proto  # noqa: E402
import serial  # noqa: E402


def send(ser, dec, msg_type, value=b"", timeout=3.0):
    ser.write(proto.frame(msg_type, value))
    ser.flush()
    end = time.time() + timeout
    while time.time() < end:
        for typ, val in dec.feed(ser.read(256)):
            if typ in (proto.TX_DONE, proto.ACK):
                return True
            if typ == proto.ERR:
                print("  device error:", val.hex())
                return False
    print("  timeout")
    return False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyUSB1")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--bytes", type=int, default=8, help="payload length")
    ap.add_argument("--repeat", type=int, default=6, help="transmissions per payload")
    ap.add_argument("--gap", type=float, default=0.12, help="within a group")
    ap.add_argument("--group-gap", type=float, default=0.6, help="between groups")
    ap.add_argument("--bits", type=int, default=0, help="stop after N bits, 0 = all")
    ap.add_argument("--random", type=int, default=0,
                    help="send N random payloads instead of walking bits, to check"
                         " a matrix against payloads it was not built from")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--out", default="walk.json")
    a = ap.parse_args()

    ser = serial.Serial(a.port, a.baud, timeout=0.05)
    dec = proto.Decoder()
    time.sleep(0.3)
    ser.reset_input_buffer()

    payloads = [bytes(a.bytes)]
    if a.random:
        rng = random.Random(a.seed)
        payloads += [bytes(rng.randrange(256) for _ in range(a.bytes))
                     for _ in range(a.random)]
    else:
        nbits = a.bits or a.bytes * 8
        for bit in range(nbits):
            p = bytearray(a.bytes)
            p[bit // 8] = 1 << (bit % 8)
            payloads.append(bytes(p))

    t0 = time.time()
    groups = []
    for i, p in enumerate(payloads):
        name = "zero" if i == 0 else (f"rnd{i - 1}" if a.random else f"bit{i - 1}")
        start = time.time() - t0
        sent = 0
        for r in range(a.repeat):
            if send(ser, dec, proto.TX, p, timeout=max(4.0, a.gap)):
                sent += 1
            time.sleep(a.gap)
        groups.append({
            "name": name,
            "payload": p.hex(),
            "start_s": round(start, 3),
            "end_s": round(time.time() - t0, 3),
            "sent": sent,
        })
        print(f"{name:8} {p.hex()}  {sent}/{a.repeat} at {start:6.2f} s", flush=True)
        time.sleep(a.group_gap)

    with open(a.out, "w") as f:
        json.dump({"payload_bytes": a.bytes, "repeat": a.repeat, "groups": groups}, f, indent=1)
    print(f"{len(groups)} groups, {time.time() - t0:.1f} s, schedule in {a.out}")


if __name__ == "__main__":
    main()
