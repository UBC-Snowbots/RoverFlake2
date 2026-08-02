#!/usr/bin/env python3
"""One-time limit switch setup for the arm's moteus controllers.

Configures aux2.pins.0 (AUX2/ABS connector pin 2) as a digital input and
persists it to flash. Wiring contract: NC switch from ABS pin 2 to pin 4
(GND); the r4.11 hard 2k pull-up makes the pin HIGH when pressed/unplugged.

Run with the moteus_driver node STOPPED (needs exclusive fdcanusb access):
    ./setup_limit_switches.py                 # configure ids 1-4
    ./setup_limit_switches.py --targets 2,3   # subset
    ./setup_limit_switches.py --monitor       # live switch state readout
"""
import argparse
import asyncio

import moteus

CONFIG_CMDS = [
    b"conf set aux2.pins.0.mode 14",   # 14 = digital input
    b"conf set aux2.pins.0.pull 1",    # no-op on r4.11 (hard pullups); correct elsewhere
    b"conf write",
]


async def configure(ids):
    for i in ids:
        stream = moteus.Stream(moteus.Controller(id=i))
        for cmd in CONFIG_CMDS:
            print(f"[id {i}] {cmd.decode()}")
            await stream.command(cmd)
        print(f"[id {i}] done — persisted to flash")


async def monitor(ids):
    qr = moteus.QueryResolution()
    qr.aux2_gpio = moteus.INT8
    ctrls = [(i, moteus.Controller(id=i, query_resolution=qr)) for i in ids]
    print("aux2 GPIO monitor (bit0 = aux2.pins.0 = ABS pin 2). Ctrl-C to stop.")
    while True:
        parts = []
        for i, c in ctrls:
            try:
                r = await asyncio.wait_for(c.query(), timeout=0.1)
                gpio = r.values[moteus.Register.AUX2_GPIO_STATUS]
                state = "PRESSED " if (gpio & 1) else "released"
                parts.append(f"id{i}: {state}({gpio:#04x})")
            except asyncio.TimeoutError:
                parts.append(f"id{i}: no reply  ")
        print("  ".join(parts), end="\r", flush=True)
        await asyncio.sleep(0.1)


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--targets", default="1,2,3,4",
                   help="comma-separated moteus CAN ids (default 1,2,3,4)")
    p.add_argument("--monitor", action="store_true",
                   help="live-print switch state instead of configuring")
    args = p.parse_args()
    ids = [int(t) for t in args.targets.split(",")]
    asyncio.run(monitor(ids) if args.monitor else configure(ids))


if __name__ == "__main__":
    main()
