#!/usr/bin/env python3
r"""
Fake fdcanusb on a pty — hardware-free double for the driver's conf-get path.

Speaks the fdcanusb line protocol (moteus_transport.h Fdcanusb):
  client -> device  'can send <arb hex> <data hex>[ flags]\n' per CAN frame
  device -> client  'OK\r\n' ack per send, 'rcv <arb hex> <data hex>\r\n' per reply
Diagnostic subframes (moteus_multiplex.h / moteus_protocol.h):
  0x40 write [ch len payload] · 0x42 poll [ch maxlen] -> 0x41 [ch varuint(len) payload]
Payloads pad to the CAN-FD DLC ladder with 0x50 (nop), as real firmware does.

Prints the pty slave path on stdout, then serves one client until SIGTERM.

Fault modes (mechanisms from the moteus client audit):
  clean            scripted replies land in the channel-1 buffer immediately
  stale            one bogus line (999.0) precedes the first conf get's reply,
                   as if a pre-flush line appeared after DiagnosticFlush ran
  drop             reply for conf get #--drop-index is withheld until the client's
                   5 empty polls are served (its timeout), THEN left in the buffer
  --lag            every reply reaches the buffer one poll late (models firmware
                   latency); combined with drop this sustains a one-register shift
"""
import argparse
import os
import signal
import sys
import tty

MOTOR_ID = 5
VALUES = {  # keep in sync with kScripted in conf_get_path_test.cpp
    "servo.default_accel_limit": 2.0,
    "servo.default_velocity_limit": 0.15,
    "servo.max_velocity": 0.35,
    "servopos.position_min": -0.45,
    "servopos.position_max": 0.55,
    "servo.max_current_A": 9.0,
    "servo.pid_position.kp": 17000.0,
    "servo.pid_position.ki": 123.0,
    "servo.pid_position.kd": 3500.0,
    "servo.max_voltage": 30.0,
    "servo.max_power_W": 250.0,
    "servo.default_timeout_s": 0.8,
}
STALE_LINE = b"999.000000\r\n"
DLC = (0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64)


def round_dlc(n):
    return next((d for d in DLC if n <= d), n)


class Emulator:
    def __init__(self, master, args):
        self.master = master
        self.args = args
        self.console = b""   # channel-1 command bytes until '\n'
        self.out_buf = b""   # channel-1 output awaiting polls
        self.stash = None    # (reply_bytes, polls_until_release)
        self.cmd_no = 0

    def send_frame(self, data):
        pad = b"\x50" * (round_dlc(len(data)) - len(data))
        arb = MOTOR_ID << 8  # source=us, destination=host 0
        os.write(self.master, f"rcv {arb:04x} {(data + pad).hex()}\r\n".encode())

    def on_can(self, arb, data):
        os.write(self.master, b"OK\r\n")  # fdcanusb acks every send
        if (arb & 0x7F) != MOTOR_ID or not data:
            return
        op = data[0]
        if op == 0x40 and len(data) >= 3:        # DiagnosticWrite
            ch, ln = data[1], data[2]
            if ch == 1:
                self.console += data[3:3 + ln]
                while b"\n" in self.console:
                    line, self.console = self.console.split(b"\n", 1)
                    self.on_command(line.strip(b"\r").decode(errors="replace"))
        elif op == 0x42 and len(data) >= 3:      # DiagnosticRead poll
            ch, maxlen = data[1], data[2]
            chunk = b""
            if ch == 1:
                chunk, self.out_buf = self.out_buf[:maxlen], self.out_buf[maxlen:]
            self.send_frame(bytes([0x41, ch, len(chunk)]) + chunk)
            if self.stash:
                reply, hold = self.stash
                self.stash = (reply, hold - 1)
                if hold - 1 <= 0:
                    self.out_buf += reply
                    self.stash = None
        elif arb & 0x8000:                        # register-mode query (SetQuery)
            self.send_frame(bytes([0x21, 0x00, 0x00]))  # reply: mode=0 stopped

    def on_command(self, cmd):
        if not cmd.startswith("conf get "):
            self.out_buf += b"ERR unknown command\r\n"
            return
        reg = cmd[len("conf get "):].strip()
        self.cmd_no += 1
        val = VALUES.get(reg)
        reply = f"{val:.6f}\r\n".encode() if val is not None else b"ERR unknown group\r\n"
        if self.args.mode == "stale" and self.cmd_no == 1:
            self.out_buf += STALE_LINE
        if self.stash:                            # never lose a queued reply
            self.out_buf += self.stash[0]
            self.stash = None
        if self.args.mode == "drop" and self.cmd_no == self.args.drop_index:
            self.stash = (reply, 5)  # client gives up after its 5th empty poll
        elif self.args.lag:
            self.stash = (reply, 1)
        else:
            self.out_buf += reply


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", choices=["clean", "stale", "drop"], default="clean")
    ap.add_argument("--drop-index", type=int, default=7)  # 1-based conf get count
    ap.add_argument("--lag", action="store_true")
    args = ap.parse_args()

    master, slave = os.openpty()
    tty.setraw(slave)  # no echo/CRLF mangling before the client sets its termios
    print(os.ttyname(slave), flush=True)
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))

    emu = Emulator(master, args)
    buf = b""
    while True:
        try:
            data = os.read(master, 4096)
        except OSError:
            return
        if not data:
            return
        buf += data
        while b"\n" in buf:
            line, buf = buf.split(b"\n", 1)
            parts = line.decode(errors="replace").split()
            if len(parts) >= 4 and parts[:2] == ["can", "send"]:
                try:
                    emu.on_can(int(parts[2], 16), bytes.fromhex(parts[3]))
                except ValueError:
                    pass


if __name__ == "__main__":
    main()
