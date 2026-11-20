#!/usr/bin/env python3
"""Teleoperation master: read this gripper's opening and publish it over UDP.

Run the slave example on the follower side:

    python3 examples/teleop_slave.py --channel can1 \
        --peer 127.0.0.1 --port 17449 --master-port 17448

Then run the master on the leader:

    python3 examples/teleop_master.py --channel can0 --peer 127.0.0.1 \
        --port 17448 --slave-port 17449

The leader stays enabled but is sent zero-gain MIT frames, so it is
back-drivable.  With ``--force-feedback`` the slave's squeeze is rendered as
resistance; without it, force feedback stays off (the SDK default).  The master
must be calibrated; the slave maps the received mm through its own calibration.

This is a demonstration, not a production teleop stack: there is no
authentication, no retransmission and no ready handshake.  Clear the travel and
support the leader before enabling it.  Ctrl+C stops the loop and disables the
gripper.
"""

from __future__ import annotations

import argparse
import logging
import math
import os
import socket
import struct
import sys
import time

_SRC = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "src")
if _SRC not in sys.path:
    sys.path.insert(0, _SRC)

from litegrip import Gripper, LiteGripError  # noqa: E402

_UDP = struct.Struct("!d")


def _hex(value: str) -> int:
    return int(value, 0)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Teleoperation master: publish the leader opening over UDP.")
    parser.add_argument("--channel", default="can0",
                        help="leader CAN interface (default: can0)")
    parser.add_argument("--can-id", type=_hex, default=0x08)
    parser.add_argument("--mst-id", type=_hex, default=0x18)
    parser.add_argument("--calib", default=None,
                        help="explicit calibration JSON")
    parser.add_argument("--peer", default="127.0.0.1",
                        help="slave host (default: 127.0.0.1)")
    parser.add_argument("--port", type=int, default=17448,
                        help="local port receiving force feedback (default: 17448)")
    parser.add_argument("--slave-port", type=int, default=17449,
                        help="slave port receiving the opening (default: 17449)")
    parser.add_argument("--rate", type=float, default=200.0,
                        help="loop rate in Hz (default: 200)")
    parser.add_argument("--force-feedback", action="store_true",
                        help="render the slave's squeeze (default: off)")
    parser.add_argument("--yes", action="store_true",
                        help="skip the confirmation prompt")
    parser.add_argument("--verbose", action="store_true")
    return parser


def _confirm(args) -> bool:
    if args.yes:
        return True
    if not sys.stdin.isatty():
        print("error: this enables the leader; re-run with --yes if intended",
              file=sys.stderr)
        return False
    print("The leader will be enabled and held limp by zero-gain frames.")
    print("Support it before continuing; clear the jaw travel.")
    try:
        input("Press Enter to start, Ctrl+C to abort: ")
    except (EOFError, KeyboardInterrupt):
        print()
        return False
    return True


def _recv_latest(sock: socket.socket):
    latest = None
    while True:
        try:
            data, _ = sock.recvfrom(64)
        except BlockingIOError:
            break
        if len(data) != _UDP.size:
            continue
        value = _UDP.unpack(data)[0]
        if math.isfinite(value):
            latest = value
    return latest


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    if args.verbose:
        logging.basicConfig(level=logging.INFO,
                            format="%(asctime)s - %(name)s - %(levelname)s - %(message)s")
    if args.rate <= 0.0:
        print("error: --rate must be positive", file=sys.stderr)
        return 2

    leader = Gripper(channel=args.channel, can_id=args.can_id,
                     mst_id=args.mst_id)
    if args.calib is not None:
        leader.load_calibration(args.calib)
    if not leader.config.calibrated:
        print("error: no usable calibration — run examples/calibrate.py first",
              file=sys.stderr)
        return 2
    if not _confirm(args):
        return 1

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("0.0.0.0", args.port))
    sock.setblocking(False)
    peer = (args.peer, args.slave_port)
    period = 1.0 / args.rate
    last_status = 0.0
    last_force = 0.0
    last_force_rx = None

    try:
        leader.connect()
        leader.enable()
        leader.configure_force_feedback(enabled=args.force_feedback)
        leader.refresh_status()
        initial = leader.get_state(wait=True)
        if not initial.fresh:
            print("error: no fresh status frame from the leader", file=sys.stderr)
            return 2
        print(f"master on {args.channel}; {args.rate:.0f} Hz -> "
              f"{args.peer}:{args.slave_port}; feedback port {args.port}; "
              f"force feedback {'on' if args.force_feedback else 'off'}")

        while True:
            t0 = time.monotonic()
            force = _recv_latest(sock)
            if force is not None:
                last_force = force
                last_force_rx = t0

            state = leader.get_state(wait=False)
            distance = state.position_mm
            if args.force_feedback:
                age = (0.0 if last_force_rx is None
                       else t0 - last_force_rx)
                leader.render_force_feedback(last_force, age_s=age)
            else:
                # Zero-gain frame: keeps the leader enabled and back-drivable.
                leader.follow_gripper_distance(distance, kp=0.0, kd=0.0)

            try:
                sock.sendto(_UDP.pack(distance), peer)
            except OSError as error:
                print(f"warning: UDP send failed: {error}", file=sys.stderr)

            if t0 - last_status >= 1.0:
                last_status = t0
                age_text = "n/a" if last_force_rx is None else f"{t0 - last_force_rx:.2f}s"
                print(f"master {distance:6.1f} mm | force {last_force:5.1f} N "
                      f"(age {age_text})")
            time.sleep(max(0.0, period - (time.monotonic() - t0)))
    except KeyboardInterrupt:
        print("\nstopping")
        return 0
    except LiteGripError as error:
        print(f"error: {error}", file=sys.stderr)
        return 2
    finally:
        try:
            leader.disable()
        except LiteGripError:
            pass
        leader.disconnect()
        sock.close()


if __name__ == "__main__":
    sys.exit(main())
