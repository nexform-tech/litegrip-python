#!/usr/bin/env python3
"""Teleoperation slave: follow the master opening received over UDP.

Run this on the follower gripper, then start examples/teleop_master.py:

    python3 examples/teleop_slave.py --channel can1 --peer 127.0.0.1 \
        --port 17449 --master-port 17448

The received value is an absolute opening in mm and is converted with THIS
gripper's calibration, so the two units may have different zero, direction or
scale.  A shorter follower stroke still clamps at its own travel.

The slave publishes its measured squeeze back to the master; the master renders
it only when started with ``--force-feedback``.  If no master frame arrives for
``--watchdog-ms``, the last target is held (the motor keeps receiving frames).
Ctrl+C stops the loop and disables the gripper.
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
        description="Teleoperation slave: follow a UDP opening target.")
    parser.add_argument("--channel", default="can1",
                        help="follower CAN interface (default: can1)")
    parser.add_argument("--can-id", type=_hex, default=0x08)
    parser.add_argument("--mst-id", type=_hex, default=0x18)
    parser.add_argument("--calib", default=None,
                        help="explicit calibration JSON")
    parser.add_argument("--peer", default="127.0.0.1",
                        help="master host for force feedback (default: 127.0.0.1)")
    parser.add_argument("--port", type=int, default=17449,
                        help="local port receiving openings (default: 17449)")
    parser.add_argument("--master-port", type=int, default=17448,
                        help="master port receiving force (default: 17448)")
    parser.add_argument("--rate", type=float, default=200.0,
                        help="follow rate in Hz (default: 200)")
    parser.add_argument("--kp", type=float, default=100.0,
                        help="MIT position stiffness (default: 100)")
    parser.add_argument("--kd", type=float, default=2.0,
                        help="MIT damping (default: 2)")
    parser.add_argument("--watchdog-ms", type=float, default=200.0,
                        help="hold after this long without a master frame "
                             "(default: 200 ms)")
    parser.add_argument("--yes", action="store_true",
                        help="skip the confirmation prompt")
    parser.add_argument("--verbose", action="store_true")
    return parser


def _confirm(args) -> bool:
    if args.yes:
        return True
    if not sys.stdin.isatty():
        print("error: this moves the follower; re-run with --yes if intended",
              file=sys.stderr)
        return False
    print("The follower will start following UDP targets. Clear the travel.")
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
    if args.watchdog_ms <= 0.0:
        print("error: --watchdog-ms must be positive", file=sys.stderr)
        return 2

    follower = Gripper(channel=args.channel, can_id=args.can_id,
                       mst_id=args.mst_id)
    if args.calib is not None:
        follower.load_calibration(args.calib)
    if not follower.config.calibrated:
        print("error: no usable calibration — run examples/calibrate.py first",
              file=sys.stderr)
        return 2
    if not _confirm(args):
        return 1

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("0.0.0.0", args.port))
    sock.setblocking(False)
    master = (args.peer, args.master_port)
    period = 1.0 / args.rate
    watchdog_s = args.watchdog_ms / 1000.0
    last_status = 0.0
    last_rx = None
    target_mm = None
    warned_stale = False

    def send_force(force_n: float) -> None:
        try:
            sock.sendto(_UDP.pack(float(force_n)), master)
        except OSError:
            pass

    try:
        follower.connect()
        follower.enable()
        follower.refresh_status()
        initial = follower.get_state(wait=True)
        if not initial.fresh:
            print("error: no fresh status frame from the follower",
                  file=sys.stderr)
            return 2
        target_mm = initial.position_mm
        print(f"slave on {args.channel}; {args.rate:.0f} Hz; listening on "
              f"port {args.port}; master {args.peer}:{args.master_port}; "
              f"watchdog {args.watchdog_ms:.0f} ms")

        while True:
            t0 = time.monotonic()
            value = _recv_latest(sock)
            if value is not None:
                target_mm = value
                last_rx = t0
                warned_stale = False
            elif last_rx is None:
                # No master yet: hold the current opening.
                target_mm = follower.get_gripper_distance()
            elif t0 - last_rx > watchdog_s and not warned_stale:
                print(f"warning: no master frame for {args.watchdog_ms:.0f} ms "
                      "— holding the last target", file=sys.stderr)
                warned_stale = True

            follower.follow_gripper_distance(
                target_mm, kp=args.kp, kd=args.kd, force_feedback=send_force)

            if t0 - last_status >= 1.0:
                last_status = t0
                state = follower.get_state(wait=False)
                age = "n/a" if last_rx is None else f"{t0 - last_rx:.2f}s"
                print(f"slave target {target_mm:6.1f} mm | "
                      f"position {state.position_mm:6.1f} mm | "
                      f"force {state.force_n:5.1f} N | master age {age}")
            time.sleep(max(0.0, period - (time.monotonic() - t0)))
    except KeyboardInterrupt:
        print("\nstopping")
        return 0
    except LiteGripError as error:
        print(f"error: {error}", file=sys.stderr)
        return 2
    finally:
        try:
            follower.disable()
        except LiteGripError:
            pass
        follower.disconnect()
        sock.close()


if __name__ == "__main__":
    sys.exit(main())
