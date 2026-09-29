#!/usr/bin/env python3
"""Run one end of a leader/follower gripper teleoperation link.

This is a runnable companion to the teleoperation section of the README. It is
meant to be started once per machine — one process per gripper:

    # Machine A (the leader you push by hand):
    python3 examples/teleop.py --mode master --channel can0 --host 192.168.1.20

    # Machine B (the follower that copies it):
    python3 examples/teleop.py --mode slave  --channel can0 --host 0.0.0.0

Both ends must share ``--master-id``. The default transport is plain UDP on
``--port``; it carries no authentication or encryption, so keep it on a trusted
network. Press Ctrl+C on either end to stop; the gripper holds its position.

This script talks to real hardware. It does not detect an object in the jaws,
and the follower holds its position on a leader dropout rather than going
slack, so it can clamp whatever is between the fingers. Keep a hand on the
power switch.
"""

from __future__ import annotations

import argparse
import sys
import time

from litegrip import LiteGrip, LiteGripError


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run one end of a LiteGrip leader/follower teleop link.")
    parser.add_argument(
        "--mode", required=True, choices=("master", "slave"),
        help="master = the leader you push by hand; slave = the follower")
    parser.add_argument(
        "--channel", default="can0", help="CAN interface (default: can0)")
    parser.add_argument(
        "--can-id", type=lambda s: int(s, 0), default=0x08,
        help="motor CAN ID (default: 0x08)")
    parser.add_argument(
        "--host", required=True,
        help="master: the follower's address; slave: the local bind address")
    parser.add_argument(
        "--port", type=int, default=7448, help="UDP port (default: 7448)")
    parser.add_argument(
        "--master-id", default="master",
        help="topic id both ends must agree on (default: master)")
    parser.add_argument(
        "--mount", choices=("normal", "reverse"), default=None,
        help="load a mount template instead of this channel's calibration")
    parser.add_argument(
        "--kp", type=float, default=None,
        help="follower stiffness (default: 100.0)")
    parser.add_argument(
        "--kd", type=float, default=None,
        help="follower damping (default: 2.0)")
    parser.add_argument(
        "--no-align", action="store_true",
        help="follower: skip the one-shot align to the first frame")
    parser.add_argument(
        "--watchdog", type=float, default=0.2,
        help="follower: hold position after this many seconds without a "
             "fresh frame (default: 0.2)")
    parser.add_argument(
        "--rate", type=float, default=50.0, help="loop rate in Hz (default: 50)")
    parser.add_argument(
        "--dry-run", action="store_true",
        help="print the resolved plan and exit without touching hardware")
    return parser


def _print_status(status: dict) -> None:
    age = status.get("last_frame_age_ms")
    age_txt = "-" if age is None else f"{age:6.1f}"
    openness = status.get("openness")
    open_txt = "-" if openness is None else f"{openness:5.3f}"
    print(f"frames={status.get('frames', 0):>7} "
          f"age_ms={age_txt} stale={str(status.get('stale', False)):>5} "
          f"openness={open_txt} loop_hz={status.get('loop_hz', 0.0):4.1f}",
          flush=True)


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)

    gripper = LiteGrip(channel=args.channel, can_id=args.can_id)
    if args.mount is not None:
        gripper.load_calibration(template=args.mount)
    else:
        gripper.load_calibration()
    print(f"mount={gripper.mount} closed={gripper.config.pos_closed_rad:+.4f} "
          f"open={gripper.config.pos_open_rad:+.4f} rad_to_mm={gripper.config.rad_to_mm}")

    if args.dry_run:
        print(f"dry run: would start {args.mode} on {args.channel} at "
              f"{args.host}:{args.port} (topic litegrip/teleop/{args.master_id})")
        return 0

    gripper.connect()
    gripper.enable()
    status = gripper.teleop_start(
        args.mode, host=args.host, port=args.port,
        kp=args.kp, kd=args.kd, align=not args.no_align,
        watchdog_s=args.watchdog, rate_hz=args.rate,
        master_id=args.master_id)
    print(f"teleop {args.mode} running; Ctrl+C to stop")
    _print_status(status)

    try:
        while True:
            time.sleep(1.0)
            _print_status(gripper.teleop_status())
    except KeyboardInterrupt:
        print("\nstopping")
    finally:
        gripper.teleop_stop()
        gripper.disconnect()
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except LiteGripError as error:
        print(f"error: {error}", file=sys.stderr)
        sys.exit(1)
