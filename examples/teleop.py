#!/usr/bin/env python3
"""Run one end of a leader/follower gripper teleoperation link.

This is a runnable companion to the teleoperation section of the README. It is
meant to be started once per machine — one process per gripper:

    # Machine A (the leader you push by hand):
    python3 examples/teleop.py --mode master --channel can0

    # Machine B (the follower that copies it):
    python3 examples/teleop.py --mode slave --channel can0 --host 192.168.1.20

Both ends must share ``--grip-id``. The default link is the point-to-point zenoh
transport used by the field teleoperation (``pip install litegrip[zenoh]``): the
leader listens on ``--port``, the follower connects to ``--host``. ``--link udp``
selects plain UDP instead, which carries no authentication or encryption — keep
either on a trusted network. Press Ctrl+C on either end to stop; the gripper
holds its position and does not disable.

This script talks to real hardware. It does not detect an object in the jaws,
and the follower holds its position on a leader dropout rather than going
slack, so it can clamp whatever is between the fingers. Keep a hand on the
power switch.

It runs from a source checkout as well as from an installed package: ``src/`` is
put on the import path below if ``litegrip`` is not installed yet.
"""

from __future__ import annotations

import argparse
import os
import sys
import time

# Like tests/_sdkpath.py: import the SDK straight out of the checkout, so the
# example works without `pip install -e .`. Inserted first, so the checkout wins
# over an installed copy — running the example exercises the code next to it.
_SRC = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "src")
if _SRC not in sys.path:
    sys.path.insert(0, _SRC)

from litegrip import (DEFAULT_GRIP_ID, DEFAULT_GRIP_PORT,  # noqa: E402
                      LiteGrip, LiteGripError)


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
        "--link", choices=("zenoh", "udp"), default="zenoh",
        help="transport (default: zenoh; udp is the plain-LAN fallback)")
    parser.add_argument(
        "--host", default=None,
        help="slave: the leader's address (the master only listens)")
    parser.add_argument(
        "--port", type=int, default=DEFAULT_GRIP_PORT,
        help=f"TCP (zenoh) or UDP port (default: {DEFAULT_GRIP_PORT})")
    parser.add_argument(
        "--grip-id", default=DEFAULT_GRIP_ID,
        help=f"topic id both ends must agree on (default: {DEFAULT_GRIP_ID})")
    parser.add_argument(
        "--mount", choices=("normal", "reverse"), default=None,
        help="load a mount template instead of this channel's calibration")
    parser.add_argument(
        "--kp", type=float, default=None,
        help="follower stiffness (default: the calibration's kp)")
    parser.add_argument(
        "--kd", type=float, default=None,
        help="follower damping (default: the calibration's kd)")
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
    extra = ""
    if status.get("matching") is not None:
        extra += f" matching={str(status['matching']):>5}"
    if status.get("rejected"):
        extra += f" rejected={status['rejected']}"
    if status.get("send_failed"):
        extra += f" send_failed={status['send_failed']}"
    if status.get("fault"):
        extra += f" fault={status['fault']}"
    print(f"frames={status.get('frames', 0):>7} "
          f"age_ms={age_txt} stale={str(status.get('stale', False)):>5} "
          f"openness={open_txt} "          f"loop_hz={status.get('loop_hz', 0.0):4.1f}"
          f"{extra}", flush=True)


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)

    gripper = LiteGrip(channel=args.channel, can_id=args.can_id)
    if args.mount is not None:
        gripper.load_calibration(template=args.mount)
    else:
        gripper.load_calibration()
    print(f"mount={gripper.mount} closed={gripper.config.pos_closed_rad:+.4f} "
          f"open={gripper.config.pos_open_rad:+.4f} rad_to_mm={gripper.config.rad_to_mm}")

    target = f"{args.host}:{args.port}" if args.host else f"*:{args.port}"
    if args.dry_run:
        print(f"dry run: would start {args.mode} on {args.channel} over "
              f"{args.link} at {target} "
              f"(topic litearm/v4/{args.grip_id}/gripper_teleop)")
        return 0

    gripper.connect()
    # enable() does not raise on failure — it returns a falsy EnableResult.
    result = gripper.enable()
    if not result.ok:
        code = None if result.state is None else result.state.error_code
        print(f"error: enable failed after {result.tries} tries "
              f"(last error_code={code}); check the 24V supply and that "
              f"{args.channel} is up at the right bitrate", file=sys.stderr)
        gripper.disconnect()
        return 1

    status = gripper.teleop_start(
        args.mode, link=args.link, host=args.host, port=args.port,
        grip_id=args.grip_id, kp=args.kp, kd=args.kd, align=not args.no_align,
        watchdog_s=args.watchdog, rate_hz=args.rate)
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
