#!/usr/bin/env python3
"""Probe both mechanical limits and save the per-channel calibration.

The gripper drives itself gently against each end stop with low stiffness and
writes the measured limits and mm/rad scale to
``~/.litegrip/<channel>_calibration.json``.  It is blocking, and the jaws move:
**clear the travel first**.

    python3 examples/calibrate.py --help
    python3 examples/calibrate.py --channel can0
    python3 examples/calibrate.py --channel can0 --yes --save-to /tmp/bench.json

The direction is not discovered by the probe — both end stops look the same to
a stall detector.  It comes from the calibration already loaded (``--calib``,
``LITEGRIP_CALIB``, the per-channel file, or the factory file), so a
reverse-mounted unit keeps its reverse direction.
"""

from __future__ import annotations

import argparse
import logging
import os
import sys

_SRC = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "src")
if _SRC not in sys.path:
    sys.path.insert(0, _SRC)

from litegrip import Gripper, LiteGripError, default_calibration_path  # noqa: E402


def _hex(value: str) -> int:
    return int(value, 0)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Calibrate a LiteGrip: probe both stops, then save the "
                    "result.")
    parser.add_argument("--channel", default="can0",
                        help="CAN interface (default: can0)")
    parser.add_argument("--can-id", type=_hex, default=0x08,
                        help="motor CAN ID (default: 0x08)")
    parser.add_argument("--mst-id", type=_hex, default=0x18,
                        help="status-frame master ID (default: 0x18)")
    parser.add_argument("--calib", default=None,
                        help="existing calibration used for direction and "
                             "gains; default search is LITEGRIP_CALIB, then "
                             "~/.litegrip/<channel>_calibration.json, then the "
                             "factory calibration")
    parser.add_argument("--save-to", default=None,
                        help="destination JSON (default: "
                             "~/.litegrip/<channel>_calibration.json)")
    parser.add_argument("--yes", action="store_true",
                        help="skip the confirmation prompt")
    parser.add_argument("--verbose", action="store_true",
                        help="log each probe step")
    return parser


def _confirm(args) -> bool:
    if args.yes:
        return True
    if not sys.stdin.isatty():
        print("error: calibration moves the jaws against both stops and stdin "
              "is not a terminal; re-run with --yes if that is what you want",
              file=sys.stderr)
        return False
    print("\nThe jaws will drive slowly into each mechanical stop.")
    print("Clear the travel, and keep hands out of the jaws.")
    try:
        input("Press Enter to start, Ctrl+C to abort: ")
    except (EOFError, KeyboardInterrupt):
        print()
        return False
    return True


def _run(args) -> int:
    g = Gripper(channel=args.channel, can_id=args.can_id, mst_id=args.mst_id)
    if args.calib is not None:
        if not g.load_calibration(args.calib):
            print(f"error: no calibration could be read from {args.calib}",
                  file=sys.stderr)
            return 2
    direction = "rad increases while closing" if g.config.close_sign > 0 else \
        "rad decreases while closing"
    print(f"direction from the loaded calibration: {direction}")

    if not _confirm(args):
        print("aborted, nothing was sent on the bus")
        return 1

    try:
        g.connect()
        if not g.enable():
            print("error: enable failed — check the 24 V supply and bitrate",
                  file=sys.stderr)
            return 1
        # Probe; save only to the destination the caller asked for. The
        # default per-channel file stays untouched when --save-to is given.
        data = g.calibrate(save=args.save_to is None)
        if args.save_to is not None:
            path = g.save_calibration(args.save_to)
        else:
            path = default_calibration_path(args.channel)
    finally:
        g.disconnect()

    print("\ncalibration complete:")
    print(f"  closed (0 mm): {data.zero_position:+.6f} rad")
    print(f"  open          : {data.max_position:+.6f} rad")
    print(f"  travel        : {data.travel_range:.6f} rad  "
          f"({data.travel_mm:.1f} mm)")
    print(f"  scale         : {data.rad_to_mm:.2f} mm/rad")
    print(f"  saved to      : {path}")
    if args.save_to is None:
        print(f"  (default per-channel path: "
              f"{default_calibration_path(args.channel)})")
    return 0


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    # INFO shows the probe progress; the SDK does not print to stdout.
    logging.basicConfig(level=logging.INFO if args.verbose else logging.WARNING,
                        format="%(levelname)s %(name)s: %(message)s")
    return _run(args)


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\ninterrupted — motor disabled, jaws left where they stopped",
              file=sys.stderr)
        sys.exit(130)
    except LiteGripError as error:
        print(f"error: {error}", file=sys.stderr)
        sys.exit(1)
