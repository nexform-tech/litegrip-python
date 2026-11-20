#!/usr/bin/env python3
"""Move to an absolute opening (mm) or motor angle (rad).

    python3 examples/move_to.py --help
    python3 examples/move_to.py --channel can0 --distance 40 --yes
    python3 examples/move_to.py --channel can0 --angle -0.5 --speed 20 --yes

``--distance`` is the opening from the closed zero, clamped into the calibrated
travel.  ``--angle`` is the raw motor angle; both are single blocking moves.
"""

from __future__ import annotations

import argparse
import logging
import os
import sys

_SRC = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "src")
if _SRC not in sys.path:
    sys.path.insert(0, _SRC)

from litegrip import Gripper, LiteGripError  # noqa: E402


def _hex(value: str) -> int:
    return int(value, 0)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Move a LiteGrip to an absolute opening or motor angle.")
    parser.add_argument("--channel", default="can0",
                        help="CAN interface (default: can0)")
    parser.add_argument("--can-id", type=_hex, default=0x08,
                        help="motor CAN ID (default: 0x08)")
    parser.add_argument("--mst-id", type=_hex, default=0x18,
                        help="status-frame master ID (default: 0x18)")
    parser.add_argument("--calib", default=None,
                        help="explicit calibration JSON; default search: "
                             "LITEGRIP_CALIB, per-channel file, factory file")
    target = parser.add_mutually_exclusive_group(required=True)
    target.add_argument("--distance", type=float,
                        help="absolute opening in mm (0 = closed)")
    target.add_argument("--angle", type=float,
                        help="absolute motor angle in rad")
    parser.add_argument("--speed", type=float, default=30.0,
                        help="move speed in mm/s (default: 30)")
    parser.add_argument("--yes", action="store_true",
                        help="skip the confirmation prompt")
    parser.add_argument("--verbose", action="store_true",
                        help="turn on the SDK's logging")
    return parser


def _confirm(args) -> bool:
    if args.yes:
        return True
    if not sys.stdin.isatty():
        print("error: this moves the jaws; re-run with --yes if intended",
              file=sys.stderr)
        return False
    what = (f"{args.distance:g} mm" if args.distance is not None
            else f"{args.angle:g} rad")
    print(f"About to move to {what}. Clear the travel first.")
    try:
        input("Press Enter to start, Ctrl+C to abort: ")
    except (EOFError, KeyboardInterrupt):
        print()
        return False
    return True


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    if args.verbose:
        logging.basicConfig(level=logging.INFO,
                            format="%(asctime)s - %(name)s - %(levelname)s - %(message)s")

    g = Gripper(channel=args.channel, can_id=args.can_id, mst_id=args.mst_id)
    if args.calib is not None:
        g.load_calibration(args.calib)
    if not g.config.calibrated:
        print("error: no usable calibration — run examples/calibrate.py first",
              file=sys.stderr)
        return 2
    if not _confirm(args):
        return 1

    try:
        g.connect()
        g.enable()
        if args.distance is not None:
            ok = g.set_gripper_distance(args.distance, args.speed)
            label = f"{args.distance:g} mm"
        else:
            ok = g.set_motor_angle(args.angle, args.speed)
            label = f"{args.angle:g} rad"
        state = g.get_state()
        print(f"target {label}: {'reached' if ok else 'NOT reached'}")
        print(f"  position {state.position_rad:+.4f} rad ({state.position_mm:+.1f} mm)")
        return 0 if ok else 1
    except LiteGripError as error:
        print(f"error: {error}", file=sys.stderr)
        return 2
    finally:
        g.disconnect()


if __name__ == "__main__":
    sys.exit(main())
