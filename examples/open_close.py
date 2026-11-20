#!/usr/bin/env python3
"""Open and close the gripper, with a status line for each move.

    python3 examples/open_close.py --help
    python3 examples/open_close.py --channel can0 --cycles 3 --yes

Clear the jaw travel first: both moves press lightly onto the calibrated stops.
The script exits non-zero if any move does not report success.
"""

from __future__ import annotations

import argparse
import logging
import os
import sys
import time

_SRC = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "src")
if _SRC not in sys.path:
    sys.path.insert(0, _SRC)

from litegrip import Gripper, LiteGripError  # noqa: E402


def _hex(value: str) -> int:
    return int(value, 0)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Open and close a LiteGrip, reporting each move.")
    parser.add_argument("--channel", default="can0",
                        help="CAN interface (default: can0)")
    parser.add_argument("--can-id", type=_hex, default=0x08,
                        help="motor CAN ID (default: 0x08)")
    parser.add_argument("--mst-id", type=_hex, default=0x18,
                        help="status-frame master ID (default: 0x18)")
    parser.add_argument("--calib", default=None,
                        help="explicit calibration JSON; default search: "
                             "LITEGRIP_CALIB, per-channel file, factory file")
    parser.add_argument("--speed", type=float, default=20.0,
                        help="move speed in mm/s (default: 20)")
    parser.add_argument("--cycles", type=int, default=1,
                        help="open/close cycles (default: 1)")
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
    print("About to drive the jaws open and closed. Clear the travel first.")
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
        for i in range(args.cycles):
            print(f"\ncycle {i + 1}/{args.cycles}: open")
            if not g.open(args.speed):
                print("error: open did not reach the calibrated open limit",
                      file=sys.stderr)
                return 1
            print(f"  open: {g.get_gripper_distance():.1f} mm")

            time.sleep(0.2)
            print(f"cycle {i + 1}/{args.cycles}: close")
            if not g.close(args.speed):
                print("error: close did not reach the calibrated closed limit",
                      file=sys.stderr)
                return 1
            print(f"  closed: {g.get_gripper_distance():.1f} mm")
        return 0
    except LiteGripError as error:
        print(f"error: {error}", file=sys.stderr)
        return 2
    finally:
        g.disconnect()


if __name__ == "__main__":
    sys.exit(main())
