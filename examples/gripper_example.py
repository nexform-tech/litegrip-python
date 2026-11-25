#!/usr/bin/env python3
"""End-to-end demo: connect, read status, open/close, position, grasp/release.

Run it from the repository root without installing the package:

    python3 examples/gripper_example.py --help
    python3 examples/gripper_example.py --channel can0
    python3 examples/gripper_example.py --channel can0 --yes --force 20

Every motion is blocking and returns ``True`` only when it actually reached its
goal; this script prints the outcome and exits non-zero if any step failed.

Clear the jaw travel before running: ``open``/``close`` drive the jaws to the
calibrated limits, and ``grasp`` closes onto whatever is between them.
"""

from __future__ import annotations

import argparse
import logging
import os
import sys

# Import the SDK straight out of the checkout, so the script works without
# `pip install -e .`.  Inserted first, so the checkout wins over an installed
# copy — running it exercises the code next to it.
_SRC = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "src")
if _SRC not in sys.path:
    sys.path.insert(0, _SRC)

from litegrip import Gripper, LiteGripError  # noqa: E402


def _hex(value: str) -> int:
    """Parse an int written as 0x08, 8, or 0o10 on the command line."""
    return int(value, 0)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Drive a LiteGrip through its main API and report every "
                    "result.")
    parser.add_argument("--channel", default="can0",
                        help="CAN interface (default: can0)")
    parser.add_argument("--can-id", type=_hex, default=0x08,
                        help="motor CAN ID (default: 0x08)")
    parser.add_argument("--mst-id", type=_hex, default=0x18,
                        help="status-frame master ID (default: 0x18)")
    parser.add_argument("--calib", default=None,
                        help="explicit calibration JSON; default is "
                             "LITEGRIP_CALIB, then "
                             "~/.litegrip/<channel>_calibration.json, then the "
                             "factory calibration")
    parser.add_argument("--speed", type=float, default=30.0,
                        help="move speed in mm/s (default: 30)")
    parser.add_argument("--distance", type=float, default=40.0,
                        help="absolute opening for set_gripper_distance, mm "
                             "(default: 40)")
    parser.add_argument("--force", type=float, default=10.0,
                        help="grasp force in N (default: 10)")
    parser.add_argument("--hold-s", type=float, default=2.0,
                        help="grasp hold duration in seconds (default: 2)")
    parser.add_argument("--yes", action="store_true",
                        help="skip the confirmation prompt")
    parser.add_argument("--verbose", action="store_true",
                        help="turn on the SDK's own logging")
    return parser


def _print_status(g: Gripper, label: str) -> None:
    state = g.get_state()
    print(f"\n[{label}] {g}")
    print(f"  position  {state.position_rad:+.4f} rad  ({state.position_mm:+.1f} mm)")
    print(f"  speed     {state.velocity_rad_s:+.3f} rad/s")
    print(f"  torque    {state.torque_nm:+.3f} Nm  (~{state.force_n:+.1f} N)")
    print(f"  driver    {state.temperature_mos} C   coil {state.temperature_coil} C")
    print(f"  status    {g.get_status_raw()}   fresh={state.fresh}  "
          f"age={state.data_age_s if state.data_age_s is not None else 'n/a'} s")


def _confirm(args) -> bool:
    if args.yes:
        return True
    if not sys.stdin.isatty():
        print("error: this moves the jaws and stdin is not a terminal; "
              "re-run with --yes if that is what you want", file=sys.stderr)
        return False
    print("\nAbout to drive the jaws open/close, to a set distance, then grasp "
          f"at {args.force:g} N.")
    print("Clear the jaw travel first.")
    try:
        input("Press Enter to start, Ctrl+C to abort: ")
    except (EOFError, KeyboardInterrupt):
        print()
        return False
    return True


def _run(args) -> int:
    g = Gripper(channel=args.channel, can_id=args.can_id, mst_id=args.mst_id)
    if args.calib is not None:
        g.load_calibration(args.calib)
    if not g.config.calibrated:
        print("error: no usable calibration — run examples/calibrate.py first",
              file=sys.stderr)
        return 2

    cfg = g.config
    travel_mm = abs(cfg.pos_open_rad - cfg.pos_closed_rad) * cfg.rad_to_mm
    print(f"calibration: closed={cfg.pos_closed_rad:+.6f} rad  "
          f"open={cfg.pos_open_rad:+.6f} rad")
    print(f"  travel={travel_mm:.1f} mm  rad_to_mm={cfg.rad_to_mm:.2f}  "
          f"kp={cfg.kp:g}  kd={cfg.kd:g}")
    print(f"  sdk version {g.get_version()} (not device firmware)")

    if not _confirm(args):
        print("aborted, nothing was sent on the bus")
        return 1

    ok = True
    try:
        g.connect()
        if not g.enable():
            print("error: enable failed — check the 24 V supply and bitrate",
                  file=sys.stderr)
            return 1
        _print_status(g, "connected and enabled")
        print(f"  motor data   {g.get_motor_data()}")
        print(f"  motor status {g.get_motor_status()}")

        step_ok = g.open(args.speed)
        ok &= step_ok
        print(f"open -> {'ok' if step_ok else 'FAILED'}; "
              f"distance {g.get_gripper_distance():.1f} mm")

        step_ok = g.close(args.speed)
        ok &= step_ok
        print(f"close -> {'ok' if step_ok else 'FAILED'}; "
              f"distance {g.get_gripper_distance():.1f} mm")

        step_ok = g.set_gripper_distance(args.distance, args.speed)
        ok &= step_ok
        print(f"set_gripper_distance({args.distance:g}) -> "
              f"{'ok' if step_ok else 'FAILED'}; "
              f"distance {g.get_gripper_distance():.1f} mm")

        step_ok = g.grasp(force_n=args.force, hold_s=args.hold_s)
        ok &= step_ok
        print(f"grasp({args.force:g} N, {args.hold_s:g} s) -> "
              f"{'ok' if step_ok else 'FAILED'}")
        _print_status(g, "after grasp")

        step_ok = g.release(clearance_mm=10.0)
        ok &= step_ok
        print(f"release -> {'ok' if step_ok else 'FAILED'}; "
              f"distance {g.get_gripper_distance():.1f} mm")
    finally:
        g.disconnect()

    return 0 if ok else 1


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.verbose:
        logging.basicConfig(level=logging.INFO,
                            format="%(levelname)s %(name)s: %(message)s")
    if args.speed <= 0.0:
        print(f"error: --speed must be positive, got {args.speed:g}",
              file=sys.stderr)
        return 2
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
