#!/usr/bin/env python3
"""Full-stroke smoke test: open to the limit, then close to the limit.

Read this when you want the one question "does this unit work?" answered on a
bench — after wiring a gripper up, after a firmware or calibration change, or
before trusting it in a program.  It drives **both ends of the travel** through
the high-level SDK (``Gripper.open`` / ``Gripper.close``), so the lead cap and
the travel-leg stall guard are in play, and it prints what each move measured.
Exit status is 0 only when every move succeeded.

It is not a calibrator (``examples/calibrate.py``) and it writes nothing to the
motor's flash.

    # What would this do, on this machine, with the calibration it finds?
    python3 examples/stroke_test.py --dry-run

    # Run one open+close cycle at the default speed.
    python3 examples/stroke_test.py --channel can0

    # Three cycles, slower, no prompt (for a script).
    python3 examples/stroke_test.py --cycles 3 --speed 10 --yes

Speed is the one number worth thinking about.  A full stroke is about 86 mm on
the shipped geometry, so the 20 mm/s default crosses it in roughly 4.3 s: slow
enough to watch and to reach Ctrl+C, quick enough not to be tedious.  Speed
does not change the force the jaws press with — that is ``kp`` times the
command lead, and the engine narrows the lead near the stop on its own — so a
faster run mainly means more kinetic energy going into the mechanical stop.
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

DEFAULT_SPEED_MM_S = 20.0
# Above this the run is fast enough that the stop takes a real hit; warn only,
# because a bench operator may know exactly what they are doing.
LOUD_SPEED_MM_S = 50.0


def _hex(value: str) -> int:
    return int(value, 0)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Drive a LiteGrip through its full travel and report what "
                    "each move measured.")
    parser.add_argument("--channel", default="can0",
                        help="CAN interface (default: can0)")
    parser.add_argument("--can-id", type=_hex, default=0x08,
                        help="motor CAN ID (default: 0x08)")
    parser.add_argument("--mst-id", type=_hex, default=0x18,
                        help="status-frame master ID (default: 0x18)")
    parser.add_argument("--speed", type=float, default=DEFAULT_SPEED_MM_S,
                        help=f"speed in mm/s (default: {DEFAULT_SPEED_MM_S:g})")
    parser.add_argument("--cycles", type=int, default=1,
                        help="open+close cycles to run (default: 1)")
    parser.add_argument("--calib", default=None,
                        help="explicit calibration JSON; default is "
                             "LITEGRIP_CALIB, then "
                             "~/.litegrip/<channel>_calibration.json, then the "
                             "factory calibration")
    parser.add_argument("--yes", action="store_true",
                        help="skip the confirmation prompt")
    parser.add_argument("--verbose", action="store_true",
                        help="turn on the SDK's own logging (calibration file, "
                             "connect/enable chatter)")
    parser.add_argument("--dry-run", action="store_true",
                        help="load the calibration and print the plan, then exit "
                             "without touching the bus")
    return parser


def _print_calibration(g: Gripper) -> float:
    """Print the geometry both moves will use, and return the open length."""
    cfg = g.config
    travel_mm = abs(cfg.pos_open_rad - cfg.pos_closed_rad) * cfg.rad_to_mm
    work_stroke_mm = cfg.work_stroke_mm
    print(f"  closed={cfg.pos_closed_rad:+.6f} rad  "
          f"open={cfg.pos_open_rad:+.6f} rad")
    print(f"  travel={travel_mm:.1f} mm  rad_to_mm={cfg.rad_to_mm:.2f}  "
          f"kp={cfg.kp:g}  kd={cfg.kd:g}")
    if 0.0 < work_stroke_mm < travel_mm:
        print(f"  open() stops at the {work_stroke_mm:g} mm work stroke, "
              f"{travel_mm - work_stroke_mm:.1f} mm short of the open stop")
        return work_stroke_mm
    print(f"  open() has no work stroke (or it exceeds the travel) — it ramps "
          f"the whole {travel_mm:.1f} mm into the open mechanical stop")
    return travel_mm


def _confirm(args) -> bool:
    """A full-stroke move can crush whatever is between the jaws, so ask."""
    if args.yes:
        return True
    if not sys.stdin.isatty():
        print("error: this moves the jaws through their whole travel and stdin is "
              "not a terminal, so there is nobody to ask; re-run with --yes if "
              "that is what you want", file=sys.stderr)
        return False
    print(f"\nAbout to run {args.cycles} open+close cycle(s) on {args.channel} at "
          f"{args.speed:g} mm/s.")
    print("Clear the jaws and the travel path first.")
    try:
        input("Press Enter to start, Ctrl+C to abort: ")
    except (EOFError, KeyboardInterrupt):
        print()
        return False
    return True


def _report(label: str, ok: bool, g: Gripper, before_mm: float) -> None:
    state = g.get_state()
    print(f"  {label}: {'ok' if ok else 'FAILED'}  "
          f"{before_mm:+.1f} -> {state.position_mm:+.1f} mm  "
          f"tau={state.torque_nm:+.3f} Nm (peak ~{state.force_n:+.1f} N)  "
          f"driver={state.temperature_mos} C coil={state.temperature_coil} C")


def _run(args) -> int:
    if args.speed <= 0.0:
        print(f"error: --speed must be positive, got {args.speed:g}",
              file=sys.stderr)
        return 2
    if args.cycles < 1:
        print(f"error: --cycles must be at least 1, got {args.cycles}",
              file=sys.stderr)
        return 2
    if args.calib is not None and not os.path.isfile(args.calib):
        print(f"error: no such calibration file: {args.calib}", file=sys.stderr)
        return 2
    if args.speed > LOUD_SPEED_MM_S:
        print(f"warning: {args.speed:g} mm/s is above the SDK's own default "
              f"({LOUD_SPEED_MM_S:g} mm/s) — the jaws hit the stop harder, and "
              f"the travel-leg stall guard needs time to notice an obstruction",
              file=sys.stderr)

    g = Gripper(channel=args.channel, can_id=args.can_id, mst_id=args.mst_id)
    if args.calib is not None:
        g.load_calibration(args.calib)
    if not g.config.calibrated:
        print("error: the calibration is not usable (calibrated=false) — run "
              "examples/calibrate.py first, or pass --calib", file=sys.stderr)
        return 2

    open_mm = _print_calibration(g)
    plan = (f"plan: {args.cycles} x (open {open_mm:.0f} mm -> close 0 mm) at "
            f"{args.speed:g} mm/s")
    print(f"{plan}; guard tripping at |tau| >= "
          f"{g.motion_config.stop_torque_nm:g} Nm for "
          f"{g.motion_config.stop_torque_cycles} slow cycles")

    if args.dry_run:
        print("dry run: nothing was sent on the bus")
        return 0
    if not _confirm(args):
        print("aborted, nothing was sent on the bus")
        return 1

    failures = 0
    try:
        g.connect()
        if not g.enable():
            print("error: enable failed — check the 24 V supply and the bitrate",
                  file=sys.stderr)
            return 1
        print(f"connected {g}")

        for cycle in range(1, args.cycles + 1):
            print(f"\ncycle {cycle}/{args.cycles}:")
            before = g.get_gripper_distance()
            ok = g.open(args.speed)
            _report("open ", ok, g, before)
            failures += 0 if ok else 1

            before = g.get_gripper_distance()
            ok = g.close(args.speed)
            _report("close", ok, g, before)
            failures += 0 if ok else 1
    finally:
        g.disconnect()

    print(f"\n{args.cycles * 2 - failures} of {args.cycles * 2} moves succeeded")
    return 1 if failures else 0


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.verbose:
        logging.basicConfig(level=logging.INFO,
                            format="%(levelname)s %(name)s: %(message)s")
    return _run(args)


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        # _run's finally has already disconnected, which disables the motor, so
        # the jaws are not left pressing.
        print("\ninterrupted — motor disabled, jaws left where they stopped",
              file=sys.stderr)
        sys.exit(130)
    except LiteGripError as error:
        print(f"error: {error}", file=sys.stderr)
        sys.exit(1)
