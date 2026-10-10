#!/usr/bin/env python3
"""Full-stroke smoke test: open to the limit, then close to the limit.

Read this when you want the one question "does this unit work?" answered on a
bench — after wiring a gripper up, after a firmware or calibration change, or
before trusting it in a program.  It drives **both ends of the travel** through
the high-level SDK (``LiteGrip.open`` / ``LiteGrip.close``), so the lead cap and
the travel-leg stall guard are in play, and it prints what each move measured:
where it ended up, how hard it pushed, and whether it got there.  Exit status is
0 only when every move succeeded.

It is not a calibrator (``examples/zero_closed.py zero``) and not a raw bench
tool (``goto``): nothing is written to the motor's flash, no parameter is set.

    # What would this do, on this machine, with the calibration it finds?
    python3 examples/stroke_test.py --dry-run

    # Run it, one open+close cycle, at the default speed.
    python3 examples/stroke_test.py --channel can0

    # Three cycles, slower, no prompt (for a script).
    python3 examples/stroke_test.py --cycles 3 --speed 10 --yes

Speed is the one number worth thinking about.  A full stroke is about 86 mm on
the shipped geometry, so the 20 mm/s default crosses it in roughly 4.3 s: slow
enough to watch and to reach Ctrl+C, quick enough not to be tedious.  Speed does
not change the force the jaws end up pressing with — that is ``kp`` times the
command lead, and the engine narrows the lead near the stop on its own — so a
faster run mainly means more kinetic energy going into the mechanical stop.  The
script warns above 50 mm/s (the SDK's own default) rather than refusing.
``--kp`` is the other knob, and it is the one that changes how hard the jaws
press; the default is whatever the calibration file says.

The work-stroke limit and the travel-leg stall guard are recent additions, so
this script reads them through ``getattr`` with the old behaviour as the default:
against a checkout that predates them it still runs, it just reports that there
is no work stroke and no guard to trip.
"""

from __future__ import annotations

import argparse
import logging
import os
import sys

# Like the other examples: import the SDK straight out of the checkout, so the
# script works without `pip install -e .`.  Inserted first, so the checkout wins
# over an installed copy — running it exercises the code next to it.
_SRC = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "src")
if _SRC not in sys.path:
    sys.path.insert(0, _SRC)

from litegrip import LiteGrip, LiteGripError, default_calib_path  # noqa: E402
from litegrip.can import MotorType  # noqa: E402

# A stroke that takes seconds, not milliseconds.  See the module docstring.
DEFAULT_SPEED_MM_S = 20.0
# Above this the run is fast enough that the stop takes a real hit; warn only,
# because a bench operator may know exactly what they are doing.
LOUD_SPEED_MM_S = 50.0


def _hex(value: str) -> int:
    """Parse an int written as 0x08, 8, or 0o10 on the command line."""
    return int(value, 0)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Drive a LiteGrip through its full travel and report what "
                    "each move measured.")
    parser.add_argument("--channel", default="can0",
                        help="CAN interface (default: can0)")
    parser.add_argument("--can-id", type=_hex, default=0x08,
                        help="motor CAN ID (default: 0x08)")
    parser.add_argument("--mst-id", type=_hex, default=None,
                        help="master/status ID; default auto-detect")
    parser.add_argument("--motor", default="DM4310",
                        choices=[m.name for m in MotorType],
                        help="motor model (default: DM4310)")
    parser.add_argument("--canfd", action="store_true",
                        help="open the transport in CAN-FD mode")
    parser.add_argument("--speed", type=float, default=DEFAULT_SPEED_MM_S,
                        help=f"speed in mm/s (default: {DEFAULT_SPEED_MM_S:g})")
    parser.add_argument("--kp", type=float, default=None,
                        help="override the calibration's press stiffness (the "
                             "DM stiffness code, 0-500); this is what sets how "
                             "hard the jaws press into the stop. Default: the "
                             "value in the calibration file")
    parser.add_argument("--cycles", type=int, default=1,
                        help="open+close cycles to run (default: 1)")
    parser.add_argument("--calib", default=None,
                        help="explicit calibration JSON; default is this "
                             "channel's own file, then the legacy/factory one")
    parser.add_argument("--template", default=None,
                        help="calibration template name; see litegrip.list_templates()")
    parser.add_argument("--yes", action="store_true",
                        help="skip the confirmation prompt")
    parser.add_argument("--verbose", action="store_true",
                        help="turn on the SDK's own logging (which calibration "
                             "file won, connect/enable chatter)")
    parser.add_argument("--dry-run", action="store_true",
                        help="load the calibration and print the plan, then exit "
                             "without touching the bus")
    return parser


def to_mm(cfg, rad: float) -> float:
    """Radians -> millimetres of opening, on this gripper's own calibration.

    The same expression the SDK uses for ``GripperState.position_mm``
    (``gripper.py``), repeated here because the moves report rad and the reader
    thinks in mm.
    """
    return (cfg.pos_closed_rad - rad) * cfg.close_sign * cfg.rad_to_mm


class MoveTrace:
    """What one move measured, accumulated from the engine's own callbacks.

    ``MoveResult`` carries the outcome; this carries the shape of the move — the
    worst torque, how fast the jaws actually travelled, and how far they got,
    none of which the result reports.
    """

    def __init__(self, sample_interval_s: float) -> None:
        self.sample_interval_s = sample_interval_s
        self.samples = 0
        self.peak_torque_nm = 0.0
        self.peak_step_rad = 0.0
        self.min_rad = float("inf")
        self.max_rad = float("-inf")

    def __call__(self, progress) -> None:
        self.samples += 1
        self.peak_torque_nm = max(self.peak_torque_nm, abs(progress.torque_nm))
        self.peak_step_rad = max(self.peak_step_rad, abs(progress.delta_rad))
        self.min_rad = min(self.min_rad, progress.pos_rad)
        self.max_rad = max(self.max_rad, progress.pos_rad)

    def peak_mm_s(self, cfg) -> float:
        """Measured travel speed, mm/s — compare it against what you asked for."""
        return self.peak_step_rad * cfg.rad_to_mm / self.sample_interval_s


def _print_calibration(g: LiteGrip, source: str) -> float:
    """Print the geometry both moves will use, and return the open move's length."""
    cfg = g.config
    travel_mm = abs(cfg.pos_open_rad - cfg.pos_closed_rad) * cfg.rad_to_mm
    work_stroke_mm = float(getattr(cfg, "work_stroke_mm", 0.0))
    print(f"calibration from {source}")
    print(f"  closed={cfg.pos_closed_rad:+.6f} rad  open={cfg.pos_open_rad:+.6f} rad  "
          f"mount={cfg.mount}")
    print(f"  travel={travel_mm:.1f} mm  rad_to_mm={cfg.rad_to_mm}  "
          f"kp={cfg.kp:g}  kd={cfg.kd:g}")
    print(f"  work_stroke_mm={work_stroke_mm:g}")
    if 0.0 < work_stroke_mm < travel_mm:
        print(f"  open() stops at the {work_stroke_mm:g} mm work stroke, "
              f"{travel_mm - work_stroke_mm:.1f} mm short of the open stop")
        open_mm = work_stroke_mm
    else:
        print(f"  open() has no work stroke (or it exceeds the travel) — it ramps "
              f"the whole {travel_mm:.1f} mm into the open mechanical stop")
        open_mm = travel_mm
    print(f"  close() presses onto the closed stop, {travel_mm:.1f} mm back")
    return open_mm


def _source_note(args) -> str:
    """Where the calibration will come from, without guessing."""
    if args.template is not None:
        return f"template {args.template!r}"
    if args.calib is not None:
        return args.calib
    own = default_calib_path(args.channel)
    if os.path.isfile(own):
        return own
    return (f"{own} (missing) -> this channel has no calibration of its own, so "
            f"the legacy/factory file is used; check it says the right mount "
            f"(run with --verbose to see which file actually won)")


def _describe(move, cfg, trace: MoveTrace, motion) -> str:
    st = move.state
    # How far the jaws stopped from the limit they were aiming at, next to the
    # tolerance ``ok`` is judged against.  When the two are the same size, the
    # verdict flips between runs on the same physical end stop, so print both.
    miss_mm = abs(st.position_rad - move.limit_rad) * cfg.rad_to_mm
    tol_mm = motion.stop_tol * cfg.rad_to_mm
    return (f"  ok={move.ok} reached={move.reached} stalled={move.stalled} "
            f"protected={bool(getattr(move, 'protected', False))} err={st.error_code}\n"
            f"  ended at {st.position_mm:+.1f} mm ({st.position_rad:+.6f} rad), "
            f"tau={st.torque_nm:+.3f} Nm\n"
            f"  missed the limit at {to_mm(cfg, move.limit_rad):+.1f} mm by "
            f"{miss_mm:.2f} mm (engine tolerance {tol_mm:.2f} mm)\n"
            f"  target_rad={move.target_rad:+.6f} final_cmd_rad={move.final_cmd_rad:+.6f} "
            f"(cmd leads the jaws by "
            f"{abs(move.final_cmd_rad - st.position_rad) * cfg.rad_to_mm:.2f} mm)\n"
            f"  {move.steps} frames ({move.steps * motion.frame_interval:.2f} s), "
            f"{trace.samples} samples; reached {to_mm(cfg, trace.min_rad):+.1f} .. "
            f"{to_mm(cfg, trace.max_rad):+.1f} mm; "
            f"peak |tau|={trace.peak_torque_nm:.3f} Nm, "
            f"peak travel {trace.peak_mm_s(cfg):.1f} mm/s")


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


def _run(args) -> int:
    if args.speed <= 0.0:
        print(f"error: --speed must be positive, got {args.speed:g}", file=sys.stderr)
        return 2
    if args.cycles < 1:
        print(f"error: --cycles must be at least 1, got {args.cycles}", file=sys.stderr)
        return 2
    if args.calib is not None and not os.path.isfile(args.calib):
        print(f"error: no such calibration file: {args.calib}", file=sys.stderr)
        return 2
    if args.speed > LOUD_SPEED_MM_S:
        print(f"warning: {args.speed:g} mm/s is above the SDK's own default "
              f"({LOUD_SPEED_MM_S:g} mm/s) — the jaws hit the stop harder, and the "
              f"travel-leg stall guard needs time to notice an obstruction",
              file=sys.stderr)

    g = LiteGrip(channel=args.channel, can_id=args.can_id, mst_id=args.mst_id,
                 canfd_mode=args.canfd or None, motor_type=MotorType[args.motor])
    try:
        # Calibration is a file read, so it happens before connect: the file may
        # carry can_id/mst_id, and connect() is what puts those on the wire.
        if not g.load_calibration(path=args.calib, template=args.template):
            print("error: no calibration file could be read — run "
                  "`python3 examples/zero_closed.py zero` first, or pass "
                  "--calib/--template", file=sys.stderr)
            return 2
        if not g.config.calibrated:
            print("error: the calibration file says calibrated=false; the motion "
                  "engine refuses to move on placeholder limits", file=sys.stderr)
            return 2

        if args.kp is not None:
            print(f"kp override: {g.config.kp:g} -> {args.kp:g} (press stiffness)")
            g.config.kp = args.kp

        open_mm = _print_calibration(g, _source_note(args))
        motion = g.actions.config
        plan = (f"plan: {args.cycles} x (open {open_mm:.0f} mm -> close 0 mm) at "
                f"{args.speed:g} mm/s")
        guard_nm = getattr(motion, "stop_torque_nm", None)
        if guard_nm is None:
            print(f"{plan}; no travel-leg guard on this checkout")
        else:
            print(f"{plan}; guard tripping at |tau| >= {guard_nm:g} Nm for "
                  f"{motion.stop_torque_cycles} slow cycles")

        if args.dry_run:
            print("dry run: nothing was sent on the bus")
            return 0

        if not _confirm(args):
            print("aborted, nothing was sent on the bus")
            return 1

        if not g.connect():
            print(f"error: could not connect to {args.channel}", file=sys.stderr)
            return 1
        enable = g.enable()
        err = None if enable.state is None else enable.state.error_code
        print(f"connected {g}  enabled={enable.ok} tries={enable.tries} err={err}")
        if not enable.ok:
            print(f"error: enable failed after {enable.tries} tries (err={err}) — "
                  f"check the 24 V supply and the bitrate", file=sys.stderr)
            return 1

        failures = 0
        for cycle in range(1, args.cycles + 1):
            for label, action in (("open", g.open), ("close", g.close)):
                trace = MoveTrace(motion.sample_interval)
                move = action(args.speed, progress=trace)
                print(f"\ncycle {cycle}/{args.cycles}  {label}:")
                print(_describe(move, g.config, trace, motion))
                if getattr(move, "protected", False):
                    print("  note: the travel-leg stall guard tripped and released "
                          "the jaw (kp=0) — something blocked the travel",
                          file=sys.stderr)
                if not move.ok:
                    failures += 1
                    print(f"  FAIL: {label} did not complete", file=sys.stderr)

        print(f"\n{args.cycles * 2 - failures} of {args.cycles * 2} moves succeeded")
        return 1 if failures else 0
    finally:
        g.disconnect()


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
        # the jaws are not left pressing.  Report that instead of a traceback.
        print("\ninterrupted — motor disabled, jaws left where they stopped",
              file=sys.stderr)
        sys.exit(130)
    except LiteGripError as error:
        print(f"error: {error}", file=sys.stderr)
        sys.exit(1)
