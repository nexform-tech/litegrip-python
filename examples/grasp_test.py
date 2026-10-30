#!/usr/bin/env python3
"""Grasp smoke test: close onto nothing, then close onto something.

Read this when you want to know whether ``grasp()`` still does what it says on a
real gripper — after a change to the motion engine, a recalibration, or wiring a
new unit up.  Every other action ends on a mechanical stop, so "did it move" is
enough to judge it.  ``grasp()`` ends on an *object*, which makes it the one
action that can be right in one case and wrong in the other: it has to reach the
closed stop when the jaws are clear, and it has to stop early and hold when they
are not.  The script runs both cases and judges them separately.

1. **Clear jaws** — expect ``reached=True stalled=False``: it closed all the way
   to the closed-side calibrated stop with nothing in the way.
2. **An object between the jaws** — expect ``stalled=True reached=False``: it met
   the object well before the stop.  You insert the object when prompted.

Both are also the two ways the closing leg is allowed to end.  ``grasp()`` hands
the jaws over to the force hold only on ``reached or stalled``, because the hold
frames carry no position term and no velocity term — a jaw handed over while it
is still travelling has nothing left to bound it.  A third ending,
``ok=False reached=False stalled=False``, is reported as its own failure below;
it means the closing leg was starved and nothing was gripped.

**Each case opens the jaws first.**  An empty grasp is only testing something if
the jaws start away from the empty target — starting them a millimetre from it
measures nothing — and an object cannot go into jaws that are already shut.  The
open is a precondition, not the subject: if it fails, the case is reported as
untestable rather than run, because closing on an obstruction a failed open
could not clear would look like a successful grasp of it. Diagnose an open that
stalls with ``python3 examples/stroke_test.py``, and try ``--open-speed 10`` —
the stall detector's window threshold scales with the speed it was given.

``ok=True`` is expected in both cases and means only that the hold phase finished
instead of being interrupted by a fault or by the runaway watch — it is not the
interesting signal here.  (The watch drops the feed-forward torque and stops the
hold when the jaws walk more than ``MotionConfig.hold_runaway_rad`` past the
closed stop; a tripped watch is reported as its own problem rather than folded
into that sentence.  Neither case here should trip it — the jaws meet something
or a mechanical stop long before the threshold.)

The script also reports what the hold applied.  ``grasp()`` ramps a feed-forward
torque up to ``force_n x 0.1`` Nm (``MotionConfig.force_ramp_n_s``) and holds it
there for ``hold_s``, so the measured torque during the hold is printed next to
that number.  A hold that reads near zero against the force you asked for means
the feed-forward never reached the motor.

**``is_grasped()`` is a torque threshold, not an object detector, and the script
does not judge on it.**  It compares the measured torque against
``GripperConfig.grasp_torque_threshold`` (0.5 Nm), and the hold streams its
feed-forward torque whether or not anything is between the jaws — so on a clear
jaw it reports whatever that feed-forward happens to produce, which a high
enough ``--force`` can push over the threshold.  Treat it as "the motor is
pushing at this level", not "something is in the jaws"; the empty/loaded verdict
comes from ``reached`` and ``stalled`` instead, and the measured hold torque is
printed next to the commanded one so you can see which side of the threshold
this unit lands on.

    # What would this do, on this machine, with the calibration it finds?
    python3 examples/grasp_test.py --dry-run

    # Both cases, 20 N for 2 s each.
    python3 examples/grasp_test.py --channel can0

    # A gentler grip, nothing to insert (only the empty case).
    python3 examples/grasp_test.py --force 8 --hold-s 3 --skip-loaded --yes

**The jaws are disabled when the script exits, so whatever is being held falls.**
Catch it, or put something soft underneath.  If you want the grip to persist,
call ``grasp(..., hold_s=0)`` from a program instead — ``hold_s=0`` means "hold
until a fault, the runaway watch, or Ctrl+C", which is why this script never
uses it: a test that holds forever is a test that never finishes.
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
from litegrip.actions import limit_target  # noqa: E402
from litegrip.can import MotorType  # noqa: E402

#: The SDK's approximate force conversion, ``MotionConfig`` -> Nm.  Repeated
#: here so the script can print the torque you are about to apply; the engine
#: uses ``UnitConversion.N_TO_NM``.
N_TO_NM = 0.1
#: A grasp that holds longer than this is a test that never finishes.
DEFAULT_HOLD_S = 2.0
#: Above this the grip is strong enough to damage a part; warn, do not refuse.
LOUD_FORCE_N = 20.0
#: How much of the commanded feed-forward the hold has to reach to count as
#: "the force arrived".  Loose on purpose — the motor's torque estimate is
#: approximate and the object may be yielding.
FORCE_ARRIVED_RATIO = 0.5


def _hex(value: str) -> int:
    """Parse an int written as 0x08, 8, or 0o10 on the command line."""
    return int(value, 0)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Close onto nothing and onto an object, and report what "
                    "grasp() did in each case.")
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
    parser.add_argument("--force", type=float, default=None,
                        help="gripping force in N (default: MotionConfig.force_n); "
                             "the hold pushes force x 0.1 Nm")
    parser.add_argument("--hold-s", type=float, default=DEFAULT_HOLD_S,
                        help=f"how long to hold each grasp, s (default: "
                             f"{DEFAULT_HOLD_S:g}); 0 is refused, because the SDK "
                             f"reads it as 'hold forever'")
    parser.add_argument("--speed", type=float, default=None,
                        help="closing speed in mm/s (default: "
                             "MotionConfig.grasp_speed_mm_s)")
    parser.add_argument("--open-speed", type=float, default=None,
                        help="speed in mm/s for the precondition open, which "
                             "runs before each case (default: "
                             "MotionConfig.speed_mm_s).  A slower open is worth "
                             "trying if it stalls: the stall detector's window "
                             "threshold scales with the speed it was given")
    parser.add_argument("--kp", type=float, default=None,
                        help="override the calibration's position stiffness (the "
                             "DM stiffness code, 0-500).  This is what the "
                             "closing move is judged on and what sets how hard "
                             "the jaws push; it does not affect the hold, which "
                             "is a bare torque command.  Default: the value in "
                             "the calibration file")
    parser.add_argument("--calib", default=None,
                        help="explicit calibration JSON.  Without it the "
                             "factory calibration is used: a file this machine "
                             "measured is never read unless you name it here "
                             "or in LITEGRIP_CALIB")
    parser.add_argument("--template", default=None,
                        help="calibration template name; see litegrip.list_templates()")
    parser.add_argument("--skip-empty", action="store_true",
                        help="run only the object case")
    parser.add_argument("--skip-loaded", action="store_true",
                        help="run only the clear-jaws case")
    parser.add_argument("--yes", action="store_true",
                        help="skip the confirmation prompt")
    parser.add_argument("--verbose", action="store_true",
                        help="turn on the SDK's own logging")
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


class Trace:
    """Accumulates both halves of one ``grasp()`` from the engine's callbacks.

    ``GraspResult`` reports the outcome of the closing move, whether the hold
    finished, and whether the runaway watch tripped; it reports nothing about
    what the hold applied.  This keeps the per-sample move data and the
    per-slice hold data so the script can say what the hold actually applied.
    """

    def __init__(self, sample_interval_s: float):
        self.sample_interval_s = sample_interval_s
        self.start_rad: float | None = None
        self.move_samples = 0
        self.peak_move_torque_nm = 0.0
        self.peak_step_rad = 0.0
        self.min_rad = float("inf")
        self.max_rad = float("-inf")
        self.hold_slices = 0
        self.hold_torque_nm: list[float] = []

    def __call__(self, progress) -> None:
        if progress.phase == "hold":
            self.hold_slices += 1
            self.hold_torque_nm.append(progress.torque_nm)
            return
        self.move_samples += 1
        if self.start_rad is None:
            self.start_rad = progress.pos_rad
        self.peak_move_torque_nm = max(self.peak_move_torque_nm,
                                       abs(progress.torque_nm))
        self.peak_step_rad = max(self.peak_step_rad, abs(progress.delta_rad))
        self.min_rad = min(self.min_rad, progress.pos_rad)
        self.max_rad = max(self.max_rad, progress.pos_rad)

    @property
    def peak_hold_torque_nm(self) -> float:
        return max((abs(t) for t in self.hold_torque_nm), default=0.0)

    def peak_mm_s(self, cfg) -> float:
        return self.peak_step_rad * cfg.rad_to_mm / self.sample_interval_s


def _source_note(args) -> str:
    """Where the calibration will come from, without guessing."""
    if args.template is not None:
        return f"template {args.template!r}"
    if args.calib is not None:
        return args.calib
    own = default_calib_path(args.channel)
    if os.environ.get("LITEGRIP_CALIB"):
        return f"{own} (LITEGRIP_CALIB)"
    if os.path.isfile(own):
        return (f"the factory calibration — this machine has its own at {own}, "
                f"which is used only when named: --calib {own}")
    return "the factory calibration"


def _pause(prompt: str, waiting_for: str) -> bool:
    """Wait for the operator to do something physical.  False if there is nobody.

    ``waiting_for`` describes the wait in the refusal message, because the
    prompt itself is a whole sentence and reads badly inside one.
    """
    if not sys.stdin.isatty():
        print(f"error: cannot wait for {waiting_for} — stdin is not a terminal, so "
              f"there is nobody to ask; use --yes and --skip-* to run without "
              f"prompts", file=sys.stderr)
        return False
    try:
        input(prompt)
    except (EOFError, KeyboardInterrupt):
        print()
        return False
    return True


def _confirm(args, force_n: float) -> bool:
    """Closing on an unknown object at an unknown force is worth one question."""
    if args.yes:
        return True
    cases = []
    if not args.skip_empty:
        cases.append("clear jaws")
    if not args.skip_loaded:
        cases.append("an object you insert")
    print(f"\nAbout to grasp {len(cases)} time(s) on {args.channel}: "
          f"{' then '.join(cases)}.")
    print(f"Each hold pushes {force_n:g} N = {force_n * N_TO_NM:g} Nm, for "
          f"{args.hold_s:g}s.")
    return _pause("Press Enter to start, Ctrl+C to abort: ",
                  "confirmation before moving the jaws")


def _report(result, trace: Trace, cfg, motion, label: str, grasped: bool) -> str:
    st = result.state
    lines = [
        f"  ok={result.ok} reached={result.reached} stalled={result.stalled} "
        f"no_bite={result.no_bite} runaway_watch={result.travel_guard} "
        f"err={st.error_code}",
        f"  stopped at {st.position_mm:+.1f} mm ({st.position_rad:+.6f} rad), "
        f"empty target was {to_mm(cfg, result.target_rad):+.1f} mm, "
        f"off by {abs(st.position_rad - result.target_rad) * cfg.rad_to_mm:.2f} mm",
        f"  hold: {result.cycles} slices x {motion.hold_interval:g}s = "
        f"{result.cycles * motion.hold_interval:.1f}s, force {result.force_n:g} N "
        f"({result.force_n * N_TO_NM:g} Nm); measured |tau| "
        f"mean {_mean([abs(t) for t in trace.hold_torque_nm]):.3f} Nm / "
        f"peak {trace.peak_hold_torque_nm:.3f} Nm",
        f"  is_grasped() right after the hold: {grasped} "
        f"(threshold {cfg.grasp_torque_threshold:g} Nm)",
        f"  close: {trace.move_samples} samples; reached "
        f"{to_mm(cfg, trace.min_rad):+.1f} .. {to_mm(cfg, trace.max_rad):+.1f} mm; "
        f"peak |tau|={trace.peak_move_torque_nm:.3f} Nm, "
        f"peak travel {trace.peak_mm_s(cfg):.1f} mm/s",
    ]
    return f"\n{label}\n" + "\n".join(lines)


def _mean(values) -> float:
    return sum(values) / len(values) if values else 0.0


def _stall_ramp_mm(result, trace: Trace, cfg, motion) -> tuple[float, float] | None:
    """``(ramp_mm, window_mm)`` for a close whose stall detector could not arm.

    ``None`` when the close has enough travel to arm it.  The engine's position
    window starts only once it holds ``stall_cycles`` samples, so a close that
    covers less than that window cannot report ``stalled=True`` whatever the
    jaws meet — which makes ``stalled=False`` no evidence at all.  A
    full-stroke close is ~80 mm and always clears it, so this only bites when
    the jaws start near the stop — which is exactly why every case opens first.
    """
    if trace.start_rad is None or motion.grasp_speed_mm_s <= 0.0:
        return None
    ramp_mm = abs(trace.start_rad - result.target_rad) * cfg.rad_to_mm
    frames = ((motion.stall_cycles + 1) * motion.sample_interval
              / motion.frame_interval)
    window_mm = frames * motion.frame_interval * motion.grasp_speed_mm_s
    return None if ramp_mm >= window_mm else (ramp_mm, window_mm)


def _hold_problem(result) -> str | None:
    """Why the hold did not finish, if it did not.

    The runaway watch drops the torque and returns on its own, so a tripped
    guard reaches here with ``ok=False`` and ``err=1`` — calling that "ended
    abnormally (err=1)" would name the fault code for something that was not a
    fault, and hide the one thing worth knowing.
    """
    if result.travel_guard:
        return ("the runaway watch tripped during the hold: the jaws walked past "
                "the closed stop by more than MotionConfig.hold_runaway_rad, so "
                "the torque was dropped — either nothing is stopping the closing "
                "direction, or the calibration no longer describes the travel")
    if result.no_bite:
        # Not a hold problem at all: grasp() refused the hand-over, so no hold
        # ran and there was no fault. The judges name this
        # (_no_bite_problem); here it must not be mistaken for "err=0".
        return None
    if not result.ok:
        return f"the hold phase ended abnormally (err={result.state.error_code})"
    return None


def _no_bite_problem(result, motion) -> str:
    """The one ending that is a refusal rather than a motion, said plainly.

    ``grasp()`` dropped the torque because the approach never moved the jaws
    and its budget was below the breakaway (``MotionConfig.breakaway_nm``).
    Reporting that as "the closing leg ran out of frames while still
    travelling" — the message the other endings get — points the operator the
    opposite way, and "err=0" reads like a fault that did not happen.
    """
    floor_n = motion.breakaway_nm / (N_TO_NM * motion.press_safety)
    return (f"refused, not held (no_bite=True): the closing leg never moved the "
            f"jaws. Its budget ({result.force_n:g} N = "
            f"{result.force_n * N_TO_NM:g} Nm, of which the approach may spend "
            f"{motion.press_safety:g}) is below the breakaway "
            f"({motion.breakaway_nm:g} Nm), so grasp() dropped the torque "
            f"instead of handing the jaws to the torque-only hold — that would "
            f"have powered them across the free stroke. The jaws stopped at "
            f"{result.state.position_mm:+.1f} mm. Raise --force above "
            f"{floor_n + 0.1:.1f} N (or check whether something is already "
            f"against the jaws before closing)")


def _judge_empty(result, trace: Trace, cfg, motion) -> list[str]:
    """Clear jaws: it should reach the closed stop and not report a stall."""
    problems = []
    if result.no_bite:
        # A refusal, not a movement. Everything below judges the closing leg
        # and the hold that followed — neither ran.
        return [_no_bite_problem(result, motion)]
    if not result.reached:
        short_mm = (abs(result.state.position_rad - result.target_rad)
                    * cfg.rad_to_mm)
        if not result.stalled:
            # The closing leg ends on arrival or on a stall, so neither of them
            # is what happened here: it ran out of frames while still in free
            # travel, and grasp() refused to hand a moving jaw to the hold.
            problems.append(
                f"did not reach the closed stop and did not stall (stopped "
                f"{short_mm:.2f} mm short) — nothing was in the way, so the "
                f"closing leg ran out of frames while still travelling.  That is "
                f"the one ending grasp() refuses to hold from, so no grip was "
                f"applied at all.  Suspect a stale state read or friction the "
                f"force budget cannot overcome (try --force 20)")
        else:
            problems.append(
                f"did not reach the closed stop (stopped {short_mm:.2f} mm short) "
                f"but reported stalled=True — either something is between the "
                f"jaws after all, or the calibration no longer describes the "
                f"travel")
    elif result.stalled:
        problems.append("reported stalled=True with nothing between the jaws — it "
                        "met an obstruction, or the closing move stalled on "
                        "friction")
    problem = _hold_problem(result)
    if problem is not None:
        problems.append(problem)
    if result.cycles == 0:
        problems.append("the hold never ran a slice")
    return problems


def _judge_loaded(result, trace: Trace, cfg, motion) -> list[str]:
    """An object between the jaws: it should stall early and then hold."""
    problems = []
    if result.no_bite:
        return [_no_bite_problem(result, motion)]
    short = _stall_ramp_mm(result, trace, cfg, motion)
    if short is not None:
        # stalled could not have become True here, so saying it did not fire is
        # not a finding — the case simply did not run.
        ramp_mm, window_mm = short
        problems.append(
            f"inconclusive: the close only had {ramp_mm:.1f} mm of ramp and the "
            f"stall detector needs {window_mm:.1f} mm before it can arm, so "
            f"nothing about the object was tested.  The jaws started too close to "
            f"the target — check the precondition open")
    else:
        if not result.stalled:
            problems.append("reported stalled=False — it never detected the "
                            "object, so it closed to the empty target instead")
    if result.reached:
        problems.append("reached the empty target — nothing was actually between "
                        "the jaws")
    problem = _hold_problem(result)
    if problem is not None:
        problems.append(problem)
    if result.cycles == 0:
        problems.append("the hold never ran a slice")
    return problems


def _force_note(trace: Trace, force_n: float) -> str | None:
    """What the hold's torque is worth, as a note rather than a verdict.

    The check is deliberately not a failure, because the SDK never promises the
    motor's reported torque equals the feed-forward it was sent: ``N_TO_NM`` is
    ``0.1`` and both the constant and the README call that mapping approximate.
    A large gap is still worth saying out loud, since it is the only reading
    that distinguishes "pressing at the force asked for" from "barely pressing
    at all".
    """
    asked = force_n * N_TO_NM
    peak = trace.peak_hold_torque_nm
    if peak >= FORCE_ARRIVED_RATIO * asked:
        return None
    return (f"note: the hold's measured torque peaked at {peak:.3f} Nm against the "
            f"{asked:g} Nm feed-forward asked for.  That is not a failure — the N "
            f"to Nm mapping is approximate by design — but if the jaws were "
            f"squeezing something, check that the motor is not saturated or "
            f"current-limited")


def _open_precondition(g: LiteGrip, open_speed: float) -> bool:
    """Open the jaws so the case starts from a known position.

    Both cases need this, and for different reasons.  An empty grasp is only
    testing something if the jaws start *away* from the empty target — starting
    them a millimetre from it measures nothing.  And an object cannot be put
    between jaws that are already shut.

    So each case opens first, and a failed open is reported as this case's
    failure rather than let through: closing on an obstruction a failed open
    could not clear would look like a successful grasp of it.
    """
    motion = g.actions.config
    trace = Trace(motion.sample_interval)
    move = g.open(open_speed, progress=trace)
    st = move.state
    print(f"\nprecondition, open at {open_speed:g} mm/s: ok={move.ok} "
          f"reached={move.reached} stalled={move.stalled} err={st.error_code}")
    print(f"  ended at {st.position_mm:+.1f} mm ({st.position_rad:+.6f} rad), "
          f"travelled {to_mm(g.config, trace.min_rad):+.1f} .. "
          f"{to_mm(g.config, trace.max_rad):+.1f} mm; peak |tau|="
          f"{trace.peak_move_torque_nm:.3f} Nm, peak travel "
          f"{trace.peak_mm_s(g.config):.1f} mm/s")
    if not move.ok:
        print("  the jaws did not open, so this case cannot be run — closing now "
              "would measure the obstruction, not grasp().  Check the jaws are "
              "clear, then try `--open-speed 10` (a slower open asks the stall "
              "detector for far less movement per window) or `--kp 100` (more "
              "breakaway torque); `python3 examples/stroke_test.py --speed 10` "
              "exercises the open on its own", file=sys.stderr)
    return move.ok


def _do_grasp(g: LiteGrip, trace: Trace, args, force_n: float, label: str,
              judge) -> bool:
    """Run one grasp, print it, and say whether this case behaved as expected.

    One case, one verdict: a case that trips three checks is still one failed
    case, so the summary line counts cases and not complaints.
    """
    result = g.grasp(force_n, args.hold_s, progress=trace)
    # The SDK's own detector, read straight after the hold so the last status
    # frame is still the one carrying the grip.
    grasped = g.is_grasped()
    print(_report(result, trace, g.config, g.actions.config, label, grasped))
    problems = judge(result, trace, g.config, g.actions.config)
    for problem in problems:
        print(f"  FAIL: {problem}", file=sys.stderr)
    short = _stall_ramp_mm(result, trace, g.config, g.actions.config)
    if short is not None:
        print(f"  note: the close only had {short[0]:.1f} mm of ramp against the "
              f"{short[1]:.1f} mm the stall detector needs to arm — "
              f"stalled={result.stalled} says nothing about what the jaws met")
    note = _force_note(trace, force_n)
    if note is not None:
        print(f"  {note}")
    return not problems


def _run(args) -> int:
    if args.hold_s <= 0.0:
        print(f"error: --hold-s must be positive, got {args.hold_s:g} (the SDK "
              f"reads 0 as 'hold forever', which a test script must not do)",
              file=sys.stderr)
        return 2
    if args.skip_empty and args.skip_loaded:
        print("error: --skip-empty and --skip-loaded together leave nothing to "
              "run", file=sys.stderr)
        return 2
    if args.calib is not None and not os.path.isfile(args.calib):
        print(f"error: no such calibration file: {args.calib}", file=sys.stderr)
        return 2

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

        motion = g.actions.config
        if args.speed is not None:
            if args.speed <= 0.0:
                print(f"error: --speed must be positive, got {args.speed:g} (a "
                      f"non-positive speed turns the closing ramp into a single "
                      f"frame, which reads as a failed grasp)", file=sys.stderr)
                return 2
            print(f"speed override: {motion.grasp_speed_mm_s:g} -> {args.speed:g} mm/s")
            motion.grasp_speed_mm_s = args.speed
        if args.kp is not None:
            print(f"kp override: {g.config.kp:g} -> {args.kp:g} "
                  f"(position stiffness, closing move only)")
            g.config.kp = args.kp
        open_speed = motion.speed_mm_s if args.open_speed is None else args.open_speed
        if open_speed <= 0.0:
            print(f"error: --open-speed must be positive, got {open_speed:g}",
                  file=sys.stderr)
            return 2
        if args.open_speed is not None:
            print(f"open speed override: {open_speed:g} mm/s")
        force_n = motion.force_n if args.force is None else args.force
        if force_n <= 0.0:
            print(f"error: --force must be positive, got {force_n:g}", file=sys.stderr)
            return 2
        if force_n > LOUD_FORCE_N:
            print(f"warning: {force_n:g} N = {force_n * N_TO_NM:g} Nm is above the "
                  f"SDK default ({LOUD_FORCE_N:g} N) — that is a firm grip on a "
                  f"rigid part", file=sys.stderr)

        source = _source_note(args)
        travel_mm = (abs(g.config.pos_open_rad - g.config.pos_closed_rad)
                     * g.config.rad_to_mm)
        _target, limit, _margin_rad, _ = limit_target(g.config, "close", motion.margin)
        print(f"calibration from {source}")
        print(f"  closed={g.config.pos_closed_rad:+.6f} rad  "
              f"open={g.config.pos_open_rad:+.6f} rad  mount={g.config.mount}")
        print(f"  travel={travel_mm:.1f} mm  rad_to_mm={g.config.rad_to_mm}  "
              f"kp={g.config.kp:g}  kd={g.config.kd:g}")
        print(f"  closed stop at {to_mm(g.config, limit):+.1f} mm — that is what "
              f"the empty grasp targets, to within reach_tol "
              f"{motion.reach_tol * g.config.rad_to_mm:.2f} mm")
        # The closing leg runs until it arrives or stalls, so there is no window
        # to run out of; what it can run out of is lead.  With a force setpoint
        # the budget caps how far the command may lead a *stale* reading, and a
        # jaw that cannot keep up with the schedule spends frames doing it —
        # which is the case `reached=False, stalled=False` reports.
        print(f"  the close ends on arrival or on a stall, command leading by at "
              f"most {motion.max_lead_mm:g} mm "
              f"(<= {motion.press_safety:g} x force with a setpoint)")
        print(f"  grasp speed={motion.grasp_speed_mm_s:g} mm/s  "
              f"precondition open at {open_speed:g} mm/s  "
              f"hold for {args.hold_s:g}s")
        print(f"  force={force_n:g} N -> {force_n * N_TO_NM:g} Nm feed-forward "
              f"(SDK: 1 N = {N_TO_NM:g} Nm); is_grasped() trips above "
              f"{g.config.grasp_torque_threshold:g} Nm")

        if args.dry_run:
            print("dry run: nothing was sent on the bus")
            return 0

        if not _confirm(args, force_n):
            print("aborted, nothing was sent on the bus")
            return 1

        if not g.connect():
            print(f"error: could not connect to {args.channel}", file=sys.stderr)
            return 1
        enable = g.enable()
        err = None if enable.state is None else enable.state.error_code
        print(f"connected {g}  enabled={enable.ok} tries={enable.tries} err={err}")
        if not enable.ok:
            # The three causes, most likely first.  A down interface is the one
            # that catches people out: connect() succeeds anyway, because
            # opening a SocketCAN socket does not touch the hardware, and the
            # interface only surfaces as ``[Errno 100] Network is down`` on the
            # first frame.
            print(f"error: enable failed after {enable.tries} tries (err={err}) — "
                  f"check that the interface is up (``ip -br link show "
                  f"{args.channel}`` must not say DOWN; bring it up with ``sudo ip "
                  f"link set {args.channel} up type can bitrate 1000000``), that "
                  f"the bitrate matches the gripper, and that the 24 V supply is "
                  f"on", file=sys.stderr)
            return 1

        failures = 0
        voided = 0
        for case in ([] if args.skip_empty else ["empty"]) + (
                [] if args.skip_loaded else ["loaded"]):
            if case == "empty":
                empty = True
                print("\n=== case 1: clear jaws "
                      "(expect reached=True stalled=False) ===")
                if not _pause("Clear the jaws, then press Enter: ",
                              "you to clear the jaws"):
                    return 1
            else:
                empty = False
                print("\n=== case 2: an object "
                      "(expect stalled=True reached=False) ===")
            # Both cases open first: a grasp that starts a millimetre from its
            # target measures nothing, and an object cannot go into shut jaws.
            if not _open_precondition(g, open_speed):
                voided += 1
                continue
            if not empty and not _pause(
                    "Put the object between the jaws, then press Enter: ",
                    "you to insert the object"):
                return 1
            label = "clear jaws" if empty else "object between the jaws"
            judge = _judge_empty if empty else _judge_loaded
            if not _do_grasp(g, Trace(motion.sample_interval), args, force_n,
                             label, judge):
                failures += 1
            if not empty:
                print("\nremove the object — the jaws are disabled when this "
                      "exits, so it will not be held", file=sys.stderr)

        cases = (0 if args.skip_empty else 1) + (0 if args.skip_loaded else 1)
        print(f"\n{cases - failures - voided} of {cases} case(s) behaved as "
              f"expected")
        if voided:
            print(f"{voided} case(s) could not be tested: the jaws did not open",
                  file=sys.stderr)
        return 1 if failures or voided else 0
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
        # nothing is left squeezing.  Report that instead of a traceback.
        print("\ninterrupted — motor disabled, the grip was released",
              file=sys.stderr)
        sys.exit(130)
    except LiteGripError as error:
        print(f"error: {error}", file=sys.stderr)
        sys.exit(1)
