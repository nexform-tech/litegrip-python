#!/usr/bin/env python3
"""Hold-force check: does the grip hold the force it was given, or does it sag?

Read this when a grasp reaches the force you set and then drifts to a smaller
one.  A held force is a feed-forward torque, so the measured torque during the
hold should climb to ``force_n x 0.1`` Nm over the first frames
(``MotionConfig.force_ramp_n_s``) and then sit there for as long as the hold
runs, whatever the object does underneath.  A hold that carried a position gain
instead had ``kp x (q - measured)`` added to that feed-forward, so the jaws
following a yielding object — or the encoder's own stick-slip steps — subtracted
from the grip: the jaws' own movement was the thing stealing the force.  This
script grasps once, samples the measured torque every
``MotionConfig.hold_interval`` for the whole hold, and prints each reading **as
it arrives** — the measured force in N next to the force you asked for — so a
drop shows up while it is happening rather than in a table afterwards.  The same
trace is repeated as a summary, and ``--csv`` writes it to a file for a bug
report or a between-builds comparison.

The verdict compares the measured trace against the feed-forward the SDK sent —
the climb to the setpoint, then the constant.  There is no threshold on the raw
number, because ``1 N = 0.1 Nm`` is approximate by design — what matters is
whether the reading *moves*.

* **sagged** — the second half of the hold read below ``--sag-ratio`` (default
  0.6) of the force asked for.  This is the reported symptom.
* **never arrived** — even the peak never reached half the setpoint, so the
  feed-forward did not reach the motor.  Check that the drive is enabled and
  powered, and that the calibration's mount matches how the gripper is mounted:
  ``close_sign`` picks the direction the torque is applied in.

**Put something that gives a little between the jaws.**  A block the jaws cannot
close on at all leaves them still, and the old defect subtracted nothing when
nothing moved — a flat trace against a rigid block is weak evidence, not a pass.
Rubber, foam, a soft tube or a spring clamp all work.  The script measures how
far the jaws moved during the hold and prints it next to the trace, so you can
judge the strength of the result yourself.

    # What would this do, on this machine, with the calibration it finds?
    python3 examples/hold_force_test.py --dry-run

    # The reported case: a big force, held long enough for a sag to show.
    python3 examples/hold_force_test.py --channel can0 --force 40 --hold-s 10

    # Keep the trace, for a bug report or a comparison between two builds.
    python3 examples/hold_force_test.py --force 20 --csv /tmp/hold.csv

**The jaws are disabled when the script exits, so whatever is being held
falls.**  Catch it, or put something soft underneath.  If you want a grip that
outlives the script, call ``grasp(..., hold_s=0)`` from a program instead —
``hold_s=0`` means "hold until a fault or Ctrl+C", which is why this script
never uses it: a test that holds forever is a test that never finishes.
"""

from __future__ import annotations

import argparse
import csv
import logging
import os
import sys

# Like the other examples: import the SDK straight out of the checkout, so the
# script works without `pip install -e .`.  Inserted first, so the checkout wins
# over an installed copy — running it exercises the code next to it.
_SRC = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "src")
if _SRC not in sys.path:
    sys.path.insert(0, _SRC)

from litegrip import (LiteGrip, LiteGripError, UnitConversion,  # noqa: E402
                      default_calib_path)
from litegrip.can import MotorType  # noqa: E402

#: The SDK's own force/torque mapping (``constants.UnitConversion``), repeated
#: nowhere: if the mapping is ever changed, this script changes with it.
N_TO_NM = UnitConversion.N_TO_NM
NM_TO_N = UnitConversion.NM_TO_N

#: Long enough that a sag has time to show: the reported case took a few
#: seconds, and the break-even is not the first slice.
DEFAULT_HOLD_S = 8.0
#: Second-half mean below this fraction of the setpoint counts as a sag.
SAG_RATIO = 0.6
#: Peak below this fraction means the feed-forward never got to the motor.
ARRIVED_RATIO = 0.5
#: Above this the grip is firm enough to damage a part; warn, do not refuse.
LOUD_FORCE_N = 20.0
#: How far the jaws have to move during the hold before the run is evidence
#: that the hold's force is independent of jaw position.
USEFUL_DRIFT_MM = 0.1
#: Width of the bar drawn next to each sample.
BAR_COLS = 36


def _hex(value: str) -> int:
    """Parse an int written as 0x08, 8, or 0o10 on the command line."""
    return int(value, 0)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Grasp once and report whether the held force stays at the "
                    "setpoint or sags over the hold.")
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
                        help=f"how long to hold, s (default: {DEFAULT_HOLD_S:g}); "
                             f"0 is refused, because the SDK reads it as 'hold "
                             f"forever'")
    parser.add_argument("--sag-ratio", type=float, default=SAG_RATIO,
                        help=f"a second-half mean below this fraction of the "
                             f"setpoint counts as a sag (default: {SAG_RATIO:g})")
    parser.add_argument("--speed", type=float, default=None,
                        help="closing speed in mm/s (default: "
                             "MotionConfig.grasp_speed_mm_s)")
    parser.add_argument("--open-speed", type=float, default=None,
                        help="speed in mm/s for the precondition open, which "
                             "runs before the grasp (default: "
                             "MotionConfig.speed_mm_s)")
    parser.add_argument("--kp", type=float, default=None,
                        help="override the calibration's position stiffness (the "
                             "DM stiffness code, 0-500).  It shapes the closing "
                             "move only; the hold is a bare torque command and "
                             "ignores it.  Default: the value in the calibration "
                             "file")
    parser.add_argument("--hold-interval", type=float, default=None,
                        help="seconds per hold slice (default: "
                             "MotionConfig.hold_interval, 0.2).  Each slice is "
                             "one emitted torque plateau and one status read, so "
                             "a smaller value samples the hold more finely — at "
                             "the cost of reading the drive's torque estimate "
                             "more often than the engine would")
    parser.add_argument("--calib", default=None,
                        help="explicit calibration JSON; default is this "
                             "channel's own file, then the legacy/factory one")
    parser.add_argument("--template", default=None,
                        help="calibration template name; see litegrip.list_templates()")
    parser.add_argument("--csv", default=None,
                        help="write the hold trace to this path as CSV "
                             "(t_s,pos_rad,pos_mm,torque_nm,force_n)")
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


class HoldTrace:
    """Collects the hold's per-slice readings from the engine's callback.

    ``GraspResult`` reports that the hold finished and how many slices it ran;
    it reports nothing about what the hold *applied*.  The callback fires once
    per slice with the torque the drive reported for that slice, which is the
    one signal that separates a held force from a decaying one.

    ``on_sample`` is called with each row as it arrives, so the force can be
    watched while the hold runs instead of only read back afterwards.  It is
    optional because the tests and the offline analysis want the rows without
    the printing.
    """

    def __init__(self, hold_interval_s: float, on_sample=None):
        self.hold_interval_s = hold_interval_s
        self.on_sample = on_sample
        #: ``(t_s, pos_rad, torque_nm)``, one row per hold slice.
        self.samples: list[tuple[float, float, float]] = []

    def __call__(self, progress) -> None:
        if progress.phase != "hold":
            return
        row = (progress.i * self.hold_interval_s,
               progress.pos_rad, progress.torque_nm)
        self.samples.append(row)
        if self.on_sample is not None:
            self.on_sample(*row)

    def forces_n(self) -> list[float]:
        """The measured hold force per slice, in N, sign removed.

        The feed-forward is signed by the mount (``close_sign``): a reverse
        mount clamps with a negative torque.  The size is what is being judged.
        """
        return [abs(torque_nm) * NM_TO_N for _t, _pos, torque_nm in self.samples]

    def drift_mm(self, cfg) -> float:
        """How far the jaws moved while the hold ran, in mm."""
        if len(self.samples) < 2:
            return 0.0
        first = self.samples[0][1]
        last = self.samples[-1][1]
        return abs(to_mm(cfg, last) - to_mm(cfg, first))


class RampEnd:
    """The last sample of the closing ramp, so the handover can be reported.

    ``grasp`` passes its callback to both phases: the closing ramp, then the
    hold.  Where the jaws were when the torque took over explains a lurch at
    the object — a ramp that ended short of the empty target hands the hold a
    gap to cross at force — and it is otherwise invisible, because the grasp
    result only carries the state left behind at the very end.
    """

    def __init__(self):
        self.pos_rad: float | None = None
        self.total_steps = 0
        self.i = 0

    def __call__(self, progress) -> None:
        if progress.phase != "move":
            return
        self.pos_rad = progress.pos_rad
        self.total_steps = progress.total_steps
        self.i = progress.i


def handover(trace: HoldTrace, ramp: RampEnd, cfg) -> str:
    """The line that separates what the ramp did from what the hold did."""
    if ramp.pos_rad is None:
        return "  close: the ramp reported no sample, so its end is unknown"
    return (f"  close: the ramp ended at {to_mm(cfg, ramp.pos_rad):+.1f} mm "
            f"after {ramp.i}/{ramp.total_steps} frames; "
            f"the hold started from {to_mm(cfg, trace.samples[0][1]):+.1f} mm "
            f"and ended at {to_mm(cfg, trace.samples[-1][1]):+.1f} mm")


def analyse(trace: HoldTrace, cfg, force_n: float, result,
            sag_ratio: float) -> tuple[list[str], list[str]]:
    """Judge one grasp.  Returns ``(problems, notes)``.

    ``problems`` are failures of the question this script asks; ``notes`` are
    readings worth saying out loud that do not change the verdict.
    """
    problems: list[str] = []
    notes: list[str] = []

    if result.cycles == 0 or not trace.samples:
        problems.append("the hold never ran a slice, so there is nothing to "
                        "judge — the closing move did not get to the hold")
        return problems, notes
    if len(trace.samples) != result.cycles:
        notes.append(f"the engine counted {result.cycles} hold slices but the "
                     f"callback delivered {len(trace.samples)}; the numbers "
                     f"below cover the ones that arrived")
    if not result.ok:
        problems.append(f"the hold ended abnormally (err="
                        f"{result.state.error_code}), so the force was not held "
                        f"for the whole time asked for")

    forces = trace.forces_n()
    asked_nm = force_n * N_TO_NM
    peak = max(forces)
    if peak < ARRIVED_RATIO * force_n:
        problems.append(
            f"the measured force never reached half of the {force_n:g} N asked "
            f"for (peak {peak:.1f} N = {peak / NM_TO_N:.3f} Nm against the "
            f"{asked_nm:g} Nm feed-forward) — the torque is not arriving at the "
            f"motor.  Check that the drive is enabled and powered, and that the "
            f"calibration's mount matches the hardware: close_sign decides which "
            f"way the torque pushes")

    half = max(1, len(forces) // 2)
    first_half = _mean(forces[:half])
    second_half = _mean(forces[half:])
    if second_half < sag_ratio * force_n:
        problems.append(
            f"the grip sagged: the second half of the hold averaged "
            f"{second_half:.1f} N against the {force_n:g} N asked for "
            f"({second_half / force_n:.0%} of the setpoint), having started at "
            f"{first_half:.1f} N.  A hold that decays like this has a position "
            f"or velocity term in it — check that the hold frames carry "
            f"kp = kd = 0")

    if result.reached and not result.stalled:
        problems.append(
            "the closing move reached the empty target without meeting "
            "anything, so there was no object in the jaws: what was measured is "
            "the hold against the closed stop, not a grip on a part")

    drift_mm = trace.drift_mm(cfg)
    if drift_mm < USEFUL_DRIFT_MM:
        notes.append(
            f"the jaws moved only {drift_mm:.2f} mm during the hold.  A hold "
            f"whose force decays with jaw position cannot show that decay "
            f"without some movement, so this trace is weak evidence — put "
            f"something softer between the jaws (rubber, foam, a tube) and run "
            f"it again")
    if peak > 1.2 * force_n:
        notes.append(f"the measured force peaked at {peak:.1f} N, above the "
                     f"{force_n:g} N asked for; the motor reports its own torque "
                     f"estimate, and a contact transient reads high in it")
    return problems, notes


def _mean(values) -> float:
    return sum(values) / len(values) if values else 0.0


def trace_header(force_n: float, hold_interval_s: float,
                 ramp_n_s: float) -> str:
    """The column headings, printed once before the hold starts."""
    return "\n".join([
        f"  the SDK ramps the hold to {force_n:g} N "
        f"({force_n * N_TO_NM:g} Nm) at {ramp_n_s:g} N/s, then holds it;",
        f"  one line per {hold_interval_s:g}s slice, printing the force the "
        f"drive reported:",
        "",
        "     t(s)   measured   / set   % of set   pos mm   measured / setpoint",
    ])


def trace_line(cfg, force_n: float, t_s: float, pos_rad: float,
               torque_nm: float) -> str:
    """One hold slice, as it arrives: the measured force next to the setpoint.

    This is the whole point of the script, so it prints while the hold runs.
    The force is ``|torque| x 10`` N, because the mount's sign (``close_sign``)
    only decides which way the torque pushes.
    """
    force = abs(torque_nm) * NM_TO_N
    ratio = force / force_n if force_n > 0 else 0.0
    bar = "#" * max(0, min(BAR_COLS, int(round(ratio * BAR_COLS))))
    return (f"    {t_s:5.1f}   {force:7.1f} N   {force_n:<5.1f}   "
            f"{ratio:7.1%}   {to_mm(cfg, pos_rad):6.1f}   |{bar}")


def _summary(trace: HoldTrace, ramp: RampEnd, cfg, force_n: float,
             result) -> str:
    """The numbers behind the verdict, printed once the hold has finished."""
    forces = trace.forces_n()
    asked_nm = force_n * N_TO_NM
    if not forces:
        return "\n".join([
            "",
            f"  the hold ran no slices (result.cycles={result.cycles}, "
            f"callbacks={len(trace.samples)}), so there is no trace to "
            f"print — the closing move never got to the hold",
            f"  close: reached={result.reached} stalled={result.stalled} "
            f"err={result.state.error_code}",
        ])
    half = max(1, len(forces) // 2)
    first_half = _mean(forces[:half])
    second_half = _mean(forces[half:])
    return "\n".join([
        "",
        f"  hold: {result.cycles} slices x {trace.hold_interval_s:g}s = "
        f"{result.cycles * trace.hold_interval_s:.1f}s; "
        f"asked {force_n:g} N ({asked_nm:g} Nm)",
        f"  measured force: first {forces[0]:.2f} N, last {forces[-1]:.2f} N, "
        f"min {min(forces):.2f} N, max {max(forces):.2f} N, "
        f"mean {_mean(forces):.2f} N",
        f"  over the hold: {forces[-1] - forces[0]:+.2f} N from first to last, "
        f"{second_half - first_half:+.2f} N from the first half to the second",
        f"  jaws moved {trace.drift_mm(cfg):.2f} mm during the hold "
        f"({to_mm(cfg, trace.samples[0][1]):+.1f} -> "
        f"{to_mm(cfg, trace.samples[-1][1]):+.1f} mm)",
        f"  close: reached={result.reached} stalled={result.stalled} "
        f"err={result.state.error_code}, empty target "
        f"{to_mm(cfg, result.target_rad):+.1f} mm",
        handover(trace, ramp, cfg),
    ])


def write_csv(path: str, trace: HoldTrace, cfg) -> None:
    """Save the trace, so a run can be compared with another one later."""
    with open(path, "w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(["t_s", "pos_rad", "pos_mm", "torque_nm", "force_n"])
        for t_s, pos_rad, torque_nm in trace.samples:
            writer.writerow([f"{t_s:.3f}", f"{pos_rad:.6f}",
                             f"{to_mm(cfg, pos_rad):.3f}",
                             f"{torque_nm:.6f}",
                             f"{abs(torque_nm) * NM_TO_N:.3f}"])


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


def _pause(prompt: str, waiting_for: str) -> bool:
    """Wait for the operator to do something physical.  False if there is nobody.

    ``waiting_for`` describes the wait in the refusal message, because the
    prompt itself is a whole sentence and reads badly inside one.
    """
    if not sys.stdin.isatty():
        print(f"error: cannot wait for {waiting_for} — stdin is not a terminal, "
              f"so there is nobody to ask; use --yes to run without prompts",
              file=sys.stderr)
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
    print(f"\nAbout to grasp once on {args.channel}: {force_n:g} N = "
          f"{force_n * N_TO_NM:g} Nm, held for {args.hold_s:g}s, jaws disabled "
          f"at the end (the part will drop).")
    return _pause("Press Enter to start, Ctrl+C to abort: ",
                  "confirmation before moving the jaws")


def _open_precondition(g: LiteGrip, open_speed: float) -> bool:
    """Open the jaws so the grasp starts from a known position.

    The grip has to be measured on an object, and an object cannot go between
    jaws that are already shut — so the open is a precondition, not the
    subject.  A failed open is reported rather than run through: the jaws are
    either jammed or moving against something, and a grasp from there would
    measure that instead.  ``python3 examples/stroke_test.py`` exercises the
    open on its own.
    """
    move = g.open(open_speed)
    print(f"\nprecondition, open at {open_speed:g} mm/s: ok={move.ok} "
          f"reached={move.reached} stalled={move.stalled} "
          f"err={move.state.error_code}, ended at "
          f"{move.state.position_mm:+.1f} mm")
    if not move.ok:
        print("  the jaws did not open, so the grasp cannot be run — check the "
              "jaws are clear, then try `--open-speed 10` (a slower open asks "
              "the stall detector for far less movement per window) or "
              "`--kp 100` (more breakaway torque)", file=sys.stderr)
    return move.ok


def _run(args) -> int:
    if args.hold_s <= 0.0:
        print(f"error: --hold-s must be positive, got {args.hold_s:g} (the SDK "
              f"reads 0 as 'hold forever', which a test script must not do)",
              file=sys.stderr)
        return 2
    if not 0.0 < args.sag_ratio <= 1.0:
        print(f"error: --sag-ratio must be in (0, 1], got {args.sag_ratio:g}",
              file=sys.stderr)
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
                print(f"error: --speed must be positive, got {args.speed:g}",
                      file=sys.stderr)
                return 2
            print(f"speed override: {motion.grasp_speed_mm_s:g} -> "
                  f"{args.speed:g} mm/s")
            motion.grasp_speed_mm_s = args.speed
        if args.hold_interval is not None:
            if args.hold_interval <= 0.0:
                print(f"error: --hold-interval must be positive, got "
                      f"{args.hold_interval:g}", file=sys.stderr)
                return 2
            print(f"hold interval override: {motion.hold_interval:g} -> "
                  f"{args.hold_interval:g}s per slice")
            motion.hold_interval = args.hold_interval
        if args.kp is not None:
            print(f"kp override: {g.config.kp:g} -> {args.kp:g} "
                  f"(position stiffness, closing move only — the hold ignores it)")
            g.config.kp = args.kp
        open_speed = motion.speed_mm_s if args.open_speed is None else args.open_speed
        if open_speed <= 0.0:
            print(f"error: --open-speed must be positive, got {open_speed:g}",
                  file=sys.stderr)
            return 2
        force_n = motion.force_n if args.force is None else args.force
        if force_n <= 0.0:
            print(f"error: --force must be positive, got {force_n:g}",
                  file=sys.stderr)
            return 2
        if force_n > LOUD_FORCE_N:
            print(f"warning: {force_n:g} N = {force_n * N_TO_NM:g} Nm is above "
                  f"the SDK default ({LOUD_FORCE_N:g} N) — a firm grip on a "
                  f"rigid part", file=sys.stderr)

        # The trace prints each slice as it arrives, so the force can be watched
        # while the hold runs — a decay that only shows up in a table read
        # afterwards is exactly the thing this script exists to make visible.
        trace = HoldTrace(
            motion.hold_interval,
            on_sample=lambda t_s, pos_rad, torque_nm: print(
                trace_line(g.config, force_n, t_s, pos_rad, torque_nm)))
        ramp = RampEnd()

        def on_progress(progress) -> None:
            """One callback for both phases: the ramp's end, then the hold."""
            ramp(progress)
            trace(progress)

        print(f"calibration from {_source_note(args)}")
        print(f"  closed={g.config.pos_closed_rad:+.6f} rad  "
              f"open={g.config.pos_open_rad:+.6f} rad  mount={g.config.mount}")
        print(f"  travel={abs(g.config.pos_open_rad - g.config.pos_closed_rad) * g.config.rad_to_mm:.1f} mm"
              f"  rad_to_mm={g.config.rad_to_mm}  kp={g.config.kp:g}  "
              f"kd={g.config.kd:g}")
        print(f"  grasp speed={motion.grasp_speed_mm_s:g} mm/s  "
              f"precondition open at {open_speed:g} mm/s")
        print(f"  force={force_n:g} N -> {force_n * N_TO_NM:g} Nm feed-forward "
              f"held for {args.hold_s:g}s, sampled every "
              f"{motion.hold_interval:g}s")

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

        print("\n=== the hold ===")
        if not _open_precondition(g, open_speed):
            print("\nnothing was tested: the jaws did not open", file=sys.stderr)
            return 1
        if not _pause("Put the object between the jaws, then press Enter: ",
                      "you to insert the object"):
            return 1

        print(f"\n=== the hold: watch the force column below ===")
        print(trace_header(force_n, motion.hold_interval,
                           motion.force_ramp_n_s))
        result = g.grasp(force_n, args.hold_s, progress=on_progress)
        print(_summary(trace, ramp, g.config, force_n, result))
        problems, notes = analyse(trace, g.config, force_n, result,
                                  args.sag_ratio)
        for note in notes:
            print(f"  note: {note}")
        for problem in problems:
            print(f"  FAIL: {problem}", file=sys.stderr)

        if args.csv is not None:
            write_csv(args.csv, trace, g.config)
            print(f"  trace written to {args.csv}")

        print("\nremove the object — the jaws are disabled when this exits, so "
              "it will not be held", file=sys.stderr)
        forces = trace.forces_n()
        drift = (f"asked {force_n:g} N, measured {forces[0]:.2f} N at the start "
                 f"and {forces[-1]:.2f} N at the end ({forces[-1] - forces[0]:+.2f} N "
                 f"over {result.cycles * trace.hold_interval_s:.1f}s)"
                 if forces else f"asked {force_n:g} N, but no slice was measured")
        if problems:
            print(f"\nFAILED: {len(problems)} problem(s) above — {drift}",
                  file=sys.stderr)
            return 1
        print(f"\nOK: no drop in the hold — {drift}, {_mean(forces):.2f} N mean")
        return 0
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
