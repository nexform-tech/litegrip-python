"""Runs the teleop follower's torque guard end to end and prints what it checked.

For anyone who wants to see the guard work — and to see where it does not —
before trusting it: run

    cd /home/qaz/litegrip-python
    python3 tests/torque_guard_check.py

It drives a follower into a rigid obstacle with a leader the check steers itself,
and prints every number behind each claim the docs make: that the guard trips on
the press, goes to zero stiffness and damping in place, keeps streaming so the
motor does not latch its comm-loss fault, does not chatter while the leader holds
at the stop, re-arms once the leader has reopened past `TORQUE_REARM_OPENNESS`,
and trips again on the next stroke.  It then runs the bench command the README and
`examples/teleop.py` tell a user to run, and checks that it does what they say.

Each step prints ``ok`` or ``FAIL``, and the exit code is 1 if any step failed, so
it can also be run from a script.

This is not the test suite.  That is ``tests/test_example_teleop.py``, which covers
the guard case by case under ``unittest``; this file is deliberately not named
``test_*`` so ``unittest discover`` does not collect it twice.  What it adds is one
run whose output a person can read.

It proves how the guard behaves against the kinematic fake CAN in
``tests/fake_can.py``.  It does **not** prove anything about real hardware: the
fake moves its jaw at ``v = dq + GAIN * (q - pos)`` and reports ``tau = kp * (q -
pos)``, so its torque is a function of following error alone.  A real motor reports
current, and its `kd * dq` term is a torque term the fake does not model.

Worse, the fake integrates a fixed ``DT = 0.005`` per frame whatever the loop rate,
so it only keeps time at 200 Hz.  At the example's default 50 Hz it runs four times
slower than the loop, the follower never catches up, and a free sweep with nothing
between the jaws reads about 2 Nm.  The documented bench command passes
``--torque-limit 1.0`` at that rate, so on the fake it trips before it reaches
anything — a property of the fake, which the run prints as a note rather than
scoring it against the guard.  Numbers from this run are evidence about the guard's
logic, not about a gripper.
"""

from __future__ import annotations

import importlib.util
import math
import os
import sys
import threading
import time
from pathlib import Path


def _find_root() -> Path:
    """The repo root, so this file runs from ``tests/`` and from anywhere else."""
    candidates = []
    if "LITEGRIP_REPO" in os.environ:
        candidates.append(Path(os.environ["LITEGRIP_REPO"]))
    here = Path(__file__).resolve()
    candidates += [Path.cwd(), here.parent, here.parent.parent]
    for candidate in candidates:
        if ((candidate / "src" / "litegrip").is_dir()
                and (candidate / "tests" / "fake_can.py").is_file()):
            return candidate.resolve()
    raise SystemExit("run this from the litegrip-python repo root, "
                     "or set LITEGRIP_REPO to it")


_ROOT = _find_root()
sys.path[:0] = [str(_ROOT / "src"), str(_ROOT / "tests")]

from litegrip import InProcTeleopTransport, encode_frame, teleop_topic  # noqa: E402
from litegrip.teleop import rad_to_openness  # noqa: E402
from fake_can import (POS_CLOSED_RAD, POS_OPEN_RAD,  # noqa: E402
                      RAD_TO_MM, make_gripper)

_EXAMPLE = _ROOT / "examples" / "teleop.py"
_spec = importlib.util.spec_from_file_location("example_teleop", _EXAMPLE)
example = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(example)

# ── the bench this check runs on ────────────────────────────────────────────
# The obstacle sits half way through the stroke: the follower presses on it well
# clear of either end, so a trip here cannot be confused with one at full open.
OBSTACLE_RAD = (POS_OPEN_RAD + POS_CLOSED_RAD) / 2.0
TRAVEL_MM = abs(POS_OPEN_RAD - POS_CLOSED_RAD) * RAD_TO_MM
TOPIC = teleop_topic("gripA")

# The configuration the guard is documented to work in: their own test's rates,
# where the sweep is slow enough that following alone never crosses the limit.
RATE_HZ = 200.0
RAMP_PER_S = 0.4
LIMIT_NM = 1.0

# The bench command the README and the example print for a user to run.
DOCUMENTED_ARGV = ["--mode", "slave", "--fake-leader", "--torque-limit", "1.0"]

_results: list[bool] = []


def check(label: str, ok: bool, detail: str = "") -> bool:
    _results.append(bool(ok))
    print(f"{'ok  ' if ok else 'FAIL'} {label}" + (f"\n       {detail}" if detail else ""))
    return bool(ok)


def note(label: str, detail: str = "") -> None:
    """Something worth reading that is not a claim about the guard."""
    print(f"note {label}" + (f"\n       {detail}" if detail else ""))


def wait(predicate, timeout_s: float) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.002)
    return bool(predicate())


class Leader:
    """A leader the check steers, so the sweep is exact rather than timed.

    ``examples.teleop._FakeLeader`` sweeps a triangle and cannot be told to hold
    still, which the no-chatter check needs.  This publishes whatever opening it
    is handed, at the follower's rate.
    """

    def __init__(self, bus, topic: str, rate_hz: float) -> None:
        self._bus, self._topic = bus, topic
        self._dt = 1.0 / rate_hz
        self.openness = 1.0
        self._halt = threading.Event()
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        self._thread = threading.Thread(target=self._run, name="check-leader",
                                        daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._halt.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)

    def ramp(self, target: float, rate: float = RAMP_PER_S,
             timeout_s: float = 15.0) -> bool:
        """Walk the published opening to ``target``; True if it got there."""
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            delta = target - self.openness
            if abs(delta) <= rate * self._dt:
                self.openness = target
                return True
            self.openness += math.copysign(rate * self._dt, delta)
            time.sleep(self._dt)
        return False

    def _run(self) -> None:
        while not self._halt.is_set():
            t0 = time.monotonic()
            self._bus.pub(self._topic, encode_frame(
                self.openness, self.openness * TRAVEL_MM, 0.0, time.time()))
            time.sleep(max(0.0, self._dt - (time.monotonic() - t0)))


def follower(limit_nm: float, rate_hz: float = RATE_HZ):
    """A calibrated, enabled follower in front of the obstacle."""
    g, fake = make_gripper(start_rad=POS_OPEN_RAD, block_rad=OBSTACLE_RAD)
    g.motion_config.sleep_fn = time.sleep         # real pacing: watch it in time
    bus = InProcTeleopTransport()
    g.teleop_start("slave", transport=bus, align=False,
                   torque_limit_nm=limit_nm, rate_hz=rate_hz)
    return g, fake, bus


def free_sweep_peak(rate_hz: float, openness_rate: float = 0.3) -> float:
    """Peak torque the fake reports following a sweep with nothing to press.

    No obstacle is installed, so every newton-metre here is the fake's own
    tracking error.  It is the yardstick for reading a trip that happens before
    the follower has reached anything.
    """
    g, _ = make_gripper(start_rad=POS_OPEN_RAD)          # no block_rad
    g.motion_config.sleep_fn = time.sleep
    bus = InProcTeleopTransport()
    g.teleop_start("slave", transport=bus, align=False, torque_limit_nm=0.0,
                   rate_hz=rate_hz)
    leader = Leader(bus, TOPIC, rate_hz)
    leader.start()
    try:
        threading.Thread(target=leader.ramp,
                         args=(1.0 - openness_rate * 1.5, openness_rate),
                         daemon=True).start()
        peak = 0.0
        deadline = time.monotonic() + 1.5
        while time.monotonic() < deadline:
            peak = max(peak, abs(g.teleop_status()["torque_nm"]))
            time.sleep(0.005)
        return peak
    finally:
        leader.stop()
        g.teleop_stop()


# ── the guard, in the configuration it is documented to work in ─────────────

def check_guard_engages() -> None:
    g, fake, bus = follower(LIMIT_NM)
    print(f"\nthe guard, at {RATE_HZ:.0f} Hz against an obstacle at openness "
          f"{rad_to_openness(OBSTACLE_RAD, g.config):.2f}")
    leader = Leader(bus, TOPIC, RATE_HZ)
    leader.start()
    try:
        sweep = threading.Thread(target=leader.ramp, args=(0.0,), daemon=True)
        sweep.start()

        tripped = wait(lambda: g.teleop_status()["over_torque"], 15.0)
        st = g.teleop_status()
        check("trips when the follower presses on the obstacle",
              tripped and st["openness"] <= 0.9,
              f"over_torque={st['over_torque']} at leader openness "
              f"{st['openness']:.3f}, torque {st['torque_nm']:.2f} Nm vs limit "
              f"{LIMIT_NM:.2f} Nm\n       the trip must come from the press, not "
              f"from the transient at start-up: openness stays under 0.9")

        # Released in place: zero stiffness and zero damping, but the frames keep
        # flowing so the motor does not latch its comm-loss fault.
        mark = len(fake.frames)
        time.sleep(0.2)
        sent = fake.frames[mark:]
        gains = sorted({(f.kp, f.kd) for f in sent})
        check("every frame sent while tripped is zero-stiffness, zero-damping",
              bool(sent) and all(f.kp == 0.0 and f.kd == 0.0 for f in sent),
              f"{len(sent)} frames, gains {gains}")
        check("it keeps streaming while tripped",
              len(sent) >= RATE_HZ * 0.15,
              f"{len(sent)} frames in 200 ms at {RATE_HZ:.0f} Hz; a silent "
              f"follower would latch the motor's comm-loss fault")

        # Latching: held at the stop, it stays released instead of chattering
        # back on against the same obstruction.
        sweep.join(timeout=10.0)
        trips_before = g.teleop_status()["torque_trips"]
        mark = len(fake.frames)
        time.sleep(0.5)
        sent = fake.frames[mark:]
        reengaged = sum(1 for f in sent if f.kp > 0.0)
        check("it stays released while the leader holds at the stop",
              g.teleop_status()["over_torque"]
              and g.teleop_status()["torque_trips"] == trips_before
              and reengaged == 0,
              f"trips {trips_before} -> {g.teleop_status()['torque_trips']}, "
              f"{reengaged} re-engaging frames of {len(sent)} in 500 ms")

        # Re-arm: the leader reopens past the margin.
        trip_openness = st["openness"]
        target = min(1.0, trip_openness + 0.15)
        threading.Thread(target=leader.ramp, args=(target,), daemon=True).start()
        rearmed = wait(lambda: not g.teleop_status()["over_torque"], 15.0)
        check(f"re-arms once the leader reopens past "
              f"{0.05:.2f} (tripped at openness {trip_openness:.3f}, "
              f"reopened to {target:.3f})",
              rearmed, f"over_torque={g.teleop_status()['over_torque']} at "
                       f"openness {g.teleop_status()['openness']:.3f}")

        mark = len(fake.frames)
        time.sleep(0.1)
        sent = fake.frames[mark:]
        check("it follows again after re-arming",
              bool(sent) and all(f.kp > 0.0 for f in sent),
              f"gains {sorted({(f.kp, f.kd) for f in sent})}")

        # And trips again on the next stroke, rather than latching off.
        trips_before = g.teleop_status()["torque_trips"]
        threading.Thread(target=leader.ramp, args=(0.0,), daemon=True).start()
        again = wait(lambda: g.teleop_status()["torque_trips"] > trips_before, 15.0)
        check("trips again on the next stroke",
              again, f"trips {trips_before} -> {g.teleop_status()['torque_trips']}")
    finally:
        leader.stop()
        g.teleop_stop()


def check_guard_off_by_default() -> None:
    print("\nthe guard off (``torque_limit_nm=0``, the SDK default)")
    g, fake, bus = follower(0.0)
    leader = Leader(bus, TOPIC, RATE_HZ)
    leader.start()
    try:
        sweep = threading.Thread(target=leader.ramp, args=(0.0,), daemon=True)
        sweep.start()
        sweep.join(timeout=10.0)
        time.sleep(0.5)
        st = g.teleop_status()
        sent = fake.frames[-int(RATE_HZ * 0.2):]
        check("leaves the follower pressing, with no trip",
              st["torque_trips"] == 0 and not st["over_torque"],
              f"trips={st['torque_trips']} over_torque={st['over_torque']} "
              f"torque {st['torque_nm']:.2f} Nm while pressing")
        check("and keeps the follow gains on while it presses",
              bool(sent) and all(f.kp > 0.0 for f in sent),
              f"gains {sorted({(f.kp, f.kd) for f in sent})}")
    finally:
        leader.stop()
        g.teleop_stop()


# ── the command the docs tell a user to run ─────────────────────────────────

def check_documented_command() -> None:
    """Run the bench command from README.md and examples/teleop.py verbatim.

    The script itself is not run: it opens a real CAN channel.  Everything it
    passes to the SDK is reproduced instead, straight from its own parser, so the
    only difference is the fake CAN in place of ``can0``.
    """
    args = example.build_parser().parse_args(DOCUMENTED_ARGV)
    print(f"\nthe documented bench command: "
          f"examples/teleop.py {' '.join(DOCUMENTED_ARGV)}")
    print(f"       defaults it runs with: rate={args.rate:.0f} "
          f"openness_rate={args.openness_rate} dq_max={args.dq_max} "
          f"kp={args.kp} kd={args.kd} align={not args.no_align}")

    free_peak = free_sweep_peak(args.rate, args.openness_rate)

    bus = InProcTeleopTransport()
    g, fake = make_gripper(start_rad=POS_OPEN_RAD, block_rad=OBSTACLE_RAD)
    g.motion_config.sleep_fn = time.sleep
    leader = example._FakeLeader(
        bus, teleop_topic(args.grip_id), TRAVEL_MM, args.rate, args.openness_rate)
    g.teleop_start(args.mode, transport=bus, grip_id=args.grip_id, kp=args.kp,
                   kd=args.kd, align=not args.no_align, watchdog_s=args.watchdog,
                   dq_max=args.dq_max, rate_hz=args.rate,
                   torque_limit_nm=args.torque_limit)
    leader.start()          # started after teleop_start, as the script does
    try:
        worker = g._teleop
        trip = None
        rearm = None
        strokes = 0
        last_dir = None
        t0 = time.monotonic()
        while time.monotonic() - t0 < 20.0:
            if trip is None and worker._over_torque:
                trip = (time.monotonic() - t0, worker._trip_openness,
                        worker._torque_nm)
            if trip is not None and rearm is None and not worker._over_torque:
                rearm = (time.monotonic() - t0, worker._last_openness)
                break
            direction = math.copysign(1.0, leader._direction)
            if last_dir is not None and direction != last_dir:
                strokes += 1
            last_dir = direction
            time.sleep(0.005)

        check("the guard trips, as the docs say it will",
              trip is not None,
              "no trip in 20 s" if trip is None else
              f"tripped at t={trip[0]:.2f}s, leader openness {trip[1]:.3f}, "
              f"torque {trip[2]:.2f} Nm vs limit {args.torque_limit:.2f} Nm")
        if trip is None:
            return
        check("it re-arms, as the docs say it will",
              rearm is not None,
              f"watched {strokes} turns of the sweep, "
              f"{time.monotonic() - t0:.1f}s, and the guard never re-armed"
              if rearm is None else
              f"re-armed at t={rearm[0]:.2f}s, leader openness {rearm[1]:.3f}")
        note("where that trip came from is a property of the fake, not the guard",
             f"it tripped at openness {trip[1]:.3f}, near full open, before the "
             f"sweep reached the obstacle — and {free_peak:.2f} "
             f"Nm is what the fake reports for a sweep with **nothing between "
             f"the jaws at all** at this rate.  FakeMotor integrates a fixed "
             f"DT=0.005 per frame whatever the loop rate, so at {args.rate:.0f} "
             f"Hz the fake runs four times slower than the loop and the follower "
             f"never catches up.  This command's follower is real hardware "
             f"(--channel can0), which this check cannot reach.")
    finally:
        leader.stop()
        g.teleop_stop()


def main() -> int:
    started = time.monotonic()
    check_guard_engages()
    check_guard_off_by_default()
    check_documented_command()

    failed = _results.count(False)
    print(f"\n{len(_results)} checks, {failed} failed, "
          f"{time.monotonic() - started:.0f}s")
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
