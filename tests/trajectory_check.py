"""Runs the whole trajectory record/replay path end to end and prints what it checked.

For anyone who wants to see the feature work before trusting it: run

    cd /home/qaz/litegrip-python
    python3 tests/trajectory_check.py

It teaches a path, saves it, loads it back, replays it on the unit that recorded
it and on a reverse-mounted one, and prints every number behind each step.  Each
step prints ``ok`` or ``FAIL``, and the exit code is 1 if any step failed, so it
can also be run from a script.

This is not the test suite.  That is ``tests/test_trajectory.py``, which covers
the same ground case by case under ``unittest``; this file is deliberately not
named ``test_*`` so ``unittest discover`` does not collect it twice.  What it
adds is one run whose output a person can read.

It proves the recording is the motion that was applied, that the file is the
recording, and that the replay is the file — sample for sample.  It does **not**
prove anything about real hardware: the motion comes from the kinematic fake CAN
in ``tests/fake_can.py`` and the clock is a fake, so no servo, no bus and no
mechanics are involved.
"""

from __future__ import annotations

import math
import os
import tempfile
import time

import _sdkpath  # noqa: F401  (puts src/ on sys.path)
from litegrip import (InProcTeleopTransport, TeleopBusyError, Trajectory,
                      TrajectoryBusyError, TrajectoryEmptyError,
                      TrajectoryError, TrajectoryFormatError)
from litegrip.teleop import openness_to_rad, rad_to_openness

from fake_can import POS_CLOSED_RAD, POS_OPEN_RAD, RAD_TO_MM, make_gripper

# ── what is being taught ────────────────────────────────────────────────────
#
# The hand sweeps the jaws closed -> fully open -> closed over DURATION seconds.
# RATE_HZ is the capture rate; INTERVAL is MotionConfig's ramp frame interval,
# which is also the replay's frame rate (so a replay runs at 200 Hz).
DURATION = 2.0
RATE_HZ = 100.0
INTERVAL = 0.005
ALIGN_S = 1.0           # the goto_rad duration a replay's align move uses
TOL = 1e-9


def profile(t: float) -> float:
    """The opening the hand sweeps, as a function of seconds since it started."""
    return 0.5 - 0.5 * math.cos(2.0 * math.pi * t / DURATION)


class FakeClock:
    """A monotonic clock that moves only when a loop sleeps.

    One cycle of either loop then advances trajectory time by exactly one cycle,
    so a capture and a replay run through in microseconds and land on the same
    samples every run.  That is what lets the checks below be equalities instead
    of tolerances around a scheduler.

    ``real_sleep`` adds a short real sleep per cycle, for the checks that need a
    session to still be running when the next line executes.
    """

    def __init__(self, on_tick=None, real_sleep: float = 0.0) -> None:
        self.t = 0.0
        self._on_tick = on_tick
        self._real_sleep = real_sleep

    def now(self) -> float:
        return self.t

    def sleep(self, seconds: float) -> None:
        self.t += seconds
        if self._on_tick is not None:
            self._on_tick(self.t)
        if self._real_sleep:
            time.sleep(self._real_sleep)

    def install(self, g) -> "FakeClock":
        g.motion_config.monotonic_fn = self.now
        g.motion_config.sleep_fn = self.sleep
        return self


class Hand:
    """Writes the profile onto the fake motor, in step with the fake clock.

    The recorder samples right after its pacing sleep, so a position written
    from inside that sleep is the position sampled at that instant: the capture
    is the profile exactly, rather than a hand that raced the sampler.

    The real hand-teaching path is ``zero_gravity=True`` — the zero-torque frames
    that make the jaws back-drivable.  That is not what moves the motor here,
    because the fake's slack motor springs toward zero; the hand is written to
    the motor directly, and the zero-torque frames are checked on their own.
    """

    def __init__(self, fake, g) -> None:
        self._fake = fake
        self._g = g

    def __call__(self, t: float) -> None:
        openness = profile(t)
        self._fake.motor.pos = openness_to_rad(openness, self._g.config)
        # The fake derives torque and velocity only from the MIT frames it is
        # sent, and a hand-driven capture sends none.  They are written here so
        # the recorded diagnostics are not all zeros: otherwise a player that
        # fed them forward would look exactly like one that does not, and the
        # check meant to catch that would pass either way.
        self._fake.motor.tau = 2.0 * openness
        self._fake.motor.vel = 2.0 - 4.0 * openness

    def park(self, openness: float) -> None:
        """Put the jaws somewhere without recording it."""
        self._fake.motor.pos = openness_to_rad(openness, self._g.config)


# ── reporting ───────────────────────────────────────────────────────────────


class Report:
    """Prints one line per check and counts what failed."""

    def __init__(self) -> None:
        self.passed = 0
        self.failures = 0

    def section(self, title: str) -> None:
        print(f"\n{title}")

    def check(self, name: str, fn) -> None:
        """Run *fn*, which returns a detail string or raises."""
        try:
            detail = fn()
        except Exception as e:  # noqa: BLE001 — the failure is the output here
            self.failures += 1
            print(f"  FAIL  {name}")
            print(f"        {type(e).__name__}: {e}")
            return
        self.passed += 1
        print(f"  ok    {name}" + (f"  [{detail}]" if detail else ""))


def bar(values, width: int = 60) -> str:
    """One ASCII row standing in for a plot of *values*, which are in [0, 1]."""
    ramp = " .:-=+*#%@"
    stride = max(1, len(values) // width)
    return "".join(ramp[min(9, max(0, int(round(v * 9))))]
                   for v in list(values)[::stride][:width])


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def close(a: float, b: float, tol: float = TOL) -> bool:
    return abs(a - b) <= tol


def commanded_openness(fake, g):
    """The opening each streamed frame asked for, in order, minus the hold.

    The final frame is dropped: it is the hold ``play_stop`` sends at the motor's
    *measured* position, which lags the trajectory by however long the servo
    took, and is therefore not a point on the trajectory.
    """
    frames = fake.frames[:-1] if fake.frames else []
    return [rad_to_openness(f.q, g.config) for f in frames]


def replay(g, fake, traj, **kwargs):
    """Replay once and return the frames it sent, minus the trailing hold."""
    fake.frames = []
    g.play(traj, **kwargs)
    return fake.frames[:-1]


# ── checks ──────────────────────────────────────────────────────────────────


def check_capture_is_the_motion(traj) -> str:
    expected = int(DURATION * RATE_HZ)
    require(len(traj) == expected,
            f"expected {expected} samples over {DURATION}s at {RATE_HZ:.0f}Hz, "
            f"got {len(traj)}")
    require(close(traj.samples[0].t, 0.0),
            f"first sample is at t={traj.samples[0].t}, not 0.0")
    worst = max(abs(s.openness - profile(s.t)) for s in traj.samples)
    require(worst < TOL,
            f"the capture drifts from the hand by {worst:.3e} openness — the "
            f"recorder sampled a different instant than the hand moved on")
    return (f"{len(traj)} samples, {traj.duration:.3f}s, worst error against "
            f"the hand {worst:.1e}")


def check_stamps_advance(traj) -> str:
    stamps = [s.t for s in traj.samples]
    require(all(b > a for a, b in zip(stamps, stamps[1:])),
            "timestamps are not strictly increasing")
    gaps = [b - a for a, b in zip(stamps, stamps[1:])]
    require(max(gaps) - min(gaps) < TOL,
            f"sample gaps vary by {max(gaps) - min(gaps):.3e}s")
    return f"{len(stamps)} distinct stamps, every gap {gaps[0]:.4f}s"


def check_zero_gravity_frames(fake, g, hand) -> str:
    """Teach-mode frames are torque-free; the one sent on stop re-engages them.

    Both rules matter and pull in opposite directions: a teaching frame that
    carried gains would hold the jaws against the hand, and a recording that
    ended without one would leave them slack.
    """
    fake.frames = []
    g.record(0.05, rate_hz=RATE_HZ, zero_gravity=True)
    frames = list(fake.frames)
    hand.park(0.0)
    require(len(frames) > 1,
            f"a zero-gravity recording streamed {len(frames)} frames in total")
    hold, teaching = frames[-1], frames[:-1]
    stray = [(f.kp, f.kd) for f in teaching if f.kp != 0.0 or f.kd != 0.0]
    require(not stray,
            f"{len(stray)} of {len(teaching)} teaching frames carried gains "
            f"{stray[0] if stray else ()} — the jaws would hold position "
            f"instead of following the hand")
    require((hold.kp, hold.kd) == (g.config.kp, g.config.kd),
            f"the frame that stopped the recording used gains "
            f"({hold.kp}, {hold.kd}) instead of the configured "
            f"({g.config.kp}, {g.config.kd}) — the jaws would be left slack")
    return (f"{len(teaching)} teaching frames all at (kp, kd) = (0, 0), then "
            f"one hold at ({hold.kp}, {hold.kd})")


def check_file_round_trip(traj, tmp: str) -> str:
    path = traj.save("demo")
    with open(path, "rb") as f:
        blob = f.read()
    want = 66 + 40 * len(traj)
    require(blob[:8] == b"LGRTRJ01", f"magic is {blob[:8]!r}")
    require(len(blob) == want,
            f"{len(blob)}B for {len(traj)} samples, expected {want}B")
    loaded = Trajectory.load(path)
    require(loaded.samples == traj.samples,
            "a sample came back different from the one written")
    require(loaded.mount == traj.mount and loaded.sample_hz == traj.sample_hz,
            f"provenance changed: mount {loaded.mount!r}, "
            f"rate {loaded.sample_hz}")
    require(close(loaded.pos_closed_rad, POS_CLOSED_RAD)
            and close(loaded.pos_open_rad, POS_OPEN_RAD)
            and close(loaded.rad_to_mm, RAD_TO_MM),
            "the calibration the trajectory was recorded against did not "
            "survive the file")
    return (f"{len(blob)}B = 66B header + {len(traj)}x40B samples, reloaded "
            f"into {os.path.basename(path)}")


def check_truncated_file_rejected(tmp: str) -> str:
    """A stream that is not whole samples must not parse into half a path."""
    with open(os.path.join(tmp, "demo.lgt"), "rb") as f:
        good = f.read()
    cases = {"header cut off": good[:40],
             "last sample cut": good[:-17],
             "one stray byte": good + b"\x00"}
    for label, blob in cases.items():
        broken = os.path.join(tmp, "broken.lgt")
        with open(broken, "wb") as f:
            f.write(blob)
        try:
            Trajectory.load(broken)
        except TrajectoryFormatError:
            continue
        raise AssertionError(f"{label}: {len(blob)}B was accepted as a "
                             f"trajectory")
    return f"{len(cases)} malformed files refused, each with a reason"


def check_replay_follows_the_file(g, fake, traj, show: bool = False) -> str:
    """Every frame replays the trajectory's own opening at that instant."""
    frames = replay(g, fake, traj, align=False)
    origin = traj.samples[0].t
    require(len(frames) >= int(traj.duration / INTERVAL),
            f"only {len(frames)} frames for {traj.duration:.3f}s of path at "
            f"{1 / INTERVAL:.0f}Hz")
    worst = 0.0
    for i, frame in enumerate(frames):
        elapsed = min(i * INTERVAL, traj.duration)
        worst = max(worst, abs(rad_to_openness(frame.q, g.config)
                               - traj.openness_at(origin + elapsed)))
    require(worst < TOL,
            f"a frame was off the trajectory by {worst:.3e} openness")
    if show:
        print(f"        recorded  |{bar([s.openness for s in traj.samples])}|")
        print(f"        replayed  |{bar(commanded_openness(fake, g))}|")
    return f"{len(frames)} frames at {1 / INTERVAL:.0f}Hz, worst deviation " \
           f"{worst:.1e}"


def check_speed_scales_the_clock(g, fake, traj) -> str:
    """Speed multiplies elapsed time; a slower loop must not stretch the path."""
    slow = len(replay(g, fake, traj, speed=1.0, align=False))
    fast = len(replay(g, fake, traj, speed=2.0, align=False))
    require(abs(slow - 2 * fast) <= 3,
            f"speed=2.0 sent {fast} frames against {slow} at speed=1.0: a "
            f"ratio of {slow / fast:.2f}, not 2")
    return f"{slow} frames at 1.0x, {fast} at 2.0x"


def check_align_moves_first(g, fake, traj, hand) -> str:
    """``align`` puts the jaws on the opening the path starts from, first."""
    start_rad = openness_to_rad(traj.samples[0].openness, g.config)

    def frames_before_the_path(frames) -> int:
        # Every frame before the path's second point commands the start opening,
        # so the first frame that does not marks where following began.
        return next(i for i, f in enumerate(frames)
                    if not close(f.q, start_rad)) - 1

    hand.park(1.0)                    # jaws left fully open; the path starts closed
    aligned = replay(g, fake, traj, align=True)
    hand.park(1.0)
    plain = replay(g, fake, traj, align=False)

    with_align = frames_before_the_path(aligned)
    without = frames_before_the_path(plain)
    require(without == 0,
            f"align=False still moved the jaws first ({without} frames)")
    require(with_align == int(ALIGN_S / INTERVAL),
            f"align sent {with_align} frames before following, expected "
            f"{int(ALIGN_S / INTERVAL)}")
    return (f"{with_align} frames move to the start, then "
            f"{len(aligned) - with_align} follow the path")


def check_only_position_is_replayed(traj) -> str:
    """Recorded velocity and torque must never come back as feed-forward."""
    peak_torque = max(abs(s.torque_nm) for s in traj.samples)
    peak_speed = max(abs(s.velocity_rad_s) for s in traj.samples)
    require(peak_torque > 0.0 and peak_speed > 0.0,
            "the recording's torque and velocity are all zero, so a player "
            "that fed them forward would look the same as one that does not")

    g, fake = make_gripper()
    FakeClock().install(g)
    fake.motor.pos = openness_to_rad(traj.samples[0].openness, g.config)
    frames = replay(g, fake, traj, align=False)
    fed = [(f.dq, f.tau_ff) for f in frames if f.dq != 0.0 or f.tau_ff != 0.0]
    require(not fed,
            f"{len(fed)} of {len(frames)} frames fed recorded motion forward, "
            f"e.g. (dq_target, tau_feedforward) = {fed[0] if fed else ()} — "
            f"that replays force, which does not carry to a unit with another "
            f"mount or calibration")
    return (f"{len(frames)} frames, all pure position against a recording "
            f"carrying up to {peak_torque:.2f}Nm and {peak_speed:.2f}rad/s")


def check_replays_on_a_reverse_mount(traj) -> str:
    """The portability claim: one path, two mountings, mirrored angles."""
    def run(reverse: bool):
        g, fake = make_gripper(reverse=reverse)
        FakeClock().install(g)
        fake.motor.pos = openness_to_rad(traj.samples[0].openness, g.config)
        return g, replay(g, fake, traj, align=False)

    normal_g, normal = run(reverse=False)
    reverse_g, reverse = run(reverse=True)
    require(len(normal) == len(reverse),
            f"{len(normal)} frames on the normal mount against {len(reverse)} "
            f"on the reverse one")

    # Same opening on every frame: the recorded channel is not a raw angle.
    worst = max(abs(rad_to_openness(a.q, normal_g.config)
                    - rad_to_openness(b.q, reverse_g.config))
                for a, b in zip(normal, reverse))
    require(worst < TOL,
            f"the two mountings replayed openings differing by {worst:.3e}")

    # Mirrored angles: the two are reflections about the travel's midpoint.
    midpoint = (POS_CLOSED_RAD + POS_OPEN_RAD) / 2.0
    mirror = max(abs((a.q + b.q) / 2.0 - midpoint)
                 for a, b in zip(normal, reverse))
    require(mirror < 1e-6,
            f"the angles are off by {mirror:.3e} rad from a reflection about "
            f"{midpoint:.4f}")

    # ... and the two run in opposite directions.
    require((normal[1].q - normal[0].q) * (reverse[1].q - reverse[0].q) < 0.0,
            "both mountings moved the same way on the first step")
    return (f"{len(normal)} frames, openings within {worst:.1e}, angles "
            f"mirrored about {midpoint:.4f} rad "
            f"({min(f.q for f in normal):+.3f}..{max(f.q for f in normal):+.3f} "
            f"vs {min(f.q for f in reverse):+.3f}.."
            f"{max(f.q for f in reverse):+.3f})")


def check_sessions_are_exclusive(g, fake, traj) -> str:
    """One long-running session at a time, and stopping always frees the slot."""
    FakeClock(real_sleep=0.002).install(g)
    g.record_start(rate_hz=RATE_HZ, zero_gravity=False)
    try:
        status = g.trajectory_status()
        require(status["kind"] == "record",
                f"status says {status!r} while a recording is running")
        for label, start in (
            ("a second recording", lambda: g.record_start(rate_hz=RATE_HZ)),
            ("a replay", lambda: g.play_start(traj)),
            ("teleoperation",
             lambda: g.teleop_start("slave",
                                    transport=InProcTeleopTransport())),
        ):
            try:
                start()
            except (TrajectoryBusyError, TeleopBusyError) as e:
                require("record" in str(e),
                        f"{label} was refused, but the reason ({e}) does not "
                        f"say what is in the way")
            else:
                raise AssertionError(f"{label} started while one was running")
    finally:
        g.record_stop(allow_empty=True)
        FakeClock().install(g)      # hand the fast clock back to the checks

    require(g.trajectory_status()["active"] is False,
            "the slot is still claimed after record_stop returned")
    g.record_start(rate_hz=RATE_HZ, zero_gravity=False)
    g.record_stop(allow_empty=True)
    return "record, replay and teleop refused each other; the slot frees on stop"


def check_refusals_leave_the_slot_free(g, traj) -> str:
    """The cases that must be refused are refused, without claiming the slot."""
    refused = []
    g.config.calibrated = False
    try:
        try:
            g.record_start(rate_hz=RATE_HZ)
        except TrajectoryError:
            refused.append("uncalibrated recording")
        else:
            raise AssertionError(
                "an uncalibrated unit started recording — the opening it would "
                "store is a guess")
    finally:
        g.config.calibrated = True

    try:
        g.play(Trajectory())
    except TrajectoryEmptyError:
        refused.append("empty trajectory")
    else:
        raise AssertionError("an empty trajectory was accepted for replay")

    require(g.trajectory_status()["active"] is False,
            "a refused call left the session claimed, so nothing can start "
            "again")
    return f"{len(refused)} refused ({', '.join(refused)}), slot still free"


def main() -> int:
    report = Report()
    with tempfile.TemporaryDirectory(prefix="litegrip-check-") as tmp:
        os.environ["LITEGRIP_TRAJ_DIR"] = tmp
        g, fake = make_gripper()
        hand = Hand(fake, g)
        FakeClock(on_tick=hand).install(g)
        hand(0.0)                    # the hand starts where the path does

        print("Trajectory record and replay — end-to-end check, no hardware")
        print(f"  fake CAN on vcan0: {RATE_HZ:.0f}Hz capture, "
              f"{1 / INTERVAL:.0f}Hz replay, {DURATION:.0f}s of taught motion")

        report.section("1. teach a path by hand")
        traj = g.record(DURATION, rate_hz=RATE_HZ, zero_gravity=False)
        print(f"        hand      |{bar([s.openness for s in traj.samples])}|")
        report.check("the capture is the motion the hand made",
                     lambda: check_capture_is_the_motion(traj))
        report.check("every timestamp advances",
                     lambda: check_stamps_advance(traj))
        report.check("zero-gravity streams zero-torque frames",
                     lambda: check_zero_gravity_frames(fake, g, hand))

        report.section("2. save and load")
        report.check("a save/load round trip preserves the capture",
                     lambda: check_file_round_trip(traj, tmp))
        report.check("a file that is not whole samples is rejected",
                     lambda: check_truncated_file_rejected(tmp))
        loaded = Trajectory.load(os.path.join(tmp, "demo.lgt"))

        report.section("3. replay on the unit that recorded it")
        report.check("every frame is the trajectory at that instant",
                     lambda: check_replay_follows_the_file(g, fake, traj,
                                                           show=True))
        report.check("speed scales elapsed time, not the frame count",
                     lambda: check_speed_scales_the_clock(g, fake, traj))
        report.check("align moves to the start before following",
                     lambda: check_align_moves_first(g, fake, traj, hand))
        report.check("only position is replayed, never force",
                     lambda: check_only_position_is_replayed(traj))

        report.section("4. replay on a reverse-mounted unit")
        report.check("the same path plays, mirrored in radians",
                     lambda: check_replays_on_a_reverse_mount(traj))

        report.section("5. sessions and refusals")
        report.check("record, replay and teleop are mutually exclusive",
                     lambda: check_sessions_are_exclusive(g, fake, traj))
        report.check("bad input is refused and leaves the slot free",
                     lambda: check_refusals_leave_the_slot_free(g, traj))
        report.check("the loaded file replays like the recording",
                     lambda: check_replay_follows_the_file(g, fake, loaded))

    total = report.passed + report.failures
    print(f"\n{total} checks, {report.failures} failed")
    return 1 if report.failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
