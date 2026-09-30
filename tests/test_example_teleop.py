"""Hardware-free tests for ``examples/teleop.py``.

The example is a script, but two of its parts carry real behavior: the argv
validation that happens before any hardware is touched, and ``_FakeLeader``, the
synthetic leader that lets a single gripper be benched.  Both are exercised here
against the same fake CAN the teleop tests use, so no gripper, CAN interface or
network is required.

The integration test drives the follower from ``_FakeLeader`` exactly the way the
script wires them — a shared ``InProcTeleopTransport`` — and asserts the torque
guard trips and then re-arms on the sweep back up.
"""

from __future__ import annotations

import contextlib
import importlib.util
import io
import time
import unittest
from pathlib import Path

import _sdkpath  # noqa: F401
from litegrip import (DEFAULT_ALIGN_SPEED_MM_S, DEFAULT_LEAD_CAP_MM,
                      DEFAULT_READY_TIMEOUT_S, GripperTeleop,
                      InProcTeleopTransport, decode_frame, teleop_topic)

from fake_can import POS_CLOSED_RAD, POS_OPEN_RAD, make_gripper

_EXAMPLE = Path(__file__).resolve().parent.parent / "examples" / "teleop.py"
_spec = importlib.util.spec_from_file_location("example_teleop", _EXAMPLE)
example = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(example)

TOPIC = teleop_topic("gripA")


def _nop_sleep(_seconds: float) -> None:
    """Stub the follower loop's pacing so tests do not pay the frame interval."""


def _wait_until(predicate, timeout_s: float = 5.0) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.001)
    return predicate()


def _run_main(argv):
    """Run ``main`` with its chatter suppressed; return the exit code."""
    with contextlib.redirect_stdout(io.StringIO()), \
            contextlib.redirect_stderr(io.StringIO()):
        return example.main(argv)


class ArgvValidationTest(unittest.TestCase):
    """The checks that run before a gripper is constructed — no hardware."""

    def test_fake_leader_requires_slave_mode(self):
        # A synthetic leader replaces the remote end, so there must be a local
        # follower for it to drive.
        self.assertEqual(_run_main(["--mode", "master", "--fake-leader"]), 2)

    def test_openness_rate_must_be_positive(self):
        self.assertEqual(
            _run_main(["--mode", "slave", "--fake-leader", "--openness-rate", "0"]),
            2)

    def test_torque_limit_must_not_be_negative(self):
        # 0 is legal (guard off); only a negative limit is nonsense.
        self.assertEqual(_run_main(["--mode", "slave", "--torque-limit", "-1"]), 2)

    def test_ready_timeout_must_not_be_negative(self):
        # 0 is legal (wait indefinitely); only a negative timeout is nonsense.
        self.assertEqual(_run_main(["--mode", "master", "--ready-timeout", "-1"]), 2)

    def test_align_speed_must_be_positive(self):
        # A zero-speed align never arrives; there is no "off" for it, only
        # --no-align.
        self.assertEqual(_run_main(["--mode", "slave", "--align-speed", "0"]), 2)

    def test_lead_cap_must_not_be_negative(self):
        # 0 is legal (cap off); only a negative cap is nonsense.
        self.assertEqual(_run_main(["--mode", "slave", "--lead-cap", "-1"]), 2)

    def test_defaults_match_the_sdk(self):
        args = example.build_parser().parse_args(["--mode", "slave"])
        self.assertFalse(args.fake_leader)
        self.assertEqual(args.torque_limit, 0.0)   # guard off by default
        self.assertEqual(args.openness_rate, 0.3)
        self.assertEqual(args.align_speed, DEFAULT_ALIGN_SPEED_MM_S)
        self.assertEqual(args.lead_cap, DEFAULT_LEAD_CAP_MM)
        # The readiness gate is on by default: the leader holds under gain for
        # the follower rather than going hand-movable at once.
        self.assertFalse(args.no_require_ready)
        self.assertEqual(args.ready_timeout, DEFAULT_READY_TIMEOUT_S)


class FakeLeaderTest(unittest.TestCase):
    def test_sweep_is_a_bounded_triangle(self):
        # Drive _publish by hand so the sequence is exact rather than timed.
        bus = InProcTeleopTransport()
        sub = bus.sub(TOPIC)
        leader = example._FakeLeader(bus, TOPIC, travel_mm=120.0, rate_hz=10.0,
                                     openness_rate=0.1)   # step = 0.01
        seen = []
        last = None
        for _ in range(103):
            leader._publish()
            last = sub.try_recv()
            seen.append(decode_frame(last)[0])
        self.assertAlmostEqual(seen[0], 1.0)              # starts where it is
        self.assertAlmostEqual(seen[100], 0.0)            # hits the closed stop
        self.assertAlmostEqual(seen[101], 0.01)           # and turns around
        self.assertTrue(all(0.0 <= o <= 1.0 for o in seen))
        # Diagnostic fields: no sensor on a synthetic leader, so force is 0 and
        # position is the opening scaled by the travel it was handed.
        _, position_mm, force_n, _ = decode_frame(last)
        self.assertEqual(force_n, 0.0)
        self.assertAlmostEqual(position_mm, seen[-1] * 120.0)

    def test_drives_the_follower_guard_through_trip_and_rearm(self):
        block = (POS_OPEN_RAD + POS_CLOSED_RAD) / 2.0
        g, fake = make_gripper(start_rad=POS_OPEN_RAD, block_rad=block)
        bus = InProcTeleopTransport()
        # align=True, as the script defaults to.  The sweep is slow enough that
        # following a single step never crosses the limit — the trip must come
        # from the press against the block, so the guard latches well clear of
        # full open and the sweep back up has room to re-arm it.
        mgr = GripperTeleop(g, bus, "slave", TOPIC, rate_hz=200.0,
                            sleep_fn=_nop_sleep, align=True, watchdog_s=5.0,
                            torque_limit_nm=1.0)
        mgr.start()
        leader = example._FakeLeader(bus, TOPIC, travel_mm=120.0, rate_hz=200.0,
                                     openness_rate=0.4)
        leader.start()
        try:
            self.assertTrue(_wait_until(lambda: mgr.status()["over_torque"], 20.0),
                            "the sweep into the block should trip the guard")
            self.assertLessEqual(mgr.status()["openness"], 0.9,
                                 "the trip should come from the block, not a "
                                 "start-up transient near full open")
            # Released in place: zero stiffness, zero damping.
            self.assertTrue(_wait_until(
                lambda: fake.frames and fake.frames[-1].kp == 0.0
                and fake.frames[-1].kd == 0.0))
            # The sweep turns around at the closed stop, so the follower must
            # re-arm once the leader has reopened past the margin.
            self.assertTrue(_wait_until(lambda: not mgr.status()["over_torque"], 20.0),
                            "reopening the leader should re-arm the guard")
        finally:
            leader.stop()
            mgr.stop()


if __name__ == "__main__":
    unittest.main()
