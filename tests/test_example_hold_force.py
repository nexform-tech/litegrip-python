"""Hardware-free tests for ``examples/hold_force_test.py``.

The example is a script, but the parts that carry real behavior are the trace it
collects, the verdict it draws from that trace, and the argv validation before
any hardware is touched.  All three are exercised here against the fake CAN the
other tests use, so no gripper and no bus are required.

The centerpiece is the regression the script exists to catch: on a workpiece that
yields under load, a hold that carries a position gain loses force as the jaws
follow the object.  The SDK's hold is a bare torque feed-forward now, ramped up
to the setpoint over its first frames (``MotionConfig.force_ramp_n_s``), so the
measured force must climb to the setpoint and stay there for the rest of the
hold — and ``_run`` must say so, not just the engine.
"""

from __future__ import annotations

import contextlib
import importlib.util
import io
import re
import unittest
from pathlib import Path
from types import SimpleNamespace

import _sdkpath  # noqa: F401
from litegrip import MotionConfig
from litegrip.actions import MoveProgress

from fake_can import POS_CLOSED_RAD, POS_OPEN_RAD, make_gripper, tick_clock

_EXAMPLE = (Path(__file__).resolve().parent.parent
            / "examples" / "hold_force_test.py")
_spec = importlib.util.spec_from_file_location("example_hold_force", _EXAMPLE)
example = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(example)

#: Mid-travel: an object the close has room to ramp into, which is what lets the
#: stall detector arm before it arrives.
OBJECT_RAD = POS_OPEN_RAD + 0.5 * (POS_CLOSED_RAD - POS_OPEN_RAD)


def _wired(*, block_rad=None, yield_rad_s=0.0, yield_tau_nm=0.0):
    """A fake gripper ``_run`` can drive end to end without a bus.

    The fake arrives connected and enabled, so ``connect``/``enable`` are stubbed
    to what ``_run`` reads back off them, and calibration loading is short
    circuited — those paths have their own tests.  ``stops=True`` gives the jaws
    the mechanical limits a real gripper has, which is what makes the
    precondition ``open()`` end in a stall instead of running on forever.

    The fake clock ticks 0.1 s per call, so the hold's slice count follows from
    the loop's own deadline arithmetic rather than from wall time; the tests read
    that count back off the script instead of hardcoding it.
    """
    g, fake = make_gripper(stops=True, block_rad=block_rad,
                           yield_rad_s=yield_rad_s, yield_tau_nm=yield_tau_nm)
    g.motion_config = MotionConfig(sleep_fn=lambda _: None,
                                   monotonic_fn=tick_clock(0.1))
    g.load_calibration = lambda **kwargs: True
    g.connect = lambda: True
    g.enable = lambda: SimpleNamespace(ok=True, tries=1, state=None)
    g.disconnect = lambda: None
    return g, fake


def _run_main(argv, gripper):
    """Run ``main`` against ``gripper``; return ``(exit_code, stdout+stderr)``.

    ``_pause`` waits for a human on a terminal, and the tests run without one —
    stubbing it to ``True`` is the operator pressing Enter.  Everything else is
    the script's own code path.
    """
    original_gripper, original_pause = example.LiteGrip, example._pause
    example.LiteGrip = lambda **kwargs: gripper
    example._pause = lambda *args, **kwargs: True
    out = io.StringIO()
    try:
        with contextlib.redirect_stdout(out), \
                contextlib.redirect_stderr(out):
            code = example.main(argv)
    finally:
        example.LiteGrip = original_gripper
        example._pause = original_pause
    return code, out.getvalue()


class RunOnAYieldingObject(unittest.TestCase):
    """The reported case: a workpiece that gives under the grip."""

    ARGV = ["--force", "20", "--hold-s", "1.0"]

    def _run(self, **workpiece):
        gripper, fake = _wired(**workpiece)
        code, text = _run_main(self.ARGV, gripper)
        return code, text, gripper, fake

    def test_the_force_does_not_drop_and_the_script_says_so(self):
        code, text, gripper, fake = self._run(
            block_rad=OBJECT_RAD, yield_rad_s=0.05, yield_tau_nm=0.5)

        self.assertEqual(code, 0, text)
        self.assertIn("OK: no drop in the hold", text)
        self.assertNotIn("FAIL", text)
        # The workpiece really did move under the load; otherwise the case would
        # prove nothing about a hold whose force depends on jaw position.
        self.assertGreater(fake.motor.block_rad - OBJECT_RAD, 0.01)

    def test_every_hold_frame_carries_the_force_and_no_gain(self):
        _code, _text, _gripper, fake = self._run(
            block_rad=OBJECT_RAD, yield_rad_s=0.05, yield_tau_nm=0.5)

        hold = [f for f in fake.frames if f.tau_ff == 2.0]
        self.assertTrue(hold, "no hold frame carried the 20 N feed-forward")
        for frame in hold:
            self.assertEqual(frame.kp, 0.0)
            self.assertEqual(frame.kd, 0.0)

    def test_the_summary_says_where_the_ramp_let_go(self):
        _code, text, _gripper, _fake = self._run(
            block_rad=OBJECT_RAD, yield_rad_s=0.05, yield_tau_nm=0.5)

        # The ramp's end and the hold's start are one handover apart: a ramp
        # that stopped short of the object hands the hold a gap to cross at
        # force, which is what a lurch at the object looks like from here.  The
        # grasp result cannot say it — it only carries the state left behind at
        # the very end — so the script has to record the move phase itself.
        match = re.search(
            r"the ramp ended at (\S+) mm after (\d+)/(\d+) frames; "
            r"the hold started from (\S+) mm and ended at (\S+) mm", text)
        self.assertIsNotNone(match, text)
        ramp_mm, hold_start_mm = (float(match.group(1)), float(match.group(4)))
        self.assertGreater(int(match.group(3)), 0, text)
        self.assertLessEqual(abs(ramp_mm - hold_start_mm), 2.0, text)

    def test_the_trace_prints_the_measured_force_next_to_the_setpoint(self):
        _code, text, _gripper, _fake = self._run(
            block_rad=OBJECT_RAD, yield_rad_s=0.05, yield_tau_nm=0.5)

        # One line per hold slice, each printing the measured force, the setpoint,
        # and how much of it arrived.  The slice count comes from the script's own
        # summary rather than a constant, so the assertion is "every slice the
        # engine reported was printed", not "the fake clock ticked n times".
        #
        # Every printed row counts, not just the rows at 100% of the setpoint.
        # The hold arrives at its setpoint by ramping there
        # (``MotionConfig.force_ramp_n_s``), so the slices the climb is still
        # crossing print the force it has reached so far — 8.5 N, 12.5 N, 16.5 N
        # on the way to 20 N.  Counting only the rows that read 100% would be
        # asserting the hold lands in one frame, which is not what it does now.
        self.assertIn("measured   / set   % of set", text)
        slices = re.search(r"hold: (\d+) slices?", text)
        self.assertIsNotNone(slices, text)
        self.assertGreater(int(slices.group(1)), 0, text)
        row = re.compile(r"^\s+\d+\.\d+\s+[\d.]+ N\s+[\d.]+\s+[\d.]+%\s+"
                         r"[-\d.]+\s+\|")
        rows = [line for line in text.splitlines() if row.match(line)]
        self.assertEqual(len(rows), int(slices.group(1)), text)


class RunAgainstSomethingThatDoesNotGive(unittest.TestCase):
    """A rigid block cannot show the defect, and the script must not pretend it did."""

    def test_a_rigid_block_passes_but_is_called_weak_evidence(self):
        gripper, _fake = _wired(block_rad=OBJECT_RAD)
        code, text = _run_main(["--force", "20", "--hold-s", "1.0"], gripper)

        self.assertEqual(code, 0, text)
        self.assertIn("the jaws moved only 0.00 mm during the hold", text)
        self.assertIn("weak evidence", text)

    def test_nothing_between_the_jaws_is_reported_as_not_testing_anything(self):
        gripper, _fake = _wired()
        code, text = _run_main(["--force", "20", "--hold-s", "1.0"], gripper)

        self.assertNotEqual(code, 0)
        self.assertIn("no object in the jaws", text)


class Verdict(unittest.TestCase):
    """``analyse`` on a trace, with the engine taken out of the picture."""

    @staticmethod
    def _trace(forces_n, drift_rad=0.01):
        """A HoldTrace fed synthetic slices declining from ``forces_n``."""
        trace = example.HoldTrace(0.2)
        for index, force in enumerate(forces_n):
            trace(MoveProgress(
                phase="hold", i=index + 1, total_steps=0,
                cmd_rad=0.0, pos_rad=drift_rad * index, delta_rad=0.0,
                win_delta_rad=None,
                torque_nm=force * example.N_TO_NM))
        return trace

    @staticmethod
    def _result(cycles, *, reached=False, stalled=True, ok=True, err=1):
        return SimpleNamespace(
            cycles=cycles, ok=ok, reached=reached, stalled=stalled,
            state=SimpleNamespace(error_code=err, position_mm=0.0,
                                  position_rad=0.0),
            target_rad=0.0, force_n=20.0)

    def _judge(self, forces, gripper):
        trace = self._trace(forces)
        result = self._result(len(forces), reached=not forces)
        problems, _notes = example.analyse(trace, gripper.config, 20.0, result,
                                           example.SAG_RATIO)
        return problems

    def test_a_decaying_hold_is_reported_as_a_sag(self):
        gripper, _fake = _wired()
        problems = self._judge([20.0, 19.0, 12.0, 5.0, 5.0, 5.0], gripper)

        self.assertTrue(any("sagged" in p for p in problems), problems)
        self.assertTrue(any("kp = kd = 0" in p for p in problems), problems)

    def test_a_hold_that_never_arrived_is_reported_as_such(self):
        gripper, _fake = _wired()
        problems = self._judge([0.0, 0.0, 0.0, 0.0], gripper)

        self.assertTrue(any("never reached half" in p for p in problems),
                        problems)

    def test_a_flat_hold_has_no_problems(self):
        gripper, _fake = _wired()
        self.assertEqual(self._judge([20.0] * 6, gripper), [])

    def test_a_hold_that_ended_in_a_fault_is_a_problem(self):
        gripper, _fake = _wired()
        trace = self._trace([20.0] * 4)
        problems, _notes = example.analyse(
            trace, gripper.config, 20.0,
            self._result(4, ok=False, err=8), example.SAG_RATIO)
        self.assertTrue(any("ended abnormally" in p for p in problems), problems)

    def test_no_slices_at_all_is_a_problem_not_an_empty_pass(self):
        gripper, _fake = _wired()
        problems, _notes = example.analyse(
            example.HoldTrace(0.2), gripper.config, 20.0,
            self._result(0), example.SAG_RATIO)
        self.assertTrue(any("never ran a slice" in p for p in problems),
                        problems)


class ArgvValidationTest(unittest.TestCase):
    """The checks that run before a gripper is constructed — no hardware."""

    def test_hold_s_must_be_positive(self):
        gripper, _fake = _wired()
        self.assertEqual(_run_main(["--hold-s", "0"], gripper)[0], 2)

    def test_sag_ratio_must_be_a_fraction(self):
        gripper, _fake = _wired()
        self.assertEqual(_run_main(["--sag-ratio", "2"], gripper)[0], 2)

    def test_force_must_be_positive(self):
        gripper, _fake = _wired()
        self.assertEqual(_run_main(["--force", "0"], gripper)[0], 2)

    def test_a_missing_calibration_file_is_refused(self):
        gripper, _fake = _wired()
        self.assertEqual(_run_main(["--calib", "/nope.json"], gripper)[0], 2)


if __name__ == "__main__":
    unittest.main()
