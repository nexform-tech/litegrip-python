"""Hardware-free tests for gripper trajectory record and replay.

The loops run for real — a thread each — against the kinematic fake CAN in
``tests/fake_can.py``.  Time comes from the module's timing seams, so two clocks
appear here:

* :func:`_use_clock` — a fake clock that advances only when the loop sleeps.
  One cycle is then exactly one step of trajectory time, so a playback finishes
  in microseconds and lands on the same samples every run.  This is what makes
  "how much path did it cover" an exact assertion instead of a timing guess.
* :func:`_pace` — the opposite: the loop parks in a short real sleep, so a
  session is still running when the test looks at it.  Anything that asserts
  "it is running *now*" needs this one; with the fast clock the loop is often
  finished before the next line executes.

The waiting side (``wait_for``, :func:`_wait_until`) always uses the real clock,
so a loop that never advances fails a test instead of hanging it.

The zero-gravity recorder is the one place a loop sends frames of its own, so
several tests read them back out of ``fake.frames``.  That is how the
must-send-a-zero-torque-frame-every-cycle rule and the hold-on-stop rule are
checked rather than asserted in a comment.
"""

from __future__ import annotations

import os
import struct
import tempfile
import threading
import time
import unittest

import _sdkpath  # noqa: F401
from litegrip import (InProcTeleopTransport, NotInitializedError, TeleopBusyError,
                      Trajectory, TrajectoryBusyError, TrajectoryEmptyError,
                      TrajectoryError, TrajectoryFormatError,
                      TrajectoryNotActiveError, TrajectoryPlayer,
                      TrajectoryRecorder, TrajectoryRecordingError,
                      TrajectorySample, resolve_path, trajectory_dir)
from litegrip.teleop import openness_to_rad, rad_to_openness

from fake_can import POS_CLOSED_RAD, POS_OPEN_RAD, RAD_TO_MM, make_gripper

# Far longer than a stubbed loop needs.  A timeout here means the loop is not
# running at all, not that the machine is slow.
WAIT_S = 2.0


def _wait_until(predicate, timeout_s: float = WAIT_S) -> bool:
    """Poll on the real clock until *predicate* holds, or give up."""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.001)
    return predicate()


def _use_clock(g, step: float = 0.005):
    """Point the loop's timing seams at a clock that moves only when it sleeps.

    Deterministic: a cycle that sleeps once advances the trajectory by exactly
    *step*, whatever the loop's own frame interval is.  A *step* larger than
    that interval simulates a cycle which overran.
    """
    now = [0.0]

    def monotonic() -> float:
        return now[0]

    def sleep(_seconds: float) -> None:
        now[0] += step

    g.motion_config.monotonic_fn = monotonic
    g.motion_config.sleep_fn = sleep
    return monotonic, sleep


def _frozen_clock(g) -> None:
    """A clock that never moves and a sleep that costs nothing."""
    g.motion_config.monotonic_fn = lambda: 5.0
    g.motion_config.sleep_fn = lambda _seconds: None


def _pace(g, step: float = 0.01, real_sleep: float = 0.002) -> None:
    """A clock that advances by *step* per cycle, paced by a real *real_sleep*.

    The loop keeps running — and keeps sampling — for as long as the test needs
    to observe it, without the fast clock's race and without a hot spin.
    """
    now = [0.0]

    def monotonic() -> float:
        return now[0]

    def sleep(_seconds: float) -> None:
        now[0] += step
        time.sleep(real_sleep)

    g.motion_config.monotonic_fn = monotonic
    g.motion_config.sleep_fn = sleep


def _trajectory(opennesses, dt: float = 0.01, mount: str = "normal",
                reverse: bool = False, **kwargs) -> Trajectory:
    """A trajectory from a list of openness values, one every *dt* seconds."""
    kwargs.setdefault("sample_hz", 1.0 / dt)
    kwargs.setdefault("mount", mount)
    kwargs.setdefault("rad_to_mm", RAD_TO_MM)
    if reverse:
        kwargs.setdefault("pos_closed_rad", POS_OPEN_RAD)
        kwargs.setdefault("pos_open_rad", POS_CLOSED_RAD)
    else:
        kwargs.setdefault("pos_closed_rad", POS_CLOSED_RAD)
        kwargs.setdefault("pos_open_rad", POS_OPEN_RAD)
    return Trajectory(
        samples=[TrajectorySample(t=i * dt, openness=float(o), position_rad=0.0)
                 for i, o in enumerate(opennesses)],
        **kwargs)


def _commanded(fake, g):
    """The openness each replayed frame asked for, in order.

    The final frame is dropped: it is the hold ``play_stop`` sends at the
    motor's *measured* position, which lags the trajectory by however long the
    servo took — not a point on the trajectory.
    """
    frames = fake.frames[:-1] if fake.frames else []
    return [rad_to_openness(f.q, g.config) for f in frames]


# ═══════════════════════════════════════════════════════════════════════════
# File format
# ═══════════════════════════════════════════════════════════════════════════


class FormatTest(unittest.TestCase):
    def test_layout_is_fixed(self):
        """Header and sample sizes *are* the format; changing them breaks files."""
        self.assertEqual(struct.calcsize("<8sHI5dI8s"), 66)
        self.assertEqual(struct.calcsize("<5d"), 40)
        self.assertEqual(len(_trajectory([0.0, 1.0]).to_bytes()), 66 + 2 * 40)

    def test_round_trip_preserves_everything(self):
        original = _trajectory([0.0, 0.25, 1.0], mount="reverse", reverse=True)
        original.samples[1] = TrajectorySample(
            t=0.01, openness=0.25, position_rad=-0.4, velocity_rad_s=1.5,
            torque_nm=-0.75)
        loaded = Trajectory.from_bytes(original.to_bytes())
        self.assertEqual(loaded.samples, original.samples)
        self.assertEqual(loaded.mount, "reverse")
        self.assertEqual(loaded.can_id, original.can_id)
        self.assertAlmostEqual(loaded.sample_hz, original.sample_hz)
        self.assertAlmostEqual(loaded.rad_to_mm, RAD_TO_MM)
        self.assertAlmostEqual(loaded.pos_closed_rad, POS_OPEN_RAD)
        self.assertAlmostEqual(loaded.pos_open_rad, POS_CLOSED_RAD)
        self.assertAlmostEqual(loaded.created, original.created)

    def test_mount_survives_both_mountings_and_none(self):
        for mount in ("normal", "reverse", None):
            with self.subTest(mount=mount):
                traj = _trajectory([0.5], mount=mount)
                self.assertEqual(
                    Trajectory.from_bytes(traj.to_bytes()).mount, mount)

    def _pack(self, **overrides) -> bytes:
        """A valid two-sample blob, with individual header fields overridden."""
        template = _trajectory([0.0, 1.0], mount="normal")
        values = dict(
            magic=b"LGRTRJ01", version=1, n=2, sample_hz=100.0,
            created=template.created, pos_closed_rad=POS_CLOSED_RAD,
            pos_open_rad=POS_OPEN_RAD, rad_to_mm=RAD_TO_MM, can_id=0x08,
            mount=b"normal\x00\x00")
        values.update(overrides)
        header = struct.pack(
            "<8sHI5dI8s", values["magic"], values["version"], values["n"],
            values["sample_hz"], values["created"], values["pos_closed_rad"],
            values["pos_open_rad"], values["rad_to_mm"], values["can_id"],
            values["mount"])
        body = b"".join(struct.pack("<5d", i * 0.01, float(i), 0.0, 0.0, 0.0)
                        for i in range(values["n"]))
        return header + body

    def test_rejects_bad_magic(self):
        with self.assertRaises(TrajectoryFormatError) as caught:
            Trajectory.from_bytes(self._pack(magic=b"NOPE\x00\x00\x00\x00"))
        self.assertIn("magic", str(caught.exception))

    def test_rejects_unknown_version(self):
        with self.assertRaises(TrajectoryFormatError) as caught:
            Trajectory.from_bytes(self._pack(version=99))
        self.assertIn("99", str(caught.exception))

    def test_rejects_truncated_payload(self):
        """Length must match the header's count exactly, not approximately."""
        with self.assertRaises(TrajectoryFormatError) as caught:
            Trajectory.from_bytes(self._pack()[:-3])
        self.assertIn("截断", str(caught.exception))

    def test_rejects_trailing_bytes(self):
        """Another file's tail appended is as wrong as a truncation."""
        with self.assertRaises(TrajectoryFormatError):
            Trajectory.from_bytes(self._pack() + b"\x00" * 40)

    def test_rejects_a_header_that_is_too_short_to_be_one(self):
        with self.assertRaises(TrajectoryFormatError):
            Trajectory.from_bytes(b"LGRTRJ01\x01\x00\x02\x00")

    def test_rejects_bad_header_fields(self):
        for field, value in (("sample_hz", 0.0), ("sample_hz", -5.0),
                             ("rad_to_mm", 0.0),
                             ("pos_closed_rad", POS_OPEN_RAD),
                             ("mount", b"sideways")):
            with self.subTest(field=field, value=value):
                with self.assertRaises(TrajectoryFormatError):
                    Trajectory.from_bytes(self._pack(**{field: value}))

    def test_rejects_corrupt_samples(self):
        header = self._pack()[:66]
        cases = {
            "nan": struct.pack("<5d", 0.0, float("nan"), 0.0, 0.0, 0.0),
            "inf": struct.pack("<5d", 0.0, 0.5, float("inf"), 0.0, 0.0),
            "openness above 1": struct.pack("<5d", 0.0, 1.5, 0.0, 0.0, 0.0),
            "openness below 0": struct.pack("<5d", 0.0, -0.1, 0.0, 0.0, 0.0),
        }
        for label, first in cases.items():
            with self.subTest(case=label):
                second = struct.pack("<5d", 0.01, 0.5, 0.0, 0.0, 0.0)
                with self.assertRaises(TrajectoryFormatError):
                    Trajectory.from_bytes(header + first + second)

    def test_rejects_time_going_backwards(self):
        header = self._pack()[:66]
        body = (struct.pack("<5d", 0.5, 0.0, 0.0, 0.0, 0.0)
                + struct.pack("<5d", 0.1, 1.0, 0.0, 0.0, 0.0))
        with self.assertRaises(TrajectoryFormatError) as caught:
            Trajectory.from_bytes(header + body)
        self.assertIn("单调", str(caught.exception))


class InterpolationTest(unittest.TestCase):
    def test_interpolates_between_samples(self):
        traj = _trajectory([0.0, 1.0], dt=1.0)
        self.assertAlmostEqual(traj.openness_at(0.25), 0.25)
        self.assertAlmostEqual(traj.openness_at(0.75), 0.75)
        self.assertAlmostEqual(traj.duration, 1.0)
        self.assertEqual(len(traj), 2)

    def test_clamps_outside_the_recorded_span(self):
        traj = _trajectory([0.0, 1.0], dt=1.0)
        self.assertAlmostEqual(traj.openness_at(-3.0), 0.0)
        self.assertAlmostEqual(traj.openness_at(9.0), 1.0)

    def test_interpolates_across_uneven_sampling(self):
        traj = Trajectory(samples=[
            TrajectorySample(t=0.0, openness=0.0, position_rad=0.0),
            TrajectorySample(t=0.1, openness=0.2, position_rad=0.0),
            TrajectorySample(t=0.4, openness=1.0, position_rad=0.0),
        ])
        self.assertAlmostEqual(traj.openness_at(0.05), 0.1)
        self.assertAlmostEqual(traj.openness_at(0.25), 0.6)
        self.assertAlmostEqual(traj.openness_at(0.1), 0.2)

    def test_a_single_sample_has_no_duration(self):
        self.assertEqual(Trajectory().duration, 0.0)
        self.assertEqual(len(Trajectory()), 0)
        self.assertEqual(_trajectory([0.7]).duration, 0.0)

    def test_duration_is_the_span_not_the_last_timestamp(self):
        """A trajectory shifted off zero lasts as long as its own samples cover."""
        traj = Trajectory(samples=[
            TrajectorySample(t=5.0, openness=0.0, position_rad=0.0),
            TrajectorySample(t=6.0, openness=1.0, position_rad=0.0),
        ])
        self.assertAlmostEqual(traj.duration, 1.0)

    def test_openness_at_on_an_empty_trajectory_raises(self):
        with self.assertRaises(TrajectoryEmptyError):
            Trajectory().openness_at(0.0)


class PathTest(unittest.TestCase):
    """The name-to-path rule and the file it produces."""

    def setUp(self):
        self._env = os.environ.get("LITEGRIP_TRAJ_DIR")
        self._home = os.environ.get("HOME")
        self._root = tempfile.mkdtemp(prefix="litegrip-traj-")
        self.addCleanup(self._restore)

    def _restore(self):
        for key, value in (("LITEGRIP_TRAJ_DIR", self._env),
                           ("HOME", self._home)):
            if value is None:
                os.environ.pop(key, None)
            else:
                os.environ[key] = value

    def test_default_directory_is_next_to_the_calibrations(self):
        os.environ.pop("LITEGRIP_TRAJ_DIR", None)
        os.environ["HOME"] = "/home/someone"
        self.assertEqual(trajectory_dir(), "/home/someone/.litegrip/trajectories")

    def test_env_override_wins(self):
        os.environ["LITEGRIP_TRAJ_DIR"] = "/tmp/trajectories"
        self.assertEqual(trajectory_dir(), "/tmp/trajectories")

    def test_a_bare_name_lands_in_the_directory_with_an_extension(self):
        os.environ["LITEGRIP_TRAJ_DIR"] = "/tmp/lg"
        self.assertEqual(resolve_path("pick"), "/tmp/lg/pick.lgt")
        self.assertEqual(resolve_path("pick.lgt"), "/tmp/lg/pick.lgt")

    def test_a_path_is_used_as_written(self):
        self.assertEqual(resolve_path("out/pick.lgt"), "out/pick.lgt")
        self.assertEqual(resolve_path("/tmp/pick.lgt"), "/tmp/pick.lgt")

    def test_save_creates_the_directory_and_load_reads_it_back(self):
        os.environ["LITEGRIP_TRAJ_DIR"] = os.path.join(self._root, "nested")
        traj = _trajectory([0.0, 0.5, 1.0], mount="reverse", reverse=True)
        written = traj.save("demo")
        self.assertEqual(written,
                         os.path.join(self._root, "nested", "demo.lgt"))
        self.assertTrue(os.path.isfile(written))
        self.assertEqual(Trajectory.load("demo").samples, traj.samples)
        self.assertEqual(Trajectory.load(written).mount, "reverse")

    def test_load_rejects_a_file_that_is_not_a_trajectory(self):
        os.environ["LITEGRIP_TRAJ_DIR"] = self._root
        with open(resolve_path("junk"), "wb") as f:
            f.write(b"definitely not a trajectory")
        with self.assertRaises(TrajectoryFormatError):
            Trajectory.load("junk")


# ═══════════════════════════════════════════════════════════════════════════
# Recording
# ═══════════════════════════════════════════════════════════════════════════


class RecorderTest(unittest.TestCase):
    def test_record_captures_the_timed_number_of_samples(self):
        g, _ = make_gripper()
        _use_clock(g)
        traj = g.record(0.5, rate_hz=100.0)
        self.assertEqual(len(traj), 50)
        self.assertEqual(traj.sample_hz, 100.0)

    def test_the_reference_sample_is_taken_at_t_zero_before_any_frame(self):
        """Sample 0 is taken by start(), so the count and t=0 are not raced."""
        g, fake = make_gripper()
        _use_clock(g)
        before = fake.motor.reported_pos()
        traj = g.record(0.05, rate_hz=100.0)
        first = traj.samples[0]
        self.assertEqual(first.t, 0.0)
        self.assertEqual(first.position_rad, before)
        self.assertAlmostEqual(first.openness, rad_to_openness(before, g.config))

    def test_a_recording_carries_the_calibration_it_was_taken_against(self):
        g, _ = make_gripper(reverse=True)
        _use_clock(g)
        traj = g.record(0.03, rate_hz=100.0)
        self.assertEqual(traj.mount, "reverse")
        self.assertAlmostEqual(traj.pos_closed_rad, POS_OPEN_RAD)
        self.assertAlmostEqual(traj.pos_open_rad, POS_CLOSED_RAD)
        self.assertAlmostEqual(traj.rad_to_mm, RAD_TO_MM)
        self.assertEqual(traj.can_id, g.can_id)

    def test_record_requires_a_positive_duration(self):
        g, _ = make_gripper()
        for bad in (0.0, -1.0):
            with self.subTest(duration_s=bad):
                with self.assertRaises(ValueError):
                    g.record(bad)

    def test_record_requires_calibration(self):
        """Without a real calibration the normalised opening would be a guess."""
        g, _ = make_gripper()
        g.config.calibrated = False
        with self.assertRaises(TrajectoryError) as caught:
            g.record(0.1)
        self.assertIn("标定", str(caught.exception))
        self.assertIsNone(g.session, "a refused start must not claim the session")

    def test_record_requires_connection_and_enable(self):
        g, _ = make_gripper()
        g._connected = False
        with self.assertRaises(NotInitializedError):
            g.record(0.1)
        g._connected = True
        g._enabled = False
        with self.assertRaises(NotInitializedError):
            g.record(0.1)
        self.assertIsNone(g.session)

    def test_start_stop_round_trip_reaches_the_cap(self):
        g, _ = make_gripper()
        _pace(g)
        status = g.record_start(rate_hz=100.0, max_samples=10)
        self.assertTrue(status["active"])
        self.assertEqual(status["kind"], "record")
        self.assertTrue(_wait_until(lambda: not g.trajectory_status()["active"]),
                        "the recorder should stop itself at max_samples")
        traj = g.record_stop()
        self.assertEqual(len(traj), 10)
        self.assertIsNone(g.session)

    def test_the_cap_counts_the_reference_sample(self):
        """`max_samples=1` is one sample, not the reference plus one more."""
        g, _ = make_gripper()
        _pace(g)
        g.record_start(rate_hz=100.0, max_samples=1)
        self.assertTrue(_wait_until(lambda: not g.trajectory_status()["active"]))
        self.assertEqual(len(g.record_stop()), 1)

    def test_a_capture_shorter_than_one_sample_interval_yields_one(self):
        """record() rounds the target up to one sample, and stops there."""
        g, _ = make_gripper()
        _use_clock(g)
        self.assertEqual(len(g.record(0.004, rate_hz=100.0)), 1)

    def test_stopping_a_recording_that_is_not_running_is_an_error(self):
        g, _ = make_gripper()
        with self.assertRaises(TrajectoryNotActiveError):
            g.record_stop()

    def test_a_second_start_while_recording_is_busy(self):
        g, _ = make_gripper()
        _pace(g)
        g.record_start(rate_hz=100.0)
        try:
            with self.assertRaises(TrajectoryBusyError) as caught:
                g.record_start(rate_hz=100.0)
            self.assertIn("record", str(caught.exception))
        finally:
            g.record_stop(allow_empty=True)

    def test_a_dead_loop_is_not_returned_as_a_capture(self):
        """A CAN error mid-recording must not come back as a short trajectory."""
        g, fake = make_gripper()
        _use_clock(g)
        original = fake.control_mit
        calls = []

        def failing(*args, **kwargs):
            calls.append(1)
            if len(calls) > 3:
                raise RuntimeError("bus down")
            return original(*args, **kwargs)

        fake.control_mit = failing
        g.record_start(rate_hz=100.0, max_samples=1000)
        self.assertTrue(_wait_until(lambda: not g.trajectory_status()["active"]))
        with self.assertRaises(TrajectoryRecordingError) as caught:
            g.record_stop()
        self.assertIn("bus down", str(caught.exception))
        self.assertGreater(len(calls), 3)
        self.assertIsNone(g.session, "the session must be released on failure")

    def test_a_timed_capture_that_did_not_fill_raises(self):
        """record() never hands back a short capture as if it were whole."""
        g, _ = make_gripper()
        _frozen_clock(g)
        with self.assertRaises(TrajectoryRecordingError) as caught:
            g.record(0.1, rate_hz=100.0)
        self.assertIn("没有前进", str(caught.exception))
        self.assertIsNone(g.session)
        self.assertIsNone(g.trajectory_status()["kind"])

    def test_a_clock_that_never_advances_cannot_spin_forever(self):
        """The non-advancing-cursor rule, ported to a sampling clock."""
        g, _ = make_gripper()
        _frozen_clock(g)
        recorder = TrajectoryRecorder(g, rate_hz=1000.0, max_samples=100000)
        recorder.start()
        try:
            with self.assertRaises(TrajectoryRecordingError) as caught:
                recorder.wait_for(100000, timeout=5.0)
            self.assertIn("没有前进", str(caught.exception))
            self.assertEqual(recorder.sample_count, 1,
                             "only the reference sample exists; a repeated "
                             "timestamp is not a second measurement")
        finally:
            recorder.stop()

    def test_result_on_a_recorder_that_never_started_is_empty(self):
        g, _ = make_gripper()
        recorder = TrajectoryRecorder(g)
        with self.assertRaises(TrajectoryEmptyError):
            recorder.result()
        self.assertEqual(len(recorder.result(allow_empty=True)), 0)

    def test_zero_gravity_streams_a_torque_free_frame_every_cycle(self):
        """The motor faults ~100 ms after frames stop, so recording must send."""
        g, fake = make_gripper()
        _use_clock(g)
        traj = g.record(0.05, rate_hz=100.0, zero_gravity=True)
        streamed = fake.frames[:-1]           # the last one is the hold on stop
        self.assertEqual(len(streamed), len(traj) - 1,
                         "every sample but the reference one has its frame")
        for frame in streamed:
            self.assertEqual((frame.kp, frame.kd), (0.0, 0.0))
            self.assertEqual(frame.q, 0.0)

    def test_a_read_only_recording_never_touches_the_bus(self):
        """With zero_gravity off the caller drives, so the recorder only reads."""
        g, fake = make_gripper()
        _use_clock(g)
        g.record(0.05, rate_hz=100.0, zero_gravity=False)
        self.assertEqual(fake.frames, [])

    def test_record_stop_leaves_the_gripper_holding_not_slack(self):
        g, fake = make_gripper()
        _use_clock(g)
        g.record(0.05, rate_hz=100.0, zero_gravity=True)
        last = fake.frames[-1]
        self.assertAlmostEqual(last.kp, g.config.kp)
        self.assertAlmostEqual(last.kd, g.config.kd)
        self.assertEqual(last.q, fake.motor.reported_pos())

    def test_status_shape_while_recording(self):
        g, _ = make_gripper()
        _pace(g)
        status = g.record_start(rate_hz=50.0)
        try:
            self.assertEqual(status["kind"], "record")
            self.assertTrue(status["active"])
            self.assertEqual(status["rate_hz"], 50.0)
            self.assertTrue(status["zero_gravity"])
            self.assertIsNone(status["error"])
            self.assertIn("samples", status)
        finally:
            g.record_stop(allow_empty=True)


# ═══════════════════════════════════════════════════════════════════════════
# Replay
# ═══════════════════════════════════════════════════════════════════════════


class PlayerTest(unittest.TestCase):
    def test_play_requires_calibration(self):
        g, _ = make_gripper()
        g.config.calibrated = False
        with self.assertRaises(TrajectoryError) as caught:
            g.play(_trajectory([0.0, 1.0]))
        self.assertIn("标定", str(caught.exception))
        self.assertIsNone(g.session)

    def test_play_rejects_an_empty_trajectory(self):
        g, _ = make_gripper()
        with self.assertRaises(TrajectoryEmptyError):
            g.play(Trajectory())
        self.assertIsNone(g.session)

    def test_play_rejects_a_non_positive_speed(self):
        g, _ = make_gripper()
        for bad in (0.0, -1.0):
            with self.subTest(speed=bad):
                with self.assertRaises(ValueError):
                    g.play(_trajectory([0.0, 1.0]), speed=bad)
        self.assertIsNone(g.session)

    def test_blocking_play_rejects_loop(self):
        """A looping blocking replay could never return."""
        g, _ = make_gripper()
        with self.assertRaises(ValueError) as caught:
            g.play(_trajectory([0.0, 1.0]), loop=True)
        self.assertIn("play_start", str(caught.exception))
        self.assertIsNone(g.session)

    def test_an_interrupted_blocking_replay_releases_the_session(self):
        """Ctrl+C out of play() must stop the player, not just unwind."""
        g, fake = make_gripper()
        _pace(g, step=0.05)
        original = TrajectoryPlayer.wait

        def interrupted(self, timeout):
            raise KeyboardInterrupt

        TrajectoryPlayer.wait = interrupted
        self.addCleanup(setattr, TrajectoryPlayer, "wait", original)
        with self.assertRaises(KeyboardInterrupt):
            g.play(_trajectory([0.0, 1.0], dt=100.0), align=False)
        self.assertIsNone(g.session, "the session must not stay claimed")
        self.assertEqual(g.trajectory_status(), {"active": False, "kind": None})
        frames = len(fake.frames)
        time.sleep(0.05)                      # several paced cycles
        self.assertEqual(len(fake.frames), frames,
                         "the player must be stopped, not still commanding")

    def test_follows_the_trajectory_from_start_to_end(self):
        g, fake = make_gripper()
        _use_clock(g)
        status = g.play(_trajectory([0.0, 0.5, 1.0], dt=0.5), align=False)
        self.assertFalse(status["active"])
        self.assertTrue(status["completed"])
        commanded = _commanded(fake, g)
        self.assertAlmostEqual(commanded[0], 0.0, places=6)
        self.assertAlmostEqual(commanded[-1], 1.0, places=6)
        self.assertEqual(commanded, sorted(commanded), "the path is monotone")
        self.assertGreater(len(commanded), 50, "a ramp, not just the endpoints")

    def test_the_pace_is_wall_clock_not_one_index_per_cycle(self):
        """Frames land between the samples: the path is interpolated."""
        g, fake = make_gripper()
        _use_clock(g, step=0.125)             # 8 frames over a 1 s trajectory
        g.play(_trajectory([0.0, 1.0], dt=1.0), align=False)
        commanded = _commanded(fake, g)
        self.assertAlmostEqual(commanded[0], 0.0, places=9)
        self.assertAlmostEqual(commanded[1], 0.125, places=9)
        self.assertAlmostEqual(commanded[2], 0.25, places=9)
        self.assertAlmostEqual(commanded[-1], 1.0, places=9)

    def test_a_nonzero_first_timestamp_does_not_delay_the_start(self):
        """t is relative to the recording, so a shifted one still plays at once."""
        traj = _trajectory([0.0, 1.0], dt=0.5)
        traj.samples = [TrajectorySample(t=s.t + 5.0, openness=s.openness,
                                         position_rad=s.position_rad)
                        for s in traj.samples]
        g, fake = make_gripper()
        _use_clock(g, step=0.125)
        g.play(traj, align=False)
        commanded = _commanded(fake, g)
        self.assertEqual(len(commanded), 5)
        self.assertAlmostEqual(commanded[0], 0.0, places=9)
        self.assertAlmostEqual(commanded[-1], 1.0, places=9)

    def test_aligning_walks_to_the_first_sample_without_a_full_torque_step(self):
        g, fake = make_gripper()
        _use_clock(g)
        g.play(_trajectory([0.25, 1.0], dt=0.5), align=True)

        cap_rad = g.motion_config.max_lead_mm / g.config.rad_to_mm
        target = openness_to_rad(0.25, g.config)
        # The align used to command the whole target on its first frame — a
        # full-torque step from wherever the jaws happened to be.  It now walks
        # there: the first frame is one capped step off the start, with the
        # travel speed fed forward as dq.
        self.assertLessEqual(abs(fake.frames[0].q - POS_OPEN_RAD), cap_rad + 1e-9)
        self.assertAlmostEqual(fake.frames[0].dq,
                               g.motion_config.speed_mm_s / g.config.rad_to_mm)
        # The gap it covers is far past the cap — that is the step this replaces.
        self.assertGreater(abs(target - POS_OPEN_RAD), 10.0 * cap_rad)
        # ...and it still arrives before the replay takes over.
        self.assertTrue(any(abs(f.q - target) < 1e-9 for f in fake.frames))

    def test_speed_scales_how_long_the_replay_takes(self):
        """Half speed means twice the frames for the same trajectory."""
        fast_g, fast_fake = make_gripper()
        _use_clock(fast_g)
        fast_g.play(_trajectory([0.0, 1.0], dt=0.5), align=False)

        slow_g, slow_fake = make_gripper()
        _use_clock(slow_g)
        slow_g.play(_trajectory([0.0, 1.0], dt=0.5), speed=0.5, align=False)

        self.assertAlmostEqual(len(_commanded(slow_fake, slow_g)),
                               2 * len(_commanded(fast_fake, fast_g)), delta=2)

    def test_a_slow_cycle_skips_ahead_instead_of_lagging(self):
        """Playback is wall-clock driven, so an overrun does not stretch time."""
        g, fake = make_gripper()
        _use_clock(g, step=0.25)              # each cycle covers 0.25 s of path
        g.play(_trajectory([0.0, 0.5, 1.0], dt=0.5), align=False)
        commanded = _commanded(fake, g)
        self.assertEqual(len(commanded), 5,
                         "an overrunning cycle must skip, not replay 200 frames")
        self.assertAlmostEqual(commanded[-1], 1.0, places=9)

    def test_the_last_frame_is_the_end_and_the_stop_holds_under_the_gains(self):
        g, fake = make_gripper()
        step = g.motion_config.frame_interval
        _use_clock(g, step=step)
        g.play_start(_trajectory([0.0, 1.0], dt=0.5), align=False)
        self.assertTrue(_wait_until(lambda: not g.trajectory_status()["active"]))
        g.play_stop()
        # One frame per cycle across the whole span, both ends included.
        self.assertEqual(len(_commanded(fake, g)), int(round(0.5 / step)) + 1)
        self.assertAlmostEqual(fake.frames[-2].q,
                               openness_to_rad(1.0, g.config), places=9)
        hold = fake.frames[-1]
        self.assertAlmostEqual(hold.q, fake.motor.reported_pos())
        self.assertAlmostEqual(hold.kp, g.config.kp)
        for frame in fake.frames:
            self.assertNotEqual(frame.kp, 0.0,
                                "replay must never leave the jaws slack")

    def test_recorded_torque_and_velocity_are_not_replayed(self):
        """Recorded tau/dq are diagnostics; they are sign- and mount-dependent."""
        g, fake = make_gripper()
        _use_clock(g)
        traj = _trajectory([0.0, 1.0], dt=0.5)
        traj.samples = [TrajectorySample(t=s.t, openness=s.openness,
                                         position_rad=-9.0, velocity_rad_s=7.5,
                                         torque_nm=3.25)
                        for s in traj.samples]
        g.play(traj, align=False)
        for frame in fake.frames:
            self.assertEqual(frame.tau_ff, 0.0)
            self.assertEqual(frame.dq, 0.0)

    def test_loop_wraps_instead_of_stopping(self):
        g, fake = make_gripper()
        _pace(g, step=0.05)
        g.play_start(_trajectory([0.0, 1.0], dt=0.1), loop=True, align=False)
        try:
            self.assertTrue(_wait_until(lambda: len(fake.frames) > 60),
                            "a looping replay should keep sending frames")
            status = g.trajectory_status()
            self.assertTrue(status["active"])
            self.assertFalse(status["completed"])
        finally:
            g.play_stop()
        self.assertIsNone(g.session)

    def test_a_failing_send_aborts_rather_than_dropping_frames(self):
        g, _ = make_gripper()
        _use_clock(g)
        g.send_mit_frame = lambda **kwargs: False
        g.play_start(_trajectory([0.0, 1.0], dt=0.1), align=False)
        self.assertTrue(_wait_until(lambda: not g.trajectory_status()["active"]))
        status = g.play_stop()
        self.assertIsNotNone(status["error"])
        self.assertIn("MIT 帧下发失败", status["error"])
        self.assertIsNone(g.session)

    def test_looping_a_one_sample_trajectory_holds_that_pose(self):
        """A pose has no length to advance along; looping it must still hold."""
        g, fake = make_gripper()
        _pace(g, step=0.05)
        g.play_start(_trajectory([0.7]), loop=True, align=False)
        try:
            self.assertTrue(_wait_until(lambda: len(fake.frames) > 20),
                            "a held pose must keep sending frames")
            self.assertFalse(g.trajectory_status()["completed"])
        finally:
            g.play_stop()
        for frame in fake.frames[:-1]:        # the last one is the hold on stop
            self.assertAlmostEqual(rad_to_openness(frame.q, g.config), 0.7,
                                   places=9)

    def test_a_clock_that_never_advances_cannot_flood_the_bus(self):
        """The player carries the recorder's stall rule, for the same reason."""
        g, fake = make_gripper()
        _frozen_clock(g)
        g.play_start(_trajectory([0.0, 1.0], dt=5.0), align=False)
        self.assertTrue(_wait_until(lambda: not g.trajectory_status()["active"]))
        status = g.play_stop()
        self.assertIn("没有前进", status["error"])
        # A few frames before the guard fires, then the hold on stop — not the
        # unbounded stream a wall-clock-driven loop would otherwise emit.
        self.assertLessEqual(len(fake.frames), 8)
        self.assertIsNone(g.session)

    def test_play_stop_leaves_the_gripper_holding(self):
        g, fake = make_gripper()
        _pace(g, step=0.05)
        g.play_start(_trajectory([0.0, 1.0], dt=100.0), align=False)
        try:
            self.assertTrue(_wait_until(lambda: len(fake.frames) > 3))
        finally:
            g.play_stop()
        last = fake.frames[-1]
        self.assertAlmostEqual(last.kp, g.config.kp)
        self.assertEqual(last.q, fake.motor.reported_pos())

    def test_status_shape_while_replaying(self):
        g, _ = make_gripper()
        _pace(g, step=0.05)
        status = g.play_start(_trajectory([0.0, 1.0], dt=100.0), speed=0.5,
                              align=False)
        try:
            self.assertEqual(status["kind"], "play")
            self.assertTrue(status["active"])
            self.assertEqual(status["samples"], 2)
            self.assertEqual(status["speed"], 0.5)
            self.assertFalse(status["loop"])
        finally:
            g.play_stop()


class PortabilityTest(unittest.TestCase):
    """The point of storing openness: a trajectory replays on another unit."""

    def _replay(self, reverse: bool):
        g, fake = make_gripper(reverse=reverse)
        _use_clock(g)
        g.play(_trajectory([0.0, 0.4, 1.0], dt=0.5), align=False)  # normal-mount
        return g, fake, _commanded(fake, g)

    def test_openness_is_reproduced_on_a_reverse_mounted_unit(self):
        normal_g, normal_fake, normal = self._replay(reverse=False)
        reverse_g, reverse_fake, reverse = self._replay(reverse=True)

        self.assertEqual(len(normal), len(reverse))
        self.assertAlmostEqual(normal[0], reverse[0], places=6)
        self.assertAlmostEqual(normal[-1], reverse[-1], places=6)

        # Same motion, opposite raw angle: the mount flips the sign, nothing else.
        self.assertEqual(normal_g.config.close_sign, 1.0)
        self.assertEqual(reverse_g.config.close_sign, -1.0)
        self.assertLess(normal_fake.frames[-1].q, normal_fake.frames[0].q)
        self.assertGreater(reverse_fake.frames[-1].q, reverse_fake.frames[0].q)

    def test_openness_endpoints_map_to_each_units_own_limits(self):
        for reverse in (False, True):
            g, _ = make_gripper(reverse=reverse)
            with self.subTest(reverse=reverse):
                self.assertAlmostEqual(openness_to_rad(0.0, g.config),
                                       g.config.pos_closed_rad, places=9)
                self.assertAlmostEqual(openness_to_rad(1.0, g.config),
                                       g.config.pos_open_rad, places=9)

    def test_a_stored_opening_beyond_travel_is_clamped(self):
        """A hand-edited or foreign file cannot command past a mechanical stop."""
        traj = Trajectory(samples=[
            TrajectorySample(t=0.0, openness=5.0, position_rad=0.0),
            TrajectorySample(t=0.5, openness=-5.0, position_rad=0.0),
        ])
        g, fake = make_gripper()
        _use_clock(g)
        g.play(traj, align=False)
        limits = sorted([g.config.pos_closed_rad, g.config.pos_open_rad])
        for frame in fake.frames:
            self.assertGreaterEqual(frame.q, limits[0] - 1e-9)
            self.assertLessEqual(frame.q, limits[1] + 1e-9)


# ═══════════════════════════════════════════════════════════════════════════
# Sessions
# ═══════════════════════════════════════════════════════════════════════════


class SessionTest(unittest.TestCase):
    def test_status_is_idle_when_nothing_runs(self):
        g, _ = make_gripper()
        self.assertEqual(g.trajectory_status(), {"active": False, "kind": None})
        self.assertIsNone(g.session)

    def test_stopping_when_idle_is_harmless(self):
        g, _ = make_gripper()
        self.assertEqual(g.play_stop(), {"active": False, "kind": None})

    def test_teleop_excludes_recording_and_replay(self):
        g, _ = make_gripper()
        _pace(g)
        g.teleop_start("master", transport=InProcTeleopTransport())
        try:
            self.assertEqual(g.session, "teleop")
            with self.assertRaises(TrajectoryBusyError) as caught:
                g.record_start(rate_hz=100.0)
            self.assertIn("teleop", str(caught.exception))
            with self.assertRaises(TrajectoryBusyError):
                g.play_start(_trajectory([0.0, 1.0]))
        finally:
            g.teleop_stop()
        self.assertIsNone(g.session)

    def test_teleop_refuses_to_start_while_recording(self):
        """Teleop keeps its own error type, and names what is in the way."""
        g, _ = make_gripper()
        _pace(g)
        g.record_start(rate_hz=100.0)
        try:
            with self.assertRaises(TeleopBusyError) as caught:
                g.teleop_start("master", transport=InProcTeleopTransport())
            self.assertIn("record", str(caught.exception))
            self.assertEqual(g.session, "record")
        finally:
            g.record_stop(allow_empty=True)

    def test_recording_and_replay_cannot_both_run(self):
        g, _ = make_gripper()
        _pace(g)
        g.record_start(rate_hz=100.0)
        try:
            with self.assertRaises(TrajectoryBusyError):
                g.play_start(_trajectory([0.0, 1.0]))
        finally:
            g.record_stop(allow_empty=True)
        self.assertIsNone(g.session)

    def test_only_one_of_two_racing_starts_wins(self):
        """The session is claimed under a lock, not checked and then taken."""
        g, _ = make_gripper()
        _pace(g)
        outcomes = []
        barrier = threading.Barrier(2)

        def start_recording():
            barrier.wait()
            try:
                g.record_start(rate_hz=100.0)
                outcomes.append("record")
            except TrajectoryBusyError:
                outcomes.append("refused")

        def start_playing():
            barrier.wait()
            try:
                g.play_start(_trajectory([0.0, 1.0]), align=False)
                outcomes.append("play")
            except TrajectoryBusyError:
                outcomes.append("refused")

        threads = [threading.Thread(target=start_recording),
                   threading.Thread(target=start_playing)]
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join(timeout=WAIT_S)
        try:
            self.assertEqual(len(outcomes), 2, outcomes)
            self.assertEqual(outcomes.count("refused"), 1, outcomes)
        finally:
            if g.session == "record":
                g.record_stop(allow_empty=True)
            elif g.session == "play":
                g.play_stop()
        self.assertIsNone(g.session)

    def test_disconnect_stops_a_running_recording(self):
        """The loop has to be stopped, not just unlinked from the gripper."""
        g, fake = make_gripper()
        _pace(g)
        g.record_start(rate_hz=100.0)
        recorder = g._trajectory_recorder
        self.assertTrue(recorder.is_recording)
        g.disconnect()
        self.assertFalse(recorder.is_recording)
        # Only TrajectoryRecorder.stop() clears this, so it pins the teardown
        # rather than the bookkeeping that runs either way.
        self.assertIsNone(recorder._thread)
        # stop() leaves zero gravity, sending one hold frame under the
        # configured gains — the gripper is held, not left slack.
        self.assertAlmostEqual(fake.frames[-1].kp, g.config.kp)
        self.assertEqual(g.trajectory_status(), {"active": False, "kind": None})
        self.assertIsNone(g.session)

    def test_disconnect_stops_a_running_replay(self):
        g, fake = make_gripper()
        _pace(g, step=0.05)
        g.play_start(_trajectory([0.0, 1.0], dt=100.0), align=False)
        player = g._trajectory_player
        self.assertTrue(_wait_until(lambda: len(fake.frames) > 3))
        g.disconnect()
        self.assertFalse(player.is_playing)
        self.assertIsNone(player._thread,
                          "disconnect must join the player, not just drop it")
        self.assertIsNone(g.session)
        frames = len(fake.frames)
        time.sleep(0.05)                      # several paced cycles
        self.assertEqual(len(fake.frames), frames,
                         "the player must not still be commanding the motor")


class SeamTest(unittest.TestCase):
    """The loops take their timing from MotionConfig, not from ``time``."""

    def test_explicit_seams_win_over_the_config(self):
        g, _ = make_gripper()
        monotonic = lambda: 0.0  # noqa: E731
        sleep = lambda _s: None  # noqa: E731
        recorder = TrajectoryRecorder(g, sleep_fn=sleep, monotonic_fn=monotonic)
        self.assertIs(recorder._sleep_fn, sleep)
        self.assertIs(recorder._monotonic_fn, monotonic)

    def test_the_loops_fall_back_to_the_config(self):
        g, _ = make_gripper()
        g.motion_config.monotonic_fn = lambda: 1.0
        recorder = TrajectoryRecorder(g)
        player = TrajectoryPlayer(g, _trajectory([0.0, 1.0]))
        self.assertIs(recorder._monotonic_fn, g.motion_config.monotonic_fn)
        self.assertIs(player._monotonic_fn, g.motion_config.monotonic_fn)


if __name__ == "__main__":
    unittest.main()
