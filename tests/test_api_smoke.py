"""Hardware-free smoke test for the complete public ``Gripper`` surface.

Everything here goes through the documented API, with the bus replaced by
``tests/fake_can.py``.  It is the guard that the facade — not just the engine —
keeps its promises: bool returns for commands, exceptions for invalid usage,
and getters that raise instead of returning plausible zeros when disconnected.
"""

from __future__ import annotations

import os
import tempfile
import unittest
from unittest import mock

import _sdkpath  # noqa: F401
import litegrip
from litegrip import (CommandError, Gripper, MotionConfig, NotInitializedError,
                      default_calibration_path)

from fake_can import POS_CLOSED_RAD, POS_OPEN_RAD, make_gripper


GETTERS = (
    "get_state",
    "refresh_status",
    "get_motor_data",
    "get_motor_status",
    "get_motor_position",
    "get_motor_speed",
    "get_motor_torque",
    "get_gripper_distance",
    "get_driver_temp",
    "get_motor_temp",
    "get_status_raw",
    "get_gripper_force",
)


class TestConstructor(unittest.TestCase):
    """The constructor signature and the auto-loaded calibration."""

    def test_defaults(self):
        with tempfile.TemporaryDirectory() as home:
            with mock.patch.dict(os.environ, {"HOME": home}, clear=False):
                os.environ.pop("LITEGRIP_CALIB", None)
                g = Gripper()
        self.assertEqual(g.channel, "can0")
        self.assertEqual(g.can_id, 0x08)
        self.assertEqual(g.mst_id, 0x18)          # no auto-detect
        self.assertFalse(g.is_connected)
        self.assertFalse(g.is_enabled)

    def test_auto_loads_the_factory_calibration(self):
        g = Gripper("vcan0")
        self.assertTrue(g.config.calibrated)
        # The bundled file sets calibrated=true, so distance commands work out
        # of the box.
        self.assertNotAlmostEqual(g.config.pos_closed_rad, g.config.pos_open_rad)

    def test_motion_config_is_accepted(self):
        motion = MotionConfig(speed_mm_s=12.0)
        g = Gripper("vcan0", motion_config=motion)
        self.assertIs(g.motion_config, motion)

    def test_invalid_ids_are_rejected(self):
        for kwargs in ({"can_id": -1}, {"can_id": 0x800},
                       {"mst_id": 0x800}):
            with self.assertRaises(CommandError):
                Gripper("can0", **kwargs)

    def test_default_calibration_path_is_per_channel(self):
        path = default_calibration_path("can7")
        self.assertTrue(path.endswith("can7_calibration.json"))
        self.assertIn(".litegrip", path)


class TestGettersNeedAConnection(unittest.TestCase):
    """A getter must not answer with a plausible zero before connect()."""

    def test_every_getter_raises(self):
        g = Gripper("vcan0")
        for name in GETTERS:
            with self.subTest(getter=name):
                with self.assertRaises(NotInitializedError):
                    getattr(g, name)()

    def test_motion_commands_raise(self):
        g = Gripper("vcan0")
        calls = (
            lambda: g.open(),
            lambda: g.close(),
            lambda: g.set_gripper_distance(10.0),
            lambda: g.set_motor_angle(0.0),
            lambda: g.follow_gripper_distance(0.0),
            lambda: g.grasp(force_n=5.0, hold_s=0.0),
            lambda: g.release(5.0),
            lambda: g.enable(),
            lambda: g.disable(),
            lambda: g.clear_fault(),
            lambda: g.set_zero(),
            lambda: g.calibrate(),
        )
        for call in calls:
            with self.subTest(call=call):
                with self.assertRaises(NotInitializedError):
                    call()


class TestFeedback(unittest.TestCase):
    """Types, keys and units of the feedback surface."""

    def setUp(self):
        self.g, self.fake = make_gripper(
            start_rad=(POS_OPEN_RAD + POS_CLOSED_RAD) / 2.0)

    def test_state_snapshot(self):
        state = self.g.get_state()
        self.assertTrue(state.fresh)
        self.assertFalse(state.is_stale())
        self.assertGreaterEqual(state.data_age_s, 0.0)

    def test_raw_motor_data(self):
        data = self.g.get_motor_data()
        self.assertEqual(set(data), {"Position", "Speed", "Torque"})
        self.assertAlmostEqual(data["Position"], self.fake.motor.pos)
        self.assertAlmostEqual(data["Speed"], self.fake.motor.vel)
        self.assertAlmostEqual(data["Torque"], self.fake.motor.tau)

    def test_motor_status(self):
        status = self.g.get_motor_status()
        self.assertEqual(set(status), {"Status", "DriverTemp", "MotorTemp"})
        self.assertEqual(status["Status"], "0x01")
        self.assertEqual(status["DriverTemp"], 30)
        self.assertEqual(status["MotorTemp"], 35)

    def test_individual_getters_match_the_snapshot(self):
        state = self.g.get_state()
        self.assertAlmostEqual(self.g.get_motor_position(), state.position_rad)
        self.assertAlmostEqual(self.g.get_motor_speed(), state.velocity_rad_s)
        self.assertAlmostEqual(self.g.get_motor_torque(), state.torque_nm)
        self.assertEqual(self.g.get_driver_temp(), state.temperature_mos)
        self.assertEqual(self.g.get_motor_temp(), state.temperature_coil)
        self.assertEqual(self.g.get_status_raw(), "0x01")
        self.assertAlmostEqual(self.g.get_gripper_distance(), state.position_mm)

    def test_distance_is_zero_closed_at_the_closed_limit(self):
        self.fake.motor.pos = self.g.config.pos_closed_rad
        self.assertAlmostEqual(self.g.get_gripper_distance(), 0.0, places=6)

    def test_version_is_the_sdk_version_not_firmware(self):
        self.assertEqual(self.g.get_version(), litegrip.__version__)

    def test_refresh_status_returns_bool(self):
        self.assertIs(self.g.refresh_status(), True)
        self.assertGreaterEqual(self.fake.refresh_calls, 1)


class TestCommandsReturnBool(unittest.TestCase):
    """Commands report success as a bool; a failed motion is False, not True."""

    def test_lifecycle_and_motion(self):
        g, fake = make_gripper(stops=True)
        self.assertIs(g.open(), True)
        self.assertIs(g.close(), True)
        self.assertIs(g.set_gripper_distance(40.0), True)
        self.assertIs(g.set_motor_angle(g.config.pos_closed_rad - 0.5), True)
        self.assertIs(g.grasp(force_n=5.0, hold_s=0.05), True)
        self.assertIs(g.release(5.0), True)

    def test_enable_disable_clear_fault(self):
        g, fake = make_gripper(err=0, initialize_results=[True])
        self.assertIs(g.enable(), True)
        self.assertIs(g.disable(), True)
        self.assertIs(g.clear_fault(), True)

    def test_a_blocked_absolute_move_returns_false(self):
        g, fake = make_gripper(block_rad=POS_OPEN_RAD + 0.03)
        self.assertIs(g.set_gripper_distance(40.0), False)

    def test_invalid_arguments_raise(self):
        g, _ = make_gripper()
        with self.assertRaises(CommandError):
            g.set_gripper_distance(float("nan"))
        with self.assertRaises(CommandError):
            g.set_motor_angle(float("inf"))
        with self.assertRaises(CommandError):
            g.set_gripper_distance(10.0, 0.0)
        with self.assertRaises(CommandError):
            g.release(-1.0)
        with self.assertRaises(CommandError):
            g.grasp(force_n=float("nan"))


class TestFollowGripperDistance(unittest.TestCase):
    """The teleoperation primitive: one frame, no ramp, bounded inputs."""

    def setUp(self):
        self.g, self.fake = make_gripper(start_rad=-0.5)
        self.fake.frames.clear()

    def test_sends_one_clamped_frame_with_default_gains(self):
        target_mm = 1000.0                            # past the open stop
        self.assertIs(self.g.follow_gripper_distance(target_mm), True)
        self.assertEqual(len(self.fake.frames), 1)
        frame = self.fake.frames[0]
        self.assertAlmostEqual(frame.q, self.g.config.pos_open_rad)
        self.assertAlmostEqual(frame.kp, self.g.config.kp)
        self.assertAlmostEqual(frame.kd, self.g.config.kd)
        self.assertEqual(frame.dq, 0.0)
        self.assertEqual(frame.tau_ff, 0.0)

    def test_distance_is_converted_with_local_calibration(self):
        distance_mm = 40.0
        self.assertIs(self.g.follow_gripper_distance(distance_mm), True)
        expected = (self.g.config.pos_closed_rad
                    - self.g.config.close_sign * distance_mm
                    / self.g.config.rad_to_mm)
        self.assertAlmostEqual(self.fake.frames[0].q, expected)

    def test_explicit_gains_and_feedforward_reach_the_frame(self):
        self.assertIs(
            self.g.follow_gripper_distance(40.0, kp=50.0, kd=1.0,
                                           dq=2.0, tau=0.5),
            True)
        frame = self.fake.frames[0]
        self.assertAlmostEqual(frame.kp, 50.0)
        self.assertAlmostEqual(frame.kd, 1.0)
        self.assertAlmostEqual(frame.dq, 2.0)
        self.assertAlmostEqual(frame.tau_ff, 0.5)

    def test_out_of_range_is_refused_before_sending(self):
        for kwargs in ({"distance_mm": float("nan")},
                       {"distance_mm": 0.0, "kp": 501.0},
                       {"distance_mm": 0.0, "kd": 6.0},
                       {"distance_mm": 0.0, "dq": 11.0},
                       {"distance_mm": 0.0, "tau": 4.0}):
            with self.subTest(kwargs=kwargs):
                with self.assertRaises(CommandError):
                    self.g.follow_gripper_distance(**kwargs)
        self.assertEqual(self.fake.frames, [])

    def test_send_failure_returns_false(self):
        self.fake.send_error = RuntimeError("bus down")
        self.assertIs(self.g.follow_gripper_distance(40.0), False)
        self.assertIsNotNone(self.g.last_send_error)

    def test_force_feedback_callback_receives_the_measured_squeeze(self):
        seen = []
        self.assertIs(
            self.g.follow_gripper_distance(
                40.0, force_feedback=seen.append), True)
        self.assertEqual(len(seen), 1)
        self.assertAlmostEqual(seen[0], self.g.get_gripper_force())

    def test_force_feedback_callback_is_not_called_on_send_failure(self):
        self.fake.send_error = RuntimeError("bus down")
        seen = []
        self.assertIs(
            self.g.follow_gripper_distance(
                40.0, force_feedback=seen.append), False)
        self.assertEqual(seen, [])


class TestForceFeedback(unittest.TestCase):
    """Leader-side haptic rendering; off until configured."""

    def setUp(self):
        self.g, self.fake = make_gripper(start_rad=-0.5)
        self.fake.frames.clear()

    def test_disabled_by_default_sends_nothing(self):
        self.assertIs(self.g.render_force_feedback(20.0), False)
        self.assertEqual(self.fake.frames, [])

    def test_enabled_renders_capped_resistance_and_damping(self):
        self.g.configure_force_feedback(enabled=True)
        self.assertIs(self.g.render_force_feedback(20.0), True)
        frame = self.fake.frames[0]
        # (20 - 2) N * 0.1 Nm/N = 1.8, capped at 1.0; opposes closing.
        self.assertAlmostEqual(frame.tau_ff, -1.0)
        self.assertAlmostEqual(frame.kd, 0.1)
        self.assertAlmostEqual(frame.kp, 0.0)

    def test_below_threshold_is_a_free_frame(self):
        self.g.configure_force_feedback(enabled=True)
        self.assertIs(self.g.render_force_feedback(1.0), True)
        frame = self.fake.frames[0]
        self.assertEqual(frame.tau_ff, 0.0)
        self.assertEqual(frame.kd, 0.0)

    def test_stale_feedback_drops_the_resistance(self):
        self.g.configure_force_feedback(enabled=True)
        self.assertIs(self.g.render_force_feedback(20.0, age_s=1.0), True)
        frame = self.fake.frames[0]
        self.assertEqual(frame.tau_ff, 0.0)
        self.assertEqual(frame.kd, 0.0)

    def test_reverse_mount_flips_the_rendered_sign(self):
        g, fake = make_gripper(start_rad=-0.5, reverse=True)
        fake.frames.clear()
        g.configure_force_feedback(enabled=True)
        self.assertIs(g.render_force_feedback(20.0), True)
        self.assertAlmostEqual(fake.frames[0].tau_ff, 1.0)

    def test_max_tau_caps_the_rendered_resistance(self):
        self.g.configure_force_feedback(enabled=True, max_tau_nm=0.5)
        self.assertIs(self.g.render_force_feedback(100.0), True)
        self.assertAlmostEqual(self.fake.frames[0].tau_ff, -0.5)

    def test_invalid_configuration_is_refused(self):
        for kwargs in ({"gain_nm_per_n": -1.0},
                       {"threshold_n": -1.0},
                       {"max_tau_nm": 4.0},
                       {"damping": 6.0},
                       {"timeout_s": 0.0},
                       {"gain_nm_per_n": float("nan")}):
            with self.subTest(kwargs=kwargs):
                with self.assertRaises(CommandError):
                    self.g.configure_force_feedback(enabled=True, **kwargs)

    def test_invalid_inputs_are_refused_before_sending(self):
        self.g.configure_force_feedback(enabled=True)
        with self.assertRaises(CommandError):
            self.g.render_force_feedback(float("nan"))
        with self.assertRaises(CommandError):
            self.g.render_force_feedback(20.0, age_s=-1.0)
        self.assertEqual(self.fake.frames, [])

    def test_enabled_but_disconnected_raises(self):
        g = Gripper("vcan0")
        g.configure_force_feedback(enabled=True)
        with self.assertRaises(NotInitializedError):
            g.render_force_feedback(20.0)


class TestSetZeroIsOnTheSurface(unittest.TestCase):
    def test_set_zero_returns_bool(self):
        g, fake = make_gripper(start_rad=-0.5)
        self.assertIs(g.set_zero(), True)
        self.assertEqual(fake.calls, ["disable", "set_zero", "initialize"])


class TestNoRemovedPublicNames(unittest.TestCase):
    """Direct replacement: the old facade and result types are gone."""

    def test_old_names_are_not_exported(self):
        for name in ("LiteGrip", "MoveResult", "GraspResult", "ReleaseResult",
                     "EnableResult", "WriteZeroResult", "HoldResult",
                     "MoveProgress", "GripperInfo", "GripperStatus",
                     "GripperMode", "list_templates", "CALIB_TEMPLATES",
                     "DEFAULT_CALIB", "default_calib_path",
                     "Trajectory", "TrajectoryRecorder", "GripperTeleop",
                     "ZenohTeleopTransport"):
            with self.subTest(name=name):
                self.assertFalse(hasattr(litegrip, name))

    def test_removed_methods_are_not_on_the_gripper(self):
        g = Gripper("vcan0")
        for name in ("home", "goto", "goto_rad", "move_to", "move_at_speed",
                     "move_at_speed_rad", "request_release", "stop",
                     "send_mit_frame", "enter_zero_gravity",
                     "exit_zero_gravity", "read_param", "get_info",
                     "get_position", "get_position_rad", "get_force",
                     "get_torque", "get_error", "get_temperature",
                     "is_moving", "is_grasped", "wait_for_ready", "poll",
                     "calibrate_manual", "calibrate_guided", "zero",
                     "write_zero", "load_template", "list_templates",
                     "record", "play", "teleop_start", "session",
                     "get_voltage", "get_bus_current", "get_motor_current",
                     "get_error_code"):
            with self.subTest(name=name):
                self.assertFalse(hasattr(g, name))


if __name__ == "__main__":
    unittest.main()
