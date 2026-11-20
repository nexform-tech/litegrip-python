"""Calibration persistence and probe tests — hardware-free.

Covers: the shipped factory geometry, the constructor's auto-load search
order, per-channel files, direction preservation through a probe run, the two
independent probe guards (command-lead cap and torque ceiling), and the
``mst_id`` default.
"""

from __future__ import annotations

import contextlib
import io
import json
import logging
import os
import shutil
import tempfile
import unittest
from unittest import mock

import _sdkpath  # noqa: F401
import litegrip.gripper as gripper_mod
from litegrip import (CommandError, Gripper, GripperConfig, GripperGeometry,
                      MotionConfig, default_calibration_path)
from litegrip.actions import _scale_from_travel
from litegrip.gripper import _FACTORY_CALIB

from fake_can import (DT, POS_CLOSED_RAD, POS_OPEN_RAD, Frame, make_gripper)


def _capture_warnings(fn):
    """Run ``fn()`` and return the litegrip WARNING+ texts."""
    records = []

    class _Sink(logging.Handler):
        def emit(self, record):
            records.append(record.getMessage())

    sink = _Sink(level=logging.WARNING)
    logger = logging.getLogger("litegrip")
    logger.addHandler(sink)
    try:
        fn()
    finally:
        logger.removeHandler(sink)
    return records


def _write_json(data, directory, name="calib.json"):
    path = os.path.join(directory, name)
    with open(path, "w") as f:
        json.dump(data, f)
    return path


@contextlib.contextmanager
def _patched_calib_env(home):
    """Point ``HOME`` at a temp dir and clear ``LITEGRIP_CALIB``."""
    with mock.patch.dict(os.environ, {"HOME": home}, clear=False):
        os.environ.pop("LITEGRIP_CALIB", None)
        yield


class TestShippedCalibrationFile(unittest.TestCase):
    """The bundled factory calibration is real geometry, not a made-up span.

    The factory file follows litegrip-cpp's ``calibration/factory_calibration.json``:
    ``0.029564 / -1.141184 / 73.45731105242119`` over 86 mm.
    """

    FACTORY_CLOSED_RAD = 0.029564
    FACTORY_OPEN_RAD = -1.141184
    FACTORY_RAD_TO_MM = 73.45731105242119
    FACTORY_TRAVEL_MM = 86.0

    def _load(self):
        with open(_FACTORY_CALIB) as f:
            return json.load(f)

    def test_the_shipped_geometry_is_the_factory_geometry(self):
        data = self._load()
        self.assertAlmostEqual(data["zero_position_rad"], self.FACTORY_CLOSED_RAD)
        self.assertAlmostEqual(data["max_position_rad"], self.FACTORY_OPEN_RAD)
        self.assertAlmostEqual(data["rad_to_mm"], self.FACTORY_RAD_TO_MM)
        self.assertAlmostEqual(data["travel_range_rad"],
                               self.FACTORY_CLOSED_RAD - self.FACTORY_OPEN_RAD)

    def test_the_travel_is_the_real_86_mm(self):
        data = self._load()
        self.assertAlmostEqual(data["travel_range_rad"] * data["rad_to_mm"],
                               self.FACTORY_TRAVEL_MM, places=6)

    def test_the_work_stroke_stays_inside_the_travel(self):
        data = self._load()
        travel_mm = data["travel_range_rad"] * data["rad_to_mm"]
        self.assertGreater(data["work_stroke_mm"], 0.0)
        self.assertLess(data["work_stroke_mm"], travel_mm)

    def test_the_constructor_loads_the_factory_file(self):
        with tempfile.TemporaryDirectory() as home:
            with _patched_calib_env(home):
                # can1 has no per-channel file; the factory file still loads.
                g = Gripper("can1")
        cfg = g.config
        self.assertTrue(cfg.calibrated)
        self.assertAlmostEqual(cfg.pos_closed_rad, self.FACTORY_CLOSED_RAD)
        self.assertAlmostEqual(cfg.pos_open_rad, self.FACTORY_OPEN_RAD)
        travel_mm = abs(cfg.pos_open_rad - cfg.pos_closed_rad) * cfg.rad_to_mm
        self.assertAlmostEqual(travel_mm, self.FACTORY_TRAVEL_MM)


class TestPersistence(unittest.TestCase):
    def setUp(self):
        self.dir = tempfile.mkdtemp()
        self.addCleanup(shutil.rmtree, self.dir, True)

    def _path(self, name="calib.json"):
        return os.path.join(self.dir, name)

    def test_round_trip_preserves_direction(self):
        path = self._path()
        g = Gripper("can0")
        g.config.pos_closed_rad, g.config.pos_open_rad = POS_OPEN_RAD, POS_CLOSED_RAD
        g.config.rad_to_mm = 74.19
        g.config.calibrated = True
        g.save_calibration(path)

        h = Gripper("can0")
        self.assertTrue(h.load_calibration(path))
        self.assertAlmostEqual(h.config.pos_closed_rad, g.config.pos_closed_rad)
        self.assertAlmostEqual(h.config.pos_open_rad, g.config.pos_open_rad)
        self.assertAlmostEqual(h.config.rad_to_mm, g.config.rad_to_mm)
        self.assertTrue(h.config.calibrated)
        self.assertAlmostEqual(h.config.close_sign, -1.0)

    def test_saved_file_records_calibrated_and_identity(self):
        path = self._path()
        g = Gripper("can1", can_id=0x0A, mst_id=0x1A)
        g.save_calibration(path)
        with open(path) as f:
            data = json.load(f)
        self.assertTrue(data["calibrated"])
        self.assertEqual(data["can_id"], 0x0A)
        self.assertEqual(data["mst_id"], 0x1A)

    def test_old_file_without_the_flag_counts_as_calibrated(self):
        path = self._path()
        _write_json({"channel": "can0",
                     "zero_position_rad": 0.052071,
                     "max_position_rad": -1.357481,
                     "travel_range_rad": 1.409552,
                     "rad_to_mm": 61.01229326764816,
                     "work_stroke_mm": 80.0}, self.dir)
        with open(path) as f:
            self.assertNotIn("calibrated", json.load(f))
        g = Gripper("can0")
        self.assertTrue(g.load_calibration(path))
        self.assertTrue(g.config.calibrated)

    def test_channel_mismatch_warns(self):
        path = _write_json({"channel": "can1", "zero_position_rad": 0.1,
                            "max_position_rad": -1.5, "rad_to_mm": 74.8},
                           self.dir)
        warnings = _capture_warnings(
            lambda: Gripper("can0").load_calibration(path))
        self.assertTrue(any("channel" in m for m in warnings), warnings)

    def test_matching_channel_is_silent(self):
        path = _write_json({"channel": "can0", "zero_position_rad": 0.1,
                            "max_position_rad": -1.5, "rad_to_mm": 74.8},
                           self.dir)
        self.assertEqual(
            _capture_warnings(lambda: Gripper("can0").load_calibration(path)),
            [])

    def test_missing_required_key_raises(self):
        path = _write_json({"zero_position_rad": 0.1}, self.dir)
        with self.assertRaises(CommandError):
            Gripper("can0").load_calibration(path)

    def test_non_finite_values_are_refused(self):
        path = _write_json({"zero_position_rad": 0.1,
                            "max_position_rad": -1.5,
                            "rad_to_mm": float("nan")}, self.dir)
        with self.assertRaises(CommandError):
            Gripper("can0").load_calibration(path)


@contextlib.contextmanager
def _home_for_calibrate():
    """Temporary HOME so ``calibrate()`` does not write the real one."""
    home = tempfile.mkdtemp()
    try:
        with _patched_calib_env(home):
            yield home
    finally:
        shutil.rmtree(home, True)


class TestCalibratePreservesDirection(unittest.TestCase):
    """Calibration cannot *discover* direction (both stops look the same); it
    preserves the direction already in the configuration."""

    def _calibrate(self, reverse):
        g, fake = make_gripper(reverse=reverse, stops=True)
        with _home_for_calibrate(), contextlib.redirect_stdout(io.StringIO()):
            data = g.calibrate()
        return g, data

    def test_normal_probe_steps_up_to_the_close_stop(self):
        g, data = self._calibrate(reverse=False)
        self.assertAlmostEqual(data.zero_position, POS_CLOSED_RAD, places=4)
        self.assertAlmostEqual(data.max_position, POS_OPEN_RAD, places=4)
        self.assertAlmostEqual(g.config.close_sign, 1.0)
        self.assertTrue(g.config.calibrated)

    def test_reverse_probe_steps_down_to_the_close_stop(self):
        g, data = self._calibrate(reverse=True)
        self.assertAlmostEqual(data.zero_position, POS_OPEN_RAD, places=4)
        self.assertAlmostEqual(data.max_position, POS_CLOSED_RAD, places=4)
        self.assertAlmostEqual(g.config.close_sign, -1.0)
        self.assertLess(g.config.pos_closed_rad, g.config.pos_open_rad)
        self.assertTrue(g.config.calibrated)

    def test_the_scale_uses_the_span_with_the_probe_inset(self):
        g, data = self._calibrate(reverse=True)
        self.assertGreater(data.travel_range, 0)
        self.assertAlmostEqual(
            g.config.rad_to_mm,
            (g.config.max_stroke_mm + GripperGeometry.STOP_INSET_MM)
            / data.travel_range,
            places=2)

    def test_calibrate_saves_automatically(self):
        g, fake = make_gripper(stops=True)
        with _home_for_calibrate() as home:
            g.calibrate()
            path = default_calibration_path(g.channel).replace(
                os.path.expanduser("~"), home, 1)
            self.assertTrue(os.path.isfile(path), path)
            with open(path) as f:
                self.assertTrue(json.load(f)["calibrated"])


class TestDerivedScale(unittest.TestCase):
    """``rad_to_mm`` is (max_stroke + probe inset) over the measured travel."""

    def test_the_reference_scale_is_the_hand_recorded_one(self):
        self.assertAlmostEqual(
            (GripperGeometry.JAW_TRAVEL_MM + GripperGeometry.STOP_INSET_MM)
            / 1.170748,
            73.45731105242119,
            places=9)

    def test_re_running_the_probe_on_a_shipped_unit_reproduces_its_scale(self):
        with open(_FACTORY_CALIB) as f:
            data = json.load(f)
        self.assertAlmostEqual(
            _scale_from_travel(GripperConfig(), data["travel_range_rad"]),
            data["rad_to_mm"], places=9)

    def test_deriving_from_the_jaw_travel_alone_would_be_short(self):
        with open(_FACTORY_CALIB) as f:
            travel_rad = json.load(f)["travel_range_rad"]
        short = GripperGeometry.JAW_TRAVEL_MM / travel_rad
        self.assertLess(short, 73.45731105242119)
        self.assertAlmostEqual(
            73.45731105242119 - short,
            GripperGeometry.STOP_INSET_MM / travel_rad,
            places=9)

    def test_a_coincident_travel_refuses_instead_of_inventing_a_scale(self):
        for travel in (0.0, -1.0):
            with self.assertRaises(RuntimeError) as ctx:
                _scale_from_travel(GripperConfig(), travel)
            self.assertIn("推不出", str(ctx.exception))

    def test_the_helper_uses_the_configured_stroke(self):
        cfg = GripperConfig()
        cfg.max_stroke_mm = 40.0
        self.assertAlmostEqual(
            _scale_from_travel(cfg, 0.5),
            (40.0 + GripperGeometry.STOP_INSET_MM) / 0.5)


class TestPerChannelPaths(unittest.TestCase):
    """One gripper per channel: per-channel file wins over the factory file."""

    def setUp(self):
        self.dir = tempfile.mkdtemp()
        self.addCleanup(shutil.rmtree, self.dir, True)

    def _seed(self, name, data):
        path = os.path.join(self.dir, name)
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "w") as f:
            json.dump(data, f)
        return path

    def test_default_path_is_per_channel(self):
        with _patched_calib_env(self.dir):
            self.assertTrue(default_calibration_path("can1").endswith(
                os.path.join(".litegrip", "can1_calibration.json")))

    def test_constructor_auto_loads_the_channel_file(self):
        self._seed(os.path.join(".litegrip", "can0_calibration.json"), {
            "channel": "can0", "calibrated": True,
            "zero_position_rad": 0.114, "max_position_rad": -1.491,
            "rad_to_mm": 74.8})
        with _patched_calib_env(self.dir):
            g = Gripper("can0")
        self.assertAlmostEqual(g.config.pos_closed_rad, 0.114)
        self.assertAlmostEqual(g.config.rad_to_mm, 74.8)

    def test_env_override_wins_over_the_channel_file(self):
        env_path = self._seed("env.json", {
            "channel": "can0", "calibrated": True,
            "zero_position_rad": 0.2, "max_position_rad": -1.4,
            "rad_to_mm": 70.0})
        self._seed(os.path.join(".litegrip", "can0_calibration.json"), {
            "channel": "can0", "calibrated": True,
            "zero_position_rad": 0.114, "max_position_rad": -1.491,
            "rad_to_mm": 74.8})
        with _patched_calib_env(self.dir):
            with mock.patch.dict(os.environ, {"LITEGRIP_CALIB": env_path}):
                g = Gripper("can0")
        self.assertAlmostEqual(g.config.pos_closed_rad, 0.2)
        self.assertAlmostEqual(g.config.rad_to_mm, 70.0)

    def test_no_channel_file_falls_back_to_the_factory(self):
        with _patched_calib_env(self.dir):
            g = Gripper("can1")
        with open(_FACTORY_CALIB) as f:
            factory = json.load(f)
        self.assertTrue(g.config.calibrated)
        self.assertAlmostEqual(g.config.pos_closed_rad,
                               factory["zero_position_rad"])
        self.assertAlmostEqual(g.config.rad_to_mm, factory["rad_to_mm"])

    def test_no_readable_file_returns_false(self):
        with _patched_calib_env(self.dir):
            g = Gripper("can0")
            with mock.patch.object(gripper_mod, "_FACTORY_CALIB",
                                   os.path.join(self.dir, "missing.json")):
                warnings = _capture_warnings(lambda: g.load_calibration())
        self.assertTrue(any("No calibration found" in m for m in warnings),
                        warnings)
        self.assertFalse(g.load_calibration(
            os.path.join(self.dir, "missing.json")))

    def test_an_unrecognized_name_is_not_a_candidate(self):
        weird = self._seed(os.path.join(".litegrip", "can0_08_calibration.json"),
                           {"channel": "can0", "calibrated": True,
                            "zero_position_rad": 0.114,
                            "max_position_rad": -1.491, "rad_to_mm": 63.42})
        with _patched_calib_env(self.dir):
            g = Gripper("can0")
            with open(_FACTORY_CALIB) as f:
                factory = json.load(f)
            self.assertAlmostEqual(g.config.rad_to_mm, factory["rad_to_mm"])
            # Naming it explicitly still works.
            self.assertTrue(g.load_calibration(weird))
            self.assertAlmostEqual(g.config.rad_to_mm, 63.42)

    def test_explicit_mismatch_still_warns_and_loads(self):
        path = self._seed("other.json", {
            "channel": "can1", "calibrated": True,
            "zero_position_rad": 0.1, "max_position_rad": -1.5,
            "rad_to_mm": 74.8})
        with _patched_calib_env(self.dir):
            g = Gripper("can0")
            warnings = _capture_warnings(lambda: g.load_calibration(path))
            self.assertTrue(any("channel" in m for m in warnings), warnings)
            self.assertTrue(g.config.calibrated)
            self.assertAlmostEqual(g.config.close_sign, 1.0)


class TestCalibrateCommandLeadIsBounded(unittest.TestCase):
    """The probe command may lead the measured position by one step at most."""

    def test_probe_never_leads_the_measured_position_by_more_than_one_step(self):
        step, kp = 0.1, 100.0
        g, fake = make_gripper(stops=True, start_rad=POS_OPEN_RAD)
        g.motion_config = MotionConfig(
            sleep_fn=lambda _: None,
            calib_kp=kp, calib_step_rad=step, calib_tau_limit=None,
            calib_stall_cycles=5, calib_max_iter=40)
        with _home_for_calibrate(), contextlib.redirect_stdout(io.StringIO()):
            g.calibrate()
        probe = [f for f in fake.frames if f.kp == kp]
        self.assertTrue(probe, "探测段一帧都没发？")
        worst = max(abs(f.q - f.pos_after) for f in probe)
        self.assertLessEqual(worst, step + 1e-9)


class TestCalibrateTorqueCeiling(unittest.TestCase):
    """The torque ceiling is an independent second guard.

    At a stop the structure yields (backlash, elastic deformation), so the
    encoder keeps reading movement and the position-stall test never fills its
    counter.  Only the torque ceiling can stop the probe.
    """

    CREEP = 0.01
    STEP = 0.1
    KP = 100.0

    def _calibrate(self, tau_limit, max_iter=40, stall_cycles=5):
        g, fake = make_gripper(stops=True, start_rad=POS_OPEN_RAD)
        g.motion_config = MotionConfig(
            sleep_fn=lambda _: None,
            calib_kp=self.KP, calib_step_rad=self.STEP,
            calib_stall_delta=0.0015, calib_stall_cycles=stall_cycles,
            calib_max_iter=max_iter, calib_tau_limit=tau_limit)
        creep, step = self.CREEP, self.STEP

        def soft_control_mit(q_target, kp, kd, dq_target=0.0,
                             tau_feedforward=0.0):
            m = fake.motor
            if (m.limit_hi is not None and q_target > m.limit_hi
                    and m.pos >= m.limit_hi - 1e-9):
                m.pos += creep * step
                m.tau = kp * (q_target - m.pos)
            else:
                m.step(q_target, kp, dq_target, tau_feedforward, DT)
            fake.frames.append(Frame(
                q=q_target, kp=kp, kd=kd, dq=dq_target,
                tau_ff=tau_feedforward, pos_after=m.pos, tau_nm=m.tau))
            return True

        fake.control_mit = soft_control_mit
        with _home_for_calibrate(), contextlib.redirect_stdout(io.StringIO()):
            return g.calibrate()

    def test_stall_counter_alone_would_not_stop_this_probe(self):
        data = self._calibrate(tau_limit=None)
        self.assertGreater(data.zero_position, POS_CLOSED_RAD + 0.5)

    def test_torque_ceiling_aborts_the_probe_the_counter_cannot(self):
        data = self._calibrate(tau_limit=2.0)
        self.assertAlmostEqual(data.zero_position, POS_CLOSED_RAD, delta=0.1)

    def test_ceiling_is_independent_of_the_stall_count(self):
        data = self._calibrate(tau_limit=2.0, max_iter=40,
                               stall_cycles=10 ** 6)
        self.assertAlmostEqual(data.zero_position, POS_CLOSED_RAD, delta=0.1)


class TestMasterId(unittest.TestCase):
    """``mst_id`` defaults to 0x18 and calibration files do not move it."""

    def setUp(self):
        self.dir = tempfile.mkdtemp()
        self.addCleanup(shutil.rmtree, self.dir, True)

    def test_default_is_0x18(self):
        g = Gripper("can1")
        self.assertEqual(g.mst_id, 0x18)

    def test_explicit_master_id_survives_auto_load(self):
        with open(_FACTORY_CALIB) as f:
            factory_declares = json.load(f).get("mst_id")
        g = Gripper("can1", mst_id=0x1A)
        self.assertEqual(g.mst_id, 0x1A)
        self.assertNotEqual(g.mst_id, factory_declares)

    def test_saving_records_the_identity(self):
        path = os.path.join(self.dir, "calib.json")
        g = Gripper("can1", can_id=0x0A, mst_id=0x1A)
        g.save_calibration(path)
        with open(path) as f:
            data = json.load(f)
        self.assertEqual(data["mst_id"], 0x1A)
        self.assertEqual(data["can_id"], 0x0A)


class TestLibraryDoesNotSilenceItsOwnLogs(unittest.TestCase):
    """The library installs no NullHandler: channel warnings must be visible."""

    def test_no_handler_is_installed_on_the_package_logger(self):
        self.assertEqual(logging.getLogger("litegrip").handlers, [])


if __name__ == "__main__":
    unittest.main()
