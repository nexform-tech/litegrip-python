"""标定持久化与方向的硬件无关测试：模板、往返、通道警告、home。"""

from __future__ import annotations

import contextlib
import io
import json
import logging
import os
import tempfile
import unittest

import _sdkpath  # noqa: F401
from litegrip import CALIB_TEMPLATES, LiteGrip
from litegrip.gripper import _FACTORY_CALIB

from fake_can import POS_CLOSED_RAD, POS_OPEN_RAD, make_gripper


def _capture_warnings(fn):
    """跑 ``fn()``，返回 litegrip logger 上的 WARNING 及以上文本。"""
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


def _write_json(data, directory):
    path = os.path.join(directory, "calib.json")
    with open(path, "w") as f:
        json.dump(data, f)
    return path


class TestDirectionTemplates(unittest.TestCase):
    """两份预制模板靠限位数值的**顺序**声明方向。"""

    def test_normal_template_closes_at_the_high_rad(self):
        g = LiteGrip("can0")
        self.assertTrue(g.load_calibration(CALIB_TEMPLATES["normal"]))
        self.assertTrue(g.config.calibrated)
        self.assertGreater(g.config.pos_closed_rad, g.config.pos_open_rad)
        self.assertAlmostEqual(g.config.close_sign, 1.0)

    def test_reverse_template_closes_at_the_low_rad(self):
        g = LiteGrip("can0")
        self.assertTrue(g.load_calibration(CALIB_TEMPLATES["reverse"]))
        self.assertTrue(g.config.calibrated)
        self.assertLess(g.config.pos_closed_rad, g.config.pos_open_rad)
        self.assertAlmostEqual(g.config.close_sign, -1.0)

    def test_both_templates_share_the_same_travel(self):
        a, b = LiteGrip("can0"), LiteGrip("can0")
        a.load_calibration(CALIB_TEMPLATES["normal"])
        b.load_calibration(CALIB_TEMPLATES["reverse"])
        self.assertAlmostEqual(a.config.rad_to_mm, b.config.rad_to_mm)
        self.assertAlmostEqual(
            abs(a.config.pos_closed_rad - a.config.pos_open_rad),
            abs(b.config.pos_closed_rad - b.config.pos_open_rad))


class TestPersistence(unittest.TestCase):
    def setUp(self):
        self.dir = tempfile.mkdtemp()
        self.addCleanup(lambda: os.path.exists(self.dir)
                        and __import__("shutil").rmtree(self.dir))

    def _path(self, name="calib.json"):
        return os.path.join(self.dir, name)

    def test_round_trip_preserves_direction(self):
        path = self._path()
        g = LiteGrip("can0")
        g.load_calibration(CALIB_TEMPLATES["reverse"])
        g.save_calibration(path)

        h = LiteGrip("can0")
        self.assertTrue(h.load_calibration(path))
        self.assertAlmostEqual(h.config.pos_closed_rad, g.config.pos_closed_rad)
        self.assertAlmostEqual(h.config.pos_open_rad, g.config.pos_open_rad)
        self.assertAlmostEqual(h.config.rad_to_mm, g.config.rad_to_mm)
        self.assertTrue(h.config.calibrated)
        self.assertAlmostEqual(h.config.close_sign, -1.0)

    def test_saved_file_records_calibrated(self):
        path = self._path()
        g = LiteGrip("can0")
        g.load_calibration(CALIB_TEMPLATES["normal"])
        g.save_calibration(path)
        with open(path) as f:
            self.assertTrue(json.load(f)["calibrated"])

    def test_old_file_without_the_flag_counts_as_calibrated(self):
        """工厂文件是 flag 之前的格式，缺键不能当未标定。"""
        with open(_FACTORY_CALIB) as f:
            self.assertNotIn("calibrated", json.load(f))
        g = LiteGrip("can0")
        self.assertTrue(g.load_calibration(_FACTORY_CALIB))
        self.assertTrue(g.config.calibrated)

    def test_channel_mismatch_warns(self):
        """通道是两台同 ID 夹爪的唯一身份键 —— 指错文件要看得见。"""
        path = _write_json({"channel": "can1", "zero_position_rad": 0.1,
                            "max_position_rad": -1.5, "rad_to_mm": 74.8},
                           self.dir)
        warnings = _capture_warnings(
            lambda: LiteGrip("can0").load_calibration(path))
        self.assertTrue(any("channel" in m for m in warnings), warnings)

    def test_matching_channel_is_silent(self):
        # assertNoLogs 要 3.10+，而本包声明 3.8 起，所以手动接一个 handler。
        path = _write_json({"channel": "can0", "zero_position_rad": 0.1,
                            "max_position_rad": -1.5, "rad_to_mm": 74.8},
                           self.dir)
        self.assertEqual(
            _capture_warnings(lambda: LiteGrip("can0").load_calibration(path)),
            [])

    def test_missing_required_key_raises(self):
        path = _write_json({"zero_position_rad": 0.1}, self.dir)
        with self.assertRaises(KeyError):
            LiteGrip("can0").load_calibration(path)


class TestCalibratePreservesDirection(unittest.TestCase):
    """标定**不能发现**方向（两端堵转现象一样），只能沿用已声明的方向。"""

    def _calibrate(self, reverse):
        g, fake = make_gripper(reverse=reverse, stops=True)
        with contextlib.redirect_stdout(io.StringIO()):
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
        # 反装的「闭合」在数值小的一端 —— 探测方向跟着翻
        self.assertAlmostEqual(data.zero_position, POS_OPEN_RAD, places=4)
        self.assertAlmostEqual(data.max_position, POS_CLOSED_RAD, places=4)
        self.assertAlmostEqual(g.config.close_sign, -1.0)
        self.assertLess(g.config.pos_closed_rad, g.config.pos_open_rad)
        self.assertTrue(g.config.calibrated)

    def test_travel_matches_the_template_after_calibration(self):
        g, data = self._calibrate(reverse=True)
        self.assertGreater(data.travel_range, 0)
        self.assertAlmostEqual(g.config.rad_to_mm,
                               g.config.max_stroke_mm / data.travel_range,
                               places=2)


class TestHomeDirection(unittest.TestCase):
    """``home()`` 走的是**本实例**标定出的闭合侧，不是模块常量。"""

    def _home_target(self, reverse):
        g, fake = make_gripper(reverse=reverse, stops=True)
        seen = []
        fake.control_mit_stream = (
            lambda q_target, **kw: seen.append(q_target) or True)
        g.home()
        return g, seen[0]

    def test_normal_home_goes_to_the_high_rad(self):
        g, target = self._home_target(reverse=False)
        self.assertAlmostEqual(target, POS_CLOSED_RAD)

    def test_reverse_home_goes_to_the_low_rad(self):
        g, target = self._home_target(reverse=True)
        self.assertAlmostEqual(target, POS_OPEN_RAD)
        self.assertAlmostEqual(target, g.config.pos_closed_rad)


if __name__ == "__main__":
    unittest.main()
