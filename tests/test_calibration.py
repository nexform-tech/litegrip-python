"""标定持久化与方向的硬件无关测试：模板、往返、通道警告、home。"""

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
from litegrip import (CALIB_TEMPLATES, LiteGrip, CommandError, GripperConfig,
                      default_calib_path, list_templates)
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


@contextlib.contextmanager
def _patched_calib_env(home, legacy=None):
    """把 ``HOME`` 与 ``DEFAULT_CALIB`` 都指到临时目录，屏蔽真机标定。

    ``LITEGRIP_CALIB`` 优先于 ``HOME``，所以一律清掉；``DEFAULT_CALIB`` 是
    导入期算好的常量，必须 patch 模块属性而不是环境变量。``patch.dict`` 会
    整份快照环境，退出时把清掉的那把键也一并还原。
    """
    target = legacy if legacy is not None else os.path.join(home, "missing.json")
    with mock.patch.dict(os.environ, {"HOME": home}, clear=False):
        os.environ.pop("LITEGRIP_CALIB", None)
        with mock.patch.object(gripper_mod, "DEFAULT_CALIB", target):
            yield


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


class TestTemplateSelection(unittest.TestCase):
    """按名字选模板：四个入口互相同义，装法读得回来。"""

    def test_list_templates_names_and_order(self):
        self.assertEqual(list_templates(), ["normal", "reverse"])

    def test_load_template_equals_load_calibration_kwarg(self):
        by_method = LiteGrip("can0")
        by_method.load_template("reverse")
        by_kwarg = LiteGrip("can0")
        by_kwarg.load_calibration(template="reverse")
        self.assertAlmostEqual(by_method.config.close_sign, -1.0)
        self.assertAlmostEqual(by_kwarg.config.close_sign, -1.0)

    def test_constructor_declares_the_mount(self):
        g = LiteGrip("can1", mount="reverse")
        self.assertEqual(g.mount, "reverse")
        self.assertEqual(g.config.mount, "reverse")

    def test_mount_is_none_until_calibrated(self):
        g = LiteGrip("can0")
        self.assertIsNone(g.mount)
        self.assertIsNone(g.config.mount)

    def test_mount_reads_back_each_template(self):
        normal, reverse = LiteGrip("can0"), LiteGrip("can0")
        normal.load_template("normal")
        reverse.load_template("reverse")
        self.assertEqual(normal.config.mount, "normal")
        self.assertEqual(reverse.config.mount, "reverse")

    def test_unknown_name_lists_the_valid_ones(self):
        with self.assertRaises(CommandError) as ctx:
            LiteGrip("can0").load_template("sideways")
        msg = str(ctx.exception)
        self.assertIn("normal", msg)
        self.assertIn("reverse", msg)

    def test_path_and_template_are_mutually_exclusive(self):
        with self.assertRaises(CommandError):
            LiteGrip("can0").load_calibration("/tmp/x.json", template="reverse")


class TestTemplateDoesNotClobber(unittest.TestCase):
    """模板只带方向与几何 —— 声明装法不能改写身份或调好的增益。"""

    def test_explicit_can_id_survives(self):
        self.assertEqual(LiteGrip("can1", can_id=0x0A, mount="reverse").can_id,
                         0x0A)

    def test_auto_master_id_is_not_pinned(self):
        """``mst_id=None`` 仍走自动探测；模板里没有这个键就不该被写死。"""
        self.assertIsNone(LiteGrip("can1", mount="reverse").mst_id)

    def test_tuned_gains_survive(self):
        cfg = GripperConfig()
        cfg.kp, cfg.kd = 321.0, 4.5
        g = LiteGrip("can1", config=cfg, mount="reverse")
        self.assertAlmostEqual(g.config.kp, 321.0)
        self.assertAlmostEqual(g.config.kd, 4.5)

    def test_unreadable_template_is_strict(self):
        """模板读不出来时**不得**回退到正装的出厂文件。"""
        with mock.patch.dict(CALIB_TEMPLATES,
                             {"reverse": os.path.join("no", "such.json")}):
            g = LiteGrip("can0")
            with self.assertRaises(CommandError):
                g.load_template("reverse")
        self.assertFalse(g.config.calibrated)


class TestPerChannelPaths(unittest.TestCase):
    """一台电脑两台夹爪：标定各存各的，自动加载按通道认领。"""

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
            self.assertTrue(default_calib_path("can1").endswith(
                os.path.join(".litegrip", "can1_calibration.json")))

    def test_env_override_wins_over_the_channel_file(self):
        with _patched_calib_env(self.dir):
            with mock.patch.dict(os.environ, {"LITEGRIP_CALIB": "/tmp/x.json"}):
                self.assertEqual(default_calib_path("can1"), "/tmp/x.json")

    def test_no_arg_save_lands_in_the_channel_file(self):
        with _patched_calib_env(self.dir):
            g = LiteGrip("can1")
            g.load_template("reverse")
            written = g.save_calibration()
            self.assertTrue(written.endswith("can1_calibration.json"))
            self.assertTrue(os.path.exists(written))

    def test_auto_load_prefers_the_channel_file_over_the_legacy_one(self):
        legacy = self._seed("legacy.json", {
            "channel": "can1", "calibrated": True,
            "zero_position_rad": 0.114, "max_position_rad": -1.491,
            "rad_to_mm": 74.8})
        with _patched_calib_env(self.dir, legacy=legacy):
            self._seed(os.path.join(".litegrip", "can1_calibration.json"), {
                "channel": "can1", "calibrated": True,
                "zero_position_rad": -1.491, "max_position_rad": 0.114,
                "rad_to_mm": 74.8})
            g = LiteGrip("can1")
            self.assertTrue(g.load_calibration())
            self.assertAlmostEqual(g.config.close_sign, -1.0)

    def test_auto_load_skips_another_channels_file(self):
        can0_file = self._seed("legacy.json", {
            "channel": "can0", "calibrated": True,
            "zero_position_rad": 0.114, "max_position_rad": -1.491,
            "rad_to_mm": 74.8})
        with _patched_calib_env(self.dir, legacy=can0_file):
            self.assertFalse(LiteGrip("can1").load_calibration())

    def test_isolated_home_finds_nothing_and_says_so(self):
        # ~/.litegrip/litegrip_calibration.json 在真机上真实存在，这条确保
        # 测试不会悄悄读到它。
        with _patched_calib_env(self.dir):
            g = LiteGrip("can1")
            warnings = _capture_warnings(lambda: g.load_calibration())
            self.assertTrue(any("No calibration found" in m for m in warnings),
                            warnings)
            self.assertFalse(g.config.calibrated)

    def test_explicit_mismatch_still_warns_and_loads(self):
        """显式指定文件是调用方的主动覆盖，语义仍是「警告但加载」。"""
        path = self._seed("other.json", {
            "channel": "can1", "calibrated": True,
            "zero_position_rad": 0.1, "max_position_rad": -1.5,
            "rad_to_mm": 74.8})
        with _patched_calib_env(self.dir):
            g = LiteGrip("can0")
            warnings = _capture_warnings(lambda: g.load_calibration(path))
            self.assertTrue(any("channel" in m for m in warnings), warnings)
            self.assertTrue(g.config.calibrated)
            self.assertAlmostEqual(g.config.close_sign, 1.0)


if __name__ == "__main__":
    unittest.main()
