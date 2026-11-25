"""actions 引擎的硬件无关测试：斜坡、领先封顶、堵转窗口、保力、使能。"""

from __future__ import annotations

import threading
import time
import unittest
from unittest import mock

import _sdkpath  # noqa: F401
from litegrip import (CommError, CommandError, Gripper, GripperConfig,
                      GripperState, MotionConfig, UnitConversion)
from litegrip.actions import (GraspResult, GripperActions, MoveResult,
                              ReleaseResult, clamp_to_travel,
                              closing_travel_excursion_rad,
                              force_approach_terms, limit_target,
                              press_target, work_limit_target)

from fake_can import (DT, POS_CLOSED_RAD, POS_OPEN_RAD, RAD_TO_MM,
                      FakeCanBus, make_gripper, planned_steps, tick_clock)

SPEED_MM_S = 50.0
SPEED_RAD_S = SPEED_MM_S / RAD_TO_MM          # ≈ 0.674 rad/s
TRAVEL_RAD = abs(POS_CLOSED_RAD - POS_OPEN_RAD)


def press_caps(g):
    """引擎算出来的 ``(行进段领先上限, 压紧段领先上限)`` rad。"""
    cfg = g.motion_config
    min_cap = SPEED_RAD_S * cfg.frame_interval
    return (max(cfg.max_lead_mm / RAD_TO_MM, min_cap),
            max(cfg.stop_lead_mm / RAD_TO_MM, min_cap))


def _calibrated_config(reverse: bool = False) -> GripperConfig:
    """一份标定好的配置，只用来算几何 —— 不建夹爪、不发帧。"""
    cfg = GripperConfig(calibrated=True)
    if reverse:
        cfg.pos_closed_rad, cfg.pos_open_rad = POS_OPEN_RAD, POS_CLOSED_RAD
    else:
        cfg.pos_closed_rad, cfg.pos_open_rad = POS_CLOSED_RAD, POS_OPEN_RAD
    cfg.rad_to_mm = RAD_TO_MM
    return cfg


class TestLimitTarget(unittest.TestCase):
    """1 / 2：目标位置与位置约定校验。"""

    def test_margin_on_both_ends(self):
        g, _ = make_gripper()
        travel = TRAVEL_RAD

        target, limit, margin_rad, got_travel = limit_target(
            g.config, "close", 0.05)
        self.assertAlmostEqual(target, POS_CLOSED_RAD - 0.05 * travel)
        self.assertAlmostEqual(limit, POS_CLOSED_RAD)
        self.assertAlmostEqual(margin_rad, 0.05 * travel)
        self.assertAlmostEqual(got_travel, travel)

        target, limit, _, _ = limit_target(g.config, "open", 0.05)
        self.assertAlmostEqual(target, POS_OPEN_RAD + 0.05 * travel)
        self.assertAlmostEqual(limit, POS_OPEN_RAD)

    def test_reverse_mount_mirrors_the_margin(self):
        """反装合法：余量朝另一侧留，目标仍然落在限位内侧。"""
        g, _ = make_gripper(reverse=True)
        travel = TRAVEL_RAD

        target, limit, margin_rad, _ = limit_target(g.config, "close", 0.05)
        self.assertAlmostEqual(limit, POS_OPEN_RAD)        # 反装时闭合在数值小的一侧
        self.assertAlmostEqual(margin_rad, 0.05 * travel)
        self.assertAlmostEqual(target, POS_OPEN_RAD + 0.05 * travel)
        # 比限位更靠近行程内侧
        self.assertGreater(target, limit)

        target, limit, _, _ = limit_target(g.config, "open", 0.05)
        self.assertAlmostEqual(limit, POS_CLOSED_RAD)
        self.assertAlmostEqual(target, POS_CLOSED_RAD - 0.05 * travel)

    def test_uncalibrated_raises(self):
        """没标定的配置里所有方向都是猜的 —— 拦住。"""
        g, _ = make_gripper()
        g.config.calibrated = False
        with self.assertRaises(CommandError):
            limit_target(g.config, "close", 0.05)

    def test_zero_travel_raises(self):
        g, _ = make_gripper()
        g.config.pos_open_rad = g.config.pos_closed_rad
        with self.assertRaises(CommandError):
            limit_target(g.config, "close", 0.05)


class TestPressTarget(unittest.TestCase):
    """open/close 的目标在标定限位**外侧**，不是内侧。"""

    def test_target_overshoots_the_limit(self):
        g, _ = make_gripper()

        target, limit, over_rad, travel = press_target(g.config, "close", 0.05)
        self.assertAlmostEqual(limit, POS_CLOSED_RAD)
        self.assertAlmostEqual(travel, TRAVEL_RAD)
        self.assertAlmostEqual(over_rad, 0.05 * TRAVEL_RAD)
        # 越过去，而不是退回来 —— 与 limit_target 方向相反
        self.assertGreater(target, limit)
        self.assertAlmostEqual(target - limit, 0.05 * TRAVEL_RAD)

        target, limit, _, _ = press_target(g.config, "open", 0.05)
        self.assertAlmostEqual(limit, POS_OPEN_RAD)
        self.assertLess(target, limit)

    def test_reverse_mount_overshoots_the_other_way(self):
        g, _ = make_gripper(reverse=True)

        target, limit, over_rad, travel = press_target(g.config, "close", 0.05)
        self.assertAlmostEqual(limit, POS_OPEN_RAD)
        self.assertAlmostEqual(travel, TRAVEL_RAD)
        self.assertAlmostEqual(over_rad, 0.05 * TRAVEL_RAD)
        self.assertLess(target, limit)          # 反装时越位朝数值更小的一侧
        self.assertAlmostEqual(limit - target, 0.05 * TRAVEL_RAD)

    def test_uncalibrated_raises(self):
        g, _ = make_gripper()
        g.config.calibrated = False
        with self.assertRaises(CommandError):
            press_target(g.config, "close", 0.05)


class TestClosingTravelExcursion(unittest.TestCase):
    """力控段护栏的判据：越过**闭合侧**限位、朝闭合方向又多走了多少。

    只判闭合侧是有意的 —— 前馈力矩朝那边压，只有那边越过去才叫失控；越过张开
    限位会被同一份力矩自己推回来。
    """

    def test_inside_the_travel_is_zero(self):
        cfg = _calibrated_config()
        for q in (POS_OPEN_RAD, POS_OPEN_RAD + 0.5, POS_CLOSED_RAD, 0.0):
            self.assertEqual(closing_travel_excursion_rad(cfg, q), 0.0, q)

    def test_past_the_closed_limit_is_the_distance(self):
        self.assertAlmostEqual(
            closing_travel_excursion_rad(_calibrated_config(),
                                         POS_CLOSED_RAD + 0.03),
            0.03, places=12)

    def test_past_the_open_limit_is_not_an_excursion(self):
        # 那边不算：力矩会把它推回来，报出来只会让正常操作误触发
        self.assertEqual(
            closing_travel_excursion_rad(_calibrated_config(),
                                         POS_OPEN_RAD - 5.0), 0.0)

    def test_reverse_mount_mirrors_the_direction(self):
        rev = _calibrated_config(reverse=True)
        self.assertLess(rev.close_sign, 0.0)               # 确认真是反装
        self.assertEqual(closing_travel_excursion_rad(rev, POS_CLOSED_RAD), 0.0)
        self.assertEqual(closing_travel_excursion_rad(rev, POS_OPEN_RAD + 5.0),
                         0.0)
        self.assertAlmostEqual(
            closing_travel_excursion_rad(rev, POS_OPEN_RAD - 0.03), 0.03,
            places=12)


class TestRamp(unittest.TestCase):
    """3 / 4 / 5：空载斜坡的形状（顶限位）。"""

    def test_close_presses_onto_the_stop(self):
        g, fake = make_gripper(stops=True)
        self.assertIsInstance(g._actions, GripperActions)
        res = g._actions.close(SPEED_MM_S)

        self.assertTrue(res.ok, res)
        self.assertTrue(res.stalled, res)             # 成功 = 撞到限位
        self.assertFalse(res.reached, res)            # 目标在限位外侧，到不了
        self.assertAlmostEqual(res.limit_rad, POS_CLOSED_RAD)
        self.assertLess(abs(res.state.position_rad - POS_CLOSED_RAD),
                        g.motion_config.stop_tol)

    def test_open_presses_onto_the_other_stop(self):
        g, fake = make_gripper(start_rad=POS_CLOSED_RAD, stops=True)
        res = g._actions.open(SPEED_MM_S)

        self.assertTrue(res.ok, res)
        self.assertAlmostEqual(res.limit_rad, POS_OPEN_RAD)
        self.assertLess(abs(res.state.position_rad - POS_OPEN_RAD),
                        g.motion_config.stop_tol)
        self.assertAlmostEqual(
            res.target_rad, POS_OPEN_RAD - 0.05 * TRAVEL_RAD)
        self.assertAlmostEqual(fake.frames[0].dq, -SPEED_RAD_S)  # 张开 = -rad

    def test_stall_ends_the_move_before_the_settle_is_over(self):
        g, fake = make_gripper(stops=True)
        _, ramp_steps, _, total = planned_steps(g, SPEED_MM_S, press=True)
        res = g._actions.close(SPEED_MM_S)

        self.assertEqual(res.steps, len(fake.frames))
        self.assertGreaterEqual(res.steps, ramp_steps)   # 没在半路停
        self.assertLess(res.steps, total)                # 也没白跑完保压段

    def test_velocity_feedforward_only_during_ramp(self):
        g, fake = make_gripper(stops=True)
        _, ramp_steps, _, _ = planned_steps(g, SPEED_MM_S, press=True)
        g._actions.close(SPEED_MM_S)

        self.assertAlmostEqual(fake.frames[0].dq, SPEED_RAD_S)   # 闭合 = +rad
        self.assertAlmostEqual(fake.frames[ramp_steps - 1].dq, SPEED_RAD_S)
        for f in fake.frames[ramp_steps:]:
            self.assertEqual(f.dq, 0.0)


class TestReverseMount(unittest.TestCase):
    """反装（``pos_closed_rad < pos_open_rad``）：同一个动作走相反方向。"""

    def test_close_presses_onto_the_low_rad_stop(self):
        g, fake = make_gripper(reverse=True, stops=True)
        res = g._actions.close(SPEED_MM_S)

        self.assertTrue(res.ok, res)
        self.assertTrue(res.stalled, res)
        self.assertAlmostEqual(res.limit_rad, POS_OPEN_RAD)   # 闭合在数值小的一侧
        self.assertLess(abs(res.state.position_rad - POS_OPEN_RAD),
                        g.motion_config.stop_tol)
        self.assertAlmostEqual(fake.frames[0].dq, -SPEED_RAD_S)  # 闭合 = -rad

    def test_open_presses_onto_the_high_rad_stop(self):
        g, fake = make_gripper(reverse=True, stops=True)
        res = g._actions.open(SPEED_MM_S)

        self.assertTrue(res.ok, res)
        self.assertAlmostEqual(res.limit_rad, POS_CLOSED_RAD)
        self.assertLess(abs(res.state.position_rad - POS_CLOSED_RAD),
                        g.motion_config.stop_tol)
        self.assertAlmostEqual(fake.frames[0].dq, SPEED_RAD_S)   # 张开 = +rad

    def test_position_mm_reads_zero_closed_full_stroke_open(self):
        g, fake = make_gripper(reverse=True)
        s = g.config.close_sign
        self.assertAlmostEqual(s, -1.0)

        fake.motor.pos = g.config.pos_closed_rad
        self.assertAlmostEqual(g.get_state().position_mm, 0.0, places=6)

        fake.motor.pos = g.config.pos_open_rad
        self.assertAlmostEqual(g.get_state().position_mm,
                               TRAVEL_RAD * RAD_TO_MM, places=4)

    def test_a_squeeze_reports_positive_force_on_both_mounts(self):
        """夹紧时反装的力矩符号相反，但换算出来的力都该是正的。"""
        g, fake = make_gripper()
        fake.motor.tau = +0.5                 # 正装：夹紧 = +力矩
        self.assertAlmostEqual(g.get_state().force_n, +5.0, places=9)

        g, fake = make_gripper(reverse=True)
        fake.motor.tau = -0.5                 # 反装：夹紧 = -力矩
        self.assertAlmostEqual(g.get_state().force_n, +5.0, places=9)

    def test_set_motor_angle_clamps_between_the_two_limits(self):
        """越界目标夹到标定限位 —— 哪个数值算哪一端由装法决定。

        ``set_motor_angle`` 走引擎斜坡，目标先经 :func:`clamp_to_travel` 再
        排程，所以下发的帧没有一帧越过标定行程。
        """
        g, fake = make_gripper(reverse=True)
        lo, hi = POS_OPEN_RAD, POS_CLOSED_RAD

        self.assertTrue(g.set_motor_angle(hi + 5.0))
        self.assertTrue(g.set_motor_angle(lo - 5.0))

        self.assertLessEqual(max(f.q for f in fake.frames), hi)
        self.assertGreaterEqual(min(f.q for f in fake.frames), lo)
        self.assertAlmostEqual(fake.motor.pos, lo, delta=0.05)

    def test_close_sign_follows_the_ordering(self):
        g, _ = make_gripper()
        self.assertAlmostEqual(g.config.close_sign, 1.0)
        g.config.pos_closed_rad, g.config.pos_open_rad = POS_OPEN_RAD, POS_CLOSED_RAD
        self.assertAlmostEqual(g.config.close_sign, -1.0)


class TestLeadCapAndStall(unittest.TestCase):
    """6 / 7：被挡住时指令有界、力矩有界，并判到堵转。"""

    def test_lead_cap_bounds_command_and_torque(self):
        # 挡在行程刚起步处（窗口很快填满），目标还在远处
        block = POS_OPEN_RAD + 0.01
        g, fake = make_gripper(block_rad=block)
        res = g._actions.close(SPEED_MM_S)

        travel_cap, _ = press_caps(g)
        leads = [abs(f.q - f.pos_after) for f in fake.frames]
        self.assertLessEqual(max(leads), travel_cap + 1e-9)
        self.assertAlmostEqual(max(leads), travel_cap, places=6)

        # 被挡住后力矩正好是 kp × 领先上限，且远小于协议上限 10 Nm
        max_tau = max(abs(f.tau_nm) for f in fake.frames)
        self.assertAlmostEqual(max_tau, g.config.kp * travel_cap, places=4)
        self.assertLess(max_tau, 10.0)

        self.assertTrue(res.stalled, res)
        self.assertFalse(res.reached, res)
        # 堵在离限位很远的地方 = 半路被挡，不算「顶到限位」
        self.assertFalse(res.ok, res)

    def test_mid_ramp_stall_stops_early(self):
        block = (POS_OPEN_RAD + POS_CLOSED_RAD) / 2
        g, fake = make_gripper(block_rad=block)
        _, _, _, total = planned_steps(g, SPEED_MM_S, press=True)
        res = g._actions.close(SPEED_MM_S)

        self.assertTrue(res.stalled, res)
        self.assertFalse(res.ok, res)
        self.assertLess(res.steps, total)
        self.assertAlmostEqual(fake.motor.pos, block, places=6)


def frame_leads(fake, start_rad):
    """每一帧下发的指令相对**下发前**实测位置的领先量 rad。

    引擎的领先上限是相对它下发前读到的那一帧位置算的，所以基准是上一帧的
    ``pos_after``（第一帧的基准是起点），不是这一帧自己的。
    """
    prev = start_rad
    leads = []
    for f in fake.frames:
        leads.append(f.q - prev)
        prev = f.pos_after
    return leads


class TestRampedPositioning(unittest.TestCase):
    """Absolute positioning goes through the engine ramp — the 2026-10-12
    "large oscillation" mechanism lives here.

    The old hard step put the whole target in the first frame with ``dq=0`` and
    ``tau_ff=0``: kp=100 Nm/rad against a 0.54 rad error is 54 Nm, while the
    DM4310 frame range is ±10 Nm — the saturated frame overshoots and the next
    one pulls back.  ``set_gripper_distance`` / ``set_motor_angle`` use the same
    ramp as open/close, sharing ``max_lead_mm``.
    """

    TARGET_MM = 40.0
    SPEED_MM_S = 40.0

    def _ramped(self, speed=None):
        """From the open side toward 40 mm, return ``(g, fake, target_rad)``."""
        g, fake = make_gripper()
        target_rad = g.config.pos_closed_rad - self.TARGET_MM / RAD_TO_MM
        ok = g.set_gripper_distance(
            self.TARGET_MM,
            self.SPEED_MM_S if speed is None else speed)
        return g, fake, ok, target_rad

    def test_the_scenario_is_one_that_used_to_saturate(self):
        g, _ = make_gripper()
        target_rad = g.config.pos_closed_rad - self.TARGET_MM / RAD_TO_MM
        jump_tau = g.config.kp * abs(target_rad - g.config.pos_open_rad)
        cap_rad = g.motion_config.max_lead_mm / RAD_TO_MM

        self.assertGreater(jump_tau, 10.0)
        self.assertLess(g.config.kp * cap_rad, 10.0)

    def test_the_first_frame_is_a_ramp_step_not_a_jump_to_the_target(self):
        g, fake, ok, target_rad = self._ramped()
        start = g.config.pos_open_rad
        first = fake.frames[0]
        travel_rad = abs(target_rad - start)

        self.assertTrue(ok)
        self.assertLess(abs(first.q - start), 0.02 * travel_rad,
                        "第一帧就贴到目标上 —— 那就是阶跃，不是斜坡")
        self.assertGreater(abs(target_rad - first.q), 0.98 * travel_rad,
                           "第一帧已经走掉大半路程 —— 不是斜坡")
        self.assertGreater(first.dq, 0.0)      # 闭合方向 = +rad，带速度前馈

    def test_no_frame_leads_the_jaws_more_than_max_lead_mm(self):
        g, fake, ok, _ = self._ramped()
        cap_rad = g.motion_config.max_lead_mm / RAD_TO_MM

        worst = max(abs(lead) for lead in
                    frame_leads(fake, g.config.pos_open_rad))

        self.assertTrue(ok)
        self.assertLessEqual(worst, cap_rad + 1e-9,
                             "有一帧的指令领先超过了领先上限 —— 领先就是力矩")
        self.assertLess(max(abs(f.tau_nm) for f in fake.frames),
                        fake.motor.TAU_MAX)

    def test_the_ramp_reaches_the_target_and_returns_true(self):
        g, fake, ok, target_rad = self._ramped()

        self.assertTrue(ok)
        self.assertAlmostEqual(fake.motor.pos, target_rad, places=3)
        self.assertGreater(len(fake.frames), 1)

    def test_set_motor_angle_takes_the_same_ramp(self):
        a, fake_a, ok_a, target_rad = self._ramped()
        b, fake_b = make_gripper()
        ok_b = b.set_motor_angle(target_rad, self.SPEED_MM_S)

        self.assertTrue(ok_a)
        self.assertTrue(ok_b)
        self.assertEqual([f.q for f in fake_a.frames],
                         [f.q for f in fake_b.frames])

    def test_blocked_halfway_returns_false_and_takes_the_force_off(self):
        """半路被挡住：返回 False，并且**撤力**。

        引擎在 ``press=False`` 下判到堵转就 ``break``，最后一帧指令停在
        「实测位置 + 领先上限」上 —— 留着它，驱动器会按 kp × 上限（≈6.6 Nm）
        一直压着工件。定位到行程内的某点压着不是意图，所以 ``ramp_to`` 补了撤力。
        """
        block = POS_OPEN_RAD + 0.03          # 刚起步就顶住，堵转窗口来得及填满
        g, fake = make_gripper(block_rad=block)
        ok = g.set_gripper_distance(self.TARGET_MM, self.SPEED_MM_S)

        self.assertFalse(ok)
        self.assertAlmostEqual(fake.motor.pos, block, places=6)

        off = [f for f in fake.frames if f.kp == 0.0]
        self.assertTrue(off, "报堵转却没有撤力帧 —— 驱动器还在压着")
        for f in off:
            self.assertEqual(f.kd, 0.0)
            self.assertEqual(f.tau_ff, 0.0)
            self.assertAlmostEqual(f.q, f.pos_after, places=6)

    def test_a_target_the_jaws_are_almost_on_is_not_a_stall(self):
        """差一点点（不是一点都不差）也是到位，不是堵转。"""
        g, fake = make_gripper(start_rad=POS_OPEN_RAD)
        here_mm = TRAVEL_RAD * RAD_TO_MM
        target_mm = here_mm - 0.3          # 差 0.3 mm，远在 reach_tol 之内

        self.assertTrue(g.set_gripper_distance(target_mm, self.SPEED_MM_S))
        target_rad = g.config.pos_closed_rad - target_mm / RAD_TO_MM
        self.assertAlmostEqual(fake.motor.pos, target_rad, places=3)

    def test_the_speed_parameter_chooses_the_ramp_length(self):
        _g_fast, fake_fast, ok_fast, _ = self._ramped(speed=80.0)
        _g_slow, fake_slow, ok_slow, _ = self._ramped(speed=40.0)

        self.assertTrue(ok_fast)
        self.assertTrue(ok_slow)
        self.assertLess(len(fake_fast.frames), len(fake_slow.frames))

    def test_omitted_speed_uses_the_motion_config_default(self):
        by_default, fake_default = make_gripper()
        ok_default = by_default.set_gripper_distance(self.TARGET_MM)
        explicit, fake_explicit = make_gripper()
        ok_explicit = explicit.set_gripper_distance(
            self.TARGET_MM, by_default.motion_config.speed_mm_s)

        self.assertTrue(ok_default)
        self.assertTrue(ok_explicit)
        self.assertEqual([f.q for f in fake_default.frames],
                         [f.q for f in fake_explicit.frames])


class TestPositioningRefusesInvalidArguments(unittest.TestCase):
    """Invalid targets/speeds are refused with a cause — and send no frames."""

    NON_FINITE = (float("nan"), float("inf"), float("-inf"))

    def test_non_finite_distance_is_refused(self):
        for bad in self.NON_FINITE:
            g, fake = make_gripper()
            with self.assertRaises(CommandError):
                g.set_gripper_distance(bad)
            self.assertEqual(fake.frames, [], f"distance={bad!r} 竟然发帧了")

    def test_non_finite_angle_is_refused(self):
        for bad in self.NON_FINITE:
            g, fake = make_gripper()
            with self.assertRaises(CommandError):
                g.set_motor_angle(bad)
            self.assertEqual(fake.frames, [])

    def test_non_positive_or_non_finite_speed_is_refused(self):
        for bad in (0.0, -30.0, float("nan"), float("inf")):
            g, fake = make_gripper()
            with self.assertRaises(CommandError) as ctx:
                g.set_gripper_distance(40.0, bad)
            self.assertIn("速度", str(ctx.exception))
            self.assertEqual(fake.frames, [], "拒绝了却还是发了帧")


class TestPressZone(unittest.TestCase):
    """顶限位的力矩是**有界**的：接近段收窄到 stop_lead_mm，指令封顶在标定限位。"""

    def test_pressing_torque_is_under_the_rating(self):
        g, fake = make_gripper(stops=True)
        res = g._actions.close(SPEED_MM_S)
        _, stop_cap = press_caps(g)

        max_tau = max(abs(f.tau_nm) for f in fake.frames)
        self.assertLessEqual(max_tau, g.config.kp * stop_cap + 1e-9)
        self.assertLess(max_tau, 3.0)                 # DM4310 额定 3 Nm
        self.assertTrue(res.ok, res)
        # 力矩有界靠的是两条一起：接近段收窄 + 指令封顶在标定行程内
        self.assertFalse([f.q for f in fake.frames
                          if f.q > POS_CLOSED_RAD + 1e-9
                          or f.q < POS_OPEN_RAD - 1e-9])

    def test_default_press_force_is_the_gentle_tier(self):
        """默认 0.7 mm 接近段：上界 kp × 0.7 / rad_to_mm ≈ 0.94 Nm；
        顶住限位后的残余力矩是 0 —— 指令封顶在限位，没有越位差可压。"""
        g, fake = make_gripper(stops=True)
        g._actions.close(SPEED_MM_S)

        self.assertAlmostEqual(g.motion_config.stop_lead_mm, 0.7)
        bound = g.config.kp * 0.7 / g.config.rad_to_mm
        max_tau = max(abs(f.tau_nm) for f in fake.frames)
        self.assertLessEqual(max_tau, bound + 1e-9)
        self.assertLess(bound, 1.2)                   # 「不太猛」的量化断言
        self.assertAlmostEqual(fake.frames[-1].tau_nm, 0.0, places=6)
        # 没跌到一帧位移的地板（speed × frame_interval = 0.25 mm）以下
        floor_mm = SPEED_MM_S * g.motion_config.frame_interval
        self.assertGreater(g.motion_config.stop_lead_mm, floor_mm)

    def test_travel_phase_uses_the_wider_cap(self):
        # 同一个默认配置：半路被挡 → max_lead_mm；顶到限位 → stop_lead_mm
        g_stop, fake_stop = make_gripper(stops=True)
        g_stop.close(SPEED_MM_S)
        tau_press = max(abs(f.tau_nm) for f in fake_stop.frames)

        g_block, fake_block = make_gripper(block_rad=POS_OPEN_RAD + 0.01)
        g_block.close(SPEED_MM_S)
        tau_travel = max(abs(f.tau_nm) for f in fake_block.frames)

        self.assertLess(tau_press, tau_travel)

    def test_settle_phase_command_is_capped_too(self):
        # 保压段的指令在限位外侧：不切上限的话领先会涨到 kp × 越位量
        g, fake = make_gripper(stops=True)
        _, ramp_steps, _, _ = planned_steps(g, SPEED_MM_S, press=True)
        g._actions.close(SPEED_MM_S)

        _, stop_cap = press_caps(g)
        tail = fake.frames[ramp_steps:]
        self.assertTrue(tail)                          # 确实进了保压段
        self.assertLessEqual(
            max(abs(f.q - f.pos_after) for f in tail), stop_cap + 1e-9)


class TestClampToTravel(unittest.TestCase):
    """指令位置夹进标定行程，正装反装通用。"""

    def test_values_inside_the_range_are_untouched(self):
        g, _ = make_gripper()
        mid = (POS_CLOSED_RAD + POS_OPEN_RAD) / 2
        self.assertAlmostEqual(clamp_to_travel(g.config, mid), mid)

    def test_outside_values_snap_to_the_endpoints_both_mounts(self):
        for reverse in (False, True):
            with self.subTest(reverse=reverse):
                g, _ = make_gripper(reverse=reverse)
                self.assertAlmostEqual(
                    clamp_to_travel(g.config, POS_CLOSED_RAD + 1.0),
                    POS_CLOSED_RAD)
                self.assertAlmostEqual(
                    clamp_to_travel(g.config, POS_OPEN_RAD - 1.0),
                    POS_OPEN_RAD)


class TestCommandClampedToCalibratedTravel(unittest.TestCase):
    """``open`` / ``close`` 的**指令**不许越过标定行程 —— 标定偏了也一样。

    目标是越过标定限位的（``press_target``）那一套仍然成立，但每一帧下发的指令
    都封顶在标定行程上，所以真限位不在标定位置上、甚至根本没有限位，夹爪也不会
    被驱动到标定行程之外。
    """

    LO = POS_OPEN_RAD                            # 数值小的一端
    HI = POS_CLOSED_RAD                          # 数值大的一端

    def _outside(self, fake):
        return [f.q for f in fake.frames
                if f.q > self.HI + 1e-9 or f.q < self.LO - 1e-9]

    def test_close_saturates_at_the_calibrated_limit(self):
        g, fake = make_gripper(stops=True)
        res = g._actions.close(SPEED_MM_S)

        self.assertEqual(self._outside(fake), [])
        self.assertAlmostEqual(max(f.q for f in fake.frames), POS_CLOSED_RAD,
                               places=9)
        self.assertTrue(res.ok, res)

    def test_open_saturates_at_the_calibrated_limit(self):
        g, fake = make_gripper(start_rad=POS_CLOSED_RAD, stops=True)
        res = g._actions.open(SPEED_MM_S)

        self.assertEqual(self._outside(fake), [])
        self.assertAlmostEqual(min(f.q for f in fake.frames), POS_OPEN_RAD,
                               places=9)
        self.assertTrue(res.ok, res)

    def test_no_stop_at_all_still_cannot_walk_past_the_limit(self):
        """真限位缺席（机械限位已经损坏）时，指令封顶是唯一的护栏。

        夹爪一路跟到标定限位就停在那里（引擎自己的「停在限位上」口径
        ``stop_tol``），不会像越位目标那样继续走下去。
        """
        g, fake = make_gripper(stops=False)
        g._actions.close(SPEED_MM_S)

        self.assertEqual(self._outside(fake), [])
        self.assertLess(abs(fake.motor.pos - POS_CLOSED_RAD),
                        g.motion_config.stop_tol)

    def test_real_stop_beyond_the_calibration_stops_at_the_calibration(self):
        g, fake = make_gripper(stops=True)
        fake.motor.limit_hi = POS_CLOSED_RAD + 0.5      # 真限位在标定外侧

        g._actions.close(SPEED_MM_S)

        self.assertEqual(self._outside(fake), [])
        self.assertLess(abs(fake.motor.pos - POS_CLOSED_RAD),
                        g.motion_config.stop_tol)

    def test_real_stop_inside_the_calibration_is_not_pushed_through(self):
        """真限位在标定内侧时，夹爪停在真限位上 —— 封顶不能抹掉「被挡住」这个
        信号：离标定限位还很远就停住，仍然报堵转、不报成功。"""
        g, fake = make_gripper(stops=True)
        fake.motor.limit_hi = POS_CLOSED_RAD - 0.3      # 真限位在标定内侧

        res = g._actions.close(SPEED_MM_S)

        self.assertEqual(self._outside(fake), [])
        self.assertTrue(res.stalled, res)
        self.assertFalse(res.ok, res)
        self.assertGreater(abs(res.state.position_rad - POS_CLOSED_RAD),
                           g.motion_config.stop_tol)

    def test_reverse_mount_clamps_the_other_end(self):
        g, fake = make_gripper(reverse=True, stops=True)
        g._actions.close(SPEED_MM_S)

        self.assertEqual(self._outside(fake), [])
        self.assertAlmostEqual(min(f.q for f in fake.frames), POS_OPEN_RAD,
                               places=9)

    def test_final_cmd_and_progress_report_the_emitted_value(self):
        seen = []
        g, fake = make_gripper(stops=True)
        res = g._actions.close(SPEED_MM_S, progress=seen.append)

        self.assertAlmostEqual(res.final_cmd_rad, fake.frames[-1].q, places=9)
        self.assertTrue(seen)
        for p in seen:
            self.assertLessEqual(p.cmd_rad, self.HI + 1e-9)
            self.assertGreaterEqual(p.cmd_rad, self.LO - 1e-9)


class TestParkTolerance(unittest.TestCase):
    """``stop_tol`` 的两条边：什么算「压到位」，什么不算（#46）。

    记录的限位总是比 ``close()`` 能到的地方深一点，因为两者用力不同：
    ``zero()`` 用 ``calib_kp 20 × calib_step_rad 0.05``（≈1.0 Nm）顶进去，而
    ``close()`` 按自己的刚度压。台架实测缺口 0.92–1.27 mm，在它的
    ``rad_to_mm=57.55`` 下是 0.016–0.022 rad —— 原来的 0.02 只差 0.001 rad
    就够不着，于是按文档顺序跑 ``stroke_test`` 第一遍就 exit 1。

    放宽 ``stop_tol`` 等于放宽「真的顶在限位上」这个判据本身，``open()`` 走同一
    段代码会一起变松，所以两条边都要钉住：真限位落在标定限位内侧 0.022 rad 处
    要判成功（原来判失败），落在再内侧 0.02 rad 以上仍要判失败。
    """

    # 台架最坏的一次缺口（1.27 mm @ 57.55 mm/rad），换算成 rad 与机器无关。
    BENCH_GAP_RAD = 0.022

    def _close_onto_a_stop_inside(self, gap_rad, sticky_rad=0.0):
        """真限位放在标定闭合限位内侧 ``gap_rad`` 处，闭合一次。"""
        g, fake = make_gripper(sticky_rad=sticky_rad, stops=True)
        fake.motor.limit_hi = POS_CLOSED_RAD - gap_rad
        return g, g._actions.close(SPEED_MM_S)

    def test_a_stop_inside_the_calibration_counts_as_pressed_home(self):
        """#46 那一格：缺口比旧容差大，但它就是「顶到真限位了」。"""
        g, res = self._close_onto_a_stop_inside(self.BENCH_GAP_RAD)

        self.assertTrue(res.ok, res)
        self.assertTrue(res.stalled, res)
        self.assertLess(abs(res.state.position_rad - POS_CLOSED_RAD),
                        g.motion_config.stop_tol)

    def test_the_closing_side_dead_band_is_absorbed_too(self):
        """带上闭合侧约 0.0103 rad 的粘滑死区，读数一跳也不该翻过这条线。"""
        g, res = self._close_onto_a_stop_inside(self.BENCH_GAP_RAD,
                                                sticky_rad=0.0103)

        self.assertTrue(res.ok, res)
        self.assertLess(abs(res.state.position_rad - POS_CLOSED_RAD),
                        g.motion_config.stop_tol)

    def test_a_stop_further_inside_is_not_pressed_home(self):
        """放宽后的反面 —— 没有这条，放宽就成了「怎么都算成功」。"""
        tol = MotionConfig().stop_tol
        g, res = self._close_onto_a_stop_inside(tol + 0.02)

        self.assertFalse(res.ok, res)
        self.assertTrue(res.stalled, res)
        self.assertGreater(abs(res.state.position_rad - POS_CLOSED_RAD), tol)

    def test_the_open_side_loosens_with_it(self):
        """``open()`` 走同一段代码：判据变松它一起变松，这条把它钉住。"""
        g, fake = make_gripper(start_rad=POS_CLOSED_RAD, stops=True)
        fake.motor.limit_lo = POS_OPEN_RAD + self.BENCH_GAP_RAD
        res = g._actions.open(SPEED_MM_S)

        self.assertTrue(res.ok, res)
        self.assertTrue(res.stalled, res)
        self.assertLess(abs(res.state.position_rad - POS_OPEN_RAD),
                        g.motion_config.stop_tol)


class TestStallFalsePositives(unittest.TestCase):
    """8 / 9：粘滑不能误判堵转；非顶限位路径的保压段不算堵转。"""

    def test_sticky_dead_band_does_not_stall_mid_travel(self):
        # 闭合侧实测约 0.0103 rad 一跳的粘滑；窗口净位移仍远大于阈值
        g, _ = make_gripper(sticky_rad=0.0103, stops=True)
        _, ramp_steps, _, _ = planned_steps(g, SPEED_MM_S, press=True)
        res = g._actions.close(SPEED_MM_S)

        self.assertGreaterEqual(res.steps, ramp_steps)  # 一路走到限位才停
        self.assertTrue(res.ok, res)

    def test_settle_phase_is_not_a_stall_without_press(self):
        # grasp 的闭合段（press=False）停在限位内侧，保压段不该判堵转
        g, fake = make_gripper()
        _, ramp_steps, settle_steps, total = planned_steps(g, SPEED_MM_S)
        res = g._actions._move_to_limit("close", SPEED_MM_S)

        self.assertFalse(res.stalled, res)
        self.assertEqual(res.steps, total)           # 跑满，没提前退

        # 保压段几乎不动（只剩速度前馈留下的一点点超调在收），但帧还在发
        tail = fake.frames[ramp_steps:]
        self.assertEqual(len(tail), settle_steps)
        self.assertLess(max(abs(f.pos_after - tail[0].pos_after) for f in tail),
                        0.01)


class TestReachTolerance(unittest.TestCase):
    """10：差 0.0103 rad 停住时，容差 0.02 判到位、0.01 判不到位。"""

    def _close_short(self, reach_tol):
        g, fake = make_gripper()
        target, _, _, _ = planned_steps(g, SPEED_MM_S)     # 非顶限位目标
        # 挡在目标前 0.0103 rad（实测闭合侧的机械死区）
        fake.motor.set_block(target - 0.0103)
        g.motion_config = MotionConfig(
            sleep_fn=lambda _: None, reach_tol=reach_tol)
        return g._actions._move_to_limit("close", SPEED_MM_S)

    def test_default_tolerance_accepts(self):
        self.assertTrue(self._close_short(0.02).reached)

    def test_tight_tolerance_rejects(self):
        self.assertFalse(self._close_short(0.01).reached)


class TestMoveResultBool(unittest.TestCase):
    """``bool(MoveResult)`` 是 ``ok``，不是旧的 ``reached and not stalled``。"""

    def test_press_success_is_truthy_though_it_stalled(self):
        g, _ = make_gripper(stops=True)
        res = g._actions.close(SPEED_MM_S)
        self.assertTrue(res.stalled)
        self.assertFalse(res.reached)
        self.assertEqual(bool(res), res.ok)
        self.assertTrue(bool(res))

    def test_blocked_halfway_is_falsy(self):
        g, _ = make_gripper(block_rad=POS_OPEN_RAD + 0.01)
        res = g._actions.close(SPEED_MM_S)
        self.assertEqual(bool(res), res.ok)
        self.assertFalse(bool(res))


class TestGrasp(unittest.TestCase):
    """11 / 12 / 13：保力前馈、夹到工件、故障中止。"""

    def _motion(self, hold_s):
        return MotionConfig(sleep_fn=lambda _: None,
                            monotonic_fn=tick_clock(0.1))

    def test_hold_applies_force_feedforward(self):
        block = (POS_OPEN_RAD + POS_CLOSED_RAD) / 2     # 当作工件
        g, fake = make_gripper(block_rad=block)
        g.motion_config = self._motion(0.4)
        # 保力按 force_ramp_n_s 从交接力矩爬上去，给够时间让爬升落到设定值
        res = g._actions.grasp(force_n=20.0, hold_s=1.2)

        self.assertIsInstance(res, GraspResult)
        self.assertEqual(res.force_n, 20.0)
        self.assertGreater(res.cycles, 0)
        hold_frames = [f for f in fake.frames if f.tau_ff == 2.0]
        self.assertTrue(hold_frames)                    # 20 N → 2.0 Nm
        self.assertAlmostEqual(hold_frames[0].tau_nm, 2.0, places=6)

    def test_object_in_the_way_stalls_but_grasps(self):
        block = (POS_OPEN_RAD + POS_CLOSED_RAD) / 2
        g, _ = make_gripper(block_rad=block)
        g.motion_config = self._motion(0.4)
        res = g._actions.grasp(force_n=20.0, hold_s=0.4)

        self.assertTrue(res.stalled)      # 撞上工件
        self.assertFalse(res.reached)     # 压不到空载目标 —— 正常
        self.assertTrue(res.ok)           # 但保力是好的

    def test_fault_during_hold_aborts(self):
        block = (POS_OPEN_RAD + POS_CLOSED_RAD) / 2
        g, fake = make_gripper(block_rad=block)
        g.motion_config = self._motion(0.4)
        fake.error_when_holding = 0x9              # 一开始施力就报故障
        res = g._actions.grasp(force_n=20.0, hold_s=0.4)

        self.assertFalse(res.ok)
        self.assertEqual(res.cycles, 1)            # 第一片之后就被叫停
        self.assertEqual(res.state.error_code, 0x9)


class TestContactVerification(unittest.TestCase):
    """Near the closed stop, a short extra squeeze separates object from pocket."""

    def test_free_air_moves_and_is_not_contact(self):
        g, _ = make_gripper(start_rad=POS_OPEN_RAD)
        self.assertFalse(g._actions._verify_contact(20.0))

    def test_a_block_stops_the_verification_step(self):
        g, _ = make_gripper(start_rad=-0.5, block_rad=-0.499)
        self.assertTrue(g._actions._verify_contact(20.0))

    def test_grasp_refuses_when_the_verification_step_moves(self):
        g, _ = make_gripper(start_rad=POS_OPEN_RAD,
                           block_rad=POS_CLOSED_RAD - 0.03)
        with mock.patch.object(g._actions, "_verify_contact",
                               return_value=False):
            res = g._actions.grasp(force_n=3.0, hold_s=0.05)
        self.assertFalse(res.ok)
        self.assertTrue(res.no_bite)
        self.assertEqual(res.cycles, 0)


class TestGraspHandsOverOnlyOnContact(unittest.TestCase):
    """11b / 11c：闭合段只在**到位**或**顶上东西**时才交给保力段。

    真机事故（2026-10-13，``grasp(10.0, hold_s=1.0)`` 空载）：闭合段在 303 帧的
    预算里只走到 25.4 mm，离目标还差 21 mm，``reached=False``、``stalled=False``
    —— 而 ``grasp()`` **无条件**把夹爪交给了 ``_hold_force``。保力帧里既没有位置
    项也没有速度项，夹爪速度当场从被速度环限住的 0.82 rad/s 跳到 5.12 rad/s，
    到 ``hold_s`` 用完仍在加速。这就是「靠近末端时飞快地移动一下」。

    闭合段为什么走不到：带设定力时 :func:`force_approach_terms` 把领先上限压到
    0.25 mm，而指令是 ``min(排程, 读数 + 领先上限)`` —— 落后量一超过领先上限，
    指令就被钉在**过期的**读数旁边，排程再也推不动夹爪。假件以前没有读数延迟，
    0.25 mm 的领先足够它跟上排程，所以这条路径在测试里永远是绿的。下面用
    ``report_lag_ticks`` 把这一维补上：3 帧 × 0.82 rad/s × 5 ms ≈ 0.012 rad，
    正好是那个领先上限的三倍。
    """

    #: 读数延迟帧数。0 时假件与改动前逐位一致（见 :class:`FakeMotor`）。
    LAG_TICKS = 3

    def _grasp(self, **kw):
        motion = MotionConfig(sleep_fn=lambda _: None,
                              monotonic_fn=tick_clock(0.1))
        g, fake = make_gripper(stops=True, report_lag_ticks=self.LAG_TICKS,
                               motion=motion, **kw)
        return g, fake, g._actions.grasp(force_n=5.0, hold_s=0.2)

    @staticmethod
    def _close_frames(fake):
        """闭合段的帧 —— 保力帧只发前馈，用 ``tau_ff`` 把两段切开。"""
        return [f for f in fake.frames if f.tau_ff == 0.0 and f.kp != 0.0]

    def test_a_stale_reading_no_longer_strands_the_close_leg(self):
        """读数落后三帧时空载夹取照样到限位，闭合段跑得比排程还长。"""
        g, fake, res = self._grasp()
        _t, ramp_steps, _s, _tot = planned_steps(
            g, g.motion_config.grasp_speed_mm_s)

        self.assertTrue(res.reached, res)
        self.assertFalse(res.stalled, res)
        self.assertTrue(res.ok, res)
        self.assertAlmostEqual(fake.motor.pos, res.target_rad,
                               delta=g.motion_config.reach_tol)
        # 领先被读数延迟吃掉一半，夹爪追不上排程 —— 所以闭合段的帧数**超过**排程
        # 帧数。旧代码的预算正是 ``ramp_steps + settle_steps``，到点即收尾，夹爪
        # 停在半路（实测差 30 mm）而 reached / stalled 都是 False。
        self.assertGreater(len(self._close_frames(fake)), ramp_steps)

    def test_the_close_leg_is_at_rest_when_the_hold_takes_over(self):
        """交接发生在静止上：闭合段收尾时夹爪已经停在限位上、不再走。

        旧行为是交接那一刻夹爪还在 0.82 rad/s 上飞。保力帧没有速度项，接过去
        就是一路加速 —— 真机上量到 5.12 rad/s。
        """
        g, fake, res = self._grasp()
        close = self._close_frames(fake)
        speed_rad_s = g.motion_config.grasp_speed_mm_s / g.config.rad_to_mm

        self.assertTrue(res.reached, res)
        self.assertGreater(len(close), 3)
        for prev, cur in zip(close[-3:], close[-2:]):
            v = abs(cur.pos_after - prev.pos_after) / DT
            self.assertLess(v, 0.05 * speed_rad_s,
                            "交接前夹爪还在 %.4f rad/s 上飞（指令 %.4f）"
                            % (v, speed_rad_s))
        self.assertAlmostEqual(close[-1].pos_after, res.target_rad,
                               delta=g.motion_config.reach_tol)

    def test_neither_reached_nor_stalled_is_not_handed_over(self):
        """闸门本身：闭合段既没到位也没堵转 —— 不交接，一帧保力帧都不发。

        把 ``_move_to_limit`` 换成一个固定结果来触发这条路径（真机上它对应
        「夹爪顶着一路缓让的软工件慢慢爬，爬到帧数用完」那一类）。
        """
        motion = MotionConfig(sleep_fn=lambda _: None,
                              monotonic_fn=tick_clock(0.1))
        g, fake = make_gripper(stops=True, motion=motion)
        stranded = POS_OPEN_RAD + 0.2
        g._actions._move_to_limit = lambda *a, **kw: MoveResult(
            ok=False, reached=False, stalled=False,
            state=GripperState(position_rad=stranded),
            target_rad=POS_CLOSED_RAD, limit_rad=POS_CLOSED_RAD,
            final_cmd_rad=stranded, steps=303)

        res = g._actions.grasp(force_n=5.0, hold_s=0.2)

        self.assertFalse(res.ok, res)
        self.assertFalse(res.reached, res)
        self.assertFalse(res.stalled, res)
        self.assertEqual(res.cycles, 0)
        self.assertEqual(res.force_n, 5.0)          # 设定值照实报出来
        # 「没交接」的硬判据：一帧前馈都没有。撤力帧是 ``kp=kd=tau=0``，所以
        # 不能用 kp 判 —— 只有 ``tau_ff`` 能把保力帧和它们分开。
        self.assertTrue(fake.frames, "一帧都没发")
        self.assertTrue(all(f.tau_ff == 0.0 for f in fake.frames),
                        "交接了：发出了前馈力矩帧")
        self.assertTrue(any(f.kp == 0.0 for f in fake.frames), "没有撤力帧")


class TestForceApproachNeedsATrueStop(unittest.TestCase):
    """2026-11-18 台架：5 N 接近段实际只有约 8 mm/s，0.25 s 走 2 mm。

    20% 的窗口阈值把「慢」判成「顶上东西」，保力段随后在自由行程里把夹爪
    加速到 7 rad/s，从 78 mm 冲到 26 mm 的工件上。交接前必须看到**真停**：
    窗口平均速度低于 ``MotionConfig.stall_speed_mm_s``。
    """

    def test_a_slow_creep_is_not_handed_over_mid_travel(self):
        object_rad = (POS_OPEN_RAD + POS_CLOSED_RAD) / 2.0
        motion = MotionConfig(sleep_fn=lambda _: None,
                              monotonic_fn=tick_clock(0.1))
        crawl_rad_s = 0.109              # ≈ 8 mm/s at RAD_TO_MM
        g, fake = make_gripper(block_rad=object_rad, stops=True,
                               motion=motion, speed_cap_rad_s=crawl_rad_s)

        g._actions.grasp(force_n=5.0, hold_s=0.2)

        holds = [f for f in fake.frames if f.tau_ff != 0.0]
        self.assertTrue(holds, "没有进入保力段")
        first = holds[0].pos_after
        self.assertLess(
            abs(first - object_rad), g.motion_config.reach_tol,
            "保力段在离工件 %.2f mm 处接手了 —— 又是自由行程里交接" %
            (abs(first - object_rad) * RAD_TO_MM))

    def test_a_slow_creep_without_contact_never_enters_the_hold(self):
        """没有工件、只是被摩擦压在 8 mm/s：走到帧数上界按拒夹收尾。

        这条钉的是「不交接」本身，而不只是「交接点提前」：慢速蠕动既没到位
        也没真停，保力段一帧都不许发 —— 它是纯力矩源，接过去只会继续加速。
        """
        motion = MotionConfig(sleep_fn=lambda _: None,
                              monotonic_fn=tick_clock(0.1))
        g, fake = make_gripper(stops=True, motion=motion,
                               speed_cap_rad_s=0.109)    # ≈ 8 mm/s

        res = g._actions.grasp(force_n=5.0, hold_s=0.2)

        self.assertFalse(res.ok, res)
        self.assertFalse(res.stalled, res)
        self.assertFalse(res.reached, res)
        self.assertEqual(res.cycles, 0)
        self.assertEqual([f for f in fake.frames if f.tau_ff != 0.0], [],
                         "慢速蠕动被交给了纯力矩的保力段")


class TestMotionRefreshesAnIdleBus(unittest.TestCase):
    """空闲总线上的读数只在状态帧里更新；动作不能从手扳之前的缓存起跳。"""

    def test_open_does_not_start_from_a_position_read_before_the_bus_went_idle(self):
        g, fake = make_gripper(stops=True, freeze_position_while_idle=True)
        g.get_state()                    # 缓存停在张开侧
        fake.delivers = False
        actual = POS_CLOSED_RAD - 0.4    # 总线空闲时被手扳到中间
        fake.motor.pos = actual

        g._actions.open()

        first = fake.frames[0]
        self.assertLess(
            abs(first.q - actual), 0.01,
            "open 从缓存位置起跳 %.1f mm（实际在 %.1f mm）" %
            (abs(first.q - actual) * RAD_TO_MM,
             abs(actual - fake.motor.pos) * RAD_TO_MM))


class TestRelease(unittest.TestCase):
    """松开：跨线程结束 ``grasp()``、从交接点张余量、急停、不重复张开。

    线程用例里 ``sleep_fn`` 用 ``time.sleep(0)`` 而不是空函数：空函数下夹取
    线程会独占 GIL 一直跑，主线程的请求要等解释器切片的 5 ms 才递得进去 ——
    用例还是能过，但等待时间不确定。``sleep(0)`` 只是让出 GIL，不真的睡。
    """

    OBJECT_RAD = (POS_OPEN_RAD + POS_CLOSED_RAD) / 2

    def _motion(self, **kw) -> MotionConfig:
        return MotionConfig(sleep_fn=lambda _: time.sleep(0),
                            monotonic_fn=tick_clock(0.1), **kw)

    def _start_hold(self, g, progress=None):
        """后台线程里起一个 ``grasp(hold_s=0)``。

        Returns:
            ``(线程, 结果容器, 保力已开始的事件)``
        """
        started = threading.Event()
        outcome = {}

        def report(p):
            if p.phase == "hold":
                started.set()
            if progress is not None:
                progress(p)

        def run():
            outcome["res"] = g._actions.grasp(force_n=20.0, hold_s=0.0, progress=report)

        t = threading.Thread(target=run, daemon=True)
        t.start()
        return t, outcome, started

    def test_release_from_another_thread_ends_a_zero_hold_and_opens_from_the_grip_pose(self):
        """``hold_s=0`` 在另一个线程里，``release()`` 把它结束并从交接点张 10 mm。

        工件会屈服（``yield_rad_s``）：设定力在保力期间把夹爪往闭合侧继续推，
        所以松开那一刻的位置**不是**交接点。基准必须取交接点 —— 也就是从夹爪
        声明抓住的位置再张 10 mm，而不是从被压实后的位置。
        """
        g, fake = make_gripper(block_rad=self.OBJECT_RAD,
                               yield_rad_s=0.05, yield_tau_nm=0.5,
                               motion=self._motion())
        t, outcome, started = self._start_hold(g)
        self.assertTrue(started.wait(2.0), "保力段没有开始")
        grip_rad = g._actions._grip_rad
        self.assertIsNotNone(grip_rad, "交接点没有记录")
        # 等保力把夹爪往里再压一段（工件屈服），这样「交接点」与「停下处」
        # 才是两个不同的数；否则这条用例分不出用哪个当基准。
        deadline = time.monotonic() + 2.0
        while fake.motor.pos - grip_rad < 0.01 and time.monotonic() < deadline:
            time.sleep(0.001)
        self.assertGreater(fake.motor.pos - grip_rad, 0.01,
                           "工件没有屈服，用例分不出基准")
        hold_from = len(fake.frames)

        result = g._actions.release(clearance_mm=10.0)
        t.join(2.0)
        self.assertFalse(t.is_alive(), "grasp 没有在松开后返回")

        res = outcome["res"]
        self.assertTrue(res.released, res)
        self.assertTrue(res.ok, res)
        self.assertIsInstance(result, ReleaseResult)
        self.assertTrue(result.ok, result)

        # 基准按**交接点**复算；若错用了松开那一刻的位置，目标会少那几毫米。
        gcfg = g.config
        s = gcfg.close_sign
        grip_mm = (gcfg.pos_closed_rad - grip_rad) * s * gcfg.rad_to_mm
        expected_mm = grip_mm + 10.0
        expected_rad = gcfg.pos_closed_rad - s * expected_mm / gcfg.rad_to_mm
        self.assertAlmostEqual(result.target_rad, expected_rad, places=6)
        self.assertAlmostEqual(result.opened_mm, expected_mm, places=4)
        self.assertAlmostEqual(res.state.position_rad, expected_rad,
                               delta=g.motion_config.reach_tol)

        # 张开是斜坡：保力之后有一串位置帧，第一帧不是一步跳到目标。
        opens = [f for f in fake.frames[hold_from:] if f.kp != 0.0]
        self.assertGreater(len(opens), 5, "张开不是一段斜坡而是阶跃")
        first_pos = opens[0].pos_after
        self.assertLess(abs(opens[0].q - first_pos),
                        abs(expected_rad - first_pos) / 2,
                        "第一帧就跳到了目标附近")
        # 正装时张开是 rad 减小，整段单调。
        self.assertTrue(all(b.q <= a.q + 1e-9
                            for a, b in zip(opens, opens[1:])))

    def test_release_during_the_approach_aborts_it(self):
        """接近途中按下松开：不再交接，改从当前位置张开。"""
        g, fake = make_gripper(motion=self._motion())
        requested = False

        def progress(p):
            nonlocal requested
            if p.phase == "move" and not requested:
                # Queue the request from inside the approach, so it is
                # unambiguously seen by the next frame instead of racing the
                # approach's end on a thread scheduler.
                self.assertTrue(g._actions._request_release(5.0))
                requested = True

        res = g._actions.grasp(
            force_n=20.0, hold_s=0.0, progress=progress)
        self.assertTrue(requested, "接近段没有开始")
        self.assertTrue(res.released, res)
        self.assertTrue(res.ok, res)
        self.assertEqual(res.cycles, 0, "接近段被中止了却还交接了保力")
        # 一帧保力帧都不该发（保力帧是唯一带 tau_ff 的帧）。
        self.assertFalse([f for f in fake.frames if f.tau_ff != 0.0])
        # 还没交接 → 基准是当前位置；当前位置离张开端不足 5 mm，目标被夹到
        # 标定张开限位。
        self.assertAlmostEqual(res.state.position_rad, POS_OPEN_RAD,
                               delta=g.motion_config.reach_tol)

    def test_release_without_a_grasp_opens_from_the_current_position(self):
        """没有夹取在跑：在调用方线程里直接张，基准是当前位置。"""
        g, fake = make_gripper(start_rad=POS_CLOSED_RAD, stops=True,
                               motion=self._motion())

        result = g._actions.release(clearance_mm=5.0)

        gcfg = g.config
        expected_rad = (gcfg.pos_closed_rad
                        - gcfg.close_sign * 5.0 / gcfg.rad_to_mm)
        self.assertTrue(result.ok, result)
        self.assertAlmostEqual(result.target_rad, expected_rad, places=6)
        self.assertAlmostEqual(result.opened_mm, 5.0, places=4)
        self.assertTrue(fake.frames, "一帧都没发")
        self.assertAlmostEqual(result.state.position_rad, expected_rad,
                               delta=g.motion_config.reach_tol)

    def test_a_second_release_does_not_open_twice(self):
        """同一个夹取里第二次 ``release()`` 直接拿第一次的结果，不再张开。"""
        g, _ = make_gripper(block_rad=self.OBJECT_RAD, motion=self._motion())
        opening = threading.Event()

        t, outcome, started = self._start_hold(g)
        self.assertTrue(started.wait(2.0), "保力段没有开始")

        calls = []
        original = g._actions._do_release

        def counted(req):
            calls.append(req)
            opening.set()
            # 等第二次请求真的排进来再放行张开斜坡 —— 这样「第二次调用发生在
            # 第一次的松开进行中」是确定的，不靠线程调度。
            deadline = time.monotonic() + 2.0
            while not g._actions._release_pending() and time.monotonic() < deadline:
                time.sleep(0.001)
            return original(req)

        g._actions._do_release = counted

        results = []

        def first_release():
            results.append(g._actions.release(clearance_mm=10.0))

        first = threading.Thread(target=first_release, daemon=True)
        first.start()
        self.assertTrue(opening.wait(2.0), "第一次松开没有开始张开")
        second = g._actions.release(clearance_mm=10.0)

        first.join(2.0)
        t.join(2.0)
        self.assertFalse(first.is_alive())
        self.assertFalse(t.is_alive(), "grasp 没有返回")
        self.assertEqual(len(calls), 1, "第二次 release 又张开了一次")
        self.assertTrue(all(r.ok for r in results + [second]))
        self.assertTrue(outcome["res"].released)


class TestForceHoldIsTorqueOnly(unittest.TestCase):
    """保力只下发前馈力矩（kp=kd=0）：力不随工件屈服而衰减。

    保力帧带上位置刚度时，MIT 律里的 ``kp × (q - 实测位置)`` 会随夹爪位移变
    化 —— 工件在设定力下让位，夹爪跟着走，这一项就从设定力里扣掉一截，现象
    是「先夹到设定力，过一会儿掉到某个更小的值」。闭合侧约 0.0103 rad 的粘滑
    死区每走一格，旧实现里 ``hold_kp=150`` 会换算成约 15 N 的力误差。
    """

    # 工件：夹在行程中段的挡块，受载后持续让位
    OBJECT_RAD = POS_OPEN_RAD + 0.5 * (POS_CLOSED_RAD - POS_OPEN_RAD)
    YIELD_RAD_S = 0.05                  # ≈ 3.7 mm/s（RAD_TO_MM = 74.19）
    YIELD_TAU_NM = 0.5                  # 载荷一过 0.5 Nm 就开始让位

    def _motion(self, **kw):
        return MotionConfig(sleep_fn=lambda _: None,
                            monotonic_fn=tick_clock(0.1), **kw)

    def _yielding_gripper(self):
        return make_gripper(block_rad=self.OBJECT_RAD,
                            yield_rad_s=self.YIELD_RAD_S,
                            yield_tau_nm=self.YIELD_TAU_NM)

    def test_a_yielding_workpiece_does_not_erode_the_hold_force(self):
        g, fake = self._yielding_gripper()
        g.motion_config = self._motion()
        # 保力先按 force_ramp_n_s 爬到设定值，之后才是这条用例要看的「停在
        # 设定值上不被工件带走」那一段 —— 时长得容下爬升
        res = g._actions.grasp(force_n=20.0, hold_s=1.2)

        self.assertTrue(res.ok, res)
        hold = [f for f in fake.frames if f.tau_ff == 2.0]      # 20 N → 2.0 Nm
        self.assertTrue(hold, "没有保力帧")
        # 工件一直在让位（夹爪被它带着往里走）—— 不然这条用例是空的
        self.assertGreater(hold[-1].pos_after - hold[0].pos_after, 0.005)
        # 让位再深，读到的力矩也得还是设定值
        for f in hold:
            self.assertAlmostEqual(f.tau_nm, 2.0, places=6)

    def test_the_hold_frame_carries_no_gains(self):
        # 保力帧是纯力矩源：没有位置/速度增益。
        g, fake = self._yielding_gripper()
        g.motion_config = self._motion()
        g._actions.grasp(force_n=20.0, hold_s=1.2)

        hold = [f for f in fake.frames if f.tau_ff == 2.0]
        self.assertTrue(hold, "没有保力帧")
        for f in hold:
            self.assertEqual(f.kp, 0.0)
            self.assertEqual(f.kd, 0.0)


class TestHeldForceRampsToSetpoint(unittest.TestCase):
    """保力力矩按固定速率（force_ramp_n_s，N/s）从飞行力矩爬到设定值。

    旧行为是一步跳到设定值：交接时电机力矩里还带着闭合压紧量（真机上约 10 N），
    一帧跳到 20 N 就是隔着机构的一次冲击，指爪会被刚碰到的东西弹开 —— 现场看到
    「停在 10 N，然后跳到 20 N，边跳边往里收」。斜坡要的是「力均匀地涨、到了设定
    值就停」，而且必须**每帧**走一步、正好落在设定值上。

    为了让交接处飞行力矩（约 10 N）低于设定值（20 N），把行进段领先上限收到
    ``max_lead_mm=0.74``：``kp × 上限 ≈ 0.997 Nm ≈ 10 N``，真机上的量级。
    接近速度也一并降到 ``grasp_speed_mm_s=25``：闭合段现在带着设定力的预算走
    （见 :func:`force_approach_terms`），而阻尼项 ``kd × v`` 先从预算里扣，默认
    50 mm/s 下它要吃掉 1.35 Nm —— 20 N 的预算只剩 1.8 Nm，领先就被压到 0.74 mm
    以下，飞行力矩不再是这 10 N。25 mm/s 下阻尼 0.67 Nm，0.74 mm 的领先上限
    还是先绑住的那一个。
    """

    #: 工件：行程中段的硬挡块（不移位，专注看力矩的形状）
    OBJECT_RAD = POS_OPEN_RAD + 0.5 * (POS_CLOSED_RAD - POS_OPEN_RAD)
    #: kp=100 × 0.74/74.19 ≈ 1.0 Nm ≈ 10 N 的压紧力
    PRESS_LEAD_MM = 0.74
    #: 让 0.74 mm 的领先上限仍然绑在预算之内的接近速度
    APPROACH_SPEED_MM_S = 25.0
    #: force_ramp_n_s × frame_interval × 0.1 Nm/N = 20 × 0.005 × 0.1
    STEP_NM = 20.0 * DT * 0.1
    FRAMES_PER_SLICE = 40                       # hold_interval 0.2 ÷ frame_interval 0.005

    def _motion(self, **kw):
        kw.setdefault("grasp_speed_mm_s", self.APPROACH_SPEED_MM_S)
        return MotionConfig(max_lead_mm=self.PRESS_LEAD_MM,
                            sleep_fn=lambda _: None,
                            monotonic_fn=tick_clock(0.1), **kw)

    def _grasp(self, hold_s=0.6, force_n=20.0, **motion_kw):
        g, fake = make_gripper(block_rad=self.OBJECT_RAD)
        g.motion_config = self._motion(**motion_kw)
        res = g._actions.grasp(force_n=force_n, hold_s=hold_s)
        move = [f for f in fake.frames if f.kp != 0.0]     # 闭合段：带位置刚度
        hold = [f for f in fake.frames if f.kp == 0.0]     # 保力段：纯前馈
        return g, fake, res, move, hold

    def test_the_climb_starts_from_the_torque_in_flight(self):
        _g, _fake, _res, move, hold = self._grasp()

        in_flight = move[-1].tau_nm
        # 交接处的飞行力矩就是压紧力，低于设定值 —— 否则这条用例证明不了爬升
        self.assertAlmostEqual(in_flight, 1.0, places=1)
        # 第一帧 = 飞行力矩 + 一步，而不是直接就是设定值
        self.assertAlmostEqual(hold[0].tau_ff, in_flight + self.STEP_NM, places=9)
        self.assertLess(hold[0].tau_ff, 2.0)

    def test_every_climbing_frame_adds_the_same_amount(self):
        _g, _fake, _res, _move, hold = self._grasp()

        climb = [f.tau_ff for f in hold if f.tau_ff < 2.0]
        self.assertGreater(len(climb), 1)
        for a, b in zip(climb, climb[1:]):
            self.assertAlmostEqual(b - a, self.STEP_NM, places=9)

    def test_it_lands_exactly_on_the_setpoint_and_stays(self):
        _g, _fake, _res, _move, hold = self._grasp()

        # 设定值 20 N = 2.0 Nm；爬到位之后每一帧都停在上面
        landed = [i for i, f in enumerate(hold) if f.tau_ff >= 2.0]
        self.assertTrue(landed, "没有一帧到达设定值")
        for f in hold[landed[0]:]:
            self.assertAlmostEqual(f.tau_ff, 2.0, places=9)

    def test_the_climb_is_linear_in_time(self):
        _g, _fake, _res, _move, hold = self._grasp()

        climb = [f.tau_ff for f in hold if f.tau_ff < 2.0]
        self.assertGreater(len(climb), 10)
        start = climb[0] - self.STEP_NM
        total = 2.0 - start
        # 逐帧等步长就是线性：第 k 帧 = 起点 + (k+1) 步
        for k in range(0, len(climb), 7):
            self.assertAlmostEqual(climb[k], start + (k + 1) * self.STEP_NM,
                                   places=9)
        # 爬到一半：一半的帧数对应一半的升幅。被替换掉的指数（时间常数 0.05 s）
        # 在这已经到顶了，这里必须还在半路。
        mid = len(climb) // 2
        self.assertLessEqual(
            abs(climb[mid] - (start + total / 2)), self.STEP_NM + 1e-9)
        self.assertLess(climb[mid], start + 0.7 * total)

    def test_advancing_once_per_frame_not_once_per_slice(self):
        _g, _fake, _res, _move, hold = self._grasp()

        # 一片 40 帧只在片首走一步的话，0.2 s 的片会一次涨 4 N（0.4 Nm）—— 台阶。
        # 逐帧走，片内相邻两帧只差一步。
        within_slice = hold[1].tau_ff - hold[0].tau_ff
        self.assertAlmostEqual(within_slice, self.STEP_NM, places=9)
        self.assertNotAlmostEqual(within_slice,
                                  self.STEP_NM * self.FRAMES_PER_SLICE, places=9)

    def test_a_torque_already_past_the_setpoint_starts_at_the_setpoint(self):
        # 交接力矩已经越过设定值时没有可爬的，直接从设定值起步 —— 往下走到设定值
        # 不是冲击。grasp 现在走不到这一支：闭合段压出的力矩由设定力的预算封顶
        # （press_safety=0.9），压紧力恒在设定值之下。所以这里直接进保力，把飞行
        # 力矩摆到设定值之上，钉住这个守卫本身。
        g, fake = make_gripper(block_rad=self.OBJECT_RAD)
        g.motion_config = MotionConfig(sleep_fn=lambda _: None,
                                       monotonic_fn=tick_clock(0.1))
        fake.motor.tau = 5.0                      # 飞行力矩 ≈ 50 N，远超 20 N 设定值
        g._actions._hold_force(20.0, 0.4)

        hold = [f for f in fake.frames if f.kp == 0.0]
        self.assertTrue(hold, "没有保力帧")
        self.assertAlmostEqual(hold[0].tau_ff, 2.0, places=9)



class TestHoldTravelGuard(unittest.TestCase):
    """保力段的位置护栏：越出闭合侧标定限位太多就撤力。

    保力帧是纯力矩源（``kp=kd=0``），没有位置指令可夹，所以护栏是**判据**不是
    夹子 —— 撤掉力矩是唯一能用的手段。默认的假电机在 ``kp=0`` 时一点不动（位置
    项按 ``kp`` 缩放），所以要开 ``creep_rad_s_per_nm`` 才复现得出那条失控路径：
    真机上闭合侧限位缺席时，恒定前馈力矩会把夹爪一路推走（2026-10-10 那台被推到
    越过标定点 104.9 mm）。
    """

    FORCE_N = 20.0
    #: 纯前馈力矩下的蠕变 rad/s per Nm。20 N = 2.0 Nm ⇒ 0.1 rad/s；一片
    #: （hold_interval=0.2 s）走 0.02 rad，所以护栏的越位量以一片为粒度。
    CREEP = 0.05

    def _motion(self, **kw):
        return MotionConfig(sleep_fn=lambda _: None,
                            monotonic_fn=tick_clock(0.1), **kw)

    def _slice_travel_rad(self, g):
        """一个回读片里、在设定力矩下最多能多走出来的量 rad。"""
        cfg = g.motion_config
        return (cfg.hold_interval * self.CREEP * self.FORCE_N
                * UnitConversion.N_TO_NM)

    def _release_frames(self, g, fake):
        """保力段尾部那些力矩为 0 的帧（= 触发后连发的释放帧）。"""
        hold = [f for f in fake.frames if f.kp == 0.0]
        zero_at = [i for i, f in enumerate(hold) if f.tau_ff == 0.0]
        self.assertTrue(zero_at, "触发之后一帧释放帧都没有")
        return hold[zero_at[0]:]

    def test_a_hold_that_walks_out_of_the_travel_is_cut_short(self):
        g, fake = make_gripper(stops=False, creep_rad_s_per_nm=self.CREEP)
        g.motion_config = self._motion()

        res = g._actions.grasp(force_n=self.FORCE_N, hold_s=5.0)

        self.assertFalse(res.ok, res)
        self.assertTrue(res.travel_guard, res)
        # 放开的位置就在阈值旁边 —— 以一片为粒度，不是「一路走到底」
        out_rad = closing_travel_excursion_rad(g.config, fake.motor.pos)
        self.assertGreater(out_rad, 0.0, "根本没走出去，这条用例没测到护栏")
        self.assertLessEqual(out_rad,
                             g.motion_config.hold_runaway_rad
                             + self._slice_travel_rad(g) + 1e-9)
        # 触发即失力：尾巴上全是 kp=kd=tau=0 的帧，数目就是 stop_release_s
        release = self._release_frames(g, fake)
        self.assertEqual(len(release),
                         int(round(g.motion_config.stop_release_s
                                   / g.motion_config.frame_interval)))
        for f in release:
            self.assertEqual(f.tau_ff, 0.0)
            self.assertEqual(f.kp, 0.0)
            self.assertEqual(f.kd, 0.0)

    def test_the_guard_does_not_fire_when_the_stop_is_there(self):
        """误报那一侧：真限位就在标定限位上，空载保力顶到它就停住。

        配了机械限位（``LIMIT_HI_RAD == POS_CLOSED_RAD``）时，恒定力矩把夹爪推到
        真限位就走不动了 —— 越位量正好 0，护栏不该响，保力该走完 ``hold_s``。
        """
        g, fake = make_gripper(stops=True, creep_rad_s_per_nm=self.CREEP)
        g.motion_config = self._motion()

        res = g._actions.grasp(force_n=self.FORCE_N, hold_s=2.0)

        self.assertFalse(res.travel_guard, res)
        self.assertTrue(res.ok, res)
        self.assertAlmostEqual(fake.motor.pos, POS_CLOSED_RAD, places=9)

    def test_a_yielding_workpiece_does_not_trip_the_guard(self):
        """工件让位时夹爪跟着走 —— 那是保力该有的样子，不是失控。

        工件在行程中段，离闭合侧限位还有一大截；它让位多少、夹爪就跟多少，护栏
        不该响。
        """
        obj = POS_OPEN_RAD + 0.5 * TRAVEL_RAD
        g, fake = make_gripper(block_rad=obj, yield_rad_s=0.02,
                               yield_tau_nm=0.5,
                               creep_rad_s_per_nm=self.CREEP)
        g.motion_config = self._motion()

        res = g._actions.grasp(force_n=self.FORCE_N, hold_s=1.0)

        self.assertFalse(res.travel_guard, res)
        self.assertTrue(res.ok, res)
        # 半路被工件挡住：判到堵转，压不到空载目标位置
        self.assertTrue(res.stalled, res)
        self.assertFalse(res.reached, res)

    def test_the_guard_follows_the_mount_direction(self):
        """反装：闭合方向是 rad 减小，闭合侧限位是数值小的那一个。"""
        g, fake = make_gripper(reverse=True, stops=False,
                               creep_rad_s_per_nm=self.CREEP)
        g.motion_config = self._motion()
        self.assertLess(g.config.close_sign, 0.0)          # 确认这真是反装

        res = g._actions.grasp(force_n=self.FORCE_N, hold_s=5.0)

        self.assertTrue(res.travel_guard, res)
        self.assertFalse(res.ok, res)
        out_rad = closing_travel_excursion_rad(g.config, fake.motor.pos)
        self.assertGreater(out_rad, 0.0, "根本没走出去，这条用例没测到护栏")
        self.assertLessEqual(out_rad,
                             g.motion_config.hold_runaway_rad
                             + self._slice_travel_rad(g) + 1e-9)

    def test_a_hold_entered_outside_the_travel_pushes_nothing(self):
        """已经在护栏之外的，一帧力矩都不发。

        夹爪被手推过去、或者换了标定之后读数落在限位外侧 —— 这时再保力只会把它
        推得更远。判在**第一帧之前**，所以整段一帧力矩都没有，直接失力返回。
        """
        g, fake = make_gripper(stops=False, creep_rad_s_per_nm=self.CREEP)
        g.motion_config = self._motion()
        fake.motor.pos = POS_CLOSED_RAD + 2.0 * g.motion_config.hold_runaway_rad

        hold = g._actions._hold_force(self.FORCE_N, 0.6)

        self.assertFalse(hold.ok, hold)
        self.assertTrue(hold.guarded, hold)
        self.assertEqual(hold.cycles, 0, "进了循环就已经把力矩发出去了")
        self.assertTrue(fake.frames, "连释放帧都没有")
        for f in fake.frames:
            self.assertEqual(f.tau_ff, 0.0, "护栏之外还发了力矩")

    def test_an_uncalibrated_config_gets_no_guard(self):
        """没标定时护栏不启用：占位限位描述的不是任何真实行程。

        ``grasp`` 走不到这里（它的闭合段先要求标定），但 ``_hold_force`` 是公开给
        同仓动作层的引擎，直接调用时也要给出这个行为。
        """
        g, fake = make_gripper(stops=False, creep_rad_s_per_nm=self.CREEP)
        g.motion_config = self._motion()
        g.config.calibrated = False
        fake.motor.pos = POS_CLOSED_RAD + 1.0
        # 这条用例要证明的是「已经越位了，但护栏没介入」
        self.assertGreater(
            closing_travel_excursion_rad(g.config, fake.motor.pos),
            g.motion_config.hold_runaway_rad)

        hold = g._actions._hold_force(self.FORCE_N, 0.4)

        self.assertTrue(hold.ok, hold)
        self.assertFalse(hold.guarded, hold)


class TestEnable(unittest.TestCase):
    """14: enable retries until the status frame says err == 1."""

    def test_retries_until_err_is_one(self):
        g, fake = make_gripper(err=0, initialize_results=[False, False, True])

        self.assertTrue(g.enable())
        self.assertEqual(fake.initialize_calls, 3)
        self.assertTrue(g.is_enabled)

    def test_gives_up_when_never_enabled(self):
        g, fake = make_gripper(err=0, initialize_results=[False, False, False])

        self.assertFalse(g.enable())
        self.assertEqual(fake.initialize_calls, 3)
        self.assertFalse(g.is_enabled)

    def test_real_fault_is_cleared_first(self):
        g, fake = make_gripper(err=0x9, initialize_results=[False, True])

        self.assertTrue(g.enable())
        self.assertEqual(fake.clear_fault_calls, 1)
        self.assertEqual(fake.initialize_calls, 2)

    def test_explicit_retries_override(self):
        g, fake = make_gripper(err=0, initialize_results=[False, True])

        self.assertFalse(g.enable(retries=1))
        self.assertEqual(fake.initialize_calls, 1)


class TestDisconnect(unittest.TestCase):
    """disconnect() disables the motor and closes the bus; it is idempotent."""

    def test_disables_and_closes(self):
        g, fake = make_gripper()
        g.disconnect()
        self.assertTrue(fake.disconnected)
        self.assertFalse(g.is_enabled)
        self.assertFalse(g.is_connected)

    def test_is_idempotent(self):
        g, fake = make_gripper()
        g.disconnect()
        g.disconnect()
        self.assertTrue(fake.disconnected)


class TestWorkLimitTarget(unittest.TestCase):
    """工作行程目标：停在张开限位**内侧**的工作点。"""

    def test_target_sits_below_the_open_stop(self):
        g, _ = make_gripper()
        target, limit, work_rad = work_limit_target(g.config, 80.0)

        self.assertAlmostEqual(limit, POS_OPEN_RAD)
        self.assertAlmostEqual(work_rad, 80.0 / RAD_TO_MM)
        self.assertAlmostEqual(target, POS_CLOSED_RAD - work_rad)
        self.assertGreater(target, POS_OPEN_RAD)      # 在限位内侧，没到停点

    def test_clamps_to_the_mechanical_travel(self):
        g, _ = make_gripper()
        travel_mm = TRAVEL_RAD * RAD_TO_MM
        target, _, work_rad = work_limit_target(g.config, travel_mm + 50.0)

        self.assertAlmostEqual(work_rad, TRAVEL_RAD)
        self.assertAlmostEqual(target, POS_OPEN_RAD)

    def test_reverse_mount_mirrors(self):
        g, _ = make_gripper(reverse=True)
        target, limit, work_rad = work_limit_target(g.config, 80.0)
        self.assertAlmostEqual(limit, POS_CLOSED_RAD)   # 反装张开在数值大的一侧
        self.assertAlmostEqual(target, POS_OPEN_RAD + work_rad)
        self.assertLess(target, POS_CLOSED_RAD)

    def test_uncalibrated_raises(self):
        g, _ = make_gripper()
        g.config.calibrated = False
        with self.assertRaises(CommandError):
            work_limit_target(g.config, 80.0)


class TestOpenWorkStroke(unittest.TestCase):
    """open() 带工作行程时停在限位内侧，不带则照旧顶停点。"""

    def test_open_stops_short_when_work_stroke_is_set(self):
        g, fake = make_gripper(start_rad=POS_CLOSED_RAD, stops=True)
        g.config.work_stroke_mm = 80.0
        res = g._actions.open(SPEED_MM_S)

        target, _, _ = work_limit_target(g.config, 80.0)
        self.assertTrue(res.ok, res)
        self.assertTrue(res.reached, res)
        self.assertFalse(res.stalled, res)
        self.assertAlmostEqual(res.limit_rad, POS_OPEN_RAD)
        self.assertAlmostEqual(fake.motor.pos, target, places=3)
        self.assertGreater(fake.motor.pos, POS_OPEN_RAD)   # 没顶到张开停点

    def test_open_presses_onto_the_stop_without_a_work_stroke(self):
        g, fake = make_gripper(start_rad=POS_CLOSED_RAD, stops=True)
        res = g._actions.open(SPEED_MM_S)
        self.assertTrue(res.stalled, res)
        self.assertAlmostEqual(fake.motor.pos, POS_OPEN_RAD, places=6)

    def test_work_stroke_at_or_over_the_travel_falls_back_to_press(self):
        g, fake = make_gripper(start_rad=POS_CLOSED_RAD, stops=True)
        g.config.work_stroke_mm = TRAVEL_RAD * RAD_TO_MM   # = 满行程 ⇒ 等同没设
        res = g._actions.open(SPEED_MM_S)
        self.assertTrue(res.stalled, res)
        self.assertAlmostEqual(fake.motor.pos, POS_OPEN_RAD, places=6)


class TestStallTorqueProtection(unittest.TestCase):
    """行进段力矩保护（≈7 N）：只在「没进压紧段 + 没跟上」时触发并失力。"""

    def test_mid_travel_block_trips_protection(self):
        g, fake = make_gripper(block_rad=POS_OPEN_RAD + 0.01)
        res = g._actions.close(SPEED_MM_S)

        self.assertTrue(res.protected, res)
        self.assertTrue(res.stalled, res)
        self.assertFalse(res.ok, res)

    def test_protection_releases_the_jaws(self):
        g, fake = make_gripper(block_rad=POS_OPEN_RAD + 0.01)
        res = g._actions.close(SPEED_MM_S)

        # res.steps 是触发那一帧；其后应全是失力帧（kp=kd=tau=0，位置不动）
        tail = fake.frames[res.steps:]
        self.assertTrue(tail, "触发后没有失力帧")
        for f in tail:
            self.assertEqual(f.kp, 0.0)
            self.assertEqual(f.kd, 0.0)
            self.assertEqual(f.dq, 0.0)
            self.assertEqual(f.tau_ff, 0.0)
            self.assertEqual(f.tau_nm, 0.0)
        self.assertAlmostEqual(fake.motor.pos, POS_OPEN_RAD + 0.01, places=6)

    def test_press_zone_does_not_trip_protection(self):
        # 顶到标定限位：压紧段领先已收窄，本来就该顶着力矩 —— 不触发
        g, fake = make_gripper(stops=True)
        res = g._actions.close(SPEED_MM_S)

        self.assertFalse(res.protected, res)
        self.assertTrue(res.ok, res)

    def test_cruise_high_torque_but_tracking_does_not_trip(self):
        # 阈值压到极低让巡航力矩也过阈；夹爪还在按指令速度跟进（rate 不慢）
        # ⇒ 不触发。「没跟上」这一条件是防误触发的关键。
        g, fake = make_gripper(stops=True)
        g.motion_config.stop_torque_nm = 0.01
        res = g._actions.close(SPEED_MM_S)

        self.assertFalse(res.protected, res)
        self.assertTrue(res.ok, res)
        over = [f for f in fake.frames
                if abs(f.tau_nm) >= g.motion_config.stop_torque_nm]
        self.assertTrue(over, "巡航段本该有超过阈值的力矩帧")

    def test_grasp_close_is_exempt(self):
        # grasp 的闭合段（press=False）要能夹住工件后转到保力 —— 不走力矩保护，
        # 否则每次夹取都会被当成保护性堵转打断。
        block = (POS_OPEN_RAD + POS_CLOSED_RAD) / 2
        g, fake = make_gripper(block_rad=block)
        g.motion_config = MotionConfig(
            sleep_fn=lambda _: None, monotonic_fn=tick_clock(0.1))
        # 保力爬升落到设定值需要时间，不然「有保力帧」这一条抓的是爬升中的帧
        res = g._actions.grasp(force_n=20.0, hold_s=1.2)

        self.assertTrue(res.stalled, res)     # 撞上工件（位置式堵转）
        self.assertTrue(res.ok, res)          # 保力正常，没被保护打断
        self.assertTrue([f for f in fake.frames if f.tau_ff == 2.0])  # 有保力帧


class TestForceApproachTerms(unittest.TestCase):
    """``force_approach_terms`` 的算术：三项加起来的压紧力矩不超预算。"""

    KP = 100.0
    KD = 2.0
    DT = 0.005

    def terms(self, force_n, speed_mm_s, budget_nm=None, ceiling_mm=4.0):
        budget = (force_n * UnitConversion.N_TO_NM * 0.9
                  if budget_nm is None else budget_nm)
        return force_approach_terms(self.KP, self.KD, speed_mm_s / RAD_TO_MM,
                                    self.DT, budget, ceiling_mm / RAD_TO_MM)

    def press(self, force_n, speed_mm_s, **kw):
        """一帧撞上工件时压出的力矩：``kp × 领先 + kd × 速度``。"""
        v_rad_s, kd, lead_rad = self.terms(force_n, speed_mm_s, **kw)
        return self.KP * lead_rad + kd * v_rad_s

    def test_the_three_terms_never_add_up_past_the_budget(self):
        for force_n in (0.5, 1.0, 5.0, 20.0, 40.0):
            for speed_mm_s in (5.0, 25.0, 50.0, 150.0):
                with self.subTest(force_n=force_n, speed_mm_s=speed_mm_s):
                    budget = force_n * UnitConversion.N_TO_NM * 0.9
                    self.assertLessEqual(self.press(force_n, speed_mm_s),
                                         budget + 1e-12)

    def test_no_budget_changes_nothing(self):
        v_rad_s, kd, lead_rad = force_approach_terms(
            self.KP, self.KD, 0.5, self.DT, 0.0, 0.05)
        self.assertAlmostEqual(v_rad_s, 0.5)
        self.assertAlmostEqual(kd, self.KD)
        self.assertAlmostEqual(lead_rad, 0.05)

    def test_the_lead_never_drops_below_one_frame(self):
        # 下限是一帧的位移：引擎本来就靠这条保住斜坡自己那一格，而它的力矩
        # 恰好就是「一帧那一格」，所以这条下限不会把预算撑破。
        for speed_mm_s in (5.0, 50.0, 150.0):
            v_rad_s, _kd, lead_rad = self.terms(1.0, speed_mm_s)
            self.assertGreaterEqual(lead_rad, v_rad_s * self.DT - 1e-12)

    def test_the_lead_never_exceeds_the_ceiling_it_was_given(self):
        _v, _kd, lead_rad = self.terms(400.0, 5.0, ceiling_mm=4.0)
        self.assertAlmostEqual(lead_rad, 4.0 / RAD_TO_MM)

    def test_a_setpoint_too_small_for_the_speed_slows_it_down(self):
        v_rad_s, kd, _lead = self.terms(0.5, 150.0)
        self.assertLess(v_rad_s * RAD_TO_MM, 150.0)
        # 慢到一帧那一格正好等于预算：这是唯一由设定值决定的接近速度
        self.assertAlmostEqual(self.KP * v_rad_s * self.DT,
                               0.5 * UnitConversion.N_TO_NM * 0.9, places=12)
        self.assertAlmostEqual(kd, 0.0, places=12)


class TestForceApproachPressBudget(unittest.TestCase):
    """带设定力的接近段（``grasp`` 的闭合段）压出的力矩不超过设定值。

    这一段是位置帧：撞上工件时力矩由驱动器自己算，``kp × 领先 + kd × 指令
    速度``，与这一次夹取设定的力无关。默认那条 4 mm 行进段上限在本假件的
    ``RAD_TO_MM`` 下（``kp=100``）折算是 5.4 Nm ≈ 54 N，所以「夹住工件」和
    「先拿 54 N 撞一下」曾经是同一件
    事 —— 5 N 的夹取和 40 N 的夹取撞上去的力矩一模一样。现在这一段带着设定力
    的预算走（见 :func:`force_approach_terms`），撞上那一下跟着设定的力走。
    """

    BLOCK = (POS_OPEN_RAD + POS_CLOSED_RAD) / 2            # 工件

    def _grasp(self, force_n, speed_mm_s=SPEED_MM_S, block=BLOCK, hold_s=0.2):
        motion = MotionConfig(sleep_fn=lambda _: None,
                              monotonic_fn=tick_clock(0.1))
        motion.grasp_speed_mm_s = speed_mm_s
        g, fake = make_gripper(block_rad=block, stops=True, motion=motion)
        return g, fake, g._actions.grasp(force_n=force_n, hold_s=hold_s)

    @staticmethod
    def _presses(fake, block):
        """顶在工件上的那些帧压出的力矩。

        只取夹爪真的**顶在工件上**的帧（``pos_after`` 正好停在挡块上）：假件的
        力矩模型就是 ``kp × (指令 − 实测)``，只有顶住不动时才等于真机撞上工件
        那一下；自由行进时它会绕着指令来回超调，|指令 − 实测| 量到的是「指令
        落后读数多少」，不是压紧力。保力帧不算（它们只发前馈）。
        """
        return [abs(f.tau_nm) + f.kd * abs(f.dq)
                for f in fake.frames
                if f.tau_ff == 0.0 and abs(f.pos_after - block) < 1e-12]

    def _budget(self, force_n):
        return force_n * UnitConversion.N_TO_NM * MotionConfig().press_safety

    def test_the_frame_never_presses_past_the_setpoint(self):
        for force_n in (5.0, 20.0, 40.0):
            for speed_mm_s in (25.0, 50.0, 150.0):
                with self.subTest(force_n=force_n, speed_mm_s=speed_mm_s):
                    _g, fake, res = self._grasp(force_n, speed_mm_s)
                    presses = self._presses(fake, self.BLOCK)
                    self.assertTrue(presses, "没有顶在工件上的帧")
                    self.assertLessEqual(max(presses),
                                         self._budget(force_n) + 1e-9)
                    self.assertTrue(res.stalled, res)      # 照样撞得到工件

    def test_the_press_follows_the_setpoint(self):
        peaks = []
        for force_n in (10.0, 20.0, 40.0):
            _g, fake, _res = self._grasp(force_n)
            peaks.append(max(self._presses(fake, self.BLOCK)))
        for force_n, peak in zip((10.0, 20.0, 40.0), peaks):
            self.assertAlmostEqual(peak, self._budget(force_n), places=9)
        self.assertLess(peaks[0], peaks[1])
        self.assertLess(peaks[1], peaks[2])

    def test_the_same_block_reached_by_close_presses_fifty_four_newtons(self):
        """不带预算那一路（普通 close）在同一块工件上还是 54 N —— 差别来自预算。"""
        block = POS_OPEN_RAD + 0.01
        g_close, fake_close = make_gripper(block_rad=block)
        g_close._actions.close(SPEED_MM_S)
        unbounded = max(abs(f.tau_nm) for f in fake_close.frames)

        _g, fake_grasp, res = self._grasp(5.0, block=block)

        self.assertAlmostEqual(
            unbounded,
            g_close.config.kp * g_close.motion_config.max_lead_mm / RAD_TO_MM,
            places=4)
        self.assertGreater(unbounded, 10.0 * self._budget(5.0))   # ≈54 N 的撞
        self.assertLess(max(self._presses(fake_grasp, block)),
                        self._budget(5.0) + 1e-9)
        self.assertTrue(res.stalled, res)

    def test_a_low_setpoint_decides_the_speed(self):
        """预算连一帧那一格都占满时，接近速度由设定值定 —— 不是由配置定。"""
        g, fake, res = self._grasp(1.0, speed_mm_s=150.0)
        cap_rad_s = self._budget(1.0) / (g.config.kp * DT)

        self.assertAlmostEqual(max(abs(f.dq) for f in fake.frames),
                               cap_rad_s, places=9)
        self.assertLess(cap_rad_s * RAD_TO_MM, 150.0)
        self.assertTrue(res.stalled, res)

    def test_an_empty_grasp_still_reaches_the_target(self):
        """空载不许被预算改成堵转：小领先只是压得轻，不是走不动。"""
        g, fake, res = self._grasp(5.0, block=None)

        self.assertTrue(res.reached, res)
        self.assertFalse(res.stalled, res)
        self.assertTrue(res.ok, res)
        self.assertAlmostEqual(fake.motor.pos, res.target_rad,
                               delta=g.motion_config.reach_tol)

    def test_the_hold_still_lands_on_the_setpoint(self):
        """保力段照旧走到设定值，而且是从压紧力矩**往上爬**到的，不是一帧跳上去。

        时长得给够：假件只按 ``kp × (指令 − 实测)`` 报力矩（它不建阻尼），交接
        那一刻报的是领先那一项（0.45 Nm），比真机撞上工件那一下（1.8 Nm）低，
        按假件的尺子爬升距离就长。
        """
        _g, fake, res = self._grasp(20.0, hold_s=1.0)

        self.assertEqual(res.force_n, 20.0)
        self.assertTrue(res.cycles > 0, res)
        hold = [f for f in fake.frames if f.tau_ff != 0.0]
        self.assertTrue(hold, "没有保力帧")
        self.assertLess(hold[0].tau_ff, 2.0)            # 交接处还在设定值之下
        landed = [i for i, f in enumerate(hold) if f.tau_ff == 2.0]
        self.assertTrue(landed, "保力段没有到达设定值")
        for f in hold[landed[0]:]:
            self.assertAlmostEqual(f.tau_ff, 2.0, places=9)


class TestTheApproachRefusesToHandOverWithoutBite(unittest.TestCase):
    """接近段从起点就没动过时不许交接：那是「没能脱离静摩擦」，不是接触。

    2026-10-29 台架实测（2 N 夹取）：接近段压 0.18 Nm，整个 0.25 s 堵转窗口只挪了
    0.0004 rad；它被判成「顶到工件」，接手的保力段压 0.20 Nm —— 设定值的 1.0 倍，
    比接近段高出一成，正好越过脱离力 —— 于是夹爪带着设定力在自由行程里推了 19 mm，
    夹取点报在工件前 19–39 mm。操作者看到的是「先不动，然后飞速闭合」。

    判据见 ``MotionConfig.no_bite_mm``：带设定力的接近段里，位移小于这个值的停住
    不算接触 —— 报 ``no_bite``、撤力，保力段一帧都不发。而且只在预算低于
    ``MotionConfig.breakaway_nm`` 时才这么判：预算远高于脱离力还没动，那是真顶上了
    东西（见 ``test_a_workpiece_at_the_start_is_contact_above_the_breakaway``）。
    """

    #: 实测脱离力落在 1.8–2.0 N，取 0.19 Nm（1.9 N）：2 N 夹取的预算 0.18 Nm 压不动，
    #: 3 N 的 0.27 Nm 压得动。
    BREAKAWAY_NM = 0.19
    #: 纯力矩的保力帧在假台上要靠这个才动（默认模型里 kp=0 一点不动）。
    CREEP = 5.0

    def _grasp(self, force_n, hold_s=0.2, *, no_bite_mm=None, **kw):
        motion = MotionConfig(sleep_fn=lambda _: None,
                              monotonic_fn=tick_clock(0.1))
        if no_bite_mm is not None:
            motion.no_bite_mm = no_bite_mm
        g, fake = make_gripper(stops=True, motion=motion,
                               breakaway_tau_nm=self.BREAKAWAY_NM,
                               creep_rad_s_per_nm=self.CREEP, **kw)
        return g, fake, g._actions.grasp(force_n=force_n, hold_s=hold_s)

    @staticmethod
    def _budget(force_n):
        return force_n * UnitConversion.N_TO_NM * MotionConfig().press_safety

    @staticmethod
    def _travelled_mm(fake):
        return abs(fake.motor.pos - POS_OPEN_RAD) * RAD_TO_MM

    def test_the_old_handover_walks_the_jaws_across_the_whole_free_stroke(self):
        """修复前的行为，钉在这里当对照：保力段把夹爪一路推到底。

        ``no_bite_mm=0`` 关掉拒夹，就是修复前的样子。零位移被判成接触之后，保力
        段的 0.20 Nm 越过了 0.19 Nm 的脱离力，夹爪在整段自由行程里被推着走 ——
        这正是台架上「先不动，然后飞速闭合」的那一台。下一条用例要证明的，就是
        拒夹把这段行程拦在了开头。两条用的是同一个假台，只差一个 ``no_bite_mm``。
        """
        _g, fake, res = self._grasp(2.0, no_bite_mm=0.0)

        self.assertTrue(res.stalled, res)       # 修复前：零位移被当成接触
        self.assertFalse(res.no_bite, res)
        self.assertGreater(res.cycles, 0, res)
        # 0.2 s 的保力就推了十几毫米（台架那次是 19 mm，夹取点报在工件前），
        # 而修复后同一个动作一毫米都不走 —— 对比就在下一条用例。
        self.assertGreater(self._travelled_mm(fake), 10.0,
                           f"保力段应该把夹爪推着走，实际只走了 "
                           f"{self._travelled_mm(fake):.1f} mm")

    def test_the_refusal_stops_that_stroke_before_it_starts(self):
        _g, fake, res = self._grasp(2.0)

        self.assertLess(self._budget(2.0), self.BREAKAWAY_NM)      # 前提
        self.assertFalse(res.ok, res)
        self.assertTrue(res.no_bite, res)
        self.assertFalse(res.stalled, res)          # 停住 ≠ 顶到工件
        self.assertFalse(res.reached, res)
        self.assertEqual(res.cycles, 0)
        self.assertLess(self._travelled_mm(fake), MotionConfig().no_bite_mm,
                        "拒夹之后夹爪不许离开起点")
        self.assertEqual([f for f in fake.frames if f.tau_ff != 0.0], [],
                         "保力段一帧都不许发：它是纯力矩源，没有位置也没有限速")
        self.assertEqual(fake.frames[-1].kp, 0.0, "结尾要撤力")

    def test_the_setpoint_above_the_breakaway_still_grasps(self):
        _g, fake, res = self._grasp(3.0)

        self.assertGreater(self._budget(3.0), self.BREAKAWAY_NM)   # 前提
        self.assertFalse(res.no_bite, res)
        self.assertTrue(res.reached, res)
        self.assertTrue(res.ok, res)
        self.assertGreater(res.cycles, 0)

    def test_a_workpiece_at_the_start_is_contact_above_the_breakaway(self):
        """预算远高于脱离力时，零位移是**顶上东西**，不是没能脱离。

        电机分不出这两种零位移，判据改用预算来分：0.20 Nm 以上还不动，只可能是
        有东西挡着。这时该交接 —— 保力段照着设定值压，压的就是用户要的力。少了
        这一条，一个已经贴在张开侧的工件、以及 ``disable()`` 之后原地
        重夹（那一次松开不发张开指令），都会变成一次失败的夹取。
        """
        start = POS_OPEN_RAD
        block = start + 0.001 * (POS_CLOSED_RAD - POS_OPEN_RAD)   # ≈ 0.08 mm
        motion = MotionConfig(sleep_fn=lambda _: None,
                              monotonic_fn=tick_clock(0.1))
        g, _fake = make_gripper(stops=True, motion=motion, block_rad=block)
        res = g._actions.grasp(force_n=20.0, hold_s=0.2)

        self.assertFalse(res.no_bite, res)
        self.assertTrue(res.stalled, res)
        self.assertTrue(res.ok, res)
        self.assertGreater(res.cycles, 0)

    def test_a_workpiece_at_the_start_is_refused_below_the_breakaway(self):
        """代价写在测试里：预算不到脱离力时，贴在起点上的工件也会被拒。

        和上一条同一块挡块、同一个起点，只把设定值降到 2 N。拒夹的代价是操作者
        看到一次失败、重摆一次；放行的代价是一次带力冲程（上一条用例里的整段）。
        两边不对称，所以选拒夹。
        """
        start = POS_OPEN_RAD
        block = start + 0.001 * (POS_CLOSED_RAD - POS_OPEN_RAD)
        motion = MotionConfig(sleep_fn=lambda _: None,
                              monotonic_fn=tick_clock(0.1))
        g, fake = make_gripper(stops=True, motion=motion, block_rad=block)
        res = g._actions.grasp(force_n=2.0, hold_s=0.2)

        self.assertTrue(res.no_bite, res)
        self.assertFalse(res.ok, res)
        self.assertEqual([f for f in fake.frames if f.tau_ff != 0.0], [])


class TestNonFiniteInputIsRefused(unittest.TestCase):
    """非有限值不许变成运动：拒绝带原因，而且**一帧都不发**。

    这里钉住的是 2026-10-10 审计里实测到的旧行为，两条都在假台上复现过：

    - ``set_gripper_distance(nan)``：限位 clamp 的 ``min`` / ``max`` 与 NaN 的
      比较全为假，NaN 静默变成一个**真实位置**（闭合侧端点），于是夹爪朝那个
      止点压过去；
    - ``grasp(nan)``：NaN 一路传到帧上，编码器把它抬到量程上端，线上是 +10 Nm。

    一帧都不发这一条是重点：NaN 不是「大」或「小」的边界，它没有意义，所以拒绝
    必须发生在机构动之前 —— 夹爪先动一下再报错是不行的。
    """

    NON_FINITE = (float("nan"), float("inf"), float("-inf"))

    def test_grasp_rejects_a_non_finite_force_before_it_moves(self):
        for bad in self.NON_FINITE:
            g, fake = make_gripper(stops=True)
            with self.assertRaises(CommandError):
                g._actions.grasp(bad, hold_s=0.0)
            self.assertEqual(fake.frames, [], f"grasp({bad!r}) 竟然先动了")

    def test_grasp_rejects_a_non_finite_hold_before_it_moves(self):
        for bad in self.NON_FINITE:
            g, fake = make_gripper(stops=True)
            with self.assertRaises(CommandError):
                g._actions.grasp(20.0, hold_s=bad)
            self.assertEqual(fake.frames, [], f"hold_s={bad!r} 竟然先动了")

    def test_the_motion_config_setpoint_is_checked_too(self):
        """``force_n=None`` 时用的是 ``MotionConfig.force_n`` —— 同一条规则。"""
        g, fake = make_gripper(
            stops=True,
            motion=MotionConfig(force_n=float("nan"), sleep_fn=lambda _: None))
        with self.assertRaises(CommandError):
            g._actions.grasp(hold_s=0.0)
        self.assertEqual(fake.frames, [])


class TestUncalibratedConfigRefusesMotion(unittest.TestCase):
    """No motion on an uncalibrated configuration — including the paths that
    clamp a target instead of using ``limit_target`` / ``press_target``.

    With placeholder limits (``1.14`` / ``0.0`` / ``rad_to_mm=75.44``) the
    clamp is arithmetic on numbers that describe no real travel and produces a
    real target.  The 2026-10-10 audit measured ``goto(40.0)`` returning True
    with a final ``q=0.6098 kp=100`` while the real closed stop was at
    ``+0.0521 rad`` — about 56 Nm pressing into the stop.

    The gate is before the clamp: the refusal must happen before the mechanism
    moves.
    """

    @staticmethod
    def fresh():
        """A fresh object still carrying the placeholder limits."""
        g = Gripper("vcan0", can_id=0x08)
        fake = FakeCanBus(pos=0.0)
        g._can = fake
        g._connected = True
        g._enabled = True
        g.config.pos_closed_rad = 1.14
        g.config.pos_open_rad = 0.0
        g.config.rad_to_mm = UnitConversion.RAD_TO_MM
        g.config.calibrated = False
        return g, fake

    def test_every_motion_refuses_and_sends_nothing(self):
        calls = [
            lambda g: g.set_gripper_distance(40.0),
            lambda g: g.set_motor_angle(0.0),
            lambda g: g.open(),
            lambda g: g.close(),
            lambda g: g.grasp(force_n=5.0, hold_s=0.05),
            lambda g: g.release(5.0),
        ]
        for call in calls:
            g, fake = self.fresh()
            with self.assertRaises(CommandError):
                call(g)
            self.assertEqual(fake.frames, [])

    def test_the_placeholder_target_lands_outside_the_real_travel(self):
        """Show what the gate blocks: the placeholder target is outside the
        real machine's travel."""
        g, _ = self.fresh()
        cfg = g.config
        target_rad = (cfg.pos_closed_rad
                      - cfg.close_sign * 40.0 / cfg.rad_to_mm)
        lo = min(POS_CLOSED_RAD, POS_OPEN_RAD)
        hi = max(POS_CLOSED_RAD, POS_OPEN_RAD)
        self.assertAlmostEqual(target_rad, 0.61, places=2)   # 占位值算出的 q
        self.assertTrue(target_rad > hi or target_rad < lo,
                        f"占位值算出的 {target_rad:.4f} rad 落在真机行程 "
                        f"[{lo:.4f}, {hi:.4f}] 之内了 —— 这条审计结论已过期")

    def test_a_calibrated_config_is_untouched(self):
        """No false positives: these calls still send frames when calibrated."""
        for call in (lambda g: g.set_gripper_distance(40.0),
                     lambda g: g.set_motor_angle(0.0),
                     lambda g: g.open(),
                     lambda g: g.close(),
                     lambda g: g.grasp(force_n=5.0, hold_s=0.05)):
            g, fake = make_gripper(stops=True)
            self.assertTrue(call(g))
            self.assertTrue(fake.frames, "标定好的配置上竟然一帧都没发")

    def test_a_calibrated_but_zero_travel_config_is_refused_too(self):
        """Calibrated but both ends identical: the clamp collapses to a point."""
        g, fake = make_gripper(stops=True)
        g.config.pos_open_rad = g.config.pos_closed_rad
        with self.assertRaises(CommandError):
            g.set_gripper_distance(40.0)
        self.assertEqual(fake.frames, [])


class TestFreshStateRefusesAnUnansweredCache(unittest.TestCase):
    """缓存还新不等于位置可信：0xCC 没人答，就不许规划运动。"""

    def test_a_recent_cache_is_not_enough_when_nothing_answers(self):
        g, fake = make_gripper(stops=True)
        g.get_state()                     # 先让缓存里有一帧、age 很小
        fake.delivers = False             # 总线安静
        fake.refresh_ok = False           # 0xCC 也发不出去

        with self.assertRaises(CommError):
            g._actions.open()


class TestFinalReachUsesAFreshFrame(unittest.TestCase):
    """移动结束判 reached/ok 前必须再要一帧，不能拿循环里的缓存。"""

    def test_the_end_of_a_move_requests_a_status_frame(self):
        g, fake = make_gripper(stops=True)

        res = g._actions.open(SPEED_MM_S)

        self.assertTrue(res.ok, res)
        self.assertGreaterEqual(
            fake.refresh_calls, 2,
            "移动结束时没有重新要状态帧 —— reached/ok 用的是缓存")


class TestGraspHoldCommError(unittest.TestCase):
    """保力中途断总线：撤力、返回 ok=False，不把异常抛给调用方。"""

    def test_a_mid_hold_comm_error_releases_and_reports_failure(self):
        block = (POS_OPEN_RAD + POS_CLOSED_RAD) / 2.0
        g, fake = make_gripper(block_rad=block)

        with mock.patch.object(g._actions, "_hold_force",
                               side_effect=CommError("bus down")):
            res = g._actions.grasp(force_n=5.0, hold_s=0.2)

        self.assertFalse(res.ok, res)
        self.assertEqual(res.cycles, 0)
        self.assertTrue(res.stalled, "接近段应当先夹到工件")
        self.assertTrue(
            any(f.kp == 0.0 and f.kd == 0.0 and f.tau_ff == 0.0
                for f in fake.frames),
            "保力中断后没有撤力帧 —— 最后一档力矩留在了总线上")
        self.assertTrue(g._actions._hold_done.is_set(),
                        "grasp() 的 finally 没有收尾")


class TestMotionConfigValidation(unittest.TestCase):

    def test_non_finite_or_negative_stall_speed_is_rejected(self):
        for value in (-1.0, float("nan"), float("inf")):
            with self.subTest(value=value):
                with self.assertRaises(ValueError):
                    MotionConfig(stall_speed_mm_s=value)


if __name__ == "__main__":
    unittest.main()
