"""actions 引擎的硬件无关测试：斜坡、领先封顶、堵转窗口、保力、使能。"""

from __future__ import annotations

import unittest

import _sdkpath  # noqa: F401
from litegrip import (CommandError, GraspResult, MotionConfig, UnitConversion,
                      force_approach_terms, limit_target, press_target,
                      work_limit_target)
from litegrip.actions import GripperActions

from fake_can import (DT, POS_CLOSED_RAD, POS_OPEN_RAD, RAD_TO_MM,
                      make_gripper, planned_steps, tick_clock)

SPEED_MM_S = 50.0
SPEED_RAD_S = SPEED_MM_S / RAD_TO_MM          # ≈ 0.674 rad/s
TRAVEL_RAD = abs(POS_CLOSED_RAD - POS_OPEN_RAD)


def press_caps(g):
    """引擎算出来的 ``(行进段领先上限, 压紧段领先上限)`` rad。"""
    cfg = g.motion_config
    min_cap = SPEED_RAD_S * cfg.frame_interval
    return (max(cfg.max_lead_mm / RAD_TO_MM, min_cap),
            max(cfg.stop_lead_mm / RAD_TO_MM, min_cap))


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


class TestRamp(unittest.TestCase):
    """3 / 4 / 5：空载斜坡的形状（顶限位）。"""

    def test_close_presses_onto_the_stop(self):
        g, fake = make_gripper(stops=True)
        self.assertIsInstance(g.actions, GripperActions)
        res = g.close(SPEED_MM_S)

        self.assertTrue(res.ok, res)
        self.assertTrue(res.stalled, res)             # 成功 = 撞到限位
        self.assertFalse(res.reached, res)            # 目标在限位外侧，到不了
        self.assertAlmostEqual(res.limit_rad, POS_CLOSED_RAD)
        self.assertLess(abs(res.state.position_rad - POS_CLOSED_RAD),
                        g.motion_config.stop_tol)

    def test_open_presses_onto_the_other_stop(self):
        g, fake = make_gripper(start_rad=POS_CLOSED_RAD, stops=True)
        res = g.open(SPEED_MM_S)

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
        res = g.close(SPEED_MM_S)

        self.assertEqual(res.steps, len(fake.frames))
        self.assertGreaterEqual(res.steps, ramp_steps)   # 没在半路停
        self.assertLess(res.steps, total)                # 也没白跑完保压段

    def test_velocity_feedforward_only_during_ramp(self):
        g, fake = make_gripper(stops=True)
        _, ramp_steps, _, _ = planned_steps(g, SPEED_MM_S, press=True)
        g.close(SPEED_MM_S)

        self.assertAlmostEqual(fake.frames[0].dq, SPEED_RAD_S)   # 闭合 = +rad
        self.assertAlmostEqual(fake.frames[ramp_steps - 1].dq, SPEED_RAD_S)
        for f in fake.frames[ramp_steps:]:
            self.assertEqual(f.dq, 0.0)


class TestReverseMount(unittest.TestCase):
    """反装（``pos_closed_rad < pos_open_rad``）：同一个动作走相反方向。"""

    def test_close_presses_onto_the_low_rad_stop(self):
        g, fake = make_gripper(reverse=True, stops=True)
        res = g.close(SPEED_MM_S)

        self.assertTrue(res.ok, res)
        self.assertTrue(res.stalled, res)
        self.assertAlmostEqual(res.limit_rad, POS_OPEN_RAD)   # 闭合在数值小的一侧
        self.assertLess(abs(res.state.position_rad - POS_OPEN_RAD),
                        g.motion_config.stop_tol)
        self.assertAlmostEqual(fake.frames[0].dq, -SPEED_RAD_S)  # 闭合 = -rad

    def test_open_presses_onto_the_high_rad_stop(self):
        g, fake = make_gripper(reverse=True, stops=True)
        res = g.open(SPEED_MM_S)

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

    def test_goto_rad_clamps_between_the_two_limits(self):
        g, fake = make_gripper(reverse=True)
        lo, hi = POS_OPEN_RAD, POS_CLOSED_RAD
        seen = []
        fake.control_mit_stream = lambda q_target, **kw: seen.append(q_target) or True

        g.goto_rad(hi + 5.0)
        g.goto_rad(lo - 5.0)
        self.assertAlmostEqual(seen[0], hi)
        self.assertAlmostEqual(seen[1], lo)

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
        res = g.close(SPEED_MM_S)

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
        res = g.close(SPEED_MM_S)

        self.assertTrue(res.stalled, res)
        self.assertFalse(res.ok, res)
        self.assertLess(res.steps, total)
        self.assertAlmostEqual(fake.motor.pos, block, places=6)


class TestPressZone(unittest.TestCase):
    """顶限位的力矩是**有界**的：压紧段收窄到 stop_lead_mm。"""

    def test_pressing_torque_is_under_the_rating(self):
        g, fake = make_gripper(stops=True)
        res = g.close(SPEED_MM_S)
        _, stop_cap = press_caps(g)

        max_tau = max(abs(f.tau_nm) for f in fake.frames)
        self.assertAlmostEqual(max_tau, g.config.kp * stop_cap, places=4)
        self.assertLess(max_tau, 3.0)                 # DM4310 额定 3 Nm
        self.assertTrue(res.ok, res)

    def test_default_press_force_is_the_gentle_tier(self):
        """默认 0.7 mm 压紧：力矩 ≈ kp × 0.7 / rad_to_mm ≈ 0.94 Nm。"""
        g, fake = make_gripper(stops=True)
        g.close(SPEED_MM_S)

        self.assertAlmostEqual(g.motion_config.stop_lead_mm, 0.7)
        expected = g.config.kp * 0.7 / g.config.rad_to_mm
        max_tau = max(abs(f.tau_nm) for f in fake.frames)
        self.assertAlmostEqual(max_tau, expected, places=4)
        self.assertLess(max_tau, 1.2)                 # 「不太猛」的量化断言
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
        g.close(SPEED_MM_S)

        _, stop_cap = press_caps(g)
        tail = fake.frames[ramp_steps:]
        self.assertTrue(tail)                          # 确实进了保压段
        self.assertLessEqual(
            max(abs(f.q - f.pos_after) for f in tail), stop_cap + 1e-9)


class TestStallFalsePositives(unittest.TestCase):
    """8 / 9：粘滑不能误判堵转；非顶限位路径的保压段不算堵转。"""

    def test_sticky_dead_band_does_not_stall_mid_travel(self):
        # 闭合侧实测约 0.0103 rad 一跳的粘滑；窗口净位移仍远大于阈值
        g, _ = make_gripper(sticky_rad=0.0103, stops=True)
        _, ramp_steps, _, _ = planned_steps(g, SPEED_MM_S, press=True)
        res = g.close(SPEED_MM_S)

        self.assertGreaterEqual(res.steps, ramp_steps)  # 一路走到限位才停
        self.assertTrue(res.ok, res)

    def test_settle_phase_is_not_a_stall_without_press(self):
        # grasp 的闭合段（press=False）停在限位内侧，保压段不该判堵转
        g, fake = make_gripper()
        _, ramp_steps, settle_steps, total = planned_steps(g, SPEED_MM_S)
        res = g.actions._move_to_limit("close", SPEED_MM_S)

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
        return g.actions._move_to_limit("close", SPEED_MM_S)

    def test_default_tolerance_accepts(self):
        self.assertTrue(self._close_short(0.02).reached)

    def test_tight_tolerance_rejects(self):
        self.assertFalse(self._close_short(0.01).reached)


class TestMoveResultBool(unittest.TestCase):
    """``bool(MoveResult)`` 是 ``ok``，不是旧的 ``reached and not stalled``。"""

    def test_press_success_is_truthy_though_it_stalled(self):
        g, _ = make_gripper(stops=True)
        res = g.close(SPEED_MM_S)
        self.assertTrue(res.stalled)
        self.assertFalse(res.reached)
        self.assertEqual(bool(res), res.ok)
        self.assertTrue(bool(res))

    def test_blocked_halfway_is_falsy(self):
        g, _ = make_gripper(block_rad=POS_OPEN_RAD + 0.01)
        res = g.close(SPEED_MM_S)
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
        res = g.grasp(force_n=20.0, hold_s=1.2)

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
        res = g.grasp(force_n=20.0, hold_s=0.4)

        self.assertTrue(res.stalled)      # 撞上工件
        self.assertFalse(res.reached)     # 压不到空载目标 —— 正常
        self.assertTrue(res.ok)           # 但保力是好的

    def test_fault_during_hold_aborts(self):
        block = (POS_OPEN_RAD + POS_CLOSED_RAD) / 2
        g, fake = make_gripper(block_rad=block)
        g.motion_config = self._motion(0.4)
        fake.error_when_holding = 0x9              # 一开始施力就报故障
        res = g.grasp(force_n=20.0, hold_s=0.4)

        self.assertFalse(res.ok)
        self.assertEqual(res.cycles, 1)            # 第一片之后就被叫停
        self.assertEqual(res.state.error_code, 0x9)


class TestForceHoldIsTorqueOnly(unittest.TestCase):
    """保力只下发前馈力矩（kp=kd=0）：力不随工件屈服而衰减。

    保力帧带上位置刚度时，MIT 律里的 ``kp × (q - 实测位置)`` 会随夹爪位移变
    化 —— 工件在设定力下让位，夹爪跟着走，这一项就从设定力里扣掉一截，现象
    是「先夹到设定力，过一会儿掉到某个更小的值」。闭合侧约 0.0103 rad 的粘滑
    死区每走一格，``hold_kp=150`` 就换算成约 15 N 的力误差。
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
        res = g.grasp(force_n=20.0, hold_s=1.2)

        self.assertTrue(res.ok, res)
        hold = [f for f in fake.frames if f.tau_ff == 2.0]      # 20 N → 2.0 Nm
        self.assertTrue(hold, "没有保力帧")
        # 工件一直在让位（夹爪被它带着往里走）—— 不然这条用例是空的
        self.assertGreater(hold[-1].pos_after - hold[0].pos_after, 0.005)
        # 让位再深，读到的力矩也得还是设定值
        for f in hold:
            self.assertAlmostEqual(f.tau_nm, 2.0, places=6)

    def test_the_hold_frame_carries_no_gains(self):
        # hold_kp / hold_kd 已废弃：显式设上也不该出现在保力帧里
        g, fake = self._yielding_gripper()
        g.motion_config = self._motion(hold_kp=300.0, hold_kd=9.0)
        # 同上：给够时间让爬升落到设定值，才取得到「停在设定值上」的保力帧
        g.grasp(force_n=20.0, hold_s=1.2)

        hold = [f for f in fake.frames if f.tau_ff == 2.0]
        self.assertTrue(hold, "没有保力帧")
        for f in hold:
            self.assertEqual(f.kp, 0.0)
            self.assertEqual(f.kd, 0.0)

    def test_set_force_carries_no_gains(self):
        g, fake = self._yielding_gripper()
        fake.motor.pos = self.OBJECT_RAD            # 已经夹在工件上
        # 时长给够，让爬升能落到设定值（力矩从 0 起步要 20 N ÷ 20 N/s = 1.0 s）
        g.set_force(20.0, duration=1.0)

        self.assertTrue(fake.frames)
        for f in fake.frames:
            self.assertEqual(f.kp, 0.0)
            self.assertEqual(f.kd, 0.0)
        # 从飞行力矩（这里恒为 0）按固定速率爬，末帧正好落在 2.0 Nm
        self.assertAlmostEqual(fake.frames[0].tau_nm, 0.01, places=9)
        self.assertAlmostEqual(fake.frames[-1].tau_nm, 2.0, places=6)
        self.assertGreater(fake.motor.pos, self.OBJECT_RAD)     # 工件确实让位了


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
        res = g.grasp(force_n=force_n, hold_s=hold_s)
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
        g.actions._hold_force(20.0, 0.4)

        hold = [f for f in fake.frames if f.kp == 0.0]
        self.assertTrue(hold, "没有保力帧")
        self.assertAlmostEqual(hold[0].tau_ff, 2.0, places=9)



class TestSetForceRampsToo(unittest.TestCase):
    """``set_force`` 是另一条下发恒定前馈力矩的保力路径，同样按时率爬。

    ``duration`` 是**爬到设定值之后**继续保力的时间，不含爬升本身，所以调用的
    墙钟是 ``climb + duration``；已经在设定值上时爬升为 0 帧，调用就是 ``duration``。
    """

    def test_the_climb_starts_from_the_torque_in_flight_and_lands(self):
        g, fake = make_gripper()
        g.motion_config = MotionConfig(sleep_fn=lambda _: None,
                                       monotonic_fn=tick_clock(0.1))
        fake.motor.tau = 1.0                         # 交接时飞行力矩 1.0 Nm
        g.set_force(20.0, duration=1.0)

        step = 20.0 * DT * 0.1                       # 0.01 Nm/帧
        taus = [f.tau_ff for f in fake.frames]
        self.assertAlmostEqual(taus[0], 1.0 + step, places=9)
        # 从 1.0 爬到 2.0 要 100 帧；之后一直停在 2.0
        self.assertAlmostEqual(taus[99], 2.0, places=9)
        for tau in taus[99:]:
            self.assertAlmostEqual(tau, 2.0, places=9)
        for f in fake.frames:
            self.assertEqual(f.kp, 0.0)
            self.assertEqual(f.kd, 0.0)

    def test_the_call_ramps_then_holds_for_duration(self):
        # duration 保的是**爬升之后**的时间：先把 1.0 Nm 爬 100 帧到设定值，
        # 再到 20 N 上保 0.3 s（60 帧）。墙钟因此是 climb + duration。
        g, fake = make_gripper()
        g.motion_config = MotionConfig(sleep_fn=lambda _: None,
                                       monotonic_fn=tick_clock(0.1))
        fake.motor.tau = 1.0                         # 交接时飞行力矩 1.0 Nm
        g.set_force(20.0, duration=0.3)

        taus = [f.tau_ff for f in fake.frames]
        climb = 100                                  # 1.0 Nm ÷ 0.01 Nm/帧
        hold = int(round(0.3 / DT))                  # 60 帧
        self.assertEqual(len(taus), climb + hold)
        # 爬升最后一帧正好落到设定值（2.0 Nm），其后每一帧都停在上面 ——
        # 设定值上的帧数 = 1 帧落点 + duration 的保力帧
        first_at = taus.index(2.0)
        self.assertEqual(first_at, climb - 1)
        self.assertTrue(all(t == 2.0 for t in taus[first_at:]))
        self.assertEqual(len(taus[first_at:]), hold + 1)

    def test_a_call_already_at_the_setpoint_is_just_the_hold(self):
        # 飞行力矩已在设定值上（或已越过）：没有可爬的，爬升 0 帧，调用就是
        # duration —— 不能为爬升多补一帧。
        for in_flight in (2.0, 3.0):
            with self.subTest(in_flight=in_flight):
                g, fake = make_gripper()
                g.motion_config = MotionConfig(
                    sleep_fn=lambda _: None, monotonic_fn=tick_clock(0.1))
                fake.motor.tau = in_flight
                g.set_force(20.0, duration=0.3)

                taus = [f.tau_ff for f in fake.frames]
                self.assertEqual(len(taus), int(round(0.3 / DT)))    # 60 帧
                for tau in taus:
                    self.assertAlmostEqual(tau, 2.0, places=9)

    def test_duration_zero_still_completes_the_climb(self):
        # duration=0 表示「爬到设定值就返回」：爬升照样要爬完，只是不保力。
        g, fake = make_gripper()
        g.motion_config = MotionConfig(sleep_fn=lambda _: None,
                                       monotonic_fn=tick_clock(0.1))
        fake.motor.tau = 1.0
        g.set_force(20.0, duration=0.0)

        taus = [f.tau_ff for f in fake.frames]
        self.assertEqual(len(taus), 100)             # 全是爬升帧，没有保力帧
        self.assertAlmostEqual(taus[0], 1.01, places=9)
        self.assertAlmostEqual(taus[-1], 2.0, places=9)

    def test_it_polls_once_per_emitted_frame(self):
        # 换掉 control_mit_stream 时不能把每帧的 poll 也丢了 —— 那是整段调用里唯一
        # 收状态帧的地方（control_mit → poll → sleep）。假总线只数 poll 次数。
        g, fake = make_gripper()
        g.motion_config = MotionConfig(sleep_fn=lambda _: None,
                                       monotonic_fn=tick_clock(0.1))
        g.set_force(20.0, duration=0.2)              # 200 帧爬升 + 40 帧保力

        self.assertEqual(len(fake.frames), 240)
        self.assertEqual(fake.poll_calls, len(fake.frames))


class TestEnable(unittest.TestCase):
    """14：使能重试 / 真故障先清。"""

    def test_retries_until_err_is_one(self):
        g, fake = make_gripper(err=0, initialize_results=[False, False, True])
        res = g.enable()

        self.assertTrue(res.ok)
        self.assertEqual(res.tries, 3)
        self.assertEqual(res.state.error_code, 1)
        self.assertEqual(fake.initialize_calls, 3)
        self.assertTrue(g.is_enabled)

    def test_gives_up_when_never_enabled(self):
        g, fake = make_gripper(err=0, initialize_results=[False, False, False])
        res = g.enable()

        self.assertFalse(res.ok)
        self.assertEqual(res.tries, 3)
        self.assertEqual(res.state.error_code, 0)
        self.assertFalse(g.is_enabled)

    def test_real_fault_is_cleared_first(self):
        g, fake = make_gripper(err=0x9, initialize_results=[False, True])
        res = g.enable()

        self.assertTrue(res.ok)
        self.assertEqual(fake.clear_fault_calls, 1)
        self.assertEqual(res.tries, 2)

    def test_explicit_retries_override(self):
        g, _ = make_gripper(err=0, initialize_results=[False, True])
        res = g.enable(retries=1)
        self.assertFalse(res.ok)
        self.assertEqual(res.tries, 1)


class TestDisableOnDisconnect(unittest.TestCase):
    """退出上下文时是否失能由 disable_on_disconnect 决定。"""

    def test_default_disables(self):
        g, fake = make_gripper()
        g.disconnect()
        self.assertTrue(fake.disconnected)
        self.assertFalse(g.is_enabled)

    def test_can_keep_enabled(self):
        g, fake = make_gripper()
        g.disable_on_disconnect = False
        seen = {}
        fake.disconnect = lambda disable=True: seen.update(disable=disable)
        g.disconnect()
        self.assertEqual(seen, {"disable": False})


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
        res = g.open(SPEED_MM_S)

        target, _, _ = work_limit_target(g.config, 80.0)
        self.assertTrue(res.ok, res)
        self.assertTrue(res.reached, res)
        self.assertFalse(res.stalled, res)
        self.assertAlmostEqual(res.limit_rad, POS_OPEN_RAD)
        self.assertAlmostEqual(fake.motor.pos, target, places=3)
        self.assertGreater(fake.motor.pos, POS_OPEN_RAD)   # 没顶到张开停点

    def test_open_presses_onto_the_stop_without_a_work_stroke(self):
        g, fake = make_gripper(start_rad=POS_CLOSED_RAD, stops=True)
        res = g.open(SPEED_MM_S)
        self.assertTrue(res.stalled, res)
        self.assertAlmostEqual(fake.motor.pos, POS_OPEN_RAD, places=6)

    def test_work_stroke_at_or_over_the_travel_falls_back_to_press(self):
        g, fake = make_gripper(start_rad=POS_CLOSED_RAD, stops=True)
        g.config.work_stroke_mm = TRAVEL_RAD * RAD_TO_MM   # = 满行程 ⇒ 等同没设
        res = g.open(SPEED_MM_S)
        self.assertTrue(res.stalled, res)
        self.assertAlmostEqual(fake.motor.pos, POS_OPEN_RAD, places=6)


class TestStallTorqueProtection(unittest.TestCase):
    """行进段力矩保护（≈7 N）：只在「没进压紧段 + 没跟上」时触发并失力。"""

    def test_mid_travel_block_trips_protection(self):
        g, fake = make_gripper(block_rad=POS_OPEN_RAD + 0.01)
        res = g.close(SPEED_MM_S)

        self.assertTrue(res.protected, res)
        self.assertTrue(res.stalled, res)
        self.assertFalse(res.ok, res)

    def test_protection_releases_the_jaws(self):
        g, fake = make_gripper(block_rad=POS_OPEN_RAD + 0.01)
        res = g.close(SPEED_MM_S)

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
        res = g.close(SPEED_MM_S)

        self.assertFalse(res.protected, res)
        self.assertTrue(res.ok, res)

    def test_cruise_high_torque_but_tracking_does_not_trip(self):
        # 阈值压到极低让巡航力矩也过阈；夹爪还在按指令速度跟进（rate 不慢）
        # ⇒ 不触发。「没跟上」这一条件是防误触发的关键。
        g, fake = make_gripper(stops=True)
        g.motion_config.stop_torque_nm = 0.01
        res = g.close(SPEED_MM_S)

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
        res = g.grasp(force_n=20.0, hold_s=1.2)

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
        return g, fake, g.grasp(force_n=force_n, hold_s=hold_s)

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
        g_close.close(SPEED_MM_S)
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


class TestNonFiniteInputIsRefused(unittest.TestCase):
    """非有限值不许变成运动：拒绝带原因，而且**一帧都不发**。

    这里钉住的是 2026-10-10 审计里实测到的旧行为，三条都在假台上复现过：

    - ``goto(nan)``：限位 clamp 的 ``min`` / ``max`` 与 NaN 的比较全为假，NaN 静默
      变成一个**真实位置**（闭合侧端点），于是夹爪朝那个止点压过去；
    - ``grasp(nan)``：NaN 一路传到帧上，编码器把它抬到量程上端，线上是 +10 Nm；
    - ``set_force(nan)``：爬升循环 ``while probe != target_nm`` 一帧都发不出去地
      永不返回（25 s 里 0 帧）。

    一帧都不发这一条是重点：NaN 不是「大」或「小」的边界，它没有意义，所以拒绝
    必须发生在机构动之前 —— 夹爪先动一下再报错是不行的。
    """

    NON_FINITE = (float("nan"), float("inf"), float("-inf"))

    def test_goto_rejects_a_non_finite_position(self):
        for bad in self.NON_FINITE:
            g, fake = make_gripper(stops=True)
            with self.assertRaises(CommandError):
                g.goto(bad)
            self.assertEqual(fake.frames, [], f"goto({bad!r}) 竟然发帧了")

    def test_goto_rad_rejects_a_non_finite_position(self):
        for bad in self.NON_FINITE:
            g, fake = make_gripper(stops=True)
            with self.assertRaises(CommandError):
                g.goto_rad(bad)
            self.assertEqual(fake.frames, [], f"goto_rad({bad!r}) 竟然发帧了")

    def test_move_at_speed_rejects_a_non_finite_argument(self):
        for field, bad in (("target_mm", float("nan")),
                           ("speed_mm_s", float("inf"))):
            g, fake = make_gripper(stops=True)
            kwargs = dict(target_mm=40.0, speed_mm_s=30.0)
            kwargs[field] = bad
            with self.assertRaises(CommandError):
                g.move_at_speed(**kwargs)
            self.assertEqual(fake.frames, [])

    def test_move_at_speed_rad_rejects_a_non_finite_argument(self):
        for field, bad in (("target_rad", float("nan")),
                           ("speed_rad_s", float("-inf"))):
            g, fake = make_gripper(stops=True)
            kwargs = dict(target_rad=0.0, speed_rad_s=0.5)
            kwargs[field] = bad
            with self.assertRaises(CommandError):
                g.move_at_speed_rad(**kwargs)
            self.assertEqual(fake.frames, [])

    def test_grasp_rejects_a_non_finite_force_before_it_moves(self):
        for bad in self.NON_FINITE:
            g, fake = make_gripper(stops=True)
            with self.assertRaises(CommandError):
                g.grasp(bad, hold_s=0.0)
            self.assertEqual(fake.frames, [], f"grasp({bad!r}) 竟然先动了")

    def test_grasp_rejects_a_non_finite_hold_before_it_moves(self):
        for bad in self.NON_FINITE:
            g, fake = make_gripper(stops=True)
            with self.assertRaises(CommandError):
                g.grasp(20.0, hold_s=bad)
            self.assertEqual(fake.frames, [], f"hold_s={bad!r} 竟然先动了")

    def test_the_motion_config_setpoint_is_checked_too(self):
        """``force_n=None`` 时用的是 ``MotionConfig.force_n`` —— 同一条规则。"""
        g, fake = make_gripper(
            stops=True,
            motion=MotionConfig(force_n=float("nan"), sleep_fn=lambda _: None))
        with self.assertRaises(CommandError):
            g.grasp(hold_s=0.0)
        self.assertEqual(fake.frames, [])

    def test_set_force_rejects_non_finite_force_and_duration(self):
        for field, bad in (("force_n", float("nan")), ("force_n", float("inf")),
                           ("duration", float("nan")),
                           ("duration", float("-inf"))):
            g, fake = make_gripper(stops=True)
            kwargs = dict(force_n=20.0, duration=0.0)
            kwargs[field] = bad
            with self.assertRaises(CommandError):
                g.set_force(**kwargs)
            self.assertEqual(fake.frames, [], f"{field}={bad!r} 竟然发帧了")

    def test_the_force_climb_never_spins_when_a_step_is_absorbed(self):
        """步长小到被浮点吃掉（``value ± step == value``）时也不许空转。

        旧写法 ``while probe != target_nm`` 这时一步都不挪、永不返回。现在每次爬升
        都验「到设定值的距离真的变小了」，不成立就抛错。
        """
        g, fake = make_gripper(
            stops=True,
            motion=MotionConfig(force_ramp_n_s=1e-15, sleep_fn=lambda _: None))
        with self.assertRaises(CommandError) as ctx:
            g.set_force(20.0, duration=0.0)
        self.assertIn("没有朝设定值前进", str(ctx.exception))
        self.assertEqual(fake.frames, [])

    def test_a_normal_climb_still_lands_on_the_setpoint(self):
        """上面那道闸门不许改到正常爬升：照旧一帧一步，正好落在设定值上。"""
        g, fake = make_gripper(stops=True)
        self.assertTrue(g.set_force(20.0, duration=0.0))
        taus = [f.tau_ff for f in fake.frames]
        self.assertTrue(taus, "一帧都没发")
        self.assertEqual(taus[-1], 2.0)
        self.assertAlmostEqual(taus[0], 0.01, places=9)   # 一帧一步


if __name__ == "__main__":
    unittest.main()
