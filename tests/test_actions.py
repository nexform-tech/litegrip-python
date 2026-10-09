"""actions 引擎的硬件无关测试：斜坡、领先封顶、堵转窗口、保力、使能。"""

from __future__ import annotations

import unittest

import _sdkpath  # noqa: F401
from litegrip import (CommandError, GraspResult, MotionConfig, limit_target,
                      press_target, work_limit_target)
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
        res = g.grasp(force_n=20.0, hold_s=0.4)

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
        res = g.grasp(force_n=20.0, hold_s=0.4)

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
        g.grasp(force_n=20.0, hold_s=0.4)

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
    """

    #: 工件：行程中段的硬挡块（不移位，专注看力矩的形状）
    OBJECT_RAD = POS_OPEN_RAD + 0.5 * (POS_CLOSED_RAD - POS_OPEN_RAD)
    #: kp=100 × 0.74/74.19 ≈ 1.0 Nm ≈ 10 N 的压紧力
    PRESS_LEAD_MM = 0.74
    #: force_ramp_n_s × frame_interval × 0.1 Nm/N = 20 × 0.005 × 0.1
    STEP_NM = 20.0 * DT * 0.1
    FRAMES_PER_SLICE = 40                       # hold_interval 0.2 ÷ frame_interval 0.005

    def _motion(self, **kw):
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
        # 默认行进上限（4 mm）下压紧力矩 ≈ 5.4 Nm，远超 20 N 的设定值：没有可爬
        # 的，直接从设定值起步 —— 往下走到设定值不是冲击。
        g, fake = make_gripper(block_rad=self.OBJECT_RAD)
        g.motion_config = MotionConfig(sleep_fn=lambda _: None,
                                       monotonic_fn=tick_clock(0.1))
        g.grasp(force_n=20.0, hold_s=0.4)

        move = [f for f in fake.frames if f.kp != 0.0]
        hold = [f for f in fake.frames if f.kp == 0.0]
        self.assertGreater(move[-1].tau_nm, 2.0)
        self.assertAlmostEqual(hold[0].tau_ff, 2.0, places=9)


class TestSetForceRampsToo(unittest.TestCase):
    """``set_force`` 是另一条下发恒定前馈力矩的保力路径，同样按时率爬。"""

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
        res = g.grasp(force_n=20.0, hold_s=0.4)

        self.assertTrue(res.stalled, res)     # 撞上工件（位置式堵转）
        self.assertTrue(res.ok, res)          # 保力正常，没被保护打断
        self.assertTrue([f for f in fake.frames if f.tau_ff == 2.0])  # 有保力帧


if __name__ == "__main__":
    unittest.main()
