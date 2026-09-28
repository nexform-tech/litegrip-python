"""actions 引擎的硬件无关测试：斜坡、领先封顶、堵转窗口、保力、使能。"""

from __future__ import annotations

import unittest

import _sdkpath  # noqa: F401
from litegrip import CommandError, GraspResult, MotionConfig, limit_target
from litegrip.actions import GripperActions

from fake_can import (DT, POS_CLOSED_RAD, POS_OPEN_RAD, RAD_TO_MM,
                      make_gripper, planned_steps, tick_clock)

SPEED_MM_S = 50.0
SPEED_RAD_S = SPEED_MM_S / RAD_TO_MM          # ≈ 0.674 rad/s


class TestLimitTarget(unittest.TestCase):
    """1 / 2：目标位置与位置约定校验。"""

    def test_margin_on_both_ends(self):
        g, _ = make_gripper()
        travel = abs(POS_CLOSED_RAD - POS_OPEN_RAD)

        target, limit, margin_rad, got_travel = limit_target(
            g.config, "close", 0.05)
        self.assertAlmostEqual(target, POS_CLOSED_RAD - 0.05 * travel)
        self.assertAlmostEqual(limit, POS_CLOSED_RAD)
        self.assertAlmostEqual(margin_rad, 0.05 * travel)
        self.assertAlmostEqual(got_travel, travel)

        target, limit, _, _ = limit_target(g.config, "open", 0.05)
        self.assertAlmostEqual(target, POS_OPEN_RAD + 0.05 * travel)
        self.assertAlmostEqual(limit, POS_OPEN_RAD)

    def test_inverted_convention_raises(self):
        g, _ = make_gripper()
        g.config.pos_open_rad, g.config.pos_closed_rad = (
            POS_CLOSED_RAD, POS_OPEN_RAD)          # 反过来
        with self.assertRaises(CommandError):
            limit_target(g.config, "close", 0.05)


class TestRamp(unittest.TestCase):
    """3 / 4 / 5：空载斜坡的形状。"""

    def test_close_reaches_target(self):
        g, fake = make_gripper()
        self.assertIsInstance(g.actions, GripperActions)
        res = g.close(SPEED_MM_S)
        self.assertTrue(res.reached, res)
        self.assertFalse(res.stalled, res)
        self.assertLess(abs(fake.motor.pos - res.target_rad), 0.02)

    def test_frame_count_matches_ramp_plus_settle(self):
        g, fake = make_gripper()
        target, ramp_steps, settle_steps, total = planned_steps(g, SPEED_MM_S)
        res = g.close(SPEED_MM_S)
        self.assertEqual(res.steps, total)
        self.assertEqual(len(fake.frames), total)
        self.assertGreater(settle_steps, 1)          # 保压段确实存在

    def test_velocity_feedforward_only_during_ramp(self):
        g, fake = make_gripper()
        _, ramp_steps, _, total = planned_steps(g, SPEED_MM_S)
        g.close(SPEED_MM_S)

        self.assertAlmostEqual(fake.frames[0].dq, SPEED_RAD_S)   # 闭合 = +rad
        self.assertAlmostEqual(fake.frames[ramp_steps - 1].dq, SPEED_RAD_S)
        for f in fake.frames[ramp_steps:]:
            self.assertEqual(f.dq, 0.0)
        self.assertEqual(len(fake.frames), total)

    def test_open_runs_the_other_way(self):
        g, fake = make_gripper(start_rad=POS_CLOSED_RAD)
        res = g.open(SPEED_MM_S)
        self.assertTrue(res.reached, res)
        self.assertAlmostEqual(res.target_rad,
                               POS_OPEN_RAD + 0.05 * abs(POS_CLOSED_RAD - POS_OPEN_RAD))
        self.assertAlmostEqual(fake.frames[0].dq, -SPEED_RAD_S)  # 张开 = -rad


class TestLeadCapAndStall(unittest.TestCase):
    """6 / 7：被挡住时指令有界、力矩有界，并判到堵转。"""

    def test_lead_cap_bounds_command_and_torque(self):
        # 挡在行程刚起步处（窗口很快填满），目标还在远处
        block = POS_OPEN_RAD + 0.01
        g, fake = make_gripper(block_rad=block)
        res = g.close(SPEED_MM_S)

        cfg = g.motion_config
        lead_cap_rad = max(cfg.max_lead_mm / RAD_TO_MM, SPEED_RAD_S * DT)
        leads = [abs(f.q - f.pos_after) for f in fake.frames]
        self.assertLessEqual(max(leads), lead_cap_rad + 1e-9)
        self.assertAlmostEqual(max(leads), lead_cap_rad, places=6)

        # 被挡住后力矩正好是 kp × 领先上限，且远小于协议上限 10 Nm
        max_tau = max(abs(f.tau_nm) for f in fake.frames)
        self.assertAlmostEqual(max_tau, g.config.kp * lead_cap_rad, places=4)
        self.assertLess(max_tau, 10.0)

        self.assertTrue(res.stalled, res)
        self.assertFalse(res.reached, res)

    def test_mid_ramp_stall_stops_early(self):
        block = (POS_OPEN_RAD + POS_CLOSED_RAD) / 2
        g, fake = make_gripper(block_rad=block)
        _, _, _, total = planned_steps(g, SPEED_MM_S)
        res = g.close(SPEED_MM_S)

        self.assertTrue(res.stalled, res)
        self.assertLess(res.steps, total)
        self.assertAlmostEqual(fake.motor.pos, block, places=6)


class TestStallFalsePositives(unittest.TestCase):
    """8 / 9：粘滑和保压段都不能误判堵转。"""

    def test_sticky_dead_band_is_not_a_stall(self):
        # 闭合侧实测约 0.0103 rad 一跳的粘滑；窗口净位移仍远大于阈值
        g, _ = make_gripper(sticky_rad=0.0103)
        res = g.close(SPEED_MM_S)
        self.assertFalse(res.stalled, res)
        self.assertTrue(res.reached, res)

    def test_settle_phase_is_not_a_stall(self):
        g, fake = make_gripper()
        _, ramp_steps, settle_steps, total = planned_steps(g, SPEED_MM_S)
        res = g.close(SPEED_MM_S)
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
        target, _, _, _ = planned_steps(g, SPEED_MM_S)
        # 挡在目标前 0.0103 rad（实测闭合侧的机械死区）
        fake.motor.set_block(target - 0.0103)
        g.motion_config = MotionConfig(
            sleep_fn=lambda _: None, reach_tol=reach_tol)
        return g.close(SPEED_MM_S)

    def test_default_tolerance_accepts(self):
        self.assertTrue(self._close_short(0.02).reached)

    def test_tight_tolerance_rejects(self):
        self.assertFalse(self._close_short(0.01).reached)


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


if __name__ == "__main__":
    unittest.main()
