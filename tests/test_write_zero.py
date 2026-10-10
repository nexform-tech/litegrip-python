"""``LiteGrip.write_zero()`` 的硬件无关测试 —— 守住命令步序与回读语义。

真机行为才是真值来源；这里只用假 CAN 守住不依赖硬件的逻辑：先把夹爪软下来
→ **先失能**（DM 电机在使能态忽略 0xFE）→ 发 0xFE → 重使能 → 回读角度。
假件把 ``disable`` / ``set_zero``（0xFE）/ ``initialize``（重使能那一步）按
发生顺序记进 ``calls``，所以断言的是真实步序，而不是「调用过这些函数」。
"""

from __future__ import annotations

import unittest

import _sdkpath  # noqa: F401
from litegrip import HardwareError, NotInitializedError

from fake_can import make_gripper

# 张开侧附近的一个非零角度 —— 置零前读数就是它。
START_RAD = -0.5


class TestWriteZeroSequence(unittest.TestCase):
    """命令步序：disable 一定在 0xFE 之前、重使能在之后。"""

    def test_disables_before_0xfe_and_re_enables_after(self):
        g, fake = make_gripper(start_rad=START_RAD)
        g.write_zero()

        # initialize() 就是重使能那一步（disable → MIT → enable）。
        self.assertEqual(fake.calls, ["disable", "set_zero", "initialize"])

    def test_jaws_are_left_limp_at_the_angle_they_were_at(self):
        # 整段没有任何一帧带刚度（kp=kd=0），目标就是置零前那一刻的角度 ——
        # 操作员把夹爪放在哪儿，就把哪儿记成零，不把它推走。
        g, fake = make_gripper(start_rad=START_RAD)
        g.write_zero()

        self.assertTrue(all(f.kp == 0.0 and f.kd == 0.0 for f in fake.frames))
        self.assertAlmostEqual(fake.frames[0].q, START_RAD)
        self.assertAlmostEqual(fake.motor.pos, START_RAD)


class TestWriteZeroResult(unittest.TestCase):
    """回读语义：before 是置零前角度，after 是改写零点后的读数。"""

    def test_before_is_the_pre_zero_angle_and_after_is_zero(self):
        g, fake = make_gripper(start_rad=START_RAD)
        res = g.write_zero()

        self.assertAlmostEqual(res.before_rad, START_RAD)
        self.assertAlmostEqual(res.after_rad, 0.0)
        self.assertTrue(res.ok)
        self.assertTrue(res)                    # __bool__ 跟 ok 一致

    def test_the_offset_is_rewritten_not_the_physical_position(self):
        g, fake = make_gripper(start_rad=START_RAD)
        g.write_zero()

        self.assertAlmostEqual(fake.motor.pos, START_RAD)          # 物理位置没动
        self.assertAlmostEqual(fake.motor.zero_offset, START_RAD)  # 只改了偏置

    def test_falsy_when_the_motor_ignores_0xfe(self):
        # 电机忽略了 0xFE（比如它仍在使能态）：帧照收，但读数不变 ——
        # 结果必须说真话，而不是照搬「命令发出去了」。
        g, fake = make_gripper(start_rad=START_RAD)
        fake.set_zero_applies = False
        res = g.write_zero()

        self.assertFalse(res.ok)
        self.assertFalse(res)
        self.assertAlmostEqual(res.before_rad, START_RAD)
        self.assertAlmostEqual(res.after_rad, START_RAD)

    def test_a_nonzero_read_back_still_restores_the_motor(self):
        # 回读不是 ~0 时结果只说「没被接受」，但重使能那一步已经跑了 ——
        # 不能把夹爪丢在失能态。
        g, fake = make_gripper(start_rad=START_RAD)
        fake.set_zero_applies = False
        g.write_zero()

        self.assertTrue(g.is_enabled)


class TestWriteZeroPreconditions(unittest.TestCase):
    """前置条件与「0xFE 发不出去」的分支。"""

    def test_requires_connection(self):
        g, _ = make_gripper()
        g._connected = False
        with self.assertRaises(NotInitializedError):
            g.write_zero()

    def test_requires_enabled(self):
        g, _ = make_gripper()
        g._enabled = False
        with self.assertRaises(NotInitializedError):
            g.write_zero()

    def test_raises_when_the_can_layer_is_gone(self):
        g, _ = make_gripper()
        g._can = None
        with self.assertRaises(HardwareError):
            g.write_zero()

    def test_raises_when_0xfe_cannot_be_sent(self):
        g, fake = make_gripper(start_rad=START_RAD)
        fake.set_zero_ok = False
        with self.assertRaises(HardwareError):
            g.write_zero()

        # 0xFE 没发出去就停在失能态：既没有回读、也没有重使能那一步。
        # 与 ``examples/zero_closed.py`` 的 `zero` 一致 —— 失败时把夹爪
        # 留在失能（手可推动）的安全侧，而不是半途重启。
        self.assertEqual(fake.calls, ["disable", "set_zero"])
        self.assertFalse(g.is_enabled)


if __name__ == "__main__":
    unittest.main()
