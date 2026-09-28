"""陈旧状态与刷新帧：失能电机不主动发状态帧，读到的可能是缓存/默认值。

``MotorState`` / ``GripperState`` 现在带 ``data_age_s`` 与 ``is_stale``，
``refresh_status()`` 给出「主动要一帧」的路径 —— 失能状态下也能读到位置，
不必为了读位置先使能（使能本身可能让电机朝上一会话遗留的目标运动）。
"""

from __future__ import annotations

import time
import unittest

import _sdkpath  # noqa: F401
from litegrip.can.motor import MotorParams, MotorState
from litegrip.constants import describe_error
from litegrip.models import STALE_AFTER_S, GripperState
from litegrip.protocols.can_bus import LiteGripCAN


def _motor_with_frame(age_s: float = 0.0) -> MotorState:
    """一个「刚收到一帧（或 age_s 秒前收到过一帧）」的 MotorState。"""
    m = MotorState(MotorParams())
    m.update_from_status(position=0.5, velocity=0.0, torque=0.0,
                         error=1, t_mos=30, t_coil=31,
                         timestamp=time.monotonic() - age_s)
    return m


class TestMotorStateFreshness(unittest.TestCase):

    def test_never_received_has_infinite_age_and_no_data(self):
        m = MotorState(MotorParams())
        self.assertFalse(m.has_data)
        self.assertEqual(m.data_age_s, float("inf"))

    def test_a_status_frame_makes_it_fresh_and_have_data(self):
        m = _motor_with_frame()
        self.assertTrue(m.has_data)
        self.assertLess(m.data_age_s, STALE_AFTER_S)

    def test_age_is_measured_from_the_frame(self):
        # 有数据，但已经旧了 —— 这两件事必须能分开判断：失能电机的典型
        # 状态就是「读到过，但此刻不新鲜」。
        m = _motor_with_frame(age_s=5.0)
        self.assertTrue(m.has_data)
        self.assertGreater(m.data_age_s, 4.0)


class TestGripperStateFreshness(unittest.TestCase):

    def test_default_state_has_no_data_and_is_stale(self):
        s = GripperState()
        self.assertFalse(s.has_data)
        self.assertTrue(s.is_stale)

    def test_fresh_state_is_not_stale(self):
        s = GripperState(data_age_s=0.0)
        self.assertTrue(s.has_data)
        self.assertFalse(s.is_stale)

    def test_threshold_is_exclusive(self):
        self.assertFalse(GripperState(data_age_s=STALE_AFTER_S).is_stale)
        self.assertTrue(GripperState(data_age_s=STALE_AFTER_S + 0.001).is_stale)


class TestCommLossIsNamed(unittest.TestCase):
    """0xD 是 DM4310 在 CAN 静默约 900 ms 后 latch 的状态。

    未使能、或刚失能的那段时间就会看到它，所以它不能报成「未知错误」——
    那会把一个正常状态当成硬件故障来排查。
    """

    def test_comm_loss_has_a_description(self):
        self.assertIn("通讯", describe_error(0xD))

    def test_overvoltage_and_overload_have_descriptions(self):
        self.assertIn("过压", describe_error(0x8))
        self.assertIn("过载", describe_error(0xE))

    def test_unknown_codes_still_say_unknown(self):
        self.assertIn("未知", describe_error(0x7F))


class FakeRefreshController:
    """只实现 ``refresh_status`` / ``poll`` 的最小控制器。"""

    def __init__(self, motor: MotorState, answer: bool):
        self.motor = motor
        self.answer = answer
        self.refresh_calls = 0

    def refresh_status(self, motor: MotorState) -> None:
        self.refresh_calls += 1

    def poll(self, timeout_s: float = 0.0):
        if self.answer:
            self.motor.update_from_status(
                position=0.25, velocity=0.0, torque=0.0,
                error=0, t_mos=30, t_coil=31,
                timestamp=time.monotonic())
        return self.motor


def make_can(answer: bool = True):
    can = LiteGripCAN(channel="vcan0")
    motor = MotorState(MotorParams())
    can._motor = motor
    can._controller = FakeRefreshController(motor, answer)
    can._connected = True
    return can, motor


class TestRefreshStatus(unittest.TestCase):

    def test_reads_a_position_while_disabled(self):
        can, motor = make_can()
        self.assertFalse(motor.has_data)          # 还没使能，一帧都没有
        self.assertTrue(can.refresh_status(timeout_s=0.2))
        self.assertTrue(motor.has_data)

    def test_reports_failure_when_the_motor_is_silent(self):
        can, _ = make_can(answer=False)
        self.assertFalse(can.refresh_status(timeout_s=0.1))

    def test_sends_the_refresh_command_and_no_motion(self):
        can, _ = make_can()
        can.refresh_status(timeout_s=0.2)
        self.assertEqual(can._controller.refresh_calls, 1)


if __name__ == "__main__":
    unittest.main()
