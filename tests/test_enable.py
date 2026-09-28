"""``LiteGripCAN.initialize()`` 必须说真话：err == 0 不算使能成功。

用的是真 ``LiteGripCAN``，只把 transport/controller/motor 换成假的 ——
所以这里验的是 can_bus 里的判定逻辑本身，而不是 actions 层的转发。
"""

from __future__ import annotations

import unittest

import _sdkpath  # noqa: F401
from litegrip import HardwareError
from litegrip.can.protocol import ControlModeCode
from litegrip.protocols.can_bus import LiteGripCAN


class FakeMotorState:
    """``MotorState`` 的最小替身（initialize 只碰这三个字段）。"""

    def __init__(self):
        self.rx_count = 0
        self.error = 0
        self.mst_id = 0x18


class FakeController:
    """``MotorController`` 的最小替身：poll 时按脚本回一帧状态。

    真 ``MotorController.poll(timeout_s)`` 不带 motor 参数 —— 它自己更新
    已注册的 MotorState（见 ``can/controller.py:243``）。
    """

    def __init__(self, err_script, motor):
        self.err_script = list(err_script)
        self.motor = motor
        self.calls = []

    def _next_err(self):
        if len(self.err_script) > 1:
            return self.err_script.pop(0)
        return self.err_script[0] if self.err_script else 0

    def disable(self, motor):
        self.calls.append("disable")

    def enable(self, motor):
        self.calls.append("enable")

    def clear_fault(self, motor):
        self.calls.append("clear_fault")

    def switch_control_mode(self, motor, mode) -> bool:
        self.calls.append(("mode", mode))
        return True

    def control_mit(self, motor, kp, kd, q, dq=0.0, tau=0.0):
        self.calls.append("control_mit")

    def poll(self, timeout_s=0.0):
        self.motor.rx_count += 1
        self.motor.error = self._next_err()
        return self.motor

    def close(self):
        self.calls.append("close")


def make_can(err_script):
    can = LiteGripCAN(channel="vcan0")
    motor = FakeMotorState()
    can._motor = motor
    can._controller = FakeController(err_script, motor)
    can._connected = True
    return can, motor


class TestInitializeRequiresErrOne(unittest.TestCase):

    def test_err_one_is_success(self):
        can, _ = make_can([1])
        self.assertTrue(can.initialize())
        self.assertTrue(can.is_initialized)

    def test_err_zero_raises_instead_of_lying(self):
        can, _ = make_can([0])
        with self.assertRaises(HardwareError) as ctx:
            can.initialize()
        self.assertEqual(ctx.exception.error_code, 0)
        self.assertFalse(can.is_initialized)

    def test_err_zero_is_retried(self):
        # 前两次丢 enable 帧（err=0），第三次才真使能
        can, _ = make_can([0, 0, 1])
        self.assertTrue(can.initialize())
        self.assertEqual(can._controller.calls.count("enable") >= 3, True)

    def test_uv_fault_names_the_power_supply(self):
        can, _ = make_can([0x9])
        with self.assertRaises(HardwareError) as ctx:
            can.initialize()
        self.assertEqual(ctx.exception.error_code, 0x9)
        self.assertIn("24V", str(ctx.exception))

    def test_other_fault_is_reported(self):
        can, _ = make_can([0x5])
        with self.assertRaises(HardwareError) as ctx:
            can.initialize()
        self.assertEqual(ctx.exception.error_code, 0x5)

    def test_mit_mode_is_selected(self):
        can, _ = make_can([1])
        can.initialize()
        self.assertIn(("mode", ControlModeCode.MIT), can._controller.calls)


class TestDisconnectFlag(unittest.TestCase):

    def _initialized_can(self):
        can, _ = make_can([1])
        can.initialize()
        ctrl = can._controller
        ctrl.calls.clear()
        return can, ctrl               # disconnect() 会把 _controller 置 None

    def test_disconnect_disables_by_default(self):
        can, ctrl = self._initialized_can()
        can.disconnect()
        self.assertIn("disable", ctrl.calls)
        self.assertIn("close", ctrl.calls)

    def test_disconnect_can_leave_motor_enabled(self):
        can, ctrl = self._initialized_can()
        can.disconnect(disable=False)
        self.assertNotIn("disable", ctrl.calls)
        self.assertIn("close", ctrl.calls)


if __name__ == "__main__":
    unittest.main()
