"""``LiteGripCAN`` 的下发边界：帧承载不了的指令在这里被拒。

用真 ``LiteGripCAN``，只把 controller / motor 换成假的 —— 引擎那批测试底下装的
是 ``FakeLiteGripCAN``，它**绕过**这一层，所以「非有限值、超量程力矩」这一类判定
必须在这里单独验。这一层是所有下发的必经之路：引擎的每一帧、``goto_rad`` 的整段
流、``set_force`` 的爬升，最后都落在 ``control_mit`` / ``control_mit_stream``。

判据是两条：每个字段必须是有限数；力矩不得超出本型号的帧量程。位置、速度、增益的
**超量程**仍由编码器钳位（那是既定语义，见 ``test_protocol.py``），边界不动它们 ——
所以这里也钉住「边界不钳位」这一点。
"""

from __future__ import annotations

import unittest

import _sdkpath  # noqa: F401
from litegrip import CommandError
from litegrip.can.protocol import MotorLimits
from litegrip.protocols.can_bus import LiteGripCAN

NON_FINITE = (float("nan"), float("inf"), float("-inf"))

#: 一帧完全合法的指令。
GOOD = dict(q_target=0.5, kp=100.0, kd=2.0, dq_target=1.0,
            tau_feedforward=1.0)


class FakeLimitsMotor:
    """只带帧量程的电机替身 —— 边界检查只读 ``limits.tau_max``。"""

    def __init__(self, tau_max: float = 10.0):
        self.limits = MotorLimits(q_max=12.5, dq_max=30.0, tau_max=tau_max)


class RecordingController:
    """记下每一帧原样的参数，不编码、不发送。"""

    def __init__(self):
        self.frames = []

    def control_mit(self, motor, kp, kd, q, dq=0.0, tau=0.0):
        self.frames.append((q, kp, kd, dq, tau))

    def poll(self, timeout_s=0.0):
        return None


def make_can(tau_max: float = 10.0):
    can = LiteGripCAN(channel="vcan0")
    can._motor = FakeLimitsMotor(tau_max)
    can._controller = RecordingController()
    can._connected = True
    return can


class TestControlMitBoundary(unittest.TestCase):

    def test_a_valid_command_is_sent(self):
        can = make_can()
        self.assertTrue(can.control_mit(**GOOD))
        self.assertEqual(can._controller.frames, [(0.5, 100.0, 2.0, 1.0, 1.0)])

    def test_every_field_refuses_non_finite_and_sends_nothing(self):
        for field in GOOD:
            for bad in NON_FINITE:
                can = make_can()
                with self.assertRaises(CommandError):
                    can.control_mit(**dict(GOOD, **{field: bad}))
                self.assertEqual(can._controller.frames, [],
                                 f"{field}={bad} 竟然下去了")

    def test_the_refusal_names_the_field(self):
        can = make_can()
        with self.assertRaises(CommandError) as ctx:
            can.control_mit(**dict(GOOD, kd=float("nan")))
        self.assertIn("kd", str(ctx.exception))

    def test_a_torque_at_the_ceiling_is_sent(self):
        can = make_can()
        self.assertTrue(can.control_mit(q_target=0.0, kp=0.0, kd=0.0,
                                        tau_feedforward=10.0))
        self.assertEqual(len(can._controller.frames), 1)

    def test_a_torque_over_the_ceiling_is_refused_not_clamped(self):
        """超量程的力矩不许静默钳到端值 —— 那是另一个力，不是要的那个。"""
        can = make_can()
        with self.assertRaises(CommandError) as ctx:
            can.control_mit(q_target=0.0, kp=0.0, kd=0.0,
                            tau_feedforward=10.5)
        self.assertEqual(can._controller.frames, [])
        self.assertIn("10.5", str(ctx.exception))

    def test_the_ceiling_follows_the_motor_type(self):
        """量程按型号来：DM4310 是 10 Nm，DM4340 是 28 Nm。"""
        with self.assertRaises(CommandError):
            make_can(tau_max=10.0).control_mit(q_target=0.0, kp=0.0, kd=0.0,
                                               tau_feedforward=20.0)
        self.assertTrue(make_can(tau_max=28.0).control_mit(
            q_target=0.0, kp=0.0, kd=0.0, tau_feedforward=20.0))

    def test_the_refusal_comes_before_the_connection_check(self):
        """没连接时也照样拒绝：那是调用方的编程错误，不该被 False 吞掉。"""
        can = LiteGripCAN(channel="vcan0")
        with self.assertRaises(CommandError):
            can.control_mit(q_target=float("nan"), kp=100.0, kd=2.0)

    def test_a_finite_out_of_range_position_is_left_to_the_codec(self):
        """位置超量程是钳位（编码器的事），这一层不改写它。"""
        can = make_can()
        self.assertTrue(can.control_mit(q_target=99.0, kp=100.0, kd=2.0))
        self.assertEqual(can._controller.frames[0][0], 99.0)


class TestControlMitStreamBoundary(unittest.TestCase):

    def test_a_valid_stream_sends_frames(self):
        can = make_can()
        self.assertTrue(can.control_mit_stream(
            q_target=0.5, kp=100.0, kd=2.0, duration_s=0.02, interval_s=0.005))
        self.assertGreaterEqual(len(can._controller.frames), 1)

    def test_a_bad_field_is_refused_before_the_first_frame(self):
        can = make_can()
        with self.assertRaises(CommandError):
            can.control_mit_stream(q_target=float("nan"), kp=100.0, kd=2.0,
                                   duration_s=0.05, interval_s=0.005)
        self.assertEqual(can._controller.frames, [])

    def test_a_non_finite_duration_is_refused_not_a_silent_success(self):
        """NaN 的时长曾经让整段一帧不发却返回 True —— 调用方以为动了。"""
        for bad in NON_FINITE:
            can = make_can()
            with self.assertRaises(CommandError) as ctx:
                can.control_mit_stream(q_target=0.5, kp=100.0, kd=2.0,
                                       duration_s=bad, interval_s=0.005)
            self.assertEqual(can._controller.frames, [])
            self.assertIn("duration_s", str(ctx.exception))

    def test_a_non_finite_interval_is_refused(self):
        for bad in NON_FINITE:
            can = make_can()
            with self.assertRaises(CommandError) as ctx:
                can.control_mit_stream(q_target=0.5, kp=100.0, kd=2.0,
                                       duration_s=0.05, interval_s=bad)
            self.assertIn("interval_s", str(ctx.exception))

    def test_a_negative_interval_is_refused(self):
        can = make_can()
        with self.assertRaises(CommandError) as ctx:
            can.control_mit_stream(q_target=0.5, kp=100.0, kd=2.0,
                                   duration_s=0.05, interval_s=-0.001)
        self.assertIn("interval_s", str(ctx.exception))

    def test_an_over_range_torque_is_refused(self):
        can = make_can()
        with self.assertRaises(CommandError):
            can.control_mit_stream(q_target=0.0, kp=0.0, kd=0.0,
                                   duration_s=0.01, interval_s=0.005,
                                   tau_feedforward=10.5)
        self.assertEqual(can._controller.frames, [])


if __name__ == "__main__":
    unittest.main()
