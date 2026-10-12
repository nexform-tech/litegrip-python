"""``LiteGripCAN`` 的失败上报：发不出去的帧必须被算进去。

``control_mit`` 返回 ``False`` 的旧契约只区分「没连接」，而发送失败被一个裸
``except Exception: return False`` 吞掉；``control_mit_stream`` 更彻底 ——
它丢弃每一帧的下发结果、无条件 ``return True``。两者合起来让一次**一帧都没
执行**的移动在调用方眼里和走完了没有区别。

真机上那次 ``goto(45.0)`` 位移 +0.000 mm 返回 True、随后 0.5 s 内冲到标定零点
外 2.75 mm，就是从这条链里长出来的。这个文件钉住新契约：失败有日志、有计数、
一帧都没发出去就抛 ``CommError``，而不是报成功。

用真 ``LiteGripCAN``，只把 controller / motor 换成假的（与
``test_bus_limits.py`` 同一套注入方式）。
"""

from __future__ import annotations

import unittest

import _sdkpath  # noqa: F401
from litegrip import CommandError, CommError
from litegrip.protocols.can_bus import LiteGripCAN

#: 一帧完全合法的指令。
GOOD = dict(q_target=0.5, kp=100.0, kd=2.0, dq_target=0.0,
            tau_feedforward=0.0)


class FakeMotor:
    """鸭子类型即可 —— 缺席 ``limits`` 时帧量程退回 DM4310（±10 Nm）。"""


class FailingController:
    """每一次下发都抛 ``OSError`` —— 模拟 ENOBUFS / 总线不可写。"""

    def __init__(self, fail_every: int = 1):
        self.attempts = 0
        self.sent = 0
        self.fail_every = fail_every

    def control_mit(self, motor, kp, kd, q, dq=0.0, tau=0.0):
        self.attempts += 1
        if self.fail_every and self.attempts % self.fail_every == 0:
            raise OSError(105, "No buffer space available")
        self.sent += 1

    def poll(self, timeout_s=0.0):
        return None


def make_can(controller):
    can = LiteGripCAN(channel="vcan0")
    can._motor = FakeMotor()
    can._controller = controller
    can._connected = True
    return can


class TestControlMitReportsTheReason(unittest.TestCase):

    def test_a_failed_single_frame_returns_false_and_says_why(self):
        ctrl = FailingController()
        can = make_can(ctrl)

        with self.assertLogs("litegrip.can_bus", level="WARNING") as caught:
            ok = can.control_mit(**GOOD)

        self.assertFalse(ok)
        log_text = "\n".join(caught.output)
        self.assertIn("MIT 帧下发失败", log_text)
        self.assertIn("No buffer space available", log_text,
                      "失败原因又被吞掉了 —— False 单靠自己分不清「没使能」"
                      "和「缓冲满」")

    def test_a_good_frame_returns_true(self):
        ctrl = FailingController(fail_every=0)
        can = make_can(ctrl)
        self.assertTrue(can.control_mit(**GOOD))
        self.assertEqual(ctrl.sent, 1)


class HalfFailingController(FailingController):
    """第一帧发得出去，之后每一帧都失败 —— 既不是全成功，也不是一帧不发。"""

    def control_mit(self, motor, kp, kd, q, dq=0.0, tau=0.0):
        self.attempts += 1
        if self.attempts == 1:
            self.sent += 1
            return
        raise OSError(105, "No buffer space available")


class TestControlMitStreamIsNotSilentlySuccessful(unittest.TestCase):

    def test_a_stream_that_sends_nothing_raises(self):
        ctrl = FailingController()
        can = make_can(ctrl)

        with self.assertRaises(CommError) as ctx:
            can.control_mit_stream(duration_s=0.01, interval_s=0.001, **GOOD)

        self.assertEqual(ctrl.sent, 0)
        self.assertIn("一帧都没发出去", str(ctx.exception))

    def test_a_partly_failed_stream_reports_false_and_counts(self):
        ctrl = HalfFailingController()
        can = make_can(ctrl)

        with self.assertLogs("litegrip.can_bus", level="WARNING") as caught:
            ok = can.control_mit_stream(
                duration_s=0.02, interval_s=0.001, **GOOD)

        self.assertFalse(ok, "有帧没发出去，却报了成功")
        self.assertEqual(ctrl.sent, 1, "本该只有第一帧发出去")
        self.assertIn("没发出去", "\n".join(caught.output))

    def test_a_fully_good_stream_returns_true(self):
        ctrl = FailingController(fail_every=0)
        can = make_can(ctrl)
        self.assertTrue(can.control_mit_stream(
            duration_s=0.01, interval_s=0.001, **GOOD))
        self.assertGreater(ctrl.sent, 0)

    def test_a_zero_duration_is_refused_rather_than_reported_as_success(self):
        ctrl = FailingController(fail_every=0)
        can = make_can(ctrl)

        for bad in (0.0, -0.1):
            with self.assertRaises(CommandError) as ctx:
                can.control_mit_stream(duration_s=bad, **GOOD)
            self.assertIn("必须为正", str(ctx.exception))

        self.assertEqual(ctrl.attempts, 0, "拒绝了却还是发帧了")


if __name__ == "__main__":
    unittest.main()
