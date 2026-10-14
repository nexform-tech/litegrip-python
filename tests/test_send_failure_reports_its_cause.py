"""一次失败的下发，原因要跟着异常走到调用方，而不是只留在日志里。

旧契约下 ``_emit`` 把 ``send_mit_frame`` 的 ``False`` 变成一句写死的话：
``CommandError("MIT 帧下发失败（未连接或未使能）")``。这句话在真机上说过谎 —— 操作
者看到它时 ``is_connected`` 和 ``is_enabled`` 都是 ``True``，而真正的原因
（``OSError(100, 'Network is down')``，CAN 接口中途掉了）只作为一条
``logging.warning`` 存在：没挂 handler 的调用方根本看不见。三种完全不同的失败
（总线写不动 / 没连 / 没使能）也都报成同一个 ``CommandError``，而 README 的错误表
里它们分别属于 ``CommError`` 和 ``NotInitializedError``。

这个文件钉住新契约，三段分开验：

- ``LiteGripCAN.control_mit`` 把原因留在 ``last_send_error``（发成功就清空）；
- ``LiteGrip.send_mit_frame`` 原样带上去，不连/没使能时自己给出那一条；
- ``GripperActions._emit`` 按错误表抛 ``NotInitializedError`` / ``CommError``，
  并且**下层没给原因时不替它猜一个**。

CAN 层用真 ``LiteGripCAN`` + 假 controller（与 ``test_can_stream_silence.py``
同一套注入方式），上面两层用 ``fake_can.make_gripper``。
"""

from __future__ import annotations

import unittest

import _sdkpath  # noqa: F401
from fake_can import make_gripper
from litegrip import CommError, NotInitializedError
from litegrip.protocols.can_bus import LiteGripCAN

#: 一帧完全合法的指令。
GOOD = dict(q_target=0.5, kp=100.0, kd=2.0, dq_target=0.0,
            tau_feedforward=0.0)


class FakeMotor:
    """鸭子类型即可 —— 缺席 ``limits`` 时帧量程退回 DM4310（±10 Nm）。"""


class NetworkDownController:
    """下发一律抛 ``ENETDOWN`` —— 真机上接口中途掉线就是这个。"""

    def __init__(self):
        self.attempts = 0

    def control_mit(self, motor, kp, kd, q, dq=0.0, tau=0.0):
        self.attempts += 1
        raise OSError(100, "Network is down")

    def poll(self, timeout_s=0.0):
        return None


class WorkingController:
    def control_mit(self, motor, kp, kd, q, dq=0.0, tau=0.0):
        pass

    def poll(self, timeout_s=0.0):
        return None


def make_can(controller=None):
    can = LiteGripCAN(channel="vcan0")
    if controller is not None:
        can._motor = FakeMotor()
        can._controller = controller
    can._connected = True
    return can


class TestControlMitKeepsTheReason(unittest.TestCase):

    def test_a_transport_failure_reaches_last_send_error(self):
        can = make_can(NetworkDownController())

        with self.assertLogs("litegrip.can_bus", level="WARNING"):
            ok = can.control_mit(**GOOD)

        self.assertFalse(ok)
        error = can.last_send_error
        self.assertIsInstance(error, OSError,
                              "原因只进了日志，没上异常 —— 调用方仍然只能读到"
                              "一个 False")
        self.assertEqual(error.errno, 100)
        self.assertIn("Network is down", str(error))

    def test_a_good_frame_clears_the_previous_reason(self):
        can = make_can(NetworkDownController())
        can.control_mit(**GOOD)
        self.assertIsNotNone(can.last_send_error)

        can._controller = WorkingController()
        self.assertTrue(can.control_mit(**GOOD))
        self.assertIsNone(can.last_send_error,
                          "上一次失败的原因留了下来 —— 下一次成功的下发会被"
                          "报成失败")

    def test_an_unregistered_can_says_it_is_not_registered(self):
        can = make_can()          # 既没 motor 也没 controller

        self.assertFalse(can.control_mit(**GOOD))
        self.assertIsInstance(can.last_send_error, NotInitializedError)
        self.assertIn("未注册", str(can.last_send_error))

    def test_nothing_is_recorded_before_the_first_send(self):
        self.assertIsNone(make_can(WorkingController()).last_send_error)

    def test_a_stream_that_sends_nothing_keeps_the_reason(self):
        can = make_can(NetworkDownController())

        with self.assertRaises(CommError):
            can.control_mit_stream(duration_s=0.01, interval_s=0.001, **GOOD)

        self.assertIsInstance(can.last_send_error, OSError)


class TestSendMitFrameCarriesTheReason(unittest.TestCase):

    def test_the_bus_reason_is_passed_through(self):
        g, fake = make_gripper()
        fake.send_error = OSError(100, "Network is down")

        self.assertFalse(g.send_mit_frame(0.5, 100.0, 2.0))
        self.assertIs(g.last_send_error, fake.send_error)

    def test_a_disabled_gripper_names_the_enable(self):
        g, _ = make_gripper()
        g._enabled = False

        self.assertFalse(g.send_mit_frame(0.5, 100.0, 2.0))
        self.assertIsInstance(g.last_send_error, NotInitializedError)
        self.assertIn("未使能", str(g.last_send_error))

    def test_an_unconnected_gripper_names_the_connect(self):
        g, _ = make_gripper()
        g._can = None

        self.assertFalse(g.send_mit_frame(0.5, 100.0, 2.0))
        self.assertIsInstance(g.last_send_error, NotInitializedError)
        self.assertIn("未连接", str(g.last_send_error))

    def test_a_good_frame_clears_the_previous_reason(self):
        g, fake = make_gripper()
        fake.send_error = OSError(100, "Network is down")
        g.send_mit_frame(0.5, 100.0, 2.0)

        fake.send_error = None
        self.assertTrue(g.send_mit_frame(0.5, 100.0, 2.0))
        self.assertIsNone(g.last_send_error)


class TestTheEngineRaisesTheTypeTheErrorTableNames(unittest.TestCase):
    """``_emit`` 是引擎每一帧的必经之路 —— 它报什么，操作者就看到什么。"""

    def test_a_transport_failure_during_a_move_is_a_commerror(self):
        """issue #65 的复现：连着、使能着，总线掉了。"""
        g, fake = make_gripper()
        fake.send_error = OSError(100, "Network is down")

        with self.assertRaises(CommError) as ctx:
            g.close(20.0)

        message = str(ctx.exception)
        self.assertIn("Network is down", message,
                      "真正的原因没跟着异常出来 —— 操作者只能看到一句"
                      "「未连接或未使能」")
        self.assertNotIn(
            "未连接或未使能", message,
            "夹爪连着也使能着（is_connected/is_enabled 都是 True），这句话是"
            "谎报，会把诊断指到 24V 供电和 enable() 上去")
        self.assertIsInstance(ctx.exception.__cause__, OSError)

    def test_a_disabled_gripper_is_a_notinitializederror(self):
        g, _ = make_gripper()
        g._enabled = False

        # 走 _emit 本身：公开入口（close / goto_rad）在发帧之前就 _check_enabled()
        # 了，用例会撞在那道检查上，测不到这一层对 False 的处理。
        with self.assertRaises(NotInitializedError) as ctx:
            g.actions._emit(0.5)

        self.assertIn("未使能", str(ctx.exception))

    def test_an_unconnected_gripper_is_a_notinitializederror(self):
        g, _ = make_gripper()
        g._can = None

        with self.assertRaises(NotInitializedError) as ctx:
            g.actions._emit(0.5)

        self.assertIn("未连接", str(ctx.exception))

    def test_an_unrecorded_reason_is_reported_as_unrecorded(self):
        """替身只回 ``False`` —— 那就照实说，不替它编一个原因。"""
        g, _ = make_gripper()
        g.send_mit_frame = lambda *args, **kwargs: False

        with self.assertRaises(CommError) as ctx:
            g.actions._emit(0.5)

        message = str(ctx.exception)
        self.assertIn("原因未记录", message)
        self.assertNotIn("未连接或未使能", message)


if __name__ == "__main__":
    unittest.main()
