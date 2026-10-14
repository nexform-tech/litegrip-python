"""``get_state()`` 要说清那份读数是不是刚有帧答过。

``get_state(wait=True)`` 等 50 ms，然后**无论有没有等到帧**都把缓存读出来，还盖上
``timestamp=time.time()``。于是空闲总线上的读数——没被驱动，电机根本不开口——看起来
和一次测量一模一样：没有异常、没有告警、时间戳是「现在」。真机上这就是
``g.stop()`` 之后连着十次 ``get_state()`` 都报出一个没人产生的「当前」值。

这个文件钉住新契约：``fresh`` 说这次读到帧没有，``data_age_s`` 说读数是哪一帧的，
``timestamp`` 是那一帧到达的时刻（不是这次调用的时刻），``is_stale()`` 把「多久了」
和阈值比；以及给调用方一条出路 —— ``refresh_status()`` 发 0xCC 主动要一帧，
空闲总线上这是唯一让夹爪开口的办法（C++ 侧有，这边之前没有）。

CAN 层用真 ``LiteGripCAN`` + 假 controller，夹爪层用 ``fake_can.make_gripper``
（``delivers=False`` 就是「总线安静」）。
"""

from __future__ import annotations

import time
import unittest

import _sdkpath  # noqa: F401
from fake_can import make_gripper
from litegrip import NotInitializedError, STALE_AFTER_S, GripperState
from litegrip.protocols.can_bus import LiteGripCAN


class FakeMotor:
    """鸭子类型即可 —— 缺席 ``limits`` 时帧量程退回 DM4310（±10 Nm）。"""


class RecordingController:
    """记下 0xCC 请求，别的什么都不做。"""

    def __init__(self, fail: bool = False):
        self.refreshed = []
        self.fail = fail

    def refresh_status(self, motor):
        if self.fail:
            raise OSError(100, "Network is down")
        self.refreshed.append(motor)


def make_can(controller=None):
    can = LiteGripCAN(channel="vcan0")
    if controller is not None:
        can._motor = FakeMotor()
        can._controller = controller
    can._connected = True
    return can


def age_the_last_frame(fake, seconds: float) -> None:
    """把替身「最近一帧」的两个时间戳一起往后退 *seconds* 秒。"""
    fake.last_update = time.monotonic() - seconds
    fake.last_frame_time = time.time() - seconds


class TestGetStateSaysWhetherAFrameAnswered(unittest.TestCase):

    def test_a_frame_marks_the_snapshot_fresh(self):
        g, _ = make_gripper()

        s = g.get_state()

        self.assertTrue(s.fresh)
        self.assertIsNotNone(s.data_age_s)
        self.assertLess(s.data_age_s, STALE_AFTER_S)
        self.assertFalse(s.is_stale())

    def test_an_idle_bus_returns_the_cache_and_says_so(self):
        g, fake = make_gripper()
        g.get_state()                      # 上一帧
        fake.delivers = False              # 从这儿起没有帧了
        age_the_last_frame(fake, 2.0)

        before = time.time()
        s = g.get_state(wait=True)

        self.assertFalse(s.fresh, "没等到帧，却报成刚读到的")
        self.assertGreater(s.data_age_s, 1.0)
        self.assertTrue(s.is_stale())
        self.assertLess(s.timestamp, before - 1.0,
                        "时间戳盖成了这次调用的时刻 —— 一个没人答过的读数因此"
                        "看起来是「现在」的")

    def test_a_bus_that_never_spoke_has_no_frame_to_date_it_by(self):
        g, fake = make_gripper()
        fake.delivers = False              # 一帧都没有过

        s = g.get_state(wait=True)

        self.assertFalse(s.fresh)
        self.assertIsNone(s.data_age_s, "没有帧就没有 age —— 不是「很老」")
        self.assertTrue(s.is_stale())
        self.assertEqual(s.timestamp, 0.0,
                         "0.0 才表示「没有帧」，盖个当前时刻就是谎报")

    def test_wait_false_reports_whether_the_poll_found_a_frame(self):
        g, fake = make_gripper()

        fake.delivers = False
        self.assertFalse(g.get_state(wait=False).fresh)

        fake.delivers = True
        self.assertTrue(g.get_state(wait=False).fresh)

    def test_the_threshold_is_the_callers_to_pick(self):
        s = GripperState(data_age_s=0.3, fresh=False)

        self.assertFalse(s.is_stale())
        self.assertTrue(s.is_stale(stale_after_s=0.1))
        self.assertEqual(STALE_AFTER_S, 0.5, "和 C++ 侧的阈值对齐")


class TestRefreshStatusElicitsAFrame(unittest.TestCase):

    def test_the_request_reaches_the_motor(self):
        ctrl = RecordingController()
        can = make_can(ctrl)

        self.assertTrue(can.refresh_status())
        self.assertEqual(len(ctrl.refreshed), 1)

    def test_an_unregistered_can_reports_false(self):
        self.assertFalse(make_can().refresh_status())

    def test_a_failed_write_reports_false(self):
        can = make_can(RecordingController(fail=True))

        with self.assertLogs("litegrip.can_bus", level="WARNING"):
            self.assertFalse(can.refresh_status())

    def test_a_disconnected_gripper_raises(self):
        g, _ = make_gripper()
        g._connected = False

        with self.assertRaises(NotInitializedError):
            g.refresh_status()

    def test_the_gripper_delegates(self):
        g, fake = make_gripper()

        self.assertTrue(g.refresh_status())
        self.assertEqual(fake.refresh_calls, 1)

    def test_a_refresh_gives_an_idle_bus_something_to_answer(self):
        """请求 → 再读：空闲总线上那唯一的出路要真的走得通。"""
        g, fake = make_gripper()
        g.get_state()
        fake.delivers = False
        age_the_last_frame(fake, 3.0)
        self.assertTrue(g.get_state(wait=True).is_stale())

        self.assertTrue(g.refresh_status())

        s = g.get_state(wait=True)
        self.assertTrue(s.fresh)
        self.assertFalse(s.is_stale())

    def test_the_request_sends_no_control_frame(self):
        g, fake = make_gripper()

        g.refresh_status()

        self.assertEqual(fake.frames, [], "要一帧状态不等于发一帧控制")


if __name__ == "__main__":
    unittest.main()
