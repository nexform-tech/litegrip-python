"""``CanTransport.send`` 的契约：发不出去就报错，不静默丢帧。

这一层是唯一碰真实 socket 的地方，也是「帧到底发没发出去」的最终事实来源。
在它下面没有别的东西兜底：丢掉一帧之后，上层拿到的只是一个 ``None``，而
``None`` 和「发出去了」长得一模一样 —— 2026-10-12 真机上那次
「``goto`` 位移 0.000 mm 却返回 True」就是这么来的。

所以这里钉住三条：未打开 → ``RuntimeError``；classic 帧超 8 字节 →
``ValueError``；发送缓冲一直满超过 ``timeout`` → ``TimeoutError``（``OSError``
子类，正是 docstring 一直承诺的那一类），而不是打条日志就 return。
"""

from __future__ import annotations

import os
import unittest

import _sdkpath  # noqa: F401
from litegrip.can.transport import CanTransport, CanMode

#: 一帧长度合法的 classic CAN 载荷。
PAYLOAD = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC])


class FakeSocket:
    """只实现 ``send`` / ``fileno`` 的 socket 替身。

    ``fileno`` 是真的（一根管道的写端）：``CanTransport.send`` 在两次尝试之间会
    用 ``select.select([], [sock], [], remaining)`` 等缓冲腾出空间，而 select
    只认真文件描述符。管道的写端正常情况下可写，select 会立刻返回。
    """

    def __init__(self, failures: int):
        self.failures = failures
        self.frames = []
        self._rfd, self._wfd = os.pipe()

    def send(self, frame: bytes) -> None:
        if self.failures > 0:
            self.failures -= 1
            raise BlockingIOError(11, "Resource temporarily unavailable")
        self.frames.append(frame)

    def fileno(self) -> int:
        return self._wfd

    def close(self) -> None:
        os.close(self._rfd)
        os.close(self._wfd)


class FakeSocketTest(unittest.TestCase):
    """替身自己的自检：不然下面那些断言可能只是替身坏了。"""

    def tearDown(self):
        if hasattr(self, "sock"):
            self.sock.close()

    def test_the_fake_itself_forwards_after_n_failures(self):
        self.sock = FakeSocket(failures=1)
        with self.assertRaises(BlockingIOError):
            self.sock.send(PAYLOAD)
        self.sock.send(PAYLOAD)
        self.assertEqual(self.sock.frames, [PAYLOAD])


class TestSendContract(unittest.TestCase):

    def test_sending_before_open_is_refused(self):
        t = CanTransport("vcan0")
        with self.assertRaises(RuntimeError):
            t.send(0x108, PAYLOAD)

    def test_a_classic_frame_over_eight_bytes_is_refused(self):
        t = CanTransport("vcan0", mode=CanMode.CAN)
        sock = FakeSocket(failures=0)
        self.addCleanup(sock.close)
        t._sock = sock
        t._is_open = True
        with self.assertRaises(ValueError):
            t.send(0x108, PAYLOAD + b"\x00")

    def test_a_retry_after_a_full_buffer_still_sends(self):
        t = CanTransport("vcan0", mode=CanMode.CAN)
        sock = FakeSocket(failures=1)
        self.addCleanup(sock.close)
        t._sock = sock
        t._is_open = True

        t.send(0x108, PAYLOAD, timeout=0.05)

        self.assertEqual(len(sock.frames), 1, "重试之后那一帧仍然没发出去")

    def test_a_buffer_full_past_the_timeout_raises_instead_of_dropping(self):
        t = CanTransport("vcan0", mode=CanMode.CAN)
        sock = FakeSocket(failures=10_000)
        self.addCleanup(sock.close)
        t._sock = sock
        t._is_open = True

        with self.assertLogs("litegrip.can.transport", level="WARNING") as caught:
            with self.assertRaises(TimeoutError) as ctx:
                t.send(0x108, PAYLOAD, timeout=0.0)

        self.assertEqual(sock.frames, [], "报告了超时却还是把帧发出去了")
        # docstring 承诺的是 OSError；TimeoutError 必须是它的一种。
        self.assertIsInstance(ctx.exception, OSError)
        self.assertIn("0x108", str(ctx.exception))
        self.assertIn("not sent", "\n".join(caught.output))


if __name__ == "__main__":
    unittest.main()
